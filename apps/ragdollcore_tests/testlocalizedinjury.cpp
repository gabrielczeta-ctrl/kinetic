#include <gtest/gtest.h>

#include <ragdollcore/anatomy.hpp>
#include <ragdollcore/physicaldefense.hpp>
#include <ragdollcore/weaponcontact.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

namespace
{
    using RagdollCore::AnatomicalRegion;
    using RagdollCore::LocalizedStrikeAggregator;
    using RagdollCore::WeaponContactEvent;

    constexpr std::uint64_t attacker = 41;
    constexpr std::uint64_t target = 73;
    constexpr std::uint64_t swing = 9;
    constexpr std::uint64_t generation = 5;

    WeaponContactEvent event(std::uint64_t id, std::uint64_t step, float speed,
        std::uint64_t targetActor = target, std::uint64_t swingIdentity = swing,
        AnatomicalRegion region = AnatomicalRegion::Chest, std::uint32_t bodyIndex = 3,
        float mass = 0.75f, std::uint64_t targetGeneration = generation, bool active = true)
    {
        const std::optional<std::uint64_t> targetIdentity
            = targetActor == 0 ? std::nullopt : std::optional<std::uint64_t>(targetActor);
        return { id, attacker, targetIdentity, "iron dagger", 101,
            static_cast<std::uint32_t>(200 + bodyIndex), step, swingIdentity, active, JPH::RVec3(1.25, -2.5, 3.75),
            JPH::Vec3(0, 1, 0), speed, std::abs(speed), mass, bodyIndex, region, targetGeneration };
    }

    std::vector<WeaponContactEvent> consumeAll(LocalizedStrikeAggregator& aggregator, std::uint64_t step)
    {
        return aggregator.consume(step);
    }
}

TEST(RagdollCoreLocalizedInjuryTest, regionMappingIsExactAndMirrored)
{
    int mapped = 0;
    const std::array<std::pair<std::string_view, AnatomicalRegion>, 20> expected{ {
        { "head", AnatomicalRegion::Head },
        { "neck", AnatomicalRegion::Neck },
        { "spine2", AnatomicalRegion::Chest },
        { "spine1", AnatomicalRegion::Abdomen },
        { "spine", AnatomicalRegion::Abdomen },
        { "pelvis", AnatomicalRegion::Pelvis },
        { "left_clavicle", AnatomicalRegion::LeftClavicle },
        { "right_clavicle", AnatomicalRegion::RightClavicle },
        { "left_upper_arm", AnatomicalRegion::LeftUpperArm },
        { "right_upper_arm", AnatomicalRegion::RightUpperArm },
        { "left_forearm", AnatomicalRegion::LeftForearm },
        { "right_forearm", AnatomicalRegion::RightForearm },
        { "left_hand", AnatomicalRegion::LeftHand },
        { "right_hand", AnatomicalRegion::RightHand },
        { "left_thigh", AnatomicalRegion::LeftThigh },
        { "right_thigh", AnatomicalRegion::RightThigh },
        { "left_calf", AnatomicalRegion::LeftCalf },
        { "right_calf", AnatomicalRegion::RightCalf },
        { "left_foot", AnatomicalRegion::LeftFoot },
        { "right_foot", AnatomicalRegion::RightFoot },
    } };
    for (const auto& [name, region] : expected)
        if (RagdollCore::regionForBodyName(name) == region)
            ++mapped;
    const bool unknownStaysUnknown = RagdollCore::regionForBodyName("equipped_weapon") == AnatomicalRegion::Unknown
        && RagdollCore::regionForBodyName("") == AnatomicalRegion::Unknown;
    const bool mirrored = RagdollCore::regionForBodyName("left_hand") != RagdollCore::regionForBodyName("right_hand")
        && RagdollCore::regionForBodyName("left_thigh") != RagdollCore::regionForBodyName("right_thigh");
    const bool legsFail = RagdollCore::regionCanKnockDown(AnatomicalRegion::LeftThigh)
        && RagdollCore::regionCanKnockDown(AnatomicalRegion::RightFoot)
        && !RagdollCore::regionCanKnockDown(AnatomicalRegion::Head)
        && !RagdollCore::regionCanKnockDown(AnatomicalRegion::Chest);
    ::testing::Test::RecordProperty("control", "region_mapping");
    ::testing::Test::RecordProperty("mapped_count", mapped);
    ::testing::Test::RecordProperty("unknown_stays_unknown", unknownStaysUnknown ? "1" : "0");
    ::testing::Test::RecordProperty("mirrored", mirrored ? "1" : "0");
    ::testing::Test::RecordProperty("legs_can_fail", legsFail ? "1" : "0");
    EXPECT_EQ(mapped, static_cast<int>(expected.size()));
    EXPECT_TRUE(unknownStaysUnknown);
    EXPECT_TRUE(mirrored);
    EXPECT_TRUE(legsFail);
}

TEST(RagdollCoreLocalizedInjuryTest, oneStrikePerTargetActorPerSwing)
{
    LocalizedStrikeAggregator aggregator;
    EXPECT_TRUE(aggregator.observe(event(1, 3, 40.0f)));
    EXPECT_FALSE(aggregator.observe(event(2, 3, 35.0f)));
    EXPECT_TRUE(aggregator.observe(event(3, 3, 40.0f, 74)));
    const auto strikes = consumeAll(aggregator, 3);
    ::testing::Test::RecordProperty("control", "one_per_target");
    ::testing::Test::RecordProperty("strike_count", static_cast<int>(strikes.size()));
    EXPECT_EQ(strikes.size(), 2u);
    // The swing already committed a strike for this target: later
    // observations are rejected rather than pricing a second hit.
    EXPECT_FALSE(aggregator.observe(event(4, 3, 40.0f)));
    EXPECT_TRUE(aggregator.observe(event(5, 3, 40.0f, target, swing + 1)));
}

TEST(RagdollCoreLocalizedInjuryTest, earliestContactingStepWins)
{
    LocalizedStrikeAggregator aggregator;
    EXPECT_TRUE(aggregator.observe(event(10, 5, 20.0f)));
    EXPECT_TRUE(aggregator.observe(event(11, 4, 12.0f)));
    const auto strikes = consumeAll(aggregator, 5);
    ::testing::Test::RecordProperty("control", "earliest_step");
    ::testing::Test::RecordProperty("selected_contact", strikes.empty() ? -1 : static_cast<int>(strikes.front().contactIdentity()));
    ASSERT_EQ(strikes.size(), 1u);
    EXPECT_EQ(strikes.front().contactIdentity(), 11u);
    // A later observation cannot replace a committed strike for the swing.
    EXPECT_FALSE(aggregator.observe(event(12, 6, 99.0f)));
}

TEST(RagdollCoreLocalizedInjuryTest, restingContactDoesNotConsumeLaterImpactInSameSwing)
{
    LocalizedStrikeAggregator aggregator;
    const auto rest = event(90, 1, 0.5f);
    const auto impact = event(91, 2, 20.0f);
    const auto defense = [](const WeaponContactEvent& contact) {
        return RagdollCore::classifyPhysicalDefense({ contact.valid(), false, false,
            contact.physicsStep(), 0, contact.relativeNormalSpeed(), contact.relativeSpeed(), -1.0f });
    };
    ASSERT_FALSE(defense(rest).emitsOutcome());
    EXPECT_FALSE(aggregator.observe(rest));
    EXPECT_TRUE(aggregator.consume(1).empty());
    ASSERT_TRUE(defense(impact).emitsOutcome());
    EXPECT_TRUE(aggregator.observe(impact));
    const auto strikes = aggregator.consume(2);
    ASSERT_EQ(strikes.size(), 1u);
    EXPECT_EQ(strikes.front().contactIdentity(), impact.contactIdentity());
    EXPECT_FALSE(aggregator.observe(event(92, 3, 40.0f)));
}

TEST(RagdollCoreLocalizedInjuryTest, aSliceStrikesByItsSlidingSpeedWhenEnabled)
{
    // A blade resting in the body, dragged through: 20 u/s along the normal,
    // 500 u/s sliding.
    const WeaponContactEvent slice{ 95, attacker, std::optional<std::uint64_t>(target), "iron claymore", 101, 203, 1,
        swing, true, JPH::RVec3(1.25, -2.5, 3.75), JPH::Vec3(0, 1, 0), 20.0f, 500.0f, 0.75f, 3,
        AnatomicalRegion::Chest, generation };
    LocalizedStrikeAggregator off;
    off.setMinimumApproachSpeed(80.0f);
    EXPECT_FALSE(off.observe(slice));
    LocalizedStrikeAggregator on;
    on.setMinimumApproachSpeed(80.0f);
    on.setSlice(300.0f, 0.6f);
    EXPECT_TRUE(on.observe(slice));
    const auto strikes = on.consume(1);
    ASSERT_EQ(strikes.size(), 1u);
    EXPECT_TRUE(strikes.front().isSlice());
    EXPECT_NEAR(strikes.front().damageSpeed(), 300.0f, 1e-3f);
    // A plain impact is unchanged by the setting.
    LocalizedStrikeAggregator impacts;
    impacts.setMinimumApproachSpeed(80.0f);
    impacts.setSlice(300.0f, 0.6f);
    EXPECT_TRUE(impacts.observe(event(96, 1, 400.0f)));
    const auto hit = impacts.consume(1);
    ASSERT_EQ(hit.size(), 1u);
    EXPECT_FALSE(hit.front().isSlice());
    EXPECT_NEAR(hit.front().damageSpeed(), 400.0f, 1e-3f);
}

TEST(RagdollCoreLocalizedInjuryTest, sameStepLargestImpactWins)
{
    LocalizedStrikeAggregator aggregator;
    EXPECT_TRUE(aggregator.observe(event(20, 3, 12.0f, target, swing, AnatomicalRegion::LeftHand, 9)));
    EXPECT_TRUE(aggregator.observe(event(21, 3, 45.0f, target, swing, AnatomicalRegion::Chest, 3)));
    // Weaker same-step observation cannot displace the pending best.
    EXPECT_FALSE(aggregator.observe(event(22, 3, 30.0f, target, swing, AnatomicalRegion::LeftCalf, 15)));
    const auto strikes = consumeAll(aggregator, 3);
    ::testing::Test::RecordProperty("control", "largest_impact");
    ::testing::Test::RecordProperty("selected_contact", strikes.empty() ? -1 : static_cast<int>(strikes.front().contactIdentity()));
    ASSERT_EQ(strikes.size(), 1u);
    EXPECT_EQ(strikes.front().contactIdentity(), 21u);
    EXPECT_EQ(strikes.front().region(), AnatomicalRegion::Chest);
}

TEST(RagdollCoreLocalizedInjuryTest, aggregationIsPermutationInvariant)
{
    const std::vector<WeaponContactEvent> observations{
        event(31, 7, 15.0f, target, swing, AnatomicalRegion::LeftHand, 9),
        event(32, 7, 44.0f, target, swing, AnatomicalRegion::Head, 5),
        event(33, 7, 28.0f, target, swing, AnatomicalRegion::LeftCalf, 15),
    };
    std::vector<int> order(observations.size());
    std::iota(order.begin(), order.end(), 0);
    std::uint64_t selected = 0;
    bool agree = true;
    do
    {
        LocalizedStrikeAggregator aggregator;
        for (const int index : order)
            aggregator.observe(observations[static_cast<std::size_t>(index)]);
        const auto strikes = consumeAll(aggregator, 7);
        if (strikes.size() != 1u)
        {
            agree = false;
            break;
        }
        if (selected == 0)
            selected = strikes.front().contactIdentity();
        else if (selected != strikes.front().contactIdentity())
            agree = false;
    } while (std::next_permutation(order.begin(), order.end()));
    ::testing::Test::RecordProperty("control", "permutation");
    ::testing::Test::RecordProperty("permutations", 6);
    ::testing::Test::RecordProperty("all_agree", agree ? "1" : "0");
    ::testing::Test::RecordProperty("selected_contact", static_cast<int>(selected));
    EXPECT_TRUE(agree);
    EXPECT_EQ(selected, 32u);
}

TEST(RagdollCoreLocalizedInjuryTest, unknownRegionNeverBeatsSupportedAtEqualImpact)
{
    LocalizedStrikeAggregator aggregator;
    EXPECT_TRUE(aggregator.observe(event(40, 2, 30.0f, target, swing, AnatomicalRegion::Unknown, 19)));
    EXPECT_TRUE(aggregator.observe(event(41, 2, 30.0f, target, swing, AnatomicalRegion::Head, 5)));
    const auto strikes = consumeAll(aggregator, 2);
    ::testing::Test::RecordProperty("control", "unknown_region");
    ::testing::Test::RecordProperty("selected_contact", strikes.empty() ? -1 : static_cast<int>(strikes.front().contactIdentity()));
    ::testing::Test::RecordProperty("selected_region", strikes.empty() ? -1 : static_cast<int>(strikes.front().region()));
    ASSERT_EQ(strikes.size(), 1u);
    EXPECT_EQ(strikes.front().contactIdentity(), 41u);
    EXPECT_NE(strikes.front().region(), AnatomicalRegion::Unknown);
}

TEST(RagdollCoreLocalizedInjuryTest, staleGenerationIsRejected)
{
    LocalizedStrikeAggregator aggregator;
    EXPECT_TRUE(aggregator.observe(event(50, 4, 40.0f, target, swing, AnatomicalRegion::Chest, 3, 0.75f, generation)));
    aggregator.rejectStaleGeneration(target, generation + 1);
    const auto strikes = consumeAll(aggregator, 4);
    ::testing::Test::RecordProperty("control", "stale_generation");
    ::testing::Test::RecordProperty("strike_count", static_cast<int>(strikes.size()));
    EXPECT_TRUE(strikes.empty());
    // The current generation still commits.
    EXPECT_TRUE(aggregator.observe(event(51, 5, 40.0f, target, swing, AnatomicalRegion::Chest, 3, 0.75f, generation + 1)));
    EXPECT_EQ(consumeAll(aggregator, 5).size(), 1u);
}

TEST(RagdollCoreLocalizedInjuryTest, scrapeRestAndCrossActorMissProduceNoStrike)
{
    LocalizedStrikeAggregator aggregator;
    // Below the approach threshold.
    EXPECT_FALSE(aggregator.observe(event(60, 3, 0.1f)));
    // Inactive swing.
    EXPECT_FALSE(aggregator.observe(event(61, 3, 40.0f, target, swing, AnatomicalRegion::Chest, 3, 0.75f, generation, false)));
    // No target actor.
    EXPECT_FALSE(aggregator.observe(event(62, 3, 40.0f, 0)));
    const auto strikes = consumeAll(aggregator, 3);
    ::testing::Test::RecordProperty("control", "scrape_rest_miss");
    ::testing::Test::RecordProperty("strike_count", static_cast<int>(strikes.size()));
    EXPECT_TRUE(strikes.empty());
}

TEST(RagdollCoreLocalizedInjuryTest, generationMatchIsExplicit)
{
    const auto strike = event(70, 3, 40.0f, target, swing, AnatomicalRegion::Chest, 3, 0.75f, 17);
    ::testing::Test::RecordProperty("control", "generation_match");
    ::testing::Test::RecordProperty("matches_17", strike.matchesGeneration(17) ? "1" : "0");
    ::testing::Test::RecordProperty("matches_18", strike.matchesGeneration(18) ? "1" : "0");
    EXPECT_TRUE(strike.matchesGeneration(17));
    EXPECT_FALSE(strike.matchesGeneration(18));
}

TEST(RagdollCoreLocalizedInjuryTest, shieldAndUnknownBodyContactsStaySingleAndUnsupported)
{
    LocalizedStrikeAggregator aggregator;
    // A shield or detached proxy body carries no anatomical mapping: its
    // contact keeps the legacy fallback, stays a single strike, and never
    // becomes a critical region just because it is the first manifold.
    EXPECT_TRUE(aggregator.observe(event(80, 4, 30.0f, target, swing, AnatomicalRegion::Unknown, 99)));
    // A same-step proxy contact with an equal score does not displace the
    // first one; it can never add a second strike or a critical region.
    EXPECT_FALSE(aggregator.observe(event(81, 4, 30.0f, target, swing, AnatomicalRegion::Unknown, 100)));
    const auto strikes = consumeAll(aggregator, 4);
    ::testing::Test::RecordProperty("control", "shield_proxy");
    ::testing::Test::RecordProperty("strike_count", static_cast<int>(strikes.size()));
    ::testing::Test::RecordProperty("region_unknown",
        !strikes.empty() && strikes.front().region() == AnatomicalRegion::Unknown ? "1" : "0");
    ASSERT_EQ(strikes.size(), 1u);
    EXPECT_EQ(strikes.front().region(), AnatomicalRegion::Unknown);
}
