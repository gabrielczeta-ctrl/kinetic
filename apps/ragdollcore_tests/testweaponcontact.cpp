#include <gtest/gtest.h>

#include <ragdollcore/weaponcontact.hpp>
#include <ragdollcore/contact.hpp>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/TransformedShape.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <thread>
#include <type_traits>

namespace
{
    // Speeds are world units/second against a 0.2 kg contact-axis mass, as
    // measured for a blade on a limb: 200 units/s is about 2.9 m/s and carries
    // more than a full impact's energy.
    RagdollCore::WeaponContactEvent physicalEvent(std::uint64_t step, float approach = 200,
        float slide = 0, RagdollCore::AnatomicalRegion region = RagdollCore::AnatomicalRegion::Chest,
        bool held = false, bool fist = false, std::uint64_t generation = 1,
        std::optional<float> impulse = 100.0f, std::string blade = "iron dagger")
    {
        RagdollCore::WoundContactSnapshot wound;
        wound.mKind = fist ? RagdollCore::WoundKind::Bruise : RagdollCore::WoundKind::Slash;
        wound.mRelativeVelocity = JPH::Vec3(slide, approach, 0);
        wound.mTargetLocalTangent = slide < 0 ? -JPH::Vec3::sAxisX() : JPH::Vec3::sAxisX();
        RagdollCore::WeaponContactEvent result{ step, 41, 73, fist ? "" : std::move(blade), 101, 202, step, 0, held,
            JPH::RVec3::sZero(), JPH::Vec3::sAxisY(), approach, std::hypot(approach, slide), 0.2f,
            2, region, generation, wound };
        return impulse ? result.withNormalImpulse(*impulse) : result;
    }

    float impactFraction(float approach, float mass = 0.2f)
    {
        const RagdollCore::ContactWorkProfile profile;
        const float metres = approach / 69.99125109f;
        return std::min(1.0f, 0.5f * mass * metres * metres / profile.mFullImpactEnergy);
    }

    RagdollCore::WeaponContactEvent event(std::uint64_t id, std::string record = "iron warhammer",
        std::uint64_t swing = 9, bool active = true, float speed = 42.5f,
        std::optional<std::uint64_t> targetActor = 73, std::uint32_t targetBody = 202,
        float effectiveMass = 0.75f)
    {
        return { id, 41, targetActor, std::move(record), 101, targetBody, 303, swing, active,
            JPH::RVec3(1.25, -2.5, 3.75), JPH::Vec3(0, 1, 0), speed, std::abs(speed), effectiveMass };
    }
}

TEST(RagdollCoreWeaponStrikeFilterTest, strikeProducesOneEvent)
{
    RagdollCore::WeaponStrikeFilter filter;
    const int count = filter.accept(event(1)) ? 1 : 0;
    ::testing::Test::RecordProperty("control", "strike");
    ::testing::Test::RecordProperty("event_count", count);
    EXPECT_EQ(count, 1);
}

TEST(RagdollCoreWeaponStrikeFilterTest, scrapeProducesNoEvent)
{
    RagdollCore::WeaponStrikeFilter filter;
    const float threshold = filter.minimumApproachSpeed();
    int count = filter.accept(event(1, "iron warhammer", 9, true, threshold - 0.01f)) ? 1 : 0;
    count += filter.accept(event(2, "iron warhammer", 9, true, -12.0f)) ? 1 : 0;
    ::testing::Test::RecordProperty("control", "scrape");
    ::testing::Test::RecordProperty("event_count", count);
    EXPECT_EQ(count, 0);
    EXPECT_TRUE(filter.accept(event(3, "iron warhammer", 9, true, threshold)));
}

TEST(RagdollCoreWeaponStrikeFilterTest, restProducesNoEvent)
{
    RagdollCore::WeaponStrikeFilter filter;
    int count = filter.accept(event(1, "iron warhammer", 0, false, 20.0f)) ? 1 : 0;
    count += filter.accept(event(2, "iron warhammer", 9, false, 20.0f)) ? 1 : 0;
    ::testing::Test::RecordProperty("control", "rest");
    ::testing::Test::RecordProperty("event_count", count);
    EXPECT_EQ(count, 0);
}

TEST(RagdollCoreWeaponStrikeFilterTest, multiManifoldProducesOneEvent)
{
    RagdollCore::WeaponStrikeFilter filter;
    int count = filter.accept(event(1, "iron warhammer", 9, true, 20.0f, 73, 202)) ? 1 : 0;
    count += filter.accept(event(2, "iron warhammer", 9, true, 19.0f, 73, 203)) ? 1 : 0;
    count += filter.accept(event(3, "iron warhammer", 9, true, 18.0f, 73, 202)) ? 1 : 0;
    ::testing::Test::RecordProperty("control", "multi_manifold");
    ::testing::Test::RecordProperty("event_count", count);
    EXPECT_EQ(count, 1);
    EXPECT_TRUE(filter.accept(event(4, "iron warhammer", 9, true, 17.0f, 74, 204)));
}

TEST(RagdollCoreWeaponStrikeFilterTest, rehitProducesOneEventForEachDistinctSwing)
{
    RagdollCore::WeaponStrikeFilter filter;
    int count = filter.accept(event(1, "iron warhammer", 9)) ? 1 : 0;
    count += filter.accept(event(2, "iron warhammer", 9)) ? 1 : 0;
    count += filter.accept(event(3, "iron warhammer", 10)) ? 1 : 0;
    count += filter.accept(event(4, "iron warhammer", 10)) ? 1 : 0;
    ::testing::Test::RecordProperty("control", "rehit");
    ::testing::Test::RecordProperty("event_count", count);
    EXPECT_EQ(count, 2);
}

TEST(RagdollCoreWeaponStrikeFilterTest, missProducesNoEvent)
{
    RagdollCore::WeaponStrikeFilter filter;
    const auto strikes = filter.consume({});
    ::testing::Test::RecordProperty("control", "miss");
    ::testing::Test::RecordProperty("event_count", static_cast<int>(strikes.size()));
    EXPECT_TRUE(strikes.empty());
}

TEST(RagdollCoreWeaponStrikeFilterTest, batchesPreserveAcceptedOrderAndResetLifetime)
{
    RagdollCore::WeaponStrikeFilter filter;
    std::vector<RagdollCore::WeaponContactEvent> observations;
    observations.push_back(event(1, "iron warhammer", 9, false, 20.0f));
    observations.push_back(event(2, "iron warhammer", 9, true, filter.minimumApproachSpeed() * 0.5f));
    observations.push_back(event(3, "iron warhammer", 9, true, 20.0f));
    observations.push_back(event(4, "iron warhammer", 9, true, 20.0f));
    observations.push_back(event(5, "iron warhammer", 10, true, 20.0f));
    const auto strikes = filter.consume(std::move(observations));
    ASSERT_EQ(strikes.size(), 2u);
    EXPECT_EQ(strikes[0].contactIdentity(), 3u);
    EXPECT_EQ(strikes[1].contactIdentity(), 5u);
    filter.reset();
    EXPECT_TRUE(filter.accept(event(6, "iron warhammer", 10)));
}

TEST(RagdollCoreWeaponContactTest, copiesACompleteValueSnapshot)
{
    static_assert(std::is_copy_constructible_v<RagdollCore::WeaponContactEvent>);
    static_assert(std::is_move_constructible_v<RagdollCore::WeaponContactEvent>);
    const auto value = event(7);
    ASSERT_TRUE(value.valid());
    EXPECT_EQ(value.contactIdentity(), 7u);
    EXPECT_EQ(value.attackerActorIdentity(), 41u);
    EXPECT_EQ(value.targetActorIdentity(), 73u);
    EXPECT_EQ(value.weaponRecordId(), "iron warhammer");
    EXPECT_EQ(value.weaponBodyIdentity(), 101u);
    EXPECT_EQ(value.targetBodyIdentity(), 202u);
    EXPECT_EQ(value.physicsStep(), 303u);
    EXPECT_EQ(value.swingIdentity(), 9u);
    EXPECT_TRUE(value.swingActive());
    EXPECT_EQ(value.point(), JPH::RVec3(1.25, -2.5, 3.75));
    EXPECT_EQ(value.normal(), JPH::Vec3(0, 1, 0));
    EXPECT_FLOAT_EQ(value.relativeNormalSpeed(), 42.5f);
    EXPECT_FLOAT_EQ(value.relativeSpeed(), 42.5f);
    EXPECT_FLOAT_EQ(value.effectiveMass(), 0.75f);
}

TEST(RagdollCoreWeaponContactTest, rejectsIncompleteOrNonFiniteSnapshots)
{
    const auto valid = event(1);
    EXPECT_FALSE(RagdollCore::WeaponContactEvent(0, 41, 73, "weapon", 1, 2, 3, 0, false,
        JPH::RVec3::sZero(), JPH::Vec3::sAxisX(), 1, 1, 1).valid());
    EXPECT_FALSE(RagdollCore::WeaponContactEvent(1, 0, 73, "weapon", 1, 2, 3, 0, false,
        JPH::RVec3::sZero(), JPH::Vec3::sAxisX(), 1, 1, 1).valid());
    EXPECT_FALSE(RagdollCore::WeaponContactEvent(1, 41, 41, "weapon", 1, 2, 3, 0, false,
        JPH::RVec3::sZero(), JPH::Vec3::sAxisX(), 1, 1, 1).valid());
    EXPECT_FALSE(RagdollCore::WeaponContactEvent(1, 41, std::nullopt, "weapon",
        std::numeric_limits<std::uint32_t>::max(), 2, 3, 0, false,
        JPH::RVec3::sZero(), JPH::Vec3::sAxisX(), 1, 1, 1).valid());
    EXPECT_FALSE(RagdollCore::WeaponContactEvent(1, 41, std::nullopt, "weapon", 1, 2, 3, 0, false,
        JPH::RVec3::sZero(), JPH::Vec3::sZero(), 1, 1, 1).valid());
    EXPECT_FALSE(RagdollCore::WeaponContactEvent(1, 41, std::nullopt, "weapon", 1, 2, 3, 0, false,
        JPH::RVec3::sZero(), JPH::Vec3::sAxisX(), std::numeric_limits<float>::quiet_NaN(), 1, 1).valid());
    EXPECT_FALSE(RagdollCore::WeaponContactEvent(1, 41, std::nullopt, "weapon", 1, 2, 3, 0, false,
        JPH::RVec3::sZero(), JPH::Vec3::sAxisX(), 1, 1, 0).valid());
    EXPECT_TRUE(valid.valid());
}

TEST(RagdollCoreContactDamageTest, HeldBladeNeedsMotionNotAttackIntent)
{
    RagdollCore::ContactDamageTracker tracker;
    EXPECT_TRUE(tracker.consume({ physicalEvent(1, 0) }, 1, 1.f/60).empty());
    EXPECT_EQ(tracker.consume({ physicalEvent(2) }, 2, 1.f/60).size(), 1u);
    EXPECT_TRUE(tracker.consume({ physicalEvent(3) }, 3, 1.f/60).empty());
    EXPECT_TRUE(tracker.consume({ physicalEvent(4, 0) }, 4, 1.f/60).empty());
}

TEST(RagdollCoreContactDamageTest, SeparateRegionsShareImpactAndDuplicatesDoNotMultiply)
{
    using R = RagdollCore::AnatomicalRegion;
    const auto chest = physicalEvent(1);
    const auto arm = physicalEvent(1, 200, 0, R::LeftForearm);
    RagdollCore::ContactDamageTracker a, b;
    const auto first = a.consume({ chest, arm, chest, arm }, 1, 1.f/60);
    const auto second = b.consume({ arm, chest }, 1, 1.f/60);
    ASSERT_EQ(first.size(), 2u);
    ASSERT_EQ(second.size(), first.size());
    for (unsigned i = 0; i < first.size(); ++i)
    {
        EXPECT_EQ(first[i].region(), second[i].region());
        EXPECT_FLOAT_EQ(first[i].damageFraction(), 0.5f);
    }
    EXPECT_TRUE(a.consume({ chest }, 1, 1.f/60).empty());
}

TEST(RagdollCoreContactDamageTest, RecontactRearmsWithoutNewSwingButBriefChatterDoesNot)
{
    RagdollCore::ContactDamageTracker tracker;
    ASSERT_EQ(tracker.consume({ physicalEvent(1) }, 1, 1.f/60).size(), 1u);
    tracker.consume({}, 2, 1.f/60);
    EXPECT_TRUE(tracker.consume({ physicalEvent(3) }, 3, 1.f/60).empty());
    for (unsigned step = 4; step <= 7; ++step) tracker.consume({}, step, 1.f/60);
    EXPECT_TRUE(tracker.consume({ physicalEvent(8) }, 8, 1.f/60).empty());
    tracker.confirmSeparation({41, 101, 73, 1});
    EXPECT_EQ(tracker.consume({ physicalEvent(9) }, 9, 1.f/60).size(), 1u);
}

TEST(RagdollCoreContactDamageTest, SlidingWorkIsIndependentOfPhysicsRate)
{
    // One second of a steady draw. Work is travel times a speed-dependent
    // depth; only the last unpaid remainder, below one repeat injury, may
    // differ between rates.
    const RagdollCore::ContactWorkProfile profile;
    for (const float speed : { 60.0f, 300.0f, 520.0f })
    for (const unsigned rate : { 30u, 60u, 120u })
    {
        RagdollCore::ContactDamageTracker tracker;
        float damage = 0;
        std::size_t events = 0;
        for (unsigned step = 1; step <= rate; ++step)
            for (const auto& hit : tracker.consume({ physicalEvent(step, 0, speed) }, step, 1.f/rate))
            {
                EXPECT_TRUE(hit.hasMeasuredWork());
                EXPECT_LE(hit.damageFraction(), 1.0f);
                damage += hit.damageFraction();
                ++events;
            }
        const float work = speed / profile.mTravelPerFullCut * std::min(1.0f, speed / profile.mFullCutSpeed);
        EXPECT_LE(damage, work + 0.001f) << rate << " Hz, " << speed << " units/s";
        EXPECT_GE(damage, work - profile.mRepeatInjuryFraction - 0.001f) << rate << " Hz, " << speed << " units/s";
        EXPECT_GT(events, 1u);
    }
}

TEST(RagdollCoreContactDamageTest, FurtherCutsConsumeFurtherWorkAndOneInjuryIsBounded)
{
    const RagdollCore::ContactWorkProfile profile;
    RagdollCore::ContactDamageTracker slow;
    std::vector<float> fractions;
    for (unsigned step = 1; step <= 120; ++step)
        for (const auto& hit : slow.consume({ physicalEvent(step, 0, 60) }, step, 1.f/60))
            fractions.push_back(hit.damageFraction());
    ASSERT_GE(fractions.size(), 2u);
    EXPECT_GE(fractions.front(), profile.mMinimumInjuryFraction);
    EXPECT_LT(fractions.front(), profile.mRepeatInjuryFraction);
    for (std::size_t i = 1; i < fractions.size(); ++i)
        EXPECT_GE(fractions[i], profile.mRepeatInjuryFraction);

    // A very fast edge at a low rate travels more than one full cut per step.
    RagdollCore::ContactDamageTracker fast;
    for (unsigned step = 1; step <= 6; ++step)
        for (const auto& hit : fast.consume({ physicalEvent(step, 0, 520) }, step, 1.f/30))
            EXPECT_LE(hit.damageFraction(), 1.0f);
}

TEST(RagdollCoreContactDamageTest, PressingPoweredBodiesNeverDamage)
{
    // Two driven bodies pushed together: solver residual approach of a few
    // units/second, tangential chatter well above the cutting speed that
    // reverses every step, a slow creep, and a manifold that drops out for
    // longer than the rearm gap and comes back still pressing.
    using R = RagdollCore::AnatomicalRegion;
    for (const unsigned rate : { 30u, 60u, 120u })
    {
        RagdollCore::ContactDamageTracker tracker;
        std::size_t events = 0;
        for (unsigned step = 1; step <= rate * 10; ++step)
        {
            if (step % 40 >= 34)
            {
                events += tracker.consume({}, step, 1.f/rate).size();
                continue;
            }
            const float approach = 2.0f + static_cast<float>(step % 5) * 3.0f;
            const float chatter = (step % 2 ? 50.0f : -50.0f) + 6.0f;
            const R region = (step / 10) % 2 ? R::RightForearm : R::RightHand;
            events += tracker.consume({ physicalEvent(step, approach, chatter, region, step % 7 == 0) },
                step, 1.f/rate).size();
        }
        EXPECT_EQ(events, 0u) << rate << " Hz";
    }
}

TEST(RagdollCoreContactDamageTest, LightTouchIsNoBlowAndWeakBlowsEarnLittle)
{
    const RagdollCore::ContactWorkProfile profile;
    RagdollCore::ContactDamageTracker touch;
    EXPECT_TRUE(touch.consume({ physicalEvent(1, 40) }, 1, 1.f/60).empty());
    // The episode was not spent by the touch: a real blow still lands.
    EXPECT_EQ(touch.consume({ physicalEvent(2, 200) }, 2, 1.f/60).size(), 1u);

    float previous = 0;
    for (const float approach : { 60.0f, 90.0f, 140.0f, 200.0f, 400.0f })
    {
        RagdollCore::ContactDamageTracker tracker;
        const auto hits = tracker.consume({ physicalEvent(1, approach) }, 1, 1.f/60);
        ASSERT_EQ(hits.size(), 1u) << approach;
        EXPECT_TRUE(hits[0].hasMeasuredWork());
        EXPECT_FLOAT_EQ(hits[0].damageFraction(), impactFraction(approach));
        EXPECT_GE(hits[0].damageFraction(), previous);
        previous = hits[0].damageFraction();
    }
    EXPECT_FLOAT_EQ(previous, 1.0f);
    EXPECT_LT(impactFraction(60), 0.2f);
    EXPECT_GE(impactFraction(60) * profile.mFullImpactEnergy, profile.mMinimumImpactEnergy);
}

TEST(RagdollCoreContactDamageTest, MovingTargetSuppliesTheWorkOnAHeldBlade)
{
    // Relative velocity is all the tracker sees: a victim running onto an
    // inactive, stationary dagger, then a steady edge drawn without a swing.
    RagdollCore::ContactDamageTracker tracker;
    const auto impact = tracker.consume({ physicalEvent(1, 150, 0) }, 1, 1.f/60);
    ASSERT_EQ(impact.size(), 1u);
    EXPECT_FALSE(impact[0].swingActive());
    EXPECT_NEAR(impact[0].damageFraction(), impactFraction(150), 1e-5f);
    float cuts = 0;
    for (unsigned step = 2; step <= 31; ++step)
        for (const auto& hit : tracker.consume({ physicalEvent(step, 0, 150) }, step, 1.f/60))
            cuts += hit.damageFraction();
    EXPECT_GT(cuts, 5.0f);
}

TEST(RagdollCoreContactDamageTest, RestJitterAndIdleFistsDoNotDamage)
{
    using R = RagdollCore::AnatomicalRegion;
    RagdollCore::ContactDamageTracker tracker;
    for (unsigned step = 1; step <= 300; ++step)
        EXPECT_TRUE(tracker.consume({ physicalEvent(step, 0.1f, 0.5f) }, step, 1.f/60).empty());
    EXPECT_TRUE(tracker.consume({ physicalEvent(301, 200, 0, R::Chest, false, true) }, 301, 1.f/60).empty());
    EXPECT_EQ(tracker.consume({ physicalEvent(302, 200, 0, R::Chest, true, true) }, 302, 1.f/60).size(), 1u);
}

TEST(RagdollCoreContactDamageTest, NewGenerationDoesNotInheritOldContactBudget)
{
    using R = RagdollCore::AnatomicalRegion;
    RagdollCore::ContactDamageTracker tracker;
    ASSERT_EQ(tracker.consume({ physicalEvent(1) }, 1, 1.f/60).size(), 1u);
    const auto result = tracker.consume({ physicalEvent(2, 200, 0, R::Chest, false, false, 2) }, 2, 1.f/60);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_FALSE(result[0].matchesGeneration(1));
    EXPECT_TRUE(result[0].matchesGeneration(2));
}

TEST(RagdollCoreContactDamageTest, LimbSeamsAndLongCallbackGapsCannotRefreshAnImpact)
{
    using R = RagdollCore::AnatomicalRegion;
    RagdollCore::ContactDamageTracker tracker;
    ASSERT_EQ(tracker.consume({ physicalEvent(1) }, 1, 1.f/60).size(), 1u);
    for (unsigned step = 2; step < 1300; ++step)
    {
        if (step < 1200) EXPECT_TRUE(tracker.consume({}, step, 1.f/60).empty());
        else EXPECT_TRUE(tracker.consume({physicalEvent(step, 200, 0,
            step % 2 ? R::RightHand : R::RightForearm)}, step, 1.f/60).empty());
    }
    ASSERT_EQ(tracker.activePairs().size(), 1u);
    tracker.confirmSeparation({41, 101, 73, 1});
    EXPECT_EQ(tracker.consume({physicalEvent(1300)}, 1300, 1.f/60).size(), 1u);
}

TEST(RagdollCoreContactDamageTest, SolverLoadIsRequiredForCuttingAndBoundsImpact)
{
    using R = RagdollCore::AnatomicalRegion;
    for (const auto impulse : {std::optional<float>{0}, std::optional<float>{}})
    {
        RagdollCore::ContactDamageTracker tracker;
        for (unsigned step = 1; step <= 1200; ++step)
            EXPECT_TRUE(tracker.consume({physicalEvent(step, 0, 150, R::Chest,
                false, false, 1, impulse)}, step, 1.f/60).empty());
    }
    RagdollCore::ContactDamageTracker unloaded, loaded;
    EXPECT_TRUE(unloaded.consume({physicalEvent(1, 200, 0, R::Chest, false, false, 1, 0)}, 1, 1.f/60).empty());
    const auto hit = loaded.consume({physicalEvent(1, 200, 0, R::Chest, false, false, 1, 3)}, 1, 1.f/60);
    ASSERT_EQ(hit.size(), 1u);
    EXPECT_LT(hit.front().damageFraction(), impactFraction(200));
}

TEST(RagdollCoreContactDamageTest, ReversingDrawsNeedFreshWorkAndRestCannotPayADebt)
{
    RagdollCore::ContactDamageTracker tracker;
    float forward = 0, reverse = 0;
    for (unsigned step = 1; step <= 60; ++step)
        for (const auto& hit : tracker.consume({physicalEvent(step, 0, step <= 30 ? 150.f : -150.f)}, step, 1.f/60))
            (step <= 30 ? forward : reverse) += hit.damageFraction();
    EXPECT_GT(forward, 5);
    EXPECT_GT(reverse, 5);
    for (unsigned step = 61; step <= 1260; ++step)
        EXPECT_TRUE(tracker.consume({physicalEvent(step, 0, 0)}, step, 1.f/60).empty());
}

TEST(RagdollCoreContactDamageTest, CuttingSeveralRegionsSharesOneMotionBudget)
{
    using R = RagdollCore::AnatomicalRegion;
    RagdollCore::ContactDamageTracker one, many;
    float single = 0, multiple = 0;
    for (unsigned step = 1; step <= 120; ++step)
    {
        const auto chest = physicalEvent(step, 0, 150);
        const auto arm = physicalEvent(step, 0, 150, R::LeftForearm);
        for (const auto& hit : one.consume({chest}, step, 1.f/60)) single += hit.damageFraction();
        for (const auto& hit : many.consume({arm, chest, arm}, step, 1.f/60)) multiple += hit.damageFraction();
    }
    EXPECT_GT(single, 10);
    EXPECT_FLOAT_EQ(single, multiple);
}

TEST(RagdollCoreContactDamageTest, GeometricClearanceIncludesMarginAndRotatedShapes)
{
    const JPH::RefConst<JPH::Shape> blade = new JPH::BoxShape(JPH::Vec3(1, 1, 10), 0.05f);
    const JPH::RefConst<JPH::Shape> limb = new JPH::SphereShape(3);
    const JPH::RVec3 origin(100000, -200000, 30000);
    const JPH::TransformedShape a(origin, JPH::Quat::sIdentity(), blade, JPH::BodyID(101));
    const auto close = [&](float gap) {
        return RagdollCore::contactShapesWithinDistance(a, JPH::TransformedShape(
            origin + JPH::Vec3(4 + gap, 0, 0), JPH::Quat::sIdentity(), limb, JPH::BodyID(202)), 2);
    };
    EXPECT_TRUE(close(-0.1f));
    EXPECT_TRUE(close(0.5f));
    EXPECT_TRUE(close(1.9f));
    EXPECT_FALSE(close(2.1f));
    const JPH::TransformedShape rotated(origin, JPH::Quat::sRotation(JPH::Vec3::sAxisY(), JPH::JPH_PI / 2), blade, JPH::BodyID(101));
    const JPH::TransformedShape target(origin + JPH::Vec3(14, 0, 0), JPH::Quat::sIdentity(), limb, JPH::BodyID(202));
    EXPECT_TRUE(RagdollCore::contactShapesWithinDistance(rotated, target, 2));
    EXPECT_FALSE(RagdollCore::contactShapesWithinDistance(a, target, 2));
}

TEST(RagdollCoreWeaponContactTest, bareHandUsesEmptyWeaponRecordWithCompleteBodyIdentity)
{
    const auto fist = event(1, "");
    ASSERT_TRUE(fist.valid());
    RagdollCore::WeaponStrikeFilter filter;
    EXPECT_TRUE(filter.accept(fist));
    EXPECT_FALSE(filter.accept(fist));
    EXPECT_FALSE(RagdollCore::WeaponContactEvent(1, 41, 73, "", 1,
        std::numeric_limits<std::uint32_t>::max(), 3, 9, true,
        JPH::RVec3::sZero(), JPH::Vec3::sAxisX(), 1, 1, 1).valid());
}

TEST(RagdollCoreWeaponContactTest, drainsEachPublishedEventExactlyOnce)
{
    RagdollCore::WeaponContactEventQueue queue;
    ASSERT_TRUE(queue.publish(event(1)));
    ASSERT_TRUE(queue.publish(event(2)));
    EXPECT_EQ(queue.pending(), 2u);
    const auto first = queue.consume();
    ASSERT_EQ(first.size(), 2u);
    EXPECT_EQ(first[0].contactIdentity(), 1u);
    EXPECT_EQ(first[1].contactIdentity(), 2u);
    EXPECT_EQ(queue.pending(), 0u);
    EXPECT_TRUE(queue.consume().empty());
}

TEST(RagdollCoreWeaponContactTest, concurrentPhysicsProducersLoseNoEvents)
{
    RagdollCore::WeaponContactEventQueue queue;
    constexpr std::uint64_t threadCount = 8;
    constexpr std::uint64_t perThread = 250;
    std::vector<std::thread> threads;
    for (std::uint64_t thread = 0; thread < threadCount; ++thread)
        threads.emplace_back([&, thread] {
            for (std::uint64_t item = 0; item < perThread; ++item)
                EXPECT_TRUE(queue.publish(event(thread * perThread + item + 1)));
        });
    for (auto& thread : threads)
        thread.join();
    const auto consumed = queue.consume();
    ASSERT_EQ(consumed.size(), threadCount * perThread);
    std::set<std::uint64_t> identities;
    for (const auto& value : consumed)
        identities.insert(value.contactIdentity());
    EXPECT_EQ(identities.size(), threadCount * perThread);
    EXPECT_EQ(*identities.begin(), 1u);
    EXPECT_EQ(*identities.rbegin(), threadCount * perThread);
}

TEST(RagdollCoreWeaponContactTest, snapshotOutlivesCallbackOwnedSourceValues)
{
    RagdollCore::WeaponContactEventQueue queue;
    {
        std::string callbackRecord = "callback-owned weapon";
        ASSERT_TRUE(queue.publish(event(1, callbackRecord)));
        callbackRecord.assign("destroyed");
    }
    const auto consumed = queue.consume();
    ASSERT_EQ(consumed.size(), 1u);
    EXPECT_EQ(consumed.front().weaponRecordId(), "callback-owned weapon");
    EXPECT_EQ(consumed.front().point(), JPH::RVec3(1.25, -2.5, 3.75));
}

TEST(RagdollCoreWeaponContactTest, consumerCanDrainWhilePhysicsProducersContinue)
{
    RagdollCore::WeaponContactEventQueue queue;
    std::atomic<bool> start = false;
    std::thread producer([&] {
        while (!start.load(std::memory_order_acquire))
            std::this_thread::yield();
        for (std::uint64_t id = 1; id <= 1000; ++id)
            ASSERT_TRUE(queue.publish(event(id)));
    });
    start.store(true, std::memory_order_release);
    std::vector<RagdollCore::WeaponContactEvent> all;
    while (producer.joinable() && all.size() < 1000)
    {
        auto batch = queue.consume();
        all.insert(all.end(), std::make_move_iterator(batch.begin()), std::make_move_iterator(batch.end()));
        if (all.size() == 1000)
            break;
        std::this_thread::yield();
    }
    producer.join();
    auto tail = queue.consume();
    all.insert(all.end(), std::make_move_iterator(tail.begin()), std::make_move_iterator(tail.end()));
    ASSERT_EQ(all.size(), 1000u);
    std::sort(all.begin(), all.end(), [](const auto& left, const auto& right) {
        return left.contactIdentity() < right.contactIdentity();
    });
    for (std::size_t i = 0; i < all.size(); ++i)
        EXPECT_EQ(all[i].contactIdentity(), i + 1);
}

TEST(RagdollCoreLongswordTest, HeldImpactRestAndSeparationUseMeasuredWork)
{
    using R = RagdollCore::AnatomicalRegion;
    const auto sword = [](unsigned step, float approach) {
        return physicalEvent(step, approach, 0, R::Chest, false, false, 1, 100, "iron longsword");
    };
    RagdollCore::ContactDamageTracker tracker;
    EXPECT_TRUE(tracker.consume({sword(1, 0)}, 1, 1.f/60).empty());
    const auto first = tracker.consume({sword(2, 150)}, 2, 1.f/60);
    ASSERT_EQ(first.size(), 1u);
    EXPECT_TRUE(first[0].hasMeasuredWork());
    EXPECT_NEAR(first[0].damageFraction(), impactFraction(150), 1e-5f);
    EXPECT_FALSE(first[0].swingActive());
    for (unsigned step = 3; step < 1203; ++step)
        EXPECT_TRUE(tracker.consume({sword(step, 0.5f)}, step, 1.f/60).empty());
    EXPECT_TRUE(tracker.consume({sword(1203, 150)}, 1203, 1.f/60).empty());
    tracker.confirmSeparation({41, 101, 73, 1});
    EXPECT_EQ(tracker.consume({sword(1204, 150)}, 1204, 1.f/60).size(), 1u);
}

TEST(RagdollCoreLongswordTest, LoadedDrawSharesOneBudgetAcrossLimbsAtAllRates)
{
    using R = RagdollCore::AnatomicalRegion;
    for (unsigned rate : {30u, 60u, 120u})
    {
        RagdollCore::ContactDamageTracker single, many, unloaded;
        float a = 0, b = 0;
        unsigned injuries = 0;
        for (unsigned step = 1; step <= rate; ++step)
        {
            auto chest = physicalEvent(step, 0, 150, R::Chest, false, false, 1, 100, "iron longsword");
            auto arm = physicalEvent(step, 0, 150, R::LeftForearm, false, false, 1, 100, "iron longsword");
            for (const auto& hit : single.consume({chest}, step, 1.f/rate))
            {
                a += hit.damageFraction();
                ++injuries;
            }
            for (const auto& hit : many.consume({arm, chest, arm}, step, 1.f/rate)) b += hit.damageFraction();
            EXPECT_TRUE(unloaded.consume({chest.withNormalImpulse(0)}, step, 1.f/rate).empty());
        }
        EXPECT_FLOAT_EQ(a, b);
        EXPECT_GT(injuries, 1u);
        EXPECT_GE(a, 12.0f);
        EXPECT_LE(a, 12.5001f);
    }
}

TEST(RagdollCoreLongswordTest, ContactSnapshotSurvivesRotationAndScale)
{
    using namespace RagdollCore;
    for (float scale : {0.5f, 1.f, 2.f})
    for (JPH::Quat rotation : {JPH::Quat::sIdentity(), JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.2f),
        JPH::Quat::sRotation(JPH::Vec3(1, 2, 3).Normalized(), 2.1f)})
    {
        const JPH::Vec3 center = scale * JPH::Vec3(6.17468f, 0.773655f, 27.2016f);
        const float length = scale * 69.2744f;
        const float tip = center.GetZ() + 0.5f * length;
        const auto classify = [&](float along, JPH::Vec3 n, JPH::Vec3 v) {
            const JPH::Vec3 point(center.GetX(), center.GetY(), along * tip);
            return classifyBladeContactSnapshot("iron longsword", rotation, rotation * (point - center),
                center, length, rotation * n, rotation * v);
        };
        EXPECT_EQ(classify(0.5f, JPH::Vec3::sAxisX(), JPH::Vec3(80, 0, 0)), WoundKind::Slash);
        EXPECT_EQ(classify(0.5f, JPH::Vec3::sAxisY(), JPH::Vec3(0, 80, 0)), WoundKind::Bruise);
        EXPECT_EQ(classify(0.05f, JPH::Vec3::sAxisX(), JPH::Vec3(80, 0, 0)), WoundKind::Bruise);
        EXPECT_EQ(classify(0.98f, JPH::Vec3::sAxisZ(), JPH::Vec3(0, 0, 80)), WoundKind::Puncture);
    }
}
