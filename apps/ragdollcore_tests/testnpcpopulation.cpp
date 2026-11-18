#include <ragdollcore/npcpopulation.hpp>
#include <ragdollcore/switchregistry.hpp>
#include <ragdollcore/tuning.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>

namespace
{
    using namespace RagdollCore;

    NpcPopulationCandidate npc(std::size_t id, float distance, bool powered, bool eligible = true)
    {
        NpcPopulationCandidate candidate;
        candidate.mId = id;
        candidate.mDistance = distance;
        candidate.mPowered = powered;
        candidate.mEligible = eligible;
        return candidate;
    }

    bool contains(const std::vector<std::size_t>& list, std::size_t id)
    {
        return std::find(list.begin(), list.end(), id) != list.end();
    }

    NpcPopulationLimits limits(std::size_t maximum, float radius = 2000.0f)
    {
        return { maximum, radius, radius * sReleaseFactor };
    }

    TEST(NpcPopulationTest, AcquiresNearestFirstUpToTheCount)
    {
        const std::vector<NpcPopulationCandidate> candidates{ npc(0, 1500.0f, false), npc(1, 300.0f, false),
            npc(2, 2500.0f, false), npc(3, 900.0f, false) };
        const NpcPopulationPlan plan = planNpcPopulation(candidates, limits(2));
        EXPECT_TRUE(plan.mRelease.empty());
        EXPECT_EQ(plan.mFreeSlots, 2u);
        // Beyond the radius (2) never listed; the rest nearest first.
        EXPECT_EQ(plan.mAcquire, (std::vector<std::size_t>{ 1, 3, 0 }));
    }

    TEST(NpcPopulationTest, ReleasesTheDeadAndThoseBeyondTheReleaseRadius)
    {
        const std::vector<NpcPopulationCandidate> candidates{ npc(0, 3000.0f, true), npc(1, 3600.0f, true),
            npc(2, 100.0f, true, false) };
        const NpcPopulationPlan plan = planNpcPopulation(candidates, limits(10));
        // 3000 is past the acquire radius but inside the release radius: kept.
        EXPECT_FALSE(contains(plan.mRelease, 0));
        EXPECT_TRUE(contains(plan.mRelease, 1));
        EXPECT_TRUE(contains(plan.mRelease, 2));
        EXPECT_EQ(plan.mFreeSlots, 9u);
    }

    TEST(NpcPopulationTest, LoweredCountReleasesTheFarthest)
    {
        const std::vector<NpcPopulationCandidate> candidates{ npc(0, 500.0f, true), npc(1, 1800.0f, true),
            npc(2, 1200.0f, true), npc(3, 200.0f, true) };
        const NpcPopulationPlan plan = planNpcPopulation(candidates, limits(2));
        EXPECT_EQ(plan.mRelease.size(), 2u);
        EXPECT_TRUE(contains(plan.mRelease, 1));
        EXPECT_TRUE(contains(plan.mRelease, 2));
        EXPECT_EQ(plan.mFreeSlots, 0u);
        EXPECT_TRUE(plan.mAcquire.empty());
    }

    TEST(NpcPopulationTest, CountZeroReleasesEveryNpcBody)
    {
        const std::vector<NpcPopulationCandidate> candidates{ npc(0, 500.0f, true), npc(1, 100.0f, false) };
        const NpcPopulationPlan plan = planNpcPopulation(candidates, limits(0));
        EXPECT_EQ(plan.mRelease, (std::vector<std::size_t>{ 0 }));
        EXPECT_TRUE(plan.mAcquire.empty());
    }

    TEST(NpcPopulationTest, FullPopulationSwapsOnlyForAClearlyNearerNewcomer)
    {
        // Full (2 of 2); a newcomer at 600 vs the farthest body at 1900: swap.
        std::vector<NpcPopulationCandidate> candidates{ npc(0, 300.0f, true), npc(1, 1900.0f, true),
            npc(2, 600.0f, false) };
        NpcPopulationPlan plan = planNpcPopulation(candidates, limits(2));
        EXPECT_EQ(plan.mRelease, (std::vector<std::size_t>{ 1 }));
        EXPECT_EQ(plan.mFreeSlots, 1u);
        EXPECT_EQ(plan.mAcquire.front(), 2u);

        // Within the margin (1500 vs 1900): no swap.
        candidates[2].mDistance = 1500.0f;
        plan = planNpcPopulation(candidates, limits(2));
        EXPECT_TRUE(plan.mRelease.empty());
        EXPECT_EQ(plan.mFreeSlots, 0u);
        EXPECT_TRUE(plan.mAcquire.empty());
    }

    TEST(NpcPopulationTest, HeldBodiesKeepTheirSlot)
    {
        NpcPopulationCandidate held = npc(0, 99999.0f, true, false);
        held.mHeld = true;
        const std::vector<NpcPopulationCandidate> candidates{ held, npc(1, 100.0f, false), npc(2, 200.0f, false) };
        const NpcPopulationPlan plan = planNpcPopulation(candidates, limits(2));
        EXPECT_TRUE(plan.mRelease.empty());
        EXPECT_EQ(plan.mFreeSlots, 1u);
        EXPECT_EQ(plan.mAcquire, (std::vector<std::size_t>{ 1, 2 }));
    }

    TEST(NpcPopulationTest, PinnedBodiesTakeASlotButAreNeverTradedAway)
    {
        // A scripted duelist (pinned, the farthest) and a population body,
        // count 1 and a newcomer far nearer: the population body goes, the
        // pinned one stays, and no slot is left.
        NpcPopulationCandidate duelist = npc(0, 1900.0f, true);
        duelist.mPinned = true;
        std::vector<NpcPopulationCandidate> candidates{ duelist, npc(1, 1000.0f, true), npc(2, 100.0f, false) };
        NpcPopulationPlan plan = planNpcPopulation(candidates, limits(1));
        EXPECT_EQ(plan.mRelease, (std::vector<std::size_t>{ 1 }));
        EXPECT_EQ(plan.mFreeSlots, 0u);
        EXPECT_TRUE(plan.mAcquire.empty());

        // Dead or past the release radius, a pinned body goes as before.
        candidates[0].mEligible = false;
        plan = planNpcPopulation(candidates, limits(1));
        EXPECT_TRUE(contains(plan.mRelease, 0));
        candidates[0].mEligible = true;
        candidates[0].mDistance = 4000.0f;
        plan = planNpcPopulation(candidates, limits(1));
        EXPECT_TRUE(contains(plan.mRelease, 0));
    }

    TEST(NpcPopulationTest, LimitsReadTheSwitchAndItsRegisteredDefault)
    {
        ASSERT_EQ(std::getenv("OPENMW_RAGDOLL_NPC_BODIES"), nullptr) << "unset it to run this test";
        const SwitchSpec* spec = findSwitch("OPENMW_RAGDOLL_NPC_BODIES");
        ASSERT_NE(spec, nullptr);
        EXPECT_EQ(spec->mDefault, "10,2000");
        NpcPopulationLimits read = npcPopulationLimits();
        EXPECT_EQ(read.mMaximum, 10u);
        EXPECT_FLOAT_EQ(read.mAcquireRadius, 2000.0f);
        EXPECT_FLOAT_EQ(read.mReleaseRadius, 3500.0f);

        setTuningOverride("OPENMW_RAGDOLL_NPC_BODIES", std::string("25,4000"));
        refreshTuning();
        read = npcPopulationLimits();
        EXPECT_EQ(read.mMaximum, 25u);
        EXPECT_FLOAT_EQ(read.mAcquireRadius, 4000.0f);
        EXPECT_FLOAT_EQ(read.mReleaseRadius, 7000.0f);

        setTuningOverride("OPENMW_RAGDOLL_NPC_BODIES", std::nullopt);
        refreshTuning();
        EXPECT_EQ(npcPopulationLimits().mMaximum, 10u);
    }
}
