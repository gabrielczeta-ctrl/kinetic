#include <ragdollcore/npcshove.hpp>

#include <gtest/gtest.h>

namespace
{
    using namespace RagdollCore;

    NpcShoveInput inReach()
    {
        NpcShoveInput input;
        input.mDistance = 45.0f;
        input.mFacing = 1.0f;
        input.mDt = 0.1f;
        input.mRoll = 0.0f;
        return input;
    }

    TEST(NpcShoveTest, ShovesAGuardInReachMoreOftenThanAnOpenFoe)
    {
        const NpcShoveSettings settings;
        NpcShoveInput guarding = inReach();
        guarding.mFoeGuarding = true;
        NpcShoveInput open = inReach();
        EXPECT_GT(npcShoveChance(guarding, settings), npcShoveChance(open, settings));
        EXPECT_GT(npcShoveChance(open, settings), 0.0f);
        open.mDistance = 60.0f; // in reach, not close, no guard: never
        EXPECT_EQ(npcShoveChance(open, settings), 0.0f);
    }

    TEST(NpcShoveTest, NeverWhileAttackingTiredOutOfReachBehindOrCoolingDown)
    {
        const NpcShoveSettings settings;
        const auto none = [&](auto change) {
            NpcShoveInput input = inReach();
            input.mFoeGuarding = true;
            change(input);
            return npcShoveChance(input, settings) == 0.0f && !npcShouldShove(input, settings);
        };
        EXPECT_TRUE(none([](NpcShoveInput& i) { i.mAttacking = true; }));
        EXPECT_TRUE(none([](NpcShoveInput& i) { i.mFatigue = 0.1f; }));
        EXPECT_TRUE(none([](NpcShoveInput& i) { i.mDistance = 90.0f; }));
        EXPECT_TRUE(none([](NpcShoveInput& i) { i.mFacing = 0.2f; }));
        EXPECT_TRUE(none([](NpcShoveInput& i) { i.mSinceLast = 1.0f; }));
        EXPECT_TRUE(none([](NpcShoveInput& i) { i.mDt = 0.0f; }));
    }

    TEST(NpcShoveTest, StrongerShovesMoreAndTheRollDecides)
    {
        const NpcShoveSettings settings;
        NpcShoveInput weak = inReach();
        weak.mFoeGuarding = true;
        weak.mStrength = 20.0f;
        NpcShoveInput strong = weak;
        strong.mStrength = 100.0f;
        EXPECT_GT(npcShoveChance(strong, settings), npcShoveChance(weak, settings));
        strong.mRoll = 0.999f;
        EXPECT_FALSE(npcShouldShove(strong, settings));
        strong.mRoll = 0.0f;
        EXPECT_TRUE(npcShouldShove(strong, settings));
    }
}
