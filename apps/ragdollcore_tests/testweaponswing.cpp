#include <ragdollcore/weaponswing.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{
    TEST(RagdollCorePunchCycleTest, heldPunchWaitsForWithdrawalAndReleasePreventsAnother)
    {
        for (const float dt : { 1.0f / 30, 1.0f / 60, 1.0f / 120 })
        {
            RagdollCore::PunchCycleState state;
            state.mPendingPress = true;
            EXPECT_TRUE(RagdollCore::stepPunchCycle(state, true, false, dt).mStarted);
            for (int i = 0; i < 120; ++i)
                EXPECT_FALSE(RagdollCore::stepPunchCycle(state, true, false, dt).mStarted);
            EXPECT_FALSE(RagdollCore::stepPunchCycle(state, false, true, dt).mStarted);
            EXPECT_TRUE(RagdollCore::stepPunchCycle(state, true, true, dt).mStarted);
            EXPECT_FALSE(RagdollCore::stepPunchCycle(state, true, true, dt).mStarted);
        }
    }

    TEST(RagdollCorePunchCycleTest, shortClickSurvivesReleaseAndCancellationRemovesRepeat)
    {
        RagdollCore::PunchCycleState state;
        state.mPendingPress = true;
        const auto click = RagdollCore::stepPunchCycle(state, false, true, 1.0f / 60);
        EXPECT_TRUE(click.mStarted);
        EXPECT_TRUE(click.mExtending);
        for (int i = 0; i < 120; ++i)
            EXPECT_FALSE(RagdollCore::stepPunchCycle(state, false, true, 1.0f / 60).mStarted);
        state = {};
        EXPECT_FALSE(RagdollCore::stepPunchCycle(state, true, true, 1.0f / 60).mStarted);
    }

    using RagdollCore::WeaponSwingInput;
    using RagdollCore::WeaponSwingProfile;
    using RagdollCore::WeaponSwingState;

    constexpr float sStep = 1.0f / 60.0f;

    WeaponSwingInput held(float right, float up)
    {
        WeaponSwingInput input;
        input.mAimRight = right;
        input.mAimUp = up;
        input.mHeld = true;
        input.mDuration = sStep;
        return input;
    }

    TEST(RagdollCoreWeaponSwingTest, newPressDuringReturnStartsAnotherAttackWithoutTeleportingHand)
    {
        WeaponSwingState state;
        WeaponSwingProfile profile;
        auto input = held(0.2f, 0);
        EXPECT_TRUE(RagdollCore::stepWeaponSwing(state, input, profile).mStarted);
        input.mAimRight = 0;
        for (int i = 0; i < 27; ++i)
            EXPECT_FALSE(RagdollCore::stepWeaponSwing(state, input, profile).mStarted);
        input.mHeld = false;
        for (int i = 0; i < 21; ++i)
            EXPECT_FALSE(RagdollCore::stepWeaponSwing(state, input, profile).mStarted);
        ASSERT_TRUE(state.mSwinging); // previous attack is still settling
        const auto before = state.mOffset;
        input.mHeld = true;
        const auto second = RagdollCore::stepWeaponSwing(state, input, profile);
        EXPECT_TRUE(second.mStarted);
        EXPECT_LE((state.mOffset - before).Length(), profile.mMaximumSpeed * sStep + 1e-4f);
        EXPECT_FALSE(RagdollCore::stepWeaponSwing(state, input, profile).mStarted);
    }

    TEST(RagdollCoreWeaponSwingTest, verticalStrokeRoutesForwardAroundElbowMinimumReachAndReverses)
    {
        WeaponSwingProfile profile;
        profile.mRestOffset = JPH::Vec3(6, 0, -32);
        profile.mMaximumSpeed = 60;
        WeaponSwingState state;
        auto input = held(0, 0.4f);
        input.mMaximumHandDistance = 34;
        input.mMinimumHandDistance = 16;
        auto previous = profile.mRestOffset;
        float maximumDepth = 0;
        for (int i = 0; i < 100; ++i)
        {
            if (i) input.mAimUp = 0;
            const auto result = RagdollCore::stepWeaponSwing(state, input, profile);
            EXPECT_GE(result.mHandOffset.Length(), 15.999f);
            EXPECT_LE(result.mHandOffset.Length(), 34.001f);
            EXPECT_LE((result.mHandOffset - previous).Length(), 1.001f);
            EXPECT_GE(result.mHandOffset.GetZ(), previous.GetZ() - 1e-4f);
            EXPECT_NEAR(result.mHandOffset.GetX(), 6, 1e-4f);
            maximumDepth = std::max(maximumDepth, result.mHandOffset.GetY());
            previous = result.mHandOffset;
        }
        EXPECT_GE(previous.GetZ(), 3.9f);
        EXPECT_GT(maximumDepth, 14);
        input.mAimUp = -0.1f;
        const auto reverse = RagdollCore::stepWeaponSwing(state, input, profile);
        EXPECT_LT(reverse.mHandOffset.GetZ(), previous.GetZ());
        EXPECT_GE(reverse.mHandOffset.Length(), 15.999f);
        EXPECT_LE(reverse.mSpeed, profile.mMaximumSpeed + 0.001f);
        input.mHeld = false;
        input.mAimUp = 0;
        for (int i = 0; i < 240; ++i)
        {
            const auto released = RagdollCore::stepWeaponSwing(state, input, profile);
            EXPECT_GE(released.mHandOffset.Length(), 15.999f);
            EXPECT_LE(released.mSpeed, profile.mMaximumSpeed + 0.001f);
        }
        EXPECT_FALSE(state.mSwinging);
    }

    TEST(RagdollCoreWeaponSwingTest, reversalDoesNotUnwindUnreachableShoulderTravel)
    {
        WeaponSwingState state;
        state.mOffset = JPH::Vec3(0, 0, 40);
        state.mDesiredOffset = JPH::Vec3(0, 0, 48);
        RagdollCore::constrainWeaponSwingToReach(state, JPH::Vec3(4, -8, 32));
        EXPECT_FLOAT_EQ(state.mOffset.GetZ(), 32);
        EXPECT_FLOAT_EQ(state.mDesiredOffset.GetZ(), 32);
        EXPECT_FLOAT_EQ(state.mOffset.GetX(), 0);
        EXPECT_FLOAT_EQ(state.mOffset.GetY(), 0);
        const auto output = RagdollCore::stepWeaponSwing(state, held(0, -0.1f), WeaponSwingProfile{});
        EXPECT_LT(state.mOffset.GetZ(), 32);
        EXPECT_TRUE(output.mSwinging);
        // A reachable intermediate point must preserve queued speed-limited input.
        state.mOffset = JPH::Vec3(0, 0, 10);
        state.mDesiredOffset = JPH::Vec3(0, 0, 30);
        RagdollCore::constrainWeaponSwingToReach(state, JPH::Vec3(0, 0, 10));
        EXPECT_FLOAT_EQ(state.mDesiredOffset.GetZ(), 30);
    }

    TEST(RagdollCoreWeaponSwingTest, restsWhereTheProfileSaysUntilAimArrives)
    {
        const WeaponSwingProfile profile;
        WeaponSwingState state;
        WeaponSwingInput idle;
        idle.mDuration = sStep;
        const auto output = RagdollCore::stepWeaponSwing(state, idle, profile);
        EXPECT_TRUE(output.mHandOffset.IsClose(profile.mRestOffset, 1e-4f));
        EXPECT_FALSE(output.mSwinging);
        EXPECT_EQ(output.mWeight, 0.0f);
    }

    TEST(RagdollCoreWeaponSwingTest, aimDrivesTheHandAcrossTheFrontalPlane)
    {
        const WeaponSwingProfile profile;
        WeaponSwingState state;
        // A steady rightward drag.
        for (int i = 0; i < 10; ++i)
            RagdollCore::stepWeaponSwing(state, held(0.05f, 0.0f), profile);
        EXPECT_GT(state.mOffset.GetX(), 1.0f);
        EXPECT_NEAR(state.mOffset.GetY(), 0.0f, 1e-4f);
        EXPECT_NEAR(state.mOffset.GetZ(), 0.0f, 1e-4f);

        // Upward drag moves it up, independently.
        WeaponSwingState vertical;
        for (int i = 0; i < 10; ++i)
            RagdollCore::stepWeaponSwing(vertical, held(0.0f, 0.05f), profile);
        EXPECT_GT(vertical.mOffset.GetZ(), 1.0f);
        EXPECT_NEAR(vertical.mOffset.GetX(), 0.0f, 1e-4f);
    }

    TEST(RagdollCoreWeaponSwingTest, theArmNotTheMouseSetsHowFastTheTargetTravels)
    {
        WeaponSwingProfile profile;
        profile.mMaximumSpeed = 100.0f;
        WeaponSwingState state;
        // A violent flick: far more than the arm could follow in one step.
        const auto output = RagdollCore::stepWeaponSwing(state, held(10.0f, 0.0f), profile);
        EXPECT_TRUE(output.mSpeedLimited);
        // The commanded point moved by the arm's limit, not the mouse's ask.
        EXPECT_NEAR(state.mOffset.Length(), profile.mMaximumSpeed * sStep, 1e-3f);
        EXPECT_NEAR(output.mSpeed, profile.mMaximumSpeed, 1e-2f);

        // A gentle drag is inside the limit and tracks the mouse exactly.
        WeaponSwingState gentle;
        const auto slow = RagdollCore::stepWeaponSwing(gentle, held(0.01f, 0.0f), profile);
        EXPECT_FALSE(slow.mSpeedLimited);
        EXPECT_NEAR(gentle.mOffset.GetX(), 0.01f * profile.mAimToTravel, 1e-4f);
    }

    TEST(RagdollCoreWeaponSwingTest, theHandNeverLeavesTheArmsReach)
    {
        const WeaponSwingProfile profile;
        WeaponSwingState state;
        // Drag hard in one direction for a long time.
        for (int i = 0; i < 600; ++i)
            RagdollCore::stepWeaponSwing(state, held(0.5f, 0.3f), profile);
        EXPECT_LE(state.mOffset.Length(), profile.mReach + 1e-3f);
    }

    TEST(RagdollCoreWeaponSwingTest, releasingFollowsThroughInsteadOfCancelling)
    {
        const WeaponSwingProfile profile;
        WeaponSwingState state;
        for (int i = 0; i < 20; ++i)
            RagdollCore::stepWeaponSwing(state, held(0.1f, 0.0f), profile);
        const float swung = state.mOffset.Length();
        ASSERT_GT(swung, 1.0f);

        WeaponSwingInput released;
        released.mDuration = sStep;
        const auto first = RagdollCore::stepWeaponSwing(state, released, profile);
        // Still swinging on the step after release: the arc finishes.
        EXPECT_TRUE(first.mSwinging);
        EXPECT_LT(state.mOffset.Length(), swung);
        EXPECT_GT(state.mOffset.Length(), 0.5f * swung);

        for (int i = 0; i < 600; ++i)
            RagdollCore::stepWeaponSwing(state, released, profile);
        const auto settled = RagdollCore::stepWeaponSwing(state, released, profile);
        EXPECT_FALSE(settled.mSwinging);
        EXPECT_TRUE(settled.mHandOffset.IsClose(profile.mRestOffset, 1e-2f));
    }

    TEST(RagdollCoreWeaponSwingTest, reportsTheSpeedTheTargetActuallyMoved)
    {
        const WeaponSwingProfile profile;
        WeaponSwingState state;
        const auto output = RagdollCore::stepWeaponSwing(state, held(0.02f, 0.0f), profile);
        // Not the mouse delta and not the profile limit: what the commanded
        // point did, which is what a caller measuring a blow needs.
        EXPECT_NEAR(output.mSpeed, 0.02f * profile.mAimToTravel / sStep, 1e-2f);
        EXPECT_NEAR(state.mVelocity.GetX(), output.mSpeed, 1e-2f);
    }

    TEST(RagdollCoreWeaponSwingTest, rejectsNonsenseWithoutDisturbingTheSwing)
    {
        const WeaponSwingProfile profile;
        WeaponSwingState state;
        for (int i = 0; i < 10; ++i)
            RagdollCore::stepWeaponSwing(state, held(0.05f, 0.0f), profile);
        const auto before = state.mOffset;

        WeaponSwingInput broken = held(std::numeric_limits<float>::quiet_NaN(), 0.0f);
        const auto rejected = RagdollCore::stepWeaponSwing(state, broken, profile);
        EXPECT_TRUE(state.mOffset.IsClose(before));
        EXPECT_FALSE(rejected.mSwinging);

        WeaponSwingInput zeroStep = held(0.05f, 0.0f);
        zeroStep.mDuration = 0.0f;
        RagdollCore::stepWeaponSwing(state, zeroStep, profile);
        EXPECT_TRUE(state.mOffset.IsClose(before));
    }
    TEST(RagdollCoreWeaponSwingTest, speedLimitedInputCatchesUpWithoutMoreMouseEvents)
    {
        const WeaponSwingProfile profile;
        WeaponSwingState state;
        const auto first = RagdollCore::stepWeaponSwing(state, held(0.0f, 0.2f), profile);
        ASSERT_TRUE(first.mSpeedLimited);
        for (int i = 0; i < 10; ++i)
            RagdollCore::stepWeaponSwing(state, held(0.0f, 0.0f), profile);
        EXPECT_NEAR(state.mOffset.GetZ(), 0.2f * profile.mAimToTravel, 1e-4f);
        EXPECT_NEAR(state.mOffset.GetY(), 0.0f, 1e-6f);
        // Undo the same input without needing another event to finish moving.
        RagdollCore::stepWeaponSwing(state, held(0.0f, -0.2f), profile);
        for (int i = 0; i < 10; ++i)
            RagdollCore::stepWeaponSwing(state, held(0.0f, 0.0f), profile);
        EXPECT_TRUE(state.mOffset.IsNearZero(1e-4f));
    }

    TEST(RagdollCoreWeaponSwingTest, releaseDiscardsUndeliveredInputAndSettlesAtDifferentStepRates)
    {
        const WeaponSwingProfile profile;
        for (const float dt : { 1.0f / 30.0f, 1.0f / 60.0f, 1.0f / 120.0f })
        {
            WeaponSwingState state;
            RagdollCore::stepWeaponSwing(state, held(1.0f, 0.0f), profile);
            const float previous = state.mOffset.Length();
            WeaponSwingInput released;
            released.mDuration = dt;
            RagdollCore::stepWeaponSwing(state, released, profile);
            EXPECT_LT(state.mOffset.Length(), previous);
            for (int i = 0; i < static_cast<int>(4.0f / dt); ++i)
                RagdollCore::stepWeaponSwing(state, released, profile);
            EXPECT_FALSE(state.mSwinging);
            EXPECT_LT(state.mOffset.Length(), 1e-3f);
        }
    }

    TEST(RagdollCoreWeaponSwingTest, nativeReachBoundsTargetsAndSmallReversalsDoNotUnwindUnreachableInput)
    {
        const WeaponSwingProfile profile;
        for (float dt : { 1.0f / 30, 1.0f / 60, 1.0f / 120 })
            for (bool horizontal : { false, true })
            {
                WeaponSwingState state;
                auto input = held(horizontal ? 2.0f : 0.0f, horizontal ? 0.0f : 2.0f);
                input.mDuration = dt;
                input.mMaximumHandDistance = 34.0f;
                for (int i = 0; i < 60; ++i)
                {
                    const auto output = RagdollCore::stepWeaponSwing(state, input, profile);
                    EXPECT_LE(output.mHandOffset.Length(), input.mMaximumHandDistance + 1e-4f);
                    EXPECT_LE(output.mSpeed, profile.mMaximumSpeed + 1e-3f);
                    EXPECT_NEAR(output.mHandOffset.GetY(), profile.mRestOffset.GetY(), 1e-4f);
                    EXPECT_NEAR(horizontal ? output.mHandOffset.GetZ() : output.mHandOffset.GetX(),
                        horizontal ? profile.mRestOffset.GetZ() : profile.mRestOffset.GetX(), 1e-4f);
                }
                const auto limit = state.mOffset;
                input.mAimRight = horizontal ? -0.1f : 0;
                input.mAimUp = horizontal ? 0 : -0.1f;
                RagdollCore::stepWeaponSwing(state, input, profile);
                input.mAimRight = input.mAimUp = 0;
                for (int i = 0; i < 20; ++i)
                    RagdollCore::stepWeaponSwing(state, input, profile);
                EXPECT_NEAR(horizontal ? limit.GetX() - state.mOffset.GetX() : limit.GetZ() - state.mOffset.GetZ(),
                    0.1f * profile.mAimToTravel, 1e-3f);
            }
    }

    TEST(RagdollCoreWeaponSwingTest, closeThrustAdvancesFromBehindTheShoulderAtBoundedSpeed)
    {
        for (float dt : { 1.0f / 30, 1.0f / 60, 1.0f / 120 })
        {
            WeaponSwingProfile profile;
            profile.mRestOffset = JPH::Vec3(0, -15, 0);
            WeaponSwingState state;
            auto input = held(0, 0);
            input.mDuration = dt;
            input.mMinimumHandDistance = 10;
            input.mMaximumHandDistance = 34;
            JPH::Vec3 previous = profile.mRestOffset;
            for (int i = 0; i < static_cast<int>(0.5f / dt); ++i)
            {
                const auto output = RagdollCore::stepWeaponSwing(state, input, profile);
                EXPECT_LE((output.mHandOffset - previous).Length(), profile.mMaximumSpeed * dt + 1e-3f);
                previous = output.mHandOffset;
            }
            EXPECT_NEAR(previous.GetY(), 10, 1e-3f);
            input.mHeld = false;
            input.mMinimumHandDistance = 0;
            for (int i = 0; i < static_cast<int>(4 / dt); ++i)
                RagdollCore::stepWeaponSwing(state, input, profile);
            EXPECT_FALSE(state.mSwinging);
        }
    }

    TEST(RagdollCoreWeaponSwingTest, smallerNativeReachScalesGuardAndStillReleases)
    {
        const WeaponSwingProfile profile;
        WeaponSwingState state;
        auto input = held(0, 0);
        input.mMaximumHandDistance = 25;
        const auto rest = RagdollCore::stepWeaponSwing(state, input, profile).mHandOffset;
        EXPECT_LT(rest.Length(), 25);
        input.mAimRight = input.mAimUp = 2;
        for (int i = 0; i < 30; ++i)
            EXPECT_LE(RagdollCore::stepWeaponSwing(state, input, profile).mHandOffset.Length(), 25.0001f);
        const auto before = state.mOffset;
        input.mMaximumHandDistance = -1;
        EXPECT_FALSE(RagdollCore::stepWeaponSwing(state, input, profile).mSwinging);
        EXPECT_TRUE(state.mOffset.IsClose(before));
        input.mMaximumHandDistance = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::stepWeaponSwing(state, input, profile).mSwinging);
        EXPECT_TRUE(state.mOffset.IsClose(before));
        input.mMaximumHandDistance = 25;
        input.mHeld = false;
        for (int i = 0; i < 240; ++i)
            EXPECT_LE(RagdollCore::stepWeaponSwing(state, input, profile).mHandOffset.Length(), 25.0001f);
        const auto released = RagdollCore::stepWeaponSwing(state, input, profile);
        EXPECT_FALSE(released.mSwinging);
        EXPECT_TRUE(released.mHandOffset.IsClose(rest, 1e-3f));
    }

}
