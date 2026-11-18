// KINETIC-2 phases 3 and 4: the moved modules (uprightholder, rootstrength,
// roothold, lifecycle). Each is the old integration's rule moved as it was;
// these pin the rules, the game's shadow runs (K2_* = 2) pin the equality.
#include <ragdollcore/lifecycle.hpp>
#include <ragdollcore/roothold.hpp>
#include <ragdollcore/calmhold.hpp>
#include <ragdollcore/rootstrength.hpp>
#include <ragdollcore/uprightholder.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{
    using namespace RagdollCore;

    constexpr float sDt = 1.0f / 60.0f;

    float upDotForDegrees(float degrees)
    {
        return std::cos(degrees * 3.14159265f / 180.0f);
    }

    TiltHoldInput standingStill(float degrees)
    {
        TiltHoldInput input;
        input.mKnown = true;
        input.mTorsoUpDot = upDotForDegrees(degrees);
        return input;
    }

    TEST(K2UprightHolder, TiltHoldEngagesAfterItsDelayAndEasesIn)
    {
        TiltHoldState state;
        const TiltHoldSettings settings;
        float share = 0.0f;
        for (int i = 0; i < 29; ++i)
            share = updateTiltHold(state, standingStill(12.0f), settings, sDt);
        EXPECT_FALSE(state.mHolding);
        EXPECT_FLOAT_EQ(share, 0.0f);
        for (int i = 0; i < 2; ++i)
            share = updateTiltHold(state, standingStill(12.0f), settings, sDt);
        EXPECT_TRUE(state.mHolding);
        EXPECT_GT(share, 0.0f);
        // Eased over 0.4 s toward max(0.5, (12 - 7) / 8).
        for (int i = 0; i < 60; ++i)
            share = updateTiltHold(state, standingStill(12.0f), settings, sDt);
        EXPECT_NEAR(share, 0.625f, 1e-4f);
    }

    TEST(K2UprightHolder, TiltHoldLetsGoOnlyAfterHalfASecondUpright)
    {
        TiltHoldState state;
        const TiltHoldSettings settings;
        for (int i = 0; i < 90; ++i)
            updateTiltHold(state, standingStill(10.0f), settings, sDt);
        ASSERT_TRUE(state.mHolding);
        // 5 deg is inside `from` but not under from - 3: still held.
        for (int i = 0; i < 60; ++i)
            updateTiltHold(state, standingStill(5.0f), settings, sDt);
        EXPECT_TRUE(state.mHolding);
        for (int i = 0; i < 31; ++i)
            updateTiltHold(state, standingStill(2.0f), settings, sDt);
        EXPECT_FALSE(state.mHolding);
    }

    TEST(K2UprightHolder, AHitLetsTheTiltHoldGoAtOnce)
    {
        TiltHoldState state;
        const TiltHoldSettings settings;
        for (int i = 0; i < 90; ++i)
            updateTiltHold(state, standingStill(14.0f), settings, sDt);
        ASSERT_GT(state.mShare, 0.5f);
        TiltHoldInput hit = standingStill(14.0f);
        hit.mHit = true;
        EXPECT_FLOAT_EQ(updateTiltHold(state, hit, settings, sDt), 0.0f);
        EXPECT_FALSE(state.mHolding);
    }

    TEST(K2UprightHolder, HeldAssistedKeepsOnlyTheScaledSpineTorque)
    {
        UprightControllerOutput output;
        output.mPelvisForce = JPH::Vec3(10, 20, 30);
        output.mPelvisTorque = JPH::Vec3(1, 2, 3);
        output.mSpineTorque = JPH::Vec3(4, 5, 6);
        const auto held = composeUprightControl(output, true, 0.5f, true, true);
        EXPECT_TRUE(held.mPelvisForce.IsNearZero());
        EXPECT_TRUE(held.mPelvisTorque.IsNearZero());
        EXPECT_TRUE(held.mSpineTorque.IsClose(JPH::Vec3(2, 2.5f, 3)));
        const auto free = composeUprightControl(output, false, 0.5f, false, true);
        EXPECT_TRUE(free.mPelvisForce.IsClose(JPH::Vec3(0, 0, 30)));
        EXPECT_TRUE(free.mSpineTorque.IsClose(output.mSpineTorque));
        const auto noVertical = composeUprightControl(output, false, 1.0f, true, false);
        EXPECT_TRUE(noVertical.mPelvisForce.IsClose(JPH::Vec3(10, 20, 0)));
    }

    TEST(K2UprightHolder, AssemblyFollowsTheGetUpClipAndHoldsWorldUpOtherwise)
    {
        UprightAssembly a;
        a.mCorrectTranslation = true;
        a.mGroundContact = true;
        a.mHasSupport = true;
        a.mTargetPelvisHeight = 66.0f;
        a.mHandoverPelvisHeight = 60.0f;
        a.mPelvisHeight = 63.0f;
        a.mCenterOfMassOffset = JPH::Vec3(1, 2, 0);
        a.mHeldFollowTargetUp = JPH::Vec3(0, 1, 0);
        auto input = assembleUprightInput(a);
        EXPECT_FLOAT_EQ(input.mTargetPelvisHeight, 60.0f);
        EXPECT_FLOAT_EQ(input.mPelvisHeight, 63.0f);
        EXPECT_TRUE(input.mCenterOfMassOffset.IsClose(JPH::Vec3(1, 2, 0)));
        EXPECT_TRUE(input.mTargetUp.IsClose(JPH::Vec3(0, 1, 0)));
        a.mGetUpTargetUp = JPH::Vec3(1, 0, 0);
        input = assembleUprightInput(a);
        EXPECT_TRUE(input.mTargetUp.IsClose(JPH::Vec3(1, 0, 0)));
        // No ground: the measured height is the target's, translation not corrected.
        a.mCorrectTranslation = false;
        a.mGroundContact = false;
        input = assembleUprightInput(a);
        EXPECT_FLOAT_EQ(input.mPelvisHeight, 66.0f);
        EXPECT_TRUE(input.mCenterOfMassOffset.IsNearZero());
        limitStandingFriction(input, 0.4f, true);
        EXPECT_TRUE(input.mLimitToSupportFriction);
        EXPECT_FLOAT_EQ(input.mSupportFriction, 0.4f);
    }

    RootStrengthInput still()
    {
        RootStrengthInput input;
        input.mTilt.mKnown = true;
        return input;
    }

    TEST(K2RootStrength, TheBudgetByStandingMovingAndBraking)
    {
        const RootStrengthSettings settings;
        RootStrengthState state;
        auto r = updateRootStrength(still(), settings, state, sDt);
        EXPECT_NEAR(r.mStrength, 0.15f, 1e-5f);
        RootStrengthInput moving = still();
        moving.mRequestSpeed = moving.mBodySpeed = 127.0f;
        r = updateRootStrength(moving, settings, state, sDt);
        EXPECT_NEAR(r.mStrength, 0.3f, 1e-5f);
        RootStrengthInput braking = still();
        braking.mRequestSpeed = 0.0f;
        braking.mBodySpeed = 127.0f;
        r = updateRootStrength(braking, settings, state, sDt);
        EXPECT_NEAR(r.mStrength, 0.45f, 1e-5f);
        // Running: the budget weakens only the horizontal hold.
        RootStrengthInput running = still();
        running.mRequestSpeed = running.mBodySpeed = 230.0f;
        r = updateRootStrength(running, settings, state, sDt);
        EXPECT_TRUE(r.mMovingVertical);
        EXPECT_NEAR(r.mStrength, 1.0f, 1e-5f);
        EXPECT_NEAR(r.mHorizontal, 0.3f, 1e-5f);
    }

    TEST(K2RootStrength, ClimbingKeepsTheWholeStrength)
    {
        RootStrengthState state;
        RootStrengthInput climbing = still();
        climbing.mRequestSpeed = climbing.mBodySpeed = 127.0f;
        climbing.mVerticalVelocity = 120.0f;
        const auto r = updateRootStrength(climbing, {}, state, sDt);
        EXPECT_TRUE(r.mClimbing);
        EXPECT_NEAR(r.mStrength, 1.0f, 1e-5f);
        EXPECT_NEAR(r.mHorizontal, 1.0f, 1e-5f);
    }

    TEST(K2RootStrength, AGetUpRampsFromItsStartShare)
    {
        RootStrengthState state;
        RootStrengthInput getUp = still();
        getUp.mRecoveryOwns = true;
        auto r = updateRootStrength(getUp, {}, state, sDt);
        EXPECT_NEAR(r.mStrength, 0.2f + 0.8f * sDt / 1.2f, 1e-5f);
        for (int i = 0; i < 80; ++i)
            r = updateRootStrength(getUp, {}, state, sDt);
        EXPECT_NEAR(r.mStrength, 1.0f, 1e-5f);
        // A rehold restarts its own ramp.
        state.mReholdElapsed = 0.0f;
        r = updateRootStrength(still(), {}, state, sDt);
        EXPECT_NEAR(r.mHeld, 0.2f + 0.8f * sDt / 1.2f, 1e-5f);
    }

    TEST(K2RootStrength, TheTiltHoldRaisesTheBudgetTowardTheHeldStrength)
    {
        RootStrengthState state;
        RootStrengthInput tipped = still();
        tipped.mTilt.mTorsoUpDot = upDotForDegrees(15.0f);
        RootStrength r;
        for (int i = 0; i < 120; ++i)
            r = updateRootStrength(tipped, {}, state, sDt);
        EXPECT_NEAR(r.mTiltShare, 1.0f, 1e-4f);
        EXPECT_NEAR(r.mBudgeted, 0.15f, 1e-5f);
        EXPECT_NEAR(r.mStrength, 1.0f, 1e-4f);
    }

    TEST(K2RootStrength, TheDropRollOwnsTheRoot)
    {
        RootStrengthState state;
        RootStrengthInput roll = still();
        roll.mRoll = RootRollPhase::Roll;
        auto r = updateRootStrength(roll, {}, state, sDt);
        EXPECT_NEAR(r.mStrength, 0.35f, 1e-5f);
        EXPECT_NEAR(r.mHorizontal, 0.0f, 1e-5f);
        roll.mRoll = RootRollPhase::Absorb;
        r = updateRootStrength(roll, {}, state, sDt);
        EXPECT_NEAR(r.mHorizontal, 0.3f, 1e-5f);
    }

    TEST(K2RootStrength, HitRecoveryIsSlowerAtLowControl)
    {
        EXPECT_NEAR(recoverRootStrength(0.5f, 1.0f, 1.0f, 0.4f, 0.1f), 0.6f, 1e-5f);
        EXPECT_NEAR(recoverRootStrength(0.5f, 1.0f, 0.2f, 0.4f, 0.1f), 0.56f, 1e-5f);
        EXPECT_FLOAT_EQ(recoverRootStrength(0.99f, 1.0f, 1.0f, 0.4f, 0.1f), 1.0f);
    }

    TEST(K2RootStrength, TheWeightBudgetEasesToTheWalkingShare)
    {
        RootSupportSettings settings;
        settings.mWalkSupport = 0.6f;
        RootSupportState state;
        RootSupportInput walking;
        walking.mGroundContact = true;
        walking.mRequestSpeed = 127.0f;
        float support = 1.0f;
        for (int i = 0; i < 3; ++i)
            support = updateRootSupport(walking, settings, state, sDt);
        EXPECT_NEAR(support, 1.0f - 3 * sDt / 0.3f, 1e-4f);
        for (int i = 0; i < 30; ++i)
            support = updateRootSupport(walking, settings, state, sDt);
        EXPECT_NEAR(support, 0.6f, 1e-5f);
        RootSupportInput running = walking;
        running.mRequestSpeed = 230.0f;
        for (int i = 0; i < 30; ++i)
            support = updateRootSupport(running, settings, state, sDt);
        EXPECT_NEAR(support, 1.0f, 1e-5f);
    }

    TEST(K2RootStrength, AWalkOnUnevenGroundBudgetsTheHorizontalHoldOnly)
    {
        RootStrengthSettings settings;
        settings.mBudget = true;
        settings.mBudgetShares = { 0.15f, 0.3f, 0.45f };
        settings.mMoveVertical = 180.0f;
        RootStrengthInput walking;
        walking.mRequestSpeed = 120.0f;
        walking.mBodySpeed = 120.0f;
        RootStrengthState level;
        level.mReholdElapsed = 100.0f;
        const RootStrength flat = updateRootStrength(walking, settings, level, sDt);
        EXPECT_LT(flat.mStrength, 0.5f);
        // On a slope the lift comes back with the hang; the horizontal
        // hold (strength x budget) stays what it was.
        settings.mSagHold = { 15.0f, 40.0f };
        walking.mUneven = true;
        walking.mSag = 10.0f;
        RootStrengthState slope;
        slope.mReholdElapsed = 100.0f;
        EXPECT_FLOAT_EQ(updateRootStrength(walking, settings, slope, sDt).mStrength, flat.mStrength);
        walking.mSag = 27.5f;
        const RootStrength half = updateRootStrength(walking, settings, slope, sDt);
        EXPECT_NEAR(half.mStrength, 0.5f * (flat.mStrength + 1.0f), 1e-4f);
        EXPECT_NEAR(half.mStrength * half.mHorizontalBudget, flat.mStrength * flat.mHorizontalBudget, 1e-4f);
        walking.mSag = 60.0f;
        const RootStrength climb = updateRootStrength(walking, settings, slope, sDt);
        EXPECT_FLOAT_EQ(climb.mStrength, 1.0f);
        EXPECT_NEAR(climb.mStrength * climb.mHorizontalBudget, flat.mStrength * flat.mHorizontalBudget, 1e-4f);
        // Level ground: no change however the pelvis hangs.
        walking.mUneven = false;
        RootStrengthState again;
        again.mReholdElapsed = 100.0f;
        EXPECT_FLOAT_EQ(updateRootStrength(walking, settings, again, sDt).mStrength, flat.mStrength);
    }

    TEST(K2RootStrength, AWalkOnUnevenGroundKeepsTheWholeLift)
    {
        RootSupportSettings settings;
        settings.mWalkSupport = 0.6f;
        RootSupportState state;
        RootSupportInput walking;
        walking.mGroundContact = true;
        walking.mRequestSpeed = 127.0f;
        float support = 1.0f;
        for (int i = 0; i < 40; ++i)
            support = updateRootSupport(walking, settings, state, sDt);
        EXPECT_NEAR(support, 0.6f, 1e-5f);
        walking.mUneven = true;
        for (int i = 0; i < 30; ++i)
            support = updateRootSupport(walking, settings, state, sDt);
        EXPECT_NEAR(support, 1.0f, 1e-5f);
        walking.mUneven = false;
        for (int i = 0; i < 30; ++i)
            support = updateRootSupport(walking, settings, state, sDt);
        EXPECT_NEAR(support, 0.6f, 1e-5f);
    }

    TEST(K2Calm, ACalmNpcIsHeldAndLetGoWhenBothered)
    {
        CalmHoldSettings settings;
        settings.mStrength = 0.9f;
        CalmHoldInput in;
        in.mAllowed = true;
        CalmHoldState state;
        // Powered unbothered: held from the first step, the legs on the clip.
        CalmHold hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_FLOAT_EQ(hold.mLevel, 1.0f);
        EXPECT_TRUE(hold.mLegsOnClip);
        // A fight: the legs step at once, the root eases down over 0.2 s.
        in.mBothered = true;
        hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_FALSE(hold.mLegsOnClip);
        EXPECT_GT(hold.mLevel, 0.8f);
        for (int i = 0; i < 12; ++i)
            hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_FLOAT_EQ(hold.mLevel, 0.0f);
        // Quiet for 4 s, then the root first and the legs from half way.
        in.mBothered = false;
        for (int i = 0; i < 239; ++i)
            hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_FLOAT_EQ(hold.mLevel, 0.0f);
        for (int i = 0; i < 20; ++i)
            hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_GT(hold.mLevel, 0.0f);
        EXPECT_FALSE(hold.mLegsOnClip);
        for (int i = 0; i < 60; ++i)
            hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_FLOAT_EQ(hold.mLevel, 1.0f);
        EXPECT_TRUE(hold.mLegsOnClip);
        EXPECT_FLOAT_EQ(hold.mDraw, 1.0f);
        // A bump that lasts lets go, all the way; a walker's start does not.
        in.mOffTarget = 20.0f;
        for (int i = 0; i < 5; ++i)
            hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_FALSE(hold.mBumped);
        in.mOffTarget = 0.0f;
        hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_FLOAT_EQ(hold.mLevel, 1.0f);
        in.mOffTarget = 20.0f;
        for (int i = 0; i < 7; ++i)
            hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_FALSE(hold.mBumped);
        for (int i = 0; i < 4 && !hold.mBumped; ++i)
            hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_TRUE(hold.mBumped);
        EXPECT_FALSE(hold.mLegsOnClip);
        in.mOffTarget = 0.0f;
        for (int i = 0; i < 15; ++i)
            hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_FLOAT_EQ(hold.mLevel, 0.0f);
        // Not taken back while the body is far from the animation, or tilted.
        in.mOffTarget = 20.0f;
        for (int i = 0; i < 600; ++i)
            hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_FLOAT_EQ(hold.mLevel, 0.0f);
        in.mOffTarget = 0.0f;
        in.mUpright = false;
        for (int i = 0; i < 600; ++i)
            hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_LT(hold.mLevel, 1.0f);
        // A blow lets the body go at once; the picture eases over the ease-down time.
        state.mLevel = 1.0f;
        state.mDraw = 1.0f;
        breakCalmHold(state);
        EXPECT_FLOAT_EQ(state.mLevel, 0.0f);
        in.mUpright = true;
        in.mBothered = true;
        hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_GT(hold.mDraw, 0.8f);
        for (int i = 0; i < 12; ++i)
            hold = updateCalmHold(in, settings, state, sDt);
        EXPECT_FLOAT_EQ(hold.mDraw, 0.0f);
        // Off, or the player: never held.
        CalmHoldState other;
        in = {};
        EXPECT_FLOAT_EQ(updateCalmHold(in, settings, other, sDt).mLevel, 0.0f);
        in.mAllowed = true;
        settings.mStrength = 0.0f;
        EXPECT_FLOAT_EQ(updateCalmHold(in, settings, other, sDt).mLevel, 0.0f);
    }

    TEST(K2Calm, TheHeldRootKeepsItsStrengthSidewaysHoldAndLift)
    {
        RootStrengthSettings settings;
        settings.mBudget = true;
        settings.mBudgetShares = { 0.15f, 0.3f, 0.45f };
        settings.mMoveVertical = 180.0f;
        settings.mCalmStrength = 0.9f;
        RootStrengthInput walking;
        walking.mRequestSpeed = 120.0f;
        walking.mBodySpeed = 120.0f;
        RootStrengthState state;
        state.mReholdElapsed = 100.0f;
        const RootStrength kinetic = updateRootStrength(walking, settings, state, sDt);
        EXPECT_LT(kinetic.mStrength, 0.5f);
        walking.mCalm = 1.0f;
        const RootStrength held = updateRootStrength(walking, settings, state, sDt);
        EXPECT_FLOAT_EQ(held.mStrength, 0.9f);
        EXPECT_FLOAT_EQ(held.mHorizontal, 1.0f);
        walking.mCalm = 0.5f;
        EXPECT_NEAR(updateRootStrength(walking, settings, state, sDt).mStrength, 0.5f * (kinetic.mStrength + 0.9f), 1e-4f);
        // A hit's strength still caps it; a get-up keeps its own ramp.
        walking.mCalm = 1.0f;
        walking.mHitStrength = 0.4f;
        EXPECT_LE(updateRootStrength(walking, settings, state, sDt).mStrength, 0.4f);
        RootSupportSettings support;
        support.mWalkSupport = 0.6f;
        RootSupportState supportState;
        RootSupportInput in;
        in.mGroundContact = true;
        in.mRequestSpeed = 127.0f;
        float carried = 1.0f;
        for (int i = 0; i < 40; ++i)
            carried = updateRootSupport(in, support, supportState, sDt);
        EXPECT_NEAR(carried, 0.6f, 1e-5f);
        in.mCalm = 1.0f;
        EXPECT_NEAR(updateRootSupport(in, support, supportState, sDt), 1.0f, 1e-5f);
    }

    TEST(K2RootStrength, ALimpUnloadsTheRootOverAWoundedSoleLeg)
    {
        RootSupportSettings settings;
        settings.mLimpBudget = 0.5f;
        RootSupportState state;
        RootSupportInput limp;
        limp.mGroundContact = true;
        limp.mFootfall = true;
        limp.mPinned = { true, false };
        limp.mWound = { 0.8f, 0.0f };
        float support = 1.0f;
        for (int i = 0; i < 120; ++i)
            support = updateRootSupport(limp, settings, state, sDt);
        EXPECT_NEAR(support, 0.6f, 1e-3f);
    }

    RootHoldInput mode(ActiveControllerMode m, float upDot = 1.0f, float speed = 0.0f)
    {
        RootHoldInput input;
        input.mMode = m;
        input.mKnown = true;
        input.mTorsoUpDot = upDot;
        input.mCenterOfMassSpeed = speed;
        input.mRootKinematic = true;
        return input;
    }

    TEST(K2RootHold, ATippingBodyTopplesAfterItsHold)
    {
        RootHoldState state;
        const RootHoldSettings settings;
        float up = 0.45f;
        bool toppled = false;
        int steps = 0;
        while (!toppled && steps < 60)
        {
            up -= 0.001f;
            toppled = judgeTopple(mode(ActiveControllerMode::Standing, up), false, settings, state, sDt).mToppled;
            ++steps;
        }
        EXPECT_TRUE(toppled);
        EXPECT_GE(steps, 9); // 0.15 s, float steps
        EXPECT_LE(steps, 10);
        EXPECT_TRUE(state.mToppled);
    }

    TEST(K2RootHold, ARisingLeanDoesNotCountButFallsAfterASecond)
    {
        RootHoldState state;
        const RootHoldSettings settings;
        float up = 0.3f;
        int steps = 0;
        bool toppled = false;
        while (!toppled && steps < 120)
        {
            up += 0.0005f; // past the line, coming back up slowly
            toppled = judgeTopple(mode(ActiveControllerMode::Standing, up), false, settings, state, sDt).mToppled;
            ++steps;
        }
        EXPECT_TRUE(toppled);
        EXPECT_GE(steps, 60);
        EXPECT_LE(steps, 61);
    }

    TEST(K2RootHold, AToppledBodyStandingUprightIsTakenBack)
    {
        RootHoldState state;
        state.mToppled = true;
        state.mFallDecidedTime = 2.0f;
        const RootHoldSettings settings;
        bool reheld = false;
        int steps = 0;
        while (!reheld && steps < 120)
        {
            reheld = judgeTopple(mode(ActiveControllerMode::Bracing, 0.9f), false, settings, state, sDt).mReheld;
            ++steps;
        }
        EXPECT_TRUE(reheld);
        EXPECT_GE(steps, 30);
        EXPECT_LE(steps, 31);
        EXPECT_TRUE(state.mReheld);
        // The rehold retakes the root at once.
        state.mPhysicsOwned = true;
        const auto retake = judgeRootRetake(mode(ActiveControllerMode::Bracing, 0.9f), settings, state, sDt);
        EXPECT_TRUE(retake.mReacquired);
        EXPECT_TRUE(retake.mBraceOrRehold);
        EXPECT_TRUE(rootHeld(retake, state, true, true, false));
    }

    TEST(K2RootHold, PhysicsKeepsTheBodyUntilItStandsSettled)
    {
        RootHoldState state;
        state.mPhysicsOwned = true;
        const RootHoldSettings settings;
        auto retake = judgeRootRetake(mode(ActiveControllerMode::Standing, 1.0f, 10.0f), settings, state, sDt);
        EXPECT_FALSE(rootHeld(retake, state, true, true, false));
        for (int i = 0; i < 15; ++i)
            retake = judgeRootRetake(mode(ActiveControllerMode::Standing, 1.0f, 10.0f), settings, state, sDt);
        EXPECT_TRUE(rootHeld(retake, state, true, true, false));
        // A get-up owns it at once and ends a landing's release.
        RootHoldState landed;
        landed.mPhysicsOwned = true;
        landed.mLandingReleased = true;
        retake = judgeRootRetake(mode(ActiveControllerMode::GettingUp), settings, landed, sDt);
        EXPECT_FALSE(landed.mLandingReleased);
        EXPECT_TRUE(rootHeld(retake, landed, true, true, false));
        // Falling never.
        retake = judgeRootRetake(mode(ActiveControllerMode::Falling), settings, landed, sDt);
        EXPECT_FALSE(rootHeld(retake, landed, true, true, false));
    }

    TEST(K2Lifecycle, TheDeathCarryCapsTheCommonVelocity)
    {
        const std::vector<JPH::Vec3> parts{ JPH::Vec3(200, 0, 0), JPH::Vec3(220, 0, 0) };
        DeathCarrySettings settings;
        auto carry = deathCarry(parts, settings);
        EXPECT_TRUE(carry.mCommon.IsClose(JPH::Vec3(15, 0, 0)));
        // The old leak: each part keeps its share off the capped common velocity.
        EXPECT_NEAR(carry.linear(parts[0]).GetX(), 15.0f + 0.15f * 185.0f, 1e-3f);
        settings.mTrueCap = true;
        carry = deathCarry(parts, settings);
        EXPECT_NEAR(carry.linear(parts[0]).GetX(), 15.0f - 0.15f * 10.0f, 1e-3f);
        EXPECT_TRUE(carry.angular(JPH::Vec3(10, 0, 0)).IsClose(JPH::Vec3(1.5f, 0, 0)));
    }

    // K2_DEATH_MOMENTUM: the whole motion carried, no cap, no drag.
    TEST(K2Lifecycle, TheDeathMomentumKeepsTheBodysRealMotion)
    {
        DeathMotion old;
        const std::array<float, 3> off{ 0.0f, 0.0f, 0.0f };
        const DeathMotion kept = deathMotion(old, off);
        EXPECT_FLOAT_EQ(kept.mCarry.mShare, 0.15f);
        EXPECT_FLOAT_EQ(kept.mDragRate, 3.0f);
        const std::array<float, 3> real{ 1.0f, 0.0f, 0.0f };
        const DeathMotion motion = deathMotion(old, real);
        EXPECT_EQ(deathDragSteps(motion.mDragRate, motion.mDragSeconds, sDt), 0);
        const std::vector<JPH::Vec3> parts{ JPH::Vec3(200, 0, 0), JPH::Vec3(260, 30, -40) };
        const DeathCarry carry = deathCarry(parts, motion.mCarry);
        EXPECT_TRUE(carry.linear(parts[0]).IsClose(parts[0], 1e-4f));
        EXPECT_TRUE(carry.linear(parts[1]).IsClose(parts[1], 1e-4f));
        EXPECT_TRUE(carry.angular(JPH::Vec3(10, 0, 0)).IsClose(JPH::Vec3(10, 0, 0)));
        // A cap still caps the mean only.
        const std::array<float, 3> capped{ 1.0f, 100.0f, 0.0f };
        const DeathCarry under = deathCarry(parts, deathMotion(old, capped).mCarry);
        EXPECT_NEAR(under.mCommon.Length(), 100.0f, 1e-3f);
    }

    // K2_DEATH_JOINTS: the living frames only for a carried corpse; no blend then.
    TEST(K2Lifecycle, ACarriedCorpseKeepsTheLivingJointsAndSkipsTheBlend)
    {
        CorpseBuild build = corpseBuild(true, true, true, 0.08f);
        EXPECT_TRUE(build.mLivingFrames);
        EXPECT_FLOAT_EQ(build.mBlendSeconds, 0.0f);
        build = corpseBuild(true, true, false, 0.08f);
        EXPECT_FALSE(build.mLivingFrames);
        EXPECT_FLOAT_EQ(build.mBlendSeconds, 0.0f);
        build = corpseBuild(true, false, true, 0.08f);
        EXPECT_FALSE(build.mLivingFrames);
        EXPECT_FLOAT_EQ(build.mBlendSeconds, 0.08f);
        build = corpseBuild(false, true, true, 0.08f);
        EXPECT_FALSE(build.mLivingFrames);
        EXPECT_FLOAT_EQ(build.mBlendSeconds, 0.08f);
    }

    // K2_DEATH_LIMITS: a joint carried over outside the corpse's range starts
    // inside an opened one, which closes to the authored range.
    TEST(K2Lifecycle, AJointOutsideItsCorpseRangeIsHeldAndClosedOverTime)
    {
        // A weapon hand's wrist, cocked 1.5 rad about the plane axis and
        // twisted 1.0 rad, against the authored 0.45/0.35 cones and 0.30 twist.
        const JointRange wrist{ false, 0.45f, 0.35f, -0.30f, 0.30f };
        JointAt cocked;
        cocked.mSwingY = std::sin(0.75f);
        cocked.mTwist = 1.0f;
        EXPECT_NEAR(jointExcess(wrist, cocked), 1.5f - 0.35f, 1e-3f);
        const JointRange opened = deathRangeOpened(wrist, cocked);
        EXPECT_FLOAT_EQ(jointExcess(opened, cocked), 0.0f);
        EXPECT_GT(opened.mPlaneHalfCone, 1.5f);
        EXPECT_GT(opened.mTwistMax, 1.0f);
        EXPECT_FLOAT_EQ(opened.mTwistMin, wrist.mTwistMin);
        // Closing: the opened range first, the authored one at the end.
        const JointRange start = deathRangeEased(wrist, opened, 0.0f);
        EXPECT_FLOAT_EQ(start.mPlaneHalfCone, opened.mPlaneHalfCone);
        const JointRange half = deathRangeEased(wrist, opened, 0.5f);
        EXPECT_NEAR(half.mPlaneHalfCone, 0.5f * (opened.mPlaneHalfCone + wrist.mPlaneHalfCone), 1e-5f);
        const JointRange end = deathRangeEased(wrist, opened, 3.0f);
        EXPECT_FLOAT_EQ(end.mPlaneHalfCone, wrist.mPlaneHalfCone);
        EXPECT_FLOAT_EQ(end.mNormalHalfCone, wrist.mNormalHalfCone);
        EXPECT_FLOAT_EQ(end.mTwistMax, wrist.mTwistMax);

        // A joint inside its range is left as authored, whichever way it points.
        JointAt relaxed;
        relaxed.mSwingY = std::sin(0.1f);
        relaxed.mSwingZ = std::sin(0.1f);
        relaxed.mTwist = -0.2f;
        EXPECT_FLOAT_EQ(jointExcess(wrist, relaxed), 0.0f);
        const JointRange same = deathRangeOpened(wrist, relaxed);
        EXPECT_FLOAT_EQ(same.mNormalHalfCone, wrist.mNormalHalfCone);
        EXPECT_FLOAT_EQ(same.mPlaneHalfCone, wrist.mPlaneHalfCone);
        EXPECT_FLOAT_EQ(same.mTwistMin, wrist.mTwistMin);

        // A hinge past its end.
        JointRange elbow;
        elbow.mHinge = true;
        elbow.mHingeMin = 0.0f;
        elbow.mHingeMax = 2.4f;
        JointAt bent;
        bent.mHinge = -0.5f;
        EXPECT_NEAR(jointExcess(elbow, bent), 0.5f, 1e-5f);
        const JointRange elbowOpened = deathRangeOpened(elbow, bent);
        EXPECT_LT(elbowOpened.mHingeMin, -0.5f);
        EXPECT_FLOAT_EQ(elbowOpened.mHingeMax, 2.4f);
        EXPECT_FLOAT_EQ(jointExcess(elbowOpened, bent), 0.0f);
        EXPECT_FLOAT_EQ(deathRangeEased(elbow, elbowOpened, 1.0f).mHingeMin, 0.0f);
    }

    TEST(K2Lifecycle, TheDeathDragDrainsOnlyTheHorizontal)
    {
        EXPECT_NEAR(deathDragSteps(3.0f, 2.5f, sDt), 150, 1);
        EXPECT_EQ(deathDragSteps(0.0f, 2.5f, sDt), 0);
        const JPH::Vec3 drain = deathDragDrain(JPH::Vec3(60, 0, -100), 3.0f, sDt);
        EXPECT_NEAR(drain.GetX(), 60.0f * (1.0f - std::exp(-3.0f * sDt)), 1e-4f);
        EXPECT_FLOAT_EQ(drain.GetZ(), 0.0f);
    }

    TEST(K2Lifecycle, TheActorFollowsItsCorpseOverTheFloor)
    {
        const JPH::RVec3 reference(0, 0, 0);
        auto next = corpseFollowTarget(JPH::RVec3(30, 5, 10), reference, JPH::Real(-2), JPH::RVec3(0, 0, 0));
        ASSERT_TRUE(next);
        EXPECT_NEAR(next->GetZ(), -2.0, 1e-6);
        next = corpseFollowTarget(JPH::RVec3(30, 5, 10), reference, std::nullopt, JPH::RVec3(0, 0, 0));
        ASSERT_TRUE(next);
        EXPECT_NEAR(next->GetZ(), 0.0, 1e-6);
        EXPECT_FALSE(corpseFollowTarget(JPH::RVec3(0.1, 0, 5), reference, JPH::Real(0), JPH::RVec3(0.1, 0, 0)));
    }

    TEST(K2Lifecycle, ALoadedCorpseLiesOnItsBack)
    {
        EXPECT_FALSE(corpseLoadPose(0.0f, JPH::RVec3(0, 0, 0), JPH::RVec3(0, 0, 20), JPH::Real(0)));
        const auto pose = corpseLoadPose(0.0f, JPH::RVec3(5, 6, 0), JPH::RVec3(5, 6, 70), JPH::Real(0));
        ASSERT_TRUE(pose);
        // Facing +y: the body's up turns to point back (-y).
        EXPECT_TRUE((pose->mTurn * JPH::Vec3::sAxisZ()).IsClose(JPH::Vec3(0, -1, 0), 1e-6f));
        const JPH::RVec3 head = pose->place(JPH::RVec3(5, 6, 100), JPH::RVec3(5, 6, 70));
        EXPECT_NEAR(head.GetY(), 6.0 - 30.0, 1e-3);
        EXPECT_NEAR(head.GetZ(), 8.0, 1e-3);
    }

    // A pelvis, a chest and two three-bone legs; the feet 8 u to either side
    // under the pelvis (standing) unless `stride` puts the left one ahead.
    struct Rig
    {
        RagdollProfile mProfile;
        SkeletonDescriptor mSkeleton;
    };
    Rig rig(float stride)
    {
        Rig r;
        const auto body = [&](const std::string& name, std::optional<std::string> parent) {
            BodyProfile b;
            b.mName = name;
            b.mBone = "b_" + name;
            b.mParentBody = std::move(parent);
            r.mProfile.mBodies.push_back(b);
        };
        body("pelvis", std::nullopt);
        body("chest", "pelvis");
        for (const std::string side : { "l", "r" })
        {
            body(side + "thigh", "pelvis");
            body(side + "calf", side + "thigh");
            body(side + "foot", side + "calf");
        }
        r.mProfile.mLandmarks.mPelvis = "pelvis";
        r.mProfile.mLandmarks.mSpine = { "chest" };
        r.mProfile.mLandmarks.mLeftFoot = "lfoot";
        r.mProfile.mLandmarks.mRightFoot = "rfoot";
        r.mSkeleton.mId = "rig";
        const auto bone = [&](const std::string& name, JPH::Vec3 at) {
            r.mSkeleton.mBones.push_back({ "b_" + name, 0, { at, JPH::Quat::sIdentity() } });
        };
        bone("pelvis", JPH::Vec3(0, 0, 0));
        bone("chest", JPH::Vec3(0, 2, 30));
        bone("lthigh", JPH::Vec3(-8, 0, 0));
        bone("lcalf", JPH::Vec3(-8, stride * 0.5f, -30));
        bone("lfoot", JPH::Vec3(-8, stride, -60));
        bone("rthigh", JPH::Vec3(8, 0, 0));
        bone("rcalf", JPH::Vec3(8, 0, -30));
        bone("rfoot", JPH::Vec3(8, 0, -60));
        bone("lhand", JPH::Vec3(-20, 0, 20));
        return r;
    }

    TEST(K2Lifecycle, BuildFramesComeFromAStandingPose)
    {
        const Rig standing = rig(0.0f);
        const BuildFrames frames = buildFrames(standing.mProfile, standing.mProfile, standing.mSkeleton, true);
        EXPECT_TRUE(frames.mStanding);
        EXPECT_EQ(frames.mFrames.mBones.size(), 8u); // not the hand
        for (const auto& bone : frames.mFrames.mBones)
            EXPECT_EQ(bone.mParent, -1);
        EXPECT_NE(frames.mKey.find(":30,30:30,30"), std::string::npos) << frames.mKey;
        EXPECT_FALSE(buildFrames(standing.mProfile, standing.mProfile, standing.mSkeleton, false).mStanding);
        // A stride: not standing; the key's bone list is the same (this rig
        // stretches its leg, so only its lengths differ).
        const Rig running = rig(27.0f);
        const BuildFrames stride = buildFrames(running.mProfile, running.mProfile, running.mSkeleton, true);
        EXPECT_FALSE(stride.mStanding);
        EXPECT_EQ(stride.mKey.substr(0, stride.mKey.find(':')), frames.mKey.substr(0, frames.mKey.find(':')));
    }
}
