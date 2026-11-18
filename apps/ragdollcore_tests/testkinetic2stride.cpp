// KINETIC-2 phase 2: ragdollcore stride, the legs' decisions moved from the
// old integration (the catches in the step order, the planner's aim, the
// legs lead, the legs' health, the swing spring, the braced legs).
#include <ragdollcore/stride.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
    using namespace RagdollCore;

    constexpr float sDt = 1.0f / 60.0f;

    struct StandingBody
    {
        FootfallInput mFootfall;
        StrideCatchInput mInput;

        StandingBody()
        {
            mFootfall.mDt = sDt;
            mFootfall.mEligible = true;
            HingeLegGeometry geometry;
            geometry.mUpperOffset = JPH::Vec3(0.0f, 0.0f, -30.0f);
            geometry.mLowerOffset = JPH::Vec3(0.0f, 0.0f, -30.0f);
            mFootfall.mLegs[0] = FootfallLeg{ 1, 2, 3, geometry };
            mFootfall.mLegs[1] = FootfallLeg{ 4, 5, 6, geometry };
            mFootfall.mGround = [](const JPH::RVec3&, double) -> std::optional<double> { return 0.0; };
            mInput.mFootfall = &mFootfall;
            mInput.mPelvisKnown = true;
            mInput.mPelvis = JPH::RVec3(0.0, 0.0, 40.0);
            mInput.mFootWorld[0] = JPH::RVec3(-6.0, 0.0, 3.0);
            mInput.mFootWorld[1] = JPH::RVec3(6.0, 0.0, 3.0);
            mInput.mHaveBody = true;
        }
    };

    TEST(K2Stride, NoCatchOnMeansNoStepAndTheRecoveryEnds)
    {
        StandingBody body;
        StrideCatchSettings settings;
        settings.mLanding = false;
        StrideCatchState state;
        state.mRecovering = true;
        const StrideCatchResult result = stepStrideCatches(body.mInput, settings, state);
        EXPECT_FALSE(result.mSource);
        EXPECT_FALSE(result.mCommand);
        EXPECT_FALSE(state.mRecovering);
    }

    TEST(K2Stride, StandingCatchOwnsAStillUprightBody)
    {
        StandingBody body;
        StrideCatchSettings settings;
        settings.mStandingCatch = true;
        StrideCatchState state;
        const StrideCatchResult result = stepStrideCatches(body.mInput, settings, state);
        ASSERT_TRUE(result.mSource);
        EXPECT_EQ(*result.mSource, StepSource::StandingCatch);
        EXPECT_FALSE(result.mFall);
        ASSERT_EQ(result.mTraces.size(), 1u);
        EXPECT_STREQ(result.mTraces[0].mBranch, "catch");
    }

    TEST(K2Stride, StuckLeanStepsUnderTheBodyThenFalls)
    {
        StandingBody body;
        body.mInput.mTorsoUpDot = 0.6f;
        StrideCatchSettings settings;
        settings.mStandingCatch = true;
        settings.mStuck = { 0.75f, 0.8f, 2.0f };
        StrideCatchState state;
        std::optional<StrideCatchResult> stepped;
        int steps = 0;
        for (; steps < 200 && !stepped; ++steps)
        {
            StrideCatchResult result = stepStrideCatches(body.mInput, settings, state);
            if (result.mSource == StepSource::Stuck)
                stepped = result;
        }
        ASSERT_TRUE(stepped);
        EXPECT_NEAR(steps * sDt, 0.8f, 2.0f * sDt);
        ASSERT_TRUE(stepped->mCommand);
        EXPECT_TRUE(stepped->mCommand->mHoldClock);
        ASSERT_TRUE(stepped->mCommand->mSwingNow);
        EXPECT_EQ(*stepped->mCommand->mSwingNow, 0u); // equal distances: the left
        ASSERT_TRUE(stepped->mCommand->mLanding[0]);
        // Under the body, a half stance width to the foot's own side.
        EXPECT_NEAR(stepped->mCommand->mLanding[0]->GetX(), 0.0, 1e-3);
        EXPECT_NEAR(std::abs(stepped->mCommand->mLanding[0]->GetY()), 6.0, 1e-3);
        EXPECT_EQ(state.mStuckSteps, 1u);
        // Still leaning: one step only, then the fall at 2 s.
        std::optional<StrideCatchResult> fell;
        for (int i = 0; i < 200 && !fell; ++i)
        {
            StrideCatchResult result = stepStrideCatches(body.mInput, settings, state);
            EXPECT_FALSE(result.mStuckStepFoot) << "no second stuck step before the fall";
            if (result.mFall)
                fell = result;
        }
        ASSERT_TRUE(fell);
        // The stuck rule owns the step it decides the fall on.
        EXPECT_EQ(fell->mSource, std::optional<StepSource>(StepSource::Stuck));
        EXPECT_STREQ(fell->mFall->mCause, "stuck");
        EXPECT_EQ(state.mStuckFalls, 1u);
        EXPECT_FLOAT_EQ(state.mStuckLeanTime, 0.0f);
    }

    TEST(K2Stride, StepOrderDecidesWhichCatchOwnsTheStep)
    {
        StandingBody body;
        StrideCatchSettings settings;
        settings.mStandingCatch = true;
        settings.mCatchSteps = true;
        // The stage catch first: a still, upright body is not in its Step
        // stage, so it passes, and the standing catch takes the step.
        settings.mOrder = { StepSource::StageCatch, StepSource::Stuck, StepSource::Landing, StepSource::MovingCatch,
            StepSource::StandingCatch };
        StrideCatchState state;
        const StrideCatchResult result = stepStrideCatches(body.mInput, settings, state);
        ASSERT_TRUE(result.mSource);
        EXPECT_EQ(*result.mSource, StepSource::StandingCatch);
        ASSERT_EQ(result.mTraces.size(), 2u);
        EXPECT_STREQ(result.mTraces[0].mBranch, "hitcatch");
        EXPECT_STREQ(result.mTraces[1].mBranch, "catch");
    }

    TEST(K2Stride, ATouchdownIsConsumedByTheLandingCatch)
    {
        StandingBody body;
        StrideCatchSettings settings;
        StrideCatchState state;
        state.mLandingTouchdown = true;
        state.mLandingImpactSpeed = 300.0f;
        const StrideCatchResult result = stepStrideCatches(body.mInput, settings, state);
        ASSERT_TRUE(result.mSource);
        EXPECT_EQ(*result.mSource, StepSource::Landing);
        EXPECT_FALSE(state.mLandingTouchdown);
    }

    TEST(K2Stride, TheAimIsOffUnlessAsked)
    {
        StrideAimInput input;
        input.mEligible = true;
        input.mCommandVelocity = JPH::Vec3(100.0f, 0.0f, 0.0f);
        EXPECT_FALSE(aimStrideLandings(input));
    }

    StrideLeadInput walkingLead(float bodySpeed)
    {
        StrideLeadInput input;
        input.mEligible = true;
        input.mGateOpen = true;
        input.mRequest = JPH::Vec3(0.0f, 120.0f, 0.0f);
        input.mBodyVelocity = JPH::Vec3(0.0f, bodySpeed, 0.0f);
        input.mPlantedFeet[0] = BalanceVec2{ -6.0f, 0.0f };
        input.mPlantedFeet[1] = BalanceVec2{ 6.0f, 0.0f };
        input.mContacts = { { 0, 3000.0f, 0.9f }, { 1, 3000.0f, 0.9f } };
        input.mTotalMass = 60.0f;
        input.mTelemetry = true;
        return input;
    }

    TEST(K2Stride, TheLegsLeadOnlyOnceCruisingAndGiveBackPastTwiceTheBand)
    {
        StrideLeadSettings settings;
        settings.mLead = { 1.0f, 0.0f, 0.45f, 10.0f };
        settings.mGrip = { 0.6f, 0.25f, 150.0f };
        StrideLeadState state;
        // 50% off the request: the capsule leads.
        StrideLeadResult r = stepStrideLead(walkingLead(60.0f), settings, state);
        EXPECT_FALSE(r.mLeading);
        EXPECT_FALSE(state.mCruising);
        // Within 25%: the legs take over and push.
        r = stepStrideLead(walkingLead(100.0f), settings, state);
        EXPECT_TRUE(r.mLeading);
        EXPECT_TRUE(r.mChanged);
        EXPECT_TRUE(state.mCruising);
        ASSERT_TRUE(r.mForce);
        EXPECT_GT(r.mForce->mBodyForce.mY, 0.0f);
        // Body + feet forces cancel: an internal pair.
        EXPECT_NEAR(r.mForce->mBodyForce.mY + r.mForce->mFootForce[0].mY + r.mForce->mFootForce[1].mY, 0.0f, 1e-3f);
        // 40% off: still inside twice the band, still leading.
        r = stepStrideLead(walkingLead(72.0f), settings, state);
        EXPECT_TRUE(r.mLeading);
        EXPECT_FALSE(r.mChanged);
        // 60% off: given back.
        r = stepStrideLead(walkingLead(48.0f), settings, state);
        EXPECT_FALSE(r.mLeading);
        EXPECT_TRUE(r.mChanged);
        EXPECT_FALSE(state.mCruising);
    }

    TEST(K2Stride, TheLegsLeadNeverOnAClosedGateOrAboveTheSpeedCap)
    {
        StrideLeadSettings settings;
        settings.mLead = { 1.0f, 0.0f, 0.45f, 10.0f };
        settings.mGrip = { 0.6f, 0.25f, 150.0f };
        StrideLeadState state;
        StrideLeadInput closed = walkingLead(120.0f);
        closed.mGateOpen = false;
        EXPECT_FALSE(stepStrideLead(closed, settings, state).mLeading);
        StrideLeadInput fast = walkingLead(200.0f);
        fast.mRequest = JPH::Vec3(0.0f, 200.0f, 0.0f);
        EXPECT_FALSE(stepStrideLead(fast, settings, state).mLeading);
    }

    StrideSpringFoot swingingFoot()
    {
        StrideSpringFoot foot;
        foot.mSwinging = true;
        foot.mValid = true;
        foot.mFoot = { 0.0f, 0.0f };
        foot.mLiftOff = { 0.0f, -30.0f };
        foot.mLanding = { 0.0f, 30.0f };
        foot.mPhase = 0.5f;
        foot.mDuration = 0.35f;
        foot.mLegMass = 12.0f;
        foot.mFootMass = 2.0f;
        return foot;
    }

    TEST(K2Stride, TheSwingSpringPullsOnlySwingingFeetAsAnInternalPair)
    {
        StrideSpringInput input;
        input.mFeet[0] = swingingFoot();
        input.mFeet[1] = swingingFoot();
        input.mFeet[1].mSwinging = false;
        input.mFeet[0].mFoot = { 0.0f, -10.0f }; // behind its eased path
        std::array<JPH::Vec3, 2> velocity{ JPH::Vec3::sZero(), JPH::Vec3::sZero() };
        const auto out = stepStrideSwingSpring(input, velocity);
        ASSERT_TRUE(out[0]);
        EXPECT_FALSE(out[1]);
        EXPECT_GT(out[0]->mFootForce.mY, 0.0f);
        EXPECT_NEAR(out[0]->mFootForce.mY + out[0]->mPelvisForce.mY, 0.0f, 1e-4f);
    }

    TEST(K2Stride, AScopedSwingSpringFiltersAndCapsTheGoalVelocity)
    {
        StrideSpringInput input;
        input.mFeet[0] = swingingFoot();
        input.mFeet[0].mGoal = JPH::RVec3(0.0, 5.0, 0.0);
        input.mFeet[0].mGoalVelocity = JPH::Vec3(0.0f, 300.0f, 50.0f);
        input.mScoped = true;
        input.mScopedDt = sDt;
        input.mPelvisMass = 10.0f;
        std::array<JPH::Vec3, 2> velocity{ JPH::Vec3::sZero(), JPH::Vec3::sZero() };
        stepStrideSwingSpring(input, velocity);
        // One step of a 0.03 s filter: 1 - e^(-0.556) of the way.
        EXPECT_NEAR(velocity[0].GetY(), 300.0f * (1.0f - std::exp(-sDt / 0.03f)), 0.1f);
        EXPECT_FLOAT_EQ(velocity[0].GetZ(), 0.0f);
        input.mFeet[0].mGoalVelocity = JPH::Vec3(0.0f, 2000.0f, 0.0f);
        for (int i = 0; i < 20; ++i)
            stepStrideSwingSpring(input, velocity);
        EXPECT_NEAR(velocity[0].Length(), 400.0f, 1e-2f);
        // A swing starting zeroes it; not scoped, it is zero.
        input.mFeet[0].mSwingStarted = true;
        stepStrideSwingSpring(input, velocity);
        EXPECT_FLOAT_EQ(velocity[0].Length(), 0.0f);
        velocity[0] = JPH::Vec3(10.0f, 0.0f, 0.0f);
        input.mScoped = false;
        stepStrideSwingSpring(input, velocity);
        EXPECT_FLOAT_EQ(velocity[0].Length(), 0.0f);
    }

    TEST(K2Stride, ABracedBodyHoldsItsPelvisAndIsPushedOverItsFeet)
    {
        StandingBody body;
        StrideBraceInput input;
        input.mFootfall = &body.mFootfall;
        input.mDt = sDt;
        input.mFeetCom = { BalanceVec2{ -6.0f, 0.0f }, BalanceVec2{ 6.0f, 0.0f } };
        input.mSupported = { true, true };
        input.mCenterOfMass = JPH::RVec3(4.0, 0.0, 40.0); // off the middle of the feet
        StrideBraceSettings settings;
        StrideBraceState state;
        const StrideBraceResult result = stepStrideBrace(input, settings, state);
        ASSERT_TRUE(result.mCommand);
        EXPECT_TRUE(result.mCommand->mStanceHoldsPelvis);
        EXPECT_TRUE(result.mCommand->mNoAnchorSettle);
        EXPECT_FALSE(result.mFall);
        EXPECT_STREQ(result.mTrace.mBranch, "brace");
        const auto push = strideBraceStance(result.mBody, input.mFeetCom, input.mSupported, input.mFeet, 60.0f);
        ASSERT_TRUE(push);
        EXPECT_LT(push->mBodyForce.mX, 0.0f); // back toward the middle
        EXPECT_NEAR(push->mBodyForce.mX + push->mFootForce[0].mX + push->mFootForce[1].mX, 0.0f, 1e-3f);
        // A swinging foot is not planted.
        std::array<FootfallController::Foot, 2> oneSwinging = input.mFeet;
        oneSwinging[0].mSwinging = true;
        const auto onOne = strideBraceStance(result.mBody, input.mFeetCom, input.mSupported, oneSwinging, 60.0f);
        if (onOne)
            EXPECT_FLOAT_EQ(onOne->mFootForce[0].mX, 0.0f);
    }

    TEST(K2Stride, TheLegsFailWhenTheyKeepSteppingStandingStill)
    {
        const std::array<float, 2> settings{ 1.5f, 4.0f };
        StrideHealthState state;
        StrideHealthInput still;
        still.mDt = sDt;
        for (int i = 0; i < 90; ++i)
            EXPECT_FALSE(stepStrideHealth(still, settings, state));
        StrideHealthInput stepping = still;
        stepping.mSwingStarted = { true, false };
        EXPECT_FALSE(stepStrideHealth(stepping, settings, state));
        EXPECT_FALSE(stepStrideHealth(stepping, settings, state));
        EXPECT_FALSE(stepStrideHealth(stepping, settings, state));
        EXPECT_TRUE(stepStrideHealth(stepping, settings, state));
        EXPECT_TRUE(state.mFailing);
        // Latched: no second report.
        EXPECT_FALSE(stepStrideHealth(stepping, settings, state));
    }

    TEST(K2Stride, MovingResetsTheLegsHealthCount)
    {
        const std::array<float, 2> settings{ 1.5f, 4.0f };
        StrideHealthState state;
        StrideHealthInput still;
        still.mDt = sDt;
        for (int i = 0; i < 100; ++i)
            stepStrideHealth(still, settings, state);
        StrideHealthInput moving = still;
        moving.mRequestSpeed = 50.0f;
        stepStrideHealth(moving, settings, state);
        EXPECT_FLOAT_EQ(state.mStillTime, 0.0f);
        EXPECT_EQ(state.mStillSteps, 0u);
        EXPECT_FALSE(stepStrideHealth(still, { 0.0f, 4.0f }, state)) << "grace 0 is off";
    }
}
