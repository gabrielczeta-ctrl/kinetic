#include <ragdollcore/recoverycontrol.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
    using namespace RagdollCore;

    constexpr float sGravity = 627.0f;
    constexpr float sMass = 80.0f;

    float length(BalanceVec2 v) { return std::hypot(v.mX, v.mY); }

    StanceForceInput standingOnTwoFeet()
    {
        StanceForceInput input;
        input.mPlantedFeet = { BalanceVec2{ -8.0f, 0.0f }, BalanceVec2{ 8.0f, 0.0f } };
        input.mTotalMass = sMass;
        input.mGravity = sGravity;
        return input;
    }

    TEST(RagdollCoreRecoveryControl, capabilityGrowsWithSkill)
    {
        const CatchCapability low = catchCapability({ 10.0f, 10.0f });
        const CatchCapability high = catchCapability({ 90.0f, 90.0f });
        EXPECT_GT(high.mReachFraction, low.mReachFraction);
        EXPECT_LT(high.mMinimumSwingTime, low.mMinimumSwingTime);
        EXPECT_GT(high.mSwingFootSpeed, low.mSwingFootSpeed);
        EXPECT_LT(high.mReactionDelay, low.mReactionDelay);
        EXPECT_GT(high.mSwingSpringFrequency, low.mSwingSpringFrequency);
        EXPECT_GE(high.mMaximumSwingTime, high.mMinimumSwingTime + 0.05f - 1e-6f);
        // The measured ends: agility 0 and 100 alone.
        EXPECT_NEAR(catchCapability({ 100.0f, 100.0f }).mReachFraction, 1.1f, 1e-5f);
        EXPECT_NEAR(catchCapability({ 0.0f, 0.0f }).mReactionDelay, 0.15f, 1e-5f);
    }

    TEST(RagdollCoreRecoveryControl, capabilityClampsStats)
    {
        const CatchCapability over = catchCapability({ 250.0f, 250.0f });
        const CatchCapability top = catchCapability({ 100.0f, 100.0f });
        EXPECT_FLOAT_EQ(over.mReachFraction, top.mReachFraction);
        const CatchCapability nan = catchCapability({ std::nanf(""), -5.0f });
        EXPECT_FLOAT_EQ(nan.mReachFraction, catchCapability({ 0.0f, 0.0f }).mReachFraction);
    }

    TEST(RagdollCoreRecoveryControl, stepLimitsFollowCapability)
    {
        const CatchCapability c = catchCapability({ 50.0f, 50.0f });
        const CaptureStepLimits limits = catchStepLimits(c, 70.0f);
        EXPECT_NEAR(limits.mMaximumReach, c.mReachFraction * 70.0f, 1e-4f);
        EXPECT_FLOAT_EQ(limits.mMinimumSwingTime, c.mMinimumSwingTime);
        EXPECT_FLOAT_EQ(limits.mSwingFootSpeed, c.mSwingFootSpeed);
        EXPECT_FLOAT_EQ(limits.mCrossoverClearance, 7.0f);
    }

    TEST(RagdollCoreRecoveryControl, stanceForceIsAnInternalPair)
    {
        StanceForceInput input = standingOnTwoFeet();
        input.mCenterOfMass = { 3.0f, 2.0f };
        input.mVelocity = { 5.0f, -4.0f };
        const auto out = computeStanceForce(input);
        ASSERT_TRUE(out);
        ASSERT_TRUE(out->mActive);
        const BalanceVec2 sum{ out->mBodyForce.mX + out->mFootForce[0].mX + out->mFootForce[1].mX,
            out->mBodyForce.mY + out->mFootForce[0].mY + out->mFootForce[1].mY };
        EXPECT_NEAR(length(sum), 0.0f, 1e-3f);
    }

    TEST(RagdollCoreRecoveryControl, stanceForcePullsTowardTheFeetAndBrakes)
    {
        StanceForceInput input = standingOnTwoFeet();
        input.mCenterOfMass = { 0.0f, 3.0f }; // ahead of the feet
        const auto held = computeStanceForce(input);
        ASSERT_TRUE(held);
        EXPECT_LT(held->mBodyForce.mY, 0.0f);
        input.mCenterOfMass = { 0.0f, 0.0f };
        input.mVelocity = { 10.0f, 0.0f };
        const auto braked = computeStanceForce(input);
        ASSERT_TRUE(braked);
        EXPECT_LT(braked->mBodyForce.mX, 0.0f);
    }

    TEST(RagdollCoreRecoveryControl, stanceForceStaysInsideTheFrictionCone)
    {
        StanceForceInput input = standingOnTwoFeet();
        input.mVelocity = { 300.0f, 0.0f };
        const auto out = computeStanceForce(input);
        ASSERT_TRUE(out);
        EXPECT_TRUE(out->mCapped);
        EXPECT_NEAR(length(out->mBodyForce), input.mSettings.mFrictionShare * sMass * sGravity, 1e-2f);
    }

    TEST(RagdollCoreRecoveryControl, stanceForceTracksTheRequestWhileMoving)
    {
        StanceForceInput input = standingOnTwoFeet();
        input.mCenterOfMass = { 0.0f, 20.0f }; // far ahead: ignored while moving
        input.mRequestVelocity = { 0.0f, 128.0f };
        input.mVelocity = { 0.0f, 128.0f };
        const auto out = computeStanceForce(input);
        ASSERT_TRUE(out);
        EXPECT_NEAR(length(out->mBodyForce), 0.0f, 1e-3f);
    }

    TEST(RagdollCoreRecoveryControl, stanceForceNeedsAPlantedFoot)
    {
        StanceForceInput input = standingOnTwoFeet();
        input.mPlantedFeet = {};
        const auto out = computeStanceForce(input);
        ASSERT_TRUE(out);
        EXPECT_FALSE(out->mActive);
        input.mTotalMass = 0.0f;
        EXPECT_FALSE(computeStanceForce(input));
    }

    TEST(RagdollCoreRecoveryControl, stanceForceShiftsTheWeightTarget)
    {
        StanceForceInput input = standingOnTwoFeet();
        input.mCenterShift = { 0.0f, 5.0f }; // lean forward over the toes
        const auto out = computeStanceForce(input);
        ASSERT_TRUE(out);
        EXPECT_GT(out->mBodyForce.mY, 0.0f);
    }

    TEST(RagdollCoreRecoveryControl, stanceForceSplitsOverPlantedFeet)
    {
        StanceForceInput input = standingOnTwoFeet();
        input.mPlantedFeet[1].reset();
        input.mVelocity = { 10.0f, 0.0f };
        const auto out = computeStanceForce(input);
        ASSERT_TRUE(out);
        EXPECT_NEAR(out->mFootForce[0].mX, -out->mBodyForce.mX, 1e-3f);
        EXPECT_FLOAT_EQ(out->mFootForce[1].mX, 0.0f);
    }

    SwingSpringInput swingHalfway()
    {
        SwingSpringInput input;
        input.mLiftOff = { 0.0f, 0.0f };
        input.mLanding = { 0.0f, 40.0f };
        input.mPhase = 0.5f;
        input.mDuration = 0.3f;
        input.mLegMass = 12.0f;
        input.mGravity = sGravity;
        return input;
    }

    TEST(RagdollCoreRecoveryControl, swingSpringPullsTheFootAlongThePath)
    {
        SwingSpringInput input = swingHalfway();
        input.mFoot = { 0.0f, 5.0f }; // behind the path (goal 20 at half phase)
        const auto out = computeSwingSpring(input);
        ASSERT_TRUE(out);
        EXPECT_GT(out->mFootForce.mY, 0.0f);
        EXPECT_FLOAT_EQ(out->mPelvisForce.mY, -out->mFootForce.mY);
    }

    TEST(RagdollCoreRecoveryControl, swingSpringIsCapped)
    {
        SwingSpringInput input = swingHalfway();
        input.mFoot = { 0.0f, -500.0f };
        const auto out = computeSwingSpring(input);
        ASSERT_TRUE(out);
        EXPECT_TRUE(out->mCapped);
        EXPECT_NEAR(length(out->mFootForce), input.mSettings.mCapLegWeights * input.mLegMass * sGravity, 1e-1f);
    }

    TEST(RagdollCoreRecoveryControl, swingSpringRestsOnTheLanding)
    {
        SwingSpringInput input = swingHalfway();
        input.mPhase = 1.0f;
        input.mFoot = input.mLanding;
        const auto out = computeSwingSpring(input);
        ASSERT_TRUE(out);
        EXPECT_NEAR(length(out->mFootForce), 0.0f, 1e-3f);
        input.mDuration = 0.0f;
        EXPECT_FALSE(computeSwingSpring(input));
    }

    CatchStepInput catchInput(BalanceVec2 velocity, float reactionDelay)
    {
        CatchStepInput input;
        input.mState.mCenterOfMass = { 0.0f, 0.0f };
        input.mState.mVelocity = velocity;
        input.mState.mHeight = 77.0f;
        input.mState.mGravity = sGravity;
        input.mFeet[0].mPosition = { -8.0f, 0.0f };
        input.mFeet[1].mPosition = { 8.0f, 0.0f };
        input.mFeet[0].mSupported = input.mFeet[1].mSupported = true;
        input.mDt = 1.0f / 60.0f;
        input.mLimits = catchStepLimits(catchCapability({ 50.0f, 50.0f }), 70.0f);
        input.mReactionDelay = reactionDelay;
        return input;
    }

    TEST(RagdollCoreRecoveryControl, catchStaysOffWhenBalanced)
    {
        CatchStepController controller;
        const auto command = controller.update(catchInput({ 0.0f, 0.0f }, 0.0f));
        ASSERT_TRUE(command);
        EXPECT_FALSE(command->mHoldClock);
        EXPECT_FALSE(command->mSwingNow);
        EXPECT_FALSE(controller.catching());
    }

    TEST(RagdollCoreRecoveryControl, catchStepsAfterTheReactionDelay)
    {
        CatchStepController controller;
        const CatchStepInput input = catchInput({ 0.0f, 60.0f }, 0.1f);
        auto command = controller.update(input);
        ASSERT_TRUE(command);
        EXPECT_EQ(command->mStage, BalanceStage::Step);
        EXPECT_FALSE(command->mSwingNow); // still reacting
        std::optional<CatchStepCommand> stepped;
        for (int i = 0; i < 10 && !stepped; ++i)
        {
            command = controller.update(input);
            ASSERT_TRUE(command);
            if (command->mSwingNow)
                stepped = command;
        }
        ASSERT_TRUE(stepped);
        EXPECT_TRUE(stepped->mHoldClock);
        ASSERT_TRUE(stepped->mLanding[*stepped->mSwingNow]);
        EXPECT_GT(stepped->mLanding[*stepped->mSwingNow]->mY, 10.0f); // forward, along the fall
        EXPECT_EQ(controller.stepsStarted(), 1u);
    }

    TEST(RagdollCoreRecoveryControl, catchReaimsASwingInFlightOnce)
    {
        CatchStepController controller;
        CatchStepInput input = catchInput({ 0.0f, 60.0f }, 0.0f);
        ASSERT_TRUE(controller.update(input));
        input.mFeet[1].mSwinging = true;
        input.mFeet[1].mSupported = false;
        input.mFeet[1].mPhase = 0.3f;
        const auto inFlight = controller.update(input);
        ASSERT_TRUE(inFlight);
        EXPECT_FALSE(inFlight->mSwingNow);
        EXPECT_TRUE(inFlight->mLanding[1]);
        input.mFeet[1].mTargetHeld = true;
        const auto held = controller.update(input);
        ASSERT_TRUE(held);
        EXPECT_FALSE(held->mLanding[1]);
    }

    TEST(RagdollCoreRecoveryControl, catchEndsWhenCaught)
    {
        CatchStepController controller;
        ASSERT_TRUE(controller.update(catchInput({ 0.0f, 60.0f }, 0.0f)));
        ASSERT_TRUE(controller.catching());
        const CatchStepInput calm = catchInput({ 0.0f, 0.0f }, 0.0f);
        for (int i = 0; i < 40; ++i)
            ASSERT_TRUE(controller.update(calm));
        EXPECT_FALSE(controller.catching());
        EXPECT_FALSE(controller.update(calm)->mHoldClock);
    }

    TEST(RagdollCoreRecoveryControl, hoverReleasesWhenNoStepCatches)
    {
        CatchStepInput input = catchInput({ 0.0f, 400.0f }, 0.0f);
        CatchStepController stepping;
        const auto stepped = stepping.update(input);
        ASSERT_TRUE(stepped);
        EXPECT_TRUE(stepped->mSwingNow); // default: steps anyway
        EXPECT_FALSE(stepped->mRelease);

        input.mReleaseWhenUncatchable = true;
        CatchStepController releasing;
        const auto released = releasing.update(input);
        ASSERT_TRUE(released);
        EXPECT_TRUE(released->mRelease);
        EXPECT_FALSE(released->mSwingNow);
        EXPECT_FALSE(releasing.catching());
    }

    TEST(RagdollCoreRecoveryControl, releaseWaitsForTheStepsItWasGiven)
    {
        CatchStepInput input = catchInput({ 0.0f, 400.0f }, 0.0f);
        input.mReleaseWhenUncatchable = true;
        input.mReleaseAfterSteps = 1;
        CatchStepController controller;
        const auto first = controller.update(input);
        ASSERT_TRUE(first);
        EXPECT_FALSE(first->mRelease);
        EXPECT_TRUE(first->mSwingNow); // best effort: the backstep is tried
        // The step landed and the push still cannot be caught: now it falls.
        const auto second = controller.update(input);
        ASSERT_TRUE(second);
        EXPECT_TRUE(second->mRelease);
        EXPECT_FALSE(controller.catching());
    }

    LandingInput landingAt(BalanceVec2 velocity)
    {
        LandingInput input;
        input.mState.mCenterOfMass = { 0.0f, 0.0f };
        input.mState.mVelocity = velocity;
        input.mState.mHeight = 77.0f;
        input.mState.mGravity = sGravity;
        input.mFeet[0].mPosition = { -8.0f, -5.0f };
        input.mFeet[1].mPosition = { 8.0f, 5.0f };
        for (auto& foot : input.mFeet)
            foot.mPinned = foot.mSupported = true;
        input.mDt = 1.0f / 60.0f;
        input.mReach = 60.0f;
        return input;
    }

    TEST(RagdollCoreRecoveryControl, landingAbsorbsTheImpactThenEases)
    {
        LandingController controller;
        LandingInput input = landingAt({ 0.0f, 0.0f });
        EXPECT_FALSE(controller.update(input)); // nothing before a touchdown
        input.mTouchdown = true;
        input.mImpactSpeed = 300.0f;
        const auto first = controller.update(input);
        ASSERT_TRUE(first);
        EXPECT_NEAR(first->mRootStrengthFloor, 1.0f, 0.06f);
        EXPECT_FALSE(first->mSwingNow); // standing still: no run-out
        input.mTouchdown = false;
        std::optional<LandingCommand> last;
        for (int i = 0; i < 30; ++i)
            last = controller.update(input);
        EXPECT_FALSE(last && last->mRootStrengthFloor > 0.0f); // eased out by 0.3 s
        EXPECT_FALSE(controller.active());
    }

    TEST(RagdollCoreRecoveryControl, landingRunsOutTheCarriedSpeed)
    {
        LandingController controller;
        LandingInput input = landingAt({ 0.0f, 250.0f }); // carried forward
        input.mTouchdown = true;
        input.mImpactSpeed = 100.0f;
        const auto command = controller.update(input);
        ASSERT_TRUE(command && command->mSwingNow);
        EXPECT_TRUE(command->mHoldClock);
        EXPECT_EQ(*command->mSwingNow, 0u); // the trailing foot (y -5)
        ASSERT_TRUE(command->mLanding[0]);
        const float tau = std::sqrt(77.0f / sGravity);
        EXPECT_NEAR(command->mLanding[0]->mY, std::min(250.0f * tau, 60.0f), 1e-3f); // the capture point, within reach
        EXPECT_LT(command->mLanding[0]->mX, 8.0f - 8.0f + 1e-3f); // its own (left) side of the right foot
    }

    TEST(RagdollCoreRecoveryControl, landingReleasesAfterTheStepsItWasGiven)
    {
        LandingController controller;
        LandingInput input = landingAt({ 0.0f, 600.0f }); // Tc x 600 > reach
        input.mTouchdown = true;
        ASSERT_TRUE(controller.update(input)->mSwingNow);
        input.mTouchdown = false;
        for (int step = 0; step < 2; ++step)
        {
            input.mFeet[0].mSwinging = true; // a run-out step in flight ...
            EXPECT_FALSE(controller.update(input)->mRelease);
            input.mFeet[0].mSwinging = false; // ... lands
            const auto landed = controller.update(input);
            ASSERT_TRUE(landed);
            EXPECT_EQ(landed->mRelease, step == 1); // released only after the second step
        }
    }

    TEST(RagdollCoreRecoveryControl, crossoverGateSparesADrivenArm)
    {
        // A slow sideways lean whose catch is a crossover: the gate takes it
        // away, a driven weapon arm keeps it.
        CatchStepInput input = catchInput({ 60.0f, 0.0f }, 0.0f);
        CatchStepController open;
        const auto free = open.update(input);
        input.mCrossoverSpeed = 80.0f;
        CatchStepController gated;
        const auto blocked = gated.update(input);
        input.mArmDriven = true;
        CatchStepController armed;
        const auto kept = armed.update(input);
        ASSERT_TRUE(free && blocked && kept);
        ASSERT_EQ(free->mSwingNow.has_value(), kept->mSwingNow.has_value());
        if (free->mSwingNow)
        {
            EXPECT_EQ(*free->mSwingNow, *kept->mSwingNow);
            EXPECT_EQ(free->mLanding[*free->mSwingNow]->mX, kept->mLanding[*kept->mSwingNow]->mX);
        }
    }

    TEST(RagdollCoreRecoveryControl, hoverKeepsCatchableSteps)
    {
        CatchStepInput input = catchInput({ 0.0f, 60.0f }, 0.0f);
        input.mReleaseWhenUncatchable = true;
        CatchStepController controller;
        const auto command = controller.update(input);
        ASSERT_TRUE(command);
        EXPECT_FALSE(command->mRelease);
        EXPECT_TRUE(command->mSwingNow);
    }

    TEST(RagdollCoreRecoveryControl, hoverLimitReleasesALeanThatCannotStep)
    {
        CatchStepInput input = catchInput({ 0.0f, 60.0f }, 0.0f);
        input.mFeet[1].mSupported = false; // one foot off the floor, not swinging: no swing now
        input.mMaximumHoverTime = 0.2f;
        CatchStepController controller;
        std::optional<CatchStepCommand> last;
        int steps = 0;
        for (; steps < 30; ++steps)
        {
            last = controller.update(input);
            ASSERT_TRUE(last);
            if (last->mRelease)
                break;
        }
        ASSERT_TRUE(last->mRelease);
        EXPECT_NEAR(steps * input.mDt, 0.2f, 2.0f * input.mDt);
    }

    TEST(RagdollCoreRecoveryControl, steeringPicksTheStepAlongTheInput)
    {
        CatchStepInput input = catchInput({ 0.0f, 60.0f }, 0.0f);
        input.mDirectionBias = { 1.0f, 0.0f };
        CatchStepController right;
        const auto toRight = right.update(input);
        input.mDirectionBias = { -1.0f, 0.0f };
        CatchStepController left;
        const auto toLeft = left.update(input);
        ASSERT_TRUE(toRight && toLeft);
        ASSERT_TRUE(toRight->mSwingNow && toLeft->mSwingNow);
        EXPECT_NE(*toRight->mSwingNow, *toLeft->mSwingNow);
        const auto goes = [&](const CatchStepCommand& c) {
            return c.mLanding[*c.mSwingNow]->mX - input.mFeet[*c.mSwingNow].mPosition.mX;
        };
        EXPECT_GT(goes(*toRight), goes(*toLeft));
    }

    TEST(RagdollCoreRecoveryControl, deliberateStepGoesAlongTheInput)
    {
        CatchStepInput input = catchInput({ 0.0f, 0.0f }, 0.0f);
        input.mFeet[1].mPosition = { 8.0f, 6.0f }; // right foot ahead: the left one trails
        input.mDirectionBias = { 0.0f, 1.0f };
        CatchStepController off;
        EXPECT_FALSE(off.update(input)->mSwingNow); // off by default
        input.mDeliberateSteps = true;
        CatchStepController on;
        const auto command = on.update(input);
        ASSERT_TRUE(command && command->mSwingNow);
        EXPECT_EQ(*command->mSwingNow, 0u);
        ASSERT_TRUE(command->mLanding[0]);
        EXPECT_NEAR(command->mLanding[0]->mY, input.mDeliberateStride, 1e-3f);
        EXPECT_NEAR(command->mLanding[0]->mX, -8.0f, 1e-3f); // keeps its side
        input.mFeet[0].mSwinging = true;
        const auto inFlight = on.update(input);
        EXPECT_TRUE(inFlight->mHoldClock);
        EXPECT_FALSE(inFlight->mSwingNow);
    }

    TEST(RagdollCoreRecoveryControl, stumbleStepsAlongASlowCarry)
    {
        // Carried slowly: the capture point stays over the feet, so no catch
        // starts; a stumble request runs the catch and a foot steps along.
        for (const BalanceVec2 velocity : { BalanceVec2{ 0.0f, 22.0f }, BalanceVec2{ 22.0f, 0.0f } })
        {
            CatchStepInput input = catchInput(velocity, 0.0f);
            CatchStepController off;
            EXPECT_FALSE(off.update(input)->mSwingNow);
            input.mStumbleSteps = 2;
            input.mStumbleSpeed = 15.0f;
            CatchStepController on;
            const auto command = on.update(input);
            ASSERT_TRUE(command && command->mSwingNow);
            EXPECT_FALSE(command->mRelease);
            const std::size_t side = *command->mSwingNow;
            ASSERT_TRUE(command->mLanding[side]);
            const BalanceVec2 moved = { command->mLanding[side]->mX - input.mFeet[side].mPosition.mX,
                command->mLanding[side]->mY - input.mFeet[side].mPosition.mY };
            EXPECT_GT(moved.mX * velocity.mX + moved.mY * velocity.mY, 0.0f); // along the carry
            // Too slow: no stumble step.
            input.mState.mVelocity = { velocity.mX * 0.4f, velocity.mY * 0.4f };
            CatchStepController slow;
            EXPECT_FALSE(slow.update(input)->mSwingNow);
        }
    }

    MovingCatchInput runningBackward(BalanceVec2 velocity)
    {
        MovingCatchInput input;
        input.mState.mCenterOfMass = { 0.0f, 0.0f };
        input.mState.mVelocity = velocity;
        input.mState.mHeight = 77.0f;
        input.mState.mGravity = sGravity;
        input.mCommandedVelocity = { 0.0f, -200.0f };
        input.mFeet[0].mPosition = { -8.0f, 10.0f };
        input.mFeet[0].mPinned = input.mFeet[0].mSupported = true;
        input.mFeet[1].mPosition = { 8.0f, 20.0f };
        input.mFeet[1].mSwinging = true;
        input.mFeet[1].mTarget = { 8.0f, -30.0f };
        input.mDt = 1.0f / 60.0f;
        input.mReach = 60.0f;
        return input;
    }

    TEST(RagdollCoreRecoveryControl, movingCatchLeavesASteadyGaitAlone)
    {
        MovingCatchController controller;
        const auto command = controller.update(runningBackward({ 0.0f, -200.0f }));
        ASSERT_TRUE(command);
        EXPECT_FALSE(command->mLanding[0]);
        EXPECT_FALSE(command->mLanding[1]);
        EXPECT_FALSE(command->mSwingNow);
        EXPECT_FALSE(command->mRelease);
    }

    TEST(RagdollCoreRecoveryControl, movingCatchMovesTheLandingByTheVelocityError)
    {
        MovingCatchController controller;
        // Falling back faster than the gait runs: the backstep goes further back.
        const auto command = controller.update(runningBackward({ 0.0f, -260.0f }));
        ASSERT_TRUE(command && command->mLanding[1]);
        const float tau = std::sqrt(77.0f / sGravity);
        EXPECT_NEAR(command->mLanding[1]->mY, -30.0f - 0.5f * 60.0f * tau, 1e-3f); // gain 0.5
        EXPECT_FALSE(command->mLanding[0]); // the planted foot is not touched
    }

    TEST(RagdollCoreRecoveryControl, movingCatchSwingsAHangingFootNow)
    {
        MovingCatchController controller;
        MovingCatchInput input = runningBackward({ 0.0f, -260.0f });
        input.mFeet[1].mSwinging = false; // neither pinned nor swinging: hanging
        std::optional<MovingCatchCommand> command;
        for (int i = 0; i < 10 && !(command && command->mSwingNow); ++i)
            command = controller.update(input);
        ASSERT_TRUE(command && command->mSwingNow);
        EXPECT_EQ(*command->mSwingNow, 1u);
        ASSERT_TRUE(command->mLanding[1]);
        EXPECT_LT(command->mLanding[1]->mY, -20.0f); // back, along the run and the error
        EXPECT_LE(length(*command->mLanding[1]), 60.0f + 1e-3f); // within reach
    }

    TEST(RagdollCoreRecoveryControl, movingCatchStepsAStanceFootLeftBehind)
    {
        // Game backward run (09-30 bookmark 1): the speed matches the command,
        // but the stance foot is planted 59 units behind along the travel.
        MovingCatchController controller;
        MovingCatchInput input = runningBackward({ 0.0f, -200.0f });
        input.mFeet[0].mPosition = { -18.0f, 59.0f };
        input.mFeet[1] = input.mFeet[0];
        input.mFeet[1].mPosition = { 30.0f, 33.0f };
        std::optional<MovingCatchCommand> command;
        for (int i = 0; i < 10 && !(command && command->mSwingNow); ++i)
            command = controller.update(input);
        ASSERT_TRUE(command && command->mSwingNow);
        EXPECT_EQ(*command->mSwingNow, 0u); // the foot further behind
        EXPECT_TRUE(command->mTrail);
        ASSERT_TRUE(command->mLanding[0]);
        EXPECT_LT(command->mLanding[0]->mY, 0.0f); // ahead along the backward travel
        EXPECT_LT(command->mLanding[0]->mX, 30.0f - 6.0f + 1e-3f); // its own (left) side of the stance foot

        // Feet within the trail limit (0.6 x reach = 36) stay with Footfall.
        MovingCatchController calm;
        input.mFeet[0].mPosition = { -8.0f, 30.0f };
        for (int i = 0; i < 10; ++i)
            EXPECT_FALSE(calm.update(input)->mSwingNow);
    }

    TEST(RagdollCoreRecoveryControl, movingCatchReleasesWhenNoStepReaches)
    {
        MovingCatchController controller;
        MovingCatchInput input = runningBackward({ 0.0f, -700.0f }); // error 500 u/s: Tc x 500 > reach
        std::optional<MovingCatchCommand> command;
        int steps = 0;
        for (; steps < 60; ++steps)
        {
            command = controller.update(input);
            if (command->mRelease)
                break;
        }
        ASSERT_TRUE(command && command->mRelease);
        EXPECT_NEAR(steps * input.mDt, input.mSettings.mReleaseTime, 2.0f * input.mDt);
        ASSERT_TRUE(command->mLanding[1]);
        EXPECT_LE(length(*command->mLanding[1]), 60.0f + 1e-3f);
    }

    TEST(RagdollCoreRecoveryControl, uprightTiltStandsThePartUp)
    {
        const BalanceVec2 upright = uprightTilt(0.0f, 0.0f, 1.0f);
        EXPECT_NEAR(length(upright), 0.0f, 1e-6f);
        const float angle = 0.3f;
        // Leaning forward (+y): turning about +x by the angle stands it up.
        const BalanceVec2 tilt = uprightTilt(0.0f, std::sin(angle), std::cos(angle));
        EXPECT_NEAR(tilt.mX, angle, 1e-5f);
        EXPECT_NEAR(tilt.mY, 0.0f, 1e-6f);
        const BalanceVec2 side = uprightTilt(std::sin(angle), 0.0f, std::cos(angle));
        EXPECT_NEAR(side.mY, -angle, 1e-5f);
    }

    TEST(RagdollCoreRecoveryControl, postureLeansAgainstTheTiltAndIsCapped)
    {
        PostureController posture;
        const BalanceVec2 tilt{ 0.2f, 0.0f };
        const BalanceVec2 first = posture.update(tilt, 1.0f / 60.0f);
        EXPECT_NEAR(first.mX, 0.5f * 0.2f + 1.0f * 0.2f / 60.0f, 1e-5f);
        BalanceVec2 later;
        for (int i = 0; i < 600; ++i)
            later = posture.update(tilt, 1.0f / 60.0f);
        EXPECT_NEAR(length(later), 25.0f * 3.14159265f / 180.0f, 1e-4f); // capped
        posture.reset();
        EXPECT_NEAR(posture.update({}, 1.0f / 60.0f).mX, 0.0f, 1e-6f);
        EXPECT_NEAR(length(posture.update(tilt, 0.0f)), 0.0f, 1e-6f); // no dt: nothing
    }

    TEST(RagdollCoreRecoveryControl, rootBudgetCapsTheHitStrength)
    {
        RootBudgetInput input;
        EXPECT_FLOAT_EQ(rootBudgetStrength(input), 0.15f);
        input.mRequestSpeed = 128.0f;
        EXPECT_FLOAT_EQ(rootBudgetStrength(input), 0.3f);
        input.mHitStrength = 0.1f;
        EXPECT_FLOAT_EQ(rootBudgetStrength(input), 0.1f);
        input.mHitStrength = std::nanf("");
        EXPECT_FLOAT_EQ(rootBudgetStrength(input), 0.0f);
        // Stopping from a run: no request, but the body still moves fast.
        RootBudgetInput braking;
        braking.mBodySpeed = 180.0f;
        EXPECT_FLOAT_EQ(rootBudgetStrength(braking), 0.3f);
    }
}
