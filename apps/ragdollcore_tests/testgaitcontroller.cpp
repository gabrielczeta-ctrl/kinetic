#include <ragdollcore/gaitcontroller.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace
{
    TEST(RagdollCoreGaitControllerTest, ownsPhaseAndRequiresLiftBeforeConfirmedContact)
    {
        RagdollCore::GaitControllerProfile profile;
        profile.mMinimumStepDuration = 0.15f;
        profile.mNominalStepDuration = 0.4f;
        profile.mContactConfirmationDuration = 0.05f;
        RagdollCore::GaitPhaseController controller;
        RagdollCore::GaitControllerInput input;
        input.mEnabled = true;
        input.mLeftFootSupported = true;
        input.mRightFootSupported = true;
        input.mStanceSupportsCenterOfMass = true;
        RagdollCore::GaitControllerOutput output;

        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        EXPECT_TRUE(output.mTransitioned);

        ASSERT_TRUE(controller.step(input, 0.2f, profile, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        EXPECT_FALSE(output.mTransitioned);

        input.mRightFootSupported = false;
        ASSERT_TRUE(controller.step(input, 0.05f, profile, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        EXPECT_FALSE(output.mSwingFootLifted);

        input.mRightFootHeight = 1.1f;
        input.mRightFootVerticalVelocity = 1.0f;
        ASSERT_TRUE(controller.step(input, 0.05f, profile, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        EXPECT_TRUE(output.mSwingFootLifted);
        EXPECT_NEAR(output.mSwingTimeSinceLift, 0.05f, 1e-6f);

        input.mRightFootVerticalVelocity = -1.0f;
        input.mSwingFootHorizontalPlacementError = 0.0f;
        input.mSwingFootVerticalPlacementError = profile.mMaximumLandingApproachVerticalDistance - 0.5f;
        ASSERT_TRUE(controller.step(input, 0.01f, profile, output));
        EXPECT_TRUE(output.mSwingTouchdownCandidate);

        input.mRightFootSupported = true;
        input.mSwingFootHorizontalPlacementError = profile.mMaximumTouchdownHorizontalDistance + 1.0f;
        ASSERT_TRUE(controller.step(input, 0.02f, profile, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        EXPECT_NEAR(output.mSwingContactCandidateTime, 0.02f, 1e-6f);
        ASSERT_TRUE(controller.step(input, 0.031f, profile, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        EXPECT_FALSE(output.mTransitioned);

        input.mSwingFootHorizontalPlacementError = 0.0f;
        input.mSwingFootRelativeVerticalSpeed = profile.mMaximumTouchdownRelativeVerticalSpeed + 1.0f;
        ASSERT_TRUE(controller.step(input, 0.001f, profile, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        EXPECT_FALSE(output.mTransitioned);
        EXPECT_TRUE(output.mSwingTouchdownCandidate);

        input.mSwingFootRelativeVerticalSpeed = 0.0f;
        ASSERT_TRUE(controller.step(input, 0.001f, profile, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::RightStance);
        EXPECT_TRUE(output.mTransitioned);
        EXPECT_FALSE(output.mSwingTouchdownCandidate);
        EXPECT_FLOAT_EQ(output.mTimeInPhase, 0.0f);
        EXPECT_FLOAT_EQ(output.mPhaseProgress, 0.0f);

        ASSERT_TRUE(controller.step(input, 0.2f, profile, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::RightStance);
        EXPECT_FALSE(output.mTransitioned);
    }

    TEST(RagdollCoreGaitControllerTest, contactFlickerDoesNotInventLiftOff)
    {
        RagdollCore::GaitPhaseController controller;
        RagdollCore::GaitControllerInput input;
        input.mEnabled = true;
        input.mLeftFootSupported = true;
        input.mRightFootSupported = true;
        input.mStanceSupportsCenterOfMass = true;
        RagdollCore::GaitControllerOutput output;
        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, {}, output));
        ASSERT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);

        input.mRightFootSupported = false;
        input.mLeftFootHeight = 10.0f;
        input.mRightFootHeight = 10.5f;
        input.mLeftFootVerticalVelocity = 100.0f;
        input.mRightFootVerticalVelocity = 100.1f;
        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, {}, output));
        EXPECT_FALSE(output.mSwingFootLifted);

        input.mLeftFootHeight = 8.0f;
        input.mRightFootHeight = 10.5f;
        input.mLeftFootVerticalVelocity = -100.0f;
        input.mRightFootVerticalVelocity = 0.0f;
        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, {}, output));
        EXPECT_FALSE(output.mSwingFootLifted);

        input.mRightFootSupported = true;
        ASSERT_TRUE(controller.step(input, 1.0f, {}, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        EXPECT_FALSE(output.mTransitioned);
        EXPECT_TRUE(output.mPhaseTimedOut);
    }

    TEST(RagdollCoreGaitControllerTest, reportsTimeoutWhenSupportedSwingNeverLifts)
    {
        RagdollCore::GaitControllerProfile profile;
        profile.mMinimumStepDuration = 0.1f;
        profile.mNominalStepDuration = 0.2f;
        profile.mMaximumStepDuration = 0.3f;
        RagdollCore::GaitPhaseController controller;
        RagdollCore::GaitControllerInput input;
        input.mEnabled = true;
        input.mRightFootSupported = true;
        input.mStanceSupportsCenterOfMass = true;
        RagdollCore::GaitControllerOutput output;

        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::RightStance);
        ASSERT_TRUE(controller.step(input, 0.31f, profile, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::RightStance);
        EXPECT_TRUE(output.mPhaseTimedOut);
        EXPECT_FLOAT_EQ(output.mPhaseProgress, 1.0f);
    }

    TEST(RagdollCoreGaitControllerTest, givesDelayedLiftItsOwnBoundedSwingInterval)
    {
        RagdollCore::GaitControllerProfile profile;
        profile.mNominalStepDuration = 0.2f;
        profile.mMaximumStepDuration = 0.3f;
        profile.mStanceConfirmationDuration = 0.0f;
        RagdollCore::GaitPhaseController controller;
        RagdollCore::GaitControllerInput input;
        input.mEnabled = true;
        input.mLeftFootSupported = true;
        input.mRightFootSupported = true;
        input.mStanceSupportsCenterOfMass = true;
        RagdollCore::GaitControllerOutput output;

        ASSERT_TRUE(controller.step(input, 0.01f, profile, output));
        ASSERT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        ASSERT_TRUE(controller.step(input, 0.25f, profile, output));
        EXPECT_FALSE(output.mSwingFootLifted);
        EXPECT_FALSE(output.mPhaseTimedOut);

        input.mRightFootSupported = false;
        input.mRightFootHeight = 2.0f;
        input.mRightFootVerticalVelocity = 2.0f;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        ASSERT_TRUE(output.mSwingFootLifted);
        EXPECT_NEAR(output.mSwingTimeSinceLift, 0.1f, 1e-6f);
        EXPECT_FALSE(output.mPhaseTimedOut)
            << "pre-lift unloading time consumed the complete post-lift interval";

        ASSERT_TRUE(controller.step(input, 0.21f, profile, output));
        EXPECT_NEAR(output.mSwingTimeSinceLift, 0.31f, 1e-6f);
        EXPECT_TRUE(output.mPhaseTimedOut);
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        EXPECT_FALSE(output.mTransitioned);
    }

    TEST(RagdollCoreGaitControllerTest, disablingLocomotionResetsOwnedPhase)
    {
        RagdollCore::GaitPhaseController controller;
        RagdollCore::GaitControllerInput input;
        input.mEnabled = true;
        input.mLeftFootSupported = true;
        input.mStanceSupportsCenterOfMass = true;
        RagdollCore::GaitControllerOutput output;
        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, {}, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);

        input.mEnabled = false;
        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, {}, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::Idle);
        EXPECT_FALSE(output.hasStance());
        EXPECT_FLOAT_EQ(output.mTimeInPhase, 0.0f);
    }

    TEST(RagdollCoreGaitControllerTest, holdsSwingPhaseUntilStanceSupportsCenterOfMass)
    {
        RagdollCore::GaitControllerProfile profile;
        profile.mNominalStepDuration = 0.2f;
        profile.mMaximumStepDuration = 0.3f;
        RagdollCore::GaitPhaseController controller;
        RagdollCore::GaitControllerInput input;
        input.mEnabled = true;
        input.mLeftFootSupported = true;
        input.mRightFootSupported = true;
        RagdollCore::GaitControllerOutput output;

        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, profile, output));
        ASSERT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        EXPECT_TRUE(output.mTransitioned);

        ASSERT_TRUE(controller.step(input, 1.0f, profile, output));
        EXPECT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        EXPECT_FLOAT_EQ(output.mTimeInPhase, 0.0f);
        EXPECT_FLOAT_EQ(output.mPhaseProgress, 0.0f);
        EXPECT_FALSE(output.mStanceReady);
        EXPECT_FALSE(output.mSwingFootLifted);
        EXPECT_FALSE(output.mPhaseTimedOut);

        input.mStanceSupportsCenterOfMass = true;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_FLOAT_EQ(output.mTimeInPhase, 0.1f);
        EXPECT_GT(output.mPhaseProgress, 0.0f);
        EXPECT_TRUE(output.mStanceReady);
        EXPECT_FALSE(output.mTransitioned);
    }

    TEST(RagdollCoreGaitControllerTest, revokesReadinessWhenStanceSupportIsLostBeforeLiftOff)
    {
        RagdollCore::GaitControllerProfile profile;
        profile.mStanceConfirmationDuration = 0.05f;
        RagdollCore::GaitPhaseController controller;
        RagdollCore::GaitControllerInput input;
        input.mEnabled = true;
        input.mLeftFootSupported = true;
        input.mRightFootSupported = true;
        input.mStanceSupportsCenterOfMass = true;
        RagdollCore::GaitControllerOutput output;

        ASSERT_TRUE(controller.step(input, 0.01f, profile, output));
        ASSERT_EQ(output.mPhase, RagdollCore::GaitPhase::LeftStance);
        ASSERT_TRUE(controller.step(input, 0.05f, profile, output));
        ASSERT_TRUE(output.mStanceReady);
        ASSERT_GT(output.mPhaseProgress, 0.0f);

        input.mLeftFootSupported = false;
        input.mStanceSupportsCenterOfMass = false;
        ASSERT_TRUE(controller.step(input, 0.01f, profile, output));
        EXPECT_TRUE(output.mStanceReady) << "a one-frame support dropout should be tolerated";
        EXPECT_GT(output.mPhaseProgress, 0.0f);
        ASSERT_TRUE(controller.step(input, 0.041f, profile, output));
        EXPECT_FALSE(output.mStanceReady);
        EXPECT_FALSE(output.mSwingFootLifted);
        EXPECT_FLOAT_EQ(output.mTimeInPhase, 0.0f);
        EXPECT_FLOAT_EQ(output.mPhaseProgress, 0.0f);
        EXPECT_FALSE(output.mPhaseTimedOut);

        input.mLeftFootSupported = true;
        input.mStanceSupportsCenterOfMass = true;
        ASSERT_TRUE(controller.step(input, 0.04f, profile, output));
        EXPECT_FALSE(output.mStanceReady);
        ASSERT_TRUE(controller.step(input, 0.011f, profile, output));
        EXPECT_TRUE(output.mStanceReady);
        EXPECT_GT(output.mPhaseProgress, 0.0f);
    }

    TEST(RagdollCoreGaitControllerTest, retainsReadinessWhenDynamicCenterOfMassLeavesStaticStanceHull)
    {
        using namespace RagdollCore;
        ActiveControllerProfile activeProfile;
        auto& profile = activeProfile.mGait;
        profile.mStanceConfirmationDuration = 0.05f;
        RagdollCore::GaitPhaseController controller;
        RagdollCore::GaitControllerInput input;
        input.mEnabled = true;
        input.mLeftFootSupported = true;
        input.mRightFootSupported = true;
        input.mStanceSupportsCenterOfMass = true;
        RagdollCore::GaitControllerOutput output;

        ASSERT_TRUE(controller.step(input, 0.01f, profile, output));
        ASSERT_TRUE(controller.step(input, 0.05f, profile, output));
        ASSERT_TRUE(output.mStanceReady);
        const float phaseTime = output.mTimeInPhase;

        const std::vector<GroundContact> contacts{
            { 3, JPH::RVec3(-2, -4, 0) }, { 3, JPH::RVec3(2, -4, 0) },
            { 3, JPH::RVec3(2, 4, 0) }, { 3, JPH::RVec3(-2, 4, 0) } };
        const auto forward = estimateGaitStanceSupport(GaitPhase::LeftStance,
            JPH::RVec3(0, 10, 50), JPH::Vec3(0, 20, 0), JPH::Vec3(0, 60, 0),
            50, 625, contacts, { 3, 6 }, activeProfile, true);
        ASSERT_TRUE(forward);
        ASSERT_GT(forward->mSupport.mCenterOfMassDistance, 0.0f);
        input.mStanceSupportsCenterOfMass = forward->mReady;
        ASSERT_TRUE(controller.step(input, 0.10f, profile, output));
        EXPECT_TRUE(output.mStanceReady);
        EXPECT_GT(output.mTimeInPhase, phaseTime);
        EXPECT_GT(output.mPhaseProgress, 0.0f);

        const auto lateral = estimateGaitStanceSupport(GaitPhase::LeftStance,
            JPH::RVec3(0, 10, 50), JPH::Vec3(10, 20, 0), JPH::Vec3(0, 60, 0),
            50, 625, contacts, { 3, 6 }, activeProfile, true);
        ASSERT_TRUE(lateral);
        input.mStanceSupportsCenterOfMass = lateral->mReady;
        ASSERT_TRUE(controller.step(input, 0.01f, profile, output));
        EXPECT_FALSE(output.mStanceReady)
            << "measured lateral escape must revoke pre-lift readiness even with both feet contacting";
        EXPECT_FLOAT_EQ(output.mTimeInPhase, 0.0f);
    }

    TEST(RagdollCoreGaitControllerTest, captureLossCannotCancelLiftOffObservedInTheSameSolve)
    {
        using namespace RagdollCore;
        GaitPhaseController controller;
        GaitControllerProfile profile;
        GaitControllerInput input;
        input.mEnabled = input.mLeftFootSupported = input.mRightFootSupported = true;
        input.mStanceSupportsCenterOfMass = true;
        GaitControllerOutput output;
        ASSERT_TRUE(controller.step(input, 0.01f, profile, output));
        ASSERT_TRUE(controller.step(input, profile.mStanceConfirmationDuration, profile, output));
        ASSERT_TRUE(output.mStanceReady);
        input.mRightFootSupported = false;
        input.mRightFootHeight = profile.mSwingFootLift;
        input.mRightFootVerticalVelocity = 1.0f;
        input.mStanceSupportsCenterOfMass = false;
        ASSERT_TRUE(controller.step(input, 0.01f, profile, output));
        EXPECT_TRUE(output.mSwingFootLifted);
        EXPECT_TRUE(output.mStanceReady);
        EXPECT_GT(output.mSwingTimeSinceLift, 0.0f);
    }

    TEST(RagdollCoreGaitControllerTest, stationaryRecoveryStepExclusivelyUsesReferenceJointSupport)
    {
        using namespace RagdollCore;
        GaitActuationDecisionInput input;
        input.mDriveJointMotors = true;
        input.mRecovering = true;
        input.mReferenceCorrectiveStep = true;
        const auto decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_TRUE(decision->mUseJointSupportControl);
        EXPECT_TRUE(decision->mLocomoting);
        EXPECT_TRUE(decision->mTargetHorizontalVelocity.IsNearZero());
        EXPECT_FALSE(decision->mRequireInternalAngularReaction);
        EXPECT_FALSE(decision->mReleaseStanceHipAxis);
        input.mRequestedHorizontalVelocity = JPH::Vec3(0, 1, 0);
        EXPECT_FALSE(calculateGaitActuationDecision(input, {}));
        input.mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        input.mRecovering = false;
        const auto standing = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(standing);
        EXPECT_TRUE(standing->mUseJointSupportControl);
        EXPECT_TRUE(standing->mTargetHorizontalVelocity.IsNearZero());
        input.mDriveJointMotors = false;
        EXPECT_FALSE(calculateGaitActuationDecision(input, {}));
    }

    TEST(RagdollCoreGaitControllerTest, standingJointSupportDoesNotWaitForHeightCalibrationOrOwnRecovery)
    {
        using namespace RagdollCore;
        GaitActuationDecisionInput input;
        input.mDriveJointMotors = true;
        auto decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_TRUE(decision->mUseJointSupportControl);
        input.mDriveJointMotors = false;
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_FALSE(decision->mUseJointSupportControl);
        input.mDriveJointMotors = true;
        input.mRecovering = true;
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_FALSE(decision->mUseJointSupportControl);
        input.mRecovering = false;
        input.mLocomotionRequested = true;
        input.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_FALSE(decision->mUseJointSupportControl);
    }

    TEST(RagdollCoreGaitControllerTest, heldMovementCannotActuateTheGaitDuringRecovery)
    {
        using namespace RagdollCore;
        GaitActuationDecisionInput input;
        input.mDriveJointMotors = true;
        input.mLocomotionRequested = true;
        input.mRequestedHorizontalVelocity = JPH::Vec3(0, 128, 0);
        input.mPelvisHeightSettled = true;
        input.mCorrectTranslation = true;
        input.mRecovering = true;
        auto decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_FALSE(decision->mLocomoting);
        EXPECT_TRUE(decision->mTargetHorizontalVelocity.IsNearZero());
        EXPECT_FALSE(decision->mUseJointSupportControl);
        EXPECT_FALSE(decision->mUseSupportedFootWorldMotors);
        input.mRecovering = false;
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_TRUE(decision->mLocomoting);
        EXPECT_EQ(decision->mTargetHorizontalVelocity, input.mRequestedHorizontalVelocity);
    }

    TEST(RagdollCoreGaitControllerTest, worldFootMotorsRequireActiveGaitMotorsAndMeasuredContact)
    {
        using namespace RagdollCore;
        GaitActuationDecisionInput input;
        input.mLocomotionRequested = true;
        input.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
        input.mPelvisHeightSettled = true;
        input.mDriveJointMotors = true;
        SupportEstimate support;
        support.mLeftFootSupported = true;
        support.mRightFootSupported = true;
        const FootBodyIndices feet{ 4, 8 };
        auto decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_TRUE(selectGaitWorldFootMotors(*decision, support, feet).empty());
        input.mGait.mPhase = GaitPhase::LeftStance;
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_EQ(selectGaitWorldFootMotors(*decision, support, feet), (std::vector<std::size_t>{ 4, 8 }));
        support.mRightFootSupported = false;
        EXPECT_EQ(selectGaitWorldFootMotors(*decision, support, feet), (std::vector<std::size_t>{ 4 }));
        support.mLeftFootSupported = false;
        EXPECT_TRUE(selectGaitWorldFootMotors(*decision, support, feet).empty());
        support.mLeftFootSupported = true;
        input.mDriveJointMotors = false;
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_TRUE(selectGaitWorldFootMotors(*decision, support, feet).empty());
    }

    TEST(RagdollCoreGaitControllerTest, derivesSharedActuationStages)
    {
        RagdollCore::GaitControllerProfile profile;
        profile.mPreLiftSwingUnloadFraction = 0.5f;
        profile.mSwingLegMotorTorqueScale = 4.0f;
        RagdollCore::GaitActuationDecisionInput input;
        input.mLocomotionRequested = true;
        input.mRequestedHorizontalVelocity = JPH::Vec3(0.0f, 60.0f, 0.0f);
        input.mDriveJointMotors = true;
        input.mCorrectTranslation = true;
        input.mBothFeetSupported = true;

        auto decision = RagdollCore::calculateGaitActuationDecision(input, profile);
        ASSERT_TRUE(decision.has_value());
        EXPECT_EQ(decision->mStage, RagdollCore::GaitActuationStage::Settling);
        EXPECT_FALSE(decision->mLocomoting);

        input.mPelvisHeightSettled = true;
        decision = RagdollCore::calculateGaitActuationDecision(input, profile);
        ASSERT_TRUE(decision.has_value());
        EXPECT_EQ(decision->mStage, RagdollCore::GaitActuationStage::AwaitingStance);
        EXPECT_TRUE(decision->mLocomoting);
        EXPECT_TRUE(decision->mTargetHorizontalVelocity.IsClose(input.mRequestedHorizontalVelocity));

        input.mGait.mPhase = RagdollCore::GaitPhase::LeftStance;
        decision = RagdollCore::calculateGaitActuationDecision(input, profile);
        ASSERT_TRUE(decision.has_value());
        EXPECT_EQ(decision->mStage, RagdollCore::GaitActuationStage::WeightTransfer);
        EXPECT_EQ(decision->mPoseSource, RagdollCore::GaitPoseSource::MeasuredSwingLeg);
        EXPECT_EQ(decision->mHorizontalControl, RagdollCore::GaitHorizontalControl::LateralStance);
        EXPECT_TRUE(decision->mApplyPreLiftSwingUnload);
        EXPECT_TRUE(decision->mTargetHorizontalVelocity.IsNearZero());

        input.mGait.mStanceReady = true;
        decision = RagdollCore::calculateGaitActuationDecision(input, profile);
        ASSERT_TRUE(decision.has_value());
        EXPECT_EQ(decision->mStage, RagdollCore::GaitActuationStage::Unloading);
        EXPECT_EQ(decision->mPoseSource, RagdollCore::GaitPoseSource::GaitCycle);
        EXPECT_EQ(decision->mHorizontalControl, RagdollCore::GaitHorizontalControl::LateralStance);
        EXPECT_FLOAT_EQ(decision->mSwingLegMotorTorqueScale, 4.0f);
        EXPECT_TRUE(decision->mTargetHorizontalVelocity.IsClose(input.mRequestedHorizontalVelocity));

        input.mGait.mSwingTouchdownCandidate = true;
        decision = RagdollCore::calculateGaitActuationDecision(input, profile);
        ASSERT_TRUE(decision.has_value());
        EXPECT_EQ(decision->mStage, RagdollCore::GaitActuationStage::Touchdown);
        EXPECT_EQ(decision->mPoseSource, RagdollCore::GaitPoseSource::MeasuredTouchdown);
    }

    TEST(RagdollCoreGaitControllerTest, approachingFloorKeepsLandingTargetUntilContact)
    {
        using namespace RagdollCore;
        GaitActuationDecisionInput input;
        input.mLocomotionRequested = input.mPelvisHeightSettled = input.mDriveJointMotors = true;
        input.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
        input.mStanceFootSupported = true;
        input.mGait.mPhase = GaitPhase::LeftStance;
        input.mGait.mStanceReady = input.mGait.mSwingFootLifted = input.mGait.mSwingTouchdownCandidate = true;
        auto decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_EQ(decision->mPoseSource, GaitPoseSource::GaitCycle)
            << "A near-floor observation cannot hold the leg above its landing target";
        input.mGait.mSwingContactCandidateTime = 1.0f / 60.0f;
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_EQ(decision->mPoseSource, GaitPoseSource::MeasuredTouchdown);
        input.mGait.mSwingContactCandidateTime = 0;
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_EQ(decision->mPoseSource, GaitPoseSource::GaitCycle);
        input.mBothFeetSupported = true;
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_EQ(decision->mPoseSource, GaitPoseSource::MeasuredTouchdown);
    }

    TEST(RagdollCoreGaitControllerTest, transferPreservesEarnedForwardSpeedAndReleasesLoadedHipAxis)
    {
        using namespace RagdollCore;
        GaitActuationDecisionInput input;
        input.mLocomotionRequested = input.mPelvisHeightSettled = input.mDriveJointMotors = true;
        input.mCorrectTranslation = input.mBothFeetSupported = input.mStanceFootSupported = true;
        input.mGait.mPhase = GaitPhase::RightStance;
        input.mRequestedHorizontalVelocity = JPH::Vec3(36, 48, 0);
        const JPH::Vec3 forward = input.mRequestedHorizontalVelocity.Normalized();
        const JPH::Vec3 lateral = JPH::Vec3::sAxisZ().Cross(forward);
        input.mMeasuredHorizontalVelocity = forward * 25.0f + lateral * 7.0f;
        auto decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_TRUE(decision->mTargetHorizontalVelocity.IsClose(forward * 25.0f, 1e-8f));
        EXPECT_TRUE(decision->mFreeHipWorldAxis.IsClose(lateral));
        EXPECT_TRUE(decision->mReleaseStanceHipAxis);
        EXPECT_TRUE(decision->mReleaseSwingHipAxis);
        EXPECT_EQ(decision->mMeasuredPoseFrame, GaitMeasuredPoseFrame::Joint);
        const auto offset = selectGaitCenterOfMassOffset(*decision,
            input.mRequestedHorizontalVelocity, JPH::Vec3::sZero(), forward * 10.0f + lateral * 2.0f);
        ASSERT_TRUE(offset);
        EXPECT_NEAR(offset->Dot(forward), 0.0f, 1e-5f);

        input.mMeasuredHorizontalVelocity = forward * 90.0f;
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_TRUE(decision->mTargetHorizontalVelocity.IsClose(input.mRequestedHorizontalVelocity));
        input.mGait.mStanceReady = true;
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_FALSE(decision->mReleaseSwingHipAxis) << "the lifting hip must track its swing target";
        input.mGait.mSwingTouchdownCandidate = true;
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_TRUE(decision->mReleaseSwingHipAxis);
        input.mStanceFootSupported = input.mBothFeetSupported = false;
        decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_FALSE(decision->mReleaseStanceHipAxis);
        EXPECT_FALSE(decision->mReleaseSwingHipAxis);
        input.mMeasuredHorizontalVelocity.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(calculateGaitActuationDecision(input, {}));
    }

    TEST(RagdollCoreGaitControllerTest, selectsSharedCenterOfMassPolicy)
    {
        RagdollCore::GaitActuationDecision decision;
        const JPH::Vec3 requested(0.0f, 60.0f, 0.0f);
        const JPH::Vec3 defaultOffset(3.0f, 4.0f, 0.0f);
        const std::optional<JPH::Vec3> stanceOffset(JPH::Vec3(10.0f, 20.0f, 0.0f));

        auto selected = RagdollCore::selectGaitCenterOfMassOffset(
            decision, requested, defaultOffset, stanceOffset);
        ASSERT_TRUE(selected.has_value());
        EXPECT_TRUE(selected->IsClose(defaultOffset));

        decision.mHorizontalControl = RagdollCore::GaitHorizontalControl::FullStance;
        selected = RagdollCore::selectGaitCenterOfMassOffset(decision, requested, defaultOffset, stanceOffset);
        ASSERT_TRUE(selected.has_value());
        EXPECT_TRUE(selected->IsClose(*stanceOffset));

        decision.mHorizontalControl = RagdollCore::GaitHorizontalControl::LateralStance;
        selected = RagdollCore::selectGaitCenterOfMassOffset(decision, requested, defaultOffset, stanceOffset);
        ASSERT_TRUE(selected.has_value());
        EXPECT_TRUE(selected->IsClose(JPH::Vec3(10.0f, 0.0f, 0.0f)));

        EXPECT_FALSE(RagdollCore::selectGaitCenterOfMassOffset(decision, JPH::Vec3::sZero(), defaultOffset,
            stanceOffset));
    }

    TEST(RagdollCoreGaitControllerTest, sharedForceReactionPolicyRequiresMeasuredSupport)
    {
        using namespace RagdollCore;
        GaitActuationDecisionInput input;
        input.mLocomotionRequested = input.mPelvisHeightSettled = input.mDriveJointMotors = true;
        input.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
        input.mGait.mPhase = GaitPhase::LeftStance;
        input.mGait.mStanceReady = input.mGait.mSwingFootLifted = true;
        input.mStanceFootSupported = input.mSwingTargetAvailable = true;
        auto decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_EQ(decision->mStage, GaitActuationStage::Swing);
        SupportEstimate support;
        support.mLeftFootSupported = true;
        auto reactions = selectGaitReactionBodies(*decision, support, { 3, 6 }, 1, 4);
        EXPECT_EQ(reactions.mVertical, std::vector<std::size_t>{ 3 });
        EXPECT_EQ(reactions.mHorizontal, std::vector<std::size_t>{ 1 });
        EXPECT_EQ(reactions.mAngular, 1u);
        EXPECT_FALSE(reactions.mSecondaryAngular);
        // Stale controller readiness cannot manufacture a contact reaction.
        support.mLeftFootSupported = false;
        reactions = selectGaitReactionBodies(*decision, support, { 3, 6 }, 1, 4);
        EXPECT_TRUE(reactions.mVertical.empty());
        EXPECT_TRUE(reactions.mHorizontal.empty());
        EXPECT_EQ(reactions.mAngular, 1u) << "torque remains internal even without ground contact";

        input.mGait.mStanceReady = input.mGait.mSwingFootLifted = false;
        input.mBothFeetSupported = true;
        GaitControllerProfile profile;
        profile.mPreLiftSwingUnloadFraction = 0.5f;
        decision = calculateGaitActuationDecision(input, profile);
        ASSERT_TRUE(decision);
        EXPECT_EQ(calculatePreLiftSwingUnloadForce(*decision, 75, 12, 700, profile), 4200.0f);
        EXPECT_FALSE(calculatePreLiftSwingUnloadForce(*decision, 75, 76, 700, profile));
        decision->mApplyPreLiftSwingUnload = false;
        EXPECT_EQ(calculatePreLiftSwingUnloadForce(*decision, 75, 12, 700, profile), 0.0f);
    }

    TEST(RagdollCoreGaitControllerTest, readinessContainsLateralCaptureAndBoundsForwardReach)
    {
        using namespace RagdollCore;
        ActiveControllerProfile profile;
        const std::vector<GroundContact> contacts{
            { 3, JPH::RVec3(-2, -4, 0) }, { 3, JPH::RVec3(2, -4, 0) },
            { 3, JPH::RVec3(2, 4, 0) }, { 3, JPH::RVec3(-2, 4, 0) } };
        const auto ready = [&](JPH::RVec3Arg position, JPH::Vec3Arg velocity,
                               std::span<const GroundContact> observations) {
            return estimateGaitStanceSupport(GaitPhase::LeftStance, position, velocity,
                JPH::Vec3(0, 60, 0), 50, 625, observations, { 3, 6 }, profile, true);
        };
        // COM and capture may have progressed ahead of the old foot: the next
        // forward step can catch them without relaxing lateral containment.
        auto stance = ready(JPH::RVec3(0, 10, 50), JPH::Vec3(0, 20, 0), contacts);
        ASSERT_TRUE(stance);
        EXPECT_TRUE(stance->mReady);
        EXPECT_GT(stance->mSupport.mCenterOfMassDistance, 0.0f);
        stance = ready(JPH::RVec3(0, 10, 50), JPH::Vec3(10, 20, 0), contacts);
        ASSERT_TRUE(stance);
        EXPECT_FALSE(stance->mReady) << "outward lateral velocity must not masquerade as a stable stance";
        stance = ready(JPH::RVec3(0, 10, 50), JPH::Vec3(0, 1000, 0), contacts);
        ASSERT_TRUE(stance);
        EXPECT_FALSE(stance->mReady) << "forward capture exceeds reachable placement";
        stance = ready(JPH::RVec3(0, 0, 50), JPH::Vec3(0, -100, 0), contacts);
        ASSERT_TRUE(stance);
        EXPECT_FALSE(stance->mReady) << "a forward step cannot catch a backward escape";
        auto speculative = contacts;
        for (auto& contact : speculative)
            contact.mPenetrationDepth = -0.01f;
        stance = ready(JPH::RVec3(0, 0, 50), JPH::Vec3::sZero(), speculative);
        ASSERT_TRUE(stance);
        EXPECT_FALSE(stance->mReady);
        stance = ready(JPH::RVec3(0, 0, 50), JPH::Vec3::sZero(), {});
        ASSERT_TRUE(stance);
        EXPECT_FALSE(stance->mReady);
    }

    TEST(RagdollCoreGaitControllerTest, backwardCaptureReadinessRequiresReachablePredictedLanding)
    {
        using namespace RagdollCore;
        ActiveControllerProfile profile;
        const std::vector<GroundContact> contacts{
            { 3, JPH::RVec3(-2, -4, 0) }, { 3, JPH::RVec3(2, -4, 0) },
            { 3, JPH::RVec3(2, 4, 0) }, { 3, JPH::RVec3(-2, 4, 0) } };
        const GaitCaptureStepInput step{ JPH::RVec3::sZero(), 0.0f };
        const auto observe = [&](JPH::Vec3Arg velocity, std::span<const GroundContact> support) {
            return estimateGaitStanceSupport(GaitPhase::LeftStance, JPH::RVec3(0, 0, 50), velocity,
                JPH::Vec3(0, 60, 0), 50, 625, support, { 3, 6 }, profile, true, step);
        };
        const auto recovery = observe(JPH::Vec3(0, -20, 0), contacts);
        ASSERT_TRUE(recovery);
        ASSERT_TRUE(recovery->mReady);
        ASSERT_TRUE(recovery->mBackwardRecoveryTarget);
        const float omega = std::sqrt(625.0f / 50.0f);
        const float landingTime = 1.5f * profile.mGait.mNominalStepDuration
            + profile.mGait.mContactConfirmationDuration + profile.mGait.mStanceConfirmationDuration;
        const float expected = -4.0f + (-20.0f / omega + 4.0f) * std::exp(omega * landingTime);
        EXPECT_NEAR(recovery->mBackwardRecoveryTarget->GetY(), expected, 1e-4f);
        EXPECT_NEAR(recovery->mBackwardRecoveryTarget->GetX(), profile.mGait.mNominalStepWidth, 1e-4f);

        const JPH::Quat rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.1f);
        const JPH::RVec3 origin(1000000, -2000000, 300);
        auto rotatedContacts = contacts;
        for (auto& contact : rotatedContacts)
            contact.mPosition = origin + rotation * JPH::Vec3(contact.mPosition);
        const auto rotated = estimateGaitStanceSupport(GaitPhase::LeftStance,
            origin + JPH::Vec3(0, 0, 50), rotation * JPH::Vec3(0, -20, 0),
            rotation * JPH::Vec3(0, 60, 0), 50, 625, rotatedContacts, { 3, 6 }, profile, true,
            GaitCaptureStepInput{ origin, 0.0f });
        ASSERT_TRUE(rotated);
        ASSERT_TRUE(rotated->mBackwardRecoveryTarget);
        EXPECT_LT(JPH::Vec3(*rotated->mBackwardRecoveryTarget - origin
            - rotation * JPH::Vec3(*recovery->mBackwardRecoveryTarget)).Length(), 1e-3f);

        for (const JPH::Vec3 velocity : { JPH::Vec3(0, -100, 0), JPH::Vec3(10, -20, 0) })
        {
            const auto rejected = observe(velocity, contacts);
            ASSERT_TRUE(rejected);
            EXPECT_FALSE(rejected->mReady);
            EXPECT_FALSE(rejected->mBackwardRecoveryTarget);
        }
        // At x=0 this triangle's rear CoP is y=0, not the bounding box's
        // y=-4. Pretending the missing corner exists understates divergence.
        const std::vector<GroundContact> triangle{ contacts[0], contacts[2], contacts[3] };
        const auto diagonal = observe(JPH::Vec3(0, -15, 0), triangle);
        ASSERT_TRUE(diagonal);
        EXPECT_FALSE(diagonal->mReady);
        EXPECT_FALSE(diagonal->mBackwardRecoveryTarget);

        profile.mGait.mMaximumStepReach = profile.mGait.mNominalStepWidth;
        const auto unreachable = observe(JPH::Vec3(0, -20, 0), contacts);
        ASSERT_TRUE(unreachable);
        EXPECT_FALSE(unreachable->mReady);
        EXPECT_FALSE(unreachable->mBackwardRecoveryTarget);
    }

    TEST(RagdollCoreGaitControllerTest, placementPolicyFreezesRecoveryInFlightAndDiscardsOldStanceProposal)
    {
        using namespace RagdollCore;
        GaitControllerOutput gait;
        gait.mPhase = GaitPhase::LeftStance;
        SwingFootTargetInput input;
        input.mPhase = gait.mPhase;
        input.mCenterOfMassPosition = JPH::RVec3(0, 0, 50);
        input.mCenterOfMassHeight = 50;
        input.mGravity = 625;
        input.mTargetHorizontalVelocity = JPH::Vec3(0, 60, 0);
        auto placement = selectGaitSwingPlacement(gait, input, JPH::RVec3(20, -20, 0), {}, {});
        ASSERT_TRUE(placement);
        ASSERT_TRUE(placement->mBackwardRecovery);
        const auto retained = selectGaitSwingPlacement(gait, input, std::nullopt, *placement, {});
        ASSERT_TRUE(retained);
        EXPECT_TRUE(retained->mBackwardRecovery);
        EXPECT_EQ(retained->mTarget, placement->mTarget);
        gait.mSwingFootLifted = true;
        EXPECT_FALSE(selectGaitSwingPlacement(gait, input, JPH::RVec3(20, -30, 0), {}, {}));
        const auto flying = selectGaitSwingPlacement(gait, input, JPH::RVec3(20, -30, 0), *placement, {});
        ASSERT_TRUE(flying);
        EXPECT_EQ(flying->mTarget, placement->mTarget);

        gait.mSwingFootLifted = false;
        gait.mTransitioned = true;
        gait.mPhase = input.mPhase = GaitPhase::RightStance;
        const auto next = selectGaitSwingPlacement(gait, input, JPH::RVec3(20, -30, 0), *placement, {});
        ASSERT_TRUE(next);
        EXPECT_FALSE(next->mBackwardRecovery);
        ASSERT_TRUE(next->mTarget);
        EXPECT_GT(next->mTarget->GetY(), 0.0);
        EXPECT_LT(next->mTarget->GetX(), 0.0);
        gait.mPhase = GaitPhase::Idle;
        const auto idle = selectGaitSwingPlacement(gait, input, std::nullopt, *placement, {});
        ASSERT_TRUE(idle);
        EXPECT_FALSE(idle->mTarget);
        EXPECT_FALSE(idle->mBackwardRecovery);
    }

    TEST(RagdollCoreGaitControllerTest, measuredLegTargetsPreserveJointRotationsAcrossPelvisTilt)
    {
        using namespace RagdollCore;
        std::vector<Transform> measured(4);
        measured[0].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.4f);
        measured[1].mRotation = measured[0].mRotation * JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.2f);
        measured[2].mRotation = measured[1].mRotation * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.7f);
        measured[3].mRotation = measured[2].mRotation * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.2f);
        auto target = measured;
        target[0].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.0f);
        const std::array<std::size_t, 3> leg{ 1, 2, 3 };
        ASSERT_TRUE(applyGaitMeasuredLegTargets(target, measured, 0, leg, GaitMeasuredPoseFrame::Joint));
        for (std::size_t child = 1; child < 4; ++child)
        {
            const auto actual = target[child - 1].mRotation.Conjugated() * target[child].mRotation;
            const auto expected = measured[child - 1].mRotation.Conjugated() * measured[child].mRotation;
            EXPECT_NEAR(std::abs(actual.Dot(expected)), 1.0f, 1e-5f);
        }
        const auto before = target;
        const std::array<std::size_t, 3> invalid{ 1, 2, 4 };
        EXPECT_FALSE(applyGaitMeasuredLegTargets(target, measured, 0, invalid, GaitMeasuredPoseFrame::Joint));
        for (std::size_t i = 0; i < target.size(); ++i)
            EXPECT_EQ(target[i].mRotation, before[i].mRotation);
    }

    TEST(RagdollCoreGaitControllerTest, samplesPoseCycleFromPhysicsOwnedPhase)
    {
        RagdollCore::RagdollPose first;
        first.mBodyTransforms.resize(1);
        RagdollCore::RagdollPose second = first;
        second.mBodyTransforms[0].mTranslation.SetX(10.0f);
        const std::vector<RagdollCore::RagdollPose> samples{ first, second };
        RagdollCore::GaitControllerOutput gait;
        RagdollCore::RagdollPose sampled;

        gait.mPhase = RagdollCore::GaitPhase::LeftStance;
        gait.mPhaseProgress = 0.0f;
        ASSERT_TRUE(RagdollCore::sampleGaitPoseCycle(samples, gait, sampled));
        EXPECT_FLOAT_EQ(sampled.mBodyTransforms[0].mTranslation.GetX(), 0.0f);

        gait.mPhaseProgress = 0.5f;
        ASSERT_TRUE(RagdollCore::sampleGaitPoseCycle(samples, gait, sampled));
        EXPECT_FLOAT_EQ(sampled.mBodyTransforms[0].mTranslation.GetX(), 5.0f);

        gait.mPhase = RagdollCore::GaitPhase::RightStance;
        gait.mPhaseProgress = 0.0f;
        ASSERT_TRUE(RagdollCore::sampleGaitPoseCycle(samples, gait, sampled));
        EXPECT_FLOAT_EQ(sampled.mBodyTransforms[0].mTranslation.GetX(), 10.0f);

        gait.mPhaseProgress = 0.5f;
        ASSERT_TRUE(RagdollCore::sampleGaitPoseCycle(samples, gait, sampled));
        EXPECT_FLOAT_EQ(sampled.mBodyTransforms[0].mTranslation.GetX(), 5.0f);

        gait.mPhaseProgress = 1.0f;
        ASSERT_TRUE(RagdollCore::sampleGaitPoseCycle(samples, gait, sampled));
        EXPECT_FLOAT_EQ(sampled.mBodyTransforms[0].mTranslation.GetX(), 0.0f);

        gait.mPhase = RagdollCore::GaitPhase::Idle;
        EXPECT_FALSE(RagdollCore::sampleGaitPoseCycle(samples, gait, sampled));
        RagdollCore::GaitControllerOutput leftStance;
        leftStance.mPhase = RagdollCore::GaitPhase::LeftStance;
        EXPECT_FALSE(RagdollCore::sampleGaitPoseCycle(
            std::span<const RagdollCore::RagdollPose>(samples).first(1), leftStance, sampled));
    }

    TEST(RagdollCoreGaitControllerTest, rejectsInvalidInputWithoutMutatingPhase)
    {
        RagdollCore::GaitPhaseController controller;
        RagdollCore::GaitControllerInput input;
        input.mEnabled = true;
        input.mLeftFootSupported = true;
        RagdollCore::GaitControllerOutput output;
        ASSERT_TRUE(controller.step(input, 0.1f, {}, output));
        const auto before = controller.getOutput();

        EXPECT_FALSE(controller.step(input, std::numeric_limits<float>::quiet_NaN(), {}, output));
        EXPECT_EQ(controller.getOutput().mPhase, before.mPhase);
        EXPECT_FLOAT_EQ(controller.getOutput().mTimeInPhase, before.mTimeInPhase);

        input.mLeftFootHeight = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(controller.step(input, 0.1f, {}, output));
        EXPECT_EQ(controller.getOutput().mPhase, before.mPhase);
        input.mLeftFootHeight = 0.0f;

        input.mSwingFootHorizontalPlacementError = -1.0f;
        EXPECT_FALSE(controller.step(input, 0.1f, {}, output));
        EXPECT_EQ(controller.getOutput().mPhase, before.mPhase);
        input.mSwingFootHorizontalPlacementError = 0.0f;

        input.mSwingFootVerticalPlacementError = -1.0f;
        EXPECT_FALSE(controller.step(input, 0.1f, {}, output));
        EXPECT_EQ(controller.getOutput().mPhase, before.mPhase);
        input.mSwingFootVerticalPlacementError = 0.0f;

        input.mSwingFootRelativeVerticalSpeed = -1.0f;
        EXPECT_FALSE(controller.step(input, 0.1f, {}, output));
        EXPECT_EQ(controller.getOutput().mPhase, before.mPhase);
        input.mSwingFootRelativeVerticalSpeed = 0.0f;

        RagdollCore::GaitControllerProfile invalid;
        invalid.mMaximumStepDuration = invalid.mMinimumStepDuration - 0.01f;
        EXPECT_FALSE(controller.step(input, 0.1f, invalid, output));
        EXPECT_EQ(controller.getOutput().mPhase, before.mPhase);
    }

    TEST(RagdollCoreGaitControllerTest, calculatesHorizontalCapturePointInConsistentUnits)
    {
        RagdollCore::CapturePointInput input;
        input.mCenterOfMassPosition = JPH::RVec3(10.0, 20.0, 30.0);
        input.mCenterOfMassVelocity = JPH::Vec3(2.0f, -3.0f, 100.0f);
        input.mCenterOfMassHeight = 9.81f;
        input.mGravity = 9.81f;
        const auto result = RagdollCore::calculateCapturePoint(input);
        ASSERT_TRUE(result.has_value());
        EXPECT_TRUE(result->IsClose(JPH::RVec3(12.0, 17.0, 30.0)));

        input.mCenterOfMassHeight = 0.0f;
        EXPECT_FALSE(RagdollCore::calculateCapturePoint(input));
        input.mCenterOfMassHeight = 1.0f;
        input.mCenterOfMassVelocity.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(RagdollCore::calculateCapturePoint(input));
    }

    TEST(RagdollCoreGaitControllerTest, placesSwingFeetSymmetricallyAndClampsReach)
    {
        RagdollCore::GaitControllerProfile profile;
        profile.mNominalStepDuration = 0.4f;
        profile.mNominalStepWidth = 20.0f;
        profile.mMaximumStepReach = 100.0f;
        profile.mCapturePointBlend = 0.0f;
        RagdollCore::SwingFootTargetInput input;
        input.mPhase = RagdollCore::GaitPhase::LeftStance;
        input.mTargetHorizontalVelocity = JPH::Vec3(0.0f, 100.0f, 0.0f);
        input.mCenterOfMassHeight = 80.0f;
        input.mGravity = 600.0f;

        const auto rightTarget = RagdollCore::calculateSwingFootTarget(input, profile);
        ASSERT_TRUE(rightTarget.has_value());
        EXPECT_TRUE(rightTarget->IsClose(JPH::RVec3(20.0, 20.0, 0.0)));

        input.mPhase = RagdollCore::GaitPhase::RightStance;
        const auto leftTarget = RagdollCore::calculateSwingFootTarget(input, profile);
        ASSERT_TRUE(leftTarget.has_value());
        EXPECT_TRUE(leftTarget->IsClose(JPH::RVec3(-20.0, 20.0, 0.0)));

        // A COM transferred over the left stance foot must not drag the
        // planned right swing foot through the actor's midline.
        input.mPhase = RagdollCore::GaitPhase::LeftStance;
        input.mCenterOfMassPosition = JPH::RVec3(-10.0, 0.0, 80.0);
        input.mStanceFootPosition = JPH::RVec3(-10.0, 0.0, 0.0);
        input.mTargetHorizontalVelocity = JPH::Vec3::sZero();
        const auto transferred = RagdollCore::calculateSwingFootTarget(input, profile);
        ASSERT_TRUE(transferred.has_value());
        EXPECT_TRUE(transferred->IsClose(JPH::RVec3(10.0, 0.0, 0.0)));

        profile.mMaximumStepReach = 5.0f;
        profile.mNominalStepWidth = 0.0f;
        input.mCenterOfMassPosition = JPH::RVec3::sZero();
        input.mStanceFootPosition = JPH::RVec3::sZero();
        input.mTargetHorizontalVelocity = JPH::Vec3(0.0f, 100.0f, 0.0f);
        const auto clamped = RagdollCore::calculateSwingFootTarget(input, profile);
        ASSERT_TRUE(clamped.has_value());
        EXPECT_NEAR(JPH::Vec3(*clamped - input.mStanceFootPosition).Length(), 5.0f, 1e-5f);
    }

    TEST(RagdollCoreGaitControllerTest, outwardLateralCaptureMovesTheSwingFootWithoutCrossing)
    {
        RagdollCore::GaitControllerProfile profile;
        profile.mNominalStepWidth = 10.0f;
        profile.mCapturePointBlend = 1.0f;
        RagdollCore::SwingFootTargetInput input;
        input.mPhase = RagdollCore::GaitPhase::LeftStance;
        input.mCenterOfMassHeight = 80.0f;
        input.mGravity = 600.0f;
        input.mForward = JPH::Vec3::sAxisY();
        input.mTargetHorizontalVelocity = JPH::Vec3::sZero();
        input.mCenterOfMassVelocity = JPH::Vec3(120.0f, 0.0f, 0.0f);

        const auto outward = RagdollCore::calculateSwingFootTarget(input, profile);
        ASSERT_TRUE(outward);
        EXPECT_GT(outward->GetX(), profile.mNominalStepWidth);

        input.mCenterOfMassVelocity = JPH::Vec3(-120.0f, 0.0f, 0.0f);
        const auto inward = RagdollCore::calculateSwingFootTarget(input, profile);
        ASSERT_TRUE(inward);
        EXPECT_DOUBLE_EQ(inward->GetX(), profile.mNominalStepWidth);
    }

    TEST(RagdollCoreGaitControllerTest, rejectsInvalidPlacementInputs)
    {
        RagdollCore::SwingFootTargetInput input;
        input.mPhase = RagdollCore::GaitPhase::LeftStance;
        input.mCenterOfMassHeight = 80.0f;
        input.mGravity = 600.0f;
        input.mForward = JPH::Vec3::sZero();
        EXPECT_FALSE(RagdollCore::calculateSwingFootTarget(input, {}));

        input.mForward = JPH::Vec3::sAxisY();
        input.mPhase = RagdollCore::GaitPhase::Idle;
        EXPECT_FALSE(RagdollCore::calculateSwingFootTarget(input, {}));

        input.mPhase = RagdollCore::GaitPhase::LeftStance;
        RagdollCore::GaitControllerProfile invalid;
        invalid.mCapturePointBlend = 1.1f;
        EXPECT_FALSE(RagdollCore::calculateSwingFootTarget(input, invalid));
    }

    TEST(RagdollCoreGaitControllerTest, swingTrajectoryWaitsForMeasuredLiftOff)
    {
        RagdollCore::GaitControllerProfile profile;
        profile.mNominalStepDuration = 0.4f;
        profile.mSwingFootLift = 10.0f;
        RagdollCore::SwingFootTrajectoryInput input;
        input.mLiftOffPosition = JPH::Vec3(1.0f, 2.0f, 3.0f);
        input.mPlacementPosition = JPH::Vec3(5.0f, 10.0f, 3.0f);
        input.mGait.mPhase = RagdollCore::GaitPhase::LeftStance;
        input.mGait.mTimeInPhase = 1.0f;

        const auto waiting = RagdollCore::calculateSwingFootTrajectory(input, profile);
        ASSERT_TRUE(waiting.has_value());
        EXPECT_TRUE(waiting->IsClose(JPH::Vec3(1.0f, 2.0f, 13.0f)));

        input.mGait.mTimeInPhase = 0.1f;
        const auto lifting = RagdollCore::calculateSwingFootTrajectoryState(input, profile);
        ASSERT_TRUE(lifting.has_value());
        EXPECT_TRUE(lifting->mPosition.IsClose(JPH::Vec3(1.0f, 2.0f, 8.0f), 1e-5f));
        EXPECT_TRUE(lifting->mLinearVelocity.IsClose(JPH::Vec3(0.0f, 0.0f, 75.0f), 1e-4f));

        input.mGait.mTimeInPhase = 1.0f;
        input.mGait.mSwingFootLifted = true;
        input.mGait.mSwingTimeSinceLift = 0.1f;
        const auto transferring = RagdollCore::calculateSwingFootTrajectory(input, profile);
        ASSERT_TRUE(transferring.has_value());
        EXPECT_TRUE(transferring->IsClose(JPH::Vec3(3.0f, 6.0f, 13.0f), 1e-5f));
        const auto transferringState = RagdollCore::calculateSwingFootTrajectoryState(input, profile);
        ASSERT_TRUE(transferringState.has_value());
        EXPECT_TRUE(transferringState->mPosition.IsClose(*transferring));
        EXPECT_TRUE(transferringState->mLinearVelocity.IsClose(JPH::Vec3(30.0f, 60.0f, 0.0f), 1e-4f));

        input.mGait.mSwingContactCandidateTime = 0.01f;
        input.mGait.mSwingTouchdownCandidate = true;
        const auto settling = RagdollCore::calculateSwingFootTrajectoryState(input, profile);
        ASSERT_TRUE(settling.has_value());
        EXPECT_TRUE(settling->mPosition.IsClose(input.mPlacementPosition));
        EXPECT_TRUE(settling->mLinearVelocity.IsNearZero());
        input.mGait.mSwingContactCandidateTime = 0.0f;
        input.mGait.mSwingTouchdownCandidate = false;

        input.mGait.mSwingTimeSinceLift = 0.2f;
        const auto holding = RagdollCore::calculateSwingFootTrajectoryState(input, profile);
        ASSERT_TRUE(holding.has_value());
        EXPECT_TRUE(holding->mPosition.IsClose(JPH::Vec3(5.0f, 10.0f, 13.0f), 1e-5f));
        EXPECT_TRUE(holding->mLinearVelocity.IsNearZero());

        input.mGait.mSwingTimeSinceLift = 0.3f;
        const auto descending = RagdollCore::calculateSwingFootTrajectoryState(input, profile);
        ASSERT_TRUE(descending.has_value());
        EXPECT_TRUE(descending->mPosition.IsClose(JPH::Vec3(5.0f, 10.0f, 8.0f), 1e-5f));
        EXPECT_TRUE(descending->mLinearVelocity.IsClose(JPH::Vec3(0.0f, 0.0f, -75.0f), 1e-4f));

        input.mGait.mSwingTimeSinceLift = 0.4f;
        const auto placed = RagdollCore::calculateSwingFootTrajectory(input, profile);
        ASSERT_TRUE(placed.has_value());
        EXPECT_TRUE(placed->IsClose(input.mPlacementPosition, 1e-5f));

        input.mGait.mSwingTimeSinceLift = -1.0f;
        EXPECT_FALSE(RagdollCore::calculateSwingFootTrajectory(input, profile));

        input.mGait.mSwingTimeSinceLift = 0.1f;
        input.mGait.mSwingFootLifted = false;
        input.mGait.mSwingTouchdownCandidate = true;
        EXPECT_FALSE(RagdollCore::calculateSwingFootTrajectory(input, profile));
    }

    TEST(RagdollCoreGaitControllerTest, liftDetectionPreservesSwingTargetPositionAndVelocity)
    {
        using namespace RagdollCore;
        GaitControllerProfile profile;
        profile.mNominalStepDuration = 0.4f;
        profile.mSwingFootLift = 10.0f;
        for (float liftTime : { 0.02f, 0.1f, 0.19f, 0.3f })
        {
            SCOPED_TRACE(liftTime);
            SwingFootTrajectoryInput input;
            input.mLiftOffPosition = JPH::Vec3(1, 2, 3);
            input.mPlacementPosition = JPH::Vec3(5, 10, 3);
            input.mGait.mPhase = GaitPhase::LeftStance;
            input.mGait.mTimeInPhase = liftTime;
            const auto before = calculateSwingFootTrajectoryState(input, profile);
            ASSERT_TRUE(before);
            input.mGait.mSwingFootLifted = true;
            const auto after = calculateSwingFootTrajectoryState(input, profile);
            ASSERT_TRUE(after);
            EXPECT_TRUE(after->mPosition.IsClose(before->mPosition, 1e-6f));
            EXPECT_TRUE(after->mLinearVelocity.IsClose(before->mLinearVelocity, 1e-6f));
            constexpr float dt = 1e-4f;
            input.mGait.mTimeInPhase += dt;
            input.mGait.mSwingTimeSinceLift = dt;
            const auto next = calculateSwingFootTrajectoryState(input, profile);
            ASSERT_TRUE(next);
            EXPECT_LT((next->mPosition - before->mPosition - before->mLinearVelocity * dt).Length(), 1e-4f);
        }
    }

    TEST(RagdollCoreGaitControllerTest, solvesReachableTwoBoneLegTargetWithoutChangingLengths)
    {
        std::vector<RagdollCore::Transform> pose(3);
        pose[0].mTranslation = JPH::Vec3(0.0f, 0.0f, 2.0f);
        pose[1].mTranslation = JPH::Vec3(0.0f, 0.5f, 1.0f);
        pose[2].mTranslation = JPH::Vec3(0.0f, 0.0f, 0.0f);
        const float upperLength = (pose[1].mTranslation - pose[0].mTranslation).Length();
        const float lowerLength = (pose[2].mTranslation - pose[1].mTranslation).Length();

        RagdollCore::TwoBoneLegTargetInput input;
        input.mHipBodyIndex = 0;
        input.mKneeBodyIndex = 1;
        input.mFootBodyIndex = 2;
        input.mTargetFootPosition = JPH::Vec3(1.0f, 0.0f, 0.5f);
        input.mPreferredBendDirection = JPH::Vec3::sAxisY();
        ASSERT_TRUE(RagdollCore::applyTwoBoneLegTarget(pose, input));
        EXPECT_TRUE(pose[2].mTranslation.IsClose(input.mTargetFootPosition));
        EXPECT_NEAR((pose[1].mTranslation - pose[0].mTranslation).Length(), upperLength, 1e-5f);
        EXPECT_NEAR((pose[2].mTranslation - pose[1].mTranslation).Length(), lowerLength, 1e-5f);
        EXPECT_GT(pose[1].mTranslation.GetY(), 0.0f);
        EXPECT_TRUE(pose[0].mRotation.IsNormalized());
        EXPECT_TRUE(pose[1].mRotation.IsNormalized());
    }

    TEST(RagdollCoreGaitControllerTest, lateralLegTargetPreservesTheKneeHingeAxis)
    {
        using namespace RagdollCore;
        for (const float bendSide : { -1.0f, 1.0f })
            for (const auto rotation : { JPH::Quat::sIdentity(),
                     JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.2f)
                         * JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.4f) })
            {
                std::vector<Transform> pose(3);
                const std::array<JPH::Vec3, 3> positions{ JPH::Vec3(0, 0, 2),
                    JPH::Vec3(0, bendSide * 0.5f, 1), JPH::Vec3(0, 0, 0) };
                for (std::size_t i = 0; i < pose.size(); ++i)
                {
                    pose[i].mTranslation = rotation * positions[i];
                    pose[i].mRotation = rotation;
                }
                const auto original = pose;
                TwoBoneLegTargetInput input;
                input.mKneeBodyIndex = 1;
                input.mFootBodyIndex = 2;
                input.mTargetFootPosition = rotation * JPH::Vec3(1, 0.2f, 0.5f);
                input.mPreferredBendDirection = rotation * JPH::Vec3::sAxisY();
                ASSERT_TRUE(applyTwoBoneLegTarget(pose, input));
                EXPECT_TRUE(pose[2].mTranslation.IsClose(input.mTargetFootPosition, 1e-5f));
                // Both halves of a physical hinge must retain the same world
                // axis. Independent shortest-arc bone rotations do not ensure it.
                const auto upperAxis = pose[0].mRotation * JPH::Vec3::sAxisX();
                const auto lowerAxis = pose[1].mRotation * JPH::Vec3::sAxisX();
                EXPECT_GT(upperAxis.Dot(lowerAxis), 0.99999f);
                for (std::size_t i = 0; i < 2; ++i)
                {
                    const auto localSegment = original[i].mRotation.Conjugated()
                        * (original[i + 1].mTranslation - original[i].mTranslation);
                    EXPECT_TRUE((pose[i].mRotation * localSegment).IsClose(
                        pose[i + 1].mTranslation - pose[i].mTranslation, 1e-5f));
                }
            }
    }

    TEST(RagdollCoreGaitControllerTest, hingeGeometryPreservesNonPlanarLinksAndAnatomicalReach)
    {
        using namespace RagdollCore;
        HingeLegGeometry geometry;
        geometry.mUpperOffset = JPH::Vec3(0.15f, 0.2f, -1.0f);
        geometry.mLowerOffset = JPH::Vec3(-0.1f, -0.3f, -1.1f);
        geometry.mLowerRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.7f);
        geometry.mMinimumAngle = 0.2f;
        geometry.mMaximumAngle = 1.5f;
        for (const float requestedAngle : { 0.3f, 0.8f, 1.4f })
            for (const auto rotation : { JPH::Quat::sIdentity(),
                     JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.5f)
                         * JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.0f) })
            {
                std::vector<Transform> pose(3);
                pose[0].mTranslation = JPH::Vec3(2, 3, 4);
                // The sampled animation may contain a different knee position
                // and rotation. Fixed geometry, not that sample, defines reach.
                pose[1].mTranslation = JPH::Vec3(9, 8, 7);
                pose[1].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.4f);
                const auto flexion = JPH::Quat::sRotation(geometry.mHingeAxis, requestedAngle);
                TwoBoneLegTargetInput input;
                input.mKneeBodyIndex = 1;
                input.mFootBodyIndex = 2;
                input.mHingeGeometry = geometry;
                input.mTargetFootPosition = pose[0].mTranslation
                    + rotation * (geometry.mUpperOffset + flexion * geometry.mLowerOffset);
                input.mPreferredBendDirection = rotation * geometry.mUpperOffset;
                ASSERT_TRUE(applyTwoBoneLegTarget(pose, input));
                EXPECT_TRUE(pose[2].mTranslation.IsClose(input.mTargetFootPosition, 1e-8f));
                EXPECT_TRUE((pose[0].mRotation * geometry.mUpperOffset).IsClose(
                    pose[1].mTranslation - pose[0].mTranslation, 1e-8f));
                EXPECT_TRUE((pose[1].mRotation * (geometry.mLowerRotation.Conjugated() * geometry.mLowerOffset)).IsClose(
                    pose[2].mTranslation - pose[1].mTranslation, 1e-8f));
                const auto upperAxis = pose[0].mRotation * geometry.mHingeAxis;
                const auto lowerAxis = pose[1].mRotation * (geometry.mLowerRotation.Conjugated() * geometry.mHingeAxis);
                EXPECT_GT(upperAxis.Dot(lowerAxis), 0.99999f);
                const auto relative = (pose[0].mRotation.Conjugated() * pose[1].mRotation
                    * geometry.mLowerRotation.Conjugated()).Normalized();
                EXPECT_GT(std::abs(relative.Dot(flexion)), 0.99999f);
            }
        std::vector<Transform> pose(3);
        TwoBoneLegTargetInput input;
        input.mKneeBodyIndex = 1;
        input.mFootBodyIndex = 2;
        input.mHingeGeometry = geometry;
        input.mTargetFootPosition = JPH::Vec3(0, 0, -100);
        ASSERT_TRUE(applyTwoBoneLegTarget(pose, input));
        const auto relative = (pose[0].mRotation.Conjugated() * pose[1].mRotation
            * geometry.mLowerRotation.Conjugated()).Normalized();
        const float angle = 2.0f * std::atan2(relative.GetX(), relative.GetW());
        EXPECT_GE(angle, geometry.mMinimumAngle - 1e-5f);
        EXPECT_LE(angle, geometry.mMaximumAngle + 1e-5f);
        EXPECT_LT(pose[2].mTranslation.Length(), geometry.mUpperOffset.Length() + geometry.mLowerOffset.Length());
    }

    TEST(RagdollCoreGaitControllerTest, extendedNonPlanarLegDoesNotInventHipTwist)
    {
        using namespace RagdollCore;
        HingeLegGeometry geometry;
        geometry.mUpperOffset = JPH::Vec3(0, 0, -1);
        geometry.mLowerOffset = JPH::Vec3(0.1f, 0, -1);
        geometry.mMinimumAngle = -2;
        geometry.mMaximumAngle = 0;
        for (const float angle : { 0.0f, -0.001f, -0.01f, -0.1f })
        {
            SCOPED_TRACE(angle);
            std::vector<Transform> pose(3);
            pose[1].mTranslation = geometry.mUpperOffset;
            pose[2].mTranslation = geometry.mUpperOffset + geometry.mLowerOffset;
            TwoBoneLegTargetInput input;
            input.mKneeBodyIndex = 1;
            input.mFootBodyIndex = 2;
            input.mHingeGeometry = geometry;
            input.mTargetFootPosition = geometry.mUpperOffset
                + JPH::Quat::sRotation(geometry.mHingeAxis, angle) * geometry.mLowerOffset;
            ASSERT_TRUE(applyTwoBoneLegTarget(pose, input));
            EXPECT_TRUE(pose[2].mTranslation.IsClose(input.mTargetFootPosition, 1e-8f));
            // The fixed lateral offset is not a new knee-bend direction.
            // Reaching the unchanged endpoint must preserve the hinge plane.
            const auto hingeAxis = pose[0].mRotation * geometry.mHingeAxis;
            EXPECT_GT(hingeAxis.Dot(JPH::Vec3::sAxisX()), 0.9999f);
        }
    }

    TEST(RagdollCoreGaitControllerTest, rejectsInvalidHingeGeometryWithoutChangingThePose)
    {
        using namespace RagdollCore;
        HingeLegGeometry valid;
        valid.mUpperOffset = valid.mLowerOffset = JPH::Vec3(0, 0, -1);
        valid.mMinimumAngle = -2.0f;
        valid.mMaximumAngle = 0.1f;
        for (int invalid = 0; invalid < 5; ++invalid)
        {
            auto geometry = valid;
            switch (invalid)
            {
                case 0: geometry.mUpperOffset = JPH::Vec3(0, 0, 1e-5f); break;
                case 1: geometry.mHingeAxis *= 2; break;
                case 2: geometry.mMinimumAngle = 1; break;
                case 3: geometry.mLowerRotation = JPH::Quat(0, 0, 0, 0); break;
                case 4: geometry.mMaximumAngle = std::numeric_limits<float>::quiet_NaN(); break;
            }
            std::vector<Transform> pose(3);
            pose[1].mTranslation = JPH::Vec3(0, 0, -1);
            pose[2].mTranslation = JPH::Vec3(0, 0, -2);
            const auto before = pose;
            TwoBoneLegTargetInput input;
            input.mKneeBodyIndex = 1;
            input.mFootBodyIndex = 2;
            input.mTargetFootPosition = JPH::Vec3(1, 0, -1);
            input.mHingeGeometry = geometry;
            EXPECT_FALSE(applyTwoBoneLegTarget(pose, input));
            for (std::size_t i = 0; i < pose.size(); ++i)
            {
                EXPECT_TRUE(pose[i].mTranslation.IsClose(before[i].mTranslation, 0));
                EXPECT_TRUE(pose[i].mRotation.IsClose(before[i].mRotation, 0));
            }
        }
    }

    TEST(RagdollCoreGaitControllerTest, resolvesFootTargetAgainstMeasuredLegFrame)
    {
        RagdollCore::LegFrameTargetInput input;
        input.mBaseHip.mTranslation = JPH::Vec3(1.0f, 0.0f, 0.0f);
        input.mMeasuredPelvis.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.5f * std::numbers::pi_v<float>);
        input.mMeasuredHip.mTranslation = JPH::Vec3(10.0f, 0.0f, 0.0f);
        input.mTargetFootPosition = JPH::Vec3(10.0f, 2.0f, 0.0f);
        input.mPreferredBendDirection = JPH::Vec3::sAxisY();

        const auto result = RagdollCore::resolveLegFrameTarget(input);
        ASSERT_TRUE(result.has_value());
        EXPECT_TRUE(result->mTargetFootPosition.IsClose(JPH::Vec3(3.0f, 0.0f, 0.0f), 1e-5f));
        EXPECT_TRUE(result->mPreferredBendDirection.IsClose(JPH::Vec3::sAxisX(), 1e-5f));

        input.mMeasuredHip.mRotation = JPH::Quat(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 1.0f);
        EXPECT_FALSE(RagdollCore::resolveLegFrameTarget(input));
    }

    TEST(RagdollCoreGaitControllerTest, measuredSeededLegTargetMatchesMeasuredJointSeedAndChangesOnlyLeg)
    {
        using namespace RagdollCore;
        std::array<Transform, 5> target{};
        target[0].mTranslation = JPH::Vec3(0, 0, 3);
        target[1].mTranslation = JPH::Vec3(0, 0, 2);
        target[2].mTranslation = JPH::Vec3(0, 0, 1);
        target[3].mTranslation = JPH::Vec3(0, 0, 0);
        target[4].mTranslation = JPH::Vec3(9, 8, 7);
        auto measured = target;
        measured[0].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.35f);
        measured[1].mRotation = (measured[0].mRotation
            * JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.2f)).Normalized();
        measured[2].mRotation = (measured[1].mRotation
            * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.5f)).Normalized();
        measured[3].mRotation = (measured[2].mRotation
            * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.15f)).Normalized();
        const auto untouched = target[4];

        HingeLegGeometry geometry;
        geometry.mUpperOffset = JPH::Vec3(0, 0, -1);
        geometry.mLowerOffset = JPH::Vec3(0, 0, -1);
        geometry.mHingeAxis = JPH::Vec3::sAxisX();
        geometry.mMinimumAngle = -2.0f;
        geometry.mMaximumAngle = 0.1f;
        const TwoBoneLegTargetInput input{ 1, 2, 3, JPH::Vec3(0.3f, 0.0f, 0.2f),
            JPH::Vec3::sAxisY(), geometry };

        auto expected = target;
        const std::array<std::size_t, 3> leg{ 1, 2, 3 };
        ASSERT_TRUE(applyGaitMeasuredLegTargets(
            expected, measured, 0, leg, GaitMeasuredPoseFrame::Joint));
        ASSERT_TRUE(applyTwoBoneLegTarget(expected, input));

        ASSERT_TRUE(applyMeasuredSeededTwoBoneLegTarget(target, measured, 0, input));
        for (const std::size_t body : leg)
        {
            EXPECT_TRUE(target[body].mTranslation.IsClose(expected[body].mTranslation, 1e-6f));
            EXPECT_NEAR(std::abs(target[body].mRotation.Dot(expected[body].mRotation)), 1.0f, 1e-6f);
        }
        EXPECT_TRUE(target[4].mTranslation.IsClose(untouched.mTranslation, 0));
        EXPECT_TRUE(target[4].mRotation.IsClose(untouched.mRotation, 0));
    }

    TEST(RagdollCoreGaitControllerTest, measuredSeededLegTargetIsTransactionalOnFailure)
    {
        using namespace RagdollCore;
        std::array<Transform, 4> target{};
        target[1].mTranslation = JPH::Vec3(0, 0, 2);
        target[2].mTranslation = JPH::Vec3(0, 0, 1);
        auto measured = target;
        measured[1].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.4f);
        const auto before = target;
        TwoBoneLegTargetInput input{ 1, 2, 3, JPH::Vec3(0.2f, 0.0f, 0.0f), JPH::Vec3::sAxisY() };
        input.mHingeGeometry = HingeLegGeometry{};

        EXPECT_FALSE(applyMeasuredSeededTwoBoneLegTarget(target, measured, 0, input));
        for (std::size_t i = 0; i < target.size(); ++i)
        {
            EXPECT_TRUE(target[i].mTranslation.IsClose(before[i].mTranslation, 0));
            EXPECT_TRUE(target[i].mRotation.IsClose(before[i].mRotation, 0));
        }
    }

    TEST(RagdollCoreGaitControllerTest, clampsUnreachableLegTargetAndPreservesBendSide)
    {
        std::vector<RagdollCore::Transform> pose(3);
        pose[1].mTranslation = JPH::Vec3(0.0f, -0.25f, -1.0f);
        pose[2].mTranslation = JPH::Vec3(0.0f, 0.0f, -2.0f);
        RagdollCore::TwoBoneLegTargetInput input;
        input.mKneeBodyIndex = 1;
        input.mFootBodyIndex = 2;
        input.mTargetFootPosition = JPH::Vec3(100.0f, 0.0f, 0.0f);
        input.mPreferredBendDirection = -JPH::Vec3::sAxisY();

        ASSERT_TRUE(RagdollCore::applyTwoBoneLegTarget(pose, input));
        EXPECT_LT((pose[2].mTranslation - pose[0].mTranslation).Length(), 2.1f);
        EXPECT_GT((pose[2].mTranslation - pose[0].mTranslation).Length(), 1.9f);
        EXPECT_LT(pose[1].mTranslation.GetY(), 0.0f);
    }

    TEST(RagdollCoreGaitControllerTest, levelsFootAgainstMeasuredCalfAndFollowsPelvisYaw)
    {
        RagdollCore::LevelFootTargetInput input;
        input.mMeasuredCalfRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.3f);
        input.mPelvisRotationDelta = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.4f);
        const auto target = RagdollCore::calculateLevelFootTarget(input);
        ASSERT_TRUE(target.has_value());
        const JPH::Quat resultingWorldRotation = (input.mMeasuredCalfRotation * *target).Normalized();
        const JPH::Quat expected = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.4f);
        EXPECT_NEAR(std::abs(resultingWorldRotation.Dot(expected)), 1.0f, 1e-5f);

        input.mPelvisRotationDelta = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.4f);
        const auto pitchIgnored = RagdollCore::calculateLevelFootTarget(input);
        ASSERT_TRUE(pitchIgnored.has_value());
        const JPH::Quat levelWorldRotation = (input.mMeasuredCalfRotation * *pitchIgnored).Normalized();
        EXPECT_NEAR(std::abs(levelWorldRotation.Dot(JPH::Quat::sIdentity())), 1.0f, 1e-5f);

        input.mMeasuredCalfRotation = JPH::Quat(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 1.0f);
        EXPECT_FALSE(RagdollCore::calculateLevelFootTarget(input));
    }

    TEST(RagdollCoreGaitControllerTest, rejectsInvalidTwoBoneTargetTransactionally)
    {
        std::vector<RagdollCore::Transform> pose(3);
        pose[1].mTranslation = JPH::Vec3(0.0f, 0.0f, -1.0f);
        pose[2].mTranslation = JPH::Vec3(0.0f, 0.0f, -2.0f);
        const auto before = pose;
        RagdollCore::TwoBoneLegTargetInput input;
        input.mKneeBodyIndex = 1;
        input.mFootBodyIndex = 3;
        input.mTargetFootPosition = JPH::Vec3(1.0f, 0.0f, -1.0f);
        EXPECT_FALSE(RagdollCore::applyTwoBoneLegTarget(pose, input));
        for (std::size_t i = 0; i < pose.size(); ++i)
        {
            EXPECT_TRUE(pose[i].mTranslation.IsClose(before[i].mTranslation));
            EXPECT_TRUE(pose[i].mRotation.IsClose(before[i].mRotation));
        }
    }

    TEST(RagdollCoreGaitControllerTest, computesBoundedWorldFrameSwingHipTorque)
    {
        RagdollCore::GaitControllerProfile profile;
        profile.mSwingHipWorldStiffness = 100.0f;
        profile.mSwingHipWorldDamping = 10.0f;
        profile.mMaximumSwingHipWorldTorque = 50.0f;
        RagdollCore::WorldOrientationControllerInput input;
        input.mTargetRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.2f);
        input.mCurrentAngularVelocity = JPH::Vec3(0.5f, 0.0f, 0.0f);

        const auto control = RagdollCore::calculateSwingHipWorldControl(input, profile);
        ASSERT_TRUE(control.has_value());
        EXPECT_TRUE(control->mTorque.IsClose(JPH::Vec3(15.0f, 0.0f, 0.0f), 1e-4f));
        EXPECT_FALSE(control->mSaturated);

        profile.mMaximumSwingHipWorldTorque = 10.0f;
        const auto bounded = RagdollCore::calculateSwingHipWorldControl(input, profile);
        ASSERT_TRUE(bounded.has_value());
        EXPECT_NEAR(bounded->mTorque.Length(), 10.0f, 1e-4f);
        EXPECT_TRUE(bounded->mSaturated);

        profile.mMaximumSwingHipWorldTorque = 50.0f;
        input.mTargetRotation = -JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.2f);
        const auto equivalentSign = RagdollCore::calculateSwingHipWorldControl(input, profile);
        ASSERT_TRUE(equivalentSign.has_value());
        EXPECT_TRUE(equivalentSign->mTorque.IsClose(JPH::Vec3(15.0f, 0.0f, 0.0f), 1e-4f));
        EXPECT_FALSE(equivalentSign->mSaturated);

        input.mCurrentRotation = JPH::Quat(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 1.0f);
        EXPECT_FALSE(RagdollCore::calculateSwingHipWorldControl(input, profile));
    }

    TEST(RagdollCoreGaitControllerTest, computesBoundedVerticalSwingFootAssistance)
    {
        RagdollCore::GaitControllerProfile profile;
        profile.mSwingFootFrequency = 1.0f;
        profile.mSwingFootDamping = 1.0f;
        profile.mMaximumSwingFootForce = 1000.0f;
        RagdollCore::SwingFootControllerInput input;
        input.mTargetPosition = JPH::RVec3(10.0, 20.0, 1.0);
        input.mEffectiveMass = 2.0f;

        const auto control = RagdollCore::calculateSwingFootControl(input, profile);
        ASSERT_TRUE(control.has_value());
        EXPECT_FLOAT_EQ(control->mForce.GetX(), 0.0f);
        EXPECT_FLOAT_EQ(control->mForce.GetY(), 0.0f);
        EXPECT_NEAR(control->mForce.GetZ(), 8.0f * std::numbers::pi_v<float> * std::numbers::pi_v<float>, 1e-4f);
        EXPECT_FALSE(control->mSaturated);

        input.mTargetVelocity = JPH::Vec3(0.0f, 0.0f, 3.0f);
        const auto movingTarget = RagdollCore::calculateSwingFootControl(input, profile);
        ASSERT_TRUE(movingTarget.has_value());
        EXPECT_NEAR(movingTarget->mForce.GetZ(),
            8.0f * std::numbers::pi_v<float> * std::numbers::pi_v<float> + 24.0f * std::numbers::pi_v<float>, 1e-4f);
        input.mTargetVelocity = JPH::Vec3::sZero();

        input.mFootLifted = true;
        const auto airborne = RagdollCore::calculateSwingFootControl(input, profile);
        ASSERT_TRUE(airborne.has_value());
        EXPECT_GT(airborne->mForce.GetX(), 0.0f);
        EXPECT_GT(airborne->mForce.GetY(), 0.0f);

        profile.mMaximumSwingFootHorizontalForce = 10.0f;
        const auto horizontallyLimited = RagdollCore::calculateSwingFootControl(input, profile);
        ASSERT_TRUE(horizontallyLimited.has_value());
        JPH::Vec3 horizontal = horizontallyLimited->mForce;
        horizontal.SetZ(0.0f);
        EXPECT_NEAR(horizontal.Length(), 10.0f, 1e-4f);
        EXPECT_GT(horizontallyLimited->mForce.GetZ(), 10.0f)
            << "the horizontal cap must not also strengthen or truncate vertical tracking";
        EXPECT_TRUE(horizontallyLimited->mSaturated);

        input.mTargetPosition.SetZ(100.0);
        input.mFootLifted = false;
        profile.mMaximumSwingFootForce = 10.0f;
        const auto limited = RagdollCore::calculateSwingFootControl(input, profile);
        ASSERT_TRUE(limited.has_value());
        EXPECT_TRUE(limited->mForce.IsClose(JPH::Vec3(0.0f, 0.0f, 10.0f)));
        EXPECT_TRUE(limited->mSaturated);

        input.mEffectiveMass = 0.0f;
        EXPECT_FALSE(RagdollCore::calculateSwingFootControl(input, profile));
        input.mEffectiveMass = 2.0f;
        input.mTargetVelocity.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(RagdollCore::calculateSwingFootControl(input, profile));
    }
    TEST(RagdollCoreGaitControllerTest, sharedLegAssemblyResolvesTheMovingHipWithoutMovingTheRoot)
    {
        using namespace RagdollCore;
        std::array<Transform, 5> base{};
        base[0].mTranslation = JPH::Vec3(0, 0, 10);
        base[1].mTranslation = JPH::Vec3(1, 0, 9);
        base[2].mTranslation = JPH::Vec3(1, 1, 5);
        base[3].mTranslation = JPH::Vec3(1, 0, 1);
        base[4].mTranslation = JPH::Vec3(12, 3, 7);
        const auto delta = (JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.3f)
            * JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.4f)).Normalized();
        const JPH::Vec3 translation(5, -3, 2);
        auto measured = base;
        for (auto& body : measured)
        {
            body.mTranslation = translation + delta * body.mTranslation;
            body.mRotation = delta;
        }
        const JPH::Vec3 referenceEndpoint(1, 1.5f, 2);
        const auto endpoint = translation + delta * referenceEndpoint;
        auto target = base;
        const TwoBoneLegTargetInput input{ 1, 2, 3, endpoint, delta * JPH::Vec3::sAxisY() };
        const auto result = applySwingLegPose(target, base, measured, 0, input, {});
        ASSERT_TRUE(result);
        ASSERT_TRUE(result->mApplied);
        EXPECT_LT((target[3].mTranslation - referenceEndpoint).Length(), 1e-4f);
        const auto achieved = measured[1].mTranslation + delta * (target[3].mTranslation - base[1].mTranslation);
        EXPECT_LT((achieved - endpoint).Length(), 1e-4f);
        EXPECT_LT(std::abs((target[2].mTranslation - target[1].mTranslation).Length() - std::sqrt(17.0f)), 1e-4f);
        EXPECT_LT(std::abs((target[3].mTranslation - target[2].mTranslation).Length() - std::sqrt(17.0f)), 1e-4f);
        EXPECT_TRUE(target[0].mTranslation == base[0].mTranslation);
        EXPECT_TRUE(target[0].mRotation == base[0].mRotation);
        EXPECT_TRUE(target[4].mTranslation == base[4].mTranslation);
        EXPECT_TRUE(target[4].mRotation == base[4].mRotation);
        EXPECT_TRUE(base[3].mTranslation == JPH::Vec3(1, 0, 1));
        EXPECT_TRUE(measured[3].mRotation == delta);
    }

    TEST(RagdollCoreGaitControllerTest, sharedLegTouchdownPreservesMeasuredJointRotations)
    {
        using namespace RagdollCore;
        std::array<Transform, 4> base{};
        base[0].mTranslation = JPH::Vec3(0, 0, 10);
        base[1].mTranslation = JPH::Vec3(1, 0, 9);
        base[2].mTranslation = JPH::Vec3(1, 1, 5);
        base[3].mTranslation = JPH::Vec3(1, 0, 1);
        const auto delta = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.5f);
        auto measured = base;
        for (std::size_t i = 0; i < measured.size(); ++i)
        {
            measured[i].mTranslation = delta * base[i].mTranslation;
            measured[i].mRotation = (delta * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.1f * static_cast<float>(i))).Normalized();
        }
        GaitActuationDecision decision;
        decision.mPoseSource = GaitPoseSource::MeasuredTouchdown;
        for (const auto frame : { GaitMeasuredPoseFrame::Joint, GaitMeasuredPoseFrame::Model })
        {
            decision.mMeasuredPoseFrame = frame;
            auto target = base;
            const TwoBoneLegTargetInput input{ 1, 2, 3, delta * JPH::Vec3(1, 1.5f, 2), delta * JPH::Vec3::sAxisY() };
            const auto result = applySwingLegPose(target, base, measured, 0, input, decision);
            ASSERT_TRUE(result);
            ASSERT_TRUE(result->mApplied);
            for (std::size_t i = 1; i < target.size(); ++i)
            {
                const auto expected = frame == GaitMeasuredPoseFrame::Joint
                    ? JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.1f * static_cast<float>(i)) : measured[i].mRotation;
                EXPECT_GT(std::abs(target[i].mRotation.Dot(expected)), 1.0f - 1e-6f);
            }
            EXPECT_LT((target[3].mTranslation - JPH::Vec3(1, 1.5f, 2)).Length(), 1e-4f);
        }
    }

    TEST(RagdollCoreGaitControllerTest, sharedLegAssemblyDoesNotPublishPartialOrDegenerateTargets)
    {
        using namespace RagdollCore;
        std::array<Transform, 4> base{};
        base[0].mTranslation = JPH::Vec3(0, 0, 10);
        base[1].mTranslation = JPH::Vec3(1, 0, 9);
        base[2].mTranslation = JPH::Vec3(1, 1, 5);
        base[3].mTranslation = JPH::Vec3(1, 0, 1);
        const auto unchanged = [&](const auto& target) {
            for (std::size_t i = 0; i < target.size(); ++i)
            {
                EXPECT_TRUE(target[i].mTranslation == base[i].mTranslation);
                EXPECT_TRUE(target[i].mRotation == base[i].mRotation);
            }
        };
        auto measured = base;
        measured[3].mRotation = JPH::Quat(0, 0, 0, std::numeric_limits<float>::quiet_NaN());
        auto target = base;
        GaitActuationDecision decision;
        decision.mPoseSource = GaitPoseSource::MeasuredTouchdown;
        TwoBoneLegTargetInput input{ 1, 2, 3, JPH::Vec3(1, 1.5f, 2), JPH::Vec3::sAxisY() };
        EXPECT_FALSE(applySwingLegPose(target, base, measured, 0, input, decision));
        unchanged(target);
        measured = base;
        input.mTargetFootPosition = base[1].mTranslation;
        const auto degenerate = applySwingLegPose(target, base, measured, 0, input, {});
        ASSERT_TRUE(degenerate);
        EXPECT_FALSE(degenerate->mApplied);
        unchanged(target);
        input.mHipBodyIndex = 0;
        EXPECT_FALSE(applySwingLegPose(target, base, measured, 0, input, {}));
        unchanged(target);
    }

    TEST(RagdollCoreGaitControllerTest, recoveryRootTorqueUsesMeasuredSupportAndRetainsAnInternalFallback)
    {
        using namespace RagdollCore;
        GaitActuationDecisionInput input;
        input.mRecovering = true;
        input.mDriveJointMotors = true;
        const auto decision = calculateGaitActuationDecision(input, {});
        ASSERT_TRUE(decision);
        EXPECT_EQ(decision->mPhase, GaitPhase::Idle);
        SupportEstimate support;
        auto reactions = selectGaitReactionBodies(*decision, support, {3, 6}, 1, 4);
        EXPECT_EQ(reactions.mAngular, 1u);
        EXPECT_TRUE(reactions.mVertical.empty());
        EXPECT_TRUE(reactions.mHorizontal.empty());
        support.mRightFootSupported = true;
        reactions = selectGaitReactionBodies(*decision, support, {3, 6}, 1, 4);
        EXPECT_EQ(reactions.mAngular, 4u);
        EXPECT_EQ(reactions.mVertical, std::vector<std::size_t>{6});
        EXPECT_FALSE(reactions.mSecondaryAngular);
        support.mLeftFootSupported = true;
        reactions = selectGaitReactionBodies(*decision, support, {3, 6}, 1, 4);
        EXPECT_EQ(reactions.mAngular, 1u);
        EXPECT_EQ(reactions.mSecondaryAngular, 4u);

        reactions = selectGaitReactionBodies(*decision, support, {3, 6}, 1, 4, true);
        EXPECT_EQ(reactions.mAngular, 3u);
        EXPECT_EQ(reactions.mSecondaryAngular, 6u);
        support.mLeftFootSupported = false;
        reactions = selectGaitReactionBodies(*decision, support, {3, 6}, 1, 4, true);
        EXPECT_EQ(reactions.mAngular, 6u);
        EXPECT_FALSE(reactions.mSecondaryAngular);
        support.mRightFootSupported = false;
        reactions = selectGaitReactionBodies(*decision, support, {3, 6}, 1, 4, true);
        EXPECT_EQ(reactions.mAngular, 1u);
        EXPECT_FALSE(reactions.mSecondaryAngular);
    }

}
