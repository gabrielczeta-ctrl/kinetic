#include "fixtures.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace
{
    TEST(RagdollCoreProfileTest, convexHullAcceptsVolumeWithInteriorAndDuplicatePoints)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& shape = profile.mBodies[0].mShape;
        shape.mType = RagdollCore::ShapeType::ConvexHull;
        shape.mPoints = { JPH::Vec3(0, 0, 0), JPH::Vec3(2, 0, 0), JPH::Vec3(0, 3, 0),
            JPH::Vec3(0, 0, 4), JPH::Vec3(0.1f, 0.1f, 0.1f), JPH::Vec3(0, 0, 0) };
        EXPECT_TRUE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        const auto original = shape.mPoints;
        const auto rotation = JPH::Quat::sRotation(JPH::Vec3(1, 2, 3).Normalized(), 0.7f);
        for (float scale : { 0.01f, 1000.f })
        {
            shape.mPoints.clear();
            for (const auto& point : original)
                shape.mPoints.push_back(rotation * (point * scale) + JPH::Vec3(7, -3, 2));
            EXPECT_TRUE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        }
    }

    TEST(RagdollCoreProfileTest, convexHullRejectsDegenerateAndNonfinitePoints)
    {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const float infinity = std::numeric_limits<float>::infinity();
        const std::vector<std::vector<JPH::Vec3>> invalid = {
            {}, { JPH::Vec3(0, 0, 0), JPH::Vec3(1, 0, 0), JPH::Vec3(0, 1, 0) },
            std::vector<JPH::Vec3>(4, JPH::Vec3(1, 2, 3)),
            { JPH::Vec3(0, 0, 0), JPH::Vec3(1, 1, 1), JPH::Vec3(2, 2, 2), JPH::Vec3(3, 3, 3) },
            { JPH::Vec3(0, 0, 0), JPH::Vec3(1, 1, 0), JPH::Vec3(0, 1, 1), JPH::Vec3(1, 2, 1) },
            { JPH::Vec3(0, 0, 0), JPH::Vec3(1, 0, 0), JPH::Vec3(0, 1, 0), JPH::Vec3(0, 0, nan) },
            { JPH::Vec3(0, 0, 0), JPH::Vec3(1, 0, 0), JPH::Vec3(0, 1, 0), JPH::Vec3(0, 0, infinity) },
        };
        for (std::size_t i = 0; i < invalid.size(); ++i)
        {
            SCOPED_TRACE(i);
            auto profile = RagdollCoreTest::makeProfile();
            profile.mBodies[0].mShape.mType = RagdollCore::ShapeType::ConvexHull;
            profile.mBodies[0].mShape.mPoints = invalid[i];
            const auto result = RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton());
            ASSERT_FALSE(result.isValid());
            EXPECT_EQ(result.mIssues.front().mCode, RagdollCore::ValidationCode::InvalidShape);
        }
    }

    TEST(RagdollCoreProfileTest, convexHullPointsCannotSilentlyBecomePrimitiveOrFittedCapsule)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& shape = profile.mBodies[0].mShape;
        shape.mPoints = { JPH::Vec3(0, 0, 0), JPH::Vec3(2, 0, 0), JPH::Vec3(0, 3, 0), JPH::Vec3(0, 0, 4) };
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        shape.mType = RagdollCore::ShapeType::ConvexHull;
        shape.mFitToBone = "torso";
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
    }

    TEST(RagdollCoreProfileTest, acceptsValidatedPlainProfile)
    {
        const auto result = RagdollCore::validateProfile(
            RagdollCoreTest::makeProfile(), RagdollCoreTest::makeSkeleton());
        EXPECT_TRUE(result.isValid());
        EXPECT_TRUE(result.mIssues.empty());
    }

    TEST(RagdollCoreProfileTest, rejectsUnknownSchema)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mSchemaVersion = RagdollCore::CurrentProfileSchemaVersion + 1;
        const auto result = RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton());
        ASSERT_FALSE(result.isValid());
        EXPECT_EQ(result.mIssues.front().mCode, RagdollCore::ValidationCode::UnsupportedSchema);
    }

    TEST(RagdollCoreProfileTest, rejectsNonTopologicalSkeleton)
    {
        auto skeleton = RagdollCoreTest::makeSkeleton();
        skeleton.mBones[0].mParent = 1;
        const auto result = RagdollCore::validateSkeleton(skeleton);
        EXPECT_FALSE(result.isValid());
    }

    TEST(RagdollCoreProfileTest, rejectsDuplicateBodyAndInvalidMassSum)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[1].mName = profile.mBodies[0].mName;
        profile.mBodies[1].mMassFraction = 0.2f;
        const auto result = RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton());
        EXPECT_FALSE(result.isValid());
        EXPECT_GE(result.mIssues.size(), 2u);
    }

    TEST(RagdollCoreProfileTest, rejectsNaNDimensions)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[1].mShape.mRadius = std::numeric_limits<float>::quiet_NaN();
        const auto result = RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton());
        EXPECT_FALSE(result.isValid());
    }

    TEST(RagdollCoreProfileTest, rejectsInvalidImpactResponse)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mImpactResponse.mMinimumImpulse = 100.0f;
        profile.mImpactResponse.mMaximumImpulse = 50.0f;
        profile.mImpactResponse.mUpwardBias = 2.0f;
        const auto result = RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton());
        ASSERT_FALSE(result.isValid());
        EXPECT_EQ(result.mIssues.front().mCode, RagdollCore::ValidationCode::InvalidNumber);
    }

    TEST(RagdollCoreProfileTest, rejectsBodyParentThatIsNotASkeletonAncestor)
    {
        auto skeleton = RagdollCoreTest::makeSkeleton();
        skeleton.mBones.push_back({ "other_root_child", 0, {} });
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[1].mBone = "other_root_child";
        profile.mBodies[1].mParentBody = "pelvis";
        EXPECT_TRUE(RagdollCore::validateProfile(profile, skeleton).isValid());

        skeleton.mBones.push_back({ "torso_child", 1, {} });
        profile.mBodies[1].mBone = "torso_child";
        profile.mBodies[0].mBone = "other_root_child";
        EXPECT_FALSE(RagdollCore::validateProfile(profile, skeleton).isValid());
    }

    TEST(RagdollCoreProfileTest, rejectsInvalidCapsuleFitTargets)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[0].mShape.mFitToBone = "missing";
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile.mBodies[0].mShape.mFitToBone = "torso";
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile.mBodies[0].mShape.mType = RagdollCore::ShapeType::Capsule;
        EXPECT_TRUE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile.mBodies[0].mShape.mLocalTransform.mTranslation = JPH::Vec3::sAxisX();
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
    }

    TEST(RagdollCoreProfileTest, validatesHingeLimitsAroundTheBindPose)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& constraint = *profile.mBodies[1].mConstraint;
        constraint.mType = RagdollCore::ConstraintType::Hinge;
        constraint.mHingeMinAngle = -0.15f;
        constraint.mHingeMaxAngle = 2.0f;
        EXPECT_TRUE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        constraint.mHingeMinAngle = 0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        constraint.mHingeMinAngle = -0.15f;
        constraint.mHingeMaxAngle = -0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        constraint.mHingeMaxAngle = 0.1f;
        constraint.mType = static_cast<RagdollCore::ConstraintType>(99);
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
    }

    TEST(RagdollCoreProfileTest, validatesMotionTransferBounds)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mMotionTransfer.mMaximumSampleDuration = 0.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile.mMotionTransfer.mMaximumSampleDuration = 0.1f;
        profile.mMotionTransfer.mMaximumLinearSpeed = -1.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile.mMotionTransfer.mMaximumLinearSpeed = 1000.0f;
        profile.mMotionTransfer.mMaximumAngularSpeed = 20.0f;
        EXPECT_TRUE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
    }

    TEST(RagdollCoreProfileTest, validatesPoseTransitionDuration)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mPoseTransition.mDeathBlendDuration = 0.0f;
        EXPECT_TRUE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile.mPoseTransition.mDeathBlendDuration = -0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile.mPoseTransition.mDeathBlendDuration = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
    }

    TEST(RagdollCoreProfileTest, validatesBoundedJointMotors)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& motor = profile.mBodies[1].mConstraint->mMotor;
        motor.mFrequency = 6.0f;
        motor.mDamping = 0.8f;
        motor.mMaximumTorque = 1000.0f;
        EXPECT_TRUE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        motor.mMaximumTorque = -1.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        motor.mMaximumTorque = 1000.0f;
        motor.mFrequency = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
    }

    TEST(RagdollCoreProfileTest, validatesActiveControllerThresholds)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mStandingMinimumUpDot = 0.9f;
        profile.mActiveController.mBracingMinimumUpDot = 0.7f;
        profile.mActiveController.mStandingMaximumSpeed = 50.0f;
        profile.mActiveController.mSettledMaximumSpeed = 10.0f;
        EXPECT_TRUE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile.mActiveController.mBracingMinimumUpDot = 0.95f;
        auto result = RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton());
        ASSERT_FALSE(result.isValid());
        EXPECT_EQ(result.mIssues.front().mCode, RagdollCore::ValidationCode::InvalidController);

        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mRecoveryDelay = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mFallingExitSupportDuration = -0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile.mActiveController.mFallingExitSupportDuration = 0.2f;
        profile.mActiveController.mFallingExitMaximumVerticalSpeed
            = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
    }

    TEST(RagdollCoreProfileTest, validatesActiveControllerUprightGainsAndLimits)
    {
        auto profile = RagdollCoreTest::makeProfile();
        EXPECT_TRUE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile.mActiveController.mMaximumPelvisForce = -1.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile.mActiveController.mMaximumPelvisForce = 1.0f;
        profile.mActiveController.mMaximumLocomotionSpeed = -1.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile.mActiveController.mMaximumLocomotionSpeed = 400.0f;
        profile.mActiveController.mLocomotionSupportGraceDuration = -0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile.mActiveController.mLocomotionSupportGraceDuration = 0.15f;
        profile.mActiveController.mPelvisHeightSettleDuration = -0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile.mActiveController.mPelvisHeightSettleDuration = 0.25f;
        profile.mActiveController.mPelvisHeightSettleMaximumVerticalSpeed = -0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile.mActiveController.mPelvisHeightSettleMaximumVerticalSpeed = 5.0f;
        profile.mActiveController.mActivationSupportGraceDuration = -0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile.mActiveController.mActivationSupportGraceDuration = 0.15f;
        profile.mActiveController.mRecoveryUnweightFactor = 3.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile.mActiveController.mRecoveryUnweightFactor = 2.5f;
        profile.mActiveController.mSpineUprightDamping = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
    }

    TEST(RagdollCoreProfileTest, validatesAnimationAssistedHeadingBounds)
    {
        auto profile = RagdollCoreTest::makeProfile();
        EXPECT_TRUE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile.mActiveController.mAnimationAssistYawReferenceMaximumRate = 0.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mAnimationAssistYawReferenceMaximumAcceleration
            = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mMaximumAnimationAssistYawTorque
            = profile.mActiveController.mMaximumAnimationAssistTorque + 1.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mAnimationAssistSupportMotionFrequency = 0.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mAnimationAssistSupportMotionDamping = -0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mAnimationAssistSupportPositionLeash
            = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mAnimationAssistContactConfirmationTicks = 0;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mAnimationAssistLoadTransferDuration = 0.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
    }

    TEST(RagdollCoreProfileTest, validatesGaitControllerDurationsAndPlacementBounds)
    {
        auto profile = RagdollCoreTest::makeProfile();
        EXPECT_TRUE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());

        profile.mActiveController.mGait.mNominalStepDuration = 0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mGait.mMinimumStepDuration = 0.0f;
        profile.mActiveController.mGait.mNominalStepDuration = 0.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mGait.mMaximumStepDuration = 0.2f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mGait.mMaximumTouchdownHorizontalDistance = -0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mGait.mMaximumLandingApproachVerticalDistance = -0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mGait.mMaximumTouchdownRelativeVerticalSpeed = -0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mGait.mNominalStepWidth
            = 2.0f * profile.mActiveController.mGait.mMaximumStepReach + 1.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mGait.mCapturePointBlend = -0.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mGait.mMaximumSwingFootHorizontalForce = -1.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mGait.mPreLiftSwingUnloadFraction = 1.1f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mGait.mSwingLegMotorTorqueScale = 0.5f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mGait.mMaximumSwingHipTargetSpeed = 0.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
        profile = RagdollCoreTest::makeProfile();
        profile.mActiveController.mGait.mSwingHipWorldStiffness = -1.0f;
        EXPECT_FALSE(RagdollCore::validateProfile(profile, RagdollCoreTest::makeSkeleton()).isValid());
    }
}
