#include <components/ragdollprofile/loader.hpp>
#include <components/vfs/filesystemarchive.hpp>
#include <components/vfs/manager.hpp>

#include <ragdollcore/builder.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <unordered_map>

namespace
{
    TEST(RagdollProfileLoaderTest, loadsAndBuildsExplicitConvexHullFixture)
    {
        const auto path = std::filesystem::path{ OPENMW_PROJECT_SOURCE_DIR }
            / "apps/components_tests/ragdollprofile/data/convex-hull-v1.yaml";
        std::ifstream input(path);
        ASSERT_TRUE(input);
        const auto loaded = RagdollProfile::load(input);
        ASSERT_TRUE(loaded.isValid());
        const auto& shape = loaded.mProfile->mBodies.front().mShape;
        EXPECT_EQ(shape.mType, RagdollCore::ShapeType::ConvexHull);
        ASSERT_EQ(shape.mPoints.size(), 8u);
        EXPECT_TRUE(shape.mPoints.front().IsClose(JPH::Vec3(-1, -50, -1)));
        RagdollCore::SkeletonDescriptor skeleton;
        skeleton.mId = "convex_hull_fixture";
        skeleton.mBones.push_back({ "pelvis", -1, {} });
        const auto built = RagdollCore::buildRagdollSettings(skeleton, *loaded.mProfile, {});
        ASSERT_TRUE(built.isValid());
        EXPECT_TRUE(built.mSettings->mParts.front().GetShape()->GetCenterOfMass().IsNearZero());
    }

    TEST(RagdollProfileLoaderTest, rejectsMalformedExplicitConvexHullPointLists)
    {
        for (const std::string& shape : {
                 "{ type: convex_hull }",
                 "{ type: convex_hull, points: [] }",
                 "{ type: convex_hull, points: { x: 0 } }",
                 "{ type: convex_hull, points: [[0,0,0],[1,0,0],[0,1,0],[0,1]] }",
                 "{ type: sphere, radius: 2, points: [[0,0,0],[1,0,0],[0,1,0],[0,0,1]] }" })
        {
            SCOPED_TRACE(shape);
            std::istringstream input("schema_version: 1\nid: invalid_hull\nbodies:\n"
                "  - name: pelvis\n    bone: pelvis\n    mass_fraction: 1\n    shape: " + shape
                + "\nlandmarks: { pelvis: pelvis }\n");
            EXPECT_FALSE(RagdollProfile::load(input).isValid());
        }
    }

    TEST(RagdollProfileLoaderTest, parsedCoplanarConvexHullFailsCoreSemanticValidation)
    {
        std::istringstream input(R"(
schema_version: 1
id: flat_hull
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 1
    shape: { type: convex_hull, points: [[0,0,0],[1,0,0],[0,1,0],[1,1,0]] }
landmarks: { pelvis: pelvis }
)");
        const auto loaded = RagdollProfile::load(input);
        ASSERT_TRUE(loaded.isValid());
        RagdollCore::SkeletonDescriptor skeleton;
        skeleton.mId = "flat_hull";
        skeleton.mBones.push_back({ "pelvis", -1, {} });
        const auto built = RagdollCore::buildRagdollSettings(skeleton, *loaded.mProfile, {});
        ASSERT_FALSE(built.isValid());
        EXPECT_EQ(built.mValidation.mIssues.front().mCode, RagdollCore::ValidationCode::InvalidShape);
    }

    TEST(RagdollProfileLoaderTest, loadsMinimalProfileFromStream)
    {
        std::istringstream input(R"(
schema_version: 1
id: minimal
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 1.0
    shape: { type: sphere, radius: 2.0 }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_TRUE(result.isValid());
        ASSERT_EQ(result.mProfile->mBodies.size(), 1u);
        EXPECT_EQ(result.mProfile->mBodies.front().mShape.mType, RagdollCore::ShapeType::Sphere);
    }

    TEST(RagdollProfileLoaderTest, rejectsUnknownShapeType)
    {
        std::istringstream input(R"(
schema_version: 1
id: invalid
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 1.0
    shape: { type: mesh }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_FALSE(result.isValid());
        ASSERT_EQ(result.mIssues.size(), 1u);
        EXPECT_NE(result.mIssues.front().mMessage.find("shape type"), std::string::npos);
    }

    TEST(RagdollProfileLoaderTest, loadsSkeletonDerivedConstraintFrameFlag)
    {
        std::istringstream input(R"(
schema_version: 1
id: derived_constraint
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 0.5
    shape: { type: sphere, radius: 2.0 }
  - name: torso
    bone: torso
    parent: pelvis
    mass_fraction: 0.5
    shape: { type: sphere, radius: 2.0 }
    constraint: { derive_frame: true }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_TRUE(result.isValid());
        ASSERT_TRUE(result.mProfile->mBodies[1].mConstraint.has_value());
        EXPECT_TRUE(result.mProfile->mBodies[1].mConstraint->mDeriveFrameFromSkeleton);
    }

    TEST(RagdollProfileLoaderTest, loadsSkeletonDerivedConstraintFrameAxisBone)
    {
        std::istringstream input(R"(
schema_version: 1
id: derived_constraint_axis
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 0.5
    shape: { type: sphere, radius: 2.0 }
  - name: thigh
    bone: thigh
    parent: pelvis
    mass_fraction: 0.5
    shape: { type: capsule, radius: 1.0, half_height: 2.0 }
    constraint: { derive_frame: true, frame_axis_bone: calf }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_TRUE(result.isValid());
        ASSERT_TRUE(result.mProfile->mBodies[1].mConstraint.has_value());
        ASSERT_TRUE(result.mProfile->mBodies[1].mConstraint->mFrameAxisBone.has_value());
        EXPECT_EQ(*result.mProfile->mBodies[1].mConstraint->mFrameAxisBone, "calf");
    }

    TEST(RagdollProfileLoaderTest, loadsHingeConstraintAndLimits)
    {
        std::istringstream input(R"(
schema_version: 1
id: hinge_constraint
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 0.5
    shape: { type: sphere, radius: 2.0 }
  - name: calf
    bone: calf
    parent: pelvis
    mass_fraction: 0.5
    shape: { type: capsule, radius: 1.0, half_height: 2.0 }
    constraint: { type: hinge, derive_frame: true, limit_min_angle: -2.1, limit_max_angle: 0.12 }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_TRUE(result.isValid());
        ASSERT_TRUE(result.mProfile->mBodies[1].mConstraint.has_value());
        const auto& constraint = *result.mProfile->mBodies[1].mConstraint;
        EXPECT_EQ(constraint.mType, RagdollCore::ConstraintType::Hinge);
        EXPECT_TRUE(constraint.mDeriveFrameFromSkeleton);
        EXPECT_FLOAT_EQ(constraint.mHingeMinAngle, -2.1f);
        EXPECT_FLOAT_EQ(constraint.mHingeMaxAngle, 0.12f);
    }

    TEST(RagdollProfileLoaderTest, loadsMotionTransferBounds)
    {
        std::istringstream input(R"(
schema_version: 1
id: motion_transfer
motion_transfer:
  maximum_sample_duration: 0.08
  maximum_linear_speed: 1500
  maximum_angular_speed: 24
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 1.0
    shape: { type: sphere, radius: 2.0 }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_TRUE(result.isValid());
        EXPECT_FLOAT_EQ(result.mProfile->mMotionTransfer.mMaximumSampleDuration, 0.08f);
        EXPECT_FLOAT_EQ(result.mProfile->mMotionTransfer.mMaximumLinearSpeed, 1500.0f);
        EXPECT_FLOAT_EQ(result.mProfile->mMotionTransfer.mMaximumAngularSpeed, 24.0f);
    }

    TEST(RagdollProfileLoaderTest, loadsPoseTransitionTuning)
    {
        std::istringstream input(R"(
schema_version: 1
id: pose_transition
pose_transition:
  death_blend_duration: 0.125
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 1.0
    shape: { type: sphere, radius: 2.0 }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_TRUE(result.isValid());
        EXPECT_FLOAT_EQ(result.mProfile->mPoseTransition.mDeathBlendDuration, 0.125f);
    }

    TEST(RagdollProfileLoaderTest, rejectsRemovedLocomotionRoutes)
    {
        for (const char* route : { "animation_assisted", "animation_load_assisted", "animation_contact_root",
                 "animation_retimed" })
        {
            SCOPED_TRACE(route);
            std::istringstream input(std::string(R"(
schema_version: 1
id: removed
active_controller: { locomotion_route: )") + route + R"( }
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 1.0
    shape: { type: sphere, radius: 2.0 }
landmarks: { pelvis: pelvis }
)");
            const auto result = RagdollProfile::load(input);
            EXPECT_FALSE(result.isValid());
            EXPECT_FALSE(result.mIssues.empty());
        }
    }

    TEST(RagdollProfileLoaderTest, loadsKinematicRootRoute)
    {
        std::istringstream input(R"(
schema_version: 1
id: kinematic
active_controller: { locomotion_route: kinematic_root }
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 1.0
    shape: { type: sphere, radius: 2.0 }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_TRUE(result.isValid());
        EXPECT_EQ(result.mProfile->mActiveController.mLocomotionRoute,
            RagdollCore::LocomotionRoute::KinematicRoot);
    }

    TEST(RagdollProfileLoaderTest, loadsBoundedJointMotorTuning)
    {
        std::istringstream input(R"(
schema_version: 1
id: motor
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 0.5
    shape: { type: sphere, radius: 2.0 }
  - name: torso
    bone: torso
    parent: pelvis
    mass_fraction: 0.5
    shape: { type: sphere, radius: 2.0 }
    constraint:
      motor: { frequency: 6.0, damping: 0.8, maximum_torque: 1234.0 }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_TRUE(result.isValid());
        const auto& motor = result.mProfile->mBodies[1].mConstraint->mMotor;
        EXPECT_FLOAT_EQ(motor.mFrequency, 6.0f);
        EXPECT_FLOAT_EQ(motor.mDamping, 0.8f);
        EXPECT_FLOAT_EQ(motor.mMaximumTorque, 1234.0f);
    }

    TEST(RagdollProfileLoaderTest, loadsActiveControllerThresholds)
    {
        std::istringstream input(R"(
schema_version: 1
id: active_controller
active_controller:
  locomotion_route: kinematic_root
  support_normal_minimum_dot: 0.6
  support_margin: 5.0
  standing_minimum_up_dot: 0.92
  bracing_minimum_up_dot: 0.7
  standing_maximum_speed: 50.0
  settled_maximum_speed: 10.0
  fallen_maximum_pelvis_height: 25.0
  minimum_state_duration: 0.1
  falling_exit_support_duration: 0.25
  falling_exit_maximum_vertical_speed: 90.0
  recovery_delay: 0.75
  blend_back_duration: 0.3
  pelvis_height_frequency: 3.5
  pelvis_height_damping: 0.9
  center_of_mass_frequency: 1.5
  center_of_mass_damping: 0.8
  maximum_locomotion_speed: 375.0
  pelvis_height_settle_duration: 0.3
  pelvis_height_settle_maximum_vertical_speed: 4.0
  activation_support_grace_duration: 0.18
  locomotion_support_grace_duration: 0.2
  maximum_pelvis_force: 50000.0
  animation_assist_horizontal_scale: 0.6
  animation_assist_vertical_scale: 0.8
  maximum_animation_assist_force: 42000.0
  animation_assist_upright_scale: 0.7
  maximum_animation_assist_torque: 73000.0
  animation_assist_pose_tracking_blend: 0.65
  animation_assist_stance_anchor_compliance: 0.4
  animation_assist_support_loss_handoff_duration: 0.2
  animation_assist_stance_position_correction: 0.7
  animation_assist_stance_maximum_endpoint_correction: 0.9
  animation_assist_support_motion_frequency: 0.55
  animation_assist_support_motion_damping: 1.1
  animation_assist_support_position_leash: 3.0
  animation_assist_contact_confirmation_ticks: 6
  animation_assist_load_transfer_duration: 0.2
  animation_assist_measured_progress_blend: 0.75
  animation_assist_feasibility_deadline: 0.15
  animation_assist_double_support_transfer_duration: 0.25
  animation_assist_reentry_slip_settle_duration: 0.3
  animation_assist_touchdown_capture_position_correction: 0.45
  animation_assist_touchdown_capture_velocity_lookahead: 0.035
  animation_assist_outgoing_hold_position_correction: 0.25
  animation_assist_outgoing_hold_velocity_lookahead: 0.015
  recovery_unweight_factor: 1.4
  pelvis_upright_stiffness: 19000.0
  pelvis_upright_damping: 2900.0
  maximum_pelvis_torque: 28000.0
  spine_upright_stiffness: 11000.0
  spine_upright_damping: 1900.0
  maximum_spine_torque: 18000.0
  gait:
    minimum_step_duration: 0.16
    nominal_step_duration: 0.38
    maximum_step_duration: 0.7
    contact_confirmation_duration: 0.04
    stance_confirmation_duration: 0.12
    maximum_touchdown_horizontal_distance: 5.5
    maximum_landing_approach_vertical_distance: 2.75
    maximum_touchdown_relative_vertical_speed: 4.5
    maximum_step_reach: 52.0
    nominal_step_width: 18.0
    capture_point_blend: 0.4
    swing_foot_lift: 12.0
    swing_foot_frequency: 5.0
    swing_foot_damping: 0.85
    maximum_swing_foot_force: 17500.0
    maximum_swing_foot_horizontal_force: 9000.0
    pre_lift_swing_unload_fraction: 0.45
    swing_leg_motor_torque_scale: 3.0
    maximum_swing_hip_target_speed: 4.5
    swing_hip_world_stiffness: 6000.0
    swing_hip_world_damping: 700.0
    maximum_swing_hip_world_torque: 8000.0
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 1.0
    shape: { type: sphere, radius: 2.0 }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_TRUE(result.isValid());
        const auto& controller = result.mProfile->mActiveController;
        EXPECT_FLOAT_EQ(controller.mSupportNormalMinimumDot, 0.6f);
        EXPECT_FLOAT_EQ(controller.mSupportMargin, 5.0f);
        EXPECT_FLOAT_EQ(controller.mStandingMinimumUpDot, 0.92f);
        EXPECT_FLOAT_EQ(controller.mBracingMinimumUpDot, 0.7f);
        EXPECT_FLOAT_EQ(controller.mStandingMaximumSpeed, 50.0f);
        EXPECT_FLOAT_EQ(controller.mSettledMaximumSpeed, 10.0f);
        EXPECT_FLOAT_EQ(controller.mFallenMaximumPelvisHeight, 25.0f);
        EXPECT_FLOAT_EQ(controller.mMinimumStateDuration, 0.1f);
        EXPECT_FLOAT_EQ(controller.mFallingExitSupportDuration, 0.25f);
        EXPECT_FLOAT_EQ(controller.mFallingExitMaximumVerticalSpeed, 90.0f);
        EXPECT_FLOAT_EQ(controller.mRecoveryDelay, 0.75f);
        EXPECT_FLOAT_EQ(controller.mBlendBackDuration, 0.3f);
        EXPECT_FLOAT_EQ(controller.mPelvisHeightFrequency, 3.5f);
        EXPECT_FLOAT_EQ(controller.mPelvisHeightDamping, 0.9f);
        EXPECT_FLOAT_EQ(controller.mCenterOfMassFrequency, 1.5f);
        EXPECT_FLOAT_EQ(controller.mCenterOfMassDamping, 0.8f);
        EXPECT_FLOAT_EQ(controller.mMaximumLocomotionSpeed, 375.0f);
        EXPECT_FLOAT_EQ(controller.mPelvisHeightSettleDuration, 0.3f);
        EXPECT_FLOAT_EQ(controller.mPelvisHeightSettleMaximumVerticalSpeed, 4.0f);
        EXPECT_FLOAT_EQ(controller.mActivationSupportGraceDuration, 0.18f);
        EXPECT_FLOAT_EQ(controller.mLocomotionSupportGraceDuration, 0.2f);
        EXPECT_FLOAT_EQ(controller.mMaximumPelvisForce, 50000.0f);
        EXPECT_EQ(controller.mLocomotionRoute, RagdollCore::LocomotionRoute::KinematicRoot);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistHorizontalScale, 0.6f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistVerticalScale, 0.8f);
        EXPECT_FLOAT_EQ(controller.mMaximumAnimationAssistForce, 42000.0f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistUprightScale, 0.7f);
        EXPECT_FLOAT_EQ(controller.mMaximumAnimationAssistTorque, 73000.0f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistPoseTrackingBlend, 0.65f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistStanceAnchorCompliance, 0.4f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistSupportLossHandoffDuration, 0.2f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistStancePositionCorrection, 0.7f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistStanceMaximumEndpointCorrection, 0.9f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistSupportMotionFrequency, 0.55f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistSupportMotionDamping, 1.1f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistSupportPositionLeash, 3.0f);
        EXPECT_EQ(controller.mAnimationAssistContactConfirmationTicks, 6u);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistLoadTransferDuration, 0.2f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistMeasuredProgressBlend, 0.75f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistFeasibilityDeadline, 0.15f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistDoubleSupportTransferDuration, 0.25f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistReentrySlipSettleDuration, 0.3f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistTouchdownCapturePositionCorrection, 0.45f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistTouchdownCaptureVelocityLookahead, 0.035f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistOutgoingHoldPositionCorrection, 0.25f);
        EXPECT_FLOAT_EQ(controller.mAnimationAssistOutgoingHoldVelocityLookahead, 0.015f);
        EXPECT_FLOAT_EQ(controller.mRecoveryUnweightFactor, 1.4f);
        EXPECT_FLOAT_EQ(controller.mPelvisUprightStiffness, 19000.0f);
        EXPECT_FLOAT_EQ(controller.mPelvisUprightDamping, 2900.0f);
        EXPECT_FLOAT_EQ(controller.mMaximumPelvisTorque, 28000.0f);
        EXPECT_FLOAT_EQ(controller.mSpineUprightStiffness, 11000.0f);
        EXPECT_FLOAT_EQ(controller.mSpineUprightDamping, 1900.0f);
        EXPECT_FLOAT_EQ(controller.mMaximumSpineTorque, 18000.0f);
        EXPECT_FLOAT_EQ(controller.mGait.mMinimumStepDuration, 0.16f);
        EXPECT_FLOAT_EQ(controller.mGait.mNominalStepDuration, 0.38f);
        EXPECT_FLOAT_EQ(controller.mGait.mMaximumStepDuration, 0.7f);
        EXPECT_FLOAT_EQ(controller.mGait.mContactConfirmationDuration, 0.04f);
        EXPECT_FLOAT_EQ(controller.mGait.mStanceConfirmationDuration, 0.12f);
        EXPECT_FLOAT_EQ(controller.mGait.mMaximumTouchdownHorizontalDistance, 5.5f);
        EXPECT_FLOAT_EQ(controller.mGait.mMaximumLandingApproachVerticalDistance, 2.75f);
        EXPECT_FLOAT_EQ(controller.mGait.mMaximumTouchdownRelativeVerticalSpeed, 4.5f);
        EXPECT_FLOAT_EQ(controller.mGait.mMaximumStepReach, 52.0f);
        EXPECT_FLOAT_EQ(controller.mGait.mNominalStepWidth, 18.0f);
        EXPECT_FLOAT_EQ(controller.mGait.mCapturePointBlend, 0.4f);
        EXPECT_FLOAT_EQ(controller.mGait.mSwingFootLift, 12.0f);
        EXPECT_FLOAT_EQ(controller.mGait.mSwingFootFrequency, 5.0f);
        EXPECT_FLOAT_EQ(controller.mGait.mSwingFootDamping, 0.85f);
        EXPECT_FLOAT_EQ(controller.mGait.mMaximumSwingFootForce, 17500.0f);
        EXPECT_FLOAT_EQ(controller.mGait.mMaximumSwingFootHorizontalForce, 9000.0f);
        EXPECT_FLOAT_EQ(controller.mGait.mPreLiftSwingUnloadFraction, 0.45f);
        EXPECT_FLOAT_EQ(controller.mGait.mSwingLegMotorTorqueScale, 3.0f);
        EXPECT_FLOAT_EQ(controller.mGait.mMaximumSwingHipTargetSpeed, 4.5f);
        EXPECT_FLOAT_EQ(controller.mGait.mSwingHipWorldStiffness, 6000.0f);
        EXPECT_FLOAT_EQ(controller.mGait.mSwingHipWorldDamping, 700.0f);
        EXPECT_FLOAT_EQ(controller.mGait.mMaximumSwingHipWorldTorque, 8000.0f);
    }

    TEST(RagdollProfileLoaderTest, rejectsUnknownConstraintType)
    {
        std::istringstream input(R"(
schema_version: 1
id: invalid_constraint
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 0.5
    shape: { type: sphere, radius: 2.0 }
  - name: torso
    bone: torso
    parent: pelvis
    mass_fraction: 0.5
    shape: { type: sphere, radius: 2.0 }
    constraint: { type: universal }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_FALSE(result.isValid());
        ASSERT_EQ(result.mIssues.size(), 1u);
        EXPECT_NE(result.mIssues.front().mMessage.find("constraint type"), std::string::npos);
    }

    TEST(RagdollProfileLoaderTest, loadsCapsuleBoneSpanFitTarget)
    {
        std::istringstream input(R"(
schema_version: 1
id: fitted_shape
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 1.0
    shape: { type: capsule, radius: 2.0, half_height: 1.0, fit_to_bone: torso }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_TRUE(result.isValid());
        ASSERT_TRUE(result.mProfile->mBodies.front().mShape.mFitToBone.has_value());
        EXPECT_EQ(*result.mProfile->mBodies.front().mShape.mFitToBone, "torso");
    }

    TEST(RagdollProfileLoaderTest, loadsImpactResponseTuning)
    {
        std::istringstream input(R"(
schema_version: 1
id: impact
impact_response:
  impulse_per_damage: 12.0
  minimum_impulse: 60.0
  maximum_impulse: 900.0
  upward_bias: 0.2
bodies:
  - name: pelvis
    bone: pelvis
    mass_fraction: 1.0
    shape: { type: sphere, radius: 2.0 }
landmarks: { pelvis: pelvis }
)");
        const auto result = RagdollProfile::load(input);
        ASSERT_TRUE(result.isValid());
        EXPECT_FLOAT_EQ(result.mProfile->mImpactResponse.mImpulsePerDamage, 12.0f);
        EXPECT_FLOAT_EQ(result.mProfile->mImpactResponse.mMinimumImpulse, 60.0f);
        EXPECT_FLOAT_EQ(result.mProfile->mImpactResponse.mMaximumImpulse, 900.0f);
        EXPECT_FLOAT_EQ(result.mProfile->mImpactResponse.mUpwardBias, 0.2f);
    }

    TEST(RagdollProfileLoaderTest, loadsAndSemanticallyValidatesBuiltinHumanoidThroughVfs)
    {
        const std::filesystem::path dataPath
            = std::filesystem::path{ OPENMW_PROJECT_SOURCE_DIR } / "files" / "data";
        VFS::Manager vfs;
        vfs.addArchive(std::make_unique<VFS::FileSystemArchive>(dataPath));
        vfs.buildIndex();

        const auto loaded = RagdollProfile::load(vfs, VFS::Path::NormalizedView("ragdoll/humanoid-v1.yaml"));
        ASSERT_TRUE(loaded.isValid());
        ASSERT_EQ(loaded.mProfile->mBodies.size(), 20u);
        EXPECT_GT(loaded.mProfile->mImpactResponse.mMaximumImpulse, 0.0f);
        EXPECT_GT(loaded.mProfile->mBodies[1].mConstraint->mMotor.mMaximumTorque, 0.0f);

        RagdollCore::SkeletonDescriptor skeleton;
        skeleton.mId = "builtin_profile_fixture";
        std::unordered_map<std::string, int> bodyIndices;
        for (std::size_t i = 0; i < loaded.mProfile->mBodies.size(); ++i)
        {
            const RagdollCore::BodyProfile& body = loaded.mProfile->mBodies[i];
            int parent = -1;
            if (body.mParentBody)
                parent = bodyIndices.at(*body.mParentBody);
            bodyIndices.emplace(body.mName, static_cast<int>(skeleton.mBones.size()));
            skeleton.mBones.push_back({ body.mBone, parent,
                { JPH::Vec3(static_cast<float>(i % 3), static_cast<float>(i + 1) * 5.0f, static_cast<float>(i % 2)),
                    JPH::Quat::sIdentity() } });
            if (body.mConstraint)
                EXPECT_TRUE(body.mConstraint->mDeriveFrameFromSkeleton);
        }

        const auto validation = RagdollCore::validateProfile(*loaded.mProfile, skeleton);
        EXPECT_TRUE(validation.isValid());
        EXPECT_TRUE(validation.mIssues.empty());
        EXPECT_TRUE(RagdollCore::buildRagdollSettings(skeleton, *loaded.mProfile, {}).isValid());
    }

    TEST(RagdollProfileLoaderTest, reportsMissingVfsProfileWithoutThrowing)
    {
        VFS::Manager vfs;
        vfs.buildIndex();
        const auto result = RagdollProfile::load(vfs, VFS::Path::NormalizedView("ragdoll/missing.yaml"));
        EXPECT_FALSE(result.isValid());
        EXPECT_FALSE(result.mIssues.empty());
    }
}
