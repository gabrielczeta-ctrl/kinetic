#include "loader.hpp"

#include <components/vfs/manager.hpp>

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string_view>
#include <utility>

namespace RagdollProfile
{
    namespace
    {
        [[noreturn]] void fail(std::string_view path, std::string_view message)
        {
            throw std::domain_error(std::string(path) + ": " + std::string(message));
        }

        YAML::Node require(const YAML::Node& node, std::string_view key, std::string_view path)
        {
            if (!node.IsMap())
                fail(path, "expected a map");
            const YAML::Node value = node[std::string(key)];
            if (!value || value.IsNull())
                fail(std::string(path) + "." + std::string(key), "required value is missing");
            return value;
        }

        template <class T>
        T scalar(const YAML::Node& node, std::string_view path)
        {
            if (!node.IsScalar())
                fail(path, "expected a scalar");
            try
            {
                return node.as<T>();
            }
            catch (const YAML::Exception& e)
            {
                fail(path, e.what());
            }
        }

        template <class T>
        T optionalScalar(const YAML::Node& node, std::string_view key, T fallback, std::string_view path)
        {
            const YAML::Node value = node[std::string(key)];
            return value ? scalar<T>(value, std::string(path) + "." + std::string(key)) : fallback;
        }

        JPH::Vec3 vector3(const YAML::Node& node, std::string_view path)
        {
            if (!node.IsSequence() || node.size() != 3)
                fail(path, "expected a three-element sequence");
            return { scalar<float>(node[0], std::string(path) + "[0]"),
                scalar<float>(node[1], std::string(path) + "[1]"),
                scalar<float>(node[2], std::string(path) + "[2]") };
        }

        JPH::Quat quaternion(const YAML::Node& node, std::string_view path)
        {
            if (!node.IsSequence() || node.size() != 4)
                fail(path, "expected a four-element [x, y, z, w] sequence");
            return { scalar<float>(node[0], std::string(path) + "[0]"),
                scalar<float>(node[1], std::string(path) + "[1]"),
                scalar<float>(node[2], std::string(path) + "[2]"),
                scalar<float>(node[3], std::string(path) + "[3]") };
        }

        RagdollCore::Transform transform(const YAML::Node& node, std::string_view path)
        {
            RagdollCore::Transform result;
            if (!node)
                return result;
            if (!node.IsMap())
                fail(path, "expected a map");
            if (const YAML::Node translation = node["translation"])
                result.mTranslation = vector3(translation, std::string(path) + ".translation");
            if (const YAML::Node rotation = node["rotation"])
                result.mRotation = quaternion(rotation, std::string(path) + ".rotation");
            return result;
        }

        RagdollCore::ShapeType shapeType(const YAML::Node& node, std::string_view path)
        {
            const std::string value = scalar<std::string>(node, path);
            if (value == "box")
                return RagdollCore::ShapeType::Box;
            if (value == "capsule")
                return RagdollCore::ShapeType::Capsule;
            if (value == "sphere")
                return RagdollCore::ShapeType::Sphere;
            if (value == "convex_hull")
                return RagdollCore::ShapeType::ConvexHull;
            fail(path, "shape type must be box, capsule, sphere, or convex_hull");
        }

        RagdollCore::ShapeProfile shape(const YAML::Node& node, std::string_view path)
        {
            RagdollCore::ShapeProfile result;
            result.mType = shapeType(require(node, "type", path), std::string(path) + ".type");
            result.mLocalTransform = transform(node["local_transform"], std::string(path) + ".local_transform");
            if (const YAML::Node value = node["fit_to_bone"])
                result.mFitToBone = scalar<std::string>(value, std::string(path) + ".fit_to_bone");
            if (const YAML::Node value = node["half_extents"])
                result.mHalfExtents = vector3(value, std::string(path) + ".half_extents");
            result.mRadius = optionalScalar<float>(node, "radius", result.mRadius, path);
            result.mHalfHeight = optionalScalar<float>(node, "half_height", result.mHalfHeight, path);
            if (result.mType == RagdollCore::ShapeType::ConvexHull)
            {
                const YAML::Node points = require(node, "points", path);
                if (!points.IsSequence() || points.size() < 4)
                    fail(std::string(path) + ".points", "expected at least four three-element points");
                result.mPoints.reserve(points.size());
                for (std::size_t i = 0; i < points.size(); ++i)
                    result.mPoints.push_back(vector3(points[i], std::string(path) + ".points[" + std::to_string(i) + "]"));
            }
            else if (node["points"])
                fail(std::string(path) + ".points", "explicit points require a convex_hull shape");
            return result;
        }

        RagdollCore::MaterialProfile material(const YAML::Node& node, std::string_view path)
        {
            RagdollCore::MaterialProfile result;
            if (!node)
                return result;
            if (!node.IsMap())
                fail(path, "expected a map");
            result.mFriction = optionalScalar<float>(node, "friction", result.mFriction, path);
            result.mRestitution = optionalScalar<float>(node, "restitution", result.mRestitution, path);
            result.mLinearDamping = optionalScalar<float>(node, "linear_damping", result.mLinearDamping, path);
            result.mAngularDamping = optionalScalar<float>(node, "angular_damping", result.mAngularDamping, path);
            return result;
        }

        RagdollCore::ConstraintType constraintType(const YAML::Node& node, std::string_view path)
        {
            const std::string value = scalar<std::string>(node, path);
            if (value == "swing_twist")
                return RagdollCore::ConstraintType::SwingTwist;
            if (value == "hinge")
                return RagdollCore::ConstraintType::Hinge;
            fail(path, "constraint type must be swing_twist or hinge");
        }

        RagdollCore::MotorProfile motor(const YAML::Node& node, std::string_view path)
        {
            RagdollCore::MotorProfile result;
            if (!node)
                return result;
            if (!node.IsMap())
                fail(path, "expected a map");
            result.mFrequency = optionalScalar<float>(node, "frequency", result.mFrequency, path);
            result.mDamping = optionalScalar<float>(node, "damping", result.mDamping, path);
            result.mMaximumTorque
                = optionalScalar<float>(node, "maximum_torque", result.mMaximumTorque, path);
            return result;
        }

        RagdollCore::ConstraintProfile constraint(const YAML::Node& node, std::string_view path)
        {
            if (!node.IsMap())
                fail(path, "expected a map");
            RagdollCore::ConstraintProfile result;
            if (const YAML::Node value = node["type"])
                result.mType = constraintType(value, std::string(path) + ".type");
            result.mDeriveFrameFromSkeleton
                = optionalScalar<bool>(node, "derive_frame", result.mDeriveFrameFromSkeleton, path);
            if (const YAML::Node value = node["frame_axis_bone"])
                result.mFrameAxisBone = scalar<std::string>(value, std::string(path) + ".frame_axis_bone");
            if (const YAML::Node value = node["parent_anchor"])
                result.mParentAnchor = vector3(value, std::string(path) + ".parent_anchor");
            if (const YAML::Node value = node["child_anchor"])
                result.mChildAnchor = vector3(value, std::string(path) + ".child_anchor");
            if (const YAML::Node value = node["parent_twist_axis"])
                result.mParentTwistAxis = vector3(value, std::string(path) + ".parent_twist_axis");
            if (const YAML::Node value = node["parent_plane_axis"])
                result.mParentPlaneAxis = vector3(value, std::string(path) + ".parent_plane_axis");
            if (const YAML::Node value = node["child_twist_axis"])
                result.mChildTwistAxis = vector3(value, std::string(path) + ".child_twist_axis");
            if (const YAML::Node value = node["child_plane_axis"])
                result.mChildPlaneAxis = vector3(value, std::string(path) + ".child_plane_axis");
            result.mNormalHalfConeAngle
                = optionalScalar<float>(node, "normal_half_cone_angle", result.mNormalHalfConeAngle, path);
            result.mPlaneHalfConeAngle
                = optionalScalar<float>(node, "plane_half_cone_angle", result.mPlaneHalfConeAngle, path);
            result.mTwistMinAngle = optionalScalar<float>(node, "twist_min_angle", result.mTwistMinAngle, path);
            result.mTwistMaxAngle = optionalScalar<float>(node, "twist_max_angle", result.mTwistMaxAngle, path);
            result.mHingeMinAngle = optionalScalar<float>(node, "limit_min_angle", result.mHingeMinAngle, path);
            result.mHingeMaxAngle = optionalScalar<float>(node, "limit_max_angle", result.mHingeMaxAngle, path);
            result.mMaxFrictionTorque
                = optionalScalar<float>(node, "max_friction_torque", result.mMaxFrictionTorque, path);
            result.mMotor = motor(node["motor"], std::string(path) + ".motor");
            return result;
        }

        RagdollCore::ImpactResponseProfile impactResponse(const YAML::Node& node, std::string_view path)
        {
            RagdollCore::ImpactResponseProfile result;
            if (!node)
                return result;
            if (!node.IsMap())
                fail(path, "expected a map");
            result.mImpulsePerDamage
                = optionalScalar<float>(node, "impulse_per_damage", result.mImpulsePerDamage, path);
            result.mMinimumImpulse = optionalScalar<float>(node, "minimum_impulse", result.mMinimumImpulse, path);
            result.mMaximumImpulse = optionalScalar<float>(node, "maximum_impulse", result.mMaximumImpulse, path);
            result.mUpwardBias = optionalScalar<float>(node, "upward_bias", result.mUpwardBias, path);
            return result;
        }

        RagdollCore::MotionTransferProfile motionTransfer(const YAML::Node& node, std::string_view path)
        {
            RagdollCore::MotionTransferProfile result;
            if (!node)
                return result;
            if (!node.IsMap())
                fail(path, "expected a map");
            result.mMaximumSampleDuration = optionalScalar<float>(
                node, "maximum_sample_duration", result.mMaximumSampleDuration, path);
            result.mMaximumLinearSpeed
                = optionalScalar<float>(node, "maximum_linear_speed", result.mMaximumLinearSpeed, path);
            result.mMaximumAngularSpeed
                = optionalScalar<float>(node, "maximum_angular_speed", result.mMaximumAngularSpeed, path);
            return result;
        }

        RagdollCore::PoseTransitionProfile poseTransition(const YAML::Node& node, std::string_view path)
        {
            RagdollCore::PoseTransitionProfile result;
            if (!node)
                return result;
            if (!node.IsMap())
                fail(path, "expected a map");
            result.mDeathBlendDuration
                = optionalScalar<float>(node, "death_blend_duration", result.mDeathBlendDuration, path);
            return result;
        }

        RagdollCore::GaitControllerProfile gaitController(const YAML::Node& node, std::string_view path)
        {
            RagdollCore::GaitControllerProfile result;
            if (!node)
                return result;
            if (!node.IsMap())
                fail(path, "expected a map");
            result.mMinimumStepDuration
                = optionalScalar<float>(node, "minimum_step_duration", result.mMinimumStepDuration, path);
            result.mNominalStepDuration
                = optionalScalar<float>(node, "nominal_step_duration", result.mNominalStepDuration, path);
            result.mMaximumStepDuration
                = optionalScalar<float>(node, "maximum_step_duration", result.mMaximumStepDuration, path);
            result.mContactConfirmationDuration = optionalScalar<float>(
                node, "contact_confirmation_duration", result.mContactConfirmationDuration, path);
            result.mStanceConfirmationDuration = optionalScalar<float>(
                node, "stance_confirmation_duration", result.mStanceConfirmationDuration, path);
            result.mMaximumTouchdownHorizontalDistance = optionalScalar<float>(node,
                "maximum_touchdown_horizontal_distance", result.mMaximumTouchdownHorizontalDistance, path);
            result.mMaximumLandingApproachVerticalDistance = optionalScalar<float>(node,
                "maximum_landing_approach_vertical_distance", result.mMaximumLandingApproachVerticalDistance, path);
            result.mMaximumTouchdownRelativeVerticalSpeed = optionalScalar<float>(node,
                "maximum_touchdown_relative_vertical_speed", result.mMaximumTouchdownRelativeVerticalSpeed, path);
            result.mMaximumStepReach
                = optionalScalar<float>(node, "maximum_step_reach", result.mMaximumStepReach, path);
            result.mNominalStepWidth
                = optionalScalar<float>(node, "nominal_step_width", result.mNominalStepWidth, path);
            result.mCapturePointBlend
                = optionalScalar<float>(node, "capture_point_blend", result.mCapturePointBlend, path);
            result.mSwingFootLift
                = optionalScalar<float>(node, "swing_foot_lift", result.mSwingFootLift, path);
            result.mSwingFootFrequency
                = optionalScalar<float>(node, "swing_foot_frequency", result.mSwingFootFrequency, path);
            result.mSwingFootDamping
                = optionalScalar<float>(node, "swing_foot_damping", result.mSwingFootDamping, path);
            result.mMaximumSwingFootForce = optionalScalar<float>(
                node, "maximum_swing_foot_force", result.mMaximumSwingFootForce, path);
            result.mMaximumSwingFootHorizontalForce = optionalScalar<float>(node,
                "maximum_swing_foot_horizontal_force", result.mMaximumSwingFootHorizontalForce, path);
            result.mPreLiftSwingUnloadFraction = optionalScalar<float>(node,
                "pre_lift_swing_unload_fraction", result.mPreLiftSwingUnloadFraction, path);
            result.mSwingLegMotorTorqueScale = optionalScalar<float>(
                node, "swing_leg_motor_torque_scale", result.mSwingLegMotorTorqueScale, path);
            result.mMaximumSwingHipTargetSpeed = optionalScalar<float>(
                node, "maximum_swing_hip_target_speed", result.mMaximumSwingHipTargetSpeed, path);
            result.mSwingHipWorldStiffness = optionalScalar<float>(
                node, "swing_hip_world_stiffness", result.mSwingHipWorldStiffness, path);
            result.mSwingHipWorldDamping = optionalScalar<float>(
                node, "swing_hip_world_damping", result.mSwingHipWorldDamping, path);
            result.mMaximumSwingHipWorldTorque = optionalScalar<float>(
                node, "maximum_swing_hip_world_torque", result.mMaximumSwingHipWorldTorque, path);
            return result;
        }

        RagdollCore::ActiveControllerProfile activeController(const YAML::Node& node, std::string_view path)
        {
            RagdollCore::ActiveControllerProfile result;
            if (!node)
                return result;
            if (!node.IsMap())
                fail(path, "expected a map");
            result.mSupportNormalMinimumDot = optionalScalar<float>(
                node, "support_normal_minimum_dot", result.mSupportNormalMinimumDot, path);
            result.mSupportMargin = optionalScalar<float>(node, "support_margin", result.mSupportMargin, path);
            result.mStandingMinimumUpDot
                = optionalScalar<float>(node, "standing_minimum_up_dot", result.mStandingMinimumUpDot, path);
            result.mBracingMinimumUpDot
                = optionalScalar<float>(node, "bracing_minimum_up_dot", result.mBracingMinimumUpDot, path);
            result.mStandingMaximumSpeed
                = optionalScalar<float>(node, "standing_maximum_speed", result.mStandingMaximumSpeed, path);
            result.mSettledMaximumSpeed
                = optionalScalar<float>(node, "settled_maximum_speed", result.mSettledMaximumSpeed, path);
            result.mFallenMaximumPelvisHeight = optionalScalar<float>(
                node, "fallen_maximum_pelvis_height", result.mFallenMaximumPelvisHeight, path);
            result.mMinimumStateDuration
                = optionalScalar<float>(node, "minimum_state_duration", result.mMinimumStateDuration, path);
            result.mFallingExitSupportDuration = optionalScalar<float>(
                node, "falling_exit_support_duration", result.mFallingExitSupportDuration, path);
            result.mFallingExitMaximumVerticalSpeed = optionalScalar<float>(
                node, "falling_exit_maximum_vertical_speed", result.mFallingExitMaximumVerticalSpeed, path);
            result.mRecoveryDelay = optionalScalar<float>(node, "recovery_delay", result.mRecoveryDelay, path);
            result.mBlendBackDuration
                = optionalScalar<float>(node, "blend_back_duration", result.mBlendBackDuration, path);
            result.mPelvisHeightFrequency
                = optionalScalar<float>(node, "pelvis_height_frequency", result.mPelvisHeightFrequency, path);
            result.mPelvisHeightDamping
                = optionalScalar<float>(node, "pelvis_height_damping", result.mPelvisHeightDamping, path);
            result.mCenterOfMassFrequency = optionalScalar<float>(
                node, "center_of_mass_frequency", result.mCenterOfMassFrequency, path);
            result.mCenterOfMassDamping
                = optionalScalar<float>(node, "center_of_mass_damping", result.mCenterOfMassDamping, path);
            result.mMaximumLocomotionSpeed = optionalScalar<float>(
                node, "maximum_locomotion_speed", result.mMaximumLocomotionSpeed, path);
            result.mPelvisHeightSettleDuration = optionalScalar<float>(
                node, "pelvis_height_settle_duration", result.mPelvisHeightSettleDuration, path);
            result.mPelvisHeightSettleMaximumVerticalSpeed = optionalScalar<float>(node,
                "pelvis_height_settle_maximum_vertical_speed", result.mPelvisHeightSettleMaximumVerticalSpeed, path);
            result.mActivationSupportGraceDuration = optionalScalar<float>(
                node, "activation_support_grace_duration", result.mActivationSupportGraceDuration, path);
            result.mLocomotionSupportGraceDuration = optionalScalar<float>(
                node, "locomotion_support_grace_duration", result.mLocomotionSupportGraceDuration, path);
            result.mMaximumPelvisForce
                = optionalScalar<float>(node, "maximum_pelvis_force", result.mMaximumPelvisForce, path);
            result.mAnimationAssistHorizontalScale = optionalScalar<float>(node,
                "animation_assist_horizontal_scale", result.mAnimationAssistHorizontalScale, path);
            result.mAnimationAssistVerticalScale = optionalScalar<float>(node,
                "animation_assist_vertical_scale", result.mAnimationAssistVerticalScale, path);
            result.mMaximumAnimationAssistForce = optionalScalar<float>(node,
                "maximum_animation_assist_force", result.mMaximumAnimationAssistForce, path);
            result.mAnimationAssistUprightScale = optionalScalar<float>(node,
                "animation_assist_upright_scale", result.mAnimationAssistUprightScale, path);
            result.mMaximumAnimationAssistTorque = optionalScalar<float>(node,
                "maximum_animation_assist_torque", result.mMaximumAnimationAssistTorque, path);
            result.mAnimationAssistYawReferenceMaximumRate = optionalScalar<float>(node,
                "animation_assist_yaw_reference_maximum_rate", result.mAnimationAssistYawReferenceMaximumRate, path);
            result.mAnimationAssistYawReferenceMaximumAcceleration = optionalScalar<float>(node,
                "animation_assist_yaw_reference_maximum_acceleration",
                result.mAnimationAssistYawReferenceMaximumAcceleration, path);
            result.mAnimationAssistYawStiffness = optionalScalar<float>(node,
                "animation_assist_yaw_stiffness", result.mAnimationAssistYawStiffness, path);
            result.mAnimationAssistYawDamping = optionalScalar<float>(node,
                "animation_assist_yaw_damping", result.mAnimationAssistYawDamping, path);
            result.mMaximumAnimationAssistYawTorque = optionalScalar<float>(node,
                "maximum_animation_assist_yaw_torque", result.mMaximumAnimationAssistYawTorque, path);
            result.mAnimationAssistPoseTrackingBlend = optionalScalar<float>(node,
                "animation_assist_pose_tracking_blend", result.mAnimationAssistPoseTrackingBlend, path);
            result.mAnimationAssistStanceAnchorCompliance = optionalScalar<float>(node,
                "animation_assist_stance_anchor_compliance", result.mAnimationAssistStanceAnchorCompliance, path);
            result.mAnimationAssistSupportLossHandoffDuration = optionalScalar<float>(node,
                "animation_assist_support_loss_handoff_duration",
                result.mAnimationAssistSupportLossHandoffDuration, path);
            result.mAnimationAssistStancePositionCorrection = optionalScalar<float>(node,
                "animation_assist_stance_position_correction",
                result.mAnimationAssistStancePositionCorrection, path);
            result.mAnimationAssistStanceMaximumEndpointCorrection = optionalScalar<float>(node,
                "animation_assist_stance_maximum_endpoint_correction",
                result.mAnimationAssistStanceMaximumEndpointCorrection, path);
            result.mAnimationAssistSupportMotionFrequency = optionalScalar<float>(node,
                "animation_assist_support_motion_frequency",
                result.mAnimationAssistSupportMotionFrequency, path);
            result.mAnimationAssistSupportMotionDamping = optionalScalar<float>(node,
                "animation_assist_support_motion_damping",
                result.mAnimationAssistSupportMotionDamping, path);
            result.mAnimationAssistSupportPositionLeash = optionalScalar<float>(node,
                "animation_assist_support_position_leash",
                result.mAnimationAssistSupportPositionLeash, path);
            result.mAnimationAssistContactConfirmationTicks = optionalScalar<unsigned>(node,
                "animation_assist_contact_confirmation_ticks",
                result.mAnimationAssistContactConfirmationTicks, path);
            result.mAnimationAssistLoadTransferDuration = optionalScalar<float>(node,
                "animation_assist_load_transfer_duration",
                result.mAnimationAssistLoadTransferDuration, path);
            result.mAnimationAssistMeasuredProgressBlend = optionalScalar<float>(node,
                "animation_assist_measured_progress_blend",
                result.mAnimationAssistMeasuredProgressBlend, path);
            result.mAnimationAssistFeasibilityDeadline = optionalScalar<float>(node,
                "animation_assist_feasibility_deadline",
                result.mAnimationAssistFeasibilityDeadline, path);
            result.mAnimationAssistDoubleSupportTransferDuration = optionalScalar<float>(node,
                "animation_assist_double_support_transfer_duration",
                result.mAnimationAssistDoubleSupportTransferDuration, path);
            result.mAnimationAssistReentrySlipSettleDuration = optionalScalar<float>(node,
                "animation_assist_reentry_slip_settle_duration",
                result.mAnimationAssistReentrySlipSettleDuration, path);
            result.mAnimationAssistTouchdownCapturePositionCorrection = optionalScalar<float>(node,
                "animation_assist_touchdown_capture_position_correction",
                result.mAnimationAssistTouchdownCapturePositionCorrection, path);
            result.mAnimationAssistTouchdownCaptureVelocityLookahead = optionalScalar<float>(node,
                "animation_assist_touchdown_capture_velocity_lookahead",
                result.mAnimationAssistTouchdownCaptureVelocityLookahead, path);
            result.mAnimationAssistOutgoingHoldPositionCorrection = optionalScalar<float>(node,
                "animation_assist_outgoing_hold_position_correction",
                result.mAnimationAssistOutgoingHoldPositionCorrection, path);
            result.mAnimationAssistOutgoingHoldVelocityLookahead = optionalScalar<float>(node,
                "animation_assist_outgoing_hold_velocity_lookahead",
                result.mAnimationAssistOutgoingHoldVelocityLookahead, path);
            result.mKinematicRootHardLandingSpeed = optionalScalar<float>(node,
                "kinematic_root_hard_landing_speed", result.mKinematicRootHardLandingSpeed, path);
            result.mKinematicRootMinimumRate = optionalScalar<float>(node,
                "kinematic_root_minimum_rate", result.mKinematicRootMinimumRate, path);
            result.mKinematicRootJumpAnticipation = optionalScalar<float>(node,
                "kinematic_root_jump_anticipation", result.mKinematicRootJumpAnticipation, path);
            result.mKinematicRootPoseTrackingBlend = optionalScalar<float>(node,
                "kinematic_root_pose_tracking_blend", result.mKinematicRootPoseTrackingBlend, path);
            result.mKinematicRootMaximumYawRate = optionalScalar<float>(node,
                "kinematic_root_maximum_yaw_rate", result.mKinematicRootMaximumYawRate, path);
            result.mRecoveryUnweightFactor = optionalScalar<float>(
                node, "recovery_unweight_factor", result.mRecoveryUnweightFactor, path);
            result.mPelvisUprightStiffness = optionalScalar<float>(
                node, "pelvis_upright_stiffness", result.mPelvisUprightStiffness, path);
            result.mPelvisUprightDamping = optionalScalar<float>(
                node, "pelvis_upright_damping", result.mPelvisUprightDamping, path);
            result.mMaximumPelvisTorque
                = optionalScalar<float>(node, "maximum_pelvis_torque", result.mMaximumPelvisTorque, path);
            result.mSpineUprightStiffness = optionalScalar<float>(
                node, "spine_upright_stiffness", result.mSpineUprightStiffness, path);
            result.mSpineUprightDamping = optionalScalar<float>(
                node, "spine_upright_damping", result.mSpineUprightDamping, path);
            result.mMaximumSpineTorque
                = optionalScalar<float>(node, "maximum_spine_torque", result.mMaximumSpineTorque, path);
            const std::string route
                = optionalScalar<std::string>(node, "locomotion_route", std::string("reference"), path);
            if (route == "reference")
                result.mLocomotionRoute = RagdollCore::LocomotionRoute::Reference;
            else if (route == "legacy")
                result.mLocomotionRoute = RagdollCore::LocomotionRoute::Legacy;
            else if (route == "kinematic_root")
                result.mLocomotionRoute = RagdollCore::LocomotionRoute::KinematicRoot;
            else
                fail(std::string(path) + ".locomotion_route",
                    "expected \"reference\", \"legacy\" or \"kinematic_root\"");
            result.mGait = gaitController(node["gait"], std::string(path) + ".gait");
            return result;
        }

        RagdollCore::BodyProfile body(const YAML::Node& node, std::size_t index)
        {
            const std::string path = "profile.bodies[" + std::to_string(index) + "]";
            RagdollCore::BodyProfile result;
            result.mName = scalar<std::string>(require(node, "name", path), path + ".name");
            result.mBone = scalar<std::string>(require(node, "bone", path), path + ".bone");
            if (const YAML::Node parent = node["parent"])
                result.mParentBody = scalar<std::string>(parent, path + ".parent");
            result.mShape = shape(require(node, "shape", path), path + ".shape");
            result.mMassFraction = scalar<float>(require(node, "mass_fraction", path), path + ".mass_fraction");
            if (const YAML::Node value = node["center_of_mass"])
                result.mCenterOfMass = vector3(value, path + ".center_of_mass");
            result.mMaterial = material(node["material"], path + ".material");
            result.mSelfCollisionGroup
                = optionalScalar<std::uint32_t>(node, "self_collision_group", result.mSelfCollisionGroup, path);
            if (const YAML::Node value = node["constraint"])
                result.mConstraint = constraint(value, path + ".constraint");
            return result;
        }

        std::optional<std::string> optionalString(const YAML::Node& node, std::string_view key, std::string_view path)
        {
            if (const YAML::Node value = node[std::string(key)])
                return scalar<std::string>(value, std::string(path) + "." + std::string(key));
            return std::nullopt;
        }

        RagdollCore::RagdollProfile parse(const YAML::Node& root)
        {
            if (!root.IsMap())
                fail("profile", "document root must be a map");
            RagdollCore::RagdollProfile result;
            result.mSchemaVersion = scalar<std::uint32_t>(require(root, "schema_version", "profile"), "profile.schema_version");
            result.mId = scalar<std::string>(require(root, "id", "profile"), "profile.id");
            result.mImpactResponse = impactResponse(root["impact_response"], "profile.impact_response");
            result.mMotionTransfer = motionTransfer(root["motion_transfer"], "profile.motion_transfer");
            result.mPoseTransition = poseTransition(root["pose_transition"], "profile.pose_transition");
            result.mActiveController = activeController(root["active_controller"], "profile.active_controller");

            if (const YAML::Node aliases = root["bone_aliases"])
            {
                if (!aliases.IsSequence())
                    fail("profile.bone_aliases", "expected a sequence");
                for (std::size_t i = 0; i < aliases.size(); ++i)
                {
                    const std::string path = "profile.bone_aliases[" + std::to_string(i) + "]";
                    RagdollCore::BoneAliases entry;
                    entry.mCanonical
                        = scalar<std::string>(require(aliases[i], "canonical", path), path + ".canonical");
                    const YAML::Node values = require(aliases[i], "aliases", path);
                    if (!values.IsSequence())
                        fail(path + ".aliases", "expected a sequence");
                    for (std::size_t j = 0; j < values.size(); ++j)
                        entry.mAliases.push_back(scalar<std::string>(values[j], path + ".aliases[" + std::to_string(j) + "]"));
                    result.mBoneAliases.push_back(std::move(entry));
                }
            }

            const YAML::Node bodies = require(root, "bodies", "profile");
            if (!bodies.IsSequence())
                fail("profile.bodies", "expected a sequence");
            for (std::size_t i = 0; i < bodies.size(); ++i)
                result.mBodies.push_back(body(bodies[i], i));

            const YAML::Node landmarks = require(root, "landmarks", "profile");
            result.mLandmarks.mPelvis
                = scalar<std::string>(require(landmarks, "pelvis", "profile.landmarks"), "profile.landmarks.pelvis");
            if (const YAML::Node spine = landmarks["spine"])
            {
                if (!spine.IsSequence())
                    fail("profile.landmarks.spine", "expected a sequence");
                for (std::size_t i = 0; i < spine.size(); ++i)
                    result.mLandmarks.mSpine.push_back(
                        scalar<std::string>(spine[i], "profile.landmarks.spine[" + std::to_string(i) + "]"));
            }
            result.mLandmarks.mHead = optionalString(landmarks, "head", "profile.landmarks");
            result.mLandmarks.mLeftHand = optionalString(landmarks, "left_hand", "profile.landmarks");
            result.mLandmarks.mRightHand = optionalString(landmarks, "right_hand", "profile.landmarks");
            result.mLandmarks.mLeftFoot = optionalString(landmarks, "left_foot", "profile.landmarks");
            result.mLandmarks.mRightFoot = optionalString(landmarks, "right_foot", "profile.landmarks");
            return result;
        }
    }

    LoadResult load(std::istream& stream)
    {
        LoadResult result;
        try
        {
            result.mProfile = parse(YAML::Load(stream));
        }
        catch (const std::exception& e)
        {
            result.mIssues.push_back({ "profile", e.what() });
        }
        return result;
    }

    LoadResult load(const VFS::Manager& vfs, VFS::Path::NormalizedView path)
    {
        LoadResult result;
        try
        {
            auto stream = vfs.get(path);
            return load(*stream);
        }
        catch (const std::exception& e)
        {
            result.mIssues.push_back({ std::string(path.value()), e.what() });
        }
        return result;
    }
}
