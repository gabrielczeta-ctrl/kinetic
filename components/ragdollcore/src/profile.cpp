#include <ragdollcore/profile.hpp>

#include <Jolt/Math/DVec3.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace RagdollCore
{
    namespace
    {
        bool isFinite(float value)
        {
            return std::isfinite(value);
        }

        bool isFinite(JPH::Vec3Arg value)
        {
            return isFinite(value.GetX()) && isFinite(value.GetY()) && isFinite(value.GetZ());
        }

        bool isFinite(JPH::QuatArg value)
        {
            return isFinite(value.GetX()) && isFinite(value.GetY()) && isFinite(value.GetZ()) && isFinite(value.GetW());
        }

        bool isNormalized(JPH::Vec3Arg value)
        {
            return isFinite(value) && std::abs(value.LengthSq() - 1.0f) <= 1e-3f;
        }

        bool isNormalized(JPH::QuatArg value)
        {
            if (!isFinite(value))
                return false;
            const float lengthSquared = value.GetX() * value.GetX() + value.GetY() * value.GetY()
                + value.GetZ() * value.GetZ() + value.GetW() * value.GetW();
            return std::abs(lengthSquared - 1.0f) <= 1e-3f;
        }

        void addIssue(ValidationResult& result, ValidationCode code, std::string path, std::string message)
        {
            result.mIssues.push_back({ code, std::move(path), std::move(message) });
        }

        bool spansVolume(const std::vector<JPH::Vec3>& points)
        {
            if (points.size() < 4)
                return false;
            for (const JPH::Vec3& point : points)
                if (!isFinite(point))
                    return false;

            // Work in doubles so finite float coordinates cannot overflow the
            // rank test. The relative float precision guard rejects degenerate
            // point/line/plane sets without introducing a world-unit size gate.
            const JPH::DVec3 origin(points.front());
            JPH::DVec3 longest = JPH::DVec3::sZero();
            for (const JPH::Vec3& point : points)
            {
                const JPH::DVec3 offset = JPH::DVec3(point) - origin;
                if (offset.LengthSq() > longest.LengthSq())
                    longest = offset;
            }
            if (longest.LengthSq() == 0)
                return false;
            const double precision = std::numeric_limits<float>::epsilon() * longest.Length();
            const JPH::DVec3 axis = longest.Normalized();
            JPH::DVec3 normal = JPH::DVec3::sZero();
            for (const JPH::Vec3& point : points)
            {
                const JPH::DVec3 candidate = axis.Cross(JPH::DVec3(point) - origin);
                if (candidate.LengthSq() > normal.LengthSq())
                    normal = candidate;
            }
            if (normal.Length() <= precision)
                return false;
            normal = normal.Normalized();
            for (const JPH::Vec3& point : points)
                if (std::abs(normal.Dot(JPH::DVec3(point) - origin)) > precision)
                    return true;
            return false;
        }

        void validateTransform(const Transform& transform, std::string_view path, ValidationResult& result)
        {
            if (!isFinite(transform.mTranslation))
                addIssue(result, ValidationCode::InvalidNumber, std::string(path) + ".translation",
                    "Translation must contain finite values.");
            if (!isNormalized(transform.mRotation))
                addIssue(result, ValidationCode::InvalidNumber, std::string(path) + ".rotation",
                    "Rotation must be finite and normalized.");
        }

        void validateAxes(const ConstraintProfile& constraint, std::string_view path, ValidationResult& result)
        {
            const auto validateFrame = [&](JPH::Vec3Arg twist, JPH::Vec3Arg plane, std::string_view suffix) {
                if (!isNormalized(twist) || !isNormalized(plane) || std::abs(twist.Dot(plane)) > 1e-3f)
                    addIssue(result, ValidationCode::InvalidConstraint, std::string(path) + std::string(suffix),
                        "Twist and plane axes must be finite, normalized, and perpendicular.");
            };
            validateFrame(constraint.mParentTwistAxis, constraint.mParentPlaneAxis, ".parentFrame");
            validateFrame(constraint.mChildTwistAxis, constraint.mChildPlaneAxis, ".childFrame");
        }

        bool containsBody(const std::unordered_map<std::string, std::size_t>& bodies, const std::string& name)
        {
            return bodies.contains(name);
        }
    }

    ValidationResult validateSkeleton(const SkeletonDescriptor& skeleton)
    {
        ValidationResult result;
        if (skeleton.mId.empty())
            addIssue(result, ValidationCode::EmptyIdentifier, "skeleton.id", "Skeleton id must not be empty.");
        if (skeleton.mBones.empty())
            addIssue(result, ValidationCode::InvalidHierarchy, "skeleton.bones", "Skeleton must contain at least one bone.");

        std::unordered_set<std::string> names;
        std::size_t rootCount = 0;
        for (std::size_t i = 0; i < skeleton.mBones.size(); ++i)
        {
            const SkeletonBone& bone = skeleton.mBones[i];
            const std::string path = "skeleton.bones[" + std::to_string(i) + "]";
            if (bone.mName.empty())
                addIssue(result, ValidationCode::EmptyIdentifier, path + ".name", "Bone name must not be empty.");
            else if (!names.insert(bone.mName).second)
                addIssue(result, ValidationCode::DuplicateIdentifier, path + ".name", "Bone name must be unique.");

            if (bone.mParent == -1)
                ++rootCount;
            else if (bone.mParent < 0 || static_cast<std::size_t>(bone.mParent) >= i)
                addIssue(result, ValidationCode::InvalidHierarchy, path + ".parent",
                    "Parent must precede the child in topological order.");
            validateTransform(bone.mModelTransform, path + ".modelTransform", result);
        }
        if (!skeleton.mBones.empty() && rootCount != 1)
            addIssue(result, ValidationCode::InvalidHierarchy, "skeleton.bones", "Skeleton must contain exactly one root.");
        return result;
    }

    ValidationResult validateProfile(const RagdollProfile& profile, const SkeletonDescriptor& skeleton)
    {
        ValidationResult result = validateSkeleton(skeleton);
        if (profile.mSchemaVersion != CurrentProfileSchemaVersion)
            addIssue(result, ValidationCode::UnsupportedSchema, "profile.schemaVersion",
                "Profile schema version is not supported by this build.");
        if (profile.mId.empty())
            addIssue(result, ValidationCode::EmptyIdentifier, "profile.id", "Profile id must not be empty.");
        if (profile.mBodies.empty())
            addIssue(result, ValidationCode::MissingBody, "profile.bodies", "Profile must contain at least one body.");

        const ImpactResponseProfile& impact = profile.mImpactResponse;
        if (!isFinite(impact.mImpulsePerDamage) || impact.mImpulsePerDamage < 0.0f || !isFinite(impact.mMinimumImpulse)
            || impact.mMinimumImpulse < 0.0f || !isFinite(impact.mMaximumImpulse)
            || impact.mMaximumImpulse < impact.mMinimumImpulse || !isFinite(impact.mUpwardBias)
            || impact.mUpwardBias < 0.0f || impact.mUpwardBias > 1.0f)
            addIssue(result, ValidationCode::InvalidNumber, "profile.impactResponse",
                "Impact response values must be finite and non-negative; maximum must be at least minimum and upward "
                "bias must be in [0, 1].");

        const MotionTransferProfile& motion = profile.mMotionTransfer;
        if (!isFinite(motion.mMaximumSampleDuration) || motion.mMaximumSampleDuration <= 0.0f
            || !isFinite(motion.mMaximumLinearSpeed) || motion.mMaximumLinearSpeed <= 0.0f
            || !isFinite(motion.mMaximumAngularSpeed) || motion.mMaximumAngularSpeed <= 0.0f)
            addIssue(result, ValidationCode::InvalidNumber, "profile.motionTransfer",
                "Motion-transfer duration and speed limits must be finite and positive.");

        if (!isFinite(profile.mPoseTransition.mDeathBlendDuration)
            || profile.mPoseTransition.mDeathBlendDuration < 0.0f)
            addIssue(result, ValidationCode::InvalidNumber, "profile.poseTransition.deathBlendDuration",
                "Death blend duration must be finite and non-negative.");

        const ActiveControllerProfile& controller = profile.mActiveController;
        const GaitControllerProfile& gait = controller.mGait;
        if (!isFinite(controller.mSupportNormalMinimumDot) || controller.mSupportNormalMinimumDot < 0.0f
            || controller.mSupportNormalMinimumDot > 1.0f || !isFinite(controller.mSupportMargin)
            || controller.mSupportMargin < 0.0f || !isFinite(controller.mStandingMinimumUpDot)
            || controller.mStandingMinimumUpDot < -1.0f || controller.mStandingMinimumUpDot > 1.0f
            || !isFinite(controller.mBracingMinimumUpDot) || controller.mBracingMinimumUpDot < -1.0f
            || controller.mBracingMinimumUpDot > controller.mStandingMinimumUpDot
            || !isFinite(controller.mStandingMaximumSpeed) || controller.mStandingMaximumSpeed < 0.0f
            || !isFinite(controller.mSettledMaximumSpeed) || controller.mSettledMaximumSpeed < 0.0f
            || controller.mSettledMaximumSpeed > controller.mStandingMaximumSpeed
            || !isFinite(controller.mFallenMaximumPelvisHeight)
            || controller.mFallenMaximumPelvisHeight < 0.0f
            || !isFinite(controller.mMinimumStateDuration) || controller.mMinimumStateDuration < 0.0f
            || !isFinite(controller.mFallingExitSupportDuration)
            || controller.mFallingExitSupportDuration < 0.0f
            || !isFinite(controller.mFallingExitMaximumVerticalSpeed)
            || controller.mFallingExitMaximumVerticalSpeed < 0.0f
            || !isFinite(controller.mRecoveryDelay) || controller.mRecoveryDelay < 0.0f
            || !isFinite(controller.mBlendBackDuration) || controller.mBlendBackDuration < 0.0f
            || !isFinite(controller.mPelvisHeightFrequency) || controller.mPelvisHeightFrequency < 0.0f
            || !isFinite(controller.mPelvisHeightDamping) || controller.mPelvisHeightDamping < 0.0f
            || !isFinite(controller.mCenterOfMassFrequency) || controller.mCenterOfMassFrequency < 0.0f
            || !isFinite(controller.mCenterOfMassDamping) || controller.mCenterOfMassDamping < 0.0f
            || !isFinite(controller.mMaximumLocomotionSpeed) || controller.mMaximumLocomotionSpeed < 0.0f
            || !isFinite(controller.mPelvisHeightSettleDuration)
            || controller.mPelvisHeightSettleDuration < 0.0f
            || !isFinite(controller.mPelvisHeightSettleMaximumVerticalSpeed)
            || controller.mPelvisHeightSettleMaximumVerticalSpeed < 0.0f
            || !isFinite(controller.mActivationSupportGraceDuration)
            || controller.mActivationSupportGraceDuration < 0.0f
            || !isFinite(controller.mLocomotionSupportGraceDuration)
            || controller.mLocomotionSupportGraceDuration < 0.0f
            || !isFinite(controller.mMaximumPelvisForce) || controller.mMaximumPelvisForce < 0.0f
            || !isFinite(controller.mRecoveryUnweightFactor) || controller.mRecoveryUnweightFactor < 0.0f
            || controller.mRecoveryUnweightFactor > 3.0f
            || !isFinite(controller.mPelvisUprightStiffness) || controller.mPelvisUprightStiffness < 0.0f
            || !isFinite(controller.mPelvisUprightDamping) || controller.mPelvisUprightDamping < 0.0f
            || !isFinite(controller.mMaximumPelvisTorque) || controller.mMaximumPelvisTorque < 0.0f
            || !isFinite(controller.mSpineUprightStiffness) || controller.mSpineUprightStiffness < 0.0f
            || !isFinite(controller.mSpineUprightDamping) || controller.mSpineUprightDamping < 0.0f
            || !isFinite(controller.mMaximumSpineTorque) || controller.mMaximumSpineTorque < 0.0f
            || !isFinite(controller.mAnimationAssistHorizontalScale)
            || controller.mAnimationAssistHorizontalScale < 0.0f
            || controller.mAnimationAssistHorizontalScale > 1.0f
            || !isFinite(controller.mAnimationAssistVerticalScale)
            || controller.mAnimationAssistVerticalScale < 0.0f
            || controller.mAnimationAssistVerticalScale > 1.0f
            || !isFinite(controller.mMaximumAnimationAssistForce)
            || controller.mMaximumAnimationAssistForce < 0.0f
            || !isFinite(controller.mAnimationAssistUprightScale)
            || controller.mAnimationAssistUprightScale < 0.0f
            || controller.mAnimationAssistUprightScale > 1.0f
            || !isFinite(controller.mMaximumAnimationAssistTorque)
            || controller.mMaximumAnimationAssistTorque < 0.0f
            || !isFinite(controller.mAnimationAssistYawReferenceMaximumRate)
            || controller.mAnimationAssistYawReferenceMaximumRate <= 0.0f
            || !isFinite(controller.mAnimationAssistYawReferenceMaximumAcceleration)
            || controller.mAnimationAssistYawReferenceMaximumAcceleration <= 0.0f
            || !isFinite(controller.mAnimationAssistYawStiffness)
            || controller.mAnimationAssistYawStiffness < 0.0f
            || !isFinite(controller.mAnimationAssistYawDamping)
            || controller.mAnimationAssistYawDamping < 0.0f
            || !isFinite(controller.mMaximumAnimationAssistYawTorque)
            || controller.mMaximumAnimationAssistYawTorque < 0.0f
            || controller.mMaximumAnimationAssistYawTorque > controller.mMaximumAnimationAssistTorque
            || !isFinite(controller.mAnimationAssistPoseTrackingBlend)
            || controller.mAnimationAssistPoseTrackingBlend < 0.0f
            || controller.mAnimationAssistPoseTrackingBlend > 1.0f
            || !isFinite(controller.mAnimationAssistStanceAnchorCompliance)
            || controller.mAnimationAssistStanceAnchorCompliance < 0.0f
            || controller.mAnimationAssistStanceAnchorCompliance > 1.0f
            || !isFinite(controller.mAnimationAssistSupportLossHandoffDuration)
            || controller.mAnimationAssistSupportLossHandoffDuration <= 0.0f
            || !isFinite(controller.mAnimationAssistStancePositionCorrection)
            || controller.mAnimationAssistStancePositionCorrection < 0.0f
            || controller.mAnimationAssistStancePositionCorrection > 1.0f
            || !isFinite(controller.mAnimationAssistStanceMaximumEndpointCorrection)
            || controller.mAnimationAssistStanceMaximumEndpointCorrection <= 0.0f
            || !isFinite(controller.mAnimationAssistSupportMotionFrequency)
            || controller.mAnimationAssistSupportMotionFrequency <= 0.0f
            || !isFinite(controller.mAnimationAssistSupportMotionDamping)
            || controller.mAnimationAssistSupportMotionDamping < 0.0f
            || !isFinite(controller.mAnimationAssistSupportPositionLeash)
            || controller.mAnimationAssistSupportPositionLeash <= 0.0f
            || controller.mAnimationAssistContactConfirmationTicks == 0
            || controller.mAnimationAssistContactConfirmationTicks > 60
            || !isFinite(controller.mAnimationAssistLoadTransferDuration)
            || controller.mAnimationAssistLoadTransferDuration <= 0.0f
            || !isFinite(controller.mAnimationAssistMeasuredProgressBlend)
            || controller.mAnimationAssistMeasuredProgressBlend < 0.0f
            || controller.mAnimationAssistMeasuredProgressBlend > 1.0f
            || !isFinite(controller.mAnimationAssistFeasibilityDeadline)
            || controller.mAnimationAssistFeasibilityDeadline <= 0.0f
            || !isFinite(controller.mAnimationAssistDoubleSupportTransferDuration)
            || controller.mAnimationAssistDoubleSupportTransferDuration <= 0.0f
            || !isFinite(controller.mAnimationAssistReentrySlipSettleDuration)
            || controller.mAnimationAssistReentrySlipSettleDuration <= 0.0f
            || !isFinite(controller.mAnimationAssistTouchdownCapturePositionCorrection)
            || controller.mAnimationAssistTouchdownCapturePositionCorrection < 0.0f
            || controller.mAnimationAssistTouchdownCapturePositionCorrection > 1.0f
            || !isFinite(controller.mAnimationAssistTouchdownCaptureVelocityLookahead)
            || controller.mAnimationAssistTouchdownCaptureVelocityLookahead < 0.0f
            || !isFinite(controller.mAnimationAssistOutgoingHoldPositionCorrection)
            || controller.mAnimationAssistOutgoingHoldPositionCorrection < 0.0f
            || controller.mAnimationAssistOutgoingHoldPositionCorrection > 1.0f
            || !isFinite(controller.mAnimationAssistOutgoingHoldVelocityLookahead)
            || controller.mAnimationAssistOutgoingHoldVelocityLookahead < 0.0f
            || !isFinite(gait.mMinimumStepDuration) || gait.mMinimumStepDuration < 0.0f
            || !isFinite(gait.mNominalStepDuration)
            || gait.mNominalStepDuration <= 0.0f
            || gait.mNominalStepDuration < gait.mMinimumStepDuration
            || !isFinite(gait.mMaximumStepDuration)
            || gait.mMaximumStepDuration < gait.mNominalStepDuration
            || !isFinite(gait.mContactConfirmationDuration) || gait.mContactConfirmationDuration < 0.0f
            || !isFinite(gait.mStanceConfirmationDuration) || gait.mStanceConfirmationDuration < 0.0f
            || !isFinite(gait.mMaximumTouchdownHorizontalDistance)
            || gait.mMaximumTouchdownHorizontalDistance < 0.0f
            || !isFinite(gait.mMaximumLandingApproachVerticalDistance)
            || gait.mMaximumLandingApproachVerticalDistance < 0.0f
            || !isFinite(gait.mMaximumTouchdownRelativeVerticalSpeed)
            || gait.mMaximumTouchdownRelativeVerticalSpeed < 0.0f
            || !isFinite(gait.mMaximumStepReach) || gait.mMaximumStepReach <= 0.0f
            || !isFinite(gait.mNominalStepWidth) || gait.mNominalStepWidth < 0.0f
            || gait.mNominalStepWidth > 2.0f * gait.mMaximumStepReach
            || !isFinite(gait.mCapturePointBlend) || gait.mCapturePointBlend < 0.0f
            || gait.mCapturePointBlend > 1.0f
            || !isFinite(gait.mSwingFootLift) || gait.mSwingFootLift < 0.0f
            || !isFinite(gait.mSwingFootFrequency) || gait.mSwingFootFrequency < 0.0f
            || !isFinite(gait.mSwingFootDamping) || gait.mSwingFootDamping < 0.0f
            || !isFinite(gait.mMaximumSwingFootForce) || gait.mMaximumSwingFootForce < 0.0f
            || !isFinite(gait.mMaximumSwingFootHorizontalForce) || gait.mMaximumSwingFootHorizontalForce < 0.0f
            || !isFinite(gait.mPreLiftSwingUnloadFraction) || gait.mPreLiftSwingUnloadFraction < 0.0f
            || gait.mPreLiftSwingUnloadFraction > 1.0f
            || !isFinite(gait.mSwingLegMotorTorqueScale) || gait.mSwingLegMotorTorqueScale < 1.0f
            || gait.mSwingLegMotorTorqueScale > 8.0f
            || !isFinite(gait.mMaximumSwingHipTargetSpeed) || gait.mMaximumSwingHipTargetSpeed <= 0.0f
            || !isFinite(gait.mSwingHipWorldStiffness) || gait.mSwingHipWorldStiffness < 0.0f
            || !isFinite(gait.mSwingHipWorldDamping) || gait.mSwingHipWorldDamping < 0.0f
            || !isFinite(gait.mMaximumSwingHipWorldTorque) || gait.mMaximumSwingHipWorldTorque < 0.0f)
            addIssue(result, ValidationCode::InvalidController, "profile.activeController",
                "Active-controller thresholds, durations, gains, and limits must be finite, ordered, and non-negative.");

        std::unordered_map<std::string, std::size_t> skeletonBones;
        for (std::size_t i = 0; i < skeleton.mBones.size(); ++i)
            skeletonBones.emplace(skeleton.mBones[i].mName, i);

        std::unordered_set<std::string> aliases;
        for (std::size_t i = 0; i < profile.mBoneAliases.size(); ++i)
        {
            const BoneAliases& entry = profile.mBoneAliases[i];
            const std::string path = "profile.boneAliases[" + std::to_string(i) + "]";
            if (entry.mCanonical.empty())
                addIssue(result, ValidationCode::EmptyIdentifier, path + ".canonical", "Canonical bone must not be empty.");
            if (!aliases.insert(entry.mCanonical).second)
                addIssue(result, ValidationCode::DuplicateIdentifier, path + ".canonical",
                    "Canonical names and aliases must be globally unique.");
            for (std::size_t aliasIndex = 0; aliasIndex < entry.mAliases.size(); ++aliasIndex)
            {
                if (entry.mAliases[aliasIndex].empty())
                    addIssue(result, ValidationCode::EmptyIdentifier,
                        path + ".aliases[" + std::to_string(aliasIndex) + "]", "Alias must not be empty.");
                else if (!aliases.insert(entry.mAliases[aliasIndex]).second)
                    addIssue(result, ValidationCode::DuplicateIdentifier,
                        path + ".aliases[" + std::to_string(aliasIndex) + "]",
                        "Canonical names and aliases must be globally unique.");
            }
        }

        std::unordered_map<std::string, std::size_t> bodies;
        std::unordered_set<std::string> bodyBones;
        float massFractionSum = 0.0f;
        std::size_t rootCount = 0;
        for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
        {
            const BodyProfile& body = profile.mBodies[i];
            const std::string path = "profile.bodies[" + std::to_string(i) + "]";
            if (body.mName.empty())
                addIssue(result, ValidationCode::EmptyIdentifier, path + ".name", "Body name must not be empty.");
            else if (!bodies.emplace(body.mName, i).second)
                addIssue(result, ValidationCode::DuplicateIdentifier, path + ".name", "Body name must be unique.");

            if (body.mBone.empty() || !skeletonBones.contains(body.mBone))
                addIssue(result, ValidationCode::MissingBone, path + ".bone", "Body bone is absent from the skeleton.");
            else if (!bodyBones.insert(body.mBone).second)
                addIssue(result, ValidationCode::DuplicateIdentifier, path + ".bone", "A bone may drive only one body.");

            if (body.mParentBody)
            {
                if (!bodies.contains(*body.mParentBody))
                    addIssue(result, ValidationCode::InvalidHierarchy, path + ".parentBody",
                        "Parent body must precede the child body.");
                else if (skeletonBones.contains(body.mBone))
                {
                    const BodyProfile& parentBody = profile.mBodies[bodies.at(*body.mParentBody)];
                    if (skeletonBones.contains(parentBody.mBone))
                    {
                        int ancestor = skeleton.mBones[skeletonBones.at(body.mBone)].mParent;
                        const int expectedAncestor = static_cast<int>(skeletonBones.at(parentBody.mBone));
                        while (ancestor >= 0 && ancestor != expectedAncestor)
                            ancestor = skeleton.mBones[ancestor].mParent;
                        if (ancestor != expectedAncestor)
                            addIssue(result, ValidationCode::InvalidHierarchy, path + ".parentBody",
                                "Parent body bone must be an ancestor of the child body bone.");
                    }
                }
                if (!body.mConstraint)
                    addIssue(result, ValidationCode::InvalidConstraint, path + ".constraint",
                        "A non-root body requires a parent constraint.");
            }
            else
            {
                ++rootCount;
                if (body.mConstraint)
                    addIssue(result, ValidationCode::InvalidConstraint, path + ".constraint",
                        "The root body must not have a parent constraint.");
            }

            validateTransform(body.mShape.mLocalTransform, path + ".shape.localTransform", result);
            if (body.mShape.mFitToBone)
            {
                const auto target = skeletonBones.find(*body.mShape.mFitToBone);
                if (body.mShape.mType != ShapeType::Capsule)
                    addIssue(result, ValidationCode::InvalidShape, path + ".shape.fitToBone",
                        "Only capsule shapes can be fitted to a bone span.");
                if (body.mShape.mFitToBone->empty() || target == skeletonBones.end())
                    addIssue(result, ValidationCode::InvalidShape, path + ".shape.fitToBone",
                        "Fit target must name a skeleton bone.");
                else if (skeletonBones.contains(body.mBone))
                {
                    const std::size_t bodyBoneIndex = skeletonBones.at(body.mBone);
                    int ancestor = static_cast<int>(target->second);
                    while (ancestor >= 0 && static_cast<std::size_t>(ancestor) < skeleton.mBones.size()
                        && static_cast<std::size_t>(ancestor) != bodyBoneIndex)
                        ancestor = skeleton.mBones[ancestor].mParent;
                    if (ancestor < 0 || static_cast<std::size_t>(ancestor) >= skeleton.mBones.size()
                        || target->second == bodyBoneIndex)
                        addIssue(result, ValidationCode::InvalidShape, path + ".shape.fitToBone",
                            "Fit target must be a non-coincident descendant of the body bone.");
                    else
                    {
                        const JPH::Vec3 span = skeleton.mBones[target->second].mModelTransform.mTranslation
                            - skeleton.mBones[bodyBoneIndex].mModelTransform.mTranslation;
                        if (!isFinite(span) || span.LengthSq() <= 1e-8f)
                            addIssue(result, ValidationCode::InvalidShape, path + ".shape.fitToBone",
                                "Fit target must have a distinct finite model-space position.");
                    }
                }

                if (!body.mShape.mLocalTransform.mTranslation.IsNearZero()
                    || !body.mShape.mLocalTransform.mRotation.IsClose(JPH::Quat::sIdentity()))
                    addIssue(result, ValidationCode::InvalidShape, path + ".shape.localTransform",
                        "A fitted capsule cannot also define a local transform.");
            }
            if (!isFinite(body.mCenterOfMass))
                addIssue(result, ValidationCode::InvalidNumber, path + ".centerOfMass",
                    "Center of mass must contain finite values.");
            if (body.mShape.mType != ShapeType::ConvexHull && !body.mShape.mPoints.empty())
                addIssue(result, ValidationCode::InvalidShape, path + ".shape.points",
                    "Explicit points are only supported by convex hull shapes.");
            switch (body.mShape.mType)
            {
                case ShapeType::Box:
                    if (!isFinite(body.mShape.mHalfExtents) || body.mShape.mHalfExtents.GetX() <= 0.0f
                        || body.mShape.mHalfExtents.GetY() <= 0.0f || body.mShape.mHalfExtents.GetZ() <= 0.0f)
                        addIssue(result, ValidationCode::InvalidShape, path + ".shape.halfExtents",
                            "Box half extents must be finite and positive.");
                    break;
                case ShapeType::Capsule:
                    if (!isFinite(body.mShape.mRadius) || !isFinite(body.mShape.mHalfHeight)
                        || body.mShape.mRadius <= 0.0f || body.mShape.mHalfHeight < 0.0f)
                        addIssue(result, ValidationCode::InvalidShape, path + ".shape",
                            "Capsule radius must be positive and half height must be non-negative.");
                    break;
                case ShapeType::Sphere:
                    if (!isFinite(body.mShape.mRadius) || body.mShape.mRadius <= 0.0f)
                        addIssue(result, ValidationCode::InvalidShape, path + ".shape.radius",
                            "Sphere radius must be finite and positive.");
                    break;
                case ShapeType::ConvexHull:
                    if (!spansVolume(body.mShape.mPoints))
                        addIssue(result, ValidationCode::InvalidShape, path + ".shape.points",
                            "Convex hull points must be finite and span a non-degenerate three-dimensional volume.");
                    break;
                default:
                    addIssue(result, ValidationCode::InvalidShape, path + ".shape.type", "Unknown shape type.");
                    break;
            }

            if (!isFinite(body.mMassFraction) || body.mMassFraction <= 0.0f)
                addIssue(result, ValidationCode::InvalidMass, path + ".massFraction",
                    "Mass fraction must be finite and positive.");
            else
                massFractionSum += body.mMassFraction;

            const MaterialProfile& material = body.mMaterial;
            if (!isFinite(material.mFriction) || material.mFriction < 0.0f || !isFinite(material.mRestitution)
                || material.mRestitution < 0.0f || material.mRestitution > 1.0f
                || !isFinite(material.mLinearDamping) || material.mLinearDamping < 0.0f
                || !isFinite(material.mAngularDamping) || material.mAngularDamping < 0.0f)
                addIssue(result, ValidationCode::InvalidNumber, path + ".material",
                    "Material values must be finite; friction/damping are non-negative and restitution is in [0, 1].");

            if (body.mConstraint)
            {
                const ConstraintProfile& constraint = *body.mConstraint;
                if (constraint.mFrameAxisBone)
                {
                    const auto axisBone = skeletonBones.find(*constraint.mFrameAxisBone);
                    if (!constraint.mDeriveFrameFromSkeleton)
                        addIssue(result, ValidationCode::InvalidConstraint,
                            path + ".constraint.frameAxisBone",
                            "A frame-axis bone requires skeleton-derived constraint frames.");
                    if (constraint.mFrameAxisBone->empty() || axisBone == skeletonBones.end())
                        addIssue(result, ValidationCode::MissingBone, path + ".constraint.frameAxisBone",
                            "Frame-axis bone is absent from the skeleton.");
                    else if (skeletonBones.contains(body.mBone))
                    {
                        const std::size_t bodyBoneIndex = skeletonBones.at(body.mBone);
                        int ancestor = static_cast<int>(axisBone->second);
                        while (ancestor >= 0 && static_cast<std::size_t>(ancestor) < skeleton.mBones.size()
                            && static_cast<std::size_t>(ancestor) != bodyBoneIndex)
                            ancestor = skeleton.mBones[ancestor].mParent;
                        const JPH::Vec3 span = skeleton.mBones[axisBone->second].mModelTransform.mTranslation
                            - skeleton.mBones[bodyBoneIndex].mModelTransform.mTranslation;
                        if (ancestor < 0 || axisBone->second == bodyBoneIndex || !isFinite(span)
                            || span.LengthSq() <= 1e-8f)
                            addIssue(result, ValidationCode::InvalidConstraint,
                                path + ".constraint.frameAxisBone",
                                "Frame-axis bone must be a non-coincident descendant of the constrained body bone.");
                    }
                }
                constexpr float pi = std::numbers::pi_v<float>;
                const bool invalidType = constraint.mType != ConstraintType::SwingTwist
                    && constraint.mType != ConstraintType::Hinge;
                bool invalidLimits = invalidType;
                if (constraint.mType == ConstraintType::Hinge)
                    invalidLimits = !isFinite(constraint.mHingeMinAngle) || !isFinite(constraint.mHingeMaxAngle)
                        || constraint.mHingeMinAngle < -pi || constraint.mHingeMinAngle > 0.0f
                        || constraint.mHingeMaxAngle < 0.0f || constraint.mHingeMaxAngle > pi
                        || constraint.mHingeMinAngle > constraint.mHingeMaxAngle;
                else if (constraint.mType == ConstraintType::SwingTwist)
                    invalidLimits = !isFinite(constraint.mNormalHalfConeAngle)
                        || constraint.mNormalHalfConeAngle < 0.0f || constraint.mNormalHalfConeAngle > pi
                        || !isFinite(constraint.mPlaneHalfConeAngle) || constraint.mPlaneHalfConeAngle < 0.0f
                        || constraint.mPlaneHalfConeAngle > pi || !isFinite(constraint.mTwistMinAngle)
                        || !isFinite(constraint.mTwistMaxAngle) || constraint.mTwistMinAngle < -pi
                        || constraint.mTwistMaxAngle > pi || constraint.mTwistMinAngle > constraint.mTwistMaxAngle;
                if (!isFinite(constraint.mParentAnchor) || !isFinite(constraint.mChildAnchor) || invalidLimits
                    || !isFinite(constraint.mMaxFrictionTorque) || constraint.mMaxFrictionTorque < 0.0f)
                    addIssue(result, ValidationCode::InvalidConstraint, path + ".constraint",
                        "Constraint anchors, limits, and friction torque are invalid.");
                if (!isFinite(constraint.mMotor.mFrequency) || constraint.mMotor.mFrequency < 0.0f
                    || !isFinite(constraint.mMotor.mDamping) || constraint.mMotor.mDamping < 0.0f
                    || !isFinite(constraint.mMotor.mMaximumTorque) || constraint.mMotor.mMaximumTorque < 0.0f)
                    addIssue(result, ValidationCode::InvalidConstraint, path + ".constraint.motor",
                        "Motor frequency, damping, and maximum torque must be finite and non-negative.");
                validateAxes(constraint, path + ".constraint", result);
            }
        }

        if (!profile.mBodies.empty() && rootCount != 1)
            addIssue(result, ValidationCode::InvalidHierarchy, "profile.bodies", "Profile must contain exactly one root body.");
        if (!profile.mBodies.empty() && std::abs(massFractionSum - 1.0f) > 1e-3f)
            addIssue(result, ValidationCode::InvalidMass, "profile.bodies",
                "Body mass fractions must sum to 1 within 0.001.");

        if (profile.mLandmarks.mPelvis.empty() || !containsBody(bodies, profile.mLandmarks.mPelvis))
            addIssue(result, ValidationCode::InvalidLandmark, "profile.landmarks.pelvis",
                "Pelvis must name an existing body.");
        const auto validateLandmark = [&](const std::optional<std::string>& value, std::string_view path) {
            if (value && !containsBody(bodies, *value))
                addIssue(result, ValidationCode::InvalidLandmark, std::string(path), "Landmark must name an existing body.");
        };
        validateLandmark(profile.mLandmarks.mHead, "profile.landmarks.head");
        validateLandmark(profile.mLandmarks.mLeftHand, "profile.landmarks.leftHand");
        validateLandmark(profile.mLandmarks.mRightHand, "profile.landmarks.rightHand");
        validateLandmark(profile.mLandmarks.mLeftFoot, "profile.landmarks.leftFoot");
        validateLandmark(profile.mLandmarks.mRightFoot, "profile.landmarks.rightFoot");
        for (std::size_t i = 0; i < profile.mLandmarks.mSpine.size(); ++i)
        {
            if (!containsBody(bodies, profile.mLandmarks.mSpine[i]))
                addIssue(result, ValidationCode::InvalidLandmark,
                    "profile.landmarks.spine[" + std::to_string(i) + "]", "Landmark must name an existing body.");
        }
        return result;
    }
}
