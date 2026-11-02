#include <ragdollcore/builder.hpp>
#include <ragdollcore/tuning.hpp>
#include <ragdollcore/unitmodel.hpp>

// clang-format off
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Skeleton/Skeleton.h>
// clang-format on

#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>

namespace RagdollCore
{
    namespace
    {
        bool isFinite(float value)
        {
            return std::isfinite(value);
        }

        bool isFinite(JPH::RVec3Arg value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        }

        bool isFinite(JPH::QuatArg value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ())
                && std::isfinite(value.GetW());
        }

        JPH::RVec3 scaled(JPH::Vec3Arg value, float scale)
        {
            return JPH::RVec3(value.GetX() * scale, value.GetY() * scale, value.GetZ() * scale);
        }

        JPH::Ref<JPH::Shape> createShape(const BodyProfile& body, const SkeletonBone& bone,
            const std::unordered_map<std::string, const SkeletonBone*>& skeletonBones, float scale,
            std::string& error)
        {
            const ShapeProfile& profile = body.mShape;
            // DUEL-A01 spawn stretch (2026-09-24): the head sphere is lifted off
            // its pivot by an authored local transform (humanoid-v1.yaml). A/B:
            // OPENMW_RAGDOLL_SHAPE_LOCAL_TRANSFORM=0 ignores authored shape
            // offsets (only the head has one) and restores the old geometry.
            static const auto useAuthoredLocalTransform_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SHAPE_LOCAL_TRANSFORM");
                return text == nullptr || text[0] != '0';
            });
            static const auto& useAuthoredLocalTransform = useAuthoredLocalTransform_tuned.get();
            Transform localTransform = useAuthoredLocalTransform ? profile.mLocalTransform : Transform{};
            float capsuleRadius = profile.mRadius;
            float capsuleHalfHeight = profile.mHalfHeight;
            if (profile.mFitToBone)
            {
                const SkeletonBone& target = *skeletonBones.at(*profile.mFitToBone);
                const JPH::Vec3 modelSpan = target.mModelTransform.mTranslation - bone.mModelTransform.mTranslation;
                const float spanLength = modelSpan.Length();
                const JPH::Vec3 localDirection
                    = bone.mModelTransform.mRotation.Conjugated() * (modelSpan / spanLength);

                // Preserve the authored thickness. A short span can be round;
                // it does not need an artificially thin cylindrical section.
                capsuleHalfHeight = std::max(0.0f, spanLength * 0.5f - capsuleRadius);
                localTransform.mTranslation = localDirection * (spanLength * 0.5f);
                localTransform.mRotation = JPH::Quat::sFromTo(JPH::Vec3::sAxisY(), localDirection);
            }

            JPH::Ref<JPH::Shape> shape;
            switch (profile.mType)
            {
                case ShapeType::Box:
                {
                    // Probe (2026-09-11, strafe-native-input): Jolt's default
                    // convex radius is 0.05 in shape units, i.e. under a
                    // millimetre in this 70 units/m world, so box parts are
                    // sharp-cornered and sink into mesh triangle edges.
                    // OPENMW_RAGDOLL_BOX_RADIUS=r rounds every box part by r
                    // units (clamped below the smallest half extent).
                    // Env-gated; unset is byte-identical.
                    static const auto boxRadiusEnv_tuned = RagdollCore::tuned<float>([] {
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_BOX_RADIUS");
                        const float parsed = text ? std::strtof(text, nullptr) : 0.0f;
                        return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
                    });
                    static const auto& boxRadiusEnv = boxRadiusEnv_tuned.get();
                    const JPH::Vec3 halfExtents = profile.mHalfExtents * scale;
                    float radius = JPH::cDefaultConvexRadius;
                    if (boxRadiusEnv > 0.0f)
                        radius = std::min(boxRadiusEnv, 0.9f * halfExtents.ReduceMin());
                    shape = new JPH::BoxShape(halfExtents, radius);
                    break;
                }
                    break;
                case ShapeType::Capsule:
                    if (capsuleHalfHeight == 0.0f)
                        shape = new JPH::SphereShape(capsuleRadius * scale);
                    else
                        shape = new JPH::CapsuleShape(capsuleHalfHeight * scale, capsuleRadius * scale);
                    break;
                case ShapeType::Sphere:
                    shape = new JPH::SphereShape(profile.mRadius * scale);
                    break;
                case ShapeType::ConvexHull:
                {
                    JPH::Array<JPH::Vec3> points;
                    points.reserve(profile.mPoints.size());
                    for (const JPH::Vec3& point : profile.mPoints)
                    {
                        const JPH::Vec3 scaledPoint = point * scale;
                        if (!isFinite(JPH::RVec3(scaledPoint)))
                        {
                            error = "Convex hull points overflow at the requested uniform scale.";
                            return nullptr;
                        }
                        points.push_back(scaledPoint);
                    }
                    // Use Jolt's existing default maximum convex radius. Jolt
                    // reduces it for narrow geometry and shrinks the core so
                    // rounding does not add an outward collision skin.
                    const JPH::ConvexHullShapeSettings hull(points, JPH::cDefaultConvexRadius);
                    const auto created = hull.Create();
                    if (created.HasError())
                    {
                        error = std::string("Convex hull creation failed: ") + created.GetError().c_str();
                        return nullptr;
                    }
                    shape = created.Get();
                    if (!std::isfinite(shape->GetVolume()) || shape->GetVolume() <= 0)
                    {
                        error = "Convex hull creation did not produce a finite positive volume.";
                        return nullptr;
                    }
                    break;
                }
            }

            const Transform& transform = localTransform;
            if (!transform.mTranslation.IsNearZero() || !transform.mRotation.IsClose(JPH::Quat::sIdentity()))
                shape = new JPH::RotatedTranslatedShape(
                    transform.mTranslation * scale, transform.mRotation, shape.GetPtr());

            const JPH::Vec3 desiredCenterOfMass = body.mCenterOfMass * scale;
            const JPH::Vec3 centerOfMassOffset = desiredCenterOfMass - shape->GetCenterOfMass();
            if (!centerOfMassOffset.IsNearZero())
                shape = new JPH::OffsetCenterOfMassShape(shape.GetPtr(), centerOfMassOffset);
            return shape;
        }

        struct ConstraintFrame
        {
            JPH::RVec3 mPosition1;
            JPH::RVec3 mPosition2;
            JPH::Vec3 mTwistAxis1;
            JPH::Vec3 mPlaneAxis1;
            JPH::Vec3 mTwistAxis2;
            JPH::Vec3 mPlaneAxis2;
        };

        ConstraintFrame createConstraintFrame(const ConstraintProfile& profile,
            const SkeletonBone& parentBone, const SkeletonBone& childBone, const SkeletonBone* frameAxisBone,
            JPH::Vec3Arg parentCenterOfMass, JPH::Vec3Arg childCenterOfMass, float scale)
        {
            ConstraintFrame result;
            if (profile.mDeriveFrameFromSkeleton)
            {
                const JPH::Vec3 modelOffset
                    = childBone.mModelTransform.mTranslation - parentBone.mModelTransform.mTranslation;
                const JPH::Vec3 modelAxis = frameAxisBone != nullptr
                    ? frameAxisBone->mModelTransform.mTranslation - childBone.mModelTransform.mTranslation
                    : modelOffset;
                const JPH::Vec3 modelTwist = modelAxis.Normalized();
                const JPH::Vec3 reference
                    = std::abs(modelTwist.Dot(JPH::Vec3::sAxisZ())) < 0.9f ? JPH::Vec3::sAxisZ() : JPH::Vec3::sAxisX();
                const JPH::Vec3 modelPlane = modelTwist.Cross(reference).Normalized();
                const JPH::Quat parentInverse = parentBone.mModelTransform.mRotation.Conjugated();
                const JPH::Quat childInverse = childBone.mModelTransform.mRotation.Conjugated();

                result.mPosition1 = scaled(parentInverse * modelOffset, scale) - JPH::RVec3(parentCenterOfMass);
                result.mPosition2 = -JPH::RVec3(childCenterOfMass);
                result.mTwistAxis1 = parentInverse * modelTwist;
                result.mPlaneAxis1 = parentInverse * modelPlane;
                result.mTwistAxis2 = childInverse * modelTwist;
                result.mPlaneAxis2 = childInverse * modelPlane;
            }
            else
            {
                result.mPosition1 = scaled(profile.mParentAnchor, scale);
                result.mPosition2 = scaled(profile.mChildAnchor, scale);
                result.mTwistAxis1 = profile.mParentTwistAxis;
                result.mPlaneAxis1 = profile.mParentPlaneAxis;
                result.mTwistAxis2 = profile.mChildTwistAxis;
                result.mPlaneAxis2 = profile.mChildPlaneAxis;
            }
            return result;
        }

        JPH::Ref<JPH::TwoBodyConstraintSettings> createConstraint(const ConstraintProfile& profile,
            const SkeletonBone& parentBone, const SkeletonBone& childBone, const SkeletonBone* frameAxisBone,
            JPH::Vec3Arg parentCenterOfMass, JPH::Vec3Arg childCenterOfMass, float scale)
        {
            const ConstraintFrame frame = createConstraintFrame(
                profile, parentBone, childBone, frameAxisBone, parentCenterOfMass, childCenterOfMass, scale);
            if (profile.mType == ConstraintType::Hinge)
            {
                JPH::Ref<JPH::TwoBodyConstraintSettings> result = new JPH::HingeConstraintSettings;
                auto* hinge = static_cast<JPH::HingeConstraintSettings*>(result.GetPtr());
                hinge->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
                hinge->mPoint1 = frame.mPosition1;
                hinge->mPoint2 = frame.mPosition2;
                hinge->mHingeAxis1 = frame.mPlaneAxis1.Cross(frame.mTwistAxis1);
                hinge->mNormalAxis1 = frame.mTwistAxis1;
                hinge->mHingeAxis2 = frame.mPlaneAxis2.Cross(frame.mTwistAxis2);
                hinge->mNormalAxis2 = frame.mTwistAxis2;
                hinge->mLimitsMin = profile.mHingeMinAngle;
                hinge->mLimitsMax = profile.mHingeMaxAngle;
                hinge->mMaxFrictionTorque = profile.mMaxFrictionTorque;
                hinge->mMotorSettings = JPH::MotorSettings(profile.mMotor.mFrequency,
                    profile.mMotor.mDamping, 0.0f, profile.mMotor.mMaximumTorque);
                return result;
            }

            JPH::Ref<JPH::TwoBodyConstraintSettings> result = new JPH::SwingTwistConstraintSettings;
            auto* swingTwist = static_cast<JPH::SwingTwistConstraintSettings*>(result.GetPtr());
            swingTwist->mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            swingTwist->mPosition1 = frame.mPosition1;
            swingTwist->mPosition2 = frame.mPosition2;
            swingTwist->mTwistAxis1 = frame.mTwistAxis1;
            swingTwist->mPlaneAxis1 = frame.mPlaneAxis1;
            swingTwist->mTwistAxis2 = frame.mTwistAxis2;
            swingTwist->mPlaneAxis2 = frame.mPlaneAxis2;
            swingTwist->mNormalHalfConeAngle = profile.mNormalHalfConeAngle;
            swingTwist->mPlaneHalfConeAngle = profile.mPlaneHalfConeAngle;
            swingTwist->mTwistMinAngle = profile.mTwistMinAngle;
            swingTwist->mTwistMaxAngle = profile.mTwistMaxAngle;
            swingTwist->mMaxFrictionTorque = profile.mMaxFrictionTorque;
            swingTwist->mSwingMotorSettings = swingTwist->mTwistMotorSettings
                = JPH::MotorSettings(profile.mMotor.mFrequency, profile.mMotor.mDamping, 0.0f,
                    profile.mMotor.mMaximumTorque);
            return result;
        }
    }

    BuildResult buildRagdollSettings(
        const SkeletonDescriptor& skeleton, const RagdollProfile& profile, const BuildOptions& options)
    {
        BuildResult result;
        result.mValidation = validateProfile(profile, skeleton);
        if (!isFinite(options.mTotalMass) || options.mTotalMass <= 0.0f)
            result.mValidation.mIssues.push_back({ ValidationCode::InvalidMass, "options.totalMass",
                "Total mass must be finite and positive." });
        if (!isFinite(options.mUniformScale) || options.mUniformScale <= 0.0f)
            result.mValidation.mIssues.push_back({ ValidationCode::InvalidNumber, "options.uniformScale",
                "Uniform scale must be finite and positive." });
        if (!isFinite(options.mRootPosition) || !isFinite(options.mRootRotation)
            || !options.mRootRotation.IsNormalized(1e-3f))
            result.mValidation.mIssues.push_back({ ValidationCode::InvalidNumber, "options.rootTransform",
                "Root transform must contain finite values and a normalized rotation." });
        if (!result.mValidation.isValid())
            return result;

        std::unordered_map<std::string, const SkeletonBone*> skeletonBones;
        for (const SkeletonBone& bone : skeleton.mBones)
            skeletonBones.emplace(bone.mName, &bone);

        std::unordered_map<std::string, const BodyProfile*> profileBodies;
        for (const BodyProfile& body : profile.mBodies)
            profileBodies.emplace(body.mName, &body);
        for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
        {
            const BodyProfile& body = profile.mBodies[i];
            if (!body.mParentBody || !body.mConstraint || !body.mConstraint->mDeriveFrameFromSkeleton)
                continue;
            const SkeletonBone& parentBone = *skeletonBones.at(profileBodies.at(*body.mParentBody)->mBone);
            const SkeletonBone& childBone = *skeletonBones.at(body.mBone);
            const JPH::Vec3 offset = childBone.mModelTransform.mTranslation - parentBone.mModelTransform.mTranslation;
            if (offset.LengthSq() <= 1e-8f)
                result.mValidation.mIssues.push_back({ ValidationCode::InvalidConstraint,
                    "profile.bodies[" + std::to_string(i) + "].constraint.deriveFrame",
                    "Cannot derive a constraint frame from coincident parent and child bones." });
        }
        if (!result.mValidation.isValid())
            return result;

        std::unordered_map<std::string, const SkeletonBone*> frameBones;
        if (options.mFrameSkeleton != nullptr)
            for (const SkeletonBone& frameBone : options.mFrameSkeleton->mBones)
                frameBones.emplace(frameBone.mName, &frameBone);

        std::unordered_map<std::string, int> bodyIndices;
        JPH::Ref<JPH::RagdollSettings> settings = new JPH::RagdollSettings;
        JPH::Ref<JPH::Skeleton> joltSkeleton = new JPH::Skeleton;
        settings->mParts.reserve(profile.mBodies.size());

        for (const BodyProfile& body : profile.mBodies)
        {
            int parentIndex = -1;
            if (body.mParentBody)
                parentIndex = bodyIndices.at(*body.mParentBody);
            const int bodyIndex = parentIndex < 0 ? joltSkeleton->AddJoint(body.mName.c_str())
                                                 : joltSkeleton->AddJoint(body.mName.c_str(), parentIndex);
            bodyIndices.emplace(body.mName, bodyIndex);

            const SkeletonBone& bone = *skeletonBones.at(body.mBone);
            JPH::RagdollSettings::Part& part = settings->mParts.emplace_back();
            std::string shapeError;
            const auto shape = createShape(body, bone, skeletonBones, options.mUniformScale, shapeError);
            if (!shape)
            {
                result.mValidation.mIssues.push_back({ ValidationCode::InvalidShape,
                    "profile.bodies[" + std::to_string(bodyIndex) + "].shape", std::move(shapeError) });
                return result;
            }
            part.SetShape(shape.GetPtr());
            const JPH::Vec3 modelOffset = options.mRootRotation * (bone.mModelTransform.mTranslation * options.mUniformScale);
            part.mPosition = options.mRootPosition + modelOffset;
            part.mRotation = options.mRootRotation * bone.mModelTransform.mRotation;
            part.mMotionType = JPH::EMotionType::Dynamic;
            part.mObjectLayer = options.mObjectLayer;
            part.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            part.mMassPropertiesOverride.mMass = options.mTotalMass * body.mMassFraction;
            part.mLinearDamping = body.mMaterial.mLinearDamping;
            part.mAngularDamping = body.mMaterial.mAngularDamping;
            part.mMaxLinearVelocity = UnitModel::fromMeters(500.0f);
            part.mFriction = body.mMaterial.mFriction;
            part.mRestitution = body.mMaterial.mRestitution;
            part.mMotionQuality
                = options.mUseContinuousCollision ? JPH::EMotionQuality::LinearCast : JPH::EMotionQuality::Discrete;
            part.mAllowSleeping = true;
            // Match ordinary actor collision handling: triangle seams are
            // not physical ledges for feet, hands or knees to push against.
            part.mEnhancedInternalEdgeRemoval = true;
            part.mGravityFactor = 1.0f;
            if (body.mConstraint)
            {
                const BodyProfile& parentBody = *profileBodies.at(*body.mParentBody);
                const SkeletonBone* parentBone = skeletonBones.at(parentBody.mBone);
                const SkeletonBone* childBone = &bone;
                const SkeletonBone* frameAxisBone = body.mConstraint->mFrameAxisBone
                    ? skeletonBones.at(*body.mConstraint->mFrameAxisBone)
                    : nullptr;
                if (body.mConstraint->mDeriveFrameFromSkeleton)
                {
                    const auto parentFrame = frameBones.find(parentBody.mBone);
                    const auto childFrame = frameBones.find(body.mBone);
                    const auto axisFrame = body.mConstraint->mFrameAxisBone
                        ? frameBones.find(*body.mConstraint->mFrameAxisBone)
                        : frameBones.end();
                    if (parentFrame != frameBones.end() && childFrame != frameBones.end()
                        && (!body.mConstraint->mFrameAxisBone || axisFrame != frameBones.end()))
                    {
                        parentBone = parentFrame->second;
                        childBone = childFrame->second;
                        if (body.mConstraint->mFrameAxisBone)
                            frameAxisBone = axisFrame->second;
                    }
                }
                const JPH::Vec3 parentCenterOfMass = settings->mParts[parentIndex].GetShape()->GetCenterOfMass();
                part.mToParent = createConstraint(*body.mConstraint, *parentBone, *childBone, frameAxisBone,
                    parentCenterOfMass, part.GetShape()->GetCenterOfMass(), options.mUniformScale);
            }
        }

        settings->mSkeleton = joltSkeleton;
        JPH::Ref<JPH::GroupFilterTable> groupFilter
            = new JPH::GroupFilterTable(static_cast<JPH::uint>(settings->mParts.size()));
        for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
        {
            const BodyProfile& body = profile.mBodies[i];
            if (body.mParentBody)
                groupFilter->DisableCollision(static_cast<JPH::uint>(i), bodyIndices.at(*body.mParentBody));
            for (std::size_t j = 0; j < i; ++j)
            {
                if (body.mSelfCollisionGroup != 0
                    && body.mSelfCollisionGroup == profile.mBodies[j].mSelfCollisionGroup)
                    groupFilter->DisableCollision(static_cast<JPH::uint>(i), static_cast<JPH::uint>(j));
            }
            settings->mParts[i].mCollisionGroup.SetSubGroupID(static_cast<JPH::uint>(i));
            settings->mParts[i].mCollisionGroup.SetGroupFilter(groupFilter);
        }
        settings->CalculateBodyIndexToConstraintIndex();
        settings->CalculateConstraintIndexToBodyIdxPair();
        // Preserve the authored mass and shape-derived inertia. Validate the
        // tensor without Stabilize(), which redistributes mass and increases
        // inertia to improve solver conditioning.
        for (std::size_t i = 0; i < settings->mParts.size(); ++i)
        {
            const auto mass = settings->mParts[i].GetMassProperties();
            JPH::Mat44 rotation;
            JPH::Vec3 diagonal;
            if (!isFinite(mass.mMass) || mass.mMass <= 0.0f
                || !isFinite(JPH::RVec3(mass.mInertia.GetColumn3(0)))
                || !isFinite(JPH::RVec3(mass.mInertia.GetColumn3(1)))
                || !isFinite(JPH::RVec3(mass.mInertia.GetColumn3(2)))
                || !mass.DecomposePrincipalMomentsOfInertia(rotation, diagonal)
                || !isFinite(JPH::RVec3(diagonal))
                || diagonal.GetX() <= 0.0f || diagonal.GetY() <= 0.0f || diagonal.GetZ() <= 0.0f)
            {
                result.mValidation.mIssues.push_back({ ValidationCode::InvalidMass,
                    "profile.bodies[" + std::to_string(i) + "]", "Derived mass and inertia must be finite and positive." });
                return result;
            }
        }
        result.mSettings = std::move(settings);
        return result;
    }
}
