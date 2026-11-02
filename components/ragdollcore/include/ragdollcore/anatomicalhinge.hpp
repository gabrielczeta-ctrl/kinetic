#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_ANATOMICALHINGE_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_ANATOMICALHINGE_H

#include "profile.hpp"
#include <cmath>

namespace RagdollCore
{
    // Give a two-segment hinge an anatomical zero at full extension. The
    // captured pose supplies the bend plane, not a new zero for its limits.
    // Explicitly authored frames are left to their author. Elbows use positive
    // flexion; the opt-in knee adapter uses negative flexion and supplies the
    // actor's anatomical lateral axis when a straight pose has no bend plane.
    inline bool setAnatomicalHingeFrame(ConstraintProfile& joint, const Transform& upper,
        const Transform& lower, const Transform& end, JPH::Vec3Arg parentCom, JPH::Vec3Arg childCom,
        float bendSign = 1.0f, std::optional<JPH::Vec3> straightAxis = std::nullopt)
    {
        const auto upperOffset = lower.mTranslation - upper.mTranslation;
        const auto lowerOffset = end.mTranslation - lower.mTranslation;
        if (joint.mType != ConstraintType::Hinge || !joint.mDeriveFrameFromSkeleton
            || !std::isfinite(upperOffset.LengthSq()) || upperOffset.LengthSq() < 1e-8f
            || !std::isfinite(lowerOffset.LengthSq()) || lowerOffset.LengthSq() < 1e-8f
            || !upper.mRotation.IsNormalized() || !lower.mRotation.IsNormalized()
            || (bendSign != 1.0f && bendSign != -1.0f)
            || (straightAxis && (!std::isfinite(straightAxis->LengthSq()) || straightAxis->IsNearZero())))
            return false;
        const auto proximal = upperOffset.Normalized();
        const auto distal = lowerOffset.Normalized();
        auto hinge = proximal.Cross(distal) * bendSign;
        if (hinge.LengthSq() < 1e-6f)
        {
            const auto reference = straightAxis ? *straightAxis
                : std::abs(proximal.Dot(JPH::Vec3::sAxisZ())) < 0.9f
                    ? JPH::Vec3::sAxisZ() : JPH::Vec3::sAxisX();
            hinge = reference - proximal * proximal.Dot(reference);
            if (hinge.LengthSq() < 1e-8f)
                return false;
        }
        hinge = hinge.Normalized();
        const auto parentInverse = upper.mRotation.Conjugated();
        const auto childInverse = lower.mRotation.Conjugated();
        joint.mParentAnchor = parentInverse * upperOffset - parentCom;
        joint.mChildAnchor = -childCom;
        joint.mParentTwistAxis = parentInverse * proximal;
        joint.mParentPlaneAxis = parentInverse * proximal.Cross(hinge).Normalized();
        joint.mChildTwistAxis = childInverse * distal;
        joint.mChildPlaneAxis = childInverse * distal.Cross(hinge).Normalized();
        joint.mDeriveFrameFromSkeleton = false;
        return true;
    }
}
#endif
