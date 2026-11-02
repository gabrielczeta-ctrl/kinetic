#ifndef OPENMW_WALKSEARCH_RECOVERYPOSE_H
#define OPENMW_WALKSEARCH_RECOVERYPOSE_H

#include <ragdollcore/reactionpipeline.hpp>
#include <ragdollcore/scenario.hpp>

#include <algorithm>
#include <string>

// Rejected offline reference experiment, retained for reproducible diagnosis.
// The low pose exceeds this rig's hip/ankle cones; do not enable by default.
// These are motor targets, never body transforms: the shared physical solve
// retains limits, contacts and release. See RXN-01-recovery-preview-v1.md.
namespace RecoveryPreview
{
    inline float ease(float t)
    {
        t = std::clamp(t, 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    inline void rotateBranch(RagdollCore::RagdollPose& pose, const std::vector<int>& parents,
        std::size_t root, JPH::Vec3Arg axis, float angle)
    {
        if (root >= parents.size()) return;
        const auto q = JPH::Quat::sRotation(axis, angle);
        const auto pivot = pose.mBodyTransforms[root].mTranslation;
        for (std::size_t i = root; i < parents.size(); ++i)
        {
            int ancestor = static_cast<int>(i);
            while (ancestor > static_cast<int>(root)) ancestor = parents[ancestor];
            if (ancestor != static_cast<int>(root)) continue;
            auto& body = pose.mBodyTransforms[i];
            body.mTranslation = pivot + q * (body.mTranslation - pivot);
            body.mRotation = (q * body.mRotation).Normalized();
        }
    }

    inline RagdollCore::RagdollPose crouch(const RagdollCore::RagdollPose& standing,
        const RagdollCore::RagdollProfile& profile)
    {
        auto result = standing;
        const auto rig = RagdollCore::makeReactionPoseTargets(profile);
        const auto index = [&](const char* name) {
            const auto it = std::find_if(profile.mBodies.begin(), profile.mBodies.end(),
                [&](const auto& body) { return body.mName == name; });
            return static_cast<std::size_t>(it - profile.mBodies.begin());
        };
        const auto right = (standing.mBodyTransforms[rig.mRightUpperArm].mTranslation
            - standing.mBodyTransforms[rig.mLeftUpperArm].mTranslation).Normalized();
        rotateBranch(result, rig.mParents, 0, right, -0.30f);
        for (const auto side : { "left", "right" })
        {
            rotateBranch(result, rig.mParents, index((std::string(side) + "_thigh").c_str()), right, 1.25f);
            rotateBranch(result, rig.mParents, index((std::string(side) + "_calf").c_str()), right, -2.10f);
            rotateBranch(result, rig.mParents, index((std::string(side) + "_foot").c_str()), right, 1.15f);
            rotateBranch(result, rig.mParents, index((std::string(side) + "_upper_arm").c_str()), right, 0.50f);
            rotateBranch(result, rig.mParents, index((std::string(side) + "_forearm").c_str()), right, 0.45f);
        }
        return result;
    }

    // Interpolate relative joint rotations and rebuild the hierarchy. Blending
    // world-space positions would shorten limbs between the fallen and low pose.
    inline RagdollCore::RagdollPose blend(const RagdollCore::RagdollPose& from,
        const RagdollCore::RagdollPose& to, const std::vector<int>& parents, float weight)
    {
        auto result = from;
        for (std::size_t i = 0; i < parents.size(); ++i)
        {
            const int p = parents[i];
            const auto& a = from.mBodyTransforms[i];
            const auto& b = to.mBodyTransforms[i];
            auto& out = result.mBodyTransforms[i];
            if (p < 0)
            {
                out.mRotation = a.mRotation.SLERP(b.mRotation, weight).Normalized();
                out.mTranslation = a.mTranslation + weight * (b.mTranslation - a.mTranslation);
                continue;
            }
            const auto inverseA = from.mBodyTransforms[p].mRotation.Conjugated();
            const auto inverseB = to.mBodyTransforms[p].mRotation.Conjugated();
            const auto localA = inverseA * a.mRotation;
            const auto localB = inverseB * b.mRotation;
            out.mRotation = (result.mBodyTransforms[p].mRotation
                * localA.SLERP(localB, weight)).Normalized();
            const auto offset = inverseB * (b.mTranslation - to.mBodyTransforms[p].mTranslation);
            out.mTranslation = result.mBodyTransforms[p].mTranslation
                + result.mBodyTransforms[p].mRotation * offset;
        }
        return result;
    }
}
#endif
