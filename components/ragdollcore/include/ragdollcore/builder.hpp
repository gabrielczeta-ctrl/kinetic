#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_BUILDER_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_BUILDER_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Core/Reference.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
// clang-format on

#include <ragdollcore/profile.hpp>

namespace RagdollCore
{
    struct BuildOptions
    {
        float mTotalMass = 75.0f;
        float mUniformScale = 1.0f;
        JPH::RVec3 mRootPosition = JPH::RVec3::sZero();
        JPH::Quat mRootRotation = JPH::Quat::sIdentity();
        JPH::ObjectLayer mObjectLayer = 0;
        bool mUseContinuousCollision = true;
        // Where skeleton-derived constraint frames come from: a constraint
        // whose parent, child (and frame-axis) bones are all in this skeleton
        // takes its frame from them, so its limit cone centres on that pose
        // rather than on the build pose (a body built mid-stride kept the
        // stride as its joints' zero). Null, or a bone missing: the build pose.
        // Must outlive the build.
        const SkeletonDescriptor* mFrameSkeleton = nullptr;
    };

    struct BuildResult
    {
        JPH::Ref<JPH::RagdollSettings> mSettings;
        ValidationResult mValidation;

        bool isValid() const { return mSettings != nullptr && mValidation.isValid(); }
    };

    BuildResult buildRagdollSettings(
        const SkeletonDescriptor& skeleton, const RagdollProfile& profile, const BuildOptions& options);
}

#endif
