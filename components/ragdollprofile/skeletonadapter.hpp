#ifndef OPENMW_COMPONENTS_RAGDOLLPROFILE_SKELETONADAPTER_H
#define OPENMW_COMPONENTS_RAGDOLLPROFILE_SKELETONADAPTER_H

#include <ragdollcore/profile.hpp>

#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace SceneUtil
{
    class Skeleton;
    class KeyframeHolder;
}

namespace RagdollProfile
{
    struct SkeletonSnapshotResult;

    struct AnimationCycleSample
    {
        float mStartTime = 0;
        float mEndTime = 0;
        float mMaximumSeamTranslation = 0;
        float mMinimumSeamRotationDot = 1;
        // Uniform phase samples excluding the repeated endpoint.
        std::vector<std::vector<RagdollCore::Transform>> mFrames;
        std::string mError;
        bool isValid() const { return mError.empty() && mFrames.size() >= 2; }
    };

    class SkeletonPoseBinding
    {
    public:
        ~SkeletonPoseBinding();
        SkeletonPoseBinding(SkeletonPoseBinding&&) noexcept;
        SkeletonPoseBinding& operator=(SkeletonPoseBinding&&) noexcept;

        SkeletonPoseBinding(const SkeletonPoseBinding&) = delete;
        SkeletonPoseBinding& operator=(const SkeletonPoseBinding&) = delete;

        // Publishes and immediately applies a complete model-space pose. An
        // update callback reapplies it after animation controllers run. Returns
        // false before publishing or changing any node if validation fails.
        bool applyPose(std::span<const RagdollCore::Transform> pose);

        // Captures evaluated model-space transforms in profile body order,
        // including configured leg target normalization. Does not change the
        // skeleton or install a callback.
        bool capturePose(std::vector<RagdollCore::Transform>& pose) const;

        // Reads the actual evaluated skeleton without animation-target
        // normalization. Use for physical/render comparisons: a fallen or
        // stepping skeleton need not have symmetric legs.
        bool captureEvaluatedPose(std::vector<RagdollCore::Transform>& pose) const;

        // Captures the most recent animation-evaluated pose observed by the
        // post-traversal physics override. This keeps powered targets separate
        // from the ragdoll pose currently presented by the skeleton nodes.
        bool captureAnimationPose(std::vector<RagdollCore::Transform>& pose) const;

        // Samples an owned animation source at an explicit source time, in the
        // same bound body order. Uses isolated controllers/transforms; does not
        // traverse, modify, or change time on the live skeleton or source.
        // Root-motion removal and cycle validation belong to the caller.
        bool sampleAnimationPose(const SceneUtil::KeyframeHolder& source, float time,
            std::vector<RagdollCore::Transform>& pose, std::string_view accumulationBone = {},
            JPH::Vec3 accumulationAxes = JPH::Vec3(1, 1, 0)) const;

        AnimationCycleSample sampleAnimationCycle(const SceneUtil::KeyframeHolder& source, std::string_view group,
            std::string_view accumulationBone, JPH::Vec3 accumulationAxes, std::size_t sampleCount) const;

    private:
        struct Impl;
        explicit SkeletonPoseBinding(std::unique_ptr<Impl> impl);

        std::unique_ptr<Impl> mImpl;

        friend SkeletonSnapshotResult snapshotSkeleton(const RagdollCore::RagdollProfile&, SceneUtil::Skeleton&);
    };

    struct SkeletonSnapshotIssue
    {
        std::string mPath;
        std::string mMessage;
        std::optional<RagdollCore::ValidationCode> mValidationCode;
    };

    struct SkeletonSnapshotResult
    {
        std::optional<RagdollCore::SkeletonDescriptor> mSkeleton;
        std::unique_ptr<SkeletonPoseBinding> mPoseBinding;
        std::vector<SkeletonSnapshotIssue> mIssues;

        bool isValid() const { return mSkeleton.has_value() && mPoseBinding != nullptr && mIssues.empty(); }
    };

    // Resolves profile bone names and aliases against one SceneUtil skeleton,
    // then snapshots the currently evaluated model-space transforms into the
    // plain Jolt/STL core representation. The returned data owns no OSG nodes.
    SkeletonSnapshotResult snapshotSkeleton(
        const RagdollCore::RagdollProfile& profile, SceneUtil::Skeleton& skeleton);
}

#endif
