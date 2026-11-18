#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_RUNTIMEPROFILE_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_RUNTIMEPROFILE_HPP

#include <ragdollcore/profile.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace RagdollCore
{
    // A powered body's per-build profile, as the game makes it (MOVE-01 W1:
    // StrideSim builds the same body). The elbows, and with `knees` the knees,
    // get anatomical hinge frames from this skeleton (setAnatomicalHingeFrame):
    // zero at full extension, the bend plane from the pose. Returns the joint
    // that could not be framed ("elbow" or "knee"), or nothing.
    std::optional<std::string_view> applyAnatomicalFrames(
        RagdollProfile& runtime, const SkeletonDescriptor& skeleton, bool knees);

    // The skeleton a body was built from, written by the game
    // (OPENMW_RAGDOLL_SKELETON_DUMP) and read by StrideSim: the bones in
    // actor-root space, the standing frame skeleton when the build used one,
    // the uniform scale and the total mass. Plain text, one bone a line.
    struct SkeletonDump
    {
        SkeletonDescriptor mSkeleton;
        std::optional<SkeletonDescriptor> mFrames;
        float mUniformScale = 1.0f;
        float mTotalMass = 75.0f;
    };

    bool writeSkeletonDump(const std::string& path, const SkeletonDump& dump);
    std::optional<SkeletonDump> readSkeletonDump(const std::string& path);

    // The animation pose the game handed Footfall each physics step (the
    // clip, model space, profile body order), with the asked horizontal speed,
    // written by the game (OPENMW_RAGDOLL_CLIP_DUMP) and played back by
    // StrideSim, so both step from the same base pose (MOVE-01 W1).
    std::string clipTrackLine(float requestSpeed, const std::vector<Transform>& bodies);
    struct ClipTrack
    {
        std::vector<float> mRequestSpeed;
        std::vector<std::vector<Transform>> mPoses;
    };
    std::optional<ClipTrack> readClipTrack(const std::string& path);
}

#endif
