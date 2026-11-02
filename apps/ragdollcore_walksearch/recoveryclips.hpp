#ifndef OPENMW_WALKSEARCH_RECOVERYCLIPS_H
#define OPENMW_WALKSEARCH_RECOVERYCLIPS_H

#include <ragdollcore/passiveragdoll.hpp>
#include <components/ragdollprofile/skeletonadapter.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/resource/keyframemanager.hpp>
#include <components/sceneutil/skeleton.hpp>
#include <components/vfs/bsaarchive.hpp>
#include <components/vfs/manager.hpp>

#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace WalkSearch
{
    struct RecoveryClip
    {
        std::string mGroup;
        float mDuration = 0;
        float mSpeed = 0;
        std::vector<RagdollCore::RagdollPose> mFrames;
    };

    // Uses the same isolated source sampler as native recovery. No renderer,
    // game process, or animation files copied into the repository are needed.
    inline std::array<RecoveryClip, 2> loadRecoveryClips(
        const std::filesystem::path& archive, const RagdollCore::RagdollProfile& profile)
    {
        VFS::Manager vfs;
        vfs.addArchive(VFS::makeBsaArchive(archive, nullptr));
        vfs.buildIndex();
        Resource::ResourceSystem resources(&vfs, 0.0, nullptr);
        auto node = resources.getSceneManager()->getInstance(VFS::Path::Normalized("meshes/base_anim.nif"));
        osg::ref_ptr<SceneUtil::Skeleton> skeleton = dynamic_cast<SceneUtil::Skeleton*>(node.get());
        if (!skeleton)
        {
            skeleton = new SceneUtil::Skeleton;
            skeleton->addChild(node);
        }
        auto binding = RagdollProfile::snapshotSkeleton(profile, *skeleton);
        if (!binding.isValid())
            throw std::runtime_error("Recovery source skeleton does not match the captured-body profile");
        auto source = resources.getKeyframeManager()->get(VFS::Path::Normalized("meshes/xbase_anim.kf"));
        std::array<RecoveryClip, 2> result;
        for (std::size_t i = 0; i < result.size(); ++i)
        {
            auto& clip = result[i];
            clip.mGroup = i == 0 ? "knockdown" : "knockout";
            std::optional<float> start, end;
            const auto findSegment = [&] {
                start.reset(); end.reset();
                for (const auto& [time, key] : source->mTextKeys)
                {
                    if (key == clip.mGroup + ": loop stop") start = time;
                    if (key == clip.mGroup + ": stop") end = time;
                }
                return start && end && std::isfinite(*start) && std::isfinite(*end) && *end > *start;
            };
            // Native worldimp.cpp uses the other group when a model lacks the
            // requested segment. Stock Morrowind's knockdown has no loop stop.
            if (!findSegment())
            {
                clip.mGroup = i == 0 ? "knockout" : "knockdown";
                findSegment();
            }
            if (!start || !end || !std::isfinite(*start) || !std::isfinite(*end) || *end <= *start)
            {
                std::string found;
                for (const auto& [time, key] : source->mTextKeys)
                    if (key.starts_with(clip.mGroup)) found += " [" + std::to_string(time) + " " + key + "]";
                throw std::runtime_error("Recovery clip lacks loop-stop/stop keys: " + clip.mGroup + found);
            }
            clip.mDuration = *end - *start;
            const auto ticks = static_cast<std::size_t>(std::ceil(clip.mDuration * 60));
            for (std::size_t tick = 0; tick <= ticks; ++tick)
            {
                RagdollCore::RagdollPose pose;
                if (!binding.mPoseBinding->sampleAnimationPose(*source,
                        std::min(*end, *start + static_cast<float>(tick) / 60), pose.mBodyTransforms,
                        "Bip01", JPH::Vec3(1, 1, 0)))
                    throw std::runtime_error("Invalid recovery animation sample: " + clip.mGroup);
                clip.mFrames.push_back(std::move(pose));
            }
        }
        return result;
    }

    inline RecoveryClip loadWalkingClip(
        const std::filesystem::path& archive, const RagdollCore::RagdollProfile& profile)
    {
        VFS::Manager vfs;
        vfs.addArchive(VFS::makeBsaArchive(archive, nullptr));
        vfs.buildIndex();
        Resource::ResourceSystem resources(&vfs, 0.0, nullptr);
        auto node = resources.getSceneManager()->getInstance(VFS::Path::Normalized("meshes/base_anim.nif"));
        osg::ref_ptr<SceneUtil::Skeleton> skeleton = dynamic_cast<SceneUtil::Skeleton*>(node.get());
        if (!skeleton) { skeleton = new SceneUtil::Skeleton; skeleton->addChild(node); }
        auto binding = RagdollProfile::snapshotSkeleton(profile, *skeleton);
        if (!binding.isValid()) throw std::runtime_error("Walking source skeleton does not match the profile");
        const auto source = resources.getKeyframeManager()->get(VFS::Path::Normalized("meshes/xbase_anim.kf"));
        const auto cycle = binding.mPoseBinding->sampleAnimationCycle(*source, "walkforwardhh", "Bip01", JPH::Vec3(1,1,0), 120);
        if (!cycle.isValid()) throw std::runtime_error("Invalid walkforwardhh cycle: " + cycle.mError
            + " translation=" + std::to_string(cycle.mMaximumSeamTranslation)
            + " rotation_dot=" + std::to_string(cycle.mMinimumSeamRotationDot));
        RecoveryClip clip;
        clip.mGroup = "walkforwardhh";
        clip.mDuration = cycle.mEndTime - cycle.mStartTime;
        for (const auto& frame : cycle.mFrames) clip.mFrames.push_back({ frame });
        std::vector<RagdollCore::Transform> first, last;
        if (!binding.mPoseBinding->sampleAnimationPose(*source, cycle.mStartTime, first, "Bip01", JPH::Vec3::sZero())
            || !binding.mPoseBinding->sampleAnimationPose(*source, cycle.mEndTime, last, "Bip01", JPH::Vec3::sZero()))
            throw std::runtime_error("Walking source has no accumulation motion");
        auto delta = last[0].mTranslation - first[0].mTranslation;
        delta.SetZ(0);
        clip.mSpeed = delta.Length() / clip.mDuration;
        if (!std::isfinite(clip.mSpeed) || clip.mSpeed < 1)
            throw std::runtime_error("Walking source has no finite forward speed");
        return clip;
    }
}
#endif
