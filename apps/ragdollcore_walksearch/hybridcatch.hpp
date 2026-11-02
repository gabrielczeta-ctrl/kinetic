#ifndef OPENMW_WALKSEARCH_HYBRIDCATCH_H
#define OPENMW_WALKSEARCH_HYBRIDCATCH_H

#include "recoverypose.hpp"
#include <ragdollcore/scenario.hpp>
#include <limits>

namespace WalkSearch
{
    struct HybridCatch
    {
        std::uint8_t mStage = 1;
        float mTime = 0, mStableTime = 0;
        bool mSawPhysicalGait = false;

        void step(const RagdollCore::ScenarioFrame* previous, bool hit, bool enabled, float dt)
        {
            if (hit) { mStage = 2; mTime = mStableTime = 0; mSawPhysicalGait = false; return; }
            mTime += dt;
            if (mStage == 3 && mTime >= 0.75f) mStage = 1;
            if (mStage != 2 || !previous) return;
            mSawPhysicalGait |= previous->mReferenceGait.has_value() && !previous->mRootKinematic;
            const auto mode = previous->mController.mMode;
            auto error = previous->mCenterOfMassVelocity - previous->mRequestedHorizontalVelocity;
            error.SetZ(0);
            const bool stable = enabled && !previous->mRootKinematic
                && (mode == RagdollCore::ActiveControllerMode::Standing || mode == RagdollCore::ActiveControllerMode::Bracing)
                && previous->mPelvisUpDot >= 0.85f && previous->mControllerInput.mTorsoUpDot >= 0.8f
                && previous->mSupport.hasSupport() && previous->mSupport.mCenterOfMassDistance <= 4.0f
                && error.Length() <= 20.0f;
            mStableTime = stable ? mStableTime + dt : 0;
            if (mTime >= 1.0f && mStableTime >= 0.25f && mSawPhysicalGait
                && previous->mSupport.mLeftFootSupported && previous->mSupport.mRightFootSupported)
            { mStage = 3; mTime = 0; }
        }
    };

    template <class Clip>
    RagdollCore::RagdollPose sampleWalk(const Clip& clip, float time,
        const RagdollCore::ReactionPoseTargets& rig)
    {
        const float phase = time / clip.mDuration - std::floor(time / clip.mDuration);
        const float position = phase * clip.mFrames.size();
        const auto a = static_cast<std::size_t>(position) % clip.mFrames.size();
        return RecoveryPreview::blend(clip.mFrames[a], clip.mFrames[(a + 1) % clip.mFrames.size()],
            rig.mParents, position - std::floor(position));
    }

    template <class Clip>
    float matchWalkPhase(const Clip& clip, const RagdollCore::RagdollPose& measured,
        const RagdollCore::ReactionPoseTargets& rig)
    {
        float best = std::numeric_limits<float>::infinity(), time = 0;
        for (std::size_t f = 0; f < clip.mFrames.size(); ++f)
        {
            float cost = 0;
            for (std::size_t b = 1; b < rig.mParents.size(); ++b)
            {
                const auto p = rig.mParents[b];
                if (p < 0) continue;
                const auto a = measured.mBodyTransforms[p].mRotation.Conjugated() * measured.mBodyTransforms[b].mRotation;
                const auto c = clip.mFrames[f].mBodyTransforms[p].mRotation.Conjugated() * clip.mFrames[f].mBodyTransforms[b].mRotation;
                cost += 1 - std::abs(a.Dot(c));
            }
            if (cost < best) { best = cost; time = clip.mDuration * static_cast<float>(f) / clip.mFrames.size(); }
        }
        return time;
    }
}
#endif
