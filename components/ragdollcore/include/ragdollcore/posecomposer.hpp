#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_POSECOMPOSER_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_POSECOMPOSER_H

// RXN-02 M1 proper: the channel leases compose the drive pose.
//
// The pose writers (swing body, arm targets, Footfall, the reflex, ...) still
// run in their order on the pose so far, because they build on one another
// (the arm solve reaches from the trunk the swing pose turned). What each one
// changes is recorded as a layer under its owner. compose() then rebuilds
// every body from the step's base with the layers of its channel in lease
// priority order: absolute layers blended over the result by their lease
// weight, additive layers (a reflex, a counterbalance) added on top. So:
// - a lower-priority writer that runs late no longer wins by accident;
// - an owner whose claim was released keeps its last layer while the lease
//   fades out (a smooth hand-over on any channel, when fades are set);
// - with claims only, no fades and writers in priority order the result is
//   the sequential pose exactly (compose reports how far it moved anything).
//
// Pure: no Jolt bodies, game or engine types.

#include <ragdollcore/bodylease.hpp>
#include <ragdollcore/passiveragdoll.hpp>

#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

namespace RagdollCore
{
    class PoseComposer
    {
    public:
        struct Result
        {
            std::size_t mWrites = 0; // bodies recorded this step
            std::size_t mInversions = 0; // bodies the composition moved off the sequential pose
            std::size_t mFading = 0; // bodies a released owner's last layer still shaped
            float mMaxTranslation = 0.0f; // the largest move off the sequential pose (units)
            float mMaxAngle = 0.0f; // and turn (radians)
            // Per inverted body: its channel, the owner whose write the
            // sequential pose ended with, and the owner the leases put on top.
            struct Inversion
            {
                BodyChannel mChannel;
                BodyOwner mSequential;
                BodyOwner mComposed;
            };
            std::vector<Inversion> mInverted;
        };

        // Starts a step on `base` (the pose before the first writer). Each
        // body's channel; an empty one (the pelvis root, the weapon, the
        // shield) is never recomposed.
        void begin(const RagdollPose& base, const std::vector<std::optional<BodyChannel>>& channels);
        bool active() const { return mActive; }

        // Brackets one writer. Bodies changed by code between writers (not
        // recorded) are taken into the base.
        void beforeWrite(const RagdollPose& pose);
        // `ownerFor(channel)` names who the write is for on that channel
        // (empty: kept as written, not recorded). Additive: the change is an
        // offset from what it was written on (a reflex), not a target.
        template <class OwnerFor>
        void afterWrite(const RagdollPose& pose, OwnerFor&& ownerFor, bool additive)
        {
            if (!mActive || pose.mBodyTransforms.size() != mBefore.mBodyTransforms.size())
                return;
            for (std::size_t body = 0; body < pose.mBodyTransforms.size() && body < mChannels.size(); ++body)
            {
                if (!mChannels[body] || !changed(mBefore.mBodyTransforms[body], pose.mBodyTransforms[body]))
                    continue;
                const std::optional<BodyOwner> owner = ownerFor(*mChannels[body]);
                if (!owner)
                    continue;
                record(body, *owner, additive, mBefore.mBodyTransforms[body], pose.mBodyTransforms[body]);
            }
            mSeen = pose;
        }

        // Rebuilds `pose` (the sequential result) from the base and the
        // layers, by the leases' priorities and weights. Ends the step.
        Result compose(RagdollPose& pose, const BodyLeaseTable& leases);

        // Ends the step without changing the pose (composition off), still
        // remembering each owner's last layer.
        void end();

        // Forgets every remembered layer (a reset body, a new skeleton).
        void reset();

    private:
        struct Write
        {
            std::size_t mBody = 0;
            BodyOwner mOwner = BodyOwner::Animation;
            bool mAdditive = false;
            Transform mValue; // absolute: the target; additive: the offset
            std::uint32_t mSequence = 0;
        };
        static bool changed(const Transform& a, const Transform& b);
        void record(std::size_t body, BodyOwner owner, bool additive, const Transform& before, const Transform& after);
        void remember();

        bool mActive = false;
        RagdollPose mBase;
        RagdollPose mBefore;
        RagdollPose mSeen; // the pose as the last writer left it
        std::vector<std::optional<BodyChannel>> mChannels;
        std::vector<Write> mWrites;
        std::uint32_t mSequence = 0;
        // Each owner's last absolute layer per body (blended out while its
        // lease is released and fading).
        std::map<std::pair<std::size_t, BodyOwner>, Transform> mLast;
    };
}

#endif
