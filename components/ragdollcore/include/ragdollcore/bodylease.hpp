#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_BODYLEASE_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_BODYLEASE_H

// RXN-02 M1: the body's channel leases (docs/ragdoll/designs/
// RXN-02-reaction-controller.md, 3.3). Every system that wants to move or
// soften a part of the body takes a lease on that part's channel: who it is
// (an owner with a priority), how long, and what it asks for - to drive the
// channel's pose, to scale its motors' torque or springs, or to switch its
// motors off. The table ages the leases (fade in, hold, fade out) and
// resolves each channel once per step; the engine composes the motor
// commands from the resolution instead of letting the last writer win.
// Pure: no Jolt, game or engine types.

#include <ragdollcore/anatomy.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string_view>
#include <vector>

namespace RagdollCore
{
    enum class BodyChannel : std::uint8_t
    {
        Root, // the soft root holding the pelvis
        LeftLeg,
        RightLeg,
        Trunk, // spine to chest
        Head, // neck and head
        LeftArm, // clavicle to hand
        RightArm,
        Count,
    };
    inline constexpr std::size_t sBodyChannelCount = static_cast<std::size_t>(BodyChannel::Count);

    // Lowest to highest default priority.
    enum class BodyOwner : std::uint8_t
    {
        Animation, // the clip the gameplay actor plays (the base layer)
        Rest, // a weapon at rest, sheathing
        Guard, // a held guard, a shield up
        HandControl, // the attack wheel, tip control, a native attack clip
        Locomotion, // Footfall's legs and the gait
        Reaction, // a blow's impact and reflex phases
        Balance, // PGC: bracing, catch steps, recovery
        Protect, // a protective fall, a get-up
        System, // released root, dead, debugging
        Count,
    };
    inline constexpr std::size_t sBodyOwnerCount = static_cast<std::size_t>(BodyOwner::Count);

    std::string_view bodyChannelName(BodyChannel channel);
    std::string_view bodyOwnerName(BodyOwner owner);
    int defaultLeasePriority(BodyOwner owner);
    // The channel a ragdoll body belongs to, by its profile name (pelvis is
    // the root's; the weapon and unknown names have none).
    std::optional<BodyChannel> bodyChannelForName(std::string_view name);
    // The same by anatomical region (Unknown has none).
    std::optional<BodyChannel> bodyChannelForRegion(AnatomicalRegion region);

    struct LeaseRequest
    {
        // The owner drives this channel's pose, blended over the layers
        // below it by this weight (0-1).
        bool mPose = false;
        float mPoseWeight = 1.0f;
        // An additive layer: the owner's pose is an offset from what is
        // below it (a flinch, a reflex), added at mPoseWeight instead of
        // replacing the lower layers. It never owns the channel.
        bool mAdditive = false;
        // Multipliers on the channel's motor torque limits and spring
        // frequency (1 = as the profile has them).
        float mTorqueScale = 1.0f;
        float mSpringScale = 1.0f;
        // Exclusive compliance: this lease's scales replace those of the
        // lower-priority leases instead of multiplying with them.
        bool mExclusiveCompliance = false;
        // Motors off on the channel (a slack mask) unless a higher-priority
        // lease drives its pose.
        bool mDriveOff = false;
    };

    struct LeaseTiming
    {
        float mDuration = std::numeric_limits<float>::infinity(); // held this long, then fades out
        float mFadeIn = 0.0f; // seconds from nothing to the full request
        float mFadeOut = 0.0f; // seconds from the full request to nothing
    };

    using LeaseId = std::uint32_t;
    inline constexpr LeaseId sNoLease = 0;

    // One channel's resolution for this step.
    struct ChannelResolution
    {
        struct Layer
        {
            BodyOwner mOwner = BodyOwner::Animation;
            float mWeight = 0.0f;
            bool mAdditive = false;
            int mPriority = 0;
            // Released and fading out: its owner has stopped writing, the
            // composer blends out the owner's last layer (PoseComposer).
            bool mReleasing = false;
        };
        // Pose layers above the animation, lowest priority first; compose
        // by blending each layer's pose over the result so far.
        std::array<Layer, 4> mPoseLayers{};
        std::uint8_t mPoseLayerCount = 0;
        // The top absolute pose layer at full weight, or the animation.
        BodyOwner mPoseOwner = BodyOwner::Animation;
        float mTorqueScale = 1.0f;
        float mSpringScale = 1.0f;
        bool mDriveOff = false;
        std::optional<BodyOwner> mDriveOffOwner;
    };

    class BodyLeaseTable
    {
    public:
        // A lease on one channel. Priority defaults to the owner's.
        LeaseId acquire(BodyChannel channel, BodyOwner owner, const LeaseRequest& request,
            const LeaseTiming& timing = {}, std::optional<int> priority = std::nullopt);
        // Changes a live lease's request (its timing is kept).
        bool update(LeaseId id, const LeaseRequest& request);
        // Holds a live lease for `duration` more seconds from now.
        bool renew(LeaseId id, float duration);
        // Starts the lease's fade-out now (removed when it has faded).
        void release(LeaseId id);
        // Removes the lease at once (no fade-out).
        void releaseNow(LeaseId id);
        void releaseOwner(BodyOwner owner);
        void clear();

        // Ages every lease by `dt` seconds and drops the faded ones.
        void step(float dt);

        ChannelResolution resolve(BodyChannel channel) const;
        // True when `owner` is the channel's top pose owner at full weight,
        // or nobody above the animation drives it and `owner` is Animation.
        bool ownsPose(BodyChannel channel, BodyOwner owner) const;
        bool live(LeaseId id) const;
        // 0-1: how much of the request is in effect (fades).
        float strength(LeaseId id) const;
        std::size_t size() const { return mLeases.size(); }

        // Audit: a system writing a channel it does not own. Counted per
        // owner and channel so the engine can trace the conflicts left.
        void noteWrite(BodyChannel channel, BodyOwner owner);
        std::uint64_t conflicts() const { return mConflicts; }
        std::uint64_t conflicts(BodyChannel channel, BodyOwner owner) const
        {
            return mConflictCounts[static_cast<std::size_t>(channel)][static_cast<std::size_t>(owner)];
        }

    private:
        struct Lease
        {
            LeaseId mId = sNoLease;
            BodyChannel mChannel = BodyChannel::Root;
            BodyOwner mOwner = BodyOwner::Animation;
            int mPriority = 0;
            LeaseRequest mRequest;
            LeaseTiming mTiming;
            float mAge = 0.0f; // seconds since acquired
            std::optional<float> mReleasedAt; // age at release (fade-out start)
            std::uint64_t mOrder = 0; // tie breaker: later wins
        };
        float factor(const Lease& lease) const;
        const Lease* find(LeaseId id) const;
        Lease* find(LeaseId id);

        std::vector<Lease> mLeases;
        LeaseId mNextId = 1;
        std::uint64_t mNextOrder = 0;
        std::uint64_t mConflicts = 0;
        std::array<std::array<std::uint64_t, sBodyOwnerCount>, sBodyChannelCount> mConflictCounts{};
    };
}

#endif
