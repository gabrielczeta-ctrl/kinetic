#include <ragdollcore/bodylease.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    std::string_view bodyChannelName(BodyChannel channel)
    {
        switch (channel)
        {
            case BodyChannel::Root: return "root";
            case BodyChannel::LeftLeg: return "left_leg";
            case BodyChannel::RightLeg: return "right_leg";
            case BodyChannel::Trunk: return "trunk";
            case BodyChannel::Head: return "head";
            case BodyChannel::LeftArm: return "left_arm";
            case BodyChannel::RightArm: return "right_arm";
            case BodyChannel::Count: break;
        }
        return "none";
    }

    std::string_view bodyOwnerName(BodyOwner owner)
    {
        switch (owner)
        {
            case BodyOwner::Animation: return "animation";
            case BodyOwner::Rest: return "rest";
            case BodyOwner::Guard: return "guard";
            case BodyOwner::HandControl: return "hand_control";
            case BodyOwner::Locomotion: return "locomotion";
            case BodyOwner::Reaction: return "reaction";
            case BodyOwner::Balance: return "balance";
            case BodyOwner::Protect: return "protect";
            case BodyOwner::System: return "system";
            case BodyOwner::Count: break;
        }
        return "none";
    }

    int defaultLeasePriority(BodyOwner owner)
    {
        return static_cast<int>(owner) * 10;
    }

    std::optional<BodyChannel> bodyChannelForName(std::string_view name)
    {
        const auto starts = [&](std::string_view prefix) { return name.substr(0, prefix.size()) == prefix; };
        if (name == "pelvis")
            return BodyChannel::Root;
        if (name == "spine" || name == "spine1" || name == "spine2")
            return BodyChannel::Trunk;
        if (name == "neck" || name == "head")
            return BodyChannel::Head;
        if (starts("left_thigh") || starts("left_calf") || starts("left_foot"))
            return BodyChannel::LeftLeg;
        if (starts("right_thigh") || starts("right_calf") || starts("right_foot"))
            return BodyChannel::RightLeg;
        if (starts("left_clavicle") || starts("left_upper_arm") || starts("left_forearm") || starts("left_hand"))
            return BodyChannel::LeftArm;
        if (starts("right_clavicle") || starts("right_upper_arm") || starts("right_forearm") || starts("right_hand"))
            return BodyChannel::RightArm;
        return std::nullopt;
    }

    std::optional<BodyChannel> bodyChannelForRegion(AnatomicalRegion region)
    {
        switch (region)
        {
            case AnatomicalRegion::Pelvis: return BodyChannel::Root;
            case AnatomicalRegion::Abdomen:
            case AnatomicalRegion::Chest: return BodyChannel::Trunk;
            case AnatomicalRegion::Neck:
            case AnatomicalRegion::Head: return BodyChannel::Head;
            case AnatomicalRegion::LeftClavicle:
            case AnatomicalRegion::LeftUpperArm:
            case AnatomicalRegion::LeftForearm:
            case AnatomicalRegion::LeftHand: return BodyChannel::LeftArm;
            case AnatomicalRegion::RightClavicle:
            case AnatomicalRegion::RightUpperArm:
            case AnatomicalRegion::RightForearm:
            case AnatomicalRegion::RightHand: return BodyChannel::RightArm;
            case AnatomicalRegion::LeftThigh:
            case AnatomicalRegion::LeftCalf:
            case AnatomicalRegion::LeftFoot: return BodyChannel::LeftLeg;
            case AnatomicalRegion::RightThigh:
            case AnatomicalRegion::RightCalf:
            case AnatomicalRegion::RightFoot: return BodyChannel::RightLeg;
            default: return std::nullopt;
        }
    }

    namespace
    {
        float finiteOr(float value, float fallback)
        {
            return std::isfinite(value) ? value : fallback;
        }

        LeaseRequest sanitized(LeaseRequest request)
        {
            request.mPoseWeight = std::clamp(finiteOr(request.mPoseWeight, 0.0f), 0.0f, 1.0f);
            request.mTorqueScale = std::max(0.0f, finiteOr(request.mTorqueScale, 1.0f));
            request.mSpringScale = std::max(0.0f, finiteOr(request.mSpringScale, 1.0f));
            return request;
        }

        LeaseTiming sanitized(LeaseTiming timing)
        {
            timing.mDuration = std::isnan(timing.mDuration) ? 0.0f : std::max(0.0f, timing.mDuration);
            timing.mFadeIn = std::max(0.0f, finiteOr(timing.mFadeIn, 0.0f));
            timing.mFadeOut = std::max(0.0f, finiteOr(timing.mFadeOut, 0.0f));
            return timing;
        }
    }

    LeaseId BodyLeaseTable::acquire(BodyChannel channel, BodyOwner owner, const LeaseRequest& request,
        const LeaseTiming& timing, std::optional<int> priority)
    {
        if (channel == BodyChannel::Count || owner == BodyOwner::Count)
            return sNoLease;
        Lease lease;
        lease.mId = mNextId++;
        if (mNextId == sNoLease)
            mNextId = 1;
        lease.mChannel = channel;
        lease.mOwner = owner;
        lease.mPriority = priority.value_or(defaultLeasePriority(owner));
        lease.mRequest = sanitized(request);
        lease.mTiming = sanitized(timing);
        lease.mOrder = mNextOrder++;
        mLeases.push_back(lease);
        return lease.mId;
    }

    BodyLeaseTable::Lease* BodyLeaseTable::find(LeaseId id)
    {
        for (Lease& lease : mLeases)
            if (lease.mId == id)
                return &lease;
        return nullptr;
    }

    const BodyLeaseTable::Lease* BodyLeaseTable::find(LeaseId id) const
    {
        for (const Lease& lease : mLeases)
            if (lease.mId == id)
                return &lease;
        return nullptr;
    }

    bool BodyLeaseTable::update(LeaseId id, const LeaseRequest& request)
    {
        Lease* lease = find(id);
        if (!lease)
            return false;
        lease->mRequest = sanitized(request);
        return true;
    }

    bool BodyLeaseTable::renew(LeaseId id, float duration)
    {
        Lease* lease = find(id);
        if (!lease || std::isnan(duration))
            return false;
        // A renewed lease that was fading out holds again from where it is.
        lease->mReleasedAt.reset();
        lease->mTiming.mDuration = lease->mAge + std::max(0.0f, duration);
        return true;
    }

    void BodyLeaseTable::release(LeaseId id)
    {
        if (Lease* lease = find(id); lease && !lease->mReleasedAt)
            lease->mReleasedAt = lease->mAge;
    }

    void BodyLeaseTable::releaseNow(LeaseId id)
    {
        std::erase_if(mLeases, [&](const Lease& lease) { return lease.mId == id; });
    }

    void BodyLeaseTable::releaseOwner(BodyOwner owner)
    {
        for (Lease& lease : mLeases)
            if (lease.mOwner == owner && !lease.mReleasedAt)
                lease.mReleasedAt = lease.mAge;
    }

    void BodyLeaseTable::clear()
    {
        mLeases.clear();
    }

    float BodyLeaseTable::factor(const Lease& lease) const
    {
        const LeaseTiming& timing = lease.mTiming;
        const float in = timing.mFadeIn > 0.0f ? std::clamp(lease.mAge / timing.mFadeIn, 0.0f, 1.0f) : 1.0f;
        float end = timing.mDuration;
        if (lease.mReleasedAt)
            end = std::min(end, *lease.mReleasedAt);
        if (lease.mAge < end)
            return in;
        if (!(timing.mFadeOut > 0.0f))
            return 0.0f;
        // The fade-out starts from whatever the fade-in had reached.
        const float inAtEnd = timing.mFadeIn > 0.0f ? std::clamp(end / timing.mFadeIn, 0.0f, 1.0f) : 1.0f;
        return inAtEnd * std::clamp(1.0f - (lease.mAge - end) / timing.mFadeOut, 0.0f, 1.0f);
    }

    void BodyLeaseTable::step(float dt)
    {
        if (!std::isfinite(dt) || dt <= 0.0f)
            return;
        for (Lease& lease : mLeases)
            lease.mAge += dt;
        std::erase_if(mLeases, [&](const Lease& lease) {
            float end = lease.mTiming.mDuration;
            if (lease.mReleasedAt)
                end = std::min(end, *lease.mReleasedAt);
            return lease.mAge >= end + lease.mTiming.mFadeOut;
        });
    }

    ChannelResolution BodyLeaseTable::resolve(BodyChannel channel) const
    {
        ChannelResolution result;
        struct Active
        {
            const Lease* mLease;
            float mFactor;
        };
        std::vector<Active> active;
        for (const Lease& lease : mLeases)
            if (lease.mChannel == channel)
                if (const float f = factor(lease); f > 0.0f)
                    active.push_back({ &lease, f });
        std::sort(active.begin(), active.end(), [](const Active& a, const Active& b) {
            if (a.mLease->mPriority != b.mLease->mPriority)
                return a.mLease->mPriority < b.mLease->mPriority;
            return a.mLease->mOrder < b.mLease->mOrder;
        });

        // Compliance, lowest priority first: scales multiply; an exclusive
        // lease replaces what is below it (blended by its fade).
        for (const Active& item : active)
        {
            const LeaseRequest& request = item.mLease->mRequest;
            const float f = item.mFactor;
            if (request.mExclusiveCompliance)
            {
                result.mTorqueScale += (request.mTorqueScale - result.mTorqueScale) * f;
                result.mSpringScale += (request.mSpringScale - result.mSpringScale) * f;
            }
            else
            {
                result.mTorqueScale *= 1.0f + (request.mTorqueScale - 1.0f) * f;
                result.mSpringScale *= 1.0f + (request.mSpringScale - 1.0f) * f;
            }
        }

        // Pose layers: the top four, lowest first.
        std::vector<ChannelResolution::Layer> layers;
        for (const Active& item : active)
            if (item.mLease->mRequest.mPose)
                if (const float weight = item.mLease->mRequest.mPoseWeight * item.mFactor; weight > 0.0f)
                    layers.push_back({ item.mLease->mOwner, weight, item.mLease->mRequest.mAdditive,
                        item.mLease->mPriority, item.mLease->mReleasedAt.has_value() });
        const std::size_t first = layers.size() > result.mPoseLayers.size() ? layers.size() - result.mPoseLayers.size() : 0;
        for (std::size_t i = first; i < layers.size(); ++i)
            result.mPoseLayers[result.mPoseLayerCount++] = layers[i];
        for (std::size_t i = layers.size(); i-- > 0;)
            if (!layers[i].mAdditive && layers[i].mWeight >= 0.999f)
            {
                result.mPoseOwner = layers[i].mOwner;
                break;
            }

        // Motors off: the highest lease asking for it, unless a lease above
        // it drives the pose.
        for (std::size_t i = active.size(); i-- > 0;)
        {
            const Lease& lease = *active[i].mLease;
            if (lease.mRequest.mDriveOff && active[i].mFactor >= 0.5f)
            {
                result.mDriveOff = true;
                result.mDriveOffOwner = lease.mOwner;
                break;
            }
            if (lease.mRequest.mPose && !lease.mRequest.mAdditive
                && lease.mRequest.mPoseWeight * active[i].mFactor > 0.0f)
                break;
        }
        return result;
    }

    bool BodyLeaseTable::ownsPose(BodyChannel channel, BodyOwner owner) const
    {
        return resolve(channel).mPoseOwner == owner;
    }

    bool BodyLeaseTable::live(LeaseId id) const
    {
        return find(id) != nullptr;
    }

    float BodyLeaseTable::strength(LeaseId id) const
    {
        const Lease* lease = find(id);
        return lease ? factor(*lease) : 0.0f;
    }

    void BodyLeaseTable::noteWrite(BodyChannel channel, BodyOwner owner)
    {
        if (channel == BodyChannel::Count || owner == BodyOwner::Count || ownsPose(channel, owner))
            return;
        ++mConflicts;
        ++mConflictCounts[static_cast<std::size_t>(channel)][static_cast<std::size_t>(owner)];
    }
}
