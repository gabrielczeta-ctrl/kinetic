#include <ragdollcore/pipeline.hpp>

#include <Jolt/Math/Quat.h>

namespace RagdollCore
{
    std::string_view channelName(Channel channel)
    {
        switch (channel)
        {
            case Channel::Root:
                return "root";
            case Channel::PelvisHeight:
                return "pelvis_height";
            case Channel::LeftLeg:
                return "left_leg";
            case Channel::RightLeg:
                return "right_leg";
            case Channel::TrunkTilt:
                return "trunk_tilt";
            case Channel::TrunkTurn:
                return "trunk_turn";
            case Channel::Head:
                return "head";
            case Channel::LeftArm:
                return "left_arm";
            case Channel::RightArm:
                return "right_arm";
            case Channel::Count:
                break;
        }
        return "unknown";
    }

    std::string_view areaName(Area area)
    {
        switch (area)
        {
            case Area::Legacy:
                return "legacy";
            case Area::Lifecycle:
                return "lifecycle";
            case Area::Root:
                return "root";
            case Area::Legs:
                return "legs";
            case Area::Upright:
                return "upright";
            case Area::Reaction:
                return "reaction";
            case Area::Weapon:
                return "weapon";
            case Area::Count:
                break;
        }
        return "unknown";
    }

    Ownership resolveOwnership(const OwnershipInput& input)
    {
        Ownership owners{};
        const auto set = [&](Channel channel, Area area) { owners[static_cast<std::size_t>(channel)] = area; };
        const auto all = [&](Area area) {
            for (Area& owner : owners)
                owner = area;
        };
        switch (input.mState)
        {
            case BodyState::Dead:
                all(Area::Lifecycle);
                break;
            case BodyState::Falling:
            case BodyState::Downed:
            case BodyState::GettingUp:
                all(Area::Reaction);
                set(Channel::Root, Area::Root);
                break;
            case BodyState::Standing:
            case BodyState::Airborne:
            case BodyState::Staggered:
            {
                const bool staggered = input.mState == BodyState::Staggered;
                set(Channel::Root, Area::Root);
                set(Channel::PelvisHeight, Area::Legs);
                set(Channel::LeftLeg, Area::Legs);
                set(Channel::RightLeg, Area::Legs);
                set(Channel::TrunkTilt, staggered ? Area::Reaction : Area::Upright);
                set(Channel::Head, staggered ? Area::Reaction : Area::Upright);
                const Channel weaponArm = input.mWeaponRight ? Channel::RightArm : Channel::LeftArm;
                const Channel freeArm = input.mWeaponRight ? Channel::LeftArm : Channel::RightArm;
                if (input.mWeaponReady)
                {
                    const Area weaponSide = input.mReactionHasWeaponArm ? Area::Reaction : Area::Weapon;
                    set(weaponArm, weaponSide);
                    set(Channel::TrunkTurn, weaponSide);
                    set(freeArm, staggered ? Area::Reaction : Area::Weapon);
                }
                else
                {
                    set(weaponArm, staggered ? Area::Reaction : Area::Legs);
                    set(freeArm, staggered ? Area::Reaction : Area::Legs);
                    set(Channel::TrunkTurn, Area::Legs);
                }
                break;
            }
        }
        for (Area& owner : owners)
            if (!input.mMoved.moved(owner))
                owner = Area::Legacy;
        return owners;
    }

    ChannelMap channelMapFromNames(std::span<const std::string_view> bodyNames)
    {
        ChannelMap map;
        map.mBodyChannel.resize(bodyNames.size());
        map.mTurnShare.assign(bodyNames.size(), 0.0f);
        const auto starts = [](std::string_view name, std::string_view prefix) { return name.starts_with(prefix); };
        for (std::size_t i = 0; i < bodyNames.size(); ++i)
        {
            const std::string_view name = bodyNames[i];
            std::optional<Channel> channel;
            float share = 0.0f;
            if (name == "pelvis")
            {
                channel = Channel::Root;
                map.mPelvis = i;
            }
            else if (name == "spine" || name == "spine1" || name == "spine2")
            {
                channel = Channel::TrunkTilt;
                share = name == "spine" ? 0.3f : name == "spine1" ? 0.65f : 1.0f;
            }
            else if (name == "neck" || name == "head")
            {
                channel = Channel::Head;
                share = 1.0f;
            }
            else if (starts(name, "left_thigh") || starts(name, "left_calf") || starts(name, "left_foot"))
                channel = Channel::LeftLeg;
            else if (starts(name, "right_thigh") || starts(name, "right_calf") || starts(name, "right_foot"))
                channel = Channel::RightLeg;
            else if (starts(name, "left_clavicle") || starts(name, "left_upper_arm") || starts(name, "left_forearm")
                || starts(name, "left_hand"))
                channel = Channel::LeftArm;
            else if (starts(name, "right_clavicle") || starts(name, "right_upper_arm")
                || starts(name, "right_forearm") || starts(name, "right_hand"))
                channel = Channel::RightArm;
            map.mBodyChannel[i] = channel;
            map.mTurnShare[i] = share;
        }
        return map;
    }

    std::uint32_t StepComposer::Rejections::total() const
    {
        std::uint32_t sum = 0;
        for (const auto& writer : mCount)
            for (std::uint32_t count : writer)
                sum += count;
        return sum;
    }

    void StepComposer::begin(const RagdollPose& base, const ChannelMap& map, const Ownership& ownership)
    {
        mPose = base;
        mMap = &map;
        mOwnership = ownership;
        mTurn = 0.0f;
        mRoot.reset();
        mPelvisHeight.reset();
    }

    bool StepComposer::accept(Area writer, Channel channel)
    {
        if (owner(mOwnership, channel) == writer)
            return true;
        ++mRejections.mCount[static_cast<std::size_t>(writer)][static_cast<std::size_t>(channel)];
        return false;
    }

    bool StepComposer::writeBody(Area writer, std::size_t body, const Transform& target)
    {
        if (mMap == nullptr || body >= mPose.mBodyTransforms.size() || body >= mMap->mBodyChannel.size()
            || !mMap->mBodyChannel[body])
            return false;
        // The pelvis carries the root channel: its pose is the root's owner's.
        if (!accept(writer, *mMap->mBodyChannel[body]))
            return false;
        mPose.mBodyTransforms[body] = target;
        return true;
    }

    bool StepComposer::writeTrunkTurn(Area writer, float radians)
    {
        if (!accept(writer, Channel::TrunkTurn))
            return false;
        mTurn = radians;
        return true;
    }

    bool StepComposer::writeRoot(Area writer, const JPH::Vec3& horizontalTarget)
    {
        if (!accept(writer, Channel::Root))
            return false;
        mRoot = horizontalTarget;
        return true;
    }

    bool StepComposer::writePelvisHeight(Area writer, float height)
    {
        if (!accept(writer, Channel::PelvisHeight))
            return false;
        mPelvisHeight = height;
        return true;
    }

    RagdollPose StepComposer::compose() const
    {
        RagdollPose pose = mPose;
        if (mMap == nullptr || mTurn == 0.0f || !mMap->mPelvis || *mMap->mPelvis >= pose.mBodyTransforms.size())
            return pose;
        const JPH::Vec3 pivot = pose.mBodyTransforms[*mMap->mPelvis].mTranslation;
        const JPH::Vec3 up = mMap->mUp.Normalized();
        for (std::size_t body = 0; body < pose.mBodyTransforms.size() && body < mMap->mTurnShare.size(); ++body)
        {
            const float share = mMap->mTurnShare[body];
            if (share == 0.0f)
                continue;
            const JPH::Quat turn = JPH::Quat::sRotation(up, share * mTurn);
            Transform& transform = pose.mBodyTransforms[body];
            transform.mTranslation = pivot + turn * (transform.mTranslation - pivot);
            transform.mRotation = (turn * transform.mRotation).Normalized();
        }
        return pose;
    }
}
