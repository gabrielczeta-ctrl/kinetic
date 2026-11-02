#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_ANATOMY_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_ANATOMY_HPP

#include <cstdint>
#include <string_view>

namespace RagdollCore
{
    // Coarse humanoid hit regions derived from the profile's canonical body
    // names. The region is resolved once per strike from the contact snapshot
    // and decides how much a blow hurts and which reaction tier it can reach.
    enum class AnatomicalRegion : std::uint8_t
    {
        Unknown = 0,
        Head,
        Neck,
        Chest,
        Abdomen,
        Pelvis,
        LeftClavicle,
        RightClavicle,
        LeftUpperArm,
        RightUpperArm,
        LeftForearm,
        RightForearm,
        LeftHand,
        RightHand,
        LeftThigh,
        RightThigh,
        LeftCalf,
        RightCalf,
        LeftFoot,
        RightFoot,
    };

    inline AnatomicalRegion regionForBodyName(std::string_view name) noexcept
    {
        if (name == "head")
            return AnatomicalRegion::Head;
        if (name == "neck")
            return AnatomicalRegion::Neck;
        if (name == "spine2")
            return AnatomicalRegion::Chest;
        if (name == "spine1" || name == "spine")
            return AnatomicalRegion::Abdomen;
        if (name == "pelvis")
            return AnatomicalRegion::Pelvis;
        if (name == "left_clavicle")
            return AnatomicalRegion::LeftClavicle;
        if (name == "right_clavicle")
            return AnatomicalRegion::RightClavicle;
        if (name == "left_upper_arm")
            return AnatomicalRegion::LeftUpperArm;
        if (name == "right_upper_arm")
            return AnatomicalRegion::RightUpperArm;
        if (name == "left_forearm")
            return AnatomicalRegion::LeftForearm;
        if (name == "right_forearm")
            return AnatomicalRegion::RightForearm;
        if (name == "left_hand")
            return AnatomicalRegion::LeftHand;
        if (name == "right_hand")
            return AnatomicalRegion::RightHand;
        if (name == "left_thigh")
            return AnatomicalRegion::LeftThigh;
        if (name == "right_thigh")
            return AnatomicalRegion::RightThigh;
        if (name == "left_calf")
            return AnatomicalRegion::LeftCalf;
        if (name == "right_calf")
            return AnatomicalRegion::RightCalf;
        if (name == "left_foot")
            return AnatomicalRegion::LeftFoot;
        if (name == "right_foot")
            return AnatomicalRegion::RightFoot;
        return AnatomicalRegion::Unknown;
    }

    inline const char* regionName(AnatomicalRegion region) noexcept
    {
        switch (region)
        {
            case AnatomicalRegion::Unknown: return "unknown";
            case AnatomicalRegion::Head: return "head";
            case AnatomicalRegion::Neck: return "neck";
            case AnatomicalRegion::Chest: return "chest";
            case AnatomicalRegion::Abdomen: return "abdomen";
            case AnatomicalRegion::Pelvis: return "pelvis";
            case AnatomicalRegion::LeftClavicle: return "left_clavicle";
            case AnatomicalRegion::RightClavicle: return "right_clavicle";
            case AnatomicalRegion::LeftUpperArm: return "left_upper_arm";
            case AnatomicalRegion::RightUpperArm: return "right_upper_arm";
            case AnatomicalRegion::LeftForearm: return "left_forearm";
            case AnatomicalRegion::RightForearm: return "right_forearm";
            case AnatomicalRegion::LeftHand: return "left_hand";
            case AnatomicalRegion::RightHand: return "right_hand";
            case AnatomicalRegion::LeftThigh: return "left_thigh";
            case AnatomicalRegion::RightThigh: return "right_thigh";
            case AnatomicalRegion::LeftCalf: return "left_calf";
            case AnatomicalRegion::RightCalf: return "right_calf";
            case AnatomicalRegion::LeftFoot: return "left_foot";
            case AnatomicalRegion::RightFoot: return "right_foot";
        }
        return "unknown";
    }

    /// Head and torso hits cannot drop a standing body by themselves; the legs
    /// and pelvis are what fail. Unknown cannot either: every profile body is
    /// mapped, so in play an unknown contact is the victim's equipped weapon,
    /// and a blow caught on a blade must not take the legs out. One
    /// overwhelming blow still drops a body wherever it lands
    /// (HitReactionState).
    inline bool regionCanKnockDown(AnatomicalRegion region) noexcept
    {
        switch (region)
        {
            case AnatomicalRegion::Pelvis:
            case AnatomicalRegion::LeftThigh:
            case AnatomicalRegion::RightThigh:
            case AnatomicalRegion::LeftCalf:
            case AnatomicalRegion::RightCalf:
            case AnatomicalRegion::LeftFoot:
            case AnatomicalRegion::RightFoot:
                return true;
            default:
                return false;
        }
    }

    inline bool regionIsHead(AnatomicalRegion region) noexcept
    {
        return region == AnatomicalRegion::Head || region == AnatomicalRegion::Neck;
    }

    inline bool regionIsExtremity(AnatomicalRegion region) noexcept
    {
        return region == AnatomicalRegion::LeftHand || region == AnatomicalRegion::RightHand
            || region == AnatomicalRegion::LeftFoot || region == AnatomicalRegion::RightFoot;
    }
}

#endif
