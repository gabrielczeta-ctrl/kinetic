#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_WEAPONALIGNMENT_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_WEAPONALIGNMENT_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Quat.h>
// clang-format on

#include <optional>
#include <string_view>

namespace RagdollCore
{
    // The two weapon-local directions a strike has to present. Local here
    // means the held weapon's frame, which is also the hand's frame: the grip
    // is a rigid attachment. The long axis runs from the grip to the tip or
    // head; the edge axis spans a blade's cutting edge, an axe's bit or a
    // hammer head's face.
    struct WeaponStrikeAxes
    {
        JPH::Vec3 mLongAxis = JPH::Vec3::sAxisY();
        JPH::Vec3 mEdgeAxis = JPH::Vec3::sAxisX();
    };

    // World hand rotation that points the long axis along strikeDirection and
    // rolls the edge toward swingTangent (projected perpendicular to the
    // strike). The result is a target for the arm solver, not a collision or
    // reach guarantee. Returns nullopt for a degenerate direction or axis.
    // A tangent parallel to the strike (or missing) keeps the roll that the
    // long-axis alignment produced, so a thrust never spins the wrist.
    std::optional<JPH::Quat> alignedStrikeHandRotation(const WeaponStrikeAxes& axes,
        JPH::Vec3Arg strikeDirection, JPH::Vec3Arg swingTangent);

    // Opposing edges/faces share one cutting plane. Choose the equivalent roll
    // nearest the previous target; retain that roll when the tangent vanishes.
    // The long axis still points exactly at the aim.
    std::optional<JPH::Quat> continuousStrikeHandRotation(const WeaponStrikeAxes& axes,
        JPH::Vec3Arg strikeDirection, JPH::Vec3Arg swingTangent, JPH::QuatArg previous);

    // Authored shape data for weapons whose silhouette the generic longest
    // mesh axis does not describe well enough for a strike. Values are in the
    // held weapon's local frame, verified against the equipped mesh:
    //   - iron claymore:    blade along +Z, edges along X;
    //   - iron battle axe:  haft along +Z with the head at the +Z end, bit
    //                       spanning X;
    //   - iron warhammer:   haft along +Z with the head at the +Z end, face
    //                       spanning X.
    // mStrikeStart is the fraction of grip-to-tip distance where the striking
    // part begins: forward of it a contact is blade/head, behind it is haft,
    // guard or butt.
    struct WeaponShapeProfile
    {
        JPH::Vec3 mStrikeAxis = JPH::Vec3::sAxisZ();
        JPH::Vec3 mEdgeAxis = JPH::Vec3::sAxisX();
        float mStrikeStart = 0.0f;
    };

    std::optional<WeaponShapeProfile> weaponShapeProfile(std::string_view record);
}

#endif
