#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_FALLPROTECTPOSE_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_FALLPROTECTPOSE_HPP

#include <ragdollcore/anatomy.hpp>
#include <ragdollcore/controller.hpp>
#include <ragdollcore/fallprotect.hpp>
#include <ragdollcore/profile.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace RagdollCore
{
    // KINETIC-2 phase 3, the reaction path's fall protection: what the
    // integration did with FallProtectController's command, moved here behind
    // OPENMW_RAGDOLL_K2_REACTION (the old path beside it until the user's
    // verdict) - the protective pose on the animation's pose, the first
    // contact's radii by region and the motor springs' scales.
    //
    // Pure: model-space transforms in, out.

    struct FallPoseChains
    {
        std::vector<std::size_t> mSpine; // pelvis-up order; the last is the chest
        std::size_t mHead = 0;
        std::optional<std::size_t> mNeck;
        struct Arm
        {
            std::size_t mFrame = 0, mShoulder = 0, mElbow = 0, mHand = 0;
        };
        std::array<Arm, 2> mArms{}; // left, right
        struct Leg
        {
            std::size_t mHip = 0, mKnee = 0, mFoot = 0;
        };
        std::array<Leg, 2> mLegs{};
    };

    // The command on `target` (model space, every body): the spine curled
    // forward about its base (spread up the spine, the head, neck and arms
    // carried), the hips and knees bent, the head pitched about the neck, and
    // each upper arm turned toward its world direction as seen from the
    // chest (`chestWorldRotation`) by its weight. The sagittal plane comes
    // from the clavicles.
    void applyFallProtectPose(std::vector<Transform>& target, const FallProtectCommand& command,
        const FallPoseChains& chains, const JPH::Quat& chestWorldRotation);

    // The first contact: a body of this region counts as down within the
    // radius of the floor (the head 9, hands and forearms 4, calves and upper
    // arms 5, thighs 6, the rest 10). Never the feet; the legs only with
    // `legContacts` (OPENMW_RAGDOLL_FALL_PROTECT=2: a knee down is no catch,
    // the arms still brace the trunk). nullopt: the region does not count.
    std::optional<float> fallContactRadius(AnatomicalRegion region, bool legContacts);
    // A contact of the trunk, not a hand or forearm.
    bool fallBodyContact(AnatomicalRegion region);

    // The bracing arms (the profile's mBracingArms): the hands reach for the
    // ground at a fall's first 0.2 s on a released root, and press on it
    // getting up on supported ground (still Fallen only with
    // OPENMW_RAGDOLL_FALLEN_BRACE: the press-up ran at full strength before
    // the get-up clip, threw the body up and back into Falling, 2026-09-24).
    struct BraceInput
    {
        bool mEnabled = false; // the profile braces, the body is powered, its pose captured, a pelvis
        std::optional<ActiveControllerMode> mMode;
        float mTimeInMode = 0.0f;
        bool mRootKinematic = false;
        bool mGroundContact = false;
        bool mFallenBrace = false;
        bool mRecoveryRequested = false;
        std::optional<JPH::Vec3> mCenterOfMassVelocity;
    };
    struct BraceDecision
    {
        bool mWanted = false;
        bool mProtectiveFall = false; // falling: the hands go along the fall
        JPH::Vec3 mFallDirection = JPH::Vec3::sZero(); // horizontal unit, or zero
    };
    BraceDecision decideBraceArms(const BraceInput& input);

    // A body's motor spring scale: an arm's own stiffness, the legs', the rest
    // the upper body's.
    float fallSpringScale(const FallProtectCommand& command, bool armPart, bool leftArm, bool leg);
}

#endif
