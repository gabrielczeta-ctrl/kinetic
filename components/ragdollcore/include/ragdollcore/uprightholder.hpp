#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_UPRIGHTHOLDER_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_UPRIGHTHOLDER_HPP

#include <ragdollcore/controller.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Vec3.h>

#include <optional>

namespace RagdollCore
{
    // KINETIC-2 phase 3, the upright holder (PLAN.md "KINETIC-2"; the
    // pipeline's Upright area, owner of the trunk's tilt). What holds the
    // trunk up on a powered body, in one place:
    // - the upright controller's input, assembled from the step's support,
    //   height and gait decisions (assembleUprightInput; the controller
    //   itself is calculateUprightControl in controller.hpp);
    // - the one rule for what of its output reaches the body
    //   (composeUprightControl);
    // - the tilt hold, which asks the soft root for its held strength when a
    //   body stands still tipped (TiltHold; rootstrength.hpp spends it).
    // Moved from the old integration behind OPENMW_RAGDOLL_K2_UPRIGHT; the
    // old path stays beside it until the user's verdict.
    //
    // Pure: no engine types.

    // The tilt hold (user bookmark 16, 2026-10-06: "stuck leaning"; DROP-01:
    // an armed body stopped from a run stood 17-20 deg off upright for 7+ s
    // while the unarmed one stood at 1). Standing still, the root's 0.15
    // budget cannot right a body a heavy weapon pulls forward. Slow (body and
    // request under 25 u/s), no hit being recovered from, and the torso past
    // mFrom degrees for mAfter seconds: the share grows toward the held
    // strength by mFull degrees (at least half), eased in and out over 0.4 s.
    // Engaged, it holds until the torso has been within mFrom - 3 degrees for
    // 0.5 s (letting go at mFrom the body sagged straight back: it swung 7-15
    // degrees). A hit lets go at once. OPENMW_RAGDOLL_PGC_TILT_HOLD =
    // "from, full, after" (from 0 = off).
    struct TiltHoldSettings
    {
        float mFrom = 7.0f; // degrees
        float mFull = 15.0f; // degrees
        float mAfter = 0.5f; // seconds
    };

    struct TiltHoldInput
    {
        bool mKnown = false; // the torso's tilt is measured this step
        float mTorsoUpDot = 1.0f;
        bool mHit = false; // a hit stun or a scoped balance episode runs
        float mBodySpeed = 0.0f; // horizontal, u/s
        float mRequestSpeed = 0.0f;
    };

    struct TiltHoldState
    {
        float mTiltTime = 0.0f; // seconds tipped past mFrom, not yet holding
        float mUprightTime = 0.0f; // seconds back within mFrom - 3, holding
        float mShare = 0.0f; // the eased share of the held strength
        bool mHolding = false;
    };

    // Steps the hold; returns its share (0-1).
    float updateTiltHold(TiltHoldState& state, const TiltHoldInput& input, const TiltHoldSettings& settings, float dt);

    // The torso's tilt from vertical, degrees.
    float torsoTiltDegrees(float torsoUpDot);

    // What the observation stage knows when it builds the upright
    // controller's input (the old updateControllerObservation's block).
    struct UprightAssembly
    {
        bool mRecovering = false; // getting up or blending back
        bool mGettingUp = false;
        bool mSoftRoot = false; // the soft root is enabled
        bool mRecoveryHeight = false; // a recovery reference height was projected
        bool mRecoveryHeightAssist = false; // the controller's recovery height assist
        bool mCorrectTranslation = false;
        bool mGroundContact = false;
        bool mHasSupport = false;
        // OPENMW_RAGDOLL_SOFT_ROOT_UNWEIGHT: the get-up's lift-only unweighting
        // also under the soft root (off: its spring carries the get-up).
        bool mSoftRootUnweight = false;
        float mPelvisUpDot = 1.0f;
        float mStandingMinimumUpDot = 0.0f; // profile

        float mTargetPelvisHeight = 0.0f;
        std::optional<float> mHandoverPelvisHeight; // the stop hand-over's blend, when active
        float mPelvisHeight = 0.0f; // measured over the support
        float mPelvisVerticalVelocity = 0.0f;
        // Selected by the gait policy (selectGaitCenterOfMassOffset); only
        // used with mCorrectTranslation and mHasSupport.
        JPH::Vec3 mCenterOfMassOffset = JPH::Vec3::sZero();
        JPH::Vec3 mCenterOfMassVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mTargetHorizontalVelocity = JPH::Vec3::sZero(); // the gait decision's

        JPH::Vec3 mPelvisUp = JPH::Vec3::sAxisZ();
        JPH::Vec3 mPelvisAngularVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mTorsoUp = JPH::Vec3::sAxisZ();
        JPH::Vec3 mTorsoAngularVelocity = JPH::Vec3::sZero();
        // The get-up follows the clip's pelvis up; under a held root
        // OPENMW_RAGDOLL_HELD_SPINE_FOLLOW aims at the clip's torso up. Unset
        // = world up.
        std::optional<JPH::Vec3> mGetUpTargetUp; // set only while getting up
        std::optional<JPH::Vec3> mHeldFollowTargetUp; // set only under a held root with the switch on

        float mTotalMass = 0.0f;
        float mSupportFriction = 0.0f;
        float mGravity = 0.0f;
        JPH::RVec3 mCenterOfMassPosition = JPH::RVec3::sZero();
        float mCenterOfMassHeight = 0.0f;
    };

    // The controller's input, before the gait policy's contact budget
    // (configureGaitSupportBudget) and the standing friction limit
    // (limitStandingFriction), which the caller applies in that order.
    UprightControllerInput assembleUprightInput(const UprightAssembly& assembly);

    // The standing friction limit: unlimited, the upright controller's
    // horizontal pull on the pelvis braked at up to ~1 g after a push and its
    // reaction dragged both planted feet through their grip (the standing
    // skate, PGC 2026-09-30). OPENMW_RAGDOLL_STAND_FRICTION = the limit as a
    // share of weight (0 = unlimited).
    void limitStandingFriction(UprightControllerInput& input, float standFriction, bool correctTranslation);

    // What of the controller's output reaches the body. Under the held root
    // on the animation-assisted route (`heldAssisted`) the pelvis parts are
    // the root's, so only the spine torque applies, scaled by
    // `heldSpineScale` (OPENMW_RAGDOLL_HELD_SPINE_UPRIGHT: toward world up it
    // pulled against the animation's lean every step, 2026-09-28). A pelvis
    // force has no reaction without reaction bodies: its horizontal and
    // vertical parts are dropped when the gait policy routes none.
    UprightControllerOutput composeUprightControl(const UprightControllerOutput& output, bool heldAssisted,
        float heldSpineScale, bool horizontalReactions, bool verticalReactions);
}

#endif
