#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_DROPROLL_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_DROPROLL_HPP

#include <ragdollcore/rootstrength.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Vec3.h>

#include <optional>

namespace RagdollCore
{
    // KINETIC-2 phase 3, the reaction path's drop roll (PGC-03 C8b, DROP-01,
    // ACRO-01): a hard landing taken as a forward roll over the shoulder. The
    // integration arms it (its lane and wall probes query the world) and
    // plants the hands; this module runs the roll once it has begun - the
    // measured roll angle, the target the root turns the body to, the tuck
    // and posture fades, the phase transitions and the root's pose - behind
    // OPENMW_RAGDOLL_K2_REACTION, the old path beside it until the user's
    // verdict. The root's strength, horizontal hold and support during the
    // roll are rootstrength's (RootRollPhase).
    //
    // Pure: no engine types.

    struct DropRollSettings
    {
        float mRadius = 24.0f; // the rolling ball's radius (the target's rate)
        float mLeadDeg = 50.0f; // the target at most this far ahead
        float mTimeout = 1.4f; // seconds
        // A twist: the torso's up this far along the roll axis for 0.25 s ends
        // the roll (0.7 for 0.15 s aborted 1 in 3 rolls that would have stood).
        float mTwist = 0.85f;
    };

    struct DropRollState
    {
        RootRollPhase mPhase = RootRollPhase::None;
        float mPhaseTime = 0.0f;
        float mTotal = 0.0f;
        float mTheta = 0.0f; // the measured roll angle, unwrapped (rad)
        float mThetaRawLast = 0.0f;
        float mRate = 0.0f; // filtered, rad/s
        float mTarget = 0.0f; // the root's roll angle (rad)
        float mTwistTime = 0.0f;
        float mStallTime = 0.0f;
        float mTuck = 0.0f;
        float mPosture = 0.0f;
        bool mHandsReleased = false;
    };

    struct DropRollInput
    {
        float mDt = 0.0f;
        JPH::Vec3 mAxis = JPH::Vec3::sAxisX(); // the roll axis (world)
        JPH::Vec3 mPelvisUp = JPH::Vec3::sAxisZ();
        JPH::Vec3 mTorsoUp = JPH::Vec3::sAxisZ();
        float mAlong = 0.0f; // the body's speed along the roll's travel, >= 0
        float mImpact = 0.0f; // the touchdown's vertical speed
        bool mToBack = false; // the plan ends on the back
        // ACRO-01: the turn rate's cap by skill (8 rad/s without the skill tier).
        float mRateCap = 8.0f;
        bool mFootSupported = false;
    };

    struct DropRollStep
    {
        bool mReleaseHands = false; // the planted hands let go (past 70 deg)
        bool mPlantHands = false; // the hands go to the ground ahead
        bool mToRoll = false;
        bool mToRise = false; // the caller records the pelvis height
        // Ended this step: the result ("back", "stalled", "twist", "timeout",
        // "slow", "stand", "rise") and whether the body falls.
        const char* mEnd = nullptr;
        bool mFall = false;
    };

    // A body vector's angle about the roll axis from straight up (positive
    // along the travel); atan2, so a twist off the axis does not drift it.
    float dropRollTheta(JPH::Vec3Arg axis, JPH::Vec3Arg v);

    // One step of a begun roll (Absorb, Roll or Rise), after the caller
    // advanced mPhaseTime and mTotal and found the root held, no hit and the
    // pose readable.
    DropRollStep stepDropRoll(DropRollState& state, const DropRollInput& input, const DropRollSettings& settings);

    // The root during the roll: pitched about the axis to the target from
    // the yaw at the touchdown (`base`), lowered on the absorb by the impact,
    // at the pelvis's own height rolling (the ground carries the body), eased
    // back up to the standing height on the rise; with the ACRO-01 skill tier
    // the run carried through by skill (`keep` x the entry speed).
    struct DropRollRootInput
    {
        RootRollPhase mPhase = RootRollPhase::None;
        float mPhaseTime = 0.0f;
        float mTarget = 0.0f;
        JPH::Vec3 mAxis = JPH::Vec3::sAxisX();
        JPH::Quat mBase = JPH::Quat::sIdentity();
        float mImpact = 0.0f;
        std::optional<JPH::Real> mPelvisZ;
        float mRiseZ = 0.0f;
        // ACRO-01: the run's share kept (keep > 0), the entry speed, the
        // body's speed along the travel and the travel.
        float mKeep = 0.0f;
        float mEntrySpeed = 0.0f;
        float mAlong = 0.0f;
        JPH::Vec3 mTravel = JPH::Vec3::sAxisY();
    };
    void dropRollRootPose(const DropRollRootInput& input, JPH::RVec3& position, JPH::Quat& rotation);
}

#endif
