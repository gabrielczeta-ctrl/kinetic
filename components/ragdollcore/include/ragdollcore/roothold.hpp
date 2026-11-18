#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_ROOTHOLD_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_ROOTHOLD_HPP

#include <ragdollcore/controller.hpp>

#include <array>
#include <optional>

namespace RagdollCore
{
    // KINETIC-2 phase 3, the reaction path's hold rules (PLAN.md
    // "KINETIC-2"): when the soft root lets a body go and when it takes it
    // back. A body tipped past the line falls (the topple); a toppled body
    // that stands upright again is taken back (the rehold); a body physics
    // owned is retaken once it stands settled, braces upright, or gets up.
    // Moved from the old integration's updateKinematicRoot behind
    // OPENMW_RAGDOLL_K2_REACTION; the old path stays beside it until the
    // user's verdict.
    //
    // Pure: no engine types.

    struct RootHoldSettings
    {
        // An upright, slow Bracing body takes the root back after this many
        // seconds, and a topple rehold at once (user bookmark 2026-10-31,
        // "NPC stuck in recovery": braced in place for 17 s).
        // OPENMW_RAGDOLL_BRACE_REACQUIRE (0 = off, the rehold too).
        float mBraceReacquire = 1.0f;
        // A get-up ends the hard landing that released the root (user
        // bookmark 12, 2026-10-01). OPENMW_RAGDOLL_GETUP_ENDS_LANDING.
        bool mGetUpEndsLanding = true;
        // The torso's up . world up under which a held body tips (the hit
        // settings' topple line).
        float mToppleUpDot = 0.5f;
        // The tilt counts only while the torso still tips further (ToppleJudge
        // mRising, PGC offline: falls 93 -> 44 of 224 near the catch limit).
        // OPENMW_RAGDOLL_TOPPLE_RISING.
        bool mToppleRising = true;
        // Upright past [0] for [1] seconds, a toppled body that stands is
        // taken back (play test 2026-09-28, "stuck in smooth criminal lean").
        // OPENMW_RAGDOLL_TOPPLE_REHOLD = "upDot,seconds" (seconds 0 = off).
        std::array<float, 2> mRehold{ 0.7f, 0.5f };
    };

    struct RootHoldState
    {
        float mReacquireTimer = 0.0f; // standing slow
        float mBracedTime = 0.0f; // bracing upright and slow
        bool mReheld = false; // a topple rehold asks for the root
        bool mLandingReleased = false; // a hard landing released the root
        bool mPhysicsOwned = false; // physics owns the body (the root released)
        float mToppleTime = 0.0f;
        float mToppleLastUp = 1.0f;
        float mTippedFor = 0.0f;
        bool mToppled = false;
        float mToppledStandTime = 0.0f;
        float mFallDecidedTime = 1000.0f; // seconds since a fall was decided
    };

    struct RootHoldInput
    {
        std::optional<ActiveControllerMode> mMode; // unset: no controller output
        bool mKnown = false; // the controller input (tilt, speed) is measured
        float mTorsoUpDot = 1.0f;
        float mCenterOfMassSpeed = 0.0f; // 3D, u/s
        bool mRootKinematic = false;
        bool mRollOwns = false; // the drop roll owns the body
    };

    // The retake, judged before the strength.
    struct RootRetake
    {
        bool mDriven = false; // a mode the held root drives
        bool mRecoveryOwns = false; // getting up or blending back
        bool mReacquired = false;
        // Retaken after physics owned it by a brace or a rehold (logged
        // RAGDOLL_BRACE_REACQUIRE); mByRehold says which.
        bool mBraceOrRehold = false;
        bool mByRehold = false;
    };
    RootRetake judgeRootRetake(const RootHoldInput& input, const RootHoldSettings& settings, RootHoldState& state, float dt);

    // The topple and the rehold, judged on a held soft root after the
    // strength. A rehold also restarts the strength's ramp (the caller's
    // RootStrengthState::mReholdElapsed = 0).
    struct ToppleEvents
    {
        bool mToppled = false; // toppled this step (RAGDOLL_TOPPLE)
        bool mReheld = false; // taken back this step (RAGDOLL_TOPPLE_REHOLD)
    };
    ToppleEvents judgeTopple(const RootHoldInput& input, bool recoveryOwns, const RootHoldSettings& settings,
        RootHoldState& state, float dt);

    // Whether the held root owns the body this step.
    bool rootHeld(const RootRetake& retake, const RootHoldState& state, bool routeMatches, bool powered,
        bool interrupted);
}

#endif
