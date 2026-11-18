#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_STRIDE_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_STRIDE_HPP

#include <ragdollcore/balancehelper.hpp>
#include <ragdollcore/footfall.hpp>
#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/recoverycontrol.hpp>
#include <ragdollcore/steparbiter.hpp>

#include <array>
#include <optional>
#include <vector>

namespace RagdollCore
{
    // KINETIC-2 phase 2, the legs (Stride; PLAN.md "KINETIC-2", MOVE-01
    // section 4): the stepping decisions the old integration made around
    // Footfall, in one module. Footfall (footfall.hpp) times and executes the
    // steps; Stride decides who takes each step and with what landing (the
    // balance catches in the arbiter's order, the step planner's aim), how
    // the planted legs push the body (the legs lead), and when the legs have
    // failed. Also the swing spring on every swing and the braced body's legs.
    //
    // Pure: the caller measures the body and the world, passes them in, and
    // applies what comes out (Footfall commands, forces, falls, logs).

    // ---- The catches: who takes this step -----------------------------------

    struct StrideCatchSettings
    {
        // OPENMW_RAGDOLL_CATCH_STEPS: the stage catch at all times.
        bool mCatchSteps = false;
        // OPENMW_RAGDOLL_CATCH_ON_HIT: the stage catch while a hit weakened
        // the balance (not while the request moves).
        bool mCatchOnHit = false;
        // OPENMW_RAGDOLL_PGC's B (standing catches) and H (the hover rules).
        bool mStandingCatch = false;
        bool mHover = false;
        // OPENMW_RAGDOLL_PGC_LANDING: a landing runs out its carried speed.
        bool mLanding = true;
        // OPENMW_RAGDOLL_PGC_LANDING_RELATIVE: the landing's run-out reads
        // the body's speed against the request while the player moves.
        bool mLandingRelative = false;
        // OPENMW_RAGDOLL_PGC_MOVING_CATCH: the moving catch's gain (0 = off).
        float mMovingGain = 0.0f;
        // OPENMW_RAGDOLL_PGC_CONTACT_SUPPORT: a foot touching supporting
        // ground stands whatever the probe says.
        bool mContactSupport = true;
        // OPENMW_RAGDOLL_PGC_REAL_COM: the situation's centre of mass, not
        // the pelvis + 15.
        bool mRealCom = false;
        // OPENMW_RAGDOLL_PGC_STUCK = "upDot, stepAfter, fallAfter" (upDot 0 = off).
        std::array<float, 3> mStuck{ 0.0f, 0.8f, 2.0f };
        // OPENMW_RAGDOLL_CATCH_VETO = "least up, most speed" (0 = off).
        std::array<float, 2> mVeto{ 0.93f, 40.0f };
        // OPENMW_RAGDOLL_PGC_RELEASE_AFTER.
        int mReleaseAfterSteps = 2;
        // OPENMW_RAGDOLL_PGC_STUMBLE's speed.
        float mStumbleSpeed = 20.0f;
        // OPENMW_RAGDOLL_STEP_ORDER.
        StepArbiter::Order mOrder = StepArbiter::defaultOrder();
    };

    // The catches' state between steps (the integration's members).
    struct StrideCatchState
    {
        CatchStepController mStanding;
        MovingCatchController mMoving;
        LandingController mLanding;
        BalanceStage mStage = BalanceStage::Ankle;
        float mStageTime = 0.0f;
        float mRecoveryTime = 0.0f;
        float mHeld = 0.0f;
        bool mRecovering = false;
        float mStuckLeanTime = 0.0f;
        bool mStuckStepped = false;
        // The landing's floor on the root's strength (rootstrength reads it).
        float mLandingRootFloor = 0.0f;
        // This step is a landing's touchdown, at this downward speed; the
        // landing catch consumes it.
        bool mLandingTouchdown = false;
        float mLandingImpactSpeed = 0.0f;
        // Stumble steps still asked for by the last blow (PGC_STUMBLE).
        int mStumbleLeft = 0;
        // Counters (diagnostics).
        unsigned mStepCount = 0;
        unsigned mStuckSteps = 0;
        unsigned mStuckFalls = 0;
        unsigned mMovingTrails = 0;
        unsigned mVetoes = 0;
        unsigned mReleases = 0;
        unsigned mStumbleSteps = 0;
    };

    struct StrideCatchInput
    {
        // Footfall's input this step (eligibility, the ground probe, the
        // legs, the frame, the command velocity, the yaw).
        const FootfallInput* mFootfall = nullptr;
        // Footfall's feet and the last step's pins.
        std::array<FootfallController::Foot, 2> mFeet;
        std::array<bool, 2> mPinned{ false, false };
        // The body: the pelvis and the feet in the world; the body's centre
        // of mass velocity and the torso's up (the controller's input; unset
        // when it has none). mPelvisKnown: the rig has a pelvis body.
        bool mPelvisKnown = false;
        std::optional<JPH::RVec3> mPelvis;
        std::array<std::optional<JPH::RVec3>, 2> mFootWorld;
        bool mHaveBody = false;
        JPH::Vec3 mBodyVelocity = JPH::Vec3::sZero();
        float mTorsoUpDot = 1.0f;
        // The contact estimate per foot (unset = none).
        std::optional<std::array<bool, 2>> mFootContact;
        // The situation's centre of mass (PGC_REAL_COM), when known.
        std::optional<JPH::RVec3> mSituationCom;
        // The movement request: its speed (the smoothed target) and the raw
        // request (world, the keys' direction).
        float mRequestSpeed = 0.0f;
        JPH::Vec3 mRawRequest = JPH::Vec3::sZero();
        // A hit weakened the balance: the soft root under full strength, a
        // hit stun, the bearings' stumble.
        bool mHitWeakened = false;
        // The weapon's footwork asks for steps (always the stage catch).
        bool mSwingFootwork = false;
        // The weapon arm is driven (the tip controller).
        bool mArmDriven = false;
        // What a catch step can do (stats, fatigue, load).
        CatchCapability mCapability;
    };

    // A fall the catches decided (the caller releases the body).
    struct StrideFall
    {
        const char* mCause = "";
        BalanceVec2 mDirection;
    };

    // One PGC trace line's values (OPENMW_RAGDOLL_PGC_TRACE).
    struct StrideCatchTrace
    {
        const char* mBranch = "";
        BalanceState mState;
        std::array<BalanceVec2, 2> mFeet;
        std::array<bool, 2> mSupported{ false, false };
        int mStage = -1;
        int mSteps = -1;
        bool mSwingNow = false;
        bool mRelease = false;
    };

    struct StrideCatchResult
    {
        // The command for Footfall this step (unset: Footfall's own).
        std::optional<FootfallCommand> mCommand;
        // Who took the step (unset: nobody, Footfall's gait).
        std::optional<StepSource> mSource;
        std::optional<StrideFall> mFall;
        // The branches that ran this step, in order (PGC_TRACE).
        std::vector<StrideCatchTrace> mTraces;
        // Log lines: a stuck step (its foot), a vetoed release.
        std::optional<std::size_t> mStuckStepFoot;
        bool mVetoed = false;
        float mVetoSpeed = 0.0f;
        // The stage catch's step this time (RAGDOLL_CATCH, dev builds).
        std::optional<BalanceVec2> mStageCapture;
        BalanceVec2 mStageVelocity;
    };

    // The balance catches (stuck, landing, moving, standing, stage) in the
    // settings' order; the first that takes the step owns it. Runs after the
    // stagger's give-up, the stagger plan and the dodge (other areas), on a
    // step none of them took.
    StrideCatchResult stepStrideCatches(
        const StrideCatchInput& input, const StrideCatchSettings& settings, StrideCatchState& state);

    // ---- The step planner's aim (MOVE-01 S3, OPENMW_RAGDOLL_STRIDE_PLANNER) --

    struct StrideAimInput
    {
        // "on, reach u, minimum width u" (on 0 = off).
        std::array<float, 3> mSettings{ 0.0f, 70.0f, 4.0f };
        bool mEligible = false;
        bool mStepOwned = false; // a balance behaviour owns this step
        float mYaw = 0.0f;
        JPH::Vec3 mCommandVelocity = JPH::Vec3::sZero();
        std::optional<RagdollMomentum> mMomentum;
        std::array<FootfallController::Foot, 2> mFeet;
        std::array<std::optional<JPH::RVec3>, 2> mFootWorld;
    };

    // The landing of each swing in its first half, from the whole body's
    // momentum about the physical stance foot; unset = no aim this step.
    std::optional<FootfallCommand> aimStrideLandings(const StrideAimInput& input);

    // ---- The legs lead (OPENMW_RAGDOLL_LEGS_LEAD, _LEGS_LEAD_GRIP) -----------

    struct StrideLeadSettings
    {
        // "on, root share, friction share, damping".
        std::array<float, 4> mLead{ 0.0f, 0.1f, 0.35f, 6.0f };
        // "margin, cruise, speed" (margin 0 = the weight cap only).
        std::array<float, 3> mGrip{ 0.0f, 0.25f, 150.0f };
        // K2_STRIDE_GRIP: the planted feet's grip only.
        bool mPlantedGrip = false;
        // The dodge's own seconds (DODGE's fifth value).
        float mDodgeTime = 0.2f;
    };

    struct StrideLeadContact
    {
        std::size_t mSide = 0;
        float mNormalForce = 0.0f; // vertical, force units
        std::optional<float> mFriction;
    };

    struct StrideLeadInput
    {
        bool mEligible = false;
        // The gate: standing on the ground, root held at full strength, no
        // stagger, stun or jump, flat ground (no slope, stair or climb).
        bool mGateOpen = false;
        // A dodge is running: its burst rides the same push.
        bool mDodging = false;
        JPH::Vec3 mDodgeDirection = JPH::Vec3::sZero();
        float mDodgeLength = 0.0f;
        JPH::Vec3 mRequest = JPH::Vec3::sZero(); // the raw movement request
        JPH::Vec3 mBodyVelocity = JPH::Vec3::sZero();
        BalanceVec2 mCenterOfMass; // the situation's
        // Each foot planted (supported and pinned) and where.
        std::array<std::optional<BalanceVec2>, 2> mPlantedFeet;
        std::vector<StrideLeadContact> mContacts; // the feet's ground contacts this step
        float mTotalMass = 0.0f;
        bool mTelemetry = false; // the masses were read
    };

    struct StrideLeadState
    {
        bool mLeading = false;
        bool mCruising = false;
        unsigned mSteps = 0;
    };

    struct StrideLeadResult
    {
        // The legs lead this step (the capsule follows the body).
        bool mLeading = false;
        // mLeading changed this step (RAGDOLL_LEGS_LEAD on=).
        bool mChanged = false;
        // The push to apply: on the upper body by mass, the reaction on the
        // planted feet.
        std::optional<StanceForceOutput> mForce;
    };

    StrideLeadResult stepStrideLead(
        const StrideLeadInput& input, const StrideLeadSettings& settings, StrideLeadState& state);

    // ---- The swing spring (OPENMW_RAGDOLL_PGC's D; every swing in play) -----

    struct StrideSpringFoot
    {
        bool mSwinging = false;
        // The rig's foot, knee and hip bodies were read (masses, motion).
        bool mValid = false;
        BalanceVec2 mFoot; // the foot's centre of mass
        BalanceVec2 mFootVelocity;
        BalanceVec2 mLiftOff; // Footfall's pin
        BalanceVec2 mLanding; // Footfall's target
        float mPhase = 0.0f;
        float mDuration = 0.3f;
        float mLegMass = 0.0f; // thigh + calf + foot
        float mFootMass = 0.0f;
        // Footfall's output this step: a swing began, the leg motors' foot
        // goal and its velocity.
        bool mSwingStarted = false;
        std::optional<JPH::RVec3> mGoal;
        JPH::Vec3 mGoalVelocity = JPH::Vec3::sZero();
    };

    struct StrideSpringInput
    {
        std::array<StrideSpringFoot, 2> mFeet;
        float mFrequency = 7.0f; // the catch capability's swing spring, Hz
        // A scoped recovery episode runs, with this step's length.
        bool mScoped = false;
        float mScopedDt = 0.0f;
        std::optional<float> mPelvisMass;
    };

    // Each swinging foot pulled along its swing, the opposite force on the
    // pelvis (an internal pair). In a scoped recovery the spring follows the
    // leg motors' foot goal (its velocity filtered over 0.03 s, at most
    // 400 u/s, zero as a swing starts) and is implicit over the step.
    // `goalVelocity`: the filtered goal velocities between steps.
    std::array<std::optional<SwingSpringOutput>, 2> stepStrideSwingSpring(
        const StrideSpringInput& input, std::array<JPH::Vec3, 2>& goalVelocity);

    // ---- The braced body's legs (OPENMW_RAGDOLL_PGC_BRACE) -------------------

    struct StrideBraceSettings
    {
        // OPENMW_RAGDOLL_PGC_BRACE_NO_ANCHOR: no anchor settle while braced.
        bool mNoAnchor = true;
        int mReleaseAfterSteps = 2; // PGC_RELEASE_AFTER
        float mStumbleSpeed = 20.0f; // PGC_STUMBLE's speed
    };

    struct StrideBraceState
    {
        PostureController mPosture;
        CatchStepController mCatch;
        unsigned mReleases = 0;
        // Stumble steps still asked for by the last blow (shared with the
        // standing catch, StrideCatchState; counted down as swings start).
        int mStumbleLeft = 0;
        unsigned mStumbleSteps = 0;
    };

    struct StrideBraceInput
    {
        // Footfall's input this step (the ground probe, the legs, the frame,
        // the yaw); its command velocity is zero while braced.
        const FootfallInput* mFootfall = nullptr;
        std::array<FootfallController::Foot, 2> mFeet;
        float mDt = 0.0f;
        JPH::Vec3 mTorsoUp = JPH::Vec3::sAxisZ(); // world
        std::array<BalanceVec2, 2> mFeetCom; // each foot's centre of mass
        std::array<bool, 2> mSupported{ false, false }; // the contact estimate
        JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero(); // the whole body's
        JPH::Vec3 mBodyVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mRawRequest = JPH::Vec3::sZero(); // the keys steer the catch
        CatchCapability mCapability;
        bool mArmDriven = false;
    };

    struct StrideBraceResult
    {
        // The command for Footfall (unset: the body falls this step, no
        // Footfall step).
        std::optional<FootfallCommand> mCommand;
        std::optional<StrideFall> mFall;
        StrideCatchTrace mTrace;
        // The body as the catch saw it, for the stance push after Footfall's
        // step (strideBraceStance).
        BalanceState mBody;
    };

    // Before Footfall's step: the stance legs hold the pelvis leaned by the
    // posture loop so the torso stands up; stat-scaled catch steps; a push
    // nothing can catch falls at once.
    StrideBraceResult stepStrideBrace(
        const StrideBraceInput& input, const StrideBraceSettings& settings, StrideBraceState& state);

    // After Footfall's step: the upper body toward the middle of the planted
    // (supported, not swinging) feet, braked, inside the friction cone.
    std::optional<StanceForceOutput> strideBraceStance(const BalanceState& body,
        const std::array<BalanceVec2, 2>& feet, const std::array<bool, 2>& supported,
        const std::array<FootfallController::Foot, 2>& feetAfterStep, float totalMass);

    // ---- The legs' health (OPENMW_RAGDOLL_LEGS_HEALTH) -----------------------

    struct StrideHealthState
    {
        bool mFailing = false;
        float mStillTime = 0.0f;
        unsigned mStillSteps = 0;
    };

    struct StrideHealthInput
    {
        float mDt = 0.0f;
        float mRequestSpeed = 0.0f;
        float mBodySpeed = 0.0f;
        bool mDisturbed = false; // a hit stun or a recovery episode
        std::array<bool, 2> mSwingStarted{ false, false };
    };

    // Standing still for `grace` s, `steps` more steps mean the legs fail
    // ("grace, steps"; grace 0 = off). True on the step they are found failing.
    bool stepStrideHealth(
        const StrideHealthInput& input, const std::array<float, 2>& settings, StrideHealthState& state);
}

#endif
