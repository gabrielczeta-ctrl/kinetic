#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_RECOVERYCONTROL_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_RECOVERYCONTROL_H

#include "balancehelper.hpp"

#include <array>
#include <optional>

namespace RagdollCore
{
    // Recovery control beside Footfall: the pieces the offline push tables
    // kept (design-pgc-balance-helper-2026-09-28.md in the dev root). Pure
    // functions on the ground plane, no Jolt types; the caller measures the
    // body, calls these each physics step and applies the outputs as forces
    // and soft-root settings. Units are the caller's (game units, seconds).

    // --- Stats -> what a catch step can do ---------------------------------

    struct RecoveryStats
    {
        float mAgility = 50.0f; // 0-100
        float mAcrobatics = 50.0f; // 0-100
    };

    struct CatchCapability
    {
        float mReachFraction = 0.9f; // of the leg length
        float mMinimumSwingTime = 0.225f;
        float mMaximumSwingTime = 0.4f;
        float mSwingFootSpeed = 450.0f;
        float mReactionDelay = 0.09f; // stage Step held this long before the first catch step
        float mSwingSpringFrequency = 7.0f; // Hz
        float mCrossoverClearance = 7.0f; // 0 = no crossover steps
    };

    // Skill s = (0.6 Agility + 0.4 Acrobatics) / 100, clamped to 0-1:
    // reach 0.7-1.1 leg, shortest swing 0.30-0.15 s (longest 0.45-0.35),
    // swing foot speed 300-600 u/s, reaction delay 0.15-0.03 s, swing
    // spring 4-10 Hz. Crossover steps at every skill (clearance 7): they
    // were the largest single gain measured. Measured offline with the
    // Agility mapping alone (s = Agility/100); the Acrobatics weight is a
    // proposal.
    CatchCapability catchCapability(const RecoveryStats& stats);

    // The planner limits for a catch step from the capability.
    CaptureStepLimits catchStepLimits(const CatchCapability& capability, float legLength);

    // --- Stance force: the planted legs push the body ---------------------

    struct StanceForceSettings
    {
        float mStiffness = 20.0f; // 1/s^2, toward the middle of the planted feet (standing)
        float mDamping = 8.0f; // 1/s, toward the request velocity
        float mFrictionShare = 0.2f; // cap: this share of body weight (inside the friction cone)
        float mMovingSpeed = 15.0f; // request faster than this: track its velocity, no position term
    };

    struct StanceForceInput
    {
        BalanceVec2 mCenterOfMass;
        BalanceVec2 mVelocity; // center of mass velocity
        BalanceVec2 mRequestVelocity; // the movement request (zero standing)
        std::array<std::optional<BalanceVec2>, 2> mPlantedFeet; // left, right; empty when not planted
        // Weight shift (WASD while bracing): standing, the target is the
        // middle of the planted feet moved by this (world x, y).
        BalanceVec2 mCenterShift;
        float mTotalMass = 0.0f;
        float mGravity = 0.0f;
        StanceForceSettings mSettings;
    };

    struct StanceForceOutput
    {
        bool mActive = false;
        // Total horizontal force on the upper body: spread it over the
        // non-leg bodies by mass so it translates, not tips.
        BalanceVec2 mBodyForce;
        // The reaction on each planted foot (sums with mBodyForce to zero:
        // an internal pair, no outside help).
        std::array<BalanceVec2, 2> mFootForce;
        bool mCapped = false;
    };

    std::optional<StanceForceOutput> computeStanceForce(const StanceForceInput& input);

    // --- Swing-foot spring --------------------------------------------------

    struct SwingSpringSettings
    {
        float mFrequency = 6.0f; // Hz
        float mDampingRatio = 1.0f;
        float mCapLegWeights = 3.0f; // cap: this many swing-leg weights
    };

    struct SwingSpringInput
    {
        // Scoped recovery: the goal and its velocity taken from the leg
        // motors' foot goal (Footfall's mMotorFootGoal) instead of the straight
        // ease from lift-off to landing. Both or neither.
        std::optional<BalanceVec2> mGoal, mGoalVelocity;
        BalanceVec2 mFoot; // swing foot center of mass
        BalanceVec2 mFootVelocity;
        BalanceVec2 mLiftOff; // where the swing started (Footfall's pin)
        BalanceVec2 mLanding; // the swing's landing (Footfall's target)
        float mPhase = 0.0f; // 0-1
        float mDuration = 0.3f; // s
        // Scoped recovery: a backward-Euler spring over this step (stable at a
        // high gain) with the foot/pelvis pair's effective mass; 0 = explicit.
        float mImplicitDt = 0.0f, mEffectiveMass = 0.0f;
        float mLegMass = 0.0f; // thigh + calf + foot
        float mGravity = 0.0f;
        SwingSpringSettings mSettings;
    };

    struct SwingSpringOutput
    {
        BalanceVec2 mFootForce; // on the swing foot
        BalanceVec2 mPelvisForce; // on the pelvis (= -mFootForce)
        bool mCapped = false;
    };

    // Pulls the foot along a straight path from lift-off to the landing
    // (quintic ease on the phase), horizontal only; height stays with the
    // leg drive.
    std::optional<SwingSpringOutput> computeSwingSpring(const SwingSpringInput& input);

    // --- Catch steps: when to step, with which foot, where ------------------

    struct CatchFootState
    {
        BalanceVec2 mPosition; // foot center of mass
        bool mSupported = false; // on the ground
        bool mSwinging = false; // Footfall swings it
        bool mTargetHeld = false; // its landing was set by a catch command
        float mPhase = 0.0f; // Footfall swing phase 0-1
        float mDuration = 0.3f; // Footfall swing duration, s
    };

    struct CatchStepInput
    {
        BalanceState mState; // center of mass, its velocity, height, gravity
        BalanceVec2 mBodyForward{ 0.0f, 1.0f };
        BalanceVec2 mReferenceVelocity; // the capsule velocity (caught = moving with it)
        std::array<CatchFootState, 2> mFeet; // left, right
        float mDt = 0.0f;
        CaptureStepLimits mLimits; // catchStepLimits(catchCapability(stats), leg length)
        float mReactionDelay = 0.0f; // CatchCapability::mReactionDelay
        float mMaximumRecoveryTime = 3.0f;
        bool mUnloadedFootOnly = false; // measured: no gain, off
        // Hover decision (the body neither stands nor falls): all off by default.
        // Release at once when the stage is Step and no plan of the actor's
        // capability captures (mRelease), instead of leaning on the root.
        bool mReleaseWhenUncatchable = false;
        // ... but only after this many steps of this catch have been taken
        // (best effort when none captures): the planner is pessimistic just
        // after a stop (35/120 pushes released offline that, unreleased, all
        // were caught, 09-30). 0 = at once.
        int mReleaseAfterSteps = 0;
        // Crossover steps only while the body moves faster than this (u/s,
        // relative to the reference; 0 = always): a slow body stepped across
        // its own stance foot, then Footfall stepped it back, again and again
        // (PGC viewer, agility 90, 09-30).
        float mCrossoverSpeed = 0.0f;
        // The weapon arm is driven (drag, strike, guard, tip controller
        // active): crossovers stay allowed at any speed. The gate alone broke
        // FIX-08 guided warhammer strikes in game (bisect 10-01: 0/2).
        bool mArmDriven = false;
        // A catch that has started no swing for this long decides: a
        // capturing step if one exists (either foot, even from one support),
        // else release. 0 = off.
        float mMaximumHoverTime = 0.0f;
        // One foot down and the other hanging free (not swinging, not on the
        // ground - a step that landed out of reach): step the free foot now.
        bool mStepFreeFoot = false;
        // Steering (WASD in world x, y; zero = none): among plans that catch
        // in the same number of steps, take the step going most along it.
        BalanceVec2 mDirectionBias;
        // Deliberate steps (WASD while bracing): with mDirectionBias held and
        // no catch running, the foot trailing along it steps a stride of
        // mDeliberateStride along it (keeping its own side), one at a time.
        bool mDeliberateSteps = false;
        float mDeliberateStride = 20.0f;
        float mDeliberateInterval = 0.0f; // s between deliberate steps (landing to lift-off)
        // Stumble (10-09, PGC robustness: "the staggers slide ... I wish it
        // would do more stepping"): a blow that moved the body asks for this
        // many more steps (the adapter counts them down as swings start).
        // While any remain and the body moves faster than mStumbleSpeed
        // against the reference, the catch runs and, with no capturing plan
        // (the capture point still inside the feet - the case where planted
        // feet were dragged along), steps a foot along the motion: the
        // trailing foot for a push from the front or back, the leading foot
        // (a side step) for a push from the side. Never a release. 0 = off.
        int mStumbleSteps = 0;
        float mStumbleSpeed = 20.0f;
        // The stumble step lands this many time constants of the motion ahead
        // of the center of mass (plus mStumbleExtra units), within reach.
        float mStumbleLead = 1.0f;
        float mStumbleExtra = 6.0f;
    };

    struct CatchStepCommand
    {
        BalanceStage mStage = BalanceStage::Ankle;
        float mHipWeight = 0.0f;
        BalanceVec2 mFallDirection;
        // A catch is running: hold the Footfall clock (FootfallCommand::mHoldClock).
        bool mHoldClock = false;
        // Start a swing of this foot now, lasting mSwingTime, landing at mLanding.
        std::optional<std::size_t> mSwingNow;
        float mSwingTime = 0.3f;
        // Landing (x, y) for a swing started now or re-aimed in flight.
        std::array<std::optional<BalanceVec2>, 2> mLanding;
        // Hover decision: no catch step exists; release the body to a fall now.
        bool mRelease = false;
        // How long the current catch has gone without a swing (the hover), s.
        float mHoverTime = 0.0f;
        // The swing started now is a stumble step (mStumbleSteps), not a
        // capturing plan's step.
        bool mStumble = false;
    };

    // Stateful: the balance stage with its hysteresis, the reaction delay,
    // and one catch at a time. A catch starts once the stage has been Step for
    // the reaction delay; with both feet down it swings the foot whose plan
    // (planRecoverySteps, 3 steps, crossovers allowed by the limits) catches
    // best; a swing in flight is re-aimed once. It ends when the capture point
    // is back inside the supported feet, the body moves with the capsule
    // (evaluateRecoveryEnd: margin 6, speed 25, held 0.2 s) and no foot swings,
    // or after mMaximumRecoveryTime.
    class CatchStepController
    {
    public:
        std::optional<CatchStepCommand> update(const CatchStepInput& input);
        void reset() { *this = CatchStepController(); }
        bool catching() const { return mCatching; }
        std::size_t stepsStarted() const { return mStepsStarted; }

    private:
        BalanceStage mStage = BalanceStage::Ankle;
        float mStageTime = 0.0f;
        float mStepStageTime = 0.0f;
        bool mCatching = false;
        float mCatchTime = 0.0f;
        float mCaughtHeld = 0.0f;
        float mHoverTime = 0.0f;
        int mCatchSteps = 0;
        float mSinceDeliberate = 1e9f;
        float mStumbleWait = 0.0f;
        std::size_t mStepsStarted = 0;
    };

    // --- Moving catch: catch steps while walking or running -----------------

    struct MovingFootState
    {
        BalanceVec2 mPosition; // foot center of mass
        bool mSupported = false; // on the ground
        bool mPinned = false; // Footfall holds it planted
        bool mSwinging = false; // Footfall swings it
        BalanceVec2 mTarget; // Footfall's landing for a swing in flight
        float mPhase = 0.0f;
        float mDuration = 0.3f;
    };

    struct MovingCatchSettings
    {
        // The landing moves by gain x Tc x (body velocity - commanded velocity),
        // Tc = sqrt(height / g): the step that turns the velocity error
        // around (foot placement on the capture point in the gait's frame).
        float mGain = 0.5f; // 0.5 kept: 1.0 fell a 2000 side push running back (09-30)
        // Corrections start above this velocity error (u/s): smaller ones
        // stay with Footfall's own gait.
        float mErrorThreshold = 40.0f;
        // Lagging behind the command (accelerating, or slowing on a slope) is
        // the gait's business; only the overspeed along the command and the
        // sideways error count, unless this is set.
        bool mCorrectLag = false;
        // One landing moves at most this far (units); a bigger error takes
        // more steps, or the release.
        float mMaximumCorrection = 30.0f;
        // A moved landing stays on its own side of the stance foot by this
        // much (units, across the commanded direction): no tangled legs.
        float mMinimumWidth = 6.0f;
        // A planted foot left this far behind the center of mass along the
        // travel (a share of the reach) steps now, even with no velocity
        // error: the game's backward run sat on a stance foot 60 units behind
        // for 1.3 s while the other foot kept re-stepping (09-30 bookmark 1).
        float mTrailLimit = 0.6f;
        // ... after it has trailed this long (s).
        float mTrailTime = 0.05f;
        // A foot neither pinned nor swinging (left hanging, as the backward
        // run stalls) for this long is swung now.
        float mFreeFootDelay = 0.05f;
        // The body falls when the landing the error needs stays beyond the
        // reach for this long.
        float mReleaseTime = 0.25f;
        bool mRelease = true;
    };

    struct MovingCatchInput
    {
        BalanceState mState; // center of mass, its velocity, height, gravity
        BalanceVec2 mCommandedVelocity; // the gait's commanded velocity
        std::array<MovingFootState, 2> mFeet;
        float mDt = 0.0f;
        float mReach = 60.0f; // landing within this of the center of mass (capability x leg)
        float mMinimumSwingTime = 0.2f;
        MovingCatchSettings mSettings;
    };

    struct MovingCatchCommand
    {
        // Landing overrides (FootfallCommand::mLanding) for a swing in flight or
        // one started now; empty = Footfall's own.
        std::array<std::optional<BalanceVec2>, 2> mLanding;
        std::optional<std::size_t> mSwingNow;
        float mSwingTime = 0.25f;
        bool mRelease = false;
        // Why: a hanging foot (free) or a planted foot left behind (trail).
        bool mTrail = false;
        // The landing correction the velocity error asks for (length, u).
        float mCorrection = 0.0f;
    };

    class MovingCatchController
    {
    public:
        std::optional<MovingCatchCommand> update(const MovingCatchInput& input);
        void reset() { *this = MovingCatchController(); }

    private:
        std::array<float, 2> mFreeTime{ 0.0f, 0.0f };
        std::array<float, 2> mTrailTime{ 0.0f, 0.0f };
        float mBeyondReach = 0.0f;
        // Footfall's own landing at the start of each swing: the correction
        // moves this one, not the already-moved target fed back.
        std::array<std::optional<BalanceVec2>, 2> mSwingBase;
    };

    // --- Posture: the stance hips stand the torso up --------------------------

    // The tilt that would stand a body part up: horizontal rotation axis times
    // angle (rad), from its up vector (world, z up). Zero when upright.
    BalanceVec2 uprightTilt(float upX, float upY, float upZ);

    struct PostureSettings
    {
        float mProportional = 0.5f; // lean per radian of torso tilt
        float mIntegral = 1.0f; // lean per radian-second
        float mMaximumDegrees = 25.0f;
    };

    // PI on the torso tilt. Output: the pelvis lean for
    // FootfallCommand::mPelvisLean (world axis x, y times radians; z = 0), used
    // with mStanceHoldsPelvis so the planted legs lean the pelvis and the
    // torso above it stands up. Measured offline: level-3 rest torso 11.7 deg
    // -> 0.8 deg (0.5, 1, 25).
    class PostureController
    {
    public:
        BalanceVec2 update(BalanceVec2 torsoTilt, float dt, const PostureSettings& settings = {});
        void reset() { mIntegral = {}; }

    private:
        BalanceVec2 mIntegral;
    };

    // --- Root budget ----------------------------------------------------------

    // --- Landing: absorb the touchdown, run out the carried speed -----------

    struct LandingSettings
    {
        // Absorb: the soft root strength is at least mAbsorbStrength at
        // touchdown, easing to none over mAbsorbTime, scaled by the impact
        // (full at mFullImpact u/s downward). Offline: a 1.2 m drop on the
        // PGC budget collapsed the legs 4/4 standing; 1.0 over 0.3 s held all.
        float mAbsorbStrength = 1.0f;
        float mAbsorbTime = 0.3f;
        float mFullImpact = 300.0f;
        // Run-out: while the body carries more than mRunOutSpeed (u/s), the
        // foot trailing the carry steps to the capture point at the carried
        // speed (com + gain x Tc x v), at most mMaximumSteps, Footfall's clock
        // held. A step starts once the capture point is mStepMargin past the
        // leading planted foot.
        float mRunOutSpeed = 60.0f;
        float mGain = 1.0f;
        float mStepMargin = 5.0f;
        int mMaximumSteps = 3;
        // Feet stay this far apart across the carry (each on its side).
        float mWidth = 8.0f;
        // Release when the capture point is still beyond the reach after
        // this many run-out steps (0 = never).
        int mReleaseAfterSteps = 2;
        // The run-out ends after this long whatever happens (s).
        float mMaximumTime = 1.5f;
    };

    struct LandingInput
    {
        BalanceState mState;
        std::array<MovingFootState, 2> mFeet;
        float mDt = 0.0f;
        float mReach = 60.0f;
        float mMinimumSwingTime = 0.2f;
        // This frame is the touchdown, with this downward speed (u/s).
        bool mTouchdown = false;
        float mImpactSpeed = 0.0f;
        LandingSettings mSettings;
    };

    struct LandingCommand
    {
        bool mActive = false;
        float mRootStrengthFloor = 0.0f; // max(budget, floor)
        bool mHoldClock = false; // FootfallCommand::mHoldClock
        std::optional<std::size_t> mSwingNow;
        float mSwingTime = 0.25f;
        std::array<std::optional<BalanceVec2>, 2> mLanding;
        bool mRelease = false;
        int mSteps = 0;
    };

    class LandingController
    {
    public:
        // Empty before the first touchdown and once the landing is over.
        std::optional<LandingCommand> update(const LandingInput& input);
        void reset() { *this = LandingController(); }
        bool active() const { return mActive; }

    private:
        bool mActive = false;
        float mSince = 0.0f;
        float mImpact = 0.0f;
        int mSteps = 0;
        bool mWasSwinging = false;
    };

    struct RootBudgetSettings
    {
        float mStanding = 0.15f;
        float mMoving = 0.3f;
        float mMovingSpeed = 15.0f;
    };

    struct RootBudgetInput
    {
        float mHitStrength = 1.0f; // the soft root strength the hit rules leave
        float mRequestSpeed = 0.0f;
        // The body's own horizontal speed (center of mass). A body still
        // running after the request stops is moving: it keeps the moving
        // budget while it brakes (0 = request only, the old rule).
        float mBodySpeed = 0.0f;
        RootBudgetSettings mSettings;
    };

    // The soft root strength to apply: min(hit strength, budget). Strength
    // scales every soft-root term, the horizontal and tilt springs, the
    // vertical spring and weight support alike, so the budget covers the
    // vertical share too (a weight-supported body hangs upright and skates
    // instead of falling).
    float rootBudgetStrength(const RootBudgetInput& input);
}

#endif
