#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_FOOTFALL_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_FOOTFALL_HPP

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <ragdollcore/footfallgait.hpp>
#include <ragdollcore/passiveragdoll.hpp>

#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace RagdollCore
{
    // Footfall as a library: procedural stepping for weighted legs on the
    // soft root, moved out of the engine adapter (applyFootfall in
    // apps/openmw/mwphysics/passiveragdollentry.inl, copied from
    // task/footfall-motionbricks 5898dbd9f9) so the offline scenario and the
    // game run the same code. Measurements in, targets out: no physics world,
    // no engine types. The caller decides eligibility, supplies the ground
    // and applies the outputs (leg springs, contact friction, pelvis drop,
    // feed-forward). Behaviour and knobs are the adapter's, unchanged.

    // Every OPENMW_RAGDOLL_FOOTFALL_* knob of the adapter, with its default.
    struct FootfallSettings
    {
        // _GAIT = stride per u/s, swing height scale, stance width scale, longest step.
        std::array<float, 4> mGait{ 0.35f, 1.0f, 1.0f, 80.0f };
        // _REF = least walking duty, swing height, crouch and bob scales.
        std::array<float, 4> mReference{ 0.55f, 1.0f, 0.0f, 0.0f };
        // _CUSHION = pelvis dip (units), duration (s), knee softness (0-0.9).
        std::array<float, 3> mCushion{ 0.0f, 0.2f, 0.0f };
        // _TURN = gain, cap (units).
        std::array<float, 2> mTurn{ 1.0f, 30.0f };
        // Anchor settle (standing): offset from the drive pelvis that starts
        // a step, and how far toward it the step lands (units; 0 = off). The
        // walk course's step variation is noisy (the same legs 3.95-10.8 in
        // repeats), so single runs could not tell it from no settle.
        std::array<float, 2> mAnchorSettle{ 8.0f, 25.0f }; // _ANCHOR_SETTLE
        // A standing settle step back to the stance spot moves a foot at
        // most this far (units; 0 = no limit; overrun and anchor steps are
        // not capped); a longer way back takes more steps. One
        // 0.3 s settle swing carried a crossed foot 35-45 u back (PGC viewer,
        // 09-30: the foot "shoots" while the body stands still).
        float mSettleStepMax = 20.0f; // _SETTLE_STEP (PGC aa43793e9d; FIX-08 bisect: safe)
        // Standing turns and sway (IDLE-01, user 10-31: "when standing without
        // moving sometimes the models do small steps to stabilize themselves
        // nonstop"): the planted feet keep the heading they were squared to
        // while the body turns less than `degrees` (the hips twist, as a
        // person's do); past it the feet re-square to the body. A stance spot
        // error must also last `patience` seconds before it steps. Turning in
        // place at 34 deg/s stepped every 0.45-0.8 s (9 steps in 6 s with
        // fists). An anchor settle no longer takes a step that moves the
        // foot under 4 units (it tapped every 0.45 s while the body stood
        // 9.8 units off its anchor). _STAND_TURN = "degrees, patience"
        // (0 = off: the old spot and anchor settles).
        std::array<float, 2> mStandTurn{ 35.0f, 0.25f };
        // _CROUCH = walk, run, bob (units).
        std::array<float, 3> mCrouch{ 0.0f, 0.0f, 0.0f };
        // _SNEAK = style on (1 = the walk_stealth reference gait while
        // sneaking), crouch scale (share of that gait's measured pelvis drop,
        // held standing too), standing crouch floor (units).
        // Off: SNEAK-01 x2 - unarmed forward steps evener (stepL 64 -> 11-40),
        // warhammer worse (15-24 -> 53-59); the extra crouch (1,1,8) sank the
        // body 13-29 units under its target and felled it - vanilla's sneak
        // animation already lowers the pelvis 8-11 units.
        std::array<float, 3> mSneak{ 0.0f, 0.0f, 0.0f };
        // _CROUCH_LEGS (CROUCH-01, docs/ragdoll/designs/CROUCH-01-physical-crouch.md):
        // sneaking, the body crouches on its own knees. The planted legs are
        // no longer than the sneak clip's longer leg (hip to ankle; `share`
        // of the way from the straight leg to it), the cap moving at most
        // `down` / `up` units a second (0 = at once: an `up` of 45 held the
        // body 4-8 units under the standing drive and the leg extension then
        // pushed a foot through the floor in 2 of 45 cases; released at once
        // the legs and root stand up in about 0.4 s as before), and the walk's
        // lean no longer straightens the clip's hunch (`lean` 1). Before it the clip's
        // pelvis dropped 12 units at once but the legs, posed from the
        // physical hip, held the body up: it sank at ~7 u/s and reached the
        // clip 1.6 s after the sneak press (CROUCH-01 baseline, unarmed).
        // "share, down u/s, up u/s, lean" (share 0 = off).
        std::array<float, 4> mCrouchLegs{ 0.0f, 40.0f, 0.0f, 1.0f };
        // _STEP_UP: a swing to a higher landing rises to it early, with this
        // share of the rise as clearance (0 = follow the ground under the foot).
        float mStepUp = 0.0f;
        // _EXTEND = gain, cap (units); _EXTEND_I = rate (1/s), cap (units).
        std::array<float, 2> mExtend{ 1.0f, 12.0f };
        std::array<float, 2> mExtendIntegral{ 2.0f, 10.0f };
        // _LEAN = walk, run lean (degrees).
        std::array<float, 2> mLean{ 3.0f, 10.0f };
        float mLateralTime = 0.6f;       // _LATERAL_TIME
        // Backward travel: swing time scale when moving straight back (user
        // 2026-09-30 "running back makes the legs start dragging"; in play
        // both feet stood 31% of the time running back armed, 6% forward).
        float mBackTime = 1.0f;          // _BACK_TIME (1 = off)
        float mClipSync = 1.0f;          // _CLIP_SYNC
        float mMoveWidth = 6.0f;         // _MOVE_WIDTH
        bool mClipWidthMoving = true;    // _CLIP_WIDTH
        bool mLeadScale = true;          // _LEAD_SCALE
        float mSideCorrection = 0.3f;    // _SIDE_CORRECTION
        float mRunPathGain = 1.5f;       // _RUN_PATH_GAIN
        float mTreadSnap = 0.0f;         // _TREAD_SNAP: most a landing moves onto a stair tread (u; 0 = off)
        // _STAIR (PGC-03 C9b, the step-down gait): lock phase (0 = off), the
        // step-down's planar finish, its release t, the trail lift's share of
        // the leg's reach (0 = none), the nosing clearance (u), the probe fix
        // (1 = on: a landing below the capsule's ground window is found).
        std::array<float, 6> mStair{ 0.0f, 0.7f, 0.85f, 0.92f, 3.0f, 0.0f };
        float mRunLiftGain = 1.5f;       // _RUN_LIFT_GAIN
        bool mFollowPath = true;         // _PATH
        float mPathMatch = 12.0f;        // _PATH_MATCH
        float mRunPathMatch = 30.0f;     // _RUN_PATH_MATCH
        float mOverrunRun = 2.0f;        // _OVERRUN_RUN
        float mFirstStep = 0.7f;         // _FIRST_STEP
        // Landing taper: the reference run lift curves hold the foot ~0.03 leg
        // lengths up from 60% to 85% of the swing (the Mixamo foot reaching
        // low before heel strike), x1.5 run lift gain = 4-5 units; a flat
        // physical foot then landed late, every run swing ending 4-8 units
        // above its pin (2026-09-29). The lift eases to zero over this last
        // share of the swing (0 = off).
        float mLandTaper = 0.0f;         // _LAND_TAPER
        float mRunSwingDamping = 0.2f;   // _RUN_SWING_DAMPING
        float mRunSwingStiffness = 2.0f; // _RUN_SWING_STIFFNESS
        float mLookAhead = 0.2f;         // _LEAD
        float mPathLead = 0.08f;         // _PATH_LEAD
        float mSettle = 0.35f;           // _SETTLE
        // Scoped recovery (zero-falls research, PORT-SCOPED-RECOVERY-2026-10-03):
        // used only while FootfallInput::mLoadPolicyActive. Steps of
        // 12 + gait * speed * this, no reference gait table (shorter, quicker
        // steps); unset = off.
        std::optional<float> mLoadProceduralStride;
        // The swing finishes its horizontal travel by this share of the swing,
        // then lowers; the landing stops re-aiming 0.15 earlier (1 = off).
        float mLoadPlanarFinish = 1.0f;
        // Fill FootfallOutput::mMotorFootGoal / mMotorFootVelocity.
        bool mObserveMotorFootGoal = false;
        float mClearance = 4.0f;         // _CLEARANCE
        float mStartBlend = 0.15f;       // _START_BLEND
        float mReach = 1.0f;             // _REACH
        float mFootPitch = 1.0f;         // _FOOT_PITCH
        float mFootPitchStance = 1.0f;   // _FOOT_PITCH_STANCE
        float mFootPitchRate = 600.0f;   // _FOOT_PITCH_RATE
        // Feet oriented from the pelvis's heading only, not its tilt (user
        // bookmarks 1 and 17, 2026-10-06: "walking on its toes"): the foot
        // drive followed the whole pelvis drive, so a pelvis tipped forward
        // (the heavy two-handers' stance, a stuck lean) pointed the toes
        // down - the recording's standing toe axis went 0 deg with a
        // longsword, -16..-22 with a claymore, -34..-66 with a warhammer.
        // 1 = level feet, 0 = the pelvis's whole rotation (before).
        float mFootLevel = 0.0f;         // _FOOT_LEVEL
        float mStanceKnee = 0.5f;        // _STANCE_KNEE
        float mRunStanceKnee = 0.8f;     // _RUN_STANCE_KNEE
        // Armed (the boxing reference gait), the stance knee bends at least
        // this share of the reference: a fighting crouch on the move (user
        // 10-05: "armed crouch while moving"; the armed walk stood 3 units
        // taller than the armed stance; at 1 the armed walk is 60.7 over the
        // feet against 65.8, the stance 62.9; 1.5 sank it to 50.7). 0 = the
        // walk's own share.
        float mArmedStanceKnee = 0.0f;   // _ARMED_STANCE_KNEE (play launcher: on)
        // A landed foot still more than [0] units above its pin [1] seconds
        // into stance is not held to the reference knee bend; [0] 0 = off.
        // 0.17 s is past 90% of normal run stances' first contact: only a
        // failing stance reaches (weak run stances 2.3% -> 0.4%, 2026-09-29).
        std::array<float, 2> mStanceKneeAir{ 1.5f, 0.17f }; // _STANCE_KNEE_AIR
        float mSwingFeedForward = 0.5f;  // _SWING_FF
        // WALK-02, capture-aware walking: the walk's steps keep the gait clock, but a
        // landing aims where the body's surplus motion carries it (the
        // pelvis velocity beyond the command, times the capture time
        // sqrt(h / g), ~0.3 s; the old fixed 0.12 s with a 0.3 sideways share
        // reached less than half way), and once that surplus puts the
        // weight more than `margin` units off course with both feet down,
        // the foot further from its landing steps at once (the clock
        // resyncs to it) instead of waiting for the clock's foot.
        // _CAPTURE = "gain, margin u, reach share" (gain 0 = off: the 0.12 s
        // lead and clock-only steps); reach share caps the surplus lead at
        // that share of the leg's length.
        std::array<float, 3> mCapture{ 0.0f, 12.0f, 0.5f };
        // WALK-02 slopes (the feet follow each probe's ground normal):
        // the foot is tilted onto the ground under it (planted: under the
        // pin; swinging: turning from the pin's ground to the landing's),
        // `share` of the slope up to `degrees`, its centre raised for the
        // tilt; an uphill swing lifts `lift` x (1 - normal z) more (30 deg:
        // x1.27 at 2). Needs FootfallInput::mGroundNormal.
        // _SLOPE = "share, degrees, lift" (share 0 = off: level feet).
        std::array<float, 3> mSlope{ 0.0f, 30.0f, 2.0f };
        // STEPPING audit 2026-11-01 (docs/ragdoll/STEPPING.md, the audit in the
        // dev root): the walking band was 26-31% on the clock, the rest
        // overrun or late lifts; feet trailed the capsule. Each item below is
        // off by default.
        // Item 5, the planned overrun: a planted foot moving lifts once it is
        // behind the hip along the travel by more than the stance's own travel
        // past the landing lead plus `share` x the step length (horizontal, not
        // the 3D leg stretch), and running the 3D rule's factor stops at `run
        // cap` x reach (was up to _OVERRUN_RUN 2.0 = 144 u: a foot that far is
        // dragged). _OVERRUN_PLAN = "share, run cap" (share 0 = off).
        std::array<float, 2> mOverrunPlan{ 0.0f, 1.3f };
        // Item 6, landing on contact: a swing
        // past `earliest` of its phase whose foot touches supporting ground and
        // no longer closes on its target lands there; without contact a swing
        // may run up to `extra` seconds past its phase end (was a fixed 0.2 s
        // while the foot was 4 u up). _CONTACT_LAND = "earliest, extra s"
        // (earliest 0 = off: land by time). Needs FootfallInput::mFootContact.
        std::array<float, 2> mContactLand{ 0.0f, 0.2f };
        // Item 7, the motor lead: the joint
        // motors take time to reach a pose, while the hip and the swing goal
        // move on, so the leg is solved for where both will be `lead` seconds
        // on: swing goal + its velocity x lead, hip + the pelvis velocity x
        // lead. _MOTOR_LEAD = "swing s, stance s" (0 = off).
        std::array<float, 2> mMotorLead{ 0.0f, 0.0f };
        // Item 9, the phase lock: walking, each landing sets the gait clock to
        // where that landing belongs in the cycle (left lands at 1 - duty,
        // right half a cycle on), `share` of the way, so the other foot's lift
        // comes a stance time after it instead of on a free-running beat (a
        // late lift no longer brings the next one early). _PHASE_LOCK = share
        // (0 = off, 1 = set it).
        float mPhaseLock = 0.0f;
        // Item 10, lift by the distance left:
        // a swing at least lifts `share` of the horizontal distance the foot
        // still has to its target, up to `most` units, so a foot far from its
        // landing clears the floor instead of scuffing (NPC swings lifted at
        // most ~2 u). _LIFT_LEFT = "share, most" (share 0 = off).
        std::array<float, 2> mLiftLeft{ 0.0f, 12.0f };
        // Item 15, the hip's range: a landing is kept within `degrees` of the
        // hip's sideways (abduction) reach from the pelvis - a wide strafe
        // step the hip cannot swing to is pulled in (the leg-length reach
        // clamp alone let targets past it). _HIP_RANGE = degrees (0 = off).
        float mHipRange = 0.0f;
        // Item 11, ported from the retired adapter (they never ran: the
        // library drove alone): the moving width blends in from 15 to 75 u/s
        // (user 10-30: "starting the run animation sometimes makes both feet
        // jump from side to side" - a fighting stance's width snapped to the
        // run's at 15 u/s); _WIDTH_BLEND = 0 snaps at 15.
        bool mWidthBlend = true;
        // Feet never cross under the body (user 11-01 bookmark 13: "we must
        // be careful to not cross legs even when running, circling,
        // strafing"): the landing gap is kept along the capsule's facing,
        // but circling and strafing turn the hips away from it (the legs
        // follow the travel), so the feet crossed under the pelvis. A
        // landing also keeps this many foot widths across the physical hip
        // line from the other foot (0 = off). _HIP_GAP.
        float mHipGap = 0.0f;
        // Once a stop is asked (FootfallInput::mStopping) a step lands beside
        // where the pelvis comes to rest, the pelvis plus its velocity x this
        // many seconds (user 10-31: after a run stop the body "needs to take a
        // step back": the re-aimed stride landed 25-30 u past the rest point).
        // _STOP_LEAD = seconds (0 = off).
        float mStopLead = 0.06f;

        // The adapter's parsing of the environment (unset keeps the default;
        // single-value knobs ignore negative or non-finite values).
        static FootfallSettings fromEnvironment();
    };

    // The ragdoll's model frame: model = inverseRotation * (world - position) * inverseScale.
    struct FootfallFrame
    {
        JPH::RVec3 mReferencePosition = JPH::RVec3::sZero();
        JPH::Quat mInverseReferenceRotation = JPH::Quat::sIdentity();
        float mInverseUniformScale = 1.0f;

        JPH::RVec3 worldPosition(JPH::Vec3Arg model) const;
        JPH::Vec3 modelPosition(JPH::RVec3Arg world) const;
        JPH::Vec3 modelVector(JPH::Vec3Arg world) const;
    };

    struct FootfallLeg
    {
        std::size_t mHip = 0;
        std::size_t mKnee = 0;
        std::size_t mFoot = 0;
        std::optional<HingeLegGeometry> mGeometry;
    };

    struct FootfallInput
    {
        // A swing lifts the foot at least this high (units, 0 = the gait's
        // own; at most 20): standing staggers clear the floor (Codex round 4,
        // PORT-STAGGER: standing steps 1.27 -> 1.46 offline).
        float mMinimumSwingLift = 0.0f;
        // A crouch asked from outside (units, added to the gait's pelvis drop,
        // moving or standing): the low stance of a low cut (aim item 2).
        float mExtraCrouch = 0.0f;
        // A stance knee bend asked from outside (degrees, at most 100, moving or
        // standing): caps the planted legs' length like the gait's stance
        // knee, so the body sinks on its legs (the root drop alone left the
        // standing chest where it was: the legs carry it under PGC).
        float mExtraKnee = 0.0f;
        float mDt = 0.0f;
        // The adapter's gate: Footfall enabled, soft root on and holding the
        // root, standing or bracing, not in a jump window, on the ground, not
        // swimming or flying. Not eligible: Footfall resets.
        bool mEligible = false;
        std::size_t mPelvis = 0;
        std::array<std::optional<FootfallLeg>, 2> mLegs; // left, right
        std::vector<std::size_t> mSpine;                 // pelvis upward
        std::vector<std::pair<std::size_t, bool>> mChestCarried; // body, is neck/head
        const RagdollPose* mPhysicsPose = nullptr;       // measured, model space
        const RagdollPose* mClipPose = nullptr;          // the animation's, model space
        FootfallFrame mFrame;
        // Ground height under a point (world); nullopt when there is none or
        // it is too steep. `reference` is the actor's height.
        std::function<std::optional<double>(const JPH::RVec3& point, double reference)> mGround;
        // The ground's normal under a point (world, up-facing; nullopt when
        // there is none): the same probe as mGround. Optional (mSlope).
        std::function<std::optional<JPH::Vec3>(const JPH::RVec3& point, double reference)> mGroundNormal;
        double mActorZ = 0.0;
        float mYaw = 0.0f;                               // radians; forward = (sin, cos)
        JPH::Vec3 mCommandVelocity = JPH::Vec3::sZero(); // the root's actual velocity, world
        bool mArmed = false;                             // weapon drawn
        bool mSneaking = false;                          // sneak stance
        const FootfallGaitTable* mGaitTable = nullptr;
        // A recovery episode is running: the mLoad* settings apply.
        bool mLoadPolicyActive = false;
        // Each foot touches supporting ground this step (the contact estimate),
        // for _CONTACT_LAND; unset = unknown (land by time).
        std::optional<std::array<bool, 2>> mFootContact;
        // A stop is asked for (the keys released, the capsule braking):
        // landings aim where the pelvis comes to rest (_STOP_LEAD).
        bool mStopping = false;
        // Diagnostics: the adapter's RAGDOLL_FOOTFALL* lines.
        std::function<void(const std::string&)> mTrace;
    };

    // Swing/stance leg motor spring scales against the profile's (hip and
    // knee; the knee also takes mKneeGive on the frequency).
    struct FootfallLegSprings
    {
        float mStiffness = 1.0f;
        float mDamping = 1.0f;
        float mKneeGive = 1.0f;
    };

    struct FootfallOutput
    {
        std::array<bool, 2> mPinned{ false, false };   // planted and held this step
        std::array<bool, 2> mSwinging{ false, false }; // swinging this step
        // Model-space angular velocity of the driven bodies (feed-forward).
        std::unordered_map<std::size_t, JPH::Vec3> mAngularVelocity;
        std::array<FootfallLegSprings, 2> mSprings;
        // The side's springs changed this step: apply mSprings (a reset sets
        // both back to the profile's, scale 1).
        std::array<bool, 2> mSpringsChanged{ false, false };
        std::array<bool, 2> mSwingStarted{ false, false }; // a swing began this step
        // With mObserveMotorFootGoal: the world point the leg motors drive the
        // foot to (planted or swinging), and its velocity while swinging.
        std::array<std::optional<JPH::RVec3>, 2> mMotorFootGoal;
        std::array<JPH::Vec3, 2> mMotorFootVelocity{ JPH::Vec3::sZero(), JPH::Vec3::sZero() };
    };

    // The balance helper's hooks (design-pgc-balance-helper-2026-09-28.md):
    // given before a step, used by that step only; an empty command leaves
    // Footfall exactly as it is.
    struct FootfallCommand
    {
        // Pause the gait clock: Footfall takes no step of its own (no clock
        // lifts, settle or stop steps). Released, it restarts from the
        // current feet as from a stand.
        bool mHoldClock = false;
        // Start a swing of this foot now (0 left, 1 right), lasting
        // mSwingTime, when both feet are planted.
        std::optional<std::size_t> mSwingNow;
        float mSwingTime = 0.3f;
        // Landing of the current (or just started) swing, world x and y;
        // the height comes from the ground. Footfall stops re-aiming it.
        std::array<std::optional<JPH::RVec3>, 2> mLanding;
        // Stance legs hold the pelvis (PGC bracing, off by default): a planted
        // leg is solved for the drive pelvis orientation, not the physical
        // one, so its hip motor turns a tipped pelvis back (the stance-hip
        // torque). Off: the leg solve follows any pelvis tilt.
        bool mStanceHoldsPelvis = false;
        // With mStanceHoldsPelvis: an extra pelvis rotation the planted legs
        // aim for (world axis * radians; RagdollCore::PostureController
        // gives it from the torso tilt), leaning the pelvis so the torso
        // stands up.
        JPH::Vec3 mPelvisLean = JPH::Vec3::sZero();
        // No anchor settle: the soft root does not hold the body to the
        // anchor (bracing, root let go), so stepping toward it walks the feet
        // away from under the body (a 65 u backstep, PGC viewer 09-30).
        bool mNoAnchorSettle = false;
        // The settle step is not capped (mSettleStepMax): set while the
        // weapon arm is driven (drag, strike, guard), so the stance is tidied
        // in one step as before - the capped settle broke FIX-08 guided
        // warhammer strikes in game (0/3 vs 3/3, 10-01).
        bool mFullSettle = false;
    };

    class FootfallController
    {
    public:
        explicit FootfallController(FootfallSettings settings = {})
            : mSettings(std::move(settings))
        {
        }

        // One fixed step: rewrites the legs (and the spine lean) of
        // `drivePose` (model space) and fills `output`.
        void step(const FootfallInput& input, RagdollPose& drivePose, FootfallOutput& output);
        void reset(FootfallOutput& output);
        // The balance helper's command for the next step (see FootfallCommand).
        void command(const FootfallCommand& command) { mCommand = command; }
        // Sets mFullSettle on the pending command without replacing it.
        void setFullSettle(bool fullSettle) { mCommand.mFullSettle = fullSettle; }
        // Sets mStanceHoldsPelvis on the pending command (never clears one a
        // helper set): the planted legs hold the pelvis outside bracing too.
        void setStanceHolds(bool holds)
        {
            if (holds)
                mCommand.mStanceHoldsPelvis = true;
        }

        bool drivesLeg(std::size_t side) const { return mPinned[side] || mSwinging[side]; }
        // Pelvis target offsets (units down) while walking or running.
        float pelvisDrop() const { return mPelvisDrop; }
        float cushionDrop() const { return mCushionDrop; }
        const FootfallSettings& settings() const { return mSettings; }

        struct Foot
        {
            bool mSwinging = false;
            JPH::RVec3 mPin = JPH::RVec3::sZero(); // planted: where; swinging: where it lifted
            JPH::RVec3 mTarget = JPH::RVec3::sZero();
            float mPhase = 0.0f;
            float mDuration = 0.3f;
            float mLift = 10.0f;
            std::optional<std::array<float, 21>> mLiftCurve;
            std::optional<std::array<float, 21>> mProgressCurve;
            std::optional<std::array<std::array<float, 21>, 3>> mPathCurve;
            JPH::Vec3 mPathStart = JPH::Vec3::sZero();
            std::optional<std::array<float, 21>> mStanceKnee;
            float mStanceTime = 0.0f;
            float mStanceDuration = 0.5f;
            // The balance helper set this swing's landing (no re-aiming).
            bool mTargetHeld = false;
            // The tread this swing's landing was snapped onto (its height):
            // a re-aim keeps it unless the raw target moves 6 u past the edge.
            std::optional<double> mTreadZ;
            // The stair gait (mStair): a riser between pin and target, a step
            // down, the ground under the pin, the locked landing height, the
            // swing's horizontal direction, scans this swing.
            bool mStair = false;
            bool mStepDown = false;
            double mPinGround = 0.0;
            double mLockedZ = 0.0;
            JPH::Vec3 mSwingDir = JPH::Vec3::sZero();
            int mScans = 0;
            // mSlope: the ground's normal under the pin and the landing, and
            // where the landing's was read.
            JPH::Vec3 mPinNormal = JPH::Vec3::sAxisZ();
            JPH::Vec3 mTargetNormal = JPH::Vec3::sAxisZ();
            JPH::RVec3 mTargetNormalAt = JPH::RVec3::sZero();
        };
        const std::array<Foot, 2>& feet() const { return mFeet; }
        bool walking() const { return mWalking; }
        float clock() const { return mClock; }
        // mCapture: early steps taken, and how many of them were not the
        // clock's foot (since construction).
        unsigned captureSteps() const { return mCaptureSteps; }
        unsigned captureSwaps() const { return mCaptureSwaps; }

        static constexpr float sUnitsPerMetre = 70.4f;

    private:
        FootfallSettings mSettings;
        FootfallCommand mCommand;
        // Where an anchor-settle step lands about, until it lands.
        std::optional<JPH::RVec3> mSettleCenter;
        // The heading the planted feet were squared to (standing; mStandTurn)
        // and how long a stance spot error has lasted.
        std::optional<JPH::Vec3> mStanceForward;
        float mSpotErrorTime = 0.0f;
        // Standing settle steps: a clock, the last settle per foot (when, the
        // move) and a hold after one stepped a foot back (mStandTurn).
        float mSettleClock = 0.0f;
        float mStillTime = 0.0f; // seconds the pelvis has stayed under 10 u/s (standing)
        float mSettleHoldUntil = 0.0f;
        std::array<std::pair<float, JPH::Vec3>, 2> mLastSettle{
            { { -100.0f, JPH::Vec3::sZero() }, { -100.0f, JPH::Vec3::sZero() } } };
        float mSinceLanding = 0.0f;
        float mLean = 0.0f;
        float mExtendIntegral = 0.0f;
        int mStepsSinceStart = 0;
        std::array<float, 3> mDebug{};
        std::array<float, 2> mLegDampingScale{ 1.0f, 1.0f };
        std::array<float, 2> mLegStiffnessScale{ 1.0f, 1.0f };
        std::array<float, 2> mFootPitch{ 0.0f, 0.0f };
        std::array<float, 2> mKneeGive{ 1.0f, 1.0f };
        std::array<bool, 2> mLiftDue{ false, false };
        std::array<Foot, 2> mFeet;
        bool mReady = false;
        float mDoubleSupport = 0.0f;
        float mClock = 0.0f;
        float mClipStrideTime = 0.0f, mClipStrideCorrection = 0.0f, mClipStridePhaseError = 0.0f;
        float mClipStrideExtremum = 0.0f;
        std::optional<float> mClipStrideHalf, mClipStrideLast, mClipStridePrevious;
        std::optional<bool> mClipStrideRising;
        bool mWalking = false;
        float mPelvisDrop = 0.0f;
        float mSneakCrouch = 0.0f;
        float mCrouchLength = -1.0f; // mCrouchLegs: the planted legs' cap (model units; < 0 = none)
        float mCushionDrop = 0.0f;
        std::size_t mLastLanded = 1;
        std::optional<JPH::RVec3> mPreviousPelvis;
        JPH::Vec3 mSurplus = JPH::Vec3::sZero(); // mCapture: pelvis velocity beyond the command, filtered
        unsigned mCaptureSteps = 0, mCaptureSwaps = 0;
        // _CONTACT_LAND: the swing foot's horizontal distance to its target last step.
        std::array<float, 2> mSwingGap{ -1.0f, -1.0f };
        // _MOTOR_LEAD: the swing goals last step (world).
        std::array<std::optional<JPH::RVec3>, 2> mPreviousGoal;
        std::optional<JPH::Vec3> mPreviousCommand;
        JPH::Vec3 mTurnAcceleration = JPH::Vec3::sZero();
        std::array<bool, 2> mPinned{ false, false };
        std::array<bool, 2> mSwinging{ false, false };
        std::optional<float> mRestHeight;
        std::optional<float> mHalfWidth;
        std::array<std::optional<JPH::Quat>, 2> mFootRotation;
        std::unordered_map<std::size_t, JPH::Quat> mPreviousRotation;
        std::optional<JPH::Quat> mPhysToDrive;
        std::array<std::optional<JPH::Vec3>, 2> mHip;
        std::array<std::optional<JPH::RVec3>, 2> mPreviousMotorFootGoal;
    };

    // The reference foot's pitch (degrees, + toes down) over the swing or
    // stance: clip 0 Mixamo walk, 1 run, 2 great sword walk.
    float footfallFootPitch(int clip, bool swinging, float phase);
}

#endif
