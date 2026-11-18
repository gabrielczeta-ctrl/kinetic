#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_ROOTTARGET_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_ROOTTARGET_HPP

#include <Jolt/Jolt.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Vec3.h>

#include <optional>

namespace RagdollCore
{
    // KINETIC-2 phase 4, the root's target (PLAN.md "KINETIC-2"): where the
    // soft root holds the pelvis. The actor's position plus the clip's
    // pelvis offset, the run's bob flattened, the drops and the teeter
    // subtracted, the blow's recoil added; then followed with inertia, and
    // the anchor's give under a shove. Moved from the old integration's
    // updateKinematicRoot behind OPENMW_RAGDOLL_K2_ROOT (with rootstrength),
    // the old path beside it until the user's verdict.
    //
    // Pure: no engine types.

    // The run's bob (user 10-30: "when running sometimes the two feet are
    // suspended and the character slides"): the root target rode the run
    // clip's pelvis bob and the spring threw the body up on every rise.
    // Running, the bob keeps `keep` of its swing about its own mean and the
    // pelvis sits `sink` lower, faded in from 60% to 100% of `speed` asked;
    // the sink fades out while the request turns (full straight, gone at
    // 1.5 rad/s: a warhammer's fast-turn run ended leaning 15-23 deg with it).
    // OPENMW_RAGDOLL_RUN_BOB = "keep, sink, speed" (1,0 = off).
    struct RunBobSettings
    {
        float mKeep = 1.0f;
        float mSink = 0.0f;
        float mSpeed = 250.0f;
    };
    struct RunBobState
    {
        std::optional<float> mMean;
        float mShare = 0.0f;
        std::optional<float> mHeading;
        float mTurnRate = 0.0f;
    };
    // The offset's height after the bob rule; `request` is the asked
    // horizontal velocity, `footfall` whether Footfall drives the legs.
    float applyRunBob(RunBobState& state, float z, JPH::Vec3Arg request, bool footfall, const RunBobSettings& settings,
        float dt);

    struct RootTargetInput
    {
        JPH::RVec3 mActor = JPH::RVec3::sZero();
        JPH::Vec3 mOffset = JPH::Vec3::sZero(); // the clip's pelvis, actor-rotated, after the bob
        // Lowered by the support drop, Footfall's pelvis drop, the daze and
        // the integration's own crouch (landing, slope).
        float mDrop = 0.0f;
        // The ledge teeter (EDGE teeter): the hips toward the edge, and the
        // pelvis lowered 20 u per unit past 0.4.
        float mTeeter = 0.0f;
        JPH::Vec3 mTeeterToward = JPH::Vec3::sZero();
        float mTeeterHips = 0.0f;
        JPH::Vec3 mRecoil = JPH::Vec3::sZero(); // a blow's recoil offset
    };
    JPH::RVec3 rootTargetPosition(const RootTargetInput& input);

    // Target inertia: Morrowind changes the capsule's velocity in one frame
    // (a stop from a 230 u/s run is ~20 g); the target follows the capsule
    // through an acceleration-limited follower, so the body carries its
    // momentum a short way instead of pivoting over planted feet.
    // OPENMW_RAGDOLL_SOFT_ROOT_INERTIA = the follower's acceleration cap
    // (u/s^2, 0 = off: the capsule itself). A capsule jump over 2000 u/s (a
    // teleport) resets the follower.
    struct RootFollowerState
    {
        std::optional<JPH::RVec3> mPrevious; // the capsule target last step
        std::optional<JPH::RVec3> mFollower;
        JPH::Vec3 mVelocity = JPH::Vec3::sZero();
    };
    JPH::RVec3 followRootTarget(RootFollowerState& state, JPH::RVec3Arg target, float maximumAcceleration, float dt);

    // The anchor gives way: the pelvis pushed more than `deadZone` off its
    // target carries the capsule toward it at `gain` per unit of excess, at
    // most `maximumSpeed`, so a shove displaces the fighter
    // (OPENMW_RAGDOLL_SOFT_ROOT_YIELD = "deadZone, gain, maximumSpeed").
    JPH::Vec3 rootYieldVelocity(
        JPH::RVec3Arg pelvis, JPH::RVec3Arg target, float deadZone, float gain, float maximumSpeed);

    // The gait's speed cap (user 11-06: fast charges; KINETIC-2 phase 4): the
    // capsule moves no faster on the ground than the legs carry the body. A
    // level-50 guard's charge ran the capsule at ~680 u/s while the walking
    // legs carry ~230 (the course's run) and Stride's sprint ~340 offline; the
    // root towed the body by the hips. Applied to the actor's own movement
    // speed (CharacterController), so the capsule, the legs' request and the
    // movement clip's playback all slow together and the AI's closing speed
    // is the real one; capped in the physics alone, the clip kept the
    // sprint's cadence over a 230 u/s capsule (PLAY-01 sprint, 11-06).
    // OPENMW_RAGDOLL_K2_GAIT_CAP = "speed, player": the cap in u/s (0 = off);
    // player 1 caps the player too, 0 NPCs only.
    struct GaitCapSettings
    {
        float mSpeed = 0.0f;
        bool mPlayer = false;
    };
    GaitCapSettings gaitCapSettings();

    // K2_NPC_START: an NPC's capsule ramps up from a stop instead of leaping
    // to full speed, and turns no faster than `turn` rad/s.
    // OPENMW_RAGDOLL_K2_NPC_START = "u/s^2, rad/s" (each 0 = off).
    struct NpcStartSettings
    {
        float mAcceleration = 0.0f;
        float mTurnRate = 0.0f;
    };
    NpcStartSettings npcStartSettings();
    // The speed allowed this step: `target` once ramped, 0 when not moving.
    float npcStartSpeed(float current, float target, bool moving, float acceleration, float dt);

    // K2_MOVE_ACCEL (user 11-12: "a bit less friction/inertia ... maybe
    // scaled by agility? speed?", "it takes so long for the player to start
    // running"): the bound on how fast a held body's capsule changes its
    // horizontal velocity (active-body.cfg capsule_acceleration, 1400 u/s^2
    // for everyone) read from the actor's Speed attribute instead: `low` at
    // Speed 0, `high` at Speed 100, linear between. Starts, stops and
    // reversals all quicken together. The user's 11-12 sessions at 1400: the
    // capsule at 90% of a 230-270 u/s run 0.17-0.2 s after the key, the
    // pelvis 0.28-0.43 s; a walk 0.08 / 0.13 s.
    // OPENMW_RAGDOLL_K2_MOVE_ACCEL = "low, high" u/s^2 (low 0 = off).
    struct MoveAccelSettings
    {
        float mLow = 0.0f;
        float mHigh = 0.0f;
    };
    MoveAccelSettings moveAccelSettings();
    // The bound for an actor of this Speed; `base` when the switch is off.
    float moveAcceleration(const MoveAccelSettings& settings, float base, float speedAttribute);

    // K2_ARM_STEP: the movement clip's loop (the arms) matched to the legs'
    // cycle, and pulled onto their phase by when the clip's feet pass.
    // OPENMW_RAGDOLL_K2_ARM_STEP = "on, gain" (on 0 = off).
    struct ArmStepSettings
    {
        bool mOn = false;
        float mGain = 0.5f;
    };
    ArmStepSettings armStepSettings();
    struct ArmStepState
    {
        float mLastAhead = 0.0f; // the clip's left foot ahead of its right, last step
        float mLastDelta = 0.0f;
        bool mHave = false;
        float mFactor = 1.0f; // the clip's rate over the legs'
        float mWanted = 1.0f;
        float mError = 0.0f; // the last phase error, cycles (clip late > 0)
        unsigned mEvents = 0;
    };
    // The clip loop to play, seconds; 0 = not walking on Footfall's clock.
    float stepArmStep(ArmStepState& state, float leftAhead, float cycle, float clock, float duty, float gain, float dt);

    // The whole-body drive (user 11-07, instead of the cap): at a run the
    // root's horizontal pull acts on the whole body by mass (every part but
    // a grounded foot) instead of on the upper body, which towed the legs
    // through the hips; the same force, only where it acts.
    // OPENMW_RAGDOLL_K2_ROOT_FIELD = "from, full" u/s asked: the share rises
    // smoothly from 0 at `from` to `full` and stays 1 above (from 0 = off).
    struct RootFieldSettings
    {
        float mFrom = 0.0f;
        float mFull = 0.0f;
    };
    RootFieldSettings rootFieldSettings();
    float rootFieldShare(const RootFieldSettings& settings, float speed);
    // `velocity`'s horizontal part held to `cap` (its direction and vertical
    // part kept); cap <= 0 returns it unchanged.
    JPH::Vec3 capGaitSpeed(JPH::Vec3Arg velocity, float cap);
}

#endif
