#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_WEAPONSWING_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_WEAPONSWING_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

namespace RagdollCore
{
    struct PunchCycleState
    {
        // Queue a real input edge so even press/release within one render frame
        // produces a punch on the next fixed step.
        bool mPendingPress = false;
        bool mReturning = false;
        float mExtensionRemaining = 0.0f;
    };

    struct PunchCycleOutput
    {
        bool mStarted = false;
        bool mExtending = false;
    };

    // The caller measures withdrawal; a held button cannot bypass it. Cancel
    // the transient state when stance, equipment or controller ownership ends.
    PunchCycleOutput stepPunchCycle(PunchCycleState& state, bool held, bool returned, float duration);

    // Turns a stream of aim input into a point the weapon hand chases.
    //
    // The swing is not animated and not scripted: the hand is given somewhere
    // to be, the arm motors pull it there, and whatever the weapon does on the
    // way is what the physics produces. That is the whole mechanism. What
    // makes it feel like a weapon rather than a cursor is that the target
    // cannot move faster than mMaximumSpeed, so a heavy swing is one the hand
    // spends time catching up to and the mouse cannot teleport a warhammer
    // across the arc.
    //
    // Offsets are expressed in whatever aiming basis the caller supplies:
    // x is right, y is forward, z is up. For a player swing that basis is the
    // camera's, because the arc has to read as the one drawn on screen, not
    // one relative to a body that is being shoved around mid-swing. Keeping
    // the basis outside means this never needs to know where the camera is.
    struct WeaponSwingProfile
    {
        // How far from the rest point the hand may be driven.
        float mReach = 45.0f;
        // Input units to model units. Input arrives normalized, so this is
        // how much of the reach a full sweep of the mouse buys.
        float mAimToTravel = 90.0f;
        // The arm's own speed limit. This is what gives the swing weight:
        // below it the hand tracks the mouse, above it the mouse is only
        // stating an intention the arm is still working through.
        float mMaximumSpeed = 520.0f;
        // The arm's acceleration limit (units/s^2), zero for none. With it the
        // commanded point speeds up and brakes like a carried mass (spec
        // v0.2 8.1) instead of jumping to mMaximumSpeed, and it brakes by its
        // stopping distance so it does not overshoot where the mouse stopped.
        float mMaximumAcceleration = 0.0f;
        // How quickly the target falls back to rest once the attack ends.
        // Slower than the swing, so releasing settles rather than snaps.
        float mReturnRate = 4.0f;
        // Where the hand sits when no swing is being driven, relative to the
        // shoulder: forward, a little out to the side and below.
        // The native humanoid elbow cannot reach the old 18-unit-deep
        // guard; shoulder clipping then turned mouse-up into sideways travel.
        JPH::Vec3 mRestOffset = JPH::Vec3(6.0f, 30.0f, -8.0f);
        // Weight the arm target is asserted with while swinging. Below one so
        // a swing still yields to a body that is losing its balance.
        float mWeight = 0.85f;
        float mBlendRate = 10.0f;
        // The authored arm motors are sized to HOLD a pose, not to drive a
        // weapon: on the humanoid profile the shoulder carries about a third
        // of the hip's torque and the wrist about a fortieth. Swinging with
        // those limits produces an arm that visibly cannot lift what it is
        // holding, so the swing raises them for as long as it lasts and puts
        // them back afterwards - the same thing the gait already does to the
        // swing leg during a step.
        // Sized so the hand can follow the vanilla attack animation's hand
        // speed (measured ~300 units/s median, ~670 at the fast end): the
        // authored budgets let the arm lift the weapon but not whip it.
        float mMotorTorqueScale = 80.0f;
        // The shoulder cone a swing needs, in radians. The authored 0.85/0.75
        // puts a point in front of the chest outside the joint's reach: the
        // solver correctly refuses to leave the cone and returns a pose about
        // 22 units short, which no amount of torque can close. Opened only
        // for the duration of a swing and restored afterwards.
        float mShoulderNormalHalfCone = 1.75f;
        float mShoulderPlaneHalfCone = 1.55f;
        float mShoulderTwistLimit = 1.05f;
        // A stiffer spring while swinging, so the hand actually chases the
        // point instead of trailing it. Restored on release.
        float mSpringFrequency = 35.0f;
        float mSpringDamping = 2.4f;
    };

    struct WeaponSwingInput
    {
        // Native shoulder-to-hand reach in the aiming basis units. Zero
        // preserves the unrestricted abstract driver. Positive values bound
        // the requested point, including the guard, before accumulating input.
        float mMaximumHandDistance = 0.0f;
        // Aim motion since the last step, normalized so that one unit is a
        // full sweep. Right and up in the actor's frame.
        float mAimRight = 0.0f;
        float mAimUp = 0.0f;
        // The attack input is held. Releasing does not cut the swing off; it
        // lets the target return to rest under mReturnRate.
        bool mHeld = false;
        float mDuration = 0.0f;
        // Elbow-limited inner reach in the same units. Positive values route
        // the gesture forward around this sphere instead of requesting a
        // position the elbow cannot reach. Zero preserves planar gestures.
        float mMinimumHandDistance = 0.0f;
        // Largest distance the commanded point may lead the hand target.
        // Zero preserves the retained-input behavior, where the mouse can bank
        // travel the arm still has to work through. Positive bounds that lag so
        // the weapon stops with the mouse instead of coasting through a
        // backlog when the stroke reverses.
        float mMaximumInputLead = 0.0f;
    };

    struct WeaponSwingState
    {
        // Commanded offset from the rest point, in the actor's frame.
        JPH::Vec3 mOffset = JPH::Vec3::sZero();
        // Retain input beyond the per-step speed budget until the hand target catches up.
        JPH::Vec3 mDesiredOffset = JPH::Vec3::sZero();
        // How fast the commanded point actually moved on the last step. This
        // measures commanded target speed, not hand/weapon impact speed.
        // Damage must use actual contact-point velocity.
        JPH::Vec3 mVelocity = JPH::Vec3::sZero();
        bool mSwinging = false;
        // Track the input edge independently of the settling tail. A new
        // press while returning is a new attack, not the previous contact ID.
        bool mHeld = false;
    };

    // Remove accumulated travel beyond the constrained IK result. The solved
    // offset is relative to the gesture rest point, not the measured hand.
    // Only driven coordinates retract toward rest; undriven depth is preserved.
    void constrainWeaponSwingToReach(WeaponSwingState& state, JPH::Vec3Arg solvedOffset);

    struct WeaponSwingOutput
    {
        // Where the hand should go, relative to the shoulder, in the actor's
        // frame. Rest plus the commanded offset.
        JPH::Vec3 mHandOffset = JPH::Vec3::sZero();
        float mWeight = 0.0f;
        float mSpeed = 0.0f;
        // The target wanted to move faster than the arm can. While this is
        // set the mouse is ahead of the weapon, which is the state a heavy
        // weapon spends most of a swing in.
        bool mSpeedLimited = false;
        bool mSwinging = false;
        bool mStarted = false;
    };

    // Advances one step. Invalid input leaves the state untouched and reports
    // no swing rather than guessing.
    WeaponSwingOutput stepWeaponSwing(
        WeaponSwingState& state, const WeaponSwingInput& input, const WeaponSwingProfile& profile);
}

#endif
