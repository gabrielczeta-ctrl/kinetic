#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_HANDDRIVE_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_HANDDRIVE_HPP

#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Vec3.h>

#include <array>
#include <optional>

namespace RagdollCore
{
    // KINETIC-2 phase 1, the hand drive (docs/ragdoll/designs/
    // K2-WEAPON-phase1.md, "The modules" 3; MOVE-01 4.8): the weapon carried
    // by the arm's own joints. The grip is pulled toward the planner's target
    // by a PD in task space; the force that asks for is mapped onto the
    // shoulder, elbow and wrist by the Jacobian transpose (each joint's
    // torque is the lever from it to the grip, crossed with the force), the
    // hand's rotation toward the target by a torque all three carry, gravity
    // on the arm and weapon cancelled joint by joint, and the elbow led
    // toward the planner's pole. Each torque is applied as a pair - on the
    // joint's child body and, opposite, on its parent - so nothing pushes
    // from outside the body: what the arm cannot do shows as lag, the weight
    // the player feels. Each joint's torque is capped at its strength.
    //
    // Pure: world-space vectors in, torques out.

    // The arm as it is this step. Joints proximal to distal: the shoulder
    // (upper arm on the chest), the elbow (forearm on the upper arm), the
    // wrist (hand on the forearm). The weapon is rigid on the hand.
    struct HandChain
    {
        struct Body
        {
            JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
            float mMass = 0.0f;
        };
        std::array<JPH::RVec3, 3> mJoint{}; // world anchors
        // The bodies distal to the shoulder: upper arm, forearm, hand, weapon.
        std::array<Body, 4> mBody{};
        JPH::RVec3 mGrip = JPH::RVec3::sZero(); // the point driven (the hand's grip)
        JPH::Vec3 mGripVelocity = JPH::Vec3::sZero();
        JPH::Quat mHandRotation = JPH::Quat::sIdentity();
        JPH::Vec3 mHandAngularVelocity = JPH::Vec3::sZero();
        // The hand and weapon's moment of inertia about the wrist (kg u^2),
        // for the rotation's torque; 0: estimated from the masses.
        float mHandInertia = 0.0f;
        // The weapon's own moment of inertia across its long axis, about its
        // centre (kg u^2), K2_HAND_EFFECTIVE_MASS. With it the mass the grip
        // carries is the hand and weapon as one rigid body pushed at the
        // grip, which partly turns it about its centre instead of moving it:
        // a weapon whose centre sits far from the grip is lighter there than
        // its mass. Taken as its whole mass, the grip's spring rang at the
        // 60 Hz step on such light one-handers (VARIANT-01 11-07, steel and
        // daedric daggers: the arm +-9.5 rad/s each step in the guard, ~450
        // jolts against the iron dagger's 90). It also sizes the blade's turn
        // (without mHandInertia) on the pair's inertia about their centre.
        // 0: the masses whole, the turn about the wrist.
        float mWeaponInertia = 0.0f;
        bool mRigidTurn = true; // mWeaponInertia sizes the blade's turn too
        // K2_TWO_HAND_TURN: the weapon's own inertia across its long axis
        // (kg u^2) added to the point-mass estimate of the turn about the
        // wrist. That estimate is the weapon's mass at its centre: a haft
        // gripped at its middle (a spear, a staff) reads as almost nothing to
        // turn, and its blade was asked for almost no torque. 0: the estimate.
        float mTurnInertia = 0.0f;
        std::array<float, 3> mStrength{}; // each joint's torque cap (N u, the motors' units)
    };

    struct HandDriveGoal
    {
        JPH::RVec3 mGrip = JPH::RVec3::sZero();
        JPH::Vec3 mGripVelocity = JPH::Vec3::sZero(); // the route's own velocity (feed-forward of the damping)
        JPH::Quat mRotation = JPH::Quat::sIdentity();
        JPH::Vec3 mAngularVelocity = JPH::Vec3::sZero();
        std::optional<JPH::Vec3> mPole; // the elbow's bend direction (world, unit)
        // The blade in the hand's frame: with it the rotation is the blade's
        // direction (driven in full) and the roll about it (by mRollShare), a
        // blade's two edges alike, so the roll is taken the nearer way (at
        // most a quarter turn): a half turn of the forearm is never asked for.
        // None: the whole rotation, as given.
        std::optional<JPH::Vec3> mBladeAxis;
        float mRollShare = 0.5f;
    };

    struct HandDriveGains
    {
        float mFrequency = 6.0f; // Hz, the grip's spring
        float mDamping = 1.0f; // critical
        float mRotationFrequency = 6.0f;
        float mRotationDamping = 1.0f;
        float mPoleFrequency = 3.0f; // the elbow's lead toward its pole
        float mStrength = 1.0f; // share of the joints' caps (skill, fatigue)
        // The asks are capped as the old weapon drive's were (12 g at the
        // grip, 400 rad/s^2 for the hand): beyond, the arm only saturates.
        float mMaxAcceleration = 12.0f * 9.80665f * 69.99125109f; // units/s^2
        float mMaxAngularAcceleration = 400.0f;
        bool mGravity = true;
        // The step the torques act over: the PD is the stable (implicit) one
        // for it (Tan, Liu and Turk 2011), so a stiff spring on a light hand
        // does not ring at 60 Hz (the explicit one overshot a 30 deg turn to
        // 88 deg in 0.17 s on the bench).
        float mDt = 1.0f / 60.0f;
        // The hand's rotation by a torque all three joints carry (true), or
        // left to the wrist joint's own motor (false: the caller drives it to
        // the goal, see nearestEdgeRotation; only the grip and gravity here).
        bool mRotationTorque = true;
        // The rotation's torque is still computed for mRotationTorque false
        // (HandDriveOutput::mRotationTorque) for a caller that applies it
        // another way (the bench's assist: on the weapon alone).
        bool mComputeRotation = false;
    };

    struct HandDriveOutput
    {
        // Torque on each joint's child body (its parent takes the opposite).
        std::array<JPH::Vec3, 3> mTorque{};
        std::array<float, 3> mUtilization{}; // |torque| / cap before the cap, per joint
        JPH::Vec3 mForce = JPH::Vec3::sZero(); // the task force asked at the grip
        JPH::Vec3 mRotationTorque = JPH::Vec3::sZero(); // the hand's turn, before any joint's share
        float mEffectiveMass = 0.0f;
    };

    // The gains for a weapon: the grip's frequency falls with the weapon's
    // carried weight as the cut's pace does (sqrt(carried / 1.6)), the
    // strength by weapon control (0.6 at none, 1 at full).
    // `response` (K2_WEAPON_PACE) scales the springs' frequencies and the
    // acceleration caps by its square, so the stiffer spring is not capped back.
    HandDriveGains handDriveGains(float weaponWeight, float control, float response = 1.0f);

    // The goal rotation, or the same blade turned a half turn about itself
    // (the other edge leading), whichever is nearer `current`.
    JPH::Quat nearestEdgeRotation(JPH::QuatArg goal, JPH::QuatArg current, JPH::Vec3Arg bladeAxis);

    HandDriveOutput solveHandDrive(const HandChain& chain, const HandDriveGoal& goal, const HandDriveGains& gains,
        float gravity);
}

#endif
