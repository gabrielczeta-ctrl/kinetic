#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_ACTIVEBODY_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_ACTIVEBODY_H

// Active body: the balance layer of a physical, wobbly character.
//
// Every body part is dynamic; joint motors (elsewhere) pull it toward the
// animation pose. This module computes the balance wrench that keeps the body
// on its feet and following the character's capsule:
//   - a stable-PD pull of the whole body toward the anchor (capsule position
//     plus the animation's pelvis offset), capped in g;
//   - weight support for the upper body;
//   - separate yaw and tilt springs on the torso chain;
//   - ground routing: a fraction of that wrench is taken back out of the
//     grounded feet, so the weight travels through the legs into the floor
//     instead of hanging the body from an invisible harness.
// Pure computation: no physics world access, so it is unit tested directly.

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <string>
#include <vector>

namespace RagdollCore
{
    struct ActiveBodySettings
    {
        // Balance springs (Hz) and damping ratio.
        float mHorizontalFrequency = 3.0f;
        float mVerticalFrequency = 6.0f;
        float mYawFrequency = 4.0f;
        float mTiltFrequency = 20.0f;
        float mDampingRatio = 1.0f;
        // Caps in g (whole-body acceleration) or g / gyration radius (angular).
        float mHorizontalCap = 5.0f;
        float mUpCap = 3.0f;
        float mDownCap = 1.0f;
        float mYawCap = 3.0f;
        float mTiltCap = 40.0f;
        float mGyrationRadius = 15.0f;
        // Extra upright (tilt) authority on the root (pelvis) only. The pelvis
        // is light and the running legs' hip motors tipped it 49 deg under an
        // upright torso in fast running turns.
        float mRootTiltBoost = 1.0f;
        // Same for yaw: the legs swivel the pelvis at every step (walking hip
        // yaw rate 134 deg/s on the active body vs 76 on the rigid root).
        float mRootYawBoost = 1.0f;
        // Maximum change of the character capsule's horizontal velocity (u/s^2,
        // 0 = unlimited). Morrowind reverses a run instantly; a physical body
        // cannot follow a 24-unit circle at run speed and was flung over.
        float mCapsuleAcceleration = 1400.0f;
        // How fast the body turns toward the character's facing (deg/s). The
        // camera can spin the facing instantly; a body spun at 360 deg/s while
        // running flung its legs out and fell.
        float mMaximumYawRate = 240.0f;
        // Rotation through the constraint solver: a kinematic anchor that
        // follows the target pose drives the pelvis orientation with a Jolt
        // 6-DOF rotation motor (spring anchor_frequency Hz, torque cap in body
        // weight x gyration radius). It is solved together with the leg motors
        // instead of one step behind them. 0 = external torques (the default).
        float mRotationConstraint = 0.0f;
        float mAnchorFrequency = 8.0f;
        float mAnchorTorque = 2.0f;
        // Upper-body weight carried by the balance force (1 = all of it).
        float mWeightSupport = 1.0f;
        // Ground routing, vertical and horizontal separately: 0 = that part of
        // the balance force is external (a harness); 1 = it is taken back out of
        // the grounded feet, so the legs carry it. Vertical routing puts the
        // weight through the legs; horizontal routing pushes the feet along the
        // floor and is limited by their grip (full horizontal routing slipped
        // the feet and felled 40 of 45 walking cases).
        float mGroundRouting = 0.0f;
        float mGroundRoutingHorizontal = 0.0f;
        // Safety net for routed support: after this long with no grounded foot
        // (legs flung out in a hard turn), the routed share fades back into
        // external support over mAirCatchFade seconds. Ordinary stride flight
        // is shorter, so normal steps still carry the real weight.
        float mAirCatchDelay = 0.12f;
        float mAirCatchFade = 0.1f;

        // Hits (impulse in the game's hit bands: stun 400, stagger 1250, fall 1750).
        float mHitZeroAt = 5000.0f; // impulse that takes all balance
        float mHitRecoveryPerSecond = 1.2f;
        float mHitFloor = 0.15f;
        float mToppleUpDot = 0.5f;
        float mShoveSpeed = 160.0f; // whole-body shove at the fall band (u/s)
        float mStumbleSeconds = 0.4f;
        float mStumbleThreshold = 0.35f;
        float mGetUpStrength = 0.8f;

        // Loads "key = value" lines (# comments); unknown keys are reported.
        // Returns the problems found, empty when the file parsed cleanly.
        std::vector<std::string> load(const std::string& path);
        // Applies one "key = value" line; false for an unknown key or bad value.
        bool set(const std::string& key, float value);
    };

    struct ActiveBodyPart
    {
        float mMass = 0.0f;
        JPH::Vec3 mVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mAngularVelocity = JPH::Vec3::sZero();
        bool mUpper = false; // receives translation and weight support
        bool mTorso = false; // receives rotation
        bool mFoot = false;
        bool mGrounded = false; // a foot in ground contact
        bool mRoot = false; // the pelvis (receives mRootTiltBoost)
    };

    struct ActiveBodyInput
    {
        float mDuration = 1.0f / 60.0f;
        JPH::Vec3 mGravity = JPH::Vec3(0.0f, 0.0f, -686.0f);
        JPH::Vec3 mRootPosition = JPH::Vec3::sZero(); // relative to any origin shared with the target
        JPH::Quat mRootRotation = JPH::Quat::sIdentity();
        JPH::Vec3 mTargetPosition = JPH::Vec3::sZero();
        JPH::Quat mTargetRotation = JPH::Quat::sIdentity();
        JPH::Vec3 mTargetVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mTargetAngularVelocity = JPH::Vec3::sZero();
        float mStrength = 1.0f; // 0..1, lowered by hits
        // 0..1 on the horizontal hold only (weight and tilt keep mStrength):
        // lowered while catch steps take a push.
        float mHorizontalScale = 1.0f;
        float mSupport = 1.0f; // 0..1
        float mAirborneTime = 0.0f; // seconds since a foot was last grounded
        // Multiplier on the vertical caps (a jump launch needs more than 3 g).
        float mUpCapScale = 1.0f;
        std::vector<ActiveBodyPart> mParts;
    };

    struct ActiveBodyOutput
    {
        std::vector<JPH::Vec3> mForces; // per part
        std::vector<JPH::Vec3> mAngularAccelerations; // per part (torso parts only)
        JPH::Vec3 mNetForce = JPH::Vec3::sZero(); // external force left after routing
        float mPositionError = 0.0f;
        float mAngleError = 0.0f;
        bool mSaturated = false;
        bool mValid = false;
    };

    ActiveBodyOutput computeActiveBodyWrench(const ActiveBodySettings& settings, const ActiveBodyInput& input);
}

#endif
