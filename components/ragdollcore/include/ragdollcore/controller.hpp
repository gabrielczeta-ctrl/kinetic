#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_CONTROLLER_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_CONTROLLER_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <ragdollcore/profile.hpp>
#include <ragdollcore/combatstate.hpp>
#include <ragdollcore/passiveragdoll.hpp>

#include <cstddef>
#include <optional>
#include <array>
#include <span>
#include <vector>

namespace RagdollCore
{
    struct GroundContact
    {
        std::size_t mBodyIndex = 0;
        JPH::RVec3 mPosition = JPH::RVec3::sZero();
        JPH::Vec3 mNormal = JPH::Vec3::sAxisZ();
        // Jolt reports pre-solve speculative contacts with a negative depth.
        // Count those only if the solver actually delivered a normal impulse.
        float mPenetrationDepth = 0.0f;
        // Actual nonnegative solver impulse during this step, when available.
        std::optional<float> mNormalImpulse;
        // Actual solver material combination, observed after contact overrides.
        std::optional<float> mFriction;
        // Post-step world velocity of the material point on this body that
        // generated the contact. Unknown when no point/body is available.
        std::optional<JPH::Vec3> mPointVelocity;
        // Solved tangential impulse magnitude at this contact point. This is
        // not a force, a world vector, or the planner's friction-limited flag.
        std::optional<float> mFrictionImpulseMagnitude;

        bool hasSupport() const
        {
            return mPenetrationDepth >= 0.0f || (mNormalImpulse && *mNormalImpulse > 0.0f);
        }
    };

    struct FootBodyIndices
    {
        std::size_t mLeft = 0;
        std::size_t mRight = 0;
    };

    struct SupportEstimate
    {
        bool mLeftFootSupported = false;
        bool mRightFootSupported = false;
        bool mCenterOfMassSupported = false;
        std::size_t mFootContactCount = 0;
        std::size_t mGroundContactCount = 0;
        // Horizontal area centroid of the measured support hull; independent
        // of duplicate or interior contact samples. Z remains the mean height.
        JPH::RVec3 mCenter = JPH::RVec3::sZero();
        JPH::RVec3 mGroundCenter = JPH::RVec3::sZero();
        // Horizontal projection of the COM onto the nearest point of the
        // measured foot-contact hull. Equals the projected COM when inside.
        JPH::RVec3 mClosestPoint = JPH::RVec3::sZero();
        float mCenterOfMassDistance = 0.0f;
        float mCenterOfMassHeight = 0.0f;
        // Minimum known friction of all load-bearing upward contacts. Unknown
        // if any such contact lacks material observation.
        std::optional<float> mFriction;

        bool hasSupport() const { return mLeftFootSupported || mRightFootSupported; }
        bool hasGroundContact() const { return mGroundContactCount > 0; }
    };

    std::optional<SupportEstimate> estimateFootSupport(JPH::RVec3Arg centerOfMass,
        std::span<const GroundContact> contacts, FootBodyIndices feet, const ActiveControllerProfile& profile);

    struct SupportJointBody
    {
        // One root at index zero, followed by children after their parents.
        int mParent = -1;
        JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
        JPH::RVec3 mJointPosition = JPH::RVec3::sZero();
        float mMass = 0.0f;
    };

    struct SupportContactForce
    {
        std::size_t mBodyIndex = 0;
        JPH::RVec3 mPosition = JPH::RVec3::sZero();
        JPH::Vec3 mForce = JPH::Vec3::sZero();
    };

    struct JointSupportInput
    {
        std::span<const SupportJointBody> mBodies;
        std::span<const GroundContact> mContacts;
        FootBodyIndices mFeet;
        JPH::Vec3 mRequestedAcceleration = JPH::Vec3::sZero();
        // Optional vertical acceleration request (e.g. leg-extension push
        // during double support). Zero preserves the historical
        // constant-height behavior exactly: the desired-CoP mapping divides
        // by (g + az), so az=0 reduces to the original formulas, and the
        // effective-gravity path already carries the vertical term.
        float mRequestedVerticalAcceleration = 0.0f;
        // Optional push-off (trailing-leg) center-of-pressure shift. After the
        // ordinary desired-CoP hull projection and before the friction-disk
        // clamp, the solved pressure point is shifted by mPushShift world units
        // along mPushDirection (expected aft, toward the trailing foot), then
        // re-projected to the hull; the disk clamp still covers the shifted
        // point. Moving only the fore-aft coordinate of the CoP leaves the
        // lateral transfer's share untouched, so a forward push and the lateral
        // weight transfer do not compete for the same CoP point. The shifted
        // CoP loads the contacts near it (the trailing foot in double support)
        // through the ordinary weight fan, and the realized acceleration,
        // totalForce, cone check and torque mapping stay the sole reality
        // guards. Zero shift skips the block and preserves the historical plan
        // byte-for-byte.
        JPH::Vec3 mPushDirection = JPH::Vec3::sZero();
        float mPushShift = 0.0f;
        // Optional angular-momentum borrow (the hip strategy inside the
        // solver; hip-moment-borrow-2026-09-11.txt): after the ordinary
        // hull/friction-clamped solve, up to this much EXTRA horizontal
        // acceleration (u/s^2) may be realized beyond the zero-moment
        // value by accepting a net moment about the COM, absorbed as
        // counter-rotation of the trunk+arms (derived: every body that is
        // neither a foot nor on a foot's parent chain). The per-contact
        // cone check sees the augmented total force, so the borrow can
        // exceed the moment cap but never friction. Zero preserves the
        // historical plan byte-for-byte.
        float mCounterRotationAcceleration = 0.0f;
        // Fable i52 (pushoff-leg-axis-2026-09-11.txt): when set, and both
        // feet have contacts, the requested net force is decomposed into
        // compressive magnitudes along the two leg axes (foot contact
        // centroid -> hip joint), trailing foot first, instead of the
        // parallel-force distribution. mLegAxisHips are the body indices
        // whose joint position is the hip joint (the thigh bodies).
        std::optional<std::array<std::size_t, 2>> mLegAxisFeet;
        std::array<std::size_t, 2> mLegAxisHips{ { 0, 0 } };
        float mGravity = 0.0f;
        float mFriction = 0.0f;
    };

    struct JointSupportOutput
    {
        bool mSupported = false;
        // Diagnostic displacement of the friction-clamped requested CoP to
        // the contact hull, in world units. Missing means no hull was solved.
        std::optional<JPH::Vec3> mHullProjectionDelta;
        bool mFrictionLimited = false;
        // Fable i52: true when the leg-axis allocation produced the forces.
        bool mLegAxisApplied = false;
        JPH::Vec3 mAcceleration = JPH::Vec3::sZero();
        JPH::RVec3 mCenterOfPressure = JPH::RVec3::sZero();
        std::vector<SupportContactForce> mContactForces;
        // Required torque on each child; its opposite reacts on the parent.
        // Root torque is zero. These are load requests, before motor limits.
        std::vector<JPH::Vec3> mJointTorques;
        JPH::Vec3 mRootForceResidual = JPH::Vec3::sZero();
        JPH::Vec3 mRootMomentResidual = JPH::Vec3::sZero();
    };

    // Constant-height, zero-angular-acceleration inverse statics. Plans contact
    // loads on the measured foot hull, then maps gravity and those loads to
    // internal joint torques. Contact forces are desired loads, never forces to
    // apply directly. No feasible support returns mSupported=false and zero
    // torques; an unsupported body cannot receive virtual ground assistance.
    std::optional<JointSupportOutput> calculateJointSupportControl(
        const JointSupportInput& input, const ActiveControllerProfile& profile);

    struct JointTorqueRequest
    {
        // Body order is topological; slot zero is the unactuated root.
        int mParent = -1;
        // Positive torque acts on this child; the parent receives its opposite.
        JPH::Vec3 mTorque = JPH::Vec3::sZero();
        JPH::Quat mMotorFrame = JPH::Quat::sIdentity();
        JPH::Vec3 mMaximumAxisTorque = JPH::Vec3::sZero();
    };

    struct RootTorqueDistribution
    {
        std::vector<JPH::Vec3> mJointTorques;
        JPH::Vec3 mAppliedRootTorque = JPH::Vec3::sZero();
        JPH::Vec3 mRootTorqueResidual = JPH::Vec3::sZero();
        bool mSupported = false;
        bool mSaturated = false;
    };

    // Route a virtual root request through the hips, accounting for every
    // root-attached joint. Foot loads come from the preceding solved step.
    // No load leaves the base joint requests unchanged (apart from limits).
    // Limits apply to the complete hip request; any unrealized root torque is
    // returned as a residual, never applied externally or reassigned silently.
    std::optional<RootTorqueDistribution> distributeRootTorque(std::span<const JointTorqueRequest> joints,
        std::size_t leftHip, std::size_t rightHip, float leftFootLoad, float rightFootLoad,
        JPH::Vec3Arg requestedRootTorque);

    struct UprightControllerInput
    {
        // Heights are measured relative to the current support surface. The
        // horizontal COM offset points from the support target to the COM.
        float mTargetPelvisHeight = 0.0f;
        float mPelvisHeight = 0.0f;
        float mPelvisVerticalVelocity = 0.0f;
        JPH::Vec3 mCenterOfMassOffset = JPH::Vec3::sZero();
        JPH::Vec3 mCenterOfMassVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mTargetHorizontalVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mTargetUp = JPH::Vec3::sAxisZ();
        JPH::Vec3 mPelvisUp = JPH::Vec3::sAxisZ();
        JPH::Vec3 mPelvisAngularVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mTorsoUp = JPH::Vec3::sAxisZ();
        JPH::Vec3 mTorsoAngularVelocity = JPH::Vec3::sZero();
        float mTotalMass = 0.0f;
        // Locomotion forces are additionally bounded by the support surface:
        // horizontal force cannot exceed mu * mass * gravity and vertical
        // assistance cannot exceed body weight. Recovery may opt out because
        // its authored unweighting is a separate, explicitly bounded mode.
        float mSupportFriction = 0.0f;
        float mGravityMagnitude = 0.0f;
        bool mLimitToSupportFriction = false;
        // When enabled, contact points describe the support surface actually
        // receiving the actuation reaction. Horizontal force is also bounded
        // so the implied center of pressure stays within those contacts.
        JPH::RVec3 mCenterOfMassPosition = JPH::RVec3::sZero();
        float mCenterOfMassHeight = 0.0f;
        std::vector<JPH::RVec3> mSupportContactPoints;
        bool mLimitToSupportMoment = false;
        bool mRecoveryUnweight = false;
    };

    struct UprightControllerOutput
    {
        JPH::Vec3 mPelvisForce = JPH::Vec3::sZero();
        JPH::Vec3 mPelvisTorque = JPH::Vec3::sZero();
        JPH::Vec3 mSpineTorque = JPH::Vec3::sZero();
        // Aggregate retained for callers that only need to know whether any
        // pelvis-force limit changed the request. The individual flags make
        // actuator, contact-friction, and support-weight failures distinct.
        bool mPelvisForceSaturated = false;
        bool mPelvisMagnitudeSaturated = false;
        bool mPelvisHorizontalForceSaturated = false;
        bool mPelvisSupportMomentSaturated = false;
        bool mPelvisVerticalForceSaturated = false;
        bool mPelvisTorqueSaturated = false;
        bool mSpineTorqueSaturated = false;
    };

    // Computes bounded world-space corrections without mutating physics. The
    // live game and standalone trainer can therefore share identical control.
    std::optional<UprightControllerOutput> calculateUprightControl(
        const UprightControllerInput& input, const ActiveControllerProfile& profile);

    // True when the limbs follow the live animation through the joint motors
    // (kinematic-root route, upright, not interrupted). The other routes use
    // the dynamic gait actuation.
    bool shouldUseAnimationAssistedActuation(LocomotionRoute route, ActiveControllerMode mode,
        bool driveJointMotors, bool locomotionInterrupted);

    struct StanceContactPositionInput
    {
        JPH::RVec3 mAnchorPosition = JPH::RVec3::sZero();
        JPH::RVec3 mDesiredFootOrigin = JPH::RVec3::sZero();
        JPH::Quat mDesiredFootRotation = JPH::Quat::sIdentity();
        JPH::Vec3 mContactOffsetInFoot = JPH::Vec3::sZero();
        float mCorrectionFraction = 0.0f;
        float mMaximumEndpointCorrection = 0.0f;
    };

    struct StanceContactPositionOutput
    {
        JPH::RVec3 mDesiredContactPosition = JPH::RVec3::sZero();
        JPH::Vec3 mContactError = JPH::Vec3::sZero();
        JPH::Vec3 mEndpointCorrection = JPH::Vec3::sZero();
        JPH::RVec3 mTargetFootOrigin = JPH::RVec3::sZero();
        bool mSaturated = false;
    };

    // Pure contact-space adapter (balance authority leg correction). It only
    // produces a pose endpoint; the caller must solve it through bounded articulated motors.
    std::optional<StanceContactPositionOutput> calculateStanceContactPositionTarget(
        const StanceContactPositionInput& input);

    enum class RecoveryDirection
    {
        FaceUp,
        FaceDown,
    };

    struct SettledPelvisHeightInput
    {
        SupportEstimate mSupport;
        ActiveControllerMode mMode = ActiveControllerMode::Standing;
        float mPelvisHeight = 0.0f;
        float mPelvisVerticalVelocity = 0.0f;
        bool mLocomotionRequested = false;
    };

    struct SettledPelvisHeightOutput
    {
        std::optional<float> mTargetHeight;
        float mCandidateTime = 0.0f;
        bool mCandidateValid = false;
        bool mSettled = false;
        bool mFrozen = false;
    };

    // Learns the supported physical standing height before locomotion begins.
    // This is shared by the game adapter and offline scenarios so neither can
    // silently tune upright control against a different insertion pose.
    class SettledPelvisHeightTracker
    {
    public:
        bool step(const SettledPelvisHeightInput& input, float duration,
            const ActiveControllerProfile& profile, SettledPelvisHeightOutput& output);
        void reset();
        SettledPelvisHeightOutput getOutput() const;

    private:
        std::optional<float> mTargetHeight;
        float mCandidateTime = 0.0f;
        bool mCandidateValid = false;
        bool mSettled = false;
        bool mFrozen = false;
    };

    struct ActiveControllerInput
    {
        SupportEstimate mSupport;
        float mPelvisUpDot = 1.0f;
        float mTorsoUpDot = 1.0f;
        // Chest-facing direction dotted with world up. Unlike the torso's
        // longitudinal up axis, this distinguishes supine from prone.
        float mTorsoFacingUpDot = 0.0f;
        float mPelvisHeight = 0.0f;
        JPH::Vec3 mCenterOfMassVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mTargetHorizontalVelocity = JPH::Vec3::sZero();
        bool mRecoveryRequested = false;
        // A gameplay hit is currently suppressing ordinary powered actuation.
        // Animation-assisted locomotion uses this to distinguish an authored
        // contact gap from a physical handoff; interrupted bodies receive no
        // synthetic support grace.
        bool mActuationInterrupted = false;
        // One-step notification that a physical hit landed while an authored
        // or settling recovery owned the body. The controller releases its
        // motors into Falling; a later request starts a clean retry.
        bool mRecoveryInterrupted = false;
        bool mRecoveryAnimationFinished = false;
        // Diagnostic override for which get-up is played. A fallen body comes
        // to rest on its SIDE - the torso reading was measured within about a
        // tenth of zero on every one of seven identical knockdowns - so the
        // sign test below is decided by which side of the boundary the noise
        // happens to fall on. Before changing how that choice is made we have
        // to know which get-up actually suits a body on its side, and the
        // only way to see that is to force each one and watch.
        std::optional<RecoveryDirection> mForcedRecoveryDirection;
        // The kinematic locomotion route holds the root externally while the
        // controller drives a recovery. The animation then owns the pelvis
        // path, so a recovery can be complete while the physical feet carry
        // no load at all - the measured-support requirement does not apply.
        bool mRootExternallyOwned = false;
        // Optional measured velocity of the actual owned root, from the same
        // completed-solve snapshot as mCenterOfMassVelocity. Only KinematicRoot
        // standing classification uses it; absence retains absolute vertical
        // error. A present value must be finite even on routes that ignore it.
        std::optional<float> mOwnedRootVerticalVelocity;
    };

    struct ActiveControllerOutput
    {
        ActiveControllerMode mMode = ActiveControllerMode::Standing;
        float mTimeInMode = 0.0f;
        float mFallingExitCandidateTime = 0.0f;
        bool mDriveJointMotors = true;
        bool mRecoveryHeightAssist = false;
        std::optional<RecoveryDirection> mRecoveryDirection;
    };

    class ActiveRagdollController
    {
    public:
        bool step(const ActiveControllerInput& input, float duration, const ActiveControllerProfile& profile,
            ActiveControllerOutput& output);
        void reset(ActiveControllerMode mode = ActiveControllerMode::Standing);
        ActiveControllerOutput getOutput() const;

    private:
        ActiveControllerMode mMode = ActiveControllerMode::Standing;
        float mTimeInMode = 0.0f;
        float mUnsupportedTime = 0.0f;
        bool mHasObservedFootSupport = false;
        float mFallingExitCandidateTime = 0.0f;
        float mRecoveryStandingTime = 0.0f;
        float mRecoveryStallTime = 0.0f;
        float mRecoveryProgressUp = -1.0f;
        std::optional<float> mRecoveryProgressDistance;
        bool mRecoveryHeightAssist = false;
        std::optional<RecoveryDirection> mRecoveryDirection;
    };
}

#endif
