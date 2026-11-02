#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_PASSIVERAGDOLL_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_PASSIVERAGDOLL_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Mat44.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <Jolt/Physics/EActivation.h>
#include <Jolt/Core/Reference.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
// clang-format on

#include <ragdollcore/activebody.hpp>
#include <ragdollcore/builder.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace JPH
{
    class SixDOFConstraint;
    class PhysicsSystem;
    class Ragdoll;
    class SkeletonPose;
}

namespace RagdollCore
{
    struct UprightControllerOutput;

    struct BodyMotion
    {
        JPH::Vec3 mLinearVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mAngularVelocity = JPH::Vec3::sZero();
    };

    struct MotionPose
    {
        Transform mRootTransform;
        float mUniformScale = 1.0f;
        std::vector<Transform> mBodyTransforms;
    };

    struct KillingImpulse
    {
        std::size_t mBodyIndex = 0;
        JPH::Vec3 mImpulse = JPH::Vec3::sZero();
        JPH::RVec3 mWorldPoint = JPH::RVec3::sZero();
    };

    struct PassiveRagdollInitialState
    {
        // Used for every body when mBodyMotions is empty. A full body-motion
        // array can carry velocities estimated from consecutive animation poses.
        // Initial linear velocities refer to body/bone origins, angular velocities
        // to world axes. Creation converts them to Jolt mass-center velocities.
        BodyMotion mActorMotion;
        std::vector<BodyMotion> mBodyMotions;
        std::optional<KillingImpulse> mKillingImpulse;
    };

    struct RagdollPose
    {
        // Model-space body transforms in profile body order. They use the same
        // unscaled reference frame as the SkeletonDescriptor passed at creation.
        std::vector<Transform> mBodyTransforms;
    };

    struct HingeLegGeometry
    {
        // Fixed creation geometry in the upper body's local axes, with lengths
        // in unscaled model units. The lower body's reference rotation is
        // relative to the upper body, independent of later animation poses.
        JPH::Vec3 mUpperOffset = JPH::Vec3::sZero();
        JPH::Vec3 mLowerOffset = JPH::Vec3::sZero();
        JPH::Quat mLowerRotation = JPH::Quat::sIdentity();
        JPH::Vec3 mHingeAxis = JPH::Vec3::sAxisX();
        float mMinimumAngle = 0.0f;
        float mMaximumAngle = 0.0f;
    };

    struct SwingTwistJointGeometry
    {
        JPH::Quat mParentFrame = JPH::Quat::sIdentity();
        JPH::Quat mChildFrame = JPH::Quat::sIdentity();
        float mNormalHalfConeAngle = 0;
        float mPlaneHalfConeAngle = 0;
        float mTwistMinimum = 0;
        float mTwistMaximum = 0;
        bool mPyramid = false;
    };

    // Pure native-limit query. Rotations share any common reference frame;
    // the returned child rotation includes Jolt's actual swing/twist clamp.
    std::optional<JPH::Quat> clampSwingTwistJointOrientation(const SwingTwistJointGeometry& geometry,
        JPH::QuatArg parentRotation, JPH::QuatArg childRotation);

    struct ConstraintTelemetry
    {
        std::size_t mParentBodyIndex = 0;
        std::size_t mChildBodyIndex = 0;
        float mAnchorSeparation = 0.0f;
        std::optional<float> mHingeAngle;
        std::optional<float> mHingeMinimum;
        std::optional<float> mHingeMaximum;
        // World-space child anchor, independently of the body's COM/origin.
        JPH::RVec3 mPosition = JPH::RVec3::sZero();
    };

    struct BodyTelemetry
    {
        JPH::RVec3 mPosition = JPH::RVec3::sZero();
        JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
        JPH::Quat mRotation = JPH::Quat::sIdentity();
        BodyMotion mMotion;
        float mMass = 0.0f;
        bool mActive = false;
    };

    struct RagdollTelemetry
    {
        JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
        float mTotalMass = 0.0f;
        std::vector<BodyTelemetry> mBodies;
        std::vector<ConstraintTelemetry> mConstraints;
    };

    struct RagdollMomentum
    {
        float mTotalMass = 0.0f;
        JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
        JPH::Vec3 mLinearMomentum = JPH::Vec3::sZero();
        // Spin plus orbital momentum about the whole-ragdoll COM, using
        // Jolt's actual (potentially stabilized) body inertia tensors.
        JPH::Vec3 mAngularMomentum = JPH::Vec3::sZero();
    };

    struct RagdollSolverImpulse
    {
        JPH::RVec3 mReferenceCenterOfMass = JPH::RVec3::sZero();
        JPH::Vec3 mLinearImpulse = JPH::Vec3::sZero();
        JPH::Vec3 mAngularImpulse = JPH::Vec3::sZero();
        JPH::Vec3 mInternalAnchorImpulseMoment = JPH::Vec3::sZero();
        // Momentum removed/added by free integration velocity caps, measured
        // separately from the subsequent solver's external impulse. Balanced
        // submitted wrenches do not guarantee balanced capped integration.
        JPH::Vec3 mFreeLinearLimitImpulse = JPH::Vec3::sZero();
        JPH::Vec3 mFreeAngularLimitImpulse = JPH::Vec3::sZero();
        bool mFreeVelocityLimitTouched = false;
        bool mVelocityLimitTouched = false;
        bool mActivationChanged = false;
        // CCD can solve additional impulses after body rotation. The angular
        // reconstruction uses pre-solve inertia and cannot separate them.
        bool mContinuousCollisionEnabled = false;
    };

    // Requires external solver interactions to be contacts on this horizontal
    // plane, with no externally attached joint. Returns no point when impulse
    // reconstruction can be affected by velocity limits, activation changes,
    // or CCD. The caller must also exclude external CCD bodies and mass scaling.
    std::optional<JPH::RVec3> estimateHorizontalPlaneCenterOfPressure(
        const RagdollSolverImpulse& impulse, double planeHeight);

    struct MotorSpringTelemetry
    {
        bool mStiffnessMode = false;
        float mFrequencyOrStiffness = 0.0f;
        float mDamping = 0.0f;
    };

    struct MotorDriveTelemetry
    {
        int mSwingOrHingeState = 0;
        int mTwistState = 0;
        JPH::Vec3 mMinimumTorque = JPH::Vec3::sZero();
        JPH::Vec3 mMaximumTorque = JPH::Vec3::sZero();
        MotorSpringTelemetry mSwingOrHingeSpring;
        MotorSpringTelemetry mTwistSpring;
    };

    struct MotorTargetTelemetry
    {
        // Available even when there is no position target (capture returns
        // false). This distinguishes an off/velocity motor from missing data.
        std::optional<MotorDriveTelemetry> mDrive;
        ConstraintType mConstraintType = ConstraintType::SwingTwist;
        bool mWorldOrientation = false;
        MotorSpringTelemetry mSpring;
        std::optional<MotorSpringTelemetry> mTwistSpring;
        // Swing-twist constraints expose their complete orientation in
        // constraint space, or world space when mWorldOrientation is true.
        // Hinge constraints instead populate the scalar
        // angle fields below.
        std::optional<JPH::Quat> mCurrentOrientation;
        std::optional<JPH::Quat> mRequestedOrientation;
        std::optional<JPH::Quat> mTargetOrientation;
        // Instantaneous orientation satisfying the final target while the other
        // body is held fixed. Diagnostic geometry, not a prediction of motion.
        std::optional<JPH::Quat> mChildWorldTarget;
        std::optional<JPH::Quat> mParentWorldTarget;
        std::optional<float> mCurrentHingeAngle;
        // Scalar angle requested at the motor boundary, before Jolt's limit
        // clamp and any subsequent target-step limiter. This is not the raw
        // animation clip angle. Absent when no request was recorded.
        std::optional<float> mRequestedHingeAngle;
        std::optional<float> mTargetHingeAngle;
        std::optional<float> mHingeMinimum;
        std::optional<float> mHingeMaximum;
        std::optional<float> mNormalHalfConeAngle;
        std::optional<float> mPlaneHalfConeAngle;
        std::optional<float> mTwistMinimum;
        std::optional<float> mTwistMaximum;
        // World axes, radians/second. Relative child-minus-parent velocity
        // for joint-space targets; absolute child velocity for world targets.
        JPH::Vec3 mTargetAngularVelocity = JPH::Vec3::sZero();
        // Shortest angular distance from the measured constraint state to the
        // target Jolt will solve during the next physics update.
        float mAngularError = 0.0f;
        // Angular distance between the recorded request and final motor
        // target, including joint limits and any target-step limiter. For a
        // hinge this is meaningful only when mRequestedHingeAngle is present.
        float mTargetClampDelta = 0.0f;
    };

    struct ConstraintImpulse
    {
        float mLinear = 0.0f;
        float mAngular = 0.0f;
    };

    class PassiveRagdoll
    {
    public:
        ~PassiveRagdoll();

        PassiveRagdoll(const PassiveRagdoll&) = delete;
        PassiveRagdoll& operator=(const PassiveRagdoll&) = delete;
        PassiveRagdoll(PassiveRagdoll&&) = delete;
        PassiveRagdoll& operator=(PassiveRagdoll&&) = delete;

        // The caller must serialize these operations with PhysicsSystem::Update.
        // A prepared ragdoll owns bodies but has not exposed them to broadphase.
        bool addToPhysicsSystem(JPH::EActivation activation = JPH::EActivation::Activate);
        void removeFromPhysicsSystem();

        bool isAdded() const { return mAdded; }
        bool isActive() const;
        std::size_t getBodyCount() const;
        JPH::BodyID getBodyId(std::size_t index) const;
        // PLAY-01 architecture base: one declared kinematic root for
        // locomotion. While enabled the caller owns the root transform and
        // must call moveRootKinematic once per fixed step; every other body
        // stays dynamic. Releasing restores dynamic motion with the supplied
        // velocity so a fall or hit handover keeps its momentum.
        bool setRootKinematic(bool kinematic);
        bool moveRootKinematic(JPH::RVec3Arg position, JPH::QuatArg rotation, float duration);
        bool releaseRootKinematic(JPH::Vec3Arg velocity);
        bool isRootKinematic() const { return mRootKinematic; }

        // Soft root: a held root stays a dynamic body and a capped spring
        // pulls it toward each moveRootKinematic target instead of moving it
        // there. isRootKinematic() still reports "held" so the rest of the
        // controller is unchanged. Forces are in multiples of body weight
        // (total ragdoll mass x gravity), torques in body weight x
        // mGyrationRadius.
        struct SoftRootSettings
        {
            bool mEnabled = false;
            ActiveBodySettings mBody;
        };
        void setSoftRoot(const SoftRootSettings& settings);
        const SoftRootSettings& getSoftRoot() const { return mSoftRoot; }
        // 0..1: scales caps and gravity support (hits lower it, 0 = limp root).
        void setSoftRootStrength(float strength);
        float getSoftRootStrength() const { return mSoftRootStrength; }
        // Scales the soft root's horizontal hold only (catch steps).
        void setSoftRootHorizontalScale(float scale) { mSoftRootHorizontalScale = std::clamp(scale, 0.0f, 1.0f); }
        // Caps the pelvis anchor's spring frequency (0 = no cap): getting up,
        // a stiff anchor snapped the lying pelvis to the clip and flipped it.
        void setSoftAnchorFrequencyCap(float cap) { mSoftAnchorFrequencyCap = cap; }
        // 0..1: fraction of weight the feet can pass to the ground (0 in the
        // air, so a held body still falls when nothing supports it).
        void setSoftRootSupport(float support);
        // A jump: the character controller launches the capsule, so for the
        // jump window the body rides with it - full support from the root, no
        // weight routed through the (leaving) feet, upward cap x upCapScale.
        // Target motion faster than teleportSpeed reads as a teleport (no
        // velocity feed-forward); a jump may launch faster than walking does.
        void setSoftRootJump(bool jumping, float upCapScale, float teleportSpeed = 2000.0f)
        {
            mSoftRootJumping = jumping;
            mSoftRootJumpCap = upCapScale;
            mSoftRootJumpTeleportSpeed = teleportSpeed;
        }
        // Part of the target's velocity that is not locomotion (the capsule
        // following the body). It is left out of the velocity feed-forward;
        // feeding it forward made body and capsule chase each other.
        void setSoftRootVelocityBias(JPH::Vec3Arg bias) { mSoftRootVelocityBias = bias; }
        // Bodies that carry the soft-root correction: upper (translation and
        // weight) and torso (rotation), indexed like the ragdoll bodies.
        void setSoftRootBodies(std::vector<bool> upper, std::vector<bool> torso, std::vector<bool> feet);
        // Per body: a foot currently in ground contact (for ground routing).
        void setSoftRootGrounded(std::size_t bodyIndex, bool grounded);
        // Object layer for the soft-root anchor body (must collide with nothing).
        void setSoftRootAnchorLayer(JPH::ObjectLayer layer) { mSoftAnchorLayer = layer; }
        // Adds the same velocity to every body (a whole-body shove).
        bool addLinearVelocity(JPH::Vec3Arg velocity);
        // The same shove's momentum spread by a per-body share (missing = 1):
        // body i gets velocity * scale[i] * (total mass / sum of mass * scale).
        bool addLinearVelocity(JPH::Vec3Arg velocity, const std::vector<float>& scale);
        // Moves one dynamic body to a world transform, moving with `velocityFrom`
        // (another body of this rig: its linear velocity, no spin). For
        // re-seating a limb chain whose joints wound up (arm reseat on draw).
        bool setBodyWorldTransform(std::size_t bodyIndex, JPH::RVec3Arg position, JPH::QuatArg rotation,
            std::size_t velocityFrom);
        struct SoftRootTelemetry
        {
            JPH::Vec3 mForce = JPH::Vec3::sZero();
            JPH::Vec3 mTorque = JPH::Vec3::sZero();
            float mPositionError = 0.0f;
            float mAngleError = 0.0f;
            bool mSaturated = false;
        };
        const SoftRootTelemetry& getSoftRootTelemetry() const { return mSoftRootTelemetry; }
        // Scale one body's contact friction relative to its authored value
        // (0..1). A held root does not need its swing foot to grip: a foot
        // that scuffs the floor mid-swing must slide on instead of being
        // pinned under the body. One restores the authored friction.
        bool setBodyFrictionScale(std::size_t bodyIndex, float scale);
        // Rescales a body to this mass (inertia with it), e.g. a sheathed weapon.
        bool setBodyMass(std::size_t bodyIndex, float mass);
        const std::vector<float>& getBodyMasses() const { return mBodyMasses; }
        // Narrow self-collision override for two distinct bodies of this rig.
        // Call outside PhysicsSystem::Update/contact callbacks. The builder's
        // canonical subgroup layout is required. A changed table is copied
        // before assigning it to this rig's bodies; settings/other instances
        // retain their original filter and every group/subgroup ID is kept.
        // A successful no-op returns true. Restore the queried prior value
        // when the owning action ends rather than assuming it was enabled.
        bool setBodyPairCollisionEnabled(std::size_t first, std::size_t second, bool enabled);
        std::optional<bool> isBodyPairCollisionEnabled(std::size_t first, std::size_t second) const;
        bool captureModelPose(RagdollPose& out);
        // Returns no geometry for a non-hinge chain or inconsistent bind axes.
        std::optional<HingeLegGeometry> getHingeLegGeometry(
            std::size_t hip, std::size_t knee, std::size_t foot) const;
        // Read-only signed hinge coordinate of a supplied model-space pose,
        // before joint limits. Uses the same skeletal-parent conversion and
        // Jolt local frames as driveToPoseUsingMotors; not an unsigned bend.
        // Does not activate bodies, replace motor scratch or set any target.
        std::optional<float> getPoseHingeAngle(const RagdollPose& pose, std::size_t bodyIndex) const;
        bool captureTelemetry(RagdollTelemetry& out) const;
        // Instantaneous momentum about the whole-ragdoll COM, using actual
        // Jolt mass/inertia and both body spin and orbital momentum. Unlike
        // impulse reconstruction, this observation also supports CCD states.
        bool captureMomentum(RagdollMomentum& out) const;

        // Read-only observation around exactly one fixed step with one Jolt
        // collision step. Predicts free velocities on copies of Jolt motion
        // properties, then removes internal joint-anchor impulse moments.
        // Remaining impulses come from external solver interactions (contacts
        // or externally attached constraints). Velocity caps, activation
        // changes or CCD make the reconstruction unsuitable for CoP inference.
        // Call after all actuation, immediately around PhysicsSystem::Update,
        // serialized with it and before any post-solve velocity correction.
        bool beginSolverImpulseMeasurement(float fixedTimeStep);
        bool endSolverImpulseMeasurement(float fixedTimeStep, RagdollSolverImpulse& out) const;
        // Conservative height of this body's COM above a horizontal floor
        // touching the lowest collision-shape bound in the supplied pose.
        // Read-only, includes existing shape scale and COM offsets. Does not
        // establish self-collision clearance, support or trajectory feasibility.
        std::optional<float> getPoseFloorHeight(const RagdollPose& pose, std::size_t bodyIndex) const;
        // World Z of this body's lowest collision bound in the supplied pose.
        // Used by the kinematic root to ground the lowest foot sole.
        std::optional<float> getPoseBodyFloorHeight(const RagdollPose& pose, std::size_t bodyIndex) const;
        std::optional<JPH::RVec3> getWorldBodyPosition(const RagdollPose& pose, std::size_t bodyIndex) const;
        std::optional<JPH::Quat> getWorldBodyRotation(const RagdollPose& pose, std::size_t bodyIndex) const;
        std::optional<JPH::Vec3> getWorldVector(JPH::Vec3Arg modelVector) const;
        std::optional<JPH::Vec3> getModelVector(JPH::Vec3Arg worldVector) const;
        std::optional<JPH::Quat> getModelRotation(JPH::QuatArg worldRotation) const;
        std::optional<JPH::Vec3> getModelPosition(JPH::RVec3Arg worldPosition) const;
        // Re-expresses one cached model-space point in a prospective actor
        // reference frame without changing the ragdoll. The represented world
        // point is preserved, including uniform-scale conversion.
        std::optional<JPH::Vec3> rebaseModelPosition(JPH::Vec3Arg modelPosition,
            JPH::RVec3Arg referencePosition, JPH::QuatArg referenceRotation) const;
        JPH::RVec3 getReferencePosition() const { return mReferencePosition; }
        // The model frame exactly: model = inverseRotation * (world - position) * inverseScale.
        JPH::Quat getInverseReferenceRotation() const { return mInverseReferenceRotation; }
        float getInverseUniformScale() const { return mInverseUniformScale; }

        // Changes the model/world conversion frame without moving any Jolt
        // bodies. Cached physics poses must first be rebased so they continue
        // to describe the same world transforms under the new actor root.
        bool rebaseModelPose(RagdollPose& pose, JPH::RVec3Arg referencePosition,
            JPH::QuatArg referenceRotation) const;
        bool setReferenceTransform(JPH::RVec3Arg referencePosition, JPH::QuatArg referenceRotation);

        // Enables the configured bounded Jolt position motors and targets the
        // supplied model-space pose. The caller must invoke this before a fixed
        // physics step and serialize it with PhysicsSystem::Update.
        // Body indices in disabledMotorBodies keep their joint motor off
        // after the remaining constraints are targeted. This lets a
        // locomotion controller reserve the stance hip for balance torques.
        // Invalid or duplicate indices reject the complete update.
        bool driveToPoseUsingMotors(
            const RagdollPose& target, std::span<const std::size_t> disabledMotorBodies = {});
        // Sets a moving joint-space pose's relative angular velocity in world
        // axes. Hinge targets project onto the hinge axis. Call after pose drive;
        // a new pose drive or motor release clears the velocity. Rejects disabled
        // motors and world-orientation overrides without changing their state.
        bool setMotorTargetAngularVelocity(std::size_t bodyIndex, JPH::Vec3Arg relativeWorldVelocity);
        // Captures the post-limit motor target and the measured constraint
        // state. Call after driveToPoseUsingMotors (and any target limiter)
        // and before PhysicsSystem::Update when diagnosing an actuation step.
        bool captureMotorTargetTelemetry(std::size_t bodyIndex, MotorTargetTelemetry& out) const;
        std::optional<SwingTwistJointGeometry> getSwingTwistJointGeometry(std::size_t bodyIndex) const;
        // Widen or restore a swing-twist joint's limits while the simulation
        // runs. The authored shoulder cone is about 49 degrees, which suits a
        // corpse and cannot reach a point in front of the chest; a swing
        // needs far more, but only for as long as it lasts. Applying it
        // globally was measured to destabilize walking, bracing and the stop,
        // because looser shoulders change the whole body's angular momentum.
        // False when the body has no swing-twist constraint.
        // A hinge joint's limits (radians, in the constraint's own angle).
        bool setHingeJointLimits(std::size_t bodyIndex, float minimum, float maximum);
        std::optional<std::pair<float, float>> getHingeJointLimits(std::size_t bodyIndex) const;
        bool setSwingTwistJointLimits(
            std::size_t bodyIndex, float normalHalfCone, float planeHalfCone, float twistMin, float twistMax);
        // Clamps one already-driven joint target relative to the constraint's
        // measured orientation. Call after driveToPoseUsingMotors and before
        // PhysicsSystem::Update.
        bool limitMotorTargetStep(std::size_t bodyIndex, float maximumAngularStep);
        // Replaces the symmetric torque limit for one configured joint motor.
        // This is used to give the current swing leg bounded extra authority
        // without changing the planted leg. Invalid input leaves the motor
        // unchanged.
        bool setMotorMaximumTorque(std::size_t bodyIndex, float maximumTorque);
        // Diagnostics (2026-10-04, the limp arm in play): a joint motor's
        // torque limit, spring, state (0 off, 1 velocity, 2 position; swing
        // motor for swing-twist) and its angle from its own target, radians.
        struct MotorDebug
        {
            float mMaximumTorque = 0, mFrequency = 0, mDamping = 0, mTargetError = 0;
            int mState = -1;
        };
        std::optional<MotorDebug> getMotorDebug(std::size_t bodyIndex) const;
        // Exact reversible limits in the MotorDriveTelemetry axis convention:
        // hinge X; swing-twist X=twist, Y=Z=swing. No other motor state changes.
        bool setMotorTorqueLimits(std::size_t bodyIndex, JPH::Vec3Arg minimum, JPH::Vec3Arg maximum);
        // Replaces the spring frequency and damping ratio of one configured
        // joint motor (swing and twist share one setting). Torque limits are
        // untouched. Invalid input leaves the motor unchanged.
        bool setMotorSpring(std::size_t bodyIndex, float frequency, float damping);

        // Keeps a loaded articulated chain on an explicitly qualified velocity
        // iteration count while powered pose drive is active. Zero restores
        // the PhysicsSystem default. This is per-ragdoll and therefore does
        // not raise the solver cost of unrelated actors.
        void setPoweredVelocitySolverSteps(std::uint32_t steps);
        // Preserve anchor closure through held-root release and reacquisition.
        // Zero uses the world default; this budget is local to the ragdoll.
        void setPoweredPositionSolverSteps(std::uint32_t steps);
        // Constraints with a greater priority are solved after lower-priority
        // joints. Use this for terminal equipment grips that must close after
        // the arm chain has moved.
        bool setConstraintPriority(std::size_t bodyIndex, std::uint32_t priority);
        bool setConstraintSolverSteps(
            std::size_t bodyIndex, std::uint32_t velocitySteps, std::uint32_t positionSteps);

        // Observes the impulse produced by one swing/twist joint in the most
        // recent solve. Values use Jolt's native linear and angular impulse
        // units. This is sufficient for a finite break threshold without
        // exposing the underlying constraint object to the game adapter.
        bool getConstraintImpulse(std::size_t bodyIndex, ConstraintImpulse& out) const;
        // Disables the constraint that owns bodyIndex. A present true value
        // means this call performed the enabled-to-disabled transition; false
        // means it was already disabled. Invalid/root bodies return no value.
        std::optional<bool> releaseConstraint(std::size_t bodyIndex);
        // Enables a released joint again (GRIP-05: the murder-stroke sets the
        // weapon's grip joint aside while another constraint holds it).
        std::optional<bool> restoreConstraint(std::size_t bodyIndex);
        std::optional<bool> isConstraintEnabled(std::size_t bodyIndex) const;

        // Replaces this step's swing/twist pose motor with a bounded internal
        // torque servo on the plane perpendicular to freeWorldAxis. The free
        // axis receives neither pose stiffness nor velocity damping. Uses the
        // configured motor frequency, damping and torque cap; limits remain
        // enforced by the native joint. Call once after driving the pose.
        bool driveMotorWithFreeRotationAxis(std::size_t bodyIndex,
            JPH::Vec3Arg freeWorldAxis, float fixedTimeStep);
        // Controls the child orientation in world space. Damping observes the
        // child's world angular velocity, so moving its parent does not brake
        // an already stationary foot. Torque still reacts on the parent and
        // respects the existing motor profile and anatomical joint limits.
        // The world target is projected through Jolt's native angular target clamp.
        bool driveMotorToWorldOrientation(std::size_t bodyIndex,
            JPH::QuatArg targetWorldRotation, float fixedTimeStep);
        // Temporary A109 matched pose intervention; removed after experiment.
        bool diagnosticOverrideMotorPose(std::size_t bodyIndex, JPH::QuatArg parentRelative, float hingeAngle,
            const std::optional<JPH::Vec3>& velocity = std::nullopt);
        void disableMotors();

        // Adds a world-space load request to the existing joint pose spring,
        // bounds their SUM by the native per-axis motor limits, and applies an
        // internal torque pair through solver motors. Supports hinges and
        // swing/twist joints; call once after driving the pose each fixed step.
        // A swing/twist motor can instead release one pose-control axis or
        // track a world orientation. Load compensation survives either choice.
        // Only the free-axis servo scales the combined torque uniformly to
        // preserve its direction. Pose/world targets use native axis clamping.
        // An optional angular velocity target feeds the position motor's
        // damping: the child's absolute world rate with a world target,
        // otherwise the child-minus-parent relative world rate.
        // Adds to the native load torque a driveMotorWithLoadTorque already set
        // on the joint this step (a second writer, e.g. the weapon's weight on
        // a joint the joint support drove); false when none was set.
        bool addMotorLoadTorque(std::size_t bodyIndex, JPH::Vec3Arg extra);
        bool driveMotorWithLoadTorque(std::size_t bodyIndex, JPH::Vec3Arg loadTorque, float fixedTimeStep,
            const std::optional<JPH::Vec3>& freeWorldAxis = std::nullopt,
            const std::optional<JPH::Quat>& targetWorldRotation = std::nullopt,
            const std::optional<JPH::Vec3>& targetWorldAngularVelocity = std::nullopt);

        // Clear the previous step's internal wrench commands before applying
        // any forces or pose servos. Solver actuation uses free SixDOF motor
        // axes with equal minimum/maximum force limits; it adds no position
        // constraints and avoids pre-solver per-body velocity clipping.
        void beginActuationStep(bool useSolverWrenches);

        // Reads the world-space angular impulse applied to the child body by
        // its joint motor during the most recent physics step and the largest
        // absolute constraint-axis impulse divided by that axis's configured
        // limit. The root, invalid timestep, and invalid body indices are
        // rejected without changing either output. Native axes are captured
        // when driving the pose; read after one collision step, before driving
        // the next pose. Post-solve body rotations are not solver motor axes.
        bool getMotorAngularImpulse(std::size_t bodyIndex, float fixedTimeStep,
            JPH::Vec3& outImpulse, float& outPeakAxisUtilization) const;

        // Applies a previously computed controller correction after validating
        // every body index, vector, and profile limit. Translation is an
        // internal force pairs. Vertical reaction goes to planted support
        // bodies; horizontal reaction may be routed through a stance leg so
        // its constraints transmit force to ground contact. A force component
        // with no corresponding reaction body is suppressed. Validation is
        // transactional: no force or torque is submitted on failure. Gait can
        // center the force on the pelvis to keep its commanded wrench intact;
        // standing/recovery retain the symmetric midpoint distribution.
        // A secondary angular reaction shares the same total reaction equally
        // with the primary. It requires a distinct, valid primary body.
        bool applyUprightControl(const UprightControllerOutput& control, std::size_t pelvisBodyIndex,
            std::size_t spineBodyIndex, std::span<const std::size_t> verticalReactionBodyIndices,
            std::span<const std::size_t> horizontalReactionBodyIndices,
            const ActiveControllerProfile& profile,
            std::optional<std::size_t> angularReactionBodyIndex = std::nullopt,
            bool centerPelvisForce = false,
            std::optional<std::size_t> secondaryAngularReactionBodyIndex = std::nullopt);

        // Applies the explicitly external Route-B pelvis wrench at the body's
        // center of mass. Unlike applyUprightControl this intentionally has
        // no ragdoll reaction body: the caller must expose and account for it
        // as assistance. Invalid input is rejected before physics is mutated.
        bool applyExternalPelvisAssist(
            std::size_t pelvisBodyIndex, JPH::Vec3Arg force, float maximumForce,
            JPH::Vec3Arg torque, float maximumTorque);

        // Applies a force and torque at one body's centre of mass with no
        // reaction body. For a wrench whose reaction belongs to a kinematic
        // root (the ground carries it); the caller bounds the magnitudes.
        bool applyExternalBodyWrench(std::size_t bodyIndex, JPH::Vec3Arg force, JPH::Vec3Arg torque);

        // Applies equal and opposite forces at one shared world point midway
        // between the body centers of mass. Submitted net force and moment
        // are zero; the legacy AddForce path is subject to per-body clipping.
        bool applyBodyForcePair(std::size_t drivenBodyIndex, std::size_t reactionBodyIndex,
            JPH::Vec3Arg force, float maximumForce);

        // Applies equal-and-opposite world-space torques to two bodies without
        // changing whole-ragdoll angular momentum.
        bool applyBodyTorquePair(std::size_t drivenBodyIndex, std::size_t reactionBodyIndex,
            JPH::Vec3Arg torque, float maximumTorque);

        // Applies an equal-and-opposite angular impulse to two bodies. This
        // transfers an internal joint reaction without adding whole-ragdoll
        // angular momentum.
        bool applyBodyAngularImpulsePair(std::size_t drivenBodyIndex, std::size_t reactionBodyIndex,
            JPH::Vec3Arg angularImpulse, float maximumAngularImpulse);

    private:
        struct ForceMotor;
        std::vector<std::unique_ptr<ForceMotor>> mForceMotors;
        bool mUseSolverWrenches = false;
        void clearForceMotors();
        void applySolverWrench(std::size_t driven, std::size_t reaction,
            JPH::Vec3Arg force, JPH::Vec3Arg torque);
        friend struct PassiveRagdollCreateResult;
        friend PassiveRagdollCreateResult createPassiveRagdoll(JPH::PhysicsSystem&, const SkeletonDescriptor&,
            const RagdollProfile&, const BuildOptions&, const PassiveRagdollInitialState&, JPH::CollisionGroup::GroupID,
            std::uint64_t);

        PassiveRagdoll(JPH::PhysicsSystem& physicsSystem, std::unique_ptr<JPH::Ragdoll> ragdoll,
            JPH::RVec3Arg referencePosition, JPH::QuatArg referenceRotation, float uniformScale,
            std::vector<float> bodyMasses, std::optional<KillingImpulse> pendingKillingImpulse);

        JPH::PhysicsSystem* mPhysicsSystem;
        bool mRootKinematic = false;
        bool applySoftRootWrench(JPH::RVec3Arg position, JPH::QuatArg rotation, float duration);
        SoftRootSettings mSoftRoot;
        float mSoftAnchorFrequencyCap = 0.0f;
        float mSoftRootStrength = 1.0f;
        float mSoftRootHorizontalScale = 1.0f;
        float mSoftRootSupport = 1.0f;
        std::optional<JPH::RVec3> mSoftRootPreviousPosition;
        std::optional<JPH::Quat> mSoftRootPreviousRotation;
        SoftRootTelemetry mSoftRootTelemetry;
        JPH::Vec3 mSoftRootVelocityBias = JPH::Vec3::sZero();
        std::vector<bool> mSoftRootUpper;
        std::vector<bool> mSoftRootTorso;
        std::vector<bool> mSoftRootFeet;
        std::vector<bool> mSoftRootGrounded;
        float mSoftRootAirborneTime = 0.0f;
        bool mSoftRootJumping = false;
        float mSoftRootJumpCap = 1.0f;
        float mSoftRootJumpTeleportSpeed = 2000.0f;
        void destroySoftAnchor();
        bool driveSoftAnchor(JPH::RVec3Arg position, JPH::QuatArg rotation, float duration, float torqueLimit);
        std::optional<JPH::ObjectLayer> mSoftAnchorLayer;
        JPH::BodyID mSoftAnchor;
        JPH::Ref<JPH::SixDOFConstraint> mSoftAnchorConstraint;
        std::unique_ptr<JPH::Ragdoll> mRagdoll;
        JPH::RVec3 mReferencePosition;
        JPH::Quat mInverseReferenceRotation;
        float mInverseUniformScale;
        std::vector<float> mBodyMasses;
        std::optional<KillingImpulse> mPendingKillingImpulse;
        std::vector<JPH::Mat44> mPoseScratch;
        std::unique_ptr<JPH::SkeletonPose> mMotorPose;
        // Observations only: preserve the raw scalar through target limiters
        // and refresh it when the next pose or explicit override is submitted.
        std::vector<std::optional<float>> mRequestedHingeAngles;
        bool driveMotorWithFreeAxisServo(std::size_t bodyIndex, float fixedTimeStep,
            JPH::Vec3Arg freeWorldAxis, const std::optional<JPH::Vec3>& loadTorque = std::nullopt);
        struct TorqueMotorImpulse
        {
            std::size_t mBodyIndex;
            JPH::Vec3 mImpulse;
            float mDuration;
            float mUtilization;
            std::optional<JPH::Quat> mWorldTarget;
            bool mJointSpaceTarget = false;
            bool mNativeLoad = false;
            // Present only when a world target was changed by anatomical projection.
            std::optional<JPH::Quat> mRequestedWorldTarget;
            // The native load torque set on the joint this step.
            JPH::Vec3 mLoadTorque = JPH::Vec3::sZero();
        };
        std::vector<TorqueMotorImpulse> mTorqueMotorImpulses;
        std::vector<std::array<JPH::Vec3, 3>> mMotorImpulseWorldAxes;
        std::uint32_t mPoweredVelocitySolverSteps = 0;
        std::uint32_t mPoweredPositionSolverSteps = 0;
        struct SolverBodySnapshot
        {
            JPH::RVec3 mCenterOfMass;
            JPH::Quat mInertiaRotation;
            JPH::Vec3 mInertiaDiagonal;
            BodyMotion mFreeMotion;
            BodyMotion mUnclampedMotion;
            float mMass;
            bool mActive;
        };
        struct SolverImpulseSnapshot
        {
            float mDuration;
            JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
            std::vector<SolverBodySnapshot> mBodies;
            std::vector<JPH::Vec3> mAnchorDifferences;
            bool mContinuousCollisionEnabled = false;
        };
        std::optional<SolverImpulseSnapshot> mSolverImpulseSnapshot;
        bool mAdded = false;
    };

    struct PassiveRagdollCreateResult
    {
        std::unique_ptr<PassiveRagdoll> mRagdoll;
        ValidationResult mValidation;

        bool isValid() const { return mRagdoll != nullptr && mValidation.isValid(); }
    };

    // Creates and initializes a prepared ragdoll. No body enters broadphase
    // until addToPhysicsSystem succeeds, allowing the OpenMW adapter to keep its
    // ordinary actor collider enabled throughout validation and allocation.
    PassiveRagdollCreateResult createPassiveRagdoll(JPH::PhysicsSystem& physicsSystem,
        const SkeletonDescriptor& skeleton, const RagdollProfile& profile, const BuildOptions& buildOptions,
        const PassiveRagdollInitialState& initialState, JPH::CollisionGroup::GroupID collisionGroup,
        std::uint64_t userData);

    // Estimates world-space velocities from two evaluated animation poses.
    // Returns no value for invalid, mismatched, or stale samples. Each result
    // is clamped independently so a discontinuity cannot destabilize Jolt.
    std::optional<std::vector<BodyMotion>> estimateBodyMotions(
        const MotionPose& previous, const MotionPose& current, float duration, const MotionTransferProfile& profile);

    // Produces a render-only pose between two authoritative fixed-step physics
    // poses. The result must never be reused as a simulation or animation target.
    bool interpolateRagdollPoses(
        const RagdollPose& previous, const RagdollPose& current, float factor, RagdollPose& out);

    // Converts an authoritative gameplay hit into a bounded Jolt impulse and
    // targets the profile body whose initial bone origin is nearest the hit.
    std::optional<KillingImpulse> makeKillingImpulse(const SkeletonDescriptor& skeleton, const RagdollProfile& profile,
        const BuildOptions& buildOptions, JPH::RVec3Arg worldPoint, JPH::Vec3Arg direction, float healthDamage);
}

#endif
