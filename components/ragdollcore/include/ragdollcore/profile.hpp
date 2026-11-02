#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_PROFILE_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_PROFILE_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace RagdollCore
{
    inline constexpr std::uint32_t CurrentProfileSchemaVersion = 1;

    struct Transform
    {
        JPH::Vec3 mTranslation = JPH::Vec3::sZero();
        JPH::Quat mRotation = JPH::Quat::sIdentity();
    };

    struct SkeletonBone
    {
        std::string mName;
        int mParent = -1;
        Transform mModelTransform;
    };

    struct SkeletonDescriptor
    {
        std::string mId;
        std::vector<SkeletonBone> mBones;
    };

    enum class ShapeType
    {
        Box,
        Capsule,
        Sphere,
        ConvexHull,
    };

    struct ShapeProfile
    {
        ShapeType mType = ShapeType::Capsule;
        Transform mLocalTransform;
        // Derive the capsule's orientation and center from this body's bone
        // to the named descendant. Preserve the authored radius and shorten
        // the cylindrical section, to zero when the span is too short.
        std::optional<std::string> mFitToBone;
        JPH::Vec3 mHalfExtents = JPH::Vec3::sReplicate(1.0f);
        float mRadius = 1.0f;
        float mHalfHeight = 1.0f;
        // Explicit points in the shape's local frame, before mLocalTransform
        // and BuildOptions::mUniformScale. Interior/duplicate points are valid;
        // the set must span a finite three-dimensional volume.
        std::vector<JPH::Vec3> mPoints;
    };

    struct MaterialProfile
    {
        float mFriction = 0.8f;
        float mRestitution = 0.0f;
        float mLinearDamping = 0.05f;
        float mAngularDamping = 0.05f;
    };

    enum class ConstraintType
    {
        SwingTwist,
        Hinge,
    };

    struct MotorProfile
    {
        // Jolt's position motor is a spring-damper (quaternion for swing/twist
        // constraints). A zero torque limit keeps the joint passive.
        float mFrequency = 4.0f;
        float mDamping = 1.0f;
        float mMaximumTorque = 0.0f;
        // Runtime only, never authored: Jolt sizes this spring from the two
        // jointed bodies alone. On a joint that carries more than its child (a
        // held weapon) the square root of carried over paired inertia is
        // folded into mFrequency and mDamping above. Where the weapon
        // attachment also records it here, a caller that substitutes its own
        // spring is expected to allow for it; one means "do not".
        float mLoadScale = 1.0f;
    };

    struct ConstraintProfile
    {
        ConstraintType mType = ConstraintType::SwingTwist;
        // Derive a coincident joint anchor and orthonormal frame from the
        // parent/child bone model transforms at build time. Explicit anchor
        // and axis fields remain available for authored constraint frames.
        bool mDeriveFrameFromSkeleton = false;
        // Optional descendant bone used to derive the anatomical twist axis
        // from the child toward that descendant. The joint anchor remains at
        // the child bone. This is important for offset sockets such as hips,
        // where parent-to-child points sideways rather than down the limb.
        std::optional<std::string> mFrameAxisBone;
        JPH::Vec3 mParentAnchor = JPH::Vec3::sZero();
        JPH::Vec3 mChildAnchor = JPH::Vec3::sZero();
        JPH::Vec3 mParentTwistAxis = JPH::Vec3::sAxisX();
        JPH::Vec3 mParentPlaneAxis = JPH::Vec3::sAxisY();
        JPH::Vec3 mChildTwistAxis = JPH::Vec3::sAxisX();
        JPH::Vec3 mChildPlaneAxis = JPH::Vec3::sAxisY();
        float mNormalHalfConeAngle = 0.5f;
        float mPlaneHalfConeAngle = 0.5f;
        float mTwistMinAngle = -0.25f;
        float mTwistMaxAngle = 0.25f;
        float mHingeMinAngle = -0.25f;
        float mHingeMaxAngle = 0.25f;
        float mMaxFrictionTorque = 0.0f;
        MotorProfile mMotor;
    };

    struct BodyProfile
    {
        std::string mName;
        std::string mBone;
        std::optional<std::string> mParentBody;
        ShapeProfile mShape;
        float mMassFraction = 0.0f;
        JPH::Vec3 mCenterOfMass = JPH::Vec3::sZero();
        MaterialProfile mMaterial;
        // Bodies sharing a non-zero group do not collide. Group zero adds no
        // filtering beyond the mandatory parent/child exclusion.
        std::uint32_t mSelfCollisionGroup = 0;
        std::optional<ConstraintProfile> mConstraint;
    };

    struct BoneAliases
    {
        std::string mCanonical;
        std::vector<std::string> mAliases;
    };

    struct LandmarkProfile
    {
        std::string mPelvis;
        std::vector<std::string> mSpine;
        std::optional<std::string> mHead;
        std::optional<std::string> mLeftHand;
        std::optional<std::string> mRightHand;
        std::optional<std::string> mLeftFoot;
        std::optional<std::string> mRightFoot;
    };

    struct ImpactResponseProfile
    {
        // Converts finalized health damage into an impulse applied at the
        // gameplay hit point. The limits keep very small and pathological
        // damage values useful without destabilizing the simulation.
        float mImpulsePerDamage = 10.0f;
        float mMinimumImpulse = 40.0f;
        float mMaximumImpulse = 800.0f;
        float mUpwardBias = 0.15f;
    };

    struct MotionTransferProfile
    {
        // Consecutive evaluated animation poses transfer their world-space
        // motion to the newly created bodies. These bounds reject stale
        // samples and clamp discontinuities such as teleports or rig rebuilds.
        float mMaximumSampleDuration = 0.1f;
        float mMaximumLinearSpeed = 2000.0f;
        float mMaximumAngularSpeed = 30.0f;
    };

    struct PoseTransitionProfile
    {
        // Render-only blend from the last evaluated animation target into the
        // interpolated physics pose. Zero disables the transition.
        float mDeathBlendDuration = 0.08f;
    };

    struct GaitControllerProfile
    {
        // Physics owns this clock. Animation sampling may consume the phase,
        // but animation time must not independently advance the stance state.
        float mMinimumStepDuration = 0.18f;
        float mNominalStepDuration = 0.42f;
        float mMaximumStepDuration = 0.75f;
        float mContactConfirmationDuration = 0.05f;
        // Stable support required before the phase clock can start. This
        // rejects a single-frame COM crossing during lateral overshoot.
        float mStanceConfirmationDuration = 0.05f;
        // A contact is a completed step only when the measured swing-foot
        // center is horizontally near its commanded placement.
        float mMaximumTouchdownHorizontalDistance = 6.0f;
        // Height above the measured stance-foot center at which a descending
        // swing foot may begin settling toward its planted target.
        float mMaximumLandingApproachVerticalDistance = 3.0f;
        // Maximum absolute swing-versus-stance vertical speed at confirmed
        // touchdown. A rebound may retain contact but cannot claim a stance.
        float mMaximumTouchdownRelativeVerticalSpeed = 5.0f;
        // OpenMW world units. Placement is always clamped around the measured
        // stance foot so an unstable observation cannot request an impossible
        // leg extension.
        float mMaximumStepReach = 55.0f;
        // Desired lateral distance between the left and right foot centers.
        float mNominalStepWidth = 20.0f;
        float mCapturePointBlend = 0.35f;
        float mSwingFootLift = 10.0f;
        float mSwingFootFrequency = 6.0f;
        float mSwingFootDamping = 1.0f;
        // Bounded Cartesian tracking trims swing-foot position and velocity
        // error. Vertical lift/descent and airborne horizontal placement have
        // separate caps so stronger landing-point braking cannot hold a foot
        // above the contact plane. Both are internal force pairs.
        float mMaximumSwingFootForce = 500.0f;
        float mMaximumSwingFootHorizontalForce = 10000.0f;
        // Fraction of the measured swing-leg weight unloaded during the
        // double-support handover. The equal reaction is applied to the
        // selected stance foot, so the whole-ragdoll net force remains zero.
        float mPreLiftSwingUnloadFraction = 0.5f;
        // Multiplies authored hip, knee, and ankle torque limits only while
        // that leg is the gait swing leg. The planted leg retains its normal
        // limits so lift authority cannot stiffen it against height support.
        float mSwingLegMotorTorqueScale = 4.0f;
        // Maximum change in the swing-thigh constraint-space motor target. This
        // prevents a new gait phase from commanding an instantaneous joint
        // orientation jump (radians per second).
        float mMaximumSwingHipTargetSpeed = 60.0f;
        // World-frame swing-thigh PD control used by the deterministic gait
        // controller when the relative Jolt hip motor is reserved. Gains are
        // torque/radian and torque/(radian/second).
        float mSwingHipWorldStiffness = 10000.0f;
        float mSwingHipWorldDamping = 2000.0f;
        float mMaximumSwingHipWorldTorque = 20000.0f;
    };

    enum class LocomotionRoute : std::uint8_t
    {
        Legacy,
        Reference,
        // Values 2 to 5 were the removed animation_assisted,
        // animation_load_assisted, animation_contact_root and
        // animation_retimed routes. KinematicRoot keeps its value so replay
        // profile hashes of kinematic-root recordings stay unchanged.
        // Legacy and Reference remain for the offline scenario controllers
        // and tests; the game ships kinematic_root.
        // PLAY-01 architecture base: the game actor owns locomotion and the
        // ragdoll root follows it as one declared kinematic body. Limbs stay
        // dynamic and track the live animation; impacts and falls release the
        // root back to dynamic motion.
        KinematicRoot = 6,
    };

    struct ActiveControllerProfile
    {
        float mSupportNormalMinimumDot = 0.5f;
        float mSupportMargin = 4.0f;
        float mStandingMinimumUpDot = 0.9f;
        float mBracingMinimumUpDot = 0.65f;
        float mStandingMaximumSpeed = 60.0f;
        float mSettledMaximumSpeed = 20.0f;
        float mFallenMaximumPelvisHeight = 30.0f;
        float mMinimumStateDuration = 0.08f;
        // Falling may only hand control back after stable, physically
        // supported contact. This prevents a bouncing foot from repeatedly
        // re-enabling motors and upright forces during a fall.
        float mFallingExitSupportDuration = 0.2f;
        float mFallingExitMaximumVerticalSpeed = 120.0f;
        float mRecoveryDelay = 0.5f;
        // A fallen actor asks to get up by itself. Recovery used to be
        // strictly opt-in and nothing in ordinary gameplay ever issued the
        // request, so a knocked-down actor simply stayed down. This only
        // supplies the missing request: the controller still waits out
        // mRecoveryDelay and requires the body to have settled before it
        // acts. Diagnostics that need to observe a fallen body for a fixed
        // window turn it off.
        bool mAutomaticRecovery = true;
        // A fallen actor puts its hands on the ground while it gets up.
        // The hands are posed, not forced: the reaction that lifts the trunk
        // comes from the shoulder motors working against a real ground
        // contact, so this adds a push rather than a shove. Turning it off
        // leaves the arms wherever the recovery pose puts them.
        bool mBracingArms = true;
        // How far out from under the shoulder the braced hand is planted, and
        // how far above the measured ground it aims. A hand aimed exactly at
        // the surface rests inside it and the contact solver spends the whole
        // rise pushing it out again.
        float mBraceHandOffset = 8.0f;
        float mBraceHandClearance = 2.0f;
        // Held against the recovery pose rather than replacing it. At full
        // weight the arms stop following the pose that is trying to stand the
        // body up, and the brace fights the rise it is meant to help.
        float mBraceWeight = 0.7f;
        float mBraceBlendRate = 5.0f;
        // How far inside the arm's own reachable band the hand is planted,
        // as a fraction of the distance from its minimum to its maximum. The
        // band comes from the elbow hinge rather than from a guess: an elbow
        // has a minimum reach as well as a maximum, and a shoulder lying flat
        // on the ground puts the naive point under it well inside the fold
        // the joint can close to. Keeping off both ends leaves the arm room
        // to give rather than arriving already locked.
        float mBraceReachMargin = 0.15f;
        // The shoulder cone a brace needs, in radians. Pushing a body up off
        // the floor asks the shoulder for far more range than the authored
        // 0.85/0.75, which suits a corpse: once the solver began honouring
        // that cone the braced hand could no longer reach the ground and fell
        // 12 to 20 units short. Opened only while the arm is reaching and
        // restored the moment it stops, so a walking or standing actor keeps
        // the authored joint.
        float mBraceShoulderNormalHalfCone = 1.75f;
        float mBraceShoulderPlaneHalfCone = 1.55f;
        float mBraceShoulderTwistLimit = 1.05f;
        float mBlendBackDuration = 0.25f;
        // Maximum continuous loss of all measured ground contact while the
        // physical settling phase owns the pose. Expiry releases the motors
        // into Falling; it never destroys the rig or returns to animation.
        float mRecoveryHandoverTimeout = 2.0f;
        // Translation gains are expressed as spring frequencies so the
        // resulting pelvis force scales with the observed total body mass.
        float mPelvisHeightFrequency = 3.0f;
        float mPelvisHeightDamping = 1.0f;
        // 0.5 Hz keeps a 60-unit/s step request below the 0.8-friction
        // support limit (2*zeta*omega*dv < mu*g) instead of commanding a
        // permanently saturated shove.
        // Balance bandwidth. At 0.5 Hz this was a two second response
        // against a 0.37 s step: the feedback holding the body over its feet
        // was about five times slower than the gait it stabilises, so a
        // disturbance could not be answered inside the step that caused it.
        // Measured on a lateral push sweep at request 60, raising it to 1 Hz
        // moves the fall boundary from 2500 to 3000 impulse units and also
        // reduces nominal foot slide from 0.86 to 0.71, at a cost of 0.4 in
        // mean speed. 1.5 Hz is no better and 2.0 Hz was worse, so this is
        // not simply "faster is better".
        float mCenterOfMassFrequency = 1.0f;
        // The hip strategy. calculateUprightControl already computes a pelvis
        // and a spine torque every step - counter-rotation, the middle rung
        // of the ankle-hip-step hierarchy a biped rejects disturbances with -
        // and both the offline harness and the live adapter discard it on the
        // production reference route. Capture-point placement treats the body
        // as a point mass and ignores angular momentum entirely, so with no
        // hip term every disturbance has to be absorbed by where the foot
        // lands; when one step cannot cover it, the actor falls.
        //
        // Only the TORQUE is applied. Joint support already owns the centre
        // of mass, and driving that twice would have two controllers contest
        // one degree of freedom.
        bool mReferenceHipStrategy = false;
        float mCenterOfMassDamping = 1.0f;
        // Desired gait velocity is clamped before it enters the balance
        // controller. Units are OpenMW world units per second.
        float mMaximumLocomotionSpeed = 400.0f;
        // The desired pelvis height is measured from the settled physical
        // ragdoll rather than its insertion pose. A candidate must remain
        // supported and vertically quiet for this duration before it can be
        // frozen for locomotion; provisional support is available to upright
        // control immediately so insertion motion can be arrested.
        float mPelvisHeightSettleDuration = 0.25f;
        float mPelvisHeightSettleMaximumVerticalSpeed = 7.5f;
        // A newly inserted ragdoll can begin a few fixed steps above its
        // eventual contact manifold. Keep motors enabled until it first
        // acquires foot support, but never reuse this grace afterward.
        float mActivationSupportGraceDuration = 0.15f;
        float mLocomotionSupportGraceDuration = 0.15f;
        float mMaximumPelvisForce = 150000.0f;
        // Route-B translation is deliberately external: the animation owns
        // the joint targets while this bounded force keeps the dynamic
        // pelvis near the requested velocity and settled height. Separate
        // component scales make an assist-off comparison and later reduction
        // toward fully physical walking explicit and reproducible.
        float mAnimationAssistHorizontalScale = 1.0f;
        float mAnimationAssistVerticalScale = 1.0f;
        float mMaximumAnimationAssistForce = 150000.0f;
        float mAnimationAssistUprightScale = 1.0f;
        float mMaximumAnimationAssistTorque = 1200000.0f;
        // PLAY-ASSISTED-v1 heading reference and yaw servo.  The final yaw
        // request is composed with tilt before mMaximumAnimationAssistTorque;
        // this component limit is not an additional actuator budget.
        float mAnimationAssistYawReferenceMaximumRate = 3.14159265f;
        float mAnimationAssistYawReferenceMaximumAcceleration = 12.5663706f;
        float mAnimationAssistYawStiffness = 200000.0f;
        float mAnimationAssistYawDamping = 50000.0f;
        float mMaximumAnimationAssistYawTorque = 400000.0f;
        // PLAY-WALK-v2: the live animation sample remains the sole joint-pose
        // reference, blended from the measured physical pose at each fixed
        // step. A supported stance foot keeps a compliant world orientation;
        // contact loss fades that anchor over a finite handoff interval.
        float mAnimationAssistPoseTrackingBlend = 0.8f;
        float mAnimationAssistStanceAnchorCompliance = 0.2f;
        float mAnimationAssistSupportLossHandoffDuration = 0.1f;
        // PLAY-WALK-v3: convert a latched stance contact position into a
        // bounded endpoint correction for the existing leg-pose motors.
        // These parameters do not alter native motor torque or joint limits.
        float mAnimationAssistStancePositionCorrection = 0.65f;
        float mAnimationAssistStanceMaximumEndpointCorrection = 1.0f;
        // PLAY-WALK-v4: measured-contact support loads follow the same
        // horizontal motion reference as the adapted animation. These gains
        // only affect support feedback and do not alter the requested speed.
        float mAnimationAssistSupportMotionFrequency = 0.5f;
        float mAnimationAssistSupportMotionDamping = 1.0f;
        float mAnimationAssistSupportPositionLeash = 2.0f;
        // PLAY-WALK-v6: unilateral contact-root authority is dormant until the
        // selected measured support remains present for this many fixed ticks,
        // then ramps in over a finite duration. These values do not alter the
        // registered motor or external-assistance limits.
        unsigned mAnimationAssistContactConfirmationTicks = 3;
        float mAnimationAssistLoadTransferDuration = 0.1f;
        // PLAY-WALK-v8: one shared pose/root cursor can catch up to positive
        // measured forward progress, but only while its physical stance is
        // feasible. The deadline bounds both an unsafe hold and waiting for
        // the registered alternate touchdown.
        float mAnimationAssistMeasuredProgressBlend = 1.0f;
        float mAnimationAssistFeasibilityDeadline = 0.1f;
        // PLAY-WALK-v9: the support solver begins on both measured feet,
        // transfers to the selected stance over this finite interval, and
        // cannot re-enter after fallback until real contact slip is settled.
        float mAnimationAssistDoubleSupportTransferDuration = 0.1f;
        float mAnimationAssistReentrySlipSettleDuration = 0.1f;
        // PLAY-WALK-v11: while v9 holds at a real alternate touchdown,
        // brake the pending measured foot toward its first world contact
        // through the existing bounded leg-pose motors.
        float mAnimationAssistTouchdownCapturePositionCorrection = 0.35f;
        float mAnimationAssistTouchdownCaptureVelocityLookahead = 0.04f;
        // PLAY-WALK-v12: during the same finite double-support window,
        // replace the outgoing animation-relative endpoint with a separate
        // measured-relative bounded motor target.
        float mAnimationAssistOutgoingHoldPositionCorrection = 0.20f;
        float mAnimationAssistOutgoingHoldVelocityLookahead = 0.02f;
        // Kinematic-root route: a landing whose downward speed reaches this
        // threshold releases the root to the passive ragdoll so the impact and
        // recovery are physical. Below it the limbs absorb the landing.
        float mKinematicRootHardLandingSpeed = 300.0f;
        // The kinematic route retimes the clip from the actor's actual ground
        // speed, so it can play slower than the reference gait's floor without
        // the clip outrunning the body.
        float mKinematicRootMinimumRate = 0.25f;
        // Jump takeoff anticipation: the requested jump is buffered through a
        // short crouch so the body has a visible buildup before leaving the
        // ground.
        float mKinematicRootJumpAnticipation = 0.15f;
        // Pose-tracking blend for the kinematic route. The shared animation
        // blend (0.8) leaves the torso, head and arms lagging behind the root,
        // which reads as leaning backwards while walking; the kinematic route
        // tracks the live animation directly.
        float mKinematicRootPoseTrackingBlend = 1.0f;
        // Fastest the kinematic root may turn (radians/second). The camera
        // can spin the actor thousands of degrees per second; a root that
        // follows that exactly flings the limbs. The body's yaw follows the
        // actor at this rate and catches up when the spin stops.
        float mKinematicRootMaximumYawRate = 9.4248f;
        float mRecoveryUnweightFactor = 2.5f;
        // Upright gains operate directly in torque/radian and torque/(radian/s).
        // Explicit torque caps are the final safety boundary.
        float mPelvisUprightStiffness = 800000.0f;
        float mPelvisUprightDamping = 120000.0f;
        float mMaximumPelvisTorque = 1200000.0f;
        float mSpineUprightStiffness = 480000.0f;
        float mSpineUprightDamping = 80000.0f;
        float mMaximumSpineTorque = 800000.0f;
        GaitControllerProfile mGait;
        // Which locomotion route the live adapter runs when a powered actor
        // requests movement: the reference joint-torque gait (the route that
        // passes the walking contract) or the legacy pelvis force-pair gait.
        LocomotionRoute mLocomotionRoute = LocomotionRoute::Reference;
    };

    struct RagdollProfile
    {
        std::uint32_t mSchemaVersion = CurrentProfileSchemaVersion;
        std::string mId;
        std::vector<BoneAliases> mBoneAliases;
        std::vector<BodyProfile> mBodies;
        LandmarkProfile mLandmarks;
        ImpactResponseProfile mImpactResponse;
        MotionTransferProfile mMotionTransfer;
        PoseTransitionProfile mPoseTransition;
        ActiveControllerProfile mActiveController;
    };

    enum class ValidationCode
    {
        UnsupportedSchema,
        EmptyIdentifier,
        DuplicateIdentifier,
        InvalidNumber,
        InvalidHierarchy,
        MissingBone,
        MissingBody,
        InvalidShape,
        InvalidMass,
        InvalidConstraint,
        InvalidLandmark,
        InvalidInitialState,
        InvalidController,
        RuntimeCreationFailed,
    };

    struct ValidationIssue
    {
        ValidationCode mCode;
        std::string mPath;
        std::string mMessage;
    };

    struct ValidationResult
    {
        std::vector<ValidationIssue> mIssues;

        bool isValid() const { return mIssues.empty(); }
    };

    ValidationResult validateSkeleton(const SkeletonDescriptor& skeleton);
    ValidationResult validateProfile(const RagdollProfile& profile, const SkeletonDescriptor& skeleton);
}

#endif
