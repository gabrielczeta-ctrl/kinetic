#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_GAITCONTROLLER_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_GAITCONTROLLER_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <ragdollcore/controller.hpp>
#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/profile.hpp>

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace RagdollCore
{
    enum class GaitPhase
    {
        Idle,
        LeftStance,
        RightStance,
    };

    struct GaitControllerInput
    {
        bool mEnabled = false;
        bool mLeftFootSupported = false;
        bool mRightFootSupported = false;
        // Shared directional capture readiness: lateral containment and a
        // reachable forward/backward capture state. This must remain viable
        // until physical lift-off. The offline COM-only mode is diagnostic.
        bool mStanceSupportsCenterOfMass = false;
        float mLeftFootHeight = 0.0f;
        float mRightFootHeight = 0.0f;
        float mLeftFootVerticalVelocity = 0.0f;
        float mRightFootVerticalVelocity = 0.0f;
        // Horizontal distance between the measured swing-foot body origin and
        // the latched body-origin placement for prospective touchdown.
        float mSwingFootHorizontalPlacementError = 0.0f;
        // Absolute height difference between the measured swing- and
        // stance-foot body origins at the prospective touchdown.
        float mSwingFootVerticalPlacementError = 0.0f;
        // Absolute difference between swing- and stance-foot vertical speeds.
        float mSwingFootRelativeVerticalSpeed = 0.0f;
    };

    struct GaitControllerOutput
    {
        GaitPhase mPhase = GaitPhase::Idle;
        float mTimeInPhase = 0.0f;
        float mPhaseProgress = 0.0f;
        float mSwingContactCandidateTime = 0.0f;
        float mSwingTimeSinceLift = 0.0f;
        bool mStanceReady = false;
        bool mSwingFootLifted = false;
        // The lifted foot is descending close to, or has contacted, its
        // horizontal landing area. Actuation may settle toward the placement,
        // but phase transfer still requires confirmed contact and the strict
        // relative-vertical-speed gate.
        bool mSwingTouchdownCandidate = false;
        bool mTransitioned = false;
        bool mPhaseTimedOut = false;

        bool hasStance() const { return mPhase != GaitPhase::Idle; }
        bool leftFootIsStance() const { return mPhase == GaitPhase::LeftStance; }
        bool rightFootIsStance() const { return mPhase == GaitPhase::RightStance; }
    };

    enum class GaitActuationStage
    {
        Inactive,
        Settling,
        AwaitingStance,
        WeightTransfer,
        Swing,
        Touchdown,
        Unloading,
    };

    GaitActuationStage classifyGaitActuationStage(
        bool locomotionRequested, bool pelvisHeightSettled, const GaitControllerOutput& gait);
    const char* gaitActuationStageName(GaitActuationStage stage);

    enum class GaitPoseSource
    {
        Base,
        MeasuredSwingLeg,
        GaitCycle,
        MeasuredTouchdown,
    };

    enum class GaitHorizontalControl
    {
        DefaultSupport,
        FullStance,
        LateralStance,
    };

    enum class GaitMeasuredPoseFrame
    {
        Model,
        Joint,
    };

    struct GaitActuationDecisionInput
    {
        bool mLocomotionRequested = false;
        bool mPelvisHeightSettled = false;
        bool mDriveJointMotors = false;
        bool mRecovering = false;
        bool mReferenceCorrectiveStep = false;
        bool mCorrectTranslation = false;
        bool mBothFeetSupported = false;
        bool mStanceFootSupported = false;
        bool mSwingTargetAvailable = false;
        JPH::Vec3 mMeasuredHorizontalVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        GaitControllerOutput mGait;
    };

    struct GaitActuationDecision
    {
        GaitActuationStage mStage = GaitActuationStage::Inactive;
        // Standing uses contact-derived joint load compensation. Recovery and
        // locomotion retain their separate actuation until their gates pass.
        bool mUseJointSupportControl = false;
        // Recovery must balance root/spine torque with a body reaction even
        // though it has no walking stance phase.
        bool mRequireInternalAngularReaction = false;
        GaitPoseSource mPoseSource = GaitPoseSource::Base;
        GaitHorizontalControl mHorizontalControl = GaitHorizontalControl::DefaultSupport;
        // A measured hold must preserve joints relative to the pelvis target.
        GaitMeasuredPoseFrame mMeasuredPoseFrame = GaitMeasuredPoseFrame::Joint;
        JPH::Vec3 mTargetHorizontalVelocity = JPH::Vec3::sZero();
        float mStanceLegMotorTorqueScale = 1.0f;
        float mSwingLegMotorTorqueScale = 1.0f;
        bool mLocomoting = false;
        bool mApplyPreLiftSwingUnload = false;
        bool mUseStanceSupportContacts = false;
        bool mUseStanceVerticalReaction = false;
        bool mUseStanceHorizontalReaction = false;
        bool mSwingFootInFlight = false;
        bool mSwingReactionThroughStanceFoot = true;
        bool mReleaseStanceHipAxis = false;
        bool mReleaseSwingHipAxis = false;
        bool mUseSupportedFootWorldMotors = false;
        JPH::Vec3 mFreeHipWorldAxis = JPH::Vec3::sZero();
        GaitPhase mPhase = GaitPhase::Idle;
    };

    // Converts measured gait/controller state into a side-effect-free actuation
    // policy. The offline harness and live adapter remain responsible only for
    // gathering observations and applying the returned policy to Jolt.
    std::optional<GaitActuationDecision> calculateGaitActuationDecision(
        const GaitActuationDecisionInput& input, const GaitControllerProfile& profile);

    struct SupportMotionReference
    {
        JPH::RVec3 mPosition = JPH::RVec3::sZero();
        JPH::Vec3 mVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mAcceleration = JPH::Vec3::sZero();
        // Absent: allocate support across both measured feet. Specifying a
        // foot excludes the other foot from planned support loads while its
        // real contacts remain in the simulation. No contact is fabricated.
        std::optional<std::size_t> mSupportFoot;
    };

    // Same measured-body assembly and horizontal COM feedback in both adapters.
    // It does not depend on a settled pelvis-height target. Returned contact
    // forces describe planned loads, never external actuation to apply.
    // An optional world-space motion reference replaces midpoint centering;
    // its horizontal acceleration feeds forward while position and velocity
    // errors retain stabilizing feedback. Vertical posture remains motor-owned.
    std::optional<JointSupportOutput> calculateStandingJointSupport(const RagdollTelemetry& telemetry,
        std::span<const GroundContact> contacts, FootBodyIndices feet, float gravity, float friction,
        const ActiveControllerProfile& profile, const std::optional<SupportMotionReference>& motion = std::nullopt);

    // Converts the solver's measured normal impulse on one physical foot into
    // a fraction of actor weight carried during this fixed step. Contacts with
    // no reported impulse contribute no load; support geometry alone must not
    // manufacture a load-transfer observation.
    std::optional<float> calculateMeasuredFootNormalLoadFraction(
        std::span<const GroundContact> contacts, std::size_t footBody,
        float actorMass, float gravity, float duration);

    // Interpolates two complete support plans at one authority weight. This is
    // used to transfer from a measured bilateral base to a unilateral stance
    // without independently switching force, torque, and residual channels.
    std::optional<JointSupportOutput> blendJointSupportOutputs(
        const JointSupportOutput& bilateral, const JointSupportOutput& unilateral, float unilateralWeight);

    // Applies the decision's positional policy to already measured offsets.
    // The stance offset is the horizontal vector from the selected stance
    // support target to the COM; it may be absent before a stance is observed.
    std::optional<JPH::Vec3> selectGaitCenterOfMassOffset(const GaitActuationDecision& decision,
        JPH::Vec3Arg requestedHorizontalVelocity, JPH::Vec3Arg defaultSupportOffset,
        const std::optional<JPH::Vec3>& stanceOffset);

    struct GaitStanceSupport
    {
        SupportEstimate mSupport;
        bool mReady = false;
        std::optional<JPH::RVec3> mBackwardRecoveryTarget;
    };

    struct GaitCaptureStepInput
    {
        JPH::RVec3 mStanceFootPosition = JPH::RVec3::sZero();
        float mTimeInPhase = 0.0f;
    };

    // Both adapters observe the same contact geometry. Capture readiness is
    // selectable here only to retain the offline diagnostic negative control.
    std::optional<GaitStanceSupport> estimateGaitStanceSupport(GaitPhase phase,
        JPH::RVec3Arg centerOfMass, JPH::Vec3Arg velocity, JPH::Vec3Arg requestedVelocity,
        float height, float gravity, std::span<const GroundContact> contacts,
        FootBodyIndices feet, const ActiveControllerProfile& profile, bool useCapturePoint,
        const std::optional<GaitCaptureStepInput>& captureStep = std::nullopt);

    void configureGaitSupportBudget(const GaitActuationDecision& decision, bool correctTranslation,
        std::span<const GroundContact> contacts, FootBodyIndices feet, UprightControllerInput& upright);

    struct GaitReactionBodies
    {
        std::vector<std::size_t> mVertical;
        std::vector<std::size_t> mHorizontal;
        std::optional<std::size_t> mAngular;
        // Recovery can distribute the fixed angular reaction across two
        // measured support chains.
        std::optional<std::size_t> mSecondaryAngular;
    };

    GaitReactionBodies selectGaitReactionBodies(const GaitActuationDecision& decision,
        const SupportEstimate& support, FootBodyIndices feet, std::size_t leftHip, std::size_t rightHip,
        bool settlingRecovery = false);

    std::vector<std::size_t> selectGaitWorldFootMotors(const GaitActuationDecision& decision,
        const SupportEstimate& support, FootBodyIndices feet);

    std::optional<float> calculatePreLiftSwingUnloadForce(const GaitActuationDecision& decision,
        float totalMass, float swingLegMass, float gravity, const GaitControllerProfile& profile);

    // Owns locomotion time in fixed physics steps. A phase can begin only on
    // measured support and can switch only after confirmed swing-foot contact;
    // reaching the time limit is reported rather than fabricating a contact.
    class GaitPhaseController
    {
    public:
        bool step(const GaitControllerInput& input, float duration, const GaitControllerProfile& profile,
            GaitControllerOutput& output);
        void reset();
        GaitControllerOutput getOutput() const;

    private:
        GaitPhase mPhase = GaitPhase::Idle;
        float mTimeInPhase = 0.0f;
        float mSwingContactCandidateTime = 0.0f;
        float mSwingTimeSinceLift = 0.0f;
        float mPhaseProgress = 0.0f;
        float mSwingFootStartRelativeHeight = 0.0f;
        float mStanceSupportCandidateTime = 0.0f;
        float mStanceSupportLossCandidateTime = 0.0f;
        bool mStanceReady = false;
        bool mSwingFootLifted = false;
        bool mSwingTouchdownCandidate = false;
        bool mPhaseTimedOut = false;
    };

    // Maps the physics-owned left/right stance phase onto a normalized gait
    // cycle. Left stance occupies [0, 0.5], right stance [0.5, 1].
    std::optional<float> calculateGaitCyclePhase(const GaitControllerOutput& gait);

    // Samples a uniformly spaced, cyclic sequence of model-space poses using
    // only the physics-owned gait phase. The final sample interpolates back to
    // the first; animation/render time is deliberately not an input.
    bool sampleGaitPoseCycle(std::span<const RagdollPose> samples, const GaitControllerOutput& gait, RagdollPose& out);

    struct CapturePointInput
    {
        JPH::RVec3 mCenterOfMassPosition = JPH::RVec3::sZero();
        JPH::Vec3 mCenterOfMassVelocity = JPH::Vec3::sZero();
        float mCenterOfMassHeight = 0.0f;
        float mGravity = 0.0f;
    };

    // Linear-inverted-pendulum capture point on a horizontal support plane.
    // Height and gravity must use the same distance unit.
    std::optional<JPH::RVec3> calculateCapturePoint(const CapturePointInput& input);

    struct SwingFootTargetInput
    {
        GaitPhase mPhase = GaitPhase::Idle;
        JPH::RVec3 mCenterOfMassPosition = JPH::RVec3::sZero();
        JPH::Vec3 mCenterOfMassVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mTargetHorizontalVelocity = JPH::Vec3::sZero();
        JPH::RVec3 mStanceFootPosition = JPH::RVec3::sZero();
        JPH::Vec3 mForward = JPH::Vec3::sAxisY();
        float mCenterOfMassHeight = 0.0f;
        float mGravity = 0.0f;
    };

    // Places the swing foot one nominal full step width from the measured
    // stance foot, blends forward desired-velocity placement with the capture
    // point, and clamps the result to a reachable horizontal radius.
    std::optional<JPH::RVec3> calculateSwingFootTarget(
        const SwingFootTargetInput& input, const GaitControllerProfile& profile);

    struct GaitSwingPlacement
    {
        std::optional<JPH::RVec3> mTarget;
        bool mBackwardRecovery = false;
    };

    // A recovery proposal belongs to the observed stance. Discard it on
    // handover, retain a chosen recovery when no new proposal is reachable,
    // and freeze every placement after physical lift-off.
    std::optional<GaitSwingPlacement> selectGaitSwingPlacement(const GaitControllerOutput& gait,
        const SwingFootTargetInput& input, const std::optional<JPH::RVec3>& backwardRecoveryTarget,
        const GaitSwingPlacement& previous, const GaitControllerProfile& profile);

    struct SwingFootTrajectoryInput
    {
        JPH::Vec3 mLiftOffPosition = JPH::Vec3::sZero();
        JPH::Vec3 mPlacementPosition = JPH::Vec3::sZero();
        GaitControllerOutput mGait;
    };

    struct SwingFootTrajectoryOutput
    {
        JPH::Vec3 mPosition = JPH::Vec3::sZero();
        JPH::Vec3 mLinearVelocity = JPH::Vec3::sZero();
    };

    // Keeps horizontal placement at lift-off until contact has actually
    // released. The descent clock starts at measured lift-off, so a delayed
    // release cannot consume the complete swing arc while the foot is pinned.
    std::optional<SwingFootTrajectoryOutput> calculateSwingFootTrajectoryState(
        const SwingFootTrajectoryInput& input, const GaitControllerProfile& profile);

    // Position-only compatibility wrapper for consumers that do not apply a
    // Cartesian trajectory controller.
    std::optional<JPH::Vec3> calculateSwingFootTrajectory(
        const SwingFootTrajectoryInput& input, const GaitControllerProfile& profile);

    struct TwoBoneLegTargetInput
    {
        std::size_t mHipBodyIndex = 0;
        std::size_t mKneeBodyIndex = 0;
        std::size_t mFootBodyIndex = 0;
        // Model-space target and preferred knee-bend direction. The solver
        // preserves segment lengths and clamps unreachable targets.
        JPH::Vec3 mTargetFootPosition = JPH::Vec3::sZero();
        JPH::Vec3 mPreferredBendDirection = JPH::Vec3::sAxisY();
        std::optional<HingeLegGeometry> mHingeGeometry;
    };

    // Applies a geometric two-bone IK result to model-space motor targets.
    // Invalid input leaves the complete pose unchanged.
    bool applyTwoBoneLegTarget(std::span<Transform> pose, const TwoBoneLegTargetInput& input);

    // Model preserves the checkpoint's pose hold. Joint copies measured
    // relative joint orientations into the target pelvis frame; that candidate
    // removes the hip error caused by pelvis tilt but has not earned adoption
    // in physical locomotion. Both choices validate the entire chain first.
    bool applyGaitMeasuredLegTargets(std::span<Transform> target, std::span<const Transform> measured,
        std::size_t pelvis, std::span<const std::size_t> leg, GaitMeasuredPoseFrame frame);

    // Seeds one target leg from its measured relative joint orientations, then
    // applies the existing hinge-limited endpoint IK. Only the selected leg is
    // changed, and any validation or IK failure leaves the pose unchanged.
    bool applyMeasuredSeededTwoBoneLegTarget(std::span<Transform> target,
        std::span<const Transform> measured, std::size_t pelvis, const TwoBoneLegTargetInput& input);

    struct LegFrameTargetInput
    {
        Transform mBasePelvis;
        Transform mBaseHip;
        Transform mMeasuredPelvis;
        Transform mMeasuredHip;
        JPH::Vec3 mTargetFootPosition = JPH::Vec3::sZero();
        JPH::Vec3 mPreferredBendDirection = JPH::Vec3::sAxisY();
    };

    struct LegFrameTargetOutput
    {
        JPH::Vec3 mTargetFootPosition = JPH::Vec3::sZero();
        JPH::Vec3 mPreferredBendDirection = JPH::Vec3::sAxisY();
        JPH::Quat mPelvisRotationDelta = JPH::Quat::sIdentity();
    };

    // Re-expresses a reference-model foot target as the local joint target
    // needed at the measured hip and pelvis orientation. This keeps a moving
    // ragdoll from solving every step against its stale bind-pose hip.
    std::optional<LegFrameTargetOutput> resolveLegFrameTarget(const LegFrameTargetInput& input);

    struct LevelFootTargetInput
    {
        JPH::Quat mBaseFootRotation = JPH::Quat::sIdentity();
        JPH::Quat mTargetCalfRotation = JPH::Quat::sIdentity();
        JPH::Quat mMeasuredCalfRotation = JPH::Quat::sIdentity();
        JPH::Quat mPelvisRotationDelta = JPH::Quat::sIdentity();
    };

    // Produces the calf-relative ankle target that keeps the physical foot at
    // its reference level orientation. Pelvis yaw follows the actor heading;
    // pelvis pitch and roll do not tilt the sole into the support surface.
    std::optional<JPH::Quat> calculateLevelFootTarget(const LevelFootTargetInput& input);

    struct SwingLegPoseOutput
    {
        LegFrameTargetOutput mFrame;
        bool mApplied = false;
        JPH::Vec3 mFootPosition = JPH::Vec3::sZero();
        JPH::Quat mHipModelRotation = JPH::Quat::sIdentity();
    };

    // Re-expresses the model-space endpoint at the measured pelvis/hip, solves
    // the native leg geometry, levels the foot, then applies touchdown policy.
    // Root/world conversion belongs to the adapter. Failure never edits target;
    // a valid frame whose IK cannot be solved returns mApplied = false.
    std::optional<SwingLegPoseOutput> applySwingLegPose(std::span<Transform> target,
        std::span<const Transform> base, std::span<const Transform> measured, std::size_t pelvis,
        TwoBoneLegTargetInput input, const GaitActuationDecision& decision);

    struct WorldOrientationControllerInput
    {
        JPH::Quat mCurrentRotation = JPH::Quat::sIdentity();
        JPH::Quat mTargetRotation = JPH::Quat::sIdentity();
        JPH::Vec3 mCurrentAngularVelocity = JPH::Vec3::sZero();
    };

    struct WorldOrientationControllerOutput
    {
        JPH::Vec3 mTorque = JPH::Vec3::sZero();
        bool mSaturated = false;
    };

    // Drives a body toward a world-frame orientation. Unlike a relative joint
    // motor, this remains well-defined while the pelvis rotates. The caller
    // must provide an equal-and-opposite stance reaction.
    std::optional<WorldOrientationControllerOutput> calculateSwingHipWorldControl(
        const WorldOrientationControllerInput& input, const GaitControllerProfile& profile);

    struct SwingFootControllerInput
    {
        JPH::RVec3 mCurrentPosition = JPH::RVec3::sZero();
        JPH::RVec3 mTargetPosition = JPH::RVec3::sZero();
        JPH::Vec3 mCurrentVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mTargetVelocity = JPH::Vec3::sZero();
        float mEffectiveMass = 0.0f;
        // A foot that is still touching support may only be unloaded
        // vertically. Once measured contact has released, Cartesian
        // assistance may track the complete swing trajectory.
        bool mFootLifted = false;
    };

    struct SwingFootControllerOutput
    {
        JPH::Vec3 mForce = JPH::Vec3::sZero();
        bool mSaturated = false;
    };

    // Cartesian swing-foot assistance complements joint orientation motors.
    // The caller applies this as an internal foot-versus-thigh force pair.
    std::optional<SwingFootControllerOutput> calculateSwingFootControl(
        const SwingFootControllerInput& input, const GaitControllerProfile& profile);
}

#endif
