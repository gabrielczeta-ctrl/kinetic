#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_SCENARIO_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_SCENARIO_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <ragdollcore/controller.hpp>
#include <ragdollcore/gaitcontroller.hpp>
#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/profile.hpp>
#include <ragdollcore/referencegait.hpp>
#include <ragdollcore/unitmodel.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace RagdollCore
{
    // Fixed-step offline scenario harness. It exercises the shared OpenMW
    // powered-ragdoll control loop (contact observation, support estimate,
    // deterministic controller, physics-owned gait phase, bounded upright
    // control through the pelvis-versus-supported-feet force pair, and joint
    // motors) inside an isolated single-threaded Jolt system with a flat
    // static ground plane. The harness depends on Jolt and the C++ standard
    // library only, so the same scenario can run in tests, tuning tools, and a
    // future trainer. A synthetic rig and isolated scene do not establish
    // equivalence to native player/NPC integration or recorded live failures.

    // Code-defined standing humanoid whose bone origins are OpenMW world units
    // in actor-root space with both foot boxes resting on z = 0. Bone
    // rotations are identity; derived constraint frames therefore come only
    // from bone positions, exactly as the builder derives them for a rig.
    SkeletonDescriptor makeScenarioHumanoidSkeleton();

    // Transcribed from files/data/ragdoll/humanoid-v1.yaml so the offline
    // scenario exercises the same bodies, limits, motors, and controller
    // thresholds as the live game without a YAML, VFS, or game-data
    // dependency. A components test loads the authored YAML and requires the
    // canonical profile hash to match this fixture after applying only the
    // documented differences: the identifier and the absent bone aliases.
    // The elbow limits now match the authored profile on both sides.
    RagdollProfile makeScenarioHumanoidProfile();

    enum class ScenarioTranslationRouting : std::uint8_t
    {
        // Existing controller path: equal and opposite forces at body centers
        // of mass. It conserves linear momentum but can inject a force couple.
        InternalForcePair,
        // Keep vertical height support as an internal pelvis/feet pair, but
        // apply horizontal locomotion as a tipping-limited virtual ground
        // reaction at measured stance-foot contacts after physical swing
        // lift-off. This permits net locomotion momentum only during a
        // measured single-stance swing phase.
        GroundContactForce,
    };

    enum class ScenarioSwingAssistAxes : std::uint8_t
    {
        // Production-compatible diagnostic: joint motors own horizontal
        // placement and the Cartesian pair only unloads the swing foot.
        VerticalOnly,
        // In-flight position/velocity control on all three world axes. This
        // remains an internal equal-and-opposite pair and is opt-in until it
        // beats the physical gait gates.
        Full3D,
    };

    enum class ScenarioSwingHorizontalReaction : std::uint8_t
    {
        // Keep the reaction inside the articulated swing leg.
        SwingHip,
        // Spread the reaction through the central body instead of loading the
        // comparatively light thigh directly.
        Pelvis,
        // Route through the planted leg without applying the reaction to its
        // small terminal body directly.
        StanceHip,
        // Diagnostic contact route. Its force cap must remain inside the
        // stance contact's measured friction budget.
        StanceFoot,
    };

    enum class ScenarioLocomotionController : std::uint8_t
    {
        // Retained force-pair/pose route with capture readiness gating.
        Legacy = 0,
        // Reference joint-torque route (referencegait.hpp): native pose
        // springs with inverse-static loads, world-frame swing thigh, pelvis
        // torque routed through the stance hip and contact-or-time stances.
        // Standing before the locomotion request keeps the shared standing
        // joint-support controller unchanged.
        Reference = 1,
        // Offline tracking experiment: the supplied reference pose and bounded
        // parent-local offsets own ordinary joint motors, including their
        // velocity targets. No procedural gait, standing support loads, upright
        // forces/torques or prepared support-step actuation is applied. Motor
        // release and the existing physical controller's drive eligibility
        // remain authoritative. This is not a qualified walking controller.
        DirectMotorTracking = 2,
        // Offline learning experiment: bounded parent-local pose corrections
        // around Reference, including standing and hand-over. All production
        // load, support, world-target, spring, release and safety decisions
        // remain authoritative. Zero offsets preserve every physical frame;
        // the controller identity in encoded scenario settings stays distinct.
        ProductionResidual = 3,
        // 4 was the removed animation contact-root parity route.
    };

    struct ScenarioReplayReferenceFrame
    {
        JPH::RVec3 mPosition = JPH::RVec3::sZero();
        JPH::Quat mRotation = JPH::Quat::sIdentity();
    };

    struct ScenarioSettings
    {
        float mFixedTimeStep = 1.0f / 60.0f;
        std::uint32_t mCollisionSteps = 1;
        // Zero retains Jolt defaults. Explicit quality for loaded-chain
        // convergence experiments; recorded in single-actor and crowd identity.
        std::uint32_t mVelocitySolverSteps = 0;
        // Discrete mode is a diagnostic counterpart for solver-wrench analysis.
        // Physical acceptance scenarios retain continuous collision by default.
        bool mUseContinuousCollision = true;
        float mGravity = UnitModel::GravityUnitsPerSecondSquared;
        // The ground is a static box whose top face is the plane z = mGroundHeight.
        float mGroundHeight = 0.0f;
        float mGroundHalfExtent = 4096.0f;
        float mGroundThickness = 32.0f;
        float mGroundFriction = 0.8f;
        // Match mesh-backed live static floors when comparing contact behavior.
        // The default box remains the original scenario fixture.
        bool mTriangleMeshGround = false;
        float mTotalMass = 75.0f;
        float mUniformScale = 1.0f;
        JPH::RVec3 mRootPosition = JPH::RVec3::sZero();
        JPH::Quat mRootRotation = JPH::Quat::sIdentity();
        // Capture-replay diagnostics: insertion motion is distinct from a
        // locomotion request. Filtered inputs bypass only the steering adapter.
        BodyMotion mInitialActorMotion;
        // Diagnostic allocation history, including Jolt sequence numbers.
        std::vector<std::uint32_t> mReplayBodyIds;
        std::optional<std::uint32_t> mReplayGroundId;
        std::vector<std::byte> mReplayGroundShape;
        JPH::RVec3 mReplayGroundPosition = JPH::RVec3::sZero();
        JPH::Quat mReplayGroundRotation = JPH::Quat::sIdentity();
        float mReplayGroundRestitution = 0;
        bool mReferenceInputsAreFiltered = false;
        bool mReplayPreserveContactOrder = false;
        // Replay adapter parity for the passive Fallen -> automatic-brace
        // shoulder-limit event. Does not implement driven arm recovery.
        bool mReplayNativeBraceLimits = false;
        bool mReplayAutomaticRecoveryRequest = false;
        bool mReplayBraceTargets = false;
        // Reconstruct physics-owned horizontal root motion; keep recorded Z/yaw.
        bool mReplayComputeHorizontalRoot = false;
        // Historical native reset discarded the first standing support plan.
        // Explicit reconstruction compatibility, not desired controller behavior.
        bool mReplayDiscardSupportOnReferenceReset = false;
        // Observe a scripted rising request before the next actuation step.
        // Reconstruct with shared steering from inputs, never captured outputs.
        bool mReplayObserveRestartRequest = false;
        // Authoritative callback events, including an explicitly empty timeline.
        bool mReplayRecoveryFinishEventsCaptured = false;
        std::vector<std::size_t> mReplayRecoveryFinishSteps;
        std::vector<std::size_t> mReplayRecoveryRequestSteps;
        // Recorded post-solve coordinate frames. Does not teleport bodies.
        std::vector<ScenarioReplayReferenceFrame> mReplayReferenceFrames;
        // A powered scenario drives motors and upright control. A passive
        // scenario observes the controller as non-actuating telemetry, as the
        // live game does for corpses.
        bool mPowered = true;
        // Route the equal-and-opposite gait reaction into the stance thigh so
        // the leg constraints carry it to the planted foot. Applying it to a
        // low-mass foot directly either pins the swing foot or launches the
        // stance foot beyond its contact-friction budget.
        bool mRouteGaitReactionThroughStanceLeg = true;
        // Experimental counterpart: apply the horizontal reaction to the
        // designated stance foot, never the swing foot. The controller's
        // support-friction clamp remains mandatory so this route cannot ask a
        // contact to transmit more shear than its configured Coulomb budget.
        bool mRouteGaitHorizontalReactionToStanceFoot = false;
        ScenarioTranslationRouting mTranslationRouting = ScenarioTranslationRouting::InternalForcePair;
        // Optional physical handover gate for experiments: Cartesian swing
        // assistance and single-support reaction routing begin only after the
        // COM projection enters the measured stance-foot contact hull. Joint
        // motor targets remain active.
        bool mRequireStanceSupportForSwingAssist = false;
        // Capture must lie laterally inside measured stance support. The COM
        // alternative is retained only as an offline diagnostic control.
        bool mUseCapturePointForStanceReadiness = true;
        bool mEnableBackwardCaptureRecovery = true;
        // Read-only observation, available for one collision step per update.
        bool mObserveSolverImpulses = true;
        // Offline diagnostic: apply internal force pairs and torque servos
        // during the constraint solve, including standing before gait begins.
        // Upright torques without a reaction body retain the external route.
        // This path must satisfy the same locomotion gates as the default.
        bool mUseSolverActuation = false;
        // Shared standing policy: contact-derived internal joint loads replace
        // pelvis force pairs and external upright assistance. False retains
        // the legacy route as an offline diagnostic control.
        bool mUseJointSupportControl = true;
        ScenarioSwingAssistAxes mSwingAssistAxes = ScenarioSwingAssistAxes::Full3D;
        ScenarioSwingHorizontalReaction mSwingHorizontalReaction = ScenarioSwingHorizontalReaction::SwingHip;
        // Experimental feed-forward balance: counter the airborne swing
        // chain's horizontal reaction with a pelvis-to-stance-foot force pair.
        // Net ragdoll force and moment remain zero; the planted foot must earn
        // locomotion through its ground contact.
        bool mCompensateSwingHorizontalReactionThroughStance = true;
        // SIMBICON-style experiment: the stance hip does not track the base
        // animation pose, leaving it available for balance reactions.
        bool mReserveStanceHipMotor = false;
        // Diagnostic counterpart: leave the swing hip motor free while the
        // Cartesian foot actuator tracks the physical trajectory.
        bool mDisableSwingHipMotor = false;
        // Replace both hip constraint motors during gait with a bounded
        // world-frame swing-thigh torque and an equal stance-thigh reaction.
        bool mUseWorldSwingHipControl = false;
        // After each solve, transfer the measured swing-hip motor reaction
        // from the pelvis to the reserved stance thigh. This remains an
        // internal angular-impulse pair.
        bool mCompensateSwingHipMotorImpulse = false;
        // Diagnostic isolation mode for validating the articulated swing-leg
        // motor chain without free-root balance or locomotion reactions.
        bool mPinPelvis = false;
        // Experimental double-support load transfer. Before the selected
        // stance owns the COM, lift a fraction of the other leg's measured
        // weight and react it on the stance foot. Zero preserves production.
        std::optional<float> mPreLiftSwingUnloadFraction;
        ScenarioLocomotionController mLocomotionController = ScenarioLocomotionController::Reference;
        ReferenceGaitProfile mReferenceGait;
        // Match the native player's direct travel input and separately
        // rate-limited body facing. False retains the NPC steering path.
        bool mReferenceGaitPlayerInput = false;
        // Historical stop fixtures omit measured velocity to reproduce their
        // original command-only deceleration. Production adapters supply it.
        bool mReferenceGaitMeasuredStopping = true;
        // Diagnostic for the reference route: the first swing foot rises to
        // its clearance and holds there, so single support can be measured
        // without a step.
        bool mReferenceGaitHoldSwingFoot = false;
        // Apply the hip strategy on the reference route. See
        // ActiveControllerProfile::mReferenceHipStrategy: the pelvis and
        // spine torque are computed every step and otherwise discarded.
        bool mReferenceHipStrategy = false;
        // Recorded in the telemetry identity; the simulation itself is seedless.
        std::uint64_t mSeed = 0;
        // Flat-floor diagnostic: use native shape bounds for landing height.
        bool mReferenceGaitGroundsPlacement = false;
        bool mReferenceGaitCoordinatesLegPose = false;
        // Pure plan observation before motor application; excluded from identity.
        std::function<void(std::uint64_t, const ReferenceGaitPlan&)> mReferenceGaitPlanObserver;
    };

    struct ScenarioImpulse
    {
        // Applied immediately before the physics update of this step.
        std::uint64_t mStep = 0;
        std::size_t mBodyIndex = 0;
        JPH::Vec3 mImpulse = JPH::Vec3::sZero();
        // Applied at the body's center of mass when absent.
        std::optional<JPH::RVec3> mWorldPoint;
    };

    // A gameplay killing blow converted through makeKillingImpulse and applied
    // through the body interface after the ragdoll has entered broadphase,
    // before the first fixed step.
    struct ScenarioKillingBlow
    {
        JPH::RVec3 mWorldPoint = JPH::RVec3::sZero();
        JPH::Vec3 mDirection = JPH::Vec3::sAxisY();
        float mHealthDamage = 0.0f;
    };

    struct ScenarioInputKeyframe
    {
        // First step this keyframe applies to. Keyframes must be sorted by
        // step and unique; a keyframe stays active until the next one.
        std::uint64_t mStep = 0;
        // Model-space base motor target in profile body order. When absent the
        // previous target remains active; the initial target is the supplied
        // skeleton pose (synthetic bind pose or a captured live pose). The
        // gait swing leg overrides this base pose while a stance is active.
        std::optional<RagdollPose> mMotorTarget;
        // Optional replay animation candidate. When supplied with mMotorTarget
        // (the locomotion candidate), select this during GettingUp/BlendBack
        // using the scenario's own pre-solve controller state.
        std::optional<RagdollPose> mRecoveryMotorTarget;
        // Offline authored recovery: bounded fraction of the settled standing
        // height. Replaces lift-only unweighting while GettingUp. Absent keeps
        // the native-compatible recovery route unchanged.
        std::optional<float> mRecoveryPelvisHeightScale;
        // Explicit offline hybrid recovery. The pelvis follows the recovery
        // clip while all other bodies stay dynamic. Never a physical-gait gate.
        bool mKinematicRecovery = false;
        // Final clip pose used once to align the recovery with the floor.
        std::optional<RagdollPose> mKinematicRecoveryTerminalPose;
        bool mRecoveryInterrupted = false;
        // Offline hybrid walking: animated motor target supplies the root path.
        // The pelvis alone is kinematic; a new impulse releases it first.
        bool mKinematicLocomotion = false;
        // Diagnostic phase: 0 ordinary, 1 animated walk, 2 PGC catch, 3 return.
        std::uint8_t mHybridStage = 0;
        // Optional per-joint child-minus-parent angular velocity in world axes,
        // radians/second, paired with a moving pose during standing tasks.
        // Empty means zero; otherwise one entry per body, zero for roots.
        // A new keyframe without velocities clears this command. Procedural
        // walking owns its motor reference and rejects this task input while
        // moving; DirectMotorTracking accepts it with a movement request.
        // The convention remains WORLD child-minus-parent angular velocity.
        // A local-reference caller must rotate by the measured parent's world
        // rotation before dispatch; hinge motors project onto their real axis.
        std::vector<JPH::Vec3> mMotorTargetAngularVelocities;
        // DirectMotorTracking or ProductionResidual. Empty is an exact
        // no-correction reference;
        // otherwise one rotation vector per body, in the parent's local axes.
        // Each corrected relative rotation is exp(offset) * referenceRelative,
        // rebuilt down the hierarchy so a parent correction does not change
        // the reference's descendant-local targets. Root/passive joints must
        // have zero offsets. Hinges retain their ordinary target projection;
        // callers must use the actual hinge axis for an effective scalar action.
        // Existing joint/torque limits still apply. The offset is persistent
        // until the next keyframe, where an empty vector clears it; it is never
        // accumulated into the stored reference pose. Journal the resolved
        // keyframes for exact offline replay. No native replay format is added.
        static constexpr float sMaximumDirectMotorRotationOffset = 0.5f;
        // ProductionResidual corrects the current final production pose, not
        // an externally generated pose. Subsequent production world targets
        // can override pose channels; it adds no velocity or force authority.
        static constexpr float sMaximumProductionMotorRotationOffset = 0.1f;
        std::vector<JPH::Vec3> mMotorTargetRotationOffsets;
        // Optional world-space support reference paired with an authored pose
        // trajectory. Applies during standing joint support, not gait control.
        // Like other inputs, absent in the next keyframe means inactive.
        std::optional<SupportMotionReference> mSupportMotion;
        // Requested gait velocity in world units per second. This is the
        // controller's native locomotion input; a zero request reproduces the
        // live idle behaviour.
        JPH::Vec3 mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        // Where the body points; zero faces the direction of travel
        // (ReferenceGaitPlanInput::mFacing, plan item 2.3a).
        JPH::Vec3 mFacing = JPH::Vec3::sZero();
        // Actuation switches for negative controls. They mask what the
        // harness applies; the controller still computes its full output so
        // the recorded telemetry shows what was suppressed.
        bool mMotorsEnabled = true;
        bool mTranslationActuationEnabled = true;
        bool mSwingFootActuationEnabled = true;
        bool mUprightTorqueEnabled = true;
        // Fraction of the controller's already finite joint-motor and
        // support/upright budget available on this step. Reaction policy may
        // reduce this value, but it cannot raise authored limits.
        float mStabilizationStrength = 1.0f;
        bool mRecoveryRequested = false;
        bool mRecoveryAnimationFinished = false;
        // One-shot external disturbances at this keyframe's step, applied
        // after ScenarioScript::mImpulses and before physics. Each impulse
        // must name mStep; unlike persistent controls these are consumed once.
        // A generated replacement key also replaces the scheduled key's
        // disturbances. Record the resolved keyframes for exact replay.
        std::vector<ScenarioImpulse> mImpulses;
        // Direct native API events between solves, before motor planning.
        // One-shot, world-space, with the same step/body validation as impulses.
        std::vector<ScenarioImpulse> mPrePlanningImpulses;
    };

    struct ScenarioScript
    {
        // Simulation time. The step count is round(duration / fixed timestep).
        float mDurationSeconds = 1.0f;
        // Optional uniformly spaced model-space samples for one complete gait
        // cycle, beginning at left stance. Physics phase selects the sample;
        // the scenario never advances a separate animation clock.
        std::vector<RagdollPose> mGaitPoseCycle;
        std::vector<ScenarioInputKeyframe> mKeyframes;
        std::vector<ScenarioImpulse> mImpulses;
        std::optional<ScenarioKillingBlow> mKillingBlow;
    };

    struct ScenarioBodyRecord
    {
        Transform mModelTransform;
        JPH::RVec3 mWorldPosition = JPH::RVec3::sZero();
        JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
        JPH::Quat mWorldRotation = JPH::Quat::sIdentity();
        JPH::Vec3 mLinearVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mAngularVelocity = JPH::Vec3::sZero();
        float mMass = 0.0f;
        bool mActive = false;
    };

    // Per-step record of the reference locomotion plan. Motor impulses and
    // utilizations for every joint are in the shared joint-support fields.
    struct ReferenceGaitFrameRecord
    {
        ReferenceGaitStage mStage = ReferenceGaitStage::Idle;
        float mTimeInStage = 0.0f;
        bool mSwingPoseApplied = false;
        std::optional<JPH::RVec3> mPlacement;
        JPH::Vec3 mRequestedAcceleration = JPH::Vec3::sZero();
        JPH::Vec3 mPelvisOrientationError = JPH::Vec3::sZero();
        JPH::Vec3 mBranchReaction = JPH::Vec3::sZero();
        JPH::Vec3 mStanceHipTorque = JPH::Vec3::sZero();
        float mLeftLoadFraction = 0.0f;
        float mRightLoadFraction = 0.0f;
        float mPlannedStanceFootLoad = 0.0f;
        // Largest post-solve motor axis utilization over every joint.
        float mMaxMotorUtilization = 0.0f;
        std::size_t mMaxMotorUtilizationBody = 0;
        // Commanded load torque and solved motor torque (impulse over the
        // step) for the stance hip and stance ankle, world frame.
        JPH::Vec3 mStanceHipLoadTorque = JPH::Vec3::sZero();
        JPH::Vec3 mStanceHipMotorTorque = JPH::Vec3::sZero();
        JPH::Vec3 mStanceAnkleLoadTorque = JPH::Vec3::sZero();
        JPH::Vec3 mStanceAnkleMotorTorque = JPH::Vec3::sZero();
        std::size_t mStanceHipBody = 0;
        std::size_t mStanceAnkleBody = 0;
        float mTransferVelocity = 0.0f;
        float mTransferReferenceVelocity = 0.0f;
        bool mSwingReachLimited = false;
        // World COM Z of a joint-valid coordinated pose; absent on fallback.
        std::optional<double> mCoordinatedPelvisTargetZ;
        // Diagnostics (not part of the telemetry identity).
        JPH::Vec3 mSwingHipFeedForwardTorque = JPH::Vec3::sZero();
        JPH::Vec3 mSwingKneeFeedForwardTorque = JPH::Vec3::sZero();
        JPH::Vec3 mSwingHipRateTarget = JPH::Vec3::sZero();
        JPH::Vec3 mSwingHipMeasuredRate = JPH::Vec3::sZero();
        std::optional<JPH::RVec3> mPlacementCenterOfMass;
        JPH::Vec3 mPlacementVelocity = JPH::Vec3::sZero();
        // Reactive step (ReferenceGaitProfile::mEmergencyCaptureMargin):
        // the current swing was started by the capture rule, and how many
        // such swings the episode has started so far.
        bool mEmergencyStep = false;
        std::uint32_t mEmergencyStepCount = 0;
        // Final-command feasibility gate engagement: hip/ankle motor targets
        // clamped onto the margin-shrunk SwingTwist cone this frame (zero when
        // ReferenceGaitProfile::mFinalCommandFeasibilityStanceMargin/SwingMargin
        // are 0/off).
        std::uint32_t mFeasibilityClampedCount = 0;
        // Frame-to-frame change of the stance knee target rotation while the
        // SAME leg keeps the stance role (radians); -1 when the stance leg
        // changed or no previous target exists. Exposes the takeoff command
        // discontinuity (E10) that mStanceKneeEaseDuration removes.
        // Actual scalar constraint-space hinge motor command; -1 jump = unavailable.
        float mStanceKneeTargetJump = -1.0f;
        float mStanceKneeModelTargetJumpLegacy = -1.0f;
        std::optional<float> mStanceKneeHingeTarget;
    };

    struct ScenarioFrame
    {
        // End-of-step history, after reference-frame rebase and observation reset.
        std::optional<SwingTrajectoryHistory> mSwingTrajectoryHistory;
        std::optional<RagdollSolverImpulse> mSolverImpulse;
        std::optional<ReferenceGaitFrameRecord> mReferenceGait;
        std::optional<JPH::RVec3> mSolverCenterOfPressure;
        // Unavailable for pinned/kinematic diagnostic bodies.
        std::optional<RagdollMomentum> mMomentum;
        std::uint64_t mStep = 0;
        // Simulation time at the end of this step.
        float mTime = 0.0f;

        // Inputs as applied before this step's physics update.
        JPH::Vec3 mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        bool mMotorsEnabled = true;
        bool mTranslationActuationEnabled = true;
        bool mSwingFootActuationEnabled = true;
        bool mUprightTorqueEnabled = true;
        float mStabilizationStrength = 1.0f;
        std::uint32_t mImpulseCount = 0;
        bool mRootKinematic = false;
        bool mKinematicLocomotion = false;
        std::uint8_t mHybridStage = 0;
        std::optional<float> mRootReleaseVelocityError;
        std::optional<Transform> mKinematicRootTarget;
        // Gait output consumed by this step's motor/actuation pass. The
        // observation-side mGait below is produced after this physics step.
        std::optional<GaitControllerOutput> mActuationGait;
        GaitActuationStage mActuationStage = GaitActuationStage::Inactive;
        GaitActuationStage mUprightActuationStage = GaitActuationStage::Inactive;
        std::optional<UprightControllerOutput> mAppliedUpright;
        bool mMotorsApplied = false;
        // The ordinary planned support loads; actual motor impulses are in
        // mJointSupportMotorAngularImpulses, including prepared-step actuation.
        std::optional<JointSupportOutput> mJointSupport;
        std::optional<SupportMotionReference> mSupportMotion;
        // Also populated by DirectMotorTracking for actual ordinary motor
        // impulses, without implying that joint support loads were applied.
        std::vector<JPH::Vec3> mJointSupportMotorAngularImpulses;
        std::vector<float> mJointSupportMotorUtilizations;
        float mMaxAnchorSeparation = 0.0f;
        bool mStanceHipMotorDisabled = false;
        bool mSwingHipMotorDisabled = false;
        bool mStanceHipAxisReleased = false;
        bool mSwingHipAxisReleased = false;
        bool mSwingHipWorldControlApplied = false;
        bool mSwingHipWorldTorqueSaturated = false;
        JPH::Vec3 mAppliedSwingHipWorldTorque = JPH::Vec3::sZero();
        std::optional<std::size_t> mDisabledStanceHipBody;
        std::optional<std::size_t> mSwingHipMotorBody;
        // World-space angular impulse from the native joint motor or the
        // bounded replacement servo when mSwingHipAxisReleased is true.
        JPH::Vec3 mSwingHipMotorAngularImpulse = JPH::Vec3::sZero();
        float mSwingHipMotorPeakAxisUtilization = 0.0f;
        JPH::Vec3 mSwingKneeMotorAngularImpulse = JPH::Vec3::sZero();
        float mSwingKneeMotorPeakAxisUtilization = 0.0f;
        JPH::Vec3 mSwingFootJointMotorAngularImpulse = JPH::Vec3::sZero();
        float mSwingFootJointMotorPeakAxisUtilization = 0.0f;
        // Hip, knee, ankle; world impulses on each child, after the solve.
        std::array<JPH::Vec3, 3> mStanceLegMotorAngularImpulses{
            JPH::Vec3::sZero(), JPH::Vec3::sZero(), JPH::Vec3::sZero() };
        std::array<float, 3> mStanceLegMotorPeakAxisUtilizations{};
        std::optional<MotorTargetTelemetry> mSwingHipMotorTarget;
        std::optional<MotorTargetTelemetry> mSwingKneeMotorTarget;
        std::optional<MotorTargetTelemetry> mSwingFootJointMotorTarget;
        // Pre-physics commands, indexed by body; root/unavailable joints are
        // null. Empty when no motors were applied. Unlike mBodies and contact
        // observations, these describe the start of this frame's physics step.
        // Prepared-step application replaces only the velocity command with
        // the actual refreshed target; the original pose/error is retained.
        std::vector<std::optional<MotorTargetTelemetry>> mJointMotorTargets;
        // Parent-local rotations from the complete model-space pose passed to
        // driveToPoseUsingMotors for this step. Diagnostic teacher telemetry;
        // later per-joint target limits or world-orientation overrides remain
        // visible separately in mJointMotorTargets.
        std::vector<JPH::Quat> mDrivenMotorLocalRotations;
        JPH::Vec3 mAppliedStanceHipCompensationImpulse = JPH::Vec3::sZero();
        bool mUprightApplied = false;
        bool mSwingFootControlApplied = false;
        bool mStanceSupportsCenterOfMass = false;
        float mPreLiftSwingUnloadForce = 0.0f;
        bool mSwingAssistReady = false;
        // Vertical unloading remains within the swing chain after lift-off;
        // horizontal tracking can independently route through a selected
        // reaction body. Recording both prevents a component split from being
        // hidden in aggregate force telemetry.
        std::size_t mSwingFootVerticalReactionBody = 0;
        std::size_t mSwingFootHorizontalReactionBody = 0;
        std::optional<SwingFootControllerOutput> mSwingFootControl;
        std::vector<std::size_t> mVerticalReactionBodies;
        std::vector<std::size_t> mHorizontalReactionBodies;
        std::optional<std::size_t> mAngularReactionBody;
        std::optional<std::size_t> mSecondaryAngularReactionBody;
        JPH::Vec3 mAppliedPelvisForce = JPH::Vec3::sZero();
        JPH::Vec3 mAppliedPelvisTorque = JPH::Vec3::sZero();
        JPH::Vec3 mAppliedSpineTorque = JPH::Vec3::sZero();
        // Complete actuator wrench about the ragdoll center of mass. Equal and
        // opposite forces make net force zero, but an offset pair can still
        // inject angular momentum through its force-pair moment.
        JPH::Vec3 mAppliedNetForce = JPH::Vec3::sZero();
        JPH::Vec3 mAppliedForcePairMoment = JPH::Vec3::sZero();
        JPH::Vec3 mAppliedSwingForcePairMoment = JPH::Vec3::sZero();
        JPH::Vec3 mAppliedUprightForcePairMoment = JPH::Vec3::sZero();
        JPH::Vec3 mAppliedGroundReactionForce = JPH::Vec3::sZero();
        JPH::Vec3 mAppliedGroundReactionMoment = JPH::Vec3::sZero();
        JPH::Vec3 mAppliedTorqueSum = JPH::Vec3::sZero();
        JPH::Vec3 mAppliedNetActuationMoment = JPH::Vec3::sZero();
        // Swing-foot motor target in model space when the gait overrode the
        // base pose for this step.
        std::optional<JPH::Vec3> mSwingFootMotorTarget;
        // The trajectory target after compensating for the measured pelvis
        // frame, followed by the reachable target emitted by two-bone IK.
        std::optional<JPH::Vec3> mResolvedSwingFootTarget;
        std::optional<JPH::Vec3> mSwingFootPoseTarget;

        // Observation after this step's physics update. The controller and
        // gait outputs recorded here drive the following step, as in the live
        // adapter.
        std::vector<GroundContact> mContacts;
        SupportEstimate mSupport;
        ActiveControllerInput mControllerInput;
        ActiveControllerOutput mController;
        GaitControllerOutput mGait;
        std::optional<JPH::RVec3> mSwingFootTarget;
        bool mBackwardRecoveryStep = false;
        float mSwingFootHorizontalPlacementError = 0.0f;
        float mSwingFootVerticalPlacementError = 0.0f;
        float mSwingFootRelativeVerticalSpeed = 0.0f;
        std::optional<UprightControllerInput> mUprightInput;
        std::optional<UprightControllerOutput> mUpright;
        JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
        JPH::Vec3 mCenterOfMassVelocity = JPH::Vec3::sZero();
        float mPelvisUpDot = 1.0f;
        float mTorsoUpDot = 1.0f;
        float mPelvisHeight = 0.0f;
        SettledPelvisHeightOutput mPelvisHeightTracker;
        bool mRagdollActive = false;
        std::vector<ScenarioBodyRecord> mBodies;
    };

    struct ScenarioLandmarks
    {
        std::size_t mPelvis = 0;
        std::size_t mTorso = 0;
        std::size_t mLeftFoot = 0;
        std::size_t mRightFoot = 0;
    };

    struct ScenarioResult
    {
        ScenarioSettings mSettings;
        std::uint64_t mProfileHash = 0;
        std::uint64_t mStepCount = 0;
        std::vector<std::string> mBodyNames;
        ScenarioLandmarks mLandmarks;
        float mTargetPelvisHeight = 0.0f;
        std::optional<KillingImpulse> mKillingImpulse;
        std::vector<ScenarioFrame> mFrames;
        // Physical state before the first solve, for creation/replay parity.
        RagdollTelemetry mInitialTelemetry;
        std::vector<std::vector<MotorTargetTelemetry>> mStartupMotorTargets;
        // Non-empty when the scenario could not be constructed or aborted.
        std::vector<std::string> mErrors;

        bool completed() const { return mErrors.empty() && mFrames.size() == mStepCount; }
    };

    // Called before each physics step, after scheduled keyframes. The observation
    // is the previous completed frame, or null at step zero; it is valid only
    // during the call. A returned keyframe replaces the active input and must
    // name the current step. Nullopt preserves the scheduled/persistent input.
    // The caller owns policy state and records returned keyframes for replay,
    // replacing scheduled entries at the same step. Replacement happens before
    // applying either entry's motor target or one-shot impulses, so overridden
    // scripted commands cannot leak into the physical run outside that replay.
    // No mutable world or alternate actuation path is exposed.
    using ScenarioInputProvider
        = std::function<std::optional<ScenarioInputKeyframe>(std::uint64_t, const ScenarioFrame*)>;

    ScenarioResult runRagdollScenario(const SkeletonDescriptor& skeleton, const RagdollProfile& profile,
        const ScenarioSettings& settings, const ScenarioScript& script, const ScenarioInputProvider& inputProvider = {});

    // Portable little-endian encoding of every recorded field using IEEE-754
    // bit patterns. Two runs are byte-equivalent when these encodings match.
    std::vector<std::byte> encodeScenarioTelemetry(const ScenarioResult& result);
    // Deterministic, one-row-per-fixed-step trace for controller diagnosis.
    // The caller owns persistence so the core stays free of runtime paths.
    std::string formatScenarioTraceCsv(const ScenarioResult& result);
    std::uint64_t hashScenarioTelemetry(const ScenarioResult& result);
    bool haveIdenticalScenarioTelemetry(const ScenarioResult& expected, const ScenarioResult& actual);

    // A shared-world standing scenario. Unlike a batch of independent
    // scenarios, every actor is advanced by the same PhysicsSystem update and
    // can collide with every other actor. This catches contact-listener
    // routing, collision-group, capacity, cleanup, and aggregate-stability
    // defects that single-ragdoll runs cannot observe.
    struct CrowdScenarioImpulse
    {
        std::uint64_t mStep = 0;
        std::size_t mActorIndex = 0;
        std::size_t mBodyIndex = 0;
        JPH::Vec3 mImpulse = JPH::Vec3::sZero();
        std::optional<JPH::RVec3> mWorldPoint;
    };

    struct CrowdScenarioSettings
    {
        ScenarioSettings mActorSettings;
        std::size_t mActorCount = 4;
        float mActorSpacing = 80.0f;
        float mDurationSeconds = 1.0f;
        // Sorted, per-actor disturbances applied before the selected fixed
        // step. All actors still share one PhysicsSystem update.
        std::vector<CrowdScenarioImpulse> mImpulses;
    };

    struct CrowdActorFrame
    {
        SupportEstimate mSupport;
        ActiveControllerMode mMode = ActiveControllerMode::Standing;
        JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
        JPH::Vec3 mCenterOfMassVelocity = JPH::Vec3::sZero();
        float mPelvisUpDot = 1.0f;
        float mMaxBodySpeed = 0.0f;
        std::uint32_t mImpulseCount = 0;
        bool mMotorsApplied = false;
        bool mUprightApplied = false;
        bool mJointSupportApplied = false;
    };

    struct CrowdScenarioFrame
    {
        std::uint64_t mStep = 0;
        float mTime = 0.0f;
        std::size_t mCrossActorContactCount = 0;
        std::vector<CrowdActorFrame> mActors;
    };

    struct CrowdScenarioResult
    {
        CrowdScenarioSettings mSettings;
        std::uint64_t mProfileHash = 0;
        std::uint64_t mStepCount = 0;
        std::vector<CrowdScenarioFrame> mFrames;
        std::vector<std::string> mErrors;

        bool completed() const { return mErrors.empty() && mFrames.size() == mStepCount; }
    };

    CrowdScenarioResult runStandingCrowdScenario(
        const SkeletonDescriptor& skeleton, const RagdollProfile& profile, const CrowdScenarioSettings& settings);
    std::vector<std::byte> encodeCrowdScenarioTelemetry(const CrowdScenarioResult& result);
    // One row per actor per fixed step, suitable for state occupancy and
    // transition-matrix tooling without parsing human-readable test output.
    std::string formatCrowdScenarioTraceCsv(const CrowdScenarioResult& result);
    std::uint64_t hashCrowdScenarioTelemetry(const CrowdScenarioResult& result);
    bool haveIdenticalCrowdScenarioTelemetry(const CrowdScenarioResult& expected, const CrowdScenarioResult& actual);

    struct ScenarioStepStatistics
    {
        std::size_t mLeftStanceCount = 0;
        std::size_t mRightStanceCount = 0;
        float mMeanLeftStanceDuration = 0.0f;
        float mMeanRightStanceDuration = 0.0f;
        // Ratio of the shorter to the longer mean stance duration; one is
        // perfectly symmetric, zero means one side never took a step.
        float mDurationSymmetry = 0.0f;
        std::size_t mTimedOutPhases = 0;
    };

    // First observable fixed-step boundary for each physical gait stage. The
    // stages are derived entirely from recorded telemetry and add no control
    // thresholds. Missing values identify the exact stage a scenario never
    // reached without folding several mechanisms into one walk assertion.
    struct ScenarioGaitMilestones
    {
        std::optional<std::uint64_t> mStanceStartedStep;
        std::optional<std::uint64_t> mStanceReadyStep;
        std::optional<std::uint64_t> mLiftOffStep;
        std::optional<std::uint64_t> mContactCandidateStep;
        std::optional<std::uint64_t> mStanceTransferStep;
        std::optional<std::uint64_t> mPhaseTimeoutStep;
        std::optional<std::uint64_t> mClosestApproachStep;
        float mClosestApproachError = 0.0f;
    };

    // One record for every physical stance observed in a scenario. Unlike
    // ScenarioGaitMilestones, which preserves the original first-step gate,
    // this sequence exposes handover continuity across multiple steps.
    struct ScenarioGaitPhaseMilestones
    {
        GaitPhase mPhase = GaitPhase::Idle;
        std::uint64_t mStanceStartedStep = 0;
        std::optional<std::uint64_t> mStanceReadyStep;
        std::optional<std::uint64_t> mLiftOffStep;
        std::optional<std::uint64_t> mContactCandidateStep;
        std::optional<std::uint64_t> mStanceTransferStep;
        std::optional<std::uint64_t> mPhaseTimeoutStep;
        std::optional<std::uint64_t> mClosestApproachStep;
        float mClosestApproachError = 0.0f;
    };

    // Physical invariants derived from recorded frames. They are the gates a
    // correct controller must satisfy; they are never thresholds the
    // controller reads.
    struct ScenarioInvariants
    {
        std::size_t mFrameCount = 0;
        // Frames with no upward ground contact on any body.
        std::size_t mAirborneFrames = 0;
        std::size_t mMaxConsecutiveAirborneFrames = 0;
        std::size_t mSolverObservedFrames = 0;
        std::size_t mFreeVelocityLimitedFrames = 0;
        std::size_t mSolverVelocityLimitedFrames = 0;
        float mMaxFreeLinearLimitImpulse = 0.0f;
        float mMaxFreeAngularLimitImpulse = 0.0f;
        std::size_t mPelvisForceSaturatedFrames = 0;
        std::size_t mPelvisMagnitudeSaturatedFrames = 0;
        std::size_t mPelvisHorizontalForceSaturatedFrames = 0;
        std::size_t mPelvisVerticalForceSaturatedFrames = 0;
        std::size_t mPelvisTorqueSaturatedFrames = 0;
        std::size_t mSpineTorqueSaturatedFrames = 0;
        std::size_t mSwingHipMotorSaturatedFrames = 0;
        std::size_t mSwingHipWorldTorqueSaturatedFrames = 0;
        float mMaxAppliedNetForce = 0.0f;
        float mMaxForcePairMoment = 0.0f;
        float mMaxSwingForcePairMoment = 0.0f;
        float mMaxUprightForcePairMoment = 0.0f;
        float mMaxGroundReactionForce = 0.0f;
        float mMaxGroundReactionMoment = 0.0f;
        float mMaxNetActuationMoment = 0.0f;
        float mMaxSwingHipMotorTorque = 0.0f;
        float mMaxSwingHipMotorPeakAxisUtilization = 0.0f;
        float mMaxSwingHipWorldTorque = 0.0f;
        float mMaxStanceHipCompensationTorque = 0.0f;
        // Horizontal centre-of-mass travel of a foot between consecutive
        // frames while it was supported in both.
        float mMaxStanceFootSlide = 0.0f;
        float mTotalStanceFootSlide = 0.0f;
        std::size_t mRequestedFrames = 0;
        float mMeanRequestedSpeed = 0.0f;
        // Centre-of-mass velocity projected on the requested direction.
        float mMeanAchievedSpeed = 0.0f;
        float mFinalHalfSecondAchievedSpeed = 0.0f;
        float mMaxTrackingError = 0.0f;
        ScenarioStepStatistics mSteps;
        std::size_t mFallingFrames = 0;
        std::size_t mFallenFrames = 0;
        float mMinPelvisUpDot = 1.0f;
        float mMaxBodySpeed = 0.0f;
        std::size_t mImpulseFrames = 0;
        // Gait transitions within one second after any scripted impulse.
        std::size_t mStepsWithinSecondAfterImpulse = 0;
        bool mFellAfterImpulse = false;
    };

    ScenarioInvariants summarizeScenarioInvariants(const ScenarioResult& result);

    struct ScenarioGaitStageSummary
    {
        GaitActuationStage mStage = GaitActuationStage::Inactive;
        std::size_t mFrames = 0;
        float mDuration = 0.0f;
        float mMeanRequestedSpeed = 0.0f;
        float mMeanAchievedSpeed = 0.0f;
        std::size_t mBrakingFrames = 0;
        std::size_t mAirborneFrames = 0;
        std::size_t mFallingFrames = 0;
        std::size_t mVerticalSaturationFrames = 0;
        std::size_t mSupportMomentSaturationFrames = 0;
        std::size_t mSwingHipSaturationFrames = 0;
        std::size_t mSwingKneeSaturationFrames = 0;
        std::size_t mSwingAnkleSaturationFrames = 0;
        std::size_t mSolverObservedFrames = 0;
        std::size_t mFreeVelocityLimitedFrames = 0;
        std::size_t mSolverVelocityLimitedFrames = 0;
        float mMaxFreeLinearLimitImpulse = 0.0f;
        float mMaxFreeAngularLimitImpulse = 0.0f;
        float mMaxNetForce = 0.0f;
        float mMaxNetMoment = 0.0f;
        float mMaxStanceSlide = 0.0f;
    };

    // Motor and upright policies consume different fixed-step observations.
    // Keep their stage attribution separate instead of charging a newly
    // observed handover for the preceding step's force or motor saturation.
    std::vector<ScenarioGaitStageSummary> summarizeScenarioGaitStages(
        const ScenarioResult& result, bool uprightStages = false);
    std::string formatScenarioGaitStagesCsv(const ScenarioResult& result);
    ScenarioGaitMilestones summarizeScenarioGaitMilestones(const ScenarioResult& result);
    std::vector<ScenarioGaitPhaseMilestones> summarizeScenarioGaitPhases(const ScenarioResult& result);
}

#endif
