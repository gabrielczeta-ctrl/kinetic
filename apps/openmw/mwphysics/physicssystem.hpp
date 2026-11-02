#ifndef OPENMW_MWPHYSICS_PHYSICSSYSTEM_H
#define OPENMW_MWPHYSICS_PHYSICSSYSTEM_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>

#include <ragdollcore/profile.hpp>
#include <ragdollcore/weaponswing.hpp>
#include <ragdollcore/balancemetrics.hpp>
#include <ragdollcore/balanceepisodes.hpp>
#include <ragdollcore/combatstate.hpp>
#include <ragdollcore/dueltactics.hpp>
#include <ragdollcore/physicaldefense.hpp>

#include <osg/BoundingBox>
#include <osg/Matrixf>
#include <osg/Quat>
#include <osg/Timer>
#include <osg/Vec3f>
#include <osg/ref_ptr>

#include <components/vfs/pathutil.hpp>

#include "../mwworld/kineticnet.hpp"
#include "../mwworld/ptr.hpp"

#include "joltlisteners.hpp"
#include "raycasting.hpp"

namespace JPH
{
    class JobSystem;
    class TempAllocatorImpl;
    class PhysicsSystem;
    class BodyInterface;
    class BodyLockInterfaceLocking;
}

namespace osg
{
    class Group;
    class Object;
    class Stats;
}

namespace MWRender
{
    class JoltDebugDrawer;
}

namespace MWMechanics
{
    struct RagdollAttackState;
}

namespace RagdollCore
{
    struct RagdollProfile;
    struct Blow;
}

namespace SceneUtil
{
    class Skeleton;
    class KeyframeHolder;
}

namespace RagdollProfile
{
    struct AnimationCycleSample;
}

namespace Resource
{
    class PhysicsShapeManager;
    class ResourceSystem;
}

namespace MWPhysics
{
    class MWWater;
    class HeightField;
    class Object;
    class Actor;
    class PhysicsTaskScheduler;
    class Projectile;
    struct PassiveRagdollEntry;
    struct PassiveRagdollMotionHistory;
    struct PassiveRagdollReplayRecording;
    enum ScriptedCollisionType : char;

    struct PendingRagdollImpact
    {
        osg::Vec3f mWorldPoint;
        osg::Vec3f mDirection;
        float mHealthDamage = 0.0f;
    };

    struct PassiveRagdollBodyState
    {
        std::string mName;
        osg::Vec3f mPosition;
        osg::Vec3f mCenterOfMass;
        osg::Vec3f mTargetPosition;
        std::optional<osg::Quat> mTargetRotation;
        // Share of this body's parent-joint motor budget spent in the last
        // solved step; negative when the step measured none.
        float mMotorUtilization = -1.0f;
        osg::Vec3f mRenderPosition;
        osg::Vec3f mPhysicsRenderPosition;
        std::optional<osg::Vec3f> mSkeletonPosition;
        osg::Quat mRotation;
        osg::Vec3f mLinearVelocity;
        osg::Vec3f mAngularVelocity;
        float mMass = 0.0f;
        bool mActive = false;
    };

    struct PassiveRagdollConstraintState
    {
        std::string mParentBody;
        std::string mChildBody;
        float mAnchorSeparation = 0.0f;
        std::optional<float> mHingeAngle;
        std::optional<float> mHingeMinimum;
        std::optional<float> mHingeMaximum;
    };

    struct PassiveRagdollSupportState
    {
        bool mLeftFootSupported = false;
        bool mRightFootSupported = false;
        bool mCenterOfMassSupported = false;
        std::size_t mFootContactCount = 0;
        std::size_t mGroundContactCount = 0;
        osg::Vec3f mCenter;
        osg::Vec3f mGroundCenter;
        float mCenterOfMassDistance = 0.0f;
        float mCenterOfMassHeight = 0.0f;
        std::optional<float> mFriction;
    };

    struct PassiveRagdollControllerState
    {
        std::string mMode;
        std::optional<std::string> mRecoveryDirection;
        float mTimeInMode = 0.0f;
        float mFallingExitCandidateTime = 0.0f;
        float mPelvisUpDot = 0.0f;
        float mTargetPelvisUpDot = 0.0f;
        float mTorsoUpDot = 0.0f;
        float mTorsoFacingUpDot = 0.0f;
        float mPelvisHeight = 0.0f;
        float mCenterOfMassSpeed = 0.0f;
        float mVelocityTrackingError = 0.0f;
        bool mRootExternallyOwned = false;
        std::optional<float> mOwnedRootVerticalVelocity;
        osg::Vec3f mCenterOfMassVelocity;
        osg::Vec3f mTargetHorizontalVelocity;
        osg::Vec3f mRawTargetHorizontalVelocity;
        osg::Vec3f mPlannedHorizontalVelocity;
        bool mDriveJointMotors = false;
        bool mJointMotorsApplied = false;
        bool mJointSupportControlApplied = false;
        bool mJointSupportContact = false;
        float mJointSupportMotorUtilization = 0;
        float mJointMotorPeakUtilization = 0;
        float mPelvisHeightCandidateTime = 0.0f;
        bool mPelvisHeightCandidateValid = false;
        bool mPelvisHeightSettled = false;
        bool mPelvisHeightFrozen = false;
        std::optional<float> mTargetPelvisHeight;
        std::optional<float> mUprightTargetPelvisHeight;
        bool mRecoveryUnweight = false;
        osg::Vec3f mPelvisForce;
        osg::Vec3f mPelvisTorque;
        osg::Vec3f mSpineTorque;
        bool mPelvisForceSaturated = false;
        bool mPelvisMagnitudeSaturated = false;
        bool mPelvisHorizontalForceSaturated = false;
        bool mPelvisVerticalForceSaturated = false;
        bool mPelvisTorqueSaturated = false;
        bool mSpineTorqueSaturated = false;
        bool mUprightControlApplied = false;
        bool mAnimationAssistedActuation = false;
        bool mLocomotionInterrupted = false;
        bool mRecoveryRequested = false;
        // Plan item 1.6 diagnostic. Total heading travel (radians) summed
        // per FIXED step over the actor's lifetime, for the gameplay
        // request, the steering command and the ragdoll reference frame.
        // All three should advance at the same rate per unit of simulated
        // time whatever the render frame rate; the one that does not is the
        // frame-rate path (frame-rate-turning-2026-09-07.txt).
        float mRequestHeadingTravel = 0.0f;
        float mCommandHeadingTravel = 0.0f;
        float mReferenceYawTravel = 0.0f;
        // Same method applied to the two remaining per-rendered-frame
        // inputs: the authored base pose the plan builds its targets from
        // (summed absolute joint rotation change) and the reference height.
        float mBasePoseTravel = 0.0f;
        float mReferenceHeightTravel = 0.0f;
        // Horizontal travel of the ragdoll reference origin. The gameplay
        // actor advances once per rendered frame while physics may take
        // several fixed steps, so a frame-rate path could live here.
        float mReferenceGroundTravel = 0.0f;
        std::uint64_t mFixedSteps = 0;
        bool mRecoveryAnimationStarted = false;
        bool mRecoveryAnimationFinished = false;
        std::string mGaitPhase;
        std::string mGaitStage;
        std::string mGaitActuationStage;
        std::string mUprightActuationStage;
        bool mStanceHipAxisReleased = false;
        bool mSwingHipAxisReleased = false;
        osg::Vec3f mAppliedPelvisForce;
        bool mAppliedVerticalForceSaturated = false;
        bool mAppliedSupportMomentSaturated = false;
        float mGaitTimeInPhase = 0.0f;
        float mGaitPhaseProgress = 0.0f;
        float mGaitSwingTimeSinceLift = 0.0f;
        float mGaitSwingContactCandidateTime = 0.0f;
        bool mGaitStanceReady = false;
        bool mGaitSwingFootLifted = false;
        bool mGaitSwingTouchdownCandidate = false;
        bool mGaitTransitioned = false;
        bool mGaitPhaseTimedOut = false;
        std::optional<osg::Vec3f> mSwingFootTarget;
        bool mBackwardRecoveryStep = false;
        std::optional<osg::Vec3f> mSwingFootTrajectoryModel;
        std::optional<osg::Vec3f> mSwingFootResolvedModel;
        std::optional<osg::Vec3f> mSwingFootPoseModel;
        std::optional<osg::Vec3f> mSwingFootControllerWorldTarget;
        std::optional<float> mSwingHipMotorError;
        std::optional<float> mSwingHipMotorClamp;
        std::optional<float> mSwingKneeMotorError;
        std::optional<float> mSwingKneeMotorClamp;
        std::optional<float> mSwingKneeCurrentAngle;
        std::optional<float> mSwingKneeTargetAngle;
        std::optional<float> mSwingAnkleMotorError;
        std::optional<float> mSwingAnkleMotorClamp;
        bool mSwingAnkleWorldMotor = false;
        std::optional<float> mSwingHipMotorUtilization;
        std::optional<float> mSwingKneeMotorUtilization;
        std::optional<float> mSwingAnkleMotorUtilization;
        std::optional<osg::Vec3f> mSwingFootForce;
        bool mSwingFootForceSaturated = false;
        bool mSwingFootControlApplied = false;
        float mPreLiftSwingUnloadForce = 0.0f;
    };

    struct PoweredRagdollRecoveryState
    {
        std::optional<bool> mFaceUp;
        bool mAnimationStarted = false;
        bool mAnimationFinished = false;
    };

    struct PassiveRagdollContactState
    {
        std::size_t mBodyIndex = 0;
        osg::Vec3f mPosition;
        osg::Vec3f mNormal;
        float mPenetrationDepth = 0;
        std::optional<float> mNormalImpulse;
        std::optional<float> mFriction;
    };

    // Lifetime peaks sampled after every physics solve, independently of render/Lua polling.
    struct PassiveRagdollStepQuality
    {
        std::uint64_t mSteps = 0;
        float mMaximumAnchorSeparation = 0;
        std::uint64_t mMaximumAnchorStep = 0;
        std::size_t mMaximumAnchorParentBody = 0;
        std::size_t mMaximumAnchorChildBody = 0;
        float mMaximumHingeViolation = 0;
    };

    struct PassiveRagdollWeaponState
    {
        struct ContactEvent
        {
            std::uint64_t mContactIdentity = 0;
            std::uint64_t mAttackerActorIdentity = 0;
            std::optional<std::uint64_t> mTargetActorIdentity;
            std::string mWeaponRecordId;
            std::uint32_t mWeaponBodyIdentity = 0;
            std::uint32_t mTargetBodyIdentity = 0;
            std::uint64_t mPhysicsStep = 0;
            std::uint64_t mSwingIdentity = 0;
            bool mSwingActive = false;
            osg::Vec3f mPoint;
            osg::Vec3f mNormal;
            float mRelativeNormalSpeed = 0;
            float mRelativeSpeed = 0;
            float mEffectiveMass = 0;
            std::optional<float> mNormalImpulse;
        };

        std::string mRecordId;
        std::size_t mBodyIndex = 0;
        std::size_t mHandBodyIndex = 0;
        float mItemWeight = 0;
        float mItemSpeed = 0;
        float mPhysicalMass = 0;
        float mLength = 0;
        float mGripSeparation = 0;
        float mGripAngularError = 0;
        bool mTwoHanded = false;
        bool mSupportActive = false;
        std::size_t mSupportHandBodyIndex = 0;
        std::optional<osg::Vec3f> mSupportTarget;
        float mSupportPositionError = 0;
        float mSupportRotationError = 0;
        // A same-capture physical target is distinct from the retained pre-solve
        // motor command above. Absent until a real point grip has solved.
        struct SupportPhysicalObservation
        {
            osg::Vec3f mTarget;
            float mPositionError = 0;
            std::uint64_t mPhysicsStep = 0;
        };
        std::optional<SupportPhysicalObservation> mSupportPhysical;
        struct SupportGripState
        {
            std::string mState = "inactive", mReason = "not-sampled";
            bool mAttached = false, mConstraintActive = false, mSolvedObservation = false;
            bool mAuthoredGripReached = false;
            std::uint64_t mObservationPhysicsStep = 0, mLastImpulsePhysicsStep = 0;
            float mAnchorSeparation = 0, mRelativeAnchorSpeed = 0;
            float mStableSeconds = 0, mCaptureDistance = 0, mMaximumRelativeSpeed = 0;
            float mCaptureImpulse = 0, mAvailableImpulse = 0, mLinearImpulse = 0;
            float mAuthoredWeight = 1, mAuthoredDistance = 0, mReachLimit = 0, mAuthoredRemaining = 0;
            std::uint64_t mAcquireCount = 0, mReleaseCount = 0, mBreakCount = 0, mRefusalCount = 0;
            std::optional<osg::Vec3f> mFirstLocalAnchor, mSecondLocalAnchor;
            std::optional<osg::Vec3f> mFirstWorldAnchor, mSecondWorldAnchor;
            std::optional<osg::Vec3f> mPreStepFirstWorldAnchor, mPreStepSecondWorldAnchor;
            std::optional<osg::Vec3f> mPreStepFirstCenter, mPreStepSecondCenter;
            osg::Vec3f mImpulseOnSecond, mFirstAngularImpulse, mSecondAngularImpulse;
        };
        // External point constraint; absent from the ragdoll's tree-joint rows.
        std::optional<SupportGripState> mSupportGrip;
        std::uint32_t mVelocitySolverSteps = 0;
        std::uint64_t mContacts = 0;
        std::uint64_t mGroundContacts = 0;
        std::uint64_t mOtherRagdollContacts = 0;
        std::uint64_t mContactEventCount = 0;
        std::optional<ContactEvent> mLastContactEvent;
        std::uint64_t mStrikeEventCount = 0;
        std::optional<ContactEvent> mLastStrikeEvent;
        // Opt-in bounded completed-step diagnostics, never contact authority.
        struct ContactWindowObservation
        {
            std::uint64_t mPhysicsStep = 0, mAttackId = 0, mSwingIdentity = 0;
            std::uint64_t mActorSerial = 0, mLifecycleEpoch = 0;
            std::optional<std::uint32_t> mWeaponBodyIdentity;
            std::string mPhase;
            bool mNativeOwned = false, mOverflow = false;
            std::vector<ContactEvent> mContacts, mStrikes, mDispatches;
            std::vector<std::uint64_t> mPhysicalHitIds;
        };
        std::optional<std::vector<ContactWindowObservation>> mContactWindows;
        std::uint64_t mContactWindowsEvictedCount = 0;
        std::uint64_t mDamageEventCount = 0;
        float mLastImpactFraction = 0;
        float mLastAuthoredDamage = 0;
        float mLastFinalDamage = 0;
        bool mLastDamageBlocked = false;
        std::uint64_t mDefenseOutcomeCount = 0;
        std::string mLastDefenseOutcome = "none";
        std::string mLastContactDisposition = "invalid";
        float mLastApproachAlignment = 0;
        float mLastGuardFacingAlignment = -1;
        float mLastDefenseDamageScale = 0;
        float mLastDefenseImpulseScale = 0;
        float mLastRedirectScale = 0;
        float mLastRedirectImpulse = 0;
        float mLinearBreakImpulse = 0;
        float mAngularBreakImpulse = 0;
        float mLastLinearImpulse = 0;
        float mLastAngularImpulse = 0;
        bool mAttached = false;
        bool mConstraintEnabled = false;
        bool mCallbackArmed = false;
        bool mBodyRetained = false;
        std::string mReleaseReason;
        std::uint64_t mReleaseRequests = 0;
        std::uint64_t mReleaseCount = 0;
    };

    enum class PassiveRagdollWeaponReleaseReason
    {
        Break,
        Drop,
        Unequip,
        Death,
        Cleanup,
    };

    struct PassiveRagdollBalanceKneeState
    {
        std::optional<std::size_t> mBodyIndex;
        std::optional<float> mActualAngle;
        std::optional<float> mRequestedMotorAngle;
        std::optional<float> mFinalMotorAngle;
        std::optional<float> mMinimumAngle;
        std::optional<float> mMaximumAngle;
        std::optional<float> mRequestedToFinalDelta;
        std::string mMotorAvailability = "not-observed";
        // Evaluated animation includes blending/procedural callbacks. There
        // is no raw, isolated clip sample at this native binding boundary.
        std::string mClipAvailability = "raw-clip-unavailable-evaluated-animation-only";
        std::optional<float> mEvaluatedAnimationAngle;
        std::optional<std::uint64_t> mAnimationSampleSerial;
        std::optional<std::uint64_t> mAnimationSampleNextPhysicsStep;
        std::string mAnimationAvailability = "not-observed";
        std::optional<float> mComposedAngle;
        std::optional<std::uint64_t> mComposedPoseSerial;
        std::optional<std::uint64_t> mComposedPhysicsStep;
        std::string mComposedSource = "none";
        std::string mComposedAvailability = "no-pose-submitted-this-step";
        std::optional<std::uint64_t> mMotorObservationPhysicsStep;
        std::optional<int> mMotorState;
        std::optional<float> mFinalToActualDelta;
    };

    struct PassiveRagdollBalanceMassBody
    {
        std::size_t mBodyIndex = 0;
        JPH::BodyID mBodyId;
        float mMass = 0;
        bool mActive = false;
        bool mDeclaredKinematicRoot = false;
        JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
        JPH::Vec3 mVelocity = JPH::Vec3::sZero();
    };

    struct PassiveRagdollBalanceContactSource
    {
        JPH::BodyID mSelfBodyId;
        JPH::BodyID mSurfaceBodyId;
        std::uint32_t mSubShape1 = 0;
        std::uint32_t mSubShape2 = 0;
        unsigned mPointIndex = 0;
        bool mFirstBody = false;
        std::optional<bool> mSurfaceStatic;
        std::optional<JPH::RVec3> mSurfacePoint;
    };

    // Immutable completed-step measurement, copied by getters without reading
    // bodies or accumulating state. Missing values are not numerical zeros.
    // Observation of the actor queue, not an acknowledgment of a keyboard or
    // test-protocol command. Immutable after completed-step publication.
    struct PassiveRagdollBalanceIntentState
    {
        std::string mAvailability = "uncaptured";
        std::optional<std::uint64_t> mCaptureSerial;
        std::optional<std::uint64_t> mCaptureActorSerial;
        std::optional<std::uint64_t> mCaptureLifecycleEpoch;
        std::optional<unsigned> mCaptureRenderFrame;
        std::optional<std::uint64_t> mFirstPossiblePhysicsStep;
        std::optional<std::uint64_t> mConsumedPhysicsStep;
        std::optional<JPH::Vec3> mQueuedWorldVelocity;
        std::optional<JPH::Vec3> mProcessedRawHorizontalVelocity;
        std::optional<JPH::Vec3> mFilteredCommandVelocity;
        std::optional<bool> mLocomotionInterrupted;
    };

    struct PassiveRagdollBalanceState
    {
        std::uint64_t mActorSerial = 0;
        std::uint64_t mPhysicsStep = 0;
        std::uint64_t mLifecycleEpoch = 0;
        std::string mLastInvalidation = "created";
        PassiveRagdollBalanceIntentState mIntent;
        std::string mActorRecord;
        std::optional<unsigned> mReplaySeed;
        unsigned mTestStepMultiplier = 0;
        float mDuration = 0;
        int mUpdateError = 0;
        bool mPowered = false;
        bool mRootKinematic = false;
        bool mAuthorityEnabled = false;
        float mAuthority = 0;
        float mAuthorityApplied = 0;
        float mAuthorityDemand = 0;
        float mAuthorityBaseline = 0;
        float mAuthorityDisturbance = 0;
        float mAuthoritySupportLoss = 0;
        float mAuthorityUnsupportedTime = 0;
        float mAuthorityConsumedImpulse = 0;
        std::uint32_t mAuthorityConsumedReceiptCount = 0;
        std::uint64_t mAuthorityFirstConsumedReceipt = 0;
        std::uint64_t mAuthorityLastConsumedReceipt = 0;
        std::uint64_t mAuthorityMissingPhysicsSteps = 0;
        bool mAuthorityBaselineAvailable = false;
        std::string mAuthorityStatus = "disabled";
        std::string mAuthorityCorrection = "disabled";
        std::uint32_t mAuthorityCorrectedLegMask = 0;
        std::string mRoute;
        std::string mModeBeforeObservation = "unknown";
        JPH::Vec3 mRootReactionOffset = JPH::Vec3::sZero();
        JPH::Vec3 mRootReactionVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mGravity = JPH::Vec3::sZero();
        std::string mWeaponRecord;
        bool mWeaponAttached = false;
        std::vector<std::size_t> mMassBodyIndices;
        std::vector<PassiveRagdollBalanceMassBody> mMassBodies;
        std::optional<std::size_t> mExcludedDetachedWeapon;
        std::optional<RagdollCore::BalanceMassMeasurement> mMass;
        std::array<bool, 2> mFootActiveBeforeUpdate{};
        std::array<bool, 2> mFootGeometryComplete{};
        std::array<bool, 2> mFootContactsComplete{};
        std::array<std::optional<std::size_t>, 2> mFootBodyIndices;
        std::array<std::optional<JPH::BodyID>, 2> mFootBodyIds;
        std::array<std::string, 2> mFootAvailabilityReason;
        std::array<std::optional<double>, 2> mFootNormalImpulse;
        std::array<std::optional<double>, 2> mFootLoadFraction;
        // Captured native reference origin error; not actual slip and not the
        // exact final composed motor target after wound/servo overrides.
        std::array<std::optional<float>, 2> mReferenceFootError;
        // References are frozen before Update; actual origins come from the
        // same postsolve body snapshot as the mass/constraint observations.
        std::array<std::optional<JPH::RVec3>, 2> mComposedFootReference;
        std::array<std::optional<JPH::RVec3>, 2> mActualFootOrigin;
        std::array<std::optional<JPH::Vec3>, 2> mComposedFootErrorVector;
        std::array<std::optional<float>, 2> mComposedFootError;
        std::array<std::string, 2> mComposedFootAvailability{ "no-pose-submitted-this-step", "no-pose-submitted-this-step" };
        float mSupportNormalMinimumDot = 0;
        std::vector<RagdollCore::BalanceContactPoint> mContacts;
        std::vector<PassiveRagdollBalanceContactSource> mContactSources;
        RagdollCore::BalanceSupportMeasurement mCenterOfMassSupport;
        std::array<RagdollCore::BalanceFootSlipMeasurement, 2> mFootSlip;
        std::string mEpisodeConsumeStatus = "unconsumed";
        std::uint64_t mEpisodeMissingSteps = 0;
        std::uint64_t mEpisodeTotalMissingSteps = 0;
        std::array<RagdollCore::BalanceFootEpisodeState, 2> mFootEpisodes;
        RagdollCore::BalanceClosedSupportSegments mClosedSupportSegments;
        // A previous generation's partial segment, closed between captures.
        RagdollCore::BalanceClosedSupportSegments mInvalidatedSupportSegments;
        std::string mCapturePointApplicability = "support-unknown";
        std::optional<double> mSupportPlaneHeight;
        // Geometric numerical tolerances, not gameplay tuning or authority.
        float mPlaneNormalTolerance = 1e-5f;
        double mPlaneHeightTolerance = 1e-4;
        std::optional<JPH::RVec3> mCapturePoint;
        RagdollCore::BalanceSupportMeasurement mCapturePointSupport;
        std::array<PassiveRagdollBalanceKneeState, 2> mKnees;
    };
    // Read-only snapshot of the native RXN consumer. Reading diagnostics must
    // never advance the pipeline or infer a behavior from the damage ledger.
    struct PassiveRagdollReactionState
    {
        bool mEnabled = false;
        std::string mBehavior = "none";
        std::uint64_t mLastServedReceipt = 0;
        float mSpeedScale = 1.0f;
        float mStabilizationStrength = 1.0f;
        bool mCradleLeft = false;
        bool mCradleRight = false;
        bool mLimpLeft = false;
        bool mLimpRight = false;
        bool mLeftArmOwned = false;
        bool mRightArmOwned = false;
        bool mRecoveryBrace = false;
    };
    struct PassiveRagdollDebugState
    {
        PassiveRagdollReactionState mReaction;
        std::optional<PassiveRagdollBalanceState> mBalance;
        std::string mAttackPhase = "idle";
        std::uint64_t mAttackId = 0;
        float mAttackStrength = 0;
        std::string mAttackType;
        bool mStrikeRightHand = true;
        bool mThrustInput = false;
        std::uint64_t mPlayerPressCount = 0;
        // Wrists outside their joint limits (degrees, -1 unknown) and the
        // tip controller's command (radians/units, body yaw frame).
        float mWristOutsideRight = -1, mWristOutsideLeft = -1;
        bool mTipActive = false;
        // Hand control: on, the hand's and the blade's elevation (radians).
        bool mHscActive = false;
        float mHscEl = 0, mHscBladeEl = 0;
        // The chamber path: its parameter, and the arm solve's hand (units)
        // and wrist (degrees) distance from what it asked (-1 unknown).
        float mChamberS = 0, mChamberHandError = -1, mChamberWristError = -1;
        // What hand control asked of the weapon arm (world): the grip, the
        // elbow's bend direction and the blade direction.
        osg::Vec3f mHscGrip, mHscPole, mHscBlade;
        int mChamberLine = -1; // attack line (0 chop, 1 forehand, 2 backhand, 3 thrust), -1 guard
        // The stance circle and the cut (roadmap 2026-10-04): the stage (0 guard,
        // 1 wind-up, 2 cut, 3 return), the stance point (counts), the line's
        // angle (radians), the weapon drive's phase factor and the commanded
        // grip's distance from the stroke's cut plane (units).
        int mHscStage = 0;
        float mHscStanceX = 0, mHscStanceY = 0, mHscAngle = 0, mHscDrivePhase = 1, mHscPlaneMiss = 0;
        bool mHscSnap = false, mHscChain = false, mHscAuto = false;
        bool mHscRiposte = false; // HSC_RIPOSTE: the running click answers a parry won
        float mHscFlow = 0; // HSC_CHAIN_FLOW: the running chain's flow (0-1)
        float mHscSinceParryWon = 1.0e9f; // s since this body's last parry won
        int mHscGripMode = 0; // GRIP-05: 0 normal, 1 half-sword, 2 murder-stroke
        std::uint64_t mHscJabs = 0; // FIST-02: left jabs thrown
        float mHscJab = 0; // ... how far the jab is out (0-1)
        float mHscRoute = 0; // the route position shown (mFlowU: 0..1 wind-up, 1..2 cut)
        float mHscReseat = 0; // the re-seat, 1 open .. 0
        int mHscReseats = 0;
        // TRAJ-01: the committed attack's cut, the commanded tip (world);
        // empty out of one.
        std::vector<osg::Vec3f> mHscTrajectory;
        std::string mArmOwnerLeft, mArmOwnerRight; // who holds each arm (consolidation)
        std::string mOwnerLegs, mOwnerTrunk, mOwnerHead; // the body arbiter's other channels
        std::uint64_t mBlockStaggers = 0;
        // Head-blow daze left (s) and how deep the knees gave (units).
        float mDazeTime = 0, mDazeDrop = 0;
        std::uint64_t mDazeCount = 0;
        float mTipGap = 0, mTipAz = 0, mTipEl = 0, mTipRadius = 0, mTipBodyTwist = 0, mTipBodyLean = 0;
        // The tip controller's goal for the tip, world space (mouse feedback).
        osg::Vec3f mTipCommand;
        std::uint64_t mWoundReceiptCount = 0;
        std::uint64_t mWoundMarkCount = 0;
        std::string mLastWoundKind;
        std::string mLastWoundRegion;
        float mLastWoundHealth = 0;
        float mLastWoundFatigue = 0;
        float mLocalReactionAngle = 0;
        std::uint64_t mOtherRagdollContacts = 0;
        float mHorizontalBodyWidth = 0;
        std::uint64_t mPhysicsStep = 0;
        PassiveRagdollStepQuality mStepQuality;
        std::vector<PassiveRagdollBodyState> mBodies;
        std::vector<PassiveRagdollConstraintState> mConstraints;
        std::vector<PassiveRagdollContactState> mContacts;
        std::optional<PassiveRagdollSupportState> mSupport;
        std::optional<PassiveRagdollControllerState> mController;
        std::optional<PassiveRagdollWeaponState> mWeapon;
        osg::Vec3f mCenterOfMass;
        osg::Vec3f mReferencePosition;
        std::optional<osg::Vec3f> mPhysicsActorPosition;
        // The soft root (balance helper) this step: the outside wrench it put
        // on the body and its strength (hits lower it). Roadmap step 3 push
        // table: how much the helper carried versus the legs.
        struct SoftRootState
        {
            osg::Vec3f mForce;
            osg::Vec3f mTorque;
            float mPositionError = 0.0f;
            float mAngleError = 0.0f;
            float mStrength = 1.0f;
            bool mSaturated = false;
        };
        std::optional<SoftRootState> mSoftRoot;
        float mTotalMass = 0.0f;
        float mRenderInterpolationFactor = 0.0f;
        std::uint64_t mRenderPhysicsStep = 0;
        float mModeTransitionFactor = 0.0f;
        bool mPendingImpact = false;
        bool mInheritedAnimationMotion = false;
        bool mPowered = false;
        bool mGuardActive = false;
        std::string mGuardZone = "middle"; // GUARD_ZONES / RMB_GUARD
        // HUD-01: the way the weight tips (x right, y forward) and how near
        // the edge, 0..1 (PassiveRagdollEntry::hudBalance).
        float mBalanceRight = 0.0f, mBalanceForward = 0.0f, mBalanceTip = 0.0f;
        float mAttackReady = 1.0f; // HUD-01: 0 just after a cut, 1 ready (attackReadiness)
        std::uint64_t mGuardStartPhysicsStep = 0;
        std::uint64_t mCombatReactionCount = 0;
        std::optional<RagdollCore::CombatReactionRecord> mLastCombatReaction;
    };

    struct PassiveRagdollDebugCounters
    {
        std::size_t mCurrent = 0;
        std::size_t mPendingImpacts = 0;
        std::uint64_t mCreated = 0;
        std::uint64_t mRemoved = 0;
        std::uint64_t mEvicted = 0;
        std::uint64_t mPoseFailures = 0;
        std::uint64_t mMotionSamples = 0;
        std::uint64_t mMotionTransfers = 0;
        std::uint64_t mWeaponBodiesCreated = 0;
        std::uint64_t mWeaponBodiesDestroyed = 0;
        std::uint64_t mWeaponReleases = 0;
        std::uint64_t mWeaponCallbacksArmed = 0;
        std::uint64_t mWeaponCallbacksDisarmed = 0;
        std::uint64_t mWeaponDuplicateReleaseRequests = 0;
    };

    struct PassiveRagdollReplayResult
    {
        std::int64_t mFirstFallingFrame = -1;
        std::uint32_t mVersion = 0;
        std::uint64_t mSeed = 0;
        std::uint64_t mProfileHash = 0;
        float mFixedTimeStep = 0.0f;
        std::uint32_t mWorkerThreadCount = 0;
        std::uint32_t mCollisionSteps = 0;
        std::uint32_t mInputCount = 0;
        std::uint32_t mControllerStateCount = 0;
        std::uint32_t mBodyCount = 0;
        std::size_t mFrameCount = 0;
        std::vector<std::byte> mData;
    };

    using ActorMap = std::unordered_map<const MWWorld::LiveCellRefBase*, std::shared_ptr<Actor>>;

    struct ContactPoint
    {
        MWWorld::Ptr mObject;
        osg::Vec3f mPoint;
        osg::Vec3f mNormal;
    };

    struct LOSRequest
    {
        LOSRequest(const std::weak_ptr<Actor>& a1, const std::weak_ptr<Actor>& a2);
        std::array<std::weak_ptr<Actor>, 2> mActors;
        std::array<const Actor*, 2> mRawActors;
        bool mResult;
        bool mStale;
        int mAge;
    };
    bool operator==(const LOSRequest& lhs, const LOSRequest& rhs) noexcept;

    struct ActorFrameData
    {
        ActorFrameData(Actor& actor, bool inert, bool waterCollision, float slowFall, float waterlevel, bool isPlayer);
        osg::Vec3f mPosition;
        osg::Vec3f mInertia;
        bool mIsOnGround;
        bool mIsOnSlope;
        bool mWalkingOnWater;
        const bool mInert;
        JPH::BodyID mStandingOn;
        JPH::BodyID mPhysicsBody;
        const float mSwimLevel;
        const float mSlowFall;
        osg::Vec2f mRotation;
        osg::Vec3f mMovement;
        osg::Vec3f mLastStuckPosition;
        const float mWaterlevel;
        const float mHalfExtentsZ;
        float mOldHeight;
        unsigned int mStuckFrames;
        const bool mFlying;
        const bool mWasOnGround;
        const bool mIsAquatic;
        const bool mWaterCollision;
        const bool mSkipCollisionDetection;
        const bool mIsPlayer;
        JPH::ObjectLayer mCollisionMask;
    };

    struct WorldFrameData
    {
        WorldFrameData();
        bool mIsInStorm;
        osg::Vec3f mStormDirection;
    };

    template <class Ptr, class FrameData>
    class SimulationImpl
    {
    public:
        explicit SimulationImpl(const std::weak_ptr<Ptr>& ptr, FrameData&& data)
            : mPtr(ptr)
            , mData(data)
        {
        }

        std::optional<std::pair<std::shared_ptr<Ptr>, std::reference_wrapper<FrameData>>> lock()
        {
            if (auto locked = mPtr.lock())
                return { { std::move(locked), std::ref(mData) } };
            return std::nullopt;
        }

    private:
        std::weak_ptr<Ptr> mPtr;
        FrameData mData;
    };

    using ActorSimulation = SimulationImpl<Actor, ActorFrameData>;
    using Simulation = std::variant<ActorSimulation>;

    class PhysicsSystem : public RayCastingInterface
    {
    public:
        PhysicsSystem(Resource::ResourceSystem* resourceSystem, osg::ref_ptr<osg::Group> parentNode);
        virtual ~PhysicsSystem();

        Resource::PhysicsShapeManager* getShapeManager();

        void enableWater(float height);
        void setWaterHeight(float height);
        void disableWater();

        void addObject(const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh, osg::Quat rotation,
            int collisionType = Layers::WORLD);
        void addActor(const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh);

        // `force`: go limp now, past a physical death (OPENMW_RAGDOLL_DEATH_PHYSICAL).
        bool addPassiveRagdoll(const MWWorld::Ptr& ptr, SceneUtil::Skeleton& skeleton, bool force = false);
        // Physical deaths that have ended this frame: the world makes them limp.
        std::vector<MWWorld::Ptr> takeFinishedDeaths();
        bool addPoweredRagdoll(const MWWorld::Ptr& ptr, SceneUtil::Skeleton& skeleton);
        // Selects the actor's currently equipped right-hand weapon for the
        // next powered-ragdoll construction. The first qualified live route
        // uses the one-kilogram training-bar proxy from the offline fixture.
        // Selection must happen before the ragdoll is created.
        bool prepareEquippedRagdollWeapon(const MWWorld::Ptr& ptr);
        bool releasePassiveRagdollWeapon(
            const MWWorld::ConstPtr& ptr, PassiveRagdollWeaponReleaseReason reason);
        void samplePassiveRagdollMotion(const MWWorld::Ptr& ptr, SceneUtil::Skeleton& skeleton, float duration);
        // Seconds each actor's weapon has been drawn (the ready-stance rebuild).
        std::unordered_map<const void*, std::pair<std::string, float>> mWeaponDrawnFor;
        float weaponDrawnFor(const MWWorld::Ptr& ptr) const
        {
            const auto found = mWeaponDrawnFor.find(ptr.mRef);
            return found == mWeaponDrawnFor.end() ? 0.0f : found->second.second;
        }
        bool beginPhysicalHitReceipt(const MWWorld::Ptr& ptr, std::uint64_t id);
        void finishPhysicalHitReceipt(const MWWorld::Ptr& ptr, std::uint64_t id, float health, float fatigue);
        // RXN-02 M2: the struck body's reactions to one counted blow (returns the push).
        float applyBlowReaction(PassiveRagdollEntry& victim, const RagdollCore::Blow& blow);
        static bool guardBlowsEnabled();
        static bool clashV2Enabled();
        static bool crowdEnabled();
        static float clashArmFactor(const MWWorld::Ptr& actor, bool guard);
        static void payBlockFatigue(const MWWorld::Ptr& blocker, const MWWorld::Ptr& attacker, float attackStrength);
        float applyGuardBlow(PassiveRagdollEntry& side, const PassiveRagdollEntry& other, JPH::RVec3Arg point,
            JPH::Vec3Arg direction, float momentum, bool shield, bool beaten, bool blunt, std::uint64_t physicsStep);
#ifdef OPENMW_RAGDOLL_DEV_API
        std::uint64_t stageWoundReceipt(const MWWorld::Ptr& ptr, std::string_view body, int kind,
            const osg::Vec3f& point, const osg::Vec3f& normal);
#endif
        void recordKillingBlow(const MWWorld::Ptr& ptr, const osg::Vec3f& worldPoint,
            const osg::Vec3f& direction, float healthDamage);
        void removePassiveRagdoll(const MWWorld::Ptr& ptr);
        bool hasPassiveRagdoll(const MWWorld::ConstPtr& ptr) const;
        // World-space involuntary movement (magnitude 0..1) while a shove plays out.
        osg::Vec3f getPassiveRagdollStumble(const MWWorld::ConstPtr& ptr) const;
        void updatePassiveRagdolls();

        /// Powers humanoid NPCs near the player and releases them again when
        /// they leave the neighbourhood. The first cut powers every NPC that
        /// comes close (OPENMW_RAGDOLL_POWER_NPCS, dev builds); the population
        /// budget and hysteresis keep the cost bounded.
        void updatePoweredRagdollPopulation();
        // BOAT_START: character creation and the player still on the prison
        // ship (its hold, deck or plank); no body is powered until off it.
        bool playerOnChargenShip();
        bool mOnChargenShip = false; // the last grounded answer (kept while airborne)
        void updateSwimFallback();
        void updateStuckFallback();
        // Powered actors unpowered while swimming, and the time each has been
        // back on dry ground since (steady clock seconds, -1 while wet).
        std::map<const MWWorld::LiveCellRefBase*, double> mSwimSuspended;
        // NPC_STUCK: a powered NPC walking without getting anywhere (its
        // anchor and since when), and the ones on the vanilla capsule (where
        // they were let go, and since when they stand still; -1 walking).
        struct StuckWatch
        {
            osg::Vec3f mAnchor;
            double mSince = 0.0;
            osg::Vec3f mLast{}; // suspended: where it last moved 5 units from
            bool mSteep = false; // suspended by NPC_STEEP: powered again only standing still
        };
        std::map<const MWWorld::LiveCellRefBase*, StuckWatch> mStuckWatch;
        std::map<const MWWorld::LiveCellRefBase*, StuckWatch> mStuckSuspended;
        // BUILD_FRAMES: a standing pose per skeleton (by its proportions) the
        // legs' and spine's joint frames are built from; never erased (the
        // build options of live bodies point into it).
        std::map<std::string, RagdollCore::SkeletonDescriptor> mStandingFrameSkeletons;
        // LEGS_HEALTH: when each actor's body was last rebuilt for failing legs.
        std::map<const MWWorld::LiveCellRefBase*, double> mLegsHealthRebuilt;
        double mStrandedSince = -1.0; // the player's body apart from the player since (OPENMW_RAGDOLL_STRANDED)

        /// Dev harness (OPENMW_RAGDOLL_AUTO_DUEL): periodically pairs the two
        /// nearest living powered NPCs as each other's combat targets, so an
        /// unattended range keeps producing NPC duels.
        void updateAutoDuel();

        /// Powered AI melee: while a powered NPC's controller is attacking a
        /// combat target, drive its physical weapon swing at that target (its
        /// torso when the body is down), so the contact pipeline prices the
        /// blow instead of the animation hit path.
        void updatePoweredAiStrikes();
        void stepDeathDrag(float dt);
        void updateGuardThreats();
        void updateHscFoes();
        void strikeCapsuleActors(PassiveRagdollEntry& entry);
        static void loadRagdollSettingsFile();
        // NPCs raise their physical guard against an incoming swing (skill
        // and OPENMW_RAGDOLL_AI_GUARD scaled chance, once per enemy swing).
        void updateAiGuard(PassiveRagdollEntry& entry, const MWWorld::Ptr& actor);
        std::optional<PassiveRagdollBalanceState> getPassiveRagdollBalanceState(
            const MWWorld::ConstPtr& ptr) const;
        std::optional<PassiveRagdollDebugState> getPassiveRagdollDebugState(
            const MWWorld::ConstPtr& ptr) const;
        std::optional<osg::Vec3f> getPassiveRagdollCameraPosition(const MWWorld::ConstPtr& ptr) const;
        bool isPoweredRagdoll(const MWWorld::ConstPtr& ptr) const;
        // True only while the actor owns an attached live physical weapon.
        // Input uses this to claim the attack button before the ordinary
        // animation-driven melee path can see it.
        bool hasAttachedRagdollWeapon(const MWWorld::ConstPtr& ptr) const;
        // Game-thread query. Copies one fixed-step observation under the simulation lock.
        std::optional<RagdollCore::CombatState> getRagdollCombatState(const MWWorld::ConstPtr& ptr) const;
        // DUEL-AI-01 stage 1: what an opponent can see of this powered body.
        std::optional<RagdollCore::DuelBodyView> getRagdollDuelView(const MWWorld::ConstPtr& ptr) const;
        // AI-02 item 2: the AI holds this body's guard up for `seconds` (renewed while wanted).
        void requestRagdollAiGuard(const MWWorld::ConstPtr& ptr, float seconds);
        void requestRagdollAiRiposte(const MWWorld::ConstPtr& ptr);
        // Tests (RIPOSTE-02): as if this body's parry had just won a clash.
        void markRagdollParryWon(const MWWorld::ConstPtr& ptr);
        PassiveRagdollDebugCounters getPassiveRagdollDebugCounters() const;
        // SHOVE-01: the actor shoves the body in front of it within reach - its
        // hands go to that chest and the push lands through the blow reaction
        // when they get there. False: nobody in reach, on cooldown, or off.
        bool shovePassiveRagdoll(const MWWorld::ConstPtr& ptr);
        // GRIP-05: the actor's long blade to its next grip; false when none.
        bool cycleRagdollGrip(const MWWorld::ConstPtr& ptr);
        // The dodge (OPENMW_RAGDOLL_DODGE): a committed step or, running, a
        // dodge roll along `direction` (horizontal, world), or the actor's
        // movement keys when unset (else back). False when the actor has no
        // powered body.
        bool requestRagdollDodge(const MWWorld::ConstPtr& ptr, std::optional<osg::Vec3f> direction);
        // TRAIL-01: each held weapon's blade, its last 40% (base) to the tip,
        // world space, keyed by the actor.
        std::vector<std::pair<const void*, std::pair<osg::Vec3f, osg::Vec3f>>> getWeaponBlades();
        // GRIP-05: the actors whose drawn weapon is turned in the hand (the
        // murder-stroke), with the turn in the hand's frame, plus those whose
        // turn just ended (none).
        std::vector<std::pair<MWWorld::Ptr, std::optional<osg::Matrixf>>> getWeaponGripTurns();
        // REACT-02: a scripted blow through the blow reaction entry point.
        bool applyTestBlow(const MWWorld::ConstPtr& ptr, std::string_view bodyName, const osg::Vec3f& direction,
            float speed, float mass, float damage);
        bool applyPassiveRagdollImpulse(const MWWorld::ConstPtr& ptr, std::size_t bodyIndex,
            const osg::Vec3f& impulse, const std::optional<osg::Vec3f>& worldPoint = std::nullopt);
        // Melee contact: applies the impulse to the ragdoll body whose center
        // of mass is nearest the world point, at that point (lever arm
        // limited so a contact plane sample far from the body does not spin
        // it unrealistically). False when the actor has no ragdoll.
        bool applyPassiveRagdollImpulseAtPoint(
            const MWWorld::ConstPtr& ptr, const osg::Vec3f& impulse, const osg::Vec3f& worldPoint);
        // Identified melee reaction path. Duplicate and out-of-order IDs are
        // rejected before they can add another impulse or extend hit stun.
        bool applyCombatReactionAtPoint(const MWWorld::ConstPtr& ptr, std::uint64_t reactionId,
            const osg::Vec3f& impulse, const osg::Vec3f& worldPoint);
        std::uint64_t nextCombatReactionId();
        // Aimed melee: returns the point on the ray nearest the ragdoll body
        // the ray passes closest to, so a hit lands where the attacker is
        // actually looking instead of at a random point on the target's
        // bounding plane. Unset when the actor has no ragdoll or no body is
        // within maxDistance of the ray.
        std::optional<osg::Vec3f> pickPassiveRagdollAimPoint(const MWWorld::ConstPtr& ptr,
            const osg::Vec3f& rayFrom, const osg::Vec3f& rayDirection, float maxDistance) const;
        // Manual range grab: picks the ragdoll body nearest the ray (within
        // 60 units) and, until released, pulls its center of mass toward
        // the anchor with a spring-damper force capped at maxForce every
        // fixed step. The controller sees it as any other external force.
        bool grabPassiveRagdoll(const MWWorld::ConstPtr& ptr, const osg::Vec3f& rayFrom,
            const osg::Vec3f& rayDirection, float maxForce);
        bool updatePassiveRagdollGrab(const MWWorld::ConstPtr& ptr, const osg::Vec3f& anchor);
        void releasePassiveRagdollGrab(const MWWorld::ConstPtr& ptr);
        // Asks one hand to reach a world point, blending in over blendRate
        // per second up to weight in [0, 1]. Repeating the call moves the
        // target without restarting the blend, so a hand can follow something.
        // The point may be out of reach: the arm extends toward it and
        // getPassiveRagdollArmReach reports how far short it fell, which is
        // how a brace tells finding the floor from waving at it. False when
        // the actor has no ragdoll or no arm on that side.
        bool setPassiveRagdollArmTarget(const MWWorld::ConstPtr& ptr, bool rightArm,
            const osg::Vec3f& worldTarget, float weight, float blendRate);
        // Applies the offline WPN-01 shoulder-subtree yaw to a captured actor's
        // motor target. This is a deterministic developer fixture, not input.
        bool setPassiveRagdollWeaponFixtureYaw(const MWWorld::ConstPtr& ptr, float angle);
        // Fades the arm out over its blend rate and then drops the request.
        // Cutting an arm loose in one step would hand the motors a step change
        // the balance controller has to absorb.
        void releasePassiveRagdollArmTarget(const MWWorld::ConstPtr& ptr, bool rightArm);
        // Records explicit defensive intent on the fixed-step clock. Raising
        // it opens the parry window; keeping it raised transitions to block,
        // and an old guard no longer converts a physical hit.
        // The guard's zone (RMB_GUARD): mouse counts accumulated while the
        // guard is held (right, up); reset = the middle.
        bool drivePassiveRagdollGuardZone(const MWWorld::ConstPtr& ptr, float countsRight, float countsUp, bool reset);
        bool setPassiveRagdollGuard(const MWWorld::ConstPtr& ptr, bool active,
            const std::optional<osg::Vec3f>& worldFacing = std::nullopt);
        // Distance between the last requested point and the point the arm
        // could actually reach. Unset when that arm holds no request.
        std::optional<float> getPassiveRagdollArmReach(const MWWorld::ConstPtr& ptr, bool rightArm) const;
        // Whether hand control (the cut wheel) holds the actor's weapon
        // arm: a tap is then its own click attack, not Morrowind's clip.
        bool isPassiveRagdollHandControlActive(const MWWorld::ConstPtr& ptr) const;
        // Physical attack (plan item 2.3d): while held, aim motion
        // drives a point the weapon hand chases, and the swing is whatever
        // the arm motors produce on the way there. The impact is then an
        // ordinary contact at whatever speed the weapon actually reached,
        // rather than a damage number attached to an animation.
        //
        // aimRight/aimUp are normalized aim motion since the last call, and
        // the basis vectors are the frame that motion is drawn in - the
        // camera's, so the arc reads as the one the player drew rather than
        // one relative to a body being shoved mid-swing. Releasing does not
        // cut the swing; it lets the arc follow through.
        //
        // Returns the speed the commanded point actually moved at, or nothing
        // when the actor has no ragdoll or no arm on that side. That speed is
        // the honest reading of how hard a blow was wound up.
        std::optional<float> driveRagdollWeaponSwing(const MWWorld::ConstPtr& ptr, bool rightArm,
            const osg::Vec3f& aimRight, const osg::Vec3f& aimForward, const osg::Vec3f& aimUp, float deltaRight,
            float deltaUp, bool held, float duration, bool playerInput = false, bool thrust = false);
        // What the actor is looking at while it swings: the crosshair ray in
        // world space. The physical swing picks the body part of a living
        // actor nearest that ray, within striking range, and points the stroke
        // at it; without a ray (or with nothing near it) the stroke closes on
        // the body's own centre line. Queued like the swing input itself.
        void setRagdollSwingAimRay(const MWWorld::ConstPtr& ptr, const osg::Vec3f& origin, const osg::Vec3f& direction);
        void setRagdollAttackAnimation(const MWWorld::ConstPtr& ptr, const MWMechanics::RagdollAttackState& state);
        // Development aid for localized limb damage: the damage each
        // anatomical region of this actor's ragdoll has taken (region name,
        // points), which is what the red body-part tint shows, and a reset of
        // every actor's ledger and tint.
        std::vector<std::pair<std::string, float>> getRagdollLimbDamage(const MWWorld::ConstPtr& ptr) const;
        // Radians of view nod owed to the player camera since the last call (alpha 0.4
        // item 8, OPENMW_RAGDOLL_CAMERA_KICK degrees, 0 = off).
        // The player's camera kick since the last call: (nod, roll) in rad.
        std::pair<float, float> takePlayerCameraKick();
        void addRagdollLimbDamage(const MWWorld::ConstPtr& ptr, std::string_view region, float damage);
        void clearRagdollLimbDamage();
        // Speed of the last commanded swing point, for callers deciding how
        // hard a blow landed. Unset when nothing is swinging.
        std::optional<float> getRagdollWeaponSwingSpeed(const MWWorld::ConstPtr& ptr) const;
        // How much stronger the arm is while swinging than the profile
        // authored it. One means the authored motors, which are sized to hold
        // a pose and visibly cannot drive a weapon.
        void setRagdollSwingStrength(float scale);
        float getRagdollSwingStrength() const;
        // Force every get-up to play one animation, or pass nothing to go
        // back to the measured choice. Diagnostic only: a fallen body rests on
        // its side, where the reading that picks the animation sits on its own
        // decision boundary, so the two have to be watched side by side before
        // that choice is worth changing.
        void setRagdollRecoveryDirection(std::optional<bool> faceUp);
        // Whether a rising actor plants its hands (ActiveControllerProfile::
        // mBracingArms). Exposed so the brace can be switched off mid-session
        // and the get-up compared with and without it; judging that by eye
        // across separate runs is not a comparison.
        void setRagdollBracingArms(bool bracing);
        bool getRagdollBracingArms() const;
        // What each arm is doing, for traces and diagnostics: how far short
        // of its target the hand fell, the weight it is asserted at, and
        // whether the request belongs to the brace.
        struct RagdollArmDebug
        {
            osg::Vec3f mTargetWorld;
            float mMeasuredErrorWorld = -1.0f;
            float mTargetDistanceModel = -1.0f;
            float mMinimumReachModel = -1.0f;
            float mMaximumReachModel = -1.0f;
            float mReachWithoutShoulderModel = -1.0f;
            float mReachError = 0.0f;
            float mWeight = 0.0f;
            bool mApplied = false;
            bool mBrace = false;
        };
        std::optional<RagdollArmDebug> getPassiveRagdollArmDebug(const MWWorld::ConstPtr& ptr, bool rightArm) const;
        // Signed radians from the heading gameplay holds to the one the
        // physical body actually lies at. The recovery restores position but
        // not yaw, so this is how far the get-up animation is turned away
        // from the body it is supposed to be lifting.
        std::optional<float> getPassiveRagdollHeadingError(const MWWorld::ConstPtr& ptr) const;
        // Whether a fallen actor asks to get up by itself
        // (ActiveControllerProfile::mAutomaticRecovery). Diagnostics that
        // observe a fallen body for a fixed window turn it off.
        void setAutomaticRagdollRecovery(bool automatic);
        bool getAutomaticRagdollRecovery() const;
        bool setPassiveRagdollActive(const MWWorld::ConstPtr& ptr, bool active);
        bool startPassiveRagdollReplay(
            const MWWorld::ConstPtr& ptr, std::uint64_t seed, std::optional<std::size_t> maximumFrames = std::nullopt, bool concurrent = false);
        std::optional<PassiveRagdollReplayResult> finishPassiveRagdollReplay(const MWWorld::ConstPtr* ptr = nullptr);
#ifdef OPENMW_RAGDOLL_DEV_API
        RagdollProfile::AnimationCycleSample sampleRagdollAnimationCycle(const MWWorld::ConstPtr& ptr,
            const SceneUtil::KeyframeHolder& source, std::string_view group, std::string_view accumulationBone,
            const osg::Vec3f& accumulationAxes, std::size_t sampleCount) const;
        bool samplePoweredRecoveryTarget(const MWWorld::ConstPtr& ptr,
            const SceneUtil::KeyframeHolder& source, float time, std::string_view accumulationBone,
            const osg::Vec3f& accumulationAxes);
        bool requestPoweredRagdollRecovery(const MWWorld::ConstPtr& ptr);
        std::optional<PoweredRagdollRecoveryState> getPoweredRagdollRecoveryState(
            const MWWorld::ConstPtr& ptr) const;
        bool setPoweredRagdollRecoveryAnimationStarted(const MWWorld::ConstPtr& ptr);
        bool setPoweredRagdollRecoveryAnimationFinished(const MWWorld::ConstPtr& ptr);
        // Where and which way the actor should be when gameplay takes it
        // back. Position alone is not enough: Morrowind's get-up animations
        // turn the character as it rises, and the ragdoll follows them, so by
        // the time it is standing the physical body faces about 180 degrees
        // away from the yaw gameplay still holds. Measured across a recovery,
        // the error grows from 5 degrees while fallen to 177 by hand-off.
        // Restoring one without the other is what made a rising actor swap
        // head for feet.
        // Complete recovery without destroying the body; return its gameplay yaw.
        std::optional<float> finishPoweredRagdollRecovery(const MWWorld::Ptr& ptr);
        void failPoweredRagdollControl(const MWWorld::ConstPtr& ptr);
#endif

        int addProjectile(
            const MWWorld::Ptr& caster, const osg::Vec3f& position, VFS::Path::NormalizedView mesh, bool computeRadius);
        void setCaster(int projectileId, const MWWorld::Ptr& caster);
        void removeProjectile(const int projectileId);

        void updatePtr(const MWWorld::Ptr& old, const MWWorld::Ptr& updated);

        Actor* getActor(const MWWorld::Ptr& ptr);
        const Actor* getActor(const MWWorld::ConstPtr& ptr) const;

        const Object* getObject(const MWWorld::ConstPtr& ptr) const;

        Projectile* getProjectile(int projectileId) const;

        // Object or Actor
        void remove(const MWWorld::Ptr& ptr);

        void updateScale(const MWWorld::Ptr& ptr);
        void updateRotation(const MWWorld::Ptr& ptr, osg::Quat rotate);
        void updatePosition(const MWWorld::Ptr& ptr);

        void addHeightField(const float* heights, int x, int y, int size, int verts, float minH, float maxH,
            const osg::Object* holdObject);

        void removeHeightField(int x, int y);

        const HeightField* getHeightField(int x, int y) const;

        bool toggleCollisionMode();

        /// Determine new position based on all queued movements, then clear the list.
        void stepSimulation(
            float dt, bool skipSimulation, osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats);

        /// Apply new positions to actors
        void moveActors();
        void debugDraw();

        std::vector<MWWorld::Ptr> getCollisions(const MWWorld::ConstPtr& ptr, int collisionGroup,
            int collisionMask) const; ///< get handles this object collides with
        std::vector<ContactPoint> getCollisionsPoints(
            const MWWorld::ConstPtr& ptr, int collisionGroup, int collisionMask) const;
        osg::Vec3f traceDown(const MWWorld::Ptr& ptr, const osg::Vec3f& position, float maxHeight);
        void optimize();

        /// @param ignore Optional, a list of Ptr to ignore in the list of results. targets are actors to filter for,
        /// ignoring all other actors.
        RayCastingResult castRay(const osg::Vec3f& from, const osg::Vec3f& to,
            const std::vector<MWWorld::ConstPtr>& ignore = {}, const std::vector<MWWorld::Ptr>& targets = {},
            int mask = CollisionMask_Default, int group = 0xff) const override;
        using RayCastingInterface::castRay;

        RayCastingResult castSphere(const osg::Vec3f& from, const osg::Vec3f& to, float radius,
            int mask = CollisionMask_Default, int group = 0xff) const override;

        /// Return true if actor1 can see actor2.
        bool getLineOfSight(const MWWorld::ConstPtr& actor1, const MWWorld::ConstPtr& actor2) const override;

        bool isOnGround(const MWWorld::Ptr& actor);

        bool canMoveToWaterSurface(const MWWorld::ConstPtr& actor, const float waterlevel);

        /// Get physical half extents (scaled) of the given actor.
        osg::Vec3f getHalfExtents(const MWWorld::ConstPtr& actor) const;

        /// Get physical half extents (not scaled) of the given actor.
        osg::Vec3f getOriginalHalfExtents(const MWWorld::ConstPtr& actor) const;

        /// @see MWPhysics::Actor::getRenderingHalfExtents
        osg::Vec3f getRenderingHalfExtents(const MWWorld::ConstPtr& actor) const;

        /// Get the position of the collision shape for the actor. Use together with getHalfExtents() to get the
        /// collision bounds in world space.
        /// @note The collision shape's origin is in its center, so the position returned can be described as center of
        /// the actor collision box in world space.
        osg::Vec3f getCollisionObjectPosition(const MWWorld::ConstPtr& actor) const;

        /// Get bounding box in world space of the given object.
        osg::BoundingBox getBoundingBox(const MWWorld::ConstPtr& object) const;

        /// Queues velocity movement for a Ptr. If a Ptr is already queued, its velocity will
        /// be overwritten. Valid until the next call to stepSimulation
        void queueObjectMovement(const MWWorld::Ptr& ptr, const osg::Vec3f& velocity);

        /// Force a collision-shape animation update for a single object.
        void updateAnimatedCollisionShape(const MWWorld::Ptr& ptr);

        /// Clear the queued movements list without applying.
        void clearQueuedMovement();

        /// Return true if \a actor has been standing on \a object in this frame
        /// This will trigger whenever the object is directly below the actor.
        /// It doesn't matter if the actor is stationary or moving.
        bool isActorStandingOn(const MWWorld::Ptr& actor, const MWWorld::ConstPtr& object) const;

        /// Get the handle of all actors standing on \a object in this frame.
        void getActorsStandingOn(const MWWorld::ConstPtr& object, std::vector<MWWorld::Ptr>& out) const;

        /// Return true if an object of the given type has collided with this object
        bool isObjectCollidingWith(const MWWorld::ConstPtr& object, ScriptedCollisionType type) const;

        /// Get the handle of all actors colliding with \a object in this frame.
        void getActorsCollidingWith(const MWWorld::ConstPtr& object, std::vector<MWWorld::Ptr>& out) const;

        bool toggleDebugRendering();

        void reportCollision(const osg::Vec3f& position, const osg::Vec3f& normal);

        /// Mark the given object as a 'non-solid' object. A non-solid object means that
        /// \a isOnSolidGround will return false for actors standing on that object.
        void markAsNonSolid(const MWWorld::ConstPtr& ptr);

        bool isOnSolidGround(const MWWorld::Ptr& actor) const;

        template <class Function>
        void forEachAnimatedObject(Function&& function) const
        {
            std::for_each(mAnimatedObjects.begin(), mAnimatedObjects.end(), function);
        }

        bool isAreaOccupiedByOtherActor(const osg::Vec3f& position, const float radius,
            std::span<const MWWorld::ConstPtr> ignore, std::vector<MWWorld::Ptr>* occupyingActors) const;

        float getPhysicsDt() const { return mPhysicsDt; }

        void reportStats(unsigned int frameNumber, osg::Stats& stats) const;

        inline const JPH::BodyLockInterfaceLocking& getBodyLockInterface() const;

        const JPH::BodyInterface& getBodyInterface() const;

        // NET-01 duel (mwworld/kineticnet.hpp). The remote player's puppet: a
        // fighter's body (weapon included) whose parts are kinematic and
        // replay the remote poses ~100 ms behind, so contacts against it and
        // its guard still happen here. It runs no controller and strikes
        // nothing; blows that land on it go to the remote game.
        bool setNetPuppet(const MWWorld::Ptr& ptr);
        void clearNetPuppet();
        bool isNetPuppet(const MWWorld::ConstPtr& ptr) const;
        MWWorld::Ptr getNetPuppet() const;
        // While on, the player's body is captured every step for the link.
        void setNetActive(bool active);
        void receiveNetPoses(std::vector<MWWorld::NetPose> poses);
        void receiveNetBlows(std::vector<MWWorld::NetBlow> blows);
        std::vector<MWWorld::NetPose> takeNetPlayerPoses();
        std::vector<MWWorld::NetBlow> takeNetBlows();

    private:
        void updateWater();
        void updatePtrHolders();

        void prepareSimulation(bool willSimulate, std::vector<Simulation>& simulations);
        bool addRagdoll(const MWWorld::Ptr& ptr, SceneUtil::Skeleton& skeleton, bool powered);
        const RagdollCore::RagdollProfile* getPassiveRagdollProfile();
        bool releaseWeapon(PassiveRagdollEntry& entry, PassiveRagdollWeaponReleaseReason reason);
        // The world point the player's physical swing should be aimed at this
        // step, from its crosshair ray and the bodies within striking range.
        void updatePlayerStrikeAim(PassiveRagdollEntry& entry);
        void erasePassiveRagdoll(const MWWorld::LiveCellRefBase* ref,
            PassiveRagdollWeaponReleaseReason reason = PassiveRagdollWeaponReleaseReason::Cleanup);
        void enforcePassiveRagdollCap();
        void recordPassiveRagdollReplayFrame();
        bool isNetPuppetEntry(const PassiveRagdollEntry& entry) const;
        void stepNetPuppet();
        void captureNetPlayerPose();
        void queueNetBlow(const RagdollCore::Blow& blow);

        const MWWorld::LiveCellRefBase* mNetPuppetRef = nullptr;
        std::uint64_t mNetPuppetSerial = 0;
        bool mNetActive = false;
        std::deque<MWWorld::NetPose> mNetPuppetPoses;
        double mNetPlayback = -1;
        bool mNetBodyCountWarned = false;
        std::vector<MWWorld::NetBlow> mNetBlowsIn;
        std::vector<MWWorld::NetPose> mNetPlayerPoses;
        std::vector<MWWorld::NetBlow> mNetBlowsOut;

        JoltContactListener mContactListener;
        JoltBPLayerInterface mBPLayerInterface;
        JoltObjectVsBroadPhaseLayerFilter mObjectVsBPLayerFilter;
        JoltObjectLayerPairFilter mObjectVsObjectLayerFilter;

        JPH::PhysicsSystem mPhysicsSystem;
        std::unique_ptr<PhysicsTaskScheduler> mTaskScheduler;
        std::unique_ptr<JPH::TempAllocatorImpl> mMemoryAllocator;
        std::unique_ptr<JPH::JobSystem> mPhysicsJobSystem;
        std::unique_ptr<Resource::PhysicsShapeManager> mShapeManager;
        Resource::ResourceSystem* mResourceSystem;

        using ObjectMap = std::unordered_map<const MWWorld::LiveCellRefBase*, std::shared_ptr<Object>>;
        ObjectMap mObjects;

        std::map<Object*, bool> mAnimatedObjects; // stores pointers to elements in mObjects

        ActorMap mActors;

        using PassiveRagdollMap
            = std::unordered_map<const MWWorld::LiveCellRefBase*, std::unique_ptr<PassiveRagdollEntry>>;
        PassiveRagdollMap mPassiveRagdolls;
        // SHOVE-01: pushes waiting for the hands to arrive.
        struct PendingShove
        {
            const MWWorld::LiveCellRefBase* mShover = nullptr;
            const MWWorld::LiveCellRefBase* mTarget = nullptr;
            float mLeft = 0.0f;
            float mStrength = 1.0f;
            int mExtraClicks = 0; // rapid right clicks after the double click
        };
        std::vector<PendingShove> mPendingShoves;
        void stepPendingShoves(float dt);
        using PassiveRagdollMotionMap
            = std::unordered_map<const MWWorld::LiveCellRefBase*, std::unique_ptr<PassiveRagdollMotionHistory>>;
        PassiveRagdollMotionMap mPassiveRagdollMotionHistory;
        std::unordered_map<const MWWorld::LiveCellRefBase*, PendingRagdollImpact> mPendingRagdollImpacts;
        struct PreparedRagdollWeapon
        {
            std::string mRecordId;
            float mItemWeight = 0;
            // Existing ESM attack-speed multiplier, also used by the finite
            // physical hand target and its recovery.
            float mSpeed = 1.0f;
            // ESM record reach in meters: the physical weapon box is built
            // from it so a dagger's grip and blade are proportioned correctly.
            float mReach = 0.8f;
        };
        std::unordered_map<const MWWorld::LiveCellRefBase*, PreparedRagdollWeapon> mPreparedRagdollWeapons;
        std::unique_ptr<RagdollCore::RagdollProfile> mPassiveRagdollProfile;
        // Tuning for the mouse-driven swing. One profile for every actor, so
        // an NPC swinging under AI control and a player swinging by hand are
        // the same mechanism with different input.
        RagdollCore::WeaponSwingProfile mRagdollWeaponSwing;
        std::unordered_map<const MWWorld::LiveCellRefBase*, std::unique_ptr<PassiveRagdollReplayRecording>> mPassiveRagdollReplays;
        bool mPassiveRagdollProfileLoadAttempted = false;
        std::uint64_t mNextPassiveRagdollSerial = 1;
        std::uint64_t mNextCombatReactionSerial = 1;
        std::uint32_t mNextPassiveRagdollGroup = 1;
        std::uint64_t mPassiveRagdollsCreated = 0;
        std::uint64_t mPassiveRagdollsRemoved = 0;
        std::uint64_t mPassiveRagdollsEvicted = 0;
        std::uint64_t mPassiveRagdollPoseFailures = 0;
        std::uint64_t mPassiveRagdollMotionSamples = 0;
        std::uint64_t mPassiveRagdollMotionTransfers = 0;
        std::uint64_t mWeaponBodiesCreated = 0;
        std::uint64_t mWeaponBodiesDestroyed = 0;
        std::uint64_t mWeaponReleases = 0;
        std::uint64_t mWeaponCallbacksArmed = 0;
        std::uint64_t mWeaponCallbacksDisarmed = 0;
        std::uint64_t mWeaponDuplicateReleaseRequests = 0;

        using ProjectileMap = std::map<int, std::shared_ptr<Projectile>>;
        ProjectileMap mProjectiles;

        using HeightFieldMap = std::map<std::pair<int, int>, std::unique_ptr<HeightField>>;
        HeightFieldMap mHeightFields;

        bool mDebugDrawEnabled;

        float mTimeAccum;

        int mProjectileId;

        float mTimeAccumJolt = 0.0f;
        std::uint64_t mJoltStepSerial = 0;
        float mWaterHeight;
        bool mWaterEnabled;

        std::unique_ptr<MWWater> mWaterInstance;

        std::unique_ptr<MWRender::JoltDebugDrawer> mJoltDebugDrawer;

        osg::ref_ptr<osg::Group> mParentNode;

        float mPhysicsDt;

        std::size_t mSimulationsCounter = 0;
        std::array<std::vector<Simulation>, 2> mSimulations;
        std::vector<std::pair<MWWorld::Ptr, osg::Vec3f>> mActorsPositions;

        PhysicsSystem(const PhysicsSystem&);
        PhysicsSystem& operator=(const PhysicsSystem&);
    };
}

#endif
