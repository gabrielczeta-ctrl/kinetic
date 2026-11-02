#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_REACTIONCONTRACT_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_REACTIONCONTRACT_H

// RXN-01 milestone 0: the versioned input/output contract between the native
// contact/damage adapter (Codex) and the offline reaction walker (DeepSeek).
//
// The contract is a value snapshot: no MWWorld, Lua, rendering or game-data
// types. Both the offline tester and the eventual native adapter feed the same
// structures and consume the same bounded requests. Inputs are immutable and
// consumed in one documented fixed-step order; outputs never award damage,
// move the root, guarantee a knock-down, or re-apply an already applied impact.

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <ragdollcore/anatomy.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace RagdollCore
{
    inline constexpr std::uint32_t ReactionContractVersion = 1;

    // Where an impulse number came from. Absent is distinct from zero.
    enum class ImpulseProvenance : std::uint8_t
    {
        None,             // no impulse information at all
        ContactEstimate,  // effectiveMass * relativeSpeed, a momentum estimate
        SolverMeasured,   // measured solver impulse, authoritative
        AppliedExternal,  // applied by the adapter; already integrated
    };

    struct ReactionImpulse
    {
        ImpulseProvenance mProvenance = ImpulseProvenance::None;
        float mMagnitude = 0.0f; // model units, >= 0
        std::optional<JPH::Vec3> mDirection;
        // True when the solver or the adapter has already applied this impulse
        // to the bodies. The reaction layer must never apply it again.
        bool mAlreadyApplied = false;
    };

    // Post-RPG applied damage. Missing means unknown, not a successful hit.
    struct ReactionReceipt
    {
        std::uint64_t mReceiptId = 0;
        float mHealthLoss = 0.0f;  // >= 0
        float mFatigueLoss = 0.0f; // >= 0
    };

    struct ReactionContact
    {
        std::uint64_t mContactId = 0;
        std::uint64_t mAttackerActorIdentity = 0;
        std::uint64_t mTargetActorIdentity = 0;
        std::uint64_t mTargetRegistryGeneration = 0;
        std::uint64_t mSwingIdentity = 0;
        std::uint64_t mPhysicsStep = 0;
        std::uint32_t mTargetBodyIndex = std::numeric_limits<std::uint32_t>::max();
        AnatomicalRegion mRegion = AnatomicalRegion::Unknown;
        bool mValid = false;
    };

    // What the body is currently doing and which limbs are already claimed.
    enum class ReactionBodyMode : std::uint8_t
    {
        Idle,
        Standing,
        Walking,
        Recovery,
        Passive,
    };

    struct ReactionLimbOwnership
    {
        bool mLeftArm = false;
        bool mRightArm = false;
        bool mLeftLeg = false;
        bool mRightLeg = false;
        // Recovery/get-up support has priority over cosmetic and guard
        // requests; a braced limb must never be stolen.
        bool mRecoveryBrace = false;
        bool mWeaponAim = false;
        bool mTwoHandedGrip = false;
    };

    struct ReactionSupportState
    {
        bool mSupportEstablished = true;
        bool mBothFeetSupported = false;
        bool mLeftFootSupported = false;
        bool mRightFootSupported = false;
        // Solved normal load fractions in [0, 1+]; unset when unknown.
        std::optional<float> mLeftFootLoad;
        std::optional<float> mRightFootLoad;
        // Horizontal speed of each measured foot; unset when unknown.
        std::optional<float> mLeftFootSlip;
        std::optional<float> mRightFootSlip;
        // Horizontal distance of the capture point beyond the support; unset
        // when the momentum or the foot measurement is unavailable.
        std::optional<float> mCaptureMargin;
    };

    struct ReactionContractInput
    {
        // Identity and ordering.
        std::uint64_t mFramePhysicsStep = 0;
        float mFixedStepDuration = 0.0f;

        ReactionContact mContact;
        ReactionImpulse mImpulse;
        std::optional<ReactionReceipt> mReceipt;
        // Applied-damage severity computed by the adapter with the existing
        // wound policy. This contract never re-derives a severity table.
        std::optional<float> mWoundSeverity;

        ReactionBodyMode mMode = ReactionBodyMode::Idle;
        bool mWalking = false;
        bool mMoving = false;
        // False when the struck limb has no remaining capability.
        bool mRegionCapable = true;

        ReactionLimbOwnership mOwned;
        ReactionSupportState mSupport;

        // Animation sample availability for the motor composition probe.
        bool mAnimationSampleValid = false;
    };

    // Which behavior policy owns the reaction this frame.
    enum class BehaviorId : std::uint8_t
    {
        None,
        FaceGuard,
        ArmCradle,
        Limp,
        DoubledOver,
        StumbleFlail,
    };

    const char* behaviorName(BehaviorId id);

    enum class ReactionPhase : std::uint8_t
    {
        Idle,
        Impact,
        Stagger,
        Catch,
        Fall,
        GetUp,
        Resuming,
    };

    struct ReactionLimbMask
    {
        bool mLeftArm = false;
        bool mRightArm = false;
        bool mLeftLeg = false;
        bool mRightLeg = false;
        bool any() const { return mLeftArm || mRightArm || mLeftLeg || mRightLeg; }
    };

    struct ReactionOutputs
    {
        BehaviorId mBehavior = BehaviorId::None;
        // Bounded local pose requests, radians. The composer clamps them.
        float mTorsoFlexion = 0.0f;
        float mHeadRecoil = 0.0f;
        float mLeftArmWithdrawal = 0.0f;
        float mRightArmWithdrawal = 0.0f;
        bool mCoverFace = false;
        bool mCradleLeft = false;
        bool mCradleRight = false;
        float mLimpBias = 0.0f; // [0, 1]
        bool mLimpLeft = false;
        bool mLimpRight = false;
        float mSpeedScale = 1.0f; // [0, 1]

        // Two distinct authorities. Never conflate them.
        // 1. Pose-correction authority: permission to depart from the sampled
        //    clip. Zero leaves the clip target unchanged.
        float mPoseCorrectionAuthority = 0.0f;
        // 2. Stabilization strength: the allowed fraction of the existing
        //    controller's finite motor/support budget. One keeps the budget.
        float mStabilizationStrength = 1.0f;
        JPH::Vec3 mCounterbalanceDirection = JPH::Vec3::sZero();
        float mCounterbalanceStrength = 0.0f;

        ReactionPhase mPhase = ReactionPhase::Idle;
        float mReactionTime = 0.0f;
        float mReactionIntensity = 0.0f;
        ReactionLimbMask mDisabledLimbs;

        // Bookkeeping for traces: which contact produced this output.
        std::uint64_t mServedContactId = 0;
        std::uint64_t mServedReceiptId = 0;
        bool mDuplicateIgnored = false;
        bool mStaleIgnored = false;
    };

    inline bool impulseIsFinite(const ReactionImpulse& impulse)
    {
        switch (impulse.mProvenance)
        {
            case ImpulseProvenance::None:
                return true;
            case ImpulseProvenance::ContactEstimate:
            case ImpulseProvenance::SolverMeasured:
            case ImpulseProvenance::AppliedExternal:
                break;
        }
        return std::isfinite(impulse.mMagnitude) && impulse.mMagnitude >= 0.0f
            && (!impulse.mDirection || (std::isfinite(impulse.mDirection->GetX())
                && std::isfinite(impulse.mDirection->GetY()) && std::isfinite(impulse.mDirection->GetZ())
                && impulse.mDirection->LengthSq() > 1e-8f));
    }

    inline bool supportIsFinite(const ReactionSupportState& support)
    {
        const auto finiteOrMissing = [](const std::optional<float>& value) {
            return !value || std::isfinite(*value);
        };
        return finiteOrMissing(support.mLeftFootLoad) && finiteOrMissing(support.mRightFootLoad)
            && finiteOrMissing(support.mLeftFootSlip) && finiteOrMissing(support.mRightFootSlip)
            && finiteOrMissing(support.mCaptureMargin);
    }

    // Rejects non-finite values, negative durations and invalid contacts. A
    // version or identity mismatch must be resolved by the caller.
    inline bool validContractInput(const ReactionContractInput& input)
    {
        if (!std::isfinite(input.mFixedStepDuration) || input.mFixedStepDuration < 0.0f)
            return false;
        if (!impulseIsFinite(input.mImpulse) || !supportIsFinite(input.mSupport))
            return false;
        if (input.mReceipt
            && (!std::isfinite(input.mReceipt->mHealthLoss)
                || !std::isfinite(input.mReceipt->mFatigueLoss)
                || input.mReceipt->mHealthLoss < 0.0f || input.mReceipt->mFatigueLoss < 0.0f))
            return false;
        if (input.mWoundSeverity
            && (!std::isfinite(*input.mWoundSeverity) || *input.mWoundSeverity < 0.0f
                || *input.mWoundSeverity > 1.0f))
            return false;
        if (input.mContact.mValid && input.mContact.mRegion == AnatomicalRegion::Unknown
            && input.mContact.mTargetBodyIndex == std::numeric_limits<std::uint32_t>::max())
            return false;
        return true;
    }
}

#endif
