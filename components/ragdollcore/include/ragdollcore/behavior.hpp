#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_BEHAVIOR_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_BEHAVIOR_H

// RXN-01 behavior policy: turns a resolved contact and injury state into
// bounded, presentational requests. It never awards damage, never moves the
// root and never guarantees a knock-down; it asks, the walker decides.

#include <ragdollcore/anatomy.hpp>
#include <ragdollcore/reactioncontract.hpp>

#include <cstdint>

namespace RagdollCore
{
    struct BehaviorProfile
    {
        bool mEnabled = false;

        // Impulse ramps for the transient reactions, model units.
        float mFaceGuardImpulse = 400.0f;
        float mFaceGuardFullImpulse = 1400.0f;
        float mDoubledOverImpulse = 900.0f;
        float mDoubledOverFullImpulse = 2200.0f;
        float mStumbleImpulse = 1250.0f;
        float mStumbleFullImpulse = 2500.0f;

        // Wound severity above which a limb behavior persists. Severity comes
        // from the existing applied-damage/wound policy, never a new table.
        float mCradleSeverity = 0.15f;
        float mLimpSeverity = 0.15f;

        // Durations, seconds.
        float mFaceGuardDuration = 0.9f;
        float mDoubledOverDuration = 0.7f;
        float mStumbleDuration = 0.5f;

        // Bounded pose requests, radians / fractions.
        float mMaximumTorsoFlexion = 0.35f;
        float mMaximumHeadRecoil = 0.25f;
        float mMaximumArmWithdrawal = 0.6f;
        float mLimpSpeedScale = 0.65f;
        float mLimpStanceBias = 0.25f;

        // Policy strength: how much the reaction may ask the walker to give up
        // (1 = full request authority). The walker owns the mechanics.
        float mMaximumAuthorityDrop = 0.65f;
        float mMinimumAuthorityScale = 0.35f;
        // Stabilization-strength floor while a heavy behavior is active.
        float mMinimumStabilizationStrength = 0.4f;
    };

    struct BehaviorInput
    {
        AnatomicalRegion mRegion = AnatomicalRegion::Unknown;
        float mImpulse = 0.0f; // already validated, >= 0
        // Applied-damage severity in [0, 1]; 0 when no receipt exists.
        float mWoundSeverity = 0.0f;
        bool mHasReceipt = false;
        // False when the limb has no remaining capability.
        bool mRegionCapable = true;
        bool mWalking = false;
        bool mMoving = false;
        ReactionLimbOwnership mOwned;
        float mDuration = 0.0f;
    };

    struct BehaviorOutput
    {
        BehaviorId mBehavior = BehaviorId::None;
        float mTorsoFlexion = 0.0f;
        float mHeadRecoil = 0.0f;
        float mLeftArmWithdrawal = 0.0f;
        float mRightArmWithdrawal = 0.0f;
        bool mCoverFace = false;
        bool mCradleLeft = false;
        bool mCradleRight = false;
        float mLimpBias = 0.0f;
        bool mLimpLeft = false;
        bool mLimpRight = false;
        float mSpeedScale = 1.0f;
        // 0 = no request; 1 = the strongest request to give up balance.
        float mAuthorityDropRequest = 0.0f;
        // Fraction of the controller's finite budget to keep, [floor, 1].
        float mStabilizationStrength = 1.0f;
        float mBehaviorTime = 0.0f;
        float mBehaviorIntensity = 0.0f;
        ReactionLimbMask mDisabledLimbs;
    };

    struct BehaviorState
    {
        BehaviorId mActive = BehaviorId::None;
        float mRemaining = 0.0f;
        float mIntensity = 0.0f;
        void reset() { *this = BehaviorState{}; }
    };

    bool regionIsTorso(AnatomicalRegion region);
    bool regionIsLeft(AnatomicalRegion region);
    bool regionIsArm(AnatomicalRegion region);
    bool regionIsLeg(AnatomicalRegion region);

    // Advances the behavior policy one fixed step. Deterministic, allocation
    // free and bounded: every request is clamped by the profile. Transient
    // behaviors (FaceGuard, DoubledOver, StumbleFlail) are timed and chosen by
    // priority; ArmCradle and Limp persist while severity or capability says
    // so and combine with the transient pose. Recovery bracing suppresses
    // cosmetic requests; weapon-owned arms are never stolen.
    BehaviorOutput stepBehavior(BehaviorState& state, const BehaviorInput& input,
        const BehaviorProfile& profile);
}

#endif
