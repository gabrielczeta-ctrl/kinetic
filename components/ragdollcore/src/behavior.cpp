#include <ragdollcore/behavior.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        float smoothRamp(float value, float engage, float full)
        {
            if (!std::isfinite(value) || !std::isfinite(engage) || !std::isfinite(full) || value <= engage)
                return 0.0f;
            if (full <= engage || value >= full)
                return 1.0f;
            const float x = (value - engage) / (full - engage);
            return x * x * (3.0f - 2.0f * x);
        }

        float finiteOr(float value, float fallback)
        {
            return std::isfinite(value) ? value : fallback;
        }
    }

    const char* behaviorName(BehaviorId id)
    {
        switch (id)
        {
            case BehaviorId::FaceGuard: return "face_guard";
            case BehaviorId::ArmCradle: return "arm_cradle";
            case BehaviorId::Limp: return "limp";
            case BehaviorId::DoubledOver: return "doubled_over";
            case BehaviorId::StumbleFlail: return "stumble_flail";
            default: return "none";
        }
    }

    bool regionIsTorso(AnatomicalRegion region)
    {
        return region == AnatomicalRegion::Chest || region == AnatomicalRegion::Abdomen;
    }

    bool regionIsLeft(AnatomicalRegion region)
    {
        switch (region)
        {
            case AnatomicalRegion::LeftClavicle:
            case AnatomicalRegion::LeftUpperArm:
            case AnatomicalRegion::LeftForearm:
            case AnatomicalRegion::LeftHand:
            case AnatomicalRegion::LeftThigh:
            case AnatomicalRegion::LeftCalf:
            case AnatomicalRegion::LeftFoot:
                return true;
            default:
                return false;
        }
    }

    bool regionIsArm(AnatomicalRegion region)
    {
        switch (region)
        {
            case AnatomicalRegion::LeftClavicle:
            case AnatomicalRegion::RightClavicle:
            case AnatomicalRegion::LeftUpperArm:
            case AnatomicalRegion::RightUpperArm:
            case AnatomicalRegion::LeftForearm:
            case AnatomicalRegion::RightForearm:
            case AnatomicalRegion::LeftHand:
            case AnatomicalRegion::RightHand:
                return true;
            default:
                return false;
        }
    }

    bool regionIsLeg(AnatomicalRegion region)
    {
        switch (region)
        {
            case AnatomicalRegion::LeftThigh:
            case AnatomicalRegion::RightThigh:
            case AnatomicalRegion::LeftCalf:
            case AnatomicalRegion::RightCalf:
            case AnatomicalRegion::LeftFoot:
            case AnatomicalRegion::RightFoot:
                return true;
            default:
                return false;
        }
    }

    BehaviorOutput stepBehavior(BehaviorState& state, const BehaviorInput& input,
        const BehaviorProfile& profile)
    {
        BehaviorOutput output;
        const float duration = std::isfinite(input.mDuration) && input.mDuration > 0.0f
            ? input.mDuration : 0.0f;
        if (!profile.mEnabled || input.mOwned.mRecoveryBrace)
        {
            state.reset();
            return output;
        }
        const float impulse = std::isfinite(input.mImpulse) && input.mImpulse > 0.0f ? input.mImpulse : 0.0f;
        const float severity = std::isfinite(input.mWoundSeverity)
            ? std::clamp(input.mWoundSeverity, 0.0f, 1.0f) : 0.0f;
        const float impulseScale = !profile.mEnabled ? 0.0f : 1.0f;

        // Transient selection. A new qualifying hit refreshes or overrides an
        // active behavior by intensity; priority resolves simultaneous hits.
        struct Candidate
        {
            BehaviorId mId = BehaviorId::None;
            float mIntensity = 0.0f;
            float mDuration = 0.0f;
        };
        Candidate best;
        const auto consider = [&](BehaviorId id, float intensity, float behaviorDuration) {
            if (!profile.mEnabled || intensity <= 0.0f || !std::isfinite(intensity))
                return;
            const float clamped = std::clamp(intensity, 0.0f, 1.0f);
            // Priority order: DoubledOver > StumbleFlail > FaceGuard.
            if (clamped > best.mIntensity + 1e-4f || best.mId == BehaviorId::None)
            {
                best = Candidate{ id, clamped, behaviorDuration };
            }
        };
        if (regionIsHead(input.mRegion))
            consider(BehaviorId::FaceGuard,
                impulseScale * smoothRamp(impulse, profile.mFaceGuardImpulse, profile.mFaceGuardFullImpulse),
                profile.mFaceGuardDuration);
        if (regionIsTorso(input.mRegion))
            consider(BehaviorId::DoubledOver,
                impulseScale * smoothRamp(impulse, profile.mDoubledOverImpulse,
                    profile.mDoubledOverFullImpulse),
                profile.mDoubledOverDuration);
        if ((input.mWalking || input.mMoving) && !regionIsHead(input.mRegion))
            consider(BehaviorId::StumbleFlail,
                impulseScale * smoothRamp(impulse, profile.mStumbleImpulse, profile.mStumbleFullImpulse),
                profile.mStumbleDuration);

        // Recovery bracing has priority over cosmetic and guard requests.
        const bool braced = input.mOwned.mRecoveryBrace;
        if (braced)
            best = Candidate{};

        state.mRemaining = std::max(0.0f, state.mRemaining - duration);
        if (best.mId != BehaviorId::None)
        {
            const bool expired = state.mRemaining <= 0.0f || state.mActive == BehaviorId::None;
            if (expired || best.mIntensity >= state.mIntensity)
            {
                state.mActive = best.mId;
                state.mRemaining = best.mDuration;
                state.mIntensity = best.mIntensity;
            }
        }
        if (state.mRemaining <= 0.0f)
        {
            state.mActive = BehaviorId::None;
            state.mIntensity = 0.0f;
        }

        output.mBehavior = state.mActive;
        output.mBehaviorTime = state.mRemaining;
        output.mBehaviorIntensity = state.mIntensity;
        const float intensity = state.mIntensity;

        // Pose requests for the active transient.
        switch (state.mActive)
        {
            case BehaviorId::FaceGuard:
                output.mCoverFace = true;
                output.mTorsoFlexion = 0.5f * profile.mMaximumTorsoFlexion * intensity;
                output.mHeadRecoil = profile.mMaximumHeadRecoil * intensity;
                output.mLeftArmWithdrawal = profile.mMaximumArmWithdrawal * intensity;
                output.mRightArmWithdrawal = profile.mMaximumArmWithdrawal * intensity;
                output.mAuthorityDropRequest = 0.4f * intensity;
                break;
            case BehaviorId::DoubledOver:
                output.mTorsoFlexion = profile.mMaximumTorsoFlexion * intensity;
                output.mHeadRecoil = 0.5f * profile.mMaximumHeadRecoil * intensity;
                output.mLeftArmWithdrawal = 0.5f * profile.mMaximumArmWithdrawal * intensity;
                output.mRightArmWithdrawal = 0.5f * profile.mMaximumArmWithdrawal * intensity;
                output.mAuthorityDropRequest = 0.8f * intensity;
                break;
            case BehaviorId::StumbleFlail:
                output.mTorsoFlexion = 0.4f * profile.mMaximumTorsoFlexion * intensity;
                output.mLeftArmWithdrawal = profile.mMaximumArmWithdrawal * intensity;
                output.mRightArmWithdrawal = profile.mMaximumArmWithdrawal * intensity;
                output.mAuthorityDropRequest = intensity;
                break;
            default:
                break;
        }

        // Persistent limb behaviors. They need a receipt-driven severity (or a
        // lost capability), never a fabricated one.
        const bool canDriveLeft = !input.mOwned.mLeftArm && !input.mOwned.mRecoveryBrace;
        const bool canDriveRight = !input.mOwned.mRightArm && !input.mOwned.mRecoveryBrace;
        const bool leftSide = regionIsLeft(input.mRegion);
        const bool armRegion = regionIsArm(input.mRegion);
        const bool legRegion = regionIsLeg(input.mRegion);
        const bool lostCapability = !input.mRegionCapable;
        const float limbSeverity = input.mHasReceipt ? severity : 0.0f;

        if (armRegion && (limbSeverity >= profile.mCradleSeverity
                || (lostCapability && input.mHasReceipt)))
        {
            output.mCradleLeft = leftSide && canDriveLeft;
            output.mCradleRight = !leftSide && canDriveRight;
        }
        if (legRegion && (limbSeverity >= profile.mLimpSeverity
                || (lostCapability && input.mHasReceipt)))
        {
            output.mLimpLeft = leftSide;
            output.mLimpRight = !leftSide;
            const float base = limbSeverity >= profile.mLimpSeverity ? limbSeverity : 1.0f;
            output.mLimpBias = std::clamp(base, 0.0f, 1.0f);
            output.mSpeedScale = std::clamp(
                1.0f - (1.0f - profile.mLimpSpeedScale) * output.mLimpBias, 0.0f, 1.0f);
        }

        // Ownership and capability masks. A weapon aim or two-handed grip owns
        // the arm; a braced limb is owned by recovery.
        output.mDisabledLimbs.mLeftArm = input.mOwned.mLeftArm || output.mCradleLeft
            || (armRegion && leftSide && lostCapability);
        output.mDisabledLimbs.mRightArm = input.mOwned.mRightArm || output.mCradleRight
            || (armRegion && !leftSide && lostCapability);
        output.mDisabledLimbs.mLeftLeg = input.mOwned.mLeftLeg
            || (legRegion && leftSide && lostCapability);
        output.mDisabledLimbs.mRightLeg = input.mOwned.mRightLeg
            || (legRegion && !leftSide && lostCapability);

        // Stabilization-strength request: heavy behaviors may ask the walker to
        // release part of its finite budget, but never below the floor.
        const float drop = std::clamp(output.mAuthorityDropRequest, 0.0f, 1.0f);
        output.mStabilizationStrength = std::clamp(
            1.0f - 0.5f * drop, profile.mMinimumStabilizationStrength, 1.0f);

        // Clamp every pose request to the profile bounds.
        output.mTorsoFlexion = std::clamp(finiteOr(output.mTorsoFlexion, 0.0f),
            0.0f, profile.mMaximumTorsoFlexion);
        output.mHeadRecoil = std::clamp(finiteOr(output.mHeadRecoil, 0.0f),
            0.0f, profile.mMaximumHeadRecoil);
        output.mLeftArmWithdrawal = std::clamp(finiteOr(output.mLeftArmWithdrawal, 0.0f),
            0.0f, profile.mMaximumArmWithdrawal);
        output.mRightArmWithdrawal = std::clamp(finiteOr(output.mRightArmWithdrawal, 0.0f),
            0.0f, profile.mMaximumArmWithdrawal);
        output.mSpeedScale = std::clamp(finiteOr(output.mSpeedScale, 1.0f), 0.0f, 1.0f);
        output.mLimpBias = std::clamp(finiteOr(output.mLimpBias, 0.0f), 0.0f, 1.0f);
        return output;
    }
}
