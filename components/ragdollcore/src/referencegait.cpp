#include <ragdollcore/referencegait.hpp>
#include <ragdollcore/tuning.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <string>

namespace RagdollCore
{
    std::uint64_t hashReferenceGaitProfile(const ReferenceGaitProfile& profile)
    {
        std::uint64_t hash = 14695981039346656037ull;
        const auto scalar = [&](std::uint32_t value) {
            for (unsigned shift = 0; shift < 32; shift += 8)
            {
                hash ^= (value >> shift) & 0xffu;
                hash *= 1099511628211ull;
            }
        };
        scalar(3); // canonical controller profile schema
        scalar(profile.mBrakeLateralTransfer ? 1u : 0u);
        scalar(profile.mRequireInitialSupportTransfer ? 1u : 0u);
        scalar(profile.mCoordinateInitialTransfer ? 1u : 0u);
        scalar(profile.mCoordinateWeightTransfers ? 1u : 0u);
        scalar(std::bit_cast<std::uint32_t>(profile.mDoubleSupportDuration));
        scalar(std::bit_cast<std::uint32_t>(profile.mSwingDuration));
        scalar(std::bit_cast<std::uint32_t>(profile.mMinimumSwingDuration));
        scalar(std::bit_cast<std::uint32_t>(profile.mContactConfirmationDuration));
        scalar(std::bit_cast<std::uint32_t>(profile.mTransferLoadFraction));
        scalar(std::bit_cast<std::uint32_t>(profile.mSwingStartLoadFraction));
        scalar(std::bit_cast<std::uint32_t>(profile.mMaximumDoubleSupportDuration));
        scalar(std::bit_cast<std::uint32_t>(profile.mMaximumStepDuration));
        scalar(std::bit_cast<std::uint32_t>(profile.mSwingFootLift));
        scalar(std::bit_cast<std::uint32_t>(profile.mTouchdownPressSpeed));
        scalar(std::bit_cast<std::uint32_t>(profile.mMaximumTouchdownPressDepth));
        scalar(std::bit_cast<std::uint32_t>(profile.mLateralCaptureOffset));
        scalar(std::bit_cast<std::uint32_t>(profile.mForwardVelocityErrorGain));
        scalar(std::bit_cast<std::uint32_t>(profile.mMinimumStepWidth));
        scalar(std::bit_cast<std::uint32_t>(profile.mMaximumStepWidth));
        scalar(std::bit_cast<std::uint32_t>(profile.mMinimumForwardPlacement));
        scalar(std::bit_cast<std::uint32_t>(profile.mMaximumStepReach));
        scalar(std::bit_cast<std::uint32_t>(profile.mVelocityGain));
        scalar(std::bit_cast<std::uint32_t>(profile.mWeightTransferFrequency));
        scalar(std::bit_cast<std::uint32_t>(profile.mWeightTransferDamping));
        scalar(std::bit_cast<std::uint32_t>(profile.mTransferVelocityGain));
        scalar(std::bit_cast<std::uint32_t>(profile.mMaximumRequestedAcceleration));
        scalar(profile.mLimitVelocityLead ? 1u : 0u);
        scalar(std::bit_cast<std::uint32_t>(profile.mUnrestrictedRequestSpeed));
        scalar(std::bit_cast<std::uint32_t>(profile.mVelocityLeadAccelerationFraction));
        scalar(std::bit_cast<std::uint32_t>(profile.mSwingLegFrequency));
        scalar(std::bit_cast<std::uint32_t>(profile.mSwingLegDamping));
        scalar(std::bit_cast<std::uint32_t>(profile.mStanceLegFrequency));
        scalar(std::bit_cast<std::uint32_t>(profile.mStanceLegDamping));
        scalar(std::bit_cast<std::uint32_t>(profile.mStanceKneeFlexion));
        scalar(std::bit_cast<std::uint32_t>(profile.mStanceKneeEaseDuration));
        scalar(std::bit_cast<std::uint32_t>(profile.mPlacementLatchFraction));
        scalar(std::bit_cast<std::uint32_t>(profile.mTrunkFrequency));
        scalar(std::bit_cast<std::uint32_t>(profile.mTrunkDamping));
        scalar(std::bit_cast<std::uint32_t>(profile.mLoadedAnkleFrequency));
        scalar(std::bit_cast<std::uint32_t>(profile.mLoadedAnkleDamping));
        scalar(std::bit_cast<std::uint32_t>(profile.mStanceAnklePushOff));
        scalar(std::bit_cast<std::uint32_t>(profile.mStanceAnklePushOffStartFraction));
        scalar(profile.mStanceAnklePushOffTrailing ? 1u : 0u);
        scalar(std::bit_cast<std::uint32_t>(profile.mTerminalStancePush));
        scalar(std::bit_cast<std::uint32_t>(profile.mTerminalStancePushStartFraction));
        scalar(std::bit_cast<std::uint32_t>(profile.mTerminalStancePushLateralLimit));
        scalar(std::bit_cast<std::uint32_t>(profile.mMaximumLateralRequestFraction));
        scalar(std::bit_cast<std::uint32_t>(profile.mSwingReachFraction));
        scalar(std::bit_cast<std::uint32_t>(profile.mSupportHullScale));
        scalar(std::bit_cast<std::uint32_t>(profile.mLostLandingSwapDelay));
        scalar(std::bit_cast<std::uint32_t>(profile.mLandingPressDepth));
        scalar(std::bit_cast<std::uint32_t>(profile.mSwingRiseEndFraction));
        scalar(std::bit_cast<std::uint32_t>(profile.mSwingTravelEndFraction));
        scalar(std::bit_cast<std::uint32_t>(profile.mSwingDescentEndFraction));
        scalar(std::bit_cast<std::uint32_t>(profile.mTouchdownPressStartFraction));
        scalar(profile.mPredictPlacement ? 1u : 0u);
        scalar(profile.mSwingVelocityFeedForward ? 1u : 0u);
        scalar(std::bit_cast<std::uint32_t>(profile.mStanceHipFrequency));
        scalar(std::bit_cast<std::uint32_t>(profile.mStanceHipDamping));
        scalar(profile.mStanceHipRateHold ? 1u : 0u);
        scalar(std::bit_cast<std::uint32_t>(profile.mSwingTravelStartFraction));
        scalar(std::bit_cast<std::uint32_t>(profile.mSwingInertiaFeedForward));
        scalar(std::bit_cast<std::uint32_t>(profile.mSwingFeedForwardAccelerationLimit));
        scalar(std::bit_cast<std::uint32_t>(profile.mEmergencyCaptureMargin));
        scalar(std::bit_cast<std::uint32_t>(profile.mEmergencyStanceLoadFraction));
        scalar(std::bit_cast<std::uint32_t>(profile.mSteeringTurnRate));
        scalar(std::bit_cast<std::uint32_t>(profile.mSteeringPelvisTwist));
        scalar(profile.mInitialStanceByTravel ? 1u : 0u);
        scalar(std::bit_cast<std::uint32_t>(profile.mSteeringSpeedChangeRate));
        scalar(std::bit_cast<std::uint32_t>(profile.mSteeringStopDeceleration));
        scalar(std::bit_cast<std::uint32_t>(profile.mSteeringStopSpeed));
        scalar(std::bit_cast<std::uint32_t>(profile.mSteeringTurnSlowdown));
        scalar(std::bit_cast<std::uint32_t>(profile.mStoppingVelocityErrorGain));
        scalar(std::bit_cast<std::uint32_t>(profile.mSteeringStopTimeout));
        scalar(std::bit_cast<std::uint32_t>(profile.mStandingHandoverDuration));
        scalar(std::bit_cast<std::uint32_t>(profile.mHitStunMinimumImpulse));
        scalar(std::bit_cast<std::uint32_t>(profile.mHitStunDuration));
        scalar(std::bit_cast<std::uint32_t>(profile.mHitStunReferenceImpulse));
        scalar(std::bit_cast<std::uint32_t>(profile.mFinalCommandFeasibilityStanceMargin));
        scalar(std::bit_cast<std::uint32_t>(profile.mFinalCommandFeasibilitySwingMargin));
        // Preserve legacy/default identities; enabled behavior gets a tagged extension.
        if (profile.mContinuousSwingTrajectory)
            scalar(0x43535731u); // CSW1
        return hash;
    }


    std::optional<JPH::Vec3> referenceGaitRootRecoveryRate(
        JPH::Vec3Arg worldOrientationError, float stiffness, float damping)
    {
        if (!std::isfinite(stiffness) || stiffness < 0 || !std::isfinite(damping) || damping <= 0
            || !std::isfinite(worldOrientationError.GetX()) || !std::isfinite(worldOrientationError.GetY())
            || !std::isfinite(worldOrientationError.GetZ())) return std::nullopt;
        const double gain = double(stiffness) / damping;
        if (gain > std::numeric_limits<float>::max()) return std::nullopt;
        const auto rate = worldOrientationError * float(gain);
        if (!std::isfinite(rate.GetX()) || !std::isfinite(rate.GetY()) || !std::isfinite(rate.GetZ()))
            return std::nullopt;
        return rate;
    }

    std::optional<RagdollPose> referenceGaitRelativeMotorPose(const ReferenceGaitPlan& plan,
        std::span<const Transform> measured, std::span<const int> parents, JPH::QuatArg rootRotation)
    {
        auto target = plan.mTargetPose;
        const auto& original = plan.mTargetPose.mBodyTransforms;
        if (original.empty() || measured.size() != original.size() || parents.size() != original.size()
            || parents[0] != -1 || !rootRotation.IsNormalized())
            return std::nullopt;
        const auto validRotation = [](JPH::QuatArg q) {
            return std::isfinite(q.GetX()) && std::isfinite(q.GetY()) && std::isfinite(q.GetZ())
                && std::isfinite(q.GetW()) && q.IsNormalized();
        };
        if (!validRotation(rootRotation)) return std::nullopt;
        for (std::size_t i = 0; i < original.size(); ++i)
            if (!validRotation(original[i].mRotation) || !validRotation(measured[i].mRotation)
                || (i && (parents[i] < 0 || static_cast<std::size_t>(parents[i]) >= i)))
                return std::nullopt;
        if (plan.mSwingHipWorldTarget && (!plan.mSwingHip || *plan.mSwingHip == 0
                || *plan.mSwingHip >= original.size() || !validRotation(*plan.mSwingHipWorldTarget)))
            return std::nullopt;
        for (const auto& [body, rotation] : plan.mWorldFootTargets)
            if (body == 0 || body >= original.size() || !validRotation(rotation)) return std::nullopt;
        for (std::size_t i = 1; i < original.size(); ++i)
        {
            const auto parent = static_cast<std::size_t>(parents[i]);
            std::optional<JPH::Quat> worldTarget;
            if (plan.mSwingHip && *plan.mSwingHip == i) worldTarget = plan.mSwingHipWorldTarget;
            for (const auto& [foot, rotation] : plan.mWorldFootTargets)
                if (foot == i) worldTarget = rotation;
            const JPH::Quat relative = worldTarget
                ? measured[parent].mRotation.Conjugated() * rootRotation.Conjugated() * *worldTarget
                : original[parent].mRotation.Conjugated() * original[i].mRotation;
            target.mBodyTransforms[i].mRotation = (target.mBodyTransforms[parent].mRotation * relative).Normalized();
        }
        return target;
    }

    namespace
    {
        bool isFinite(float value)
        {
            return std::isfinite(value);
        }

        bool isFinite(JPH::Vec3Arg value)
        {
            return isFinite(value.GetX()) && isFinite(value.GetY()) && isFinite(value.GetZ());
        }

        bool isFinite(JPH::RVec3Arg value)
        {
            return std::isfinite(static_cast<double>(value.GetX())) && std::isfinite(static_cast<double>(value.GetY()))
                && std::isfinite(static_cast<double>(value.GetZ()));
        }

        bool isFinite(JPH::QuatArg value)
        {
            return isFinite(value.GetX()) && isFinite(value.GetY()) && isFinite(value.GetZ()) && isFinite(value.GetW());
        }

        bool validProfile(const ReferenceGaitProfile& profile)
        {
            return isFinite(profile.mDoubleSupportDuration) && profile.mDoubleSupportDuration >= 0.0f
                && isFinite(profile.mSwingDuration) && profile.mSwingDuration > 0.0f
                && isFinite(profile.mMinimumSwingDuration) && profile.mMinimumSwingDuration >= 0.0f
                && profile.mMinimumSwingDuration <= profile.mSwingDuration
                && isFinite(profile.mContactConfirmationDuration) && profile.mContactConfirmationDuration >= 0.0f
                && isFinite(profile.mMaximumStepDuration) && profile.mMaximumStepDuration >= profile.mSwingDuration
                && isFinite(profile.mSwingFootLift) && profile.mSwingFootLift >= 0.0f
                && isFinite(profile.mTouchdownPressSpeed) && profile.mTouchdownPressSpeed >= 0.0f
                && isFinite(profile.mMaximumTouchdownPressDepth) && profile.mMaximumTouchdownPressDepth >= 0.0f
                && isFinite(profile.mLateralCaptureOffset) && profile.mLateralCaptureOffset >= 0.0f
                && isFinite(profile.mMinimumStepWidth) && profile.mMinimumStepWidth >= 0.0f
                && isFinite(profile.mMaximumStepWidth) && profile.mMaximumStepWidth >= profile.mMinimumStepWidth
                && isFinite(profile.mMinimumForwardPlacement) && isFinite(profile.mMaximumStepReach)
                && profile.mMaximumStepReach > 0.0f && isFinite(profile.mVelocityGain)
                && profile.mVelocityGain >= 0.0f && isFinite(profile.mWeightTransferFrequency)
                && profile.mWeightTransferFrequency >= 0.0f && isFinite(profile.mWeightTransferDamping)
                && profile.mWeightTransferDamping >= 0.0f && isFinite(profile.mMaximumRequestedAcceleration)
                && profile.mMaximumRequestedAcceleration >= 0.0f && isFinite(profile.mUnrestrictedRequestSpeed)
                && profile.mUnrestrictedRequestSpeed >= 0.0f
                && isFinite(profile.mVelocityLeadAccelerationFraction) && profile.mVelocityLeadAccelerationFraction > 0.0f
                && profile.mVelocityLeadAccelerationFraction <= 1.0f && isFinite(profile.mSwingLegFrequency)
                && profile.mSwingLegFrequency >= 0.0f && isFinite(profile.mSwingLegDamping)
                && profile.mSwingLegDamping >= 0.0f && isFinite(profile.mStanceLegFrequency)
                && profile.mStanceLegFrequency >= 0.0f && isFinite(profile.mStanceLegDamping)
                && profile.mStanceLegDamping >= 0.0f && isFinite(profile.mStanceKneeFlexion)
                && profile.mStanceKneeFlexion >= 0.0f && isFinite(profile.mStanceKneeEaseDuration)
                && profile.mStanceKneeEaseDuration >= 0.0f && isFinite(profile.mPlacementLatchFraction)
                && profile.mPlacementLatchFraction >= 0.0f && profile.mPlacementLatchFraction <= 1.0f
                && isFinite(profile.mTrunkFrequency) && profile.mTrunkFrequency >= 0.0f
                && isFinite(profile.mTrunkDamping) && profile.mTrunkDamping >= 0.0f
                && isFinite(profile.mLoadedAnkleFrequency) && profile.mLoadedAnkleFrequency >= 0.0f
                && isFinite(profile.mLoadedAnkleDamping) && profile.mLoadedAnkleDamping >= 0.0f
                && isFinite(profile.mStanceAnklePushOff) && profile.mStanceAnklePushOff >= 0.0f
                && isFinite(profile.mStanceAnklePushOffStartFraction)
                && profile.mStanceAnklePushOffStartFraction >= 0.0f
                && profile.mStanceAnklePushOffStartFraction < 1.0f
                && isFinite(profile.mTerminalStancePush) && profile.mTerminalStancePush >= 0.0f
                && isFinite(profile.mTerminalStancePushStartFraction)
                && profile.mTerminalStancePushStartFraction >= 0.0f
                && profile.mTerminalStancePushStartFraction < 1.0f
                && isFinite(profile.mTerminalStancePushLateralLimit)
                && isFinite(profile.mMaximumLateralRequestFraction)
                && profile.mMaximumLateralRequestFraction > 0.0f
                && profile.mMaximumLateralRequestFraction <= 1.0f
                && isFinite(profile.mTransferVelocityGain) && profile.mTransferVelocityGain >= 0.0f
                && isFinite(profile.mSwingReachFraction) && profile.mSwingReachFraction >= 0.0f
                && profile.mSwingReachFraction <= 1.0f && isFinite(profile.mTransferLoadFraction)
                && profile.mTransferLoadFraction >= 0.0f && isFinite(profile.mSwingStartLoadFraction)
                && profile.mSwingStartLoadFraction >= 0.0f && isFinite(profile.mMaximumDoubleSupportDuration)
                && profile.mMaximumDoubleSupportDuration >= profile.mDoubleSupportDuration
                && isFinite(profile.mForwardVelocityErrorGain) && profile.mForwardVelocityErrorGain >= 0.0f
                && isFinite(profile.mSupportHullScale) && profile.mSupportHullScale > 0.0f
                && profile.mSupportHullScale <= 1.0f && isFinite(profile.mLostLandingSwapDelay)
                && profile.mLostLandingSwapDelay >= 0.0f && isFinite(profile.mLandingPressDepth)
                && profile.mLandingPressDepth >= 0.0f && isFinite(profile.mSwingRiseEndFraction)
                && profile.mSwingRiseEndFraction > 0.0f && isFinite(profile.mSwingTravelEndFraction)
                && profile.mSwingTravelEndFraction > profile.mSwingRiseEndFraction
                && profile.mSwingTravelEndFraction <= 1.0f && isFinite(profile.mSwingDescentEndFraction)
                && profile.mSwingDescentEndFraction > profile.mSwingRiseEndFraction
                && profile.mSwingDescentEndFraction <= 1.0f && isFinite(profile.mTouchdownPressStartFraction)
                && profile.mTouchdownPressStartFraction >= 0.0f && isFinite(profile.mStanceHipFrequency)
                && profile.mStanceHipFrequency >= 0.0f && isFinite(profile.mStanceHipDamping)
                && profile.mStanceHipDamping >= 0.0f && isFinite(profile.mSwingTravelStartFraction)
                && profile.mSwingTravelStartFraction >= 0.0f && isFinite(profile.mEmergencyCaptureMargin)
                && profile.mEmergencyCaptureMargin >= 0.0f && isFinite(profile.mEmergencyStanceLoadFraction)
                && profile.mEmergencyStanceLoadFraction >= 0.0f && profile.mEmergencyStanceLoadFraction <= 1.0f
                && isFinite(profile.mSteeringTurnRate)
                && profile.mSteeringTurnRate >= 0.0f && isFinite(profile.mSteeringPelvisTwist)
                && profile.mSteeringPelvisTwist >= 0.0f && isFinite(profile.mSteeringSpeedChangeRate)
                && profile.mSteeringSpeedChangeRate >= 0.0f && isFinite(profile.mSteeringStopDeceleration)
                && profile.mSteeringStopDeceleration >= 0.0f && isFinite(profile.mSteeringStopSpeed)
                && profile.mSteeringStopSpeed >= 0.0f && isFinite(profile.mSteeringTurnSlowdown)
                && profile.mSteeringTurnSlowdown >= 0.0f && profile.mSteeringTurnSlowdown <= 1.0f
                && isFinite(profile.mStoppingVelocityErrorGain) && profile.mStoppingVelocityErrorGain >= 0.0f
                && isFinite(profile.mSteeringStopTimeout) && profile.mSteeringStopTimeout >= 0.0f
                && isFinite(profile.mStandingHandoverDuration) && profile.mStandingHandoverDuration >= 0.0f
                && isFinite(profile.mHitStunMinimumImpulse) && profile.mHitStunMinimumImpulse >= 0.0f
                && isFinite(profile.mHitStunDuration) && profile.mHitStunDuration >= 0.0f
                && isFinite(profile.mHitStunReferenceImpulse) && profile.mHitStunReferenceImpulse > 0.0f
                && isFinite(profile.mHitStunStaggerImpulse)
                && profile.mHitStunStaggerImpulse >= profile.mHitStunMinimumImpulse
                && isFinite(profile.mHitStunFallImpulse)
                && profile.mHitStunFallImpulse >= profile.mHitStunStaggerImpulse
                && profile.mSwingTravelStartFraction < profile.mSwingTravelEndFraction
                && isFinite(profile.mSwingInertiaFeedForward) && profile.mSwingInertiaFeedForward >= 0.0f
                && isFinite(profile.mSwingFeedForwardAccelerationLimit)
                && profile.mSwingFeedForwardAccelerationLimit >= 0.0f
                && isFinite(profile.mFinalCommandFeasibilityStanceMargin)
                && profile.mFinalCommandFeasibilityStanceMargin >= 0.0f
                && isFinite(profile.mFinalCommandFeasibilitySwingMargin)
                && profile.mFinalCommandFeasibilitySwingMargin >= 0.0f;
        }

        float smoothStep(float value)
        {
            const float clamped = std::clamp(value, 0.0f, 1.0f);
            return clamped * clamped * (3.0f - 2.0f * clamped);
        }

        JPH::Vec3 horizontal(JPH::Vec3Arg value)
        {
            return JPH::Vec3(value.GetX(), value.GetY(), 0.0f);
        }

        GaitPhase oppositeStance(GaitPhase phase)
        {
            return phase == GaitPhase::LeftStance ? GaitPhase::RightStance : GaitPhase::LeftStance;
        }

        struct FootLoad
        {
            double mNormalImpulse = 0.0;
            bool mKnown = true;
            bool mSupported = false;
        };

        FootLoad measureFootLoad(std::span<const GroundContact> contacts, std::size_t foot)
        {
            FootLoad load;
            for (const GroundContact& contact : contacts)
            {
                if (contact.mBodyIndex != foot || !contact.hasSupport())
                    continue;
                load.mSupported = true;
                if (!contact.mNormalImpulse || !isFinite(*contact.mNormalImpulse) || *contact.mNormalImpulse < 0.0f)
                    load.mKnown = false;
                else
                    load.mNormalImpulse += *contact.mNormalImpulse;
            }
            return load;
        }
    }

    const char* referenceGaitStageName(ReferenceGaitStage stage)
    {
        switch (stage)
        {
            case ReferenceGaitStage::Idle:
                return "idle";
            case ReferenceGaitStage::DoubleSupport:
                return "double_support";
            case ReferenceGaitStage::Swing:
                return "swing";
        }
        return "invalid";
    }

    JPH::Vec3 governForwardReserve(JPH::Vec3Arg acceleration, JPH::Vec3Arg forward, JPH::Vec3Arg right,
        float maximumAcceleration, float forwardReserveFraction)
    {
        if (!std::isfinite(forwardReserveFraction) || forwardReserveFraction <= 0.0f
            || !std::isfinite(maximumAcceleration) || maximumAcceleration <= 0.0f
            || !isFinite(acceleration) || !isFinite(forward) || !isFinite(right))
            return acceleration;
        const float fwd = acceleration.Dot(forward);
        const float lat = acceleration.Dot(right);
        const float reserve
            = std::min(std::abs(fwd), std::min(forwardReserveFraction, 1.0f) * maximumAcceleration);
        const float lateralCap
            = std::sqrt(std::max(maximumAcceleration * maximumAcceleration - reserve * reserve, 0.0f));
        const float capped = std::clamp(lat, -lateralCap, lateralCap);
        if (capped == lat)
            return acceleration;
        return forward * fwd + right * capped;
    }

    bool ReferenceGaitStateMachine::step(const ReferenceGaitStateInput& input, float duration,
        const ReferenceGaitProfile& profile, ReferenceGaitState& output)
    {
        if (!isFinite(duration) || duration <= 0.0f || !validProfile(profile) || !isFinite(input.mLeftFootHeight)
            || !isFinite(input.mRightFootHeight) || !isFinite(input.mLeftFootVerticalVelocity)
            || !isFinite(input.mRightFootVerticalVelocity) || !isFinite(input.mLeftFootLoadFraction)
            || !isFinite(input.mRightFootLoadFraction) || !isFinite(input.mLeftFootCenter)
            || !isFinite(input.mRightFootCenter))
            return false;
        // Support is always required; a known load must also reach the
        // threshold, an unknown load falls back to contact presence.
        const auto loaded = [](bool supported, float fraction, float threshold) {
            return supported && (fraction < 0.0f || fraction >= threshold);
        };

        ReferenceGaitState& state = mState;
        state.mTransitioned = false;
        if (!input.mEnabled)
        {
            reset();
            output = mState;
            return true;
        }

        // Count continuous support, including support before an emergency is
        // requested. A standing loaded foot can react promptly; one impact
        // sample on a newly landed foot cannot unload its partner immediately.
        const float emergencyLoad = std::max(profile.mSwingStartLoadFraction, profile.mEmergencyStanceLoadFraction);
        // Disturbance observer (i12, placement-lead-observer-2026-09-11.txt):
        // a one-step horizontal COM velocity jump is a push signature
        // (measured: cruise <= 3.2 u/s including touchdowns, a 3000 spine
        // impulse 41.5 u/s). Latch a 0.5 s window covering the arrest swing.
        // The latch is published read-only; nothing in this machine reads it.
        if (isFinite(input.mCenterOfMassVelocity))
        {
            const JPH::Vec3 planar = horizontal(input.mCenterOfMassVelocity);
            if (mPreviousComVelocity && (planar - *mPreviousComVelocity).Length() > 10.0f)
                mDisturbanceLatch = 0.5f;
            mPreviousComVelocity = planar;
        }
        mDisturbanceLatch = std::max(0.0f, mDisturbanceLatch - duration);
        state.mDisturbanceLatchRemaining = mDisturbanceLatch;
        const auto confirmLoad = [&](float& time, bool supported, float fraction) {
            time = loaded(supported, fraction, emergencyLoad)
                ? std::min(time + duration, profile.mContactConfirmationDuration) : 0.0f;
        };
        confirmLoad(mLeftEmergencyLoadTime, input.mLeftFootSupported, input.mLeftFootLoadFraction);
        confirmLoad(mRightEmergencyLoadTime, input.mRightFootSupported, input.mRightFootLoadFraction);
        const auto swingRelativeHeight = [&](GaitPhase phase) {
            return phase == GaitPhase::LeftStance ? input.mRightFootHeight - input.mLeftFootHeight
                                                  : input.mLeftFootHeight - input.mRightFootHeight;
        };
        const auto transferCenter = [&](GaitPhase phase) {
            return phase == GaitPhase::LeftStance ? input.mLeftFootCenter : input.mRightFootCenter;
        };
        if (state.mPhase == GaitPhase::Idle)
        {
            if (input.mLeftFootSupported)
                state.mPhase = GaitPhase::LeftStance;
            else if (input.mRightFootSupported)
                state.mPhase = GaitPhase::RightStance;
            else
            {
                output = state;
                return true;
            }
            // Fable i57 (env-gated): a sideways travel starts with the LEADING
            // foot swinging (the foot on the travel side), so the first step of
            // a strafe is a side step and not a crossover. Forward travel and
            // a single supported foot keep the historical choice.
            static const auto initialStanceEnv_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_INITIAL_STANCE_BY_TRAVEL") != nullptr; });
            static const auto& initialStanceEnv = initialStanceEnv_tuned.get();
            const bool initialStanceByTravel = initialStanceEnv || profile.mInitialStanceByTravel;
            if (initialStanceByTravel && input.mLeftFootSupported && input.mRightFootSupported)
            {
                const JPH::Vec3 lateral = horizontal(JPH::Vec3(input.mLeftFootCenter - input.mRightFootCenter));
                const JPH::Vec3 heading = horizontal(input.mHeading);
                const float denominator = lateral.Length() * heading.Length();
                if (denominator > 1e-6f)
                {
                    const float cosine = lateral.Dot(heading) / denominator;
                    if (cosine > 0.5f)
                        state.mPhase = GaitPhase::RightStance; // travel toward the left foot: left swings first
                    else if (cosine < -0.5f)
                        state.mPhase = GaitPhase::LeftStance;
                }
            }
            state.mStage = ReferenceGaitStage::DoubleSupport;
            state.mTransferCenter = transferCenter(state.mPhase);
            state.mTimeInPhase = 0.0f;
            state.mTimeInStage = 0.0f;
            state.mSwingContactTime = 0.0f;
            state.mSwingTimeSinceLift = 0.0f;
            state.mSwingStartRelativeHeight = swingRelativeHeight(state.mPhase);
            state.mSwingFootLifted = false;
            state.mTimedOut = false;
            state.mTransitioned = true;
            output = state;
            return true;
        }

        state.mTimeInPhase += duration;
        state.mTimeInStage += duration;
        const bool left = state.mPhase == GaitPhase::LeftStance;
        const bool swingSupported = left ? input.mRightFootSupported : input.mLeftFootSupported;
        const float swingVerticalVelocity = left ? input.mRightFootVerticalVelocity : input.mLeftFootVerticalVelocity;
        const float stanceVerticalVelocity = left ? input.mLeftFootVerticalVelocity : input.mRightFootVerticalVelocity;
        const bool stanceLoaded = loaded(left ? input.mLeftFootSupported : input.mRightFootSupported,
            left ? input.mLeftFootLoadFraction : input.mRightFootLoadFraction, profile.mSwingStartLoadFraction);
        const bool swingLoaded = loaded(swingSupported,
            left ? input.mRightFootLoadFraction : input.mLeftFootLoadFraction, profile.mTransferLoadFraction);
        if (state.mStage == ReferenceGaitStage::DoubleSupport)
        {
            if (input.mEmergencyOnly)
            {
                // A prolonged stationary hold ends the walking transfer cycle.
                // Its next ordinary swing must establish support again, rather
                // than inheriting permission to lift from the previous walk.
                state.mInitialTransfer = true;
                mInitialStanceLoadTime = 0.0f;
            }
            const bool otherLoaded = loaded(swingSupported,
                left ? input.mRightFootLoadFraction : input.mLeftFootLoadFraction, profile.mSwingStartLoadFraction);
            const bool stanceSupported = left ? input.mLeftFootSupported : input.mRightFootSupported;
            if (state.mEnteredFromTouchdown && !stanceLoaded && otherLoaded
                && (state.mTimeInStage >= profile.mMaximumDoubleSupportDuration
                    || (!stanceSupported && state.mTimeInStage >= profile.mLostLandingSwapDelay)))
            {
                // The landed foot bounced off and never took the load while the
                // trailing foot still carries it: swap the stance back.
                state.mPhase = oppositeStance(state.mPhase);
                state.mTransferCenter = transferCenter(state.mPhase);
                state.mEnteredFromTouchdown = false;
                state.mTimedOut = false;
                state.mTimeInStage = 0.0f;
                state.mSwingStartRelativeHeight = swingRelativeHeight(state.mPhase);
                output = state;
                return true;
            }
            const auto beginSwing = [&](bool emergency) {
                state.mInitialTransfer = false;
                state.mStage = ReferenceGaitStage::Swing;
                state.mEnteredFromTouchdown = false;
                state.mTimedOut = false;
                state.mTimeInStage = 0.0f;
                state.mSwingContactTime = 0.0f;
                state.mSwingTimeSinceLift = 0.0f;
                state.mSwingStartRelativeHeight = swingRelativeHeight(state.mPhase);
                const bool alreadyUnsupported = state.mPhase == GaitPhase::LeftStance
                    ? !input.mRightFootSupported : !input.mLeftFootSupported;
                // Recovery may hand over a foot already clear of the floor,
                // including on its descent. Do not require a second lift event
                // before accepting its later confirmed, loaded touchdown.
                state.mSwingFootLifted = emergency && alreadyUnsupported
                    && state.mSwingStartRelativeHeight >= std::max(1e-3f, 0.1f * profile.mSwingFootLift);
                state.mEmergencyStep = emergency;
                if (emergency)
                    ++state.mEmergencyStepCount;
            };
            // Capture-brake hold (env-gated experiment, inert by default):
            // while a capture escape stays within K margins and the
            // double-support window has not expired, take no emergency step
            // and extend the double support so the plan's capture brake can
            // arrest the escape (capture-brake-hold-2026-09-10.txt).
            static const auto captureBrakeHold_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CAPTURE_BRAKE");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& captureBrakeHold = captureBrakeHold_tuned.get();
            float brakeExcess = -1.0f;
            // Reactive step: a capture point outside the two-foot support
            // (plus margin) cannot be held by either ankle, so the loaded foot
            // nearer to it becomes the stance foot and the other steps toward
            // it now. A swing already in progress is never interrupted.
            if (profile.mEmergencyCaptureMargin > 0.0f && input.mCenterOfMass && isFinite(*input.mCenterOfMass)
                && isFinite(input.mCenterOfMassVelocity) && isFinite(input.mCenterOfMassHeight)
                && input.mCenterOfMassHeight > 0.0f && isFinite(input.mGravity) && input.mGravity > 0.0f)
            {
                const float tau = std::sqrt(input.mCenterOfMassHeight / input.mGravity);
                const JPH::Vec3 capture
                    = horizontal(JPH::Vec3(*input.mCenterOfMass)) + horizontal(input.mCenterOfMassVelocity) * tau;
                const JPH::Vec3 leftCenter = horizontal(JPH::Vec3(input.mLeftFootCenter));
                const JPH::Vec3 rightCenter = horizontal(JPH::Vec3(input.mRightFootCenter));
                const JPH::Vec3 span = rightCenter - leftCenter;
                const float spanLength2 = span.LengthSq();
                // An airborne foot supplies anatomical side information, not
                // ground support. Collapse the support segment to the actual
                // contacting foot instead of balancing against empty space.
                const JPH::Vec3 supportStart = input.mLeftFootSupported ? leftCenter : rightCenter;
                const JPH::Vec3 supportEnd = input.mRightFootSupported ? rightCenter : leftCenter;
                const JPH::Vec3 supportSpan = supportEnd - supportStart;
                const float supportLength2 = supportSpan.LengthSq();
                const float along = supportLength2 > 1e-6f
                    ? std::clamp((capture - supportStart).Dot(supportSpan) / supportLength2, 0.0f, 1.0f) : 0.0f;
                const float excess = (capture - (supportStart + supportSpan * along)).Length();
                brakeExcess = excess;
                // Diagnostic (env-gated; hip-moment-borrow-2026-09-11.txt):
                // log every reactive-step evaluation above the margin with
                // the gate values, to find why high-impulse falls never
                // trigger the rule. Debug only, no behavior change.
                const bool debugReactive = RagdollCore::tuningText("OPENMW_RAGDOLL_DEBUG_REACTIVE") != nullptr;
                if (debugReactive && std::isfinite(excess) && excess > profile.mEmergencyCaptureMargin)
                    std::fprintf(stderr,
                        "reactive-eval stage_t=%.3f excess=%.1f lsup=%d rsup=%d span2=%.4f cap=(%.1f,%.1f) L=(%.1f,%.1f) R=(%.1f,%.1f)\n",
                        state.mTimeInStage, excess, (int)input.mLeftFootSupported,
                        (int)input.mRightFootSupported, spanLength2,
                        capture.GetX(), capture.GetY(), leftCenter.GetX(), leftCenter.GetY(),
                        rightCenter.GetX(), rightCenter.GetY());
                const bool holdForBrake = captureBrakeHold > 0.0f && std::isfinite(excess)
                    && excess <= captureBrakeHold * profile.mEmergencyCaptureMargin
                    && state.mTimeInStage < profile.mMaximumDoubleSupportDuration;
                if (!holdForBrake && (input.mLeftFootSupported || input.mRightFootSupported)
                    && std::isfinite(excess) && excess > profile.mEmergencyCaptureMargin && spanLength2 > 1e-6f)
                {
                    // The stepping foot must not cross the stance leg: its
                    // landing lies on its own side of the stance foot along the
                    // lateral axis of the requested heading. A capture point
                    // beside the feet therefore steps the near foot outward;
                    // one ahead or behind steps whichever foot is farther from
                    // it (the trailing foot).
                    JPH::Vec3 lateral = horizontal(input.mHeading).Cross(JPH::Vec3::sAxisZ());
                    if (!isFinite(lateral) || lateral.IsNearZero())
                        lateral = span;
                    lateral = lateral.Normalized();
                    const bool rightMaySwing = (capture - leftCenter).Dot(lateral) >= 0.0f;
                    const bool leftMaySwing = (capture - rightCenter).Dot(lateral) <= 0.0f;
                    const bool leftFarther = (capture - leftCenter).LengthSq() >= (capture - rightCenter).LengthSq();
                    // i16 (reactive-stance-tenure-2026-09-11.txt): during a
                    // latched disturbance, the current phase's stance foot is
                    // support-proven by the phase's own transfer gate, which
                    // accepted it on contact (mTransferLoadFraction = 0);
                    // hold it to that same standard here. The other foot
                    // keeps the full 30 ms / 50% emergency tenure.
                    const bool latchActive
                        = state.mDisturbanceLatchRemaining > 0.0f && !state.mInitialTransfer;
                    const bool stanceProvenLeft = latchActive && state.mPhase == GaitPhase::LeftStance;
                    const bool stanceProvenRight = latchActive && state.mPhase == GaitPhase::RightStance;
                    const bool leftCanSupport = stanceProvenLeft
                        ? loaded(input.mLeftFootSupported, input.mLeftFootLoadFraction,
                            profile.mTransferLoadFraction)
                        : loaded(input.mLeftFootSupported, input.mLeftFootLoadFraction,
                            emergencyLoad) && mLeftEmergencyLoadTime >= profile.mContactConfirmationDuration;
                    const bool rightCanSupport = stanceProvenRight
                        ? loaded(input.mRightFootSupported, input.mRightFootLoadFraction,
                            profile.mTransferLoadFraction)
                        : loaded(input.mRightFootSupported, input.mRightFootLoadFraction,
                            emergencyLoad) && mRightEmergencyLoadTime >= profile.mContactConfirmationDuration;
                    if (debugReactive)
                        std::fprintf(stderr,
                            "  gates: rightMay=%d leftMay=%d leftCan=%d rightCan=%d leftLoadT=%.3f rightLoadT=%.3f confirm=%.3f latOff=%.2f\n",
                            (int)rightMaySwing, (int)leftMaySwing, (int)leftCanSupport, (int)rightCanSupport,
                            mLeftEmergencyLoadTime, mRightEmergencyLoadTime,
                            profile.mContactConfirmationDuration,
                            (capture - (supportStart + supportSpan * along)).Dot(lateral));
                    // Prefer the farther stepping foot only among physically
                    // available supports. An unavailable preferred stance must
                    // not suppress a legal correction using the other foot.
                    bool swingLeft = leftMaySwing && rightCanSupport
                        && (!rightMaySwing || !leftCanSupport || leftFarther);
                    // Opt-in A62: when both choices are safe, avoid lifting the
                    // newly landed foot again solely because capture distances
                    // exchange order. Unavailable supports retain the old rule.
                    if (RagdollCore::tuningText("OPENMW_RAGDOLL_KEEP_EMERGENCY_STANCE")
                        && leftMaySwing && rightCanSupport && rightMaySwing && leftCanSupport)
                        swingLeft = state.mPhase == GaitPhase::RightStance;
                    const GaitPhase stance = swingLeft ? GaitPhase::RightStance : GaitPhase::LeftStance;
                    if (swingLeft || (rightMaySwing && leftCanSupport))
                    {
                        if (stance != state.mPhase)
                        {
                            state.mPhase = stance;
                            state.mTransferCenter = transferCenter(stance);
                            state.mTimeInPhase = 0.0f;
                        }
                        beginSwing(true);
                        state.mTransitioned = true;
                        output = state;
                        return true;
                    }
                }
            }
            // Initial weight transfer is not a failed landing. Keep its goal
            // until the intended support foot carries load; do not alternate
            // the goal or start an unsupported swing when this clock expires.
            bool approachingInitialStance = true;
            bool initialStanceReady = true;
            if (profile.mRequireInitialSupportTransfer && state.mInitialTransfer
                && input.mCenterOfMass && isFinite(*input.mCenterOfMass)
                && isFinite(input.mCenterOfMassVelocity))
            {
                // Only startup: later transfers retain their existing timing.
                // Reuse the emergency support threshold and confirmation
                // interval instead of lifting off a barely loaded initial foot.
                const float fraction = left ? input.mLeftFootLoadFraction : input.mRightFootLoadFraction;
                const bool knownLoad = stanceSupported && fraction >= std::max(
                    profile.mSwingStartLoadFraction, profile.mEmergencyStanceLoadFraction);
                mInitialStanceLoadTime = knownLoad ? mInitialStanceLoadTime + duration : 0.0f;
                initialStanceReady = knownLoad && mInitialStanceLoadTime >= profile.mContactConfirmationDuration;
                JPH::Vec3 lateral = horizontal(input.mHeading).Cross(JPH::Vec3::sAxisZ());
                if (lateral.IsNearZero())
                    lateral = horizontal(JPH::Vec3(input.mRightFootCenter - input.mLeftFootCenter));
                if (isFinite(lateral) && !lateral.IsNearZero())
                {
                    lateral = lateral.Normalized();
                    const float offset = JPH::Vec3(transferCenter(state.mPhase) - *input.mCenterOfMass).Dot(lateral);
                    // Starting single support while still moving away from
                    // its pivot amplifies the initial lateral excursion.
                    // Retain the ordinary transfer controller until that
                    // motion reverses. Emergency steps retain their own gate.
                    approachingInitialStance = offset * input.mCenterOfMassVelocity.Dot(lateral) >= 0.0f;
                }
            }
            // While a capture escape remains outside the margin, extend the
            // double support (up to its maximum) so the brake can arrest it.
            const bool brakeHoldActive = captureBrakeHold > 0.0f
                && brakeExcess > profile.mEmergencyCaptureMargin
                && state.mTimeInStage < profile.mMaximumDoubleSupportDuration;
            if (!input.mEmergencyOnly && state.mTimeInStage >= profile.mDoubleSupportDuration && stanceLoaded
                && approachingInitialStance && initialStanceReady && !brakeHoldActive)
                beginSwing(false);
            else
                state.mTimedOut = !input.mEmergencyOnly && state.mTimeInStage >= profile.mMaximumStepDuration;
            output = state;
            return true;
        }

        const float minimumClearance = std::max(1e-3f, 0.1f * profile.mSwingFootLift);
        const float clearance = swingRelativeHeight(state.mPhase) - state.mSwingStartRelativeHeight;
        if (!state.mSwingFootLifted && !swingSupported && clearance >= minimumClearance
            && swingVerticalVelocity - stanceVerticalVelocity > 0.0f && swingVerticalVelocity > 0.0f)
            state.mSwingFootLifted = true;
        if (state.mSwingFootLifted)
            state.mSwingTimeSinceLift += duration;
        // Emergency follow-through (i15b,
        // emergency-transfer-2026-09-11.txt): a reactive step lands the catch
        // foot AWAY from the current COM motion, so organic press-down never
        // comes, and the foot may skim to the catch position without ever
        // lifting (measured i14/i15: vz +/-60 u/s, rsup=1, swing_lifted=0).
        // For an emergency swing, touchdown = contact + minimum swing
        // duration + 30 ms contact confirmation; the ordinary path keeps the
        // lift and load-fraction requirements. Cruise never sets
        // mEmergencyStep, so unperturbed walking is byte-identical.
        const bool touchdownReady = state.mEmergencyStep
            ? swingSupported : (state.mSwingFootLifted && swingLoaded);
        if (touchdownReady)
            state.mSwingContactTime += duration;
        else
            state.mSwingContactTime = 0.0f;
        if (touchdownReady && state.mTimeInStage >= profile.mMinimumSwingDuration
            && state.mSwingContactTime >= profile.mContactConfirmationDuration)
        {
            // i39 (delivery-comp-2026-09-11.txt): delivery-error observer.
            // At a confirmed touchdown, compare the measured COM velocity
            // with the plan's prediction for this touchdown along the
            // travel axis, and low-pass the difference into the bias the
            // placement adds to its predicted speed. Delivered speed
            // systematically exceeds the plan in backpedal (saturated-hip
            // pumping, i38) and falls short forward; a signed bias
            // compensates both. Env-gated: unset alpha = no update, bias
            // stays exactly 0, placement path byte-identical.
            static const auto deliveryCompAlpha_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DELIVERY_COMP");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& deliveryCompAlpha = deliveryCompAlpha_tuned.get();
            if (deliveryCompAlpha > 0.0f && input.mPlacementPredictedVelocity
                && isFinite(*input.mPlacementPredictedVelocity))
            {
                const JPH::Vec3 axis = horizontal(input.mHeading);
                if (!axis.IsNearZero())
                {
                    const JPH::Vec3 travel = axis.Normalized();
                    const float error = horizontal(input.mCenterOfMassVelocity - *input.mPlacementPredictedVelocity)
                                            .Dot(travel);
                    state.mDeliveryVelocityBias += deliveryCompAlpha
                        * (std::clamp(error, -40.0f, 40.0f) - state.mDeliveryVelocityBias);
                    state.mDeliveryVelocityBias = std::clamp(state.mDeliveryVelocityBias, -40.0f, 40.0f);
                    // Temporary i39 diagnostic (delivery-comp doc): bias
                    // trajectory at each touchdown. Diagnostic-only; remove
                    // or keep inert before final disposition.
                    if (RagdollCore::tuningText("OPENMW_RAGDOLL_DELIVERY_COMP_TRACE"))
                        std::fprintf(stderr, "DELIVERY_COMP error=%f bias=%f\n", static_cast<double>(error),
                            static_cast<double>(state.mDeliveryVelocityBias));
                }
            }
            state.mPhase = oppositeStance(state.mPhase);
            state.mTransferCenter = transferCenter(state.mPhase);
            state.mEnteredFromTouchdown = true;
            state.mEmergencyStep = false;
            state.mStage = ReferenceGaitStage::DoubleSupport;
            state.mTimeInPhase = 0.0f;
            state.mTimeInStage = 0.0f;
            state.mSwingContactTime = 0.0f;
            state.mSwingTimeSinceLift = 0.0f;
            state.mSwingStartRelativeHeight = swingRelativeHeight(state.mPhase);
            state.mSwingFootLifted = false;
            state.mTimedOut = false;
            state.mTransitioned = true;
        }
        else
            state.mTimedOut = state.mTimeInStage >= profile.mMaximumStepDuration;
        // i17 diagnostic (reactive-single-support-2026-09-11.txt;
        // env-gated, no behavior change): evaluate the capture-point
        // excess against the single stance foot while in Swing, to test
        // whether the 3000/3500 lateral falls carry un-served capture
        // escapes through single support and whether a swing-foot
        // redirect could be actuation-feasible.
        static const auto debugSwingReactive_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_DEBUG_SWING_REACTIVE") != nullptr; });
        static const auto& debugSwingReactive = debugSwingReactive_tuned.get();
        if (debugSwingReactive && profile.mEmergencyCaptureMargin > 0.0f && input.mCenterOfMass
            && isFinite(*input.mCenterOfMass) && isFinite(input.mCenterOfMassVelocity)
            && isFinite(input.mCenterOfMassHeight) && input.mCenterOfMassHeight > 0.0f
            && isFinite(input.mGravity) && input.mGravity > 0.0f)
        {
            const float tau = std::sqrt(input.mCenterOfMassHeight / input.mGravity);
            const JPH::Vec3 capture
                = horizontal(JPH::Vec3(*input.mCenterOfMass)) + horizontal(input.mCenterOfMassVelocity) * tau;
            const JPH::Vec3 stanceCenter
                = horizontal(JPH::Vec3(left ? input.mLeftFootCenter : input.mRightFootCenter));
            const JPH::Vec3 swingCenter
                = horizontal(JPH::Vec3(left ? input.mRightFootCenter : input.mLeftFootCenter));
            const float swingExcess = (capture - stanceCenter).Length();
            if (swingExcess > profile.mEmergencyCaptureMargin)
                std::fprintf(stderr,
                    "swing-reactive t=%.3f excess=%.1f swing_t=%.3f remain=%.3f foot_dist=%.1f foot_h=%.1f emergency=%d\n",
                    state.mTimeInPhase, swingExcess, state.mTimeInStage,
                    profile.mSwingDuration - state.mTimeInStage,
                    (capture - swingCenter).Length(),
                    left ? input.mRightFootHeight : input.mLeftFootHeight,
                    (int)state.mEmergencyStep);
        }
        output = state;
        return true;
    }

    void ReferenceGaitStateMachine::reset()
    {
        mState = ReferenceGaitState{};
        mLeftEmergencyLoadTime = 0.0f;
        mRightEmergencyLoadTime = 0.0f;
        mInitialStanceLoadTime = 0.0f;
        mPreviousComVelocity = std::nullopt;
        mDisturbanceLatch = 0.0f;
    }

    void ReferenceGaitSteering::stepFacing(
        JPH::Vec3Arg requestedFacing, float duration, const ReferenceGaitProfile& profile)
    {
        if (!isFinite(requestedFacing) || !isFinite(duration) || duration <= 0.0f || !validProfile(profile))
            return;
        const JPH::Vec3 request = horizontal(requestedFacing);
        if (request.IsNearZero())
            return;
        const JPH::Vec3 target = request.Normalized();
        if (mFacing.IsNearZero() || profile.mSteeringTurnRate <= 0.0f)
        {
            mFacing = target;
            return;
        }
        const float cosine = std::clamp(mFacing.Dot(target), -1.0f, 1.0f);
        const float angle = std::acos(cosine);
        const float maximum = profile.mSteeringTurnRate * duration;
        if (angle <= maximum || angle <= 1e-4f)
        {
            mFacing = target;
            return;
        }
        // Signed turn about Z; an exact reversal turns left, as the travel
        // heading does.
        const float cross = mFacing.GetX() * target.GetY() - mFacing.GetY() * target.GetX();
        const float rotation = (cross >= 0.0f ? 1.0f : -1.0f) * maximum;
        const float c = std::cos(rotation), s = std::sin(rotation);
        mFacing = JPH::Vec3(
            mFacing.GetX() * c - mFacing.GetY() * s, mFacing.GetX() * s + mFacing.GetY() * c, 0.0f)
                      .Normalized();
    }

    JPH::Vec3 ReferenceGaitSteering::step(JPH::Vec3Arg rawRequest, float duration,
        const ReferenceGaitProfile& profile, bool standingReady, bool standingReadyIgnoringSpeed, bool directTravel,
        std::optional<JPH::Vec3> measuredVelocity)
    {
        if (!isFinite(rawRequest) || !isFinite(duration) || duration <= 0.0f || !validProfile(profile))
            return mCommand;
        const JPH::Vec3 request = horizontal(rawRequest);
        const float requestedSpeed = request.Length();
        float commandSpeed = mCommand.Length();
        // Creep speed: the slowest command the gait still walks; a stop holds
        // it until both feet carry the body.
        const float creep = profile.mSteeringStopSpeed + 1.0f;
        if (request.IsNearZero())
        {
            // Stop: decelerate along the current heading to the creep speed,
            // then hand over (snap to zero) once standing is possible, or at
            // the next double support after the stop timeout. With no
            // deceleration limit the command snaps as soon as it may.
            if (commandSpeed <= 1e-3f || profile.mSteeringStopDeceleration <= 0.0f)
            {
                // No deceleration limit: the pre-steering semantics, the
                // request is followed at once and standing takes over now.
                mCommand = JPH::Vec3::sZero();
                mStopping = false;
                mStoppingTime = 0.0f;
                return mCommand;
            }
            if (!mStopping && measuredVelocity && isFinite(*measuredVelocity))
            {
                const float forwardSpeed = horizontal(*measuredVelocity).Dot(mCommand / commandSpeed);
                const float stoppingSpeed = std::min(commandSpeed, std::max(creep, forwardSpeed));
                mCommand *= stoppingSpeed / commandSpeed;
                commandSpeed = stoppingSpeed;
            }
            mStopping = true;
            mStoppingTime += duration;
            const bool timedOut = profile.mSteeringStopTimeout > 0.0f && mStoppingTime >= profile.mSteeringStopTimeout;
            if (commandSpeed <= creep + 1e-3f)
            {
                if (standingReady || (timedOut && standingReadyIgnoringSpeed))
                {
                    mCommand = JPH::Vec3::sZero();
                    mStopping = false;
                    mStoppingTime = 0.0f;
                }
                else
                    mCommand *= creep / commandSpeed;
            }
            else
                mCommand *= std::max(creep, commandSpeed - profile.mSteeringStopDeceleration * duration) / commandSpeed;
            return mCommand;
        }
        mStopping = false;
        mStoppingTime = 0.0f;
        if (commandSpeed <= profile.mSteeringStopSpeed)
        {
            // Starting from rest keeps the last heading when there is one
            // (the turn toward the request then runs at the turn rate from
            // the next step on) and otherwise takes the requested heading
            // at once; the speed still ramps when a change rate is set.
            float speed = requestedSpeed;
            if (profile.mSteeringSpeedChangeRate > 0.0f)
                speed = std::min(requestedSpeed,
                    std::max(profile.mSteeringStopSpeed, commandSpeed) + profile.mSteeringSpeedChangeRate * duration);
            JPH::Vec3 heading = request / requestedSpeed;
            if (profile.mSteeringTurnRate > 0.0f && !mLastHeading.IsNearZero())
                heading = mLastHeading;
            mCommand = heading * speed;
            mLastHeading = heading;
            return mCommand;
        }
        // Heading: rotate the command toward the request by at most the turn
        // rate. Speed: move toward the requested speed by at most the rate.
        JPH::Vec3 heading = mCommand / commandSpeed;
        const JPH::Vec3 requestHeading = request / requestedSpeed;
        if (directTravel)
        {
            // The player's input is not swept round at the turn rate: the
            // request takes effect at once and the body accelerates into it.
            heading = requestHeading;
            // i25 (player-turn-snap-2026-09-11.txt): opt-in speed-aware sweep.
            // An instant 90-degree snap at cruise demands a velocity rotation
            // the friction cone cannot deliver (measured over-brake -194.5
            // u/s^2 in turn90pi, harder than a full stop) and creates the
            // post-turn collapse state. Bounding the heading rotation to
            // budget/speed keeps the mismatch inside the friction budget.
            // Env-gated, inert when unset or 0; not encoded, not profiled.
            static const auto snapBudget_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SNAP_BUDGET");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& snapBudget = snapBudget_tuned.get();
            if (snapBudget > 0.0f)
            {
                const JPH::Vec3 commandHeading = mCommand / commandSpeed;
                const float cosine = std::clamp(commandHeading.Dot(requestHeading), -1.0f, 1.0f);
                const float angle = std::acos(cosine);
                const float maximum = snapBudget / std::max(commandSpeed, 1.0f) * duration;
                if (angle > maximum && angle > 1e-4f)
                {
                    // Signed turn about Z; an exact reversal turns left.
                    const float cross = commandHeading.GetX() * requestHeading.GetY()
                        - commandHeading.GetY() * requestHeading.GetX();
                    const float sign = cross >= 0.0f ? 1.0f : -1.0f;
                    const float rotation = sign * maximum;
                    const float c = std::cos(rotation), s = std::sin(rotation);
                    heading = JPH::Vec3(commandHeading.GetX() * c - commandHeading.GetY() * s,
                        commandHeading.GetX() * s + commandHeading.GetY() * c, 0.0f);
                }
            }
        }
        else if (profile.mSteeringTurnRate > 0.0f)
        {
            const float cosine = std::clamp(heading.Dot(requestHeading), -1.0f, 1.0f);
            const float angle = std::acos(cosine);
            const float maximum = profile.mSteeringTurnRate * duration;
            if (angle > maximum && angle > 1e-4f)
            {
                // Signed turn about Z; an exact reversal turns left.
                const float cross = heading.GetX() * requestHeading.GetY() - heading.GetY() * requestHeading.GetX();
                const float sign = cross >= 0.0f ? 1.0f : -1.0f;
                const float rotation = sign * maximum;
                const float c = std::cos(rotation), s = std::sin(rotation);
                heading = JPH::Vec3(heading.GetX() * c - heading.GetY() * s, heading.GetX() * s + heading.GetY() * c, 0.0f);
            }
            else
                heading = requestHeading;
        }
        else
            heading = requestHeading;
        float speed = requestedSpeed;
        if (profile.mSteeringTurnSlowdown > 0.0f)
        {
            const float remaining = std::acos(std::clamp(heading.Dot(requestHeading), -1.0f, 1.0f));
            const float fraction = std::min(1.0f, remaining / (0.5f * std::numbers::pi_v<float>));
            speed *= std::max(0.0f, 1.0f - profile.mSteeringTurnSlowdown * fraction);
        }
        if (profile.mSteeringSpeedChangeRate > 0.0f)
        {
            const float maximum = profile.mSteeringSpeedChangeRate * duration;
            speed = std::clamp(speed, commandSpeed - maximum, commandSpeed + maximum);
        }
        mCommand = heading * std::max(speed, creep);
        mLastHeading = heading;
        return mCommand;
    }

    void ReferenceGaitHitStun::begin(float impulseMagnitude, const ReferenceGaitProfile& profile)
    {
        if (!validProfile(profile) || profile.mHitStunDuration <= 0.0f || !std::isfinite(impulseMagnitude)
            || impulseMagnitude < profile.mHitStunMinimumImpulse)
            return;
        const float scale = std::clamp(impulseMagnitude / profile.mHitStunReferenceImpulse, 0.0f, 2.0f);
        mRemaining = std::max(mRemaining, profile.mHitStunDuration * scale);
    }

    void ReferenceGaitHitStun::step(float duration)
    {
        if (!std::isfinite(duration) || duration <= 0.0f)
            return;
        mRemaining = std::max(0.0f, mRemaining - duration);
    }

    void ReferenceGaitHandover::begin(const RagdollPose& measuredPose, float pelvisHeight,
        JPH::RVec3Arg centerOfMass, const ReferenceGaitProfile& profile)
    {
        reset();
        if (!validProfile(profile) || profile.mStandingHandoverDuration <= 0.0f || !std::isfinite(pelvisHeight)
            || !isFinite(centerOfMass) || measuredPose.mBodyTransforms.empty())
            return;
        for (const Transform& transform : measuredPose.mBodyTransforms)
            if (!isFinite(transform.mTranslation) || !isFinite(transform.mRotation))
                return;
        mSourcePose = measuredPose;
        mSourcePelvisHeight = pelvisHeight;
        mSourceCenterOfMass = centerOfMass;
        mTime = 0.0f;
    }

    void ReferenceGaitHandover::step(float duration, const ReferenceGaitProfile& profile)
    {
        if (!active() || !std::isfinite(duration) || duration <= 0.0f)
            return;
        mTime += duration;
        if (mTime >= profile.mStandingHandoverDuration)
            reset();
    }

    float ReferenceGaitHandover::factor(const ReferenceGaitProfile& profile) const
    {
        if (!active() || profile.mStandingHandoverDuration <= 0.0f)
            return 1.0f;
        const float t = std::clamp(mTime / profile.mStandingHandoverDuration, 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    bool ReferenceGaitHandover::blendPose(
        const RagdollPose& standing, const ReferenceGaitProfile& profile, RagdollPose& out) const
    {
        if (!active())
            return false;
        return interpolateRagdollPoses(*mSourcePose, standing, factor(profile), out);
    }

    float ReferenceGaitHandover::blendPelvisHeight(float standingTarget, const ReferenceGaitProfile& profile) const
    {
        if (!active())
            return standingTarget;
        return mSourcePelvisHeight + (standingTarget - mSourcePelvisHeight) * factor(profile);
    }

    std::optional<SupportMotionReference> ReferenceGaitHandover::supportReference(JPH::RVec3Arg supportCenter,
        JPH::Vec3Arg centerOfMassVelocity, const ReferenceGaitProfile& profile) const
    {
        if (!active() || !isFinite(supportCenter) || !isFinite(centerOfMassVelocity))
            return std::nullopt;
        const float weight = factor(profile);
        SupportMotionReference reference;
        reference.mPosition = mSourceCenterOfMass + (supportCenter - mSourceCenterOfMass) * weight;
        reference.mPosition.SetZ(supportCenter.GetZ());
        reference.mVelocity = horizontal(centerOfMassVelocity) * (1.0f - weight);
        return reference;
    }

    GaitControllerOutput toGaitControllerOutput(const ReferenceGaitState& state, const ReferenceGaitProfile& profile)
    {
        GaitControllerOutput output;
        output.mPhase = state.mPhase;
        output.mTimeInPhase = state.mTimeInPhase;
        const float period = profile.mDoubleSupportDuration + profile.mSwingDuration;
        output.mPhaseProgress = period > 0.0f ? std::clamp(state.mTimeInPhase / period, 0.0f, 1.0f) : 0.0f;
        output.mSwingContactCandidateTime = state.mSwingContactTime;
        output.mSwingTimeSinceLift = state.mSwingTimeSinceLift;
        output.mStanceReady = state.hasStance() && state.mStage == ReferenceGaitStage::Swing;
        output.mSwingFootLifted = state.mSwingFootLifted;
        output.mSwingTouchdownCandidate = output.mStanceReady && state.mSwingFootLifted
            && (state.mSwingContactTime > 0.0f || state.mTimeInStage >= 0.8f * profile.mSwingDuration);
        output.mTransitioned = state.mTransitioned;
        output.mPhaseTimedOut = state.mTimedOut;
        return output;
    }

    std::optional<ReferenceGaitPlacement> calculateReferenceGaitPlacement(
        const ReferenceGaitPlacementInput& input, const ReferenceGaitProfile& profile)
    {
        if (!validProfile(profile) || input.mPhase == GaitPhase::Idle || !isFinite(input.mCenterOfMass)
            || !isFinite(input.mCenterOfMassVelocity) || !isFinite(input.mRequestedVelocity)
            || !isFinite(input.mStanceFootPosition) || !isFinite(input.mCenterOfMassHeight)
            || (input.mSwingStartPosition && !isFinite(*input.mSwingStartPosition))
            || input.mCenterOfMassHeight <= 0.0f || !isFinite(input.mGravity) || input.mGravity <= 0.0f
            || !isFinite(input.mStancePeriod) || input.mStancePeriod <= 0.0f)
            return std::nullopt;
        const JPH::Vec3 request = horizontal(input.mRequestedVelocity);
        const auto facing = horizontal(input.mFacing);
        if (input.mStationaryCorrectiveStep)
        {
            if (!request.IsNearZero() || !isFinite(facing) || facing.IsNearZero()) return std::nullopt;
        }
        else if (request.IsNearZero())
            return std::nullopt;

        ReferenceGaitPlacement result;
        result.mForward = input.mStationaryCorrectiveStep ? facing.Normalized() : request.Normalized();
        result.mRight = result.mForward.Cross(JPH::Vec3::sAxisZ()).Normalized();
        // Fable i50 (body-frame-side-2026-09-11.txt), env-gated
        // OPENMW_RAGDOLL_BODY_SIDE, inert when unset. The anatomical side
        // rule below assumes travel-right = body-right. That holds for
        // forward walking only: a backpedal inverts it (every swing crosses
        // the legs, i36) and a strafe makes it perpendicular (the "width"
        // becomes a body fore-aft crossover; the front-side crossover
        // livelocks, i40). Measure the alignment c = bodyRight . travelRight
        // once and (a) choose the travel-frame side from the foot's real
        // anatomical side, (b) scale the travel-frame lateral offset and
        // minimum width by |c| so a sidestep no longer demands a fore-aft
        // crossover, and (c) forbid crossing in the BODY frame. At c = 1 all
        // three reduce to the existing computation exactly.
        static const auto bodySideMode_tuned = RagdollCore::tuned<int>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_BODY_SIDE");
            if (!text)
                return 0;
            const int parsed = std::atoi(text);
            return parsed > 0 ? parsed : 1;
        });
        static const auto& bodySideMode = bodySideMode_tuned.get();
        static const bool bodySideRule = bodySideMode > 0;
        JPH::Vec3 bodyRight = result.mRight;
        float bodyAlignment = 1.0f;
        const JPH::Vec3 bodyForward = input.mBodyForward ? horizontal(*input.mBodyForward) : facing;
        if (bodySideRule && isFinite(bodyForward) && !bodyForward.IsNearZero())
        {
            bodyRight = bodyForward.Normalized().Cross(JPH::Vec3::sAxisZ()).Normalized();
            bodyAlignment = std::clamp(bodyRight.Dot(result.mRight), -1.0f, 1.0f);
        }
        // Dead band: the pelvis heading wobbles ~14 deg rms in a forward
        // walk, so |c| in [0.866, 1] (within 30 deg) is treated as 1 and
        // [0, 0.5] (beyond 60 deg) as 0; a sidestep is the low end, a
        // forward walk the identity.
        const auto alignmentWeight = [](float c) {
            return std::clamp((std::abs(c) - 0.5f) / (0.8660254f - 0.5f), 0.0f, 1.0f);
        };
        const float alignmentScale = bodySideRule ? alignmentWeight(bodyAlignment) : 1.0f;
        result.mCaptureTimeConstant = std::sqrt(input.mCenterOfMassHeight / input.mGravity);
        result.mSteadyForwardFraction = std::tanh(input.mStancePeriod / (2.0f * result.mCaptureTimeConstant));
        // Tracked gait (NEW-ROUTE-PLAN-2026-09-11.txt), env-gated: the stride is
        // COMMITTED, not captured. The swing foot lands a fixed distance ahead
        // of the stance foot (scaled by the requested speed), on its own side
        // at a fixed width, plus SIMBICON-style velocity feedback:
        //   target = stance + fwd*(stride*scale + cvF*(vF - vReq)*tau)
        //                   + right*(side*width + cvL*vL*tau)
        // Nothing else in the placement law applies. Env unset: identity.
        static const auto trackedStride_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_TRACKED_STRIDE");
            if (!text)
                return 0.0f;
            const float parsed = std::strtof(text, nullptr);
            return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
        });
        static const auto& trackedStride = trackedStride_tuned.get();
        if (trackedStride > 0.0f && !input.mStationaryCorrectiveStep)
        {
            static const auto envFloat = [](const char* name, float fallback) {
                const char* text = RagdollCore::tuningText(name);
                if (!text)
                    return fallback;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) ? parsed : fallback;
            };
            static const auto width_tuned = RagdollCore::tuned<float>([] { return envFloat("OPENMW_RAGDOLL_TRACKED_WIDTH", 12.0f); });
            static const auto& width = width_tuned.get();
            static const auto cvForward_tuned = RagdollCore::tuned<float>([] { return envFloat("OPENMW_RAGDOLL_TRACKED_CVF", 0.0f); });
            static const auto& cvForward = cvForward_tuned.get();
            static const auto cvLateral_tuned = RagdollCore::tuned<float>([] { return envFloat("OPENMW_RAGDOLL_TRACKED_CVL", 1.0f); });
            static const auto& cvLateral = cvLateral_tuned.get();
            static const auto cdLateral_tuned = RagdollCore::tuned<float>([] { return envFloat("OPENMW_RAGDOLL_TRACKED_CDL", 0.0f); });
            static const auto& cdLateral = cdLateral_tuned.get();
            // Stride = step period x blended speed x gain: the foot lands where
            // the COM will be at the next touchdown (alpha weights the measured
            // speed against the request, so a start does not overreach).
            static const auto alpha_tuned = RagdollCore::tuned<float>([] { return envFloat("OPENMW_RAGDOLL_TRACKED_ALPHA", 0.7f); });
            static const auto& alpha = alpha_tuned.get();
            static const auto minimumStride_tuned = RagdollCore::tuned<float>([] { return envFloat("OPENMW_RAGDOLL_TRACKED_MIN", 8.0f); });
            static const auto& minimumStride = minimumStride_tuned.get();
            const float requestedSpeed = request.Length();
            const JPH::Vec3 velocity = horizontal(input.mCenterOfMassVelocity);
            const float vF = velocity.Dot(result.mForward), vL = velocity.Dot(result.mRight);
            const float tau = result.mCaptureTimeConstant;
            const JPH::Vec3 offset = horizontal(JPH::Vec3(input.mCenterOfMass - input.mStanceFootPosition));
            const float dL = offset.Dot(result.mRight);
            const float side = input.mPhase == GaitPhase::LeftStance ? 1.0f : -1.0f;
            const float blended = alpha * std::max(vF, 0.0f) + (1.0f - alpha) * requestedSpeed;
            // Raibert variant (OPENMW_RAGDOLL_TRACKED_RAIBERT = neutral-point
            // fraction, e.g. 0.5): the foot lands ahead of the COM predicted
            // at touchdown by v*T*k plus the speed-error lead, so the body
            // enters the stance behind the foot and brakes; a step-relative
            // stride (below) cannot get ahead of a fast body under the reach
            // clamp and runs away (energy trace, tracked-gait grids 1-4).
            static const auto raibert_tuned = RagdollCore::tuned<float>([] { return envFloat("OPENMW_RAGDOLL_TRACKED_RAIBERT", 0.0f); });
            static const auto& raibert = raibert_tuned.get();
            const float remaining = std::isfinite(input.mRemainingSwingTime) ? std::max(input.mRemainingSwingTime, 0.0f) : 0.0f;
            result.mLateralOffset = side * width + cvLateral * vL * tau + cdLateral * dL;
            result.mPlacementCenterOfMass = input.mCenterOfMass;
            result.mPlacementVelocity = velocity;
            JPH::RVec3 target;
            if (raibert > 0.0f)
            {
                const float ahead = std::max(vF, 0.0f) * remaining + raibert * input.mStancePeriod * blended
                    + cvForward * (vF - requestedSpeed) * tau;
                const float comForward = offset.Dot(result.mForward);
                result.mForwardOffset = std::max(minimumStride, comForward + ahead);
                target = input.mStanceFootPosition + result.mForward * result.mForwardOffset
                    + result.mRight * result.mLateralOffset;
            }
            else
            {
                result.mForwardOffset = std::max(minimumStride,
                    trackedStride * input.mStancePeriod * blended + cvForward * (vF - requestedSpeed) * tau);
                target = input.mStanceFootPosition + result.mForward * result.mForwardOffset
                    + result.mRight * result.mLateralOffset;
            }
            target.SetZ(input.mStanceFootPosition.GetZ());
            JPH::Vec3 reach = horizontal(JPH::Vec3(target - input.mStanceFootPosition));
            const float distance = reach.Length();
            if (distance > profile.mMaximumStepReach)
            {
                reach *= profile.mMaximumStepReach / distance;
                target = JPH::RVec3(input.mStanceFootPosition.GetX() + reach.GetX(),
                    input.mStanceFootPosition.GetY() + reach.GetY(), target.GetZ());
                result.mReachClamped = true;
            }
            if (!isFinite(target))
                return std::nullopt;
            result.mTarget = target;
            return result;
        }
        JPH::Vec3 velocity = horizontal(input.mCenterOfMassVelocity);
        JPH::RVec3 centerOfMass = input.mCenterOfMass;
        const float requestedSpeed = request.Length();
        if (input.mPredict)
        {
            // Integrate the linear inverted pendulum over the remaining swing
            // with the plan's single-support center-of-pressure policy: the
            // lateral center of pressure at the stance foot center, the
            // forward one at the point that realizes the velocity-error
            // request, clamped to the effective foot half-length.
            if (!isFinite(input.mRemainingSwingTime) || input.mRemainingSwingTime < 0.0f
                || !isFinite(input.mVelocityGain) || input.mVelocityGain < 0.0f
                || !isFinite(input.mForwardPressureHalfLength) || input.mForwardPressureHalfLength < 0.0f
                || !isFinite(input.mStanceFootCenter))
                return std::nullopt;
            const float tau = result.mCaptureTimeConstant;
            const JPH::Vec3 offset = horizontal(JPH::Vec3(centerOfMass - input.mStanceFootCenter));
            float xf = offset.Dot(result.mForward), xr = offset.Dot(result.mRight);
            float vf = velocity.Dot(result.mForward), vr = velocity.Dot(result.mRight);
            constexpr int steps = 8;
            const float dt = input.mRemainingSwingTime / steps;
            const float c = std::cosh(dt / tau), s = std::sinh(dt / tau);
            for (int i = 0; i < steps && dt > 0.0f; ++i)
            {
                const float acceleration = input.mVelocityGain * (requestedSpeed - vf);
                const float pressure = std::clamp(xf - acceleration * input.mCenterOfMassHeight / input.mGravity,
                    -input.mForwardPressureHalfLength, input.mForwardPressureHalfLength);
                const float nextXf = pressure + (xf - pressure) * c + vf * tau * s;
                const float nextVf = (xf - pressure) / tau * s + vf * c;
                const float nextXr = xr * c + vr * tau * s;
                const float nextVr = xr / tau * s + vr * c;
                xf = nextXf;
                vf = nextVf;
                xr = nextXr;
                vr = nextVr;
            }
            if (!std::isfinite(xf) || !std::isfinite(vf) || !std::isfinite(xr) || !std::isfinite(vr))
                return std::nullopt;
            centerOfMass = input.mStanceFootCenter + result.mForward * xf + result.mRight * xr;
            centerOfMass.SetZ(input.mCenterOfMass.GetZ());
            velocity = result.mForward * vf + result.mRight * vr;
        }
        result.mPlacementCenterOfMass = centerOfMass;
        result.mPlacementVelocity = velocity;
        const float forwardSpeed = velocity.Dot(result.mForward);
        const float lateralSpeed = velocity.Dot(result.mRight);
        const float errorGain = input.mForwardVelocityErrorGain.value_or(profile.mForwardVelocityErrorGain);
        // i39 (delivery-comp-2026-09-11.txt): anticipate systematic
        // over/under-delivery by adding the observed touchdown-velocity
        // bias to the predicted speed in the error term. Absent or zero
        // bias leaves the computation byte-identical.
        const float deliveryBias = input.mDeliveryVelocityBias.value_or(0.0f);
        result.mForwardOffset = std::max(profile.mMinimumForwardPlacement,
            result.mCaptureTimeConstant
                * (result.mSteadyForwardFraction * requestedSpeed
                    + errorGain * (forwardSpeed + deliveryBias - requestedSpeed)));
        // Anatomical stance side. A facing-vs-travel sign flip was added
        // here to stop a backpedal crossing its legs, and REVERTED: the
        // condition (facing.Dot(forward) < 0) fires on any facing more than
        // 90 degrees from travel, and the player's facing is rate limited, so
        // an ordinary forward walk could trip it mid-stride, cross the legs
        // and veer off. The historical screen did not cover the native
        // player's direct-travel and dynamic-facing combination. The harness
        // does have steering; see docs/ragdoll/offline-live-parity-review-2026-09-07.txt.
        // Anatomical sign of the SWING foot: LeftStance swings the right foot.
        const float anatomicalSide = input.mPhase == GaitPhase::LeftStance ? 1.0f : -1.0f;
        float side = anatomicalSide;
        if (bodySideRule && bodyAlignment < -0.5f)
            side = -anatomicalSide;
        // Foot separation stays in the TRAVEL frame. Moving it into the
        // body's frame was tried and measured worse: a 90 degree strafe went
        // from walking at 24 u/s without falling to falling at step 169, and
        // a backpedal fell sooner (139 against 229). With separation in the
        // body frame a sidestep puts the capture point and the step width on
        // the same axis, where they fight each other
        // (facing-travel-split-2026-09-07.txt).
        // Sway-reduction experiment (env-gated, inert by default;
        // sway-reduction-2026-09-10.txt): scale the lateral capture offset
        // to narrow the lateral shuttle. The same scale is applied at the
        // transfer-reference consumption point in the gait body. Unset uses
        // the profile value exactly (byte-identical). Not encoded/profiled.
        static const auto lateralCaptureScale_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_LAT_CAPTURE_SCALE");
            if (!text)
                return 0.0f;
            const float parsed = std::strtof(text, nullptr);
            return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
        });
        static const auto& lateralCaptureScale = lateralCaptureScale_tuned.get();
        const float lateralCaptureOffset = lateralCaptureScale > 0.0f
            ? lateralCaptureScale * profile.mLateralCaptureOffset : profile.mLateralCaptureOffset;
        // i37 (backpedal-side-guard-2026-09-11.txt): measured-side override,
        // now actually supplied by the production callsite. The anatomical
        // side assumes travel-right is body-right; when travel opposes
        // facing (backpedal) that inverts every swing side and the feet
        // cross (i36 trace: right foot at +8.1 commanded to x=-19.2).
        // Deriving the side from the measured swing-foot-to-stance
        // separation keeps each foot on the side it is already on,
        // independent of facing. The guard requires the separation to
        // exceed half the capture offset: near the travel-frame midline
        // (strafe, where separation is perpendicular to mRight, or a
        // near-crossing) noise would otherwise flip the side step to
        // step - the plausible cause of the "repeated stance reversals"
        // that rejected the unguarded 1e-4 version natively on 2026-09-08
        // (measured-placement-side-2026-09-08.txt). In forward walking the
        // measured side always agrees with the anatomical side, so this is
        // byte-identical there (P0 contract checked).
        if (input.mSwingStartPosition)
        {
            const float separation = JPH::Vec3(*input.mSwingStartPosition - input.mStanceFootPosition).Dot(result.mRight);
            if (std::abs(separation) > 0.5f * lateralCaptureOffset)
                side = separation > 0.0f ? 1.0f : -1.0f;
        }
        result.mLateralOffset
            = lateralSpeed * result.mCaptureTimeConstant + side * lateralCaptureOffset * alignmentScale;

        // Rotation-predict experiment (env-gated, inert by default;
        // rotation-predict-2026-09-11.txt, i33): the offset axes are computed
        // in the CURRENT travel frame, but the foot lands
        // mRemainingSwingTime later, when the frame has yawed by
        // omega * remaining. In a turn the world-frame target therefore moves
        // at omega x r (~30 u/s at the steering rate) and the ballistic
        // swing terminus converts the untracked motion into touchdown slide.
        // Rotating the offset axes to the predicted touchdown frame keeps the
        // world target approximately stationary through the swing. The LIPM
        // prediction above stays in the current frame (second-order
        // coupling, noted in the pre-registration). Unset env = byte-exact.
        static const auto rotPredict_tuned = RagdollCore::tuned<bool>([] {
            return RagdollCore::tuningText("OPENMW_RAGDOLL_ROT_PREDICT") != nullptr;
        });
        static const auto& rotPredict = rotPredict_tuned.get();
        JPH::Vec3 offsetForward = result.mForward;
        JPH::Vec3 offsetRight = result.mRight;
        if (rotPredict && std::isfinite(input.mTravelYawRate) && std::abs(input.mTravelYawRate) > 1e-6f
            && input.mRemainingSwingTime > 0.0f)
        {
            const float rotation = input.mTravelYawRate * input.mRemainingSwingTime;
            const float rc = std::cos(rotation), rs = std::sin(rotation);
            offsetForward = JPH::Vec3(
                result.mForward.GetX() * rc - result.mForward.GetY() * rs,
                result.mForward.GetX() * rs + result.mForward.GetY() * rc, 0.0f);
            offsetRight = offsetForward.Cross(JPH::Vec3::sAxisZ()).Normalized();
        }

        JPH::RVec3 target = centerOfMass + offsetForward * result.mForwardOffset
            + offsetRight * result.mLateralOffset;
        target.SetZ(input.mStanceFootPosition.GetZ());
        // Capture lead (i11 mechanism, i12 gating;
        // placement-lead-observer-2026-09-11.txt): inside a latched
        // disturbance window the caller passes k > 0 and the placement tracks
        // the capture point with an extended lateral time constant, landing
        // the foot beyond the capture point with braking margin proportional
        // to the disturbance. Zero keeps the nominal target exactly.
        if (input.mCaptureLead > 0.0f)
            target += result.mRight * (input.mCaptureLead * lateralSpeed);
        if (RagdollCore::tuningText("OPENMW_RAGDOLL_DEBUG_LEAD") && std::abs(lateralSpeed) > 10.0f)
            std::fprintf(stderr, "lead-check vlat=%.1f\n", lateralSpeed);
        const float width = side * JPH::Vec3(target - input.mStanceFootPosition).Dot(offsetRight);
        const float minimumWidth = profile.mMinimumStepWidth * alignmentScale;
        if (width < minimumWidth)
        {
            target += offsetRight * (side * (minimumWidth - width));
            result.mWidthClamped = true;
        }
        else if (width > profile.mMaximumStepWidth)
        {
            target -= offsetRight * (side * (width - profile.mMaximumStepWidth));
            result.mWidthClamped = true;
        }
        if (bodySideRule)
        {
            // No leg crossing in the body frame: the swing foot must land at
            // least the minimum step width on its own anatomical side of
            // the stance foot along body-right. Forward walking already
            // satisfies this through the travel-frame clamp (same axis), so
            // it only acts when travel and facing disagree.
            // The required body-frame separation is the complement of the
            // travel-frame one: zero when aligned (the travel clamp already
            // guarantees a positive body width there, so this never fires in
            // a forward walk), the full minimum in a sidestep.
            // Mode 2 skips the clamp (crossover gait allowed, only the
            // fore-aft demand is removed); mode 3 keeps a small closing gap.
            const float gapScale = bodySideMode == 2 ? 0.0f : bodySideMode == 3 ? 0.3f : 1.0f;
            const float bodyMinimum = gapScale * profile.mMinimumStepWidth * (1.0f - alignmentScale);
            const float bodyWidth
                = anatomicalSide * JPH::Vec3(target - input.mStanceFootPosition).Dot(bodyRight);
            if (bodyWidth < bodyMinimum)
            {
                target += bodyRight * (anatomicalSide * (bodyMinimum - bodyWidth));
                result.mWidthClamped = true;
            }
        }
        JPH::Vec3 offset = horizontal(JPH::Vec3(target - input.mStanceFootPosition));
        const float distance = offset.Length();
        if (distance > profile.mMaximumStepReach)
        {
            offset *= profile.mMaximumStepReach / distance;
            target = JPH::RVec3(input.mStanceFootPosition.GetX() + offset.GetX(),
                input.mStanceFootPosition.GetY() + offset.GetY(), target.GetZ());
            result.mReachClamped = true;
        }
        if (!isFinite(target))
            return std::nullopt;
        result.mTarget = target;
        if (bodySideRule && RagdollCore::tuningText("OPENMW_RAGDOLL_DEBUG_BODY_SIDE"))
        {
            const JPH::Vec3 bodyForward = JPH::Vec3::sAxisZ().Cross(bodyRight);
            const JPH::Vec3 rel = horizontal(JPH::Vec3(target - input.mStanceFootPosition));
            const JPH::Vec3 com = horizontal(JPH::Vec3(centerOfMass - input.mStanceFootPosition));
            std::fprintf(stderr, "bodyside phase=%d c=%.2f anat=%+.0f side=%+.0f tgt_bodyR=%.1f tgt_bodyF=%.1f com_bodyR=%.1f com_bodyF=%.1f v_travelF=%.1f v_travelR=%.1f fwdOff=%.1f latOff=%.1f clampW=%d\n",
                static_cast<int>(input.mPhase), bodyAlignment, anatomicalSide, side, rel.Dot(bodyRight), rel.Dot(bodyForward),
                com.Dot(bodyRight), com.Dot(bodyForward), forwardSpeed, lateralSpeed, result.mForwardOffset, result.mLateralOffset,
                result.mWidthClamped ? 1 : 0);
        }
        return result;
    }

    std::optional<JPH::Vec3> calculateReferenceSwingTrajectory(
        const ReferenceSwingTrajectoryInput& input, const ReferenceGaitProfile& profile)
    {
        if (!validProfile(profile) || !isFinite(input.mLiftOffPosition) || !isFinite(input.mPlacementPosition)
            || !isFinite(input.mSwingTime) || input.mSwingTime < 0.0f)
            return std::nullopt;
        const float progress = std::clamp(input.mSwingTime / profile.mSwingDuration, 0.0f, 1.0f);
        const float riseEnd = profile.mSwingRiseEndFraction;
        // Swing travel-end experiment (env-gated, inert by default;
        // touchdown-scuff-2026-09-10.txt, iteration 7): extend the horizontal
        // travel window so the smoothstep's zero-velocity end coincides with
        // the actual touchdown moment (~93% of swing) instead of leaving the
        // foot ballistic for the last ~10%. Unset uses the profile value
        // exactly (byte-identical). Not encoded, not profiled.
        static const auto travelEndOverride_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SWING_TRAVEL_END");
            if (!text)
                return 0.0f;
            const float parsed = std::strtof(text, nullptr);
            return std::isfinite(parsed) && parsed > 0.0f && parsed <= 1.0f ? parsed : 0.0f;
        });
        static const auto& travelEndOverride = travelEndOverride_tuned.get();
        const float travelEnd = travelEndOverride > 0.0f ? travelEndOverride : profile.mSwingTravelEndFraction;
        JPH::Vec3 position = input.mLiftOffPosition;
        float clearance = 0.0f;
        if (input.mHoldLifted)
            clearance = profile.mSwingFootLift * smoothStep(progress / riseEnd);
        else
        {
            const float travelStart = profile.mSwingTravelStartFraction;
            const float travel = smoothStep((progress - travelStart) / (travelEnd - travelStart));
            position += (input.mPlacementPosition - input.mLiftOffPosition) * travel;
            // i45 intercept lead (env-gated, inert by default;
            // swing-intercept-2026-09-12.txt): when the placement target
            // moves (rotating travel frame), offset the trajectory by
            // v*(t - T*travel). The offset vanishes at touchdown while its
            // derivative adds v to the commanded arrival velocity, so the
            // foot arrives moving WITH the target instead of relative to
            // it. The placement itself is unchanged (i33 lesson).
            if (!input.mPlacementVelocity.IsNearZero())
            {
                JPH::Vec3 offset = input.mPlacementVelocity
                    * (input.mSwingTime - profile.mSwingDuration * travel);
                const float lead = offset.Length();
                if (lead > 5.0f)
                    offset *= 5.0f / lead;
                position += offset;
            }
            const float descentEnd = profile.mSwingDescentEndFraction;
            if (progress < riseEnd)
                clearance = profile.mSwingFootLift * smoothStep(progress / riseEnd);
            else if (progress < descentEnd)
                clearance = profile.mSwingFootLift * (1.0f - smoothStep((progress - riseEnd) / (descentEnd - riseEnd)));
            // From the press start the leg keeps pressing toward the floor so
            // a slightly short reach or a late landing still produces contact.
            const float pressStart = profile.mTouchdownPressStartFraction * profile.mSwingDuration;
            if (input.mSwingTime > pressStart)
                clearance -= std::min(profile.mMaximumTouchdownPressDepth,
                    profile.mTouchdownPressSpeed * (input.mSwingTime - pressStart));
        }
        position.SetZ(position.GetZ() + clearance);
        return isFinite(position) ? std::optional<JPH::Vec3>(position) : std::nullopt;
    }

    std::optional<ReferenceRootRouting> routeReferenceRootTorque(const ReferenceRootRoutingInput& input)
    {
        if (!isFinite(input.mPelvisTorqueRequest) || !isFinite(input.mLeftHipLoadFraction)
            || input.mLeftHipLoadFraction < 0.0f || !isFinite(input.mRightHipLoadFraction)
            || input.mRightHipLoadFraction < 0.0f)
            return std::nullopt;
        ReferenceRootRouting result;
        for (const JPH::Vec3& torque : input.mBranchDynamicTorques)
        {
            if (!isFinite(torque))
                return std::nullopt;
            result.mBranchReaction += torque;
        }
        const double load = static_cast<double>(input.mLeftHipLoadFraction) + input.mRightHipLoadFraction;
        if (load <= 0.0)
            return result;
        const JPH::Vec3 total = -(input.mPelvisTorqueRequest + result.mBranchReaction);
        result.mLeftHipTorque = total * static_cast<float>(input.mLeftHipLoadFraction / load);
        result.mRightHipTorque = total * static_cast<float>(input.mRightHipLoadFraction / load);
        return result;
    }

    std::optional<RagdollPose> projectReferenceLegPosture(std::span<const Transform> pose,
        std::size_t pelvis, const ReferenceGaitLeg& left, const ReferenceGaitLeg& right)
    {
        std::array indices{ pelvis, left.mHip, left.mKnee, left.mFoot, right.mHip, right.mKnee, right.mFoot };
        std::sort(indices.begin(), indices.end());
        if (indices.back() >= pose.size() || std::adjacent_find(indices.begin(), indices.end()) != indices.end())
            return std::nullopt;
        for (const auto& transform : pose)
            if (!isFinite(transform.mTranslation) || !isFinite(transform.mRotation) || !transform.mRotation.IsNormalized())
                return std::nullopt;
        RagdollPose result;
        result.mBodyTransforms.assign(pose.begin(), pose.end());
        for (const auto& leg : { left, right })
        {
            if (!leg.mHingeGeometry || !leg.mHipGeometry || !leg.mAnkleGeometry) return std::nullopt;
            const auto& geometry = *leg.mHingeGeometry;
            const float pi = std::numbers::pi_v<float>;
            if (!isFinite(geometry.mUpperOffset) || !isFinite(geometry.mLowerOffset)
                || geometry.mUpperOffset.IsNearZero() || geometry.mLowerOffset.IsNearZero()
                || !isFinite(geometry.mLowerRotation) || !geometry.mLowerRotation.IsNormalized()
                || !isFinite(geometry.mHingeAxis) || !geometry.mHingeAxis.IsNormalized()
                || !isFinite(geometry.mMinimumAngle) || !isFinite(geometry.mMaximumAngle)
                || geometry.mMinimumAngle > geometry.mMaximumAngle
                || geometry.mMaximumAngle - geometry.mMinimumAngle > 2 * pi
                || geometry.mMinimumAngle < -2 * pi || geometry.mMaximumAngle > 2 * pi)
                return std::nullopt;
            const auto hip = clampSwingTwistJointOrientation(*leg.mHipGeometry,
                pose[pelvis].mRotation, pose[leg.mHip].mRotation);
            if (!hip) return std::nullopt;
            const auto desiredKnee = (pose[leg.mHip].mRotation.Conjugated() * pose[leg.mKnee].mRotation).Normalized();
            const auto flexion = (desiredKnee * geometry.mLowerRotation.Conjugated()).Normalized();
            float unconstrained = 2 * std::atan2(flexion.GetXYZ().Dot(geometry.mHingeAxis), flexion.GetW());
            unconstrained = std::remainder(unconstrained, 2 * pi);
            float angle = geometry.mMinimumAngle, best = -1;
            const auto consider = [&](float candidate) {
                candidate = std::clamp(candidate, geometry.mMinimumAngle, geometry.mMaximumAngle);
                const auto rotation = JPH::Quat::sRotation(geometry.mHingeAxis, candidate) * geometry.mLowerRotation;
                const float alignment = std::abs(rotation.Dot(desiredKnee));
                if (alignment > best) { angle = candidate; best = alignment; }
            };
            consider(geometry.mMinimumAngle);
            consider(geometry.mMaximumAngle);
            for (const int turn : { -1, 0, 1 }) consider(unconstrained + turn * 2 * pi);
            const auto knee = (*hip * JPH::Quat::sRotation(geometry.mHingeAxis, angle)
                * geometry.mLowerRotation).Normalized();
            const auto desiredAnkle = (pose[leg.mKnee].mRotation.Conjugated() * pose[leg.mFoot].mRotation).Normalized();
            const auto foot = clampSwingTwistJointOrientation(*leg.mAnkleGeometry, knee,
                (knee * desiredAnkle).Normalized());
            if (!foot) return std::nullopt;
            auto& output = result.mBodyTransforms;
            output[leg.mHip].mRotation = *hip;
            output[leg.mKnee] = { pose[leg.mHip].mTranslation + *hip * geometry.mUpperOffset, knee };
            output[leg.mFoot] = { output[leg.mKnee].mTranslation
                + knee * geometry.mLowerRotation.Conjugated() * geometry.mLowerOffset, *foot };
        }
        for (const auto& transform : result.mBodyTransforms)
            if (!isFinite(transform.mTranslation) || !isFinite(transform.mRotation) || !transform.mRotation.IsNormalized())
                return std::nullopt;
        return result;
    }

    std::optional<ReferenceStancePosture> planReferenceStancePosture(std::span<const Transform> basePose,
        std::size_t pelvis, const ReferenceGaitLeg& left, const ReferenceGaitLeg& right,
        const Transform& desiredPelvis, JPH::Vec3Arg leftFoot, JPH::Vec3Arg rightFoot,
        JPH::Vec3Arg preferredBendDirection)
    {
        std::array indices{ pelvis, left.mHip, left.mKnee, left.mFoot, right.mHip, right.mKnee, right.mFoot };
        std::sort(indices.begin(), indices.end());
        if (indices.back() >= basePose.size() || std::adjacent_find(indices.begin(), indices.end()) != indices.end()
            || !left.mHingeGeometry || !right.mHingeGeometry || !isFinite(desiredPelvis.mTranslation)
            || !isFinite(desiredPelvis.mRotation) || !desiredPelvis.mRotation.IsNormalized()
            || !isFinite(leftFoot) || !isFinite(rightFoot) || !isFinite(preferredBendDirection))
            return std::nullopt;
        for (const auto& transform : basePose)
            if (!isFinite(transform.mTranslation) || !isFinite(transform.mRotation) || !transform.mRotation.IsNormalized())
                return std::nullopt;

        const auto delta = (desiredPelvis.mRotation * basePose[pelvis].mRotation.Conjugated()).Normalized();
        float minimumHeight = -std::numeric_limits<float>::infinity();
        float maximumHeight = std::numeric_limits<float>::infinity();
        const std::array legs{ left, right };
        const std::array feet{ leftFoot, rightFoot };
        for (std::size_t i = 0; i < legs.size(); ++i)
        {
            const auto& geometry = *legs[i].mHingeGeometry;
            const auto& upper = geometry.mUpperOffset;
            const auto& lower = geometry.mLowerOffset;
            const auto& axis = geometry.mHingeAxis;
            const float minimumAngle = geometry.mMinimumAngle, maximumAngle = geometry.mMaximumAngle;
            if (!isFinite(upper) || !isFinite(lower) || upper.LengthSq() <= 1e-8f || lower.LengthSq() <= 1e-8f
                || !isFinite(axis) || !axis.IsNormalized() || !isFinite(minimumAngle) || !isFinite(maximumAngle)
                || minimumAngle > maximumAngle || maximumAngle - minimumAngle > 2.0f * std::numbers::pi_v<float>
                || minimumAngle < -2.0f * std::numbers::pi_v<float>
                || maximumAngle > 2.0f * std::numbers::pi_v<float>)
                return std::nullopt;
            const auto upperPerpendicular = upper - axis * upper.Dot(axis);
            const auto lowerPerpendicular = lower - axis * lower.Dot(axis);
            const float amplitude = 2.0f * upperPerpendicular.Length() * lowerPerpendicular.Length();
            const float base = upper.LengthSq() + lower.LengthSq() + 2.0f * upper.Dot(axis) * lower.Dot(axis);
            const float reference = std::atan2(axis.Dot(upperPerpendicular.Cross(lowerPerpendicular)),
                upperPerpendicular.Dot(lowerPerpendicular));
            // Squared reach is base + amplitude*cos(reference + kneeAngle).
            // Its extrema occur at joint limits or integer multiples of pi.
            const auto radiusSquared = [&](float angle) { return base + amplitude * std::cos(reference + angle); };
            float minimumRadiusSquared = std::min(radiusSquared(minimumAngle), radiusSquared(maximumAngle));
            float maximumRadiusSquared = std::max(radiusSquared(minimumAngle), radiusSquared(maximumAngle));
            for (int multiple = -3; multiple <= 3; ++multiple)
            {
                const float angle = multiple * std::numbers::pi_v<float> - reference;
                if (angle >= minimumAngle && angle <= maximumAngle)
                {
                    minimumRadiusSquared = std::min(minimumRadiusSquared, radiusSquared(angle));
                    maximumRadiusSquared = std::max(maximumRadiusSquared, radiusSquared(angle));
                }
            }
            const auto hipOffset = delta * (basePose[legs[i].mHip].mTranslation - basePose[pelvis].mTranslation);
            auto horizontalOffset = desiredPelvis.mTranslation + hipOffset - feet[i];
            horizontalOffset.SetZ(0.0f);
            const float horizontalSquared = horizontalOffset.LengthSq();
            if (horizontalSquared > maximumRadiusSquared)
                return std::nullopt;
            const float origin = feet[i].GetZ() - hipOffset.GetZ();
            minimumHeight = std::max(minimumHeight,
                origin + std::sqrt(std::max(0.0f, minimumRadiusSquared - horizontalSquared)));
            maximumHeight = std::min(maximumHeight,
                origin + std::sqrt(std::max(0.0f, maximumRadiusSquared - horizontalSquared)));
        }
        if (!isFinite(minimumHeight) || !isFinite(maximumHeight) || minimumHeight > maximumHeight)
            return std::nullopt;

        ReferenceStancePosture result;
        result.mMinimumPelvisHeight = minimumHeight;
        result.mMaximumPelvisHeight = maximumHeight;
        auto position = desiredPelvis.mTranslation;
        position.SetZ(std::clamp(position.GetZ(), minimumHeight, maximumHeight));
        auto& pose = result.mPose.mBodyTransforms;
        pose.assign(basePose.begin(), basePose.end());
        for (auto& transform : pose)
        {
            transform.mTranslation = position + delta * (transform.mTranslation - basePose[pelvis].mTranslation);
            transform.mRotation = (delta * transform.mRotation).Normalized();
        }
        for (std::size_t i = 0; i < legs.size(); ++i)
        {
            const auto& leg = legs[i];
            TwoBoneLegTargetInput target{ leg.mHip, leg.mKnee, leg.mFoot, feet[i], preferredBendDirection,
                leg.mHingeGeometry };
            if (!applyTwoBoneLegTarget(pose, target)
                || (pose[leg.mFoot].mTranslation - feet[i]).Length() > 1e-3f)
                return std::nullopt;
        }
        return result;
    }

    std::optional<ReferenceStancePosture> planJointLimitedReferenceStancePosture(std::span<const Transform> basePose,
        std::size_t pelvis, const ReferenceGaitLeg& left, const ReferenceGaitLeg& right,
        const Transform& desiredPelvis, JPH::Vec3Arg leftFoot, JPH::Vec3Arg rightFoot,
        const std::optional<JPH::Quat>& leftFootRotation, const std::optional<JPH::Quat>& rightFootRotation,
        JPH::Vec3Arg preferredBendDirection)
    {
        struct QueryAudit
        {
            bool enabled = RagdollCore::tuningText("OPENMW_RAGDOLL_COORDINATED_QUERY") != nullptr;
            bool radial = false;
            unsigned heights = 0, fixedFootRejected = 0, hipRejected = 0, ankleRejected = 0;
            ~QueryAudit()
            {
                if (enabled) std::fprintf(stderr,
                    "POSTURE_QUERY radial=%d heights=%u fixed_foot_rejected=%u hip_rejected=%u ankle_rejected=%u\n",
                    radial, heights, fixedFootRejected, hipRejected, ankleRejected);
            }
        } audit;
        if (!left.mHipGeometry || !left.mAnkleGeometry || !right.mHipGeometry || !right.mAnkleGeometry
            || (leftFootRotation && (!isFinite(*leftFootRotation) || !leftFootRotation->IsNormalized()))
            || (rightFootRotation && (!isFinite(*rightFootRotation) || !rightFootRotation->IsNormalized())))
            return std::nullopt;
        const auto radial = planReferenceStancePosture(basePose, pelvis, left, right, desiredPelvis,
            leftFoot, rightFoot, preferredBendDirection);
        if (!radial)
            return std::nullopt;
        audit.radial = true;
        const auto atHeight = [&](float height) -> std::optional<ReferenceStancePosture> {
            ++audit.heights;
            auto requested = desiredPelvis;
            requested.mTranslation.SetZ(height);
            auto result = planReferenceStancePosture(basePose, pelvis, left, right, requested,
                leftFoot, rightFoot, preferredBendDirection);
            if (!result)
                return std::nullopt;
            auto& pose = result->mPose.mBodyTransforms;
            const std::array<const ReferenceGaitLeg*, 2> legs{ &left, &right };
            const std::array<JPH::Vec3, 2> feet{ leftFoot, rightFoot };
            const std::array<std::optional<JPH::Quat>, 2> footRotations{ leftFootRotation, rightFootRotation };
            const auto unchanged = [](const SwingTwistJointGeometry& geometry, JPH::QuatArg parent, JPH::QuatArg child) {
                const auto clamped = clampSwingTwistJointOrientation(geometry, parent, child);
                return clamped && (clamped->IsClose(child, 1e-10f) || clamped->IsClose(-child, 1e-10f));
            };
            for (std::size_t i = 0; i < legs.size(); ++i)
            {
                const auto& leg = *legs[i];
                const Transform radialHip = pose[leg.mHip], radialKnee = pose[leg.mKnee];
                const JPH::Quat radialFoot = pose[leg.mFoot].mRotation;
                const auto fullSwivel = [&] {
                    const auto line = feet[i] - radialHip.mTranslation;
                    if (line.IsNearZero()) return false;
                    const auto axis = line.Normalized();
                    const auto evaluate = [&](float angle) {
                        const auto rotation = JPH::Quat::sRotation(axis, angle);
                        pose[leg.mHip].mRotation = (rotation * radialHip.mRotation).Normalized();
                        pose[leg.mKnee].mRotation = (rotation * radialKnee.mRotation).Normalized();
                        pose[leg.mKnee].mTranslation = radialHip.mTranslation
                            + rotation * (radialKnee.mTranslation - radialHip.mTranslation);
                        const auto& geometry = *leg.mHingeGeometry;
                        pose[leg.mFoot].mTranslation = pose[leg.mKnee].mTranslation
                            + pose[leg.mKnee].mRotation * geometry.mLowerRotation.Conjugated() * geometry.mLowerOffset;
                        if (!unchanged(*leg.mHipGeometry, pose[pelvis].mRotation, pose[leg.mHip].mRotation))
                            return false;
                        if (footRotations[i]) pose[leg.mFoot].mRotation = *footRotations[i];
                        else
                        {
                            const auto ankle = clampSwingTwistJointOrientation(*leg.mAnkleGeometry,
                                pose[leg.mKnee].mRotation, radialFoot);
                            if (!ankle) return false;
                            pose[leg.mFoot].mRotation = *ankle;
                        }
                        return unchanged(*leg.mAnkleGeometry, pose[leg.mKnee].mRotation, pose[leg.mFoot].mRotation)
                            && (pose[leg.mFoot].mTranslation - feet[i]).Length() <= 1e-3f;
                    };
                    // Keep the first bracketed feasible endpoint; do not assume
                    // the full swivel circle is a connected feasible interval.
                    for (unsigned sample = 1; sample <= 64; ++sample)
                        for (const float sign : { 1.0f, -1.0f })
                        {
                            float feasible = sign * std::numbers::pi_v<float> * sample / 64;
                            if (!evaluate(feasible)) continue;
                            float rejected = sign * std::numbers::pi_v<float> * (sample - 1) / 64;
                            for (unsigned iteration = 0; iteration < 18; ++iteration)
                            {
                                const float middle = 0.5f * (feasible + rejected);
                                if (evaluate(middle)) feasible = middle; else rejected = middle;
                            }
                            return evaluate(feasible);
                        }
                    return false;
                };
                if (footRotations[i])
                {
                    pose[leg.mFoot].mRotation = *footRotations[i];
                    const auto valid = [&] {
                        return unchanged(*leg.mHipGeometry, pose[pelvis].mRotation, pose[leg.mHip].mRotation)
                            && unchanged(*leg.mAnkleGeometry, pose[leg.mKnee].mRotation, pose[leg.mFoot].mRotation);
                    };
                    if (valid())
                        continue;
                    // Rotating the knee plane around the hip-to-foot line
                    // preserves both endpoints, both lengths and knee flexion.
                    // Search toward the planted foot's reference plane so
                    // neither hip nor ankle must take the whole turn alone.
                    const Transform originalHip = pose[leg.mHip], originalKnee = pose[leg.mKnee];
                    const auto axis = (pose[leg.mFoot].mTranslation - originalHip.mTranslation).Normalized();
                    const auto bend = (*footRotations[i] * basePose[leg.mFoot].mRotation.Conjugated())
                        * preferredBendDirection;
                    if (!applyTwoBoneLegTarget(pose,
                            { leg.mHip, leg.mKnee, leg.mFoot, feet[i], bend, leg.mHingeGeometry })
                        || (pose[leg.mFoot].mTranslation - feet[i]).Length() > 1e-3f)
                        return std::nullopt;
                    auto turn = (pose[leg.mHip].mRotation * originalHip.mRotation.Conjugated()).Normalized();
                    if (turn.GetW() < 0) turn = -turn;
                    const float angle = 2.0f * std::atan2(turn.GetXYZ().Dot(axis), turn.GetW());
                    const auto rotate = [&](float fraction) {
                        const auto rotation = JPH::Quat::sRotation(axis, angle * fraction);
                        pose[leg.mHip].mRotation = (rotation * originalHip.mRotation).Normalized();
                        pose[leg.mKnee].mRotation = (rotation * originalKnee.mRotation).Normalized();
                        pose[leg.mKnee].mTranslation = originalHip.mTranslation
                            + rotation * (originalKnee.mTranslation - originalHip.mTranslation);
                        const auto& geometry = *leg.mHingeGeometry;
                        pose[leg.mFoot].mTranslation = pose[leg.mKnee].mTranslation
                            + pose[leg.mKnee].mRotation * geometry.mLowerRotation.Conjugated() * geometry.mLowerOffset;
                    };
                    bool found = false;
                    for (unsigned sample = 1; sample <= 32; ++sample)
                    {
                        float feasible = static_cast<float>(sample) / 32;
                        rotate(feasible);
                        if (!valid()) continue;
                        float rejected = static_cast<float>(sample - 1) / 32;
                        for (unsigned iteration = 0; iteration < 18; ++iteration)
                        {
                            const float middle = 0.5f * (feasible + rejected);
                            rotate(middle);
                            if (valid()) feasible = middle;
                            else rejected = middle;
                        }
                        rotate(feasible);
                        found = valid() && (pose[leg.mFoot].mTranslation - feet[i]).Length() <= 1e-3f;
                        break;
                    }
                    if (!found && !fullSwivel())
                    {
                        ++audit.fixedFootRejected;
                        return std::nullopt;
                    }
                }
                else
                {
                    if (!unchanged(*leg.mHipGeometry, pose[pelvis].mRotation, pose[leg.mHip].mRotation))
                    {
                        if (!fullSwivel()) { ++audit.hipRejected; return std::nullopt; }
                        continue;
                    }
                    const auto orientation = clampSwingTwistJointOrientation(*leg.mAnkleGeometry,
                        pose[leg.mKnee].mRotation, pose[leg.mFoot].mRotation);
                    if (!orientation)
                        return std::nullopt;
                    pose[leg.mFoot].mRotation = *orientation;
                }
            }
            for (const auto& leg : { left, right })
            {
                if (!unchanged(*leg.mHipGeometry, pose[pelvis].mRotation, pose[leg.mHip].mRotation))
                {
                    ++audit.hipRejected;
                    return std::nullopt;
                }
                if (!unchanged(*leg.mAnkleGeometry, pose[leg.mKnee].mRotation, pose[leg.mFoot].mRotation))
                {
                    ++audit.ankleRejected;
                    return std::nullopt;
                }
            }
            return result;
        };
        const float start = radial->mPose.mBodyTransforms[pelvis].mTranslation.GetZ();
        if (auto result = atHeight(start))
            return result;
        std::optional<ReferenceStancePosture> best;
        float bestDistance = std::numeric_limits<float>::max();
        // This bounds geometry work, not controller tuning. Always retain a
        // verified feasible endpoint while refining; narrow disconnected
        // feasible intervals can be missed and must never become false passes.
        for (const float end : { radial->mMinimumPelvisHeight, radial->mMaximumPelvisHeight })
        {
            float previous = start;
            for (unsigned sample = 1; sample <= 64; ++sample)
            {
                const float height = start + (end - start) * (static_cast<float>(sample) / 64);
                auto result = atHeight(height);
                if (!result)
                {
                    previous = height;
                    continue;
                }
                float feasible = height, rejected = previous;
                for (unsigned iteration = 0; iteration < 18; ++iteration)
                {
                    const float middle = 0.5f * (feasible + rejected);
                    if (auto refined = atHeight(middle))
                    {
                        feasible = middle;
                        result = std::move(refined);
                    }
                    else
                        rejected = middle;
                }
                const float distance = std::abs(feasible - desiredPelvis.mTranslation.GetZ());
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    best = std::move(result);
                }
                break;
            }
        }
        return best;
    }

    std::optional<ReferenceStancePosture> planBalancedReferenceStancePosture(
        std::span<const Transform> basePose, std::span<const ReferencePostureMass> masses,
        std::size_t pelvis, const ReferenceGaitLeg& left, const ReferenceGaitLeg& right,
        const Transform& desiredPelvis, const Transform& leftFoot, const Transform& rightFoot,
        JPH::Vec3Arg desiredCenterOfMass, JPH::Vec3Arg preferredBendDirection, bool allowHeightLowering)
    {
        if (masses.size() != basePose.size() || masses.empty() || !isFinite(desiredCenterOfMass)
            || !isFinite(leftFoot.mRotation) || !leftFoot.mRotation.IsNormalized()
            || !isFinite(rightFoot.mRotation) || !rightFoot.mRotation.IsNormalized())
            return std::nullopt;
        float totalMass = 0;
        for (const auto& mass : masses)
        {
            if (!isFinite(mass.mMass) || mass.mMass <= 0 || !isFinite(mass.mLocalCenterOfMass))
                return std::nullopt;
            totalMass += mass.mMass;
        }
        if (!isFinite(totalMass)) return std::nullopt;
        const auto center = [&](std::span<const Transform> pose) {
            JPH::Vec3 weighted = JPH::Vec3::sZero();
            for (std::size_t body = 0; body < pose.size(); ++body)
                weighted += masses[body].mMass * (pose[body].mTranslation
                    + pose[body].mRotation * masses[body].mLocalCenterOfMass);
            return weighted / totalMass;
        };
        auto requested = desiredPelvis;
        for (unsigned iteration = 0; iteration < 32; ++iteration)
        {
            auto pose = planReferenceStancePosture(basePose, pelvis, left, right, requested,
                leftFoot.mTranslation, rightFoot.mTranslation, preferredBendDirection);
            if (!pose || (!allowHeightLowering && std::abs(pose->mPose.mBodyTransforms[pelvis].mTranslation.GetZ()
                    - desiredPelvis.mTranslation.GetZ()) > 1e-3f))
                return std::nullopt;
            if (allowHeightLowering)
                requested.mTranslation.SetZ(std::min(requested.mTranslation.GetZ(),
                    pose->mPose.mBodyTransforms[pelvis].mTranslation.GetZ()));
            auto& transforms = pose->mPose.mBodyTransforms;
            transforms[left.mFoot].mRotation = leftFoot.mRotation;
            transforms[right.mFoot].mRotation = rightFoot.mRotation;
            auto difference = horizontal(desiredCenterOfMass - center(transforms));
            if (!isFinite(difference)) return std::nullopt;
            if (difference.Length() > 1e-4f)
            {
                requested.mTranslation += difference;
                continue;
            }
            auto limited = planJointLimitedReferenceStancePosture(basePose, pelvis, left, right, requested,
                leftFoot.mTranslation, rightFoot.mTranslation, leftFoot.mRotation, rightFoot.mRotation,
                preferredBendDirection);
            if (!limited || (!allowHeightLowering && std::abs(limited->mPose.mBodyTransforms[pelvis].mTranslation.GetZ()
                    - desiredPelvis.mTranslation.GetZ()) > 1e-3f))
                return std::nullopt;
            if (allowHeightLowering)
            {
                const float height = limited->mPose.mBodyTransforms[pelvis].mTranslation.GetZ();
                if (height > requested.mTranslation.GetZ() + 1e-3f)
                    return std::nullopt;
                requested.mTranslation.SetZ(height);
            }
            difference = horizontal(desiredCenterOfMass - center(limited->mPose.mBodyTransforms));
            if (!isFinite(difference)) return std::nullopt;
            if (difference.Length() <= 1e-4f) return limited;
            requested.mTranslation += difference;
        }
        return std::nullopt;
    }

    std::optional<ReferenceStancePosture> planBalancedReferenceSwingPosture(
        std::span<const Transform> basePose, std::span<const ReferencePostureMass> masses,
        std::size_t pelvis, const ReferenceGaitLeg& left, const ReferenceGaitLeg& right,
        const Transform& desiredPelvis, const Transform& leftFoot, const Transform& rightFoot,
        std::size_t swingFoot, JPH::Vec3Arg desiredCenterOfMass, JPH::Vec3Arg preferredBendDirection)
    {
        if (masses.size() != basePose.size() || masses.empty() || !isFinite(desiredCenterOfMass)
            || (swingFoot != left.mFoot && swingFoot != right.mFoot) || left.mFoot == right.mFoot)
            return std::nullopt;
        const auto leftRotation = swingFoot == left.mFoot ? std::nullopt : std::optional(leftFoot.mRotation);
        const auto rightRotation = swingFoot == right.mFoot ? std::nullopt : std::optional(rightFoot.mRotation);
        if ((leftRotation && (!isFinite(*leftRotation) || !leftRotation->IsNormalized()))
            || (rightRotation && (!isFinite(*rightRotation) || !rightRotation->IsNormalized())))
            return std::nullopt;
        const auto& swing = swingFoot == left.mFoot ? left : right;
        if (!swing.mAnkleGeometry) return std::nullopt;
        float totalMass = 0;
        for (const auto& mass : masses)
        {
            if (!isFinite(mass.mMass) || mass.mMass <= 0 || !isFinite(mass.mLocalCenterOfMass))
                return std::nullopt;
            totalMass += mass.mMass;
        }
        if (!isFinite(totalMass)) return std::nullopt;
        const auto center = [&](std::span<const Transform> pose) {
            JPH::Vec3 weighted = JPH::Vec3::sZero();
            for (std::size_t body = 0; body < pose.size(); ++body)
                weighted += masses[body].mMass * (pose[body].mTranslation
                    + pose[body].mRotation * masses[body].mLocalCenterOfMass);
            return weighted / totalMass;
        };
        auto requested = desiredPelvis;
        for (unsigned iteration = 0; iteration < 32; ++iteration)
        {
            auto pose = planReferenceStancePosture(basePose, pelvis, left, right, requested,
                leftFoot.mTranslation, rightFoot.mTranslation, preferredBendDirection);
            if (!pose) return std::nullopt;
            auto& transforms = pose->mPose.mBodyTransforms;
            if (leftRotation) transforms[left.mFoot].mRotation = *leftRotation;
            if (rightRotation) transforms[right.mFoot].mRotation = *rightRotation;
            const auto orientation = clampSwingTwistJointOrientation(*swing.mAnkleGeometry,
                transforms[swing.mKnee].mRotation, basePose[swing.mFoot].mRotation);
            if (!orientation) return std::nullopt;
            transforms[swing.mFoot].mRotation = *orientation;
            auto difference = horizontal(desiredCenterOfMass - center(transforms));
            if (!isFinite(difference)) return std::nullopt;
            if (difference.Length() >= 1e-3f)
            {
                requested.mTranslation += difference;
                continue;
            }
            // Verify this coordinated radial solution directly. Starting a
            // separate height/knee-plane search here can change the trajectory
            // after COM convergence. A rejected pose is conservative; callers
            // must choose an admissible pelvis trajectory.
            for (const auto& leg : { left, right })
            {
                if (!leg.mHipGeometry || !leg.mAnkleGeometry) return std::nullopt;
                const auto hip = clampSwingTwistJointOrientation(*leg.mHipGeometry,
                    transforms[pelvis].mRotation, transforms[leg.mHip].mRotation);
                const auto ankle = clampSwingTwistJointOrientation(*leg.mAnkleGeometry,
                    transforms[leg.mKnee].mRotation, transforms[leg.mFoot].mRotation);
                const auto unchanged = [](const auto& clamped, JPH::QuatArg original) {
                    return clamped && (clamped->IsClose(original, 1e-10f) || clamped->IsClose(-original, 1e-10f));
                };
                if (!unchanged(hip, transforms[leg.mHip].mRotation)
                    || !unchanged(ankle, transforms[leg.mFoot].mRotation))
                    return std::nullopt;
            }
            return pose;
        }
        return std::nullopt;
    }

    std::optional<ReferenceGaitPlan> planReferenceGaitActuation(
        const ReferenceGaitPlanInput& input, const ReferenceGaitProfile& profile, std::string* rejection)
    {
        const auto reject = [&](const char* reason) -> std::optional<ReferenceGaitPlan> {
            if (rejection != nullptr)
                *rejection = reason;
            return std::nullopt;
        };
        if (input.mTelemetry == nullptr || !validProfile(profile))
            return reject("invalid telemetry pointer or profile");
        const RagdollTelemetry& telemetry = *input.mTelemetry;
        const std::size_t count = telemetry.mBodies.size();
        const auto validLeg = [&](const ReferenceGaitLeg& leg) {
            return leg.mHip < count && leg.mKnee < count && leg.mFoot < count && leg.mHip != leg.mKnee
                && leg.mHip != leg.mFoot && leg.mKnee != leg.mFoot && leg.mHip != input.mPelvis
                && leg.mKnee != input.mPelvis && leg.mFoot != input.mPelvis;
        };
        if (count == 0 || input.mBasePose.size() != count || input.mMeasuredPose.size() != count
            || input.mPelvis >= count || !validLeg(input.mLeftLeg) || !validLeg(input.mRightLeg)
            || input.mLeftLeg.mHip == input.mRightLeg.mHip || input.mLeftLeg.mFoot == input.mRightLeg.mFoot)
            return reject("invalid body indices");
        if (!isFinite(input.mGravity) || input.mGravity <= 0.0f || !isFinite(input.mFixedTimeStep)
            || input.mFixedTimeStep <= 0.0f || !isFinite(input.mRootPosition) || !isFinite(input.mRootRotation)
            || !input.mRootRotation.IsNormalized() || !isFinite(input.mUniformScale) || input.mUniformScale <= 0.0f
            || !isFinite(input.mCenterOfMassVelocity) || !isFinite(input.mRequestedHorizontalVelocity)
            || !isFinite(input.mFacing))
            return reject("invalid scalar or frame input");
        if (input.mLandingFootOriginHeight && !std::isfinite(*input.mLandingFootOriginHeight))
            return reject("invalid landing origin height");
        if (input.mPreferredPelvisCenterHeight && (!std::isfinite(*input.mPreferredPelvisCenterHeight)
                || *input.mPreferredPelvisCenterHeight <= 0))
            return reject("invalid preferred pelvis height");
        if (!input.mState.hasStance())
            return reject("no stance");
        if ((!input.mPreviousLoadTorques.empty() && input.mPreviousLoadTorques.size() != count)
            || (!input.mPreviousMotorTorques.empty() && input.mPreviousMotorTorques.size() != count)
            || (input.mSwingStartModel && !isFinite(*input.mSwingStartModel))
            || (input.mState.mTransferCenter && !isFinite(*input.mState.mTransferCenter)))
            return reject("invalid history");
        JPH::Vec3 request = horizontal(input.mRequestedHorizontalVelocity);
        const bool stationary = input.mStationaryCorrectiveStep;
        const JPH::Vec3 stationaryFacing = horizontal(input.mFacing);
        if (stationary && (!request.IsNearZero() || !isFinite(stationaryFacing) || stationaryFacing.IsNearZero()))
            return reject("stationary correction requires zero travel and explicit facing");
        if (!stationary && request.IsNearZero())
            return reject("zero locomotion request");
        // Preserve requests in the qualified low-speed range. For larger
        // demands, let the planning ceiling advance with measured speed and
        // one step of bounded acceleration. Full player input can request
        // 155 units/s while the support plan permits only about 28 units/s
        // of acceleration per step; that distant goal must not dictate the
        // immediate capture placement. Reserve only the configured forward share
        // of that budget in the prediction; lateral balance consumes the rest.
        // This bounds the lead, not the eventual walking speed: as measured
        // velocity grows, the next step's request grows with it. Slower
        // requests still brake immediately, and gameplay retains its raw goal.
        const float speed = request.Length();
        const float attainableSpeed = std::max(0.0f,
            horizontal(input.mCenterOfMassVelocity).Dot(stationary ? stationaryFacing.Normalized() : request / speed))
            + profile.mVelocityLeadAccelerationFraction * profile.mMaximumRequestedAcceleration
                * (profile.mSwingDuration + profile.mDoubleSupportDuration);
        if (profile.mLimitVelocityLead && speed > profile.mUnrestrictedRequestSpeed && attainableSpeed > 0.0f)
        {
            // Blend into the limiter above the qualified range, reaching
            // full authority at twice that speed. A minimum ceiling of 60
            // was tried, but kept the excessive startup demand on full-run
            // input. Blending preserves nominal handovers without carrying
            // that minimum demand into a high-speed start.
            const float blend = profile.mUnrestrictedRequestSpeed > 0.0f
                ? smoothStep(std::clamp(speed / profile.mUnrestrictedRequestSpeed - 1.0f, 0.0f, 1.0f)) : 1.0f;
            const float ceiling = std::max(attainableSpeed,
                profile.mUnrestrictedRequestSpeed * (1.0f - blend) + attainableSpeed * blend);
            if (speed > ceiling)
                request *= ceiling / speed;
        }
        for (const auto& body : telemetry.mBodies)
        {
            if (!isFinite(body.mCenterOfMass) || !isFinite(body.mPosition) || !isFinite(body.mRotation)
                || !isFinite(body.mMotion.mAngularVelocity) || !isFinite(body.mMass) || body.mMass <= 0.0f)
                return reject("non-finite body telemetry");
        }

        const bool leftStance = input.mState.leftFootIsStance();
        const ReferenceGaitLeg& stanceLeg = leftStance ? input.mLeftLeg : input.mRightLeg;
        const ReferenceGaitLeg& swingLeg = leftStance ? input.mRightLeg : input.mLeftLeg;
        const bool swinging = input.mState.mStage == ReferenceGaitStage::Swing;
        const JPH::Vec3 forward = stationary ? stationaryFacing.Normalized() : request.Normalized();
        const JPH::Vec3 right = forward.Cross(JPH::Vec3::sAxisZ()).Normalized();
        const JPH::Quat inverseRoot = input.mRootRotation.Conjugated();
        const auto toModel = [&](JPH::RVec3Arg world) {
            return (inverseRoot * JPH::Vec3(world - input.mRootPosition)) / input.mUniformScale;
        };

        // Heading: the base pelvis facing rotated onto the direction the body
        // should POINT. That is the travel direction unless a facing is given
        // (plan item 2.3a), which is what lets the body strafe or back up
        // while still looking where it means to.
        const JPH::Vec3 requestedFacing = horizontal(input.mFacing);
        const JPH::Vec3 facing = requestedFacing.IsNearZero() ? forward : requestedFacing.Normalized();
        const JPH::Quat basePelvisWorld = (input.mRootRotation * input.mBasePose[input.mPelvis].mRotation).Normalized();
        JPH::Vec3 baseHeading = horizontal(basePelvisWorld * JPH::Vec3::sAxisY());
        // Fable i50: measured pelvis heading (same construction as the swing
        // knee-plane alignment below), for the env-gated body-frame rule.
        JPH::Vec3 measuredHeading = horizontal(telemetry.mBodies[input.mPelvis].mRotation
            * input.mBasePose[input.mPelvis].mRotation.Conjugated() * input.mRootRotation.Conjugated()
            * (basePelvisWorld * JPH::Vec3::sAxisY()));
        if (!isFinite(measuredHeading) || measuredHeading.IsNearZero())
            measuredHeading = forward;
        if (baseHeading.IsNearZero())
            return reject("base pelvis has no horizontal heading");
        baseHeading = baseHeading.Normalized();
        // Fable i55 (pelvis-follow-travel-2026-09-11.txt): env-gated. The
        // pelvis heading target turns from the requested facing toward the
        // travel direction by at most PELVIS_TWIST degrees, so a strafe
        // becomes a diagonal walk with a twisted torso instead of a 90-degree
        // crossover gait. Backpedal (angle > 120 deg) keeps its facing.
        static const auto pelvisTwistEnv_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_PELVIS_TWIST");
            if (!text)
                return 0.0f;
            const float parsed = std::strtof(text, nullptr);
            return std::isfinite(parsed) && parsed > 0.0f ? parsed * (std::numbers::pi_v<float> / 180.0f) : 0.0f;
        });
        static const auto& pelvisTwistEnv = pelvisTwistEnv_tuned.get();
        // Profile field by default (adopted 2026-09-11); the env overrides it.
        const float pelvisTwistLimit = pelvisTwistEnv > 0.0f
            ? pelvisTwistEnv : profile.mSteeringPelvisTwist * (std::numbers::pi_v<float> / 180.0f);
        JPH::Vec3 pelvisFacing = facing;
        if (pelvisTwistLimit > 0.0f && !stationary)
        {
            const float cosine = std::clamp(facing.Dot(forward), -1.0f, 1.0f);
            const float angle = std::acos(cosine);
            if (angle > 1e-3f && angle < 2.0944f)
            {
                const float turn = std::min(angle, pelvisTwistLimit);
                const float sign = (facing.GetX() * forward.GetY() - facing.GetY() * forward.GetX()) >= 0.0f ? 1.0f : -1.0f;
                const float c = std::cos(sign * turn), sn = std::sin(sign * turn);
                pelvisFacing = JPH::Vec3(facing.GetX() * c - facing.GetY() * sn, facing.GetX() * sn + facing.GetY() * c, 0.0f);
            }
        }
        const JPH::Quat headingDelta = JPH::Quat::sFromTo(baseHeading, pelvisFacing).Normalized();
        const JPH::Quat desiredPelvis = (headingDelta * basePelvisWorld).Normalized();

        ReferenceGaitPlan plan;
        plan.mTrackingVelocity = request;
        plan.mTargetPose.mBodyTransforms.assign(input.mBasePose.begin(), input.mBasePose.end());
        plan.mLoadTorques.assign(count, JPH::Vec3::sZero());

        // 1. Inverse statics. Keep real contacts from both feet through
        //    unloading; entering Swing does not mean lift-off has occurred.
        //    After lift-off, plan on the stance foot alone. Double support
        //    spans both feet so lateral transfer loads the trailing foot.
        std::vector<SupportJointBody> bodies;
        bodies.reserve(count);
        for (const auto& body : telemetry.mBodies)
            bodies.push_back({ -1, body.mCenterOfMass, body.mPosition, body.mMass });
        std::vector<int> parents(count, -1);
        for (const auto& joint : telemetry.mConstraints)
        {
            if (joint.mChildBodyIndex == 0 || joint.mChildBodyIndex >= count
                || joint.mParentBodyIndex >= joint.mChildBodyIndex || bodies[joint.mChildBodyIndex].mParent != -1
                || !isFinite(joint.mPosition))
                return reject("invalid constraint topology");
            bodies[joint.mChildBodyIndex].mParent = static_cast<int>(joint.mParentBodyIndex);
            bodies[joint.mChildBodyIndex].mJointPosition = joint.mPosition;
            parents[joint.mChildBodyIndex] = static_cast<int>(joint.mParentBodyIndex);
        }
        const FootLoad leftLoad = measureFootLoad(input.mContacts, input.mLeftLeg.mFoot);
        const FootLoad rightLoad = measureFootLoad(input.mContacts, input.mRightLeg.mFoot);
        const ActiveControllerProfile supportProfile;
        std::vector<GroundContact> supportContacts;
        for (const GroundContact& contact : input.mContacts)
        {
            const bool stanceContact = contact.mBodyIndex == stanceLeg.mFoot;
            const bool swingContact = contact.mBodyIndex == swingLeg.mFoot;
            if (!contact.hasSupport()
                || !(stanceContact || ((!swinging || !input.mState.mSwingFootLifted) && swingContact)))
                continue;
            supportContacts.push_back(contact);
        }
        if (profile.mSupportHullScale < 1.0f)
        {
            for (const auto foot : { input.mLeftLeg.mFoot, input.mRightLeg.mFoot })
            {
                std::vector<GroundContact> patch;
                for (const auto& contact : supportContacts)
                    if (contact.mBodyIndex == foot) patch.push_back(contact);
                if (patch.empty()) continue;
                const auto support = estimateFootSupport(telemetry.mBodies[foot].mCenterOfMass,
                    patch, { input.mLeftLeg.mFoot, input.mRightLeg.mFoot }, supportProfile);
                if (!support) return reject("invalid support contraction patch");
                if (!support->hasSupport()) continue;
                // Contract toward the foot COM only after projecting it onto
                // the actual patch. A toe/heel/edge contact cannot acquire
                // fictional support beneath an airborne part of the foot.
                // A full patch containing the COM keeps the original center.
                const auto center = support->mClosestPoint;
                for (auto& contact : supportContacts)
                    if (contact.mBodyIndex == foot)
                        contact.mPosition = JPH::RVec3(
                            center.GetX() + (contact.mPosition.GetX() - center.GetX()) * profile.mSupportHullScale,
                            center.GetY() + (contact.mPosition.GetY() - center.GetY()) * profile.mSupportHullScale,
                            contact.mPosition.GetZ());
            }
        }
        const float requestedSpeed = request.Length();
        const JPH::Vec3 velocity = horizontal(input.mCenterOfMassVelocity);
        float dsLiftAcceleration = 0.0f;
        float dsPushShiftAmount = 0.0f;
        float forwardAcceleration = profile.mVelocityGain * (requestedSpeed - velocity.Dot(forward));
        // Fable i52 (pushoff-leg-axis-2026-09-11.txt): env-gated push-off.
        static const auto pushOffGain_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_PUSHOFF");
            if (!text)
                return 0.0f;
            const float parsed = std::strtof(text, nullptr);
            return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
        });
        static const auto& pushOffGain = pushOffGain_tuned.get();
        float pushOffBias = 0.0f;
        bool pushOffActive = false;
        if (swinging && !input.mCoordinateLegPose && profile.mTerminalStancePush > 0.0f && profile.mSwingDuration > 0.0f
            && forwardAcceleration > 0.0f
            && input.mState.mTimeInStage >= profile.mTerminalStancePushStartFraction * profile.mSwingDuration)
        {
            // Terminal-stance push (layer 4): bias the request forward late in
            // single support. Added before the magnitude clamp below, so it
            // reallocates the existing request budget rather than raising it.
            // Skipped on the coordinated-pose path: that pose is produced by
            // the joint-valid solver from the unmodified request (same
            // exemption as the stance knee ease and the feasibility gate).
            // Gated off while braking is requested (a forward shove must get
            // the full budget to brake) and faded out with lateral speed, so a
            // lateral shove keeps the whole budget for the capture response.
            float pushGate = 1.0f;
            if (profile.mTerminalStancePushLateralLimit > 0.0f)
            {
                const float lateralSpeed = std::abs(velocity.Dot(right));
                pushGate = std::clamp(2.0f - lateralSpeed / profile.mTerminalStancePushLateralLimit,
                    0.0f, 1.0f);
            }
            const float pushPhase = (input.mState.mTimeInStage
                - profile.mTerminalStancePushStartFraction * profile.mSwingDuration)
                / ((1.0f - profile.mTerminalStancePushStartFraction) * profile.mSwingDuration);
            forwardAcceleration += profile.mTerminalStancePush * smoothStep(pushPhase) * pushGate;
        }
        // An unloaded foot can move while weight is being transferred. Keep
        // the transfer goal at its entry location instead of chasing that
        // motion. Single support still uses the actual foot as its pivot.
        const JPH::Vec3 stanceFootCenter = horizontal(JPH::Vec3(!swinging && input.mState.mTransferCenter
            ? *input.mState.mTransferCenter : telemetry.mBodies[stanceLeg.mFoot].mCenterOfMass));
        const float lateralOffset
            = (stanceFootCenter - horizontal(JPH::Vec3(telemetry.mCenterOfMass))).Dot(right);
        float lateralAcceleration = 0.0f;
        float height = input.mSupport.mCenterOfMassHeight;
        if (!(height > 0.0f))
            height = static_cast<float>(
                telemetry.mCenterOfMass.GetZ() - telemetry.mBodies[stanceLeg.mFoot].mCenterOfMass.GetZ());
        // Velocity toward the stance foot, positive when approaching it.
        const float towardStance = lateralOffset >= 0.0f ? 1.0f : -1.0f;
        plan.mTransferVelocity = velocity.Dot(right) * towardStance;
        if (height > 1e-3f)
        {
            const float tau = std::sqrt(height / input.mGravity);
            const float period = (profile.mDoubleSupportDuration + profile.mSwingDuration) / tau;
            // Same env-gated sway experiment scale as the placement point
            // (sway-reduction-2026-09-10.txt); unset = profile value exactly.
            static const auto lateralCaptureScale_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_LAT_CAPTURE_SCALE");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& lateralCaptureScale = lateralCaptureScale_tuned.get();
            const float lateralCaptureOffset = lateralCaptureScale > 0.0f
                ? lateralCaptureScale * profile.mLateralCaptureOffset : profile.mLateralCaptureOffset;
            // Fable i50: the double-support lateral shuttle exists to reach
            // the travel-frame lateral offset; scale its reference by the
            // same body/travel alignment as the placement (inert unset).
            static const auto bodySideRule_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_BODY_SIDE") != nullptr; });
            static const auto& bodySideRule = bodySideRule_tuned.get();
            float alignmentScale = 1.0f;
            if (bodySideRule)
            {
                const float c = measuredHeading.Normalized().Cross(JPH::Vec3::sAxisZ()).Normalized().Dot(right);
                alignmentScale = std::clamp((std::abs(c) - 0.5f) / (0.8660254f - 0.5f), 0.0f, 1.0f);
            }
            plan.mTransferReferenceVelocity = lateralCaptureOffset * alignmentScale / tau * std::sinh(period)
                / (1.0f + std::exp(-period));
        }
        if (swinging)
        {
            // Center of pressure at the stance foot center: the natural
            // inverted-pendulum acceleration about it, not a hold.
            if (height > 1e-3f)
                lateralAcceleration = -lateralOffset * input.mGravity / height;
            // Lateral ankle CoP authority (candidate D). Off unless
            // OPENMW_RAGDOLL_LATERAL_COP carries the maximum CoP shift from
            // the stance foot center in units; the env knob is a
            // matched-experiment control only, not an encoded or profiled
            // setting. Measured motivation (turn-entry-fps-comparison
            // -2026-09-10.txt): the 120 FPS turn diverges because the
            // ballistic swing amplifies a small post-apex state difference
            // until the capture point is unreachable; the support solver
            // already expresses a 2D CoP inside the contact hull, so placing
            // the CoP under the capture point (clamped to the foot) turns
            // the ballistic fall into a bounded damper. Reduces exactly to
            // the foot-center policy when the capture point is under the
            // foot; hull projection in the solver still enforces reality.
            static const auto lateralCopShift_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_LATERAL_COP");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& lateralCopShift = lateralCopShift_tuned.get();
            if (lateralCopShift > 0.0f && height > 1e-3f && input.mGravity > 0.0f)
            {
                const float captureRelativeToFoot
                    = -lateralOffset + velocity.Dot(right) * std::sqrt(height / input.mGravity);
                const float shift = std::clamp(captureRelativeToFoot, -lateralCopShift, lateralCopShift);
                lateralAcceleration = input.mGravity / height * (-lateralOffset - shift);
            }
        }
        else
        {
            // Double support provides the lateral braking opportunity between
            // single-support vaults. Reduce the cycle target without removing
            // the velocity needed to complete the next weight transfer.
            if (profile.mBrakeLateralTransfer && !input.mState.mInitialTransfer
                && !input.mCoordinateLegPose && !profile.mCoordinateWeightTransfers
                && !profile.mCoordinateInitialTransfer)
                plan.mTransferReferenceVelocity *= 0.5f;
            const float transferOmega = 2.0f * std::numbers::pi_v<float> * profile.mWeightTransferFrequency;
            lateralAcceleration = lateralOffset * transferOmega * transferOmega
                - velocity.Dot(right) * (2.0f * profile.mWeightTransferDamping * transferOmega);
            if (height > 1e-3f)
                lateralAcceleration += towardStance * profile.mTransferVelocityGain
                    * (plan.mTransferReferenceVelocity - plan.mTransferVelocity);
            // Double-support propulsion (env-gated experiment, inert by
            // default; ds-propulsion-2026-09-10.txt). The two-foot hull gives
            // the CoP a backward lever onto the trailing foot that single
            // support lacks, so a forward push is physically realizable here
            // when it is not in late stance. Shaped sin(pi*frac) to peak
            // early/mid DS, while the trailing foot is still loaded (it
            // carries 23% of weight on average, 8.6% at DS end). Added before
            // the magnitude clamp: reallocates the existing request budget,
            // never raises it. The solver's per-contact cone check and hull
            // projection still enforce reality. Not encoded, not profiled.
            // Iteration 3 (same doc): v2's instantaneous lateral-speed fade
            // still let the push re-fire while the post-impulse pendulum was
            // settling (push200-lat1500 fell @307, long after the immediate
            // recovery). DS is where the load-bearing lateral transfer lives;
            // only push when laterally SETTLED: low lateral speed AND the COM
            // near the cycle center, plus clearly positive forward demand.
            static const auto dsPropulsion_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DS_PROPULSION");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& dsPropulsion = dsPropulsion_tuned.get();
            if (dsPropulsion > 0.0f && !input.mState.mInitialTransfer && !input.mCoordinateLegPose
                && !profile.mCoordinateWeightTransfers && !profile.mCoordinateInitialTransfer
                && forwardAcceleration > 5.0f && profile.mDoubleSupportDuration > 0.0f
                && std::abs(velocity.Dot(right)) < 5.0f && std::abs(lateralOffset) < 4.0f)
            {
                const float frac = std::clamp(
                    input.mState.mTimeInStage / profile.mDoubleSupportDuration, 0.0f, 1.0f);
                forwardAcceleration
                    += dsPropulsion * std::sin(std::numbers::pi_v<float> * frac);
            }
            // Double-support leg-extension lift (env-gated experiment, inert
            // by default; ds-propulsion-2026-09-10.txt, iteration 4): a
            // vertical acceleration request through the joint-support
            // solver's Z channel. Raising N = m(g + az) during DS raises the
            // friction budget exactly when the trailing foot pushes, and the
            // extra vertical force is what extends the trailing leg (the
            // biomechanical push-off). Unlike the horizontal request bias,
            // this ADDS contact-force budget instead of reallocating it.
            // v1 gates (active in cruise): DS, not initial transfer, not
            // coordinated, forward demand present. The solver enforces the
            // per-contact cone; az = 0 when unset preserves the historical
            // constant-height plan byte-for-byte.
            if (pushOffGain > 0.0f && !input.mState.mInitialTransfer && !input.mCoordinateLegPose
                && !profile.mCoordinateWeightTransfers && !profile.mCoordinateInitialTransfer
                && input.mState.mStage == ReferenceGaitStage::DoubleSupport && profile.mDoubleSupportDuration > 0.0f)
            {
                const float frac = std::clamp(
                    input.mState.mTimeInStage / profile.mDoubleSupportDuration, 0.0f, 1.0f);
                // Demand-proportional: a fixed bias overspeeds and skids at low
                // requests (grid-3 ensemble: pass 0.7%). Scale by the forward
                // speed deficit fraction, zero when the request is met.
                const float deficit = requestedSpeed > 1e-3f
                    ? std::clamp((requestedSpeed - velocity.Dot(forward)) / requestedSpeed, 0.0f, 1.0f) : 0.0f;
                pushOffBias = pushOffGain * std::sin(std::numbers::pi_v<float> * frac) * deficit;
                forwardAcceleration += pushOffBias;
                pushOffActive = pushOffBias > 0.0f;
            }
            static const auto dsLift_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DS_LIFT");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& dsLift = dsLift_tuned.get();
            if (dsLift > 0.0f && !input.mState.mInitialTransfer && !input.mCoordinateLegPose
                && !profile.mCoordinateWeightTransfers && !profile.mCoordinateInitialTransfer
                && forwardAcceleration > 0.0f && profile.mDoubleSupportDuration > 0.0f)
            {
                const float frac = std::clamp(
                    input.mState.mTimeInStage / profile.mDoubleSupportDuration, 0.0f, 1.0f);
                dsLiftAcceleration = dsLift * std::sin(std::numbers::pi_v<float> * frac);
            }
            // Double-support push-off CoP shift (env-gated experiment, inert
            // by default; ds-propulsion-2026-09-10.txt, iteration 5): the
            // faithful version of the trailing-leg push. Instead of biasing
            // the acceleration request (zero-sum against the lateral
            // transfer, iterations 1-3), the solver shifts the solved center
            // of pressure aft along -forward, moving only the fore-aft
            // coordinate: the lateral transfer keeps its solved CoP share and
            // the shifted CoP loads the trailing-foot contacts through the
            // ordinary weight fan, with the cone check and hull projection
            // still the reality guards. Shaped sin(pi*frac) to peak mid-DS
            // while the trailing foot is still loaded. Not encoded, not
            // profiled; zero when unset preserves the plan byte-for-byte.
            static const auto dsPushShift_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DS_PUSH_SHIFT");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& dsPushShift = dsPushShift_tuned.get();
            if (dsPushShift > 0.0f && !input.mState.mInitialTransfer && !input.mCoordinateLegPose
                && !profile.mCoordinateWeightTransfers && !profile.mCoordinateInitialTransfer
                && forwardAcceleration > 0.0f && profile.mDoubleSupportDuration > 0.0f)
            {
                const float frac = std::clamp(
                    input.mState.mTimeInStage / profile.mDoubleSupportDuration, 0.0f, 1.0f);
                dsPushShiftAmount = dsPushShift * std::sin(std::numbers::pi_v<float> * frac);
            }
        }
        JPH::Vec3 acceleration = forward * forwardAcceleration + right * lateralAcceleration;
        const bool coordinatedProfile = input.mCoordinateLegPose || profile.mCoordinateWeightTransfers
            || profile.mCoordinateInitialTransfer;
        // Capture-brake pull (set only when the env knob is on and the capture
        // point is computable): direction and magnitude that drive the capture
        // point back to the nearest support point, a = (s - capture) / tau^2.
        float captureBrakeExcess = -1.0f;
        JPH::Vec3 captureBrakePull = JPH::Vec3::sZero();
        {
            // Capture-runaway governor (layer 7 candidate). Off unless
            // OPENMW_RAGDOLL_CAPTURE_GOVERNOR carries a positive gain; the env
            // knob is a matched-experiment control only, not an encoded or
            // profiled setting. Measured motivation on the a63 immediate-turn
            // 120 FPS replay (default config): every landing after step 598 is
            // re-swung by the emergency path because the capture point is
            // already 11-22 units outside the support (margin 8), so no double
            // support consolidates while the saturated velocity-error term
            // keeps the request pinned at the 75.56 clamp and |v| runs 26->94.
            // Chasing the velocity goal during a capture escape is the
            // positive feedback; attenuate only the forward goal term with the
            // measured excess and leave the lateral/balance terms intact.
            // Same coordinated-profile and stationary exemptions as layer 5.
            // REJECTED on experiment (capture-governor-rejection-2026-09-10.txt):
            // the forward term measured 2-16 against the -75.5 lateral pendulum
            // term, so attenuating it only delayed the 120 fall and broke the
            // surviving 60 FPS run. Retained inert for reference.
            static const auto captureGovernor_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CAPTURE_GOVERNOR");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& captureGovernor = captureGovernor_tuned.get();
            static const auto captureBrake_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CAPTURE_BRAKE");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& captureBrake = captureBrake_tuned.get();
            if (!coordinatedProfile && !stationary && (captureGovernor > 0.0f || captureBrake > 0.0f)
                && height > 0.0f && input.mGravity > 0.0f && profile.mEmergencyCaptureMargin > 0.0f
                && (leftLoad.mSupported || rightLoad.mSupported))
            {
                const float captureTau = std::sqrt(height / input.mGravity);
                const JPH::Vec3 capture = horizontal(JPH::Vec3(telemetry.mCenterOfMass)) + velocity * captureTau;
                const JPH::Vec3 leftCenter = horizontal(JPH::Vec3(telemetry.mBodies[input.mLeftLeg.mFoot].mCenterOfMass));
                const JPH::Vec3 rightCenter = horizontal(JPH::Vec3(telemetry.mBodies[input.mRightLeg.mFoot].mCenterOfMass));
                // Same collapse as the state machine: an airborne foot supplies
                // no support, so the segment degenerates to the contacting foot.
                const JPH::Vec3 supportStart = leftLoad.mSupported ? leftCenter : rightCenter;
                const JPH::Vec3 supportEnd = rightLoad.mSupported ? rightCenter : leftCenter;
                const JPH::Vec3 supportSpan = supportEnd - supportStart;
                const float supportLength2 = supportSpan.LengthSq();
                const float along = supportLength2 > 1e-6f
                    ? std::clamp((capture - supportStart).Dot(supportSpan) / supportLength2, 0.0f, 1.0f) : 0.0f;
                const JPH::Vec3 supportPoint = supportStart + supportSpan * along;
                const float excess = (capture - supportPoint).Length();
                if (std::isfinite(excess))
                {
                    captureBrakeExcess = excess;
                    captureBrakePull = (supportPoint - capture) * (input.mGravity / height);
                }
                if (captureGovernor > 0.0f && std::isfinite(excess) && excess > profile.mEmergencyCaptureMargin)
                {
                    const float scale = std::max(0.0f,
                        1.0f - captureGovernor * (excess - profile.mEmergencyCaptureMargin)
                            / profile.mEmergencyCaptureMargin);
                    acceleration = forward * (forwardAcceleration * scale) + right * lateralAcceleration;
                }
            }
        }
        if (!coordinatedProfile && profile.mMaximumLateralRequestFraction < 1.0f
            && profile.mMaximumRequestedAcceleration > 0.0f)
        {
            // Layer 5: cap the lateral component so the wide-stance pendulum
            // term cannot consume the whole request budget (see the profile
            // field comment). Applied before the magnitude clamp below, and
            // skipped for coordinated profiles: their pose comes from the
            // joint-valid solver fed by the unmodified request.
            const float lateralCap = profile.mMaximumLateralRequestFraction * profile.mMaximumRequestedAcceleration;
            const float cappedLateral = std::clamp(lateralAcceleration, -lateralCap, lateralCap);
            if (cappedLateral != lateralAcceleration)
                acceleration = forward * forwardAcceleration + right * cappedLateral;
        }
        {
            // Forward-reserve governor pilot (layer 6 candidate). Off unless
            // OPENMW_RAGDOLL_FORWARD_RESERVE carries a fraction in (0,1]; the
            // env knob is a matched-experiment control only, not an encoded or
            // profiled setting. Measured motivation on the a63 immediate-turn
            // 120 FPS replay (margin0 baseline): post-apex the lateral
            // pendulum term pins the request at the 75.56 magnitude clamp
            // while forward demand of 44-78 u/s^2 is served 2-18, and the run
            // falls at step 683. Reserving a forward share only when forward
            // demand exists keeps directed progress without touching pure
            // lateral crises. Same coordinated-profile exemption as layer 5.
            static const auto forwardReserve_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_FORWARD_RESERVE");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f && parsed <= 1.0f ? parsed : 0.0f;
            });
            static const auto& forwardReserve = forwardReserve_tuned.get();
            if (!coordinatedProfile && forwardReserve > 0.0f && !stationary)
                acceleration = governForwardReserve(acceleration, forward, right,
                    profile.mMaximumRequestedAcceleration, forwardReserve);
        }
        // Capture brake (capture-brake-hold-2026-09-10.txt): while the state
        // machine holds a double support through a recoverable capture escape,
        // travel demand yields the whole request budget to the brake that
        // drives the capture point back to the support. Engages only when the
        // excess exceeds the emergency margin, which healthy walking never
        // reaches (measured < 3 units at steps 540-560 of the default-120
        // replay), so nominal transfers are untouched.
        static const auto captureBrakeApply_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CAPTURE_BRAKE");
            if (!text)
                return 0.0f;
            const float parsed = std::strtof(text, nullptr);
            return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
        });
        static const auto& captureBrakeApply = captureBrakeApply_tuned.get();
        if (captureBrakeApply > 0.0f && captureBrakeExcess > profile.mEmergencyCaptureMargin
            && input.mState.mStage == ReferenceGaitStage::DoubleSupport)
            acceleration = captureBrakePull;
        if (stationary && !swinging && height > 1e-3f)
        {
            // An idle corrective controller must not perform the walking
            // transfer toward one foot while waiting for an emergency.
            // Balance about the measured combined support until a swing begins.
            const float omega = std::sqrt(input.mGravity / height);
            acceleration = horizontal(JPH::Vec3(input.mSupport.mCenter - telemetry.mCenterOfMass))
                * (omega * omega) - velocity * (2.0f * omega);
            plan.mTransferReferenceVelocity = 0.0f;
        }
        const float accelerationLength = acceleration.Length();
        const float accelerationCap = profile.mMaximumRequestedAcceleration + pushOffBias;
        if (accelerationLength > accelerationCap && accelerationLength > 0.0f)
            acceleration *= accelerationCap / accelerationLength;
        plan.mRequestedAcceleration = JPH::Vec3(acceleration.GetX(), acceleration.GetY(), 0.0f);
        {
            JointSupportInput supportInput;
            supportInput.mBodies = bodies;
            supportInput.mContacts = supportContacts;
            supportInput.mFeet = { input.mLeftLeg.mFoot, input.mRightLeg.mFoot };
            supportInput.mGravity = input.mGravity;
            supportInput.mFriction = input.mSupport.mFriction.value_or(0.0f);
            supportInput.mRequestedAcceleration = plan.mRequestedAcceleration;
            // Fable i51 (run-hop-2026-09-11.txt): launch request late in the
            // swing. Env-gated; unset leaves dsLiftAcceleration exactly.
            static const auto runLift_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_RUN_LIFT");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& runLift = runLift_tuned.get();
            static const auto runLaunch_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_RUN_LAUNCH");
                if (!text)
                    return 0.7f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f && parsed < 1.0f ? parsed : 0.7f;
            });
            static const auto& runLaunch = runLaunch_tuned.get();
            float launchAcceleration = dsLiftAcceleration;
            if (runLift > 0.0f && swinging && profile.mSwingDuration > 0.0f)
            {
                const float start = runLaunch * profile.mSwingDuration;
                const float span = std::max(profile.mSwingDuration - start, 1e-3f);
                const float f = std::clamp((input.mState.mTimeInStage - start) / span, 0.0f, 1.0f);
                if (f > 0.0f)
                    launchAcceleration = runLift * std::sin(std::numbers::pi_v<float> * f);
            }
            // Tracked-gait compass vault (env OPENMW_RAGDOLL_TRACKED_VAULT = leg
            // length L): request the vertical acceleration of a COM riding a
            // rigid strut over the stance foot, az = -vF^2 L^2 / h^3 with
            // h = sqrt(L^2 - x^2), so the solver plans the arc (unloading at
            // the apex) instead of a flat height. Clamped to -0.8 g.
            static const auto vaultLength_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_TRACKED_VAULT");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 10.0f ? parsed : 0.0f;
            });
            static const auto& vaultLength = vaultLength_tuned.get();
            if (vaultLength > 0.0f && swinging)
            {
                const float x = horizontal(JPH::Vec3(telemetry.mCenterOfMass - telemetry.mBodies[stanceLeg.mFoot].mCenterOfMass)).Dot(forward);
                const float h2 = vaultLength * vaultLength - x * x;
                if (h2 > 100.0f)
                {
                    const float h = std::sqrt(h2);
                    const float vF = velocity.Dot(forward);
                    const float az = -(vF * vF) * vaultLength * vaultLength / (h * h * h);
                    launchAcceleration += std::max(az, -0.8f * input.mGravity);
                }
            }
            supportInput.mRequestedVerticalAcceleration = launchAcceleration;
            if (pushOffActive)
            {
                supportInput.mLegAxisFeet = { { swingLeg.mFoot, stanceLeg.mFoot } };
                supportInput.mLegAxisHips = { { swingLeg.mHip, stanceLeg.mHip } };
            }
            if (dsPushShiftAmount > 0.0f && forward.LengthSq() > 1e-6f)
            {
                const JPH::Vec3 aft = -forward.Normalized();
                supportInput.mPushDirection = JPH::Vec3(aft.GetX(), aft.GetY(), 0.0f);
                supportInput.mPushShift = dsPushShiftAmount;
            }
            // Hip-moment borrow (env-gated, inert by default;
            // hip-moment-borrow-2026-09-11.txt): engage the solver's
            // counter-rotation budget only while the state machine itself is
            // executing a reactive emergency step. Cruise capture excess in
            // this harness measured 12-24 u (always above the 8-u margin), so
            // an excess-threshold gate would spend the angular-momentum
            // battery on ordinary walking; the machine's own emergency flag
            // is the honest discriminator (never set in cruise).
            static const auto hipMoment_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HIP_MOMENT");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& hipMoment = hipMoment_tuned.get();
            if (hipMoment > 0.0f && input.mState.mEmergencyStep)
                supportInput.mCounterRotationAcceleration = hipMoment;
            if (runLift > 0.0f && supportContacts.empty())
            {
                // Fable i51: airborne. No support to solve; pose targets only.
                plan.mLoadTorques.assign(count, JPH::Vec3::sZero());
            }
            else
            {
                const auto support = calculateJointSupportControl(supportInput, supportProfile);
                if (!support || support->mJointTorques.size() != count)
                    return reject("inverse-static support plan rejected");
                plan.mSupport = *support;
                plan.mLoadTorques = support->mJointTorques;
                for (const auto& force : support->mContactForces)
                    if (force.mBodyIndex == stanceLeg.mFoot)
                        plan.mPlannedStanceFootLoad += force.mForce.GetZ();
            }
        }

        // 2. Target pose. The pelvis target is its heading-upright world
        //    orientation and every loaded thigh keeps its measured world
        //    orientation, so the loaded hip springs pull the pelvis upright
        //    with the anchored leg as reaction. Calves retain their base model
        //    rotations unless explicit stance flexion is selected; consequently
        //    the knee target is coupled to the measured-thigh hip hold. The soft
        //    ankle preserves its measured joint angle, and the swing leg follows
        //    its trajectory. This is not a pure relative-knee reference controller.
        auto& target = plan.mTargetPose.mBodyTransforms;
        target[input.mPelvis].mRotation = (inverseRoot * desiredPelvis).Normalized();
        const std::array<std::size_t, 1> stanceHip{ stanceLeg.mHip };
        if (!applyGaitMeasuredLegTargets(target, input.mMeasuredPose, input.mPelvis, stanceHip, GaitMeasuredPoseFrame::Model))
            return reject("stance hip hold rejected");
        if (!swinging)
        {
            const std::array<std::size_t, 1> otherHip{ swingLeg.mHip };
            if (!applyGaitMeasuredLegTargets(target, input.mMeasuredPose, input.mPelvis, otherHip, GaitMeasuredPoseFrame::Model))
                return reject("trailing hip hold rejected");
        }
        // Rate-consistent hold: the implicit motor solves for the end of the
        // step, so a held thigh target is extrapolated by its measured world
        // rate and that rate becomes the motor's relative rate target. The
        // damping then acts on the pelvis rate instead of braking the leg's
        // rotation over the planted foot.
        // A held stance (no step coming) keeps the hold's damping of its sway
        // through the loaded hip; a walking stance must not brake its
        // rotation over the planted foot.
        const auto rateHold = [&](std::size_t hip) {
            if (!profile.mStanceHipRateHold || input.mHoldSwingFoot)
                return;
            const JPH::Vec3 omega = telemetry.mBodies[hip].mMotion.mAngularVelocity;
            if (!isFinite(omega))
                return;
            const float angle = omega.Length() * input.mFixedTimeStep;
            if (angle > 1e-6f)
            {
                const JPH::Quat stepRotation = JPH::Quat::sRotation(omega / omega.Length(), angle);
                target[hip].mRotation
                    = (inverseRoot * stepRotation * input.mRootRotation * target[hip].mRotation).Normalized();
            }
            plan.mJointAngularVelocityTargets.emplace_back(hip, omega);
        };
        rateHold(stanceLeg.mHip);
        if (!swinging)
            rateHold(swingLeg.mHip);
        // A planted foot stays level about its own measured heading. Yawing it
        // toward the request would twist the ankle against ground friction;
        // heading is corrected through the pelvis hold instead.
        const auto levelFoot = [&](const ReferenceGaitLeg& leg) {
            const JPH::Quat baseFootWorld
                = (input.mRootRotation * input.mBasePose[leg.mFoot].mRotation).Normalized();
            JPH::Vec3 footHeading = horizontal(telemetry.mBodies[leg.mFoot].mRotation
                * input.mBasePose[leg.mFoot].mRotation.Conjugated() * input.mRootRotation.Conjugated()
                * (baseFootWorld * JPH::Vec3::sAxisY()));
            JPH::Quat footYaw = headingDelta;
            if (!footHeading.IsNearZero())
                footYaw = JPH::Quat::sFromTo(horizontal(baseFootWorld * JPH::Vec3::sAxisY()).Normalized(),
                    footHeading.Normalized()).Normalized();
            plan.mWorldFootTargets.emplace_back(leg.mFoot, (footYaw * baseFootWorld).Normalized());
        };
        const auto softAnkle = [&](const ReferenceGaitLeg& leg) {
            // The ankle is relative to its calf, not to the pelvis. Assemble
            // it after the loaded knee target so posture changes cannot add
            // an unintended ankle spring command to the planned support load.
            target[leg.mFoot].mRotation = (target[leg.mKnee].mRotation
                * input.mMeasuredPose[leg.mKnee].mRotation.Conjugated()
                * input.mMeasuredPose[leg.mFoot].mRotation).Normalized();
            plan.mLoadedAnkleBodies.push_back(leg.mFoot);
            plan.mLoadedAnkleFrequency = profile.mLoadedAnkleFrequency;
            plan.mLoadedAnkleDamping = profile.mLoadedAnkleDamping;
        };
        const auto loadedLeg = [&](const ReferenceGaitLeg& leg) {
            if (profile.mStanceKneeFlexion > 0.0f && leg.mHingeGeometry)
            {
                // Flex the loaded knee about its native hinge axis, relative
                // to the held thigh target, by the configured angle.
                const JPH::Quat relativeBase
                    = (input.mBasePose[leg.mHip].mRotation.Conjugated() * input.mBasePose[leg.mKnee].mRotation).Normalized();
                float flexion = profile.mStanceKneeFlexion;
                // Continuity at takeoff: a leg that has just become the
                // stance leg eases its knee target from the MEASURED knee
                // flexion instead of snapping to the stance flexion. The snap
                // jumped the command when the landing press handed the leg
                // over at takeoff (0.487 rad, tracking error 0.013 -> 0.505
                // in the E10 trace). The stage clock restarts at the
                // takeover: during the landing press the press owns the knee,
                // so the ease covers the first frames of single support. The
                // measured start is recomputed every step from the current
                // pose -- no stored command history -- so the reference
                // chases the body smoothly as the motor tracks. Skipped when
                // a coordinated pose is planned this frame: that solver reads
                // the eased target as its reference and already produces a
                // joint-valid continuous pose, so easing there only degrades
                // its workspace (measured 2026-09-08: coordinated transfers
                // fell with the ease, nearly pass without it).
                const bool coordinatedThisFrame = input.mCoordinateLegPose || (!swinging
                    && (profile.mCoordinateWeightTransfers
                        || (profile.mCoordinateInitialTransfer && input.mState.mInitialTransfer)));
                if (leg.mHip == stanceLeg.mHip && profile.mStanceKneeEaseDuration > 0.0f
                    && !coordinatedThisFrame && input.mState.mTimeInStage < profile.mStanceKneeEaseDuration)
                {
                    const JPH::Quat relativeMeasured = (input.mMeasuredPose[leg.mHip].mRotation.Conjugated()
                        * input.mMeasuredPose[leg.mKnee].mRotation).Normalized();
                    const JPH::Quat delta
                        = (relativeMeasured * relativeBase.Conjugated()).Normalized().EnsureWPositive();
                    JPH::Vec3 deltaAxis;
                    float deltaAngle = 0.0f;
                    delta.GetAxisAngle(deltaAxis, deltaAngle);
                    const float measuredFlexion = std::clamp(
                        -deltaAxis.Dot(leg.mHingeGeometry->mHingeAxis) * deltaAngle, 0.0f, 1.2f);
                    flexion = measuredFlexion + (flexion - measuredFlexion)
                        * smoothStep(input.mState.mTimeInStage / profile.mStanceKneeEaseDuration);
                }
                // Tracked-gait landing strut (NEW-ROUTE-PLAN, env-gated): for
                // the first STRUT seconds of a new stance the knee target is the
                // extended pose, so heel strike meets a rigid leg and the
                // collision redirects the COM instead of being absorbed.
                static const auto strutDuration_tuned = RagdollCore::tuned<float>([] {
                    const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_TRACKED_STRUT");
                    if (!text)
                        return 0.0f;
                    const float parsed = std::strtof(text, nullptr);
                    return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
                });
                static const auto& strutDuration = strutDuration_tuned.get();
                static const auto strutFlexion_tuned = RagdollCore::tuned<float>([] {
                    const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_TRACKED_STRUT_FLEXION");
                    if (!text)
                        return 0.1f;
                    const float parsed = std::strtof(text, nullptr);
                    return std::isfinite(parsed) && parsed >= 0.0f ? parsed : 0.1f;
                });
                static const auto& strutFlexion = strutFlexion_tuned.get();
                if (strutDuration > 0.0f && leg.mHip == stanceLeg.mHip && !coordinatedThisFrame
                    && input.mState.mTimeInPhase < strutDuration)
                {
                    const float blend = smoothStep(input.mState.mTimeInPhase / strutDuration);
                    flexion = strutFlexion + (flexion - strutFlexion) * blend;
                }
                target[leg.mKnee].mRotation = (target[leg.mHip].mRotation
                    * JPH::Quat::sRotation(leg.mHingeGeometry->mHingeAxis, -flexion) * relativeBase)
                                                  .Normalized();
            }
            if (profile.mStanceLegFrequency > 0.0f)
            {
                for (const std::size_t body : { leg.mHip, leg.mKnee, leg.mFoot })
                    plan.mStanceLegSpringBodies.push_back(body);
                plan.mStanceLegFrequency = profile.mStanceLegFrequency;
                plan.mStanceLegDamping = profile.mStanceLegDamping;
            }
            if (profile.mStanceHipFrequency > 0.0f)
            {
                plan.mStanceHipSpringBodies.push_back(leg.mHip);
                plan.mStanceHipFrequency = profile.mStanceHipFrequency;
                plan.mStanceHipDamping = profile.mStanceHipDamping;
            }
        };
        loadedLeg(stanceLeg);
        if (!swinging)
            loadedLeg(swingLeg);
        const bool softLoadedAnkle = profile.mLoadedAnkleFrequency > 0.0f;
        if (softLoadedAnkle)
        {
            softAnkle(stanceLeg);
            if (!swinging)
                softAnkle(swingLeg);
        }
        else
        {
            levelFoot(stanceLeg);
            if (!swinging && (leftStance ? rightLoad.mSupported : leftLoad.mSupported))
                levelFoot(swingLeg);
        }
        plan.mStanceHip = stanceLeg.mHip;
        plan.mStanceAnkle = stanceLeg.mFoot;
        if (profile.mTrunkFrequency > 0.0f)
        {
            for (const std::size_t body : input.mTrunk)
                if (body < count && body != input.mPelvis)
                    plan.mTrunkSpringBodies.push_back(body);
            plan.mTrunkFrequency = profile.mTrunkFrequency;
            plan.mTrunkDamping = profile.mTrunkDamping;
        }

        // 3. Swing trajectory and leg pose.
        bool swingFeedForwardComputed = false;
        bool placementReachLimited = false;
        if (swinging && input.mSwingStartModel)
        {
            ReferenceGaitPlacementInput placementInput;
            placementInput.mPhase = input.mState.mPhase;
            placementInput.mCenterOfMass = telemetry.mCenterOfMass;
            placementInput.mCenterOfMassVelocity = input.mCenterOfMassVelocity;
            placementInput.mRequestedVelocity = request;
            placementInput.mFacing = facing;
            placementInput.mBodyForward = measuredHeading;
            placementInput.mStationaryCorrectiveStep = stationary;
            placementInput.mStanceFootPosition = telemetry.mBodies[stanceLeg.mFoot].mPosition;
            // i37 (backpedal-side-guard-2026-09-11.txt): supply the swing
            // foot's current position so the guarded measured-side override
            // in calculateReferenceGaitPlacement can fire. Placement is
            // recomputed every step, so current geometry - not a lift-off
            // snapshot - is the consistent choice. Inert in forward walking
            // (measured side agrees with anatomical side there).
            placementInput.mSwingStartPosition = telemetry.mBodies[swingLeg.mFoot].mPosition;
            // i39 (delivery-comp-2026-09-11.txt): the delivery-error
            // observer bias maintained by the state machine. Stays exactly
            // 0 unless the env gate fed it at a touchdown, so the default
            // path is byte-identical.
            placementInput.mDeliveryVelocityBias = input.mState.mDeliveryVelocityBias;
            placementInput.mCenterOfMassHeight = input.mSupport.mCenterOfMassHeight;
            if (!(placementInput.mCenterOfMassHeight > 0.0f))
                placementInput.mCenterOfMassHeight = static_cast<float>(
                    telemetry.mCenterOfMass.GetZ() - telemetry.mBodies[stanceLeg.mFoot].mPosition.GetZ());
            placementInput.mGravity = input.mGravity;
            placementInput.mStancePeriod = profile.mDoubleSupportDuration + profile.mSwingDuration;
            // i33 (rotation-predict-2026-09-11.txt): caller-measured signed
            // travel-frame yaw rate; consumed only under the env gate inside
            // calculateReferenceGaitPlacement.
            placementInput.mTravelYawRate = input.mTravelYawRate;
            placementInput.mPredict = profile.mPredictPlacement;
            placementInput.mRemainingSwingTime = std::max(0.0f, profile.mSwingDuration - input.mState.mTimeInStage);
            placementInput.mVelocityGain = profile.mVelocityGain;
            placementInput.mStanceFootCenter = telemetry.mBodies[stanceLeg.mFoot].mCenterOfMass;
            if (input.mStopping && profile.mStoppingVelocityErrorGain > 0.0f)
                placementInput.mForwardVelocityErrorGain = profile.mStoppingVelocityErrorGain;
            {
                // Effective forward half-length of the planned stance hull.
                float halfLength = 0.0f;
                const auto center = horizontal(JPH::Vec3(placementInput.mStanceFootCenter));
                for (const GroundContact& contact : supportContacts)
                    if (contact.mBodyIndex == stanceLeg.mFoot)
                        halfLength = std::max(halfLength,
                            std::abs((horizontal(JPH::Vec3(contact.mPosition)) - center).Dot(forward)));
                placementInput.mForwardPressureHalfLength = halfLength;
            }
            // i12 (placement-lead-observer-2026-09-11.txt): the opt-in
            // lateral capture lead applies only inside a latched disturbance
            // window, so cruise placements stay nominal with the knob set.
            static const auto captureLeadK_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CAPTURE_LEAD");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f ? parsed : 0.0f;
            });
            static const auto& captureLeadK = captureLeadK_tuned.get();
            if (captureLeadK > 0.0f && input.mState.mDisturbanceLatchRemaining > 0.0f)
                placementInput.mCaptureLead = captureLeadK;
            const auto placement = calculateReferenceGaitPlacement(placementInput, profile);
            // No valid placement (for example a tumbling body with the foot
            // above its center of mass) keeps the previous placement when
            // there is one; otherwise the swing pose is skipped this step.
            if (placement)
            {
                plan.mPlacement = placement->mTarget;
                plan.mPlacementCenterOfMass = placement->mPlacementCenterOfMass;
                plan.mPlacementVelocity = placement->mPlacementVelocity;
            }
            else if (input.mPreviousPlacement && isFinite(*input.mPreviousPlacement))
                plan.mPlacement = input.mPreviousPlacement;
        }
        if (swinging && input.mSwingStartModel && plan.mPlacement)
        {
            // Placement-latch experiment (env-gated, inert by default;
            // touchdown-scuff-2026-09-10.txt): the unlatched placement
            // retreats during swing as the vault brakes the COM, the foot
            // cannot track the retreat, and it lands ~11 u past the final
            // target carrying ~50 u/s of forward scuff that spends the
            // slide margin. Latching mid-swing freezes the target so the
            // smoothstep travel ends against a fixed point. Not encoded,
            // not profiled; unset preserves the profile value exactly.
            static const auto latchOverride_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_PLACEMENT_LATCH");
                if (!text)
                    return 0.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed > 0.0f && parsed < 1.0f ? parsed : 0.0f;
            });
            static const auto& latchOverride = latchOverride_tuned.get();
            const float latchFraction
                = latchOverride > 0.0f ? latchOverride : profile.mPlacementLatchFraction;
            // i41b (chase-commit-2026-09-11.txt): commit a TIMED-OUT swing.
            // A swing that outlives mMaximumStepDuration without loading is
            // the i40 livelock (touchdown-stage chase, apex re-ascending,
            // foot hovering). Latching its current target stops the retreat
            // and lets the descent complete against a fixed point.
            // timed_out is 0 in all 11 screen baselines, so this is inert
            // across the qualified envelope by construction; env-gated for
            // byte-identity with the gate unset. i41a's growth-streak
            // criterion was refuted (retreat is 0.1-0.6 u/step, below any
            // safe noise floor).
            static const auto chaseCommitOnTimeout_tuned = RagdollCore::tuned<bool>([] {
                return RagdollCore::tuningText("OPENMW_RAGDOLL_CHASE_COMMIT") != nullptr;
            });
            static const auto& chaseCommitOnTimeout = chaseCommitOnTimeout_tuned.get();
            const bool commitTimeout = chaseCommitOnTimeout && input.mState.mTimedOut
                && input.mState.mStage == ReferenceGaitStage::Swing;
            if (input.mPreviousPlacement && isFinite(*input.mPreviousPlacement)
                && (commitTimeout
                    || (latchFraction < 1.0f
                        && input.mState.mTimeInStage >= latchFraction * profile.mSwingDuration)))
            {
                plan.mPlacement = input.mPreviousPlacement;
                plan.mPlacementLatched = true;
            }
            // Horizontal placement/latching must not copy a rolling stance
            // body's elevated origin into the observed landing surface.
            if (input.mLandingFootOriginHeight)
                plan.mPlacement->SetZ(*input.mLandingFootOriginHeight);
            JPH::Vec3 placementModel = toModel(*plan.mPlacement);
            ReferenceSwingTrajectoryInput trajectoryInput;
            trajectoryInput.mLiftOffPosition = *input.mSwingStartModel;
            trajectoryInput.mPlacementPosition = placementModel;
            trajectoryInput.mSwingTime = input.mState.mTimeInStage;
            trajectoryInput.mHoldLifted = input.mHoldSwingFoot;
            // i45 (swing-intercept-2026-09-12.txt): supply the placement
            // target's world-frame velocity v = omega x r (r = placement
            // minus stance foot, horizontal) while the travel frame yaws,
            // so the trajectory can command a velocity-matched arrival.
            // Env-gated; unset or |omega| <= 0.1 rad/s leaves the field
            // zero and the trajectory byte-identical.
            static const auto swingIntercept_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_SWING_INTERCEPT") != nullptr; });
            static const auto& swingIntercept = swingIntercept_tuned.get();
            if (swingIntercept && std::isfinite(input.mTravelYawRate)
                && std::abs(input.mTravelYawRate) > 0.1f)
            {
                const JPH::Vec3 lever = horizontal(JPH::Vec3(*plan.mPlacement
                    - telemetry.mBodies[stanceLeg.mFoot].mPosition));
                const JPH::Vec3 velocityWorld = input.mTravelYawRate
                    * JPH::Vec3(-lever.GetY(), lever.GetX(), 0.0f);
                trajectoryInput.mPlacementVelocity = (inverseRoot * velocityWorld) / input.mUniformScale;
            }
            auto footTarget = calculateReferenceSwingTrajectory(trajectoryInput, profile);
            if (!footTarget)
                return reject("swing trajectory rejected");
            if (profile.mContinuousSwingTrajectory && !input.mCoordinateLegPose && !input.mHoldSwingFoot
                && input.mState.mTimeInStage >= profile.mSwingTravelStartFraction * profile.mSwingDuration)
            {
                auto history = input.mSwingTrajectoryHistory;
                if (history && history->mSwingFoot != swingLeg.mFoot)
                    history.reset();
                const float remaining = profile.mSwingTravelEndFraction * profile.mSwingDuration
                    - input.mState.mTimeInStage;
                if (remaining > 0.0f)
                {
                    SwingTrajectorySample initial;
                    initial.mPosition = horizontal(*footTarget);
                    // First entry seeds from the legacy trajectory at this phase.
                    // Subsequent updates use only the retained command history.
                    auto nextInput = trajectoryInput;
                    nextInput.mSwingTime += input.mFixedTimeStep;
                    const auto next = calculateReferenceSwingTrajectory(nextInput, profile);
                    nextInput.mSwingTime += input.mFixedTimeStep;
                    const auto after = calculateReferenceSwingTrajectory(nextInput, profile);
                    if (!next || !after || !(input.mFixedTimeStep > 0.0f))
                        return reject("continuous swing initial trajectory rejected");
                    initial.mVelocity = horizontal((*next - *footTarget) / input.mFixedTimeStep);
                    initial.mAcceleration = horizontal((*after - 2.0f * *next + *footTarget)
                        / (input.mFixedTimeStep * input.mFixedTimeStep));
                    SwingTrajectorySample goal;
                    goal.mPosition = horizontal(placementModel);
                    const float advance = history ? std::min(input.mFixedTimeStep,
                        std::max(0.0f, history->mSegment.mDuration - history->mElapsed)) : 0.0f;
                    if (!updateSwingTrajectoryHistory(history, static_cast<std::uint32_t>(swingLeg.mFoot),
                            initial, goal, std::max(remaining, input.mFixedTimeStep), advance))
                        return reject("continuous swing history rejected");
                }
                else if (history)
                {
                    // This path plans zero terminal horizontal velocity/acceleration.
                    // Hold its committed endpoint until contact; do not chase a new
                    // placement with a one-step, unbounded acceleration request.
                    history->mElapsed = history->mSegment.mDuration;
                    placementModel.SetX(history->mSegment.mEnd.mPosition.GetX());
                    placementModel.SetY(history->mSegment.mEnd.mPosition.GetY());
                    plan.mPlacement = input.mRootPosition
                        + JPH::RVec3(input.mRootRotation * placementModel * input.mUniformScale);
                    plan.mPlacementLatched = true;
                }
                plan.mSwingTrajectoryHistory = history;
            }
            const auto continuousTarget = [&](JPH::Vec3 value, float ahead) -> std::optional<JPH::Vec3> {
                if (plan.mSwingTrajectoryHistory)
                {
                    const auto& history = *plan.mSwingTrajectoryHistory;
                    const auto sample = sampleSwingTrajectory(history.mSegment,
                        std::min(history.mElapsed + ahead, history.mSegment.mDuration));
                    if (!sample)
                        return std::nullopt;
                    value.SetX(sample->mPosition.GetX());
                    value.SetY(sample->mPosition.GetY());
                }
                return value;
            };
            footTarget = continuousTarget(*footTarget, 0.0f);
            if (!footTarget)
                return reject("continuous swing sample rejected");
            if (profile.mSwingReachFraction > 0.0f)
            {
                float legLength = 0.0f;
                if (swingLeg.mHingeGeometry)
                    legLength = swingLeg.mHingeGeometry->mUpperOffset.Length()
                        + swingLeg.mHingeGeometry->mLowerOffset.Length();
                else
                    legLength = (input.mBasePose[swingLeg.mKnee].mTranslation - input.mBasePose[swingLeg.mHip].mTranslation).Length()
                        + (input.mBasePose[swingLeg.mFoot].mTranslation - input.mBasePose[swingLeg.mKnee].mTranslation).Length();
                const float reach = legLength * profile.mSwingReachFraction;
                const JPH::Vec3 hip = input.mMeasuredPose[swingLeg.mHip].mTranslation;
                const JPH::Vec3 offset = *footTarget - hip;
                const float drop = -offset.GetZ();
                JPH::Vec3 planar(offset.GetX(), offset.GetY(), 0.0f);
                const float planarLength = planar.Length();
                if (reach > 0.0f && drop > 0.0f)
                {
                    const float maximumPlanar = drop < reach ? std::sqrt(reach * reach - drop * drop) : 0.0f;
                    if (planarLength > maximumPlanar)
                    {
                        planar = planarLength > 0.0f ? planar * (maximumPlanar / planarLength) : JPH::Vec3::sZero();
                        footTarget = hip + planar + JPH::Vec3(0.0f, 0.0f, offset.GetZ());
                        plan.mSwingReachLimited = true;
                    }
                }
            }
            plan.mSwingFootTarget = footTarget;
            placementReachLimited = plan.mSwingReachLimited;
            // The knee plane follows the pelvis' own heading, so the swing hip
            // never demands twist against the pelvis; only the pelvis hold
            // through the loaded hip corrects heading.
            JPH::Vec3 pelvisHeading = horizontal(telemetry.mBodies[input.mPelvis].mRotation
                * input.mBasePose[input.mPelvis].mRotation.Conjugated() * input.mRootRotation.Conjugated()
                * (basePelvisWorld * JPH::Vec3::sAxisY()));
            if (pelvisHeading.IsNearZero())
                pelvisHeading = forward;
            const JPH::Vec3 bend = (inverseRoot * pelvisHeading).NormalizedOr(JPH::Vec3::sAxisY());
            const TwoBoneLegTargetInput legInput{ swingLeg.mHip, swingLeg.mKnee, swingLeg.mFoot, *footTarget, bend,
                swingLeg.mHingeGeometry };
            GaitActuationDecision decision;
            decision.mPoseSource = GaitPoseSource::GaitCycle;
            const auto legPose = applySwingLegPose(target, input.mBasePose, input.mMeasuredPose, input.mPelvis,
                legInput, decision);
            if (!legPose)
                return reject("swing leg pose assembly rejected");
            plan.mSwingPoseApplied = legPose->mApplied;
            // IK can return a pose after shortening an unreachable request.
            // Report that limit even when the optional placement bound is off.
            plan.mSwingReachLimited = plan.mSwingReachLimited
                || (legPose->mApplied
                    && (legPose->mFootPosition - legPose->mFrame.mTargetFootPosition).Length() > 1e-3f);
            if (legPose->mApplied)
            {
                plan.mSwingHip = swingLeg.mHip;
                plan.mSwingHipWorldTarget = (input.mRootRotation * legPose->mHipModelRotation).Normalized();
                // Land the swing foot level at the requested heading, not at
                // the pelvis' current yaw, so each touchdown re-bases the
                // heading toward the request within the ankle twist range.
                const JPH::Quat desiredFootModel
                    = (inverseRoot * headingDelta * input.mRootRotation * input.mBasePose[swingLeg.mFoot].mRotation)
                          .Normalized();
                const JPH::Quat calfTargetToMeasured = (target[swingLeg.mKnee].mRotation
                    * input.mMeasuredPose[swingLeg.mKnee].mRotation.Conjugated()).Normalized();
                target[swingLeg.mFoot].mRotation = (calfTargetToMeasured * desiredFootModel).Normalized();
            }
            if ((profile.mSwingVelocityFeedForward || profile.mSwingInertiaFeedForward > 0.0f) && legPose->mApplied
                && input.mFixedTimeStep > 0.0f)
            {
                // Trajectory targets one and two steps ahead give the target
                // angular velocity (fed to the motor damping so it tracks the
                // moving target instead of braking it) and acceleration (fed
                // as computed torque through the load path).
                const auto solveAhead = [&](unsigned steps, std::vector<Transform>& pose)
                    -> std::optional<SwingLegPoseOutput> {
                    ReferenceSwingTrajectoryInput aheadInput = trajectoryInput;
                    aheadInput.mSwingTime += steps * input.mFixedTimeStep;
                    auto aheadTarget = calculateReferenceSwingTrajectory(aheadInput, profile);
                    if (aheadTarget)
                        aheadTarget = continuousTarget(*aheadTarget, steps * input.mFixedTimeStep);
                    if (!aheadTarget)
                        return std::nullopt;
                    pose.assign(target.begin(), target.end());
                    TwoBoneLegTargetInput aheadLegInput = legInput;
                    aheadLegInput.mTargetFootPosition = *aheadTarget;
                    auto solved = applySwingLegPose(pose, input.mBasePose, input.mMeasuredPose, input.mPelvis,
                        aheadLegInput, decision);
                    if (!solved || !solved->mApplied)
                        return std::nullopt;
                    return solved;
                };
                const auto rate = [&](JPH::QuatArg from, JPH::QuatArg to) {
                    JPH::Quat delta = (to * from.Conjugated()).Normalized();
                    if (delta.GetW() < 0.0f)
                        delta = -delta;
                    JPH::Vec3 axis;
                    float angle = 0.0f;
                    delta.GetAxisAngle(axis, angle);
                    return axis * (angle / input.mFixedTimeStep);
                };
                std::vector<Transform> nextPose, afterPose;
                const auto nextLeg = solveAhead(1, nextPose);
                const auto afterLeg = solveAhead(2, afterPose);
                if (nextLeg)
                {
                    const JPH::Quat hipNow = (input.mRootRotation * legPose->mHipModelRotation).Normalized();
                    const JPH::Quat hipNext = (input.mRootRotation * nextLeg->mHipModelRotation).Normalized();
                    const auto calfWorld = [&](std::span<const Transform> pose, JPH::QuatArg hipWorld) {
                        return (hipWorld * pose[swingLeg.mHip].mRotation.Conjugated() * pose[swingLeg.mKnee].mRotation)
                            .Normalized();
                    };
                    const JPH::Quat calfNow = calfWorld(target, hipNow);
                    const JPH::Quat calfNext = calfWorld(nextPose, hipNext);
                    const JPH::Vec3 hipRate = rate(hipNow, hipNext);
                    const JPH::Vec3 calfRate = rate(calfNow, calfNext);
                    const JPH::Vec3 kneeRate = calfRate - hipRate;
                    plan.mSwingHipRateTarget = hipRate;
                    plan.mSwingHipMeasuredRate = telemetry.mBodies[swingLeg.mHip].mMotion.mAngularVelocity;
                    if (profile.mSwingVelocityFeedForward && isFinite(hipRate) && isFinite(kneeRate))
                    {
                        plan.mJointAngularVelocityTargets.emplace_back(swingLeg.mHip, hipRate);
                        plan.mJointAngularVelocityTargets.emplace_back(swingLeg.mKnee, kneeRate);
                    }
                    const bool reachLimited = plan.mSwingReachLimited
                        || (nextLeg->mFootPosition - nextLeg->mFrame.mTargetFootPosition).Length() > 1e-3f
                        || (afterLeg && (afterLeg->mFootPosition - afterLeg->mFrame.mTargetFootPosition).Length() > 1e-3f);
                    if (profile.mSwingInertiaFeedForward > 0.0f && afterLeg && !reachLimited)
                    {
                        const JPH::Quat hipAfter = (input.mRootRotation * afterLeg->mHipModelRotation).Normalized();
                        const JPH::Quat calfAfter = calfWorld(afterPose, hipAfter);
                        const auto bounded = [&](JPH::Vec3Arg acceleration) {
                            const float limit = profile.mSwingFeedForwardAccelerationLimit;
                            const float length = acceleration.Length();
                            return limit > 0.0f && length > limit ? acceleration * (limit / length) : acceleration;
                        };
                        const JPH::Vec3 hipAcceleration = bounded((rate(hipNext, hipAfter) - hipRate) / input.mFixedTimeStep);
                        const JPH::Vec3 calfAcceleration = bounded((rate(calfNext, calfAfter) - calfRate) / input.mFixedTimeStep);
                        // Point-mass inertia of a chain about a joint, world axes.
                        const auto inertiaTorque = [&](std::initializer_list<std::size_t> chain,
                                                       JPH::RVec3Arg joint, JPH::Vec3Arg acceleration) {
                            JPH::Vec3 torque = JPH::Vec3::sZero();
                            for (const std::size_t body : chain)
                            {
                                const JPH::Vec3 r(telemetry.mBodies[body].mCenterOfMass - joint);
                                const float mass = telemetry.mBodies[body].mMass;
                                // I * a = m (|r|^2 a - r (r . a))
                                torque += mass * (r.LengthSq() * acceleration - r * r.Dot(acceleration));
                            }
                            return torque;
                        };
                        const JPH::Vec3 kneeTorque = profile.mSwingInertiaFeedForward
                            * inertiaTorque({ swingLeg.mKnee, swingLeg.mFoot }, bodies[swingLeg.mKnee].mJointPosition,
                                calfAcceleration);
                        const JPH::Vec3 hipTorque = profile.mSwingInertiaFeedForward
                                * inertiaTorque({ swingLeg.mHip, swingLeg.mKnee, swingLeg.mFoot },
                                    bodies[swingLeg.mHip].mJointPosition, hipAcceleration)
                            + kneeTorque;
                        if (isFinite(hipTorque) && isFinite(kneeTorque))
                        {
                            plan.mSwingHipFeedForwardTorque = hipTorque;
                            plan.mSwingKneeFeedForwardTorque = kneeTorque;
                            swingFeedForwardComputed = true;
                        }
                    }
                }
            }
            if (profile.mSwingLegFrequency > 0.0f)
            {
                plan.mSwingLegSpringBodies = { swingLeg.mHip, swingLeg.mKnee, swingLeg.mFoot };
                plan.mSwingLegFrequency = profile.mSwingLegFrequency;
                plan.mSwingLegDamping = profile.mSwingLegDamping;
            }
        }

        // 3b. Landing press. Through the double support entered by a touchdown
        //     the landed leg keeps a swing-style target at its latched landing
        //     point pressed below the landing height until the swing begins,
        //     so the leg loads the foot instead of holding its arrival pose.
        // Stationary correction has no timed next swing. Release its arrival
        // press within the normal maximum transfer window rather than keeping
        // an asymmetric loading target for an arbitrarily long idle period.
        const float landingPressWeight = stationary
            ? (profile.mMaximumDoubleSupportDuration > 0.0f
                    ? std::clamp(1.0f - input.mState.mTimeInStage / profile.mMaximumDoubleSupportDuration, 0.0f, 1.0f)
                    : 0.0f)
            : 1.0f;
        if (!swinging && input.mState.mEnteredFromTouchdown && profile.mLandingPressDepth > 0.0f
            && input.mLandingFootModel && landingPressWeight > 0.0f)
        {
            JPH::Vec3 footTarget = *input.mLandingFootModel;
            footTarget.SetZ(footTarget.GetZ() - landingPressWeight * profile.mLandingPressDepth / input.mUniformScale);
            JPH::Vec3 pelvisHeading = horizontal(telemetry.mBodies[input.mPelvis].mRotation
                * input.mBasePose[input.mPelvis].mRotation.Conjugated() * input.mRootRotation.Conjugated()
                * (basePelvisWorld * JPH::Vec3::sAxisY()));
            if (pelvisHeading.IsNearZero())
                pelvisHeading = forward;
            const JPH::Vec3 bend = (inverseRoot * pelvisHeading).NormalizedOr(JPH::Vec3::sAxisY());
            const TwoBoneLegTargetInput legInput{ stanceLeg.mHip, stanceLeg.mKnee, stanceLeg.mFoot, footTarget, bend,
                stanceLeg.mHingeGeometry };
            GaitActuationDecision decision;
            decision.mPoseSource = GaitPoseSource::GaitCycle;
            // The loaded-thigh hold has already modified target. Using that
            // rotation to pick the hinge-axis hemisphere can flip the landing
            // solve by pi during a turn. Compare a base-frame solve and select
            // the branch nearest the measured thigh, retaining the original
            // output for numerically equivalent solutions.
            std::vector<Transform> alternativeSeed(target.begin(), target.end());
            for (const auto body : { stanceLeg.mHip, stanceLeg.mKnee, stanceLeg.mFoot })
                alternativeSeed[body] = input.mBasePose[body];
            auto legPose = applySwingLegPose(target, input.mBasePose, input.mMeasuredPose, input.mPelvis,
                legInput, decision);
            if (legPose && legPose->mApplied)
            {
                const auto alternative = applySwingLegPose(alternativeSeed, input.mBasePose, input.mMeasuredPose,
                    input.mPelvis, legInput, decision);
                if (alternative && alternative->mApplied
                    && !alternative->mHipModelRotation.IsClose(legPose->mHipModelRotation, 1e-6f)
                    && !alternative->mHipModelRotation.IsClose(-legPose->mHipModelRotation, 1e-6f)
                    && std::abs(alternative->mHipModelRotation.Dot(input.mMeasuredPose[stanceLeg.mHip].mRotation))
                        > std::abs(legPose->mHipModelRotation.Dot(input.mMeasuredPose[stanceLeg.mHip].mRotation)))
                {
                    for (const auto body : { stanceLeg.mHip, stanceLeg.mKnee, stanceLeg.mFoot })
                        target[body] = alternativeSeed[body];
                    legPose = alternative;
                }
            }
            if (legPose && legPose->mApplied)
            {
                plan.mLandingPressApplied = true;
                plan.mSwingHip = stanceLeg.mHip;
                // The pressed leg tracks a fixed target: drop the hold's rate
                // target and loaded-hip spring so the swing spring applies.
                std::erase_if(plan.mJointAngularVelocityTargets,
                    [&](const auto& entry) { return entry.first == stanceLeg.mHip; });
                std::erase(plan.mStanceHipSpringBodies, stanceLeg.mHip);
                plan.mSwingHipWorldTarget = (input.mRootRotation * legPose->mHipModelRotation).Normalized();
                const JPH::Quat desiredFootModel
                    = (inverseRoot * headingDelta * input.mRootRotation * input.mBasePose[stanceLeg.mFoot].mRotation)
                          .Normalized();
                const JPH::Quat calfTargetToMeasured = (target[stanceLeg.mKnee].mRotation
                    * input.mMeasuredPose[stanceLeg.mKnee].mRotation.Conjugated()).Normalized();
                target[stanceLeg.mFoot].mRotation = (calfTargetToMeasured * desiredFootModel).Normalized();
                std::erase(plan.mLoadedAnkleBodies, stanceLeg.mFoot);
                plan.mSwingFootTarget = footTarget;
                if (profile.mSwingLegFrequency > 0.0f)
                {
                    plan.mSwingLegSpringBodies = { stanceLeg.mHip, stanceLeg.mKnee, stanceLeg.mFoot };
                    plan.mSwingLegFrequency = profile.mSwingLegFrequency;
                    plan.mSwingLegDamping = profile.mSwingLegDamping;
                }
            }
        }

        const bool coordinateInitialTransfer = !swinging && (profile.mCoordinateWeightTransfers
            || (profile.mCoordinateInitialTransfer && input.mState.mInitialTransfer));
        if (input.mCoordinateLegPose || coordinateInitialTransfer)
        {
            plan.mCoordinatedPoseAttempted = true;
            auto pelvis = input.mMeasuredPose[input.mPelvis];
            pelvis.mRotation = target[input.mPelvis].mRotation;
            if (input.mPreferredPelvisCenterHeight && !coordinateInitialTransfer)
            {
                const auto& observed = telemetry.mBodies[input.mPelvis];
                const auto localCenter = observed.mRotation.Conjugated()
                    * JPH::Vec3(observed.mCenterOfMass - observed.mPosition);
                auto origin = input.mRootPosition + input.mRootRotation * pelvis.mTranslation * input.mUniformScale;
                origin.SetZ(input.mSupport.mGroundCenter.GetZ() + *input.mPreferredPelvisCenterHeight
                    - (desiredPelvis * localCenter).GetZ());
                pelvis.mTranslation = toModel(origin);
            }
            auto leftFoot = input.mMeasuredPose[input.mLeftLeg.mFoot].mTranslation;
            auto rightFoot = input.mMeasuredPose[input.mRightLeg.mFoot].mTranslation;
            std::optional<JPH::Quat> leftRotation = input.mMeasuredPose[input.mLeftLeg.mFoot].mRotation;
            std::optional<JPH::Quat> rightRotation = input.mMeasuredPose[input.mRightLeg.mFoot].mRotation;
            if (swinging && plan.mSwingFootTarget)
            {
                (leftStance ? rightFoot : leftFoot) = *plan.mSwingFootTarget;
                // The airborne foot may orient within its native ankle limits.
                (leftStance ? rightRotation : leftRotation).reset();
            }
            std::optional<ReferenceStancePosture> posture;
            if (coordinateInitialTransfer)
            {
                std::vector<ReferencePostureMass> masses;
                masses.reserve(count);
                for (const auto& body : telemetry.mBodies)
                    masses.push_back({ body.mMass, body.mRotation.Conjugated()
                        * JPH::Vec3(body.mCenterOfMass - body.mPosition) / input.mUniformScale });
                // Backtrack the COM shift when the full stance-center request
                // is outside the joint-valid planted-foot workspace.
                for (const float fraction : { 1.0f, 0.5f, 0.25f, 0.125f, 0.0f })
                {
                    const auto desiredCenter = toModel(telemetry.mCenterOfMass + right * (lateralOffset * fraction));
                    posture = planBalancedReferenceStancePosture(target, masses, input.mPelvis,
                        input.mLeftLeg, input.mRightLeg, pelvis,
                        input.mMeasuredPose[input.mLeftLeg.mFoot], input.mMeasuredPose[input.mRightLeg.mFoot],
                        desiredCenter, inverseRoot * forward, true);
                    if (posture)
                        break;
                }
            }
            else
                posture = planJointLimitedReferenceStancePosture(input.mBasePose, input.mPelvis,
                    input.mLeftLeg, input.mRightLeg, pelvis, leftFoot, rightFoot,
                    leftRotation, rightRotation, inverseRoot * forward);
            if (posture)
            {
                const auto& coordinated = posture->mPose.mBodyTransforms;
                const auto shift = coordinated[input.mPelvis].mTranslation - target[input.mPelvis].mTranslation;
                for (auto& transform : target) transform.mTranslation += shift;
                for (const auto body : { input.mPelvis, input.mLeftLeg.mHip, input.mLeftLeg.mKnee,
                         input.mLeftLeg.mFoot, input.mRightLeg.mHip, input.mRightLeg.mKnee, input.mRightLeg.mFoot })
                    target[body] = coordinated[body];
                // The coordinated pose specifies relative joints in ONE pelvis
                // frame. Freezing a hip/foot world target against its measured
                // parent would replace those verified relative orientations.
                plan.mSwingHipWorldTarget.reset();
                std::erase_if(plan.mJointAngularVelocityTargets, [&](const auto& entry) {
                    return entry.first == input.mLeftLeg.mHip || entry.first == input.mRightLeg.mHip
                        || entry.first == input.mLeftLeg.mKnee || entry.first == input.mRightLeg.mKnee
                        || entry.first == input.mLeftLeg.mFoot || entry.first == input.mRightLeg.mFoot;
                });
                std::erase_if(plan.mWorldFootTargets, [&](const auto& entry) {
                    return entry.first == input.mLeftLeg.mFoot || entry.first == input.mRightLeg.mFoot;
                });
                plan.mCoordinatedPoseApplied = true;
                if (swinging && plan.mSwingFootTarget)
                {
                    plan.mSwingHip = swingLeg.mHip;
                    plan.mSwingPoseApplied = true;
                    plan.mSwingReachLimited = placementReachLimited;
                }
            }
            // Conservative geometry failure retains the original motor plan.
            // The attempted/applied flags expose this fallback to diagnostics.
        }

        // Feed-forward was differentiated from the independent swing pose.
        // A coordinated replacement has different joint trajectories, so those
        // rates and torques (including their stance reaction) are not valid.
        // Defer adding them rather than adding/subtracting and losing precision.
        if (plan.mCoordinatedPoseApplied)
        {
            plan.mSwingHipFeedForwardTorque = JPH::Vec3::sZero();
            plan.mSwingKneeFeedForwardTorque = JPH::Vec3::sZero();
            plan.mSwingHipRateTarget = JPH::Vec3::sZero();
        }
        else if (swingFeedForwardComputed)
        {
            plan.mLoadTorques[swingLeg.mKnee] += plan.mSwingKneeFeedForwardTorque;
            plan.mLoadTorques[swingLeg.mHip] += plan.mSwingHipFeedForwardTorque;
            plan.mLoadTorques[stanceLeg.mHip] -= plan.mSwingHipFeedForwardTorque;
        }

        // 4. Root reaction accounting. The loaded hip springs hold the pelvis;
        //    the measured dynamic torques of the other root-attached joints
        //    are cancelled on the loaded hips by load fraction.
        const JPH::Quat pelvisRotation = telemetry.mBodies[input.mPelvis].mRotation;
        const JPH::Quat error = (desiredPelvis * pelvisRotation.Conjugated()).Normalized().EnsureWPositive();
        JPH::Vec3 axis;
        float angle = 0.0f;
        error.GetAxisAngle(axis, angle);
        plan.mPelvisOrientationError = axis * angle;

        ReferenceRootRoutingInput routing;
        if (swinging)
        {
            routing.mLeftHipLoadFraction = leftStance ? 1.0f : 0.0f;
            routing.mRightHipLoadFraction = leftStance ? 0.0f : 1.0f;
        }
        else if (leftLoad.mKnown && rightLoad.mKnown && leftLoad.mNormalImpulse + rightLoad.mNormalImpulse > 0.0)
        {
            routing.mLeftHipLoadFraction = static_cast<float>(leftLoad.mNormalImpulse);
            routing.mRightHipLoadFraction = static_cast<float>(rightLoad.mNormalImpulse);
        }
        else
        {
            routing.mLeftHipLoadFraction = leftLoad.mSupported ? 1.0f : 0.0f;
            routing.mRightHipLoadFraction = rightLoad.mSupported ? 1.0f : 0.0f;
        }
        const bool haveHistory = !input.mPreviousLoadTorques.empty() && !input.mPreviousMotorTorques.empty();
        for (std::size_t i = 1; i < count; ++i)
        {
            if (parents[i] != static_cast<int>(input.mPelvis))
                continue;
            const bool routingHip = i == stanceLeg.mHip || (!swinging && i == swingLeg.mHip);
            if (routingHip || !haveHistory)
                continue;
            routing.mBranchDynamicTorques.push_back(input.mPreviousMotorTorques[i] - input.mPreviousLoadTorques[i]);
        }
        const auto routed = routeReferenceRootTorque(routing);
        if (!routed)
            return reject("root routing rejected");
        plan.mBranchReaction = routed->mBranchReaction;
        const double loadSum = static_cast<double>(routing.mLeftHipLoadFraction) + routing.mRightHipLoadFraction;
        plan.mLeftLoadFraction = loadSum > 0.0 ? static_cast<float>(routing.mLeftHipLoadFraction / loadSum) : 0.0f;
        plan.mRightLoadFraction = loadSum > 0.0 ? static_cast<float>(routing.mRightHipLoadFraction / loadSum) : 0.0f;
        plan.mLoadTorques[input.mLeftLeg.mHip] += routed->mLeftHipTorque;
        plan.mLoadTorques[input.mRightLeg.mHip] += routed->mRightHipTorque;
        plan.mStanceHipTorque = plan.mLoadTorques[stanceLeg.mHip];
        for (const JPH::Vec3& torque : plan.mLoadTorques)
            if (!isFinite(torque))
                return reject("non-finite load torque");

        // 4.5. Active ankle push-off. In the last fraction of single support
        //      before lift, rotate the stance ankle's target toward plantar
        //      flexion about the leg's lateral (knee-hinge) axis, expressed
        //      in the calf frame; in double support the trailing (about to
        //      lift) ankle gets the same offset ramped over the transfer.
        //      The loaded ankle is otherwise a pure angle-holder that never
        //      pushes; this offset shifts the soft spring's equilibrium, so
        //      the existing motor authority does the work -- no torque or
        //      limit change. Placed after every pose override (E13) and
        //      before the feasibility gate, which still bounds the final
        //      command. Skipped unless the ankle is under the soft holder
        //      (a landing press or a world foot target owns it instead).
        if (profile.mStanceAnklePushOff > 0.0f && profile.mSwingDuration > 0.0f
            && profile.mDoubleSupportDuration > 0.0f)
        {
            const auto pushOff = [&](const ReferenceGaitLeg& leg, float ramp) {
                if (ramp <= 0.0f || !leg.mHingeGeometry
                    || std::find(plan.mLoadedAnkleBodies.begin(), plan.mLoadedAnkleBodies.end(), leg.mFoot)
                           == plan.mLoadedAnkleBodies.end())
                    return;
                const JPH::Quat relativeBase = (input.mBasePose[leg.mHip].mRotation.Conjugated()
                    * input.mBasePose[leg.mKnee].mRotation).Normalized();
                const JPH::Vec3 axisKnee
                    = (relativeBase.Conjugated() * leg.mHingeGeometry->mHingeAxis).NormalizedOr(JPH::Vec3::sZero());
                if (axisKnee.IsNearZero())
                    return;
                // Same rotational sense as the knee flexion command: a
                // negative angle about the lateral axis points the toes
                // down (plantar flexion).
                const JPH::Quat offset = JPH::Quat::sRotation(axisKnee, -profile.mStanceAnklePushOff * ramp);
                target[leg.mFoot].mRotation = (target[leg.mKnee].mRotation * offset
                    * target[leg.mKnee].mRotation.Conjugated() * target[leg.mFoot].mRotation).Normalized();
            };
            if (swinging)
            {
                const float phase = input.mState.mTimeInStage / profile.mSwingDuration;
                if (phase >= profile.mStanceAnklePushOffStartFraction)
                    pushOff(stanceLeg, smoothStep((phase - profile.mStanceAnklePushOffStartFraction)
                        / (1.0f - profile.mStanceAnklePushOffStartFraction)));
            }
            else if (profile.mStanceAnklePushOffTrailing)
            {
                // Terminal-stance push: the trailing leg unloads over the
                // double support and lifts at its end.
                pushOff(swingLeg, smoothStep(input.mState.mTimeInStage / profile.mDoubleSupportDuration));
            }
        }

        // 5. Final-command feasibility gate. Every override
        //    -- the rate-consistent thigh hold, the landing press and the
        //    coordinated pose -- is complete at this point, so clamping here
        //    acts on the command the motors actually receive; a feasibility
        //    query placed upstream is invalidated by those overrides. Each
        //    driven hip/ankle target with known SwingTwist geometry is
        //    projected onto the authored cone shrunken by the role's margin
        //    (cone half-angles and twist range), keeping Jolt's
        //    ClampSwingTwist boundary projection and its twist preservation.
        //    World-submitted targets are evaluated against the planned
        //    parent's world frame. The stance role uses the strict margin,
        //    the airborne swing role the lenient one; in double support both
        //    legs are loaded and take the stance margin. A zero margin
        //    disables that role's gate and leaves its command bit-identical.
        //    When a coordinated pose was applied, the targets were PRODUCED
        //    by the joint-limited posture solve, so the margin gate would
        //    only double-shrink an already validated command; skip it.
        if (!plan.mCoordinatedPoseApplied
            && (profile.mFinalCommandFeasibilityStanceMargin > 0.0f
                || profile.mFinalCommandFeasibilitySwingMargin > 0.0f))
        {
            const auto clampedToMargin = [&](const SwingTwistJointGeometry& geometry, float margin,
                                             JPH::QuatArg parent, JPH::QuatArg child, JPH::Quat& result) {
                if (margin <= 0.0f)
                    return false;
                SwingTwistJointGeometry shrunk = geometry;
                shrunk.mNormalHalfConeAngle = std::max(0.0f, geometry.mNormalHalfConeAngle - margin);
                shrunk.mPlaneHalfConeAngle = std::max(0.0f, geometry.mPlaneHalfConeAngle - margin);
                shrunk.mTwistMinimum = geometry.mTwistMinimum + margin;
                shrunk.mTwistMaximum = geometry.mTwistMaximum - margin;
                if (shrunk.mTwistMinimum > shrunk.mTwistMaximum)
                {
                    // A margin wider than the twist range leaves only its center.
                    shrunk.mTwistMinimum = shrunk.mTwistMaximum
                        = 0.5f * (geometry.mTwistMinimum + geometry.mTwistMaximum);
                }
                const auto clamped = clampSwingTwistJointOrientation(shrunk, parent, child);
                // Invalid geometry or telemetry skips the joint: the gate never
                // rejects a plan the baseline would have accepted.
                if (!clamped || clamped->IsClose(child, 1e-10f) || clamped->IsClose(-child, 1e-10f))
                    return false;
                result = *clamped;
                return true;
            };
            const auto gateLeg = [&](const ReferenceGaitLeg& leg, float margin) {
                JPH::Quat clamped;
                if (leg.mHipGeometry)
                {
                    // A world hip target overrides the relative pose target at
                    // the motor; gate the command that actually reaches it.
                    if (plan.mSwingHip && *plan.mSwingHip == leg.mHip && plan.mSwingHipWorldTarget)
                    {
                        if (clampedToMargin(*leg.mHipGeometry, margin,
                                (input.mRootRotation * target[input.mPelvis].mRotation).Normalized(),
                                *plan.mSwingHipWorldTarget, clamped))
                        {
                            plan.mSwingHipWorldTarget = clamped;
                            ++plan.mFeasibilityClampedCount;
                        }
                    }
                    else if (clampedToMargin(*leg.mHipGeometry, margin, target[input.mPelvis].mRotation,
                                 target[leg.mHip].mRotation, clamped))
                    {
                        // A clipped relative hip target can turn a yaw request
                        // into pelvis tilt over the planted thigh. Preserve an
                        // already upright-restoring command; otherwise seek
                        // feasible yaw progress without changing tilt priority.
                        // Native A52 qualification covers the opt-in candidate;
                        // broader regressions/FPS qualification remain separate.
                        const auto projectedParent = (telemetry.mBodies[leg.mHip].mRotation
                            * (target[input.mPelvis].mRotation.Conjugated() * clamped).Conjugated()).Normalized();
                        const float projectedUp = ((projectedParent * desiredPelvis.Conjugated()).Normalized()
                            * JPH::Vec3::sAxisZ()).GetZ();
                        const float measuredUp = ((telemetry.mBodies[input.mPelvis].mRotation
                            * desiredPelvis.Conjugated()).Normalized() * JPH::Vec3::sAxisZ()).GetZ();
                        if (RagdollCore::tuningText("OPENMW_RAGDOLL_DIRECTED_HIP_YAW") && swinging
                            && leg.mHip == stanceLeg.mHip && projectedUp < measuredUp - 1e-6f)
                        {
                            const auto held = target[leg.mHip].mRotation;
                            const auto parent = target[input.mPelvis].mRotation;
                            const auto measuredHeading = horizontal(telemetry.mBodies[input.mPelvis].mRotation
                                * JPH::Vec3::sAxisY()).NormalizedOr(facing);
                            const float gap = std::atan2(facing.Cross(measuredHeading).GetZ(), facing.Dot(measuredHeading));
                            const auto parentAt = [&](float remaining) {
                                return (inverseRoot * JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), gap * remaining)
                                    * input.mRootRotation * parent).Normalized();
                            };
                            const auto feasible = [&](float remaining) {
                                JPH::Quat ignored;
                                return !clampedToMargin(*leg.mHipGeometry, margin, parentAt(remaining), held, ignored);
                            };
                            for (int sample = 1; sample <= 64; ++sample)
                            {
                                float high = sample / 64.0f;
                                if (!feasible(high)) continue;
                                float low = (sample - 1) / 64.0f;
                                for (int iteration = 0; iteration < 16; ++iteration)
                                {
                                    const float mid = 0.5f * (low + high);
                                    if (feasible(mid)) high = mid;
                                    else low = mid;
                                }
                                const auto next = (parent * parentAt(high).Conjugated() * held).Normalized();
                                // Preserve the baseline's post-gate relative knee/ankle commands.
                                const auto delta = (next * clamped.Conjugated()).Normalized();
                                for (const auto body : { leg.mKnee, leg.mFoot })
                                    target[body].mRotation = (delta * target[body].mRotation).Normalized();
                                clamped = next;
                                break;
                            }
                        }
                        target[leg.mHip].mRotation = clamped;
                        ++plan.mFeasibilityClampedCount;
                    }
                }
                if (leg.mAnkleGeometry)
                {
                    const auto worldFoot = std::find_if(plan.mWorldFootTargets.begin(), plan.mWorldFootTargets.end(),
                        [&](const auto& entry) { return entry.first == leg.mFoot; });
                    if (worldFoot != plan.mWorldFootTargets.end())
                    {
                        if (clampedToMargin(*leg.mAnkleGeometry, margin,
                                (input.mRootRotation * target[leg.mKnee].mRotation).Normalized(), worldFoot->second,
                                clamped))
                        {
                            worldFoot->second = clamped;
                            ++plan.mFeasibilityClampedCount;
                        }
                    }
                    else if (clampedToMargin(*leg.mAnkleGeometry, margin, target[leg.mKnee].mRotation,
                                 target[leg.mFoot].mRotation, clamped))
                    {
                        target[leg.mFoot].mRotation = clamped;
                        ++plan.mFeasibilityClampedCount;
                    }
                }
            };
            // In single support the stance leg carries the load and takes the
            // strict margin; the airborne swing leg keeps the lenient one.
            // In double support both legs are loaded, so both are strict.
            gateLeg(stanceLeg, profile.mFinalCommandFeasibilityStanceMargin);
            gateLeg(swingLeg,
                swinging ? profile.mFinalCommandFeasibilitySwingMargin : profile.mFinalCommandFeasibilityStanceMargin);
        }

        for (const Transform& transform : target)
            if (!isFinite(transform.mTranslation) || !isFinite(transform.mRotation))
                return reject("non-finite target pose");
        return plan;
    }
}
