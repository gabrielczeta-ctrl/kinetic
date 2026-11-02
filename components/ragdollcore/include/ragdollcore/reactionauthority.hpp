#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_REACTIONAUTHORITY_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_REACTIONAUTHORITY_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <ragdollcore/profile.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>

namespace RagdollCore
{
    // RXN-01 offline pose authority and its deviation-capped blend. This is
    // separate from native BalanceAuthority's generation-bound leg correction;
    // the native reaction adapter does not consume this helper's pose blend.
    //
    // The kinematic route keeps the gameplay actor in charge of locomotion;
    // this layer only exists so the body can do balance work of its own when
    // the harness is lying: a strong disturbance, or actual loss of support.
    // At authority zero the clip targets pass through untouched.
    struct ReactionAuthorityProfile
    {
        // Disturbance ramp: an impulse at or above the engage value engages
        // the balance layer; full authority at the full value, model units.
        float mImpulseEngage = 800.0f;
        float mImpulseFull = 2500.0f;
        // Support-loss ramp: seconds without any foot carrying the body.
        float mSupportLossEngage = 0.05f;
        float mSupportLossFull = 0.25f;
        // Baselines. Standing keeps half the authority for balance posture and
        // ordinary walking a quarter for placement bias; zero makes the layer
        // purely disturbance-driven.
        float mStandingAuthority = 0.5f;
        float mWalkingAuthority = 0.25f;
        // How far the blended pose may deviate from the clip at authority 1.
        float mMaximumAnimationDeviationDegrees = 8.0f;
        float mMaximumAnimationDeviationUnits = 3.0f;
        // Authority attack (rising) and release (falling) rates, 1/s.
        float mAttackRate = 6.0f;
        float mReleaseRate = 1.5f;
        // A hit keeps disturbing the body for this long before the retained
        // magnitude decays, so the response cannot snap back within the step
        // that received it.
        float mDisturbanceRetention = 0.35f;
        float mDisturbanceDecayRate = 1.0f;
    };

    // Disturbance retention: latches a hit magnitude for the retention window,
    // then decays it. Shared by the offline tester and the eventual adapter.
    struct ReactionDisturbanceLatch
    {
        float mRemaining = 0.0f;
        float mMagnitude = 0.0f;
        void begin(float magnitude, const ReactionAuthorityProfile& profile)
        {
            if (!std::isfinite(magnitude) || magnitude <= 0.0f)
                return;
            mMagnitude = std::max(mMagnitude, magnitude);
            mRemaining = std::max(mRemaining, std::max(0.0f, profile.mDisturbanceRetention));
        }
        void step(float duration, const ReactionAuthorityProfile& profile)
        {
            if (!std::isfinite(duration) || duration <= 0.0f)
                return;
            if (mRemaining > 0.0f)
            {
                mRemaining = std::max(0.0f, mRemaining - duration);
                return;
            }
            const float rate = std::isfinite(profile.mDisturbanceDecayRate)
                ? std::max(0.0f, profile.mDisturbanceDecayRate) : 0.0f;
            // Exponential decay: a retained hit fades over a few time
            // constants instead of lingering for magnitude/rate seconds.
            mMagnitude *= std::exp(-rate * duration);
            if (mMagnitude < 1.0f)
                mMagnitude = 0.0f;
        }
        float retained() const { return mMagnitude; }
        void reset() { *this = ReactionDisturbanceLatch{}; }
    };

    struct ReactionAuthorityState
    {
        float mAuthority = 0.0f;
        // Triggering disturbance level of the last step, for telemetry.
        float mDisturbance = 0.0f;
        float mUnsupportedTime = 0.0f;
        void reset() { *this = ReactionAuthorityState{}; }
    };

    struct ReactionAuthorityInput
    {
        bool mSupportEstablished = true;
        bool mBothFeetSupported = false;
        bool mWalking = false;
        // Strongest impulse applied to the body this step, model units.
        float mDisturbanceImpulse = 0.0f;
        float mDuration = 0.0f;
    };

    // Smooth 0..1 ramp between two thresholds. Non-finite values clamp to 0,
    // and an inverted or empty span becomes a step at the engage value.
    float reactionAuthorityRamp(float value, float engage, float full);

    // Advances the authority one fixed step. The target is the strongest of
    // the standing/walking baseline, the impulse ramp and the support-loss
    // ramp; the state moves toward it at the attack or release rate.
    float stepReactionAuthority(ReactionAuthorityState& state, const ReactionAuthorityInput& input,
        const ReactionAuthorityProfile& profile);

    // Blends every transform between the clip pose and the balance pose, then
    // clamps the result so it can never sit farther from the clip than the
    // profile allows at the current authority. Mismatched spans or non-finite
    // inputs leave the clip untouched.
    void blendReactionBalancePose(std::span<const Transform> clip, std::span<const Transform> balance,
        float authority, const ReactionAuthorityProfile& profile, std::span<Transform> out);
}

#endif
