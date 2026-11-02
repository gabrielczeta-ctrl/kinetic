#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_REACTIONMECHANICS_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_REACTIONMECHANICS_H

// RXN-01 mechanical slice: measured support/slip classification, hinge-target
// clamping and mirror checks (the right-knee class of defect), disturbance
// retention, and the composition of clip + reaction offsets + bounded balance
// correction into motor targets. All functions are pure and offline-testable.

#include <ragdollcore/reactionauthority.hpp>
#include <ragdollcore/reactionpipeline.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>

namespace RagdollCore
{
    struct FootObservation
    {
        bool mSupported = false;
        float mHeight = 0.0f;          // model units above the support plane
        float mVerticalSpeed = 0.0f;   // model units/s
        float mHorizontalSpeed = 0.0f; // model units/s
    };

    struct SupportSlipProfile
    {
        // A foot moving faster than these is not a planted support, whatever
        // the contact manifold says.
        float mMaximumPlantedHorizontalSpeed = 2.0f;
        float mMaximumPlantedVerticalSpeed = 3.0f;
        float mMinimumLiftHeight = 0.5f;
    };

    struct SupportSlipMeasurement
    {
        bool mLeftPlanted = false;
        bool mRightPlanted = false;
        int mPlantedCount = 0;
        // Worst horizontal speed among planted feet; the stance slide.
        float mStanceSlip = 0.0f;
        // Height of the highest foot that is not planted; the lift.
        float mSwingLift = 0.0f;
    };

    // Contact alone is not support: a sliding or bouncing manifold point must
    // not count as planted. Mirrors the harness rule used by the reference
    // walker, exposed here for the offline fixtures.
    inline SupportSlipMeasurement measureSupportSlip(const FootObservation& left,
        const FootObservation& right, const SupportSlipProfile& profile)
    {
        SupportSlipMeasurement result;
        const auto planted = [&](const FootObservation& foot) {
            return foot.mSupported
                && std::abs(foot.mHorizontalSpeed) <= profile.mMaximumPlantedHorizontalSpeed
                && std::abs(foot.mVerticalSpeed) <= profile.mMaximumPlantedVerticalSpeed;
        };
        result.mLeftPlanted = planted(left);
        result.mRightPlanted = planted(right);
        result.mPlantedCount = (result.mLeftPlanted ? 1 : 0) + (result.mRightPlanted ? 1 : 0);
        if (result.mLeftPlanted)
            result.mStanceSlip = std::max(result.mStanceSlip, std::abs(left.mHorizontalSpeed));
        if (result.mRightPlanted)
            result.mStanceSlip = std::max(result.mStanceSlip, std::abs(right.mHorizontalSpeed));
        if (!result.mLeftPlanted && left.mHeight >= profile.mMinimumLiftHeight)
            result.mSwingLift = std::max(result.mSwingLift, left.mHeight);
        if (!result.mRightPlanted && right.mHeight >= profile.mMinimumLiftHeight)
            result.mSwingLift = std::max(result.mSwingLift, right.mHeight);
        return result;
    }

    struct HingeClampProfile
    {
        // The target is kept this far inside the anatomical limit so the
        // motor never sits saturated against it (the right-knee defect).
        float mMargin = 0.02f;
    };

    // Clamps a hinge target into [minimum + margin, maximum - margin] and
    // reports how much angle was removed. Non-finite input returns the middle
    // of the safe band with a zero delta.
    inline float clampHingeTarget(float target, float minimum, float maximum,
        const HingeClampProfile& profile, float& removed)
    {
        removed = 0.0f;
        if (!std::isfinite(target) || !std::isfinite(minimum) || !std::isfinite(maximum)
            || maximum < minimum)
            return 0.0f;
        const float margin = std::max(0.0f, profile.mMargin);
        const float low = minimum + margin;
        const float high = maximum - margin;
        if (high < low)
        {
            const float middle = 0.5f * (minimum + maximum);
            removed = target - middle;
            return middle;
        }
        const float clamped = std::clamp(target, low, high);
        removed = target - clamped;
        return clamped;
    }

    struct KneeMirrorCheck
    {
        float mDelta = 0.0f;
        bool mAsymmetric = false;
    };

    // Mirrored left/right hinge targets must agree within tolerance. This is
    // the offline probe for the sign/clamp asymmetry that pins one knee.
    inline KneeMirrorCheck compareMirroredHingeTargets(float leftTarget, float rightTarget,
        float tolerance = 0.05f)
    {
        KneeMirrorCheck result;
        if (!std::isfinite(leftTarget) || !std::isfinite(rightTarget))
        {
            result.mAsymmetric = true;
            return result;
        }
        result.mDelta = leftTarget - rightTarget;
        result.mAsymmetric = std::abs(result.mDelta) > std::abs(tolerance);
        return result;
    }

    struct ReactionComposeProfile
    {
        ReactionAuthorityProfile mAuthority;
    };

    // Composition order: start from the clip, apply the bounded reaction pose
    // offsets, then blend toward the balance pose with the pose-correction
    // authority and its deviation cap. Mismatched spans leave out untouched.
    bool composeReactionTargets(std::span<const Transform> clip, std::span<const Transform> balance,
        const ReactionOutputs& outputs, const ReactionPoseTargets& targets, std::span<Transform> out,
        const ReactionComposeProfile& profile);
}

#endif
