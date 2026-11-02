#include <ragdollcore/reactionauthority.hpp>

namespace RagdollCore
{
    namespace
    {
        bool isFiniteTransform(const Transform& transform)
        {
            return std::isfinite(transform.mTranslation.GetX())
                && std::isfinite(transform.mTranslation.GetY())
                && std::isfinite(transform.mTranslation.GetZ())
                && std::isfinite(transform.mRotation.GetX()) && std::isfinite(transform.mRotation.GetY())
                && std::isfinite(transform.mRotation.GetZ()) && std::isfinite(transform.mRotation.GetW());
        }

        float rotationAngle(JPH::QuatArg first, JPH::QuatArg second)
        {
            const JPH::Quat relative = (first * second.Conjugated()).Normalized();
            return 2.0f * std::atan2(relative.GetXYZ().Length(), std::abs(relative.GetW()));
        }
    }

    float reactionAuthorityRamp(float value, float engage, float full)
    {
        if (!std::isfinite(value) || !std::isfinite(engage) || !std::isfinite(full)
            || value <= engage)
            return 0.0f;
        if (full <= engage)
            return 1.0f;
        if (value >= full)
            return 1.0f;
        const float x = (value - engage) / (full - engage);
        return x * x * (3.0f - 2.0f * x);
    }

    float stepReactionAuthority(ReactionAuthorityState& state, const ReactionAuthorityInput& input,
        const ReactionAuthorityProfile& profile)
    {
        const float duration = std::isfinite(input.mDuration) && input.mDuration > 0.0f ? input.mDuration : 0.0f;
        if (input.mSupportEstablished)
            state.mUnsupportedTime = 0.0f;
        else
            state.mUnsupportedTime += duration;

        const float baseline = !input.mSupportEstablished ? 0.0f
            : input.mBothFeetSupported ? std::clamp(profile.mStandingAuthority, 0.0f, 1.0f)
                                       : std::clamp(profile.mWalkingAuthority, 0.0f, 1.0f);
        const float impulse = reactionAuthorityRamp(input.mDisturbanceImpulse, profile.mImpulseEngage, profile.mImpulseFull);
        const float loss = reactionAuthorityRamp(state.mUnsupportedTime, profile.mSupportLossEngage, profile.mSupportLossFull);
        const float disturbance = std::max(impulse, loss);
        const float target = std::clamp(std::max(baseline, disturbance), 0.0f, 1.0f);

        const float rate = target > state.mAuthority ? profile.mAttackRate : profile.mReleaseRate;
        if (std::isfinite(rate) && rate > 0.0f)
        {
            const float step = rate * duration;
            if (target > state.mAuthority)
                state.mAuthority = std::min(target, state.mAuthority + step);
            else
                state.mAuthority = std::max(target, state.mAuthority - step);
        }
        else
            state.mAuthority = target;
        state.mDisturbance = disturbance;
        return state.mAuthority;
    }

    void blendReactionBalancePose(std::span<const Transform> clip, std::span<const Transform> balance,
        float authority, const ReactionAuthorityProfile& profile, std::span<Transform> out)
    {
        if (clip.size() != balance.size() || out.size() != clip.size())
        {
            for (std::size_t i = 0; i < out.size() && i < clip.size(); ++i)
                out[i] = clip[i];
            return;
        }
        const float weight = std::isfinite(authority) ? std::clamp(authority, 0.0f, 1.0f) : 0.0f;
        const float maximumAngle = std::max(0.0f, profile.mMaximumAnimationDeviationDegrees)
            * 3.14159265358979323846f / 180.0f * weight;
        const float maximumDistance = std::max(0.0f, profile.mMaximumAnimationDeviationUnits) * weight;
        for (std::size_t i = 0; i < out.size(); ++i)
        {
            const Transform& from = clip[i];
            const Transform& to = balance[i];
            if (!isFiniteTransform(from) || !isFiniteTransform(to))
            {
                out[i] = from;
                continue;
            }
            float rotationWeight = weight;
            const float fullAngle = rotationAngle(from.mRotation, to.mRotation);
            if (fullAngle > 1e-5f)
                rotationWeight = std::min(rotationWeight, maximumAngle / fullAngle);
            float translationWeight = weight;
            const float fullDistance = (to.mTranslation - from.mTranslation).Length();
            if (fullDistance > 1e-5f)
                translationWeight = std::min(translationWeight, maximumDistance / fullDistance);
            Transform blended;
            blended.mRotation = from.mRotation.SLERP(to.mRotation, rotationWeight).Normalized();
            blended.mTranslation = from.mTranslation
                + (to.mTranslation - from.mTranslation) * translationWeight;
            out[i] = blended;
        }
    }
}
