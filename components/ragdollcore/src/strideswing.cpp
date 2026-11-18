#include <ragdollcore/strideswing.hpp>

#include <ragdollcore/tuning.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace RagdollCore
{
    float strideSwingReach()
    {
        static const auto reach_tuned = tuned<float>([] {
            const char* text = tuningText("OPENMW_RAGDOLL_K2_STRIDE_SWING");
            const float value = text ? static_cast<float>(std::atof(text)) : 0.0f;
            return std::isfinite(value) ? std::clamp(value, 0.0f, 1.5f) : 0.0f;
        });
        return reach_tuned.get();
    }

    std::optional<StrideSwingOutput> computeStrideSwing(const StrideSwingInput& input)
    {
        const auto& s = input.mSettings;
        if (!(input.mGravity > 0.0f) || !std::isfinite(input.mPelvisVelocity.mX)
            || !std::isfinite(input.mPelvisVelocity.mY) || !(s.mReach >= 0.0f))
            return std::nullopt;
        StrideSwingOutput out;
        if (!(s.mFrequency > 0.0f))
            return out;
        for (std::size_t side = 0; side < 2; ++side)
        {
            const StrideSwingFoot& foot = input.mFeet[side];
            if (!foot.mSwinging)
                continue;
            if (!std::isfinite(foot.mLandingHeight) || !std::isfinite(foot.mHip[0]) || !std::isfinite(foot.mHip[1])
                || !std::isfinite(foot.mHip[2]) || !std::isfinite(foot.mLegLength) || foot.mLegLength < 0.0f)
                return std::nullopt;
            SwingSpringInput spring;
            spring.mFoot = foot.mFoot;
            spring.mFootVelocity = foot.mFootVelocity;
            spring.mLiftOff = foot.mLiftOff;
            spring.mLanding = foot.mLanding;
            spring.mPhase = foot.mPhase;
            spring.mDuration = foot.mDuration;
            spring.mLegMass = foot.mLegMass;
            spring.mGravity = input.mGravity;
            spring.mSettings = { s.mFrequency, s.mDampingRatio, s.mCapLegWeights };
            // Within the leg's reach of the hip at touchdown: pulled past it, a
            // strafing foot hung at full stretch, never touched down, and the
            // stance knee buckled (StrideSim 11-06); the hip carried forward
            // by the time the swing has left, or a running foot was held 16 u
            // short and still up.
            if (s.mReach > 0.0f && foot.mLegLength > 0.0f)
            {
                const float remaining = std::max(0.0f, 1.0f - foot.mPhase) * foot.mDuration;
                const float hipX = foot.mHip[0] + input.mPelvisVelocity.mX * remaining;
                const float hipY = foot.mHip[1] + input.mPelvisVelocity.mY * remaining;
                const float length = s.mReach * foot.mLegLength;
                const float dz = foot.mHip[2] - foot.mLandingHeight;
                const float across = std::sqrt(std::max(0.0f, length * length - dz * dz));
                const float offsetX = foot.mLanding.mX - hipX, offsetY = foot.mLanding.mY - hipY;
                const float far = std::hypot(offsetX, offsetY);
                if (far > across && far > 1e-3f)
                {
                    spring.mLanding = { hipX + offsetX * across / far, hipY + offsetY * across / far };
                    out.mReachClamped[side] = true;
                }
            }
            const auto force = computeSwingSpring(spring);
            if (!force)
                return std::nullopt;
            BalanceVec2 horizontal = force->mFootForce;
            if (s.mLiftFrequency > 0.0f && foot.mPhase < s.mLiftUntil && std::isfinite(foot.mFootHeight)
                && std::isfinite(foot.mFootVerticalVelocity) && std::isfinite(foot.mLift))
            {
                constexpr float pi = 3.14159265358979323846f;
                const float want = foot.mLandingHeight + std::max(0.0f, foot.mLift) * std::sin(pi * std::clamp(foot.mPhase, 0.0f, 1.0f));
                const float below = want - foot.mFootHeight;
                if (below > 0.0f)
                {
                    const float omega = 2.0f * pi * s.mLiftFrequency;
                    float lift = foot.mLegMass * (omega * omega * below - 2.0f * omega * foot.mFootVerticalVelocity);
                    lift = std::clamp(lift, 0.0f, s.mLiftCap * foot.mLegMass * input.mGravity);
                    out.mFootLift[side] = lift;
                    out.mPelvisLift -= lift;
                }
                // Still on the floor: no pull along it.
                if (foot.mFootHeight < foot.mLandingHeight + 2.0f)
                    horizontal = {};
            }
            out.mFootForce[side] = horizontal;
            out.mCapped[side] = force->mCapped;
            out.mPelvisForce.mX -= horizontal.mX;
            out.mPelvisForce.mY -= horizontal.mY;
        }
        return out;
    }
}
