#include <ragdollcore/strikecooldown.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    StrikeCooldown::StrikeCooldown(float cooldownSeconds, float stepSeconds)
    {
        if (std::isfinite(cooldownSeconds) && cooldownSeconds >= 0.0f)
            mCooldownSeconds = cooldownSeconds;
        if (std::isfinite(stepSeconds) && stepSeconds > 0.0f)
        {
            const double ratio = static_cast<double>(mCooldownSeconds) / static_cast<double>(stepSeconds);
            // A float duration that is mathematically a whole number of steps
            // (0.35 s at 60 Hz) must not round up because of representation
            // error; an epsilon relative to the ratio absorbs that without
            // changing genuine fractional cooldowns.
            const double epsilon = 1e-6 * std::max(1.0, ratio);
            const double steps = std::ceil(ratio - epsilon);
            mCooldownSteps = steps > 0.0 ? static_cast<std::uint64_t>(steps) : 0;
        }
    }

    bool StrikeCooldown::allows(std::uint64_t targetIdentity, std::uint64_t physicsStep) const noexcept
    {
        if (targetIdentity == 0)
            return true;
        const auto found = mLastHitStep.find(targetIdentity);
        if (found == mLastHitStep.end())
            return true;
        // A restarted or rewound clock cannot be compared to the recorded
        // step; allow the hit instead of suppressing it forever.
        if (physicsStep < found->second)
            return true;
        return physicsStep - found->second >= mCooldownSteps;
    }

    void StrikeCooldown::commit(std::uint64_t targetIdentity, std::uint64_t physicsStep)
    {
        if (targetIdentity != 0)
            mLastHitStep[targetIdentity] = physicsStep;
    }

    void StrikeCooldown::reset() noexcept
    {
        mLastHitStep.clear();
    }
}
