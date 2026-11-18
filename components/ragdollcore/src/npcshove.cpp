#include <ragdollcore/npcshove.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    float npcShoveChance(const NpcShoveInput& input, const NpcShoveSettings& settings)
    {
        if (!(input.mDt > 0.0f) || input.mAttacking || input.mSinceLast < settings.mCooldown
            || input.mFatigue < settings.mMinFatigue || input.mDistance > settings.mReach
            || input.mFacing < settings.mCone)
            return 0.0f;
        float rate = 0.0f;
        if (input.mFoeGuarding)
            rate = settings.mGuardRate;
        else if (input.mDistance <= settings.mClose)
            rate = settings.mCloseRate;
        if (!(rate > 0.0f))
            return 0.0f;
        // Stronger fighters shove more readily (Strength 50: x1; 100: x1.5).
        rate *= std::clamp(0.5f + input.mStrength / 100.0f, 0.6f, 1.5f);
        return 1.0f - std::exp(-rate * std::min(input.mDt, 0.25f));
    }

    bool npcShouldShove(const NpcShoveInput& input, const NpcShoveSettings& settings)
    {
        const float chance = npcShoveChance(input, settings);
        return chance > 0.0f && input.mRoll < chance;
    }
}
