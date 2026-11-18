#include <ragdollcore/armstrain.hpp>

#include <ragdollcore/tuning.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace RagdollCore
{
    void ArmStrain::step(float dt, const ArmStrainSettings& settings)
    {
        if (!(dt > 0.0f))
            return;
        if (!settings.on())
        {
            reset();
            return;
        }
        mSinceClash += dt;
        if (mSinceClash > settings.mHold)
            mStrain = std::max(0.0f, mStrain - settings.mRecover * dt);
        // Spent until it has eased to half.
        if (mSpent && mStrain < 0.5f)
            mSpent = false;
    }

    bool ArmStrain::takeClash(float ratio, float share, const ArmStrainSettings& settings)
    {
        if (!settings.on() || !(share > 0.0f) || !std::isfinite(ratio))
            return false;
        // A tap still tells (0.3); a crushing blow counts 1.8 even ones at
        // most, so at the game's 0.5 one blow alone never spends a fresh arm.
        mStrain = std::min(1.0f, mStrain + settings.mPerClash * std::clamp(ratio, 0.3f, 1.8f) * share);
        mSinceClash = 0.0f;
        if (mSpent || mStrain < 1.0f)
            return false;
        mSpent = true;
        return true;
    }

    float ArmStrain::strength(const ArmStrainSettings& settings) const
    {
        if (!settings.on())
            return 1.0f;
        return std::clamp(1.0f - settings.mWeaken * mStrain, 0.1f, 1.0f);
    }

    ArmStrainSettings armStrainSettings()
    {
        ArmStrainSettings settings;
        std::array<float, 5> values{ 0.0f, settings.mHold, settings.mRecover, settings.mWeaken, settings.mPlayer };
        tuningList("OPENMW_RAGDOLL_K2_ARM_STRAIN", values);
        for (float& v : values)
            v = std::isfinite(v) ? std::max(0.0f, v) : 0.0f;
        settings.mPerClash = std::min(values[0], 2.0f);
        settings.mHold = std::min(values[1], 10.0f);
        settings.mRecover = std::min(values[2], 5.0f);
        settings.mWeaken = std::min(values[3], 0.9f);
        settings.mPlayer = std::min(values[4], 1.0f);
        return settings;
    }
}
