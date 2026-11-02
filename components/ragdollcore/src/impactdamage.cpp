#include <ragdollcore/impactdamage.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        bool validImpact(float relativeNormalSpeed, float effectiveMass,
            const PhysicalImpactDamageProfile& profile) noexcept
        {
            return std::isfinite(relativeNormalSpeed) && std::isfinite(effectiveMass)
                && std::isfinite(profile.mMinimumNormalSpeed) && std::isfinite(profile.mFullDamageMomentum)
                && relativeNormalSpeed >= profile.mMinimumNormalSpeed && effectiveMass > 0.0f
                && profile.mMinimumNormalSpeed >= 0.0f && profile.mFullDamageMomentum > 0.0f;
        }
    }

    float physicalImpactDamageFraction(float relativeNormalSpeed, float effectiveMass,
        const PhysicalImpactDamageProfile& profile) noexcept
    {
        if (!validImpact(relativeNormalSpeed, effectiveMass, profile))
            return 0.0f;
        return std::clamp(effectiveMass * relativeNormalSpeed / profile.mFullDamageMomentum, 0.0f, 1.0f);
    }

    float physicalImpactWeaponDamage(float minimumDamage, float maximumDamage,
        float relativeNormalSpeed, float effectiveMass, const PhysicalImpactDamageProfile& profile) noexcept
    {
        if (!std::isfinite(minimumDamage) || !std::isfinite(maximumDamage)
            || !validImpact(relativeNormalSpeed, effectiveMass, profile))
            return 0.0f;
        const float low = std::max(0.0f, std::min(minimumDamage, maximumDamage));
        const float high = std::max(low, std::max(minimumDamage, maximumDamage));
        return low + (high - low) * physicalImpactDamageFraction(relativeNormalSpeed, effectiveMass, profile);
    }

    float physicalContactWorkDamage(float minimumDamage, float maximumDamage, float workFraction) noexcept
    {
        if (!std::isfinite(minimumDamage) || !std::isfinite(maximumDamage) || !std::isfinite(workFraction)
            || workFraction <= 0.0f)
            return 0.0f;
        return std::max(0.0f, std::max(minimumDamage, maximumDamage)) * std::min(workFraction, 1.0f);
    }

    float physicalChargedWeaponDamage(
        float minimumDamage, float maximumDamage, float charge, float physicalFraction) noexcept
    {
        if (!std::isfinite(minimumDamage) || !std::isfinite(maximumDamage) || !std::isfinite(charge))
            return 0.0f;
        const float low = std::max(0.0f, std::min(minimumDamage, maximumDamage));
        const float high = std::max(low, std::max(minimumDamage, maximumDamage));
        const float budget = low + (high - low) * std::clamp(charge, 0.0f, 1.0f);
        return physicalContactWorkDamage(0.0f, budget, physicalFraction);
    }

    SwingQualityProfile swingQualityProfile(float weaponWeight, float referenceSpeed) noexcept
    {
        SwingQualityProfile profile;
        if (!std::isfinite(referenceSpeed) || !(referenceSpeed > profile.mMinimumSpeed))
            referenceSpeed = 320.0f;
        if (!std::isfinite(weaponWeight) || weaponWeight <= 0.0f)
            profile.mFullSpeed = 0.8f * referenceSpeed;
        else
            profile.mFullSpeed = referenceSpeed * std::clamp(std::sqrt(20.0f / weaponWeight), 0.7f, 1.3f);
        profile.mBonusSpeed = profile.mFullSpeed * 1.5f;
        return profile;
    }

    float swingQuality(float approachSpeed, const SwingQualityProfile& profile) noexcept
    {
        if (!std::isfinite(approachSpeed) || !(profile.mFullSpeed > profile.mMinimumSpeed))
            return 0.0f;
        return std::clamp(
            (approachSpeed - profile.mMinimumSpeed) / (profile.mFullSpeed - profile.mMinimumSpeed), 0.0f, 1.0f);
    }

    float fullSwingBonus(float approachSpeed, const SwingQualityProfile& profile) noexcept
    {
        if (!std::isfinite(approachSpeed) || !(profile.mBonusSpeed > profile.mFullSpeed)
            || !(profile.mMaximumBonus > 0.0f))
            return 1.0f;
        return 1.0f
            + profile.mMaximumBonus
            * std::clamp((approachSpeed - profile.mFullSpeed) / (profile.mBonusSpeed - profile.mFullSpeed), 0.0f,
                1.0f);
    }

    float morrowindBandDamage(float minimumDamage, float maximumDamage, float quality) noexcept
    {
        if (!std::isfinite(minimumDamage) || !std::isfinite(maximumDamage) || !std::isfinite(quality))
            return 0.0f;
        const float low = std::max(0.0f, std::min(minimumDamage, maximumDamage));
        const float high = std::max(low, std::max(minimumDamage, maximumDamage));
        return low + (high - low) * std::clamp(quality, 0.0f, 1.0f);
    }

    float weaponControl(float attackTerm) noexcept
    {
        if (!std::isfinite(attackTerm))
            return 1.0f;
        return std::clamp(attackTerm / 100.0f, 0.2f, 1.0f);
    }
}
