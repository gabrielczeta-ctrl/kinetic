#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_IMPACTDAMAGE_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_IMPACTDAMAGE_H

namespace RagdollCore
{
    struct PhysicalImpactDamageProfile final
    {
        float mMinimumNormalSpeed = 1.0f;
        float mFullDamageMomentum = 12.0f;
    };

    // Fraction of a weapon's authored min-to-max damage range earned by the
    // measured contact-axis momentum. Invalid, resting, and separating
    // contacts return zero; valid results are bounded to [0, 1].
    float physicalImpactDamageFraction(float relativeNormalSpeed, float effectiveMass,
        const PhysicalImpactDamageProfile& profile = {}) noexcept;

    // Maps the fraction into an authored weapon damage band. A qualifying
    // contact always retains the band's minimum damage; invalid contacts do
    // zero damage. Reversed bands are normalized before mapping.
    float physicalImpactWeaponDamage(float minimumDamage, float maximumDamage,
        float relativeNormalSpeed, float effectiveMass,
        const PhysicalImpactDamageProfile& profile = {}) noexcept;

    // Damage for a contact whose injury was measured as physical work: the
    // authored maximum scaled by the delivered fraction of a full injury.
    // There is no floor, so a weak contact does little and no work does
    // nothing. Reversed bands are normalized; invalid input does zero damage.
    float physicalContactWorkDamage(float minimumDamage, float maximumDamage, float workFraction) noexcept;

    // Native animation charge selects the authored damage budget; measured
    // contact quality delivers a fraction of that entire budget. Charge is
    // never a damage floor: no physical work means no damage, at any charge.
    float physicalChargedWeaponDamage(
        float minimumDamage, float maximumDamage, float charge, float physicalFraction) noexcept;

    // DMG-04: how good a swing was, from the struck point's approach speed
    // (world units/s). Below mMinimumSpeed nothing, at mFullSpeed a vanilla
    // full-strength attack, past it a small bonus up to mBonusSpeed.
    struct SwingQualityProfile final
    {
        float mMinimumSpeed = 40.0f;
        float mFullSpeed = 300.0f;
        float mBonusSpeed = 450.0f;
        float mMaximumBonus = 0.15f;
    };

    // The full-swing speeds of a weapon by its weight (Morrowind weight units;
    // 0 or less for hand-to-hand). A heavier weapon reaches a full swing at a
    // lower speed: referenceSpeed at weight 20 (a longsword), scaled by
    // sqrt(20 / weight) and clamped to 0.7..1.3 of it. Fists: 0.8 of it.
    SwingQualityProfile swingQualityProfile(float weaponWeight, float referenceSpeed = 320.0f) noexcept;

    // 0 at or below the minimum speed, 1 at the full speed, clamped.
    float swingQuality(float approachSpeed, const SwingQualityProfile& profile) noexcept;

    // 1 up to the full speed, rising linearly to 1 + mMaximumBonus.
    float fullSwingBonus(float approachSpeed, const SwingQualityProfile& profile) noexcept;

    // Morrowind's band read at a quality: min + (max - min) * q. Reversed
    // bands are normalized; invalid input does zero damage.
    float morrowindBandDamage(float minimumDamage, float maximumDamage, float quality) noexcept;

    // Skill as control (user decision 2026-09-28): Morrowind's attack term
    // (weapon skill + Agility/5 + Luck/10, times the fatigue term, plus
    // Fortify Attack minus Blind) read as how well the weapon is handled,
    // 0.2..1. 100 and above is full control.
    float weaponControl(float attackTerm) noexcept;
}

#endif
