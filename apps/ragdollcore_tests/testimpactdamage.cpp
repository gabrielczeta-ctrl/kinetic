#include <gtest/gtest.h>

#include <ragdollcore/impactdamage.hpp>

#include <limits>

TEST(RagdollCoreImpactDamageTest, speedSweepIsMonotonicAndBounded)
{
    float previous = -1.0f;
    for (float speed : { 1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f, 64.0f })
    {
        const float value = RagdollCore::physicalImpactDamageFraction(speed, 0.5f);
        EXPECT_GE(value, previous);
        EXPECT_GE(value, 0.0f);
        EXPECT_LE(value, 1.0f);
        previous = value;
    }
    ::testing::Test::RecordProperty("control", "speed_sweep");
    ::testing::Test::RecordProperty("monotonic", true);
    ::testing::Test::RecordProperty("minimum_fraction",
        RagdollCore::physicalImpactDamageFraction(1.0f, 0.5f));
    ::testing::Test::RecordProperty("maximum_fraction",
        RagdollCore::physicalImpactDamageFraction(64.0f, 0.5f));
}

TEST(RagdollCoreImpactDamageTest, effectiveMassSweepIsMonotonicAndBounded)
{
    float previous = -1.0f;
    for (float mass : { 0.05f, 0.1f, 0.25f, 0.5f, 1.0f, 2.0f, 4.0f })
    {
        const float value = RagdollCore::physicalImpactDamageFraction(12.0f, mass);
        EXPECT_GE(value, previous);
        EXPECT_GE(value, 0.0f);
        EXPECT_LE(value, 1.0f);
        previous = value;
    }
    ::testing::Test::RecordProperty("control", "mass_sweep");
    ::testing::Test::RecordProperty("monotonic", true);
}

TEST(RagdollCoreImpactDamageTest, authoredDamageBandIsRespected)
{
    const float low = RagdollCore::physicalImpactWeaponDamage(2.0f, 10.0f, 1.0f, 0.5f);
    const float middle = RagdollCore::physicalImpactWeaponDamage(2.0f, 10.0f, 12.0f, 0.5f);
    const float high = RagdollCore::physicalImpactWeaponDamage(2.0f, 10.0f, 100.0f, 1.0f);
    EXPECT_GE(low, 2.0f);
    EXPECT_GT(middle, low);
    EXPECT_LT(middle, 10.0f);
    EXPECT_FLOAT_EQ(high, 10.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalImpactWeaponDamage(10.0f, 2.0f, 100.0f, 1.0f), 10.0f);
    ::testing::Test::RecordProperty("control", "damage_band");
    ::testing::Test::RecordProperty("low_damage", low);
    ::testing::Test::RecordProperty("middle_damage", middle);
    ::testing::Test::RecordProperty("high_damage", high);
}

TEST(RagdollCoreImpactDamageTest, restMissAndInvalidInputsDoZeroDamage)
{
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FLOAT_EQ(RagdollCore::physicalImpactWeaponDamage(2.0f, 10.0f, 0.0f, 1.0f), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalImpactWeaponDamage(2.0f, 10.0f, -5.0f, 1.0f), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalImpactWeaponDamage(2.0f, 10.0f, 20.0f, 0.0f), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalImpactWeaponDamage(2.0f, 10.0f, nan, 1.0f), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalImpactWeaponDamage(2.0f, 10.0f, 20.0f, nan), 0.0f);
    ::testing::Test::RecordProperty("control", "zero_damage");
    ::testing::Test::RecordProperty("damage", 0.0f);
}

TEST(RagdollCoreImpactDamageTest, measuredContactWorkScalesFromZeroWithoutTheBandMinimum)
{
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FLOAT_EQ(RagdollCore::physicalContactWorkDamage(4.0f, 5.0f, 0.0f), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalContactWorkDamage(4.0f, 5.0f, -1.0f), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalContactWorkDamage(4.0f, 5.0f, nan), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalContactWorkDamage(nan, 5.0f, 0.5f), 0.0f);
    const float weak = RagdollCore::physicalContactWorkDamage(4.0f, 5.0f, 0.1f);
    EXPECT_FLOAT_EQ(weak, 0.5f);
    EXPECT_LT(weak, 4.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalContactWorkDamage(4.0f, 5.0f, 1.0f), 5.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalContactWorkDamage(4.0f, 5.0f, 3.0f), 5.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalContactWorkDamage(5.0f, 4.0f, 1.0f), 5.0f);
    ::testing::Test::RecordProperty("control", "contact_work");
    ::testing::Test::RecordProperty("weak_damage", weak);
}

TEST(RagdollCoreImpactDamageTest, duplicateContactCannotIncreaseMappedDamage)
{
    const float once = RagdollCore::physicalImpactWeaponDamage(2.0f, 10.0f, 12.0f, 0.5f);
    const float duplicate = RagdollCore::physicalImpactWeaponDamage(2.0f, 10.0f, 12.0f, 0.5f);
    EXPECT_FLOAT_EQ(once, duplicate);
    ::testing::Test::RecordProperty("control", "duplicate_input");
    ::testing::Test::RecordProperty("damage", once);
}

TEST(RagdollCoreImpactDamageTest, nativeChargeSelectsBudgetAndPhysicalWorkDeliversIt)
{
    struct Case
    {
        float mCharge;
        float mWork;
        float mDamage;
    };
    constexpr Case cases[] = {
        { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f },
        { 0.0f, 0.1f, 0.4f }, { 1.0f, 0.1f, 0.5f },
        { 0.0f, 1.0f, 4.0f }, { 0.5f, 1.0f, 4.5f }, { 1.0f, 1.0f, 5.0f },
    };
    for (const auto& test : cases)
    {
        SCOPED_TRACE(::testing::Message() << "charge=" << test.mCharge << " work=" << test.mWork);
        EXPECT_FLOAT_EQ(RagdollCore::physicalChargedWeaponDamage(4.0f, 5.0f, test.mCharge, test.mWork), test.mDamage);
    }
    // Negative control for applying the vanilla minimum *after* attenuation:
    // a weak charged contact must remain below even the uncharged full hit.
    EXPECT_LT(RagdollCore::physicalChargedWeaponDamage(4.0f, 5.0f, 1.0f, 0.1f),
        RagdollCore::physicalChargedWeaponDamage(4.0f, 5.0f, 0.0f, 1.0f));
}

TEST(RagdollCoreImpactDamageTest, nativeChargeMappingRejectsInvalidWorkAndBoundsInputs)
{
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    constexpr float infinity = std::numeric_limits<float>::infinity();
    EXPECT_FLOAT_EQ(RagdollCore::physicalChargedWeaponDamage(4.0f, 5.0f, 1.0f, -0.1f), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalChargedWeaponDamage(4.0f, 5.0f, 1.0f, nan), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalChargedWeaponDamage(4.0f, 5.0f, 1.0f, infinity), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalChargedWeaponDamage(4.0f, 5.0f, nan, 1.0f), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalChargedWeaponDamage(4.0f, 5.0f, infinity, 1.0f), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalChargedWeaponDamage(nan, 5.0f, 1.0f, 1.0f), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalChargedWeaponDamage(4.0f, infinity, 1.0f, 1.0f), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalChargedWeaponDamage(4.0f, 5.0f, -1.0f, 1.0f), 4.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalChargedWeaponDamage(4.0f, 5.0f, 2.0f, 3.0f), 5.0f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalChargedWeaponDamage(5.0f, 4.0f, 0.5f, 1.0f), 4.5f);
    EXPECT_FLOAT_EQ(RagdollCore::physicalChargedWeaponDamage(-2.0f, -1.0f, 1.0f, 1.0f), 0.0f);
}

TEST(RagdollCoreImpactDamageTest, nativeChargeAndWorkAreIndependentlyMonotonic)
{
    for (const float charge : { 0.0f, 0.25f, 0.5f, 1.0f })
    {
        float previous = 0.0f;
        for (const float work : { 0.0f, 0.1f, 0.5f, 1.0f })
        {
            const float damage = RagdollCore::physicalChargedWeaponDamage(2.0f, 20.0f, charge, work);
            EXPECT_GE(damage, previous);
            EXPECT_LE(damage, 20.0f);
            previous = damage;
        }
    }
    for (const float work : { 0.0f, 0.1f, 0.5f, 1.0f })
    {
        float previous = 0.0f;
        for (const float charge : { 0.0f, 0.25f, 0.5f, 1.0f })
        {
            const float damage = RagdollCore::physicalChargedWeaponDamage(2.0f, 20.0f, charge, work);
            EXPECT_GE(damage, previous);
            previous = damage;
        }
    }
}

// DMG-04: swing quality from approach speed reads Morrowind's band.
TEST(RagdollCoreImpactDamageTest, swingQualityReadsTheBandFromSpeed)
{
    const RagdollCore::SwingQualityProfile longsword = RagdollCore::swingQualityProfile(20.0f);
    EXPECT_FLOAT_EQ(longsword.mFullSpeed, 320.0f);
    EXPECT_FLOAT_EQ(RagdollCore::swingQuality(longsword.mMinimumSpeed, longsword), 0.0f);
    EXPECT_FLOAT_EQ(RagdollCore::swingQuality(longsword.mFullSpeed, longsword), 1.0f);
    EXPECT_FLOAT_EQ(RagdollCore::swingQuality(5000.0f, longsword), 1.0f);
    EXPECT_FLOAT_EQ(RagdollCore::swingQuality(std::numeric_limits<float>::quiet_NaN(), longsword), 0.0f);
    float previous = -1.0f;
    for (float speed = 0.0f; speed <= 600.0f; speed += 20.0f)
    {
        const float quality = RagdollCore::swingQuality(speed, longsword);
        EXPECT_GE(quality, previous);
        previous = quality;
    }
    // Iron claymore chop 1-24: a full swing is the vanilla full-strength hit,
    // a clean faster one up to 15% more, a half-speed swing about half.
    EXPECT_FLOAT_EQ(RagdollCore::morrowindBandDamage(1.0f, 24.0f, 1.0f), 24.0f);
    EXPECT_FLOAT_EQ(RagdollCore::morrowindBandDamage(24.0f, 1.0f, 0.0f), 1.0f);
    EXPECT_FLOAT_EQ(RagdollCore::fullSwingBonus(longsword.mFullSpeed, longsword), 1.0f);
    EXPECT_FLOAT_EQ(RagdollCore::fullSwingBonus(10000.0f, longsword), 1.15f);
    EXPECT_FLOAT_EQ(RagdollCore::fullSwingBonus(100.0f, longsword), 1.0f);
}

TEST(RagdollCoreImpactDamageTest, heavierWeaponsReachAFullSwingSlower)
{
    const float dagger = RagdollCore::swingQualityProfile(3.0f).mFullSpeed;
    const float longsword = RagdollCore::swingQualityProfile(20.0f).mFullSpeed;
    const float claymore = RagdollCore::swingQualityProfile(27.0f).mFullSpeed;
    const float warhammer = RagdollCore::swingQualityProfile(32.0f).mFullSpeed;
    EXPECT_GT(dagger, longsword);
    EXPECT_GT(longsword, claymore);
    EXPECT_GT(claymore, warhammer);
    EXPECT_LE(dagger, 1.3f * 320.0f + 1e-3f);
    EXPECT_GE(RagdollCore::swingQualityProfile(200.0f).mFullSpeed, 0.7f * 320.0f - 1e-3f);
    EXPECT_FLOAT_EQ(RagdollCore::swingQualityProfile(0.0f).mFullSpeed, 0.8f * 320.0f);
    EXPECT_FLOAT_EQ(RagdollCore::swingQualityProfile(20.0f, 400.0f).mFullSpeed, 400.0f);
}

TEST(RagdollCoreImpactDamageTest, weaponControlIsTheAttackTermBounded)
{
    EXPECT_FLOAT_EQ(RagdollCore::weaponControl(150.0f), 1.0f);
    EXPECT_FLOAT_EQ(RagdollCore::weaponControl(74.0f), 0.74f);
    EXPECT_FLOAT_EQ(RagdollCore::weaponControl(-30.0f), 0.2f);
    EXPECT_FLOAT_EQ(RagdollCore::weaponControl(std::numeric_limits<float>::infinity()), 1.0f);
}
