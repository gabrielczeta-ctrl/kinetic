#include <ragdollcore/armstrain.hpp>

#include <gtest/gtest.h>

namespace
{
    using RagdollCore::ArmStrain;
    using RagdollCore::ArmStrainSettings;

    ArmStrainSettings gameSettings()
    {
        ArmStrainSettings settings;
        settings.mPerClash = 0.5f;
        settings.mHold = 1.5f;
        settings.mRecover = 0.2f;
        settings.mWeaken = 0.6f;
        return settings;
    }

    void wait(ArmStrain& arm, float seconds, const ArmStrainSettings& settings)
    {
        for (float t = 0.0f; t < seconds; t += 1.0f / 60.0f)
            arm.step(1.0f / 60.0f, settings);
    }

    // The 11-07 play: a deflect (ratio 1.07), then 2.8 s later a hard blow
    // (1.74) on the Hlaalu guard's sword - two clashes leave the arm spent.
    TEST(ArmStrainTest, twoSolidClashesSpendTheArm)
    {
        const ArmStrainSettings settings = gameSettings();
        ArmStrain arm;
        EXPECT_FALSE(arm.takeClash(1.07f, 1.0f, settings));
        EXPECT_LT(arm.strength(settings), 1.0f);
        wait(arm, 2.8f, settings);
        EXPECT_TRUE(arm.takeClash(1.74f, 1.0f, settings));
        EXPECT_TRUE(arm.spent());
        EXPECT_NEAR(arm.strength(settings), 0.4f, 1e-4f);
    }

    TEST(ArmStrainTest, oneCrushingBlowAloneLeavesAFreshArmStanding)
    {
        const ArmStrainSettings settings = gameSettings();
        ArmStrain arm;
        EXPECT_FALSE(arm.takeClash(3.6f, 1.0f, settings));
        EXPECT_NEAR(arm.strain(), 0.9f, 1e-4f);
        EXPECT_TRUE(arm.takeClash(0.5f, 1.0f, settings));
    }

    TEST(ArmStrainTest, lightTapsTakeLongerAndEaseOffBetween)
    {
        const ArmStrainSettings settings = gameSettings();
        ArmStrain arm;
        // Bounces (ratio 0.6) 4 s apart never build up.
        for (int i = 0; i < 6; ++i)
        {
            EXPECT_FALSE(arm.takeClash(0.6f, 1.0f, settings));
            wait(arm, 4.0f, settings);
        }
        EXPECT_FALSE(arm.spent());
        // In quick succession they do.
        int clashes = 0;
        while (!arm.spent() && clashes < 10)
        {
            arm.takeClash(0.6f, 1.0f, settings);
            ++clashes;
            wait(arm, 0.5f, settings);
        }
        EXPECT_TRUE(arm.spent());
        EXPECT_GE(clashes, 3);
        EXPECT_LE(clashes, 4);
    }

    TEST(ArmStrainTest, aSpentArmRecoversAtHalfStrain)
    {
        const ArmStrainSettings settings = gameSettings();
        ArmStrain arm;
        arm.takeClash(2.0f, 1.0f, settings);
        arm.takeClash(2.0f, 1.0f, settings);
        ASSERT_TRUE(arm.spent());
        wait(arm, 1.5f, settings); // held
        EXPECT_TRUE(arm.spent());
        EXPECT_NEAR(arm.strain(), 1.0f, 0.01f);
        wait(arm, 2.0f, settings); // 0.6 left
        EXPECT_TRUE(arm.spent());
        wait(arm, 0.6f, settings); // under half
        EXPECT_FALSE(arm.spent());
        wait(arm, 10.0f, settings);
        EXPECT_FLOAT_EQ(arm.strength(settings), 1.0f);
    }

    TEST(ArmStrainTest, theShareAndTheSwitch)
    {
        ArmStrainSettings settings = gameSettings();
        ArmStrain arm;
        // The player's share 0: untouched.
        EXPECT_FALSE(arm.takeClash(3.0f, 0.0f, settings));
        EXPECT_FLOAT_EQ(arm.strain(), 0.0f);
        arm.takeClash(1.0f, 1.0f, settings);
        EXPECT_GT(arm.strain(), 0.0f);
        // Off: no strain, full strength, and the step clears what was there.
        settings.mPerClash = 0.0f;
        EXPECT_FLOAT_EQ(arm.strength(settings), 1.0f);
        EXPECT_FALSE(arm.takeClash(3.0f, 1.0f, settings));
        arm.step(1.0f / 60.0f, settings);
        EXPECT_FLOAT_EQ(arm.strain(), 0.0f);
    }
}
