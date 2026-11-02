#include <gtest/gtest.h>

#include <ragdollcore/strikecooldown.hpp>

#include <cmath>
#include <limits>

namespace
{
    constexpr float stepSeconds = 1.0f / 60.0f;
}

TEST(RagdollCoreStrikeCooldownTest, firstHitIsAllowed)
{
    RagdollCore::StrikeCooldown cooldown(0.35f, stepSeconds);
    EXPECT_TRUE(cooldown.allows(73, 100));
    cooldown.commit(73, 100);
    EXPECT_EQ(cooldown.cooldownSteps(), 21u);
}

TEST(RagdollCoreStrikeCooldownTest, contactChatterIsSuppressedUntilTheGateElapses)
{
    RagdollCore::StrikeCooldown cooldown(0.35f, stepSeconds);
    cooldown.commit(73, 100);
    // Every solver step in between is still contact on the same blow.
    for (std::uint64_t step = 101; step < 121; ++step)
        EXPECT_FALSE(cooldown.allows(73, step)) << "step " << step;
    EXPECT_TRUE(cooldown.allows(73, 121));
    cooldown.commit(73, 121);
    EXPECT_FALSE(cooldown.allows(73, 122));
}

TEST(RagdollCoreStrikeCooldownTest, targetsAreIndependent)
{
    RagdollCore::StrikeCooldown cooldown(0.35f, stepSeconds);
    cooldown.commit(73, 100);
    EXPECT_FALSE(cooldown.allows(73, 110));
    EXPECT_TRUE(cooldown.allows(74, 110));
    EXPECT_TRUE(cooldown.allows(75, 110));
}

TEST(RagdollCoreStrikeCooldownTest, zeroCooldownDisablesTheGate)
{
    RagdollCore::StrikeCooldown cooldown(0.0f, stepSeconds);
    cooldown.commit(73, 100);
    EXPECT_TRUE(cooldown.allows(73, 100));
    EXPECT_TRUE(cooldown.allows(73, 101));
}

TEST(RagdollCoreStrikeCooldownTest, fractionalCooldownRoundsUp)
{
    RagdollCore::StrikeCooldown cooldown(0.36f, stepSeconds);
    EXPECT_EQ(cooldown.cooldownSteps(), 22u);
    cooldown.commit(73, 10);
    EXPECT_FALSE(cooldown.allows(73, 31));
    EXPECT_TRUE(cooldown.allows(73, 32));
}

TEST(RagdollCoreStrikeCooldownTest, invalidInputTakesTheDefault)
{
    RagdollCore::StrikeCooldown cooldown(std::numeric_limits<float>::quiet_NaN(), stepSeconds);
    EXPECT_FLOAT_EQ(cooldown.cooldownSeconds(), RagdollCore::StrikeCooldown::sDefaultCooldownSeconds);
    EXPECT_EQ(cooldown.cooldownSteps(), 21u);

    RagdollCore::StrikeCooldown negative(-1.0f, stepSeconds);
    EXPECT_FLOAT_EQ(negative.cooldownSeconds(), RagdollCore::StrikeCooldown::sDefaultCooldownSeconds);
}

TEST(RagdollCoreStrikeCooldownTest, clockRestartAllowsTheHit)
{
    RagdollCore::StrikeCooldown cooldown(0.35f, stepSeconds);
    cooldown.commit(73, 100);
    EXPECT_FALSE(cooldown.allows(73, 110));
    // A reload or new physics clock must not suppress forever.
    EXPECT_TRUE(cooldown.allows(73, 5));
}

TEST(RagdollCoreStrikeCooldownTest, unidentifiableTargetsAreNeverGated)
{
    RagdollCore::StrikeCooldown cooldown(0.35f, stepSeconds);
    cooldown.commit(0, 100);
    EXPECT_TRUE(cooldown.allows(0, 100));
    EXPECT_TRUE(cooldown.allows(0, 101));
}

TEST(RagdollCoreStrikeCooldownTest, resetClearsEveryTarget)
{
    RagdollCore::StrikeCooldown cooldown(0.35f, stepSeconds);
    cooldown.commit(73, 100);
    cooldown.commit(74, 100);
    cooldown.reset();
    EXPECT_TRUE(cooldown.allows(73, 101));
    EXPECT_TRUE(cooldown.allows(74, 101));
}
