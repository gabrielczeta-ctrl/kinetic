#include <gtest/gtest.h>

#include <ragdollcore/anatomy.hpp>
#include <ragdollcore/hitreaction.hpp>

#include <limits>

namespace
{
    using RagdollCore::AnatomicalRegion;
    using RagdollCore::HitReactionBand;
    using RagdollCore::HitReactionState;

    RagdollCore::ReferenceGaitProfile gait()
    {
        RagdollCore::ReferenceGaitProfile profile;
        profile.mHitStunMinimumImpulse = 400.0f;
        profile.mHitStunStaggerImpulse = 1250.0f;
        profile.mHitStunFallImpulse = 1750.0f;
        return profile;
    }

    constexpr float frame = 1.0f / 60.0f;
}

TEST(RagdollCoreHitReactionTest, noStunMeansNoBand)
{
    HitReactionState state;
    state.registerHit(9000.0f, true);
    EXPECT_EQ(state.band(false, gait()), HitReactionBand::None);
}

TEST(RagdollCoreHitReactionTest, singleHitsGradeByImpulse)
{
    const auto profile = gait();
    HitReactionState light;
    light.registerHit(800.0f, false);
    EXPECT_EQ(light.band(true, profile), HitReactionBand::Absorb);

    HitReactionState firm;
    firm.registerHit(1250.0f, false);
    EXPECT_EQ(firm.band(true, profile), HitReactionBand::Stagger);
}

TEST(RagdollCoreHitReactionTest, headAndTorsoBlowsStaggerButNeverDropByThemselves)
{
    const auto profile = gait();
    ASSERT_FALSE(RagdollCore::regionCanKnockDown(AnatomicalRegion::Head));
    ASSERT_FALSE(RagdollCore::regionCanKnockDown(AnatomicalRegion::Chest));
    ASSERT_FALSE(RagdollCore::regionCanKnockDown(AnatomicalRegion::RightForearm));
    HitReactionState state;
    // Reaction impulses measured in the duel of 2026-09-18: 2100-2625 * region scale.
    state.registerHit(3937.0f, RagdollCore::regionCanKnockDown(AnatomicalRegion::Head));
    EXPECT_EQ(state.band(true, profile), HitReactionBand::Stagger);
    // A sustained flurry to the body keeps staggering; it does not add up to a fall.
    for (int i = 0; i < 6; ++i)
    {
        state.registerHit(2415.0f, RagdollCore::regionCanKnockDown(AnatomicalRegion::Chest));
        EXPECT_EQ(state.band(true, profile), HitReactionBand::Stagger);
    }
}

TEST(RagdollCoreHitReactionTest, legGrazeDoesNotConvertTorsoFlurryIntoFall)
{
    const auto profile = gait();
    HitReactionState state;
    state.registerHit(2310.0f, false);
    state.registerHit(2310.0f, false);
    state.registerHit(450.0f, RagdollCore::regionCanKnockDown(AnatomicalRegion::LeftThigh));
    EXPECT_EQ(state.band(true, profile), HitReactionBand::Stagger);
}

TEST(RagdollCoreHitReactionTest, hardLegBlowOrLegFlurryDropsTheBody)
{
    const auto profile = gait();
    ASSERT_TRUE(RagdollCore::regionCanKnockDown(AnatomicalRegion::RightCalf));
    ASSERT_TRUE(RagdollCore::regionCanKnockDown(AnatomicalRegion::Pelvis));
    HitReactionState single;
    single.registerHit(1750.0f, true);
    EXPECT_EQ(single.band(true, profile), HitReactionBand::Fall);

    HitReactionState flurry;
    flurry.registerHit(900.0f, true);
    EXPECT_NE(flurry.band(true, profile), HitReactionBand::Fall);
    flurry.registerHit(900.0f, true);
    EXPECT_EQ(flurry.band(true, profile), HitReactionBand::Fall);
}

TEST(RagdollCoreHitReactionTest, overwhelmingBlowDropsTheBodyAnywhere)
{
    const auto profile = gait();
    HitReactionState state;
    state.registerHit(profile.mHitStunFallImpulse * HitReactionState::sOverwhelmingFallFactor, false);
    EXPECT_EQ(state.band(true, profile), HitReactionBand::Fall);
}

TEST(RagdollCoreHitReactionTest, caughtOnTheWeaponCannotKnockDown)
{
    // Every profile body maps to a region; an unknown contact in play is the
    // victim's equipped weapon.
    EXPECT_FALSE(RagdollCore::regionCanKnockDown(AnatomicalRegion::Unknown));
}

TEST(RagdollCoreHitReactionTest, sumsHoldThroughTheStunAndDecayAfterIt)
{
    const auto profile = gait();
    HitReactionState state;
    state.registerHit(1000.0f, true);
    for (int i = 0; i < 30; ++i)
        state.decay(frame, true);
    EXPECT_FLOAT_EQ(state.mKnockdownAccumulator, 1000.0f);
    EXPECT_FLOAT_EQ(state.mKnockdownPeakImpulse, 1000.0f);

    state.decay(frame, false);
    EXPECT_FLOAT_EQ(state.mPeakImpulse, 0.0f);
    EXPECT_FLOAT_EQ(state.mKnockdownPeakImpulse, 0.0f);
    EXPECT_LT(state.mKnockdownAccumulator, 1000.0f);
    // Two seconds later a second identical leg blow is a fresh hit, not a flurry.
    for (int i = 0; i < 120; ++i)
        state.decay(frame, false);
    state.registerHit(1000.0f, true);
    EXPECT_NE(state.band(true, profile), HitReactionBand::Fall);
}

TEST(RagdollCoreHitReactionTest, rejectsNonFiniteAndNonPositiveImpulses)
{
    HitReactionState state;
    state.registerHit(std::numeric_limits<float>::quiet_NaN(), true);
    state.registerHit(-5.0f, true);
    state.registerHit(0.0f, true);
    EXPECT_FLOAT_EQ(state.mAccumulator, 0.0f);
    EXPECT_FLOAT_EQ(state.mKnockdownAccumulator, 0.0f);
}
