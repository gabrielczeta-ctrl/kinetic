#include <gtest/gtest.h>
#include <ragdollcore/woundheal.hpp>

#include <limits>

using namespace RagdollCore;

namespace
{
    const WoundHealSettings sSettings{ true, 1.0f };
}

TEST(RagdollCoreWoundHealTest, TheFirstStepOnlyTakesTheReadings)
{
    WoundHealState state;
    EXPECT_FLOAT_EQ(woundHealShare(state, 100.0f, 100.0f, 5.0, sSettings), 0.0f);
    EXPECT_FLOAT_EQ(state.mLastPool, 100.0f);
}

TEST(RagdollCoreWoundHealTest, RestoredHealthHealsItsShareOfWhatWasMissing)
{
    WoundHealState state;
    woundHealShare(state, 40.0f, 100.0f, 0.0, sSettings);
    // 30 of the 60 missing came back: half of what is left heals.
    EXPECT_FLOAT_EQ(woundHealShare(state, 70.0f, 100.0f, 0.0, sSettings), 0.5f);
    // The other 30: everything left.
    EXPECT_FLOAT_EQ(woundHealShare(state, 100.0f, 100.0f, 0.0, sSettings), 1.0f);
}

TEST(RagdollCoreWoundHealTest, StepsAddUpToTheShareOfTheMissingPool)
{
    // 20 -> 60 of 100 in forty small steps leaves half the wounds, as one
    // step would: the remaining wounds are the remaining missing health.
    WoundHealState state;
    woundHealShare(state, 20.0f, 100.0f, 0.0, sSettings);
    float left = 1.0f;
    for (int step = 1; step <= 40; ++step)
        left *= 1.0f - woundHealShare(state, 20.0f + static_cast<float>(step), 100.0f, 0.0, sSettings);
    EXPECT_NEAR(left, 0.5f, 1e-4f);
}

TEST(RagdollCoreWoundHealTest, DamageAndAStillPoolHealNothing)
{
    WoundHealState state;
    woundHealShare(state, 80.0f, 100.0f, 0.0, sSettings);
    EXPECT_FLOAT_EQ(woundHealShare(state, 50.0f, 100.0f, 0.5, sSettings), 0.0f);
    // Hurt and waiting: time alone heals nothing below a full pool.
    EXPECT_FLOAT_EQ(woundHealShare(state, 50.0f, 100.0f, 9.0, sSettings), 0.0f);
}

TEST(RagdollCoreWoundHealTest, AFullPoolFadesWhatIsLeftOverGameTime)
{
    WoundHealState state;
    woundHealShare(state, 100.0f, 100.0f, 10.0, sSettings);
    EXPECT_NEAR(woundHealShare(state, 100.0f, 100.0f, 10.25, sSettings), 0.25f, 1e-5f);
    // An hour's wait: all of it.
    EXPECT_FLOAT_EQ(woundHealShare(state, 100.0f, 100.0f, 12.0, sSettings), 1.0f);
    // A loaded game's clock ran backwards: no time passed.
    EXPECT_FLOAT_EQ(woundHealShare(state, 100.0f, 100.0f, 3.0, sSettings), 0.0f);
    // Full hours 0: a full pool's wounds stay.
    WoundHealState kept;
    const WoundHealSettings noFade{ true, 0.0f };
    woundHealShare(kept, 100.0f, 100.0f, 0.0, noFade);
    EXPECT_FLOAT_EQ(woundHealShare(kept, 100.0f, 100.0f, 50.0, noFade), 0.0f);
}

TEST(RagdollCoreWoundHealTest, ADeadOrUnreadablePoolHealsNothing)
{
    WoundHealState state;
    woundHealShare(state, 30.0f, 100.0f, 0.0, sSettings);
    EXPECT_FLOAT_EQ(woundHealShare(state, 0.0f, 100.0f, 1.0, sSettings), 0.0f);
    EXPECT_FLOAT_EQ(woundHealShare(state, std::numeric_limits<float>::quiet_NaN(), 100.0f, 1.0, sSettings), 0.0f);
    EXPECT_LT(state.mLastPool, 0.0f); // forgotten: the next step starts over
    EXPECT_FLOAT_EQ(woundHealShare(state, 100.0f, 0.0f, 1.0, sSettings), 0.0f);
}

TEST(RagdollCoreWoundHealTest, OffHealsNothing)
{
    WoundHealState state;
    const WoundHealSettings off{ false, 1.0f };
    woundHealShare(state, 10.0f, 100.0f, 0.0, off);
    EXPECT_FLOAT_EQ(woundHealShare(state, 100.0f, 100.0f, 5.0, off), 0.0f);
}

TEST(RagdollCoreWoundHealTest, AChangedMaximumKeepsTheShareInRange)
{
    // Fortify Health: 100 of 100 becomes 150 of 150, a full pool again.
    WoundHealState state;
    woundHealShare(state, 100.0f, 100.0f, 0.0, sSettings);
    EXPECT_FLOAT_EQ(woundHealShare(state, 150.0f, 150.0f, 0.0, sSettings), 1.0f);
    // The maximum fell under the last reading: still inside 0..1.
    WoundHealState drained;
    woundHealShare(drained, 90.0f, 100.0f, 0.0, sSettings);
    const float share = woundHealShare(drained, 60.0f, 60.0f, 0.0, sSettings);
    EXPECT_GE(share, 0.0f);
    EXPECT_LE(share, 1.0f);
}

TEST(RagdollCoreWoundHealTest, MarksGoOneAtATimeAndAllAtAFullShare)
{
    WoundHealState state;
    EXPECT_EQ(woundMarksHealed(state, 4, 0.0f), 0);
    // A tenth of four marks a step: the first goes on the third step.
    EXPECT_EQ(woundMarksHealed(state, 4, 0.1f), 0);
    EXPECT_EQ(woundMarksHealed(state, 4, 0.1f), 0);
    EXPECT_EQ(woundMarksHealed(state, 4, 0.1f), 1);
    EXPECT_EQ(woundMarksHealed(state, 3, 1.0f), 3);
    EXPECT_FLOAT_EQ(state.mCarry, 0.0f);
    // Never more than there are; no marks, no carry.
    EXPECT_EQ(woundMarksHealed(state, 2, 0.99f), 1);
    EXPECT_EQ(woundMarksHealed(state, 0, 0.5f), 0);
    EXPECT_FLOAT_EQ(state.mCarry, 0.0f);
}

TEST(RagdollCoreWoundHealTest, HalfTheMissingHealthRemovesHalfTheMarks)
{
    WoundHealState state;
    woundHealShare(state, 20.0f, 100.0f, 0.0, sSettings);
    int marks = 12;
    for (int step = 1; step <= 40; ++step)
        marks -= woundMarksHealed(state, marks,
            woundHealShare(state, 20.0f + static_cast<float>(step), 100.0f, 0.0, sSettings));
    EXPECT_GE(marks, 5);
    EXPECT_LE(marks, 7);
    marks -= woundMarksHealed(state, marks, woundHealShare(state, 100.0f, 100.0f, 0.0, sSettings));
    EXPECT_EQ(marks, 0);
}

TEST(RagdollCoreWoundHealTest, SettingsReadOnAndFullHours)
{
    EXPECT_TRUE(parseWoundHealSettings(nullptr).mOn);
    EXPECT_FLOAT_EQ(parseWoundHealSettings(nullptr).mFullHours, 1.0f);
    EXPECT_FALSE(parseWoundHealSettings("0").mOn);
    EXPECT_FLOAT_EQ(parseWoundHealSettings("0").mFullHours, 1.0f);
    const WoundHealSettings set = parseWoundHealSettings("1,2.5");
    EXPECT_TRUE(set.mOn);
    EXPECT_FLOAT_EQ(set.mFullHours, 2.5f);
    EXPECT_FLOAT_EQ(parseWoundHealSettings("1,-3").mFullHours, 1.0f);
    EXPECT_TRUE(parseWoundHealSettings("junk").mOn);
}
