#include <ragdollcore/duelread.hpp>

#include <gtest/gtest.h>

namespace
{
    using RagdollCore::GuardRead;
    using RagdollCore::GuardReadInput;
    using RagdollCore::readGuardZone;

    GuardReadInput blade(float x, float y, float z, float vx = 0.0f, float vy = 0.0f, float vz = 0.0f)
    {
        GuardReadInput in;
        in.mX = x;
        in.mY = y;
        in.mZ = z;
        in.mVX = vx;
        in.mVY = vy;
        in.mVZ = vz;
        return in;
    }
}

TEST(RagdollCoreDuelReadTest, aWindUpShowsWhereTheCutComesFrom)
{
    EXPECT_EQ(readGuardZone(blade(5.0f, 60.0f, 50.0f)), GuardRead::High); // over the head: a chop
    EXPECT_EQ(readGuardZone(blade(40.0f, 60.0f, 10.0f)), GuardRead::Right); // beside the defender's right
    EXPECT_EQ(readGuardZone(blade(-40.0f, 60.0f, 10.0f)), GuardRead::Left);
    EXPECT_EQ(readGuardZone(blade(10.0f, 60.0f, -45.0f)), GuardRead::Low); // below the belt: rising
    EXPECT_EQ(readGuardZone(blade(0.0f, 60.0f, 0.0f)), GuardRead::Middle);
}

TEST(RagdollCoreDuelReadTest, aFastBladeIsReadWhereItArrives)
{
    // A chop seen high in front, coming down: still high on arrival.
    EXPECT_EQ(readGuardZone(blade(0.0f, 70.0f, 60.0f, 0.0f, -300.0f, -150.0f)), GuardRead::High);
    // A forehand from the defender's right sweeping across: arrives at the right.
    EXPECT_EQ(readGuardZone(blade(70.0f, 50.0f, 5.0f, -400.0f, -200.0f, 0.0f)), GuardRead::Right);
    // Rising from low on the left: low until it arrives.
    EXPECT_EQ(readGuardZone(blade(-20.0f, 60.0f, -60.0f, 50.0f, -350.0f, 60.0f)), GuardRead::Low);
    // A thrust down the chest axis.
    EXPECT_EQ(readGuardZone(blade(2.0f, 90.0f, 5.0f, 0.0f, -600.0f, 0.0f)), GuardRead::Middle);
}

TEST(RagdollCoreDuelReadTest, theErrorTurnsTheReadAboutTheChest)
{
    GuardReadInput in = blade(40.0f, 60.0f, 10.0f); // a side cut from the right
    EXPECT_EQ(readGuardZone(in), GuardRead::Right);
    in.mErrorDegrees = 10.0f;
    in.mDraw = 1.0f;
    EXPECT_EQ(readGuardZone(in), GuardRead::Right); // a skilled defender errs a little
    in.mErrorDegrees = 50.0f;
    EXPECT_EQ(readGuardZone(in), GuardRead::High); // a clumsy one guards the wrong place
    in.mErrorDegrees = 80.0f;
    in.mDraw = -1.0f;
    EXPECT_EQ(readGuardZone(in), GuardRead::Low);
}

TEST(RagdollCoreDuelReadTest, aSkilledAttackerCutsWhereTheGuardIsNot)
{
    constexpr float q = 3.14159265f * 0.25f;
    const float lines[8] = { 0.0f, 2.0f * q, 4.0f * q, q, 6.0f * q, 3.0f * q, 7.0f * q, 5.0f * q };
    RagdollCore::LineChoiceInput in;
    in.mLines = lines;
    in.mCount = 8;
    in.mBestChance = 1.0f;
    in.mDraw = 0.0f;
    in.mTurnLine = 2.0f * q; // the chop
    in.mGuardZone = static_cast<int>(GuardRead::High);
    const float chosen = RagdollCore::chooseAttackLine(in);
    EXPECT_EQ(RagdollCore::lineCover(in.mGuardZone, chosen), 0.0f);
    // Guard low: not a rising cut.
    in.mGuardZone = static_cast<int>(GuardRead::Low);
    EXPECT_EQ(RagdollCore::lineCover(in.mGuardZone, RagdollCore::chooseAttackLine(in)), 0.0f);
    // The foe's blade held over its right shoulder (attacker's left, high): not
    // the high diagonal from there.
    in.mGuardZone = -1;
    in.mBladeKnown = true;
    in.mBladeX = -25.0f;
    in.mBladeZ = 25.0f;
    EXPECT_NE(RagdollCore::chooseAttackLine(in), 3.0f * q);
    // Repeating is dearer.
    in.mBladeKnown = false;
    in.mRecent[0] = 0.0f;
    EXPECT_NE(RagdollCore::chooseAttackLine(in), 0.0f);
    // Unskilled: the line in turn.
    in.mGuardZone = static_cast<int>(GuardRead::High);
    in.mBestChance = 0.2f;
    in.mDraw = 0.5f;
    EXPECT_EQ(RagdollCore::chooseAttackLine(in), 2.0f * q);
}

TEST(RagdollCoreDuelReadTest, aFastBladeTellsWhenItArrives)
{
    float arrival = 0.0f;
    // 70 out, closing at 300: (70 - 25) / 300 = 0.15 s.
    readGuardZone(blade(0.0f, 70.0f, 40.0f, 0.0f, -300.0f, -100.0f), &arrival);
    EXPECT_NEAR(arrival, 0.15f, 1e-3f);
    readGuardZone(blade(0.0f, 70.0f, 40.0f), &arrival); // a wind-up: no arrival
    EXPECT_LT(arrival, 0.0f);
}
