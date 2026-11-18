#include <ragdollcore/fistcontrol.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

namespace
{
    using namespace RagdollCore;

    TEST(FistControlTest, CrosshairAimKeepsBothShouldersOnTheCameraRayWithinReach)
    {
        const JPH::Vec3 eye(0.0f, -5.0f, 18.0f);
        for (float side : { -12.0f, 12.0f })
            for (float pitch : { -0.35f, 0.0f, 0.25f })
            {
                const JPH::Vec3 shoulder(side, 0.0f, 0.0f);
                const JPH::Vec3 ray = JPH::Vec3(0.12f, 1.0f, pitch).Normalized();
                const JPH::Vec3 result = aimFistTarget(JPH::Vec3(-side, 40, 8), eye - shoulder, ray, 36, 1);
                const JPH::Vec3 fromEye = shoulder + result - eye;
                EXPECT_LT(fromEye.Cross(ray).Length(), 0.001f);
                EXPECT_GT(fromEye.Dot(ray), 0.0f);
                EXPECT_LE(result.Length(), 36.001f);
            }
    }

    TEST(FistControlTest, CrosshairAimFollowsMovingShouldersAndRotatedViews)
    {
        const JPH::Vec3 eye(0, -10, 20), ray = JPH::Vec3(0.1f, 1, -0.1f).Normalized();
        const JPH::Vec3 target(10, 35, 8);
        const auto turn = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.8f);
        for (float x : { -8.0f, 0.0f, 8.0f })
        {
            const JPH::Vec3 shoulder(x, 0, 0);
            const JPH::Vec3 result = aimFistTarget(target, eye - shoulder, ray, 40, 1);
            EXPECT_LT((shoulder + result - eye).Cross(ray).Length(), 0.001f);
            const JPH::Vec3 rotated = aimFistTarget(turn * target, turn * (eye - shoulder), turn * ray, 40, 1);
            EXPECT_LT((rotated - turn * result).Length(), 0.001f);
        }
    }

    TEST(FistControlTest, UnreachableCrosshairAndPartialCorrectionNeverExtendTheArm)
    {
        const JPH::Vec3 target(10, 30, 0), origin(0, 0, 100), ray = JPH::Vec3::sAxisY();
        for (float weight : { 0.1f, 0.5f, 1.0f })
        {
            const auto result = aimFistTarget(target, origin, ray, 28, weight);
            EXPECT_FALSE(result.IsNaN());
            EXPECT_LE(result.Length(), 28.001f);
        }
        EXPECT_EQ(aimFistTarget(target, origin, ray, 28, 0), target);
        EXPECT_EQ(aimFistTarget(target, origin, JPH::Vec3::sZero(), 28, 1), target);
        EXPECT_EQ(aimFistTarget(target, origin, ray, std::numeric_limits<float>::quiet_NaN(), 1), target);
    }

    TEST(FistControlTest, EveryPunchKindCanFinishOnTheRay)
    {
        for (auto kind : { PunchKind::Straight, PunchKind::Body, PunchKind::Hook, PunchKind::Uppercut })
            for (auto hand : { FistHand::Left, FistHand::Right })
            {
                const auto timing = punchTiming(kind, 0.5f, 1.0f);
                const auto pose = fistPunchPose(kind, hand, timing, timing.mWind + timing.mCut - 0.001f);
                const auto result = aimFistTarget(pose.mHand, JPH::Vec3(0, -10, 10), JPH::Vec3::sAxisY(), 40, 1);
                EXPECT_NEAR(result.GetX(), 0.0f, 0.001f);
                EXPECT_NEAR(result.GetZ(), 10.0f, 0.001f);
                EXPECT_LE(result.Length(), 40.001f);
            }
    }

    TEST(FistControlTest, AFistEasesFromItsPunchsEndToWhereItIsHeld)
    {
        const JPH::Vec3 from(-6, 14, 13), hold(-20, 20, 15);
        // It starts where the punch left it, ends where it is held, and no
        // step of a 60 Hz run is longer than a fifth of the way.
        EXPECT_TRUE(easeFistToHold(from, hold, 0.0f, 0.12f).IsClose(from, 1e-6f));
        EXPECT_TRUE(easeFistToHold(from, hold, 0.12f, 0.12f).IsClose(hold, 1e-6f));
        EXPECT_TRUE(easeFistToHold(from, hold, 1.0f, 0.12f).IsClose(hold, 1e-6f));
        JPH::Vec3 last = from;
        float longest = 0.0f;
        for (int step = 1; step <= 8; ++step)
        {
            const JPH::Vec3 at = easeFistToHold(from, hold, step / 60.0f, 0.12f);
            longest = std::max(longest, (at - last).Length());
            last = at;
        }
        EXPECT_TRUE(last.IsClose(hold, 1e-6f));
        EXPECT_LT(longest, 0.25f * (hold - from).Length());
        // Off, or nothing to ease from: the held place at once.
        EXPECT_TRUE(easeFistToHold(from, hold, 0.0f, 0.0f).IsClose(hold, 1e-6f));
        EXPECT_TRUE(easeFistToHold(JPH::Vec3::sNaN(), hold, 0.0f, 0.12f).IsClose(hold, 1e-6f));
    }

    TEST(FistControlTest, AWoundHookOrUppercutStrikesNoQuickerThanItsSeconds)
    {
        for (auto kind : { PunchKind::Hook, PunchKind::Uppercut })
        {
            EXPECT_FLOAT_EQ(woundStrikeSeconds(0.10f, kind, 0.18f), 0.18f);
            EXPECT_FLOAT_EQ(woundStrikeSeconds(0.25f, kind, 0.18f), 0.25f); // never quicker than it was
            EXPECT_FLOAT_EQ(woundStrikeSeconds(0.10f, kind, 0.0f), 0.10f); // off
            EXPECT_FLOAT_EQ(woundWindShare(kind, 1.0f), 1.0f);
            EXPECT_FLOAT_EQ(woundWindShare(kind, 0.0f), 0.5f); // off: the half it has
        }
        // Straights and body shots keep their times.
        for (auto kind : { PunchKind::Straight, PunchKind::Body })
        {
            EXPECT_FLOAT_EQ(woundStrikeSeconds(0.10f, kind, 0.18f), 0.10f);
            EXPECT_FLOAT_EQ(woundWindShare(kind, 1.0f), 0.5f);
        }
    }

    TEST(FistControlTest, AFlicksDirectionPicksThePunchAndItsSideTheHand)
    {
        const auto pick = [](float right, float up) { return *classifyFlick(right, up, false, FistHand::Left); };
        EXPECT_EQ(pick(-300, 300).mHand, FistHand::Left); // up-left: the jab
        EXPECT_EQ(pick(-300, 300).mKind, PunchKind::Straight);
        EXPECT_EQ(pick(300, 300).mHand, FistHand::Right); // up-right: the cross
        EXPECT_EQ(pick(300, 300).mKind, PunchKind::Straight);
        EXPECT_EQ(pick(-400, 0).mHand, FistHand::Right); // to the left: the right hook
        EXPECT_EQ(pick(-400, 0).mKind, PunchKind::Hook);
        EXPECT_EQ(pick(400, 0).mHand, FistHand::Left); // to the right: the left hook
        EXPECT_EQ(pick(400, 0).mKind, PunchKind::Hook);
        EXPECT_EQ(pick(-300, -300).mKind, PunchKind::Uppercut);
        EXPECT_EQ(pick(-300, -300).mHand, FistHand::Left);
        EXPECT_EQ(pick(300, -300).mHand, FistHand::Right);
        EXPECT_EQ(pick(0, -400).mKind, PunchKind::Body);
        // Straight up and straight down follow the alternation.
        EXPECT_EQ(classifyFlick(0, 400, false, FistHand::Right)->mHand, FistHand::Right);
        EXPECT_EQ(classifyFlick(0, 400, false, FistHand::Left)->mHand, FistHand::Left);
    }

    TEST(FistControlTest, ShortFlicksAreNoPunchLongOnesHaymakersAndCrouchedStraightsGoLow)
    {
        EXPECT_FALSE(classifyFlick(50, 50, false, FistHand::Left).has_value());
        EXPECT_NEAR(classifyFlick(0, 120, false, FistHand::Left)->mPower, 0.0f, 1e-4f);
        EXPECT_NEAR(classifyFlick(0, 2000, false, FistHand::Left)->mPower, 1.0f, 1e-4f);
        EXPECT_EQ(classifyFlick(300, 300, true, FistHand::Left)->mKind, PunchKind::Body);
        EXPECT_EQ(classifyFlick(-400, 0, true, FistHand::Left)->mKind, PunchKind::Hook);
    }

    TEST(FistControlTest, AHaymakerWindsLongerStrikesQuickerAndTires)
    {
        const PunchTiming quick = punchTiming(PunchKind::Hook, 0.0f, 1.0f);
        const PunchTiming hay = punchTiming(PunchKind::Hook, 1.0f, 1.0f);
        EXPECT_NEAR(hay.mWind, 2.5f * quick.mWind, 1e-5f);
        EXPECT_LT(hay.mCut, quick.mCut);
        EXPECT_NEAR(hay.mFatigue, 3.0f * quick.mFatigue, 1e-5f);
        EXPECT_GT(punchTiming(PunchKind::Hook, 0.0f, 0.0f).mWind, quick.mWind);
    }

    TEST(FistControlTest, TheLeftHandMirrorsTheRightAcrossTheBody)
    {
        const PunchTiming timing = punchTiming(PunchKind::Hook, 0.0f, 1.0f);
        const float mid = timing.mWind + 0.5f * timing.mCut;
        const FistPunchPose right = fistPunchPose(PunchKind::Hook, FistHand::Right, timing, mid);
        const FistPunchPose left = fistPunchPose(PunchKind::Hook, FistHand::Left, timing, mid);
        EXPECT_NEAR(left.mHand.GetX(), -right.mHand.GetX(), 1e-4f);
        EXPECT_NEAR(left.mHand.GetY(), right.mHand.GetY(), 1e-4f);
        EXPECT_NEAR(left.mTwistDegrees, -right.mTwistDegrees, 1e-4f);
        EXPECT_TRUE(left.mStriking);
    }

    TEST(FistControlTest, APunchGoesOutStrikesAndComesBackToItsGuard)
    {
        const PunchTiming timing = punchTiming(PunchKind::Straight, 0.0f, 1.0f);
        const FistPunchPose start = fistPunchPose(PunchKind::Straight, FistHand::Left, timing, 0.0f);
        const FistPunchPose out = fistPunchPose(PunchKind::Straight, FistHand::Left, timing, timing.mWind + timing.mCut);
        const FistPunchPose done
            = fistPunchPose(PunchKind::Straight, FistHand::Left, timing, timing.mWind + timing.mCut + timing.mBack + 0.01f);
        EXPECT_FALSE(start.mStriking);
        EXPECT_GT(out.mHand.GetY(), start.mHand.GetY() + 30.0f); // the straight reaches out
        EXPECT_TRUE(done.mDone);
        EXPECT_FALSE(done.mStriking);
        EXPECT_TRUE(done.mHand.IsClose(start.mHand, 1e-3f));
    }
}
