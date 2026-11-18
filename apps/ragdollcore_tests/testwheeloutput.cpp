#include <ragdollcore/wheeloutput.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace
{
    using namespace RagdollCore;

    constexpr float sPi = std::numbers::pi_v<float>;
    constexpr float sDt = 1.0f / 60.0f;

    struct Rig
    {
        WheelPlanner mPlanner;
        WheelStriker mStriker;
        WheelActor mActor;
        WheelHooks mHooks;
        std::optional<WheelAimInput> mAim;
        WheelInput mInput;
        float mClickAngle = 0.5f * sPi;

        Rig()
        {
            mStriker.mAxes = { JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX() };
            mHooks.mClickLine = [this] { return mClickAngle; };
            mInput.mDt = sDt;
        }

        void step(bool held)
        {
            mInput.mHeld = held;
            WheelEvents events;
            mPlanner.step(mInput, mStriker, mActor, mHooks, mAim ? &*mAim : nullptr, events);
        }

        void click()
        {
            step(true);
            step(true);
            step(false);
        }

        void until(WheelStage stage, float limit = 3.0f)
        {
            for (float t = 0.0f; mPlanner.stage() != stage && t < limit; t += sDt)
                step(false);
        }
    };

    TEST(WheelOutputTest, TheDriveIsStiffInTheCutSoftInTheFollowThrough)
    {
        Rig rig;
        rig.click();
        rig.until(WheelStage::Cut);
        float stiffest = 0.0f, softest = 2.0f;
        while (rig.mPlanner.stage() == WheelStage::Cut)
        {
            rig.step(false);
            stiffest = std::max(stiffest, rig.mPlanner.drivePhase());
            if (rig.mPlanner.flow().mU > 1.8f)
                softest = std::min(softest, rig.mPlanner.drivePhase());
        }
        EXPECT_GT(stiffest, 1.4f);
        EXPECT_LT(softest, 1.0f);
        rig.until(WheelStage::Guard);
        rig.step(false);
        EXPECT_NEAR(rig.mPlanner.drivePhase(), 1.0f, 1e-4f);
    }

    TEST(WheelOutputTest, TwoHandersKeepAFirmFollowThroughAndALaggingTipKeepsItStiff)
    {
        const auto softest = [](bool twoHanded, std::optional<float> gap) {
            Rig rig;
            rig.mStriker.mTwoHanded = twoHanded;
            rig.mInput.mTipGap = gap;
            rig.click();
            rig.until(WheelStage::Cut);
            float low = 2.0f;
            while (rig.mPlanner.stage() == WheelStage::Cut)
            {
                rig.step(false);
                low = std::min(low, rig.mPlanner.drivePhase());
            }
            return low;
        };
        EXPECT_GT(softest(true, std::nullopt), 0.95f);
        EXPECT_GT(softest(false, 60.0f), 1.0f);
    }

    TEST(WheelOutputTest, AClashThrowsTheHandAndEndsTheStroke)
    {
        Rig plain, knocked;
        for (Rig* rig : { &plain, &knocked })
        {
            rig->click();
            rig->until(WheelStage::Cut);
        }
        knocked.mPlanner.beginKnock(JPH::Vec3(0.0f, 0.0f, 10.0f), true, false, 1.0f);
        plain.step(false);
        knocked.step(false);
        // The stroke ends: the return from the thrown pose.
        EXPECT_EQ(knocked.mPlanner.stage(), WheelStage::Recover);
        EXPECT_TRUE(knocked.mPlanner.knocking());
        // Out in 0.07 s, the drive let go by half.
        float peak = 0.0f, loosest = 2.0f;
        for (int i = 0; i < 6; ++i)
        {
            knocked.step(false);
            peak = std::max(peak, knocked.mPlanner.lastPose()->mHand.GetZ());
            loosest = std::min(loosest, knocked.mPlanner.drivePhase());
        }
        EXPECT_LT(loosest, 0.6f);
        // Back over 0.35 s at full control.
        for (int i = 0; i < 30; ++i)
            knocked.step(false);
        EXPECT_FALSE(knocked.mPlanner.knocking());
    }

    TEST(WheelOutputTest, AnEvenBounceChainsFromTheRebound)
    {
        Rig rig;
        rig.click();
        rig.until(WheelStage::Cut);
        rig.mPlanner.beginKnock(JPH::Vec3(5.0f, 0.0f, 0.0f), true, true, 1.0f);
        rig.step(true);
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Guard);
        EXPECT_TRUE(rig.mPlanner.chaining());
    }

    TEST(WheelOutputTest, AThrustBendsTheTrunkForward)
    {
        Rig rig;
        rig.mAim = WheelAimInput{};
        rig.mClickAngle = 1.5f * sPi;
        rig.click();
        float most = 0.0f;
        for (int i = 0; i < 60; ++i)
        {
            rig.step(false);
            most = std::min(most, rig.mPlanner.trunk().mLean);
        }
        // Up to 0.22 rad forward at the full drive (positive lean bends back).
        EXPECT_LT(most, -0.15f);
        EXPECT_GE(most, -0.22f - 1e-4f);
    }

    TEST(WheelOutputTest, ASlashTurnsTheChestOnPastItsEnd)
    {
        Rig rig;
        rig.mClickAngle = 0.0f; // the forehand: across right to left
        rig.mInput.mSwingSpeed = 1200.0f;
        rig.click();
        float most = 0.0f;
        for (int i = 0; i < 90; ++i)
        {
            rig.step(false);
            most = std::min(most, rig.mPlanner.trunk().mTwist - rig.mPlanner.lastPose()->mTwist);
        }
        EXPECT_NEAR(most, -18.0f * sPi / 180.0f, 0.01f);
    }

    TEST(WheelOutputTest, TheTorsoHelpsAHandThatLags)
    {
        Rig rig;
        rig.mInput.mHand = JPH::Vec3(-30.0f, 10.0f, -20.0f); // far from any target
        rig.click();
        rig.until(WheelStage::Cut);
        const float twist = rig.mPlanner.trunk().mTwist - rig.mPlanner.lastPose()->mTwist;
        EXPECT_NE(twist, 0.0f);
        EXPECT_LE(std::abs(twist), 0.5f + 1e-5f);
        // Out of the attack it comes back.
        rig.mInput.mHand.reset();
        rig.until(WheelStage::Guard);
        for (int i = 0; i < 40; ++i)
            rig.step(false);
        EXPECT_NEAR(rig.mPlanner.trunk().mTwist - rig.mPlanner.lastPose()->mTwist, 0.0f, 1e-4f);
    }

    TEST(WheelOutputTest, TheChestTurnsTowardAnAimedFoe)
    {
        Rig rig;
        WheelAimInput aim;
        const float side = 15.0f * sPi / 180.0f;
        aim.mFoe = JPH::Vec3(60.0f * std::sin(side), 60.0f * std::cos(side), 0.0f);
        rig.mAim = aim;
        rig.step(true);
        for (int i = 0; i < 10; ++i)
        {
            WheelEvents events;
            rig.mInput.mHeld = true;
            rig.mInput.mDragUp = 40.0f;
            rig.mPlanner.step(rig.mInput, rig.mStriker, rig.mActor, rig.mHooks, &*rig.mAim, events);
        }
        rig.mInput.mDragUp = 0.0f;
        for (int i = 0; i < 20; ++i)
            rig.step(true);
        ASSERT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        const float twist = rig.mPlanner.trunk().mTwist - rig.mPlanner.lastPose()->mTwist;
        EXPECT_NEAR(twist, side, 0.02f);
    }

    TEST(WheelOutputTest, OverADownedFoeTheBodyStoops)
    {
        Rig rig;
        WheelAimInput aim;
        aim.mFoe = JPH::Vec3(0.0f, 50.0f, -60.0f);
        aim.mFoeDowned = true;
        rig.mAim = aim;
        for (int i = 0; i < 30; ++i)
            rig.step(false);
        EXPECT_NEAR(rig.mPlanner.trunk().mLow, 1.0f, 1e-5f);
        EXPECT_NEAR(rig.mPlanner.trunk().mLean, -0.6f, 1e-4f);
    }

    TEST(WheelOutputTest, FirstPersonClearsTheCrosshair)
    {
        ChamberPose pose;
        pose.mHand = JPH::Vec3(0.0f, 30.0f, 0.0f);
        clearFirstPersonView(pose, WheelStage::Guard, 0.0f);
        EXPECT_NEAR(std::atan2(pose.mHand.GetX(), pose.mHand.GetY()), 12.0f * sPi / 180.0f, 1e-4f);
        pose = {};
        pose.mHand = JPH::Vec3(0.0f, 30.0f, 0.0f);
        clearFirstPersonView(pose, WheelStage::Wind, 0.25f);
        EXPECT_NEAR(std::atan2(pose.mHand.GetX(), pose.mHand.GetY()), 6.0f * sPi / 180.0f, 1e-4f);
        pose = {};
        pose.mHand = JPH::Vec3(0.0f, 30.0f, 0.0f);
        clearFirstPersonView(pose, WheelStage::Cut, 1.0f);
        EXPECT_FLOAT_EQ(pose.mHand.GetX(), 0.0f);
    }

    TEST(WheelOutputTest, TheHaftStaysOutOfTheBody)
    {
        // A staff held by its middle, pointing forward: its butt runs back
        // through the belly.
        ChamberPose pose;
        pose.mHand = JPH::Vec3(4.0f, 20.0f, -10.0f);
        const float turned = keepHaftOutOfBody(pose, JPH::Vec3::sAxisY(), 60.0f);
        EXPECT_GT(turned, 0.0f);
        const JPH::Vec3 blade = pose.mRotation * JPH::Vec3::sAxisY();
        const JPH::Vec3 butt = pose.mHand - blade * 60.0f;
        // The butt is out to the side now.
        EXPECT_GT(std::abs(butt.GetX()), 15.0f);
        // A sword's (little behind the hand) is left alone.
        ChamberPose sword;
        sword.mHand = pose.mHand;
        EXPECT_FLOAT_EQ(keepHaftOutOfBody(sword, JPH::Vec3::sAxisY(), 10.0f), 0.0f);
    }

    // K2_HAFT_GRIP: the support point slides toward the shoulder, within reach and the haft.
    TEST(WheelOutputTest, TheSupportHandSlidesAlongTheHaft)
    {
        HaftGripSettings settings;
        settings.mShare = 1.0f;
        const JPH::Vec3 primary(0, 0, 0), authored(0, -22, 0);
        // The shoulder level with a point 40 behind the hand, 10 off the haft.
        float target = haftGripTarget(settings, primary, authored, JPH::Vec3(10, -40, 0), 60.0f, 80.0f);
        EXPECT_NEAR(target, 40.0f, 1e-3f);
        // Never past the haft's end behind the hand.
        target = haftGripTarget(settings, primary, authored, JPH::Vec3(10, -100, 0), 60.0f, 50.0f);
        EXPECT_NEAR(target, 50.0f, 1e-3f);
        // Never over the weapon hand: the shoulder ahead of it keeps a hand's width.
        target = haftGripTarget(settings, primary, authored, JPH::Vec3(10, 30, 0), 60.0f, 80.0f);
        EXPECT_NEAR(target, settings.mMinimum, 1e-3f);
        // Off: the authored spacing.
        settings.mShare = 0.0f;
        EXPECT_NEAR(haftGripTarget(settings, primary, authored, JPH::Vec3(10, -40, 0), 60.0f, 80.0f), 22.0f, 1e-3f);
        // Part of the way, then held within the arm's reach of the shoulder.
        settings.mShare = 0.5f;
        EXPECT_NEAR(haftGripTarget(settings, primary, authored, JPH::Vec3(10, -40, 0), 60.0f, 80.0f), 31.0f, 1e-3f);
        settings.mShare = 1e-3f;
        // Reach 26 from a shoulder 10 off the haft at 60 behind: 60 - 24 = 36 at least.
        EXPECT_NEAR(haftGripTarget(settings, primary, authored, JPH::Vec3(10, -60, 0), 26.0f, 80.0f), 36.0f, 1e-2f);
        // The step is bounded by speed x dt.
        EXPECT_NEAR(haftGripStep(22.0f, 40.0f, 60.0f, sDt), 23.0f, 1e-4f);
        EXPECT_NEAR(haftGripStep(22.0f, 22.5f, 60.0f, sDt), 22.5f, 1e-4f);
    }

    // K2_HAFT_PIVOT: the point between the hands stays where the reference put it.
    TEST(WheelOutputTest, AHaftTurnsAboutThePointBetweenTheHands)
    {
        const JPH::Vec3 blade = JPH::Vec3::sAxisY();
        const float support = -24.0f; // behind the weapon hand
        ChamberPose pose;
        pose.mHand = JPH::Vec3(10, 30, 0);
        pose.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.5f * sPi);
        const JPH::Quat reference = JPH::Quat::sIdentity();
        const JPH::Vec3 plannedMid = pose.mHand + reference * blade * (0.5f * support);
        pivotBetweenHands(pose, reference, blade, support, 1.0f);
        const JPH::Vec3 mid = pose.mHand + pose.mRotation * blade * (0.5f * support);
        EXPECT_TRUE(mid.IsClose(plannedMid, 1e-4f));
        // At the reference turn nothing moves; off, nothing moves.
        ChamberPose still;
        still.mHand = JPH::Vec3(10, 30, 0);
        EXPECT_TRUE(pivotBetweenHands(still, reference, blade, support, 1.0f).IsNearZero());
        ChamberPose off = pose;
        EXPECT_TRUE(pivotBetweenHands(off, reference, blade, support, 0.0f).IsNearZero());
    }

    TEST(WheelOutputTest, AnNpcGuardIsHeldInItsOwnFrame)
    {
        NpcGuardHold hold;
        JPH::RVec3 target(10.0, 20.0, 30.0);
        JPH::Quat rotation = JPH::Quat::sIdentity();
        EXPECT_TRUE(hold.step(true, sDt, JPH::RVec3::sZero(), 0.0f, target, rotation).IsNearZero());
        // The wobbly body throws the target 6 units; the hold follows a
        // little (dt / 0.35 of it).
        JPH::RVec3 bounced(16.0, 20.0, 30.0);
        const JPH::Vec3 shift = hold.step(true, sDt, JPH::RVec3::sZero(), 0.0f, bounced, rotation);
        EXPECT_NEAR(bounced.GetX(), 10.0 + 6.0 * (sDt / 0.35f), 1e-3);
        EXPECT_NEAR(shift.GetX(), -6.0f * (1.0f - sDt / 0.35f), 1e-3f);
        // The actor walking carries it along unchanged.
        JPH::RVec3 walked(110.0, 20.0, 30.0);
        EXPECT_LT(hold.step(true, sDt, JPH::RVec3(100.0, 0.0, 0.0), 0.0f, walked, rotation).Length(), 45.0f);
        EXPECT_NEAR(walked.GetX(), 100.0 + 10.0 + 6.0 * (sDt / 0.35f), 0.2);
        // In an attack it lets go.
        JPH::RVec3 attack(50.0, 20.0, 30.0);
        EXPECT_TRUE(hold.step(false, sDt, JPH::RVec3::sZero(), 0.0f, attack, rotation).IsNearZero());
        EXPECT_DOUBLE_EQ(attack.GetX(), 50.0);
    }
}
