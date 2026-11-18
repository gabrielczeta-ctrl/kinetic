#include <ragdollcore/wheelplanner.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace
{
    using namespace RagdollCore;

    constexpr float sPi = std::numbers::pi_v<float>;
    constexpr float sDt = 1.0f / 60.0f;

    // One fighter's hand on the wheel, stepped at 60 Hz.
    struct Rig
    {
        WheelPlanner mPlanner;
        WheelStriker mStriker;
        WheelActor mActor;
        WheelHooks mHooks;
        std::optional<WheelAimInput> mAim;
        float mClickAngle = 0.5f * sPi; // a standing click: the chop
        int mSteps = 0;
        std::optional<JPH::Vec3> mShoulder; // the weapon arm's shoulder (K2_CUT_REACH)
        std::optional<JPH::Vec3> mThreat; // an incoming blade (K2_GUARD_THREAT)
        std::optional<JPH::Vec3> mThreatStrike; // its striking part (K2_PARRY_STEER)
        JPH::Vec3 mThreatVelocity = JPH::Vec3::sZero();
        bool mShove = false; // K2_SHOVE_ARMS
        float mArmLength = 55.0f;
        std::optional<JPH::Vec3> mPoint; // the physical point (K2_POINT_AIM)
        std::optional<FistSteerHold> mFistHold; // K2_FIST_STEER
        std::optional<FistSteerPunch> mFistPunch; // ... thrown on the next step

        Rig()
        {
            mStriker.mAxes = { JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX() };
            mHooks.mClickLine = [this] { return mClickAngle; };
        }

        WheelEvents step(bool held, float dragRight = 0.0f, float dragUp = 0.0f, bool blocking = false,
            WheelGuardZone zone = WheelGuardZone::Middle, bool nativeStarted = false)
        {
            WheelInput input;
            input.mDt = sDt;
            input.mHeld = held;
            input.mDragRight = dragRight;
            input.mDragUp = dragUp;
            input.mBlocking = blocking;
            input.mGuardZone = zone;
            input.mNativeStarted = nativeStarted;
            input.mShoulder = mShoulder;
            input.mThreat = mThreat;
            input.mThreatStrike = mThreatStrike;
            input.mThreatVelocity = mThreatVelocity;
            input.mShove = mShove;
            input.mArmLength = mArmLength;
            input.mPoint = mPoint;
            input.mFistHold = mFistHold;
            input.mFistPunch = mFistPunch;
            mFistPunch.reset();
            WheelEvents events;
            mPlanner.step(input, mStriker, mActor, mHooks, mAim ? &*mAim : nullptr, events);
            ++mSteps;
            return events;
        }

        // A dragged wind-up struck back (the button held): on into the cut
        // until it shows its path.
        void releaseToPath()
        {
            for (int i = 0; i < 8 && mPlanner.stage() != WheelStage::Cut; ++i)
                step(true, 0.0f, -100.0f);
            for (int i = 0; i < 60 && mPlanner.preview().empty(); ++i)
                step(true);
        }

        // Steps until the stage is `stage` (at most `limit` s); the time taken.
        float until(WheelStage stage, float limit = 3.0f, bool held = false)
        {
            float t = 0.0f;
            while (mPlanner.stage() != stage && t < limit)
            {
                step(held);
                t += sDt;
            }
            return t;
        }

        // A tap: pressed for two steps, released without a drag.
        WheelEvents click()
        {
            step(true);
            step(true);
            return step(false);
        }

        float handTo(const JPH::Vec3& point) const { return (mPlanner.lastPose()->mHand - point).Length(); }
    };

    TEST(WheelPlannerTest, AClickPlaysThroughHoldsAndReturns)
    {
        Rig rig;
        rig.step(false);
        ASSERT_EQ(rig.mPlanner.stage(), WheelStage::Guard);
        const WheelEvents events = rig.click();
        ASSERT_TRUE(events.mClick);
        EXPECT_NEAR(events.mClickAngle, 0.5f * sPi, 1e-5f);
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        EXPECT_TRUE(rig.mPlanner.autoAttack());

        // The longsword's wind-up (0.203 s) and then the flow's arrival.
        PaceInput input;
        const Pace pace = attackPace(input);
        const float toCut = rig.until(WheelStage::Cut);
        EXPECT_GE(toCut, pace.mWind - 2.0f * sDt);
        EXPECT_LT(toCut, pace.mWind + 0.12f);
        // The cut (0.145 s), the hold (0.3 s) and the return (0.4 s).
        const float toRecover = rig.until(WheelStage::Recover);
        EXPECT_NEAR(toRecover, pace.mCut + pace.mReleaseHold, 0.1f);
        const float toGuard = rig.until(WheelStage::Guard);
        EXPECT_NEAR(toGuard, pace.mReturn, 2.0f * sDt);
        const auto guard = guardPose(weaponLines(), rig.mStriker.mAxes);
        ASSERT_TRUE(guard);
        EXPECT_LT(rig.handTo(guard->mHand), 1e-3f);
    }

    TEST(WheelPlannerTest, ACutBeginningIsAnAttack)
    {
        Rig rig;
        rig.click();
        bool started = false;
        for (int i = 0; i < 60 && !started; ++i)
        {
            const WheelEvents events = rig.step(false);
            if (events.mCutStarted)
            {
                started = true;
                EXPECT_FLOAT_EQ(events.mSwishSize, 1.0f);
                EXPECT_FLOAT_EQ(events.mFatigueShare, 1.0f);
            }
        }
        EXPECT_TRUE(started);
    }

    TEST(WheelPlannerTest, AnNpcHoldsTheChamberBeforeItCuts)
    {
        // The tell: an NPC holds its chamber 0.15 s, and winds up 1.6x slower.
        const auto toCut = [](bool player) {
            Rig rig;
            rig.mActor.mPlayer = player;
            rig.step(false);
            rig.step(false, 0.0f, 0.0f, false, WheelGuardZone::Middle, true);
            return rig.until(WheelStage::Cut);
        };
        const float player = toCut(true);
        const float npc = toCut(false);
        EXPECT_GT(npc, player + 0.15f);
    }

    TEST(WheelPlannerTest, ADragWindsUpAndAStanceStays)
    {
        Rig rig;
        rig.step(true);
        for (int i = 0; i < 5; ++i)
            rig.step(true, 0.0f, 30.0f); // 150 counts up
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        EXPECT_NEAR(rig.mPlanner.angle(), 0.5f * sPi, 1e-4f);
        EXPECT_NEAR(rig.mPlanner.wind(), windAt(150.0f), 1e-5f);
        // Let go inside the circle: the stance stays.
        for (int i = 0; i < 90; ++i)
            rig.step(false);
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        EXPECT_FALSE(rig.mPlanner.autoAttack());
    }

    TEST(WheelPlannerTest, APressThatDoesNotMoveIsAClickNotADrag)
    {
        Rig rig;
        rig.step(true);
        rig.step(true, 10.0f, 10.0f); // 14 counts: under the 40 that pick a line
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Guard);
        EXPECT_TRUE(rig.step(false).mClick);
    }

    // Winds up past the circle (1200 counts up), then strokes back down fast.
    void windDeepAndCut(Rig& rig)
    {
        rig.step(true);
        for (int i = 0; i < 12; ++i)
            rig.step(true, 0.0f, 100.0f);
        ASSERT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        ASSERT_GT(rig.mPlanner.wind(), 0.5f);
        for (int i = 0; i < 4 && rig.mPlanner.stage() != WheelStage::Cut; ++i)
            rig.step(true, 0.0f, -100.0f); // 6000 counts/s inward
    }

    TEST(WheelPlannerTest, AFastStrokeBackFromDeepCuts)
    {
        Rig rig;
        windDeepAndCut(rig);
        ASSERT_EQ(rig.mPlanner.stage(), WheelStage::Cut);
        // The line whose wind-up is opposite the stroke (down): the chop.
        EXPECT_NEAR(rig.mPlanner.cutAngle(), 0.5f * sPi, 1e-3f);
        EXPECT_FALSE(rig.mPlanner.autoAttack());
    }

    // K2_THRUST_KEEP: a spear wound up on the thrust line (down) and pushed
    // 39 deg off straight up thrusts; without it the push is a shaft swing.
    TEST(WheelPlannerTest, ASpearsThrustWindUpStaysAThrustWithTheSwitch)
    {
        const auto cutAngle = [](float keep) {
            Rig rig;
            rig.mStriker.mSpearSwings = true;
            rig.mActor.mThrustKeep = keep;
            rig.step(true);
            for (int i = 0; i < 12; ++i)
                rig.step(true, 0.0f, -100.0f);
            EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
            for (int i = 0; i < 4 && rig.mPlanner.stage() != WheelStage::Cut; ++i)
                rig.step(true, 90.0f, 110.0f);
            EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Cut);
            return rig.mPlanner.cutAngle();
        };
        EXPECT_NEAR(cutAngle(1.05f), 1.5f * sPi, 1e-4f);
        EXPECT_GT(std::abs(cutAngle(0.0f) - 1.5f * sPi), 0.5f);
    }

    TEST(WheelPlannerTest, ASlowStrokeBackRetractsInstead)
    {
        Rig rig;
        rig.step(true);
        for (int i = 0; i < 12; ++i)
            rig.step(true, 0.0f, 100.0f);
        const float deep = rig.mPlanner.wind();
        for (int i = 0; i < 30; ++i)
            rig.step(true, 0.0f, -10.0f); // 600 counts/s: slower than a cut
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        EXPECT_LT(rig.mPlanner.wind(), deep);
    }

    TEST(WheelPlannerTest, AFlickInsideTheCircleSnapCuts)
    {
        Rig rig;
        rig.step(true);
        for (int i = 0; i < 10; ++i)
            rig.step(true, 0.0f, 28.0f); // 280 counts up: a guard near the circle's edge
        ASSERT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        for (int i = 0; i < 3 && rig.mPlanner.stage() != WheelStage::Cut; ++i)
            rig.step(true, 0.0f, -120.0f); // 7200 counts/s across the centre
        ASSERT_EQ(rig.mPlanner.stage(), WheelStage::Cut);
        EXPECT_GE(rig.mPlanner.cut(), 0.2f);
    }

    TEST(WheelPlannerTest, ACommittedCutFinishesAfterTheRelease)
    {
        Rig rig;
        windDeepAndCut(rig);
        ASSERT_EQ(rig.mPlanner.stage(), WheelStage::Cut);
        const float before = rig.mPlanner.cut();
        rig.step(false);
        // At least the click's cut speed (0.25 s at the weight's pace).
        PaceInput input;
        const Pace pace = attackPace(input);
        float t = 0.0f;
        while (rig.mPlanner.cut() < 1.0f && rig.mPlanner.stage() == WheelStage::Cut && t < 1.0f)
        {
            rig.step(false);
            t += sDt;
        }
        EXPECT_GT(rig.mPlanner.cut(), before);
        EXPECT_LE(t, 0.25f * pace.mWeightPace + 2.0f * sDt);
        EXPECT_GT(rig.until(WheelStage::Recover), 0.0f);
    }

    TEST(WheelPlannerTest, DraggedBackACutIsAFeintThatCosts)
    {
        Rig rig;
        windDeepAndCut(rig);
        ASSERT_EQ(rig.mPlanner.stage(), WheelStage::Cut);
        float fatigue = 0.0f;
        for (int i = 0; i < 20 && rig.mPlanner.stage() == WheelStage::Cut; ++i)
            fatigue += rig.step(true, 0.0f, 200.0f).mFeintFatigue; // back up the stroke
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        EXPECT_FLOAT_EQ(rig.mPlanner.wind(), 1.0f);
        EXPECT_FLOAT_EQ(fatigue, 4.0f);
    }

    // B14: a held cut, finished, with the button still down.
    TEST(WheelPlannerTest, AHeldFinishedCutRecoversWithTheSwitch)
    {
        for (const bool heldRecover : { false, true })
        {
            Rig rig;
            rig.mActor.mHeldRecover = heldRecover;
            windDeepAndCut(rig);
            ASSERT_EQ(rig.mPlanner.stage(), WheelStage::Cut);
            // Held, the mouse still: off, the follow-through waits for the
            // release; on, it recovers and returns to the guard.
            for (int i = 0; i < 120; ++i)
                rig.step(true);
            if (!heldRecover)
            {
                EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Cut);
                continue;
            }
            EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Guard);
            // Still held: a drag winds up from the guard again.
            for (int i = 0; i < 5; ++i)
                rig.step(true, 0.0f, 30.0f);
            EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
            EXPECT_NEAR(rig.mPlanner.wind(), windAt(150.0f), 1e-4f);
        }
    }

    TEST(WheelPlannerTest, AFinishedCutIsNotDraggedBackWithTheSwitch)
    {
        Rig rig;
        rig.mActor.mHeldRecover = true;
        windDeepAndCut(rig);
        // Until the end of the cut, held.
        for (int i = 0; i < 30 && rig.mPlanner.cut() < 1.0f; ++i)
            rig.step(true);
        ASSERT_GE(rig.mPlanner.cut(), 1.0f);
        // Back up the stroke: no feint, no fatigue; the cut stays finished.
        float fatigue = 0.0f;
        for (int i = 0; i < 10; ++i)
            fatigue += rig.step(true, 0.0f, 200.0f).mFeintFatigue;
        EXPECT_FLOAT_EQ(fatigue, 0.0f);
        EXPECT_NE(rig.mPlanner.stage(), WheelStage::Wind);
        // Before its end a cut dragged back is still a feint.
        Rig early;
        early.mActor.mHeldRecover = true;
        windDeepAndCut(early);
        ASSERT_LT(early.mPlanner.cut(), 1.0f);
        for (int i = 0; i < 20 && early.mPlanner.stage() == WheelStage::Cut; ++i)
            early.step(true, 0.0f, 200.0f);
        EXPECT_EQ(early.mPlanner.stage(), WheelStage::Wind);
    }

    TEST(WheelPlannerTest, DraggingOnPastAHeldCutStillRunsOn)
    {
        Rig rig;
        rig.mActor.mHeldRecover = true;
        windDeepAndCut(rig);
        bool chained = false;
        for (int i = 0; i < 60 && !chained; ++i)
            chained = rig.step(true, 0.0f, -100.0f).mChained; // on down the stroke
        EXPECT_TRUE(chained);
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
    }

    TEST(WheelPlannerTest, ASkilledPressChainsWithMomentum)
    {
        Rig rig;
        rig.mActor.mSkill = 100.0f;
        rig.click();
        rig.until(WheelStage::Cut);
        const WheelEvents events = rig.step(true); // press mid-cut
        EXPECT_TRUE(events.mChained);
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Guard);
        EXPECT_TRUE(rig.mPlanner.chaining());
        // A drag now picks the next line from where the weapon is.
        for (int i = 0; i < 6; ++i)
            rig.step(true, 60.0f, 0.0f);
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        EXPECT_NEAR(rig.mPlanner.angle(), 0.0f, 1e-3f);
    }

    TEST(WheelPlannerTest, AnUnskilledPressWaitsForTheReturn)
    {
        Rig rig;
        rig.mActor.mSkill = 30.0f;
        rig.click();
        rig.until(WheelStage::Cut);
        const WheelEvents events = rig.step(true); // press mid-cut
        EXPECT_FALSE(events.mChained);
        // The click's cut goes on, skips its hold, and the waiting press
        // starts 0.15 into the return, with nothing carried.
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Cut);
        rig.until(WheelStage::Recover, 1.0f, true);
        const float wait = rig.until(WheelStage::Guard, 1.0f, true);
        EXPECT_NEAR(wait, 0.15f * rig.mPlanner.returnTime(), 2.0f * sDt);
        EXPECT_TRUE(rig.mPlanner.chaining());
    }

    TEST(WheelPlannerTest, AClickReleasedWhileWaitingPlaysAtTheGuard)
    {
        Rig rig;
        rig.mActor.mSkill = 30.0f;
        rig.click();
        rig.until(WheelStage::Cut);
        rig.step(true);
        rig.step(false); // released while it waits: queued
        int clicks = 0;
        for (int i = 0; i < 120; ++i)
            clicks += rig.step(false).mClick ? 1 : 0;
        EXPECT_EQ(clicks, 1);
    }

    TEST(WheelPlannerTest, TheBlockAndItsZones)
    {
        Rig rig;
        for (int i = 0; i < 12; ++i)
            rig.step(false, 0.0f, 0.0f, true);
        EXPECT_TRUE(rig.mPlanner.blocking());
        EXPECT_LT(rig.handTo(JPH::Vec3(4.0f, 24.0f, 6.0f)), 1e-3f);
        for (int i = 0; i < 12; ++i)
            rig.step(false, 0.0f, 0.0f, true, WheelGuardZone::High);
        EXPECT_LT(rig.handTo(JPH::Vec3(6.0f, 20.0f, 22.0f)), 1e-3f);
        // Let go: the return to the guard.
        rig.step(false);
        EXPECT_FALSE(rig.mPlanner.blocking());
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Recover);
    }

    TEST(WheelPlannerTest, AnAttackPressEndingTheBlockAttacksFromIt)
    {
        Rig rig;
        for (int i = 0; i < 12; ++i)
            rig.step(false, 0.0f, 0.0f, true);
        rig.step(true); // the block drops with the attack button held
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Guard);
        EXPECT_TRUE(rig.mPlanner.chaining());
        EXPECT_LT(rig.handTo(JPH::Vec3(4.0f, 24.0f, 6.0f)), 1e-3f);
    }

    // "Never twice running" is per cut: the cut after a feint is a swing of
    // its own and never feints; the next attack may.
    TEST(WheelPlannerTest, AnNpcFeintsBySkillOncePerAttackAtMost)
    {
        Rig rig;
        rig.mActor.mPlayer = false;
        rig.mActor.mSkill = 100.0f;
        rig.mActor.mSerial = 12345;
        int feints = 0, attacks = 0;
        for (int attack = 0; attack < 40; ++attack)
        {
            rig.until(WheelStage::Guard);
            rig.step(false, 0.0f, 0.0f, false, WheelGuardZone::Middle, true);
            ++attacks;
            int feinted = 0;
            for (int i = 0; i < 180 && rig.mPlanner.stage() != WheelStage::Recover; ++i)
                feinted += rig.step(false).mNpcFeint ? 1 : 0;
            EXPECT_LE(feinted, 1) << "attack " << attack;
            feints += feinted;
        }
        // A quarter of the cuts at skill 100: a handful in 40 attacks.
        EXPECT_GT(feints, 2);
        EXPECT_LT(feints, 20);
    }

    TEST(WheelPlannerTest, AnOpenedGuardDoesNotBlockUntilItCloses)
    {
        Rig rig;
        rig.mPlanner.openGuard(0.5f);
        // The block key held: nothing for half a second, then the block.
        for (int i = 0; i < 25; ++i)
            rig.step(false, 0.0f, 0.0f, true);
        EXPECT_FALSE(rig.mPlanner.blocking());
        for (int i = 0; i < 10; ++i)
            rig.step(false, 0.0f, 0.0f, true);
        EXPECT_TRUE(rig.mPlanner.blocking());
        // A shorter window does not shorten a longer one.
        rig.mPlanner.openGuard(1.0f);
        rig.mPlanner.openGuard(0.1f);
        EXPECT_FLOAT_EQ(rig.mPlanner.guardOpen(), 1.0f);
    }

    TEST(WheelPlannerTest, ASidewaysCutTurnsTheChestMore)
    {
        // The largest chest turn of one click, with and without K2_SIDE_TURN.
        const auto peakTurn = [](float angle, float sideTurn, float* settled) {
            Rig rig;
            rig.mActor.mSideTurn = sideTurn;
            rig.mClickAngle = angle;
            rig.step(false);
            rig.click();
            float peak = 0.0f;
            for (int i = 0; i < 150; ++i)
            {
                rig.step(false);
                peak = std::max(peak, std::abs(rig.mPlanner.trunk().mTwist));
                EXPECT_TRUE(std::isfinite(rig.mPlanner.trunk().mTwistLead));
            }
            if (settled)
                *settled = std::abs(rig.mPlanner.trunk().mTwist);
            return peak;
        };
        float settled = 1.0f;
        // A forehand (angle 0): nearly twice the keys' turn at 1, and back
        // to square in the guard.
        const float forehand = peakTurn(0.0f, 0.0f, nullptr);
        EXPECT_GT(forehand, 0.3f);
        EXPECT_GT(peakTurn(0.0f, 1.0f, &settled), 1.7f * forehand);
        EXPECT_LT(settled, 0.02f);
        // A backhand (angle pi) the same; a chop (pi/2) unchanged.
        EXPECT_GT(peakTurn(sPi, 1.0f, nullptr), 1.7f * peakTurn(sPi, 0.0f, nullptr));
        EXPECT_NEAR(peakTurn(0.5f * sPi, 1.0f, nullptr), peakTurn(0.5f * sPi, 0.0f, nullptr), 1e-4f);
    }

    TEST(WheelPlannerTest, TheHipsTakeTheirShareOfTheTurn)
    {
        // The same forehand without and with K2_HIP_TURN: the hips turn (no
        // more than 0.45 rad), the spine is asked that much less, and the
        // chest's sum from the actor is the same at every step.
        Rig spine, hips;
        hips.mActor.mHipShare = 0.5f;
        spine.mClickAngle = hips.mClickAngle = 0.0f;
        for (Rig* rig : { &spine, &hips })
        {
            rig->step(false);
            rig->click();
        }
        float most = 0.0f;
        for (int i = 0; i < 150; ++i)
        {
            spine.step(false);
            hips.step(false);
            const WheelTrunk& a = spine.mPlanner.trunk();
            const WheelTrunk& b = hips.mPlanner.trunk();
            EXPECT_FLOAT_EQ(a.mHipTurn, 0.0f);
            EXPECT_LE(std::abs(b.mHipTurn), 0.45f + 1e-6f);
            EXPECT_NEAR(b.mTwist + b.mHipTurn, a.mTwist, 1e-4f);
            EXPECT_NEAR(b.mTwistLead + b.mHipTurn, a.mTwistLead, 1e-4f);
            most = std::max(most, std::abs(b.mHipTurn));
        }
        EXPECT_GT(most, 0.2f);
    }

    TEST(WheelPlannerTest, AnNpcBlocksTheZoneTheBladeComesFromWithTheSwitch)
    {
        // A blade above the head: with K2_GUARD_THREAT an NPC's block takes
        // the high hanging block (hands at head height); without it, or for
        // the player, the middle block.
        const auto blockHandZ = [](bool npc, bool zones, JPH::Vec3 threat) {
            Rig rig;
            rig.mActor.mPlayer = !npc;
            rig.mActor.mThreatZone = zones;
            rig.mThreat = threat;
            rig.step(false);
            for (int i = 0; i < 60; ++i)
                rig.step(false, 0.0f, 0.0f, true);
            return rig.mPlanner.lastPose()->mHand.GetZ();
        };
        const JPH::Vec3 overhead(0.0f, 30.0f, 30.0f);
        EXPECT_GT(blockHandZ(true, true, overhead), 18.0f);
        EXPECT_LT(blockHandZ(true, false, overhead), 12.0f);
        EXPECT_LT(blockHandZ(false, true, overhead), 12.0f);
        // Low and to a side.
        EXPECT_LT(blockHandZ(true, true, JPH::Vec3(0.0f, 30.0f, -30.0f)), -8.0f);
        EXPECT_GT(blockHandZ(true, true, JPH::Vec3(0.0f, 30.0f, 0.0f)), 2.0f); // the middle
    }

    TEST(WheelPlannerTest, TheBlockSteersToTheIncomingBladeWithTheSwitch)
    {
        // The middle block's hand is at (4, 24, 6). A blade's striking part
        // to its right and above, at rest: with K2_PARRY_STEER the hand goes
        // toward it, no further than the switch's units; without, it stays.
        const auto hand = [](float steer, float control, JPH::Vec3 strike, JPH::Vec3 velocity) {
            Rig rig;
            rig.mActor.mParrySteer = steer;
            rig.mActor.mControl = control;
            rig.mThreatStrike = strike;
            rig.mThreatVelocity = velocity;
            rig.step(false);
            for (int i = 0; i < 90; ++i)
                rig.step(false, 0.0f, 0.0f, true);
            return rig.mPlanner.lastPose()->mHand;
        };
        const JPH::Vec3 key(4.0f, 24.0f, 6.0f);
        const JPH::Vec3 farRight(60.0f, 40.0f, 30.0f);
        EXPECT_LT((hand(0.0f, 1.0f, farRight, JPH::Vec3::sZero()) - key).Length(), 1e-3f);
        const JPH::Vec3 steered = hand(12.0f, 1.0f, farRight, JPH::Vec3::sZero());
        EXPECT_NEAR((steered - key).Length(), 12.0f, 0.05f);
        EXPECT_GT(steered.GetX(), key.GetX() + 6.0f);
        EXPECT_NEAR(steered.GetY(), key.GetY(), 1e-3f); // across the front only
        // Low weapon control reaches less far.
        EXPECT_NEAR((hand(12.0f, 0.0f, farRight, JPH::Vec3::sZero()) - key).Length(), 4.8f, 0.05f);
        // A blow coming in is read ahead: at rest to the left, moving right
        // and toward the guard, the hand goes right.
        const JPH::Vec3 crossing = hand(12.0f, 1.0f, JPH::Vec3(-30.0f, 64.0f, 20.0f), JPH::Vec3(300.0f, -200.0f, 0.0f));
        EXPECT_GT(crossing.GetX(), key.GetX() + 3.0f);
        // Letting go and blocking again starts from the zone's place.
        Rig rig;
        rig.mActor.mParrySteer = 12.0f;
        rig.mThreatStrike = farRight;
        rig.step(false);
        for (int i = 0; i < 60; ++i)
            rig.step(false, 0.0f, 0.0f, true);
        rig.mThreatStrike.reset();
        for (int i = 0; i < 90; ++i)
            rig.step(false, 0.0f, 0.0f, true);
        EXPECT_LT((rig.mPlanner.lastPose()->mHand - key).Length(), 0.05f); // no blade: back to the zone
    }

    TEST(WheelPlannerTest, AShoveThrustsTheWeaponHandOut)
    {
        // The shove's thrust: the hand goes out from the shoulder, 0.97 of
        // the arm ahead of it, the trunk leaning in, with no guard up; then
        // back to the guard.
        Rig rig;
        rig.mShoulder = JPH::Vec3(11.5f, 0.0f, 8.0f);
        rig.step(false);
        const float rest = rig.mPlanner.lastPose()->mHand.GetY();
        rig.mShove = true;
        for (int i = 0; i < 12; ++i)
            rig.step(false);
        EXPECT_NEAR(rig.mPlanner.lastPose()->mHand.GetY(), 0.97f * 55.0f, 1e-2f);
        EXPECT_LT(rig.mPlanner.trunk().mLean, -0.1f);
        EXPECT_GT(rig.mPlanner.lastPose()->mHand.GetY(), rest + 15.0f);
        rig.mShove = false;
        rig.step(false);
        EXPECT_FALSE(rig.mPlanner.blocking());
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Recover);
        for (int i = 0; i < 90; ++i)
            rig.step(false);
        EXPECT_NEAR(rig.mPlanner.lastPose()->mHand.GetY(), rest, 0.5f);
    }

    TEST(WheelPlannerTest, AOneHandedCutReachesOutWithTheSwitch)
    {
        // A backhand and a forehand (the game's arm: shoulder 11.5 right and
        // 6 up of the chest centre, reach 35): with K2_CUT_REACH the grip
        // goes further from the shoulder through the cut (its mean); a
        // two-handed grip and a thrust are left as they were.
        const JPH::Vec3 shoulder(11.5f, 0.0f, 6.0f);
        const auto run = [&](float angle, float share, bool twoHanded) {
            Rig rig;
            rig.mShoulder = shoulder;
            rig.mArmLength = 35.0f;
            rig.mActor.mCutReach = share;
            rig.mStriker.mTwoHanded = twoHanded;
            rig.mClickAngle = angle;
            rig.step(false);
            rig.click();
            float sum = 0.0f;
            int n = 0;
            for (int i = 0; i < 120; ++i)
            {
                rig.step(false);
                if (rig.mPlanner.stage() != WheelStage::Cut)
                    continue;
                const float out = (rig.mPlanner.lastPose()->mHand - shoulder).Length();
                sum += out;
                ++n;
            }
            return n > 0 ? sum / n : 0.0f;
        };
        for (const float angle : { sPi, 0.0f })
        {
            const float keys = run(angle, 0.0f, false);
            const float reached = run(angle, 0.8f, false);
            EXPECT_GT(reached, keys + 1.0f) << angle;
            EXPECT_NEAR(run(angle, 0.8f, true), run(angle, 0.0f, true), 1e-3f) << angle;
        }
        EXPECT_NEAR(run(1.5f * sPi, 0.8f, false), run(1.5f * sPi, 0.0f, false), 1e-3f);
    }

    TEST(WheelPlannerTest, TheHipsTurnFurtherThroughTheCutWithTheSwitch)
    {
        // A forehand with the hips' share: capped at 0.45 rad throughout
        // without K2_CUT_HIP_CAP; with it at 0.6 they turn past 0.45 in the
        // follow-through, never past 0.6, and the cap is back at 0.45 (eased)
        // in the guard.
        const auto run = [](float cap, float* settled) {
            Rig rig;
            rig.mActor.mHipShare = 0.5f;
            rig.mActor.mSideTurn = 1.0f; // as played
            rig.mActor.mCutHipCap = cap;
            rig.mClickAngle = 0.0f;
            rig.step(false);
            rig.click();
            float most = 0.0f;
            for (int i = 0; i < 300; ++i)
            {
                rig.step(false);
                const float hip = std::abs(rig.mPlanner.trunk().mHipTurn);
                EXPECT_LE(hip, std::max(0.45f, cap) + 1e-5f);
                most = std::max(most, hip);
            }
            *settled = std::abs(rig.mPlanner.trunk().mHipTurn);
            return most;
        };
        float settled = 1.0f;
        EXPECT_LE(run(0.0f, &settled), 0.45f + 1e-5f);
        EXPECT_GT(run(0.6f, &settled), 0.5f);
        EXPECT_LT(settled, 0.02f);
    }

    TEST(WheelPlannerTest, TheCutsChestTurnRunsAheadWithTheSwitch)
    {
        // The same forehand without and with K2_CUT_LEAD: in the cut the
        // chest's turn is ahead of the blade's (the forehand's keys turn from
        // +35 to -35 deg, so ahead is lower); outside the cut nothing changes,
        // and both end square in the guard.
        Rig plain, lead;
        lead.mActor.mCutLead = 0.1f;
        plain.mClickAngle = lead.mClickAngle = 0.0f;
        for (Rig* rig : { &plain, &lead })
        {
            rig->step(false);
            rig->click();
        }
        float ahead = 0.0f;
        int cutSteps = 0;
        for (int i = 0; i < 150; ++i)
        {
            plain.step(false);
            lead.step(false);
            ASSERT_EQ(plain.mPlanner.stage(), lead.mPlanner.stage());
            const float a = plain.mPlanner.trunk().mTwist, b = lead.mPlanner.trunk().mTwist;
            if (plain.mPlanner.stage() == WheelStage::Cut)
            {
                ++cutSteps;
                EXPECT_LE(b, a + 1e-4f);
                ahead = std::max(ahead, a - b);
            }
            else if (plain.mPlanner.stage() == WheelStage::Wind)
                EXPECT_NEAR(a, b, 1e-4f);
        }
        EXPECT_GT(cutSteps, 3);
        EXPECT_GT(ahead, 0.1f);
        EXPECT_NEAR(lead.mPlanner.trunk().mTwist, plain.mPlanner.trunk().mTwist, 1e-4f);
    }

    // B6: a two-handed grip measures its step-in with a share of the arm.
    TEST(WheelPlannerTest, ATwoHandedGripStepsInSooner)
    {
        const auto gapWith = [](bool twoHanded, float share) {
            Rig rig;
            rig.mStriker.mTwoHanded = twoHanded;
            rig.mActor.mTwoHandReach = share;
            WheelAimInput aim;
            aim.mFoe = JPH::Vec3(0.0f, 110.0f, 0.0f);
            aim.mArmLength = 50.0f;
            rig.mAim = aim;
            rig.step(false);
            rig.click();
            for (int i = 0; i < 5; ++i)
                rig.step(false);
            return rig.mPlanner.aim().mLungeGap;
        };
        const float whole = gapWith(true, 1.0f);
        EXPECT_NEAR(gapWith(true, 0.6f) - whole, 0.4f * 50.0f, 1e-3f);
        // A one-handed grip keeps its whole arm.
        EXPECT_NEAR(gapWith(false, 0.6f), whole, 1e-4f);
    }

    // The play-6 bookmark: an NPC's attack can chain into two or three cuts.
    TEST(WheelPlannerTest, AnNpcAttackChainsItsCombo)
    {
        // The chains of one NPC attack (its own feints, by skill, add cuts
        // of their own: every chain is a cut more, not every cut a chain).
        const auto chainsOf = [](float second, float third, float skill) {
            Rig rig;
            rig.mActor.mPlayer = false;
            rig.mActor.mSkill = skill;
            rig.mActor.mComboSecond = second;
            rig.mActor.mComboThird = third;
            int turn = 0;
            const float slashes[] = { 0.0f, sPi };
            rig.mHooks.mClickLine = [&] { return slashes[turn++ % 2]; };
            rig.step(false);
            rig.click();
            int cuts = 0, chained = 0;
            for (int i = 0; i < 360; ++i)
            {
                const WheelEvents events = rig.step(false);
                cuts += events.mCutStarted ? 1 : 0;
                chained += events.mChained ? 1 : 0;
            }
            EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Guard);
            EXPECT_GE(cuts, chained + 1);
            // The combo's lines alternate (the hook's turn).
            EXPECT_EQ(turn, chained + 1);
            return chained;
        };
        EXPECT_EQ(chainsOf(0.0f, 0.0f, 100.0f), 0);
        EXPECT_EQ(chainsOf(1.0f, 0.0f, 100.0f), 1);
        EXPECT_EQ(chainsOf(1.0f, 1.0f, 100.0f), 2);
        // At skill 0 never.
        EXPECT_EQ(chainsOf(1.0f, 1.0f, 0.0f), 0);
    }

    // User 11-08 (tick 29223): no path while winding up; the cut has one.
    TEST(WheelPlannerTest, TheWindUpShowsNoPathTheCutDoes)
    {
        Rig rig;
        rig.mAim = WheelAimInput{};
        rig.step(true);
        for (int i = 0; i < 8; ++i)
            rig.step(true, 0.0f, 100.0f);
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        EXPECT_EQ(rig.mPlanner.preview().size(), 0u);
        rig.releaseToPath();
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Cut);
        EXPECT_EQ(rig.mPlanner.preview().size(), 16u);
    }

    // K2_WINDUP_RADIUS: a wider stance circle keeps a long drag aiming.
    TEST(WheelPlannerTest, AWiderStanceRadiusKeepsALongDragInTheStance)
    {
        const auto windAfter = [](float radius) {
            Rig rig;
            rig.mActor.mStanceRadius = radius;
            rig.step(true);
            for (int i = 0; i < 4; ++i)
                rig.step(true, 0.0f, 100.0f);
            return rig.mPlanner.wind();
        };
        EXPECT_NEAR(windAfter(0.0f), windAt(400.0f), 1e-3f);
        EXPECT_NEAR(windAfter(450.0f), windAt(400.0f, 450.0f), 1e-3f);
        EXPECT_LT(windAfter(450.0f), windAfter(0.0f));
    }

    TEST(WheelPlannerTest, ThePreviewMarksWhereTheBladeMeetsTheFoe)
    {
        // Winds a chop up, foe at `foe` (chest-centre frame), radius 16.
        const auto markWith = [](std::optional<JPH::Vec3> foe, std::vector<JPH::Vec3>* path) {
            Rig rig;
            rig.mActor.mPreviewFoeRadius = 16.0f;
            rig.mAim = WheelAimInput{};
            rig.mAim->mFoe = foe;
            rig.step(true);
            for (int i = 0; i < 8; ++i)
                rig.step(true, 0.0f, 100.0f);
            rig.releaseToPath();
            if (path)
                *path = rig.mPlanner.preview();
            return rig.mPlanner.mark();
        };
        std::vector<JPH::Vec3> path;
        EXPECT_FALSE(markWith(std::nullopt, &path).has_value());
        ASSERT_EQ(path.size(), 16u);
        // A foe whose chest sits on the chop's path: a hit, on the blade.
        const JPH::Vec3 onPath = path[8];
        const auto hit = markWith(onPath, nullptr);
        ASSERT_TRUE(hit.has_value());
        EXPECT_TRUE(hit->mHit);
        EXPECT_LT(hit->mGap, -10.0f); // through the body's middle
        // Far out of reach: the closest pass, short by about the distance.
        const JPH::Vec3 far(onPath.GetX(), onPath.GetY() + 300.0f, onPath.GetZ());
        const auto miss = markWith(far, nullptr);
        ASSERT_TRUE(miss.has_value());
        EXPECT_FALSE(miss->mHit);
        EXPECT_GT(miss->mGap, 200.0f);
        EXPECT_LT(miss->mGap, 300.0f);
    }

    TEST(WheelPlannerTest, OnlyTheHeadMarksAHit)
    {
        // An axe whose head is the outer 45%: a foe the haft would brush but
        // the head passes beyond is no hit.
        const auto hitWith = [](float strikeStart, const JPH::Vec3& foe) {
            Rig rig;
            rig.mStriker.mTip = 80.0f;
            rig.mStriker.mStrikeStart = strikeStart;
            rig.mActor.mPreviewFoeRadius = 8.0f;
            rig.mAim = WheelAimInput{};
            rig.mAim->mFoe = foe;
            rig.step(true);
            for (int i = 0; i < 8; ++i)
                rig.step(true, 0.0f, 100.0f);
            rig.releaseToPath();
            const auto mark = rig.mPlanner.mark();
            return mark && mark->mHit;
        };
        // The chop's tip at mid-cut.
        Rig rig;
        rig.mStriker.mTip = 80.0f;
        rig.mAim = WheelAimInput{};
        rig.step(true);
        for (int i = 0; i < 8; ++i)
            rig.step(true, 0.0f, 100.0f);
        rig.releaseToPath();
        ASSERT_EQ(rig.mPlanner.preview().size(), 16u);
        const JPH::Vec3 tip = rig.mPlanner.preview()[8];
        // A thin foe on the line from the chest centre to that tip that the
        // whole weapon meets but the head (the outer 45%) does not.
        std::optional<JPH::Vec3> haftOnly;
        for (float share = 0.05f; share < 0.95f && !haftOnly; share += 0.05f)
        {
            const JPH::Vec3 foe = tip * share;
            if (hitWith(0.0f, foe) && !hitWith(0.55f, foe))
                haftOnly = foe;
        }
        ASSERT_TRUE(haftOnly.has_value()) << "no point the haft meets and the head does not";
        EXPECT_TRUE(hitWith(0.0f, tip));
        EXPECT_TRUE(hitWith(0.55f, tip));
    }

    // K2_POINT_AIM: a long haft, the button held, is carried point-forward;
    // the mouse moves the point, the release thrusts straight down the shaft.
    void makePointRig(Rig& rig)
    {
        rig.mStriker.mSpearSwings = true;
        rig.mStriker.mPointGuard = true;
        rig.mStriker.mTip = 67.0f;
        rig.mActor.mPointCone = 0.35f;
        rig.mActor.mPointCounts = 1200.0f;
        rig.mActor.mPointFlick = 500.0f;
        rig.mShoulder = JPH::Vec3(11.5f, 0.0f, 8.0f);
        rig.mArmLength = 45.0f;
    }

    JPH::Vec3 pointOf(const Rig& rig)
    {
        const ChamberPose& pose = *rig.mPlanner.lastPose();
        return pose.mHand + pose.mRotation * rig.mStriker.mBladeAxis * rig.mStriker.mTip;
    }

    TEST(WheelPlannerTest, AHeldLongHaftPointsWhereTheMouseGoes)
    {
        Rig rig;
        makePointRig(rig);
        for (int i = 0; i < 30; ++i)
            rig.step(true);
        EXPECT_TRUE(rig.mPlanner.pointing());
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Guard);
        // The shaft runs from the rear hand's place to the aim's point,
        // straight ahead of the chest centre: the point near that line.
        JPH::Vec3 point = pointOf(rig);
        EXPECT_NEAR(point.GetX(), 0.0f, 3.0f);
        EXPECT_NEAR(point.GetZ(), 0.0f, 3.0f);
        EXPECT_GT(point.GetY(), 80.0f);
        const JPH::Vec3 hand = rig.mPlanner.lastPose()->mHand;
        // A slow drag right and up: the point goes there.
        for (int i = 0; i < 12; ++i)
            rig.step(true, 20.0f, 10.0f);
        for (int i = 0; i < 12; ++i)
            rig.step(true);
        EXPECT_TRUE(rig.mPlanner.pointing());
        EXPECT_NEAR(rig.mPlanner.pointX(), 0.2f, 1e-3f);
        point = pointOf(rig);
        EXPECT_NEAR(std::atan2(point.GetX(), point.GetY()), 0.2f, 0.04f);
        EXPECT_GT(point.GetZ(), 5.0f);
        // The shaft turns about the rear hand's place, 21 behind the weapon hand's.
        const JPH::Vec3 rear = rig.mPlanner.lastPose()->mHand - (point - rig.mPlanner.lastPose()->mHand).Normalized() * 21.0f;
        EXPECT_LT((rear - (rig.mActor.mPointHand - JPH::Vec3(0.0f, 21.0f, 0.0f))).Length(), 0.01f);
        EXPECT_GT((rig.mPlanner.lastPose()->mHand - hand).Length(), 2.0f);
        // Never past the cone.
        for (int i = 0; i < 10; ++i)
            rig.step(true, 20.0f, 0.0f);
        EXPECT_TRUE(rig.mPlanner.pointing());
        EXPECT_NEAR(std::hypot(rig.mPlanner.pointX(), rig.mPlanner.pointY()), 0.35f, 1e-3f);
    }

    TEST(WheelPlannerTest, ThePointGuardsReleaseThrustsStraightDownTheShaft)
    {
        Rig rig;
        makePointRig(rig);
        for (int i = 0; i < 20; ++i)
            rig.step(true);
        for (int i = 0; i < 12; ++i)
            rig.step(true, -20.0f, 5.0f);
        for (int i = 0; i < 20; ++i)
            rig.step(true);
        EXPECT_NEAR(rig.mPlanner.pointX(), -0.2f, 1e-3f);
        const JPH::Vec3 hand = rig.mPlanner.lastPose()->mHand;
        const JPH::Vec3 shaft = (pointOf(rig) - hand).Normalized();
        bool cut = false, shown = false;
        float reach = 0.0f, drawn = 0.0f, turned = 0.0f, strayed = 0.0f;
        WheelEvents events = rig.step(false);
        EXPECT_TRUE(events.mClick);
        for (int i = 0; i < 90 && rig.mPlanner.stage() != WheelStage::Recover; ++i)
        {
            cut = cut || events.mCutStarted;
            shown = shown || !rig.mPlanner.preview().empty();
            EXPECT_TRUE(rig.mPlanner.pointThrusting());
            const JPH::Vec3 moved = rig.mPlanner.lastPose()->mHand - hand;
            const float along = moved.Dot(shaft);
            // The shaft never turns and the hand never leaves its line.
            turned = std::max(turned, 1.0f - (pointOf(rig) - rig.mPlanner.lastPose()->mHand).Normalized().Dot(shaft));
            strayed = std::max(strayed, (moved - shaft * along).Length());
            reach = std::max(reach, along);
            drawn = std::min(drawn, along);
            events = rig.step(false);
        }
        EXPECT_TRUE(cut);
        EXPECT_TRUE(shown);
        EXPECT_EQ(rig.mPlanner.cutAngle(), 1.5f * sPi);
        EXPECT_LT(turned, 1e-4f);
        EXPECT_LT(strayed, 0.05f);
        EXPECT_GT(reach, 10.0f);
        EXPECT_LT(drawn, -4.0f);
    }

    // The held weapon settles off its command; the point guard moves the
    // command until the real point is where it was asked.
    TEST(WheelPlannerTest, ThePointGuardTrimsItsCommandToTheRealPoint)
    {
        Rig rig;
        makePointRig(rig);
        for (int i = 0; i < 30; ++i)
            rig.step(true);
        const JPH::Vec3 asked = pointOf(rig);
        for (int i = 0; i < 400; ++i)
        {
            // The real point: the commanded one 0.1 rad further right, 0.05 lower.
            const JPH::Vec3 point = pointOf(rig);
            const float flat = std::hypot(point.GetX(), point.GetY());
            const float azimuth = std::atan2(point.GetX(), point.GetY()) + 0.1f;
            const float elevation = std::atan2(point.GetZ(), flat) - 0.05f;
            rig.mPoint = JPH::Vec3(std::sin(azimuth) * std::cos(elevation), std::cos(azimuth) * std::cos(elevation),
                             std::sin(elevation))
                * point.Length();
            rig.step(true);
        }
        EXPECT_NEAR(rig.mPlanner.pointTrimX(), -0.1f, 0.02f);
        EXPECT_NEAR(rig.mPlanner.pointTrimY(), 0.05f, 0.02f);
        EXPECT_NEAR(std::atan2(rig.mPoint->GetX(), rig.mPoint->GetY()), std::atan2(asked.GetX(), asked.GetY()), 0.01f);
    }

    // A swing's drag leaves the point guard: enough net travel within 0.2 s
    // (not the mouse's speed: its counts come in bursts), or a slow drag
    // pushed on past the cone's edge.
    TEST(WheelPlannerTest, ASwingsDragLeavesThePointGuard)
    {
        Rig rig;
        makePointRig(rig);
        for (int i = 0; i < 10; ++i)
            rig.step(true);
        EXPECT_TRUE(rig.mPlanner.pointing());
        for (int i = 0; i < 6; ++i)
            rig.step(true, 100.0f, 0.0f);
        EXPECT_FALSE(rig.mPlanner.pointing());
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        EXPECT_GT(rig.mPlanner.stanceX(), 100.0f);
        // A burst is not a swing: 300 counts in one step (18,000 counts/s),
        // then stillness - the point moved, the guard is kept.
        Rig burst;
        makePointRig(burst);
        for (int i = 0; i < 10; ++i)
            burst.step(true);
        burst.step(true, 300.0f, 0.0f);
        for (int i = 0; i < 20; ++i)
            burst.step(true);
        EXPECT_TRUE(burst.mPlanner.pointing());
        EXPECT_NEAR(burst.mPlanner.pointX(), 0.25f, 1e-3f);
        // A slow drag pushed on past the cone's edge is a swing's.
        Rig slow;
        makePointRig(slow);
        for (int i = 0; i < 10; ++i)
            slow.step(true);
        for (int i = 0; i < 40 && slow.mPlanner.pointing(); ++i)
            slow.step(true, 0.0f, 20.0f);
        EXPECT_FALSE(slow.mPlanner.pointing());
        EXPECT_EQ(slow.mPlanner.stage(), WheelStage::Wind);
        EXPECT_NEAR(slow.mPlanner.angle(), 0.5f * sPi, 0.05f);
    }

    TEST(WheelPlannerTest, AQuickPressIsStillAClickAndTheSwitchOffIsTheWheel)
    {
        Rig rig;
        makePointRig(rig);
        rig.step(true);
        rig.step(true);
        const WheelEvents events = rig.step(false);
        EXPECT_TRUE(events.mClick);
        EXPECT_NEAR(events.mClickAngle, 0.5f * sPi, 1e-4f);
        EXPECT_FALSE(rig.mPlanner.pointThrusting());
        // A click on the thrust line is the point guard's thrust at the aim.
        Rig thrust;
        makePointRig(thrust);
        thrust.mClickAngle = 1.5f * sPi;
        thrust.step(true);
        thrust.step(false);
        EXPECT_TRUE(thrust.mPlanner.pointThrusting());
        EXPECT_EQ(thrust.mPlanner.pointX(), 0.0f);
        // Off: a held drag winds up on the wheel.
        Rig off;
        makePointRig(off);
        off.mActor.mPointCone = 0.0f;
        for (int i = 0; i < 10; ++i)
            off.step(true, 20.0f, 0.0f);
        EXPECT_FALSE(off.mPlanner.pointing());
        EXPECT_EQ(off.mPlanner.stage(), WheelStage::Wind);
    }

    TEST(WheelPlannerTest, TakingTheArmEasesToTheGuard)
    {
        Rig rig;
        ChamberPose taken;
        taken.mHand = JPH::Vec3(30.0f, 10.0f, -20.0f);
        rig.mPlanner.take(taken);
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Recover);
        rig.step(false);
        EXPECT_LT(rig.handTo(taken.mHand), 1.0f);
        EXPECT_GT(rig.until(WheelStage::Guard), 0.3f);
    }
    // FIST-06 (K2_FIST_STEER): the held right fist is shown where the mouse
    // has it, and its punch starts from there on its line.
    TEST(WheelPlannerTest, ASteeredFistIsShownWhereItIsHeldAndThrownFromThere)
    {
        Rig rig;
        rig.mStriker.mLines = &fistLines();
        rig.mStriker.mShort = true;
        rig.mStriker.mWeight = 0.0f;
        rig.mActor.mFistSteer = true;
        rig.mActor.mFightNight = true; // steering takes its place
        for (int i = 0; i < 5; ++i)
            rig.step(false);
        const JPH::Vec3 guard = rig.mPlanner.lastPose()->mHand;
        // The press itself is not the planner's: held with a drag, no wind-up.
        for (int i = 0; i < 10; ++i)
            rig.step(true, 80.0f, 0.0f);
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Guard);
        EXPECT_TRUE(rig.mPlanner.lastPose()->mHand.IsClose(guard, 1e-3f));
        // Held out to the hook's side.
        const FistSteerHold hold = fistHoldPlace(FistHand::Right, 0.0f, 1.0f);
        rig.mFistHold = hold;
        rig.step(false);
        // (taken from the guard over 0.12 s, not at a jump)
        EXPECT_LT((rig.mPlanner.lastPose()->mHand - guard).Length(), 4.0f);
        for (int i = 0; i < 10; ++i)
            rig.step(false);
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Guard);
        EXPECT_TRUE(rig.mPlanner.lastPose()->mHand.IsClose(hold.mHand, 1e-3f));
        EXPECT_GT(hold.mHand.GetX(), guard.GetX() + 10.0f);
        // Let go: the hook, from there.
        rig.mFistHold.reset();
        FistSteerPunch punch;
        punch.mLine = fistLineIndex(PunchKind::Hook);
        punch.mKind = PunchKind::Hook;
        punch.mPower = 1.0f;
        punch.mWound = true;
        punch.mFrom = hold.mHand;
        rig.mFistPunch = punch;
        const WheelEvents thrown = rig.step(false);
        EXPECT_TRUE(thrown.mPunch);
        EXPECT_EQ(thrown.mPunchHand, FistHand::Right);
        EXPECT_EQ(thrown.mPunchKind, PunchKind::Hook);
        EXPECT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        EXPECT_LT((rig.mPlanner.lastPose()->mHand - hold.mHand).Length(), 4.0f); // no jump back to the guard
        // It strikes soon (its wind-up was done by hand) and across the body.
        bool cut = false;
        float least = 1e9f;
        const float wound = rig.until(WheelStage::Cut, 1.0f);
        EXPECT_LT(wound, 0.2f);
        for (int i = 0; i < 60 && rig.mPlanner.stage() != WheelStage::Guard; ++i)
        {
            const WheelEvents events = rig.step(false);
            cut = cut || events.mCutStarted || rig.mPlanner.stage() == WheelStage::Cut;
            least = std::min(least, rig.mPlanner.lastPose()->mHand.GetX());
        }
        EXPECT_TRUE(cut);
        EXPECT_LT(least, 2.0f);

        // A hold that ends with nothing thrown returns from where it was.
        Rig dropped;
        dropped.mStriker = rig.mStriker;
        dropped.mActor = rig.mActor;
        for (int i = 0; i < 5; ++i)
            dropped.step(false);
        dropped.mFistHold = hold;
        for (int i = 0; i < 10; ++i)
            dropped.step(false);
        dropped.mFistHold.reset();
        dropped.step(false);
        EXPECT_EQ(dropped.mPlanner.stage(), WheelStage::Recover);
        EXPECT_LT((dropped.mPlanner.lastPose()->mHand - hold.mHand).Length(), 4.0f);
        dropped.until(WheelStage::Guard);
        for (int i = 0; i < 5; ++i)
            dropped.step(false);
        EXPECT_TRUE(dropped.mPlanner.lastPose()->mHand.IsClose(guard, 1e-2f));
    }
}
