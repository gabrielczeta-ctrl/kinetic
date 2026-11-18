#include <ragdollcore/palmparry.hpp>

#include <gtest/gtest.h>

namespace
{
    using namespace RagdollCore;

    constexpr float sDt = 1.0f / 60.0f;

    PalmParrySettings on()
    {
        return parsePalmParrySettings("1,0.22,1,9,30,0.9");
    }

    // A cut from `from` toward the chest at `speed`, the foe's hand 45 u
    // behind the striking part; stepped until `seconds` have run.
    struct Rig
    {
        PalmParry mParry;
        PalmParryInput mInput;
        PalmParryOutput mLast;
        JPH::Vec3 mStrike = JPH::Vec3(20.0f, 90.0f, 10.0f);
        JPH::Vec3 mVelocity = JPH::Vec3::sZero();
        float mLaunchedAt = -1.0f, mLiveFrom = -1.0f, mLiveTo = -1.0f, mClock = 0.0f;
        int mLaunches = 0;

        Rig()
        {
            mInput.mDt = sDt;
            mInput.mGuard = true;
        }
        void cut(const JPH::Vec3& from, float speed)
        {
            mStrike = from;
            mVelocity = -from.Normalized() * speed;
        }
        void run(float seconds, const PalmParrySettings& settings = on())
        {
            for (float t = 0.0f; t < seconds; t += sDt)
            {
                PalmThreat threat;
                threat.mStrike = mStrike;
                threat.mVelocity = mVelocity;
                threat.mHand = mStrike + JPH::Vec3(0.0f, 45.0f, 0.0f);
                threat.mAttacking = mVelocity.Length() > 1.0f;
                mInput.mThreat = threat;
                mLast = mParry.step(mInput, settings);
                if (mLast.mLaunched)
                {
                    mLaunchedAt = mClock;
                    ++mLaunches;
                }
                if (mLast.mLive && mLiveFrom < 0.0f)
                    mLiveFrom = mClock;
                if (mLast.mLive)
                    mLiveTo = mClock;
                mClock += sDt;
                // The blade stops short of the chest, as on a body.
                if (mStrike.Length() > 16.0f)
                    mStrike += mVelocity * sDt;
                else
                    mVelocity = JPH::Vec3::sZero();
            }
        }
    };

    TEST(PalmParryTest, settingsParseAndOff)
    {
        EXPECT_FALSE(parsePalmParrySettings(nullptr).mOn);
        EXPECT_FALSE(parsePalmParrySettings("0").mOn);
        const PalmParrySettings settings = parsePalmParrySettings("1,0.3,0.4,12,20,1.5");
        EXPECT_TRUE(settings.mOn);
        EXPECT_FLOAT_EQ(settings.mReact, 0.3f);
        EXPECT_FLOAT_EQ(settings.mHold, 0.4f);
        EXPECT_FLOAT_EQ(settings.mCover, 12.0f);
        EXPECT_FLOAT_EQ(settings.mKnock, 20.0f);
        EXPECT_FLOAT_EQ(settings.mOpen, 1.5f);
        // A short value keeps the rest.
        EXPECT_FLOAT_EQ(parsePalmParrySettings("1").mHold, 1.0f);
    }

    TEST(PalmParryTest, skillIsVanillasBlockTermWithHandToHand)
    {
        PalmParryStats stats;
        stats.mHandToHand = 50.0f;
        stats.mAgility = 50.0f;
        stats.mLuck = 40.0f;
        stats.mFatigue = 1.0f;
        EXPECT_NEAR(palmParrySkill(stats), (50.0f + 10.0f + 4.0f) * 1.25f / 100.0f, 1e-5f);
        stats.mFatigue = 0.0f;
        EXPECT_NEAR(palmParrySkill(stats), 64.0f * 0.75f / 100.0f, 1e-5f);
        stats.mHandToHand = 100.0f;
        stats.mFatigue = 1.0f;
        EXPECT_FLOAT_EQ(palmParrySkill(stats), 1.0f);
    }

    TEST(PalmParryTest, guardDownGivesNoHands)
    {
        Rig rig;
        rig.mInput.mGuard = false;
        rig.run(0.2f);
        EXPECT_EQ(rig.mLast.mPhase, PalmPhase::Down);
        EXPECT_FALSE(rig.mLast.mHand[0].has_value());
        EXPECT_FALSE(rig.mLast.mHand[1].has_value());
        // And off, the same with the guard up.
        Rig off;
        off.run(0.2f, PalmParrySettings{});
        EXPECT_EQ(off.mLast.mPhase, PalmPhase::Down);
    }

    TEST(PalmParryTest, handsWaitWhereTheZonePutsThem)
    {
        Rig rig;
        rig.run(0.5f);
        ASSERT_TRUE(rig.mLast.mHand[0] && rig.mLast.mHand[1]);
        const JPH::Vec3 middleLeft = *rig.mLast.mHand[0], middleRight = *rig.mLast.mHand[1];
        EXPECT_LT(middleLeft.GetX(), 0.0f);
        EXPECT_GT(middleRight.GetX(), 0.0f);
        EXPECT_GT(middleRight.GetY(), 15.0f);
        EXPECT_GT(middleRight.GetZ(), 10.0f);
        rig.mInput.mZone = WheelGuardZone::High;
        rig.run(0.6f);
        EXPECT_GT(rig.mLast.mHand[1]->GetZ(), middleRight.GetZ() + 6.0f);
        rig.mInput.mZone = WheelGuardZone::Left;
        rig.run(0.6f);
        EXPECT_LT(rig.mLast.mHand[0]->GetX(), middleLeft.GetX() - 5.0f);
        EXPECT_EQ(rig.mLaunches, 0);
    }

    TEST(PalmParryTest, aBlowIsAnsweredByTheHandOnItsSide)
    {
        for (const float side : { -1.0f, 1.0f })
        {
            Rig rig;
            rig.cut(JPH::Vec3(side * 25.0f, 90.0f, 8.0f), 250.0f);
            rig.run(0.9f);
            EXPECT_EQ(rig.mLaunches, 1);
            ASSERT_GE(rig.mLiveFrom, 0.0f);
            EXPECT_EQ(rig.mParry.palm(), -1); // back in the guard
            // Launched, it went out toward the blade's side.
            Rig again;
            again.cut(JPH::Vec3(side * 25.0f, 90.0f, 8.0f), 250.0f);
            again.run(0.25f);
            ASSERT_EQ(again.mLast.mPhase, PalmPhase::Reach);
            EXPECT_EQ(again.mLast.mPalm, side < 0.0f ? 0 : 1);
            const std::size_t palm = static_cast<std::size_t>(again.mLast.mPalm);
            // Within the arm, ahead of the chest.
            EXPECT_LE((*again.mLast.mHand[palm] - again.mInput.mShoulder[palm]).Length(), 0.95f * 34.0f + 0.01f);
            EXPECT_GE(again.mLast.mHand[palm]->GetY(), 8.0f);
        }
    }

    TEST(PalmParryTest, skillShortensTheReactionAndLengthensTheHold)
    {
        Rig low, high;
        low.mInput.mSkill = 0.0f;
        high.mInput.mSkill = 1.0f;
        for (Rig* rig : { &low, &high })
        {
            // Already inside the lead: the reaction time alone decides.
            rig->cut(JPH::Vec3(15.0f, 60.0f, 8.0f), 120.0f);
            rig->run(1.2f);
            ASSERT_EQ(rig->mLaunches, 1);
        }
        EXPECT_NEAR(low.mLaunchedAt, 0.22f, 2.5f * sDt);
        EXPECT_NEAR(high.mLaunchedAt, 0.22f * 0.35f, 2.5f * sDt);
        EXPECT_GT((high.mLiveTo - high.mLiveFrom), (low.mLiveTo - low.mLiveFrom) + 0.08f);
    }

    TEST(PalmParryTest, theChosenZoneCommitsOrSlows)
    {
        Rig neutral, right, wrong;
        right.mInput.mZone = WheelGuardZone::Right;
        wrong.mInput.mZone = WheelGuardZone::High;
        for (Rig* rig : { &neutral, &right, &wrong })
        {
            rig->mInput.mSkill = 0.0f;
            rig->cut(JPH::Vec3(30.0f, 40.0f, 8.0f), 100.0f);
            rig->run(0.2f);
        }
        // 0.2 s in: the committed palm (0.11 s) is out, the others (0.22, 0.33 s) are not.
        EXPECT_EQ(right.mLaunches, 1);
        EXPECT_TRUE(right.mLast.mCommitted);
        EXPECT_EQ(right.mLast.mPalm, 1);
        EXPECT_EQ(neutral.mLaunches, 0);
        EXPECT_EQ(wrong.mLaunches, 0);
        neutral.run(0.1f);
        wrong.run(0.1f);
        EXPECT_EQ(neutral.mLaunches, 1);
        EXPECT_FALSE(neutral.mLast.mCommitted);
        EXPECT_EQ(wrong.mLaunches, 0);
    }

    TEST(PalmParryTest, onePalmABlowAndAnotherForTheNext)
    {
        Rig rig;
        rig.cut(JPH::Vec3(20.0f, 90.0f, 8.0f), 250.0f);
        rig.run(1.5f); // the blade rests against the body: no second palm
        EXPECT_EQ(rig.mLaunches, 1);
        EXPECT_EQ(rig.mLast.mPhase, PalmPhase::Ready);
        rig.cut(JPH::Vec3(-20.0f, 90.0f, 8.0f), 250.0f);
        rig.run(0.6f);
        EXPECT_EQ(rig.mLaunches, 2);
    }

    TEST(PalmParryTest, aFarOrSlowBladeIsNoBlow)
    {
        Rig far;
        far.cut(JPH::Vec3(20.0f, 200.0f, 8.0f), 100.0f);
        far.run(0.3f);
        EXPECT_EQ(far.mLaunches, 0);
        EXPECT_FALSE(far.mLast.mBlow);
        // A resting blade in reach, its stroke not running.
        Rig still;
        still.mStrike = JPH::Vec3(10.0f, 50.0f, 8.0f);
        still.run(0.5f);
        EXPECT_EQ(still.mLaunches, 0);
    }

    TEST(PalmParryTest, metEndsThePalmEarly)
    {
        Rig rig;
        rig.mInput.mSkill = 1.0f;
        rig.cut(JPH::Vec3(15.0f, 60.0f, 8.0f), 120.0f);
        rig.run(0.2f);
        ASSERT_TRUE(rig.mLast.mLive);
        rig.mParry.met();
        rig.run(0.15f);
        EXPECT_FALSE(rig.mLast.mLive);
        EXPECT_EQ(rig.mLast.mPhase, PalmPhase::Recover);
    }

    // FIST-08 11-16: a palm that left on the slow wind-up was back before the
    // cut, one that waited for the blade to close left as it landed.
    TEST(PalmParryTest, thePalmStaysOutThroughTheWindUpAndTheCut)
    {
        Rig rig;
        rig.mInput.mSkill = 1.0f;
        // The wind-up: the blade drawn back, away from the chest, 0.45 s.
        rig.mStrike = JPH::Vec3(25.0f, 60.0f, 20.0f);
        rig.mVelocity = JPH::Vec3(20.0f, 40.0f, 30.0f);
        rig.run(0.45f);
        EXPECT_EQ(rig.mLaunches, 1);
        EXPECT_TRUE(rig.mLast.mLive);
        // The cut: 900 u/s at the chest.
        rig.cut(rig.mStrike, 900.0f);
        rig.run(0.05f);
        EXPECT_TRUE(rig.mLast.mLive);
        EXPECT_EQ(rig.mLast.mPalm, 1);
        // It stopped: the palm comes back.
        rig.run(0.3f);
        EXPECT_FALSE(rig.mLast.mLive);
        EXPECT_EQ(rig.mLaunches, 1);
    }

    // A stroke held longer than the palm's hold: the palm gives up.
    TEST(PalmParryTest, thePalmDoesNotStayOutForEver)
    {
        Rig rig;
        rig.mInput.mSkill = 0.0f;
        rig.mStrike = JPH::Vec3(25.0f, 60.0f, 20.0f);
        rig.mVelocity = JPH::Vec3(0.0f, 2.0f, 0.0f); // a feint: the stroke runs, the blade waits
        rig.run(1.2f);
        EXPECT_EQ(rig.mLaunches, 1);
        EXPECT_FALSE(rig.mLast.mLive);
        EXPECT_NEAR(rig.mLiveTo - rig.mLaunchedAt, 0.6f, 3.0f * sDt);
    }

    // A combo: the cut goes straight into the next wind-up.
    TEST(PalmParryTest, eachStrokeOfAComboIsABlow)
    {
        PalmParry parry;
        PalmParryInput input;
        input.mDt = sDt;
        input.mGuard = true;
        input.mSkill = 1.0f;
        PalmThreat threat;
        threat.mStrike = JPH::Vec3(20.0f, 50.0f, 15.0f);
        threat.mHand = JPH::Vec3(20.0f, 90.0f, 10.0f);
        int launches = 0;
        const auto run = [&](int stage, float seconds) {
            PalmParryOutput last;
            for (float t = 0.0f; t < seconds; t += sDt)
            {
                threat.mStage = stage;
                input.mThreat = threat;
                last = parry.step(input, on());
                launches += last.mLaunched;
            }
            return last;
        };
        EXPECT_EQ(run(0, 0.3f).mPhase, PalmPhase::Ready); // its guard: no blow, however near
        run(1, 0.4f);
        EXPECT_TRUE(run(2, 0.1f).mLive);
        EXPECT_EQ(launches, 1);
        // The next stroke follows at once: the palm is still out, and stays for it.
        EXPECT_TRUE(run(1, 0.3f).mLive);
        EXPECT_EQ(launches, 1);
        // It met that cut and came back during the return; the stroke after gets a new palm.
        parry.met();
        run(3, 0.5f);
        run(1, 0.4f);
        EXPECT_EQ(launches, 2);
        EXPECT_TRUE(run(2, 0.1f).mLive);
        // Its return and guard: the palm comes back and waits.
        run(3, 0.3f);
        EXPECT_EQ(run(0, 0.5f).mPhase, PalmPhase::Ready);
        EXPECT_EQ(launches, 2);
    }

    TEST(PalmParryTest, contactOutcomes)
    {
        const PalmParrySettings settings = on();
        PalmContactInput input;
        EXPECT_EQ(resolvePalmContact(input, settings).mOutcome, PalmContactOutcome::None);
        input.mPhase = PalmPhase::Ready;
        EXPECT_EQ(resolvePalmContact(input, settings).mOutcome, PalmContactOutcome::Late);
        EXPECT_FLOAT_EQ(resolvePalmContact(input, settings).mDamageScale, 1.0f);
        // Slowly onto the raised hands: the guard's cover holds; fast, or off them, it lands.
        input.mCovered = true;
        input.mSpeed = 120.0f;
        EXPECT_EQ(resolvePalmContact(input, settings).mOutcome, PalmContactOutcome::Covered);
        input.mSpeed = 400.0f;
        EXPECT_EQ(resolvePalmContact(input, settings).mOutcome, PalmContactOutcome::Late);
        input.mCovered = false;
        input.mSpeed = 120.0f;
        EXPECT_EQ(resolvePalmContact(input, settings).mOutcome, PalmContactOutcome::Late);
        input.mSpeed = 400.0f;
        input.mPhase = PalmPhase::Reach;
        input.mLive = true;
        input.mDistance = 30.0f;
        EXPECT_EQ(resolvePalmContact(input, settings).mOutcome, PalmContactOutcome::Miss);
        input.mDistance = 6.0f;
        input.mSkill = 0.0f;
        const PalmContactResult low = resolvePalmContact(input, settings);
        EXPECT_EQ(low.mOutcome, PalmContactOutcome::Deflect);
        EXPECT_NEAR(low.mDamageScale, 0.5f, 1e-5f);
        EXPECT_NEAR(low.mKnock, 15.0f, 1e-4f);
        input.mSkill = 1.0f;
        const PalmContactResult high = resolvePalmContact(input, settings);
        EXPECT_NEAR(high.mDamageScale, 0.0f, 1e-5f);
        EXPECT_NEAR(high.mKnock, 30.0f, 1e-4f);
        EXPECT_NEAR(high.mOpen, 0.9f, 1e-5f);
        // A claymore gives up less; the palm's own arm struck counts whatever the distance.
        input.mWeaponWeight = 45.0f;
        input.mTwoHanded = true;
        input.mDistance = 30.0f;
        input.mOnPalmArm = true;
        const PalmContactResult heavy = resolvePalmContact(input, settings);
        EXPECT_EQ(heavy.mOutcome, PalmContactOutcome::Deflect);
        EXPECT_NEAR(heavy.mDamageScale, 0.35f, 1e-5f);
        // Committed: more turned, a longer opening.
        input.mCommitted = true;
        EXPECT_LT(resolvePalmContact(input, settings).mDamageScale, heavy.mDamageScale);
        EXPECT_GT(resolvePalmContact(input, settings).mOpen, heavy.mOpen);
        // Off: nothing.
        EXPECT_EQ(resolvePalmContact(input, PalmParrySettings{}).mOutcome, PalmContactOutcome::None);
    }

    // K2_PALM_TAP: 11-17, 45 of the player's 52 presses of both buttons were
    // down under 0.2 s (median 0.127 s).
    TEST(PalmParryTest, aTapOfBothKeepsTheGuardForItsWindow)
    {
        EXPECT_FLOAT_EQ(parsePalmTapSettings(nullptr).mWindow, 0.0f);
        EXPECT_FLOAT_EQ(parsePalmTapSettings("0").mWindow, 0.0f);
        const PalmTapSettings tap = parsePalmTapSettings("0.45,0.3");
        EXPECT_FLOAT_EQ(tap.mWindow, 0.45f);
        EXPECT_FLOAT_EQ(tap.mShove, 0.3f);
        // A short value keeps the shove's time.
        EXPECT_FLOAT_EQ(parsePalmTapSettings("0.5").mShove, 0.3f);
        // The window counts from the press.
        EXPECT_NEAR(palmTapLeft(tap, 0.0f), 0.45f, 1e-5f);
        EXPECT_NEAR(palmTapLeft(tap, 0.127f), 0.323f, 1e-5f);
        // A hold let go ends the guard, as before.
        EXPECT_FLOAT_EQ(palmTapLeft(tap, 0.2f), 0.0f);
        EXPECT_FLOAT_EQ(palmTapLeft(tap, 0.48f), 0.0f);
        // A window shorter than the tap leaves nothing, never less.
        EXPECT_FLOAT_EQ(palmTapLeft(parsePalmTapSettings("0.1"), 0.15f), 0.0f);
        // Off: no window (the tap is the shove, the caller's).
        EXPECT_FLOAT_EQ(palmTapLeft(PalmTapSettings{}, 0.1f), 0.0f);
    }

    TEST(PalmParryTest, theShoveIsTheSecondTapOfBoth)
    {
        const PalmTapSettings tap = parsePalmTapSettings("0.45,0.3");
        EXPECT_TRUE(palmTapShoves(tap, 0.15f));
        EXPECT_TRUE(palmTapShoves(tap, 0.29f));
        // The player's repeated parries came 0.4-0.65 s apart: each its own parry.
        EXPECT_FALSE(palmTapShoves(tap, 0.3f));
        EXPECT_FALSE(palmTapShoves(tap, 0.65f));
        EXPECT_FALSE(palmTapShoves(tap, 10.0f));
        // Off, or no shove time: never from the press.
        EXPECT_FALSE(palmTapShoves(PalmTapSettings{}, 0.1f));
        EXPECT_FALSE(palmTapShoves(parsePalmTapSettings("0.45,0"), 0.1f));
    }

    // 11-17, the first session with the tap: taps at the attack's start, the
    // window over 12-100 ms before the blade in 8 of 15 failed parries.
    TEST(PalmParryTest, aTappedGuardStaysWhileABlowIsRead)
    {
        const PalmTapSettings tap = parsePalmTapSettings("0.45,0.3,0.8");
        EXPECT_FLOAT_EQ(tap.mCarry, 0.8f);
        EXPECT_TRUE(palmTapCarries(tap, 0.45f));
        EXPECT_TRUE(palmTapCarries(tap, 1.2f));
        // ... and no longer than the window and the carry together.
        EXPECT_FALSE(palmTapCarries(tap, 1.25f));
        // No carry given: the window ends when it ends.
        EXPECT_FLOAT_EQ(parsePalmTapSettings("0.45,0.3").mCarry, 0.0f);
        EXPECT_FALSE(palmTapCarries(parsePalmTapSettings("0.45,0.3"), 0.45f));
        EXPECT_FALSE(palmTapCarries(PalmTapSettings{}, 0.1f));
    }

    TEST(PalmParryTest, aWrongZoneCostsWhatItIsSetTo)
    {
        EXPECT_FLOAT_EQ(parsePalmParrySettings("1").mWrong, 1.5f);
        EXPECT_FLOAT_EQ(parsePalmParrySettings("1,0.22,1,12,30,0.9,1").mWrong, 1.0f);
        // The time from the blow read to the palm leaving, hands waiting low
        // against a blow down the middle.
        const auto launchAfter = [](const char* text) {
            const PalmParrySettings settings = parsePalmParrySettings(text);
            PalmParry parry;
            PalmParryInput input;
            input.mDt = 1.0f / 60.0f;
            input.mGuard = true;
            input.mSkill = 0.0f;
            input.mZone = WheelGuardZone::Low;
            input.mShoulder = { JPH::Vec3(-9.0f, 0.0f, 10.0f), JPH::Vec3(9.0f, 0.0f, 10.0f) };
            input.mArmLength = { 30.0f, 30.0f };
            PalmThreat threat;
            threat.mStrike = JPH::Vec3(0.0f, 60.0f, 0.0f);
            threat.mVelocity = JPH::Vec3(0.0f, -10.0f, 0.0f);
            threat.mHand = JPH::Vec3(0.0f, 90.0f, 0.0f);
            threat.mStage = 1;
            input.mThreat = threat;
            for (int step = 1; step < 120; ++step)
                if (parry.step(input, settings).mLaunched)
                    return static_cast<float>(step) / 60.0f;
            return -1.0f;
        };
        EXPECT_NEAR(launchAfter("1,0.2,1,12,30,0.9"), 0.3f, 0.02f);
        EXPECT_NEAR(launchAfter("1,0.2,1,12,30,0.9,1"), 0.2f, 0.02f);
    }

    TEST(PalmParryTest, zones)
    {
        EXPECT_EQ(palmZone(JPH::Vec3(0.0f, 30.0f, 0.0f)), WheelGuardZone::Middle);
        EXPECT_EQ(palmZone(JPH::Vec3(0.0f, 30.0f, 20.0f)), WheelGuardZone::High);
        EXPECT_EQ(palmZone(JPH::Vec3(0.0f, 30.0f, -20.0f)), WheelGuardZone::Low);
        EXPECT_EQ(palmZone(JPH::Vec3(-10.0f, 30.0f, 0.0f)), WheelGuardZone::Left);
        EXPECT_EQ(palmZone(JPH::Vec3(10.0f, 30.0f, 0.0f)), WheelGuardZone::Right);
    }
}
