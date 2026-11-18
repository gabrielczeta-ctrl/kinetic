#include <ragdollcore/fiststeer.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace
{
    using namespace RagdollCore;

    constexpr float sDt = 1.0f / 60.0f;

    // Steps `steps` times with the right or left press as given; the last output.
    struct Rig
    {
        FistSteer mSteer;
        FistSteerInput mInput;
        FistSteerOutput mLast;
        std::optional<FistSteerPunch> mLeftPunch, mRightPunch;

        void run(int steps)
        {
            mInput.mDt = sDt;
            for (int i = 0; i < steps; ++i)
            {
                mLast = mSteer.step(mInput);
                if (mLast.mLeftPunch)
                    mLeftPunch = mLast.mLeftPunch;
                if (mLast.mRightPunch)
                    mRightPunch = mLast.mRightPunch;
                mInput.mLeft.mDragRight = mInput.mLeft.mDragUp = 0.0f;
                mInput.mRight.mDragRight = mInput.mRight.mDragUp = 0.0f;
                mInput.mLeft.mTap = mInput.mRight.mTap = false;
            }
        }
        // Drags `hand` by (right, up) counts over `steps` steps.
        void drag(FistSteerPress& press, float right, float up, int steps)
        {
            for (int i = 0; i < steps; ++i)
            {
                press.mDragRight = right / static_cast<float>(steps);
                press.mDragUp = up / static_cast<float>(steps);
                run(1);
            }
        }
    };

    const JPH::Vec3 sGuard = fistLines()[0].mWind.front().mHand;

    TEST(FistSteerTest, ATapThrowsThatHandsStraight)
    {
        Rig rig;
        rig.mInput.mLeft.mHeld = true;
        rig.run(6); // 0.1 s
        rig.mInput.mLeft.mHeld = false;
        rig.run(1);
        ASSERT_TRUE(rig.mLeftPunch.has_value());
        EXPECT_EQ(rig.mLeftPunch->mHand, FistHand::Left);
        EXPECT_EQ(rig.mLeftPunch->mKind, PunchKind::Straight);
        EXPECT_FALSE(rig.mLeftPunch->mWound);
        EXPECT_FALSE(rig.mRightPunch.has_value());

        // The right hand's tap comes from the input layer.
        rig.mInput.mRight.mTap = true;
        rig.run(1);
        ASSERT_TRUE(rig.mRightPunch.has_value());
        EXPECT_EQ(rig.mRightPunch->mHand, FistHand::Right);
        EXPECT_EQ(fistLines()[rig.mRightPunch->mLine].mName, "cross");
    }

    TEST(FistSteerTest, AHeldFistShowsAtOnceAndFollowsTheMouse)
    {
        Rig rig;
        rig.mInput.mRight.mHeld = true;
        rig.run(20);
        ASSERT_TRUE(rig.mLast.mRight.has_value());
        // No travel yet: out of the guard all the same.
        EXPECT_GT(rig.mLast.mRight->mHand.GetY(), sGuard.GetY() + 2.0f);
        const float restX = rig.mLast.mRight->mHand.GetX();
        // Dragged to its own side: the fist goes out to the right, drawn back.
        rig.drag(rig.mInput.mRight, 300.0f, 0.0f, 10);
        rig.run(30);
        ASSERT_TRUE(rig.mLast.mRight.has_value());
        EXPECT_GT(rig.mLast.mRight->mHand.GetX(), restX + 10.0f);
        EXPECT_NEAR(rig.mLast.mRight->mAmount, 1.0f, 0.02f);
        // Halfway there on the way (it follows, it does not jump).
        Rig half;
        half.mInput.mRight.mHeld = true;
        half.run(1);
        half.drag(half.mInput.mRight, 150.0f, 0.0f, 5);
        half.run(30);
        EXPECT_NEAR(half.mLast.mRight->mAmount, 0.5f, 0.02f);
        EXPECT_GT(half.mLast.mRight->mHand.GetX(), restX + 3.0f);
        EXPECT_LT(half.mLast.mRight->mHand.GetX(), rig.mLast.mRight->mHand.GetX() - 3.0f);
        // Pushing past the full draw banks nothing: back 150 is half again.
        rig.drag(rig.mInput.mRight, 900.0f, 0.0f, 5);
        rig.drag(rig.mInput.mRight, -150.0f, 0.0f, 5);
        rig.run(30);
        EXPECT_NEAR(rig.mLast.mRight->mAmount, 0.5f, 0.02f);
    }

    TEST(FistSteerTest, TheLeftHandMirrorsTheRight)
    {
        Rig right, left;
        right.mInput.mRight.mHeld = true;
        left.mInput.mLeft.mHeld = true;
        right.run(1);
        left.run(1);
        right.drag(right.mInput.mRight, 200.0f, -120.0f, 8);
        left.drag(left.mInput.mLeft, -200.0f, -120.0f, 8);
        right.run(30);
        left.run(30);
        ASSERT_TRUE(right.mLast.mRight && left.mLast.mLeft);
        EXPECT_NEAR(left.mLast.mLeft->mHand.GetX(), -right.mLast.mRight->mHand.GetX(), 1e-3f);
        EXPECT_NEAR(left.mLast.mLeft->mHand.GetY(), right.mLast.mRight->mHand.GetY(), 1e-3f);
        EXPECT_NEAR(left.mLast.mLeft->mHand.GetZ(), right.mLast.mRight->mHand.GetZ(), 1e-3f);
        EXPECT_NEAR(left.mLast.mLeft->mTwist, -right.mLast.mRight->mTwist, 1e-4f);
        EXPECT_FALSE(right.mLast.mLeft.has_value());
    }

    TEST(FistSteerTest, LettingGoThrowsTheFistFromWhereItWasDrawn)
    {
        const auto release = [](float right, float up) {
            Rig rig;
            rig.mInput.mRight.mHeld = true;
            rig.run(1);
            rig.drag(rig.mInput.mRight, right, up, 10);
            rig.run(20);
            const JPH::Vec3 held = rig.mLast.mRight->mHand;
            rig.mInput.mRight.mHeld = false;
            rig.run(1);
            EXPECT_TRUE(rig.mRightPunch.has_value());
            EXPECT_FALSE(rig.mLast.mRight.has_value());
            FistSteerPunch punch = rig.mRightPunch.value_or(FistSteerPunch{});
            EXPECT_TRUE(punch.mFrom.IsClose(held, 1.0f));
            return punch;
        };
        // To its own side: the hook, at full power.
        const FistSteerPunch hook = release(300.0f, 0.0f);
        EXPECT_EQ(fistLines()[hook.mLine].mName, "hook");
        EXPECT_EQ(hook.mKind, PunchKind::Hook);
        EXPECT_TRUE(hook.mWound);
        EXPECT_NEAR(hook.mPower, 1.0f, 1e-3f);
        // Down and out: the uppercut. Straight back: the cross. Up: the overhand.
        EXPECT_EQ(fistLines()[release(200.0f, -200.0f).mLine].mName, "uppercut");
        EXPECT_EQ(fistLines()[release(0.0f, -250.0f).mLine].mName, "cross");
        EXPECT_EQ(fistLines()[release(0.0f, 250.0f).mLine].mName, "overhand");
        // Half the draw is about a third of the power.
        const FistSteerPunch half = release(150.0f, 0.0f);
        EXPECT_GT(half.mPower, 0.2f);
        EXPECT_LT(half.mPower, 0.4f);
        // Barely drawn: the straight, wound (it was held), no power.
        const FistSteerPunch small = release(40.0f, 30.0f);
        EXPECT_EQ(fistLines()[small.mLine].mName, "cross");
        EXPECT_TRUE(small.mWound);
        EXPECT_NEAR(small.mPower, 0.0f, 1e-4f);
    }

    TEST(FistSteerTest, CrouchedTheStraightGoesToTheBody)
    {
        Rig rig;
        rig.mInput.mCrouched = true;
        rig.mInput.mRight.mTap = true;
        rig.run(1);
        ASSERT_TRUE(rig.mRightPunch.has_value());
        EXPECT_EQ(rig.mRightPunch->mKind, PunchKind::Body);
        EXPECT_EQ(fistLines()[rig.mRightPunch->mLine].mName, "body");
    }

    TEST(FistSteerTest, TheGuardDropsAHoldAndThrowsNothing)
    {
        Rig rig;
        rig.mInput.mLeft.mHeld = rig.mInput.mRight.mHeld = true;
        rig.run(1);
        rig.drag(rig.mInput.mRight, 300.0f, 0.0f, 10);
        rig.mInput.mBlocked = true;
        rig.mInput.mLeft.mHeld = rig.mInput.mRight.mHeld = false;
        rig.run(2);
        EXPECT_FALSE(rig.mLast.mRight.has_value());
        EXPECT_FALSE(rig.mLeftPunch.has_value());
        EXPECT_FALSE(rig.mRightPunch.has_value());
        // And nothing is left over when the guard ends.
        rig.mInput.mBlocked = false;
        rig.run(5);
        EXPECT_FALSE(rig.mLeftPunch.has_value());
        EXPECT_FALSE(rig.mRightPunch.has_value());
        // A button still down from before the guard is not a new press.
        rig.mInput.mBlocked = true;
        rig.mInput.mLeft.mHeld = true;
        rig.run(2);
        rig.mInput.mBlocked = false;
        rig.run(5);
        EXPECT_FALSE(rig.mLast.mLeft.has_value());
    }

    TEST(FistSteerTest, APunchWaitsForItsBusyHandAndNoLonger)
    {
        Rig rig;
        rig.mInput.mRight.mBusy = true;
        rig.mInput.mRight.mTap = true;
        rig.run(12); // 0.2 s
        EXPECT_FALSE(rig.mRightPunch.has_value());
        rig.mInput.mRight.mBusy = false;
        rig.run(1);
        EXPECT_TRUE(rig.mRightPunch.has_value());

        Rig late;
        late.mInput.mRight.mBusy = true;
        late.mInput.mRight.mTap = true;
        late.run(45); // 0.75 s
        late.mInput.mRight.mBusy = false;
        late.run(2);
        EXPECT_FALSE(late.mRightPunch.has_value());
    }

    TEST(FistSteerTest, ALinePunchStartsFromTheHeldFist)
    {
        const AttackLine& hook = fistLines()[fistLineIndex(PunchKind::Hook)];
        const PunchTiming timing = punchTiming(PunchKind::Hook, 0.5f, 1.0f);
        const JPH::Vec3 from(-20.0f, 9.0f, 12.0f);
        const FistPunchPose start = fistLinePunchPose(hook, FistHand::Left, timing, 0.0f, &from);
        EXPECT_TRUE(start.mHand.IsClose(from, 1e-3f));
        const FistPunchPose plain = fistLinePunchPose(hook, FistHand::Left, timing, timing.mWind + 0.5f * timing.mCut);
        const FistPunchPose wound
            = fistLinePunchPose(hook, FistHand::Left, timing, timing.mWind + 0.5f * timing.mCut, &from);
        EXPECT_TRUE(wound.mHand.IsClose(plain.mHand, 1e-3f));
        EXPECT_TRUE(wound.mStriking);
    }
    // FIST-07: the movement keys pick the punch; the mouse is not read.
    TEST(FistSteerTest, TheFeetPickThePunch)
    {
        const auto name = [](float side, float forward, bool crouched = false) {
            return fistLines()[fistLineForMovement(side, forward, crouched)].mName;
        };
        EXPECT_EQ(name(0.0f, 0.0f), "cross");
        EXPECT_EQ(name(1.0f, 0.0f), "hook");
        EXPECT_EQ(name(-1.0f, 0.0f), "hook");
        EXPECT_EQ(name(0.0f, -1.0f), "uppercut");
        EXPECT_EQ(name(0.0f, 1.0f), "long cross");
        EXPECT_EQ(name(1.0f, 1.0f), "hook"); // a diagonal: the strafe wins
        EXPECT_EQ(name(0.0f, 0.0f, true), "body");
        EXPECT_EQ(name(0.2f, -0.2f), "cross"); // a stick at rest

        FistSteerSettings feet;
        feet.mFeet = true;
        const auto tapWith = [&](float side, float forward, FistHand hand) {
            FistSteer steer;
            FistSteerInput input;
            input.mDt = sDt;
            input.mMoveSide = side;
            input.mMoveForward = forward;
            FistSteerPress& press = hand == FistHand::Left ? input.mLeft : input.mRight;
            press.mHeld = true;
            press.mDragRight = 500.0f; // the mouse is the view's: no part in it
            FistSteerOutput out;
            for (int i = 0; i < 3; ++i)
                out = steer.step(input, feet);
            EXPECT_FALSE(out.mLeft || out.mRight); // a tap shows no held fist
            press.mHeld = false;
            out = steer.step(input, feet);
            return hand == FistHand::Left ? out.mLeftPunch : out.mRightPunch;
        };
        const auto jab = tapWith(0.0f, 0.0f, FistHand::Left);
        ASSERT_TRUE(jab.has_value());
        EXPECT_EQ(jab->mHand, FistHand::Left);
        EXPECT_EQ(fistLines()[jab->mLine].mName, "cross");
        EXPECT_FALSE(jab->mWound);
        EXPECT_NEAR(jab->mPower, 0.0f, 1e-5f);
        const auto hook = tapWith(-1.0f, 0.0f, FistHand::Right);
        ASSERT_TRUE(hook.has_value());
        EXPECT_EQ(hook->mKind, PunchKind::Hook);
        EXPECT_EQ(fistLines()[tapWith(0.0f, -1.0f, FistHand::Right)->mLine].mName, "uppercut");
    }

    TEST(FistSteerTest, AHeldButtonWindsThePickedPunchUpAndLettingGoThrowsItHarder)
    {
        FistSteerSettings feet;
        feet.mFeet = true;
        FistSteer steer;
        FistSteerInput input;
        input.mDt = sDt;
        input.mMoveSide = 1.0f; // the hook
        input.mRight.mHeld = true;
        input.mRight.mHoldLead = 0.16f; // the left button's hold arrives late
        FistSteerOutput out;
        for (int i = 0; i < 60; ++i) // 1 s
            out = steer.step(input, feet);
        ASSERT_TRUE(out.mRight.has_value());
        EXPECT_NEAR(out.mRight->mAmount, 1.0f, 0.02f);
        const FistSteerHold hookPlace = fistHoldPlace(FistHand::Right, 0.0f, 1.0f);
        EXPECT_TRUE(out.mRight->mHand.IsClose(hookPlace.mHand, 1.0f));
        input.mRight.mHeld = false;
        out = steer.step(input, feet);
        ASSERT_TRUE(out.mRightPunch.has_value());
        EXPECT_EQ(out.mRightPunch->mKind, PunchKind::Hook);
        EXPECT_TRUE(out.mRightPunch->mWound);
        EXPECT_NEAR(out.mRightPunch->mPower, 1.0f, 1e-3f);

        // Half the charge is half the power; the left button's 1-step pulse is a tap.
        FistSteer half;
        input.mRight.mHeld = true;
        for (int i = 0; i < 21; ++i) // 0.16 + 0.35 s: 0.31 past the tap of 0.6
            out = half.step(input, feet);
        input.mRight.mHeld = false;
        out = half.step(input, feet);
        ASSERT_TRUE(out.mRightPunch.has_value());
        EXPECT_NEAR(out.mRightPunch->mPower, 0.52f, 0.06f);
        FistSteer pulse;
        input.mRight.mHeld = true;
        pulse.step(input, feet);
        input.mRight.mHeld = false;
        out = pulse.step(input, feet);
        ASSERT_TRUE(out.mRightPunch.has_value());
        EXPECT_FALSE(out.mRightPunch->mWound);
    }
    TEST(FistSteerTest, BothHandsPublishChargeAndPunchChoiceAndClearOnGuard)
    {
        for (auto hand : { FistHand::Left, FistHand::Right })
        {
            const std::size_t side = hand == FistHand::Left ? 0 : 1;
            FistSteer steer;
            FistSteerSettings settings;
            settings.mAim = true;
            FistSteerInput input;
            input.mDt = sDt;
            input.mAimZone = 1;
            auto& press = side == 0 ? input.mLeft : input.mRight;
            press.mHeld = true;
            auto output = steer.step(input, settings);
            EXPECT_FLOAT_EQ(output.mCharge[side], 0.0f);
            EXPECT_EQ(output.mKind[side], PunchKind::Straight);
            for (int i = 0; i < 60; ++i)
                output = steer.step(input, settings);
            EXPECT_FLOAT_EQ(output.mCharge[side], 1.0f);
            EXPECT_EQ(output.mKind[side], PunchKind::Hook);
            EXPECT_LT(output.mCharge[1 - side], 0.0f);
            input.mAimZone = -1;
            output = steer.step(input, settings);
            EXPECT_EQ(output.mKind[side], PunchKind::Uppercut);
            press.mHeld = false;
            output = steer.step(input, settings);
            const auto punch = side == 0 ? output.mLeftPunch : output.mRightPunch;
            ASSERT_TRUE(punch.has_value());
            EXPECT_FLOAT_EQ(punch->mPower, 1.0f);
            EXPECT_EQ(punch->mKind, PunchKind::Uppercut);
            EXPECT_LT(output.mCharge[side], 0.0f);
            press.mHeld = true;
            steer.step(input, settings);
            input.mBlocked = true;
            output = steer.step(input, settings);
            EXPECT_LT(output.mCharge[side], 0.0f);
            EXPECT_FALSE(output.mLeftPunch);
            EXPECT_FALSE(output.mRightPunch);
        }
    }

    // FIST-09: the crosshair picks the punch.
    TEST(FistSteerTest, TheCrosshairPicksThePunch)
    {
        const auto name = [](int zone, bool charged, bool crouched = false) {
            return fistLines()[fistLineForAim(zone, charged, crouched)].mName;
        };
        EXPECT_EQ(name(1, false), "cross");
        EXPECT_EQ(name(0, false), "cross");
        EXPECT_EQ(name(-1, false), "body");
        EXPECT_EQ(name(0, false, true), "body");
        EXPECT_EQ(name(1, true), "hook");
        EXPECT_EQ(name(0, true), "cross");
        EXPECT_EQ(name(-1, true), "uppercut");

        FistSteerSettings aim;
        aim.mAim = true;
        const auto thrown = [&](int zone, int steps) {
            FistSteer steer;
            FistSteerInput input;
            input.mDt = sDt;
            input.mAimZone = zone;
            input.mMoveSide = 1.0f; // strafing: no part in it
            input.mRight.mHeld = true;
            for (int i = 0; i < steps; ++i)
                steer.step(input, aim);
            input.mRight.mHeld = false;
            return steer.step(input, aim).mRightPunch;
        };
        const auto tap = thrown(1, 3);
        ASSERT_TRUE(tap.has_value());
        EXPECT_EQ(fistLines()[tap->mLine].mName, "cross");
        EXPECT_FALSE(tap->mWound);
        const auto hook = thrown(1, 40);
        ASSERT_TRUE(hook.has_value());
        EXPECT_EQ(fistLines()[hook->mLine].mName, "hook");
        EXPECT_TRUE(hook->mWound);
        EXPECT_GT(hook->mPower, 0.5f);
        EXPECT_EQ(fistLines()[thrown(0, 40)->mLine].mName, "cross");
        EXPECT_EQ(fistLines()[thrown(-1, 40)->mLine].mName, "uppercut");
        EXPECT_EQ(fistLines()[thrown(-1, 3)->mLine].mName, "body");
    }
    // K2_FIST_CHARGE: a charged fist draws further back and coils the trunk.
    TEST(FistSteerTest, AChargeShows)
    {
        const auto heldAt = [](FistHand hand, float draw, float coil) {
            FistSteerSettings aim;
            aim.mAim = true;
            aim.mChargeDraw = draw;
            aim.mChargeCoil = coil;
            FistSteer steer;
            FistSteerInput input;
            input.mDt = sDt;
            FistSteerPress& press = hand == FistHand::Left ? input.mLeft : input.mRight;
            press.mHeld = true;
            FistSteerOutput out;
            for (int i = 0; i < 90; ++i)
                out = steer.step(input, aim);
            return hand == FistHand::Left ? out.mLeft : out.mRight;
        };
        const auto plain = heldAt(FistHand::Right, 0.0f, 0.0f);
        const auto drawn = heldAt(FistHand::Right, 12.0f, 0.4f);
        ASSERT_TRUE(plain && drawn);
        EXPECT_NEAR((drawn->mHand - plain->mHand).Length(), 12.0f, 0.5f);
        EXPECT_LT(drawn->mHand.GetY(), plain->mHand.GetY()); // the straight: back
        EXPECT_NEAR(drawn->mTwist - plain->mTwist, 0.4f, 0.02f);
        const auto left = heldAt(FistHand::Left, 12.0f, 0.4f);
        const auto leftPlain = heldAt(FistHand::Left, 0.0f, 0.0f);
        ASSERT_TRUE(left && leftPlain);
        EXPECT_NEAR(left->mHand.GetX(), -drawn->mHand.GetX(), 1e-3f); // mirrored
        EXPECT_NEAR(left->mTwist - leftPlain->mTwist, -0.4f, 0.02f);
    }
    // FIST-12: a click is the crosshair's punch; a held button winds up by
    // the mouse as the weapons do, each direction its own punch.
    TEST(FistSteerTest, AHeldFistChargesByTheMouseAsTheWeaponsDo)
    {
        FistSteerSettings drag;
        drag.mAim = true;
        drag.mDrag = true;
        struct Result
        {
            std::optional<FistSteerPunch> mPunch;
            FistSteerOutput mHeld;
        };
        // Holds `hand` for `steps`, dragging (right, up) counts over them, then lets go.
        const auto thrown = [&](FistHand hand, float right, float up, int steps, int zone = 0) {
            FistSteer steer;
            FistSteerInput input;
            input.mDt = sDt;
            input.mAimZone = zone;
            FistSteerPress& press = hand == FistHand::Left ? input.mLeft : input.mRight;
            press.mHeld = true;
            Result result;
            for (int i = 0; i < steps; ++i)
            {
                press.mDragRight = right / static_cast<float>(steps);
                press.mDragUp = up / static_cast<float>(steps);
                result.mHeld = steer.step(input, drag);
            }
            press.mHeld = false;
            press.mDragRight = press.mDragUp = 0.0f;
            const FistSteerOutput out = steer.step(input, drag);
            result.mPunch = hand == FistHand::Left ? out.mLeftPunch : out.mRightPunch;
            return result;
        };
        const auto name = [](const Result& r) { return fistLines()[r.mPunch->mLine].mName; };

        // A click: the crosshair's tap, not wound.
        const Result tap = thrown(FistHand::Right, 0.0f, 0.0f, 4);
        ASSERT_TRUE(tap.mPunch.has_value());
        EXPECT_EQ(name(tap), "cross");
        EXPECT_FALSE(tap.mPunch->mWound);
        EXPECT_EQ(name(thrown(FistHand::Right, 0.0f, 0.0f, 4, -1)), "body");

        // Time alone charges nothing: a long still hold shows no charge and throws no power.
        const Result still = thrown(FistHand::Right, 0.0f, 0.0f, 90);
        ASSERT_TRUE(still.mPunch.has_value());
        EXPECT_NEAR(still.mHeld.mCharge[1], 0.0f, 1e-4f);
        EXPECT_NEAR(still.mPunch->mPower, 0.0f, 1e-4f);

        // The mouse winds it up: each way its own line, the travel its power.
        std::vector<std::string> kinds;
        for (const auto& [right, up] : { std::pair{ 300.0f, 0.0f }, { 0.0f, 300.0f }, { -300.0f, 0.0f }, { 0.0f, -300.0f } })
        {
            const Result full = thrown(FistHand::Right, right, up, 20);
            ASSERT_TRUE(full.mPunch.has_value());
            EXPECT_TRUE(full.mPunch->mWound);
            EXPECT_NEAR(full.mPunch->mPower, 1.0f, 1e-3f);
            EXPECT_NEAR(full.mHeld.mCharge[1], 1.0f, 1e-3f);
            EXPECT_NEAR(full.mHeld.mDrawRight[1], right / 300.0f, 1e-3f);
            EXPECT_NEAR(full.mHeld.mDrawUp[1], up / 300.0f, 1e-3f);
            EXPECT_EQ(full.mHeld.mKind[1], full.mPunch->mKind); // the cue is the punch thrown
            kinds.emplace_back(name(full));
        }
        std::sort(kinds.begin(), kinds.end());
        EXPECT_GE(std::unique(kinds.begin(), kinds.end()) - kinds.begin(), 3); // the directions differ
        const Result half = thrown(FistHand::Right, 195.0f, 0.0f, 20);
        ASSERT_TRUE(half.mPunch.has_value());
        EXPECT_NEAR(half.mPunch->mPower, 0.5f, 0.02f);
        EXPECT_NEAR(half.mHeld.mCharge[1], 0.65f, 0.01f);

        // The left hand mirrors: a drag to its own side is the right's punch,
        // and its reticle draw is the way the mouse went.
        const Result right = thrown(FistHand::Right, 300.0f, 0.0f, 20);
        const Result left = thrown(FistHand::Left, -300.0f, 0.0f, 20);
        ASSERT_TRUE(left.mPunch.has_value());
        EXPECT_EQ(left.mPunch->mLine, right.mPunch->mLine);
        EXPECT_NEAR(left.mHeld.mDrawRight[0], -1.0f, 1e-3f);
        EXPECT_LT(left.mHeld.mCharge[1], 0.0f); // the right fist is not held
    }

    // The swing's rig: one hand held in mode 4, the mouse moved as the
    // player's (the left hand's drags are given mirrored, so one table of
    // gestures serves both hands).
    struct Swing
    {
        FistSteer mSteer;
        FistSteerInput mInput;
        FistSteerSettings mSettings;
        FistHand mHand;
        FistSteerOutput mLast;
        std::vector<FistSteerPunch> mPunches;

        explicit Swing(FistHand hand)
            : mHand(hand)
        {
            mSettings.mAim = true;
            mSettings.mDrag = true;
            mInput.mDt = sDt;
            press().mHeld = true;
        }
        FistSteerPress& press() { return mHand == FistHand::Left ? mInput.mLeft : mInput.mRight; }
        std::size_t side() const { return mHand == FistHand::Left ? 0 : 1; }
        // Moves the mouse (to the hand's own side, up) counts over `steps`.
        void move(float out, float up, int steps)
        {
            for (int i = 0; i < steps; ++i)
            {
                press().mDragRight = (mHand == FistHand::Left ? -out : out) / static_cast<float>(steps);
                press().mDragUp = up / static_cast<float>(steps);
                mLast = mSteer.step(mInput, mSettings);
                if (const auto& punch = mHand == FistHand::Left ? mLast.mLeftPunch : mLast.mRightPunch)
                    mPunches.push_back(*punch);
            }
            press().mDragRight = press().mDragUp = 0.0f;
        }
        std::string line(std::size_t punch) const { return std::string(fistLines()[mPunches[punch].mLine].mName); }
    };

    // FIST-12, the swing: a wound fist is thrown, the button still down, by a
    // stroke back against its draw - the punch of the wind-up, as on letting go.
    TEST(FistSteerTest, AWoundFistIsThrownByAStrokeAgainstItsDraw)
    {
        struct Gesture
        {
            float mOut, mUp; // the wind-up (to the hand's own side, up)
            float mStrokeOut, mStrokeUp;
            const char* mLine;
            PunchKind mKind;
        };
        const Gesture gestures[] = {
            { 212.0f, -212.0f, 0.0f, 300.0f, "uppercut", PunchKind::Uppercut }, // down and out, then up
            { 300.0f, 0.0f, -300.0f, 0.0f, "hook", PunchKind::Hook }, // to its own side, then across
            { -300.0f, 0.0f, 300.0f, 0.0f, "backfist", PunchKind::Hook }, // across, then out
            { 0.0f, 300.0f, 0.0f, -300.0f, "overhand", PunchKind::Straight }, // up, then down
            // Straight down winds the uppercut too ("down up works for uppercut").
            { 0.0f, -300.0f, 0.0f, 300.0f, "uppercut", PunchKind::Uppercut },
        };
        for (const FistHand hand : { FistHand::Right, FistHand::Left })
            for (const Gesture& gesture : gestures)
            {
                SCOPED_TRACE(std::string(hand == FistHand::Left ? "left " : "right ") + gesture.mLine);
                Swing swing(hand);
                swing.move(gesture.mOut, gesture.mUp, 20);
                EXPECT_TRUE(swing.mPunches.empty());
                EXPECT_NEAR(swing.mLast.mCharge[swing.side()], 1.0f, 1e-2f);
                swing.move(gesture.mStrokeOut, gesture.mStrokeUp, 3); // 0.05 s
                ASSERT_EQ(swing.mPunches.size(), 1u);
                EXPECT_EQ(swing.mPunches[0].mHand, hand);
                EXPECT_EQ(swing.line(0), gesture.mLine);
                EXPECT_EQ(swing.mPunches[0].mKind, gesture.mKind);
                EXPECT_TRUE(swing.mPunches[0].mWound);
                EXPECT_NEAR(swing.mPunches[0].mPower, 1.0f, 1e-2f); // the draw before the stroke
                // The button is still down; nothing reads as wound.
                EXPECT_TRUE(swing.mSteer.holding(hand));
                EXPECT_LE(swing.mLast.mCharge[swing.side()], 0.0f);
                EXPECT_FALSE((hand == FistHand::Left ? swing.mLast.mLeft : swing.mLast.mRight).has_value());
                // Letting go later throws nothing more.
                swing.press().mBusy = true;
                swing.move(0.0f, 0.0f, 20);
                swing.press().mBusy = false;
                swing.move(0.0f, 0.0f, 20);
                EXPECT_NEAR(swing.mLast.mCharge[swing.side()], 0.0f, 1e-4f);
                swing.press().mHeld = false;
                swing.move(0.0f, 0.0f, 40);
                EXPECT_EQ(swing.mPunches.size(), 1u);
            }

        // Half a draw swung: half the power.
        Swing half(FistHand::Right);
        half.move(195.0f, 0.0f, 20);
        half.move(-240.0f, 0.0f, 3);
        ASSERT_EQ(half.mPunches.size(), 1u);
        EXPECT_EQ(half.line(0), "hook");
        EXPECT_NEAR(half.mPunches[0].mPower, 0.5f, 0.02f);
    }

    TEST(FistSteerTest, ASmallOrSlowReturnThrowsNothing)
    {
        // A small reversal, however quick: the draw is less, and letting go throws it.
        Swing small(FistHand::Right);
        small.move(300.0f, 0.0f, 20);
        small.move(-60.0f, 0.0f, 1);
        small.move(0.0f, 0.0f, 30);
        EXPECT_TRUE(small.mPunches.empty());
        EXPECT_NEAR(small.mLast.mCharge[1], 0.8f, 1e-2f);
        small.press().mHeld = false;
        small.move(0.0f, 0.0f, 1);
        ASSERT_EQ(small.mPunches.size(), 1u);
        EXPECT_EQ(small.line(0), "hook");
        EXPECT_NEAR(small.mPunches[0].mPower, (0.8f - 0.3f) / 0.7f, 0.02f);

        // A slow way back to the guard (240 counts a second) only lessens the draw.
        Swing slow(FistHand::Left);
        slow.move(300.0f, 0.0f, 20);
        slow.move(-240.0f, 0.0f, 60);
        EXPECT_TRUE(slow.mPunches.empty());
        EXPECT_NEAR(slow.mLast.mCharge[0], 0.2f, 1e-2f);
        slow.press().mHeld = false;
        slow.move(0.0f, 0.0f, 1);
        ASSERT_EQ(slow.mPunches.size(), 1u);
        EXPECT_EQ(slow.line(0), "cross"); // under the straight's draw
        EXPECT_NEAR(slow.mPunches[0].mPower, 0.0f, 1e-4f);

        // A stroke around the wheel (to another wind-up, not back) throws nothing.
        Swing around(FistHand::Right);
        around.move(300.0f, 0.0f, 20);
        around.move(0.0f, 300.0f, 3);
        EXPECT_TRUE(around.mPunches.empty());

        // A fist not wound (under the straight's draw) has nothing to swing.
        Swing unwound(FistHand::Right);
        unwound.move(60.0f, 0.0f, 10);
        unwound.move(-300.0f, 0.0f, 3);
        EXPECT_TRUE(unwound.mPunches.empty());

        // Swing counts 0: letting go only.
        Swing off(FistHand::Right);
        off.mSettings.mSwing = 0.0f;
        off.move(300.0f, 0.0f, 20);
        off.move(-200.0f, 0.0f, 3);
        EXPECT_TRUE(off.mPunches.empty());
        EXPECT_NEAR(off.mLast.mCharge[1], 1.0f / 3.0f, 1e-2f);
        off.press().mHeld = false;
        off.move(0.0f, 0.0f, 1);
        ASSERT_EQ(off.mPunches.size(), 1u);
        EXPECT_EQ(off.line(0), "hook");
    }

    TEST(FistSteerTest, AHeldFistSwingsAgainOnceItsPunchIsDone)
    {
        Swing swing(FistHand::Right);
        swing.move(300.0f, 0.0f, 20);
        swing.move(-300.0f, 0.0f, 3);
        ASSERT_EQ(swing.mPunches.size(), 1u);
        EXPECT_EQ(swing.line(0), "hook");

        // The punch is out: the mouse's follow-through and a stroke back wind and throw nothing.
        swing.press().mBusy = true;
        swing.move(-250.0f, 0.0f, 6);
        swing.move(250.0f, 0.0f, 3);
        swing.move(0.0f, 0.0f, 10);
        EXPECT_EQ(swing.mPunches.size(), 1u);
        EXPECT_LT(swing.mLast.mCharge[1], 0.0f);

        // Done: the hand winds up again from the guard and is swung again.
        swing.press().mBusy = false;
        swing.move(0.0f, 0.0f, 2);
        EXPECT_NEAR(swing.mLast.mCharge[1], 0.0f, 1e-4f);
        EXPECT_TRUE(swing.mLast.mRight.has_value());
        swing.move(0.0f, 300.0f, 20);
        EXPECT_NEAR(swing.mLast.mCharge[1], 1.0f, 1e-2f);
        swing.move(0.0f, -300.0f, 3);
        ASSERT_EQ(swing.mPunches.size(), 2u);
        EXPECT_EQ(swing.line(1), "overhand");
        EXPECT_NEAR(swing.mPunches[1].mPower, 1.0f, 1e-2f);

        // Wound again after a swing and let go: that punch, as any release.
        swing.press().mBusy = true;
        swing.move(0.0f, 0.0f, 10);
        swing.press().mBusy = false;
        swing.move(212.0f, -212.0f, 20);
        swing.press().mHeld = false;
        swing.move(0.0f, 0.0f, 1);
        ASSERT_EQ(swing.mPunches.size(), 3u);
        EXPECT_EQ(swing.line(2), "uppercut");

        // A punch never seen busy (it did not start): the hand winds up again after 0.3 s.
        Swing unseen(FistHand::Left);
        unseen.move(300.0f, 0.0f, 20);
        unseen.move(-300.0f, 0.0f, 3);
        ASSERT_EQ(unseen.mPunches.size(), 1u);
        unseen.move(0.0f, 0.0f, 24);
        unseen.move(0.0f, 300.0f, 20);
        EXPECT_NEAR(unseen.mLast.mCharge[0], 1.0f, 1e-2f);
    }
    // The weapons' rule (mRelease false): only the swing and the click punch,
    // and the swing's follow-through winds nothing.
    TEST(FistSteerTest, OnlyTheSwingDeliversAndItsFollowThroughWindsNothing)
    {
        FistSteerSettings rule;
        rule.mAim = true;
        rule.mDrag = true;
        rule.mRelease = false;
        struct Run
        {
            FistSteer mSteer;
            FistSteerInput mInput;
            FistSteerSettings mSettings;
            std::vector<FistSteerPunch> mPunches;
            FistSteerOutput mLast;
            void move(float right, float up, int steps)
            {
                for (int i = 0; i < steps; ++i)
                {
                    mInput.mDt = sDt;
                    mInput.mRight.mDragRight = right / static_cast<float>(steps);
                    mInput.mRight.mDragUp = up / static_cast<float>(steps);
                    mLast = mSteer.step(mInput, mSettings);
                    if (mLast.mRightPunch)
                        mPunches.push_back(*mLast.mRightPunch);
                }
            }
        };
        // Wound and let go: back to the guard, nothing thrown.
        {
            Run run;
            run.mSettings = rule;
            run.mInput.mRight.mHeld = true;
            run.move(300.0f, 0.0f, 20);
            EXPECT_NEAR(run.mLast.mCharge[1], 1.0f, 1e-3f);
            run.mInput.mRight.mHeld = false;
            run.move(0.0f, 0.0f, 30);
            EXPECT_TRUE(run.mPunches.empty());
        }
        // A click, and a click held a little long: the jab. A long still hold: nothing.
        for (const auto& [steps, punches] : { std::pair{ 4, 1u }, { 18, 1u }, { 40, 0u } })
        {
            Run run;
            run.mSettings = rule;
            run.mInput.mRight.mHeld = true;
            run.move(10.0f, 0.0f, steps);
            run.mInput.mRight.mHeld = false;
            run.move(0.0f, 0.0f, 2);
            ASSERT_EQ(run.mPunches.size(), punches) << steps;
            if (punches)
                EXPECT_FALSE(run.mPunches[0].mWound);
        }
        // The player's stroke (11-17): wound 1,500 counts to the side, swung
        // 2,300 back through the middle and out the other side, the punch
        // done meanwhile, then let go. One hook, nothing from the follow-through.
        {
            Run run;
            run.mSettings = rule;
            run.mInput.mRight.mHeld = true;
            run.move(1500.0f, 0.0f, 20);
            run.move(-600.0f, 0.0f, 3);
            ASSERT_EQ(run.mPunches.size(), 1u);
            EXPECT_EQ(fistLines()[run.mPunches[0].mLine].mName, "hook");
            run.mInput.mRight.mBusy = true;
            run.move(-900.0f, 0.0f, 5);
            run.mInput.mRight.mBusy = false; // the punch is done, the mouse still moving
            run.move(-800.0f, 0.0f, 5);
            EXPECT_LT(run.mLast.mCharge[1], 0.0f); // not winding the backfist
            run.mInput.mRight.mHeld = false;
            run.move(0.0f, 0.0f, 30);
            EXPECT_EQ(run.mPunches.size(), 1u);
        }
        // At rest again, the same hold winds and swings the next punch.
        {
            Run run;
            run.mSettings = rule;
            run.mInput.mRight.mHeld = true;
            run.move(300.0f, 0.0f, 20);
            run.move(-300.0f, 0.0f, 3);
            run.mInput.mRight.mBusy = true;
            run.move(-300.0f, 0.0f, 5);
            run.mInput.mRight.mBusy = false;
            run.move(0.0f, 0.0f, 8); // at rest
            EXPECT_NEAR(run.mLast.mCharge[1], 0.0f, 1e-4f);
            run.move(0.0f, 300.0f, 20);
            run.move(0.0f, -300.0f, 3);
            ASSERT_EQ(run.mPunches.size(), 2u);
            EXPECT_EQ(fistLines()[run.mPunches[1].mLine].mName, "overhand");
        }
    }
}
