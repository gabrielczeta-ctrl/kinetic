#include <ragdollcore/cutwheel.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <string_view>

namespace
{
    using namespace RagdollCore;

    constexpr float sPi = std::numbers::pi_v<float>;

    // A sword's frame: the blade along +Y, the edge along X.
    const WeaponStrikeAxes sAxes{ JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX() };

    TEST(CutWheelLinesTest, OrderedRoundTheWheel)
    {
        const std::array<std::string_view, 8> weapon{ "forehand", "up-right", "chop", "up-left", "backhand",
            "down-left", "thrust", "down-right" };
        const std::array<std::string_view, 8> fist{ "hook", "long cross", "overhand", "overhand hook", "backfist",
            "body", "cross", "uppercut" };
        for (std::size_t i = 0; i < 8; ++i)
        {
            EXPECT_EQ(weaponLines()[i].mName, weapon[i]);
            EXPECT_EQ(fistLines()[i].mName, fist[i]);
            EXPECT_NEAR(weaponLines()[i].angle(), static_cast<float>(i) * sPi / 4.0f, 1e-3f);
        }
        EXPECT_EQ(weaponLines()[2].mAttackType, "chop");
        EXPECT_EQ(weaponLines()[6].mAttackType, "thrust");
        EXPECT_EQ(weaponLines()[0].mAttackType, "slash");
    }

    TEST(CutWheelLinesTest, EveryRouteRunsThroughItsKeys)
    {
        for (const WheelLines* lines : { &weaponLines(), &fistLines() })
            for (const AttackLine& line : *lines)
                for (const auto* route : { &line.mWind, &line.mCut })
                {
                    const auto rotations = routeRotations(*route, sAxes, route == &line.mCut, std::nullopt);
                    ASSERT_TRUE(rotations) << line.mName;
                    for (const ChamberKey& key : *route)
                    {
                        const ChamberPose pose = routePose(*route, *rotations, key.mS);
                        EXPECT_TRUE(pose.mHand.IsClose(key.mHand, 1e-6f)) << line.mName << " s=" << key.mS;
                        EXPECT_NEAR(pose.mTwist, key.mTwistDegrees * sPi / 180.0f, 1e-5f) << line.mName;
                    }
                }
    }

    TEST(CutWheelLinesTest, TheChamberJoinsWindUpAndCut)
    {
        // The wind-up's last key takes the cut's first rotation: no turn at
        // the chamber.
        for (const AttackLine& line : weaponLines())
        {
            const auto end = linePose(line, sAxes, 1.0f);
            const auto start = linePose(line, sAxes, 1.0001f);
            ASSERT_TRUE(end && start) << line.mName;
            EXPECT_LT((end->mHand - start->mHand).Length(), 0.05f) << line.mName;
            EXPECT_GT(std::abs(end->mRotation.Dot(start->mRotation)), 0.9999f) << line.mName;
        }
    }

    TEST(CutWheelLinesTest, TheGuardIsTheFirstKey)
    {
        const auto guard = guardPose(weaponLines(), sAxes);
        ASSERT_TRUE(guard);
        EXPECT_TRUE(guard->mHand.IsClose(JPH::Vec3(8.0f, 24.0f, -8.0f), 1e-6f));
        // Every line's wind-up starts there.
        for (const AttackLine& line : weaponLines())
        {
            const auto start = linePose(line, sAxes, 0.0f);
            ASSERT_TRUE(start);
            EXPECT_TRUE(start->mHand.IsClose(guard->mHand, 1e-6f)) << line.mName;
        }
    }

    TEST(CutWheelLinesTest, BetweenTwoLinesTheWheelBlends)
    {
        WheelSlot slot = wheelSlot(3.0f * sPi / 8.0f);
        EXPECT_EQ(slot.mFirst, 1u);
        EXPECT_EQ(slot.mSecond, 2u);
        EXPECT_NEAR(slot.mWeight, 0.5f, 1e-5f);
        slot = wheelSlot(2.0f * sPi - 0.01f);
        EXPECT_EQ(slot.mFirst, 7u);
        EXPECT_EQ(slot.mSecond, 0u);
        slot = wheelSlot(-0.01f);
        EXPECT_EQ(slot.mFirst, 7u);

        // On a line the wheel is that line; halfway, the hands' midpoint.
        const auto chop = wheelPose(weaponLines(), sAxes, sPi / 2.0f, 1.5f);
        const auto chopLine = linePose(weaponLines()[2], sAxes, 1.5f);
        ASSERT_TRUE(chop && chopLine);
        EXPECT_TRUE(chop->mHand.IsClose(chopLine->mHand, 1e-6f));
        const auto between = wheelPose(weaponLines(), sAxes, 3.0f * sPi / 8.0f, 1.5f);
        const auto upRight = linePose(weaponLines()[1], sAxes, 1.5f);
        ASSERT_TRUE(between && upRight);
        EXPECT_TRUE(between->mHand.IsClose((chopLine->mHand + upRight->mHand) * 0.5f, 1e-4f));
    }

    TEST(CutWheelBlendTest, ThePoleTurnsTheShortWay)
    {
        ChamberPose a, b;
        a.mPole = JPH::Vec3(1.0f, 0.0f, 0.0f);
        b.mPole = JPH::Vec3(-1.0f, 0.001f, 0.0f);
        const ChamberPose half = blendPose(a, b, 0.5f);
        // Mixed straight, nearly opposite poles pass through zero and flip.
        EXPECT_NEAR(half.mPole.Length(), 1.0f, 1e-4f);
        EXPECT_GT(std::abs(half.mPole.GetY()), 0.9f);
        a.mTwist = 0.2f;
        b.mTwist = -0.4f;
        EXPECT_NEAR(blendPose(a, b, 0.25f).mTwist, 0.05f, 1e-6f);
    }

    TEST(CutWheelPaceTest, TheGamesNumbers)
    {
        // The player's iron longsword (weight 20): 0.35 s and 0.25 s at the
        // weight, divided by 1.5 x 1.15.
        PaceInput input;
        input.mWeight = 20.0f;
        Pace pace = attackPace(input);
        EXPECT_NEAR(pace.mAttackSpeed, 1.725f, 1e-5f);
        EXPECT_NEAR(pace.mWind, 0.35f / 1.725f, 1e-5f);
        EXPECT_NEAR(pace.mCut, 0.25f / 1.725f, 1e-5f);
        EXPECT_FLOAT_EQ(pace.mChamberHold, 0.0f);
        EXPECT_FLOAT_EQ(pace.mReleaseHold, 0.3f);
        EXPECT_FLOAT_EQ(pace.mReturn, 0.4f);

        // An NPC's: slower wind-up, the chamber tell, a longer hold and return.
        input.mPlayer = false;
        pace = attackPace(input);
        EXPECT_NEAR(pace.mWind, 0.35f / 1.5f * 1.6f, 1e-5f);
        EXPECT_NEAR(pace.mCut, 0.25f / 1.5f, 1e-5f);
        EXPECT_FLOAT_EQ(pace.mChamberHold, 0.15f);
        EXPECT_FLOAT_EQ(pace.mReleaseHold, 0.7f);
        EXPECT_NEAR(pace.mReturn, 0.6f, 1e-6f);
        input.mNpcRiposte = true;
        pace = attackPace(input);
        EXPECT_NEAR(pace.mWind, 0.35f / 1.5f * 0.8f, 1e-5f);
        EXPECT_FLOAT_EQ(pace.mChamberHold, 0.0f);

        // The player's claymore (weight 30): heavier, two-handed.
        input = {};
        input.mWeight = 30.0f;
        input.mTwoHanded = true;
        pace = attackPace(input);
        const float claymore = std::sqrt(2.4f / 1.6f) / (1.3f * 1.05f);
        EXPECT_NEAR(pace.mWind, 0.35f * claymore, 1e-5f);
        EXPECT_NEAR(pace.mCut, 0.25f * claymore, 1e-5f);
        EXPECT_FLOAT_EQ(pace.mReleaseHold, 1.0f);
        EXPECT_FLOAT_EQ(pace.mReturn, 0.6f);

        // The player's dagger: its own short pace.
        input = {};
        input.mWeight = 3.0f;
        input.mShort = true;
        pace = attackPace(input);
        EXPECT_NEAR(pace.mWind, 0.12f / 1.15f, 1e-5f);
        EXPECT_NEAR(pace.mCut, 0.15f / 1.15f, 1e-5f);
        EXPECT_FLOAT_EQ(pace.mReleaseHold, 0.35f);
        EXPECT_FLOAT_EQ(pace.mReturn, 0.5f);
    }

    TEST(CutWheelPaceTest, TheResponseSpeedsTheAttackNotItsHolds)
    {
        // K2_WEAPON_PACE: 1.5 x the response is 1.5 x the attack speed.
        PaceInput input;
        input.mWeight = 20.0f;
        const Pace plain = attackPace(input);
        input.mResponse = 1.5f;
        const Pace fast = attackPace(input);
        EXPECT_NEAR(fast.mAttackSpeed, 1.5f * plain.mAttackSpeed, 1e-5f);
        EXPECT_NEAR(fast.mWind, plain.mWind / 1.5f, 1e-5f);
        EXPECT_NEAR(fast.mCut, plain.mCut / 1.5f, 1e-5f);
        EXPECT_NEAR(fast.mReturn, plain.mReturn / 1.5f, 1e-5f);
        EXPECT_FLOAT_EQ(fast.mReleaseHold, plain.mReleaseHold);
        // The dagger's short pace too.
        input = {};
        input.mWeight = 3.0f;
        input.mShort = true;
        const float shortCut = attackPace(input).mCut;
        input.mResponse = 1.5f;
        EXPECT_NEAR(attackPace(input).mCut, shortCut / 1.5f, 1e-5f);
        // The route covers a wind-up in 1/1.5 the time.
        const auto timeToWind = [](const Pace& pace) {
            RouteFlow flow;
            float t = 0.0f;
            while (flow.mU < 0.99f && t < 2.0f)
            {
                stepRouteFlow(flow, 1.0f, 1.0f / 600.0f, pace, 1.0f);
                t += 1.0f / 600.0f;
            }
            return t;
        };
        EXPECT_NEAR(timeToWind(fast), timeToWind(plain) / 1.5f, 0.01f);
    }

    TEST(CutWheelPaceTest, RiposteChainsAndControl)
    {
        PaceInput input;
        const float plain = attackPace(input).mWind;
        input.mPlayerRiposte = true;
        EXPECT_NEAR(attackPace(input).mWind, 0.5f * plain, 1e-6f);
        EXPECT_NEAR(attackPace(input).mWindHaste, 2.0f, 1e-5f);
        input.mPlayerRiposte = false;
        input.mChainFlowGain = 1.0f;
        EXPECT_NEAR(attackPace(input).mWind, 0.6f * plain, 1e-6f);
        // No control: the return takes 1.2 times as long; from 0.2 up it
        // shortens to the full-control time at 1.
        input = {};
        input.mControl = 0.0f;
        EXPECT_NEAR(attackPace(input).mReturn, 0.48f, 1e-6f);
        input.mControl = 0.6f;
        EXPECT_NEAR(attackPace(input).mReturn, 0.4f * 1.1f, 1e-6f);
    }

    // A click's wind-up: the target ramps 0 -> 1 over the click's wind time,
    // as in the game, and the cut begins once the flow reaches 0.97. (Held on
    // the chamber the old flow - kept by the port - passes it by about 6% and
    // rings; with a riposte's haste it never settles. In the game the cut has
    // begun by then; the design's "Known" has it.)
    struct ClickRun
    {
        float mReached = -1.0f; // s, when the flow reached 0.97
        float mFastest = 0.0f;
        float mTop = 0.0f;
    };

    ClickRun runClick(float weight, bool riposte)
    {
        PaceInput input;
        input.mWeight = weight;
        input.mPlayerRiposte = riposte;
        const Pace pace = attackPace(input);
        ClickRun run;
        run.mTop = 5.0f * pace.mAttackSpeed * std::sqrt(1.6f / pace.mCarried) * pace.mWindHaste;
        RouteFlow flow;
        const float dt = 1.0f / 60.0f;
        for (int step = 1; step <= 120 && run.mReached < 0.0f; ++step)
        {
            const float target = std::min(1.0f, step * dt / pace.mWind);
            stepRouteFlow(flow, target, dt, pace, pace.mWindHaste);
            run.mFastest = std::max(run.mFastest, flow.mV);
            if (flow.mU >= 0.97f)
                run.mReached = step * dt;
        }
        return run;
    }

    TEST(CutWheelFlowTest, AClickReachesItsChamberAtTheWeaponsPace)
    {
        PaceInput input;
        const Pace pace = attackPace(input);
        const ClickRun run = runClick(20.0f, false);
        ASSERT_GT(run.mReached, 0.0f);
        EXPECT_GE(run.mReached, pace.mWind - 1e-4f);
        EXPECT_LT(run.mReached, pace.mWind + 0.1f);
        EXPECT_LE(run.mFastest, run.mTop + 1e-4f);
    }

    TEST(CutWheelFlowTest, HeavierLaterARiposteSooner)
    {
        const ClickRun longsword = runClick(20.0f, false);
        const ClickRun warhammer = runClick(70.0f, false);
        const ClickRun riposte = runClick(20.0f, true);
        ASSERT_GT(longsword.mReached, 0.0f);
        ASSERT_GT(warhammer.mReached, 0.0f);
        ASSERT_GT(riposte.mReached, 0.0f);
        EXPECT_GT(warhammer.mReached, longsword.mReached);
        EXPECT_LT(riposte.mReached, longsword.mReached);
        EXPECT_LE(riposte.mFastest, riposte.mTop + 1e-4f);
    }

    TEST(CutWheelFlowTest, ADragHeldStillSettles)
    {
        // A stance held at half wind-up (no haste): the flow comes to rest on it.
        PaceInput input;
        const Pace pace = attackPace(input);
        RouteFlow flow;
        for (int step = 0; step < 120; ++step)
            stepRouteFlow(flow, 0.5f, 1.0f / 60.0f, pace, 1.0f);
        EXPECT_FLOAT_EQ(flow.mU, 0.5f);
        EXPECT_FLOAT_EQ(flow.mV, 0.0f);
    }

    TEST(CutWheelLinesTest, TheSpearSweepRisesAcrossTheFront)
    {
        const auto backhand = [](const WheelLines& lines) -> const AttackLine& {
            for (const AttackLine& line : lines)
                if (line.mName == "backhand")
                    return line;
            return lines.front();
        };
        const AttackLine& sword = backhand(weaponLines());
        const AttackLine& spear = backhand(spearLines());
        // The chamber joins wind-up and cut, the tip not behind the back.
        EXPECT_FLOAT_EQ(spear.mWind.back().mHand.GetX(), spear.mCut.front().mHand.GetX());
        EXPECT_GT(spear.mWind.back().mBlade.GetY(), -0.2f);
        EXPECT_LT(sword.mWind.back().mBlade.GetY(), -0.5f);
        // Rising, where the sword's falls.
        EXPECT_GT(spear.mCut.back().mHand.GetZ(), spear.mCut.front().mHand.GetZ());
        EXPECT_LT(sword.mCut.back().mHand.GetZ(), sword.mCut.front().mHand.GetZ());
        EXPECT_EQ(spear.mDragRight, sword.mDragRight);
    }

    TEST(CutWheelStanceTest, WindUpAlongTheCircle)
    {
        EXPECT_FLOAT_EQ(windAt(0.0f), 0.0f);
        EXPECT_FLOAT_EQ(windAt(150.0f), 0.25f);
        EXPECT_FLOAT_EQ(windAt(300.0f), 0.5f);
        EXPECT_FLOAT_EQ(windAt(1800.0f), 1.0f);
        EXPECT_FLOAT_EQ(windAt(5000.0f), 1.0f);
        // The same slope either side of the circle's edge (FLOW-01).
        const float inside = (windAt(300.0f) - windAt(299.0f)) / 1.0f;
        const float outside = (windAt(301.0f) - windAt(300.0f)) / 1.0f;
        EXPECT_NEAR(inside, outside, 1e-5f);
    }

    // Following a drag (a target moving on with a wobble) the route lands on
    // it instead of overshooting and coming back (play 11-06: the dagger's
    // wind-up vibrated).
    TEST(CutWheelPaceTest, TheRouteFollowsADragWithoutChatter)
    {
        for (const float weight : { 3.0f, 20.0f, 30.0f })
        {
            PaceInput input;
            input.mWeight = weight;
            input.mShort = weight < 5.0f;
            input.mResponse = 1.5f;
            const Pace pace = attackPace(input);
            RouteFlow flow;
            int reversals = 0;
            float lastStep = 0.0f;
            for (int i = 0; i < 240; ++i)
            {
                const float target = std::min(1.0f, 0.02f * i + 0.01f * std::sin(i * 0.9f));
                const float before = flow.mU;
                stepRouteFlow(flow, target, 1.0f / 60.0f, pace, 1.0f);
                const float step = flow.mU - before;
                if (step * lastStep < 0.0f)
                    ++reversals;
                if (std::abs(step) > 1e-6f)
                    lastStep = step;
            }
            EXPECT_LE(reversals, 2) << "weight " << weight;
        }
    }

    // K2_DAGGER_THRUST and K2_MOMENTUM_WIND.
    TEST(CutWheelPaceTest, DaggerThrustsAndMomentum)
    {
        PaceInput input;
        input.mWeight = 3.0f;
        input.mShort = true;
        const Pace shortPace = attackPace(input);
        input.mDaggerThrust = true;
        const Pace thrust = attackPace(input);
        EXPECT_LT(thrust.mWind, shortPace.mWind * 0.5f);
        EXPECT_LT(thrust.mCut, shortPace.mCut);
        EXPECT_LT(thrust.mReturn, shortPace.mReturn * 0.5f);
        // NPCs keep their pace.
        input.mPlayer = false;
        EXPECT_FLOAT_EQ(attackPace(input).mWind, [] {
            PaceInput npc;
            npc.mWeight = 3.0f;
            npc.mShort = true;
            npc.mPlayer = false;
            return attackPace(npc).mWind;
        }());
        // K2_SPEAR_THRUST_PACE: quicker than the spear's weight, slower than the dagger.
        PaceInput spear;
        spear.mWeight = 30.0f;
        spear.mTwoHanded = true;
        const Pace heavy = attackPace(spear);
        spear.mSpearThrustWind = 0.1f;
        spear.mSpearThrustCut = 0.14f;
        spear.mSpearThrustReturn = 0.3f;
        const Pace quick = attackPace(spear);
        EXPECT_LT(quick.mWind, heavy.mWind * 0.5f);
        EXPECT_LT(quick.mCut, heavy.mCut);
        EXPECT_LT(quick.mReturn, heavy.mReturn);
        EXPECT_GT(quick.mWind, thrust.mWind);
        EXPECT_GT(quick.mCut, thrust.mCut);
        spear.mPlayer = false;
        EXPECT_GT(attackPace(spear).mWind, quick.mWind * 2.0f);
        PaceInput sword;
        sword.mWeight = 20.0f;
        const Pace still = attackPace(sword);
        sword.mMomentumHaste = 1.5f;
        const Pace moving = attackPace(sword);
        EXPECT_NEAR(moving.mWind, still.mWind / 1.5f, 1e-5f);
        EXPECT_NEAR(moving.mWindHaste, still.mWindHaste * 1.5f, 1e-5f);
        sword.mMomentumHaste = std::nanf("");
        EXPECT_FLOAT_EQ(attackPace(sword).mWind, still.mWind);
    }

    TEST(CutWheelPaceTest, ALightWeaponPacesLikeTheIronOneOfItsKind)
    {
        // The glass claymore (16.2) at the iron claymore's 27; the daedric (81)
        // keeps its own; share 0 = the weapon's own weight.
        EXPECT_FLOAT_EQ(pacedWeaponWeight(16.2f, PacedWeaponKind::LongBladeTwoHand, 1.0f), 27.0f);
        EXPECT_FLOAT_EQ(pacedWeaponWeight(81.0f, PacedWeaponKind::LongBladeTwoHand, 1.0f), 81.0f);
        EXPECT_FLOAT_EQ(pacedWeaponWeight(16.2f, PacedWeaponKind::LongBladeTwoHand, 0.0f), 16.2f);
        EXPECT_FLOAT_EQ(pacedWeaponWeight(4.8f, PacedWeaponKind::BluntTwoWide, 1.0f), 8.0f);
        EXPECT_FLOAT_EQ(pacedWeaponWeight(8.4f, PacedWeaponKind::Spear, 0.5f), 8.4f);
        EXPECT_FLOAT_EQ(pacedWeaponWeight(5.0f, PacedWeaponKind::Other, 1.0f), 5.0f);
        PaceInput glass;
        glass.mTwoHanded = true;
        glass.mWeight = pacedWeaponWeight(16.2f, PacedWeaponKind::LongBladeTwoHand, 1.0f);
        PaceInput steel = glass;
        steel.mWeight = 27.0f;
        EXPECT_FLOAT_EQ(attackPace(glass).mCut, attackPace(steel).mCut);
    }
}
