#include <ragdollcore/wheelaim.hpp>
#include <ragdollcore/wheelplanner.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace
{
    using namespace RagdollCore;

    constexpr float sPi = std::numbers::pi_v<float>;
    constexpr float sDegree = sPi / 180.0f;
    constexpr float sChop = 0.5f * sPi;
    constexpr float sThrust = 1.5f * sPi;
    constexpr float sTip = 40.0f;

    TEST(WheelAimTest, WithNothingAimedTheLookPitch)
    {
        WheelAimInput input;
        input.mLookPitch = 0.3f; // looking down
        WheelAim aim = solveWheelAim(input, sChop, sTip);
        EXPECT_FLOAT_EQ(aim.mElevation, -0.3f);
        EXPECT_FLOAT_EQ(aim.mAzimuth, 0.0f);
        EXPECT_FLOAT_EQ(aim.mDistance, 0.0f);
        EXPECT_FLOAT_EQ(aim.mFoeAssist, 0.0f);
        EXPECT_FLOAT_EQ(aim.mFloor, 0.9f);
        EXPECT_FLOAT_EQ(aim.mThrustness, 0.0f);
        EXPECT_FLOAT_EQ(solveWheelAim(input, sThrust, sTip).mThrustness, 1.0f);
        input.mLookPitch = 2.0f;
        EXPECT_FLOAT_EQ(solveWheelAim(input, sChop, sTip).mElevation, -0.6f);
    }

    TEST(WheelAimTest, TheCrosshairPoint)
    {
        // The camera behind and above the chest, the ray at a point ahead.
        WheelAimInput input;
        input.mRayOrigin = JPH::Vec3(0.0f, -60.0f, 30.0f);
        const JPH::Vec3 point(10.0f, 50.0f, -5.0f);
        input.mRayDirection = (point - *input.mRayOrigin).Normalized();
        // The axis it is taken at: no foe, the sweet spot (35 + 0.55 x tip)
        // straight along the flat ray.
        const WheelAim aim = solveWheelAim(input, sChop, sTip);
        EXPECT_GT(aim.mDistance, 0.0f);
        EXPECT_GT(aim.mAzimuth, 0.0f); // to the right
        EXPECT_LT(aim.mElevation, 0.0f); // below the chest
        EXPECT_NEAR(aim.mDirection.Length(), 1.0f, 1e-5f);
        // NPCs have no crosshair.
        input.mPlayer = false;
        EXPECT_FLOAT_EQ(solveWheelAim(input, sChop, sTip).mDistance, 0.0f);
    }

    TEST(WheelAimTest, TheFoeAssistTakesTheAimNearIt)
    {
        WheelAimInput input;
        input.mFoe = JPH::Vec3(50.0f * std::sin(10.0f * sDegree), 50.0f * std::cos(10.0f * sDegree), 0.0f);
        WheelAim aim = solveWheelAim(input, sChop, sTip);
        EXPECT_FLOAT_EQ(aim.mFoeAssist, 1.0f);
        EXPECT_NEAR(aim.mAzimuth, 10.0f * sDegree, 1e-5f);
        // 30 deg off: fading between 20 and 35.
        input.mFoe = JPH::Vec3(50.0f * std::sin(30.0f * sDegree), 50.0f * std::cos(30.0f * sDegree), 0.0f);
        aim = solveWheelAim(input, sChop, sTip);
        EXPECT_NEAR(aim.mFoeAssist, 1.0f / 3.0f, 1e-4f);
        EXPECT_NEAR(aim.mAzimuth, 10.0f * sDegree, 1e-4f);
        // Out of reach (tip + 40): no assist, but the lunge gap.
        input.mFoe = JPH::Vec3(0.0f, 100.0f, 0.0f);
        aim = solveWheelAim(input, sChop, sTip);
        EXPECT_FLOAT_EQ(aim.mFoeAssist, 0.0f);
        EXPECT_NEAR(aim.mLungeGap, 100.0f - (55.0f + 0.6f * sTip + 12.0f), 1e-4f);
    }

    TEST(WheelAimTest, TheAimStaysOnTheFoesBody)
    {
        // Looking up (elevation 0.6) at a foe ahead: kept down to its head
        // (25 above its chest).
        WheelAimInput input;
        input.mLookPitch = -1.0f;
        input.mFoe = JPH::Vec3(0.0f, 50.0f, 0.0f);
        WheelAim aim = solveWheelAim(input, sChop, sTip);
        EXPECT_NEAR(aim.mElevation, std::atan2(25.0f, 50.0f), 1e-5f);
        // Looking far down at a standing foe: kept up to its shins (95 below).
        input.mLookPitch = 1.0f;
        aim = solveWheelAim(input, sChop, sTip);
        EXPECT_NEAR(aim.mElevation, -0.6f, 1e-5f); // the look's own floor
        // A downed foe: the floor is 1.3 and the band +-15 about its chest.
        // A body lying 80 below the chest, 40 ahead: 1.11 rad down, past the
        // standing floor (0.9) but inside the downed one (1.3).
        input.mFoeDowned = true;
        input.mFoe = JPH::Vec3(0.0f, 40.0f, -80.0f);
        input.mRayOrigin = JPH::Vec3(0.0f, -40.0f, 40.0f);
        input.mRayDirection = (*input.mFoe - *input.mRayOrigin).Normalized();
        aim = solveWheelAim(input, sChop, sTip);
        EXPECT_FLOAT_EQ(aim.mFloor, 1.3f);
        EXPECT_NEAR(aim.mElevation, std::atan2(-80.0f, 40.0f), 1e-3f);
        // Standing that low (its head below the 0.9 floor): the head wins.
        input.mFoeDowned = false;
        EXPECT_NEAR(solveWheelAim(input, sChop, sTip).mElevation, std::atan2(-55.0f, 40.0f), 1e-4f);
    }

    TEST(WheelAimTest, AChopsPlaneIsUpright)
    {
        WheelAimInput input;
        const WheelAim aim = solveWheelAim(input, sChop, sTip);
        EXPECT_TRUE(aim.mPlaneNormal.IsClose(JPH::Vec3::sAxisX(), 1e-5f));
        // A forehand (angle 0, stroke right to left): a level plane.
        const WheelAim slash = solveWheelAim(input, 0.0f, sTip);
        EXPECT_NEAR(std::abs(slash.mPlaneNormal.GetZ()), 1.0f, 1e-5f);
    }

    TEST(WheelAimTest, ThePlanesWeight)
    {
        WheelAimInput input;
        const WheelAim aim = solveWheelAim(input, sChop, sTip);
        // A drag's cut: in through its first 0.4, out from 0.65 over 0.25.
        EXPECT_NEAR(planeWeight(aim, sChop, false, 1.5f, true, false), 1.0f, 1e-5f);
        EXPECT_NEAR(planeWeight(aim, sChop, false, 1.95f, true, false), 0.0f, 1e-5f);
        // Its wind-up's second half goes in.
        EXPECT_NEAR(planeWeight(aim, sChop, false, 0.25f, false, true), 0.0f, 1e-5f);
        EXPECT_NEAR(planeWeight(aim, sChop, false, 1.0f, false, true), 1.0f, 1e-5f);
        // A click keeps its keyed line unless a foe is aimed.
        EXPECT_NEAR(planeWeight(aim, sChop, true, 1.5f, true, false), 0.0f, 1e-5f);
        // A thrust stays out of it.
        const WheelAim thrust = solveWheelAim(input, sThrust, sTip);
        EXPECT_NEAR(planeWeight(thrust, sThrust, false, 1.5f, true, false), 0.0f, 1e-5f);
        // A low chamber (a rising cut) stays out in its wind-up.
        const float rising = 1.25f * sPi;
        const WheelAim low = solveWheelAim(input, rising, sTip);
        EXPECT_NEAR(planeWeight(low, rising, false, 1.0f, false, true), 0.0f, 1e-5f);
    }

    TEST(WheelAimTest, InThePlaneTheHandIsOnIt)
    {
        WheelAimInput input;
        const WheelAim aim = solveWheelAim(input, sChop, sTip);
        ChamberPose pose;
        pose.mHand = JPH::Vec3(10.0f, 20.0f, 5.0f);
        pose.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.3f); // the blade forward and up
        putInPlane(pose, aim, 1.0f, JPH::Vec3::sAxisY(), sTip);
        EXPECT_NEAR(pose.mHand.GetX(), 0.0f, 1e-5f);
        EXPECT_NEAR((pose.mRotation * JPH::Vec3::sAxisY()).GetX(), 0.0f, 1e-5f);
        // Half the weight, half the way.
        pose.mHand = JPH::Vec3(10.0f, 20.0f, 5.0f);
        putInPlane(pose, aim, 0.5f, JPH::Vec3::sAxisY(), sTip);
        EXPECT_NEAR(pose.mHand.GetX(), 5.0f, 1e-5f);
    }

    TEST(WheelAimTest, TheHandReachesForTheAimPoint)
    {
        WheelAim aim;
        aim.mPlaneNormal = JPH::Vec3::sAxisX();
        aim.mDirection = JPH::Vec3::sAxisY();
        aim.mDistance = 90.0f;
        ChamberPose pose;
        pose.mHand = JPH::Vec3(0.0f, 20.0f, 0.0f);
        // The blade along the aim: the tip at 60, 30 short: reached out 30.
        putInPlane(pose, aim, 1.0f, JPH::Vec3::sAxisY(), sTip);
        EXPECT_NEAR(pose.mHand.GetY(), 50.0f, 1e-4f);
        // At most 35.
        pose.mHand = JPH::Vec3(0.0f, 0.0f, 0.0f);
        aim.mDistance = 200.0f;
        putInPlane(pose, aim, 1.0f, JPH::Vec3::sAxisY(), sTip);
        EXPECT_NEAR(pose.mHand.GetY(), 35.0f, 1e-4f);
    }

    TEST(WheelAimTest, ThrustsTurnToTheAimAndTheBodyDrivesThem)
    {
        WheelAimInput input;
        input.mFoe = JPH::Vec3(50.0f * std::sin(10.0f * sDegree), 50.0f * std::cos(10.0f * sDegree), 0.0f);
        const WheelAim aim = solveWheelAim(input, sThrust, sTip);
        ChamberPose pose;
        pose.mHand = JPH::Vec3(0.0f, 30.0f, 0.0f);
        turnThrust(pose, aim, 1.0f);
        EXPECT_NEAR(std::atan2(pose.mHand.GetX(), pose.mHand.GetY()), 10.0f * sDegree, 1e-4f);
        EXPECT_FLOAT_EQ(thrustDrive(aim, true, 1.5f, true), 1.0f);
        EXPECT_NEAR(thrustDrive(aim, true, 1.0f, false), -0.35f, 1e-6f);
        EXPECT_FLOAT_EQ(thrustDrive(aim, false, 1.5f, true), 0.0f); // the player's only
        pose = {};
        reachThrust(pose, 1.0f, JPH::Vec3::sAxisY(), false);
        EXPECT_NEAR(pose.mHand.GetY(), 6.0f, 1e-5f);
        pose = {};
        reachThrust(pose, 1.0f, JPH::Vec3::sAxisY(), true); // fists and daggers: SHORT_REACH instead
        EXPECT_FLOAT_EQ(pose.mHand.GetY(), 0.0f);
    }

    TEST(WheelAimTest, ALongHaftsThrustPointsAtTheAim)
    {
        WheelAimInput input;
        input.mFoe = JPH::Vec3(0.0f, 50.0f, 0.0f);
        const WheelAim aim = solveWheelAim(input, sThrust, sTip);
        const auto offLine = [&](const ChamberPose& p) {
            const JPH::Vec3 tip = p.mHand + p.mRotation * JPH::Vec3::sAxisY() * 67.0f;
            return (tip - aim.mDirection * tip.Dot(aim.mDirection)).Length();
        };
        ChamberPose pose;
        pose.mHand = JPH::Vec3(-12.0f, 40.0f, -7.0f); // the hand low on the left, the blade straight ahead
        const float before = offLine(pose);
        EXPECT_GT(before, 10.0f);
        ChamberPose half = pose;
        pointThrust(half, aim, 0.5f, JPH::Vec3::sAxisY(), 67.0f);
        pointThrust(pose, aim, 1.0f, JPH::Vec3::sAxisY(), 67.0f);
        EXPECT_NEAR(offLine(pose), 0.0f, 0.5f);
        EXPECT_GT(offLine(half), 0.3f * before);
        EXPECT_LT(offLine(half), 0.7f * before);
        EXPECT_EQ(pose.mHand, JPH::Vec3(-12.0f, 40.0f, -7.0f)); // about the hand
        // With no foe a pointed thrust's aim point lies at the point's reach.
        WheelAimInput ray;
        ray.mRayOrigin = JPH::Vec3(0.0f, 5.0f, 20.0f);
        ray.mRayDirection = JPH::Vec3::sAxisY();
        const float near = solveWheelAim(ray, sThrust, 67.0f).mElevation;
        ray.mPointReach = 140.0f;
        const WheelAim reached = solveWheelAim(ray, sThrust, 67.0f);
        EXPECT_GT(near, 14.0f * sDegree);
        EXPECT_NEAR(reached.mElevation, std::atan2(20.0f, 140.0f), 0.01f);
        EXPECT_NEAR(solveWheelAim(ray, sChop, 67.0f).mElevation, near, 1e-5f); // thrusts only
        // At most 45 degrees; a blade not ahead is left alone.
        ChamberPose far;
        far.mHand = JPH::Vec3(-160.0f, 30.0f, 0.0f);
        pointThrust(far, aim, 1.0f, JPH::Vec3::sAxisY(), 67.0f);
        EXPECT_NEAR(std::acos((far.mRotation * JPH::Vec3::sAxisY()).Dot(JPH::Vec3::sAxisY())), 45.0f * sDegree, 1e-3f);
        ChamberPose back;
        back.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 3.0f);
        const JPH::Quat kept = back.mRotation;
        pointThrust(back, aim, 1.0f, JPH::Vec3::sAxisY(), 67.0f);
        EXPECT_EQ(back.mRotation, kept);
    }

    TEST(WheelAimTest, SteeringMovesTheAimAndALowAimTakesTheLowStance)
    {
        WheelAimInput input;
        WheelAim aim = solveWheelAim(input, sChop, sTip);
        steerWheelAim(aim, sChop, 0.3f, -0.85f, false);
        EXPECT_FLOAT_EQ(aim.mAzimuth, 0.3f);
        EXPECT_FLOAT_EQ(aim.mElevation, -0.85f);
        EXPECT_GT(aim.mLowWant, 0.9f);
        steerWheelAim(aim, sChop, 2.0f, 0.0f, false);
        EXPECT_FLOAT_EQ(aim.mAzimuth, 0.8f);
    }

    // The planner with an aim: a dragged chop cuts down its upright plane,
    // and a committed click shows its trajectory.
    struct AimedRig
    {
        WheelPlanner mPlanner;
        WheelStriker mStriker;
        WheelActor mActor;
        WheelHooks mHooks;
        WheelAimInput mAim;

        AimedRig()
        {
            mStriker.mAxes = { JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX() };
            mHooks.mClickLine = [] { return sChop; };
        }

        void step(bool held, float dragUp = 0.0f)
        {
            WheelInput input;
            input.mHeld = held;
            input.mDragUp = dragUp;
            WheelEvents events;
            mPlanner.step(input, mStriker, mActor, mHooks, &mAim, events);
        }
    };

    TEST(WheelAimTest, ADraggedChopCutsDownItsPlane)
    {
        AimedRig rig;
        rig.step(true);
        for (int i = 0; i < 12; ++i)
            rig.step(true, 100.0f);
        for (int i = 0; i < 4 && rig.mPlanner.stage() != WheelStage::Cut; ++i)
            rig.step(true, -100.0f);
        ASSERT_EQ(rig.mPlanner.stage(), WheelStage::Cut);
        float worst = 0.0f;
        for (int i = 0; i < 20 && rig.mPlanner.stage() == WheelStage::Cut; ++i)
        {
            rig.step(true, -150.0f);
            if (rig.mPlanner.shownU() > 1.2f && rig.mPlanner.shownU() < 1.6f)
                worst = std::max(worst, std::abs(rig.mPlanner.lastPose()->mHand.GetX()));
        }
        EXPECT_LT(worst, 0.5f);
    }

    // A click's wind-up shows no path either (user 11-08); its cut does.
    TEST(WheelAimTest, ACommittedClickShowsItsTrajectoryInTheCut)
    {
        AimedRig rig;
        rig.mAim.mFoe = JPH::Vec3(0.0f, 60.0f, 0.0f);
        rig.step(true);
        rig.step(true);
        rig.step(false);
        rig.step(false);
        ASSERT_EQ(rig.mPlanner.stage(), WheelStage::Wind);
        EXPECT_TRUE(rig.mPlanner.preview().empty());
        for (int i = 0; i < 120 && rig.mPlanner.preview().empty(); ++i)
            rig.step(false);
        EXPECT_EQ(rig.mPlanner.preview().size(), 16u);
        rig.mActor.mPlayer = false;
        rig.step(false);
        EXPECT_TRUE(rig.mPlanner.preview().empty());
    }
}
