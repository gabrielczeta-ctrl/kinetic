#include <ragdollcore/handdrive.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace
{
    using namespace RagdollCore;

    constexpr float sGravity = 627.2f;

    // An arm held out forward from a shoulder at the origin: elbow 30 ahead,
    // wrist 55 ahead, the grip 60 ahead; a 1 kg weapon 40 past the grip.
    HandChain straightArm()
    {
        HandChain chain;
        chain.mJoint = { JPH::RVec3(0, 0, 0), JPH::RVec3(0, 30, 0), JPH::RVec3(0, 55, 0) };
        chain.mBody[0] = { JPH::RVec3(0, 15, 0), 2.0f };
        chain.mBody[1] = { JPH::RVec3(0, 42, 0), 1.5f };
        chain.mBody[2] = { JPH::RVec3(0, 58, 0), 0.5f };
        chain.mBody[3] = { JPH::RVec3(0, 100, 0), 1.0f };
        chain.mGrip = JPH::RVec3(0, 60, 0);
        chain.mStrength = { 1e9f, 1e9f, 1e9f };
        return chain;
    }

    HandDriveGoal holdStill(const HandChain& chain)
    {
        HandDriveGoal goal;
        goal.mGrip = chain.mGrip;
        goal.mRotation = chain.mHandRotation;
        return goal;
    }

    TEST(HandDriveTest, TheResponseStiffensTheSpringsAndRaisesTheirCaps)
    {
        const HandDriveGains plain = handDriveGains(20.0f, 1.0f);
        const HandDriveGains fast = handDriveGains(20.0f, 1.0f, 1.5f);
        EXPECT_NEAR(fast.mFrequency, 1.5f * plain.mFrequency, 1e-4f);
        EXPECT_NEAR(fast.mRotationFrequency, 1.5f * plain.mRotationFrequency, 1e-4f);
        EXPECT_NEAR(fast.mMaxAngularAcceleration, 2.25f * plain.mMaxAngularAcceleration, 1e-2f);
        EXPECT_NEAR(fast.mMaxAcceleration, 2.25f * plain.mMaxAcceleration, 1.0f);
        EXPECT_FLOAT_EQ(fast.mStrength, plain.mStrength);
    }

    TEST(HandDriveTest, AWeaponFarFromTheGripIsLighterThereThanItsMass)
    {
        HandChain chain = straightArm();
        const float whole = solveHandDrive(chain, holdStill(chain), HandDriveGains{}, sGravity).mEffectiveMass;
        chain.mWeaponInertia = 100.0f;
        const float near = solveHandDrive(chain, holdStill(chain), HandDriveGains{}, sGravity).mEffectiveMass;
        // The far weapon (40 past the grip) pushed at the grip mostly turns.
        EXPECT_LT(near, whole);
        EXPECT_GT(near, 0.5f * chain.mBody[1].mMass + 0.25f * chain.mBody[0].mMass);
        // Nearer the grip it counts for more.
        chain.mBody[3].mCenterOfMass = JPH::RVec3(0, 70, 0);
        const float closer = solveHandDrive(chain, holdStill(chain), HandDriveGains{}, sGravity).mEffectiveMass;
        EXPECT_GT(closer, near);
        EXPECT_LT(closer, whole);
    }

    TEST(HandDriveTest, TheBladesTurnIsSizedOnItsInertiaAboutItsCentre)
    {
        HandChain chain = straightArm();
        HandDriveGoal goal = holdStill(chain);
        goal.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.3f);
        HandDriveGains gains;
        gains.mComputeRotation = true;
        gains.mRotationTorque = false;
        const float wrist = solveHandDrive(chain, goal, gains, sGravity).mRotationTorque.Length();
        chain.mWeaponInertia = 100.0f;
        const float centre = solveHandDrive(chain, goal, gains, sGravity).mRotationTorque.Length();
        // The 1 kg weapon 40 past the grip: ~1600 about the wrist as a point,
        // ~100 plus the hand's share about the pair's centre.
        EXPECT_LT(centre, 0.5f * wrist);
        EXPECT_GT(centre, 0.0f);
    }

    TEST(HandDriveTest, AHaftGrippedAtItsMiddleStillTakesTorqueToTurn)
    {
        // A spear held at its centre: the weapon's centre of mass at the wrist.
        HandChain chain = straightArm();
        chain.mBody[3].mCenterOfMass = chain.mJoint[2];
        HandDriveGoal goal = holdStill(chain);
        goal.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.3f);
        HandDriveGains gains;
        gains.mComputeRotation = true;
        gains.mRotationTorque = false;
        const float point = solveHandDrive(chain, goal, gains, sGravity).mRotationTorque.Length();
        chain.mTurnInertia = 1500.0f;
        const float real = solveHandDrive(chain, goal, gains, sGravity).mRotationTorque.Length();
        // As a point at the wrist only the hand's 0.5 kg 3 out counts (4.5).
        EXPECT_GT(real, 100.0f * point);
        EXPECT_NEAR(real / point, (1500.0f + 4.5f) / 4.5f, 1.0f);
    }

    TEST(HandDriveTest, OnTargetItOnlyHoldsTheWeightUp)
    {
        const HandChain chain = straightArm();
        const HandDriveOutput out = solveHandDrive(chain, holdStill(chain), HandDriveGains{}, sGravity);
        EXPECT_TRUE(out.mForce.IsNearZero(1e-6f));
        // Each joint carries the weight of everything past it: the torque
        // about it is the levers crossed with the upward support.
        for (std::size_t j = 0; j < 3; ++j)
        {
            JPH::Vec3 expected = JPH::Vec3::sZero();
            for (std::size_t b = j; b < 4; ++b)
                expected += JPH::Vec3(chain.mBody[b].mCenterOfMass - chain.mJoint[j])
                                .Cross(JPH::Vec3(0, 0, chain.mBody[b].mMass * sGravity));
            EXPECT_TRUE(out.mTorque[j].IsClose(expected, 1e-2f)) << "joint " << j;
        }
        // Held out forward, the support turns the arm up about +X... the
        // shoulder carries the most.
        EXPECT_GT(out.mTorque[0].Length(), out.mTorque[1].Length());
        EXPECT_GT(out.mTorque[1].Length(), out.mTorque[2].Length());
    }

    TEST(HandDriveTest, TheGripForceIsSharedByTheLevers)
    {
        const HandChain chain = straightArm();
        HandDriveGoal goal = holdStill(chain);
        goal.mGrip += JPH::RVec3(0, 0, 2); // 2 units up
        HandDriveGains gains;
        gains.mGravity = false;
        const HandDriveOutput out = solveHandDrive(chain, goal, gains, sGravity);
        const float omega = 2.0f * std::numbers::pi_v<float> * gains.mFrequency;
        const float mass = 0.5f + 1.0f + 0.75f + 0.5f;
        EXPECT_NEAR(out.mEffectiveMass, mass, 1e-5f);
        // The stable PD: the spring taken at the step's end.
        const float kp = omega * omega, kd = 2.0f * omega, dt = gains.mDt;
        const float expected = 2.0f * kp / (1.0f + kd * dt + kp * dt * dt) * mass;
        EXPECT_NEAR(out.mForce.GetZ(), expected, 1e-1f);
        EXPECT_LT(out.mForce.GetZ(), 2.0f * kp * mass);
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_TRUE(out.mTorque[j].IsClose(JPH::Vec3(chain.mGrip - chain.mJoint[j]).Cross(out.mForce), 1e-1f));
    }

    TEST(HandDriveTest, EveryJointCarriesTheHandsTurn)
    {
        const HandChain chain = straightArm();
        HandDriveGoal goal = holdStill(chain);
        goal.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.1f);
        HandDriveGains gains;
        gains.mGravity = false;
        const HandDriveOutput out = solveHandDrive(chain, goal, gains, sGravity);
        EXPECT_GT(out.mTorque[2].GetZ(), 0.0f);
        EXPECT_TRUE(out.mTorque[0].IsClose(out.mTorque[2], 1e-3f));
        EXPECT_TRUE(out.mTorque[1].IsClose(out.mTorque[2], 1e-3f));
    }

    TEST(HandDriveTest, TheVelocityErrorDamps)
    {
        HandChain chain = straightArm();
        chain.mGripVelocity = JPH::Vec3(10, 0, 0); // drifting right
        HandDriveGains gains;
        gains.mGravity = false;
        const HandDriveOutput out = solveHandDrive(chain, holdStill(chain), gains, sGravity);
        EXPECT_LT(out.mForce.GetX(), 0.0f);
    }

    TEST(HandDriveTest, AJointCannotPassItsStrength)
    {
        HandChain chain = straightArm();
        chain.mStrength = { 100.0f, 1e9f, 1e9f };
        const HandDriveOutput out = solveHandDrive(chain, holdStill(chain), HandDriveGains{}, sGravity);
        EXPECT_GT(out.mUtilization[0], 1.0f);
        EXPECT_NEAR(out.mTorque[0].Length(), 100.0f, 1e-2f);
        // A weaker fighter (no control) has 0.6 of it.
        HandDriveGains weak = handDriveGains(20.0f, 0.0f);
        EXPECT_NEAR(solveHandDrive(chain, holdStill(chain), weak, sGravity).mTorque[0].Length(), 60.0f, 1e-2f);
    }

    TEST(HandDriveTest, HeavierWeaponsAreDrivenSlower)
    {
        const HandDriveGains longsword = handDriveGains(20.0f, 1.0f);
        const HandDriveGains warhammer = handDriveGains(70.0f, 1.0f);
        EXPECT_FLOAT_EQ(longsword.mFrequency, 6.0f);
        EXPECT_LT(warhammer.mFrequency, longsword.mFrequency);
        EXPECT_FLOAT_EQ(longsword.mStrength, 1.0f);
    }

    TEST(HandDriveTest, TheElbowIsLedTowardItsPole)
    {
        HandChain chain = straightArm();
        chain.mJoint[1] = JPH::RVec3(0, 30, -5); // elbow bent down
        HandDriveGoal goal = holdStill(chain);
        goal.mPole = JPH::Vec3(1, 0, 0); // wanted out to the right
        HandDriveGains gains;
        gains.mGravity = false;
        const HandDriveOutput with = solveHandDrive(chain, goal, gains, sGravity);
        goal.mPole.reset();
        const HandDriveOutput without = solveHandDrive(chain, goal, gains, sGravity);
        // Only the shoulder turns the elbow: about the arm's line (Y).
        EXPECT_GT(std::abs((with.mTorque[0] - without.mTorque[0]).GetY()), 0.0f);
        EXPECT_TRUE(with.mTorque[1].IsClose(without.mTorque[1], 1e-4f));
    }
}
