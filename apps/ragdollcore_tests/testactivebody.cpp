#include <ragdollcore/activebody.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>

namespace
{
    using namespace RagdollCore;

    // Pelvis, chest, two feet: enough to exercise roles and routing.
    ActiveBodyInput standingBody()
    {
        ActiveBodyInput input;
        input.mParts.resize(4);
        input.mParts[0] = { 10.0f, JPH::Vec3::sZero(), JPH::Vec3::sZero(), true, true, false, false };
        input.mParts[1] = { 40.0f, JPH::Vec3::sZero(), JPH::Vec3::sZero(), true, true, false, false };
        input.mParts[2] = { 12.5f, JPH::Vec3::sZero(), JPH::Vec3::sZero(), false, false, true, true };
        input.mParts[3] = { 12.5f, JPH::Vec3::sZero(), JPH::Vec3::sZero(), false, false, true, true };
        return input;
    }

    TEST(RagdollCoreActiveBody, holdsUpperWeightExternallyWithoutRouting)
    {
        ActiveBodySettings settings;
        settings.mGroundRouting = 0.0f;
        const ActiveBodyOutput out = computeActiveBodyWrench(settings, standingBody());
        ASSERT_TRUE(out.mValid);
        // Upper parts (50 mass) get their weight; feet get nothing.
        EXPECT_NEAR(out.mNetForce.GetZ(), 50.0f * 686.0f, 1.0f);
        EXPECT_TRUE(out.mForces[2].IsNearZero());
    }

    TEST(RagdollCoreActiveBody, fullRoutingSendsTheWeightThroughGroundedFeet)
    {
        ActiveBodySettings settings;
        settings.mGroundRouting = 1.0f;
        settings.mGroundRoutingHorizontal = 1.0f;
        const ActiveBodyOutput out = computeActiveBodyWrench(settings, standingBody());
        ASSERT_TRUE(out.mValid);
        // No external force remains: the feet are pressed down by the weight
        // the upper body is lifted by, so the ground carries it.
        EXPECT_NEAR(out.mNetForce.Length(), 0.0f, 1e-2f);
        EXPECT_NEAR(out.mForces[2].GetZ(), -25.0f * 686.0f, 1.0f);
        EXPECT_NEAR(out.mForces[3].GetZ(), -25.0f * 686.0f, 1.0f);
    }

    TEST(RagdollCoreActiveBody, fullRoutingHoldsNothingInTheAir)
    {
        ActiveBodySettings settings;
        settings.mGroundRouting = 1.0f;
        settings.mGroundRoutingHorizontal = 1.0f;
        ActiveBodyInput input = standingBody();
        input.mParts[2].mGrounded = input.mParts[3].mGrounded = false;
        const ActiveBodyOutput out = computeActiveBodyWrench(settings, input);
        ASSERT_TRUE(out.mValid);
        EXPECT_NEAR(out.mNetForce.Length(), 0.0f, 1e-2f);
        EXPECT_TRUE(out.mForces[0].IsNearZero());
    }

    TEST(RagdollCoreActiveBody, airCatchRestoresSupportAfterLongFlight)
    {
        ActiveBodySettings settings;
        settings.mGroundRouting = 1.0f;
        ActiveBodyInput input = standingBody();
        input.mParts[2].mGrounded = input.mParts[3].mGrounded = false;
        input.mAirborneTime = 0.05f; // a stride's flight: no support
        EXPECT_NEAR(computeActiveBodyWrench(settings, input).mNetForce.GetZ(), 0.0f, 1.0f);
        input.mAirborneTime = 0.5f; // flung off its feet: caught
        EXPECT_NEAR(computeActiveBodyWrench(settings, input).mNetForce.GetZ(), 50.0f * 686.0f, 1.0f);
    }

    TEST(RagdollCoreActiveBody, horizontalPullIsCappedInG)
    {
        ActiveBodySettings settings;
        settings.mHorizontalCap = 1.0f;
        settings.mWeightSupport = 0.0f;
        ActiveBodyInput input = standingBody();
        input.mTargetPosition = JPH::Vec3(1000.0f, 0.0f, 0.0f);
        const ActiveBodyOutput out = computeActiveBodyWrench(settings, input);
        ASSERT_TRUE(out.mValid);
        EXPECT_TRUE(out.mSaturated);
        // Whole-body acceleration is at most 1 g (75 mass).
        EXPECT_NEAR(out.mNetForce.GetX(), 75.0f * 686.0f, 1.0f);
    }

    TEST(RagdollCoreActiveBody, stiffSpringDoesNotRingAtTheStep)
    {
        // A single body integrated at 60 Hz with a 30 Hz spring: explicit PD
        // would flip sign every step; stable PD settles.
        ActiveBodySettings settings;
        settings.mHorizontalFrequency = 30.0f;
        settings.mHorizontalCap = 1000.0f;
        settings.mWeightSupport = 0.0f;
        ActiveBodyInput input;
        input.mParts = { { 10.0f, JPH::Vec3::sZero(), JPH::Vec3::sZero(), true, true, false, false } };
        float x = -10.0f;
        float v = 0.0f;
        int flips = 0;
        float previous = 0.0f;
        for (int step = 0; step < 120; ++step)
        {
            input.mRootPosition = JPH::Vec3(x, 0.0f, 0.0f);
            input.mParts[0].mVelocity = JPH::Vec3(v, 0.0f, 0.0f);
            const ActiveBodyOutput out = computeActiveBodyWrench(settings, input);
            v += out.mForces[0].GetX() / 10.0f * input.mDuration;
            x += v * input.mDuration;
            if (step > 0 && v * previous < 0.0f)
                ++flips;
            previous = v;
        }
        EXPECT_LE(flips, 1);
        EXPECT_NEAR(x, 0.0f, 0.1f);
    }

    TEST(RagdollCoreActiveBody, loadsSettingsFileAndReportsBadKeys)
    {
        const char* path = "activebody-test.cfg";
        {
            std::ofstream file(path);
            file << "# comment\nground_routing = 0.75\ntilt_cap=12 # trailing\nnot_a_key = 1\n";
        }
        ActiveBodySettings settings;
        const auto problems = settings.load(path);
        std::remove(path);
        EXPECT_FLOAT_EQ(settings.mGroundRouting, 0.75f);
        EXPECT_FLOAT_EQ(settings.mTiltCap, 12.0f);
        ASSERT_EQ(problems.size(), 1u);
    }
}
