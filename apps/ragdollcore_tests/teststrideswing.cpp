#include <ragdollcore/strideswing.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
    using namespace RagdollCore;

    constexpr float sGravity = 627.0f;

    // The left foot halfway through a 40-unit step forward; the hip 60 units
    // up over the start, the leg 70 long.
    StrideSwingInput halfway()
    {
        StrideSwingInput input;
        input.mGravity = sGravity;
        input.mSettings.mFrequency = 6.0f;
        input.mSettings.mReach = 1.0f;
        StrideSwingFoot& foot = input.mFeet[0];
        foot.mSwinging = true;
        foot.mPhase = 0.5f;
        foot.mDuration = 0.3f;
        foot.mLiftOff = { 0.0f, 0.0f };
        foot.mLanding = { 0.0f, 40.0f };
        foot.mLandingHeight = 7.5f;
        foot.mFoot = { 0.0f, 5.0f };
        foot.mHip = { 0.0f, 20.0f, 67.5f };
        foot.mLegLength = 70.0f;
        foot.mLegMass = 12.0f;
        return input;
    }

    TEST(RagdollCoreStrideSwing, offAtZeroFrequency)
    {
        StrideSwingInput input = halfway();
        input.mSettings.mFrequency = 0.0f;
        const auto out = computeStrideSwing(input);
        ASSERT_TRUE(out);
        EXPECT_FALSE(out->mFootForce[0]);
        EXPECT_FALSE(out->mFootForce[1]);
    }

    TEST(RagdollCoreStrideSwing, pullsTheSwingFootOnTheReactionOfThePelvis)
    {
        const auto out = computeStrideSwing(halfway());
        ASSERT_TRUE(out);
        ASSERT_TRUE(out->mFootForce[0]);
        EXPECT_FALSE(out->mFootForce[1]); // the planted foot is left alone
        EXPECT_GT(out->mFootForce[0]->mY, 0.0f);
        EXPECT_FLOAT_EQ(out->mPelvisForce.mY, -out->mFootForce[0]->mY);
        EXPECT_FALSE(out->mReachClamped[0]);
    }

    TEST(RagdollCoreStrideSwing, holdsTheLandingInsideTheReachAtTouchdown)
    {
        StrideSwingInput input = halfway();
        input.mFeet[0].mPhase = 1.0f;
        input.mFeet[0].mLanding = { 0.0f, 120.0f }; // 100 ahead of the hip: out of reach
        input.mFeet[0].mFoot = { 0.0f, 54.0f };
        input.mSettings.mFrequency = 2.0f; // under the cap
        const auto out = computeStrideSwing(input);
        ASSERT_TRUE(out);
        EXPECT_TRUE(out->mReachClamped[0]);
        // Reach across at 60 under the hip: sqrt(70^2 - 60^2) = 36: the goal 56, the foot pulled toward it.
        const auto unclamped = [&] {
            StrideSwingInput far = input;
            far.mSettings.mReach = 0.0f; // no clamp
            return computeStrideSwing(far);
        }();
        ASSERT_TRUE(unclamped);
        EXPECT_LT(out->mFootForce[0]->mY, unclamped->mFootForce[0]->mY);
        EXPECT_GT(out->mFootForce[0]->mY, 0.0f);
        EXPECT_FALSE(out->mCapped[0]);
    }

    TEST(RagdollCoreStrideSwing, theHipIsCarriedToTouchdown)
    {
        StrideSwingInput input = halfway();
        input.mFeet[0].mLanding = { 0.0f, 70.0f };
        const auto still = computeStrideSwing(input);
        input.mPelvisVelocity = { 0.0f, 200.0f }; // 0.15 s left: the hip 30 further on
        const auto moving = computeStrideSwing(input);
        ASSERT_TRUE(still && moving);
        EXPECT_TRUE(still->mReachClamped[0]);
        EXPECT_FALSE(moving->mReachClamped[0]);
    }

    TEST(RagdollCoreStrideSwing, rejectsBadInput)
    {
        StrideSwingInput input = halfway();
        input.mGravity = 0.0f;
        EXPECT_FALSE(computeStrideSwing(input));
        input = halfway();
        input.mFeet[0].mHip[2] = NAN;
        EXPECT_FALSE(computeStrideSwing(input));
    }
}
