#include <ragdollcore/balancehelper.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace
{
    using namespace RagdollCore;

    constexpr float sHeight = 90.0f;
    constexpr float sGravity = 900.0f;
    const float sTau = std::sqrt(sHeight / sGravity);

    float distance(BalanceVec2 a, BalanceVec2 b) { return std::hypot(a.mX - b.mX, a.mY - b.mY); }

    // Left foot stance at x = -6, right foot swinging from x = +6; body
    // faces +y, so right is +x.
    CaptureStepInput standing(BalanceVec2 velocity)
    {
        CaptureStepInput input;
        input.mState.mCenterOfMass = { 0.0f, 0.0f };
        input.mState.mVelocity = velocity;
        input.mState.mHeight = sHeight;
        input.mState.mGravity = sGravity;
        input.mStanceFoot = { -6.0f, 0.0f };
        input.mSwingFoot = { 6.0f, 0.0f };
        input.mBodyForward = { 0.0f, 1.0f };
        input.mSwingIsLeft = false;
        input.mLimits.mFootRadius = 5.0f;
        input.mLimits.mMaximumReach = 45.0f;
        input.mLimits.mMinimumWidth = 8.0f;
        input.mLimits.mMinimumSwingTime = 0.2f;
        input.mLimits.mMaximumSwingTime = 0.35f;
        input.mLimits.mSwingFootSpeed = 400.0f;
        return input;
    }

    TEST(RagdollCoreBalanceHelper, capturePointIsComPlusVelocityTimesTimeConstant)
    {
        BalanceState state{ { 1.0f, 2.0f }, { 10.0f, -20.0f }, sHeight, sGravity };
        ASSERT_TRUE(balanceTimeConstant(state));
        EXPECT_NEAR(*balanceTimeConstant(state), sTau, 1e-6f);
        const auto point = capturePoint(state);
        ASSERT_TRUE(point);
        EXPECT_NEAR(point->mX, 1.0f + 10.0f * sTau, 1e-5f);
        EXPECT_NEAR(point->mY, 2.0f - 20.0f * sTau, 1e-5f);
    }

    TEST(RagdollCoreBalanceHelper, capturePointDivergesExponentiallyFromThePivot)
    {
        BalanceState state{ { 1.0f, 0.0f }, { 0.0f, 0.0f }, sHeight, sGravity };
        const auto later = predictCapturePoint(state, { 0.0f, 0.0f }, sTau);
        ASSERT_TRUE(later);
        EXPECT_NEAR(later->mX, std::exp(1.0f), 1e-5f);
        // A capture point on the pivot stays there: the body is balanced.
        const auto still = predictCapturePoint(state, { 1.0f, 0.0f }, 2.0f);
        ASSERT_TRUE(still);
        EXPECT_NEAR(still->mX, 1.0f, 1e-6f);
    }

    TEST(RagdollCoreBalanceHelper, rejectsInvalidInput)
    {
        BalanceState state{ { 0.0f, 0.0f }, { 0.0f, 0.0f }, 0.0f, sGravity };
        EXPECT_FALSE(capturePoint(state));
        auto input = standing({ 0.0f, 50.0f });
        input.mLimits.mMaximumReach = 0.0f;
        EXPECT_FALSE(planCaptureStep(input));
        input = standing({ 0.0f, 50.0f });
        input.mState.mVelocity.mX = std::nanf("");
        EXPECT_FALSE(planCaptureStep(input));
        input = standing({ 0.0f, 50.0f });
        input.mBodyForward = { 0.0f, 0.0f };
        EXPECT_FALSE(planCaptureStep(input));
        EXPECT_FALSE(planRecoverySteps(standing({ 0.0f, 50.0f }), 0));
    }

    TEST(RagdollCoreBalanceHelper, forwardPushLandsOnTheCapturePointAtTouchdown)
    {
        const auto plan = planCaptureStep(standing({ 0.0f, 60.0f }));
        ASSERT_TRUE(plan);
        EXPECT_TRUE(plan->mCaptures);
        EXPECT_FALSE(plan->mReachClamped);
        // The capture point keeps running during the swing, so the foot lands
        // beyond where it was at the push.
        EXPECT_GT(plan->mPredictedCapturePoint.mY, plan->mCapturePoint.mY);
        EXPECT_NEAR(plan->mLanding.mY, plan->mPredictedCapturePoint.mY, 1e-4f);
        // Own side of the stance foot by at least the minimum width.
        EXPECT_GE(plan->mLanding.mX - (-6.0f), 8.0f - 1e-4f);
        EXPECT_LE(plan->mResidual, 5.0f);
    }

    TEST(RagdollCoreBalanceHelper, pressureAtTheFootEdgeSlowsTheFall)
    {
        auto input = standing({ 0.0f, 60.0f });
        const auto edge = planCaptureStep(input);
        input.mPressureAtFootEdge = false;
        const auto centre = planCaptureStep(input);
        ASSERT_TRUE(edge && centre);
        EXPECT_LT(edge->mPredictedCapturePoint.mY, centre->mPredictedCapturePoint.mY);
    }

    TEST(RagdollCoreBalanceHelper, swingTimeGrowsWithStepLengthWithinItsBounds)
    {
        const auto small = planCaptureStep(standing({ 0.0f, 20.0f }));
        const auto large = planCaptureStep(standing({ 0.0f, 90.0f }));
        ASSERT_TRUE(small && large);
        EXPECT_LT(small->mSwingTime, large->mSwingTime);
        EXPECT_GE(small->mSwingTime, 0.2f);
        EXPECT_LE(large->mSwingTime, 0.35f);
        auto fixed = standing({ 0.0f, 90.0f });
        fixed.mTimeToTouchdown = 0.1f;
        const auto late = planCaptureStep(fixed);
        ASSERT_TRUE(late);
        EXPECT_FLOAT_EQ(late->mSwingTime, 0.1f);
    }

    TEST(RagdollCoreBalanceHelper, aStepBeyondReachIsClampedAndDoesNotCapture)
    {
        const auto plan = planCaptureStep(standing({ 0.0f, 250.0f }));
        ASSERT_TRUE(plan);
        EXPECT_TRUE(plan->mReachClamped);
        EXPECT_FALSE(plan->mCaptures);
        EXPECT_NEAR(distance(plan->mLanding, { -6.0f, 0.0f }), 45.0f, 1e-3f);
    }

    TEST(RagdollCoreBalanceHelper, aPushTowardTheStanceSideDoesNotCrossTheLegs)
    {
        // Pushed to the left while the right foot swings: the right foot may
        // not cross in front of the left one.
        const auto plan = planCaptureStep(standing({ -80.0f, 0.0f }));
        ASSERT_TRUE(plan);
        EXPECT_TRUE(plan->mWidthClamped);
        EXPECT_GE(plan->mLanding.mX - (-6.0f), 8.0f - 1e-4f);
        EXPECT_FALSE(plan->mCaptures);
    }

    TEST(RagdollCoreBalanceHelper, keepsTheFootfallStepWhileItIsCloseEnough)
    {
        auto input = standing({ 0.0f, 60.0f });
        const auto free = planCaptureStep(input);
        ASSERT_TRUE(free);
        const BalanceVec2 preferred{ 6.0f, free->mPredictedCapturePoint.mY - 3.0f };
        input.mPreferredLanding = preferred;
        input.mPreferredTolerance = 6.0f;
        const auto kept = planCaptureStep(input);
        ASSERT_TRUE(kept);
        EXPECT_TRUE(kept->mKeptPreferred);
        EXPECT_FLOAT_EQ(kept->mLanding.mX, preferred.mX);
        EXPECT_FLOAT_EQ(kept->mLanding.mY, preferred.mY);

        // Far from it: the landing moves toward the capture point only by
        // the excess over the tolerance.
        input.mPreferredLanding = BalanceVec2{ 6.0f, free->mPredictedCapturePoint.mY - 20.0f };
        const auto moved = planCaptureStep(input);
        ASSERT_TRUE(moved);
        EXPECT_FALSE(moved->mKeptPreferred);
        EXPECT_NEAR(distance(moved->mLanding, moved->mPredictedCapturePoint), 6.0f, 0.05f);
    }

    // Forward push speeds from a stand, with the step count each needs.
    std::vector<std::pair<float, RecoveryPlan>> sweep()
    {
        std::vector<std::pair<float, RecoveryPlan>> result;
        for (float speed = 10.0f; speed <= 400.0f; speed += 5.0f)
        {
            const auto plan = planRecoverySteps(standing({ 0.0f, speed }), 3);
            EXPECT_TRUE(plan) << speed;
            if (plan)
                result.emplace_back(speed, *plan);
        }
        return result;
    }

    TEST(RagdollCoreBalanceHelper, recoveryStepCountGrowsWithThePush)
    {
        std::size_t previous = 1;
        bool failed = false;
        std::array<bool, 4> seen{};
        for (const auto& [speed, plan] : sweep())
        {
            if (!plan.mCaptured)
            {
                failed = true;
                EXPECT_EQ(plan.mCount, 3u) << speed;
                continue;
            }
            // Once a push is too strong for three steps, no stronger one is caught.
            EXPECT_FALSE(failed) << speed;
            EXPECT_GE(plan.mCount, previous) << speed;
            previous = plan.mCount;
            seen[plan.mCount] = true;
        }
        EXPECT_TRUE(seen[1] && seen[2] && seen[3]);
        EXPECT_TRUE(failed);
    }

    TEST(RagdollCoreBalanceHelper, recoveryStepsAlternateLegsAndVaryInLength)
    {
        std::optional<RecoveryPlan> three;
        for (const auto& [speed, plan] : sweep())
            if (plan.mCaptured && plan.mCount == 3)
            {
                three = plan;
                break;
            }
        ASSERT_TRUE(three);
        for (std::size_t i = 1; i < three->mCount; ++i)
            EXPECT_NE(three->mSteps[i].mIsLeft, three->mSteps[i - 1].mIsLeft);
        // The last step only covers what is left, so it is not a copy of the
        // reach-limited ones before it; its swing is shorter too.
        EXPECT_GT(std::abs(three->mSteps[0].mLength - three->mSteps[2].mLength), 1.0f);
        EXPECT_NE(three->mSteps[0].mSwingTime, three->mSteps[2].mSwingTime);
        EXPECT_TRUE(three->mSteps[2].mCaptures);
        EXPECT_FALSE(three->mSteps[0].mCaptures);
    }

    TEST(RagdollCoreBalanceHelper, aSidePushTowardTheStanceFootIsCaughtByTheOtherLeg)
    {
        const auto plan = planRecoverySteps(standing({ -45.0f, 0.0f }), 3);
        ASSERT_TRUE(plan);
        ASSERT_TRUE(plan->mCaptured);
        ASSERT_EQ(plan->mCount, 2u);
        // The first (right) step stays right of the left foot; the left foot
        // then steps out to the left and catches the body.
        EXPECT_FALSE(plan->mSteps[0].mIsLeft);
        EXPECT_GE(plan->mSteps[0].mLanding.mX, 2.0f - 1e-4f);
        EXPECT_TRUE(plan->mSteps[1].mIsLeft);
        EXPECT_LT(plan->mSteps[1].mLanding.mX, -6.0f);
    }

    TEST(RagdollCoreBalanceHelper, aSidePushTowardTheStanceFootCanCrossOver)
    {
        // Pushed left while the right foot swings: the ordinary step stays
        // right of the left foot and cannot catch; the crossover lands left
        // of it, in front, with the feet passing each other.
        auto input = standing({ -80.0f, 0.0f });
        const auto ordinary = planCaptureStep(input);
        input.mLimits.mCrossoverClearance = 7.0f;
        const auto crossed = planCaptureStep(input);
        ASSERT_TRUE(ordinary && crossed);
        EXPECT_FALSE(ordinary->mCrossover);
        EXPECT_TRUE(crossed->mCrossover);
        EXPECT_LT(crossed->mLanding.mX, -6.0f);
        // In front of the stance foot and clear of it.
        EXPECT_GE(crossed->mLanding.mY, 3.5f - 1e-3f);
        EXPECT_GE(distance(crossed->mLanding, input.mStanceFoot), 7.0f - 1e-3f);
        EXPECT_LE(distance(crossed->mLanding, input.mStanceFoot), 45.0f + 1e-3f);
        EXPECT_LT(crossed->mResidual + 5.0f, ordinary->mResidual);
    }

    TEST(RagdollCoreBalanceHelper, aBackwardSidePushCrossesBehind)
    {
        auto input = standing({ -70.0f, -40.0f });
        input.mLimits.mCrossoverClearance = 7.0f;
        const auto plan = planCaptureStep(input);
        ASSERT_TRUE(plan);
        ASSERT_TRUE(plan->mCrossover);
        EXPECT_LE(plan->mLanding.mY, -3.5f + 1e-3f);
    }

    TEST(RagdollCoreBalanceHelper, noCrossoverWhileTheOrdinaryStepCaptures)
    {
        auto input = standing({ 0.0f, 60.0f });
        input.mLimits.mCrossoverClearance = 7.0f;
        const auto plan = planCaptureStep(input);
        ASSERT_TRUE(plan);
        EXPECT_TRUE(plan->mCaptures);
        EXPECT_FALSE(plan->mCrossover);
    }

    TEST(RagdollCoreBalanceHelper, crossoversCatchStrongerSidePushes)
    {
        // Side pushes toward the stance foot: the strongest one each planner
        // catches in at most three steps.
        const auto strongest = [](float clearance) {
            float best = 0.0f;
            for (float speed = 10.0f; speed <= 300.0f; speed += 5.0f)
            {
                auto input = standing({ -speed, 0.0f });
                input.mLimits.mCrossoverClearance = clearance;
                const auto plan = planRecoverySteps(input, 3);
                if (plan && plan->mCaptured)
                    best = speed;
            }
            return best;
        };
        const float ordinary = strongest(0.0f);
        const float crossover = strongest(7.0f);
        EXPECT_GT(crossover, ordinary + 10.0f) << ordinary << " " << crossover;
    }

    TEST(RagdollCoreBalanceHelper, theBrakeMarginPutsTheCapturePointBetweenTheFeet)
    {
        // A push the first step catches with room left in the leg's reach.
        auto input = standing({ 0.0f, 40.0f });
        const auto plain = planCaptureStep(input);
        input.mLimits.mBrakeMargin = 6.0f;
        const auto braked = planCaptureStep(input);
        ASSERT_TRUE(plain && braked);
        ASSERT_TRUE(plain->mCaptures && braked->mCaptures);
        // Further along the fall, and the capture point now on the segment.
        EXPECT_GT(braked->mLanding.mY, plain->mLanding.mY + 3.0f);
        EXPECT_LT(braked->mResidual, plain->mResidual + 1e-4f);
        EXPECT_LE(braked->mResidual, 2.0f);
        EXPECT_LE(distance(braked->mLanding, input.mStanceFoot), 45.0f + 1e-3f);
    }

    TEST(RagdollCoreBalanceHelper, theBrakeMarginLeavesAStepThatCannotCaptureAlone)
    {
        auto input = standing({ 0.0f, 250.0f });
        const auto plain = planCaptureStep(input);
        input.mLimits.mBrakeMargin = 6.0f;
        const auto braked = planCaptureStep(input);
        ASSERT_TRUE(plain && braked);
        EXPECT_FLOAT_EQ(braked->mLanding.mX, plain->mLanding.mX);
        EXPECT_FLOAT_EQ(braked->mLanding.mY, plain->mLanding.mY);
    }

    RecoveryEndInput endInput(BalanceVec2 capture, BalanceVec2 velocity, float held)
    {
        RecoveryEndInput input;
        input.mCapturePoint = capture;
        input.mVelocity = velocity;
        input.mLeftFoot = { -6.0f, 0.0f };
        input.mRightFoot = { 6.0f, 20.0f };
        input.mHeld = held;
        input.mDt = 0.05f;
        input.mLimits = { 5.0f, 2.0f, 15.0f, 0.2f };
        return input;
    }

    TEST(RagdollCoreBalanceHelper, aRecoveryEndsWhenTheCapturePointStaysInsideAndSlow)
    {
        // Between the feet, slow: holds, and ends once held long enough.
        const auto early = evaluateRecoveryEnd(endInput({ 0.0f, 10.0f }, { 5.0f, 5.0f }, 0.0f));
        ASSERT_TRUE(early);
        EXPECT_FALSE(early->mCaught);
        EXPECT_NEAR(early->mHeld, 0.05f, 1e-6f);
        const auto done = evaluateRecoveryEnd(endInput({ 0.0f, 10.0f }, { 5.0f, 5.0f }, 0.16f));
        ASSERT_TRUE(done);
        EXPECT_TRUE(done->mCaught);
        // Too fast, or near the support's edge: the hold starts over.
        const auto fast = evaluateRecoveryEnd(endInput({ 0.0f, 10.0f }, { 20.0f, 0.0f }, 0.16f));
        ASSERT_TRUE(fast);
        EXPECT_FALSE(fast->mCaught);
        EXPECT_EQ(fast->mHeld, 0.0f);
        const auto edge = evaluateRecoveryEnd(endInput({ -10.0f, 0.0f }, { 0.0f, 0.0f }, 0.16f));
        ASSERT_TRUE(edge);
        EXPECT_FALSE(edge->mCaught);
        EXPECT_NEAR(edge->mInside, 1.0f, 1e-4f);
        // No foot down: never caught.
        auto air = endInput({ 0.0f, 10.0f }, { 0.0f, 0.0f }, 1.0f);
        air.mLeftSupported = air.mRightSupported = false;
        const auto airborne = evaluateRecoveryEnd(air);
        ASSERT_TRUE(airborne);
        EXPECT_FALSE(airborne->mCaught);
    }

    BalanceStageInput stageInput(BalanceVec2 capture, BalanceStage previous = BalanceStage::Ankle)
    {
        BalanceStageInput input;
        input.mCapturePoint = capture;
        input.mLeftFoot = { -6.0f, 0.0f };
        input.mRightFoot = { 6.0f, 0.0f };
        input.mPrevious = previous;
        input.mLimits.mFootRadius = 5.0f;
        input.mLimits.mHipReach = 6.0f;
        input.mLimits.mHysteresis = 1.0f;
        return input;
    }

    TEST(RagdollCoreBalanceHelper, stagesRiseFromAnkleToHipToStep)
    {
        const auto ankle = classifyBalanceStage(stageInput({ 0.0f, 3.0f }));
        const auto hip = classifyBalanceStage(stageInput({ 0.0f, 8.0f }));
        const auto step = classifyBalanceStage(stageInput({ 0.0f, 14.0f }));
        ASSERT_TRUE(ankle && hip && step);
        EXPECT_EQ(ankle->mStage, BalanceStage::Ankle);
        EXPECT_EQ(ankle->mHipWeight, 0.0f);
        EXPECT_EQ(hip->mStage, BalanceStage::HipAndArms);
        EXPECT_GT(hip->mHipWeight, 0.0f);
        EXPECT_NEAR(hip->mFallDirection.mY, 1.0f, 1e-6f);
        EXPECT_EQ(step->mStage, BalanceStage::Step);
        EXPECT_EQ(step->mHipWeight, 1.0f);
        // Beside a foot, the distance is to that foot, not the midline.
        const auto side = classifyBalanceStage(stageInput({ 14.0f, 0.0f }));
        ASSERT_TRUE(side);
        EXPECT_NEAR(side->mExcess, 3.0f, 1e-5f);
    }

    TEST(RagdollCoreBalanceHelper, stagesHoldWithinTheHysteresis)
    {
        // Just below the hip band's top: stays stepping, but an ankle-held
        // body would only enter the hip band.
        const auto held = classifyBalanceStage(stageInput({ 0.0f, 10.5f }, BalanceStage::Step));
        ASSERT_TRUE(held);
        EXPECT_EQ(held->mStage, BalanceStage::Step);
        const auto entered = classifyBalanceStage(stageInput({ 0.0f, 10.5f }, BalanceStage::Ankle));
        ASSERT_TRUE(entered);
        EXPECT_EQ(entered->mStage, BalanceStage::HipAndArms);
        const auto hipHeld = classifyBalanceStage(stageInput({ 0.0f, 4.5f }, BalanceStage::HipAndArms));
        ASSERT_TRUE(hipHeld);
        EXPECT_EQ(hipHeld->mStage, BalanceStage::HipAndArms);
    }

    TEST(RagdollCoreBalanceHelper, theHipBandStepsWhenItHasNotCaughtTheBodyInTime)
    {
        auto input = stageInput({ 0.0f, 8.0f }, BalanceStage::HipAndArms);
        input.mLimits.mMaximumHipTime = 0.3f;
        input.mTimeInPrevious = 0.2f;
        const auto early = classifyBalanceStage(input);
        ASSERT_TRUE(early);
        EXPECT_EQ(early->mStage, BalanceStage::HipAndArms);
        input.mTimeInPrevious = 0.3f;
        const auto late = classifyBalanceStage(input);
        ASSERT_TRUE(late);
        EXPECT_EQ(late->mStage, BalanceStage::Step);
        // Back over the feet in time: no step, whatever the time.
        input.mCapturePoint = { 0.0f, 4.5f };
        const auto inside = classifyBalanceStage(input);
        ASSERT_TRUE(inside);
        EXPECT_EQ(inside->mStage, BalanceStage::HipAndArms);
    }

    TEST(RagdollCoreBalanceHelper, singleSupportAndAirborneStages)
    {
        auto input = stageInput({ 6.0f, 8.0f });
        input.mLeftSupported = false;
        const auto right = classifyBalanceStage(input);
        ASSERT_TRUE(right);
        EXPECT_NEAR(right->mExcess, 3.0f, 1e-5f);
        input.mRightSupported = false;
        const auto air = classifyBalanceStage(input);
        ASSERT_TRUE(air);
        EXPECT_EQ(air->mStage, BalanceStage::Step);
    }
}
