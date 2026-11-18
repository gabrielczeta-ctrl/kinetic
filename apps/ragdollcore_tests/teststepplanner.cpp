#include <ragdollcore/stepplanner.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <optional>

namespace
{
    using namespace RagdollCore;

    // The balance helper tests' body: a 90-unit pendulum under 900 u/s^2,
    // left foot stance at x = -6, right foot swinging from x = +6, facing +y
    // (right is +x).
    constexpr float sHeight = 90.0f;
    constexpr float sGravity = 900.0f;
    const float sOmega = std::sqrt(sGravity / sHeight);

    float distance(BalanceVec2 a, BalanceVec2 b) { return std::hypot(a.mX - b.mX, a.mY - b.mY); }

    float segmentDistance(BalanceVec2 p, BalanceVec2 a, BalanceVec2 b)
    {
        const float abx = b.mX - a.mX, aby = b.mY - a.mY;
        const float len2 = abx * abx + aby * aby;
        const float t = len2 > 0.0f ? std::clamp(((p.mX - a.mX) * abx + (p.mY - a.mY) * aby) / len2, 0.0f, 1.0f) : 0.0f;
        return std::hypot(p.mX - (a.mX + abx * t), p.mY - (a.mY + aby * t));
    }

    CaptureStepLimits catchLimits()
    {
        CaptureStepLimits limits;
        limits.mFootRadius = 5.0f;
        limits.mMaximumReach = 45.0f;
        limits.mMinimumWidth = 8.0f;
        limits.mMinimumSwingTime = 0.2f;
        limits.mMaximumSwingTime = 0.35f;
        limits.mSwingFootSpeed = 400.0f;
        return limits;
    }

    StrideInput standing(BalanceVec2 velocity, BalanceVec2 desired = {})
    {
        StrideInput input;
        input.mState.mCenterOfMass = { 0.0f, 0.0f };
        input.mState.mVelocity = velocity;
        input.mState.mMass = 75.0f;
        input.mState.mHeight = sHeight;
        input.mState.mGravity = sGravity;
        input.mStanceFoot = { -6.0f, 0.0f };
        input.mSwingFoot = { 6.0f, 0.0f };
        input.mSwingIsLeft = false;
        input.mForward = { 0.0f, 1.0f };
        input.mDesiredVelocity = desired;
        input.mLimits.mNominalStepTime = 0.3f;
        input.mLimits.mStanceWidth = 12.0f;
        input.mLimits.mMinimumSwingTime = 0.2f;
        input.mLimits.mMaximumSwingTime = 0.45f;
        input.mLimits.mMaximumReach = 45.0f;
        input.mLimits.mMinimumWidth = 8.0f;
        input.mLimits.mFootRadius = 5.0f;
        input.mLimits.mCatch = catchLimits();
        return input;
    }

    // The body in the steady gait at `speed` along +y: the DCM where the
    // gait's own step and offset meet (p + a tau = u0 + b0).
    StrideInput steady(float speed)
    {
        StrideInput input = standing({ 0.0f, speed }, { 0.0f, speed });
        const float tau = std::exp(sOmega * input.mLimits.mNominalStepTime);
        const float step = speed * input.mLimits.mNominalStepTime;
        const BalanceVec2 landing{ input.mStanceFoot.mX + input.mLimits.mStanceWidth, step };
        const BalanceVec2 offset{ -input.mLimits.mStanceWidth / (1.0f + tau), step / (tau - 1.0f) };
        // xi0 = p + (u0 + b0 - p) / tau, and the centre of mass behind it.
        const BalanceVec2 xi{ input.mStanceFoot.mX + (landing.mX + offset.mX - input.mStanceFoot.mX) / tau,
            input.mStanceFoot.mY + (landing.mY + offset.mY - input.mStanceFoot.mY) / tau };
        input.mState.mVelocity = { 0.0f, speed };
        input.mState.mCenterOfMass = { xi.mX, xi.mY - speed / sOmega };
        return input;
    }

    TEST(RagdollCoreStepPlanner, dcmIsComPlusVelocityOverOmega)
    {
        StrideState state;
        state.mCenterOfMass = { 1.0f, 2.0f };
        state.mVelocity = { 10.0f, -20.0f };
        state.mHeight = sHeight;
        state.mGravity = sGravity;
        const auto dcm = strideDcm(state, { 0.0f, 0.0f });
        ASSERT_TRUE(dcm);
        EXPECT_NEAR(dcm->mX, 1.0f + 10.0f / sOmega, 1e-4f);
        EXPECT_NEAR(dcm->mY, 2.0f - 20.0f / sOmega, 1e-4f);
    }

    TEST(RagdollCoreStepPlanner, angularMomentumGivesTheSameDcmForAPointMassAndMoreWithSwingingLegs)
    {
        StrideState state;
        state.mCenterOfMass = { 0.0f, 0.0f };
        state.mVelocity = { 0.0f, 60.0f };
        state.mMass = 75.0f;
        state.mHeight = sHeight;
        state.mGravity = sGravity;
        const auto byVelocity = strideDcm(state, { -6.0f, 0.0f });
        // Lx = -m h vy for the point mass.
        state.mAngularMomentum = BalanceVec2{ -75.0f * sHeight * 60.0f, 0.0f };
        const auto byMomentum = strideDcm(state, { -6.0f, 0.0f });
        ASSERT_TRUE(byVelocity && byMomentum);
        EXPECT_NEAR(byVelocity->mY, byMomentum->mY, 1e-3f);
        // A leg swinging forward adds momentum the pelvis velocity misses.
        state.mAngularMomentum = BalanceVec2{ -75.0f * sHeight * 75.0f, 0.0f };
        const auto withLegs = strideDcm(state, { -6.0f, 0.0f });
        ASSERT_TRUE(withLegs);
        EXPECT_GT(withLegs->mY, byVelocity->mY + 4.0f);
    }

    TEST(RagdollCoreStepPlanner, rejectsInvalidInput)
    {
        auto input = standing({ 0.0f, 50.0f });
        input.mState.mHeight = 0.0f;
        EXPECT_FALSE(planStride(input));
        input = standing({ 0.0f, 50.0f });
        input.mForward = { 0.0f, 0.0f };
        EXPECT_FALSE(planStride(input));
        input = standing({ 0.0f, 50.0f });
        input.mState.mVelocity.mX = std::nanf("");
        EXPECT_FALSE(planStride(input));
        input = standing({ 0.0f, 50.0f });
        input.mState.mAngularMomentum = BalanceVec2{ 1.0f, 1.0f };
        input.mState.mMass = 0.0f;
        EXPECT_FALSE(planStride(input));
    }

    // balancehelper: forwardPushLandsOnTheCapturePointAtTouchdown.
    TEST(RagdollCoreStepPlanner, aForwardPushIsCaughtLikeTheCaptureStep)
    {
        const auto input = standing({ 0.0f, 50.0f });
        const auto plan = planStride(input);
        ASSERT_TRUE(plan);
        EXPECT_FALSE(plan->mRecovery);
        EXPECT_GT(plan->mLanding.mY, 3.0f);
        // Standing still is the gait: the DCM ends between the feet.
        EXPECT_LT(segmentDistance(plan->mPredictedDcm, input.mStanceFoot, plan->mLanding), input.mLimits.mFootRadius);
        // Near the capture step planner's own landing.
        CaptureStepInput reference;
        reference.mState = { input.mState.mCenterOfMass, input.mState.mVelocity, sHeight, sGravity };
        reference.mStanceFoot = input.mStanceFoot;
        reference.mSwingFoot = input.mSwingFoot;
        reference.mBodyForward = input.mForward;
        reference.mLimits = catchLimits();
        const auto capture = planCaptureStep(reference);
        ASSERT_TRUE(capture && capture->mCaptures);
        EXPECT_LT(std::abs(plan->mLanding.mY - capture->mLanding.mY), 8.0f);
    }

    // balancehelper: aStepBeyondReachIsClampedAndDoesNotCapture;
    // recoveryStepCountGrowsWithThePush.
    TEST(RagdollCoreStepPlanner, aPushOneStepCannotHoldGoesToTheCatchSteps)
    {
        // The balance helper's own sweep (10-400 u/s): what it catches, the
        // planner never gives up on (one adapted step often holds a push the
        // helper needs 2-3 steps for); when the planner hands over, the catch
        // is the helper's, with its verdict and step count.
        bool handedOver = false, gaveUp = false;
        for (float speed = 10.0f; speed <= 400.0f; speed += 5.0f)
        {
            const auto input = standing({ 0.0f, speed });
            const auto plan = planStride(input);
            ASSERT_TRUE(plan) << speed;
            CaptureStepInput reference;
            reference.mState = { input.mState.mCenterOfMass, input.mState.mVelocity, sHeight, sGravity };
            reference.mStanceFoot = input.mStanceFoot;
            reference.mSwingFoot = input.mSwingFoot;
            reference.mBodyForward = input.mForward;
            reference.mLimits = catchLimits();
            const auto recovery = planRecoverySteps(reference, 3);
            ASSERT_TRUE(recovery) << speed;
            if (recovery->mCaptured)
                EXPECT_FALSE(plan->mGiveUp) << speed;
            if (!plan->mRecovery)
                continue;
            handedOver = true;
            EXPECT_EQ(plan->mGiveUp, !recovery->mCaptured) << speed;
            EXPECT_EQ(plan->mRecoverySteps, recovery->mCount) << speed;
            gaveUp = gaveUp || plan->mGiveUp;
        }
        EXPECT_TRUE(handedOver);
        EXPECT_TRUE(gaveUp);
    }

    // recoverycontrol: hoverReleasesWhenNoStepCatches.
    TEST(RagdollCoreStepPlanner, givesUpWhenNoCatchHolds)
    {
        const auto plan = planStride(standing({ 0.0f, 900.0f }));
        ASSERT_TRUE(plan);
        EXPECT_TRUE(plan->mRecovery);
        EXPECT_TRUE(plan->mGiveUp);
    }

    // balancehelper: aPushTowardTheStanceSideDoesNotCrossTheLegs.
    TEST(RagdollCoreStepPlanner, aPushTowardTheStanceSideNeverCrossesTheFeet)
    {
        const auto input = standing({ -70.0f, 0.0f });
        const auto plan = planStride(input);
        ASSERT_TRUE(plan);
        // Right foot swinging: it lands on its own (+x) side of the stance foot.
        EXPECT_GE(plan->mLanding.mX - input.mStanceFoot.mX, input.mLimits.mMinimumWidth - 1e-3f);
        if (!plan->mRecovery)
            EXPECT_TRUE(plan->mWidthActive);
        // The same push with the left foot swinging: it goes left of the stance.
        auto left = input;
        left.mSwingIsLeft = true;
        left.mStanceFoot = { 6.0f, 0.0f };
        left.mSwingFoot = { -6.0f, 0.0f };
        left.mLimits.mCatch.mCrossoverClearance = 0.0f;
        const auto leftPlan = planStride(left);
        ASSERT_TRUE(leftPlan);
        EXPECT_LE(leftPlan->mLanding.mX - left.mStanceFoot.mX, -left.mLimits.mMinimumWidth + 1e-3f);
    }

    // recoverycontrol: movingCatchLeavesASteadyGaitAlone.
    TEST(RagdollCoreStepPlanner, theSteadyGaitKeepsItsOwnStep)
    {
        const auto input = steady(120.0f);
        const auto plan = planStride(input);
        ASSERT_TRUE(plan);
        EXPECT_FALSE(plan->mRecovery);
        EXPECT_LT(distance(plan->mLanding, plan->mNominalLanding), 0.5f);
        EXPECT_NEAR(plan->mSwingTime, input.mLimits.mNominalStepTime, 0.01f);
        EXPECT_LT(distance(plan->mOffset, plan->mNominalOffset), 0.5f);
    }

    // recoverycontrol: movingCatchMovesTheLandingByTheVelocityError.
    TEST(RagdollCoreStepPlanner, aPushWhileWalkingMovesTheLandingAndHastensIt)
    {
        const auto calm = planStride(steady(120.0f));
        auto pushed = steady(120.0f);
        pushed.mState.mVelocity.mY += 40.0f;
        const auto plan = planStride(pushed);
        ASSERT_TRUE(calm && plan);
        EXPECT_FALSE(plan->mRecovery);
        EXPECT_GT(plan->mLanding.mY, calm->mLanding.mY + 2.0f);
        EXPECT_LT(plan->mSwingTime, calm->mSwingTime - 0.005f);
    }

    TEST(RagdollCoreStepPlanner, theTimeAdaptsMidSwingWithinItsBounds)
    {
        auto input = steady(120.0f);
        input.mElapsed = 0.15f;
        input.mState.mVelocity.mY += 60.0f;
        const auto plan = planStride(input);
        ASSERT_TRUE(plan);
        EXPECT_GE(plan->mSwingTime, input.mLimits.mMinimumSwingTime - 1e-3f);
        EXPECT_LE(plan->mSwingTime, input.mLimits.mMaximumSwingTime + 1e-3f);
        EXPECT_GT(plan->mRemainingTime, 0.0f);
    }

    TEST(RagdollCoreStepPlanner, aStartFromRestStepsForwardButShort)
    {
        const auto input = standing({ 0.0f, 0.0f }, { 0.0f, 120.0f });
        const auto plan = planStride(input);
        ASSERT_TRUE(plan);
        EXPECT_FALSE(plan->mRecovery);
        // Forward, on its own side, and shorter than the steady gait's step:
        // the body has no momentum yet.
        EXPECT_GT(plan->mLanding.mY, 0.5f);
        EXPECT_LT(plan->mLanding.mY, plan->mNominalLanding.mY);
        EXPECT_GE(plan->mLanding.mX - input.mStanceFoot.mX, input.mLimits.mMinimumWidth - 1e-3f);
    }

    TEST(RagdollCoreStepPlanner, aStopFromAWalkCatchesTheBody)
    {
        auto input = steady(120.0f);
        input.mDesiredVelocity = { 0.0f, 0.0f };
        const auto plan = planStride(input);
        ASSERT_TRUE(plan);
        EXPECT_FALSE(plan->mGiveUp);
        // The predicted DCM ends inside the new support: the body stops.
        EXPECT_LT(segmentDistance(plan->mPredictedDcm, input.mStanceFoot, plan->mLanding),
            input.mLimits.mFootRadius + 2.0f);
    }

    TEST(RagdollCoreStepPlanner, aReversalBrakesHarderThanTheGait)
    {
        const auto calm = planStride(steady(120.0f));
        auto input = steady(120.0f);
        input.mDesiredVelocity = { 0.0f, -120.0f };
        const auto plan = planStride(input);
        ASSERT_TRUE(calm && plan);
        EXPECT_FALSE(plan->mGiveUp);
        // One step cannot turn 120 u/s round within its reach; the first
        // lands farther out under the DCM than the gait's (a brake), so less
        // of the forward run is left for the next.
        EXPECT_GT(plan->mLanding.mY, calm->mLanding.mY + 5.0f);
        EXPECT_LT(plan->mOffset.mY, calm->mOffset.mY);
    }

    // recoverycontrol: catchStaysOffWhenBalanced.
    TEST(RagdollCoreStepPlanner, standingBalancedNeedsNoStep)
    {
        StrideStandInput stand;
        stand.mState.mCenterOfMass = { 0.0f, 0.0f };
        stand.mState.mHeight = sHeight;
        stand.mState.mGravity = sGravity;
        stand.mLeftFoot = { -6.0f, 0.0f };
        stand.mRightFoot = { 6.0f, 0.0f };
        stand.mFootRadius = 5.0f;
        EXPECT_EQ(strideNeedsStep(stand), std::optional<bool>(false));
        stand.mState.mVelocity = { 0.0f, 40.0f };
        EXPECT_EQ(strideNeedsStep(stand), std::optional<bool>(true));
        stand.mMargin = 20.0f;
        EXPECT_EQ(strideNeedsStep(stand), std::optional<bool>(false));
    }

    TEST(RagdollCoreStepPlanner, angularMomentumAboutTheContactAddsTheOrbitOfTheCentreOfMass)
    {
        // 75 kg at 90 units over the contact moving +y at 60: Lx = -m h vy.
        const BalanceVec2 l
            = strideAngularMomentumAbout({ 0.0f, 0.0f, 90.0f }, { 0.0f, 75.0f * 60.0f, 0.0f }, { 0.0f, 0.0f, 0.0f },
                { 0.0f, 0.0f, 0.0f });
        EXPECT_NEAR(l.mX, -75.0f * 90.0f * 60.0f, 1e-2f);
        EXPECT_NEAR(l.mY, 0.0f, 1e-3f);
        // Read back by strideDcm it is the velocity's DCM.
        StrideState state;
        state.mCenterOfMass = { 0.0f, 0.0f };
        state.mVelocity = { 0.0f, 60.0f };
        state.mMass = 75.0f;
        state.mHeight = 90.0f;
        state.mGravity = sGravity;
        const auto byVelocity = strideDcm(state, { 0.0f, 0.0f });
        state.mAngularMomentum = l;
        const auto byMomentum = strideDcm(state, { 0.0f, 0.0f });
        ASSERT_TRUE(byVelocity && byMomentum);
        EXPECT_NEAR(byMomentum->mY, byVelocity->mY, 1e-3f);
        // Spin about the centre of mass (a swinging leg) adds to it.
        const BalanceVec2 spun
            = strideAngularMomentumAbout({ 0.0f, 0.0f, 90.0f }, { 0.0f, 75.0f * 60.0f, 0.0f }, { -5000.0f, 0.0f, 0.0f },
                { 0.0f, 0.0f, 0.0f });
        EXPECT_NEAR(spun.mX, l.mX - 5000.0f, 1e-2f);
    }

    TEST(RagdollCoreStepPlanner, theGaitTableSetsTheNominalStep)
    {
        StrideLimits limits;
        // A walk row: 0.4 s swings at duty 0.6: a 1.0 s cycle, 0.5 s steps.
        const StrideLimits walk = strideLimitsFromGait(0.4f, 0.6f, 6.0f, limits);
        EXPECT_NEAR(walk.mNominalStepTime, 0.5f, 1e-5f);
        EXPECT_NEAR(walk.mStanceWidth, 12.0f, 1e-5f);
        EXPECT_EQ(walk.mMaximumReach, limits.mMaximumReach);
        // Nonsense keeps the defaults.
        const StrideLimits kept = strideLimitsFromGait(-1.0f, 1.0f, -3.0f, limits);
        EXPECT_EQ(kept.mNominalStepTime, limits.mNominalStepTime);
        EXPECT_EQ(kept.mStanceWidth, limits.mStanceWidth);
    }

    TEST(RagdollCoreStepPlanner, guardFootworkMovesTheFootNearerTheTravelFirst)
    {
        // A guard facing +y, left foot forward (the lead), right foot back.
        const BalanceVec2 left{ -6.0f, 10.0f }, right{ 6.0f, -10.0f };
        // Advancing: the lead (left) foot first; retreating: the rear (right).
        EXPECT_TRUE(strideFirstFootIsLeft(left, right, { 0.0f, 1.0f }, true));
        EXPECT_FALSE(strideFirstFootIsLeft(left, right, { 0.0f, -1.0f }, true));
        // Sidestepping right: the right foot first; left: the left foot.
        EXPECT_FALSE(strideFirstFootIsLeft(left, right, { 1.0f, 0.0f }, true));
        EXPECT_TRUE(strideFirstFootIsLeft(left, right, { -1.0f, 0.0f }, true));
        // Walking from a stand steps the trailing foot out first.
        EXPECT_FALSE(strideFirstFootIsLeft(left, right, { 0.0f, 1.0f }, false));
        EXPECT_TRUE(strideFirstFootIsLeft(left, right, { 0.0f, -1.0f }, false));
    }

    TEST(RagdollCoreStepPlanner, guardFootworkNeverCrossesWhenTheNearFootSidesteps)
    {
        // Sidestepping right in a guard: the right foot (nearer the travel)
        // swings and keeps to its own side of the left stance foot; then the
        // left foot follows and stays left of the right one.
        auto input = standing({ 30.0f, 0.0f }, { 60.0f, 0.0f });
        const auto first = planStride(input);
        ASSERT_TRUE(first);
        EXPECT_GE(first->mLanding.mX - input.mStanceFoot.mX, input.mLimits.mMinimumWidth - 1e-3f);
        auto follow = input;
        follow.mSwingIsLeft = true;
        follow.mStanceFoot = first->mLanding;
        follow.mSwingFoot = input.mStanceFoot;
        const auto second = planStride(follow);
        ASSERT_TRUE(second);
        EXPECT_LE(second->mLanding.mX - follow.mStanceFoot.mX, -follow.mLimits.mMinimumWidth + 1e-3f);
    }

    TEST(RagdollCoreStepPlanner, isDeterministic)
    {
        const auto a = planStride(steady(150.0f));
        const auto b = planStride(steady(150.0f));
        ASSERT_TRUE(a && b);
        EXPECT_EQ(a->mLanding.mX, b->mLanding.mX);
        EXPECT_EQ(a->mLanding.mY, b->mLanding.mY);
        EXPECT_EQ(a->mSwingTime, b->mSwingTime);
    }
}
