#include <gtest/gtest.h>
#include <ragdollcore/limbclearance.hpp>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <array>
#include <limits>

using namespace RagdollCore;

TEST(LimbClearance, DistinguishesMarginFromPenetration)
{
    JPH::RefConst<JPH::Shape> shape = new JPH::CapsuleShape(10.0f, 3.5f);
    std::array<LimbShapePose, 1> a{{{shape.GetPtr()}}};
    auto b = a;
    b[0].mPosition = JPH::RVec3(8, 0, 0);
    const auto near = measureLimbClearance(a, b, 2);
    ASSERT_TRUE(near); ASSERT_TRUE(near->mMinimumReportedGap);
    EXPECT_NEAR(*near->mMinimumReportedGap, 1, 1e-4);
    EXPECT_FALSE(near->mClear);
    const auto far = measureLimbClearance(a, b, 0.5f);
    ASSERT_TRUE(far); EXPECT_TRUE(far->mClear); EXPECT_FALSE(far->mMinimumReportedGap);
    b[0].mPosition = JPH::RVec3(6, 0, 0);
    const auto overlap = measureLimbClearance(a, b, 0);
    ASSERT_TRUE(overlap); ASSERT_TRUE(overlap->mMinimumReportedGap);
    EXPECT_NEAR(*overlap->mMinimumReportedGap, -1, 1e-4);
}

TEST(LimbClearance, AccountsForShapeComAndLargeWorldOrigin)
{
    JPH::RefConst<JPH::Shape> sphere = new JPH::SphereShape(1);
    JPH::RefConst<JPH::Shape> offset = new JPH::RotatedTranslatedShape(
        JPH::Vec3(0, 4, 0), JPH::Quat::sIdentity(), sphere.GetPtr());
    std::array<LimbShapePose, 1> a{{{offset.GetPtr()}}};
    std::array<LimbShapePose, 1> b{{{sphere.GetPtr(), JPH::RVec3(3, 4, 0)}}};
    const auto before = measureLimbClearance(a, b, 2);
    ASSERT_TRUE(before); ASSERT_TRUE(before->mMinimumReportedGap);
    EXPECT_NEAR(*before->mMinimumReportedGap, 1, 1e-4);
    const auto rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.5f);
    const JPH::RVec3 translation(100000, -200000, 300000);
    for (auto* poses : {&a, &b})
        for (auto& pose : *poses)
        {
            pose.mPosition = translation + JPH::RVec3(rotation * JPH::Vec3(pose.mPosition));
            pose.mRotation = rotation;
        }
    const auto after = measureLimbClearance(a, b, 2);
    ASSERT_TRUE(after); ASSERT_TRUE(after->mMinimumReportedGap);
    EXPECT_NEAR(*after->mMinimumReportedGap, 1, 1e-4);
}

TEST(LimbClearance, TestsEveryPairAndRejectsInvalidInputs)
{
    JPH::RefConst<JPH::Shape> sphere = new JPH::SphereShape(1);
    std::array<LimbShapePose, 2> a{{{sphere.GetPtr()}, {sphere.GetPtr(), JPH::RVec3(10, 0, 0)}}};
    std::array<LimbShapePose, 2> b{{{sphere.GetPtr(), JPH::RVec3(20, 0, 0)},
        {sphere.GetPtr(), JPH::RVec3(11, 0, 0)}}};
    const auto result = measureLimbClearance(a, b, 0);
    ASSERT_TRUE(result); EXPECT_FALSE(result->mClear); EXPECT_EQ(result->mTestedPairs, 4u);
    EXPECT_EQ(result->mFirstIndex, 1u); EXPECT_EQ(result->mSecondIndex, 1u);
    EXPECT_FALSE(measureLimbClearance({}, b, 0));
    EXPECT_FALSE(measureLimbClearance(a, b, -1));
    b[0].mPosition = JPH::RVec3(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    EXPECT_FALSE(measureLimbClearance(a, b, 0));
    b[0] = {};
    EXPECT_FALSE(measureLimbClearance(a, b, 0));
}

TEST(LimbClearance, ReproducesRecordedTurnCalfContact)
{
    // A63 immediate120: scenario590 pre-solve = capture588. Fitted capsule
    // parameters from the captured bind skeleton; actual body-origin poses.
    // Independent A87 segment reconstruction and native manifold: -1.665196.
    JPH::RefConst<JPH::Shape> leftCapsule = new JPH::CapsuleShape(15.1783804f, 3.5f);
    JPH::RefConst<JPH::Shape> rightCapsule = new JPH::CapsuleShape(15.1783832f, 3.5f);
    JPH::RefConst<JPH::Shape> leftShape = new JPH::RotatedTranslatedShape(
        JPH::Vec3(18.6783804f, -8.3283e-8f, 1.719699e-6f),
        JPH::Quat(6.510258e-8f, 0, -0.707106783f, 0.707106780f), leftCapsule.GetPtr());
    JPH::RefConst<JPH::Shape> rightShape = new JPH::RotatedTranslatedShape(
        JPH::Vec3(18.6783832f, -1.208361e-6f, -1.494764e-8f),
        JPH::Quat(-5.658722e-10f, 0, -0.707106804f, 0.707106758f), rightCapsule.GetPtr());
    std::array<LimbShapePose, 1> left{{{leftShape.GetPtr(),
        JPH::RVec3(22.146470448871405, 104.36011715988255, 42.81901027867447),
        JPH::Quat(-0.2790172696f, 0.6606531143f, -0.1270663589f, 0.6852306128f)}}};
    std::array<LimbShapePose, 1> right{{{rightShape.GetPtr(),
        JPH::RVec3(31.062419088826033, 91.72785512635451, 39.58401720966113),
        JPH::Quat(-0.1814088076f, 0.7122429609f, 0.0147549659f, 0.6779256463f)}}};
    const auto result = measureLimbClearance(left, right, 0);
    ASSERT_TRUE(result); ASSERT_TRUE(result->mMinimumReportedGap);
    EXPECT_FALSE(result->mClear);
    EXPECT_NEAR(*result->mMinimumReportedGap, -1.66519582f, 1e-3f);
}

TEST(LimbClearance, UsesOrientedFootExtentRatherThanFootOrigin)
{
    JPH::RefConst<JPH::Shape> box = new JPH::BoxShape(JPH::Vec3(4, 8, 2.5f));
    JPH::RefConst<JPH::Shape> sphere = new JPH::SphereShape(3.5f);
    const auto rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.4f);
    std::array<LimbShapePose, 1> a{{{box.GetPtr(), JPH::RVec3::sZero(), rotation}}};
    std::array<LimbShapePose, 1> b{{{sphere.GetPtr(),
        JPH::RVec3(rotation * JPH::Vec3(0, 0, 7))}}};
    const auto gap = measureLimbClearance(a, b, 2);
    ASSERT_TRUE(gap); ASSERT_TRUE(gap->mMinimumReportedGap);
    EXPECT_NEAR(*gap->mMinimumReportedGap, 1, 1e-3);
    b[0].mPosition = JPH::RVec3(rotation * JPH::Vec3(0, 0, 5));
    const auto overlap = measureLimbClearance(a, b, 0);
    ASSERT_TRUE(overlap); ASSERT_TRUE(overlap->mMinimumReportedGap);
    EXPECT_NEAR(*overlap->mMinimumReportedGap, -1, 1e-3);
}

TEST(LimbClearance, FindsBoundedJointValidAlternativeAndPreservesClearRequest)
{
    JPH::RefConst<JPH::Shape> sphere = new JPH::SphereShape(1);
    LimbPoseSearchInput input;
    input.mShapes = { sphere.GetPtr(), sphere.GetPtr(), sphere.GetPtr() };
    input.mHipPosition = JPH::RVec3(0, 0, 20);
    input.mLeg.mUpperOffset = JPH::Vec3(0, 0, -10);
    input.mLeg.mLowerOffset = JPH::Vec3(0, 0, -10);
    input.mLeg.mMinimumAngle = 0;
    input.mLeg.mMaximumAngle = 1.2f;
    input.mHipLimits.mNormalHalfConeAngle = input.mHipLimits.mPlaneHalfConeAngle = 0.8f;
    input.mHipLimits.mTwistMinimum = -0.8f;
    input.mHipLimits.mTwistMaximum = 0.8f;
    input.mAnkleLimits = input.mHipLimits;
    std::array<LimbShapePose, 1> support{{{sphere.GetPtr(), JPH::RVec3(100, 0, 0)}}};
    const auto clear = findClearLimbPose(input, support, 0.1f, 4);
    ASSERT_TRUE(clear); EXPECT_FLOAT_EQ(clear->mFootDisplacement, 0);
    EXPECT_EQ(clear->mEvaluatedCandidates, 1u);
    support[0].mPosition = JPH::RVec3::sZero();
    const auto alternative = findClearLimbPose(input, support, 0.1f, 4);
    ASSERT_TRUE(alternative);
    EXPECT_GT(alternative->mFootDisplacement, 2.0f);
    EXPECT_LE(alternative->mFootDisplacement, 4.0f);
    EXPECT_LE(alternative->mEvaluatedCandidates, 321u);
    const auto alternatives = enumerateClearLimbPoses(input, support, 0.1f, 4);
    ASSERT_GT(alternatives.size(), 1u);
    EXPECT_LE(alternatives.size(), 321u);
    float minimum = 4.0f;
    bool positiveSide = false, negativeSide = false;
    for (const auto& candidate : alternatives)
    {
        EXPECT_LE(candidate.mFootDisplacement, 4.0f);
        const auto checked = measureLimbClearance(candidate.mPoses, support, 0.1f);
        ASSERT_TRUE(checked); EXPECT_TRUE(checked->mClear);
        minimum = std::min(minimum, candidate.mFootDisplacement);
        positiveSide = positiveSide || candidate.mPoses[2].mPosition.GetX() > 1;
        negativeSide = negativeSide || candidate.mPoses[2].mPosition.GetX() < -1;
    }
    EXPECT_FLOAT_EQ(minimum, alternative->mFootDisplacement);
    EXPECT_TRUE(positiveSide && negativeSide);
    EXPECT_TRUE(enumerateClearLimbPoses(input, {}, 0.1f, 4).empty());
    EXPECT_TRUE(enumerateClearLimbPoses(input, support, 0.1f, -1).empty());
    const auto clearance = measureLimbClearance(alternative->mPoses, support, 0.1f);
    ASSERT_TRUE(clearance); EXPECT_TRUE(clearance->mClear);
    EXPECT_GE(alternative->mKneeAngleFromReference, input.mLeg.mMinimumAngle);
    EXPECT_LE(alternative->mKneeAngleFromReference, input.mLeg.mMaximumAngle);
    const auto hip = clampSwingTwistJointOrientation(input.mHipLimits, input.mParentRotation,
        alternative->mPoses[0].mRotation);
    ASSERT_TRUE(hip);
    EXPECT_NEAR(std::abs(hip->Dot(alternative->mPoses[0].mRotation)), 1, 1e-5);
    const auto ankle = clampSwingTwistJointOrientation(input.mAnkleLimits, alternative->mPoses[1].mRotation,
        alternative->mPoses[2].mRotation);
    ASSERT_TRUE(ankle);
    EXPECT_NEAR(std::abs(ankle->Dot(alternative->mPoses[2].mRotation)), 1, 1e-5);
    EXPECT_FALSE(findClearLimbPose(input, support, 0.1f, 0));
    EXPECT_FALSE(findClearLimbPose(input, support, 0.1f, -1));
    EXPECT_FALSE(findClearLimbPose(input, {}, 0.1f, 4));
    support[0].mPosition = JPH::RVec3(100, 0, 0);
    input.mMinimumWorldZ = 0;
    EXPECT_FALSE(findClearLimbPose(input, support, 0.1f, 0));
    const auto aboveFloor = findClearLimbPose(input, support, 0.1f, 12);
    ASSERT_TRUE(aboveFloor);
    for (const auto& pose : aboveFloor->mPoses) EXPECT_GE(pose.mPosition.GetZ(), 1.0);
    JPH::RefConst<JPH::Shape> obstruction = new JPH::SphereShape(75);
    support[0] = { obstruction.GetPtr(), JPH::RVec3(0, 0, 10) };
    EXPECT_FALSE(findClearLimbPose(input, support, 0.1f, 12));


}

TEST(LimbClearance, RejectsCollidingTransitionBetweenClearEndpoints)
{
    JPH::RefConst<JPH::Shape> sphere = new JPH::SphereShape(1);
    LimbPoseSearchInput input;
    input.mShapes = { sphere.GetPtr(), sphere.GetPtr(), sphere.GetPtr() };
    input.mHipPosition = JPH::RVec3(0, 0, 20);
    input.mLeg.mUpperOffset = input.mLeg.mLowerOffset = JPH::Vec3(0, 0, -10);
    input.mLeg.mMinimumAngle = 0; input.mLeg.mMaximumAngle = 1.2f;
    input.mHipLimits.mNormalHalfConeAngle = input.mHipLimits.mPlaneHalfConeAngle = 0.8f;
    input.mHipLimits.mTwistMinimum = -0.8f; input.mHipLimits.mTwistMaximum = 0.8f;
    input.mAnkleLimits = input.mHipLimits;
    std::array<LimbShapePose, 1> support{{{sphere.GetPtr()}}};
    input.mHipRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.2f);
    const auto first = findClearLimbPose(input, support, 0.1f, 12);
    ASSERT_TRUE(first);
    LimbPoseContinuity continuity;
    continuity.mPrevious = *first;
    continuity.mPreviousSupport = support;
    continuity.mMaximumFootTravel = 10;
    continuity.mMaximumJointRotation = 0.5f;
    input.mHipRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.2f);
    const auto opposite = findClearLimbPose(input, support, 0.1f, 12);
    ASSERT_TRUE(opposite);
    EXPECT_FALSE(isLimbTransitionClear(input, *opposite, support, 0.1f, continuity));
    continuity.mMaximumFootTravel = 3;
    const auto connected = findClearLimbPose(input, support, 0.1f, 12, &continuity);
    ASSERT_TRUE(connected);
    EXPECT_TRUE(isLimbTransitionClear(input, *connected, support, 0.1f, continuity));
    EXPECT_LE(JPH::Vec3(connected->mPoses[2].mPosition - first->mPoses[2].mPosition).Length(), 3);
    input.mHipRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.25f);
    const auto nearby = findClearLimbPose(input, support, 0.1f, 12);
    ASSERT_TRUE(nearby);
    EXPECT_TRUE(isLimbTransitionClear(input, *nearby, support, 0.1f, continuity));
    continuity.mMaximumFootTravel = 0.1f;
    EXPECT_FALSE(isLimbTransitionClear(input, *nearby, support, 0.1f, continuity));
    continuity.mMaximumFootTravel = 3; continuity.mMaximumJointRotation = 0.01f;
    EXPECT_FALSE(isLimbTransitionClear(input, *nearby, support, 0.1f, continuity));
    continuity.mMaximumJointRotation = 0.5f; continuity.mIntervals = 1;
    EXPECT_FALSE(isLimbTransitionClear(input, *nearby, support, 0.1f, continuity));
}

TEST(LimbClearance, DifferentiatesWorldHipAndParentRelativeAnkle)
{
    JPH::RefConst<JPH::Shape> sphere = new JPH::SphereShape(1);
    ClearLimbPose start, end;
    for (std::size_t i = 0; i < 3; ++i)
    {
        start.mPoses[i].mShape = end.mPoses[i].mShape = sphere.GetPtr();
        start.mPoses[i].mRotation = JPH::Quat::sIdentity();
        end.mPoses[i].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.2f);
    }
    end.mKneeAngleFromReference = 0.1f;
    // All bodies rotate together: hip has a world rate, ankle has no relative rate.
    const auto result = calculateLimbPoseRates(start, end, 0.1f);
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->mHipWorld.GetZ(), 2, 1e-5);
    EXPECT_NEAR(result->mKnee, 1, 1e-5);
    EXPECT_LT(result->mAnkleParent.Length(), 1e-5);
    // Stationary world foot while calf rotates: relative ankle must counter-rotate.
    end.mPoses[2].mRotation = JPH::Quat::sIdentity();
    const auto counter = calculateLimbPoseRates(start, end, 0.1f);
    ASSERT_TRUE(counter);
    EXPECT_NEAR(counter->mAnkleParent.GetZ(), -2, 1e-5);
    // Common world rotation changes world hip axes, not parent-relative ankle axes.
    const auto rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.7f);
    for (std::size_t i = 0; i < 3; ++i)
    {
        start.mPoses[i].mRotation = rotation * start.mPoses[i].mRotation;
        end.mPoses[i].mRotation = -(rotation * end.mPoses[i].mRotation);
    }
    const auto rebased = calculateLimbPoseRates(start, end, 0.1f);
    ASSERT_TRUE(rebased);
    EXPECT_LT((rebased->mHipWorld - rotation * counter->mHipWorld).Length(), 1e-5);
    EXPECT_LT((rebased->mAnkleParent - counter->mAnkleParent).Length(), 1e-5);
    auto tiny = start;
    tiny.mPoses[0].mRotation = (JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.0001f)
        * start.mPoses[0].mRotation).Normalized();
    const auto smallRate = calculateLimbPoseRates(start, tiny, 0.01f);
    ASSERT_TRUE(smallRate);
    EXPECT_NEAR(smallRate->mHipWorld.GetY(), 0.01f, 1e-5f);
    EXPECT_FALSE(calculateLimbPoseRates(start, end, 0));
    EXPECT_FALSE(calculateLimbPoseRates(start, end, -0.1f));
    EXPECT_FALSE(calculateLimbPoseRates(start, end, std::numeric_limits<float>::infinity()));
    end.mPoses[2].mShape = nullptr;
    EXPECT_FALSE(calculateLimbPoseRates(start, end, 0.1f));
}

TEST(LimbClearance, DiagnosisMatchesExactCandidateOutcome)
{
    JPH::RefConst<JPH::Shape> sphere = new JPH::SphereShape(1);
    LimbPoseSearchInput input;
    input.mShapes = { sphere.GetPtr(), sphere.GetPtr(), sphere.GetPtr() };
    input.mHipPosition = JPH::RVec3(0, 0, 20);
    input.mLeg.mUpperOffset = JPH::Vec3(0, 0, -10);
    input.mLeg.mLowerOffset = JPH::Vec3(0, 0, -10);
    input.mLeg.mMinimumAngle = 0;
    input.mLeg.mMaximumAngle = 1.2f;
    input.mHipLimits.mNormalHalfConeAngle = input.mHipLimits.mPlaneHalfConeAngle = 0.8f;
    input.mHipLimits.mTwistMinimum = -0.8f;
    input.mHipLimits.mTwistMaximum = 0.8f;
    input.mAnkleLimits = input.mHipLimits;

    // Distant support: exact request is feasible, zero displacement accepted.
    std::array<LimbShapePose, 1> far{{{sphere.GetPtr(), JPH::RVec3(100, 0, 0)}}};
    ASSERT_TRUE(findClearLimbPose(input, far, 0.1f, 0));
    const auto clearDiagnosis = diagnoseLimbPose(input, far, 0.1f);
    EXPECT_TRUE(clearDiagnosis.mValidInput);
    EXPECT_FALSE(clearDiagnosis.mHipClamped);
    EXPECT_FALSE(clearDiagnosis.mKneeClamped);
    EXPECT_FALSE(clearDiagnosis.mAnkleClamped);
    EXPECT_NEAR(clearDiagnosis.mExactFootDisplacement, 0, 1e-5f);
    EXPECT_TRUE(clearDiagnosis.mSupportClear);
    EXPECT_TRUE(std::isnan(clearDiagnosis.mFloorMargin));

    // Support at the leg: exact request penetrates, bound-0 query rejects, and
    // the diagnosis attributes the rejection to support clearance.
    std::array<LimbShapePose, 1> near{{{sphere.GetPtr(), JPH::RVec3::sZero()}}};
    EXPECT_FALSE(findClearLimbPose(input, near, 0.1f, 0));
    const auto contactDiagnosis = diagnoseLimbPose(input, near, 0.1f);
    EXPECT_TRUE(contactDiagnosis.mValidInput);
    EXPECT_FALSE(contactDiagnosis.mSupportClear);
    ASSERT_TRUE(std::isfinite(contactDiagnosis.mSupportGap));
    EXPECT_LT(contactDiagnosis.mSupportGap, 0.1f);
    EXPECT_FALSE(contactDiagnosis.mHipClamped);
    EXPECT_NEAR(contactDiagnosis.mExactFootDisplacement, 0, 1e-5f);

    // Requested hip outside the supplied cone: clamp delta explains rejection.
    auto clampedInput = input;
    clampedInput.mHipRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 2.0f);
    EXPECT_FALSE(findClearLimbPose(clampedInput, far, 0.1f, 0));
    const auto clampDiagnosis = diagnoseLimbPose(clampedInput, far, 0.1f);
    EXPECT_TRUE(clampDiagnosis.mValidInput);
    EXPECT_TRUE(clampDiagnosis.mHipClamped);
    EXPECT_GT(clampDiagnosis.mHipClampDelta, 0.5f);
    EXPECT_GT(clampDiagnosis.mExactFootDisplacement, 1.0f);
    EXPECT_TRUE(clampDiagnosis.mSupportClear);

    // Floor bound: straight-down leg ends at z=0, floor at 1 must reject.
    auto floorInput = input;
    floorInput.mMinimumWorldZ = 1;
    EXPECT_FALSE(findClearLimbPose(floorInput, far, 0.1f, 0));
    const auto floorDiagnosis = diagnoseLimbPose(floorInput, far, 0.1f);
    EXPECT_FALSE(floorDiagnosis.mFloorClear);
    EXPECT_LT(floorDiagnosis.mFloorMargin, 0.0f);
    EXPECT_TRUE(floorDiagnosis.mSupportClear);
}

TEST(LimbClearance, DiagnosisAttributesTransitionFailures)
{
    JPH::RefConst<JPH::Shape> sphere = new JPH::SphereShape(1);
    LimbPoseSearchInput input;
    input.mShapes = { sphere.GetPtr(), sphere.GetPtr(), sphere.GetPtr() };
    input.mHipPosition = JPH::RVec3(0, 0, 20);
    input.mLeg.mUpperOffset = input.mLeg.mLowerOffset = JPH::Vec3(0, 0, -10);
    input.mLeg.mMinimumAngle = 0; input.mLeg.mMaximumAngle = 1.2f;
    input.mHipLimits.mNormalHalfConeAngle = input.mHipLimits.mPlaneHalfConeAngle = 0.8f;
    input.mHipLimits.mTwistMinimum = -0.8f; input.mHipLimits.mTwistMaximum = 0.8f;
    input.mAnkleLimits = input.mHipLimits;
    std::array<LimbShapePose, 1> support{{{sphere.GetPtr()}}};
    input.mHipRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.2f);
    const auto first = findClearLimbPose(input, support, 0.1f, 12);
    ASSERT_TRUE(first);
    LimbPoseContinuity continuity;
    continuity.mPrevious = *first;
    continuity.mPreviousSupport = support;
    continuity.mMaximumFootTravel = 10;
    continuity.mMaximumJointRotation = 0.5f;
    input.mHipRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.25f);
    const auto nearby = findClearLimbPose(input, support, 0.1f, 12);
    ASSERT_TRUE(nearby);
    ASSERT_TRUE(isLimbTransitionClear(input, *nearby, support, 0.1f, continuity));
    const auto clear = diagnoseLimbTransition(input, *nearby, support, 0.1f, continuity);
    EXPECT_EQ(clear.mFailure, LimbTransitionFailure::None);
    EXPECT_GE(clear.mFootTravel, 0.0f);
    // Same pair, tight travel bound: diagnosis must name the bound, not a sample.
    continuity.mMaximumFootTravel = 0.1f;
    ASSERT_FALSE(isLimbTransitionClear(input, *nearby, support, 0.1f, continuity));
    const auto travel = diagnoseLimbTransition(input, *nearby, support, 0.1f, continuity);
    EXPECT_EQ(travel.mFailure, LimbTransitionFailure::FootTravel);
    EXPECT_EQ(travel.mFirstFailingSample, -1);
    EXPECT_GT(travel.mFootTravel, 0.1f);
    // Tight joint-rotation bound: named as hip change in parent-relative space.
    continuity.mMaximumFootTravel = 10;
    continuity.mMaximumJointRotation = 0.01f;
    ASSERT_FALSE(isLimbTransitionClear(input, *nearby, support, 0.1f, continuity));
    const auto joint = diagnoseLimbTransition(input, *nearby, support, 0.1f, continuity);
    EXPECT_EQ(joint.mFailure, LimbTransitionFailure::HipChange);
    EXPECT_GT(joint.mHipChange, 0.01f);
}
