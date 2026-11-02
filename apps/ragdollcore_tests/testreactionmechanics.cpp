#include <gtest/gtest.h>

#include <ragdollcore/reactionmechanics.hpp>

#include <limits>
#include <vector>

using namespace RagdollCore;

namespace
{
    Transform rotated(float angle)
    {
        Transform transform;
        transform.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), angle);
        return transform;
    }

    float deviation(JPH::QuatArg first, JPH::QuatArg second)
    {
        const JPH::Quat relative = (first * second.Conjugated()).Normalized();
        return 2.0f * std::atan2(relative.GetXYZ().Length(), std::abs(relative.GetW()));
    }
}

TEST(RagdollCoreReactionMechanicsTest, SlidingContactIsNotPlantedSupport)
{
    SupportSlipProfile profile;
    FootObservation left;
    left.mSupported = true;
    left.mHorizontalSpeed = 40.0f; // sliding
    FootObservation right;
    right.mSupported = true;
    right.mHorizontalSpeed = 0.4f;
    const auto measurement = measureSupportSlip(left, right, profile);
    EXPECT_FALSE(measurement.mLeftPlanted);
    EXPECT_TRUE(measurement.mRightPlanted);
    EXPECT_EQ(measurement.mPlantedCount, 1);
    EXPECT_NEAR(measurement.mStanceSlip, 0.4f, 1e-4f);
    EXPECT_FLOAT_EQ(measurement.mSwingLift, 0.0f);
}

TEST(RagdollCoreReactionMechanicsTest, LiftComesFromTheUnsupportedFoot)
{
    SupportSlipProfile profile;
    FootObservation left;
    left.mSupported = true;
    left.mHorizontalSpeed = 0.1f;
    FootObservation right;
    right.mSupported = false;
    right.mHeight = 4.0f;
    const auto measurement = measureSupportSlip(left, right, profile);
    EXPECT_EQ(measurement.mPlantedCount, 1);
    EXPECT_NEAR(measurement.mSwingLift, 4.0f, 1e-4f);
}

TEST(RagdollCoreReactionMechanicsTest, HingeClampKeepsTheTargetInsideTheLimit)
{
    HingeClampProfile profile;
    profile.mMargin = 0.02f;
    float removed = 0.0f;
    EXPECT_NEAR(clampHingeTarget(0.12f, -2.1f, 0.12f, profile, removed), 0.10f, 1e-5f);
    EXPECT_GT(removed, 0.0f);
    EXPECT_NEAR(clampHingeTarget(-0.5f, -2.1f, 0.12f, profile, removed), -0.5f, 1e-5f);
    EXPECT_FLOAT_EQ(removed, 0.0f);
    // Non-finite input is safe and reports no removal.
    const float safe = clampHingeTarget(std::numeric_limits<float>::quiet_NaN(), -2.1f, 0.12f,
        profile, removed);
    EXPECT_TRUE(std::isfinite(safe));
    EXPECT_GE(safe, -2.1f);
    EXPECT_LE(safe, 0.12f);
    EXPECT_FLOAT_EQ(removed, 0.0f);
}

TEST(RagdollCoreReactionMechanicsTest, MirroredKneeTargetsExposeAsymmetry)
{
    const auto symmetric = compareMirroredHingeTargets(-0.3f, -0.3f);
    EXPECT_FALSE(symmetric.mAsymmetric);
    const auto clamped = compareMirroredHingeTargets(-0.3f, 0.12f); // right pinned at the limit
    EXPECT_TRUE(clamped.mAsymmetric);
    EXPECT_GT(std::abs(clamped.mDelta), 0.4f);
    const auto invalid = compareMirroredHingeTargets(std::numeric_limits<float>::quiet_NaN(), 0.0f);
    EXPECT_TRUE(invalid.mAsymmetric);
}

TEST(RagdollCoreReactionMechanicsTest, ReactionDisturbanceLatchesThenDecays)
{
    ReactionAuthorityProfile profile;
    profile.mDisturbanceRetention = 0.2f;
    profile.mDisturbanceDecayRate = 1.0f;
    ReactionDisturbanceLatch latch;
    latch.begin(2000.0f, profile);
    latch.step(0.1f, profile);
    EXPECT_FLOAT_EQ(latch.retained(), 2000.0f); // inside the retention window
    latch.begin(1500.0f, profile);
    EXPECT_FLOAT_EQ(latch.retained(), 2000.0f); // a weaker echo cannot lower it
    latch.step(0.2f, profile); // consumes the retention, no decay yet
    EXPECT_FLOAT_EQ(latch.retained(), 2000.0f);
    latch.step(0.5f, profile);
    EXPECT_LT(latch.retained(), 2000.0f);
    EXPECT_GT(latch.retained(), 0.0f);
    for (int i = 0; i < 40; ++i)
        latch.step(0.5f, profile); // exponential decay reaches zero
    EXPECT_FLOAT_EQ(latch.retained(), 0.0f);
    latch.reset();
    EXPECT_FLOAT_EQ(latch.retained(), 0.0f);
}

TEST(RagdollCoreReactionMechanicsTest, CompositionOrderIsClipThenReactionThenBalance)
{
    std::vector<Transform> clip(4);
    std::vector<Transform> balance(4);
    std::vector<Transform> out(4);
    ReactionOutputs outputs;
    outputs.mTorsoFlexion = 0.2f;
    outputs.mCradleLeft = true;
    outputs.mPoseCorrectionAuthority = 0.0f;
    ReactionPoseTargets targets;
    targets.mSpine = 1;
    targets.mLeftUpperArm = 2;
    ReactionComposeProfile profile;
    ASSERT_TRUE(composeReactionTargets(clip, balance, outputs, targets, out, profile));
    // Zero authority: the reaction pose survives untouched by the balance pose.
    EXPECT_NEAR(deviation(out[1].mRotation, clip[1].mRotation), 0.2f, 1e-3f);
    EXPECT_GT(deviation(out[2].mRotation, clip[2].mRotation), 0.2f);

    // High authority pulls toward the balance pose, capped by the deviation
    // budget (8 degrees by default).
    std::vector<Transform> balancePose(4);
    for (auto& transform : balancePose)
        transform = rotated(0.6f);
    outputs.mPoseCorrectionAuthority = 1.0f;
    ASSERT_TRUE(composeReactionTargets(clip, balancePose, outputs, targets, out, profile));
    const float toBalance = deviation(out[1].mRotation, balancePose[1].mRotation);
    const float fromReaction = deviation(out[1].mRotation, clip[1].mRotation);
    EXPECT_GT(toBalance, 0.0f);
    EXPECT_LT(fromReaction, 0.6f);
    EXPECT_LE(fromReaction, 0.2f + profile.mAuthority.mMaximumAnimationDeviationDegrees * 3.14159f / 180.0f + 1e-3f);
}

TEST(RagdollCoreReactionMechanicsTest, CompositionRejectsMismatchedSpans)
{
    std::vector<Transform> clip(4);
    std::vector<Transform> balance(3);
    std::vector<Transform> out(4);
    ReactionOutputs outputs;
    ReactionPoseTargets targets;
    ReactionComposeProfile profile;
    EXPECT_FALSE(composeReactionTargets(clip, balance, outputs, targets, out, profile));
}

TEST(RagdollCoreReactionMechanicsTest, CompositionIsDeterministic)
{
    const auto run = [] {
        std::vector<Transform> clip(4);
        std::vector<Transform> balance(4);
        for (auto& transform : balance)
            transform = rotated(0.5f);
        std::vector<Transform> out(4);
        ReactionOutputs outputs;
        outputs.mTorsoFlexion = 0.15f;
        outputs.mPoseCorrectionAuthority = 0.6f;
        ReactionPoseTargets targets;
        targets.mSpine = 1;
        ReactionComposeProfile profile;
        composeReactionTargets(clip, balance, outputs, targets, out, profile);
        return out;
    };
    const auto first = run();
    const auto second = run();
    for (std::size_t i = 0; i < first.size(); ++i)
    {
        EXPECT_FLOAT_EQ(first[i].mRotation.GetX(), second[i].mRotation.GetX());
        EXPECT_FLOAT_EQ(first[i].mRotation.GetW(), second[i].mRotation.GetW());
    }
}
