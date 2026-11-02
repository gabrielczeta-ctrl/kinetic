#include <ragdollcore/armtarget.hpp>
#include <ragdollcore/weaponswing.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace
{
    using RagdollCore::ArmTargetInput;
    using RagdollCore::Transform;

    TEST(RagdollCoreArmTargetTest, protectiveArmDriveOnlyOverridesDownedMotorSuppressionForABracedArm)
    {
        using Mode = RagdollCore::ActiveControllerMode;
        EXPECT_FALSE(RagdollCore::protectiveArmDriveEnabled(Mode::Standing, true, false));
        EXPECT_FALSE(RagdollCore::protectiveArmDriveEnabled(Mode::Falling, false, false));
        EXPECT_TRUE(RagdollCore::protectiveArmDriveEnabled(Mode::Falling, true, false));
        EXPECT_TRUE(RagdollCore::protectiveArmDriveEnabled(Mode::Fallen, false, true));
        EXPECT_FALSE(RagdollCore::protectiveArmDriveEnabled(Mode::Stumbling, true, true));
    }

    TEST(RagdollCoreArmTargetTest, acceptsWallsButRejectsFloorAndCeilingForWallBrace)
    {
        EXPECT_TRUE(RagdollCore::isVerticalBraceSurface(JPH::Vec3(1, 0, 0)));
        EXPECT_TRUE(RagdollCore::isVerticalBraceSurface(JPH::Vec3(0, 1, 0.2f)));
        EXPECT_FALSE(RagdollCore::isVerticalBraceSurface(JPH::Vec3(0, 0, 1)));
        EXPECT_FALSE(RagdollCore::isVerticalBraceSurface(JPH::Vec3(0, 0, -1)));
        EXPECT_FALSE(RagdollCore::isVerticalBraceSurface(JPH::Vec3::sZero()));
    }

    constexpr std::size_t sFrame = 0;
    constexpr std::size_t sShoulder = 1;
    constexpr std::size_t sElbow = 2;
    constexpr std::size_t sHand = 3;

    // A right arm hanging along -Z from a clavicle, at the shoulder height of
    // the humanoid profile and in the same model units the real chain uses.
    std::vector<Transform> restArm()
    {
        std::vector<Transform> pose(4);
        pose[sFrame].mTranslation = JPH::Vec3(2.0f, 0.0f, 60.0f);
        pose[sShoulder].mTranslation = JPH::Vec3(6.0f, 0.0f, 58.0f);
        pose[sElbow].mTranslation = JPH::Vec3(6.0f, 0.0f, 46.0f);
        pose[sHand].mTranslation = JPH::Vec3(6.0f, 0.0f, 34.0f);
        return pose;
    }

    ArmTargetInput armInput()
    {
        ArmTargetInput input;
        input.mShoulderBodyIndex = sShoulder;
        input.mElbowBodyIndex = sElbow;
        input.mHandBodyIndex = sHand;
        input.mPreferredBendDirection = JPH::Vec3::sAxisY();
        return input;
    }

    TEST(RagdollCoreArmTargetTest, partialBlendKeepsItsPathAcrossHalfTurnAndQuaternionSigns)
    {
        const auto base = restArm();
        auto input = armInput();
        input.mTargetHandPosition = JPH::Vec3(14, 6, 48);
        input.mWeight = 0.5f;
        std::optional<JPH::Quat> previous;
        constexpr float radians = 3.14159265359f / 180.0f;
        for (int degrees = 175; degrees <= 185; ++degrees)
        {
            auto pose = base;
            auto wanted = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), degrees * radians);
            if (degrees % 2) wanted = -wanted;
            input.mHandRotation = wanted;
            const auto result = RagdollCore::applyArmTargetPose(pose, base, base, sFrame, input);
            ASSERT_TRUE(result && result->mApplied && result->mRotationBlendHistory);
            const auto expected = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), degrees * radians * 0.5f);
            EXPECT_GT(std::abs(pose[sHand].mRotation.Dot(expected)), 0.99999f);
            if (previous)
                EXPECT_GT(std::abs(previous->Dot(pose[sHand].mRotation)), 0.9999f);
            previous = pose[sHand].mRotation;
            input.mRotationBlendHistory = result->mRotationBlendHistory;
        }
        // A fresh request still chooses the original shortest arc.
        input.mRotationBlendHistory.reset();
        auto fresh = base;
        const auto result = RagdollCore::applyArmTargetPose(fresh, base, base, sFrame, input);
        ASSERT_TRUE(result && result->mApplied);
        EXPECT_LT(std::abs(fresh[sHand].mRotation.Dot(*previous)), 0.01f);
        // Endpoints remain exact even when history selects the longer path.
        input.mRotationBlendHistory = result->mRotationBlendHistory;
        for (float weight : { 0.0f, 1.0f })
        {
            auto pose = base;
            input.mWeight = weight;
            const auto endpoint = RagdollCore::applyArmTargetPose(pose, base, base, sFrame, input);
            ASSERT_TRUE(endpoint && endpoint->mApplied);
            const auto expected = weight == 0 ? base[sHand].mRotation : *input.mHandRotation;
            EXPECT_GT(std::abs(pose[sHand].mRotation.Dot(expected)), 0.99999f);
        }
    }

    TEST(RagdollCoreArmTargetTest, braceStaysOnFloorAndReleasesWhenFloorIsUnreachable)
    {
        RagdollCore::HingeLegGeometry geometry;
        geometry.mUpperOffset = JPH::Vec3(0, 0, -12);
        geometry.mLowerOffset = JPH::Vec3(0, 0, -12);
        geometry.mMinimumAngle = -2.0f;
        geometry.mMaximumAngle = 0.0f;
        RagdollCore::ActiveControllerProfile profile;
        profile.mBraceHandOffset = 0;
        profile.mBraceHandClearance = 1;
        profile.mBraceReachMargin = 0;
        // The floor directly under the shoulder is too close to fold to.
        // The target must move outward, retain floor height and satisfy reach.
        const auto target = RagdollCore::calculateBraceHandTarget(JPH::RVec3(0, 0, 5),
            JPH::RVec3(6, 0, 5), JPH::RVec3(6, 0, -7), JPH::RVec3(6, 0, -19), 0, geometry, profile);
        ASSERT_TRUE(target);
        EXPECT_DOUBLE_EQ(target->mWorldTarget.GetZ(), 1);
        EXPECT_GT(target->mWorldTarget.GetX(), 6);
        EXPECT_TRUE(target->mWorldBend.IsClose(JPH::Vec3::sAxisX()));
        const auto range = RagdollCore::hingeChainReachRange(geometry);
        ASSERT_TRUE(range);
        EXPECT_NEAR((target->mWorldTarget - JPH::RVec3(6, 0, 5)).Length(), range->first, 1e-5);
        EXPECT_FALSE(RagdollCore::calculateBraceHandTarget(JPH::RVec3(0, 0, 60),
            JPH::RVec3(6, 0, 60), JPH::RVec3(6, 0, 48), JPH::RVec3(6, 0, 36), 0, geometry, profile));
        EXPECT_FALSE(RagdollCore::calculateBraceHandTarget(JPH::RVec3(0, 0, 5),
            JPH::RVec3(6, 0, 5), JPH::RVec3(6, 0, -7), JPH::RVec3(6, 0, -19), 0, std::nullopt, profile));
    }

    TEST(RagdollCoreArmTargetTest, placesTheHandOnAReachableTargetWithoutStretchingTheArm)
    {
        const auto base = restArm();
        auto target = base;
        const float upper = (base[sElbow].mTranslation - base[sShoulder].mTranslation).Length();
        const float lower = (base[sHand].mTranslation - base[sElbow].mTranslation).Length();

        auto input = armInput();
        input.mTargetHandPosition = JPH::Vec3(14.0f, 6.0f, 48.0f);
        const auto result = RagdollCore::applyArmTargetPose(target, base, base, sFrame, input);
        ASSERT_TRUE(result);
        EXPECT_TRUE(result->mApplied);
        EXPECT_LT(result->mReachError, 1e-3f);
        EXPECT_TRUE(target[sHand].mTranslation.IsClose(input.mTargetHandPosition, 1e-3f));
        EXPECT_NEAR((target[sElbow].mTranslation - target[sShoulder].mTranslation).Length(), upper, 1e-2f);
        EXPECT_NEAR((target[sHand].mTranslation - target[sElbow].mTranslation).Length(), lower, 1e-2f);
        for (const std::size_t body : { sShoulder, sElbow, sHand })
            EXPECT_TRUE(target[body].mRotation.IsNormalized());
        // The frame body is never an output of the solve.
        EXPECT_TRUE(target[sFrame].mTranslation.IsClose(base[sFrame].mTranslation));
    }

    TEST(RagdollCoreArmTargetTest, projectsAReachableWallBraceAlongTheMeasuredNormal)
    {
        RagdollCore::HingeLegGeometry geometry;
        geometry.mUpperOffset = JPH::Vec3(0, 0, -12);
        geometry.mLowerOffset = JPH::Vec3(0, 0, -12);
        geometry.mMinimumAngle = -2.0f;
        geometry.mMaximumAngle = 0.0f;
        RagdollCore::ActiveControllerProfile profile;
        profile.mBraceHandClearance = 2.0f;
        profile.mBraceReachMargin = 0.0f;

        const auto target = RagdollCore::calculateSurfaceBraceHandTarget(
            JPH::RVec3(0, 0, 40), JPH::RVec3(6, 0, 50), JPH::RVec3(6, 0, 38),
            JPH::RVec3(6, 0, 26), JPH::RVec3(18, 0, 38), JPH::Vec3(-1, 0, 0),
            geometry, profile);
        ASSERT_TRUE(target);
        EXPECT_NEAR(target->mWorldTarget.GetX(), 16.0, 1e-5);
        EXPECT_NEAR(target->mWorldTarget.GetY(), 0.0, 1e-5);
        EXPECT_NEAR(target->mWorldTarget.GetZ(), 38.0, 1e-5);
        EXPECT_TRUE(target->mWorldBend.IsClose(JPH::Vec3::sAxisX()));
        EXPECT_FALSE(RagdollCore::calculateSurfaceBraceHandTarget(
            JPH::RVec3(0, 0, 40), JPH::RVec3(6, 0, 50), JPH::RVec3(6, 0, 38),
            JPH::RVec3(6, 0, 26), JPH::RVec3(18, 0, 38), JPH::Vec3::sZero(),
            geometry, profile));
    }

    TEST(RagdollCoreArmTargetTest, shoulderConstrainedSolvePreservesRequestedElbowSide)
    {
        const auto base = restArm();
        for (const float sign : { -1.0f, 1.0f })
        {
            auto pose = base;
            auto input = armInput();
            input.mTargetHandPosition = base[sShoulder].mTranslation + JPH::Vec3(16, 0, 0);
            input.mPreferredBendDirection = JPH::Vec3(0, sign, 0);
            input.mShoulderGeometry.emplace();
            input.mShoulderGeometry->mNormalHalfConeAngle = 3.14159265f;
            input.mShoulderGeometry->mPlaneHalfConeAngle = 3.14159265f;
            input.mShoulderGeometry->mTwistMinimum = -3.14159265f;
            input.mShoulderGeometry->mTwistMaximum = 3.14159265f;
            const auto result = RagdollCore::applyArmTargetPose(pose, base, base, sFrame, input);
            ASSERT_TRUE(result && result->mApplied);
            EXPECT_LT(result->mReachError, 0.03f);
            auto bend = pose[sElbow].mTranslation - pose[sShoulder].mTranslation;
            bend.SetX(0);
            EXPECT_GT(bend.NormalizedOr(JPH::Vec3::sZero()).Dot(input.mPreferredBendDirection), 0.95f);
        }
    }

    TEST(RagdollCoreArmTargetTest, constrainedGesturePrioritizesItsUndrivenDepth)
    {
        const auto base = restArm();
        auto input = armInput();
        input.mHingeGeometry.emplace();
        input.mHingeGeometry->mUpperOffset = JPH::Vec3(0, 0, -12);
        input.mHingeGeometry->mLowerOffset = JPH::Vec3(0, 0, -12);
        input.mHingeGeometry->mHingeAxis = JPH::Vec3::sAxisX();
        input.mHingeGeometry->mMinimumAngle = 0;
        input.mHingeGeometry->mMaximumAngle = 2;
        input.mShoulderGeometry.emplace();
        input.mShoulderGeometry->mNormalHalfConeAngle = 0.4f;
        input.mShoulderGeometry->mPlaneHalfConeAngle = 0.4f;
        input.mShoulderGeometry->mTwistMinimum = -0.6f;
        input.mShoulderGeometry->mTwistMaximum = 0.6f;
        input.mTargetHandPosition = base[sShoulder].mTranslation + JPH::Vec3(0, 8, 4);
        auto ordinary = base;
        ASSERT_TRUE(RagdollCore::applyArmTargetPose(ordinary, base, base, sFrame, input));
        input.mPreservedPositionAxis = JPH::Vec3::sAxisY();
        auto gesture = base;
        const auto result = RagdollCore::applyArmTargetPose(gesture, base, base, sFrame, input);
        ASSERT_TRUE(result && result->mApplied);
        const float ordinaryDepthError = std::abs(ordinary[sHand].mTranslation.GetY() - 8.0f);
        const float gestureDepthError = std::abs(gesture[sHand].mTranslation.GetY() - 8.0f);
        EXPECT_LT(gestureDepthError, ordinaryDepthError);
        EXPECT_LT(gestureDepthError, 1.0f);
        const auto legal = RagdollCore::clampSwingTwistJointOrientation(*input.mShoulderGeometry,
            base[sFrame].mRotation, gesture[sShoulder].mRotation);
        ASSERT_TRUE(legal);
        EXPECT_GT(std::abs(legal->Dot(gesture[sShoulder].mRotation)), 0.99999f);
    }

    TEST(RagdollCoreArmTargetTest, reportsHowFarShortAnOutOfReachTargetFalls)
    {
        const auto base = restArm();
        auto target = base;
        auto input = armInput();
        // Straight out to the side, far past the arm's span.
        input.mTargetHandPosition = JPH::Vec3(200.0f, 0.0f, 58.0f);
        const auto result = RagdollCore::applyArmTargetPose(target, base, base, sFrame, input);
        ASSERT_TRUE(result);
        EXPECT_TRUE(result->mApplied);
        EXPECT_GT(result->mReachError, 100.0f);
        // Clamped, not rejected: the arm still points at the request.
        const auto reach = target[sHand].mTranslation - base[sShoulder].mTranslation;
        const auto wanted = input.mTargetHandPosition - base[sShoulder].mTranslation;
        EXPECT_GT(reach.Normalized().Dot(wanted.Normalized()), 0.99f);
    }

    TEST(RagdollCoreArmTargetTest, weightScalesHowFarTheArmMovesFromThePoseItWasGiven)
    {
        const auto base = restArm();
        auto input = armInput();
        input.mTargetHandPosition = JPH::Vec3(14.0f, 6.0f, 48.0f);

        auto full = base;
        ASSERT_TRUE(RagdollCore::applyArmTargetPose(full, base, base, sFrame, input));

        auto none = base;
        input.mWeight = 0.0f;
        const auto idle = RagdollCore::applyArmTargetPose(none, base, base, sFrame, input);
        ASSERT_TRUE(idle);
        // Zero weight still solves, so a caller can ask whether the arm could
        // reach before it commits to reaching.
        EXPECT_TRUE(idle->mApplied);
        EXPECT_LT(idle->mReachError, 1e-3f);
        for (std::size_t i = 0; i < none.size(); ++i)
            EXPECT_TRUE(none[i].mTranslation.IsClose(base[i].mTranslation, 1e-5f));

        auto half = base;
        input.mWeight = 0.5f;
        ASSERT_TRUE(RagdollCore::applyArmTargetPose(half, base, base, sFrame, input));
        const float fullTravel = (full[sHand].mTranslation - base[sHand].mTranslation).Length();
        const float halfTravel = (half[sHand].mTranslation - base[sHand].mTranslation).Length();
        EXPECT_GT(fullTravel, 1.0f);
        EXPECT_NEAR(halfTravel, 0.5f * fullTravel, 1e-2f);
    }

    TEST(RagdollCoreArmTargetTest, keepsTheWristAgainstTheForearmUnlessGivenAnOrientation)
    {
        auto base = restArm();
        // Give the hand an angle against its forearm worth preserving.
        base[sHand].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.4f);
        auto target = base;
        auto input = armInput();
        input.mTargetHandPosition = JPH::Vec3(14.0f, 6.0f, 48.0f);
        ASSERT_TRUE(RagdollCore::applyArmTargetPose(target, base, base, sFrame, input));
        const auto expected = base[sElbow].mRotation.Conjugated() * base[sHand].mRotation;
        const auto actual = target[sElbow].mRotation.Conjugated() * target[sHand].mRotation;
        EXPECT_GT(std::abs(expected.Dot(actual)), 0.999f);

        auto oriented = base;
        input.mHandRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.1f);
        ASSERT_TRUE(RagdollCore::applyArmTargetPose(oriented, base, base, sFrame, input));
        EXPECT_GT(std::abs(oriented[sHand].mRotation.Dot(*input.mHandRotation)), 0.999f);
    }

    TEST(RagdollCoreArmTargetTest, orientedToolUsesShoulderRollWithoutExceedingWristLimits)
    {
        const auto base = restArm();
        auto input = armInput();
        input.mHingeGeometry.emplace();
        input.mHingeGeometry->mUpperOffset = JPH::Vec3(0, 0, -12);
        input.mHingeGeometry->mLowerOffset = JPH::Vec3(0, 0, -12);
        input.mHingeGeometry->mHingeAxis = JPH::Vec3::sAxisX();
        input.mHingeGeometry->mMinimumAngle = 0;
        input.mHingeGeometry->mMaximumAngle = 2;
        input.mShoulderGeometry.emplace();
        input.mShoulderGeometry->mNormalHalfConeAngle = 3.14159265f;
        input.mShoulderGeometry->mPlaneHalfConeAngle = 3.14159265f;
        input.mShoulderGeometry->mTwistMinimum = -3.14159265f;
        input.mShoulderGeometry->mTwistMaximum = 3.14159265f;
        // Construct a reachable point AND blade direction with a nearly fixed
        // wrist. The requested pole alone prefers a different shoulder roll.
        const auto shoulder = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.6f);
        const auto elbow = shoulder * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 1.2f);
        input.mTargetHandPosition = base[sShoulder].mTranslation
            + shoulder * JPH::Vec3(0, 0, -12) + elbow * JPH::Vec3(0, 0, -12);
        input.mPreferredBendDirection = JPH::Vec3::sAxisX();
        input.mHandRotation = elbow;
        input.mHandGeometry.emplace();
        input.mHandGeometry->mNormalHalfConeAngle = 0.03f;
        input.mHandGeometry->mPlaneHalfConeAngle = 0.03f;
        input.mHandGeometry->mTwistMinimum = -0.03f;
        input.mHandGeometry->mTwistMaximum = 0.03f;
        auto positionOnly = base;
        ASSERT_TRUE(RagdollCore::applyArmTargetPose(positionOnly, base, base, sFrame, input));
        const auto desiredTip = elbow * JPH::Vec3(0, 0, 12);
        const float oldError = (positionOnly[sHand].mRotation * JPH::Vec3(0, 0, 12) - desiredTip).Length();

        input.mHandAimOffset = JPH::Vec3(0, 0, 12);
        auto oriented = base;
        const auto result = RagdollCore::applyArmTargetPose(oriented, base, base, sFrame, input);
        ASSERT_TRUE(result && result->mApplied);
        EXPECT_LT(result->mReachError, 0.03f);
        const float tipError = (oriented[sHand].mRotation * input.mHandAimOffset - desiredTip).Length();
        EXPECT_GT(oldError, 1.0f);
        EXPECT_LT(tipError, 0.05f);
        const auto legal = RagdollCore::clampSwingTwistJointOrientation(
            *input.mHandGeometry, oriented[sElbow].mRotation, oriented[sHand].mRotation);
        ASSERT_TRUE(legal);
        EXPECT_GT(std::abs(legal->Dot(oriented[sHand].mRotation)), 0.99999f);
        EXPECT_NEAR((oriented[sElbow].mTranslation - oriented[sShoulder].mTranslation).Length(), 12, 1e-3f);
        EXPECT_NEAR((oriented[sHand].mTranslation - oriented[sElbow].mTranslation).Length(), 12, 1e-3f);
    }

    TEST(RagdollCoreArmTargetTest, nativeClaymoreReachKeepsTheHandAndToolAimTogether)
    {
        // PLAY-04 native witness: cb5010eeb4, claymore first quick release.
        // Raw capture: runtime-PLAY-04-guided-claymore-trace/arm-witness.txt.
        std::vector<Transform> base(4), measured(4);
        auto input = armInput();
        base[0].mTranslation = JPH::Vec3(3.60586095f, 8.61002541f, 99.8069839f);
        base[0].mRotation = JPH::Quat(0.913877606f, 0.203328311f, -0.244885117f, 0.252025038f).Normalized();
        measured[0].mTranslation = JPH::Vec3(7.0359745f, 3.12772679f, 97.8544464f);
        measured[0].mRotation = JPH::Quat(0.848172605f, 0.483955294f, -0.214325264f, -0.0213389862f).Normalized();
        base[1].mTranslation = JPH::Vec3(11.2298231f, 10.983139f, 94.5475311f);
        base[1].mRotation = JPH::Quat(-0.595392525f, -0.412444711f, 0.655126631f, 0.214956388f).Normalized();
        measured[1].mTranslation = JPH::Vec3(11.1896152f, 11.1434765f, 94.6038895f);
        measured[1].mRotation = JPH::Quat(0.684528112f, 0.606808007f, -0.400274903f, -0.0546379089f).Normalized();
        base[2].mTranslation = JPH::Vec3(7.65230227f, 24.9035568f, 83.6890488f);
        base[2].mRotation = JPH::Quat(-0.592773616f, -0.416199774f, 0.65375489f, 0.21909298f).Normalized();
        measured[2].mTranslation = JPH::Vec3(10.0999842f, 26.9963913f, 86.037796f);
        measured[2].mRotation = JPH::Quat(0.729359925f, 0.583431304f, -0.35390681f, 0.0489076972f).Normalized();
        base[3].mTranslation = JPH::Vec3(4.24421597f, 38.1116371f, 73.6514511f);
        base[3].mRotation = JPH::Quat(-0.521591902f, 0.542379141f, 0.540179968f, 0.376792222f).Normalized();
        measured[3].mTranslation = JPH::Vec3(11.003499f, 40.9128113f, 76.3203812f);
        measured[3].mRotation = JPH::Quat(-0.29102537f, 0.477387339f, 0.811839938f, -0.168290347f).Normalized();
        input.mTargetHandPosition = JPH::Vec3(30.6695366f, 34.725647f, 107.517921f);
        input.mPreferredBendDirection = JPH::Vec3(0.0f, 0.0f, -1.0f);
        input.mHingeGeometry.emplace();
        input.mHingeGeometry->mUpperOffset = JPH::Vec3(18.0134315f, 0.0f, 0.0f);
        input.mHingeGeometry->mLowerOffset = JPH::Vec3(8.10890961f, -14.8683338f, 3.81469727e-06f);
        input.mHingeGeometry->mLowerRotation = JPH::Quat(-5.96046448e-08f, -5.96046448e-08f, -0.510488868f, 0.859884381f).Normalized();
        input.mHingeGeometry->mHingeAxis = JPH::Vec3(-5.96046519e-08f, -2.98023224e-07f, -1.0f);
        input.mHingeGeometry->mMinimumAngle = -1.22151f;
        input.mHingeGeometry->mMaximumAngle = 0.928494f;
        input.mShoulderGeometry.emplace();
        input.mShoulderGeometry->mParentFrame = JPH::Quat(-0.515338838f, -0.240021423f, -0.487581402f, 0.662631154f).Normalized();
        input.mShoulderGeometry->mChildFrame = JPH::Quat(-0.481775135f, 1.70047336e-08f, 7.82911069e-09f, 0.876294911f).Normalized();
        input.mShoulderGeometry->mNormalHalfConeAngle = 1.75f;
        input.mShoulderGeometry->mPlaneHalfConeAngle = 1.55f;
        input.mShoulderGeometry->mTwistMinimum = -1.05f;
        input.mShoulderGeometry->mTwistMaximum = 1.05f;
        input.mHandGeometry.emplace();
        input.mHandGeometry->mParentFrame = JPH::Quat(0.985141695f, 1.30233943e-07f, 3.78147647e-08f, -0.171743765f).Normalized();
        input.mHandGeometry->mChildFrame = JPH::Quat(0.950517893f, 0.0207036063f, -0.0917794257f, -0.29608047f).Normalized();
        input.mHandGeometry->mNormalHalfConeAngle = 2.0f;
        input.mHandGeometry->mPlaneHalfConeAngle = 2.0f;
        input.mHandGeometry->mTwistMinimum = -1.6f;
        input.mHandGeometry->mTwistMaximum = 1.6f;
        input.mHandRotation = JPH::Quat(-0.360717654f, 0.551108181f, 0.646835625f, 0.384403676f).Normalized();
        input.mHandAimOffset = JPH::Vec3(0.0f, 0.0f, 91.532608f);
        auto originalPose = base;
        const auto original = RagdollCore::applyArmTargetPose(originalPose, base, measured, sFrame, input);
        ASSERT_TRUE(original && original->mApplied);
        EXPECT_GT(original->mReachError, 14.0f); // The unchanged roll reproduces restricted reach.
        const auto originalAxis = *input.mHandRotation * input.mHandAimOffset;
        ASSERT_TRUE(RagdollCore::preferOppositeArmToolRoll(base, base, measured, sFrame, input));
        input.mHandRotation = (*input.mHandRotation
            * JPH::Quat::sRotation(input.mHandAimOffset.Normalized(), 3.14159265359f)).Normalized();
        EXPECT_LT((*input.mHandRotation * input.mHandAimOffset - originalAxis).Length(), 1e-4f);
        EXPECT_FALSE(RagdollCore::preferOppositeArmToolRoll(base, base, measured, sFrame, input));
        auto pose = base;
        const auto result = RagdollCore::applyArmTargetPose(pose, base, measured, sFrame, input);
        ASSERT_TRUE(result && result->mApplied);
        const auto delta = (measured[sFrame].mRotation * base[sFrame].mRotation.Conjugated()).Normalized();
        const auto reachedHand = measured[sShoulder].mTranslation
            + delta * (pose[sHand].mTranslation - base[sShoulder].mTranslation);
        const auto reachedTip = reachedHand + delta * pose[sHand].mRotation * input.mHandAimOffset;
        const auto desiredTip = input.mTargetHandPosition + *input.mHandRotation * input.mHandAimOffset;
        RecordProperty("hand_error", result->mReachError);
        RecordProperty("tip_error", (reachedTip - desiredTip).Length());
        EXPECT_LT(result->mReachError, 2.0f);
        EXPECT_LT((reachedTip - desiredTip).Length(), 2.0f);
        const auto legal = RagdollCore::clampSwingTwistJointOrientation(
            *input.mHandGeometry, pose[sElbow].mRotation, pose[sHand].mRotation);
        ASSERT_TRUE(legal);
        EXPECT_GT(std::abs(legal->Dot(pose[sHand].mRotation)), 0.99999f);
        EXPECT_NEAR((pose[sElbow].mTranslation - pose[sShoulder].mTranslation).Length(),
            input.mHingeGeometry->mUpperOffset.Length(), 1e-3f);
        EXPECT_NEAR((pose[sHand].mTranslation - pose[sElbow].mTranslation).Length(),
            input.mHingeGeometry->mLowerOffset.Length(), 1e-3f);
    }

    TEST(RagdollCoreArmTargetTest, solvesAgainstTheMeasuredFrameRatherThanTheBindPose)
    {
        const auto base = restArm();
        // The body has turned a quarter circle since the base pose was taken.
        const auto turn = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.5f * 3.14159265f);
        auto measured = base;
        for (const std::size_t body : { sFrame, sShoulder, sElbow, sHand })
        {
            measured[body].mTranslation = turn * base[body].mTranslation;
            measured[body].mRotation = (turn * base[body].mRotation).Normalized();
        }
        auto target = base;
        auto input = armInput();
        // Ask in the measured frame: the same reach, turned with the body.
        const JPH::Vec3 wanted(14.0f, 6.0f, 48.0f);
        input.mTargetHandPosition = turn * wanted;
        input.mPreferredBendDirection = turn * JPH::Vec3::sAxisY();
        const auto result = RagdollCore::applyArmTargetPose(target, base, measured, sFrame, input);
        ASSERT_TRUE(result);
        EXPECT_TRUE(result->mApplied);
        // The written pose is a base-frame pose, so it must land on the
        // unturned point. Solving against the bind pose would miss by the turn.
        EXPECT_TRUE(target[sHand].mTranslation.IsClose(wanted, 1e-2f));
    }

    TEST(RagdollCoreArmTargetTest, rejectsDegenerateChainsWithoutEditingThePose)
    {
        const auto base = restArm();
        auto input = armInput();
        input.mTargetHandPosition = JPH::Vec3(14.0f, 6.0f, 48.0f);

        const auto unchanged = [&](const ArmTargetInput& broken, std::size_t frame) {
            auto target = base;
            const auto result = RagdollCore::applyArmTargetPose(target, base, base, frame, broken);
            for (std::size_t i = 0; i < target.size(); ++i)
            {
                EXPECT_TRUE(target[i].mTranslation.IsClose(base[i].mTranslation));
                EXPECT_GT(std::abs(target[i].mRotation.Dot(base[i].mRotation)), 0.999f);
            }
            return result.has_value();
        };

        auto outOfRange = input;
        outOfRange.mHandBodyIndex = 9;
        EXPECT_FALSE(unchanged(outOfRange, sFrame));

        auto duplicated = input;
        duplicated.mElbowBodyIndex = sShoulder;
        EXPECT_FALSE(unchanged(duplicated, sFrame));

        // The frame body may not also be part of the chain it frames.
        EXPECT_FALSE(unchanged(input, sShoulder));

        auto infinite = input;
        infinite.mTargetHandPosition = JPH::Vec3(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);
        EXPECT_FALSE(unchanged(infinite, sFrame));

        auto unnormalized = input;
        unnormalized.mHandRotation = JPH::Quat(0.0f, 0.0f, 0.0f, 0.0f);
        EXPECT_FALSE(unchanged(unnormalized, sFrame));
    }

    TEST(RagdollCoreArmTargetTest, reportsBothEndsOfTheBandTheElbowCanHold)
    {
        using RagdollCore::HingeLegGeometry;
        HingeLegGeometry geometry;
        geometry.mUpperOffset = JPH::Vec3(0.0f, 0.0f, -18.0f);
        geometry.mLowerOffset = JPH::Vec3(0.0f, 0.0f, -17.0f);
        geometry.mHingeAxis = JPH::Vec3::sAxisX();
        geometry.mMinimumAngle = -0.15f;
        geometry.mMaximumAngle = 2.0f;
        const auto range = RagdollCore::hingeChainReachRange(geometry);
        ASSERT_TRUE(range);
        // A hinge has a near end as well as a far one. Missing that is what
        // let a brace aim a hand under a shoulder lying on the floor, inside
        // a fold the joint cannot close to.
        EXPECT_GT(range->first, 1.0f);
        EXPECT_LT(range->first, range->second);
        EXPECT_LE(range->second, 35.0f);

        // Every angle the joint allows must land inside the reported band,
        // and the solver must agree with it.
        for (float angle = geometry.mMinimumAngle; angle <= geometry.mMaximumAngle; angle += 0.05f)
        {
            const auto reach
                = geometry.mUpperOffset + JPH::Quat::sRotation(geometry.mHingeAxis, angle) * geometry.mLowerOffset;
            EXPECT_GE(reach.Length(), range->first - 1e-2f);
            EXPECT_LE(reach.Length(), range->second + 1e-2f);
        }

        // A joint free to travel a full turn reaches from folded to straight.
        HingeLegGeometry free = geometry;
        free.mMinimumAngle = -3.2f;
        free.mMaximumAngle = 3.2f;
        const auto full = RagdollCore::hingeChainReachRange(free);
        ASSERT_TRUE(full);
        EXPECT_NEAR(full->first, 1.0f, 1e-2f);
        EXPECT_NEAR(full->second, 35.0f, 1e-2f);
    }

    TEST(RagdollCoreArmTargetTest, honoursTheElbowHingeLimitsWhenGivenNativeGeometry)
    {
        using RagdollCore::HingeLegGeometry;
        const auto base = restArm();
        // The humanoid profile's elbow: a hinge that bends one way only.
        HingeLegGeometry geometry;
        geometry.mUpperOffset = base[sElbow].mTranslation - base[sShoulder].mTranslation;
        geometry.mLowerOffset = base[sHand].mTranslation - base[sElbow].mTranslation;
        geometry.mHingeAxis = JPH::Vec3::sAxisX();
        geometry.mMinimumAngle = -0.15f;
        geometry.mMaximumAngle = 2.0f;

        auto input = armInput();
        input.mHingeGeometry = geometry;
        // Reachable only by a joint angle the hinge does not have: the hand
        // folded back up beside its own shoulder on the wrong side.
        input.mTargetHandPosition = base[sShoulder].mTranslation + JPH::Vec3(0.0f, 0.0f, 6.0f);
        auto target = base;
        const auto result = RagdollCore::applyArmTargetPose(target, base, base, sFrame, input);
        ASSERT_TRUE(result);
        EXPECT_TRUE(result->mApplied);
        // The hinge says how far it got rather than inventing an angle the
        // constraint would fight on every step.
        EXPECT_GT(result->mReachError, 1.0f);
        // Both halves of the physical hinge keep a common axis.
        const auto upperAxis = target[sShoulder].mRotation * geometry.mHingeAxis;
        const auto lowerAxis = target[sElbow].mRotation * geometry.mHingeAxis;
        EXPECT_GT(upperAxis.Dot(lowerAxis), 0.999f);
    }
    TEST(RagdollCoreArmTargetTest, unreachableHandTargetRespectsTheShoulderConeWithoutStretching)
    {
        const auto base = restArm();
        auto pose = base;
        auto input = armInput();
        input.mTargetHandPosition = JPH::Vec3(6.0f, 15.0f, 60.0f);
        input.mShoulderGeometry.emplace();
        input.mShoulderGeometry->mNormalHalfConeAngle = 0.3f;
        input.mShoulderGeometry->mPlaneHalfConeAngle = 0.3f;
        input.mShoulderGeometry->mTwistMinimum = -0.3f;
        input.mShoulderGeometry->mTwistMaximum = 0.3f;
        const auto result = RagdollCore::applyArmTargetPose(pose, base, base, sFrame, input);
        ASSERT_TRUE(result);
        ASSERT_TRUE(result->mApplied);
        const auto clamped = RagdollCore::clampSwingTwistJointOrientation(*input.mShoulderGeometry,
            base[sFrame].mRotation, pose[sShoulder].mRotation);
        ASSERT_TRUE(clamped);
        EXPECT_GT(std::abs(clamped->Dot(pose[sShoulder].mRotation)), 0.99999f);
        EXPECT_GT(result->mReachError, 1.0f);
        EXPECT_NEAR((pose[sElbow].mTranslation - pose[sShoulder].mTranslation).Length(), 12.0f, 1e-3f);
        EXPECT_NEAR((pose[sHand].mTranslation - pose[sElbow].mTranslation).Length(), 12.0f, 1e-3f);
        EXPECT_NEAR(result->mReachError, (pose[sHand].mTranslation - input.mTargetHandPosition).Length(), 1e-4f);
    }

    TEST(RagdollCoreArmTargetTest, rollContinuityKeepsThePreviousArmRollForTheSameHandPoint)
    {
        // A hand point leaves the roll about the shoulder-to-hand ray free.
        // With only the tiny elbow-pole preference to choose it, a changed
        // pole flips the whole arm; the continuity term keeps the previous
        // roll unless another roll reaches the point clearly better.
        const auto base = restArm();
        auto input = armInput();
        input.mShoulderGeometry.emplace();
        input.mShoulderGeometry->mNormalHalfConeAngle = 3.0f;
        input.mShoulderGeometry->mPlaneHalfConeAngle = 3.0f;
        input.mShoulderGeometry->mTwistMinimum = -3.0f;
        input.mShoulderGeometry->mTwistMaximum = 3.0f;
        input.mTargetHandPosition = JPH::Vec3(6.0f, 14.0f, 50.0f);
        input.mPreferredBendDirection = JPH::Vec3::sAxisX();
        auto first = base;
        const auto seeded = RagdollCore::applyArmTargetPose(first, base, base, sFrame, input);
        ASSERT_TRUE(seeded && seeded->mApplied && seeded->mElbowPosition);
        EXPECT_LT(seeded->mReachError, 0.05f);

        input.mPreferredBendDirection = -JPH::Vec3::sAxisX();
        auto flipped = base;
        const auto free = RagdollCore::applyArmTargetPose(flipped, base, base, sFrame, input);
        ASSERT_TRUE(free && free->mApplied && free->mElbowPosition);
        EXPECT_GT((*free->mElbowPosition - *seeded->mElbowPosition).Length(), 5.0f);

        input.mPreviousElbowPosition = seeded->mElbowPosition;
        input.mRollContinuityWeight = 0.25f;
        auto kept = base;
        const auto continuous = RagdollCore::applyArmTargetPose(kept, base, base, sFrame, input);
        ASSERT_TRUE(continuous && continuous->mApplied && continuous->mElbowPosition);
        EXPECT_LT((*continuous->mElbowPosition - *seeded->mElbowPosition).Length(), 0.5f);
        EXPECT_LT(continuous->mReachError, 0.05f);
    }

    TEST(RagdollCoreArmTargetTest, twistContinuityKeepsANearlyStraightArmFromSpinningItsForearm)
    {
        // Near full reach the elbow lies almost on the shoulder-to-hand ray,
        // so its position cannot tell rolls apart while the forearm (and a
        // held blade) still spins with them.
        const auto base = restArm();
        auto input = armInput();
        input.mShoulderGeometry.emplace();
        input.mShoulderGeometry->mNormalHalfConeAngle = 3.0f;
        input.mShoulderGeometry->mPlaneHalfConeAngle = 3.0f;
        input.mShoulderGeometry->mTwistMinimum = -3.0f;
        input.mShoulderGeometry->mTwistMaximum = 3.0f;
        input.mTargetHandPosition = JPH::Vec3(6.0f, 23.5f, 58.0f);
        input.mPreferredBendDirection = JPH::Vec3::sAxisX();
        auto first = base;
        const auto seeded = RagdollCore::applyArmTargetPose(first, base, base, sFrame, input);
        ASSERT_TRUE(seeded && seeded->mApplied && seeded->mElbowRotation && seeded->mElbowPosition);

        input.mPreferredBendDirection = -JPH::Vec3::sAxisX();
        auto flipped = base;
        const auto free = RagdollCore::applyArmTargetPose(flipped, base, base, sFrame, input);
        ASSERT_TRUE(free && free->mApplied && free->mElbowRotation);
        const float freeDot = std::abs(free->mElbowRotation->Dot(*seeded->mElbowRotation));
        input.mPreviousElbowPosition = seeded->mElbowPosition;
        input.mPreviousElbowRotation = seeded->mElbowRotation;
        input.mRollContinuityWeight = 0.25f;
        input.mTwistContinuityWeight = 80.0f;
        auto kept = base;
        const auto continuous = RagdollCore::applyArmTargetPose(kept, base, base, sFrame, input);
        ASSERT_TRUE(continuous && continuous->mApplied && continuous->mElbowRotation);
        // Two-bone IK still re-derives the upper arm's twist, so the forearm
        // cannot return exactly; it must stay far closer than a free solve.
        const float keptDot = std::abs(continuous->mElbowRotation->Dot(*seeded->mElbowRotation));
        EXPECT_GT(keptDot, 0.97f);
        EXPECT_GT(keptDot, freeDot);
        EXPECT_LT(continuous->mReachError, 0.1f);
    }

    TEST(RagdollCoreArmTargetTest, shoulderRollFindsAReachablePoseInsteadOfClippingThePreferredElbowPlane)
    {
        const auto base = restArm();
        auto pose = base;
        auto input = armInput();
        input.mHingeGeometry.emplace();
        input.mHingeGeometry->mUpperOffset = JPH::Vec3(0, 0, -12);
        input.mHingeGeometry->mLowerOffset = JPH::Vec3(0, 0, -12);
        input.mHingeGeometry->mHingeAxis = JPH::Vec3::sAxisX();
        input.mHingeGeometry->mLowerRotation = JPH::Quat::sIdentity();
        input.mHingeGeometry->mMinimumAngle = 0.0f;
        input.mHingeGeometry->mMaximumAngle = 2.0f;
        input.mShoulderGeometry.emplace();
        input.mShoulderGeometry->mNormalHalfConeAngle = 0.4f;
        input.mShoulderGeometry->mPlaneHalfConeAngle = 0.4f;
        input.mShoulderGeometry->mTwistMinimum = -0.6f;
        input.mShoulderGeometry->mTwistMaximum = 0.6f;
        // A constructive witness inside both joint limits.
        const auto shoulder = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.3f);
        const auto elbow = shoulder * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 1.4f);
        input.mTargetHandPosition = base[sShoulder].mTranslation
            + shoulder * JPH::Vec3(0, 0, -12) + elbow * JPH::Vec3(0, 0, -12);
        input.mPreferredBendDirection = JPH::Vec3::sAxisX();
        const auto result = RagdollCore::applyArmTargetPose(pose, base, base, sFrame, input);
        ASSERT_TRUE(result);
        ASSERT_TRUE(result->mApplied);
        EXPECT_LT(result->mReachError, 0.03f);
        const auto clamped = RagdollCore::clampSwingTwistJointOrientation(*input.mShoulderGeometry,
            base[sFrame].mRotation, pose[sShoulder].mRotation);
        ASSERT_TRUE(clamped);
        EXPECT_GT(std::abs(clamped->Dot(pose[sShoulder].mRotation)), 0.99999f);
    }

    TEST(RagdollCoreArmTargetTest, nativeRestrictedShoulderDoesNotFreezeTheElbowAtRadialSolution)
    {
        // Native player geometry and measured frame, captured in run
        // 20260908T055545501Z. Isolate the arm from the reference pose;
        // this is geometric evidence, not a replay of the whole live step.
        std::vector<Transform> base(4), measured(4);
        auto input = armInput();
        base[0].mTranslation = JPH::Vec3(2.34692168f, 1.11245894f, 114.29895f);
        base[0].mRotation = JPH::Quat(0.969259501f, -0.0189715419f, -0.244340584f, 0.0217654128f).Normalized();
        measured[0].mTranslation = JPH::Vec3(5.47173929f, 1.26754761f, 105.243294f);
        measured[0].mRotation = JPH::Quat(0.954907894f, -0.0636865571f, -0.281967282f, 0.067745842f).Normalized();
        base[1].mTranslation = JPH::Vec3(10.7596693f, 0.659129262f, 109.778053f);
        base[1].mRotation = JPH::Quat(0.748468637f, 0.0815439224f, -0.629238009f, 0.192885235f).Normalized();
        measured[1].mTranslation = JPH::Vec3(13.4632673f, -0.270339042f, 100.08326f);
        measured[1].mRotation = JPH::Quat(0.733915925f, 0.149907053f, -0.639826536f, 0.171806917f).Normalized();
        base[2].mTranslation = JPH::Vec3(14.2690487f, -1.51464713f, 92.2440186f);
        base[2].mRotation = JPH::Quat(0.736194909f, 0.157707796f, -0.645673037f, 0.127481803f).Normalized();
        measured[2].mTranslation = JPH::Vec3(15.9896898f, -0.276752383f, 82.1683426f);
        measured[2].mRotation = JPH::Quat(0.700551987f, 0.110677846f, -0.676595271f, 0.197980359f).Normalized();
        base[3].mTranslation = JPH::Vec3(16.2415447f, -0.370054126f, 75.4624557f);
        base[3].mRotation = JPH::Quat(0.596856773f, -0.368521184f, -0.584683299f, -0.407553166f).Normalized();
        measured[3].mTranslation = JPH::Vec3(16.9427891f, -2.19624233f, 65.2360535f);
        measured[3].mRotation = JPH::Quat(0.623758197f, -0.41079247f, -0.574894369f, -0.334173352f).Normalized();
        input.mTargetHandPosition = JPH::Vec3(19.4632664f, 17.7296619f, 92.0832596f);
        input.mPreferredBendDirection = JPH::Vec3(0.330350429f, -3.5527141e-15f, -0.943858385f);
        input.mHingeGeometry.emplace();
        input.mHingeGeometry->mUpperOffset = JPH::Vec3(18.0134258f, 1.43051147e-06f, 0.0f);
        input.mHingeGeometry->mLowerOffset = JPH::Vec3(16.5662155f, -3.51879501f, 3.81469727e-06f);
        input.mHingeGeometry->mLowerRotation = JPH::Quat(1.4901163e-08f, 2.04890984e-08f, -0.104457662f, 0.994529366f);
        input.mHingeGeometry->mHingeAxis = JPH::Vec3(-2.49964192e-08f, 0.372773618f, -0.927922428f);
        input.mHingeGeometry->mMinimumAngle = -2.0f;
        input.mHingeGeometry->mMaximumAngle = 0.15f;
        input.mShoulderGeometry.emplace();
        input.mShoulderGeometry->mParentFrame = JPH::Quat(-0.689670026f, 3.34395658e-08f, 7.06591097e-09f, 0.724123836f).Normalized();
        input.mShoulderGeometry->mChildFrame = JPH::Quat(-0.454300523f, 0.276171118f, 0.34638688f, 0.772888541f).Normalized();
        input.mShoulderGeometry->mNormalHalfConeAngle = 1.75f;
        input.mShoulderGeometry->mPlaneHalfConeAngle = 1.55f;
        input.mShoulderGeometry->mTwistMinimum = -1.05f;
        input.mShoulderGeometry->mTwistMaximum = 1.05f;
        auto pose = base;
        const auto result = RagdollCore::applyArmTargetPose(pose, base, measured, sFrame, input);
        ASSERT_TRUE(result);
        ASSERT_TRUE(result->mApplied);
        RecordProperty("native_reach_error", result->mReachError);
        // Before the fix: 8.50241. An independent sweep with the elbow
        // fixed at -1.75 found a legal 7.90478-unit witness. Keep that bound
        // fixed rather than defining success using the current optimizer.
        EXPECT_LT(result->mReachError, 7.96f);
        const auto clamped = RagdollCore::clampSwingTwistJointOrientation(*input.mShoulderGeometry,
            base[sFrame].mRotation, pose[sShoulder].mRotation);
        ASSERT_TRUE(clamped);
        EXPECT_GT(std::abs(clamped->Dot(pose[sShoulder].mRotation)), 0.99999f);
        EXPECT_NEAR((pose[sElbow].mTranslation - pose[sShoulder].mTranslation).Length(),
            input.mHingeGeometry->mUpperOffset.Length(), 1e-3f);
        EXPECT_NEAR((pose[sHand].mTranslation - pose[sElbow].mTranslation).Length(),
            input.mHingeGeometry->mLowerOffset.Length(), 1e-3f);
        const auto flexion = (pose[sShoulder].mRotation.Conjugated() * pose[sElbow].mRotation
            * input.mHingeGeometry->mLowerRotation.Conjugated()).Normalized();
        const JPH::Vec3 imaginary(flexion.GetX(), flexion.GetY(), flexion.GetZ());
        const float angle = 2.0f * std::atan2(imaginary.Dot(input.mHingeGeometry->mHingeAxis), flexion.GetW());
        EXPECT_GE(angle, input.mHingeGeometry->mMinimumAngle - 1e-4f);
        EXPECT_LE(angle, input.mHingeGeometry->mMaximumAngle + 1e-4f);
        // A small cardinal stroke from the default guard must mostly lift,
        // including when this native shoulder is already tilted under load.
        const auto delta = (measured[sFrame].mRotation * base[sFrame].mRotation.Conjugated()).Normalized();
        const auto solveOffset = [&](float up) {
            auto trialInput = input;
            trialInput.mTargetHandPosition = measured[sShoulder].mTranslation
                + RagdollCore::WeaponSwingProfile{}.mRestOffset + JPH::Vec3(0, 0, up);
            auto trialPose = base;
            const auto trial = RagdollCore::applyArmTargetPose(trialPose, base, measured, sFrame, trialInput);
            EXPECT_TRUE(trial && trial->mApplied);
            return delta * (trialPose[sHand].mTranslation - base[sShoulder].mTranslation);
        };
        const auto rest = solveOffset(0);
        const auto raised = solveOffset(9);
        const auto lowered = solveOffset(-9);
        EXPECT_GE(raised.GetZ() - rest.GetZ(), 6.0f);
        EXPECT_GE(raised.GetZ() - lowered.GetZ(), 6.0f);
        EXPECT_GE(raised.GetZ() - rest.GetZ(), std::abs(raised.GetX() - rest.GetX()));
        EXPECT_GE(raised.GetZ() - rest.GetZ(), std::abs(raised.GetY() - rest.GetY()));
    }

    TEST(RagdollCoreArmTargetTest, nativeGuardEntryReachDoesNotDependOnElbowSeed)
    {
        std::vector<Transform> base(4), measured(4);
        auto input = armInput();
        base[0].mTranslation = JPH::Vec3(2.34772491f, 1.14313424f, 114.29747f);
        base[0].mRotation = JPH::Quat(0.967760921f, -0.0194666646f, -0.250054628f, 0.0230787378f).Normalized();
        measured[0].mTranslation = JPH::Vec3(4.99938202f, 1.92609978f, 106.236481f);
        measured[0].mRotation = JPH::Quat(0.889770031f, -0.195612267f, -0.396819323f, 0.112158492f).Normalized();
        base[1].mTranslation = JPH::Vec3(10.7060871f, 0.672527254f, 109.678528f);
        base[1].mRotation = JPH::Quat(0.74927789f, 0.081374377f, -0.628296852f, 0.192883044f).Normalized();
        measured[1].mTranslation = JPH::Vec3(10.725523f, -2.55566669f, 99.7828445f);
        measured[1].mRotation = JPH::Quat(0.833514929f, 0.220756441f, -0.456878245f, 0.21859017f).Normalized();
        base[2].mTranslation = JPH::Vec3(14.2591047f, -1.496858f, 92.1527405f);
        base[2].mRotation = JPH::Quat(0.737143457f, 0.157031104f, -0.644634068f, 0.128092706f).Normalized();
        measured[2].mTranslation = JPH::Vec3(19.4883614f, 0.448322684f, 84.1964035f);
        measured[2].mRotation = JPH::Quat(0.799340785f, 0.346380442f, -0.468706638f, 0.14624989f).Normalized();
        base[3].mTranslation = JPH::Vec3(16.2842236f, -0.372946352f, 75.3760452f);
        base[3].mRotation = JPH::Quat(0.597968817f, -0.368308932f, -0.583480299f, -0.407838851f).Normalized();
        measured[3].mTranslation = JPH::Vec3(24.9565868f, 7.52934551f, 69.6713791f);
        measured[3].mRotation = JPH::Quat(0.668308139f, -0.109102145f, -0.595122039f, -0.432770818f).Normalized();
        input.mTargetHandPosition = JPH::Vec3(16.725523f, 27.444334f, 100.92347f);
        input.mPreferredBendDirection = JPH::Vec3(0.330350429f, 0.0f, -0.943858385f);
        input.mHingeGeometry.emplace();
        input.mHingeGeometry->mUpperOffset = JPH::Vec3(18.0134258f, 1.43051147e-06f, 0.0f);
        input.mHingeGeometry->mLowerOffset = JPH::Vec3(16.5662155f, -3.51879454f, 3.81469727e-06f);
        input.mHingeGeometry->mLowerRotation = JPH::Quat(1.49011612e-08f, 0.0f, -0.104457676f, 0.994529366f);
        input.mHingeGeometry->mHingeAxis = JPH::Vec3(-2.49964192e-08f, 0.372773618f, -0.927922428f);
        input.mHingeGeometry->mMinimumAngle = -2.0f;
        input.mHingeGeometry->mMaximumAngle = 0.15f;
        input.mShoulderGeometry.emplace();
        input.mShoulderGeometry->mParentFrame = JPH::Quat(-0.689670026f, 3.34395658e-08f, 7.06591097e-09f, 0.724123836f).Normalized();
        input.mShoulderGeometry->mChildFrame = JPH::Quat(-0.454300523f, 0.276171118f, 0.34638688f, 0.772888541f).Normalized();
        input.mShoulderGeometry->mNormalHalfConeAngle = 1.75f;
        input.mShoulderGeometry->mPlaneHalfConeAngle = 1.55f;
        input.mShoulderGeometry->mTwistMinimum = -1.05f;
        input.mShoulderGeometry->mTwistMaximum = 1.05f;
        // Captured native initial-press pose, run 20260908T102158868Z.
        // Elbow preference may select a different roll, but must not create a
        // large reach discontinuity for this same constrained target/frame.
        float minimumError = std::numeric_limits<float>::max();
        float maximumError = 0.0f;
        for (const auto bend : {input.mPreferredBendDirection, JPH::Vec3::sAxisX(), -JPH::Vec3::sAxisX(),
                 JPH::Vec3::sAxisY(), -JPH::Vec3::sAxisY(), JPH::Vec3::sAxisZ(), -JPH::Vec3::sAxisZ()})
        {
            input.mPreferredBendDirection = bend;
            auto pose = base;
            const auto result = RagdollCore::applyArmTargetPose(pose, base, measured, sFrame, input);
            ASSERT_TRUE(result && result->mApplied);
            minimumError = std::min(minimumError, result->mReachError);
            maximumError = std::max(maximumError, result->mReachError);
            const auto legal = RagdollCore::clampSwingTwistJointOrientation(*input.mShoulderGeometry,
                base[sFrame].mRotation, pose[sShoulder].mRotation);
            ASSERT_TRUE(legal);
            EXPECT_GT(std::abs(legal->Dot(pose[sShoulder].mRotation)), 0.99999f);
        }
        EXPECT_LT(maximumError - minimumError, 0.05f);
        RecordProperty("guard_entry_reach_error", maximumError);
    }

    TEST(RagdollCoreArmTargetTest, consecutiveNativeGuardFramesKeepShoulderPoseContinuous)
    {
        // Consecutive fixed-step inputs 13/14 from native initial press,
        // run 20260908T104235798Z. Reproduce the incoming pose as well as
        // the reference/measurement; starting from base hid the live jump.
        std::vector<Transform> base(4), measured(4), incoming(4);
        auto input = armInput();
        JPH::Quat solved[2];

        // Native sample 13.
        base[0] = Transform{ JPH::Vec3(2.34843874f, 1.1703974f, 114.296082f), JPH::Quat(0.966175675f, -0.0199414175f, -0.255968899f, 0.02422438f).Normalized() };
        measured[0] = Transform{ JPH::Vec3(2.77857041f, 1.5374434f, 106.736336f), JPH::Quat(0.882492959f, -0.254217714f, -0.375170827f, 0.125803217f).Normalized() };
        incoming[0] = Transform{ JPH::Vec3(2.34843874f, 1.1703974f, 114.296082f), JPH::Quat(0.966175675f, -0.0199414175f, -0.255968899f, 0.02422438f).Normalized() };
        base[1] = Transform{ JPH::Vec3(10.6492138f, 0.683391333f, 109.576096f), JPH::Quat(0.750161529f, 0.0813339502f, -0.627297819f, 0.19271718f).Normalized() };
        measured[1] = Transform{ JPH::Vec3(8.31433678f, -3.93648338f, 100.923317f), JPH::Quat(0.868981242f, 0.272304595f, -0.390758246f, 0.134275317f).Normalized() };
        incoming[1] = Transform{ JPH::Vec3(10.6492138f, 0.683391333f, 109.576096f), JPH::Quat(0.750161529f, 0.0813339502f, -0.627297819f, 0.19271718f).Normalized() };
        base[2] = Transform{ JPH::Vec3(14.2476597f, -1.47380531f, 92.0580673f), JPH::Quat(0.738156855f, 0.156467035f, -0.643516719f, 0.128564119f).Normalized() };
        measured[2] = Transform{ JPH::Vec3(18.1696815f, 2.6761148f, 87.2348175f), JPH::Quat(0.823586941f, 0.4088507f, -0.387319356f, 0.0672997981f).Normalized() };
        incoming[2] = Transform{ JPH::Vec3(14.2476597f, -1.47380531f, 92.0580673f), JPH::Quat(0.738156855f, 0.156467035f, -0.643516719f, 0.128564119f).Normalized() };
        base[3] = Transform{ JPH::Vec3(16.3275204f, -0.36403358f, 75.2871323f), JPH::Quat(0.599033475f, -0.367959738f, -0.582307279f, -0.408268183f).Normalized() };
        measured[3] = Transform{ JPH::Vec3(24.400074f, 13.2445755f, 75.383049f), JPH::Quat(-0.631054759f, 0.00857115537f, 0.582358897f, 0.512400448f).Normalized() };
        incoming[3] = Transform{ JPH::Vec3(16.3275204f, -0.36403358f, 75.2871323f), JPH::Quat(0.599033475f, -0.367959738f, -0.582307279f, -0.408268183f).Normalized() };
        input.mTargetHandPosition = JPH::Vec3(14.3143368f, 26.0635166f, 102.063942f);
        input.mPreferredBendDirection = JPH::Vec3(0.330350429f, 0.0f, -0.943858385f);
        input.mHingeGeometry.emplace();
        input.mHingeGeometry->mUpperOffset = JPH::Vec3(18.0134258f, 1.43051147e-06f, 0.0f);
        input.mHingeGeometry->mLowerOffset = JPH::Vec3(16.5662155f, -3.51879454f, 3.81469727e-06f);
        input.mHingeGeometry->mLowerRotation = JPH::Quat(1.49011612e-08f, 0.0f, -0.104457676f, 0.994529366f);
        input.mHingeGeometry->mHingeAxis = JPH::Vec3(-2.49964192e-08f, 0.372773618f, -0.927922428f);
        input.mHingeGeometry->mMinimumAngle = -2.0f;
        input.mHingeGeometry->mMaximumAngle = 0.15f;
        input.mShoulderGeometry.emplace();
        input.mShoulderGeometry->mParentFrame = JPH::Quat(-0.689670026f, 3.34395658e-08f, 7.06591097e-09f, 0.724123836f).Normalized();
        input.mShoulderGeometry->mChildFrame = JPH::Quat(-0.454300523f, 0.276171118f, 0.34638688f, 0.772888541f).Normalized();
        input.mShoulderGeometry->mNormalHalfConeAngle = 1.75f;
        input.mShoulderGeometry->mPlaneHalfConeAngle = 1.55f;
        input.mShoulderGeometry->mTwistMinimum = -1.05f;
        input.mShoulderGeometry->mTwistMaximum = 1.05f;
        {
            auto pose = incoming;
            const auto result = RagdollCore::applyArmTargetPose(pose, base, measured, sFrame, input);
            ASSERT_TRUE(result && result->mApplied);
            EXPECT_LE(result->mReachError, 11.041300f); // Captured reach bound.
            solved[0] = (pose[sFrame].mRotation.Conjugated() * pose[sShoulder].mRotation).Normalized();
        }
        // Native sample 14.
        base[0] = Transform{ JPH::Vec3(2.34846425f, 1.17137098f, 114.296028f), JPH::Quat(0.966062486f, -0.0199639387f, -0.256390691f, 0.0242608506f).Normalized() };
        measured[0] = Transform{ JPH::Vec3(2.80634212f, 1.4295522f, 106.754105f), JPH::Quat(0.881786883f, -0.26497829f, -0.368691564f, 0.127691135f).Normalized() };
        incoming[0] = Transform{ JPH::Vec3(2.34846425f, 1.17137098f, 114.296028f), JPH::Quat(0.966062486f, -0.0199639387f, -0.256390691f, 0.0242608506f).Normalized() };
        base[1] = Transform{ JPH::Vec3(10.6450891f, 0.68361789f, 109.568832f), JPH::Quat(0.750234425f, 0.0813601166f, -0.627221704f, 0.19267033f).Normalized() };
        measured[1] = Transform{ JPH::Vec3(8.35264969f, -4.184937f, 101.097359f), JPH::Quat(0.87456274f, 0.288968593f, -0.371990889f, 0.115151323f).Normalized() };
        incoming[1] = Transform{ JPH::Vec3(10.6450891f, 0.68361789f, 109.568832f), JPH::Quat(0.750234425f, 0.0813601166f, -0.627221704f, 0.19267033f).Normalized() };
        base[2] = Transform{ JPH::Vec3(14.2468224f, -1.471071f, 92.0511703f), JPH::Quat(0.73823601f, 0.15645656f, -0.643428564f, 0.128563374f).Normalized() };
        measured[2] = Transform{ JPH::Vec3(18.3986263f, 3.36026478f, 88.0441513f), JPH::Quat(0.825772822f, 0.426751345f, -0.365284562f, 0.0504953377f).Normalized() };
        incoming[2] = Transform{ JPH::Vec3(14.2468224f, -1.471071f, 92.0511703f), JPH::Quat(0.73823601f, 0.15645656f, -0.643428564f, 0.128563374f).Normalized() };
        base[3] = Transform{ JPH::Vec3(16.3306332f, -0.360742062f, 75.2807617f), JPH::Quat(0.599091053f, -0.367907077f, -0.582239807f, -0.408327281f).Normalized() };
        measured[3] = Transform{ JPH::Vec3(24.6936302f, 14.7198696f, 76.9867401f), JPH::Quat(-0.621206939f, -0.0198684204f, 0.579987824f, 0.526613057f).Normalized() };
        incoming[3] = Transform{ JPH::Vec3(16.3306332f, -0.360742062f, 75.2807617f), JPH::Quat(0.599091053f, -0.367907077f, -0.582239807f, -0.408327281f).Normalized() };
        input.mTargetHandPosition = JPH::Vec3(14.3526497f, 25.8150635f, 102.237984f);
        input.mPreferredBendDirection = JPH::Vec3(0.330350429f, 0.0f, -0.943858385f);
        input.mHingeGeometry.emplace();
        input.mHingeGeometry->mUpperOffset = JPH::Vec3(18.0134258f, 1.43051147e-06f, 0.0f);
        input.mHingeGeometry->mLowerOffset = JPH::Vec3(16.5662155f, -3.51879454f, 3.81469727e-06f);
        input.mHingeGeometry->mLowerRotation = JPH::Quat(1.49011612e-08f, 0.0f, -0.104457676f, 0.994529366f);
        input.mHingeGeometry->mHingeAxis = JPH::Vec3(-2.49964192e-08f, 0.372773618f, -0.927922428f);
        input.mHingeGeometry->mMinimumAngle = -2.0f;
        input.mHingeGeometry->mMaximumAngle = 0.15f;
        input.mShoulderGeometry.emplace();
        input.mShoulderGeometry->mParentFrame = JPH::Quat(-0.689670026f, 3.34395658e-08f, 7.06591097e-09f, 0.724123836f).Normalized();
        input.mShoulderGeometry->mChildFrame = JPH::Quat(-0.454300523f, 0.276171118f, 0.34638688f, 0.772888541f).Normalized();
        input.mShoulderGeometry->mNormalHalfConeAngle = 1.75f;
        input.mShoulderGeometry->mPlaneHalfConeAngle = 1.55f;
        input.mShoulderGeometry->mTwistMinimum = -1.05f;
        input.mShoulderGeometry->mTwistMaximum = 1.05f;
        {
            auto pose = incoming;
            const auto result = RagdollCore::applyArmTargetPose(pose, base, measured, sFrame, input);
            ASSERT_TRUE(result && result->mApplied);
            EXPECT_LE(result->mReachError, 16.425800f); // Captured reach bound.
            solved[1] = (pose[sFrame].mRotation.Conjugated() * pose[sShoulder].mRotation).Normalized();
        }
        const auto delta = (solved[1] * solved[0].Conjugated()).Normalized();
        const float jump = 2.0f * std::atan2(delta.GetXYZ().Length(), std::abs(delta.GetW()));
        RecordProperty("consecutive_shoulder_jump", jump);
        EXPECT_LT(jump, 0.5f);
    }

}
