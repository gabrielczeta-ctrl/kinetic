#include <gtest/gtest.h>

#include <ragdollcore/weaponalignment.hpp>

#include <cmath>

namespace
{
    bool nearlyEqual(JPH::Vec3Arg left, JPH::Vec3Arg right, float tolerance = 1e-4f)
    {
        return static_cast<float>((left - right).Length()) <= tolerance;
    }
}

TEST(RagdollCoreWeaponAlignmentTest, longAxisFollowsTheStrikeDirection)
{
    const RagdollCore::WeaponStrikeAxes axes{ JPH::Vec3::sAxisZ(), JPH::Vec3::sAxisX() };
    const auto rotation = RagdollCore::alignedStrikeHandRotation(axes, JPH::Vec3::sAxisY(), JPH::Vec3::sZero());
    ASSERT_TRUE(rotation.has_value());
    EXPECT_TRUE(nearlyEqual(*rotation * axes.mLongAxis, JPH::Vec3::sAxisY()));
}

TEST(RagdollCoreWeaponAlignmentTest, edgeRollsTowardTheSwingTangent)
{
    const RagdollCore::WeaponStrikeAxes axes{ JPH::Vec3::sAxisZ(), JPH::Vec3::sAxisX() };
    const auto rotation
        = RagdollCore::alignedStrikeHandRotation(axes, JPH::Vec3::sAxisY(), JPH::Vec3::sAxisZ());
    ASSERT_TRUE(rotation.has_value());
    EXPECT_TRUE(nearlyEqual(*rotation * axes.mLongAxis, JPH::Vec3::sAxisY()));
    EXPECT_TRUE(nearlyEqual(*rotation * axes.mEdgeAxis, JPH::Vec3::sAxisZ()));
}

TEST(RagdollCoreWeaponAlignmentTest, parallelTangentKeepsTheLongAxisRoll)
{
    const RagdollCore::WeaponStrikeAxes axes{ JPH::Vec3::sAxisZ(), JPH::Vec3::sAxisX() };
    const auto rotation
        = RagdollCore::alignedStrikeHandRotation(axes, JPH::Vec3::sAxisY(), JPH::Vec3::sAxisY());
    ASSERT_TRUE(rotation.has_value());
    EXPECT_TRUE(nearlyEqual(*rotation * axes.mLongAxis, JPH::Vec3::sAxisY()));
    // The unchanged roll keeps the edge perpendicular to the strike.
    EXPECT_NEAR(static_cast<float>((*rotation * axes.mEdgeAxis).Dot(JPH::Vec3::sAxisY())), 0.0f, 1e-4f);
}

TEST(RagdollCoreWeaponAlignmentTest, missingEdgeStillAlignsTheLongAxis)
{
    const RagdollCore::WeaponStrikeAxes axes{ JPH::Vec3::sAxisZ(), JPH::Vec3::sZero() };
    const auto rotation
        = RagdollCore::alignedStrikeHandRotation(axes, JPH::Vec3::sAxisX(), JPH::Vec3::sAxisZ());
    ASSERT_TRUE(rotation.has_value());
    EXPECT_TRUE(nearlyEqual(*rotation * axes.mLongAxis, JPH::Vec3::sAxisX()));
}

TEST(RagdollCoreWeaponAlignmentTest, degenerateInputIsRejected)
{
    const RagdollCore::WeaponStrikeAxes axes{ JPH::Vec3::sAxisZ(), JPH::Vec3::sAxisX() };
    EXPECT_FALSE(RagdollCore::alignedStrikeHandRotation(axes, JPH::Vec3::sZero(), JPH::Vec3::sAxisX()).has_value());
    EXPECT_FALSE(
        RagdollCore::alignedStrikeHandRotation({ JPH::Vec3::sZero(), JPH::Vec3::sAxisX() },
            JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX())
            .has_value());
}

TEST(RagdollCoreWeaponAlignmentTest, claymoreAxeAndHammerHaveAuthoredProfiles)
{
    for (const std::string_view record : { "iron claymore", "iron battle axe", "iron warhammer" })
    {
        const auto profile = RagdollCore::weaponShapeProfile(record);
        ASSERT_TRUE(profile.has_value()) << record;
        EXPECT_NEAR(profile->mStrikeAxis.Length(), 1.0f, 1e-6f);
        EXPECT_NEAR(profile->mEdgeAxis.Length(), 1.0f, 1e-6f);
        EXPECT_GT(profile->mStrikeStart, 0.0f);
        EXPECT_LT(profile->mStrikeStart, 1.0f);
    }
    EXPECT_FALSE(RagdollCore::weaponShapeProfile("iron dagger").has_value());
}

TEST(RagdollCoreWeaponAlignmentTest, opposingEdgesDoNotHalfTurnOnReversalOrLoseRollAtRest)
{
    const RagdollCore::WeaponStrikeAxes axes{ JPH::Vec3::sAxisZ(), JPH::Vec3::sAxisX() };
    // Tilted/yawed aim exercises the same world frame as the native adapter.
    const auto forward = JPH::Vec3(1, 2, 1).Normalized();
    const auto tangent = forward.Cross(JPH::Vec3::sAxisZ()).Normalized();
    const auto first = *RagdollCore::alignedStrikeHandRotation(axes, forward, tangent);
    const auto oldReverse = *RagdollCore::alignedStrikeHandRotation(axes, forward, -tangent);
    EXPECT_NEAR((first * axes.mEdgeAxis).Dot(oldReverse * axes.mEdgeAxis), -1.0f, 1e-5f);
    auto previous = first;
    for (const auto motion : { tangent, -tangent, JPH::Vec3::sZero(), forward, tangent * 0.001f })
    {
        const auto next = RagdollCore::continuousStrikeHandRotation(axes, forward, motion, previous);
        ASSERT_TRUE(next);
        EXPECT_TRUE(nearlyEqual(*next * axes.mLongAxis, forward));
        EXPECT_TRUE(nearlyEqual(*next * axes.mEdgeAxis, first * axes.mEdgeAxis));
        previous = *next;
    }
}

TEST(RagdollCoreWeaponAlignmentTest, aSpearHeldAtItsMiddleKeepsItsHeadForward)
{
    // The 11-06 playthrough's proxies: centre along the long axis, half length.
    EXPECT_EQ(RagdollCore::weaponTipSign(-1.30322f, 68.8168f, 0.2f), 1.0f); // steel spear
    EXPECT_EQ(RagdollCore::weaponTipSign(-7.21472f, 61.047f, 0.2f), 1.0f); // steel staff of divine judgement
    EXPECT_EQ(RagdollCore::weaponTipSign(2.33f, 53.11f, 0.2f), 1.0f); // iron spear
    EXPECT_EQ(RagdollCore::weaponTipSign(33.59f, 57.95f, 0.2f), 1.0f); // iron claymore
    // A grip well off the centre still decides, either way.
    EXPECT_EQ(RagdollCore::weaponTipSign(-30.0f, 57.0f, 0.2f), -1.0f);
    // Share 0: the centre's sign alone (the old rule).
    EXPECT_EQ(RagdollCore::weaponTipSign(-1.30322f, 68.8168f, 0.0f), -1.0f);
}
