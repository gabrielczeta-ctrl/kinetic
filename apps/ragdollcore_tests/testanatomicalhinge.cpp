#include "fixtures.hpp"

#include <ragdollcore/anatomicalhinge.hpp>
#include <ragdollcore/armtarget.hpp>
#include <ragdollcore/builder.hpp>

// clang-format off
#include <Jolt/Physics/Constraints/HingeConstraint.h>
// clang-format on

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

namespace
{
    using RagdollCore::Transform;

    float signedFrameAngle(const RagdollCore::ConstraintProfile& joint,
        const Transform& upper, const Transform& lower)
    {
        const auto axis = upper.mRotation * joint.mParentPlaneAxis.Cross(joint.mParentTwistAxis);
        const auto one = upper.mRotation * joint.mParentTwistAxis;
        const auto two = lower.mRotation * joint.mChildTwistAxis;
        return std::atan2(axis.Dot(one.Cross(two)), one.Dot(two));
    }

    RagdollCore::ConstraintProfile knee()
    {
        RagdollCore::ConstraintProfile joint;
        joint.mType = RagdollCore::ConstraintType::Hinge;
        joint.mDeriveFrameFromSkeleton = true;
        joint.mHingeMinAngle = -2.1f;
        joint.mHingeMaxAngle = 0.12f;
        joint.mMotor.mMaximumTorque = 822994.125f;
        return joint;
    }

    TEST(AnatomicalKneeTest, capturedClaymoreLegsHaveAnatomicalZeroAndFullReach)
    {
        // Raw target origins from native135564, before the measured rear-foot lift.
        const std::array<std::array<JPH::Vec3, 3>, 2> legs{{
            { JPH::Vec3(-5.7325358f, 3.3096671f, 70.4776764f),
              JPH::Vec3(-9.7262974f, 21.9770927f, 44.2395554f),
              JPH::Vec3(-13.9974899f, 17.6541805f, 7.3803992f) },
            { JPH::Vec3(5.7325363f, -3.3096747f, 70.4776764f),
              JPH::Vec3(17.5265541f, -0.1111224f, 40.4188766f),
              JPH::Vec3(8.3800602f, -15.0869846f, 7.4402080f) }
        }};
        for (const auto& points : legs)
        for (float yaw : { 0.0f, 0.8f, 3.0f })
        {
            const auto scene = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), yaw);
            Transform upper{ scene * points[0], scene * JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.7f) };
            Transform lower{ scene * points[1], scene * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.3f) };
            Transform end{ scene * points[2], JPH::Quat::sIdentity() };
            auto joint = knee();
            ASSERT_TRUE(RagdollCore::setAnatomicalHingeFrame(joint, upper, lower, end,
                JPH::Vec3::sZero(), JPH::Vec3::sZero(), -1.0f, scene * JPH::Vec3::sAxisX()));
            const auto u = lower.mTranslation - upper.mTranslation;
            const auto v = end.mTranslation - lower.mTranslation;
            const float bend = std::acos(std::clamp(u.Normalized().Dot(v.Normalized()), -1.0f, 1.0f));
            EXPECT_NEAR(signedFrameAngle(joint, upper, lower), -bend, 1e-5f);
            EXPECT_FLOAT_EQ(joint.mHingeMinAngle, -2.1f);
            EXPECT_FLOAT_EQ(joint.mHingeMaxAngle, 0.12f);
            EXPECT_FLOAT_EQ(joint.mMotor.mMaximumTorque, 822994.125f);
            const auto axis = upper.mRotation * joint.mParentPlaneAxis.Cross(joint.mParentTwistAxis);
            Transform straight = lower;
            straight.mRotation = (JPH::Quat::sRotation(axis, bend) * lower.mRotation).Normalized();
            EXPECT_NEAR(signedFrameAngle(joint, upper, straight), 0.0f, 1e-5f);

            RagdollCore::HingeLegGeometry geometry;
            geometry.mUpperOffset = upper.mRotation.Conjugated() * u;
            geometry.mLowerOffset = upper.mRotation.Conjugated() * v;
            geometry.mLowerRotation = (upper.mRotation.Conjugated() * lower.mRotation).Normalized();
            geometry.mHingeAxis = joint.mParentPlaneAxis.Cross(joint.mParentTwistAxis);
            geometry.mMinimumAngle = joint.mHingeMinAngle + bend;
            geometry.mMaximumAngle = joint.mHingeMaxAngle + bend;
            const auto reach = RagdollCore::hingeChainReachRange(geometry);
            ASSERT_TRUE(reach);
            EXPECT_NEAR(reach->second, u.Length() + v.Length(), 1e-4f);
        }
    }

    TEST(AnatomicalKneeTest, straightPoseUsesRotatingAnatomicalAxis)
    {
        for (float yaw : { 0.0f, 1.3f, 3.0f })
        {
            const auto rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), yaw);
            Transform upper{ JPH::Vec3::sZero(), rotation };
            Transform lower{ rotation * JPH::Vec3(0, 0, -10), rotation };
            Transform end{ rotation * JPH::Vec3(0, 0, -20), rotation };
            auto joint = knee();
            ASSERT_TRUE(RagdollCore::setAnatomicalHingeFrame(joint, upper, lower, end,
                JPH::Vec3::sZero(), JPH::Vec3::sZero(), -1, rotation * JPH::Vec3::sAxisX()));
            EXPECT_NEAR(signedFrameAngle(joint, upper, lower), 0, 1e-6f);
            lower.mRotation = rotation * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.8f);
            EXPECT_NEAR(signedFrameAngle(joint, upper, lower), -0.8f, 1e-5f);
        }
    }

    TEST(AnatomicalKneeTest, invalidDirectionOrStraightAxisDoesNotMutate)
    {
        Transform upper, lower{ JPH::Vec3(0, 0, -10), JPH::Quat::sIdentity() };
        Transform end{ JPH::Vec3(0, 0, -20), JPH::Quat::sIdentity() };
        for (float direction : { 0.0f, 0.5f, std::numeric_limits<float>::quiet_NaN() })
        {
            auto joint = knee();
            EXPECT_FALSE(RagdollCore::setAnatomicalHingeFrame(joint, upper, lower, end,
                JPH::Vec3::sZero(), JPH::Vec3::sZero(), direction, JPH::Vec3::sAxisX()));
            EXPECT_TRUE(joint.mDeriveFrameFromSkeleton);
            EXPECT_TRUE(joint.mParentAnchor.IsNearZero());
        }
        for (const auto& axis : { JPH::Vec3::sZero(), JPH::Vec3::sAxisZ(),
                 JPH::Vec3(std::numeric_limits<float>::quiet_NaN(), 0, 0) })
        {
            auto joint = knee();
            EXPECT_FALSE(RagdollCore::setAnatomicalHingeFrame(joint, upper, lower, end,
                JPH::Vec3::sZero(), JPH::Vec3::sZero(), -1, axis));
            EXPECT_TRUE(joint.mDeriveFrameFromSkeleton);
        }
    }

    TEST(AnatomicalKneeTest, explicitFrameIsNotReauthored)
    {
        auto joint = knee();
        joint.mDeriveFrameFromSkeleton = false;
        Transform upper, lower{ JPH::Vec3(0, 0, -10), JPH::Quat::sIdentity() };
        Transform end{ JPH::Vec3(0, -7, -17), JPH::Quat::sIdentity() };
        EXPECT_FALSE(RagdollCore::setAnatomicalHingeFrame(joint, upper, lower, end,
            JPH::Vec3::sZero(), JPH::Vec3::sZero(), -1, JPH::Vec3::sAxisX()));
        EXPECT_TRUE(joint.mParentAnchor.IsNearZero());
        EXPECT_TRUE(joint.mParentTwistAxis.IsClose(JPH::Vec3::sAxisX()));
    }
}
