#include "fixtures.hpp"

#include <ragdollcore/builder.hpp>
#include <ragdollcore/anatomicalhinge.hpp>
#include <ragdollcore/armtarget.hpp>
#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/unitmodel.hpp>
#include <ragdollcore/scenario.hpp>

// clang-format off
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
// clang-format on

#include <gtest/gtest.h>

#include <limits>
#include <cmath>
#include <sstream>

namespace
{
    std::vector<JPH::Vec3> boxHullPoints(JPH::Vec3Arg center, JPH::Vec3Arg halfExtents)
    {
        std::vector<JPH::Vec3> result;
        for (float x : { -1.f, 1.f })
            for (float y : { -1.f, 1.f })
                for (float z : { -1.f, 1.f })
                    result.push_back(center + JPH::Vec3(x, y, z) * halfExtents);
        return result;
    }

    TEST(RagdollCoreBuilderTest, convexHullPreservesScaleTransformAuthoredMassCenterAndJointAnchor)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& body = profile.mBodies[0];
        const JPH::Vec3 pointCenter(3, 5, -2);
        body.mShape.mType = RagdollCore::ShapeType::ConvexHull;
        body.mShape.mPoints = boxHullPoints(pointCenter, JPH::Vec3(1, 2, 3));
        body.mShape.mLocalTransform.mTranslation = JPH::Vec3(4, -2, 1);
        body.mShape.mLocalTransform.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.57079632679f);
        body.mCenterOfMass = JPH::Vec3(-1, 2, 0.5f);
        profile.mBodies[1].mConstraint->mDeriveFrameFromSkeleton = true;
        RagdollCore::BuildOptions options;
        options.mUniformScale = 2;
        const auto result = RagdollCore::buildRagdollSettings(RagdollCoreTest::makeSkeleton(), profile, options);
        ASSERT_TRUE(result.isValid());
        const auto& part = result.mSettings->mParts[0];
        const JPH::Shape* shape = part.GetShape();
        EXPECT_TRUE(shape->GetCenterOfMass().IsClose(JPH::Vec3(-2, 4, 1)));
        EXPECT_TRUE(shape->GetLocalBounds().mMin.IsClose(JPH::Vec3(-4, -4, -9), 1e-6f));
        EXPECT_TRUE(shape->GetLocalBounds().mMax.IsClose(JPH::Vec3(4, 0, 3), 1e-6f));
        EXPECT_FLOAT_EQ(part.GetMassProperties().mMass, options.mTotalMass * body.mMassFraction);

        auto boxProfile = profile;
        auto& box = boxProfile.mBodies[0].mShape;
        box.mType = RagdollCore::ShapeType::Box;
        box.mPoints.clear();
        box.mHalfExtents = JPH::Vec3(1, 2, 3);
        box.mLocalTransform.mTranslation += box.mLocalTransform.mRotation * pointCenter;
        const auto boxResult = RagdollCore::buildRagdollSettings(RagdollCoreTest::makeSkeleton(), boxProfile, options);
        ASSERT_TRUE(boxResult.isValid());
        const auto actualInertia = part.GetMassProperties().mInertia;
        const auto expectedInertia = boxResult.mSettings->mParts[0].GetMassProperties().mInertia;
        for (unsigned row = 0; row < 3; ++row)
            for (unsigned column = 0; column < 3; ++column)
            {
                EXPECT_TRUE(std::isfinite(actualInertia(row, column)));
                EXPECT_NEAR(actualInertia(row, column), expectedInertia(row, column), 0.01f);
            }
        const auto* joint = static_cast<const JPH::SwingTwistConstraintSettings*>(
            result.mSettings->mParts[1].mToParent.GetPtr());
        EXPECT_TRUE(joint->mPosition1.IsClose(JPH::RVec3(2, 16, -1)));
        EXPECT_TRUE(joint->mPosition2.IsNearZero());
    }

    TEST(RagdollCoreBuilderTest, convexHullAvoidsEmptyBoxRegionAndRoundTripsNativeShape)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& shape = profile.mBodies[0].mShape;
        shape.mType = RagdollCore::ShapeType::ConvexHull;
        for (float y : { -50.f, 50.f })
            for (float x : { -1.f, 1.f })
                for (float z : { -1.f, 1.f })
                    shape.mPoints.emplace_back(x * (y < 0 ? 1.f : 9.f), y, z);
        const auto result = RagdollCore::buildRagdollSettings(RagdollCoreTest::makeSkeleton(), profile, {});
        ASSERT_TRUE(result.isValid());
        const JPH::Shape* hull = result.mSettings->mParts[0].GetShape();
        const JPH::RayCast emptySpace(JPH::Vec3(5, -20, -5), JPH::Vec3(0, 0, 10));
        const JPH::RayCast material(JPH::Vec3(0, -20, -5), JPH::Vec3(0, 0, 10));
        JPH::RayCastResult emptyHit;
        JPH::RayCastResult materialHit;
        EXPECT_FALSE(hull->CastRay(emptySpace, {}, emptyHit));
        EXPECT_TRUE(hull->CastRay(material, {}, materialHit));
        auto boxProfile = profile;
        boxProfile.mBodies[0].mShape.mType = RagdollCore::ShapeType::Box;
        boxProfile.mBodies[0].mShape.mPoints.clear();
        boxProfile.mBodies[0].mShape.mHalfExtents = JPH::Vec3(9, 50, 1);
        const auto box = RagdollCore::buildRagdollSettings(RagdollCoreTest::makeSkeleton(), boxProfile, {});
        ASSERT_TRUE(box.isValid());
        JPH::RayCastResult boxHit;
        EXPECT_TRUE(box.mSettings->mParts[0].GetShape()->CastRay(emptySpace, {}, boxHit));

        std::stringstream bytes(std::ios::in | std::ios::out | std::ios::binary);
        JPH::StreamOutWrapper output(bytes);
        JPH::Shape::ShapeToIDMap savedShapes;
        JPH::Shape::MaterialToIDMap savedMaterials;
        hull->SaveWithChildren(output, savedShapes, savedMaterials);
        ASSERT_FALSE(output.IsFailed());
        bytes.seekg(0);
        JPH::StreamInWrapper input(bytes);
        JPH::Shape::IDToShapeMap loadedShapes;
        JPH::Shape::IDToMaterialMap loadedMaterials;
        const auto restored = JPH::Shape::sRestoreWithChildren(input, loadedShapes, loadedMaterials);
        ASSERT_FALSE(restored.HasError());
        EXPECT_TRUE(restored.Get()->GetCenterOfMass().IsClose(hull->GetCenterOfMass()));
        EXPECT_TRUE(restored.Get()->GetLocalBounds().mMin.IsClose(hull->GetLocalBounds().mMin));
        EXPECT_TRUE(restored.Get()->GetLocalBounds().mMax.IsClose(hull->GetLocalBounds().mMax));
        JPH::RayCastResult restoredEmpty;
        JPH::RayCastResult restoredMaterial;
        EXPECT_FALSE(restored.Get()->CastRay(emptySpace, {}, restoredEmpty));
        EXPECT_TRUE(restored.Get()->CastRay(material, {}, restoredMaterial));
    }

    TEST(RagdollCoreBuilderTest, convexHullRejectsScaledPointOverflowWithoutJoltAssertion)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& shape = profile.mBodies[0].mShape;
        shape.mType = RagdollCore::ShapeType::ConvexHull;
        shape.mPoints = boxHullPoints(JPH::Vec3::sZero(), JPH::Vec3::sReplicate(2));
        RagdollCore::BuildOptions options;
        options.mUniformScale = std::numeric_limits<float>::max();
        const auto result = RagdollCore::buildRagdollSettings(RagdollCoreTest::makeSkeleton(), profile, options);
        ASSERT_FALSE(result.isValid());
        ASSERT_FALSE(result.mValidation.mIssues.empty());
        EXPECT_EQ(result.mValidation.mIssues.front().mCode, RagdollCore::ValidationCode::InvalidShape);
    }

    TEST(RagdollCoreBuilderTest, convexHullUsesJoltReducedConvexRadiusWithoutOutwardPadding)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& shape = profile.mBodies[0].mShape;
        shape.mType = RagdollCore::ShapeType::ConvexHull;
        shape.mPoints = boxHullPoints(JPH::Vec3::sZero(), JPH::Vec3(1, 2, 0.02f));
        const auto result = RagdollCore::buildRagdollSettings(RagdollCoreTest::makeSkeleton(), profile, {});
        ASSERT_TRUE(result.isValid());
        const auto* built = result.mSettings->mParts[0].GetShape();
        ASSERT_EQ(built->GetSubType(), JPH::EShapeSubType::ConvexHull);
        const auto* hull = static_cast<const JPH::ConvexHullShape*>(built);
        EXPECT_GE(hull->GetConvexRadius(), 0.f);
        EXPECT_LE(hull->GetConvexRadius(), 0.020001f);
        EXPECT_LT(hull->GetConvexRadius(), JPH::cDefaultConvexRadius);
        EXPECT_TRUE(hull->GetLocalBounds().mMin.IsClose(JPH::Vec3(-1, -2, -0.02f)));
        EXPECT_TRUE(hull->GetLocalBounds().mMax.IsClose(JPH::Vec3(1, 2, 0.02f)));
    }

    TEST(RagdollCoreBuilderTest, combatPoseElbowsKeepFullExtensionAndAuthoredFlexionLimits)
    {
        for (float bend : { 0.0f, 0.6f, 1.5f, 1.9f })
        for (float mirroredRotation : { 0.0f, 3.14159265f })
        {
            SCOPED_TRACE(bend);
            SCOPED_TRACE(mirroredRotation);
            auto skeleton = RagdollCoreTest::makeSkeleton();
            auto& upper = skeleton.mBones[0].mModelTransform;
            auto& lower = skeleton.mBones[1].mModelTransform;
            upper.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), mirroredRotation);
            lower.mTranslation = JPH::Vec3(0, 0, -10);
            lower.mRotation = upper.mRotation;
            RagdollCore::Transform end;
            end.mTranslation = lower.mTranslation
                + JPH::Quat::sRotation(JPH::Vec3::sAxisX(), bend) * JPH::Vec3(0, 0, -10);
            auto profile = RagdollCoreTest::makeProfile();
            auto& joint = *profile.mBodies[1].mConstraint;
            joint.mType = RagdollCore::ConstraintType::Hinge;
            joint.mDeriveFrameFromSkeleton = true;
            joint.mHingeMinAngle = -0.15f;
            joint.mHingeMaxAngle = 2.0f;
            ASSERT_TRUE(RagdollCore::setAnatomicalHingeFrame(joint, upper, lower, end,
                JPH::Vec3::sZero(), JPH::Vec3::sZero()));
            const auto built = RagdollCore::buildRagdollSettings(skeleton, profile, {});
            ASSERT_TRUE(built.isValid());
            const auto* hinge = static_cast<const JPH::HingeConstraintSettings*>(built.mSettings->mParts[1].mToParent.GetPtr());
            const auto axis = upper.mRotation * hinge->mHingeAxis1;
            const auto normal1 = upper.mRotation * hinge->mNormalAxis1;
            const auto normal2 = lower.mRotation * hinge->mNormalAxis2;
            const float reference = std::atan2(axis.Dot(normal1.Cross(normal2)), normal1.Dot(normal2));
            EXPECT_NEAR(reference, bend, 1e-5f);
            EXPECT_FLOAT_EQ(hinge->mLimitsMin, -0.15f);
            EXPECT_FLOAT_EQ(hinge->mLimitsMax, 2.0f);
            RagdollCore::HingeLegGeometry geometry;
            geometry.mUpperOffset = upper.mRotation.Conjugated() * (lower.mTranslation - upper.mTranslation);
            geometry.mLowerOffset = upper.mRotation.Conjugated() * (end.mTranslation - lower.mTranslation);
            geometry.mLowerRotation = (upper.mRotation.Conjugated() * lower.mRotation).Normalized();
            geometry.mHingeAxis = hinge->mHingeAxis1;
            geometry.mMinimumAngle = hinge->mLimitsMin - reference;
            geometry.mMaximumAngle = hinge->mLimitsMax - reference;
            const auto reach = RagdollCore::hingeChainReachRange(geometry);
            ASSERT_TRUE(reach);
            EXPECT_NEAR(reach->second, 20.0f, 1e-4f);
        }
    }

    TEST(RagdollCoreBuilderTest, humanoidShoulderTwistsAlongTheUpperArm)
    {
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto profile = RagdollCore::makeScenarioHumanoidProfile();
        const auto result = RagdollCore::buildRagdollSettings(skeleton, profile, {});
        ASSERT_TRUE(result.isValid());
        for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
        {
            const auto& name = profile.mBodies[i].mName;
            if (name != "left_upper_arm" && name != "right_upper_arm") continue;
            // Body order is parent-before-child; the next part is the elbow.
            ASSERT_LT(i + 1, profile.mBodies.size());
            ASSERT_EQ(profile.mBodies[i + 1].mParentBody, name);
            const auto& shoulder = result.mSettings->mParts[i];
            const auto& elbow = result.mSettings->mParts[i + 1];
            const auto* joint = static_cast<const JPH::SwingTwistConstraintSettings*>(shoulder.mToParent.GetPtr());
            ASSERT_NE(joint, nullptr);
            const auto alongArm = JPH::Vec3(elbow.mPosition - shoulder.mPosition).Normalized();
            EXPECT_GT((shoulder.mRotation * joint->mTwistAxis2).Dot(alongArm), 0.9999f) << name;
        }
    }

    TEST(RagdollCoreBuilderTest, mirroredBoneRotationsDoNotReverseDerivedElbowFlexion)
    {
        auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        // Change local bone orientations without moving their model-space
        // joint origins, as an authored mirrored skeleton does.
        for (auto& bone : skeleton.mBones)
            if (bone.mName == "right_upper_arm" || bone.mName == "right_forearm")
                bone.mModelTransform.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 3.14159265f);
        const auto profile = RagdollCore::makeScenarioHumanoidProfile();
        const auto result = RagdollCore::buildRagdollSettings(skeleton, profile, {});
        ASSERT_TRUE(result.isValid());
        for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
        {
            const auto& name = profile.mBodies[i].mName;
            if (name != "left_forearm" && name != "right_forearm") continue;
            const auto& elbow = result.mSettings->mParts[i];
            const auto* joint = static_cast<const JPH::HingeConstraintSettings*>(elbow.mToParent.GetPtr());
            ASSERT_NE(joint, nullptr);
            EXPECT_LE(joint->mLimitsMin, 0.5f) << name;
            EXPECT_GE(joint->mLimitsMax, 0.5f) << name;
            const auto axis = elbow.mRotation * joint->mHingeAxis2;
            EXPECT_GT(axis.Dot(JPH::Vec3::sAxisX()), 0.9999f) << name;
            const auto lower = JPH::Vec3(result.mSettings->mParts[i + 1].mPosition - elbow.mPosition);
            const auto bent = JPH::Quat::sRotation(axis, 0.5f) * lower;
            EXPECT_GT(bent.GetY(), lower.GetY()) << name;
        }
    }

    TEST(RagdollCoreBuilderTest, buildsJoltSettingsFromValidatedPlainData)
    {
        RagdollCore::BuildOptions options;
        options.mTotalMass = 80.0f;
        options.mObjectLayer = 7;
        const auto result = RagdollCore::buildRagdollSettings(
            RagdollCoreTest::makeSkeleton(), RagdollCoreTest::makeProfile(), options);

        ASSERT_TRUE(result.isValid());
        ASSERT_NE(result.mSettings, nullptr);
        ASSERT_EQ(result.mSettings->mParts.size(), 2u);
        ASSERT_EQ(result.mSettings->GetSkeleton()->GetJointCount(), 2u);
        // Body mass fractions are a physical contract, not suggestions for
        // a solver pass to redistribute between the trunk and extremities.
        const float pelvisMass = result.mSettings->mParts[0].mMassPropertiesOverride.mMass;
        const float torsoMass = result.mSettings->mParts[1].mMassPropertiesOverride.mMass;
        EXPECT_FLOAT_EQ(pelvisMass + torsoMass, 80.0f);
        EXPECT_FLOAT_EQ(pelvisMass, 48.0f);
        EXPECT_FLOAT_EQ(torsoMass, 32.0f);
        for (const auto& part : result.mSettings->mParts)
        {
            auto expected = part.GetShape()->GetMassProperties();
            expected.ScaleToMass(part.mMassPropertiesOverride.mMass);
            EXPECT_TRUE(part.GetMassProperties().mInertia.IsClose(expected.mInertia));
        }
        EXPECT_EQ(result.mSettings->mParts[0].mObjectLayer, 7);
        EXPECT_EQ(result.mSettings->mParts[1].mObjectLayer, 7);
        EXPECT_FLOAT_EQ(result.mSettings->mParts[0].mMaxLinearVelocity,
            RagdollCore::UnitModel::fromMeters(500.0f));
        EXPECT_FLOAT_EQ(result.mSettings->mParts[1].mMaxLinearVelocity,
            RagdollCore::UnitModel::fromMeters(500.0f));
        EXPECT_EQ(result.mSettings->mParts[0].mToParent, nullptr);
        EXPECT_NE(result.mSettings->mParts[1].mToParent, nullptr);
    }

    TEST(RagdollCoreBuilderTest, doesNotConstructFromInvalidOptions)
    {
        RagdollCore::BuildOptions options;
        options.mTotalMass = 0.0f;
        const auto result = RagdollCore::buildRagdollSettings(
            RagdollCoreTest::makeSkeleton(), RagdollCoreTest::makeProfile(), options);
        EXPECT_FALSE(result.isValid());
        EXPECT_EQ(result.mSettings, nullptr);
    }

    TEST(RagdollCoreBuilderTest, rejectsOverflowInDerivedInertiaBeforeCreatingPhysicsBodies)
    {
        RagdollCore::BuildOptions options;
        options.mTotalMass = std::numeric_limits<float>::max();
        const auto result = RagdollCore::buildRagdollSettings(
            RagdollCoreTest::makeSkeleton(), RagdollCoreTest::makeProfile(), options);
        EXPECT_FALSE(result.isValid());
        EXPECT_EQ(result.mSettings, nullptr);
        ASSERT_FALSE(result.mValidation.mIssues.empty());
        EXPECT_EQ(result.mValidation.mIssues.front().mCode, RagdollCore::ValidationCode::InvalidMass);
    }

    TEST(RagdollCoreBuilderTest, rejectsNonNormalizedRootRotation)
    {
        RagdollCore::BuildOptions options;
        options.mRootRotation = JPH::Quat(0.0f, 0.0f, 0.0f, 2.0f);
        const auto result = RagdollCore::buildRagdollSettings(
            RagdollCoreTest::makeSkeleton(), RagdollCoreTest::makeProfile(), options);
        EXPECT_FALSE(result.isValid());
        EXPECT_EQ(result.mSettings, nullptr);
    }

    TEST(RagdollCoreBuilderTest, derivesCoincidentConstraintAnchorsAndFramesFromSkeleton)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[1].mConstraint->mDeriveFrameFromSkeleton = true;

        const auto result = RagdollCore::buildRagdollSettings(RagdollCoreTest::makeSkeleton(), profile, {});
        ASSERT_TRUE(result.isValid());
        const auto* constraint
            = static_cast<const JPH::SwingTwistConstraintSettings*>(result.mSettings->mParts[1].mToParent.GetPtr());
        ASSERT_NE(constraint, nullptr);
        EXPECT_TRUE(constraint->mPosition1.IsClose(JPH::RVec3(0.0f, 10.0f, 0.0f)));
        EXPECT_TRUE(constraint->mPosition2.IsNearZero());
        EXPECT_TRUE(constraint->mTwistAxis1.IsClose(JPH::Vec3::sAxisY()));
        EXPECT_TRUE(constraint->mTwistAxis2.IsClose(JPH::Vec3::sAxisY()));
        EXPECT_TRUE(constraint->mTwistAxis1.IsNormalized());
        EXPECT_TRUE(constraint->mPlaneAxis1.IsNormalized());
        EXPECT_NEAR(constraint->mTwistAxis1.Dot(constraint->mPlaneAxis1), 0.0f, 1e-6f);
    }

    TEST(RagdollCoreBuilderTest, rejectsDerivedFrameForCoincidentBones)
    {
        auto skeleton = RagdollCoreTest::makeSkeleton();
        skeleton.mBones[1].mModelTransform.mTranslation = JPH::Vec3::sZero();
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[1].mConstraint->mDeriveFrameFromSkeleton = true;

        const auto result = RagdollCore::buildRagdollSettings(skeleton, profile, {});
        ASSERT_FALSE(result.isValid());
        ASSERT_EQ(result.mSettings, nullptr);
        ASSERT_EQ(result.mValidation.mIssues.size(), 1u);
        EXPECT_EQ(result.mValidation.mIssues.front().mCode, RagdollCore::ValidationCode::InvalidConstraint);
    }

    TEST(RagdollCoreBuilderTest, derivesAnatomicalHingeFrameAndAsymmetricLimits)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& constraint = *profile.mBodies[1].mConstraint;
        constraint.mType = RagdollCore::ConstraintType::Hinge;
        constraint.mDeriveFrameFromSkeleton = true;
        constraint.mHingeMinAngle = -0.15f;
        constraint.mHingeMaxAngle = 2.0f;

        const auto result = RagdollCore::buildRagdollSettings(RagdollCoreTest::makeSkeleton(), profile, {});
        ASSERT_TRUE(result.isValid());
        const auto* hinge
            = dynamic_cast<const JPH::HingeConstraintSettings*>(result.mSettings->mParts[1].mToParent.GetPtr());
        ASSERT_NE(hinge, nullptr);
        EXPECT_TRUE(hinge->mPoint1.IsClose(JPH::RVec3(0.0f, 10.0f, 0.0f)));
        EXPECT_TRUE(hinge->mPoint2.IsNearZero());
        EXPECT_TRUE(hinge->mHingeAxis1.IsClose(JPH::Vec3::sAxisZ()));
        EXPECT_TRUE(hinge->mHingeAxis2.IsClose(JPH::Vec3::sAxisZ()));
        EXPECT_TRUE(hinge->mNormalAxis1.IsClose(JPH::Vec3::sAxisY()));
        EXPECT_TRUE(hinge->mNormalAxis2.IsClose(JPH::Vec3::sAxisY()));
        EXPECT_FLOAT_EQ(hinge->mLimitsMin, -0.15f);
        EXPECT_FLOAT_EQ(hinge->mLimitsMax, 2.0f);
    }

    TEST(RagdollCoreBuilderTest, configuresSymmetricBoundedJointMotors)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& authored = profile.mBodies[1].mConstraint->mMotor;
        authored.mFrequency = 6.0f;
        authored.mDamping = 0.8f;
        authored.mMaximumTorque = 1234.0f;

        auto result = RagdollCore::buildRagdollSettings(RagdollCoreTest::makeSkeleton(), profile, {});
        ASSERT_TRUE(result.isValid());
        const auto* swingTwist
            = static_cast<const JPH::SwingTwistConstraintSettings*>(result.mSettings->mParts[1].mToParent.GetPtr());
        EXPECT_FLOAT_EQ(swingTwist->mSwingMotorSettings.mSpringSettings.mFrequency, 6.0f);
        EXPECT_FLOAT_EQ(swingTwist->mSwingMotorSettings.mSpringSettings.mDamping, 0.8f);
        EXPECT_FLOAT_EQ(swingTwist->mSwingMotorSettings.mMinTorqueLimit, -1234.0f);
        EXPECT_FLOAT_EQ(swingTwist->mSwingMotorSettings.mMaxTorqueLimit, 1234.0f);
        EXPECT_FLOAT_EQ(swingTwist->mTwistMotorSettings.mMaxTorqueLimit, 1234.0f);

        profile.mBodies[1].mConstraint->mType = RagdollCore::ConstraintType::Hinge;
        result = RagdollCore::buildRagdollSettings(RagdollCoreTest::makeSkeleton(), profile, {});
        ASSERT_TRUE(result.isValid());
        const auto* hinge
            = static_cast<const JPH::HingeConstraintSettings*>(result.mSettings->mParts[1].mToParent.GetPtr());
        EXPECT_FLOAT_EQ(hinge->mMotorSettings.mSpringSettings.mFrequency, 6.0f);
        EXPECT_FLOAT_EQ(hinge->mMotorSettings.mSpringSettings.mDamping, 0.8f);
        EXPECT_FLOAT_EQ(hinge->mMotorSettings.mMinTorqueLimit, -1234.0f);
        EXPECT_FLOAT_EQ(hinge->mMotorSettings.mMaxTorqueLimit, 1234.0f);
    }

    TEST(RagdollCoreBuilderTest, disablesCollisionWithinExplicitSelfCollisionGroup)
    {
        auto skeleton = RagdollCoreTest::makeSkeleton();
        skeleton.mBones.push_back({ "arm", 0, { JPH::Vec3(10.0f, 0.0f, 0.0f), JPH::Quat::sIdentity() } });
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[0].mMassFraction = 0.4f;
        profile.mBodies[1].mMassFraction = 0.3f;
        profile.mBodies[1].mSelfCollisionGroup = 1;
        auto arm = profile.mBodies[1];
        arm.mName = "arm";
        arm.mBone = "arm";
        arm.mMassFraction = 0.3f;
        profile.mBodies.push_back(arm);

        const auto result = RagdollCore::buildRagdollSettings(skeleton, profile, {});
        ASSERT_TRUE(result.isValid());
        auto torsoGroup = result.mSettings->mParts[1].mCollisionGroup;
        auto armGroup = result.mSettings->mParts[2].mCollisionGroup;
        torsoGroup.SetGroupID(42);
        armGroup.SetGroupID(42);
        EXPECT_FALSE(torsoGroup.CanCollide(armGroup));
    }

    TEST(RagdollCoreBuilderTest, fitsCapsuleAcrossAuthoredSkeletonSpanAndKeepsJointCenterOfMass)
    {
        auto skeleton = RagdollCoreTest::makeSkeleton();
        skeleton.mBones[1].mModelTransform.mTranslation = JPH::Vec3(10.0f, 0.0f, 0.0f);
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[0].mShape.mType = RagdollCore::ShapeType::Capsule;
        profile.mBodies[0].mShape.mRadius = 2.0f;
        profile.mBodies[0].mShape.mHalfHeight = 1.0f;
        profile.mBodies[0].mShape.mFitToBone = "torso";

        RagdollCore::BuildOptions options;
        options.mUniformScale = 2.0f;
        const auto result = RagdollCore::buildRagdollSettings(skeleton, profile, options);
        ASSERT_TRUE(result.isValid());

        const JPH::Shape* shape = result.mSettings->mParts[0].GetShape();
        EXPECT_TRUE(shape->GetCenterOfMass().IsNearZero());
        ASSERT_EQ(shape->GetSubType(), JPH::EShapeSubType::OffsetCenterOfMass);
        const auto* centered = static_cast<const JPH::OffsetCenterOfMassShape*>(shape);
        ASSERT_EQ(centered->GetInnerShape()->GetSubType(), JPH::EShapeSubType::RotatedTranslated);
        const auto* fitted = static_cast<const JPH::RotatedTranslatedShape*>(centered->GetInnerShape());
        ASSERT_EQ(fitted->GetInnerShape()->GetSubType(), JPH::EShapeSubType::Capsule);
        const auto* capsule = static_cast<const JPH::CapsuleShape*>(fitted->GetInnerShape());
        EXPECT_TRUE(fitted->GetPosition().IsClose(JPH::Vec3(10.0f, 0.0f, 0.0f)));
        EXPECT_TRUE((fitted->GetRotation() * JPH::Vec3::sAxisY()).IsClose(JPH::Vec3::sAxisX()));
        EXPECT_FLOAT_EQ(capsule->GetRadius(), 4.0f);
        EXPECT_FLOAT_EQ(capsule->GetHalfHeightOfCylinder(), 6.0f);
    }
    TEST(RagdollCoreBuilderTest, shortFittedSpanPreservesAuthoredThicknessAndJointFrames)
    {
        auto skeleton = RagdollCoreTest::makeSkeleton();
        skeleton.mBones[1].mModelTransform.mTranslation = JPH::Vec3(2, 0, 0);
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[0].mShape.mType = RagdollCore::ShapeType::Capsule;
        profile.mBodies[0].mShape.mRadius = 2;
        profile.mBodies[0].mShape.mHalfHeight = 1;
        profile.mBodies[0].mShape.mFitToBone = "torso";
        profile.mBodies[1].mConstraint->mDeriveFrameFromSkeleton = true;
        RagdollCore::BuildOptions options;
        options.mUniformScale = 2;
        const auto built = RagdollCore::buildRagdollSettings(skeleton, profile, options);
        ASSERT_TRUE(built.isValid());
        const auto& part = built.mSettings->mParts[0];
        EXPECT_TRUE(part.GetShape()->GetCenterOfMass().IsNearZero());
        ASSERT_EQ(part.GetShape()->GetSubType(), JPH::EShapeSubType::OffsetCenterOfMass);
        const auto* centered = static_cast<const JPH::OffsetCenterOfMassShape*>(part.GetShape());
        ASSERT_EQ(centered->GetInnerShape()->GetSubType(), JPH::EShapeSubType::RotatedTranslated);
        const auto* fitted = static_cast<const JPH::RotatedTranslatedShape*>(centered->GetInnerShape());
        EXPECT_TRUE(fitted->GetPosition().IsClose(JPH::Vec3(2, 0, 0)));
        ASSERT_EQ(fitted->GetInnerShape()->GetSubType(), JPH::EShapeSubType::Sphere);
        EXPECT_FLOAT_EQ(static_cast<const JPH::SphereShape*>(fitted->GetInnerShape())->GetRadius(), 4);
        EXPECT_FLOAT_EQ(part.GetMassProperties().mMass, options.mTotalMass * profile.mBodies[0].mMassFraction);
        const auto* joint = static_cast<const JPH::SwingTwistConstraintSettings*>(built.mSettings->mParts[1].mToParent.GetPtr());
        EXPECT_TRUE(joint->mPosition1.IsClose(JPH::RVec3(4, 0, 0)));
        EXPECT_TRUE(joint->mPosition2.IsNearZero());
    }

}
