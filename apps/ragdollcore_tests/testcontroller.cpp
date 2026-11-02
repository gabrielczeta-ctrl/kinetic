#include <ragdollcore/controller.hpp>
#include <ragdollcore/gaitcontroller.hpp>
#include <ragdollcore/unitmodel.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string_view>
#include <vector>

namespace
{
    RagdollCore::SupportEstimate supported(float height = 80.0f)
    {
        RagdollCore::SupportEstimate result;
        result.mLeftFootSupported = true;
        result.mRightFootSupported = true;
        result.mCenterOfMassSupported = true;
        result.mFootContactCount = 2;
        result.mGroundContactCount = 2;
        result.mCenterOfMassHeight = height;
        return result;
    }

    std::vector<RagdollCore::SupportJointBody> supportBodies()
    {
        return { { -1, JPH::RVec3(0, 0, 3), JPH::RVec3(0, 0, 3), 6 },
            { 0, JPH::RVec3(-1, 0, 0), JPH::RVec3(-1, 0, 0), 1 },
            { 0, JPH::RVec3(1, 0, 0), JPH::RVec3(1, 0, 0), 1 },
            { 0, JPH::RVec3(1, 0, 3), JPH::RVec3(0, 0, 3), 2 } };
    }

    std::vector<RagdollCore::GroundContact> supportCorners()
    {
        return { { 1, JPH::RVec3(-1, -1, 0) }, { 1, JPH::RVec3(-1, 1, 0) },
            { 2, JPH::RVec3(1, -1, 0) }, { 2, JPH::RVec3(1, 1, 0) } };
    }

    TEST(RagdollCoreSupportTest, frictionUsesOnlyMeasuredLoadBearingContacts)
    {
        auto contacts = supportCorners();
        for (auto& contact : contacts)
            contact.mFriction = 0.4f;
        contacts.back().mFriction = 0.2f;
        const auto estimate = [&] {
            return RagdollCore::estimateFootSupport(JPH::RVec3(0, 0, 3), contacts, { 1, 2 }, {});
        };
        auto support = estimate();
        ASSERT_TRUE(support && support->mFriction);
        EXPECT_FLOAT_EQ(*support->mFriction, 0.2f);
        contacts.back().mPenetrationDepth = -0.1f;
        contacts.back().mNormalImpulse = 0;
        support = estimate();
        ASSERT_TRUE(support && support->mFriction);
        EXPECT_FLOAT_EQ(*support->mFriction, 0.4f);
        contacts.back().mNormalImpulse = 1;
        support = estimate();
        ASSERT_TRUE(support && support->mFriction);
        EXPECT_FLOAT_EQ(*support->mFriction, 0.2f);
        contacts.back().mFriction.reset();
        support = estimate();
        ASSERT_TRUE(support);
        EXPECT_FALSE(support->mFriction) << "unknown material must not silently promise grip";
        for (const float invalid : { -1.0f, std::numeric_limits<float>::quiet_NaN() })
        {
            contacts.back().mFriction = invalid;
            EXPECT_FALSE(estimate());
        }
    }

    TEST(RagdollCoreSupportTest, supportCenterDoesNotDependOnManifoldSamplingDensity)
    {
        auto contacts = supportCorners();
        const auto baseline = RagdollCore::estimateFootSupport(JPH::RVec3(0, 0, 3), contacts, { 1, 2 }, {});
        ASSERT_TRUE(baseline);
        // A mesh can report several manifolds over the same supporting patch.
        // Duplicate corners and extra interior/edge samples do not move it.
        contacts.insert(contacts.end(), 5, contacts.front());
        contacts.push_back({ 1, JPH::RVec3(-1, 0, 0) });
        contacts.push_back({ 1, JPH::RVec3(-0.5, 0.5, 0) });
        std::reverse(contacts.begin(), contacts.end());
        const auto sampled = RagdollCore::estimateFootSupport(JPH::RVec3(0, 0, 3), contacts, { 1, 2 }, {});
        ASSERT_TRUE(sampled);
        EXPECT_LT((sampled->mCenter - baseline->mCenter).Length(), 1e-6);
        EXPECT_EQ(sampled->mCenterOfMassSupported, baseline->mCenterOfMassSupported);
        EXPECT_LT((sampled->mClosestPoint - baseline->mClosestPoint).Length(), 1e-6);
    }

    TEST(RagdollCoreJointSupportTest, rejectsInvalidMeasuredJointTopology)
    {
        RagdollCore::RagdollTelemetry telemetry;
        telemetry.mTotalMass = 3;
        telemetry.mBodies.resize(3);
        for (auto& body : telemetry.mBodies)
            body.mMass = 1;
        telemetry.mConstraints = { { 0, 1 }, { 0, 2 } };
        const auto calculate = [&] {
            return RagdollCore::calculateStandingJointSupport(telemetry, {}, { 1, 2 }, 10, 0.5f, {});
        };
        ASSERT_TRUE(calculate());
        telemetry.mConstraints[1].mChildBodyIndex = 3;
        EXPECT_FALSE(calculate());
        telemetry.mConstraints[1].mChildBodyIndex = 1;
        EXPECT_FALSE(calculate());
        telemetry.mConstraints.pop_back();
        EXPECT_FALSE(calculate());
    }

    TEST(RagdollCoreJointSupportTest, holdsAnOffsetLoadWithBalancedContactWrench)
    {
        const auto bodies = supportBodies();
        const auto contacts = supportCorners();
        RagdollCore::JointSupportInput input{ .mBodies = bodies, .mContacts = contacts, .mFeet = { 1, 2 },
            .mRequestedAcceleration = JPH::Vec3(1, 0, 0), .mGravity = 10, .mFriction = 0.5f };
        const auto control = RagdollCore::calculateJointSupportControl(input, {});
        ASSERT_TRUE(control);
        ASSERT_TRUE(control->mSupported);
        ASSERT_TRUE(control->mHullProjectionDelta);
        EXPECT_LT(control->mHullProjectionDelta->Length(), 1e-5f);
        EXPECT_FALSE(control->mFrictionLimited);
        EXPECT_LT((control->mAcceleration - JPH::Vec3(1, 0, 0)).Length(), 1e-5f);
        EXPECT_NEAR(control->mCenterOfPressure.GetX(), -0.04, 1e-6);
        EXPECT_NEAR(control->mCenterOfPressure.GetY(), 0, 1e-6);
        // The two-kilogram arm is one unit from its joint: 2 * 10 * 1.
        EXPECT_LT((control->mJointTorques[3] - JPH::Vec3(0, -20, 0)).Length(), 1e-5f);
        EXPECT_TRUE(control->mJointTorques[0].IsNearZero());
        EXPECT_LT(control->mRootForceResidual.Length(), 1e-4f);
        EXPECT_LT(control->mRootMomentResidual.Length(), 1e-4f);
        JPH::Vec3 force = JPH::Vec3::sZero();
        for (const auto& contact : control->mContactForces)
        {
            EXPECT_GE(contact.mForce.GetZ(), 0.0f);
            EXPECT_LE(JPH::Vec3(contact.mForce.GetX(), contact.mForce.GetY(), 0).Length(),
                input.mFriction * contact.mForce.GetZ() + 1e-5f);
            force += contact.mForce;
        }
        EXPECT_LT((force - JPH::Vec3(10, 0, 100)).Length(), 1e-4f);
    }

    TEST(RagdollCoreJointSupportTest, isCovariantAndDoesNotDependOnContactOrdering)
    {
        auto bodies = supportBodies();
        auto contacts = supportCorners();
        RagdollCore::JointSupportInput input{ .mBodies = bodies, .mContacts = contacts, .mFeet = { 1, 2 },
            .mRequestedAcceleration = JPH::Vec3(1, -0.3f, 0), .mGravity = 10, .mFriction = 0.5f };
        const auto baseline = RagdollCore::calculateJointSupportControl(input, {});
        ASSERT_TRUE(baseline && baseline->mSupported);
        const auto rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.7f);
        const JPH::RVec3 translation(100000, -300000, 5);
        for (auto& body : bodies)
        {
            body.mCenterOfMass = translation + rotation * JPH::Vec3(body.mCenterOfMass);
            body.mJointPosition = translation + rotation * JPH::Vec3(body.mJointPosition);
        }
        for (auto& contact : contacts)
            contact.mPosition = translation + rotation * JPH::Vec3(contact.mPosition);
        std::reverse(contacts.begin(), contacts.end());
        input.mRequestedAcceleration = rotation * input.mRequestedAcceleration;
        const auto transformed = RagdollCore::calculateJointSupportControl(input, {});
        ASSERT_TRUE(transformed && transformed->mSupported);
        EXPECT_LT((transformed->mAcceleration - rotation * baseline->mAcceleration).Length(), 1e-5f);
        for (std::size_t i = 0; i < bodies.size(); ++i)
            EXPECT_LT((transformed->mJointTorques[i] - rotation * baseline->mJointTorques[i]).Length(), 1e-4f);
        EXPECT_LT(transformed->mRootForceResidual.Length(), 1e-4f);
        EXPECT_LT(transformed->mRootMomentResidual.Length(), 1e-4f);
    }

    TEST(RagdollCoreJointSupportTest, cannotInventAirborneOrSpeculativeSupport)
    {
        const auto bodies = supportBodies();
        auto contacts = supportCorners();
        for (auto& contact : contacts)
            contact.mPenetrationDepth = -0.01f;
        RagdollCore::JointSupportInput input{ .mBodies = bodies, .mContacts = contacts, .mFeet = { 1, 2 },
            .mRequestedAcceleration = JPH::Vec3::sZero(), .mGravity = 10, .mFriction = 0.5f };
        for (unsigned i = 0; i < 2; ++i)
        {
            const auto result = RagdollCore::calculateJointSupportControl(input, {});
            ASSERT_TRUE(result);
            EXPECT_FALSE(result->mSupported);
            EXPECT_TRUE(result->mContactForces.empty());
            for (const auto& torque : result->mJointTorques)
                EXPECT_TRUE(torque.IsNearZero());
            input.mContacts = {};
        }
    }

    TEST(RagdollCoreJointSupportTest, recognizesSpeculativeContactsOnlyAfterMeasuredSolverResponse)
    {
        const auto bodies = supportBodies();
        auto contacts = supportCorners();
        for (auto& contact : contacts)
        {
            contact.mPenetrationDepth = -0.1f;
            contact.mNormalImpulse = 0.0f;
        }
        const RagdollCore::JointSupportInput input{ .mBodies = bodies, .mContacts = contacts, .mFeet = { 1, 2 },
            .mRequestedAcceleration = JPH::Vec3::sZero(), .mGravity = 10, .mFriction = 0.5f };
        const auto unloaded = RagdollCore::calculateJointSupportControl(input, {});
        ASSERT_TRUE(unloaded);
        EXPECT_FALSE(unloaded->mSupported);
        const auto absent = RagdollCore::estimateFootSupport(JPH::RVec3(0, 0, 3), contacts, { 1, 2 }, {});
        ASSERT_TRUE(absent);
        EXPECT_FALSE(absent->hasSupport());

        for (auto& contact : contacts)
            contact.mNormalImpulse = 2.0f;
        const auto loaded = RagdollCore::calculateJointSupportControl(input, {});
        ASSERT_TRUE(loaded);
        EXPECT_TRUE(loaded->mSupported);
        const auto support = RagdollCore::estimateFootSupport(JPH::RVec3(0, 0, 3), contacts, { 1, 2 }, {});
        ASSERT_TRUE(support);
        EXPECT_TRUE(support->mLeftFootSupported);
        EXPECT_TRUE(support->mRightFootSupported);
        EXPECT_TRUE(support->mCenterOfMassSupported);
        EXPECT_LT(loaded->mRootForceResidual.Length(), 1e-4f);
        EXPECT_LT(loaded->mRootMomentResidual.Length(), 1e-4f);
        for (const auto invalid : { -1.0f, std::numeric_limits<float>::quiet_NaN() })
        {
            contacts[0].mNormalImpulse = invalid;
            EXPECT_FALSE(RagdollCore::calculateJointSupportControl(input, {}));
            EXPECT_FALSE(RagdollCore::estimateFootSupport(JPH::RVec3(0, 0, 3), contacts, { 1, 2 }, {}));
        }
    }

    TEST(RagdollCoreJointSupportTest, respectsFrictionAndDegenerateContactHulls)
    {
        const auto bodies = supportBodies();
        auto contacts = supportCorners();
        RagdollCore::JointSupportInput input{ .mBodies = bodies, .mContacts = contacts, .mFeet = { 1, 2 },
            .mRequestedAcceleration = JPH::Vec3(100, 0, 0), .mGravity = 10, .mFriction = 0.01f };
        const auto limited = RagdollCore::calculateJointSupportControl(input, {});
        ASSERT_TRUE(limited && limited->mSupported);
        EXPECT_TRUE(limited->mFrictionLimited);
        EXPECT_LE(limited->mAcceleration.Length(), 0.10001f);
        input.mFriction = 0;
        const auto frictionless = RagdollCore::calculateJointSupportControl(input, {});
        ASSERT_TRUE(frictionless && frictionless->mSupported);
        EXPECT_LT(frictionless->mAcceleration.Length(), 1e-6f);
        contacts = { { 1, JPH::RVec3(-1, 0, 0) }, { 2, JPH::RVec3(1, 0, 0) } };
        input.mContacts = contacts;
        input.mRequestedAcceleration = JPH::Vec3::sZero();
        const auto line = RagdollCore::calculateJointSupportControl(input, {});
        ASSERT_TRUE(line && line->mSupported);
        EXPECT_LT(line->mRootMomentResidual.Length(), 1e-4f);
        contacts.resize(1);
        input.mContacts = contacts;
        const auto unreachable = RagdollCore::calculateJointSupportControl(input, {});
        ASSERT_TRUE(unreachable);
        EXPECT_FALSE(unreachable->mSupported);
    }

    TEST(RagdollCoreJointSupportTest, distinguishesHullProjectionFromFrictionLimiting)
    {
        const auto bodies = supportBodies();
        const auto contacts = supportCorners();
        RagdollCore::JointSupportInput input{ .mBodies = bodies, .mContacts = contacts, .mFeet = { 1, 2 },
            .mRequestedAcceleration = JPH::Vec3(100, 0, 0), .mGravity = 10, .mFriction = 100 };
        const auto result = RagdollCore::calculateJointSupportControl(input, {});
        ASSERT_TRUE(result && result->mSupported && result->mHullProjectionDelta);
        EXPECT_FALSE(result->mFrictionLimited);
        EXPECT_GT(result->mHullProjectionDelta->GetX(), 0);
        EXPECT_NEAR(result->mHullProjectionDelta->GetY(), 0, 1e-5f);
        EXPECT_LT(result->mAcceleration.GetX(), input.mRequestedAcceleration.GetX());
        input.mContacts = {};
        const auto missing = RagdollCore::calculateJointSupportControl(input, {});
        ASSERT_TRUE(missing);
        EXPECT_FALSE(missing->mHullProjectionDelta);
    }

    TEST(RagdollCoreJointSupportTest, retainsFeasibleLoadsOnThinRotatedContactPatches)
    {
        auto bodies = supportBodies();
        // A foot rolling onto its edge produces a thin, rotated support patch.
        // The outside request must project to that patch without losing control.
        for (const double width : { 0.1, 0.01, 0.001, 0.0001 })
        {
            for (int angle = 0; angle < 180; ++angle)
            {
                const double radians = angle * JPH::JPH_PI / 180.0;
                const double sine = std::sin(radians), cosine = std::cos(radians);
                auto contacts = supportCorners();
                for (auto& contact : contacts)
                {
                    const double x = contact.mPosition.GetX();
                    const double y = contact.mPosition.GetY() * width;
                    contact.mPosition = JPH::RVec3(0.2 + cosine * x - sine * y, sine * x + cosine * y, 0);
                }
                RagdollCore::JointSupportInput input{ .mBodies = bodies, .mContacts = contacts, .mFeet = { 1, 2 },
                    .mRequestedAcceleration = JPH::Vec3(3, -2, 0), .mGravity = 10, .mFriction = 0.5f };
                const auto result = RagdollCore::calculateJointSupportControl(input, {});
                ASSERT_TRUE(result) << "width=" << width << " angle=" << angle;
                ASSERT_TRUE(result->mSupported) << "width=" << width << " angle=" << angle;
                const auto pressure = result->mCenterOfPressure - JPH::RVec3(0.2, 0, 0);
                EXPECT_LE(std::abs(cosine * pressure.GetX() + sine * pressure.GetY()), 1.0 + 1e-6);
                EXPECT_LE(std::abs(-sine * pressure.GetX() + cosine * pressure.GetY()), width + 1e-6);
                EXPECT_LE(result->mAcceleration.Length(), input.mFriction * input.mGravity + 1e-5f);
                EXPECT_LT(result->mRootForceResidual.Length(), 1e-4f);
                EXPECT_LT(result->mRootMomentResidual.Length(), 1e-4f);
                for (const auto& contact : result->mContactForces)
                    EXPECT_GE(contact.mForce.GetZ(), 0.0f);
            }
        }
    }

    TEST(RagdollCoreJointSupportTest, rejectsInvalidTreeAndNonFiniteLoad)
    {
        auto bodies = supportBodies();
        const auto contacts = supportCorners();
        RagdollCore::JointSupportInput input{ .mBodies = bodies, .mContacts = contacts, .mFeet = { 1, 2 },
            .mRequestedAcceleration = JPH::Vec3::sZero(), .mGravity = 10, .mFriction = 0.5f };
        bodies[1].mParent = 3;
        EXPECT_FALSE(RagdollCore::calculateJointSupportControl(input, {}));
        bodies[1].mParent = 0;
        input.mRequestedAcceleration.SetX(std::numeric_limits<float>::infinity());
        EXPECT_FALSE(RagdollCore::calculateJointSupportControl(input, {}));
    }

    TEST(RagdollCoreSupportTest, estimatesFootSupportAndTracksOtherGroundContacts)
    {
        const std::vector<RagdollCore::GroundContact> contacts = {
            { 3, JPH::RVec3(-10.0f, 0.0f, 2.0f), JPH::Vec3::sAxisZ() },
            { 4, JPH::RVec3(10.0f, 0.0f, 2.0f), JPH::Vec3(0.0f, 0.6f, 0.8f) },
            { 8, JPH::RVec3(100.0f, 100.0f, 0.0f), JPH::Vec3::sAxisZ() },
            { 3, JPH::RVec3(-200.0f, 0.0f, 50.0f), JPH::Vec3::sAxisZ(), -0.25f },
        };
        RagdollCore::ActiveControllerProfile profile;
        profile.mSupportMargin = 2.0f;
        const auto estimate = RagdollCore::estimateFootSupport(
            JPH::RVec3(0.0f, 1.0f, 82.0f), contacts, { 3, 4 }, profile);
        ASSERT_TRUE(estimate.has_value());
        EXPECT_TRUE(estimate->mLeftFootSupported);
        EXPECT_TRUE(estimate->mRightFootSupported);
        EXPECT_TRUE(estimate->mCenterOfMassSupported);
        EXPECT_EQ(estimate->mFootContactCount, 2u);
        EXPECT_EQ(estimate->mGroundContactCount, 3u);
        EXPECT_TRUE(estimate->mCenter.IsClose(JPH::RVec3(0.0f, 0.0f, 2.0f)));
        EXPECT_NEAR(estimate->mGroundCenter.GetX(), 100.0 / 3.0, 1e-5);
        EXPECT_NEAR(estimate->mGroundCenter.GetY(), 100.0 / 3.0, 1e-5);
        EXPECT_NEAR(estimate->mGroundCenter.GetZ(), 4.0 / 3.0, 1e-5);
        EXPECT_NEAR(estimate->mClosestPoint.GetX(), 0.0, 1e-6);
        EXPECT_NEAR(estimate->mClosestPoint.GetY(), 0.0, 1e-6);
        EXPECT_NEAR(estimate->mClosestPoint.GetZ(), 4.0 / 3.0, 1e-6);
        EXPECT_FLOAT_EQ(estimate->mCenterOfMassDistance, 1.0f);
        EXPECT_NEAR(estimate->mCenterOfMassHeight, 82.0f - 4.0f / 3.0f, 1e-5f);
    }

    TEST(RagdollCoreSupportTest, measuresDistanceOutsideSupportHull)
    {
        const std::vector<RagdollCore::GroundContact> contacts = {
            { 3, JPH::RVec3(-1.0f, -1.0f, 0.0f), JPH::Vec3::sAxisZ() },
            { 3, JPH::RVec3(-1.0f, 1.0f, 0.0f), JPH::Vec3::sAxisZ() },
            { 4, JPH::RVec3(1.0f, -1.0f, 0.0f), JPH::Vec3::sAxisZ() },
            { 4, JPH::RVec3(1.0f, 1.0f, 0.0f), JPH::Vec3::sAxisZ() },
        };
        RagdollCore::ActiveControllerProfile profile;
        profile.mSupportMargin = 0.5f;
        const auto estimate = RagdollCore::estimateFootSupport(
            JPH::RVec3(3.0f, 0.0f, 10.0f), contacts, { 3, 4 }, profile);
        ASSERT_TRUE(estimate.has_value());
        EXPECT_FALSE(estimate->mCenterOfMassSupported);
        EXPECT_TRUE(estimate->mClosestPoint.IsClose(JPH::RVec3(1.0f, 0.0f, 0.0f)));
        EXPECT_FLOAT_EQ(estimate->mCenterOfMassDistance, 2.0f);
    }

    TEST(RagdollCoreSupportTest, rejectsMalformedObservationsTransactionally)
    {
        RagdollCore::ActiveControllerProfile profile;
        std::vector<RagdollCore::GroundContact> contacts = {
            { 3, JPH::RVec3::sZero(), JPH::Vec3::sAxisZ() },
        };
        EXPECT_FALSE(RagdollCore::estimateFootSupport(JPH::RVec3::sZero(), contacts, { 3, 3 }, profile));
        contacts[0].mNormal.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(RagdollCore::estimateFootSupport(JPH::RVec3::sZero(), contacts, { 3, 4 }, profile));
        contacts[0].mNormal = JPH::Vec3::sAxisZ();
        contacts[0].mPenetrationDepth = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::estimateFootSupport(JPH::RVec3::sZero(), contacts, { 3, 4 }, profile));
    }

    TEST(RagdollCoreUprightControllerTest, producesZeroCorrectionAtRestOnTarget)
    {
        RagdollCore::UprightControllerInput input;
        input.mTargetPelvisHeight = 80.0f;
        input.mPelvisHeight = 80.0f;
        input.mPelvisAngularVelocity = JPH::Vec3(0.0f, 0.0f, 2.0f);
        input.mTorsoAngularVelocity = JPH::Vec3(0.0f, 0.0f, -3.0f);
        input.mTotalMass = 75.0f;
        const auto output = RagdollCore::calculateUprightControl(input, {});
        ASSERT_TRUE(output.has_value());
        EXPECT_TRUE(output->mPelvisForce.IsNearZero());
        EXPECT_TRUE(output->mPelvisTorque.IsNearZero());
        EXPECT_TRUE(output->mSpineTorque.IsNearZero());
        EXPECT_FALSE(output->mPelvisForceSaturated);
        EXPECT_FALSE(output->mPelvisMagnitudeSaturated);
        EXPECT_FALSE(output->mPelvisHorizontalForceSaturated);
        EXPECT_FALSE(output->mPelvisVerticalForceSaturated);
        EXPECT_FALSE(output->mPelvisTorqueSaturated);
        EXPECT_FALSE(output->mSpineTorqueSaturated);
    }

    TEST(RagdollCoreUprightControllerTest, correctsHeightCenterOfMassAndTilt)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mPelvisHeightFrequency = 1.0f;
        profile.mPelvisHeightDamping = 0.0f;
        profile.mCenterOfMassFrequency = 1.0f;
        profile.mCenterOfMassDamping = 0.0f;
        profile.mMaximumPelvisForce = 1000.0f;
        profile.mPelvisUprightStiffness = 10.0f;
        profile.mPelvisUprightDamping = 0.0f;
        profile.mMaximumPelvisTorque = 1000.0f;
        profile.mSpineUprightStiffness = 20.0f;
        profile.mSpineUprightDamping = 0.0f;
        profile.mMaximumSpineTorque = 1000.0f;

        RagdollCore::UprightControllerInput input;
        input.mTargetPelvisHeight = 2.0f;
        input.mPelvisHeight = 1.0f;
        input.mCenterOfMassOffset = JPH::Vec3(1.0f, 0.0f, 7.0f);
        input.mPelvisUp = JPH::Vec3::sAxisX();
        input.mTorsoUp = JPH::Vec3::sAxisY();
        input.mTotalMass = 2.0f;
        const auto output = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(output.has_value());
        constexpr float expectedForce = 78.9568352f;
        EXPECT_NEAR(output->mPelvisForce.GetX(), -expectedForce, 1e-4f);
        EXPECT_NEAR(output->mPelvisForce.GetZ(), expectedForce, 1e-4f);
        constexpr float halfPi = 1.57079632679489661923f;
        EXPECT_TRUE(output->mPelvisTorque.IsClose(JPH::Vec3(0.0f, -10.0f * halfPi, 0.0f)));
        EXPECT_TRUE(output->mSpineTorque.IsClose(JPH::Vec3(20.0f * halfPi, 0.0f, 0.0f)));

        input.mPelvisUp = -JPH::Vec3::sAxisZ();
        input.mTorsoUp = JPH::Vec3::sAxisZ();
        const auto inverted = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(inverted.has_value());
        EXPECT_NEAR(inverted->mPelvisTorque.Length(), 10.0f * 2.0f * halfPi, 1e-4f);
    }

    TEST(RagdollCoreUprightControllerTest, saturatesEveryOutputAndRejectsInvalidInput)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mMaximumPelvisForce = 5.0f;
        profile.mMaximumPelvisTorque = 4.0f;
        profile.mMaximumSpineTorque = 3.0f;
        RagdollCore::UprightControllerInput input;
        input.mTargetPelvisHeight = 100.0f;
        input.mPelvisUp = JPH::Vec3::sAxisX();
        input.mTorsoUp = JPH::Vec3::sAxisY();
        input.mTotalMass = 75.0f;
        const auto output = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(output.has_value());
        EXPECT_NEAR(output->mPelvisForce.Length(), 5.0f, 1e-5f);
        EXPECT_NEAR(output->mPelvisTorque.Length(), 4.0f, 1e-5f);
        EXPECT_NEAR(output->mSpineTorque.Length(), 3.0f, 1e-5f);
        EXPECT_TRUE(output->mPelvisForceSaturated);
        EXPECT_TRUE(output->mPelvisMagnitudeSaturated);
        EXPECT_FALSE(output->mPelvisHorizontalForceSaturated);
        EXPECT_FALSE(output->mPelvisVerticalForceSaturated);
        EXPECT_TRUE(output->mPelvisTorqueSaturated);
        EXPECT_TRUE(output->mSpineTorqueSaturated);

        input.mTotalMass = 0.0f;
        EXPECT_FALSE(RagdollCore::calculateUprightControl(input, profile).has_value());
        input.mTotalMass = 75.0f;
        input.mPelvisUp.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(RagdollCore::calculateUprightControl(input, profile).has_value());
        profile.mPelvisHeightFrequency = -1.0f;
        input.mPelvisUp = JPH::Vec3::sAxisZ();
        EXPECT_FALSE(RagdollCore::calculateUprightControl(input, profile).has_value());
    }

    TEST(RagdollCoreUprightControllerTest, unweightsRecoveryWithoutAddingHorizontalAcceleration)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mMaximumPelvisForce = 150000.0f;
        RagdollCore::UprightControllerInput input;
        input.mTargetPelvisHeight = 80.0f;
        input.mPelvisHeight = 10.0f;
        input.mCenterOfMassOffset = JPH::Vec3(20.0f, -30.0f, 0.0f);
        input.mCenterOfMassVelocity = JPH::Vec3(10.0f, 10.0f, 0.0f);
        input.mTotalMass = 75.0f;
        input.mRecoveryUnweight = true;

        const auto output = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(output.has_value());
        EXPECT_NEAR(output->mPelvisForce.GetX(), 0.0f, 1e-4f);
        EXPECT_NEAR(output->mPelvisForce.GetY(), 0.0f, 1e-4f);
        EXPECT_NEAR(output->mPelvisForce.GetZ(),
            input.mTotalMass * RagdollCore::UnitModel::GravityUnitsPerSecondSquared
                * profile.mRecoveryUnweightFactor,
            0.1f);
    }

    TEST(RagdollCoreUprightControllerTest, tracksBoundedHorizontalGaitVelocity)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mCenterOfMassFrequency = 1.0f;
        profile.mCenterOfMassDamping = 1.0f;
        profile.mMaximumLocomotionSpeed = 5.0f;
        profile.mMaximumPelvisForce = 10000.0f;
        RagdollCore::UprightControllerInput input;
        input.mTargetHorizontalVelocity = JPH::Vec3(100.0f, 0.0f, 50.0f);
        input.mTotalMass = 2.0f;

        const auto output = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(output.has_value());
        constexpr float expectedForce = 125.663706f;
        EXPECT_NEAR(output->mPelvisForce.GetX(), expectedForce, 1e-4f);
        EXPECT_NEAR(output->mPelvisForce.GetY(), 0.0f, 1e-4f);
        EXPECT_NEAR(output->mPelvisForce.GetZ(), 0.0f, 1e-4f);

        input.mCenterOfMassVelocity = JPH::Vec3(5.0f, 0.0f, 0.0f);
        const auto atTarget = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(atTarget.has_value());
        EXPECT_TRUE(atTarget->mPelvisForce.IsNearZero());
    }

    TEST(RagdollCoreJointSupportTest, measuredFootLoadUsesOnlyRealSolverImpulse)
    {
        std::vector<RagdollCore::GroundContact> contacts{
            { .mBodyIndex = 1, .mNormalImpulse = 2.0f },
            { .mBodyIndex = 1 },
            { .mBodyIndex = 2, .mNormalImpulse = 8.0f },
        };
        const auto load = RagdollCore::calculateMeasuredFootNormalLoadFraction(
            contacts, 1, 10.0f, 10.0f, 0.1f);
        ASSERT_TRUE(load);
        EXPECT_FLOAT_EQ(*load, 0.2f);
        contacts[0].mNormalImpulse = -1.0f;
        EXPECT_FALSE(RagdollCore::calculateMeasuredFootNormalLoadFraction(
            contacts, 1, 10.0f, 10.0f, 0.1f));
    }

    TEST(RagdollCoreJointSupportTest, supportBlendUsesOneWeightForEveryActuationChannel)
    {
        RagdollCore::JointSupportOutput bilateral;
        bilateral.mSupported = true;
        bilateral.mAcceleration = JPH::Vec3(2, 4, 6);
        bilateral.mCenterOfPressure = JPH::RVec3(2, 0, 0);
        bilateral.mContactForces = { { 1, JPH::RVec3::sZero(), JPH::Vec3(0, 0, 8) } };
        bilateral.mJointTorques = { JPH::Vec3(2, 0, 0), JPH::Vec3(4, 0, 0) };
        bilateral.mRootForceResidual = JPH::Vec3(6, 0, 0);
        bilateral.mRootMomentResidual = JPH::Vec3(8, 0, 0);
        auto unilateral = bilateral;
        unilateral.mAcceleration = JPH::Vec3(10, 12, 14);
        unilateral.mCenterOfPressure = JPH::RVec3(10, 0, 0);
        unilateral.mContactForces = { { 2, JPH::RVec3::sZero(), JPH::Vec3(0, 0, 16) } };
        unilateral.mJointTorques = { JPH::Vec3(10, 0, 0), JPH::Vec3(12, 0, 0) };
        unilateral.mRootForceResidual = JPH::Vec3(14, 0, 0);
        unilateral.mRootMomentResidual = JPH::Vec3(16, 0, 0);
        const auto blended = RagdollCore::blendJointSupportOutputs(bilateral, unilateral, 0.25f);
        ASSERT_TRUE(blended);
        EXPECT_TRUE(blended->mAcceleration.IsClose(JPH::Vec3(4, 6, 8)));
        EXPECT_NEAR(blended->mCenterOfPressure.GetX(), 4.0, 1e-6);
        EXPECT_TRUE(blended->mJointTorques[0].IsClose(JPH::Vec3(4, 0, 0)));
        EXPECT_TRUE(blended->mRootForceResidual.IsClose(JPH::Vec3(8, 0, 0)));
        ASSERT_EQ(blended->mContactForces.size(), 2u);
        EXPECT_FLOAT_EQ(blended->mContactForces[0].mForce.GetZ(), 6.0f);
        EXPECT_FLOAT_EQ(blended->mContactForces[1].mForce.GetZ(), 4.0f);
    }

    TEST(RagdollCoreUprightControllerTest, animationAssistReleasesForHitsAndFalls)
    {
        using RagdollCore::ActiveControllerMode;
        using RagdollCore::LocomotionRoute;
        EXPECT_TRUE(RagdollCore::shouldUseAnimationAssistedActuation(
            LocomotionRoute::KinematicRoot, ActiveControllerMode::Standing, true, false));
        EXPECT_TRUE(RagdollCore::shouldUseAnimationAssistedActuation(
            LocomotionRoute::KinematicRoot, ActiveControllerMode::Bracing, true, false));
        EXPECT_TRUE(RagdollCore::shouldUseAnimationAssistedActuation(
            LocomotionRoute::KinematicRoot, ActiveControllerMode::Stumbling, true, false));
        EXPECT_FALSE(RagdollCore::shouldUseAnimationAssistedActuation(
            LocomotionRoute::KinematicRoot, ActiveControllerMode::Standing, true, true));
        EXPECT_FALSE(RagdollCore::shouldUseAnimationAssistedActuation(
            LocomotionRoute::KinematicRoot, ActiveControllerMode::Falling, true, false));
        EXPECT_FALSE(RagdollCore::shouldUseAnimationAssistedActuation(
            LocomotionRoute::KinematicRoot, ActiveControllerMode::Fallen, true, false));
        EXPECT_FALSE(RagdollCore::shouldUseAnimationAssistedActuation(
            LocomotionRoute::Reference, ActiveControllerMode::Standing, true, false));
        EXPECT_FALSE(RagdollCore::shouldUseAnimationAssistedActuation(
            LocomotionRoute::Legacy, ActiveControllerMode::Standing, true, false));
        EXPECT_FALSE(RagdollCore::shouldUseAnimationAssistedActuation(
            LocomotionRoute::KinematicRoot, ActiveControllerMode::Standing, false, false));
    }

    TEST(RagdollCoreUprightControllerTest, stanceContactTargetClampsAndScalesPoseEndpoint)
    {
        RagdollCore::StanceContactPositionInput input;
        input.mAnchorPosition = JPH::RVec3(10.0, 0.0, 0.0);
        input.mDesiredFootOrigin = JPH::RVec3(6.0, 0.0, 1.0);
        input.mDesiredFootRotation = JPH::Quat::sIdentity();
        input.mContactOffsetInFoot = JPH::Vec3(0.0f, 0.0f, -1.0f);
        input.mCorrectionFraction = 0.5f;
        input.mMaximumEndpointCorrection = 2.0f;
        const auto output = RagdollCore::calculateStanceContactPositionTarget(input);
        ASSERT_TRUE(output);
        EXPECT_TRUE(output->mSaturated);
        EXPECT_TRUE(output->mDesiredContactPosition.IsClose(JPH::RVec3(6.0, 0.0, 0.0)));
        EXPECT_TRUE(output->mContactError.IsClose(JPH::Vec3(4.0f, 0.0f, 0.0f)));
        EXPECT_TRUE(output->mEndpointCorrection.IsClose(JPH::Vec3(1.0f, 0.0f, 0.0f)));
        EXPECT_TRUE(output->mTargetFootOrigin.IsClose(JPH::RVec3(7.0, 0.0, 1.0)));
    }

    TEST(RagdollCoreUprightControllerTest, stanceContactTargetRejectsInvalidAuthority)
    {
        RagdollCore::StanceContactPositionInput input;
        input.mCorrectionFraction = 1.1f;
        input.mMaximumEndpointCorrection = 1.0f;
        EXPECT_FALSE(RagdollCore::calculateStanceContactPositionTarget(input));
        input.mCorrectionFraction = 0.5f;
        input.mMaximumEndpointCorrection = 0.0f;
        EXPECT_FALSE(RagdollCore::calculateStanceContactPositionTarget(input));
    }

    TEST(RagdollCoreUprightControllerTest, limitsLocomotionToSupportFrictionAndBodyWeight)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mPelvisHeightFrequency = 1.0f;
        profile.mCenterOfMassFrequency = 1.0f;
        profile.mCenterOfMassDamping = 1.0f;
        profile.mMaximumLocomotionSpeed = 100.0f;
        profile.mMaximumPelvisForce = 100000.0f;
        RagdollCore::UprightControllerInput input;
        input.mTargetPelvisHeight = 10.0f;
        input.mTargetHorizontalVelocity = JPH::Vec3(100.0f, 0.0f, 0.0f);
        input.mTotalMass = 2.0f;
        input.mSupportFriction = 0.5f;
        input.mGravityMagnitude = 10.0f;
        input.mLimitToSupportFriction = true;

        const auto output = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(output.has_value());
        EXPECT_NEAR(output->mPelvisForce.GetX(), 10.0f, 1e-4f);
        EXPECT_NEAR(output->mPelvisForce.GetY(), 0.0f, 1e-4f);
        EXPECT_NEAR(output->mPelvisForce.GetZ(), 20.0f, 1e-4f);
        EXPECT_TRUE(output->mPelvisForceSaturated);
        EXPECT_FALSE(output->mPelvisMagnitudeSaturated);
        EXPECT_TRUE(output->mPelvisHorizontalForceSaturated);
        EXPECT_TRUE(output->mPelvisVerticalForceSaturated);

        input.mSupportFriction = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::calculateUprightControl(input, profile).has_value());
    }

    TEST(RagdollCoreUprightControllerTest, limitsLocomotionToMeasuredSupportMomentArm)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mCenterOfMassFrequency = 1.0f;
        profile.mCenterOfMassDamping = 1.0f;
        profile.mMaximumLocomotionSpeed = 100.0f;
        profile.mMaximumPelvisForce = 100000.0f;
        RagdollCore::UprightControllerInput input;
        input.mTargetHorizontalVelocity = JPH::Vec3(100.0f, 0.0f, 0.0f);
        input.mTotalMass = 2.0f;
        input.mSupportFriction = 200.0f;
        input.mGravityMagnitude = 10.0f;
        input.mLimitToSupportFriction = true;
        input.mCenterOfMassPosition = JPH::RVec3(0.0, 0.0, 10.0);
        input.mCenterOfMassHeight = 10.0f;
        input.mSupportContactPoints = { JPH::RVec3(-2.0, 0.0, 0.0) };
        input.mLimitToSupportMoment = true;

        const auto output = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(output.has_value());
        EXPECT_NEAR(output->mPelvisForce.GetX(), 4.0f, 1e-4f);
        EXPECT_NEAR(output->mPelvisForce.GetY(), 0.0f, 1e-4f);
        EXPECT_NEAR(output->mPelvisForce.GetZ(), 0.0f, 1e-4f);
        EXPECT_TRUE(output->mPelvisForceSaturated);
        EXPECT_FALSE(output->mPelvisHorizontalForceSaturated);
        EXPECT_TRUE(output->mPelvisSupportMomentSaturated);

        // Height correction is routed as an internal pelvis/support pair, so
        // it must not inflate the external ground-normal budget used by the
        // center-of-pressure limit.
        input.mTargetPelvisHeight = 10.0f;
        input.mPelvisHeight = 0.0f;
        const auto lifting = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(lifting.has_value());
        EXPECT_NEAR(lifting->mPelvisForce.GetX(), 4.0f, 1e-4f);
        EXPECT_GT(lifting->mPelvisForce.GetZ(), 0.0f);
        EXPECT_TRUE(lifting->mPelvisSupportMomentSaturated);

        input.mSupportContactPoints.clear();
        EXPECT_FALSE(RagdollCore::calculateUprightControl(input, profile).has_value());
    }

    TEST(RagdollCoreUprightControllerTest, boundsForceByPhysicalCenterOfPressureWhenCenterOfMassIsOutside)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mCenterOfMassFrequency = 1.0f;
        profile.mCenterOfMassDamping = 1.0f;
        profile.mMaximumLocomotionSpeed = 100.0f;
        profile.mMaximumPelvisForce = 100000.0f;

        RagdollCore::UprightControllerInput input;
        input.mCenterOfMassPosition = JPH::RVec3(4.0, 0.0, 10.0);
        input.mCenterOfMassHeight = 10.0f;
        input.mTotalMass = 2.0f;
        input.mSupportFriction = 200.0f;
        input.mGravityMagnitude = 10.0f;
        input.mLimitToSupportFriction = true;
        input.mLimitToSupportMoment = true;
        input.mSupportContactPoints = {
            JPH::RVec3(-2.0, -1.0, 0.0), JPH::RVec3(2.0, 1.0, 0.0) };

        input.mTargetHorizontalVelocity = JPH::Vec3(-100.0f, 0.0f, 0.0f);
        const auto inward = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(inward.has_value());
        EXPECT_NEAR(inward->mPelvisForce.GetX(), 0.0f, 1e-4f);
        EXPECT_TRUE(inward->mPelvisSupportMomentSaturated);

        input.mTargetHorizontalVelocity = JPH::Vec3(100.0f, 0.0f, 0.0f);
        const auto outward = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(outward.has_value());
        // The support segment crosses y = 0 at x = 0. A force of 8
        // places CoP at 4 - 10 * 8 / 20 = 0, exactly on that segment.
        EXPECT_NEAR(outward->mPelvisForce.GetX(), 8.0f, 1e-4f);
        const float centerOfPressureX = 4.0f - 10.0f * outward->mPelvisForce.GetX() / 20.0f;
        EXPECT_NEAR(centerOfPressureX, 0.0f, 1e-4f);
        EXPECT_TRUE(outward->mPelvisSupportMomentSaturated);
    }

    TEST(RagdollCoreUprightControllerTest, constrainsCenterOfPressureToActualPolygonAndForcePlane)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mCenterOfMassFrequency = 1.0f;
        profile.mMaximumLocomotionSpeed = 200.0f;
        RagdollCore::UprightControllerInput input;
        input.mTotalMass = 2.0f;
        input.mGravityMagnitude = 10.0f;
        input.mSupportFriction = 200.0f;
        input.mCenterOfMassHeight = 10.0f;
        input.mCenterOfMassPosition = JPH::RVec3(0, 0, 10);
        input.mLimitToSupportFriction = input.mLimitToSupportMoment = true;
        input.mSupportContactPoints = { JPH::RVec3(-2, -1, 0), JPH::RVec3(2, -1, 0),
            JPH::RVec3(2, 1, 0), JPH::RVec3(-2, 1, 0) };
        input.mTargetHorizontalVelocity = JPH::Vec3(100, 100, 0);
        auto output = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(output);
        const JPH::Vec3 cop = -output->mPelvisForce * input.mCenterOfMassHeight
            / (input.mTotalMass * input.mGravityMagnitude);
        EXPECT_NEAR(cop.GetX(), -1.0f, 1e-4f);
        EXPECT_NEAR(cop.GetY(), -1.0f, 1e-4f);
        // A support point behind the COM but outside the force plane cannot
        // supply this wrench, despite its otherwise generous moment arm.
        input.mTargetHorizontalVelocity = JPH::Vec3(100, 0, 0);
        input.mSupportContactPoints = { JPH::RVec3(-2, 1, 0) };
        output = RagdollCore::calculateUprightControl(input, profile);
        ASSERT_TRUE(output);
        EXPECT_TRUE(output->mPelvisForce.IsNearZero());
        EXPECT_TRUE(output->mPelvisSupportMomentSaturated);
    }

    TEST(RagdollCoreSettledPelvisHeightTest, requiresSustainedQuietSupportedStanding)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mPelvisHeightSettleDuration = 0.2f;
        profile.mPelvisHeightSettleMaximumVerticalSpeed = 2.0f;
        RagdollCore::SettledPelvisHeightTracker tracker;
        RagdollCore::SettledPelvisHeightInput input;
        input.mPelvisHeight = 73.0f;
        RagdollCore::SettledPelvisHeightOutput output;

        ASSERT_TRUE(tracker.step(input, 0.1f, profile, output));
        EXPECT_FALSE(output.mCandidateValid);
        EXPECT_FALSE(output.mTargetHeight.has_value());

        input.mSupport = supported();
        input.mSupport.mCenterOfMassSupported = false;
        ASSERT_TRUE(tracker.step(input, 0.1f, profile, output));
        // Feet on the ground are enough: a weapon held out carries the centre
        // of mass past the support hull, and requiring it kept the height from
        // ever settling (2026-09-24 N9). Bracing below still resets it.
        EXPECT_TRUE(output.mCandidateValid);

        input.mSupport.mCenterOfMassSupported = true;
        input.mMode = RagdollCore::ActiveControllerMode::Bracing;
        ASSERT_TRUE(tracker.step(input, 0.1f, profile, output));
        EXPECT_FALSE(output.mCandidateValid);
        ASSERT_TRUE(output.mTargetHeight.has_value());
        EXPECT_FLOAT_EQ(*output.mTargetHeight, 73.0f);

        input.mMode = RagdollCore::ActiveControllerMode::Standing;
        input.mPelvisVerticalVelocity = 2.1f;
        ASSERT_TRUE(tracker.step(input, 0.1f, profile, output));
        EXPECT_FALSE(output.mCandidateValid);
        EXPECT_FLOAT_EQ(*output.mTargetHeight, 73.0f);

        input.mPelvisVerticalVelocity = 1.0f;
        input.mLocomotionRequested = true;
        ASSERT_TRUE(tracker.step(input, 0.1f, profile, output));
        EXPECT_TRUE(output.mCandidateValid);
        EXPECT_FLOAT_EQ(output.mCandidateTime, 0.1f);
        ASSERT_TRUE(output.mTargetHeight.has_value());
        EXPECT_FLOAT_EQ(*output.mTargetHeight, 73.0f);
        EXPECT_FALSE(output.mSettled);
        EXPECT_FALSE(output.mFrozen);
        input.mPelvisHeight = 71.5f;
        ASSERT_TRUE(tracker.step(input, 0.1f, profile, output));
        ASSERT_TRUE(output.mTargetHeight.has_value());
        EXPECT_FLOAT_EQ(*output.mTargetHeight, 71.5f);
        EXPECT_TRUE(output.mSettled);
        EXPECT_TRUE(output.mFrozen);
    }

    TEST(RagdollCoreSettledPelvisHeightTest, updatesWhileSettledAndFreezesAtLocomotionRequest)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mPelvisHeightSettleDuration = 0.1f;
        RagdollCore::SettledPelvisHeightTracker tracker;
        RagdollCore::SettledPelvisHeightInput input;
        input.mSupport = supported();
        input.mPelvisHeight = 72.0f;
        RagdollCore::SettledPelvisHeightOutput output;

        ASSERT_TRUE(tracker.step(input, 0.1f, profile, output));
        ASSERT_TRUE(output.mTargetHeight.has_value());
        EXPECT_FLOAT_EQ(*output.mTargetHeight, 72.0f);
        input.mPelvisHeight = 71.0f;
        ASSERT_TRUE(tracker.step(input, 0.1f, profile, output));
        EXPECT_FLOAT_EQ(*output.mTargetHeight, 71.0f);

        input.mLocomotionRequested = true;
        input.mPelvisHeight = 68.0f;
        ASSERT_TRUE(tracker.step(input, 0.1f, profile, output));
        EXPECT_TRUE(output.mFrozen);
        EXPECT_FLOAT_EQ(*output.mTargetHeight, 71.0f);
        input.mLocomotionRequested = false;
        ASSERT_TRUE(tracker.step(input, 0.1f, profile, output));
        EXPECT_TRUE(output.mFrozen);
        EXPECT_FLOAT_EQ(*output.mTargetHeight, 71.0f);

        tracker.reset();
        EXPECT_FALSE(tracker.getOutput().mTargetHeight.has_value());
        EXPECT_FALSE(tracker.getOutput().mSettled);
        EXPECT_FALSE(tracker.getOutput().mFrozen);
    }

    TEST(RagdollCoreSettledPelvisHeightTest, rejectsInvalidInputTransactionally)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mPelvisHeightSettleDuration = 0.1f;
        RagdollCore::SettledPelvisHeightTracker tracker;
        RagdollCore::SettledPelvisHeightInput input;
        input.mSupport = supported();
        input.mPelvisHeight = 70.0f;
        RagdollCore::SettledPelvisHeightOutput output;
        ASSERT_TRUE(tracker.step(input, 0.1f, profile, output));
        const auto before = tracker.getOutput();

        input.mPelvisHeight = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(tracker.step(input, 0.1f, profile, output));
        const auto after = tracker.getOutput();
        EXPECT_EQ(after.mTargetHeight, before.mTargetHeight);
        EXPECT_FLOAT_EQ(after.mCandidateTime, before.mCandidateTime);
        EXPECT_EQ(after.mCandidateValid, before.mCandidateValid);
        EXPECT_EQ(after.mSettled, before.mSettled);
        EXPECT_EQ(after.mFrozen, before.mFrozen);
    }

    TEST(RagdollCoreControllerTest, recoveryNeedsContinuousStandingAndCannotCompleteWhileSeated)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0;
        profile.mBlendBackDuration = 0.2f;
        RagdollCore::ActiveRagdollController controller;
        controller.reset(RagdollCore::ActiveControllerMode::BlendBack);
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported(10);
        input.mPelvisHeight = 10;
        RagdollCore::ActiveControllerOutput output;
        ASSERT_TRUE(controller.step(input, 5, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::BlendBack);
        input.mPelvisHeight = 80;
        input.mSupport = supported(80);
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::BlendBack);
        input.mSupport.mCenterOfMassSupported = false;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        input.mSupport.mCenterOfMassSupported = true;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::BlendBack);
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Standing);
    }

    TEST(RagdollCoreControllerTest, recoveryQualifiesMeasuredRestRatherThanMatchingWalkingInput)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0;
        profile.mBlendBackDuration = 0.2f;
        RagdollCore::ActiveRagdollController controller;
        controller.reset(RagdollCore::ActiveControllerMode::BlendBack);
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported(80);
        input.mPelvisHeight = 80;
        // A speed acceptable for locomoting Standing is still too fast to
        // release recovery assistance and declare that the rise has settled.
        input.mTargetHorizontalVelocity
            = JPH::Vec3(0, (profile.mStandingMaximumSpeed + profile.mSettledMaximumSpeed) / 2, 0);
        input.mCenterOfMassVelocity = input.mTargetHorizontalVelocity;
        RagdollCore::ActiveControllerOutput output;
        ASSERT_TRUE(controller.step(input, 1, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::BlendBack);
        input.mCenterOfMassVelocity = JPH::Vec3::sZero();
        ASSERT_TRUE(controller.step(input, 0.2f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Standing);
    }

    TEST(RagdollCoreControllerTest, recoveryCannotFinishInTheStandingSupportMargin)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0;
        profile.mBlendBackDuration = 0.2f;
        RagdollCore::ActiveRagdollController controller;
        controller.reset(RagdollCore::ActiveControllerMode::BlendBack);
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported(80);
        input.mPelvisHeight = 80;
        input.mSupport.mCenterOfMassDistance = profile.mSupportMargin / 2;
        RagdollCore::ActiveControllerOutput output;
        ASSERT_TRUE(controller.step(input, 1, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::BlendBack);
        input.mSupport.mCenterOfMassDistance = 0;
        ASSERT_TRUE(controller.step(input, 0.2f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Standing);
    }

    TEST(RagdollCoreControllerTest, heldKinematicRootStaysDrivenThroughHitStunUntilReleased)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0.0f;
        profile.mLocomotionRoute = RagdollCore::LocomotionRoute::KinematicRoot;
        RagdollCore::ActiveRagdollController controller;
        RagdollCore::ActiveControllerInput input;
        // A held body: upright, but the feet carry no load and the recoil
        // moves the centre of mass faster than any standing margin allows.
        input.mSupport = RagdollCore::SupportEstimate{};
        input.mPelvisHeight = 0.0f;
        input.mRootExternallyOwned = true;
        input.mActuationInterrupted = true;
        input.mCenterOfMassVelocity = JPH::Vec3(0, profile.mStandingMaximumSpeed * 4.0f, 0);
        RagdollCore::ActiveControllerOutput output;
        for (int i = 0; i < 60; ++i)
        {
            ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, profile, output));
            EXPECT_NE(output.mMode, RagdollCore::ActiveControllerMode::Falling);
            EXPECT_NE(output.mMode, RagdollCore::ActiveControllerMode::Fallen);
            EXPECT_TRUE(output.mDriveJointMotors);
        }

        // The knock-down decision releases the root; from then on the same
        // unsupported, interrupted body is a physical fall.
        input.mRootExternallyOwned = false;
        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Falling);
        EXPECT_FALSE(output.mDriveJointMotors);
    }

    TEST(RagdollCoreControllerTest, ownedRootVerticalReferenceAcceptsPrescribedDownAndUpMotion)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mLocomotionRoute = RagdollCore::LocomotionRoute::KinematicRoot;
        profile.mMinimumStateDuration = 0;
        RagdollCore::ActiveRagdollController controller;
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported();
        input.mPelvisHeight = 80;
        input.mRootExternallyOwned = true;
        RagdollCore::ActiveControllerOutput output;
        for (const float vertical : { -180.0f, -90.0f, 0.0f, 90.0f, 180.0f })
        {
            SCOPED_TRACE(vertical);
            input.mOwnedRootVerticalVelocity = vertical;
            input.mCenterOfMassVelocity = JPH::Vec3(0, 0, vertical);
            ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, profile, output));
            EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Standing);
        }
    }

    TEST(RagdollCoreControllerTest, ownedRootVerticalReferenceStillBracesForUnexplainedVerticalMotion)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mLocomotionRoute = RagdollCore::LocomotionRoute::KinematicRoot;
        profile.mMinimumStateDuration = 0;
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported();
        input.mPelvisHeight = 80;
        input.mRootExternallyOwned = true;
        for (const float rootVelocity : { 0.0f, -150.0f, 150.0f })
        {
            SCOPED_TRACE(rootVelocity);
            RagdollCore::ActiveRagdollController controller;
            RagdollCore::ActiveControllerOutput output;
            input.mOwnedRootVerticalVelocity = rootVelocity;
            input.mCenterOfMassVelocity = JPH::Vec3(0, 0, rootVelocity + 2 * profile.mStandingMaximumSpeed);
            ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, profile, output));
            EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);
        }
    }

    TEST(RagdollCoreControllerTest, ownedRootVerticalReferencePreservesHorizontalCommandAndError)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mLocomotionRoute = RagdollCore::LocomotionRoute::KinematicRoot;
        profile.mMinimumStateDuration = 0;
        RagdollCore::ActiveRagdollController controller;
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported();
        input.mPelvisHeight = 80;
        input.mRootExternallyOwned = true;
        input.mOwnedRootVerticalVelocity = -150;
        input.mCenterOfMassVelocity = JPH::Vec3(2 * profile.mStandingMaximumSpeed, 0, -150);
        RagdollCore::ActiveControllerOutput output;
        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);
        input.mTargetHorizontalVelocity = JPH::Vec3(input.mCenterOfMassVelocity.GetX(), 0, 0);
        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Standing);
    }

    TEST(RagdollCoreControllerTest, ownedRootVerticalReferenceDoesNotExemptOtherRoutesOrReleasedRoot)
    {
        using Route = RagdollCore::LocomotionRoute;
        for (const auto route : { Route::Legacy, Route::Reference, Route::KinematicRoot })
        {
            SCOPED_TRACE(static_cast<int>(route));
            RagdollCore::ActiveControllerProfile profile;
            profile.mLocomotionRoute = route;
            profile.mMinimumStateDuration = 0;
            RagdollCore::ActiveRagdollController controller;
            RagdollCore::ActiveControllerInput input;
            input.mSupport = supported();
            input.mPelvisHeight = 80;
            input.mRootExternallyOwned = route != Route::KinematicRoot;
            input.mOwnedRootVerticalVelocity = -150;
            input.mCenterOfMassVelocity = JPH::Vec3(0, 0, -150);
            RagdollCore::ActiveControllerOutput output;
            ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, profile, output));
            EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);
        }
    }

    TEST(RagdollCoreControllerTest, ownedRootVerticalReferenceAbsenceRetainsAbsoluteSpeedCheck)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mLocomotionRoute = RagdollCore::LocomotionRoute::KinematicRoot;
        profile.mMinimumStateDuration = 0;
        RagdollCore::ActiveRagdollController controller;
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported();
        input.mPelvisHeight = 80;
        input.mRootExternallyOwned = true;
        ASSERT_FALSE(input.mOwnedRootVerticalVelocity);
        RagdollCore::ActiveControllerOutput output;
        input.mCenterOfMassVelocity = JPH::Vec3(0, 0, profile.mStandingMaximumSpeed);
        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Standing);
        input.mCenterOfMassVelocity = JPH::Vec3(0, 0, 2 * profile.mStandingMaximumSpeed);
        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);
    }

    TEST(RagdollCoreControllerTest, ownedRootVerticalReferenceRejectsNonfiniteBeforeStateMutation)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mLocomotionRoute = RagdollCore::LocomotionRoute::KinematicRoot;
        RagdollCore::ActiveRagdollController controller;
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported();
        input.mPelvisHeight = 80;
        input.mRootExternallyOwned = true;
        input.mOwnedRootVerticalVelocity = 0;
        RagdollCore::ActiveControllerOutput output;
        ASSERT_TRUE(controller.step(input, 0.04f, profile, output));
        const auto before = controller.getOutput();
        for (const float invalid : { std::numeric_limits<float>::quiet_NaN(),
                 std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity() })
        {
            input.mOwnedRootVerticalVelocity = invalid;
            EXPECT_FALSE(controller.step(input, 10.0f, profile, output));
            EXPECT_EQ(controller.getOutput().mMode, before.mMode);
            EXPECT_FLOAT_EQ(controller.getOutput().mTimeInMode, before.mTimeInMode);
        }
    }

    TEST(RagdollCoreControllerTest, ownedRootVerticalReferenceDoesNotReplaceRawSettlingSpeed)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mLocomotionRoute = RagdollCore::LocomotionRoute::KinematicRoot;
        profile.mMinimumStateDuration = 0;
        RagdollCore::ActiveRagdollController controller;
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported(10);
        input.mPelvisHeight = 10;
        input.mPelvisUpDot = (profile.mStandingMinimumUpDot + profile.mBracingMinimumUpDot) / 2;
        input.mRootExternallyOwned = true;
        input.mOwnedRootVerticalVelocity = -150;
        input.mCenterOfMassVelocity = JPH::Vec3(0, 0, -150);
        RagdollCore::ActiveControllerOutput output;
        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, profile, output));
        // Relative motion is zero, but this fast moving body is not settled.
        // Nor may reference compensation bypass the original posture threshold.
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);
        input.mCenterOfMassVelocity = JPH::Vec3::sZero();
        input.mOwnedRootVerticalVelocity = 0;
        ASSERT_TRUE(controller.step(input, 1.0f / 60.0f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Fallen);
    }

    TEST(RagdollCoreControllerTest, ownedRootVerticalReferenceExplainsRecordedStaffDip)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mLocomotionRoute = RagdollCore::LocomotionRoute::KinematicRoot;
        profile.mMinimumStateDuration = 0;
        struct Sample { JPH::Vec3 mCom; float mCompletedRootVelocity; };
        // Native595 staff steps120-122. The root velocities are the preceding
        // MoveKinematic commands consumed by each solve, not the future root
        // velocity visible in the later same-row body debug observation.
        const std::array samples{
            Sample{ JPH::Vec3(-0.74089f, 27.10668f, -73.13477f), -88.10989f },
            Sample{ JPH::Vec3(-0.10704f, 38.97638f, -135.94179f), -158.74236f },
            Sample{ JPH::Vec3(-0.47160f, 36.80274f, -176.06804f), -205.28410f },
        };
        for (const auto& sample : samples)
        {
            RagdollCore::ActiveControllerInput input;
            input.mSupport = supported();
            input.mPelvisHeight = 80;
            input.mRootExternallyOwned = true;
            input.mCenterOfMassVelocity = sample.mCom;
            RagdollCore::ActiveRagdollController withoutMeasurement;
            RagdollCore::ActiveControllerOutput output;
            ASSERT_TRUE(withoutMeasurement.step(input, 1.0f / 60.0f, profile, output));
            EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);
            input.mOwnedRootVerticalVelocity = sample.mCompletedRootVelocity;
            RagdollCore::ActiveRagdollController withMeasurement;
            ASSERT_TRUE(withMeasurement.step(input, 1.0f / 60.0f, profile, output));
            EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Standing);
        }
    }

    TEST(RagdollCoreControllerTest, followsDeterministicFallAndRecoveryStateSequence)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0.0f;
        profile.mRecoveryDelay = 0.2f;
        profile.mBlendBackDuration = 0.2f;
        RagdollCore::ActiveRagdollController controller;
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported();
        input.mPelvisHeight = 80.0f;
        RagdollCore::ActiveControllerOutput output;

        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Standing);
        EXPECT_TRUE(output.mDriveJointMotors);

        input.mPelvisUpDot = 0.8f;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);

        input.mPelvisUpDot = 0.4f;
        input.mSupport.mCenterOfMassSupported = false;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Stumbling);

        input.mSupport = {};
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Falling);
        EXPECT_FALSE(output.mDriveJointMotors);

        input.mSupport = {};
        input.mSupport.mGroundContactCount = 1;
        input.mSupport.mCenterOfMassHeight = 10.0f;
        input.mPelvisHeight = 10.0f;
        input.mCenterOfMassVelocity = JPH::Vec3::sZero();
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Fallen);

        input.mRecoveryRequested = true;
        input.mTorsoUpDot = -0.5f;
        input.mTorsoFacingUpDot = -1.0f;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Fallen);
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::GettingUp);
        ASSERT_TRUE(output.mRecoveryDirection.has_value());
        EXPECT_EQ(*output.mRecoveryDirection, RagdollCore::RecoveryDirection::FaceDown);

        input.mSupport = {};
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::GettingUp);
        EXPECT_FALSE(output.mRecoveryHeightAssist);

        input.mPelvisUpDot = 0.7f;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_TRUE(output.mRecoveryHeightAssist);

        input.mSupport = supported(10.0f);
        input.mPelvisUpDot = -0.5f;
        input.mRecoveryAnimationFinished = true;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::BlendBack);
        EXPECT_TRUE(output.mRecoveryHeightAssist);
        input.mSupport = {};
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::BlendBack);
        input.mPelvisUpDot = 1.0f;
        input.mSupport = supported(80.0f);
        input.mPelvisHeight = 80.0f;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::BlendBack);
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Standing);
        EXPECT_FALSE(output.mRecoveryDirection.has_value());
    }

    TEST(RagdollCoreControllerTest, horizontalOrInvertedPelvisFallsDespiteFootContact)
    {
        using namespace RagdollCore;
        ActiveControllerProfile profile;
        ActiveControllerInput input;
        input.mSupport = supported();
        input.mPelvisHeight = 40; // Not settled below the fallen-height limit.
        ActiveControllerOutput output;
        for (const auto mode : { ActiveControllerMode::Standing, ActiveControllerMode::Bracing,
                 ActiveControllerMode::Stumbling })
            for (const float up : { 0.0f, -0.25f })
                for (const float speed : { 0.0f, 60.0f })
                {
                    ActiveRagdollController controller;
                    controller.reset(mode);
                    input.mPelvisUpDot = up;
                    input.mTargetHorizontalVelocity = JPH::Vec3(0, speed, 0);
                    ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
                    EXPECT_EQ(output.mMode, ActiveControllerMode::Falling);
                    EXPECT_FALSE(output.mDriveJointMotors);
                }
        // Recoverable upright leaning still permits a stumble; authored
        // recovery can start horizontal without being interrupted by this rule.
        ActiveRagdollController controller;
        input.mPelvisUpDot = 0.4f;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::Stumbling);
        controller.reset(ActiveControllerMode::GettingUp);
        input.mPelvisUpDot = -0.25f;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::GettingUp);
        EXPECT_TRUE(output.mDriveJointMotors);
    }

    TEST(RagdollCoreControllerTest, keepsMotorsDuringInitialContactAcquisitionOnly)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0.0f;
        profile.mActivationSupportGraceDuration = 0.15f;
        RagdollCore::ActiveRagdollController controller;
        RagdollCore::ActiveControllerInput input;
        input.mPelvisHeight = 80.0f;
        RagdollCore::ActiveControllerOutput output;

        ASSERT_TRUE(controller.step(input, 0.05f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);
        EXPECT_TRUE(output.mDriveJointMotors);
        ASSERT_TRUE(controller.step(input, 0.05f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);
        EXPECT_TRUE(output.mDriveJointMotors);

        input.mSupport = supported();
        ASSERT_TRUE(controller.step(input, 0.05f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Standing);

        input.mSupport = {};
        ASSERT_TRUE(controller.step(input, 0.01f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Falling);
        EXPECT_FALSE(output.mDriveJointMotors);
    }

    TEST(RagdollCoreControllerTest, stateTransitionMatrixMatchesPhysicalObservations)
    {
        using Mode = RagdollCore::ActiveControllerMode;
        enum class Observation
        {
            Upright,
            Braced,
            Stumbling,
            Unsupported,
            Settled,
            RecoverFaceUp,
            AnimationFinished,
        };
        struct Row
        {
            std::string_view mName;
            Mode mFrom;
            Observation mObservation;
            Mode mTo;
        };
        constexpr std::array rows{
            Row{ "standing remains standing", Mode::Standing, Observation::Upright, Mode::Standing },
            Row{ "standing braces", Mode::Standing, Observation::Braced, Mode::Bracing },
            Row{ "bracing stumbles", Mode::Bracing, Observation::Stumbling, Mode::Stumbling },
            Row{ "stumbling loses support", Mode::Stumbling, Observation::Unsupported, Mode::Falling },
            Row{ "standing settles low", Mode::Standing, Observation::Settled, Mode::Fallen },
            Row{ "falling settles low", Mode::Falling, Observation::Settled, Mode::Fallen },
            Row{ "falling regains stable support", Mode::Falling, Observation::Upright, Mode::Standing },
            Row{ "falling regains braced support", Mode::Falling, Observation::Braced, Mode::Bracing },
            Row{ "fallen without ground falls", Mode::Fallen, Observation::Unsupported, Mode::Falling },
            Row{ "fallen starts recovery", Mode::Fallen, Observation::RecoverFaceUp, Mode::GettingUp },
            Row{ "getting up ignores contact loss", Mode::GettingUp, Observation::Unsupported, Mode::GettingUp },
            Row{ "getting up completes animation", Mode::GettingUp, Observation::AnimationFinished,
                Mode::BlendBack },
            Row{ "blend back releases after bounded support loss", Mode::BlendBack, Observation::Unsupported,
                Mode::Falling },
            Row{ "blend back hands off when upright", Mode::BlendBack, Observation::Upright, Mode::Standing },
        };

        RagdollCore::ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0.0f;
        profile.mActivationSupportGraceDuration = 0.0f;
        profile.mRecoveryHandoverTimeout = 0.0f;
        profile.mFallingExitSupportDuration = 0.0f;
        profile.mRecoveryDelay = 0.0f;
        profile.mBlendBackDuration = 0.0f;
        for (const Row& row : rows)
        {
            SCOPED_TRACE(row.mName);
            RagdollCore::ActiveRagdollController controller;
            controller.reset(row.mFrom);
            RagdollCore::ActiveControllerInput input;
            input.mPelvisHeight = 80.0f;
            switch (row.mObservation)
            {
                case Observation::Upright:
                    input.mSupport = supported(80.0f);
                    break;
                case Observation::Braced:
                    input.mSupport = supported(80.0f);
                    input.mSupport.mCenterOfMassSupported = false;
                    input.mPelvisUpDot = 0.8f;
                    break;
                case Observation::Stumbling:
                    input.mSupport = supported(80.0f);
                    input.mSupport.mCenterOfMassSupported = false;
                    input.mPelvisUpDot = 0.4f;
                    break;
                case Observation::Unsupported:
                    input.mSupport = {};
                    break;
                case Observation::Settled:
                    input.mSupport = {};
                    input.mSupport.mGroundContactCount = 1;
                    input.mSupport.mCenterOfMassHeight = 10.0f;
                    input.mPelvisHeight = 10.0f;
                    break;
                case Observation::RecoverFaceUp:
                    input.mSupport = {};
                    input.mSupport.mGroundContactCount = 1;
                    input.mSupport.mCenterOfMassHeight = 10.0f;
                    input.mPelvisHeight = 10.0f;
                    input.mRecoveryRequested = true;
                    break;
                case Observation::AnimationFinished:
                    input.mSupport = supported(80.0f);
                    input.mRecoveryAnimationFinished = true;
                    break;
            }

            RagdollCore::ActiveControllerOutput output;
            ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
            EXPECT_EQ(output.mMode, row.mTo);
            EXPECT_EQ(output.mDriveJointMotors,
                row.mTo != Mode::Falling && row.mTo != Mode::Fallen);
            if (row.mObservation == Observation::RecoverFaceUp)
            {
                ASSERT_TRUE(output.mRecoveryDirection.has_value());
                EXPECT_EQ(*output.mRecoveryDirection, RagdollCore::RecoveryDirection::FaceUp);
            }
        }
    }

    TEST(RagdollCoreControllerTest, rejectsInvalidInputWithoutMutatingState)
    {
        RagdollCore::ActiveRagdollController controller;
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported();
        input.mPelvisHeight = 80.0f;
        RagdollCore::ActiveControllerOutput output;
        ASSERT_TRUE(controller.step(input, 0.1f, {}, output));
        const auto before = controller.getOutput();

        input.mCenterOfMassVelocity.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(controller.step(input, 0.1f, {}, output));
        const auto after = controller.getOutput();
        EXPECT_EQ(after.mMode, before.mMode);
        EXPECT_FLOAT_EQ(after.mTimeInMode, before.mTimeInMode);
        EXPECT_FALSE(controller.step({}, -0.1f, {}, output));

        input.mCenterOfMassVelocity = JPH::Vec3::sZero();
        input.mPelvisHeight = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(controller.step(input, 0.1f, {}, output));
    }

    TEST(RagdollCoreControllerTest, classifiesIntentionalLocomotionByTrackingError)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0.0f;
        profile.mMaximumLocomotionSpeed = 300.0f;
        RagdollCore::ActiveRagdollController controller;
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported();
        input.mPelvisHeight = 80.0f;
        input.mCenterOfMassVelocity = JPH::Vec3(200.0f, 0.0f, 0.0f);
        input.mTargetHorizontalVelocity = input.mCenterOfMassVelocity;
        RagdollCore::ActiveControllerOutput output;

        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Standing);

        input.mTargetHorizontalVelocity = JPH::Vec3::sZero();
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);

        input.mTargetHorizontalVelocity.SetY(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(controller.step(input, 0.1f, profile, output));
    }

    TEST(RagdollCoreControllerTest, gaitDoesNotRelaxThePhysicalBracingThreshold)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0.0f;
        RagdollCore::ActiveRagdollController controller;
        RagdollCore::ActiveControllerInput input;
        input.mSupport.mGroundContactCount = 1;
        input.mPelvisHeight = 80.0f;
        input.mCenterOfMassVelocity = JPH::Vec3(100.0f, 0.0f, 0.0f);
        input.mTargetHorizontalVelocity = input.mCenterOfMassVelocity;
        RagdollCore::ActiveControllerOutput output;

        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);
        EXPECT_TRUE(output.mDriveJointMotors);

        input.mPelvisUpDot = 0.4f;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Falling);
        EXPECT_FALSE(output.mDriveJointMotors);

        input.mTargetHorizontalVelocity = JPH::Vec3::sZero();
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Falling);

        controller.reset(RagdollCore::ActiveControllerMode::Falling);
        input.mPelvisUpDot = 1.0f;
        input.mTargetHorizontalVelocity = input.mCenterOfMassVelocity;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Falling);
        EXPECT_FALSE(output.mDriveJointMotors);

        controller.reset();
        input.mSupport = {};
        ASSERT_TRUE(controller.step(input, 0.05f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);
        ASSERT_TRUE(controller.step(input, 0.05f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Bracing);
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Falling);

        input.mTargetHorizontalVelocity = JPH::Vec3::sZero();
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Falling);
    }

    TEST(RagdollCoreControllerTest, requiresStablePhysicalSupportBeforeLeavingFalling)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0.0f;
        profile.mFallingExitSupportDuration = 0.2f;
        profile.mFallingExitMaximumVerticalSpeed = 100.0f;
        RagdollCore::ActiveRagdollController controller;
        controller.reset(RagdollCore::ActiveControllerMode::Falling);
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported();
        input.mPelvisHeight = 80.0f;
        input.mCenterOfMassVelocity = JPH::Vec3(0.0f, 0.0f, -150.0f);
        RagdollCore::ActiveControllerOutput output;

        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Falling);
        EXPECT_FLOAT_EQ(output.mFallingExitCandidateTime, 0.0f);

        input.mCenterOfMassVelocity = JPH::Vec3::sZero();
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Falling);
        EXPECT_FLOAT_EQ(output.mFallingExitCandidateTime, 0.1f);

        input.mSupport = {};
        ASSERT_TRUE(controller.step(input, 0.05f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Falling);
        EXPECT_FLOAT_EQ(output.mFallingExitCandidateTime, 0.0f);

        input.mSupport = supported();
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Falling);
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Standing);
        EXPECT_TRUE(output.mDriveJointMotors);
    }

    TEST(RagdollCoreControllerTest, doesNotPublishRecoveryDirectionBeforeStateTransition)
    {
        RagdollCore::ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0.5f;
        profile.mRecoveryDelay = 0.1f;
        RagdollCore::ActiveRagdollController controller;
        controller.reset(RagdollCore::ActiveControllerMode::Fallen);
        RagdollCore::ActiveControllerInput input;
        input.mSupport = supported(10.0f);
        input.mPelvisHeight = 10.0f;
        input.mRecoveryRequested = true;
        input.mTorsoUpDot = -1.0f;
        RagdollCore::ActiveControllerOutput output;

        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, RagdollCore::ActiveControllerMode::Fallen);
        EXPECT_FALSE(output.mRecoveryDirection.has_value());
    }
    TEST(RagdollCoreControllerTest, rootTorqueDistributionIncludesEveryRootJoint)
    {
        std::array<RagdollCore::JointTorqueRequest, 5> joints;
        for (std::size_t i = 1; i < joints.size(); ++i)
        {
            joints[i].mParent = i == 4 ? 1 : 0;
            joints[i].mMaximumAxisTorque = JPH::Vec3::sReplicate(100);
        }
        joints[1].mTorque = JPH::Vec3(3, -2, 5); // spine
        joints[2].mTorque = JPH::Vec3(1, 0, 2); // left hip
        joints[3].mTorque = JPH::Vec3(-2, 3, 1); // right hip
        joints[4].mTorque = JPH::Vec3(50, 40, 30); // not attached to root
        const JPH::Vec3 requested(4, 8, -6);
        const JPH::Vec3 correction(-6, -9, -2);
        for (const float leftLoad : { 0.0f, 0.25f, 1.0f })
        {
            const auto plan = RagdollCore::distributeRootTorque(joints, 2, 3, leftLoad, 1-leftLoad, requested);
            ASSERT_TRUE(plan);
            EXPECT_TRUE(plan->mSupported);
            EXPECT_FALSE(plan->mSaturated);
            EXPECT_TRUE(plan->mJointTorques[2].IsClose(joints[2].mTorque + correction * leftLoad));
            EXPECT_TRUE(plan->mJointTorques[3].IsClose(joints[3].mTorque + correction * (1-leftLoad)));
            EXPECT_EQ(plan->mJointTorques[1], joints[1].mTorque);
            EXPECT_EQ(plan->mJointTorques[4], joints[4].mTorque);
            EXPECT_TRUE(plan->mAppliedRootTorque.IsClose(requested));
            EXPECT_TRUE(plan->mRootTorqueResidual.IsNearZero());
            std::array<JPH::Vec3, 5> bodyTorque;
            bodyTorque.fill(JPH::Vec3::sZero());
            for (std::size_t i = 1; i < joints.size(); ++i)
            {
                bodyTorque[i] += plan->mJointTorques[i];
                bodyTorque[joints[i].mParent] -= plan->mJointTorques[i];
            }
            EXPECT_TRUE(bodyTorque[0].IsClose(requested));
            JPH::Vec3 total = JPH::Vec3::sZero();
            for (const auto torque : bodyTorque)
                total += torque;
            EXPECT_TRUE(total.IsNearZero());
        }
        const auto airborne = RagdollCore::distributeRootTorque(joints, 2, 3, 0, 0, requested);
        ASSERT_TRUE(airborne);
        EXPECT_FALSE(airborne->mSupported);
        for (std::size_t i = 0; i < joints.size(); ++i)
            EXPECT_EQ(airborne->mJointTorques[i], joints[i].mTorque);
        EXPECT_TRUE(airborne->mRootTorqueResidual.IsClose(JPH::Vec3(6, 9, 2)));
    }

    TEST(RagdollCoreControllerTest, rootTorqueDistributionReportsUnrealizedTorqueAtNativeAxisLimits)
    {
        std::array<RagdollCore::JointTorqueRequest, 4> joints;
        for (std::size_t i = 1; i < joints.size(); ++i)
        {
            joints[i].mParent = 0;
            joints[i].mMaximumAxisTorque = JPH::Vec3::sReplicate(100);
        }
        const auto frame = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.7f);
        joints[1].mMotorFrame = frame;
        joints[1].mMaximumAxisTorque = JPH::Vec3(2, 3, 0);
        joints[2].mMotorFrame = frame;
        joints[2].mMaximumAxisTorque = JPH::Vec3(10, 10, 0);
        joints[3].mMotorFrame = frame;
        joints[3].mMaximumAxisTorque = JPH::Vec3(1, 1, 0);
        joints[3].mTorque = frame * JPH::Vec3(20, 0, 0);
        const auto request = frame * JPH::Vec3(8, -7, 5);
        const auto plan = RagdollCore::distributeRootTorque(joints, 1, 2, 1, 0, request);
        ASSERT_TRUE(plan);
        EXPECT_TRUE(plan->mSaturated);
        EXPECT_TRUE(plan->mJointTorques[1].IsClose(frame * JPH::Vec3(-2, 3, 0), 1e-8f));
        EXPECT_TRUE(plan->mJointTorques[2].IsNearZero()); // No silent redistribution to unloaded leg.
        EXPECT_TRUE(plan->mJointTorques[3].IsClose(frame * JPH::Vec3(1, 0, 0), 1e-8f));
        EXPECT_TRUE(plan->mAppliedRootTorque.IsClose(frame * JPH::Vec3(1, -3, 0), 1e-8f));
        EXPECT_TRUE(plan->mRootTorqueResidual.IsClose(frame * JPH::Vec3(7, -4, 5), 1e-8f));
    }

    TEST(RagdollCoreControllerTest, rootTorqueDistributionRejectsInvalidTopologyAndLoads)
    {
        std::array<RagdollCore::JointTorqueRequest, 3> joints;
        joints[1].mParent = joints[2].mParent = 0;
        const auto calculate = [&](float load = 1.0f) {
            return RagdollCore::distributeRootTorque(joints, 1, 2, load, 0, JPH::Vec3::sAxisX());
        };
        ASSERT_TRUE(calculate());
        EXPECT_FALSE(calculate(-1));
        EXPECT_FALSE(calculate(std::numeric_limits<float>::quiet_NaN()));
        joints[2].mParent = 1;
        EXPECT_FALSE(calculate());
        joints[2].mParent = 0;
        joints[1].mMaximumAxisTorque.SetZ(-1);
        EXPECT_FALSE(calculate());
        joints[1].mMaximumAxisTorque = JPH::Vec3::sZero();
        joints[0].mTorque = JPH::Vec3::sAxisX();
        EXPECT_FALSE(calculate());
    }

    TEST(RagdollCoreControllerTest, recoveryOrientationDoesNotRotateTranslationBudgets)
    {
        using namespace RagdollCore;
        ActiveControllerProfile profile;
        profile.mMaximumPelvisForce = 1e9f;
        profile.mRecoveryUnweightFactor = 1;
        for (const bool unweight : { false, true })
            for (const bool friction : { false, true })
            {
                UprightControllerInput input;
                input.mTargetPelvisHeight = 60;
                input.mPelvisHeight = 40;
                input.mTotalMass = 10;
                input.mCenterOfMassOffset = JPH::Vec3(1, 2, 0);
                input.mRecoveryUnweight = unweight;
                input.mLimitToSupportFriction = friction;
                input.mSupportFriction = 0.8f;
                input.mGravityMagnitude = UnitModel::GravityUnitsPerSecondSquared;
                const auto upright = calculateUprightControl(input, profile);
                ASSERT_TRUE(upright);
                ASSERT_GT(upright->mPelvisForce.GetZ(), 0);
                input.mTargetUp = JPH::Vec3::sAxisY();
                const auto turning = calculateUprightControl(input, profile);
                ASSERT_TRUE(turning);
                EXPECT_TRUE(turning->mPelvisForce.IsClose(upright->mPelvisForce, 1e-6f));
                EXPECT_EQ(turning->mPelvisHorizontalForceSaturated, upright->mPelvisHorizontalForceSaturated);
                EXPECT_EQ(turning->mPelvisVerticalForceSaturated, upright->mPelvisVerticalForceSaturated);
                EXPECT_GT(turning->mPelvisTorque.Length(), upright->mPelvisTorque.Length());
            }
    }

    TEST(RagdollCoreControllerTest, recoveryDirectionUsesChestFacingWhenTorsoIsHorizontal)
    {
        using namespace RagdollCore;
        ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0;
        profile.mRecoveryDelay = 0;
        // Both back and stomach have a horizontal longitudinal torso axis.
        // Small signed tilts of that axis must not select the get-up side.
        for (const float tilt : { -0.01f, 0.0f, 0.01f })
            for (const float facing : { -1.0f, 1.0f })
            {
                ActiveRagdollController controller;
                controller.reset(ActiveControllerMode::Fallen);
                ActiveControllerInput input;
                input.mSupport.mGroundContactCount = 1;
                input.mPelvisHeight = 10;
                input.mPelvisUpDot = 0;
                input.mTorsoUpDot = tilt;
                input.mTorsoFacingUpDot = facing;
                input.mRecoveryRequested = true;
                ActiveControllerOutput output;
                ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
                ASSERT_EQ(output.mMode, ActiveControllerMode::GettingUp);
                EXPECT_EQ(output.mRecoveryDirection, facing > 0 ? RecoveryDirection::FaceUp : RecoveryDirection::FaceDown);
            }
    }

    TEST(RagdollCoreControllerTest, collapsedRecoverySettlingReturnsToFallenBeforeRetry)
    {
        using namespace RagdollCore;
        ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0.0f;
        profile.mRecoveryDelay = 0.2f;
        ActiveRagdollController controller;
        controller.reset(ActiveControllerMode::BlendBack);
        ActiveControllerInput input;
        input.mSupport.mGroundContactCount = 1;
        input.mPelvisHeight = 10.0f;
        input.mPelvisUpDot = 0.0f;
        input.mTorsoUpDot = -1.0f;
        input.mTorsoFacingUpDot = -1.0f;
        ActiveControllerOutput output;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::BlendBack);
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        ASSERT_EQ(output.mMode, ActiveControllerMode::Fallen);
        EXPECT_FALSE(output.mDriveJointMotors);
        EXPECT_FALSE(output.mRecoveryDirection.has_value());
        EXPECT_FALSE(output.mRecoveryHeightAssist);
        input.mRecoveryRequested = true;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::Fallen);
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::GettingUp);
        EXPECT_EQ(output.mRecoveryDirection, RecoveryDirection::FaceDown);
    }

    TEST(RagdollCoreControllerTest, recoverySupportLossReleasesMotorsAfterTheConfiguredGrace)
    {
        using namespace RagdollCore;
        ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0.0f;
        profile.mRecoveryHandoverTimeout = 0.2f;
        ActiveRagdollController controller;
        controller.reset(ActiveControllerMode::BlendBack);
        ActiveControllerInput input;
        input.mPelvisHeight = 70.0f;
        input.mPelvisUpDot = 0.7f;
        ActiveControllerOutput output;

        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::BlendBack);
        EXPECT_TRUE(output.mDriveJointMotors);
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::Falling);
        EXPECT_FALSE(output.mDriveJointMotors);
        EXPECT_FALSE(output.mRecoveryHeightAssist);
        EXPECT_FALSE(output.mRecoveryDirection.has_value());

        controller.reset(ActiveControllerMode::GettingUp);
        ASSERT_TRUE(controller.step(input, 1.0f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::GettingUp);
        EXPECT_TRUE(output.mDriveJointMotors);

        controller.reset(ActiveControllerMode::BlendBack);
        input.mSupport = supported(70.0f);
        input.mRecoveryInterrupted = true;
        ASSERT_TRUE(controller.step(input, 0.01f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::Falling);
        EXPECT_FALSE(output.mDriveJointMotors);

        controller.reset(ActiveControllerMode::GettingUp);
        ASSERT_TRUE(controller.step(input, 0.01f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::Falling);
        EXPECT_FALSE(output.mDriveJointMotors);
    }

    TEST(RagdollCoreControllerTest, recoveryOutsideSupportAllowsAlignmentProgressThenReleases)
    {
        using namespace RagdollCore;
        ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0;
        profile.mRecoveryDelay = profile.mBlendBackDuration = 0.2f;
        ActiveRagdollController controller;
        controller.reset(ActiveControllerMode::BlendBack);
        ActiveControllerInput input;
        input.mSupport = supported();
        input.mSupport.mCenterOfMassSupported = false;
        input.mSupport.mCenterOfMassDistance = 15;
        input.mPelvisHeight = 69;
        input.mPelvisUpDot = 0.70f;
        ActiveControllerOutput output;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::BlendBack);
        input.mSupport.mCenterOfMassDistance = 14;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::BlendBack);
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::Falling);
        EXPECT_FALSE(output.mDriveJointMotors);
    }

    TEST(RagdollCoreControllerTest, highRecoveryStallAllowsProgressThenReleasesToFalling)
    {
        using namespace RagdollCore;
        ActiveControllerProfile profile;
        profile.mMinimumStateDuration = 0;
        profile.mRecoveryDelay = profile.mBlendBackDuration = 0.2f;
        ActiveRagdollController controller;
        controller.reset(ActiveControllerMode::BlendBack);
        ActiveControllerInput input;
        input.mSupport.mGroundContactCount = 1;
        input.mPelvisHeight = 75;
        input.mPelvisUpDot = 0.45f;
        input.mCenterOfMassVelocity = JPH::Vec3(6, 0, 0);
        ActiveControllerOutput output;
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::BlendBack);
        input.mPelvisUpDot = 0.48f; // Real progress must restart the stall interval.
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::BlendBack);
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::Falling);
        EXPECT_FALSE(output.mDriveJointMotors);
        EXPECT_FALSE(output.mRecoveryHeightAssist);
        EXPECT_FALSE(output.mRecoveryDirection.has_value());
        controller.reset(ActiveControllerMode::BlendBack);
        ASSERT_TRUE(controller.step(input, 0.1f, profile, output));
        EXPECT_EQ(output.mMode, ActiveControllerMode::BlendBack);
    }

}
