#include <ragdollcore/balancemetrics.hpp>
#include <ragdollcore/unitmodel.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
    using namespace RagdollCore;
    using Availability = BalanceSupportAvailability;

    BodyTelemetry body(float mass, JPH::RVec3Arg position, JPH::Vec3Arg velocity, bool active = true)
    {
        BodyTelemetry result;
        result.mMass = mass;
        result.mCenterOfMass = position;
        result.mMotion.mLinearVelocity = velocity;
        result.mActive = active;
        return result;
    }

    BalanceContactPoint contact(std::size_t foot, double x, double y, double z = 0.0)
    {
        BalanceContactPoint result;
        result.mContact.mBodyIndex = foot;
        result.mContact.mPosition = JPH::RVec3(x, y, z);
        result.mContact.mNormalImpulse = 1.0f;
        result.mContact.mPointVelocity = JPH::Vec3::sZero();
        result.mSurfacePointVelocity = JPH::Vec3::sZero();
        return result;
    }

    std::vector<BalanceContactPoint> rectangularFoot()
    {
        return { contact(1, -2, -4), contact(1, 2, -4), contact(1, 2, 4), contact(1, -2, 4) };
    }

    TEST(RagdollCoreBalanceMassTest, includesSleepingBodiesAndDeclaredKinematicRoot)
    {
        // An inactive/sleeping mass must not disappear, and the declared root
        // contributes its retained physical mass without an inverse-mass query.
        const std::array bodies{ body(2, JPH::RVec3(0, 0, 10), JPH::Vec3(4, 0, 0), false),
            body(6, JPH::RVec3(8, 0, 14), JPH::Vec3::sZero(), false),
            body(2, JPH::RVec3(6, 5, 12), JPH::Vec3(0, 5, 0)) };
        const auto result = measureBalanceMass(bodies, 0);
        ASSERT_TRUE(result);
        EXPECT_FLOAT_EQ(result->mTotalMass, 10.0f);
        EXPECT_DOUBLE_EQ(result->mCenterOfMass.GetX(), 6.0);
        EXPECT_DOUBLE_EQ(result->mCenterOfMass.GetY(), 1.0);
        EXPECT_DOUBLE_EQ(result->mCenterOfMass.GetZ(), 12.8);
        EXPECT_FLOAT_EQ(result->mCenterOfMassVelocity.GetX(), 0.8f);
        EXPECT_FLOAT_EQ(result->mCenterOfMassVelocity.GetY(), 1.0f);
        ASSERT_TRUE(result->mKinematicRootBody);
        EXPECT_EQ(*result->mKinematicRootBody, 0u);
    }

    TEST(RagdollCoreBalanceMassTest, changingOnlyActivationCannotMoveCenterOfMass)
    {
        std::array bodies{ body(1, JPH::RVec3(0, 0, 1), JPH::Vec3::sZero()),
            body(3, JPH::RVec3(8, 0, 1), JPH::Vec3(4, 0, 0)) };
        const auto awake = measureBalanceMass(bodies);
        bodies[0].mActive = false;
        bodies[1].mActive = false;
        const auto sleeping = measureBalanceMass(bodies);
        ASSERT_TRUE(awake && sleeping);
        EXPECT_EQ(awake->mCenterOfMass, sleeping->mCenterOfMass);
        EXPECT_EQ(awake->mCenterOfMassVelocity, sleeping->mCenterOfMassVelocity);
        EXPECT_FLOAT_EQ(awake->mTotalMass, sleeping->mTotalMass);
        EXPECT_FALSE(sleeping->mKinematicRootBody);
    }

    TEST(RagdollCoreBalanceMassTest, retainsLocalPrecisionFarFromOrigin)
    {
        const double origin = 1e10;
        const std::array bodies{ body(1, JPH::RVec3(origin, -origin, 10), JPH::Vec3::sZero()),
            body(3, JPH::RVec3(origin + 4, -origin + 8, 10), JPH::Vec3::sZero()) };
        const auto result = measureBalanceMass(bodies);
        ASSERT_TRUE(result);
        EXPECT_DOUBLE_EQ(result->mCenterOfMass.GetX() - origin, 3.0);
        EXPECT_DOUBLE_EQ(result->mCenterOfMass.GetY() + origin, 6.0);
    }

    TEST(RagdollCoreBalanceMassTest, rejectsIncompleteOrInvalidBodyAssembly)
    {
        EXPECT_FALSE(measureBalanceMass({}));
        std::array bodies{ body(1, JPH::RVec3(0, 0, 1), JPH::Vec3::sZero()) };
        EXPECT_FALSE(measureBalanceMass(bodies, 1));
        bodies[0].mMass = 0;
        EXPECT_FALSE(measureBalanceMass(bodies, 0));
        bodies[0].mMass = -1;
        EXPECT_FALSE(measureBalanceMass(bodies));
        bodies[0].mMass = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(measureBalanceMass(bodies));
        bodies[0].mMass = 1;
        bodies[0].mMotion.mLinearVelocity.SetX(std::numeric_limits<float>::infinity());
        EXPECT_FALSE(measureBalanceMass(bodies));
    }

    TEST(RagdollCoreBalanceSupportTest, usesActualSingleFootPatchAndExactOutsideDistance)
    {
        const auto contacts = rectangularFoot();
        const auto inside = measureBalanceSupport(JPH::RVec3(1, 3, 50), contacts, { 1, 2 }, {}, true);
        ASSERT_EQ(inside.mAvailability, Availability::Supported);
        EXPECT_TRUE(inside.mLeftFootSupported);
        EXPECT_FALSE(inside.mRightFootSupported);
        ASSERT_TRUE(inside.mHorizontalDistance && inside.mClosestPoint);
        EXPECT_FLOAT_EQ(*inside.mHorizontalDistance, 0.0f);
        EXPECT_EQ(*inside.mClosestPoint, JPH::RVec3(1, 3, 50));
        const auto outside = measureBalanceSupport(JPH::RVec3(2.25, 3, 50), contacts, { 1, 2 }, {}, true);
        ASSERT_TRUE(outside.mHorizontalDistance && outside.mClosestPoint);
        // The controller's permissive SupportMargin must not turn this into 0.
        EXPECT_FLOAT_EQ(*outside.mHorizontalDistance, 0.25f);
        EXPECT_EQ(*outside.mClosestPoint, JPH::RVec3(2, 3, 50));
    }

    TEST(RagdollCoreBalanceSupportTest, projectsToDoubleFootConvexHull)
    {
        const std::array contacts{ contact(1, -4, -2), contact(1, -4, 2), contact(2, 4, -2), contact(2, 4, 2) };
        const auto result = measureBalanceSupport(JPH::RVec3(0, 3, 10), contacts, { 1, 2 }, {}, true);
        ASSERT_EQ(result.mAvailability, Availability::Supported);
        EXPECT_TRUE(result.mLeftFootSupported && result.mRightFootSupported);
        ASSERT_TRUE(result.mHorizontalDistance && result.mClosestPoint);
        EXPECT_FLOAT_EQ(*result.mHorizontalDistance, 1.0f);
        EXPECT_EQ(*result.mClosestPoint, JPH::RVec3(0, 2, 10));
    }

    TEST(RagdollCoreBalanceSupportTest, duplicateAndInteriorSamplesCannotAlterHullDistance)
    {
        auto contacts = rectangularFoot();
        contacts[0].mContact.mPosition.SetZ(-1);
        contacts[2].mContact.mPosition.SetZ(1);
        const auto baseline = measureBalanceSupport(JPH::RVec3(4, 6, 20), contacts, { 1, 2 }, {}, true);
        for (int i = 0; i < 20; ++i)
            contacts.push_back(contacts.front());
        contacts.push_back(contact(1, 0, 0));
        std::reverse(contacts.begin(), contacts.end());
        const auto duplicated = measureBalanceSupport(JPH::RVec3(4, 6, 20), contacts, { 1, 2 }, {}, true);
        ASSERT_EQ(baseline.mAvailability, Availability::Supported);
        ASSERT_EQ(duplicated.mAvailability, Availability::Supported);
        EXPECT_EQ(baseline.mHorizontalDistance, duplicated.mHorizontalDistance);
        EXPECT_EQ(baseline.mClosestPoint, duplicated.mClosestPoint);
        EXPECT_EQ(baseline.mMinimumContactHeight, duplicated.mMinimumContactHeight);
        EXPECT_EQ(baseline.mMaximumContactHeight, duplicated.mMaximumContactHeight);
        EXPECT_DOUBLE_EQ(*duplicated.mMinimumContactHeight, -1.0);
        EXPECT_DOUBLE_EQ(*duplicated.mMaximumContactHeight, 1.0);
    }

    TEST(RagdollCoreBalanceSupportTest, handlesPointAndCollinearSupportWithoutInventedArea)
    {
        std::vector<BalanceContactPoint> contacts{ contact(1, 0, 0) };
        const auto point = measureBalanceSupport(JPH::RVec3(3, 4, 20), contacts, { 1, 2 }, {}, true);
        ASSERT_TRUE(point.mHorizontalDistance);
        EXPECT_FLOAT_EQ(*point.mHorizontalDistance, 5.0f);
        contacts.push_back(contact(2, 6, 0));
        contacts.push_back(contact(1, 3, 0));
        const auto line = measureBalanceSupport(JPH::RVec3(3, 4, 20), contacts, { 1, 2 }, {}, true);
        ASSERT_TRUE(line.mHorizontalDistance);
        EXPECT_FLOAT_EQ(*line.mHorizontalDistance, 4.0f);
    }

    TEST(RagdollCoreBalanceSupportTest, distinguishesKnownNoSupportFromAbsentTelemetry)
    {
        const auto noSupport = measureBalanceSupport(JPH::RVec3(0, 0, 50), {}, { 1, 2 }, {}, true);
        EXPECT_EQ(noSupport.mAvailability, Availability::Unsupported);
        EXPECT_FALSE(noSupport.mHorizontalDistance || noSupport.mClosestPoint || noSupport.mMinimumContactHeight);
        const auto missing = measureBalanceSupport(JPH::RVec3(0, 0, 50), {}, { 1, 2 }, {}, false);
        EXPECT_EQ(missing.mAvailability, Availability::Unknown);
        EXPECT_FALSE(missing.mHorizontalDistance);
        auto contacts = rectangularFoot();
        contacts.push_back(contact(2, 10, 0));
        contacts.back().mContact.mNormalImpulse.reset();
        const auto partial = measureBalanceSupport(JPH::RVec3(0, 0, 50), contacts, { 1, 2 }, {}, true);
        EXPECT_EQ(partial.mAvailability, Availability::Unknown);
        EXPECT_FALSE(partial.mHorizontalDistance);
    }

    TEST(RagdollCoreBalanceSupportTest, requiresActualLoadAndUpwardFootNormal)
    {
        std::array contacts{ contact(1, 0, 0), contact(2, 0, 0), contact(9, 0, 0) };
        contacts[0].mContact.mNormalImpulse = 0.0f;
        contacts[0].mContact.mPenetrationDepth = 10;
        contacts[1].mContact.mNormal = JPH::Vec3::sAxisX();
        contacts[1].mContact.mNormalImpulse.reset();
        const auto unloaded = measureBalanceSupport(JPH::RVec3(0, 0, 50), contacts, { 1, 2 }, {}, true);
        EXPECT_EQ(unloaded.mAvailability, Availability::Unsupported);
        contacts[0].mContact.mNormalImpulse = 1.0f;
        contacts[0].mContact.mPenetrationDepth = -0.1f;
        const auto speculativeSolved = measureBalanceSupport(JPH::RVec3(0, 0, 50), contacts, { 1, 2 }, {}, true);
        EXPECT_EQ(speculativeSolved.mAvailability, Availability::Supported);
    }

    TEST(RagdollCoreBalanceSupportTest, invalidQualifiedContactCannotProducePartialHull)
    {
        auto contacts = rectangularFoot();
        contacts.back().mContact.mNormalImpulse = -1.0f;
        EXPECT_EQ(measureBalanceSupport(JPH::RVec3(0, 0, 50), contacts, { 1, 2 }, {}, true).mAvailability,
            Availability::Unknown);
        contacts.back().mContact.mNormalImpulse = 1.0f;
        contacts.back().mContact.mNormal = JPH::Vec3::sZero();
        EXPECT_EQ(measureBalanceSupport(JPH::RVec3(0, 0, 50), contacts, { 1, 2 }, {}, true).mAvailability,
            Availability::Unknown);
        EXPECT_EQ(measureBalanceSupport(JPH::RVec3(0, 0, 50), {}, { 1, 1 }, {}, true).mAvailability,
            Availability::Unknown);
    }

    TEST(RagdollCoreBalanceSupportTest, unrepresentableDistanceStaysUnknown)
    {
        const std::array contacts{ contact(1, 0, 0) };
        const auto result = measureBalanceSupport(
            JPH::RVec3(std::numeric_limits<double>::max(), 0, 50), contacts, { 1, 2 }, {}, true);
        EXPECT_EQ(result.mAvailability, Availability::Unknown);
        EXPECT_FALSE(result.mHorizontalDistance || result.mClosestPoint);
    }

    TEST(RagdollCoreBalanceCaptureTest, usesWorldUnitGravityAndHorizontalVelocity)
    {
        BalanceMassMeasurement mass;
        mass.mTotalMass = 10;
        mass.mCenterOfMass = JPH::RVec3(10, 20, 30 + UnitModel::GravityUnitsPerSecondSquared);
        mass.mCenterOfMassVelocity = JPH::Vec3(3, -4, 100);
        const auto result = calculateBalanceCapturePoint(mass, 30, UnitModel::GravityUnitsPerSecondSquared);
        ASSERT_TRUE(result);
        EXPECT_NEAR(result->GetX(), 13.0, 1e-6);
        EXPECT_NEAR(result->GetY(), 16.0, 1e-6);
        EXPECT_DOUBLE_EQ(result->GetZ(), 30.0);
    }

    TEST(RagdollCoreBalanceCaptureTest, projectsCapturePointAgainstPatchRatherThanFootCenter)
    {
        BalanceMassMeasurement mass;
        mass.mTotalMass = 10;
        mass.mCenterOfMass = JPH::RVec3(0, 0, 10);
        mass.mCenterOfMassVelocity = JPH::Vec3(3, 0, 0);
        const auto capture = calculateBalanceCapturePoint(mass, 0, 10);
        ASSERT_TRUE(capture);
        const auto support = measureBalanceSupport(*capture, rectangularFoot(), { 1, 2 }, {}, true);
        ASSERT_TRUE(support.mHorizontalDistance);
        EXPECT_FLOAT_EQ(*support.mHorizontalDistance, 1.0f);
    }

    TEST(RagdollCoreBalanceCaptureTest, rejectsUnknownMassAndInvalidHeightInsteadOfClamping)
    {
        BalanceMassMeasurement mass;
        mass.mCenterOfMass = JPH::RVec3(0, 0, 1);
        EXPECT_FALSE(calculateBalanceCapturePoint(mass, 0, 10));
        mass.mTotalMass = 1;
        EXPECT_FALSE(calculateBalanceCapturePoint(mass, 1, 10));
        EXPECT_FALSE(calculateBalanceCapturePoint(mass, 2, 10));
        EXPECT_FALSE(calculateBalanceCapturePoint(mass, 0, 0));
        EXPECT_FALSE(calculateBalanceCapturePoint(mass, std::numeric_limits<double>::quiet_NaN(), 10));
    }

    TEST(RagdollCoreBalanceSlipTest, subtractsMovingSurfaceVelocityAtContact)
    {
        std::array contacts{ contact(1, 0, 0) };
        contacts[0].mContact.mPointVelocity = JPH::Vec3(10, 2, 7);
        contacts[0].mSurfacePointVelocity = JPH::Vec3(10, 2, -9);
        const auto carried = measureBalanceFootSlip(contacts, 1, 0.5f, true, 1.0f / 60.0f);
        ASSERT_EQ(carried.mAvailability, Availability::Supported);
        ASSERT_TRUE(carried.mMaximumTangentSpeed && carried.mMaximumTangentDistanceThisStep);
        EXPECT_FLOAT_EQ(*carried.mMaximumTangentSpeed, 0);
        EXPECT_FLOAT_EQ(*carried.mMaximumTangentDistanceThisStep, 0);
        contacts[0].mSurfacePointVelocity = JPH::Vec3(7, -2, -9);
        const auto slipping = measureBalanceFootSlip(contacts, 1, 0.5f, true, 0.1f);
        ASSERT_TRUE(slipping.mMaximumTangentSpeed && slipping.mMaximumTangentDistanceThisStep);
        EXPECT_FLOAT_EQ(*slipping.mMaximumTangentSpeed, 5);
        EXPECT_FLOAT_EQ(*slipping.mMaximumTangentDistanceThisStep, 0.5f);
    }

    TEST(RagdollCoreBalanceSlipTest, removesNormalVelocityOnSlopedContact)
    {
        std::array contacts{ contact(1, 0, 0) };
        contacts[0].mContact.mNormal = JPH::Vec3(0, 3, 4);
        contacts[0].mContact.mPointVelocity = JPH::Vec3(2, 6, 8);
        const auto result = measureBalanceFootSlip(contacts, 1, 0.5f, true, 0.25f);
        ASSERT_TRUE(result.mMaximumTangentSpeed && result.mMaximumTangentDistanceThisStep);
        EXPECT_NEAR(*result.mMaximumTangentSpeed, 2, 1e-5f);
        EXPECT_NEAR(*result.mMaximumTangentDistanceThisStep, 0.5f, 1e-5f);
    }

    TEST(RagdollCoreBalanceSlipTest, duplicateManifoldsDoNotMultiplySlipDistance)
    {
        std::vector contacts{ contact(1, -2, 0), contact(1, 2, 0), contact(2, 0, 0) };
        // Distinct velocities can arise from rotation. Measure the maximum,
        // avoiding cancellation between opposite contact-point directions.
        contacts[0].mContact.mPointVelocity = JPH::Vec3(3, 0, 0);
        contacts[1].mContact.mPointVelocity = JPH::Vec3(-6, 0, 0);
        contacts[2].mContact.mPointVelocity = JPH::Vec3(100, 0, 0);
        const auto baseline = measureBalanceFootSlip(contacts, 1, 0.5f, true, 0.1f);
        for (int i = 0; i < 30; ++i)
            contacts.push_back(contacts[1]);
        std::reverse(contacts.begin(), contacts.end());
        const auto duplicated = measureBalanceFootSlip(contacts, 1, 0.5f, true, 0.1f);
        ASSERT_TRUE(baseline.mMaximumTangentSpeed && baseline.mMaximumTangentDistanceThisStep);
        EXPECT_FLOAT_EQ(*baseline.mMaximumTangentSpeed, 6);
        EXPECT_FLOAT_EQ(*baseline.mMaximumTangentDistanceThisStep, 0.6f);
        EXPECT_EQ(baseline.mMaximumTangentSpeed, duplicated.mMaximumTangentSpeed);
        EXPECT_EQ(baseline.mMaximumTangentDistanceThisStep, duplicated.mMaximumTangentDistanceThisStep);
        const auto repeated = measureBalanceFootSlip(contacts, 1, 0.5f, true, 0.1f);
        EXPECT_EQ(repeated.mMaximumTangentDistanceThisStep, duplicated.mMaximumTangentDistanceThisStep);
    }

    TEST(RagdollCoreBalanceSlipTest, missingSurfaceOrFootVelocityIsUnknownNotStatic)
    {
        std::array contacts{ contact(1, 0, 0), contact(1, 1, 0) };
        contacts[0].mContact.mPointVelocity = JPH::Vec3(2, 0, 0);
        contacts[1].mSurfacePointVelocity.reset();
        const auto missingSurface = measureBalanceFootSlip(contacts, 1, 0.5f, true, 0.1f);
        EXPECT_EQ(missingSurface.mAvailability, Availability::Supported);
        EXPECT_FALSE(missingSurface.mMaximumTangentSpeed || missingSurface.mMaximumTangentDistanceThisStep);
        contacts[1].mSurfacePointVelocity = JPH::Vec3::sZero();
        contacts[1].mContact.mPointVelocity.reset();
        const auto missingFoot = measureBalanceFootSlip(contacts, 1, 0.5f, true, 0.1f);
        EXPECT_FALSE(missingFoot.mMaximumTangentSpeed || missingFoot.mMaximumTangentDistanceThisStep);
        contacts[1].mContact.mPointVelocity = JPH::Vec3(std::numeric_limits<float>::quiet_NaN(), 0, 0);
        EXPECT_FALSE(measureBalanceFootSlip(contacts, 1, 0.5f, true, 0.1f).mMaximumTangentSpeed);
    }

    TEST(RagdollCoreBalanceSlipTest, unsupportedAndMissingContactDataCannotClaimZeroSlip)
    {
        const auto empty = measureBalanceFootSlip({}, 1, 0.5f, true, 0.1f);
        EXPECT_EQ(empty.mAvailability, Availability::Unsupported);
        EXPECT_FALSE(empty.mMaximumTangentSpeed || empty.mMaximumTangentDistanceThisStep);
        const auto incomplete = measureBalanceFootSlip({}, 1, 0.5f, false, 0.1f);
        EXPECT_EQ(incomplete.mAvailability, Availability::Unknown);
        EXPECT_FALSE(incomplete.mMaximumTangentSpeed);
        std::array contacts{ contact(1, 0, 0) };
        contacts[0].mContact.mNormalImpulse.reset();
        const auto missingLoad = measureBalanceFootSlip(contacts, 1, 0.5f, true, 0.1f);
        EXPECT_EQ(missingLoad.mAvailability, Availability::Unknown);
        EXPECT_FALSE(missingLoad.mMaximumTangentSpeed);
    }

    TEST(RagdollCoreBalanceSlipTest, invalidDurationCannotCreatePerStepDistance)
    {
        std::array contacts{ contact(1, 0, 0) };
        contacts[0].mContact.mPointVelocity = JPH::Vec3(1, 0, 0);
        for (const float duration : { 0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN() })
        {
            const auto result = measureBalanceFootSlip(contacts, 1, 0.5f, true, duration);
            ASSERT_TRUE(result.mMaximumTangentSpeed);
            EXPECT_FLOAT_EQ(*result.mMaximumTangentSpeed, 1);
            EXPECT_FALSE(result.mMaximumTangentDistanceThisStep);
        }
    }
}
