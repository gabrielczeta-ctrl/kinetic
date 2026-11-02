#include <ragdollcore/supportgrip.hpp>

// clang-format off
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/MotionProperties.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
// clang-format on

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <numbers>
#include <vector>

namespace
{
    // This is a free rigid-body mechanism fixture, not a native gameplay
    // qualification or a selection of acquisition/break tuning values.
    constexpr JPH::ObjectLayer sObjectLayer = 1;
    constexpr float sStep = 1.0f / 60.0f;

    class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterface
    {
    public:
        JPH::uint GetNumBroadPhaseLayers() const override { return 1; }
        JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer) const override { return JPH::BroadPhaseLayer(0); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
        const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override { return "support-grip-test"; }
#endif
    };

    class ObjectVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter
    {
    public:
        bool ShouldCollide(JPH::ObjectLayer, JPH::BroadPhaseLayer) const override { return true; }
    };

    class ObjectPairs final : public JPH::ObjectLayerPairFilter
    {
    public:
        bool ShouldCollide(JPH::ObjectLayer, JPH::ObjectLayer) const override { return true; }
    };

    struct BodyMotion
    {
        JPH::Vec3 mLinearMomentum;
        JPH::Vec3 mAngularMomentum;
        float mEnergy;
    };

    class SupportGripTest : public testing::Test
    {
    protected:
        void SetUp() override
        {
            mPhysics.Init(128, 0, 128, 128, mBroadPhaseLayers, mObjectVsBroadPhase, mObjectPairs);
            mPhysics.SetGravity(JPH::Vec3::sZero());
        }

        void TearDown() override
        {
            // Local SupportGrip owners must already have destroyed/cleared
            // their constraints before any fixture body is removed.
            EXPECT_TRUE(mPhysics.GetConstraints().empty());
            auto& bodies = mPhysics.GetBodyInterface();
            for (const auto id : mBodies)
            {
                bodies.RemoveBody(id);
                bodies.DestroyBody(id);
            }
        }

        JPH::BodyID add(JPH::RVec3Arg position, float mass = 2,
            JPH::QuatArg rotation = JPH::Quat::sIdentity(), const JPH::Shape* shape = nullptr,
            JPH::EMotionType motion = JPH::EMotionType::Dynamic)
        {
            JPH::RefConst<JPH::Shape> ownedShape = shape ? shape : new JPH::SphereShape(0.2f);
            JPH::BodyCreationSettings settings(ownedShape, position, rotation, motion, sObjectLayer);
            settings.mLinearDamping = settings.mAngularDamping = 0;
            settings.mAllowSleeping = false;
            settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            settings.mMassPropertiesOverride.mMass = mass;
            const auto id = mPhysics.GetBodyInterface().CreateAndAddBody(settings,
                motion == JPH::EMotionType::Static ? JPH::EActivation::DontActivate : JPH::EActivation::Activate);
            EXPECT_FALSE(id.IsInvalid());
            if (!id.IsInvalid())
                mBodies.push_back(id);
            return id;
        }

        RagdollCore::SupportGripRequest pair()
        {
            return { add(JPH::RVec3(-1, 0, 0), 2), add(JPH::RVec3(1, 0, 0), 3),
                JPH::Vec3(1, 0, 0), JPH::Vec3(-1, 0, 0) };
        }

        void step()
        {
            ASSERT_EQ(mPhysics.Update(sStep, 1, &mAllocator, &mJobs), JPH::EPhysicsUpdateError::None);
        }

        BodyMotion motion(JPH::BodyID id)
        {
            JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), id);
            EXPECT_TRUE(lock.Succeeded());
            const auto& body = lock.GetBody();
            const auto linear = body.GetLinearVelocity() / body.GetMotionProperties()->GetInverseMass();
            const auto spin = body.GetInverseInertia().Inversed3x3().Multiply3x3(body.GetAngularVelocity());
            return { linear, spin + JPH::Vec3(body.GetCenterOfMassPosition()).Cross(linear),
                0.5f * (linear.Dot(body.GetLinearVelocity()) + spin.Dot(body.GetAngularVelocity())) };
        }

        BroadPhaseLayers mBroadPhaseLayers;
        ObjectVsBroadPhase mObjectVsBroadPhase;
        ObjectPairs mObjectPairs;
        JPH::TempAllocatorImpl mAllocator{ 1024 * 1024 };
        JPH::JobSystemSingleThreaded mJobs{ JPH::cMaxPhysicsJobs };
        JPH::PhysicsSystem mPhysics;
        std::vector<JPH::BodyID> mBodies;
    };

    TEST_F(SupportGripTest, rejectsInvalidSeparatedAndUnaddedBodiesWithoutChangingTheWorld)
    {
        using Result = RagdollCore::SupportGripAcquireResult;
        RagdollCore::SupportGrip grip(mPhysics);
        auto request = pair();
        const auto originalPosition = mPhysics.GetBodyInterface().GetPosition(request.mSecondBody);
        auto invalid = request;
        invalid.mSecondBody = invalid.mFirstBody;
        EXPECT_EQ(grip.acquire(invalid, { 0.01f, 0.1f }), Result::InvalidBodies);
        invalid.mSecondBody = JPH::BodyID();
        EXPECT_EQ(grip.acquire(invalid, { 0.01f, 0.1f }), Result::InvalidBodies);
        invalid = request;
        invalid.mFirstLocalAnchor.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_EQ(grip.acquire(invalid, { 0.01f, 0.1f }), Result::InvalidRequest);
        EXPECT_EQ(grip.acquire(request, { -1, 1 }), Result::InvalidRequest);
        EXPECT_EQ(grip.acquire(request, { 1, std::numeric_limits<float>::infinity() }), Result::InvalidRequest);
        auto separated = request;
        separated.mSecondLocalAnchor = JPH::Vec3::sZero();
        EXPECT_EQ(grip.acquire(separated, { 0.01f, 0.1f }), Result::TooFar);
        auto& bodies = mPhysics.GetBodyInterface();
        bodies.RemoveBody(request.mSecondBody);
        EXPECT_EQ(grip.acquire(request, { 0.01f, 0.1f }), Result::InvalidBodies);
        bodies.AddBody(request.mSecondBody, JPH::EActivation::Activate);
        auto anchored = request;
        anchored.mSecondBody = add(JPH::RVec3(5, 0, 0), 1, JPH::Quat::sIdentity(), nullptr,
            JPH::EMotionType::Static);
        EXPECT_EQ(grip.acquire(anchored, { 10, 1 }), Result::InvalidBodies);
        EXPECT_EQ(bodies.GetPosition(request.mSecondBody), originalPosition);
        EXPECT_TRUE(mPhysics.GetConstraints().empty());
        EXPECT_FALSE(grip.attached());
        EXPECT_FALSE(grip.observe());
    }

    TEST_F(SupportGripTest, acquisitionUsesRelativeAnchorSpeedIncludingAngularMotion)
    {
        using Result = RagdollCore::SupportGripAcquireResult;
        RagdollCore::SupportGrip grip(mPhysics);
        const auto request = pair();
        auto& bodies = mPhysics.GetBodyInterface();
        bodies.SetAngularVelocity(request.mFirstBody, JPH::Vec3(0, 0, 5));
        EXPECT_EQ(grip.acquire(request, { 0.01f, 0.1f }), Result::TooFast);
        EXPECT_TRUE(mPhysics.GetConstraints().empty());
        // Body COM speeds differ, but their anchor velocities now agree.
        bodies.SetLinearVelocity(request.mSecondBody, JPH::Vec3(0, 5, 0));
        ASSERT_EQ(grip.acquire(request, { 0.01f, 0.1f }), Result::Acquired);
        const auto state = grip.observe();
        ASSERT_TRUE(state);
        EXPECT_NEAR(state->mRelativeAnchorSpeed, 0, 1e-6f);
        EXPECT_EQ(bodies.GetAngularVelocity(request.mFirstBody), JPH::Vec3(0, 0, 5));
        EXPECT_EQ(bodies.GetLinearVelocity(request.mSecondBody), JPH::Vec3(0, 5, 0));
    }

    TEST_F(SupportGripTest, permittedNearAcquisitionDoesNotTeleportOrCancelMotion)
    {
        using Result = RagdollCore::SupportGripAcquireResult;
        RagdollCore::SupportGrip grip(mPhysics);
        const auto request = pair();
        auto& bodies = mPhysics.GetBodyInterface();
        const JPH::RVec3 secondPosition(1.005, 0, 0);
        const JPH::Vec3 secondVelocity(0.05f, 0, 0);
        bodies.SetPosition(request.mSecondBody, secondPosition, JPH::EActivation::Activate);
        bodies.SetLinearVelocity(request.mSecondBody, secondVelocity);
        ASSERT_EQ(grip.acquire(request, { 0.01f, 0.1f }), Result::Acquired);
        EXPECT_EQ(bodies.GetPosition(request.mSecondBody), secondPosition);
        EXPECT_EQ(bodies.GetLinearVelocity(request.mSecondBody), secondVelocity);
        const auto state = grip.observe();
        ASSERT_TRUE(state);
        EXPECT_NEAR(state->mAnchorSeparation, 0.005f, 1e-6f);
        EXPECT_NEAR(state->mRelativeAnchorSpeed, 0.05f, 1e-6f);
        EXPECT_TRUE(state->mImpulseOnSecond.IsNearZero());
    }

    TEST_F(SupportGripTest, attachedGripMovesItsWeaponAnchorAndRejectsInvalidUpdates)
    {
        using Result = RagdollCore::SupportGripAcquireResult;
        RagdollCore::SupportGrip grip(mPhysics);
        const auto request = pair();
        EXPECT_FALSE(grip.setSecondLocalAnchor(JPH::Vec3(-0.75f, 0, 0)));
        ASSERT_EQ(grip.acquire(request, { 0.01f, 0.1f }), Result::Acquired);
        auto invalid = JPH::Vec3(-0.5f, 0, 0);
        invalid.SetY(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(grip.setSecondLocalAnchor(invalid));
        ASSERT_TRUE(grip.setSecondLocalAnchor(JPH::Vec3(-0.75f, 0, 0)));
        const auto state = grip.observe();
        ASSERT_TRUE(state);
        EXPECT_NEAR(state->mSecondWorldAnchor.GetX(), 0.25f, 1e-6f);
        ASSERT_EQ(mPhysics.GetConstraints().size(), 1u);
        const auto* point = static_cast<const JPH::PointConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
        EXPECT_EQ(point->GetLocalSpacePoint2(), JPH::Vec3(-0.75f, 0, 0));
    }

    TEST_F(SupportGripTest, fixedAnchorsPreserveLocalFrameSolverBudgetAndPriority)
    {
        using Result = RagdollCore::SupportGripAcquireResult;
        JPH::RefConst<JPH::Shape> sphere = new JPH::SphereShape(0.2f);
        JPH::RefConst<JPH::Shape> shifted = new JPH::OffsetCenterOfMassShape(sphere, JPH::Vec3(0.3f, 0, 0));
        const auto first = add(JPH::RVec3(-2, 0, 0), 2,
            JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), std::numbers::pi_v<float> / 2), shifted);
        const auto second = add(JPH::RVec3(2, 0, 0));
        RagdollCore::SupportGripRequest request{ first, second,
            JPH::Vec3(-0.3f, -2, 0), JPH::Vec3(-2, 0, 0) };
        request.mVelocitySolverSteps = 17;
        request.mPositionSolverSteps = 19;
        request.mConstraintPriority = 3;
        RagdollCore::SupportGrip grip(mPhysics);
        ASSERT_EQ(grip.acquire(request, { 1e-5f, 0.1f }), Result::Acquired);
        const auto observed = grip.observe();
        ASSERT_TRUE(observed);
        EXPECT_LT(JPH::Vec3(observed->mFirstWorldAnchor).Length(), 1e-5f);
        EXPECT_LT(JPH::Vec3(observed->mSecondWorldAnchor).Length(), 1e-5f);
        auto moved = request;
        moved.mFirstLocalAnchor = JPH::Vec3(10, 10, 10);
        EXPECT_EQ(grip.acquire(moved, { 100, 100 }), Result::AlreadyAttached);
        ASSERT_EQ(mPhysics.GetConstraints().size(), 1u);
        const auto* point = static_cast<const JPH::PointConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
        EXPECT_EQ(point->GetLocalSpacePoint1(), request.mFirstLocalAnchor);
        EXPECT_EQ(point->GetLocalSpacePoint2(), request.mSecondLocalAnchor);
        EXPECT_EQ(point->GetNumVelocityStepsOverride(), 17u);
        EXPECT_EQ(point->GetNumPositionStepsOverride(), 19u);
        EXPECT_EQ(point->GetConstraintPriority(), 3u);
    }

    TEST_F(SupportGripTest, passiveLoadSharingConservesFreePairMomentumAndReportsEqualOppositeImpulse)
    {
        using Result = RagdollCore::SupportGripAcquireResult;
        const auto request = pair();
        RagdollCore::SupportGrip grip(mPhysics);
        const auto settings = mPhysics.GetPhysicsSettings();
        ASSERT_EQ(grip.acquire(request, { 0.01f, 0.1f }), Result::Acquired);
        mPhysics.GetBodyInterface().AddImpulse(request.mFirstBody, JPH::Vec3(0, 6, 0));
        const auto firstBefore = motion(request.mFirstBody);
        const auto secondBefore = motion(request.mSecondBody);
        step();
        const auto state = grip.observe();
        ASSERT_TRUE(state);
        ASSERT_TRUE(state->mConstraintActive);
        const auto firstAfter = motion(request.mFirstBody);
        const auto secondAfter = motion(request.mSecondBody);
        EXPECT_GT(state->mImpulseOnSecond.Length(), 0.001f);
        EXPECT_LT((secondAfter.mLinearMomentum - secondBefore.mLinearMomentum - state->mImpulseOnSecond).Length(), 1e-4f);
        EXPECT_LT((firstAfter.mLinearMomentum - firstBefore.mLinearMomentum + state->mImpulseOnSecond).Length(), 1e-4f);
        EXPECT_LT((firstAfter.mLinearMomentum + secondAfter.mLinearMomentum
            - firstBefore.mLinearMomentum - secondBefore.mLinearMomentum).Length(), 1e-4f);
        EXPECT_LT((firstAfter.mAngularMomentum + secondAfter.mAngularMomentum
            - firstBefore.mAngularMomentum - secondBefore.mAngularMomentum).Length(), 0.01f);
        EXPECT_LE(firstAfter.mEnergy + secondAfter.mEnergy, firstBefore.mEnergy + secondBefore.mEnergy + 1e-4f);
        EXPECT_LT(state->mAnchorSeparation, 0.01f);
        EXPECT_EQ(mPhysics.GetPhysicsSettings().mNumVelocitySteps, settings.mNumVelocitySteps);
        EXPECT_EQ(mPhysics.GetPhysicsSettings().mNumPositionSteps, settings.mNumPositionSteps);
    }

    TEST_F(SupportGripTest, permitsRelativeTwistWithoutMotorTorque)
    {
        using Result = RagdollCore::SupportGripAcquireResult;
        const auto request = pair();
        RagdollCore::SupportGrip grip(mPhysics);
        ASSERT_EQ(grip.acquire(request, { 0.01f, 0.1f }), Result::Acquired);
        auto& bodies = mPhysics.GetBodyInterface();
        bodies.SetAngularVelocity(request.mFirstBody, JPH::Vec3(1, 0, 0));
        for (int i = 0; i < 30; ++i)
            step();
        EXPECT_NEAR(bodies.GetRotation(request.mFirstBody).GetRotationAngle(JPH::Vec3::sAxisX()), 0.5f, 1e-3f);
        EXPECT_NEAR(bodies.GetRotation(request.mSecondBody).GetRotationAngle(JPH::Vec3::sAxisX()), 0, 1e-6f);
        EXPECT_LT((bodies.GetAngularVelocity(request.mFirstBody) - JPH::Vec3::sAxisX()).Length(), 1e-6f);
        const auto state = grip.observe();
        ASSERT_TRUE(state);
        EXPECT_LT(state->mAnchorSeparation, 1e-6f);
        EXPECT_LT(state->mImpulseOnSecond.Length(), 1e-6f);
    }

    TEST_F(SupportGripTest, overloadReportsFinalImpulseClearsOnceAndNeverReattaches)
    {
        using Result = RagdollCore::SupportGripAcquireResult;
        const auto request = pair();
        RagdollCore::SupportGrip grip(mPhysics);
        ASSERT_EQ(grip.acquire(request, { 0.01f, 0.1f }), Result::Acquired);
        EXPECT_FALSE(grip.releaseIfOverloaded(-1));
        EXPECT_FALSE(grip.releaseIfOverloaded(std::numeric_limits<float>::infinity()));
        EXPECT_TRUE(grip.attached());
        mPhysics.GetBodyInterface().AddImpulse(request.mFirstBody, JPH::Vec3(0, 6, 0));
        step();
        const auto held = grip.releaseIfOverloaded(100);
        ASSERT_TRUE(held);
        EXPECT_FALSE(held->mReleased);
        const auto broken = grip.releaseIfOverloaded(0);
        ASSERT_TRUE(broken);
        EXPECT_TRUE(broken->mReleased);
        EXPECT_GT(broken->mObservation.mImpulseOnSecond.Length(), 0.001f);
        EXPECT_FALSE(grip.attached());
        EXPECT_FALSE(grip.clear());
        EXPECT_FALSE(grip.observe());
        EXPECT_FALSE(grip.releaseIfOverloaded(0));
        for (int i = 0; i < 5; ++i)
            step();
        EXPECT_TRUE(mPhysics.GetConstraints().empty());
        EXPECT_FALSE(grip.attached());
        EXPECT_GT((mPhysics.GetBodyInterface().GetLinearVelocity(request.mSecondBody)
            - mPhysics.GetBodyInterface().GetLinearVelocity(request.mFirstBody)).Length(), 0.01f);
    }

    TEST_F(SupportGripTest, explicitClearAndDestructorRemoveConstraintBeforeBodies)
    {
        using Result = RagdollCore::SupportGripAcquireResult;
        const auto request = pair();
        {
            RagdollCore::SupportGrip grip(mPhysics);
            ASSERT_EQ(grip.acquire(request, { 0.01f, 0.1f }), Result::Acquired);
            ASSERT_TRUE(grip.clear());
            EXPECT_FALSE(grip.clear());
            EXPECT_TRUE(mPhysics.GetConstraints().empty());
            ASSERT_EQ(grip.acquire(request, { 0.01f, 0.1f }), Result::Acquired);
            ASSERT_EQ(mPhysics.GetConstraints().size(), 1u);
        }
        EXPECT_TRUE(mPhysics.GetConstraints().empty());
        EXPECT_TRUE(mPhysics.GetBodyInterface().IsAdded(request.mFirstBody));
        EXPECT_TRUE(mPhysics.GetBodyInterface().IsAdded(request.mSecondBody));
    }

    TEST_F(SupportGripTest, preservesActualGroundAndOtherBodyContacts)
    {
        using Result = RagdollCore::SupportGripAcquireResult;
        const auto request = pair();
        JPH::RefConst<JPH::Shape> floorShape = new JPH::BoxShape(JPH::Vec3(3, 3, 0.1f));
        const auto floor = add(JPH::RVec3(0, 0, -0.25f), 1, JPH::Quat::sIdentity(), floorShape,
            JPH::EMotionType::Static);
        const auto other = add(JPH::RVec3(-1, 0.3f, 0));
        const auto group = mPhysics.GetBodyInterface().GetCollisionGroup(request.mFirstBody);
        RagdollCore::SupportGrip grip(mPhysics);
        ASSERT_EQ(grip.acquire(request, { 0.01f, 0.1f }), Result::Acquired);
        step();
        EXPECT_TRUE(mPhysics.WereBodiesInContact(request.mFirstBody, floor));
        EXPECT_TRUE(mPhysics.WereBodiesInContact(request.mFirstBody, other));
        const auto after = mPhysics.GetBodyInterface().GetCollisionGroup(request.mFirstBody);
        EXPECT_EQ(after.GetGroupFilter(), group.GetGroupFilter());
        EXPECT_EQ(after.GetGroupID(), group.GetGroupID());
        EXPECT_EQ(after.GetSubGroupID(), group.GetSubGroupID());
    }
}
