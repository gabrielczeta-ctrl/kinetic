#include <ragdollcore/balancecontacts.hpp>

// clang-format off
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
// clang-format on

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>

namespace
{
    using namespace RagdollCore;

    class BalanceBroadPhaseLayers final : public JPH::BroadPhaseLayerInterface
    {
    public:
        JPH::uint GetNumBroadPhaseLayers() const override { return 1; }
        JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer) const override { return JPH::BroadPhaseLayer(0); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
        const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override { return "balance-contact-test"; }
#endif
    };

    class BalanceObjectVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter
    {
    public:
        bool ShouldCollide(JPH::ObjectLayer, JPH::BroadPhaseLayer) const override { return true; }
    };

    class BalanceObjectPairs final : public JPH::ObjectLayerPairFilter
    {
    public:
        bool ShouldCollide(JPH::ObjectLayer, JPH::ObjectLayer) const override { return true; }
    };

    class RagdollCoreBalanceContactTest : public testing::Test
    {
    protected:
        void SetUp() override
        {
            mPhysics.Init(32, 0, 64, 64, mBroadPhase, mObjectVsBroadPhase, mPairs);
        }

        void TearDown() override
        {
            mPhysics.SetContactListener(nullptr);
            auto& bodies = mPhysics.GetBodyInterface();
            for (const auto id : mBodies)
            {
                if (bodies.IsAdded(id))
                    bodies.RemoveBody(id);
                bodies.DestroyBody(id);
            }
        }

        JPH::BodyID body(JPH::EMotionType motion, JPH::RVec3Arg position,
            JPH::Vec3Arg centerOffset = JPH::Vec3::sZero(), JPH::QuatArg rotation = JPH::Quat::sIdentity())
        {
            const JPH::RefConst<JPH::Shape> shape
                = new JPH::OffsetCenterOfMassShape(new JPH::BoxShape(JPH::Vec3::sReplicate(1.0f)), centerOffset);
            JPH::BodyCreationSettings settings(shape, position, rotation, motion, 1);
            settings.mAllowSleeping = false;
            const auto id = mPhysics.GetBodyInterface().CreateAndAddBody(settings,
                motion == JPH::EMotionType::Static ? JPH::EActivation::DontActivate : JPH::EActivation::Activate);
            EXPECT_FALSE(id.IsInvalid());
            if (!id.IsInvalid())
                mBodies.push_back(id);
            return id;
        }

        void destroy(JPH::BodyID id)
        {
            auto& bodies = mPhysics.GetBodyInterface();
            if (bodies.IsAdded(id))
                bodies.RemoveBody(id);
            bodies.DestroyBody(id);
            std::erase(mBodies, id);
        }

        static JPH::ContactManifold manifold(unsigned count = 1)
        {
            JPH::ContactManifold result;
            result.mBaseOffset = JPH::RVec3(10.0, 20.0, 30.0);
            result.mWorldSpaceNormal = -JPH::Vec3::sAxisZ();
            result.mPenetrationDepth = 0.2f;
            for (unsigned i = 0; i < count; ++i)
            {
                result.mRelativeContactPointsOn1.push_back(JPH::Vec3(static_cast<float>(i), 2.0f, 0.0f));
                result.mRelativeContactPointsOn2.push_back(JPH::Vec3(static_cast<float>(i), 2.0f, 0.2f));
            }
            return result;
        }

        bool observe(std::vector<BalanceContactObservation>& out, JPH::BodyID first, JPH::BodyID second,
            const JPH::ContactManifold& contact, bool firstBody = true, std::size_t index = 7)
        {
            const JPH::BodyID ids[]{ first, second };
            JPH::BodyLockMultiRead lock(mPhysics.GetBodyLockInterface(), ids, 2);
            const auto* one = lock.GetBody(0);
            const auto* two = lock.GetBody(1);
            EXPECT_NE(one, nullptr);
            EXPECT_NE(two, nullptr);
            return one && two && replaceBalanceContactObservations(out, index, *one, *two, contact, firstBody, 0.6f);
        }

        BalanceBroadPhaseLayers mBroadPhase;
        BalanceObjectVsBroadPhase mObjectVsBroadPhase;
        BalanceObjectPairs mPairs;
        JPH::PhysicsSystem mPhysics;
        std::vector<JPH::BodyID> mBodies;
    };

    TEST_F(RagdollCoreBalanceContactTest, followsBothMaterialPointsAcrossMotionAndBodyOrder)
    {
        const auto first = body(JPH::EMotionType::Dynamic, JPH::RVec3(2.0, 3.0, 4.0),
            JPH::Vec3(0.3f, -0.4f, 0.2f), JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.4f));
        const auto second = body(JPH::EMotionType::Kinematic, JPH::RVec3(-4.0, 5.0, -6.0),
            JPH::Vec3(-0.2f, 0.1f, 0.5f), JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), -0.7f));
        auto& bodies = mPhysics.GetBodyInterface();
        const auto contact = manifold();
        const std::array ids{ first, second };
        const std::array originalPoints{ contact.GetWorldSpaceContactPointOn1(0), contact.GetWorldSpaceContactPointOn2(0) };
        std::array<JPH::Vec3, 2> local;
        for (std::size_t i = 0; i < ids.size(); ++i)
            local[i] = bodies.GetRotation(ids[i]).Conjugated()
                * JPH::Vec3(originalPoints[i] - bodies.GetCenterOfMassPosition(ids[i]));
        std::array<std::vector<BalanceContactObservation>, 2> observations;
        ASSERT_TRUE(observe(observations[0], first, second, contact, true, 7));
        ASSERT_TRUE(observe(observations[1], first, second, contact, false, 8));
        const std::array rotations{ JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.8f),
            JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.9f) };
        const std::array linear{ JPH::Vec3(3.0f, 4.0f, 5.0f), JPH::Vec3(-5.0f, 2.0f, 1.0f) };
        const std::array angular{ JPH::Vec3(-1.0f, 2.0f, 3.0f), JPH::Vec3(3.0f, -4.0f, 2.0f) };
        std::array<JPH::RVec3, 2> movedPoints;
        std::array<JPH::Vec3, 2> expectedVelocity;
        for (std::size_t i = 0; i < ids.size(); ++i)
        {
            bodies.SetPositionAndRotation(ids[i], JPH::RVec3(100.0 * (i + 1), -50.0, 20.0),
                rotations[i], JPH::EActivation::Activate);
            bodies.SetLinearAndAngularVelocity(ids[i], linear[i], angular[i]);
            const auto offset = rotations[i] * local[i];
            movedPoints[i] = bodies.GetCenterOfMassPosition(ids[i]) + JPH::RVec3(offset);
            expectedVelocity[i] = linear[i] + angular[i].Cross(offset);
        }
        for (std::size_t side = 0; side < observations.size(); ++side)
        {
            SCOPED_TRACE(side);
            const auto resolved = resolveBalanceContacts(observations[side], mPhysics);
            ASSERT_EQ(resolved.mContacts.size(), 1u);
            EXPECT_TRUE(resolved.mIncompleteBodyIndices.empty());
            const auto& point = resolved.mContacts[0];
            ASSERT_TRUE(point.mContact.mPointVelocity && point.mSurfacePointVelocity);
            EXPECT_TRUE(point.mContact.mPointVelocity->IsClose(expectedVelocity[side], 1e-8f));
            EXPECT_TRUE(point.mSurfacePointVelocity->IsClose(expectedVelocity[1 - side], 1e-8f));
            EXPECT_TRUE(point.mContact.mPosition.IsClose(movedPoints[0] + (movedPoints[1] - movedPoints[0]) * 0.5f, 1e-8));
            EXPECT_TRUE(point.mContact.mNormal.IsClose(side == 0 ? JPH::Vec3::sAxisZ() : -JPH::Vec3::sAxisZ()));
            EXPECT_EQ(point.mContact.mBodyIndex, 7u + side);
            EXPECT_FALSE(point.mContact.mNormalImpulse);
        }
    }

    TEST_F(RagdollCoreBalanceContactTest, reportsKnownZeroOnlyForObservedStationarySurface)
    {
        const auto foot = body(JPH::EMotionType::Dynamic, JPH::RVec3::sZero());
        const auto surface = body(JPH::EMotionType::Static, JPH::RVec3(0.0, 0.0, -2.0));
        std::vector<BalanceContactObservation> observations;
        ASSERT_TRUE(observe(observations, surface, foot, manifold(), false));
        mPhysics.GetBodyInterface().SetLinearVelocity(foot, JPH::Vec3(3.0f, 4.0f, 0.0f));
        const auto resolved = resolveBalanceContacts(observations, mPhysics);
        ASSERT_EQ(resolved.mContacts.size(), 1u);
        const auto& point = resolved.mContacts[0];
        ASSERT_TRUE(point.mSurfacePointVelocity && point.mContact.mPointVelocity);
        EXPECT_TRUE(point.mSurfacePointVelocity->IsNearZero());
        EXPECT_TRUE(point.mContact.mPointVelocity->IsClose(JPH::Vec3(3.0f, 4.0f, 0.0f)));
        EXPECT_TRUE(resolved.mIncompleteBodyIndices.empty());
    }

    TEST_F(RagdollCoreBalanceContactTest, removedOrReplacedSurfaceCannotMasqueradeAsStatic)
    {
        const auto foot = body(JPH::EMotionType::Dynamic, JPH::RVec3::sZero());
        const auto surface = body(JPH::EMotionType::Static, JPH::RVec3(0.0, 0.0, -2.0));
        std::vector<BalanceContactObservation> observations;
        ASSERT_TRUE(observe(observations, foot, surface, manifold(3)));
        mPhysics.GetBodyInterface().RemoveBody(surface);
        for (const bool replaced : { false, true })
        {
            if (replaced)
            {
                destroy(surface);
                const auto replacement = body(JPH::EMotionType::Static, JPH::RVec3(0.0, 0.0, -2.0));
                EXPECT_NE(replacement.GetIndexAndSequenceNumber(), surface.GetIndexAndSequenceNumber());
            }
            const auto resolved = resolveBalanceContacts(observations, mPhysics);
            EXPECT_EQ(resolved.mIncompleteBodyIndices, std::vector<std::size_t>{ 7 });
            ASSERT_EQ(resolved.mContacts.size(), 3u);
            for (const auto& point : resolved.mContacts)
            {
                EXPECT_TRUE(point.mContact.mPointVelocity);
                EXPECT_FALSE(point.mSurfacePointVelocity);
            }
        }
    }

    TEST_F(RagdollCoreBalanceContactTest, missingFootRetainsOnlyKnownSurfaceVelocity)
    {
        const auto foot = body(JPH::EMotionType::Dynamic, JPH::RVec3::sZero());
        const auto surface = body(JPH::EMotionType::Kinematic, JPH::RVec3(0.0, 0.0, -2.0));
        std::vector<BalanceContactObservation> observations;
        ASSERT_TRUE(observe(observations, surface, foot, manifold(2), false, 11));
        mPhysics.GetBodyInterface().SetLinearVelocity(surface, JPH::Vec3(2.0f, 0.0f, 0.0f));
        destroy(foot);
        const auto resolved = resolveBalanceContacts(observations, mPhysics);
        EXPECT_EQ(resolved.mIncompleteBodyIndices, std::vector<std::size_t>{ 11 });
        for (const auto& point : resolved.mContacts)
        {
            EXPECT_FALSE(point.mContact.mPointVelocity);
            ASSERT_TRUE(point.mSurfacePointVelocity);
            EXPECT_TRUE(point.mSurfacePointVelocity->IsClose(JPH::Vec3(2.0f, 0.0f, 0.0f)));
        }
    }

    TEST_F(RagdollCoreBalanceContactTest, unavailableImpulseNeverRetainsStaleLoad)
    {
        const auto first = body(JPH::EMotionType::Dynamic, JPH::RVec3::sZero());
        const auto second = body(JPH::EMotionType::Static, JPH::RVec3(0.0, 0.0, -2.0));
        std::vector<BalanceContactObservation> observations;
        ASSERT_TRUE(observe(observations, first, second, manifold()));
        observations[0].mGround.mContact.mNormalImpulse = 100.0f;
        observations[0].mGround.mContact.mFrictionImpulseMagnitude = 50.0f;
        const auto resolved = resolveBalanceContacts(observations, mPhysics);
        ASSERT_EQ(resolved.mContacts.size(), 1u);
        EXPECT_FALSE(resolved.mContacts[0].mContact.mNormalImpulse);
        EXPECT_FALSE(resolved.mContacts[0].mContact.mFrictionImpulseMagnitude);
        const auto support = measureBalanceSupport(JPH::RVec3(0.0, 0.0, 5.0),
            resolved.mContacts, { 7, 8 }, ActiveControllerProfile{}, true);
        EXPECT_EQ(support.mAvailability, BalanceSupportAvailability::Unknown);
    }

    TEST_F(RagdollCoreBalanceContactTest, repeatedManifoldShrinksWithoutLosingOtherPairsOrSides)
    {
        const auto first = body(JPH::EMotionType::Dynamic, JPH::RVec3::sZero());
        const auto second = body(JPH::EMotionType::Dynamic, JPH::RVec3(0.0, 0.0, -2.0));
        const auto third = body(JPH::EMotionType::Static, JPH::RVec3(0.0, 3.0, -2.0));
        std::vector<BalanceContactObservation> observations;
        ASSERT_TRUE(observe(observations, first, second, manifold(3), true, 7));
        ASSERT_TRUE(observe(observations, first, second, manifold(2), false, 8));
        ASSERT_TRUE(observe(observations, first, third, manifold(2), true, 7));
        auto replacement = manifold();
        replacement.mBaseOffset += JPH::RVec3(10.0, 0.0, 0.0);
        ASSERT_TRUE(observe(observations, first, second, replacement, true, 7));
        ASSERT_EQ(observations.size(), 5u);
        ASSERT_TRUE(observe(observations, first, second, replacement, true, 7));
        EXPECT_EQ(observations.size(), 5u);
        EXPECT_EQ(std::count_if(observations.begin(), observations.end(), [](const auto& point) {
            return !point.mGround.mFirstBody;
        }), 2);
        const auto& latest = observations.back().mGround;
        EXPECT_EQ(latest.mPointIndex, 0u);
        EXPECT_EQ(latest.mPair.GetBody2ID(), second);
        EXPECT_TRUE(latest.mContact.mPosition.IsClose(JPH::RVec3(20.0, 22.0, 30.1), 1e-8));
    }

    TEST_F(RagdollCoreBalanceContactTest, distinctSubshapePatchSurvivesReplacementAndInvalidation)
    {
        const auto first = body(JPH::EMotionType::Dynamic, JPH::RVec3::sZero());
        const auto second = body(JPH::EMotionType::Static, JPH::RVec3(0.0, 0.0, -2.0));
        std::vector<BalanceContactObservation> observations;
        ASSERT_TRUE(observe(observations, first, second, manifold(2)));
        auto otherPatch = manifold(2);
        otherPatch.mSubShapeID2.SetValue(5);
        ASSERT_TRUE(observe(observations, first, second, otherPatch));
        ASSERT_TRUE(observe(observations, first, second, manifold()));
        ASSERT_EQ(observations.size(), 3u);
        EXPECT_FALSE(observe(observations, first, second, manifold(0)));
        ASSERT_EQ(observations.size(), 2u);
        for (const auto& point : observations)
            EXPECT_EQ(point.mGround.mPair.GetSubShapeID2().GetValue(), 5u);
    }

    TEST_F(RagdollCoreBalanceContactTest, invalidManifoldRemovesPriorPatchWithoutInventedVertex)
    {
        const auto first = body(JPH::EMotionType::Dynamic, JPH::RVec3::sZero());
        const auto second = body(JPH::EMotionType::Static, JPH::RVec3(0.0, 0.0, -2.0));
        const float nan = std::numeric_limits<float>::quiet_NaN();
        for (unsigned invalid = 0; invalid < 5; ++invalid)
        {
            SCOPED_TRACE(invalid);
            std::vector<BalanceContactObservation> observations;
            ASSERT_TRUE(observe(observations, first, second, manifold(3)));
            auto bad = manifold(2);
            switch (invalid)
            {
                case 0: bad = manifold(0); break;
                case 1: bad.mRelativeContactPointsOn2.pop_back(); break;
                case 2: bad.mWorldSpaceNormal = JPH::Vec3::sZero(); break;
                case 3: bad.mRelativeContactPointsOn1[1].SetX(nan); break;
                case 4: bad.mPenetrationDepth = nan; break;
            }
            EXPECT_FALSE(observe(observations, first, second, bad));
            EXPECT_TRUE(observations.empty());
        }
    }

    TEST_F(RagdollCoreBalanceContactTest, invalidMaterialPointsInvalidateGeometryAndClearVelocity)
    {
        const auto first = body(JPH::EMotionType::Dynamic, JPH::RVec3::sZero());
        const auto second = body(JPH::EMotionType::Static, JPH::RVec3(0.0, 0.0, -2.0));
        std::vector<BalanceContactObservation> observations;
        ASSERT_TRUE(observe(observations, first, second, manifold()));
        observations[0].mGround.mLocalPoint.reset();
        observations[0].mGround.mContact.mPointVelocity = JPH::Vec3(100.0f, 0.0f, 0.0f);
        observations[0].mSurfaceLocalPoint.SetY(std::numeric_limits<float>::quiet_NaN());
        const auto resolved = resolveBalanceContacts(observations, mPhysics);
        EXPECT_EQ(resolved.mIncompleteBodyIndices, std::vector<std::size_t>{ 7 });
        ASSERT_EQ(resolved.mContacts.size(), 1u);
        EXPECT_FALSE(resolved.mContacts[0].mContact.mPointVelocity);
        EXPECT_FALSE(resolved.mContacts[0].mSurfacePointVelocity);
    }

    TEST_F(RagdollCoreBalanceContactTest, readsActualSolvedContactImpulsesAndLoadedSupport)
    {
        const auto foot = body(JPH::EMotionType::Dynamic, JPH::RVec3(0.0, 0.0, 0.9));
        body(JPH::EMotionType::Static, JPH::RVec3(0.0, 0.0, -1.0));
        class Listener final : public JPH::ContactListener
        {
        public:
            JPH::BodyID mFoot;
            bool mComplete = true;
            std::vector<BalanceContactObservation> mObservations;
            void OnContactAdded(const JPH::Body& one, const JPH::Body& two,
                const JPH::ContactManifold& contact, JPH::ContactSettings& settings) override
            {
                if (one.GetID() == mFoot || two.GetID() == mFoot)
                    mComplete &= replaceBalanceContactObservations(mObservations, 7, one, two,
                        contact, one.GetID() == mFoot, settings.mCombinedFriction);
            }
            void OnContactPersisted(const JPH::Body& one, const JPH::Body& two,
                const JPH::ContactManifold& contact, JPH::ContactSettings& settings) override
            {
                OnContactAdded(one, two, contact, settings);
            }
        } listener;
        listener.mFoot = foot;
        mPhysics.SetContactListener(&listener);
        struct ResetListener
        {
            JPH::PhysicsSystem& mPhysics;
            ~ResetListener() { mPhysics.SetContactListener(nullptr); }
        } reset{ mPhysics };
        mPhysics.SetGravity(JPH::Vec3(0.0f, 0.0f, -9.81f));
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        ASSERT_EQ(mPhysics.Update(1.0f / 60.0f, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        ASSERT_TRUE(listener.mComplete);
        ASSERT_FALSE(listener.mObservations.empty());
        const auto resolved = resolveBalanceContacts(listener.mObservations, mPhysics);
        ASSERT_TRUE(resolved.mIncompleteBodyIndices.empty());
        float normalImpulse = 0.0f;
        for (const auto& point : resolved.mContacts)
        {
            ASSERT_TRUE(point.mContact.mNormalImpulse);
            EXPECT_GE(*point.mContact.mNormalImpulse, 0.0f);
            normalImpulse += *point.mContact.mNormalImpulse;
            ASSERT_TRUE(point.mSurfacePointVelocity);
            EXPECT_TRUE(point.mSurfacePointVelocity->IsNearZero());
        }
        EXPECT_GT(normalImpulse, 0.0f);
        const auto support = measureBalanceSupport(mPhysics.GetBodyInterface().GetCenterOfMassPosition(foot),
            resolved.mContacts, { 7, 8 }, ActiveControllerProfile{}, true);
        EXPECT_EQ(support.mAvailability, BalanceSupportAvailability::Supported);
        EXPECT_TRUE(support.mLeftFootSupported);
        EXPECT_FALSE(support.mRightFootSupported);
        const auto slip = measureBalanceFootSlip(resolved.mContacts, 7, 0.5f, true, 1.0f / 60.0f);
        EXPECT_EQ(slip.mAvailability, BalanceSupportAvailability::Supported);
        ASSERT_TRUE(slip.mMaximumTangentSpeed);
        EXPECT_NEAR(*slip.mMaximumTangentSpeed, 0.0f, 1e-4f);
    }
}
