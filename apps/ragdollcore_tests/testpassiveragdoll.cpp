#include "fixtures.hpp"

#include <ragdollcore/controller.hpp>
#include <ragdollcore/contact.hpp>
#include <ragdollcore/gaitcontroller.hpp>
#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/scenario.hpp>
#include <ragdollcore/weaponalignment.hpp>

// clang-format off
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
// clang-format on

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
    constexpr JPH::ObjectLayer sObjectLayer = 1;

    class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterface
    {
    public:
        JPH::uint GetNumBroadPhaseLayers() const override { return 1; }
        JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer) const override { return JPH::BroadPhaseLayer(0); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
        const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override { return "test"; }
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

    class PassiveRagdollTest : public testing::Test
    {
    protected:
        void SetUp() override
        {
            mPhysics.Init(128, 0, 128, 128, mBroadPhaseLayers, mObjectVsBroadPhase, mObjectPairs);
        }

        RagdollCore::PassiveRagdollCreateResult create(
            const RagdollCore::PassiveRagdollInitialState& initialState = {})
        {
            RagdollCore::BuildOptions buildOptions;
            buildOptions.mObjectLayer = sObjectLayer;
            return RagdollCore::createPassiveRagdoll(mPhysics, RagdollCoreTest::makeSkeleton(),
                RagdollCoreTest::makeProfile(), buildOptions, initialState, 7, 1234);
        }

        BroadPhaseLayers mBroadPhaseLayers;
        ObjectVsBroadPhase mObjectVsBroadPhase;
        ObjectPairs mObjectPairs;
        JPH::PhysicsSystem mPhysics;
    };

    TEST_F(PassiveRagdollTest, reactionTorqueBudgetRestoresExactLimits)
    {
        for (bool hinge : { false, true })
        {
            auto profile = RagdollCoreTest::makeProfile();
            if (hinge)
                profile.mBodies[1].mConstraint->mType = RagdollCore::ConstraintType::Hinge;
            RagdollCore::BuildOptions options;
            options.mObjectLayer = sObjectLayer;
            auto result = RagdollCore::createPassiveRagdoll(mPhysics, RagdollCoreTest::makeSkeleton(),
                profile, options, {}, 7, 1234);
            ASSERT_TRUE(result.isValid());
            auto& ragdoll = *result.mRagdoll;
            ASSERT_TRUE(ragdoll.addToPhysicsSystem());
            const JPH::Vec3 minimum = hinge ? JPH::Vec3(-17, 0, 0) : JPH::Vec3(-17, -23, -23);
            const JPH::Vec3 maximum = hinge ? JPH::Vec3(19, 0, 0) : JPH::Vec3(19, 31, 31);
            ASSERT_TRUE(ragdoll.setMotorTorqueLimits(1, minimum, maximum));
            RagdollCore::MotorTargetTelemetry original;
            ragdoll.captureMotorTargetTelemetry(1, original);
            ASSERT_TRUE(original.mDrive);
            for (int frame = 0; frame < 120; ++frame)
            {
                ASSERT_TRUE(ragdoll.setMotorTorqueLimits(1, minimum * 0.4f, maximum * 0.4f));
                RagdollCore::MotorTargetTelemetry reduced;
                ragdoll.captureMotorTargetTelemetry(1, reduced);
                ASSERT_TRUE(reduced.mDrive);
                EXPECT_EQ(reduced.mDrive->mMaximumTorque, maximum * 0.4f);
                EXPECT_EQ(reduced.mDrive->mMinimumTorque, minimum * 0.4f);
                ASSERT_TRUE(ragdoll.setMotorTorqueLimits(1, minimum, maximum));
            }
            EXPECT_FALSE(ragdoll.setMotorTorqueLimits(1, JPH::Vec3::sReplicate(1), maximum));
            EXPECT_FALSE(ragdoll.setMotorTorqueLimits(1, minimum, JPH::Vec3::sReplicate(-1)));
            EXPECT_FALSE(ragdoll.setMotorTorqueLimits(1, minimum, JPH::Vec3(std::nanf(""), 0, 0)));
            RagdollCore::MotorTargetTelemetry restored;
            ragdoll.captureMotorTargetTelemetry(1, restored);
            ASSERT_TRUE(restored.mDrive);
            EXPECT_EQ(restored.mDrive->mMaximumTorque, maximum);
            EXPECT_EQ(restored.mDrive->mMinimumTorque, minimum);
            EXPECT_EQ(restored.mDrive->mSwingOrHingeState, original.mDrive->mSwingOrHingeState);
            EXPECT_EQ(restored.mDrive->mSwingOrHingeSpring.mFrequencyOrStiffness,
                original.mDrive->mSwingOrHingeSpring.mFrequencyOrStiffness);
        }
    }

    TEST_F(PassiveRagdollTest, bodyPairCollisionOverridesAreIsolatedAndReversible)
    {
        auto skeleton = RagdollCoreTest::makeSkeleton();
        skeleton.mBones.push_back({ "hand", 1, { JPH::Vec3(0, 20, 0), JPH::Quat::sIdentity() } });
        auto profile = RagdollCoreTest::makeProfile();
        auto hand = profile.mBodies.back();
        hand.mName = hand.mBone = "hand";
        hand.mParentBody = "torso";
        hand.mMassFraction = 0.3f;
        profile.mBodies[0].mMassFraction = 0.4f;
        profile.mBodies[1].mMassFraction = 0.3f;
        profile.mBodies.push_back(hand);
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        auto first = RagdollCore::createPassiveRagdoll(mPhysics, skeleton, profile, options, {}, 7, 1234);
        options.mRootPosition = JPH::RVec3(100, 0, 0);
        auto second = RagdollCore::createPassiveRagdoll(mPhysics, skeleton, profile, options, {}, 8, 5678);
        ASSERT_TRUE(first.isValid() && second.isValid());
        ASSERT_TRUE(first.mRagdoll->addToPhysicsSystem());
        ASSERT_TRUE(second.mRagdoll->addToPhysicsSystem());
        auto& bodies = mPhysics.GetBodyInterface();
        const auto original = bodies.GetCollisionGroup(first.mRagdoll->getBodyId(0));
        const auto* originalTable = JPH::DynamicCast<JPH::GroupFilterTable>(original.GetGroupFilter());
        ASSERT_NE(originalTable, nullptr);
        // Deliberately share the actual source table with another instance,
        // as settings reuse can do. The other actor keeps its own group ID.
        for (std::size_t i = 0; i < second.mRagdoll->getBodyCount(); ++i)
        {
            auto group = bodies.GetCollisionGroup(second.mRagdoll->getBodyId(i));
            group.SetGroupFilter(original.GetGroupFilter());
            bodies.SetCollisionGroup(second.mRagdoll->getBodyId(i), group);
        }
        const auto initial = first.mRagdoll->isBodyPairCollisionEnabled(0, 2);
        ASSERT_TRUE(initial && *initial);
        ASSERT_TRUE(first.mRagdoll->setBodyPairCollisionEnabled(0, 2, true));
        EXPECT_EQ(bodies.GetCollisionGroup(first.mRagdoll->getBodyId(0)).GetGroupFilter(), original.GetGroupFilter());
        EXPECT_FALSE(first.mRagdoll->setBodyPairCollisionEnabled(0, 0, false));
        EXPECT_FALSE(first.mRagdoll->setBodyPairCollisionEnabled(0, 99, false));
        EXPECT_FALSE(first.mRagdoll->isBodyPairCollisionEnabled(0, 0));
        EXPECT_FALSE(first.mRagdoll->isBodyPairCollisionEnabled(0, 99));
        ASSERT_TRUE(first.mRagdoll->setBodyPairCollisionEnabled(0, 2, false));
        EXPECT_EQ(first.mRagdoll->isBodyPairCollisionEnabled(2, 0), std::optional(false));
        EXPECT_EQ(second.mRagdoll->isBodyPairCollisionEnabled(0, 2), std::optional(true));
        EXPECT_TRUE(originalTable->IsCollisionEnabled(0, 2));
        const auto changed = bodies.GetCollisionGroup(first.mRagdoll->getBodyId(0));
        EXPECT_NE(changed.GetGroupFilter(), original.GetGroupFilter());
        for (std::size_t i = 0; i < first.mRagdoll->getBodyCount(); ++i)
        {
            const auto firstGroup = bodies.GetCollisionGroup(first.mRagdoll->getBodyId(i));
            const auto secondGroup = bodies.GetCollisionGroup(second.mRagdoll->getBodyId(i));
            EXPECT_EQ(firstGroup.GetGroupFilter(), changed.GetGroupFilter());
            EXPECT_EQ(firstGroup.GetGroupID(), 7u);
            EXPECT_EQ(firstGroup.GetSubGroupID(), i);
            EXPECT_EQ(secondGroup.GetGroupFilter(), original.GetGroupFilter());
            EXPECT_EQ(secondGroup.GetGroupID(), 8u);
            EXPECT_EQ(secondGroup.GetSubGroupID(), i);
            EXPECT_TRUE(firstGroup.CanCollide(secondGroup));
            EXPECT_TRUE(firstGroup.CanCollide(JPH::CollisionGroup::sInvalid));
        }
        for (const std::size_t i : { 0u, 2u })
        {
            JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), first.mRagdoll->getBodyId(i));
            ASSERT_TRUE(lock.Succeeded());
            EXPECT_TRUE(lock.GetBody().IsCollisionCacheInvalid());
        }
        ASSERT_TRUE(first.mRagdoll->setBodyPairCollisionEnabled(0, 2, *initial));
        EXPECT_EQ(first.mRagdoll->isBodyPairCollisionEnabled(0, 2), initial);
        const auto adjacentInitial = first.mRagdoll->isBodyPairCollisionEnabled(0, 1);
        ASSERT_TRUE(adjacentInitial);
        EXPECT_FALSE(*adjacentInitial);
        ASSERT_TRUE(first.mRagdoll->setBodyPairCollisionEnabled(0, 1, true));
        ASSERT_TRUE(first.mRagdoll->setBodyPairCollisionEnabled(0, 1, *adjacentInitial));
        EXPECT_EQ(first.mRagdoll->isBodyPairCollisionEnabled(0, 1), adjacentInitial);
        EXPECT_EQ(first.mRagdoll->isBodyPairCollisionEnabled(0, 2), initial);
        EXPECT_EQ(second.mRagdoll->isBodyPairCollisionEnabled(0, 1), adjacentInitial);
    }

    TEST_F(PassiveRagdollTest, bodyPairCollisionRejectsIncompatibleGroupsWithoutMutation)
    {
        auto created = create();
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        EXPECT_FALSE(ragdoll.isBodyPairCollisionEnabled(0, 1));
        EXPECT_FALSE(ragdoll.setBodyPairCollisionEnabled(0, 1, true));
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        auto& bodies = mPhysics.GetBodyInterface();
        const auto original = bodies.GetCollisionGroup(ragdoll.getBodyId(1));
        auto changed = original;
        changed.SetSubGroupID(JPH::CollisionGroup::cInvalidSubGroup);
        bodies.SetCollisionGroup(ragdoll.getBodyId(1), changed);
        EXPECT_FALSE(ragdoll.isBodyPairCollisionEnabled(0, 1));
        EXPECT_FALSE(ragdoll.setBodyPairCollisionEnabled(0, 1, true));
        EXPECT_EQ(bodies.GetCollisionGroup(ragdoll.getBodyId(0)).GetGroupFilter(), original.GetGroupFilter());
        changed = original;
        changed.SetGroupID(original.GetGroupID() + 1);
        bodies.SetCollisionGroup(ragdoll.getBodyId(1), changed);
        EXPECT_FALSE(ragdoll.isBodyPairCollisionEnabled(0, 1));
        EXPECT_FALSE(ragdoll.setBodyPairCollisionEnabled(0, 1, true));
        EXPECT_EQ(bodies.GetCollisionGroup(ragdoll.getBodyId(0)).GetGroupFilter(), original.GetGroupFilter());
        bodies.SetCollisionGroup(ragdoll.getBodyId(1), original);
        EXPECT_EQ(ragdoll.isBodyPairCollisionEnabled(0, 1), std::optional(false));
    }

    TEST_F(PassiveRagdollTest, bodyPairCollisionChangeInvalidatesSolvedContacts)
    {
        auto created = create();
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        mPhysics.SetGravity(JPH::Vec3::sZero());
        auto& bodies = mPhysics.GetBodyInterface();
        const auto first = ragdoll.getBodyId(0);
        const auto second = ragdoll.getBodyId(1);
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        // Update builds the full dependency graph even for two bodies. A
        // smaller arbitrary pool can exhaust before any jobs are released.
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        const auto overlap = [&] {
            bodies.SetPosition(first, JPH::RVec3::sZero(), JPH::EActivation::Activate);
            bodies.SetPosition(second, JPH::RVec3::sZero(), JPH::EActivation::Activate);
            bodies.SetLinearAndAngularVelocity(first, JPH::Vec3::sZero(), JPH::Vec3::sZero());
            bodies.SetLinearAndAngularVelocity(second, JPH::Vec3::sZero(), JPH::Vec3::sZero());
        };
        overlap();
        ASSERT_TRUE(ragdoll.setBodyPairCollisionEnabled(0, 1, true));
        ASSERT_EQ(mPhysics.Update(1.0f / 60.0f, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        EXPECT_TRUE(mPhysics.WereBodiesInContact(first, second));
        overlap();
        ASSERT_TRUE(ragdoll.setBodyPairCollisionEnabled(0, 1, false));
        ASSERT_EQ(mPhysics.Update(1.0f / 60.0f, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        EXPECT_FALSE(mPhysics.WereBodiesInContact(first, second));
        overlap();
        ASSERT_TRUE(ragdoll.setBodyPairCollisionEnabled(0, 1, true));
        ASSERT_EQ(mPhysics.Update(1.0f / 60.0f, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        EXPECT_TRUE(mPhysics.WereBodiesInContact(first, second));
    }

    TEST_F(PassiveRagdollTest, swingTwistGeometryMatchesNativeClampWithoutChangingTheJoint)
    {
        using namespace RagdollCore;
        auto created = create();
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        auto* joint = static_cast<JPH::SwingTwistConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
        joint->SetNormalHalfConeAngle(0.6f);
        joint->SetPlaneHalfConeAngle(0.2f);
        joint->SetTwistMinAngle(-0.1f);
        joint->SetTwistMaxAngle(0.3f);
        const auto geometry = ragdoll.getSwingTwistJointGeometry(1);
        ASSERT_TRUE(geometry);
        EXPECT_FALSE(ragdoll.getSwingTwistJointGeometry(0));
        EXPECT_FALSE(ragdoll.getSwingTwistJointGeometry(99));
        const auto before = joint->GetTargetOrientationCS();
        for (const float parentAngle : { -0.8f, 0.0f, 0.7f })
            for (const float childAngle : { -1.4f, -0.1f, 0.0f, 0.6f, 1.8f })
                for (const auto& axis : { JPH::Vec3::sAxisX(), JPH::Vec3::sAxisY(), JPH::Vec3::sAxisZ(),
                         JPH::Vec3(1, 2, 3).Normalized() })
                {
                    const auto parent = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), parentAngle);
                    const auto child = JPH::Quat::sRotation(axis, childAngle);
                    const auto frame = parent * joint->GetConstraintToBody1();
                    const auto target = (frame.Conjugated() * child * joint->GetConstraintToBody2()).Normalized();
                    const auto expected = (frame * joint->GetClampedTargetOrientationCS(target)
                        * joint->GetConstraintToBody2().Conjugated()).Normalized();
                    const auto actual = clampSwingTwistJointOrientation(*geometry, parent, child);
                    ASSERT_TRUE(actual);
                    EXPECT_TRUE(actual->IsClose(expected, 1e-10f) || actual->IsClose(-expected, 1e-10f));
                }
        EXPECT_TRUE(joint->GetTargetOrientationCS().IsClose(before, 0));
        auto invalid = *geometry;
        invalid.mPlaneHalfConeAngle = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(clampSwingTwistJointOrientation(invalid, JPH::Quat::sIdentity(), JPH::Quat::sIdentity()));
    }

    TEST_F(PassiveRagdollTest, contactVelocityFollowsTheMaterialPointAfterTranslationAndRotation)
    {
        using namespace RagdollCore;
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[0].mCenterOfMass = JPH::Vec3(1, 2, 3);
        profile.mBodies[1].mCenterOfMass = JPH::Vec3(-2, 1, 0);
        BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        auto created = createPassiveRagdoll(mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());
        auto& bodies = mPhysics.GetBodyInterface();
        const auto firstId = created.mRagdoll->getBodyId(0);
        const auto secondId = created.mRagdoll->getBodyId(1);
        for (const bool firstBody : { true, false })
        {
            const auto id = firstBody ? firstId : secondId;
            const auto point = JPH::RVec3(2, 3, -1);
            JPH::Vec3 localPoint;
            std::vector<GroundContactObservation> observations;
            {
                // The callback normally already holds these body locks.
                JPH::BodyLockRead firstLock(mPhysics.GetBodyLockInterfaceNoLock(), firstId);
                JPH::BodyLockRead secondLock(mPhysics.GetBodyLockInterfaceNoLock(), secondId);
                ASSERT_TRUE(firstLock.Succeeded());
                ASSERT_TRUE(secondLock.Succeeded());
                const auto& body = firstBody ? firstLock.GetBody() : secondLock.GetBody();
                localPoint = JPH::Vec3(body.GetInverseCenterOfMassTransform() * point);
                JPH::ContactManifold manifold;
                manifold.mWorldSpaceNormal = JPH::Vec3::sAxisZ();
                manifold.mBaseOffset = point;
                manifold.mRelativeContactPointsOn1.push_back(JPH::Vec3::sZero());
                manifold.mRelativeContactPointsOn2.push_back(JPH::Vec3::sZero());
                appendGroundContactObservations(observations, firstBody ? 0 : 1,
                    firstLock.GetBody(), secondLock.GetBody(), manifold, firstBody, 0.4f);
            }
            const auto rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.8f);
            const auto linear = JPH::Vec3(3, 4, 5);
            const auto angular = JPH::Vec3(-1, 2, 3);
            bodies.SetPositionAndRotation(id, JPH::RVec3(100, -50, 20), rotation, JPH::EActivation::Activate);
            bodies.SetLinearAndAngularVelocity(id, linear, angular);
            auto contacts = resolveGroundContacts(observations, mPhysics);
            ASSERT_EQ(contacts.size(), 1u);
            EXPECT_TRUE(contacts[0].mPosition.IsClose(point));
            EXPECT_FALSE(contacts[0].mNormalImpulse);
            ASSERT_TRUE(contacts[0].mPointVelocity);
            const auto expected = linear + angular.Cross(rotation * localPoint);
            EXPECT_TRUE(contacts[0].mPointVelocity->IsClose(expected, 1e-8f));
            // A contact with no material point must not inherit stale velocity.
            observations[0].mLocalPoint.reset();
            observations[0].mContact.mPointVelocity = JPH::Vec3::sZero();
            contacts = resolveGroundContacts(observations, mPhysics);
            EXPECT_FALSE(contacts[0].mPointVelocity);
        }
    }

    TEST_F(PassiveRagdollTest, hingeLegGeometryMatchesNativeTargetsAcrossScaleAndRootRebase)
    {
        using namespace RagdollCore;
        for (const float scale : { 0.5f, 2.0f })
        {
            auto profile = RagdollCoreTest::makeProfile();
            profile.mBodies.push_back(profile.mBodies[1]);
            profile.mBodies[2].mName = "foot";
            profile.mBodies[2].mBone = "foot";
            profile.mBodies[2].mParentBody = "torso";
            profile.mBodies[0].mMassFraction = 0.4f;
            profile.mBodies[1].mMassFraction = profile.mBodies[2].mMassFraction = 0.3f;
            for (std::size_t i = 1; i < 3; ++i)
                profile.mBodies[i].mConstraint->mDeriveFrameFromSkeleton = true;
            auto& kneeProfile = *profile.mBodies[1].mConstraint;
            kneeProfile.mType = ConstraintType::Hinge;
            kneeProfile.mHingeMinAngle = -0.05f;
            kneeProfile.mHingeMaxAngle = 2.0f;
            kneeProfile.mMotor.mMaximumTorque = 100.0f;
            auto skeleton = RagdollCoreTest::makeSkeleton();
            skeleton.mBones[0].mModelTransform.mTranslation = JPH::Vec3(0, 0, 25);
            skeleton.mBones[0].mModelTransform.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.4f);
            skeleton.mBones[1].mModelTransform = { JPH::Vec3(0.2f, 0.1f, 15),
                JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.6f) };
            skeleton.mBones.push_back({ "foot", 1, { JPH::Vec3(0.4f, -2, 3), JPH::Quat::sIdentity() } });
            BuildOptions options;
            options.mObjectLayer = sObjectLayer;
            options.mUniformScale = scale;
            options.mRootPosition = JPH::RVec3(100, -200, 4);
            options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.8f);
            auto created = createPassiveRagdoll(mPhysics, skeleton, profile, options, {}, 7, 1234);
            ASSERT_TRUE(created.isValid());
            auto& ragdoll = *created.mRagdoll;
            ASSERT_TRUE(ragdoll.addToPhysicsSystem());
            const auto geometry = ragdoll.getHingeLegGeometry(0, 1, 2);
            ASSERT_TRUE(geometry);
            EXPECT_FALSE(ragdoll.getHingeLegGeometry(0, 2, 1));
            EXPECT_FALSE(ragdoll.getHingeLegGeometry(0, 1, 3));
            const auto& upper = skeleton.mBones[0].mModelTransform;
            EXPECT_TRUE((upper.mRotation * geometry->mUpperOffset).IsClose(
                skeleton.mBones[1].mModelTransform.mTranslation - upper.mTranslation, 1e-8f));
            EXPECT_TRUE((upper.mRotation * geometry->mLowerOffset).IsClose(
                skeleton.mBones[2].mModelTransform.mTranslation - skeleton.mBones[1].mModelTransform.mTranslation, 1e-8f));
            RagdollPose pose;
            ASSERT_TRUE(ragdoll.captureModelPose(pose));
            const float angle = 0.7f;
            const auto flexion = JPH::Quat::sRotation(geometry->mHingeAxis, angle);
            const auto worldRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), -0.3f);
            TwoBoneLegTargetInput input;
            input.mKneeBodyIndex = 1;
            input.mFootBodyIndex = 2;
            input.mHingeGeometry = geometry;
            input.mTargetFootPosition = pose.mBodyTransforms[0].mTranslation
                + worldRotation * (geometry->mUpperOffset + flexion * geometry->mLowerOffset);
            input.mPreferredBendDirection = worldRotation * geometry->mUpperOffset;
            ASSERT_TRUE(applyTwoBoneLegTarget(pose.mBodyTransforms, input));
            ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
            MotorTargetTelemetry target;
            ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, target));
            ASSERT_TRUE(target.mTargetHingeAngle);
            EXPECT_NEAR(*target.mTargetHingeAngle, angle, 1e-5f);
            ASSERT_TRUE(ragdoll.setReferenceTransform(JPH::RVec3(-50, 30, 12), JPH::Quat::sIdentity()));
            const auto rebased = ragdoll.getHingeLegGeometry(0, 1, 2);
            ASSERT_TRUE(rebased);
            EXPECT_TRUE(rebased->mUpperOffset.IsClose(geometry->mUpperOffset, 0));
            EXPECT_TRUE(rebased->mLowerOffset.IsClose(geometry->mLowerOffset, 0));
            EXPECT_TRUE(rebased->mHingeAxis.IsClose(geometry->mHingeAxis, 0));
        }
    }

    TEST_F(PassiveRagdollTest, centroidalMomentumIncludesRotatedBodyInertia)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies.resize(1);
        profile.mBodies[0].mMassFraction = 1.0f;
        profile.mLandmarks.mSpine.clear();
        RagdollCore::BuildOptions options;
        options.mTotalMass = 75.0f;
        options.mObjectLayer = sObjectLayer;
        options.mRootPosition = JPH::RVec3(100, -200, 4);
        options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), JPH::JPH_PI / 2.0f);
        RagdollCore::PassiveRagdollInitialState initial;
        initial.mActorMotion = { JPH::Vec3(4, 5, 6), JPH::Vec3(1, 2, 3) };
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, initial, 7, 1234);
        ASSERT_TRUE(created.isValid());
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());
        RagdollCore::RagdollMomentum momentum;
        ASSERT_TRUE(created.mRagdoll->captureMomentum(momentum));
        EXPECT_FLOAT_EQ(momentum.mTotalMass, 75.0f);
        EXPECT_LT(JPH::Vec3(momentum.mCenterOfMass - options.mRootPosition).Length(), 1e-4f);
        EXPECT_LT((momentum.mLinearMomentum - JPH::Vec3(300, 375, 450)).Length(), 1e-3f);
        // Box half-extents (5,3,4): I = m/3 * (25,41,34),
        // with X/Y inertias exchanged by the quarter turn.
        EXPECT_LT((momentum.mAngularMomentum - JPH::Vec3(1025, 1250, 2550)).Length(), 1e-2f);
    }

    TEST_F(PassiveRagdollTest, centroidalOrbitalMomentumIgnoresTranslationAndCommonVelocity)
    {
        for (const float offset : { 0.0f, 10000.0f })
        {
            auto profile = RagdollCoreTest::makeProfile();
            for (auto& body : profile.mBodies)
                body.mMassFraction = 0.5f;
            RagdollCore::BuildOptions options;
            options.mTotalMass = 75.0f;
            options.mObjectLayer = sObjectLayer;
            options.mRootPosition = JPH::RVec3(offset, -offset, 4);
            options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), JPH::JPH_PI / 2.0f);
            RagdollCore::PassiveRagdollInitialState initial;
            const JPH::Vec3 drift = offset == 0.0f ? JPH::Vec3::sZero() : JPH::Vec3(20, 30, 40);
            initial.mBodyMotions = {
                { drift + JPH::Vec3(0, 1, 0), JPH::Vec3::sZero() },
                { drift + JPH::Vec3(0, -1, 0), JPH::Vec3::sZero() },
            };
            auto created = RagdollCore::createPassiveRagdoll(
                mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, initial, 7, 1234);
            ASSERT_TRUE(created.isValid());
            ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());
            RagdollCore::RagdollMomentum momentum;
            ASSERT_TRUE(created.mRagdoll->captureMomentum(momentum));
            EXPECT_LT((momentum.mLinearMomentum - drift * 75.0f).Length(), 1e-3f);
            EXPECT_LT((momentum.mAngularMomentum - JPH::Vec3(0, 0, 375)).Length(), 1e-2f);
        }
    }

    TEST_F(PassiveRagdollTest, solverForceMotorAvoidsMomentumInjectionFromFreeVelocityLimits)
    {
        for (const bool solverMotor : { false, true })
        {
            SCOPED_TRACE(solverMotor ? "solver_motor" : "free_integration");
            auto profile = RagdollCoreTest::makeProfile();
            profile.mBodies[0].mMassFraction = 0.99f;
            profile.mBodies[1].mMassFraction = 0.01f;
            profile.mBodies[1].mShape = profile.mBodies[0].mShape;
            profile.mBodies[1].mConstraint->mParentAnchor = JPH::Vec3(0, 10, 0);
            RagdollCore::BuildOptions options;
            options.mObjectLayer = sObjectLayer;
            options.mUseContinuousCollision = false;
            auto created = RagdollCore::createPassiveRagdoll(
                mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
            ASSERT_TRUE(created.isValid());
            auto& ragdoll = *created.mRagdoll;
            ASSERT_TRUE(ragdoll.addToPhysicsSystem());
            mPhysics.SetGravity(JPH::Vec3::sZero());
            auto& bodies = mPhysics.GetBodyInterface();
            JPH::FixedConstraintSettings fixed;
            fixed.mAutoDetectPoint = true;
            struct ConstraintCleanup
            {
                JPH::PhysicsSystem& mPhysics;
                JPH::Ref<JPH::TwoBodyConstraint> mConstraint;
                ~ConstraintCleanup() { if (mConstraint) mPhysics.RemoveConstraint(mConstraint); }
            } weld{ mPhysics, bodies.CreateConstraint(&fixed, ragdoll.getBodyId(0), ragdoll.getBodyId(1)) };
            mPhysics.AddConstraint(weld.mConstraint);
            RagdollCore::RagdollPose pose;
            ASSERT_TRUE(ragdoll.captureModelPose(pose));
            ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
            RagdollCore::UprightControllerOutput command;
            constexpr float dt = 1.0f / 60.0f;
            float force;
            {
                JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(1));
                ASSERT_TRUE(lock.Succeeded());
                const auto& body = lock.GetBody();
                const auto& properties = *body.GetMotionProperties();
                const float inverseInertia = properties.MultiplyWorldSpaceInverseInertiaByVector(
                    body.GetRotation(), JPH::Vec3::sAxisZ()).GetZ();
                // Ask for twice the free-spin limit using actual body
                // inertia. The rigid assembly needs no angular motion at all.
                force = 2.0f * properties.GetMaxAngularVelocity() / (inverseInertia * dt * 10.0f);
            }
            command.mPelvisForce = JPH::Vec3(force, 0, 0);
            auto controller = profile.mActiveController;
            controller.mMaximumPelvisForce = force;
            const std::array<std::size_t, 1> reaction{ 1 };
            ragdoll.beginActuationStep(solverMotor);
            ASSERT_TRUE(ragdoll.applyUprightControl(command, 0, 1, reaction, reaction, controller,
                solverMotor ? std::optional<std::size_t>(1) : std::nullopt, true));
            ASSERT_TRUE(ragdoll.beginSolverImpulseMeasurement(dt));
            JPH::TempAllocatorImpl allocator(1024 * 1024);
            JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
            ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
            RagdollCore::RagdollSolverImpulse impulse;
            ASSERT_TRUE(ragdoll.endSolverImpulseMeasurement(dt, impulse));
            RagdollCore::RagdollMomentum momentum;
            ASSERT_TRUE(ragdoll.captureMomentum(momentum));
            EXPECT_EQ(impulse.mFreeVelocityLimitTouched, !solverMotor);
            EXPECT_FALSE(impulse.mVelocityLimitTouched);
            if (solverMotor)
            {
                EXPECT_LT(momentum.mLinearMomentum.Length(), 1e-2f);
                EXPECT_LT(momentum.mAngularMomentum.Length(), 1e-2f);
            }
            else
            {
                EXPECT_GT(impulse.mFreeAngularLimitImpulse.Length(), 100.0f);
                EXPECT_GT(momentum.mAngularMomentum.Length(), 100.0f);
            }
            ragdoll.disableMotors();
            mPhysics.RemoveConstraint(weld.mConstraint);
            weld.mConstraint = nullptr;
            ragdoll.removeFromPhysicsSystem();
            EXPECT_TRUE(mPhysics.GetConstraints().empty());
        }
    }

    TEST_F(PassiveRagdollTest, freeVelocityLimitObserverMeasuresLinearMomentumLoss)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[0].mMassFraction = 0.99f;
        profile.mBodies[1].mMassFraction = 0.01f;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mUseContinuousCollision = false;
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        mPhysics.SetGravity(JPH::Vec3::sZero());
        for (const auto& constraint : mPhysics.GetConstraints())
            constraint->SetEnabled(false);
        constexpr float dt = 1.0f / 60.0f;
        std::array<float, 2> limitImpulses;
        for (std::size_t i = 0; i < 2; ++i)
        {
            JPH::BodyLockWrite lock(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(i));
            ASSERT_TRUE(lock.Succeeded());
            auto& properties = *lock.GetBody().GetMotionProperties();
            properties.SetLinearDamping(0.0f);
            properties.SetMaxLinearVelocity(10.0f);
            limitImpulses[i] = properties.GetMaxLinearVelocity() / properties.GetInverseMass();
        }
        // Stabilize redistributes the authored masses. Choose an impulse
        // between the two actual caps so exactly one body is clipped.
        ASSERT_GT(limitImpulses[0], limitImpulses[1]);
        const float commandImpulse = 0.5f * (limitImpulses[0] + limitImpulses[1]);
        const float force = commandImpulse / dt;
        const float expectedImpulse = commandImpulse - limitImpulses[1];
        // The line through both COMs gives no torque. Only the light body's
        // free linear velocity exceeds its limit; its missing reaction must
        // match the assembly's measured residual momentum.
        ASSERT_TRUE(ragdoll.applyBodyForcePair(0, 1, JPH::Vec3(0, force, 0), force));
        ASSERT_TRUE(ragdoll.beginSolverImpulseMeasurement(dt));
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        RagdollCore::RagdollSolverImpulse impulse;
        ASSERT_TRUE(ragdoll.endSolverImpulseMeasurement(dt, impulse));
        RagdollCore::RagdollMomentum momentum;
        ASSERT_TRUE(ragdoll.captureMomentum(momentum));
        EXPECT_TRUE(impulse.mFreeVelocityLimitTouched);
        EXPECT_LT((impulse.mFreeLinearLimitImpulse - JPH::Vec3(0, expectedImpulse, 0)).Length(), 1e-2f);
        EXPECT_LT((momentum.mLinearMomentum - impulse.mFreeLinearLimitImpulse).Length(), 1e-2f);
        EXPECT_LT(impulse.mFreeAngularLimitImpulse.Length(), 1e-2f);
        EXPECT_LT(impulse.mLinearImpulse.Length(), 1e-2f);
    }

    TEST_F(PassiveRagdollTest, solverWrenchesPreserveWorldForcesTorquesAndStepLifetime)
    {
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mUseContinuousCollision = false;
        options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3(1, 2, 3).Normalized(), 0.7f);
        auto created = RagdollCore::createPassiveRagdoll(mPhysics, RagdollCoreTest::makeSkeleton(),
            RagdollCoreTest::makeProfile(), options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        mPhysics.SetGravity(JPH::Vec3::sZero());
        // Isolate the actuator from anatomical constraints and contacts.
        for (const auto& constraint : mPhysics.GetConstraints())
            constraint->SetEnabled(false);
        auto& bodies = mPhysics.GetBodyInterface();
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        constexpr float dt = 1.0f / 60.0f;
        for (int step = 0; step < 4; ++step)
        {
            SCOPED_TRACE(step);
            // A cleared step between commands must not replay a warm start;
            // the last step disables motors after queuing a command.
            ragdoll.beginActuationStep(true);
            std::array<JPH::Vec3, 2> expectedLinear;
            std::array<JPH::Vec3, 2> expectedAngular;
            const JPH::Vec3 force = step == 1 ? JPH::Vec3::sZero() : JPH::Vec3(120, -80, 40);
            const JPH::Vec3 torque = step == 1 ? JPH::Vec3::sZero() : JPH::Vec3(-13, 19, 7);
            ASSERT_TRUE(ragdoll.applyBodyForcePair(0, 1, force * 0.25f, 200));
            ASSERT_TRUE(ragdoll.applyBodyForcePair(0, 1, force * 0.75f, 200));
            ASSERT_TRUE(ragdoll.applyBodyTorquePair(0, 1, torque, 50));
            RagdollCore::RagdollPose pose;
            ASSERT_TRUE(ragdoll.captureModelPose(pose));
            ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
            if (step == 3)
                ragdoll.disableMotors();
            const JPH::RVec3 midpoint = 0.5 * (bodies.GetCenterOfMassPosition(ragdoll.getBodyId(0))
                + bodies.GetCenterOfMassPosition(ragdoll.getBodyId(1)));
            for (std::size_t i = 0; i < 2; ++i)
            {
                bodies.SetLinearAndAngularVelocity(ragdoll.getBodyId(i), JPH::Vec3::sZero(), JPH::Vec3::sZero());
                JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(i));
                ASSERT_TRUE(lock.Succeeded());
                const auto& body = lock.GetBody();
                const auto& properties = *body.GetMotionProperties();
                const float sign = i == 0 ? 1.0f : -1.0f;
                expectedLinear[i] = step == 3 ? JPH::Vec3::sZero()
                    : sign * force * (dt * properties.GetInverseMass());
                const JPH::Vec3 angularImpulse = sign * dt
                    * (JPH::Vec3(midpoint - body.GetCenterOfMassPosition()).Cross(force) + torque);
                expectedAngular[i] = step == 3 ? JPH::Vec3::sZero()
                    : properties.MultiplyWorldSpaceInverseInertiaByVector(body.GetRotation(), angularImpulse);
            }
            ASSERT_TRUE(ragdoll.beginSolverImpulseMeasurement(dt));
            ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
            RagdollCore::RagdollSolverImpulse impulse;
            ASSERT_TRUE(ragdoll.endSolverImpulseMeasurement(dt, impulse));
            EXPECT_FALSE(impulse.mFreeVelocityLimitTouched);
            EXPECT_FALSE(impulse.mVelocityLimitTouched);
            EXPECT_LT(impulse.mLinearImpulse.Length(), 1e-3f);
            EXPECT_LT(impulse.mAngularImpulse.Length(), 1e-3f);
            for (std::size_t i = 0; i < 2; ++i)
            {
                EXPECT_LT((bodies.GetLinearVelocity(ragdoll.getBodyId(i)) - expectedLinear[i]).Length(), 1e-4f);
                EXPECT_LT((bodies.GetAngularVelocity(ragdoll.getBodyId(i)) - expectedAngular[i]).Length(), 1e-4f);
            }
            EXPECT_EQ(mPhysics.GetConstraints().size(), 2u);
        }
        ragdoll.removeFromPhysicsSystem();
        EXPECT_TRUE(mPhysics.GetConstraints().empty());
    }

    TEST_F(PassiveRagdollTest, nativePositionMotorTracksAMovingReference)
    {
        for (const auto axis : { JPH::Vec3::sAxisX(), JPH::Vec3::sAxisY(), JPH::Vec3::sAxisZ() })
        for (const bool compensate : { false, true })
        {
            SCOPED_TRACE(compensate);
            auto skeleton = RagdollCoreTest::makeSkeleton();
            skeleton.mBones[1].mModelTransform.mTranslation = JPH::Vec3::sZero();
            auto profile = RagdollCoreTest::makeProfile();
            for (auto& body : profile.mBodies)
            {
                body.mMaterial.mLinearDamping = 0;
                body.mMaterial.mAngularDamping = 0;
            }
            auto& limits = *profile.mBodies[1].mConstraint;
            limits.mNormalHalfConeAngle = limits.mPlaneHalfConeAngle = 1.4f;
            limits.mTwistMinAngle = -1.4f;
            limits.mTwistMaxAngle = 1.4f;
            limits.mMotor.mFrequency = 6;
            limits.mMotor.mMaximumTorque = 1000;
            RagdollCore::BuildOptions options;
            options.mObjectLayer = sObjectLayer;
            options.mUseContinuousCollision = false;
            options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.4f);
            auto created = RagdollCore::createPassiveRagdoll(mPhysics, skeleton, profile, options, {}, 7, 1234);
            ASSERT_TRUE(created.isValid());
            auto& ragdoll = *created.mRagdoll;
            ASSERT_TRUE(ragdoll.addToPhysicsSystem());
            auto& bodies = mPhysics.GetBodyInterface();
            bodies.SetMotionType(ragdoll.getBodyId(0), JPH::EMotionType::Kinematic, JPH::EActivation::Activate);
            bodies.SetAngularVelocity(ragdoll.getBodyId(0), axis);
            mPhysics.SetGravity(JPH::Vec3::sZero());
            auto* joint = static_cast<JPH::SwingTwistConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
            JPH::TempAllocatorImpl allocator(1024 * 1024);
            JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
            float maximumSpeed = 0;
            for (unsigned step = 0; step < 30; ++step)
            {
                RagdollCore::RagdollPose pose;
                ASSERT_TRUE(ragdoll.captureModelPose(pose));
                ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
                const auto parentFrame = joint->GetBody1()->GetRotation() * joint->GetConstraintToBody1();
                const auto childFrame = joint->GetBody2()->GetRotation() * joint->GetConstraintToBody2();
                joint->SetTargetOrientationCS((parentFrame.Conjugated() * options.mRootRotation
                    * joint->GetConstraintToBody2()).Normalized());
                joint->SetTargetAngularVelocityCS(compensate ? childFrame.Conjugated() * -axis : JPH::Vec3::sZero());
                ASSERT_EQ(mPhysics.Update(1.0f / 60.0f, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
                maximumSpeed = std::max(maximumSpeed, bodies.GetAngularVelocity(ragdoll.getBodyId(1)).Length());
            }
            if (compensate)
            {
                EXPECT_LT(maximumSpeed, 1e-3f);
                EXPECT_GT(std::abs(bodies.GetRotation(ragdoll.getBodyId(1)).Dot(options.mRootRotation)), 0.99999f);
            }
            else
                EXPECT_GT(maximumSpeed, 0.05f);
        }
    }

    TEST_F(PassiveRagdollTest, nativePositionMotorsTrackMovingReferenceVelocity)
    {
        for (const auto type : { RagdollCore::ConstraintType::SwingTwist, RagdollCore::ConstraintType::Hinge })
        {
            double errors[2]{};
            for (int feedForward = 0; feedForward < 2; ++feedForward)
            {
                SCOPED_TRACE(type == RagdollCore::ConstraintType::Hinge ? "hinge" : "swing_twist");
                SCOPED_TRACE(feedForward);
                auto profile = RagdollCoreTest::makeProfile();
                for (auto& body : profile.mBodies)
                {
                    body.mMaterial.mLinearDamping = 0;
                    body.mMaterial.mAngularDamping = 0;
                }
                auto& settings = *profile.mBodies[1].mConstraint;
                settings.mType = type;
                settings.mDeriveFrameFromSkeleton = true;
                settings.mMotor.mFrequency = 6;
                settings.mMotor.mDamping = 1;
                settings.mMotor.mMaximumTorque = 1000;
                RagdollCore::BuildOptions options;
                options.mObjectLayer = sObjectLayer;
                options.mUseContinuousCollision = false;
                options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3(1, 2, 3).Normalized(), 0.7f);
                auto created = RagdollCore::createPassiveRagdoll(
                    mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
                ASSERT_TRUE(created.isValid());
                auto& ragdoll = *created.mRagdoll;
                ASSERT_TRUE(ragdoll.addToPhysicsSystem());
                auto& bodies = mPhysics.GetBodyInterface();
                bodies.SetMotionType(ragdoll.getBodyId(0), JPH::EMotionType::Static, JPH::EActivation::DontActivate);
                mPhysics.SetGravity(JPH::Vec3::sZero());
                RagdollCore::RagdollPose pose;
                ASSERT_TRUE(ragdoll.captureModelPose(pose));
                auto* joint = mPhysics.GetConstraints()[0].GetPtr();
                constexpr float dt = 1.0f / 60.0f, velocity = 0.2f;
                JPH::TempAllocatorImpl allocator(1024 * 1024);
                JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
                for (int step = 0; step < 180; ++step)
                {
                    const float angle = std::min(step, 60) * dt * velocity;
                    const float targetVelocity = feedForward && step < 60 ? velocity : 0;
                    ragdoll.beginActuationStep(false);
                    ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
                    RagdollCore::MotorTargetTelemetry reset;
                    ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, reset));
                    EXPECT_TRUE(reset.mTargetAngularVelocity.IsNearZero());
                    ASSERT_TRUE(reset.mDrive);
                    EXPECT_EQ(reset.mDrive->mSwingOrHingeState, static_cast<int>(JPH::EMotorState::Position));
                    EXPECT_FLOAT_EQ(reset.mDrive->mMaximumTorque.GetX(), 1000);
                    EXPECT_FLOAT_EQ(reset.mDrive->mMinimumTorque.GetX(), -1000);
                    if (type == RagdollCore::ConstraintType::SwingTwist && step == 0)
                    {
                        auto* motor = static_cast<JPH::SwingTwistConstraint*>(joint);
                        ASSERT_TRUE(reset.mChildWorldTarget);
                        ASSERT_TRUE(reset.mParentWorldTarget);
                        EXPECT_TRUE(reset.mChildWorldTarget->IsClose(motor->GetBody2()->GetRotation(), 1e-10f));
                        EXPECT_TRUE(reset.mParentWorldTarget->IsClose(motor->GetBody1()->GetRotation(), 1e-10f));
                    }
                    JPH::Vec3 worldVelocity;
                    if (type == RagdollCore::ConstraintType::Hinge)
                    {
                        auto* motor = static_cast<JPH::HingeConstraint*>(joint);
                        motor->SetTargetAngle(angle);
                        const auto axis = motor->GetBody1()->GetRotation() * motor->GetLocalSpaceHingeAxis1();
                        worldVelocity = axis * targetVelocity;
                        // A hinge accepts only its axial component.
                        ASSERT_TRUE(ragdoll.setMotorTargetAngularVelocity(1,
                            worldVelocity + axis.GetNormalizedPerpendicular()));
                    }
                    else
                    {
                        auto* motor = static_cast<JPH::SwingTwistConstraint*>(joint);
                        motor->SetTargetOrientationCS(JPH::Quat::sRotation(JPH::Vec3::sAxisY(), angle));
                        worldVelocity = (motor->GetBody2()->GetRotation() * motor->GetConstraintToBody2())
                            * (JPH::Vec3::sAxisY() * targetVelocity);
                        ASSERT_TRUE(ragdoll.setMotorTargetAngularVelocity(1, worldVelocity));
                    }
                    EXPECT_FALSE(ragdoll.setMotorTargetAngularVelocity(0, worldVelocity));
                    EXPECT_FALSE(ragdoll.setMotorTargetAngularVelocity(ragdoll.getBodyCount(), worldVelocity));
                    EXPECT_FALSE(ragdoll.setMotorTargetAngularVelocity(1,
                        JPH::Vec3(std::numeric_limits<float>::quiet_NaN(), 0, 0)));
                    ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, reset));
                    EXPECT_LT((reset.mTargetAngularVelocity - worldVelocity).Length(), 1e-6f);
                    ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
                    const float expected = std::min(step + 1, 60) * dt * velocity;
                    RagdollCore::MotorTargetTelemetry observed;
                    ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, observed));
                    float error;
                    if (type == RagdollCore::ConstraintType::Hinge)
                        error = std::abs(*observed.mCurrentHingeAngle - expected);
                    else
                    {
                        auto difference = (JPH::Quat::sRotation(JPH::Vec3::sAxisY(), expected)
                            * observed.mCurrentOrientation->Conjugated()).Normalized();
                        error = 2 * std::atan2(difference.GetXYZ().Length(), std::abs(difference.GetW()));
                    }
                    if (step >= 30 && step < 60) errors[feedForward] += error;
                    JPH::Vec3 impulse;
                    float utilization;
                    ASSERT_TRUE(ragdoll.getMotorAngularImpulse(1, dt, impulse, utilization));
                    EXPECT_LE(utilization, 1);
                    if (step >= 150)
                    {
                        EXPECT_LT(error, 1e-3f);
                        EXPECT_LT(bodies.GetAngularVelocity(ragdoll.getBodyId(1)).Length(), 1e-3f);
                    }
                }
                ASSERT_TRUE(ragdoll.setMotorTargetAngularVelocity(1, JPH::Vec3(0.2f, 0.3f, 0.4f)));
                if (type == RagdollCore::ConstraintType::SwingTwist)
                {
                    ASSERT_TRUE(ragdoll.driveMotorWithFreeRotationAxis(1, JPH::Vec3::sAxisX(), dt));
                    EXPECT_TRUE(static_cast<JPH::SwingTwistConstraint*>(joint)->GetTargetAngularVelocityCS().IsNearZero());
                    EXPECT_FALSE(ragdoll.setMotorTargetAngularVelocity(1, JPH::Vec3::sAxisY()));
                    ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
                    ASSERT_TRUE(ragdoll.setMotorTargetAngularVelocity(1, JPH::Vec3::sAxisY()));
                    ASSERT_TRUE(ragdoll.driveMotorWithLoadTorque(1, JPH::Vec3::sZero(), dt,
                        std::nullopt, bodies.GetRotation(ragdoll.getBodyId(1))));
                    EXPECT_FALSE(ragdoll.setMotorTargetAngularVelocity(1, JPH::Vec3::sAxisY()));
                    RagdollCore::MotorTargetTelemetry worldTarget;
                    ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, worldTarget));
                    EXPECT_TRUE(worldTarget.mWorldOrientation);
                    EXPECT_TRUE(worldTarget.mTargetAngularVelocity.IsNearZero());
                }
                ragdoll.disableMotors();
                RagdollCore::MotorTargetTelemetry disabledTarget;
                EXPECT_FALSE(ragdoll.captureMotorTargetTelemetry(1, disabledTarget));
                ASSERT_TRUE(disabledTarget.mDrive);
                EXPECT_EQ(disabledTarget.mDrive->mSwingOrHingeState, static_cast<int>(JPH::EMotorState::Off));
                EXPECT_FALSE(ragdoll.setMotorTargetAngularVelocity(1, JPH::Vec3::sZero()));
                if (type == RagdollCore::ConstraintType::Hinge)
                    EXPECT_EQ(static_cast<JPH::HingeConstraint*>(joint)->GetTargetAngularVelocity(), 0);
                else
                    EXPECT_TRUE(static_cast<JPH::SwingTwistConstraint*>(joint)->GetTargetAngularVelocityCS().IsNearZero());
            }
            std::cout << "TRAJECTORY_VELOCITY type=" << static_cast<int>(type)
                      << " zero_error=" << errors[0] / 30 << " feedforward_error=" << errors[1] / 30 << std::endl;
            EXPECT_GT(errors[0], 1e-3);
            EXPECT_LT(errors[1], errors[0] / 4);
        }
    }

    TEST_F(PassiveRagdollTest, acceleratedHingeReferenceSeparatesTimingAndInertialLoad)
    {
        // The implicit motor solves v_next - v_ref + dt*k*gamma*C
        // + gamma*(lambda - dt*load) = 0, C = angle_now - target_now.
        // A next-sample secant reference and I*(v_ref-v_ref_previous)/dt
        // therefore reproduce a sampled accelerating trajectory without
        // requiring spring deflection. Current-time velocity alone does not.
        constexpr float dt = 1.0f / 60.0f;
        constexpr int motionSteps = 21;
        constexpr float excursion = 0.2f;
        const auto position = [](int step) {
            const double u = std::clamp(static_cast<double>(step) / motionSteps, 0.0, 1.0);
            return excursion * u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
        };
        std::array<double, 4> maximumErrors{};
        for (const bool secantReference : { false, true })
        for (const bool inertialLoad : { false, true })
        {
            SCOPED_TRACE(secantReference);
            SCOPED_TRACE(inertialLoad);
            const std::size_t variant = 2 * secantReference + inertialLoad;
            auto profile = RagdollCoreTest::makeProfile();
            for (auto& body : profile.mBodies)
            {
                body.mMaterial.mLinearDamping = 0;
                body.mMaterial.mAngularDamping = 0;
            }
            // The derived joint anchor is at this sphere's center of mass.
            // Its isotropic inertia removes changing-axis and anchor terms.
            profile.mBodies[1].mShape.mType = RagdollCore::ShapeType::Sphere;
            profile.mBodies[1].mShape.mRadius = 1;
            auto& jointProfile = *profile.mBodies[1].mConstraint;
            jointProfile.mType = RagdollCore::ConstraintType::Hinge;
            jointProfile.mDeriveFrameFromSkeleton = true;
            jointProfile.mMotor.mFrequency = 6;
            jointProfile.mMotor.mDamping = 1;
            jointProfile.mMotor.mMaximumTorque = 1000;
            RagdollCore::BuildOptions options;
            options.mObjectLayer = sObjectLayer;
            options.mUseContinuousCollision = false;
            options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3(1, 2, 3).Normalized(), 0.7f);
            auto created = RagdollCore::createPassiveRagdoll(
                mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
            ASSERT_TRUE(created.isValid());
            auto& ragdoll = *created.mRagdoll;
            ASSERT_TRUE(ragdoll.addToPhysicsSystem());
            auto& bodies = mPhysics.GetBodyInterface();
            bodies.SetMotionType(ragdoll.getBodyId(0), JPH::EMotionType::Static, JPH::EActivation::DontActivate);
            mPhysics.SetGravity(JPH::Vec3::sZero());
            auto* joint = static_cast<JPH::HingeConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
            EXPECT_TRUE(joint->GetLocalSpacePoint2().IsNearZero());
            const auto axis = joint->GetBody1()->GetRotation() * joint->GetLocalSpaceHingeAxis1();
            float inertia;
            {
                JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(1));
                ASSERT_TRUE(lock.Succeeded());
                const auto& body = lock.GetBody();
                const auto& properties = *body.GetMotionProperties();
                const float inverseInertia = axis.Dot(
                    properties.MultiplyWorldSpaceInverseInertiaByVector(body.GetRotation(), axis));
                ASSERT_GT(inverseInertia, 0);
                inertia = 1 / inverseInertia;
                EXPECT_NEAR(inertia, 0.4f / properties.GetInverseMass(), 1e-4f);
            }
            RagdollCore::RagdollPose pose;
            ASSERT_TRUE(ragdoll.captureModelPose(pose));
            JPH::TempAllocatorImpl allocator(1024 * 1024);
            JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
            float maximumUtilization = 0;
            for (int step = 0; step < motionSteps + 60; ++step)
            {
                const double u = std::clamp(static_cast<double>(step) / motionSteps, 0.0, 1.0);
                const float analyticVelocity = static_cast<float>(
                    excursion * 30.0 * u * u * (1 - u) * (1 - u) / (motionSteps * dt));
                const float secantVelocity = static_cast<float>((position(step + 1) - position(step)) / dt);
                const float previousVelocity = static_cast<float>((position(step) - position(step - 1)) / dt);
                const float torque = inertialLoad ? inertia * (secantVelocity - previousVelocity) / dt : 0;
                ragdoll.beginActuationStep(false);
                ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
                joint->SetTargetAngle(static_cast<float>(position(step)));
                ASSERT_TRUE(ragdoll.setMotorTargetAngularVelocity(1,
                    axis * (secantReference ? secantVelocity : analyticVelocity)));
                ASSERT_TRUE(ragdoll.driveMotorWithLoadTorque(1, axis * torque, dt));
                ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
                const double error = std::abs(joint->GetCurrentAngle() - position(step + 1));
                maximumErrors[variant] = std::max(maximumErrors[variant], error);
                JPH::Vec3 impulse;
                float utilization;
                ASSERT_TRUE(ragdoll.getMotorAngularImpulse(1, dt, impulse, utilization));
                maximumUtilization = std::max(maximumUtilization, utilization);
                EXPECT_LT(utilization, 1);
                EXPECT_GT(joint->GetCurrentAngle(), jointProfile.mHingeMinAngle);
                EXPECT_LT(joint->GetCurrentAngle(), jointProfile.mHingeMaxAngle);
                if (secantReference && inertialLoad)
                {
                    EXPECT_LT(error, 1e-4);
                    EXPECT_NEAR(bodies.GetAngularVelocity(ragdoll.getBodyId(1)).Dot(axis), secantVelocity, 1e-3f);
                    EXPECT_NEAR(impulse.Dot(axis) / dt, torque, 0.1f);
                }
                if (step >= motionSteps + 50)
                {
                    EXPECT_LT(error, 1e-3);
                    EXPECT_LT(bodies.GetAngularVelocity(ragdoll.getBodyId(1)).Length(), 1e-3f);
                }
            }
            std::cout << "ACCELERATED_HINGE secant=" << secantReference << " inertia_load=" << inertialLoad
                      << " inertia=" << inertia << " max_error=" << maximumErrors[variant]
                      << " max_utilization=" << maximumUtilization << std::endl;
        }
        // Both omissions are observable on this independent motion. The load
        // cannot repair a reference evaluated at a different point in time.
        EXPECT_GT(maximumErrors[0], 1e-3);
        EXPECT_GT(maximumErrors[1], 1e-3);
        EXPECT_GT(maximumErrors[2], 1e-3);
        EXPECT_LT(maximumErrors[3], 1e-4);
    }

    TEST_F(PassiveRagdollTest, reducedHumanoidLegSeparatesLocalAndCoupledOneStepDemand)
    {
        using namespace RagdollCore;
        // This named reduction preserves the synthetic humanoid's actual leg
        // masses, shapes, COM offsets, native limits, gains, and capacities.
        // The pinned pelvis supplies an external reaction. It qualifies a
        // state-dependent one-step computed demand, not reference-only
        // feedforward, floating-base balance, or a complete swing trajectory.
        constexpr float dt = 1.0f / 60.0f;
        constexpr float requestedSpeed = 0.2f;
        constexpr float angularTolerance = 1e-3f;
        constexpr float linearTolerance = 59.5f * angularTolerance;
        constexpr std::array<std::size_t, 4> selected{ 0, 17, 18, 19 };
        const auto fullSkeleton = makeScenarioHumanoidSkeleton();
        const auto fullProfile = makeScenarioHumanoidProfile();
        auto skeleton = fullSkeleton;
        auto profile = fullProfile;
        skeleton.mId = "reduced_synthetic_right_leg";
        skeleton.mBones.clear();
        profile.mId = "reduced_synthetic_right_leg";
        profile.mBodies.clear();
        profile.mLandmarks = {};
        profile.mLandmarks.mPelvis = "pelvis";
        profile.mLandmarks.mRightFoot = "right_foot";
        float retainedMassFraction = 0;
        for (const auto index : selected)
            retainedMassFraction += fullProfile.mBodies[index].mMassFraction;
        for (std::size_t i = 0; i < selected.size(); ++i)
        {
            auto bone = fullSkeleton.mBones[selected[i]];
            bone.mParent = static_cast<int>(i) - 1;
            skeleton.mBones.push_back(bone);
            auto body = fullProfile.mBodies[selected[i]];
            body.mMassFraction /= retainedMassFraction;
            body.mMaterial.mLinearDamping = 0;
            body.mMaterial.mAngularDamping = 0;
            profile.mBodies.push_back(body);
        }

        enum class LoadVariant { None, LocalRelativeInertia, CoupledSubtree };
        const std::array variants{ LoadVariant::None, LoadVariant::LocalRelativeInertia,
            LoadVariant::CoupledSubtree };
        for (const bool oblique : { false, true })
        for (const bool kneeOnly : { false, true })
        {
            std::array<float, 3> maximumAngularErrors{};
            for (std::size_t variantIndex = 0; variantIndex < variants.size(); ++variantIndex)
            {
                SCOPED_TRACE(oblique);
                SCOPED_TRACE(kneeOnly);
                SCOPED_TRACE(variantIndex);
                BuildOptions options;
                options.mObjectLayer = sObjectLayer;
                options.mTotalMass = 75.0f * retainedMassFraction;
                options.mUseContinuousCollision = false;
                options.mRootRotation = oblique
                    ? JPH::Quat::sRotation(JPH::Vec3(1, 2, 3).Normalized(), 0.7f)
                    : JPH::Quat::sIdentity();
                auto created = createPassiveRagdoll(mPhysics, skeleton, profile, options, {}, 7, 1234);
                ASSERT_TRUE(created.isValid());
                auto& ragdoll = *created.mRagdoll;
                ASSERT_TRUE(ragdoll.addToPhysicsSystem());
                auto& bodies = mPhysics.GetBodyInterface();
                bodies.SetMotionType(ragdoll.getBodyId(0), JPH::EMotionType::Static,
                    JPH::EActivation::DontActivate);
                mPhysics.SetGravity(JPH::Vec3::sZero());
                const auto axis = options.mRootRotation * JPH::Vec3::sAxisX();

                struct DemandBody
                {
                    JPH::RVec3 mCenter = JPH::RVec3::sZero();
                    JPH::Quat mInertiaFrame = JPH::Quat::sIdentity();
                    JPH::Vec3 mInertiaDiagonal = JPH::Vec3::sZero();
                    JPH::Vec3 mDesiredLinear = JPH::Vec3::sZero();
                    JPH::Vec3 mDesiredAngular = JPH::Vec3::sZero();
                    JPH::Vec3 mLinearImpulse = JPH::Vec3::sZero();
                    JPH::Vec3 mAngularImpulse = JPH::Vec3::sZero();
                    float mMass = 0;

                    JPH::Vec3 inertiaTimes(JPH::Vec3Arg value) const
                    {
                        return mInertiaFrame * (mInertiaDiagonal * (mInertiaFrame.Conjugated() * value));
                    }
                };
                std::array<DemandBody, 4> demand;
                demand[0].mCenter = bodies.GetCenterOfMassPosition(ragdoll.getBodyId(0));
                std::array<JPH::TwoBodyConstraint*, 4> joints{};
                for (const auto& entry : mPhysics.GetConstraints())
                {
                    auto* joint = static_cast<JPH::TwoBodyConstraint*>(entry.GetPtr());
                    for (std::size_t i = 1; i < demand.size(); ++i)
                        if (joint->GetBody2()->GetID() == ragdoll.getBodyId(i))
                            joints[i] = joint;
                }
                std::array<JPH::RVec3, 4> anchors;
                for (std::size_t i = 1; i < demand.size(); ++i)
                {
                    ASSERT_NE(joints[i], nullptr);
                    const auto& joint = *joints[i];
                    ASSERT_EQ(joint.GetBody1()->GetID(), ragdoll.getBodyId(i - 1));
                    const auto parentAnchor = joint.GetBody1()->GetCenterOfMassTransform()
                        * joint.GetConstraintToBody1Matrix();
                    const auto childAnchor = joint.GetBody2()->GetCenterOfMassTransform()
                        * joint.GetConstraintToBody2Matrix();
                    anchors[i] = childAnchor.GetTranslation();
                    ASSERT_LT((anchors[i] - parentAnchor.GetTranslation()).Length(), 1e-4);
                    JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(i));
                    ASSERT_TRUE(lock.Succeeded());
                    const auto& body = lock.GetBody();
                    const auto& properties = *body.GetMotionProperties();
                    auto& current = demand[i];
                    current.mCenter = body.GetCenterOfMassPosition();
                    current.mMass = 1.0f / properties.GetInverseMass();
                    current.mInertiaFrame = body.GetRotation() * properties.GetInertiaRotation();
                    current.mInertiaDiagonal = JPH::Vec3::sOne() / properties.GetInverseInertiaDiagonal();
                    EXPECT_NEAR(current.mMass, 75.0f * fullProfile.mBodies[selected[i]].mMassFraction, 1e-5f);
                    // The fitted capsule shape is offset back to the authored
                    // bone COM; its midpoint must not be substituted here.
                    ASSERT_LT((current.mCenter - anchors[i]).Length(), 1e-4);
                    ASSERT_TRUE(body.GetLinearVelocity().IsNearZero());
                    ASSERT_TRUE(body.GetAngularVelocity().IsNearZero());
                    current.mDesiredAngular = axis * (kneeOnly && i == 1 ? 0.0f : requestedSpeed);
                    // Jolt constrains material-point velocities at the current
                    // anchors. A next-pose COM chord is not this tangent.
                    current.mDesiredLinear = demand[i - 1].mDesiredLinear
                        + demand[i - 1].mDesiredAngular.Cross(JPH::Vec3(anchors[i] - demand[i - 1].mCenter))
                        - current.mDesiredAngular.Cross(JPH::Vec3(anchors[i] - current.mCenter));
                    current.mLinearImpulse = current.mMass * (current.mDesiredLinear - body.GetLinearVelocity());
                    current.mAngularImpulse = current.inertiaTimes(current.mDesiredAngular - body.GetAngularVelocity());
                }

                std::array<JPH::Vec3, 4> coupledImpulses;
                for (std::size_t i = 1; i < demand.size(); ++i)
                {
                    coupledImpulses[i] = JPH::Vec3::sZero();
                    for (std::size_t descendant = i; descendant < demand.size(); ++descendant)
                    {
                        const auto& body = demand[descendant];
                        coupledImpulses[i] += body.mAngularImpulse
                            + JPH::Vec3(body.mCenter - anchors[i]).Cross(body.mLinearImpulse);
                    }
                }
                // Independent straight-leg lever-arm identities expose the
                // off-diagonal demand omitted by local I * relative delta-w.
                const float thighInertia = demand[1].inertiaTimes(axis).Dot(axis);
                const float calfInertia = demand[2].inertiaTimes(axis).Dot(axis);
                const float footInertia = demand[3].inertiaTimes(axis).Dot(axis);
                EXPECT_NEAR((anchors[2] - anchors[1]).Length(), 29.0, 1e-4);
                EXPECT_NEAR((anchors[3] - anchors[2]).Length(), 30.5, 1e-4);
                const float expectedHipImpulse = requestedSpeed * (kneeOnly
                    ? calfInertia + footInertia + demand[3].mMass * 59.5f * 30.5f
                    : thighInertia + calfInertia + footInertia + demand[2].mMass * 29.0f * 29.0f
                        + demand[3].mMass * 59.5f * 59.5f);
                const float expectedKneeImpulse = requestedSpeed * (calfInertia + footInertia
                    + demand[3].mMass * 30.5f * (kneeOnly ? 30.5f : 59.5f));
                EXPECT_NEAR(coupledImpulses[1].Dot(axis), expectedHipImpulse, 0.01f);
                EXPECT_NEAR(coupledImpulses[2].Dot(axis), expectedKneeImpulse, 0.01f);
                EXPECT_NEAR(coupledImpulses[3].Dot(axis), footInertia * requestedSpeed, 0.01f);
                EXPECT_GT(expectedHipImpulse, 0);

                RagdollPose currentPose;
                ASSERT_TRUE(ragdoll.captureModelPose(currentPose));
                ragdoll.beginActuationStep(false);
                ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(currentPose));
                for (std::size_t i = 1; i < demand.size(); ++i)
                {
                    const auto relativeAngular = demand[i].mDesiredAngular - demand[i - 1].mDesiredAngular;
                    ASSERT_TRUE(ragdoll.setMotorTargetAngularVelocity(i, relativeAngular));
                    const auto impulse = variants[variantIndex] == LoadVariant::CoupledSubtree
                        ? coupledImpulses[i] : variants[variantIndex] == LoadVariant::LocalRelativeInertia
                            ? demand[i].inertiaTimes(relativeAngular) : JPH::Vec3::sZero();
                    ASSERT_TRUE(ragdoll.driveMotorWithLoadTorque(i, impulse / dt, dt));
                    MotorTargetTelemetry target;
                    ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(i, target));
                    EXPECT_LT(target.mTargetClampDelta, 1e-5f);
                    EXPECT_LT((target.mTargetAngularVelocity - relativeAngular).Length(), 1e-5f);
                }
                JPH::TempAllocatorImpl allocator(1024 * 1024);
                JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
                ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
                float maximumLinearError = 0;
                float maximumImpulseRelativeError = 0;
                float maximumUtilization = 0;
                for (std::size_t i = 1; i < demand.size(); ++i)
                {
                    const float angularError = (bodies.GetAngularVelocity(ragdoll.getBodyId(i))
                        - demand[i].mDesiredAngular).Length();
                    const float linearError = (bodies.GetLinearVelocity(ragdoll.getBodyId(i))
                        - demand[i].mDesiredLinear).Length();
                    maximumAngularErrors[variantIndex] = std::max(maximumAngularErrors[variantIndex], angularError);
                    maximumLinearError = std::max(maximumLinearError, linearError);
                    JPH::Vec3 actualImpulse;
                    float utilization;
                    ASSERT_TRUE(ragdoll.getMotorAngularImpulse(i, dt, actualImpulse, utilization));
                    maximumUtilization = std::max(maximumUtilization, utilization);
                    EXPECT_LT(utilization, 1);
                    const float impulseScale = std::max(1.0f, coupledImpulses[i].Length());
                    const float impulseRelativeError = (actualImpulse - coupledImpulses[i]).Length() / impulseScale;
                    maximumImpulseRelativeError = std::max(maximumImpulseRelativeError, impulseRelativeError);
                    if (variants[variantIndex] == LoadVariant::CoupledSubtree)
                    {
                        EXPECT_LT(angularError, angularTolerance);
                        EXPECT_LT(linearError, linearTolerance);
                        EXPECT_LT(impulseRelativeError, 0.005f);
                    }
                    const auto parentAnchor = joints[i]->GetBody1()->GetCenterOfMassTransform()
                        * joints[i]->GetConstraintToBody1Matrix();
                    const auto childAnchor = joints[i]->GetBody2()->GetCenterOfMassTransform()
                        * joints[i]->GetConstraintToBody2Matrix();
                    EXPECT_LT((parentAnchor.GetTranslation() - childAnchor.GetTranslation()).Length(), 1e-3);
                }
                EXPECT_TRUE(bodies.GetLinearVelocity(ragdoll.getBodyId(0)).IsNearZero());
                EXPECT_TRUE(bodies.GetAngularVelocity(ragdoll.getBodyId(0)).IsNearZero());
                std::cout << "COUPLED_LEG_ONE_STEP oblique=" << oblique << " knee_only=" << kneeOnly
                          << " load=" << variantIndex << " max_angular_error=" << maximumAngularErrors[variantIndex]
                          << " max_linear_error=" << maximumLinearError
                          << " max_impulse_relative_error=" << maximumImpulseRelativeError
                          << " max_utilization=" << maximumUtilization << std::endl;
            }
            EXPECT_GT(maximumAngularErrors[0], angularTolerance);
            EXPECT_GT(maximumAngularErrors[1], angularTolerance);
            EXPECT_LT(maximumAngularErrors[2], angularTolerance);
        }
    }

    TEST_F(PassiveRagdollTest, floatingHumanoidPreservesMomentumWhileMeetingOneStepJointDemand)
    {
        using namespace RagdollCore;
        // A fixed-configuration impulse contract on the complete dynamic tree.
        // It supplies no root wrench and makes no claim about contact balance
        // or a finite walking trajectory. Inertias and anchors stay frozen only
        // in the calculation/check; Jolt advances the normal dynamic bodies.
        constexpr std::size_t count = 20;
        constexpr float dt = 1.0f / 60.0f;
        constexpr float requestedSpeed = 0.2f;
        constexpr float angularTolerance = 1e-3f;
        constexpr float linearTolerance = 59.5f * angularTolerance;
        const auto skeleton = makeScenarioHumanoidSkeleton();
        auto profile = makeScenarioHumanoidProfile();
        ASSERT_EQ(skeleton.mBones.size(), count);
        ASSERT_EQ(profile.mBodies.size(), count);
        for (auto& body : profile.mBodies)
        {
            body.mMaterial.mLinearDamping = 0;
            body.mMaterial.mAngularDamping = 0;
            // Isolate internal joint actuation from contact. The unmodified
            // bind pose overlaps head and upper spine by one unit. Preserve
            // every body/constraint/inertia, but exclude self-contact only in
            // this zero-gravity mechanics fixture; walking keeps its filter.
            body.mSelfCollisionGroup = 1;
        }

        struct DemandBody
        {
            JPH::RVec3 mCenter = JPH::RVec3::sZero();
            JPH::Quat mInertiaFrame = JPH::Quat::sIdentity();
            JPH::Vec3 mInertiaDiagonal = JPH::Vec3::sZero();
            JPH::Vec3 mShapeLinear = JPH::Vec3::sZero();
            JPH::Vec3 mShapeAngular = JPH::Vec3::sZero();
            JPH::Vec3 mDesiredLinear = JPH::Vec3::sZero();
            JPH::Vec3 mDesiredAngular = JPH::Vec3::sZero();
            JPH::Vec3 mLinearImpulse = JPH::Vec3::sZero();
            JPH::Vec3 mAngularImpulse = JPH::Vec3::sZero();
            float mMass = 0;

            JPH::Vec3 inertiaTimes(JPH::Vec3Arg value) const
            {
                return mInertiaFrame * (mInertiaDiagonal * mInertiaFrame.InverseRotate(value));
            }
        };
        class ContactCounter final : public JPH::ContactListener
        {
        public:
            unsigned mCount = 0;
            void OnContactAdded(const JPH::Body& first, const JPH::Body& second,
                const JPH::ContactManifold& manifold, JPH::ContactSettings&) override
            {
                ++mCount;
                std::cout << "FLOATING_INITIAL_CONTACT first=" << first.GetID().GetIndex()
                          << " second=" << second.GetID().GetIndex()
                          << " depth=" << manifold.mPenetrationDepth << std::endl;
            }
            void OnContactPersisted(const JPH::Body&, const JPH::Body&,
                const JPH::ContactManifold&, JPH::ContactSettings&) override { ++mCount; }
        };
        struct RestoreListener
        {
            JPH::PhysicsSystem& mPhysics;
            JPH::ContactListener* mPrevious;
            ~RestoreListener() { mPhysics.SetContactListener(mPrevious); }
        };

        for (const std::size_t driven : { std::size_t(18), std::size_t(17) })
        {
            std::array<float, 2> maximumAngularErrors{};
            for (const bool withLoad : { false, true })
            {
                SCOPED_TRACE(driven);
                SCOPED_TRACE(withLoad);
                ASSERT_EQ(mPhysics.GetNumBodies(), 0u);
                BuildOptions options;
                options.mObjectLayer = sObjectLayer;
                options.mUseContinuousCollision = false;
                options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3(1, 2, 3).Normalized(), 0.7f);
                auto created = createPassiveRagdoll(mPhysics, skeleton, profile, options, {}, 7, 1234);
                ASSERT_TRUE(created.isValid());
                auto& ragdoll = *created.mRagdoll;
                ASSERT_TRUE(ragdoll.addToPhysicsSystem());
                ASSERT_EQ(ragdoll.getBodyCount(), count);
                ASSERT_EQ(mPhysics.GetNumBodies(), count);
                mPhysics.SetGravity(JPH::Vec3::sZero());
                auto& bodies = mPhysics.GetBodyInterface();
                ContactCounter contacts;
                RestoreListener restore{ mPhysics, mPhysics.GetContactListener() };
                mPhysics.SetContactListener(&contacts);

                std::array<DemandBody, count> demand;
                std::array<JPH::TwoBodyConstraint*, count> joints{};
                std::array<JPH::RVec3, count> anchors;
                std::array<int, count> parents;
                std::array<JPH::Vec3, count> relativeTargets;
                std::array<JPH::Vec3, count> hingeAxes;
                relativeTargets.fill(JPH::Vec3::sZero());
                hingeAxes.fill(JPH::Vec3::sZero());
                float totalMass = 0;
                JPH::RVec3 center = JPH::RVec3::sZero();
                for (std::size_t i = 0; i < count; ++i)
                {
                    parents[i] = skeleton.mBones[i].mParent;
                    ASSERT_LT(parents[i], static_cast<int>(i));
                    ASSERT_GE(parents[i], i == 0 ? -1 : 0);
                    JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(i));
                    ASSERT_TRUE(lock.Succeeded());
                    const auto& body = lock.GetBody();
                    ASSERT_TRUE(body.IsDynamic());
                    ASSERT_TRUE(body.IsActive());
                    ASSERT_FALSE(body.GetApplyGyroscopicForce());
                    ASSERT_TRUE(body.GetLinearVelocity().IsNearZero());
                    ASSERT_TRUE(body.GetAngularVelocity().IsNearZero());
                    const auto& properties = *body.GetMotionProperties();
                    ASSERT_EQ(properties.GetAllowedDOFs(), JPH::EAllowedDOFs::All);
                    const auto inverseInertia = properties.GetInverseInertiaDiagonal();
                    ASSERT_GT(inverseInertia.GetX(), 0);
                    ASSERT_GT(inverseInertia.GetY(), 0);
                    ASSERT_GT(inverseInertia.GetZ(), 0);
                    auto& current = demand[i];
                    current.mCenter = body.GetCenterOfMassPosition();
                    current.mMass = 1.0f / properties.GetInverseMass();
                    current.mInertiaFrame = body.GetRotation() * properties.GetInertiaRotation();
                    current.mInertiaDiagonal = JPH::Vec3::sOne() / inverseInertia;
                    EXPECT_NEAR(current.mMass, 75.0f * profile.mBodies[i].mMassFraction, 1e-5f);
                    totalMass += current.mMass;
                    center += current.mCenter * current.mMass;
                }
                EXPECT_NEAR(totalMass, 75.0f, 1e-4f);
                center /= totalMass;
                for (const auto& entry : mPhysics.GetConstraints())
                {
                    auto* joint = static_cast<JPH::TwoBodyConstraint*>(entry.GetPtr());
                    for (std::size_t i = 1; i < count; ++i)
                        if (joint->GetBody2()->GetID() == ragdoll.getBodyId(i))
                            joints[i] = joint;
                }
                ASSERT_NE(joints[18], nullptr);
                ASSERT_EQ(joints[18]->GetSubType(), JPH::EConstraintSubType::Hinge);
                const auto* knee = static_cast<JPH::HingeConstraint*>(joints[18]);
                const auto axis = knee->GetBody1()->GetRotation() * knee->GetLocalSpaceHingeAxis1();
                relativeTargets[driven] = axis * requestedSpeed;

                JPH::Vec3 weightedShapeLinear = JPH::Vec3::sZero();
                for (std::size_t i = 1; i < count; ++i)
                {
                    ASSERT_NE(joints[i], nullptr);
                    const auto parent = static_cast<std::size_t>(parents[i]);
                    const auto& joint = *joints[i];
                    ASSERT_EQ(joint.GetBody1()->GetID(), ragdoll.getBodyId(parent));
                    const auto parentFrame = joint.GetBody1()->GetCenterOfMassTransform()
                        * joint.GetConstraintToBody1Matrix();
                    const auto childFrame = joint.GetBody2()->GetCenterOfMassTransform()
                        * joint.GetConstraintToBody2Matrix();
                    anchors[i] = childFrame.GetTranslation();
                    // A nonzero gap invalidates the rigid-twist nullspace used
                    // in this composite-inertia solve. Never average it away.
                    ASSERT_LT((anchors[i] - parentFrame.GetTranslation()).Length(), 1e-4);
                    auto& current = demand[i];
                    current.mShapeAngular = demand[parent].mShapeAngular + relativeTargets[i];
                    current.mShapeLinear = demand[parent].mShapeLinear
                        + demand[parent].mShapeAngular.Cross(JPH::Vec3(anchors[i] - demand[parent].mCenter))
                        - current.mShapeAngular.Cross(JPH::Vec3(anchors[i] - current.mCenter));
                    weightedShapeLinear += current.mMass * current.mShapeLinear;
                    if (joint.GetSubType() == JPH::EConstraintSubType::Hinge)
                    {
                        const auto& hinge = static_cast<const JPH::HingeConstraint&>(joint);
                        hingeAxes[i] = joint.GetBody1()->GetRotation() * hinge.GetLocalSpaceHingeAxis1();
                        ASSERT_LT(relativeTargets[i].Cross(hingeAxes[i]).Length(), 1e-6f);
                    }
                    else
                        ASSERT_EQ(joint.GetSubType(), JPH::EConstraintSubType::SwingTwist);
                }
                const auto meanShapeLinear = weightedShapeLinear / totalMass;
                JPH::Vec3 shapeMomentum = JPH::Vec3::sZero();
                std::array<JPH::Vec3, 3> inertiaColumns;
                inertiaColumns.fill(JPH::Vec3::sZero());
                const std::array axes{ JPH::Vec3::sAxisX(), JPH::Vec3::sAxisY(), JPH::Vec3::sAxisZ() };
                for (const auto& body : demand)
                {
                    const JPH::Vec3 radius(body.mCenter - center);
                    shapeMomentum += body.inertiaTimes(body.mShapeAngular)
                        + radius.Cross(body.mMass * (body.mShapeLinear - meanShapeLinear));
                    for (std::size_t column = 0; column < 3; ++column)
                        inertiaColumns[column] += body.inertiaTimes(axes[column])
                            + body.mMass * (radius.LengthSq() * axes[column] - radius.Dot(axes[column]) * radius);
                }
                const auto components = [](JPH::Vec3Arg value) {
                    return std::array<double, 3>{ value.GetX(), value.GetY(), value.GetZ() };
                };
                std::array<std::array<double, 3>, 3> inertia{};
                for (std::size_t column = 0; column < 3; ++column)
                {
                    const auto values = components(inertiaColumns[column]);
                    for (std::size_t row = 0; row < 3; ++row)
                        inertia[row][column] = values[row];
                }
                // The only solve is the positive-definite 3x3 locked inertia.
                // Compute in double; body properties and motor inputs are float.
                std::array<std::array<double, 3>, 3> lower{};
                for (std::size_t row = 0; row < 3; ++row)
                    for (std::size_t column = 0; column <= row; ++column)
                    {
                        EXPECT_NEAR(inertia[row][column], inertia[column][row], 0.02);
                        double value = inertia[row][column];
                        for (std::size_t k = 0; k < column; ++k)
                            value -= lower[row][k] * lower[column][k];
                        if (row == column)
                        {
                            ASSERT_GT(value, 0);
                            ASSERT_TRUE(std::isfinite(value));
                            lower[row][column] = std::sqrt(value);
                        }
                        else
                            lower[row][column] = value / lower[column][column];
                    }
                auto solution = components(-shapeMomentum);
                for (std::size_t row = 0; row < 3; ++row)
                {
                    for (std::size_t k = 0; k < row; ++k)
                        solution[row] -= lower[row][k] * solution[k];
                    solution[row] /= lower[row][row];
                }
                for (int row = 2; row >= 0; --row)
                {
                    for (int k = row + 1; k < 3; ++k)
                        solution[row] -= lower[k][row] * solution[k];
                    solution[row] /= lower[row][row];
                }
                const JPH::Vec3 rootAngular(static_cast<float>(solution[0]),
                    static_cast<float>(solution[1]), static_cast<float>(solution[2]));
                JPH::Vec3 rootLinearImpulse = JPH::Vec3::sZero();
                JPH::Vec3 rootAngularImpulse = JPH::Vec3::sZero();
                for (auto& body : demand)
                {
                    const JPH::Vec3 radius(body.mCenter - center);
                    body.mDesiredAngular = rootAngular + body.mShapeAngular;
                    body.mDesiredLinear = rootAngular.Cross(radius) + body.mShapeLinear - meanShapeLinear;
                    body.mLinearImpulse = body.mMass * body.mDesiredLinear;
                    body.mAngularImpulse = body.inertiaTimes(body.mDesiredAngular);
                    rootLinearImpulse += body.mLinearImpulse;
                    rootAngularImpulse += body.mAngularImpulse + radius.Cross(body.mLinearImpulse);
                }
                ASSERT_GT(demand[0].mDesiredAngular.Length(), 1e-5f);
                ASSERT_GT(demand[0].mDesiredLinear.Length(), 1e-5f);
                EXPECT_LT(rootLinearImpulse.Length(), 1e-3f);
                EXPECT_LT(rootAngularImpulse.Length(), 1e-2f);

                std::array<JPH::Vec3, count> jointLinearImpulses;
                std::array<JPH::Vec3, count> jointAngularImpulses;
                jointLinearImpulses.fill(JPH::Vec3::sZero());
                jointAngularImpulses.fill(JPH::Vec3::sZero());
                for (std::size_t i = 1; i < count; ++i)
                {
                    const auto parent = static_cast<std::size_t>(parents[i]);
                    EXPECT_LT((demand[i].mDesiredAngular - demand[parent].mDesiredAngular
                        - relativeTargets[i]).Length(), 1e-6f);
                    const auto parentPointVelocity = demand[parent].mDesiredLinear
                        + demand[parent].mDesiredAngular.Cross(JPH::Vec3(anchors[i] - demand[parent].mCenter));
                    const auto childPointVelocity = demand[i].mDesiredLinear
                        + demand[i].mDesiredAngular.Cross(JPH::Vec3(anchors[i] - demand[i].mCenter));
                    EXPECT_LT((parentPointVelocity - childPointVelocity).Length(), 1e-4f);
                    for (std::size_t descendant = i; descendant < count; ++descendant)
                    {
                        int ancestor = static_cast<int>(descendant);
                        while (ancestor > static_cast<int>(i))
                            ancestor = parents[ancestor];
                        if (ancestor != static_cast<int>(i))
                            continue;
                        const auto& body = demand[descendant];
                        jointLinearImpulses[i] += body.mLinearImpulse;
                        jointAngularImpulses[i] += body.mAngularImpulse
                            + JPH::Vec3(body.mCenter - anchors[i]).Cross(body.mLinearImpulse);
                    }
                }
                // Reconstruct the root's required external reaction separately
                // from the per-body total momentum check above. Child subtree
                // impulses act with the opposite sign on their parent.
                auto requiredRootLinear = demand[0].mLinearImpulse;
                auto requiredRootAngular = demand[0].mAngularImpulse;
                for (std::size_t i = 1; i < count; ++i)
                    if (parents[i] == 0)
                    {
                        requiredRootLinear += jointLinearImpulses[i];
                        requiredRootAngular += jointAngularImpulses[i]
                            + JPH::Vec3(anchors[i] - demand[0].mCenter).Cross(jointLinearImpulses[i]);
                    }
                EXPECT_LT(requiredRootLinear.Length(), 1e-3f);
                EXPECT_LT(requiredRootAngular.Length(), 1e-2f);
                RagdollPose currentPose;
                ASSERT_TRUE(ragdoll.captureModelPose(currentPose));
                ragdoll.beginActuationStep(false);
                ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(currentPose));
                for (std::size_t i = 1; i < count; ++i)
                {
                    ASSERT_TRUE(ragdoll.setMotorTargetAngularVelocity(i, relativeTargets[i]));
                    ASSERT_TRUE(ragdoll.driveMotorWithLoadTorque(i,
                        withLoad ? jointAngularImpulses[i] / dt : JPH::Vec3::sZero(), dt));
                    MotorTargetTelemetry target;
                    ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(i, target));
                    EXPECT_FALSE(target.mWorldOrientation);
                    if (target.mRequestedOrientation && target.mTargetOrientation)
                    {
                        const auto delta = (*target.mRequestedOrientation
                            * target.mTargetOrientation->Conjugated()).Normalized();
                        EXPECT_LT(2 * std::atan2(delta.GetXYZ().Length(), std::abs(delta.GetW())), 1e-5f);
                    }
                    else
                        EXPECT_LT(target.mTargetClampDelta, 1e-5f);
                    EXPECT_LT((target.mTargetAngularVelocity - relativeTargets[i]).Length(), 1e-5f);
                }
                ASSERT_TRUE(ragdoll.beginSolverImpulseMeasurement(dt));
                JPH::TempAllocatorImpl allocator(1024 * 1024);
                JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
                ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
                EXPECT_EQ(contacts.mCount, 0u);
                RagdollSolverImpulse solverImpulse;
                ASSERT_TRUE(ragdoll.endSolverImpulseMeasurement(dt, solverImpulse));
                EXPECT_FALSE(solverImpulse.mVelocityLimitTouched);
                EXPECT_FALSE(solverImpulse.mActivationChanged);
                EXPECT_FALSE(solverImpulse.mContinuousCollisionEnabled);
                EXPECT_LT(solverImpulse.mLinearImpulse.Length(), 1e-2f);
                EXPECT_LT(solverImpulse.mAngularImpulse.Length(), 1e-2f);
                JPH::Vec3 measuredLinearMomentum = JPH::Vec3::sZero();
                JPH::Vec3 measuredAngularMomentum = JPH::Vec3::sZero();
                float maximumLinearError = 0;
                float maximumMotorImpulseError = 0;
                float maximumPointImpulseError = 0;
                float maximumUtilization = 0;
                for (std::size_t i = 0; i < count; ++i)
                {
                    const auto actualLinear = bodies.GetLinearVelocity(ragdoll.getBodyId(i));
                    const auto actualAngular = bodies.GetAngularVelocity(ragdoll.getBodyId(i));
                    const float angularError = (actualAngular - demand[i].mDesiredAngular).Length();
                    const float linearError = (actualLinear - demand[i].mDesiredLinear).Length();
                    maximumAngularErrors[withLoad] = std::max(maximumAngularErrors[withLoad], angularError);
                    maximumLinearError = std::max(maximumLinearError, linearError);
                    const auto linearMomentum = demand[i].mMass * actualLinear;
                    measuredLinearMomentum += linearMomentum;
                    measuredAngularMomentum += demand[i].inertiaTimes(actualAngular)
                        + JPH::Vec3(demand[i].mCenter - center).Cross(linearMomentum);
                    if (withLoad)
                    {
                        EXPECT_LT(angularError, angularTolerance) << "body " << i;
                        EXPECT_LT(linearError, linearTolerance) << "body " << i;
                    }
                    if (i == 0)
                        continue;
                    JPH::Vec3 actualMotorImpulse;
                    float utilization;
                    ASSERT_TRUE(ragdoll.getMotorAngularImpulse(i, dt, actualMotorImpulse, utilization));
                    maximumUtilization = std::max(maximumUtilization, utilization);
                    EXPECT_LT(utilization, 1);
                    auto expectedMotorImpulse = jointAngularImpulses[i];
                    JPH::Vec3 actualPointImpulse;
                    if (joints[i]->GetSubType() == JPH::EConstraintSubType::Hinge)
                    {
                        const auto* hinge = static_cast<JPH::HingeConstraint*>(joints[i]);
                        // Motor demand is only the permitted component. Hard
                        // hinge alignment supplies the perpendicular moment.
                        // Use the pre-solve axis, not the integrated orientation.
                        const auto hingeAxis = hingeAxes[i];
                        expectedMotorImpulse = hingeAxis * hingeAxis.Dot(expectedMotorImpulse);
                        actualPointImpulse = hinge->GetTotalLambdaPosition();
                        EXPECT_EQ(hinge->GetTotalLambdaRotationLimits(), 0);
                    }
                    else
                    {
                        const auto* swing = static_cast<JPH::SwingTwistConstraint*>(joints[i]);
                        actualPointImpulse = swing->GetTotalLambdaPosition();
                        EXPECT_EQ(swing->GetTotalLambdaTwist(), 0);
                        EXPECT_EQ(swing->GetTotalLambdaSwingY(), 0);
                        EXPECT_EQ(swing->GetTotalLambdaSwingZ(), 0);
                    }
                    maximumMotorImpulseError = std::max(maximumMotorImpulseError,
                        (actualMotorImpulse - expectedMotorImpulse).Length()
                            / std::max(1.0f, expectedMotorImpulse.Length()));
                    maximumPointImpulseError = std::max(maximumPointImpulseError,
                        (actualPointImpulse - jointLinearImpulses[i]).Length()
                            / std::max(1.0f, jointLinearImpulses[i].Length()));
                }
                // Frozen geometry distinguishes internal solve impulses from
                // changes caused by rotating anisotropic inertia during Update.
                EXPECT_LT(measuredLinearMomentum.Length(), 1e-2f);
                EXPECT_LT(measuredAngularMomentum.Length(), 1e-2f);
                std::cout << "FLOATING_HUMANOID_ONE_STEP driven=" << driven << " load=" << withLoad
                          << " max_angular_error=" << maximumAngularErrors[withLoad]
                          << " max_linear_error=" << maximumLinearError
                          << " max_motor_impulse_relative_error=" << maximumMotorImpulseError
                          << " max_point_impulse_relative_error=" << maximumPointImpulseError
                          << " max_utilization=" << maximumUtilization
                          << " root_angular_target=" << demand[0].mDesiredAngular.Length()
                          << " root_linear_target=" << demand[0].mDesiredLinear.Length()
                          << " computed_root_linear_impulse=" << rootLinearImpulse.Length()
                          << " computed_root_angular_impulse=" << rootAngularImpulse.Length()
                          << " measured_linear_momentum=" << measuredLinearMomentum.Length()
                          << " measured_frozen_angular_momentum=" << measuredAngularMomentum.Length()
                          << " contacts=" << contacts.mCount << std::endl;
            }
            EXPECT_GT(maximumAngularErrors[0], angularTolerance);
            EXPECT_LT(maximumAngularErrors[1], angularTolerance);
        }
    }

    TEST_F(PassiveRagdollTest, nativeMotorLoadCancelsKnownTorqueWithoutPoseOffset)
    {
        for (const float load : { 50.0f, 2000.0f })
        for (const bool biased : { false, true })
        {
            SCOPED_TRACE(biased);
            SCOPED_TRACE(load);
            auto profile = RagdollCoreTest::makeProfile();
            for (auto& body : profile.mBodies)
            {
                body.mMaterial.mLinearDamping = 0;
                body.mMaterial.mAngularDamping = 0;
            }
            auto& jointProfile = *profile.mBodies[1].mConstraint;
            jointProfile.mType = RagdollCore::ConstraintType::Hinge;
            jointProfile.mDeriveFrameFromSkeleton = true;
            jointProfile.mMotor.mFrequency = 6;
            jointProfile.mMotor.mDamping = 1;
            jointProfile.mMotor.mMaximumTorque = 1000;
            RagdollCore::BuildOptions options;
            options.mObjectLayer = sObjectLayer;
            options.mUseContinuousCollision = false;
            auto created = RagdollCore::createPassiveRagdoll(
                mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
            ASSERT_TRUE(created.isValid());
            auto& ragdoll = *created.mRagdoll;
            ASSERT_TRUE(ragdoll.addToPhysicsSystem());
            auto& bodies = mPhysics.GetBodyInterface();
            bodies.SetMotionType(ragdoll.getBodyId(0), JPH::EMotionType::Static, JPH::EActivation::DontActivate);
            mPhysics.SetGravity(JPH::Vec3::sZero());
            auto* joint = static_cast<JPH::HingeConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
            const auto axis = joint->GetBody1()->GetRotation() * joint->GetLocalSpaceHingeAxis1();
            RagdollCore::RagdollPose pose;
            ASSERT_TRUE(ragdoll.captureModelPose(pose));
            ragdoll.beginActuationStep(false);
            ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
            constexpr float dt = 1.0f / 60.0f;
            if (biased)
            {
                EXPECT_FALSE(ragdoll.driveMotorWithLoadTorque(0, axis * load, dt));
                EXPECT_FALSE(ragdoll.driveMotorWithLoadTorque(1, axis * load, 0));
                joint->SetEnabled(false);
                EXPECT_FALSE(ragdoll.driveMotorWithLoadTorque(1, axis * load, dt));
                joint->SetEnabled(true);
                joint->GetMotorSettings().mSpringSettings.mFrequency = 0;
                EXPECT_FALSE(ragdoll.driveMotorWithLoadTorque(1, axis * load, dt));
                joint->GetMotorSettings().mSpringSettings.mFrequency = jointProfile.mMotor.mFrequency;
                ASSERT_TRUE(ragdoll.driveMotorWithLoadTorque(1, axis * load, dt));
                EXPECT_FALSE(ragdoll.driveMotorWithLoadTorque(1, axis * load, dt));
            }
            bodies.AddTorque(ragdoll.getBodyId(1), -axis * load, JPH::EActivation::Activate);
            JPH::TempAllocatorImpl allocator(1024 * 1024);
            JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
            ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
            JPH::Vec3 impulse;
            float utilization = 0;
            ASSERT_TRUE(ragdoll.getMotorAngularImpulse(1, dt, impulse, utilization));
            EXPECT_LE(utilization, 1);
            if (biased)
                EXPECT_FALSE(ragdoll.getMotorAngularImpulse(1, dt * 2, impulse, utilization));
            if (biased)
            {
                EXPECT_NEAR(impulse.Dot(axis), std::min(load, jointProfile.mMotor.mMaximumTorque) * dt, 1e-4f);
                if (load < jointProfile.mMotor.mMaximumTorque)
                {
                    EXPECT_LT(bodies.GetAngularVelocity(ragdoll.getBodyId(1)).Length(), 1e-4f);
                    EXPECT_NEAR(joint->GetCurrentAngle(), joint->GetTargetAngle(), 1e-4f);
                }
                else
                    EXPECT_GT(bodies.GetAngularVelocity(ragdoll.getBodyId(1)).Length(), 1e-4f);
            }
            else
                EXPECT_LT(impulse.Dot(axis), 0.99f * load * dt);
            ragdoll.disableMotors();
            ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
            ASSERT_TRUE(ragdoll.getMotorAngularImpulse(1, dt, impulse, utilization));
            EXPECT_TRUE(impulse.IsNearZero());
        }
    }

    TEST_F(PassiveRagdollTest, nativeMotorAccelerationMatchesImplicitSpringAndLoad)
    {
        // Independent implicit-Euler prediction, with no external opposing load:
        // I*w/dt = load + k*(target - dt*w) - c*w.
        // This is an acceleration test, not static load compensation. The
        // spherical child and COM anchor exclude changing axes/anchor coupling.
        for (const float dt : { 1.0f / 30, 1.0f / 60, 1.0f / 120 })
        for (const float frequency : { 1.7418f, 6.0f })
        for (const float target : { -0.05f, 0.0f, 0.05f })
        for (const float load : { -50.0f, 50.0f })
        {
            SCOPED_TRACE(dt);
            SCOPED_TRACE(frequency);
            SCOPED_TRACE(target);
            SCOPED_TRACE(load);
            auto profile = RagdollCoreTest::makeProfile();
            for (auto& body : profile.mBodies)
            {
                body.mMaterial.mLinearDamping = 0;
                body.mMaterial.mAngularDamping = 0;
            }
            profile.mBodies[1].mShape.mType = RagdollCore::ShapeType::Sphere;
            profile.mBodies[1].mShape.mRadius = 1;
            auto& jointProfile = *profile.mBodies[1].mConstraint;
            jointProfile.mType = RagdollCore::ConstraintType::Hinge;
            jointProfile.mDeriveFrameFromSkeleton = true;
            jointProfile.mMotor.mFrequency = frequency;
            jointProfile.mMotor.mDamping = 1;
            jointProfile.mMotor.mMaximumTorque = 1000;
            RagdollCore::BuildOptions options;
            options.mObjectLayer = sObjectLayer;
            options.mUseContinuousCollision = false;
            options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3(1, 2, 3).Normalized(), 0.7f);
            auto created = RagdollCore::createPassiveRagdoll(
                mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
            ASSERT_TRUE(created.isValid());
            auto& ragdoll = *created.mRagdoll;
            ASSERT_TRUE(ragdoll.addToPhysicsSystem());
            auto& bodies = mPhysics.GetBodyInterface();
            bodies.SetMotionType(ragdoll.getBodyId(0), JPH::EMotionType::Static, JPH::EActivation::DontActivate);
            mPhysics.SetGravity(JPH::Vec3::sZero());
            auto* joint = static_cast<JPH::HingeConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
            ASSERT_TRUE(joint->GetLocalSpacePoint2().IsNearZero());
            const auto axis = joint->GetBody1()->GetRotation() * joint->GetLocalSpaceHingeAxis1();
            const auto* child = joint->GetBody2();
            const float inverseInertia = axis.Dot(child->GetMotionProperties()->MultiplyWorldSpaceInverseInertiaByVector(
                child->GetRotation(), axis));
            ASSERT_GT(inverseInertia, 0);
            const float inertia = 1 / inverseInertia;
            const float omega = 2 * JPH::JPH_PI * frequency;
            const float stiffness = inertia * omega * omega;
            const float damping = 2 * inertia * omega;
            const float expectedVelocity = dt * (load + stiffness * target)
                / (inertia + dt * damping + dt * dt * stiffness);
            const float expectedImpulse = inertia * expectedVelocity;
            ASSERT_LT(std::abs(expectedImpulse), dt * jointProfile.mMotor.mMaximumTorque);
            RagdollCore::RagdollPose pose;
            ASSERT_TRUE(ragdoll.captureModelPose(pose));
            ragdoll.beginActuationStep(false);
            ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
            ASSERT_NEAR(joint->GetCurrentAngle(), 0, 1e-6f);
            joint->SetTargetAngle(target);
            ASSERT_TRUE(ragdoll.driveMotorWithLoadTorque(1, axis * load, dt));
            JPH::TempAllocatorImpl allocator(1024 * 1024);
            JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
            ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
            JPH::Vec3 impulse;
            float utilization;
            ASSERT_TRUE(ragdoll.getMotorAngularImpulse(1, dt, impulse, utilization));
            EXPECT_NEAR(impulse.Dot(axis), expectedImpulse, 1e-4f);
            EXPECT_NEAR(bodies.GetAngularVelocity(ragdoll.getBodyId(1)).Dot(axis), expectedVelocity, 1e-3f);
            EXPECT_LT(utilization, 1);
        }
    }

    TEST_F(PassiveRagdollTest, jointLoadAndPoseShareTheMotorBudgetAndHaveNoExternalImpulse)
    {
        for (const auto type : { RagdollCore::ConstraintType::SwingTwist, RagdollCore::ConstraintType::Hinge })
        {
            SCOPED_TRACE(type == RagdollCore::ConstraintType::Hinge ? "hinge" : "swing_twist");
            auto profile = RagdollCoreTest::makeProfile();
            auto& joint = *profile.mBodies[1].mConstraint;
            joint.mType = type;
            joint.mDeriveFrameFromSkeleton = true;
            joint.mMotor.mFrequency = 6.0f;
            joint.mMotor.mDamping = 1.0f;
            joint.mMotor.mMaximumTorque = 1000.0f;
            RagdollCore::BuildOptions options;
            options.mObjectLayer = sObjectLayer;
            options.mUseContinuousCollision = false;
            options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3(1, 2, 3).Normalized(), 0.7f);
            auto created = RagdollCore::createPassiveRagdoll(
                mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
            ASSERT_TRUE(created.isValid());
            auto& ragdoll = *created.mRagdoll;
            ASSERT_TRUE(ragdoll.addToPhysicsSystem());
            mPhysics.SetGravity(JPH::Vec3::sZero());
            // Keep the native joint enabled: its position motor and load share
            // one solved budget. Whole-body momentum checks below subtract
            // internal anchor reactions and must still find no external wrench.
            auto* constraint = static_cast<JPH::TwoBodyConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
            if (type == RagdollCore::ConstraintType::SwingTwist)
            {
                auto& twist = static_cast<JPH::SwingTwistConstraint*>(constraint)->GetTwistMotorSettings();
                twist.mMaxTorqueLimit = 125.0f;
                twist.mMinTorqueLimit = -250.0f;
            }
            JPH::TempAllocatorImpl allocator(1024 * 1024);
            JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
            auto& bodies = mPhysics.GetBodyInterface();
            constexpr float dt = 1.0f / 60.0f;
            for (int step = 0; step < 3; ++step)
            {
                SCOPED_TRACE(step);
                ragdoll.beginActuationStep(true);
                for (std::size_t i = 0; i < 2; ++i)
                    bodies.SetLinearAndAngularVelocity(ragdoll.getBodyId(i), JPH::Vec3::sZero(), JPH::Vec3::sZero());
                RagdollCore::RagdollPose pose;
                ASSERT_TRUE(ragdoll.captureModelPose(pose));
                pose.mBodyTransforms[1].mRotation = pose.mBodyTransforms[1].mRotation
                    * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.1f);
                ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
                const JPH::Vec3 load(1e6f, -2e6f, 3e6f);
                JPH::Vec3 expected = JPH::Vec3::sZero();
                if (type == RagdollCore::ConstraintType::Hinge)
                {
                    const auto* hinge = static_cast<const JPH::HingeConstraint*>(constraint);
                    const auto axis = constraint->GetBody1()->GetRotation() * hinge->GetLocalSpaceHingeAxis1();
                    expected = axis * std::copysign(joint.mMotor.mMaximumTorque, load.Dot(axis));
                }
                else
                {
                    const auto* swing = static_cast<const JPH::SwingTwistConstraint*>(constraint);
                    const auto frame = constraint->GetBody2()->GetRotation() * swing->GetConstraintToBody2();
                    for (std::size_t i = 0; i < 3; ++i)
                    {
                        const auto axis = frame
                            * (i == 0 ? JPH::Vec3::sAxisX() : i == 1 ? JPH::Vec3::sAxisY() : JPH::Vec3::sAxisZ());
                        const auto& motor = i == 0 ? swing->GetTwistMotorSettings() : swing->GetSwingMotorSettings();
                        expected += axis * (load.Dot(axis) >= 0 ? motor.mMaxTorqueLimit : motor.mMinTorqueLimit);
                    }
                }
                if (step != 1)
                {
                    EXPECT_FALSE(ragdoll.driveMotorWithLoadTorque(0, load, dt));
                    EXPECT_FALSE(ragdoll.driveMotorWithLoadTorque(1, load, 0));
                    EXPECT_FALSE(ragdoll.driveMotorWithLoadTorque(
                        1, JPH::Vec3(std::numeric_limits<float>::quiet_NaN(), 0, 0), dt));
                    ASSERT_TRUE(ragdoll.driveMotorWithLoadTorque(1, load, dt));
                    EXPECT_FALSE(ragdoll.driveMotorWithLoadTorque(1, load, dt)) << "one combined budget per step";
                    RagdollCore::MotorTargetTelemetry target;
                    ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, target));
                    EXPECT_FALSE(target.mWorldOrientation);
                }
                if (step != 0)
                {
                    // Clearing a step and disabling after queuing must both
                    // remove force-motor warm starts and target telemetry.
                    ragdoll.disableMotors();
                    expected = JPH::Vec3::sZero();
                    RagdollCore::MotorTargetTelemetry target;
                    EXPECT_FALSE(ragdoll.captureMotorTargetTelemetry(1, target));
                }
                for (std::size_t i = 0; i < 2; ++i)
                {
                    JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(i));
                    ASSERT_TRUE(lock.Succeeded());
                    const auto& body = lock.GetBody();
                    EXPECT_TRUE(body.GetAccumulatedTorque().IsNearZero());
                }
                ASSERT_TRUE(ragdoll.beginSolverImpulseMeasurement(dt));
                ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
                RagdollCore::RagdollSolverImpulse observed;
                ASSERT_TRUE(ragdoll.endSolverImpulseMeasurement(dt, observed));
                EXPECT_FALSE(observed.mFreeVelocityLimitTouched);
                EXPECT_FALSE(observed.mVelocityLimitTouched);
                EXPECT_LT(observed.mLinearImpulse.Length(), 1e-3f);
                EXPECT_LT(observed.mAngularImpulse.Length(), 1e-2f);
                JPH::Vec3 impulse;
                float utilization = 0;
                ASSERT_TRUE(ragdoll.getMotorAngularImpulse(1, dt, impulse, utilization));
                EXPECT_LT((impulse - expected * dt).Length(), 1e-4f);
                EXPECT_FLOAT_EQ(utilization, step == 0 ? 1.0f : 0.0f);
            }
            ragdoll.removeFromPhysicsSystem();
            EXPECT_TRUE(mPhysics.GetConstraints().empty());
        }
    }

    TEST_F(PassiveRagdollTest, jointLoadSurvivesPoseAxisReleaseWithoutAddingBraking)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& motor = profile.mBodies[1].mConstraint->mMotor;
        motor.mFrequency = 6;
        motor.mDamping = 1;
        motor.mMaximumTorque = 100000;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mUseContinuousCollision = false;
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        mPhysics.GetConstraints()[0]->SetEnabled(false);
        mPhysics.SetGravity(JPH::Vec3::sZero());
        auto& bodies = mPhysics.GetBodyInterface();
        const auto freeAxis = JPH::Vec3(1, 2, 3).Normalized();
        const auto controlledAxis = freeAxis.GetNormalizedPerpendicular();
        const auto load = freeAxis * 20.0f;
        RagdollCore::RagdollPose pose;
        ASSERT_TRUE(ragdoll.captureModelPose(pose));
        pose.mBodyTransforms[1].mRotation = JPH::Quat::sRotation(freeAxis, 0.1f);
        constexpr float dt = 1.0f / 60.0f;
        std::array<JPH::Vec3, 3> torques;
        for (std::size_t variant = 0; variant < torques.size(); ++variant)
        {
            SCOPED_TRACE(variant);
            ragdoll.beginActuationStep(true);
            bodies.SetAngularVelocity(ragdoll.getBodyId(1),
                0.02f * controlledAxis + (variant == 2 ? 2.0f : 0.0f) * freeAxis);
            ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
            EXPECT_FALSE(ragdoll.driveMotorWithLoadTorque(1, load, dt, freeAxis, JPH::Quat::sIdentity()));
            ASSERT_TRUE(ragdoll.driveMotorWithLoadTorque(
                1, variant == 0 ? JPH::Vec3::sZero() : load, dt, freeAxis));
            EXPECT_FALSE(ragdoll.driveMotorWithFreeRotationAxis(1, freeAxis, dt));
            JPH::Vec3 impulse;
            float utilization = 0;
            ASSERT_TRUE(ragdoll.getMotorAngularImpulse(1, dt, impulse, utilization));
            EXPECT_LT(utilization, 1.0f) << "this case isolates load and braking below saturation";
            torques[variant] = impulse / dt;
            EXPECT_NEAR(torques[variant].Dot(freeAxis), variant == 0 ? 0.0f : 20.0f, 1e-3f);
            EXPECT_LT(torques[variant].Dot(controlledAxis), 0.0f);
            RagdollCore::MotorTargetTelemetry target;
            ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, target));
            EXPECT_FALSE(target.mWorldOrientation);
            for (std::size_t bodyIndex = 0; bodyIndex < 2; ++bodyIndex)
            {
                JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(bodyIndex));
                ASSERT_TRUE(lock.Succeeded());
                EXPECT_TRUE(lock.GetBody().GetAccumulatedTorque().IsNearZero());
            }
        }
        EXPECT_LT((torques[1] - torques[0] - load).Length(), 1e-3f);
        EXPECT_LT((torques[2] - torques[1]).Length(), 1e-3f)
            << "velocity along the released axis must not add pose damping";
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        ASSERT_TRUE(ragdoll.beginSolverImpulseMeasurement(dt));
        ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        RagdollCore::RagdollSolverImpulse observed;
        ASSERT_TRUE(ragdoll.endSolverImpulseMeasurement(dt, observed));
        EXPECT_FALSE(observed.mFreeVelocityLimitTouched);
        EXPECT_FALSE(observed.mVelocityLimitTouched);
        EXPECT_LT(observed.mLinearImpulse.Length(), 1e-3f);
        EXPECT_LT(observed.mAngularImpulse.Length(), 1e-2f);
    }

    TEST_F(PassiveRagdollTest, orientationTelemetryResolvesTinyErrorsAndIgnoresQuaternionNormRoundoff)
    {
        for (const auto root : { JPH::Quat::sIdentity(),
                 JPH::Quat::sRotation(JPH::Vec3(1, 2, 3).Normalized(), 0.7f) })
        {
            auto profile = RagdollCoreTest::makeProfile();
            auto& limits = *profile.mBodies[1].mConstraint;
            limits.mDeriveFrameFromSkeleton = true;
            limits.mPlaneHalfConeAngle = 0.25f;
            limits.mMotor.mMaximumTorque = 1000;
            RagdollCore::BuildOptions options;
            options.mObjectLayer = sObjectLayer;
            options.mRootRotation = root;
            auto created = RagdollCore::createPassiveRagdoll(
                mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
            ASSERT_TRUE(created.isValid());
            auto& ragdoll = *created.mRagdoll;
            ASSERT_TRUE(ragdoll.addToPhysicsSystem());
            RagdollCore::RagdollPose base;
            ASSERT_TRUE(ragdoll.captureModelPose(base));
            auto* joint = static_cast<JPH::SwingTwistConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
            const auto frame1 = joint->GetConstraintToBody1();
            const auto frame2 = joint->GetConstraintToBody2();
            const auto parentWorldFrame = joint->GetBody1()->GetRotation() * frame1;
            // A one-ULP scale change represents normal accumulated quaternion
            // roundoff. It is accepted by the native normalized-quaternion API.
            const float scale = std::nextafter(1.0f, 0.0f);
            for (const bool world : { false, true })
            for (const float sign : { -1.0f, 1.0f })
            for (const float angle : { 0.0f, 1e-4f, 0.2501f, 1.3f })
            {
                SCOPED_TRACE(world);
                SCOPED_TRACE(sign);
                SCOPED_TRACE(angle);
                const auto rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), angle);
                if (world)
                {
                    ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(base));
                    const auto requested = (parentWorldFrame * rotation * frame2.Conjugated()).Normalized()
                        * (sign * scale);
                    ASSERT_TRUE(requested.IsNormalized());
                    ASSERT_TRUE(ragdoll.driveMotorToWorldOrientation(1, requested, 1.0f / 60.0f));
                }
                else
                {
                    auto pose = base;
                    pose.mBodyTransforms[1].mRotation = (pose.mBodyTransforms[0].mRotation
                        * frame1 * rotation * frame2.Conjugated()).Normalized();
                    ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
                    // Keep the actual native target orientation, including its
                    // anatomical clamp; change only its equivalent representation.
                    const auto equivalent = joint->GetTargetOrientationCS() * (sign * scale);
                    ASSERT_TRUE(equivalent.IsNormalized());
                    joint->SetTargetOrientationCS(equivalent);
                }
                RagdollCore::MotorTargetTelemetry telemetry;
                ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, telemetry));
                EXPECT_EQ(telemetry.mWorldOrientation, world);
                const float targetAngle = std::min(angle, limits.mPlaneHalfConeAngle);
                EXPECT_NEAR(telemetry.mAngularError, targetAngle, 2e-6f);
                EXPECT_NEAR(telemetry.mTargetClampDelta, angle - targetAngle, 2e-6f);
                EXPECT_TRUE(std::isfinite(telemetry.mAngularError));
                EXPECT_TRUE(std::isfinite(telemetry.mTargetClampDelta));
                if (angle == 1e-4f)
                    EXPECT_GT(telemetry.mAngularError, 0.5e-4f);
                if (angle == 0.2501f)
                    EXPECT_GT(telemetry.mTargetClampDelta, 0.5e-4f);
            }
        }
    }

    TEST_F(PassiveRagdollTest, worldFootTargetRespectsTheNativeAnatomicalClamp)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& limits = *profile.mBodies[1].mConstraint;
        limits.mDeriveFrameFromSkeleton = true;
        limits.mNormalHalfConeAngle = 0.2f;
        limits.mPlaneHalfConeAngle = 0.25f;
        limits.mTwistMinAngle = -0.1f;
        limits.mTwistMaxAngle = 0.1f;
        limits.mMotor.mFrequency = 4;
        limits.mMotor.mDamping = 1;
        limits.mMotor.mMaximumTorque = 1000;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3(1, 2, 3).Normalized(), 0.7f);
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        RagdollCore::RagdollPose pose;
        ASSERT_TRUE(ragdoll.captureModelPose(pose));
        ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
        const auto* joint = static_cast<const JPH::SwingTwistConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
        const auto parentFrame = joint->GetBody1()->GetRotation() * joint->GetConstraintToBody1();
        const auto requested = (parentFrame * JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 1.3f)
            * joint->GetConstraintToBody2().Conjugated()).Normalized();
        ASSERT_TRUE(ragdoll.driveMotorToWorldOrientation(1, requested, 1.0f / 60.0f));
        RagdollCore::MotorTargetTelemetry target;
        ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, target));
        ASSERT_TRUE(target.mWorldOrientation && target.mRequestedOrientation && target.mTargetOrientation);
        EXPECT_GT(std::abs(target.mRequestedOrientation->Dot(requested)), 0.99999f);
        const auto actualConstraintTarget = (parentFrame.Conjugated() * *target.mTargetOrientation
            * joint->GetConstraintToBody2()).Normalized();
        const auto expected = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), limits.mPlaneHalfConeAngle);
        EXPECT_GT(std::abs(actualConstraintTarget.Dot(expected)), 0.99999f);
        EXPECT_NEAR(target.mTargetClampDelta, 1.3f - limits.mPlaneHalfConeAngle, 1e-4f);
        EXPECT_NEAR(target.mAngularError, limits.mPlaneHalfConeAngle, 1e-4f);
        for (const float maximum : { 0.1f, 0.05f })
        {
            ASSERT_TRUE(ragdoll.limitMotorTargetStep(1, maximum));
            ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, target));
            ASSERT_TRUE(target.mRequestedOrientation);
            ASSERT_TRUE(target.mTargetOrientation);
            EXPECT_GT(std::abs(target.mRequestedOrientation->Dot(requested)), 1.0f - 1e-6f);
            const auto applied = (parentFrame * joint->GetTargetOrientationCS()
                * joint->GetConstraintToBody2().Conjugated()).Normalized();
            EXPECT_GT(std::abs(target.mTargetOrientation->Dot(applied)), 1.0f - 1e-6f);
            EXPECT_NEAR(target.mAngularError, maximum, 1e-4f);
            EXPECT_NEAR(target.mTargetClampDelta, 1.3f - maximum, 1e-4f);
        }
    }

    TEST_F(PassiveRagdollTest, worldMotorSharesLoadBudgetWithoutBrakingItsMovingParent)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto skeleton = RagdollCoreTest::makeSkeleton();
        skeleton.mBones[1].mModelTransform.mTranslation = JPH::Vec3::sZero();
        for (auto& body : profile.mBodies)
        {
            body.mMaterial.mLinearDamping = 0;
            body.mMaterial.mAngularDamping = 0;
        }
        auto& motor = profile.mBodies[1].mConstraint->mMotor;
        motor.mFrequency = 6;
        motor.mDamping = 1;
        motor.mMaximumTorque = 1000;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mUseContinuousCollision = false;
        options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3(1, 2, 3).Normalized(), 0.7f);
        auto created = RagdollCore::createPassiveRagdoll(mPhysics, skeleton, profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        auto* joint = static_cast<JPH::SwingTwistConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
        joint->GetTwistMotorSettings().mMinTorqueLimit = -250;
        joint->GetTwistMotorSettings().mMaxTorqueLimit = 125;
        auto& bodies = mPhysics.GetBodyInterface();
        const auto targetRotation = options.mRootRotation;
        const auto frame = targetRotation * joint->GetConstraintToBody2();
        const std::array<JPH::Vec3, 3> axes{ frame * JPH::Vec3::sAxisX(),
            frame * JPH::Vec3::sAxisY(), frame * JPH::Vec3::sAxisZ() };
        constexpr float dt = 1.0f / 60.0f;
        mPhysics.SetGravity(JPH::Vec3::sZero());
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        for (const float sign : { -1.0f, 1.0f })
        {
            const auto load = JPH::Vec3(1e6f, -2e6f, 3e6f) * sign;
            std::array<JPH::Vec3, 2> torques;
            for (std::size_t variant = 0; variant < torques.size(); ++variant)
            {
                ragdoll.beginActuationStep(false);
                for (std::size_t body = 0; body < 2; ++body)
                {
                    bodies.SetPositionAndRotation(ragdoll.getBodyId(body), options.mRootPosition,
                        options.mRootRotation, JPH::EActivation::Activate);
                    bodies.SetLinearAndAngularVelocity(ragdoll.getBodyId(body), JPH::Vec3::sZero(),
                        body == 0 && variant == 1 ? JPH::Vec3(1, 2, 3) : JPH::Vec3::sZero());
                }
                RagdollCore::RagdollPose pose;
                ASSERT_TRUE(ragdoll.captureModelPose(pose));
                ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
                joint->SetEnabled(false);
                EXPECT_FALSE(ragdoll.driveMotorWithLoadTorque(1, load, dt, std::nullopt, targetRotation));
                joint->SetEnabled(true);
                ASSERT_TRUE(ragdoll.driveMotorWithLoadTorque(1, load, dt, std::nullopt, targetRotation));
                RagdollCore::MotorTargetTelemetry target;
                ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, target));
                EXPECT_TRUE(target.mWorldOrientation);
                for (std::size_t body = 0; body < 2; ++body)
                {
                    JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(body));
                    ASSERT_TRUE(lock.Succeeded());
                    EXPECT_TRUE(lock.GetBody().GetAccumulatedTorque().IsNearZero());
                }
                ASSERT_TRUE(ragdoll.beginSolverImpulseMeasurement(dt));
                ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
                RagdollCore::RagdollSolverImpulse observed;
                ASSERT_TRUE(ragdoll.endSolverImpulseMeasurement(dt, observed));
                EXPECT_FALSE(observed.mFreeVelocityLimitTouched);
                EXPECT_FALSE(observed.mVelocityLimitTouched);
                EXPECT_LT(observed.mLinearImpulse.Length(), 1e-3f);
                EXPECT_LT(observed.mAngularImpulse.Length(), 1e-2f);
                JPH::Vec3 impulse;
                float utilization = 0;
                ASSERT_TRUE(ragdoll.getMotorAngularImpulse(1, dt, impulse, utilization));
                EXPECT_FLOAT_EQ(utilization, 1.0f);
                torques[variant] = impulse / dt;
                for (std::size_t axis = 0; axis < axes.size(); ++axis)
                {
                    const auto& limits = axis == 0 ? joint->GetTwistMotorSettings() : joint->GetSwingMotorSettings();
                    const float component = torques[variant].Dot(axes[axis]);
                    EXPECT_NEAR(component, std::clamp(load.Dot(axes[axis]),
                        limits.mMinTorqueLimit, limits.mMaxTorqueLimit), 1e-3f);
                }
            }
            EXPECT_LT((torques[0] - torques[1]).Length(), 1e-3f);
        }
    }

    TEST_F(PassiveRagdollTest, nativeMotorImpulseUsesTheSolverFrameBeforeBodyRotation)
    {
        for (const auto type : { RagdollCore::ConstraintType::SwingTwist, RagdollCore::ConstraintType::Hinge })
        {
            SCOPED_TRACE(type == RagdollCore::ConstraintType::Hinge ? "hinge" : "swing_twist");
            auto profile = RagdollCoreTest::makeProfile();
            auto& joint = *profile.mBodies[1].mConstraint;
            joint.mType = type;
            joint.mMotor.mFrequency = 6.0f;
            joint.mMotor.mDamping = 1.0f;
            joint.mMotor.mMaximumTorque = 1000.0f;
            RagdollCore::BuildOptions options;
            options.mObjectLayer = sObjectLayer;
            RagdollCore::PassiveRagdollInitialState initial;
            initial.mActorMotion.mAngularVelocity = JPH::Vec3(3, 4, 2);
            auto created = RagdollCore::createPassiveRagdoll(
                mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, initial, 7, 1234);
            ASSERT_TRUE(created.isValid());
            auto& ragdoll = *created.mRagdoll;
            ASSERT_TRUE(ragdoll.addToPhysicsSystem());
            RagdollCore::RagdollPose pose;
            ASSERT_TRUE(ragdoll.captureModelPose(pose));
            const JPH::Vec3 targetAxis = type == RagdollCore::ConstraintType::Hinge
                ? joint.mChildPlaneAxis.Cross(joint.mChildTwistAxis) : JPH::Vec3::sAxisX();
            pose.mBodyTransforms[1].mRotation = JPH::Quat::sRotation(targetAxis, 0.1f);
            ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
            const auto constraints = mPhysics.GetConstraints();
            ASSERT_EQ(constraints.size(), 1u);
            const auto* constraint = static_cast<const JPH::TwoBodyConstraint*>(constraints[0].GetPtr());
            const JPH::Quat parentBefore = constraint->GetBody1()->GetRotation();
            const JPH::Quat childBefore = constraint->GetBody2()->GetRotation();
            JPH::TempAllocatorImpl allocator(1024 * 1024);
            JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
            constexpr float dt = 1.0f / 60.0f;
            ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
            JPH::Vec3 measured, expected, postRotation;
            float utilization = 0.0f;
            ASSERT_TRUE(ragdoll.getMotorAngularImpulse(1, dt, measured, utilization));
            if (type == RagdollCore::ConstraintType::SwingTwist)
            {
                const auto* swing = static_cast<const JPH::SwingTwistConstraint*>(constraint);
                const JPH::Vec3 local = swing->GetConstraintToBody2() * swing->GetTotalLambdaMotor();
                expected = childBefore * local;
                postRotation = constraint->GetBody2()->GetRotation() * local;
            }
            else
            {
                const auto* hinge = static_cast<const JPH::HingeConstraint*>(constraint);
                expected = parentBefore * (hinge->GetLocalSpaceHingeAxis1() * hinge->GetTotalLambdaMotor());
                postRotation = constraint->GetBody2()->GetRotation()
                    * (hinge->GetLocalSpaceHingeAxis2() * hinge->GetTotalLambdaMotor());
            }
            EXPECT_LT((measured - expected).Length(), 1e-4f);
            EXPECT_GT((postRotation - expected).Length(), 1e-3f);
        }
    }

    TEST_F(PassiveRagdollTest, solverImpulseObserverExcludesFreeIntegrationAndInternalMotors)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& motor = profile.mBodies[1].mConstraint->mMotor;
        motor.mFrequency = 6.0f;
        motor.mDamping = 1.0f;
        motor.mMaximumTorque = 1000.0f;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        mPhysics.SetGravity(JPH::Vec3(0, 0, -10));
        auto& bodies = mPhysics.GetBodyInterface();
        RagdollCore::RagdollPose pose;
        ASSERT_TRUE(ragdoll.captureModelPose(pose));
        pose.mBodyTransforms[1].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.1f);
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        constexpr float dt = 1.0f / 60.0f;
        for (int step = 0; step < 20; ++step)
        {
            ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
            bodies.AddForce(ragdoll.getBodyId(0), JPH::Vec3(20, 30, 40));
            bodies.AddTorque(ragdoll.getBodyId(1), JPH::Vec3(2, 4, 6));
            const auto velocity = bodies.GetLinearVelocity(ragdoll.getBodyId(0));
            ASSERT_TRUE(ragdoll.beginSolverImpulseMeasurement(dt));
            EXPECT_EQ(bodies.GetLinearVelocity(ragdoll.getBodyId(0)), velocity);
            ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
            RagdollCore::RagdollSolverImpulse observed;
            EXPECT_FALSE(ragdoll.endSolverImpulseMeasurement(2.0f * dt, observed));
            ASSERT_TRUE(ragdoll.endSolverImpulseMeasurement(dt, observed));
            EXPECT_FALSE(observed.mVelocityLimitTouched);
            EXPECT_FALSE(observed.mActivationChanged);
            EXPECT_TRUE(observed.mContinuousCollisionEnabled);
            EXPECT_LT(observed.mLinearImpulse.Length(), 1e-3f);
            EXPECT_LT(observed.mAngularImpulse.Length(), 1e-2f);
        }
        // Jolt keeps old constraint lambdas while sleeping. They must not be
        // counted as a new external impulse after all bodies deactivate.
        for (std::size_t i = 0; i < ragdoll.getBodyCount(); ++i)
            bodies.DeactivateBody(ragdoll.getBodyId(i));
        ASSERT_TRUE(ragdoll.beginSolverImpulseMeasurement(dt));
        ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        RagdollCore::RagdollSolverImpulse sleeping;
        ASSERT_TRUE(ragdoll.endSolverImpulseMeasurement(dt, sleeping));
        EXPECT_FALSE(sleeping.mActivationChanged);
        EXPECT_EQ(sleeping.mLinearImpulse, JPH::Vec3::sZero());
        EXPECT_EQ(sleeping.mAngularImpulse, JPH::Vec3::sZero());
    }

    TEST(RagdollSolverImpulseTest, planePressurePointIncludesShearAndRejectsInvalidObservations)
    {
        RagdollCore::RagdollSolverImpulse impulse;
        impulse.mReferenceCenterOfMass = JPH::RVec3(100, -200, 80);
        impulse.mLinearImpulse = JPH::Vec3(30, -20, 100);
        const JPH::RVec3 contact(104, -207, 10);
        impulse.mAngularImpulse = JPH::Vec3(contact - impulse.mReferenceCenterOfMass).Cross(impulse.mLinearImpulse);
        // A free yaw moment does not change the horizontal pressure point.
        impulse.mAngularImpulse.SetZ(impulse.mAngularImpulse.GetZ() + 50.0f);
        auto cop = RagdollCore::estimateHorizontalPlaneCenterOfPressure(impulse, 10.0);
        ASSERT_TRUE(cop);
        EXPECT_LT(JPH::Vec3(*cop - contact).Length(), 1e-5f);
        impulse.mVelocityLimitTouched = true;
        EXPECT_FALSE(RagdollCore::estimateHorizontalPlaneCenterOfPressure(impulse, 10.0));
        impulse.mVelocityLimitTouched = false;
        impulse.mActivationChanged = true;
        EXPECT_FALSE(RagdollCore::estimateHorizontalPlaneCenterOfPressure(impulse, 10.0));
        impulse.mActivationChanged = false;
        impulse.mContinuousCollisionEnabled = true;
        EXPECT_FALSE(RagdollCore::estimateHorizontalPlaneCenterOfPressure(impulse, 10.0));
        impulse.mContinuousCollisionEnabled = false;
        impulse.mLinearImpulse.SetZ(0.0f);
        EXPECT_FALSE(RagdollCore::estimateHorizontalPlaneCenterOfPressure(impulse, 10.0));
        impulse.mLinearImpulse.SetZ(-100.0f);
        EXPECT_FALSE(RagdollCore::estimateHorizontalPlaneCenterOfPressure(impulse, 10.0));
        impulse.mLinearImpulse.SetZ(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(RagdollCore::estimateHorizontalPlaneCenterOfPressure(impulse, 10.0));
    }

    TEST_F(PassiveRagdollTest, solverImpulseObserverRecoversGroundSupportImpulse)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies.resize(1);
        profile.mBodies[0].mMassFraction = 1.0f;
        profile.mLandmarks.mSpine.clear();
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mRootPosition = JPH::RVec3(2, -3, 4);
        options.mUseContinuousCollision = false;
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        mPhysics.SetGravity(JPH::Vec3(0, 0, -10));
        auto& bodies = mPhysics.GetBodyInterface();
        const auto ground = bodies.CreateAndAddBody(JPH::BodyCreationSettings(
            new JPH::BoxShape(JPH::Vec3(50, 50, 1)), JPH::RVec3(0, 0, -1),
            JPH::Quat::sIdentity(), JPH::EMotionType::Static, sObjectLayer), JPH::EActivation::DontActivate);
        ASSERT_FALSE(ground.IsInvalid());
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        constexpr float dt = 1.0f / 60.0f;
        RagdollCore::RagdollSolverImpulse observed;
        for (int step = 0; step < 5; ++step)
        {
            ASSERT_TRUE(ragdoll.beginSolverImpulseMeasurement(dt));
            ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
            ASSERT_TRUE(ragdoll.endSolverImpulseMeasurement(dt, observed));
        }
        RagdollCore::RagdollTelemetry telemetry;
        ASSERT_TRUE(ragdoll.captureTelemetry(telemetry));
        const float expected = telemetry.mTotalMass * 10.0f * dt
            * (1.0f - profile.mBodies[0].mMaterial.mLinearDamping * dt);
        EXPECT_NEAR(observed.mLinearImpulse.GetZ(), expected, 0.01f);
        EXPECT_LT(observed.mAngularImpulse.Length(), 0.01f);
        const auto cop = RagdollCore::estimateHorizontalPlaneCenterOfPressure(observed, 0.0);
        ASSERT_TRUE(cop);
        EXPECT_LT(JPH::Vec3(*cop - JPH::RVec3(2, -3, 0)).Length(), 0.01f);
        EXPECT_FALSE(observed.mVelocityLimitTouched);
        EXPECT_FALSE(observed.mActivationChanged);
        EXPECT_FALSE(observed.mContinuousCollisionEnabled);
        bodies.RemoveBody(ground);
        bodies.DestroyBody(ground);
    }

    TEST_F(PassiveRagdollTest, preparesAddsRemovesAndDestroysBodies)
    {
        auto created = create();
        ASSERT_TRUE(created.isValid());
        ASSERT_EQ(created.mRagdoll->getBodyCount(), 2u);
        EXPECT_EQ(mPhysics.GetNumBodies(), 2u);
        EXPECT_FALSE(created.mRagdoll->isAdded());
        EXPECT_FALSE(mPhysics.GetBodyInterface().IsAdded(created.mRagdoll->getBodyId(0)));

        EXPECT_TRUE(created.mRagdoll->addToPhysicsSystem());
        EXPECT_TRUE(created.mRagdoll->isAdded());
        EXPECT_TRUE(mPhysics.GetBodyInterface().IsAdded(created.mRagdoll->getBodyId(0)));
        EXPECT_FALSE(created.mRagdoll->addToPhysicsSystem());
        EXPECT_TRUE(created.mRagdoll->getBodyId(8).IsInvalid());

        created.mRagdoll->removeFromPhysicsSystem();
        EXPECT_FALSE(created.mRagdoll->isAdded());
        EXPECT_FALSE(mPhysics.GetBodyInterface().IsAdded(created.mRagdoll->getBodyId(0)));
        created.mRagdoll.reset();
        EXPECT_EQ(mPhysics.GetNumBodies(), 0u);
    }

    TEST_F(PassiveRagdollTest, observesAndReleasesOneConstraintIdempotently)
    {
        auto created = create();
        ASSERT_TRUE(created.isValid());
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_EQ(ragdoll.isConstraintEnabled(1), true);
        EXPECT_FALSE(ragdoll.isConstraintEnabled(0));
        EXPECT_FALSE(ragdoll.isConstraintEnabled(99));

        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        ASSERT_EQ(mPhysics.Update(1.0f / 60.0f, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        RagdollCore::ConstraintImpulse impulse;
        ASSERT_TRUE(ragdoll.getConstraintImpulse(1, impulse));
        EXPECT_TRUE(std::isfinite(impulse.mLinear));
        EXPECT_TRUE(std::isfinite(impulse.mAngular));

        ASSERT_EQ(ragdoll.releaseConstraint(1), true);
        EXPECT_EQ(ragdoll.isConstraintEnabled(1), false);
        EXPECT_EQ(ragdoll.releaseConstraint(1), false);
        EXPECT_FALSE(ragdoll.releaseConstraint(0));
        EXPECT_FALSE(ragdoll.releaseConstraint(99));
        EXPECT_FALSE(ragdoll.getConstraintImpulse(1, impulse));
        EXPECT_EQ(mPhysics.GetNumBodies(), 2u);
    }

    TEST_F(PassiveRagdollTest, newPassiveBodyMatchesExplicitPassiveModeOnItsFirstStep)
    {
        mPhysics.SetGravity(JPH::Vec3::sZero());
        auto ordinary = create();
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mRootPosition = JPH::RVec3(100, 0, 0);
        auto explicitPassive = RagdollCore::createPassiveRagdoll(mPhysics, RagdollCoreTest::makeSkeleton(),
            RagdollCoreTest::makeProfile(), options, {}, 8, 1234);
        ASSERT_TRUE(ordinary.isValid());
        ASSERT_TRUE(explicitPassive.isValid());
        ASSERT_TRUE(ordinary.mRagdoll->addToPhysicsSystem());
        ASSERT_TRUE(explicitPassive.mRagdoll->addToPhysicsSystem());
        explicitPassive.mRagdoll->disableMotors();
        // This fixture deliberately starts with separated joint anchors. The
        // first solve must use passive constraint settings even when no caller
        // has visited a powered-controller branch to disable its motors.
        RagdollCore::RagdollTelemetry before;
        ASSERT_TRUE(ordinary.mRagdoll->captureTelemetry(before));
        ASSERT_GT(before.mConstraints.front().mAnchorSeparation, 1.0f);
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        ASSERT_EQ(mPhysics.Update(1.0f / 60.0f, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        RagdollCore::RagdollPose first, second;
        ASSERT_TRUE(ordinary.mRagdoll->captureModelPose(first));
        ASSERT_TRUE(explicitPassive.mRagdoll->captureModelPose(second));
        ASSERT_EQ(first.mBodyTransforms.size(), second.mBodyTransforms.size());
        for (std::size_t i = 0; i < first.mBodyTransforms.size(); ++i)
        {
            EXPECT_TRUE(first.mBodyTransforms[i].mTranslation.IsClose(second.mBodyTransforms[i].mTranslation));
            EXPECT_TRUE(first.mBodyTransforms[i].mRotation.IsClose(second.mBodyTransforms[i].mRotation));
        }
    }

    TEST_F(PassiveRagdollTest, transfersBoneOriginMotionToOffsetMassCenterWithoutMovingThePose)
    {
        auto skeleton = RagdollCoreTest::makeSkeleton();
        skeleton.mBones[1].mModelTransform.mTranslation = JPH::Vec3(10, 0, 0);
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[0].mShape.mType = RagdollCore::ShapeType::Capsule;
        profile.mBodies[0].mShape.mRadius = 2;
        profile.mBodies[0].mShape.mHalfHeight = 1;
        profile.mBodies[0].mShape.mFitToBone = "torso";
        profile.mBodies[0].mCenterOfMass = JPH::Vec3(5, 0, 0);
        profile.mBodies[1].mConstraint->mDeriveFrameFromSkeleton = true;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mUniformScale = 2;
        options.mRootPosition = JPH::RVec3(10, 20, 30);
        options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), JPH::JPH_PI / 2);
        RagdollCore::PassiveRagdollInitialState initial;
        initial.mBodyMotions = { { JPH::Vec3(1, 2, 3), JPH::Vec3(0, 0, 2) }, {} };
        auto created = RagdollCore::createPassiveRagdoll(mPhysics, skeleton, profile, options, initial, 7, 1234);
        ASSERT_TRUE(created.isValid());
        const auto id = created.mRagdoll->getBodyId(0);
        const auto& bodies = mPhysics.GetBodyInterface();
        EXPECT_LT((bodies.GetPosition(id) - options.mRootPosition).Length(), 1e-4);
        EXPECT_LT((bodies.GetCenterOfMassPosition(id) - JPH::RVec3(10, 30, 30)).Length(), 1e-4);
        EXPECT_LT((bodies.GetLinearVelocity(id) - JPH::Vec3(-19, 2, 3)).Length(), 1e-4);
        EXPECT_LT((bodies.GetPointVelocity(id, options.mRootPosition) - JPH::Vec3(1, 2, 3)).Length(), 1e-4);
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem(JPH::EActivation::DontActivate));
        RagdollCore::RagdollPose pose;
        ASSERT_TRUE(created.mRagdoll->captureModelPose(pose));
        ASSERT_EQ(pose.mBodyTransforms.size(), skeleton.mBones.size());
        for (std::size_t i = 0; i < skeleton.mBones.size(); ++i)
        {
            EXPECT_LT((pose.mBodyTransforms[i].mTranslation - skeleton.mBones[i].mModelTransform.mTranslation).Length(), 1e-4);
            EXPECT_GT(std::abs(pose.mBodyTransforms[i].mRotation.Dot(skeleton.mBones[i].mModelTransform.mRotation)), 0.99999f);
        }
        RagdollCore::RagdollTelemetry telemetry;
        ASSERT_TRUE(created.mRagdoll->captureTelemetry(telemetry));
        ASSERT_EQ(telemetry.mConstraints.size(), 1u);
        EXPECT_LT(telemetry.mConstraints.front().mAnchorSeparation, 1e-4);
    }

    TEST_F(PassiveRagdollTest, inheritsPerBodyMotionAndAppliesTargetedKillingImpulse)
    {
        RagdollCore::PassiveRagdollInitialState initialState;
        initialState.mBodyMotions = {
            { JPH::Vec3(1.0f, 2.0f, 3.0f), JPH::Vec3(0.1f, 0.2f, 0.3f) },
            { JPH::Vec3(4.0f, 5.0f, 6.0f), JPH::Vec3(0.4f, 0.5f, 0.6f) },
        };
        initialState.mKillingImpulse
            = RagdollCore::KillingImpulse{ 1, JPH::Vec3(20.0f, 0.0f, 0.0f), JPH::RVec3(0.0f, 10.0f, 0.0f) };

        auto created = create(initialState);
        ASSERT_TRUE(created.isValid());
        const JPH::BodyInterface& bodies = mPhysics.GetBodyInterface();
        EXPECT_TRUE(bodies.GetLinearVelocity(created.mRagdoll->getBodyId(0)).IsClose(
            initialState.mBodyMotions[0].mLinearVelocity));
        EXPECT_TRUE(bodies.GetLinearVelocity(created.mRagdoll->getBodyId(1)).IsClose(
            initialState.mBodyMotions[1].mLinearVelocity));
        EXPECT_FALSE(bodies.IsAdded(created.mRagdoll->getBodyId(1)));

        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem(JPH::EActivation::DontActivate));
        EXPECT_GT(bodies.GetLinearVelocity(created.mRagdoll->getBodyId(1)).GetX(), 4.0f);
        EXPECT_TRUE(bodies.GetAngularVelocity(created.mRagdoll->getBodyId(1)).IsClose(
            initialState.mBodyMotions[1].mAngularVelocity, 0.5f));
        EXPECT_FALSE(bodies.IsActive(created.mRagdoll->getBodyId(1)));
    }

    TEST_F(PassiveRagdollTest, rejectsMismatchedOrNonFiniteInitialStateBeforeAllocation)
    {
        RagdollCore::PassiveRagdollInitialState initialState;
        initialState.mBodyMotions.resize(1);
        initialState.mActorMotion.mLinearVelocity
            = JPH::Vec3(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);
        initialState.mKillingImpulse = RagdollCore::KillingImpulse{ 8, {}, {} };

        const auto created = create(initialState);
        EXPECT_FALSE(created.isValid());
        EXPECT_EQ(created.mRagdoll, nullptr);
        EXPECT_GE(created.mValidation.mIssues.size(), 3u);
        EXPECT_EQ(mPhysics.GetNumBodies(), 0u);
    }

    TEST_F(PassiveRagdollTest, destructionRollsBackAnAddedRagdoll)
    {
        auto created = create();
        ASSERT_TRUE(created.isValid());
        created.mRagdoll->addToPhysicsSystem();
        ASSERT_EQ(mPhysics.GetNumBodies(), 2u);
        created.mRagdoll.reset();
        EXPECT_EQ(mPhysics.GetNumBodies(), 0u);
    }

    TEST_F(PassiveRagdollTest, poseFloorHeightIncludesEveryShapeWithoutMovingBodies)
    {
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mUniformScale = 2.0f;
        options.mRootPosition = JPH::RVec3(100, 200, 300);
        options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.7f);
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[0].mCenterOfMass = JPH::Vec3(1, 2, 0.5f);
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());
        auto& ragdoll = *created.mRagdoll;
        RagdollCore::RagdollPose pose;
        ASSERT_TRUE(ragdoll.captureModelPose(pose));
        float lowest = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < ragdoll.getBodyCount(); ++i)
        {
            JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(i));
            ASSERT_TRUE(lock.Succeeded());
            lowest = std::min(lowest, lock.GetBody().GetWorldSpaceBounds().mMin.GetZ());
        }
        const auto center = mPhysics.GetBodyInterface().GetCenterOfMassPosition(ragdoll.getBodyId(0));
        const auto height = ragdoll.getPoseFloorHeight(pose, 0);
        ASSERT_TRUE(height);
        EXPECT_NEAR(*height, center.GetZ() - lowest, 1e-4);
        auto lowered = pose;
        const auto down = ragdoll.getModelVector(JPH::Vec3(0, 0, -100));
        ASSERT_TRUE(down);
        lowered.mBodyTransforms[1].mTranslation += *down;
        const auto lowerHeight = ragdoll.getPoseFloorHeight(lowered, 0);
        ASSERT_TRUE(lowerHeight);
        EXPECT_GT(*lowerHeight, *height + 50);
        for (auto& body : lowered.mBodyTransforms) body.mTranslation += *down;
        const auto translated = ragdoll.getPoseFloorHeight(lowered, 0);
        ASSERT_TRUE(translated);
        EXPECT_NEAR(*translated, *lowerHeight, 1e-4);
        RagdollCore::RagdollPose after;
        ASSERT_TRUE(ragdoll.captureModelPose(after));
        for (std::size_t i = 0; i < pose.mBodyTransforms.size(); ++i)
        {
            EXPECT_EQ(after.mBodyTransforms[i].mTranslation, pose.mBodyTransforms[i].mTranslation);
            EXPECT_EQ(after.mBodyTransforms[i].mRotation, pose.mBodyTransforms[i].mRotation);
        }
        EXPECT_FALSE(ragdoll.getPoseFloorHeight(pose, pose.mBodyTransforms.size()));
        lowered.mBodyTransforms.pop_back();
        EXPECT_FALSE(ragdoll.getPoseFloorHeight(lowered, 0));
        pose.mBodyTransforms[1].mRotation.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(ragdoll.getPoseFloorHeight(pose, 0));
    }

    TEST_F(PassiveRagdollTest, capturesUnscaledModelPoseInSkeletonOrder)
    {
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mUniformScale = 2.0f;
        options.mRootPosition = JPH::RVec3(100.0f, 200.0f, 300.0f);
        options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.5f);
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), RagdollCoreTest::makeProfile(), options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());

        RagdollCore::RagdollPose pose;
        EXPECT_FALSE(created.mRagdoll->captureModelPose(pose));
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());
        ASSERT_TRUE(created.mRagdoll->captureModelPose(pose));
        ASSERT_EQ(pose.mBodyTransforms.size(), 2u);
        EXPECT_TRUE(pose.mBodyTransforms[0].mTranslation.IsNearZero());
        EXPECT_TRUE(pose.mBodyTransforms[0].mRotation.IsClose(JPH::Quat::sIdentity()));
        EXPECT_TRUE(pose.mBodyTransforms[1].mTranslation.IsClose(JPH::Vec3(0.0f, 10.0f, 0.0f)));
        EXPECT_TRUE(pose.mBodyTransforms[1].mRotation.IsClose(JPH::Quat::sIdentity()));

        const auto worldPosition = created.mRagdoll->getWorldBodyPosition(pose, 1);
        ASSERT_TRUE(worldPosition.has_value());
        EXPECT_TRUE(worldPosition->IsClose(mPhysics.GetBodyInterface().GetPosition(created.mRagdoll->getBodyId(1))));
        const auto modelPosition = created.mRagdoll->getModelPosition(*worldPosition);
        ASSERT_TRUE(modelPosition.has_value());
        EXPECT_TRUE(modelPosition->IsClose(pose.mBodyTransforms[1].mTranslation));
        EXPECT_FALSE(created.mRagdoll->getModelPosition(JPH::RVec3(
            std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)).has_value());
        EXPECT_FALSE(created.mRagdoll->getWorldBodyPosition(pose, 2).has_value());
        const auto worldRotation = created.mRagdoll->getWorldBodyRotation(pose, 1);
        ASSERT_TRUE(worldRotation.has_value());
        EXPECT_TRUE(worldRotation->IsClose(options.mRootRotation));
        EXPECT_FALSE(created.mRagdoll->getWorldBodyRotation(pose, 2).has_value());
        const auto worldVector = created.mRagdoll->getWorldVector(JPH::Vec3::sAxisY());
        ASSERT_TRUE(worldVector.has_value());
        EXPECT_TRUE(worldVector->IsClose(options.mRootRotation * (2.0f * JPH::Vec3::sAxisY())));
        const auto modelVector = created.mRagdoll->getModelVector(*worldVector);
        ASSERT_TRUE(modelVector);
        EXPECT_TRUE(modelVector->IsClose(JPH::Vec3::sAxisY(), 1e-8f));
        const auto zeroVector = created.mRagdoll->getModelVector(JPH::Vec3::sZero());
        ASSERT_TRUE(zeroVector);
        EXPECT_TRUE(zeroVector->IsNearZero());
        EXPECT_FALSE(created.mRagdoll->getModelVector(JPH::Vec3(
            std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f)));
        EXPECT_FALSE(created.mRagdoll->getWorldVector(JPH::Vec3(
            std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f)).has_value());

        pose.mBodyTransforms[1].mTranslation.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(created.mRagdoll->getWorldBodyPosition(pose, 1).has_value());
        pose.mBodyTransforms[1].mRotation.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(created.mRagdoll->getWorldBodyRotation(pose, 1).has_value());
    }

    TEST_F(PassiveRagdollTest, weaponAimSurvivesActorRotationAndScale)
    {
        const RagdollCore::WeaponStrikeAxes axes{ JPH::Vec3::sAxisZ(), JPH::Vec3::sAxisX() };
        const JPH::Vec3 direction = JPH::Vec3(1, 2, -1).Normalized();
        const JPH::Vec3 tangent = JPH::Vec3::sAxisX();
        const auto worldAim = RagdollCore::alignedStrikeHandRotation(axes, direction, tangent);
        ASSERT_TRUE(worldAim);
        for (const float yaw : { 0.0f, 0.7f, 1.5707963f, 3.1415927f })
            for (const float scale : { 0.75f, 1.0f, 2.0f })
            {
                SCOPED_TRACE(testing::Message() << "yaw=" << yaw << " scale=" << scale);
                RagdollCore::BuildOptions options;
                options.mObjectLayer = sObjectLayer;
                options.mUniformScale = scale;
                options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), yaw);
                auto created = RagdollCore::createPassiveRagdoll(mPhysics, RagdollCoreTest::makeSkeleton(),
                    RagdollCoreTest::makeProfile(), options, {}, 7, 1234);
                ASSERT_TRUE(created.isValid());
                const auto modelAim = created.mRagdoll->getModelRotation(*worldAim);
                const auto modelTip = created.mRagdoll->getModelVector(direction * 50.0f);
                ASSERT_TRUE(modelAim);
                ASSERT_TRUE(modelTip);
                const auto actualTip = created.mRagdoll->getWorldVector(
                    *modelAim * axes.mLongAxis * modelTip->Length());
                ASSERT_TRUE(actualTip);
                EXPECT_LT((*actualTip - direction * 50.0f).Length(), 1e-4f);
                const auto actualEdge = created.mRagdoll->getWorldVector(*modelAim * axes.mEdgeAxis);
                ASSERT_TRUE(actualEdge);
                EXPECT_LT((actualEdge->Normalized() - *worldAim * axes.mEdgeAxis).Length(), 1e-4f);
                EXPECT_FALSE(created.mRagdoll->getModelRotation(JPH::Quat(0, 0, 0, 0)));
                EXPECT_FALSE(created.mRagdoll->getModelRotation(
                    JPH::Quat(std::numeric_limits<float>::quiet_NaN(), 0, 0, 1)));
            }
    }

    TEST_F(PassiveRagdollTest, rebasesCachedModelPoseWhenActorReferenceMoves)
    {
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mUniformScale = 2.0f;
        options.mRootPosition = JPH::RVec3(100.0f, 200.0f, 300.0f);
        options.mRootRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.5f);
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), RagdollCoreTest::makeProfile(), options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());

        RagdollCore::RagdollPose pose;
        ASSERT_TRUE(created.mRagdoll->captureModelPose(pose));
        const auto originalPosition = created.mRagdoll->getWorldBodyPosition(pose, 1);
        const auto originalRotation = created.mRagdoll->getWorldBodyRotation(pose, 1);
        ASSERT_TRUE(originalPosition.has_value());
        ASSERT_TRUE(originalRotation.has_value());

        const JPH::RVec3 newPosition(140.0f, 180.0f, 320.0f);
        const JPH::Quat newRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), -0.25f);
        const JPH::Vec3 cachedPoint = pose.mBodyTransforms[1].mTranslation;
        const auto rebasedPoint = created.mRagdoll->rebaseModelPosition(cachedPoint, newPosition, newRotation);
        ASSERT_TRUE(rebasedPoint.has_value());
        ASSERT_TRUE(created.mRagdoll->rebaseModelPose(pose, newPosition, newRotation));
        ASSERT_TRUE(created.mRagdoll->setReferenceTransform(newPosition, newRotation));
        const auto rebasedPosition = created.mRagdoll->getWorldBodyPosition(pose, 1);
        const auto rebasedRotation = created.mRagdoll->getWorldBodyRotation(pose, 1);
        ASSERT_TRUE(rebasedPosition.has_value());
        ASSERT_TRUE(rebasedRotation.has_value());
        EXPECT_NEAR(rebasedPosition->GetX(), originalPosition->GetX(), 1e-4);
        EXPECT_NEAR(rebasedPosition->GetY(), originalPosition->GetY(), 1e-4);
        EXPECT_NEAR(rebasedPosition->GetZ(), originalPosition->GetZ(), 1e-4);
        const auto rebasedPointWorld = created.mRagdoll->getWorldBodyPosition(
            RagdollCore::RagdollPose{ { RagdollCore::Transform{ *rebasedPoint, JPH::Quat::sIdentity() } } }, 0);
        ASSERT_TRUE(rebasedPointWorld.has_value());
        EXPECT_NEAR(rebasedPointWorld->GetX(), originalPosition->GetX(), 1e-4);
        EXPECT_NEAR(rebasedPointWorld->GetY(), originalPosition->GetY(), 1e-4);
        EXPECT_NEAR(rebasedPointWorld->GetZ(), originalPosition->GetZ(), 1e-4);
        EXPECT_TRUE(rebasedRotation->IsClose(*originalRotation));
        EXPECT_TRUE(created.mRagdoll->getReferencePosition().IsClose(newPosition));

        RagdollCore::RagdollPose invalid = pose;
        invalid.mBodyTransforms[0].mTranslation.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(created.mRagdoll->rebaseModelPose(invalid, options.mRootPosition, options.mRootRotation));
        EXPECT_FALSE(created.mRagdoll->rebaseModelPosition(JPH::Vec3(
            std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f), options.mRootPosition,
            options.mRootRotation));
        EXPECT_FALSE(created.mRagdoll->setReferenceTransform(JPH::RVec3(
            std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0), JPH::Quat::sIdentity()));
        EXPECT_TRUE(created.mRagdoll->getReferencePosition().IsClose(newPosition));
    }

    TEST_F(PassiveRagdollTest, capturesCenterOfMassConstraintErrorAndHingeState)
    {
        auto profile = RagdollCoreTest::makeProfile();
        ASSERT_TRUE(profile.mBodies[1].mConstraint.has_value());
        profile.mBodies[1].mConstraint->mType = RagdollCore::ConstraintType::Hinge;
        profile.mBodies[1].mConstraint->mDeriveFrameFromSkeleton = true;
        profile.mBodies[1].mConstraint->mHingeMinAngle = -0.4f;
        profile.mBodies[1].mConstraint->mHingeMaxAngle = 0.7f;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());

        RagdollCore::RagdollTelemetry telemetry;
        EXPECT_FALSE(created.mRagdoll->captureTelemetry(telemetry));
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());
        ASSERT_TRUE(created.mRagdoll->captureTelemetry(telemetry));
        EXPECT_NEAR(telemetry.mTotalMass, 75.0f, 1e-4f);
        EXPECT_NEAR(telemetry.mCenterOfMass.GetX(), 0.0, 1e-5);
        EXPECT_NEAR(telemetry.mCenterOfMass.GetY(), 4.0, 1e-5);
        EXPECT_NEAR(telemetry.mCenterOfMass.GetZ(), 0.0, 1e-5);
        ASSERT_EQ(telemetry.mBodies.size(), 2u);
        EXPECT_NEAR(telemetry.mBodies[0].mMass, 45.0f, 1e-4f);
        EXPECT_NEAR(telemetry.mBodies[1].mMass, 30.0f, 1e-4f);
        EXPECT_NEAR(telemetry.mBodies[1].mMass / telemetry.mBodies[0].mMass, 2.0f / 3.0f, 1e-5f);
        EXPECT_TRUE(telemetry.mBodies[0].mPosition.IsClose(JPH::RVec3::sZero()));
        EXPECT_TRUE(telemetry.mBodies[1].mPosition.IsClose(JPH::RVec3(0.0f, 10.0f, 0.0f)));
        EXPECT_TRUE(telemetry.mBodies[0].mCenterOfMass.IsClose(JPH::RVec3::sZero()));
        EXPECT_TRUE(telemetry.mBodies[0].mRotation.IsClose(JPH::Quat::sIdentity()));
        EXPECT_TRUE(telemetry.mBodies[0].mMotion.mLinearVelocity.IsNearZero());
        EXPECT_TRUE(telemetry.mBodies[0].mMotion.mAngularVelocity.IsNearZero());
        EXPECT_TRUE(telemetry.mBodies[0].mActive);
        ASSERT_EQ(telemetry.mConstraints.size(), 1u);
        EXPECT_EQ(telemetry.mConstraints[0].mParentBodyIndex, 0u);
        EXPECT_EQ(telemetry.mConstraints[0].mChildBodyIndex, 1u);
        EXPECT_NEAR(telemetry.mConstraints[0].mAnchorSeparation, 0.0f, 1e-4f);
        ASSERT_TRUE(telemetry.mConstraints[0].mHingeAngle.has_value());
        EXPECT_NEAR(*telemetry.mConstraints[0].mHingeAngle, 0.0f, 1e-4f);
        EXPECT_FLOAT_EQ(*telemetry.mConstraints[0].mHingeMinimum, -0.4f);
        EXPECT_FLOAT_EQ(*telemetry.mConstraints[0].mHingeMaximum, 0.7f);

        JPH::BodyInterface& bodies = mPhysics.GetBodyInterface();
        const JPH::BodyID child = created.mRagdoll->getBodyId(1);
        bodies.SetPosition(child, bodies.GetPosition(child) + JPH::RVec3(3.0f, 0.0f, 0.0f),
            JPH::EActivation::DontActivate);
        ASSERT_TRUE(created.mRagdoll->captureTelemetry(telemetry));
        EXPECT_NEAR(telemetry.mConstraints[0].mAnchorSeparation, 3.0f, 1e-3f);
    }

    TEST_F(PassiveRagdollTest, boundedJointMotorDrivesTowardValidatedTargetPose)
    {
        auto profile = RagdollCoreTest::makeProfile();
        profile.mBodies[1].mConstraint->mMotor.mFrequency = 6.0f;
        profile.mBodies[1].mConstraint->mMotor.mDamping = 1.0f;
        profile.mBodies[1].mConstraint->mMotor.mMaximumTorque = 100000.0f;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());

        RagdollCore::RagdollPose target;
        target.mBodyTransforms = {
            { JPH::Vec3::sZero(), JPH::Quat::sIdentity() },
            { JPH::Vec3(0.0f, 10.0f, 0.0f), JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.2f) },
        };
        RagdollCore::MotorTargetTelemetry unchangedTarget;
        unchangedTarget.mAngularError = 2.0f;
        EXPECT_FALSE(created.mRagdoll->captureMotorTargetTelemetry(1, unchangedTarget));
        EXPECT_FLOAT_EQ(unchangedTarget.mAngularError, 2.0f);
        EXPECT_FALSE(created.mRagdoll->driveToPoseUsingMotors(target));
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());

        RagdollCore::RagdollPose initial;
        ASSERT_TRUE(created.mRagdoll->captureModelPose(initial));
        const float initialDot = std::abs(initial.mBodyTransforms[1].mRotation.Dot(target.mBodyTransforms[1].mRotation));

        mPhysics.SetGravity(JPH::Vec3::sZero());
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        const std::array<std::size_t, 1> disabledChild{ 1 };
        const std::array<std::size_t, 1> disabledRoot{ 0 };
        const std::array<std::size_t, 2> duplicateChild{ 1, 1 };
        EXPECT_FALSE(created.mRagdoll->driveToPoseUsingMotors(target, disabledRoot));
        EXPECT_FALSE(created.mRagdoll->driveToPoseUsingMotors(target, duplicateChild));
        JPH::Vec3 unchangedImpulse(1.0f, 2.0f, 3.0f);
        float unchangedUtilization = 2.0f;
        EXPECT_FALSE(created.mRagdoll->getMotorAngularImpulse(
            0, 1.0f / 60.0f, unchangedImpulse, unchangedUtilization));
        EXPECT_FALSE(created.mRagdoll->getMotorAngularImpulse(
            1, 0.0f, unchangedImpulse, unchangedUtilization));
        EXPECT_TRUE(unchangedImpulse.IsClose(JPH::Vec3(1.0f, 2.0f, 3.0f)));
        EXPECT_FLOAT_EQ(unchangedUtilization, 2.0f);
        for (int step = 0; step < 30; ++step)
        {
            ASSERT_TRUE(created.mRagdoll->driveToPoseUsingMotors(target, disabledChild));
            ASSERT_EQ(mPhysics.Update(1.0f / 60.0f, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        }
        RagdollCore::RagdollPose excluded;
        ASSERT_TRUE(created.mRagdoll->captureModelPose(excluded));
        const float excludedDot
            = std::abs(excluded.mBodyTransforms[1].mRotation.Dot(target.mBodyTransforms[1].mRotation));
        EXPECT_NEAR(excludedDot, initialDot, 1e-4f);

        JPH::Vec3 firstMotorImpulse = JPH::Vec3::sZero();
        float firstMotorUtilization = 0.0f;
        for (int step = 0; step < 30; ++step)
        {
            ASSERT_TRUE(created.mRagdoll->driveToPoseUsingMotors(target));
            if (step == 0)
            {
                EXPECT_FALSE(created.mRagdoll->limitMotorTargetStep(0, 0.1f));
                EXPECT_FALSE(created.mRagdoll->limitMotorTargetStep(1, 0.0f));
            }
            ASSERT_TRUE(created.mRagdoll->limitMotorTargetStep(1, 0.1f));
            RagdollCore::MotorTargetTelemetry motorTarget;
            ASSERT_TRUE(created.mRagdoll->captureMotorTargetTelemetry(1, motorTarget));
            EXPECT_EQ(motorTarget.mConstraintType, RagdollCore::ConstraintType::SwingTwist);
            EXPECT_TRUE(motorTarget.mCurrentOrientation.has_value());
            EXPECT_TRUE(motorTarget.mRequestedOrientation.has_value());
            EXPECT_TRUE(motorTarget.mTargetOrientation.has_value());
            EXPECT_TRUE(motorTarget.mNormalHalfConeAngle.has_value());
            EXPECT_TRUE(motorTarget.mPlaneHalfConeAngle.has_value());
            EXPECT_FALSE(motorTarget.mCurrentHingeAngle.has_value());
            EXPECT_LE(motorTarget.mAngularError, 0.1001f);
            ASSERT_EQ(mPhysics.Update(1.0f / 60.0f, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
            if (step == 0)
                ASSERT_TRUE(created.mRagdoll->getMotorAngularImpulse(
                    1, 1.0f / 60.0f, firstMotorImpulse, firstMotorUtilization));
        }
        EXPECT_GT(firstMotorImpulse.Length(), 0.0f);
        EXPECT_GT(firstMotorUtilization, 0.0f);
        EXPECT_LE(firstMotorUtilization, 1.0001f);

        RagdollCore::RagdollPose driven;
        ASSERT_TRUE(created.mRagdoll->captureModelPose(driven));
        const float drivenDot = std::abs(driven.mBodyTransforms[1].mRotation.Dot(target.mBodyTransforms[1].mRotation));
        EXPECT_GT(drivenDot, excludedDot);
        created.mRagdoll->disableMotors();

        target.mBodyTransforms[1].mRotation = JPH::Quat(
            std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 1.0f);
        EXPECT_FALSE(created.mRagdoll->driveToPoseUsingMotors(target));
    }

    TEST_F(PassiveRagdollTest, releasesOnlySelectedMotorRotationAndReportsInternalTorque)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& motor = profile.mBodies[1].mConstraint->mMotor;
        motor.mFrequency = 6.0f;
        motor.mDamping = 1.0f;
        motor.mMaximumTorque = 1000.0f;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        RagdollCore::RagdollPose target;
        ASSERT_TRUE(ragdoll.captureModelPose(target));
        // Exercise an oblique world axis, with both pose error and velocity
        // along it. None may leak back into the released direction.
        const JPH::Vec3 freeAxis = JPH::Vec3(1, 2, 3).Normalized();
        const JPH::Vec3 controlledAxis = freeAxis.GetNormalizedPerpendicular();
        target.mBodyTransforms[1].mRotation = JPH::Quat::sRotation(freeAxis, 0.1f);
        mPhysics.GetBodyInterface().SetAngularVelocity(ragdoll.getBodyId(1),
            2.0f * freeAxis + controlledAxis);
        ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(target));
        constexpr float step = 1.0f / 60.0f;
        EXPECT_FALSE(ragdoll.driveMotorWithFreeRotationAxis(0, freeAxis, step));
        EXPECT_FALSE(ragdoll.driveMotorWithFreeRotationAxis(1, 2.0f * freeAxis, step));
        EXPECT_FALSE(ragdoll.driveMotorWithFreeRotationAxis(1, freeAxis, 0.0f));
        ASSERT_TRUE(ragdoll.driveMotorWithFreeRotationAxis(1, freeAxis, step));
        EXPECT_FALSE(ragdoll.driveMotorWithFreeRotationAxis(1, freeAxis, step));
        JPH::Vec3 torque;
        {
            JPH::BodyLockRead parent(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(0));
            JPH::BodyLockRead child(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(1));
            ASSERT_TRUE(parent.Succeeded() && child.Succeeded());
            torque = child.GetBody().GetAccumulatedTorque();
            EXPECT_NEAR(torque.Dot(freeAxis), 0.0f, 1e-3f);
            EXPECT_LT(torque.Dot(controlledAxis), 0.0f) << "control on the other axes still damps motion";
            EXPECT_LE(torque.Length(), motor.mMaximumTorque + 1e-3f);
            EXPECT_TRUE((torque + parent.GetBody().GetAccumulatedTorque()).IsNearZero());
        }
        mPhysics.SetGravity(JPH::Vec3::sZero());
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        ASSERT_EQ(mPhysics.Update(step, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        JPH::Vec3 impulse;
        float utilization = 0.0f;
        ASSERT_TRUE(ragdoll.getMotorAngularImpulse(1, step, impulse, utilization));
        EXPECT_TRUE(impulse.IsClose(torque * step));
        EXPECT_GT(utilization, 0.0f);
        EXPECT_LE(utilization, 1.0f);
        // The next ordinary pose drive restores the native motor.
        ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(target));
        RagdollCore::MotorTargetTelemetry telemetry;
        EXPECT_TRUE(ragdoll.captureMotorTargetTelemetry(1, telemetry));
    }

    TEST_F(PassiveRagdollTest, worldMotorDoesNotFollowParentAccelerationAfterTargetAssembly)
    {
        for (const bool solverDrive : { false, true })
            for (const bool worldReference : { false, true })
            {
                SCOPED_TRACE(testing::Message() << "solver_drive=" << solverDrive << " world=" << worldReference);
                auto profile = RagdollCoreTest::makeProfile();
                auto& motor = profile.mBodies[1].mConstraint->mMotor;
                motor.mFrequency = 6.0f;
                motor.mDamping = 1.0f;
                motor.mMaximumTorque = 1000.0f;
                auto skeleton = RagdollCoreTest::makeSkeleton();
                skeleton.mBones[1].mModelTransform.mTranslation = JPH::Vec3::sZero();
                for (auto& body : profile.mBodies)
                {
                    body.mMaterial.mLinearDamping = 0;
                    body.mMaterial.mAngularDamping = 0;
                }
                RagdollCore::BuildOptions options;
                options.mObjectLayer = sObjectLayer;
                options.mUseContinuousCollision = false;
                auto created = RagdollCore::createPassiveRagdoll(
                    mPhysics, skeleton, profile, options, {}, 7, 1234);
                ASSERT_TRUE(created.isValid());
                auto& ragdoll = *created.mRagdoll;
                ASSERT_TRUE(ragdoll.addToPhysicsSystem());
                auto& bodies = mPhysics.GetBodyInterface();
                RagdollCore::RagdollPose pose;
                ASSERT_TRUE(ragdoll.captureModelPose(pose));
                ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
                constexpr float dt = 1.0f / 60.0f;
                if (worldReference)
                    ASSERT_TRUE(ragdoll.driveMotorToWorldOrientation(1, bodies.GetRotation(ragdoll.getBodyId(1)), dt));
                auto* joint = static_cast<JPH::SwingTwistConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
                EXPECT_EQ(joint->GetPositionMotorWorldVelocityReference(), worldReference);
                JPH::StateRecorderImpl saved;
                joint->SaveState(saved);
                joint->SetPositionMotorWorldVelocityReference(!worldReference);
                saved.Rewind();
                joint->RestoreState(saved);
                ASSERT_FALSE(saved.IsFailed());
                EXPECT_EQ(joint->GetPositionMotorWorldVelocityReference(), worldReference);
                struct ConstraintCleanup
                {
                    JPH::PhysicsSystem& mPhysics;
                    JPH::Ref<JPH::TwoBodyConstraint> mConstraint;
                    ~ConstraintCleanup() { if (mConstraint) mPhysics.RemoveConstraint(mConstraint); }
                } parentDrive{ mPhysics, nullptr };
                JPH::Vec3 driveAxis = JPH::Vec3::sAxisY();
                if (solverDrive)
                {
                    JPH::SwingTwistConstraintSettings settings;
                    settings.mNormalHalfConeAngle = settings.mPlaneHalfConeAngle = 0.5f;
                    settings.mTwistMinAngle = -0.5f;
                    settings.mTwistMaxAngle = 0.5f;
                    settings.mSwingMotorSettings = JPH::MotorSettings(6.0f, 1.0f);
                    settings.mTwistMotorSettings = settings.mSwingMotorSettings;
                    settings.mSwingMotorSettings.SetTorqueLimit(6000.0f);
                    settings.mTwistMotorSettings.SetTorqueLimit(6000.0f);
                    parentDrive.mConstraint = bodies.CreateConstraint(&settings, JPH::BodyID(), ragdoll.getBodyId(0));
                    mPhysics.AddConstraint(parentDrive.mConstraint);
                    auto* driver = static_cast<JPH::SwingTwistConstraint*>(parentDrive.mConstraint.GetPtr());
                    driver->SetSwingMotorState(JPH::EMotorState::Position);
                    driver->SetTwistMotorState(JPH::EMotorState::Position);
                    driver->SetTargetOrientationCS(JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.1f));
                    driveAxis = (driver->GetBody1()->GetRotation() * driver->GetConstraintToBody1()).RotateAxisY();
                }
                else
                    bodies.AddTorque(ragdoll.getBodyId(0), JPH::Vec3(0, 6000, 0));
                mPhysics.SetGravity(JPH::Vec3::sZero());
                JPH::TempAllocatorImpl allocator(1024 * 1024);
                JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
                ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
                EXPECT_GT(bodies.GetAngularVelocity(ragdoll.getBodyId(0)).Dot(driveAxis), 0.01f);
                JPH::Vec3 impulse;
                float utilization = 0;
                ASSERT_TRUE(ragdoll.getMotorAngularImpulse(1, dt, impulse, utilization));
                if (worldReference)
                {
                    EXPECT_LT(bodies.GetAngularVelocity(ragdoll.getBodyId(1)).Length(), 1e-4f)
                        << "The world target must not acquire parent motion from integration or another solver motor";
                    EXPECT_LT(impulse.Length(), 1e-4f);
                }
                else
                {
                    EXPECT_GT(bodies.GetAngularVelocity(ragdoll.getBodyId(1)).Dot(driveAxis), 0.01f);
                    EXPECT_GT(impulse.Length(), 1e-4f);
                }
                EXPECT_LE(utilization, 1.0f);
            }
    }

    TEST_F(PassiveRagdollTest, worldMotorDoesNotBrakeParentMotionThroughStationaryChild)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& motor = profile.mBodies[1].mConstraint->mMotor;
        motor.mFrequency = 6.0f;
        motor.mDamping = 1.0f;
        motor.mMaximumTorque = 1000.0f;
        auto skeleton = RagdollCoreTest::makeSkeleton();
        skeleton.mBones[1].mModelTransform.mTranslation = JPH::Vec3::sZero();
        for (auto& body : profile.mBodies)
        {
            body.mMaterial.mLinearDamping = 0;
            body.mMaterial.mAngularDamping = 0;
        }
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mUseContinuousCollision = false;
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, skeleton, profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        auto& bodies = mPhysics.GetBodyInterface();
        RagdollCore::RagdollPose pose;
        ASSERT_TRUE(ragdoll.captureModelPose(pose));
        bodies.SetAngularVelocity(ragdoll.getBodyId(0), JPH::Vec3(1, 2, 3));
        ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
        constexpr float step = 1.0f / 60.0f;
        const JPH::Quat target = bodies.GetRotation(ragdoll.getBodyId(1));
        EXPECT_FALSE(ragdoll.driveMotorToWorldOrientation(1,
            JPH::Quat(std::numeric_limits<float>::quiet_NaN(), 0, 0, 1), step));
        ASSERT_TRUE(ragdoll.driveMotorToWorldOrientation(1, target, step));
        RagdollCore::MotorTargetTelemetry worldTarget;
        ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, worldTarget));
        EXPECT_TRUE(worldTarget.mWorldOrientation);
        ASSERT_TRUE(worldTarget.mTargetOrientation);
        EXPECT_TRUE(worldTarget.mTargetOrientation->IsClose(target));
        EXPECT_NEAR(worldTarget.mAngularError, 0.0f, 1e-3f);
        EXPECT_FALSE(ragdoll.driveMotorToWorldOrientation(1, target, step));
        {
            JPH::BodyLockRead child(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(1));
            ASSERT_TRUE(child.Succeeded());
            EXPECT_TRUE(child.GetBody().GetAccumulatedTorque().IsNearZero());
        }
        mPhysics.SetGravity(JPH::Vec3::sZero());
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        ASSERT_EQ(mPhysics.Update(step, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        EXPECT_LT(bodies.GetAngularVelocity(ragdoll.getBodyId(1)).Length(), 1e-3f);
        for (std::size_t body = 0; body < 2; ++body)
            bodies.SetPositionAndRotation(ragdoll.getBodyId(body), options.mRootPosition,
                options.mRootRotation, JPH::EActivation::Activate);
        // Relative damping would accelerate the child toward its faster
        // parent. World damping must instead oppose the child's own motion.
        bodies.SetAngularVelocity(ragdoll.getBodyId(0), JPH::Vec3(0, 5, 0));
        bodies.SetAngularVelocity(ragdoll.getBodyId(1), JPH::Vec3(0, 1, 0));
        ASSERT_TRUE(ragdoll.captureModelPose(pose));
        ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
        ASSERT_TRUE(ragdoll.driveMotorToWorldOrientation(1, bodies.GetRotation(ragdoll.getBodyId(1)), step));
        ASSERT_TRUE(ragdoll.beginSolverImpulseMeasurement(step));
        ASSERT_EQ(mPhysics.Update(step, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        JPH::Vec3 impulse;
        float utilization = 0;
        ASSERT_TRUE(ragdoll.getMotorAngularImpulse(1, step, impulse, utilization));
        EXPECT_LT(impulse.GetY(), 0.0f);
        EXPECT_LE(impulse.Length() / step, motor.mMaximumTorque + 1e-3f);
        EXPECT_LE(utilization, 1);
        RagdollCore::RagdollSolverImpulse observed;
        ASSERT_TRUE(ragdoll.endSolverImpulseMeasurement(step, observed));
        EXPECT_LT(observed.mAngularImpulse.Length(), 1e-2f);
        ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
        auto* joint = static_cast<JPH::SwingTwistConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
        EXPECT_TRUE(joint->GetTargetAngularVelocityCS().IsNearZero());
        EXPECT_FALSE(joint->GetPositionMotorWorldVelocityReference());
    }

    TEST_F(PassiveRagdollTest, capturesPostLimitHingeMotorTarget)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& constraint = *profile.mBodies[1].mConstraint;
        constraint.mType = RagdollCore::ConstraintType::Hinge;
        constraint.mHingeMinAngle = -0.25f;
        constraint.mHingeMaxAngle = 0.25f;
        constraint.mMotor.mFrequency = 6.0f;
        constraint.mMotor.mDamping = 1.0f;
        constraint.mMotor.mMaximumTorque = 1000.0f;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());

        RagdollCore::RagdollPose target;
        target.mBodyTransforms = {
            { JPH::Vec3::sZero(), JPH::Quat::sIdentity() },
            { JPH::Vec3(0.0f, 10.0f, 0.0f), JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.7f) },
        };
        ASSERT_TRUE(created.mRagdoll->driveToPoseUsingMotors(target));
        RagdollCore::MotorTargetTelemetry telemetry;
        ASSERT_TRUE(created.mRagdoll->captureMotorTargetTelemetry(1, telemetry));
        EXPECT_EQ(telemetry.mConstraintType, RagdollCore::ConstraintType::Hinge);
        EXPECT_FALSE(telemetry.mCurrentOrientation.has_value());
        EXPECT_FALSE(telemetry.mTargetOrientation.has_value());
        ASSERT_TRUE(telemetry.mCurrentHingeAngle.has_value());
        ASSERT_TRUE(telemetry.mRequestedHingeAngle.has_value());
        ASSERT_TRUE(telemetry.mTargetHingeAngle.has_value());
        ASSERT_TRUE(telemetry.mHingeMinimum.has_value());
        ASSERT_TRUE(telemetry.mHingeMaximum.has_value());
        EXPECT_NEAR(*telemetry.mCurrentHingeAngle, 0.0f, 1e-4f);
        EXPECT_NEAR(std::abs(*telemetry.mRequestedHingeAngle), 0.7f, 1e-4f);
        EXPECT_NEAR(std::abs(*telemetry.mTargetHingeAngle), 0.25f, 1e-4f);
        EXPECT_FLOAT_EQ(*telemetry.mHingeMinimum, -0.25f);
        EXPECT_FLOAT_EQ(*telemetry.mHingeMaximum, 0.25f);
        EXPECT_NEAR(telemetry.mAngularError, 0.25f, 1e-4f);
        EXPECT_NEAR(telemetry.mTargetClampDelta, 0.45f, 1e-4f);
    }

    TEST_F(PassiveRagdollTest, hingeTelemetryUsesNativeFramesAndRefreshesEachPoseRequest)
    {
        auto skeleton = RagdollCoreTest::makeSkeleton();
        skeleton.mBones[0].mModelTransform.mRotation = JPH::Quat::sEulerAngles(JPH::Vec3(0.3f, -0.4f, 0.2f));
        skeleton.mBones[1].mModelTransform.mRotation = (skeleton.mBones[0].mModelTransform.mRotation
            * JPH::Quat::sEulerAngles(JPH::Vec3(-0.2f, 0.25f, -0.1f))).Normalized();
        auto profile = RagdollCoreTest::makeProfile();
        auto& constraint = *profile.mBodies[1].mConstraint;
        constraint.mType = RagdollCore::ConstraintType::Hinge;
        const auto parentFrame = JPH::Quat::sEulerAngles(JPH::Vec3(0.1f, -0.2f, 0.3f));
        const auto childFrame = JPH::Quat::sEulerAngles(JPH::Vec3(-0.3f, 0.2f, -0.1f));
        constraint.mParentTwistAxis = parentFrame * JPH::Vec3::sAxisX();
        constraint.mParentPlaneAxis = parentFrame * JPH::Vec3::sAxisY();
        constraint.mChildTwistAxis = childFrame * JPH::Vec3::sAxisX();
        constraint.mChildPlaneAxis = childFrame * JPH::Vec3::sAxisY();
        constraint.mHingeMinAngle = -0.25f;
        constraint.mHingeMaxAngle = 0.4f;
        constraint.mMotor.mFrequency = 6.0f;
        constraint.mMotor.mDamping = 1.0f;
        constraint.mMotor.mMaximumTorque = 1000.0f;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        auto created = RagdollCore::createPassiveRagdoll(mPhysics, skeleton, profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        auto* joint = static_cast<JPH::HingeConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
        RagdollCore::RagdollPose pose;
        ASSERT_TRUE(ragdoll.captureModelPose(pose));
        // Align the public constraint frames to construct anatomical zero,
        // independently of the implementation's Jolt initial-frame helper.
        const auto zeroRelative = (joint->GetConstraintToBody1Matrix().GetQuaternion()
            * joint->GetConstraintToBody2Matrix().GetQuaternion().Conjugated()).Normalized();
        RagdollCore::MotorTargetTelemetry telemetry;
        for (const float angle : { -0.7f, -0.1f, 0.0f, 0.2f, 0.8f, -0.15f })
        {
            SCOPED_TRACE(angle);
            const auto relative = JPH::Quat::sRotation(joint->GetLocalSpaceHingeAxis1(), angle) * zeroRelative;
            pose.mBodyTransforms[1].mRotation = (pose.mBodyTransforms[0].mRotation * relative).Normalized();
            ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
            const float nativeTarget = joint->GetTargetAngle();
            for (int capture = 0; capture < 2; ++capture)
            {
                ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, telemetry));
                ASSERT_TRUE(telemetry.mRequestedHingeAngle && telemetry.mTargetHingeAngle);
                EXPECT_NEAR(*telemetry.mRequestedHingeAngle, angle, 1e-5f);
                EXPECT_NEAR(*telemetry.mTargetHingeAngle, std::clamp(angle, -0.25f, 0.4f), 1e-5f);
                EXPECT_NEAR(telemetry.mTargetClampDelta, std::abs(angle - nativeTarget), 1e-5f);
                EXPECT_FLOAT_EQ(joint->GetTargetAngle(), nativeTarget);
                EXPECT_FALSE(telemetry.mRequestedOrientation);
            }
        }
    }

    TEST_F(PassiveRagdollTest, poseHingeMeasurementPreservesNativeSignFramesAndReadOnlyState)
    {
        auto skeleton = RagdollCoreTest::makeSkeleton();
        skeleton.mBones[0].mModelTransform.mRotation = JPH::Quat::sEulerAngles(JPH::Vec3(0.3f, -0.4f, 0.2f));
        skeleton.mBones[1].mModelTransform.mRotation = (skeleton.mBones[0].mModelTransform.mRotation
            * JPH::Quat::sEulerAngles(JPH::Vec3(-0.2f, 0.25f, -0.1f))).Normalized();
        auto profile = RagdollCoreTest::makeProfile();
        auto& constraint = *profile.mBodies[1].mConstraint;
        constraint.mType = RagdollCore::ConstraintType::Hinge;
        const auto parentFrame = JPH::Quat::sEulerAngles(JPH::Vec3(0.1f, -0.2f, 0.3f));
        const auto childFrame = JPH::Quat::sEulerAngles(JPH::Vec3(-0.3f, 0.2f, -0.1f));
        constraint.mParentTwistAxis = parentFrame * JPH::Vec3::sAxisX();
        constraint.mParentPlaneAxis = parentFrame * JPH::Vec3::sAxisY();
        constraint.mChildTwistAxis = childFrame * JPH::Vec3::sAxisX();
        constraint.mChildPlaneAxis = childFrame * JPH::Vec3::sAxisY();
        constraint.mHingeMinAngle = -0.25f;
        constraint.mHingeMaxAngle = 0.4f;
        constraint.mMotor.mFrequency = 6.0f;
        constraint.mMotor.mDamping = 1.0f;
        constraint.mMotor.mMaximumTorque = 1000.0f;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mRootPosition = JPH::RVec3(150, -210, 330);
        options.mRootRotation = JPH::Quat::sEulerAngles(JPH::Vec3(-0.2f, 0.3f, 0.6f));
        options.mUniformScale = 1.7f;
        auto created = RagdollCore::createPassiveRagdoll(mPhysics, skeleton, profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem(JPH::EActivation::DontActivate));
        auto* joint = static_cast<JPH::HingeConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
        RagdollCore::RagdollPose pose;
        ASSERT_TRUE(ragdoll.captureModelPose(pose));
        // Construct known signed angles from independently exposed native
        // frames; neither bind orientation nor knee zero is assumed identity.
        const auto zeroRelative = (joint->GetConstraintToBody1Matrix().GetQuaternion()
            * joint->GetConstraintToBody2Matrix().GetQuaternion().Conjugated()).Normalized();
        const auto initialTarget = joint->GetTargetAngle();
        const auto initialAngle = joint->GetCurrentAngle();
        const auto initialMotor = joint->GetMotorState();
        for (const float angle : { -0.7f, -0.1f, 0.0f, 0.2f, 0.8f })
        {
            SCOPED_TRACE(angle);
            const auto relative = JPH::Quat::sRotation(joint->GetLocalSpaceHingeAxis1(), angle) * zeroRelative;
            pose.mBodyTransforms[1].mRotation = (pose.mBodyTransforms[0].mRotation * relative).Normalized();
            const auto measured = ragdoll.getPoseHingeAngle(pose, 1);
            ASSERT_TRUE(measured);
            EXPECT_NEAR(*measured, angle, 1e-5f);
            EXPECT_FALSE(ragdoll.isActive());
            EXPECT_EQ(joint->GetMotorState(), initialMotor);
            EXPECT_FLOAT_EQ(joint->GetTargetAngle(), initialTarget);
            EXPECT_FLOAT_EQ(joint->GetCurrentAngle(), initialAngle);
        }
        const auto measured = ragdoll.getPoseHingeAngle(pose, 1);
        ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
        RagdollCore::MotorTargetTelemetry target;
        ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, target));
        ASSERT_TRUE(target.mRequestedHingeAngle && target.mTargetHingeAngle);
        EXPECT_NEAR(*measured, *target.mRequestedHingeAngle, 1e-6f);
        EXPECT_NEAR(*target.mTargetHingeAngle, 0.4f, 1e-5f);
        // A reference-frame change preserves the same world pose and angle.
        const auto position = JPH::RVec3(-300, 100, 20);
        const auto rotation = JPH::Quat::sEulerAngles(JPH::Vec3(0.5f, 0.1f, -0.8f));
        ASSERT_TRUE(ragdoll.rebaseModelPose(pose, position, rotation));
        ASSERT_TRUE(ragdoll.setReferenceTransform(position, rotation));
        const auto rebased = ragdoll.getPoseHingeAngle(pose, 1);
        ASSERT_TRUE(rebased);
        EXPECT_NEAR(*rebased, *measured, 1e-5f);
        EXPECT_FLOAT_EQ(joint->GetTargetAngle(), *target.mTargetHingeAngle);
    }

    TEST_F(PassiveRagdollTest, poseHingeMeasurementKeepsUpstreamStagesSeparateFromLimitedMotor)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& constraint = *profile.mBodies[1].mConstraint;
        constraint.mType = RagdollCore::ConstraintType::Hinge;
        constraint.mHingeMinAngle = -0.25f;
        constraint.mHingeMaxAngle = 0.25f;
        constraint.mMotor.mFrequency = 6.0f;
        constraint.mMotor.mDamping = 1.0f;
        constraint.mMotor.mMaximumTorque = 1000.0f;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        auto* joint = static_cast<JPH::HingeConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
        RagdollCore::RagdollPose animation;
        ASSERT_TRUE(ragdoll.captureModelPose(animation));
        const auto zeroRelative = (joint->GetConstraintToBody1Matrix().GetQuaternion()
            * joint->GetConstraintToBody2Matrix().GetQuaternion().Conjugated()).Normalized();
        const auto setAngle = [&](RagdollCore::RagdollPose& pose, float angle) {
            pose.mBodyTransforms[1].mRotation = (pose.mBodyTransforms[0].mRotation
                * JPH::Quat::sRotation(joint->GetLocalSpaceHingeAxis1(), angle) * zeroRelative).Normalized();
        };
        setAngle(animation, -0.12f);
        auto composed = animation;
        setAngle(composed, 0.7f);
        ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(composed));
        ASSERT_TRUE(ragdoll.limitMotorTargetStep(1, 0.05f));
        const auto animationAngle = ragdoll.getPoseHingeAngle(animation, 1);
        const auto composedAngle = ragdoll.getPoseHingeAngle(composed, 1);
        ASSERT_TRUE(animationAngle && composedAngle);
        EXPECT_NEAR(*animationAngle, -0.12f, 1e-5f);
        EXPECT_NEAR(*composedAngle, 0.7f, 1e-5f);
        RagdollCore::MotorTargetTelemetry motor;
        ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, motor));
        ASSERT_TRUE(motor.mRequestedHingeAngle && motor.mTargetHingeAngle && motor.mCurrentHingeAngle);
        EXPECT_NEAR(*motor.mRequestedHingeAngle, *composedAngle, 1e-5f);
        EXPECT_NEAR(*motor.mTargetHingeAngle, 0.05f, 1e-5f);
        EXPECT_NEAR(*motor.mCurrentHingeAngle, 0.0f, 1e-5f);
        EXPECT_NEAR(motor.mTargetClampDelta, 0.65f, 1e-5f);
        EXPECT_FALSE(ragdoll.getPoseHingeAngle({}, 1));
        EXPECT_FALSE(ragdoll.getPoseHingeAngle(composed, 0));
        EXPECT_FALSE(ragdoll.getPoseHingeAngle(composed, ragdoll.getBodyCount()));
        auto invalid = composed;
        invalid.mBodyTransforms[1].mRotation = JPH::Quat(0, 0, 0, 0);
        EXPECT_FALSE(ragdoll.getPoseHingeAngle(invalid, 1));
        invalid = composed;
        invalid.mBodyTransforms[0].mTranslation.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(ragdoll.getPoseHingeAngle(invalid, 1));
        EXPECT_FLOAT_EQ(joint->GetTargetAngle(), *motor.mTargetHingeAngle);
        ragdoll.disableMotors();
        EXPECT_TRUE(ragdoll.getPoseHingeAngle(animation, 1));
        EXPECT_EQ(joint->GetMotorState(), JPH::EMotorState::Off);
    }


    TEST_F(PassiveRagdollTest, hingeTelemetryPreservesRequestThroughLimiterAndExplicitOverride)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& constraint = *profile.mBodies[1].mConstraint;
        constraint.mType = RagdollCore::ConstraintType::Hinge;
        constraint.mHingeMinAngle = -0.25f;
        constraint.mHingeMaxAngle = 0.25f;
        constraint.mMotor.mFrequency = 6.0f;
        constraint.mMotor.mDamping = 1.0f;
        constraint.mMotor.mMaximumTorque = 1000.0f;
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        auto created = RagdollCore::createPassiveRagdoll(
            mPhysics, RagdollCoreTest::makeSkeleton(), profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        auto* joint = static_cast<JPH::HingeConstraint*>(mPhysics.GetConstraints()[0].GetPtr());
        RagdollCore::RagdollPose pose;
        ASSERT_TRUE(ragdoll.captureModelPose(pose));
        ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
        ASSERT_TRUE(ragdoll.diagnosticOverrideMotorPose(1, JPH::Quat::sIdentity(), 0.7f, std::nullopt));
        ASSERT_TRUE(ragdoll.limitMotorTargetStep(1, 0.05f));
        RagdollCore::MotorTargetTelemetry telemetry;
        ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, telemetry));
        ASSERT_TRUE(telemetry.mRequestedHingeAngle && telemetry.mTargetHingeAngle);
        EXPECT_FLOAT_EQ(*telemetry.mRequestedHingeAngle, 0.7f);
        EXPECT_NEAR(*telemetry.mTargetHingeAngle, 0.05f, 1e-5f);
        EXPECT_NEAR(telemetry.mTargetClampDelta, 0.65f, 1e-5f);

        ASSERT_TRUE(ragdoll.diagnosticOverrideMotorPose(1, JPH::Quat::sIdentity(), -0.6f, std::nullopt));
        EXPECT_FALSE(ragdoll.diagnosticOverrideMotorPose(
            1, JPH::Quat::sIdentity(), std::numeric_limits<float>::quiet_NaN(), std::nullopt));
        ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, telemetry));
        ASSERT_TRUE(telemetry.mRequestedHingeAngle && telemetry.mTargetHingeAngle);
        EXPECT_FLOAT_EQ(*telemetry.mRequestedHingeAngle, -0.6f);
        EXPECT_FLOAT_EQ(*telemetry.mTargetHingeAngle, -0.25f);
        EXPECT_NEAR(telemetry.mTargetClampDelta, 0.35f, 1e-5f);

        ragdoll.disableMotors();
        // A raw native target has no recorded adapter request. Unknown must
        // not resurrect the previous explicit override after motor release.
        joint->SetMotorState(JPH::EMotorState::Position);
        joint->SetTargetAngle(0.1f);
        ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, telemetry));
        EXPECT_FALSE(telemetry.mRequestedHingeAngle);
        ASSERT_TRUE(telemetry.mTargetHingeAngle);
        EXPECT_FLOAT_EQ(*telemetry.mTargetHingeAngle, 0.1f);
        ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose));
        ASSERT_TRUE(ragdoll.captureMotorTargetTelemetry(1, telemetry));
        ASSERT_TRUE(telemetry.mRequestedHingeAngle && telemetry.mTargetHingeAngle);
        EXPECT_NEAR(*telemetry.mRequestedHingeAngle, 0.0f, 1e-5f);
        EXPECT_NEAR(*telemetry.mTargetHingeAngle, 0.0f, 1e-5f);
        EXPECT_NEAR(telemetry.mTargetClampDelta, 0.0f, 1e-5f);
    }

    TEST_F(PassiveRagdollTest, appliesValidatedBoundedUprightControlTransactionally)
    {
        auto created = create();
        ASSERT_TRUE(created.isValid());
        RagdollCore::ActiveControllerProfile profile;
        profile.mMaximumPelvisForce = 1000.0f;
        profile.mMaximumPelvisTorque = 1000.0f;
        profile.mMaximumSpineTorque = 1000.0f;
        RagdollCore::UprightControllerOutput control;
        control.mPelvisForce = JPH::Vec3(0.0f, 0.0f, 900.0f);
        control.mPelvisTorque = JPH::Vec3(100.0f, 0.0f, 0.0f);
        control.mSpineTorque = JPH::Vec3(0.0f, 100.0f, 0.0f);
        const std::array<std::size_t, 1> supportBodies{ 1 };

        EXPECT_FALSE(created.mRagdoll->applyUprightControl(control, 0, 1, supportBodies, supportBodies, profile));
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());
        EXPECT_FALSE(created.mRagdoll->applyUprightControl(control, 0, 2, supportBodies, supportBodies, profile));
        EXPECT_TRUE(mPhysics.GetBodyInterface().GetLinearVelocity(created.mRagdoll->getBodyId(0)).IsNearZero());

        control.mPelvisForce.SetZ(1001.0f);
        EXPECT_FALSE(created.mRagdoll->applyUprightControl(control, 0, 1, supportBodies, supportBodies, profile));
        mPhysics.SetGravity(JPH::Vec3::sZero());
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        ASSERT_EQ(mPhysics.Update(1.0f / 60.0f, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        EXPECT_TRUE(mPhysics.GetBodyInterface().GetLinearVelocity(created.mRagdoll->getBodyId(0)).IsNearZero());
        EXPECT_TRUE(mPhysics.GetBodyInterface().GetAngularVelocity(created.mRagdoll->getBodyId(0)).IsNearZero());
        EXPECT_TRUE(mPhysics.GetBodyInterface().GetAngularVelocity(created.mRagdoll->getBodyId(1)).IsNearZero());

        control.mPelvisForce.SetZ(900.0f);
        RagdollCore::UprightControllerOutput unsupportedControl = control;
        unsupportedControl.mPelvisTorque = JPH::Vec3::sZero();
        unsupportedControl.mSpineTorque = JPH::Vec3::sZero();
        ASSERT_TRUE(created.mRagdoll->applyUprightControl(unsupportedControl, 0, 1,
            std::span<const std::size_t>(), std::span<const std::size_t>(), profile));
        {
            JPH::BodyLockRead pelvisLock(
                mPhysics.GetBodyLockInterface(), created.mRagdoll->getBodyId(0));
            JPH::BodyLockRead torsoLock(
                mPhysics.GetBodyLockInterface(), created.mRagdoll->getBodyId(1));
            ASSERT_TRUE(pelvisLock.Succeeded());
            ASSERT_TRUE(torsoLock.Succeeded());
            EXPECT_TRUE(pelvisLock.GetBody().GetAccumulatedForce().IsNearZero());
            EXPECT_TRUE(torsoLock.GetBody().GetAccumulatedForce().IsNearZero());
        }

        const std::array<std::size_t, 1> invalidSupportBody{ 0 };
        EXPECT_FALSE(created.mRagdoll->applyUprightControl(
            control, 0, 1, invalidSupportBody, supportBodies, profile));
        ASSERT_TRUE(created.mRagdoll->applyUprightControl(
            control, 0, 1, supportBodies, supportBodies, profile, std::nullopt, true));
        {
            JPH::BodyLockRead pelvisLock(
                mPhysics.GetBodyLockInterface(), created.mRagdoll->getBodyId(0));
            JPH::BodyLockRead torsoLock(
                mPhysics.GetBodyLockInterface(), created.mRagdoll->getBodyId(1));
            ASSERT_TRUE(pelvisLock.Succeeded());
            ASSERT_TRUE(torsoLock.Succeeded());
            EXPECT_TRUE(pelvisLock.GetBody().GetAccumulatedForce().IsClose(JPH::Vec3(0.0f, 0.0f, 900.0f)));
            EXPECT_TRUE(torsoLock.GetBody().GetAccumulatedForce().IsClose(JPH::Vec3(0.0f, 0.0f, -900.0f)));
            EXPECT_TRUE(pelvisLock.GetBody().GetAccumulatedTorque().IsClose(control.mPelvisTorque))
                << "the translation actuator must not add an unrequested pelvis torque";
            const JPH::Vec3 lever(torsoLock.GetBody().GetCenterOfMassPosition()
                - pelvisLock.GetBody().GetCenterOfMassPosition());
            const JPH::Vec3 netMoment = pelvisLock.GetBody().GetAccumulatedTorque()
                + torsoLock.GetBody().GetAccumulatedTorque()
                + lever.Cross(torsoLock.GetBody().GetAccumulatedForce());
            EXPECT_TRUE(netMoment.IsClose(control.mPelvisTorque + control.mSpineTorque));
        }
        ASSERT_EQ(mPhysics.Update(1.0f / 60.0f, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        RagdollCore::RagdollTelemetry telemetry;
        ASSERT_TRUE(created.mRagdoll->captureTelemetry(telemetry));
        JPH::Vec3 totalMomentum = JPH::Vec3::sZero();
        for (const RagdollCore::BodyTelemetry& body : telemetry.mBodies)
            totalMomentum += body.mMotion.mLinearVelocity * body.mMass;
        EXPECT_TRUE(totalMomentum.IsNearZero(1e-4f));
        EXPECT_FALSE(mPhysics.GetBodyInterface().GetAngularVelocity(created.mRagdoll->getBodyId(0)).IsNearZero());
        EXPECT_FALSE(mPhysics.GetBodyInterface().GetAngularVelocity(created.mRagdoll->getBodyId(1)).IsNearZero());

        // Isolate the angular reaction assertion from the off-centre forces
        // applied above. Their common application point preserves zero net
        // wrench, but each body still receives an individual torque.
        control.mPelvisForce = JPH::Vec3::sZero();
        ASSERT_TRUE(created.mRagdoll->applyUprightControl(
            control, 0, 1, supportBodies, supportBodies, profile, 1));
        {
            JPH::BodyLockRead pelvisLock(
                mPhysics.GetBodyLockInterface(), created.mRagdoll->getBodyId(0));
            JPH::BodyLockRead reactionLock(
                mPhysics.GetBodyLockInterface(), created.mRagdoll->getBodyId(1));
            ASSERT_TRUE(pelvisLock.Succeeded());
            ASSERT_TRUE(reactionLock.Succeeded());
            EXPECT_TRUE(pelvisLock.GetBody().GetAccumulatedTorque().IsClose(control.mPelvisTorque));
            EXPECT_TRUE(reactionLock.GetBody().GetAccumulatedTorque().IsClose(-control.mPelvisTorque));
        }
        EXPECT_FALSE(created.mRagdoll->applyUprightControl(
            control, 0, 1, supportBodies, supportBodies, profile, 0));

        control.mSpineTorque.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(created.mRagdoll->applyUprightControl(
            control, 0, 1, supportBodies, supportBodies, profile));
    }

    TEST_F(PassiveRagdollTest, appliesExternalPelvisAssistWithoutInternalReaction)
    {
        auto created = create();
        ASSERT_TRUE(created.isValid());
        const JPH::Vec3 force(120.0f, -40.0f, 80.0f);
        const JPH::Vec3 torque(10.0f, 20.0f, -30.0f);
        EXPECT_FALSE(created.mRagdoll->applyExternalPelvisAssist(0, force, 200.0f, torque, 50.0f));
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());
        EXPECT_FALSE(created.mRagdoll->applyExternalPelvisAssist(2, force, 200.0f, torque, 50.0f));
        EXPECT_FALSE(created.mRagdoll->applyExternalPelvisAssist(0, force, 100.0f, torque, 50.0f));
        EXPECT_FALSE(created.mRagdoll->applyExternalPelvisAssist(0, force, 200.0f, torque, 20.0f));
        ASSERT_TRUE(created.mRagdoll->applyExternalPelvisAssist(0, force, 200.0f, torque, 50.0f));
        JPH::BodyLockRead pelvisLock(mPhysics.GetBodyLockInterface(), created.mRagdoll->getBodyId(0));
        JPH::BodyLockRead torsoLock(mPhysics.GetBodyLockInterface(), created.mRagdoll->getBodyId(1));
        ASSERT_TRUE(pelvisLock.Succeeded());
        ASSERT_TRUE(torsoLock.Succeeded());
        EXPECT_TRUE(pelvisLock.GetBody().GetAccumulatedForce().IsClose(force));
        EXPECT_TRUE(torsoLock.GetBody().GetAccumulatedForce().IsNearZero());
        EXPECT_TRUE(pelvisLock.GetBody().GetAccumulatedTorque().IsClose(torque));
        EXPECT_TRUE(torsoLock.GetBody().GetAccumulatedTorque().IsNearZero());
    }

    TEST_F(PassiveRagdollTest, sharesRecoveryAngularReactionWithoutAddingTorque)
    {
        using namespace RagdollCore;
        BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        auto created = createPassiveRagdoll(mPhysics, makeScenarioHumanoidSkeleton(),
            makeScenarioHumanoidProfile(), options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        UprightControllerOutput command;
        command.mPelvisTorque = JPH::Vec3(100, 0, 0);
        command.mSpineTorque = JPH::Vec3(0, 200, 0);
        const auto torque = [&](std::size_t index) {
            JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(index));
            return lock.GetBody().GetAccumulatedTorque();
        };
        ASSERT_TRUE(ragdoll.applyUprightControl(command, 0, 3, {}, {}, {}, 14, false, 17));
        EXPECT_TRUE(torque(0).IsClose(command.mPelvisTorque));
        EXPECT_TRUE(torque(3).IsClose(command.mSpineTorque));
        EXPECT_TRUE(torque(14).IsClose(JPH::Vec3(-50, -100, 0)));
        EXPECT_TRUE(torque(17).IsClose(JPH::Vec3(-50, -100, 0)));
        EXPECT_TRUE((torque(0) + torque(3) + torque(14) + torque(17)).IsNearZero());
        // Invalid secondary routes must fail before adding any primary torque.
        EXPECT_FALSE(ragdoll.applyUprightControl(command, 0, 3, {}, {}, {}, 14, false, 14));
        EXPECT_FALSE(ragdoll.applyUprightControl(command, 0, 3, {}, {}, {}, 14, false, 0));
        EXPECT_FALSE(ragdoll.applyUprightControl(command, 0, 3, {}, {}, {}, std::nullopt, false, 17));
        EXPECT_TRUE(torque(0).IsClose(command.mPelvisTorque));
        EXPECT_TRUE(torque(14).IsClose(JPH::Vec3(-50, -100, 0)));
        EXPECT_TRUE(torque(17).IsClose(JPH::Vec3(-50, -100, 0)));
    }

    TEST_F(PassiveRagdollTest, appliesBoundedInternalBodyForcePairWithoutNetMomentum)
    {
        auto created = create();
        ASSERT_TRUE(created.isValid());
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());
        mPhysics.SetGravity(JPH::Vec3::sZero());

        EXPECT_FALSE(created.mRagdoll->applyBodyForcePair(
            0, 1, JPH::Vec3(0.0f, 0.0f, 1001.0f), 1000.0f));
        ASSERT_TRUE(created.mRagdoll->applyBodyForcePair(
            0, 1, JPH::Vec3(0.0f, 0.0f, 900.0f), 1000.0f));

        for (std::size_t i = 0; i < 2; ++i)
        {
            JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), created.mRagdoll->getBodyId(i));
            ASSERT_TRUE(lock.Succeeded());
            EXPECT_TRUE(lock.GetBody().GetAccumulatedForce().IsClose(JPH::Vec3(0, 0, i == 0 ? 900.0f : -900.0f)));
        }

        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        ASSERT_EQ(mPhysics.Update(1.0f / 60.0f, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        RagdollCore::RagdollTelemetry telemetry;
        ASSERT_TRUE(created.mRagdoll->captureTelemetry(telemetry));
        ASSERT_EQ(telemetry.mBodies.size(), 2u);
        JPH::Vec3 totalMomentum = JPH::Vec3::sZero();
        for (const RagdollCore::BodyTelemetry& body : telemetry.mBodies)
            totalMomentum += body.mMotion.mLinearVelocity * body.mMass;
        EXPECT_TRUE(totalMomentum.IsNearZero(1e-4f));
        // Native joints may cancel either body's requested motion entirely;
        // conservation does not require a nonzero velocity on one body.
    }

    TEST_F(PassiveRagdollTest, appliesBoundedInternalAngularImpulsePair)
    {
        auto created = create();
        ASSERT_TRUE(created.isValid());
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());
        const JPH::Vec3 impulse(10.0f, 0.0f, 0.0f);

        EXPECT_FALSE(created.mRagdoll->applyBodyAngularImpulsePair(0, 1, impulse, 9.0f));
        EXPECT_TRUE(mPhysics.GetBodyInterface().GetAngularVelocity(
            created.mRagdoll->getBodyId(0)).IsNearZero());
        ASSERT_TRUE(created.mRagdoll->applyBodyAngularImpulsePair(0, 1, impulse, 10.0f));
        EXPECT_GT(mPhysics.GetBodyInterface().GetAngularVelocity(
            created.mRagdoll->getBodyId(0)).Dot(impulse), 0.0f);
        EXPECT_LT(mPhysics.GetBodyInterface().GetAngularVelocity(
            created.mRagdoll->getBodyId(1)).Dot(impulse), 0.0f);
    }

    TEST_F(PassiveRagdollTest, appliesBoundedInternalTorquePair)
    {
        auto created = create();
        ASSERT_TRUE(created.isValid());
        EXPECT_FALSE(created.mRagdoll->applyBodyTorquePair(
            0, 1, JPH::Vec3(11.0f, 0.0f, 0.0f), 10.0f));
        ASSERT_TRUE(created.mRagdoll->addToPhysicsSystem());
        const JPH::Vec3 torque(10.0f, 0.0f, 0.0f);
        ASSERT_TRUE(created.mRagdoll->applyBodyTorquePair(0, 1, torque, 10.0f));
        JPH::BodyLockRead drivenLock(mPhysics.GetBodyLockInterface(), created.mRagdoll->getBodyId(0));
        JPH::BodyLockRead reactionLock(mPhysics.GetBodyLockInterface(), created.mRagdoll->getBodyId(1));
        ASSERT_TRUE(drivenLock.Succeeded());
        ASSERT_TRUE(reactionLock.Succeeded());
        EXPECT_TRUE(drivenLock.GetBody().GetAccumulatedTorque().IsClose(torque));
        EXPECT_TRUE(reactionLock.GetBody().GetAccumulatedTorque().IsClose(-torque));
    }

    TEST(RagdollCoreImpactTest, targetsNearestInitialBodyAndBoundsDamageImpulse)
    {
        const auto skeleton = RagdollCoreTest::makeSkeleton();
        auto profile = RagdollCoreTest::makeProfile();
        profile.mImpactResponse.mImpulsePerDamage = 10.0f;
        profile.mImpactResponse.mMinimumImpulse = 50.0f;
        profile.mImpactResponse.mMaximumImpulse = 300.0f;
        profile.mImpactResponse.mUpwardBias = 0.25f;
        RagdollCore::BuildOptions options;
        options.mUniformScale = 2.0f;
        options.mRootPosition = JPH::RVec3(100.0f, 200.0f, 300.0f);

        const JPH::RVec3 hitPoint(100.0f, 219.0f, 300.0f);
        const auto impact
            = RagdollCore::makeKillingImpulse(skeleton, profile, options, hitPoint, JPH::Vec3::sAxisX(), 100.0f);
        ASSERT_TRUE(impact.has_value());
        EXPECT_EQ(impact->mBodyIndex, 1u);
        EXPECT_TRUE(impact->mWorldPoint.IsClose(hitPoint));
        EXPECT_NEAR(impact->mImpulse.Length(), 300.0f, 1e-3f);
        EXPECT_GT(impact->mImpulse.GetZ(), 0.0f);

        const auto minimum
            = RagdollCore::makeKillingImpulse(skeleton, profile, options, hitPoint, JPH::Vec3::sAxisX(), 1.0f);
        ASSERT_TRUE(minimum.has_value());
        EXPECT_NEAR(minimum->mImpulse.Length(), 50.0f, 1e-3f);
    }

    TEST(RagdollCoreImpactTest, rejectsNonPhysicalInput)
    {
        const auto skeleton = RagdollCoreTest::makeSkeleton();
        const auto profile = RagdollCoreTest::makeProfile();
        const RagdollCore::BuildOptions options;

        EXPECT_FALSE(RagdollCore::makeKillingImpulse(
            skeleton, profile, options, JPH::RVec3::sZero(), JPH::Vec3::sZero(), 10.0f));
        EXPECT_FALSE(RagdollCore::makeKillingImpulse(
            skeleton, profile, options, JPH::RVec3::sZero(), JPH::Vec3::sAxisX(), 0.0f));
    }

    TEST(RagdollCoreMotionTest, estimatesAndBoundsWorldSpaceBodyMotion)
    {
        RagdollCore::MotionPose previous;
        previous.mBodyTransforms = {
            { JPH::Vec3::sZero(), JPH::Quat::sIdentity() },
            { JPH::Vec3(0.0f, 1.0f, 0.0f), JPH::Quat::sIdentity() },
        };
        RagdollCore::MotionPose current = previous;
        current.mRootTransform.mTranslation = JPH::Vec3(10.0f, 0.0f, 0.0f);
        current.mBodyTransforms[1].mTranslation = JPH::Vec3(0.0f, 2.0f, 0.0f);
        current.mBodyTransforms[1].mRotation
            = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.5f);

        RagdollCore::MotionTransferProfile profile;
        profile.mMaximumSampleDuration = 1.0f;
        profile.mMaximumLinearSpeed = 100.0f;
        profile.mMaximumAngularSpeed = 10.0f;
        const auto result = RagdollCore::estimateBodyMotions(previous, current, 0.5f, profile);
        ASSERT_TRUE(result.has_value());
        ASSERT_EQ(result->size(), 2u);
        EXPECT_TRUE((*result)[0].mLinearVelocity.IsClose(JPH::Vec3(20.0f, 0.0f, 0.0f)));
        EXPECT_TRUE((*result)[1].mLinearVelocity.IsClose(JPH::Vec3(20.0f, 2.0f, 0.0f)));
        EXPECT_TRUE((*result)[0].mAngularVelocity.IsNearZero());
        EXPECT_TRUE((*result)[1].mAngularVelocity.IsClose(JPH::Vec3(0.0f, 0.0f, 1.0f)));

        profile.mMaximumLinearSpeed = 5.0f;
        profile.mMaximumAngularSpeed = 0.25f;
        const auto clamped = RagdollCore::estimateBodyMotions(previous, current, 0.5f, profile);
        ASSERT_TRUE(clamped.has_value());
        EXPECT_NEAR((*clamped)[1].mLinearVelocity.Length(), 5.0f, 1e-4f);
        EXPECT_NEAR((*clamped)[1].mAngularVelocity.Length(), 0.25f, 1e-4f);
    }

    TEST(RagdollCoreMotionTest, rejectsStaleOrMismatchedPoseHistory)
    {
        RagdollCore::MotionPose previous;
        previous.mBodyTransforms.resize(2);
        RagdollCore::MotionPose current = previous;
        RagdollCore::MotionTransferProfile profile;
        EXPECT_FALSE(RagdollCore::estimateBodyMotions(previous, current, 0.2f, profile));

        current.mBodyTransforms.pop_back();
        EXPECT_FALSE(RagdollCore::estimateBodyMotions(previous, current, 0.01f, profile));
    }

    TEST(RagdollCorePoseTest, interpolatesRenderPoseWithoutMutatingOutputOnInvalidInput)
    {
        RagdollCore::RagdollPose previous;
        previous.mBodyTransforms = {
            { JPH::Vec3::sZero(), JPH::Quat::sIdentity() },
            { JPH::Vec3(0.0f, 2.0f, 0.0f), JPH::Quat::sIdentity() },
        };
        RagdollCore::RagdollPose current = previous;
        current.mBodyTransforms[0].mTranslation = JPH::Vec3(10.0f, 4.0f, 2.0f);
        current.mBodyTransforms[0].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.0f);

        RagdollCore::RagdollPose result;
        ASSERT_TRUE(RagdollCore::interpolateRagdollPoses(previous, current, 0.5f, result));
        ASSERT_EQ(result.mBodyTransforms.size(), 2u);
        EXPECT_TRUE(result.mBodyTransforms[0].mTranslation.IsClose(JPH::Vec3(5.0f, 2.0f, 1.0f)));
        EXPECT_TRUE((result.mBodyTransforms[0].mRotation * JPH::Vec3::sAxisX())
                        .IsClose(JPH::Vec3(std::cos(0.5f), std::sin(0.5f), 0.0f), 1e-5f));
        EXPECT_TRUE(result.mBodyTransforms[1].mTranslation.IsClose(previous.mBodyTransforms[1].mTranslation));

        current.mBodyTransforms.pop_back();
        EXPECT_FALSE(RagdollCore::interpolateRagdollPoses(previous, current, 0.5f, result));
        EXPECT_EQ(result.mBodyTransforms.size(), 2u);
        EXPECT_FALSE(RagdollCore::interpolateRagdollPoses(previous, previous, -0.1f, result));
        EXPECT_FALSE(RagdollCore::interpolateRagdollPoses(previous, previous, 1.1f, result));
    }
    TEST_F(PassiveRagdollTest, distributedRootTorqueProducesThePredictedPhysicsResponse)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto skeleton = RagdollCoreTest::makeSkeleton();
        profile.mBodies.resize(4, profile.mBodies[1]);
        skeleton.mBones.resize(4);
        constexpr std::array<const char*, 4> names{ "pelvis", "left_thigh", "right_thigh", "spine" };
        profile.mLandmarks.mSpine = { "spine" };
        std::array<RagdollCore::JointTorqueRequest, 4> joints;
        for (std::size_t i = 0; i < 4; ++i)
        {
            auto& body = profile.mBodies[i];
            body.mName = body.mBone = names[i];
            body.mMassFraction = i == 0 ? 0.4f : 0.2f;
            body.mShape.mType = RagdollCore::ShapeType::Box;
            body.mShape.mHalfExtents = JPH::Vec3::sReplicate(0.5f);
            body.mSelfCollisionGroup = 1;
            body.mMaterial.mLinearDamping = body.mMaterial.mAngularDamping = 0;
            if (i != 0)
            {
                body.mParentBody = "pelvis";
                body.mConstraint->mMotor.mMaximumTorque = 100;
                joints[i].mParent = 0;
                joints[i].mMaximumAxisTorque = JPH::Vec3::sReplicate(100);
            }
            skeleton.mBones[i] = { names[i], i == 0 ? -1 : 0, {} };
        }
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sObjectLayer;
        options.mUseContinuousCollision = false;
        auto created = RagdollCore::createPassiveRagdoll(mPhysics, skeleton, profile, options, {}, 7, 1234);
        ASSERT_TRUE(created.isValid());
        auto& ragdoll = *created.mRagdoll;
        ASSERT_TRUE(ragdoll.addToPhysicsSystem());
        RagdollCore::RagdollPose pose;
        ASSERT_TRUE(ragdoll.captureModelPose(pose));
        const std::array<std::size_t, 3> disabled{ 1, 2, 3 };
        ASSERT_TRUE(ragdoll.driveToPoseUsingMotors(pose, disabled));
        ragdoll.beginActuationStep(false);
        joints[1].mTorque = JPH::Vec3(1, 0, 0);
        joints[2].mTorque = JPH::Vec3(0, 1, 0);
        joints[3].mTorque = JPH::Vec3(3, -2, 1);
        const JPH::Vec3 requested(10, 20, 30);
        const auto plan = RagdollCore::distributeRootTorque(joints, 1, 2, 1, 0, requested);
        ASSERT_TRUE(plan);
        ASSERT_FALSE(plan->mSaturated);
        constexpr float dt = 1.0f / 60.0f;
        JPH::Vec3 expected;
        {
            JPH::BodyLockRead lock(mPhysics.GetBodyLockInterface(), ragdoll.getBodyId(0));
            ASSERT_TRUE(lock.Succeeded());
            const auto& body = lock.GetBody();
            expected = body.GetMotionProperties()->MultiplyWorldSpaceInverseInertiaByVector(
                body.GetRotation(), requested * dt);
        }
        for (std::size_t i = 1; i < 4; ++i)
            ASSERT_TRUE(ragdoll.applyBodyTorquePair(i, 0, plan->mJointTorques[i],
                joints[i].mMaximumAxisTorque.Length()));
        mPhysics.SetGravity(JPH::Vec3::sZero());
        ASSERT_TRUE(ragdoll.beginSolverImpulseMeasurement(dt));
        JPH::TempAllocatorImpl allocator(1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        ASSERT_EQ(mPhysics.Update(dt, 1, &allocator, &jobs), JPH::EPhysicsUpdateError::None);
        const auto actual = mPhysics.GetBodyInterface().GetAngularVelocity(ragdoll.getBodyId(0));
        EXPECT_TRUE(actual.IsClose(expected, 1e-10f));
        RagdollCore::RagdollMomentum momentum;
        ASSERT_TRUE(ragdoll.captureMomentum(momentum));
        EXPECT_LT(momentum.mAngularMomentum.Length(), 1e-4f);
        EXPECT_LT(momentum.mLinearMomentum.Length(), 1e-4f);
        RagdollCore::RagdollSolverImpulse impulse;
        ASSERT_TRUE(ragdoll.endSolverImpulseMeasurement(dt, impulse));
        EXPECT_FALSE(impulse.mVelocityLimitTouched);
    }

}
