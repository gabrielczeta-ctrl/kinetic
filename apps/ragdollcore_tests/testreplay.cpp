#include "fixtures.hpp"

#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/replay.hpp>

// clang-format off
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/PhysicsSystem.h>
// clang-format on

#include <gtest/gtest.h>

#include <limits>

namespace
{
    constexpr JPH::ObjectLayer sReplayObjectLayer = 1;

    class ReplayBroadPhaseLayers final : public JPH::BroadPhaseLayerInterface
    {
    public:
        JPH::uint GetNumBroadPhaseLayers() const override { return 1; }
        JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer) const override { return JPH::BroadPhaseLayer(0); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
        const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override { return "replay"; }
#endif
    };

    class ReplayObjectVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter
    {
    public:
        bool ShouldCollide(JPH::ObjectLayer, JPH::BroadPhaseLayer) const override { return true; }
    };

    class ReplayObjectPairs final : public JPH::ObjectLayerPairFilter
    {
    public:
        bool ShouldCollide(JPH::ObjectLayer, JPH::ObjectLayer) const override { return true; }
    };

    std::uint64_t hashBytes(std::span<const std::byte> data)
    {
        std::uint64_t hash = 14695981039346656037ull;
        for (const std::byte value : data)
        {
            hash ^= std::to_integer<std::uint8_t>(value);
            hash *= 1099511628211ull;
        }
        return hash;
    }

    std::optional<RagdollCore::Replay> runDeterministicSimulation()
    {
        constexpr float fixedTimeStep = 1.0f / 60.0f;
        constexpr std::uint32_t collisionSteps = 1;
        constexpr std::uint32_t frameCount = 60;

        ReplayBroadPhaseLayers broadPhaseLayers;
        ReplayObjectVsBroadPhase objectVsBroadPhase;
        ReplayObjectPairs objectPairs;
        JPH::PhysicsSystem physics;
        physics.Init(128, 0, 128, 128, broadPhaseLayers, objectVsBroadPhase, objectPairs);
        physics.SetGravity(JPH::Vec3(0.0f, 0.0f, -627.1216f));
        JPH::TempAllocatorImpl allocator(4 * 1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);

        const auto profile = RagdollCoreTest::makeProfile();
        RagdollCore::BuildOptions options;
        options.mObjectLayer = sReplayObjectLayer;
        options.mRootPosition = JPH::RVec3(100.0f, 200.0f, 300.0f);
        RagdollCore::PassiveRagdollInitialState initialState;
        initialState.mBodyMotions = {
            { JPH::Vec3(15.0f, 2.0f, 0.0f), JPH::Vec3(0.0f, 0.1f, 0.0f) },
            { JPH::Vec3(10.0f, -1.0f, 4.0f), JPH::Vec3(0.2f, 0.0f, -0.1f) },
        };
        initialState.mKillingImpulse
            = RagdollCore::KillingImpulse{ 1, JPH::Vec3(80.0f, 20.0f, 10.0f), JPH::RVec3(100.0f, 210.0f, 300.0f) };
        auto created = RagdollCore::createPassiveRagdoll(
            physics, RagdollCoreTest::makeSkeleton(), profile, options, initialState, 7, 1234);
        if (!created.isValid() || !created.mRagdoll->addToPhysicsSystem())
            return std::nullopt;

        RagdollCore::Replay replay;
        replay.mMetadata.mSeed = 0x4f70656e4d575244ull;
        replay.mMetadata.mProfileHash = RagdollCore::hashProfile(profile);
        replay.mMetadata.mFixedTimeStep = fixedTimeStep;
        replay.mMetadata.mWorkerThreadCount = jobs.GetMaxConcurrency();
        replay.mMetadata.mCollisionSteps = collisionSteps;
        replay.mBodyCount = static_cast<std::uint32_t>(created.mRagdoll->getBodyCount());
        replay.mFrames.reserve(frameCount);

        for (std::uint64_t step = 0; step < frameCount; ++step)
        {
            if (physics.Update(fixedTimeStep, collisionSteps, &allocator, &jobs) != JPH::EPhysicsUpdateError::None)
                return std::nullopt;
            RagdollCore::RagdollPose pose;
            RagdollCore::RagdollTelemetry telemetry;
            if (!created.mRagdoll->captureModelPose(pose) || !created.mRagdoll->captureTelemetry(telemetry)
                || pose.mBodyTransforms.size() != replay.mBodyCount || telemetry.mBodies.size() != replay.mBodyCount)
                return std::nullopt;

            RagdollCore::ReplayFrame frame;
            frame.mStepIndex = step;
            frame.mBodies.reserve(replay.mBodyCount);
            for (std::size_t i = 0; i < replay.mBodyCount; ++i)
            {
                RagdollCore::ReplayBodyState body;
                body.mTransform = pose.mBodyTransforms[i];
                body.mLinearVelocity = telemetry.mBodies[i].mMotion.mLinearVelocity;
                body.mAngularVelocity = telemetry.mBodies[i].mMotion.mAngularVelocity;
                frame.mBodies.push_back(body);
            }
            replay.mFrames.push_back(std::move(frame));
        }
        return replay;
    }

    RagdollCore::Replay makeReplay()
    {
        RagdollCore::Replay replay;
        replay.mMetadata.mSeed = 0x0123456789abcdefull;
        replay.mMetadata.mProfileHash = RagdollCore::hashProfile(RagdollCoreTest::makeProfile());
        replay.mMetadata.mFixedTimeStep = 1.0f / 60.0f;
        replay.mMetadata.mWorkerThreadCount = 1;
        replay.mMetadata.mCollisionSteps = 1;
        replay.mInputCount = 2;
        replay.mControllerStateCount = 1;
        replay.mBodyCount = 2;

        for (std::uint64_t step = 0; step < 2; ++step)
        {
            RagdollCore::ReplayFrame frame;
            frame.mStepIndex = step;
            frame.mInputs = { static_cast<float>(step), -0.5f };
            frame.mControllerState = { 0.25f * static_cast<float>(step) };
            frame.mBodies.resize(2);
            frame.mBodies[0].mTransform.mTranslation = JPH::Vec3(static_cast<float>(step), 2.0f, 3.0f);
            frame.mBodies[0].mLinearVelocity = JPH::Vec3(4.0f, 5.0f, 6.0f);
            frame.mBodies[1].mTransform.mTranslation = JPH::Vec3(7.0f, 8.0f, 9.0f);
            frame.mBodies[1].mTransform.mRotation
                = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.1f * static_cast<float>(step));
            frame.mBodies[1].mAngularVelocity = JPH::Vec3(0.0f, 0.0f, 0.2f);
            replay.mFrames.push_back(std::move(frame));
        }
        return replay;
    }

    TEST(RagdollCoreReplayTest, convexHullPointCoordinatesCountAndOrderBindReplayIdentity)
    {
        auto profile = RagdollCoreTest::makeProfile();
        auto& shape = profile.mBodies[0].mShape;
        shape.mType = RagdollCore::ShapeType::ConvexHull;
        shape.mPoints = { JPH::Vec3(0, 0, 0), JPH::Vec3(2, 0, 0), JPH::Vec3(0, 3, 0), JPH::Vec3(0, 0, 4) };
        const auto hash = RagdollCore::hashProfile(profile);
        auto changed = profile;
        changed.mBodies[0].mShape.mPoints[2] += JPH::Vec3(0, 0.25f, 0);
        EXPECT_NE(hash, RagdollCore::hashProfile(changed));
        changed = profile;
        changed.mBodies[0].mShape.mPoints.push_back(JPH::Vec3::sZero());
        EXPECT_NE(hash, RagdollCore::hashProfile(changed));
        changed = profile;
        std::swap(changed.mBodies[0].mShape.mPoints[0], changed.mBodies[0].mShape.mPoints[1]);
        EXPECT_NE(hash, RagdollCore::hashProfile(changed));

        auto replay = makeReplay();
        replay.mMetadata.mProfileHash = hash;
        const auto encoded = RagdollCore::serializeReplay(replay);
        ASSERT_TRUE(encoded);
        const auto decoded = RagdollCore::deserializeReplay(*encoded);
        ASSERT_TRUE(decoded);
        EXPECT_EQ(decoded->mMetadata.mProfileHash, hash);
    }

    TEST(RagdollCoreReplayTest, profileHashIsStableAndContentSensitive)
    {
        const auto profile = RagdollCoreTest::makeProfile();
        // Independently compiled from the exact pre-hull 135564f750 profile,
        // fixture and hash algorithm; ConvexHull leaves old-shape hashes intact.
        EXPECT_EQ(RagdollCore::hashProfile(profile), 15182504287881083105ull);
        // Keep the 0.5 Hz variant as a separate content-sensitive baseline.
        auto historical = profile;
        historical.mActiveController.mCenterOfMassFrequency = 0.5f;
        EXPECT_EQ(RagdollCore::hashProfile(historical), 10445620769420252769ull);
        EXPECT_EQ(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(profile));

        auto changed = profile;
        changed.mPoseTransition.mDeathBlendDuration += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mBodies[1].mMaterial.mFriction += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mBodies[1].mConstraint->mMotor.mMaximumTorque = 1000.0f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mSupportMargin += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mMaximumPelvisTorque += 1.0f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mLocomotionRoute = RagdollCore::LocomotionRoute::Legacy;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mAnimationAssistHorizontalScale = 0.5f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mAnimationAssistVerticalScale = 0.5f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mMaximumAnimationAssistForce += 1.0f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mAnimationAssistUprightScale = 0.5f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mMaximumAnimationAssistTorque += 1.0f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mAnimationAssistYawReferenceMaximumRate += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mAnimationAssistYawReferenceMaximumAcceleration += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mAnimationAssistYawStiffness += 1.0f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mAnimationAssistYawDamping += 1.0f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mMaximumAnimationAssistYawTorque += 1.0f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mAnimationAssistSupportMotionFrequency += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mAnimationAssistSupportMotionDamping += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mAnimationAssistSupportPositionLeash += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mAnimationAssistContactConfirmationTicks += 1;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mAnimationAssistLoadTransferDuration += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mGait.mCapturePointBlend += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mGait.mMaximumTouchdownHorizontalDistance += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mGait.mMaximumLandingApproachVerticalDistance += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mGait.mMaximumTouchdownRelativeVerticalSpeed += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mGait.mSwingFootFrequency += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mGait.mPreLiftSwingUnloadFraction += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mGait.mMaximumSwingHipTargetSpeed += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mGait.mSwingHipWorldStiffness += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mMaximumLocomotionSpeed += 1.0f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mPelvisHeightSettleDuration += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mPelvisHeightSettleMaximumVerticalSpeed += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mActivationSupportGraceDuration += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mLocomotionSupportGraceDuration += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mFallingExitSupportDuration += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));

        changed = profile;
        changed.mActiveController.mFallingExitMaximumVerticalSpeed += 1.0f;
        EXPECT_NE(RagdollCore::hashProfile(profile), RagdollCore::hashProfile(changed));
    }

    TEST(RagdollCoreReplayTest, producesStablePortableBinaryAndRoundTrips)
    {
        const auto replay = makeReplay();
        const auto first = RagdollCore::serializeReplay(replay);
        const auto second = RagdollCore::serializeReplay(replay);
        ASSERT_TRUE(first.has_value());
        ASSERT_TRUE(second.has_value());
        EXPECT_EQ(*first, *second);
        EXPECT_EQ(first->size(), 304u);

        const auto decoded = RagdollCore::deserializeReplay(*first);
        ASSERT_TRUE(decoded.has_value());
        EXPECT_EQ(decoded->mMetadata.mSeed, replay.mMetadata.mSeed);
        EXPECT_EQ(decoded->mMetadata.mProfileHash, replay.mMetadata.mProfileHash);
        EXPECT_FLOAT_EQ(decoded->mMetadata.mFixedTimeStep, replay.mMetadata.mFixedTimeStep);
        ASSERT_EQ(decoded->mFrames.size(), 2u);
        EXPECT_EQ(decoded->mFrames[1].mStepIndex, 1u);
        EXPECT_TRUE(decoded->mFrames[1].mBodies[1].mTransform.mRotation.IsClose(
            replay.mFrames[1].mBodies[1].mTransform.mRotation));

        const auto roundTrip = RagdollCore::serializeReplay(*decoded);
        ASSERT_TRUE(roundTrip.has_value());
        EXPECT_EQ(*roundTrip, *first);
    }

    TEST(RagdollCoreReplayTest, decodesPoweredAnimationTargetsUsingSharedChannelLayout)
    {
        RagdollCore::Replay replay;
        replay.mBodyCount = 2;
        replay.mInputCount = RagdollCore::LegacyPoweredReplayGlobalInputChannels
            + replay.mBodyCount * RagdollCore::PoweredReplayPoseChannelsPerBody;
        RagdollCore::ReplayFrame frame;
        frame.mInputs = {
            12.0f,
            -34.0f,
            0.0f,
            1.0f,
            2.0f,
            3.0f,
            0.0f,
            0.0f,
            0.0f,
            1.0f,
            4.0f,
            5.0f,
            6.0f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
        };

        const auto input = RagdollCore::decodePoweredReplayInput(replay, frame);
        ASSERT_TRUE(input.has_value());
        EXPECT_EQ(input->mRequestedHorizontalVelocity, JPH::Vec3(12.0f, -34.0f, 0.0f));
        EXPECT_FALSE(input->mRawRequestedHorizontalVelocity);
        EXPECT_FALSE(input->mFacing);
        ASSERT_EQ(input->mAnimationTargetPose.size(), 2u);
        EXPECT_EQ(input->mAnimationTargetPose[0].mTranslation, JPH::Vec3(1.0f, 2.0f, 3.0f));
        EXPECT_TRUE(input->mAnimationTargetPose[0].mRotation.IsClose(JPH::Quat::sIdentity()));
        EXPECT_EQ(input->mAnimationTargetPose[1].mTranslation, JPH::Vec3(4.0f, 5.0f, 6.0f));
        EXPECT_TRUE(input->mAnimationTargetPose[1].mRotation.IsClose(JPH::Quat(0.0f, 0.0f, 1.0f, 0.0f)));

        // Extend the same old recording without moving or changing its poses.
        frame.mInputs.insert(frame.mInputs.begin() + 3, { 0, 128, 0, 1, 0, 0 });
        replay.mInputCount += 6;
        const auto extended = RagdollCore::decodePoweredReplayInput(replay, frame);
        ASSERT_TRUE(extended);
        EXPECT_EQ(*extended->mRawRequestedHorizontalVelocity, JPH::Vec3(0, 128, 0));
        EXPECT_EQ(*extended->mFacing, JPH::Vec3(1, 0, 0));
        EXPECT_EQ(extended->mAnimationTargetPose[1].mTranslation, input->mAnimationTargetPose[1].mTranslation);
        replay.mFrames = { frame };
        replay.mFrames.front().mBodies.resize(replay.mBodyCount);
        const auto bytes = RagdollCore::serializeReplay(replay);
        ASSERT_TRUE(bytes);
        const auto roundTrip = RagdollCore::deserializeReplay(*bytes);
        ASSERT_TRUE(roundTrip);
        const auto roundTripInput = RagdollCore::decodePoweredReplayInput(*roundTrip, roundTrip->mFrames.front());
        ASSERT_TRUE(roundTripInput);
        EXPECT_EQ(roundTripInput->mFacing, extended->mFacing);
        EXPECT_EQ(roundTripInput->mRawRequestedHorizontalVelocity, extended->mRawRequestedHorizontalVelocity);
        EXPECT_FALSE(extended->mRecoveryAnimationFinished.has_value());
        frame.mInputs.insert(frame.mInputs.begin() + 9, 1.0f);
        ++replay.mInputCount;
        const auto recovery = RagdollCore::decodePoweredReplayInput(replay, frame);
        ASSERT_TRUE(recovery);
        ASSERT_TRUE(recovery->mRecoveryAnimationFinished.has_value());
        EXPECT_TRUE(*recovery->mRecoveryAnimationFinished);
        replay.mFrames.front().mInputs = frame.mInputs;
        const auto recoveryBytes = RagdollCore::serializeReplay(replay);
        ASSERT_TRUE(recoveryBytes);
        const auto recoveryReplay = RagdollCore::deserializeReplay(*recoveryBytes);
        ASSERT_TRUE(recoveryReplay);
        const auto recoveryRoundTrip = RagdollCore::decodePoweredReplayInput(
            *recoveryReplay, recoveryReplay->mFrames.front());
        ASSERT_TRUE(recoveryRoundTrip);
        EXPECT_EQ(recoveryRoundTrip->mRecoveryAnimationFinished, recovery->mRecoveryAnimationFinished);
        EXPECT_EQ(recovery->mAnimationTargetPose[1].mTranslation, input->mAnimationTargetPose[1].mTranslation);
        frame.mInputs[9] = 0.0f;
        EXPECT_FALSE(*RagdollCore::decodePoweredReplayInput(replay, frame)->mRecoveryAnimationFinished);
        frame.mInputs[9] = 0.5f;
        EXPECT_FALSE(RagdollCore::decodePoweredReplayInput(replay, frame));
        frame.mInputs[9] = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::decodePoweredReplayInput(replay, frame));
        frame.mInputs[9] = 0.0f;
        frame.mInputs[6] = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::decodePoweredReplayInput(replay, frame));
        frame.mInputs[6] = 1;
        frame.mInputs.pop_back();
        EXPECT_FALSE(RagdollCore::decodePoweredReplayInput(replay, frame).has_value());

        frame.mInputs.push_back(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(RagdollCore::decodePoweredReplayInput(replay, frame).has_value());
    }

    TEST(RagdollCoreReplayTest, rejectsInvalidStateAndCorruptBinary)
    {
        auto replay = makeReplay();
        replay.mFrames[1].mStepIndex = 3;
        EXPECT_FALSE(RagdollCore::serializeReplay(replay).has_value());

        replay = makeReplay();
        replay.mFrames[0].mControllerState[0] = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(RagdollCore::serializeReplay(replay).has_value());

        replay = makeReplay();
        auto binary = *RagdollCore::serializeReplay(replay);
        binary.pop_back();
        EXPECT_FALSE(RagdollCore::deserializeReplay(binary).has_value());

        binary = *RagdollCore::serializeReplay(replay);
        binary.push_back(std::byte{ 0 });
        EXPECT_FALSE(RagdollCore::deserializeReplay(binary).has_value());

        binary[0] = std::byte{ 0 };
        EXPECT_FALSE(RagdollCore::deserializeReplay(binary).has_value());
    }

    TEST(RagdollCoreReplayTest, comparesExactEncodingAndQuantifiesSemanticDivergence)
    {
        const auto expected = makeReplay();
        auto actual = expected;
        EXPECT_TRUE(RagdollCore::haveIdenticalReplayEncoding(expected, actual));
        EXPECT_TRUE(RagdollCore::compareReplays(expected, actual).isMatch());

        auto signEquivalent = expected;
        JPH::Quat& rotation = signEquivalent.mFrames[1].mBodies[1].mTransform.mRotation;
        rotation = JPH::Quat(-rotation.GetX(), -rotation.GetY(), -rotation.GetZ(), -rotation.GetW());
        EXPECT_FALSE(RagdollCore::haveIdenticalReplayEncoding(expected, signEquivalent));
        EXPECT_TRUE(RagdollCore::compareReplays(expected, signEquivalent).isMatch());

        actual.mFrames[0].mInputs[0] += 0.25f;
        actual.mFrames[0].mBodies[0].mTransform.mTranslation += JPH::Vec3(0.0f, 0.5f, 0.0f);
        actual.mFrames[1].mBodies[1].mLinearVelocity += JPH::Vec3(0.0f, 0.0f, 0.75f);
        EXPECT_FALSE(RagdollCore::haveIdenticalReplayEncoding(expected, actual));
        const auto comparison = RagdollCore::compareReplays(expected, actual);
        EXPECT_EQ(comparison.mStatus, RagdollCore::ReplayComparisonStatus::Diverged);
        ASSERT_EQ(comparison.mFirstDivergentStep, 0u);
        EXPECT_FLOAT_EQ(comparison.mMaximumInputError, 0.25f);
        EXPECT_FLOAT_EQ(comparison.mMaximumTranslationError, 0.5f);
        EXPECT_FLOAT_EQ(comparison.mMaximumLinearVelocityError, 0.75f);

        RagdollCore::ReplayComparisonTolerances tolerances;
        tolerances.mInput = 0.25f;
        tolerances.mTranslation = 0.5f;
        tolerances.mLinearVelocity = 0.75f;
        EXPECT_TRUE(RagdollCore::compareReplays(expected, actual, tolerances).isMatch());
    }

    TEST(RagdollCoreReplayTest, poweredDiscreteDifferencesCannotBeHiddenByContinuousTolerance)
    {
        auto expected = makeReplay();
        expected.mControllerStateCount = RagdollCore::PoweredReplayControllerStateChannels;
        for (auto& frame : expected.mFrames)
            frame.mControllerState.assign(expected.mControllerStateCount, 0.0f);
        auto actual = expected;
        RagdollCore::ReplayComparisonTolerances tolerance;
        tolerance.mExactPoweredDiscreteChannels = true;
        tolerance.mControllerState = 2.0f;
        using Channel = RagdollCore::PoweredReplayControllerChannel;
        for (const auto channel : { Channel::TimeInMode, Channel::TimeInGaitPhase, Channel::GaitPhaseProgress })
            actual.mFrames[0].mControllerState[static_cast<std::size_t>(channel)] = 0.01f;
        EXPECT_TRUE(RagdollCore::compareReplays(expected, actual, tolerance).isMatch());
        for (const auto channel : { Channel::Mode, Channel::GaitPhase, Channel::SwingFootLifted,
                 Channel::LeftFootSupported, Channel::RightFootSupported, Channel::CenterOfMassSupported,
                 Channel::GaitPhaseTimedOut })
        {
            auto changed = actual;
            const auto index = static_cast<std::size_t>(channel);
            changed.mFrames[0].mControllerState[index] = 1.0f;
            // Input mismatch in the same step must not obscure the discrete one.
            changed.mFrames[0].mInputs[0] += 0.1f;
            const auto result = RagdollCore::compareReplays(expected, changed, tolerance);
            EXPECT_EQ(result.mStatus, RagdollCore::ReplayComparisonStatus::Diverged);
            EXPECT_EQ(result.mFirstDivergentStep, 0u);
            EXPECT_EQ(result.mFirstDivergentField, RagdollCore::ReplayDifferenceField::ControllerState);
            EXPECT_EQ(result.mFirstDivergentIndex, index);
            EXPECT_EQ(result.mFirstDiscreteDivergentStep, 0u);
            EXPECT_EQ(result.mFirstDiscreteDivergentChannel, index);
        }
    }

    TEST(RagdollCoreReplayTest, continuousSwingLayoutKeepsIdentityDiscreteAndMotionTolerant)
    {
        using namespace RagdollCore;
        auto expected = makeReplay();
        expected.mControllerStateCount = ContinuousSwingReplayControllerStateChannels;
        for (auto& frame : expected.mFrames)
            frame.mControllerState.assign(expected.mControllerStateCount, 0.0f);
        auto actual = expected;
        ReplayComparisonTolerances tolerance;
        tolerance.mExactPoweredDiscreteChannels = true;
        tolerance.mControllerState = 0.1f;
        actual.mFrames[0].mControllerState[PoweredReplayControllerStateChannels + 4] = 0.01f;
        EXPECT_EQ(compareReplays(expected, actual, tolerance).mStatus, ReplayComparisonStatus::Match);
        actual.mFrames[0].mControllerState[PoweredReplayControllerStateChannels] = 0.01f;
        const auto compared = compareReplays(expected, actual, tolerance);
        EXPECT_EQ(compared.mStatus, ReplayComparisonStatus::Diverged);
        EXPECT_EQ(compared.mFirstDiscreteDivergentChannel, PoweredReplayControllerStateChannels);
    }

    TEST(RagdollCoreReplayTest, keepsEarlierPhysicalDivergenceAndLaterDiscreteBoundary)
    {
        auto expected = makeReplay();
        expected.mControllerStateCount = RagdollCore::PoweredReplayControllerStateChannels;
        for (auto& frame : expected.mFrames)
            frame.mControllerState.assign(expected.mControllerStateCount, 0.0f);
        auto actual = expected;
        actual.mFrames[0].mBodies[1].mLinearVelocity += JPH::Vec3::sAxisX();
        actual.mFrames[1].mControllerState[0] = 1.0f;
        RagdollCore::ReplayComparisonTolerances tolerance;
        tolerance.mExactPoweredDiscreteChannels = true;
        const auto result = RagdollCore::compareReplays(expected, actual, tolerance);
        EXPECT_EQ(result.mFirstDivergentStep, 0u);
        EXPECT_EQ(result.mFirstDivergentField, RagdollCore::ReplayDifferenceField::LinearVelocity);
        EXPECT_EQ(result.mFirstDivergentIndex, 1u);
        EXPECT_EQ(result.mFirstDiscreteDivergentStep, 1u);
        EXPECT_EQ(result.mFirstDiscreteDivergentChannel, 0u);
        EXPECT_FLOAT_EQ(result.mMaximumLinearVelocityError, 1.0f);
        EXPECT_EQ(RagdollCore::compareReplays(makeReplay(), makeReplay(), tolerance).mStatus,
            RagdollCore::ReplayComparisonStatus::InvalidTolerances);
    }

    TEST(RagdollCoreReplayTest, rejectsIncompatibleReplayComparisons)
    {
        const auto expected = makeReplay();

        auto actual = expected;
        ++actual.mMetadata.mSeed;
        EXPECT_EQ(RagdollCore::compareReplays(expected, actual).mStatus,
            RagdollCore::ReplayComparisonStatus::MetadataMismatch);

        actual = expected;
        actual.mBodyCount = 1;
        for (auto& frame : actual.mFrames)
            frame.mBodies.resize(1);
        EXPECT_EQ(RagdollCore::compareReplays(expected, actual).mStatus,
            RagdollCore::ReplayComparisonStatus::DimensionsMismatch);

        actual = expected;
        actual.mFrames.pop_back();
        EXPECT_EQ(RagdollCore::compareReplays(expected, actual).mStatus,
            RagdollCore::ReplayComparisonStatus::FrameCountMismatch);

        RagdollCore::ReplayComparisonTolerances tolerances;
        tolerances.mTranslation = -1.0f;
        EXPECT_EQ(RagdollCore::compareReplays(expected, expected, tolerances).mStatus,
            RagdollCore::ReplayComparisonStatus::InvalidTolerances);
    }

    TEST(RagdollCoreReplayTest, repeatsFixedStepSingleThreadedSimulationBitForBit)
    {
        const auto expected = runDeterministicSimulation();
        const auto actual = runDeterministicSimulation();
        ASSERT_TRUE(expected.has_value());
        ASSERT_TRUE(actual.has_value());
        EXPECT_EQ(expected->mMetadata.mWorkerThreadCount, 1u);
        EXPECT_EQ(expected->mMetadata.mCollisionSteps, 1u);
        EXPECT_FLOAT_EQ(expected->mMetadata.mFixedTimeStep, 1.0f / 60.0f);
        EXPECT_EQ(expected->mMetadata.mSeed, 0x4f70656e4d575244ull);
        EXPECT_EQ(expected->mBodyCount, 2u);
        EXPECT_EQ(expected->mFrames.size(), 60u);
        EXPECT_TRUE(RagdollCore::haveIdenticalReplayEncoding(*expected, *actual));
        EXPECT_TRUE(RagdollCore::compareReplays(*expected, *actual).isMatch());

        const auto binary = RagdollCore::serializeReplay(*expected);
        ASSERT_TRUE(binary.has_value());
        EXPECT_EQ(binary->size(), 6776u);
        // Authored masses/shape inertia and passive solver settings applied
        // before the first impact. The portable replay layout is unchanged.
        EXPECT_EQ(hashBytes(*binary), 15184897664667125350ull);
        // The independently compiled pre-hull profile hash already differs
        // from the former golden. Binary comparison proved only header bytes
        // 20..27 differ. Retain both old full-file goldens by substituting only
        // their profile metadata, so no passive trajectory change is hidden.
        auto historical = *expected;
        historical.mMetadata.mProfileHash = 16011936253135065062ull;
        const auto formerDefaultBinary = RagdollCore::serializeReplay(historical);
        ASSERT_TRUE(formerDefaultBinary);
        EXPECT_EQ(hashBytes(*formerDefaultBinary), 11498972955689166952ull);
        historical.mMetadata.mProfileHash = 8385351487244885245ull;
        const auto historicalBinary = RagdollCore::serializeReplay(historical);
        ASSERT_TRUE(historicalBinary);
        EXPECT_EQ(hashBytes(*historicalBinary), 14499053398726705243ull);
    }
}
