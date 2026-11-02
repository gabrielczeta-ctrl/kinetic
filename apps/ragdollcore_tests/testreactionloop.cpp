#include <gtest/gtest.h>

#include <ragdollcore/reactionpipeline.hpp>
#include <ragdollcore/scenario.hpp>
#include "../ragdollcore_walksearch/recoverypose.hpp"
#include "../ragdollcore_walksearch/hybridcatch.hpp"

#include <algorithm>
#include <limits>
#include <vector>

using namespace RagdollCore;

TEST(RagdollCoreRecoveryPreviewTest, ArticulatedBlendPreservesLengthsAndEndpoints)
{
    const auto skeleton = makeScenarioHumanoidSkeleton();
    const auto profile = makeScenarioHumanoidProfile();
    RagdollPose standing;
    for (const auto& body : profile.mBodies)
    {
        const auto bone = std::find_if(skeleton.mBones.begin(), skeleton.mBones.end(),
            [&](const auto& b) { return b.mName == body.mBone; });
        ASSERT_NE(bone, skeleton.mBones.end());
        standing.mBodyTransforms.push_back(bone->mModelTransform);
    }
    const auto rig = makeReactionPoseTargets(profile);
    const auto low = RecoveryPreview::crouch(standing, profile);
    auto fallen = standing;
    RecoveryPreview::rotateBranch(fallen, rig.mParents, 0, JPH::Vec3::sAxisY(), 1.8f);
    for (const float weight : { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f })
    {
        const auto pose = RecoveryPreview::blend(fallen, low, rig.mParents, weight);
        for (std::size_t b = 0; b < rig.mParents.size(); ++b)
        {
            const auto& transform = pose.mBodyTransforms[b];
            EXPECT_TRUE(transform.mRotation.IsNormalized());
            if (rig.mParents[b] >= 0)
            {
                const auto p = rig.mParents[b];
                EXPECT_NEAR((transform.mTranslation - pose.mBodyTransforms[p].mTranslation).Length(),
                    (standing.mBodyTransforms[b].mTranslation - standing.mBodyTransforms[p].mTranslation).Length(), 0.001f);
            }
            if (weight == 0.0f || weight == 1.0f)
            {
                const auto& expected = weight == 0.0f ? fallen : low;
                EXPECT_GT(std::abs(transform.mRotation.Dot(expected.mBodyTransforms[b].mRotation)), 0.99999f);
                EXPECT_LT((transform.mTranslation - expected.mBodyTransforms[b].mTranslation).Length(), 0.001f);
            }
        }
    }
}

TEST(RagdollCoreRecoveryPreviewTest, HeightInputRejectsMissingPoseAndInvalidBounds)
{
    const auto skeleton = makeScenarioHumanoidSkeleton();
    const auto profile = makeScenarioHumanoidProfile();
    RagdollPose pose;
    for (const auto& body : profile.mBodies)
        for (const auto& bone : skeleton.mBones)
            if (bone.mName == body.mBone) pose.mBodyTransforms.push_back(bone.mModelTransform);
    for (float height : { 0.0f, 0.149f, 1.01f, std::numeric_limits<float>::quiet_NaN(), 0.5f })
    {
        ScenarioScript script;
        script.mDurationSeconds = 0.05f;
        ScenarioInputKeyframe key;
        key.mRecoveryPelvisHeightScale = height;
        if (height != 0.5f)
        {
            key.mMotorTarget = pose;
            key.mRecoveryMotorTarget = pose;
        }
        script.mKeyframes.push_back(key);
        EXPECT_FALSE(runRagdollScenario(skeleton, profile, {}, script).completed());
    }
}

TEST(RagdollCoreRecoveryPreviewTest, HeightReferenceCannotChangeOrdinaryStanding)
{
    const auto skeleton = makeScenarioHumanoidSkeleton();
    const auto profile = makeScenarioHumanoidProfile();
    RagdollPose pose;
    for (const auto& body : profile.mBodies)
        for (const auto& bone : skeleton.mBones)
            if (bone.mName == body.mBone) pose.mBodyTransforms.push_back(bone.mModelTransform);
    ScenarioScript script;
    script.mDurationSeconds = 0.5f;
    ScenarioInputKeyframe key;
    key.mMotorTarget = pose;
    key.mRecoveryMotorTarget = pose;
    script.mKeyframes.push_back(key);
    const auto baseline = runRagdollScenario(skeleton, profile, {}, script);
    script.mKeyframes[0].mRecoveryPelvisHeightScale = 0.15f;
    const auto candidate = runRagdollScenario(skeleton, profile, {}, script);
    ASSERT_TRUE(baseline.completed());
    ASSERT_TRUE(candidate.completed());
    EXPECT_EQ(encodeScenarioTelemetry(baseline), encodeScenarioTelemetry(candidate));
}

namespace
{
    struct Episode
    {
        ScenarioResult mScenario;
        std::vector<ReactionOutputs> mReactions;
    };

    ReactionPipelineProfile enabledProfile()
    {
        ReactionPipelineProfile profile;
        profile.mEnabled = true;
        profile.mBehavior.mEnabled = true;
        return profile;
    }

    std::size_t bodyIndex(const RagdollProfile& profile, const std::string& name)
    {
        const auto found = std::find_if(profile.mBodies.begin(), profile.mBodies.end(),
            [&](const BodyProfile& body) { return body.mName == name; });
        return found == profile.mBodies.end() ? std::numeric_limits<std::size_t>::max()
                                              : static_cast<std::size_t>(found - profile.mBodies.begin());
    }

    RagdollPose bindPose(const SkeletonDescriptor& skeleton, const RagdollProfile& profile)
    {
        RagdollPose pose;
        pose.mBodyTransforms.reserve(profile.mBodies.size());
        for (const BodyProfile& body : profile.mBodies)
        {
            const auto bone = std::find_if(skeleton.mBones.begin(), skeleton.mBones.end(),
                [&](const SkeletonBone& candidate) { return candidate.mName == body.mBone; });
            if (bone == skeleton.mBones.end())
                return {};
            pose.mBodyTransforms.push_back(bone->mModelTransform);
        }
        return pose;
    }

    ReactionBodyMode reactionMode(const ScenarioFrame* previous)
    {
        if (!previous)
            return ReactionBodyMode::Standing;
        switch (previous->mController.mMode)
        {
            case ActiveControllerMode::GettingUp:
            case ActiveControllerMode::BlendBack:
                return ReactionBodyMode::Recovery;
            case ActiveControllerMode::Falling:
            case ActiveControllerMode::Fallen:
                return ReactionBodyMode::Passive;
            case ActiveControllerMode::Standing:
            case ActiveControllerMode::Bracing:
            case ActiveControllerMode::Stumbling:
                return previous->mRequestedHorizontalVelocity.IsNearZero()
                    ? ReactionBodyMode::Standing : ReactionBodyMode::Walking;
        }
        return ReactionBodyMode::Idle;
    }

    ReactionSupportState reactionSupport(const ScenarioFrame* previous)
    {
        ReactionSupportState support;
        if (!previous)
            return support;
        support.mLeftFootSupported = previous->mSupport.mLeftFootSupported;
        support.mRightFootSupported = previous->mSupport.mRightFootSupported;
        support.mBothFeetSupported = support.mLeftFootSupported && support.mRightFootSupported;
        support.mSupportEstablished = previous->mSupport.hasSupport();
        support.mCaptureMargin = previous->mSupport.mCenterOfMassDistance;
        return support;
    }

    Episode runEpisode(bool recoveryEnabled, bool kinematic = false, bool interruptRecovery = false)
    {
        const SkeletonDescriptor skeleton = makeScenarioHumanoidSkeleton();
        const RagdollProfile profile = makeScenarioHumanoidProfile();
        const RagdollPose standing = bindPose(skeleton, profile);
        const std::size_t impactBody = bodyIndex(profile, "spine2");
        const ReactionPoseTargets poseTargets{ bodyIndex(profile, "spine"), bodyIndex(profile, "neck"),
            bodyIndex(profile, "left_upper_arm"), bodyIndex(profile, "right_upper_arm") };

        ScenarioSettings settings;
        settings.mTriangleMeshGround = true;
        settings.mLocomotionController = ScenarioLocomotionController::Reference;
        ScenarioScript script;
        script.mDurationSeconds = 12.0f;

        ReactionPipelineState reactionState;
        const ReactionPipelineProfile reactionProfile = enabledProfile();
        Episode episode;
        bool interrupted = false;
        episode.mReactions.reserve(720);
        const ScenarioInputProvider provider = [&](std::uint64_t step,
                                                   const ScenarioFrame* previous)
            -> std::optional<ScenarioInputKeyframe> {
            constexpr std::uint64_t walkStep = 30;
            constexpr std::uint64_t impactStep = 150;
            constexpr std::uint64_t contactReceiptStep = impactStep + 1;
            ReactionContractInput input;
            input.mFramePhysicsStep = step + 1;
            input.mFixedStepDuration = settings.mFixedTimeStep;
            input.mMode = reactionMode(previous);
            input.mWalking = input.mMode == ReactionBodyMode::Walking;
            input.mMoving = input.mWalking;
            input.mSupport = reactionSupport(previous);
            input.mAnimationSampleValid = standing.mBodyTransforms.size() == profile.mBodies.size();
            input.mOwned.mRecoveryBrace = input.mMode == ReactionBodyMode::Recovery;
            // The provider observes the completed previous physics frame. Feed
            // the reaction contract one step after the external impulse so
            // AlreadyApplied truthfully means the body has received it.
            if (step == contactReceiptStep)
            {
                input.mContact.mValid = true;
                input.mContact.mContactId = 1;
                input.mContact.mAttackerActorIdentity = 2;
                input.mContact.mTargetActorIdentity = 1;
                input.mContact.mTargetRegistryGeneration = 1;
                input.mContact.mSwingIdentity = 1;
                input.mContact.mPhysicsStep = impactStep;
                input.mContact.mTargetBodyIndex = static_cast<std::uint32_t>(impactBody);
                input.mContact.mRegion = AnatomicalRegion::Chest;
                input.mImpulse.mProvenance = ImpulseProvenance::AppliedExternal;
                input.mImpulse.mMagnitude = 5200.0f;
                input.mImpulse.mAlreadyApplied = true;
            }
            const ReactionOutputs reaction = stepReactionPipeline(reactionState, input, reactionProfile);
            episode.mReactions.push_back(reaction);

            ScenarioInputKeyframe key;
            key.mStep = step;
            key.mRequestedHorizontalVelocity = step >= walkStep
                ? JPH::Vec3(0.0f, 30.0f * reaction.mSpeedScale, 0.0f) : JPH::Vec3::sZero();
            key.mStabilizationStrength = reaction.mStabilizationStrength;
            RagdollPose reacted = standing;
            applyReactionPose(standing.mBodyTransforms, reaction, poseTargets, reacted.mBodyTransforms);
            key.mMotorTarget = std::move(reacted);
            key.mRecoveryMotorTarget = standing;
            if (kinematic && recoveryEnabled)
            {
                key.mKinematicRecovery = true;
                key.mKinematicRecoveryTerminalPose = standing;
                if (interruptRecovery && !interrupted && previous && previous->mRootKinematic
                    && previous->mController.mMode == ActiveControllerMode::GettingUp
                    && previous->mController.mTimeInMode >= 0.3f)
                {
                    key.mPrePlanningImpulses.push_back({ step, 0, JPH::Vec3(500.0f, 0, 0), std::nullopt });
                    interrupted = true;
                }
            }
            if (step == impactStep)
                key.mImpulses.push_back({ step, impactBody, JPH::Vec3(5200.0f, 0.0f, 0.0f), std::nullopt });

            if (previous && previous->mController.mMode == ActiveControllerMode::Fallen)
                key.mRecoveryRequested = recoveryEnabled;
            if (previous && previous->mController.mMode == ActiveControllerMode::GettingUp
                && previous->mController.mTimeInMode >= 1.0f)
                key.mRecoveryAnimationFinished = recoveryEnabled;
            if (!recoveryEnabled && step >= impactStep)
            {
                key.mMotorsEnabled = false;
                key.mTranslationActuationEnabled = false;
                key.mSwingFootActuationEnabled = false;
                key.mUprightTorqueEnabled = false;
            }
            return key;
        };
        episode.mScenario = runRagdollScenario(skeleton, profile, settings, script, provider);
        return episode;
    }
}

TEST(RagdollCoreReactionLoopTest, WalkImpactCatchGetUpResume)
{
    const Episode episode = runEpisode(true);
    ASSERT_TRUE(episode.mScenario.completed())
        << (episode.mScenario.mErrors.empty() ? "" : episode.mScenario.mErrors.front());
    ASSERT_EQ(episode.mReactions.size(), episode.mScenario.mFrames.size());

    bool walkedBeforeImpact = false;
    bool sawPhysicalImpact = false;
    bool sawReducedBudget = false;
    bool sawScaledMotorBudget = false;
    bool sawCatchOrFall = false;
    bool sawFallen = false;
    bool sawGettingUp = false;
    bool sawBlendBack = false;
    bool resumedStanding = false;
    bool resumedWalking = false;
    std::uint64_t standingAgainStep = 0;
    const RagdollProfile authored = makeScenarioHumanoidProfile();
    const std::size_t spine = bodyIndex(authored, "spine");
    for (std::size_t i = 0; i < episode.mScenario.mFrames.size(); ++i)
    {
        const ScenarioFrame& frame = episode.mScenario.mFrames[i];
        const ReactionOutputs& reaction = episode.mReactions[i];
        if (frame.mStep < 150 && frame.mReferenceGait && frame.mGait.hasStance()
            && frame.mCenterOfMass.GetY() > 4.0)
            walkedBeforeImpact = true;
        if (frame.mStep == 150)
            sawPhysicalImpact = frame.mImpulseCount == 1;
        if (reaction.mStabilizationStrength < 0.999f
            && frame.mStabilizationStrength == reaction.mStabilizationStrength)
        {
            sawReducedBudget = true;
            if (spine < frame.mJointMotorTargets.size() && frame.mJointMotorTargets[spine]
                && frame.mJointMotorTargets[spine]->mDrive)
            {
                const float expected = authored.mBodies[spine].mConstraint->mMotor.mMaximumTorque
                    * reaction.mStabilizationStrength;
                EXPECT_NEAR(frame.mJointMotorTargets[spine]->mDrive->mMaximumTorque.GetX(), expected, 1e-3f);
                sawScaledMotorBudget = true;
            }
        }
        sawCatchOrFall = sawCatchOrFall || reaction.mPhase == ReactionPhase::Catch
            || reaction.mPhase == ReactionPhase::Fall
            || frame.mController.mMode == ActiveControllerMode::Falling;
        sawFallen = sawFallen || frame.mController.mMode == ActiveControllerMode::Fallen;
        sawGettingUp = sawGettingUp || frame.mController.mMode == ActiveControllerMode::GettingUp;
        sawBlendBack = sawBlendBack || frame.mController.mMode == ActiveControllerMode::BlendBack;
        if (sawBlendBack && frame.mController.mMode == ActiveControllerMode::Standing)
        {
            resumedStanding = true;
            if (standingAgainStep == 0)
                standingAgainStep = frame.mStep;
        }
        if (resumedStanding && frame.mStep > standingAgainStep + 30 && frame.mGait.hasStance()
            && !frame.mRequestedHorizontalVelocity.IsNearZero())
            resumedWalking = true;
    }
    EXPECT_TRUE(walkedBeforeImpact);
    EXPECT_TRUE(sawPhysicalImpact);
    EXPECT_TRUE(sawReducedBudget);
    EXPECT_TRUE(sawScaledMotorBudget);
    EXPECT_TRUE(sawCatchOrFall);
    EXPECT_TRUE(sawFallen);
    EXPECT_TRUE(sawGettingUp);
    EXPECT_TRUE(sawBlendBack);
    EXPECT_TRUE(resumedStanding);
    EXPECT_TRUE(resumedWalking);

    const Episode passive = runEpisode(false);
    ASSERT_TRUE(passive.mScenario.completed())
        << (passive.mScenario.mErrors.empty() ? "" : passive.mScenario.mErrors.front());
    bool passiveFell = false;
    bool passiveRecovered = false;
    for (const ScenarioFrame& frame : passive.mScenario.mFrames)
    {
        passiveFell = passiveFell || frame.mController.mMode == ActiveControllerMode::Fallen;
        if (passiveFell && (frame.mController.mMode == ActiveControllerMode::GettingUp
                || frame.mController.mMode == ActiveControllerMode::BlendBack))
            passiveRecovered = true;
    }
    EXPECT_TRUE(passiveFell);
    EXPECT_FALSE(passiveRecovered);
}

TEST(RagdollCoreKinematicRecoveryTest, OwnsOnlyRecoveryAndReleasesBeforeAnImpact)
{
    for (const bool interrupted : { false, true })
    {
        const auto episode = runEpisode(true, true, interrupted);
        ASSERT_TRUE(episode.mScenario.completed())
            << (episode.mScenario.mErrors.empty() ? "" : episode.mScenario.mErrors.front());
        bool owned = false, released = false, impactReleased = false;
        const auto& frames = episode.mScenario.mFrames;
        for (std::size_t i = 1; i < frames.size(); ++i)
        {
            const auto& frame = frames[i];
            const auto& previous = frames[i - 1];
            if (frame.mRootKinematic)
            {
                owned = true;
                EXPECT_TRUE(previous.mController.mMode == ActiveControllerMode::GettingUp
                    || previous.mController.mMode == ActiveControllerMode::BlendBack);
                EXPECT_TRUE(frame.mKinematicRootTarget.has_value());
                EXPECT_TRUE(frame.mControllerInput.mRootExternallyOwned);
                EXPECT_FALSE(frame.mUprightApplied);
                EXPECT_FALSE(frame.mMomentum.has_value());
                EXPECT_FALSE(frame.mSolverImpulse.has_value());
            }
            if (previous.mRootKinematic && !frame.mRootKinematic)
            {
                released = true;
                if (frame.mImpulseCount)
                {
                    impactReleased = true;
                    EXPECT_TRUE(frame.mControllerInput.mRecoveryInterrupted);
                    EXPECT_EQ(frame.mController.mMode, ActiveControllerMode::Falling);
                }
            }
        }
        EXPECT_TRUE(owned);
        EXPECT_TRUE(released);
        EXPECT_EQ(impactReleased, interrupted);
    }
}

TEST(RagdollCoreKinematicRecoveryTest, RequiresValidPoseAndDoesNotOwnStanding)
{
    const auto skeleton = makeScenarioHumanoidSkeleton();
    const auto profile = makeScenarioHumanoidProfile();
    const auto standing = bindPose(skeleton, profile);
    ScenarioScript script;
    script.mDurationSeconds = 0.5f;
    ScenarioInputKeyframe key;
    key.mMotorTarget = standing;
    key.mRecoveryMotorTarget = standing;
    script.mKeyframes.push_back(key);
    const auto baseline = runRagdollScenario(skeleton, profile, {}, script);
    script.mKeyframes[0].mKinematicRecovery = true;
    EXPECT_FALSE(runRagdollScenario(skeleton, profile, {}, script).completed());
    script.mKeyframes[0].mKinematicRecoveryTerminalPose = standing;
    const auto candidate = runRagdollScenario(skeleton, profile, {}, script);
    ASSERT_TRUE(baseline.completed());
    ASSERT_TRUE(candidate.completed());
    EXPECT_EQ(encodeScenarioTelemetry(baseline), encodeScenarioTelemetry(candidate));
    script.mKeyframes[0].mKinematicRecoveryTerminalPose->mBodyTransforms[0].mTranslation
        = JPH::Vec3(std::numeric_limits<float>::quiet_NaN(), 0, 0);
    EXPECT_FALSE(runRagdollScenario(skeleton, profile, {}, script).completed());
}

// Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreHybridCatchTest, DISABLED_AnimationDoesNotSpendPhysicalSupportGrace)
{
    auto profile = makeScenarioHumanoidProfile().mActiveController;
    profile.mLocomotionRoute = LocomotionRoute::Reference;
    profile.mLocomotionSupportGraceDuration = 0.3f;
    ActiveRagdollController controller;
    ActiveControllerInput input;
    ActiveControllerOutput output;
    input.mPelvisHeight = 70;
    input.mTargetHorizontalVelocity = JPH::Vec3(0,30,0);
    input.mCenterOfMassVelocity = input.mTargetHorizontalVelocity;
    input.mRootExternallyOwned = true;
    for (int i = 0; i < 120; ++i) ASSERT_TRUE(controller.step(input, 1.0f/60, profile, output));
    EXPECT_EQ(output.mMode, ActiveControllerMode::Standing);
    input.mRootExternallyOwned = false;
    for (int i = 0; i < 12; ++i) ASSERT_TRUE(controller.step(input, 1.0f/60, profile, output));
    EXPECT_EQ(output.mMode, ActiveControllerMode::Bracing);
    EXPECT_TRUE(output.mDriveJointMotors);
    for (int i = 0; i < 30; ++i) ASSERT_TRUE(controller.step(input, 1.0f/60, profile, output));
    EXPECT_EQ(output.mMode, ActiveControllerMode::Falling);
    EXPECT_FALSE(output.mDriveJointMotors); // The ordinary grace remains bounded.
}

TEST(RagdollCoreHybridCatchTest, ReturnNeedsSustainedMeasuredSupportAndPhysicalGait)
{
    WalkSearch::HybridCatch state;
    ScenarioFrame frame;
    frame.mPelvisUpDot = frame.mControllerInput.mTorsoUpDot = 1;
    frame.mSupport.mLeftFootSupported = frame.mSupport.mRightFootSupported = true;
    state.step(&frame, true, true, 1.0f / 60);
    EXPECT_EQ(state.mStage, 2);
    for (int i = 0; i < 120; ++i) state.step(&frame, false, true, 1.0f / 60);
    EXPECT_EQ(state.mStage, 2); // Supported time without PGC is not a catch.
    frame.mReferenceGait.emplace();
    frame.mSupport.mRightFootSupported = false;
    for (int i = 0; i < 120; ++i) state.step(&frame, false, true, 1.0f / 60);
    EXPECT_EQ(state.mStage, 2); // Acquisition waits for two feet.
    frame.mSupport.mRightFootSupported = true;
    frame.mCenterOfMassVelocity = JPH::Vec3(40,0,0);
    state.step(&frame, false, true, 1.0f / 60);
    EXPECT_EQ(state.mStableTime, 0);
    frame.mCenterOfMassVelocity = JPH::Vec3::sZero();
    for (int i = 0; i < 14; ++i) state.step(&frame, false, true, 1.0f / 60);
    EXPECT_EQ(state.mStage, 2);
    for (int i = 0; i < 3; ++i) state.step(&frame, false, true, 1.0f / 60);
    EXPECT_EQ(state.mStage, 3);
    state.step(&frame, true, true, 1.0f / 60);
    EXPECT_EQ(state.mStage, 2); // Another hit interrupts the return.
    frame.mController.mMode = ActiveControllerMode::Fallen;
    for (int i = 0; i < 180; ++i) state.step(&frame, false, true, 1.0f / 60);
    EXPECT_EQ(state.mStage, 2);
    frame.mController.mMode = ActiveControllerMode::Standing;
    for (int i = 0; i < 180; ++i) state.step(&frame, false, false, 1.0f / 60);
    EXPECT_EQ(state.mStage, 2); // Motors-off control cannot acquire a root.
}

TEST(RagdollCoreHybridCatchTest, KinematicWalkReleasesBeforeEachImpulseRouteWithoutResettingVelocity)
{
    const auto skeleton = makeScenarioHumanoidSkeleton();
    const auto profile = makeScenarioHumanoidProfile();
    const auto standing = bindPose(skeleton, profile);
    for (int impulseRoute = 0; impulseRoute < 3; ++impulseRoute)
    {
        ScenarioSettings settings;
        settings.mLocomotionController = ScenarioLocomotionController::Reference;
        ScenarioScript script;
        script.mDurationSeconds = 3;
        const ScenarioImpulse hit{ 100, 0, JPH::Vec3(300,0,0), std::nullopt };
        if (impulseRoute == 2) script.mImpulses.push_back(hit);
        const auto result = runRagdollScenario(skeleton, profile, settings, script,
            [&](std::uint64_t step, const ScenarioFrame*) -> std::optional<ScenarioInputKeyframe> {
                ScenarioInputKeyframe key;
                key.mStep = step;
                key.mMotorTarget = standing;
                key.mKinematicLocomotion = step < 100;
                key.mRequestedHorizontalVelocity = step >= 90 ? JPH::Vec3(0,10,0) : JPH::Vec3::sZero();
                if (step >= 90 && step < 100)
                    for (auto& body : key.mMotorTarget->mBodyTransforms)
                        body.mTranslation += JPH::Vec3(0, (step - 89) / 6.0f, 0);
                if (step == 100 && impulseRoute == 0) key.mImpulses.push_back(hit);
                if (step == 100 && impulseRoute == 1) key.mPrePlanningImpulses.push_back(hit);
                return key;
            });
        ASSERT_TRUE(result.completed()) << (result.mErrors.empty() ? "" : result.mErrors.front());
        ASSERT_TRUE(result.mFrames[99].mRootKinematic);
        EXPECT_FALSE(result.mFrames[100].mRootKinematic);
        EXPECT_EQ(result.mFrames[100].mImpulseCount, 1);
        ASSERT_TRUE(result.mFrames[100].mRootReleaseVelocityError.has_value());
        EXPECT_FLOAT_EQ(*result.mFrames[100].mRootReleaseVelocityError, 0);
        for (const auto& frame : result.mFrames)
            if (frame.mRootKinematic)
            {
                EXPECT_FALSE(frame.mReferenceGait.has_value());
                EXPECT_FALSE(frame.mUprightApplied);
                EXPECT_FALSE(frame.mMomentum.has_value());
            }
    }
}
