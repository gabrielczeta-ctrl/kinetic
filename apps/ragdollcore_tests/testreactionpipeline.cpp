#include <gtest/gtest.h>

#include <ragdollcore/reactioncontract.hpp>
#include <ragdollcore/reactionpipeline.hpp>
#include <ragdollcore/scenario.hpp>

#include <cmath>
#include <limits>
#include <vector>

using namespace RagdollCore;

namespace
{
    ReactionContractInput validInput()
    {
        ReactionContractInput input;
        input.mFramePhysicsStep = 100;
        input.mFixedStepDuration = 1.0f / 60.0f;
        input.mContact.mContactId = 7;
        input.mContact.mValid = true;
        input.mContact.mRegion = AnatomicalRegion::Chest;
        input.mContact.mTargetBodyIndex = 3;
        input.mImpulse.mProvenance = ImpulseProvenance::SolverMeasured;
        input.mImpulse.mMagnitude = 1500.0f;
        input.mImpulse.mAlreadyApplied = true;
        input.mMode = ReactionBodyMode::Standing;
        input.mSupport.mSupportEstablished = true;
        input.mSupport.mBothFeetSupported = true;
        return input;
    }

    ReactionPipelineProfile enabledProfile()
    {
        ReactionPipelineProfile profile;
        profile.mEnabled = true;
        profile.mBehavior.mEnabled = true;
        return profile;
    }
}

TEST(RagdollCoreReactionContractTest, AcceptsWellFormedInput)
{
    EXPECT_TRUE(validContractInput(validInput()));
}

TEST(RagdollCoreReactionContractTest, AbsentImpulseIsDistinctFromZero)
{
    ReactionContractInput absent = validInput();
    absent.mImpulse = ReactionImpulse{};
    EXPECT_EQ(absent.mImpulse.mProvenance, ImpulseProvenance::None);
    EXPECT_TRUE(validContractInput(absent));
    ReactionContractInput zero = validInput();
    zero.mImpulse.mMagnitude = 0.0f;
    EXPECT_TRUE(validContractInput(zero));
}

TEST(RagdollCoreReactionContractTest, RejectsInvalidValues)
{
    ReactionContractInput bad = validInput();
    bad.mFixedStepDuration = -1.0f;
    EXPECT_FALSE(validContractInput(bad));

    bad = validInput();
    bad.mImpulse.mMagnitude = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(validContractInput(bad));

    bad = validInput();
    bad.mImpulse.mMagnitude = -5.0f;
    EXPECT_FALSE(validContractInput(bad));

    bad = validInput();
    bad.mSupport.mCaptureMargin = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(validContractInput(bad));

    bad = validInput();
    bad.mReceipt = ReactionReceipt{ 1, -3.0f, 0.0f };
    EXPECT_FALSE(validContractInput(bad));

    bad = validInput();
    bad.mWoundSeverity = 1.5f;
    EXPECT_FALSE(validContractInput(bad));

    bad = validInput();
    bad.mContact.mTargetBodyIndex = std::numeric_limits<std::uint32_t>::max();
    bad.mContact.mRegion = AnatomicalRegion::Unknown;
    EXPECT_FALSE(validContractInput(bad));
}

TEST(RagdollCoreReactionPipelineTest, DisabledPipelineLeavesClipAndBudgetUntouched)
{
    ReactionPipelineState state;
    ReactionPipelineProfile profile;
    const ReactionOutputs outputs = stepReactionPipeline(state, validInput(), profile);
    EXPECT_FLOAT_EQ(outputs.mPoseCorrectionAuthority, 0.0f);
    EXPECT_FLOAT_EQ(outputs.mStabilizationStrength, 1.0f);
    EXPECT_EQ(outputs.mBehavior, BehaviorId::None);
    EXPECT_EQ(outputs.mPhase, ReactionPhase::Idle);
}

TEST(RagdollCoreReactionPipelineTest, DisturbanceRaisesPoseAuthorityAndBehavior)
{
    ReactionPipelineState state;
    const ReactionPipelineProfile profile = enabledProfile();
    // Settle at the standing baseline first.
    ReactionContractInput idle = validInput();
    idle.mContact = ReactionContact{};
    idle.mImpulse = ReactionImpulse{};
    ReactionOutputs baseline;
    for (int i = 0; i < 120; ++i)
    {
        idle.mFramePhysicsStep += 1;
        baseline = stepReactionPipeline(state, idle, profile);
    }
    const float standingAuthority = baseline.mPoseCorrectionAuthority;
    EXPECT_GT(standingAuthority, 0.0f);
    // A strong hit raises the authority above the standing baseline and
    // triggers the behavior.
    ReactionContractInput input = validInput();
    input.mFramePhysicsStep = idle.mFramePhysicsStep;
    input.mContact.mRegion = AnatomicalRegion::Head;
    input.mImpulse.mMagnitude = 4000.0f; // above the balance disturbance ramp so authority rises
    ReactionOutputs hit;
    float peakAuthority = 0.0f;
    float minimumStabilization = 1.0f;
    for (int i = 0; i < 6; ++i)
    {
        input.mFramePhysicsStep += 1;
        hit = stepReactionPipeline(state, input, profile);
        peakAuthority = std::max(peakAuthority, hit.mPoseCorrectionAuthority);
        minimumStabilization = std::min(minimumStabilization, hit.mStabilizationStrength);
    }
    EXPECT_EQ(hit.mBehavior, BehaviorId::FaceGuard);
    EXPECT_GT(peakAuthority, standingAuthority);
    EXPECT_LT(minimumStabilization, 1.0f);
    EXPECT_LE(peakAuthority, 1.0f);
    // Authority and budget recover once the disturbance and behavior decay.
    ReactionOutputs after;
    for (int i = 0; i < 400; ++i)
    {
        idle.mFramePhysicsStep = input.mFramePhysicsStep + i + 1;
        after = stepReactionPipeline(state, idle, profile);
    }
    EXPECT_EQ(after.mBehavior, BehaviorId::None);
    EXPECT_FLOAT_EQ(after.mStabilizationStrength, 1.0f);
    EXPECT_NEAR(after.mPoseCorrectionAuthority, standingAuthority, 1e-3f);
}

TEST(RagdollCoreReactionPipelineTest, DuplicateContactIsConsumedOnce)
{
    ReactionPipelineState state;
    const ReactionPipelineProfile profile = enabledProfile();
    ReactionContractInput input = validInput();
    const ReactionOutputs first = stepReactionPipeline(state, input, profile);
    EXPECT_EQ(first.mServedContactId, input.mContact.mContactId);
    EXPECT_FALSE(first.mDuplicateIgnored);
    input.mFramePhysicsStep += 1;
    const ReactionOutputs second = stepReactionPipeline(state, input, profile);
    EXPECT_TRUE(second.mDuplicateIgnored);
    EXPECT_EQ(second.mServedContactId, 0u);
}

TEST(RagdollCoreReactionPipelineTest, DelayedInterleavedDuplicateIsConsumedOncePerActorGeneration)
{
    ReactionPipelineState state;
    const ReactionPipelineProfile profile = enabledProfile();
    ReactionContractInput first = validInput();
    first.mContact.mAttackerActorIdentity = 10;
    first.mContact.mTargetActorIdentity = 20;
    first.mContact.mTargetRegistryGeneration = 3;
    EXPECT_EQ(stepReactionPipeline(state, first, profile).mServedContactId, first.mContact.mContactId);

    ReactionContractInput interleaved = first;
    interleaved.mFramePhysicsStep += 1;
    interleaved.mContact.mContactId += 1;
    EXPECT_EQ(stepReactionPipeline(state, interleaved, profile).mServedContactId,
        interleaved.mContact.mContactId);

    // The first identity remains served after more than the former fixed-size
    // history. Actor removal/reset, rather than incidental traffic volume,
    // defines the lifecycle boundary.
    for (std::uint64_t contact = 0; contact < 80; ++contact)
    {
        interleaved.mFramePhysicsStep += 1;
        interleaved.mContact.mContactId = 1000 + contact;
        EXPECT_EQ(stepReactionPipeline(state, interleaved, profile).mServedContactId,
            interleaved.mContact.mContactId);
    }

    first.mFramePhysicsStep = interleaved.mFramePhysicsStep + 1;
    const ReactionOutputs delayed = stepReactionPipeline(state, first, profile);
    EXPECT_TRUE(delayed.mDuplicateIgnored);
    EXPECT_EQ(delayed.mServedContactId, 0u);

    first.mFramePhysicsStep += 1;
    first.mContact.mTargetRegistryGeneration += 1;
    const ReactionOutputs nextGeneration = stepReactionPipeline(state, first, profile);
    EXPECT_FALSE(nextGeneration.mDuplicateIgnored);
    EXPECT_EQ(nextGeneration.mServedContactId, first.mContact.mContactId);
}

TEST(RagdollCoreReactionPipelineTest, FreshReceiptOnDuplicateContactStillProcesses)
{
    ReactionPipelineState state;
    const ReactionPipelineProfile profile = enabledProfile();
    ReactionContractInput input = validInput();
    input.mContact.mRegion = AnatomicalRegion::LeftCalf;
    stepReactionPipeline(state, input, profile);
    input.mFramePhysicsStep += 1;
    input.mReceipt = ReactionReceipt{ 4, 4.0f, 1.0f };
    input.mWoundSeverity = 0.7f;
    const ReactionOutputs outputs = stepReactionPipeline(state, input, profile);
    EXPECT_TRUE(outputs.mLimpLeft);
    EXPECT_TRUE(outputs.mDuplicateIgnored);
    EXPECT_EQ(outputs.mServedReceiptId, 4u);
}

TEST(RagdollCoreReactionPipelineTest, StaleReceiptIsIgnored)
{
    ReactionPipelineState state;
    const ReactionPipelineProfile profile = enabledProfile();
    ReactionContractInput input = validInput();
    input.mReceipt = ReactionReceipt{ 5, 3.0f, 0.0f };
    stepReactionPipeline(state, input, profile);
    input.mFramePhysicsStep += 1;
    input.mReceipt = ReactionReceipt{ 4, 3.0f, 0.0f };
    const ReactionOutputs outputs = stepReactionPipeline(state, input, profile);
    EXPECT_TRUE(outputs.mStaleIgnored);
    EXPECT_EQ(outputs.mServedReceiptId, 0u);
}

TEST(RagdollCoreReactionPipelineTest, ReceiptInjuriesPersistUntilExplicitUpdateOrReset)
{
    ReactionPipelineState state;
    const auto profile = enabledProfile();
    auto hit = validInput();
    hit.mContact.mRegion = AnatomicalRegion::LeftForearm;
    hit.mImpulse.mMagnitude = 300.0f;
    hit.mReceipt = ReactionReceipt{ 1, 3.0f, 0.0f };
    hit.mWoundSeverity = 0.55f;
    EXPECT_EQ(stepReactionPipeline(state, hit, profile).mBehavior, BehaviorId::ArmCradle);
    auto idle = validInput();
    idle.mContact = {};
    idle.mImpulse = {};
    for (int i = 0; i < 180; ++i)
    {
        idle.mFramePhysicsStep = hit.mFramePhysicsStep + i + 1;
        const auto output = stepReactionPipeline(state, idle, profile);
        EXPECT_TRUE(output.mCradleLeft);
        EXPECT_EQ(output.mServedReceiptId, 0u);
        EXPECT_EQ(output.mServedContactId, 0u);
    }
    hit.mFramePhysicsStep = ++idle.mFramePhysicsStep;
    hit.mContact.mRegion = AnatomicalRegion::RightCalf;
    hit.mContact.mContactId = 8;
    hit.mReceipt = ReactionReceipt{ 2, 4.0f, 0.0f };
    hit.mWoundSeverity = 0.6f;
    const auto both = stepReactionPipeline(state, hit, profile);
    EXPECT_TRUE(both.mCradleLeft);
    EXPECT_TRUE(both.mLimpRight);
    EXPECT_LT(both.mSpeedScale, 1.0f);
    idle.mFramePhysicsStep += 1;
    idle.mOwned.mLeftArm = true;
    EXPECT_FALSE(stepReactionPipeline(state, idle, profile).mCradleLeft);
    idle.mFramePhysicsStep += 1;
    idle.mOwned.mRecoveryBrace = true;
    const auto recovery = stepReactionPipeline(state, idle, profile);
    EXPECT_FALSE(recovery.mLimpRight);
    EXPECT_FLOAT_EQ(recovery.mSpeedScale, 1.0f);
    idle.mOwned = {};
    idle.mFramePhysicsStep += 1;
    EXPECT_TRUE(stepReactionPipeline(state, idle, profile).mLimpRight);
    hit.mFramePhysicsStep = ++idle.mFramePhysicsStep;
    hit.mWoundSeverity = 0.0f; // stale receipt cannot heal
    EXPECT_TRUE(stepReactionPipeline(state, hit, profile).mLimpRight);
    hit.mFramePhysicsStep += 1;
    hit.mReceipt->mReceiptId = 3;
    const auto healedLeg = stepReactionPipeline(state, hit, profile);
    EXPECT_FALSE(healedLeg.mLimpRight);
    EXPECT_TRUE(healedLeg.mCradleLeft);
    state.reset();
    idle.mFramePhysicsStep = 1;
    EXPECT_EQ(stepReactionPipeline(state, idle, profile).mBehavior, BehaviorId::None);

    // A receipt can explicitly report lost capability without a numeric wound.
    hit.mFramePhysicsStep += 1;
    hit.mWoundSeverity.reset();
    hit.mRegionCapable = false;
    const auto disabledLeg = stepReactionPipeline(state, hit, profile);
    EXPECT_TRUE(disabledLeg.mLimpRight);
    EXPECT_TRUE(disabledLeg.mDisabledLimbs.mRightLeg);
    EXPECT_FLOAT_EQ(disabledLeg.mReactionIntensity, 1.0f);
    idle.mFramePhysicsStep = hit.mFramePhysicsStep + 1;
    EXPECT_TRUE(stepReactionPipeline(state, idle, profile).mLimpRight);
    // Missing injury information does not implicitly restore capability.
    hit.mFramePhysicsStep = idle.mFramePhysicsStep + 1;
    hit.mReceipt->mReceiptId += 1;
    hit.mRegionCapable = true;
    EXPECT_TRUE(stepReactionPipeline(state, hit, profile).mLimpRight);
    hit.mFramePhysicsStep += 1;
    hit.mReceipt->mReceiptId += 1;
    hit.mWoundSeverity = 0.0f;
    EXPECT_FALSE(stepReactionPipeline(state, hit, profile).mLimpRight);
}

TEST(RagdollCoreReactionPipelineTest, OutOfOrderFrameIsIgnored)
{
    ReactionPipelineState state;
    const ReactionPipelineProfile profile = enabledProfile();
    ReactionContractInput input = validInput();
    stepReactionPipeline(state, input, profile);
    input.mFramePhysicsStep -= 1;
    input.mContact.mContactId += 1;
    const ReactionOutputs outputs = stepReactionPipeline(state, input, profile);
    EXPECT_TRUE(outputs.mDuplicateIgnored);
    EXPECT_EQ(outputs.mServedContactId, 0u);
}

TEST(RagdollCoreReactionPipelineTest, InvalidInputIsIgnoredLikeIdle)
{
    ReactionPipelineState state;
    const ReactionPipelineProfile profile = enabledProfile();
    ReactionContractInput input = validInput();
    input.mFixedStepDuration = std::numeric_limits<float>::quiet_NaN();
    const ReactionOutputs outputs = stepReactionPipeline(state, input, profile);
    EXPECT_TRUE(outputs.mStaleIgnored);
    EXPECT_EQ(outputs.mBehavior, BehaviorId::None);
}

TEST(RagdollCoreReactionPipelineTest, PassiveModeReportsFallPhase)
{
    ReactionPipelineState state;
    const ReactionPipelineProfile profile = enabledProfile();
    ReactionContractInput input = validInput();
    input.mMode = ReactionBodyMode::Passive;
    input.mContact = ReactionContact{};
    input.mImpulse = ReactionImpulse{};
    const ReactionOutputs outputs = stepReactionPipeline(state, input, profile);
    EXPECT_EQ(outputs.mPhase, ReactionPhase::Fall);
}

TEST(RagdollCoreReactionPipelineTest, RecoveryModeReportsGetUpAndSuppressesCosmetics)
{
    ReactionPipelineState state;
    const ReactionPipelineProfile profile = enabledProfile();
    ReactionContractInput input = validInput();
    input.mMode = ReactionBodyMode::Recovery;
    input.mOwned.mRecoveryBrace = true;
    input.mContact.mRegion = AnatomicalRegion::Head;
    input.mImpulse.mMagnitude = 4000.0f; // above the balance disturbance ramp so authority rises
    auto beforeRecovery = input;
    beforeRecovery.mMode = ReactionBodyMode::Standing;
    beforeRecovery.mOwned.mRecoveryBrace = false;
    EXPECT_TRUE(stepReactionPipeline(state, beforeRecovery, profile).mCoverFace);
    input.mFramePhysicsStep += 1;
    const ReactionOutputs outputs = stepReactionPipeline(state, input, profile);
    EXPECT_EQ(outputs.mPhase, ReactionPhase::GetUp);
    EXPECT_EQ(outputs.mBehavior, BehaviorId::None);
    EXPECT_FALSE(outputs.mCoverFace);
}

TEST(RagdollCoreReactionPipelineTest, PoseCompositionAppliesBoundedOffsets)
{
    std::vector<Transform> pose(6);
    std::vector<Transform> out(6);
    ReactionOutputs outputs;
    outputs.mTorsoFlexion = 0.2f;
    outputs.mHeadRecoil = 0.1f;
    outputs.mLeftArmWithdrawal = 0.3f;
    outputs.mCradleLeft = true;
    ReactionPoseTargets targets;
    targets.mSpine = 1;
    targets.mNeck = 2;
    targets.mLeftUpperArm = 3;
    targets.mRightUpperArm = 4;
    EXPECT_TRUE(applyReactionPose(pose, outputs, targets, out));
    const auto angle = [](const Transform& transform) {
        const JPH::Quat relative = (transform.mRotation * JPH::Quat::sIdentity().Conjugated()).Normalized();
        return 2.0f * std::atan2(relative.GetXYZ().Length(), std::abs(relative.GetW()));
    };
    EXPECT_GT(angle(out[1]), 0.15f);
    EXPECT_GT(angle(out[2]), 0.05f);
    EXPECT_GT(angle(out[3]), 0.3f);
    EXPECT_NEAR(angle(out[4]), 0.0f, 1e-6f);
    EXPECT_NEAR(angle(out[0]), 0.0f, 1e-6f);
    EXPECT_NEAR(angle(out[5]), 0.0f, 1e-6f);
    // Mismatched spans keep the pose untouched.
    std::vector<Transform> shortOut(3);
    EXPECT_FALSE(applyReactionPose(pose, outputs, targets, shortOut));
    EXPECT_NEAR(angle(shortOut[1]), 0.0f, 1e-6f);
}

TEST(RagdollCoreReactionPipelineTest, AnatomicalPosePropagatesBranchesAndPreservesOwnership)
{
    const auto profile = makeScenarioHumanoidProfile();
    const auto skeleton = makeScenarioHumanoidSkeleton();
    auto targets = makeReactionPoseTargets(profile);
    std::vector<Transform> base;
    for (const auto& body : profile.mBodies)
        for (const auto& bone : skeleton.mBones)
            if (body.mBone == bone.mName)
                base.push_back(bone.mModelTransform);
    ASSERT_EQ(base.size(), profile.mBodies.size());
    // Rotated bone bases reproduce the captured rig's non-anatomical local X.
    for (auto& body : base)
        body.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 1.3f);
    auto guard = base;
    ReactionOutputs reaction;
    reaction.mBehavior = BehaviorId::FaceGuard;
    reaction.mReactionIntensity = 1.0f;
    reaction.mCoverFace = true;
    reaction.mLeftArmWithdrawal = reaction.mRightArmWithdrawal = 0.6f;
    ASSERT_TRUE(applyReactionPose(base, reaction, targets, guard));
    for (const auto hand : { targets.mLeftHand, targets.mRightHand })
        EXPECT_GT(guard[hand].mTranslation.GetZ() - base[hand].mTranslation.GetZ(), 15.0f);
    EXPECT_TRUE(guard[0].mTranslation.IsClose(base[0].mTranslation));
    EXPECT_TRUE(guard[0].mRotation.IsClose(base[0].mRotation));
    auto alias = base;
    ASSERT_TRUE(applyReactionPose(alias, reaction, targets, alias));
    for (std::size_t i = 0; i < base.size(); ++i)
    {
        EXPECT_TRUE(alias[i].mTranslation.IsClose(guard[i].mTranslation));
        EXPECT_TRUE(alias[i].mRotation.IsClose(guard[i].mRotation));
    }
    reaction.mDisabledLimbs.mLeftArm = true;
    ASSERT_TRUE(applyReactionPose(base, reaction, targets, guard));
    EXPECT_TRUE(guard[targets.mLeftHand].mTranslation.IsClose(base[targets.mLeftHand].mTranslation));
    EXPECT_GT(guard[targets.mRightHand].mTranslation.GetZ() - base[targets.mRightHand].mTranslation.GetZ(), 15.0f);

    reaction = {};
    reaction.mBehavior = BehaviorId::DoubledOver;
    reaction.mReactionIntensity = 1.0f;
    reaction.mTorsoFlexion = 0.35f;
    ASSERT_TRUE(applyReactionPose(base, reaction, targets, guard));
    EXPECT_GT(guard[targets.mNeck].mTranslation.GetY() - base[targets.mNeck].mTranslation.GetY(), 5.0f);
    // All three spine joints contribute; the head follows the trunk.
    const auto deviation = [&](std::size_t b) {
        return 2.0f * std::acos(std::min(1.0f, std::abs(guard[b].mRotation.Dot(base[b].mRotation))));
    };
    EXPECT_NEAR(deviation(targets.mSpine), 0.35f / 3.0f, 1e-4f);
    EXPECT_NEAR(deviation(targets.mSpine2), 0.35f, 1e-4f);
    EXPECT_NEAR(deviation(targets.mHead), 0.35f, 1e-4f);
    targets.mParents[1] = 1;
    const auto unchanged = guard;
    EXPECT_FALSE(applyReactionPose(base, reaction, targets, guard));
    EXPECT_TRUE(guard[targets.mHead].mTranslation.IsClose(unchanged[targets.mHead].mTranslation));
}

TEST(RagdollCoreReactionPipelineTest, DeterministicRepeat)
{
    const auto run = [] {
        ReactionPipelineState state;
        const ReactionPipelineProfile profile = enabledProfile();
        std::vector<ReactionOutputs> trace;
        ReactionContractInput input = validInput();
        for (int i = 0; i < 30; ++i)
        {
            input.mFramePhysicsStep += 1;
            input.mContact.mContactId += i == 5 ? 1 : 0;
            trace.push_back(stepReactionPipeline(state, input, profile));
        }
        return trace;
    };
    const auto first = run();
    const auto second = run();
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i)
    {
        EXPECT_FLOAT_EQ(first[i].mPoseCorrectionAuthority, second[i].mPoseCorrectionAuthority);
        EXPECT_FLOAT_EQ(first[i].mStabilizationStrength, second[i].mStabilizationStrength);
        EXPECT_EQ(first[i].mBehavior, second[i].mBehavior);
        EXPECT_EQ(first[i].mPhase, second[i].mPhase);
    }
}
