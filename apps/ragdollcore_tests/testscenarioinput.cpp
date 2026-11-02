#include <ragdollcore/scenario.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
    using namespace RagdollCore;

    RagdollPose bindPose()
    {
        RagdollPose pose;
        for (const auto& bone : makeScenarioHumanoidSkeleton().mBones)
            pose.mBodyTransforms.push_back(bone.mModelTransform);
        return pose;
    }

    ScenarioResult run(const ScenarioScript& script, const ScenarioInputProvider& provider = {})
    {
        return runRagdollScenario(makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), {}, script, provider);
    }

    ScenarioResult runDirect(const ScenarioScript& script, const ScenarioInputProvider& provider = {})
    {
        ScenarioSettings settings;
        settings.mLocomotionController = ScenarioLocomotionController::DirectMotorTracking;
        return runRagdollScenario(makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), settings, script, provider);
    }

    ScenarioResult runProductionResidual(const ScenarioScript& script, const ScenarioInputProvider& provider = {})
    {
        ScenarioSettings settings;
        settings.mLocomotionController = ScenarioLocomotionController::ProductionResidual;
        return runRagdollScenario(makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), settings, script, provider);
    }

    TEST(RagdollCoreScenarioInputTest, productionResidualZeroPreservesEveryPhysicalTelemetryByte)
    {
        static_assert(static_cast<int>(ScenarioLocomotionController::ProductionResidual) == 3);
        ScenarioScript script;
        script.mDurationSeconds = 4.0f;
        ScenarioInputKeyframe key;
        script.mKeyframes.push_back(key);
        key.mStep = 60;
        key.mRequestedHorizontalVelocity = JPH::Vec3(0, 45, 0);
        script.mKeyframes.push_back(key);
        key.mStep = 180;
        key.mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        script.mKeyframes.push_back(key);
        key.mStep = 210;
        key.mMotorsEnabled = false;
        script.mKeyframes.push_back(key);
        const auto reference = run(script);
        ASSERT_TRUE(reference.completed());
        for (auto& entry : script.mKeyframes)
            entry.mMotorTargetRotationOffsets.assign(20, JPH::Vec3::sZero());
        auto zero = runProductionResidual(script);
        ASSERT_TRUE(zero.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(zero, runProductionResidual(script)));
        // Provenance remains distinct; only that declared enum is normalized
        // to compare every encoded physical/actuation frame and startup field.
        EXPECT_NE(hashScenarioTelemetry(reference), hashScenarioTelemetry(zero));
        zero.mSettings.mLocomotionController = ScenarioLocomotionController::Reference;
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(reference, zero));
    }

    TEST(RagdollCoreScenarioInputTest, productionResidualChangesPoseWithoutAddingActuationAuthority)
    {
        ScenarioScript script;
        script.mDurationSeconds = 1.0f / 60.0f;
        ScenarioInputKeyframe key;
        script.mKeyframes.push_back(key);
        const auto reference = runProductionResidual(script);
        ASSERT_TRUE(reference.completed());
        key.mMotorTargetRotationOffsets.assign(20, JPH::Vec3::sZero());
        key.mMotorTargetRotationOffsets[1] = JPH::Vec3(0.1f, 0, 0);
        script.mKeyframes[0] = key;
        const auto corrected = runProductionResidual(script);
        ASSERT_TRUE(corrected.completed());
        const auto& before = reference.mFrames[0];
        const auto& after = corrected.mFrames[0];
        ASSERT_TRUE(before.mJointMotorTargets[1]);
        ASSERT_TRUE(after.mJointMotorTargets[1]);
        ASSERT_TRUE(before.mJointMotorTargets[1]->mTargetOrientation);
        ASSERT_TRUE(after.mJointMotorTargets[1]->mTargetOrientation);
        EXPECT_FALSE(before.mJointMotorTargets[1]->mTargetOrientation->IsClose(
            *after.mJointMotorTargets[1]->mTargetOrientation, 1e-6f));
        // The child's own local target is preserved through a parent offset.
        EXPECT_TRUE(before.mDrivenMotorLocalRotations[2].IsClose(after.mDrivenMotorLocalRotations[2], 1e-6f));
        ASSERT_TRUE(before.mJointSupport);
        ASSERT_TRUE(after.mJointSupport);
        ASSERT_EQ(before.mJointSupport->mJointTorques.size(), after.mJointSupport->mJointTorques.size());
        for (std::size_t body = 0; body < before.mJointSupport->mJointTorques.size(); ++body)
            EXPECT_EQ(before.mJointSupport->mJointTorques[body], after.mJointSupport->mJointTorques[body]);
        EXPECT_EQ(before.mAppliedPelvisForce, after.mAppliedPelvisForce);
        EXPECT_EQ(before.mAppliedPelvisTorque, after.mAppliedPelvisTorque);
        EXPECT_EQ(before.mAppliedSpineTorque, after.mAppliedSpineTorque);
        EXPECT_EQ(before.mAppliedNetForce, after.mAppliedNetForce);
        EXPECT_EQ(before.mJointMotorTargets[1]->mTargetAngularVelocity,
            after.mJointMotorTargets[1]->mTargetAngularVelocity);
    }

    TEST(RagdollCoreScenarioInputTest, productionResidualMotorReleaseIgnoresCorrections)
    {
        ScenarioScript script;
        script.mDurationSeconds = 0.5f;
        ScenarioInputKeyframe key;
        script.mKeyframes.push_back(key);
        key.mStep = 10;
        key.mMotorsEnabled = false;
        script.mKeyframes.push_back(key);
        const auto reference = runProductionResidual(script);
        ASSERT_TRUE(reference.completed());
        script.mKeyframes.back().mMotorTargetRotationOffsets.assign(20, JPH::Vec3::sZero());
        script.mKeyframes.back().mMotorTargetRotationOffsets[1] = JPH::Vec3(0.1f, 0, 0);
        const auto corrected = runProductionResidual(script);
        ASSERT_TRUE(corrected.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(reference, corrected));
        for (std::size_t step = 10; step < corrected.mFrames.size(); ++step)
        {
            EXPECT_FALSE(corrected.mFrames[step].mMotorsApplied);
            EXPECT_TRUE(corrected.mFrames[step].mJointMotorTargets.empty());
        }
    }

    TEST(RagdollCoreScenarioInputTest, productionResidualRejectsInvalidOffsetsBeforeSolve)
    {
        for (int invalid = 0; invalid < 6; ++invalid)
        {
            ScenarioInputKeyframe key;
            key.mStep = 3;
            key.mMotorTargetRotationOffsets.assign(20, JPH::Vec3::sZero());
            if (invalid == 0) key.mMotorTargetRotationOffsets.pop_back();
            if (invalid == 1) key.mMotorTargetRotationOffsets[1].SetX(std::numeric_limits<float>::quiet_NaN());
            if (invalid == 2) key.mMotorTargetRotationOffsets[1] = JPH::Vec3(0.1001f, 0, 0);
            if (invalid == 3) key.mMotorTargetRotationOffsets[0] = JPH::Vec3(0.01f, 0, 0);
            if (invalid == 4) key.mMotorTargetRotationOffsets[1] = JPH::Vec3(0.08f, 0.08f, 0);
            if (invalid == 5) key.mMotorTargetAngularVelocities.assign(20, JPH::Vec3::sZero());
            const auto result = runProductionResidual({}, [&](std::uint64_t step, const ScenarioFrame*)
                -> std::optional<ScenarioInputKeyframe> { return step == 3 ? std::optional(key) : std::nullopt; });
            EXPECT_FALSE(result.completed()) << invalid;
            EXPECT_EQ(result.mFrames.size(), 3u) << invalid;
        }
        auto profile = makeScenarioHumanoidProfile();
        profile.mBodies[1].mConstraint->mMotor.mMaximumTorque = 0;
        ScenarioSettings settings;
        settings.mLocomotionController = ScenarioLocomotionController::ProductionResidual;
        ScenarioScript script;
        ScenarioInputKeyframe key;
        key.mMotorTargetRotationOffsets.assign(20, JPH::Vec3::sZero());
        key.mMotorTargetRotationOffsets[1] = JPH::Vec3(0.01f, 0, 0);
        script.mKeyframes.push_back(key);
        const auto passive = runRagdollScenario(makeScenarioHumanoidSkeleton(), profile, settings, script);
        EXPECT_FALSE(passive.completed());
        EXPECT_TRUE(passive.mFrames.empty());
        CrowdScenarioSettings crowd;
        crowd.mActorSettings = settings;
        const auto unsupported = runStandingCrowdScenario(makeScenarioHumanoidSkeleton(), profile, crowd);
        EXPECT_FALSE(unsupported.completed());
        EXPECT_TRUE(unsupported.mFrames.empty());
    }

    TEST(RagdollCoreScenarioInputTest, directTrackingZeroResidualPreservesReferenceBytes)
    {
        // These values are already part of encoded scenario identity.
        static_assert(static_cast<int>(ScenarioLocomotionController::Legacy) == 0);
        static_assert(static_cast<int>(ScenarioLocomotionController::Reference) == 1);
        static_assert(static_cast<int>(ScenarioLocomotionController::DirectMotorTracking) == 2);
        ScenarioScript script;
        script.mDurationSeconds = 0.25f;
        ScenarioInputKeyframe key;
        key.mMotorTarget = bindPose();
        key.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
        key.mMotorTarget->mBodyTransforms[1].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.05f);
        script.mKeyframes.push_back(key);
        const auto reference = runDirect(script);
        ASSERT_TRUE(reference.completed());
        script.mKeyframes[0].mMotorTargetRotationOffsets.assign(reference.mBodyNames.size(), JPH::Vec3::sZero());
        const auto zero = runDirect(script);
        ASSERT_TRUE(zero.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(reference, zero));
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(zero, runDirect(script)));
    }

    TEST(RagdollCoreScenarioInputTest, directTrackingOwnsFinalMotorTargetsWhileMoving)
    {
        ScenarioScript script;
        script.mDurationSeconds = 0.25f;
        ScenarioInputKeyframe key;
        key.mMotorTarget = bindPose();
        key.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
        key.mMotorTargetAngularVelocities.assign(20, JPH::Vec3::sZero());
        key.mMotorTargetAngularVelocities[14] = JPH::Vec3(0.15f, -0.1f, 0.05f);
        key.mMotorTargetAngularVelocities[16] = JPH::Vec3(-0.1f, 0.05f, 0.1f);
        script.mKeyframes.push_back(key);
        const auto reference = runDirect(script);
        ASSERT_TRUE(reference.completed());
        script.mKeyframes[0].mMotorTargetRotationOffsets.assign(20, JPH::Vec3::sZero());
        script.mKeyframes[0].mMotorTargetRotationOffsets[14] = JPH::Vec3(0.12f, 0, 0);
        const auto corrected = runDirect(script);
        ASSERT_TRUE(corrected.completed());
        ASSERT_FALSE(reference.mFrames.empty());
        ASSERT_EQ(reference.mFrames.size(), corrected.mFrames.size());
        ASSERT_EQ(reference.mFrames[0].mJointMotorTargets.size(), 20u);
        ASSERT_EQ(corrected.mFrames[0].mJointMotorTargets.size(), 20u);
        ASSERT_TRUE(reference.mFrames[0].mJointMotorTargets[14]);
        ASSERT_TRUE(corrected.mFrames[0].mJointMotorTargets[14]);
        ASSERT_TRUE(reference.mFrames[0].mJointMotorTargets[14]->mTargetOrientation);
        ASSERT_TRUE(corrected.mFrames[0].mJointMotorTargets[14]->mTargetOrientation);
        EXPECT_FALSE(reference.mFrames[0].mJointMotorTargets[14]->mTargetOrientation->IsClose(
            *corrected.mFrames[0].mJointMotorTargets[14]->mTargetOrientation, 1e-6f));
        // A parent correction must preserve the child's own relative target.
        ASSERT_TRUE(reference.mFrames[0].mJointMotorTargets[15]);
        ASSERT_TRUE(corrected.mFrames[0].mJointMotorTargets[15]);
        ASSERT_TRUE(reference.mFrames[0].mJointMotorTargets[15]->mTargetHingeAngle);
        ASSERT_TRUE(corrected.mFrames[0].mJointMotorTargets[15]->mTargetHingeAngle);
        EXPECT_NEAR(*reference.mFrames[0].mJointMotorTargets[15]->mTargetHingeAngle,
            *corrected.mFrames[0].mJointMotorTargets[15]->mTargetHingeAngle, 1e-6f);
        const auto& initialHip = *corrected.mFrames[0].mJointMotorTargets[14]->mTargetOrientation;
        std::size_t driven = 0;
        bool physicalDifference = false;
        for (std::size_t step = 0; step < corrected.mFrames.size(); ++step)
        {
            const auto& frame = corrected.mFrames[step];
            EXPECT_FALSE(frame.mReferenceGait);
            EXPECT_FALSE(frame.mJointSupport);
            EXPECT_FALSE(frame.mUprightApplied);
            EXPECT_FALSE(frame.mSwingFootControlApplied);
            EXPECT_FALSE(frame.mSwingHipWorldControlApplied);
            EXPECT_FLOAT_EQ(frame.mPreLiftSwingUnloadForce, 0);
            EXPECT_TRUE(frame.mAppliedPelvisForce.IsNearZero());
            EXPECT_TRUE(frame.mAppliedPelvisTorque.IsNearZero());
            EXPECT_TRUE(frame.mAppliedSpineTorque.IsNearZero());
            EXPECT_TRUE(frame.mAppliedNetForce.IsNearZero());
            EXPECT_TRUE(frame.mAppliedNetActuationMoment.IsNearZero());
            EXPECT_FALSE(frame.mGait.hasStance());
            if (frame.mMotorsApplied)
            {
                ++driven;
                ASSERT_TRUE(frame.mJointMotorTargets[14]);
                ASSERT_TRUE(frame.mJointMotorTargets[14]->mTargetOrientation);
                EXPECT_TRUE(initialHip.IsClose(*frame.mJointMotorTargets[14]->mTargetOrientation, 1e-6f));
                for (const auto body : { 14u, 16u })
                {
                    ASSERT_TRUE(frame.mJointMotorTargets[body]);
                    EXPECT_FALSE(frame.mJointMotorTargets[body]->mWorldOrientation);
                    EXPECT_LT((frame.mJointMotorTargets[body]->mTargetAngularVelocity
                                  - key.mMotorTargetAngularVelocities[body]).Length(),
                        1e-5f);
                }
                ASSERT_EQ(frame.mJointSupportMotorAngularImpulses.size(), 20u);
            }
            physicalDifference |= !frame.mBodies[14].mWorldRotation.IsClose(
                reference.mFrames[step].mBodies[14].mWorldRotation, 1e-6f);
        }
        EXPECT_GE(driven, 3u);
        EXPECT_TRUE(physicalDifference);
    }

    TEST(RagdollCoreScenarioInputTest, directTrackingPreservesParentAxesInRotatedFrames)
    {
        const auto skeleton = makeScenarioHumanoidSkeleton();
        ScenarioSettings settings;
        settings.mLocomotionController = ScenarioLocomotionController::DirectMotorTracking;
        settings.mRootRotation = (JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.7f)
            * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.08f)).Normalized();
        ScenarioScript script;
        script.mDurationSeconds = settings.mFixedTimeStep;
        ScenarioInputKeyframe key;
        key.mMotorTarget = bindPose();
        key.mRequestedHorizontalVelocity = JPH::Vec3(17, 43, 0);
        const auto modelRoot = (JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.31f)
            * JPH::Quat::sRotation(JPH::Vec3::sAxisY(), -0.23f)).Normalized();
        std::vector<JPH::Quat> referenceLocals(skeleton.mBones.size(), JPH::Quat::sIdentity());
        referenceLocals[1] = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.06f);
        referenceLocals[2] = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.07f);
        referenceLocals[3] = (JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.04f)
            * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.03f)).Normalized();
        for (std::size_t body = 0; body < skeleton.mBones.size(); ++body)
        {
            const auto parent = skeleton.mBones[body].mParent;
            key.mMotorTarget->mBodyTransforms[body].mRotation = parent < 0 ? modelRoot
                : (key.mMotorTarget->mBodyTransforms[parent].mRotation * referenceLocals[body]).Normalized();
        }
        key.mMotorTargetRotationOffsets.assign(skeleton.mBones.size(), JPH::Vec3::sZero());
        key.mMotorTargetRotationOffsets[1] = JPH::Vec3(0.08f, 0, 0);
        key.mMotorTargetRotationOffsets[2] = JPH::Vec3(0, 0, 0.09f);
        key.mMotorTargetAngularVelocities.assign(skeleton.mBones.size(), JPH::Vec3::sZero());
        key.mMotorTargetAngularVelocities[1] = JPH::Vec3(0.19f, -0.08f, 0.12f);
        key.mMotorTargetAngularVelocities[2] = JPH::Vec3(0.03f, 0.16f, -0.07f);
        key.mMotorTargetAngularVelocities[3] = JPH::Vec3(-0.11f, 0.04f, 0.09f);
        script.mKeyframes.push_back(key);
        const auto result = runRagdollScenario(skeleton, makeScenarioHumanoidProfile(), settings, script);
        ASSERT_TRUE(result.completed());
        ASSERT_EQ(result.mFrames.size(), 1u);
        ASSERT_EQ(result.mFrames[0].mJointMotorTargets.size(), skeleton.mBones.size());

        const std::array residuals{ JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.08f),
            JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.09f), JPH::Quat::sIdentity() };
        // Jolt orders the frame columns as twist, plane.Cross(twist), plane.
        // The synthetic spine's +Z twist and +Y plane therefore give (+Z,+X,+Y)
        // in both bind-pose bodies. Decode independently of pose reconstruction.
        const JPH::Quat jointFrame(-0.5f, -0.5f, -0.5f, 0.5f);
        for (std::size_t body = 1; body <= 3; ++body)
        {
            const auto& target = result.mFrames[0].mJointMotorTargets[body];
            ASSERT_TRUE(target);
            ASSERT_TRUE(target->mRequestedOrientation);
            ASSERT_TRUE(target->mTargetOrientation);
            const auto expected = (residuals[body - 1] * referenceLocals[body]).Normalized();
            const auto requestedLocal
                = (jointFrame * *target->mRequestedOrientation * jointFrame.Conjugated()).Normalized();
            const auto finalLocal = (jointFrame * *target->mTargetOrientation * jointFrame.Conjugated()).Normalized();
            const auto targetDelta = requestedLocal * expected.Conjugated();
            EXPECT_TRUE(requestedLocal.IsClose(expected, 1e-8f)) << body
                << " target delta=" << targetDelta.GetX() << ',' << targetDelta.GetY() << ','
                << targetDelta.GetZ() << ',' << targetDelta.GetW();
            EXPECT_TRUE(finalLocal.IsClose(expected, 1e-8f)) << body;
            EXPECT_NEAR(target->mTargetClampDelta, 0, 1e-5f) << body;
            if (body < 3)
            {
                // These rotations do not commute: a child-axis residual would
                // produce a measurably different target. Body 3's zero residual
                // instead keeps its own reference local through both ancestors.
                const auto wrongOrder = (referenceLocals[body] * residuals[body - 1]).Normalized();
                EXPECT_GT((expected * wrongOrder.Conjugated()).GetXYZ().Length(), 1e-3f) << body;
            }
            EXPECT_FALSE(target->mWorldOrientation);
            const auto worldVelocity = key.mMotorTargetAngularVelocities[body];
            EXPECT_LT((target->mTargetAngularVelocity - worldVelocity).Length(), 1e-5f) << body;
            EXPECT_GT((settings.mRootRotation * worldVelocity - worldVelocity).Length(), 0.01f) << body;
        }
    }

    TEST(RagdollCoreScenarioInputTest, directTrackingReplaysFinalCommandsAndMotorRelease)
    {
        ScenarioScript script;
        script.mDurationSeconds = 0.4f;
        ScenarioScript recorded = script;
        const auto feedback = runDirect(script, [&](std::uint64_t step, const ScenarioFrame*) {
            ScenarioInputKeyframe key;
            key.mStep = step;
            key.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
            key.mMotorTarget = bindPose();
            key.mMotorsEnabled = step < 8 || step >= 14;
            if (step < 18)
            {
                key.mMotorTargetRotationOffsets.assign(20, JPH::Vec3::sZero());
                key.mMotorTargetRotationOffsets[1] = JPH::Vec3(0.06f * std::sin(static_cast<float>(step)), 0, 0);
                key.mMotorTargetAngularVelocities.assign(20, JPH::Vec3::sZero());
                key.mMotorTargetAngularVelocities[1] = JPH::Vec3(0.1f, 0.05f, 0);
            }
            recorded.mKeyframes.push_back(key);
            return std::optional(key);
        });
        ASSERT_TRUE(feedback.completed());
        const auto replay = runDirect(recorded);
        ASSERT_TRUE(replay.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(feedback, replay));
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(replay, runDirect(recorded)));
        ASSERT_GE(feedback.mStartupMotorTargets.size(), 14u);
        auto ignored = recorded;
        for (std::size_t step = 8; step < 14; ++step)
        {
            auto& key = ignored.mKeyframes[step];
            key.mMotorTargetRotationOffsets[1] = JPH::Vec3(0, 0.4f, 0);
            key.mMotorTargetAngularVelocities[1] = JPH::Vec3(2, 3, 4);
            const auto& frame = feedback.mFrames[step];
            EXPECT_FALSE(frame.mMotorsApplied);
            EXPECT_TRUE(frame.mJointMotorTargets.empty());
            ASSERT_EQ(frame.mJointSupportMotorAngularImpulses.size(), 20u);
            for (const auto impulse : frame.mJointSupportMotorAngularImpulses)
                EXPECT_TRUE(impulse.IsNearZero(1e-6f));
            for (const float utilization : frame.mJointSupportMotorUtilizations)
                EXPECT_FLOAT_EQ(utilization, 0);
            // Startup captures include drive state even when position-target
            // capture returns false. Jolt's Off motor state is encoded as 0.
            ASSERT_EQ(feedback.mStartupMotorTargets[step].size(), 20u);
            for (std::size_t body = 1; body < 20; ++body)
            {
                const auto& target = feedback.mStartupMotorTargets[step][body];
                ASSERT_TRUE(target.mDrive);
                EXPECT_EQ(target.mDrive->mSwingOrHingeState, 0);
                EXPECT_EQ(target.mDrive->mTwistState, 0);
            }
            EXPECT_FALSE(frame.mUprightApplied);
        }
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(feedback, runDirect(ignored)));
        for (std::size_t step = 18; step < feedback.mFrames.size(); ++step)
            if (feedback.mFrames[step].mMotorsApplied)
            {
                ASSERT_TRUE(feedback.mFrames[step].mJointMotorTargets[1]);
                EXPECT_TRUE(feedback.mFrames[step].mJointMotorTargets[1]->mTargetAngularVelocity.IsNearZero());
            }
        // The existing telemetry encoding includes dispatched targets, not only
        // upstream input: changing a captured command must break exact identity.
        auto altered = replay;
        ASSERT_TRUE(altered.mFrames[0].mJointMotorTargets[1]);
        altered.mFrames[0].mJointMotorTargets[1]->mTargetAngularVelocity = JPH::Vec3::sAxisZ();
        EXPECT_FALSE(haveIdenticalScenarioTelemetry(replay, altered));
    }

    TEST(RagdollCoreScenarioInputTest, directTrackingRejectsInvalidActionsBeforeTheirSolve)
    {
        for (int invalid = 0; invalid < 9; ++invalid)
        {
            ScenarioInputKeyframe key;
            key.mStep = 3;
            key.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
            key.mMotorTarget = bindPose();
            key.mMotorTargetRotationOffsets.assign(20, JPH::Vec3::sZero());
            key.mMotorTargetAngularVelocities.assign(20, JPH::Vec3::sZero());
            if (invalid == 0) key.mMotorTargetRotationOffsets.pop_back();
            if (invalid == 1) key.mMotorTargetRotationOffsets[1].SetX(std::numeric_limits<float>::quiet_NaN());
            if (invalid == 2) key.mMotorTargetRotationOffsets[1] = JPH::Vec3(0.51f, 0, 0);
            if (invalid == 3) key.mMotorTargetRotationOffsets[0] = JPH::Vec3(0.1f, 0, 0);
            if (invalid == 4) key.mMotorTargetAngularVelocities.pop_back();
            if (invalid == 5) key.mMotorTargetAngularVelocities[1].SetX(std::numeric_limits<float>::infinity());
            if (invalid == 6) key.mMotorTargetAngularVelocities[0] = JPH::Vec3::sAxisX();
            if (invalid == 7) key.mSupportMotion.emplace();
            if (invalid == 8) key.mRecoveryRequested = true;
            const auto result = runDirect({}, [&](std::uint64_t step, const ScenarioFrame*)
                -> std::optional<ScenarioInputKeyframe> { return step == 3 ? std::optional(key) : std::nullopt; });
            EXPECT_FALSE(result.completed()) << invalid;
            EXPECT_EQ(result.mFrames.size(), 3u) << invalid;
        }
        auto profile = makeScenarioHumanoidProfile();
        profile.mBodies[1].mConstraint->mMotor.mMaximumTorque = 0;
        ScenarioSettings settings;
        settings.mLocomotionController = ScenarioLocomotionController::DirectMotorTracking;
        ScenarioScript passive;
        ScenarioInputKeyframe key;
        key.mMotorTargetRotationOffsets.assign(20, JPH::Vec3::sZero());
        key.mMotorTargetRotationOffsets[1] = JPH::Vec3(0.1f, 0, 0);
        passive.mKeyframes.push_back(key);
        EXPECT_FALSE(runRagdollScenario(makeScenarioHumanoidSkeleton(), profile, settings, passive).completed());
        // Existing routes reject the new input instead of silently ignoring it.
        EXPECT_FALSE(run(passive).completed());
    }

    TEST(RagdollCoreScenarioInputTest, directTrackingRejectsCompetingActuationSettings)
    {
        for (int conflict = 0; conflict < 7; ++conflict)
        {
            ScenarioSettings settings;
            settings.mLocomotionController = ScenarioLocomotionController::DirectMotorTracking;
            if (conflict == 0) settings.mPinPelvis = true;
            if (conflict == 1) settings.mUseSolverActuation = true;
            if (conflict == 2) settings.mReserveStanceHipMotor = true;
            if (conflict == 3) settings.mUseWorldSwingHipControl = true;
            if (conflict == 4) settings.mCompensateSwingHipMotorImpulse = true;
            if (conflict == 5) settings.mReplayNativeBraceLimits = true;
            if (conflict == 6) settings.mReplayAutomaticRecoveryRequest = true;
            const auto result
                = runRagdollScenario(makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), settings, {});
            EXPECT_FALSE(result.completed()) << conflict;
            EXPECT_TRUE(result.mFrames.empty()) << conflict;
        }
        CrowdScenarioSettings crowd;
        crowd.mActorSettings.mLocomotionController = ScenarioLocomotionController::DirectMotorTracking;
        const auto result = runStandingCrowdScenario(makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), crowd);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());
    }

    TEST(RagdollCoreScenarioInputTest, unchangedFeedbackPreservesPhysicsAndObservationTime)
    {
        ScenarioScript script;
        script.mDurationSeconds = 2.0f;
        const auto baseline = run(script);
        ASSERT_TRUE(baseline.completed());
        std::uint64_t calls = 0;
        const auto observed = run(script, [&](std::uint64_t step, const ScenarioFrame* previous)
            -> std::optional<ScenarioInputKeyframe> {
            EXPECT_EQ(step, calls++);
            if (step == 0)
                EXPECT_EQ(previous, nullptr);
            else
            {
                EXPECT_NE(previous, nullptr);
                if (previous)
                {
                    EXPECT_EQ(previous->mStep, step - 1);
                    EXPECT_FLOAT_EQ(previous->mTime, step * baseline.mSettings.mFixedTimeStep);
                    EXPECT_EQ(previous->mBodies.size(), baseline.mBodyNames.size());
                }
            }
            return std::nullopt;
        });
        ASSERT_TRUE(observed.completed());
        EXPECT_EQ(calls, baseline.mStepCount);
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(baseline, observed));
    }

    TEST(RagdollCoreScenarioInputTest, allJointTargetsRecordCommandsAndRespectMotorRelease)
    {
        ScenarioScript script;
        script.mDurationSeconds = 0.5f;
        ScenarioInputKeyframe release;
        release.mStep = 10;
        release.mMotorsEnabled = false;
        script.mKeyframes.push_back(release);
        const auto result = run(script);
        ASSERT_TRUE(result.completed());
        for (const auto& frame : result.mFrames)
        {
            if (frame.mStep >= release.mStep)
            {
                EXPECT_FALSE(frame.mMotorsApplied);
                EXPECT_TRUE(frame.mJointMotorTargets.empty());
                continue;
            }
            ASSERT_EQ(frame.mJointMotorTargets.size(), result.mBodyNames.size());
            EXPECT_FALSE(frame.mJointMotorTargets[0]);
            for (const auto body : { 14u, 15u, 16u, 17u, 18u, 19u })
                EXPECT_TRUE(frame.mJointMotorTargets[body]) << body;
        }
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, run(script)));
        auto changed = result;
        ASSERT_TRUE(changed.mFrames.front().mJointMotorTargets[15]);
        changed.mFrames.front().mJointMotorTargets[15]->mTargetHingeAngle = 0.3f;
        EXPECT_FALSE(haveIdenticalScenarioTelemetry(result, changed));
        changed = result;
        changed.mFrames.front().mJointMotorTargets[15]->mTargetAngularVelocity = JPH::Vec3::sAxisX();
        EXPECT_FALSE(haveIdenticalScenarioTelemetry(result, changed));
    }

    TEST(RagdollCoreScenarioInputTest, movingPoseVelocityReplaysClearsAndRejectsInvalidInputs)
    {
        ScenarioScript script;
        script.mDurationSeconds = 0.5f;
        ScenarioInputKeyframe move;
        move.mStep = 5;
        move.mMotorTarget = bindPose();
        move.mMotorTargetAngularVelocities.assign(move.mMotorTarget->mBodyTransforms.size(), JPH::Vec3::sZero());
        move.mMotorTargetAngularVelocities[1] = JPH::Vec3(0.1f, 0.2f, -0.1f);
        script.mKeyframes.push_back(move);
        ScenarioInputKeyframe hold;
        hold.mStep = 10;
        script.mKeyframes.push_back(hold);
        auto release = move;
        release.mStep = 15;
        release.mMotorsEnabled = false;
        script.mKeyframes.push_back(release);
        auto resume = hold;
        resume.mStep = 20;
        script.mKeyframes.push_back(resume);
        const auto result = run(script);
        ASSERT_TRUE(result.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, run(script)));
        for (const auto& frame : result.mFrames)
        {
            if (frame.mStep >= 15 && frame.mStep < 20)
            {
                EXPECT_TRUE(frame.mJointMotorTargets.empty());
                continue;
            }
            ASSERT_TRUE(frame.mJointMotorTargets[1]);
            const auto expected = frame.mStep >= 5 && frame.mStep < 10
                ? move.mMotorTargetAngularVelocities[1] : JPH::Vec3::sZero();
            EXPECT_LT((frame.mJointMotorTargets[1]->mTargetAngularVelocity - expected).Length(), 1e-6f);
        }
        auto zero = script;
        zero.mKeyframes[0].mMotorTargetAngularVelocities.clear();
        EXPECT_FALSE(haveIdenticalScenarioTelemetry(result, run(zero)));
        for (int invalid = 0; invalid < 4; ++invalid)
        {
            auto key = move;
            key.mStep = 5;
            if (invalid == 0) key.mMotorTargetAngularVelocities.pop_back();
            if (invalid == 1) key.mMotorTargetAngularVelocities[1].SetY(std::numeric_limits<float>::infinity());
            if (invalid == 2) key.mMotorTargetAngularVelocities[0] = JPH::Vec3::sAxisY();
            if (invalid == 3) key.mRequestedHorizontalVelocity = JPH::Vec3::sAxisY();
            const auto rejected = run({}, [&](std::uint64_t step, const ScenarioFrame*)
                -> std::optional<ScenarioInputKeyframe> { return step == 5 ? std::optional(key) : std::nullopt; });
            EXPECT_FALSE(rejected.completed());
            EXPECT_EQ(rejected.mFrames.size(), 5u);
        }
    }

    TEST(RagdollCoreScenarioInputTest, measuredFeedbackReplaysExactlyAndRespectsRelease)
    {
        ScenarioScript script;
        script.mDurationSeconds = 2.0f;
        script.mImpulses.push_back({ 20, 0, JPH::Vec3(150.0f, 0.0f, 0.0f), std::nullopt });
        ScenarioScript recorded = script;
        const auto initial = bindPose();
        const auto feedback = run(script, [&](std::uint64_t step, const ScenarioFrame* previous)
            -> std::optional<ScenarioInputKeyframe> {
            if (step % 4 != 0)
                return std::nullopt;
            ScenarioInputKeyframe key;
            key.mStep = step;
            key.mMotorTarget = initial;
            const float angle = previous
                ? std::clamp(previous->mCenterOfMassVelocity.GetX() * 0.01f, -0.12f, 0.12f) : 0.0f;
            key.mMotorTarget->mBodyTransforms[1].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), angle);
            key.mMotorsEnabled = step < 80;
            recorded.mKeyframes.push_back(key);
            return key;
        });
        ASSERT_TRUE(feedback.completed());
        ASSERT_EQ(recorded.mKeyframes.size(), 30u);
        bool changed = false;
        for (const auto& key : recorded.mKeyframes)
            changed |= key.mMotorTarget->mBodyTransforms[1].mRotation.GetY() != 0.0f;
        EXPECT_TRUE(changed);
        const auto replay = run(recorded);
        ASSERT_TRUE(replay.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(feedback, replay));
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(replay, run(recorded)));
        EXPECT_FALSE(haveIdenticalScenarioTelemetry(feedback, run(script)));
        for (std::size_t step = 80; step < feedback.mFrames.size(); ++step)
        {
            EXPECT_FALSE(feedback.mFrames[step].mMotorsEnabled);
            EXPECT_FALSE(feedback.mFrames[step].mMotorsApplied);
        }
    }

    TEST(RagdollCoreScenarioInputTest, generatedInputPrecedesPhysicsAndLaterScriptCanReplaceIt)
    {
        ScenarioScript script;
        script.mDurationSeconds = 0.5f;
        for (std::uint64_t step : { 10u, 20u })
        {
            ScenarioInputKeyframe key;
            key.mStep = step;
            key.mMotorsEnabled = false;
            key.mMotorTarget = bindPose();
            key.mMotorTarget->mBodyTransforms[1].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.1f);
            script.mKeyframes.push_back(key);
        }
        const auto result = run(script, [](std::uint64_t step, const ScenarioFrame*)
            -> std::optional<ScenarioInputKeyframe> {
            if (step != 10)
                return std::nullopt;
            ScenarioInputKeyframe key;
            key.mStep = step;
            return key;
        });
        ASSERT_TRUE(result.completed());
        EXPECT_TRUE(result.mFrames[10].mMotorsEnabled);
        EXPECT_TRUE(result.mFrames[19].mMotorsEnabled);
        EXPECT_FALSE(result.mFrames[20].mMotorsEnabled);
        EXPECT_FALSE(result.mFrames[20].mMotorsApplied);
        // The generated step-10 key replaces the whole scripted key, including
        // its pose. Replaying the merged keys must not lose a hidden pose change.
        script.mKeyframes[0] = ScenarioInputKeyframe{};
        script.mKeyframes[0].mStep = 10;
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, run(script)));
    }

    TEST(RagdollCoreScenarioInputTest, observedLiftTriggersOneReplayableImpulse)
    {
        ScenarioSettings settings;
        settings.mLocomotionController = ScenarioLocomotionController::Reference;
        ScenarioScript script;
        script.mDurationSeconds = 1.6f;
        ScenarioInputKeyframe walking;
        walking.mStep = 30;
        walking.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
        script.mKeyframes.push_back(walking);
        auto recorded = script;
        std::optional<std::uint64_t> impulseStep;
        const auto simulate = [&](const ScenarioScript& input, const ScenarioInputProvider& provider = {}) {
            return runRagdollScenario(makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(),
                settings, input, provider);
        };
        const auto feedback = simulate(script, [&](std::uint64_t step, const ScenarioFrame* previous)
            -> std::optional<ScenarioInputKeyframe> {
            if (impulseStep || !previous || !previous->mGait.mSwingFootLifted)
                return std::nullopt;
            EXPECT_EQ(previous->mStep + 1, step);
            impulseStep = step;
            auto key = walking;
            key.mStep = step;
            key.mImpulses.push_back({ step, 3, JPH::Vec3(150, 0, 0), previous->mBodies[3].mWorldPosition });
            recorded.mKeyframes.push_back(key);
            return key;
        });
        ASSERT_TRUE(feedback.completed());
        ASSERT_TRUE(impulseStep);
        for (const auto& frame : feedback.mFrames)
            EXPECT_EQ(frame.mImpulseCount, frame.mStep == *impulseStep ? 1u : 0u);
        const auto replay = simulate(recorded);
        ASSERT_TRUE(replay.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(feedback, replay));
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(replay, simulate(recorded)));
        EXPECT_FALSE(haveIdenticalScenarioTelemetry(feedback, simulate(script)));
        // Resolve the event into the original fixed-step impulse schedule.
        // Both entry points must apply exactly the same physical disturbance.
        recorded.mImpulses = recorded.mKeyframes.back().mImpulses;
        recorded.mKeyframes.back().mImpulses.clear();
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(feedback, simulate(recorded)));
    }

    TEST(RagdollCoreScenarioInputTest, keyframeImpulsesApplyOnceAndReplacementCancelsThem)
    {
        ScenarioScript script;
        script.mDurationSeconds = 0.5f;
        script.mImpulses.push_back({ 10, 5, JPH::Vec3(40, 0, 0), std::nullopt });
        ScenarioInputKeyframe key;
        key.mStep = 10;
        key.mImpulses.push_back({ 10, 0, JPH::Vec3(100, 0, 0), JPH::RVec3(0, 2, 65) });
        key.mImpulses.push_back({ 10, 3, JPH::Vec3(0, 50, 0), std::nullopt });
        script.mKeyframes.push_back(key);
        const auto result = run(script);
        ASSERT_TRUE(result.completed());
        for (const auto& frame : result.mFrames)
            EXPECT_EQ(frame.mImpulseCount, frame.mStep == 10 ? 3u : 0u);
        auto fixed = script;
        fixed.mImpulses.insert(fixed.mImpulses.end(), key.mImpulses.begin(), key.mImpulses.end());
        fixed.mKeyframes.front().mImpulses.clear();
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, run(fixed)));

        const auto replaced = run(script, [](std::uint64_t step, const ScenarioFrame*)
            -> std::optional<ScenarioInputKeyframe> {
            if (step != 10)
                return std::nullopt;
            ScenarioInputKeyframe replacement;
            replacement.mStep = step;
            return replacement;
        });
        ASSERT_TRUE(replaced.completed());
        // The independent script-level impulse remains; the overridden
        // keyframe contributes neither of its two impulses.
        EXPECT_EQ(replaced.mFrames[10].mImpulseCount, 1u);
        script.mKeyframes.front().mImpulses.clear();
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(replaced, run(script)));
    }

    TEST(RagdollCoreScenarioInputTest, keyframeImpulseRemainsPhysicalWithMotorsReleased)
    {
        ScenarioScript script;
        script.mDurationSeconds = 0.5f;
        ScenarioInputKeyframe key;
        key.mStep = 10;
        key.mMotorsEnabled = false;
        key.mImpulses.push_back({ 10, 0, JPH::Vec3(250, 0, 0), std::nullopt });
        script.mKeyframes.push_back(key);
        const auto pushed = run(script);
        ASSERT_TRUE(pushed.completed());
        for (std::size_t step = 10; step < pushed.mFrames.size(); ++step)
        {
            EXPECT_FALSE(pushed.mFrames[step].mMotorsApplied);
            EXPECT_EQ(pushed.mFrames[step].mImpulseCount, step == 10 ? 1u : 0u);
        }
        auto fixed = script;
        fixed.mImpulses = key.mImpulses;
        fixed.mKeyframes.front().mImpulses.clear();
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(pushed, run(fixed)));
        fixed.mImpulses.clear();
        const auto unpushed = run(fixed);
        ASSERT_TRUE(unpushed.completed());
        EXPECT_GT((pushed.mFrames[10].mCenterOfMassVelocity
            - unpushed.mFrames[10].mCenterOfMassVelocity).Length(), 0.1f);
    }

    TEST(RagdollCoreScenarioInputTest, malformedImpulseBatchStopsBeforeAnyDisturbance)
    {
        for (int invalid = 0; invalid < 5; ++invalid)
        {
            SCOPED_TRACE(invalid);
            ScenarioScript script;
            script.mDurationSeconds = 0.5f;
            ScenarioInputKeyframe key;
            key.mStep = 10;
            key.mImpulses.push_back({ 10, 0, JPH::Vec3(100, 0, 0), std::nullopt });
            auto bad = key.mImpulses.front();
            const float nan = std::numeric_limits<float>::quiet_NaN();
            switch (invalid)
            {
                case 0: bad.mBodyIndex = makeScenarioHumanoidProfile().mBodies.size(); break;
                case 1: bad.mImpulse.SetX(nan); break;
                case 2: bad.mWorldPoint = JPH::RVec3(nan, 0, 0); break;
                case 3: bad.mStep = 9; break;
                case 4: bad.mStep = 11; break;
            }
            key.mImpulses.push_back(bad);
            const auto result = run(script, [&](std::uint64_t step, const ScenarioFrame*)
                -> std::optional<ScenarioInputKeyframe> {
                return step == 10 ? std::optional(key) : std::nullopt;
            });
            EXPECT_FALSE(result.completed());
            ASSERT_EQ(result.mFrames.size(), 10u);
            for (const auto& frame : result.mFrames)
                EXPECT_EQ(frame.mImpulseCount, 0u);
            script.mKeyframes.push_back(key);
            const auto scripted = run(script);
            EXPECT_FALSE(scripted.completed());
            EXPECT_TRUE(scripted.mFrames.empty());
        }
    }

    TEST(RagdollCoreScenarioInputTest, malformedFeedbackStopsBeforeItsPhysicsStep)
    {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        for (int invalid = 0; invalid < 7; ++invalid)
        {
            SCOPED_TRACE(invalid);
            ScenarioScript script;
            script.mDurationSeconds = 0.5f;
            std::uint64_t calls = 0;
            const auto result = run(script, [&](std::uint64_t step, const ScenarioFrame*)
                -> std::optional<ScenarioInputKeyframe> {
                ++calls;
                if (step != 10)
                    return std::nullopt;
                ScenarioInputKeyframe key;
                key.mStep = step;
                key.mMotorTarget = bindPose();
                switch (invalid)
                {
                    case 0: key.mStep = step + 1; break;
                    case 1: key.mRequestedHorizontalVelocity.SetX(nan); break;
                    case 2: key.mMotorTarget->mBodyTransforms.pop_back(); break;
                    case 3: key.mMotorTarget->mBodyTransforms[1].mTranslation.SetX(nan); break;
                    case 4: key.mMotorTarget->mBodyTransforms[1].mRotation = JPH::Quat(0, 0, 0, 2); break;
                    case 5:
                        key.mSupportMotion.emplace();
                        key.mSupportMotion->mAcceleration.SetX(nan);
                        break;
                    case 6:
                        key.mSupportMotion.emplace();
                        key.mSupportMotion->mSupportFoot = 0;
                        break;
                }
                return key;
            });
            EXPECT_FALSE(result.completed());
            EXPECT_FALSE(result.mErrors.empty());
            EXPECT_EQ(result.mFrames.size(), 10u);
            EXPECT_EQ(calls, 11u);
        }
    }
}
