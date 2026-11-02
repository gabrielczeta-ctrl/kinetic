#include <ragdollcore/reactionpipeline.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        float appliedMagnitude(const ReactionContractInput& input, bool contactEvent)
        {
            if (!contactEvent || !input.mContact.mValid
                || input.mImpulse.mProvenance == ImpulseProvenance::None)
                return 0.0f;
            return std::isfinite(input.mImpulse.mMagnitude) && input.mImpulse.mMagnitude > 0.0f
                ? input.mImpulse.mMagnitude : 0.0f;
        }

        ReactionPhase phaseFor(const ReactionContractInput& input, const ReactionOutputs& outputs,
            float poseAuthority)
        {
            if (input.mMode == ReactionBodyMode::Recovery)
                return ReactionPhase::GetUp;
            if (outputs.mBehavior != BehaviorId::None)
                return ReactionPhase::Stagger;
            if (!input.mSupport.mSupportEstablished && poseAuthority > 0.5f)
                return ReactionPhase::Catch;
            if (input.mMode == ReactionBodyMode::Passive)
                return ReactionPhase::Fall;
            return ReactionPhase::Idle;
        }

        void rotateLocal(Transform& transform, const JPH::Vec3& axis, float angle)
        {
            if (std::abs(angle) <= 1e-6f || axis.LengthSq() < 0.5f)
                return;
            transform.mRotation = (transform.mRotation
                * JPH::Quat::sRotation(axis.Normalized(), angle)).Normalized();
        }
    }

    ReactionOutputs stepReactionPipeline(ReactionPipelineState& state,
        const ReactionContractInput& input, const ReactionPipelineProfile& profile)
    {
        ReactionOutputs outputs;
        if (!profile.mEnabled)
            return outputs;

        bool processFrame = validContractInput(input);
        if (!processFrame)
            outputs.mStaleIgnored = true;

        if (processFrame && state.mFrameSeeded && input.mFramePhysicsStep <= state.mLastFrameStep)
        {
            outputs.mDuplicateIgnored = true;
            processFrame = false;
        }
        const float duration = processFrame && std::isfinite(input.mFixedStepDuration)
            ? std::max(0.0f, input.mFixedStepDuration) : 0.0f;
        if (processFrame)
        {
            state.mLastFrameStep = input.mFramePhysicsStep;
            state.mFrameSeeded = true;
        }

        bool contactEvent = false;
        bool receiptEvent = false;
        if (processFrame && input.mContact.mValid && input.mContact.mContactId != 0)
        {
            const ReactionPipelineState::ServedContact identity{
                input.mContact.mAttackerActorIdentity,
                input.mContact.mTargetActorIdentity,
                input.mContact.mTargetRegistryGeneration,
                input.mContact.mContactId,
            };
            const bool inserted = state.mServedContacts.insert(identity).second;
            if (!inserted)
                outputs.mDuplicateIgnored = true;
            else
            {
                contactEvent = true;
                outputs.mServedContactId = input.mContact.mContactId;
            }
        }
        if (processFrame && input.mReceipt)
        {
            const std::uint64_t receiptId = input.mReceipt->mReceiptId;
            if (receiptId != 0 && receiptId <= state.mLastServedReceiptId)
                outputs.mStaleIgnored = true;
            else
            {
                receiptEvent = true;
                if (receiptId != 0)
                    state.mLastServedReceiptId = receiptId;
                outputs.mServedReceiptId = receiptId;
            }
        }

        // Behavior policy.
        BehaviorInput behaviorInput;
        behaviorInput.mDuration = duration;
        behaviorInput.mRegion = input.mContact.mRegion;
        behaviorInput.mImpulse = appliedMagnitude(input, contactEvent);
        if (receiptEvent && (input.mWoundSeverity || !input.mRegionCapable)
            && (regionIsArm(input.mContact.mRegion) || regionIsLeg(input.mContact.mRegion)))
        {
            const std::size_t limb = (regionIsLeg(input.mContact.mRegion) ? 2 : 0)
                + (regionIsLeft(input.mContact.mRegion) ? 0 : 1);
            auto& wound = state.mWounds[limb];
            wound = { input.mWoundSeverity.value_or(wound.mSeverity), input.mRegionCapable, true };
        }
        behaviorInput.mWalking = input.mWalking;
        behaviorInput.mMoving = input.mMoving;
        behaviorInput.mOwned = input.mOwned;
        BehaviorOutput behavior = stepBehavior(state.mBehavior, behaviorInput, profile.mBehavior);
        constexpr std::array regions{ AnatomicalRegion::LeftForearm, AnatomicalRegion::RightForearm,
            AnatomicalRegion::LeftCalf, AnatomicalRegion::RightCalf };
        if (profile.mBehavior.mEnabled && !input.mOwned.mRecoveryBrace)
        {
            for (std::size_t limb = 0; limb < state.mWounds.size(); ++limb)
            {
                const auto& wound = state.mWounds[limb];
                if (!wound.mKnown)
                    continue;
                BehaviorInput persistent;
                persistent.mRegion = regions[limb];
                persistent.mHasReceipt = true; // cached, already consumed; no new damage/event
                persistent.mWoundSeverity = wound.mSeverity;
                persistent.mRegionCapable = wound.mCapable;
                persistent.mOwned = input.mOwned;
                BehaviorState limbState;
                const auto output = stepBehavior(limbState, persistent, profile.mBehavior);
                behavior.mCradleLeft |= output.mCradleLeft;
                behavior.mCradleRight |= output.mCradleRight;
                behavior.mLimpLeft |= output.mLimpLeft;
                behavior.mLimpRight |= output.mLimpRight;
                behavior.mLimpBias = std::max(behavior.mLimpBias, output.mLimpBias);
                behavior.mSpeedScale = std::min(behavior.mSpeedScale, output.mSpeedScale);
                behavior.mDisabledLimbs.mLeftArm |= output.mDisabledLimbs.mLeftArm;
                behavior.mDisabledLimbs.mRightArm |= output.mDisabledLimbs.mRightArm;
                behavior.mDisabledLimbs.mLeftLeg |= output.mDisabledLimbs.mLeftLeg;
                behavior.mDisabledLimbs.mRightLeg |= output.mDisabledLimbs.mRightLeg;
                if (behavior.mBehavior == BehaviorId::None)
                {
                    if (output.mCradleLeft || output.mCradleRight)
                        behavior.mBehavior = BehaviorId::ArmCradle;
                    else if (output.mLimpLeft || output.mLimpRight)
                        behavior.mBehavior = BehaviorId::Limp;
                    if (behavior.mBehavior != BehaviorId::None)
                        behavior.mBehaviorIntensity = wound.mCapable ? wound.mSeverity : 1.0f;
                }
            }
        }

        // Pose-correction authority: rises with disturbance and support loss.
        if (contactEvent)
            state.mLatch.begin(appliedMagnitude(input, contactEvent), profile.mBalance);
        state.mLatch.step(duration, profile.mBalance);
        ReactionAuthorityInput balanceInput;
        balanceInput.mSupportEstablished = input.mSupport.mSupportEstablished;
        balanceInput.mBothFeetSupported = input.mSupport.mBothFeetSupported;
        balanceInput.mWalking = input.mWalking;
        balanceInput.mDisturbanceImpulse = state.mLatch.retained();
        balanceInput.mDuration = duration;
        const float poseAuthority = stepReactionAuthority(state.mBalance, balanceInput, profile.mBalance);

        outputs.mBehavior = behavior.mBehavior;
        outputs.mTorsoFlexion = behavior.mTorsoFlexion;
        outputs.mHeadRecoil = behavior.mHeadRecoil;
        outputs.mLeftArmWithdrawal = behavior.mLeftArmWithdrawal;
        outputs.mRightArmWithdrawal = behavior.mRightArmWithdrawal;
        outputs.mCoverFace = behavior.mCoverFace;
        outputs.mCradleLeft = behavior.mCradleLeft;
        outputs.mCradleRight = behavior.mCradleRight;
        outputs.mLimpBias = behavior.mLimpBias;
        outputs.mLimpLeft = behavior.mLimpLeft;
        outputs.mLimpRight = behavior.mLimpRight;
        outputs.mSpeedScale = behavior.mSpeedScale;
        outputs.mDisabledLimbs = behavior.mDisabledLimbs;
        outputs.mPoseCorrectionAuthority = poseAuthority;
        outputs.mStabilizationStrength = behavior.mStabilizationStrength;
        if (input.mImpulse.mDirection)
        {
            JPH::Vec3 direction = *input.mImpulse.mDirection;
            direction.SetZ(0.0f);
            if (!direction.IsNearZero())
            {
                outputs.mCounterbalanceDirection = direction.Normalized();
                outputs.mCounterbalanceStrength = std::clamp(
                    reactionAuthorityRamp(state.mLatch.retained(), profile.mBalance.mImpulseEngage,
                        profile.mBalance.mImpulseFull), 0.0f, 1.0f);
            }
        }
        outputs.mReactionTime = behavior.mBehaviorTime;
        outputs.mReactionIntensity = behavior.mBehaviorIntensity;
        outputs.mPhase = phaseFor(input, outputs, poseAuthority);
        return outputs;
    }

    ReactionPoseTargets makeReactionPoseTargets(const RagdollProfile& profile)
    {
        const auto find = [&](const std::string& name) {
            for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
                if (profile.mBodies[i].mName == name)
                    return i;
            return static_cast<std::size_t>(-1);
        };
        ReactionPoseTargets targets;
        targets.mSpine = find("spine");
        targets.mSpine1 = find("spine1");
        targets.mSpine2 = find("spine2");
        targets.mNeck = find("neck");
        targets.mHead = find("head");
        targets.mLeftUpperArm = find("left_upper_arm");
        targets.mRightUpperArm = find("right_upper_arm");
        targets.mLeftForearm = find("left_forearm");
        targets.mRightForearm = find("right_forearm");
        targets.mLeftHand = find("left_hand");
        targets.mRightHand = find("right_hand");
        for (const auto& body : profile.mBodies)
        {
            const auto parent = body.mParentBody ? find(*body.mParentBody) : profile.mBodies.size();
            targets.mParents.push_back(!body.mParentBody ? -1
                : parent < profile.mBodies.size() ? static_cast<int>(parent) : -2);
        }
        return targets;
    }

    bool applyReactionPose(std::span<const Transform> pose, const ReactionOutputs& outputs,
        const ReactionPoseTargets& targets, std::span<Transform> out)
    {
        if (pose.size() != out.size())
            return false;
        if (!targets.mParents.empty())
        {
            if (targets.mParents.size() != pose.size() || !std::isfinite(targets.mExpressionScale))
                return false;
            for (std::size_t i = 0; i < pose.size(); ++i)
                if (targets.mParents[i] < -1 || targets.mParents[i] >= static_cast<int>(i))
                    return false;
        }
        bool applied = false;
        if (pose.data() != out.data())
            std::copy(pose.begin(), pose.end(), out.begin());
        const auto valid = [&](std::size_t index) { return index < out.size(); };
        if (!targets.mParents.empty())
        {
            if (!valid(targets.mLeftUpperArm) || !valid(targets.mRightUpperArm)
                || !valid(targets.mSpine) || !valid(targets.mNeck))
                return false;
            const auto right = (out[targets.mRightUpperArm].mTranslation
                - out[targets.mLeftUpperArm].mTranslation).NormalizedOr(JPH::Vec3::sAxisX());
            const auto up = (out[targets.mNeck].mTranslation - out[targets.mSpine].mTranslation)
                .NormalizedOr(JPH::Vec3::sAxisZ());
            const auto forward = up.Cross(right).NormalizedOr(JPH::Vec3::sAxisY());
            const auto leftBaseRotation = out[targets.mLeftUpperArm].mRotation;
            const auto rightBaseRotation = out[targets.mRightUpperArm].mRotation;
            const float scale = std::clamp(targets.mExpressionScale, 0.0f, 1.2f);
            const auto rotateBranch = [&](std::size_t root, JPH::Vec3Arg axis, float angle) {
                if (!valid(root) || targets.mParents[root] < 0 || !std::isfinite(angle)
                    || std::abs(angle) < 1e-6f)
                    return;
                const auto rotation = JPH::Quat::sRotation(axis, angle);
                const auto pivot = out[root].mTranslation;
                for (std::size_t b = root; b < out.size(); ++b)
                {
                    int ancestor = static_cast<int>(b);
                    while (ancestor > static_cast<int>(root))
                        ancestor = targets.mParents[ancestor];
                    if (ancestor != static_cast<int>(root))
                        continue;
                    out[b].mTranslation = pivot + rotation * (out[b].mTranslation - pivot);
                    out[b].mRotation = (rotation * out[b].mRotation).Normalized();
                }
                applied = true;
            };
            const float flex = std::clamp(outputs.mTorsoFlexion * scale, 0.0f, 0.54f);
            const std::array trunk{ targets.mSpine, targets.mSpine1, targets.mSpine2 };
            const auto trunkCount = std::count_if(trunk.begin(), trunk.end(), valid);
            for (const auto b : trunk)
                if (valid(b))
                    rotateBranch(b, right, -flex / static_cast<float>(trunkCount));
            if (outputs.mCounterbalanceStrength > 0.0f && !outputs.mCounterbalanceDirection.IsNearZero())
            {
                const auto momentum = outputs.mCounterbalanceDirection.Normalized();
                const auto counterAxis = up.Cross(momentum).NormalizedOr(JPH::Vec3::sAxisX());
                const float counterAngle = -0.22f * std::clamp(outputs.mCounterbalanceStrength, 0.0f, 1.0f);
                for (const auto b : trunk)
                    if (valid(b))
                        rotateBranch(b, counterAxis, counterAngle / static_cast<float>(trunkCount));
            }
            rotateBranch(targets.mNeck, right, std::clamp(outputs.mHeadRecoil * scale, 0.0f, 0.3f));
            const float intensity = std::clamp(outputs.mReactionIntensity, 0.0f, 1.0f) * scale;
            const auto arm = [&](std::size_t upper, std::size_t elbow, bool left,
                                 float withdrawal, bool cradle, bool disabled) {
                if (disabled && !cradle)
                    return;
                const float side = left ? -1.0f : 1.0f;
                float raise = std::clamp(withdrawal * scale, 0.0f, 0.8f);
                float bend = 0.0f;
                float inward = 0.0f;
                float outward = 0.0f;
                if (outputs.mCoverFace)
                {
                    bend = std::min(1.9f, 1.8f * intensity);
                    inward = side * 0.20f * intensity;
                }
                else if (outputs.mBehavior == BehaviorId::DoubledOver)
                {
                    bend = std::min(1.9f, 1.65f * intensity);
                    inward = side * 0.35f * intensity;
                }
                else if (outputs.mBehavior == BehaviorId::StumbleFlail)
                {
                    // An open catch is distinct from a folded guard. Flexing
                    // the elbows here shortened the lateral reach precisely
                    // when the struck arm was already moving outward.
                    raise = 0.15f * intensity;
                    outward = -side * std::min(0.85f, 0.80f * intensity);
                    bend = 0.10f * intensity;
                }
                if (cradle)
                {
                    raise = 0.30f * scale;
                    bend = std::min(1.9f, 1.75f * scale);
                    inward = side * 0.45f * scale;
                    outward = 0.0f;
                }
                rotateBranch(upper, right, raise);
                rotateBranch(upper, forward, outward);
                rotateBranch(upper, up, inward);
                // Bend about the arm's anatomical hinge, which is carried by
                // its shoulder. A bone-local X axis is not the elbow axis.
                if (valid(upper) && valid(elbow))
                {
                    const auto upperDelta = (out[upper].mRotation
                        * (left ? leftBaseRotation : rightBaseRotation).Conjugated()).Normalized();
                    rotateBranch(elbow, upperDelta * right, bend);
                }
            };
            arm(targets.mLeftUpperArm, targets.mLeftForearm, true, outputs.mLeftArmWithdrawal,
                outputs.mCradleLeft, outputs.mDisabledLimbs.mLeftArm);
            arm(targets.mRightUpperArm, targets.mRightForearm, false, outputs.mRightArmWithdrawal,
                outputs.mCradleRight, outputs.mDisabledLimbs.mRightArm);
            return applied;
        }
        if (valid(targets.mSpine))
        {
            rotateLocal(out[targets.mSpine], JPH::Vec3::sAxisX(), outputs.mTorsoFlexion);
            applied = applied || outputs.mTorsoFlexion > 0.0f;
        }
        if (valid(targets.mNeck))
        {
            rotateLocal(out[targets.mNeck], JPH::Vec3::sAxisX(), outputs.mHeadRecoil);
            applied = applied || outputs.mHeadRecoil > 0.0f;
        }
        const auto applyArm = [&](std::size_t index, float withdrawal, bool cradle, float cradleSign) {
            if (!valid(index))
                return;
            rotateLocal(out[index], JPH::Vec3::sAxisX(), withdrawal);
            if (cradle)
                rotateLocal(out[index], JPH::Vec3::sAxisZ(), cradleSign * 0.5f);
            applied = applied || withdrawal > 0.0f || cradle;
        };
        applyArm(targets.mLeftUpperArm, outputs.mLeftArmWithdrawal, outputs.mCradleLeft, -1.0f);
        applyArm(targets.mRightUpperArm, outputs.mRightArmWithdrawal, outputs.mCradleRight, 1.0f);
        return applied;
    }
}
