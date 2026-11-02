#include <ragdollcore/replay.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <string_view>
#include <type_traits>

namespace RagdollCore
{
    namespace
    {
        constexpr std::array<std::byte, 8> sMagic{
            std::byte{ 'O' },
            std::byte{ 'M' },
            std::byte{ 'W' },
            std::byte{ 'R' },
            std::byte{ 'D' },
            std::byte{ 'P' },
            std::byte{ 'L' },
            std::byte{ 0 },
        };
        bool isFinite(JPH::Vec3Arg value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        }

        bool isFinite(const ReplayBodyState& value)
        {
            return isFinite(value.mTransform.mTranslation) && isFinite(value.mLinearVelocity)
                && isFinite(value.mAngularVelocity) && std::isfinite(value.mTransform.mRotation.GetX())
                && std::isfinite(value.mTransform.mRotation.GetY()) && std::isfinite(value.mTransform.mRotation.GetZ())
                && std::isfinite(value.mTransform.mRotation.GetW()) && value.mTransform.mRotation.IsNormalized(1e-3f);
        }

        bool isFinite(const Transform& value)
        {
            return isFinite(value.mTranslation) && std::isfinite(value.mRotation.GetX())
                && std::isfinite(value.mRotation.GetY()) && std::isfinite(value.mRotation.GetZ())
                && std::isfinite(value.mRotation.GetW()) && value.mRotation.IsNormalized(1e-3f);
        }

        bool isFiniteAndNonNegative(float value)
        {
            return std::isfinite(value) && value >= 0.0f;
        }

        float quaternionAngle(JPH::QuatArg expected, JPH::QuatArg actual)
        {
            const bool identical = expected.GetX() == actual.GetX() && expected.GetY() == actual.GetY()
                && expected.GetZ() == actual.GetZ() && expected.GetW() == actual.GetW();
            const bool antipodal = expected.GetX() == -actual.GetX() && expected.GetY() == -actual.GetY()
                && expected.GetZ() == -actual.GetZ() && expected.GetW() == -actual.GetW();
            if (identical || antipodal)
                return 0.0f;
            const float dot = expected.GetX() * actual.GetX() + expected.GetY() * actual.GetY()
                + expected.GetZ() * actual.GetZ() + expected.GetW() * actual.GetW();
            return 2.0f * std::acos(std::clamp(std::abs(dot), 0.0f, 1.0f));
        }

        class HashBuilder
        {
        public:
            void addByte(std::uint8_t value)
            {
                mValue ^= value;
                mValue *= 1099511628211ull;
            }

            template <class T>
            void addUnsigned(T value)
            {
                static_assert(std::is_unsigned_v<T>);
                for (std::size_t i = 0; i < sizeof(T); ++i)
                    addByte(static_cast<std::uint8_t>(value >> (i * 8)));
            }

            void addBool(bool value) { addByte(value ? 1 : 0); }

            void addFloat(float value) { addUnsigned(value == 0.0f ? 0u : std::bit_cast<std::uint32_t>(value)); }

            void addString(std::string_view value)
            {
                addUnsigned(static_cast<std::uint64_t>(value.size()));
                for (const char byte : value)
                    addByte(static_cast<std::uint8_t>(byte));
            }

            void addVector(JPH::Vec3Arg value)
            {
                addFloat(value.GetX());
                addFloat(value.GetY());
                addFloat(value.GetZ());
            }

            void addQuaternion(JPH::QuatArg value)
            {
                addFloat(value.GetX());
                addFloat(value.GetY());
                addFloat(value.GetZ());
                addFloat(value.GetW());
            }

            void addTransform(const Transform& value)
            {
                addVector(value.mTranslation);
                addQuaternion(value.mRotation);
            }

            void addOptionalString(const std::optional<std::string>& value)
            {
                addBool(value.has_value());
                if (value)
                    addString(*value);
            }

            std::uint64_t value() const { return mValue; }

        private:
            std::uint64_t mValue = 14695981039346656037ull;
        };

        class Writer
        {
        public:
            void addByte(std::byte value) { mData.push_back(value); }

            template <class T>
            void addUnsigned(T value)
            {
                static_assert(std::is_unsigned_v<T>);
                for (std::size_t i = 0; i < sizeof(T); ++i)
                    addByte(static_cast<std::byte>(value >> (i * 8)));
            }

            void addFloat(float value) { addUnsigned(std::bit_cast<std::uint32_t>(value)); }

            void addVector(JPH::Vec3Arg value)
            {
                addFloat(value.GetX());
                addFloat(value.GetY());
                addFloat(value.GetZ());
            }

            void addQuaternion(JPH::QuatArg value)
            {
                addFloat(value.GetX());
                addFloat(value.GetY());
                addFloat(value.GetZ());
                addFloat(value.GetW());
            }

            std::vector<std::byte> take() { return std::move(mData); }

        private:
            std::vector<std::byte> mData;
        };

        class Reader
        {
        public:
            explicit Reader(std::span<const std::byte> data)
                : mData(data)
            {
            }

            bool readByte(std::byte& value)
            {
                if (mOffset >= mData.size())
                    return false;
                value = mData[mOffset++];
                return true;
            }

            template <class T>
            bool readUnsigned(T& value)
            {
                static_assert(std::is_unsigned_v<T>);
                if (mData.size() - mOffset < sizeof(T))
                    return false;
                value = 0;
                for (std::size_t i = 0; i < sizeof(T); ++i)
                    value |= static_cast<T>(std::to_integer<std::uint8_t>(mData[mOffset++])) << (i * 8);
                return true;
            }

            bool readFloat(float& value)
            {
                std::uint32_t bits = 0;
                if (!readUnsigned(bits))
                    return false;
                value = std::bit_cast<float>(bits);
                return true;
            }

            bool readVector(JPH::Vec3& value)
            {
                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;
                if (!readFloat(x) || !readFloat(y) || !readFloat(z))
                    return false;
                value = JPH::Vec3(x, y, z);
                return true;
            }

            bool readQuaternion(JPH::Quat& value)
            {
                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;
                float w = 0.0f;
                if (!readFloat(x) || !readFloat(y) || !readFloat(z) || !readFloat(w))
                    return false;
                value = JPH::Quat(x, y, z, w);
                return true;
            }

            std::size_t remaining() const { return mData.size() - mOffset; }

        private:
            std::span<const std::byte> mData;
            std::size_t mOffset = 0;
        };
    }

    std::uint64_t hashProfile(const RagdollProfile& profile)
    {
        HashBuilder hash;
        hash.addUnsigned(profile.mSchemaVersion);
        hash.addString(profile.mId);
        hash.addUnsigned(static_cast<std::uint64_t>(profile.mBoneAliases.size()));
        for (const BoneAliases& aliases : profile.mBoneAliases)
        {
            hash.addString(aliases.mCanonical);
            hash.addUnsigned(static_cast<std::uint64_t>(aliases.mAliases.size()));
            for (const std::string& alias : aliases.mAliases)
                hash.addString(alias);
        }

        hash.addUnsigned(static_cast<std::uint64_t>(profile.mBodies.size()));
        for (const BodyProfile& body : profile.mBodies)
        {
            hash.addString(body.mName);
            hash.addString(body.mBone);
            hash.addOptionalString(body.mParentBody);
            hash.addUnsigned(static_cast<std::uint32_t>(body.mShape.mType));
            hash.addTransform(body.mShape.mLocalTransform);
            hash.addOptionalString(body.mShape.mFitToBone);
            hash.addVector(body.mShape.mHalfExtents);
            hash.addFloat(body.mShape.mRadius);
            hash.addFloat(body.mShape.mHalfHeight);
            // Append hull-only data conditionally: legacy shape hashes are
            // part of existing replay identities and must remain unchanged.
            if (body.mShape.mType == ShapeType::ConvexHull)
            {
                hash.addUnsigned(static_cast<std::uint64_t>(body.mShape.mPoints.size()));
                for (const JPH::Vec3& point : body.mShape.mPoints)
                    hash.addVector(point);
            }
            hash.addFloat(body.mMassFraction);
            hash.addVector(body.mCenterOfMass);
            hash.addFloat(body.mMaterial.mFriction);
            hash.addFloat(body.mMaterial.mRestitution);
            hash.addFloat(body.mMaterial.mLinearDamping);
            hash.addFloat(body.mMaterial.mAngularDamping);
            hash.addUnsigned(body.mSelfCollisionGroup);
            hash.addBool(body.mConstraint.has_value());
            if (body.mConstraint)
            {
                const ConstraintProfile& constraint = *body.mConstraint;
                hash.addUnsigned(static_cast<std::uint32_t>(constraint.mType));
                hash.addBool(constraint.mDeriveFrameFromSkeleton);
                hash.addOptionalString(constraint.mFrameAxisBone);
                hash.addVector(constraint.mParentAnchor);
                hash.addVector(constraint.mChildAnchor);
                hash.addVector(constraint.mParentTwistAxis);
                hash.addVector(constraint.mParentPlaneAxis);
                hash.addVector(constraint.mChildTwistAxis);
                hash.addVector(constraint.mChildPlaneAxis);
                hash.addFloat(constraint.mNormalHalfConeAngle);
                hash.addFloat(constraint.mPlaneHalfConeAngle);
                hash.addFloat(constraint.mTwistMinAngle);
                hash.addFloat(constraint.mTwistMaxAngle);
                hash.addFloat(constraint.mHingeMinAngle);
                hash.addFloat(constraint.mHingeMaxAngle);
                hash.addFloat(constraint.mMaxFrictionTorque);
                hash.addFloat(constraint.mMotor.mFrequency);
                hash.addFloat(constraint.mMotor.mDamping);
                hash.addFloat(constraint.mMotor.mMaximumTorque);
            }
        }

        hash.addString(profile.mLandmarks.mPelvis);
        hash.addUnsigned(static_cast<std::uint64_t>(profile.mLandmarks.mSpine.size()));
        for (const std::string& body : profile.mLandmarks.mSpine)
            hash.addString(body);
        hash.addOptionalString(profile.mLandmarks.mHead);
        hash.addOptionalString(profile.mLandmarks.mLeftHand);
        hash.addOptionalString(profile.mLandmarks.mRightHand);
        hash.addOptionalString(profile.mLandmarks.mLeftFoot);
        hash.addOptionalString(profile.mLandmarks.mRightFoot);
        hash.addFloat(profile.mImpactResponse.mImpulsePerDamage);
        hash.addFloat(profile.mImpactResponse.mMinimumImpulse);
        hash.addFloat(profile.mImpactResponse.mMaximumImpulse);
        hash.addFloat(profile.mImpactResponse.mUpwardBias);
        hash.addFloat(profile.mMotionTransfer.mMaximumSampleDuration);
        hash.addFloat(profile.mMotionTransfer.mMaximumLinearSpeed);
        hash.addFloat(profile.mMotionTransfer.mMaximumAngularSpeed);
        hash.addFloat(profile.mPoseTransition.mDeathBlendDuration);
        hash.addUnsigned(static_cast<std::uint64_t>(profile.mActiveController.mLocomotionRoute));
        hash.addFloat(profile.mActiveController.mAnimationAssistHorizontalScale);
        hash.addFloat(profile.mActiveController.mAnimationAssistVerticalScale);
        hash.addFloat(profile.mActiveController.mMaximumAnimationAssistForce);
        hash.addFloat(profile.mActiveController.mAnimationAssistUprightScale);
        hash.addFloat(profile.mActiveController.mMaximumAnimationAssistTorque);
        hash.addFloat(profile.mActiveController.mAnimationAssistYawReferenceMaximumRate);
        hash.addFloat(profile.mActiveController.mAnimationAssistYawReferenceMaximumAcceleration);
        hash.addFloat(profile.mActiveController.mAnimationAssistYawStiffness);
        hash.addFloat(profile.mActiveController.mAnimationAssistYawDamping);
        hash.addFloat(profile.mActiveController.mMaximumAnimationAssistYawTorque);
        hash.addFloat(profile.mActiveController.mAnimationAssistPoseTrackingBlend);
        hash.addFloat(profile.mActiveController.mAnimationAssistStanceAnchorCompliance);
        hash.addFloat(profile.mActiveController.mAnimationAssistSupportLossHandoffDuration);
        hash.addFloat(profile.mActiveController.mAnimationAssistStancePositionCorrection);
        hash.addFloat(profile.mActiveController.mAnimationAssistStanceMaximumEndpointCorrection);
        hash.addFloat(profile.mActiveController.mAnimationAssistSupportMotionFrequency);
        hash.addFloat(profile.mActiveController.mAnimationAssistSupportMotionDamping);
        hash.addFloat(profile.mActiveController.mAnimationAssistSupportPositionLeash);
        hash.addUnsigned(profile.mActiveController.mAnimationAssistContactConfirmationTicks);
        hash.addFloat(profile.mActiveController.mAnimationAssistLoadTransferDuration);
        hash.addFloat(profile.mActiveController.mAnimationAssistMeasuredProgressBlend);
        hash.addFloat(profile.mActiveController.mAnimationAssistFeasibilityDeadline);
        hash.addFloat(profile.mActiveController.mAnimationAssistDoubleSupportTransferDuration);
        hash.addFloat(profile.mActiveController.mAnimationAssistReentrySlipSettleDuration);
        hash.addFloat(profile.mActiveController.mAnimationAssistTouchdownCapturePositionCorrection);
        hash.addFloat(profile.mActiveController.mAnimationAssistTouchdownCaptureVelocityLookahead);
        hash.addFloat(profile.mActiveController.mAnimationAssistOutgoingHoldPositionCorrection);
        hash.addFloat(profile.mActiveController.mAnimationAssistOutgoingHoldVelocityLookahead);
        hash.addFloat(profile.mActiveController.mSupportNormalMinimumDot);
        hash.addFloat(profile.mActiveController.mSupportMargin);
        hash.addFloat(profile.mActiveController.mStandingMinimumUpDot);
        hash.addFloat(profile.mActiveController.mBracingMinimumUpDot);
        hash.addFloat(profile.mActiveController.mStandingMaximumSpeed);
        hash.addFloat(profile.mActiveController.mSettledMaximumSpeed);
        hash.addFloat(profile.mActiveController.mFallenMaximumPelvisHeight);
        hash.addFloat(profile.mActiveController.mMinimumStateDuration);
        hash.addFloat(profile.mActiveController.mFallingExitSupportDuration);
        hash.addFloat(profile.mActiveController.mFallingExitMaximumVerticalSpeed);
        hash.addFloat(profile.mActiveController.mRecoveryDelay);
        hash.addFloat(profile.mActiveController.mBlendBackDuration);
        hash.addFloat(profile.mActiveController.mPelvisHeightFrequency);
        hash.addFloat(profile.mActiveController.mPelvisHeightDamping);
        hash.addFloat(profile.mActiveController.mCenterOfMassFrequency);
        hash.addFloat(profile.mActiveController.mCenterOfMassDamping);
        hash.addFloat(profile.mActiveController.mMaximumLocomotionSpeed);
        hash.addFloat(profile.mActiveController.mPelvisHeightSettleDuration);
        hash.addFloat(profile.mActiveController.mPelvisHeightSettleMaximumVerticalSpeed);
        hash.addFloat(profile.mActiveController.mActivationSupportGraceDuration);
        hash.addFloat(profile.mActiveController.mLocomotionSupportGraceDuration);
        hash.addFloat(profile.mActiveController.mMaximumPelvisForce);
        hash.addFloat(profile.mActiveController.mRecoveryUnweightFactor);
        hash.addFloat(profile.mActiveController.mPelvisUprightStiffness);
        hash.addFloat(profile.mActiveController.mPelvisUprightDamping);
        hash.addFloat(profile.mActiveController.mMaximumPelvisTorque);
        hash.addFloat(profile.mActiveController.mSpineUprightStiffness);
        hash.addFloat(profile.mActiveController.mSpineUprightDamping);
        hash.addFloat(profile.mActiveController.mMaximumSpineTorque);
        hash.addFloat(profile.mActiveController.mGait.mMinimumStepDuration);
        hash.addFloat(profile.mActiveController.mGait.mNominalStepDuration);
        hash.addFloat(profile.mActiveController.mGait.mMaximumStepDuration);
        hash.addFloat(profile.mActiveController.mGait.mContactConfirmationDuration);
        hash.addFloat(profile.mActiveController.mGait.mStanceConfirmationDuration);
        hash.addFloat(profile.mActiveController.mGait.mMaximumTouchdownHorizontalDistance);
        hash.addFloat(profile.mActiveController.mGait.mMaximumLandingApproachVerticalDistance);
        hash.addFloat(profile.mActiveController.mGait.mMaximumTouchdownRelativeVerticalSpeed);
        hash.addFloat(profile.mActiveController.mGait.mMaximumStepReach);
        hash.addFloat(profile.mActiveController.mGait.mNominalStepWidth);
        hash.addFloat(profile.mActiveController.mGait.mCapturePointBlend);
        hash.addFloat(profile.mActiveController.mGait.mSwingFootLift);
        hash.addFloat(profile.mActiveController.mGait.mSwingFootFrequency);
        hash.addFloat(profile.mActiveController.mGait.mSwingFootDamping);
        hash.addFloat(profile.mActiveController.mGait.mMaximumSwingFootForce);
        hash.addFloat(profile.mActiveController.mGait.mMaximumSwingFootHorizontalForce);
        hash.addFloat(profile.mActiveController.mGait.mPreLiftSwingUnloadFraction);
        hash.addFloat(profile.mActiveController.mGait.mSwingLegMotorTorqueScale);
        hash.addFloat(profile.mActiveController.mGait.mMaximumSwingHipTargetSpeed);
        hash.addFloat(profile.mActiveController.mGait.mSwingHipWorldStiffness);
        hash.addFloat(profile.mActiveController.mGait.mSwingHipWorldDamping);
        hash.addFloat(profile.mActiveController.mGait.mMaximumSwingHipWorldTorque);
        return hash.value();
    }

    bool validateReplay(const Replay& replay)
    {
        if (replay.mMetadata.mVersion != CurrentReplayVersion || !std::isfinite(replay.mMetadata.mFixedTimeStep)
            || replay.mMetadata.mFixedTimeStep <= 0.0f || replay.mMetadata.mWorkerThreadCount == 0
            || replay.mMetadata.mCollisionSteps == 0 || replay.mInputCount > MaximumReplayChannels
            || replay.mControllerStateCount > MaximumReplayChannels || replay.mBodyCount == 0
            || replay.mBodyCount > MaximumReplayBodies || replay.mFrames.size() > MaximumReplayFrames)
            return false;

        for (std::size_t frameIndex = 0; frameIndex < replay.mFrames.size(); ++frameIndex)
        {
            const ReplayFrame& frame = replay.mFrames[frameIndex];
            if (frame.mStepIndex != frameIndex || frame.mInputs.size() != replay.mInputCount
                || frame.mControllerState.size() != replay.mControllerStateCount
                || frame.mBodies.size() != replay.mBodyCount)
                return false;
            for (const float value : frame.mInputs)
            {
                if (!std::isfinite(value))
                    return false;
            }
            for (const float value : frame.mControllerState)
            {
                if (!std::isfinite(value))
                    return false;
            }
            for (const ReplayBodyState& body : frame.mBodies)
            {
                if (!isFinite(body))
                    return false;
            }
        }
        return true;
    }

    std::optional<std::vector<std::byte>> serializeReplay(const Replay& replay)
    {
        if (!validateReplay(replay))
            return std::nullopt;

        Writer writer;
        for (const std::byte value : sMagic)
            writer.addByte(value);
        writer.addUnsigned(replay.mMetadata.mVersion);
        writer.addUnsigned(replay.mMetadata.mSeed);
        writer.addUnsigned(replay.mMetadata.mProfileHash);
        writer.addFloat(replay.mMetadata.mFixedTimeStep);
        writer.addUnsigned(replay.mMetadata.mWorkerThreadCount);
        writer.addUnsigned(replay.mMetadata.mCollisionSteps);
        writer.addUnsigned(replay.mInputCount);
        writer.addUnsigned(replay.mControllerStateCount);
        writer.addUnsigned(replay.mBodyCount);
        writer.addUnsigned(static_cast<std::uint32_t>(replay.mFrames.size()));
        for (const ReplayFrame& frame : replay.mFrames)
        {
            writer.addUnsigned(frame.mStepIndex);
            for (const float value : frame.mInputs)
                writer.addFloat(value);
            for (const float value : frame.mControllerState)
                writer.addFloat(value);
            for (const ReplayBodyState& body : frame.mBodies)
            {
                writer.addVector(body.mTransform.mTranslation);
                writer.addQuaternion(body.mTransform.mRotation);
                writer.addVector(body.mLinearVelocity);
                writer.addVector(body.mAngularVelocity);
            }
        }
        return writer.take();
    }

    std::optional<Replay> deserializeReplay(std::span<const std::byte> data)
    {
        Reader reader(data);
        for (const std::byte expected : sMagic)
        {
            std::byte actual{};
            if (!reader.readByte(actual) || actual != expected)
                return std::nullopt;
        }

        Replay replay;
        std::uint32_t frameCount = 0;
        if (!reader.readUnsigned(replay.mMetadata.mVersion) || !reader.readUnsigned(replay.mMetadata.mSeed)
            || !reader.readUnsigned(replay.mMetadata.mProfileHash) || !reader.readFloat(replay.mMetadata.mFixedTimeStep)
            || !reader.readUnsigned(replay.mMetadata.mWorkerThreadCount)
            || !reader.readUnsigned(replay.mMetadata.mCollisionSteps) || !reader.readUnsigned(replay.mInputCount)
            || !reader.readUnsigned(replay.mControllerStateCount) || !reader.readUnsigned(replay.mBodyCount)
            || !reader.readUnsigned(frameCount) || replay.mInputCount > MaximumReplayChannels
            || replay.mControllerStateCount > MaximumReplayChannels || replay.mBodyCount > MaximumReplayBodies
            || frameCount > MaximumReplayFrames)
            return std::nullopt;

        const std::uint64_t valuesPerFrame = static_cast<std::uint64_t>(replay.mInputCount)
            + replay.mControllerStateCount + static_cast<std::uint64_t>(replay.mBodyCount) * 13;
        const std::uint64_t bytesPerFrame = sizeof(std::uint64_t) + valuesPerFrame * sizeof(std::uint32_t);
        if (frameCount != 0 && bytesPerFrame > reader.remaining() / frameCount)
            return std::nullopt;

        replay.mFrames.resize(frameCount);
        for (ReplayFrame& frame : replay.mFrames)
        {
            frame.mInputs.resize(replay.mInputCount);
            frame.mControllerState.resize(replay.mControllerStateCount);
            frame.mBodies.resize(replay.mBodyCount);
            if (!reader.readUnsigned(frame.mStepIndex))
                return std::nullopt;
            for (float& value : frame.mInputs)
            {
                if (!reader.readFloat(value))
                    return std::nullopt;
            }
            for (float& value : frame.mControllerState)
            {
                if (!reader.readFloat(value))
                    return std::nullopt;
            }
            for (ReplayBodyState& body : frame.mBodies)
            {
                if (!reader.readVector(body.mTransform.mTranslation)
                    || !reader.readQuaternion(body.mTransform.mRotation) || !reader.readVector(body.mLinearVelocity)
                    || !reader.readVector(body.mAngularVelocity))
                    return std::nullopt;
            }
        }

        if (reader.remaining() != 0 || !validateReplay(replay))
            return std::nullopt;
        return replay;
    }

    std::optional<PoweredReplayInput> decodePoweredReplayInput(const Replay& replay, const ReplayFrame& frame)
    {
        const std::uint64_t expectedInputCount = PoweredReplayGlobalInputChannels
            + static_cast<std::uint64_t>(replay.mBodyCount) * PoweredReplayPoseChannelsPerBody;
        const std::uint64_t legacyInputCount = LegacyPoweredReplayGlobalInputChannels
            + static_cast<std::uint64_t>(replay.mBodyCount) * PoweredReplayPoseChannelsPerBody;
        const std::uint64_t travelInputCount = TravelPoweredReplayGlobalInputChannels
            + static_cast<std::uint64_t>(replay.mBodyCount) * PoweredReplayPoseChannelsPerBody;
        const bool travelOnly = replay.mInputCount == travelInputCount;
        const bool legacy = replay.mInputCount == legacyInputCount;
        if (replay.mBodyCount == 0 || replay.mInputCount > MaximumReplayChannels
            || (!legacy && !travelOnly && replay.mInputCount != expectedInputCount) || frame.mInputs.size() != replay.mInputCount)
            return std::nullopt;

        PoweredReplayInput result;
        result.mRequestedHorizontalVelocity = JPH::Vec3(frame.mInputs[0], frame.mInputs[1], frame.mInputs[2]);
        if (!isFinite(result.mRequestedHorizontalVelocity))
            return std::nullopt;
        if (!legacy)
        {
            result.mRawRequestedHorizontalVelocity = JPH::Vec3(frame.mInputs[3], frame.mInputs[4], frame.mInputs[5]);
            result.mFacing = JPH::Vec3(frame.mInputs[6], frame.mInputs[7], frame.mInputs[8]);
            if (!isFinite(*result.mRawRequestedHorizontalVelocity) || !isFinite(*result.mFacing))
                return std::nullopt;
        }

        if (!legacy && !travelOnly)
        {
            const float finished = frame.mInputs[9];
            if (finished != 0.0f && finished != 1.0f) return std::nullopt;
            result.mRecoveryAnimationFinished = finished == 1.0f;
        }
        result.mAnimationTargetPose.reserve(replay.mBodyCount);
        std::size_t channel = legacy ? LegacyPoweredReplayGlobalInputChannels
            : (travelOnly ? TravelPoweredReplayGlobalInputChannels : PoweredReplayGlobalInputChannels);
        for (std::uint32_t body = 0; body < replay.mBodyCount; ++body)
        {
            Transform transform;
            transform.mTranslation
                = JPH::Vec3(frame.mInputs[channel], frame.mInputs[channel + 1], frame.mInputs[channel + 2]);
            transform.mRotation = JPH::Quat(frame.mInputs[channel + 3], frame.mInputs[channel + 4],
                frame.mInputs[channel + 5], frame.mInputs[channel + 6]);
            if (!isFinite(transform))
                return std::nullopt;
            result.mAnimationTargetPose.push_back(transform);
            channel += PoweredReplayPoseChannelsPerBody;
        }
        return result;
    }

    bool haveIdenticalReplayEncoding(const Replay& expected, const Replay& actual)
    {
        const auto expectedData = serializeReplay(expected);
        const auto actualData = serializeReplay(actual);
        return expectedData && actualData && *expectedData == *actualData;
    }

    ReplayComparisonResult compareReplays(
        const Replay& expected, const Replay& actual, const ReplayComparisonTolerances& tolerances)
    {
        ReplayComparisonResult result;
        if (!validateReplay(expected))
        {
            result.mStatus = ReplayComparisonStatus::InvalidExpected;
            return result;
        }
        if (!validateReplay(actual))
        {
            result.mStatus = ReplayComparisonStatus::InvalidActual;
            return result;
        }
        if (!isFiniteAndNonNegative(tolerances.mInput) || !isFiniteAndNonNegative(tolerances.mControllerState)
            || !isFiniteAndNonNegative(tolerances.mTranslation) || !isFiniteAndNonNegative(tolerances.mRotationAngle)
            || !isFiniteAndNonNegative(tolerances.mLinearVelocity)
            || !isFiniteAndNonNegative(tolerances.mAngularVelocity))
        {
            result.mStatus = ReplayComparisonStatus::InvalidTolerances;
            return result;
        }

        const ReplayMetadata& expectedMetadata = expected.mMetadata;
        const ReplayMetadata& actualMetadata = actual.mMetadata;
        if (expectedMetadata.mVersion != actualMetadata.mVersion || expectedMetadata.mSeed != actualMetadata.mSeed
            || expectedMetadata.mProfileHash != actualMetadata.mProfileHash
            || std::bit_cast<std::uint32_t>(expectedMetadata.mFixedTimeStep)
                != std::bit_cast<std::uint32_t>(actualMetadata.mFixedTimeStep)
            || expectedMetadata.mWorkerThreadCount != actualMetadata.mWorkerThreadCount
            || expectedMetadata.mCollisionSteps != actualMetadata.mCollisionSteps)
        {
            result.mStatus = ReplayComparisonStatus::MetadataMismatch;
            return result;
        }
        if (expected.mInputCount != actual.mInputCount || expected.mControllerStateCount != actual.mControllerStateCount
            || expected.mBodyCount != actual.mBodyCount)
        {
            result.mStatus = ReplayComparisonStatus::DimensionsMismatch;
            return result;
        }
        if (expected.mFrames.size() != actual.mFrames.size())
        {
            result.mStatus = ReplayComparisonStatus::FrameCountMismatch;
            return result;
        }

        if (tolerances.mExactPoweredDiscreteChannels
            && expected.mControllerStateCount != PoweredReplayControllerStateChannels
            && expected.mControllerStateCount != ContinuousSwingReplayControllerStateChannels)
        {
            result.mStatus = ReplayComparisonStatus::InvalidTolerances;
            return result;
        }
        const auto isDiscrete = [](std::size_t channel) {
            if (channel >= PoweredReplayControllerStateChannels)
                return channel < PoweredReplayControllerStateChannels + 2; // active and foot
            return channel != static_cast<std::size_t>(PoweredReplayControllerChannel::TimeInMode)
                && channel != static_cast<std::size_t>(PoweredReplayControllerChannel::TimeInGaitPhase)
                && channel != static_cast<std::size_t>(PoweredReplayControllerChannel::GaitPhaseProgress);
        };
        const auto recordDivergence = [&](std::uint64_t step, float error, float tolerance,
                                         ReplayDifferenceField field, std::size_t index) {
            if (error > tolerance && !result.mFirstDivergentStep)
            {
                result.mFirstDivergentStep = step;
                result.mFirstDivergentField = field;
                result.mFirstDivergentIndex = index;
            }
        };
        for (std::size_t frameIndex = 0; frameIndex < expected.mFrames.size(); ++frameIndex)
        {
            const ReplayFrame& expectedFrame = expected.mFrames[frameIndex];
            const ReplayFrame& actualFrame = actual.mFrames[frameIndex];
            // Discrete mismatches take priority within the same step, not over
            // an earlier physical or input divergence. Keep their own first
            // step too, so continuous drift cannot hide a later mode change.
            if (tolerances.mExactPoweredDiscreteChannels)
                for (std::size_t i = 0; i < expectedFrame.mControllerState.size(); ++i)
                {
                    if (!isDiscrete(i))
                        continue;
                    const float error = std::abs(expectedFrame.mControllerState[i] - actualFrame.mControllerState[i]);
                    if (error > 0.0f && !result.mFirstDiscreteDivergentStep)
                    {
                        result.mFirstDiscreteDivergentStep = expectedFrame.mStepIndex;
                        result.mFirstDiscreteDivergentChannel = i;
                    }
                    recordDivergence(expectedFrame.mStepIndex, error, 0.0f,
                        ReplayDifferenceField::ControllerState, i);
                }
            for (std::size_t i = 0; i < expectedFrame.mInputs.size(); ++i)
            {
                const float error = std::abs(expectedFrame.mInputs[i] - actualFrame.mInputs[i]);
                result.mMaximumInputError = std::max(result.mMaximumInputError, error);
                recordDivergence(expectedFrame.mStepIndex, error, tolerances.mInput,
                    ReplayDifferenceField::Input, i);
            }
            for (std::size_t i = 0; i < expectedFrame.mControllerState.size(); ++i)
            {
                const float error = std::abs(expectedFrame.mControllerState[i] - actualFrame.mControllerState[i]);
                result.mMaximumControllerStateError = std::max(result.mMaximumControllerStateError, error);
                recordDivergence(expectedFrame.mStepIndex, error, tolerances.mControllerState,
                    ReplayDifferenceField::ControllerState, i);
            }
            for (std::size_t i = 0; i < expectedFrame.mBodies.size(); ++i)
            {
                const ReplayBodyState& expectedBody = expectedFrame.mBodies[i];
                const ReplayBodyState& actualBody = actualFrame.mBodies[i];
                const float translationError
                    = (expectedBody.mTransform.mTranslation - actualBody.mTransform.mTranslation).Length();
                const float rotationError
                    = quaternionAngle(expectedBody.mTransform.mRotation, actualBody.mTransform.mRotation);
                const float linearVelocityError = (expectedBody.mLinearVelocity - actualBody.mLinearVelocity).Length();
                const float angularVelocityError
                    = (expectedBody.mAngularVelocity - actualBody.mAngularVelocity).Length();
                result.mMaximumTranslationError = std::max(result.mMaximumTranslationError, translationError);
                result.mMaximumRotationAngleError = std::max(result.mMaximumRotationAngleError, rotationError);
                result.mMaximumLinearVelocityError = std::max(result.mMaximumLinearVelocityError, linearVelocityError);
                result.mMaximumAngularVelocityError
                    = std::max(result.mMaximumAngularVelocityError, angularVelocityError);
                recordDivergence(expectedFrame.mStepIndex, translationError, tolerances.mTranslation,
                    ReplayDifferenceField::Translation, i);
                recordDivergence(expectedFrame.mStepIndex, rotationError, tolerances.mRotationAngle,
                    ReplayDifferenceField::Rotation, i);
                recordDivergence(expectedFrame.mStepIndex, linearVelocityError, tolerances.mLinearVelocity,
                    ReplayDifferenceField::LinearVelocity, i);
                recordDivergence(expectedFrame.mStepIndex, angularVelocityError, tolerances.mAngularVelocity,
                    ReplayDifferenceField::AngularVelocity, i);
            }
        }
        if (result.mFirstDivergentStep)
            result.mStatus = ReplayComparisonStatus::Diverged;
        return result;
    }
}
