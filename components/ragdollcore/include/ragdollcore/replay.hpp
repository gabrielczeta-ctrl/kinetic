#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_REPLAY_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_REPLAY_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <ragdollcore/profile.hpp>
#include <ragdollcore/swingtrajectory.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace RagdollCore
{
    inline constexpr std::uint32_t CurrentReplayVersion = 10;
    inline constexpr std::uint32_t MaximumReplayChannels = 4096;
    inline constexpr std::uint32_t MaximumReplayBodies = 1024;
    inline constexpr std::uint32_t MaximumReplayFrames = 1'000'000;
    inline constexpr std::uint32_t LegacyPoweredReplayGlobalInputChannels = 3;
    // Filtered travel, raw travel, then filtered gameplay facing. The binary
    // container carries the channel count; older three/nine-channel files remain readable.
    inline constexpr std::uint32_t TravelPoweredReplayGlobalInputChannels = 9;
    // Animation completion is an external controller input, not a physics result.
    inline constexpr std::uint32_t PoweredReplayGlobalInputChannels = 10;
    inline constexpr std::uint32_t PoweredReplayPoseChannelsPerBody = 7;
    inline constexpr std::uint32_t PoweredReplayControllerStateChannels = 10;
    // Continuous-swing layout v1, appended after the ten legacy channels.
    inline constexpr std::uint32_t ContinuousSwingReplayControllerStateChannels
        = PoweredReplayControllerStateChannels + SwingTrajectoryHistoryChannels;

    enum class PoweredReplayControllerChannel : std::uint32_t
    {
        Mode,
        TimeInMode,
        GaitPhase,
        TimeInGaitPhase,
        GaitPhaseProgress,
        SwingFootLifted,
        LeftFootSupported,
        RightFootSupported,
        CenterOfMassSupported,
        GaitPhaseTimedOut,
    };
    static_assert(static_cast<std::uint32_t>(PoweredReplayControllerChannel::GaitPhaseTimedOut) + 1
        == PoweredReplayControllerStateChannels);

    struct ReplayMetadata
    {
        std::uint32_t mVersion = CurrentReplayVersion;
        std::uint64_t mSeed = 0;
        std::uint64_t mProfileHash = 0;
        float mFixedTimeStep = 1.0f / 60.0f;
        std::uint32_t mWorkerThreadCount = 1;
        std::uint32_t mCollisionSteps = 1;
    };

    struct ReplayBodyState
    {
        Transform mTransform;
        JPH::Vec3 mLinearVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mAngularVelocity = JPH::Vec3::sZero();
    };

    struct ReplayFrame
    {
        std::uint64_t mStepIndex = 0;
        std::vector<float> mInputs;
        std::vector<float> mControllerState;
        std::vector<ReplayBodyState> mBodies;
    };

    struct Replay
    {
        ReplayMetadata mMetadata;
        std::uint32_t mInputCount = 0;
        std::uint32_t mControllerStateCount = 0;
        std::uint32_t mBodyCount = 0;
        std::vector<ReplayFrame> mFrames;
    };

    struct PoweredReplayInput
    {
        JPH::Vec3 mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        std::optional<JPH::Vec3> mRawRequestedHorizontalVelocity;
        std::optional<JPH::Vec3> mFacing;
        std::optional<bool> mRecoveryAnimationFinished;
        std::vector<Transform> mAnimationTargetPose;
    };

    struct ReplayComparisonTolerances
    {
        float mInput = 0.0f;
        float mControllerState = 0.0f;
        float mTranslation = 0.0f;
        float mRotationAngle = 0.0f;
        float mLinearVelocity = 0.0f;
        float mAngularVelocity = 0.0f;
        // Opt in only for the documented powered controller layout. Modes,
        // phases and flags remain exact regardless of mControllerState;
        // that tolerance then applies only to timers and phase progress.
        bool mExactPoweredDiscreteChannels = false;
    };

    enum class ReplayDifferenceField
    {
        Input,
        ControllerState,
        Translation,
        Rotation,
        LinearVelocity,
        AngularVelocity,
    };

    enum class ReplayComparisonStatus
    {
        Match,
        InvalidExpected,
        InvalidActual,
        InvalidTolerances,
        MetadataMismatch,
        DimensionsMismatch,
        FrameCountMismatch,
        Diverged,
    };

    struct ReplayComparisonResult
    {
        ReplayComparisonStatus mStatus = ReplayComparisonStatus::Match;
        std::optional<std::uint64_t> mFirstDivergentStep;
        std::optional<ReplayDifferenceField> mFirstDivergentField;
        // Input/controller channel, or body index for physical differences.
        std::optional<std::size_t> mFirstDivergentIndex;
        std::optional<std::uint64_t> mFirstDiscreteDivergentStep;
        std::optional<std::size_t> mFirstDiscreteDivergentChannel;
        float mMaximumInputError = 0.0f;
        float mMaximumControllerStateError = 0.0f;
        float mMaximumTranslationError = 0.0f;
        float mMaximumRotationAngleError = 0.0f;
        float mMaximumLinearVelocityError = 0.0f;
        float mMaximumAngularVelocityError = 0.0f;

        bool isMatch() const { return mStatus == ReplayComparisonStatus::Match; }
    };

    // Stable FNV-1a hash over every profile field in profile/body order.
    // It deliberately does not depend on object layout or host endianness.
    std::uint64_t hashProfile(const RagdollProfile& profile);

    // The portable binary format uses explicit little-endian integers and
    // IEEE-754 float bit patterns. Invalid values, dimensions, or trailing data
    // are rejected transactionally.
    bool validateReplay(const Replay& replay);
    std::optional<std::vector<std::byte>> serializeReplay(const Replay& replay);
    std::optional<Replay> deserializeReplay(std::span<const std::byte> data);

    // Decodes the powered-controller inputs shared by live capture and the
    // standalone scenario harness. It rejects any mismatched channel layout.
    std::optional<PoweredReplayInput> decodePoweredReplayInput(const Replay& replay, const ReplayFrame& frame);

    // Exact encoding equality is the controlled bitwise-determinism gate.
    // compareReplays additionally quantifies bounded semantic divergence while
    // requiring identical simulation metadata and channel dimensions.
    bool haveIdenticalReplayEncoding(const Replay& expected, const Replay& actual);
    ReplayComparisonResult compareReplays(
        const Replay& expected, const Replay& actual, const ReplayComparisonTolerances& tolerances = {});
}

#endif
