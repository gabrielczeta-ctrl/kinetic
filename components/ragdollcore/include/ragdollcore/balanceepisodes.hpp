#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_BALANCEEPISODES_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_BALANCEEPISODES_H

#include <ragdollcore/balancemetrics.hpp>

#include <array>
#include <cstdint>
#include <optional>

namespace RagdollCore
{
    struct BalanceEpisodeGeneration
    {
        std::uint64_t mActorSerial = 0;
        std::uint64_t mLifecycleEpoch = 0;

        bool operator==(const BalanceEpisodeGeneration&) const = default;
    };

    enum class BalanceSupportSegmentEnd
    {
        Open,
        Unsupported,
        UnknownSupport,
        MissingPhysicsSteps,
        LifecycleInvalidated,
    };

    // A contiguous observed support segment, not an inferred physical stance.
    // Only adjacent Unsupported -> Supported -> Unsupported observations can
    // bracket a complete support episode at the supplied physics resolution.
    struct BalanceSupportSegment
    {
        BalanceEpisodeGeneration mGeneration;
        std::uint64_t mFirstPhysicsStep = 0;
        std::uint64_t mLastPhysicsStep = 0;
        bool mOnsetObserved = false;
        BalanceSupportSegmentEnd mEnd = BalanceSupportSegmentEnd::Open;
        std::uint64_t mSupportedSteps = 0;
        double mObservedSupportDuration = 0;
        std::uint64_t mMeasuredSlipSteps = 0;
        double mObservedSlipDuration = 0;
        // Sum of observed per-step maximum tangent distances, once per step.
        // Different points can attain each maximum. This is conservative
        // tangent travel, NOT net displacement or the BAL stride-axis gate.
        // Absent means no valid slip sample; measured zero remains a value.
        std::optional<double> mObservedMaximumTangentTravel;
        bool mSlipComplete = true;

        bool hasCompleteSupportEpisode() const
        {
            return mOnsetObserved && mEnd == BalanceSupportSegmentEnd::Unsupported;
        }

        bool hasCompleteEpisodeTravel() const
        {
            return hasCompleteSupportEpisode() && mSlipComplete;
        }
    };

    struct BalanceFootEpisodeState
    {
        BalanceSupportAvailability mAvailability = BalanceSupportAvailability::Unknown;
        std::optional<BalanceSupportSegment> mCurrentSegment;
        // Supplied Unknown observations since reset. Missing step IDs have no
        // known duration and are counted separately, never extrapolated here.
        std::uint64_t mObservedUnknownSteps = 0;
        double mObservedUnknownDuration = 0;
    };

    using BalanceClosedSupportSegments = std::array<std::optional<BalanceSupportSegment>, 2>;

    enum class BalanceEpisodeConsumeStatus
    {
        Consumed,
        UnboundGeneration,
        WrongGeneration,
        DuplicatePhysicsStep,
        OlderPhysicsStep,
        InvalidDuration,
    };

    struct BalanceEpisodeConsumeResult
    {
        BalanceEpisodeConsumeStatus mStatus = BalanceEpisodeConsumeStatus::UnboundGeneration;
        std::uint64_t mMissingPhysicsSteps = 0;
        BalanceClosedSupportSegments mClosedSegments;
    };

    struct BalanceEpisodeResetResult
    {
        bool mChanged = false;
        BalanceClosedSupportSegments mClosedSegments;
    };

    // Measurement-only, no body/solver/controller access. The caller explicitly
    // binds each new actor/lifecycle identity with reset; samples never rebind
    // themselves. Resetting the same identity is a no-op so it cannot replay a
    // consumed step. Every real invalidation needs a new lifecycle identity.
    // Completed segments are returned to the caller; storage is bounded.
    class BalanceSupportEpisodeAccumulator
    {
    public:
        BalanceEpisodeResetResult reset(BalanceEpisodeGeneration generation);

        // Identity/order/duration rejection leaves all state unchanged. Gaps
        // close existing segments with an unknown boundary; they do not prove
        // physical loss of support. Unknown support also breaks continuity.
        // Supported with absent/invalid/inconsistent slip retains support but
        // marks travel incomplete. Both feet are processed independently.
        BalanceEpisodeConsumeResult consume(BalanceEpisodeGeneration generation, std::uint64_t physicsStep,
            float duration, const std::array<BalanceFootSlipMeasurement, 2>& feet);

        const std::optional<BalanceEpisodeGeneration>& getGeneration() const { return mGeneration; }
        const std::optional<std::uint64_t>& getLastPhysicsStep() const { return mLastPhysicsStep; }
        const std::array<BalanceFootEpisodeState, 2>& getFeet() const { return mFeet; }
        std::uint64_t getMissingPhysicsStepCount() const { return mMissingPhysicsStepCount; }

    private:
        std::optional<BalanceEpisodeGeneration> mGeneration;
        std::optional<std::uint64_t> mLastPhysicsStep;
        std::array<BalanceFootEpisodeState, 2> mFeet;
        std::uint64_t mMissingPhysicsStepCount = 0;
    };
}

#endif
