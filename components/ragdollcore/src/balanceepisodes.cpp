#include <ragdollcore/balanceepisodes.hpp>

#include <bit>
#include <cmath>
#include <utility>

namespace RagdollCore
{
    namespace
    {
        std::optional<BalanceSupportSegment> closeSegment(
            BalanceFootEpisodeState& foot, BalanceSupportSegmentEnd end)
        {
            auto result = std::move(foot.mCurrentSegment);
            foot.mCurrentSegment.reset();
            if (result)
                result->mEnd = end;
            return result;
        }

        bool validSlip(const BalanceFootSlipMeasurement& sample, float duration)
        {
            if (!sample.mMaximumTangentSpeed || !sample.mMaximumTangentDistanceThisStep)
                return false;
            const float speed = *sample.mMaximumTangentSpeed;
            const float distance = *sample.mMaximumTangentDistanceThisStep;
            if (!std::isfinite(speed) || speed < 0 || !std::isfinite(distance) || distance < 0)
                return false;
            // Compare with the producer's actual float product. A world-unit
            // epsilon admits very large relative errors near zero and can turn
            // nonzero travel into a purported measured zero (or vice versa).
            const float expected = speed * duration;
            if (!std::isfinite(expected))
                return false;
            if (expected == 0.0f || distance == 0.0f)
                return expected == distance;
            // Finite positive IEEE float bit patterns are monotonically
            // ordered. Permit only two representable product values of roundoff;
            // neither zero nor an overflowed product gets an absolute tolerance.
            constexpr std::uint32_t maximumProductUlps = 2;
            const auto expectedBits = std::bit_cast<std::uint32_t>(expected);
            const auto distanceBits = std::bit_cast<std::uint32_t>(distance);
            return expectedBits >= distanceBits ? expectedBits - distanceBits <= maximumProductUlps
                                                : distanceBits - expectedBits <= maximumProductUlps;
        }
    }

    BalanceEpisodeResetResult BalanceSupportEpisodeAccumulator::reset(BalanceEpisodeGeneration generation)
    {
        BalanceEpisodeResetResult result;
        if (mGeneration && *mGeneration == generation)
            return result;
        for (std::size_t side = 0; side < mFeet.size(); ++side)
            result.mClosedSegments[side] = closeSegment(mFeet[side], BalanceSupportSegmentEnd::LifecycleInvalidated);
        mGeneration = generation;
        mLastPhysicsStep.reset();
        mFeet = {};
        mMissingPhysicsStepCount = 0;
        result.mChanged = true;
        return result;
    }

    BalanceEpisodeConsumeResult BalanceSupportEpisodeAccumulator::consume(BalanceEpisodeGeneration generation,
        std::uint64_t physicsStep, float duration, const std::array<BalanceFootSlipMeasurement, 2>& feet)
    {
        BalanceEpisodeConsumeResult result;
        if (!mGeneration)
            return result;
        if (*mGeneration != generation)
        {
            result.mStatus = BalanceEpisodeConsumeStatus::WrongGeneration;
            return result;
        }
        if (mLastPhysicsStep && physicsStep <= *mLastPhysicsStep)
        {
            result.mStatus = physicsStep == *mLastPhysicsStep ? BalanceEpisodeConsumeStatus::DuplicatePhysicsStep
                                                            : BalanceEpisodeConsumeStatus::OlderPhysicsStep;
            return result;
        }
        if (!std::isfinite(duration) || duration <= 0)
        {
            result.mStatus = BalanceEpisodeConsumeStatus::InvalidDuration;
            return result;
        }
        if (mLastPhysicsStep)
            result.mMissingPhysicsSteps = physicsStep - *mLastPhysicsStep - 1;
        mMissingPhysicsStepCount += result.mMissingPhysicsSteps;
        for (std::size_t side = 0; side < mFeet.size(); ++side)
        {
            auto& foot = mFeet[side];
            const auto& sample = feet[side];
            if (result.mMissingPhysicsSteps > 0)
            {
                result.mClosedSegments[side] = closeSegment(foot, BalanceSupportSegmentEnd::MissingPhysicsSteps);
                foot.mAvailability = BalanceSupportAvailability::Unknown;
            }
            if (sample.mAvailability == BalanceSupportAvailability::Supported)
            {
                if (!foot.mCurrentSegment)
                {
                    BalanceSupportSegment segment;
                    segment.mGeneration = generation;
                    segment.mFirstPhysicsStep = physicsStep;
                    segment.mOnsetObserved = mLastPhysicsStep.has_value()
                        && foot.mAvailability == BalanceSupportAvailability::Unsupported;
                    foot.mCurrentSegment = segment;
                }
                auto& segment = *foot.mCurrentSegment;
                segment.mLastPhysicsStep = physicsStep;
                ++segment.mSupportedSteps;
                segment.mObservedSupportDuration += duration;
                if (validSlip(sample, duration))
                {
                    ++segment.mMeasuredSlipSteps;
                    segment.mObservedSlipDuration += duration;
                    segment.mObservedMaximumTangentTravel = segment.mObservedMaximumTangentTravel.value_or(0.0)
                        + *sample.mMaximumTangentDistanceThisStep;
                }
                else
                    segment.mSlipComplete = false;
            }
            else
            {
                const auto end = sample.mAvailability == BalanceSupportAvailability::Unsupported
                    ? BalanceSupportSegmentEnd::Unsupported : BalanceSupportSegmentEnd::UnknownSupport;
                if (auto closed = closeSegment(foot, end))
                    result.mClosedSegments[side] = std::move(closed);
                if (sample.mAvailability != BalanceSupportAvailability::Unsupported)
                {
                    ++foot.mObservedUnknownSteps;
                    foot.mObservedUnknownDuration += duration;
                }
            }
            // An invalid enum value is conservatively unknown, never support.
            foot.mAvailability = sample.mAvailability == BalanceSupportAvailability::Supported
                    || sample.mAvailability == BalanceSupportAvailability::Unsupported
                ? sample.mAvailability : BalanceSupportAvailability::Unknown;
        }
        mLastPhysicsStep = physicsStep;
        result.mStatus = BalanceEpisodeConsumeStatus::Consumed;
        return result;
    }
}
