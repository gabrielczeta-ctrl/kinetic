#include <ragdollcore/balanceepisodes.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{
    using namespace RagdollCore;
    using Availability = BalanceSupportAvailability;
    using Status = BalanceEpisodeConsumeStatus;
    using End = BalanceSupportSegmentEnd;

    constexpr BalanceEpisodeGeneration generation{ 7, 2 };
    constexpr float stepDuration = 0.125f;

    BalanceFootSlipMeasurement supported(float speed = 2.0f, float duration = stepDuration)
    {
        return { Availability::Supported, speed, speed * duration };
    }

    BalanceFootSlipMeasurement unsupported()
    {
        return { Availability::Unsupported, std::nullopt, std::nullopt };
    }

    BalanceFootSlipMeasurement unknown()
    {
        return {};
    }

    TEST(RagdollCoreBalanceEpisodeTest, completeEpisodeUsesOneDistanceAndDurationPerPhysicsStep)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        EXPECT_TRUE(accumulator.reset(generation).mChanged);
        accumulator.consume(generation, 10, stepDuration, { unsupported(), unsupported() });
        accumulator.consume(generation, 11, stepDuration, { supported(2), unsupported() });
        accumulator.consume(generation, 12, 0.25f, { supported(4, 0.25f), unsupported() });
        const auto ended = accumulator.consume(generation, 13, stepDuration, { unsupported(), unsupported() });
        ASSERT_TRUE(ended.mClosedSegments[0]);
        const auto& segment = *ended.mClosedSegments[0];
        EXPECT_EQ(segment.mGeneration, generation);
        EXPECT_EQ(segment.mFirstPhysicsStep, 11u);
        EXPECT_EQ(segment.mLastPhysicsStep, 12u);
        EXPECT_EQ(segment.mSupportedSteps, 2u);
        EXPECT_EQ(segment.mMeasuredSlipSteps, 2u);
        EXPECT_DOUBLE_EQ(segment.mObservedSupportDuration, 0.375);
        EXPECT_DOUBLE_EQ(segment.mObservedSlipDuration, 0.375);
        ASSERT_TRUE(segment.mObservedMaximumTangentTravel);
        EXPECT_DOUBLE_EQ(*segment.mObservedMaximumTangentTravel, 1.25);
        EXPECT_EQ(segment.mEnd, End::Unsupported);
        EXPECT_TRUE(segment.hasCompleteSupportEpisode());
        EXPECT_TRUE(segment.hasCompleteEpisodeTravel());
        EXPECT_FALSE(accumulator.getFeet()[0].mCurrentSegment);
        EXPECT_FALSE(ended.mClosedSegments[1]);
    }

    TEST(RagdollCoreBalanceEpisodeTest, duplicateOlderAndMismatchedSamplesCannotAccumulateOrEndSupport)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        accumulator.reset(generation);
        accumulator.consume(generation, 30, stepDuration, { supported(), supported() });
        EXPECT_EQ(accumulator.consume(generation, 30, stepDuration, { unsupported(), unsupported() }).mStatus,
            Status::DuplicatePhysicsStep);
        EXPECT_EQ(accumulator.consume(generation, 29, stepDuration, { unknown(), unknown() }).mStatus,
            Status::OlderPhysicsStep);
        EXPECT_EQ(accumulator.consume({ 7, 1 }, 31, stepDuration, { unsupported(), unsupported() }).mStatus,
            Status::WrongGeneration);
        EXPECT_EQ(accumulator.consume({ 8, 2 }, 31, stepDuration, { unsupported(), unsupported() }).mStatus,
            Status::WrongGeneration);
        for (const auto& foot : accumulator.getFeet())
        {
            ASSERT_TRUE(foot.mCurrentSegment);
            EXPECT_EQ(foot.mCurrentSegment->mSupportedSteps, 1u);
            EXPECT_DOUBLE_EQ(*foot.mCurrentSegment->mObservedMaximumTangentTravel, 0.25);
            EXPECT_EQ(foot.mObservedUnknownSteps, 0u);
        }
        EXPECT_EQ(accumulator.getLastPhysicsStep(), 30u);
    }

    TEST(RagdollCoreBalanceEpisodeTest, readOnlyAccessAndSameGenerationResetCannotConsumeAgain)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        accumulator.reset(generation);
        accumulator.consume(generation, 4, stepDuration, { supported(), unknown() });
        for (int read = 0; read < 128; ++read)
        {
            ASSERT_TRUE(accumulator.getFeet()[0].mCurrentSegment);
            EXPECT_EQ(accumulator.getFeet()[0].mCurrentSegment->mSupportedSteps, 1u);
            EXPECT_EQ(accumulator.getLastPhysicsStep(), 4u);
        }
        const auto reset = accumulator.reset(generation);
        EXPECT_FALSE(reset.mChanged);
        EXPECT_FALSE(reset.mClosedSegments[0] || reset.mClosedSegments[1]);
        EXPECT_EQ(accumulator.consume(generation, 4, stepDuration, { supported(), unknown() }).mStatus,
            Status::DuplicatePhysicsStep);
        EXPECT_EQ(accumulator.getFeet()[1].mObservedUnknownSteps, 1u);
    }

    TEST(RagdollCoreBalanceEpisodeTest, startingSupportedDoesNotInventObservedOnset)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        accumulator.reset(generation);
        accumulator.consume(generation, 100, stepDuration, { supported(), unknown() });
        const auto ended = accumulator.consume(generation, 101, stepDuration, { unsupported(), unknown() });
        ASSERT_TRUE(ended.mClosedSegments[0]);
        EXPECT_FALSE(ended.mClosedSegments[0]->mOnsetObserved);
        EXPECT_FALSE(ended.mClosedSegments[0]->hasCompleteSupportEpisode());
        EXPECT_FALSE(ended.mClosedSegments[0]->hasCompleteEpisodeTravel());
        EXPECT_TRUE(ended.mClosedSegments[0]->mSlipComplete);
    }

    TEST(RagdollCoreBalanceEpisodeTest, unknownGapClosesPartialSegmentsWithoutClaimingPhysicalLiftoff)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        accumulator.reset(generation);
        accumulator.consume(generation, 0, stepDuration, { unsupported(), unsupported() });
        accumulator.consume(generation, 1, stepDuration, { supported(), unsupported() });
        const auto gap = accumulator.consume(generation, 2, stepDuration, { unknown(), unsupported() });
        ASSERT_TRUE(gap.mClosedSegments[0]);
        EXPECT_TRUE(gap.mClosedSegments[0]->mOnsetObserved);
        EXPECT_EQ(gap.mClosedSegments[0]->mEnd, End::UnknownSupport);
        EXPECT_FALSE(gap.mClosedSegments[0]->hasCompleteSupportEpisode());
        accumulator.consume(generation, 3, 0.25f, { unknown(), unsupported() });
        accumulator.consume(generation, 4, stepDuration, { supported(4), unsupported() });
        const auto ended = accumulator.consume(generation, 5, stepDuration, { unsupported(), unsupported() });
        ASSERT_TRUE(ended.mClosedSegments[0]);
        EXPECT_FALSE(ended.mClosedSegments[0]->mOnsetObserved);
        EXPECT_EQ(ended.mClosedSegments[0]->mSupportedSteps, 1u);
        EXPECT_DOUBLE_EQ(*ended.mClosedSegments[0]->mObservedMaximumTangentTravel, 0.5);
        EXPECT_FALSE(ended.mClosedSegments[0]->hasCompleteEpisodeTravel());
        EXPECT_EQ(accumulator.getFeet()[0].mObservedUnknownSteps, 2u);
        EXPECT_DOUBLE_EQ(accumulator.getFeet()[0].mObservedUnknownDuration, 0.375);
    }

    TEST(RagdollCoreBalanceEpisodeTest, skippedStepsBreakBothContinuitiesWithoutExtrapolatingTimeOrTravel)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        accumulator.reset(generation);
        accumulator.consume(generation, 8, stepDuration, { unsupported(), unsupported() });
        accumulator.consume(generation, 9, stepDuration, { supported(), supported(4) });
        const auto gap = accumulator.consume(generation, 13, stepDuration, { supported(), unsupported() });
        EXPECT_EQ(gap.mMissingPhysicsSteps, 3u);
        EXPECT_EQ(accumulator.getMissingPhysicsStepCount(), 3u);
        for (const auto& closed : gap.mClosedSegments)
        {
            ASSERT_TRUE(closed);
            EXPECT_EQ(closed->mEnd, End::MissingPhysicsSteps);
            EXPECT_EQ(closed->mSupportedSteps, 1u);
            EXPECT_DOUBLE_EQ(closed->mObservedSupportDuration, stepDuration);
            EXPECT_FALSE(closed->hasCompleteSupportEpisode());
        }
        ASSERT_TRUE(accumulator.getFeet()[0].mCurrentSegment);
        EXPECT_FALSE(accumulator.getFeet()[0].mCurrentSegment->mOnsetObserved);
        EXPECT_EQ(accumulator.getFeet()[0].mCurrentSegment->mFirstPhysicsStep, 13u);
        EXPECT_DOUBLE_EQ(*accumulator.getFeet()[0].mCurrentSegment->mObservedMaximumTangentTravel, 0.25);
        EXPECT_EQ(accumulator.getFeet()[0].mObservedUnknownSteps, 0u);
        EXPECT_DOUBLE_EQ(accumulator.getFeet()[0].mObservedUnknownDuration, 0);
        EXPECT_EQ(accumulator.getFeet()[1].mAvailability, Availability::Unsupported);
    }

    TEST(RagdollCoreBalanceEpisodeTest, unsupportedBeforeMissingStepCannotEstablishLaterOnset)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        accumulator.reset(generation);
        accumulator.consume(generation, 1, stepDuration, { unsupported(), unsupported() });
        accumulator.consume(generation, 3, stepDuration, { supported(), supported() });
        EXPECT_FALSE(accumulator.getFeet()[0].mCurrentSegment->mOnsetObserved);
        EXPECT_FALSE(accumulator.getFeet()[1].mCurrentSegment->mOnsetObserved);
    }

    TEST(RagdollCoreBalanceEpisodeTest, missingSlipRetainsSupportAndSeparatesObservedTravelFromCompleteTravel)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        accumulator.reset(generation);
        accumulator.consume(generation, 1, stepDuration, { unsupported(), unsupported() });
        accumulator.consume(generation, 2, stepDuration, { supported(), supported(0) });
        const BalanceFootSlipMeasurement supportedUnknown{ Availability::Supported, std::nullopt, std::nullopt };
        accumulator.consume(generation, 3, 0.25f, { supportedUnknown, supported(0, 0.25f) });
        accumulator.consume(generation, 4, stepDuration, { supported(4), supported(0) });
        const auto ended = accumulator.consume(generation, 5, stepDuration, { unsupported(), unsupported() });
        ASSERT_TRUE(ended.mClosedSegments[0] && ended.mClosedSegments[1]);
        const auto& left = *ended.mClosedSegments[0];
        EXPECT_TRUE(left.hasCompleteSupportEpisode());
        EXPECT_FALSE(left.hasCompleteEpisodeTravel());
        EXPECT_EQ(left.mSupportedSteps, 3u);
        EXPECT_EQ(left.mMeasuredSlipSteps, 2u);
        EXPECT_DOUBLE_EQ(left.mObservedSupportDuration, 0.5);
        EXPECT_DOUBLE_EQ(left.mObservedSlipDuration, 0.25);
        EXPECT_DOUBLE_EQ(*left.mObservedMaximumTangentTravel, 0.75);
        EXPECT_TRUE(ended.mClosedSegments[1]->hasCompleteEpisodeTravel());
        EXPECT_DOUBLE_EQ(*ended.mClosedSegments[1]->mObservedMaximumTangentTravel, 0.0);
    }

    TEST(RagdollCoreBalanceEpisodeTest, feetHaveIndependentEpisodesAndUnknownHistories)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        accumulator.reset(generation);
        accumulator.consume(generation, 1, stepDuration, { unsupported(), unknown() });
        accumulator.consume(generation, 2, stepDuration, { supported(), supported() });
        const auto third = accumulator.consume(generation, 3, stepDuration, { unknown(), supported() });
        ASSERT_TRUE(third.mClosedSegments[0]);
        EXPECT_FALSE(third.mClosedSegments[1]);
        const auto fourth = accumulator.consume(generation, 4, stepDuration, { unsupported(), unsupported() });
        EXPECT_FALSE(fourth.mClosedSegments[0]);
        ASSERT_TRUE(fourth.mClosedSegments[1]);
        EXPECT_EQ(fourth.mClosedSegments[1]->mSupportedSteps, 2u);
        EXPECT_FALSE(fourth.mClosedSegments[1]->mOnsetObserved);
        EXPECT_EQ(accumulator.getFeet()[0].mObservedUnknownSteps, 1u);
        EXPECT_EQ(accumulator.getFeet()[1].mObservedUnknownSteps, 1u);
        accumulator.consume(generation, 5, stepDuration, { supported(), unsupported() });
        EXPECT_TRUE(accumulator.getFeet()[0].mCurrentSegment->mOnsetObserved);
    }

    TEST(RagdollCoreBalanceEpisodeTest, invalidationReturnsPartialAndClearsHistoryBeforeNewIdentity)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        accumulator.reset(generation);
        accumulator.consume(generation, 7, stepDuration, { unsupported(), unknown() });
        accumulator.consume(generation, 10, stepDuration, { supported(), unknown() });
        const BalanceEpisodeGeneration next{ generation.mActorSerial, generation.mLifecycleEpoch + 1 };
        const auto reset = accumulator.reset(next);
        EXPECT_TRUE(reset.mChanged);
        ASSERT_TRUE(reset.mClosedSegments[0]);
        EXPECT_EQ(reset.mClosedSegments[0]->mGeneration, generation);
        EXPECT_EQ(reset.mClosedSegments[0]->mEnd, End::LifecycleInvalidated);
        EXPECT_FALSE(reset.mClosedSegments[0]->hasCompleteSupportEpisode());
        EXPECT_FALSE(accumulator.getLastPhysicsStep());
        EXPECT_EQ(accumulator.getMissingPhysicsStepCount(), 0u);
        for (const auto& foot : accumulator.getFeet())
        {
            EXPECT_EQ(foot.mAvailability, Availability::Unknown);
            EXPECT_FALSE(foot.mCurrentSegment);
            EXPECT_EQ(foot.mObservedUnknownSteps, 0u);
            EXPECT_DOUBLE_EQ(foot.mObservedUnknownDuration, 0);
        }
        EXPECT_EQ(accumulator.consume(generation, 11, stepDuration, { supported(), supported() }).mStatus,
            Status::WrongGeneration);
        EXPECT_EQ(accumulator.consume(next, 0, stepDuration, { supported(), supported() }).mStatus, Status::Consumed);
        EXPECT_FALSE(accumulator.getFeet()[0].mCurrentSegment->mOnsetObserved);
        const auto recreated = accumulator.reset({ 8, 0 });
        EXPECT_TRUE(recreated.mChanged);
        EXPECT_EQ(accumulator.consume(next, 1, stepDuration, { supported(), supported() }).mStatus,
            Status::WrongGeneration);
    }

    TEST(RagdollCoreBalanceEpisodeTest, unboundOrInvalidDurationNeverConsumesStepOrChangesContinuity)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        EXPECT_EQ(accumulator.consume(generation, 1, stepDuration, { supported(), supported() }).mStatus,
            Status::UnboundGeneration);
        accumulator.reset(generation);
        accumulator.consume(generation, 1, stepDuration, { unsupported(), unsupported() });
        for (const float duration : { 0.0f, -1.0f, std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::quiet_NaN() })
        {
            EXPECT_EQ(accumulator.consume(generation, 2, duration, { supported(), supported() }).mStatus,
                Status::InvalidDuration);
            EXPECT_EQ(accumulator.getLastPhysicsStep(), 1u);
            EXPECT_FALSE(accumulator.getFeet()[0].mCurrentSegment);
        }
        EXPECT_EQ(accumulator.consume(generation, 2, stepDuration, { supported(), supported() }).mStatus,
            Status::Consumed);
        EXPECT_TRUE(accumulator.getFeet()[0].mCurrentSegment->mOnsetObserved);
    }

    TEST(RagdollCoreBalanceEpisodeTest, invalidOrInconsistentSlipCannotFabricateMeasuredZeroOrTravel)
    {
        const std::array invalid{
            BalanceFootSlipMeasurement{ Availability::Supported, std::nullopt, 1.0f },
            BalanceFootSlipMeasurement{ Availability::Supported, 1.0f, std::nullopt },
            BalanceFootSlipMeasurement{ Availability::Supported, -1.0f, 0.0f },
            BalanceFootSlipMeasurement{ Availability::Supported, 1.0f, -1.0f },
            BalanceFootSlipMeasurement{ Availability::Supported, std::numeric_limits<float>::infinity(), 1.0f },
            BalanceFootSlipMeasurement{ Availability::Supported, 1.0f, std::numeric_limits<float>::quiet_NaN() },
            BalanceFootSlipMeasurement{ Availability::Supported, 2.0f, 5.0f },
        };
        for (const auto& sample : invalid)
        {
            BalanceSupportEpisodeAccumulator accumulator;
            accumulator.reset(generation);
            accumulator.consume(generation, 1, stepDuration, { unsupported(), unsupported() });
            accumulator.consume(generation, 2, stepDuration, { sample, supported() });
            const auto ended = accumulator.consume(generation, 3, stepDuration, { unsupported(), unsupported() });
            ASSERT_TRUE(ended.mClosedSegments[0]);
            EXPECT_TRUE(ended.mClosedSegments[0]->hasCompleteSupportEpisode());
            EXPECT_FALSE(ended.mClosedSegments[0]->hasCompleteEpisodeTravel());
            EXPECT_FALSE(ended.mClosedSegments[0]->mObservedMaximumTangentTravel);
            EXPECT_EQ(ended.mClosedSegments[0]->mMeasuredSlipSteps, 0u);
            EXPECT_DOUBLE_EQ(ended.mClosedSegments[0]->mObservedSlipDuration, 0);
            EXPECT_TRUE(ended.mClosedSegments[1]->hasCompleteEpisodeTravel());
        }
    }

    TEST(RagdollCoreBalanceEpisodeTest, unknownAndUnsupportedIgnoreExtraneousSlipNumbers)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        accumulator.reset(generation);
        accumulator.consume(generation, 0, stepDuration,
            { BalanceFootSlipMeasurement{ Availability::Unknown, 10.0f, 1.25f },
                BalanceFootSlipMeasurement{ Availability::Unsupported, 10.0f, 1.25f } });
        EXPECT_FALSE(accumulator.getFeet()[0].mCurrentSegment);
        EXPECT_FALSE(accumulator.getFeet()[1].mCurrentSegment);
        EXPECT_EQ(accumulator.getFeet()[0].mObservedUnknownSteps, 1u);
        EXPECT_EQ(accumulator.getFeet()[1].mObservedUnknownSteps, 0u);
        accumulator.consume(generation, 1, stepDuration, { supported(), supported() });
        EXPECT_FALSE(accumulator.getFeet()[0].mCurrentSegment->mOnsetObserved);
        EXPECT_TRUE(accumulator.getFeet()[1].mCurrentSegment->mOnsetObserved);
    }

    TEST(RagdollCoreBalanceEpisodeTest, maximumStepDoesNotWrapAndFloatProductRoundingIsAccepted)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        accumulator.reset(generation);
        constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
        constexpr float duration = 1.0f / 60.0f;
        accumulator.consume(generation, maximum - 1, duration, { unsupported(), unsupported() });
        const auto sample = supported(1.234567f, duration);
        const auto accepted = accumulator.consume(generation, maximum, duration, { sample, sample });
        EXPECT_EQ(accepted.mMissingPhysicsSteps, 0u);
        EXPECT_TRUE(accumulator.getFeet()[0].mCurrentSegment->mSlipComplete);
        EXPECT_TRUE(accumulator.getFeet()[0].mCurrentSegment->mObservedMaximumTangentTravel);
        EXPECT_EQ(accumulator.consume(generation, 0, duration, { sample, sample }).mStatus, Status::OlderPhysicsStep);
        EXPECT_EQ(accumulator.getLastPhysicsStep(), maximum);
    }

    TEST(RagdollCoreBalanceEpisodeTest, nearZeroInconsistentSlipCannotClaimMeasuredTravelOrZero)
    {
        const std::array invalid{
            BalanceFootSlipMeasurement{ Availability::Supported, 0.0f, 9.0e-7f },
            BalanceFootSlipMeasurement{ Availability::Supported, 0.0f, std::numeric_limits<float>::min() },
            BalanceFootSlipMeasurement{ Availability::Supported, 1.0e-7f, 0.0f },
            BalanceFootSlipMeasurement{ Availability::Supported, 1.0e-5f, 4.0e-7f },
        };
        for (const auto& sample : invalid)
        {
            BalanceSupportEpisodeAccumulator accumulator;
            accumulator.reset(generation);
            accumulator.consume(generation, 1, stepDuration, { unsupported(), unsupported() });
            accumulator.consume(generation, 2, stepDuration, { sample, supported(0) });
            const auto ended = accumulator.consume(generation, 3, stepDuration, { unsupported(), unsupported() });
            ASSERT_TRUE(ended.mClosedSegments[0] && ended.mClosedSegments[1]);
            const auto& left = *ended.mClosedSegments[0];
            EXPECT_TRUE(left.hasCompleteSupportEpisode());
            EXPECT_FALSE(left.hasCompleteEpisodeTravel());
            EXPECT_FALSE(left.mObservedMaximumTangentTravel);
            EXPECT_EQ(left.mMeasuredSlipSteps, 0u);
            EXPECT_TRUE(ended.mClosedSegments[1]->hasCompleteEpisodeTravel());
            EXPECT_DOUBLE_EQ(*ended.mClosedSegments[1]->mObservedMaximumTangentTravel, 0.0);
        }
    }

    TEST(RagdollCoreBalanceEpisodeTest, smallProductRoundoffIsBoundedByFloatUlpsRatherThanWorldUnits)
    {
        constexpr float speed = 1.0e-5f;
        constexpr float expected = speed * stepDuration;
        for (const float direction : { 0.0f, std::numeric_limits<float>::infinity() })
        {
            float distance = expected;
            for (unsigned ulps = 0; ulps <= 3; ++ulps)
            {
                BalanceSupportEpisodeAccumulator accumulator;
                accumulator.reset(generation);
                accumulator.consume(generation, 1, stepDuration, { unsupported(), unsupported() });
                accumulator.consume(generation, 2, stepDuration,
                    { BalanceFootSlipMeasurement{ Availability::Supported, speed, distance }, unsupported() });
                const auto ended = accumulator.consume(generation, 3, stepDuration, { unsupported(), unsupported() });
                ASSERT_TRUE(ended.mClosedSegments[0]);
                const auto& segment = *ended.mClosedSegments[0];
                EXPECT_EQ(segment.hasCompleteEpisodeTravel(), ulps <= 2) << "ULPs=" << ulps;
                if (ulps <= 2)
                {
                    ASSERT_TRUE(segment.mObservedMaximumTangentTravel);
                    EXPECT_DOUBLE_EQ(*segment.mObservedMaximumTangentTravel, static_cast<double>(distance));
                }
                else
                    EXPECT_FALSE(segment.mObservedMaximumTangentTravel);
                distance = std::nextafter(distance, direction);
            }
        }
    }

    TEST(RagdollCoreBalanceEpisodeTest, overflowedFloatProductCannotBecomeCompleteTravel)
    {
        BalanceSupportEpisodeAccumulator accumulator;
        accumulator.reset(generation);
        constexpr float maximum = std::numeric_limits<float>::max();
        accumulator.consume(generation, 1, 2.0f, { unsupported(), unsupported() });
        accumulator.consume(generation, 2, 2.0f,
            { BalanceFootSlipMeasurement{ Availability::Supported, maximum, maximum }, supported(0, 2.0f) });
        const auto ended = accumulator.consume(generation, 3, 2.0f, { unsupported(), unsupported() });
        ASSERT_TRUE(ended.mClosedSegments[0] && ended.mClosedSegments[1]);
        EXPECT_TRUE(ended.mClosedSegments[0]->hasCompleteSupportEpisode());
        EXPECT_FALSE(ended.mClosedSegments[0]->hasCompleteEpisodeTravel());
        EXPECT_FALSE(ended.mClosedSegments[0]->mObservedMaximumTangentTravel);
        EXPECT_TRUE(ended.mClosedSegments[1]->hasCompleteEpisodeTravel());
    }

}
