#include <gtest/gtest.h>

#include <ragdollcore/balanceauthority.hpp>

#include <cmath>
#include <limits>

using namespace RagdollCore;

namespace
{
    constexpr BalanceAuthorityGeneration Generation{ 7, 3 };

    BalanceDisturbanceReceipt receipt(std::uint64_t serial, std::uint64_t occurrence,
        BalanceDisturbancePhase phase, float magnitude)
    {
        return { Generation, serial, 100 + serial, occurrence,
            phase == BalanceDisturbancePhase::BeforeUpdate ? occurrence : occurrence + 1,
            phase, magnitude, 17 };
    }

    BalanceAuthorityInput input(std::uint64_t step, BalanceSupportAvailability support,
        std::optional<bool> walking = false, float duration = 1.0f / 60.0f)
    {
        return { Generation, step, duration, support, walking };
    }
}

TEST(RagdollCoreBalanceAuthorityTest, RampIsFiniteSmoothAndClamped)
{
    EXPECT_FLOAT_EQ(balanceAuthorityRamp(-5.0f, 2.0f, 12.0f), 0.0f);
    EXPECT_FLOAT_EQ(balanceAuthorityRamp(2.0f, 2.0f, 12.0f), 0.0f);
    EXPECT_NEAR(balanceAuthorityRamp(7.0f, 2.0f, 12.0f), 0.5f, 1e-6f);
    EXPECT_FLOAT_EQ(balanceAuthorityRamp(12.0f, 2.0f, 12.0f), 1.0f);
    EXPECT_FLOAT_EQ(balanceAuthorityRamp(99.0f, 5.0f, 5.0f), 1.0f);
    EXPECT_FLOAT_EQ(balanceAuthorityRamp(std::numeric_limits<float>::quiet_NaN(), 2.0f, 12.0f), 0.0f);
}

TEST(RagdollCoreBalanceAuthorityTest, ProcessedIntentSelectsStandingAndWalkingBaselines)
{
    BalanceAuthorityController standing;
    BalanceAuthorityController walking;
    BalanceAuthorityController unknown;
    ASSERT_TRUE(standing.reset(Generation).mChanged);
    ASSERT_TRUE(walking.reset(Generation).mChanged);
    ASSERT_TRUE(unknown.reset(Generation).mChanged);
    BalanceAuthorityStep standingStep, walkingStep, unknownStep;
    for (std::uint64_t step = 1; step <= 60; ++step)
    {
        standingStep = standing.consume(input(step, BalanceSupportAvailability::Supported, false));
        walkingStep = walking.consume(input(step, BalanceSupportAvailability::Supported, true));
        unknownStep = unknown.consume(input(step, BalanceSupportAvailability::Supported, std::nullopt));
    }
    EXPECT_TRUE(standingStep.mBaselineAvailable);
    EXPECT_TRUE(walkingStep.mBaselineAvailable);
    EXPECT_FALSE(unknownStep.mBaselineAvailable);
    EXPECT_NEAR(standingStep.mAuthority, 0.5f, 1e-6f);
    EXPECT_NEAR(walkingStep.mAuthority, 0.25f, 1e-6f);
    EXPECT_FLOAT_EQ(unknownStep.mAuthority, 0.0f);
}

TEST(RagdollCoreBalanceAuthorityTest, OnePulseIsConsumedOnceAndReleasesAtDeclaredRate)
{
    BalanceAuthorityController controller;
    controller.reset(Generation);
    ASSERT_EQ(controller.receive(receipt(1, 1, BalanceDisturbancePhase::BeforeUpdate, 3000.0f)),
        BalanceReceiptStatus::Accepted);
    const auto pulse = controller.consume(input(1, BalanceSupportAvailability::Supported, false));
    EXPECT_EQ(pulse.mConsumedReceiptCount, 1u);
    EXPECT_FLOAT_EQ(pulse.mConsumedImpulse, 3000.0f);
    EXPECT_NEAR(pulse.mAuthority, 0.1f, 1e-6f);
    const auto release = controller.consume(input(2, BalanceSupportAvailability::Supported, false));
    EXPECT_EQ(release.mConsumedReceiptCount, 0u);
    EXPECT_FLOAT_EQ(release.mConsumedImpulse, 0.0f);
    // The standing baseline is above 0.1, so ordinary baseline attack resumes;
    // the event itself was not replayed.
    EXPECT_NEAR(release.mAuthority, 0.2f, 1e-6f);
    EXPECT_EQ(controller.getPendingReceiptCount(), 0u);
}

TEST(RagdollCoreBalanceAuthorityTest, MultipleDistinctReceiptsUseMaximumNotSum)
{
    BalanceAuthorityController controller;
    controller.reset(Generation);
    ASSERT_EQ(controller.receive(receipt(1, 1, BalanceDisturbancePhase::BeforeUpdate, 900.0f)),
        BalanceReceiptStatus::Accepted);
    ASSERT_EQ(controller.receive(receipt(2, 1, BalanceDisturbancePhase::BeforeUpdate, 1800.0f)),
        BalanceReceiptStatus::Accepted);
    ASSERT_EQ(controller.receive(receipt(3, 1, BalanceDisturbancePhase::BeforeUpdate, 1200.0f)),
        BalanceReceiptStatus::Accepted);
    const auto result = controller.consume(input(1, BalanceSupportAvailability::Supported, false));
    EXPECT_EQ(result.mConsumedReceiptCount, 3u);
    EXPECT_EQ(result.mFirstConsumedReceipt, 1u);
    EXPECT_EQ(result.mLastConsumedReceipt, 3u);
    EXPECT_FLOAT_EQ(result.mConsumedImpulse, 1800.0f);
    EXPECT_LT(result.mDisturbance, 1.0f);
}

TEST(RagdollCoreBalanceAuthorityTest, PhaseGenerationAndMonotonicIdentityAreEnforced)
{
    BalanceAuthorityController controller;
    EXPECT_EQ(controller.receive(receipt(1, 1, BalanceDisturbancePhase::BeforeUpdate, 1000.0f)),
        BalanceReceiptStatus::UnboundGeneration);
    controller.reset(Generation);
    auto wrong = receipt(1, 1, BalanceDisturbancePhase::BeforeUpdate, 1000.0f);
    wrong.mGeneration.mLifecycleEpoch++;
    EXPECT_EQ(controller.receive(wrong), BalanceReceiptStatus::WrongGeneration);
    auto impossible = receipt(1, 1, BalanceDisturbancePhase::AfterUpdate, 1000.0f);
    impossible.mFirstUsablePhysicsStep = 1;
    EXPECT_EQ(controller.receive(impossible), BalanceReceiptStatus::InvalidReceipt);
    ASSERT_EQ(controller.receive(receipt(2, 1, BalanceDisturbancePhase::AfterUpdate, 1000.0f)),
        BalanceReceiptStatus::Accepted);
    EXPECT_EQ(controller.receive(receipt(2, 2, BalanceDisturbancePhase::BeforeUpdate, 1000.0f)),
        BalanceReceiptStatus::DuplicateOrOlderReceipt);
    EXPECT_EQ(controller.receive(receipt(1, 2, BalanceDisturbancePhase::BeforeUpdate, 1000.0f)),
        BalanceReceiptStatus::DuplicateOrOlderReceipt);
    EXPECT_EQ(controller.consume(input(1, BalanceSupportAvailability::Supported)).mConsumedReceiptCount, 0u);
    EXPECT_EQ(controller.consume(input(2, BalanceSupportAvailability::Supported)).mConsumedReceiptCount, 1u);
}

TEST(RagdollCoreBalanceAuthorityTest, UnknownAndMissingStepsBreakUnsupportedContinuityAndExpireEvents)
{
    BalanceAuthorityController controller;
    controller.reset(Generation);
    auto first = input(1, BalanceSupportAvailability::Unsupported, false, 0.05f);
    EXPECT_FLOAT_EQ(controller.consume(first).mUnsupportedTime, 0.05f);
    auto unknown = input(2, BalanceSupportAvailability::Unknown, false, 0.05f);
    EXPECT_FLOAT_EQ(controller.consume(unknown).mUnsupportedTime, 0.0f);
    EXPECT_FLOAT_EQ(controller.consume(input(3, BalanceSupportAvailability::Unsupported, false, 0.05f))
                        .mUnsupportedTime,
        0.05f);
    ASSERT_EQ(controller.receive(receipt(5, 4, BalanceDisturbancePhase::BeforeUpdate, 3000.0f)),
        BalanceReceiptStatus::Accepted);
    const auto gap = controller.consume(input(5, BalanceSupportAvailability::Unsupported, false, 0.05f));
    EXPECT_EQ(gap.mMissingPhysicsSteps, 1u);
    EXPECT_EQ(gap.mConsumedReceiptCount, 0u);
    EXPECT_FLOAT_EQ(gap.mUnsupportedTime, 0.05f);
    EXPECT_EQ(controller.getPendingReceiptCount(), 0u);
}

TEST(RagdollCoreBalanceAuthorityTest, RejectedStepsAndGettersCannotAdvanceState)
{
    BalanceAuthorityController controller;
    controller.reset(Generation);
    const auto first = controller.consume(input(1, BalanceSupportAvailability::Supported, false));
    const float authority = controller.getAuthority();
    EXPECT_FLOAT_EQ(controller.getAuthority(), authority);
    EXPECT_FLOAT_EQ(controller.getUnsupportedTime(), 0.0f);
    EXPECT_EQ(controller.consume(input(1, BalanceSupportAvailability::Unsupported, false)).mStatus,
        BalanceAuthorityConsumeStatus::DuplicatePhysicsStep);
    EXPECT_EQ(controller.consume(input(2, BalanceSupportAvailability::Unsupported, false, 0.0f)).mStatus,
        BalanceAuthorityConsumeStatus::InvalidDuration);
    EXPECT_FLOAT_EQ(controller.getAuthority(), authority);
    EXPECT_FLOAT_EQ(controller.getUnsupportedTime(), 0.0f);
    EXPECT_EQ(first.mStatus, BalanceAuthorityConsumeStatus::Consumed);
}

TEST(RagdollCoreBalanceAuthorityTest, LifecycleResetClearsStateAndSameGenerationResetIsNoOp)
{
    BalanceAuthorityController controller;
    controller.reset(Generation);
    controller.receive(receipt(1, 2, BalanceDisturbancePhase::BeforeUpdate, 3000.0f));
    controller.consume(input(1, BalanceSupportAvailability::Unsupported, false, 0.1f));
    EXPECT_FALSE(controller.reset(Generation).mChanged);
    EXPECT_EQ(controller.getPendingReceiptCount(), 1u);
    const BalanceAuthorityGeneration next{ Generation.mActorSerial, Generation.mLifecycleEpoch + 1 };
    const auto reset = controller.reset(next);
    EXPECT_TRUE(reset.mChanged);
    EXPECT_EQ(reset.mDiscardedReceipts, 1u);
    EXPECT_EQ(controller.getPendingReceiptCount(), 0u);
    EXPECT_FLOAT_EQ(controller.getAuthority(), 0.0f);
    EXPECT_FLOAT_EQ(controller.getUnsupportedTime(), 0.0f);
    EXPECT_EQ(controller.getLastReceiptSerial(), 0u);
}

TEST(RagdollCoreBalanceAuthorityTest, OwnershipSuspensionClearsWorkButRetainsReceiptFloor)
{
    BalanceAuthorityController controller;
    controller.reset(Generation);
    ASSERT_EQ(controller.receive(receipt(4, 2, BalanceDisturbancePhase::BeforeUpdate, 3000.0f)),
        BalanceReceiptStatus::Accepted);
    controller.consume(input(1, BalanceSupportAvailability::Unsupported, false, 0.1f));
    controller.suspend();
    EXPECT_EQ(controller.getPendingReceiptCount(), 0u);
    EXPECT_FLOAT_EQ(controller.getAuthority(), 0.0f);
    EXPECT_FLOAT_EQ(controller.getUnsupportedTime(), 0.0f);
    EXPECT_EQ(controller.getLastReceiptSerial(), 4u);
    EXPECT_EQ(controller.receive(receipt(4, 3, BalanceDisturbancePhase::BeforeUpdate, 3000.0f)),
        BalanceReceiptStatus::DuplicateOrOlderReceipt);
    EXPECT_EQ(controller.receive(receipt(5, 3, BalanceDisturbancePhase::BeforeUpdate, 3000.0f)),
        BalanceReceiptStatus::Accepted);
}

TEST(RagdollCoreBalanceAuthorityTest, RenderBatchingDoesNotChangeTheFixedStepSchedule)
{
    const auto run = [](int renderBatch) {
        BalanceAuthorityController controller;
        controller.reset(Generation);
        controller.receive(receipt(1, 4, BalanceDisturbancePhase::BeforeUpdate, 2500.0f));
        std::vector<float> history;
        for (std::uint64_t step = 1; step <= 12; ++step)
        {
            for (int getter = 0; getter < renderBatch; ++getter)
                (void)controller.getAuthority();
            history.push_back(controller.consume(input(step,
                step >= 7 && step <= 9 ? BalanceSupportAvailability::Unsupported
                                       : BalanceSupportAvailability::Supported,
                step >= 5)).mAuthority);
        }
        return history;
    };
    EXPECT_EQ(run(1), run(2));
    EXPECT_EQ(run(1), run(4));
}
