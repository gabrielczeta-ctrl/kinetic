#include <gtest/gtest.h>

#include <ragdollcore/physicaldefense.hpp>

#include <limits>

namespace
{
    RagdollCore::PhysicalDefenseInput direct(std::uint64_t contactStep = 100)
    {
        return { true, false, false, contactStep, 0, 12.0f, 15.0f, 1.0f };
    }

    void record(const char* control, const RagdollCore::PhysicalDefenseResult& result)
    {
        ::testing::Test::RecordProperty("control", control);
        ::testing::Test::RecordProperty("outcome",
            std::string(RagdollCore::physicalDefenseOutcomeName(result.mOutcome)));
        ::testing::Test::RecordProperty("disposition",
            std::string(RagdollCore::physicalContactDispositionName(result.mDisposition)));
        ::testing::Test::RecordProperty("event_count", result.emitsOutcome() ? 1 : 0);
        ::testing::Test::RecordProperty("damage_scale", result.mDamageScale);
        ::testing::Test::RecordProperty("guard_facing_alignment", result.mGuardFacingAlignment);
        ::testing::Test::RecordProperty("impulse_scale", result.mImpulseScale);
        ::testing::Test::RecordProperty("redirect_scale", result.mRedirectScale);
    }
}

TEST(RagdollCorePhysicalDefenseTest, directContactProducesOneHit)
{
    const auto result = RagdollCore::classifyPhysicalDefense(direct());
    EXPECT_EQ(result.mOutcome, RagdollCore::PhysicalDefenseOutcome::Hit);
    EXPECT_EQ(result.mDisposition, RagdollCore::PhysicalContactDisposition::Direct);
    EXPECT_FLOAT_EQ(result.mDamageScale, 1.0f);
    EXPECT_FLOAT_EQ(result.mImpulseScale, 1.0f);
    record("hit", result);
}

TEST(RagdollCorePhysicalDefenseTest, establishedGuardProducesOneBlock)
{
    auto input = direct();
    input.mGuardActive = true;
    input.mGuardStartStep = 80;
    const auto result = RagdollCore::classifyPhysicalDefense(input);
    EXPECT_EQ(result.mOutcome, RagdollCore::PhysicalDefenseOutcome::Block);
    EXPECT_FLOAT_EQ(result.mDamageScale, 0.0f);
    EXPECT_GT(result.mRedirectScale, 0.0f);
    record("block", result);
}

TEST(RagdollCorePhysicalDefenseTest, shallowContactProducesOneReducedHit)
{
    auto input = direct();
    input.mRelativeNormalSpeed = 2.0f;
    input.mRelativeSpeed = 10.0f;
    const auto result = RagdollCore::classifyPhysicalDefense(input);
    EXPECT_EQ(result.mOutcome, RagdollCore::PhysicalDefenseOutcome::Hit);
    EXPECT_EQ(result.mDisposition, RagdollCore::PhysicalContactDisposition::Glancing);
    EXPECT_GT(result.mDamageScale, 0.0f);
    EXPECT_LT(result.mDamageScale, 1.0f);
    EXPECT_GT(result.mImpulseScale, 0.0f);
    EXPECT_LT(result.mImpulseScale, 1.0f);
    record("glancing", result);
}

TEST(RagdollCorePhysicalDefenseTest, newlyRaisedGuardProducesOneParry)
{
    auto input = direct();
    input.mGuardActive = true;
    input.mGuardStartStep = 94;
    const auto result = RagdollCore::classifyPhysicalDefense(input);
    EXPECT_EQ(result.mOutcome, RagdollCore::PhysicalDefenseOutcome::Parry);
    EXPECT_FLOAT_EQ(result.mDamageScale, 0.0f);
    EXPECT_GT(result.mRedirectScale, 0.0f);
    record("timed_parry", result);
}

TEST(RagdollCorePhysicalDefenseTest, lateGuardProducesOneHit)
{
    auto input = direct();
    input.mGuardActive = true;
    input.mGuardStartStep = 20;
    const auto result = RagdollCore::classifyPhysicalDefense(input);
    EXPECT_EQ(result.mOutcome, RagdollCore::PhysicalDefenseOutcome::Hit);
    record("late_parry", result);
}

TEST(RagdollCorePhysicalDefenseTest, restProducesNoOutcome)
{
    auto input = direct();
    input.mRelativeNormalSpeed = 0.25f;
    const auto result = RagdollCore::classifyPhysicalDefense(input);
    EXPECT_EQ(result.mOutcome, RagdollCore::PhysicalDefenseOutcome::None);
    EXPECT_EQ(result.mDisposition, RagdollCore::PhysicalContactDisposition::Rest);
    record("rest", result);
}

TEST(RagdollCorePhysicalDefenseTest, friendlyContactProducesNoOutcome)
{
    auto input = direct();
    input.mFriendlyContact = true;
    const auto result = RagdollCore::classifyPhysicalDefense(input);
    EXPECT_EQ(result.mOutcome, RagdollCore::PhysicalDefenseOutcome::None);
    EXPECT_EQ(result.mDisposition, RagdollCore::PhysicalContactDisposition::Friendly);
    record("friendly", result);
}

TEST(RagdollCorePhysicalDefenseTest, uncoveredAndInvalidGuardsCannotDefend)
{
    auto behind = direct();
    behind.mGuardActive = true;
    behind.mGuardStartStep = 95;
    behind.mGuardFacingAlignment = -1.0f;
    EXPECT_EQ(RagdollCore::classifyPhysicalDefense(behind).mOutcome,
        RagdollCore::PhysicalDefenseOutcome::Hit);

    auto future = direct();
    future.mGuardActive = true;
    future.mGuardStartStep = 101;
    EXPECT_EQ(RagdollCore::classifyPhysicalDefense(future).mOutcome,
        RagdollCore::PhysicalDefenseOutcome::Hit);

    auto invalid = direct();
    invalid.mRelativeSpeed = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(RagdollCore::classifyPhysicalDefense(invalid).emitsOutcome());
}

TEST(RagdollCorePhysicalDefenseTest, aHeldGuardKeepsBlockingWhatItCovers)
{
    // A guard raised at step 0, the blow at step 600 (10 s later).
    RagdollCore::PhysicalDefenseInput input = direct(600);
    input.mGuardActive = true;
    EXPECT_EQ(RagdollCore::classifyPhysicalDefense(input).mOutcome, RagdollCore::PhysicalDefenseOutcome::Hit);
    RagdollCore::PhysicalDefenseProfile held;
    held.mHeldGuardBlocks = true;
    EXPECT_EQ(RagdollCore::classifyPhysicalDefense(input, held).mOutcome, RagdollCore::PhysicalDefenseOutcome::Block);
    // Still only in front of it.
    input.mGuardFacingAlignment = 0.2f;
    EXPECT_EQ(RagdollCore::classifyPhysicalDefense(input, held).mOutcome, RagdollCore::PhysicalDefenseOutcome::Hit);
}
