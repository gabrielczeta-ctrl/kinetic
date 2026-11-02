#include <ragdollcore/steparbiter.hpp>

#include <gtest/gtest.h>

namespace
{
    using namespace RagdollCore;

    TEST(RagdollCoreStepArbiter, theDefaultOrderIsTheEnginesOld)
    {
        const auto order = StepArbiter::defaultOrder();
        EXPECT_EQ(order[0], StepSource::Stuck);
        EXPECT_EQ(order[1], StepSource::Landing);
        EXPECT_EQ(order[2], StepSource::MovingCatch);
        EXPECT_EQ(order[3], StepSource::StandingCatch);
        EXPECT_EQ(order[4], StepSource::StageCatch);
    }

    TEST(RagdollCoreStepArbiter, parsesAnOrderNamingEachCatchOnce)
    {
        const auto order = StepArbiter::parseOrder("landing, stuck,moving,stage,standing");
        ASSERT_TRUE(order);
        EXPECT_EQ((*order)[0], StepSource::Landing);
        EXPECT_EQ((*order)[1], StepSource::Stuck);
        EXPECT_EQ((*order)[4], StepSource::StandingCatch);
        EXPECT_FALSE(StepArbiter::parseOrder("landing,stuck,moving,stage"));
        EXPECT_FALSE(StepArbiter::parseOrder("landing,landing,moving,stage,standing"));
        EXPECT_FALSE(StepArbiter::parseOrder("landing,stuck,moving,stage,standing,stagger"));
        EXPECT_FALSE(StepArbiter::parseOrder("landing,stuck,walking,stage,standing"));
    }

    TEST(RagdollCoreStepArbiter, reportsAChangeOfOwnerOnce)
    {
        StepArbiter arbiter;
        EXPECT_FALSE(arbiter.owner());
        EXPECT_TRUE(arbiter.claim(StepSource::StandingCatch, "catch"));
        EXPECT_FALSE(arbiter.claim(StepSource::StandingCatch, "catch"));
        EXPECT_TRUE(arbiter.claim(StepSource::StandingCatch, "release"));
        EXPECT_TRUE(arbiter.claim(StepSource::Landing, "run-out"));
        EXPECT_EQ(arbiter.owner(), StepSource::Landing);
        EXPECT_EQ(arbiter.reason(), "run-out");
        EXPECT_TRUE(arbiter.idle());
        EXPECT_FALSE(arbiter.idle());
        EXPECT_EQ(arbiter.steps(StepSource::StandingCatch), 3u);
        EXPECT_EQ(arbiter.steps(StepSource::Landing), 1u);
    }
}
