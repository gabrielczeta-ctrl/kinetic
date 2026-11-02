#include <ragdollcore/staggerplan.hpp>

#include <gtest/gtest.h>

namespace
{
    using namespace RagdollCore;

    StaggerSettings on()
    {
        StaggerSettings s;
        s.mEnabled = true;
        return s;
    }

    StaggerStart start(ReactionTier tier, float dv, JPH::Vec3 direction = JPH::Vec3::sAxisY())
    {
        StaggerStart st;
        st.mTier = tier;
        st.mDv = dv;
        st.mDirection = direction;
        return st;
    }

    TEST(RagdollCoreStaggerPlan, tiersAskForTheirSteps)
    {
        const auto s = on();
        EXPECT_EQ(StaggerPlan::stepsFor(start(ReactionTier::Absorb, 200), s), 0);
        EXPECT_EQ(StaggerPlan::stepsFor(start(ReactionTier::Flinch, 10), s), 0);
        EXPECT_EQ(StaggerPlan::stepsFor(start(ReactionTier::Flinch, 40), s), 1);
        EXPECT_EQ(StaggerPlan::stepsFor(start(ReactionTier::Stagger, 50), s), 2);
        EXPECT_EQ(StaggerPlan::stepsFor(start(ReactionTier::Stagger, 120), s), 4);
        EXPECT_EQ(StaggerPlan::stepsFor(start(ReactionTier::Stagger, 200), s), 5);
        auto tired = start(ReactionTier::Stagger, 50);
        tired.mFatigue = 0.1f;
        EXPECT_EQ(StaggerPlan::stepsFor(tired, s), 3);
        auto skilled = start(ReactionTier::Stagger, 50);
        skilled.mSkill = 0.9f;
        EXPECT_EQ(StaggerPlan::stepsFor(skilled, s), 1);
        StaggerSettings off;
        EXPECT_EQ(StaggerPlan::stepsFor(start(ReactionTier::Stagger, 200), off), 0);
    }

    TEST(RagdollCoreStaggerPlan, stepsAlternateAlongThePushAndUnderRecover)
    {
        StaggerPlan plan;
        ASSERT_TRUE(plan.begin(start(ReactionTier::Stagger, 120), on()));
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 5, 0) } };
        const JPH::RVec3 com(0, 0, 90), capture(0, 80, 0);
        auto first = plan.step(1.0f / 60.0f, feet, com, capture, 150.0f);
        ASSERT_TRUE(first.has_value());
        EXPECT_EQ(first->mFoot, 0u); // the left foot is behind along +Y: it steps past
        EXPECT_GT(first->mLanding.GetY(), 20.0);
        EXPECT_LT(first->mLanding.GetY(), 80.0); // short of the capture point
        // While it swings no step; after it lands the other foot goes.
        feet[0].mSwinging = true;
        EXPECT_FALSE(plan.step(1.0f / 60.0f, feet, com, capture, 150.0f).has_value());
        feet[0].mSwinging = false;
        std::optional<StaggerStep> second;
        for (int i = 0; i < 10 && !second; ++i)
            second = plan.step(1.0f / 60.0f, feet, com, capture, 150.0f);
        ASSERT_TRUE(second.has_value());
        EXPECT_EQ(second->mFoot, 1u);
        EXPECT_EQ(plan.stepsLeft(), 2);
    }

    TEST(RagdollCoreStaggerPlan, aStoppedBodyEndsThePlan)
    {
        StaggerPlan plan;
        ASSERT_TRUE(plan.begin(start(ReactionTier::Stagger, 50), on()));
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        ASSERT_TRUE(plan.step(1.0f / 60.0f, feet, JPH::RVec3(0, 0, 90), JPH::RVec3(0, 30, 0), 80.0f).has_value());
        for (int i = 0; i < 30; ++i)
            plan.step(1.0f / 60.0f, feet, JPH::RVec3(0, 0, 90), JPH::RVec3(0, 0, 0), 2.0f);
        EXPECT_FALSE(plan.active());
        EXPECT_GT(plan.posture(), 0.0f); // easing out
    }

    TEST(RagdollCoreStaggerPlan, aStepThatNeverLandsDoesNotHoldThePlan)
    {
        // One step (a flinch): its foot keeps swinging (it never lands, the
        // body dragged along) - the plan lets go after 0.5 + 0.45 s.
        StaggerPlan plan;
        ASSERT_TRUE(plan.begin(start(ReactionTier::Flinch, 40), on()));
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        ASSERT_TRUE(plan.step(1.0f / 60.0f, feet, JPH::RVec3(0, 0, 90), JPH::RVec3(0, 40, 0), 130.0f).has_value());
        feet[0].mSwinging = true;
        int frames = 0;
        while (plan.active() && frames < 600)
        {
            plan.step(1.0f / 60.0f, feet, JPH::RVec3(0, 0, 90), JPH::RVec3(0, 40, 0), 130.0f);
            ++frames;
        }
        EXPECT_FALSE(plan.active());
        EXPECT_LE(frames, 60); // within ~1 s, not the 3 s maximum
    }

    TEST(RagdollCoreStaggerPlan, aSidewaysPushStepsTheLeadingFootOut)
    {
        StaggerPlan plan;
        ASSERT_TRUE(plan.begin(start(ReactionTier::Stagger, 90, JPH::Vec3::sAxisX()), on()));
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        const auto first = plan.step(1.0f / 60.0f, feet, JPH::RVec3(0, 0, 90), JPH::RVec3(60, 0, 0), 120.0f);
        ASSERT_TRUE(first.has_value());
        EXPECT_EQ(first->mFoot, 1u); // the right foot leads toward +X
        EXPECT_GT(first->mLanding.GetX(), 15.0);
    }

    TEST(RagdollCoreStaggerPlan, aPushFromTheFrontMidStrideDoesNotCrossTheLegs)
    {
        // Walking along +Y, feet a stride apart fore-and-aft, pushed back
        // (-Y): not a sideways push, so no step lands across the midline.
        auto st = start(ReactionTier::Stagger, 120, -JPH::Vec3::sAxisY());
        st.mFacing = JPH::Vec3::sAxisY();
        StaggerPlan plan;
        ASSERT_TRUE(plan.begin(st, on()));
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-5, 15, 0) }, StaggerFoot{ JPH::RVec3(5, -15, 0) } };
        const JPH::RVec3 com(0, 0, 90), capture(0, -60, 0);
        for (int n = 0; n < 2; ++n)
        {
            std::optional<StaggerStep> next;
            for (int i = 0; i < 10 && !next; ++i)
                next = plan.step(1.0f / 60.0f, feet, com, capture, 120.0f);
            ASSERT_TRUE(next.has_value());
            // The left foot (x < 0) stays left, the right right.
            EXPECT_EQ(next->mLanding.GetX() < 0.0, next->mFoot == 0u);
            feet[next->mFoot].mPosition = next->mLanding;
        }
    }

    TEST(RagdollCoreStaggerPlan, anExtendedPlanTimesOutOnTheStepsLeft)
    {
        // Three steps taken, then a light blow adds one: the plan lets go
        // within 0.5 + 0.45 s of that blow even if the step never lands.
        StaggerPlan plan;
        ASSERT_TRUE(plan.begin(start(ReactionTier::Stagger, 120), on()));
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        for (int taken = 0; taken < 3;)
        {
            if (plan.step(1.0f / 60.0f, feet, JPH::RVec3(0, 0, 90), JPH::RVec3(0, 80, 0), 150.0f))
                ++taken;
        }
        ASSERT_TRUE(plan.begin(start(ReactionTier::Flinch, 40), on()));
        feet[0].mSwinging = true;
        int frames = 0;
        while (plan.active() && frames < 600)
        {
            plan.step(1.0f / 60.0f, feet, JPH::RVec3(0, 0, 90), JPH::RVec3(0, 80, 0), 150.0f);
            ++frames;
        }
        EXPECT_LE(frames, 60);
    }

    // STEP-01 S3: chained steps follow the capture point, not a count.
    StaggerSettings chained()
    {
        StaggerSettings s = on();
        s.mChain = true;
        s.mStepBudget = 6;
        return s;
    }

    // Steps once, lets the foot land, returns the step (or none).
    std::optional<StaggerStep> stepAndLand(StaggerPlan& plan, std::array<StaggerFoot, 2>& feet, JPH::RVec3 com,
        JPH::RVec3 capture)
    {
        for (int i = 0; i < 30; ++i)
            if (const auto s = plan.step(1.0f / 60.0f, feet, com, capture, 150.0f))
            {
                feet[s->mFoot].mPosition = JPH::RVec3(s->mLanding.GetX(), s->mLanding.GetY(), 0.0);
                return s;
            }
        return std::nullopt;
    }

    TEST(RagdollCoreStaggerPlan, chainedStepsGoOnWhileTheCaptureIsOutside)
    {
        // A flinch counts one step; chained, the plan keeps stepping while
        // the capture point stays ahead of the feet.
        StaggerPlan plan;
        ASSERT_TRUE(plan.begin(start(ReactionTier::Stagger, 40), chained()));
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        int steps = 0;
        for (float y = 0.0f; steps < 4; y += 40.0f)
        {
            const auto s = stepAndLand(plan, feet, JPH::RVec3(0, y, 90), JPH::RVec3(0, y + 100, 0));
            ASSERT_TRUE(s.has_value()) << "step " << steps;
            ++steps;
        }
        EXPECT_TRUE(plan.active());
        EXPECT_TRUE(plan.captureOutside());
    }

    TEST(RagdollCoreStaggerPlan, chainedStepsReachForTheCapturePoint)
    {
        StaggerPlan plan;
        ASSERT_TRUE(plan.begin(start(ReactionTier::Stagger, 120), chained()));
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        // 50 u ahead: past the counted plan's 38 u cap, inside the reach.
        const auto s = stepAndLand(plan, feet, JPH::RVec3(0, 0, 90), JPH::RVec3(0, 50, 0));
        ASSERT_TRUE(s.has_value());
        EXPECT_NEAR(s->mLanding.GetY(), 45.0, 1.0); // 0.9 of the way
        // Sideways capture: the step goes sideways though the blow was +Y.
        StaggerPlan side;
        ASSERT_TRUE(side.begin(start(ReactionTier::Stagger, 120), chained()));
        std::array<StaggerFoot, 2> feet2{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        const auto t = stepAndLand(side, feet2, JPH::RVec3(0, 0, 90), JPH::RVec3(60, 0, 0));
        ASSERT_TRUE(t.has_value());
        EXPECT_GT(t->mLanding.GetX(), 40.0);
        EXPECT_LT(std::abs(t->mLanding.GetY()), 10.0);
    }

    TEST(RagdollCoreStaggerPlan, chainedPlanEndsWhenCaughtOrOutOfSteps)
    {
        StaggerPlan plan;
        ASSERT_TRUE(plan.begin(start(ReactionTier::Stagger, 120), chained()));
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        ASSERT_TRUE(stepAndLand(plan, feet, JPH::RVec3(0, 0, 90), JPH::RVec3(0, 30, 0)).has_value());
        // The capture point now between the feet: no more steps, the plan ends.
        const JPH::RVec3 between = (feet[0].mPosition + feet[1].mPosition) * 0.5;
        for (int i = 0; i < 30; ++i)
            EXPECT_FALSE(plan.step(1.0f / 60.0f, feet, JPH::RVec3(between.GetX(), between.GetY(), 90), between, 150.0f));
        EXPECT_FALSE(plan.active());
        // A budget of 2 stops after two steps whatever the capture point.
        StaggerSettings two = chained();
        two.mStepBudget = 2;
        StaggerPlan short2;
        ASSERT_TRUE(short2.begin(start(ReactionTier::Stagger, 120), two));
        std::array<StaggerFoot, 2> feet2{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        EXPECT_TRUE(stepAndLand(short2, feet2, JPH::RVec3(0, 0, 90), JPH::RVec3(0, 200, 0)).has_value());
        EXPECT_TRUE(stepAndLand(short2, feet2, JPH::RVec3(0, 40, 90), JPH::RVec3(0, 240, 0)).has_value());
        EXPECT_FALSE(stepAndLand(short2, feet2, JPH::RVec3(0, 80, 90), JPH::RVec3(0, 280, 0)).has_value());
    }

    // STEP-01 S4: the budget by agility, and the give-up.
    StaggerSettings givingUp()
    {
        StaggerSettings s = chained();
        s.mGiveUp = true;
        return s;
    }

    TEST(RagdollCoreStaggerPlan, budgetFollowsAgilityTirednessAndHurt)
    {
        const StaggerSettings s = givingUp();
        StaggerStart st = start(ReactionTier::Stagger, 120);
        EXPECT_EQ(StaggerPlan::budgetFor(st, s), 6); // no agility given: the settings' budget
        st.mAgility = 0.0f;
        EXPECT_EQ(StaggerPlan::budgetFor(st, s), 2);
        st.mAgility = 50.0f;
        EXPECT_EQ(StaggerPlan::budgetFor(st, s), 4);
        st.mAgility = 150.0f;
        EXPECT_EQ(StaggerPlan::budgetFor(st, s), 6); // capped by mStepBudget
        st.mAgility = 50.0f;
        st.mFatigue = 0.2f;
        EXPECT_EQ(StaggerPlan::budgetFor(st, s), 3);
        st.mHealth = 0.3f;
        EXPECT_EQ(StaggerPlan::budgetFor(st, s), 2);
        st.mAgility = 0.0f;
        EXPECT_EQ(StaggerPlan::budgetFor(st, s), 1); // never none
        EXPECT_EQ(StaggerPlan::budgetFor(st, chained()), 6); // give-up off: the settings' budget
    }

    TEST(RagdollCoreStaggerPlan, stepsSpentWithTheWeightStillGoingGiveUp)
    {
        StaggerStart st = start(ReactionTier::Stagger, 120);
        st.mAgility = 0.0f; // two steps
        StaggerPlan plan;
        ASSERT_TRUE(plan.begin(st, givingUp()));
        EXPECT_EQ(plan.stepsLeft(), 2);
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        EXPECT_TRUE(stepAndLand(plan, feet, JPH::RVec3(0, 0, 90), JPH::RVec3(0, 200, 0)).has_value());
        EXPECT_TRUE(stepAndLand(plan, feet, JPH::RVec3(0, 40, 90), JPH::RVec3(0, 240, 0)).has_value());
        // The weight past the feet (0.39 of its height) and still going.
        EXPECT_FALSE(stepAndLand(plan, feet, JPH::RVec3(0, 130, 90), JPH::RVec3(0, 260, 0)).has_value());
        EXPECT_FALSE(plan.active());
        ASSERT_NE(plan.gaveUp(), nullptr);
        EXPECT_STREQ(plan.gaveUp(), "spent");

        // The same steps spent upright, the weight between the feet, the body
        // only moving on (walking): the plan ends, no fall.
        StaggerPlan walking;
        ASSERT_TRUE(walking.begin(st, givingUp()));
        std::array<StaggerFoot, 2> feet3{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        EXPECT_TRUE(stepAndLand(walking, feet3, JPH::RVec3(0, 0, 90), JPH::RVec3(0, 200, 0)).has_value());
        EXPECT_TRUE(stepAndLand(walking, feet3, JPH::RVec3(0, 40, 90), JPH::RVec3(0, 240, 0)).has_value());
        const JPH::RVec3 mid = (feet3[0].mPosition + feet3[1].mPosition) * 0.5;
        for (int i = 0; i < 30; ++i)
            walking.step(1.0f / 60.0f, feet3, JPH::RVec3(mid.GetX(), mid.GetY(), 90), JPH::RVec3(0, 260, 0), 150.0f);
        EXPECT_FALSE(walking.active());
        EXPECT_EQ(walking.gaveUp(), nullptr);

        // The same steps with the weight caught by the last one: no give-up.
        StaggerPlan caught;
        ASSERT_TRUE(caught.begin(st, givingUp()));
        std::array<StaggerFoot, 2> feet2{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        EXPECT_TRUE(stepAndLand(caught, feet2, JPH::RVec3(0, 0, 90), JPH::RVec3(0, 60, 0)).has_value());
        EXPECT_TRUE(stepAndLand(caught, feet2, JPH::RVec3(0, 30, 90), JPH::RVec3(0, 70, 0)).has_value());
        const JPH::RVec3 between = (feet2[0].mPosition + feet2[1].mPosition) * 0.5;
        for (int i = 0; i < 30; ++i)
            caught.step(1.0f / 60.0f, feet2, JPH::RVec3(between.GetX(), between.GetY(), 90), between, 150.0f);
        EXPECT_FALSE(caught.active());
        EXPECT_EQ(caught.gaveUp(), nullptr);
    }

    TEST(RagdollCoreStaggerPlan, aFlinchNeitherChainsNorGivesUp)
    {
        StaggerStart st = start(ReactionTier::Flinch, 40);
        st.mAgility = 50.0f;
        StaggerPlan plan;
        ASSERT_TRUE(plan.begin(st, givingUp()));
        EXPECT_EQ(plan.stepsLeft(), 1);
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        // Leaning far out: a flinch only steps.
        EXPECT_TRUE(plan.step(1.0f / 60.0f, feet, JPH::RVec3(0, 70, 90), JPH::RVec3(0, 120, 0), 150.0f).has_value());
        EXPECT_EQ(plan.gaveUp(), nullptr);
    }

    TEST(RagdollCoreStaggerPlan, aBlowDuringAChainedStaggerAddsAFewSteps)
    {
        StaggerStart st = start(ReactionTier::Stagger, 120);
        st.mAgility = 100.0f; // budget 6
        StaggerPlan plan;
        ASSERT_TRUE(plan.begin(st, givingUp()));
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        for (int i = 0; i < 5; ++i)
            ASSERT_TRUE(stepAndLand(plan, feet, JPH::RVec3(0, 20.0 * i, 90), JPH::RVec3(0, 20.0 * i + 200, 0)).has_value());
        EXPECT_EQ(plan.stepsLeft(), 1);
        ASSERT_TRUE(plan.begin(st, givingUp()));
        EXPECT_EQ(plan.stepsLeft(), 3); // 1 left + 2, not a new budget of 6
    }

    TEST(RagdollCoreStaggerPlan, leaningPastTheAbortRatioGivesUp)
    {
        std::array<StaggerFoot, 2> feet{ StaggerFoot{ JPH::RVec3(-8, 0, 0) }, StaggerFoot{ JPH::RVec3(8, 0, 0) } };
        // 0.6 x 90 = 54: 30 u out is a stagger, 70 u out is lost.
        StaggerPlan near;
        ASSERT_TRUE(near.begin(start(ReactionTier::Stagger, 120), givingUp()));
        EXPECT_TRUE(near.step(1.0f / 60.0f, feet, JPH::RVec3(0, 30, 90), JPH::RVec3(0, 80, 0), 150.0f).has_value());
        EXPECT_EQ(near.gaveUp(), nullptr);
        StaggerPlan far;
        ASSERT_TRUE(far.begin(start(ReactionTier::Stagger, 120), givingUp()));
        EXPECT_FALSE(far.step(1.0f / 60.0f, feet, JPH::RVec3(0, 70, 90), JPH::RVec3(0, 120, 0), 150.0f).has_value());
        EXPECT_FALSE(far.active());
        ASSERT_NE(far.gaveUp(), nullptr);
        EXPECT_STREQ(far.gaveUp(), "lean");
        // Held (the actor asks to move): the same lean only steps.
        StaggerPlan moving;
        ASSERT_TRUE(moving.begin(start(ReactionTier::Stagger, 120), givingUp()));
        moving.holdGiveUp(true);
        EXPECT_TRUE(moving.step(1.0f / 60.0f, feet, JPH::RVec3(0, 70, 90), JPH::RVec3(0, 120, 0), 150.0f).has_value());
        EXPECT_EQ(moving.gaveUp(), nullptr);
        // Give-up off: the same lean only steps.
        StaggerPlan off;
        ASSERT_TRUE(off.begin(start(ReactionTier::Stagger, 120), chained()));
        EXPECT_TRUE(off.step(1.0f / 60.0f, feet, JPH::RVec3(0, 70, 90), JPH::RVec3(0, 120, 0), 150.0f).has_value());
        EXPECT_EQ(off.gaveUp(), nullptr);
    }
}
