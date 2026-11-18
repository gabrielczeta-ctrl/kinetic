#include <ragdollcore/lodge.hpp>

#include <gtest/gtest.h>

namespace
{
    using namespace RagdollCore;

    LodgeSettings on()
    {
        LodgeSettings s;
        s.mStab = s.mChop = s.mPass = true;
        return s;
    }

    LodgeInput blow(BlowKind kind, BodyContactOutcome outcome, float damage, float vn, float vt, float hardness = 0.0f)
    {
        LodgeInput input;
        input.mKind = kind;
        input.mOutcome = outcome;
        input.mDamage = damage;
        input.mNormalSpeed = vn;
        input.mTangentSpeed = vt;
        input.mHardness = hardness;
        input.mCarriedMass = 1.6f;
        return input;
    }

    TEST(RagdollCoreLodge, offPlansNothing)
    {
        const LodgeSettings off;
        EXPECT_EQ(planLodge(blow(BlowKind::Point, BodyContactOutcome::Bite, 20, 600, 50), off).mKind, LodgeKind::None);
        EXPECT_EQ(planLodge(blow(BlowKind::Edge, BodyContactOutcome::Bite, 20, 600, 50), off).mKind, LodgeKind::None);
        EXPECT_EQ(planLodge(blow(BlowKind::Edge, BodyContactOutcome::Cleave, 20, 600, 900), off).mKind, LodgeKind::None);
    }

    TEST(RagdollCoreLodge, aPaidPointGoesInAsDeepAsItsDamage)
    {
        const auto poke = planLodge(blow(BlowKind::Point, BodyContactOutcome::Bite, 5, 400, 30), on());
        const auto thrust = planLodge(blow(BlowKind::Point, BodyContactOutcome::Bite, 25, 900, 30), on());
        const auto huge = planLodge(blow(BlowKind::Point, BodyContactOutcome::Bite, 200, 900, 30), on());
        EXPECT_EQ(poke.mKind, LodgeKind::Stab);
        EXPECT_FLOAT_EQ(poke.mDepth, 3.0f);
        EXPECT_FLOAT_EQ(thrust.mDepth, 15.0f);
        EXPECT_FLOAT_EQ(huge.mDepth, 20.0f);
        // Through armour that it paid through, too (a stop that hurt).
        EXPECT_EQ(planLodge(blow(BlowKind::Point, BodyContactOutcome::Stop, 8, 900, 30, 0.9f), on()).mKind,
            LodgeKind::Stab);
        // Nothing paid, or a skid: nothing.
        EXPECT_EQ(planLodge(blow(BlowKind::Point, BodyContactOutcome::Bite, 0, 900, 30), on()).mKind, LodgeKind::None);
        EXPECT_EQ(planLodge(blow(BlowKind::Point, BodyContactOutcome::Glance, 5, 100, 900), on()).mKind,
            LodgeKind::None);
    }

    TEST(RagdollCoreLodge, aSquareEdgeLodgesADrawnOnePasses)
    {
        const auto chop = planLodge(blow(BlowKind::Edge, BodyContactOutcome::Bite, 20, 800, 200), on());
        EXPECT_EQ(chop.mKind, LodgeKind::Chop);
        EXPECT_FLOAT_EQ(chop.mDepth, 5.0f);
        EXPECT_GT(chop.mHold, 0.0f);
        const auto drawn = planLodge(blow(BlowKind::Edge, BodyContactOutcome::Bite, 20, 300, 900), on());
        EXPECT_EQ(drawn.mKind, LodgeKind::Pass);
        EXPECT_FLOAT_EQ(drawn.mDragImpulse, 1.6f * 900.0f * 0.5f);
        const auto cleave = planLodge(blow(BlowKind::Edge, BodyContactOutcome::Cleave, 30, 1600, 300), on());
        EXPECT_EQ(cleave.mKind, LodgeKind::Pass);
        // The drag is capped.
        auto heavy = blow(BlowKind::Edge, BodyContactOutcome::Cleave, 30, 300, 2000);
        heavy.mCarriedMass = 8.0f;
        EXPECT_FLOAT_EQ(planLodge(heavy, on()).mDragImpulse, on().mPassDragCap);
    }

    TEST(RagdollCoreLodge, armourBluntAndStoppedEdgesKeepTheirOldWays)
    {
        EXPECT_EQ(planLodge(blow(BlowKind::Edge, BodyContactOutcome::Bite, 20, 800, 200, 0.6f), on()).mKind,
            LodgeKind::None);
        EXPECT_EQ(planLodge(blow(BlowKind::Edge, BodyContactOutcome::Stop, 20, 800, 200), on()).mKind, LodgeKind::None);
        EXPECT_EQ(planLodge(blow(BlowKind::Blunt, BodyContactOutcome::Stop, 20, 800, 200), on()).mKind,
            LodgeKind::None);
        EXPECT_EQ(planLodge(blow(BlowKind::Fist, BodyContactOutcome::Stop, 20, 800, 200), on()).mKind, LodgeKind::None);
    }

    LodgeStepInput pinned(JPH::Vec3 offset, JPH::Vec3 velocity, float age = 0.2f)
    {
        LodgeStepInput input;
        input.mOffset = offset;
        input.mAxis = JPH::Vec3::sAxisX();
        input.mRelativeVelocity = velocity;
        input.mEffectiveMass = 2.0f;
        input.mDepthMax = 10.0f;
        input.mAge = age;
        input.mHold = 1.5f;
        return input;
    }

    TEST(RagdollCoreLodge, thePinPullsBackOntoTheLineAndStopsSidewaysSpeed)
    {
        const auto step = stepLodge(pinned(JPH::Vec3(5, 2, 0), JPH::Vec3(0, 30, 0)), on());
        EXPECT_EQ(step.mRelease, LodgeRelease::Held);
        EXPECT_FLOAT_EQ(step.mDepth, 5.0f);
        EXPECT_FLOAT_EQ(step.mOffLine, 2.0f);
        EXPECT_LT(step.mImpulse.GetY(), 0.0f); // toward the line, against the speed
        EXPECT_FLOAT_EQ(step.mImpulse.GetX(), 0.0f); // at rest along it: no friction
        // On the line and still: nothing.
        EXPECT_TRUE(stepLodge(pinned(JPH::Vec3(5, 0, 0), JPH::Vec3::sZero()), on()).mImpulse.IsNearZero());
    }

    TEST(RagdollCoreLodge, frictionResistsSlidingAndGrowsWithDepth)
    {
        const auto shallow = stepLodge(pinned(JPH::Vec3(1, 0, 0), JPH::Vec3(-400, 0, 0)), on());
        const auto deep = stepLodge(pinned(JPH::Vec3(9, 0, 0), JPH::Vec3(-400, 0, 0)), on());
        EXPECT_GT(shallow.mImpulse.GetX(), 0.0f); // against the withdrawal
        EXPECT_GT(deep.mImpulse.GetX(), shallow.mImpulse.GetX());
        // Never more than stops the slide.
        const auto slow = stepLodge(pinned(JPH::Vec3(9, 0, 0), JPH::Vec3(-1, 0, 0)), on());
        EXPECT_FLOAT_EQ(slow.mImpulse.GetX(), 2.0f);
    }

    TEST(RagdollCoreLodge, itStopsAtTheDepthItEarned)
    {
        const auto past = stepLodge(pinned(JPH::Vec3(12, 0, 0), JPH::Vec3(300, 0, 0)), on());
        EXPECT_LT(past.mImpulse.GetX(), -2.0f * 300.0f + 1.0f); // the whole inward speed and some of the overshoot
    }

    TEST(RagdollCoreLodge, theForceIsCapped)
    {
        const auto step = stepLodge(pinned(JPH::Vec3(5, 11, 0), JPH::Vec3(0, 5000, 0)), on());
        EXPECT_NEAR(step.mImpulse.Length(), on().mPinForce / 60.0f, 0.5f);
    }

    TEST(RagdollCoreLodge, itLetsGoDrawnOutInTimeOrTorn)
    {
        EXPECT_EQ(stepLodge(pinned(JPH::Vec3(-2, 0, 0), JPH::Vec3::sZero()), on()).mRelease, LodgeRelease::Out);
        // Not in its first moments (it starts at the surface).
        EXPECT_EQ(stepLodge(pinned(JPH::Vec3(-2, 0, 0), JPH::Vec3::sZero(), 0.01f), on()).mRelease,
            LodgeRelease::Held);
        EXPECT_EQ(stepLodge(pinned(JPH::Vec3(5, 0, 0), JPH::Vec3::sZero(), 1.6f), on()).mRelease, LodgeRelease::Time);
        EXPECT_EQ(stepLodge(pinned(JPH::Vec3(5, 26, 0), JPH::Vec3::sZero()), on()).mRelease, LodgeRelease::Torn);
        auto broken = pinned(JPH::Vec3(5, 0, 0), JPH::Vec3::sZero());
        broken.mAxis = JPH::Vec3::sZero();
        EXPECT_EQ(stepLodge(broken, on()).mRelease, LodgeRelease::Torn);
    }
}
