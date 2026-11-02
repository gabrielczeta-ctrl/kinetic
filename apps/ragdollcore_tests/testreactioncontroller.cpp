#include <ragdollcore/reactioncontroller.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
    using namespace RagdollCore;

    constexpr float sDt = 1.0f / 60.0f;
    constexpr float sMass = 75.0f; // kg
    constexpr float sU = sReactionUnitsPerMetre;

    // A blow from the front (pushing the body back, -y; the body faces +y).
    Blow blowAt(AnatomicalRegion region, std::uint64_t id = 1, std::uint64_t attacker = 7)
    {
        Blow blow;
        blow.mId = id;
        blow.mAttacker = attacker;
        blow.mRegion = region;
        blow.mDirection = JPH::Vec3(0.0f, -1.0f, 0.0f);
        return blow;
    }

    // A disturbance with the COM dv and the part dv given in m/s.
    BlowDisturbance disturbance(float comDv, float partDv, JPH::Vec3 spin = JPH::Vec3::sZero())
    {
        BlowDisturbance d;
        d.mComVelocityChange = JPH::Vec3(0.0f, -comDv * sU, 0.0f);
        d.mPartVelocityChange = partDv * sU;
        d.mAngularMomentumChange = spin;
        return d;
    }

    ReactionStepOutput run(ReactionController& controller, float seconds, const ReactionVictim& victim = {})
    {
        ReactionStepOutput out;
        for (float t = 0.0f; t < seconds - 1e-6f; t += sDt)
            out = controller.step(sDt, victim);
        return out;
    }

    TEST(RagdollCoreReactionController, disturbanceIsTheMomentumChange)
    {
        const JPH::Vec3 impulse(0.0f, -150.0f * sMass, 0.0f);
        const auto d = blowDisturbance(impulse, JPH::Vec3(10.0f, 0.0f, 40.0f), sMass, 5.0f,
            JPH::Vec3(0.0f, 1.0f, 0.2f), JPH::Vec3::sAxisZ());
        EXPECT_NEAR(d.mComVelocityChange.GetY(), -150.0f, 1e-3f);
        EXPECT_NEAR(d.mPartVelocityChange, 150.0f * sMass / 5.0f, 1e-2f);
        const JPH::Vec3 spin = JPH::Vec3(10.0f, 0.0f, 40.0f).Cross(impulse);
        EXPECT_NEAR((d.mAngularMomentumChange - spin).Length(), 0.0f, 1e-2f);
        // The facing is flattened against the up axis.
        EXPECT_NEAR(d.mBodyForward.GetZ(), 0.0f, 1e-5f);
        EXPECT_NEAR(d.mBodyForward.Length(), 1.0f, 1e-5f);
    }

    TEST(RagdollCoreReactionController, tiersFollowTheComAndTheStruckPart)
    {
        ReactionController controller;
        EXPECT_EQ(controller.onBlow(blowAt(AnatomicalRegion::Chest, 1), disturbance(0.1f, 1.0f), {}).mTier,
            ReactionTier::Absorb);
        controller.reset();
        EXPECT_EQ(controller.onBlow(blowAt(AnatomicalRegion::Chest, 2), disturbance(0.5f, 2.0f), {}).mTier,
            ReactionTier::Flinch);
        controller.reset();
        EXPECT_EQ(controller.onBlow(blowAt(AnatomicalRegion::Chest, 3), disturbance(1.0f, 4.0f), {}).mTier,
            ReactionTier::Stagger);
        controller.reset();
        // A hand knocked hard flinches with no COM change; a head knocked
        // that hard staggers (a daze-strength blow).
        EXPECT_EQ(controller.onBlow(blowAt(AnatomicalRegion::RightHand, 4), disturbance(0.05f, 7.0f), {}).mTier,
            ReactionTier::Flinch);
        controller.reset();
        EXPECT_EQ(controller.onBlow(blowAt(AnatomicalRegion::Head, 5), disturbance(0.05f, 7.0f), {}).mTier,
            ReactionTier::Stagger);
    }

    TEST(RagdollCoreReactionController, aBlockedBlowIsAtMostAFlinch)
    {
        ReactionController controller;
        Blow blow = blowAt(AnatomicalRegion::Chest);
        blow.mBlocked = true;
        EXPECT_EQ(controller.onBlow(blow, disturbance(1.5f, 6.0f), {}).mTier, ReactionTier::Flinch);
    }

    TEST(RagdollCoreReactionController, balanceSeverityIsHarderBackwardThanSideways)
    {
        ReactionController controller;
        const auto back = controller.onBlow(blowAt(AnatomicalRegion::Chest, 1), disturbance(0.2f, 1.0f), {});
        BlowDisturbance side = disturbance(0.0f, 1.0f);
        side.mComVelocityChange = JPH::Vec3(0.2f * sU, 0.0f, 0.0f);
        const auto sideways = controller.onBlow(blowAt(AnatomicalRegion::Chest, 2), side, {});
        EXPECT_GT(back.mBalanceSeverity, 1.0f); // past the backward step threshold
        EXPECT_LT(sideways.mBalanceSeverity, 0.5f);
    }

    TEST(RagdollCoreReactionController, poiseLeaksAndRecovers)
    {
        ReactionController controller;
        float last = 1.0f;
        for (std::uint64_t i = 0; i < 6; ++i)
        {
            const auto event = controller.onBlow(blowAt(AnatomicalRegion::Chest, i + 1), disturbance(0.9f, 4.0f), {});
            EXPECT_LE(event.mScale, last + 1e-6f);
            last = event.mScale;
            controller.step(sDt, {});
        }
        EXPECT_LT(controller.leakage(), 0.4f);
        EXPECT_GE(controller.leakage(), controller.settings().mLeakageFloor - 1e-6f);
        run(controller, 2.0f);
        EXPECT_FLOAT_EQ(controller.leakage(), 1.0f);
        // Light touches do not drain it.
        for (std::uint64_t i = 0; i < 5; ++i)
            controller.onBlow(blowAt(AnatomicalRegion::Chest, 100 + i), disturbance(0.05f, 0.5f), {});
        EXPECT_FLOAT_EQ(controller.leakage(), 1.0f);
    }

    TEST(RagdollCoreReactionController, reflexesAreDebouncedUnlessTheTierRises)
    {
        ReactionController controller;
        EXPECT_TRUE(controller.onBlow(blowAt(AnatomicalRegion::Chest, 1), disturbance(0.5f, 2.0f), {}).mReflex);
        run(controller, 0.1f);
        EXPECT_FALSE(controller.onBlow(blowAt(AnatomicalRegion::Chest, 2), disturbance(0.5f, 2.0f), {}).mReflex);
        EXPECT_TRUE(controller.onBlow(blowAt(AnatomicalRegion::Chest, 3), disturbance(1.0f, 4.0f), {}).mReflex);
        run(controller, 0.5f);
        EXPECT_TRUE(controller.onBlow(blowAt(AnatomicalRegion::Chest, 4), disturbance(0.5f, 2.0f), {}).mReflex);
    }

    TEST(RagdollCoreReactionController, interruptsOnlyBreakWhatTheBlowBeats)
    {
        ReactionVictim winding;
        winding.mAttacking = true;
        winding.mWindShare = 0.3f;
        ReactionVictim committed = winding;
        committed.mCutting = true;
        ReactionVictim idle;

        ReactionController controller;
        EXPECT_FALSE(controller.onBlow(blowAt(AnatomicalRegion::Chest, 1), disturbance(0.5f, 2.0f), idle).mInterrupt);
        controller.reset();
        EXPECT_TRUE(controller.onBlow(blowAt(AnatomicalRegion::Chest, 2), disturbance(0.5f, 2.0f), winding).mInterrupt);
        EXPECT_TRUE(controller.step(sDt, winding).mInterrupt);
        EXPECT_FALSE(controller.step(sDt, winding).mInterrupt); // once
        controller.reset();
        // A committed cut carries through a flinch (hyper armour)...
        EXPECT_FALSE(controller.onBlow(blowAt(AnatomicalRegion::Chest, 3), disturbance(0.5f, 2.0f), committed).mInterrupt);
        // ...but not a stagger.
        EXPECT_TRUE(controller.onBlow(blowAt(AnatomicalRegion::Chest, 4), disturbance(1.0f, 4.0f), committed).mInterrupt);
        // The same attacker cannot stagger it again at once with a blow no stronger.
        EXPECT_FALSE(controller.onBlow(blowAt(AnatomicalRegion::Chest, 5), disturbance(1.0f, 4.0f), committed).mInterrupt);
    }

    TEST(RagdollCoreReactionController, reflexTimingFollowsHumanOnsets)
    {
        ReactionController controller;
        controller.onBlow(blowAt(AnatomicalRegion::Head), disturbance(0.6f, 6.0f), {});
        float peak = 0.0f, peakTime = 0.0f, firstMove = -1.0f, firstArms = -1.0f;
        for (int i = 0; i < 90; ++i)
        {
            const float t = i * sDt;
            const auto out = controller.step(sDt, {});
            const float head = out.mReflex.mHead.Length();
            if (head > 0.01f && firstMove < 0.0f)
                firstMove = t;
            if (out.mReflex.mArmsUp > 0.01f && firstArms < 0.0f)
                firstArms = t;
            if (head > peak)
            {
                peak = head;
                peakTime = t;
            }
        }
        EXPECT_GE(firstMove, 0.06f); // nothing voluntary before ~70 ms
        EXPECT_LE(firstMove, 0.12f);
        EXPECT_GE(firstArms, 0.1f);
        EXPECT_LE(firstArms, 0.2f);
        EXPECT_GT(peak, 0.2f);
        EXPECT_LT(peakTime, 0.4f);
        run(controller, 1.5f);
        EXPECT_EQ(controller.activeTracks(), 0u);
        EXPECT_LT(controller.step(sDt, {}).mReflex.mHead.Length(), 0.01f);
    }

    TEST(RagdollCoreReactionController, theHeadIsPushedTheWayOfTheBlow)
    {
        ReactionController controller;
        controller.onBlow(blowAt(AnatomicalRegion::Head), disturbance(0.6f, 6.0f), {});
        const auto out = run(controller, 0.25f);
        // Blow toward -y: the head tips back - a rotation about +x in the
        // body frame (x right, y forward, z up: up x back = +x, which turns
        // the top of the head toward -y).
        EXPECT_GT(out.mReflex.mHead.GetX(), 0.1f);
    }

    TEST(RagdollCoreReactionController, onlyTheStruckChainGivesAndItRecovers)
    {
        ReactionController controller;
        controller.onBlow(blowAt(AnatomicalRegion::RightUpperArm), disturbance(0.4f, 5.0f), {});
        auto out = controller.step(sDt, {});
        EXPECT_LT(out.mTorqueScale[static_cast<std::size_t>(BodyChannel::RightArm)], 0.5f);
        EXPECT_FLOAT_EQ(out.mTorqueScale[static_cast<std::size_t>(BodyChannel::LeftLeg)], 1.0f);
        EXPECT_FLOAT_EQ(out.mTorqueScale[static_cast<std::size_t>(BodyChannel::Trunk)], 1.0f);
        out = run(controller, 0.6f);
        EXPECT_NEAR(out.mTorqueScale[static_cast<std::size_t>(BodyChannel::RightArm)], 1.0f, 1e-3f);
    }

    TEST(RagdollCoreReactionController, aStruckLegWeakensAfterADelayAndBends)
    {
        ReactionController controller;
        controller.onBlow(blowAt(AnatomicalRegion::LeftThigh), disturbance(0.4f, 4.0f), {});
        float bend = 0.0f, at045 = 1.0f, at075 = 0.0f;
        for (int i = 0; i < 60; ++i)
        {
            const float t = i * sDt;
            const auto out = controller.step(sDt, {});
            const float left = out.mTorqueScale[static_cast<std::size_t>(BodyChannel::LeftLeg)];
            if (std::abs(t - 0.45f) < 0.5f * sDt)
                at045 = left;
            if (std::abs(t - 0.75f) < 0.5f * sDt)
                at075 = left;
            bend = std::max(bend, out.mReflex.mKneeBend[0]);
            EXPECT_FLOAT_EQ(out.mTorqueScale[static_cast<std::size_t>(BodyChannel::RightLeg)], 1.0f);
        }
        // The impact give has recovered by 0.45 s; the leg is weak then
        // (0.25-0.65 s) and strong again after.
        EXPECT_NEAR(at045, controller.settings().mLegWeakness, 1e-3f);
        EXPECT_NEAR(at075, 1.0f, 1e-3f);
        EXPECT_GT(bend, 0.05f);
    }

    // RXN-02 M5 leg give-out: injury (damage), not momentum.
    TEST(RagdollCoreReactionController, aDeepLegCutGivesTheLegOut)
    {
        ReactionSettings settings;
        settings.mLegInjuryDamage = 30.0f;
        ReactionController controller(settings);
        Blow blow = blowAt(AnatomicalRegion::RightCalf);
        blow.mDamage = 30.0f;
        const auto event = controller.onBlow(blow, disturbance(0.4f, 4.0f), {});
        EXPECT_FLOAT_EQ(event.mInjury, 1.0f);
        float weakest = 1.0f, giveOut = 0.0f, bend = 0.0f, at1 = 1.0f, at18 = 0.0f;
        for (int i = 0; i < 120; ++i)
        {
            const float t = i * sDt;
            const auto out = controller.step(sDt, {});
            const float right = out.mTorqueScale[static_cast<std::size_t>(BodyChannel::RightLeg)];
            weakest = std::min(weakest, right);
            giveOut = std::max(giveOut, out.mLegGiveOut[1]);
            bend = std::max(bend, out.mReflex.mKneeBend[1]);
            if (std::abs(t - 1.0f) < 0.5f * sDt)
                at1 = right;
            if (std::abs(t - 1.8f) < 0.5f * sDt)
                at18 = right;
            EXPECT_FLOAT_EQ(out.mLegGiveOut[0], 0.0f);
        }
        EXPECT_NEAR(weakest, settings.mLegGiveOutWeakness, 1e-3f);
        EXPECT_NEAR(giveOut, 1.0f, 1e-3f);
        EXPECT_GT(bend, 0.9f * settings.mLegGiveOutKnee);
        // Still weak a second in (an uninjured leg is back at 0.65 s); back by
        // the end of the window (0.25 + 1.2 s).
        EXPECT_LT(at1, 0.5f);
        EXPECT_NEAR(at18, 1.0f, 1e-3f);
    }

    TEST(RagdollCoreReactionController, legGiveOutScalesWithDamageAndIgnoresBlocksAndOtherParts)
    {
        ReactionSettings settings;
        settings.mLegInjuryDamage = 30.0f;
        const auto peak = [&](AnatomicalRegion region, float damage, bool blocked) {
            ReactionController controller(settings);
            Blow blow = blowAt(region);
            blow.mDamage = damage;
            blow.mBlocked = blocked;
            controller.onBlow(blow, disturbance(0.4f, 4.0f), {});
            float giveOut = 0.0f;
            for (int i = 0; i < 120; ++i)
            {
                const auto out = controller.step(sDt, {});
                giveOut = std::max({ giveOut, out.mLegGiveOut[0], out.mLegGiveOut[1] });
            }
            return giveOut;
        };
        EXPECT_NEAR(peak(AnatomicalRegion::LeftThigh, 15.0f, false), 0.5f, 1e-3f);
        EXPECT_FLOAT_EQ(peak(AnatomicalRegion::LeftThigh, 30.0f, true), 0.0f);
        EXPECT_FLOAT_EQ(peak(AnatomicalRegion::Chest, 30.0f, false), 0.0f);
        // Off by default.
        settings.mLegInjuryDamage = 0.0f;
        EXPECT_FLOAT_EQ(peak(AnatomicalRegion::LeftThigh, 30.0f, false), 0.0f);
    }

    // Bearings (user 10-07): a stagger's recovery window, longer when tired
    // or hurt, longer for a blunt blow; a tired body staggers sooner.
    TEST(RagdollCoreReactionController, aStaggerTakesBearingsLongerWhenTiredOrHurt)
    {
        ReactionSettings settings;
        settings.mBearingsBase = 0.5f;
        const auto window = [&](float fatigue, float health, BlowKind kind) {
            ReactionController controller(settings);
            ReactionVictim victim;
            victim.mFatigue = fatigue;
            victim.mHealth = health;
            Blow blow = blowAt(AnatomicalRegion::Chest);
            blow.mKind = kind;
            controller.onBlow(blow, disturbance(1.0f, 4.0f), victim);
            const auto out = controller.step(sDt, victim);
            EXPECT_EQ(out.mWeaponArmBusy, out.mBearings > 0.0f);
            if (out.mBearings > 0.0f)
                EXPECT_LT(out.mTorqueScale[static_cast<std::size_t>(BodyChannel::LeftLeg)], 0.9f);
            return out.mBearings;
        };
        const float fresh = window(1.0f, 1.0f, BlowKind::Edge);
        EXPECT_GT(fresh, 0.5f);
        EXPECT_GT(window(0.2f, 1.0f, BlowKind::Edge), 1.5f * fresh);
        EXPECT_GT(window(1.0f, 0.3f, BlowKind::Edge), 1.2f * fresh);
        EXPECT_GT(window(1.0f, 1.0f, BlowKind::Blunt), 1.2f * fresh);
        EXPECT_LE(window(0.0f, 0.0f, BlowKind::Blunt), settings.mBearingsMax + 1e-4f);
        // It runs out.
        ReactionController controller(settings);
        controller.onBlow(blowAt(AnatomicalRegion::Chest), disturbance(1.0f, 4.0f), {});
        const auto later = run(controller, 3.0f);
        EXPECT_FLOAT_EQ(later.mBearings, 0.0f);
        EXPECT_FALSE(later.mWeaponArmBusy);
    }

    TEST(RagdollCoreReactionController, aTiredBodyStaggersSoonerAndBearingsAreOffByDefault)
    {
        // 0.6 m/s at the COM: a flinch for a fresh body, a stagger for a spent one.
        ReactionSettings settings;
        settings.mBearingsBase = 0.5f;
        ReactionController fresh(settings);
        EXPECT_EQ(fresh.onBlow(blowAt(AnatomicalRegion::Chest), disturbance(0.6f, 2.0f), {}).mTier,
            ReactionTier::Flinch);
        ReactionController spent(settings);
        ReactionVictim tired;
        tired.mFatigue = 0.0f;
        EXPECT_EQ(spent.onBlow(blowAt(AnatomicalRegion::Chest), disturbance(0.6f, 2.0f), tired).mTier,
            ReactionTier::Stagger);
        ReactionController off;
        EXPECT_EQ(off.onBlow(blowAt(AnatomicalRegion::Chest), disturbance(0.6f, 2.0f), tired).mTier,
            ReactionTier::Flinch);
        off.onBlow(blowAt(AnatomicalRegion::Chest, 2), disturbance(1.0f, 4.0f), {});
        EXPECT_FLOAT_EQ(off.step(sDt, {}).mBearings, 0.0f);
    }

    TEST(RagdollCoreReactionController, aBeatenGuardStaggersAHeldOneFlinches)
    {
        ReactionController held;
        Blow blocked = blowAt(AnatomicalRegion::RightForearm);
        blocked.mBlocked = true;
        EXPECT_EQ(held.onBlow(blocked, disturbance(1.6f, 4.0f), {}).mTier, ReactionTier::Flinch);
        ReactionController beaten;
        blocked.mGuardBeaten = true;
        EXPECT_EQ(beaten.onBlow(blocked, disturbance(1.6f, 4.0f), {}).mTier, ReactionTier::Stagger);
    }

    TEST(RagdollCoreReactionController, snapThenReverseSnapOnHeavyBlowsOnly)
    {
        ReactionController controller;
        controller.onBlow(blowAt(AnatomicalRegion::Head), disturbance(0.9f, 9.0f), {});
        const auto first = controller.step(sDt, {});
        EXPECT_GT(first.mHeadSnap.Length(), 5.0f);
        EXPECT_LE(first.mHeadSnap.Length(), controller.settings().mHeadSpinCap + 1e-3f);
        JPH::Vec3 reverse = JPH::Vec3::sZero();
        for (int i = 0; i < 10; ++i)
            reverse += controller.step(sDt, {}).mHeadSnap;
        EXPECT_LT(reverse.Dot(first.mHeadSnap), 0.0f);
        EXPECT_NEAR(reverse.Length(), first.mHeadSnap.Length() * controller.settings().mUnsnapRatio, 0.5f);

        controller.reset();
        controller.onBlow(blowAt(AnatomicalRegion::Head, 2), disturbance(0.1f, 1.5f), {});
        controller.step(sDt, {});
        JPH::Vec3 none = JPH::Vec3::sZero();
        for (int i = 0; i < 10; ++i)
            none += controller.step(sDt, {}).mHeadSnap;
        EXPECT_LT(none.Length(), 1e-4f);
    }

    TEST(RagdollCoreReactionController, aDownedBodyGetsNoReflexOrSnap)
    {
        ReactionController controller;
        ReactionVictim down;
        down.mDown = true;
        const auto event = controller.onBlow(blowAt(AnatomicalRegion::Chest), disturbance(1.0f, 5.0f), down);
        EXPECT_FALSE(event.mReflex);
        const auto out = controller.step(sDt, down);
        EXPECT_LT(out.mTrunkSnap.Length(), 1e-6f);
        EXPECT_FALSE(run(controller, 0.3f, down).mReflex.any());
    }

    TEST(RagdollCoreReactionController, aStaggerTakesTheWeaponArmForAWhile)
    {
        ReactionController controller;
        controller.onBlow(blowAt(AnatomicalRegion::Chest), disturbance(1.2f, 5.0f), {});
        bool busy = false;
        for (int i = 0; i < 30; ++i)
            busy = busy || controller.step(sDt, {}).mWeaponArmBusy;
        EXPECT_TRUE(busy);
        EXPECT_FALSE(run(controller, 1.5f).mWeaponArmBusy);
    }

    TEST(RagdollCoreReactionController, nonFiniteInputIsHarmless)
    {
        ReactionController controller;
        BlowDisturbance bad;
        bad.mComVelocityChange = JPH::Vec3(std::nanf(""), 0.0f, 0.0f);
        bad.mPartVelocityChange = std::nanf("");
        bad.mAngularMomentumChange = JPH::Vec3(0.0f, std::numeric_limits<float>::infinity(), 0.0f);
        Blow blow = blowAt(AnatomicalRegion::Chest);
        blow.mDirection = JPH::Vec3(std::nanf(""), 0.0f, 0.0f);
        const auto event = controller.onBlow(blow, bad, {});
        EXPECT_EQ(event.mTier, ReactionTier::Absorb);
        const auto out = controller.step(std::nanf(""), {});
        EXPECT_TRUE(std::isfinite(out.mHeadSnap.Length()));
        EXPECT_TRUE(std::isfinite(out.mReflex.mTrunk.Length()));
    }
}
