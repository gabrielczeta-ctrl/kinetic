#include <ragdollcore/contactoutcome.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
    using namespace RagdollCore;

    BodyContactInput cut(float speed, float mass, float hardness = 0.0f, bool soft = true)
    {
        BodyContactInput input;
        input.mKind = BlowKind::Edge;
        input.mNormalSpeed = speed;
        input.mTangentSpeed = speed * 0.3f;
        input.mEffectiveMass = mass;
        input.mHardness = hardness;
        input.mSoftPart = soft;
        return input;
    }

    // Weapons: a longsword brings about 1.6 kg, a claymore 2.4 kg (the
    // carried mass the strike loop uses); light/solid/heavy cuts at
    // 300/800/1600 u/s.
    TEST(RagdollCoreContactOutcome, unarmouredFleshIsBittenOrCleaved)
    {
        EXPECT_EQ(classifyBodyContact(cut(300, 1.6f)).mOutcome, BodyContactOutcome::Bite);
        EXPECT_EQ(classifyBodyContact(cut(800, 1.6f, 0.0f, false)).mOutcome, BodyContactOutcome::Bite);
        const auto cleave = classifyBodyContact(cut(1600, 2.4f));
        EXPECT_EQ(cleave.mOutcome, BodyContactOutcome::Cleave);
        EXPECT_GT(cleave.mSpeedAfter, 0.0f);
        EXPECT_LT(cleave.mSpeedAfter, 1600.0f);
        // The trunk is not cleaved: the same blow bites.
        EXPECT_EQ(classifyBodyContact(cut(1600, 2.4f, 0.0f, false)).mOutcome, BodyContactOutcome::Bite);
    }

    TEST(RagdollCoreContactOutcome, aBiteHoldsLongerTheHarderItWent)
    {
        const auto light = classifyBodyContact(cut(300, 1.6f, 0.0f, false));
        const auto solid = classifyBodyContact(cut(800, 1.6f, 0.0f, false));
        EXPECT_GE(light.mHoldTime, 0.04f);
        EXPECT_LE(solid.mHoldTime, 0.08f);
        EXPECT_GT(solid.mHoldTime, light.mHoldTime);
    }

    TEST(RagdollCoreContactOutcome, armourTurnsCutsIntoStops)
    {
        // Mail stops a solid longsword cut (blunt trauma) but not a heavy
        // claymore cut; plate stops both.
        EXPECT_EQ(classifyBodyContact(cut(800, 1.6f, 0.6f)).mOutcome, BodyContactOutcome::Stop);
        EXPECT_EQ(classifyBodyContact(cut(1600, 2.4f, 0.6f)).mOutcome, BodyContactOutcome::Bite);
        EXPECT_EQ(classifyBodyContact(cut(1600, 2.4f, 0.9f)).mOutcome, BodyContactOutcome::Stop);
    }

    TEST(RagdollCoreContactOutcome, anEdgeStoppedByArmourSticksWithTheSetting)
    {
        // K2_EDGE_BITE: mail stops a solid cut; the edge holds in it.
        EXPECT_FLOAT_EQ(classifyBodyContact(cut(800, 1.6f, 0.6f)).mHoldTime, 0.0f);
        BodyContactSettings settings;
        settings.mEdgeStick = 0.1f;
        const auto stuck = classifyBodyContact(cut(800, 1.6f, 0.6f), settings);
        EXPECT_EQ(stuck.mOutcome, BodyContactOutcome::Stop);
        EXPECT_FLOAT_EQ(stuck.mHoldTime, 0.1f);
        // Not the flat, not a blunt head.
        BodyContactInput flat = cut(800, 1.6f, 0.2f);
        flat.mEdgeAlignment = 0.2f;
        EXPECT_FLOAT_EQ(classifyBodyContact(flat, settings).mHoldTime, 0.0f);
        BodyContactInput blunt = cut(800, 1.6f, 0.6f);
        blunt.mKind = BlowKind::Blunt;
        EXPECT_FLOAT_EQ(classifyBodyContact(blunt, settings).mHoldTime, 0.0f);
    }

    TEST(RagdollCoreContactOutcome, shallowAndFlatBlowsGlance)
    {
        // A shallow edge on flesh is a draw cut; on mail it glances.
        BodyContactInput shallow = cut(300, 1.6f);
        shallow.mTangentSpeed = 900.0f;
        EXPECT_EQ(classifyBodyContact(shallow).mOutcome, BodyContactOutcome::Bite);
        shallow.mHardness = 0.6f;
        const auto result = classifyBodyContact(shallow);
        EXPECT_EQ(result.mOutcome, BodyContactOutcome::Glance);
        EXPECT_FLOAT_EQ(result.mSpeedAfter, 900.0f);
        EXPECT_LT(result.mDamageScale, 1.0f);
        // A shallow point skids even on flesh.
        BodyContactInput point = cut(200, 1.6f);
        point.mKind = BlowKind::Point;
        point.mTangentSpeed = 900.0f;
        EXPECT_EQ(classifyBodyContact(point).mOutcome, BodyContactOutcome::Glance);
        BodyContactInput flat = cut(800, 1.6f, 0.6f);
        flat.mEdgeAlignment = 0.2f;
        EXPECT_EQ(classifyBodyContact(flat).mOutcome, BodyContactOutcome::Glance);
        // The flat on flesh bruises: it stops at half price.
        flat.mHardness = 0.0f;
        const auto bruise = classifyBodyContact(flat);
        EXPECT_EQ(bruise.mOutcome, BodyContactOutcome::Stop);
        EXPECT_FLOAT_EQ(bruise.mDamageScale, 0.5f);
        // A touch far under the cut energy is a nick.
        EXPECT_EQ(classifyBodyContact(cut(60, 1.6f)).mOutcome, BodyContactOutcome::Glance);
    }

    TEST(RagdollCoreContactOutcome, bluntBlowsStopAndPointsGoInEasier)
    {
        BodyContactInput hammer = cut(800, 3.0f, 0.9f);
        hammer.mKind = BlowKind::Blunt;
        EXPECT_EQ(classifyBodyContact(hammer).mOutcome, BodyContactOutcome::Stop);
        hammer.mTangentSpeed = 2000.0f;
        EXPECT_EQ(classifyBodyContact(hammer).mOutcome, BodyContactOutcome::Glance);
        BodyContactInput thrust = cut(700, 1.6f, 0.3f, false);
        BodyContactInput slash = thrust;
        thrust.mKind = BlowKind::Point;
        EXPECT_LT(classifyBodyContact(thrust).mCutEnergy, classifyBodyContact(slash).mCutEnergy);
        EXPECT_EQ(classifyBodyContact(thrust).mOutcome, BodyContactOutcome::Bite);
    }

    TEST(RagdollCoreContactOutcome, clashTableFollowsPowerAngleAndTiming)
    {
        ClashInput input;
        input.mGuardFirmness = 100.0f;
        input.mAttackMomentum = 100.0f;
        input.mBladeAlignment = 0.1f;
        EXPECT_EQ(classifyClash(input).mOutcome, ClashOutcome::Bounce);
        input.mBladeAlignment = 0.6f;
        const auto deflect = classifyClash(input);
        EXPECT_EQ(deflect.mOutcome, ClashOutcome::Deflect);
        EXPECT_GT(deflect.mAttackContinues, 0.5f);
        input.mAttackMomentum = 200.0f;
        const auto broken = classifyClash(input);
        EXPECT_EQ(broken.mOutcome, ClashOutcome::BrokenGuard);
        EXPECT_GT(broken.mDefenderLockout, 0.0f);
        EXPECT_GT(broken.mAttackContinues, 0.0f);
        input.mAttackMomentum = 50.0f;
        EXPECT_EQ(classifyClash(input).mOutcome, ClashOutcome::Deflect); // a passive guard only deflects
        input.mActiveParry = true;
        const auto knock = classifyClash(input);
        EXPECT_EQ(knock.mOutcome, ClashOutcome::Knockaway);
        EXPECT_GT(knock.mAttackerLockout, 0.0f);
    }

    TEST(RagdollCoreContactOutcome, disarmsNeedAClearMarginAndAWeakGrip)
    {
        ClashInput input;
        input.mGuardFirmness = 100.0f;
        input.mAttackMomentum = 400.0f;
        EXPECT_EQ(classifyClash(input).mOutcome, ClashOutcome::BrokenGuard);
        input.mDefenderGrip = 0.3f;
        const auto result = classifyClash(input);
        EXPECT_EQ(result.mOutcome, ClashOutcome::Disarm);
        EXPECT_TRUE(result.mDefenderDisarmed);
        input = {};
        input.mGuardFirmness = 400.0f;
        input.mAttackMomentum = 100.0f;
        input.mActiveParry = true;
        input.mAttackerGrip = 0.3f;
        EXPECT_TRUE(classifyClash(input).mAttackerDisarmed);
    }

    TEST(RagdollCoreContactOutcome, nonFiniteInputIsHarmless)
    {
        BodyContactInput input = cut(std::nanf(""), std::nanf(""), std::nanf(""));
        input.mTangentSpeed = std::numeric_limits<float>::infinity();
        const auto result = classifyBodyContact(input);
        EXPECT_TRUE(std::isfinite(result.mEnergy));
        ClashInput clash;
        clash.mAttackMomentum = std::nanf("");
        clash.mGuardFirmness = std::nanf("");
        EXPECT_TRUE(std::isfinite(classifyClash(clash).mRatio));
    }

    // K2_BLOW_PRESS: flesh takes the blade as a shove, plate still rebounds.
    TEST(RagdollCoreContactOutcome, pressKeepsTheBladeOnFlesh)
    {
        BlowPressSettings press;
        press.mOn = true;
        const ImpactHold before{ 0.1f, 300.0f };
        for (const auto outcome :
            { BodyContactOutcome::Bite, BodyContactOutcome::Cleave, BodyContactOutcome::Glance })
        {
            const ImpactHold held = pressImpact(outcome, 0.0f, 0.05f, before, press);
            EXPECT_EQ(held.mFreeze, 0.0f);
            EXPECT_EQ(held.mRecoil, 0.0f);
        }
        const ImpactHold flesh = pressImpact(BodyContactOutcome::Stop, 0.2f, 0.0f, before, press);
        EXPECT_EQ(flesh.mFreeze, 0.0f);
        EXPECT_EQ(flesh.mRecoil, 0.0f);
        const ImpactHold plate = pressImpact(BodyContactOutcome::Stop, 0.9f, 0.0f, before, press);
        EXPECT_EQ(plate.mFreeze, before.mFreeze);
        EXPECT_EQ(plate.mRecoil, before.mRecoil);
        const ImpactHold stuck = pressImpact(BodyContactOutcome::Stop, 0.9f, 0.1f, before, press);
        EXPECT_EQ(stuck.mFreeze, 0.1f);
        EXPECT_EQ(stuck.mRecoil, 0.0f);
        press.mOn = false;
        const ImpactHold off = pressImpact(BodyContactOutcome::Bite, 0.0f, 0.05f, before, press);
        EXPECT_EQ(off.mFreeze, before.mFreeze);
        EXPECT_EQ(off.mRecoil, before.mRecoil);
    }

    TEST(RagdollCoreContactOutcome, pressCarriesTheBodyMovingAlongTheBlow)
    {
        BlowPressSettings press;
        press.mOn = true;
        press.mBodyShare = 0.15f;
        press.mMassCap = 8.0f;
        // Standing: the weapon alone.
        EXPECT_FLOAT_EQ(pressCarriedMass(2.0f, 75.0f, 0.0f, 900.0f, press), 2.0f);
        // Moving away from the blow: never less than the weapon.
        EXPECT_FLOAT_EQ(pressCarriedMass(2.0f, 75.0f, -300.0f, 900.0f, press), 2.0f);
        // A jumping chop, a third of the closing speed the body's: + 0.15 x 75 / 3.
        EXPECT_NEAR(pressCarriedMass(2.0f, 75.0f, 300.0f, 900.0f, press), 2.0f + 0.15f * 75.0f / 3.0f, 1e-4f);
        // Capped.
        EXPECT_FLOAT_EQ(pressCarriedMass(2.0f, 75.0f, 900.0f, 900.0f, press), 8.0f);
        EXPECT_TRUE(std::isfinite(pressCarriedMass(std::nanf(""), 75.0f, std::nanf(""), 0.0f, press)));
        press.mOn = false;
        EXPECT_FLOAT_EQ(pressCarriedMass(2.0f, 75.0f, 900.0f, 900.0f, press), 2.0f);
    }

    // K2_PUSH_KICK: Strength drives the foot, Hand-to-hand couples the body,
    // the run-in adds; off is KICK-01's push.
    TEST(RagdollCoreContactOutcome, pushKickGrowsWithStrengthSkillAndRunIn)
    {
        PushKickInput in;
        in.mStrength = 50.0f;
        in.mHandToHand = 0.0f;
        in.mFootSpeed = 350.0f;
        PushKickSettings off;
        const PushKick old = pushKick(in, off);
        EXPECT_FLOAT_EQ(old.mSpeed, 240.0f);
        EXPECT_FLOAT_EQ(old.mMass, 14.0f);
        in.mStrength = 200.0f;
        EXPECT_FLOAT_EQ(pushKick(in, off).mSpeed, 240.0f * 1.4f); // the old cap
        PushKickSettings on;
        on.mOn = true;
        const PushKick strong = pushKick(in, on);
        EXPECT_FLOAT_EQ(strong.mSpeed, 240.0f * 4.0f);
        EXPECT_FLOAT_EQ(strong.mMass, 14.0f + 75.0f * 0.1f);
        in.mHandToHand = 100.0f;
        in.mKickerAlong = 200.0f;
        const PushKick master = pushKick(in, on);
        EXPECT_FLOAT_EQ(master.mMass, 14.0f + 75.0f * 0.35f);
        EXPECT_FLOAT_EQ(master.mSpeed, 240.0f * 4.0f + 200.0f);
        in.mGuarded = true;
        EXPECT_FLOAT_EQ(pushKick(in, on).mSpeed, master.mSpeed * 0.8f);
        in.mStrength = std::nanf("");
        EXPECT_TRUE(std::isfinite(pushKick(in, on).mSpeed));
    }

    // K2_PUSH_KICK's carry: the whole body takes a share of the push's
    // momentum, up to a cap; the kicker takes a share of that back.
    TEST(RagdollCoreContactOutcome, pushKickThrowsTheWholeBodyUpToItsCap)
    {
        PushKickInput in;
        in.mStrength = 50.0f;
        in.mHandToHand = 0.0f;
        in.mFootSpeed = 350.0f;
        PushKickSettings on;
        on.mOn = true;
        EXPECT_FLOAT_EQ(pushKick(in, on).mCarry, 0.0f);
        on.mCarry = 1.0f;
        on.mCarryCap = 1000.0f;
        on.mRecoil = 0.25f;
        const PushKick kick = pushKick(in, on);
        EXPECT_FLOAT_EQ(kick.mCarry, kick.mSpeed * kick.mMass / 75.0f);
        EXPECT_FLOAT_EQ(kick.mRecoil, 0.25f * kick.mCarry);
        in.mVictimMass = 150.0f;
        EXPECT_FLOAT_EQ(pushKick(in, on).mCarry, 0.5f * kick.mCarry);
        // It fells from a set speed, never through a guard.
        in.mVictimMass = 75.0f;
        on.mFell = kick.mCarry - 1.0f;
        EXPECT_TRUE(pushKick(in, on).mFells);
        on.mFell = kick.mCarry + 1.0f;
        EXPECT_FALSE(pushKick(in, on).mFells);
        on.mFell = 1.0f;
        in.mGuarded = true;
        EXPECT_FALSE(pushKick(in, on).mFells);
        in.mGuarded = false;
        on.mCarryCap = 10.0f;
        EXPECT_FLOAT_EQ(pushKick(in, on).mCarry, 10.0f);
        on.mOn = false;
        EXPECT_FLOAT_EQ(pushKick(in, on).mCarry, 0.0f);
        // The feet take the least of a carry, the trunk all of it.
        EXPECT_FLOAT_EQ(bodyCarryShare(AnatomicalRegion::Chest), 1.0f);
        EXPECT_LT(bodyCarryShare(AnatomicalRegion::LeftFoot), bodyCarryShare(AnatomicalRegion::LeftCalf));
        EXPECT_LT(bodyCarryShare(AnatomicalRegion::RightCalf), bodyCarryShare(AnatomicalRegion::RightThigh));
        EXPECT_LT(bodyCarryShare(AnatomicalRegion::RightThigh), 1.0f);
    }
}
