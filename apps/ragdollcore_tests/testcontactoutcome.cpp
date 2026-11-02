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
}
