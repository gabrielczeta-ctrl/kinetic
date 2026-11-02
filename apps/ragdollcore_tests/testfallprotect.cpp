#include <ragdollcore/fallprotect.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
    using namespace RagdollCore;

    constexpr float sDt = 1.0f / 60.0f;

    FallProtectInput falling(FallVec3 velocity)
    {
        FallProtectInput input;
        input.mDt = sDt;
        input.mComVelocity = velocity;
        input.mTorsoUp = { 0.0f, 0.0f, 1.0f };
        input.mBodyForward = { 0.0f, 1.0f, 0.0f };
        return input;
    }

    FallProtectCommand run(FallProtectController& controller, const FallProtectInput& input, float seconds)
    {
        FallProtectCommand command;
        for (float t = 0.0f; t < seconds - 1e-6f; t += input.mDt)
            command = controller.update(input);
        return command;
    }

    TEST(RagdollCoreFallProtect, classifiesByBodyForward)
    {
        const FallVec3 forward{ 0.0f, 1.0f, 0.0f };
        EXPECT_EQ(classifyFall({ 0.0f, 1.0f, 0.0f }, forward), FallKind::Forward);
        EXPECT_EQ(classifyFall({ 0.0f, -1.0f, 0.0f }, forward), FallKind::Backward);
        EXPECT_EQ(classifyFall({ 1.0f, 0.2f, 0.0f }, forward), FallKind::Side);
    }

    TEST(RagdollCoreFallProtect, reactsAfterTheDelayThenBraces)
    {
        FallProtectController controller;
        FallProtectInput input = falling({ 0.0f, 120.0f, -50.0f });
        input.mSettings.mReactionDelay = 0.1f;
        const FallProtectCommand early = run(controller, input, 0.05f);
        EXPECT_EQ(early.mPhase, FallPhase::Reacting);
        EXPECT_EQ(early.mArmWeight[0], 0.0f);
        const FallProtectCommand braced = run(controller, input, 0.3f);
        EXPECT_EQ(braced.mPhase, FallPhase::Bracing);
        EXPECT_FLOAT_EQ(braced.mArmWeight[0], 1.0f);
        EXPECT_FLOAT_EQ(braced.mArmWeight[1], 1.0f);
        EXPECT_LT(braced.mLegStiffness, braced.mArmStiffness);
    }

    TEST(RagdollCoreFallProtect, armsReachAlongTheFallAndDown)
    {
        FallProtectController controller;
        const FallProtectCommand command = run(controller, falling({ 0.0f, 150.0f, 0.0f }), 0.3f);
        EXPECT_EQ(command.mKind, FallKind::Forward);
        EXPECT_GT(command.mArmDirection.mY, 0.5f);
        EXPECT_LT(command.mArmDirection.mZ, -0.3f);
        const float length = std::sqrt(command.mArmDirection.mX * command.mArmDirection.mX
            + command.mArmDirection.mY * command.mArmDirection.mY
            + command.mArmDirection.mZ * command.mArmDirection.mZ);
        EXPECT_NEAR(length, 1.0f, 1e-4f);
        EXPECT_LT(command.mHeadPitch, 0.0f); // face up, away from the ground
    }

    TEST(RagdollCoreFallProtect, backwardFallTucksTheChin)
    {
        FallProtectController controller;
        const FallProtectCommand command = run(controller, falling({ 0.0f, -150.0f, 0.0f }), 0.3f);
        EXPECT_EQ(command.mKind, FallKind::Backward);
        EXPECT_GT(command.mHeadPitch, 0.5f);
        EXPECT_GT(command.mSpineCurl, 0.2f);
        EXPECT_LT(command.mArmDirection.mY, 0.0f);
    }

    TEST(RagdollCoreFallProtect, sideFallLeadsWithTheNearArm)
    {
        FallProtectController controller;
        // Falling to the body's right (+x when facing +y): the right arm leads.
        const FallProtectCommand command = run(controller, falling({ 150.0f, 0.0f, 0.0f }), 0.3f);
        EXPECT_EQ(command.mKind, FallKind::Side);
        EXPECT_FLOAT_EQ(command.mArmWeight[1], 1.0f);
        EXPECT_LT(command.mArmWeight[0], 1.0f);
    }

    TEST(RagdollCoreFallProtect, slowBodyTakesTheTorsoTilt)
    {
        FallProtectController controller;
        FallProtectInput input = falling({ 0.0f, 0.0f, 0.0f });
        input.mTorsoUp = { -0.5f, 0.0f, 0.866f };
        const FallProtectCommand command = run(controller, input, 0.2f);
        EXPECT_LT(command.mDirection.mX, -0.99f);
    }

    TEST(RagdollCoreFallProtect, contactYieldsThenSettles)
    {
        FallProtectController controller;
        FallProtectInput input = falling({ 0.0f, 150.0f, -100.0f });
        run(controller, input, 0.3f);
        input.mNonFootContact = true;
        const FallProtectCommand hit = controller.update(input);
        EXPECT_EQ(hit.mPhase, FallPhase::Impact);
        const float braceArms = hit.mArmStiffness;
        const FallProtectCommand absorbing = run(controller, input, 0.25f);
        EXPECT_LT(absorbing.mArmStiffness, braceArms);
        // The direction is frozen once something touched.
        input.mComVelocity = { 150.0f, 0.0f, 0.0f };
        EXPECT_GT(controller.update(input).mDirection.mY, 0.99f);
        input.mComVelocity = { 0.0f, 0.0f, 0.0f };
        const FallProtectCommand settled = run(controller, input, 0.5f);
        EXPECT_EQ(settled.mPhase, FallPhase::Settled);
        EXPECT_FLOAT_EQ(settled.mArmStiffness, input.mSettings.mSettleTone);
    }

    TEST(RagdollCoreFallProtect, longFallWithoutContactStillYields)
    {
        FallProtectController controller;
        FallProtectInput input = falling({ 0.0f, 50.0f, -300.0f });
        const FallProtectCommand command = run(controller, input, input.mSettings.mMaximumFallTime + 0.05f);
        EXPECT_EQ(command.mPhase, FallPhase::Impact);
    }

    TEST(RagdollCoreFallProtect, ignoresBadTimeSteps)
    {
        FallProtectController controller;
        FallProtectInput input = falling({ 0.0f, 150.0f, 0.0f });
        input.mDt = std::nanf("");
        const FallProtectCommand command = controller.update(input);
        EXPECT_EQ(command.mPhase, FallPhase::Reacting);
        EXPECT_EQ(command.mTime, 0.0f);
    }

    TEST(RagdollCoreFallPredict, quietTiltIsNoRisk)
    {
        FallPredictor predictor;
        float risk = 0.0f;
        for (int i = 0; i < 60; ++i)
            risk = predictor.update(std::cos(0.15f), 1.0f / 60.0f);
        EXPECT_EQ(risk, 0.0f);
        EXPECT_FALSE(predictor.alert());
    }

    TEST(RagdollCoreFallPredict, fastTiltWarnsBeforeTheTopple)
    {
        FallPredictor predictor;
        float risk = 0.0f;
        float tilt = 0.0f;
        int warnedAt = -1;
        for (int i = 0; i < 60 && tilt < 1.047f; ++i) // 2 rad/s toward 60 degrees
        {
            tilt += 2.0f / 60.0f;
            risk = predictor.update(std::cos(tilt), 1.0f / 60.0f);
            if (risk > 0.0f && warnedAt < 0)
                warnedAt = i;
        }
        EXPECT_GE(warnedAt, 0);
        EXPECT_LT(warnedAt * (2.0f / 60.0f), 0.8f); // warned while still under ~45 degrees
        EXPECT_FLOAT_EQ(risk, 1.0f);
        EXPECT_TRUE(predictor.alert());
    }

    TEST(RagdollCoreFallPredict, alertClearsAfterRecovery)
    {
        FallPredictor predictor;
        for (int i = 0; i < 20; ++i)
            predictor.update(std::cos(0.9f), 1.0f / 60.0f);
        EXPECT_TRUE(predictor.alert());
        for (int i = 0; i < 30; ++i)
            predictor.update(1.0f, 1.0f / 60.0f);
        EXPECT_FALSE(predictor.alert());
    }

    TEST(RagdollCoreFallProtect, alertMovesTheArmsOnlyAndKeepsTheClock)
    {
        FallProtectController controller;
        FallProtectInput input;
        input.mDt = 1.0f / 60.0f;
        input.mComVelocity = { 0.0f, -150.0f, 0.0f };
        input.mBodyForward = { 0.0f, 1.0f, 0.0f };
        input.mTorsoUp = { 0.0f, 0.0f, 1.0f };
        input.mFallen = false;
        input.mRisk = 0.5f;
        input.mNonFootContact = true; // ignored while up
        FallProtectCommand command;
        for (int i = 0; i < 30; ++i)
            command = controller.update(input);
        EXPECT_EQ(command.mPhase, FallPhase::Bracing);
        EXPECT_FALSE(command.mFallen);
        EXPECT_FLOAT_EQ(command.mArmWeight[0], 0.5f);
        EXPECT_EQ(command.mHeadPitch, 0.0f);
        EXPECT_EQ(command.mArmStiffness, 1.0f);
        input.mFallen = true;
        input.mNonFootContact = false;
        command = controller.update(input);
        EXPECT_EQ(command.mPhase, FallPhase::Bracing); // braced already, no new delay
        EXPECT_FLOAT_EQ(command.mArmWeight[0], 1.0f);
        EXPECT_GT(command.mHeadPitch, 0.0f);
    }

    // CATCH-02: off by default, each arm's command is the shared one.
    TEST(RagdollCoreFallProtect, catchUpgradesOffKeepTheSharedArm)
    {
        FallProtectController controller;
        FallProtectInput input = falling({ 0.0f, 150.0f, -50.0f });
        input.mArms[0].mHasProbe = true;
        input.mArms[0].mProbeHit = { -100.0f, 0.0f, 0.0f };
        input.mArms[0].mTouching = true;
        input.mSeed = 77;
        const FallProtectCommand command = run(controller, input, 0.3f);
        for (std::size_t side = 0; side < 2; ++side)
        {
            EXPECT_FLOAT_EQ(command.mArmDirections[side].mX, command.mArmDirection.mX);
            EXPECT_FLOAT_EQ(command.mArmDirections[side].mY, command.mArmDirection.mY);
            EXPECT_FLOAT_EQ(command.mArmDirections[side].mZ, command.mArmDirection.mZ);
            EXPECT_FLOAT_EQ(command.mArmStiffnesses[side], command.mArmStiffness);
        }
    }

    TEST(RagdollCoreFallProtect, eachArmAimsAtItsProbeHit)
    {
        FallProtectController controller;
        FallProtectInput input = falling({ 0.0f, 150.0f, -50.0f });
        input.mSettings.mProbeAim = 1.0f;
        // A wall to the left: the left probe hit it at the shoulder's height.
        input.mArms[0].mHasProbe = true;
        input.mArms[0].mShoulder = { -10.0f, 0.0f, 60.0f };
        input.mArms[0].mProbeHit = { -50.0f, 0.0f, 60.0f };
        const FallProtectCommand command = run(controller, input, 0.3f);
        EXPECT_NEAR(command.mArmDirections[0].mX, -1.0f, 1e-4f);
        EXPECT_FLOAT_EQ(command.mArmDirections[1].mY, command.mArmDirection.mY);
    }

    TEST(RagdollCoreFallProtect, aTouchingHandYieldsAloneAndTheBraceHolds)
    {
        FallProtectController controller;
        FallProtectInput input = falling({ 0.0f, -150.0f, -80.0f });
        input.mSettings.mArmYield = 1.0f;
        run(controller, input, 0.3f);
        input.mArms[1].mTouching = true;
        input.mNonFootContact = true; // the hand: no body contact
        const FallProtectCommand command = run(controller, input, input.mSettings.mAbsorbTime + 0.05f);
        EXPECT_EQ(command.mPhase, FallPhase::Bracing);
        EXPECT_TRUE(command.mArmTouched[1]);
        EXPECT_FALSE(command.mArmTouched[0]);
        EXPECT_NEAR(command.mArmStiffnesses[1], input.mSettings.mImpactArms, 1e-4f);
        EXPECT_FLOAT_EQ(command.mArmStiffnesses[0], input.mSettings.mBraceArms);
        input.mBodyContact = true;
        EXPECT_EQ(controller.update(input).mPhase, FallPhase::Impact);
    }

    TEST(RagdollCoreFallProtect, toneStaysBracedWhileTheBodyStillMoves)
    {
        const auto impactArms = [](float toneSpeed, FallVec3 velocity) {
            FallProtectController controller;
            FallProtectInput input = falling({ 0.0f, -150.0f, -80.0f });
            input.mSettings.mToneSpeed = toneSpeed;
            run(controller, input, 0.3f);
            input.mComVelocity = velocity;
            input.mNonFootContact = true;
            return run(controller, input, input.mSettings.mAbsorbTime).mArmStiffness;
        };
        const FallProtectSettings s;
        EXPECT_NEAR(impactArms(0.0f, { 0.0f, -150.0f, 0.0f }), s.mImpactArms, 0.05f); // by time
        EXPECT_GT(impactArms(100.0f, { 0.0f, -150.0f, 0.0f }), 0.7f); // still rolling: braced
        EXPECT_LT(impactArms(100.0f, { 0.0f, 0.0f, 0.0f }), 0.5f); // at rest: yielded
    }

    TEST(RagdollCoreFallProtect, variationSpreadsFallsBySeed)
    {
        const auto pose = [](std::uint32_t seed) {
            FallProtectController controller;
            FallProtectInput input = falling({ 0.0f, -150.0f, -50.0f });
            input.mSettings.mVariation = 1.0f;
            input.mSeed = seed;
            return run(controller, input, 0.3f);
        };
        const FallProtectCommand a = pose(1), again = pose(1), b = pose(2);
        EXPECT_FLOAT_EQ(a.mSpineCurl, again.mSpineCurl);
        EXPECT_NE(a.mSpineCurl, b.mSpineCurl);
        EXPECT_GE(a.mKneeBend, FallProtectSettings().mKneeBendBackward);
        EXPECT_LE(std::abs(a.mSpineCurl - FallProtectSettings().mSpineBackward), 0.25f + 1e-4f);
    }

    TEST(RagdollCoreFallProtect, aHandDownTucksTheHeadFurther)
    {
        FallProtectController controller;
        FallProtectInput input = falling({ 0.0f, -150.0f, -80.0f });
        input.mSettings.mContactTuck = 0.3f;
        const float before = run(controller, input, 0.3f).mHeadPitch;
        input.mArms[0].mTouching = true;
        const float after = run(controller, input, 0.2f).mHeadPitch;
        EXPECT_NEAR(after - before, 0.3f, 1e-4f); // backward: chin further in
    }
}
