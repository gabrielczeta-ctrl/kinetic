#include <ragdollcore/combatstate.hpp>
#include <ragdollcore/controller.hpp>

#include <gtest/gtest.h>

#include <array>
#include <limits>

namespace
{
    using Mode = RagdollCore::ActiveControllerMode;
    constexpr std::array modes{ Mode::Standing, Mode::Bracing, Mode::Stumbling, Mode::Falling, Mode::Fallen,
        Mode::GettingUp, Mode::BlendBack };

    TEST(RagdollCombatStateTest, copiesEveryControllerModeWithoutOwningTheController)
    {
        constexpr std::array names{ "standing", "bracing", "stumbling", "falling", "fallen", "getting_up",
            "blend_back" };
        for (std::size_t i = 0; i < modes.size(); ++i)
        {
            RagdollCore::ActiveControllerOutput output;
            output.mMode = modes[i];
            output.mTimeInMode = 0.25f;
            const auto state = RagdollCore::makeCombatState(true, &output, 123);
            output = {};
            ASSERT_TRUE(state);
            EXPECT_EQ(state->mMode, modes[i]);
            EXPECT_EQ(RagdollCore::controllerModeName(state->mMode), names[i]);
            EXPECT_FLOAT_EQ(state->mTimeInMode, 0.25f);
            EXPECT_EQ(state->mPhysicsStep, 123u);
        }
    }

    TEST(RagdollCombatStateTest, absentAndPassiveActorsHaveNoCombatMode)
    {
        RagdollCore::ActiveControllerOutput output;
        EXPECT_FALSE(RagdollCore::makeCombatState(false, nullptr, 0));
        EXPECT_FALSE(RagdollCore::makeCombatState(true, nullptr, 0));
        EXPECT_FALSE(RagdollCore::makeCombatState(false, &output, 123));
    }

    TEST(RagdollCombatStateTest, drivenModesKeepGameplayActionsWhileRecoveryModesInterruptThem)
    {
        EXPECT_TRUE(RagdollCore::isDrivenControllerMode(Mode::Standing));
        EXPECT_TRUE(RagdollCore::isDrivenControllerMode(Mode::Bracing));
        EXPECT_TRUE(RagdollCore::isDrivenControllerMode(Mode::Stumbling));
        EXPECT_FALSE(RagdollCore::isDrivenControllerMode(Mode::Falling));
        EXPECT_FALSE(RagdollCore::isDrivenControllerMode(Mode::Fallen));
        EXPECT_FALSE(RagdollCore::isDrivenControllerMode(Mode::GettingUp));
        EXPECT_FALSE(RagdollCore::isDrivenControllerMode(Mode::BlendBack));
        EXPECT_FALSE(RagdollCore::isDrivenControllerMode(static_cast<Mode>(100)));
    }

    TEST(RagdollCombatStateTest, rejectsIllegalAndUnknownAdjacentTransitions)
    {
        // Rows and columns are the public mode order above. Includes staying in a mode.
        constexpr std::array<std::string_view, 7> expected{ "1111100", "1111100", "1111100", "1101100", "0001110",
            "0000011", "1001101" };
        for (std::size_t from = 0; from < modes.size(); ++from)
            for (std::size_t to = 0; to < modes.size(); ++to)
            {
                SCOPED_TRACE(std::to_string(from) + " -> " + std::to_string(to));
                EXPECT_EQ(RagdollCore::isLegalControllerTransition(modes[from], modes[to]), expected[from][to] == '1');
            }
        const auto unknown = static_cast<Mode>(100);
        EXPECT_FALSE(RagdollCore::isLegalControllerTransition(unknown, unknown));
        for (Mode mode : modes)
        {
            EXPECT_FALSE(RagdollCore::isLegalControllerTransition(mode, unknown));
            EXPECT_FALSE(RagdollCore::isLegalControllerTransition(unknown, mode));
        }
    }

    RagdollCore::ActiveControllerInput upright()
    {
        RagdollCore::ActiveControllerInput input;
        input.mSupport.mLeftFootSupported = true;
        input.mSupport.mRightFootSupported = true;
        input.mSupport.mFootContactCount = 2;
        input.mSupport.mGroundContactCount = 2;
        input.mSupport.mCenterOfMassSupported = true;
        input.mSupport.mCenterOfMassHeight = 80;
        input.mPelvisHeight = 80;
        return input;
    }

    TEST(RagdollCombatStateTest, fixedStepPhysicalObservationsReachEveryMode)
    {
        // Repeat the same physical observation sequence with the production timings.
        // This qualifies the observation boundary, not physical recovery success.
        for (int repeat = 0; repeat < 2; ++repeat)
        {
            RagdollCore::ActiveRagdollController controller;
            RagdollCore::ActiveControllerProfile profile;
            auto input = upright();
            std::uint64_t step = 0;
            const auto reach = [&](Mode expected) {
                bool observed = false;
                for (int i = 0; i < 180; ++i)
                {
                    const auto previous = controller.getOutput().mMode;
                    RagdollCore::ActiveControllerOutput output;
                    ASSERT_TRUE(controller.step(input, 1.0f / 60, profile, output));
                    ASSERT_TRUE(RagdollCore::isLegalControllerTransition(previous, output.mMode));
                    const auto state = RagdollCore::makeCombatState(true, &output, ++step);
                    ASSERT_TRUE(state);
                    EXPECT_EQ(state->mMode, controller.getOutput().mMode);
                    EXPECT_FLOAT_EQ(state->mTimeInMode, controller.getOutput().mTimeInMode);
                    EXPECT_EQ(state->mPhysicsStep, step);
                    if (state->mMode == expected)
                    {
                        observed = true;
                        break;
                    }
                }
                ASSERT_TRUE(observed) << RagdollCore::controllerModeName(expected);
            };
            reach(Mode::Standing);
            input.mSupport.mCenterOfMassSupported = false;
            input.mPelvisUpDot = 0.8f;
            reach(Mode::Bracing);
            input.mPelvisUpDot = 0.4f;
            reach(Mode::Stumbling);
            input.mSupport = {};
            reach(Mode::Falling);
            input.mSupport.mGroundContactCount = 1;
            input.mPelvisHeight = 10;
            reach(Mode::Fallen);
            input.mRecoveryRequested = true;
            reach(Mode::GettingUp);
            input.mRecoveryAnimationFinished = true;
            reach(Mode::BlendBack);
            input = upright();
            reach(Mode::Standing);
        }
    }

    TEST(RagdollCombatStateTest, negativeControlsCannotInventStandingOrRecovery)
    {
        RagdollCore::ActiveRagdollController controller;
        RagdollCore::ActiveControllerProfile profile;
        controller.reset(Mode::Fallen);
        auto input = upright();
        // Upright telemetry alone cannot skip the requested get-up sequence.
        for (int i = 0; i < 180; ++i)
        {
            RagdollCore::ActiveControllerOutput output;
            ASSERT_TRUE(controller.step(input, 1.0f / 60, profile, output));
            ASSERT_EQ(RagdollCore::makeCombatState(true, &output, i)->mMode, Mode::Fallen);
        }
        // Invalid input must preserve the last valid observation and its timer.
        const auto before = controller.getOutput();
        input.mPelvisHeight = std::numeric_limits<float>::quiet_NaN();
        RagdollCore::ActiveControllerOutput rejected;
        EXPECT_FALSE(controller.step(input, 1.0f / 60, profile, rejected));
        const auto after = controller.getOutput();
        EXPECT_EQ(after.mMode, before.mMode);
        EXPECT_FLOAT_EQ(after.mTimeInMode, before.mTimeInMode);
    }

    TEST(RagdollCombatStateTest, reactionLedgerAcceptsEachIdentifiedCommandExactlyOnce)
    {
        RagdollCore::CombatReactionLedger ledger;
        RagdollCore::CombatReactionCommand command;
        command.mId = 41;
        command.mWorldPoint = { 1, 2, 3 };
        command.mImpulse = { 1500, 0, 300 };
        command.mStunSeconds = 0.35f;
        ASSERT_TRUE(ledger.accept(command, Mode::Standing, 90));
        EXPECT_EQ(ledger.count(), 1u);
        ASSERT_TRUE(ledger.last());
        EXPECT_EQ(ledger.last()->mCommand.mId, 41u);
        EXPECT_FLOAT_EQ(ledger.last()->mCommand.mWorldPoint.mZ, 3);
        EXPECT_FLOAT_EQ(ledger.last()->mCommand.mImpulse.mX, 1500);
        EXPECT_FLOAT_EQ(ledger.last()->mCommand.mStunSeconds, 0.35f);
        EXPECT_EQ(ledger.last()->mModeAtReceipt, Mode::Standing);
        EXPECT_EQ(ledger.last()->mPhysicsStep, 90u);

        EXPECT_FALSE(ledger.accept(command, Mode::Standing, 91));
        command.mId = 40;
        EXPECT_FALSE(ledger.accept(command, Mode::Standing, 91));
        command.mId = 42;
        ASSERT_TRUE(ledger.accept(command, Mode::Bracing, 92));
        EXPECT_EQ(ledger.count(), 2u);
    }

    TEST(RagdollCombatStateTest, reactionLedgerRejectsMalformedCommands)
    {
        RagdollCore::CombatReactionLedger ledger;
        RagdollCore::CombatReactionCommand command;
        command.mId = 1;
        command.mWorldPoint = { 1, 2, 3 };
        command.mImpulse = { 1, 0, 0 };
        command.mStunSeconds = 0;
        command.mImpulse.mX = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(ledger.accept(command, Mode::Standing, 1));
        command.mImpulse = {};
        EXPECT_FALSE(ledger.accept(command, Mode::Standing, 1));
        command.mImpulse.mX = 1;
        command.mWorldPoint.mZ = std::numeric_limits<float>::infinity();
        EXPECT_FALSE(ledger.accept(command, Mode::Standing, 1));
        command.mWorldPoint = {};
        command.mStunSeconds = -1;
        EXPECT_FALSE(ledger.accept(command, Mode::Standing, 1));
        command.mStunSeconds = 0;
        EXPECT_FALSE(ledger.accept(command, static_cast<Mode>(99), 1));
        EXPECT_EQ(ledger.count(), 0u);
    }
}
