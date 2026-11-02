#include <ragdollcore/combatstate.hpp>
#include <ragdollcore/controller.hpp>

#include <cmath>

namespace RagdollCore
{
    namespace
    {
        bool finite(const CombatReactionVector& value)
        {
            return std::isfinite(value.mX) && std::isfinite(value.mY) && std::isfinite(value.mZ);
        }

        bool known(ActiveControllerMode mode)
        {
            return mode >= ActiveControllerMode::Standing && mode <= ActiveControllerMode::BlendBack;
        }
    }

    bool CombatReactionLedger::accept(
        const CombatReactionCommand& command, ActiveControllerMode mode, std::uint64_t physicsStep)
    {
        const float impulse2 = command.mImpulse.mX * command.mImpulse.mX
            + command.mImpulse.mY * command.mImpulse.mY + command.mImpulse.mZ * command.mImpulse.mZ;
        if (command.mId == 0 || (mLast && command.mId <= mLast->mCommand.mId) || !finite(command.mWorldPoint)
            || !finite(command.mImpulse) || !(impulse2 > 0.0f) || !std::isfinite(command.mStunSeconds)
            || command.mStunSeconds < 0.0f || !known(mode))
            return false;
        mLast = CombatReactionRecord{ command, mode, physicsStep };
        ++mCount;
        return true;
    }

    std::string_view controllerModeName(ActiveControllerMode mode)
    {
        switch (mode)
        {
            case ActiveControllerMode::Standing:
                return "standing";
            case ActiveControllerMode::Bracing:
                return "bracing";
            case ActiveControllerMode::Stumbling:
                return "stumbling";
            case ActiveControllerMode::Falling:
                return "falling";
            case ActiveControllerMode::Fallen:
                return "fallen";
            case ActiveControllerMode::GettingUp:
                return "getting_up";
            case ActiveControllerMode::BlendBack:
                return "blend_back";
        }
        return "unknown";
    }

    std::optional<CombatState> makeCombatState(
        bool powered, const ActiveControllerOutput* output, std::uint64_t physicsStep)
    {
        if (!powered || output == nullptr)
            return std::nullopt;
        return CombatState{ output->mMode, output->mTimeInMode, physicsStep };
    }

    bool isDrivenControllerMode(ActiveControllerMode mode)
    {
        return mode == ActiveControllerMode::Standing || mode == ActiveControllerMode::Bracing
            || mode == ActiveControllerMode::Stumbling;
    }

    bool isLegalControllerTransition(ActiveControllerMode from, ActiveControllerMode to)
    {
        using Mode = ActiveControllerMode;
        switch (from)
        {
            case Mode::Standing:
            case Mode::Bracing:
            case Mode::Stumbling:
                return to == Mode::Standing || to == Mode::Bracing || to == Mode::Stumbling || to == Mode::Falling
                    || to == Mode::Fallen;
            case Mode::Falling:
                return to == Mode::Falling || to == Mode::Fallen || to == Mode::Standing || to == Mode::Bracing;
            case Mode::Fallen:
                return to == Mode::Fallen || to == Mode::Falling || to == Mode::GettingUp;
            case Mode::GettingUp:
                return to == Mode::GettingUp || to == Mode::BlendBack;
            case Mode::BlendBack:
                return to == Mode::BlendBack || to == Mode::Fallen || to == Mode::Falling || to == Mode::Standing;
        }
        return false;
    }
}
