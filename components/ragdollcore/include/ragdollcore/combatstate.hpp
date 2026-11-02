#ifndef OPENMW_RAGDOLLCORE_COMBATSTATE_H
#define OPENMW_RAGDOLLCORE_COMBATSTATE_H

#include <cstdint>
#include <optional>
#include <string_view>

namespace RagdollCore
{
    enum class ActiveControllerMode
    {
        Standing,
        Bracing,
        Stumbling,
        Falling,
        Fallen,
        GettingUp,
        BlendBack,
    };

    // A value snapshot of the powered controller, not another state machine.
    // GettingUp and BlendBack remain explicit so recovery is never reported as standing.
    struct CombatState
    {
        ActiveControllerMode mMode;
        float mTimeInMode;
        std::uint64_t mPhysicsStep;
    };

    struct CombatReactionVector
    {
        float mX = 0.0f;
        float mY = 0.0f;
        float mZ = 0.0f;
    };

    // Immutable value command crossing from a landed melee connection into
    // powered physics. Its monotonic identity makes retries and duplicates
    // observable without retaining actor or physics pointers.
    struct CombatReactionCommand
    {
        std::uint64_t mId = 0;
        CombatReactionVector mWorldPoint;
        CombatReactionVector mImpulse;
        float mStunSeconds = 0.0f;
    };

    struct CombatReactionRecord
    {
        CombatReactionCommand mCommand;
        ActiveControllerMode mModeAtReceipt = ActiveControllerMode::Standing;
        std::uint64_t mPhysicsStep = 0;
    };

    class CombatReactionLedger
    {
    public:
        bool accept(const CombatReactionCommand& command, ActiveControllerMode mode, std::uint64_t physicsStep);
        std::uint64_t count() const { return mCount; }
        const std::optional<CombatReactionRecord>& last() const { return mLast; }

    private:
        std::uint64_t mCount = 0;
        std::optional<CombatReactionRecord> mLast;
    };

    struct ActiveControllerOutput;
    std::optional<CombatState> makeCombatState(
        bool powered, const ActiveControllerOutput* output, std::uint64_t physicsStep);
    std::string_view controllerModeName(ActiveControllerMode mode);

    // Modes that still own powered pose and balance control. Gameplay actions
    // may continue through these modes; falling and recovery states interrupt
    // them.
    bool isDrivenControllerMode(ActiveControllerMode mode);

    // Legal adjacent fixed-step transitions. Polling can skip intermediate modes;
    // callers must not use this to reject two nonadjacent render-frame snapshots.
    bool isLegalControllerTransition(ActiveControllerMode from, ActiveControllerMode to);
}

#endif
