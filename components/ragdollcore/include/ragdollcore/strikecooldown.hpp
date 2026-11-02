#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_STRIKECOOLDOWN_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_STRIKECOOLDOWN_H

#include <cstdint>
#include <unordered_map>

namespace RagdollCore
{
    // Per-attacker, per-target gate for counted hits. One physical blow can
    // arrive as many solver observations: several victim bodies, several
    // manifolds, and the cutting model paying work in installments. The RPG
    // pipeline must still see the strike once. Every accepted hit arms a
    // cooldown measured on the fixed physics-step clock, so a held drag or an
    // animation that keeps touching cannot charge condition, skill progress
    // or hit reactions again until the gate elapses. Targets are independent,
    // so one swing can still strike several opponents.
    class StrikeCooldown final
    {
    public:
        // WPN-07 default: long enough to swallow contact chatter and cutting
        // installments, short enough that ordinary authored attack rates
        // (even a 2.5-speed dagger) are never throttled.
        static constexpr float sDefaultCooldownSeconds = 0.35f;

        StrikeCooldown()
            : StrikeCooldown(sDefaultCooldownSeconds, 1.0f / 60.0f)
        {
        }

        // Non-finite or negative durations take the default. An explicit zero
        // disables the gate. Step seconds is the fixed physics step.
        StrikeCooldown(float cooldownSeconds, float stepSeconds);

        // True when a hit on this target may be counted on this step. Reads
        // state only; commit() records the hit once it was actually applied.
        bool allows(std::uint64_t targetIdentity, std::uint64_t physicsStep) const noexcept;

        void commit(std::uint64_t targetIdentity, std::uint64_t physicsStep);

        void reset() noexcept;

        float cooldownSeconds() const noexcept { return mCooldownSeconds; }
        std::uint64_t cooldownSteps() const noexcept { return mCooldownSteps; }

    private:
        float mCooldownSeconds = sDefaultCooldownSeconds;
        std::uint64_t mCooldownSteps = 0;
        std::unordered_map<std::uint64_t, std::uint64_t> mLastHitStep;
    };
}

#endif
