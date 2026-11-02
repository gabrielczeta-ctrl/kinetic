#ifndef OPENMW_MECHANICS_RAGDOLLATTACKSTATE_H
#define OPENMW_MECHANICS_RAGDOLLATTACKSTATE_H

#include <cstdint>
#include <string>

namespace MWMechanics
{
    // Native animation intent for the current hybrid controller. This does
    // not add an animation dependency to the offline physics controller.
    enum class RagdollAttackPhase
    {
        Idle,
        WindUp,
        Release,
        FollowThrough,
    };

    // Shared by pre-aggregation and delivery: no caller may bank work from an
    // ineligible phase or revive an observation from another swing.
    inline bool permitsRagdollAttackContact(bool owned, RagdollAttackPhase phase,
        bool capturedActive, std::uint64_t capturedSwing, std::uint64_t currentSwing) noexcept
    {
        return !owned || ((phase == RagdollAttackPhase::Release || phase == RagdollAttackPhase::FollowThrough)
            && capturedActive && capturedSwing == currentSwing);
    }

    struct RagdollAttackState
    {
        RagdollAttackPhase mPhase = RagdollAttackPhase::Idle;
        std::uint64_t mAttackId = 0;
        std::string mWeaponRecordId;
        std::string mAttackType;
        float mStrength = 0.f;
        // Normal playback completion differs from cancellation: the physical
        // weapon may still be finishing its bounded braking interval.
        bool mPlaybackComplete = false;
    };

    // Retain a press even if its release arrives in the same event batch.
    // Item identity is copied, never a pointer that can outlive its actor.
    struct RagdollAttackInput
    {
        bool mPressed = false;
        bool mHeld = false;
        bool mCancelled = false;
        std::string mWeaponRecordId;
    };
}

#endif
