#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_PHYSICALDEFENSE_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_PHYSICALDEFENSE_H

#include <cstdint>
#include <string_view>

namespace RagdollCore
{
    enum class PhysicalDefenseOutcome
    {
        None,
        Hit,
        Block,
        Parry,
    };

    enum class PhysicalContactDisposition
    {
        Invalid,
        Rest,
        Friendly,
        Glancing,
        Direct,
    };

    struct PhysicalDefenseProfile final
    {
        float mMinimumNormalSpeed = 1.0f;
        float mMaximumGlancingAlignment = 0.35f;
        float mMinimumGuardFacingAlignment = 0.5f;
        std::uint64_t mParryWindowSteps = 9;
        std::uint64_t mBlockWindowSteps = 36;
        // A guard held past the block window still blocks what it covers (the
        // right-click guard is held for seconds; the window was for the NPC's
        // per-swing raise). Off = the window only.
        bool mHeldGuardBlocks = false;
        float mGlancingDamageScale = 0.25f;
        float mGlancingImpulseScale = 0.35f;
        float mBlockRedirectScale = 0.35f;
        float mParryRedirectScale = 0.65f;
    };

    struct PhysicalDefenseInput final
    {
        bool mContactValid = false;
        bool mFriendlyContact = false;
        bool mGuardActive = false;
        std::uint64_t mContactStep = 0;
        std::uint64_t mGuardStartStep = 0;
        float mRelativeNormalSpeed = 0.0f;
        float mRelativeSpeed = 0.0f;
        // One means the attacker is directly in front of the defender, zero
        // means perpendicular and negative means behind the guard.
        float mGuardFacingAlignment = -1.0f;
    };

    struct PhysicalDefenseResult final
    {
        PhysicalDefenseOutcome mOutcome = PhysicalDefenseOutcome::None;
        PhysicalContactDisposition mDisposition = PhysicalContactDisposition::Invalid;
        float mApproachAlignment = 0.0f;
        float mGuardFacingAlignment = -1.0f;
        float mDamageScale = 0.0f;
        float mImpulseScale = 0.0f;
        float mRedirectScale = 0.0f;

        bool emitsOutcome() const noexcept { return mOutcome != PhysicalDefenseOutcome::None; }
    };

    // Classifies one already de-duplicated physical weapon contact. Rest and
    // friendly controls emit no combat outcome. Every other valid contact
    // emits exactly one hit, block, or parry for downstream combat logic.
    PhysicalDefenseResult classifyPhysicalDefense(const PhysicalDefenseInput& input,
        const PhysicalDefenseProfile& profile = {}) noexcept;

    std::string_view physicalDefenseOutcomeName(PhysicalDefenseOutcome outcome) noexcept;
    std::string_view physicalContactDispositionName(PhysicalContactDisposition disposition) noexcept;
}

#endif
