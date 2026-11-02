#include <ragdollcore/physicaldefense.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        bool validProfile(const PhysicalDefenseProfile& profile) noexcept
        {
            return std::isfinite(profile.mMinimumNormalSpeed) && profile.mMinimumNormalSpeed >= 0.0f
                && std::isfinite(profile.mMaximumGlancingAlignment)
                && profile.mMaximumGlancingAlignment >= 0.0f && profile.mMaximumGlancingAlignment <= 1.0f
                && std::isfinite(profile.mMinimumGuardFacingAlignment)
                && profile.mMinimumGuardFacingAlignment >= -1.0f && profile.mMinimumGuardFacingAlignment <= 1.0f
                && profile.mParryWindowSteps <= profile.mBlockWindowSteps
                && std::isfinite(profile.mGlancingDamageScale) && profile.mGlancingDamageScale >= 0.0f
                && profile.mGlancingDamageScale <= 1.0f
                && std::isfinite(profile.mGlancingImpulseScale) && profile.mGlancingImpulseScale >= 0.0f
                && profile.mGlancingImpulseScale <= 1.0f
                && std::isfinite(profile.mBlockRedirectScale) && profile.mBlockRedirectScale >= 0.0f
                && std::isfinite(profile.mParryRedirectScale) && profile.mParryRedirectScale >= 0.0f;
        }
    }

    PhysicalDefenseResult classifyPhysicalDefense(
        const PhysicalDefenseInput& input, const PhysicalDefenseProfile& profile) noexcept
    {
        PhysicalDefenseResult result;
        if (!input.mContactValid || !validProfile(profile) || !std::isfinite(input.mRelativeNormalSpeed)
            || !std::isfinite(input.mRelativeSpeed) || !std::isfinite(input.mGuardFacingAlignment)
            || input.mRelativeSpeed < 0.0f)
            return result;
        result.mGuardFacingAlignment = input.mGuardFacingAlignment;
        if (input.mFriendlyContact)
        {
            result.mDisposition = PhysicalContactDisposition::Friendly;
            return result;
        }
        if (input.mRelativeNormalSpeed < profile.mMinimumNormalSpeed || input.mRelativeSpeed <= 0.0f)
        {
            result.mDisposition = PhysicalContactDisposition::Rest;
            return result;
        }

        result.mApproachAlignment
            = std::clamp(input.mRelativeNormalSpeed / input.mRelativeSpeed, 0.0f, 1.0f);
        const bool glancing = result.mApproachAlignment < profile.mMaximumGlancingAlignment;
        result.mDisposition = glancing ? PhysicalContactDisposition::Glancing : PhysicalContactDisposition::Direct;

        const bool guardCoversContact = input.mGuardActive
            && input.mGuardFacingAlignment >= profile.mMinimumGuardFacingAlignment
            && input.mContactStep >= input.mGuardStartStep;
        if (guardCoversContact)
        {
            const std::uint64_t guardAge = input.mContactStep - input.mGuardStartStep;
            if (guardAge <= profile.mParryWindowSteps)
            {
                result.mOutcome = PhysicalDefenseOutcome::Parry;
                result.mRedirectScale = profile.mParryRedirectScale;
                return result;
            }
            if (guardAge <= profile.mBlockWindowSteps || profile.mHeldGuardBlocks)
            {
                result.mOutcome = PhysicalDefenseOutcome::Block;
                result.mRedirectScale = profile.mBlockRedirectScale;
                return result;
            }
        }

        result.mOutcome = PhysicalDefenseOutcome::Hit;
        result.mDamageScale = glancing ? profile.mGlancingDamageScale : 1.0f;
        result.mImpulseScale = glancing ? profile.mGlancingImpulseScale : 1.0f;
        return result;
    }

    std::string_view physicalDefenseOutcomeName(PhysicalDefenseOutcome outcome) noexcept
    {
        switch (outcome)
        {
            case PhysicalDefenseOutcome::None: return "none";
            case PhysicalDefenseOutcome::Hit: return "hit";
            case PhysicalDefenseOutcome::Block: return "block";
            case PhysicalDefenseOutcome::Parry: return "parry";
        }
        return "none";
    }

    std::string_view physicalContactDispositionName(PhysicalContactDisposition disposition) noexcept
    {
        switch (disposition)
        {
            case PhysicalContactDisposition::Invalid: return "invalid";
            case PhysicalContactDisposition::Rest: return "rest";
            case PhysicalContactDisposition::Friendly: return "friendly";
            case PhysicalContactDisposition::Glancing: return "glancing";
            case PhysicalContactDisposition::Direct: return "direct";
        }
        return "invalid";
    }
}
