#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_WOUND_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_WOUND_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string_view>

#include <ragdollcore/anatomy.hpp>

namespace RagdollCore
{
    enum class WoundKind : std::uint8_t { None, Bruise, Slash, Puncture };

    inline const char* woundKindName(WoundKind kind)
    {
        switch (kind)
        {
            case WoundKind::Bruise: return "bruise";
            case WoundKind::Slash: return "slash";
            case WoundKind::Puncture: return "puncture";
            default: return "none";
        }
    }

    // Supported stock blades' authored hand frame: +Z points toward the tip,
    // X spans the cutting edges, Y is the flat. Inputs are projected into
    // that frame in the contact callback, never read back after integration.
    struct BladeContact
    {
        float mAlong = 0; // distance from grip along +Z, divided by grip-to-tip length
        float mNormalTip = 0;
        float mNormalEdge = 0;
        float mNormalFlat = 0;
        float mVelocityTip = 0;
        float mSpeed = 0;
        float mApproach = 0;
    };

    struct BladeSurfaceProfile
    {
        float mBladeStart; // fraction of measured grip-to-tip distance
        float mTipStart;
    };

    inline std::optional<BladeSurfaceProfile> bladeSurfaceProfile(std::string_view record)
    {
        if (record == "iron dagger") return BladeSurfaceProfile{ 0.10f, 0.80f };
        // Keep the guard/root blunt; only the distal tenth can puncture.
        // Stock mesh in hand space: tip +Z, edges X, flat Y; grip-to-tip 61.84.
        if (record == "iron longsword") return BladeSurfaceProfile{ 0.10f, 0.90f };
        return std::nullopt;
    }

    inline WoundKind classifyBladeContact(const BladeContact& hit, const BladeSurfaceProfile& profile)
    {
        for (const float v : { hit.mAlong, hit.mNormalTip, hit.mNormalEdge, hit.mNormalFlat,
                 hit.mVelocityTip, hit.mSpeed, hit.mApproach })
            if (!std::isfinite(v))
                return WoundKind::None;
        if (hit.mSpeed < 1.0f || hit.mApproach < -0.5f)
            return WoundKind::None;
        if (hit.mAlong <= profile.mBladeStart) // grip, guard and pommel
            return WoundKind::Bruise;
        if (hit.mAlong >= profile.mTipStart && hit.mApproach >= 1.0f && hit.mNormalTip >= 0.55f
            && hit.mVelocityTip >= 0.55f * hit.mSpeed)
            return WoundKind::Puncture;
        if (std::abs(hit.mNormalEdge) >= 0.45f
            && std::abs(hit.mNormalEdge) > std::abs(hit.mNormalFlat) * 1.1f)
            return WoundKind::Slash;
        return hit.mApproach >= 1.0f ? WoundKind::Bruise : WoundKind::None;
    }

    // Every melee weapon's wound class (user 2026-10-01: "expand the wound
    // system to the rest of the weapons"). Blades cut along the edge and stab
    // with the tip, axes cut with the head and bruise with the haft, spears
    // stab with the tip, blunt weapons bruise. Contacts are measured in the
    // weapon's own frame (blade axis toward the tip, edge axis across it).
    enum class WoundClass : std::uint8_t { None, Blade, Axe, Spear, Blunt };
    struct WoundProfile
    {
        WoundClass mClass = WoundClass::None;
        float mStrikeStart = 0.10f; // fraction of grip-to-tip: grip and guard bruise below it
        float mTipStart = 0.85f; // fraction where the point can puncture
    };

    inline WoundKind classifyWeaponContact(const BladeContact& hit, const WoundProfile& profile)
    {
        for (const float v : { hit.mAlong, hit.mNormalTip, hit.mNormalEdge, hit.mNormalFlat,
                 hit.mVelocityTip, hit.mSpeed, hit.mApproach })
            if (!std::isfinite(v))
                return WoundKind::None;
        if (hit.mSpeed < 1.0f || hit.mApproach < -0.5f)
            return WoundKind::None;
        const WoundKind blunt = hit.mApproach >= 1.0f ? WoundKind::Bruise : WoundKind::None;
        const bool edge = std::abs(hit.mNormalEdge) >= 0.45f
            && std::abs(hit.mNormalEdge) > std::abs(hit.mNormalFlat) * 1.1f;
        const bool point = hit.mAlong >= profile.mTipStart && hit.mApproach >= 1.0f && hit.mNormalTip >= 0.55f
            && hit.mVelocityTip >= 0.55f * hit.mSpeed;
        switch (profile.mClass)
        {
            case WoundClass::Blade:
                return classifyBladeContact(hit, BladeSurfaceProfile{ profile.mStrikeStart, profile.mTipStart });
            case WoundClass::Axe:
                if (hit.mAlong < profile.mStrikeStart)
                    return blunt;
                return edge ? WoundKind::Slash : blunt;
            case WoundClass::Spear:
                return point ? WoundKind::Puncture : blunt;
            case WoundClass::Blunt:
                return blunt;
            default:
                return WoundKind::None;
        }
    }

    using DaggerContact = BladeContact;
    inline WoundKind classifyIronDagger(const DaggerContact& hit)
    {
        return classifyBladeContact(hit, *bladeSurfaceProfile("iron dagger"));
    }

    // Bruises include fatigue-first punches; sharp wounds need actual health
    // loss. This is called only with an attributable post-application receipt.
    inline float woundSeverity(WoundKind kind, float healthLoss, float fatigueLoss)
    {
        if (!std::isfinite(healthLoss) || !std::isfinite(fatigueLoss))
            return 0;
        const float damage = std::max(0.0f, healthLoss)
            + (kind == WoundKind::Bruise ? 0.2f * std::max(0.0f, fatigueLoss) : 0.0f);
        if (kind == WoundKind::None || damage <= 0)
            return 0;
        return std::clamp(damage / 12.0f, 0.15f, 1.0f);
    }

    // Bounded transient response, separate from durable disability. New hits
    // merge in place: they cannot build an unbounded angular displacement.
    struct LocalWoundReaction
    {
        float mAge = 1;
        float mStrength = 0;
        void begin(float severity)
        {
            if (!std::isfinite(severity) || severity <= 0)
                return;
            mStrength = std::clamp(std::max(mStrength * envelope(), severity), 0.0f, 1.0f);
            mAge = 0;
        }
        void step(float dt)
        {
            if (std::isfinite(dt) && dt > 0)
                mAge = std::min(1.0f, mAge + dt);
        }
        float envelope() const
        {
            // Smooth attack (80 ms) and recovery (370 ms), no pose jump.
            const float x = mAge < 0.08f ? mAge / 0.08f : (0.45f - mAge) / 0.37f;
            const float t = std::clamp(x, 0.0f, 1.0f);
            return t * t * (3.0f - 2.0f * t);
        }
        float angle(AnatomicalRegion region) const
        {
            // Small support-leg response, stronger arm withdrawal. Radians.
            const float limit = regionCanKnockDown(region) ? 0.07f
                : region == AnatomicalRegion::Head || region == AnatomicalRegion::Neck ? 0.12f : 0.28f;
            return limit * mStrength * envelope();
        }
    };
}
#endif
