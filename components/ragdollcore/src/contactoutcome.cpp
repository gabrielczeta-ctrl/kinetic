#include <ragdollcore/contactoutcome.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        float finiteOr(float value, float fallback)
        {
            return std::isfinite(value) ? value : fallback;
        }
    }

    const char* bodyContactOutcomeName(BodyContactOutcome outcome)
    {
        switch (outcome)
        {
            case BodyContactOutcome::Glance: return "glance";
            case BodyContactOutcome::Bite: return "bite";
            case BodyContactOutcome::Stop: return "stop";
            case BodyContactOutcome::Cleave: return "cleave";
        }
        return "glance";
    }

    const char* clashOutcomeName(ClashOutcome outcome)
    {
        switch (outcome)
        {
            case ClashOutcome::Deflect: return "deflect";
            case ClashOutcome::Bounce: return "bounce";
            case ClashOutcome::Knockaway: return "knockaway";
            case ClashOutcome::BrokenGuard: return "broken_guard";
            case ClashOutcome::Disarm: return "disarm";
        }
        return "bounce";
    }

    BodyContactResult classifyBodyContact(const BodyContactInput& input, const BodyContactSettings& s)
    {
        BodyContactResult result;
        const float vn = std::max(0.0f, finiteOr(input.mNormalSpeed, 0.0f));
        const float vt = std::max(0.0f, finiteOr(input.mTangentSpeed, 0.0f));
        const float mass = std::max(0.0f, finiteOr(input.mEffectiveMass, 0.0f));
        const float hardness = std::clamp(finiteOr(input.mHardness, 0.0f), 0.0f, 1.0f);
        const float alignment = std::clamp(finiteOr(input.mEdgeAlignment, 1.0f), 0.0f, 1.0f);
        const float vnMetres = vn / sContactUnitsPerMetre;
        result.mEnergy = 0.5f * mass * vnMetres * vnMetres;
        result.mCutEnergy = s.mCutEnergySoft + (s.mCutEnergyHard - s.mCutEnergySoft) * hardness * hardness;
        if (input.mKind == BlowKind::Point)
            result.mCutEnergy *= s.mPointShare;

        const bool shallow = vn < s.mShallowRatio * vt;
        // Blunt weapons and fists never cut: square, they stop; shallow,
        // they glance off.
        if (input.mKind == BlowKind::Blunt || input.mKind == BlowKind::Fist || input.mKind == BlowKind::Scripted)
        {
            result.mOutcome = shallow ? BodyContactOutcome::Glance : BodyContactOutcome::Stop;
            result.mSpeedAfter = shallow ? vt : 0.0f;
            result.mDamageScale = shallow ? 0.3f : 1.0f;
            return result;
        }
        // An edge sliding along flesh is a draw cut, the best cut there is,
        // not a glance (FIX-08 10-07: sweeping claymore cuts paid 30%); it
        // glances off a hard slot when shallow or with the flat. A point
        // skids off anything when shallow. A touch under the cut energy is a
        // nick, judged against flesh (on mail a weak cut is still a blunt
        // blow, a stop).
        const bool flat = input.mKind == BlowKind::Edge && alignment < s.mMisalignedEdge;
        const bool skids = shallow && (input.mKind == BlowKind::Point || hardness >= 0.5f);
        if (skids || (flat && hardness >= 0.5f) || result.mEnergy < 0.3f * s.mCutEnergySoft)
        {
            result.mOutcome = BodyContactOutcome::Glance;
            result.mSpeedAfter = vt;
            result.mDamageScale = 0.3f;
            return result;
        }
        if (result.mEnergy < result.mCutEnergy || flat)
        {
            result.mOutcome = BodyContactOutcome::Stop;
            result.mDamageScale = flat ? 0.5f : 1.0f; // the flat bruises
            // A square edge the armour stops bites into it and holds.
            if (!flat && input.mKind == BlowKind::Edge && hardness > 0.0f)
                result.mHoldTime = std::max(0.0f, finiteOr(s.mEdgeStick, 0.0f));
            return result;
        }
        const float over = result.mEnergy - result.mCutEnergy;
        if (input.mSoftPart && input.mKind == BlowKind::Edge && hardness < 0.3f && alignment >= s.mCleaveAlignment
            && over >= s.mCleaveEnergy)
        {
            result.mOutcome = BodyContactOutcome::Cleave;
            const float spent = s.mCleaveSpend * result.mCutEnergy;
            const float vAfterMetres = mass > 0.0f
                ? std::sqrt(std::max(0.0f, vnMetres * vnMetres - 2.0f * spent / mass)) : 0.0f;
            result.mSpeedAfter = vAfterMetres * sContactUnitsPerMetre;
            return result;
        }
        result.mOutcome = BodyContactOutcome::Bite;
        const float share = std::clamp(over / std::max(1e-3f, s.mCleaveEnergy), 0.0f, 1.0f);
        result.mHoldTime = s.mBiteHoldMin + (s.mBiteHoldMax - s.mBiteHoldMin) * share;
        return result;
    }

    ClashResult classifyClash(const ClashInput& input, const ClashSettings& s)
    {
        ClashResult result;
        const float attack = std::max(0.0f, finiteOr(input.mAttackMomentum, 0.0f));
        const float guard = std::max(1e-3f, finiteOr(input.mGuardFirmness, 0.0f));
        const float alignment = std::clamp(finiteOr(input.mBladeAlignment, 0.0f), 0.0f, 1.0f);
        result.mRatio = attack / guard;
        const float even = std::max(1.0f, s.mEven);

        if (result.mRatio > even)
        {
            // The attack wins: the guard is pushed through.
            result.mOutcome = ClashOutcome::BrokenGuard;
            result.mDefenderLockout = s.mBrokenGuardLockout;
            result.mAttackContinues = std::clamp(1.0f - 1.0f / result.mRatio, 0.0f, 1.0f);
            if (result.mRatio > s.mDisarmMargin && finiteOr(input.mDefenderGrip, 1.0f) < s.mWeakGrip)
            {
                result.mOutcome = ClashOutcome::Disarm;
                result.mDefenderDisarmed = true;
            }
            return result;
        }
        if (input.mActiveParry && result.mRatio < 1.0f / even)
        {
            // The parry wins: the attack is thrown off line.
            result.mOutcome = ClashOutcome::Knockaway;
            result.mAttackerLockout = s.mKnockawayLockout;
            if (result.mRatio < 1.0f / s.mDisarmMargin && finiteOr(input.mAttackerGrip, 1.0f) < s.mWeakGrip)
            {
                result.mOutcome = ClashOutcome::Disarm;
                result.mAttackerDisarmed = true;
            }
            return result;
        }
        if (alignment > s.mDeflectAlignment)
        {
            // Shallow: it slides off along the guard and keeps its speed.
            result.mOutcome = ClashOutcome::Deflect;
            result.mAttackContinues = std::clamp(alignment, 0.0f, 1.0f);
            return result;
        }
        result.mOutcome = ClashOutcome::Bounce;
        return result;
    }

    ImpactHold pressImpact(BodyContactOutcome outcome, float hardness, float holdTime, ImpactHold before,
        const BlowPressSettings& s)
    {
        if (!s.mOn)
            return before;
        switch (outcome)
        {
            case BodyContactOutcome::Glance:
            case BodyContactOutcome::Cleave:
                return {};
            case BodyContactOutcome::Bite:
                return {}; // presses on: the edge is in, the body is driven
            case BodyContactOutcome::Stop:
                // An edge stuck in armour sticks; a hard slot rebounds;
                // flesh (or leather) takes the blow as a shove.
                if (holdTime > 0.0f)
                    return { holdTime, 0.0f };
                if (finiteOr(hardness, 0.0f) >= s.mHardFrom)
                    return before;
                return {};
        }
        return before;
    }

    PushKick pushKick(const PushKickInput& in, const PushKickSettings& s)
    {
        const float foot = std::clamp(finiteOr(in.mFootSpeed, 0.0f) / 350.0f, 0.5f, 1.25f);
        const float strengthRaw = std::max(0.0f, finiteOr(in.mStrength, 50.0f)) / 50.0f;
        const float guard = in.mGuarded ? 0.8f : 1.0f;
        PushKick out;
        const float base = std::max(0.0f, finiteOr(in.mBaseSpeed, 240.0f));
        const float leg = std::max(0.0f, finiteOr(in.mLegMass, 14.0f));
        if (!s.mOn)
        {
            out.mSpeed = base * std::clamp(strengthRaw, 0.6f, 1.4f) * foot * guard;
            out.mMass = leg;
            return out;
        }
        const float strength = std::clamp(strengthRaw, 0.6f, std::max(0.6f, finiteOr(s.mStrengthCap, 4.0f)));
        const float skill = std::clamp(finiteOr(in.mHandToHand, 0.0f) / 100.0f, 0.0f, 1.5f);
        out.mSpeed = (base * strength * foot + std::max(0.0f, finiteOr(in.mKickerAlong, 0.0f))) * guard;
        out.mMass = leg
            + std::max(0.0f, finiteOr(in.mKickerMass, 75.0f))
                * std::clamp(finiteOr(s.mBodyBase, 0.0f) + finiteOr(s.mBodySkill, 0.0f) * skill, 0.0f, 0.8f);
        const float carryShare = std::max(0.0f, finiteOr(s.mCarry, 0.0f));
        const float carryCap = std::max(0.0f, finiteOr(s.mCarryCap, 0.0f));
        if (carryShare > 0.0f && carryCap > 0.0f)
        {
            const float victim = std::max(20.0f, finiteOr(in.mVictimMass, 75.0f));
            out.mCarry = std::min(carryCap, carryShare * out.mSpeed * out.mMass / victim);
            out.mRecoil = std::clamp(finiteOr(s.mRecoil, 0.0f), 0.0f, 1.0f) * out.mCarry * victim
                / std::max(20.0f, finiteOr(in.mKickerMass, 75.0f));
            const float fell = finiteOr(s.mFell, 0.0f);
            out.mFells = fell > 0.0f && !in.mGuarded && out.mCarry >= fell;
        }
        return out;
    }

    float bodyCarryShare(AnatomicalRegion region)
    {
        switch (region)
        {
            case AnatomicalRegion::LeftFoot:
            case AnatomicalRegion::RightFoot:
                return 0.2f;
            case AnatomicalRegion::LeftCalf:
            case AnatomicalRegion::RightCalf:
                return 0.4f;
            case AnatomicalRegion::LeftThigh:
            case AnatomicalRegion::RightThigh:
                return 0.75f;
            default:
                return 1.0f;
        }
    }

    float pressCarriedMass(float weaponMass, float bodyMass, float bodyAlong, float approach,
        const BlowPressSettings& s)
    {
        const float weapon = std::max(0.0f, finiteOr(weaponMass, 0.0f));
        if (!s.mOn || !(s.mBodyShare > 0.0f))
            return weapon;
        const float closing = std::max(1.0f, finiteOr(approach, 0.0f));
        const float riding = std::clamp(finiteOr(bodyAlong, 0.0f) / closing, 0.0f, 1.0f);
        const float mass = weapon + s.mBodyShare * std::max(0.0f, finiteOr(bodyMass, 0.0f)) * riding;
        return std::min(std::max(weapon, finiteOr(s.mMassCap, weapon)), mass);
    }
}
