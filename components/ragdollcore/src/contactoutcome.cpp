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
}
