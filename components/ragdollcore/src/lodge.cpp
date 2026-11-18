#include <ragdollcore/lodge.hpp>

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

    const char* lodgeKindName(LodgeKind kind)
    {
        switch (kind)
        {
            case LodgeKind::None: return "none";
            case LodgeKind::Stab: return "stab";
            case LodgeKind::Chop: return "chop";
            case LodgeKind::Pass: return "pass";
        }
        return "none";
    }

    const char* lodgeReleaseName(LodgeRelease release)
    {
        switch (release)
        {
            case LodgeRelease::Held: return "held";
            case LodgeRelease::Out: return "out";
            case LodgeRelease::Time: return "time";
            case LodgeRelease::Torn: return "torn";
        }
        return "held";
    }

    LodgePlan planLodge(const LodgeInput& input, const LodgeSettings& s)
    {
        LodgePlan plan;
        const float damage = finiteOr(input.mDamage, 0.0f);
        if (!(damage > 0.0f) || input.mOutcome == BodyContactOutcome::Glance)
            return plan;
        const float hardness = std::clamp(finiteOr(input.mHardness, 0.0f), 0.0f, 1.0f);
        const float vn = std::max(0.0f, finiteOr(input.mNormalSpeed, 0.0f));
        const float vt = std::max(0.0f, finiteOr(input.mTangentSpeed, 0.0f));
        if (input.mKind == BlowKind::Point)
        {
            // A point that was paid for went in, through whatever it met.
            if (!s.mStab)
                return plan;
            plan.mKind = LodgeKind::Stab;
            plan.mDepth = std::clamp(damage * s.mStabDepthPerDamage, s.mStabDepthMin, s.mStabDepthMax);
            plan.mHold = s.mStabHold;
            return plan;
        }
        if (input.mKind != BlowKind::Edge || hardness >= s.mHardFrom)
            return plan;
        const bool drawn = vn < s.mChopRatio * vt;
        if (input.mOutcome == BodyContactOutcome::Cleave || (input.mOutcome == BodyContactOutcome::Bite && drawn))
        {
            if (!s.mPass)
                return plan;
            plan.mKind = LodgeKind::Pass;
            plan.mHold = s.mPassTime;
            plan.mDragImpulse = std::min(s.mPassDragCap,
                std::max(0.0f, finiteOr(input.mCarriedMass, 0.0f)) * vt * std::max(0.0f, s.mPassDrag));
            return plan;
        }
        if (input.mOutcome == BodyContactOutcome::Bite && s.mChop)
        {
            plan.mKind = LodgeKind::Chop;
            plan.mDepth = std::clamp(damage * s.mChopDepthPerDamage, s.mChopDepthMin, s.mChopDepthMax);
            plan.mHold = s.mChopHold;
        }
        return plan;
    }

    LodgeStep stepLodge(const LodgeStepInput& input, const LodgeSettings& s)
    {
        LodgeStep step;
        const float dt = input.mDt > 1e-5f ? input.mDt : 1.0f / 60.0f;
        const float mass = std::max(0.0f, finiteOr(input.mEffectiveMass, 0.0f));
        JPH::Vec3 axis = input.mAxis;
        if (!(axis.LengthSq() > 1e-6f) || axis.IsNaN() || input.mOffset.IsNaN() || input.mRelativeVelocity.IsNaN())
        {
            step.mRelease = LodgeRelease::Torn;
            return step;
        }
        axis /= axis.Length();
        step.mDepth = input.mOffset.Dot(axis);
        const JPH::Vec3 off = input.mOffset - axis * step.mDepth;
        step.mOffLine = off.Length();
        if (step.mOffLine > s.mTearDistance)
            step.mRelease = LodgeRelease::Torn;
        else if (input.mAge >= input.mHold)
            step.mRelease = LodgeRelease::Time;
        else if (step.mDepth < -1.5f && input.mAge > 0.05f)
            step.mRelease = LodgeRelease::Out;
        if (step.mRelease != LodgeRelease::Held)
            return step;

        const float along = input.mRelativeVelocity.Dot(axis);
        const JPH::Vec3 across = input.mRelativeVelocity - axis * along;
        // Sideways: back onto the line, a share of the error a step, and the
        // sideways speed taken out.
        JPH::Vec3 impulse = -(off * (std::clamp(s.mPinPull, 0.0f, 1.0f) / dt) + across) * mass;
        // Along the line: friction, more of it the deeper it sits; past the
        // depth it earned, a stop.
        const float friction = (s.mFriction + s.mFrictionPerDepth * std::max(0.0f, step.mDepth)) * dt;
        float alongImpulse = -std::clamp(along * mass, -friction, friction);
        if (step.mDepth > input.mDepthMax)
            alongImpulse = -(std::clamp(s.mPinPull, 0.0f, 1.0f) * (step.mDepth - input.mDepthMax) / dt
                               + std::max(0.0f, along))
                * mass;
        impulse += axis * alongImpulse;
        const float most = std::max(0.0f, s.mPinForce) * dt;
        if (const float length = impulse.Length(); length > most && length > 0.0f)
            impulse *= most / length;
        step.mImpulse = impulse;
        return step;
    }
}
