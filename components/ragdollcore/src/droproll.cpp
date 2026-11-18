#include <ragdollcore/droproll.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    float dropRollTheta(JPH::Vec3Arg axis, JPH::Vec3Arg v)
    {
        const JPH::Vec3 e = axis.Cross(JPH::Vec3::sAxisZ()).NormalizedOr(JPH::Vec3::sAxisY());
        return std::atan2(v.Dot(e), v.Dot(JPH::Vec3::sAxisZ()));
    }

    DropRollStep stepDropRoll(DropRollState& state, const DropRollInput& input, const DropRollSettings& settings)
    {
        DropRollStep out;
        const float dt = input.mDt;
        const auto ease = [dt](float& value, float wanted, float seconds) {
            value += std::clamp(wanted - value, -dt / seconds, dt / seconds);
        };
        // The measured angle, unwrapped.
        const float raw = dropRollTheta(input.mAxis, input.mPelvisUp);
        float delta = raw - state.mThetaRawLast;
        delta = std::remainder(delta, 6.2831853f);
        state.mThetaRawLast = raw;
        state.mTheta += delta;
        state.mRate = 0.5f * state.mRate + 0.5f * (dt > 0.0f ? delta / dt : 0.0f);
        state.mTwistTime = std::abs(input.mTorsoUp.Dot(input.mAxis)) > settings.mTwist ? state.mTwistTime + dt : 0.0f;
        state.mStallTime = state.mPhase == RootRollPhase::Roll && state.mRate < 1.5f && state.mTheta < 4.4f
            ? state.mStallTime + dt
            : 0.0f;
        // The target turns at a rolling ball's rate (at most 8 rad/s: at 9-10
        // the spin flung the tucked legs out straight, a handspring, 10-13),
        // never more than `lead` ahead of the measured angle, never behind it.
        const float rate = std::clamp(
            (input.mAlong + 0.15f * std::max(0.0f, input.mImpact - 300.0f)) / std::max(settings.mRadius, 1.0f),
            std::min(5.0f, input.mRateCap), input.mRateCap);
        const float lead = settings.mLeadDeg * 3.14159265f / 180.0f;
        const float leadShare
            = state.mPhase == RootRollPhase::Absorb ? std::clamp(state.mPhaseTime / 0.12f, 0.0f, 1.0f) : 1.0f;
        float target = std::clamp(state.mTarget + rate * dt, state.mTheta, state.mTheta + lead * leadShare);
        if (input.mToBack)
            target = std::min(target, 2.62f);
        if (state.mPhase == RootRollPhase::Rise)
            target = std::min(target, 6.2831853f);
        state.mTarget = target;
        // The fades.
        if (state.mPhase == RootRollPhase::Rise)
        {
            ease(state.mTuck, 0.0f, 0.2f);
            ease(state.mPosture, 0.0f, 0.3f);
        }
        else
        {
            ease(state.mTuck, 1.0f, 0.1f);
            state.mPosture = 1.0f;
        }
        // The hands to the ground ahead until 70 deg, then let go.
        if (!state.mHandsReleased && state.mTheta >= 1.22f)
        {
            state.mHandsReleased = true;
            out.mReleaseHands = true;
        }
        else if (!state.mHandsReleased && state.mPhase != RootRollPhase::Rise)
            out.mPlantHands = true;
        // Transitions.
        switch (state.mPhase)
        {
            case RootRollPhase::Absorb:
                if (state.mPhaseTime >= 0.12f || state.mTheta >= 0.52f)
                {
                    state.mPhase = RootRollPhase::Roll;
                    state.mPhaseTime = 0.0f;
                    out.mToRoll = true;
                }
                break;
            case RootRollPhase::Roll:
                if (input.mToBack && (state.mTheta >= 2.44f || state.mPhaseTime > 0.6f))
                    out.mEnd = "back", out.mFall = true;
                else if (state.mStallTime >= 0.25f)
                    out.mEnd = "stalled", out.mFall = true;
                else if (state.mTwistTime >= 0.25f)
                    out.mEnd = "twist", out.mFall = true;
                else if (state.mTotal > settings.mTimeout)
                    out.mEnd = "timeout", out.mFall = true;
                else if (state.mTheta >= 5.06f)
                {
                    if (input.mAlong >= 50.0f)
                    {
                        state.mPhase = RootRollPhase::Rise;
                        state.mPhaseTime = 0.0f;
                        out.mToRise = true;
                    }
                    else
                        out.mEnd = "slow", out.mFall = true;
                }
                break;
            case RootRollPhase::Rise:
                if (state.mPhaseTime >= 0.2f && input.mTorsoUp.GetZ() >= 0.8f && input.mFootSupported)
                    out.mEnd = "stand", out.mFall = false;
                else if (state.mPhaseTime >= 0.6f)
                {
                    out.mEnd = input.mTorsoUp.GetZ() >= 0.6f ? "stand" : "rise";
                    out.mFall = input.mTorsoUp.GetZ() < 0.6f;
                }
                else if (state.mTotal > settings.mTimeout + 0.6f)
                    out.mEnd = "timeout", out.mFall = true;
                break;
            default:
                break;
        }
        return out;
    }

    void dropRollRootPose(const DropRollRootInput& in, JPH::RVec3& position, JPH::Quat& rotation)
    {
        if (in.mPhase == RootRollPhase::Rise && in.mTarget >= 6.28f)
            rotation = in.mBase;
        else
            rotation = (JPH::Quat::sRotation(in.mAxis, in.mTarget) * in.mBase).Normalized();
        const auto smooth = [](float t) {
            t = std::clamp(t, 0.0f, 1.0f);
            return t * t * (3.0f - 2.0f * t);
        };
        if (in.mPhase == RootRollPhase::Absorb)
            position.SetZ(position.GetZ() - std::min(30.0f, 0.04f * in.mImpact) * smooth(in.mPhaseTime / 0.12f));
        else if (in.mPhase == RootRollPhase::Roll && in.mPelvisZ)
            position.SetZ(*in.mPelvisZ);
        else if (in.mPhase == RootRollPhase::Rise)
            position.SetZ(in.mRiseZ + (position.GetZ() - in.mRiseZ) * smooth(in.mPhaseTime / 0.4f));
        if (in.mKeep > 0.0f
            && (in.mPhase == RootRollPhase::Roll || (in.mPhase == RootRollPhase::Rise && in.mPhaseTime < 0.3f)))
        {
            const float lack = std::max(0.0f, in.mKeep * in.mEntrySpeed - in.mAlong);
            position += JPH::RVec3(in.mTravel * (0.15f * lack));
        }
    }
}
