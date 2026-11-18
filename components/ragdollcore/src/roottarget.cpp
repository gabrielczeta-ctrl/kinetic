#include <ragdollcore/roottarget.hpp>

#include <ragdollcore/tuning.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace RagdollCore
{
    float applyRunBob(RunBobState& state, float z, JPH::Vec3Arg request, bool footfall, const RunBobSettings& settings,
        float dt)
    {
        if (!state.mMean)
            state.mMean = z;
        *state.mMean += (z - *state.mMean) * std::min(1.0f, dt / 0.35f);
        const JPH::Vec3 asked(request.GetX(), request.GetY(), 0.0f);
        const float run = settings.mSpeed > 0.0f && footfall
            ? std::clamp((asked.Length() - 0.6f * settings.mSpeed) / (0.4f * settings.mSpeed), 0.0f, 1.0f)
            : 0.0f;
        state.mShare += (run - state.mShare) * std::min(1.0f, dt * 6.0f);
        if (asked.Length() > 1.0f && dt > 0.0f)
        {
            const float heading = std::atan2(asked.GetX(), asked.GetY());
            if (state.mHeading)
            {
                float turn = heading - *state.mHeading;
                turn = std::atan2(std::sin(turn), std::cos(turn));
                const float rate = std::abs(turn) / dt;
                if (std::isfinite(rate))
                    state.mTurnRate += (rate - state.mTurnRate) * std::min(1.0f, dt * 4.0f);
            }
            state.mHeading = heading;
        }
        const float straight = std::clamp(1.0f - state.mTurnRate / 1.5f, 0.0f, 1.0f);
        if (state.mShare > 1e-3f && std::isfinite(*state.mMean))
            return *state.mMean + (z - *state.mMean) * (1.0f - (1.0f - settings.mKeep) * state.mShare)
                - settings.mSink * state.mShare * straight;
        return z;
    }

    JPH::RVec3 rootTargetPosition(const RootTargetInput& in)
    {
        return in.mActor + JPH::RVec3(in.mOffset) - JPH::RVec3(0.0f, 0.0f, in.mDrop)
            - JPH::RVec3(in.mTeeterToward * (in.mTeeter * in.mTeeterHips))
            - JPH::RVec3(0.0f, 0.0f, std::max(0.0f, in.mTeeter - 0.4f) * 20.0f) + JPH::RVec3(in.mRecoil);
    }

    JPH::RVec3 followRootTarget(RootFollowerState& state, JPH::RVec3Arg target, float maximumAcceleration, float dt)
    {
        JPH::Vec3 capsuleVelocity = JPH::Vec3::sZero();
        if (state.mPrevious && dt > 0.0f)
            capsuleVelocity = JPH::Vec3(target - *state.mPrevious) / dt;
        capsuleVelocity.SetZ(0.0f);
        state.mPrevious = target;
        if (!state.mFollower || capsuleVelocity.Length() > 2000.0f || maximumAcceleration <= 0.0f)
        {
            state.mFollower = target;
            state.mVelocity = capsuleVelocity;
        }
        else
        {
            JPH::Vec3 lag(target - *state.mFollower);
            lag.SetZ(0.0f);
            const JPH::Vec3 desired = capsuleVelocity + lag * 8.0f;
            JPH::Vec3 change = desired - state.mVelocity;
            const float limit = maximumAcceleration * dt;
            if (change.Length() > limit)
                change = change.Normalized() * limit;
            state.mVelocity += change;
            JPH::RVec3 next = *state.mFollower + JPH::RVec3(state.mVelocity * dt);
            next.SetZ(target.GetZ());
            state.mFollower = next;
        }
        return *state.mFollower;
    }

    JPH::Vec3 rootYieldVelocity(
        JPH::RVec3Arg pelvis, JPH::RVec3Arg target, float deadZone, float gain, float maximumSpeed)
    {
        JPH::Vec3 offset(pelvis - target);
        offset.SetZ(0.0f);
        const float excess = offset.Length() - deadZone;
        return excess > 0.0f ? offset.Normalized() * std::min(excess * gain, maximumSpeed) : JPH::Vec3::sZero();
    }

    GaitCapSettings gaitCapSettings()
    {
        std::array<float, 2> values{ 0.0f, 0.0f };
        tuningList("OPENMW_RAGDOLL_K2_GAIT_CAP", values);
        GaitCapSettings settings;
        settings.mSpeed = std::isfinite(values[0]) ? std::clamp(values[0], 0.0f, 5000.0f) : 0.0f;
        settings.mPlayer = std::isfinite(values[1]) && values[1] >= 0.5f;
        return settings;
    }

    NpcStartSettings npcStartSettings()
    {
        std::array<float, 2> values{ 0.0f, 0.0f };
        tuningList("OPENMW_RAGDOLL_K2_NPC_START", values);
        NpcStartSettings settings;
        settings.mAcceleration = std::isfinite(values[0]) ? std::clamp(values[0], 0.0f, 20000.0f) : 0.0f;
        settings.mTurnRate = std::isfinite(values[1]) ? std::clamp(values[1], 0.0f, 100.0f) : 0.0f;
        return settings;
    }

    MoveAccelSettings moveAccelSettings()
    {
        std::array<float, 2> values{ 0.0f, 0.0f };
        tuningList("OPENMW_RAGDOLL_K2_MOVE_ACCEL", values);
        MoveAccelSettings settings;
        settings.mLow = std::isfinite(values[0]) ? std::clamp(values[0], 0.0f, 20000.0f) : 0.0f;
        settings.mHigh = std::isfinite(values[1]) ? std::clamp(values[1], settings.mLow, 20000.0f) : settings.mLow;
        return settings;
    }

    float moveAcceleration(const MoveAccelSettings& settings, float base, float speedAttribute)
    {
        if (settings.mLow <= 0.0f || !std::isfinite(speedAttribute))
            return base;
        return settings.mLow + (settings.mHigh - settings.mLow) * std::clamp(speedAttribute / 100.0f, 0.0f, 1.0f);
    }

    float npcStartSpeed(float current, float target, bool moving, float acceleration, float dt)
    {
        if (!moving)
            return 0.0f;
        return std::min(target, std::max(0.0f, current) + acceleration * std::max(0.0f, dt));
    }

    ArmStepSettings armStepSettings()
    {
        std::array<float, 2> values{ 0.0f, 0.5f };
        tuningList("OPENMW_RAGDOLL_K2_ARM_STEP", values);
        ArmStepSettings settings;
        settings.mOn = std::isfinite(values[0]) && values[0] >= 0.5f;
        settings.mGain = std::isfinite(values[1]) ? std::clamp(values[1], 0.0f, 2.0f) : 0.5f;
        return settings;
    }

    float stepArmStep(ArmStepState& state, float leftAhead, float cycle, float clock, float duty, float gain, float dt)
    {
        if (!(cycle > 0.0f) || !std::isfinite(leftAhead))
        {
            state = ArmStepState{};
            return 0.0f;
        }
        const float delta = leftAhead - state.mLastAhead;
        if (state.mHave)
        {
            // The clip's foot furthest ahead is that foot's landing; the
            // legs land the left at clock 1 - duty, the right half a cycle on.
            std::optional<float> at;
            if (state.mLastDelta > 0.0f && delta <= 0.0f && state.mLastAhead > 0.0f)
                at = 1.0f - duty;
            else if (state.mLastDelta < 0.0f && delta >= 0.0f && state.mLastAhead < 0.0f)
                at = 1.5f - duty;
            if (at)
            {
                float error = std::fmod(clock - *at + 2.0f, 1.0f);
                if (error > 0.5f)
                    error -= 1.0f;
                state.mError = error;
                state.mWanted = std::clamp(1.0f + gain * error, 0.75f, 1.25f);
                ++state.mEvents;
            }
            state.mLastDelta = delta;
        }
        state.mLastAhead = leftAhead;
        state.mHave = true;
        state.mFactor += (state.mWanted - state.mFactor) * std::min(1.0f, 6.0f * std::max(0.0f, dt));
        return cycle / state.mFactor;
    }

    RootFieldSettings rootFieldSettings()
    {
        std::array<float, 2> values{ 0.0f, 0.0f };
        tuningList("OPENMW_RAGDOLL_K2_ROOT_FIELD", values);
        RootFieldSettings settings;
        settings.mFrom = std::isfinite(values[0]) ? std::clamp(values[0], 0.0f, 5000.0f) : 0.0f;
        settings.mFull = std::isfinite(values[1]) ? std::clamp(values[1], 0.0f, 5000.0f) : 0.0f;
        return settings;
    }

    float rootFieldShare(const RootFieldSettings& settings, float speed)
    {
        if (!(settings.mFrom > 0.0f) || !std::isfinite(speed))
            return 0.0f;
        if (settings.mFull <= settings.mFrom)
            return speed >= settings.mFrom ? 1.0f : 0.0f;
        const float k = std::clamp((speed - settings.mFrom) / (settings.mFull - settings.mFrom), 0.0f, 1.0f);
        return k * k * (3.0f - 2.0f * k);
    }

    JPH::Vec3 capGaitSpeed(JPH::Vec3Arg velocity, float cap)
    {
        const float speed = std::hypot(velocity.GetX(), velocity.GetY());
        if (cap <= 0.0f || speed <= cap)
            return velocity;
        const float share = cap / speed;
        return JPH::Vec3(velocity.GetX() * share, velocity.GetY() * share, velocity.GetZ());
    }
}
