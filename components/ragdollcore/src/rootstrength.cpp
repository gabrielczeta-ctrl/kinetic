#include <ragdollcore/rootstrength.hpp>

#include <ragdollcore/recoverycontrol.hpp>

#include <algorithm>

namespace RagdollCore
{
    namespace
    {
        float ramp(const std::array<float, 2>& settings, float elapsed)
        {
            return settings[1] > 0.0f
                ? settings[0] + (1.0f - settings[0]) * std::clamp(elapsed / settings[1], 0.0f, 1.0f)
                : 1.0f;
        }

        float smoothstep(float k)
        {
            return k * k * (3.0f - 2.0f * k);
        }
    }

    float recoverRootStrength(float strength, float perSecond, float weaponControl, float recoverySkill, float dt)
    {
        const float scale
            = 1.0f - std::clamp(recoverySkill, 0.0f, 0.9f) * (1.0f - std::clamp(weaponControl, 0.2f, 1.0f)) / 0.8f;
        return std::min(1.0f, strength + perSecond * scale * dt);
    }

    RootStrength updateRootStrength(
        const RootStrengthInput& input, const RootStrengthSettings& settings, RootStrengthState& state, float dt)
    {
        RootStrength r;
        state.mGetUpElapsed = input.mRecoveryOwns ? state.mGetUpElapsed + dt : 0.0f;
        r.mGetUpRamp = ramp(settings.mGetUpRamp, state.mGetUpElapsed);
        state.mReholdElapsed += dt;
        r.mReholdRamp = ramp(settings.mGetUpRamp, state.mReholdElapsed);
        r.mHeld = std::min(input.mHitStrength, r.mReholdRamp);

        // The budget. The body's own speed counts as moving too: a body still
        // running after the keys are released brakes on the moving budget,
        // not the standing one (offline every stop from a run fell).
        // Braking holds harder (user 10-30: "less drifting", "change
        // direction faster"); a firmer moving budget everywhere leaned the
        // hill course's end 11 -> 16 deg, so only braking takes it.
        RootBudgetInput budget{ r.mHeld, input.mRequestSpeed, input.mBodySpeed, {} };
        const bool braking = input.mBodySpeed > input.mRequestSpeed + 20.0f;
        budget.mSettings.mStanding = std::clamp(settings.mBudgetShares[0], 0.0f, 1.0f);
        budget.mSettings.mMoving
            = std::clamp(braking ? settings.mBudgetShares[2] : settings.mBudgetShares[1], 0.0f, 1.0f);
        state.mHorizontalBudget = 1.0f;
        float strength = r.mHeld;
        r.mBudgeted = strength;
        if (settings.mBudget && !input.mRecoveryOwns)
        {
            if (settings.mMoveVertical > 0.0f
                && std::max(input.mRequestSpeed, input.mBodySpeed) > settings.mMoveVertical)
            {
                state.mHorizontalBudget = budget.mSettings.mMoving;
                r.mMovingVertical = true;
            }
            else
                strength = rootBudgetStrength(budget);
            strength = std::max(strength, std::min(r.mHeld, input.mLandingFloor));
            if (settings.mClimbHold > 0.0f && !input.mAirborne && input.mVerticalVelocity > settings.mClimbHold)
            {
                strength = r.mHeld;
                state.mHorizontalBudget = 1.0f;
                r.mClimbing = true;
            }
            if (input.mLandingFloor > 0.0f)
                state.mHorizontalBudget = std::max(state.mHorizontalBudget, input.mLandingFloor);
            // Uneven ground (K2_UNEVEN): the further the pelvis hangs under
            // its target, the more of the held strength the lift gets back
            // (the budgeted strength scales the weight support and the upward
            // cap too: a walk up a slope had 0.3 of them). The horizontal
            // hold keeps its budget.
            if (input.mUneven && settings.mSagHold[1] > settings.mSagHold[0] && strength < r.mHeld)
            {
                const float k = std::clamp(
                    (input.mSag - settings.mSagHold[0]) / (settings.mSagHold[1] - settings.mSagHold[0]), 0.0f, 1.0f);
                if (k > 0.0f)
                {
                    const float raised = strength + (r.mHeld - strength) * k;
                    state.mHorizontalBudget = std::min(state.mHorizontalBudget, strength / std::max(raised, 1e-3f));
                    strength = raised;
                    r.mSagHold = k;
                }
            }
            r.mBudgeted = strength;
            r.mTiltShare = updateTiltHold(state.mTilt, input.mTilt, settings.mTiltHold, dt);
            if (r.mTiltShare > 0.0f)
            {
                strength = std::max(strength, strength + (r.mHeld - strength) * r.mTiltShare);
                state.mHorizontalBudget = std::max(state.mHorizontalBudget, r.mTiltShare);
            }
        }
        r.mHorizontalBudget = state.mHorizontalBudget;

        float rootStrength = input.mRecoveryOwns
            ? std::min(input.mHitStrength, settings.mGetUpStrength * r.mGetUpRamp)
            : strength;
        // The drop roll: the root's authority is its strength.
        if (input.mRoll != RootRollPhase::None)
        {
            const float roll = settings.mRollStrength;
            const float k = std::clamp(
                input.mRollPhaseTime / (input.mRoll == RootRollPhase::Rise ? 0.4f : 0.12f), 0.0f, 1.0f);
            rootStrength = input.mRoll == RootRollPhase::Absorb ? rootStrength + (roll - rootStrength) * k
                : input.mRoll == RootRollPhase::Roll            ? roll
                                                                : roll + (rootStrength - roll) * smoothstep(k);
        }
        // The calm hold (K2_CALM): a calm NPC's root holds it to the animation.
        const float calm = input.mRecoveryOwns || input.mRoll != RootRollPhase::None || !(settings.mCalmStrength > 0.0f)
            ? 0.0f
            : std::clamp(input.mCalm, 0.0f, 1.0f);
        if (calm > 0.0f)
            rootStrength += (std::max(rootStrength, std::min(r.mHeld, settings.mCalmStrength)) - rootStrength) * calm;
        r.mStrength = rootStrength;

        // The horizontal hold: a catch step carries the push, a landing
        // drags the pelvis less along the floor, a stumble lets the blow's
        // push carry the body (PGC_FALLS).
        float horizontal = input.mCatchRecovering && !input.mRecoveryOwns ? settings.mCatchHold : 1.0f;
        if (input.mLanding)
            horizontal = std::min(horizontal, settings.mLandingHold);
        if (input.mStumble > 0.0f)
            horizontal = std::min(horizontal, 1.0f - input.mStumble);
        r.mHorizontalHold = horizontal;
        float rootHorizontal = std::min(horizontal, state.mHorizontalBudget);
        if (input.mLegsLeading)
            rootHorizontal = std::min(rootHorizontal, std::max(0.0f, settings.mLegsLeadShare));
        if (input.mRoll != RootRollPhase::None)
        {
            const float k = std::clamp(input.mRollPhaseTime / 0.4f, 0.0f, 1.0f);
            rootHorizontal = input.mRoll == RootRollPhase::Absorb ? 0.3f
                : input.mRoll == RootRollPhase::Roll              ? 0.0f
                                                                  : smoothstep(k);
        }
        if (calm > 0.0f)
            rootHorizontal += (1.0f - rootHorizontal) * calm;
        r.mHorizontal = rootHorizontal;
        return r;
    }

    float updateRootSupport(
        const RootSupportInput& input, const RootSupportSettings& settings, RootSupportState& state, float dt)
    {
        const bool grounded = !settings.mAirFade || input.mGroundContact;
        state.mAir = std::clamp(state.mAir + (grounded ? 1.0f : -1.0f) * dt / 0.1f, 0.0f, 1.0f);
        float limp = 1.0f;
        if (settings.mLimpBudget > 0.0f && input.mFootfall)
            for (std::size_t side = 0; side < 2; ++side)
                if (input.mPinned[side] && !input.mPinned[1 - side])
                    limp = 1.0f - std::clamp(settings.mLimpBudget, 0.0f, 0.8f) * input.mWound[side];
        state.mLimp += (limp - state.mLimp) * std::clamp(dt * 10.0f, 0.0f, 1.0f);
        // The walk's lighter lift is for level ground: on a climb the legs
        // lose their load and the body hangs under its target.
        const bool walking = input.mRequestSpeed >= 15.0f && input.mRequestSpeed <= 150.0f && !input.mAirborne
            && input.mGroundContact && !input.mUneven;
        const float walkWant = walking ? std::clamp(settings.mWalkSupport, 0.0f, 1.0f) : 1.0f;
        state.mWalk += std::clamp(walkWant - state.mWalk, -dt / 0.3f, dt / 0.3f);
        float support = state.mAir * state.mLimp * state.mWalk;
        support += (state.mAir - support) * std::clamp(input.mCalm, 0.0f, 1.0f);
        if (input.mRoll != RootRollPhase::None)
        {
            const float roll = settings.mRollSupport;
            const float k = std::clamp(input.mRollPhaseTime / 0.4f, 0.0f, 1.0f);
            support = input.mRoll == RootRollPhase::Absorb ? 1.0f
                : input.mRoll == RootRollPhase::Roll       ? roll
                                                           : roll + (support - roll) * smoothstep(k);
        }
        return support;
    }
}
