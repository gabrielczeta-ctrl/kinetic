#include <ragdollcore/balancehelper.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace RagdollCore
{
    namespace
    {
        BalanceVec2 operator+(BalanceVec2 a, BalanceVec2 b) { return { a.mX + b.mX, a.mY + b.mY }; }
        BalanceVec2 operator-(BalanceVec2 a, BalanceVec2 b) { return { a.mX - b.mX, a.mY - b.mY }; }
        BalanceVec2 operator*(BalanceVec2 a, float s) { return { a.mX * s, a.mY * s }; }
        float dot(BalanceVec2 a, BalanceVec2 b) { return a.mX * b.mX + a.mY * b.mY; }
        float length(BalanceVec2 a) { return std::sqrt(dot(a, a)); }

        bool finite(float value) { return std::isfinite(value); }
        bool finite(BalanceVec2 v) { return finite(v.mX) && finite(v.mY); }

        // Shortens `v` to at most `limit`.
        BalanceVec2 clampLength(BalanceVec2 v, float limit)
        {
            const float l = length(v);
            return l > limit && l > 0.0f ? v * (limit / l) : v;
        }

        bool validLimits(const CaptureStepLimits& limits)
        {
            return finite(limits.mFootRadius) && limits.mFootRadius >= 0.0f && finite(limits.mMaximumReach)
                && limits.mMaximumReach > 0.0f && finite(limits.mMinimumWidth) && limits.mMinimumWidth >= 0.0f
                && limits.mMinimumWidth < limits.mMaximumReach && finite(limits.mMinimumSwingTime)
                && limits.mMinimumSwingTime > 0.0f && finite(limits.mMaximumSwingTime)
                && limits.mMaximumSwingTime >= limits.mMinimumSwingTime && finite(limits.mSwingFootSpeed)
                && limits.mSwingFootSpeed > 0.0f && finite(limits.mCrossoverClearance)
                && limits.mCrossoverClearance >= 0.0f && limits.mCrossoverClearance < limits.mMaximumReach
                && finite(limits.mBrakeMargin) && limits.mBrakeMargin >= 0.0f;
        }

        struct Landing
        {
            BalanceVec2 mPoint;
            bool mReachClamped = false;
            bool mWidthClamped = false;
            bool mKeptPreferred = false;
        };

        // Distance from p to segment [a, b].
        float segmentDistance(BalanceVec2 p, BalanceVec2 a, BalanceVec2 b, BalanceVec2& nearest)
        {
            const BalanceVec2 ab = b - a;
            const float span = dot(ab, ab);
            const float t = span > 0.0f ? std::clamp(dot(p - a, ab) / span, 0.0f, 1.0f) : 0.0f;
            nearest = a + ab * t;
            return length(p - nearest);
        }
        Landing chooseLanding(
            const CaptureStepInput& input, BalanceVec2 forward, BalanceVec2 predicted, float outwardBias)
        {
            Landing result;
            const auto& limits = input.mLimits;
            const BalanceVec2 right{ forward.mY, -forward.mX };
            const BalanceVec2 stance = input.mStanceFoot;
            const float side = input.mSwingIsLeft ? -1.0f : 1.0f;
            BalanceVec2 target = predicted + right * (side * outwardBias);
            if (input.mPreferredLanding)
            {
                const BalanceVec2 error = predicted - *input.mPreferredLanding;
                const float distance = length(error);
                if (distance <= input.mPreferredTolerance)
                {
                    target = *input.mPreferredLanding;
                    result.mKeptPreferred = true;
                }
                else
                    target = *input.mPreferredLanding + error * (1.0f - input.mPreferredTolerance / distance);
            }
            // The swing foot lands on its own side of the stance foot, at
            // least the minimum width out (no crossed legs).
            const float widthLine = side * limits.mMinimumWidth;
            const float lineReach = std::sqrt(
                limits.mMaximumReach * limits.mMaximumReach - limits.mMinimumWidth * limits.mMinimumWidth);
            const auto onWidthLine = [&](float along) { return stance + right * widthLine + forward * along; };
            if (dot(target - stance, right) * side < limits.mMinimumWidth)
            {
                result.mWidthClamped = true;
                // On the width line, the landing slides fore or aft toward
                // the capture point. When some landing on this line captures,
                // take the shortest one that does; otherwise stay level with
                // the capture point and leave the rest to the other leg (a
                // longer step would put the capture point behind this foot).
                const float base = std::clamp(dot(target - stance, forward), -lineReach, lineReach);
                const float goal = std::clamp(dot(predicted - stance, forward), -lineReach, lineReach);
                const float end = goal >= base ? lineReach : -lineReach;
                constexpr int samples = 32;
                float chosen = goal;
                BalanceVec2 nearest;
                for (int i = 0; i <= samples; ++i)
                {
                    const float along = base + (end - base) * static_cast<float>(i) / samples;
                    if (segmentDistance(predicted, stance, onWidthLine(along), nearest) <= limits.mFootRadius)
                    {
                        chosen = along;
                        break;
                    }
                }
                target = onWidthLine(chosen);
            }
            const BalanceVec2 reach = target - stance;
            if (length(reach) > limits.mMaximumReach)
            {
                result.mReachClamped = true;
                target = stance + clampLength(reach, limits.mMaximumReach);
                // Shortening toward the stance foot may cross the width line.
                if (dot(target - stance, right) * side < limits.mMinimumWidth)
                {
                    result.mWidthClamped = true;
                    target = onWidthLine(dot(reach, forward) >= 0.0f ? lineReach : -lineReach);
                }
            }
            if (result.mWidthClamped || result.mReachClamped)
                result.mKeptPreferred = false;
            result.mPoint = target;
            return result;
        }

        struct Crossover
        {
            BalanceVec2 mLanding;
            float mPathLength = 0.0f;
        };

        // A crossover landing: across the stance foot at the capture point,
        // at least half the clearance fore (aft, falling backward) of it and
        // the whole clearance away from it, within reach. The swing goes
        // round through the waypoint the clearance fore (aft) of the stance
        // foot. Nullopt when the capture point is not across.
        std::optional<Crossover> chooseCrossoverLanding(
            const CaptureStepInput& input, BalanceVec2 forward, BalanceVec2 predicted)
        {
            const auto& limits = input.mLimits;
            const float clearance = limits.mCrossoverClearance;
            const BalanceVec2 right{ forward.mY, -forward.mX };
            const BalanceVec2 stance = input.mStanceFoot;
            const float side = input.mSwingIsLeft ? -1.0f : 1.0f;
            const BalanceVec2 offset = predicted - stance;
            const float across = -dot(offset, right) * side;
            if (across <= 0.0f)
                return std::nullopt;
            const float along = dot(offset, forward);
            const float direction = along >= 0.0f ? 1.0f : -1.0f;
            float fore = std::max(std::abs(along), 0.5f * clearance);
            float lateral = std::max(across, std::sqrt(std::max(0.0f, clearance * clearance - fore * fore)));
            const float reach = std::sqrt(lateral * lateral + fore * fore);
            if (reach > limits.mMaximumReach)
            {
                // Shorten, keeping the fore offset's floor.
                const float scale = limits.mMaximumReach / reach;
                fore = std::max(fore * scale, 0.5f * clearance);
                lateral = std::sqrt(std::max(0.0f, limits.mMaximumReach * limits.mMaximumReach - fore * fore));
            }
            Crossover result;
            result.mLanding = stance + right * (-side * lateral) + forward * (direction * fore);
            const BalanceVec2 waypoint = stance + forward * (direction * clearance);
            result.mPathLength = length(waypoint - input.mSwingFoot) + length(result.mLanding - waypoint);
            return result;
        }
    }

    std::optional<float> balanceTimeConstant(const BalanceState& state)
    {
        if (!finite(state.mHeight) || !finite(state.mGravity) || state.mHeight <= 0.0f || state.mGravity <= 0.0f)
            return std::nullopt;
        return std::sqrt(state.mHeight / state.mGravity);
    }

    std::optional<BalanceVec2> capturePoint(const BalanceState& state)
    {
        const auto tau = balanceTimeConstant(state);
        if (!tau || !finite(state.mCenterOfMass) || !finite(state.mVelocity))
            return std::nullopt;
        return state.mCenterOfMass + state.mVelocity * *tau;
    }

    std::optional<BalanceVec2> predictCapturePoint(const BalanceState& state, BalanceVec2 pivot, float time)
    {
        const auto tau = balanceTimeConstant(state);
        const auto capture = capturePoint(state);
        if (!tau || !capture || !finite(pivot) || !finite(time) || time < 0.0f)
            return std::nullopt;
        const BalanceVec2 result = pivot + (*capture - pivot) * std::exp(time / *tau);
        if (!finite(result))
            return std::nullopt;
        return result;
    }

    std::optional<CaptureStepPlan> planCaptureStep(const CaptureStepInput& input)
    {
        if (!validLimits(input.mLimits) || !finite(input.mStanceFoot) || !finite(input.mSwingFoot)
            || !finite(input.mBodyForward) || length(input.mBodyForward) < 1e-6f
            || (input.mTimeToTouchdown && (!finite(*input.mTimeToTouchdown) || *input.mTimeToTouchdown < 0.0f))
            || (input.mPreferredLanding
                && (!finite(*input.mPreferredLanding) || !finite(input.mPreferredTolerance)
                    || input.mPreferredTolerance < 0.0f)))
            return std::nullopt;
        const auto capture = capturePoint(input.mState);
        if (!capture)
            return std::nullopt;
        const BalanceVec2 forward = input.mBodyForward * (1.0f / length(input.mBodyForward));

        // The stance ankle holds the center of pressure as close to the
        // capture point as the foot allows, which slows the divergence.
        const BalanceVec2 pivot = input.mPressureAtFootEdge
            ? input.mStanceFoot + clampLength(*capture - input.mStanceFoot, input.mLimits.mFootRadius)
            : input.mStanceFoot;

        CaptureStepPlan plan;
        plan.mCapturePoint = *capture;
        // Fixed point of swing time <-> landing distance: a farther landing
        // takes longer, and the capture point runs further meanwhile. The
        // plan always reports the time its prediction used.
        struct Solution
        {
            float mTime = 0.0f;
            BalanceVec2 mPredicted;
            Landing mLanding;
            float mResidual = 0.0f;
        };
        const auto solve = [&](float outwardBias, bool crossover) -> std::optional<Solution> {
            Solution solution;
            solution.mTime = input.mTimeToTouchdown ? *input.mTimeToTouchdown : input.mLimits.mMinimumSwingTime;
            for (int pass = 0; pass < 6; ++pass)
            {
                const auto next = predictCapturePoint(input.mState, pivot, solution.mTime);
                if (!next)
                    return std::nullopt;
                solution.mPredicted = *next;
                float pathLength = 0.0f;
                if (crossover)
                {
                    const auto crossed = chooseCrossoverLanding(input, forward, solution.mPredicted);
                    if (!crossed)
                        return std::nullopt;
                    solution.mLanding = Landing{ crossed->mLanding, false, false, false };
                    pathLength = crossed->mPathLength;
                }
                else
                {
                    solution.mLanding = chooseLanding(input, forward, solution.mPredicted, outwardBias);
                    pathLength = length(solution.mLanding.mPoint - input.mSwingFoot);
                }
                if (input.mTimeToTouchdown)
                    break;
                const float settled = std::clamp(
                    input.mLimits.mMinimumSwingTime + pathLength / input.mLimits.mSwingFootSpeed,
                    input.mLimits.mMinimumSwingTime, input.mLimits.mMaximumSwingTime);
                if (std::abs(settled - solution.mTime) < 1e-4f)
                    break;
                solution.mTime = settled;
            }
            // After touchdown both feet carry the body, so the center of
            // pressure can sit anywhere between them: the step captures when
            // the capture point is within a foot radius of that support.
            BalanceVec2 nearest;
            solution.mResidual
                = segmentDistance(solution.mPredicted, input.mStanceFoot, solution.mLanding.mPoint, nearest);
            return solution;
        };
        auto solution = solve(0.0f, false);
        if (!solution)
            return std::nullopt;
        // A step that cannot capture lands outward of the capture point by
        // half the minimum width, so the capture point ends up between the
        // feet and the other leg can catch it next.
        if (solution->mResidual > input.mLimits.mFootRadius && !solution->mLanding.mKeptPreferred)
        {
            const auto outward = solve(0.5f * input.mLimits.mMinimumWidth, false);
            if (!outward)
                return std::nullopt;
            solution = outward;
        }
        // Crossover: only when the ordinary step cannot capture and crossing
        // brings the support at least a foot radius closer to the capture point.
        if (solution->mResidual > input.mLimits.mFootRadius && input.mLimits.mCrossoverClearance > 0.0f
            && !solution->mLanding.mKeptPreferred)
            if (const auto crossed = solve(0.0f, true);
                crossed && crossed->mResidual + input.mLimits.mFootRadius < solution->mResidual)
            {
                solution = crossed;
                plan.mCrossover = true;
            }
        // Braking: a capturing step lands past the capture point along the
        // fall (from the stance foot), when that stays within reach and still
        // captures; the capture point then lies between the feet.
        if (solution->mResidual <= input.mLimits.mFootRadius && input.mLimits.mBrakeMargin > 0.0f
            && !solution->mLanding.mKeptPreferred)
        {
            const BalanceVec2 fall = solution->mPredicted - input.mStanceFoot;
            if (length(fall) > 1e-3f)
            {
                BalanceVec2 braked = solution->mLanding.mPoint + fall * (input.mLimits.mBrakeMargin / length(fall));
                if (length(braked - input.mStanceFoot) > input.mLimits.mMaximumReach)
                    braked = input.mStanceFoot + clampLength(braked - input.mStanceFoot, input.mLimits.mMaximumReach);
                BalanceVec2 nearest;
                const float residual = segmentDistance(solution->mPredicted, input.mStanceFoot, braked, nearest);
                if (residual <= input.mLimits.mFootRadius)
                {
                    solution->mLanding.mPoint = braked;
                    solution->mResidual = residual;
                }
            }
        }
        plan.mPredictedCapturePoint = solution->mPredicted;
        plan.mLanding = solution->mLanding.mPoint;
        plan.mSwingTime = solution->mTime;
        plan.mResidual = solution->mResidual;
        plan.mCaptures = plan.mResidual <= input.mLimits.mFootRadius;
        const Landing& landing = solution->mLanding;
        plan.mReachClamped = landing.mReachClamped;
        plan.mWidthClamped = landing.mWidthClamped;
        plan.mKeptPreferred = landing.mKeptPreferred;
        if (!finite(plan.mLanding) || !finite(plan.mResidual))
            return std::nullopt;
        return plan;
    }

    namespace
    {
        // Each step the best single step the planner finds (greedy).
        std::optional<RecoveryPlan> planRecoveryGreedy(const CaptureStepInput& input, std::size_t maximumSteps)
        {
        RecoveryPlan result;
        CaptureStepInput step = input;
        const std::size_t count = std::min(maximumSteps, result.mSteps.size());
        for (std::size_t i = 0; i < count; ++i)
        {
            const auto plan = planCaptureStep(step);
            if (!plan)
                return std::nullopt;
            RecoveryStep& out = result.mSteps[result.mCount++];
            out.mLanding = plan->mLanding;
            out.mSwingTime = plan->mSwingTime;
            out.mLength = length(plan->mLanding - step.mSwingFoot);
            out.mIsLeft = step.mSwingIsLeft;
            out.mResidual = plan->mResidual;
            out.mCaptures = plan->mCaptures;
            out.mCrossover = plan->mCrossover;
            if (plan->mCaptures)
            {
                result.mCaptured = true;
                break;
            }
            // The capture point alone carries the pendulum's unstable state:
            // a body at rest over it has the same future capture point, so
            // the next step plans from there with the new stance foot.
            step.mState.mCenterOfMass = plan->mPredictedCapturePoint;
            step.mState.mVelocity = {};
            step.mSwingFoot = step.mStanceFoot;
            step.mStanceFoot = plan->mLanding;
            step.mSwingIsLeft = !step.mSwingIsLeft;
            step.mTimeToTouchdown.reset();
            step.mPreferredLanding.reset();
        }
        return result;
        }
    }

    std::optional<RecoveryPlan> planRecoverySteps(const CaptureStepInput& input, std::size_t maximumSteps)
    {
        if (maximumSteps == 0)
            return std::nullopt;
        const auto withCrossovers = planRecoveryGreedy(input, maximumSteps);
        if (input.mLimits.mCrossoverClearance <= 0.0f || !withCrossovers)
            return withCrossovers;
        // A crossover that helps one step can leave the next one worse off:
        // keep the sequence without crossovers when it catches in as few
        // steps, or catches where the other does not.
        CaptureStepInput plain = input;
        plain.mLimits.mCrossoverClearance = 0.0f;
        const auto without = planRecoveryGreedy(plain, maximumSteps);
        if (!without)
            return withCrossovers;
        const auto better = [](const RecoveryPlan& a, const RecoveryPlan& b) {
            if (a.mCaptured != b.mCaptured)
                return a.mCaptured;
            if (a.mCount != b.mCount)
                return a.mCaptured ? a.mCount < b.mCount : false;
            return a.mSteps[a.mCount - 1].mResidual < b.mSteps[b.mCount - 1].mResidual;
        };
        return better(*withCrossovers, *without) ? withCrossovers : without;
    }

    std::optional<RecoveryEnd> evaluateRecoveryEnd(const RecoveryEndInput& input)
    {
        const auto& limits = input.mLimits;
        if (!finite(input.mCapturePoint) || !finite(input.mVelocity) || !finite(input.mLeftFoot)
            || !finite(input.mRightFoot) || !finite(input.mHeld) || input.mHeld < 0.0f || !finite(input.mDt)
            || input.mDt < 0.0f || !finite(limits.mFootRadius) || limits.mFootRadius < 0.0f || !finite(limits.mMargin)
            || !finite(limits.mSpeed) || limits.mSpeed < 0.0f || !finite(limits.mTime) || limits.mTime < 0.0f)
            return std::nullopt;
        RecoveryEnd end;
        if (!input.mLeftSupported && !input.mRightSupported)
        {
            end.mInside = -std::numeric_limits<float>::infinity();
            return end;
        }
        BalanceVec2 nearest;
        const float distance = input.mLeftSupported && input.mRightSupported
            ? segmentDistance(input.mCapturePoint, input.mLeftFoot, input.mRightFoot, nearest)
            : length(input.mCapturePoint - (input.mLeftSupported ? input.mLeftFoot : input.mRightFoot));
        end.mInside = limits.mFootRadius - distance;
        const bool holding = end.mInside >= limits.mMargin && length(input.mVelocity) <= limits.mSpeed;
        end.mHeld = holding ? input.mHeld + input.mDt : 0.0f;
        end.mCaught = holding && end.mHeld >= limits.mTime;
        return end;
    }

    std::optional<BalanceStageDecision> classifyBalanceStage(const BalanceStageInput& input)
    {
        const auto& limits = input.mLimits;
        if (!finite(input.mCapturePoint) || !finite(input.mLeftFoot) || !finite(input.mRightFoot)
            || !finite(limits.mFootRadius) || limits.mFootRadius < 0.0f || !finite(limits.mHipReach)
            || limits.mHipReach <= 0.0f || !finite(limits.mHysteresis) || limits.mHysteresis < 0.0f
            || limits.mHysteresis >= limits.mHipReach || !finite(limits.mMaximumHipTime)
            || limits.mMaximumHipTime < 0.0f || !finite(input.mTimeInPrevious) || input.mTimeInPrevious < 0.0f)
            return std::nullopt;
        BalanceStageDecision decision;
        if (!input.mLeftSupported && !input.mRightSupported)
        {
            decision.mStage = BalanceStage::Step;
            decision.mExcess = std::numeric_limits<float>::infinity();
            decision.mHipWeight = 1.0f;
            return decision;
        }
        BalanceVec2 nearest;
        float distance;
        if (input.mLeftSupported && input.mRightSupported)
            distance = segmentDistance(input.mCapturePoint, input.mLeftFoot, input.mRightFoot, nearest);
        else
        {
            nearest = input.mLeftSupported ? input.mLeftFoot : input.mRightFoot;
            distance = length(input.mCapturePoint - nearest);
        }
        decision.mExcess = distance - limits.mFootRadius;
        if (distance > 1e-6f)
            decision.mFallDirection = (input.mCapturePoint - nearest) * (1.0f / distance);

        // Entry thresholds 0 (leave the feet) and hipReach (leave what the
        // hip can pull back); a stage is left downward only below its entry
        // threshold minus the hysteresis.
        const float excess = decision.mExcess;
        const float h = limits.mHysteresis;
        switch (input.mPrevious)
        {
            case BalanceStage::Ankle:
                decision.mStage = excess > limits.mHipReach ? BalanceStage::Step
                    : excess > 0.0f                         ? BalanceStage::HipAndArms
                                                            : BalanceStage::Ankle;
                break;
            case BalanceStage::HipAndArms:
            {
                const bool expired = limits.mMaximumHipTime > 0.0f && input.mTimeInPrevious >= limits.mMaximumHipTime
                    && excess > 0.0f;
                decision.mStage = excess > limits.mHipReach || expired ? BalanceStage::Step
                    : excess > -h                                      ? BalanceStage::HipAndArms
                                                                       : BalanceStage::Ankle;
                break;
            }
            case BalanceStage::Step:
                decision.mStage = excess > limits.mHipReach - h ? BalanceStage::Step
                    : excess > -h                               ? BalanceStage::HipAndArms
                                                                : BalanceStage::Ankle;
                break;
        }
        decision.mHipWeight = decision.mStage == BalanceStage::Step
            ? 1.0f
            : std::clamp((excess + h) / (0.5f * limits.mHipReach + h), 0.0f, 1.0f);
        if (decision.mStage == BalanceStage::Ankle)
            decision.mHipWeight = 0.0f;
        return decision;
    }
}
