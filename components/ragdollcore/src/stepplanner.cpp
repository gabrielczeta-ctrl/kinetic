#include <ragdollcore/stepplanner.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        BalanceVec2 add(BalanceVec2 a, BalanceVec2 b) { return { a.mX + b.mX, a.mY + b.mY }; }
        BalanceVec2 sub(BalanceVec2 a, BalanceVec2 b) { return { a.mX - b.mX, a.mY - b.mY }; }
        BalanceVec2 scale(BalanceVec2 a, float s) { return { a.mX * s, a.mY * s }; }
        float dot(BalanceVec2 a, BalanceVec2 b) { return a.mX * b.mX + a.mY * b.mY; }
        float length(BalanceVec2 a) { return std::hypot(a.mX, a.mY); }
        bool finite(BalanceVec2 a) { return std::isfinite(a.mX) && std::isfinite(a.mY); }

        // Distance from p to the segment a-b.
        float segmentDistance(BalanceVec2 p, BalanceVec2 a, BalanceVec2 b)
        {
            const BalanceVec2 ab = sub(b, a);
            const float len2 = dot(ab, ab);
            const float t = len2 > 1e-8f ? std::clamp(dot(sub(p, a), ab) / len2, 0.0f, 1.0f) : 0.0f;
            return length(sub(p, add(a, scale(ab, t))));
        }

        // The landing kept on the swing foot's side (`side` points there,
        // unit) at least `width` from the stance foot, and within `reach`.
        // Moving (`travel` a unit vector, else zero), it never lands behind
        // the stance foot along the travel.
        BalanceVec2 constrainLanding(BalanceVec2 u, BalanceVec2 stance, BalanceVec2 side, BalanceVec2 travel,
            float width, float reach, bool& widthActive, bool& reachActive)
        {
            for (int pass = 0; pass < 4; ++pass)
            {
                const float across = dot(sub(u, stance), side);
                if (across < width)
                {
                    u = add(u, scale(side, width - across));
                    widthActive = true;
                }
                const float along = dot(sub(u, stance), travel);
                if (along < 0.0f)
                    u = sub(u, scale(travel, along));
                const BalanceVec2 from = sub(u, stance);
                const float distance = length(from);
                if (reach > 0.0f && distance > reach)
                {
                    u = add(stance, scale(from, reach / distance));
                    reachActive = true;
                }
                else
                    break;
            }
            return u;
        }
    }

    std::optional<float> strideOmega(const StrideState& state)
    {
        if (!(state.mHeight > 0.0f) || !(state.mGravity > 0.0f) || !std::isfinite(state.mHeight)
            || !std::isfinite(state.mGravity))
            return std::nullopt;
        return std::sqrt(state.mGravity / state.mHeight);
    }

    std::optional<BalanceVec2> strideDcm(const StrideState& state, BalanceVec2 stanceFoot)
    {
        const auto omega = strideOmega(state);
        if (!omega || !finite(state.mCenterOfMass) || !finite(stanceFoot))
            return std::nullopt;
        BalanceVec2 velocity = state.mVelocity;
        if (state.mAngularMomentum)
        {
            // L about the contact of a point mass at height h moving at v:
            // Lx = -m h vy, Ly = m h vx; read back as the equivalent velocity,
            // which carries the swinging legs' and arms' momentum too.
            if (!(state.mMass > 0.0f) || !finite(*state.mAngularMomentum))
                return std::nullopt;
            const float mh = state.mMass * state.mHeight;
            velocity = { state.mAngularMomentum->mY / mh, -state.mAngularMomentum->mX / mh };
        }
        if (!finite(velocity))
            return std::nullopt;
        return add(state.mCenterOfMass, scale(velocity, 1.0f / *omega));
    }

    std::optional<StridePlan> planStride(const StrideInput& input)
    {
        const StrideLimits& limits = input.mLimits;
        const auto omega = strideOmega(input.mState);
        const auto dcm = strideDcm(input.mState, input.mStanceFoot);
        const float forwardLength = length(input.mForward);
        if (!omega || !dcm || !(forwardLength > 1e-6f) || !finite(input.mDesiredVelocity) || !finite(input.mStanceFoot)
            || !finite(input.mSwingFoot) || !(limits.mNominalStepTime > 0.0f) || !(limits.mMinimumSwingTime > 0.0f)
            || limits.mMaximumSwingTime < limits.mMinimumSwingTime || !(limits.mMaximumReach > 0.0f)
            || !std::isfinite(input.mElapsed) || input.mElapsed < 0.0f)
            return std::nullopt;

        const BalanceVec2 forward = scale(input.mForward, 1.0f / forwardLength);
        const BalanceVec2 right{ forward.mY, -forward.mX };
        const BalanceVec2 side = input.mSwingIsLeft ? scale(right, -1.0f) : right;
        const BalanceVec2 p = input.mStanceFoot;

        StridePlan plan;
        plan.mDcm = *dcm;

        // The steady gait at the desired velocity: a step of v T to the
        // swing foot's side, and the DCM offset that gait keeps at each
        // touchdown (b = L / (tau - 1) along the travel, W / (1 + tau) back
        // toward the middle).
        const float nominalTime = std::clamp(limits.mNominalStepTime, limits.mMinimumSwingTime, limits.mMaximumSwingTime);
        const float tauGait = std::max(1.2f, std::exp(*omega * nominalTime));
        const BalanceVec2 step = scale(input.mDesiredVelocity, nominalTime);
        plan.mNominalLanding = add(add(p, step), scale(side, limits.mStanceWidth));
        plan.mNominalOffset
            = add(scale(step, 1.0f / (tauGait - 1.0f)), scale(side, -limits.mStanceWidth / (1.0f + tauGait)));

        // The time left in this swing, within its bounds.
        const float earliest = std::max(limits.mMinimumSwingTime - input.mElapsed, 1.0f / 60.0f);
        const float latest = std::max(limits.mMaximumSwingTime - input.mElapsed, earliest);
        const float tauMin = std::exp(*omega * earliest);
        const float tauMax = std::exp(*omega * latest);
        const float tauNominal = std::exp(*omega * std::clamp(nominalTime - input.mElapsed, earliest, latest));

        // Minimise wu |u - u0|^2 + wt (tau - tau0)^2 + wb |p + a tau - u - b0|^2
        // over the landing u and tau, by blocks: each block's minimum is in
        // closed form and the constraints are convex.
        const BalanceVec2 a = sub(*dcm, p);
        // Asked to move: the steps go that way. A body slower than its gait
        // (a start from rest) takes a short step forward; the push that
        // starts it is the stance's (MOVE-01 4.5), not a step back.
        const float desiredSpeed = length(input.mDesiredVelocity);
        const BalanceVec2 travel
            = desiredSpeed > 1.0f ? scale(input.mDesiredVelocity, 1.0f / desiredSpeed) : BalanceVec2{};
        const float wu = std::max(limits.mLandingWeight, 1e-4f);
        const float wt = std::max(limits.mTimeWeight, 1e-4f);
        // The offset's weight per axis of the travel frame (the travel, or
        // facing standing still, and across it). Slower than the gait along
        // the travel is safe (a start, a slowing body): that axis weighs
        // mLagWeight of it, so the landing stays near the gait's step.
        const BalanceVec2 along = desiredSpeed > 1.0f ? travel : forward;
        const BalanceVec2 across{ along.mY, -along.mX };
        const float gaitAlong = dot(sub(add(plan.mNominalLanding, plan.mNominalOffset), p), along) / tauGait;
        const bool lagging = desiredSpeed > 1.0f && dot(a, along) < gaitAlong;
        const float wb = std::max(limits.mOffsetWeight, 1e-4f);
        const float wAlong = lagging ? wb * std::clamp(limits.mLagWeight, 0.0f, 1.0f) : wb;
        const float wAcross = wb;
        const auto weighted = [&](BalanceVec2 x, BalanceVec2 y) {
            return wAlong * dot(x, along) * dot(y, along) + wAcross * dot(x, across) * dot(y, across);
        };
        BalanceVec2 u = plan.mNominalLanding;
        float tau = tauNominal;
        for (int iteration = 0; iteration < 24; ++iteration)
        {
            const BalanceVec2 c = sub(sub(p, u), plan.mNominalOffset);
            float next = (wt * tauNominal - weighted(a, c)) / (wt + weighted(a, a));
            next = std::clamp(next, tauMin, tauMax);
            const BalanceVec2 q = sub(add(p, scale(a, next)), plan.mNominalOffset);
            const float landingAlong
                = (wu * dot(plan.mNominalLanding, along) + wAlong * dot(q, along)) / (wu + wAlong);
            const float landingAcross
                = (wu * dot(plan.mNominalLanding, across) + wAcross * dot(q, across)) / (wu + wAcross);
            BalanceVec2 landing = add(scale(along, landingAlong), scale(across, landingAcross));
            bool widthActive = false, reachActive = false;
            landing = constrainLanding(
                landing, p, side, travel, limits.mMinimumWidth, limits.mMaximumReach, widthActive, reachActive);
            const bool settled = std::abs(next - tau) < 1e-5f && length(sub(landing, u)) < 1e-4f;
            tau = next;
            u = landing;
            plan.mWidthActive = widthActive;
            plan.mReachActive = reachActive;
            if (settled)
                break;
        }
        plan.mTimeClamped = tau <= tauMin + 1e-5f || tau >= tauMax - 1e-5f;
        plan.mLanding = u;
        plan.mRemainingTime = std::log(tau) / *omega;
        plan.mSwingTime = input.mElapsed + plan.mRemainingTime;
        plan.mPredictedDcm = add(p, scale(a, tau));
        plan.mOffset = sub(plan.mPredictedDcm, u);

        // The DCM would end farther from the foot than the gait keeps it (the
        // body falling away, not merely slower than its gait): the N-step
        // catch (to a stop) plans the first step instead; none catches: give up.
        const float runaway = length(plan.mOffset) - length(plan.mNominalOffset);
        if (runaway > limits.mOffsetTolerance + 0.25f * length(plan.mNominalOffset))
        {
            CaptureStepInput catchInput;
            catchInput.mState.mCenterOfMass = input.mState.mCenterOfMass;
            // The velocity that reproduces this DCM (ALIP's included).
            catchInput.mState.mVelocity = scale(sub(*dcm, input.mState.mCenterOfMass), *omega);
            catchInput.mState.mHeight = input.mState.mHeight;
            catchInput.mState.mGravity = input.mState.mGravity;
            catchInput.mStanceFoot = p;
            catchInput.mSwingFoot = input.mSwingFoot;
            catchInput.mBodyForward = forward;
            catchInput.mSwingIsLeft = input.mSwingIsLeft;
            catchInput.mLimits = limits.mCatch;
            if (input.mElapsed > 0.0f)
                catchInput.mTimeToTouchdown = std::max(limits.mCatch.mMinimumSwingTime - input.mElapsed, 1.0f / 60.0f);
            if (const auto recovery = planRecoverySteps(catchInput, std::max<std::size_t>(1, limits.mCatchSteps));
                recovery && recovery->mCount > 0)
            {
                const RecoveryStep& first = recovery->mSteps[0];
                plan.mRecovery = true;
                plan.mRecoverySteps = recovery->mCount;
                plan.mGiveUp = !recovery->mCaptured;
                plan.mLanding = first.mLanding;
                plan.mRemainingTime = input.mElapsed > 0.0f ? *catchInput.mTimeToTouchdown : first.mSwingTime;
                plan.mSwingTime = input.mElapsed + plan.mRemainingTime;
                plan.mPredictedDcm = add(p, scale(a, std::exp(*omega * plan.mRemainingTime)));
                plan.mOffset = sub(plan.mPredictedDcm, plan.mLanding);
                plan.mWidthActive = false;
                plan.mReachActive = false;
                plan.mTimeClamped = false;
            }
        }
        return plan;
    }

    std::optional<bool> strideNeedsStep(const StrideStandInput& input)
    {
        const auto dcm = strideDcm(input.mState, scale(add(input.mLeftFoot, input.mRightFoot), 0.5f));
        if (!dcm || !finite(input.mLeftFoot) || !finite(input.mRightFoot))
            return std::nullopt;
        return segmentDistance(*dcm, input.mLeftFoot, input.mRightFoot) - input.mFootRadius > input.mMargin;
    }

    BalanceVec2 strideAngularMomentumAbout(const std::array<float, 3>& centerOfMass,
        const std::array<float, 3>& linearMomentum, const std::array<float, 3>& angularMomentumAboutCom,
        const std::array<float, 3>& contact)
    {
        const float rx = centerOfMass[0] - contact[0];
        const float ry = centerOfMass[1] - contact[1];
        const float rz = centerOfMass[2] - contact[2];
        // (r x P) x and y.
        const float x = ry * linearMomentum[2] - rz * linearMomentum[1];
        const float y = rz * linearMomentum[0] - rx * linearMomentum[2];
        return { angularMomentumAboutCom[0] + x, angularMomentumAboutCom[1] + y };
    }

    StrideLimits strideLimitsFromGait(float swingTime, float duty, float halfWidth, StrideLimits limits)
    {
        if (swingTime > 0.0f && duty < 0.95f && std::isfinite(swingTime) && std::isfinite(duty))
            limits.mNominalStepTime = 0.5f * swingTime / (1.0f - std::max(duty, 0.0f));
        if (halfWidth > 0.0f && std::isfinite(halfWidth))
            limits.mStanceWidth = 2.0f * halfWidth;
        return limits;
    }

    bool strideFirstFootIsLeft(BalanceVec2 leftFoot, BalanceVec2 rightFoot, BalanceVec2 travel, bool guard)
    {
        // How far the left foot is ahead of the right along the travel.
        const float ahead = dot(sub(leftFoot, rightFoot), travel);
        return guard ? ahead >= 0.0f : ahead < 0.0f;
    }
}
