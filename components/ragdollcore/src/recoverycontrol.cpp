#include <ragdollcore/recoverycontrol.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        constexpr float sPi = 3.14159265358979f;

        BalanceVec2 add(BalanceVec2 a, BalanceVec2 b) { return { a.mX + b.mX, a.mY + b.mY }; }
        BalanceVec2 sub(BalanceVec2 a, BalanceVec2 b) { return { a.mX - b.mX, a.mY - b.mY }; }
        BalanceVec2 scale(BalanceVec2 a, float s) { return { a.mX * s, a.mY * s }; }
        float length(BalanceVec2 a) { return std::sqrt(a.mX * a.mX + a.mY * a.mY); }
        bool finite(float v) { return std::isfinite(v); }
        bool finite(BalanceVec2 v) { return finite(v.mX) && finite(v.mY); }

        BalanceVec2 capLength(BalanceVec2 v, float cap, bool& capped)
        {
            const float l = length(v);
            if (l > cap && l > 0.0f)
            {
                capped = true;
                return scale(v, cap / l);
            }
            return v;
        }
    }

    CatchCapability catchCapability(const RecoveryStats& stats)
    {
        const float agility = std::clamp(finite(stats.mAgility) ? stats.mAgility : 0.0f, 0.0f, 100.0f);
        const float acrobatics = std::clamp(finite(stats.mAcrobatics) ? stats.mAcrobatics : 0.0f, 0.0f, 100.0f);
        const float s = (0.6f * agility + 0.4f * acrobatics) / 100.0f;
        CatchCapability c;
        c.mReachFraction = 0.7f + 0.4f * s;
        c.mMinimumSwingTime = 0.30f - 0.15f * s;
        c.mMaximumSwingTime = std::max(c.mMinimumSwingTime + 0.05f, 0.45f - 0.1f * s);
        c.mSwingFootSpeed = 300.0f + 300.0f * s;
        c.mReactionDelay = 0.15f - 0.12f * s;
        c.mSwingSpringFrequency = 4.0f + 6.0f * s;
        c.mCrossoverClearance = 7.0f;
        return c;
    }

    CaptureStepLimits catchStepLimits(const CatchCapability& capability, float legLength)
    {
        CaptureStepLimits limits;
        limits.mFootRadius = 6.0f;
        limits.mMaximumReach = capability.mReachFraction * legLength;
        limits.mMinimumWidth = 8.0f;
        limits.mMinimumSwingTime = capability.mMinimumSwingTime;
        limits.mMaximumSwingTime = capability.mMaximumSwingTime;
        limits.mSwingFootSpeed = capability.mSwingFootSpeed;
        limits.mCrossoverClearance = capability.mCrossoverClearance;
        limits.mBrakeMargin = 0.0f;
        return limits;
    }

    std::optional<StanceForceOutput> computeStanceForce(const StanceForceInput& input)
    {
        const auto& s = input.mSettings;
        if (!finite(input.mCenterOfMass) || !finite(input.mVelocity) || !finite(input.mRequestVelocity)
            || !(input.mTotalMass > 0.0f) || !(input.mGravity > 0.0f) || !finite(s.mStiffness) || !finite(s.mDamping)
            || !(s.mFrictionShare >= 0.0f))
            return std::nullopt;
        StanceForceOutput out;
        BalanceVec2 centre;
        int planted = 0;
        for (const auto& foot : input.mPlantedFeet)
            if (foot)
            {
                if (!finite(*foot))
                    return std::nullopt;
                centre = add(centre, *foot);
                ++planted;
            }
        if (planted == 0)
            return out;
        centre = scale(centre, 1.0f / static_cast<float>(planted));
        if (finite(input.mCenterShift))
            centre = add(centre, input.mCenterShift);
        const bool moving = length(input.mRequestVelocity) > s.mMovingSpeed;
        BalanceVec2 accel = scale(sub(input.mRequestVelocity, input.mVelocity), s.mDamping);
        if (!moving)
            accel = add(accel, scale(sub(centre, input.mCenterOfMass), s.mStiffness));
        accel = capLength(accel, s.mFrictionShare * input.mGravity, out.mCapped);
        out.mActive = true;
        out.mBodyForce = scale(accel, input.mTotalMass);
        for (std::size_t side = 0; side < 2; ++side)
            if (input.mPlantedFeet[side])
                out.mFootForce[side] = scale(out.mBodyForce, -1.0f / static_cast<float>(planted));
        return out;
    }

    std::optional<SwingSpringOutput> computeSwingSpring(const SwingSpringInput& input)
    {
        const auto& s = input.mSettings;
        if (input.mGoal.has_value() != input.mGoalVelocity.has_value()
            || (input.mGoal && (!finite(*input.mGoal) || !finite(*input.mGoalVelocity))))
            return std::nullopt;
        if (!finite(input.mImplicitDt) || input.mImplicitDt < 0.0f
            || (input.mImplicitDt > 0.0f && !(input.mEffectiveMass > 0.0f && finite(input.mEffectiveMass))))
            return std::nullopt;
        if (!finite(input.mFoot) || !finite(input.mFootVelocity) || !finite(input.mLiftOff) || !finite(input.mLanding)
            || !finite(input.mPhase) || !(input.mDuration > 0.0f) || !(input.mLegMass > 0.0f) || !(input.mGravity > 0.0f)
            || !(s.mFrequency > 0.0f) || !(s.mDampingRatio >= 0.0f) || !(s.mCapLegWeights >= 0.0f))
            return std::nullopt;
        const float t = std::clamp(input.mPhase, 0.0f, 1.0f);
        const float ease = t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
        const float easeRate = 30.0f * t * t * (1.0f - t) * (1.0f - t) / std::max(0.05f, input.mDuration);
        const BalanceVec2 span = sub(input.mLanding, input.mLiftOff);
        const BalanceVec2 goal = input.mGoal.value_or(add(input.mLiftOff, scale(span, ease)));
        const BalanceVec2 goalVelocity = input.mGoalVelocity.value_or(scale(span, easeRate));
        const float omega = 2.0f * sPi * s.mFrequency;
        float stiffness = omega * omega, damping = 2.0f * s.mDampingRatio * omega;
        if (input.mImplicitDt > 0.0f)
        {
            const float dt = input.mImplicitDt;
            const float denominator
                = 1.0f + input.mLegMass / input.mEffectiveMass * (damping * dt + stiffness * dt * dt);
            if (!finite(denominator) || !(denominator > 0.0f))
                return std::nullopt;
            damping = (damping + stiffness * dt) / denominator;
            stiffness /= denominator;
        }
        BalanceVec2 force = scale(add(scale(sub(goal, input.mFoot), stiffness),
                                      scale(sub(goalVelocity, input.mFootVelocity), damping)),
            input.mLegMass);
        SwingSpringOutput out;
        force = capLength(force, s.mCapLegWeights * input.mLegMass * input.mGravity, out.mCapped);
        out.mFootForce = force;
        out.mPelvisForce = scale(force, -1.0f);
        return out;
    }

    std::optional<CatchStepCommand> CatchStepController::update(const CatchStepInput& input)
    {
        if (!(input.mDt > 0.0f) || !finite(input.mReferenceVelocity) || !finite(input.mBodyForward))
            return std::nullopt;
        const auto capture = capturePoint(input.mState);
        if (!capture)
            return std::nullopt;
        const auto& feet = input.mFeet;
        CatchStepCommand command;
        BalanceStageInput stageInput;
        stageInput.mCapturePoint = *capture;
        stageInput.mLeftFoot = feet[0].mPosition;
        stageInput.mRightFoot = feet[1].mPosition;
        stageInput.mLeftSupported = feet[0].mSupported;
        stageInput.mRightSupported = feet[1].mSupported;
        stageInput.mPrevious = mStage;
        stageInput.mTimeInPrevious = mStageTime;
        stageInput.mLimits = { 6.0f, 8.0f, 1.5f, 0.3f };
        if (const auto decision = classifyBalanceStage(stageInput))
        {
            mStageTime = decision->mStage == mStage ? mStageTime + input.mDt : 0.0f;
            mStage = decision->mStage;
            command.mHipWeight = decision->mHipWeight;
            command.mFallDirection = decision->mFallDirection;
        }
        command.mStage = mStage;

        mStepStageTime = mStage == BalanceStage::Step ? mStepStageTime + input.mDt : 0.0f;
        const BalanceVec2 relative = sub(input.mState.mVelocity, input.mReferenceVelocity);
        const bool stumbling = input.mStumbleSteps > 0 && length(relative) > input.mStumbleSpeed;
        mStumbleWait = stumbling ? mStumbleWait + input.mDt : 0.0f;
        if (!mCatching
            && ((mStage == BalanceStage::Step && mStepStageTime >= input.mReactionDelay)
                || (stumbling && mStumbleWait >= input.mReactionDelay)))
        {
            mCatching = true;
            mCatchTime = 0.0f;
            mCaughtHeld = 0.0f;
            mHoverTime = 0.0f;
            mCatchSteps = 0;
        }
        if (mCatching)
        {
            mCatchTime += input.mDt;
            RecoveryEndInput endInput;
            endInput.mCapturePoint = *capture;
            endInput.mVelocity = sub(input.mState.mVelocity, input.mReferenceVelocity);
            endInput.mLeftFoot = feet[0].mPosition;
            endInput.mRightFoot = feet[1].mPosition;
            endInput.mLeftSupported = feet[0].mSupported;
            endInput.mRightSupported = feet[1].mSupported;
            endInput.mHeld = mCaughtHeld;
            endInput.mDt = input.mDt;
            endInput.mLimits = { 6.0f, input.mLimits.mBrakeMargin > 0.0f ? 2.0f : 0.0f, 25.0f, 0.2f };
            const auto end = evaluateRecoveryEnd(endInput);
            mCaughtHeld = end ? end->mHeld : 0.0f;
            const bool swinging = feet[0].mSwinging || feet[1].mSwinging;
            if ((end && end->mCaught && !swinging && !stumbling) || mCatchTime > input.mMaximumRecoveryTime)
                mCatching = false;
        }
        if (!mCatching)
        {
            // Deliberate step (WASD while bracing): the trailing foot steps a
            // stride along the pressed direction.
            const float bias = length(input.mDirectionBias);
            const bool anySwing = feet[0].mSwinging || feet[1].mSwinging;
            mSinceDeliberate = anySwing ? 0.0f : mSinceDeliberate + input.mDt;
            if (input.mDeliberateSteps && bias > 1e-3f && !anySwing && (feet[0].mSupported || feet[1].mSupported)
                && mSinceDeliberate >= input.mDeliberateInterval)
            {
                const BalanceVec2 along = scale(input.mDirectionBias, 1.0f / bias);
                const auto ahead = [&](std::size_t side) {
                    const BalanceVec2 d = sub(feet[side].mPosition, input.mState.mCenterOfMass);
                    return d.mX * along.mX + d.mY * along.mY;
                };
                std::size_t side = ahead(0) <= ahead(1) ? 0 : 1;
                if (!feet[1 - side].mSupported)
                    side = 1 - side; // step the free foot, stand on the planted one
                const BalanceVec2 fromCom = sub(feet[side].mPosition, input.mState.mCenterOfMass);
                const float alongNow = fromCom.mX * along.mX + fromCom.mY * along.mY;
                const BalanceVec2 lateral = sub(fromCom, scale(along, alongNow));
                BalanceVec2 landing = add(add(input.mState.mCenterOfMass, scale(along, input.mDeliberateStride)), lateral);
                const BalanceVec2 fromStance = sub(landing, feet[1 - side].mPosition);
                if (length(fromStance) > input.mLimits.mMaximumReach && input.mLimits.mMaximumReach > 0.0f)
                    landing = add(feet[1 - side].mPosition,
                        scale(fromStance, input.mLimits.mMaximumReach / length(fromStance)));
                command.mHoldClock = true;
                command.mSwingNow = side;
                command.mSwingTime = std::max(input.mLimits.mMinimumSwingTime, 0.2f);
                command.mLanding[side] = landing;
                mSinceDeliberate = 0.0f;
            }
            else if (input.mDeliberateSteps && bias > 1e-3f)
                command.mHoldClock = true; // a deliberate step in flight
            return command;
        }

        command.mHoldClock = true;
        // Hover: the catch runs, but no foot swings.
        mHoverTime = feet[0].mSwinging || feet[1].mSwinging ? 0.0f : mHoverTime + input.mDt;
        command.mHoverTime = mHoverTime;
        const bool hoverDue = input.mMaximumHoverTime > 0.0f && mHoverTime >= input.mMaximumHoverTime;
        const auto planFor = [&](std::size_t swing, std::optional<float> timeToTouchdown) {
            CaptureStepInput step;
            step.mState = input.mState;
            step.mStanceFoot = feet[1 - swing].mPosition;
            step.mSwingFoot = feet[swing].mPosition;
            step.mBodyForward = input.mBodyForward;
            step.mSwingIsLeft = swing == 0;
            step.mTimeToTouchdown = timeToTouchdown;
            step.mLimits = input.mLimits;
            if (input.mCrossoverSpeed > 0.0f && !input.mArmDriven
                && length(sub(input.mState.mVelocity, input.mReferenceVelocity)) < input.mCrossoverSpeed)
                step.mLimits.mCrossoverClearance = 0.0f;
            return planRecoverySteps(step, 3);
        };
        std::optional<std::size_t> inFlight;
        for (std::size_t side = 0; side < 2; ++side)
            if (feet[side].mSwinging)
                inFlight = side;
        if (inFlight)
        {
            const auto& foot = feet[*inFlight];
            if (!foot.mTargetHeld && feet[1 - *inFlight].mSupported)
            {
                const float remaining = std::max(0.05f, (1.0f - foot.mPhase) * foot.mDuration);
                if (const auto plan = planFor(*inFlight, remaining); plan && plan->mCount > 0)
                    command.mLanding[*inFlight] = plan->mSteps[0].mLanding;
            }
            return command;
        }
        const bool bothDown = feet[0].mSupported && feet[1].mSupported;
        // One foot down, the other hanging free and not swinging (a step that
        // landed out of reach): step the free foot now - the stork hover.
        std::optional<std::size_t> freeFoot;
        if (input.mStepFreeFoot && feet[0].mSupported != feet[1].mSupported)
            freeFoot = feet[0].mSupported ? 1 : 0;
        if (!freeFoot && !(bothDown && (mStage == BalanceStage::Step || hoverDue || stumbling)))
        {
            // A hover that cannot step (no foot on the ground) falls.
            if (hoverDue)
            {
                command.mRelease = true;
                mCatching = false;
            }
            return command;
        }
        std::optional<std::size_t> onlySide;
        if (input.mUnloadedFootOnly)
        {
            const BalanceVec2 fall = sub(*capture, input.mState.mCenterOfMass);
            const auto along = [&](std::size_t side) {
                const BalanceVec2 d = sub(feet[side].mPosition, input.mState.mCenterOfMass);
                return d.mX * fall.mX + d.mY * fall.mY;
            };
            onlySide = along(0) < along(1) ? 0 : 1;
        }
        if (freeFoot)
            onlySide = freeFoot;
        std::optional<RecoveryPlan> best;
        std::size_t bestSide = 0;
        for (std::size_t side = 0; side < 2; ++side)
        {
            if (onlySide && side != *onlySide)
                continue;
            const auto plan = planFor(side, std::nullopt);
            if (!plan || plan->mCount == 0)
                continue;
            const auto along = [&](const RecoveryPlan& p, std::size_t swing) {
                const BalanceVec2 d = sub(p.mSteps[0].mLanding, feet[swing].mPosition);
                const float l = length(d);
                return l > 1e-3f ? (d.mX * input.mDirectionBias.mX + d.mY * input.mDirectionBias.mY) / l : 0.0f;
            };
            const bool steered = length(input.mDirectionBias) > 1e-3f;
            const auto better = [&](const RecoveryPlan& a, std::size_t sideA, const RecoveryPlan& b, std::size_t sideB) {
                if (a.mCaptured != b.mCaptured)
                    return a.mCaptured;
                if (a.mCount != b.mCount)
                    return a.mCount < b.mCount;
                if (steered)
                {
                    const float difference = along(a, sideA) - along(b, sideB);
                    if (std::abs(difference) > 0.01f)
                        return difference > 0.0f;
                }
                return a.mSteps[a.mCount - 1].mResidual < b.mSteps[b.mCount - 1].mResidual;
            };
            if (!best || better(*plan, side, *best, bestSide))
            {
                best = plan;
                bestSide = side;
            }
        }
        // Stumble step: no capturing plan is needed (the capture point is
        // still over the feet while the body is carried along), so step a foot
        // along the motion instead of letting the planted feet drag.
        if (stumbling && bothDown && (!best || best->mCount == 0) && mStage != BalanceStage::Step)
        {
            const float speed = length(relative);
            const BalanceVec2 along = scale(relative, 1.0f / speed);
            const BalanceVec2 right{ input.mBodyForward.mY, -input.mBodyForward.mX };
            const bool sideways = std::abs(along.mX * right.mX + along.mY * right.mY) > 0.7f;
            const auto alongOf = [&](BalanceVec2 p) {
                const BalanceVec2 d = sub(p, input.mState.mCenterOfMass);
                return d.mX * along.mX + d.mY * along.mY;
            };
            // Sideways: the leading foot side-steps; else the trailing foot.
            const bool leftAhead = alongOf(feet[0].mPosition) >= alongOf(feet[1].mPosition);
            const std::size_t side = sideways ? (leftAhead ? 0 : 1) : (leftAhead ? 1 : 0);
            const auto tau = balanceTimeConstant(input.mState);
            const float lead = (tau ? *tau : 0.3f) * speed * input.mStumbleLead + input.mStumbleExtra;
            // Keep the foot's own side across the motion (at least half the
            // minimum width), the landing `lead` ahead of the center of mass.
            const BalanceVec2 across{ -along.mY, along.mX };
            const BalanceVec2 fromCom = sub(feet[side].mPosition, input.mState.mCenterOfMass);
            float side_ = fromCom.mX * across.mX + fromCom.mY * across.mY;
            const BalanceVec2 otherFrom = sub(feet[1 - side].mPosition, input.mState.mCenterOfMass);
            const float otherSide = otherFrom.mX * across.mX + otherFrom.mY * across.mY;
            const float halfWidth = 0.5f * std::max(input.mLimits.mMinimumWidth, 1.0f);
            if (std::abs(side_ - otherSide) < 2.0f * halfWidth)
                side_ = otherSide + (side_ >= otherSide ? 2.0f : -2.0f) * halfWidth;
            BalanceVec2 landing = add(input.mState.mCenterOfMass, add(scale(along, lead), scale(across, side_)));
            const BalanceVec2 fromStance = sub(landing, feet[1 - side].mPosition);
            if (input.mLimits.mMaximumReach > 0.0f && length(fromStance) > input.mLimits.mMaximumReach)
                landing = add(feet[1 - side].mPosition, scale(fromStance, input.mLimits.mMaximumReach / length(fromStance)));
            command.mSwingNow = side;
            command.mSwingTime = std::max(input.mLimits.mMinimumSwingTime, 0.22f);
            command.mLanding[side] = landing;
            command.mStumble = true;
            ++mStepsStarted;
            ++mCatchSteps;
            return command;
        }
        // No step of this capability catches: fall now instead of leaning on
        // the root (or after the hover limit).
        const bool releaseUncatchable = input.mReleaseWhenUncatchable && mCatchSteps >= input.mReleaseAfterSteps;
        if ((!best || !best->mCaptured) && (releaseUncatchable || hoverDue || (input.mReleaseWhenUncatchable && !best)))
        {
            command.mRelease = true;
            mCatching = false;
            return command;
        }
        if (best)
        {
            command.mSwingNow = bestSide;
            command.mSwingTime = best->mSteps[0].mSwingTime;
            command.mLanding[bestSide] = best->mSteps[0].mLanding;
            ++mStepsStarted;
            ++mCatchSteps;
        }
        return command;
    }

    std::optional<MovingCatchCommand> MovingCatchController::update(const MovingCatchInput& input)
    {
        const auto& s = input.mSettings;
        const auto tau = balanceTimeConstant(input.mState);
        if (!tau || !(input.mDt > 0.0f) || !finite(input.mCommandedVelocity) || !finite(input.mState.mVelocity)
            || !finite(input.mState.mCenterOfMass) || !(input.mReach > 0.0f))
            return std::nullopt;
        MovingCatchCommand command;
        BalanceVec2 error = sub(input.mState.mVelocity, input.mCommandedVelocity);
        const float commanded = length(input.mCommandedVelocity);
        if (!s.mCorrectLag && commanded > 1e-3f)
        {
            const BalanceVec2 along = scale(input.mCommandedVelocity, 1.0f / commanded);
            const float lag = error.mX * along.mX + error.mY * along.mY;
            if (lag < 0.0f)
                error = sub(error, scale(along, lag));
        }
        const BalanceVec2 needed = scale(error, s.mGain * *tau);
        command.mCorrection = length(needed);
        bool capped = false;
        const BalanceVec2 correction = capLength(needed, std::max(0.0f, s.mMaximumCorrection), capped);
        const bool correcting = length(error) > s.mErrorThreshold;
        const BalanceVec2 com = input.mState.mCenterOfMass;
        // Keep a landing within reach of the center of mass.
        bool beyond = false;
        const auto withinReach = [&](BalanceVec2 landing) {
            const BalanceVec2 offset = sub(landing, com);
            if (length(offset) <= input.mReach)
                return landing;
            beyond = true;
            return add(com, scale(offset, input.mReach / length(offset)));
        };
        // Across the commanded direction (the gait's left-right).
        const float commandedLength = length(input.mCommandedVelocity);
        const BalanceVec2 across = commandedLength > 1e-3f
            ? BalanceVec2{ -input.mCommandedVelocity.mY / commandedLength, input.mCommandedVelocity.mX / commandedLength }
            : BalanceVec2{ 1.0f, 0.0f };
        const auto acrossOf = [&](BalanceVec2 v) { return v.mX * across.mX + v.mY * across.mY; };
        // A landing keeps its side of the other (stance) foot.
        const auto ownSide = [&](std::size_t side, BalanceVec2 base, BalanceVec2 landing) {
            const BalanceVec2 stance = input.mFeet[1 - side].mPosition;
            const float baseSide = acrossOf(sub(base, stance));
            const float sign = baseSide >= 0.0f ? 1.0f : -1.0f;
            const float offset = acrossOf(sub(landing, stance)) * sign;
            if (offset >= s.mMinimumWidth)
                return landing;
            return add(landing, scale(across, (s.mMinimumWidth - offset) * sign));
        };
        // Swings in flight: Footfall's landing (taken at the swing start)
        // moved by the correction.
        for (std::size_t side = 0; side < 2; ++side)
        {
            const auto& foot = input.mFeet[side];
            if (!foot.mSwinging || !finite(foot.mTarget))
            {
                mSwingBase[side].reset();
                continue;
            }
            if (!mSwingBase[side])
                mSwingBase[side] = foot.mTarget;
            if (correcting)
                command.mLanding[side]
                    = withinReach(ownSide(side, *mSwingBase[side], add(*mSwingBase[side], correction)));
        }
        // A foot left hanging (neither pinned nor swinging) while the other
        // stands: swing it now, to where the body's motion is caught - the
        // capture point in the gait's frame, plus half a swing of travel.
        for (std::size_t side = 0; side < 2; ++side)
        {
            const auto& foot = input.mFeet[side];
            const bool free = !foot.mPinned && !foot.mSwinging;
            mFreeTime[side] = free ? mFreeTime[side] + input.mDt : 0.0f;
        }
        // A planted foot left behind the center of mass along the travel.
        const BalanceVec2 travel = commandedLength > 1e-3f ? scale(input.mCommandedVelocity, 1.0f / commandedLength)
                                                           : BalanceVec2{};
        std::array<float, 2> trail{ 0.0f, 0.0f };
        for (std::size_t side = 0; side < 2; ++side)
        {
            const auto& foot = input.mFeet[side];
            const BalanceVec2 offset = sub(foot.mPosition, com);
            trail[side] = -(offset.mX * travel.mX + offset.mY * travel.mY);
            const bool trailing = (foot.mPinned || foot.mSupported) && !foot.mSwinging && commandedLength > 1e-3f
                && trail[side] > s.mTrailLimit * input.mReach;
            mTrailTime[side] = trailing ? mTrailTime[side] + input.mDt : 0.0f;
        }
        const bool anySwing = input.mFeet[0].mSwinging || input.mFeet[1].mSwinging;
        if (!anySwing && correcting)
            for (std::size_t side = 0; side < 2; ++side)
            {
                const auto& other = input.mFeet[1 - side];
                if (mFreeTime[side] < s.mFreeFootDelay || !(other.mPinned || other.mSupported))
                    continue;
                const float swing = std::max(0.1f, input.mMinimumSwingTime);
                const BalanceVec2 landing
                    = add(add(com, scale(input.mCommandedVelocity, 0.5f * swing)), correction);
                command.mSwingNow = side;
                command.mSwingTime = swing;
                command.mLanding[side] = withinReach(ownSide(side, input.mFeet[side].mPosition, landing));
                mFreeTime[side] = 0.0f;
                break;
            }
        if (!anySwing && !command.mSwingNow)
        {
            // The foot further behind steps first; the other must hold the body.
            const std::size_t side = trail[0] >= trail[1] ? 0 : 1;
            const auto& other = input.mFeet[1 - side];
            if (mTrailTime[side] >= s.mTrailTime && (other.mPinned || other.mSupported))
            {
                const float swing = std::max(0.1f, input.mMinimumSwingTime);
                const BalanceVec2 landing = add(add(com, scale(input.mCommandedVelocity, 0.5f * swing)), correction);
                command.mSwingNow = side;
                command.mSwingTime = swing;
                command.mTrail = true;
                command.mLanding[side] = withinReach(ownSide(side, input.mFeet[side].mPosition, landing));
                mTrailTime[side] = 0.0f;
            }
        }
        // Uncatchable: the correction the error needs stays beyond the reach.
        const bool needsTooMuch = correcting && command.mCorrection > input.mReach;
        mBeyondReach = needsTooMuch ? mBeyondReach + input.mDt : 0.0f;
        command.mRelease = s.mRelease && mBeyondReach >= s.mReleaseTime;
        (void)beyond;
        return command;
    }

    std::optional<LandingCommand> LandingController::update(const LandingInput& input)
    {
        const auto& s = input.mSettings;
        if (input.mTouchdown)
        {
            mActive = true;
            mSince = 0.0f;
            mSteps = 0;
            mWasSwinging = false;
            mImpact = finite(input.mImpactSpeed) ? std::max(0.0f, input.mImpactSpeed) : 0.0f;
        }
        if (!mActive || !(input.mDt > 0.0f))
            return std::nullopt;
        mSince += input.mDt;
        LandingCommand command;
        command.mActive = true;
        // Absorb.
        if (s.mAbsorbTime > 0.0f)
        {
            const float impact = std::clamp(mImpact / std::max(1.0f, s.mFullImpact), 0.0f, 1.0f);
            command.mRootStrengthFloor
                = s.mAbsorbStrength * impact * std::clamp(1.0f - mSince / s.mAbsorbTime, 0.0f, 1.0f);
        }
        const auto tau = balanceTimeConstant(input.mState);
        const BalanceVec2 v = input.mState.mVelocity;
        const float speed = length(v);
        const bool anySwing = input.mFeet[0].mSwinging || input.mFeet[1].mSwinging;
        if (mWasSwinging && !anySwing)
            ++mSteps; // a run-out step landed
        mWasSwinging = anySwing;
        command.mSteps = mSteps;
        const bool runningOut = tau && finite(v) && speed > s.mRunOutSpeed && mSteps < s.mMaximumSteps;
        const bool absorbing = command.mRootStrengthFloor > 0.0f;
        if ((!runningOut && !anySwing && !absorbing) || mSince > s.mMaximumTime)
        {
            mActive = false;
            return command.mRootStrengthFloor > 0.0f ? std::optional<LandingCommand>(command) : std::nullopt;
        }
        if (!runningOut)
            return command;
        command.mHoldClock = true;
        const BalanceVec2 along = scale(v, 1.0f / speed);
        const BalanceVec2 across{ -along.mY, along.mX };
        const BalanceVec2 com = input.mState.mCenterOfMass;
        const BalanceVec2 capture = add(com, scale(v, s.mGain * *tau));
        const auto alongOf = [&](BalanceVec2 p) { const BalanceVec2 d = sub(p, com); return d.mX * along.mX + d.mY * along.mY; };
        const auto acrossOf = [&](BalanceVec2 p) { const BalanceVec2 d = sub(p, com); return d.mX * across.mX + d.mY * across.mY; };
        const float captureAhead = alongOf(capture);
        // Release: still beyond the reach after the steps it was given.
        if (s.mReleaseAfterSteps > 0 && mSteps >= s.mReleaseAfterSteps && !anySwing && captureAhead > input.mReach)
        {
            command.mRelease = true;
            mActive = false;
            return command;
        }
        if (anySwing)
            return command;
        // The leading planted foot: the support the capture point must not outrun.
        float lead = -1e9f;
        for (const auto& foot : input.mFeet)
            if (foot.mPinned || foot.mSupported)
                lead = std::max(lead, alongOf(foot.mPosition));
        if (captureAhead < lead + s.mStepMargin)
            return command;
        // The trailing foot steps; the other must hold the body.
        const std::size_t side = alongOf(input.mFeet[0].mPosition) <= alongOf(input.mFeet[1].mPosition) ? 0 : 1;
        const auto& other = input.mFeet[1 - side];
        if (!(other.mPinned || other.mSupported))
            return command;
        // Its side of the other foot, the capture point along the carry.
        const float otherAcross = acrossOf(other.mPosition);
        const float ownAcross = acrossOf(input.mFeet[side].mPosition);
        const float sign = ownAcross >= otherAcross ? 1.0f : -1.0f;
        const float landingAcross = sign > 0.0f ? std::max(ownAcross, otherAcross + s.mWidth)
                                                : std::min(ownAcross, otherAcross - s.mWidth);
        const float landingAlong = std::min(captureAhead, input.mReach);
        command.mSwingNow = side;
        command.mSwingTime = std::max(0.1f, input.mMinimumSwingTime);
        command.mLanding[side] = add(com, add(scale(along, landingAlong), scale(across, landingAcross)));
        return command;
    }

    BalanceVec2 uprightTilt(float upX, float upY, float upZ)
    {
        if (!finite(upX) || !finite(upY) || !finite(upZ))
            return {};
        // up x Z = (upY, -upX, 0); its length is sin(angle).
        const BalanceVec2 cross{ upY, -upX };
        const float sine = length(cross);
        if (sine < 1e-6f)
            return {};
        return scale(cross, std::atan2(sine, upZ) / sine);
    }

    BalanceVec2 PostureController::update(BalanceVec2 torsoTilt, float dt, const PostureSettings& settings)
    {
        if (!finite(torsoTilt) || !(dt > 0.0f))
            return {};
        const float maximum = settings.mMaximumDegrees * sPi / 180.0f;
        mIntegral = add(mIntegral, scale(torsoTilt, dt));
        bool capped = false;
        if (settings.mIntegral > 0.0f)
            mIntegral = capLength(mIntegral, maximum / settings.mIntegral, capped);
        BalanceVec2 lean = add(scale(torsoTilt, settings.mProportional), scale(mIntegral, settings.mIntegral));
        return capLength(lean, maximum, capped);
    }

    float rootBudgetStrength(const RootBudgetInput& input)
    {
        const auto& s = input.mSettings;
        const float hit = finite(input.mHitStrength) ? std::clamp(input.mHitStrength, 0.0f, 1.0f) : 0.0f;
        const bool moving = (finite(input.mRequestSpeed) && input.mRequestSpeed > s.mMovingSpeed)
            || (finite(input.mBodySpeed) && input.mBodySpeed > s.mMovingSpeed);
        const float budget = std::clamp(moving ? s.mMoving : s.mStanding, 0.0f, 1.0f);
        return std::min(hit, budget);
    }
}
