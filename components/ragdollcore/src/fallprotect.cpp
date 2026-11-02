#include <ragdollcore/fallprotect.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        float horizontalLength(FallVec3 v)
        {
            return std::hypot(v.mX, v.mY);
        }
        float mix(float a, float b, float t)
        {
            return a + (b - a) * std::clamp(t, 0.0f, 1.0f);
        }
        // -1..1 from a seed (splitmix32 steps).
        float spread(std::uint32_t& state)
        {
            state += 0x9e3779b9u;
            std::uint32_t z = state;
            z = (z ^ (z >> 16)) * 0x85ebca6bu;
            z = (z ^ (z >> 13)) * 0xc2b2ae35u;
            z ^= z >> 16;
            return static_cast<float>(z) / 2147483647.5f - 1.0f;
        }
        FallVec3 unit(FallVec3 v, FallVec3 fallback)
        {
            const float l = std::sqrt(v.mX * v.mX + v.mY * v.mY + v.mZ * v.mZ);
            if (!(l > 1e-4f) || !std::isfinite(l))
                return fallback;
            return { v.mX / l, v.mY / l, v.mZ / l };
        }
    }

    bool ToppleJudge::update(float torsoUpDot, float dt, const ToppleSettings& settings)
    {
        if (mToppled)
            return true;
        if (!std::isfinite(torsoUpDot) || !std::isfinite(dt) || dt < 0.0f)
            return false;
        const bool rising = !mHas || torsoUpDot <= mLast + 1e-4f;
        mLast = torsoUpDot;
        mHas = true;
        if (torsoUpDot < settings.mHardUpDot)
            mToppled = true;
        else if (torsoUpDot < settings.mUpDot)
        {
            if (!settings.mRising || rising)
                mHeld += dt;
        }
        else
            mHeld = 0.0f;
        if (mHeld >= settings.mHold)
            mToppled = true;
        return mToppled;
    }

    FallProtectSettings fallProtectSettings(float agility, float acrobatics, FallProtectSettings base)
    {
        const float a = std::isfinite(agility) ? agility : 50.0f;
        const float b = std::isfinite(acrobatics) ? acrobatics : a;
        const float skill = std::clamp((0.6f * a + 0.4f * b) / 100.0f, 0.0f, 1.0f);
        base.mReactionDelay = mix(0.30f, 0.05f, skill);
        base.mBraceArms = mix(0.5f, 1.2f, skill);
        base.mAimLanding = mix(0.3f, 1.0f, skill);
        return base;
    }

    FallKind classifyFall(FallVec3 direction, FallVec3 bodyForward)
    {
        const float d = horizontalLength(direction);
        const float f = horizontalLength(bodyForward);
        if (d < 1e-6f || f < 1e-6f)
            return FallKind::Forward;
        const float along = (direction.mX * bodyForward.mX + direction.mY * bodyForward.mY) / (d * f);
        if (along > 0.7071f)
            return FallKind::Forward;
        if (along < -0.7071f)
            return FallKind::Backward;
        return FallKind::Side;
    }

    float FallPredictor::update(float torsoUpDot, float dt, const FallPredictSettings& settings)
    {
        if (!std::isfinite(torsoUpDot) || !std::isfinite(dt) || dt <= 0.0f)
            return 0.0f;
        const float tilt = std::acos(std::clamp(torsoUpDot, -1.0f, 1.0f));
        if (mHas)
        {
            const float k = std::min(1.0f, settings.mRateFilter * dt);
            mRate += ((tilt - mTilt) / dt - mRate) * k;
        }
        mTilt = tilt;
        mHas = true;
        float risk = 0.0f;
        if (tilt >= settings.mMinimumTilt)
        {
            const float predicted = tilt + std::max(0.0f, mRate) * settings.mLookahead;
            const float span = std::max(1e-3f, settings.mFull - settings.mOnset);
            risk = std::clamp((predicted - settings.mOnset) / span, 0.0f, 1.0f);
        }
        if (risk > 0.0f)
        {
            mAlert = true;
            mClear = 0.0f;
        }
        else if (mAlert)
        {
            mClear += dt;
            if (mClear >= settings.mClearTime)
                mAlert = false;
        }
        return risk;
    }

    FallProtectCommand FallProtectController::update(const FallProtectInput& input)
    {
        const FallProtectSettings& s = input.mSettings;
        const float dt = std::isfinite(input.mDt) ? std::max(0.0f, input.mDt) : 0.0f;
        mTime += dt;
        if (!input.mFallen)
            mAlertTime = mTime;
        if (!mHasSpread)
        {
            std::uint32_t state = input.mSeed;
            for (float& value : mBendSpread)
                value = spread(state);
            for (float& value : mReachSpread)
                value = spread(state);
            mHasSpread = true;
        }
        for (std::size_t side = 0; side < 2; ++side)
        {
            if (mArmTouchTime[side] >= 0.0f)
                mArmTouchTime[side] += dt;
            else if (input.mFallen && input.mArms[side].mTouching && mPhase != FallPhase::Reacting)
                mArmTouchTime[side] = 0.0f;
        }
        const float speed3 = std::sqrt(input.mComVelocity.mX * input.mComVelocity.mX
            + input.mComVelocity.mY * input.mComVelocity.mY + input.mComVelocity.mZ * input.mComVelocity.mZ);
        if (s.mToneSpeed > 0.0f && std::isfinite(speed3))
            mTone += (std::clamp(speed3 / s.mToneSpeed, 0.0f, 1.0f) - mTone) * std::min(1.0f, s.mToneRate * dt);

        // Direction: the centre of mass's horizontal travel, or the torso's
        // tilt while slow; followed at a rate so it cannot flip in one step,
        // frozen once something touched.
        if (mPhase == FallPhase::Reacting || mPhase == FallPhase::Bracing)
        {
            float x = 0.0f, y = 0.0f;
            const float speed = horizontalLength(input.mComVelocity);
            const float tilt = horizontalLength(input.mTorsoUp);
            if (speed >= s.mDirectionSpeed)
            {
                x = input.mComVelocity.mX / speed;
                y = input.mComVelocity.mY / speed;
            }
            else if (tilt > 0.1f)
            {
                x = input.mTorsoUp.mX / tilt;
                y = input.mTorsoUp.mY / tilt;
            }
            if (x != 0.0f || y != 0.0f)
            {
                if (!mHasDirection)
                {
                    mDirectionX = x;
                    mDirectionY = y;
                    mHasDirection = true;
                }
                else
                {
                    const float k = std::min(1.0f, s.mDirectionRate * dt);
                    mDirectionX += (x - mDirectionX) * k;
                    mDirectionY += (y - mDirectionY) * k;
                    const float l = std::hypot(mDirectionX, mDirectionY);
                    if (l > 1e-6f)
                    {
                        mDirectionX /= l;
                        mDirectionY /= l;
                    }
                }
            }
        }

        // Phases.
        if (mPhase == FallPhase::Reacting && mTime >= s.mReactionDelay)
            mPhase = FallPhase::Bracing;
        // With per-arm yield a hand's touch is the catch, not the end of it.
        const bool contact = s.mArmYield > 0.5f ? input.mBodyContact : input.mNonFootContact;
        if (input.mFallen && (mPhase == FallPhase::Reacting || mPhase == FallPhase::Bracing)
            && (contact || mTime >= s.mMaximumFallTime + mAlertTime))
        {
            mPhase = FallPhase::Impact;
            mImpactTime = 0.0f;
        }
        else if (mImpactTime >= 0.0f)
            mImpactTime += dt;
        if (mPhase == FallPhase::Impact)
        {
            mSlowTime = speed3 < s.mSettleSpeed ? mSlowTime + dt : 0.0f;
            if (mImpactTime >= s.mAbsorbTime && mSlowTime >= s.mSettleHold)
                mPhase = FallPhase::Settled;
        }

        FallProtectCommand command;
        command.mPhase = mPhase;
        command.mTime = mTime;
        command.mImpactTime = mImpactTime;
        command.mDirection = { mDirectionX, mDirectionY, 0.0f };
        if (!mHasStartForward)
        {
            const float l = horizontalLength(input.mBodyForward);
            if (l > 1e-3f)
            {
                mStartForwardX = input.mBodyForward.mX / l;
                mStartForwardY = input.mBodyForward.mY / l;
            }
            mHasStartForward = true;
        }
        const FallVec3 startForward{ mStartForwardX, mStartForwardY, 0.0f };
        if (!mKindLatched && input.mLegSwept && horizontalLength(input.mBlowDirection) > 1e-3f && s.mLatchKind > 0.5f)
        {
            mKind = classifyFall({ -input.mBlowDirection.mX, -input.mBlowDirection.mY, 0.0f }, startForward);
            mKindLatched = true;
        }
        if (!mKindLatched)
        {
            mKind = classifyFall(command.mDirection, startForward);
            // Latched at once when the body already moves decisively (a push:
            // a limp kind is then limp from the first step), else when
            // bracing starts.
            const bool decisive
                = s.mLatchOnPush > 0.5f && mHasDirection && horizontalLength(input.mComVelocity) >= s.mDirectionSpeed;
            mKindLatched = s.mLatchKind > 0.5f && (decisive || mPhase != FallPhase::Reacting);
        }
        command.mKind = mKind;
        if (mKindLatched && input.mFallen)
            command.mLimp = (mKind == FallKind::Side && s.mProtectSide < 0.5f)
                || (mKind == FallKind::Forward && !input.mLegSwept && s.mProtectForwardPushed < 0.5f);

        float reach = s.mForwardReach, head = s.mHeadForward, spine = s.mSpineForward;
        if (command.mKind == FallKind::Backward)
        {
            reach = s.mBackwardReach;
            head = s.mHeadBackward;
            spine = s.mSpineBackward;
        }
        else if (command.mKind == FallKind::Side)
        {
            reach = s.mSideReach;
            head = s.mHeadSide;
            spine = s.mSpineSide;
        }
        const float down = std::sqrt(std::max(0.0f, 1.0f - reach * reach));
        command.mArmDirection = { mDirectionX * reach, mDirectionY * reach, -down };
        if (input.mHasHead && s.mAimLanding > 0.0f)
        {
            // The head's landing point: carried by its velocity for the time
            // it takes to reach the floor, the hands a little further along.
            const float height = std::max(0.0f, input.mHead.mZ - input.mFloorHeight);
            const float drop = std::max(s.mMinimumDropSpeed, -input.mHeadVelocity.mZ);
            const float time = std::min(s.mMaximumLookahead, height / drop);
            const float x = input.mHead.mX + input.mHeadVelocity.mX * time + mDirectionX * s.mHandLead;
            const float y = input.mHead.mY + input.mHeadVelocity.mY * time + mDirectionY * s.mHandLead;
            const float dx = x - input.mShoulders.mX;
            const float dy = y - input.mShoulders.mY;
            const float dz = input.mFloorHeight - input.mShoulders.mZ;
            const float length = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (length > 1e-3f && std::isfinite(length))
            {
                const float k = std::clamp(s.mAimLanding, 0.0f, 1.0f);
                FallVec3 aim{ command.mArmDirection.mX * (1.0f - k) + dx / length * k,
                    command.mArmDirection.mY * (1.0f - k) + dy / length * k,
                    command.mArmDirection.mZ * (1.0f - k) + dz / length * k };
                const float l = std::sqrt(aim.mX * aim.mX + aim.mY * aim.mY + aim.mZ * aim.mZ);
                if (l > 1e-3f)
                    command.mArmDirection = { aim.mX / l, aim.mY / l, aim.mZ / l };
            }
        }

        // Arm weights: both arms forward and back; the far arm less on a side
        // fall (right = forward x up).
        float armWeight = 0.0f;
        if (mPhase == FallPhase::Bracing)
            armWeight = s.mArmRamp > 0.0f ? std::min(1.0f, (mTime - s.mReactionDelay) / s.mArmRamp) : 1.0f;
        else if (mPhase == FallPhase::Impact || mPhase == FallPhase::Settled)
            armWeight = 1.0f;
        std::array<float, 2> weights{ 1.0f, 1.0f };
        if (command.mKind == FallKind::Side)
        {
            const float rightX = mStartForwardY, rightY = -mStartForwardX;
            const bool toRight = mDirectionX * rightX + mDirectionY * rightY > 0.0f;
            weights[toRight ? 0 : 1] = s.mFarArmSide;
        }
        const float risk = input.mFallen ? 1.0f : std::clamp(input.mRisk, 0.0f, 1.0f);
        command.mArmWeight = { armWeight * weights[0] * risk, armWeight * weights[1] * risk };
        command.mFallen = input.mFallen;

        // Each arm's own axis: the shared aim turned by the fall's spread
        // about world up, then bent toward what its probe found.
        const float variation = std::clamp(s.mVariation, 0.0f, 1.0f);
        for (std::size_t side = 0; side < 2; ++side)
        {
            const FallVec3 a = command.mArmDirection;
            const float angle = 0.15f * variation * mReachSpread[side];
            const float c = std::cos(angle), sn = std::sin(angle);
            FallVec3 axis{ a.mX * c - a.mY * sn, a.mX * sn + a.mY * c, a.mZ };
            const FallArmInput& arm = input.mArms[side];
            if (arm.mHasProbe && s.mProbeAim > 0.0f)
            {
                const FallVec3 to = unit({ arm.mProbeHit.mX - arm.mShoulder.mX, arm.mProbeHit.mY - arm.mShoulder.mY,
                                             arm.mProbeHit.mZ - arm.mShoulder.mZ },
                    axis);
                const float k = std::clamp(s.mProbeAim, 0.0f, 1.0f);
                axis = unit({ axis.mX * (1.0f - k) + to.mX * k, axis.mY * (1.0f - k) + to.mY * k,
                                axis.mZ * (1.0f - k) + to.mZ * k },
                    axis);
            }
            command.mArmDirections[side] = axis;
            command.mArmTouched[side] = mArmTouchTime[side] >= 0.0f;
        }
        if (!input.mFallen)
        {
            command.mArmStiffnesses = { command.mArmStiffness, command.mArmStiffness };
            return command; // the arms only: no bends, full stiffness
        }
        const float poseWeight = armWeight;
        const float bendSpread = 0.25f * variation * poseWeight;
        const bool backward = command.mKind == FallKind::Backward;
        command.mHeadPitch = head * poseWeight + mBendSpread[3] * bendSpread;
        command.mSpineCurl = spine * poseWeight + mBendSpread[0] * bendSpread;
        command.mKneeBend = (backward ? s.mKneeBendBackward : s.mKneeBend) * poseWeight
            + std::abs(mBendSpread[2]) * bendSpread;
        command.mHipBend = (backward ? s.mHipBendBackward : s.mHipBend) * poseWeight + mBendSpread[1] * bendSpread;
        // A hand down: the head turns further from the ground over 0.15 s.
        const float touched = std::max(mArmTouchTime[0], mArmTouchTime[1]);
        if (s.mContactTuck > 0.0f && touched >= 0.0f && head != 0.0f)
            command.mHeadPitch += (head > 0.0f ? 1.0f : -1.0f) * s.mContactTuck * std::min(1.0f, touched / 0.15f);

        const bool forward = command.mKind == FallKind::Forward;
        const float braceUpper = forward ? s.mForwardUpper : s.mBraceUpper;
        const float braceLegs = forward ? s.mForwardLegs : s.mBraceLegs;
        switch (mPhase)
        {
            case FallPhase::Reacting:
                command.mArmStiffness = s.mReactArms;
                command.mUpperStiffness = forward ? s.mForwardUpper : s.mReactUpper;
                command.mLegStiffness = forward ? s.mForwardLegs : s.mReactLegs;
                break;
            case FallPhase::Bracing:
                command.mArmStiffness = s.mBraceArms;
                command.mUpperStiffness = braceUpper;
                command.mLegStiffness = braceLegs;
                break;
            case FallPhase::Impact:
            {
                float t = s.mAbsorbTime > 0.0f ? mImpactTime / s.mAbsorbTime : 1.0f;
                // By the speed: braced while it still moves fast, the time
                // ramp (stretched x3) the bound.
                if (s.mToneSpeed > 0.0f)
                    t = std::max(1.0f - mTone, s.mAbsorbTime > 0.0f ? mImpactTime / (3.0f * s.mAbsorbTime) : 1.0f);
                command.mArmStiffness = mix(s.mBraceArms, s.mImpactArms, t);
                command.mUpperStiffness = mix(braceUpper, s.mImpactUpper, t);
                command.mLegStiffness = mix(braceLegs, s.mImpactLegs, t);
                break;
            }
            case FallPhase::Settled:
                command.mArmStiffness = command.mUpperStiffness = command.mLegStiffness = s.mSettleTone;
                break;
        }
        if (input.mLegSwept && mPhase != FallPhase::Settled)
        {
            command.mUpperStiffness = std::min(command.mUpperStiffness, s.mSweptTone);
            command.mLegStiffness = std::min(command.mLegStiffness, s.mSweptTone);
        }
        // Each arm: a hand that touched yields over the absorb time, never
        // stiffer than the phase allows.
        for (std::size_t side = 0; side < 2; ++side)
        {
            float k = command.mArmStiffness;
            if (s.mArmYield > 0.5f && mArmTouchTime[side] >= 0.0f && mPhase != FallPhase::Settled)
                k = std::min(k,
                    mix(s.mBraceArms, s.mImpactArms, s.mAbsorbTime > 0.0f ? mArmTouchTime[side] / s.mAbsorbTime : 1.0f));
            command.mArmStiffnesses[side] = k;
        }
        return command;
    }
}
