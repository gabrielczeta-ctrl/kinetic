#include <ragdollcore/fiststeer.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace RagdollCore
{
    namespace
    {
        constexpr float sTau = 2.0f * std::numbers::pi_v<float>;
        // A press shows: the resting held fist comes this far out of the guard.
        const JPH::Vec3 sReadyLift(0.0f, 4.0f, 1.0f);
        constexpr float sPendingWait = 0.6f;

        float wrap(float a)
        {
            a = std::fmod(a, sTau);
            return a < 0.0f ? a + sTau : a;
        }

        // FIST-12 (user 11-16: "down up works for uppercut"): with mDrag a
        // draw straight down winds the uppercut as the down-and-out diagonal
        // does (the cross, whose wind-up lies straight down on the wheel, is
        // the click's). The right hand's sense.
        float drawAngle(float angle, bool drag)
        {
            angle = wrap(std::isfinite(angle) ? angle : 0.0f);
            constexpr float eighth = sTau / 8.0f;
            if (drag && angle >= 5.5f * eighth && angle < 7.0f * eighth)
                return 7.0f * eighth;
            return angle;
        }

        // The wind-up of one line at s (two keys or more), eased as the punches are.
        void windAt(const AttackLine& line, float s, JPH::Vec3& hand, JPH::Vec3& pole, float& twistDegrees)
        {
            const auto& keys = line.mWind;
            if (keys.empty())
                return;
            s = std::clamp(s, 0.0f, 1.0f);
            std::size_t i = 0;
            while (i + 2 < keys.size() && s > keys[i + 1].mS)
                ++i;
            const ChamberKey& a = keys[i];
            const ChamberKey& b = keys[std::min(i + 1, keys.size() - 1)];
            const float t = std::clamp((s - a.mS) / std::max(1e-4f, b.mS - a.mS), 0.0f, 1.0f);
            hand = a.mHand + (b.mHand - a.mHand) * t;
            pole = (a.mPole + (b.mPole - a.mPole) * t).NormalizedOr(a.mPole);
            twistDegrees = a.mTwistDegrees + (b.mTwistDegrees - a.mTwistDegrees) * t;
        }
    }

    std::size_t fistLineIndex(PunchKind kind)
    {
        const std::string_view name = fistLineName(kind);
        const WheelLines& lines = fistLines();
        for (std::size_t i = 0; i < lines.size(); ++i)
            if (lines[i].mName == name)
                return i;
        return 0;
    }

    PunchKind fistLineKind(const AttackLine& line)
    {
        if (line.mName == "hook" || line.mName == "backfist" || line.mName == "overhand hook")
            return PunchKind::Hook;
        if (line.mName == "uppercut")
            return PunchKind::Uppercut;
        if (line.mName == "body")
            return PunchKind::Body;
        return PunchKind::Straight;
    }

    std::size_t fistLineForMovement(float side, float forward, bool crouched)
    {
        const WheelLines& lines = fistLines();
        const auto named = [&](std::string_view name) {
            for (std::size_t i = 0; i < lines.size(); ++i)
                if (lines[i].mName == name)
                    return i;
            return std::size_t(0);
        };
        if (!std::isfinite(side) || !std::isfinite(forward))
            side = forward = 0.0f;
        constexpr float pressed = 0.3f;
        if (std::abs(side) > pressed && std::abs(side) >= std::abs(forward))
            return named("hook");
        if (forward < -pressed)
            return named("uppercut");
        if (forward > pressed)
            return named("long cross");
        return named(crouched ? "body" : "cross");
    }

    std::size_t fistLineForAim(int zone, bool charged, bool crouched)
    {
        const WheelLines& lines = fistLines();
        const auto named = [&](std::string_view name) {
            for (std::size_t i = 0; i < lines.size(); ++i)
                if (lines[i].mName == name)
                    return i;
            return std::size_t(0);
        };
        if (charged)
            return named(zone > 0 ? "hook" : zone < 0 ? "uppercut" : crouched ? "body" : "cross");
        return named(zone < 0 || crouched ? "body" : "cross");
    }

    FistSteerHold fistHoldPlace(FistHand hand, float angle, float amount)
    {
        FistSteerHold hold;
        hold.mAngle = wrap(std::isfinite(angle) ? angle : 0.0f);
        hold.mAmount = std::isfinite(amount) ? std::clamp(amount, 0.0f, 1.0f) : 0.0f;
        const WheelLines& lines = fistLines();
        const WheelSlot slot = wheelSlot(hold.mAngle);
        JPH::Vec3 handA = JPH::Vec3::sZero(), handB = JPH::Vec3::sZero();
        JPH::Vec3 poleA = -JPH::Vec3::sAxisZ(), poleB = -JPH::Vec3::sAxisZ();
        float twistA = 0.0f, twistB = 0.0f;
        windAt(lines[slot.mFirst], hold.mAmount, handA, poleA, twistA);
        windAt(lines[slot.mSecond], hold.mAmount, handB, poleB, twistB);
        hold.mHand = handA + (handB - handA) * slot.mWeight + sReadyLift * (1.0f - hold.mAmount);
        hold.mPole = (poleA + (poleB - poleA) * slot.mWeight).NormalizedOr(poleA);
        hold.mTwist = (twistA + (twistB - twistA) * slot.mWeight) * (sTau / 360.0f);
        if (hand == FistHand::Left)
        {
            hold.mHand = JPH::Vec3(-hold.mHand.GetX(), hold.mHand.GetY(), hold.mHand.GetZ());
            hold.mPole = JPH::Vec3(-hold.mPole.GetX(), hold.mPole.GetY(), hold.mPole.GetZ());
            hold.mTwist = -hold.mTwist;
        }
        return hold;
    }

    void FistSteer::stepHand(FistHand hand, const FistSteerPress& press, const FistSteerInput& input,
        const FistSteerSettings& settings, std::optional<FistSteerHold>& hold, std::optional<FistSteerPunch>& punch)
    {
        Hand& state = mHands[index(hand)];
        const float dt = std::isfinite(input.mDt) ? std::max(0.0f, input.mDt) : 0.0f;
        const float counts = std::max(20.0f, settings.mCounts);
        const float mirror = hand == FistHand::Left ? -1.0f : 1.0f;
        // FIST-12: mDrag winds up by the mouse (the weapons' charge); only
        // its click is the crosshair's.
        const bool drag = settings.mDrag;
        const bool feet = (settings.mFeet || settings.mAim) && !drag;
        const auto straight = [&](bool wound, float power) {
            FistSteerPunch p;
            p.mHand = hand;
            p.mKind = input.mCrouched ? PunchKind::Body : PunchKind::Straight;
            p.mLine = fistLineIndex(p.mKind);
            if (drag && settings.mAim)
            {
                p.mLine = fistLineForAim(input.mAimZone, false, input.mCrouched);
                p.mKind = fistLineKind(fistLines()[p.mLine]);
            }
            p.mPower = power;
            p.mWound = wound;
            return p;
        };
        // The punch of a wound draw (counts): the line that lies that way,
        // as hard as it is drawn, from where the fist is shown.
        const auto drawn = [&](float x, float y) {
            FistSteerPunch p;
            p.mHand = hand;
            const WheelSlot slot = wheelSlot(drawAngle(std::atan2(y, x), settings.mDrag));
            p.mLine = slot.mWeight < 0.5f ? slot.mFirst : slot.mSecond;
            p.mKind = fistLineKind(fistLines()[p.mLine]);
            p.mPower = std::clamp((std::hypot(x, y) / counts - settings.mStraight) / std::max(1e-3f, 1.0f - settings.mStraight),
                0.0f, 1.0f);
            p.mWound = true;
            return p;
        };

        std::optional<FistSteerPunch> thrown;
        if (input.mBlocked || press.mCancel)
        {
            state.mHolding = false;
            state.mPending.reset();
            state.mSwung = 0;
        }
        else
        {
            if (press.mHeld && !state.mWasHeld)
            {
                state.mHolding = true;
                state.mTime = std::isfinite(press.mHoldLead) ? std::max(0.0f, press.mHoldLead) : 0.0f;
                state.mTravel = 0.0f;
                state.mX = state.mY = state.mShownX = state.mShownY = 0.0f;
                state.mDrawnCount = state.mDrawnNext = 0;
                state.mStruck = false;
                state.mSwung = 0;
            }
            const auto feetPick = [&](bool charged) {
                return settings.mAim ? fistLineForAim(input.mAimZone, charged, input.mCrouched)
                                     : fistLineForMovement(input.mMoveSide, input.mMoveForward, input.mCrouched);
            };
            const std::size_t feetLine = feet ? feetPick(state.mTime >= settings.mTap) : 0;
            const float feetAmount = feet
                ? std::clamp((state.mTime - settings.mTap) / std::max(0.05f, settings.mCharge), 0.0f, 1.0f)
                : 0.0f;
            const auto feetPunch = [&](bool wound) {
                FistSteerPunch p;
                p.mHand = hand;
                p.mLine = feetPick(wound);
                p.mKind = fistLineKind(fistLines()[p.mLine]);
                p.mPower = wound ? feetAmount : 0.0f;
                p.mWound = wound;
                return p;
            };
            if (feet && state.mHolding && press.mHeld)
            {
                // The wind-up of the punch the feet pick, by how long it is held.
                state.mTime += dt;
                const float angle = fistLines()[feetLine].angle();
                state.mX = std::cos(angle) * feetAmount * counts;
                state.mY = std::sin(angle) * feetAmount * counts;
            }
            else if (feet && state.mHolding)
            {
                state.mHolding = false;
                thrown = feetPunch(state.mTime >= settings.mTap);
                if (thrown->mWound)
                    thrown->mFrom = fistHoldPlace(hand, drawAngle(std::atan2(state.mShownY, state.mShownX), settings.mDrag),
                        std::hypot(state.mShownX, state.mShownY) / counts)
                                        .mHand;
            }
            else if (feet)
            {
                if (press.mTap && !thrown && state.mSinceThrown > 0.3f)
                    thrown = feetPunch(false);
            }
            else if (state.mHolding && press.mHeld)
            {
                state.mTime += dt;
                const float dx = std::isfinite(press.mDragRight) ? press.mDragRight * mirror : 0.0f;
                const float dy = std::isfinite(press.mDragUp) ? press.mDragUp : 0.0f;
                state.mTravel += std::hypot(dx, dy);
                state.mStill = dt > 0.0f && std::hypot(dx, dy) / dt < settings.mSettle ? state.mStill + dt : 0.0f;
                if (state.mSwung != 0 && state.mStill >= settings.mSettleTime)
                    state.mSettled = true;
                if (state.mSwung != 0)
                {
                    // The swing's punch is on its way: its follow-through draws nothing.
                    state.mX = state.mY = 0.0f;
                    state.mDrawnCount = state.mDrawnNext = 0;
                }
                else
                {
                    state.mX += dx;
                    state.mY += dy;
                    // The draw stops at the full wind-up: pushing on does not bank travel.
                    const float length = std::hypot(state.mX, state.mY);
                    if (length > counts)
                    {
                        state.mX *= counts / length;
                        state.mY *= counts / length;
                    }
                }
                if (drag && state.mSwung == 0 && std::isfinite(settings.mSwing) && settings.mSwing > 0.0f)
                {
                    // The swing: a stroke back against a wound draw throws it,
                    // from the deepest draw of the last `mSwingTime` it runs against.
                    const float window = std::isfinite(settings.mSwingTime) ? std::clamp(settings.mSwingTime, 0.02f, 0.5f) : 0.1f;
                    const Hand::Drawn* from = nullptr;
                    float deepest = 0.0f;
                    for (std::size_t i = 0; i < state.mDrawnCount; ++i)
                    {
                        const Hand::Drawn& was = state.mDrawn[i];
                        const float depth = std::hypot(was.mX, was.mY);
                        if (state.mTime - was.mAt > window + 1e-4f || depth < 1.0f || depth / counts < settings.mStraight)
                            continue;
                        const float vx = state.mX - was.mX, vy = state.mY - was.mY;
                        const float against = -(vx * was.mX + vy * was.mY) / depth;
                        const float across = std::abs(vx * was.mY - vy * was.mX) / depth;
                        if (against >= settings.mSwing && across <= 1.2f * against && depth > deepest)
                        {
                            deepest = depth;
                            from = &was;
                        }
                    }
                    if (from)
                    {
                        thrown = drawn(from->mX, from->mY);
                        thrown->mFrom = fistHoldPlace(hand, drawAngle(std::atan2(state.mShownY, state.mShownX), settings.mDrag),
                            std::hypot(state.mShownX, state.mShownY) / counts)
                                            .mHand;
                        state.mStruck = true;
                        state.mSwung = 1;
                        state.mSettled = false;
                        state.mStill = 0.0f;
                        state.mX = state.mY = 0.0f;
                        state.mDrawnCount = state.mDrawnNext = 0;
                    }
                    else
                    {
                        state.mDrawn[state.mDrawnNext] = { state.mTime, state.mX, state.mY };
                        state.mDrawnNext = (state.mDrawnNext + 1) % state.mDrawn.size();
                        state.mDrawnCount = std::min(state.mDrawnCount + 1, state.mDrawn.size());
                    }
                }
            }
            else if (state.mHolding)
            {
                // Let go: a tap, or the fist thrown from where it is.
                state.mHolding = false;
                const float amount = std::hypot(state.mX, state.mY) / counts;
                if (state.mStruck && amount < settings.mStraight)
                {
                    // The swing threw this hold's punch; nothing is wound since.
                }
                else if (state.mTime < settings.mTap && state.mTravel < settings.mTapCounts)
                    thrown = straight(false, 0.0f);
                else if (drag && settings.mSwing > 0.0f && !settings.mRelease)
                {
                    // The weapons' rule: only the swing delivers. A sloppy
                    // click (held a little long, barely moved) is still the click.
                    if (!state.mStruck && state.mTime < settings.mClick && state.mTravel < settings.mTapCounts)
                        thrown = straight(false, 0.0f);
                }
                else if (amount < settings.mStraight)
                    thrown = straight(true, 0.0f);
                else
                    thrown = drawn(state.mX, state.mY);
                if (thrown && thrown->mWound)
                    thrown->mFrom = fistHoldPlace(hand, drawAngle(std::atan2(state.mShownY, state.mShownX), settings.mDrag),
                        std::hypot(state.mShownX, state.mShownY) / counts)
                                        .mHand;
            }
            if (!feet && press.mTap && !state.mHolding && !thrown && state.mSinceThrown > 0.3f)
                thrown = straight(false, 0.0f);
        }
        state.mWasHeld = press.mHeld;

        // A punch waits for its busy hand; a newer one replaces it.
        state.mSinceThrown += dt;
        if (thrown)
        {
            state.mSinceThrown = 0.0f;
            state.mPending = thrown;
            state.mPendingAge = 0.0f;
        }
        else if (state.mPending && (state.mPendingAge += dt) > sPendingWait)
            state.mPending.reset();
        if (state.mPending && !press.mBusy)
        {
            punch = state.mPending;
            state.mPending.reset();
        }
        // The swing's punch: started (or given up), its hand busy (0.3 s to
        // show it), then done - the hand winds up again.
        if (state.mSwung == 1 && !state.mPending)
        {
            state.mSwung = 2;
            state.mSwungAge = 0.0f;
        }
        else if (state.mSwung == 2)
        {
            if (press.mBusy)
                state.mSwung = 3;
            else if ((state.mSwungAge += dt) > 0.3f)
                state.mSwung = 4;
        }
        else if (state.mSwung == 3 && !press.mBusy)
            state.mSwung = 4;
        // ... and the mouse at rest: the follow-through is over.
        if (state.mSwung == 4 && (state.mSettled || !press.mHeld))
            state.mSwung = 0;

        if (state.mHolding && !press.mBusy && state.mSwung == 0 && !(feet && state.mTime < settings.mTap))
        {
            const float k = 1.0f - std::exp(-std::max(1.0f, settings.mRate) * dt);
            state.mShownX += (state.mX - state.mShownX) * k;
            state.mShownY += (state.mY - state.mShownY) * k;
            const float shownAngle = drawAngle(std::atan2(state.mShownY, state.mShownX), settings.mDrag);
            hold = fistHoldPlace(hand, shownAngle, std::hypot(state.mShownX, state.mShownY) / counts);
            // K2_FIST_CHARGE: the charge shows - further back along the
            // wind-up, the punching shoulder drawn back.
            if ((feet || drag) && hold && (settings.mChargeDraw > 0.0f || settings.mChargeCoil != 0.0f))
            {
                JPH::Vec3 rest = sReadyLift;
                if (hand == FistHand::Left)
                    rest = JPH::Vec3(-rest.GetX(), rest.GetY(), rest.GetZ());
                const JPH::Vec3 along = fistHoldPlace(hand, shownAngle, 1.0f).mHand
                    - (fistHoldPlace(hand, shownAngle, 0.0f).mHand - rest);
                if (along.LengthSq() > 1.0f && std::isfinite(settings.mChargeDraw))
                    hold->mHand += along.Normalized() * (std::max(0.0f, settings.mChargeDraw) * hold->mAmount);
                if (std::isfinite(settings.mChargeCoil))
                    hold->mTwist += mirror * settings.mChargeCoil * hold->mAmount;
            }
        }
        else
            state.mShownX = state.mShownY = 0.0f;
    }

    FistSteerOutput FistSteer::step(const FistSteerInput& input, const FistSteerSettings& settings)
    {
        FistSteerOutput output;
        stepHand(FistHand::Left, input.mLeft, input, settings, output.mLeft, output.mLeftPunch);
        stepHand(FistHand::Right, input.mRight, input, settings, output.mRight, output.mRightPunch);
        for (std::size_t side = 0; side < mHands.size(); ++side)
        {
            const Hand& hand = mHands[side];
            const auto& press = side == 0 ? input.mLeft : input.mRight;
            if (!hand.mHolding || press.mBusy || hand.mSwung != 0 || input.mBlocked || press.mCancel)
                continue;
            const bool charged = hand.mTime >= settings.mTap;
            if (settings.mDrag)
            {
                const float counts = std::max(20.0f, settings.mCounts);
                const float amount = std::min(1.0f, std::hypot(hand.mX, hand.mY) / counts);
                output.mCharge[side] = amount;
                output.mDrawRight[side] = (side == 0 ? -hand.mX : hand.mX) / counts;
                output.mDrawUp[side] = hand.mY / counts;
                std::size_t line = settings.mAim ? fistLineForAim(input.mAimZone, false, input.mCrouched)
                                                 : fistLineIndex(input.mCrouched ? PunchKind::Body : PunchKind::Straight);
                if (amount >= settings.mStraight)
                {
                    const WheelSlot slot = wheelSlot(drawAngle(std::atan2(hand.mY, hand.mX), settings.mDrag));
                    line = slot.mWeight < 0.5f ? slot.mFirst : slot.mSecond;
                }
                output.mKind[side] = fistLineKind(fistLines()[line]);
            }
            else if (settings.mAim || settings.mFeet)
            {
                output.mCharge[side] = std::clamp(
                    (hand.mTime - settings.mTap) / std::max(0.05f, settings.mCharge), 0.0f, 1.0f);
                const auto line = settings.mAim ? fistLineForAim(input.mAimZone, charged, input.mCrouched)
                    : fistLineForMovement(input.mMoveSide, input.mMoveForward, input.mCrouched);
                output.mKind[side] = fistLineKind(fistLines()[line]);
            }
        }
        return output;
    }
}
