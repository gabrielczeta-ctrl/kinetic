#include <ragdollcore/weaponswing.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    PunchCycleOutput stepPunchCycle(PunchCycleState& state, bool held, bool returned, float duration)
    {
        PunchCycleOutput output;
        if (!std::isfinite(duration) || duration <= 0.0f)
            return output;
        output.mStarted = state.mPendingPress || (held && state.mReturning && returned);
        if (output.mStarted)
        {
            state.mPendingPress = false;
            state.mReturning = false;
            state.mExtensionRemaining = 0.16f; // existing short-click extension
        }
        output.mExtending = state.mExtensionRemaining > 0.0f;
        state.mExtensionRemaining = std::max(0.0f, state.mExtensionRemaining - duration);
        if (output.mExtending && state.mExtensionRemaining == 0.0f)
            state.mReturning = true;
        return output;
    }

    namespace
    {
        bool isFinite(float value)
        {
            return std::isfinite(value);
        }

        bool isFinite(JPH::Vec3Arg value)
        {
            return isFinite(value.GetX()) && isFinite(value.GetY()) && isFinite(value.GetZ());
        }

        bool validProfile(const WeaponSwingProfile& profile)
        {
            return isFinite(profile.mReach) && profile.mReach > 0.0f && isFinite(profile.mAimToTravel)
                && isFinite(profile.mMaximumSpeed) && profile.mMaximumSpeed > 0.0f && isFinite(profile.mReturnRate)
                && profile.mReturnRate >= 0.0f && isFinite(profile.mRestOffset) && isFinite(profile.mWeight)
                && isFinite(profile.mBlendRate);
        }
    }

    void constrainWeaponSwingToReach(WeaponSwingState& state, JPH::Vec3Arg solvedOffset)
    {
        if (!isFinite(solvedOffset) || !isFinite(state.mOffset) || !isFinite(state.mDesiredOffset))
            return;
        for (const unsigned axis : { 0u, 2u })
        {
            const float commanded = state.mOffset[axis];
            const float solved = solvedOffset[axis];
            if (commanded * (commanded - solved) <= 0.0f)
                continue;
            const float bounded = commanded > 0 ? std::max(0.0f, solved) : std::min(0.0f, solved);
            if (axis == 0) state.mOffset.SetX(bounded);
            else state.mOffset.SetZ(bounded);
            const float desired = state.mDesiredOffset[axis];
            if (commanded * (desired - bounded) > 0.0f)
            {
                if (axis == 0) state.mDesiredOffset.SetX(bounded);
                else state.mDesiredOffset.SetZ(bounded);
            }
        }
    }

    WeaponSwingOutput stepWeaponSwing(
        WeaponSwingState& state, const WeaponSwingInput& input, const WeaponSwingProfile& profile)
    {
        WeaponSwingOutput output;
        if (!validProfile(profile) || !isFinite(input.mAimRight) || !isFinite(input.mAimUp)
            || !isFinite(input.mMaximumHandDistance) || input.mMaximumHandDistance < 0.0f
            || !isFinite(input.mMinimumHandDistance) || input.mMinimumHandDistance < 0.0f
            || (input.mMaximumHandDistance > 0 && input.mMinimumHandDistance > input.mMaximumHandDistance)
            || !isFinite(input.mDuration) || input.mDuration <= 0.0f || !isFinite(state.mOffset) || !isFinite(state.mDesiredOffset)
            || !isFinite(input.mMaximumInputLead) || input.mMaximumInputLead < 0.0f)
        {
            output.mHandOffset = profile.mRestOffset;
            return output;
        }

        output.mStarted = input.mHeld && !state.mHeld;
        state.mHeld = input.mHeld;
        JPH::Vec3 rest = profile.mRestOffset;
        if (input.mMaximumHandDistance > 0 && rest.Length() >= input.mMaximumHandDistance)
            rest *= 0.95f * input.mMaximumHandDistance / rest.Length();
        const JPH::Vec3 previous = state.mOffset;
        JPH::Vec3 wanted = input.mHeld ? state.mDesiredOffset : state.mOffset;
        // Depth is a geometric clearance, not accumulated mouse input. Rebuild
        // it from the requested planar coordinates so reversal has no depth
        // backlog to unwind and release can return to the activation point.
        if (input.mMinimumHandDistance > 0)
            wanted.SetY(0);
        if (input.mHeld)
        {
            // Aim moves the hand across the actor's frontal plane: right is
            // the actor's right, up is up. Depth is not driven, because a
            // mouse has two axes and inventing the third from them is how a
            // swing stops reading as the arc the player drew.
            wanted += JPH::Vec3(input.mAimRight, 0.0f, input.mAimUp) * profile.mAimToTravel;
            const float distance = wanted.Length();
            if (distance > profile.mReach)
                wanted *= profile.mReach / distance;
        }
        else
        {
            // Not a snap back to rest. The arm carries what it was doing and
            // settles, so letting go of the button ends a swing rather than
            // cancelling it mid-arc.
            const float decay = 1.0f - std::exp(-profile.mReturnRate * input.mDuration);
            wanted -= wanted * decay;
        }

        if (!isFinite(wanted))
        {
            output.mHandOffset = rest;
            return output;
        }
        if (input.mMaximumHandDistance > 0)
        {
            // Intersect the stroke with the arm sphere at constant depth.
            // Segment clipping preserves the undriven axis. Clamp the input
            // accumulator itself so reversing does not unwind impossible reach.
            const float radiusSquared = std::max(0.0f,
                input.mMaximumHandDistance * input.mMaximumHandDistance - rest.GetY() * rest.GetY());
            JPH::Vec3 start = rest + state.mDesiredOffset;
            start.SetY(0);
            if (start.LengthSq() > radiusSquared)
                start *= std::sqrt(radiusSquared / start.LengthSq());
            JPH::Vec3 end = rest + wanted;
            end.SetY(0);
            if (end.LengthSq() > radiusSquared)
            {
                const auto direction = end - start;
                const float a = direction.LengthSq();
                const float b = start.Dot(direction);
                const float c = start.LengthSq() - radiusSquared;
                const float fraction = a > 1e-8f
                    ? std::clamp((-b + std::sqrt(std::max(0.0f, b * b - a * c))) / a, 0.0f, 1.0f) : 0.0f;
                end = start + direction * fraction;
                end.SetY(rest.GetY());
                wanted = end - rest;
            }
        }
        if (input.mHeld && input.mMaximumInputLead > 0.0f)
        {
            // Bound the lag the mouse may bank ahead of the hand. The arm
            // keeps its speed limit, but the stroke cannot leave a backlog
            // that keeps the weapon moving after the mouse has stopped or
            // reversed.
            const JPH::Vec3 lead = wanted - previous;
            const float leadDistance = lead.Length();
            if (isFinite(leadDistance) && leadDistance > input.mMaximumInputLead)
            {
                wanted = previous + lead * (input.mMaximumInputLead / leadDistance);
                output.mSpeedLimited = true;
            }
        }
        state.mDesiredOffset = wanted;

        // The arm's speed limit, applied to the commanded point rather than to
        // the input. The mouse may ask for anything; what the hand is told to
        // do stays inside what an arm could plausibly deliver, and the gap
        // between the two is exactly the weight the player feels.
        JPH::Vec3 step = wanted - previous;
        const float travel = step.Length();
        const float limit = profile.mMaximumSpeed * input.mDuration;
        if (isFinite(travel) && travel > limit && travel > 0.0f)
        {
            step *= limit / travel;
            output.mSpeedLimited = true;
        }
        if (isFinite(profile.mMaximumAcceleration) && profile.mMaximumAcceleration > 0.0f && isFinite(state.mVelocity))
        {
            // Speed toward the goal is bounded by what can still be braked
            // over the remaining distance, then the change of velocity by the
            // acceleration budget.
            const float accel = profile.mMaximumAcceleration;
            JPH::Vec3 desired = JPH::Vec3::sZero();
            if (travel > 1e-4f)
                desired = (wanted - previous) / travel
                    * std::min({ profile.mMaximumSpeed, std::sqrt(2.0f * accel * travel), travel / input.mDuration });
            JPH::Vec3 change = desired - state.mVelocity;
            const float budget = accel * input.mDuration;
            if (change.Length() > budget)
            {
                change *= budget / change.Length();
                output.mSpeedLimited = true;
            }
            step = (state.mVelocity + change) * input.mDuration;
        }

        JPH::Vec3 offset = previous + step;
        if (input.mMinimumHandDistance > 0)
        {
            const float radiusSquared = input.mMinimumHandDistance * input.mMinimumHandDistance;
            const auto clearInnerReach = [&](JPH::Vec3 point) {
                // An attack's inner reach is on the forward hemisphere. A
                // hand withdrawn behind the shoulder must not satisfy reach
                // merely because its negative depth has sufficient length.
                if (point.LengthSq() < radiusSquared || point.GetY() < 0)
                    point.SetY(std::sqrt(std::max(0.0f,
                        radiusSquared - point.GetX() * point.GetX() - point.GetZ() * point.GetZ())));
                return point;
            };
            const auto start = rest + previous;
            const auto end = rest + wanted;
            const auto pointAt = [&](float fraction) {
                return clearInnerReach(start + (end - start) * fraction);
            };
            auto point = pointAt(1);
            if ((point - start).LengthSq() > limit * limit)
            {
                // A straight speed-limited chord can cross the inner sphere.
                // Limit travel on the cleared path itself instead.
                float low = 0, high = 1;
                for (int i = 0; i < 28; ++i)
                {
                    const float mid = (low + high) * 0.5f;
                    if ((pointAt(mid) - start).LengthSq() <= limit * limit)
                        low = mid;
                    else
                        high = mid;
                }
                point = pointAt(low);
                // An invalid activation point can already be inside the inner
                // sphere. Preserve the speed bound while moving out of it.
                const auto travel = point - start;
                if (travel.LengthSq() > limit * limit)
                    point = start + travel.Normalized() * limit;
                output.mSpeedLimited = true;
            }
            offset = point - rest;
        }
        if (!isFinite(offset))
            offset = previous;
        state.mOffset = offset;
        state.mVelocity = (offset - previous) / input.mDuration;
        if (!isFinite(state.mVelocity))
            state.mVelocity = JPH::Vec3::sZero();

        // A swing is running while the button is held, and stays running
        // while the target is still travelling back, so the arm is driven
        // through the follow-through instead of being dropped at release.
        state.mSwinging = input.mHeld || !state.mOffset.IsNearZero(1e-3f);

        output.mHandOffset = rest + state.mOffset;
        output.mSpeed = state.mVelocity.Length();
        output.mSwinging = state.mSwinging;
        output.mWeight = state.mSwinging ? std::clamp(profile.mWeight, 0.0f, 1.0f) : 0.0f;
        if (!isFinite(output.mSpeed))
            output.mSpeed = 0.0f;
        return output;
    }
}
