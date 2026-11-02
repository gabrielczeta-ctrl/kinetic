#include <ragdollcore/staggerplan.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    int StaggerPlan::stepsFor(const StaggerStart& start, const StaggerSettings& s)
    {
        if (!s.mEnabled || !(start.mDv > 0.0f))
            return 0;
        int steps = 0;
        switch (start.mTier)
        {
            case ReactionTier::Absorb:
                return 0;
            case ReactionTier::Flinch:
                return start.mDv >= s.mFlinchDv ? 1 : 0;
            case ReactionTier::Stagger:
            case ReactionTier::Down:
                steps = 2 + (start.mDv >= s.mStaggerDv[1] ? 1 : 0) + (start.mDv >= s.mStaggerDv[2] ? 1 : 0);
                break;
        }
        if (start.mDv >= s.mHeavyDv)
            ++steps;
        // A tired body staggers further; a skilled one recovers sooner.
        if (start.mFatigue < 0.3f)
            ++steps;
        if (start.mSkill > 0.7f)
            --steps;
        return std::clamp(steps, 1, 5);
    }

    int StaggerPlan::budgetFor(const StaggerStart& start, const StaggerSettings& s)
    {
        if (!s.mGiveUp || start.mAgility < 0.0f)
            return std::max(1, s.mStepBudget);
        int budget = std::min(2 + static_cast<int>(start.mAgility / 25.0f), std::max(2, s.mStepBudget));
        if (start.mFatigue < 0.3f)
            --budget;
        if (start.mHealth < 0.4f)
            --budget;
        return std::max(1, budget);
    }

    namespace
    {
        // Horizontal distance from p to the segment a-b.
        float segmentDistance(JPH::Vec3 p, JPH::Vec3 a, JPH::Vec3 b)
        {
            const JPH::Vec3 ab = b - a;
            const float l = ab.LengthSq();
            const float t = l > 1e-6f ? std::clamp((p - a).Dot(ab) / l, 0.0f, 1.0f) : 0.0f;
            return (p - (a + ab * t)).Length();
        }
        JPH::Vec3 flat(JPH::RVec3Arg v)
        {
            return JPH::Vec3(static_cast<float>(v.GetX()), static_cast<float>(v.GetY()), 0.0f);
        }
    }

    bool StaggerPlan::begin(const StaggerStart& start, const StaggerSettings& settings)
    {
        int steps = stepsFor(start, settings);
        // Chained only from a stagger or harder (or a chained plan already
        // running); the count is then only the budget, the capture point
        // decides.
        const bool heavy = start.mTier == ReactionTier::Stagger || start.mTier == ReactionTier::Down;
        StaggerSettings own = settings;
        own.mChain = settings.mChain && (heavy || (mActive && mSettings.mChain));
        if (steps > 0 && own.mChain)
            steps = budgetFor(start, own);
        if (steps <= 0)
            return false;
        // A blow during a chained stagger: a couple more steps, not a new budget.
        if (mActive && mSettings.mChain && own.mChain)
            steps = std::min(steps, mStepsLeft + std::max(0, own.mExtraSteps));
        mSettings = own;
        const float l = std::hypot(start.mDirection.GetX(), start.mDirection.GetY());
        const JPH::Vec3 direction = l > 1e-4f ? JPH::Vec3(start.mDirection.GetX() / l, start.mDirection.GetY() / l, 0.0f)
                                              : JPH::Vec3::sAxisY();
        const float length = std::clamp(start.mDv * settings.mStepPerDv, settings.mMinimumStep, settings.mMaximumStep);
        if (mActive)
        {
            // A blow during a stagger: more steps, the new way.
            mStepsLeft = std::max(mStepsLeft, steps);
            // The time limit runs from now: only the steps still to take
            // count (the taken ones gave 0.45 s each to a single last step).
            mPlannedSteps = mStepsLeft;
            mStepLength = std::max(mStepLength, length);
            mDirection = (mDirection + direction).NormalizedOr(direction);
            mAge = 0.0f;
            return true;
        }
        mActive = true;
        mStepsLeft = steps;
        mPlannedSteps = steps;
        mStepsTaken = 0;
        mGaveUp = nullptr;
        mMaximumLean = 0.0f;
        mStepLength = length;
        mDirection = direction;
        mAge = 0.0f;
        mSinceLanding = 1.0f;
        mWasSwinging = false;
        mSideways = false;
        mLastFoot.reset();
        const float f = std::hypot(start.mFacing.GetX(), start.mFacing.GetY());
        mFacing = f > 1e-4f ? JPH::Vec3(start.mFacing.GetX() / f, start.mFacing.GetY() / f, 0.0f) : JPH::Vec3::sZero();
        return true;
    }

    void StaggerPlan::reset()
    {
        mActive = false;
        mStepsLeft = 0;
        mStepsTaken = 0;
        mPosture = 0.0f;
        mCaptureOutside = false;
        mGaveUp = nullptr;
        mLastFoot.reset();
    }

    std::optional<StaggerStep> StaggerPlan::step(float dt, const std::array<StaggerFoot, 2>& feet,
        JPH::RVec3Arg centerOfMass, JPH::RVec3Arg capture, float speed)
    {
        // The posture eases in fast and out over 0.4 s.
        const float wanted = mActive ? 1.0f : 0.0f;
        mPosture += std::clamp(wanted - mPosture, -dt / 0.4f, dt / 0.08f);
        if (!mActive)
            return std::nullopt;
        mAge += dt;
        const bool swinging = feet[0].mSwinging || feet[1].mSwinging;
        if (mWasSwinging && !swinging)
            mSinceLanding = 0.0f;
        mSinceLanding += dt;
        mWasSwinging = swinging;
        // Done: the steps are spent and the last has landed, the body has
        // stopped, or it ran too long - 0.5 s + 0.45 s a step, at most the
        // maximum (a one-step plan whose step never landed held the walk
        // request for the whole 3 s while the capsule walked on and dragged
        // the body: push walk front 2000 settled in 3.05 s).
        const float limit = std::min(mSettings.mMaximumTime, 0.5f + 0.45f * static_cast<float>(mPlannedSteps));
        const JPH::Vec3 com = flat(centerOfMass), cap = flat(capture);
        const JPH::Vec3 left = flat(feet[0].mPosition), right = flat(feet[1].mPosition);
        const float captureOff = segmentDistance(cap, left, right);
        mCaptureOutside = captureOff > mSettings.mCaptureMargin;
        mCaptureDistance = captureOff;
        const float height = static_cast<float>(centerOfMass.GetZ())
            - 0.5f * static_cast<float>(feet[0].mPosition.GetZ() + feet[1].mPosition.GetZ());
        const float lean = height > 1.0f && !swinging ? segmentDistance(com, left, right) / height : 0.0f;
        mMaximumLean = std::max(mMaximumLean, lean);
        // S4, the give-up: the steps ran out with the weight still going, or
        // the body leans too far outside its planted feet to be caught.
        if (mSettings.mChain && mSettings.mGiveUp && !swinging && !mHoldGiveUp)
        {
            const char* cause = nullptr;
            if (mStepsLeft <= 0 && mStepsTaken > 0 && mSinceLanding > 0.12f && captureOff > mSettings.mGiveUpMargin
                && lean > mSettings.mSpentLean)
                cause = "spent";
            else if (lean > mSettings.mAbortRatio)
                cause = "lean";
            if (cause)
            {
                mGaveUp = cause;
                mActive = false;
                mStepsLeft = 0;
                return std::nullopt;
            }
        }
        // Chained: caught once a step has landed and the capture point is
        // back between the feet.
        const bool caught = mSettings.mChain && mStepsTaken > 0 && !mCaptureOutside;
        if ((mStepsLeft <= 0 && !swinging && mSinceLanding > 0.12f) || mAge > limit
            || (mStepsTaken > 0 && !swinging && speed < 10.0f && mSinceLanding > 0.15f)
            || (caught && !swinging && mSinceLanding > 0.12f))
        {
            mActive = false;
            mStepsLeft = 0;
            return std::nullopt;
        }
        if (mStepsLeft <= 0 || swinging || mSinceLanding < 0.03f || caught)
            return std::nullopt;
        JPH::Vec3 toCapture = cap - com;
        // Chained steps go where the weight goes now, not where the blow
        // first pushed it.
        const JPH::Vec3 along = mSettings.mChain && toCapture.Length() > 5.0f ? toCapture.Normalized() : mDirection;
        // Which foot: the first step takes the foot behind along the push
        // (it steps past the other), or for a sideways push the foot on the
        // push's side (a side step); after that the feet alternate - for a
        // sideways push the other foot then crosses in front.
        const JPH::Vec3 across(-along.GetY(), along.GetX(), 0.0f);
        std::size_t foot;
        if (mLastFoot)
            foot = 1 - *mLastFoot;
        else
        {
            const JPH::Vec3 between(feet[1].mPosition - feet[0].mPosition);
            const float alongGap = between.Dot(along); // + : right foot is further along the push
            const float sideGap = std::abs(between.Dot(across));
            // A push across the facing is sideways: the leading foot
            // (further along the push) steps out first. Not the feet's
            // layout when the facing is known: a walking body's feet are a
            // stride apart fore-and-aft, and a push from the front read as
            // sideways crossed the legs on the second step.
            mSideways = mFacing.IsNearZero() ? std::abs(alongGap) > sideGap
                                             : std::abs(along.Dot(mFacing)) < 0.7071f;
            if (mSideways)
                foot = alongGap >= 0.0f ? 1 : 0;
            else
                foot = alongGap >= 0.0f ? 0 : 1;
        }
        // Where: short of the capture point (under-recovery), at least a
        // step's length on from the body along the push, kept a little to
        // the foot's own side (a crossing foot lands just past the line).
        float reach = std::max(toCapture.Dot(along) * mSettings.mUnderRecover, mStepLength * 0.6f);
        reach = std::min(reach, mStepLength * 1.4f);
        if (mSettings.mChain)
            reach = std::clamp(toCapture.Length() * mSettings.mChainRecover, mSettings.mMinimumStep, mSettings.mMaximumReach);
        const JPH::Vec3 own(JPH::Vec3(feet[foot].mPosition - centerOfMass));
        const float side = own.Dot(across) >= 0.0f ? 1.0f : -1.0f;
        // Sideways, every second step crosses in front of the leading foot.
        const bool crossing = mSideways && mLastFoot.has_value() && mStepsTaken % 2 == 1;
        const JPH::Vec3 landing = com + along * reach + across * (side * (crossing ? -2.0f : 7.0f));
        StaggerStep out;
        out.mFoot = foot;
        out.mLanding = JPH::RVec3(landing.GetX(), landing.GetY(), centerOfMass.GetZ());
        out.mSwingTime = mSettings.mSwingTime;
        mLastFoot = foot;
        --mStepsLeft;
        ++mStepsTaken;
        mSinceLanding = 0.0f;
        mWasSwinging = true;
        return out;
    }
}
