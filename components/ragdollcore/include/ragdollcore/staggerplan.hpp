#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_STAGGERPLAN_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_STAGGERPLAN_H

// PGC-03 B4-B7: the stagger. A blow big enough (by the reaction tier and the
// body's speed change) starts a planned run of short, rushed steps in the
// push direction that deliberately under-recover - each lands short of where
// the momentum would be caught, so the next one is needed - until the planned
// count is spent or the body has stopped. Lateral pushes step sideways (the
// leading foot out, the other crossing in front of it). A foot dragged along
// (a skid) starts a short plan too. While it runs the body's posture leans
// into the stagger (the torso leads, the arms fly out, the head drops) and
// the actor's walk request is held.
//
// Pure: no Jolt bodies or engine types.

#include <ragdollcore/reactioncontroller.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Vec3.h>

#include <array>
#include <optional>

namespace RagdollCore
{
    struct StaggerSettings
    {
        bool mEnabled = false;
        float mFlinchDv = 25.0f; // a flinch this hard takes one step
        std::array<float, 3> mStaggerDv{ 0.0f, 70.0f, 110.0f }; // a stagger's 2nd/3rd/4th step from these dv (u/s)
        float mHeavyDv = 160.0f; // one more step (a stumble, up to 5)
        float mStepPerDv = 0.22f; // step length per dv (units per u/s)
        float mMinimumStep = 12.0f, mMaximumStep = 38.0f;
        float mSwingTime = 0.26f; // rushed swings
        float mUnderRecover = 0.7f; // each step lands this share of the way to the capture point
        float mLean = 0.3f, mArms = 1.1f, mHead = 0.35f; // posture at full (radians)
        float mMaximumTime = 3.0f;
        float mSkidSpeed = 0.0f; // a planted foot dragged faster than this starts a plan (0 = off)
        // STEP-01 S3, chained steps (off = the counted plan
        // above). After each landing, while the capture point is more than
        // mCaptureMargin outside the feet, another step goes toward it (this
        // share of the way, along the capture point, not the blow), up to
        // mMaximumReach from the body and mStepBudget steps in all; the plan
        // ends when the capture point is back between the feet.
        bool mChain = false;
        float mChainRecover = 0.9f;
        float mMaximumReach = 55.0f;
        float mCaptureMargin = 8.0f;
        int mStepBudget = 6;
        // STEP-01 S4, the step budget and the give-up (chained plans only).
        // The budget is 2 + Agility / 25 (at most mStepBudget), one fewer when
        // tired, one fewer when hurt. The balance is lost - the plan ends
        // given up and the body falls - when the budget is spent and the
        // last step has landed with the capture point still more than
        // mGiveUpMargin outside the feet, or when, both feet down, the centre
        // of mass is outside them by more than mAbortRatio x its height.
        // Play 10-29 (user: "falling too much ... too large distances"): only
        // a stagger or knock-down blow chains and can give up - a flinch keeps
        // its one counted step (flinches chained up to the budget and, with
        // the body walking on, gave up upright); "spent" also needs the weight
        // already this share of its height past the feet (a moving body's
        // capture point lies ahead by its own walking speed); a blow during a
        // chained stagger adds at most mExtraSteps.
        bool mGiveUp = false;
        float mGiveUpMargin = 15.0f;
        float mAbortRatio = 0.6f;
        float mSpentLean = 0.3f;
        int mExtraSteps = 2;
    };

    struct StaggerStart
    {
        ReactionTier mTier = ReactionTier::Stagger;
        float mDv = 0.0f; // the body's horizontal speed change (u/s)
        JPH::Vec3 mDirection = JPH::Vec3::sAxisY(); // horizontal, where the blow pushes
        float mFatigue = 1.0f; // share of fatigue left (0..1)
        float mSkill = 0.5f; // 0..1
        // The body's facing (horizontal). It decides a sideways push; zero =
        // judge by the feet's layout alone.
        JPH::Vec3 mFacing = JPH::Vec3::sZero();
        float mAgility = -1.0f; // the attribute (0..100+); < 0 = the settings' budget
        float mHealth = 1.0f; // share of health left (0..1)
    };

    struct StaggerFoot
    {
        JPH::RVec3 mPosition;
        bool mSwinging = false;
        bool mContact = false;
    };

    struct StaggerStep
    {
        std::size_t mFoot = 0;
        JPH::RVec3 mLanding; // ground height to be filled by the caller
        float mSwingTime = 0.26f;
    };

    class StaggerPlan
    {
    public:
        // Steps a start asks for (0: none).
        static int stepsFor(const StaggerStart& start, const StaggerSettings& settings);
        // A chained plan's step budget.
        static int budgetFor(const StaggerStart& start, const StaggerSettings& settings);
        // Starts (or extends) a plan; false when the blow asks for no steps.
        bool begin(const StaggerStart& start, const StaggerSettings& settings);
        bool active() const { return mActive; }
        int stepsLeft() const { return mStepsLeft; }
        int stepsTaken() const { return mStepsTaken; }
        const JPH::Vec3& direction() const { return mDirection; }
        // 0..1: how much of the stagger posture applies now.
        float posture() const { return mPosture; }
        // One fixed step: the next step to take, if both feet are down and
        // one is due. `centerOfMass`/`capture` horizontal (z ignored), `speed`
        // the body's horizontal speed.
        std::optional<StaggerStep> step(float dt, const std::array<StaggerFoot, 2>& feet, JPH::RVec3Arg centerOfMass,
            JPH::RVec3Arg capture, float speed);
        void reset();
        // Chained plans: the capture point is still outside the feet (it
        // needs a step) after the last landing.
        bool captureOutside() const { return mCaptureOutside; }
        // S4: the last plan ended in a lost balance (nullptr: it did not):
        // "spent" (budget gone, capture outside) or "lean".
        const char* gaveUp() const { return mGaveUp; }
        // No give-up this step: the actor is asking to move (walking, running,
        // an AI chasing). A moving body's weight leads its feet and its
        // capture point lies ahead by its own speed; read as a lost balance,
        // running into a frontal blow fell every time (balance suite 10-29).
        void holdGiveUp(bool hold) { mHoldGiveUp = hold; }
        // The capture point's distance outside the feet at the last step,
        // and the largest lean (centre of mass outside the planted feet over
        // its height) this plan saw.
        float captureDistance() const { return mCaptureDistance; }
        float maximumLean() const { return mMaximumLean; }

    private:
        float mCaptureDistance = 0.0f;
        float mMaximumLean = 0.0f;
        bool mCaptureOutside = false;
        const char* mGaveUp = nullptr;
        bool mHoldGiveUp = false;
        StaggerSettings mSettings;
        bool mActive = false;
        int mStepsLeft = 0;
        int mStepsTaken = 0;
        int mPlannedSteps = 0;
        float mStepLength = 20.0f;
        float mAge = 0.0f;
        float mSinceLanding = 1.0f;
        float mPosture = 0.0f;
        bool mWasSwinging = false;
        bool mSideways = false;
        std::optional<std::size_t> mLastFoot;
        JPH::Vec3 mDirection = JPH::Vec3::sAxisY();
        JPH::Vec3 mFacing = JPH::Vec3::sZero();
    };
}

#endif
