#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_BALANCEHELPER_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_BALANCEHELPER_HPP

#include <array>
#include <cstddef>
#include <optional>

namespace RagdollCore
{
    // Balance-and-recovery helper beside Footfall (design:
    // design-pgc-balance-helper-2026-09-28.md in the dev root). Pure
    // horizontal-plane maths on the linear inverted pendulum; no physics
    // world, no Jolt types, so Footfall and the offline harness call the
    // same functions. Units are the caller's (game units and seconds);
    // x/y is the ground plane.
    struct BalanceVec2
    {
        float mX = 0.0f;
        float mY = 0.0f;
    };

    struct BalanceState
    {
        BalanceVec2 mCenterOfMass;
        BalanceVec2 mVelocity;
        // Center of mass height above the support surface.
        float mHeight = 0.0f;
        float mGravity = 0.0f;
    };

    // sqrt(height / gravity), the pendulum's time constant; nullopt when the
    // state is not a standing pendulum.
    std::optional<float> balanceTimeConstant(const BalanceState& state);

    // Instantaneous capture point: com + velocity * timeConstant. The body
    // comes to rest over this point if the center of pressure is put there.
    std::optional<BalanceVec2> capturePoint(const BalanceState& state);

    // Capture point after `time` seconds with the center of pressure held at
    // `pivot`: pivot + (capture - pivot) * exp(time / timeConstant).
    std::optional<BalanceVec2> predictCapturePoint(const BalanceState& state, BalanceVec2 pivot, float time);

    struct CaptureStepLimits
    {
        // Radius of the region around a foot centre the center of pressure
        // can reach (the ankle strategy). A capture point within it needs no
        // further step.
        float mFootRadius = 0.0f;
        // Horizontal distance from the stance foot a landing may take.
        float mMaximumReach = 0.0f;
        // A landing stays at least this far on its own side of the stance
        // foot, measured along the body's right axis (no crossed legs).
        float mMinimumWidth = 0.0f;
        // Swing time model: time = clamp(minimum + distance / footSpeed,
        // minimum, maximum). Longer steps take longer; urgency comes from
        // the shortest swing.
        float mMinimumSwingTime = 0.0f;
        float mMaximumSwingTime = 0.0f;
        float mSwingFootSpeed = 0.0f;
        // Crossover steps (0 = off): when the capture point runs past the
        // stance foot on its own side, the swing foot may cross in front of
        // it (behind it when the fall is backward) to land there. The swing
        // passes through a waypoint this far fore (aft) of the stance foot,
        // so the feet and shins clear each other, and its time follows that
        // longer path; the landing stays at least this far from the stance
        // foot and half of it fore (aft).
        float mCrossoverClearance = 0.0f;
        // Braking: a step that can capture lands this far past the predicted
        // capture point along the fall, so the capture point ends up between
        // the feet and the centre of pressure can bring the body to rest
        // (0 = land on it).
        float mBrakeMargin = 0.0f;
    };

    struct CaptureStepInput
    {
        BalanceState mState;
        BalanceVec2 mStanceFoot;
        // Where the swing foot is now; the swing time follows the distance
        // it has to travel.
        BalanceVec2 mSwingFoot;
        // Body forward on the ground plane; right = (forward.y, -forward.x).
        BalanceVec2 mBodyForward{ 0.0f, 1.0f };
        bool mSwingIsLeft = false;
        // The stance ankle may move the center of pressure to the foot edge
        // nearest the capture point during the swing; false pins it at the
        // foot centre (the plain point-foot pendulum).
        bool mPressureAtFootEdge = true;
        // Fixed time to touchdown (for a swing already under way); absent
        // uses the swing time model.
        std::optional<float> mTimeToTouchdown;
        // The step Footfall would take anyway. The helper keeps it while the
        // predicted capture point is within `mPreferredTolerance` of it and
        // moves it only by the excess, so ordinary walking is untouched.
        std::optional<BalanceVec2> mPreferredLanding;
        float mPreferredTolerance = 0.0f;
        CaptureStepLimits mLimits;
    };

    struct CaptureStepPlan
    {
        BalanceVec2 mCapturePoint;
        // Capture point at touchdown (center of pressure on the stance foot
        // during the swing).
        BalanceVec2 mPredictedCapturePoint;
        BalanceVec2 mLanding;
        float mSwingTime = 0.0f;
        // Distance from the predicted capture point to the support after
        // touchdown (the segment from the stance foot to the landing). At
        // most the foot radius: this step captures the body.
        float mResidual = 0.0f;
        bool mCaptures = false;
        bool mReachClamped = false;
        bool mWidthClamped = false;
        // The preferred landing was kept unchanged.
        bool mKeptPreferred = false;
        // The swing foot crosses the stance leg.
        bool mCrossover = false;
    };

    std::optional<CaptureStepPlan> planCaptureStep(const CaptureStepInput& input);

    // 1-3 recovery steps of varying length and timing. Each step takes the
    // shortest landing toward the predicted capture point that the limits
    // allow; the legs alternate; the next step starts from the previous
    // landing with the body state the pendulum predicts. Stops at the first
    // step that captures.
    struct RecoveryStep
    {
        BalanceVec2 mLanding;
        float mSwingTime = 0.0f;
        float mLength = 0.0f;
        bool mIsLeft = false;
        float mResidual = 0.0f;
        bool mCaptures = false;
        bool mCrossover = false;
    };

    struct RecoveryPlan
    {
        std::array<RecoveryStep, 3> mSteps{};
        std::size_t mCount = 0;
        bool mCaptured = false;
    };

    std::optional<RecoveryPlan> planRecoverySteps(const CaptureStepInput& input, std::size_t maximumSteps);

    // The end of a recovery: the capture point inside the support (the
    // supported feet as disks joined by a segment) by the margin, and the
    // body slower than the speed, both held for the time. The caller keeps
    // the time already held (mHeld) from step to step.
    struct RecoveryEndLimits
    {
        float mFootRadius = 0.0f;
        float mMargin = 0.0f;
        float mSpeed = 0.0f;
        float mTime = 0.0f;
    };

    struct RecoveryEndInput
    {
        BalanceVec2 mCapturePoint;
        BalanceVec2 mVelocity;
        BalanceVec2 mLeftFoot;
        BalanceVec2 mRightFoot;
        bool mLeftSupported = true;
        bool mRightSupported = true;
        float mHeld = 0.0f;
        float mDt = 0.0f;
        RecoveryEndLimits mLimits;
    };

    struct RecoveryEnd
    {
        bool mCaught = false;
        // Time the end condition has held, including this step.
        float mHeld = 0.0f;
        // How far inside the support the capture point is (negative: outside).
        float mInside = 0.0f;
    };

    std::optional<RecoveryEnd> evaluateRecoveryEnd(const RecoveryEndInput& input);

    // Staged push response. Order of rungs: the ankle moves the center of
    // pressure inside the feet; the hip bend and arms add angular momentum
    // that pulls the capture point back a bounded distance; beyond that a
    // step is the only answer.
    enum class BalanceStage
    {
        Ankle,
        HipAndArms,
        Step,
    };

    struct BalanceStageLimits
    {
        float mFootRadius = 0.0f;
        // How far outside the support the hip and arms can pull the capture
        // point back (angular momentum budget / (mass * gravity) in length).
        float mHipReach = 0.0f;
        // Hysteresis: a stage is left only after its error falls this far
        // below the entry threshold.
        float mHysteresis = 0.0f;
        // The hip and arms only buy time: when the capture point is still
        // outside the feet after this long in their band, step. Zero: no
        // limit.
        float mMaximumHipTime = 0.0f;
    };

    struct BalanceStageInput
    {
        BalanceVec2 mCapturePoint;
        BalanceVec2 mLeftFoot;
        BalanceVec2 mRightFoot;
        bool mLeftSupported = true;
        bool mRightSupported = true;
        BalanceStage mPrevious = BalanceStage::Ankle;
        // Seconds already spent in mPrevious.
        float mTimeInPrevious = 0.0f;
        BalanceStageLimits mLimits;
    };

    struct BalanceStageDecision
    {
        BalanceStage mStage = BalanceStage::Ankle;
        // Distance of the capture point outside the support (the feet as
        // disks joined by their hull); zero or negative inside.
        float mExcess = 0.0f;
        // 0..1 authority for the hip bend and arms, rising over the hip
        // band so there is no switch at its edge.
        float mHipWeight = 0.0f;
        // Unit direction from the support to the capture point (the way the
        // body is falling); zero inside the support.
        BalanceVec2 mFallDirection;
    };

    std::optional<BalanceStageDecision> classifyBalanceStage(const BalanceStageInput& input);
}

#endif
