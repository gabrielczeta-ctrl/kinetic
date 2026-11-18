#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_STEPPLANNER_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_STEPPLANNER_HPP

#include <ragdollcore/balancehelper.hpp>

#include <array>
#include <cstddef>
#include <optional>

namespace RagdollCore
{
    // The step planner of MOVE-01 (docs/ragdoll/designs/MOVE-01-movement-and-
    // combat-controller.md, section 4.2): one planner decides where and when
    // the swing foot lands, every tick, from the divergent component of
    // motion (the DCM, the capture point). Standing, walking, starting,
    // stopping and catches are the same equation with a different desired
    // velocity.
    //
    // Step timing adaptation (Khadiv et al. 2016): on the stance foot p the
    // DCM moves as xi(t) = p + (xi0 - p) e^(omega t). With tau = e^(omega T)
    // the end-of-step DCM is linear in tau, so landing u and tau are chosen
    // together to bring the offset xi(T) - u to the steady gait's offset b,
    // staying near the gait's own step. Too far for one step: the N-step
    // catch (planRecoverySteps); not catchable in N: give up (the body falls).
    //
    // The DCM may come from the angular momentum about the stance foot
    // (ALIP, Gong & Grizzle 2022), which predicts better with heavy legs.
    //
    // Pure ground-plane maths like balancehelper, in the caller's units.
    // MOVE-01 S2: offline, not wired into the game.

    struct StrideState
    {
        BalanceVec2 mCenterOfMass;
        BalanceVec2 mVelocity;
        // The whole body's horizontal angular momentum about the stance
        // foot's contact (x and y components, mass * length^2 / time).
        // Present: the DCM reads it (ALIP); absent: the velocity.
        std::optional<BalanceVec2> mAngularMomentum;
        float mMass = 0.0f;
        // Centre of mass over the stance foot's ground.
        float mHeight = 0.0f;
        float mGravity = 0.0f;
    };

    struct StrideLimits
    {
        // The style's steady gait at the desired speed: its step time (one
        // foot's swing plus its share of double support) and the sideways
        // distance between the feet.
        float mNominalStepTime = 0.35f;
        float mStanceWidth = 12.0f;
        // A step's time from lift to touchdown may adapt within these.
        float mMinimumSwingTime = 0.2f;
        float mMaximumSwingTime = 0.6f;
        // A landing within this of the stance foot.
        float mMaximumReach = 45.0f;
        // A landing at least this far on the swing foot's own side of the
        // stance foot, along the lateral axis: feet never cross.
        float mMinimumWidth = 6.0f;
        float mFootRadius = 5.0f;
        // The adaptation's weights: the landing against the gait's (per
        // unit^2), tau against the gait's, the end-of-step DCM offset
        // against the gait's (per unit^2).
        float mLandingWeight = 1.0f;
        float mTimeWeight = 400.0f;
        float mOffsetWeight = 20.0f;
        // Along the travel, a body slower than its gait (a start, slowing)
        // weighs the offset by this share: it takes a short step forward
        // rather than one back to catch up with the gait's offset.
        float mLagWeight = 0.02f;
        // One step leaves the offset off the gait's by more than this (plus
        // a quarter of the gait's own offset): the N-step catch plans it.
        float mOffsetTolerance = 8.0f;
        // The N-step catch (balancehelper): its limits and how many steps.
        CaptureStepLimits mCatch;
        std::size_t mCatchSteps = 3;
    };

    struct StrideInput
    {
        StrideState mState;
        BalanceVec2 mStanceFoot;
        BalanceVec2 mSwingFoot;
        bool mSwingIsLeft = false;
        // Body forward on the ground plane; the lateral axis is its right,
        // (forward.y, -forward.x). In a guard it is the threat axis.
        BalanceVec2 mForward{ 0.0f, 1.0f };
        BalanceVec2 mDesiredVelocity;
        // Seconds this swing has already flown (0: a step about to lift).
        float mElapsed = 0.0f;
        StrideLimits mLimits;
    };

    struct StridePlan
    {
        BalanceVec2 mDcm;
        // The DCM at touchdown (centre of pressure on the stance foot).
        BalanceVec2 mPredictedDcm;
        BalanceVec2 mLanding;
        BalanceVec2 mNominalLanding;
        // Predicted DCM minus landing, and the steady gait's.
        BalanceVec2 mOffset;
        BalanceVec2 mNominalOffset;
        // Seconds from now to touchdown, and the whole swing's.
        float mRemainingTime = 0.0f;
        float mSwingTime = 0.0f;
        bool mTimeClamped = false;
        bool mWidthActive = false;
        bool mReachActive = false;
        // The N-step catch planned this step (one step could not hold the
        // gait's offset), and how many steps it plans.
        bool mRecovery = false;
        std::size_t mRecoverySteps = 0;
        // The catch found no capture within its steps: the body falls.
        bool mGiveUp = false;
    };

    // omega = sqrt(gravity / height); nullopt for an invalid state.
    std::optional<float> strideOmega(const StrideState& state);
    // The DCM: com + v / omega, with v from the angular momentum about
    // `stanceFoot` when the state has it (v = (Ly, -Lx) / (m h)).
    std::optional<BalanceVec2> strideDcm(const StrideState& state, BalanceVec2 stanceFoot);
    std::optional<StridePlan> planStride(const StrideInput& input);

    // Both feet down and no travel asked: a step is needed only when the DCM
    // leaves the support (the feet as disks joined by a segment) by more than
    // `margin`.
    struct StrideStandInput
    {
        StrideState mState;
        BalanceVec2 mLeftFoot;
        BalanceVec2 mRightFoot;
        float mFootRadius = 5.0f;
        float mMargin = 0.0f;
    };
    std::optional<bool> strideNeedsStep(const StrideStandInput& input);

    // The whole body's angular momentum about a ground contact point (the
    // stance foot) from its momentum about its own centre of mass, as
    // PassiveRagdoll::captureMomentum gives it: L_p = L_com + (com - p) x P.
    // The horizontal (x, y) components, StrideState::mAngularMomentum's.
    BalanceVec2 strideAngularMomentumAbout(const std::array<float, 3>& centerOfMass,
        const std::array<float, 3>& linearMomentum, const std::array<float, 3>& angularMomentumAboutCom,
        const std::array<float, 3>& contact);

    // Which foot lifts first from both feet down (MOVE-01 4.7). Walking, the
    // foot further behind along the travel (a step out of the stance, as
    // Footfall starts); in a guard, the foot nearer the direction of travel
    // (fencing and boxing footwork: the lead foot advances, the rear foot
    // retreats first, the near foot sidesteps), so the base opens and is
    // restored and never crosses. True: the left foot.
    bool strideFirstFootIsLeft(BalanceVec2 leftFoot, BalanceVec2 rightFoot, BalanceVec2 travel, bool guard);

    // The planner's nominal step from a gait table sample: a foot's swing
    // time and duty give the cycle (swing / (1 - duty)); one step is half of
    // it. `halfWidth` is a foot's distance from the middle (Footfall's
    // moving width). The rest of `limits` is kept.
    StrideLimits strideLimitsFromGait(float swingTime, float duty, float halfWidth, StrideLimits limits);
}

#endif
