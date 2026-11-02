#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_LIMBCLEARANCE_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_LIMBCLEARANCE_H

#include <Jolt/Jolt.h>
#include <Jolt/Math/DVec3.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <ragdollcore/passiveragdoll.hpp>
#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace RagdollCore
{
    // Shape already includes builder fitting/scaling. Position is BODY origin,
    // not shape COM. Caller owns the shape and initializes Jolt's type registry.
    struct LimbShapePose
    {
        const JPH::Shape* mShape = nullptr;
        JPH::RVec3 mPosition = JPH::RVec3::sZero();
        JPH::Quat mRotation = JPH::Quat::sIdentity();
    };

    struct LimbClearance
    {
        bool mClear = true;
        // Only reported within the query margin; negative means penetration.
        // No value means all tested pairs are outside the query margin.
        std::optional<float> mMinimumReportedGap;
        std::size_t mFirstIndex = 0;
        std::size_t mSecondIndex = 0;
        std::size_t mTestedPairs = 0;
    };

    // Tests EVERY cross-limb shape pair with Jolt's actual collision shapes.
    // This is instantaneous geometry, not a swept path, future stance forecast,
    // ground-contact test or proof of motor tracking. No collision filters are
    // silently applied: callers choose the two distinct limbs to test.
    // Invalid/empty inputs fail closed via nullopt, never a clear result.
    std::optional<LimbClearance> measureLimbClearance(std::span<const LimbShapePose> first,
        std::span<const LimbShapePose> second, float margin);

    struct LimbPoseSearchInput
    {
        std::array<const JPH::Shape*, 3> mShapes{};
        JPH::RVec3 mHipPosition = JPH::RVec3::sZero();
        JPH::Quat mParentRotation = JPH::Quat::sIdentity();
        JPH::Quat mHipRotation = JPH::Quat::sIdentity();
        JPH::Quat mFootRotation = JPH::Quat::sIdentity();
        HingeLegGeometry mLeg;
        SwingTwistJointGeometry mHipLimits;
        SwingTwistJointGeometry mAnkleLimits;
        float mKneeAngleFromReference = 0;
        float mScale = 1;
        // Optional flat floor bound for every supplied shape (not terrain).
        std::optional<double> mMinimumWorldZ;
    };

    struct ClearLimbPose
    {
        std::array<LimbShapePose, 3> mPoses;
        float mKneeAngleFromReference = 0;
        float mFootDisplacement = 0;
        std::size_t mEvaluatedCandidates = 0;
    };

    // All accepted samples of the SAME bounded 321-candidate search, in
    // deterministic enumeration order. May contain equivalent clamped poses.
    // Empty means invalid/exhausted; this is not a complete configuration space.
    std::vector<ClearLimbPose> enumerateClearLimbPoses(const LimbPoseSearchInput& input,
        std::span<const LimbShapePose> support, float margin, float maximumFootDisplacement);

    struct LimbPoseRates
    {
        JPH::Vec3 mHipWorld = JPH::Vec3::sZero();
        float mKnee = 0;
        // Parent (calf) coordinates. The adapter rotates this through the
        // measured calf before submitting a relative-world motor velocity.
        JPH::Vec3 mAnkleParent = JPH::Vec3::sZero();
    };

    // Rates of the SAME shortest-SLERP / linear-hinge interval used by the
    // transition checker. No measured future state or stored recording needed.
    // Bounds/clearance must be checked separately; rates do not certify torque
    // feasibility or continuous acceleration. Invalid inputs fail closed.
    std::optional<LimbPoseRates> calculateLimbPoseRates(
        const ClearLimbPose& from, const ClearLimbPose& to, float interval);

    struct LimbPoseContinuity
    {
        ClearLimbPose mPrevious;
        JPH::Quat mPreviousParentRotation = JPH::Quat::sIdentity();
        std::span<const LimbShapePose> mPreviousSupport;
        float mMaximumFootTravel = 0;
        float mMaximumJointRotation = 0;
        unsigned mIntervals = 8;
    };

    // Checks hip-world SLERP, linear knee and ankle-parent SLERP poses
    // against interpolated support poses.
    // Preserves fixed link lengths; bounds endpoint foot travel and relative
    // joint angle changes. Sampling is NOT a continuous collision certificate
    // or a prediction of future physics. Invalid inputs fail closed.
    bool isLimbTransitionClear(const LimbPoseSearchInput& input, const ClearLimbPose& candidate,
        std::span<const LimbShapePose> support, float margin, const LimbPoseContinuity& continuity);

    // Bounded deterministic local search, not complete motion planning. Shape
    // fitting is already baked in; mScale only scales the unscaled leg offsets.
    // Limits must include the caller's desired safety margins. The displacement
    // bound is relative to the requested foot origin BEFORE joint clamping.
    // No result means invalid input OR no sampled clear pose; never proof that
    // no solution exists. Returned pose is instantaneous, not a safe route.
    std::optional<ClearLimbPose> findClearLimbPose(const LimbPoseSearchInput& input,
        std::span<const LimbShapePose> support, float margin, float maximumFootDisplacement,
        const LimbPoseContinuity* continuity = nullptr);

    // Query-only decomposition of the SAME checks findClearLimbPose applies to
    // its first (exactly requested) candidate. Reports every subcondition
    // independently so a rejection can be attributed without rerunning physics.
    // Deltas are the orientation/angle change the supplied limits would impose;
    // zero means the requested value is inside the limits as supplied.
    struct LimbPoseDiagnosis
    {
        bool mValidInput = false;
        bool mHipClamped = false;
        float mHipClampDelta = 0; // rad, requested vs limit-clamped hip
        bool mKneeClamped = false;
        float mKneeClampDelta = 0; // rad, requested vs hinge-clamped knee
        bool mAnkleClamped = false;
        float mAnkleClampDelta = 0; // rad, requested vs limit-clamped ankle
        // FK foot displacement after clamping vs the pre-clamp request.
        float mExactFootDisplacement = -1;
        bool mFloorClear = false;
        // Minimum over the three shapes of (shape bottom z - mMinimumWorldZ);
        // negative means below the floor bound. NaN when no floor bound.
        float mFloorMargin = std::numeric_limits<float>::quiet_NaN();
        bool mSupportClear = false;
        // Most restrictive reported gap within margin; negative is penetration.
        // NaN when every pair is outside the query margin.
        float mSupportGap = std::numeric_limits<float>::quiet_NaN();
    };

    LimbPoseDiagnosis diagnoseLimbPose(const LimbPoseSearchInput& input,
        std::span<const LimbShapePose> support, float margin);

    enum class LimbTransitionFailure
    {
        None,
        InvalidInput,
        FootTravel,
        KneeChange,
        HipChange,
        AnkleChange,
        SampleKneeRange,
        SampleHipClamp,
        SampleAnkleClamp,
        SampleEndpoint,
        SampleFloor,
        SampleClearance,
    };

    // Query-only decomposition of isLimbTransitionClear: reports the bound
    // checks and the first failing interpolated sample with its reason.
    struct LimbTransitionDiagnosis
    {
        LimbTransitionFailure mFailure = LimbTransitionFailure::InvalidInput;
        float mFootTravel = -1;
        float mKneeChange = 0; // rad
        float mHipChange = 0; // rad, parent-relative
        float mAnkleChange = 0; // rad, calf-relative
        int mFirstFailingSample = -1; // 0..mIntervals, -1 when bounds fail first
        float mSampleFloorMargin = std::numeric_limits<float>::quiet_NaN();
        float mSampleSupportGap = std::numeric_limits<float>::quiet_NaN();
    };

    LimbTransitionDiagnosis diagnoseLimbTransition(const LimbPoseSearchInput& input,
        const ClearLimbPose& candidate, std::span<const LimbShapePose> support, float margin,
        const LimbPoseContinuity& continuity);
}
#endif
