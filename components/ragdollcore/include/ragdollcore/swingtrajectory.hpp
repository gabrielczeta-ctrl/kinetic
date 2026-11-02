#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_SWINGTRAJECTORY_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_SWINGTRAJECTORY_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Quat.h>
// clang-format on

#include <optional>
#include <array>
#include <cstdint>
#include <span>

namespace RagdollCore
{
    struct SwingTrajectorySample
    {
        JPH::Vec3 mPosition = JPH::Vec3::sZero();
        JPH::Vec3 mVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mAcceleration = JPH::Vec3::sZero();
    };

    // All vectors share one fixed coordinate frame. A moving model frame
    // must not be mixed with history recorded in a preceding model frame.
    // These two boundaries and duration are the complete reconstruction
    // state; callers must capture them and their segment clock if integrated.
    struct SwingTrajectorySegment
    {
        SwingTrajectorySample mStart;
        SwingTrajectorySample mEnd;
        float mDuration = 0.0f;
    };

    // Quintic interpolation of position, velocity and acceleration. This is
    // a kinematic command, not a collision, reach or force-feasibility test.
    // Queries outside [0, duration] are rejected rather than silently holding
    // an endpoint with a potentially nonzero terminal velocity.
    std::optional<SwingTrajectorySample> sampleSwingTrajectory(
        const SwingTrajectorySegment& segment, float elapsed);

    // Start a new segment at exactly the old segment's sampled command.
    // Preserves C2 continuity at the splice, even when the new goal changes.
    // A short deadline may demand excessive acceleration/jerk: callers must
    // qualify that separately, not interpret continuity as physical safety.
    std::optional<SwingTrajectorySegment> retargetSwingTrajectory(
        const SwingTrajectorySegment& previous, float elapsed,
        const SwingTrajectorySample& goal, float remainingDuration);

    struct SwingTrajectoryHistory
    {
        std::uint32_t mSwingFoot = 0;
        float mElapsed = 0.0f;
        SwingTrajectorySegment mSegment;
    };

    // nullopt means no active swing. A foot change starts from initial;
    // otherwise dt advances the old command before splicing. Reject an
    // expired segment instead of silently extrapolating a stale swing.
    // On rejection history is unchanged. A null swingFoot clears history,
    // covering support, recovery and disable transitions explicitly.
    bool updateSwingTrajectoryHistory(std::optional<SwingTrajectoryHistory>& history,
        std::optional<std::uint32_t> swingFoot, const SwingTrajectorySample& initial,
        const SwingTrajectorySample& goal, float remainingDuration, float dt);

    // Change coordinate representation: p'=rotation*p+translation,
    // v'=rotation*v, a'=rotation*a. These are physical vector components,
    // not derivatives measured relative to a moving coordinate origin.
    bool rebaseSwingTrajectoryHistory(std::optional<SwingTrajectoryHistory>& history,
        JPH::QuatArg rotation, JPH::Vec3Arg translation);

    // Version-1 payload: active, foot, elapsed, duration, start p/v/a, end
    // p/v/a. Float channels preserve the existing replay transport's scalar
    // representation; the enclosing capture must identify this layout.
    inline constexpr std::size_t SwingTrajectoryHistoryChannels = 22;
    std::optional<std::array<float, SwingTrajectoryHistoryChannels>> encodeSwingTrajectoryHistory(
        const std::optional<SwingTrajectoryHistory>& history);
    bool decodeSwingTrajectoryHistory(std::span<const float> channels,
        std::optional<SwingTrajectoryHistory>& history);
}

#endif
