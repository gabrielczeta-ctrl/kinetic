#include <ragdollcore/swingtrajectory.hpp>

#include <array>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        bool finite(JPH::Vec3Arg value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        }

        bool finite(const SwingTrajectorySample& value)
        {
            return finite(value.mPosition) && finite(value.mVelocity) && finite(value.mAcceleration);
        }
    }

    std::optional<SwingTrajectorySample> sampleSwingTrajectory(const SwingTrajectorySegment& segment, float elapsed)
    {
        if (!finite(segment.mStart) || !finite(segment.mEnd) || !std::isfinite(segment.mDuration)
            || segment.mDuration <= 0.0f || !std::isfinite(elapsed) || elapsed < 0.0f || elapsed > segment.mDuration)
            return std::nullopt;
        if (elapsed == 0.0f)
            return segment.mStart;
        if (elapsed == segment.mDuration)
            return segment.mEnd;

        // Normalized time and double intermediates avoid powers of a short
        // duration in polynomial coefficients. Translation is added last.
        const double duration = segment.mDuration;
        const double u = elapsed / duration;
        std::array<float, 3> position, velocity, acceleration;
        for (unsigned axis = 0; axis < 3; ++axis)
        {
            const double v0 = segment.mStart.mVelocity[axis] * duration;
            const double v1 = segment.mEnd.mVelocity[axis] * duration;
            const double a0 = segment.mStart.mAcceleration[axis] * duration * duration;
            const double a1 = segment.mEnd.mAcceleration[axis] * duration * duration;
            const double displacement = double(segment.mEnd.mPosition[axis]) - segment.mStart.mPosition[axis];
            const double c1 = v0, c2 = 0.5 * a0;
            const double c3 = 10 * displacement - 6 * v0 - 4 * v1 - 1.5 * a0 + 0.5 * a1;
            const double c4 = -15 * displacement + 8 * v0 + 7 * v1 + 1.5 * a0 - a1;
            const double c5 = 6 * displacement - 3 * v0 - 3 * v1 - 0.5 * a0 + 0.5 * a1;
            position[axis] = static_cast<float>(segment.mStart.mPosition[axis]
                + u * (c1 + u * (c2 + u * (c3 + u * (c4 + u * c5)))));
            velocity[axis] = static_cast<float>((c1 + u * (2 * c2 + u * (3 * c3 + u * (4 * c4 + u * 5 * c5)))) / duration);
            acceleration[axis] = static_cast<float>((2 * c2 + u * (6 * c3 + u * (12 * c4 + u * 20 * c5))) / (duration * duration));
        }
        SwingTrajectorySample result{
            JPH::Vec3(position[0], position[1], position[2]),
            JPH::Vec3(velocity[0], velocity[1], velocity[2]),
            JPH::Vec3(acceleration[0], acceleration[1], acceleration[2]) };
        return finite(result) ? std::optional<SwingTrajectorySample>(result) : std::nullopt;
    }

    std::optional<SwingTrajectorySegment> retargetSwingTrajectory(const SwingTrajectorySegment& previous,
        float elapsed, const SwingTrajectorySample& goal, float remainingDuration)
    {
        if (!finite(goal) || !std::isfinite(remainingDuration) || remainingDuration <= 0.0f)
            return std::nullopt;
        const auto start = sampleSwingTrajectory(previous, elapsed);
        if (!start)
            return std::nullopt;
        return SwingTrajectorySegment{ *start, goal, remainingDuration };
    }

    bool updateSwingTrajectoryHistory(std::optional<SwingTrajectoryHistory>& history,
        std::optional<std::uint32_t> swingFoot, const SwingTrajectorySample& initial,
        const SwingTrajectorySample& goal, float remainingDuration, float dt)
    {
        if (!swingFoot)
        {
            history.reset();
            return true;
        }
        if (*swingFoot > 16'777'215 || !std::isfinite(dt) || dt < 0.0f)
            return false;
        std::optional<SwingTrajectorySegment> segment;
        if (history && history->mSwingFoot == *swingFoot)
            segment = retargetSwingTrajectory(history->mSegment, history->mElapsed + dt, goal, remainingDuration);
        else
        {
            SwingTrajectorySegment proposed{ initial, goal, remainingDuration };
            if (sampleSwingTrajectory(proposed, 0))
                segment = proposed;
        }
        if (!segment)
            return false;
        history = SwingTrajectoryHistory{ *swingFoot, 0.0f, *segment };
        return true;
    }

    bool rebaseSwingTrajectoryHistory(std::optional<SwingTrajectoryHistory>& history,
        JPH::QuatArg rotation, JPH::Vec3Arg translation)
    {
        if (!history)
            return true;
        if (!rotation.IsNormalized() || !finite(translation)
            || !sampleSwingTrajectory(history->mSegment, history->mElapsed))
            return false;
        auto next = *history;
        for (auto* state : { &next.mSegment.mStart, &next.mSegment.mEnd })
        {
            state->mPosition = rotation * state->mPosition + translation;
            state->mVelocity = rotation * state->mVelocity;
            state->mAcceleration = rotation * state->mAcceleration;
            if (!finite(*state))
                return false;
        }
        history = next;
        return true;
    }

    std::optional<std::array<float, SwingTrajectoryHistoryChannels>> encodeSwingTrajectoryHistory(
        const std::optional<SwingTrajectoryHistory>& history)
    {
        std::array<float, SwingTrajectoryHistoryChannels> result{};
        if (!history)
            return result;
        if (history->mSwingFoot > 16'777'215 || !sampleSwingTrajectory(history->mSegment, history->mElapsed))
            return std::nullopt;
        result[0] = 1.0f;
        result[1] = static_cast<float>(history->mSwingFoot);
        result[2] = history->mElapsed;
        result[3] = history->mSegment.mDuration;
        std::size_t index = 4;
        for (const auto* state : { &history->mSegment.mStart, &history->mSegment.mEnd })
            for (const auto value : { state->mPosition, state->mVelocity, state->mAcceleration })
                for (unsigned axis = 0; axis < 3; ++axis)
                    result[index++] = value[axis];
        return result;
    }

    bool decodeSwingTrajectoryHistory(std::span<const float> channels, std::optional<SwingTrajectoryHistory>& history)
    {
        if (channels.size() != SwingTrajectoryHistoryChannels)
            return false;
        for (const float value : channels)
            if (!std::isfinite(value))
                return false;
        if (channels[0] == 0.0f)
        {
            for (const float value : channels)
                if (value != 0.0f)
                    return false;
            history.reset();
            return true;
        }
        if (channels[0] != 1.0f || channels[1] < 0.0f || channels[1] > 16'777'215.0f
            || std::floor(channels[1]) != channels[1])
            return false;
        SwingTrajectoryHistory next;
        next.mSwingFoot = static_cast<std::uint32_t>(channels[1]);
        next.mElapsed = channels[2];
        next.mSegment.mDuration = channels[3];
        std::size_t index = 4;
        for (auto* state : { &next.mSegment.mStart, &next.mSegment.mEnd })
            for (auto* value : { &state->mPosition, &state->mVelocity, &state->mAcceleration })
            {
                *value = JPH::Vec3(channels[index], channels[index + 1], channels[index + 2]);
                index += 3;
            }
        if (!sampleSwingTrajectory(next.mSegment, next.mElapsed))
            return false;
        history = next;
        return true;
    }
}
