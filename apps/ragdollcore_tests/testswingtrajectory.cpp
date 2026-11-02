#include <ragdollcore/swingtrajectory.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace
{
    void near(JPH::Vec3Arg actual, JPH::Vec3Arg expected, float tolerance)
    {
        EXPECT_LE((actual - expected).Length(), tolerance);
    }
}

TEST(SwingTrajectory, ReproducesConstantAccelerationMotion)
{
    using namespace RagdollCore;
    const SwingTrajectorySample start{ JPH::Vec3(2, -3, 7), JPH::Vec3(10, 4, -2), JPH::Vec3(-5, 8, 3) };
    const float duration = 0.4f;
    const SwingTrajectorySample end{
        start.mPosition + start.mVelocity * duration + 0.5f * start.mAcceleration * duration * duration,
        start.mVelocity + start.mAcceleration * duration, start.mAcceleration };
    const SwingTrajectorySegment segment{ start, end, duration };
    for (unsigned i = 0; i <= 20; ++i)
    {
        const float t = duration * i / 20.0f;
        const auto sample = sampleSwingTrajectory(segment, t);
        ASSERT_TRUE(sample);
        near(sample->mPosition, start.mPosition + start.mVelocity * t + 0.5f * start.mAcceleration * t * t, 2e-6f);
        near(sample->mVelocity, start.mVelocity + start.mAcceleration * t, 1e-5f);
        near(sample->mAcceleration, start.mAcceleration, 1e-4f);
    }
}

TEST(SwingTrajectory, RepeatedGoalChangesPreserveSpliceStateAndReachFinalBoundary)
{
    using namespace RagdollCore;
    SwingTrajectorySegment segment{ { JPH::Vec3(0, 0, 8), JPH::Vec3(20, 0, 0), JPH::Vec3::sZero() },
        { JPH::Vec3(30, 4, 0), JPH::Vec3::sZero(), JPH::Vec3::sZero() }, 0.5f };
    for (unsigned i = 0; i < 12; ++i)
    {
        const auto old = sampleSwingTrajectory(segment, 1.0f / 60.0f);
        ASSERT_TRUE(old);
        const SwingTrajectorySample goal{ JPH::Vec3(30, i % 2 ? -12.0f : 12.0f, 0), JPH::Vec3::sZero(), JPH::Vec3::sZero() };
        const auto next = retargetSwingTrajectory(segment, 1.0f / 60.0f, goal, segment.mDuration - 1.0f / 60.0f);
        ASSERT_TRUE(next);
        const auto splice = sampleSwingTrajectory(*next, 0);
        ASSERT_TRUE(splice);
        near(splice->mPosition, old->mPosition, 0);
        near(splice->mVelocity, old->mVelocity, 0);
        near(splice->mAcceleration, old->mAcceleration, 0);
        segment = *next;
    }
    const auto end = sampleSwingTrajectory(segment, segment.mDuration);
    ASSERT_TRUE(end);
    near(end->mPosition, segment.mEnd.mPosition, 0);
    near(end->mVelocity, JPH::Vec3::sZero(), 0);
    near(end->mAcceleration, JPH::Vec3::sZero(), 0);
    // Also approach the endpoint through the polynomial, rather than only
    // testing the exact-endpoint return path.
    const auto almost = sampleSwingTrajectory(segment, segment.mDuration - 1e-5f);
    ASSERT_TRUE(almost);
    near(almost->mPosition, end->mPosition, 1e-4f);
    near(almost->mVelocity, end->mVelocity, 1e-3f);
}

TEST(SwingTrajectory, RejectsInvalidClockAndNonfiniteHistory)
{
    using namespace RagdollCore;
    SwingTrajectorySegment segment{ {}, {}, 0.3f };
    EXPECT_FALSE(sampleSwingTrajectory(segment, -0.01f));
    EXPECT_FALSE(sampleSwingTrajectory(segment, 0.31f));
    EXPECT_FALSE(retargetSwingTrajectory(segment, 0.1f, {}, 0));
    segment.mStart.mVelocity = JPH::Vec3(std::numeric_limits<float>::quiet_NaN(), 0, 0);
    EXPECT_FALSE(sampleSwingTrajectory(segment, 0));
    EXPECT_FALSE(retargetSwingTrajectory(segment, 0.1f, {}, 0.2f));
}

TEST(SwingTrajectory, HistoryHandoverAndDisableDiscardPreviousSwing)
{
    using namespace RagdollCore;
    std::optional<SwingTrajectoryHistory> history;
    const SwingTrajectorySample initial{ JPH::Vec3(1, 2, 3), JPH::Vec3::sZero(), JPH::Vec3::sZero() };
    const SwingTrajectorySample goal{ JPH::Vec3(30, 10, 3), JPH::Vec3::sZero(), JPH::Vec3::sZero() };
    ASSERT_TRUE(updateSwingTrajectoryHistory(history, 16, initial, goal, 0.3f, 1.0f / 60));
    const auto expected = sampleSwingTrajectory(history->mSegment, 1.0f / 60);
    ASSERT_TRUE(expected);
    ASSERT_TRUE(updateSwingTrajectoryHistory(history, 16, {}, goal, 0.28f, 1.0f / 60));
    near(history->mSegment.mStart.mPosition, expected->mPosition, 0);
    near(history->mSegment.mStart.mVelocity, expected->mVelocity, 0);
    near(history->mSegment.mStart.mAcceleration, expected->mAcceleration, 0);
    // Handover must seed the new foot, not inherit the moving old foot.
    ASSERT_TRUE(updateSwingTrajectoryHistory(history, 19, initial, goal, 0.3f, 1.0f / 60));
    EXPECT_EQ(history->mSwingFoot, 19u);
    near(history->mSegment.mStart.mPosition, initial.mPosition, 0);
    near(history->mSegment.mStart.mVelocity, initial.mVelocity, 0);
    ASSERT_TRUE(updateSwingTrajectoryHistory(history, std::nullopt, {}, {}, 0, 0));
    EXPECT_FALSE(history);
    ASSERT_TRUE(updateSwingTrajectoryHistory(history, 19, initial, goal, 0.3f, 1.0f / 60));
    near(history->mSegment.mStart.mPosition, initial.mPosition, 0);
}

TEST(SwingTrajectory, CaptureRestorePreservesFutureCommandsAndRejectsCorruptionAtomically)
{
    using namespace RagdollCore;
    std::optional<SwingTrajectoryHistory> history;
    const SwingTrajectorySample initial{ JPH::Vec3(1, 2, 8), JPH::Vec3(2, 5, -1), JPH::Vec3(3, -2, 4) };
    const SwingTrajectorySample goal{ JPH::Vec3(30, -5, 3), JPH::Vec3::sZero(), JPH::Vec3::sZero() };
    ASSERT_TRUE(updateSwingTrajectoryHistory(history, 16, initial, goal, 0.3f, 0));
    history->mElapsed = 0.071f;
    const auto encoded = encodeSwingTrajectoryHistory(history);
    ASSERT_TRUE(encoded);
    std::optional<SwingTrajectoryHistory> restored;
    ASSERT_TRUE(decodeSwingTrajectoryHistory(*encoded, restored));
    ASSERT_TRUE(restored);
    EXPECT_EQ(encodeSwingTrajectoryHistory(restored), encoded);
    for (unsigned i = 0; i < 10; ++i)
    {
        const auto a = sampleSwingTrajectory(history->mSegment, history->mElapsed + i * 0.01f);
        const auto b = sampleSwingTrajectory(restored->mSegment, restored->mElapsed + i * 0.01f);
        ASSERT_TRUE(a);
        ASSERT_TRUE(b);
        near(a->mPosition, b->mPosition, 0);
        near(a->mVelocity, b->mVelocity, 0);
        near(a->mAcceleration, b->mAcceleration, 0);
    }
    auto invalid = *encoded;
    invalid[1] = 16.5f;
    EXPECT_FALSE(decodeSwingTrajectoryHistory(invalid, restored));
    EXPECT_EQ(encodeSwingTrajectoryHistory(restored), encoded);
    invalid = *encoded;
    invalid[2] = 2.0f; // clock past deadline
    EXPECT_FALSE(decodeSwingTrajectoryHistory(invalid, restored));
    EXPECT_EQ(encodeSwingTrajectoryHistory(restored), encoded);
    EXPECT_FALSE(updateSwingTrajectoryHistory(restored, 16, initial, goal, 0.2f, 2.0f));
    EXPECT_EQ(encodeSwingTrajectoryHistory(restored), encoded);
    const auto cleared = encodeSwingTrajectoryHistory(std::nullopt);
    ASSERT_TRUE(cleared);
    ASSERT_TRUE(decodeSwingTrajectoryHistory(*cleared, restored));
    EXPECT_FALSE(restored);
}

TEST(SwingTrajectory, CoordinateRebasePreservesPhysicalCommandAndDerivatives)
{
    using namespace RagdollCore;
    std::optional<SwingTrajectoryHistory> history;
    const SwingTrajectorySample initial{ JPH::Vec3(1, 2, 8), JPH::Vec3(2, 5, -1), JPH::Vec3(3, -2, 4) };
    const SwingTrajectorySample goal{ JPH::Vec3(30, -5, 3), JPH::Vec3::sZero(), JPH::Vec3::sZero() };
    ASSERT_TRUE(updateSwingTrajectoryHistory(history, 16, initial, goal, 0.3f, 0));
    const auto before = *history;
    const auto rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.2f);
    const JPH::Vec3 translation(-17, 8, 4);
    ASSERT_TRUE(rebaseSwingTrajectoryHistory(history, rotation, translation));
    for (unsigned i = 0; i <= 10; ++i)
    {
        const float t = before.mSegment.mDuration * i / 10.0f;
        const auto a = sampleSwingTrajectory(before.mSegment, t);
        const auto b = sampleSwingTrajectory(history->mSegment, t);
        ASSERT_TRUE(a);
        ASSERT_TRUE(b);
        near(b->mPosition, rotation * a->mPosition + translation, 1e-5f);
        near(b->mVelocity, rotation * a->mVelocity, 5e-5f);
        near(b->mAcceleration, rotation * a->mAcceleration, 1e-3f);
    }
}
