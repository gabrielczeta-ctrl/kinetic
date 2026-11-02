#include <ragdollcore/posecomposer.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
    using namespace RagdollCore;

    constexpr float sDt = 1.0f / 60.0f;

    // Three bodies: trunk, right arm, left leg.
    std::vector<std::optional<BodyChannel>> channels()
    {
        return { BodyChannel::Trunk, BodyChannel::RightArm, BodyChannel::LeftLeg };
    }

    RagdollPose basePose()
    {
        RagdollPose pose;
        pose.mBodyTransforms.resize(3);
        pose.mBodyTransforms[0].mTranslation = JPH::Vec3(0, 0, 100);
        pose.mBodyTransforms[1].mTranslation = JPH::Vec3(20, 0, 120);
        pose.mBodyTransforms[2].mTranslation = JPH::Vec3(-10, 0, 50);
        return pose;
    }

    LeaseRequest pose(bool additive = false)
    {
        LeaseRequest request;
        request.mPose = true;
        request.mAdditive = additive;
        return request;
    }

    // One writer: move `body` to x = value, for `owner`.
    void write(PoseComposer& composer, RagdollPose& drive, std::size_t body, float x, BodyOwner owner)
    {
        composer.beforeWrite(drive);
        drive.mBodyTransforms[body].mTranslation.SetX(x);
        composer.afterWrite(drive, [&](BodyChannel) { return std::optional<BodyOwner>(owner); }, false);
    }

    TEST(RagdollCorePoseComposer, writersInPriorityOrderComposeToTheSequentialPose)
    {
        BodyLeaseTable leases;
        leases.acquire(BodyChannel::RightArm, BodyOwner::Guard, pose());
        leases.acquire(BodyChannel::RightArm, BodyOwner::HandControl, pose());
        PoseComposer composer;
        RagdollPose drive = basePose();
        composer.begin(drive, channels());
        write(composer, drive, 1, 30.0f, BodyOwner::Guard);
        write(composer, drive, 1, 40.0f, BodyOwner::HandControl);
        const auto result = composer.compose(drive, leases);
        EXPECT_FLOAT_EQ(drive.mBodyTransforms[1].mTranslation.GetX(), 40.0f);
        EXPECT_EQ(result.mInversions, 0u);
        EXPECT_EQ(result.mWrites, 2u);
    }

    TEST(RagdollCorePoseComposer, aLateLowerPriorityWriterNoLongerWins)
    {
        BodyLeaseTable leases;
        leases.acquire(BodyChannel::LeftLeg, BodyOwner::Balance, pose());
        leases.acquire(BodyChannel::LeftLeg, BodyOwner::Locomotion, pose());
        PoseComposer composer;
        RagdollPose drive = basePose();
        composer.begin(drive, channels());
        write(composer, drive, 2, 5.0f, BodyOwner::Balance);
        write(composer, drive, 2, -30.0f, BodyOwner::Locomotion); // runs later, lower priority
        const auto result = composer.compose(drive, leases);
        EXPECT_FLOAT_EQ(drive.mBodyTransforms[2].mTranslation.GetX(), 5.0f);
        EXPECT_EQ(result.mInversions, 1u);
        EXPECT_NEAR(result.mMaxTranslation, 35.0f, 1e-3f);
    }

    TEST(RagdollCorePoseComposer, additiveLayersAddOnTopOfTheAbsoluteResult)
    {
        BodyLeaseTable leases;
        leases.acquire(BodyChannel::Trunk, BodyOwner::HandControl, pose());
        leases.acquire(BodyChannel::Trunk, BodyOwner::Reaction, pose(true));
        PoseComposer composer;
        RagdollPose drive = basePose();
        composer.begin(drive, channels());
        write(composer, drive, 0, 12.0f, BodyOwner::HandControl);
        // The reflex bends what is there: +3 on x, a 0.2 rad turn.
        composer.beforeWrite(drive);
        drive.mBodyTransforms[0].mTranslation += JPH::Vec3(3, 0, 0);
        drive.mBodyTransforms[0].mRotation
            = (JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.2f) * drive.mBodyTransforms[0].mRotation).Normalized();
        composer.afterWrite(drive, [](BodyChannel) { return std::optional<BodyOwner>(BodyOwner::Reaction); }, true);
        const RagdollPose sequential = drive;
        const auto result = composer.compose(drive, leases);
        EXPECT_NEAR(drive.mBodyTransforms[0].mTranslation.GetX(), 15.0f, 1e-4f);
        EXPECT_NEAR(std::abs(drive.mBodyTransforms[0].mRotation.Dot(sequential.mBodyTransforms[0].mRotation)), 1.0f,
            1e-5f);
        EXPECT_EQ(result.mInversions, 0u);
    }

    TEST(RagdollCorePoseComposer, anOffsetUnderALaterTargetIsNotAddedTwice)
    {
        BodyLeaseTable leases;
        leases.acquire(BodyChannel::Trunk, BodyOwner::HandControl, pose());
        PoseComposer composer;
        RagdollPose drive = basePose();
        composer.begin(drive, channels());
        // A counterbalance offset first, then a writer that builds on it.
        composer.beforeWrite(drive);
        drive.mBodyTransforms[0].mTranslation += JPH::Vec3(2, 0, 0);
        composer.afterWrite(drive, [](BodyChannel) { return std::optional<BodyOwner>(BodyOwner::Reaction); }, true);
        composer.beforeWrite(drive);
        drive.mBodyTransforms[0].mTranslation += JPH::Vec3(5, 0, 0);
        composer.afterWrite(drive, [](BodyChannel) { return std::optional<BodyOwner>(BodyOwner::HandControl); }, false);
        const auto result = composer.compose(drive, leases);
        EXPECT_FLOAT_EQ(drive.mBodyTransforms[0].mTranslation.GetX(), 7.0f);
        EXPECT_EQ(result.mInversions, 0u);
    }

    TEST(RagdollCorePoseComposer, unrecordedWritesBelongToTheBase)
    {
        BodyLeaseTable leases;
        PoseComposer composer;
        RagdollPose drive = basePose();
        composer.begin(drive, channels());
        drive.mBodyTransforms[2].mTranslation.SetX(-20.0f); // between writers, no owner
        write(composer, drive, 1, 25.0f, BodyOwner::Rest);
        const auto result = composer.compose(drive, leases);
        EXPECT_FLOAT_EQ(drive.mBodyTransforms[2].mTranslation.GetX(), -20.0f);
        EXPECT_FLOAT_EQ(drive.mBodyTransforms[1].mTranslation.GetX(), 25.0f);
        EXPECT_EQ(result.mInversions, 0u);
    }

    TEST(RagdollCorePoseComposer, aReleasedOwnerFadesOutFromItsLastLayer)
    {
        BodyLeaseTable leases;
        const LeaseId hand = leases.acquire(BodyChannel::RightArm, BodyOwner::HandControl, pose(), { 10.0f, 0.0f, 0.2f });
        PoseComposer composer;
        RagdollPose drive = basePose();
        composer.begin(drive, channels());
        write(composer, drive, 1, 60.0f, BodyOwner::HandControl);
        composer.compose(drive, leases);
        // Hand control lets go; the rest pose takes the arm (lower priority).
        leases.release(hand);
        leases.acquire(BodyChannel::RightArm, BodyOwner::Rest, pose());
        float previous = 60.0f;
        for (int step = 0; step < 15; ++step)
        {
            leases.step(sDt);
            drive = basePose();
            composer.begin(drive, channels());
            write(composer, drive, 1, 20.0f, BodyOwner::Rest);
            const auto result = composer.compose(drive, leases);
            const float x = drive.mBodyTransforms[1].mTranslation.GetX();
            EXPECT_LE(x, previous + 1e-4f);
            EXPECT_GE(x, 20.0f - 1e-4f);
            if (step < 10)
                EXPECT_EQ(result.mFading, 1u);
            previous = x;
        }
        EXPECT_NEAR(previous, 20.0f, 1e-3f);
    }

    TEST(RagdollCorePoseComposer, aFadingInOwnerBlendsOverTheLayersBelow)
    {
        BodyLeaseTable leases;
        leases.acquire(BodyChannel::RightArm, BodyOwner::Rest, pose());
        leases.acquire(BodyChannel::RightArm, BodyOwner::Guard, pose(), { 10.0f, 0.2f, 0.0f });
        leases.step(0.1f); // half way in
        PoseComposer composer;
        RagdollPose drive = basePose();
        composer.begin(drive, channels());
        write(composer, drive, 1, 0.0f, BodyOwner::Rest);
        write(composer, drive, 1, 40.0f, BodyOwner::Guard);
        composer.compose(drive, leases);
        EXPECT_NEAR(drive.mBodyTransforms[1].mTranslation.GetX(), 20.0f, 1.0f);
    }
}
