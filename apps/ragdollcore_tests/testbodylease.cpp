#include <ragdollcore/bodylease.hpp>

#include <gtest/gtest.h>

namespace
{
    using namespace RagdollCore;

    constexpr float sDt = 1.0f / 60.0f;

    void run(BodyLeaseTable& table, float seconds)
    {
        for (float t = 0.0f; t < seconds - 1e-6f; t += sDt)
            table.step(sDt);
    }

    LeaseRequest pose(float weight = 1.0f)
    {
        LeaseRequest request;
        request.mPose = true;
        request.mPoseWeight = weight;
        return request;
    }

    LeaseRequest compliance(float torque, float spring = 1.0f, bool exclusive = false)
    {
        LeaseRequest request;
        request.mTorqueScale = torque;
        request.mSpringScale = spring;
        request.mExclusiveCompliance = exclusive;
        return request;
    }

    TEST(RagdollCoreBodyLease, emptyChannelIsTheAnimation)
    {
        const BodyLeaseTable table;
        const auto resolved = table.resolve(BodyChannel::Trunk);
        EXPECT_EQ(resolved.mPoseOwner, BodyOwner::Animation);
        EXPECT_EQ(resolved.mPoseLayerCount, 0);
        EXPECT_FLOAT_EQ(resolved.mTorqueScale, 1.0f);
        EXPECT_FLOAT_EQ(resolved.mSpringScale, 1.0f);
        EXPECT_FALSE(resolved.mDriveOff);
        EXPECT_TRUE(table.ownsPose(BodyChannel::Trunk, BodyOwner::Animation));
    }

    TEST(RagdollCoreBodyLease, higherPriorityPoseOwnsTheChannelAndLayersAboveTheLower)
    {
        BodyLeaseTable table;
        table.acquire(BodyChannel::RightArm, BodyOwner::Reaction, pose(0.5f));
        table.acquire(BodyChannel::RightArm, BodyOwner::HandControl, pose());
        const auto resolved = table.resolve(BodyChannel::RightArm);
        ASSERT_EQ(resolved.mPoseLayerCount, 2);
        EXPECT_EQ(resolved.mPoseLayers[0].mOwner, BodyOwner::HandControl);
        EXPECT_EQ(resolved.mPoseLayers[1].mOwner, BodyOwner::Reaction);
        EXPECT_FLOAT_EQ(resolved.mPoseLayers[1].mWeight, 0.5f);
        // The reaction blends at half weight: hand control still owns the arm.
        EXPECT_EQ(resolved.mPoseOwner, BodyOwner::HandControl);
        EXPECT_TRUE(table.ownsPose(BodyChannel::RightArm, BodyOwner::HandControl));
        EXPECT_FALSE(table.ownsPose(BodyChannel::LeftArm, BodyOwner::HandControl));
    }

    TEST(RagdollCoreBodyLease, explicitPriorityOverridesTheOwnerDefault)
    {
        BodyLeaseTable table;
        table.acquire(BodyChannel::Trunk, BodyOwner::Balance, pose());
        table.acquire(BodyChannel::Trunk, BodyOwner::Guard, pose(), {}, 1000);
        EXPECT_EQ(table.resolve(BodyChannel::Trunk).mPoseOwner, BodyOwner::Guard);
    }

    TEST(RagdollCoreBodyLease, equalPriorityLaterLeaseWins)
    {
        BodyLeaseTable table;
        table.acquire(BodyChannel::Head, BodyOwner::Reaction, pose());
        table.acquire(BodyChannel::Head, BodyOwner::Protect, pose(), {}, defaultLeasePriority(BodyOwner::Reaction));
        EXPECT_EQ(table.resolve(BodyChannel::Head).mPoseOwner, BodyOwner::Protect);
    }

    TEST(RagdollCoreBodyLease, fadesInHoldsThenFadesOutAndExpires)
    {
        BodyLeaseTable table;
        const LeaseId id = table.acquire(BodyChannel::Trunk, BodyOwner::Reaction, compliance(0.0f), { 0.5f, 0.1f, 0.2f });
        EXPECT_FLOAT_EQ(table.strength(id), 0.0f);
        run(table, 0.05f);
        EXPECT_NEAR(table.strength(id), 0.5f, 0.02f);
        EXPECT_NEAR(table.resolve(BodyChannel::Trunk).mTorqueScale, 0.5f, 0.02f);
        run(table, 0.25f);
        EXPECT_FLOAT_EQ(table.strength(id), 1.0f);
        EXPECT_FLOAT_EQ(table.resolve(BodyChannel::Trunk).mTorqueScale, 0.0f);
        run(table, 0.3f); // 0.6 s: 0.1 s into the 0.2 s fade-out
        EXPECT_NEAR(table.strength(id), 0.5f, 0.05f);
        run(table, 0.15f);
        EXPECT_FALSE(table.live(id));
        EXPECT_FLOAT_EQ(table.resolve(BodyChannel::Trunk).mTorqueScale, 1.0f);
    }

    TEST(RagdollCoreBodyLease, releaseStartsTheFadeOutAndRenewHoldsAgain)
    {
        BodyLeaseTable table;
        const LeaseId id = table.acquire(BodyChannel::LeftArm, BodyOwner::Guard, pose(), { 1e9f, 0.0f, 0.2f });
        run(table, 0.5f);
        table.release(id);
        run(table, 0.1f);
        EXPECT_NEAR(table.strength(id), 0.5f, 0.05f);
        EXPECT_TRUE(table.renew(id, 1.0f));
        EXPECT_FLOAT_EQ(table.strength(id), 1.0f);
        run(table, 1.1f);
        EXPECT_NEAR(table.strength(id), 0.5f, 0.1f);
        run(table, 0.2f);
        EXPECT_FALSE(table.live(id));
    }

    TEST(RagdollCoreBodyLease, releaseOwnerReleasesOnlyThatOwner)
    {
        BodyLeaseTable table;
        const LeaseId a = table.acquire(BodyChannel::LeftLeg, BodyOwner::Balance, pose());
        const LeaseId b = table.acquire(BodyChannel::RightLeg, BodyOwner::Balance, pose());
        const LeaseId c = table.acquire(BodyChannel::LeftLeg, BodyOwner::Locomotion, pose());
        table.releaseOwner(BodyOwner::Balance);
        table.step(sDt);
        EXPECT_FALSE(table.live(a));
        EXPECT_FALSE(table.live(b));
        EXPECT_TRUE(table.live(c));
        EXPECT_EQ(table.resolve(BodyChannel::LeftLeg).mPoseOwner, BodyOwner::Locomotion);
    }

    TEST(RagdollCoreBodyLease, complianceMultipliesAndExclusiveReplacesTheLower)
    {
        BodyLeaseTable table;
        table.acquire(BodyChannel::RightArm, BodyOwner::HandControl, compliance(2.0f, 1.5f));
        table.acquire(BodyChannel::RightArm, BodyOwner::Reaction, compliance(0.25f));
        auto resolved = table.resolve(BodyChannel::RightArm);
        EXPECT_FLOAT_EQ(resolved.mTorqueScale, 0.5f);
        EXPECT_FLOAT_EQ(resolved.mSpringScale, 1.5f);
        // A protective fall sets its own compliance whatever is below it.
        table.acquire(BodyChannel::RightArm, BodyOwner::Protect, compliance(0.8f, 0.6f, true));
        resolved = table.resolve(BodyChannel::RightArm);
        EXPECT_FLOAT_EQ(resolved.mTorqueScale, 0.8f);
        EXPECT_FLOAT_EQ(resolved.mSpringScale, 0.6f);
        // A higher non-exclusive lease still multiplies on top of it.
        table.acquire(BodyChannel::RightArm, BodyOwner::System, compliance(0.5f));
        EXPECT_FLOAT_EQ(table.resolve(BodyChannel::RightArm).mTorqueScale, 0.4f);
    }

    TEST(RagdollCoreBodyLease, driveOffUnlessAHigherLeaseDrivesThePose)
    {
        BodyLeaseTable table;
        LeaseRequest slack;
        slack.mDriveOff = true;
        table.acquire(BodyChannel::LeftArm, BodyOwner::HandControl, pose());
        table.acquire(BodyChannel::LeftArm, BodyOwner::Reaction, slack);
        auto resolved = table.resolve(BodyChannel::LeftArm);
        EXPECT_TRUE(resolved.mDriveOff);
        ASSERT_TRUE(resolved.mDriveOffOwner);
        EXPECT_EQ(*resolved.mDriveOffOwner, BodyOwner::Reaction);
        // A protective fall drives the arm: the slack mask no longer applies.
        table.acquire(BodyChannel::LeftArm, BodyOwner::Protect, pose());
        resolved = table.resolve(BodyChannel::LeftArm);
        EXPECT_FALSE(resolved.mDriveOff);
        EXPECT_EQ(resolved.mPoseOwner, BodyOwner::Protect);
    }

    TEST(RagdollCoreBodyLease, additiveLayersStackButNeverOwnOrUnmask)
    {
        BodyLeaseTable table;
        LeaseRequest flinch = pose(0.7f);
        flinch.mAdditive = true;
        LeaseRequest slack;
        slack.mDriveOff = true;
        table.acquire(BodyChannel::Head, BodyOwner::HandControl, pose());
        table.acquire(BodyChannel::Head, BodyOwner::Balance, slack);
        table.acquire(BodyChannel::Head, BodyOwner::Protect, flinch);
        const auto resolved = table.resolve(BodyChannel::Head);
        ASSERT_EQ(resolved.mPoseLayerCount, 2);
        EXPECT_TRUE(resolved.mPoseLayers[1].mAdditive);
        EXPECT_FLOAT_EQ(resolved.mPoseLayers[1].mWeight, 0.7f);
        EXPECT_EQ(resolved.mPoseOwner, BodyOwner::HandControl);
        // An offset on a slack channel does not switch its motors back on.
        EXPECT_TRUE(resolved.mDriveOff);
    }

    TEST(RagdollCoreBodyLease, onlyTheTopFourPoseLayersAreKept)
    {
        BodyLeaseTable table;
        for (BodyOwner owner : { BodyOwner::Rest, BodyOwner::Guard, BodyOwner::HandControl, BodyOwner::Reaction,
                 BodyOwner::Balance })
            table.acquire(BodyChannel::Trunk, owner, pose(0.5f));
        const auto resolved = table.resolve(BodyChannel::Trunk);
        ASSERT_EQ(resolved.mPoseLayerCount, 4);
        EXPECT_EQ(resolved.mPoseLayers[0].mOwner, BodyOwner::Guard);
        EXPECT_EQ(resolved.mPoseLayers[3].mOwner, BodyOwner::Balance);
        // None at full weight: the animation still owns the pose.
        EXPECT_EQ(resolved.mPoseOwner, BodyOwner::Animation);
    }

    TEST(RagdollCoreBodyLease, writesByNonOwnersAreCounted)
    {
        BodyLeaseTable table;
        table.acquire(BodyChannel::Trunk, BodyOwner::HandControl, pose());
        table.noteWrite(BodyChannel::Trunk, BodyOwner::HandControl);
        EXPECT_EQ(table.conflicts(), 0u);
        table.noteWrite(BodyChannel::Trunk, BodyOwner::Reaction);
        table.noteWrite(BodyChannel::Trunk, BodyOwner::Reaction);
        EXPECT_EQ(table.conflicts(), 2u);
        EXPECT_EQ(table.conflicts(BodyChannel::Trunk, BodyOwner::Reaction), 2u);
        EXPECT_EQ(table.conflicts(BodyChannel::Head, BodyOwner::Reaction), 0u);
    }

    TEST(RagdollCoreBodyLease, nonFiniteRequestsAreSanitized)
    {
        BodyLeaseTable table;
        LeaseRequest request = pose(std::numeric_limits<float>::quiet_NaN());
        request.mTorqueScale = std::numeric_limits<float>::infinity();
        request.mSpringScale = -3.0f;
        const LeaseId id = table.acquire(BodyChannel::Head, BodyOwner::Reaction, request,
            { std::numeric_limits<float>::quiet_NaN(), -1.0f, std::numeric_limits<float>::quiet_NaN() });
        EXPECT_NE(id, sNoLease);
        const auto resolved = table.resolve(BodyChannel::Head);
        EXPECT_FLOAT_EQ(resolved.mTorqueScale, 1.0f);
        EXPECT_FLOAT_EQ(resolved.mSpringScale, 1.0f);
        EXPECT_EQ(table.acquire(BodyChannel::Count, BodyOwner::Reaction, request), sNoLease);
        table.step(std::numeric_limits<float>::quiet_NaN());
        table.step(-1.0f);
    }

    TEST(RagdollCoreBodyLease, channelsByBodyName)
    {
        EXPECT_EQ(bodyChannelForName("pelvis"), BodyChannel::Root);
        EXPECT_EQ(bodyChannelForName("spine1"), BodyChannel::Trunk);
        EXPECT_EQ(bodyChannelForName("head"), BodyChannel::Head);
        EXPECT_EQ(bodyChannelForName("neck"), BodyChannel::Head);
        EXPECT_EQ(bodyChannelForName("left_calf"), BodyChannel::LeftLeg);
        EXPECT_EQ(bodyChannelForName("right_foot"), BodyChannel::RightLeg);
        EXPECT_EQ(bodyChannelForName("left_clavicle"), BodyChannel::LeftArm);
        EXPECT_EQ(bodyChannelForName("right_hand"), BodyChannel::RightArm);
        EXPECT_FALSE(bodyChannelForName("weapon"));
        EXPECT_EQ(bodyChannelForRegion(AnatomicalRegion::Neck), BodyChannel::Head);
        EXPECT_EQ(bodyChannelForRegion(AnatomicalRegion::Abdomen), BodyChannel::Trunk);
        EXPECT_EQ(bodyChannelForRegion(AnatomicalRegion::RightHand), BodyChannel::RightArm);
        EXPECT_EQ(bodyChannelForRegion(AnatomicalRegion::LeftFoot), BodyChannel::LeftLeg);
        EXPECT_FALSE(bodyChannelForRegion(AnatomicalRegion::Unknown));
    }
}
