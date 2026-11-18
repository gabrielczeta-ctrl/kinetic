// KINETIC-2 phases 3 and 4, the second moves: the drop roll, the fall
// protection's pose and bracing, the root's target, the physical death and
// the rebuild rule. Each is the old integration's rule moved as it was; the
// game's shadow runs (K2_* = 2) pin the equality.
#include <ragdollcore/droproll.hpp>
#include <ragdollcore/fallprotectpose.hpp>
#include <ragdollcore/lifecycle.hpp>
#include <ragdollcore/roottarget.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{
    using namespace RagdollCore;

    constexpr float sDt = 1.0f / 60.0f;

    // Rolling over the x axis: up turns toward +y as theta grows.
    JPH::Vec3 upAt(float theta)
    {
        return JPH::Vec3(0.0f, std::sin(theta), std::cos(theta));
    }

    TEST(K2DropRoll, ThetaIsTheAngleAboutTheAxisFromUp)
    {
        const JPH::Vec3 axis = JPH::Vec3::sAxisX();
        EXPECT_NEAR(dropRollTheta(axis, upAt(0.0f)), 0.0f, 1e-5f);
        EXPECT_NEAR(dropRollTheta(axis, upAt(1.0f)), -1.0f, 1e-5f); // axis x Z = -y
    }

    TEST(K2DropRoll, AbsorbTurnsToRollThenTheRollStandsUp)
    {
        // axis -x: e = (-x) x z = +y, so theta grows as up turns toward +y.
        const JPH::Vec3 axis = -JPH::Vec3::sAxisX();
        DropRollState state;
        state.mPhase = RootRollPhase::Absorb;
        DropRollInput in;
        in.mDt = sDt;
        in.mAxis = axis;
        in.mAlong = 200.0f;
        in.mImpact = 700.0f;
        in.mFootSupported = true;
        const DropRollSettings settings;
        float theta = 0.0f;
        bool rolled = false, rose = false;
        const char* end = nullptr;
        for (int i = 0; i < 200 && !end; ++i)
        {
            state.mPhaseTime += sDt;
            state.mTotal += sDt;
            // The body follows the target with a lag.
            theta += (state.mTarget - theta) * 0.5f + 0.02f;
            in.mPelvisUp = upAt(theta);
            in.mTorsoUp = rose ? JPH::Vec3::sAxisZ() : upAt(theta);
            const DropRollStep step = stepDropRoll(state, in, settings);
            rolled = rolled || step.mToRoll;
            rose = rose || step.mToRise;
            end = step.mEnd;
            if (end)
                EXPECT_FALSE(step.mFall);
        }
        EXPECT_TRUE(rolled);
        EXPECT_TRUE(rose);
        ASSERT_NE(end, nullptr);
        EXPECT_STREQ(end, "stand");
        EXPECT_TRUE(state.mHandsReleased);
    }

    TEST(K2DropRoll, AStallEndsTheRollInAFall)
    {
        DropRollState state;
        state.mPhase = RootRollPhase::Roll;
        DropRollInput in;
        in.mDt = sDt;
        in.mAxis = -JPH::Vec3::sAxisX();
        in.mPelvisUp = upAt(1.0f);
        in.mTorsoUp = upAt(1.0f);
        const char* end = nullptr;
        bool fall = false;
        for (int i = 0; i < 30 && !end; ++i)
        {
            state.mPhaseTime += sDt;
            state.mTotal += sDt;
            const auto step = stepDropRoll(state, in, {});
            end = step.mEnd;
            fall = step.mFall;
        }
        ASSERT_NE(end, nullptr);
        EXPECT_STREQ(end, "stalled");
        EXPECT_TRUE(fall);
    }

    TEST(K2DropRoll, TheRootPitchesAboutTheAxisAndSinksOnTheAbsorb)
    {
        DropRollRootInput in;
        in.mPhase = RootRollPhase::Absorb;
        in.mPhaseTime = 0.12f;
        in.mImpact = 1000.0f;
        in.mTarget = 0.5f;
        in.mAxis = JPH::Vec3::sAxisX();
        JPH::RVec3 position(0, 0, 70);
        JPH::Quat rotation = JPH::Quat::sIdentity();
        dropRollRootPose(in, position, rotation);
        EXPECT_NEAR(position.GetZ(), 40.0, 1e-4);
        EXPECT_TRUE(rotation.IsClose(JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.5f)));
        in.mPhase = RootRollPhase::Roll;
        in.mPelvisZ = JPH::Real(12);
        position = JPH::RVec3(0, 0, 70);
        dropRollRootPose(in, position, rotation);
        EXPECT_NEAR(position.GetZ(), 12.0, 1e-6);
    }

    TEST(K2FallProtect, TheSpineCurlCarriesTheHeadForward)
    {
        // Pelvis 0, spine 1-2 up the z axis, head 3, neck 4, arms 5-12, legs 13-18.
        std::vector<Transform> pose(19);
        pose[1].mTranslation = JPH::Vec3(0, 0, 10);
        pose[2].mTranslation = JPH::Vec3(0, 0, 20);
        pose[4].mTranslation = JPH::Vec3(0, 0, 28);
        pose[3].mTranslation = JPH::Vec3(0, 0, 32);
        pose[5].mTranslation = JPH::Vec3(-5, 0, 24); // left clavicle
        pose[9].mTranslation = JPH::Vec3(5, 0, 24); // right clavicle
        FallPoseChains chains;
        chains.mSpine = { 1, 2 };
        chains.mHead = 3;
        chains.mNeck = 4;
        chains.mArms[0] = { 5, 6, 7, 8 };
        chains.mArms[1] = { 9, 10, 11, 12 };
        chains.mLegs[0] = { 13, 14, 15 };
        chains.mLegs[1] = { 16, 17, 18 };
        FallProtectCommand command;
        command.mSpineCurl = 0.5f;
        applyFallProtectPose(pose, command, chains, JPH::Quat::sIdentity());
        // Forward is +y (z x across, across = +x): the head comes forward.
        EXPECT_GT(pose[3].mTranslation.GetY(), 5.0f);
        EXPECT_LT(pose[3].mTranslation.GetZ(), 32.0f);
        EXPECT_TRUE(pose[0].mTranslation.IsNearZero());
    }

    TEST(K2FallProtect, ContactRadiiAndBracing)
    {
        EXPECT_FALSE(fallContactRadius(AnatomicalRegion::LeftFoot, true));
        EXPECT_FALSE(fallContactRadius(AnatomicalRegion::LeftThigh, false));
        EXPECT_EQ(fallContactRadius(AnatomicalRegion::LeftThigh, true), 6.0f);
        EXPECT_EQ(fallContactRadius(AnatomicalRegion::Head, false), 9.0f);
        EXPECT_FALSE(fallBodyContact(AnatomicalRegion::RightForearm));
        BraceInput in;
        in.mEnabled = true;
        in.mMode = ActiveControllerMode::Falling;
        in.mTimeInMode = 0.1f;
        in.mCenterOfMassVelocity = JPH::Vec3(0, -50, -200);
        auto d = decideBraceArms(in);
        EXPECT_TRUE(d.mWanted);
        EXPECT_TRUE(d.mProtectiveFall);
        EXPECT_TRUE(d.mFallDirection.IsClose(JPH::Vec3(0, -1, 0)));
        in.mTimeInMode = 0.3f;
        EXPECT_FALSE(decideBraceArms(in).mWanted);
        in.mMode = ActiveControllerMode::Fallen;
        in.mGroundContact = true;
        in.mRecoveryRequested = true;
        EXPECT_FALSE(decideBraceArms(in).mWanted);
        in.mFallenBrace = true;
        d = decideBraceArms(in);
        EXPECT_TRUE(d.mWanted);
        EXPECT_FALSE(d.mProtectiveFall);
    }

    TEST(K2RootTarget, TheRunBobFlattensAndSinksAStraightRun)
    {
        RunBobState state;
        const RunBobSettings settings{ 0.3f, 5.0f, 250.0f };
        const JPH::Vec3 run(0, 260, 0);
        float z = 0.0f;
        for (int i = 0; i < 120; ++i)
            z = applyRunBob(state, 60.0f + (i % 2 ? 4.0f : -4.0f), run, true, settings, sDt);
        EXPECT_GT(state.mShare, 0.99f);
        EXPECT_NEAR(z, 60.0f + 4.0f * 0.3f - 5.0f, 0.6f);
        // Walking: the clip as it is.
        RunBobState walk;
        EXPECT_FLOAT_EQ(applyRunBob(walk, 64.0f, JPH::Vec3(0, 127, 0), true, settings, sDt), 64.0f);
    }

    TEST(K2RootTarget, TheTargetTakesItsDropsAndRecoil)
    {
        RootTargetInput in;
        in.mActor = JPH::RVec3(100, 200, 10);
        in.mOffset = JPH::Vec3(0, 2, 66);
        in.mDrop = 6.0f;
        in.mTeeter = 0.5f;
        in.mTeeterToward = JPH::Vec3(1, 0, 0);
        in.mTeeterHips = 10.0f;
        in.mRecoil = JPH::Vec3(0, 3, 0);
        const JPH::RVec3 at = rootTargetPosition(in);
        EXPECT_NEAR(at.GetX(), 95.0, 1e-4);
        EXPECT_NEAR(at.GetY(), 205.0, 1e-4);
        EXPECT_NEAR(at.GetZ(), 10.0 + 66.0 - 6.0 - 2.0, 1e-4);
    }

    TEST(K2RootTarget, TheFollowerCarriesMomentumThroughAStop)
    {
        RootFollowerState state;
        JPH::RVec3 capsule(0, 0, 0);
        for (int i = 0; i < 60; ++i)
        {
            capsule += JPH::RVec3(0, 230.0 * sDt, 0);
            followRootTarget(state, capsule, 1500.0f, sDt);
        }
        // The capsule stops dead; the follower brakes at most 1500 u/s^2.
        const JPH::RVec3 stop = capsule;
        const JPH::RVec3 first = followRootTarget(state, stop, 1500.0f, sDt);
        EXPECT_GT(state.mVelocity.GetY(), 200.0f);
        EXPECT_GT(first.GetY(), stop.GetY() - 30.0);
        // Off: the capsule itself.
        RootFollowerState off;
        EXPECT_TRUE(followRootTarget(off, JPH::RVec3(5, 5, 5), 0.0f, sDt).IsClose(JPH::RVec3(5, 5, 5)));
        // The yield.
        EXPECT_TRUE(rootYieldVelocity(JPH::RVec3(10, 0, 50), JPH::RVec3(0, 0, 0), 4.0f, 2.0f, 250.0f)
                        .IsClose(JPH::Vec3(12, 0, 0)));
        EXPECT_TRUE(rootYieldVelocity(JPH::RVec3(3, 0, 0), JPH::RVec3(0, 0, 0), 4.0f, 2.0f, 250.0f).IsNearZero());
    }

    TEST(K2RootTarget, TheGaitCapHoldsTheCapsuleToWhatTheLegsCarry)
    {
        // A 680 u/s charge, diagonal, falling a little: held to 340 along the
        // same heading, the vertical part kept.
        const JPH::Vec3 charge(480.83f, 480.83f, -20.0f);
        const JPH::Vec3 capped = capGaitSpeed(charge, 340.0f);
        EXPECT_NEAR(std::hypot(capped.GetX(), capped.GetY()), 340.0f, 0.05f);
        EXPECT_NEAR(capped.GetX(), capped.GetY(), 1e-3f);
        EXPECT_FLOAT_EQ(capped.GetZ(), -20.0f);
        // Under the cap, and off: unchanged.
        EXPECT_TRUE(capGaitSpeed(JPH::Vec3(0, 230, 0), 340.0f).IsClose(JPH::Vec3(0, 230, 0)));
        EXPECT_TRUE(capGaitSpeed(charge, 0.0f).IsClose(charge));
    }

    TEST(K2RootTarget, TheFieldShareRisesFromARunToASprint)
    {
        const RootFieldSettings field{ 250.0f, 400.0f };
        EXPECT_FLOAT_EQ(rootFieldShare(field, 230.0f), 0.0f);
        EXPECT_NEAR(rootFieldShare(field, 325.0f), 0.5f, 1e-4f);
        EXPECT_FLOAT_EQ(rootFieldShare(field, 680.0f), 1.0f);
        EXPECT_FLOAT_EQ(rootFieldShare(RootFieldSettings{}, 680.0f), 0.0f);
        EXPECT_FLOAT_EQ(rootFieldShare(RootFieldSettings{ 300.0f, 0.0f }, 300.0f), 1.0f);
    }

    TEST(K2Lifecycle, APhysicalDeathGoesLimpWhenItLiesOrAtItsLimit)
    {
        PhysicalDeathSettings settings{ true, 1.4f, 6000.0f };
        PhysicalDeathState state;
        EXPECT_FALSE(beginPhysicalDeath(state, settings, true, ActiveControllerMode::Falling));
        ASSERT_TRUE(beginPhysicalDeath(state, settings, true, ActiveControllerMode::Standing));
        EXPECT_FALSE(beginPhysicalDeath(state, settings, true, ActiveControllerMode::Standing));
        int steps = 0;
        while (!stepPhysicalDeath(state, settings, false, true, sDt) && steps < 200)
            ++steps;
        EXPECT_NEAR(state.mTime, 1.4f, 2 * sDt);
        PhysicalDeathState lying;
        beginPhysicalDeath(lying, settings, true, ActiveControllerMode::Bracing);
        EXPECT_TRUE(stepPhysicalDeath(lying, settings, true, true, sDt));
        EXPECT_FALSE(stepPhysicalDeath(lying, settings, true, true, sDt));
        PhysicalDeathState off;
        EXPECT_FALSE(beginPhysicalDeath(off, {}, true, ActiveControllerMode::Standing));
    }

    TEST(K2Lifecycle, ARigRebuildWaitsForAQuietMoment)
    {
        RebuildInput in;
        in.mRigChanged = true;
        in.mStanding = true;
        in.mRootKinematic = true;
        EXPECT_TRUE(rebuildForRig(in));
        in.mBusy = true;
        EXPECT_FALSE(rebuildForRig(in));
        in.mBusy = false;
        in.mStanding = false;
        EXPECT_FALSE(rebuildForRig(in));
        EXPECT_TRUE(rebuildKeepsHeight(true, true));
        EXPECT_FALSE(rebuildKeepsHeight(true, false));
    }

    // K2_REGRIP (11-16 bookmark: "the spear ... only attached to the right
    // hand"): a broken grip on a weapon still equipped is rebuilt; a dropped
    // or unequipped weapon is not.
    TEST(K2Lifecycle, ABrokenGripOnAnEquippedWeaponIsGrippedAgain)
    {
        RegripInput grip;
        grip.mReleased = true;
        grip.mBroke = true;
        grip.mStillEquipped = true;
        grip.mSinceRelease = 0.5f;
        EXPECT_TRUE(regripAfterRelease(grip, 0.5f));
        EXPECT_FALSE(regripAfterRelease(grip, 0.0f)); // off
        grip.mSinceRelease = 0.4f;
        EXPECT_FALSE(regripAfterRelease(grip, 0.5f)); // too soon
        grip.mSinceRelease = 2.0f;
        grip.mBroke = false;
        EXPECT_FALSE(regripAfterRelease(grip, 0.5f)); // dropped, unequipped, death
        grip.mBroke = true;
        grip.mStillEquipped = false;
        EXPECT_FALSE(regripAfterRelease(grip, 0.5f)); // another weapon or none
        grip.mStillEquipped = true;
        grip.mReleased = false;
        EXPECT_FALSE(regripAfterRelease(grip, 0.5f)); // still held
        // The re-grip waits for the same quiet moment as an equipment change.
        RebuildInput in;
        in.mRegrip = true;
        in.mStanding = true;
        in.mRootKinematic = true;
        EXPECT_TRUE(rebuildForRig(in));
        in.mBusy = true;
        EXPECT_FALSE(rebuildForRig(in));
    }
}
