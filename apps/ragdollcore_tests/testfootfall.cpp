#include <ragdollcore/footfall.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace
{
    using namespace RagdollCore;

    // A pelvis and two legs (hip = thigh body, knee = shin body, foot) in
    // model space, standing on z = 0 with the pelvis at 75 units; the body
    // faces +y (yaw 0).
    enum Body : std::size_t
    {
        Pelvis,
        LeftHip,
        LeftKnee,
        LeftFoot,
        RightHip,
        RightKnee,
        RightFoot,
        Count
    };

    RagdollPose standingPose(JPH::Vec3Arg offset)
    {
        RagdollPose pose;
        pose.mBodyTransforms.resize(Count);
        const auto set = [&](Body body, float x, float z) {
            pose.mBodyTransforms[body].mTranslation = offset + JPH::Vec3(x, 0.0f, z);
        };
        set(Pelvis, 0.0f, 75.0f);
        set(LeftHip, -9.0f, 70.0f);
        set(LeftKnee, -9.0f, 38.0f);
        set(LeftFoot, -9.0f, 6.5f);
        set(RightHip, 9.0f, 70.0f);
        set(RightKnee, 9.0f, 38.0f);
        set(RightFoot, 9.0f, 6.5f);
        return pose;
    }

    struct Rig
    {
        FootfallController controller;
        RagdollPose physics = standingPose(JPH::Vec3::sZero());
        RagdollPose clip = standingPose(JPH::Vec3::sZero());
        FootfallInput input;

        Rig()
        {
            input.mDt = 1.0f / 60.0f;
            input.mEligible = true;
            input.mPelvis = Pelvis;
            input.mLegs[0] = FootfallLeg{ LeftHip, LeftKnee, LeftFoot, std::nullopt };
            input.mLegs[1] = FootfallLeg{ RightHip, RightKnee, RightFoot, std::nullopt };
            input.mPhysicsPose = &physics;
            input.mClipPose = &clip;
            input.mGround = [](const JPH::RVec3&, double) -> std::optional<double> { return 0.0; };
        }

        // One step with the body carried along at `velocity` (a perfectly
        // tracking root): the physical pose is the standing pose moved to
        // where the root is, and the feet are where Footfall drove them.
        FootfallOutput step(JPH::Vec3Arg velocity, JPH::Vec3& root, std::vector<RagdollPose>* drives = nullptr)
        {
            root += velocity * input.mDt;
            RagdollPose drive = standingPose(root);
            const auto previousLeft = physics.mBodyTransforms[LeftFoot].mTranslation;
            const auto previousRight = physics.mBodyTransforms[RightFoot].mTranslation;
            physics = standingPose(root);
            physics.mBodyTransforms[LeftFoot].mTranslation = previousLeft;
            physics.mBodyTransforms[RightFoot].mTranslation = previousRight;
            clip = standingPose(root);
            input.mCommandVelocity = velocity;
            FootfallOutput output;
            controller.step(input, drive, output);
            // The physical feet follow the driven feet exactly.
            physics.mBodyTransforms[LeftFoot].mTranslation = drive.mBodyTransforms[LeftFoot].mTranslation;
            physics.mBodyTransforms[RightFoot].mTranslation = drive.mBodyTransforms[RightFoot].mTranslation;
            if (drives)
                drives->push_back(drive);
            return output;
        }
    };

    TEST(RagdollCoreFootfall, anIneligibleBodyResets)
    {
        Rig rig;
        rig.input.mEligible = false;
        JPH::Vec3 root = JPH::Vec3::sZero();
        const auto output = rig.step(JPH::Vec3(0.0f, 60.0f, 0.0f), root);
        EXPECT_FALSE(output.mPinned[0] || output.mPinned[1] || output.mSwinging[0] || output.mSwinging[1]);
        EXPECT_FALSE(rig.controller.walking());
    }

    TEST(RagdollCoreFootfall, standingPinsBothFeetAndTakesNoStep)
    {
        Rig rig;
        JPH::Vec3 root = JPH::Vec3::sZero();
        for (int i = 0; i < 60; ++i)
        {
            const auto output = rig.step(JPH::Vec3::sZero(), root);
            EXPECT_TRUE(output.mPinned[0] && output.mPinned[1]) << i;
            EXPECT_FALSE(output.mSwinging[0] || output.mSwinging[1]) << i;
        }
        EXPECT_NEAR(rig.controller.pelvisDrop(), 0.0f, 1e-6f);
    }

    TEST(RagdollCoreFootfall, walkingAlternatesStepsThatLandAhead)
    {
        Rig rig;
        JPH::Vec3 root = JPH::Vec3::sZero();
        for (int i = 0; i < 30; ++i)
            rig.step(JPH::Vec3::sZero(), root);
        const JPH::Vec3 velocity(0.0f, 60.0f, 0.0f);
        std::vector<std::size_t> lifts;
        std::vector<float> liftTimes;
        std::array<bool, 2> was{ false, false };
        int bothSwinging = 0;
        for (int i = 0; i < 240; ++i)
        {
            const auto output = rig.step(velocity, root);
            for (std::size_t side = 0; side < 2; ++side)
            {
                if (output.mSwinging[side] && !was[side])
                {
                    lifts.push_back(side);
                    liftTimes.push_back(static_cast<float>(i) * rig.input.mDt);
                    // The step aims ahead of the pelvis along the travel.
                    const auto& foot = rig.controller.feet()[side];
                    EXPECT_GT(foot.mTarget.GetY(), root.GetY()) << i;
                    // On its own side (left foot at -x of the pelvis).
                    EXPECT_LT((side == 0 ? 1.0f : -1.0f) * (foot.mTarget.GetX() - root.GetX()), 0.0f) << i;
                }
                was[side] = output.mSwinging[side];
            }
            bothSwinging += output.mSwinging[0] && output.mSwinging[1];
        }
        EXPECT_TRUE(rig.controller.walking());
        ASSERT_GE(lifts.size(), 6u);
        for (std::size_t i = 1; i < lifts.size(); ++i)
            EXPECT_NE(lifts[i], lifts[i - 1]) << i;
        // A walk keeps a double support: never both feet up.
        EXPECT_EQ(bothSwinging, 0);
        // Steps come at a walking cadence (0.26-0.6 s per step).
        const float mean = (liftTimes.back() - liftTimes.front()) / static_cast<float>(liftTimes.size() - 1);
        EXPECT_GT(mean, 0.26f);
        EXPECT_LT(mean, 0.6f);
    }

    TEST(RagdollCoreFootfall, stoppingSettlesBackToStanding)
    {
        Rig rig;
        JPH::Vec3 root = JPH::Vec3::sZero();
        for (int i = 0; i < 120; ++i)
            rig.step(JPH::Vec3(0.0f, 60.0f, 0.0f), root);
        FootfallOutput output;
        for (int i = 0; i < 120; ++i)
            output = rig.step(JPH::Vec3::sZero(), root);
        EXPECT_FALSE(rig.controller.walking());
        EXPECT_TRUE(output.mPinned[0] && output.mPinned[1]);
    }

    TEST(RagdollCoreFootfall, anEmptyCommandChangesNothing)
    {
        Rig plain, commanded;
        JPH::Vec3 rootA = JPH::Vec3::sZero(), rootB = JPH::Vec3::sZero();
        for (int i = 0; i < 200; ++i)
        {
            const JPH::Vec3 velocity = i < 30 ? JPH::Vec3::sZero() : JPH::Vec3(0.0f, 60.0f, 0.0f);
            const auto a = plain.step(velocity, rootA);
            commanded.controller.command({});
            const auto b = commanded.step(velocity, rootB);
            ASSERT_EQ(a.mSwinging, b.mSwinging) << i;
            for (std::size_t side = 0; side < 2; ++side)
            {
                EXPECT_EQ(plain.controller.feet()[side].mPin, commanded.controller.feet()[side].mPin) << i;
                EXPECT_EQ(plain.controller.feet()[side].mTarget, commanded.controller.feet()[side].mTarget) << i;
            }
        }
    }

    TEST(RagdollCoreFootfall, swingNowStepsFromAStandToTheGivenLanding)
    {
        Rig rig;
        JPH::Vec3 root = JPH::Vec3::sZero();
        for (int i = 0; i < 30; ++i)
            rig.step(JPH::Vec3::sZero(), root);
        FootfallCommand command;
        command.mHoldClock = true;
        command.mSwingNow = 1;
        command.mSwingTime = 0.3f;
        command.mLanding[1] = JPH::RVec3(12.0, 25.0, 0.0);
        rig.controller.command(command);
        auto output = rig.step(JPH::Vec3::sZero(), root);
        EXPECT_TRUE(output.mSwinging[1]);
        EXPECT_TRUE(output.mPinned[0]);
        EXPECT_NEAR(rig.controller.feet()[1].mTarget.GetX(), 12.0, 1e-4);
        EXPECT_NEAR(rig.controller.feet()[1].mTarget.GetY(), 25.0, 1e-4);
        // It lands after about the swing time and Footfall takes no step of
        // its own while the clock is held.
        int swingSteps = 1, lifts = 0;
        for (int i = 0; i < 60; ++i)
        {
            FootfallCommand hold;
            hold.mHoldClock = true;
            rig.controller.command(hold);
            const bool was = output.mSwinging[0] || output.mSwinging[1];
            output = rig.step(JPH::Vec3::sZero(), root);
            swingSteps += output.mSwinging[1] ? 1 : 0;
            lifts += !was && (output.mSwinging[0] || output.mSwinging[1]) ? 1 : 0;
        }
        // Footfall's own rule: a foot still up at the end of its swing keeps
        // reaching down for up to 0.2 s more; it plants where it really is.
        EXPECT_GE(swingSteps * rig.input.mDt, 0.3f - 0.02f);
        EXPECT_LE(swingSteps * rig.input.mDt, 0.5f + 0.05f);
        EXPECT_EQ(lifts, 0);
        EXPECT_NEAR(rig.controller.feet()[1].mPin.GetY(), 25.0, 3.0);
    }

    TEST(RagdollCoreFootfall, aHeldClockTakesNoStepWhileWalking)
    {
        Rig rig;
        JPH::Vec3 root = JPH::Vec3::sZero();
        for (int i = 0; i < 30; ++i)
            rig.step(JPH::Vec3::sZero(), root);
        int lifts = 0;
        std::array<bool, 2> was{ false, false };
        for (int i = 0; i < 120; ++i)
        {
            FootfallCommand hold;
            hold.mHoldClock = true;
            rig.controller.command(hold);
            const auto output = rig.step(JPH::Vec3(0.0f, 60.0f, 0.0f), root);
            for (std::size_t side = 0; side < 2; ++side)
            {
                lifts += output.mSwinging[side] && !was[side] ? 1 : 0;
                was[side] = output.mSwinging[side];
            }
        }
        // Only a leg that runs out of reach would step (overrun is part of
        // the clock too, so nothing lifts).
        EXPECT_EQ(lifts, 0);
        EXPECT_FALSE(rig.controller.walking());
        // Released, Footfall walks again.
        for (int i = 0; i < 120; ++i)
        {
            const auto output = rig.step(JPH::Vec3(0.0f, 60.0f, 0.0f), root);
            for (std::size_t side = 0; side < 2; ++side)
            {
                lifts += output.mSwinging[side] && !was[side] ? 1 : 0;
                was[side] = output.mSwinging[side];
            }
        }
        EXPECT_GE(lifts, 2);
        EXPECT_TRUE(rig.controller.walking());
    }

    TEST(RagdollCoreFootfall, anOverriddenLandingIsNotReAimed)
    {
        Rig rig;
        JPH::Vec3 root = JPH::Vec3::sZero();
        for (int i = 0; i < 30; ++i)
            rig.step(JPH::Vec3::sZero(), root);
        std::optional<std::size_t> swinging;
        for (int i = 0; i < 240 && !swinging; ++i)
        {
            const auto output = rig.step(JPH::Vec3(0.0f, 60.0f, 0.0f), root);
            for (std::size_t side = 0; side < 2; ++side)
                if (output.mSwinging[side])
                    swinging = side;
        }
        ASSERT_TRUE(swinging);
        const JPH::RVec3 landing(root.GetX() + (*swinging == 0 ? -20.0 : 20.0), root.GetY() + 40.0, 0.0);
        FootfallCommand command;
        command.mLanding[*swinging] = landing;
        rig.controller.command(command);
        rig.step(JPH::Vec3(0.0f, 60.0f, 0.0f), root);
        for (int i = 0; i < 6 && rig.controller.feet()[*swinging].mSwinging; ++i)
        {
            EXPECT_NEAR(rig.controller.feet()[*swinging].mTarget.GetX(), landing.GetX(), 1e-4) << i;
            EXPECT_NEAR(rig.controller.feet()[*swinging].mTarget.GetY(), landing.GetY(), 1e-4) << i;
            rig.step(JPH::Vec3(0.0f, 60.0f, 0.0f), root);
        }
    }

    // Stairs along +y: 40 u treads rising 16 u. With mTreadSnap the landing
    // targets keep the foot (half length 11) off the riser edges.
    struct StairLandings
    {
        int total = 0, nearEdge = 0;
        double closest = 1e9;
    };
    StairLandings walkUpStairs(float snap)
    {
        FootfallSettings settings;
        settings.mTreadSnap = snap;
        Rig rig;
        rig.controller = FootfallController(settings);
        rig.input.mGround = [](const JPH::RVec3& p, double) -> std::optional<double> {
            return 16.0 * std::floor(p.GetY() / 40.0);
        };
        JPH::Vec3 root = JPH::Vec3::sZero();
        for (int i = 0; i < 30; ++i)
            rig.step(JPH::Vec3::sZero(), root);
        StairLandings out;
        std::array<bool, 2> was{ false, false };
        std::array<double, 2> lastTarget{ 0.0, 0.0 };
        for (int i = 0; i < 360; ++i)
        {
            const auto output = rig.step(JPH::Vec3(0.0f, 60.0f, 0.0f), root);
            for (std::size_t side = 0; side < 2; ++side)
            {
                if (was[side] && !output.mSwinging[side])
                {
                    const double y = lastTarget[side];
                    const double r = std::fmod(std::fmod(y, 40.0) + 40.0, 40.0);
                    const double edge = std::min(r, 40.0 - r);
                    ++out.total;
                    out.nearEdge += edge < 8.0;
                    out.closest = std::min(out.closest, edge);
                }
                if (output.mSwinging[side])
                    lastTarget[side] = rig.controller.feet()[side].mTarget.GetY();
                was[side] = output.mSwinging[side];
            }
        }
        return out;
    }

    TEST(RagdollCoreFootfall, stairLandingsKeepOffTheTreadEdges)
    {
        const auto plain = walkUpStairs(0.0f);
        const auto snapped = walkUpStairs(12.0f);
        ASSERT_GE(plain.total, 6);
        ASSERT_GE(snapped.total, 6);
        std::printf("tread edges: plain %d/%d near (closest %.1f), snapped %d/%d near (closest %.1f)%c",
            plain.nearEdge, plain.total, plain.closest, snapped.nearEdge, snapped.total, snapped.closest, 10);
        EXPECT_EQ(snapped.nearEdge, 0);
        EXPECT_GE(snapped.closest, 8.0); // the edges are found to the 4 u sampling
    }

    // The stair gait (mStair): walking down tall stairs (24 u risers, 40 u
    // treads) along +y, the root riding the ground under it; the capsule
    // (mActorZ) one riser above it, and with `window` the ground probe sees
    // only 40 u under its reference (the engine's).
    struct StairDown
    {
        int heelClips = 0; // swing frames, phase < 0.45, sole under the ground at the heel
        int landings = 0;
        int nearEdge = 0;
        int lockRejects = 0;
    };

    StairDown walkDownStairs(bool stairGait, bool window)
    {
        FootfallSettings settings;
        settings.mTreadSnap = 12.0f;
        if (stairGait)
            settings.mStair = { 0.3f, 0.7f, 0.85f, 0.92f, 3.0f, 1.0f };
        Rig rig;
        rig.controller = FootfallController(settings);
        const auto stairs = [](double y) { return -24.0 * std::floor(y / 40.0); };
        rig.input.mGround = [stairs, window](const JPH::RVec3& p, double reference) -> std::optional<double> {
            const double g = stairs(p.GetY());
            if (window && (g < reference - 40.0 || g > reference + 30.0))
                return std::nullopt;
            return g;
        };
        StairDown out;
        rig.input.mTrace = [&out](const std::string& line) {
            if (line.find("lock_reject") != std::string::npos)
                ++out.lockRejects;
        };
        JPH::Vec3 root(0.0f, 20.0f, 0.0f);
        const auto ride = [&] {
            root.SetZ(static_cast<float>(stairs(root.GetY())));
            rig.input.mActorZ = root.GetZ() + 24.0;
        };
        for (int i = 0; i < 30; ++i)
        {
            ride();
            rig.step(JPH::Vec3::sZero(), root);
        }
        std::array<bool, 2> was{ false, false };
        std::array<double, 2> lastTarget{ 0.0, 0.0 };
        for (int i = 0; i < 480; ++i)
        {
            ride();
            std::vector<RagdollPose> drives;
            const auto output = rig.step(JPH::Vec3(0.0f, 60.0f, 0.0f), root, &drives);
            for (std::size_t side = 0; side < 2; ++side)
            {
                const auto& foot = rig.controller.feet()[side];
                const Body body = side == 0 ? LeftFoot : RightFoot;
                // From the first landings on (the rig's first step still drives the
                // standing pose's foot).
                if (output.mSwinging[side] && foot.mPhase < 0.45f && i >= 60)
                {
                    const JPH::Vec3 at = drives.back().mBodyTransforms[body].mTranslation;
                    const double heel = stairs(at.GetY() - 11.0);
                    // The nosing: the heel still over a higher tread than the toe.
                    if (at.GetZ() - 6.5 < heel - 0.5 && heel > stairs(at.GetY()) + 4.0
                        && stairs(foot.mPin.GetY()) > stairs(foot.mTarget.GetY()) + 4.0)
                    {
                        ++out.heelClips;
                        if (std::getenv("STAIR_DEBUG"))
                            std::printf("clip i=%d side=%zu phase=%.2f down=%d stair=%d pinY=%.1f pinG=%.1f tgtY=%.1f tgtG=%.1f footY=%.1f soleZ=%.1f heelG=%.1f%c",
                                i, side, foot.mPhase, foot.mStepDown, foot.mStair, foot.mPin.GetY(), stairs(foot.mPin.GetY()),
                                foot.mTarget.GetY(), stairs(foot.mTarget.GetY()), at.GetY(), at.GetZ() - 6.5, heel, 10);
                    }
                }
                if (was[side] && !output.mSwinging[side])
                {
                    const double r = std::fmod(std::fmod(lastTarget[side], 40.0) + 40.0, 40.0);
                    ++out.landings;
                    out.nearEdge += std::min(r, 40.0 - r) < 8.0;
                }
                if (output.mSwinging[side])
                    lastTarget[side] = foot.mTarget.GetY();
                was[side] = output.mSwinging[side];
            }
        }
        return out;
    }

    TEST(RagdollCoreFootfall, stairDownSwingClearsTheNosing)
    {
        for (const bool window : { false, true })
        {
            const auto off = walkDownStairs(false, window);
            const auto on = walkDownStairs(true, window);
            std::printf("stairs down (window %d): off clips %d, edge %d/%d; on clips %d, edge %d/%d, locks %d%c", window,
                off.heelClips, off.nearEdge, off.landings, on.heelClips, on.nearEdge, on.landings, on.lockRejects, 10);
            ASSERT_GE(on.landings, 6);
            EXPECT_EQ(on.heelClips, 0);
            EXPECT_EQ(on.nearEdge, 0);
        }
    }

    TEST(RagdollCoreFootfall, theStairGaitLeavesFlatGroundAndSlopesAlone)
    {
        for (const double grade : { 0.0, -0.7, 0.7 })
        {
            std::array<std::vector<RagdollPose>, 2> drives;
            for (const bool on : { false, true })
            {
                FootfallSettings settings;
                settings.mTreadSnap = 12.0f;
                if (on)
                    settings.mStair = { 0.3f, 0.7f, 0.85f, 0.92f, 3.0f, 1.0f };
                Rig rig;
                rig.controller = FootfallController(settings);
                rig.input.mGround = [grade](const JPH::RVec3& p, double) -> std::optional<double> {
                    return grade * p.GetY();
                };
                JPH::Vec3 root = JPH::Vec3::sZero();
                for (int i = 0; i < 200; ++i)
                {
                    root.SetZ(static_cast<float>(grade * root.GetY()));
                    rig.input.mActorZ = root.GetZ();
                    rig.step(JPH::Vec3(0.0f, i < 20 ? 0.0f : 60.0f, 0.0f), root, &drives[on ? 1 : 0]);
                }
            }
            ASSERT_EQ(drives[0].size(), drives[1].size());
            int differing = 0;
            for (std::size_t i = 0; i < drives[0].size(); ++i)
                for (std::size_t b = 0; b < Count; ++b)
                    differing += drives[0][i].mBodyTransforms[b].mTranslation != drives[1][i].mBodyTransforms[b].mTranslation;
            EXPECT_EQ(differing, 0) << "grade " << grade;
        }
    }

    TEST(RagdollCoreFootfall, settingsReadTheStairGait)
    {
#ifdef _WIN32
        _putenv_s("OPENMW_RAGDOLL_FOOTFALL_STAIR", "0.3,0.7,0.85,0.92,3,1");
#else
        setenv("OPENMW_RAGDOLL_FOOTFALL_STAIR", "0.3,0.7,0.85,0.92,3,1", 1);
#endif
        const auto settings = FootfallSettings::fromEnvironment();
        EXPECT_FLOAT_EQ(settings.mStair[0], 0.3f);
        EXPECT_FLOAT_EQ(settings.mStair[3], 0.92f);
        EXPECT_FLOAT_EQ(settings.mStair[5], 1.0f);
#ifdef _WIN32
        _putenv_s("OPENMW_RAGDOLL_FOOTFALL_STAIR", "");
#else
        unsetenv("OPENMW_RAGDOLL_FOOTFALL_STAIR");
#endif
        EXPECT_FLOAT_EQ(FootfallSettings::fromEnvironment().mStair[0], 0.0f);
    }

    TEST(RagdollCoreFootfall, settingsReadTheAdaptersKnobs)
    {
#ifdef _WIN32
        _putenv_s("OPENMW_RAGDOLL_FOOTFALL_GAIT", "0.4,1.2,0.9,70");
        _putenv_s("OPENMW_RAGDOLL_FOOTFALL_SETTLE", "-1");
#else
        setenv("OPENMW_RAGDOLL_FOOTFALL_GAIT", "0.4,1.2,0.9,70", 1);
        setenv("OPENMW_RAGDOLL_FOOTFALL_SETTLE", "-1", 1);
#endif
        const auto settings = FootfallSettings::fromEnvironment();
        EXPECT_FLOAT_EQ(settings.mGait[0], 0.4f);
        EXPECT_FLOAT_EQ(settings.mGait[3], 70.0f);
        // A negative single value keeps the default, like the adapter.
        EXPECT_FLOAT_EQ(settings.mSettle, 0.35f);
#ifdef _WIN32
        _putenv_s("OPENMW_RAGDOLL_FOOTFALL_GAIT", "");
        _putenv_s("OPENMW_RAGDOLL_FOOTFALL_SETTLE", "");
#else
        unsetenv("OPENMW_RAGDOLL_FOOTFALL_GAIT");
        unsetenv("OPENMW_RAGDOLL_FOOTFALL_SETTLE");
#endif
    }

    // WALK-02: one step of the rig whose body moves at `command + drift`
    // while the gait is asked for `command` (a body carried off its course).
    void stepDrift(Rig& rig, JPH::Vec3Arg command, JPH::Vec3Arg drift, JPH::Vec3& root)
    {
        root += (command + drift) * rig.input.mDt;
        RagdollPose drive = standingPose(root);
        const auto left = rig.physics.mBodyTransforms[LeftFoot].mTranslation;
        const auto right = rig.physics.mBodyTransforms[RightFoot].mTranslation;
        rig.physics = standingPose(root);
        rig.physics.mBodyTransforms[LeftFoot].mTranslation = left;
        rig.physics.mBodyTransforms[RightFoot].mTranslation = right;
        rig.clip = standingPose(root);
        rig.input.mCommandVelocity = command;
        FootfallOutput output;
        rig.controller.step(rig.input, drive, output);
        rig.physics.mBodyTransforms[LeftFoot].mTranslation = drive.mBodyTransforms[LeftFoot].mTranslation;
        rig.physics.mBodyTransforms[RightFoot].mTranslation = drive.mBodyTransforms[RightFoot].mTranslation;
    }

    TEST(RagdollCoreFootfall, captureTakesNoExtraStepOnASteadyWalk)
    {
        FootfallSettings settings;
        settings.mCapture = { 1.0f, 12.0f, 0.5f };
        Rig rig;
        rig.controller = FootfallController(settings);
        JPH::Vec3 root = JPH::Vec3::sZero();
        for (int i = 0; i < 360; ++i)
            rig.step(JPH::Vec3(0.0f, i < 30 ? 0.0f : (i < 180 ? 60.0f : 150.0f), 0.0f), root);
        EXPECT_EQ(rig.controller.captureSteps(), 0u);
    }

    TEST(RagdollCoreFootfall, captureStepsUnderABodyCarriedSideways)
    {
        // Walking +y at 60; for 0.6 s the body is carried +x at 150 u/s.
        // Landings made in and just after it, measured across from the
        // pelvis toward the drift.
        std::array<float, 2> across{};
        std::array<unsigned, 2> captureSteps{};
        for (const bool on : { false, true })
        {
            FootfallSettings settings;
            if (on)
                settings.mCapture = { 1.0f, 12.0f, 0.5f };
            Rig rig;
            rig.controller = FootfallController(settings);
            JPH::Vec3 root = JPH::Vec3::sZero();
            std::array<bool, 2> was{ false, false };
            float sum = 0.0f;
            int count = 0;
            for (int i = 0; i < 240; ++i)
            {
                const bool drifting = i >= 90 && i < 126;
                stepDrift(rig, JPH::Vec3(0.0f, i < 30 ? 0.0f : 60.0f, 0.0f),
                    JPH::Vec3(drifting ? 150.0f : 0.0f, 0.0f, 0.0f), root);
                for (std::size_t side = 0; side < 2; ++side)
                {
                    const auto& foot = rig.controller.feet()[side];
                    if (foot.mSwinging && !was[side] && i >= 90 && i < 160)
                    {
                        sum += static_cast<float>(foot.mTarget.GetX()) - root.GetX();
                        ++count;
                    }
                    was[side] = foot.mSwinging;
                }
            }
            ASSERT_GT(count, 0) << on;
            across[on ? 1 : 0] = sum / static_cast<float>(count);
            captureSteps[on ? 1 : 0] = rig.controller.captureSteps();
        }
        EXPECT_EQ(captureSteps[0], 0u);
        EXPECT_GE(captureSteps[1], 1u);
        // The landings go with the drift (off: the 0.12 s x 0.3 sideways lead).
        EXPECT_GT(across[1], across[0] + 8.0f) << across[0] << " " << across[1];
    }

    TEST(RagdollCoreFootfall, slopeTiltsThePlantedSoleOntoTheGround)
    {
        // A plane rising 0.5 per unit along +y (26.6 deg).
        const JPH::Vec3 normal = JPH::Vec3(0.0f, -0.5f, 1.0f).Normalized();
        for (const bool on : { false, true })
        {
            FootfallSettings settings;
            if (on)
                settings.mSlope = { 1.0f, 30.0f, 2.0f };
            Rig rig;
            rig.controller = FootfallController(settings);
            rig.input.mGround = [](const JPH::RVec3& p, double) -> std::optional<double> { return 0.5 * p.GetY(); };
            rig.input.mGroundNormal = [normal](const JPH::RVec3&, double) -> std::optional<JPH::Vec3> { return normal; };
            JPH::Vec3 root = JPH::Vec3::sZero();
            RagdollPose drive;
            for (int i = 0; i < 60; ++i)
            {
                drive = standingPose(root);
                rig.input.mCommandVelocity = JPH::Vec3::sZero();
                FootfallOutput output;
                rig.controller.step(rig.input, drive, output);
            }
            const JPH::Vec3 soleUp = drive.mBodyTransforms[LeftFoot].mRotation * JPH::Vec3::sAxisZ();
            if (on)
            {
                EXPECT_GT(soleUp.Dot(normal), std::cos(1.0f * 3.14159265f / 180.0f));
                // The centre rides higher for the tilt: 6.5 / cos(26.6 deg).
                EXPECT_NEAR(rig.controller.feet()[0].mPin.GetZ(), 6.5 / normal.GetZ(), 0.05);
            }
            else
            {
                EXPECT_GT(soleUp.GetZ(), 0.9999f);
                EXPECT_NEAR(rig.controller.feet()[0].mPin.GetZ(), 6.5, 0.05);
            }
        }
    }

    TEST(RagdollCoreFootfall, slopeOffWithoutANormalProbe)
    {
        // mSlope on but no mGroundNormal: the old level feet, bit for bit.
        std::array<std::vector<RagdollPose>, 2> drives;
        for (const bool on : { false, true })
        {
            FootfallSettings settings;
            if (on)
                settings.mSlope = { 1.0f, 30.0f, 2.0f };
            Rig rig;
            rig.controller = FootfallController(settings);
            rig.input.mGround = [](const JPH::RVec3& p, double) -> std::optional<double> { return 0.3 * p.GetY(); };
            JPH::Vec3 root = JPH::Vec3::sZero();
            for (int i = 0; i < 120; ++i)
            {
                root.SetZ(0.3f * root.GetY());
                rig.input.mActorZ = root.GetZ();
                rig.step(JPH::Vec3(0.0f, i < 20 ? 0.0f : 60.0f, 0.0f), root, &drives[on ? 1 : 0]);
            }
        }
        ASSERT_EQ(drives[0].size(), drives[1].size());
        for (std::size_t i = 0; i < drives[0].size(); ++i)
            for (std::size_t b = 0; b < Count; ++b)
            {
                EXPECT_EQ(drives[0][i].mBodyTransforms[b].mTranslation, drives[1][i].mBodyTransforms[b].mTranslation);
                EXPECT_EQ(drives[0][i].mBodyTransforms[b].mRotation, drives[1][i].mBodyTransforms[b].mRotation);
            }
    }

    TEST(RagdollCoreFootfall, settingsReadCaptureAndSlope)
    {
#ifdef _WIN32
        _putenv_s("OPENMW_RAGDOLL_FOOTFALL_CAPTURE", "1,10,0.4");
        _putenv_s("OPENMW_RAGDOLL_FOOTFALL_SLOPE", "0.7,25,2");
#else
        setenv("OPENMW_RAGDOLL_FOOTFALL_CAPTURE", "1,10,0.4", 1);
        setenv("OPENMW_RAGDOLL_FOOTFALL_SLOPE", "0.7,25,2", 1);
#endif
        const auto settings = FootfallSettings::fromEnvironment();
        EXPECT_FLOAT_EQ(settings.mCapture[0], 1.0f);
        EXPECT_FLOAT_EQ(settings.mCapture[1], 10.0f);
        EXPECT_FLOAT_EQ(settings.mCapture[2], 0.4f);
        EXPECT_FLOAT_EQ(settings.mSlope[0], 0.7f);
        EXPECT_FLOAT_EQ(settings.mSlope[1], 25.0f);
#ifdef _WIN32
        _putenv_s("OPENMW_RAGDOLL_FOOTFALL_CAPTURE", "");
        _putenv_s("OPENMW_RAGDOLL_FOOTFALL_SLOPE", "");
#else
        unsetenv("OPENMW_RAGDOLL_FOOTFALL_CAPTURE");
        unsetenv("OPENMW_RAGDOLL_FOOTFALL_SLOPE");
#endif
        EXPECT_FLOAT_EQ(FootfallSettings::fromEnvironment().mCapture[0], 0.0f);
        EXPECT_FLOAT_EQ(FootfallSettings::fromEnvironment().mSlope[0], 0.0f);
    }

    // STEPPING audit items 6, 9, 11: the landing trace, the phase lock, the stop lead.
    std::vector<float> liftAfterLanding(float phaseLock)
    {
        FootfallSettings settings;
        settings.mPhaseLock = phaseLock;
        Rig rig;
        rig.controller = FootfallController(settings);
        JPH::Vec3 root = JPH::Vec3::sZero();
        for (int i = 0; i < 30; ++i)
            rig.step(JPH::Vec3::sZero(), root);
        std::vector<float> gaps;
        std::array<bool, 2> was{ false, false };
        int landedAt = -1;
        for (int i = 0; i < 360; ++i)
        {
            const auto output = rig.step(JPH::Vec3(0.0f, 90.0f, 0.0f), root);
            for (std::size_t side = 0; side < 2; ++side)
            {
                if (was[side] && !output.mSwinging[side])
                    landedAt = i;
                else if (!was[side] && output.mSwinging[side] && landedAt >= 0 && i > 120)
                    gaps.push_back(static_cast<float>(i - landedAt) * rig.input.mDt);
                was[side] = output.mSwinging[side];
            }
        }
        return gaps;
    }

    TEST(RagdollCoreFootfall, thePhaseLockTimesEachLiftFromTheLastLanding)
    {
        const auto locked = liftAfterLanding(1.0f);
        ASSERT_GE(locked.size(), 6u);
        float low = 1e9f, high = -1e9f;
        for (const float g : locked)
        {
            low = std::min(low, g);
            high = std::max(high, g);
        }
        // Every lift comes the same stance time after the other foot's landing.
        EXPECT_LT(high - low, 0.05f);
    }

    TEST(RagdollCoreFootfall, aStopLandsBesideWhereThePelvisComesToRest)
    {
        Rig rig;
        JPH::Vec3 root = JPH::Vec3::sZero();
        for (int i = 0; i < 30; ++i)
            rig.step(JPH::Vec3::sZero(), root);
        for (int i = 0; i < 90; ++i)
            rig.step(JPH::Vec3(0.0f, 150.0f, 0.0f), root);
        // Keys released: the capsule still moves for a moment, the stop is asked.
        rig.input.mStopping = true;
        bool checked = false;
        std::array<bool, 2> was{ rig.controller.feet()[0].mSwinging, rig.controller.feet()[1].mSwinging };
        for (int i = 0; i < 30 && !checked; ++i)
        {
            const float speed = std::max(0.0f, 150.0f * (1.0f - static_cast<float>(i) / 10.0f));
            const auto output = rig.step(JPH::Vec3(0.0f, speed, 0.0f), root);
            for (std::size_t side = 0; side < 2; ++side)
            {
                if (output.mSwinging[side] && !was[side])
                {
                    // Not a run's landing a stride ahead: beside the pelvis.
                    EXPECT_LT(std::abs(rig.controller.feet()[side].mTarget.GetY() - root.GetY()), 12.0f) << i;
                    checked = true;
                }
                was[side] = output.mSwinging[side];
            }
        }
        EXPECT_TRUE(checked);
    }

    TEST(RagdollCoreFootfall, everyLandingIsTracedWithItsMiss)
    {
        Rig rig;
        std::vector<std::string> lines;
        rig.input.mTrace = [&](const std::string& line) { lines.push_back(line); };
        JPH::Vec3 root = JPH::Vec3::sZero();
        for (int i = 0; i < 30; ++i)
            rig.step(JPH::Vec3::sZero(), root);
        for (int i = 0; i < 120; ++i)
            rig.step(JPH::Vec3(0.0f, 90.0f, 0.0f), root);
        int lands = 0;
        for (const auto& line : lines)
            if (line.rfind("RAGDOLL_FOOTFALL_LAND side=", 0) == 0)
            {
                ++lands;
                EXPECT_NE(line.find(" miss="), std::string::npos);
            }
        EXPECT_GE(lands, 3);
    }
}
