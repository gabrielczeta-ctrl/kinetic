#include "getuppolicy.hpp"

#include <ragdollcore/stridesim.hpp>
#include <ragdollcore/unitmodel.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace
{
    using namespace RagdollCore;

    // Median swing clearance from Footfall's per-step trace: the highest the
    // foot rose above the straight line from its pin to its target, per swing
    // (tools/body/walk_traces.py's measure).
    // goal: the swing goal Footfall solved the leg for instead of the foot.
    float medianClearance(const std::vector<std::string>& lines, bool goal = false)
    {
        std::vector<float> swings;
        std::array<float, 2> best{ -1e9f, -1e9f };
        std::array<bool, 2> open{ false, false };
        for (const std::string& line : lines)
        {
            if (line.rfind("RAGDOLL_FOOTFALL side=", 0) != 0)
                continue;
            int side = 0, swinging = 0;
            float phase = 0, fx = 0, fy = 0, fz = 0, px = 0, py = 0, pz = 0, tx = 0, ty = 0, tz = 0, gx = 0, gy = 0,
                  gz = 0;
            if (std::sscanf(line.c_str(),
                    "RAGDOLL_FOOTFALL side=%d swing=%d phase=%f foot=%f,%f,%f pin=%f,%f,%f target=%f,%f,%f "
                    "goal=%f,%f,%f",
                    &side, &swinging, &phase, &fx, &fy, &fz, &px, &py, &pz, &tx, &ty, &tz, &gx, &gy, &gz)
                != 15 || side < 0 || side > 1)
                continue;
            if (goal)
                fz = gz;
            if (swinging)
            {
                const float t = std::clamp(phase, 0.0f, 1.0f);
                best[side] = std::max(best[side], fz - (pz + (tz - pz) * t));
                open[side] = true;
            }
            else if (open[side])
            {
                swings.push_back(best[side]);
                best[side] = -1e9f;
                open[side] = false;
            }
        }
        if (swings.empty())
            return -1.0f;
        std::sort(swings.begin(), swings.end());
        return swings[swings.size() / 2];
    }

    TEST(RagdollCoreStrideSim, aSoftRootBodyStandsOnItsFeet)
    {
        StrideSim::Settings settings;
        settings.mActiveBodyPath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/active-body.cfg";
        settings.mGaitTablePath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/footfall-gaits.cfg";
        StrideSim sim(settings);
        ASSERT_TRUE(sim.valid()) << sim.error();
        for (int i = 0; i < 120; ++i)
            sim.step();
        const float pelvis = static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ());
        EXPECT_GT(pelvis, 0.9f * sim.standingPelvisHeight());
        EXPECT_NE(sim.footfall(), nullptr);
    }

    // Feasibility probe for the get-up trainer (run by name): how high the
    // joint motors alone lift a lying body toward the standing pose.
    TEST(RagdollCoreStrideSim, DISABLED_motorsAloneTowardStanding)
    {
        for (const float assist : { 0.0f, 0.3f, 0.6f, 1.0f })
            for (int start = 0; start < 2; ++start)
            {
                StrideSim::Settings settings;
                settings.mLyingRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), start == 0 ? -1.5708f : 1.5708f);
                settings.mLimpAssist = true;
                settings.mActiveBodyPath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/active-body.cfg";
                StrideSim sim(settings);
                ASSERT_TRUE(sim.valid());
                sim.setLimpTarget(sim.basePose(), 1.0f);
                float best = 0.0f;
                for (int i = 0; i < 300; ++i)
                {
                    const JPH::RVec3 at = sim.bodyPosition(sim.pelvis());
                    sim.setAssist(JPH::RVec3(at.GetX(), at.GetY(), sim.standingPelvisHeight()), JPH::Quat::sIdentity(),
                        assist);
                    sim.step();
                    best = std::max(best, static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ()));
                }
                std::printf("assist %.1f start %d: pelvis best %.1f end %.1f head %.1f (standing pelvis %.1f)\n",
                    assist, start, best, static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ()),
                    static_cast<float>(sim.bodyPosition(sim.head()).GetZ()), sim.standingPelvisHeight());
            }
    }

    // MOVE-01 S3 offline (run by name): the walk and the run with the step
    // planner aiming Footfall's landings, against Footfall alone.
    TEST(RagdollCoreStrideSim, DISABLED_plannerAgainstFootfallAlone)
    {
        // Variants: Footfall alone; the planner aiming landings; the planner
        // with event timing (Footfall's PHASE_LOCK: lifts follow touchdowns,
        // CONTACT_LAND: a swing lands on contact).
        // 3 and 4: Footfall alone and the planner with event timing, both
        // with the standing joint support on the leg motors; 5 the planner
        // with the support but Footfall's clock; 6 as 4 with a 50-unit reach.
        for (const int variant : { 0, 1, 2, 3, 4, 5, 6 })
            for (const float speed : { 127.0f, 230.0f })
            {
                const bool planner = variant == 1 || variant == 2 || variant >= 4;
                std::vector<std::string> lines;
                std::vector<std::size_t> liftSteps;
                StrideSim::Settings settings;
                settings.mActiveBodyPath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/active-body.cfg";
                settings.mGaitTablePath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/footfall-gaits.cfg";
                StrideSim* simPointer = nullptr;
                settings.mTrace = [&](const std::string& line) {
                    lines.push_back(line);
                    if (line.find("RAGDOLL_FOOTFALL_LIFT") != std::string::npos && simPointer)
                        liftSteps.push_back(simPointer->steps());
                };
                settings.mPlanner = planner;
                if (variant == 2 || variant == 4 || variant == 6)
                {
                    settings.mFootfall.mPhaseLock = 1.0f;
                    settings.mFootfall.mContactLand = { 0.5f, 0.2f, 0.0f };
                }
                settings.mJointSupport = variant >= 3;
                settings.mPlannerLimits.mMaximumReach = variant == 6 ? 50.0f : 70.0f;
                settings.mPlannerLimits.mMinimumWidth = 4.0f;
                settings.mPlannerLimits.mStanceWidth = 12.0f;
                StrideSim sim(settings);
                ASSERT_TRUE(sim.valid());
                simPointer = &sim;
                for (int i = 0; i < 120; ++i)
                    sim.step();
                sim.setRequestVelocity(JPH::Vec3(0.0f, speed, 0.0f));
                liftSteps.clear();
                float lowest = 1e9f;
                std::vector<float> lags;
                for (int i = 0; i < 480; ++i)
                {
                    sim.step();
                    lowest = std::min(lowest, static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ()));
                    const JPH::RVec3 gap = sim.bodyPosition(sim.pelvis()) - sim.capsulePosition();
                    if (i >= 60)
                        lags.push_back(static_cast<float>(std::hypot(gap.GetX(), gap.GetY())));
                }
                std::sort(lags.begin(), lags.end());
                std::vector<float> misses;
                int clock = 0, lifts = 0;
                for (const std::string& line : lines)
                {
                    if (const auto at = line.find("RAGDOLL_FOOTFALL_LAND"); at != std::string::npos)
                        misses.push_back(std::stof(line.substr(line.find("miss=") + 5)));
                    if (line.find("RAGDOLL_FOOTFALL_LIFT") != std::string::npos)
                    {
                        ++lifts;
                        clock += line.find("crossed=1 overrun=0") != std::string::npos ? 1 : 0;
                    }
                }
                std::sort(misses.begin(), misses.end());
                // Step time variation: the time between lifts after the first second.
                std::vector<float> gaps;
                for (std::size_t i = 1; i < liftSteps.size(); ++i)
                    if (liftSteps[i - 1] > 180)
                        gaps.push_back(static_cast<float>(liftSteps[i] - liftSteps[i - 1]) / 60.0f);
                float mean = 0.0f, variance = 0.0f;
                for (float g : gaps)
                    mean += g / std::max<std::size_t>(gaps.size(), 1);
                for (float g : gaps)
                    variance += (g - mean) * (g - mean) / std::max<std::size_t>(gaps.size(), 1);
                std::printf("variant=%d speed=%.0f landings=%zu miss p50=%.1f lifts=%d on clock=%d step cv=%.2f "
                            "lowest pelvis=%.1f%s lag p50=%.1f p95=%.1f recoveries=%zu giveups=%zu\n",
                    variant, speed, misses.size(), misses.empty() ? -1.0f : misses[misses.size() / 2], lifts, clock,
                    mean > 0.0f ? std::sqrt(variance) / mean : -1.0f, lowest,
                    lowest < 0.5f * sim.standingPelvisHeight() ? " FELL" : "", lags[lags.size() / 2],
                    lags[lags.size() * 95 / 100], sim.recoveryPlans(), sim.giveUps());
            }
    }

    // MOVE-01 S3 offline tuning (run by name): the planner's weights at a
    // walk on the supported body with event timing; steps, their length,
    // the lowest pelvis.
    TEST(RagdollCoreStrideSim, DISABLED_plannerWalkWeights)
    {
        struct Weights
        {
            float mOffset, mTime, mLag;
        };
        // The last field: CONTACT_LAND's travel share (0 = by phase alone).
        struct Run
        {
            Weights mWeights;
            float mTravelShare;
        };
        for (const Run run : { Run{ { 20.0f, 400.0f, 0.02f }, 0.0f }, Run{ { 5.0f, 400.0f, 0.02f }, 0.0f },
                 Run{ { 1.0f, 400.0f, 0.02f }, 0.0f }, Run{ { 5.0f, 50.0f, 0.02f }, 0.0f },
                 Run{ { 5.0f, 2000.0f, 0.02f }, 0.0f }, Run{ { 1.0f, 2000.0f, 0.0f }, 0.0f },
                 Run{ { 0.2f, 2000.0f, 0.0f }, 0.0f }, Run{ { 20.0f, 400.0f, 0.02f }, 0.6f },
                 Run{ { 20.0f, 400.0f, 0.02f }, 0.8f }, Run{ { 20.0f, 400.0f, 0.02f }, 0.9f } })
        {
            const Weights& w = run.mWeights;
            int lifts = 0;
            StrideSim::Settings settings;
            settings.mActiveBodyPath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/active-body.cfg";
            settings.mGaitTablePath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/footfall-gaits.cfg";
            settings.mTrace = [&](const std::string& line) {
                lifts += line.find("RAGDOLL_FOOTFALL_LIFT") != std::string::npos ? 1 : 0;
            };
            settings.mPlanner = true;
            settings.mJointSupport = true;
            settings.mFootfall.mPhaseLock = 1.0f;
            settings.mFootfall.mContactLand = { 0.5f, 0.2f, run.mTravelShare };
            settings.mPlannerLimits.mMaximumReach = 70.0f;
            settings.mPlannerLimits.mMinimumWidth = 4.0f;
            settings.mPlannerLimits.mOffsetWeight = w.mOffset;
            settings.mPlannerLimits.mTimeWeight = w.mTime;
            settings.mPlannerLimits.mLagWeight = w.mLag;
            StrideSim sim(settings);
            ASSERT_TRUE(sim.valid());
            for (int i = 0; i < 120; ++i)
                sim.step();
            sim.setRequestVelocity(JPH::Vec3(0.0f, 127.0f, 0.0f));
            lifts = 0;
            float lowest = 1e9f;
            for (int i = 0; i < 480; ++i)
            {
                sim.step();
                lowest = std::min(lowest, static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ()));
            }
            std::printf("offset=%.1f time=%.0f lag=%.2f travel=%.1f: lifts=%d step=%.0f u lowest pelvis=%.1f "
                        "catches=%zu\n",
                w.mOffset, w.mTime, w.mLag, run.mTravelShare, lifts, lifts > 0 ? 127.0f * 8.0f / lifts : 0.0f, lowest,
                sim.recoveryPlans());
        }
    }

    // MOVE-01 S3 offline (run by name): strafes, backpedal and a stop, Footfall
    // alone against the planner with event timing, both on the supported body.
    TEST(RagdollCoreStrideSim, DISABLED_plannerMoves)
    {
        struct Move
        {
            const char* mName;
            JPH::Vec3 mVelocity;
            float mSeconds;
        };
        const Move moves[] = { { "walk", JPH::Vec3(0, 127, 0), 5.0f }, { "strafe right", JPH::Vec3(120, 0, 0), 5.0f },
            { "strafe left", JPH::Vec3(-120, 0, 0), 5.0f },
            { "backpedal", JPH::Vec3(0, -100, 0), 5.0f }, { "diagonal", JPH::Vec3(85, 85, 0), 5.0f } };
        // 0: Footfall alone; 1-3: the planner aiming until half way, a tenth
        // of the swing, the lift step only.
        // 4: the planner alone (Footfall's clock); 5: event timing alone.
        for (const int variant : { 0, 1, 2, 3, 4, 5 })
            for (const Move& move : moves)
            {
                const bool planner = variant > 0 && variant < 5;
                int lifts = 0;
                std::vector<std::string> lines;
                StrideSim::Settings settings;
                settings.mActiveBodyPath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/active-body.cfg";
                settings.mGaitTablePath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/footfall-gaits.cfg";
                settings.mTrace = [&](const std::string& line) {
                    lifts += line.find("RAGDOLL_FOOTFALL_LIFT") != std::string::npos ? 1 : 0;
                    lines.push_back(line);
                };
                settings.mJointSupport = true;
                settings.mPlanner = planner;
                settings.mPlanUntil = variant == 2 ? 0.1f : variant == 3 ? 0.0f : 0.5f;
                if ((planner && variant != 4) || variant == 5)
                {
                    settings.mFootfall.mPhaseLock = 1.0f;
                    settings.mFootfall.mContactLand = { 0.5f, 0.2f, 0.8f };
                }
                settings.mPlannerLimits.mMaximumReach = 70.0f;
                settings.mPlannerLimits.mMinimumWidth = 4.0f;
                StrideSim sim(settings);
                ASSERT_TRUE(sim.valid());
                sim.setFacing(0.0f); // facing +y throughout: true strafes and backpedals
                for (int i = 0; i < 120; ++i)
                    sim.step();
                sim.setRequestVelocity(move.mVelocity);
                lifts = 0;
                float lowest = 1e9f, lag = 0.0f;
                int crossed = 0, frames = 0;
                const int steps = static_cast<int>(move.mSeconds * 60.0f);
                for (int i = 0; i < steps + 120; ++i)
                {
                    if (i == steps)
                        sim.setRequestVelocity(JPH::Vec3::sZero()); // then stop for 2 s
                    sim.step();
                    lowest = std::min(lowest, static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ()));
                    const JPH::RVec3 gap = sim.bodyPosition(sim.pelvis()) - sim.capsulePosition();
                    lag = std::max(lag, static_cast<float>(std::hypot(gap.GetX(), gap.GetY())));
                    // Crossed: the left foot right of the right one across the pelvis.
                    const JPH::Vec3 side = sim.bodyRotation(sim.pelvis()) * JPH::Vec3::sAxisX();
                    const JPH::Vec3 apart(sim.bodyPosition(sim.feet()[0]) - sim.bodyPosition(sim.feet()[1]));
                    ++frames;
                    crossed += apart.GetX() * side.GetX() + apart.GetY() * side.GetY() > 0.0f ? 1 : 0;
                }
                std::printf("variant=%d %-12s lifts=%d clearance p50=%.1f (goal %.1f) lowest pelvis=%.1f%s worst "
                            "lag=%.1f crossed=%.0f%%\n",
                    variant, move.mName, lifts, medianClearance(lines), medianClearance(lines, true), lowest,
                    lowest < 0.5f * sim.standingPelvisHeight() ? " FELL" : "", lag, 100.0f * crossed / std::max(frames, 1));
            }
    }

    // MOVE-01 S1 (run by name): the walk over the prison ship's hold floor
    // (its collision: a slab 0.6 u thick of 481 x 600 u triangles) against a
    // box floor, with Jolt's metre tolerances and with the game's, from three
    // starts across the hold. Per run: planted-foot frames with the foot's
    // centre under 4 u above the top and without a ground contact, the
    // pelvis's sag and the swing clearance.
    TEST(RagdollCoreStrideSim, DISABLED_holdSlab)
    {
        for (const bool game : { false, true })
            for (const auto floor : { StrideSim::Settings::Floor::Box, StrideSim::Settings::Floor::HoldSlab })
                for (const float startX : { 61.0f, 0.0f, -45.0f })
                {
                    std::vector<std::string> lines;
                    StrideSim::Settings settings;
                    settings.mActiveBodyPath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/active-body.cfg";
                    settings.mGaitTablePath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/footfall-gaits.cfg";
                    settings.mTrace = [&](const std::string& line) { lines.push_back(line); };
                    settings.mJointSupport = true;
                    settings.mGameContacts = game;
                    settings.mFloor = floor;
                    settings.mSlabStart = { startX, -135.0f };
                    StrideSim sim(settings);
                    ASSERT_TRUE(sim.valid());
                    sim.setFacing(0.0f);
                    for (int i = 0; i < 120; ++i)
                        sim.step();
                    sim.setRequestVelocity(JPH::Vec3(0, 127, 0));
                    lines.clear();
                    int planted = 0, deep = 0, unsupported = 0;
                    std::vector<float> footHeights, sags;
                    float lowestFoot = 1e9f;
                    for (int i = 0; i < 9 * 60; ++i)
                    {
                        sim.step();
                        sags.push_back(sim.standingPelvisHeight() - static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ()));
                        if (!sim.footfall())
                            continue;
                        for (std::size_t side = 0; side < 2; ++side)
                        {
                            if (sim.footfall()->feet()[side].mSwinging)
                                continue;
                            const std::size_t foot = sim.feet()[side];
                            const float z = static_cast<float>(sim.bodyPosition(foot).GetZ());
                            ++planted;
                            footHeights.push_back(z);
                            lowestFoot = std::min(lowestFoot, z);
                            deep += z < 4.0f ? 1 : 0;
                            const auto& contacts = sim.groundContacts();
                            unsupported += std::none_of(contacts.begin(), contacts.end(),
                                               [&](const GroundContact& c) { return c.mBodyIndex == foot; })
                                ? 1
                                : 0;
                        }
                    }
                    std::sort(footHeights.begin(), footHeights.end());
                    std::sort(sags.begin(), sags.end());
                    const auto at = [](const std::vector<float>& v, float q) {
                        return v.empty() ? 0.0f : v[static_cast<std::size_t>(q * (v.size() - 1))];
                    };
                    std::printf("%s %-5s x=%4.0f planted=%d foot p05=%.1f p50=%.1f lowest=%.1f deep=%.0f%% "
                                "unsupported=%.0f%% sag p50=%.1f p95=%.1f clearance=%.1f end y=%.0f\n",
                        game ? "game  " : "metres", floor == StrideSim::Settings::Floor::Box ? "box" : "slab", startX,
                        planted, at(footHeights, 0.05f), at(footHeights, 0.5f), lowestFoot,
                        100.0f * deep / std::max(planted, 1), 100.0f * unsupported / std::max(planted, 1),
                        at(sags, 0.5f), at(sags, 0.95f), medianClearance(lines),
                        static_cast<float>(sim.bodyPosition(sim.pelvis()).GetY()));
                }
    }

    // MOVE-01 S4 offline (run by name): the capsule leading against the legs
    // leading (the stance force), on the supported body: a start from rest,
    // 4 s of walking, a reversal, a stop.
    TEST(RagdollCoreStrideSim, DISABLED_legsLead)
    {
        struct Variant
        {
            const char* mName;
            bool mLegsLead;
            float mFriction;
            float mDamping;
            bool mIntent;
            bool mSupport = false;
            // The stance push (mLegsLeadPush), the planner with event timing,
            // the root's horizontal share.
            bool mPush = false;
            bool mPlanner = false;
            float mRootShare = 0.1f;
            // The reference on a leash (mLegsLeadLeash).
            float mLeash = 0.0f;
            bool mJointSupport = true;
            bool mAim = false; // mLegsLeadAimSwing
            bool mTransfer = false;
        };
        for (const Variant variant : { Variant{ "capsule leads", false, 0.0f, 0.0f, false },
                 Variant{ "legs 0.35 / 6", true, 0.35f, 6.0f, false }, Variant{ "legs 0.5 / 10", true, 0.5f, 10.0f, false },
                 Variant{ "intent 0.35/6", true, 0.35f, 6.0f, true }, Variant{ "intent 0.5/10", true, 0.5f, 10.0f, true },
                 Variant{ "intent 0.7/14", true, 0.7f, 14.0f, true },
                 Variant{ "via support", true, 0.0f, 0.0f, true, true },
                 Variant{ "push+planner", true, 0.0f, 0.0f, true, true, true, true },
                 Variant{ "leash 20 r.1", true, 0.0f, 0.0f, true, false, false, false, 0.1f, 20.0f },
                 Variant{ "leash 20 r0", true, 0.0f, 0.0f, true, false, false, false, 0.0f, 20.0f },
                 Variant{ "leash 40 r.1", true, 0.0f, 0.0f, true, false, false, false, 0.1f, 40.0f },
                 Variant{ "leash 20 push", true, 0.0f, 0.0f, true, true, true, false, 0.1f, 20.0f },
                 Variant{ "leash 20 plan", true, 0.0f, 0.0f, true, false, false, true, 0.1f, 20.0f },
                 Variant{ "leash 20 p+p", true, 0.0f, 0.0f, true, true, true, true, 0.1f, 20.0f },
                 Variant{ "r0 push", true, 0.0f, 0.0f, true, true, true, false, 0.0f, 20.0f },
                 Variant{ "r0 p+p", true, 0.0f, 0.0f, true, true, true, true, 0.0f, 20.0f },
                 Variant{ "r0 no support", true, 0.0f, 0.0f, true, false, false, false, 0.0f, 20.0f, false },
                 Variant{ "r0 l60 no sup", true, 0.0f, 0.0f, true, false, false, false, 0.0f, 60.0f, false },
                 Variant{ "r0 aim l10", true, 0.0f, 0.0f, true, true, true, false, 0.0f, 10.0f, true, true },
                 Variant{ "r0 aim l20", true, 0.0f, 0.0f, true, true, true, false, 0.0f, 20.0f, true, true },
                 Variant{ "r0 aim l30", true, 0.0f, 0.0f, true, true, true, false, 0.0f, 30.0f, true, true },
                 Variant{ "r0 aim l45", true, 0.0f, 0.0f, true, true, true, false, 0.0f, 45.0f, true, true },
                 Variant{ "r.05 aim l20", true, 0.0f, 0.0f, true, true, true, false, 0.05f, 20.0f, true, true },
                 Variant{ "r0 transfer", true, 0.0f, 0.0f, true, true, true, false, 0.0f, 20.0f, true, true, true },
                 Variant{ "r0 transf l40", true, 0.0f, 0.0f, true, true, true, false, 0.0f, 40.0f, true, true, true },
                 Variant{ "r.05 transfer", true, 0.0f, 0.0f, true, true, true, false, 0.05f, 20.0f, true, true, true } })
            for (const float speed : { 127.0f, 230.0f })
            {
                std::vector<std::string> lines;
                StrideSim::Settings settings;
                settings.mActiveBodyPath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/active-body.cfg";
                settings.mGaitTablePath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/footfall-gaits.cfg";
                settings.mTrace = [&](const std::string& line) { lines.push_back(line); };
                settings.mJointSupport = variant.mJointSupport;
                settings.mLegsLead = variant.mLegsLead;
                settings.mLegsLeadStepForIntent = variant.mIntent;
                settings.mLegsLeadThroughSupport = variant.mSupport;
                settings.mStanceForce.mFrictionShare = variant.mFriction;
                settings.mStanceForce.mDamping = variant.mDamping;
                settings.mLegsLeadPush = variant.mPush;
                settings.mLegsLeadRootShare = variant.mRootShare;
                settings.mLegsLeadLeash = variant.mLeash;
                settings.mLegsLeadAimSwing = variant.mAim;
                settings.mTransfer = variant.mTransfer;
                settings.mPlanner = variant.mPlanner;
                if (variant.mPlanner)
                {
                    settings.mFootfall.mPhaseLock = 1.0f;
                    settings.mFootfall.mContactLand = { 0.5f, 0.2f, 0.8f };
                    settings.mPlannerLimits.mMaximumReach = 70.0f;
                    settings.mPlannerLimits.mMinimumWidth = 4.0f;
                }
                StrideSim sim(settings);
                ASSERT_TRUE(sim.valid());
                sim.setFacing(0.0f);
                for (int i = 0; i < 120; ++i)
                    sim.step();
                const auto along = [&](float sign) {
                    return sign * static_cast<float>(sim.bodyVelocity(sim.pelvis()).GetY());
                };
                float reach80 = -1.0f, lowest = 1e9f, reversal = -1.0f;
                std::vector<float> cruise;
                // The root's horizontal force against the weight while cruising.
                float assist = 0.0f;
                sim.setRequestVelocity(JPH::Vec3(0.0f, speed, 0.0f));
                for (int i = 0; i < 240; ++i)
                {
                    sim.step();
                    lowest = std::min(lowest, static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ()));
                    if (reach80 < 0.0f && along(1.0f) >= 0.8f * speed)
                        reach80 = (i + 1) / 60.0f;
                    if (i >= 120)
                    {
                        cruise.push_back(along(1.0f));
                        const JPH::Vec3 root = sim.rootForce();
                        assist += std::hypot(root.GetX(), root.GetY())
                            / (settings.mTotalMass * UnitModel::GravityUnitsPerSecondSquared) / 120.0f;
                    }
                }
                sim.setRequestVelocity(JPH::Vec3(0.0f, -speed, 0.0f));
                for (int i = 0; i < 120; ++i)
                {
                    sim.step();
                    lowest = std::min(lowest, static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ()));
                    if (reversal < 0.0f && along(-1.0f) >= 0.5f * speed)
                        reversal = (i + 1) / 60.0f;
                }
                sim.setRequestVelocity(JPH::Vec3::sZero());
                const double stopAt = sim.bodyPosition(sim.pelvis()).GetY();
                double furthest = stopAt;
                for (int i = 0; i < 120; ++i)
                {
                    sim.step();
                    lowest = std::min(lowest, static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ()));
                    furthest = std::min(furthest, sim.bodyPosition(sim.pelvis()).GetY());
                }
                float mean = 0.0f;
                for (float v : cruise)
                    mean += v / std::max<std::size_t>(cruise.size(), 1);
                int lifts = 0;
                for (const std::string& line : lines)
                    lifts += line.find("RAGDOLL_FOOTFALL_LIFT") != std::string::npos ? 1 : 0;
                std::printf("%-14s speed=%.0f to 80%%=%.2fs cruise=%.0f assist=%.3f reversal to half=%.2fs stop "
                            "travel=%.1f lowest pelvis=%.1f%s lifts=%d (unloaded %zu, timed out %zu) clearance=%.1f\n",
                    variant.mName, speed, reach80, mean, assist, reversal, static_cast<float>(stopAt - furthest),
                    lowest, lowest < 0.5f * sim.standingPelvisHeight() ? " FELL" : "", lifts, sim.transferLifts(),
                    sim.transferTimeouts(), medianClearance(lines));
            }
    }

    // MOVE-01 S4 offline (run by name): one legs-leading walk (the reference
    // on a 20 u leash, the stance push through the joint support) with the
    // root's horizontal hold at 0 (RAGDOLL_TRACE_ROOT_SHARE sets it), traced
    // every tenth of a second: the pelvis, the feet, the capsule along the
    // travel, each foot's swing state and target, the root's force.
    TEST(RagdollCoreStrideSim, DISABLED_legsLeadTrace)
    {
        StrideSim::Settings settings;
        settings.mActiveBodyPath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/active-body.cfg";
        settings.mGaitTablePath = std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/footfall-gaits.cfg";
        settings.mLegsLead = true;
        settings.mLegsLeadStepForIntent = true;
        settings.mLegsLeadRootShare = 0.0f;
        settings.mLegsLeadLeash = 20.0f;
        settings.mJointSupport = true;
        settings.mLegsLeadThroughSupport = true;
        settings.mLegsLeadPush = true;
        settings.mLegsLeadAimSwing = std::getenv("RAGDOLL_TRACE_AIM") != nullptr;
        settings.mTransfer = std::getenv("RAGDOLL_TRACE_TRANSFER") != nullptr;
        if (const char* share = std::getenv("RAGDOLL_TRACE_ROOT_SHARE"))
            settings.mLegsLeadRootShare = std::stof(share);
        StrideSim sim(settings);
        ASSERT_TRUE(sim.valid());
        sim.setFacing(0.0f);
        for (int i = 0; i < 120; ++i)
            sim.step();
        sim.setRequestVelocity(JPH::Vec3(0.0f, 127.0f, 0.0f));
        for (int i = 0; i < 180; ++i)
        {
            sim.step();
            if (i % 6 != 5)
                continue;
            const auto& feet = sim.footfall()->feet();
            std::printf("t=%.2f pelvis y=%.1f z=%.1f vy=%.0f capsule=%.1f | L y=%.1f z=%.1f %s tgt=%.1f | R y=%.1f "
                        "z=%.1f %s tgt=%.1f | root=%.0f,%.0f\n",
                (i + 1) / 60.0f, sim.bodyPosition(sim.pelvis()).GetY(), sim.bodyPosition(sim.pelvis()).GetZ(),
                sim.bodyVelocity(sim.pelvis()).GetY(), sim.capsulePosition().GetY(),
                sim.bodyPosition(sim.feet()[0]).GetY(), sim.bodyPosition(sim.feet()[0]).GetZ(),
                feet[0].mSwinging ? "swing" : "plant", feet[0].mTarget.GetY(), sim.bodyPosition(sim.feet()[1]).GetY(),
                sim.bodyPosition(sim.feet()[1]).GetZ(), feet[1].mSwinging ? "swing" : "plant", feet[1].mTarget.GetY(),
                sim.rootForce().GetY(), sim.rootForce().GetZ());
        }
    }

    // The trained get-up network (run by name; RAGDOLL_GETUP_NET = its file):
    // from the four lying starts at fixed caps on the root's help, how many
    // stand, how high the pelvis gets, how much help it takes.
    TEST(RagdollCoreStrideSim, DISABLED_getUpNetworkByAssist)
    {
        const char* path = std::getenv("RAGDOLL_GETUP_NET");
        ASSERT_NE(path, nullptr) << "set RAGDOLL_GETUP_NET";
        std::ifstream file(path, std::ios::binary);
        std::uint32_t sizes[4] = {};
        file.read(reinterpret_cast<char*>(sizes), sizeof(sizes));
        ASSERT_TRUE(file);
        GetUpPolicy::Net net;
        net.mIn = static_cast<int>(sizes[0]);
        net.mOut = static_cast<int>(sizes[3]);
        std::vector<float> weights(net.size());
        file.read(reinterpret_cast<char*>(weights.data()), weights.size() * sizeof(float));
        ASSERT_TRUE(file);
        GetUpPolicy::gDataDir = RAGDOLLCORE_TEST_DATA_DIR;
        for (const float cap : { 0.3f, 0.25f, 0.2f, 0.17f, 0.15f, 0.1f, 0.0f })
        {
            int stood = 0;
            float pelvis = 0.0f, assist = 0.0f;
            for (int start = 0; start < 4; ++start)
            {
                const auto episode = GetUpPolicy::runEpisode(net, weights.data(), start, cap);
                stood += episode.mStood ? 1 : 0;
                pelvis += episode.mBestPelvis / 4.0f;
                assist += episode.mMeanAssist / 4.0f;
            }
            std::printf("cap %.2f: stood %d/4, best pelvis %.2f of standing, mean help %.2f\n", cap, stood, pelvis,
                assist);
        }
    }

    TEST(RagdollCoreStrideSim, aLimpBodyLaidDownStaysDown)
    {
        StrideSim::Settings settings;
        settings.mLyingRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.5f * 3.14159265f);
        StrideSim sim(settings);
        ASSERT_TRUE(sim.valid()) << sim.error();
        sim.setLimpTarget(sim.basePose(), 0.15f);
        for (int i = 0; i < 120; ++i)
            sim.step();
        EXPECT_LT(static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ()), 0.5f * sim.standingPelvisHeight());
        EXPECT_LT(static_cast<float>(sim.bodyPosition(sim.head()).GetZ()), 0.5f * sim.standingPelvisHeight());
    }
}
