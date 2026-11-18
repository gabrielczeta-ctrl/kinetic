// ragdollcore-trainer: the MOVE-01 training pilot (section 9), offline only.
//
//   ragdollcore-trainer course DATA_DIR OUT.log [SKELETON_DUMP [CLIP_TRACK]]
//       The game's walking loop in StrideSim (stand, walk, stop, run, stop);
//       writes Footfall's RAGDOLL_FOOTFALL* lines to OUT.log so
//       tools/body/lift_reasons.py and walk_traces.py read it like a game log.
//   ragdollcore-trainer getup OUT_DIR HH:MM [threads [DATA_DIR]]
//       Evolution strategies (antithetic pairs, centred ranks, Adam) on a
//       2 x 64 tanh network that sets the joint targets and stiffness of a
//       body lying on the floor, rewarded for getting the pelvis, head and
//       torso up. One rollout per thread. Stops at HH:MM local time, or at
//       once when an openmw.exe is running (the player's game comes first).
//       Writes OUT_DIR/curve.csv every generation and OUT_DIR/getup.net
//       (four uint32 layer sizes, then float32 weights) when it improves.
//       Nothing here goes into the game.
//   ragdollcore-trainer arm DATA_DIR SKELETON_DUMP OUT.csv [HZ [STRENGTH]]
//       The hand drive's bench (KINETIC-2 phase 1): each weapon at 1 kg and
//       its real mass, the planner's click on each line, the right arm moved
//       only by the hand drive on the game's body with the pelvis held;
//       one CSV row per run (grip errors, tip speed, joint use). HZ overrides
//       the longsword's grip frequency (6), the others scaled with it;
//       STRENGTH scales the arm joints' caps (1 = the profile's).

#include "getuppolicy.hpp"

#include <components/ragdollprofile/loader.hpp>
#include <components/vfs/filesystemarchive.hpp>
#include <components/vfs/manager.hpp>

#include <ragdollcore/armsim.hpp>
#include <ragdollcore/playdefaults.hpp>
#include <ragdollcore/runtimeprofile.hpp>
#include <ragdollcore/stridesim.hpp>
#include <ragdollcore/tuning.hpp>
#include <ragdollcore/unitmodel.hpp>
#include <ragdollcore/wheelplanner.hpp>

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
// clang-format on

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace
{
    using namespace RagdollCore;

    bool gameRunning()
    {
#ifdef _WIN32
        FILE* pipe = _popen("tasklist /FI \"IMAGENAME eq openmw.exe\" /NH", "r");
        if (!pipe)
            return false;
        char buffer[512];
        bool found = false;
        while (std::fgets(buffer, sizeof(buffer), pipe))
            if (std::string(buffer).find("openmw.exe") != std::string::npos)
                found = true;
        _pclose(pipe);
        return found;
#else
        return false;
#endif
    }

    // ---- course -------------------------------------------------------------
    // The game's play values (RagdollCore::playDefaults), set as the engine
    // sets them: only where the environment has none. Unset
    // OPENMW_RAGDOLL_PLAY_DEFAULTS or non-zero applies them (W1: the course
    // is compared with the game's course run with the play values).
    void applyPlayDefaults()
    {
        if (const char* play = std::getenv("OPENMW_RAGDOLL_PLAY_DEFAULTS"); play && std::atoi(play) == 0)
            return;
        for (const RagdollCore::PlayDefault& entry : RagdollCore::playDefaults())
        {
            const std::string key(entry.mName);
            if (std::getenv(key.c_str()) != nullptr)
                continue;
            const std::string value(entry.mValue);
#ifdef _WIN32
            _putenv_s(key.c_str(), value.c_str());
#else
            setenv(key.c_str(), value.c_str(), 0);
#endif
        }
        RagdollCore::refreshTuning();
    }

    // StrideSim set up as the game walks (MOVE-01 W1): the play values, the
    // game's body and authored profile, its clip played back from step 120,
    // the floor, grip, feed-forward, root routing and solver tolerances.
    std::optional<StrideSim::Settings> gameSettings(
        const std::string& data, const std::string& skeleton, const std::string& clip)
    {
        applyPlayDefaults();
        StrideSim::Settings settings;
        settings.mActiveBodyPath = data + "/active-body.cfg";
        settings.mGaitTablePath = data + "/footfall-gaits.cfg";
        settings.mFootfall = FootfallSettings::fromEnvironment();
        // The game's own body (OPENMW_RAGDOLL_SKELETON_DUMP), when given, with
        // the profile the game loads.
        settings.mSkeletonPath = skeleton;
        if (!skeleton.empty())
        {
            VFS::Manager vfs;
            vfs.addArchive(std::make_unique<VFS::FileSystemArchive>(std::filesystem::path(data).parent_path()));
            vfs.buildIndex();
            const auto loaded = ::RagdollProfile::load(vfs, VFS::Path::NormalizedView("ragdoll/humanoid-v1.yaml"));
            if (!loaded.isValid() || !loaded.mProfile)
            {
                std::cerr << "humanoid-v1.yaml could not be loaded from " << data << std::endl;
                return std::nullopt;
            }
            settings.mProfile = *loaded.mProfile;
        }
        // The game's clip played back from the walk's start (step 120, 2 s).
        settings.mClipTrackPath = clip;
        settings.mClipTrackAlign = 120;
        // The game's active controller adds the legs' joint support.
        settings.mJointSupport = true;
        // The game's floor (a static at Jolt's default friction, 0.2) and its
        // per-foot grip.
        settings.mGroundFriction = 0.2f;
        settings.mGameFootGrip = true;
        settings.mSwingFeedForward = true;
        settings.mGameRootSupport = true;
        settings.mActorHeight = 4.2f;
        // OPENMW_RAGDOLL_SIM_ACTOR_HEIGHT: the actor origin over the floor (11-05:
        // the game's PLAY-01 course stands it 1.0 over its floor, the state dump's
        // origin z against LEG_DRIVE's floorZ, all walk).
        if (const char* text = std::getenv("OPENMW_RAGDOLL_SIM_ACTOR_HEIGHT"))
            settings.mActorHeight = std::stof(text);
        settings.mGameContacts = true;
        // OPENMW_RAGDOLL_SIM_FLOOR=holdslab: the prison ship's hold floor
        // (W2, the thin floor the AI walker sinks into), from its start.
        if (const char* floor = std::getenv("OPENMW_RAGDOLL_SIM_FLOOR"); floor && std::string(floor) == "holdslab")
            settings.mFloor = StrideSim::Settings::Floor::HoldSlab;
        if (const char* text = tuningText("OPENMW_RAGDOLL_FLOOR_UNDERSIDE"))
            settings.mRejectFloorUnderside = std::atoi(text) != 0;
        // Experiments: the root's moving strength (OPENMW_RAGDOLL_SIM_MOVING);
        // the legs leading by the stance force pair, "friction share,
        // damping" (OPENMW_RAGDOLL_SIM_LEGS_LEAD, MOVE-01 W4).
        if (const char* text = std::getenv("OPENMW_RAGDOLL_SIM_MOVING"))
            settings.mMovingStrength = std::stof(text);
        if (const char* text = std::getenv("OPENMW_RAGDOLL_SIM_WALK_SUPPORT"))
            settings.mWalkSupport = std::stof(text);
        // The game's upright spine torque while walking (11-06: on in the game's
        // walking route, missing here; neutral on the course, 20 seeds: landing
        // 8.8 -> 9.1, falls 3 -> 3). OPENMW_RAGDOLL_SIM_UPRIGHT=0 takes it off.
        settings.mUprightSpine = true;
        if (const char* text = std::getenv("OPENMW_RAGDOLL_SIM_UPRIGHT"))
            settings.mUprightSpine = std::atoi(text) != 0;
        // The game's walking swing spring (PGC's, on in play: OPENMW_RAGDOLL_PGC's
        // third knob; W1 11-06, the landing gap: missing here, the sim's foot
        // ended 9 u from its target against the game's 2), at the course
        // player's 4 + 6 (0.6 Agility + 0.4 Acrobatics) / 100 Hz: Acrobatics 5
        // (PLAY01_ACROBATICS), Agility taken as 40 -> 5.6.
        // OPENMW_RAGDOLL_SIM_PGC_SWING=Hz (0 = off). Stride's reach on it:
        // OPENMW_RAGDOLL_K2_STRIDE_SWING, as the game reads it.
        settings.mStrideSwing.mFrequency = 5.6f;
        if (const char* text = std::getenv("OPENMW_RAGDOLL_SIM_PGC_SWING"))
        {
            float frequency = settings.mStrideSwing.mFrequency, cap = settings.mStrideSwing.mCapLegWeights;
            std::sscanf(text, "%f,%f", &frequency, &cap);
            settings.mStrideSwing.mFrequency = std::max(0.0f, frequency);
            settings.mStrideSwing.mCapLegWeights = std::max(0.0f, cap);
        }
        settings.mStrideSwing.mReach = strideSwingReach();
        // OPENMW_RAGDOLL_SIM_SWING_LIFT="Hz,cap,until": Stride's lift (offline).
        if (const char* text = std::getenv("OPENMW_RAGDOLL_SIM_SWING_LIFT"))
            std::sscanf(text, "%f,%f,%f", &settings.mStrideSwing.mLiftFrequency, &settings.mStrideSwing.mLiftCap,
                &settings.mStrideSwing.mLiftUntil);
        // The game's run bob (OPENMW_RAGDOLL_RUN_BOB "keep, sink, speed", the play
        // value; 11-06 parity: the running sim floated without it).
        if (const char* text = tuningText("OPENMW_RAGDOLL_RUN_BOB"))
        {
            RunBobSettings bob;
            std::sscanf(text, "%f,%f,%f", &bob.mKeep, &bob.mSink, &bob.mSpeed);
            bob.mKeep = std::clamp(bob.mKeep, 0.0f, 1.0f);
            bob.mSink = std::clamp(bob.mSink, 0.0f, 20.0f);
            settings.mRunBob = bob;
        }
        // OPENMW_RAGDOLL_SIM_PLANNER=until: the step planner aims each swing's
        // landing while its phase is at most `until` (MOVE-01 4.2; 0 = off).
        if (const char* text = std::getenv("OPENMW_RAGDOLL_SIM_PLANNER"))
        {
            settings.mPlanUntil = std::stof(text);
            settings.mPlanner = settings.mPlanUntil > 0.0f;
        }
        if (const char* text = std::getenv("OPENMW_RAGDOLL_SIM_ROOT_LOWER"))
            settings.mRootLower = std::stof(text);
        if (const char* text = std::getenv("OPENMW_RAGDOLL_SIM_LEGS_LEAD"))
        {
            float friction = 0.5f, damping = 10.0f, maximum = 0.0f, grip = 0.0f;
            std::sscanf(text, "%f,%f,%f,%f", &friction, &damping, &maximum, &grip);
            settings.mStanceForce.mGripMargin = grip;
            settings.mLegsLeadMaxSpeed = maximum;
            if (const char* cruise = std::getenv("OPENMW_RAGDOLL_SIM_LEGS_CRUISE"))
                settings.mLegsLeadCruiseError = std::stof(cruise);
            // Steps by events (mTransfer): "load, time limit, swing time".
            if (const char* transfer = std::getenv("OPENMW_RAGDOLL_SIM_TRANSFER"))
            {
                settings.mTransfer = true;
                std::sscanf(transfer, "%f,%f,%f", &settings.mTransferLoad, &settings.mTransferTime,
                    &settings.mTransferSwingTime);
            }
            settings.mLegsLead = true;
            settings.mLegsLeadRootShare = 0.0f;
            // The game steps for the root's actual velocity (gaitCommandVelocity),
            // which follows the body while the legs lead; stepping for the asked
            // speed planned 80-87 u strafe steps against the game's 60 and the
            // body fell sideways past them (W1 11-05).
            // OPENMW_RAGDOLL_SIM_STEP_INTENT=1 = step for the asked speed.
            const char* intent = std::getenv("OPENMW_RAGDOLL_SIM_STEP_INTENT");
            settings.mLegsLeadStepForIntent = intent && std::atoi(intent) != 0;
            settings.mStanceForce.mFrictionShare = friction;
            settings.mStanceForce.mDamping = damping;
            // OPENMW_RAGDOLL_SIM_LEGS_ASSIST="root share, error share, leash": the root's
            // horizontal hold while the legs lead, as a budget (MOVE-01 4.6
            // `assist`: the fixed share plus error share x the velocity error
            // over the asked speed).
            if (const char* assist = std::getenv("OPENMW_RAGDOLL_SIM_LEGS_ASSIST"))
                std::sscanf(assist, "%f,%f,%f", &settings.mLegsLeadRootShare, &settings.mLegsLeadErrorShare,
                    &settings.mLegsLeadLeash);
        }
        return settings;
    }

    int runCourse(const std::string& data, const std::string& outPath, const std::string& skeleton,
        const std::string& clip)
    {
        std::ofstream out(outPath);
        auto made = gameSettings(data, skeleton, clip);
        if (!made)
            return 1;
        StrideSim::Settings settings = *made;
        // Stamped with the simulated time, so the game's log readers
        // (tools/body/walk_traces.py) read the course as they read a game log.
        const StrideSim* clock = nullptr;
        settings.mTrace = [&](const std::string& line) {
            const double t = clock ? clock->time() : 0.0;
            char stamp[32];
            std::snprintf(stamp, sizeof(stamp), "[%02d:%02d:%06.3f I] ", static_cast<int>(t / 3600.0),
                static_cast<int>(t / 60.0) % 60, std::fmod(t, 60.0));
            out << stamp << line << "\n";
        };
        StrideSim sim(settings);
        clock = &sim;
        if (!sim.valid())
        {
            std::cerr << "StrideSim: " << sim.error() << "\n";
            return 1;
        }
        if (!sim.error().empty())
            std::cerr << "StrideSim warnings: " << sim.error() << "\n";
        // The game's test course (test_play01_walking protocols.lua "course"):
        // forward and right inputs against the facing, running or not, and a
        // turn rate (rad/s).
        struct Segment
        {
            const char* mName;
            float mSeconds;
            float mForward, mRight;
            bool mRun;
            float mTurn;
        };
        const Segment course[] = { { "stand", 2.0f, 0, 0, false, 0 }, { "walk", 4.0f, 1, 0, false, 0 },
            { "run", 4.0f, 1, 0, true, 0 }, { "walk again", 3.0f, 1, 0, false, 0 },
            { "strafe left", 3.0f, 0, -1, false, 0 }, { "strafe right", 3.0f, 0, 1, false, 0 },
            { "backpedal", 3.0f, -1, 0, false, 0 }, { "walk to run", 1.5f, 1, 0, false, 0 },
            { "run again", 3.0f, 1, 0, true, 0 }, { "run strafe left", 2.5f, 0, -1, true, 0 },
            { "run strafe right", 2.5f, 0, 1, true, 0 }, { "run turn", 3.0f, 1, 0, true, 1.2f },
            { "run to walk", 3.0f, 1, 0, false, 0 }, { "stop", 2.0f, 0, 0, false, 0 },
            // A fight's circling (MOVE-01 W6; not in the game's course): a
            // strafe around a foe 120 units away, turning to keep facing it,
            // at the game's strafe speeds (96 walking, 173 running).
            { "circle walk", 6.0f, 0, 1, false, 96.0f / 120.0f }, { "circle stop", 1.5f, 0, 0, false, 0 },
            { "circle run", 4.0f, 0, 1, true, 173.0f / 120.0f }, { "circle end", 2.0f, 0, 0, false, 0 } };
        // OPENMW_RAGDOLL_SIM_COURSE=sprint: the game's `course sprint` (Speed and
        // Athletics 100: ~200 u/s walking, ~540 running; its clip dump
        // tools-workspace/w1-sprint-clip.txt).
        const Segment sprintCourse[] = { { "stand", 2.0f, 0, 0, false, 0 }, { "walk", 2.0f, 1, 0, false, 0 },
            { "run", 4.0f, 1, 0, true, 0 }, { "walk again", 2.0f, 1, 0, false, 0 }, { "stop", 2.0f, 0, 0, false, 0 } };
        const char* courseName = std::getenv("OPENMW_RAGDOLL_SIM_COURSE");
        const bool sprinting = courseName && std::string_view(courseName) == "sprint";
        const std::span<const Segment> segments = sprinting ? std::span<const Segment>(sprintCourse)
                                                            : std::span<const Segment>(course);
        const float walk = 127.0f, run = 230.0f;
        // With the game's clip, the speed the game asked for at that frame
        // (its strafes and backpedal are slower than its walk).
        std::optional<ClipTrack> track;
        std::size_t trackStart = 0;
        if (!clip.empty())
        {
            track = readClipTrack(clip);
            while (track && trackStart + 1 < track->mRequestSpeed.size() && track->mRequestSpeed[trackStart] < 1.0f)
                ++trackStart;
        }
        // The replay bridge (MOVE-01 W1): OPENMW_RAGDOLL_SIM_STATE="STATE_DUMP,seconds"
        // puts the body where the game's was that many seconds into the walk
        // (the state dump's lines run with the clip track's).
        std::vector<std::string> stateLines;
        long long stateStep = -1;
        std::size_t stateLine = 0;
        // OPENMW_RAGDOLL_SIM_STATE_OUT=path: the sim's own state each step, in the
        // state dump's format (the capsule for the actor's origin; no angular
        // velocities), written where the game writes its (before the step).
        std::ofstream stateOut;
        // OPENMW_RAGDOLL_SIM_DRIVE_OUT=path: each step, the feet's motor targets (world
        // rotations x y z w, left then right), after the step.
        std::ofstream driveOut;
        if (const char* text = std::getenv("OPENMW_RAGDOLL_SIM_DRIVE_OUT"); text && *text)
            driveOut.open(text);
        if (const char* text = std::getenv("OPENMW_RAGDOLL_SIM_STATE_OUT"); text && *text)
            stateOut.open(text);
        if (const char* text = std::getenv("OPENMW_RAGDOLL_SIM_STATE"); text && track)
        {
            const std::string spec(text);
            const auto comma = spec.rfind(',');
            std::ifstream stateIn(spec.substr(0, comma));
            for (std::string line; std::getline(stateIn, line);)
                stateLines.push_back(line);
            const float seconds = comma == std::string::npos ? 0.0f : std::stof(spec.substr(comma + 1));
            stateStep = 120 + std::lround(seconds * 60.0f);
            stateLine = trackStart + static_cast<std::size_t>(std::lround(seconds * 60.0f));
            if (stateLine >= stateLines.size())
            {
                std::cerr << "state dump " << spec << ": " << stateLines.size() << " lines, none at " << stateLine << std::endl;
                return 1;
            }
        }
        float yaw = 0.0f;
        bool fell = false;
        for (const Segment& segment : segments)
        {
            float lowest = 1e9f, lag = 0.0f;
            const int steps = static_cast<int>(segment.mSeconds * 60.0f);
            for (int i = 0; i < steps; ++i)
            {
                yaw += segment.mTurn / 60.0f;
                float speed = segment.mRun ? run : walk;
                if (std::string_view(segment.mName).rfind("circle", 0) == 0)
                    speed = segment.mRun ? 173.0f : 96.0f;
                else if (track)
                {
                    // OPENMW_RAGDOLL_SIM_REQUEST_DELAY=n: the asked speed n steps late
                    // (the game's drive starts 3 steps after its walk request, W1 11-05).
                    static const long long requestDelay = [] {
                        const char* text = std::getenv("OPENMW_RAGDOLL_SIM_REQUEST_DELAY");
                        return text ? std::atoll(text) : 0LL;
                    }();
                    const long long frame = static_cast<long long>(trackStart) + static_cast<long long>(sim.steps()) - 120
                        - requestDelay;
                    speed = track->mRequestSpeed[static_cast<std::size_t>(
                        std::clamp(frame, 0LL, static_cast<long long>(track->mRequestSpeed.size()) - 1))];
                }
                const JPH::Vec3 forward(std::sin(yaw), std::cos(yaw), 0.0f), right(std::cos(yaw), -std::sin(yaw), 0.0f);
                JPH::Vec3 request = forward * segment.mForward + right * segment.mRight;
                if (!request.IsNearZero())
                    request = request.Normalized() * speed;
                if (static_cast<long long>(sim.steps()) == stateStep)
                {
                    std::cout << sim.bodyMassesLine() << std::endl;
                    std::cout << "state from the game (line " << stateLine << "): "
                              << (sim.setGameState(stateLines[stateLine], yaw) ? "set" : "did not fit the body") << std::endl;
                }
                // OPENMW_RAGDOLL_SIM_STATE_HOLD=n: the game's next n lines are put
                // in too, step by step (its bodies sit asleep 3 steps into a walk
                // while Footfall already lifts; W1 11-05).
                else if (stateStep >= 0 && static_cast<long long>(sim.steps()) > stateStep)
                    if (const char* hold = std::getenv("OPENMW_RAGDOLL_SIM_STATE_HOLD"))
                    {
                        const long long k = static_cast<long long>(sim.steps()) - stateStep;
                        const std::size_t line = stateLine + static_cast<std::size_t>(k);
                        if (k <= std::atoll(hold) && line < stateLines.size())
                            sim.setGameState(stateLines[line], yaw);
                    }
                if (stateOut.is_open())
                {
                    const JPH::RVec3 capsule = sim.capsulePosition();
                    stateOut << std::setprecision(9) << capsule.GetX() << ' ' << capsule.GetY() << ' ' << capsule.GetZ()
                             << ' ' << yaw << ' ' << sim.bodyCount();
                    for (std::size_t body = 0; body < sim.bodyCount(); ++body)
                    {
                        const JPH::RVec3 p = sim.bodyPosition(body);
                        const JPH::Quat q = sim.bodyRotation(body);
                        const JPH::Vec3 v = sim.bodyVelocity(body);
                        stateOut << ' ' << p.GetX() << ' ' << p.GetY() << ' ' << p.GetZ() << ' ' << q.GetX() << ' '
                                 << q.GetY() << ' ' << q.GetZ() << ' ' << q.GetW() << ' ' << v.GetX() << ' ' << v.GetY()
                                 << ' ' << v.GetZ() << " 0 0 0";
                    }
                    stateOut << std::endl;
                }
                sim.setFacing(yaw);
                sim.setRequestVelocity(request);
                sim.step();
                if (driveOut.is_open())
                {
                    driveOut << sim.steps();
                    for (const std::size_t foot : sim.feet())
                    {
                        const JPH::Quat q = sim.driveRotation(foot).value_or(JPH::Quat::sIdentity());
                        driveOut << ' ' << q.GetX() << ' ' << q.GetY() << ' ' << q.GetZ() << ' ' << q.GetW();
                    }
                    driveOut << std::endl;
                }
                const float pelvis = static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ());
                lowest = std::min(lowest, pelvis);
                const JPH::RVec3 gap = sim.bodyPosition(sim.pelvis()) - sim.capsulePosition();
                lag = std::max(lag, static_cast<float>(std::hypot(gap.GetX(), gap.GetY())));
            }
            fell = fell || lowest < 0.5f * sim.standingPelvisHeight();
            std::cout << segment.mName << ": lowest pelvis " << lowest << " (standing " << sim.standingPelvisHeight()
                      << "), worst lag " << lag << "\n";
        }
        std::cout << (fell ? "FELL" : "upright throughout") << "\n";
        return fell ? 2 : 0;
    }

    // ---- lead (MOVE-01 W4: the legs leading, on the matched body) ----------
    // A start from rest at the game's walk (128 u/s), 4 s, a reversal, a stop,
    // per variant: the time to 80% speed, the cruise speed, the root's
    // horizontal share of the weight (assist) and its vertical, the legs'
    // load, the planted feet's slide (median, u/s), the lowest pelvis.
    struct LeadVariant
    {
        const char* mName;
        bool mLegsLead = false;
        float mRootShare = 0.1f;
        float mLeash = 0.0f;
        bool mThroughSupport = false;
        bool mPush = false;
        bool mAim = false;
        bool mTransfer = false;
        bool mIntent = true;
        float mErrorShare = 0.0f;
        // The stance force pair (StrideSim::legsLead, computeStanceForce):
        // friction share and damping (0 = StrideSim's defaults).
        float mFriction = 0.0f;
        float mDamping = 0.0f;
        float mGrip = 0.0f; // StanceForceSettings::mGripMargin
        float mCruise = 0.0f; // mLegsLeadCruiseError
    };

    int runLead(const std::string& data, const std::string& skeleton, const std::string& clip)
    {
        const LeadVariant variants[] = { { "capsule leads" },
            { "f.45/10 g.6", true, 0.0f, 0.0f, false, false, false, false, true, 0.0f, 0.45f, 10.0f, 0.6f },
            { "f.45/10 g.6 c.15", true, 0.0f, 0.0f, false, false, false, false, true, 0.0f, 0.45f, 10.0f, 0.6f, 0.15f },
            { "f.45/10 g.6 c.25", true, 0.0f, 0.0f, false, false, false, false, true, 0.0f, 0.45f, 10.0f, 0.6f, 0.25f },
            { "f.45/10 g.8 c.15", true, 0.0f, 0.0f, false, false, false, false, true, 0.0f, 0.45f, 10.0f, 0.8f, 0.15f },
            { "f.6/15 g.6 c.15", true, 0.0f, 0.0f, false, false, false, false, true, 0.0f, 0.6f, 15.0f, 0.6f, 0.15f } };
        for (const LeadVariant& variant : variants)
        {
            auto made = gameSettings(data, skeleton, clip);
            if (!made)
                return 1;
            StrideSim::Settings settings = *made;
            settings.mLegsLead = variant.mLegsLead;
            settings.mLegsLeadRootShare = variant.mRootShare;
            settings.mLegsLeadLeash = variant.mLeash;
            settings.mLegsLeadThroughSupport = variant.mThroughSupport;
            settings.mLegsLeadPush = variant.mPush;
            settings.mLegsLeadAimSwing = variant.mAim;
            settings.mTransfer = variant.mTransfer;
            settings.mLegsLeadStepForIntent = variant.mIntent;
            settings.mLegsLeadErrorShare = variant.mErrorShare;
            if (variant.mFriction > 0.0f)
                settings.mStanceForce.mFrictionShare = variant.mFriction;
            if (variant.mDamping > 0.0f)
                settings.mStanceForce.mDamping = variant.mDamping;
            settings.mStanceForce.mGripMargin = variant.mGrip;
            settings.mLegsLeadCruiseError = variant.mCruise;
            StrideSim sim(settings);
            if (!sim.valid())
            {
                std::cerr << "StrideSim: " << sim.error() << "\n";
                return 1;
            }
            sim.setFacing(0.0f);
            const float weight = settings.mTotalMass * RagdollCore::UnitModel::GravityUnitsPerSecondSquared;
            const float speed = 128.18f;
            std::vector<float> slide, cruise, contactSlip;
            float reach80 = -1.0f, lowest = 1e9f, reversal = -1.0f, assist = 0.0f, rootUp = 0.0f, legs = 0.0f;
            int cruiseSteps = 0;
            std::array<std::optional<JPH::RVec3>, 2> lastPin;
            const auto stepAndMeasure = [&](bool inCruise) {
                sim.step();
                lowest = std::min(lowest, static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ()));
                const auto& feet = sim.footfall()->feet();
                for (std::size_t side = 0; side < 2; ++side)
                {
                    const JPH::RVec3 at = sim.bodyPosition(sim.feet()[side]);
                    if (!feet[side].mSwinging && lastPin[side])
                        slide.push_back(static_cast<float>(
                            std::hypot(at.GetX() - lastPin[side]->GetX(), at.GetY() - lastPin[side]->GetY()) * 60.0));
                    lastPin[side] = feet[side].mSwinging ? std::nullopt : std::optional(at);
                }
                if (inCruise)
                {
                    const JPH::Vec3 root = sim.rootForce();
                    assist += std::hypot(root.GetX(), root.GetY()) / weight;
                    rootUp += root.GetZ() / weight;
                    float load = 0.0f;
                    std::array<float, 2> footLoad{}, footSlip{};
                    for (const GroundContact& contact : sim.groundContacts())
                        for (std::size_t side = 0; side < 2; ++side)
                            if (contact.mBodyIndex == sim.feet()[side] && contact.mNormalImpulse)
                            {
                                const float up = *contact.mNormalImpulse * contact.mNormal.GetZ() * 60.0f;
                                load += up;
                                footLoad[side] += up;
                                if (contact.mPointVelocity)
                                    footSlip[side] = std::max(footSlip[side],
                                        std::hypot(contact.mPointVelocity->GetX(), contact.mPointVelocity->GetY()));
                            }
                    legs += load / weight;
                    // Contact slip of a foot carrying a third of the weight or more.
                    for (std::size_t side = 0; side < 2; ++side)
                        if (footLoad[side] >= weight / 3.0f)
                            contactSlip.push_back(footSlip[side]);
                    ++cruiseSteps;
                }
            };
            for (int i = 0; i < 120; ++i)
                stepAndMeasure(false);
            slide.clear();
            sim.setRequestVelocity(JPH::Vec3(0.0f, speed, 0.0f));
            for (int i = 0; i < 240; ++i)
            {
                stepAndMeasure(i >= 120);
                const float vy = static_cast<float>(sim.bodyVelocity(sim.pelvis()).GetY());
                if (reach80 < 0.0f && vy >= 0.8f * speed)
                    reach80 = (i + 1) / 60.0f;
                if (i >= 120)
                    cruise.push_back(vy);
            }
            std::vector<float> cruiseSlide = slide;
            sim.setRequestVelocity(JPH::Vec3(0.0f, -speed, 0.0f));
            for (int i = 0; i < 120; ++i)
            {
                stepAndMeasure(false);
                if (reversal < 0.0f && -sim.bodyVelocity(sim.pelvis()).GetY() >= 0.5f * speed)
                    reversal = (i + 1) / 60.0f;
            }
            sim.setRequestVelocity(JPH::Vec3::sZero());
            for (int i = 0; i < 120; ++i)
                stepAndMeasure(false);
            const auto median = [](std::vector<float> v) {
                if (v.empty())
                    return -1.0f;
                std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
                return v[v.size() / 2];
            };
            float mean = 0.0f;
            for (float v : cruise)
                mean += v / std::max<std::size_t>(cruise.size(), 1);
            const float n = static_cast<float>(std::max(cruiseSteps, 1));
            std::printf("%-18s to80%%=%.2fs cruise=%.0f assist=%.3f rootUp=%.2f legs=%.2f slide=%.1f cslip=%.1f "
                        "reversal=%.2fs lowest=%.1f%s\n",
                variant.mName, reach80, mean, assist / n, rootUp / n, legs / n, median(cruiseSlide),
                median(contactSlip), reversal, lowest,
                lowest < 0.5f * sim.standingPelvisHeight() ? " FELL" : "");
        }
        return 0;
    }

    // ---- getup (the policy: getuppolicy.hpp) ------------------------------
    using namespace GetUpPolicy;

    std::string clock()
    {
        const std::time_t now = std::time(nullptr);
        char buffer[16];
        std::strftime(buffer, sizeof(buffer), "%H:%M:%S", std::localtime(&now));
        return buffer;
    }

    bool pastDeadline(int hour, int minute)
    {
        const std::time_t now = std::time(nullptr);
        const std::tm* t = std::localtime(&now);
        return t->tm_hour * 60 + t->tm_min >= hour * 60 + minute && t->tm_hour < 12;
    }

    int runGetUp(const std::string& outDir, const std::string& until, int threads)
    {
        int hour = 8, minute = 0;
        std::sscanf(until.c_str(), "%d:%d", &hour, &minute);
        std::filesystem::create_directories(outDir);
        // Sizes from one probe body.
        StrideSim::Settings probeSettings;
        probeSettings.mLyingRotation = lyingRotation(0);
        probeSettings.mLimpAssist = true;
        StrideSim probe(probeSettings);
        if (!probe.valid())
        {
            std::cerr << "StrideSim: " << probe.error() << "\n";
            return 1;
        }
        std::vector<float> x;
        observe(probe, 0.0f, x);
        Net net;
        net.mIn = static_cast<int>(x.size());
        // Per joint a rotation offset, then the stiffness and the root's help.
        net.mOut = static_cast<int>((probe.bodyCount() - 1) * 3 + 2);
        const std::size_t n = net.size();
        std::mt19937 rng(20261103);
        std::normal_distribution<float> normal(0.0f, 1.0f);
        std::vector<float> theta(n);
        for (float& v : theta)
            v = 0.05f * normal(rng);
        // Resume from the best network so far (a run stopped for the game).
        if (std::ifstream saved{ outDir + "/getup.net", std::ios::binary })
        {
            std::uint32_t sizes[4] = {};
            saved.read(reinterpret_cast<char*>(sizes), sizeof(sizes));
            if (saved && sizes[0] == static_cast<std::uint32_t>(net.mIn) && sizes[3] == static_cast<std::uint32_t>(net.mOut))
            {
                std::vector<float> loaded(n);
                saved.read(reinterpret_cast<char*>(loaded.data()), loaded.size() * sizeof(float));
                if (saved)
                {
                    theta = loaded;
                    std::cout << "resumed from " << outDir << "/getup.net" << std::endl;
                }
            }
        }
        std::vector<float> m(n, 0.0f), v2(n, 0.0f);
        const int pairs = 24;
        const float sigma = 0.04f, rate = 0.02f, beta1 = 0.9f, beta2 = 0.999f, decay = 0.005f;
        // Appends across resumed runs (each run's generations count from 1).
        const bool fresh = !std::filesystem::exists(outDir + "/curve.csv");
        std::ofstream curve(outDir + "/curve.csv", std::ios::app);
        if (fresh)
            curve << "generation,time,mean,best,center,center_best_pelvis,center_stood,center_assist,seconds\n";
        std::cout << "params " << n << " in " << net.mIn << " out " << net.mOut << " threads " << threads
                  << " until " << until << "\n";
        float bestCenter = -1e9f;
        auto lastCheck = std::chrono::steady_clock::now();
        const auto begun = std::chrono::steady_clock::now();
        // The assist curriculum, adaptive: the cap on the root's help falls by
        // 0.01 after an evaluation where all four starts stand and rises by
        // 0.01 after one where at most one does, so the run tracks the least
        // help the network can stand up with. OUT_DIR/cap.txt keeps it across
        // resumed runs. (A fixed fall from 0.6 to 0.12 lost all four starts
        // below ~0.17.)
        float cap = 0.6f;
        if (std::ifstream saved{ outDir + "/cap.txt" })
            saved >> cap;
        for (int generation = 1;; ++generation)
        {
            if (pastDeadline(hour, minute))
            {
                std::cout << clock() << " deadline reached\n";
                break;
            }
            if (std::chrono::steady_clock::now() - lastCheck > std::chrono::seconds(20))
            {
                lastCheck = std::chrono::steady_clock::now();
                if (gameRunning())
                {
                    std::cout << clock() << " openmw.exe is running: stopping\n";
                    break;
                }
            }
            const auto started = std::chrono::steady_clock::now();
            std::vector<std::vector<float>> noise(pairs, std::vector<float>(n));
            for (auto& e : noise)
                for (float& v : e)
                    v = normal(rng);
            // Each candidate on two starts, the same for the whole generation.
            const int startA = generation % 4, startB = (generation + 2) % 4;
            std::vector<float> rewards(pairs * 2, 0.0f);
            std::atomic<int> next{ 0 };
            const auto work = [&] {
                std::vector<float> w(n);
                for (int job; (job = next.fetch_add(1)) < pairs * 2;)
                {
                    const int pair = job / 2;
                    const float sign = job % 2 == 0 ? 1.0f : -1.0f;
                    for (std::size_t k = 0; k < n; ++k)
                        w[k] = theta[k] + sign * sigma * noise[pair][k];
                    rewards[job] = 0.5f * (runEpisode(net, w.data(), startA, cap).mReward + runEpisode(net, w.data(), startB, cap).mReward);
                }
            };
            std::vector<std::thread> pool;
            for (int t = 0; t < threads; ++t)
                pool.emplace_back(work);
            for (auto& t : pool)
                t.join();
            // Centred ranks, then the gradient estimate and an Adam step.
            std::vector<int> order(rewards.size());
            std::iota(order.begin(), order.end(), 0);
            std::sort(order.begin(), order.end(), [&](int a, int b) { return rewards[a] < rewards[b]; });
            std::vector<float> shaped(rewards.size());
            for (std::size_t r = 0; r < order.size(); ++r)
                shaped[order[r]] = static_cast<float>(r) / (order.size() - 1) - 0.5f;
            std::vector<float> grad(n, 0.0f);
            for (int pair = 0; pair < pairs; ++pair)
            {
                const float weight = shaped[pair * 2] - shaped[pair * 2 + 1];
                for (std::size_t k = 0; k < n; ++k)
                    grad[k] += weight * noise[pair][k];
            }
            for (std::size_t k = 0; k < n; ++k)
            {
                const float g = -grad[k] / (pairs * 2 * sigma) + decay * theta[k];
                m[k] = beta1 * m[k] + (1 - beta1) * g;
                v2[k] = beta2 * v2[k] + (1 - beta2) * g * g;
                const float mh = m[k] / (1 - std::pow(beta1, static_cast<float>(generation)));
                const float vh = v2[k] / (1 - std::pow(beta2, static_cast<float>(generation)));
                theta[k] -= rate * mh / (std::sqrt(vh) + 1e-8f);
            }
            // The centre on all four starts, every 5 generations.
            float center = 0.0f, centerPelvis = 0.0f, centerAssist = 0.0f;
            int stood = 0;
            if (generation % 5 == 1)
            {
                std::vector<Episode> eval(4);
                std::vector<std::thread> evalPool;
                for (int s = 0; s < 4; ++s)
                    evalPool.emplace_back([&, s] { eval[s] = runEpisode(net, theta.data(), s, cap); });
                for (auto& t : evalPool)
                    t.join();
                for (const Episode& e : eval)
                {
                    center += e.mReward / 4.0f;
                    centerPelvis += e.mBestPelvis / 4.0f;
                    centerAssist += e.mMeanAssist / 4.0f;
                    stood += e.mStood ? 1 : 0;
                }
                if (stood == 4)
                    cap = std::max(0.05f, cap - 0.01f);
                else if (stood <= 1)
                    cap = std::min(0.6f, cap + 0.01f);
                std::ofstream(outDir + "/cap.txt") << cap;
                // getup.net: the latest centre (the run resumes from it, at the
                // curriculum's current cap); getup-best.net: the best reward.
                const auto save = [&](const std::string& path) {
                    std::ofstream file(path, std::ios::binary);
                    const std::uint32_t sizes[4] = { static_cast<std::uint32_t>(net.mIn), sHidden, sHidden,
                        static_cast<std::uint32_t>(net.mOut) };
                    file.write(reinterpret_cast<const char*>(sizes), sizeof(sizes));
                    file.write(reinterpret_cast<const char*>(theta.data()), theta.size() * sizeof(float));
                };
                save(outDir + "/getup.net");
                if (center > bestCenter)
                {
                    bestCenter = center;
                    save(outDir + "/getup-best.net");
                }
            }
            const float mean = std::accumulate(rewards.begin(), rewards.end(), 0.0f) / rewards.size();
            const float best = *std::max_element(rewards.begin(), rewards.end());
            const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            curve << generation << "," << clock() << "," << mean << "," << best << ","
                  << (generation % 5 == 1 ? std::to_string(center) : "") << ","
                  << (generation % 5 == 1 ? std::to_string(centerPelvis) : "") << ","
                  << (generation % 5 == 1 ? std::to_string(stood) : "") << ","
                  << (generation % 5 == 1 ? std::to_string(centerAssist) : "") << "," << seconds << "\n";
            curve.flush();
            if (generation % 5 == 1)
                std::cout << clock() << " gen " << generation << " cap " << cap << " mean " << mean << " best " << best
                          << " centre "
                          << center << " pelvis " << centerPelvis << " assist " << centerAssist << " stood " << stood
                          << "/4 (" << seconds
                          << " s/gen)" << std::endl;
        }
        std::cout << "trained for "
                  << std::chrono::duration<double>(std::chrono::steady_clock::now() - begun).count() / 60.0
                  << " min; best centre " << bestCenter << "\n";
        return 0;
    }

    // The hand drive's bench (KINETIC-2 phase 1, order step 3): each weapon
    // at the game's mass (1 kg) and at its real one, the planner's click on
    // each of the eight lines, the arm moved only by the hand drive.
    // Per run: the grip's error held in the guard, its RMS and worst over
    // the wind-up and cut, at mid-cut, the tip's top speed, the rotation's
    // worst error, the joints' top use of their strength and the share of
    // steps saturated, and where it settles back in the guard.
    struct BenchWeapon
    {
        const char* mName;
        float mWeight; // Morrowind weight (the planner's pace)
        float mRealMass; // kg (weight x 0.08)
        float mLength, mTip;
        bool mTwoHanded, mShort;
    };

    int runArm(const std::string& data, const std::string& skeleton, const std::string& outPath, float frequency,
        float strengthScale)
    {
        VFS::Manager vfs;
        vfs.addArchive(std::make_unique<VFS::FileSystemArchive>(std::filesystem::path(data).parent_path()));
        vfs.buildIndex();
        const auto loaded = ::RagdollProfile::load(vfs, VFS::Path::NormalizedView("ragdoll/humanoid-v1.yaml"));
        if (!loaded.isValid() || !loaded.mProfile)
        {
            std::cerr << "humanoid-v1.yaml could not be loaded from " << data << std::endl;
            return 1;
        }
        std::ofstream out(outPath);
        out << "weapon,mass,strength,line,guard_err,guard_blade_deg,rms_err,max_err,mid_err,tip_speed,blade_err_deg,blade_rms_deg,blade_mid_deg,solved_blade_rms,solved_reach_max,util_shoulder,util_elbow,"
               "util_wrist,saturated,settle_err\n";
        const std::array<BenchWeapon, 4> weapons{ {
            { "iron dagger", 3.0f, 0.4f, 40.0f, 32.0f, false, true },
            { "iron longsword", 20.0f, 1.6f, 95.0f, 85.0f, false, false },
            { "iron claymore", 27.5f, 2.2f, 130.0f, 115.0f, true, false },
            { "iron warhammer", 32.5f, 2.6f, 95.0f, 85.0f, true, false },
        } };
        constexpr float dt = 1.0f / 60.0f;
        for (const BenchWeapon& weapon : weapons)
            for (const bool real : { false, true })
                for (std::size_t line = 0; line < 8; ++line)
                {
                    ArmSim::Settings settings;
                    settings.mSkeletonPath = skeleton;
                    settings.mProfile = *loaded.mProfile;
                    settings.mWeapon.mName = weapon.mName;
                    settings.mWeapon.mMass = real ? weapon.mRealMass : 1.0f;
                    settings.mWeapon.mLength = weapon.mLength;
                    settings.mWeapon.mTip = weapon.mTip;
                    ArmSim sim(settings);
                    if (!sim.valid())
                    {
                        std::cerr << sim.error() << std::endl;
                        return 1;
                    }
                    if (line == 0 && !real && std::string(weapon.mName) == "iron longsword")
                    {
                        // The hand's frame check: the weapon's centre where the
                        // hand's rotation says it is.
                        const auto chainNow = sim.step(HandDriveGoal{ sim.grip(), JPH::Vec3::sZero(),
                                                           sim.handRotation(), JPH::Vec3::sZero(), std::nullopt },
                            HandDriveGains{});
                        const JPH::Vec3 actual(chainNow.mChain.mBody[3].mCenterOfMass - chainNow.mChain.mGrip);
                        const JPH::Vec3 predicted = chainNow.mChain.mHandRotation
                            * (ArmSim::gripOffset() + sim.bladeAxis() * (weapon.mTip - 0.5f * weapon.mLength));
                        std::cerr << "frame check: actual " << actual.GetX() << ',' << actual.GetY() << ','
                                  << actual.GetZ() << " predicted " << predicted.GetX() << ',' << predicted.GetY()
                                  << ',' << predicted.GetZ() << std::endl;
                        // The rotation drive alone: the grip held, a 30 deg turn asked.
                        if (std::getenv("OPENMW_ARM_DEBUG"))
                        {
                            const JPH::RVec3 holdAt = sim.grip();
                            const JPH::Quat want
                                = (JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.52f) * sim.handRotation()).Normalized();
                            HandDriveGains only;
                            only.mStrength = strengthScale;
                            for (int i = 0; i < 120; ++i)
                            {
                                const auto s = sim.step(
                                    HandDriveGoal{ holdAt, JPH::Vec3::sZero(), want, JPH::Vec3::sZero(), std::nullopt },
                                    only);
                                const JPH::Quat d = (want * s.mHandRotation.Conjugated()).Normalized();
                                if (i % 10 == 0)
                                    std::cerr << "rot only t=" << i / 60.0f << " err="
                                              << 2.0f * std::acos(std::clamp(std::abs(d.GetW()), 0.0f, 1.0f)) * 57.2958f
                                              << " pos=" << JPH::Vec3(s.mGrip - holdAt).Length()
                                              << " w=" << s.mChain.mHandAngularVelocity.Length() << std::endl;
                            }
                        }
                    }
                    WheelPlanner planner;
                    WheelStriker striker;
                    striker.mAxes = { sim.bladeAxis(), sim.edgeAxis() };
                    striker.mBladeAxis = sim.bladeAxis();
                    striker.mTip = weapon.mTip;
                    striker.mWeight = weapon.mWeight;
                    striker.mTwoHanded = weapon.mTwoHanded;
                    striker.mShort = weapon.mShort;
                    WheelActor actor;
                    WheelHooks hooks;
                    const float angle = static_cast<float>(line) * 0.25f * 3.14159265f;
                    hooks.mClickLine = [angle] { return angle; };
                    HandDriveGains gains = handDriveGains(weapon.mWeight, 1.0f);
                    gains.mStrength *= strengthScale;
                    gains.mRotationTorque = std::getenv("OPENMW_ARM_ROT_TORQUE") != nullptr;
                    gains.mComputeRotation = std::getenv("OPENMW_ARM_ASSIST") != nullptr;
                    if (frequency > 0.0f)
                    {
                        const float scale = frequency / 6.0f;
                        gains.mFrequency *= scale;
                        gains.mRotationFrequency *= scale;
                    }
                    const JPH::Quat toWorld = sim.bodyToWorld();
                    std::optional<HandDriveGoal> previous;
                    const auto goalFrom = [&](const ChamberPose& pose) {
                        HandDriveGoal goal;
                        goal.mGrip = sim.centre() + JPH::RVec3(toWorld * pose.mHand);
                        goal.mRotation = (toWorld * pose.mRotation).Normalized();
                        goal.mPole = (toWorld * pose.mPole).NormalizedOr(JPH::Vec3::sAxisX());
                        goal.mBladeAxis = sim.bladeAxis();
                        if (previous)
                        {
                            goal.mGripVelocity = JPH::Vec3(goal.mGrip - previous->mGrip) / dt;
                            JPH::Quat delta = (goal.mRotation * previous->mRotation.Conjugated()).Normalized();
                            if (delta.GetW() < 0.0f)
                                delta = -delta;
                            JPH::Vec3 axis;
                            float turn = 0.0f;
                            delta.GetAxisAngle(axis, turn);
                            goal.mAngularVelocity = turn > 1e-6f ? axis * (turn / dt) : JPH::Vec3::sZero();
                        }
                        previous = goal;
                        return goal;
                    };
                    const bool posed = std::getenv("OPENMW_ARM_POSED") != nullptr;
                    const auto stepArm = [&](const HandDriveGoal& goal) {
                        if (!posed)
                            return sim.step(goal, gains);
                        HandDriveGains feed = gains;
                        feed.mStrength = 1.0f;
                        if (const char* text = std::getenv("OPENMW_ARM_PUSH_HZ"))
                            feed.mFrequency = feed.mRotationFrequency = static_cast<float>(std::atof(text));
                        else
                            feed.mFrequency = 0.0f; // gravity support only
                        return sim.stepPosed(goal, feed, strengthScale);
                    };
                    const auto plan = [&](bool held) {
                        WheelInput input;
                        input.mDt = dt;
                        input.mHeld = held;
                        input.mArmLength = sim.armLength();
                        WheelEvents events;
                        return planner.step(input, striker, actor, hooks, nullptr, events);
                    };
                    // Held in the guard 1.5 s.
                    float guardErr = 0.0f, guardRot = 0.0f;
                    for (int i = 0; i < 90; ++i)
                    {
                        const auto step = stepArm(goalFrom(plan(false)));
                        guardErr = static_cast<float>((step.mGrip - previous->mGrip).Length());
                        guardRot = std::acos(std::clamp((previous->mRotation * sim.bladeAxis())
                                                             .Dot(step.mHandRotation * sim.bladeAxis()),
                                                 -1.0f, 1.0f))
                            * 57.2958f;
                        if (std::getenv("OPENMW_ARM_DEBUG") && line == 0 && !real && i % 6 == 0)
                            std::cerr << weapon.mName << " guard t=" << i / 60.0f << " pos_err=" << guardErr
                                      << " rot_err=" << guardRot << " util=" << step.mDrive.mUtilization[0] << ','
                                      << step.mDrive.mUtilization[1] << ',' << step.mDrive.mUtilization[2]
                                      << " hand_w=" << step.mChain.mHandAngularVelocity.Length() << " | "
                                      << sim.jointReport() << std::endl;
                    }
                    // The click and its attack.
                    double sumSq = 0.0;
                    int counted = 0, saturated = 0;
                    float maxErr = 0.0f, midErr = -1.0f, tipSpeed = 0.0f, rotErr = 0.0f, bladeMid = -1.0f;
                    double bladeSq = 0.0, solvedSq = 0.0;
                    int solvedCount = 0;
                    float solvedReach = 0.0f;
                    std::array<float, 3> util{};
                    std::optional<JPH::RVec3> lastTip;
                    bool attacked = false;
                    int afterGuard = 0;
                    float settleErr = 0.0f;
                    for (int i = 0; i < 400; ++i)
                    {
                        const bool held = i < 2;
                        const HandDriveGoal goal = goalFrom(plan(held));
                        const auto step = stepArm(goal);
                        const float err = static_cast<float>((step.mGrip - goal.mGrip).Length());
                        const WheelStage stage = planner.stage();
                        if (lastTip)
                            tipSpeed = std::max(tipSpeed, static_cast<float>((step.mTip - *lastTip).Length()) / dt);
                        lastTip = step.mTip;
                        if (stage == WheelStage::Wind || stage == WheelStage::Cut)
                        {
                            attacked = true;
                            sumSq += err * err;
                            ++counted;
                            maxErr = std::max(maxErr, err);
                            if (midErr < 0.0f && stage == WheelStage::Cut && planner.flow().mU >= 1.5f)
                                midErr = err;
                            const float bladeErr
                                = std::acos(std::clamp((goal.mRotation * sim.bladeAxis()).Dot(step.mHandRotation * sim.bladeAxis()),
                                      -1.0f, 1.0f))
                                * 57.2958f;
                            rotErr = std::max(rotErr, bladeErr);
                            if (step.mSolvedBladeError >= 0.0f)
                            {
                                solvedSq += step.mSolvedBladeError * step.mSolvedBladeError;
                                solvedReach = std::max(solvedReach, step.mSolvedReachError);
                                ++solvedCount;
                            }
                            bladeSq += bladeErr * bladeErr;
                            if (bladeMid < 0.0f && stage == WheelStage::Cut && planner.flow().mU >= 1.5f)
                                bladeMid = bladeErr;
                            bool full = false;
                            for (std::size_t j = 0; j < 3; ++j)
                            {
                                util[j] = std::max(util[j], step.mDrive.mUtilization[j]);
                                full = full || step.mDrive.mUtilization[j] >= 1.0f;
                            }
                            saturated += full ? 1 : 0;
                        }
                        if (attacked && stage == WheelStage::Guard && ++afterGuard >= 30)
                        {
                            settleErr = err;
                            break;
                        }
                    }
                    out << weapon.mName << ',' << (real ? "real" : "game") << ',' << strengthScale << ',' << line << ','
                        << guardErr << ',' << guardRot << ','
                        << (counted ? std::sqrt(sumSq / counted) : 0.0) << ',' << maxErr << ',' << midErr << ','
                        << tipSpeed << ',' << rotErr << ',' << (counted ? std::sqrt(bladeSq / counted) : 0.0) << ',' << bladeMid
                        << ',' << (solvedCount ? std::sqrt(solvedSq / solvedCount) : -1.0) << ',' << solvedReach << ',' << util[0] << ',' << util[1] << ',' << util[2] << ','
                        << (counted ? static_cast<float>(saturated) / counted : 0.0f) << ',' << settleErr << '\n';
                }
        std::cerr << "wrote " << outPath << std::endl;
        return 0;
    }
}

int main(int argc, char** argv)
{
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory;
    JPH::RegisterTypes();
    int result = 1;
    const std::string mode = argc >= 2 ? argv[1] : "";
    if (mode == "course" && argc >= 4)
        result = runCourse(argv[2], argv[3], argc >= 5 ? argv[4] : "", argc >= 6 ? argv[5] : "");
    else if (mode == "lead" && argc >= 5)
        result = runLead(argv[2], argv[3], argc >= 5 ? argv[4] : "");
    else if (mode == "arm" && argc >= 5)
        result = runArm(argv[2], argv[3], argv[4], argc >= 6 ? static_cast<float>(std::atof(argv[5])) : 0.0f,
            argc >= 7 ? static_cast<float>(std::atof(argv[6])) : 1.0f);
    else if (mode == "getup" && argc >= 4)
    {
        GetUpPolicy::gDataDir = argc >= 6 ? argv[5] : "";
        result = runGetUp(argv[2], argv[3],
            argc >= 5 ? std::atoi(argv[4]) : std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 2));
    }
    else
        std::cerr << "usage: ragdollcore-trainer course DATA_DIR OUT.log [SKELETON_DUMP [CLIP_TRACK]] | lead DATA_DIR SKELETON_DUMP CLIP_TRACK | getup OUT_DIR HH:MM [threads [DATA_DIR]] | arm DATA_DIR SKELETON_DUMP OUT.csv [HZ [STRENGTH]]\n";
    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
    return result;
}
