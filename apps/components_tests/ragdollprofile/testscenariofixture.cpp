#include <components/ragdollprofile/loader.hpp>
#include <components/vfs/filesystemarchive.hpp>
#include <components/vfs/manager.hpp>

#include <ragdollcore/profile.hpp>
#include <ragdollcore/replay.hpp>
#include <ragdollcore/scenario.hpp>
#include <ragdollcore/unitmodel.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <unordered_map>

namespace
{
    RagdollCore::RagdollProfile loadAuthoredHumanoid()
    {
        const std::filesystem::path dataPath = std::filesystem::path{ OPENMW_PROJECT_SOURCE_DIR } / "files" / "data";
        VFS::Manager vfs;
        vfs.addArchive(std::make_unique<VFS::FileSystemArchive>(dataPath));
        vfs.buildIndex();
        const auto loaded = RagdollProfile::load(vfs, VFS::Path::NormalizedView("ragdoll/humanoid-v1.yaml"));
        EXPECT_TRUE(loaded.isValid());
        for (const RagdollProfile::LoadIssue& issue : loaded.mIssues)
            ADD_FAILURE() << issue.mPath << ": " << issue.mMessage;
        return loaded.mProfile.value_or(RagdollCore::RagdollProfile{});
    }

    // The offline scenario fixture is a code transcription of the authored
    // profile. Only two differences are permitted: the identifier and the
    // bone aliases (an adapter concern). Derived hinge axes already account
    // for bone rotations; elbow limits must match the native profile. After
    // applying exactly those substitutions the canonical profile hash must
    // match, so any other drift in either file fails this test.
    TEST(RagdollProfileScenarioFixtureTest, codeDefinedHumanoidMatchesAuthoredProfile)
    {
        RagdollCore::RagdollProfile authored = loadAuthoredHumanoid();
        const RagdollCore::RagdollProfile fixture = RagdollCore::makeScenarioHumanoidProfile();
        ASSERT_EQ(authored.mBodies.size(), fixture.mBodies.size());
        for (std::size_t i = 0; i < authored.mBodies.size(); ++i)
        {
            EXPECT_EQ(authored.mBodies[i].mName, fixture.mBodies[i].mName) << "body " << i;
            EXPECT_EQ(authored.mBodies[i].mBone, fixture.mBodies[i].mBone) << "body " << i;
        }

        authored.mId = fixture.mId;
        authored.mBoneAliases.clear();
        EXPECT_EQ(RagdollCore::hashProfile(authored), RagdollCore::hashProfile(fixture))
            << "makeScenarioHumanoidProfile() has drifted from files/data/ragdoll/humanoid-v1.yaml";

        // Prove the gate is strict: a single changed field must be detected.
        RagdollCore::RagdollProfile drifted = fixture;
        drifted.mActiveController.mGait.mNominalStepDuration += 0.01f;
        EXPECT_NE(RagdollCore::hashProfile(authored), RagdollCore::hashProfile(drifted));
        drifted = fixture;
        drifted.mBodies[16].mShape.mHalfExtents += JPH::Vec3(0.0f, 0.0f, 0.1f);
        EXPECT_NE(RagdollCore::hashProfile(authored), RagdollCore::hashProfile(drifted));
    }

    TEST(RagdollProfileScenarioFixtureTest, syntheticSkeletonSatisfiesAuthoredProfile)
    {
        RagdollCore::RagdollProfile authored = loadAuthoredHumanoid();
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto validation = RagdollCore::validateProfile(authored, skeleton);
        for (const auto& issue : validation.mIssues)
            ADD_FAILURE() << issue.mPath << ": " << issue.mMessage;
        EXPECT_TRUE(validation.isValid());
    }

    struct CapturedRig
    {
        std::optional<RagdollCore::Replay> mReplay;
        RagdollCore::RagdollProfile mProfile;
        RagdollCore::SkeletonDescriptor mSkeleton;
        std::unordered_map<std::string, int> mBodyIndices;
    };

    void loadCapturedRig(CapturedRig& rig)
    {
        using namespace RagdollCore;
        const char* path = std::getenv("OPENMW_RAGDOLL_GAIT_REPLAY");
        ASSERT_NE(path, nullptr) << "Set OPENMW_RAGDOLL_GAIT_REPLAY to a local powered capture";
        std::ifstream input(path, std::ios::binary | std::ios::ate);
        ASSERT_TRUE(input);
        const auto size = input.tellg();
        ASSERT_GT(size, 0);
        std::vector<std::byte> bytes(static_cast<std::size_t>(size));
        input.seekg(0);
        input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        ASSERT_TRUE(input);
        auto& replay = rig.mReplay;
        replay = deserializeReplay(bytes);
        ASSERT_TRUE(replay);
        auto& profile = rig.mProfile;
        profile = loadAuthoredHumanoid();
        ASSERT_EQ(replay->mMetadata.mProfileHash, hashProfile(profile));
        ASSERT_FALSE(replay->mFrames.empty());
        const auto initial = decodePoweredReplayInput(*replay, replay->mFrames.front());
        ASSERT_TRUE(initial);
        ASSERT_EQ(initial->mAnimationTargetPose.size(), profile.mBodies.size());
        auto& skeleton = rig.mSkeleton;
        skeleton.mId = "local-powered-target";
        auto& parents = rig.mBodyIndices;
        for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
        {
            const auto& body = profile.mBodies[i];
            int parent = -1;
            if (body.mParentBody)
            {
                const auto found = parents.find(*body.mParentBody);
                ASSERT_NE(found, parents.end());
                parent = found->second;
            }
            skeleton.mBones.push_back({ body.mBone, parent, initial->mAnimationTargetPose[i] });
            parents.emplace(body.mName, static_cast<int>(i));
        }
    }

    // Local content stays outside the source tree. This isolates the captured
    // pose and motor targets on flat ground; it does not reproduce the live
    // terrain, root frame, insertion velocity or controller history.
    void evaluateCapturedPoweredTargets(bool jointSupport, unsigned impulseDirection = 0,
        float groundFriction = 0.8f, bool meshGround = false)
    {
        using namespace RagdollCore;
        CapturedRig rig;
        ASSERT_NO_FATAL_FAILURE(loadCapturedRig(rig));
        const auto& replay = rig.mReplay;
        const auto& profile = rig.mProfile;
        const auto& skeleton = rig.mSkeleton;
        const auto& parents = rig.mBodyIndices;
        ScenarioScript frozen;
        frozen.mDurationSeconds = static_cast<float>(replay->mFrames.size()) * replay->mMetadata.mFixedTimeStep;
        frozen.mKeyframes.emplace_back();
        ScenarioScript recorded = frozen;
        recorded.mKeyframes.clear();
        for (const auto& frame : replay->mFrames)
        {
            const auto decoded = decodePoweredReplayInput(*replay, frame);
            ASSERT_TRUE(decoded);
            ASSERT_TRUE(decoded->mRequestedHorizontalVelocity.IsNearZero());
            ScenarioInputKeyframe keyframe;
            keyframe.mStep = frame.mStepIndex;
            keyframe.mMotorTarget = RagdollPose{ decoded->mAnimationTargetPose };
            recorded.mKeyframes.push_back(std::move(keyframe));
        }
        for (const auto [solverActuation, recordedTargets] : {
                 std::pair{ false, false }, std::pair{ false, true },
                 std::pair{ true, false }, std::pair{ true, true } })
        {
            const std::string name = std::string(jointSupport ? "authored-" : "")
                + (jointSupport
                    ? (solverActuation ? "joint-loads-" : "joint-pose-only-")
                    : (solverActuation ? "solver-" : ""))
                + (recordedTargets ? "recorded-targets" : "frozen-target")
                + (impulseDirection ? "-push-" + std::to_string(impulseDirection) : "");
            SCOPED_TRACE(name);
            ScenarioSettings settings;
            settings.mGroundFriction = groundFriction;
            settings.mTriangleMeshGround = meshGround;
            if (meshGround)
                settings.mRootPosition = JPH::RVec3(-301, 165, 1);
            settings.mFixedTimeStep = replay->mMetadata.mFixedTimeStep;
            settings.mUseJointSupportControl = jointSupport;
            settings.mUseSolverActuation = !jointSupport && solverActuation;
            auto script = recordedTargets ? recorded : frozen;
            if (impulseDirection)
            {
                const JPH::Vec3 directions[] = { JPH::Vec3::sAxisX(), -JPH::Vec3::sAxisX(),
                    JPH::Vec3::sAxisY(), -JPH::Vec3::sAxisY() };
                ASSERT_LE(impulseDirection, std::size(directions));
                ASSERT_GT(replay->mFrames.size(), 180u);
                // Pelvis shove after three seconds: 0.1 m/s whole-body COM
                // velocity change, with no pose, root or contact manipulation.
                script.mImpulses.push_back({ 180, 0,
                    directions[impulseDirection - 1] * settings.mTotalMass * UnitModel::fromMeters(0.1f) });
            }
            if (jointSupport)
                for (auto& keyframe : script.mKeyframes)
                {
                    keyframe.mTranslationActuationEnabled = solverActuation;
                    keyframe.mUprightTorqueEnabled = false;
                }
            const auto result = runRagdollScenario(skeleton, profile, settings, script);
            for (const auto& error : result.mErrors)
                ADD_FAILURE() << error;
            ASSERT_TRUE(result.completed());
            const auto summary = summarizeScenarioInvariants(result);
            float maxAnchor = 0;
            float maxRootForce = 0, maxRootMoment = 0;
            float maxFootDrift = 0, maxMotorUtilization = 0;
            bool landed = false;
            std::size_t airborneAfterLanding = 0;
            for (const auto& frame : result.mFrames)
            {
                maxAnchor = std::max(maxAnchor, frame.mMaxAnchorSeparation);
                if (frame.mStep >= 60)
                    for (const auto foot : { parents.at("left_foot"), parents.at("right_foot") })
                    {
                        auto delta = JPH::Vec3(frame.mBodies[foot].mWorldPosition
                            - result.mFrames[60].mBodies[foot].mWorldPosition);
                        delta.SetZ(0);
                        maxFootDrift = std::max(maxFootDrift, delta.Length());
                    }
                for (const auto utilization : frame.mJointSupportMotorUtilizations)
                    maxMotorUtilization = std::max(maxMotorUtilization, utilization);
                if (frame.mSupport.hasGroundContact())
                    landed = true;
                else if (landed)
                    ++airborneAfterLanding;
                if (frame.mJointSupport)
                {
                    maxRootForce = std::max(maxRootForce, frame.mJointSupport->mRootForceResidual.Length());
                    maxRootMoment = std::max(maxRootMoment, frame.mJointSupport->mRootMomentResidual.Length());
                }
            }
            std::cout << "CAPTURED_IDLE " << name << " frames=" << summary.mFrameCount
                      << " ground_friction=" << groundFriction
                      << " mesh_ground=" << meshGround
                      << " falling=" << summary.mFallingFrames + summary.mFallenFrames
                      << " min_up=" << summary.mMinPelvisUpDot << " airborne=" << summary.mAirborneFrames
                      << " free_caps=" << summary.mFreeVelocityLimitedFrames
                      << " solver_caps=" << summary.mSolverVelocityLimitedFrames
                      << " height_target=" << result.mTargetPelvisHeight
                      << " max_anchor=" << maxAnchor << " max_speed=" << summary.mMaxBodySpeed
                      << " airborne_after_landing=" << airborneAfterLanding
                      << " max_slide=" << summary.mMaxStanceFootSlide
                      << " foot_drift=" << maxFootDrift << " motor_utilization=" << maxMotorUtilization
                      << " final_com_speed=" << result.mFrames.back().mCenterOfMassVelocity.Length()
                      << " net_force=" << summary.mMaxAppliedNetForce
                      << " net_moment=" << summary.mMaxNetActuationMoment
                      << " plan_root_force=" << maxRootForce << " plan_root_moment=" << maxRootMoment << std::endl;
            if (const char* directory = std::getenv("OPENMW_RAGDOLL_TRACE_DIR"))
            {
                std::filesystem::create_directories(directory);
                std::ofstream trace(std::filesystem::path(directory) / (name + ".csv"));
                ASSERT_TRUE(trace);
                trace << formatScenarioTraceCsv(result);
                ASSERT_TRUE(trace);
                if (jointSupport)
                {
                    std::ofstream joints(std::filesystem::path(directory) / (name + "-joints.csv"));
                    ASSERT_TRUE(joints);
                    std::ofstream contacts(std::filesystem::path(directory) / (name + "-contacts.csv"));
                    ASSERT_TRUE(contacts);
                    contacts << "step,body,x,y,z,nx,ny,nz,depth,normal_impulse\n";
                    contacts << std::setprecision(9);
                    for (const auto& frame : result.mFrames)
                        for (const auto& contact : frame.mContacts)
                            contacts << frame.mStep << ',' << contact.mBodyIndex
                                     << ',' << contact.mPosition.GetX() << ',' << contact.mPosition.GetY()
                                     << ',' << contact.mPosition.GetZ() << ',' << contact.mNormal.GetX()
                                     << ',' << contact.mNormal.GetY() << ',' << contact.mNormal.GetZ()
                                     << ',' << contact.mPenetrationDepth << ',' << contact.mNormalImpulse.value_or(-1) << '\n';
                    ASSERT_TRUE(contacts);
                    joints << "step,body,name,load_x,load_y,load_z,torque_x,torque_y,torque_z,utilization,cap\n";
                    joints << std::setprecision(9);
                    for (const auto& frame : result.mFrames)
                    {
                        if (!frame.mJointSupport || frame.mJointSupportMotorAngularImpulses.empty())
                            continue;
                        for (std::size_t i = 1; i < profile.mBodies.size(); ++i)
                        {
                            const auto load = frame.mJointSupport->mJointTorques[i];
                            const auto torque = frame.mJointSupportMotorAngularImpulses[i] / settings.mFixedTimeStep;
                            joints << frame.mStep << ',' << i << ',' << profile.mBodies[i].mName
                                   << ',' << load.GetX() << ',' << load.GetY() << ',' << load.GetZ()
                                   << ',' << torque.GetX() << ',' << torque.GetY() << ',' << torque.GetZ()
                                   << ',' << frame.mJointSupportMotorUtilizations[i]
                                   << ',' << profile.mBodies[i].mConstraint->mMotor.mMaximumTorque << '\n';
                        }
                    }
                    ASSERT_TRUE(joints);
                }
            }
            if (jointSupport && !solverActuation)
            {
                // Identical servo and limits, with only the load bias removed.
                EXPECT_GT(summary.mFallingFrames + summary.mFallenFrames, 0u);
                EXPECT_LT(summary.mMinPelvisUpDot, 0.9f);
                continue;
            }
            if (jointSupport)
            {
                const auto repeated = runRagdollScenario(skeleton, profile, settings, script);
                ASSERT_TRUE(repeated.completed());
                EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, repeated));
                EXPECT_TRUE(landed);
                EXPECT_EQ(airborneAfterLanding, 0u);
                EXPECT_LE(maxAnchor, 2.5f);
                EXPECT_LT(summary.mMaxBodySpeed, profile.mMotionTransfer.mMaximumLinearSpeed);
                EXPECT_LT(summary.mMaxStanceFootSlide, 1.0f);
                EXPECT_LT(summary.mMaxAppliedNetForce, 1e-3f);
                EXPECT_LT(summary.mMaxNetActuationMoment, 1e-3f);
                EXPECT_LT(maxFootDrift, profile.mActiveController.mSupportMargin);
                EXPECT_LT(result.mFrames.back().mCenterOfMassVelocity.Length(), UnitModel::fromMeters(0.1f));
                EXPECT_TRUE(result.mFrames.back().mSupport.mCenterOfMassSupported);
            }
            EXPECT_EQ(summary.mFallingFrames + summary.mFallenFrames, 0u);
            EXPECT_GE(summary.mMinPelvisUpDot, 0.9f);
            EXPECT_EQ(summary.mFreeVelocityLimitedFrames, 0u);
            EXPECT_EQ(summary.mSolverVelocityLimitedFrames, 0u);
        }
    }

    // Private owned-content diagnostic: source validation precedes every
    // physical comparison. A successful import or repeat is not a gait pass.
    TEST(RagdollProfileScenarioFixtureTest, DISABLED_comparesOwnedWalkReferenceOnCapturedRig)
    {
        using namespace RagdollCore;
        CapturedRig rig;
        ASSERT_NO_FATAL_FAILURE(loadCapturedRig(rig));
        const char* path = std::getenv("OPENMW_RAGDOLL_GAIT_CYCLE");
        ASSERT_NE(path, nullptr);
        std::ifstream input(path);
        ASSERT_TRUE(input);
        std::vector<RagdollPose> cycle;
        std::vector<bool> seen;
        std::size_t samplesRead = 0;
        for (std::string line; std::getline(input, line);)
        {
            const auto row = YAML::Load(line);
            const auto scenario = row["scenario"].as<std::string>();
            if (scenario == "authored_walk_cycle_summary")
            {
                ASSERT_TRUE(cycle.empty());
                ASSERT_TRUE(row["valid"].as<bool>());
                ASSERT_EQ(row["group"].as<std::string>(), "walkforwardhh");
                ASSERT_EQ(row["profile_hash"].as<std::uint64_t>(), hashProfile(rig.mProfile));
                ASSERT_EQ(row["initial_pose_replay_seed"].as<std::uint64_t>(), rig.mReplay->mMetadata.mSeed);
                ASSERT_LT(row["maximum_seam_translation"].as<float>(), 0.05f);
                ASSERT_GT(row["minimum_seam_rotation_dot"].as<float>(), 0.99999f);
                ASSERT_EQ(row["sample_count"].as<std::size_t>(), 180u);
                ASSERT_EQ(row["body_count"].as<std::size_t>(), rig.mProfile.mBodies.size());
                cycle.resize(180);
                for (auto& pose : cycle)
                    pose.mBodyTransforms.resize(rig.mProfile.mBodies.size());
                seen.resize(cycle.size() * rig.mProfile.mBodies.size());
            }
            else if (scenario == "authored_walk_sample")
            {
                const auto frame = row["frame"].as<std::size_t>();
                const auto body = row["body"].as<std::size_t>();
                ASSERT_LT(frame, cycle.size());
                ASSERT_LT(body, rig.mProfile.mBodies.size());
                ASSERT_EQ(row["name"].as<std::string>(), rig.mProfile.mBodies[body].mName);
                const auto index = frame * rig.mProfile.mBodies.size() + body;
                ASSERT_FALSE(seen[index]);
                seen[index] = true;
                ++samplesRead;
                auto& transform = cycle[frame].mBodyTransforms[body];
                transform.mTranslation = JPH::Vec3(row["tx"].as<float>(), row["ty"].as<float>(), row["tz"].as<float>());
                transform.mRotation = JPH::Quat(row["qx"].as<float>(), row["qy"].as<float>(),
                    row["qz"].as<float>(), row["qw"].as<float>());
                ASSERT_TRUE(std::isfinite(transform.mTranslation.GetX()));
                ASSERT_TRUE(std::isfinite(transform.mTranslation.GetY()));
                ASSERT_TRUE(std::isfinite(transform.mTranslation.GetZ()));
                ASSERT_TRUE(transform.mRotation.IsNormalized());
            }
        }
        ASSERT_FALSE(cycle.empty());
        ASSERT_EQ(samplesRead, seen.size());
        // This clip begins in right stance. Its second half starts left stance,
        // matching the core's phase origin. Never advance a wall-clock here.
        std::rotate(cycle.begin(), cycle.begin() + cycle.size() / 2, cycle.end());
        for (const bool solver : { false, true })
            for (const bool authored : { false, true })
            for (const std::string_view isolation : { "full", "no-swing-force", "no-prelift", "no-actuation" })
            {
                const std::string name = std::string("owned-walk-") + (authored ? "authored" : "frozen")
                    + (solver ? "-solver" : "-free") + "-" + std::string(isolation);
                SCOPED_TRACE(name);
                const auto& profile = rig.mProfile;
                ScenarioSettings settings;
                settings.mFixedTimeStep = rig.mReplay->mMetadata.mFixedTimeStep;
                settings.mGroundFriction = 0.2f;
                settings.mTriangleMeshGround = true;
                settings.mRootPosition = JPH::RVec3(-301, 165, 1);
                settings.mUseSolverActuation = solver;
                if (isolation == "no-prelift")
                    settings.mPreLiftSwingUnloadFraction = 0;
                ScenarioScript script;
                script.mDurationSeconds = 7;
                script.mKeyframes.emplace_back();
                auto& walk = script.mKeyframes.emplace_back();
                walk.mStep = 60;
                walk.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
                if (authored)
                    script.mGaitPoseCycle = cycle;
                for (auto& keyframe : script.mKeyframes)
                {
                    keyframe.mSwingFootActuationEnabled = isolation != "no-swing-force" && isolation != "no-actuation";
                    if (isolation == "no-actuation")
                    {
                        keyframe.mMotorsEnabled = false;
                        keyframe.mTranslationActuationEnabled = false;
                        keyframe.mUprightTorqueEnabled = false;
                    }
                }
                const auto result = runRagdollScenario(rig.mSkeleton, profile, settings, script);
                for (const auto& error : result.mErrors)
                    ADD_FAILURE() << error;
                ASSERT_TRUE(result.completed());
                const auto repeated = runRagdollScenario(rig.mSkeleton, profile, settings, script);
                ASSERT_TRUE(repeated.completed());
                EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, repeated));
                if (const char* directory = std::getenv("OPENMW_RAGDOLL_TRACE_DIR"))
                {
                    std::filesystem::create_directories(directory);
                    std::ofstream trace(std::filesystem::path(directory) / (name + ".csv"));
                    ASSERT_TRUE(trace);
                    trace << formatScenarioTraceCsv(result);
                    ASSERT_TRUE(trace);
                }
                const auto summary = summarizeScenarioInvariants(result);
                const auto walkingAirborne = std::count_if(result.mFrames.begin(), result.mFrames.end(),
                    [](const auto& frame) { return frame.mStep >= 60 && !frame.mSupport.hasGroundContact(); });
                std::cout << "OWNED_WALK " << name
                          << " step_width=" << profile.mActiveController.mGait.mNominalStepWidth
                          << " run_profile=" << hashProfile(profile)
                          << " falling=" << summary.mFallingFrames + summary.mFallenFrames
                          << " setup_airborne=" << summary.mAirborneFrames - walkingAirborne
                          << " walking_airborne=" << walkingAirborne
                          << " left=" << summary.mSteps.mLeftStanceCount
                          << " right=" << summary.mSteps.mRightStanceCount
                          << " timeouts=" << summary.mSteps.mTimedOutPhases
                          << " speed=" << summary.mFinalHalfSecondAchievedSpeed
                          << " slide=" << summary.mMaxStanceFootSlide
                          << " free_caps=" << summary.mFreeVelocityLimitedFrames
                          << " solver_caps=" << summary.mSolverVelocityLimitedFrames << std::endl;
                // Counts include stance entry, not only completed steps. Show
                // the existing physical milestones so a stalled first swing
                // cannot be mistaken for a successful step or handover.
                const auto stepLabel = [](const std::optional<std::uint64_t>& step) {
                    return step ? std::to_string(*step) : std::string("none");
                };
                for (const auto& phase : summarizeScenarioGaitPhases(result))
                    std::cout << "OWNED_WALK_PHASE " << name
                              << " stance=" << (phase.mPhase == GaitPhase::LeftStance ? "left" : "right")
                              << " start=" << phase.mStanceStartedStep
                              << " ready=" << stepLabel(phase.mStanceReadyStep)
                              << " lift=" << stepLabel(phase.mLiftOffStep)
                              << " contact=" << stepLabel(phase.mContactCandidateStep)
                              << " transfer=" << stepLabel(phase.mStanceTransferStep)
                              << " timeout=" << stepLabel(phase.mPhaseTimeoutStep)
                              << " closest_step=" << stepLabel(phase.mClosestApproachStep)
                              << " closest_error=" << (phase.mClosestApproachStep
                                  ? std::to_string(phase.mClosestApproachError) : "none") << std::endl;
                if (isolation == "no-actuation")
                {
                    EXPECT_GT(summary.mFallingFrames + summary.mFallenFrames, 0u);
                    continue;
                }
                EXPECT_EQ(summary.mFallingFrames + summary.mFallenFrames, 0u);
                EXPECT_EQ(walkingAirborne, 0);
                EXPECT_EQ(summary.mSteps.mTimedOutPhases, 0u);
                EXPECT_GE(summary.mSteps.mLeftStanceCount, 2u);
                EXPECT_GE(summary.mSteps.mRightStanceCount, 2u);
                EXPECT_GE(summary.mFinalHalfSecondAchievedSpeed, 30.0f);
                EXPECT_LE(summary.mMaxStanceFootSlide, 1.5f);
                EXPECT_EQ(summary.mFreeVelocityLimitedFrames, 0u);
                EXPECT_EQ(summary.mSolverVelocityLimitedFrames, 0u);
            }
    }

    TEST(RagdollProfileScenarioFixtureTest, DISABLED_evaluatesCapturedPoweredTargetsOnFlatGround)
    {
        evaluateCapturedPoweredTargets(false);
    }

    TEST(RagdollProfileScenarioFixtureTest, DISABLED_evaluatesCapturedJointSupportWithAuthoredLimits)
    {
        for (unsigned impulseDirection = 0; impulseDirection <= 4; ++impulseDirection)
            evaluateCapturedPoweredTargets(true, impulseDirection);
    }

    TEST(RagdollProfileScenarioFixtureTest, DISABLED_evaluatesCapturedJointSupportWithLiveGroundFriction)
    {
        // Ordinary live static bodies retain Jolt's default friction. Keep the
        // same physical gates while testing this material separately from the
        // original 0.8-friction offline floor.
        for (unsigned impulseDirection = 0; impulseDirection <= 4; ++impulseDirection)
            evaluateCapturedPoweredTargets(true, impulseDirection, 0.2f);
    }

    TEST(RagdollProfileScenarioFixtureTest, DISABLED_evaluatesCapturedJointSupportWithLiveMeshFloor)
    {
        for (unsigned impulseDirection = 0; impulseDirection <= 4; ++impulseDirection)
            evaluateCapturedPoweredTargets(true, impulseDirection, 0.2f, true);
    }
}
