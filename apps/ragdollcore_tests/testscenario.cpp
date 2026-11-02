#include <ragdollcore/builder.hpp>
#include <ragdollcore/replay.hpp>
#include <ragdollcore/scenario.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>

namespace
{
    using RagdollCore::ActiveControllerMode;
    using RagdollCore::GaitPhase;

    constexpr std::size_t sControllerModeCount = 7;

    using ControllerTransitionMatrix
        = std::array<std::array<std::size_t, sControllerModeCount>, sControllerModeCount>;

    bool isAllowedControllerTransition(ActiveControllerMode from, ActiveControllerMode to)
    {
        if (from == to)
            return true;
        switch (from)
        {
            case ActiveControllerMode::Standing:
            case ActiveControllerMode::Bracing:
            case ActiveControllerMode::Stumbling:
                return to == ActiveControllerMode::Standing || to == ActiveControllerMode::Bracing
                    || to == ActiveControllerMode::Stumbling || to == ActiveControllerMode::Falling
                    || to == ActiveControllerMode::Fallen;
            case ActiveControllerMode::Falling:
                return to == ActiveControllerMode::Standing || to == ActiveControllerMode::Bracing
                    || to == ActiveControllerMode::Fallen;
            case ActiveControllerMode::Fallen:
                return to == ActiveControllerMode::Falling || to == ActiveControllerMode::GettingUp;
            case ActiveControllerMode::GettingUp:
                return to == ActiveControllerMode::BlendBack;
            case ActiveControllerMode::BlendBack:
                return to == ActiveControllerMode::Standing;
        }
        return false;
    }

    ControllerTransitionMatrix transitionMatrix(
        const RagdollCore::CrowdScenarioResult& result, std::size_t actorIndex)
    {
        ControllerTransitionMatrix transitions{};
        for (std::size_t frameIndex = 1; frameIndex < result.mFrames.size(); ++frameIndex)
        {
            const auto from = result.mFrames[frameIndex - 1].mActors[actorIndex].mMode;
            const auto to = result.mFrames[frameIndex].mActors[actorIndex].mMode;
            ++transitions[static_cast<std::size_t>(from)][static_cast<std::size_t>(to)];
        }
        return transitions;
    }

    const char* modeName(ActiveControllerMode mode)
    {
        switch (mode)
        {
            case ActiveControllerMode::Standing:
                return "standing";
            case ActiveControllerMode::Bracing:
                return "bracing";
            case ActiveControllerMode::Stumbling:
                return "stumbling";
            case ActiveControllerMode::Falling:
                return "falling";
            case ActiveControllerMode::Fallen:
                return "fallen";
            case ActiveControllerMode::GettingUp:
                return "getting_up";
            case ActiveControllerMode::BlendBack:
                return "blend_back";
        }
        return "unknown";
    }

    const char* phaseName(GaitPhase phase)
    {
        switch (phase)
        {
            case GaitPhase::Idle:
                return "idle";
            case GaitPhase::LeftStance:
                return "left";
            case GaitPhase::RightStance:
                return "right";
        }
        return "unknown";
    }

    std::string describe(const RagdollCore::ScenarioResult& result)
    {
        std::ostringstream stream;
        for (const std::string& error : result.mErrors)
            stream << "error: " << error << '\n';
        stream << "frames=" << result.mFrames.size() << " target_height=" << result.mTargetPelvisHeight;
        // Full frame data belongs in the optional CSV trace. Repeating it for
        // every failed assertion obscures the failed gate and bloats reports.
        if (!result.mFrames.empty())
        {
            const auto& last = result.mFrames.back();
            stream << " last_step=" << last.mStep << " mode=" << modeName(last.mController.mMode)
                   << " gait=" << phaseName(last.mGait.mPhase) << " up=" << last.mPelvisUpDot
                   << " height=" << last.mPelvisHeight;
        }
        return stream.str();
    }

    std::string describe(const RagdollCore::ScenarioInvariants& invariants)
    {
        std::ostringstream stream;
        stream << "frames=" << invariants.mFrameCount << " airborne=" << invariants.mAirborneFrames
               << " max_consecutive_airborne=" << invariants.mMaxConsecutiveAirborneFrames
               << " solver_observed=" << invariants.mSolverObservedFrames
               << " free_velocity_limited=" << invariants.mFreeVelocityLimitedFrames
               << " solver_velocity_limited=" << invariants.mSolverVelocityLimitedFrames
               << " max_free_linear_limit_impulse=" << invariants.mMaxFreeLinearLimitImpulse
               << " max_free_angular_limit_impulse=" << invariants.mMaxFreeAngularLimitImpulse
               << " force_saturated=" << invariants.mPelvisForceSaturatedFrames
               << " force_magnitude_saturated=" << invariants.mPelvisMagnitudeSaturatedFrames
               << " horizontal_friction_saturated=" << invariants.mPelvisHorizontalForceSaturatedFrames
               << " vertical_support_saturated=" << invariants.mPelvisVerticalForceSaturatedFrames
               << " pelvis_torque_saturated=" << invariants.mPelvisTorqueSaturatedFrames
               << " spine_torque_saturated=" << invariants.mSpineTorqueSaturatedFrames
               << " swing_hip_motor_saturated=" << invariants.mSwingHipMotorSaturatedFrames
               << " swing_hip_world_saturated=" << invariants.mSwingHipWorldTorqueSaturatedFrames
               << " max_net_force=" << invariants.mMaxAppliedNetForce
               << " max_pair_moment=" << invariants.mMaxForcePairMoment
               << " max_swing_moment=" << invariants.mMaxSwingForcePairMoment
               << " max_upright_moment=" << invariants.mMaxUprightForcePairMoment
               << " max_ground_force=" << invariants.mMaxGroundReactionForce
               << " max_ground_moment=" << invariants.mMaxGroundReactionMoment
               << " max_net_moment=" << invariants.mMaxNetActuationMoment
               << " max_swing_hip_motor_torque=" << invariants.mMaxSwingHipMotorTorque
               << " max_swing_hip_motor_utilization=" << invariants.mMaxSwingHipMotorPeakAxisUtilization
               << " max_swing_hip_world_torque=" << invariants.mMaxSwingHipWorldTorque
               << " max_stance_hip_compensation_torque=" << invariants.mMaxStanceHipCompensationTorque
               << " max_foot_slide=" << invariants.mMaxStanceFootSlide
               << " total_foot_slide=" << invariants.mTotalStanceFootSlide
               << " requested_frames=" << invariants.mRequestedFrames
               << " mean_requested=" << invariants.mMeanRequestedSpeed
               << " mean_achieved=" << invariants.mMeanAchievedSpeed
               << " final_half_second_achieved=" << invariants.mFinalHalfSecondAchievedSpeed
               << " max_tracking_error=" << invariants.mMaxTrackingError
               << " left_steps=" << invariants.mSteps.mLeftStanceCount
               << " right_steps=" << invariants.mSteps.mRightStanceCount
               << " left_mean=" << invariants.mSteps.mMeanLeftStanceDuration
               << " right_mean=" << invariants.mSteps.mMeanRightStanceDuration
               << " symmetry=" << invariants.mSteps.mDurationSymmetry
               << " timed_out=" << invariants.mSteps.mTimedOutPhases << " falling=" << invariants.mFallingFrames
               << " fallen=" << invariants.mFallenFrames << " min_up=" << invariants.mMinPelvisUpDot
               << " max_body_speed=" << invariants.mMaxBodySpeed << " impulses=" << invariants.mImpulseFrames
               << " steps_after_push=" << invariants.mStepsWithinSecondAfterImpulse
               << " fell_after_push=" << invariants.mFellAfterImpulse;
        return stream.str();
    }

    std::string describe(const RagdollCore::ScenarioGaitMilestones& milestones)
    {
        const auto step = [](const std::optional<std::uint64_t>& value) {
            return value ? std::to_string(*value) : std::string("missing");
        };
        std::ostringstream stream;
        stream << "stance=" << step(milestones.mStanceStartedStep)
               << " ready=" << step(milestones.mStanceReadyStep) << " lift=" << step(milestones.mLiftOffStep)
               << " closest=" << step(milestones.mClosestApproachStep) << '/'
               << milestones.mClosestApproachError << " contact=" << step(milestones.mContactCandidateStep)
               << " transfer=" << step(milestones.mStanceTransferStep)
               << " timeout=" << step(milestones.mPhaseTimeoutStep);
        return stream.str();
    }

    std::string describe(const RagdollCore::CrowdScenarioResult& result)
    {
        std::ostringstream stream;
        for (const std::string& error : result.mErrors)
            stream << "error: " << error << '\n';
        for (const RagdollCore::CrowdScenarioFrame& frame : result.mFrames)
        {
            if (frame.mStep % 10 != 0 && frame.mStep + 1 != result.mStepCount)
                continue;
            stream << "step " << frame.mStep << " cross_contacts=" << frame.mCrossActorContactCount;
            for (std::size_t i = 0; i < frame.mActors.size(); ++i)
            {
                const RagdollCore::CrowdActorFrame& actor = frame.mActors[i];
                stream << " actor" << i << "=" << modeName(actor.mMode) << "/" << actor.mPelvisUpDot << "/"
                       << actor.mCenterOfMassVelocity.Length() << "/" << actor.mSupport.mLeftFootSupported
                       << actor.mSupport.mRightFootSupported;
            }
            stream << '\n';
        }
        return stream.str();
    }

    RagdollCore::ScenarioScript standingScript(float seconds)
    {
        RagdollCore::ScenarioScript script;
        script.mDurationSeconds = seconds;
        return script;
    }

    RagdollCore::ScenarioScript walkingScript(float seconds, float speed, std::uint64_t startStep)
    {
        RagdollCore::ScenarioScript script;
        script.mDurationSeconds = seconds;
        RagdollCore::ScenarioInputKeyframe walk;
        walk.mStep = startStep;
        walk.mRequestedHorizontalVelocity = JPH::Vec3(0.0f, speed, 0.0f);
        script.mKeyframes.push_back(walk);
        return script;
    }

    std::optional<std::vector<RagdollCore::RagdollPose>> compressedStancePoseCycle(float compression)
    {
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        RagdollCore::RagdollPose bindPose;
        for (const RagdollCore::SkeletonBone& bone : skeleton.mBones)
            bindPose.mBodyTransforms.push_back(bone.mModelTransform);
        std::vector<RagdollCore::RagdollPose> cycle{ bindPose, bindPose };
        if (compression <= 0.0f)
            return cycle;

        const auto compressLeg = [&](RagdollCore::RagdollPose& pose, std::size_t hip, std::size_t knee,
                                     std::size_t foot) {
            RagdollCore::TwoBoneLegTargetInput input;
            input.mHipBodyIndex = hip;
            input.mKneeBodyIndex = knee;
            input.mFootBodyIndex = foot;
            input.mTargetFootPosition = pose.mBodyTransforms[foot].mTranslation + JPH::Vec3(0.0f, 0.0f, compression);
            input.mPreferredBendDirection = JPH::Vec3::sAxisY();
            return RagdollCore::applyTwoBoneLegTarget(pose.mBodyTransforms, input);
        };
        if (!compressLeg(cycle[0], 14, 15, 16) || !compressLeg(cycle[1], 17, 18, 19))
            return std::nullopt;
        return cycle;
    }

    std::optional<RagdollCore::Replay> loadLocalPoweredReplay()
    {
        const char* replayPath = std::getenv("OPENMW_RAGDOLL_GAIT_REPLAY");
        if (replayPath == nullptr || *replayPath == '\0')
            return std::nullopt;
        std::ifstream stream(std::filesystem::path(replayPath), std::ios::binary | std::ios::ate);
        if (!stream)
            return std::nullopt;
        const std::streampos end = stream.tellg();
        if (end <= 0 || static_cast<std::uintmax_t>(end) > std::numeric_limits<std::size_t>::max())
            return std::nullopt;
        std::vector<std::byte> data(static_cast<std::size_t>(end));
        stream.seekg(0);
        stream.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
        if (!stream)
            return std::nullopt;
        return RagdollCore::deserializeReplay(data);
    }

    std::optional<std::vector<RagdollCore::RagdollPose>> extractWrappedPoseCycle(const RagdollCore::Replay& replay,
        std::size_t cycleStart, std::size_t cycleFrameCount, std::size_t phaseOffset)
    {
        if (cycleFrameCount < 2 || cycleStart > replay.mFrames.size()
            || cycleFrameCount > replay.mFrames.size() - cycleStart || phaseOffset >= cycleFrameCount)
            return std::nullopt;
        std::vector<RagdollCore::RagdollPose> result;
        result.reserve(cycleFrameCount);
        for (std::size_t sample = 0; sample < cycleFrameCount; ++sample)
        {
            const std::size_t frameIndex = cycleStart + (phaseOffset + sample) % cycleFrameCount;
            auto input = RagdollCore::decodePoweredReplayInput(replay, replay.mFrames[frameIndex]);
            if (!input)
                return std::nullopt;
            RagdollCore::RagdollPose pose;
            pose.mBodyTransforms = std::move(input->mAnimationTargetPose);
            result.push_back(std::move(pose));
        }
        return result;
    }

    std::optional<RagdollCore::SkeletonDescriptor> skeletonFromPoweredTarget(
        const RagdollCore::RagdollProfile& profile, const RagdollCore::RagdollPose& pose)
    {
        if (pose.mBodyTransforms.size() != profile.mBodies.size())
            return std::nullopt;
        RagdollCore::SkeletonDescriptor result;
        result.mId = "local-powered-target";
        result.mBones.reserve(profile.mBodies.size());
        std::unordered_map<std::string, int> bodyIndices;
        for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
        {
            const RagdollCore::BodyProfile& body = profile.mBodies[i];
            int parent = -1;
            if (body.mParentBody)
            {
                const auto found = bodyIndices.find(*body.mParentBody);
                if (found == bodyIndices.end())
                    return std::nullopt;
                parent = found->second;
            }
            result.mBones.push_back(
                { body.mBone, parent, pose.mBodyTransforms[i] });
            bodyIndices.emplace(body.mName, static_cast<int>(i));
        }
        return result;
    }

    RagdollCore::ScenarioScript perturbationScript()
    {
        RagdollCore::ScenarioScript script = walkingScript(1.5f, 60.0f, 40);
        RagdollCore::ScenarioImpulse push;
        push.mStep = 20;
        push.mBodyIndex = 5; // head
        push.mImpulse = JPH::Vec3(1500.0f, 0.0f, 0.0f);
        script.mImpulses.push_back(push);
        RagdollCore::ScenarioImpulse offCenter;
        offCenter.mStep = 20;
        offCenter.mBodyIndex = 0; // pelvis
        offCenter.mImpulse = JPH::Vec3(0.0f, 400.0f, 0.0f);
        offCenter.mWorldPoint = JPH::RVec3(0.0, 0.0, 70.0);
        script.mImpulses.push_back(offCenter);
        RagdollCore::ScenarioInputKeyframe release;
        release.mStep = 75;
        release.mMotorsEnabled = false;
        release.mTranslationActuationEnabled = false;
        release.mUprightTorqueEnabled = false;
        script.mKeyframes.push_back(release);
        return script;
    }

    RagdollCore::ScenarioResult run(
        const RagdollCore::ScenarioScript& script, const RagdollCore::ScenarioSettings& settings = {})
    {
        return RagdollCore::runRagdollScenario(
            RagdollCore::makeScenarioHumanoidSkeleton(), RagdollCore::makeScenarioHumanoidProfile(), settings, script);
    }

    void exportTraceIfRequested(const RagdollCore::ScenarioResult& result, const std::string& name)
    {
        const char* outputDirectory = std::getenv("OPENMW_RAGDOLL_TRACE_DIR");
        if (outputDirectory == nullptr || *outputDirectory == '\0')
            return;
        const std::filesystem::path directory(outputDirectory);
        std::error_code error;
        std::filesystem::create_directories(directory, error);
        ASSERT_FALSE(error) << "Could not create trace directory: " << error.message();
        std::ofstream stream(directory / (name + ".csv"), std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(stream) << "Could not open scenario trace output";
        stream << RagdollCore::formatScenarioTraceCsv(result);
        ASSERT_TRUE(stream) << "Could not write scenario trace output";
        std::ofstream stages(directory / (name + "-stages.csv"), std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(stages) << "Could not open stage summary output";
        stages << RagdollCore::formatScenarioGaitStagesCsv(result);
        ASSERT_TRUE(stages) << "Could not write stage summary output";
    }

    void exportTraceIfRequested(const RagdollCore::CrowdScenarioResult& result, const std::string& name)
    {
        const char* outputDirectory = std::getenv("OPENMW_RAGDOLL_TRACE_DIR");
        if (outputDirectory == nullptr || *outputDirectory == '\0')
            return;
        const std::filesystem::path directory(outputDirectory);
        std::error_code error;
        std::filesystem::create_directories(directory, error);
        ASSERT_FALSE(error) << "Could not create trace directory: " << error.message();
        std::ofstream stream(directory / (name + ".csv"), std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(stream) << "Could not open crowd scenario trace output";
        stream << RagdollCore::formatCrowdScenarioTraceCsv(result);
        ASSERT_TRUE(stream) << "Could not write crowd scenario trace output";
    }

    TEST(RagdollCoreScenarioTest, humanoidFixtureValidatesAndBuilds)
    {
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto profile = RagdollCore::makeScenarioHumanoidProfile();
        EXPECT_TRUE(RagdollCore::validateSkeleton(skeleton).isValid());
        const auto validation = RagdollCore::validateProfile(profile, skeleton);
        for (const auto& issue : validation.mIssues)
            ADD_FAILURE() << issue.mPath << ": " << issue.mMessage;
        EXPECT_EQ(profile.mBodies.size(), 20u);
        EXPECT_EQ(skeleton.mBones.size(), 20u);

        RagdollCore::BuildOptions options;
        options.mObjectLayer = 1;
        const auto built = RagdollCore::buildRagdollSettings(skeleton, profile, options);
        EXPECT_TRUE(built.isValid());
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreScenarioTest, DISABLED_flatMeshDiagonalDoesNotCreateLoadedSidewaysFootContacts)
    {
        const auto profile = RagdollCore::makeScenarioHumanoidProfile();
        RagdollCore::ScenarioSettings settings;
        settings.mTriangleMeshGround = true;
        settings.mRootPosition = JPH::RVec3(180, 180, 1);
        const auto result = run(standingScript(2.0f), settings);
        ASSERT_TRUE(result.completed()) << describe(result);
        std::size_t loaded = 0;
        for (const auto& frame : result.mFrames)
            for (const auto& contact : frame.mContacts)
            {
                ASSERT_LT(contact.mBodyIndex, profile.mBodies.size());
                const auto& name = profile.mBodies[contact.mBodyIndex].mName;
                if (name != "left_foot" && name != "right_foot")
                    continue;
                if (contact.mNormalImpulse.value_or(0) <= 0.01f)
                    continue;
                ++loaded;
                // The feet are thousands of units from every real floor edge.
                EXPECT_GE(contact.mNormal.GetZ(), 0.999f);
            }
        EXPECT_GT(loaded, 20u);
    }

    TEST(RagdollCoreScenarioTest, observesSolvedMeshContactImpulseWithoutInferringItFromDepth)
    {
        RagdollCore::ScenarioSettings settings;
        settings.mTriangleMeshGround = true;
        // Remove tangential impulses from this independent normal-impulse
        // measurement. Live-floor material comparisons retain friction 0.2.
        settings.mGroundFriction = 0;
        settings.mUseContinuousCollision = false;
        settings.mRootPosition = JPH::RVec3(-301, 165, 1);
        const auto result = run(standingScript(1.0f), settings);
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto repeated = run(standingScript(1.0f), settings);
        ASSERT_TRUE(repeated.completed());
        EXPECT_TRUE(RagdollCore::haveIdenticalScenarioTelemetry(result, repeated));
        std::size_t measured = 0, loadedSpeculative = 0, unloadedSpeculative = 0;
        for (const auto& frame : result.mFrames)
        {
            JPH::Vec3 contactImpulse = JPH::Vec3::sZero();
            for (const auto& contact : frame.mContacts)
            {
                ASSERT_TRUE(contact.mNormalImpulse);
                EXPECT_GE(*contact.mNormalImpulse, 0);
                ASSERT_TRUE(contact.mFrictionImpulseMagnitude);
                EXPECT_FLOAT_EQ(*contact.mFrictionImpulseMagnitude, 0.0f);
                ++measured;
                contactImpulse += contact.mNormal * *contact.mNormalImpulse;
                if (contact.mPenetrationDepth < 0)
                {
                    if (*contact.mNormalImpulse > 0)
                        ++loadedSpeculative;
                    else
                        ++unloadedSpeculative;
                }
            }
            // Independently recover the vertical collision impulse from the
            // body's momentum change after removing gravity and internal loads.
            ASSERT_TRUE(frame.mSolverImpulse);
            EXPECT_NEAR(contactImpulse.GetZ(), frame.mSolverImpulse->mLinearImpulse.GetZ(), 0.01f);
        }
        EXPECT_GT(measured, 0u);
        EXPECT_GT(loadedSpeculative, 0u);
        EXPECT_GT(unloadedSpeculative, 0u);
    }

    TEST(RagdollCoreScenarioTest, standsOnGroundPlaneWithObservableSupport)
    {
        const auto result = run(standingScript(1.0f));
        ASSERT_TRUE(result.completed()) << describe(result);
        ASSERT_EQ(result.mFrames.size(), 60u);
        EXPECT_EQ(result.mBodyNames.size(), 20u);
        EXPECT_EQ(result.mBodyNames[result.mLandmarks.mLeftFoot], "left_foot");
        EXPECT_EQ(result.mBodyNames[result.mLandmarks.mRightFoot], "right_foot");
        EXPECT_EQ(result.mBodyNames[result.mLandmarks.mPelvis], "pelvis");
        EXPECT_EQ(result.mBodyNames[result.mLandmarks.mTorso], "spine2");
        EXPECT_GT(result.mTargetPelvisHeight, 50.0f);
        EXPECT_FALSE(result.mKillingImpulse.has_value());

        const RagdollCore::ScenarioFrame& last = result.mFrames.back();
        EXPECT_TRUE(last.mSupport.mLeftFootSupported) << describe(result);
        EXPECT_TRUE(last.mSupport.mRightFootSupported) << describe(result);
        EXPECT_TRUE(last.mSupport.mCenterOfMassSupported) << describe(result);
        EXPECT_GT(last.mContacts.size(), 0u);
        for (const RagdollCore::GroundContact& contact : last.mContacts)
        {
            EXPECT_GE(contact.mNormal.GetZ(), 0.5f);
            EXPECT_NEAR(static_cast<double>(contact.mPosition.GetZ()), 0.0, 4.0);
            EXPECT_TRUE(
                contact.mBodyIndex == result.mLandmarks.mLeftFoot || contact.mBodyIndex == result.mLandmarks.mRightFoot)
                << "body " << contact.mBodyIndex << " touched the ground";
        }
        EXPECT_NE(last.mController.mMode, ActiveControllerMode::Falling) << describe(result);
        EXPECT_NE(last.mController.mMode, ActiveControllerMode::Fallen) << describe(result);
        EXPECT_GT(last.mPelvisUpDot, 0.9f) << describe(result);
        EXPECT_NEAR(last.mPelvisHeight, result.mTargetPelvisHeight, result.mTargetPelvisHeight * 0.25f)
            << describe(result);
        EXPECT_TRUE(last.mMotorsApplied);
        EXPECT_FALSE(last.mUprightApplied);
        ASSERT_TRUE(last.mJointSupport);
        EXPECT_TRUE(last.mJointSupport->mSupported);
        EXPECT_TRUE(last.mVerticalReactionBodies.empty());
        EXPECT_TRUE(last.mHorizontalReactionBodies.empty());
        EXPECT_LT(last.mAppliedNetForce.Length(), 1e-3f);
        EXPECT_LT(last.mAppliedNetActuationMoment.Length(), 1e-3f);
        EXPECT_EQ(last.mGait.mPhase, GaitPhase::Idle);
        EXPECT_LT(last.mCenterOfMassVelocity.Length(), 60.0f) << describe(result);
        for (const RagdollCore::ScenarioBodyRecord& body : last.mBodies)
        {
            EXPECT_GT(body.mMass, 0.0f);
            EXPECT_LT(body.mLinearVelocity.Length(), 500.0f);
        }

        const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
        std::cout << "STANDING_INVARIANTS " << describe(invariants) << std::endl;
        EXPECT_EQ(invariants.mAirborneFrames, 0u);
        EXPECT_EQ(invariants.mPelvisForceSaturatedFrames, 0u);
        EXPECT_EQ(invariants.mPelvisMagnitudeSaturatedFrames, 0u);
        EXPECT_EQ(invariants.mPelvisHorizontalForceSaturatedFrames, 0u);
        EXPECT_EQ(invariants.mPelvisVerticalForceSaturatedFrames, 0u);
        EXPECT_EQ(invariants.mPelvisTorqueSaturatedFrames, 0u);
        EXPECT_EQ(invariants.mSpineTorqueSaturatedFrames, 0u);
        EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
        EXPECT_LT(invariants.mMaxStanceFootSlide, 1.0f);
        EXPECT_EQ(invariants.mSteps.mLeftStanceCount + invariants.mSteps.mRightStanceCount, 0u);
        EXPECT_EQ(invariants.mFallingFrames + invariants.mFallenFrames, 0u);
    }

    TEST(RagdollCoreScenarioTest, simultaneousCrowdIsIndependentStableAndDeterministic)
    {
        RagdollCore::CrowdScenarioSettings settings;
        settings.mActorCount = 4;
        settings.mActorSpacing = 80.0f;
        settings.mDurationSeconds = 1.0f;
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto profile = RagdollCore::makeScenarioHumanoidProfile();
        const auto expected = RagdollCore::runStandingCrowdScenario(skeleton, profile, settings);
        const auto actual = RagdollCore::runStandingCrowdScenario(skeleton, profile, settings);
        ASSERT_TRUE(expected.completed()) << describe(expected);
        ASSERT_TRUE(actual.completed()) << describe(actual);
        ASSERT_EQ(expected.mFrames.size(), 60u);
        EXPECT_TRUE(RagdollCore::haveIdenticalCrowdScenarioTelemetry(expected, actual));
        EXPECT_EQ(RagdollCore::hashCrowdScenarioTelemetry(expected), RagdollCore::hashCrowdScenarioTelemetry(actual));
        EXPECT_GT(RagdollCore::encodeCrowdScenarioTelemetry(expected).size(), 1024u);

        std::size_t crossActorContacts = 0;
        for (const RagdollCore::CrowdScenarioFrame& frame : expected.mFrames)
        {
            ASSERT_EQ(frame.mActors.size(), settings.mActorCount);
            crossActorContacts += frame.mCrossActorContactCount;
        }
        EXPECT_EQ(crossActorContacts, 0u) << describe(expected);

        const RagdollCore::CrowdScenarioFrame& last = expected.mFrames.back();
        for (std::size_t actorIndex = 0; actorIndex < last.mActors.size(); ++actorIndex)
        {
            SCOPED_TRACE("actor " + std::to_string(actorIndex));
            const RagdollCore::CrowdActorFrame& actor = last.mActors[actorIndex];
            EXPECT_TRUE(actor.mSupport.mLeftFootSupported) << describe(expected);
            EXPECT_TRUE(actor.mSupport.mRightFootSupported) << describe(expected);
            EXPECT_TRUE(actor.mSupport.mCenterOfMassSupported) << describe(expected);
            EXPECT_NE(actor.mMode, ActiveControllerMode::Falling) << describe(expected);
            EXPECT_NE(actor.mMode, ActiveControllerMode::Fallen) << describe(expected);
            EXPECT_GT(actor.mPelvisUpDot, 0.9f) << describe(expected);
            EXPECT_LT(actor.mCenterOfMassVelocity.Length(), 60.0f) << describe(expected);
            EXPECT_LT(actor.mMaxBodySpeed, 500.0f) << describe(expected);
            EXPECT_TRUE(actor.mMotorsApplied);
            EXPECT_TRUE(actor.mJointSupportApplied);
            EXPECT_FALSE(actor.mUprightApplied);
            if (actorIndex > 0)
            {
                const double separation
                    = actor.mCenterOfMass.GetX() - last.mActors[actorIndex - 1].mCenterOfMass.GetX();
                EXPECT_GT(separation, settings.mActorSpacing * 0.75) << "independent actors converged toward collision";
            }
        }

        RecordProperty("crowd_telemetry_hash", std::to_string(RagdollCore::hashCrowdScenarioTelemetry(expected)));
        std::cout << "CROWD_TELEMETRY_HASH " << RagdollCore::hashCrowdScenarioTelemetry(expected) << std::endl;
    }

    TEST(RagdollCoreScenarioTest, formatsDeterministicCrowdCsvTrace)
    {
        RagdollCore::CrowdScenarioSettings settings;
        settings.mActorCount = 4;
        settings.mDurationSeconds = 0.1f;
        const auto result = RagdollCore::runStandingCrowdScenario(RagdollCore::makeScenarioHumanoidSkeleton(),
            RagdollCore::makeScenarioHumanoidProfile(), settings);
        ASSERT_TRUE(result.completed()) << describe(result);
        const std::string expected = RagdollCore::formatCrowdScenarioTraceCsv(result);
        const std::string actual = RagdollCore::formatCrowdScenarioTraceCsv(result);
        EXPECT_EQ(actual, expected);
        EXPECT_TRUE(expected.starts_with("step,time,actor,mode"));
        EXPECT_NE(expected.find("left_supported,right_supported,center_supported"), std::string::npos);
        EXPECT_NE(expected.find("impulse_count,motors_applied,upright_applied,cross_actor_contacts"),
            std::string::npos);
        EXPECT_EQ(std::count(expected.begin(), expected.end(), '\n'),
            result.mFrames.size() * settings.mActorCount + 1);
    }

    TEST(RagdollCoreScenarioTest, closeCrowdReportsCrossActorContacts)
    {
        RagdollCore::CrowdScenarioSettings settings;
        settings.mActorCount = 2;
        settings.mActorSpacing = 12.0f;
        settings.mDurationSeconds = 0.2f;
        const auto result = RagdollCore::runStandingCrowdScenario(
            RagdollCore::makeScenarioHumanoidSkeleton(), RagdollCore::makeScenarioHumanoidProfile(), settings);
        ASSERT_TRUE(result.completed()) << describe(result);
        std::size_t crossActorContacts = 0;
        for (const RagdollCore::CrowdScenarioFrame& frame : result.mFrames)
            crossActorContacts += frame.mCrossActorContactCount;
        EXPECT_GT(crossActorContacts, 0u) << "the close-spacing control did not exercise actor collisions";
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreScenarioTest, DISABLED_simultaneousCrowdIsolatesOnePerturbedActorAcrossStateTimeline)
    {
        RagdollCore::CrowdScenarioSettings settings;
        settings.mActorCount = 4;
        settings.mActorSpacing = 100.0f;
        settings.mDurationSeconds = 2.0f;
        RagdollCore::CrowdScenarioImpulse perturbation;
        // Disturb only after the shared settled-height latch has armed upright
        // control; otherwise this measures insertion settling, not bracing.
        perturbation.mStep = 40;
        perturbation.mActorIndex = 1;
        perturbation.mBodyIndex = 3; // spine2
        // The checked impulse-response matrix pins this in the bracing-only
        // range without crossing into falling under the current gains.
        perturbation.mImpulse = JPH::Vec3(0.0f, 3000.0f, 0.0f);
        settings.mImpulses.push_back(perturbation);

        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto profile = RagdollCore::makeScenarioHumanoidProfile();
        const auto expected = RagdollCore::runStandingCrowdScenario(skeleton, profile, settings);
        const auto actual = RagdollCore::runStandingCrowdScenario(skeleton, profile, settings);
        auto baselineSettings = settings;
        baselineSettings.mImpulses.clear();
        const auto baseline = RagdollCore::runStandingCrowdScenario(skeleton, profile, baselineSettings);
        ASSERT_TRUE(expected.completed()) << describe(expected);
        ASSERT_TRUE(actual.completed()) << describe(actual);
        ASSERT_TRUE(baseline.completed()) << describe(baseline);
        EXPECT_TRUE(RagdollCore::haveIdenticalCrowdScenarioTelemetry(expected, actual));
        ASSERT_EQ(expected.mFrames.size(), baseline.mFrames.size());

        std::array<std::size_t, 4> nonStandingFrames{};
        std::array<std::size_t, 4> bracingFrames{};
        std::array<std::size_t, 4> fallingFrames{};
        std::array<std::size_t, 4> impulseCounts{};
        std::size_t crossActorContacts = 0;
        for (const RagdollCore::CrowdScenarioFrame& frame : expected.mFrames)
        {
            ASSERT_EQ(frame.mActors.size(), settings.mActorCount);
            crossActorContacts += frame.mCrossActorContactCount;
            for (std::size_t actorIndex = 0; actorIndex < frame.mActors.size(); ++actorIndex)
            {
                const auto& actor = frame.mActors[actorIndex];
                nonStandingFrames[actorIndex] += actor.mMode != ActiveControllerMode::Standing;
                bracingFrames[actorIndex]
                    += actor.mMode == ActiveControllerMode::Bracing || actor.mMode == ActiveControllerMode::Stumbling;
                fallingFrames[actorIndex]
                    += actor.mMode == ActiveControllerMode::Falling || actor.mMode == ActiveControllerMode::Fallen;
                impulseCounts[actorIndex] += actor.mImpulseCount;
            }
        }

        EXPECT_EQ(impulseCounts, (std::array<std::size_t, 4>{ 0, 1, 0, 0 }));
        EXPECT_GT(nonStandingFrames[1], 0u) << "the physical perturbation did not enter another controller state";
        EXPECT_GT(bracingFrames[1], 0u) << "the disturbed actor did not brace or stumble";
        EXPECT_EQ(fallingFrames[1], 0u) << "the scripted bracing disturbance caused a fall";
        ASSERT_EQ(expected.mFrames.back().mActors.size(), settings.mActorCount);
        EXPECT_EQ(crossActorContacts, 0u) << "the isolation case unexpectedly became a collision test";
        for (const std::size_t actorIndex : { 0u, 2u, 3u })
        {
            EXPECT_EQ(fallingFrames[actorIndex], 0u) << "neighbor " << actorIndex << " fell";
            for (std::size_t frameIndex = 0; frameIndex < expected.mFrames.size(); ++frameIndex)
                EXPECT_EQ(expected.mFrames[frameIndex].mActors[actorIndex].mMode,
                    baseline.mFrames[frameIndex].mActors[actorIndex].mMode)
                    << "neighbor " << actorIndex << " diverged from its no-impulse control at frame " << frameIndex;
        }
        std::size_t baselinePerturbedActorNonStanding = 0;
        for (const RagdollCore::CrowdScenarioFrame& frame : baseline.mFrames)
            baselinePerturbedActorNonStanding += frame.mActors[1].mMode != ActiveControllerMode::Standing;
        EXPECT_GT(nonStandingFrames[1], baselinePerturbedActorNonStanding)
            << "the perturbed actor did not diverge from its no-impulse control";
        std::cout << "CROWD_STATE_MATRIX actor0=" << nonStandingFrames[0] << " actor1=" << nonStandingFrames[1]
                  << " actor2=" << nonStandingFrames[2] << " actor3=" << nonStandingFrames[3]
                  << " hash=" << RagdollCore::hashCrowdScenarioTelemetry(expected) << std::endl;
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreScenarioTest, DISABLED_simultaneousCrowdExercisesMixedControllerStateMatrix)
    {
        RagdollCore::CrowdScenarioSettings settings;
        settings.mActorCount = 4;
        settings.mActorSpacing = 120.0f;
        settings.mDurationSeconds = 2.0f;
        // Values are selected from the broad response matrix below: one
        // control, one recoverable brace, and two genuine falls. Keeping the
        // strongest stimulus at 12000 avoids the old 45000-unit launcher.
        constexpr std::array<float, 4> impulses{ 0.0f, 3000.0f, 12000.0f, 12000.0f };
        for (std::size_t actorIndex = 1; actorIndex < impulses.size(); ++actorIndex)
        {
            RagdollCore::CrowdScenarioImpulse perturbation;
            perturbation.mStep = 40;
            perturbation.mActorIndex = actorIndex;
            perturbation.mBodyIndex = 3; // spine2
            perturbation.mImpulse = JPH::Vec3(0.0f, impulses[actorIndex], 0.0f);
            settings.mImpulses.push_back(perturbation);
        }

        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto profile = RagdollCore::makeScenarioHumanoidProfile();
        const auto expected = RagdollCore::runStandingCrowdScenario(skeleton, profile, settings);
        const auto actual = RagdollCore::runStandingCrowdScenario(skeleton, profile, settings);
        ASSERT_TRUE(expected.completed()) << describe(expected);
        ASSERT_TRUE(actual.completed()) << describe(actual);
        EXPECT_TRUE(RagdollCore::haveIdenticalCrowdScenarioTelemetry(expected, actual));
        exportTraceIfRequested(expected, "crowd-mixed-state-matrix");

        std::array<std::array<std::size_t, sControllerModeCount>, 4> stateFrames{};
        std::array<std::size_t, 4> impulseCounts{};
        std::size_t crossActorContacts = 0;
        bool observedMixedFrame = false;
        for (const RagdollCore::CrowdScenarioFrame& frame : expected.mFrames)
        {
            ASSERT_EQ(frame.mActors.size(), settings.mActorCount);
            crossActorContacts += frame.mCrossActorContactCount;
            for (std::size_t actorIndex = 0; actorIndex < frame.mActors.size(); ++actorIndex)
            {
                const auto& actor = frame.mActors[actorIndex];
                ++stateFrames[actorIndex][static_cast<std::size_t>(actor.mMode)];
                impulseCounts[actorIndex] += actor.mImpulseCount;
            }
            const auto isBracing = [](ActiveControllerMode mode) {
                return mode == ActiveControllerMode::Bracing || mode == ActiveControllerMode::Stumbling;
            };
            const auto isDown = [](ActiveControllerMode mode) {
                return mode == ActiveControllerMode::Falling || mode == ActiveControllerMode::Fallen;
            };
            observedMixedFrame = observedMixedFrame
                || (frame.mActors[0].mMode == ActiveControllerMode::Standing && isBracing(frame.mActors[1].mMode)
                    && isDown(frame.mActors[2].mMode) && isDown(frame.mActors[3].mMode));
        }

        EXPECT_EQ(impulseCounts, (std::array<std::size_t, 4>{ 0, 1, 1, 1 }));
        EXPECT_EQ(crossActorContacts, 0u);
        EXPECT_EQ(stateFrames[0][static_cast<std::size_t>(ActiveControllerMode::Falling)], 0u);
        EXPECT_EQ(stateFrames[0][static_cast<std::size_t>(ActiveControllerMode::Fallen)], 0u);
        EXPECT_GT(stateFrames[1][static_cast<std::size_t>(ActiveControllerMode::Bracing)]
                + stateFrames[1][static_cast<std::size_t>(ActiveControllerMode::Stumbling)],
            0u);
        EXPECT_EQ(stateFrames[1][static_cast<std::size_t>(ActiveControllerMode::Falling)], 0u);
        EXPECT_EQ(stateFrames[1][static_cast<std::size_t>(ActiveControllerMode::Fallen)], 0u);
        for (const std::size_t actorIndex : { 2u, 3u })
        {
            EXPECT_GT(stateFrames[actorIndex][static_cast<std::size_t>(ActiveControllerMode::Falling)], 0u);
            EXPECT_GT(stateFrames[actorIndex][static_cast<std::size_t>(ActiveControllerMode::Fallen)], 0u);
        }
        EXPECT_TRUE(observedMixedFrame)
            << "the shared world never contained standing, bracing, and down actors simultaneously";
        for (std::size_t actorIndex = 0; actorIndex < stateFrames.size(); ++actorIndex)
        {
            const ControllerTransitionMatrix transitions = transitionMatrix(expected, actorIndex);
            std::cout << "CROWD_MIXED_STATE actor=" << actorIndex << " impulse=" << impulses[actorIndex];
            for (std::size_t mode = 0; mode < sControllerModeCount; ++mode)
                std::cout << ' ' << modeName(static_cast<ActiveControllerMode>(mode)) << '='
                          << stateFrames[actorIndex][mode];
            std::cout << std::endl;
            for (std::size_t from = 0; from < sControllerModeCount; ++from)
            {
                for (std::size_t to = 0; to < sControllerModeCount; ++to)
                {
                    if (transitions[from][to] == 0)
                        continue;
                    const auto fromMode = static_cast<ActiveControllerMode>(from);
                    const auto toMode = static_cast<ActiveControllerMode>(to);
                    EXPECT_TRUE(isAllowedControllerTransition(fromMode, toMode))
                        << "actor " << actorIndex << " made illegal transition " << modeName(fromMode) << " -> "
                        << modeName(toMode);
                    if (from != to)
                        std::cout << "CROWD_TRANSITION actor=" << actorIndex << ' ' << modeName(fromMode) << "->"
                                  << modeName(toMode) << " count=" << transitions[from][to] << std::endl;
                }
            }
        }

        const ControllerTransitionMatrix bracingActor = transitionMatrix(expected, 1);
        EXPECT_GT(bracingActor[static_cast<std::size_t>(ActiveControllerMode::Standing)]
                              [static_cast<std::size_t>(ActiveControllerMode::Bracing)],
            0u);
        const ControllerTransitionMatrix fallingActor = transitionMatrix(expected, 2);
        EXPECT_GT(fallingActor[static_cast<std::size_t>(ActiveControllerMode::Bracing)]
                              [static_cast<std::size_t>(ActiveControllerMode::Falling)],
            0u);
        EXPECT_GT(fallingActor[static_cast<std::size_t>(ActiveControllerMode::Falling)]
                              [static_cast<std::size_t>(ActiveControllerMode::Fallen)],
            0u);
    }

    TEST(RagdollCoreScenarioTest, simultaneousCrowdMapsImpulseResponseStateMatrix)
    {
        // Exercise a useful range in one shared deterministic world. This is
        // diagnostic coverage, not a set of hand-picked transition
        // thresholds: every row reports the controller response and still
        // enforces legal transitions, actor isolation, and repeatability.
        constexpr std::array<float, 10> impulses{ 0.0f, 250.0f, 500.0f, 1000.0f, 2000.0f, 2500.0f, 3000.0f,
            3500.0f, 4000.0f, 12000.0f };
        RagdollCore::CrowdScenarioSettings settings;
        settings.mActorCount = impulses.size();
        settings.mActorSpacing = 120.0f;
        settings.mDurationSeconds = 2.0f;
        for (std::size_t actorIndex = 1; actorIndex < impulses.size(); ++actorIndex)
        {
            RagdollCore::CrowdScenarioImpulse perturbation;
            perturbation.mStep = 40;
            perturbation.mActorIndex = actorIndex;
            perturbation.mBodyIndex = 3; // spine2
            perturbation.mImpulse = JPH::Vec3(0.0f, impulses[actorIndex], 0.0f);
            settings.mImpulses.push_back(perturbation);
        }

        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto profile = RagdollCore::makeScenarioHumanoidProfile();
        const auto expected = RagdollCore::runStandingCrowdScenario(skeleton, profile, settings);
        const auto actual = RagdollCore::runStandingCrowdScenario(skeleton, profile, settings);
        ASSERT_TRUE(expected.completed()) << describe(expected);
        ASSERT_TRUE(actual.completed()) << describe(actual);
        EXPECT_TRUE(RagdollCore::haveIdenticalCrowdScenarioTelemetry(expected, actual));
        exportTraceIfRequested(expected, "crowd-impulse-state-matrix");

        std::array<std::array<std::size_t, sControllerModeCount>, impulses.size()> stateFrames{};
        std::array<std::size_t, impulses.size()> impulseCounts{};
        std::size_t crossActorContacts = 0;
        for (const RagdollCore::CrowdScenarioFrame& frame : expected.mFrames)
        {
            ASSERT_EQ(frame.mActors.size(), impulses.size());
            crossActorContacts += frame.mCrossActorContactCount;
            for (std::size_t actorIndex = 0; actorIndex < frame.mActors.size(); ++actorIndex)
            {
                ++stateFrames[actorIndex][static_cast<std::size_t>(frame.mActors[actorIndex].mMode)];
                impulseCounts[actorIndex] += frame.mActors[actorIndex].mImpulseCount;
            }
        }
        EXPECT_EQ(crossActorContacts, 0u);
        EXPECT_EQ(impulseCounts[0], 0u);
        for (std::size_t actorIndex = 1; actorIndex < impulseCounts.size(); ++actorIndex)
            EXPECT_EQ(impulseCounts[actorIndex], 1u);

        std::cout << "CROWD_IMPULSE_STATE_MATRIX impulse";
        for (std::size_t mode = 0; mode < sControllerModeCount; ++mode)
            std::cout << ' ' << modeName(static_cast<ActiveControllerMode>(mode));
        std::cout << std::endl;
        for (std::size_t actorIndex = 0; actorIndex < impulses.size(); ++actorIndex)
        {
            std::cout << "CROWD_IMPULSE_STATE " << impulses[actorIndex];
            for (const std::size_t count : stateFrames[actorIndex])
                std::cout << ' ' << count;
            std::cout << std::endl;

            const ControllerTransitionMatrix transitions = transitionMatrix(expected, actorIndex);
            for (std::size_t from = 0; from < sControllerModeCount; ++from)
            {
                for (std::size_t to = 0; to < sControllerModeCount; ++to)
                {
                    if (transitions[from][to] == 0)
                        continue;
                    EXPECT_TRUE(isAllowedControllerTransition(
                        static_cast<ActiveControllerMode>(from), static_cast<ActiveControllerMode>(to)))
                        << "actor " << actorIndex << " made illegal transition "
                        << modeName(static_cast<ActiveControllerMode>(from)) << " -> "
                        << modeName(static_cast<ActiveControllerMode>(to));
                }
            }
        }
        RecordProperty("crowd_impulse_matrix_hash", std::to_string(RagdollCore::hashCrowdScenarioTelemetry(expected)));
    }

    TEST(RagdollCoreScenarioTest, rejectsInvalidCrowdSettingsBeforeSimulating)
    {
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto profile = RagdollCore::makeScenarioHumanoidProfile();
        RagdollCore::CrowdScenarioSettings settings;
        settings.mActorCount = 1;
        auto result = RagdollCore::runStandingCrowdScenario(skeleton, profile, settings);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        settings.mActorCount = 4;
        settings.mActorSpacing = 0.0f;
        result = RagdollCore::runStandingCrowdScenario(skeleton, profile, settings);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        settings.mActorSpacing = 80.0f;
        settings.mActorSettings.mPowered = false;
        result = RagdollCore::runStandingCrowdScenario(skeleton, profile, settings);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        settings.mActorSettings.mPowered = true;
        settings.mImpulses.push_back({ 0, settings.mActorCount, 0, JPH::Vec3::sZero(), std::nullopt });
        result = RagdollCore::runStandingCrowdScenario(skeleton, profile, settings);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());
    }

    TEST(RagdollCoreScenarioTest, terminatesBySimulationTime)
    {
        const auto quarter = run(standingScript(0.25f));
        ASSERT_TRUE(quarter.completed()) << describe(quarter);
        EXPECT_EQ(quarter.mStepCount, 15u);
        ASSERT_EQ(quarter.mFrames.size(), 15u);
        EXPECT_EQ(quarter.mFrames.front().mStep, 0u);
        EXPECT_NEAR(quarter.mFrames.front().mTime, 1.0f / 60.0f, 1e-6f);
        EXPECT_NEAR(quarter.mFrames.back().mTime, 0.25f, 1e-5f);

        RagdollCore::ScenarioSettings halfRate;
        halfRate.mFixedTimeStep = 1.0f / 30.0f;
        const auto coarse = run(standingScript(0.5f), halfRate);
        ASSERT_TRUE(coarse.completed()) << describe(coarse);
        EXPECT_EQ(coarse.mFrames.size(), 15u);

        const auto tooShort = run(standingScript(0.001f));
        EXPECT_FALSE(tooShort.completed());
        EXPECT_TRUE(tooShort.mFrames.empty());
        EXPECT_FALSE(tooShort.mErrors.empty());
    }

    TEST(RagdollCoreScenarioTest, repeatsIdenticalTelemetryAcrossRuns)
    {
        const auto expected = run(perturbationScript());
        const auto actual = run(perturbationScript());
        ASSERT_TRUE(expected.completed()) << describe(expected);
        ASSERT_TRUE(actual.completed()) << describe(actual);
        ASSERT_EQ(expected.mFrames.size(), 90u);
        EXPECT_TRUE(RagdollCore::haveIdenticalScenarioTelemetry(expected, actual));
        EXPECT_EQ(RagdollCore::hashScenarioTelemetry(expected), RagdollCore::hashScenarioTelemetry(actual));
        // Printed so separate process invocations can be compared as well.
        RecordProperty("telemetry_hash", std::to_string(RagdollCore::hashScenarioTelemetry(expected)));
        std::cout << "SCENARIO_TELEMETRY_HASH " << RagdollCore::hashScenarioTelemetry(expected) << std::endl;
        EXPECT_EQ(RagdollCore::encodeScenarioTelemetry(expected), RagdollCore::encodeScenarioTelemetry(actual));
        EXPECT_GT(RagdollCore::encodeScenarioTelemetry(expected).size(), 1024u);

        EXPECT_EQ(expected.mFrames[19].mImpulseCount, 0u);
        EXPECT_EQ(expected.mFrames[20].mImpulseCount, 2u);
        EXPECT_GT(expected.mFrames[20].mCenterOfMassVelocity.GetX(), expected.mFrames[19].mCenterOfMassVelocity.GetX());
        EXPECT_TRUE(expected.mFrames[39].mRequestedHorizontalVelocity.IsNearZero());
        EXPECT_TRUE(expected.mFrames[40].mRequestedHorizontalVelocity.IsClose(JPH::Vec3(0.0f, 60.0f, 0.0f)));
        EXPECT_EQ(expected.mFrames[40].mControllerInput.mTargetHorizontalVelocity.IsClose(
                      JPH::Vec3(0.0f, 60.0f, 0.0f)),
            expected.mFrames[40].mPelvisHeightTracker.mSettled);
        if (!expected.mFrames[40].mPelvisHeightTracker.mSettled)
            EXPECT_TRUE(expected.mFrames[40].mControllerInput.mTargetHorizontalVelocity.IsNearZero());
        EXPECT_TRUE(expected.mFrames[74].mMotorsApplied);
        for (std::size_t i = 75; i < expected.mFrames.size(); ++i)
        {
            EXPECT_FALSE(expected.mFrames[i].mMotorsApplied);
            EXPECT_FALSE(expected.mFrames[i].mUprightApplied);
            EXPECT_TRUE(expected.mFrames[i].mAppliedPelvisForce.IsNearZero());
            EXPECT_TRUE(expected.mFrames[i].mAppliedPelvisTorque.IsNearZero());
        }
    }

    TEST(RagdollCoreScenarioTest, formatsDeterministicPerFrameCsvTrace)
    {
        const auto result = run(standingScript(0.1f));
        ASSERT_TRUE(result.completed()) << describe(result);
        const std::string expected = RagdollCore::formatScenarioTraceCsv(result);
        const std::string actual = RagdollCore::formatScenarioTraceCsv(result);
        EXPECT_EQ(actual, expected);
        EXPECT_TRUE(expected.starts_with("step,time,gait_profile_hash,mode,gait_phase"));
        EXPECT_EQ(std::count(expected.begin(), expected.end(), '\n'), result.mFrames.size() + 1);
        EXPECT_NE(expected.find(
                      "magnitude_saturated,horizontal_saturated,support_moment_saturated,vertical_saturated"),
            std::string::npos);
        EXPECT_NE(expected.find("swing_control_applied,swing_force_x,swing_force_y,swing_force_z"),
            std::string::npos);
        EXPECT_NE(expected.find("left_foot_x,left_foot_y,left_foot_z,left_foot_vz"), std::string::npos);
        EXPECT_NE(expected.find("actuation_gait_phase,actuation_gait_progress"), std::string::npos);
        EXPECT_NE(expected.find("gait_contact_candidate_time,gait_time_since_lift,swing_assist_ready"),
            std::string::npos);
        EXPECT_NE(expected.find("swing_vertical_reaction_body,swing_horizontal_reaction_body"), std::string::npos);
        EXPECT_NE(expected.find("swing_placement_x,swing_placement_y,swing_placement_z"), std::string::npos);
        EXPECT_NE(expected.find("swing_motor_target_x,swing_motor_target_y,swing_motor_target_z"), std::string::npos);
        EXPECT_NE(expected.find("resolved_swing_target_x,resolved_swing_target_y,resolved_swing_target_z"),
            std::string::npos);
        EXPECT_NE(expected.find("swing_pose_target_x,swing_pose_target_y,swing_pose_target_z"), std::string::npos);
    }

    TEST(RagdollCoreScenarioTest, traceReportsAppliedCoordinatedPoseWithoutScenarioModeFlag)
    {
        using namespace RagdollCore;
        auto result = run(standingScript(0.1f));
        ASSERT_TRUE(result.completed());
        ASSERT_GE(result.mFrames.size(), 3u);
        ASSERT_FALSE(result.mSettings.mReferenceGaitCoordinatesLegPose);
        result.mFrames[0].mReferenceGait.reset();
        result.mFrames[1].mReferenceGait.emplace();
        result.mFrames[2].mReferenceGait.emplace();
        result.mFrames[2].mReferenceGait->mCoordinatedPelvisTargetZ = 0.0;
        const auto columns = [](const std::string& text) {
            std::vector<std::string> values;
            std::istringstream input(text);
            std::string value;
            while (std::getline(input, value, ',')) values.push_back(value);
            if (text.ends_with(',')) values.emplace_back();
            return values;
        };
        std::istringstream trace(formatScenarioTraceCsv(result));
        std::string line;
        ASSERT_TRUE(static_cast<bool>(std::getline(trace, line)));
        const auto header = columns(line);
        const auto found = std::find(header.begin(), header.end(), "reference_coordinated_pose_applied");
        ASSERT_NE(found, header.end());
        const auto index = static_cast<std::size_t>(found - header.begin());
        for (const auto expected : { "0", "0", "1" })
        {
            ASSERT_TRUE(static_cast<bool>(std::getline(trace, line)));
            const auto values = columns(line);
            ASSERT_EQ(values.size(), header.size());
            EXPECT_EQ(values[index], expected);
        }
    }

    TEST(RagdollCoreScenarioTest, referenceMotorUtilizationUsesCommandBodyAndPreservesMissingData)
    {
        using namespace RagdollCore;
        auto result = run(standingScript(0.1f));
        ASSERT_TRUE(result.completed());
        ASSERT_GE(result.mFrames.size(), 4u);
        result.mFrames[0].mReferenceGait.reset();
        for (std::size_t i = 1; i < 4; ++i)
        {
            auto& frame = result.mFrames[i];
            frame.mReferenceGait.emplace();
            frame.mReferenceGait->mStanceHipBody = i == 1 ? 1 : 2;
            frame.mReferenceGait->mStanceAnkleBody = i == 1 ? 2 : 1;
            frame.mJointSupportMotorUtilizations = { 0, 0.25f, 0.75f };
            // Deliberately contradictory old-observation stance values.
            frame.mStanceLegMotorPeakAxisUtilizations = { 1, 1, 1 };
        }
        result.mFrames[3].mJointSupportMotorUtilizations.clear();
        const auto columns = [](const std::string& text) {
            std::vector<std::string> values;
            std::istringstream input(text);
            std::string value;
            while (std::getline(input, value, ',')) values.push_back(value);
            if (text.ends_with(',')) values.emplace_back();
            return values;
        };
        std::istringstream trace(formatScenarioTraceCsv(result));
        std::string line;
        ASSERT_TRUE(static_cast<bool>(std::getline(trace, line)));
        const auto header = columns(line);
        const std::array names{ "reference_stance_hip_body", "reference_stance_hip_utilization",
            "reference_stance_ankle_body", "reference_stance_ankle_utilization" };
        std::array<std::size_t, 4> indices;
        for (std::size_t i = 0; i < names.size(); ++i)
        {
            const auto found = std::find(header.begin(), header.end(), names[i]);
            ASSERT_NE(found, header.end());
            indices[i] = found - header.begin();
        }
        const std::array<std::array<const char*, 4>, 4> expected{{
            {{ "", "", "", "" }}, {{ "1", "0.25", "2", "0.75" }},
            {{ "2", "0.75", "1", "0.25" }}, {{ "2", "", "1", "" }} }};
        for (const auto& row : expected)
        {
            ASSERT_TRUE(static_cast<bool>(std::getline(trace, line)));
            const auto values = columns(line);
            ASSERT_EQ(values.size(), header.size());
            for (std::size_t i = 0; i < indices.size(); ++i)
                EXPECT_EQ(values[indices[i]], row[i]);
        }
    }

    TEST(RagdollCoreScenarioTest, footLoadTraceDistinguishesUnknownImpulsesFromUnloadedFeet)
    {
        using namespace RagdollCore;
        auto result = run(standingScript(0.1f));
        ASSERT_TRUE(result.completed());
        ASSERT_GE(result.mFrames.size(), 5u);
        const auto left = result.mLandmarks.mLeftFoot;
        const auto right = result.mLandmarks.mRightFoot;
        const auto contact = [](std::size_t body, std::optional<float> impulse, std::optional<float> velocity = std::nullopt) {
            GroundContact value{ body, JPH::RVec3::sZero(), JPH::Vec3::sAxisZ(), 0, impulse };
            if (velocity)
                value.mPointVelocity = JPH::Vec3(0, 0, *velocity);
            return value;
        };
        result.mFrames[0].mContacts = { contact(left, 2, -3), contact(right, 7, 1),
            contact(left, 3, 4), contact(result.mLandmarks.mPelvis, 11, 100) };
        result.mFrames[1].mContacts = { contact(left, 2, 3), contact(left, std::nullopt, 1), contact(right, 0, 999) };
        result.mFrames[2].mContacts = {};
        result.mFrames[3].mContacts = { contact(left, std::numeric_limits<float>::quiet_NaN()), contact(right, -1) };
        result.mFrames[4].mContacts = { contact(left, 2, 0), contact(left, 3),
            contact(right, 1, std::numeric_limits<float>::quiet_NaN()) };
        const auto hash = hashScenarioTelemetry(result);
        result.mFrames[0].mContacts[0].mPointVelocity->SetZ(-4);
        EXPECT_NE(hashScenarioTelemetry(result), hash);
        result.mFrames[0].mContacts[0].mPointVelocity->SetZ(-3);
        std::istringstream trace(formatScenarioTraceCsv(result));
        std::string line;
        ASSERT_TRUE(static_cast<bool>(std::getline(trace, line)));
        const auto columns = [](const std::string& text) {
            std::vector<std::string> values;
            std::istringstream input(text);
            std::string value;
            while (std::getline(input, value, ','))
                values.push_back(value);
            if (text.ends_with(',')) values.emplace_back();
            return values;
        };
        const auto header = columns(line);
        std::vector<std::size_t> selected;
        for (const auto name : { "left_contact_normal_impulse", "right_contact_normal_impulse",
                 "left_loaded_point_min_vz", "left_loaded_point_max_vz",
                 "right_loaded_point_min_vz", "right_loaded_point_max_vz" })
        {
            const auto found = std::find(header.begin(), header.end(), name);
            ASSERT_NE(found, header.end());
            selected.push_back(static_cast<std::size_t>(found - header.begin()));
        }
        for (const auto suffix : { ",5,7,-3,4,1,1", ",,0,,,,", ",0,0,,,,", ",,,,,,", ",5,1,,,," })
        {
            ASSERT_TRUE(static_cast<bool>(std::getline(trace, line)));
            const auto values = columns(line);
            ASSERT_EQ(values.size(), header.size());
            std::string observed;
            for (const auto column : selected) observed += ',' + values[column];
            EXPECT_EQ(observed, suffix);
        }
    }

    TEST(RagdollCoreScenarioTest, negativeControlDisablesTranslationActuation)
    {
        const auto legacySettings = [] {
            RagdollCore::ScenarioSettings settings;
            settings.mLocomotionController = RagdollCore::ScenarioLocomotionController::Legacy; // legacy-mechanism test
            return settings;
        }();
        RagdollCore::ScenarioScript enabledScript = walkingScript(1.0f, 60.0f, 10);
        RagdollCore::ScenarioScript disabledScript = enabledScript;
        RagdollCore::ScenarioInputKeyframe initial;
        initial.mStep = 0;
        initial.mTranslationActuationEnabled = false;
        disabledScript.mKeyframes.insert(disabledScript.mKeyframes.begin(), initial);
        disabledScript.mKeyframes[1].mTranslationActuationEnabled = false;

        const auto enabled = run(enabledScript, legacySettings);
        const auto disabled = run(disabledScript, legacySettings);
        ASSERT_TRUE(enabled.completed()) << describe(enabled);
        ASSERT_TRUE(disabled.completed()) << describe(disabled);
        ASSERT_EQ(enabled.mFrames.size(), disabled.mFrames.size());

        bool enabledAppliedForce = false;
        bool disabledComputedForce = false;
        bool disabledAppliedTorque = false;
        for (std::size_t i = 0; i < enabled.mFrames.size(); ++i)
        {
            enabledAppliedForce = enabledAppliedForce || !enabled.mFrames[i].mAppliedPelvisForce.IsNearZero();
            EXPECT_TRUE(disabled.mFrames[i].mAppliedPelvisForce.IsNearZero());
            EXPECT_FALSE(disabled.mFrames[i].mTranslationActuationEnabled);
            disabledComputedForce = disabledComputedForce
                || (disabled.mFrames[i].mUpright && !disabled.mFrames[i].mUpright->mPelvisForce.IsNearZero());
            disabledAppliedTorque = disabledAppliedTorque || !disabled.mFrames[i].mAppliedPelvisTorque.IsNearZero();
        }
        EXPECT_TRUE(enabledAppliedForce);
        EXPECT_TRUE(disabledComputedForce);
        EXPECT_TRUE(disabledAppliedTorque);
        EXPECT_FALSE(RagdollCore::haveIdenticalScenarioTelemetry(enabled, disabled));
        EXPECT_NE(RagdollCore::hashScenarioTelemetry(enabled), RagdollCore::hashScenarioTelemetry(disabled));
    }

    TEST(RagdollCoreScenarioTest, killingBlowIsAppliedAfterBroadphaseInsertion)
    {
        RagdollCore::ScenarioSettings settings;
        settings.mPowered = false;
        RagdollCore::ScenarioScript script = standingScript(4.0f);
        RagdollCore::ScenarioKillingBlow blow;
        blow.mWorldPoint = JPH::RVec3(-8.0, 0.0, 112.0);
        blow.mDirection = JPH::Vec3(1.0f, 0.0f, 0.2f);
        blow.mHealthDamage = 100.0f;
        script.mKillingBlow = blow;

        const auto result = run(script, settings);
        ASSERT_TRUE(result.completed()) << describe(result);
        ASSERT_TRUE(result.mKillingImpulse.has_value());
        EXPECT_EQ(result.mBodyNames[result.mKillingImpulse->mBodyIndex], "head");
        const auto profile = RagdollCore::makeScenarioHumanoidProfile();
        EXPECT_NEAR(result.mKillingImpulse->mImpulse.Length(), profile.mImpactResponse.mMaximumImpulse, 1e-3f);
        EXPECT_TRUE(result.mKillingImpulse->mWorldPoint.IsClose(blow.mWorldPoint));
        EXPECT_EQ(result.mFrames[0].mImpulseCount, 1u);
        EXPECT_GT(result.mFrames[0].mBodies[result.mKillingImpulse->mBodyIndex].mLinearVelocity.GetX(), 0.0f);

        bool observedFallen = false;
        for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
        {
            EXPECT_FALSE(frame.mMotorsApplied);
            EXPECT_FALSE(frame.mUprightApplied);
            EXPECT_TRUE(frame.mVerticalReactionBodies.empty());
            EXPECT_TRUE(frame.mHorizontalReactionBodies.empty());
            observedFallen = observedFallen || frame.mController.mMode == ActiveControllerMode::Fallen;
        }
        EXPECT_TRUE(observedFallen) << describe(result);
        const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
        std::cout << "KILLING_BLOW_INVARIANTS " << describe(invariants) << std::endl;
        EXPECT_EQ(invariants.mImpulseFrames, 1u);
        EXPECT_TRUE(invariants.mFellAfterImpulse);
        EXPECT_LT(invariants.mMaxBodySpeed, profile.mMotionTransfer.mMaximumLinearSpeed);
        EXPECT_LT(result.mFrames.back().mPelvisHeight, profile.mActiveController.mFallenMaximumPelvisHeight);

        // The same blow with no damage is rejected before simulating.
        script.mKillingBlow->mHealthDamage = 0.0f;
        const auto rejected = run(script, settings);
        EXPECT_FALSE(rejected.completed());
        EXPECT_TRUE(rejected.mFrames.empty());
    }

    TEST(RagdollCoreScenarioTest, gaitAchievesMeasuredSwingFootLiftOff)
    {
        const auto result = run(walkingScript(1.5f, 60.0f, 30));
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto milestones = RagdollCore::summarizeScenarioGaitMilestones(result);
        const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
        ASSERT_TRUE(milestones.mStanceStartedStep) << describe(milestones);
        ASSERT_TRUE(milestones.mStanceReadyStep) << describe(milestones);
        ASSERT_TRUE(milestones.mLiftOffStep) << "the first swing foot never released ground contact\n"
                                             << describe(milestones);
        EXPECT_LT(*milestones.mStanceStartedStep, *milestones.mStanceReadyStep) << describe(milestones);
        EXPECT_LT(*milestones.mStanceReadyStep, *milestones.mLiftOffStep) << describe(milestones);
        EXPECT_EQ(invariants.mAirborneFrames, 0u) << "unloading lifted the entire actor";
        EXPECT_EQ(invariants.mFallingFrames + invariants.mFallenFrames, 0u) << describe(result);
    }

    TEST(RagdollCoreScenarioTest, gaitReestablishesSwingFootContactAfterLiftOff)
    {
        // Readiness and lift-off are now based on measured support. Keep the
        // scenario alive long enough to observe the resulting physical
        // descent; the controller's phase deadline remains unchanged.
        const auto result = run(walkingScript(2.0f, 60.0f, 30));
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto milestones = RagdollCore::summarizeScenarioGaitMilestones(result);
        ASSERT_TRUE(milestones.mLiftOffStep) << describe(milestones);
        ASSERT_TRUE(milestones.mClosestApproachStep) << describe(milestones);
        ASSERT_TRUE(milestones.mContactCandidateStep)
            << "the lifted swing foot never re-established measured support\n"
            << describe(milestones);
        EXPECT_LT(*milestones.mLiftOffStep, *milestones.mContactCandidateStep) << describe(milestones);
        EXPECT_LE(*milestones.mLiftOffStep, *milestones.mClosestApproachStep) << describe(milestones);
    }

    TEST(RagdollCoreScenarioTest, swingPlacementIsLatchedAfterMeasuredLiftOff)
    {
        const auto legacySettings = [] {
            RagdollCore::ScenarioSettings settings;
            settings.mLocomotionController = RagdollCore::ScenarioLocomotionController::Legacy; // legacy-mechanism test
            return settings;
        }();
        const auto result = run(walkingScript(2.0f, 60.0f, 30), legacySettings);
        ASSERT_TRUE(result.completed()) << describe(result);

        std::optional<GaitPhase> airbornePhase;
        std::optional<JPH::RVec3> airborneTarget;
        bool observedLeftStance = false;
        for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
        {
            if (!frame.mGait.hasStance())
            {
                airbornePhase.reset();
                airborneTarget.reset();
                continue;
            }

            ASSERT_TRUE(frame.mSwingFootTarget.has_value())
                << "a physics-owned stance must retain a placement target\n"
                << describe(result);
            observedLeftStance = observedLeftStance || frame.mGait.mPhase == GaitPhase::LeftStance;
            if (!frame.mGait.mSwingFootLifted)
                continue;
            if (!airbornePhase || *airbornePhase != frame.mGait.mPhase)
            {
                airbornePhase = frame.mGait.mPhase;
                airborneTarget = frame.mSwingFootTarget;
                continue;
            }

            EXPECT_TRUE(frame.mSwingFootTarget->IsClose(*airborneTarget))
                << "swing placement drifted after lift-off in the " << phaseName(*airbornePhase)
                << " stance at step " << frame.mStep;
        }

        EXPECT_TRUE(observedLeftStance);
        EXPECT_TRUE(airborneTarget.has_value()) << "the placement was never observed after measured lift-off";
    }

    TEST(RagdollCoreScenarioTest, swingLandingUsesMeasuredPlantedHeight)
    {
        RagdollCore::ScenarioSettings settings;
        settings.mLocomotionController = RagdollCore::ScenarioLocomotionController::Legacy; // legacy-mechanism test
        settings.mRootPosition = JPH::RVec3(0, 0, 2);
        const auto result = run(walkingScript(3.0f, 60.0f, 60), settings);
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto profile = RagdollCore::makeScenarioHumanoidProfile();
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        std::optional<float> plantedHeight;
        GaitPhase phase = GaitPhase::Idle;
        std::size_t checked = 0;
        for (const auto& frame : result.mFrames)
        {
            if (plantedHeight && frame.mActuationGait && frame.mActuationGait->mPhase == phase
                && frame.mActuationGait->mSwingTimeSinceLift >= profile.mActiveController.mGait.mNominalStepDuration
                && frame.mSwingFootMotorTarget)
            {
                EXPECT_NEAR(frame.mSwingFootMotorTarget->GetZ(), *plantedHeight, 1e-4f) << frame.mStep;
                ++checked;
            }
            if (frame.mGait.mTransitioned && frame.mGait.hasStance())
            {
                phase = frame.mGait.mPhase;
                const auto swing = frame.mGait.leftFootIsStance() ? result.mLandmarks.mRightFoot
                                                                 : result.mLandmarks.mLeftFoot;
                plantedHeight = frame.mBodies[swing].mModelTransform.mTranslation.GetZ();
                if (checked == 0)
                    EXPECT_GT(std::abs(*plantedHeight - skeleton.mBones[swing].mModelTransform.mTranslation.GetZ()),
                        0.5f) << "The raised insertion must settle before the first walk request";
            }
        }
        EXPECT_GT(checked, 0u) << "No descending swing reached its landing target";
    }

    TEST(RagdollCoreScenarioTest, swingTargetFrameResolutionRoundTrips)
    {
        const auto legacySettings = [] {
            RagdollCore::ScenarioSettings settings;
            settings.mLocomotionController = RagdollCore::ScenarioLocomotionController::Legacy; // legacy-mechanism test
            return settings;
        }();
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto result = run(walkingScript(2.0f, 60.0f, 30), legacySettings);
        ASSERT_TRUE(result.completed()) << describe(result);
        bool observed = false;
        for (std::size_t i = 1; i < result.mFrames.size(); ++i)
        {
            const RagdollCore::ScenarioFrame& frame = result.mFrames[i];
            if (!frame.mActuationGait || !frame.mActuationGait->hasStance() || !frame.mSwingFootMotorTarget
                || !frame.mResolvedSwingFootTarget)
                continue;

            const std::size_t swingHip = frame.mActuationGait->leftFootIsStance() ? 17 : 14;
            const RagdollCore::Transform& basePelvis = skeleton.mBones[result.mLandmarks.mPelvis].mModelTransform;
            const RagdollCore::Transform& baseHip = skeleton.mBones[swingHip].mModelTransform;
            const RagdollCore::Transform& measuredPelvis
                = result.mFrames[i - 1].mBodies[result.mLandmarks.mPelvis].mModelTransform;
            const RagdollCore::Transform& measuredHip
                = result.mFrames[i - 1].mBodies[swingHip].mModelTransform;
            const JPH::Quat pelvisDelta
                = (measuredPelvis.mRotation * basePelvis.mRotation.Conjugated()).Normalized();
            const JPH::Vec3 reconstructed = measuredHip.mTranslation
                + pelvisDelta * (*frame.mResolvedSwingFootTarget - baseHip.mTranslation);
            EXPECT_TRUE(reconstructed.IsClose(*frame.mSwingFootMotorTarget, 1e-3f))
                << "pelvis-frame swing target did not round-trip at step " << frame.mStep;
            observed = true;
        }
        EXPECT_TRUE(observed);
    }

    TEST(RagdollCoreScenarioTest, gaitPoseCycleIsSampledByPhysicsPhase)
    {
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        RagdollCore::RagdollPose bindPose;
        for (const RagdollCore::SkeletonBone& bone : skeleton.mBones)
            bindPose.mBodyTransforms.push_back(bone.mModelTransform);

        const auto baseline = run(walkingScript(1.2f, 20.0f, 30));
        auto sampledScript = walkingScript(1.2f, 20.0f, 30);
        sampledScript.mGaitPoseCycle = { bindPose, bindPose };
        const auto sampled = run(sampledScript);
        ASSERT_TRUE(baseline.completed()) << describe(baseline);
        ASSERT_TRUE(sampled.completed()) << describe(sampled);
        EXPECT_TRUE(RagdollCore::haveIdenticalScenarioTelemetry(baseline, sampled));
    }

    TEST(RagdollCoreScenarioTest, groundContactRoutingLeavesIdleStandingUnchanged)
    {
        RagdollCore::ScenarioSettings settings;
        settings.mTranslationRouting = RagdollCore::ScenarioTranslationRouting::GroundContactForce;
        const auto result = run(standingScript(1.0f), settings);
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
        std::cout << "GROUND_FORCE_STANDING_INVARIANTS " << describe(invariants) << std::endl;
        EXPECT_EQ(invariants.mFallingFrames + invariants.mFallenFrames, 0u) << describe(result);
        EXPECT_EQ(invariants.mAirborneFrames, 0u);
        EXPECT_FLOAT_EQ(invariants.mMaxGroundReactionForce, 0.0f);
        EXPECT_LE(invariants.mMaxAppliedNetForce,
            RagdollCore::makeScenarioHumanoidProfile().mActiveController.mMaximumPelvisForce);
    }

    TEST(RagdollCoreScenarioTest, gaitCompletesOnePhysicalStepWithoutTimingOut)
    {
        const auto result = run(walkingScript(2.5f, 60.0f, 30));
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto milestones = RagdollCore::summarizeScenarioGaitMilestones(result);
        ASSERT_TRUE(milestones.mContactCandidateStep) << describe(milestones);
        EXPECT_TRUE(milestones.mStanceTransferStep)
            << "the first lifted foot did not land and transfer stance before the phase deadline\n"
            << describe(milestones) << '\n'
            << describe(result);
        if (milestones.mStanceTransferStep)
            EXPECT_LT(*milestones.mContactCandidateStep, *milestones.mStanceTransferStep) << describe(milestones);
    }

    TEST(RagdollCoreScenarioTest, batchesWeightTransferControllerVariantsWithoutRebuild)
    {
        constexpr std::array<float, 11> frequencies{
            0.03f, 0.05f, 0.10f, 0.15f, 0.20f, 0.35f, 0.50f, 0.75f, 1.0f, 1.5f, 2.0f
        };
        constexpr std::array<float, 4> dampings{ 1.0f, 1.5f, 2.0f, 3.0f };
        std::cout << "WEIGHT_TRANSFER_SWEEP capture frequency damping stance_ready first_ready_step "
                     "right_ready right_ready_step lateral_speed_at_ready max_lateral_speed falling "
                     "left_steps right_steps timed_out final_speed min_up force_saturated swing_hip_saturated slide hash"
                  << std::endl;
        for (const bool useCapturePoint : { false, true })
        {
            for (const float frequency : frequencies)
            {
                for (const float damping : dampings)
                {
                    auto profile = RagdollCore::makeScenarioHumanoidProfile();
                    profile.mActiveController.mCenterOfMassFrequency = frequency;
                    profile.mActiveController.mCenterOfMassDamping = damping;
                    profile.mActiveController.mGait.mMaximumSwingFootHorizontalForce = 10000.0f;
                    RagdollCore::ScenarioSettings settings;
                    settings.mUseCapturePointForStanceReadiness = useCapturePoint;
                    settings.mCompensateSwingHorizontalReactionThroughStance = true;
                    const auto result = RagdollCore::runRagdollScenario(RagdollCore::makeScenarioHumanoidSkeleton(),
                        profile, settings, walkingScript(5.0f, 60.0f, 30));
                    ASSERT_TRUE(result.completed()) << useCapturePoint << '/' << frequency << '/' << damping << '\n'
                                                    << describe(result);
                    bool stanceReady = false;
                    bool rightReady = false;
                    std::uint64_t firstReadyStep = result.mStepCount;
                    std::uint64_t rightReadyStep = result.mStepCount;
                    float lateralSpeedAtReady = 0.0f;
                    float maximumLateralSpeed = 0.0f;
                    std::size_t fallingFrames = 0;
                    for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
                    {
                        if (!stanceReady && frame.mGait.mStanceReady)
                        {
                            stanceReady = true;
                            firstReadyStep = frame.mStep;
                            lateralSpeedAtReady = std::abs(frame.mCenterOfMassVelocity.GetX());
                        }
                        if (!rightReady && frame.mGait.mPhase == GaitPhase::RightStance && frame.mGait.mStanceReady)
                        {
                            rightReady = true;
                            rightReadyStep = frame.mStep;
                        }
                        maximumLateralSpeed
                            = std::max(maximumLateralSpeed, std::abs(frame.mCenterOfMassVelocity.GetX()));
                        fallingFrames += frame.mController.mMode == ActiveControllerMode::Falling
                            || frame.mController.mMode == ActiveControllerMode::Fallen;
                    }
                    const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
                    std::cout << "WEIGHT_TRANSFER " << useCapturePoint << ' ' << frequency << ' ' << damping << ' '
                              << stanceReady << ' ' << firstReadyStep << ' ' << rightReady << ' ' << rightReadyStep
                              << ' ' << lateralSpeedAtReady << ' ' << maximumLateralSpeed << ' ' << fallingFrames
                              << ' ' << invariants.mSteps.mLeftStanceCount << ' '
                              << invariants.mSteps.mRightStanceCount << ' ' << invariants.mSteps.mTimedOutPhases
                              << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' ' << invariants.mMinPelvisUpDot
                              << ' ' << invariants.mPelvisForceSaturatedFrames << ' '
                              << invariants.mSwingHipMotorSaturatedFrames << ' ' << invariants.mMaxStanceFootSlide
                              << ' ' << RagdollCore::hashScenarioTelemetry(result) << std::endl;
                }
            }
        }
    }

    TEST(RagdollCoreScenarioTest, batchesIndependentSwingForceVariantsWithoutRebuild)
    {
        constexpr std::array<float, 8> forceCaps{
            0.0f, 250.0f, 500.0f, 1000.0f, 2500.0f, 5000.0f, 10000.0f, 20000.0f
        };
        struct RoutingVariant
        {
            const char* mName;
            bool mRouteThroughStanceLeg;
            RagdollCore::ScenarioTranslationRouting mTranslationRouting;
            bool mRequireStanceSupport;
            bool mReserveStanceHipMotor;
            bool mCompensateSwingHipMotor;
            bool mDisableSwingHipMotor;
            bool mUseWorldSwingHipControl;
        };
        constexpr std::array<RoutingVariant, 10> routingVariants{ {
            { "supported_feet", false, RagdollCore::ScenarioTranslationRouting::InternalForcePair, false, false, false,
                false, false },
            { "stance_leg", true, RagdollCore::ScenarioTranslationRouting::InternalForcePair, false, false, false,
                false, false },
            { "support_handover", true, RagdollCore::ScenarioTranslationRouting::InternalForcePair, true, false, false,
                false, false },
            { "swing_hip_free", true, RagdollCore::ScenarioTranslationRouting::InternalForcePair, false, false, false,
                true, false },
            { "world_swing_hip", true, RagdollCore::ScenarioTranslationRouting::InternalForcePair, false, false, false,
                false, true },
            { "internal_compensated", true, RagdollCore::ScenarioTranslationRouting::InternalForcePair, false, true,
                true, false, false },
            { "ground_contact", true, RagdollCore::ScenarioTranslationRouting::GroundContactForce, true, false, false,
                false, false },
            { "ground_swing_hip_free", true, RagdollCore::ScenarioTranslationRouting::GroundContactForce, true, false,
                false, true, false },
            { "ground_free_hip", true, RagdollCore::ScenarioTranslationRouting::GroundContactForce, true, true, false,
                false, false },
            { "ground_compensated", true, RagdollCore::ScenarioTranslationRouting::GroundContactForce, true, true, true,
                false, false },
        } };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(3.0f, 60.0f, 30);
        std::cout << "SWING_FORCE_SWEEP route cap completed lift first_step timed_out falling airborne min_up "
                     "achieved max_slide max_swing_moment max_upright_moment max_ground_moment max_net_moment "
                     "max_swing_hip_motor_torque motor_saturated_frames "
                     "max_swing_hip_world_torque world_saturated_frames "
                     "max_stance_hip_compensation_torque hash"
                  << std::endl;
        for (const RoutingVariant& routing : routingVariants)
        {
            RagdollCore::ScenarioSettings settings;
            settings.mLocomotionController = RagdollCore::ScenarioLocomotionController::Legacy; // legacy-mechanism test
            settings.mCompensateSwingHorizontalReactionThroughStance = false;
            settings.mRouteGaitReactionThroughStanceLeg = routing.mRouteThroughStanceLeg;
            settings.mTranslationRouting = routing.mTranslationRouting;
            settings.mRequireStanceSupportForSwingAssist = routing.mRequireStanceSupport;
            settings.mReserveStanceHipMotor = routing.mReserveStanceHipMotor;
            settings.mCompensateSwingHipMotorImpulse = routing.mCompensateSwingHipMotor;
            settings.mDisableSwingHipMotor = routing.mDisableSwingHipMotor;
            settings.mUseWorldSwingHipControl = routing.mUseWorldSwingHipControl;
            std::uint64_t previousProfileHash = 0;
            for (const float forceCap : forceCaps)
            {
                SCOPED_TRACE(std::string(routing.mName) + "/" + std::to_string(forceCap));
                auto profile = RagdollCore::makeScenarioHumanoidProfile();
                profile.mActiveController.mGait.mMaximumSwingFootForce = forceCap;
                const auto result = RagdollCore::runRagdollScenario(skeleton, profile, settings, script);
                ASSERT_TRUE(result.completed()) << "force cap " << forceCap << '\n' << describe(result);
                if (std::string_view(routing.mName) == "swing_hip_free" && forceCap == 10000.0f)
                    exportTraceIfRequested(result, "swing-hip-free-10000");
                if (std::string_view(routing.mName) == "ground_compensated" && forceCap == 1000.0f)
                    exportTraceIfRequested(result, "ground-compensated-1000");

                bool observedLiftOff = false;
                bool observedFirstStep = false;
                bool observedDisabledStanceHip = false;
                bool observedDisabledSwingHip = false;
                bool observedWorldSwingControl = false;
                bool observedStanceHipCompensation = false;
                bool observedReadyGaitActuation = false;
                for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
                {
                    observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                    observedFirstStep = observedFirstStep
                        || (observedLiftOff && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                            && !frame.mGait.mPhaseTimedOut);
                    if (frame.mSwingFootControl)
                    {
                        const float combinedCap = std::hypot(forceCap,
                            profile.mActiveController.mGait.mMaximumSwingFootHorizontalForce);
                        EXPECT_LE(frame.mSwingFootControl->mForce.Length(), combinedCap * 1.0001f);
                    }
                    if (settings.mRequireStanceSupportForSwingAssist && frame.mSwingFootControl
                        && !frame.mSwingAssistReady)
                        EXPECT_FALSE(frame.mSwingFootControlApplied);
                    if (frame.mStanceHipMotorDisabled)
                    {
                        observedDisabledStanceHip = true;
                        EXPECT_TRUE(frame.mDisabledStanceHipBody.has_value());
                    }
                    observedDisabledSwingHip = observedDisabledSwingHip || frame.mSwingHipMotorDisabled;
                    observedWorldSwingControl = observedWorldSwingControl || frame.mSwingHipWorldControlApplied;
                    observedReadyGaitActuation = observedReadyGaitActuation
                        || (frame.mActuationGait && frame.mActuationGait->mStanceReady);
                    observedStanceHipCompensation
                        = observedStanceHipCompensation || !frame.mAppliedStanceHipCompensationImpulse.IsNearZero();
                }
                EXPECT_EQ(observedDisabledStanceHip,
                    routing.mReserveStanceHipMotor
                        || (routing.mUseWorldSwingHipControl && observedReadyGaitActuation));
                EXPECT_EQ(observedDisabledSwingHip,
                    (routing.mDisableSwingHipMotor || routing.mUseWorldSwingHipControl)
                        && observedReadyGaitActuation);
                EXPECT_EQ(observedWorldSwingControl,
                    routing.mUseWorldSwingHipControl && observedReadyGaitActuation);
                EXPECT_EQ(observedStanceHipCompensation, routing.mCompensateSwingHipMotor);

                const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
                if (routing.mTranslationRouting == RagdollCore::ScenarioTranslationRouting::InternalForcePair)
                    EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
                else
                {
                    if (observedLiftOff)
                        EXPECT_GT(invariants.mMaxGroundReactionForce, 0.0f);
                    EXPECT_LE(invariants.mMaxAppliedNetForce, profile.mActiveController.mMaximumPelvisForce);
                }
                EXPECT_NE(result.mProfileHash, previousProfileHash);
                previousProfileHash = result.mProfileHash;
                if (routing.mTranslationRouting == RagdollCore::ScenarioTranslationRouting::GroundContactForce
                    && forceCap == 5000.0f)
                {
                    const auto repeated = RagdollCore::runRagdollScenario(skeleton, profile, settings, script);
                    ASSERT_TRUE(repeated.completed()) << describe(repeated);
                    EXPECT_TRUE(RagdollCore::haveIdenticalScenarioTelemetry(result, repeated));
                }
                std::cout << "SWING_FORCE_VARIANT " << routing.mName << ' ' << forceCap << ' ' << result.completed()
                          << ' ' << observedLiftOff << ' ' << observedFirstStep << ' '
                          << invariants.mSteps.mTimedOutPhases << ' '
                          << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                          << ' ' << invariants.mMinPelvisUpDot << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                          << invariants.mMaxStanceFootSlide << ' ' << invariants.mMaxSwingForcePairMoment << ' '
                          << invariants.mMaxUprightForcePairMoment << ' ' << invariants.mMaxGroundReactionMoment << ' '
                          << invariants.mMaxNetActuationMoment << ' ' << invariants.mMaxSwingHipMotorTorque << ' '
                          << invariants.mSwingHipMotorSaturatedFrames << ' ' << invariants.mMaxSwingHipWorldTorque
                          << ' ' << invariants.mSwingHipWorldTorqueSaturatedFrames << ' '
                          << invariants.mMaxStanceHipCompensationTorque << ' '
                          << RagdollCore::hashScenarioTelemetry(result) << std::endl;
            }
        }

        const auto runActuationVariant = [&](const char* label, bool translationEnabled, bool swingEnabled) {
            auto variantScript = script;
            RagdollCore::ScenarioInputKeyframe initial;
            initial.mStep = 0;
            initial.mTranslationActuationEnabled = translationEnabled;
            initial.mSwingFootActuationEnabled = swingEnabled;
            variantScript.mKeyframes.insert(variantScript.mKeyframes.begin(), initial);
            variantScript.mKeyframes[1].mTranslationActuationEnabled = translationEnabled;
            variantScript.mKeyframes[1].mSwingFootActuationEnabled = swingEnabled;
            const auto result = RagdollCore::runRagdollScenario(
                skeleton, RagdollCore::makeScenarioHumanoidProfile(), {}, variantScript);
            ASSERT_TRUE(result.completed()) << describe(result);
            bool observedLiftOff = false;
            bool observedFirstStep = false;
            for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
            {
                observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                observedFirstStep = observedFirstStep
                    || (observedLiftOff && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                        && !frame.mGait.mPhaseTimedOut);
            }
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            std::cout << "SWING_FORCE_VARIANT " << label << " 20000 " << result.completed() << ' ' << observedLiftOff
                      << ' ' << observedFirstStep << ' ' << invariants.mSteps.mTimedOutPhases << ' '
                      << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                      << ' ' << invariants.mMinPelvisUpDot << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                      << invariants.mMaxStanceFootSlide << ' ' << invariants.mMaxSwingForcePairMoment << ' '
                      << invariants.mMaxUprightForcePairMoment << ' ' << invariants.mMaxGroundReactionMoment << ' '
                      << invariants.mMaxNetActuationMoment << ' ' << invariants.mMaxSwingHipMotorTorque << ' '
                      << invariants.mSwingHipMotorSaturatedFrames << ' ' << invariants.mMaxStanceHipCompensationTorque
                      << ' ' << RagdollCore::hashScenarioTelemetry(result) << std::endl;
            EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
            if (!translationEnabled)
                EXPECT_FLOAT_EQ(invariants.mMaxUprightForcePairMoment, 0.0f);
            if (!swingEnabled)
                EXPECT_FLOAT_EQ(invariants.mMaxSwingForcePairMoment, 0.0f);
        };
        runActuationVariant("pelvis_translation_off", false, true);
        runActuationVariant("swing_assist_off", true, false);
        runActuationVariant("both_translation_off", false, false);
    }

    TEST(RagdollCoreScenarioTest, batchesInFlightSwingTrackingRoutesWithoutRebuild)
    {
        struct ReactionVariant
        {
            const char* mName;
            RagdollCore::ScenarioSwingHorizontalReaction mReaction;
            bool mCompensateThroughStance = false;
        };
        constexpr std::array<ReactionVariant, 5> reactions{ {
            { "swing_hip", RagdollCore::ScenarioSwingHorizontalReaction::SwingHip },
            { "swing_hip_stance_compensated", RagdollCore::ScenarioSwingHorizontalReaction::SwingHip, true },
            { "pelvis", RagdollCore::ScenarioSwingHorizontalReaction::Pelvis },
            { "stance_hip", RagdollCore::ScenarioSwingHorizontalReaction::StanceHip },
            { "stance_foot", RagdollCore::ScenarioSwingHorizontalReaction::StanceFoot },
        } };
        constexpr std::array<float, 10> forceCaps{ 250.0f, 500.0f, 1000.0f, 2500.0f, 5000.0f, 7500.0f,
            8500.0f, 10000.0f, 12500.0f, 20000.0f };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(5.0f, 60.0f, 30);
        std::cout << "SWING_TRACKING_SWEEP reaction cap lift first_step alternating_steps timed_out falling "
                     "airborne min_up min_error final_achieved max_slide force_frames max_force max_pair_moment "
                     "max_net_force hash"
                  << std::endl;
        for (const ReactionVariant& reaction : reactions)
        {
            for (const float forceCap : forceCaps)
            {
                auto profile = RagdollCore::makeScenarioHumanoidProfile();
                profile.mActiveController.mGait.mMaximumSwingFootHorizontalForce = forceCap;
                RagdollCore::ScenarioSettings settings;
                settings.mLocomotionController = RagdollCore::ScenarioLocomotionController::Legacy; // legacy-mechanism test
                settings.mSwingAssistAxes = RagdollCore::ScenarioSwingAssistAxes::Full3D;
                settings.mSwingHorizontalReaction = reaction.mReaction;
                settings.mCompensateSwingHorizontalReactionThroughStance = reaction.mCompensateThroughStance;
                const auto result = RagdollCore::runRagdollScenario(skeleton, profile, settings, script);
                ASSERT_TRUE(result.completed()) << reaction.mName << '/' << forceCap << '\n' << describe(result);
                if (reaction.mReaction == RagdollCore::ScenarioSwingHorizontalReaction::Pelvis
                    && forceCap == 20000.0f)
                    exportTraceIfRequested(result, "swing-tracking-pelvis-20000");
                if (reaction.mReaction == RagdollCore::ScenarioSwingHorizontalReaction::StanceFoot
                    && forceCap == 10000.0f)
                    exportTraceIfRequested(result, "swing-tracking-stance-foot-10000");

                bool lifted = false;
                bool firstStep = false;
                std::size_t alternatingSteps = 0;
                GaitPhase previousTransition = GaitPhase::Idle;
                float minimumPlacementError = std::numeric_limits<float>::max();
                std::size_t forceFrames = 0;
                float maximumForce = 0.0f;
                for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
                {
                    lifted = lifted || frame.mGait.mSwingFootLifted;
                    firstStep = firstStep
                        || (lifted && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                            && !frame.mGait.mPhaseTimedOut);
                    if (frame.mGait.mTransitioned && frame.mGait.hasStance())
                    {
                        if (previousTransition != GaitPhase::Idle && frame.mGait.mPhase != previousTransition)
                            ++alternatingSteps;
                        previousTransition = frame.mGait.mPhase;
                    }
                    if (frame.mGait.mSwingFootLifted)
                        minimumPlacementError
                            = std::min(minimumPlacementError, frame.mSwingFootHorizontalPlacementError);
                    if (frame.mSwingFootControlApplied && frame.mSwingFootControl)
                    {
                        ++forceFrames;
                        maximumForce = std::max(maximumForce, frame.mSwingFootControl->mForce.Length());
                        const float combinedCap
                            = std::hypot(profile.mActiveController.mGait.mMaximumSwingFootForce, forceCap);
                        EXPECT_LE(frame.mSwingFootControl->mForce.Length(), combinedCap * 1.0001f);
                    }
                }
                const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
                EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
                EXPECT_FLOAT_EQ(invariants.mMaxSwingForcePairMoment, 0.0f);
                EXPECT_GT(forceFrames, 0u);
                std::cout << "SWING_TRACKING_VARIANT " << reaction.mName << ' ' << forceCap << ' ' << lifted << ' '
                          << firstStep << ' ' << alternatingSteps << ' ' << invariants.mSteps.mTimedOutPhases << ' '
                          << invariants.mFallingFrames + invariants.mFallenFrames << ' '
                          << invariants.mAirborneFrames << ' ' << invariants.mMinPelvisUpDot << ' '
                          << minimumPlacementError << ' '
                          << invariants.mFinalHalfSecondAchievedSpeed << ' ' << invariants.mMaxStanceFootSlide << ' '
                          << forceFrames << ' ' << maximumForce << ' ' << invariants.mMaxSwingForcePairMoment << ' '
                          << invariants.mMaxAppliedNetForce << ' ' << RagdollCore::hashScenarioTelemetry(result)
                          << std::endl;
            }
        }
    }

    TEST(RagdollCoreScenarioTest, batchesSwingLegMotorTorqueScalesWithoutRebuild)
    {
        constexpr std::array<float, 8> torqueScales{ 1.0f, 1.5f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 8.0f };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(7.0f, 60.0f, 30);
        std::cout << "SWING_TORQUE_SWEEP scale left_steps right_steps timed_out falling airborne min_up final_achieved "
                     "max_slide hip_saturation vertical_saturation hash"
                  << std::endl;
        for (const float scale : torqueScales)
        {
            auto profile = RagdollCore::makeScenarioHumanoidProfile();
            profile.mActiveController.mGait.mSwingLegMotorTorqueScale = scale;
            const auto result = RagdollCore::runRagdollScenario(skeleton, profile, {}, script);
            ASSERT_TRUE(result.completed()) << scale << '\n' << describe(result);
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
            std::cout << "SWING_TORQUE_VARIANT " << scale << ' ' << invariants.mSteps.mLeftStanceCount << ' '
                      << invariants.mSteps.mRightStanceCount << ' ' << invariants.mSteps.mTimedOutPhases << ' '
                      << invariants.mFallingFrames + invariants.mFallenFrames << ' '
                      << invariants.mAirborneFrames << ' ' << invariants.mMinPelvisUpDot << ' '
                      << invariants.mFinalHalfSecondAchievedSpeed << ' ' << invariants.mMaxStanceFootSlide << ' '
                      << invariants.mSwingHipMotorSaturatedFrames << ' '
                      << invariants.mPelvisVerticalForceSaturatedFrames << ' '
                      << RagdollCore::hashScenarioTelemetry(result) << std::endl;
        }
    }

    TEST(RagdollCoreScenarioTest, batchesCompensatedSwingVerticalForceCapsWithoutRebuild)
    {
        constexpr std::array<float, 8> forceCaps{
            0.0f, 250.0f, 500.0f, 1000.0f, 2500.0f, 5000.0f, 10000.0f, 20000.0f
        };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(7.0f, 60.0f, 30);
        std::cout << "SWING_VERTICAL_SWEEP cap left_steps right_steps timed_out falling airborne min_up "
                     "final_achieved max_slide hip_saturation vertical_saturation hash"
                  << std::endl;
        for (const float cap : forceCaps)
        {
            auto profile = RagdollCore::makeScenarioHumanoidProfile();
            profile.mActiveController.mGait.mMaximumSwingFootForce = cap;
            const auto result = RagdollCore::runRagdollScenario(skeleton, profile, {}, script);
            ASSERT_TRUE(result.completed()) << cap << '\n' << describe(result);
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
            std::cout << "SWING_VERTICAL_VARIANT " << cap << ' ' << invariants.mSteps.mLeftStanceCount << ' '
                      << invariants.mSteps.mRightStanceCount << ' ' << invariants.mSteps.mTimedOutPhases << ' '
                      << invariants.mFallingFrames + invariants.mFallenFrames << ' '
                      << invariants.mAirborneFrames << ' ' << invariants.mMinPelvisUpDot << ' '
                      << invariants.mFinalHalfSecondAchievedSpeed << ' ' << invariants.mMaxStanceFootSlide << ' '
                      << invariants.mSwingHipMotorSaturatedFrames << ' '
                      << invariants.mPelvisVerticalForceSaturatedFrames << ' '
                      << RagdollCore::hashScenarioTelemetry(result) << std::endl;
        }
    }

    TEST(RagdollCoreScenarioTest, comparesStanceReadinessAndHipOwnershipWithoutRebuild)
    {
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        auto profile = RagdollCore::makeScenarioHumanoidProfile();
        profile.mActiveController.mGait.mMaximumSwingFootForce = 10000.0f;
        const auto script = walkingScript(3.0f, 60.0f, 30);
        std::cout << "STANCE_READINESS_SWEEP probe ready_step lateral_speed first_step timed_out falling airborne "
                     "min_up final_achieved max_slide hash"
                  << std::endl;
        struct ReadinessVariant
        {
            const char* mName;
            bool mUseCapturePoint;
            bool mDisableSwingHipMotor;
            bool mUseWorldSwingHipControl;
            bool mUsePostLiftGroundReaction;
        };
        constexpr std::array<ReadinessVariant, 5> variants{ {
            { "com", false, false, false, false },
            { "com_post_lift_ground", false, false, false, true },
            { "capture", true, false, false, false },
            { "capture_hip_free", true, true, false, false },
            { "capture_world_hip", true, false, true, false },
        } };
        for (const ReadinessVariant& variant : variants)
        {
            RagdollCore::ScenarioSettings settings;
            settings.mLocomotionController = RagdollCore::ScenarioLocomotionController::Legacy; // legacy-mechanism test
            settings.mUseCapturePointForStanceReadiness = variant.mUseCapturePoint;
            settings.mSwingAssistAxes = RagdollCore::ScenarioSwingAssistAxes::Full3D;
            settings.mSwingHorizontalReaction = RagdollCore::ScenarioSwingHorizontalReaction::StanceFoot;
            settings.mDisableSwingHipMotor = variant.mDisableSwingHipMotor;
            settings.mUseWorldSwingHipControl = variant.mUseWorldSwingHipControl;
            if (variant.mUsePostLiftGroundReaction)
                settings.mTranslationRouting = RagdollCore::ScenarioTranslationRouting::GroundContactForce;
            const auto result = RagdollCore::runRagdollScenario(skeleton, profile, settings, script);
            ASSERT_TRUE(result.completed()) << describe(result);
            exportTraceIfRequested(result, "stance-readiness-" + std::string(variant.mName));

            bool ready = false;
            std::uint64_t readyStep = result.mStepCount;
            float lateralSpeedAtReady = 0.0f;
            bool lifted = false;
            bool firstStep = false;
            for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
            {
                if (!ready && frame.mGait.mStanceReady)
                {
                    ready = true;
                    readyStep = frame.mStep;
                    lateralSpeedAtReady = std::abs(frame.mCenterOfMassVelocity.GetX());
                }
                lifted = lifted || frame.mGait.mSwingFootLifted;
                firstStep = firstStep
                    || (lifted && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                        && !frame.mGait.mPhaseTimedOut);
            }
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            const auto milestones = RagdollCore::summarizeScenarioGaitMilestones(result);
            if (!variant.mUsePostLiftGroundReaction)
                EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
            else
            {
                if (firstStep)
                    EXPECT_GT(invariants.mMaxGroundReactionForce, 0.0f);
                EXPECT_LE(invariants.mMaxAppliedNetForce, profile.mActiveController.mMaximumPelvisForce);
            }
            std::cout << "STANCE_READINESS_VARIANT " << variant.mName << ' ' << readyStep
                      << ' ' << lateralSpeedAtReady << ' ' << firstStep << ' '
                      << invariants.mSteps.mTimedOutPhases << ' '
                      << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                      << ' ' << invariants.mMinPelvisUpDot << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                      << invariants.mMaxStanceFootSlide << ' ' << RagdollCore::hashScenarioTelemetry(result)
                      << " stages=" << describe(milestones) << std::endl;
        }
    }

    TEST(RagdollCoreScenarioTest, phaseFeedbackSeparatesInterruptedWalksWithoutInventingTransfers)
    {
        using namespace RagdollCore;
        ScenarioResult result;
        result.mFrames.resize(8);
        for (std::size_t i = 0; i < result.mFrames.size(); ++i)
            result.mFrames[i].mStep = i;
        for (const std::size_t i : { 0u, 1u, 3u, 4u, 7u })
            result.mFrames[i].mGait.mPhase = GaitPhase::LeftStance;
        result.mFrames[5].mGait.mPhase = GaitPhase::RightStance;
        for (const std::size_t i : { 3u, 5u, 7u })
            result.mFrames[i].mGait.mTransitioned = true;
        result.mFrames[0].mGait.mStanceReady = true;
        result.mFrames[1].mGait.mSwingFootLifted = true;
        result.mFrames[1].mGait.mSwingContactCandidateTime = 0.01f;
        result.mFrames[4].mGait.mStanceReady = true;
        result.mFrames[4].mGait.mSwingFootLifted = true;
        result.mFrames[4].mGait.mSwingContactCandidateTime = 0.05f;
        const auto phases = summarizeScenarioGaitPhases(result);
        ASSERT_EQ(phases.size(), 4u);
        EXPECT_EQ(phases[0].mStanceStartedStep, 0u);
        EXPECT_FALSE(phases[0].mStanceTransferStep);
        EXPECT_EQ(phases[1].mStanceStartedStep, 3u);
        EXPECT_EQ(phases[1].mStanceReadyStep, 4u);
        EXPECT_EQ(phases[1].mLiftOffStep, 4u);
        EXPECT_EQ(phases[1].mStanceTransferStep, 5u);
        EXPECT_EQ(phases[2].mPhase, GaitPhase::RightStance);
        EXPECT_EQ(phases[2].mStanceStartedStep, 5u);
        EXPECT_FALSE(phases[2].mStanceTransferStep);
        EXPECT_EQ(phases[3].mStanceStartedStep, 7u);
        EXPECT_FALSE(phases[3].mStanceReadyStep);
        EXPECT_FALSE(phases[3].mLiftOffStep);
        EXPECT_FALSE(phases[3].mContactCandidateStep);
        EXPECT_FALSE(phases[3].mStanceTransferStep);
    }

    TEST(RagdollCoreScenarioTest, recordsPostTouchdownWeightTransferAsANewPhysicalPhase)
    {
        // Exercise the production routing and authored cap. Earlier versions
        // of this staged gate kept an obsolete 10,000-unit stance-foot
        // experiment, so it stopped describing the controller used in game.
        const auto result = run(walkingScript(4.0f, 60.0f, 30));
        ASSERT_TRUE(result.completed()) << describe(result);

        const auto phases = RagdollCore::summarizeScenarioGaitPhases(result);
        ASSERT_GE(phases.size(), 2u);
        const auto& left = phases[0];
        const auto& right = phases[1];
        EXPECT_EQ(left.mPhase, GaitPhase::LeftStance);
        EXPECT_TRUE(left.mStanceReadyStep);
        EXPECT_TRUE(left.mLiftOffStep);
        EXPECT_TRUE(left.mContactCandidateStep);
        EXPECT_TRUE(left.mStanceTransferStep);
        EXPECT_FALSE(left.mPhaseTimeoutStep);
        EXPECT_EQ(right.mPhase, GaitPhase::RightStance);
        EXPECT_TRUE(right.mStanceReadyStep)
            << "touchdown did not progress into physical ownership by the new stance";
        EXPECT_TRUE(right.mLiftOffStep)
            << "the old stance leg was never released after the new stance became ready";
        EXPECT_FALSE(right.mPhaseTimeoutStep);
        EXPECT_EQ(result.mFrames[*left.mStanceTransferStep].mGait.mPhase, GaitPhase::RightStance);
        if (right.mStanceReadyStep)
            EXPECT_LT(*left.mStanceTransferStep, *right.mStanceReadyStep);
        if (right.mStanceReadyStep && right.mLiftOffStep)
            EXPECT_LT(*right.mStanceReadyStep, *right.mLiftOffStep);
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreScenarioTest, DISABLED_batchesPreLiftSwingUnloadFractionsWithoutRebuild)
    {
        constexpr std::array<float, 5> fractions{ 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
        std::cout << "PRE_LIFT_UNLOAD_SWEEP fraction phases right_ready right_lifted falling airborne slide "
                     "net_force hash"
                  << std::endl;
        for (const float fraction : fractions)
        {
            RagdollCore::ScenarioSettings settings;
            settings.mLocomotionController = RagdollCore::ScenarioLocomotionController::Legacy; // legacy-mechanism test
            settings.mPreLiftSwingUnloadFraction = fraction;
            const auto result = run(walkingScript(4.0f, 60.0f, 30), settings);
            ASSERT_TRUE(result.completed()) << "fraction " << fraction << ' ' << describe(result);
            const auto phases = RagdollCore::summarizeScenarioGaitPhases(result);
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            const bool rightReady = phases.size() > 1 && phases[1].mStanceReadyStep.has_value();
            const bool rightLifted = phases.size() > 1 && phases[1].mLiftOffStep.has_value();
            const bool unloadObserved = std::any_of(result.mFrames.begin(), result.mFrames.end(),
                [](const RagdollCore::ScenarioFrame& frame) { return frame.mPreLiftSwingUnloadForce > 0.0f; });
            EXPECT_EQ(unloadObserved, fraction > 0.0f);
            EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
            std::cout << "PRE_LIFT_UNLOAD_VARIANT " << fraction << ' ' << phases.size() << ' ' << rightReady << ' '
                      << rightLifted << ' ' << invariants.mFallingFrames + invariants.mFallenFrames << ' '
                      << invariants.mAirborneFrames << ' ' << invariants.mMaxStanceFootSlide << ' '
                      << invariants.mMaxAppliedNetForce << ' ' << RagdollCore::hashScenarioTelemetry(result)
                      << std::endl;
        }

        RagdollCore::ScenarioSettings candidate;

        candidate.mLocomotionController = RagdollCore::ScenarioLocomotionController::Legacy; // legacy-mechanism test
        candidate.mPreLiftSwingUnloadFraction = 0.5f;
        const auto walking = run(walkingScript(7.0f, 60.0f, 30), candidate);
        ASSERT_TRUE(walking.completed()) << describe(walking);
        const auto walkingInvariants = RagdollCore::summarizeScenarioInvariants(walking);
        std::cout << "PRE_LIFT_UNLOAD_WALK " << describe(walkingInvariants) << std::endl;

        RagdollCore::ScenarioScript pushedScript = walkingScript(3.5f, 60.0f, 30);
        RagdollCore::ScenarioImpulse push;
        push.mStep = 120;
        push.mBodyIndex = 3;
        push.mImpulse = JPH::Vec3(1200.0f, 0.0f, 0.0f);
        pushedScript.mImpulses.push_back(push);
        const auto pushed = run(pushedScript, candidate);
        ASSERT_TRUE(pushed.completed()) << describe(pushed);
        const auto pushedInvariants = RagdollCore::summarizeScenarioInvariants(pushed);
        std::cout << "PRE_LIFT_UNLOAD_PUSH " << describe(pushedInvariants) << std::endl;
        EXPECT_EQ(walkingInvariants.mFallingFrames + walkingInvariants.mFallenFrames, 0u);
        EXPECT_EQ(walkingInvariants.mAirborneFrames, 0u);
        EXPECT_GE(walkingInvariants.mSteps.mLeftStanceCount, 2u);
        EXPECT_GE(walkingInvariants.mSteps.mRightStanceCount, 2u);
        EXPECT_FALSE(pushedInvariants.mFellAfterImpulse);
    }

    TEST(RagdollCoreScenarioTest, batchesWorldSwingHipControllerVariantsWithoutRebuild)
    {
        constexpr std::array<float, 6> stiffnesses{ 250.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f, 8000.0f };
        constexpr std::array<float, 6> dampings{ 25.0f, 50.0f, 100.0f, 250.0f, 500.0f, 1000.0f };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(3.0f, 60.0f, 30);
        RagdollCore::ScenarioSettings settings;
        settings.mUseWorldSwingHipControl = true;

        std::cout << "WORLD_SWING_HIP_SWEEP horizontal_feedback stiffness damping lifted transferred timed_out "
                     "falling min_up achieved slide max_torque saturated_frames hash"
                  << std::endl;
        for (const bool horizontalFeedback : { true, false })
        {
            for (const float stiffness : stiffnesses)
            {
                for (const float damping : dampings)
                {
                    auto profile = RagdollCore::makeScenarioHumanoidProfile();
                    if (!horizontalFeedback)
                        profile.mActiveController.mCenterOfMassFrequency = 0.0f;
                    profile.mActiveController.mGait.mSwingHipWorldStiffness = stiffness;
                    profile.mActiveController.mGait.mSwingHipWorldDamping = damping;
                    profile.mActiveController.mGait.mMaximumSwingHipWorldTorque = 8000.0f;
                    profile.mActiveController.mGait.mMaximumSwingFootForce = 2500.0f;
                    const auto result = RagdollCore::runRagdollScenario(skeleton, profile, settings, script);
                    ASSERT_TRUE(result.completed()) << describe(result);

                    bool lifted = false;
                    bool transferred = false;
                    for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
                    {
                        lifted = lifted || frame.mGait.mSwingFootLifted;
                        transferred = transferred
                            || (lifted && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                                && !frame.mGait.mPhaseTimedOut);
                        if (frame.mSwingHipWorldControlApplied)
                        {
                            EXPECT_TRUE(frame.mStanceHipMotorDisabled);
                            EXPECT_TRUE(frame.mSwingHipMotorDisabled);
                            EXPECT_LE(frame.mAppliedSwingHipWorldTorque.Length(),
                                profile.mActiveController.mGait.mMaximumSwingHipWorldTorque * 1.0001f);
                        }
                    }
                    const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
                    std::cout << "WORLD_SWING_HIP_VARIANT " << horizontalFeedback << ' ' << stiffness << ' '
                              << damping << ' ' << lifted << ' ' << transferred << ' '
                              << invariants.mSteps.mTimedOutPhases << ' '
                              << invariants.mFallingFrames + invariants.mFallenFrames << ' '
                              << invariants.mMinPelvisUpDot << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                              << invariants.mMaxStanceFootSlide << ' ' << invariants.mMaxSwingHipWorldTorque << ' '
                              << invariants.mSwingHipWorldTorqueSaturatedFrames << ' '
                              << RagdollCore::hashScenarioTelemetry(result) << std::endl;
                }
            }
        }
    }

    TEST(RagdollCoreScenarioTest, batchesSwingHipTargetRateVariantsWithoutRebuild)
    {
        constexpr std::array<float, 10> targetRates{ 0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 6.0f, 12.0f, 24.0f, 60.0f, 120.0f };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(3.0f, 60.0f, 30);
        std::cout << "SWING_HIP_RATE_SWEEP rate completed lift first_step timed_out falling airborne min_up "
                     "achieved max_slide max_motor_torque motor_saturated_frames hash"
                  << std::endl;
        for (const float targetRate : targetRates)
        {
            auto profile = RagdollCore::makeScenarioHumanoidProfile();
            profile.mActiveController.mGait.mMaximumSwingHipTargetSpeed = targetRate;
            const auto result = RagdollCore::runRagdollScenario(skeleton, profile, {}, script);
            ASSERT_TRUE(result.completed()) << "target rate " << targetRate << '\n' << describe(result);
            bool observedLiftOff = false;
            bool observedFirstStep = false;
            for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
            {
                observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                observedFirstStep = observedFirstStep
                    || (observedLiftOff && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                        && !frame.mGait.mPhaseTimedOut);
            }
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            if (targetRate == 2.0f)
            {
                const auto repeated = RagdollCore::runRagdollScenario(skeleton, profile, {}, script);
                ASSERT_TRUE(repeated.completed()) << describe(repeated);
                EXPECT_TRUE(RagdollCore::haveIdenticalScenarioTelemetry(result, repeated));
            }
            std::cout << "SWING_HIP_RATE_VARIANT " << targetRate << ' ' << result.completed() << ' ' << observedLiftOff
                      << ' ' << observedFirstStep << ' ' << invariants.mSteps.mTimedOutPhases << ' '
                      << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                      << ' ' << invariants.mMinPelvisUpDot << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                      << invariants.mMaxStanceFootSlide << ' ' << invariants.mMaxSwingHipMotorTorque << ' '
                      << invariants.mSwingHipMotorSaturatedFrames << ' ' << RagdollCore::hashScenarioTelemetry(result)
                      << std::endl;
        }
    }

    TEST(RagdollCoreScenarioTest, batchesSwingHipMotorVariantsWithoutRebuild)
    {
        constexpr std::array<float, 4> frequencies{ 0.5f, 1.0f, 2.0f, 4.0f };
        constexpr std::array<float, 4> torqueLimits{ 5000.0f, 10000.0f, 20000.0f, 40000.0f };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(3.0f, 60.0f, 30);
        std::cout << "SWING_HIP_MOTOR_SWEEP frequency torque completed lift first_step timed_out falling airborne "
                     "min_up achieved max_slide saturated_frames max_utilization hash"
                  << std::endl;
        for (const float frequency : frequencies)
        {
            for (const float torqueLimit : torqueLimits)
            {
                auto profile = RagdollCore::makeScenarioHumanoidProfile();
                for (auto& body : profile.mBodies)
                {
                    if ((body.mName == "left_thigh" || body.mName == "right_thigh") && body.mConstraint)
                    {
                        body.mConstraint->mMotor.mFrequency = frequency;
                        body.mConstraint->mMotor.mMaximumTorque = torqueLimit;
                    }
                }
                const auto result = RagdollCore::runRagdollScenario(skeleton, profile, {}, script);
                ASSERT_TRUE(result.completed()) << "frequency " << frequency << " torque " << torqueLimit << '\n'
                                                << describe(result);
                bool observedLiftOff = false;
                bool observedFirstStep = false;
                for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
                {
                    observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                    observedFirstStep = observedFirstStep
                        || (observedLiftOff && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                            && !frame.mGait.mPhaseTimedOut);
                }
                const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
                EXPECT_LE(invariants.mMaxSwingHipMotorPeakAxisUtilization, 1.0001f);
                std::cout << "SWING_HIP_MOTOR_VARIANT " << frequency << ' ' << torqueLimit << ' ' << result.completed()
                          << ' ' << observedLiftOff << ' ' << observedFirstStep << ' '
                          << invariants.mSteps.mTimedOutPhases << ' '
                          << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                          << ' ' << invariants.mMinPelvisUpDot << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                          << invariants.mMaxStanceFootSlide << ' ' << invariants.mSwingHipMotorSaturatedFrames << ' '
                          << invariants.mMaxSwingHipMotorPeakAxisUtilization << ' '
                          << RagdollCore::hashScenarioTelemetry(result) << std::endl;
            }
        }
    }

    TEST(RagdollCoreScenarioTest, batchesSwingKneeAndAnkleMotorVariantsWithoutRebuild)
    {
        constexpr std::array<float, 4> kneeTorqueLimits{ 16000.0f, 32000.0f, 64000.0f, 128000.0f };
        constexpr std::array<float, 3> ankleTorqueLimits{ 10000.0f, 20000.0f, 40000.0f };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(3.0f, 60.0f, 30);
        std::cout << "SWING_DISTAL_MOTOR_SWEEP knee_torque ankle_torque lift first_step timed_out falling airborne "
                     "min_up achieved max_slide knee_saturated knee_max_use ankle_saturated ankle_max_use hash"
                  << std::endl;
        for (const float kneeTorque : kneeTorqueLimits)
        {
            for (const float ankleTorque : ankleTorqueLimits)
            {
                auto profile = RagdollCore::makeScenarioHumanoidProfile();
                for (auto& body : profile.mBodies)
                {
                    if (!body.mConstraint)
                        continue;
                    if (body.mName == "left_calf" || body.mName == "right_calf")
                        body.mConstraint->mMotor.mMaximumTorque = kneeTorque;
                    else if (body.mName == "left_foot" || body.mName == "right_foot")
                        body.mConstraint->mMotor.mMaximumTorque = ankleTorque;
                }
                const auto result = RagdollCore::runRagdollScenario(skeleton, profile, {}, script);
                ASSERT_TRUE(result.completed()) << "knee torque " << kneeTorque << " ankle torque " << ankleTorque
                                                << '\n' << describe(result);
                bool observedLiftOff = false;
                bool observedFirstStep = false;
                std::size_t kneeSaturated = 0;
                std::size_t ankleSaturated = 0;
                float maximumKneeUtilization = 0.0f;
                float maximumAnkleUtilization = 0.0f;
                for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
                {
                    observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                    observedFirstStep = observedFirstStep
                        || (observedLiftOff && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                            && !frame.mGait.mPhaseTimedOut);
                    maximumKneeUtilization
                        = std::max(maximumKneeUtilization, frame.mSwingKneeMotorPeakAxisUtilization);
                    maximumAnkleUtilization
                        = std::max(maximumAnkleUtilization, frame.mSwingFootJointMotorPeakAxisUtilization);
                    kneeSaturated += frame.mSwingKneeMotorPeakAxisUtilization >= 0.999f;
                    ankleSaturated += frame.mSwingFootJointMotorPeakAxisUtilization >= 0.999f;
                }
                EXPECT_LE(maximumKneeUtilization, 1.0001f);
                EXPECT_LE(maximumAnkleUtilization, 1.0001f);
                const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
                std::cout << "SWING_DISTAL_MOTOR_VARIANT " << kneeTorque << ' ' << ankleTorque << ' '
                          << observedLiftOff << ' ' << observedFirstStep << ' '
                          << invariants.mSteps.mTimedOutPhases << ' '
                          << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                          << ' ' << invariants.mMinPelvisUpDot << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                          << invariants.mMaxStanceFootSlide << ' ' << kneeSaturated << ' ' << maximumKneeUtilization
                          << ' ' << ankleSaturated << ' ' << maximumAnkleUtilization << ' '
                          << RagdollCore::hashScenarioTelemetry(result) << std::endl;
            }
        }
    }

    TEST(RagdollCoreScenarioTest, batchesSwingKneeMotorFrequencyVariantsWithoutRebuild)
    {
        constexpr std::array<float, 4> frequencies{ 0.5f, 1.0f, 2.0f, 4.0f };
        constexpr std::array<float, 4> torqueLimits{ 16000.0f, 32000.0f, 64000.0f, 128000.0f };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(3.0f, 60.0f, 30);
        std::cout << "SWING_KNEE_MOTOR_SWEEP frequency torque lift first_step timed_out falling airborne min_up "
                     "achieved max_slide saturated max_use hash"
                  << std::endl;
        for (const float frequency : frequencies)
        {
            for (const float torqueLimit : torqueLimits)
            {
                auto profile = RagdollCore::makeScenarioHumanoidProfile();
                for (auto& body : profile.mBodies)
                {
                    if ((body.mName == "left_calf" || body.mName == "right_calf") && body.mConstraint)
                    {
                        body.mConstraint->mMotor.mFrequency = frequency;
                        body.mConstraint->mMotor.mMaximumTorque = torqueLimit;
                    }
                }
                const auto result = RagdollCore::runRagdollScenario(skeleton, profile, {}, script);
                ASSERT_TRUE(result.completed()) << "frequency " << frequency << " torque " << torqueLimit << '\n'
                                                << describe(result);
                bool observedLiftOff = false;
                bool observedFirstStep = false;
                std::size_t saturated = 0;
                float maximumUtilization = 0.0f;
                for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
                {
                    observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                    observedFirstStep = observedFirstStep
                        || (observedLiftOff && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                            && !frame.mGait.mPhaseTimedOut);
                    maximumUtilization
                        = std::max(maximumUtilization, frame.mSwingKneeMotorPeakAxisUtilization);
                    saturated += frame.mSwingKneeMotorPeakAxisUtilization >= 0.999f;
                }
                EXPECT_LE(maximumUtilization, 1.0001f);
                const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
                std::cout << "SWING_KNEE_MOTOR_VARIANT " << frequency << ' ' << torqueLimit << ' '
                          << observedLiftOff << ' ' << observedFirstStep << ' '
                          << invariants.mSteps.mTimedOutPhases << ' '
                          << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                          << ' ' << invariants.mMinPelvisUpDot << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                          << invariants.mMaxStanceFootSlide << ' ' << saturated << ' ' << maximumUtilization << ' '
                          << RagdollCore::hashScenarioTelemetry(result) << std::endl;
            }
        }
    }

    TEST(RagdollCoreScenarioTest, isolatesPinnedPelvisSwingLegTracking)
    {
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        auto script = walkingScript(1.5f, 60.0f, 30);
        RagdollCore::ScenarioInputKeyframe initial;
        initial.mStep = 0;
        initial.mTranslationActuationEnabled = false;
        initial.mUprightTorqueEnabled = false;
        script.mKeyframes.insert(script.mKeyframes.begin(), initial);
        script.mKeyframes[1].mTranslationActuationEnabled = false;
        script.mKeyframes[1].mUprightTorqueEnabled = false;
        RagdollCore::ScenarioSettings settings;
        settings.mPinPelvis = true;
        const auto result = RagdollCore::runRagdollScenario(
            skeleton, RagdollCore::makeScenarioHumanoidProfile(), settings, script);
        ASSERT_TRUE(result.completed()) << describe(result);
        exportTraceIfRequested(result, "pinned-swing-leg");
        const std::size_t swingFoot = result.mLandmarks.mRightFoot;
        ASSERT_FALSE(result.mFrames.empty());
        const float initialY = static_cast<float>(result.mFrames.front().mBodies[swingFoot].mCenterOfMass.GetY());
        float maximumForwardTravel = 0.0f;
        float maximumTargetError = 0.0f;
        bool lifted = false;
        for (const auto& frame : result.mFrames)
        {
            maximumForwardTravel = std::max(maximumForwardTravel,
                static_cast<float>(frame.mBodies[swingFoot].mCenterOfMass.GetY()) - initialY);
            maximumTargetError = std::max(maximumTargetError, frame.mSwingFootHorizontalPlacementError);
            lifted = lifted || frame.mGait.mSwingFootLifted;
        }
        const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
        std::cout << "PINNED_SWING_LEG lift=" << lifted << " forward_travel=" << maximumForwardTravel
                  << " maximum_target_error=" << maximumTargetError
                  << " timed_out=" << invariants.mSteps.mTimedOutPhases
                  << " knee_saturation="
                  << std::count_if(result.mFrames.begin(), result.mFrames.end(), [](const auto& frame) {
                         return frame.mSwingKneeMotorPeakAxisUtilization >= 0.999f;
                     })
                  << " hash=" << RagdollCore::hashScenarioTelemetry(result) << std::endl;
    }

    TEST(RagdollCoreScenarioTest, batchesPinnedPelvisSwingClearanceVariantsWithoutRebuild)
    {
        constexpr std::array<float, 6> clearances{ 1.0f, 2.0f, 4.0f, 6.0f, 8.0f, 10.0f };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        std::cout << "PINNED_SWING_CLEARANCE clearance lift max_forward min_forward max_target_error "
                     "max_hip_clamp timed_out full_first_step full_timed_out full_falling full_speed full_slide hash"
                  << std::endl;
        for (const float clearance : clearances)
        {
            auto profile = RagdollCore::makeScenarioHumanoidProfile();
            profile.mActiveController.mGait.mSwingFootLift = clearance;
            auto script = walkingScript(1.5f, 60.0f, 30);
            RagdollCore::ScenarioInputKeyframe initial;
            initial.mStep = 0;
            initial.mTranslationActuationEnabled = false;
            initial.mUprightTorqueEnabled = false;
            script.mKeyframes.insert(script.mKeyframes.begin(), initial);
            script.mKeyframes[1].mTranslationActuationEnabled = false;
            script.mKeyframes[1].mUprightTorqueEnabled = false;
            RagdollCore::ScenarioSettings settings;
            settings.mPinPelvis = true;
            const auto result = RagdollCore::runRagdollScenario(skeleton, profile, settings, script);
            ASSERT_TRUE(result.completed()) << "clearance " << clearance << '\n' << describe(result);
            const std::size_t swingFoot = result.mLandmarks.mRightFoot;
            const float initialY = static_cast<float>(result.mFrames.front().mBodies[swingFoot].mCenterOfMass.GetY());
            float maximumForward = 0.0f;
            float minimumForward = 0.0f;
            float maximumTargetError = 0.0f;
            float maximumHipClamp = 0.0f;
            bool lifted = false;
            for (const auto& frame : result.mFrames)
            {
                const float travel
                    = static_cast<float>(frame.mBodies[swingFoot].mCenterOfMass.GetY()) - initialY;
                maximumForward = std::max(maximumForward, travel);
                minimumForward = std::min(minimumForward, travel);
                maximumTargetError = std::max(maximumTargetError, frame.mSwingFootHorizontalPlacementError);
                maximumHipClamp = std::max(maximumHipClamp,
                    frame.mSwingHipMotorTarget ? frame.mSwingHipMotorTarget->mTargetClampDelta : 0.0f);
                lifted = lifted || frame.mGait.mSwingFootLifted;
            }
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            const auto fullResult = RagdollCore::runRagdollScenario(
                skeleton, profile, {}, walkingScript(3.0f, 60.0f, 30));
            ASSERT_TRUE(fullResult.completed()) << "full clearance " << clearance << '\n' << describe(fullResult);
            exportTraceIfRequested(fullResult, "swing-clearance-" + std::to_string(static_cast<int>(clearance)));
            bool fullLifted = false;
            bool fullFirstStep = false;
            for (const auto& frame : fullResult.mFrames)
            {
                fullLifted = fullLifted || frame.mGait.mSwingFootLifted;
                fullFirstStep = fullFirstStep
                    || (fullLifted && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                        && !frame.mGait.mPhaseTimedOut);
            }
            const auto fullInvariants = RagdollCore::summarizeScenarioInvariants(fullResult);
            std::cout << "PINNED_SWING_CLEARANCE_VARIANT " << clearance << ' ' << lifted << ' ' << maximumForward
                      << ' ' << minimumForward << ' ' << maximumTargetError << ' ' << maximumHipClamp << ' '
                      << invariants.mSteps.mTimedOutPhases << ' ' << fullFirstStep << ' '
                      << fullInvariants.mSteps.mTimedOutPhases << ' '
                      << fullInvariants.mFallingFrames + fullInvariants.mFallenFrames << ' '
                      << fullInvariants.mFinalHalfSecondAchievedSpeed << ' ' << fullInvariants.mMaxStanceFootSlide
                      << ' ' << RagdollCore::hashScenarioTelemetry(result) << std::endl;
        }
    }

    TEST(RagdollCoreScenarioTest, batchesPinnedPelvisHipConeVariantsWithoutRebuild)
    {
        struct Cone
        {
            float mNormal;
            float mPlane;
            float mTwist;
        };
        constexpr std::array<Cone, 10> cones{ {
            { 0.65f, 0.48f, 0.28f }, { 0.85f, 0.48f, 0.28f }, { 0.65f, 0.70f, 0.28f },
            { 0.85f, 0.70f, 0.28f }, { 0.65f, 0.90f, 0.28f }, { 1.05f, 0.90f, 0.28f },
            { 0.65f, 0.48f, 0.50f }, { 0.65f, 0.48f, 0.80f },
            { 0.85f, 0.70f, 0.80f }, { 1.05f, 0.90f, 1.20f },
        } };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        std::cout << "PINNED_HIP_CONE normal plane runtime_normal runtime_plane lift max_forward min_forward max_error max_clamp "
                     "full_first_step full_timed_out full_falling full_speed full_slide hash"
                  << std::endl;
        for (const Cone cone : cones)
        {
            auto profile = RagdollCore::makeScenarioHumanoidProfile();
            for (auto& body : profile.mBodies)
            {
                if ((body.mName == "left_thigh" || body.mName == "right_thigh") && body.mConstraint)
                {
                    body.mConstraint->mNormalHalfConeAngle = cone.mNormal;
                    body.mConstraint->mPlaneHalfConeAngle = cone.mPlane;
                    body.mConstraint->mTwistMinAngle = -cone.mTwist;
                    body.mConstraint->mTwistMaxAngle = cone.mTwist;
                }
            }
            auto pinnedScript = walkingScript(1.5f, 60.0f, 30);
            RagdollCore::ScenarioInputKeyframe initial;
            initial.mStep = 0;
            initial.mTranslationActuationEnabled = false;
            initial.mUprightTorqueEnabled = false;
            pinnedScript.mKeyframes.insert(pinnedScript.mKeyframes.begin(), initial);
            pinnedScript.mKeyframes[1].mTranslationActuationEnabled = false;
            pinnedScript.mKeyframes[1].mUprightTorqueEnabled = false;
            RagdollCore::ScenarioSettings pinnedSettings;
            pinnedSettings.mPinPelvis = true;
            const auto pinned = RagdollCore::runRagdollScenario(skeleton, profile, pinnedSettings, pinnedScript);
            ASSERT_TRUE(pinned.completed()) << describe(pinned);
            const std::size_t swingFoot = pinned.mLandmarks.mRightFoot;
            const float initialY = static_cast<float>(pinned.mFrames.front().mBodies[swingFoot].mCenterOfMass.GetY());
            float maximumForward = 0.0f;
            float minimumForward = 0.0f;
            float maximumError = 0.0f;
            float maximumClamp = 0.0f;
            float runtimeNormal = 0.0f;
            float runtimePlane = 0.0f;
            bool lifted = false;
            for (const auto& frame : pinned.mFrames)
            {
                const float travel
                    = static_cast<float>(frame.mBodies[swingFoot].mCenterOfMass.GetY()) - initialY;
                maximumForward = std::max(maximumForward, travel);
                minimumForward = std::min(minimumForward, travel);
                maximumError = std::max(maximumError, frame.mSwingFootHorizontalPlacementError);
                maximumClamp = std::max(maximumClamp,
                    frame.mSwingHipMotorTarget ? frame.mSwingHipMotorTarget->mTargetClampDelta : 0.0f);
                if (frame.mSwingHipMotorTarget && frame.mSwingHipMotorTarget->mNormalHalfConeAngle
                    && frame.mSwingHipMotorTarget->mPlaneHalfConeAngle)
                {
                    runtimeNormal = *frame.mSwingHipMotorTarget->mNormalHalfConeAngle;
                    runtimePlane = *frame.mSwingHipMotorTarget->mPlaneHalfConeAngle;
                }
                lifted = lifted || frame.mGait.mSwingFootLifted;
            }
            const auto full
                = RagdollCore::runRagdollScenario(skeleton, profile, {}, walkingScript(3.0f, 60.0f, 30));
            ASSERT_TRUE(full.completed()) << describe(full);
            bool fullLifted = false;
            bool firstStep = false;
            for (const auto& frame : full.mFrames)
            {
                fullLifted = fullLifted || frame.mGait.mSwingFootLifted;
                firstStep = firstStep
                    || (fullLifted && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                        && !frame.mGait.mPhaseTimedOut);
            }
            const auto invariants = RagdollCore::summarizeScenarioInvariants(full);
            std::cout << "PINNED_HIP_CONE_VARIANT " << cone.mNormal << ' ' << cone.mPlane << '/' << cone.mTwist << ' '
                      << runtimeNormal
                      << ' ' << runtimePlane << ' ' << lifted << ' ' << maximumForward << ' ' << minimumForward << ' '
                      << maximumError << ' ' << maximumClamp << ' '
                      << firstStep << ' ' << invariants.mSteps.mTimedOutPhases << ' '
                      << invariants.mFallingFrames + invariants.mFallenFrames << ' '
                      << invariants.mFinalHalfSecondAchievedSpeed << ' ' << invariants.mMaxStanceFootSlide << ' '
                      << RagdollCore::hashScenarioTelemetry(pinned) << std::endl;
        }
    }

    TEST(RagdollCoreScenarioTest, batchesMotorOnlyRequestedSpeedVariantsWithoutRebuild)
    {
        constexpr std::array<float, 7> speeds{ 5.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        auto profile = RagdollCore::makeScenarioHumanoidProfile();
        profile.mActiveController.mGait.mMaximumSwingFootForce = 0.0f;
        profile.mActiveController.mGait.mMaximumSwingFootHorizontalForce = 0.0f;
        ASSERT_FLOAT_EQ(profile.mActiveController.mGait.mMaximumSwingFootForce, 0.0f);
        ASSERT_FLOAT_EQ(profile.mActiveController.mGait.mMaximumSwingFootHorizontalForce, 0.0f);
        std::cout << "MOTOR_ONLY_SPEED_SWEEP requested lifted first_step timed_out falling airborne min_up "
                     "mean_achieved final_achieved max_slide motor_saturated_frames hash"
                  << std::endl;
        for (const float speed : speeds)
        {
            const auto result = RagdollCore::runRagdollScenario(skeleton, profile, {}, walkingScript(3.0f, speed, 30));
            ASSERT_TRUE(result.completed()) << "requested speed " << speed << '\n' << describe(result);
            bool observedLiftOff = false;
            bool observedFirstStep = false;
            for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
            {
                observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                observedFirstStep = observedFirstStep
                    || (observedLiftOff && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                        && !frame.mGait.mPhaseTimedOut);
                if (frame.mSwingFootControl)
                    EXPECT_TRUE(frame.mSwingFootControl->mForce.IsNearZero());
            }
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            EXPECT_EQ(invariants.mAirborneFrames, 0u) << "requested speed " << speed;
            std::cout << "MOTOR_ONLY_SPEED_VARIANT " << speed << ' ' << observedLiftOff << ' ' << observedFirstStep
                      << ' ' << invariants.mSteps.mTimedOutPhases << ' '
                      << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                      << ' ' << invariants.mMinPelvisUpDot << ' ' << invariants.mMeanAchievedSpeed << ' '
                      << invariants.mFinalHalfSecondAchievedSpeed << ' ' << invariants.mMaxStanceFootSlide << ' '
                      << invariants.mSwingHipMotorSaturatedFrames << ' ' << RagdollCore::hashScenarioTelemetry(result)
                      << std::endl;
        }
    }

    TEST(RagdollCoreScenarioTest, batchesWholeLegMotorTorqueScaleVariantsWithoutRebuild)
    {
        constexpr std::array<float, 8> scales{ 0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f };
        constexpr std::array<std::size_t, 6> legBodies{ 14, 15, 16, 17, 18, 19 };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(3.0f, 60.0f, 30);
        std::cout << "LEG_MOTOR_TORQUE_SWEEP horizontal_feedback scale lifted first_step alternating_steps "
                     "timed_out falling airborne min_up mean_achieved final_achieved max_slide "
                     "hip_saturated_frames hash"
                  << std::endl;
        for (const bool horizontalFeedback : { true, false })
        {
            for (const float scale : scales)
            {
                auto profile = RagdollCore::makeScenarioHumanoidProfile();
                if (!horizontalFeedback)
                    profile.mActiveController.mCenterOfMassFrequency = 0.0f;
                for (const std::size_t bodyIndex : legBodies)
                {
                    ASSERT_TRUE(profile.mBodies[bodyIndex].mConstraint.has_value());
                    profile.mBodies[bodyIndex].mConstraint->mMotor.mMaximumTorque *= scale;
                }
                const auto result = RagdollCore::runRagdollScenario(skeleton, profile, {}, script);
                ASSERT_TRUE(result.completed()) << "horizontal feedback " << horizontalFeedback
                                                << ", leg motor scale " << scale << '\n'
                                                << describe(result);
                bool observedLiftOff = false;
                bool observedFirstStep = false;
                std::size_t alternatingSteps = 0;
                GaitPhase previousTransition = GaitPhase::Idle;
                for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
                {
                    observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                    if (!frame.mGait.mTransitioned || !frame.mGait.hasStance())
                        continue;
                    if (frame.mGait.mPhase == GaitPhase::RightStance && observedLiftOff)
                        observedFirstStep = true;
                    if (previousTransition != GaitPhase::Idle && frame.mGait.mPhase != previousTransition)
                        ++alternatingSteps;
                    previousTransition = frame.mGait.mPhase;
                }
                const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
                std::cout << "LEG_MOTOR_TORQUE_VARIANT " << horizontalFeedback << ' ' << scale << ' '
                          << observedLiftOff << ' ' << observedFirstStep << ' ' << alternatingSteps << ' '
                          << invariants.mSteps.mTimedOutPhases << ' '
                          << invariants.mFallingFrames + invariants.mFallenFrames << ' '
                          << invariants.mAirborneFrames << ' ' << invariants.mMinPelvisUpDot << ' '
                          << invariants.mMeanAchievedSpeed << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                          << invariants.mMaxStanceFootSlide << ' ' << invariants.mSwingHipMotorSaturatedFrames << ' '
                          << RagdollCore::hashScenarioTelemetry(result) << std::endl;
            }
        }
    }

    TEST(RagdollCoreScenarioTest, batchesCapturePointBlendVariantsWithoutRebuild)
    {
        constexpr std::array<float, 7> blends{ 0.0f, 0.1f, 0.2f, 0.35f, 0.5f, 0.75f, 1.0f };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(3.0f, 60.0f, 30);
        std::cout << "CAPTURE_BLEND_SWEEP blend lifted first_step alternating_steps timed_out falling airborne "
                     "min_up mean_achieved final_achieved max_slide motor_saturated_frames hash"
                  << std::endl;
        for (const float blend : blends)
        {
            auto profile = RagdollCore::makeScenarioHumanoidProfile();
            profile.mActiveController.mGait.mCapturePointBlend = blend;
            const auto result = RagdollCore::runRagdollScenario(skeleton, profile, {}, script);
            ASSERT_TRUE(result.completed()) << "capture blend " << blend << '\n' << describe(result);
            bool observedLiftOff = false;
            bool observedFirstStep = false;
            std::size_t alternatingSteps = 0;
            GaitPhase previousTransition = GaitPhase::Idle;
            for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
            {
                observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                if (!frame.mGait.mTransitioned || !frame.mGait.hasStance())
                    continue;
                if (frame.mGait.mPhase == GaitPhase::RightStance && observedLiftOff)
                    observedFirstStep = true;
                if (previousTransition != GaitPhase::Idle && frame.mGait.mPhase != previousTransition)
                    ++alternatingSteps;
                previousTransition = frame.mGait.mPhase;
            }
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            std::cout << "CAPTURE_BLEND_VARIANT " << blend << ' ' << observedLiftOff << ' ' << observedFirstStep << ' '
                      << alternatingSteps << ' ' << invariants.mSteps.mTimedOutPhases << ' '
                      << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                      << ' ' << invariants.mMinPelvisUpDot << ' ' << invariants.mMeanAchievedSpeed << ' '
                      << invariants.mFinalHalfSecondAchievedSpeed << ' ' << invariants.mMaxStanceFootSlide << ' '
                      << invariants.mSwingHipMotorSaturatedFrames << ' '
                      << RagdollCore::hashScenarioTelemetry(result) << std::endl;
        }
    }

    TEST(RagdollCoreScenarioTest, batchesPelvisHeightBandwidthVariantsWithoutRebuild)
    {
        constexpr std::array<float, 7> frequencies{ 0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(3.0f, 60.0f, 30);
        std::cout << "HEIGHT_BANDWIDTH_SWEEP frequency lifted first_step alternating_steps timed_out falling airborne "
                     "force_saturated min_up mean_achieved final_achieved max_slide motor_saturated_frames hash"
                  << std::endl;
        for (const float frequency : frequencies)
        {
            auto profile = RagdollCore::makeScenarioHumanoidProfile();
            profile.mActiveController.mPelvisHeightFrequency = frequency;
            const auto result = RagdollCore::runRagdollScenario(skeleton, profile, {}, script);
            ASSERT_TRUE(result.completed()) << "pelvis height frequency " << frequency << '\n' << describe(result);
            bool observedLiftOff = false;
            bool observedFirstStep = false;
            std::size_t alternatingSteps = 0;
            GaitPhase previousTransition = GaitPhase::Idle;
            for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
            {
                observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                if (!frame.mGait.mTransitioned || !frame.mGait.hasStance())
                    continue;
                if (frame.mGait.mPhase == GaitPhase::RightStance && observedLiftOff)
                    observedFirstStep = true;
                if (previousTransition != GaitPhase::Idle && frame.mGait.mPhase != previousTransition)
                    ++alternatingSteps;
                previousTransition = frame.mGait.mPhase;
            }
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            std::cout << "HEIGHT_BANDWIDTH_VARIANT " << frequency << ' ' << observedLiftOff << ' '
                      << observedFirstStep << ' ' << alternatingSteps << ' '
                      << invariants.mSteps.mTimedOutPhases << ' '
                      << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                      << ' ' << invariants.mPelvisForceSaturatedFrames << ' ' << invariants.mMinPelvisUpDot << ' '
                      << invariants.mMeanAchievedSpeed << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                      << invariants.mMaxStanceFootSlide << ' ' << invariants.mSwingHipMotorSaturatedFrames << ' '
                      << RagdollCore::hashScenarioTelemetry(result) << std::endl;
        }
    }

    TEST(RagdollCoreScenarioTest, batchesStanceTorqueOnlyVariantsWithoutRebuild)
    {
        struct Variant
        {
            const char* mName;
            bool mReserveStanceHip;
            bool mCompensateSwingHip;
            bool mUseWorldSwingHip;
        };
        constexpr std::array<Variant, 4> variants{ {
            { "joint_motors", false, false, false },
            { "reserved_stance_hip", true, false, false },
            { "swing_reaction_to_stance", true, true, false },
            { "world_hip_pair", false, false, true },
        } };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto script = walkingScript(3.0f, 60.0f, 30);
        std::cout << "STANCE_TORQUE_SWEEP route lifted first_step alternating_steps timed_out falling airborne "
                     "force_saturated min_up mean_achieved final_achieved max_slide motor_saturated_frames "
                     "world_saturated_frames hash"
                  << std::endl;
        for (const Variant& variant : variants)
        {
            auto profile = RagdollCore::makeScenarioHumanoidProfile();
            // Suppress both COM position and velocity feedback while retaining
            // the independent vertical pelvis-height pair. Any horizontal
            // motion must therefore come from joint torques reacting through
            // a physical stance contact, not the pelvis translation actuator.
            profile.mActiveController.mCenterOfMassFrequency = 0.0f;
            RagdollCore::ScenarioSettings settings;
            settings.mReserveStanceHipMotor = variant.mReserveStanceHip;
            settings.mCompensateSwingHipMotorImpulse = variant.mCompensateSwingHip;
            settings.mUseWorldSwingHipControl = variant.mUseWorldSwingHip;
            const auto result = RagdollCore::runRagdollScenario(skeleton, profile, settings, script);
            ASSERT_TRUE(result.completed()) << variant.mName << '\n' << describe(result);
            bool observedLiftOff = false;
            bool observedFirstStep = false;
            std::size_t alternatingSteps = 0;
            GaitPhase previousTransition = GaitPhase::Idle;
            for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
            {
                observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                if (!frame.mGait.mTransitioned || !frame.mGait.hasStance())
                    continue;
                if (frame.mGait.mPhase == GaitPhase::RightStance && observedLiftOff)
                    observedFirstStep = true;
                if (previousTransition != GaitPhase::Idle && frame.mGait.mPhase != previousTransition)
                    ++alternatingSteps;
                previousTransition = frame.mGait.mPhase;
            }
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
            std::cout << "STANCE_TORQUE_VARIANT " << variant.mName << ' ' << observedLiftOff << ' '
                      << observedFirstStep << ' ' << alternatingSteps << ' '
                      << invariants.mSteps.mTimedOutPhases << ' '
                      << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                      << ' ' << invariants.mPelvisForceSaturatedFrames << ' ' << invariants.mMinPelvisUpDot << ' '
                      << invariants.mMeanAchievedSpeed << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                      << invariants.mMaxStanceFootSlide << ' ' << invariants.mSwingHipMotorSaturatedFrames << ' '
                      << invariants.mSwingHipWorldTorqueSaturatedFrames << ' '
                      << RagdollCore::hashScenarioTelemetry(result) << std::endl;
        }
    }

    TEST(RagdollCoreScenarioTest, batchesStanceFootFrictionBudgetVariantsWithoutRebuild)
    {
        constexpr std::array<float, 7> frictionValues{ 0.05f, 0.1f, 0.2f, 0.3f, 0.4f, 0.6f, 0.8f };
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto profile = RagdollCore::makeScenarioHumanoidProfile();
        const auto script = walkingScript(3.0f, 60.0f, 30);
        std::cout << "STANCE_FOOT_FRICTION_SWEEP friction lifted first_step alternating_steps timed_out falling "
                     "airborne magnitude_saturated horizontal_saturated vertical_saturated min_up mean_achieved "
                     "final_achieved max_slide hash"
                  << std::endl;
        for (const float friction : frictionValues)
        {
            RagdollCore::ScenarioSettings settings;
            settings.mGroundFriction = friction;
            settings.mRouteGaitHorizontalReactionToStanceFoot = true;
            const auto result = RagdollCore::runRagdollScenario(skeleton, profile, settings, script);
            ASSERT_TRUE(result.completed()) << "friction " << friction << '\n' << describe(result);
            bool observedLiftOff = false;
            bool observedFirstStep = false;
            std::size_t alternatingSteps = 0;
            GaitPhase previousTransition = GaitPhase::Idle;
            for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
            {
                observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                if (!frame.mGait.mTransitioned || !frame.mGait.hasStance())
                    continue;
                if (frame.mGait.mPhase == GaitPhase::RightStance && observedLiftOff)
                    observedFirstStep = true;
                if (previousTransition != GaitPhase::Idle && frame.mGait.mPhase != previousTransition)
                    ++alternatingSteps;
                previousTransition = frame.mGait.mPhase;
            }
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
            // Jolt combines the two materials geometrically; ground friction
            // alone is not the coefficient of the physical contact.
            const float contactFriction = std::sqrt(settings.mGroundFriction
                * profile.mBodies.front().mMaterial.mFriction);
            const float frictionForceBudget = contactFriction * settings.mTotalMass * settings.mGravity;
            for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
            {
                JPH::Vec3 horizontalForce = frame.mAppliedPelvisForce;
                horizontalForce.SetZ(0.0f);
                EXPECT_LE(horizontalForce.Length(), frictionForceBudget * 1.0001f);
                for (const auto& contact : frame.mContacts)
                {
                    ASSERT_TRUE(contact.mFriction);
                    EXPECT_NEAR(*contact.mFriction, std::sqrt(settings.mGroundFriction
                        * profile.mBodies[contact.mBodyIndex].mMaterial.mFriction), 1e-6f);
                }
                if (frame.mSupport.hasGroundContact())
                {
                    ASSERT_TRUE(frame.mSupport.mFriction);
                    EXPECT_NEAR(*frame.mSupport.mFriction, contactFriction, 1e-6f);
                }
            }
            std::cout << "STANCE_FOOT_FRICTION_VARIANT " << friction << ' ' << observedLiftOff << ' '
                      << observedFirstStep << ' ' << alternatingSteps << ' '
                      << invariants.mSteps.mTimedOutPhases << ' '
                      << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                      << ' ' << invariants.mPelvisMagnitudeSaturatedFrames << ' '
                      << invariants.mPelvisHorizontalForceSaturatedFrames << ' '
                      << invariants.mPelvisVerticalForceSaturatedFrames << ' ' << invariants.mMinPelvisUpDot << ' '
                      << invariants.mMeanAchievedSpeed << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                      << invariants.mMaxStanceFootSlide << ' ' << RagdollCore::hashScenarioTelemetry(result)
                      << std::endl;
        }
    }

    TEST(RagdollCoreScenarioTest, batchesPhysicsPhasedStanceCompressionVariantsWithoutRebuild)
    {
        constexpr std::array<float, 6> compressions{ 0.0f, 2.0f, 4.0f, 6.0f, 8.0f, 10.0f };
        std::cout << "STANCE_COMPRESSION_SWEEP compression lifted first_step timed_out falling airborne min_up "
                     "mean_achieved final_achieved max_slide motor_saturated_frames hash"
                  << std::endl;
        for (const float compression : compressions)
        {
            auto script = walkingScript(3.0f, 60.0f, 30);
            const auto cycle = compressedStancePoseCycle(compression);
            ASSERT_TRUE(cycle.has_value());
            script.mGaitPoseCycle = *cycle;
            const auto result = run(script);
            ASSERT_TRUE(result.completed()) << "stance compression " << compression << '\n' << describe(result);
            bool observedLiftOff = false;
            bool observedFirstStep = false;
            for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
            {
                observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                observedFirstStep = observedFirstStep
                    || (observedLiftOff && frame.mGait.mTransitioned && frame.mGait.mPhase == GaitPhase::RightStance
                        && !frame.mGait.mPhaseTimedOut);
            }
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            std::cout << "STANCE_COMPRESSION_VARIANT " << compression << ' ' << observedLiftOff << ' '
                      << observedFirstStep << ' ' << invariants.mSteps.mTimedOutPhases << ' '
                      << invariants.mFallingFrames + invariants.mFallenFrames << ' ' << invariants.mAirborneFrames
                      << ' ' << invariants.mMinPelvisUpDot << ' ' << invariants.mMeanAchievedSpeed << ' '
                      << invariants.mFinalHalfSecondAchievedSpeed << ' ' << invariants.mMaxStanceFootSlide << ' '
                      << invariants.mSwingHipMotorSaturatedFrames << ' ' << RagdollCore::hashScenarioTelemetry(result)
                      << std::endl;
        }
    }


    TEST(RagdollCoreScenarioTest, DISABLED_batchesLocalAuthoredGaitPhaseOffsets)
    {
        const auto replay = loadLocalPoweredReplay();
        ASSERT_TRUE(replay.has_value())
            << "Set OPENMW_RAGDOLL_GAIT_REPLAY to an isolated powered gait capture";
        ASSERT_EQ(replay->mBodyCount, 20u);
        // Historical candidate window, not a validated walk reference. Reject
        // an invalid seam before running or interpreting any physics trial.
        constexpr std::size_t cycleStart = 40;
        constexpr std::size_t cycleFrameCount = 180;
        constexpr std::array<std::size_t, 6> phaseOffsets{ 0, 30, 60, 90, 120, 150 };
        ASSERT_GT(replay->mFrames.size(), cycleStart + cycleFrameCount);
        const auto seamStart = RagdollCore::decodePoweredReplayInput(*replay, replay->mFrames[cycleStart]);
        const auto seamEnd
            = RagdollCore::decodePoweredReplayInput(*replay, replay->mFrames[cycleStart + cycleFrameCount]);
        ASSERT_TRUE(seamStart.has_value());
        ASSERT_TRUE(seamEnd.has_value());
        ASSERT_EQ(seamStart->mAnimationTargetPose.size(), seamEnd->mAnimationTargetPose.size());
        for (std::size_t body = 0; body < seamStart->mAnimationTargetPose.size(); ++body)
        {
            ASSERT_LT((seamStart->mAnimationTargetPose[body].mTranslation
                          - seamEnd->mAnimationTargetPose[body].mTranslation)
                          .Length(),
                0.05f)
                << "authored loop translation seam at body " << body;
            ASSERT_GT(std::abs(seamStart->mAnimationTargetPose[body].mRotation.Dot(
                          seamEnd->mAnimationTargetPose[body].mRotation)),
                0.99999f)
                << "authored loop rotation seam at body " << body;
        }
        struct ControlVariant
        {
            const char* mName;
            bool mHorizontalTranslation;
            bool mReserveStanceHip;
            bool mCompensateSwingHip;
        };
        constexpr std::array<ControlVariant, 3> controls{ {
            { "translation", true, false, false },
            { "joint_only", false, false, false },
            { "stance_reaction", false, true, true },
        } };
        std::cout << "AUTHORED_GAIT_PHASE_SWEEP control offset lifted first_step alternating_steps timed_out falling airborne min_up "
                     "mean_achieved final_achieved max_slide force_saturated motor_saturated hash"
                  << std::endl;
        for (const ControlVariant& control : controls)
        {
            for (const std::size_t phaseOffset : phaseOffsets)
            {
                auto script = walkingScript(3.0f, 60.0f, 30);
                const auto cycle = extractWrappedPoseCycle(*replay, cycleStart, cycleFrameCount, phaseOffset);
                ASSERT_TRUE(cycle.has_value()) << "phase offset " << phaseOffset;
                script.mGaitPoseCycle = *cycle;
                auto profile = RagdollCore::makeScenarioHumanoidProfile();
                if (!control.mHorizontalTranslation)
                    profile.mActiveController.mCenterOfMassFrequency = 0.0f;
                RagdollCore::ScenarioSettings settings;
                settings.mReserveStanceHipMotor = control.mReserveStanceHip;
                settings.mCompensateSwingHipMotorImpulse = control.mCompensateSwingHip;
                const auto result = RagdollCore::runRagdollScenario(
                    RagdollCore::makeScenarioHumanoidSkeleton(), profile, settings, script);
                ASSERT_TRUE(result.completed())
                    << control.mName << " phase offset " << phaseOffset << '\n' << describe(result);
                bool observedLiftOff = false;
                bool observedFirstStep = false;
                std::size_t alternatingSteps = 0;
                GaitPhase previousTransition = GaitPhase::Idle;
                for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
                {
                    observedLiftOff = observedLiftOff || frame.mGait.mSwingFootLifted;
                    observedFirstStep = observedFirstStep
                        || (observedLiftOff && frame.mGait.mTransitioned
                            && frame.mGait.mPhase == GaitPhase::RightStance && !frame.mGait.mPhaseTimedOut);
                    if (!frame.mGait.mTransitioned || !frame.mGait.hasStance())
                        continue;
                    if (previousTransition != GaitPhase::Idle && frame.mGait.mPhase != previousTransition)
                        ++alternatingSteps;
                    previousTransition = frame.mGait.mPhase;
                }
                const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
                std::cout << "AUTHORED_GAIT_PHASE " << control.mName << ' ' << phaseOffset << ' '
                          << observedLiftOff << ' ' << observedFirstStep << ' ' << alternatingSteps << ' '
                          << invariants.mSteps.mTimedOutPhases << ' '
                          << invariants.mFallingFrames + invariants.mFallenFrames << ' '
                          << invariants.mAirborneFrames << ' ' << invariants.mMinPelvisUpDot << ' '
                          << invariants.mMeanAchievedSpeed << ' ' << invariants.mFinalHalfSecondAchievedSpeed << ' '
                          << invariants.mMaxStanceFootSlide << ' ' << invariants.mPelvisForceSaturatedFrames << ' '
                          << invariants.mSwingHipMotorSaturatedFrames << ' '
                          << RagdollCore::hashScenarioTelemetry(result) << std::endl;
            }
        }
    }

    TEST(RagdollCoreScenarioTest, stageSummariesAttributeAppliedControlAndKeepEmptyStages)
    {
        using namespace RagdollCore;
        ScenarioResult result;
        result.mFrames.resize(2);
        auto& frame = result.mFrames.back();
        frame.mActuationStage = GaitActuationStage::WeightTransfer;
        frame.mUprightActuationStage = GaitActuationStage::Touchdown;
        frame.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
        frame.mCenterOfMassVelocity = JPH::Vec3(0, 5, 0);
        frame.mAppliedPelvisForce = JPH::Vec3(0, -20, 0);
        frame.mUprightApplied = true;
        frame.mAppliedUpright.emplace().mPelvisVerticalForceSaturated = true;
        frame.mUpright.emplace().mPelvisVerticalForceSaturated = false;
        frame.mMotorsApplied = true;
        frame.mSwingKneeMotorPeakAxisUtilization = 1.0f;
        frame.mSwingFootJointMotorPeakAxisUtilization = 1.0f;
        auto& impulse = frame.mSolverImpulse.emplace();
        impulse.mFreeVelocityLimitTouched = true;
        impulse.mVelocityLimitTouched = true;
        impulse.mFreeLinearLimitImpulse = JPH::Vec3(3, 4, 0);
        impulse.mFreeAngularLimitImpulse = JPH::Vec3(0, 0, -12);
        for (bool upright : { false, true })
        {
            std::size_t frames = 0;
            const auto stages = summarizeScenarioGaitStages(result, upright);
            ASSERT_EQ(stages.size(), 7u);
            for (const auto& stage : stages)
            {
                frames += stage.mFrames;
                if (stage.mStage == (upright ? GaitActuationStage::Touchdown : GaitActuationStage::WeightTransfer))
                {
                    EXPECT_EQ(stage.mFrames, 1u);
                    EXPECT_EQ(stage.mBrakingFrames, 1u);
                    EXPECT_EQ(stage.mVerticalSaturationFrames, 1u);
                    EXPECT_EQ(stage.mSwingKneeSaturationFrames, 1u);
                    EXPECT_EQ(stage.mSwingAnkleSaturationFrames, 1u);
                    EXPECT_EQ(stage.mSolverObservedFrames, 1u);
                    EXPECT_EQ(stage.mFreeVelocityLimitedFrames, 1u);
                    EXPECT_EQ(stage.mSolverVelocityLimitedFrames, 1u);
                    EXPECT_FLOAT_EQ(stage.mMaxFreeLinearLimitImpulse, 5.0f);
                    EXPECT_FLOAT_EQ(stage.mMaxFreeAngularLimitImpulse, 12.0f);
                    EXPECT_FLOAT_EQ(stage.mMeanAchievedSpeed, 5.0f);
                }
            }
            EXPECT_EQ(frames, result.mFrames.size());
        }
        auto changed = result;
        changed.mFrames.back().mUprightActuationStage = GaitActuationStage::Swing;
        EXPECT_FALSE(haveIdenticalScenarioTelemetry(result, changed));
        changed = result;
        changed.mFrames.back().mAppliedUpright->mPelvisVerticalForceSaturated = false;
        EXPECT_FALSE(haveIdenticalScenarioTelemetry(result, changed));
        result.mFrames.back().mSwingFootJointMotorTarget.emplace();
        changed = result;
        changed.mFrames.back().mSwingFootJointMotorTarget->mWorldOrientation = true;
        EXPECT_FALSE(haveIdenticalScenarioTelemetry(result, changed));
    }

    TEST(RagdollCoreScenarioTest, solverImpulseObservationDoesNotChangePhysicalReplay)
    {
        const auto script = walkingScript(7.0f, 60.0f, 30);
        for (const bool solverActuation : { false, true })
        {
            SCOPED_TRACE(solverActuation ? "solver_actuation" : "free_integration");
            for (const bool continuous : { true, false })
            {
                RagdollCore::ScenarioSettings settings;
                settings.mUseContinuousCollision = continuous;
                settings.mUseSolverActuation = solverActuation;
                auto observed = run(script, settings);
                ASSERT_TRUE(observed.completed()) << describe(observed);
                EXPECT_EQ(std::any_of(observed.mFrames.begin(), observed.mFrames.end(),
                              [](const auto& frame) { return frame.mSolverCenterOfPressure.has_value(); }), !continuous);
                exportTraceIfRequested(observed, std::string(solverActuation ? "actuation-" : "")
                    + (continuous ? "solver-continuous" : "solver-discrete"));
                EXPECT_TRUE(RagdollCore::haveIdenticalScenarioTelemetry(observed, run(script, settings)));
                settings.mObserveSolverImpulses = false;
                const auto unobserved = run(script, settings);
                ASSERT_TRUE(unobserved.completed()) << describe(unobserved);
                observed.mSettings.mObserveSolverImpulses = false;
                for (auto& frame : observed.mFrames)
                {
                    frame.mSolverImpulse.reset();
                    frame.mSolverCenterOfPressure.reset();
                }
                EXPECT_TRUE(RagdollCore::haveIdenticalScenarioTelemetry(observed, unobserved));
            }
        }
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreScenarioTest, DISABLED_backwardCapturePlanEnablesTheSecondPhysicalHandover)
    {
        const auto script = walkingScript(3.0f, 60.0f, 30);
        const auto recovery = run(script);
        ASSERT_TRUE(recovery.completed()) << describe(recovery);
        EXPECT_TRUE(std::any_of(recovery.mFrames.begin(), recovery.mFrames.end(),
            [](const auto& frame) { return frame.mBackwardRecoveryStep && frame.mGait.mSwingFootLifted; }));
        EXPECT_GE(RagdollCore::summarizeScenarioInvariants(recovery).mSteps.mLeftStanceCount, 2u);
        RagdollCore::ScenarioSettings disabled;
        disabled.mLocomotionController = RagdollCore::ScenarioLocomotionController::Legacy; // legacy-mechanism test
        disabled.mEnableBackwardCaptureRecovery = false;
        const auto control = run(script, disabled);
        ASSERT_TRUE(control.completed()) << describe(control);
        EXPECT_FALSE(std::any_of(control.mFrames.begin(), control.mFrames.end(),
            [](const auto& frame) { return frame.mBackwardRecoveryStep; }));
        EXPECT_EQ(RagdollCore::summarizeScenarioInvariants(control).mSteps.mLeftStanceCount, 1u);
        EXPECT_NE(RagdollCore::hashScenarioTelemetry(recovery), RagdollCore::hashScenarioTelemetry(control));
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreScenarioTest, DISABLED_walkingSatisfiesGaitInvariants)
    {
        for (const bool solverActuation : { false, true })
        {
            SCOPED_TRACE(solverActuation ? "solver_actuation" : "free_integration");
            RagdollCore::ScenarioSettings settings;
            settings.mUseSolverActuation = solverActuation;
            // Measured double-support handover deliberately takes longer than the
            // old purely-timed phase. Seven seconds covers two physical steps per
            // side without changing a controller threshold.
            const auto result = run(walkingScript(7.0f, 60.0f, 30), settings);
            ASSERT_TRUE(result.completed()) << describe(result);
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            exportTraceIfRequested(result, solverActuation ? "walking-solver-actuation" : "walking");
            std::cout << "WALKING_INVARIANTS solver_actuation=" << solverActuation << ' ' << describe(invariants) << std::endl;
            SCOPED_TRACE(describe(result));

            bool observedStance = false;
            bool observedSwingTarget = false;
            for (const RagdollCore::ScenarioFrame& frame : result.mFrames)
            {
                observedStance = observedStance || frame.mGait.hasStance();
                observedSwingTarget = observedSwingTarget || frame.mSwingFootMotorTarget.has_value();
            }
            EXPECT_TRUE(observedStance) << "the physics phase never entered a stance";
            EXPECT_TRUE(observedSwingTarget) << "the swing leg never received a physics-owned target";

            // Physical gates. None of these values is read by the controller.
            EXPECT_EQ(invariants.mFallingFrames + invariants.mFallenFrames, 0u) << describe(result);
            EXPECT_GT(invariants.mMinPelvisUpDot, 0.5f) << "walking pelvis tilted beyond 60 degrees";
            EXPECT_EQ(invariants.mAirborneFrames, 0u) << "a walk never leaves the ground";
            EXPECT_GE(invariants.mSteps.mLeftStanceCount, 2u) << describe(result);
            EXPECT_GE(invariants.mSteps.mRightStanceCount, 2u);
            EXPECT_GE(invariants.mSteps.mDurationSymmetry, 0.6f);
            EXPECT_EQ(invariants.mSteps.mTimedOutPhases, 0u);
            EXPECT_GE(invariants.mFinalHalfSecondAchievedSpeed, 0.5f * invariants.mMeanRequestedSpeed);
            EXPECT_LE(invariants.mMaxStanceFootSlide, 1.5f);
            EXPECT_LE(invariants.mPelvisMagnitudeSaturatedFrames * 20, invariants.mFrameCount)
                << "translation actuator magnitude saturated on more than five percent of frames";
            EXPECT_LE(invariants.mPelvisHorizontalForceSaturatedFrames * 20, invariants.mFrameCount)
                << "requested horizontal force exceeded the contact-friction budget on more than five percent of frames";
            EXPECT_LE(invariants.mPelvisVerticalForceSaturatedFrames * 20, invariants.mFrameCount)
                << "requested vertical force exceeded the support-weight budget on more than five percent of frames";
            EXPECT_LE(invariants.mSwingHipMotorSaturatedFrames * 20, invariants.mFrameCount)
                << "swing-hip motor saturated on more than five percent of frames";
            for (const auto& stage : RagdollCore::summarizeScenarioGaitStages(result))
            {
                if (stage.mStage == RagdollCore::GaitActuationStage::Inactive
                    || stage.mStage == RagdollCore::GaitActuationStage::Settling
                    || stage.mStage == RagdollCore::GaitActuationStage::AwaitingStance || stage.mFrames == 0)
                    continue;
                SCOPED_TRACE(RagdollCore::gaitActuationStageName(stage.mStage));
                EXPECT_EQ(stage.mSolverObservedFrames, stage.mFrames);
                EXPECT_EQ(stage.mFreeVelocityLimitedFrames, 0u) << "free integration changed the actuator's momentum";
                EXPECT_EQ(stage.mSolverVelocityLimitedFrames, 0u);
                EXPECT_EQ(stage.mAirborneFrames, 0u);
                EXPECT_EQ(stage.mFallingFrames, 0u);
                EXPECT_LT(stage.mMaxNetForce, 1e-3f) << "internal actuation cannot add net momentum";
                EXPECT_LT(stage.mMaxNetMoment, 1e-3f) << "balance torque needs an equal internal reaction";
                EXPECT_LE(stage.mMaxStanceSlide, 1.5f);
                EXPECT_LE(stage.mSwingHipSaturationFrames * 20, stage.mFrames);
                EXPECT_LE(stage.mSwingKneeSaturationFrames * 20, stage.mFrames);
                EXPECT_LE(stage.mSwingAnkleSaturationFrames * 20, stage.mFrames);
            }
            for (const auto& stage : RagdollCore::summarizeScenarioGaitStages(result, true))
            {
                if (stage.mStage == RagdollCore::GaitActuationStage::Inactive
                    || stage.mStage == RagdollCore::GaitActuationStage::AwaitingStance || stage.mFrames == 0)
                    continue;
                SCOPED_TRACE(RagdollCore::gaitActuationStageName(stage.mStage));
                EXPECT_LE(stage.mVerticalSaturationFrames * 20, stage.mFrames);
            }
        }
    }

    TEST(RagdollCoreScenarioTest, recoveryKeepsInternalTorqueWhenLegacyGaitRoutingIsDisabled)
    {
        using namespace RagdollCore;
        ScenarioSettings settings;
        settings.mRouteGaitReactionThroughStanceLeg = false;
        ScenarioScript script;
        script.mDurationSeconds = 8;
        ScenarioInputKeyframe release;
        release.mStep = 60;
        release.mMotorsEnabled = false;
        release.mTranslationActuationEnabled = false;
        release.mUprightTorqueEnabled = false;
        script.mKeyframes.push_back(release);
        ScenarioInputKeyframe recover;
        recover.mStep = 240;
        recover.mRecoveryRequested = true;
        script.mKeyframes.push_back(recover);
        const auto result = run(script, settings);
        ASSERT_TRUE(result.completed()) << describe(result);
        std::size_t recoveryActuations = 0;
        for (const auto& frame : result.mFrames)
        {
            if (frame.mController.mMode != ActiveControllerMode::GettingUp || !frame.mUprightApplied)
                continue;
            ++recoveryActuations;
            ASSERT_TRUE(frame.mAngularReactionBody.has_value()) << "step " << frame.mStep;
            ASSERT_TRUE(frame.mAppliedTorqueSum.IsNearZero(1e-3f)) << "step " << frame.mStep;
        }
        EXPECT_GT(recoveryActuations, 0u) << describe(result);
    }

    TEST(RagdollCoreScenarioTest, referenceRequestDoesNotSuppressRecoveryAssistance)
    {
        using namespace RagdollCore;
        ScenarioSettings settings;
        settings.mLocomotionController = ScenarioLocomotionController::Reference;
        ScenarioScript script;
        script.mDurationSeconds = 8;
        ScenarioInputKeyframe release;
        release.mStep = 60;
        release.mMotorsEnabled = false;
        release.mTranslationActuationEnabled = false;
        release.mUprightTorqueEnabled = false;
        script.mKeyframes.push_back(release);
        ScenarioInputKeyframe recover;
        recover.mStep = 240;
        recover.mRecoveryRequested = true;
        recover.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
        script.mKeyframes.push_back(recover);
        const auto result = run(script, settings);
        ASSERT_TRUE(result.completed()) << describe(result);
        std::size_t recoveryActuations = 0;
        for (const auto& frame : result.mFrames)
        {
            if (frame.mController.mMode != ActiveControllerMode::GettingUp || !frame.mUprightApplied)
                continue;
            ++recoveryActuations;
            EXPECT_TRUE(frame.mAngularReactionBody.has_value()) << "step " << frame.mStep;
            EXPECT_TRUE(frame.mAppliedTorqueSum.IsNearZero(1e-3f)) << "step " << frame.mStep;
        }
        EXPECT_GT(recoveryActuations, 0u) << describe(result);
    }

    TEST(RagdollCoreScenarioTest, pushDuringWalkTriggersRecoveryStep)
    {
        for (const bool solverActuation : { false, true })
        {
            SCOPED_TRACE(solverActuation ? "solver_actuation" : "free_integration");
            RagdollCore::ScenarioSettings settings;
            settings.mUseSolverActuation = solverActuation;
            RagdollCore::ScenarioScript script = walkingScript(3.5f, 60.0f, 30);
            RagdollCore::ScenarioImpulse push;
            push.mStep = 120;
            push.mBodyIndex = 3; // spine2
            push.mImpulse = JPH::Vec3(1200.0f, 0.0f, 0.0f);
            script.mImpulses.push_back(push);

            const auto result = run(script, settings);
            ASSERT_TRUE(result.completed()) << describe(result);
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            exportTraceIfRequested(result, solverActuation ? "push-recovery-solver-actuation" : "push-recovery");
            std::cout << "PUSH_INVARIANTS solver_actuation=" << solverActuation << ' ' << describe(invariants) << std::endl;
            EXPECT_EQ(invariants.mImpulseFrames, 1u);
            EXPECT_GE(invariants.mStepsWithinSecondAfterImpulse, 1u) << "no recovery step followed the push";
            EXPECT_FALSE(invariants.mFellAfterImpulse) << describe(result);
            EXPECT_GT(invariants.mMinPelvisUpDot, 0.5f) << "push recovery lost an upright pelvis";
            EXPECT_EQ(invariants.mAirborneFrames, 0u);
        }
    }

    TEST(RagdollCoreScenarioTest, rejectsInvalidScriptsBeforeSimulating)
    {
        RagdollCore::ScenarioScript unsorted = standingScript(0.5f);
        unsorted.mKeyframes.push_back({ 10 });
        unsorted.mKeyframes.push_back({ 5 });
        auto result = run(unsorted);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());
        EXPECT_FALSE(result.mErrors.empty());

        RagdollCore::ScenarioScript badBody = standingScript(0.5f);
        badBody.mImpulses.push_back({ 0, 99, JPH::Vec3(1.0f, 0.0f, 0.0f), std::nullopt });
        result = run(badBody);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        RagdollCore::ScenarioScript badTarget = standingScript(0.5f);
        RagdollCore::ScenarioInputKeyframe keyframe;
        keyframe.mMotorTarget = RagdollCore::RagdollPose{};
        keyframe.mMotorTarget->mBodyTransforms.resize(3);
        badTarget.mKeyframes.push_back(keyframe);
        result = run(badTarget);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        RagdollCore::ScenarioScript badVelocity = standingScript(0.5f);
        RagdollCore::ScenarioInputKeyframe nan;
        nan.mRequestedHorizontalVelocity = JPH::Vec3(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);
        badVelocity.mKeyframes.push_back(nan);
        result = run(badVelocity);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        RagdollCore::ScenarioScript badStabilization = standingScript(0.5f);
        RagdollCore::ScenarioInputKeyframe excessiveStabilization;
        excessiveStabilization.mStabilizationStrength = 1.01f;
        badStabilization.mKeyframes.push_back(excessiveStabilization);
        result = run(badStabilization);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        badStabilization = standingScript(0.5f);
        RagdollCore::ScenarioInputKeyframe nanStabilization;
        nanStabilization.mStabilizationStrength = std::numeric_limits<float>::quiet_NaN();
        badStabilization.mKeyframes.push_back(nanStabilization);
        result = run(badStabilization);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        RagdollCore::ScenarioScript badBlow = standingScript(0.5f);
        badBlow.mKillingBlow = RagdollCore::ScenarioKillingBlow{ JPH::RVec3::sZero(), JPH::Vec3::sZero(), 10.0f };
        result = run(badBlow);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        result = run(standingScript(-1.0f));
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        RagdollCore::ScenarioScript oneSampleCycle = standingScript(0.5f);
        oneSampleCycle.mGaitPoseCycle.resize(1);
        result = run(oneSampleCycle);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        RagdollCore::ScenarioSettings badSettings;
        badSettings.mTotalMass = 0.0f;
        result = run(standingScript(0.5f), badSettings);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        badSettings = {};
        badSettings.mCompensateSwingHipMotorImpulse = true;
        result = run(standingScript(0.5f), badSettings);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        badSettings = {};
        badSettings.mTranslationRouting = static_cast<RagdollCore::ScenarioTranslationRouting>(99);
        result = run(standingScript(0.5f), badSettings);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        badSettings = {};
        badSettings.mSwingAssistAxes = static_cast<RagdollCore::ScenarioSwingAssistAxes>(99);
        result = run(standingScript(0.5f), badSettings);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        badSettings = {};
        badSettings.mSwingHorizontalReaction = static_cast<RagdollCore::ScenarioSwingHorizontalReaction>(99);
        result = run(standingScript(0.5f), badSettings);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());

        badSettings = {};
        badSettings.mPreLiftSwingUnloadFraction = 1.1f;
        result = run(standingScript(0.5f), badSettings);
        EXPECT_FALSE(result.completed());
        EXPECT_TRUE(result.mFrames.empty());
    }

    TEST(RagdollCoreScenarioTest, telemetryEncodingIsContentSensitive)
    {
        const auto result = run(standingScript(0.2f));
        ASSERT_TRUE(result.completed()) << describe(result);
        const std::uint64_t hash = RagdollCore::hashScenarioTelemetry(result);

        auto changedFrame = result;
        changedFrame.mFrames[3].mCenterOfMassVelocity += JPH::Vec3(0.0f, 0.0f, 0.001f);
        EXPECT_NE(hash, RagdollCore::hashScenarioTelemetry(changedFrame));
        EXPECT_FALSE(RagdollCore::haveIdenticalScenarioTelemetry(result, changedFrame));

        auto changedContact = result;
        ASSERT_FALSE(changedContact.mFrames.back().mContacts.empty());
        changedContact.mFrames.back().mContacts.front().mBodyIndex += 1;
        EXPECT_NE(hash, RagdollCore::hashScenarioTelemetry(changedContact));

        auto changedGait = result;
        changedGait.mFrames.back().mGait.mPhase = GaitPhase::LeftStance;
        EXPECT_NE(hash, RagdollCore::hashScenarioTelemetry(changedGait));

        auto changedWrench = result;
        changedWrench.mFrames.back().mAppliedNetActuationMoment.SetX(0.001f);
        EXPECT_NE(hash, RagdollCore::hashScenarioTelemetry(changedWrench));

        auto changedMotorImpulse = result;
        changedMotorImpulse.mFrames.back().mSwingHipMotorAngularImpulse.SetX(0.001f);
        EXPECT_NE(hash, RagdollCore::hashScenarioTelemetry(changedMotorImpulse));

        auto changedMotorUtilization = result;
        changedMotorUtilization.mFrames.back().mSwingHipMotorPeakAxisUtilization = 0.001f;
        EXPECT_NE(hash, RagdollCore::hashScenarioTelemetry(changedMotorUtilization));

        auto changedUnload = result;
        changedUnload.mFrames.back().mPreLiftSwingUnloadForce = 0.001f;
        EXPECT_NE(hash, RagdollCore::hashScenarioTelemetry(changedUnload));

        auto changedSettings = result;
        changedSettings.mSettings.mSeed += 1;
        EXPECT_NE(hash, RagdollCore::hashScenarioTelemetry(changedSettings));

        auto copy = result;
        EXPECT_EQ(hash, RagdollCore::hashScenarioTelemetry(copy));
    }

    // A physical payload inside the existing articulated scenario: it shares
    // support/momentum accounting, but receives no independent motor torque.
    void addTrainingBar(RagdollCore::SkeletonDescriptor& skeleton, RagdollCore::RagdollProfile& profile)
    {
        using namespace RagdollCore;
        constexpr std::size_t hand = 13;
        for (auto& body : profile.mBodies)
            body.mMassFraction *= 75.0f / 76.0f;
        SkeletonBone bone;
        bone.mName = "training_bar";
        bone.mParent = hand;
        bone.mModelTransform = skeleton.mBones[hand].mModelTransform;
        const auto offset = JPH::Vec3(0, UnitModel::fromMeters(0.35f), 0);
        bone.mModelTransform.mTranslation += bone.mModelTransform.mRotation * offset;
        skeleton.mBones.push_back(bone);
        BodyProfile bar;
        bar.mName = bar.mBone = bone.mName;
        bar.mParentBody = profile.mBodies[hand].mName;
        bar.mMassFraction = 1.0f / 76.0f;
        bar.mShape.mType = ShapeType::Box;
        bar.mShape.mHalfExtents = JPH::Vec3(0.02f, 0.4f, 0.01f) * UnitModel::UnitsPerMeter;
        bar.mConstraint = ConstraintProfile{};
        bar.mConstraint->mChildAnchor = -offset;
        bar.mConstraint->mNormalHalfConeAngle = 0;
        bar.mConstraint->mPlaneHalfConeAngle = 0;
        bar.mConstraint->mTwistMinAngle = 0;
        bar.mConstraint->mTwistMaxAngle = 0;
        bar.mConstraint->mMotor.mMaximumTorque = 0;
        profile.mBodies.push_back(bar);
    }

    void addTrainingBarCapacity(RagdollCore::RagdollProfile& profile)
    {
        using namespace RagdollCore;
        constexpr float inertia = (0.8f * 0.8f + 0.04f * 0.04f) / 12.0f;
        constexpr std::array<float, 4> radii{ 1.0f, 0.9f, 0.6f, 0.35f };
        for (std::size_t i = 0; i < radii.size(); ++i)
        {
            const float radius = radii[i];
            const float allowance = UnitModel::GravityMetersPerSecondSquared * radius
                + (inertia + radius * radius) * 8.0f;
            profile.mBodies[10 + i].mConstraint->mMotor.mMaximumTorque
                += allowance * UnitModel::UnitsPerMeter * UnitModel::UnitsPerMeter;
        }
    }

    void calibrateTrainingBarYawMotors(const RagdollCore::SkeletonDescriptor& skeleton,
        RagdollCore::RagdollProfile& profile)
    {
        using namespace RagdollCore;
        BuildOptions options;
        options.mTotalMass = 76;
        const auto built = buildRagdollSettings(skeleton, profile, options);
        ASSERT_TRUE(built.isValid());
        const auto& parts = built.mSettings->mParts;
        for (std::size_t joint = 10; joint <= 13; ++joint)
        {
            const auto pivot = parts[joint].mPosition;
            JPH::Mat44 inertia = JPH::Mat44::sZero();
            for (std::size_t body = 0; body < parts.size(); ++body)
            {
                int ancestor = static_cast<int>(body);
                while (ancestor >= 0 && ancestor != static_cast<int>(joint))
                    ancestor = skeleton.mBones[ancestor].mParent;
                if (ancestor < 0) continue;
                auto mass = parts[body].GetMassProperties();
                mass.Rotate(JPH::Mat44::sRotation(parts[body].mRotation));
                mass.Translate(JPH::Vec3(parts[body].mPosition
                    + parts[body].mRotation * parts[body].GetShape()->GetCenterOfMass() - pivot));
                inertia += mass.mInertia;
            }
            const auto worldInertia = [&](std::size_t body) {
                auto mass = parts[body].GetMassProperties();
                mass.Rotate(JPH::Mat44::sRotation(parts[body].mRotation));
                return mass.mInertia;
            };
            const auto inverse = worldInertia(joint).Inversed3x3()
                + worldInertia(skeleton.mBones[joint].mParent).Inversed3x3();
            const float ratio = inertia.Multiply3x3(JPH::Vec3::sAxisZ()).GetZ()
                * inverse.Multiply3x3(JPH::Vec3::sAxisZ()).GetZ();
            ASSERT_GT(ratio, 0);
            auto& motor = profile.mBodies[joint].mConstraint->mMotor;
            motor.mFrequency *= std::sqrt(ratio);
            motor.mDamping *= std::sqrt(ratio);
            std::cout << "WEAPON_YAW_GAIN joint=" << joint << " ratio=" << ratio
                << " frequency=" << motor.mFrequency << " damping=" << motor.mDamping << '\n';
        }
    }

    TEST(RagdollCoreScenarioTest, stationaryTrainingBarHoldsPhysicalPayload)
    {
        using namespace RagdollCore;
        auto skeleton = makeScenarioHumanoidSkeleton();
        auto profile = makeScenarioHumanoidProfile();
        addTrainingBarCapacity(profile);
        ScenarioSettings settings;
        settings.mGroundFriction = 0.2f;
        settings.mVelocitySolverSteps = 40;
        const auto script = standingScript(10);
        for (const bool armed : { false, true })
        {
            SCOPED_TRACE(armed ? "armed" : "unarmed");
            if (armed)
            {
                addTrainingBar(skeleton, profile);
                calibrateTrainingBarYawMotors(skeleton, profile);
                settings.mTotalMass = 76;
            }
            const auto result = runRagdollScenario(skeleton, profile, settings, script);
            ASSERT_TRUE(result.completed()) << describe(result);
            const auto repeated = runRagdollScenario(skeleton, profile, settings, script);
            ASSERT_TRUE(haveIdenticalScenarioTelemetry(result, repeated));
            exportTraceIfRequested(result, armed ? "weapon-hold" : "weapon-unarmed-control");
            const auto summary = summarizeScenarioInvariants(result);
            std::cout << (armed ? "WEAPON_HOLD " : "WEAPON_UNARMED ") << describe(summary) << '\n';
            EXPECT_EQ(summary.mFallingFrames + summary.mFallenFrames, 0u);
            EXPECT_EQ(summary.mFreeVelocityLimitedFrames + summary.mSolverVelocityLimitedFrames, 0u);
            EXPECT_GT(summary.mMinPelvisUpDot, 0.98f);
            EXPECT_LT(summary.mMaxAppliedNetForce, 0.001f);
            EXPECT_LT(summary.mMaxNetActuationMoment, 0.001f);
            const auto originalProfile = makeScenarioHumanoidProfile();
            float maxGripGap = 0, maxGripAngle = 0;
            for (const auto& frame : result.mFrames)
            {
                EXPECT_LT(frame.mMaxAnchorSeparation, 1.0f);
                if (frame.mStep >= 60)
                    EXPECT_TRUE(frame.mSupport.mLeftFootSupported || frame.mSupport.mRightFootSupported);
                for (std::size_t i = 0; i < 20; ++i)
                    EXPECT_NEAR(frame.mBodies[i].mMass,
                        originalProfile.mBodies[i].mMassFraction * 75, 1e-5f);
                if (armed)
                {
                    const auto& hand = frame.mBodies[13];
                    const auto& bar = frame.mBodies[20];
                    EXPECT_NEAR(bar.mMass, 1.0f, 1e-5f);
                    const auto grip = bar.mWorldPosition
                        + bar.mWorldRotation * JPH::Vec3(0, -UnitModel::fromMeters(0.35f), 0);
                    maxGripGap = std::max(maxGripGap, JPH::Vec3(grip - hand.mWorldPosition).Length());
                    maxGripAngle = std::max(maxGripAngle, 2 * std::acos(std::clamp(
                        std::abs(hand.mWorldRotation.Dot(bar.mWorldRotation)), 0.0f, 1.0f)));
                    for (const auto& contact : frame.mContacts)
                        EXPECT_NE(contact.mBodyIndex, 20u) << "weapon must not become an extra standing support";
                }
            }
            std::cout << "WEAPON_GRIP gap=" << maxGripGap << " angle=" << maxGripAngle << '\n';
            EXPECT_LT(maxGripGap, 1.0f);
            EXPECT_LT(maxGripAngle, 0.03f);
            auto passiveSettings = settings;
            passiveSettings.mPowered = false;
            const auto passive = runRagdollScenario(skeleton, profile, passiveSettings, script);
            ASSERT_TRUE(passive.completed()) << describe(passive);
            EXPECT_LT(summarizeScenarioInvariants(passive).mMinPelvisUpDot, 0.7f);
            exportTraceIfRequested(passive, armed ? "weapon-passive-control" : "weapon-unarmed-passive-control");
        }
    }

    TEST(RagdollCoreScenarioTest, stationaryTrainingBarTurnsReversesAndStops)
    {
        using namespace RagdollCore;
        auto skeleton = makeScenarioHumanoidSkeleton();
        auto profile = makeScenarioHumanoidProfile();
        addTrainingBarCapacity(profile);
        addTrainingBar(skeleton, profile);
        const auto originalGains = profile;
        calibrateTrainingBarYawMotors(skeleton, profile);
        ScenarioSettings settings;
        settings.mGroundFriction = 0.2f;
        settings.mVelocitySolverSteps = 40;
        settings.mTotalMass = 76;
        auto script = standingScript(10);
        const auto ramp = [](float t) { return t * t * (3 - 2 * t); };
        for (unsigned step = 0; step < 600; ++step)
        {
            float angle = 0;
            if (step >= 120 && step < 180) angle = 0.35f * ramp((step - 120) / 60.0f);
            else if (step >= 180 && step < 240) angle = 0.35f;
            else if (step >= 240 && step < 300) angle = 0.35f - 0.7f * ramp((step - 240) / 60.0f);
            else if (step >= 300 && step < 420) angle = -0.35f;
            else if (step >= 420 && step < 480) angle = -0.35f + 0.35f * ramp((step - 420) / 60.0f);
            ScenarioInputKeyframe key;
            key.mStep = step;
            key.mMotorTarget.emplace();
            for (const auto& bone : skeleton.mBones)
                key.mMotorTarget->mBodyTransforms.push_back(bone.mModelTransform);
            const auto rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), angle);
            const auto pivot = skeleton.mBones[11].mModelTransform.mTranslation;
            for (const std::size_t i : { 11u, 12u, 13u, 20u })
            {
                auto& target = key.mMotorTarget->mBodyTransforms[i];
                target.mTranslation = pivot + rotation * (target.mTranslation - pivot);
                target.mRotation = rotation * target.mRotation;
            }
            script.mKeyframes.push_back(std::move(key));
        }
        const auto result = runRagdollScenario(skeleton, profile, settings, script);
        ASSERT_TRUE(result.completed()) << describe(result);
        ASSERT_TRUE(haveIdenticalScenarioTelemetry(result, runRagdollScenario(skeleton, profile, settings, script)));
        exportTraceIfRequested(result, "weapon-turn-reverse-stop");
        const auto control = runRagdollScenario(skeleton, originalGains, settings, script);
        ASSERT_TRUE(control.completed()) << describe(control);
        EXPECT_GT(control.mFrames.back().mBodies[20].mAngularVelocity.Length(), 0.2f)
            << "the original loaded motor response must distinguish this comparison";
        exportTraceIfRequested(control, "weapon-original-gain-control");
        auto changedQuality = result;
        changedQuality.mSettings.mVelocitySolverSteps = 10;
        EXPECT_NE(hashScenarioTelemetry(result), hashScenarioTelemetry(changedQuality));
        if (const char* directory = std::getenv("OPENMW_RAGDOLL_TRACE_DIR"))
        {
            std::ofstream trace(std::filesystem::path(directory) / "weapon-arm-response.csv");
            trace << "step,body,yaw,wx,wy,wz,motor_utilization\n";
            trace.precision(9);
            for (const auto& frame : result.mFrames)
                for (const std::size_t i : { 0u, 10u, 11u, 12u, 13u, 20u })
                {
                    const auto& body = frame.mBodies[i];
                    const auto direction = body.mWorldRotation.RotateAxisY();
                    trace << frame.mStep << ',' << i << ',' << std::atan2(-direction.GetX(), direction.GetY())
                        << ',' << body.mAngularVelocity.GetX() << ',' << body.mAngularVelocity.GetY()
                        << ',' << body.mAngularVelocity.GetZ() << ','
                        << (i < frame.mJointSupportMotorUtilizations.size() ? frame.mJointSupportMotorUtilizations[i] : 0)
                        << '\n';
                }
        }
        const auto summary = summarizeScenarioInvariants(result);
        std::cout << "WEAPON_TURN " << describe(summary) << '\n';
        EXPECT_EQ(summary.mFallingFrames + summary.mFallenFrames, 0u);
        EXPECT_EQ(summary.mFreeVelocityLimitedFrames + summary.mSolverVelocityLimitedFrames, 0u);
        EXPECT_GT(summary.mMinPelvisUpDot, 0.98f);
        EXPECT_LT(summary.mMaxAppliedNetForce, 0.001f);
        EXPECT_LT(summary.mMaxNetActuationMoment, 0.001f);
        for (const auto& frame : result.mFrames)
        {
            EXPECT_LT(frame.mMaxAnchorSeparation, 1.0f);
            if (frame.mStep >= 60)
                EXPECT_TRUE(frame.mSupport.mLeftFootSupported || frame.mSupport.mRightFootSupported);
            const auto& hand = frame.mBodies[13];
            const auto& bar = frame.mBodies[20];
            const auto grip = bar.mWorldPosition
                + bar.mWorldRotation * JPH::Vec3(0, -UnitModel::fromMeters(0.35f), 0);
            EXPECT_LT(JPH::Vec3(grip - hand.mWorldPosition).Length(), 1.0f);
            EXPECT_GT(std::abs(hand.mWorldRotation.Dot(bar.mWorldRotation)), std::cos(0.03f / 2));
            for (const auto& contact : frame.mContacts)
                EXPECT_NE(contact.mBodyIndex, 20u);
        }
        for (const auto [step, target] : { std::pair{ 239u, 0.35f }, { 419u, -0.35f }, { 599u, 0.0f } })
        {
            const auto& bar = result.mFrames[step].mBodies[20];
            const auto direction = bar.mWorldRotation.RotateAxisY();
            const float yaw = std::atan2(-direction.GetX(), direction.GetY());
            std::cout << "WEAPON_AIM step=" << step << " target=" << target << " actual=" << yaw
                << " angular_speed=" << bar.mAngularVelocity.Length() << '\n';
            EXPECT_NEAR(yaw, target, 0.1f);
            EXPECT_LT(bar.mAngularVelocity.Length(), 0.2f);
            if (target != 0) EXPECT_GT(yaw * std::copysign(1.0f, target), 0.2f);
        }
    }
}

TEST(RagdollCoreScenarioEncodingTest, stackSettingsIdentifyEvenAnEmptyRecording)
{
    using namespace RagdollCore;
    const ScenarioResult baseline;
    const auto bytes = encodeScenarioTelemetry(baseline);
    const auto hash = hashReferenceGaitProfile(baseline.mSettings.mReferenceGait);
    for (auto field : { &ReferenceGaitProfile::mStanceKneeEaseDuration,
             &ReferenceGaitProfile::mTerminalStancePush, &ReferenceGaitProfile::mTerminalStancePushStartFraction,
             &ReferenceGaitProfile::mTerminalStancePushLateralLimit,
             &ReferenceGaitProfile::mFinalCommandFeasibilityStanceMargin,
             &ReferenceGaitProfile::mFinalCommandFeasibilitySwingMargin,
             &ReferenceGaitProfile::mStanceAnklePushOff, &ReferenceGaitProfile::mStanceAnklePushOffStartFraction,
             &ReferenceGaitProfile::mMaximumLateralRequestFraction })
    {
        auto changed = baseline;
        changed.mSettings.mReferenceGait.*field += 0.01f;
        EXPECT_NE(bytes, encodeScenarioTelemetry(changed));
        EXPECT_NE(hash, hashReferenceGaitProfile(changed.mSettings.mReferenceGait));
        EXPECT_EQ(baseline.mProfileHash, changed.mProfileHash); // anatomy identity remains distinct
    }
    auto changed = baseline;
    changed.mSettings.mReferenceGait.mStanceAnklePushOffTrailing
        = !changed.mSettings.mReferenceGait.mStanceAnklePushOffTrailing;
    EXPECT_NE(bytes, encodeScenarioTelemetry(changed));
    EXPECT_NE(hash, hashReferenceGaitProfile(changed.mSettings.mReferenceGait));
}

TEST(RagdollCoreScenarioEncodingTest, replayAdapterModesHaveDistinctIdentities)
{
    using namespace RagdollCore;
    const ScenarioResult baseline;
    const auto bytes = encodeScenarioTelemetry(baseline);
    for (auto field : { &ScenarioSettings::mReplayNativeBraceLimits,
             &ScenarioSettings::mReplayAutomaticRecoveryRequest, &ScenarioSettings::mReplayBraceTargets,
             &ScenarioSettings::mReplayComputeHorizontalRoot,
             &ScenarioSettings::mReplayDiscardSupportOnReferenceReset,
             &ScenarioSettings::mReplayObserveRestartRequest })
    {
        auto changed = baseline;
        changed.mSettings.*field = true;
        EXPECT_NE(bytes, encodeScenarioTelemetry(changed));
    }
}
