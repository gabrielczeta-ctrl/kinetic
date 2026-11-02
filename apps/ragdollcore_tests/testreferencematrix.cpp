#include <ragdollcore/referencegait.hpp>
#include <ragdollcore/scenario.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

// Robustness matrix around the retained Reference locomotion controller.
//
// This is DIAGNOSTIC evidence, not acceptance. Every case reuses the ordinary
// walking script (walk request at a fixed step), the fixed 60 Hz timestep,
// the default scenario settings and the existing scripted-impulse input. No
// physical gate of the walking or push contracts is asserted here; those live
// unchanged in testreferencegait.cpp. The only assertions are harness
// contracts: the scenario completes, the requested disturbance was applied,
// and an exact repeat reproduces the telemetry (determinism, plan rule R6).
//
// The per-case metrics are printed as one "REFERENCE_MATRIX" key=value line
// so scripts/ragdoll-matrix/ragdoll_report.py can tabulate baseline and
// candidate executables on identical cases. Metrics that cannot be measured
// are printed as "unavailable", never as zero.
namespace
{
    struct MatrixCase
    {
        // gtest parameter name: identifier characters only.
        const char* mName;
        float mSpeed; // requested forward speed, world units per second
        float mSeconds; // scenario duration
        std::uint64_t mWalkStep; // step of the walk request
        bool mPush;
        std::uint64_t mPushStep; // scripted impulse step (applied before that step's physics)
        std::size_t mPushBody; // body index; 3 is spine2, as in the existing push test
        float mImpulseX; // world impulse, mass units x world units / s
        float mImpulseY;
        const char* mKind; // diagnostic | held-out candidates are chosen by the main developer
        const char* mRationale;
        std::optional<std::uint64_t> mSpeedChangeStep;
        float mFollowingSpeed = 0.0f;
    };

    // Predeclared case list (see docs/ragdoll/reference-matrix.md). Kept to
    // eleven cases on purpose: speeds, signed lateral/forward pushes at two
    // predeclared times, one half-magnitude push and one speed/push pairing.
    // Step 120 (2.0 s) and step 180 (3.0 s) fall inside different stances of
    // the retained 60 units/s walk (transfers at 60/100/129/157/201), so the
    // two timings sample two phases without any event-triggered push.
    // 1200 impulse units on a 75 mass-unit actor is a 16 units/s (0.23 m/s)
    // whole-body velocity change, identical to the existing short push.
    const MatrixCase sCases[] = {
        { "nominal60", 60.0f, 7.0f, 30, false, 0, 3, 0.0f, 0.0f, "diagnostic",
            "same script as referenceWalkingSatisfiesGaitInvariants; anchors the fall time" },
        { "unpushed45", 45.0f, 7.0f, 30, false, 0, 3, 0.0f, 0.0f, "diagnostic",
            "slower request; separates speed-tracking failure from balance failure" },
        { "unpushed30", 30.0f, 7.0f, 30, false, 0, 3, 0.0f, 0.0f, "diagnostic",
            "half speed; the legacy contract's original request magnitude" },
        { "lateralPos1200at120", 60.0f, 7.0f, 30, true, 120, 3, 1200.0f, 0.0f, "diagnostic",
            "identical impulse to the retained short push, observed for the full 7 s" },
        { "lateralNeg1200at120", 60.0f, 7.0f, 30, true, 120, 3, -1200.0f, 0.0f, "diagnostic",
            "mirror of the retained push; exposes left/right asymmetry of the stance" },
        { "forward1200at120", 60.0f, 7.0f, 30, true, 120, 3, 0.0f, 1200.0f, "diagnostic",
            "forward shove in the request direction; probes braking/placement authority" },
        { "backward1200at120", 60.0f, 7.0f, 30, true, 120, 3, 0.0f, -1200.0f, "diagnostic",
            "backward shove; probes forward-acceleration authority and stance retention" },
        { "lateralPos1200at180", 60.0f, 7.0f, 30, true, 180, 3, 1200.0f, 0.0f, "diagnostic",
            "same lateral push one second later, inside a different stance phase" },
        { "backward1200at180", 60.0f, 7.0f, 30, true, 180, 3, 0.0f, -1200.0f, "diagnostic",
            "same backward push one second later, inside a different stance phase" },
        { "lateralPos600at120", 60.0f, 7.0f, 30, true, 120, 3, 600.0f, 0.0f, "diagnostic",
            "half magnitude; shows whether recovery scales with the disturbance" },
        { "speed45LateralPos1200at120", 45.0f, 7.0f, 30, true, 120, 3, 1200.0f, 0.0f, "diagnostic",
            "speed/push pairing; whether the slower gait tolerates the retained push" },
        { "reduce60to30at120", 60.0f, 7.0f, 30, false, 0, 3, 0.0f, 0.0f, "diagnostic",
            "same prefix as nominal60; nonzero reduction preserves the reference route", 120, 30.0f },
        { "stop60at120", 60.0f, 7.0f, 30, false, 0, 3, 0.0f, 0.0f, "diagnostic",
            "same prefix as nominal60; zero request separately tests transition to standing", 120, 0.0f },
    };

    RagdollCore::ScenarioSettings referenceSettings()
    {
        RagdollCore::ScenarioSettings settings;
        settings.mLocomotionController = RagdollCore::ScenarioLocomotionController::Reference;
        return settings;
    }

    RagdollCore::ScenarioScript scriptFor(const MatrixCase& matrixCase)
    {
        RagdollCore::ScenarioScript script;
        script.mDurationSeconds = matrixCase.mSeconds;
        RagdollCore::ScenarioInputKeyframe walk;
        walk.mStep = matrixCase.mWalkStep;
        walk.mRequestedHorizontalVelocity = JPH::Vec3(0.0f, matrixCase.mSpeed, 0.0f);
        script.mKeyframes.push_back(walk);
        if (matrixCase.mSpeedChangeStep)
        {
            auto change = walk;
            change.mStep = *matrixCase.mSpeedChangeStep;
            change.mRequestedHorizontalVelocity = JPH::Vec3(0.0f, matrixCase.mFollowingSpeed, 0.0f);
            script.mKeyframes.push_back(change);
        }
        if (matrixCase.mPush)
        {
            RagdollCore::ScenarioImpulse push;
            push.mStep = matrixCase.mPushStep;
            push.mBodyIndex = matrixCase.mPushBody;
            push.mImpulse = JPH::Vec3(matrixCase.mImpulseX, matrixCase.mImpulseY, 0.0f);
            script.mImpulses.push_back(push);
        }
        return script;
    }

    RagdollCore::ScenarioResult run(
        const RagdollCore::ScenarioScript& script, const RagdollCore::ScenarioSettings& settings)
    {
        return RagdollCore::runRagdollScenario(
            RagdollCore::makeScenarioHumanoidSkeleton(), RagdollCore::makeScenarioHumanoidProfile(), settings, script);
    }

    std::string optionalStep(const std::optional<std::uint64_t>& value)
    {
        return value ? std::to_string(*value) : std::string("unavailable");
    }

    void exportIfRequested(const RagdollCore::ScenarioResult& result, const std::string& name)
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
        std::ofstream stages(directory / (name + "-stages.csv"), std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(stages) << "Could not open stage summary output";
        stages << RagdollCore::formatScenarioGaitStagesCsv(result);
    }

    class ReferenceMatrixTest : public ::testing::TestWithParam<MatrixCase>
    {
    };

    TEST_P(ReferenceMatrixTest, runsCase)
    {
        const MatrixCase& matrixCase = GetParam();
        const auto settings = referenceSettings();
        const auto script = scriptFor(matrixCase);
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        const auto& profile = settings.mReferenceGait;

        // Resolved settings, once per case, so a report can prove that
        // baseline and candidate ran the same fixture.
        std::cout << "REFERENCE_MATRIX_SETTINGS case=" << matrixCase.mName
                  << " fixed_time_step=" << settings.mFixedTimeStep
                  << " collision_steps=" << settings.mCollisionSteps
                  << " velocity_solver_steps=" << settings.mVelocitySolverSteps
                  << " continuous_collision=" << settings.mUseContinuousCollision
                  << " gravity=" << settings.mGravity << " ground_friction=" << settings.mGroundFriction
                  << " total_mass=" << settings.mTotalMass
                  << " controller=" << static_cast<int>(settings.mLocomotionController)
                  << " double_support=" << profile.mDoubleSupportDuration
                  << " swing_duration=" << profile.mSwingDuration
                  << " max_step_duration=" << profile.mMaximumStepDuration
                  << " lateral_capture_offset=" << profile.mLateralCaptureOffset
                  << " velocity_gain=" << profile.mVelocityGain
                  << " transfer_velocity_gain=" << profile.mTransferVelocityGain
                  << " max_requested_acceleration=" << profile.mMaximumRequestedAcceleration
                  << " swing_leg_frequency=" << profile.mSwingLegFrequency
                  << " loaded_ankle_frequency=" << profile.mLoadedAnkleFrequency
                  << " max_step_reach=" << profile.mMaximumStepReach << std::endl;
        std::cout << "REFERENCE_MATRIX_SPEC case=" << matrixCase.mName << " kind=" << matrixCase.mKind
                  << " speed=" << matrixCase.mSpeed << " seconds=" << matrixCase.mSeconds
                  << " walk_step=" << matrixCase.mWalkStep << " push=" << matrixCase.mPush
                  << " push_step=" << (matrixCase.mPush ? std::to_string(matrixCase.mPushStep) : "none")
                  << " push_body=" << (matrixCase.mPush ? std::to_string(matrixCase.mPushBody) : "none")
                  << " push_body_name=" << (matrixCase.mPush ? skeleton.mBones[matrixCase.mPushBody].mName : "none")
                  << " impulse_x=" << matrixCase.mImpulseX << " impulse_y=" << matrixCase.mImpulseY
                  << " impulse_magnitude=" << std::hypot(matrixCase.mImpulseX, matrixCase.mImpulseY)
                  << " impulse_units=mass_units*world_units/s"
                  << " rationale=\"" << matrixCase.mRationale << '"';
        if (matrixCase.mSpeedChangeStep)
            std::cout << " speed_change_step=" << *matrixCase.mSpeedChangeStep
                      << " following_speed=" << matrixCase.mFollowingSpeed;
        std::cout << std::endl;

        const auto result = run(script, settings);
        exportIfRequested(result, std::string("matrix-") + matrixCase.mName);
        ASSERT_TRUE(result.completed()) << "harness did not complete: "
                                        << (result.mErrors.empty() ? "unknown" : result.mErrors.front());

        const auto invariants = RagdollCore::summarizeScenarioInvariants(result);

        // Reference-specific counts (same definitions as testreferencegait.cpp).
        std::size_t referenceFrames = 0, saturatedFrames = 0, swingFrames = 0, doubleSupportFrames = 0;
        std::size_t timedOutFrames = 0, reachLimitedFrames = 0, liftedFrames = 0;
        std::size_t leftSupportFrames = 0, rightSupportFrames = 0, noSupportFrames = 0;
        float maxUtilization = 0.0f, minPelvisHeight = std::numeric_limits<float>::max(), maxPelvisHeight = 0.0f;
        std::optional<std::uint64_t> firstFalling, firstTransfer, firstSwing, firstLift;
        std::vector<std::uint64_t> transfers;
        std::map<std::string, std::size_t> saturatedBodies;
        for (const auto& frame : result.mFrames)
        {
            const bool falling = frame.mController.mMode == RagdollCore::ActiveControllerMode::Falling
                || frame.mController.mMode == RagdollCore::ActiveControllerMode::Fallen;
            if (falling && !firstFalling)
                firstFalling = frame.mStep;
            if (frame.mGait.mTransitioned && frame.mGait.hasStance() && frame.mReferenceGait)
            {
                transfers.push_back(frame.mStep);
                if (!firstTransfer)
                    firstTransfer = frame.mStep;
            }
            if (frame.mGait.mPhaseTimedOut)
                ++timedOutFrames;
            bool left = false, right = false;
            for (const auto& contact : frame.mContacts)
            {
                if (!contact.hasSupport())
                    continue;
                left = left || contact.mBodyIndex == result.mLandmarks.mLeftFoot;
                right = right || contact.mBodyIndex == result.mLandmarks.mRightFoot;
            }
            if (frame.mStep >= matrixCase.mWalkStep)
            {
                leftSupportFrames += left;
                rightSupportFrames += right;
                noSupportFrames += !left && !right;
            }
            if (!frame.mReferenceGait)
                continue;
            const auto& record = *frame.mReferenceGait;
            ++referenceFrames;
            minPelvisHeight = std::min(minPelvisHeight, frame.mPelvisHeight);
            maxPelvisHeight = std::max(maxPelvisHeight, frame.mPelvisHeight);
            maxUtilization = std::max(maxUtilization, record.mMaxMotorUtilization);
            if (record.mMaxMotorUtilization >= 0.999f)
            {
                ++saturatedFrames;
                const std::size_t body = record.mMaxMotorUtilizationBody;
                ++saturatedBodies[body < skeleton.mBones.size() ? skeleton.mBones[body].mName : std::to_string(body)];
            }
            if (record.mSwingReachLimited)
                ++reachLimitedFrames;
            if (record.mStage == RagdollCore::ReferenceGaitStage::Swing)
            {
                ++swingFrames;
                if (!firstSwing)
                    firstSwing = frame.mStep;
            }
            if (record.mStage == RagdollCore::ReferenceGaitStage::DoubleSupport)
                ++doubleSupportFrames;
            if (frame.mGait.mSwingFootLifted)
            {
                ++liftedFrames;
                if (!firstLift)
                    firstLift = frame.mStep;
            }
        }

        // Exact repeat: determinism is a harness contract, recorded separately
        // from the physical outcome.
        const auto repeat = run(script, settings);
        const bool repeatCompleted = repeat.completed();
        const bool repeatIdentical
            = repeatCompleted && RagdollCore::haveIdenticalScenarioTelemetry(result, repeat);
        std::cout << "REFERENCE_MATRIX_REPEAT case=" << matrixCase.mName << " completed=" << repeatCompleted
                  << " identical=" << repeatIdentical << std::endl;

        std::ostringstream line;
        line << "REFERENCE_MATRIX case=" << matrixCase.mName << " completed=1 frames=" << result.mFrames.size()
             << " first_falling_step=" << optionalStep(firstFalling)
             << " first_falling_time=" << (firstFalling ? std::to_string(*firstFalling * settings.mFixedTimeStep) : "unavailable")
             << " survived_seconds_after_request="
             << ((firstFalling ? static_cast<float>(*firstFalling) : static_cast<float>(result.mFrames.size()))
                    - static_cast<float>(matrixCase.mWalkStep)) * settings.mFixedTimeStep
             << " transfers=" << transfers.size() << " transfer_steps=";
        for (const auto step : transfers)
            line << step << ';';
        line << " first_swing=" << optionalStep(firstSwing) << " first_lift=" << optionalStep(firstLift)
             << " first_transfer=" << optionalStep(firstTransfer)
             << " left_steps=" << invariants.mSteps.mLeftStanceCount
             << " right_steps=" << invariants.mSteps.mRightStanceCount
             << " symmetry=" << invariants.mSteps.mDurationSymmetry
             << " timed_out_phases=" << invariants.mSteps.mTimedOutPhases << " timed_out_frames=" << timedOutFrames
             << " reference_frames=" << referenceFrames << " double_support=" << doubleSupportFrames
             << " swing=" << swingFrames << " lifted=" << liftedFrames
             << " saturated=" << saturatedFrames
             << " saturated_fraction=" << (referenceFrames ? static_cast<float>(saturatedFrames) / referenceFrames : 0.0f)
             << " max_utilization=" << maxUtilization << " saturated_bodies=";
        if (saturatedBodies.empty())
            line << "none";
        for (const auto& [name, count] : saturatedBodies)
            line << name << ':' << count << ';';
        line << " reach_limited=" << reachLimitedFrames
             << " mean_requested=" << invariants.mMeanRequestedSpeed
             << " mean_achieved=" << invariants.mMeanAchievedSpeed
             << " final_half_second_achieved=" << invariants.mFinalHalfSecondAchievedSpeed
             << " max_tracking_error=" << invariants.mMaxTrackingError
             << " max_foot_slide=" << invariants.mMaxStanceFootSlide
             << " total_foot_slide=" << invariants.mTotalStanceFootSlide
             << " airborne=" << invariants.mAirborneFrames
             << " max_consecutive_airborne=" << invariants.mMaxConsecutiveAirborneFrames
             << " free_velocity_limited=" << invariants.mFreeVelocityLimitedFrames
             << " solver_velocity_limited=" << invariants.mSolverVelocityLimitedFrames
             << " solver_observed=" << invariants.mSolverObservedFrames
             << " left_support_frames=" << leftSupportFrames << " right_support_frames=" << rightSupportFrames
             << " no_support_frames=" << noSupportFrames
             << " pelvis_height_min=" << (referenceFrames ? minPelvisHeight : 0.0f)
             << " pelvis_height_max=" << maxPelvisHeight
             << " min_up=" << invariants.mMinPelvisUpDot << " max_body_speed=" << invariants.mMaxBodySpeed
             << " falling=" << invariants.mFallingFrames << " fallen=" << invariants.mFallenFrames
             << " max_net_force=" << invariants.mMaxAppliedNetForce
             << " max_net_moment=" << invariants.mMaxNetActuationMoment
             << " impulses=" << invariants.mImpulseFrames
             << " steps_after_push=" << (matrixCase.mPush ? std::to_string(invariants.mStepsWithinSecondAfterImpulse) : "unavailable")
             << " fell_after_push=" << (matrixCase.mPush ? std::to_string(invariants.mFellAfterImpulse) : "unavailable")
             << " repeat_identical=" << repeatIdentical;
        std::cout << line.str() << std::endl;

        // Harness contracts only. No physical acceptance gate is applied.
        EXPECT_EQ(invariants.mImpulseFrames, matrixCase.mPush ? 1u : 0u) << "the scripted disturbance was not applied as declared";
        EXPECT_TRUE(repeatCompleted) << "the exact repeat did not complete";
        EXPECT_TRUE(repeatIdentical) << "the exact repeat changed the telemetry";
    }

    INSTANTIATE_TEST_SUITE_P(Cases, ReferenceMatrixTest, ::testing::ValuesIn(sCases),
        [](const ::testing::TestParamInfo<MatrixCase>& info) { return std::string(info.param.mName); });
}
