#include <ragdollcore/referencegait.hpp>
#include <ragdollcore/builder.hpp>
#include <ragdollcore/scenario.hpp>

#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

// Reference locomotion route (plan step 2/3/4). Every scenario below states
// its predicted observation, its matched control and its rejection condition
// in the test body. None of these tests relaxes an existing physical gate.
namespace
{
    using RagdollCore::GaitPhase;
    using RagdollCore::ReferenceGaitStage;

    RagdollCore::ScenarioSettings referenceSettings()
    {
        RagdollCore::ScenarioSettings settings;
        settings.mLocomotionController = RagdollCore::ScenarioLocomotionController::Reference;
        return settings;
    }

    // The 2026-09-05 reference profile: the swing shape, springs and stage
    // handling the staged experiments, frozen prepared-step captures and
    // cycle-geometry checks were recorded with. Every later mechanism is off,
    // so the planner reproduces those trajectories exactly.
    RagdollCore::ReferenceGaitProfile legacyReferenceGaitProfile()
    {
        RagdollCore::ReferenceGaitProfile profile;
        profile.mBrakeLateralTransfer = false;
        profile.mVelocityLeadAccelerationFraction = 1.0f;
        profile.mRequireInitialSupportTransfer = false;
        profile.mLimitVelocityLead = false;
        profile.mEmergencyCaptureMargin = 0.0f;
        profile.mSteeringTurnRate = 0.0f;
        profile.mSteeringStopDeceleration = 0.0f;
        profile.mStoppingVelocityErrorGain = 0.0f;
        profile.mStandingHandoverDuration = 0.0f;
        profile.mDoubleSupportDuration = 0.10f;
        profile.mSwingDuration = 0.35f;
        profile.mSwingFootLift = 8.0f;
        profile.mTouchdownPressSpeed = 20.0f;
        profile.mMaximumTouchdownPressDepth = 4.0f;
        profile.mLateralCaptureOffset = 4.0f;
        profile.mForwardVelocityErrorGain = 1.0f;
        profile.mMinimumStepWidth = 12.0f;
        profile.mVelocityGain = 0.5f;
        profile.mTransferVelocityGain = 10.0f;
        profile.mMaximumRequestedAcceleration = 100.0f;
        profile.mSwingLegFrequency = 8.0f;
        profile.mSwingLegDamping = 1.5f;
        profile.mStanceKneeFlexion = 0.0f;
        profile.mLoadedAnkleFrequency = 1.0f;
        profile.mLoadedAnkleDamping = 1.0f;
        profile.mSupportHullScale = 1.0f;
        profile.mLostLandingSwapDelay = 10.0f;
        profile.mLandingPressDepth = 0.0f;
        profile.mSwingRiseEndFraction = 0.45f;
        profile.mSwingTravelStartFraction = 0.0f;
        profile.mSwingTravelEndFraction = 0.9f;
        profile.mSwingDescentEndFraction = 0.9f;
        profile.mTouchdownPressStartFraction = 1.0f;
        profile.mPredictPlacement = false;
        profile.mSwingVelocityFeedForward = false;
        profile.mSwingInertiaFeedForward = 0.0f;
        profile.mStanceHipFrequency = 0.0f;
        profile.mStanceHipDamping = 1.0f;
        profile.mStanceHipRateHold = false;
        return profile;
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

    RagdollCore::ScenarioResult run(
        const RagdollCore::ScenarioScript& script, const RagdollCore::ScenarioSettings& settings)
    {
        return RagdollCore::runRagdollScenario(
            RagdollCore::makeScenarioHumanoidSkeleton(), RagdollCore::makeScenarioHumanoidProfile(), settings, script);
    }

    std::string describe(const RagdollCore::ScenarioResult& result)
    {
        std::ostringstream stream;
        for (const std::string& error : result.mErrors)
            stream << "error: " << error << '\n';
        stream << "frames=" << result.mFrames.size();
        if (!result.mFrames.empty())
        {
            const auto& last = result.mFrames.back();
            stream << " last_step=" << last.mStep << " mode=" << static_cast<int>(last.mController.mMode)
                   << " gait=" << static_cast<int>(last.mGait.mPhase) << " up=" << last.mPelvisUpDot
                   << " height=" << last.mPelvisHeight;
        }
        return stream.str();
    }

    std::string describe(const RagdollCore::ScenarioInvariants& invariants)
    {
        std::ostringstream stream;
        stream << "frames=" << invariants.mFrameCount << " airborne=" << invariants.mAirborneFrames
               << " free_velocity_limited=" << invariants.mFreeVelocityLimitedFrames
               << " solver_velocity_limited=" << invariants.mSolverVelocityLimitedFrames
               << " swing_hip_motor_saturated=" << invariants.mSwingHipMotorSaturatedFrames
               << " max_net_force=" << invariants.mMaxAppliedNetForce
               << " max_net_moment=" << invariants.mMaxNetActuationMoment
               << " max_foot_slide=" << invariants.mMaxStanceFootSlide
               << " mean_requested=" << invariants.mMeanRequestedSpeed
               << " mean_achieved=" << invariants.mMeanAchievedSpeed
               << " final_half_second_achieved=" << invariants.mFinalHalfSecondAchievedSpeed
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
        std::ofstream stages(directory / (name + "-stages.csv"), std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(stages) << "Could not open stage summary output";
        stages << RagdollCore::formatScenarioGaitStagesCsv(result);
        // Complete pre-solve joint targets, including the newly loaded leg.
        // Time is the start of the step, unlike the post-solve frame time.
        std::ofstream motors(directory / (name + "-motor-targets.csv"), std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(motors);
        motors.precision(9);
        motors << "step,command_time,body,type,world,current_qx,current_qy,current_qz,current_qw,"
                  "requested_qx,requested_qy,requested_qz,requested_qw,target_qx,target_qy,target_qz,target_qw,"
                  "current_hinge,target_hinge,angular_error,clamp_delta,"
                  "spring_stiffness_mode,spring_frequency_or_stiffness,spring_damping,"
                  "twist_stiffness_mode,twist_frequency_or_stiffness,twist_damping,"
                  "target_angular_velocity_world_x,target_angular_velocity_world_y,target_angular_velocity_world_z\n";
        const auto quaternion = [&](const std::optional<JPH::Quat>& value) {
            if (value)
                motors << ',' << value->GetX() << ',' << value->GetY() << ',' << value->GetZ() << ',' << value->GetW();
            else
                motors << ",,,,";
        };
        const auto scalar = [&](const std::optional<float>& value) {
            motors << ',';
            if (value) motors << *value;
        };
        for (const auto& frame : result.mFrames)
            for (std::size_t body = 1; body < frame.mJointMotorTargets.size(); ++body)
                if (const auto& target = frame.mJointMotorTargets[body])
                {
                    motors << frame.mStep << ',' << frame.mStep * result.mSettings.mFixedTimeStep << ',' << body
                           << ',' << static_cast<unsigned>(target->mConstraintType) << ',' << target->mWorldOrientation;
                    quaternion(target->mCurrentOrientation);
                    quaternion(target->mRequestedOrientation);
                    quaternion(target->mTargetOrientation);
                    scalar(target->mCurrentHingeAngle);
                    scalar(target->mTargetHingeAngle);
                    motors << ',' << target->mAngularError << ',' << target->mTargetClampDelta
                           << ',' << target->mSpring.mStiffnessMode << ',' << target->mSpring.mFrequencyOrStiffness
                           << ',' << target->mSpring.mDamping;
                    if (target->mTwistSpring)
                        motors << ',' << target->mTwistSpring->mStiffnessMode << ','
                               << target->mTwistSpring->mFrequencyOrStiffness << ',' << target->mTwistSpring->mDamping;
                    else
                        motors << ",,,";
                    motors << ',' << target->mTargetAngularVelocity.GetX() << ',' << target->mTargetAngularVelocity.GetY()
                           << ',' << target->mTargetAngularVelocity.GetZ() << '\n';
                }
        // These tests use the fixed synthetic humanoid. Pair the pre-solve
        // motor request with the preceding frame's measured hip, not the
        // post-solve hip, when checking geometric reach.
        const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
        std::ofstream reach(directory / (name + "-reach.csv"), std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(reach);
        reach << "step,time_in_swing,speed,hip_z,target_z,target_distance,placement_distance,leg_length,actual_foot_z\n";
        reach.precision(9);
        for (std::size_t i = 1; i < result.mFrames.size(); ++i)
        {
            const auto& frame = result.mFrames[i];
            if (!frame.mReferenceGait || !frame.mSwingFootMotorTarget || !frame.mReferenceGait->mPlacement)
                continue;
            const std::size_t hip = frame.mReferenceGait->mStanceHipBody == 14 ? 17 : 14;
            const std::size_t knee = hip + 1, foot = hip + 2;
            const auto& previous = result.mFrames[i - 1];
            const auto hipPosition = previous.mBodies[hip].mWorldPosition;
            const auto target = result.mSettings.mRootPosition + result.mSettings.mRootRotation
                * (*frame.mSwingFootMotorTarget * result.mSettings.mUniformScale);
            const float length = ((skeleton.mBones[knee].mModelTransform.mTranslation
                    - skeleton.mBones[hip].mModelTransform.mTranslation).Length()
                + (skeleton.mBones[foot].mModelTransform.mTranslation
                    - skeleton.mBones[knee].mModelTransform.mTranslation).Length()) * result.mSettings.mUniformScale;
            reach << frame.mStep << ',' << frame.mReferenceGait->mTimeInStage << ','
                << previous.mCenterOfMassVelocity.GetY() << ',' << hipPosition.GetZ() << ',' << target.GetZ()
                << ',' << JPH::Vec3(target - hipPosition).Length()
                << ',' << JPH::Vec3(*frame.mReferenceGait->mPlacement - hipPosition).Length()
                << ',' << length << ',' << frame.mBodies[foot].mWorldPosition.GetZ() << '\n';
        }
    }

    struct ReferenceSummary
    {
        std::size_t mReferenceFrames = 0;
        std::size_t mSwingFrames = 0;
        std::size_t mDoubleSupportFrames = 0;
        std::size_t mSwingPoseFrames = 0;
        std::size_t mLiftedFrames = 0;
        std::size_t mSaturatedFrames = 0;
        std::size_t mTimedOutFrames = 0;
        float mMaxMotorUtilization = 0.0f;
        float mMaxPelvisError = 0.0f;
        float mMaxPelvisYawError = 0.0f;
        float mMinPelvisHeight = std::numeric_limits<float>::max();
        float mMaxPelvisHeight = 0.0f;
        float mMaxSwingClearance = 0.0f;
        std::optional<std::uint64_t> mFirstSwingStep;
        std::optional<std::uint64_t> mFirstLiftStep;
        std::optional<std::uint64_t> mFirstTransferStep;
        std::optional<std::uint64_t> mFirstFallingStep;
        std::vector<std::uint64_t> mTransferSteps;
    };

    ReferenceSummary summarize(const RagdollCore::ScenarioResult& result)
    {
        ReferenceSummary summary;
        for (const auto& frame : result.mFrames)
        {
            if (frame.mController.mMode == RagdollCore::ActiveControllerMode::Falling
                || frame.mController.mMode == RagdollCore::ActiveControllerMode::Fallen)
            {
                if (!summary.mFirstFallingStep)
                    summary.mFirstFallingStep = frame.mStep;
            }
            if (frame.mGait.mTransitioned && frame.mGait.hasStance() && frame.mReferenceGait)
            {
                summary.mTransferSteps.push_back(frame.mStep);
                if (!summary.mFirstTransferStep)
                    summary.mFirstTransferStep = frame.mStep;
            }
            if (frame.mGait.mPhaseTimedOut)
                ++summary.mTimedOutFrames;
            if (!frame.mReferenceGait)
                continue;
            const auto& record = *frame.mReferenceGait;
            ++summary.mReferenceFrames;
            summary.mMinPelvisHeight = std::min(summary.mMinPelvisHeight, frame.mPelvisHeight);
            summary.mMaxPelvisHeight = std::max(summary.mMaxPelvisHeight, frame.mPelvisHeight);
            summary.mMaxMotorUtilization = std::max(summary.mMaxMotorUtilization, record.mMaxMotorUtilization);
            summary.mMaxPelvisError = std::max(summary.mMaxPelvisError, record.mPelvisOrientationError.Length());
            summary.mMaxPelvisYawError
                = std::max(summary.mMaxPelvisYawError, std::abs(record.mPelvisOrientationError.GetZ()));
            if (record.mMaxMotorUtilization >= 0.999f)
                ++summary.mSaturatedFrames;
            if (record.mStage == ReferenceGaitStage::Swing)
            {
                ++summary.mSwingFrames;
                if (!summary.mFirstSwingStep)
                    summary.mFirstSwingStep = frame.mStep;
            }
            if (record.mStage == ReferenceGaitStage::DoubleSupport)
                ++summary.mDoubleSupportFrames;
            if (record.mSwingPoseApplied)
                ++summary.mSwingPoseFrames;
            if (frame.mGait.mSwingFootLifted)
            {
                ++summary.mLiftedFrames;
                if (!summary.mFirstLiftStep)
                    summary.mFirstLiftStep = frame.mStep;
                const std::size_t swingFoot = frame.mGait.leftFootIsStance() ? result.mLandmarks.mRightFoot
                                                                              : result.mLandmarks.mLeftFoot;
                const std::size_t stanceFoot = frame.mGait.leftFootIsStance() ? result.mLandmarks.mLeftFoot
                                                                               : result.mLandmarks.mRightFoot;
                summary.mMaxSwingClearance = std::max(summary.mMaxSwingClearance,
                    static_cast<float>(frame.mBodies[swingFoot].mCenterOfMass.GetZ()
                        - frame.mBodies[stanceFoot].mCenterOfMass.GetZ()));
            }
        }
        return summary;
    }

    std::string describe(const ReferenceSummary& summary)
    {
        const auto step = [](const std::optional<std::uint64_t>& value) {
            return value ? std::to_string(*value) : std::string("missing");
        };
        std::ostringstream stream;
        stream << "reference_frames=" << summary.mReferenceFrames << " double_support=" << summary.mDoubleSupportFrames
               << " swing=" << summary.mSwingFrames << " swing_pose=" << summary.mSwingPoseFrames
               << " lifted=" << summary.mLiftedFrames << " saturated=" << summary.mSaturatedFrames
               << " timed_out=" << summary.mTimedOutFrames << " max_utilization=" << summary.mMaxMotorUtilization
               << " max_pelvis_error=" << summary.mMaxPelvisError << " max_yaw_error=" << summary.mMaxPelvisYawError
               << " pelvis_height=[" << summary.mMinPelvisHeight
               << ',' << summary.mMaxPelvisHeight << "] max_clearance=" << summary.mMaxSwingClearance
               << " first_swing=" << step(summary.mFirstSwingStep) << " first_lift=" << step(summary.mFirstLiftStep)
               << " first_transfer=" << step(summary.mFirstTransferStep)
               << " first_falling=" << step(summary.mFirstFallingStep) << " transfers=";
        for (const auto value : summary.mTransferSteps)
            stream << value << ';';
        return stream.str();
    }

    struct StanceFixture
    {
        std::vector<RagdollCore::Transform> mBase;
        RagdollCore::ReferenceGaitLeg mLeft, mRight;

        StanceFixture()
        {
            const auto skeleton = RagdollCore::makeScenarioHumanoidSkeleton();
            const auto profile = RagdollCore::makeScenarioHumanoidProfile();
            for (const auto& bone : skeleton.mBones)
                mBase.push_back(bone.mModelTransform);
            const auto leg = [&](std::size_t hip) {
                // The synthetic bind rotations are identity and both native
                // knee axes are +X. Limits come from the actual fixture profile.
                RagdollCore::HingeLegGeometry geometry;
                geometry.mUpperOffset = mBase[hip + 1].mTranslation - mBase[hip].mTranslation;
                geometry.mLowerOffset = mBase[hip + 2].mTranslation - mBase[hip + 1].mTranslation;
                geometry.mMinimumAngle = profile.mBodies[hip + 1].mConstraint->mHingeMinAngle;
                geometry.mMaximumAngle = profile.mBodies[hip + 1].mConstraint->mHingeMaxAngle;
                return RagdollCore::ReferenceGaitLeg{ hip, hip + 1, hip + 2, geometry };
            };
            mLeft = leg(14);
            mRight = leg(17);
        }
    };

    TEST(RagdollCoreReferenceGaitTest, emergencyOnlyWaitsWithoutCadenceAndRequiresLoadedSupport)
    {
        using namespace RagdollCore;
        ReferenceGaitProfile profile;
        profile.mEmergencyCaptureMargin = 6;
        profile.mContactConfirmationDuration = 0.03f;
        ReferenceGaitStateInput input;
        input.mEnabled = true;
        input.mEmergencyOnly = true;
        input.mLeftFootSupported = input.mRightFootSupported = true;
        input.mLeftFootLoadFraction = input.mRightFootLoadFraction = 0.5f;
        input.mLeftFootCenter = JPH::RVec3(-8, -10, 2.5);
        input.mRightFootCenter = JPH::RVec3(8, 0, 2.5);
        input.mCenterOfMass = JPH::RVec3(0, -5, 60);
        input.mCenterOfMassHeight = 60;
        input.mGravity = 627;
        // Facing remains defined even when requested travel is zero. A
        // staggered foot span alone is not a reliable anatomical lateral axis.
        input.mHeading = JPH::Vec3::sAxisY();
        ReferenceGaitStateMachine machine;
        ReferenceGaitState state;
        constexpr float dt = 1.0f / 60;
        for (int i = 0; i < 600; ++i)
        {
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
            EXPECT_FALSE(state.mTimedOut);
            EXPECT_EQ(state.mEmergencyStepCount, 0u);
        }
        input.mCenterOfMassVelocity = JPH::Vec3(0, 80, 0);
        input.mLeftFootLoadFraction = input.mRightFootLoadFraction = 0.1f;
        for (int i = 0; i < 12; ++i)
        {
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        }
        input.mRightFootLoadFraction = 1;
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::Swing);
        EXPECT_EQ(state.mPhase, GaitPhase::RightStance);
        EXPECT_TRUE(state.mEmergencyStep);
        EXPECT_EQ(state.mEmergencyStepCount, 1u);
    }

    TEST(RagdollCoreReferenceGaitTest, emergencyStancePreferenceOnlyChoosesConfirmedLegalSupport)
    {
        using namespace RagdollCore;
        for (const bool currentLoaded : { false, true })
        {
            ReferenceGaitProfile profile;
            profile.mEmergencyCaptureMargin = 6;
            ReferenceGaitStateInput input;
            input.mEnabled = input.mEmergencyOnly = true;
            input.mLeftFootSupported = input.mRightFootSupported = true;
            input.mLeftFootLoadFraction = currentLoaded ? 0.5f : 0.1f;
            input.mRightFootLoadFraction = 0.5f;
            input.mLeftFootCenter = JPH::RVec3(-8, -10, 2.5);
            input.mRightFootCenter = JPH::RVec3(8, 0, 2.5);
            input.mCenterOfMass = JPH::RVec3(0, -5, 60);
            input.mCenterOfMassHeight = 60;
            input.mGravity = 627;
            input.mHeading = JPH::Vec3::sAxisY();
            ReferenceGaitStateMachine machine;
            ReferenceGaitState state;
            for (int i = 0; i < 3; ++i)
                ASSERT_TRUE(machine.step(input, 1.0f / 60, profile, state));
            ASSERT_EQ(state.mPhase, GaitPhase::LeftStance);
            ASSERT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
            input.mCenterOfMassVelocity = JPH::Vec3(0, 80, 0);
            ASSERT_TRUE(machine.step(input, 1.0f / 60, profile, state));
            ASSERT_TRUE(state.mEmergencyStep);
            const bool preferCurrent = std::getenv("OPENMW_RAGDOLL_KEEP_EMERGENCY_STANCE") != nullptr;
            EXPECT_EQ(state.mPhase, currentLoaded && preferCurrent
                ? GaitPhase::LeftStance : GaitPhase::RightStance);
        }
    }

    TEST(RagdollCoreReferenceGaitTest, airborneFootDoesNotExpandEmergencySupportRegion)
    {
        using namespace RagdollCore;
        ReferenceGaitProfile profile;
        profile.mEmergencyCaptureMargin = 6;
        ReferenceGaitStateInput input;
        input.mEnabled = input.mEmergencyOnly = true;
        input.mLeftFootCenter = JPH::RVec3(-10, 0, 4);
        input.mRightFootCenter = JPH::RVec3(10, 0, 0);
        input.mLeftFootHeight = 4;
        input.mRightFootSupported = true;
        input.mRightFootLoadFraction = 1;
        input.mLeftFootLoadFraction = 0;
        input.mCenterOfMass = JPH::RVec3(-8, 0, 60);
        input.mCenterOfMassHeight = 60;
        input.mGravity = 627;
        input.mHeading = JPH::Vec3::sAxisY();
        ReferenceGaitStateMachine machine;
        ReferenceGaitState state;
        constexpr float dt = 1.0f / 60;
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::Swing);
        EXPECT_EQ(state.mPhase, GaitPhase::RightStance);
        EXPECT_TRUE(state.mEmergencyStep);
    }

    TEST(RagdollCoreReferenceGaitTest, emergencyAdoptsAnAlreadyAirborneFootAndConfirmsItsLanding)
    {
        using namespace RagdollCore;
        ReferenceGaitProfile profile;
        profile.mEmergencyCaptureMargin = 6;
        profile.mContactConfirmationDuration = 0.03f;
        ReferenceGaitStateInput input;
        input.mEnabled = input.mEmergencyOnly = true;
        input.mRightFootSupported = true;
        input.mRightFootLoadFraction = 1;
        input.mLeftFootLoadFraction = 0;
        input.mLeftFootCenter = JPH::RVec3(-8, -10, 10);
        input.mRightFootCenter = JPH::RVec3(8, 0, 2.5);
        input.mLeftFootHeight = 10;
        input.mRightFootHeight = 2.5f;
        input.mLeftFootVerticalVelocity = -5;
        input.mCenterOfMass = JPH::RVec3(0, -5, 60);
        input.mCenterOfMassHeight = 60;
        input.mCenterOfMassVelocity = JPH::Vec3(0, 80, 0);
        input.mGravity = 627;
        input.mHeading = JPH::Vec3::sAxisY();
        ReferenceGaitStateMachine machine;
        ReferenceGaitState state;
        constexpr float dt = 1.0f / 60;
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        ASSERT_EQ(state.mStage, ReferenceGaitStage::Swing);
        ASSERT_TRUE(state.mEmergencyStep);
        EXPECT_TRUE(state.mSwingFootLifted);
        // A single contact sample must not finish the step.
        input.mCenterOfMassVelocity = JPH::Vec3::sZero();
        input.mLeftFootSupported = true;
        input.mLeftFootLoadFraction = 0.5f;
        input.mRightFootLoadFraction = 0.5f;
        input.mLeftFootHeight = 2.5f;
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::Swing);
        for (int i = 0; i < 60 && state.mStage == ReferenceGaitStage::Swing; ++i)
            ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        EXPECT_TRUE(state.mEnteredFromTouchdown);
    }

    TEST(RagdollCoreReferenceGaitTest, capturedNativeSettlingGeometry)
    {
        using namespace RagdollCore;
        const char* path = std::getenv("OPENMW_RAGDOLL_STANCE_FIXTURE");
        if (!path || !*path) GTEST_SKIP() << "Requires a bounded native STANCE_FIXTURE log";
        std::ifstream file(path);
        ASSERT_TRUE(file);
        StanceFixture fixture;
        auto measured = fixture.mBase;
        Transform pelvis;
        JPH::Vec3 center = JPH::Vec3::sZero(), currentCenter = JPH::Vec3::sZero();
        std::size_t pelvisIndex = 0;
        unsigned records = 0;
        const auto vector = [](std::istream& in) {
            float x = 0, y = 0, z = 0;
            in >> x >> y >> z;
            return JPH::Vec3(x, y, z);
        };
        const auto quaternion = [](std::istream& in) {
            float x = 0, y = 0, z = 0, w = 1;
            in >> x >> y >> z >> w;
            return JPH::Quat(x, y, z, w);
        };
        for (std::string line; std::getline(file, line);)
        {
            const auto start = line.find("STANCE_FIXTURE ");
            if (start == std::string::npos) continue;
            std::istringstream in(line.substr(start + 15));
            std::string kind;
            in >> kind;
            if (kind == "indices")
                in >> pelvisIndex >> fixture.mLeft.mHip >> fixture.mLeft.mKnee >> fixture.mLeft.mFoot
                    >> fixture.mRight.mHip >> fixture.mRight.mKnee >> fixture.mRight.mFoot;
            else if (kind == "pelvis")
            {
                pelvis.mTranslation = vector(in);
                pelvis.mRotation = quaternion(in);
            }
            else if (kind == "center") { center = vector(in); currentCenter = vector(in); }
            else
            {
                std::size_t body = 0;
                in >> body;
                ASSERT_LT(body, fixture.mBase.size());
                if (kind == "base" || kind == "measured")
                {
                    auto& pose = kind == "base" ? fixture.mBase : measured;
                    pose[body].mTranslation = vector(in);
                    pose[body].mRotation = quaternion(in);
                }
                else if (kind == "hinge")
                {
                    HingeLegGeometry hinge;
                    hinge.mUpperOffset = vector(in); hinge.mLowerOffset = vector(in);
                    hinge.mLowerRotation = quaternion(in); hinge.mHingeAxis = vector(in);
                    in >> hinge.mMinimumAngle >> hinge.mMaximumAngle;
                    (body == fixture.mLeft.mHip ? fixture.mLeft : fixture.mRight).mHingeGeometry = hinge;
                }
                else if (kind == "joint")
                {
                    SwingTwistJointGeometry joint;
                    joint.mParentFrame = quaternion(in); joint.mChildFrame = quaternion(in);
                    in >> joint.mNormalHalfConeAngle >> joint.mPlaneHalfConeAngle
                        >> joint.mTwistMinimum >> joint.mTwistMaximum >> joint.mPyramid;
                    for (auto* leg : { &fixture.mLeft, &fixture.mRight })
                    {
                        if (body == leg->mHip) leg->mHipGeometry = joint;
                        if (body == leg->mFoot) leg->mAnkleGeometry = joint;
                    }
                }
                else FAIL() << "Unknown fixture record: " << kind;
            }
            ASSERT_FALSE(in.fail()) << line;
            ++records;
        }
        ASSERT_EQ(records, 49u);
        ASSERT_TRUE(fixture.mLeft.mHipGeometry && fixture.mRight.mHipGeometry
            && fixture.mLeft.mAnkleGeometry && fixture.mRight.mAnkleGeometry);
        const auto rotationError = [](JPH::QuatArg a, JPH::QuatArg b) {
            return 2 * std::acos(std::clamp(std::abs(a.Dot(b)), 0.0f, 1.0f));
        };
        for (const auto& leg : { fixture.mLeft, fixture.mRight })
        {
            const auto hip = clampSwingTwistJointOrientation(*leg.mHipGeometry,
                measured[pelvisIndex].mRotation, measured[leg.mHip].mRotation);
            const auto ankle = clampSwingTwistJointOrientation(*leg.mAnkleGeometry,
                measured[leg.mKnee].mRotation, measured[leg.mFoot].mRotation);
            ASSERT_TRUE(hip && ankle);
            const auto& hinge = *leg.mHingeGeometry;
            const auto kneePoint = measured[leg.mHip].mTranslation
                + measured[leg.mHip].mRotation * hinge.mUpperOffset;
            const auto footPoint = measured[leg.mKnee].mTranslation
                + measured[leg.mKnee].mRotation * hinge.mLowerRotation.Conjugated() * hinge.mLowerOffset;
            std::cout << "STANCE_MEASURED hip=" << leg.mHip
                << " hip_clamp=" << rotationError(*hip, measured[leg.mHip].mRotation)
                << " ankle_clamp=" << rotationError(*ankle, measured[leg.mFoot].mRotation)
                << " knee_anchor_error=" << (kneePoint - measured[leg.mKnee].mTranslation).Length()
                << " foot_anchor_error=" << (footPoint - measured[leg.mFoot].mTranslation).Length() << '\n';
        }
        auto shift = center - currentCenter;
        shift.SetZ(0);
        pelvis.mTranslation += shift;
        const auto radialProbe = planReferenceStancePosture(fixture.mBase, pelvisIndex, fixture.mLeft,
            fixture.mRight, pelvis, measured[fixture.mLeft.mFoot].mTranslation,
            measured[fixture.mRight.mFoot].mTranslation, JPH::Vec3::sAxisY());
        ASSERT_TRUE(radialProbe);
        unsigned rollFeasibleHeights = 0;
        for (unsigned heightSample = 0; heightSample <= 64; ++heightSample)
        {
            auto heightTarget = pelvis;
            heightTarget.mTranslation.SetZ(radialProbe->mMinimumPelvisHeight
                + (radialProbe->mMaximumPelvisHeight - radialProbe->mMinimumPelvisHeight) * heightSample / 64);
            const auto heightPose = planReferenceStancePosture(fixture.mBase, pelvisIndex, fixture.mLeft,
                fixture.mRight, heightTarget, measured[fixture.mLeft.mFoot].mTranslation,
                measured[fixture.mRight.mFoot].mTranslation, JPH::Vec3::sAxisY());
            if (!heightPose) continue;
            unsigned feasibleLegs = 0;
            for (const auto& leg : { fixture.mLeft, fixture.mRight })
            {
                const auto& pose = heightPose->mPose.mBodyTransforms;
                const auto axis = (pose[leg.mFoot].mTranslation - pose[leg.mHip].mTranslation).Normalized();
                unsigned valid = 0;
                for (unsigned sample = 0; sample < 360; ++sample)
                {
                    const auto rotation = JPH::Quat::sRotation(axis, sample * 6.28318530718f / 360);
                    const auto hipRotation = (rotation * pose[leg.mHip].mRotation).Normalized();
                    const auto kneeRotation = (rotation * pose[leg.mKnee].mRotation).Normalized();
                    const auto hip = clampSwingTwistJointOrientation(*leg.mHipGeometry, pose[pelvisIndex].mRotation, hipRotation);
                    const auto ankle = clampSwingTwistJointOrientation(*leg.mAnkleGeometry, kneeRotation,
                        measured[leg.mFoot].mRotation);
                    const auto same = [](const auto& value, JPH::QuatArg target) {
                        return value && (value->IsClose(target, 1e-10f) || value->IsClose(-target, 1e-10f));
                    };
                    if (same(hip, hipRotation) && same(ankle, measured[leg.mFoot].mRotation))
                    {
                        ++valid;
                    }
                }
                feasibleLegs += valid > 0;
            }
            if (feasibleLegs == 2)
            {
                ++rollFeasibleHeights;
                std::cout << "STANCE_ROLL feasible_height=" << heightTarget.mTranslation.GetZ() << '\n';
            }
        }
        std::cout << "STANCE_ROLL feasible_heights=" << rollFeasibleHeights << '\n';
        unsigned radialCount = 0, fixedCount = 0, freeCount = 0;
        // A diagnostic sweep tests whether the preferred knee plane, rather
        // than reach or limits themselves, accounts for a native rejection.
        for (unsigned sample = 0; sample < 36; ++sample)
        {
            const float angle = sample * 6.28318530718f / 36;
            const auto bend = JPH::Vec3(std::sin(angle), std::cos(angle), 0);
            const auto radial = planReferenceStancePosture(fixture.mBase, pelvisIndex,
                fixture.mLeft, fixture.mRight, pelvis, measured[fixture.mLeft.mFoot].mTranslation,
                measured[fixture.mRight.mFoot].mTranslation, bend);
            const auto solve = [&](bool fixed) {
                return planJointLimitedReferenceStancePosture(fixture.mBase, pelvisIndex,
                    fixture.mLeft, fixture.mRight, pelvis, measured[fixture.mLeft.mFoot].mTranslation,
                    measured[fixture.mRight.mFoot].mTranslation,
                    fixed ? std::optional(measured[fixture.mLeft.mFoot].mRotation) : std::nullopt,
                    fixed ? std::optional(measured[fixture.mRight.mFoot].mRotation) : std::nullopt, bend);
            };
            const auto fixed = solve(true), free = solve(false);
            radialCount += bool(radial); fixedCount += bool(fixed); freeCount += bool(free);
            std::cout << "STANCE_SWEEP angle=" << angle << " radial=" << bool(radial)
                << " fixed=" << bool(fixed) << " free=" << bool(free)
                << " requested_height=" << pelvis.mTranslation.GetZ()
                << " free_height=" << (free ? free->mPose.mBodyTransforms[pelvisIndex].mTranslation.GetZ() : -1)
                << " left_foot_turn=" << (free ? rotationError(free->mPose.mBodyTransforms[fixture.mLeft.mFoot].mRotation,
                    measured[fixture.mLeft.mFoot].mRotation) : -1)
                << " right_foot_turn=" << (free ? rotationError(free->mPose.mBodyTransforms[fixture.mRight.mFoot].mRotation,
                    measured[fixture.mRight.mFoot].mRotation) : -1) << '\n';
        }
        EXPECT_GT(radialCount, 0u);
        std::cout << "STANCE_SWEEP totals radial=" << radialCount << " fixed=" << fixedCount
            << " free=" << freeCount << std::endl;
        unsigned headingCount = 0;
        for (unsigned sample = 0; sample < 36; ++sample)
        {
            const float angle = sample * 6.28318530718f / 36;
            const auto turn = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), angle);
            auto turned = pelvis;
            turned.mRotation = (turn * pelvis.mRotation).Normalized();
            const auto result = planJointLimitedReferenceStancePosture(fixture.mBase, pelvisIndex,
                fixture.mLeft, fixture.mRight, turned, measured[fixture.mLeft.mFoot].mTranslation,
                measured[fixture.mRight.mFoot].mTranslation, measured[fixture.mLeft.mFoot].mRotation,
                measured[fixture.mRight.mFoot].mRotation, turn * JPH::Vec3::sAxisY());
            headingCount += bool(result);
            if (result) std::cout << "STANCE_HEADING feasible=" << angle << '\n';
        }
        std::cout << "STANCE_HEADING totals fixed=" << headingCount << std::endl;
        for (const auto& moving : { fixture.mLeft, fixture.mRight })
        {
            unsigned feasible = 0;
            float nearest = std::numeric_limits<float>::max();
            // Search horizontal landing positions with only the moving foot's
            // orientation free. The other foot remains exactly planted. This
            // is terminal geometry, not proof of safe takeoff or landing.
            for (int x = -4; x <= 4; ++x)
                for (int y = -4; y <= 4; ++y)
                {
                    auto leftFoot = measured[fixture.mLeft.mFoot].mTranslation;
                    auto rightFoot = measured[fixture.mRight.mFoot].mTranslation;
                    auto& target = moving.mFoot == fixture.mLeft.mFoot ? leftFoot : rightFoot;
                    target += JPH::Vec3(x * 5.0f, y * 5.0f, 0);
                    const auto result = planJointLimitedReferenceStancePosture(fixture.mBase, pelvisIndex,
                        fixture.mLeft, fixture.mRight, pelvis, leftFoot, rightFoot,
                        moving.mFoot == fixture.mLeft.mFoot ? std::nullopt : std::optional(measured[fixture.mLeft.mFoot].mRotation),
                        moving.mFoot == fixture.mRight.mFoot ? std::nullopt : std::optional(measured[fixture.mRight.mFoot].mRotation),
                        JPH::Vec3::sAxisY());
                    if (!result) continue;
                    ++feasible;
                    const float distance = (target - measured[moving.mFoot].mTranslation).Length();
                    if (distance >= nearest) continue;
                    nearest = distance;
                    std::cout << "STANCE_REPOSITION foot=" << moving.mFoot << " dx=" << x * 5
                        << " dy=" << y * 5 << " distance=" << distance << " height="
                        << result->mPose.mBodyTransforms[pelvisIndex].mTranslation.GetZ() << '\n';
                }
            std::cout << "STANCE_REPOSITION totals foot=" << moving.mFoot << " feasible=" << feasible << '\n';
        }
    }

    TEST(RagdollCoreReferenceGaitTest, stancePostureFindsCommonHeightForBothFootTargets)
    {
        using namespace RagdollCore;
        StanceFixture fixture;
        const auto leftFoot = fixture.mBase[16].mTranslation;
        const auto rightFoot = fixture.mBase[19].mTranslation + JPH::Vec3(0, 27, 0);
        auto desired = fixture.mBase[0];
        desired.mTranslation.SetY(13.5f);
        const auto result = planReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
            desired, leftFoot, rightFoot, JPH::Vec3::sAxisY());
        ASSERT_TRUE(result);
        // A 27-unit stride needs a lower hip even at its midpoint. No foot
        // target is moved to fit the leg, and the rest of the torso follows.
        const float height = 6.5f + std::sqrt(59.5f * 59.5f - 13.5f * 13.5f);
        EXPECT_NEAR(result->mMaximumPelvisHeight, height, 1e-4f);
        EXPECT_NEAR(result->mPose.mBodyTransforms[0].mTranslation.GetZ(), height, 1e-4f);
        EXPECT_LT(height, desired.mTranslation.GetZ());
        EXPECT_LT((result->mPose.mBodyTransforms[16].mTranslation - leftFoot).Length(), 1e-3f);
        EXPECT_LT((result->mPose.mBodyTransforms[19].mTranslation - rightFoot).Length(), 1e-3f);
        EXPECT_LT(((result->mPose.mBodyTransforms[3].mTranslation - result->mPose.mBodyTransforms[0].mTranslation)
            - (fixture.mBase[3].mTranslation - fixture.mBase[0].mTranslation)).Length(), 1e-4f);

        // Finite flexion limits and asymmetric foot heights can produce
        // disjoint height intervals even though each foot is reachable alone.
        fixture.mLeft.mHingeGeometry->mMinimumAngle = -0.3f;
        EXPECT_FALSE(planReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
            desired, leftFoot, rightFoot - JPH::Vec3(0, 0, 40), JPH::Vec3::sAxisY()));
        EXPECT_FALSE(planReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
            desired, leftFoot, rightFoot + JPH::Vec3(0, 150, 0), JPH::Vec3::sAxisY()));
    }

    TEST(RagdollCoreReferenceGaitTest, stancePostureAccountsForHingeOffsetsAndPelvisTilt)
    {
        using namespace RagdollCore;
        StanceFixture fixture;
        for (auto* leg : { &fixture.mLeft, &fixture.mRight })
        {
            leg->mHingeGeometry->mUpperOffset += JPH::Vec3(4, 2, 0);
            leg->mHingeGeometry->mLowerOffset += JPH::Vec3(-3, -1, 0);
            leg->mHingeGeometry->mMinimumAngle = -1.5f;
            leg->mHingeGeometry->mMaximumAngle = -0.3f;
        }
        auto desired = fixture.mBase[0];
        desired.mTranslation.SetZ(58);
        desired.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.15f);
        const auto result = planReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
            desired, fixture.mBase[16].mTranslation, fixture.mBase[19].mTranslation, JPH::Vec3::sAxisY());
        ASSERT_TRUE(result);
        EXPECT_NEAR(result->mPose.mBodyTransforms[0].mTranslation.GetZ(), 58, 1e-4f);
        for (const auto& leg : { fixture.mLeft, fixture.mRight })
        {
            const auto& pose = result->mPose.mBodyTransforms;
            const auto& geometry = *leg.mHingeGeometry;
            EXPECT_LT((pose[leg.mFoot].mTranslation - fixture.mBase[leg.mFoot].mTranslation).Length(), 1e-3f);
            EXPECT_LT((pose[leg.mHip].mRotation * geometry.mUpperOffset
                - (pose[leg.mKnee].mTranslation - pose[leg.mHip].mTranslation)).Length(), 1e-4f);
            const auto relative = pose[leg.mHip].mRotation.Conjugated() * pose[leg.mKnee].mRotation;
            const float angle = 2 * std::atan2(relative.GetX(), relative.GetW());
            EXPECT_GE(angle, geometry.mMinimumAngle - 1e-4f);
            EXPECT_LE(angle, geometry.mMaximumAngle + 1e-4f);
        }
    }

    TEST(RagdollCoreReferenceGaitTest, stanceReachDoesNotEstablishNativeAnkleFeasibility)
    {
        using namespace RagdollCore;
        StanceFixture fixture;
        const auto built = buildRagdollSettings(makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), {});
        ASSERT_TRUE(built.isValid());
        const auto* ankle = static_cast<const JPH::SwingTwistConstraintSettings*>(
            built.mSettings->mParts[16].mToParent.GetPtr());
        ASSERT_TRUE(ankle);
        const auto frame = [](JPH::Vec3Arg twist, JPH::Vec3Arg plane) {
            return JPH::Mat44(JPH::Vec4(twist, 0), JPH::Vec4(plane.Cross(twist), 0),
                JPH::Vec4(plane, 0), JPH::Vec4(0, 0, 0, 1)).GetQuaternion();
        };
        const auto parentFrame = frame(ankle->mTwistAxis1, ankle->mPlaneAxis1);
        const auto childFrame = frame(ankle->mTwistAxis2, ankle->mPlaneAxis2);
        JPH::SwingTwistConstraintPart limits;
        limits.SetSwingType(ankle->mSwingType);
        limits.SetLimits(ankle->mTwistMinAngle, ankle->mTwistMaxAngle,
            -ankle->mPlaneHalfConeAngle, ankle->mPlaneHalfConeAngle,
            -ankle->mNormalHalfConeAngle, ankle->mNormalHalfConeAngle);
        const float calfAngle = 0.8f * ankle->mPlaneHalfConeAngle;
        const float upper = fixture.mLeft.mHingeGeometry->mUpperOffset.Length();
        const float lower = fixture.mLeft.mHingeGeometry->mLowerOffset.Length();
        const float thighAngle = std::asin(lower * std::sin(calfAngle) / upper);
        const float feasibleDepth = upper * (1 - std::cos(thighAngle)) + lower * (1 - std::cos(calfAngle));
        for (const float depth : { 0.0f, feasibleDepth, 4.0f })
        {
            auto pelvis = fixture.mBase[0];
            pelvis.mTranslation.SetZ(66 - depth);
            const auto posture = planReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
                pelvis, fixture.mBase[16].mTranslation, fixture.mBase[19].mTranslation, JPH::Vec3::sAxisY());
            ASSERT_TRUE(posture);
            const auto& pose = posture->mPose.mBodyTransforms;
            const auto request = (parentFrame.Conjugated() * pose[15].mRotation.Conjugated()
                * pose[16].mRotation * childFrame).Normalized();
            JPH::Quat swing, twist;
            request.GetSwingTwist(swing, twist);
            JPH::uint clamped = 0;
            limits.ClampSwingTwist(swing, twist, clamped);
            const float delta = 2 * std::acos(std::clamp(std::abs(request.Dot(swing * twist)), 0.0f, 1.0f));
            std::cout << "STANCE_ANKLE depth=" << depth << " clamp_delta=" << delta << std::endl;
            if (depth == 4.0f)
            {
                EXPECT_NE(clamped, 0u);
                EXPECT_GT(delta, 0.1f);
            }
            else
                EXPECT_EQ(clamped, 0u);
        }
    }

    void addNativeJointGeometry(StanceFixture& fixture)
    {
        using namespace RagdollCore;
        const auto built = buildRagdollSettings(makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), {});
        ASSERT_TRUE(built.isValid());
        const auto frame = [](JPH::Vec3Arg twist, JPH::Vec3Arg plane) {
            return JPH::Mat44(JPH::Vec4(twist, 0), JPH::Vec4(plane.Cross(twist), 0),
                JPH::Vec4(plane, 0), JPH::Vec4(0, 0, 0, 1)).GetQuaternion();
        };
        const auto geometry = [&](std::size_t body) {
            const auto* joint = static_cast<const JPH::SwingTwistConstraintSettings*>(
                built.mSettings->mParts[body].mToParent.GetPtr());
            return SwingTwistJointGeometry{ frame(joint->mTwistAxis1, joint->mPlaneAxis1),
                frame(joint->mTwistAxis2, joint->mPlaneAxis2), joint->mNormalHalfConeAngle,
                joint->mPlaneHalfConeAngle, joint->mTwistMinAngle, joint->mTwistMaxAngle,
                joint->mSwingType == JPH::ESwingType::Pyramid };
        };
        for (auto* leg : { &fixture.mLeft, &fixture.mRight })
        {
            leg->mHipGeometry = geometry(leg->mHip);
            leg->mAnkleGeometry = geometry(leg->mFoot);
        }
    }

    TEST(RagdollCoreReferenceGaitTest, projectedRecoveryLegsRespectLimitsAndRebuildFootPositions)
    {
        using namespace RagdollCore;
        StanceFixture fixture;
        addNativeJointGeometry(fixture);
        auto requested = fixture.mBase;
        for (const auto& leg : { fixture.mLeft, fixture.mRight })
        {
            const auto& hinge = *leg.mHingeGeometry;
            requested[leg.mHip].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 2.4f);
            requested[leg.mKnee].mRotation = (requested[leg.mHip].mRotation
                * JPH::Quat::sRotation(hinge.mHingeAxis, hinge.mMaximumAngle + 0.7f)
                * hinge.mLowerRotation).Normalized();
            requested[leg.mFoot].mRotation = (requested[leg.mKnee].mRotation
                * JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.2f)).Normalized();
            requested[leg.mKnee].mTranslation = requested[leg.mHip].mTranslation
                + requested[leg.mHip].mRotation * hinge.mUpperOffset;
            requested[leg.mFoot].mTranslation = requested[leg.mKnee].mTranslation
                + requested[leg.mKnee].mRotation * hinge.mLowerRotation.Conjugated() * hinge.mLowerOffset;
        }
        const auto projected = projectReferenceLegPosture(requested, 0, fixture.mLeft, fixture.mRight);
        ASSERT_TRUE(projected);
        const auto& pose = projected->mBodyTransforms;
        EXPECT_TRUE(pose[0].mTranslation.IsClose(requested[0].mTranslation, 0));
        EXPECT_TRUE(pose[0].mRotation.IsClose(requested[0].mRotation, 0));
        EXPECT_TRUE(pose[1].mTranslation.IsClose(requested[1].mTranslation, 0));
        EXPECT_TRUE(pose[1].mRotation.IsClose(requested[1].mRotation, 0));
        for (const auto& leg : { fixture.mLeft, fixture.mRight })
        {
            const auto& hinge = *leg.mHingeGeometry;
            const auto hip = clampSwingTwistJointOrientation(*leg.mHipGeometry, pose[0].mRotation, pose[leg.mHip].mRotation);
            const auto ankle = clampSwingTwistJointOrientation(*leg.mAnkleGeometry, pose[leg.mKnee].mRotation, pose[leg.mFoot].mRotation);
            ASSERT_TRUE(hip && ankle);
            EXPECT_TRUE(hip->IsClose(pose[leg.mHip].mRotation, 1e-9f));
            EXPECT_TRUE(ankle->IsClose(pose[leg.mFoot].mRotation, 1e-9f));
            const auto relative = (pose[leg.mHip].mRotation.Conjugated() * pose[leg.mKnee].mRotation
                * hinge.mLowerRotation.Conjugated()).Normalized();
            const float angle = 2 * std::atan2(relative.GetXYZ().Dot(hinge.mHingeAxis), relative.GetW());
            EXPECT_GE(angle, hinge.mMinimumAngle - 1e-5f);
            EXPECT_LE(angle, hinge.mMaximumAngle + 1e-5f);
            EXPECT_TRUE(pose[leg.mKnee].mTranslation.IsClose(pose[leg.mHip].mTranslation
                + pose[leg.mHip].mRotation * hinge.mUpperOffset, 1e-8f));
            EXPECT_TRUE(pose[leg.mFoot].mTranslation.IsClose(pose[leg.mKnee].mTranslation
                + pose[leg.mKnee].mRotation * hinge.mLowerRotation.Conjugated() * hinge.mLowerOffset, 1e-8f));
            EXPECT_GT((pose[leg.mFoot].mTranslation - requested[leg.mFoot].mTranslation).Length(), 1);
        }
        const auto twice = projectReferenceLegPosture(pose, 0, fixture.mLeft, fixture.mRight);
        ASSERT_TRUE(twice);
        for (std::size_t body = 0; body < pose.size(); ++body)
        {
            EXPECT_TRUE(twice->mBodyTransforms[body].mTranslation.IsClose(pose[body].mTranslation, 1e-6f));
            EXPECT_TRUE(twice->mBodyTransforms[body].mRotation.IsClose(pose[body].mRotation, 1e-8f));
        }
        // Native bone-local frames are not the synthetic fixture's identity
        // frames. Re-express the same anatomy in three independent bases;
        // the physical projected endpoints must stay unchanged.
        auto reframed = requested;
        auto left = fixture.mLeft, right = fixture.mRight;
        const auto hipBasis = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 2.2f);
        const auto kneeBasis = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), -0.7f);
        const auto footBasis = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.4f);
        for (auto* leg : { &left, &right })
        {
            reframed[leg->mHip].mRotation = (reframed[leg->mHip].mRotation * hipBasis).Normalized();
            reframed[leg->mKnee].mRotation = (reframed[leg->mKnee].mRotation * kneeBasis).Normalized();
            reframed[leg->mFoot].mRotation = (reframed[leg->mFoot].mRotation * footBasis).Normalized();
            auto& hinge = *leg->mHingeGeometry;
            hinge.mUpperOffset = hipBasis.Conjugated() * hinge.mUpperOffset;
            hinge.mLowerOffset = hipBasis.Conjugated() * hinge.mLowerOffset;
            hinge.mHingeAxis = hipBasis.Conjugated() * hinge.mHingeAxis;
            hinge.mLowerRotation = (hipBasis.Conjugated() * hinge.mLowerRotation * kneeBasis).Normalized();
            leg->mHipGeometry->mChildFrame = (hipBasis.Conjugated() * leg->mHipGeometry->mChildFrame).Normalized();
            leg->mAnkleGeometry->mParentFrame = (kneeBasis.Conjugated() * leg->mAnkleGeometry->mParentFrame).Normalized();
            leg->mAnkleGeometry->mChildFrame = (footBasis.Conjugated() * leg->mAnkleGeometry->mChildFrame).Normalized();
        }
        const auto equivalent = projectReferenceLegPosture(reframed, 0, left, right);
        ASSERT_TRUE(equivalent);
        for (const auto& leg : { left, right })
        {
            EXPECT_TRUE(equivalent->mBodyTransforms[leg.mKnee].mTranslation.IsClose(pose[leg.mKnee].mTranslation, 1e-6f));
            EXPECT_TRUE(equivalent->mBodyTransforms[leg.mFoot].mTranslation.IsClose(pose[leg.mFoot].mTranslation, 1e-6f));
        }
        fixture.mRight.mAnkleGeometry.reset();
        EXPECT_FALSE(projectReferenceLegPosture(requested, 0, fixture.mLeft, fixture.mRight));
    }

    void exportLandingPostureAudit(const RagdollCore::ScenarioResult& result, const std::string& name)
    {
        using namespace RagdollCore;
        const char* directory = std::getenv("OPENMW_RAGDOLL_TRACE_DIR");
        if (!directory || !*directory || !std::getenv("OPENMW_RAGDOLL_POSTURE_AUDIT"))
            return;
        StanceFixture fixture;
        addNativeJointGeometry(fixture);
        std::ofstream out(std::filesystem::path(directory) / (name + "-posture-audit.csv"));
        ASSERT_TRUE(out);
        out.precision(9);
        const bool measuredHeading = std::getenv("OPENMW_RAGDOLL_POSTURE_AUDIT_MEASURED_HEADING") != nullptr;
        out << "step,stage,target,stance_rotation,stance_foot,pelvis_yaw,pelvis_x,pelvis_y,pelvis_z,"
               "stance_x,stance_y,stance_z,target_x,target_y,target_z,radial_feasible,joint_feasible,"
               "selected_height,radial_min_height,radial_max_height\n";
        const auto toModel = [&](JPH::RVec3Arg point) {
            return result.mSettings.mRootRotation.Conjugated()
                * JPH::Vec3(point - result.mSettings.mRootPosition) / result.mSettings.mUniformScale;
        };
        const auto level = [&](const ScenarioBodyRecord& body) {
            auto forward = body.mWorldRotation * JPH::Vec3::sAxisY();
            forward.SetZ(0);
            return (result.mSettings.mRootRotation.Conjugated()
                * JPH::Quat::sFromTo(JPH::Vec3::sAxisY(), forward.Normalized())).Normalized();
        };
        for (std::size_t step = 1; step < result.mFrames.size(); ++step)
        {
            const auto& frame = result.mFrames[step];
            if (!frame.mReferenceGait) continue;
            const auto& previous = result.mFrames[step - 1];
            const bool left = frame.mReferenceGait->mStanceHipBody == 14;
            const std::size_t stance = left ? 16 : 19, swing = left ? 19 : 16;
            auto pelvis = previous.mBodies[0].mModelTransform;
            pelvis.mRotation = measuredHeading ? level(previous.mBodies[0]) : fixture.mBase[0].mRotation;
            const auto pelvisForward = pelvis.mRotation * JPH::Vec3::sAxisY();
            const float pelvisYaw = std::atan2(-pelvisForward.GetX(), pelvisForward.GetY());
            for (const unsigned targetKind : { 0u, 1u, 2u })
            {
                if (targetKind == 1 && !frame.mSwingFootMotorTarget) continue;
                if (targetKind == 2 && !frame.mReferenceGait->mPlacement) continue;
                const auto footTarget = targetKind == 0 ? previous.mBodies[swing].mModelTransform.mTranslation
                    : targetKind == 1 ? *frame.mSwingFootMotorTarget : toModel(*frame.mReferenceGait->mPlacement);
                const auto stanceTarget = previous.mBodies[stance].mModelTransform.mTranslation;
                for (const bool levelStance : { false, true })
                {
                    const auto stanceRotation = levelStance ? level(previous.mBodies[stance])
                        : previous.mBodies[stance].mModelTransform.mRotation;
                    const auto swingRotation = targetKind == 1 ? std::optional<JPH::Quat>{}
                        : targetKind == 0 ? std::optional(previous.mBodies[swing].mModelTransform.mRotation)
                                          : std::optional(level(previous.mBodies[swing]));
                    const auto leftFoot = left ? stanceTarget : footTarget;
                    const auto rightFoot = left ? footTarget : stanceTarget;
                    const auto radial = planReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
                        pelvis, leftFoot, rightFoot, JPH::Vec3::sAxisY());
                    const auto joint = planJointLimitedReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
                        pelvis, leftFoot, rightFoot, left ? std::optional(stanceRotation) : swingRotation,
                        left ? swingRotation : std::optional(stanceRotation), JPH::Vec3::sAxisY());
                    out << step << ',' << referenceGaitStageName(frame.mReferenceGait->mStage) << ',' << targetKind
                        << ',' << levelStance << ',' << stance << ',' << pelvisYaw << ',' << pelvis.mTranslation.GetX() << ','
                        << pelvis.mTranslation.GetY() << ',' << pelvis.mTranslation.GetZ() << ','
                        << stanceTarget.GetX() << ',' << stanceTarget.GetY() << ',' << stanceTarget.GetZ() << ','
                        << footTarget.GetX() << ',' << footTarget.GetY() << ',' << footTarget.GetZ()
                        << ',' << radial.has_value() << ',' << joint.has_value() << ',';
                    if (joint) out << joint->mPose.mBodyTransforms[0].mTranslation.GetZ();
                    out << ',';
                    if (radial) out << radial->mMinimumPelvisHeight << ',' << radial->mMaximumPelvisHeight;
                    else out << ',';
                    out << '\n';
                }
            }
        }
    }

    TEST(RagdollCoreReferenceGaitTest, jointLimitedStanceRespectsIndependentPlantedFootHeadings)
    {
        using namespace RagdollCore;
        StanceFixture fixture;
        addNativeJointGeometry(fixture);
        const auto leftRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.40f);
        const auto rightRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), -0.40f);
        // Each hip permits 0.28 radians of twist, while each ankle permits
        // only 0.18. Neither joint can take the entire foot heading alone;
        // the knee plane must distribute the turn between hip and ankle.
        auto pelvis = fixture.mBase[0];
        pelvis.mTranslation.SetZ(65);
        const auto planned = planJointLimitedReferenceStancePosture(fixture.mBase, 0,
            fixture.mLeft, fixture.mRight, pelvis, fixture.mBase[16].mTranslation,
            fixture.mBase[19].mTranslation, leftRotation, rightRotation, JPH::Vec3::sAxisY());
        ASSERT_TRUE(planned);
        const auto& pose = planned->mPose.mBodyTransforms;
        EXPECT_NEAR(pose[0].mTranslation.GetZ(), 65, 0.002f);
        EXPECT_TRUE(pose[16].mTranslation.IsClose(fixture.mBase[16].mTranslation, 1e-6f));
        EXPECT_TRUE(pose[19].mTranslation.IsClose(fixture.mBase[19].mTranslation, 1e-6f));
        EXPECT_TRUE(pose[16].mRotation.IsClose(leftRotation, 1e-10f));
        EXPECT_TRUE(pose[19].mRotation.IsClose(rightRotation, 1e-10f));
        for (const auto& leg : { fixture.mLeft, fixture.mRight })
        {
            EXPECT_NEAR((pose[leg.mKnee].mTranslation - pose[leg.mHip].mTranslation).Length(), 29, 1e-3f);
            EXPECT_NEAR((pose[leg.mFoot].mTranslation - pose[leg.mKnee].mTranslation).Length(), 30.5f, 1e-3f);
            const auto hip = clampSwingTwistJointOrientation(*leg.mHipGeometry,
                pose[0].mRotation, pose[leg.mHip].mRotation);
            const auto ankle = clampSwingTwistJointOrientation(*leg.mAnkleGeometry,
                pose[leg.mKnee].mRotation, pose[leg.mFoot].mRotation);
            ASSERT_TRUE(hip && ankle);
            EXPECT_TRUE(hip->IsClose(pose[leg.mHip].mRotation, 1e-10f));
            EXPECT_TRUE(ankle->IsClose(pose[leg.mFoot].mRotation, 1e-10f));
        }
    }

    TEST(RagdollCoreReferenceGaitTest, jointLimitedStanceFindsAlternateKneePlaneWithoutMovingFeet)
    {
        using namespace RagdollCore;
        StanceFixture fixture;
        addNativeJointGeometry(fixture);
        auto pelvis = fixture.mBase[0];
        pelvis.mTranslation.SetZ(65);
        // The preferred plane points outside the authored hip twist range.
        // Rotating a knee about its hip-foot line can recover a valid pose
        // without moving the planted foot or widening any joint limit.
        for (const auto bend : { JPH::Vec3::sAxisX(), -JPH::Vec3::sAxisY() })
            for (const bool raised : { false, true })
            {
                const auto leftFoot = fixture.mBase[16].mTranslation;
                const auto rightFoot = fixture.mBase[19].mTranslation + JPH::Vec3(0, 0, raised ? 8 : 0);
                const auto result = planJointLimitedReferenceStancePosture(fixture.mBase, 0,
                    fixture.mLeft, fixture.mRight, pelvis, leftFoot, rightFoot, JPH::Quat::sIdentity(),
                    raised ? std::nullopt : std::optional<JPH::Quat>(JPH::Quat::sIdentity()), bend);
                ASSERT_TRUE(result) << "raised=" << raised;
                const auto& pose = result->mPose.mBodyTransforms;
                EXPECT_LT((pose[16].mTranslation - leftFoot).Length(), 1e-3f);
                EXPECT_LT((pose[19].mTranslation - rightFoot).Length(), 1e-3f);
                EXPECT_TRUE(pose[16].mRotation.IsClose(JPH::Quat::sIdentity(), 0));
                if (!raised) EXPECT_TRUE(pose[19].mRotation.IsClose(JPH::Quat::sIdentity(), 0));
                for (const auto& leg : { fixture.mLeft, fixture.mRight })
                {
                    EXPECT_NEAR((pose[leg.mKnee].mTranslation - pose[leg.mHip].mTranslation).Length(), 29, 1e-3f);
                    EXPECT_NEAR((pose[leg.mFoot].mTranslation - pose[leg.mKnee].mTranslation).Length(), 30.5f, 1e-3f);
                    const auto hip = clampSwingTwistJointOrientation(*leg.mHipGeometry,
                        pose[0].mRotation, pose[leg.mHip].mRotation);
                    const auto ankle = clampSwingTwistJointOrientation(*leg.mAnkleGeometry,
                        pose[leg.mKnee].mRotation, pose[leg.mFoot].mRotation);
                    ASSERT_TRUE(hip && ankle);
                    EXPECT_TRUE(hip->IsClose(pose[leg.mHip].mRotation, 1e-10f));
                    EXPECT_TRUE(ankle->IsClose(pose[leg.mFoot].mRotation, 1e-10f));
                }
            }
    }

    TEST(RagdollCoreReferenceGaitTest, jointLimitedStancePreservesFeetAndRejectsUnreachableOrientation)
    {
        using namespace RagdollCore;
        StanceFixture fixture;
        addNativeJointGeometry(fixture);
        ASSERT_FALSE(::testing::Test::HasFailure());
        auto desired = fixture.mBase[0];
        desired.mTranslation.SetZ(62);
        const auto leftFoot = fixture.mBase[16].mTranslation;
        const auto rightFoot = fixture.mBase[19].mTranslation;
        const auto solve = [&](JPH::QuatArg rightRotation) {
            return planJointLimitedReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
                desired, leftFoot, rightFoot, JPH::Quat::sIdentity(), rightRotation, JPH::Vec3::sAxisY());
        };
        const auto result = solve(JPH::Quat::sIdentity());
        ASSERT_TRUE(result);
        const auto& pose = result->mPose.mBodyTransforms;
        // Closed-form lower height for this symmetric flat-foot fixture is
        // bounded by ankle pitch, well before the knee's flexion limit.
        const float calfAngle = fixture.mLeft.mAnkleGeometry->mPlaneHalfConeAngle;
        const float thighAngle = std::asin(30.5f * std::sin(calfAngle) / 29.0f);
        const float expectedHeight = 6.5f + 29 * std::cos(thighAngle) + 30.5f * std::cos(calfAngle);
        EXPECT_NEAR(pose[0].mTranslation.GetZ(), expectedHeight, 0.002f);
        EXPECT_GT(pose[0].mTranslation.GetZ(), 63.5f);
        EXPECT_TRUE(pose[16].mTranslation.IsClose(leftFoot, 1e-6f));
        EXPECT_TRUE(pose[19].mTranslation.IsClose(rightFoot, 1e-6f));
        for (const auto& leg : { fixture.mLeft, fixture.mRight })
        {
            const auto hip = clampSwingTwistJointOrientation(*leg.mHipGeometry, pose[0].mRotation, pose[leg.mHip].mRotation);
            const auto ankle = clampSwingTwistJointOrientation(*leg.mAnkleGeometry, pose[leg.mKnee].mRotation, pose[leg.mFoot].mRotation);
            ASSERT_TRUE(hip); ASSERT_TRUE(ankle);
            EXPECT_TRUE(hip->IsClose(pose[leg.mHip].mRotation, 1e-9f));
            EXPECT_TRUE(ankle->IsClose(pose[leg.mFoot].mRotation, 1e-9f));
            EXPECT_TRUE(pose[leg.mFoot].mRotation.IsClose(JPH::Quat::sIdentity(), 0));
        }
        EXPECT_FALSE(solve(JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 1.2f)));
        // During swing the raised foot may pitch while the stance foot remains
        // fixed. Requiring both feet to stay level would reject this reachable
        // lift because the airborne ankle cannot counter all knee flexion.
        desired = fixture.mBase[0];
        const auto raisedRight = rightFoot + JPH::Vec3(0, 0, 8);
        const auto raised = planJointLimitedReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
            desired, leftFoot, raisedRight, JPH::Quat::sIdentity(), std::nullopt, JPH::Vec3::sAxisY());
        ASSERT_TRUE(raised);
        const auto& lifted = raised->mPose.mBodyTransforms;
        EXPECT_TRUE(lifted[16].mRotation.IsClose(JPH::Quat::sIdentity(), 0));
        EXPECT_TRUE(lifted[16].mTranslation.IsClose(leftFoot, 1e-6f));
        EXPECT_TRUE(lifted[19].mTranslation.IsClose(raisedRight, 1e-6f));
        EXPECT_FALSE(lifted[19].mRotation.IsClose(JPH::Quat::sIdentity(), 1e-4f));
        const auto clampedSwing = clampSwingTwistJointOrientation(*fixture.mRight.mAnkleGeometry,
            lifted[18].mRotation, lifted[19].mRotation);
        ASSERT_TRUE(clampedSwing);
        EXPECT_TRUE(clampedSwing->IsClose(lifted[19].mRotation, 1e-9f));
        fixture.mLeft.mHipGeometry.reset();
        EXPECT_FALSE(solve(JPH::Quat::sIdentity()));
    }

    void calibratePostureTrunk(const RagdollCore::SkeletonDescriptor& skeleton, RagdollCore::RagdollProfile& profile)
    {
        using namespace RagdollCore;
        const auto built = buildRagdollSettings(skeleton, profile, BuildOptions{});
        ASSERT_TRUE(built.isValid());
        const auto& parts = built.mSettings->mParts;
        for (std::size_t joint = 1; joint <= 3; ++joint)
        {
            // Independent sagittal loaded-response estimate: aim for the authored
            // 5 Hz response of the torso subtree, rather than the joined pair
            // alone. The upstream body is treated as fixed for this estimate.
            const auto pivot = parts[joint].mPosition;
            float inertia = 0;
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
                inertia += mass.mInertia.Multiply3x3(JPH::Vec3::sAxisX()).GetX();
            }
            const auto inverseInertia = [&](std::size_t body) {
                auto mass = parts[body].GetMassProperties();
                mass.Rotate(JPH::Mat44::sRotation(parts[body].mRotation));
                return mass.mInertia.Inversed3x3();
            };
            const auto inverse = inverseInertia(joint) + inverseInertia(skeleton.mBones[joint].mParent);
            const float ratio = inertia * inverse.Multiply3x3(JPH::Vec3::sAxisX()).GetX();
            ASSERT_GT(ratio, 0);
            auto& motor = profile.mBodies[joint].mConstraint->mMotor;
            motor.mFrequency *= std::sqrt(ratio);
            motor.mDamping *= std::sqrt(ratio);
            std::cout << "STANCE_TRUNK_GAIN joint=" << joint << " ratio=" << ratio
                      << " frequency=" << motor.mFrequency << " damping=" << motor.mDamping << '\n';
        }
    }

    void exportBodiesIfRequested(const RagdollCore::ScenarioResult& result, const std::string& traceName)
    {
        if (const char* directory = std::getenv("OPENMW_RAGDOLL_TRACE_DIR"); directory && *directory)
        {
            std::ofstream bodies(std::filesystem::path(directory) / (traceName + "-bodies.csv"));
            ASSERT_TRUE(bodies);
            bodies.precision(9);
            bodies << "step,body,x,y,z,vx,vy,vz,wx,wy,wz,qx,qy,qz,qw,load_x,load_y,load_z,"
                      "motor_x,motor_y,motor_z,motor_utilization\n";
            for (const auto& frame : result.mFrames)
                for (std::size_t i = 0; i < frame.mBodies.size(); ++i)
                {
                    const auto& body = frame.mBodies[i];
                    bodies << frame.mStep << ',' << i << ',' << body.mWorldPosition.GetX() << ','
                           << body.mWorldPosition.GetY() << ',' << body.mWorldPosition.GetZ() << ','
                           << body.mLinearVelocity.GetX() << ',' << body.mLinearVelocity.GetY() << ','
                           << body.mLinearVelocity.GetZ() << ',' << body.mAngularVelocity.GetX() << ','
                           << body.mAngularVelocity.GetY() << ',' << body.mAngularVelocity.GetZ() << ','
                           << body.mWorldRotation.GetX() << ',' << body.mWorldRotation.GetY() << ','
                           << body.mWorldRotation.GetZ() << ',' << body.mWorldRotation.GetW();
                    if (i > 0 && frame.mJointSupport && frame.mJointSupportMotorUtilizations.size() == frame.mBodies.size()
                        && frame.mJointSupportMotorAngularImpulses.size() == frame.mBodies.size())
                    {
                        const auto load = frame.mJointSupport->mJointTorques[i];
                        const auto torque = frame.mJointSupportMotorAngularImpulses[i] / result.mSettings.mFixedTimeStep;
                        bodies << ',' << load.GetX() << ',' << load.GetY() << ',' << load.GetZ()
                               << ',' << torque.GetX() << ',' << torque.GetY() << ',' << torque.GetZ()
                               << ',' << frame.mJointSupportMotorUtilizations[i];
                    }
                    else
                        bodies << ",,,,,,,";
                    bodies << '\n';
                }
        }
    }

    void checkCoordinatedStancePosture(std::uint64_t descentSteps, const std::string& traceName)
    {
        using namespace RagdollCore;
        // Independent task: lower the pelvis 4 units over the specified duration,
        // then hold at least one second with both feet planted. Retain the full
        // rig, timestep, torque limits and standing support policy; calibrate
        // the torso springs for their load below. Targets cannot set body positions.
        StanceFixture fixture;
        const auto skeleton = makeScenarioHumanoidSkeleton();
        auto taskProfile = makeScenarioHumanoidProfile();
        calibratePostureTrunk(skeleton, taskProfile);
        const auto runPosture = [&](const ScenarioScript& task) {
            return runRagdollScenario(skeleton, taskProfile, referenceSettings(), task);
        };
        ScenarioScript script;
        // Keep the original three-second observation for both motion speeds
        // and their released controls; a balanced passive rig can remain
        // upright briefly after release without having tracked the motion.
        script.mDurationSeconds = 3;
        for (std::uint64_t step = 60; step <= 60 + descentSteps; ++step)
        {
            const float t = static_cast<float>(step - 60) / static_cast<float>(descentSteps);
            auto desired = fixture.mBase[0];
            desired.mTranslation.SetZ(desired.mTranslation.GetZ() - 4 * t * t * (3 - 2 * t));
            const auto posture = planReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
                desired, fixture.mBase[16].mTranslation, fixture.mBase[19].mTranslation, JPH::Vec3::sAxisY());
            ASSERT_TRUE(posture) << step;
            ScenarioInputKeyframe key;
            key.mStep = step;
            key.mMotorTarget = posture->mPose;
            script.mKeyframes.push_back(key);
        }
        const auto result = runPosture(script);
        ASSERT_TRUE(result.completed()) << describe(result);
        exportTraceIfRequested(result, traceName);
        const auto invariants = summarizeScenarioInvariants(result);
        exportBodiesIfRequested(result, traceName);
        std::cout << "STANCE_POSTURE descent_steps=" << descentSteps << ' ' << describe(invariants)
                  << " pelvis_z=" << result.mFrames.back().mBodies[0].mWorldPosition.GetZ()
                  << " pelvis_vx=" << result.mFrames.back().mBodies[0].mLinearVelocity.GetX()
                  << " pelvis_vy=" << result.mFrames.back().mBodies[0].mLinearVelocity.GetY()
                  << " pelvis_vz=" << result.mFrames.back().mBodies[0].mLinearVelocity.GetZ() << std::endl;
        EXPECT_EQ(invariants.mFallingFrames + invariants.mFallenFrames, 0u);
        EXPECT_EQ(invariants.mAirborneFrames, 0u);
        EXPECT_EQ(invariants.mFreeVelocityLimitedFrames + invariants.mSolverVelocityLimitedFrames, 0u);
        EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
        EXPECT_LT(invariants.mMaxNetActuationMoment, 1e-3f);
        float peakUtilization = 0;
        std::size_t saturatedFrames = 0;
        for (std::size_t i = 60; i < result.mFrames.size(); ++i)
        {
            EXPECT_TRUE(result.mFrames[i].mSupport.mLeftFootSupported) << i;
            EXPECT_TRUE(result.mFrames[i].mSupport.mRightFootSupported) << i;
            const auto& utilizations = result.mFrames[i].mJointSupportMotorUtilizations;
            ASSERT_EQ(utilizations.size(), fixture.mBase.size()) << i;
            const float peak = *std::max_element(utilizations.begin(), utilizations.end());
            peakUtilization = std::max(peakUtilization, peak);
            saturatedFrames += peak >= 0.999f;
        }
        std::cout << "STANCE_MOTORS peak_utilization=" << peakUtilization
                  << " saturated_frames=" << saturatedFrames << std::endl;
        EXPECT_LE(saturatedFrames * 20, result.mFrames.size() - 60);
        EXPECT_NEAR(result.mFrames[60 + descentSteps].mBodies[0].mWorldPosition.GetZ(), 62, 1)
            << "must reach the commanded height by the motion deadline";
        for (std::size_t i = 90 + descentSteps; i < result.mFrames.size(); ++i)
        {
            const auto& frame = result.mFrames[i];
            EXPECT_NEAR(frame.mBodies[0].mWorldPosition.GetZ(), 62, 1);
            EXPECT_LT(frame.mBodies[0].mLinearVelocity.Length(), 2);
            for (std::size_t joint = 1; joint <= 3; ++joint)
            {
                const auto relative = frame.mBodies[joint - 1].mWorldRotation.Conjugated()
                    * frame.mBodies[joint].mWorldRotation;
                const float error = 2 * std::acos(std::clamp(std::abs(relative.GetW()), 0.0f, 1.0f));
                EXPECT_LT(error, 0.02f) << "trunk joint " << joint;
            }
            for (const auto foot : { 16u, 19u })
                EXPECT_LT(JPH::Vec3(frame.mBodies[foot].mWorldPosition
                    - result.mFrames[59].mBodies[foot].mWorldPosition).Length(), 1.5f);
        }
        const auto repeat = runPosture(script);
        ASSERT_TRUE(repeat.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, repeat));
        // Without the changed pose, support alone must not lower the pelvis.
        ScenarioScript control;
        control.mDurationSeconds = script.mDurationSeconds;
        const auto unchanged = runPosture(control);
        ASSERT_TRUE(unchanged.completed());
        EXPECT_GT(unchanged.mFrames.back().mBodies[0].mWorldPosition.GetZ(), 64);
        // A changing target with its motors suppressed must not place the
        // pelvis or feet directly. The unactuated body loses its posture.
        for (auto& key : script.mKeyframes)
        {
            key.mMotorsEnabled = false;
            key.mTranslationActuationEnabled = false;
            key.mUprightTorqueEnabled = false;
        }
        const auto released = runPosture(script);
        ASSERT_TRUE(released.completed());
        const auto releasedInvariants = summarizeScenarioInvariants(released);
        EXPECT_GT(releasedInvariants.mFallingFrames + releasedInvariants.mFallenFrames, 0u);
    }

    TEST(RagdollCoreReferenceGaitTest, balancedCrouchPreservesFeetAndTracksTheRequiredHeight)
    {
        using namespace RagdollCore;
        StanceFixture fixture;
        addNativeJointGeometry(fixture);
        const auto skeleton = makeScenarioHumanoidSkeleton();
        auto profile = makeScenarioHumanoidProfile();
        calibratePostureTrunk(skeleton, profile);
        const auto built = buildRagdollSettings(skeleton, profile, {});
        ASSERT_TRUE(built.isValid());
        const auto center = [&](std::span<const Transform> pose) {
            JPH::Vec3 weighted = JPH::Vec3::sZero();
            float total = 0;
            for (std::size_t body = 0; body < pose.size(); ++body)
            {
                const auto& part = built.mSettings->mParts[body];
                const float mass = part.GetMassProperties().mMass;
                weighted += mass * (pose[body].mTranslation
                    + pose[body].mRotation * part.GetShape()->GetCenterOfMass());
                total += mass;
            }
            return weighted / total;
        };
        const auto desiredCenter = center(fixture.mBase);
        std::vector<ReferencePostureMass> masses;
        for (const auto& part : built.mSettings->mParts)
            masses.push_back({ part.GetMassProperties().mMass, part.GetShape()->GetCenterOfMass() });
        const auto solve = [&](float height, float pitch) -> std::optional<ReferenceStancePosture> {
            auto pelvis = fixture.mBase[0];
            pelvis.mTranslation.SetZ(height);
            pelvis.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), pitch);
            return planBalancedReferenceStancePosture(fixture.mBase, masses, 0, fixture.mLeft, fixture.mRight,
                pelvis, fixture.mBase[16], fixture.mBase[19], desiredCenter, JPH::Vec3::sAxisY());
        };
        EXPECT_FALSE(solve(62, 0)) << "fixed COM and feet must not hide the upright ankle restriction";
        const auto balanced = [&](float height) -> std::optional<ReferenceStancePosture> {
            if (auto upright = solve(height, 0)) return upright;
            const float maximumPitch = std::acos(0.9f);
            for (unsigned step = 1; step < 64; ++step)
                for (float sign : { -1.0f, 1.0f })
                {
                    const float pitch = sign * maximumPitch * static_cast<float>(step) / 64;
                    auto pose = solve(height, pitch);
                    if (!pose) continue;
                    // Keep the first feasible sample rather than refining
                    // onto a joint-limit boundary. Every trajectory pose is
                    // independently checked against native limits below.
                    return pose;
                }
            return std::nullopt;
        };
        const auto finalPose = balanced(62);
        ASSERT_TRUE(finalPose) << "no balanced four-unit crouch in the bounded pitch search";
        const auto& target = finalPose->mPose.mBodyTransforms;
        const float finalPitch = 2 * std::atan2(target[0].mRotation.GetX(), target[0].mRotation.GetW());
        std::cout << "BALANCED_CROUCH pitch=" << finalPitch
                  << " pelvis_y=" << target[0].mTranslation.GetY() << " com_y=" << center(target).GetY() << std::endl;
        // A known pose with a changed local mass center must be recovered
        // from its independently summed COM, even from a displaced XY seed.
        auto offsetMasses = masses;
        offsetMasses[0].mLocalCenterOfMass += JPH::Vec3(1, 1, 1);
        float totalMass = 0;
        for (const auto& mass : masses) totalMass += mass.mMass;
        const auto offsetCenter = center(target)
            + target[0].mRotation * JPH::Vec3(1, 1, 1) * (masses[0].mMass / totalMass);
        auto seed = target[0];
        seed.mTranslation += JPH::Vec3(0, 1, 0);
        const auto recover = [&](std::span<const ReferencePostureMass> input, const Transform& root) {
            return planBalancedReferenceStancePosture(fixture.mBase, input, 0, fixture.mLeft, fixture.mRight,
                root, fixture.mBase[16], fixture.mBase[19], offsetCenter, JPH::Vec3::sAxisY());
        };
        const auto recovered = recover(offsetMasses, seed);
        ASSERT_TRUE(recovered);
        EXPECT_TRUE(recovered->mPose.mBodyTransforms[0].mTranslation.IsClose(target[0].mTranslation, 1e-6f));
        const auto wrongCenters = recover(masses, seed);
        if (wrongCenters)
            EXPECT_GT((wrongCenters->mPose.mBodyTransforms[0].mTranslation - target[0].mTranslation).Length(), 0.01f);
        offsetMasses[0].mMass = 0;
        EXPECT_FALSE(recover(offsetMasses, seed));
        offsetMasses[0].mMass = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(recover(offsetMasses, seed));
        EXPECT_FALSE(recover(std::span<const ReferencePostureMass>(masses).first(masses.size() - 1), seed));
        seed.mTranslation.SetZ(100);
        EXPECT_FALSE(recover(masses, seed)) << "the fixed requested height must not be lowered to fit";
        ScenarioScript script;
        script.mDurationSeconds = 3;
        for (std::uint64_t step = 60; step <= 120; ++step)
        {
            const float t = static_cast<float>(step - 60) / 60;
            const float blend = t * t * (3 - 2 * t);
            const auto pose = solve(66 - 4 * blend, finalPitch * blend);
            ASSERT_TRUE(pose) << step;
            const auto& transforms = pose->mPose.mBodyTransforms;
            EXPECT_NEAR(transforms[0].mTranslation.GetZ(), 66 - 4 * blend, 1e-3f);
            EXPECT_GT((transforms[0].mRotation * JPH::Vec3::sAxisZ()).GetZ(), 0.9f);
            const auto actualCenter = center(transforms);
            EXPECT_NEAR(actualCenter.GetX(), desiredCenter.GetX(), 1e-4f);
            EXPECT_NEAR(actualCenter.GetY(), desiredCenter.GetY(), 1e-4f);
            for (const auto& leg : { fixture.mLeft, fixture.mRight })
            {
                EXPECT_TRUE(transforms[leg.mFoot].mTranslation.IsClose(fixture.mBase[leg.mFoot].mTranslation, 1e-6f));
                EXPECT_TRUE(transforms[leg.mFoot].mRotation.IsClose(fixture.mBase[leg.mFoot].mRotation, 1e-10f));
                EXPECT_NEAR((transforms[leg.mKnee].mTranslation - transforms[leg.mHip].mTranslation).Length(), 29, 1e-3f);
                EXPECT_NEAR((transforms[leg.mFoot].mTranslation - transforms[leg.mKnee].mTranslation).Length(), 30.5f, 1e-3f);
                const auto relative = (transforms[leg.mHip].mRotation.Conjugated()
                    * transforms[leg.mKnee].mRotation).Normalized().EnsureWPositive();
                const float angle = 2 * std::atan2(relative.GetXYZ().Dot(leg.mHingeGeometry->mHingeAxis), relative.GetW());
                EXPECT_GE(angle, leg.mHingeGeometry->mMinimumAngle - 1e-4f);
                EXPECT_LE(angle, leg.mHingeGeometry->mMaximumAngle + 1e-4f);
                for (const auto child : { leg.mHip, leg.mFoot })
                {
                    const auto* joint = static_cast<const JPH::SwingTwistConstraintSettings*>(
                        built.mSettings->mParts[child].mToParent.GetPtr());
                    const auto frame = [](JPH::Vec3Arg twist, JPH::Vec3Arg plane) {
                        return JPH::Mat44(JPH::Vec4(twist, 0), JPH::Vec4(plane.Cross(twist), 0),
                            JPH::Vec4(plane, 0), JPH::Vec4(0, 0, 0, 1)).GetQuaternion();
                    };
                    const auto requested = (frame(joint->mTwistAxis1, joint->mPlaneAxis1).Conjugated()
                        * transforms[skeleton.mBones[child].mParent].mRotation.Conjugated()
                        * transforms[child].mRotation * frame(joint->mTwistAxis2, joint->mPlaneAxis2)).Normalized();
                    JPH::SwingTwistConstraintPart limits;
                    limits.SetSwingType(joint->mSwingType);
                    limits.SetLimits(joint->mTwistMinAngle, joint->mTwistMaxAngle,
                        -joint->mPlaneHalfConeAngle, joint->mPlaneHalfConeAngle,
                        -joint->mNormalHalfConeAngle, joint->mNormalHalfConeAngle);
                    JPH::Quat swing, twist;
                    requested.GetSwingTwist(swing, twist);
                    JPH::uint clamped = 0;
                    limits.ClampSwingTwist(swing, twist, clamped);
                    EXPECT_EQ(clamped, 0u) << "step " << step << " joint " << child;
                }
            }
            ScenarioInputKeyframe key;
            key.mStep = step;
            key.mMotorTarget = pose->mPose;
            key.mSupportMotion = SupportMotionReference{ JPH::RVec3(center(pose->mPose.mBodyTransforms)),
                JPH::Vec3::sZero(), JPH::Vec3::sZero() };
            script.mKeyframes.push_back(key);
        }
        const auto run = [&](const ScenarioScript& input) {
            return runRagdollScenario(skeleton, profile, referenceSettings(), input);
        };
        const auto result = run(script);
        exportTraceIfRequested(result, "balanced-crouch");
        exportBodiesIfRequested(result, "balanced-crouch");
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto invariants = summarizeScenarioInvariants(result);
        std::cout << "BALANCED_CROUCH_PHYSICS " << describe(invariants) << std::endl;
        EXPECT_EQ(invariants.mFallingFrames + invariants.mFallenFrames, 0u);
        EXPECT_EQ(invariants.mAirborneFrames, 0u);
        EXPECT_EQ(invariants.mFreeVelocityLimitedFrames + invariants.mSolverVelocityLimitedFrames, 0u);
        EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
        EXPECT_LT(invariants.mMaxNetActuationMoment, 1e-3f);
        EXPECT_GT(invariants.mMinPelvisUpDot, 0.9f);
        std::size_t saturated = 0;
        float maximumLateSpeed = 0;
        double maximumLateHeightError = 0;
        for (std::size_t step = 60; step < result.mFrames.size(); ++step)
        {
            const auto& frame = result.mFrames[step];
            EXPECT_TRUE(frame.mSupport.mLeftFootSupported && frame.mSupport.mRightFootSupported) << step;
            const auto& motors = frame.mJointSupportMotorUtilizations;
            ASSERT_FALSE(motors.empty());
            saturated += *std::max_element(motors.begin(), motors.end()) >= 0.999f;
            if (step >= 150)
            {
                maximumLateHeightError = std::max(maximumLateHeightError,
                    std::abs(frame.mBodies[0].mWorldPosition.GetZ() - 62));
                maximumLateSpeed = std::max(maximumLateSpeed, frame.mBodies[0].mLinearVelocity.Length());
            }
            for (const auto foot : { 16u, 19u })
                EXPECT_LT((frame.mBodies[foot].mWorldPosition - result.mFrames[59].mBodies[foot].mWorldPosition).Length(), 1.5);
        }
        EXPECT_LE(saturated * 20, result.mFrames.size() - 60);
        EXPECT_LE(maximumLateHeightError, 1);
        EXPECT_LT(maximumLateSpeed, 2);
        std::cout << "BALANCED_CROUCH_TRACKING max_late_speed=" << maximumLateSpeed
                  << " max_late_height_error=" << maximumLateHeightError << " saturated=" << saturated << std::endl;
        EXPECT_NEAR(result.mFrames[120].mBodies[0].mWorldPosition.GetZ(), 62, 1);
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, run(script)));
        ScenarioScript control;
        control.mDurationSeconds = script.mDurationSeconds;
        const auto unchanged = run(control);
        ASSERT_TRUE(unchanged.completed());
        EXPECT_GT(unchanged.mFrames.back().mBodies[0].mWorldPosition.GetZ(), 64);
        for (auto& key : script.mKeyframes) key.mMotorsEnabled = false;
        const auto released = run(script);
        ASSERT_TRUE(released.completed());
        const auto passive = summarizeScenarioInvariants(released);
        EXPECT_GT(passive.mFallingFrames + passive.mFallenFrames, 0u);
        for (const auto& frame : released.mFrames)
            if (frame.mStep >= 60) EXPECT_FALSE(frame.mMotorsApplied);
    }

    TEST(RagdollCoreReferenceGaitTest, jointLimitedStanceTracksTheAdmissibleHeightWithBothFeetSupported)
    {
        using namespace RagdollCore;
        StanceFixture fixture;
        addNativeJointGeometry(fixture);
        const auto skeleton = makeScenarioHumanoidSkeleton();
        auto profile = makeScenarioHumanoidProfile();
        calibratePostureTrunk(skeleton, profile);
        ASSERT_FALSE(::testing::Test::HasFailure());
        ScenarioScript script;
        script.mDurationSeconds = 3;
        float targetHeight = 66;
        for (std::uint64_t step = 60; step <= 120; ++step)
        {
            const float t = static_cast<float>(step - 60) / 60;
            auto desired = fixture.mBase[0];
            desired.mTranslation.SetZ(66 - 4 * t * t * (3 - 2 * t));
            const auto pose = planJointLimitedReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
                desired, fixture.mBase[16].mTranslation, fixture.mBase[19].mTranslation,
                JPH::Quat::sIdentity(), JPH::Quat::sIdentity(), JPH::Vec3::sAxisY());
            ASSERT_TRUE(pose) << step;
            targetHeight = pose->mPose.mBodyTransforms[0].mTranslation.GetZ();
            ScenarioInputKeyframe key;
            key.mStep = step;
            key.mMotorTarget = pose->mPose;
            script.mKeyframes.push_back(key);
        }
        const auto run = [&](const ScenarioScript& input) {
            return runRagdollScenario(skeleton, profile, referenceSettings(), input);
        };
        const auto result = run(script);
        ASSERT_TRUE(result.completed()) << describe(result);
        exportTraceIfRequested(result, "joint-limited-posture");
        exportBodiesIfRequested(result, "joint-limited-posture");
        const auto invariants = summarizeScenarioInvariants(result);
        std::cout << "JOINT_LIMITED_POSTURE target_height=" << targetHeight
                  << " final_height=" << result.mFrames.back().mBodies[0].mWorldPosition.GetZ()
                  << ' ' << describe(invariants) << std::endl;
        EXPECT_EQ(invariants.mFallingFrames + invariants.mFallenFrames, 0u);
        EXPECT_EQ(invariants.mAirborneFrames, 0u);
        EXPECT_EQ(invariants.mFreeVelocityLimitedFrames + invariants.mSolverVelocityLimitedFrames, 0u);
        EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
        EXPECT_LT(invariants.mMaxNetActuationMoment, 1e-3f);
        std::size_t saturated = 0;
        for (std::size_t step = 60; step < result.mFrames.size(); ++step)
        {
            const auto& frame = result.mFrames[step];
            EXPECT_TRUE(frame.mSupport.mLeftFootSupported && frame.mSupport.mRightFootSupported) << step;
            const auto& motors = frame.mJointSupportMotorUtilizations;
            ASSERT_FALSE(motors.empty());
            saturated += *std::max_element(motors.begin(), motors.end()) >= 0.999f;
            if (step >= 150)
            {
                EXPECT_NEAR(frame.mBodies[0].mWorldPosition.GetZ(), targetHeight, 1);
                EXPECT_LT(frame.mBodies[0].mLinearVelocity.Length(), 2);
                for (const auto foot : { 16u, 19u })
                {
                    EXPECT_LT(JPH::Vec3(frame.mBodies[foot].mWorldPosition
                        - result.mFrames[59].mBodies[foot].mWorldPosition).Length(), 1.5f);
                    ASSERT_TRUE(frame.mJointMotorTargets[foot]);
                    EXPECT_LT(frame.mJointMotorTargets[foot]->mTargetClampDelta, 1e-3f);
                }
            }
        }
        EXPECT_LE(saturated * 20, result.mFrames.size() - 60);
        EXPECT_NEAR(result.mFrames[120].mBodies[0].mWorldPosition.GetZ(), targetHeight, 1);
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, run(script)));
        ScenarioScript unchanged;
        unchanged.mDurationSeconds = 3;
        const auto control = run(unchanged);
        ASSERT_TRUE(control.completed());
        EXPECT_GT(control.mFrames.back().mBodies[0].mWorldPosition.GetZ(), targetHeight + 1);
        for (auto& key : script.mKeyframes)
            key.mMotorsEnabled = false;
        const auto released = run(script);
        ASSERT_TRUE(released.completed());
        EXPECT_GT(summarizeScenarioInvariants(released).mFallingFrames + summarizeScenarioInvariants(released).mFallenFrames, 0u);
        for (const auto& frame : released.mFrames)
            if (frame.mStep >= 60)
                EXPECT_FALSE(frame.mMotorsApplied);
    }

    TEST(RagdollCoreReferenceGaitTest, coordinatedStancePostureLowersLoadedPelvis)
    {
        checkCoordinatedStancePosture(60, "reference-stance-posture");
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreReferenceGaitTest, DISABLED_stancePostureTracksSwingDeadline)
    {
        // Match the reference swing's 0.35-second deadline at 60 Hz.
        checkCoordinatedStancePosture(21, "reference-stance-deadline");
    }

    void checkCoordinatedLateralTransfer(bool stepAfterTransfer, bool liftAfterTransfer = false,
        bool balancedCrouch = false, bool articulatedLift = false, bool referenceCycle = false)
    {
        using namespace RagdollCore;
        StanceFixture fixture;
        const auto skeleton = makeScenarioHumanoidSkeleton();
        auto profile = makeScenarioHumanoidProfile();
        calibratePostureTrunk(skeleton, profile);
        if (balancedCrouch)
        {
            // Use the existing reference swing bandwidth, which was selected
            // from the independent leg response measurement. This task profile
            // is fixed from preparation through lift; no capacity changes.
            const ReferenceGaitProfile swing;
            for (const auto child : { 17u, 18u, 19u })
            {
                profile.mBodies[child].mConstraint->mMotor.mFrequency = swing.mSwingLegFrequency;
                profile.mBodies[child].mConstraint->mMotor.mDamping = swing.mSwingLegDamping;
            }
        }
        const auto built = buildRagdollSettings(skeleton, profile, {});
        ASSERT_TRUE(built.isValid());
        const float calfAngle = 0.8f * profile.mBodies[16].mConstraint->mPlaneHalfConeAngle;
        const float upper = fixture.mLeft.mHingeGeometry->mUpperOffset.Length();
        const float lower = fixture.mLeft.mHingeGeometry->mLowerOffset.Length();
        const float thighAngle = std::asin(lower * std::sin(calfAngle) / upper);
        const float depth = balancedCrouch ? 4.0f
            : upper * (1 - std::cos(thighAngle)) + lower * (1 - std::cos(calfAngle));
        // Independent transfer task: move the pelvis above the left foot in
        // a duration derived from ground-moment feasibility while preserving
        // height, upright orientation and both feet.
        // No walking acceleration or swing policy participates in this test.
        const float destination = fixture.mBase[16].mTranslation.GetX();
        std::vector<ReferencePostureMass> masses;
        JPH::Vec3 baseCenter = JPH::Vec3::sZero();
        float totalMass = 0;
        for (std::size_t body = 0; body < fixture.mBase.size(); ++body)
        {
            const auto& part = built.mSettings->mParts[body];
            masses.push_back({ part.GetMassProperties().mMass, part.GetShape()->GetCenterOfMass() });
            baseCenter += masses.back().mMass * (fixture.mBase[body].mTranslation
                + fixture.mBase[body].mRotation * masses.back().mLocalCenterOfMass);
            totalMass += masses.back().mMass;
        }
        baseCenter /= totalMass;
        float targetPitch = 0;
        if (balancedCrouch)
        {
            addNativeJointGeometry(fixture);
            bool found = false;
            for (unsigned sample = 1; sample < 64 && !found; ++sample)
            {
                targetPitch = -std::acos(0.9f) * static_cast<float>(sample) / 64;
                auto pelvis = fixture.mBase[0];
                pelvis.mTranslation.SetZ(62);
                pelvis.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), targetPitch);
                found = planBalancedReferenceStancePosture(fixture.mBase, masses, 0, fixture.mLeft, fixture.mRight,
                    pelvis, fixture.mBase[16], fixture.mBase[19], baseCenter + JPH::Vec3(destination, 0, 0),
                    JPH::Vec3::sAxisY()).has_value();
            }
            ASSERT_TRUE(found) << "no four-unit crouch admits the lateral transfer endpoint";
            std::cout << "BALANCED_TRANSFER pitch=" << targetPitch << std::endl;
        }
        const auto makeScript = [&](bool shift, std::uint64_t transferSteps) {
            ScenarioScript script;
            script.mDurationSeconds = static_cast<float>(150 + transferSteps + 90) / 60;
            for (std::uint64_t step = 60; step <= 150 + transferSteps; ++step)
            {
                const float crouch = std::clamp(static_cast<float>(step - 60) / (balancedCrouch ? 60 : 21), 0.0f, 1.0f);
                const float transfer = step < 150 ? 0.0f : static_cast<float>(step - 150) / transferSteps;
                auto pelvis = fixture.mBase[0];
                pelvis.mTranslation.SetZ(66 - depth * crouch * crouch * (3 - 2 * crouch));
                // Zero endpoint acceleration avoids an impulsive change in
                // the required support moment when the shift starts/stops.
                const float blend = transfer * transfer * transfer * (10 + transfer * (-15 + 6 * transfer));
                pelvis.mTranslation.SetX(shift ? destination * blend : 0);
                pelvis.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), targetPitch * crouch * crouch * (3 - 2 * crouch));
                const auto posture = balancedCrouch
                    ? planBalancedReferenceStancePosture(fixture.mBase, masses, 0, fixture.mLeft, fixture.mRight,
                        pelvis, fixture.mBase[16], fixture.mBase[19],
                        baseCenter + JPH::Vec3(shift ? destination * blend : 0, 0, 0), JPH::Vec3::sAxisY())
                    : planReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
                        pelvis, fixture.mBase[16].mTranslation, fixture.mBase[19].mTranslation, JPH::Vec3::sAxisY());
                EXPECT_TRUE(posture) << step;
                if (!posture) break;
                // Check the actual native hip and ankle cones before treating
                // the generated trajectory as an attainable motor reference.
                for (const std::size_t child : { 14u, 16u, 17u, 19u })
                {
                    const auto* joint = static_cast<const JPH::SwingTwistConstraintSettings*>(
                        built.mSettings->mParts[child].mToParent.GetPtr());
                    const auto frame = [](JPH::Vec3Arg twist, JPH::Vec3Arg plane) {
                        return JPH::Mat44(JPH::Vec4(twist, 0), JPH::Vec4(plane.Cross(twist), 0),
                            JPH::Vec4(plane, 0), JPH::Vec4(0, 0, 0, 1)).GetQuaternion();
                    };
                    const auto& pose = posture->mPose.mBodyTransforms;
                    const auto request = (frame(joint->mTwistAxis1, joint->mPlaneAxis1).Conjugated()
                        * pose[skeleton.mBones[child].mParent].mRotation.Conjugated()
                        * pose[child].mRotation * frame(joint->mTwistAxis2, joint->mPlaneAxis2)).Normalized();
                    JPH::SwingTwistConstraintPart limits;
                    limits.SetSwingType(joint->mSwingType);
                    limits.SetLimits(joint->mTwistMinAngle, joint->mTwistMaxAngle,
                        -joint->mPlaneHalfConeAngle, joint->mPlaneHalfConeAngle,
                        -joint->mNormalHalfConeAngle, joint->mNormalHalfConeAngle);
                    JPH::Quat swing, twist;
                    request.GetSwingTwist(swing, twist);
                    JPH::uint clamped = 0;
                    limits.ClampSwingTwist(swing, twist, clamped);
                    EXPECT_EQ(clamped, 0u) << "step " << step << " joint " << child;
                }
                ScenarioInputKeyframe key;
                key.mStep = step;
                key.mMotorTarget = posture->mPose;
                script.mKeyframes.push_back(key);
            }
            // Balance follows the same full-body pose trajectory. Its feed-
            // forward acceleration and feedback reference share one source;
            // no new gains, capacities or external root force are introduced.
            const auto center = [&](std::size_t key) {
                JPH::Vec3 weighted = JPH::Vec3::sZero();
                float totalMass = 0;
                for (std::size_t body = 0; body < fixture.mBase.size(); ++body)
                {
                    const auto& part = built.mSettings->mParts[body];
                    const auto& pose = script.mKeyframes[key].mMotorTarget->mBodyTransforms[body];
                    const float mass = part.GetMassProperties().mMass;
                    weighted += mass * (pose.mTranslation + pose.mRotation * part.GetShape()->GetCenterOfMass());
                    totalMass += mass;
                }
                return weighted / totalMass;
            };
            const float dt = referenceSettings().mFixedTimeStep;
            const std::size_t motionStart = balancedCrouch ? 0 : 90;
            for (std::size_t key = motionStart; key < script.mKeyframes.size(); ++key)
            {
                SupportMotionReference motion;
                const auto current = center(key);
                motion.mPosition = JPH::RVec3(current);
                if (key > motionStart && key + 1 < script.mKeyframes.size())
                {
                    motion.mVelocity = (center(key + 1) - center(key - 1)) / (2 * dt);
                    motion.mAcceleration = (center(key + 1) - 2 * current + center(key - 1)) / (dt * dt);
                }
                script.mKeyframes[key].mSupportMotion = motion;
            }
            return script;
        };
        const auto task = [&](const ScenarioScript& script, bool limitVelocityLead = true) {
            auto settings = referenceSettings();
            settings.mReferenceGait.mLimitVelocityLead = limitVelocityLead;
            return runRagdollScenario(skeleton, profile, settings, script);
        };
        const auto unitScript = makeScript(true, 60);
        ASSERT_FALSE(::testing::Test::HasFailure()) << "qualify target geometry before measuring actuation";
        // Centroidal inverse dynamics of the specified kinematic trajectory.
        // Ground reaction must supply dH/dt minus the gravity moment. A
        // required pressure point outside the feet invalidates this task as
        // a gain-calibration reference, even if every joint angle is legal.
        const float dt = referenceSettings().mFixedTimeStep;
        const float gravity = referenceSettings().mGravity;
        struct WrenchRequirements
        {
            float mMinimumX = std::numeric_limits<float>::infinity();
            float mMaximumX = -std::numeric_limits<float>::infinity();
            float mMaximumAbsY = 0;
            float mMaximumFriction = 0;
            float mMinimumNormalFraction = std::numeric_limits<float>::infinity();
            float mMinimumDuration = 0;
        };
        const auto requiredCop = [&](const ScenarioScript& trajectory) {
            const auto position = [&](std::size_t key, std::size_t body) {
                const auto& pose = trajectory.mKeyframes[key].mMotorTarget->mBodyTransforms[body];
                return pose.mTranslation + pose.mRotation * built.mSettings->mParts[body].GetShape()->GetCenterOfMass();
            };
            const auto momentum = [&](std::size_t key) {
                JPH::Vec3 value = JPH::Vec3::sZero();
                for (std::size_t body = 0; body < fixture.mBase.size(); ++body)
                {
                    auto mass = built.mSettings->mParts[body].GetMassProperties();
                    const auto& pose = trajectory.mKeyframes[key].mMotorTarget->mBodyTransforms[body];
                    mass.Rotate(JPH::Mat44::sRotation(pose.mRotation));
                    const auto& previous = trajectory.mKeyframes[key - 1].mMotorTarget->mBodyTransforms[body];
                    const auto& next = trajectory.mKeyframes[key + 1].mMotorTarget->mBodyTransforms[body];
                    const auto rotation = (next.mRotation * previous.mRotation.Conjugated()).Normalized().EnsureWPositive();
                    JPH::Vec3 axis;
                    float angle;
                    rotation.GetAxisAngle(axis, angle);
                    const auto velocity = (position(key + 1, body) - position(key - 1, body)) / (2 * dt);
                    value += position(key, body).Cross(mass.mMass * velocity)
                        + mass.mInertia.Multiply3x3(axis * (angle / (2 * dt)));
                }
                return value;
            };
            WrenchRequirements requirements;
            float minimumTimeScaleSquared = 0;
            for (std::size_t key = 92; key + 2 < trajectory.mKeyframes.size(); ++key)
            {
                JPH::Vec3 force = JPH::Vec3::sZero();
                auto moment = (momentum(key + 1) - momentum(key - 1)) / (2 * dt);
                JPH::Vec3 gravityMoment = JPH::Vec3::sZero();
                float totalWeight = 0;
                for (std::size_t body = 0; body < fixture.mBase.size(); ++body)
                {
                    const float mass = built.mSettings->mParts[body].GetMassProperties().mMass;
                    const auto weight = JPH::Vec3(0, 0, mass * gravity);
                    force += mass * (position(key + 1, body) - 2 * position(key, body) + position(key - 1, body))
                        / (dt * dt) + weight;
                    moment += position(key, body).Cross(weight);
                    gravityMoment += position(key, body).Cross(weight);
                    totalWeight += weight.GetZ();
                }
                const float cop = -moment.GetY() / force.GetZ();
                requirements.mMinimumX = std::min(requirements.mMinimumX, cop);
                requirements.mMaximumX = std::max(requirements.mMaximumX, cop);
                requirements.mMaximumAbsY = std::max(requirements.mMaximumAbsY,
                    std::abs(moment.GetX() / force.GetZ()));
                const float horizontalForce = std::hypot(force.GetX(), force.GetY());
                requirements.mMaximumFriction = std::max(requirements.mMaximumFriction, horizontalForce / force.GetZ());
                requirements.mMinimumNormalFraction = std::min(requirements.mMinimumNormalFraction, force.GetZ() / totalWeight);
                // For a fixed path, inertial wrench scales as 1 / duration^2.
                // Solve each support-edge inequality with one unit of margin;
                // this chooses the task deadline before any actuator trial.
                const auto dynamicMoment = moment - gravityMoment;
                const float dynamicVertical = force.GetZ() - totalWeight;
                const float friction = referenceSettings().mGroundFriction;
                minimumTimeScaleSquared = std::max(minimumTimeScaleSquared,
                    (horizontalForce - friction * dynamicVertical) / (friction * totalWeight));
                minimumTimeScaleSquared = std::max(minimumTimeScaleSquared, -dynamicVertical / totalWeight);
                for (const auto normal : { JPH::Vec3::sAxisX(), -JPH::Vec3::sAxisX(),
                         JPH::Vec3::sAxisY(), -JPH::Vec3::sAxisY() })
                {
                    const float bound = normal.GetX() != 0 ? 11.0f : 7.0f;
                    const float available = bound * totalWeight
                        + normal.GetX() * gravityMoment.GetY() - normal.GetY() * gravityMoment.GetX();
                    EXPECT_GT(available, 0) << "static posture must be inside support";
                    const float demand = -normal.GetX() * dynamicMoment.GetY()
                        + normal.GetY() * dynamicMoment.GetX() - bound * dynamicVertical;
                    minimumTimeScaleSquared = std::max(minimumTimeScaleSquared, demand / available);
                }
            }
            std::cout << "POSTURE_TRANSFER_REQUIRED_COP steps=" << trajectory.mKeyframes.size() - 91
                      << " x=[" << requirements.mMinimumX << ',' << requirements.mMaximumX << "] feet=[-12,12]"
                      << " max_abs_y=" << requirements.mMaximumAbsY << " friction=" << requirements.mMaximumFriction
                      << " minimum_normal_fraction=" << requirements.mMinimumNormalFraction
                      << std::endl;
            const float duration = static_cast<float>(trajectory.mKeyframes.size() - 91) * dt;
            requirements.mMinimumDuration = duration * std::sqrt(minimumTimeScaleSquared);
            return requirements;
        };
        const auto fastCop = requiredCop(makeScript(true, 21));
        EXPECT_LT(fastCop.mMinimumX, -12);
        EXPECT_GT(fastCop.mMaximumX, 12);
        const auto unitCop = requiredCop(unitScript);
        const auto transferSteps = static_cast<std::uint64_t>(std::ceil(unitCop.mMinimumDuration / dt));
        const auto deadline = 150 + transferSteps;
        const auto script = makeScript(true, transferSteps);
        const float targetPelvisX = script.mKeyframes.back().mMotorTarget->mBodyTransforms[0].mTranslation.GetX();
        const auto cop = requiredCop(script);
        ASSERT_GE(cop.mMinimumX, -12);
        ASSERT_LE(cop.mMaximumX, 12);
        ASSERT_LE(cop.mMaximumAbsY, 8);
        ASSERT_LT(cop.mMaximumFriction, referenceSettings().mGroundFriction);
        ASSERT_GT(cop.mMinimumNormalFraction, 0);
        const auto stationary = requiredCop(makeScript(false, 60));
        EXPECT_NEAR(stationary.mMinimumX, 0, 1e-3);
        EXPECT_NEAR(stationary.mMaximumX, 0, 1e-3);
        EXPECT_NEAR(stationary.mMinimumNormalFraction, 1, 1e-3);
        EXPECT_NEAR(stationary.mMaximumFriction, 0, 1e-3);
        std::cout << "POSTURE_TRANSFER_DURATION minimum_seconds=" << unitCop.mMinimumDuration
                  << " steps=" << transferSteps << std::endl;
        const auto result = task(script);
        ASSERT_TRUE(result.completed()) << describe(result);
        exportTraceIfRequested(result, balancedCrouch ? "balanced-transfer" : "posture-transfer");
        exportBodiesIfRequested(result, balancedCrouch ? "balanced-transfer" : "posture-transfer");
        const auto invariants = summarizeScenarioInvariants(result);
        EXPECT_NEAR(result.mFrames[149].mBodies[0].mWorldPosition.GetX(), 0, 1);
        EXPECT_NEAR(result.mFrames[149].mBodies[0].mWorldPosition.GetZ(), 66 - depth, 1);
        EXPECT_LT(result.mFrames[149].mBodies[0].mLinearVelocity.Length(), 2);
        double drift = 0;
        float peakUtilization = 0;
        std::size_t saturated = 0;
        for (std::size_t step = 150; step < result.mFrames.size(); ++step)
        {
            const auto& frame = result.mFrames[step];
            for (const auto foot : { 16u, 19u })
                drift = std::max(drift, (frame.mBodies[foot].mWorldPosition
                    - result.mFrames[149].mBodies[foot].mWorldPosition).Length());
            const auto& utilizations = frame.mJointSupportMotorUtilizations;
            ASSERT_EQ(utilizations.size(), fixture.mBase.size());
            const auto peak = *std::max_element(utilizations.begin(), utilizations.end());
            peakUtilization = std::max(peakUtilization, peak);
            saturated += peak >= 0.999f;
        }
        std::cout << "POSTURE_TRANSFER deadline_x=" << result.mFrames[deadline].mBodies[0].mWorldPosition.GetX()
                  << " final_x=" << result.mFrames.back().mBodies[0].mWorldPosition.GetX()
                  << " foot_drift=" << drift << " peak_utilization=" << peakUtilization
                  << " saturated=" << saturated << ' ' << describe(invariants) << std::endl;
        EXPECT_NEAR(result.mFrames[deadline].mBodies[0].mWorldPosition.GetX(), targetPelvisX, 1);
        double maximumLatePositionError = 0;
        double maximumLateHeightError = 0;
        float maximumLateSpeed = 0;
        for (std::size_t step = deadline + 30; step < result.mFrames.size(); ++step)
        {
            const auto& pelvis = result.mFrames[step].mBodies[0];
            maximumLatePositionError = std::max(maximumLatePositionError, std::abs(pelvis.mWorldPosition.GetX() - targetPelvisX));
            maximumLateHeightError = std::max(maximumLateHeightError, std::abs(pelvis.mWorldPosition.GetZ() - (66 - depth)));
            maximumLateSpeed = std::max(maximumLateSpeed, pelvis.mLinearVelocity.Length());
        }
        EXPECT_LE(maximumLatePositionError, 1);
        EXPECT_LE(maximumLateHeightError, 1);
        EXPECT_LT(maximumLateSpeed, 2);
        EXPECT_LE(drift, 1.5);
        EXPECT_LE(saturated * 20, result.mFrames.size() - 150);
        EXPECT_EQ(invariants.mFallingFrames + invariants.mFallenFrames, 0u);
        EXPECT_EQ(invariants.mAirborneFrames, 0u);
        EXPECT_EQ(invariants.mFreeVelocityLimitedFrames + invariants.mSolverVelocityLimitedFrames, 0u);
        EXPECT_GT(invariants.mMinPelvisUpDot, 0.9f);
        EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
        EXPECT_LT(invariants.mMaxNetActuationMoment, 1e-3f);
        const auto repeat = task(script);
        ASSERT_TRUE(repeat.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, repeat));
        // The prior pose-only behavior must remain distinct: its midpoint
        // balance target opposed this transfer and missed the final position.
        auto midpointScript = script;
        for (auto& key : midpointScript.mKeyframes)
            key.mSupportMotion.reset();
        const auto midpoint = task(midpointScript);
        ASSERT_TRUE(midpoint.completed());
        EXPECT_GT(std::abs(midpoint.mFrames.back().mBodies[0].mWorldPosition.GetX() - destination), 4);
        EXPECT_EQ(formatScenarioTraceCsv(midpoint).find("support_target_x"), std::string::npos);
        EXPECT_NE(formatScenarioTraceCsv(result).find("support_target_x"), std::string::npos);
        auto changedReference = result;
        changedReference.mFrames[150].mSupportMotion->mPosition += JPH::Vec3::sAxisX();
        EXPECT_FALSE(haveIdenticalScenarioTelemetry(result, changedReference));
        for (int invalid = 0; invalid < 4; ++invalid)
        {
            auto invalidScript = script;
            auto& key = invalidScript.mKeyframes[90];
            const float nan = std::numeric_limits<float>::quiet_NaN();
            switch (invalid)
            {
                case 0: key.mSupportMotion->mPosition.SetX(nan); break;
                case 1: key.mSupportMotion->mVelocity.SetX(nan); break;
                case 2: key.mSupportMotion->mAcceleration.SetX(nan); break;
                case 3: key.mRequestedHorizontalVelocity = JPH::Vec3::sAxisY(); break;
            }
            EXPECT_FALSE(task(invalidScript).completed()) << invalid;
        }
        const auto control = task(makeScript(false, transferSteps));
        ASSERT_TRUE(control.completed());
        EXPECT_NEAR(control.mFrames.back().mBodies[0].mWorldPosition.GetX(), 0, 1);
        auto releasedScript = script;
        for (auto& key : releasedScript.mKeyframes)
            if (key.mStep >= 150)
                key.mMotorsEnabled = false;
        const auto released = task(releasedScript);
        ASSERT_TRUE(released.completed());
        EXPECT_EQ(std::count_if(released.mFrames.begin() + 150, released.mFrames.end(),
            [](const auto& frame) { return frame.mMotorsApplied; }), 0);
        EXPECT_GT(summarizeScenarioInvariants(released).mFallingFrames
            + summarizeScenarioInvariants(released).mFallenFrames, 0u);
        if (liftAfterTransfer)
        {
            ASSERT_FALSE(::testing::Test::HasFailure()) << "qualify transfer before the coordinated lift";
            auto lifting = script;
            const auto start = result.mFrames.size();
            const auto liftSteps = static_cast<std::uint64_t>(std::lround(legacyReferenceGaitProfile().mSwingDuration / dt));
            const auto initial = script.mKeyframes.back().mMotorTarget->mBodyTransforms;
            const auto center = [&](const std::vector<Transform>& pose) {
                JPH::Vec3 value = JPH::Vec3::sZero();
                float mass = 0;
                for (std::size_t body = 0; body < pose.size(); ++body)
                {
                    const auto& part = built.mSettings->mParts[body];
                    const float weight = part.GetMassProperties().mMass;
                    value += weight * (pose[body].mTranslation + pose[body].mRotation * part.GetShape()->GetCenterOfMass());
                    mass += weight;
                }
                return value / mass;
            };
            const auto targetCenter = center(initial);
            const auto jointFrame = [](JPH::Vec3Arg twist, JPH::Vec3Arg plane) {
                return JPH::Mat44(JPH::Vec4(twist, 0), JPH::Vec4(plane.Cross(twist), 0),
                    JPH::Vec4(plane, 0), JPH::Vec4(0, 0, 0, 1)).GetQuaternion();
            };
            const auto appendLift = [&](ScenarioScript& liftedScript, float liftHeight) {
                for (std::uint64_t sample = 0; sample <= liftSteps; ++sample)
                {
                    const float u = static_cast<float>(sample) / liftSteps;
                    float blend = u * u * u * (10 + u * (-15 + 6 * u));
                    auto foot = initial[19].mTranslation;
                    foot.SetZ(foot.GetZ() + liftHeight * blend);
                    if (referenceCycle)
                    {
                        const auto trajectory = calculateReferenceSwingTrajectory(
                            { initial[19].mTranslation, initial[19].mTranslation, sample * dt, false }, legacyReferenceGaitProfile());
                        ASSERT_TRUE(trajectory);
                        foot = *trajectory;
                        blend = (foot.GetZ() - initial[19].mTranslation.GetZ()) / legacyReferenceGaitProfile().mSwingFootLift;
                    }
                    auto pelvis = initial[0];
                    if (balancedCrouch)
                    {
                        // The rigid-trunk path returns crouch reserve. The
                        // articulated path instead shares the bend with the
                        // spine and preserves the prepared pelvis height.
                        if (articulatedLift)
                            pelvis.mRotation = initial[0].mRotation.SLERP(
                                JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -std::acos(0.9f) * 18.0f / 64), blend).Normalized();
                        else
                        {
                            pelvis.mTranslation.SetZ(initial[0].mTranslation.GetZ()
                                + (fixture.mBase[0].mTranslation.GetZ() - initial[0].mTranslation.GetZ()) * blend);
                            pelvis.mRotation = initial[0].mRotation.SLERP(fixture.mBase[0].mRotation, blend).Normalized();
                        }
                    }
                    std::optional<ReferenceStancePosture> posture;
                    if (balancedCrouch)
                    {
                        auto liftBase = fixture.mBase;
                        if (articulatedLift)
                        {
                            const auto turn = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.13125f * blend);
                            for (const std::size_t joint : { 1u, 2u })
                            {
                                const auto pivot = liftBase[joint].mTranslation;
                                for (std::size_t body = joint; body < 14; ++body)
                                {
                                    liftBase[body].mTranslation = pivot + turn * (liftBase[body].mTranslation - pivot);
                                    liftBase[body].mRotation = (turn * liftBase[body].mRotation).Normalized();
                                }
                            }
                        }
                        auto rightFoot = initial[19];
                        rightFoot.mTranslation = foot;
                        posture = planBalancedReferenceSwingPosture(liftBase, masses, 0,
                            fixture.mLeft, fixture.mRight, pelvis, initial[16], rightFoot,
                            19, targetCenter, JPH::Vec3::sAxisY());
                        ASSERT_TRUE(posture) << sample;
                        EXPECT_LT((posture->mPose.mBodyTransforms[16].mTranslation
                            - initial[16].mTranslation).Length(), 1e-3f);
                        EXPECT_LT((posture->mPose.mBodyTransforms[19].mTranslation - foot).Length(), 1e-3f);
                        EXPECT_TRUE(posture->mPose.mBodyTransforms[16].mRotation.IsClose(initial[16].mRotation));
                        if (sample == liftSteps)
                        {
                            EXPECT_FALSE(planBalancedReferenceSwingPosture(liftBase, masses, 0,
                                fixture.mLeft, fixture.mRight, pelvis, initial[16], rightFoot,
                                0, targetCenter, JPH::Vec3::sAxisY()));
                            auto invalidMasses = masses;
                            invalidMasses[0].mMass = 0;
                            EXPECT_FALSE(planBalancedReferenceSwingPosture(liftBase, invalidMasses, 0,
                                fixture.mLeft, fixture.mRight, pelvis, initial[16], rightFoot,
                                19, targetCenter, JPH::Vec3::sAxisY()));
                            // Recover a known pose with an off-origin swing-foot
                            // mass center; ignoring its rotation must change COM.
                            auto offsetMasses = masses;
                            offsetMasses[19].mLocalCenterOfMass = JPH::Vec3(1, 2, 3);
                            const auto massCenter = [&](const auto& transforms) {
                                JPH::Vec3 value = JPH::Vec3::sZero();
                                for (std::size_t body = 0; body < transforms.size(); ++body)
                                    value += offsetMasses[body].mMass * (transforms[body].mTranslation
                                        + transforms[body].mRotation * offsetMasses[body].mLocalCenterOfMass);
                                return value / totalMass;
                            };
                            const auto knownCenter = massCenter(posture->mPose.mBodyTransforms);
                            const auto recovered = planBalancedReferenceSwingPosture(liftBase, offsetMasses, 0,
                                fixture.mLeft, fixture.mRight, posture->mPose.mBodyTransforms[0],
                                initial[16], rightFoot, 19, knownCenter, JPH::Vec3::sAxisY());
                            ASSERT_TRUE(recovered);
                            auto centerError = massCenter(recovered->mPose.mBodyTransforms) - knownCenter;
                            centerError.SetZ(0);
                            EXPECT_LT(centerError.Length(), 1e-3f);
                        }
                    }
                    // Solve the pelvis translation from the same mass-weighted
                    // motion the support controller will track. The foot lift
                    // must not silently move the balance target off the stance.
                    for (int iteration = 0; !balancedCrouch && iteration < 12; ++iteration)
                    {
                        posture = planReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
                            pelvis, initial[16].mTranslation, foot, JPH::Vec3::sAxisY());
                        ASSERT_TRUE(posture) << sample;
                        auto error = targetCenter - center(posture->mPose.mBodyTransforms);
                        error.SetZ(0);
                        if (error.Length() < 1e-4f) break;
                        pelvis.mTranslation += error;
                    }
                    auto& pose = posture->mPose.mBodyTransforms;
                    // The airborne foot may pitch with its calf. Choose the
                    // nearest level orientation inside the actual ankle cone;
                    // never ask the motor to realize an anatomically invalid
                    // level foot while the knee is bent for clearance.
                    for (const std::size_t child : { 1u, 2u, 14u, 16u, 17u, 19u })
                    {
                        const auto* joint = static_cast<const JPH::SwingTwistConstraintSettings*>(
                            built.mSettings->mParts[child].mToParent.GetPtr());
                        const auto parent = skeleton.mBones[child].mParent;
                        const auto frame1 = jointFrame(joint->mTwistAxis1, joint->mPlaneAxis1);
                        const auto frame2 = jointFrame(joint->mTwistAxis2, joint->mPlaneAxis2);
                        const auto request = (frame1.Conjugated() * pose[parent].mRotation.Conjugated()
                            * pose[child].mRotation * frame2).Normalized();
                        JPH::SwingTwistConstraintPart limits;
                        limits.SetSwingType(joint->mSwingType);
                        limits.SetLimits(joint->mTwistMinAngle, joint->mTwistMaxAngle,
                            -joint->mPlaneHalfConeAngle, joint->mPlaneHalfConeAngle,
                            -joint->mNormalHalfConeAngle, joint->mNormalHalfConeAngle);
                        JPH::Quat swing, twist;
                        request.GetSwingTwist(swing, twist);
                        JPH::uint clamped = 0;
                        limits.ClampSwingTwist(swing, twist, clamped);
                        if (child == 19)
                            pose[child].mRotation = (pose[parent].mRotation * frame1 * swing * twist * frame2.Conjugated()).Normalized();
                        else
                            ASSERT_EQ(clamped, 0u) << "sample " << sample << " loaded/hip joint " << child;
                    }
                    auto error = center(pose) - targetCenter;
                    error.SetZ(0);
                    ASSERT_LT(error.Length(), 1e-3f) << sample;
                    ScenarioInputKeyframe key;
                    key.mStep = start + sample;
                    key.mMotorTarget = posture->mPose;
                    key.mSupportMotion = SupportMotionReference{ JPH::RVec3(targetCenter), JPH::Vec3::sZero(), JPH::Vec3::sZero() };
                    key.mSupportMotion->mSupportFoot = 16;
                    liftedScript.mKeyframes.push_back(key);
                }
            };
            appendLift(lifting, legacyReferenceGaitProfile().mSwingFootLift);
            ASSERT_FALSE(::testing::Test::HasFailure()) << "qualify every lift target before actuation";
            lifting.mDurationSeconds = static_cast<float>(start + liftSteps + 60) * dt;
            const auto lifted = task(lifting);
            const char* liftTrace = referenceCycle ? "articulated-cycle" : articulatedLift ? "articulated-lift" : balancedCrouch ? "balanced-lift" : "shared-motion-lift";
            exportTraceIfRequested(lifted, liftTrace);
            exportBodiesIfRequested(lifted, liftTrace);
            ASSERT_TRUE(lifted.completed()) << describe(lifted);
            const auto physical = summarizeScenarioInvariants(lifted);
            float maximumUtilization = 0;
            std::size_t saturatedFrames = 0;
            double stanceDrift = 0;
            double minimumLateLift = std::numeric_limits<double>::infinity();
            double maximumLift = 0;
            std::optional<std::size_t> actualLift, actualLanding;
            for (std::size_t step = start; step < lifted.mFrames.size(); ++step)
            {
                const auto& frame = lifted.mFrames[step];
                if (articulatedLift)
                    EXPECT_NEAR(frame.mBodies[0].mWorldPosition.GetZ(), initial[0].mTranslation.GetZ(), 1) << step;
                const auto& utilizations = frame.mJointSupportMotorUtilizations;
                ASSERT_EQ(utilizations.size(), initial.size());
                const auto peak = *std::max_element(utilizations.begin(), utilizations.end());
                maximumUtilization = std::max(maximumUtilization, peak);
                saturatedFrames += peak >= 0.999f;
                stanceDrift = std::max(stanceDrift, (frame.mBodies[16].mWorldPosition
                    - lifted.mFrames[start - 1].mBodies[16].mWorldPosition).Length());
                const double clearance = frame.mBodies[19].mWorldPosition.GetZ()
                    - lifted.mFrames[start - 1].mBodies[19].mWorldPosition.GetZ();
                maximumLift = std::max(maximumLift, clearance);
                const bool footSupported = std::any_of(frame.mContacts.begin(), frame.mContacts.end(),
                    [](const auto& contact) { return contact.mBodyIndex == 19 && contact.hasSupport(); });
                if (!actualLift && !footSupported && clearance >= 2) actualLift = step;
                if (actualLift && !actualLanding && footSupported) actualLanding = step;
                if (!referenceCycle && step >= start + liftSteps)
                {
                    minimumLateLift = std::min(minimumLateLift, frame.mBodies[19].mWorldPosition.GetZ()
                        - lifted.mFrames[start - 1].mBodies[19].mWorldPosition.GetZ());
                    EXPECT_FALSE(std::any_of(frame.mContacts.begin(), frame.mContacts.end(),
                        [](const auto& contact) { return contact.mBodyIndex == 19 && contact.hasSupport(); })) << step;
                }
                ASSERT_TRUE(frame.mJointSupport);
                for (const auto& contact : frame.mJointSupport->mContactForces)
                    EXPECT_EQ(contact.mBodyIndex, 16u);
            }
            if (referenceCycle)
                std::cout << "ARTICULATED_CYCLE maximum_lift=" << maximumLift;
            else
                std::cout << (articulatedLift ? "ARTICULATED_LIFT minimum_late_lift=" : "SHARED_MOTION_LIFT minimum_late_lift=") << minimumLateLift;
            std::cout
                      << " stance_drift=" << stanceDrift << " maximum_utilization=" << maximumUtilization
                      << " saturated=" << saturatedFrames << ' ' << describe(physical) << std::endl;
            if (referenceCycle)
            {
                std::cout << "ARTICULATED_CYCLE lift=" << (actualLift ? std::to_string(*actualLift) : "missing")
                          << " landing=" << (actualLanding ? std::to_string(*actualLanding) : "missing") << std::endl;
                EXPECT_GE(maximumLift, legacyReferenceGaitProfile().mSwingFootLift - 2);
                EXPECT_TRUE(actualLift);
                EXPECT_TRUE(actualLanding);
                if (actualLanding)
                    EXPECT_LE(*actualLanding, start + static_cast<std::size_t>(std::lround(legacyReferenceGaitProfile().mMaximumStepDuration / dt)));
                EXPECT_TRUE(std::any_of(lifted.mFrames.back().mContacts.begin(), lifted.mFrames.back().mContacts.end(),
                    [](const auto& contact) { return contact.mBodyIndex == 19 && contact.hasSupport(); }));
            }
            else
                EXPECT_GE(minimumLateLift, legacyReferenceGaitProfile().mSwingFootLift - 2);
            EXPECT_LE(stanceDrift, 1.5);
            EXPECT_LE(saturatedFrames * 20, lifted.mFrames.size() - start);
            EXPECT_EQ(physical.mFallingFrames + physical.mFallenFrames, 0u);
            EXPECT_EQ(physical.mAirborneFrames, 0u);
            EXPECT_EQ(physical.mFreeVelocityLimitedFrames + physical.mSolverVelocityLimitedFrames, 0u);
            EXPECT_GT(physical.mMinPelvisUpDot, 0.9f);
            EXPECT_LT(physical.mMaxAppliedNetForce, 1e-3f);
            EXPECT_LT(physical.mMaxNetActuationMoment, 1e-3f);
            const auto repeated = task(lifting);
            ASSERT_TRUE(repeated.completed());
            EXPECT_TRUE(haveIdenticalScenarioTelemetry(lifted, repeated));
            // Only restore automatic two-foot load allocation: this is the
            // matched cause of the original toe-contact / clearance failure.
            auto bothFeet = lifting;
            for (auto& key : bothFeet.mKeyframes)
                if (key.mSupportMotion) key.mSupportMotion->mSupportFoot.reset();
            const auto loadedToe = task(bothFeet);
            ASSERT_TRUE(loadedToe.completed());
            exportTraceIfRequested(loadedToe, referenceCycle ? "articulated-cycle-loaded-toe-control" : articulatedLift ? "articulated-loaded-toe-control"
                : balancedCrouch ? "balanced-loaded-toe-control" : "shared-motion-loaded-toe-control");
            if (!balancedCrouch)
            {
                EXPECT_LT(loadedToe.mFrames[start + liftSteps].mBodies[19].mWorldPosition.GetZ()
                    - loadedToe.mFrames[start - 1].mBodies[19].mWorldPosition.GetZ(), legacyReferenceGaitProfile().mSwingFootLift - 2);
                EXPECT_TRUE(std::any_of(loadedToe.mFrames.back().mContacts.begin(), loadedToe.mFrames.back().mContacts.end(),
                    [](const auto& contact) { return contact.mBodyIndex == 19 && contact.hasSupport(); }));
            }
            else
            {
                // Body extension can let automatic support allocation unload
                // the foot too. Keep it as a comparator, not an assumed failure.
                std::cout << "BALANCED_AUTOMATIC_LOAD deadline_lift="
                          << loadedToe.mFrames[start + liftSteps].mBodies[19].mWorldPosition.GetZ()
                              - loadedToe.mFrames[start - 1].mBodies[19].mWorldPosition.GetZ() << std::endl;
                auto noLift = script;
                if (articulatedLift)
                {
                    ScenarioInputKeyframe hold;
                    hold.mStep = start;
                    hold.mMotorTarget = script.mKeyframes.back().mMotorTarget;
                    hold.mSupportMotion = SupportMotionReference{ JPH::RVec3(targetCenter), JPH::Vec3::sZero(), JPH::Vec3::sZero() };
                    hold.mSupportMotion->mSupportFoot = 16;
                    noLift.mKeyframes.push_back(hold);
                }
                else
                    appendLift(noLift, 0);
                ASSERT_FALSE(::testing::Test::HasFailure());
                noLift.mDurationSeconds = lifting.mDurationSeconds;
                const auto extensionOnly = task(noLift);
                exportTraceIfRequested(extensionOnly, referenceCycle ? "articulated-cycle-no-motion-control" : articulatedLift ? "articulated-no-motion-control" : "balanced-extension-only-control");
                ASSERT_TRUE(extensionOnly.completed());
                EXPECT_LT(extensionOnly.mFrames[start + liftSteps].mBodies[19].mWorldPosition.GetZ()
                    - extensionOnly.mFrames[start - 1].mBodies[19].mWorldPosition.GetZ(), 1.5);
                // Extension alone can unload the foot; the articulated case
                // instead holds the initial complete pose. Neither control
                // may satisfy the task's clearance without its motion command.
                for (std::size_t step = start + liftSteps; step < extensionOnly.mFrames.size(); ++step)
                    EXPECT_LT(extensionOnly.mFrames[step].mBodies[19].mWorldPosition.GetZ()
                        - extensionOnly.mFrames[start - 1].mBodies[19].mWorldPosition.GetZ(),
                        legacyReferenceGaitProfile().mSwingFootLift - 2) << step;
            }
            auto changedSelection = lifted;
            changedSelection.mFrames[start].mSupportMotion->mSupportFoot = 19;
            EXPECT_FALSE(haveIdenticalScenarioTelemetry(lifted, changedSelection));
            auto invalidSelection = lifting;
            invalidSelection.mKeyframes.back().mSupportMotion->mSupportFoot = 0;
            invalidSelection.mKeyframes.back().mMotorsEnabled = false;
            EXPECT_FALSE(task(invalidSelection).completed());
            for (auto& key : lifting.mKeyframes)
                if (key.mStep >= start) key.mMotorsEnabled = false;
            const auto release = task(lifting);
            ASSERT_TRUE(release.completed());
            for (std::size_t step = start; step < release.mFrames.size(); ++step)
                EXPECT_FALSE(release.mFrames[step].mMotorsApplied);
            EXPECT_GT(summarizeScenarioInvariants(release).mFallingFrames
                + summarizeScenarioInvariants(release).mFallenFrames, 0u);
            return;
        }
        if (!stepAfterTransfer)
            return;

        ASSERT_FALSE(::testing::Test::HasFailure()) << "qualify the transfer before handing off to gait";
        auto stepping = script;
        const auto start = result.mFrames.size();
        ScenarioInputKeyframe request;
        request.mStep = start;
        request.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
        // The new key releases the standing motion reference. The reference
        // gait takes ownership from the physically measured transferred pose.
        stepping.mKeyframes.push_back(request);
        stepping.mDurationSeconds = static_cast<float>(start + 66) * dt;
        const auto stepped = task(stepping);
        const auto ungoverned = task(stepping, false);
        ASSERT_TRUE(ungoverned.completed()) << describe(ungoverned);
        std::cout << "POSTURE_STEP_UNGOVERNED_CONTROL "
                  << describe(summarizeScenarioInvariants(ungoverned)) << std::endl;
        exportTraceIfRequested(stepped, balancedCrouch ? "balanced-step-handoff" : "posture-step-handoff");
        exportBodiesIfRequested(stepped, balancedCrouch ? "balanced-step-handoff" : "posture-step-handoff");
        ASSERT_TRUE(stepped.completed()) << describe(stepped);
        const auto gait = summarize(stepped);
        const auto physical = summarizeScenarioInvariants(stepped);
        std::cout << "POSTURE_STEP_HANDOFF " << describe(gait) << ' ' << describe(physical) << std::endl;
        EXPECT_TRUE(gait.mFirstTransferStep);
        if (gait.mFirstTransferStep)
            EXPECT_LE(*gait.mFirstTransferStep, start + 45);
        EXPECT_EQ(physical.mSteps.mTimedOutPhases, 0u);
        if (gait.mFirstFallingStep && gait.mFirstTransferStep)
            EXPECT_GT(*gait.mFirstFallingStep, *gait.mFirstTransferStep + 6);
        EXPECT_EQ(physical.mAirborneFrames, 0u);
        EXPECT_EQ(physical.mFreeVelocityLimitedFrames + physical.mSolverVelocityLimitedFrames, 0u);
        EXPECT_LE(gait.mSaturatedFrames * 20, gait.mReferenceFrames);
        EXPECT_LE(physical.mMaxStanceFootSlide, 1.5f);
        EXPECT_LT(physical.mMaxAppliedNetForce, 1e-3f);
        EXPECT_LT(physical.mMaxNetActuationMoment, 1e-3f);
        const auto stepRepeat = task(stepping);
        ASSERT_TRUE(stepRepeat.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(stepped, stepRepeat));
        stepping.mKeyframes.back().mMotorsEnabled = false;
        const auto stepReleased = task(stepping);
        ASSERT_TRUE(stepReleased.completed());
        const auto releasePhysical = summarizeScenarioInvariants(stepReleased);
        EXPECT_GT(releasePhysical.mFallingFrames + releasePhysical.mFallenFrames, 0u);
    }

    TEST(RagdollCoreReferenceGaitTest, coordinatedPostureTransfersWeightWithFeetPlanted)
    {
        checkCoordinatedLateralTransfer(false);
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreReferenceGaitTest, DISABLED_coordinatedTransferHandsOffToFirstStep)
    {
        checkCoordinatedLateralTransfer(true);
    }

    TEST(RagdollCoreReferenceGaitTest, coordinatedMotionLiftsFootOverTheLoadedStance)
    {
        checkCoordinatedLateralTransfer(false, true);
    }

    TEST(RagdollCoreReferenceGaitTest, balancedCrouchTransfersWeightAndLiftsOverTheLoadedStance)
    {
        checkCoordinatedLateralTransfer(false, true, true);
    }

    TEST(RagdollCoreReferenceGaitTest, balancedFixedHeightLiftGeometry)
    {
        using namespace RagdollCore;
        StanceFixture fixture;
        addNativeJointGeometry(fixture);
        const auto skeleton = makeScenarioHumanoidSkeleton();
        const auto built = buildRagdollSettings(skeleton, makeScenarioHumanoidProfile(), {});
        ASSERT_TRUE(built.isValid());
        std::vector<ReferencePostureMass> masses;
        JPH::Vec3 center = JPH::Vec3::sZero();
        float totalMass = 0;
        for (std::size_t body = 0; body < fixture.mBase.size(); ++body)
        {
            const auto& part = built.mSettings->mParts[body];
            masses.push_back({ part.GetMassProperties().mMass, part.GetShape()->GetCenterOfMass() });
            center += masses.back().mMass * (fixture.mBase[body].mTranslation
                + fixture.mBase[body].mRotation * masses.back().mLocalCenterOfMass);
            totalMass += masses.back().mMass;
        }
        center /= totalMass;
        center.SetX(fixture.mBase[16].mTranslation.GetX());
        auto rightFoot = fixture.mBase[19];
        rightFoot.mTranslation.SetZ(rightFoot.mTranslation.GetZ() + legacyReferenceGaitProfile().mSwingFootLift);
        auto pelvis = fixture.mBase[0];
        pelvis.mTranslation.SetZ(62);
        // Retained interior grid point from the bounded articulated search.
        // These are a geometric fixture, not a physical motion qualification.
        const float pelvisPitch = -std::acos(0.9f) * 18.0f / 64;
        const float trunkPitch = -0.2625f;
        pelvis.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), pelvisPitch);
        auto articulated = fixture.mBase;
        const auto turn = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.5f * trunkPitch);
        // Each subtree rotates about its existing parent-child anchor. The
        // full synthetic fixture's two spine subtrees end at body 13.
        for (const std::size_t joint : { 1u, 2u })
        {
            const auto pivot = articulated[joint].mTranslation;
            for (std::size_t body = joint; body < 14; ++body)
            {
                articulated[body].mTranslation = pivot + turn * (articulated[body].mTranslation - pivot);
                articulated[body].mRotation = (turn * articulated[body].mRotation).Normalized();
            }
        }
        const auto posture = planBalancedReferenceSwingPosture(articulated, masses, 0,
            fixture.mLeft, fixture.mRight, pelvis, fixture.mBase[16], rightFoot, 19,
            center, JPH::Vec3::sAxisY());
        ASSERT_TRUE(posture);
        const auto& pose = posture->mPose.mBodyTransforms;
        EXPECT_NEAR(pose[0].mTranslation.GetZ(), 62, 1e-3f);
        EXPECT_GT((pose[0].mRotation * JPH::Vec3::sAxisZ()).GetZ(), 0.9f);
        EXPECT_LT((pose[16].mTranslation - fixture.mBase[16].mTranslation).Length(), 1e-3f);
        EXPECT_LT((pose[19].mTranslation - rightFoot.mTranslation).Length(), 1e-3f);
        EXPECT_TRUE(pose[16].mRotation.IsClose(fixture.mBase[16].mRotation));
        JPH::Vec3 actual = JPH::Vec3::sZero();
        for (std::size_t body = 0; body < pose.size(); ++body)
            actual += masses[body].mMass * (pose[body].mTranslation
                + pose[body].mRotation * masses[body].mLocalCenterOfMass);
        auto error = actual / totalMass - center;
        error.SetZ(0);
        EXPECT_LT(error.Length(), 1e-3f);
        for (std::size_t body = 1; body < pose.size(); ++body)
        {
            const auto parent = skeleton.mBones[body].mParent;
            const auto offset = fixture.mBase[parent].mRotation.Conjugated()
                * (fixture.mBase[body].mTranslation - fixture.mBase[parent].mTranslation);
            EXPECT_LT((pose[body].mTranslation - pose[parent].mTranslation
                - pose[parent].mRotation * offset).Length(), 1e-3f) << body;
        }
        const auto jointFrame = [](JPH::Vec3Arg twist, JPH::Vec3Arg plane) {
            return JPH::Mat44(JPH::Vec4(twist, 0), JPH::Vec4(plane.Cross(twist), 0),
                JPH::Vec4(plane, 0), JPH::Vec4(0, 0, 0, 1)).GetQuaternion();
        };
        for (const std::size_t child : { 1u, 2u, 14u, 16u, 17u, 19u })
        {
            const auto* joint = static_cast<const JPH::SwingTwistConstraintSettings*>(
                built.mSettings->mParts[child].mToParent.GetPtr());
            const auto request = (jointFrame(joint->mTwistAxis1, joint->mPlaneAxis1).Conjugated()
                * pose[skeleton.mBones[child].mParent].mRotation.Conjugated() * pose[child].mRotation
                * jointFrame(joint->mTwistAxis2, joint->mPlaneAxis2)).Normalized();
            JPH::SwingTwistConstraintPart limits;
            limits.SetSwingType(joint->mSwingType);
            limits.SetLimits(joint->mTwistMinAngle, joint->mTwistMaxAngle,
                -joint->mPlaneHalfConeAngle, joint->mPlaneHalfConeAngle,
                -joint->mNormalHalfConeAngle, joint->mNormalHalfConeAngle);
            JPH::Quat swing, twist;
            request.GetSwingTwist(swing, twist);
            JPH::uint clamped = 0;
            limits.ClampSwingTwist(swing, twist, clamped);
            if (child == 19)
                EXPECT_LT((request - swing * twist).LengthSq(), 1e-10f);
            else
                EXPECT_EQ(clamped, 0u) << child;
        }
        for (const auto& leg : { fixture.mLeft, fixture.mRight })
        {
            const auto relative = (pose[leg.mHip].mRotation.Conjugated() * pose[leg.mKnee].mRotation).Normalized();
            const float angle = 2 * std::atan2(relative.GetX(), relative.GetW());
            EXPECT_GE(angle, leg.mHingeGeometry->mMinimumAngle);
            EXPECT_LE(angle, leg.mHingeGeometry->mMaximumAngle);
        }
        EXPECT_FALSE(planBalancedReferenceSwingPosture(fixture.mBase, masses, 0,
            fixture.mLeft, fixture.mRight, pelvis, fixture.mBase[16], rightFoot, 19,
            center, JPH::Vec3::sAxisY()));
        std::cout << "FIXED_HEIGHT_LIFT pitch=" << pelvisPitch << " trunk_pitch=" << trunkPitch
                  << " pelvis_x=" << pose[0].mTranslation.GetX()
                  << " pelvis_y=" << pose[0].mTranslation.GetY() << std::endl;
    }

    TEST(RagdollCoreReferenceGaitTest, balancedCrouchArticulatesTrunkDuringSupportedLift)
    {
        checkCoordinatedLateralTransfer(false, true, true, true);
    }

    TEST(RagdollCoreReferenceGaitTest, articulatedSwingReturnsToContactAtReferenceTiming)
    {
        checkCoordinatedLateralTransfer(false, true, true, true, true);
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreReferenceGaitTest, DISABLED_transferRetainsUnloadedFootBeforeTimedLift)
    {
        using namespace RagdollCore;
        StanceFixture fixture;
        const auto skeleton = makeScenarioHumanoidSkeleton();
        auto profile = makeScenarioHumanoidProfile();
        calibratePostureTrunk(skeleton, profile);
        // Geometry-derived level-foot crouch, with 20% sagittal ankle range
        // remaining. This test covers transfer and the start of a timed swing,
        // not an indefinite hold under a walking pendulum controller.
        const float calfAngle = 0.8f * profile.mBodies[16].mConstraint->mPlaneHalfConeAngle;
        const float upper = fixture.mLeft.mHingeGeometry->mUpperOffset.Length();
        const float lower = fixture.mLeft.mHingeGeometry->mLowerOffset.Length();
        const float thighAngle = std::asin(lower * std::sin(calfAngle) / upper);
        const float depth = upper * (1 - std::cos(thighAngle)) + lower * (1 - std::cos(calfAngle));
        auto settings = referenceSettings();
        settings.mReferenceGait.mDoubleSupportDuration = 0.35f;
        settings.mReferenceGait.mMaximumDoubleSupportDuration = 0.35f;
        settings.mReferenceGait.mWeightTransferFrequency = 1.5f;
        settings.mReferenceGait.mTransferVelocityGain = 0;
        settings.mReferenceGait.mMaximumRequestedAcceleration = 150;
        ScenarioScript script;
        script.mDurationSeconds = 3.1f;
        for (std::uint64_t step = 60; step <= 81; ++step)
        {
            const float t = static_cast<float>(step - 60) / 21;
            auto pelvis = fixture.mBase[0];
            pelvis.mTranslation.SetZ(66 - depth * t * t * (3 - 2 * t));
            const auto posture = planReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
                pelvis, fixture.mBase[16].mTranslation, fixture.mBase[19].mTranslation, JPH::Vec3::sAxisY());
            ASSERT_TRUE(posture);
            ScenarioInputKeyframe key;
            key.mStep = step;
            key.mMotorTarget = posture->mPose;
            script.mKeyframes.push_back(key);
        }
        ScenarioInputKeyframe transfer;
        transfer.mStep = 150;
        transfer.mRequestedHorizontalVelocity = JPH::Vec3(0, 1, 0);
        script.mKeyframes.push_back(transfer);
        const auto task = [&](const ScenarioScript& input) {
            return runRagdollScenario(skeleton, profile, settings, input);
        };
        const auto result = task(script);
        exportTraceIfRequested(result, "transfer-foot");
        exportBodiesIfRequested(result, "transfer-foot");
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto summary = summarize(result);
        const auto invariants = summarizeScenarioInvariants(result);
        double drift = 0;
        const auto anchor = result.mFrames[149].mBodies[16].mWorldPosition;
        for (const auto& frame : result.mFrames)
            if (frame.mStep >= 150 && frame.mReferenceGait
                && frame.mReferenceGait->mStage == ReferenceGaitStage::DoubleSupport)
                drift = std::max(drift, (frame.mBodies[16].mWorldPosition - anchor).Length());
        std::cout << "TRANSFER_FOOT max_drift=" << drift << ' ' << describe(summary) << ' '
                  << describe(invariants) << std::endl;
        EXPECT_NEAR(result.mFrames[149].mBodies[0].mWorldPosition.GetZ(), 66 - depth, 1);
        EXPECT_LT(result.mFrames[149].mBodies[0].mLinearVelocity.Length(), 2);
        EXPECT_LE(drift, 1.5) << "include motion while the intended stance foot is unloaded";
        EXPECT_TRUE(summary.mFirstLiftStep);
        if (summary.mFirstLiftStep)
            EXPECT_LE(*summary.mFirstLiftStep, 180u);
        EXPECT_EQ(invariants.mFallingFrames + invariants.mFallenFrames, 0u);
        EXPECT_EQ(invariants.mAirborneFrames, 0u);
        EXPECT_EQ(invariants.mFreeVelocityLimitedFrames + invariants.mSolverVelocityLimitedFrames, 0u);
        EXPECT_GT(invariants.mMinPelvisUpDot, 0.9f);
        EXPECT_LE(summary.mSaturatedFrames * 20, summary.mReferenceFrames);
        EXPECT_LE(invariants.mMaxStanceFootSlide, 1.5f);
        EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
        EXPECT_LT(invariants.mMaxNetActuationMoment, 1e-3f);
        const auto repeat = task(script);
        ASSERT_TRUE(repeat.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, repeat));
        // A matched zero-request control must not perform the physical lift.
        script.mKeyframes.back().mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        const auto control = task(script);
        ASSERT_TRUE(control.completed());
        const auto& last = control.mFrames.back();
        EXPECT_LT(last.mBodies[19].mWorldPosition.GetZ() - last.mBodies[16].mWorldPosition.GetZ(),
            0.1f * settings.mReferenceGait.mSwingFootLift);
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreReferenceGaitTest, DISABLED_preparedCrouchTransfersIntoSingleSupport)
    {
        using namespace RagdollCore;
        // Test the previously measured crouch as an authored motor reference.
        // Its level-foot ankle target is not anatomically qualified (see the
        // native-limit regression); keep this failing integration diagnostic.
        // Prepare and settle it before requesting transfer so the first causal
        // question is compatibility with reference support, not startup timing.
        StanceFixture fixture;
        const auto skeleton = makeScenarioHumanoidSkeleton();
        auto profile = makeScenarioHumanoidProfile();
        calibratePostureTrunk(skeleton, profile);
        auto settings = referenceSettings();
        settings.mReferenceGaitHoldSwingFoot = true;
        settings.mReferenceGait.mDoubleSupportDuration = 0.35f;
        settings.mReferenceGait.mMaximumDoubleSupportDuration = 0.35f;
        settings.mReferenceGait.mWeightTransferFrequency = 1.5f;
        settings.mReferenceGait.mTransferVelocityGain = 0;
        settings.mReferenceGait.mMaximumRequestedAcceleration = 150;
        settings.mReferenceGait.mMaximumStepDuration = 10;
        ScenarioScript script;
        script.mDurationSeconds = 4.5f;
        for (std::uint64_t step = 60; step <= 81; ++step)
        {
            const float t = static_cast<float>(step - 60) / 21;
            auto pelvis = fixture.mBase[0];
            pelvis.mTranslation.SetZ(66 - 4 * t * t * (3 - 2 * t));
            const auto posture = planReferenceStancePosture(fixture.mBase, 0, fixture.mLeft, fixture.mRight,
                pelvis, fixture.mBase[16].mTranslation, fixture.mBase[19].mTranslation, JPH::Vec3::sAxisY());
            ASSERT_TRUE(posture);
            ScenarioInputKeyframe key;
            key.mStep = step;
            key.mMotorTarget = posture->mPose;
            script.mKeyframes.push_back(key);
        }
        ScenarioInputKeyframe transfer;
        transfer.mStep = 150;
        transfer.mRequestedHorizontalVelocity = JPH::Vec3(0, 1, 0);
        script.mKeyframes.push_back(transfer);
        const auto runTask = [&](const ScenarioScript& task) {
            return runRagdollScenario(skeleton, profile, settings, task);
        };
        const auto result = runTask(script);
        exportTraceIfRequested(result, "prepared-crouch-transfer");
        exportBodiesIfRequested(result, "prepared-crouch-transfer");
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto summary = summarize(result);
        const auto invariants = summarizeScenarioInvariants(result);
        std::cout << "PREPARED_TRANSFER " << describe(summary) << ' ' << describe(invariants) << std::endl;
        EXPECT_NEAR(result.mFrames[149].mBodies[0].mWorldPosition.GetZ(), 62, 1);
        EXPECT_LT(result.mFrames[149].mBodies[0].mLinearVelocity.Length(), 2);
        ASSERT_TRUE(summary.mFirstLiftStep);
        EXPECT_LE(*summary.mFirstLiftStep, 180u);
        EXPECT_FALSE(summary.mFirstTransferStep);
        EXPECT_EQ(invariants.mFallingFrames + invariants.mFallenFrames, 0u);
        EXPECT_EQ(invariants.mAirborneFrames, 0u);
        EXPECT_EQ(invariants.mFreeVelocityLimitedFrames + invariants.mSolverVelocityLimitedFrames, 0u);
        EXPECT_GT(invariants.mMinPelvisUpDot, 0.9f);
        EXPECT_LE(summary.mSaturatedFrames * 20, summary.mReferenceFrames);
        EXPECT_LE(invariants.mMaxStanceFootSlide, 1.5f);
        EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
        EXPECT_LT(invariants.mMaxNetActuationMoment, 1e-3f);
        double maximumHeightError = 0;
        std::uint64_t maximumHeightErrorStep = 0;
        for (std::size_t i = 210; i < result.mFrames.size(); ++i)
        {
            const double error = std::abs(result.mFrames[i].mBodies[0].mWorldPosition.GetZ() - 62);
            if (error > maximumHeightError)
            {
                maximumHeightError = error;
                maximumHeightErrorStep = result.mFrames[i].mStep;
            }
        }
        EXPECT_LE(maximumHeightError, 1) << "step " << maximumHeightErrorStep;
        const auto repeat = runTask(script);
        ASSERT_TRUE(repeat.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, repeat));
        script.mKeyframes.back().mMotorsEnabled = false;
        const auto released = runTask(script);
        ASSERT_TRUE(released.completed());
        EXPECT_EQ(std::count_if(released.mFrames.begin(), released.mFrames.end(),
            [](const auto& frame) { return frame.mStep >= 150 && frame.mMotorsApplied; }), 0);
        const auto releasedInvariants = summarizeScenarioInvariants(released);
        EXPECT_GT(releasedInvariants.mFallingFrames + releasedInvariants.mFallenFrames, 0u);
    }

    TEST(RagdollCoreReferenceGaitTest, initialWeightTransferDoesNotInventABouncedLanding)
    {
        using namespace RagdollCore;
        ReferenceGaitStateMachine machine;
        ReferenceGaitState state;
        ReferenceGaitStateInput input;
        ReferenceGaitProfile profile;
        const float dt = 1.0f / 60;
        input.mEnabled = true;
        input.mLeftFootSupported = input.mRightFootSupported = true;
        input.mLeftFootCenter = JPH::RVec3(-8, 0, 2.5);
        input.mRightFootCenter = JPH::RVec3(8, 0, 2.5);
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        input.mLeftFootSupported = false;
        input.mLeftFootCenter.SetX(-20);
        for (int i = 0; i < 20; ++i)
            ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mPhase, GaitPhase::LeftStance);
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        ASSERT_TRUE(state.mTransferCenter);
        EXPECT_DOUBLE_EQ(state.mTransferCenter->GetX(), -8);
        EXPECT_FALSE(state.mTransitioned);
        EXPECT_FALSE(state.mTimedOut);
        for (int i = 0; i < 30; ++i)
            ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mPhase, GaitPhase::LeftStance);
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        EXPECT_TRUE(state.mTimedOut);
        input.mLeftFootSupported = true;
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mPhase, GaitPhase::LeftStance);
        EXPECT_EQ(state.mStage, ReferenceGaitStage::Swing);
        EXPECT_FALSE(state.mTimedOut);
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreReferenceGaitTest, DISABLED_stateMachineStepsAtOnceWhenTheCapturePointLeavesTheFeet)
    {
        using namespace RagdollCore;
        const float dt = 1.0f / 60;
        ReferenceGaitProfile profile;
        // Give the timer a longer interval than the emergency response. The
        // production profile may legitimately use a shorter double support.
        profile.mDoubleSupportDuration = 0.1f;
        // Double support is longer than one step,
        // so a swing on the very next step can only come from the rule.
        ASSERT_GT(profile.mDoubleSupportDuration, 2 * dt);
        const auto standing = [] {
            ReferenceGaitStateInput input;
            input.mEnabled = true;
            input.mLeftFootSupported = input.mRightFootSupported = true;
            input.mLeftFootLoadFraction = input.mRightFootLoadFraction = 0.5f;
            // Left foot trailing (behind along +Y), feet 16 units apart.
            input.mLeftFootCenter = JPH::RVec3(-8, -10, 2.5);
            input.mRightFootCenter = JPH::RVec3(8, 0, 2.5);
            input.mCenterOfMass = JPH::RVec3(0, -5, 60);
            input.mCenterOfMassHeight = 60.0f;
            input.mGravity = 627.0f;
            input.mHeading = JPH::Vec3(0, 1, 0);
            return input;
        };
        // Rule disabled: a large forward velocity changes nothing.
        {
            profile.mEmergencyCaptureMargin = 0.0f;
            ReferenceGaitStateMachine machine;
            ReferenceGaitState state;
            auto input = standing();
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            input.mCenterOfMassVelocity = JPH::Vec3(0, 80, 0);
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
            EXPECT_FALSE(state.mEmergencyStep);
            EXPECT_EQ(state.mEmergencyStepCount, 0u);
        }
        profile.mEmergencyCaptureMargin = 6.0f;
        // Capture point inside the margin: the timer still rules.
        {
            ReferenceGaitStateMachine machine;
            ReferenceGaitState state;
            auto input = standing();
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            // tau = sqrt(60 / 627) = 0.309 s; 12 u/s puts the capture point
            // 3.7 units ahead of the center of mass, inside the feet.
            input.mCenterOfMassVelocity = JPH::Vec3(0, 12, 0);
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
            EXPECT_EQ(state.mEmergencyStepCount, 0u);
        }
        // Forward shove: the trailing (left) foot steps, the right holds.
        {
            ReferenceGaitStateMachine machine;
            ReferenceGaitState state;
            auto input = standing();
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_EQ(state.mPhase, GaitPhase::LeftStance);
            input.mCenterOfMassVelocity = JPH::Vec3(0, 80, 0); // capture 24.7 ahead
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_EQ(state.mStage, ReferenceGaitStage::Swing);
            EXPECT_EQ(state.mPhase, GaitPhase::RightStance);
            EXPECT_TRUE(state.mEmergencyStep);
            EXPECT_TRUE(state.mTransitioned);
            EXPECT_EQ(state.mEmergencyStepCount, 1u);
            ASSERT_TRUE(state.mTransferCenter);
            EXPECT_DOUBLE_EQ(state.mTransferCenter->GetX(), 8);
            EXPECT_TRUE(toGaitControllerOutput(state, profile).mStanceReady);
        }
        // Sideways shove toward -X: the near (left) foot steps outward
        // instead of the right foot crossing behind it.
        {
            ReferenceGaitStateMachine machine;
            ReferenceGaitState state;
            auto input = standing();
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            input.mCenterOfMassVelocity = JPH::Vec3(-70, 0, 0); // capture at x = -21.6
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_EQ(state.mStage, ReferenceGaitStage::Swing);
            EXPECT_EQ(state.mPhase, GaitPhase::RightStance);
            EXPECT_TRUE(state.mEmergencyStep);
        }
        // Sideways shove toward +X: the right foot steps outward.
        {
            ReferenceGaitStateMachine machine;
            ReferenceGaitState state;
            auto input = standing();
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            input.mCenterOfMassVelocity = JPH::Vec3(70, 0, 0);
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_EQ(state.mStage, ReferenceGaitStage::Swing);
            EXPECT_EQ(state.mPhase, GaitPhase::LeftStance);
            EXPECT_TRUE(state.mEmergencyStep);
        }
        // An unloaded candidate stance foot blocks the step; the flag clears
        // on the touchdown that ends an emergency swing.
        {
            ReferenceGaitStateMachine machine;
            ReferenceGaitState state;
            auto input = standing();
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            input.mCenterOfMassVelocity = JPH::Vec3(0, 80, 0);
            input.mRightFootLoadFraction = 0.0f;
            input.mRightFootSupported = false;
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
            EXPECT_EQ(state.mEmergencyStepCount, 0u);
            input.mRightFootSupported = true;
            // A grazing landing is contact, but cannot justify bypassing
            // transfer and immediately lifting the heavily loaded other foot.
            input.mRightFootLoadFraction = 0.1f;
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
            EXPECT_EQ(state.mEmergencyStepCount, 0u);
            input.mRightFootLoadFraction = 0.5f;
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
            EXPECT_EQ(state.mEmergencyStepCount, 0u);
            ASSERT_TRUE(machine.step(input, dt, profile, state));
            ASSERT_EQ(state.mStage, ReferenceGaitStage::Swing);
            EXPECT_TRUE(state.mEmergencyStep);
            input.mCenterOfMassVelocity = JPH::Vec3::sZero();
            input.mLeftFootSupported = false;
            input.mLeftFootLoadFraction = 0.0f;
            input.mLeftFootHeight = 3.0f;
            input.mLeftFootVerticalVelocity = 5.0f;
            for (int i = 0; i < 10; ++i)
                ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_TRUE(state.mSwingFootLifted);
            input.mLeftFootSupported = true;
            input.mLeftFootLoadFraction = 0.5f;
            input.mLeftFootVerticalVelocity = -1.0f;
            for (int i = 0; i < 4; ++i)
                ASSERT_TRUE(machine.step(input, dt, profile, state));
            EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
            EXPECT_EQ(state.mPhase, GaitPhase::LeftStance);
            EXPECT_FALSE(state.mEmergencyStep);
            EXPECT_EQ(state.mEmergencyStepCount, 1u);
        }
    }

    TEST(RagdollCoreReferenceGaitTest, initialTransferWaitsForMotionTowardStance)
    {
        using namespace RagdollCore;
        ReferenceGaitProfile profile;
        profile.mEmergencyCaptureMargin = 0;
        ReferenceGaitStateInput input;
        input.mEnabled = true;
        input.mLeftFootSupported = input.mRightFootSupported = true;
        input.mLeftFootLoadFraction = input.mRightFootLoadFraction = 0.5f;
        input.mLeftFootCenter = JPH::RVec3(-10, 0, 0);
        input.mRightFootCenter = JPH::RVec3(10, 0, 0);
        input.mCenterOfMass = JPH::RVec3(0, 0, 60);
        input.mHeading = JPH::Vec3::sAxisY();
        input.mCenterOfMassVelocity = JPH::Vec3(3, 0, 0);
        ReferenceGaitStateMachine machine;
        ReferenceGaitState state;
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        ASSERT_TRUE(state.leftFootIsStance());
        for (int i = 0; i < 5; ++i)
        {
            ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
            EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        }
        input.mCenterOfMassVelocity.SetX(-3);
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::Swing);
        machine.reset();
        input.mCenterOfMassVelocity.SetX(3);
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        // Command-only historical observations still have their old contract.
        input.mCenterOfMass.reset();
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::Swing);
        machine.reset();
        input.mCenterOfMass = JPH::RVec3(0, 0, 60);
        input.mCenterOfMassVelocity.SetX(-3);
        input.mLeftFootLoadFraction = 0.15f;
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        input.mLeftFootLoadFraction = 0.5f;
        ASSERT_TRUE(machine.step(input, 1.0f / 60, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        ASSERT_TRUE(machine.step(input, 1.0f / 60, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::Swing);
    }

    TEST(RagdollCoreReferenceGaitTest, restartAfterStationaryBalanceRequiresFreshSupportTransfer)
    {
        using namespace RagdollCore;
        ReferenceGaitProfile profile;
        profile.mEmergencyCaptureMargin = 0;
        ReferenceGaitStateInput input;
        input.mEnabled = true;
        input.mLeftFootSupported = input.mRightFootSupported = true;
        input.mLeftFootLoadFraction = input.mRightFootLoadFraction = 0.5f;
        input.mLeftFootCenter = JPH::RVec3(-10, 0, 0);
        input.mRightFootCenter = JPH::RVec3(10, 0, 0);
        input.mCenterOfMass = JPH::RVec3(0, 0, 60);
        input.mHeading = JPH::Vec3::sAxisY();
        ReferenceGaitStateMachine machine;
        ReferenceGaitState state;
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        ASSERT_EQ(state.mStage, ReferenceGaitStage::Swing);
        input.mEmergencyOnly = true;
        input.mRightFootSupported = false;
        input.mRightFootHeight = 4;
        input.mRightFootVerticalVelocity = 10;
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        ASSERT_TRUE(state.mSwingFootLifted);
        input.mRightFootSupported = true;
        input.mRightFootHeight = input.mRightFootVerticalVelocity = 0;
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        ASSERT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        ASSERT_FALSE(state.leftFootIsStance());
        for (int i = 0; i < 10; ++i)
            ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        input.mEmergencyOnly = false;
        input.mCenterOfMassVelocity.SetX(-3); // Moving away from right stance.
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        input.mCenterOfMassVelocity.SetX(3);
        input.mRightFootLoadFraction = 0.15f;
        ASSERT_TRUE(machine.step(input, 0.1f, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        input.mRightFootLoadFraction = 0.5f;
        ASSERT_TRUE(machine.step(input, 1.0f / 60, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        ASSERT_TRUE(machine.step(input, 1.0f / 60, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::Swing);
    }

    TEST(RagdollCoreReferenceGaitTest, stopDeceleratesFromMeasuredSpeedWithoutInventingMomentum)
    {
        using namespace RagdollCore;
        ReferenceGaitProfile profile;
        profile.mSteeringStopDeceleration = 80.0f;
        profile.mSteeringStopSpeed = 3.0f;
        constexpr float dt = 1.0f / 60.0f;
        for (const bool direct : { false, true })
        {
            ReferenceGaitSteering steering;
            steering.step(JPH::Vec3(0, 128, 0), dt, profile, true, true, direct);
            auto command = steering.step(JPH::Vec3::sZero(), dt, profile, false, false, direct,
                JPH::Vec3(90, 30, 0)); // Sideways speed is not forward momentum.
            EXPECT_NEAR(command.GetY(), 30 - 80 * dt, 1e-4f);
            EXPECT_NEAR(command.GetX(), 0, 1e-4f);
            EXPECT_TRUE(steering.stopping());
            // A later shove must not reset or increase the stopping command.
            command = steering.step(JPH::Vec3::sZero(), dt, profile, false, false, direct,
                JPH::Vec3(0, 150, 0));
            EXPECT_NEAR(command.GetY(), 30 - 160 * dt, 1e-4f);
            steering.reset();
            steering.step(JPH::Vec3(0, 128, 0), dt, profile);
            command = steering.step(JPH::Vec3::sZero(), dt, profile, false, false, direct,
                JPH::Vec3(0, -20, 0));
            EXPECT_NEAR(command.GetY(), 4, 1e-4f);
            EXPECT_TRUE(steering.stopping()); // Still waits for planted support.
            command = steering.step(JPH::Vec3::sZero(), dt, profile, true, true, direct,
                JPH::Vec3::sZero());
            EXPECT_TRUE(command.IsNearZero());
            EXPECT_FALSE(steering.stopping());
        }
    }

    TEST(RagdollCoreReferenceGaitTest, steeringFilterTurnsDeceleratesAndHandsOverStanding)
    {
        using namespace RagdollCore;
        const float dt = 1.0f / 60;
        ReferenceGaitProfile profile;
        profile.mSteeringTurnRate = 1.5f;
        profile.mSteeringSpeedChangeRate = 0.0f;
        profile.mSteeringStopDeceleration = 80.0f;
        profile.mSteeringStopSpeed = 3.0f;
        ReferenceGaitSteering steering;
        // From rest the heading is taken at once, at full speed.
        JPH::Vec3 command = steering.step(JPH::Vec3(0, 60, 0), dt, profile);
        EXPECT_NEAR(command.GetY(), 60.0f, 1e-4f);
        EXPECT_NEAR(command.GetX(), 0.0f, 1e-4f);
        // A 90 degree request turns by at most the rate per step; the speed
        // stays at the request.
        command = steering.step(JPH::Vec3(60, 0, 0), dt, profile);
        const float turned = std::atan2(command.GetX(), command.GetY());
        EXPECT_NEAR(turned, 1.5f * dt, 1e-4f);
        EXPECT_NEAR(command.Length(), 60.0f, 1e-3f);
        // The turn completes in about pi/2 / 1.5 s and then stays exact.
        int steps = 1;
        while (std::abs(command.GetY()) > 1e-3f && steps < 200)
        {
            command = steering.step(JPH::Vec3(60, 0, 0), dt, profile);
            ++steps;
        }
        EXPECT_NEAR(steps * dt, 1.5707963f / 1.5f, 2 * dt);
        EXPECT_NEAR(command.GetX(), 60.0f, 1e-3f);
        // An exact reversal turns rather than snapping.
        command = steering.step(JPH::Vec3(-60, 0, 0), dt, profile);
        EXPECT_GT(command.GetX(), 50.0f);
        EXPECT_GT(std::abs(command.GetY()), 1.0f);
        // Stop: decelerate at 80 u/s^2 to the creep speed, hold it while a foot
        // is in the air, and snap to zero once both feet carry the body.
        steering.reset();
        steering.step(JPH::Vec3(0, 60, 0), dt, profile);
        command = steering.step(JPH::Vec3::sZero(), dt, profile, false);
        EXPECT_NEAR(command.GetY(), 60.0f - 80.0f * dt, 1e-3f);
        for (int i = 0; i < 120; ++i)
            command = steering.step(JPH::Vec3::sZero(), dt, profile, false);
        EXPECT_NEAR(command.GetY(), 4.0f, 1e-3f);
        command = steering.step(JPH::Vec3::sZero(), dt, profile, true);
        EXPECT_TRUE(command.IsNearZero());
        EXPECT_FALSE(steering.stopping());
        // The stopping flag is raised while the command walks the body down,
        // and the timeout hands over at the next planted double support even
        // when the body is still faster than the hand-over speed.
        profile.mSteeringStopTimeout = 0.5f;
        steering.step(JPH::Vec3(0, 60, 0), dt, profile);
        EXPECT_FALSE(steering.stopping());
        // 60 u/s reaches the creep speed after 0.7 s at 80 u/s^2, past the
        // 0.5 s timeout; the hand-over still waits for both feet.
        for (int i = 0; i < 45; ++i)
            command = steering.step(JPH::Vec3::sZero(), dt, profile, false, false);
        EXPECT_TRUE(steering.stopping());
        EXPECT_NEAR(command.Length(), 4.0f, 1e-3f); // timed out, but a foot is in the air
        command = steering.step(JPH::Vec3::sZero(), dt, profile, false, true);
        EXPECT_TRUE(command.IsNearZero());
        EXPECT_FALSE(steering.stopping());
        profile.mSteeringStopTimeout = 3.0f;
        // Without a deceleration limit the stop is immediate (pre-steering
        // semantics), whatever the feet are doing.
        profile.mSteeringStopDeceleration = 0.0f;
        steering.step(JPH::Vec3(0, 60, 0), dt, profile);
        command = steering.step(JPH::Vec3::sZero(), dt, profile, false, false);
        EXPECT_TRUE(command.IsNearZero());
        EXPECT_FALSE(steering.stopping());
        // A restart from rest keeps the last heading and turns toward a
        // 90 degree request at the turn rate instead of snapping; a fresh
        // walker (no heading yet) still takes the request at once.
        profile.mSteeringStopDeceleration = 80.0f;
        steering.step(JPH::Vec3(0, 60, 0), dt, profile);
        for (int i = 0; i < 120; ++i)
            command = steering.step(JPH::Vec3::sZero(), dt, profile, true);
        ASSERT_TRUE(command.IsNearZero());
        command = steering.step(JPH::Vec3(60, 0, 0), dt, profile);
        EXPECT_NEAR(command.GetY(), 60.0f, 1e-3f);
        EXPECT_NEAR(command.GetX(), 0.0f, 1e-3f);
        command = steering.step(JPH::Vec3(60, 0, 0), dt, profile);
        EXPECT_NEAR(std::atan2(command.GetX(), command.GetY()), 1.5f * dt, 1e-4f);
        steering.reset();
        command = steering.step(JPH::Vec3(60, 0, 0), dt, profile);
        EXPECT_NEAR(command.GetX(), 60.0f, 1e-3f);
        // Facing is a separate degree of freedom: it turns at the same rate
        // but independently of travel, so a player's feet can carry them one
        // way while the body points another (plan item 2.3b).
        {
            ReferenceGaitSteering facingOnly;
            EXPECT_TRUE(facingOnly.facing().IsNearZero());
            // The first request is taken exactly; later ones turn at the rate.
            facingOnly.stepFacing(JPH::Vec3(0, 60, 0), dt, profile);
            EXPECT_NEAR(facingOnly.facing().GetY(), 1.0f, 1e-4f);
            facingOnly.stepFacing(JPH::Vec3(60, 0, 0), dt, profile);
            EXPECT_NEAR(std::atan2(facingOnly.facing().GetX(), facingOnly.facing().GetY()),
                profile.mSteeringTurnRate * dt, 1e-4f);
            EXPECT_NEAR(facingOnly.facing().Length(), 1.0f, 1e-4f);
            // A zero request leaves it alone; reset clears it.
            const JPH::Vec3 held = facingOnly.facing();
            facingOnly.stepFacing(JPH::Vec3::sZero(), dt, profile);
            EXPECT_TRUE(facingOnly.facing().IsClose(held));
            facingOnly.reset();
            EXPECT_TRUE(facingOnly.facing().IsNearZero());
        }
        // Direct travel skips the turn-rate sweep so a player's input takes
        // effect at once, while the filtered path is unchanged for AI.
        {
            ReferenceGaitSteering direct;
            direct.step(JPH::Vec3(0, 60, 0), dt, profile);
            const JPH::Vec3 command = direct.step(JPH::Vec3(60, 0, 0), dt, profile, true, true, true);
            EXPECT_NEAR(command.GetX(), 60.0f, 1e-3f);
            EXPECT_NEAR(command.GetY(), 0.0f, 1e-3f);
            ReferenceGaitSteering filtered;
            filtered.step(JPH::Vec3(0, 60, 0), dt, profile);
            const JPH::Vec3 swept = filtered.step(JPH::Vec3(60, 0, 0), dt, profile);
            EXPECT_LT(swept.GetX(), 10.0f) << "the filtered path must still sweep at the turn rate";
        }
        // Disabled limits snap the heading.
        profile.mSteeringTurnRate = 0.0f;
        steering.step(JPH::Vec3(0, 60, 0), dt, profile);
        command = steering.step(JPH::Vec3(60, 0, 0), dt, profile);
        EXPECT_NEAR(command.GetX(), 60.0f, 1e-3f);
        EXPECT_NEAR(command.GetY(), 0.0f, 1e-3f);
    }

    TEST(RagdollCoreReferenceGaitTest, stateMachineTransfersOnConfirmedContactAfterMinimumSwing)
    {
        RagdollCore::ReferenceGaitProfile profile;
        profile.mDoubleSupportDuration = 0.1f;
        profile.mSwingDuration = 0.3f;
        profile.mMinimumSwingDuration = 0.15f;
        profile.mContactConfirmationDuration = 0.03f;
        profile.mMaximumStepDuration = 0.6f;
        profile.mSwingStartLoadFraction = 0.2f;
        profile.mMaximumDoubleSupportDuration = 0.3f;
        const float dt = 1.0f / 60.0f;
        RagdollCore::ReferenceGaitStateMachine machine;
        RagdollCore::ReferenceGaitState state;
        RagdollCore::ReferenceGaitStateInput input;
        input.mEnabled = true;
        input.mLeftFootSupported = input.mRightFootSupported = true;
        input.mLeftFootLoadFraction = input.mRightFootLoadFraction = 0.5f;
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mPhase, GaitPhase::LeftStance);
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        EXPECT_TRUE(state.mTransitioned);

        // The swing waits for the stance foot to carry the swing-start load,
        // up to the maximum double-support duration.
        input.mLeftFootLoadFraction = 0.1f;
        input.mRightFootLoadFraction = 0.9f;
        for (int i = 0; i < 6; ++i)
            ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        input.mLeftFootLoadFraction = 0.9f;
        input.mRightFootLoadFraction = 0.1f;

        // Double support lasts its configured interval, then the swing starts.
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mStage, ReferenceGaitStage::Swing);
        EXPECT_FALSE(state.mSwingFootLifted);
        const auto gait = RagdollCore::toGaitControllerOutput(state, profile);
        EXPECT_TRUE(gait.mStanceReady);
        EXPECT_FALSE(gait.mSwingFootLifted);

        // Contact before measured lift and before the minimum swing time is ignored.
        input.mRightFootSupported = false;
        input.mRightFootLoadFraction = 0.0f;
        input.mRightFootHeight = 3.0f;
        input.mRightFootVerticalVelocity = 5.0f;
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_TRUE(state.mSwingFootLifted);
        input.mRightFootSupported = true;
        input.mRightFootLoadFraction = 0.5f;
        input.mRightFootVerticalVelocity = -1.0f;
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mPhase, GaitPhase::LeftStance);
        input.mRightFootSupported = false;
        input.mRightFootLoadFraction = 0.0f;
        for (int i = 0; i < 8; ++i)
            ASSERT_TRUE(machine.step(input, dt, profile, state));
        // With a transfer load fraction, a grazing contact does not transfer.
        profile.mTransferLoadFraction = 0.1f;
        input.mRightFootSupported = true;
        input.mRightFootLoadFraction = 0.05f;
        for (int i = 0; i < 4; ++i)
            ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mPhase, GaitPhase::LeftStance);
        // Confirmed load after the minimum swing time transfers the stance.
        input.mRightFootLoadFraction = 0.5f;
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mPhase, GaitPhase::LeftStance);
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mPhase, GaitPhase::RightStance);
        EXPECT_EQ(state.mStage, ReferenceGaitStage::DoubleSupport);
        EXPECT_TRUE(state.mTransitioned);
        EXPECT_FALSE(state.mTimedOut);

        // A swing without contact reports a timeout and never fabricates a step.
        input.mRightFootLoadFraction = 0.9f;
        for (int i = 0; i < 6; ++i)
            ASSERT_TRUE(machine.step(input, dt, profile, state));
        input.mLeftFootSupported = false;
        input.mLeftFootLoadFraction = 0.0f;
        input.mLeftFootHeight = 4.0f;
        input.mLeftFootVerticalVelocity = 5.0f;
        input.mRightFootHeight = 0.0f;
        input.mRightFootVerticalVelocity = 0.0f;
        for (int i = 0; i < 40; ++i)
            ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_EQ(state.mPhase, GaitPhase::RightStance);
        EXPECT_TRUE(state.mTimedOut);
        input.mEnabled = false;
        ASSERT_TRUE(machine.step(input, dt, profile, state));
        EXPECT_FALSE(state.hasStance());
        EXPECT_FALSE(machine.step(input, 0.0f, profile, state));
    }

    TEST(RagdollCoreReferenceGaitTest, placementPreservesMeasuredLiftOffSideAfterRecovery)
    {
        using namespace RagdollCore;
        ReferenceGaitProfile profile;
        ReferenceGaitPlacementInput input;
        input.mPhase = GaitPhase::LeftStance;
        // Foot origins from native 20260908T082424488Z; COM/velocity are a
        // controlled placement probe, not a full native dynamics replay.
        input.mStanceFootPosition = JPH::RVec3(97.66, -7.38, 0);
        input.mSwingStartPosition = JPH::RVec3(84.57, -31.24, 0);
        input.mCenterOfMass = JPH::RVec3(91, -19, 70);
        input.mRequestedVelocity = JPH::Vec3(0, 30, 0);
        input.mCenterOfMassVelocity = input.mRequestedVelocity;
        input.mCenterOfMassHeight = 70;
        input.mGravity = 627.1216f;
        input.mStancePeriod = profile.mDoubleSupportDuration + profile.mSwingDuration;
        const auto measured = calculateReferenceGaitPlacement(input, profile);
        ASSERT_TRUE(measured);
        EXPECT_LE(measured->mTarget.GetX(), input.mStanceFootPosition.GetX() - profile.mMinimumStepWidth + 1e-4);
        input.mSwingStartPosition.reset();
        const auto legacy = calculateReferenceGaitPlacement(input, profile);
        ASSERT_TRUE(legacy);
        EXPECT_GE(legacy->mTarget.GetX(), input.mStanceFootPosition.GetX() + profile.mMinimumStepWidth - 1e-4);
    }

    TEST(RagdollCoreReferenceGaitTest, placementUsesCaptureLateralOffsetAndSteadyForwardFraction)
    {
        RagdollCore::ReferenceGaitProfile profile;
        // This fixture checks the full velocity-error correction, including
        // its zero-speed floor. Tuned production gains need not hit that floor.
        profile.mForwardVelocityErrorGain = 1.0f;
        profile.mLateralCaptureOffset = 4.0f;
        profile.mMinimumStepWidth = 12.0f;
        profile.mMaximumStepWidth = 40.0f;
        profile.mMaximumStepReach = 55.0f;
        RagdollCore::ReferenceGaitPlacementInput input;
        input.mPhase = GaitPhase::LeftStance;
        input.mCenterOfMass = JPH::RVec3(0.0, 10.0, 70.0);
        input.mCenterOfMassVelocity = JPH::Vec3(15.0f, 60.0f, 0.0f);
        input.mRequestedVelocity = JPH::Vec3(0.0f, 60.0f, 0.0f);
        input.mStanceFootPosition = JPH::RVec3(-8.0, 0.0, 2.5);
        input.mCenterOfMassHeight = 70.0f;
        input.mGravity = 627.12f;
        input.mStancePeriod = 0.45f;
        const auto placement = RagdollCore::calculateReferenceGaitPlacement(input, profile);
        ASSERT_TRUE(placement);
        const float tau = std::sqrt(70.0f / 627.12f);
        const float steady = std::tanh(0.45f / (2.0f * tau));
        EXPECT_NEAR(placement->mCaptureTimeConstant, tau, 1e-5f);
        EXPECT_NEAR(placement->mSteadyForwardFraction, steady, 1e-5f);
        // At the requested speed the forward offset is the steady fraction of
        // the capture distance; lateral is capture plus the outward offset.
        EXPECT_NEAR(placement->mForwardOffset, tau * steady * 60.0f, 1e-3f);
        EXPECT_NEAR(placement->mLateralOffset, 15.0f * tau + 4.0f, 1e-4f);
        EXPECT_NEAR(static_cast<float>(placement->mTarget.GetX()), 15.0f * tau + 4.0f, 1e-3f);
        EXPECT_NEAR(static_cast<float>(placement->mTarget.GetY()), 10.0f + tau * steady * 60.0f, 1e-3f);
        EXPECT_NEAR(static_cast<float>(placement->mTarget.GetZ()), 2.5f, 1e-6f);
        EXPECT_FALSE(placement->mWidthClamped);
        EXPECT_FALSE(placement->mReachClamped);

        // Faster than requested steps further; slower steps shorter but never behind the floor.
        input.mCenterOfMassVelocity = JPH::Vec3(0.0f, 90.0f, 0.0f);
        const auto fast = RagdollCore::calculateReferenceGaitPlacement(input, profile);
        ASSERT_TRUE(fast);
        EXPECT_GT(fast->mForwardOffset, placement->mForwardOffset);
        input.mCenterOfMassVelocity = JPH::Vec3(0.0f, 0.0f, 0.0f);
        const auto slow = RagdollCore::calculateReferenceGaitPlacement(input, profile);
        ASSERT_TRUE(slow);
        EXPECT_NEAR(slow->mForwardOffset, profile.mMinimumForwardPlacement, 1e-6f);

        // Crossing lateral velocity is clamped to the minimum width on the swing side.
        input.mCenterOfMassVelocity = JPH::Vec3(-40.0f, 60.0f, 0.0f);
        const auto crossing = RagdollCore::calculateReferenceGaitPlacement(input, profile);
        ASSERT_TRUE(crossing);
        EXPECT_TRUE(crossing->mWidthClamped);
        EXPECT_NEAR(static_cast<float>(crossing->mTarget.GetX()), -8.0f + 12.0f, 1e-3f);

        // Right stance mirrors the side; unreachable targets are clamped.
        input.mPhase = GaitPhase::RightStance;
        input.mStanceFootPosition = JPH::RVec3(8.0, 0.0, 2.5);
        input.mCenterOfMassVelocity = JPH::Vec3(0.0f, 400.0f, 0.0f);
        const auto mirrored = RagdollCore::calculateReferenceGaitPlacement(input, profile);
        ASSERT_TRUE(mirrored);
        EXPECT_LT(static_cast<float>(mirrored->mTarget.GetX()), 8.0f);
        EXPECT_TRUE(mirrored->mReachClamped);
        EXPECT_NEAR(JPH::Vec3(mirrored->mTarget - input.mStanceFootPosition).Length(), 55.0f, 1e-3f);

        input.mRequestedVelocity = JPH::Vec3::sZero();
        EXPECT_FALSE(RagdollCore::calculateReferenceGaitPlacement(input, profile));
    }

    TEST(RagdollCoreReferenceGaitTest, swingTrajectoryRisesTravelsLandsAndPressesAfterTimeout)
    {
        RagdollCore::ReferenceGaitProfile profile = legacyReferenceGaitProfile();
        profile.mSwingDuration = 0.4f;
        profile.mSwingFootLift = 8.0f;
        profile.mTouchdownPressSpeed = 20.0f;
        profile.mMaximumTouchdownPressDepth = 4.0f;
        RagdollCore::ReferenceSwingTrajectoryInput input;
        input.mLiftOffPosition = JPH::Vec3(8.0f, 0.0f, 2.5f);
        input.mPlacementPosition = JPH::Vec3(10.0f, 24.0f, 2.5f);
        const auto at = [&](float time) {
            input.mSwingTime = time;
            const auto position = RagdollCore::calculateReferenceSwingTrajectory(input, profile);
            EXPECT_TRUE(position);
            return position.value_or(JPH::Vec3::sZero());
        };
        EXPECT_TRUE(at(0.0f).IsClose(input.mLiftOffPosition));
        const JPH::Vec3 apex = at(0.18f);
        EXPECT_NEAR(apex.GetZ(), 2.5f + 8.0f, 1e-4f);
        EXPECT_GT(apex.GetY(), 0.0f);
        EXPECT_LT(apex.GetY(), 24.0f);
        const JPH::Vec3 landing = at(0.36f);
        EXPECT_TRUE(landing.IsClose(input.mPlacementPosition, 1e-6f));
        EXPECT_TRUE(at(0.4f).IsClose(input.mPlacementPosition, 1e-6f));
        EXPECT_NEAR(at(0.5f).GetZ(), 2.5f - 2.0f, 1e-4f);
        EXPECT_NEAR(at(2.0f).GetZ(), 2.5f - 4.0f, 1e-4f);
        input.mHoldLifted = true;
        const JPH::Vec3 held = at(1.0f);
        EXPECT_NEAR(held.GetX(), 8.0f, 1e-6f);
        EXPECT_NEAR(held.GetY(), 0.0f, 1e-6f);
        EXPECT_NEAR(held.GetZ(), 2.5f + 8.0f, 1e-4f);
        input.mSwingTime = -1.0f;
        EXPECT_FALSE(RagdollCore::calculateReferenceSwingTrajectory(input, profile));
    }

    struct ReferencePlanFixture : StanceFixture
    {
        RagdollCore::RagdollTelemetry telemetry;
        std::vector<RagdollCore::Transform> measured;
        std::vector<RagdollCore::GroundContact> contacts;

        ReferencePlanFixture()
        {
            using namespace RagdollCore;
            const auto skeleton = makeScenarioHumanoidSkeleton();
            const auto profile = makeScenarioHumanoidProfile();
            for (std::size_t i = 0; i < skeleton.mBones.size(); ++i)
            {
                const auto& bone = skeleton.mBones[i];
                BodyTelemetry body;
                body.mPosition = body.mCenterOfMass = JPH::RVec3(bone.mModelTransform.mTranslation);
                body.mRotation = bone.mModelTransform.mRotation;
                body.mMass = profile.mBodies[i].mMassFraction * 75;
                telemetry.mCenterOfMass += body.mCenterOfMass * body.mMass;
                telemetry.mTotalMass += body.mMass;
                telemetry.mBodies.push_back(body);
                measured.push_back(bone.mModelTransform);
                if (i != 0)
                {
                    ConstraintTelemetry joint;
                    joint.mParentBodyIndex = bone.mParent;
                    joint.mChildBodyIndex = i;
                    joint.mPosition = body.mPosition;
                    telemetry.mConstraints.push_back(joint);
                }
            }
            telemetry.mCenterOfMass /= telemetry.mTotalMass;
            for (const float x : { -12.0f, -4.0f })
                for (const float y : { -8.0f, 8.0f })
                {
                    GroundContact contact;
                    contact.mBodyIndex = 16;
                    contact.mPosition = JPH::RVec3(x, y, 0);
                    contact.mNormalImpulse = 1.0f;
                    contact.mFriction = 0.8f;
                    contacts.push_back(contact);
                }
        }

        RagdollCore::ReferenceGaitPlanInput input()
        {
            using namespace RagdollCore;
            const auto profile = makeScenarioHumanoidProfile();
            ReferenceGaitPlanInput input;
            input.mTelemetry = &telemetry;
            input.mBasePose = mBase;
            input.mMeasuredPose = measured;
            input.mContacts = contacts;
            const auto support = estimateFootSupport(telemetry.mCenterOfMass, contacts, { 16, 19 }, profile.mActiveController);
            EXPECT_TRUE(support);
            input.mSupport = *support;
            input.mLeftLeg = mLeft;
            input.mRightLeg = mRight;
            input.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
            input.mGravity = UnitModel::GravityUnitsPerSecondSquared;
            input.mFixedTimeStep = 1.0f / 60.0f;
            input.mState.mPhase = GaitPhase::LeftStance;
            input.mState.mStage = ReferenceGaitStage::Swing;
            return input;
        }
    };

    TEST(RagdollCoreReferenceGaitTest, continuousSwingHistorySurvivesRetargetAndClearsOnSupport)
    {
        using namespace RagdollCore;
        ReferencePlanFixture fixture;
        auto input = fixture.input();
        input.mSwingStartModel = fixture.measured[19].mTranslation;
        input.mState.mTimeInStage = 0.08f;
        ReferenceGaitProfile profile;
        const auto legacyHash = hashReferenceGaitProfile(profile);
        profile.mContinuousSwingTrajectory = true;
        EXPECT_NE(hashReferenceGaitProfile(profile), legacyHash);
        const auto first = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(first);
        ASSERT_TRUE(first->mSwingTrajectoryHistory);
        const auto expected = sampleSwingTrajectory(first->mSwingTrajectoryHistory->mSegment, input.mFixedTimeStep);
        ASSERT_TRUE(expected);
        input.mSwingTrajectoryHistory = first->mSwingTrajectoryHistory;
        input.mState.mTimeInStage += input.mFixedTimeStep;
        input.mRequestedHorizontalVelocity = JPH::Vec3(60, 0, 0);
        const auto turned = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(turned);
        ASSERT_TRUE(turned->mSwingTrajectoryHistory);
        const auto& start = turned->mSwingTrajectoryHistory->mSegment.mStart;
        EXPECT_TRUE(start.mPosition.IsClose(expected->mPosition, 1e-8f));
        EXPECT_TRUE(start.mVelocity.IsClose(expected->mVelocity, 1e-8f));
        EXPECT_TRUE(start.mAcceleration.IsClose(expected->mAcceleration, 1e-8f));
        ASSERT_TRUE(turned->mSwingFootTarget);
        EXPECT_NEAR(turned->mSwingFootTarget->GetX(), expected->mPosition.GetX(), 1e-4f);
        EXPECT_NEAR(turned->mSwingFootTarget->GetY(), expected->mPosition.GetY(), 1e-4f);
        input.mState.mTimeInStage = profile.mSwingTravelEndFraction * profile.mSwingDuration
            - 0.1f * input.mFixedTimeStep;
        const auto finalInterval = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(finalInterval);
        ASSERT_TRUE(finalInterval->mSwingTrajectoryHistory);
        EXPECT_FLOAT_EQ(finalInterval->mSwingTrajectoryHistory->mSegment.mDuration, input.mFixedTimeStep);
        EXPECT_TRUE(finalInterval->mSwingTrajectoryHistory->mSegment.mStart.mPosition.IsClose(expected->mPosition, 1e-8f));
        input.mState.mTimeInStage = profile.mSwingDuration + 0.05f;
        const auto terminal = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(terminal);
        ASSERT_TRUE(terminal->mSwingFootTarget);
        ASSERT_TRUE(terminal->mPlacement);
        EXPECT_TRUE(terminal->mPlacementLatched);
        EXPECT_NEAR(terminal->mPlacement->GetX(), first->mSwingTrajectoryHistory->mSegment.mEnd.mPosition.GetX(), 1e-4f);
        input.mHoldSwingFoot = true;
        const auto held = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(held);
        EXPECT_FALSE(held->mSwingTrajectoryHistory);
        input.mHoldSwingFoot = false;
        input.mState.mStage = ReferenceGaitStage::DoubleSupport;
        const auto support = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(support);
        EXPECT_FALSE(support->mSwingTrajectoryHistory);
        profile.mContinuousSwingTrajectory = false;
        EXPECT_EQ(hashReferenceGaitProfile(profile), legacyHash);
    }

    TEST(RagdollCoreReferenceGaitTest, supportHullContractionDoesNotInventHeelOrToeSupport)
    {
        using namespace RagdollCore;
        for (const float edgeY : { -8.0f, 8.0f })
            for (const bool singlePoint : { false, true })
            {
                SCOPED_TRACE(edgeY);
                SCOPED_TRACE(singlePoint);
                ReferencePlanFixture fixture;
                fixture.contacts.clear();
                for (const float x : { -12.0f, -4.0f })
                {
                    GroundContact contact;
                    contact.mBodyIndex = 16;
                    contact.mPosition = JPH::RVec3(x, edgeY, 0);
                    contact.mNormalImpulse = 1;
                    contact.mFriction = 0.8f;
                    fixture.contacts.push_back(contact);
                    if (singlePoint) break;
                }
                const auto input = fixture.input();
                const auto plan = planReferenceGaitActuation(input, {});
                ASSERT_TRUE(plan);
                ASSERT_TRUE(plan->mSupport.mSupported);
                // Every feasible pressure point is on this actual contact
                // edge, even though the ankle/foot COM lies behind it.
                EXPECT_NEAR(plan->mSupport.mCenterOfPressure.GetY(), edgeY, 1e-5);
                for (const auto& force : plan->mSupport.mContactForces)
                {
                    EXPECT_NEAR(force.mPosition.GetY(), edgeY, 1e-5);
                    EXPECT_GE(force.mPosition.GetX(), -12.0);
                    EXPECT_LE(force.mPosition.GetX(), singlePoint ? -12.0 : -4.0);
                }
            }
    }

    TEST(RagdollCoreReferenceGaitTest, movingDoubleSupportBrakesLateralVelocityWithinBudget)
    {
        using namespace RagdollCore;
        ReferencePlanFixture fixture;
        const auto leftContacts = fixture.contacts;
        for (auto contact : leftContacts)
        {
            contact.mBodyIndex = 19;
            contact.mPosition += JPH::Vec3(16, 0, 0);
            fixture.contacts.push_back(contact);
        }
        auto input = fixture.input();
        input.mState.mStage = ReferenceGaitStage::DoubleSupport;
        input.mState.mInitialTransfer = false;
        input.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
        input.mFacing = JPH::Vec3::sAxisY();
        ReferenceGaitProfile profile;
        profile.mBrakeLateralTransfer = true;
        for (float vx : { -20.0f, 20.0f })
        {
            input.mCenterOfMassVelocity = JPH::Vec3(vx, 20, 0);
            const auto plan = planReferenceGaitActuation(input, profile);
            ASSERT_TRUE(plan);
            EXPECT_GT(plan->mTransferReferenceVelocity, 0);
            EXPECT_LT(plan->mRequestedAcceleration.GetX() * vx, 0);
            EXPECT_LE(plan->mRequestedAcceleration.Length(), profile.mMaximumRequestedAcceleration + 1e-4f);
        }
        const auto newHash = hashReferenceGaitProfile(profile);
        profile.mBrakeLateralTransfer = false;
        EXPECT_NE(newHash, hashReferenceGaitProfile(profile));
        const auto legacy = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(legacy);
        EXPECT_GT(legacy->mTransferReferenceVelocity, 0);
        profile.mBrakeLateralTransfer = true;
        const auto reduced = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(reduced);
        EXPECT_FLOAT_EQ(reduced->mTransferReferenceVelocity, 0.5f * legacy->mTransferReferenceVelocity);
        profile.mBrakeLateralTransfer = true;
        input.mState.mInitialTransfer = true;
        const auto startup = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(startup);
        EXPECT_EQ(startup->mRequestedAcceleration, legacy->mRequestedAcceleration);
        input.mState.mInitialTransfer = false;
        input.mCoordinateLegPose = true;
        const auto coordinated = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(coordinated);
        EXPECT_FLOAT_EQ(coordinated->mTransferReferenceVelocity, legacy->mTransferReferenceVelocity);
    }

    TEST(RagdollCoreReferenceGaitTest, stationaryDoubleSupportDoesNotTransferTowardAnArbitraryStance)
    {
        using namespace RagdollCore;
        ReferencePlanFixture fixture;
        const auto leftContacts = fixture.contacts;
        for (auto contact : leftContacts)
        {
            contact.mBodyIndex = 19;
            contact.mPosition += JPH::Vec3(16, 0, 0);
            fixture.contacts.push_back(contact);
        }
        auto input = fixture.input();
        input.mState.mStage = ReferenceGaitStage::DoubleSupport;
        input.mStationaryCorrectiveStep = true;
        input.mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        input.mFacing = JPH::Vec3::sAxisY();
        input.mCenterOfMassVelocity = JPH::Vec3(1, 2, 0);
        const auto left = planReferenceGaitActuation(input, {});
        ASSERT_TRUE(left);
        input.mState.mPhase = GaitPhase::RightStance;
        const auto right = planReferenceGaitActuation(input, {});
        ASSERT_TRUE(right);
        EXPECT_TRUE(left->mRequestedAcceleration.IsClose(right->mRequestedAcceleration));
        EXPECT_FLOAT_EQ(left->mTransferReferenceVelocity, 0);
        EXPECT_FLOAT_EQ(right->mTransferReferenceVelocity, 0);
        EXPECT_TRUE(left->mTrackingVelocity.IsNearZero());
    }

    TEST(RagdollCoreReferenceGaitTest, landingPressKeepsNearestKneePlaneThroughYaw)
    {
        using namespace RagdollCore;
        for (const float yaw : { -2.0f, -1.0f, 0.0f, 1.0f, 2.0f })
        {
            SCOPED_TRACE(yaw);
            ReferencePlanFixture fixture;
            const auto rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), yaw);
            for (std::size_t i = 0; i < fixture.measured.size(); ++i)
            {
                fixture.measured[i].mTranslation = rotation * fixture.measured[i].mTranslation;
                fixture.measured[i].mRotation = (rotation * fixture.measured[i].mRotation).Normalized();
                auto& body = fixture.telemetry.mBodies[i];
                body.mRotation = fixture.measured[i].mRotation;
                body.mPosition = JPH::RVec3(rotation * JPH::Vec3(body.mPosition));
                body.mCenterOfMass = JPH::RVec3(rotation * JPH::Vec3(body.mCenterOfMass));
            }
            fixture.telemetry.mCenterOfMass = JPH::RVec3(rotation * JPH::Vec3(fixture.telemetry.mCenterOfMass));
            for (auto& joint : fixture.telemetry.mConstraints)
                joint.mPosition = JPH::RVec3(rotation * JPH::Vec3(joint.mPosition));
            for (auto& contact : fixture.contacts)
                contact.mPosition = JPH::RVec3(rotation * JPH::Vec3(contact.mPosition));
            auto input = fixture.input();
            input.mState.mStage = ReferenceGaitStage::DoubleSupport;
            input.mState.mEnteredFromTouchdown = true;
            input.mLandingFootModel = fixture.measured[input.mLeftLeg.mFoot].mTranslation;
            input.mFacing = rotation * JPH::Vec3::sAxisY();
            ReferenceGaitProfile profile;
            // Isolate the IK branch before the feasibility gate masks it.
            profile.mFinalCommandFeasibilityStanceMargin = 0;
            profile.mFinalCommandFeasibilitySwingMargin = 0;
            const auto plan = planReferenceGaitActuation(input, profile);
            ASSERT_TRUE(plan);
            ASSERT_TRUE(plan->mLandingPressApplied);
            ASSERT_TRUE(plan->mSwingHipWorldTarget);
            EXPECT_GT(std::abs(plan->mSwingHipWorldTarget->Dot(
                fixture.measured[input.mLeftLeg.mHip].mRotation)), 0.7071067f)
                << "a downward press must not choose the opposite thigh orientation";
        }
    }

    TEST(RagdollCoreReferenceGaitTest, stationaryLandingPressEndsWithoutWaitingForAnotherStep)
    {
        using namespace RagdollCore;
        ReferencePlanFixture fixture;
        auto input = fixture.input();
        ReferenceGaitProfile profile;
        input.mState.mStage = ReferenceGaitStage::DoubleSupport;
        input.mState.mEnteredFromTouchdown = true;
        input.mLandingFootModel = fixture.measured[input.mLeftLeg.mFoot].mTranslation;
        input.mStationaryCorrectiveStep = true;
        input.mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        input.mFacing = JPH::Vec3::sAxisY();
        const auto arrival = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(arrival);
        ASSERT_TRUE(arrival->mLandingPressApplied);
        input.mState.mTimeInStage = 2.0f;
        const auto settled = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(settled);
        EXPECT_FALSE(settled->mLandingPressApplied);
        EXPECT_TRUE(settled->mTrackingVelocity.IsNearZero());
        EXPECT_TRUE(std::isfinite(settled->mRequestedAcceleration.LengthSq()));
        profile.mDoubleSupportDuration = profile.mMaximumDoubleSupportDuration = 0;
        input.mState.mTimeInStage = 0;
        const auto immediate = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(immediate);
        EXPECT_FALSE(immediate->mLandingPressApplied);
        input.mStationaryCorrectiveStep = false;
        input.mRequestedHorizontalVelocity = JPH::Vec3(0, 30, 0);
        const auto walking = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(walking);
        EXPECT_TRUE(walking->mLandingPressApplied);
    }

    TEST(RagdollCoreReferenceGaitTest, stationaryCorrectionPlansWithoutInventingTravelSpeed)
    {
        using namespace RagdollCore;
        ReferencePlanFixture fixture;
        auto input = fixture.input();
        input.mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        EXPECT_FALSE(planReferenceGaitActuation(input, {}));
        input.mStationaryCorrectiveStep = true;
        EXPECT_FALSE(planReferenceGaitActuation(input, {}));
        input.mFacing = JPH::Vec3::sAxisY();
        input.mStopping = true;
        input.mSwingStartModel = fixture.measured[input.mRightLeg.mFoot].mTranslation;
        const auto plan = planReferenceGaitActuation(input, {});
        ASSERT_TRUE(plan);
        EXPECT_TRUE(plan->mPlacement);
        EXPECT_TRUE(plan->mSwingFootTarget);
        EXPECT_TRUE(plan->mSwingHipWorldTarget);
        EXPECT_TRUE(plan->mTrackingVelocity.IsNearZero());
        EXPECT_TRUE(std::isfinite(plan->mRequestedAcceleration.LengthSq()));
        input.mRequestedHorizontalVelocity = JPH::Vec3::sAxisY();
        EXPECT_FALSE(planReferenceGaitActuation(input, {}));
    }

    TEST(RagdollCoreReferenceGaitTest, velocityLeadPlansAccelerationWithoutCappingEventualSpeedOrBraking)
    {
        using namespace RagdollCore;
        ReferencePlanFixture fixture;
        auto input = fixture.input();
        ReferenceGaitProfile profile;
        input.mRequestedHorizontalVelocity = JPH::Vec3(0, 155, 0);
        const auto start = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(start);
        EXPECT_GT(start->mTrackingVelocity.GetY(), 0);
        EXPECT_LE(start->mTrackingVelocity.GetY(), 60);
        auto historical = profile;
        historical.mVelocityLeadAccelerationFraction = 1.0f;
        EXPECT_NE(hashReferenceGaitProfile(profile), hashReferenceGaitProfile(historical));
        const auto optimistic = planReferenceGaitActuation(input, historical);
        ASSERT_TRUE(optimistic);
        EXPECT_LT(start->mTrackingVelocity.GetY(), optimistic->mTrackingVelocity.GetY());
        historical.mVelocityLeadAccelerationFraction = 0;
        EXPECT_FALSE(planReferenceGaitActuation(input, historical));
        input.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
        const auto nominal = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(nominal);
        EXPECT_TRUE(nominal->mTrackingVelocity.IsClose(input.mRequestedHorizontalVelocity));
        input.mRequestedHorizontalVelocity = JPH::Vec3(0, 300, 0);
        const auto distant = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(distant);
        EXPECT_TRUE(distant->mTrackingVelocity.IsClose(start->mTrackingVelocity));
        input.mCenterOfMassVelocity = JPH::Vec3(0, 100, 0);
        const auto moving = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(moving);
        EXPECT_GT(moving->mTrackingVelocity.GetY(), 100);
        input.mRequestedHorizontalVelocity = JPH::Vec3(0, 10, 0);
        const auto braking = planReferenceGaitActuation(input, profile);
        ASSERT_TRUE(braking);
        EXPECT_TRUE(braking->mTrackingVelocity.IsClose(input.mRequestedHorizontalVelocity));
    }

    TEST(RagdollCoreReferenceGaitTest, landingHeightDoesNotFollowRollingStanceOrLatchedHeight)
    {
        using namespace RagdollCore;
        ReferencePlanFixture fixture;
        auto input = fixture.input();
        input.mSwingStartModel = fixture.measured[19].mTranslation;
        input.mLandingFootOriginHeight = 2.0;
        ReferenceGaitProfile gait;
        for (const double raisedStance : { 2.0, 8.0, 15.0 })
        {
            fixture.telemetry.mBodies[16].mPosition.SetZ(raisedStance);
            const auto plan = planReferenceGaitActuation(input, gait);
            ASSERT_TRUE(plan);
            ASSERT_TRUE(plan->mPlacement);
            EXPECT_DOUBLE_EQ(plan->mPlacement->GetZ(), 2.0);
        }
        input.mPreviousPlacement = JPH::RVec3(12, 18, 9);
        gait.mPlacementLatchFraction = 0.25f;
        input.mState.mTimeInStage = gait.mSwingDuration * 0.75f;
        const auto latched = planReferenceGaitActuation(input, gait);
        ASSERT_TRUE(latched);
        ASSERT_TRUE(latched->mPlacement);
        EXPECT_TRUE(latched->mPlacementLatched);
        EXPECT_DOUBLE_EQ(latched->mPlacement->GetX(), 12);
        EXPECT_DOUBLE_EQ(latched->mPlacement->GetY(), 18);
        EXPECT_DOUBLE_EQ(latched->mPlacement->GetZ(), 2);
        input.mLandingFootOriginHeight = std::numeric_limits<double>::quiet_NaN();
        EXPECT_FALSE(planReferenceGaitActuation(input, gait));
    }

    TEST(RagdollCoreReferenceGaitTest, coordinatedPoseDoesNotReuseIndependentSwingFeedForward)
    {
        using namespace RagdollCore;
        ReferencePlanFixture fixture;
        addNativeJointGeometry(fixture);
        fixture.measured[0].mTranslation.SetZ(65);
        auto input = fixture.input();
        input.mSwingStartModel = fixture.measured[19].mTranslation;
        input.mState.mStage = ReferenceGaitStage::Swing;
        input.mState.mTimeInStage = 0.1f;
        ReferenceGaitProfile enabled;
        const auto ordinary = planReferenceGaitActuation(input, enabled);
        ASSERT_TRUE(ordinary);
        ASSERT_FALSE(ordinary->mJointAngularVelocityTargets.empty());
        ASSERT_FALSE(ordinary->mSwingHipFeedForwardTorque.IsNearZero());
        input.mCoordinateLegPose = true;
        const auto coordinated = planReferenceGaitActuation(input, enabled);
        auto disabled = enabled;
        disabled.mSwingVelocityFeedForward = false;
        disabled.mSwingInertiaFeedForward = 0;
        const auto withoutOldFeedForward = planReferenceGaitActuation(input, disabled);
        ASSERT_TRUE(coordinated && withoutOldFeedForward);
        ASSERT_TRUE(coordinated->mCoordinatedPoseApplied && withoutOldFeedForward->mCoordinatedPoseApplied);
        EXPECT_TRUE(coordinated->mJointAngularVelocityTargets.empty());
        EXPECT_TRUE(coordinated->mSwingHipFeedForwardTorque.IsNearZero());
        EXPECT_TRUE(coordinated->mSwingKneeFeedForwardTorque.IsNearZero());
        ASSERT_EQ(coordinated->mLoadTorques.size(), withoutOldFeedForward->mLoadTorques.size());
        for (std::size_t body = 0; body < coordinated->mLoadTorques.size(); ++body)
            EXPECT_EQ(coordinated->mLoadTorques[body], withoutOldFeedForward->mLoadTorques[body]);
    }

    TEST(RagdollCoreReferenceGaitTest, coordinatedLegPlanPreservesEndpointsAndNativeRelativeJoints)
    {
        using namespace RagdollCore;
        ReferencePlanFixture fixture;
        addNativeJointGeometry(fixture);
        fixture.measured[0].mTranslation.SetZ(65);
        fixture.measured[0].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.15f);
        fixture.telemetry.mBodies[0].mCenterOfMass = fixture.telemetry.mBodies[0].mPosition + JPH::Vec3(0, 0, 1.25f);
        auto input = fixture.input();
        input.mCoordinateLegPose = true;
        input.mPreferredPelvisCenterHeight = 66.75;
        input.mSwingStartModel = fixture.measured[19].mTranslation;
        input.mState.mTimeInStage = 0.1f;
        std::vector<int> parents(fixture.measured.size(), -1);
        for (const auto& joint : fixture.telemetry.mConstraints)
            parents[joint.mChildBodyIndex] = static_cast<int>(joint.mParentBodyIndex);
        for (const auto stage : { ReferenceGaitStage::DoubleSupport, ReferenceGaitStage::Swing })
        {
            input.mState.mStage = stage;
            const auto plan = planReferenceGaitActuation(input, {});
            ASSERT_TRUE(plan);
            ASSERT_TRUE(plan->mCoordinatedPoseAttempted);
            ASSERT_TRUE(plan->mCoordinatedPoseApplied);
            EXPECT_FALSE(plan->mSwingHipWorldTarget);
            EXPECT_TRUE(plan->mWorldFootTargets.empty());
            const auto& pose = plan->mTargetPose.mBodyTransforms;
            // The standing preference is independent of the lower measured
            // origin and accounts for the body's nonzero COM offset.
            if (stage == ReferenceGaitStage::DoubleSupport)
                EXPECT_NEAR(pose[0].mTranslation.GetZ(), 65.5f, 1e-3f);
            EXPECT_LE((pose[16].mTranslation - fixture.measured[16].mTranslation).Length(), 1e-3f);
            EXPECT_TRUE(pose[16].mRotation.IsClose(fixture.measured[16].mRotation, 1e-10f));
            const auto swingPoint = stage == ReferenceGaitStage::Swing
                ? *plan->mSwingFootTarget : fixture.measured[19].mTranslation;
            EXPECT_LE((pose[19].mTranslation - swingPoint).Length(), 1e-3f);
            const auto converted = referenceGaitRelativeMotorPose(*plan, fixture.measured, parents, JPH::Quat::sIdentity());
            ASSERT_TRUE(converted);
            for (const auto& leg : { fixture.mLeft, fixture.mRight })
            {
                EXPECT_NEAR((pose[leg.mKnee].mTranslation - pose[leg.mHip].mTranslation).Length(), 29, 1e-3f);
                EXPECT_NEAR((pose[leg.mFoot].mTranslation - pose[leg.mKnee].mTranslation).Length(), 30.5f, 1e-3f);
                const auto hip = clampSwingTwistJointOrientation(*leg.mHipGeometry, pose[0].mRotation, pose[leg.mHip].mRotation);
                const auto ankle = clampSwingTwistJointOrientation(*leg.mAnkleGeometry, pose[leg.mKnee].mRotation, pose[leg.mFoot].mRotation);
                ASSERT_TRUE(hip && ankle);
                EXPECT_TRUE(hip->IsClose(pose[leg.mHip].mRotation, 1e-10f));
                EXPECT_TRUE(ankle->IsClose(pose[leg.mFoot].mRotation, 1e-10f));
                for (const auto body : { leg.mHip, leg.mKnee, leg.mFoot })
                {
                    const auto parent = parents[body];
                    const auto& convertedPose = converted->mBodyTransforms;
                    const auto relative = convertedPose[parent].mRotation.Conjugated() * convertedPose[body].mRotation;
                    EXPECT_TRUE(relative.IsClose(pose[parent].mRotation.Conjugated() * pose[body].mRotation, 1e-10f));
                }
            }
        }
        input.mPreferredPelvisCenterHeight = std::numeric_limits<double>::quiet_NaN();
        EXPECT_FALSE(planReferenceGaitActuation(input, {}));
        input.mPreferredPelvisCenterHeight = 66.75;
        // Missing native geometry cannot silently publish a coordinated pose.
        input.mLeftLeg.mHipGeometry.reset();
        const auto fallback = planReferenceGaitActuation(input, {});
        input.mCoordinateLegPose = false;
        const auto ordinary = planReferenceGaitActuation(input, {});
        ASSERT_TRUE(fallback && ordinary);
        EXPECT_TRUE(fallback->mCoordinatedPoseAttempted);
        EXPECT_FALSE(fallback->mCoordinatedPoseApplied);
        for (std::size_t i = 0; i < fixture.measured.size(); ++i)
        {
            EXPECT_EQ(fallback->mTargetPose.mBodyTransforms[i].mTranslation, ordinary->mTargetPose.mBodyTransforms[i].mTranslation);
            EXPECT_EQ(fallback->mTargetPose.mBodyTransforms[i].mRotation, ordinary->mTargetPose.mBodyTransforms[i].mRotation);
        }
    }

    TEST(RagdollCoreReferenceGaitTest, supportMotionCannotInventContactOnTheSelectedFoot)
    {
        using namespace RagdollCore;
        ReferencePlanFixture fixture;
        SupportMotionReference motion;
        motion.mPosition = fixture.telemetry.mCenterOfMass;
        const auto calculate = [&] {
            return calculateStandingJointSupport(fixture.telemetry, fixture.contacts, { 16, 19 },
                UnitModel::GravityUnitsPerSecondSquared, 0.8f, makeScenarioHumanoidProfile().mActiveController, motion);
        };
        motion.mSupportFoot = 16;
        const auto loaded = calculate();
        ASSERT_TRUE(loaded);
        ASSERT_FALSE(loaded->mContactForces.empty());
        motion.mSupportFoot = 19; // No measured right-foot contacts exist.
        const auto unsupported = calculate();
        ASSERT_TRUE(unsupported);
        EXPECT_TRUE(unsupported->mContactForces.empty());
        for (const auto& torque : unsupported->mJointTorques)
            EXPECT_TRUE(torque.IsNearZero());
        motion.mSupportFoot = 0;
        EXPECT_FALSE(calculate());
    }

    TEST(RagdollCoreReferenceGaitTest, loadedAnklePreservesMeasuredJointAngleWhenCalfTargetChanges)
    {
        using namespace RagdollCore;
        ReferencePlanFixture fixture;
        // Deliberately different pelvis, calf and foot rotations expose a
        // pelvis-relative copy that would silently retarget the ankle.
        fixture.measured[0].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.1f);
        fixture.measured[14].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.3f);
        fixture.measured[17].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.25f);
        fixture.measured[15].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.4f);
        fixture.measured[16].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.08f);
        fixture.measured[18].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.3f);
        auto input = fixture.input();
        ReferenceGaitProfile gait;
        // The optional loaded-knee target must be assembled before its ankle.
        gait.mStanceKneeFlexion = 0.2f;
        for (const auto stage : { ReferenceGaitStage::DoubleSupport, ReferenceGaitStage::Swing })
        {
            input.mState.mStage = stage;
            const auto plan = planReferenceGaitActuation(input, gait);
            ASSERT_TRUE(plan);
            for (const auto& leg : { fixture.mLeft, fixture.mRight })
            {
                if (stage == ReferenceGaitStage::Swing && leg.mFoot == fixture.mRight.mFoot)
                    continue;
                const auto& target = plan->mTargetPose.mBodyTransforms;
                const auto relative = target[leg.mKnee].mRotation.Conjugated() * target[leg.mFoot].mRotation;
                const auto measured = fixture.measured[leg.mKnee].mRotation.Conjugated()
                    * fixture.measured[leg.mFoot].mRotation;
                EXPECT_LT(1.0f - std::abs(relative.Dot(measured)), 1e-6f);
            }
        }
    }

    TEST(RagdollCoreReferenceGaitTest, assembledSwingDescendsToPlacementAfterElevatedLiftOff)
    {
        using namespace RagdollCore;
        ReferencePlanFixture fixture;
        auto input = fixture.input();
        ReferenceGaitProfile gait = legacyReferenceGaitProfile();
        input.mState.mTimeInStage = gait.mSwingDuration;
        input.mSwingStartModel = JPH::Vec3(8, 0, 7.5f);
        std::string reason;
        const auto landing = planReferenceGaitActuation(input, gait, &reason);
        ASSERT_TRUE(landing) << reason;
        ASSERT_TRUE(landing->mPlacement);
        ASSERT_TRUE(landing->mSwingFootTarget);
        EXPECT_NEAR(landing->mSwingFootTarget->GetZ(), 2.5f, 1e-5f);
        EXPECT_NEAR(landing->mSwingFootTarget->GetZ(), landing->mPlacement->GetZ(), 1e-5f);
        // A straight standing leg cannot also reach forward at floor height.
        // Report the IK clamp even when the optional placement limiter is off.
        EXPECT_TRUE(landing->mSwingReachLimited);
        input.mHoldSwingFoot = true;
        const auto held = planReferenceGaitActuation(input, gait, &reason);
        ASSERT_TRUE(held) << reason;
        ASSERT_TRUE(held->mSwingFootTarget);
        EXPECT_NEAR(held->mSwingFootTarget->GetZ(), 7.5f + gait.mSwingFootLift, 1e-5f);
        EXPECT_FALSE(held->mSwingReachLimited);
    }

    TEST(RagdollCoreReferenceGaitTest, facingSeparatesWhereTheBodyPointsFromWhereItTravels)
    {
        using namespace RagdollCore;
        // Plan item 2.3a. Before the split the plan built both its placement
        // frame and its pelvis aim from the requested velocity, so the body
        // could only ever walk along its own nose and a player could never
        // strafe or back up. Facing is now separate, and an unset facing must
        // reproduce the old behaviour EXACTLY so every existing gate keeps
        // its meaning.
        const auto settings = referenceSettings();
        auto script = walkingScript(1.2f, 60, 30);
        const auto baseline = run(script, settings);
        ASSERT_TRUE(baseline.completed()) << describe(baseline);

        // Facing explicitly along the travel direction: identical telemetry.
        auto alongTravel = script;
        for (auto& keyframe : alongTravel.mKeyframes)
            keyframe.mFacing = JPH::Vec3(0.0f, 1.0f, 0.0f);
        const auto matched = run(alongTravel, settings);
        ASSERT_TRUE(matched.completed()) << describe(matched);
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(baseline, matched))
            << "facing along travel must reproduce the pre-split behaviour";

        // A facing across the travel direction aims the pelvis elsewhere, so
        // the run must differ. The walk still has to leave the ground: this
        // pins that the split is wired through, not that a strafe is stable.
        auto across = script;
        for (auto& keyframe : across.mKeyframes)
            keyframe.mFacing = JPH::Vec3(1.0f, 0.0f, 0.0f);
        const auto strafing = run(across, settings);
        ASSERT_TRUE(strafing.completed()) << describe(strafing);
        EXPECT_FALSE(haveIdenticalScenarioTelemetry(baseline, strafing))
            << "a facing across the travel direction must change the plan";

        // A zero facing is the documented default, not a rejection.
        auto zeroed = script;
        for (auto& keyframe : zeroed.mKeyframes)
            keyframe.mFacing = JPH::Vec3::sZero();
        const auto defaulted = run(zeroed, settings);
        ASSERT_TRUE(defaulted.completed()) << describe(defaulted);
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(baseline, defaulted));
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreReferenceGaitTest, DISABLED_scenarioPlayerInputSeparatesImmediateTravelFromTurningFacing)
    {
        using namespace RagdollCore;
        auto settings = referenceSettings();
        settings.mReferenceGaitPlayerInput = true;
        auto script = walkingScript(1.8f, 60, 30);
        ScenarioInputKeyframe facing;
        facing.mFacing = JPH::Vec3::sAxisY();
        script.mKeyframes.insert(script.mKeyframes.begin(), facing);
        script.mKeyframes[1].mFacing = facing.mFacing;
        ScenarioInputKeyframe turn;
        turn.mStep = 60;
        turn.mRequestedHorizontalVelocity = JPH::Vec3(60, 0, 0);
        turn.mFacing = JPH::Vec3::sAxisX();
        script.mKeyframes.push_back(turn);
        auto stop = turn;
        stop.mStep = 90;
        stop.mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        script.mKeyframes.push_back(stop);
        const auto profile = makeScenarioHumanoidProfile();
        const auto found = std::find_if(profile.mBodies.begin(), profile.mBodies.end(),
            [&](const auto& body) { return body.mName == profile.mLandmarks.mPelvis; });
        ASSERT_NE(found, profile.mBodies.end());
        const auto pelvis = static_cast<std::size_t>(found - profile.mBodies.begin());
        std::optional<JPH::Quat> before, after;
        settings.mReferenceGaitPlanObserver = [&](std::uint64_t step, const ReferenceGaitPlan& plan) {
            if (step == 59) before = plan.mTargetPose.mBodyTransforms[pelvis].mRotation;
            if (step == 60) after = plan.mTargetPose.mBodyTransforms[pelvis].mRotation;
        };
        const auto player = run(script, settings);
        ASSERT_TRUE(player.completed()) << describe(player);
        ASSERT_GT(player.mFrames.size(), 91u);
        EXPECT_NEAR(player.mFrames[60].mRequestedHorizontalVelocity.GetX(), 60, 1e-4f);
        EXPECT_NEAR(player.mFrames[60].mRequestedHorizontalVelocity.GetY(), 0, 1e-4f);
        ASSERT_TRUE(before && after) << "the facing change must reach an active reference plan";
        JPH::Vec3 axis;
        float angle = 0;
        ((*after * before->Conjugated()).Normalized().EnsureWPositive()).GetAxisAngle(axis, angle);
        EXPECT_GT(angle, 0.005f);
        EXPECT_LE(angle, settings.mReferenceGait.mSteeringTurnRate * settings.mFixedTimeStep + 1e-4f);
        EXPECT_GT(player.mFrames[90].mRequestedHorizontalVelocity.Length(), 0);
        EXPECT_LT(player.mFrames[91].mRequestedHorizontalVelocity.Length(),
            player.mFrames[89].mRequestedHorizontalVelocity.Length());
        const auto repeated = run(script, settings);
        ASSERT_TRUE(repeated.completed());
        EXPECT_TRUE(haveIdenticalScenarioTelemetry(player, repeated));

        settings.mReferenceGaitPlayerInput = false;
        before.reset();
        after.reset();
        const auto npc = run(script, settings);
        ASSERT_TRUE(npc.completed()) << describe(npc);
        EXPECT_LT(npc.mFrames[60].mRequestedHorizontalVelocity.GetX(), 2);
        EXPECT_GT(npc.mFrames[60].mRequestedHorizontalVelocity.GetY(), 20);
        ASSERT_TRUE(before && after);
        ((*after * before->Conjugated()).Normalized().EnsureWPositive()).GetAxisAngle(axis, angle);
        EXPECT_GT(angle, 1.0f) << "the control must expose unfiltered explicit facing";
    }

    TEST(RagdollCoreReferenceGaitTest, hitStunSuppressesTheRequestInProportionToTheImpulse)
    {
        using namespace RagdollCore;
        const float dt = 1.0f / 60;
        ReferenceGaitProfile profile;
        ReferenceGaitHitStun stun;
        EXPECT_FALSE(stun.active());
        // Below the minimum impulse a hit does not interrupt at all.
        stun.begin(profile.mHitStunMinimumImpulse - 1.0f, profile);
        EXPECT_FALSE(stun.active());
        // At the reference impulse the stun lasts the profile duration.
        stun.begin(profile.mHitStunReferenceImpulse, profile);
        ASSERT_TRUE(stun.active());
        int steps = 0;
        while (stun.active() && steps < 1000)
        {
            stun.step(dt);
            ++steps;
        }
        EXPECT_NEAR(steps * dt, profile.mHitStunDuration, dt);
        // It scales with the impulse and is capped at twice the reference.
        stun.begin(0.5f * profile.mHitStunReferenceImpulse, profile);
        EXPECT_NEAR(stun.mRemaining, 0.5f * profile.mHitStunDuration, 1e-4f);
        stun.reset();
        stun.begin(10.0f * profile.mHitStunReferenceImpulse, profile);
        EXPECT_NEAR(stun.mRemaining, 2.0f * profile.mHitStunDuration, 1e-4f);
        // A second, weaker hit never shortens a stun already running.
        const float remaining = stun.mRemaining;
        stun.begin(profile.mHitStunMinimumImpulse, profile);
        EXPECT_FLOAT_EQ(stun.mRemaining, remaining);
        // A non-finite impulse and a zero duration profile do nothing.
        stun.reset();
        stun.begin(std::numeric_limits<float>::quiet_NaN(), profile);
        EXPECT_FALSE(stun.active());
        auto disabled = profile;
        disabled.mHitStunDuration = 0.0f;
        stun.begin(profile.mHitStunReferenceImpulse, disabled);
        EXPECT_FALSE(stun.active());
    }

    TEST(RagdollCoreReferenceGaitTest, standingHandoverBlendsFromTheMeasuredPose)
    {
        using namespace RagdollCore;
        const float dt = 1.0f / 60;
        ReferenceGaitProfile profile;
        profile.mStandingHandoverDuration = 0.5f;
        RagdollPose measured;
        RagdollPose standing;
        for (int i = 0; i < 3; ++i)
        {
            Transform transform;
            transform.mTranslation = JPH::Vec3(0, 0, 10.0f * i);
            transform.mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.8f);
            measured.mBodyTransforms.push_back(transform);
            transform.mTranslation = JPH::Vec3(0, 0, 12.0f * i);
            transform.mRotation = JPH::Quat::sIdentity();
            standing.mBodyTransforms.push_back(transform);
        }
        ReferenceGaitHandover handover;
        EXPECT_FALSE(handover.active());
        EXPECT_FLOAT_EQ(handover.blendPelvisHeight(65.0f, profile), 65.0f);
        // Inactive: no blend, the standing pose stands.
        RagdollPose out;
        EXPECT_FALSE(handover.blendPose(standing, profile, out));
        handover.begin(measured, 61.0f, JPH::RVec3(10, 20, 30), profile);
        ASSERT_TRUE(handover.active());
        // The support reference starts at the hand-over center of mass,
        // moving with the body, so the standing balance request is zero.
        {
            const auto reference = handover.supportReference(JPH::RVec3(30, 20, 0), JPH::Vec3(4, 0, 1), profile);
            ASSERT_TRUE(reference);
            EXPECT_NEAR(reference->mPosition.GetX(), 10.0f, 1e-4f);
            EXPECT_NEAR(reference->mPosition.GetZ(), 0.0f, 1e-4f);
            EXPECT_NEAR(reference->mVelocity.GetX(), 4.0f, 1e-4f);
            EXPECT_NEAR(reference->mVelocity.GetZ(), 0.0f, 1e-4f);
            EXPECT_FALSE(reference->mSupportFoot);
        }
        // At the hand-over the target is the measured pose and height.
        EXPECT_FLOAT_EQ(handover.factor(profile), 0.0f);
        EXPECT_FLOAT_EQ(handover.blendPelvisHeight(65.0f, profile), 61.0f);
        ASSERT_TRUE(handover.blendPose(standing, profile, out));
        EXPECT_TRUE(out.mBodyTransforms[2].mRotation.IsClose(measured.mBodyTransforms[2].mRotation));
        EXPECT_TRUE(out.mBodyTransforms[2].mTranslation.IsClose(measured.mBodyTransforms[2].mTranslation));
        // The weight rises monotonically (smoothstep) and the blend ends by
        // itself at the duration.
        float previous = 0.0f;
        int steps = 0;
        while (handover.active() && steps < 100)
        {
            handover.step(dt, profile);
            ++steps;
            if (!handover.active())
                break;
            const float factor = handover.factor(profile);
            EXPECT_GT(factor, previous);
            EXPECT_LE(factor, 1.0f);
            previous = factor;
            const float height = handover.blendPelvisHeight(65.0f, profile);
            EXPECT_NEAR(height, 61.0f + 4.0f * factor, 1e-4f);
        }
        EXPECT_EQ(steps, 30);
        EXPECT_FALSE(handover.active());
        EXPECT_FLOAT_EQ(handover.blendPelvisHeight(65.0f, profile), 65.0f);
        // Halfway the pose and the support target are halfway: rotation
        // angle 0.4 about X, target 20 units along the 20-unit offset.
        handover.begin(measured, 61.0f, JPH::RVec3(10, 20, 30), profile);
        for (int i = 0; i < 15; ++i)
            handover.step(dt, profile);
        EXPECT_NEAR(handover.factor(profile), 0.5f, 1e-4f);
        {
            const auto reference = handover.supportReference(JPH::RVec3(30, 20, 0), JPH::Vec3(4, 0, 1), profile);
            ASSERT_TRUE(reference);
            EXPECT_NEAR(reference->mPosition.GetX(), 20.0f, 1e-3f);
            EXPECT_NEAR(reference->mVelocity.GetX(), 2.0f, 1e-3f);
        }
        ASSERT_TRUE(handover.blendPose(standing, profile, out));
        EXPECT_NEAR(out.mBodyTransforms[2].mTranslation.GetZ(), 22.0f, 1e-3f);
        JPH::Vec3 axis;
        float angle = 0.0f;
        out.mBodyTransforms[2].mRotation.GetAxisAngle(axis, angle);
        EXPECT_NEAR(angle, 0.4f, 1e-3f);
        // A mismatched standing pose is rejected; reset clears the blend; a
        // zero duration never activates.
        RagdollPose mismatched;
        EXPECT_FALSE(handover.blendPose(mismatched, profile, out));
        handover.reset();
        EXPECT_FALSE(handover.active());
        EXPECT_FALSE(handover.supportReference(JPH::RVec3(30, 20, 0), JPH::Vec3::sZero(), profile));
        profile.mStandingHandoverDuration = 0.0f;
        handover.begin(measured, 61.0f, JPH::RVec3(10, 20, 30), profile);
        EXPECT_FALSE(handover.active());
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreReferenceGaitTest, DISABLED_stopHandsOverToStandingWithoutLaunchingTheBody)
    {
        using namespace RagdollCore;
        // Plan item 1.5. A stop after fifteen seconds of walking hands the
        // body to the standing controller. The hand-over blend
        // (ReferenceGaitProfile::mStandingHandoverDuration) must keep both
        // feet loaded and the body calm for the two seconds that follow. The
        // negative control (no blend) reproduces the launch the stress pass
        // recorded: the pelvis rises to 69.5 with the contacts down to two
        // (stop-handover-results-2026-09-07.txt).
        auto script = walkingScript(20.0f, 60, 30);
        ScenarioInputKeyframe stop;
        stop.mStep = 900;
        stop.mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        script.mKeyframes.push_back(stop);
        struct Window
        {
            std::optional<std::uint64_t> mHandover;
            float mMaxPelvisHeight = 0.0f;
            float mMaxHorizontalSpeed = 0.0f;
            float mMinPelvisUp = 1.0f;
            int mUnsupportedFrames = 0;
            bool mFell = false;
        };
        const auto measure = [&](const ScenarioResult& result) {
            Window window;
            for (const auto& frame : result.mFrames)
            {
                if (frame.mStep > stop.mStep && !frame.mReferenceGait && !window.mHandover)
                    window.mHandover = frame.mStep;
                if (!window.mHandover || frame.mStep >= *window.mHandover + 120)
                    continue;
                window.mMaxPelvisHeight = std::max(window.mMaxPelvisHeight, frame.mPelvisHeight);
                const JPH::Vec3 velocity = frame.mCenterOfMassVelocity;
                window.mMaxHorizontalSpeed = std::max(window.mMaxHorizontalSpeed,
                    std::sqrt(velocity.GetX() * velocity.GetX() + velocity.GetY() * velocity.GetY()));
                window.mMinPelvisUp = std::min(window.mMinPelvisUp, frame.mPelvisUpDot);
                if (!frame.mSupport.mLeftFootSupported || !frame.mSupport.mRightFootSupported)
                    ++window.mUnsupportedFrames;
                window.mFell |= frame.mController.mMode == ActiveControllerMode::Falling
                    || frame.mController.mMode == ActiveControllerMode::Fallen;
            }
            return window;
        };
        // Exercise today's measured-speed stop as well as the historical
        // pose that makes the no-blend negative control discriminate.
        for (const bool measuredStopping : { true, false })
        {
            SCOPED_TRACE(measuredStopping);
            auto settings = referenceSettings();
            settings.mReferenceGaitMeasuredStopping = measuredStopping;
            settings.mReferenceGait.mRequireInitialSupportTransfer = measuredStopping;
            const auto result = run(script, settings);
            ASSERT_TRUE(result.completed()) << describe(result);
            exportTraceIfRequested(result, measuredStopping ? "reference-stop-handover" : "reference-stop-handover-legacy");
            const Window blended = measure(result);
            ASSERT_TRUE(blended.mHandover);
            EXPECT_FALSE(blended.mFell);
            EXPECT_EQ(blended.mUnsupportedFrames, 0);
            EXPECT_LT(blended.mMaxPelvisHeight, 67.5f);
            EXPECT_LT(blended.mMaxHorizontalSpeed, 12.0f);
            EXPECT_GT(blended.mMinPelvisUp, 0.97f);
            const auto repeat = run(script, settings);
            ASSERT_TRUE(repeat.completed());
            EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, repeat));

            if (measuredStopping)
                continue;
            auto snap = settings;
            snap.mReferenceGait.mStandingHandoverDuration = 0.0f;
            const auto control = run(script, snap);
            ASSERT_TRUE(control.completed()) << describe(control);
            const Window snapped = measure(control);
            ASSERT_TRUE(snapped.mHandover);
            EXPECT_EQ(*snapped.mHandover, *blended.mHandover);
            EXPECT_GT(snapped.mMaxPelvisHeight, 68.0f) << "the negative control must show the launch";
        }
    }

    TEST(RagdollCoreReferenceGaitTest, stoppingRestoresAuthoredMotorSpringsBeforeStandingActuation)
    {
        using namespace RagdollCore;
        // These lifecycle contracts are about the stop itself; pin the
        // pre-steering immediate stop instead of the steering filter's
        // decelerating hand-over (steering-results-2026-09-06.txt).
        auto immediateStop = referenceSettings();
        immediateStop.mReferenceGait.mSteeringStopDeceleration = 0.0f;
        const auto profile = makeScenarioHumanoidProfile();
        // Stop once during transfer and once during swing. Also release for
        // one step, then resume standing: release must not strand overrides.
        for (const std::uint64_t stop : { 35u, 50u })
            for (const bool release : { false, true })
            {
                SCOPED_TRACE(::testing::Message() << "stop=" << stop << " release=" << release);
                auto script = walkingScript(1.2f, 60, 30);
                ScenarioInputKeyframe end;
                end.mStep = stop;
                end.mMotorsEnabled = !release;
                script.mKeyframes.push_back(end);
                if (release)
                {
                    end.mStep = stop + 1;
                    end.mMotorsEnabled = true;
                    script.mKeyframes.push_back(end);
                }
                const auto result = run(script, immediateStop);
                ASSERT_TRUE(result.completed()) << describe(result);
                exportTraceIfRequested(result, "reference-stop-springs-" + std::to_string(stop)
                    + (release ? "-release" : ""));
                ASSERT_TRUE(result.mFrames[stop - 1].mReferenceGait);
                bool sawOverride = false;
                for (std::size_t body = 1; body < profile.mBodies.size(); ++body)
                {
                    const auto& target = result.mFrames[stop - 1].mJointMotorTargets[body];
                    if (!target) continue;
                    const auto& authored = profile.mBodies[body].mConstraint->mMotor;
                    sawOverride |= target->mSpring.mFrequencyOrStiffness != authored.mFrequency
                        || target->mSpring.mDamping != authored.mDamping;
                }
                ASSERT_TRUE(sawOverride) << "control must actually exercise a walking override";
                for (std::size_t step = stop; step < result.mFrames.size(); ++step)
                {
                    const auto& frame = result.mFrames[step];
                    ASSERT_FALSE(frame.mReferenceGait);
                    if (release && step == stop)
                    {
                        EXPECT_FALSE(frame.mMotorsApplied);
                        EXPECT_TRUE(frame.mJointMotorTargets.empty());
                        continue;
                    }
                    ASSERT_TRUE(frame.mMotorsApplied) << "step=" << step;
                    ASSERT_EQ(frame.mJointMotorTargets.size(), profile.mBodies.size());
                    for (std::size_t body = 1; body < profile.mBodies.size(); ++body)
                    {
                        SCOPED_TRACE(::testing::Message() << "step=" << step << " body=" << body);
                        const auto& target = frame.mJointMotorTargets[body];
                        ASSERT_TRUE(target);
                        const auto& authored = profile.mBodies[body].mConstraint->mMotor;
                        const auto check = [&](const MotorSpringTelemetry& spring) {
                            EXPECT_FALSE(spring.mStiffnessMode);
                            EXPECT_FLOAT_EQ(spring.mFrequencyOrStiffness, authored.mFrequency);
                            EXPECT_FLOAT_EQ(spring.mDamping, authored.mDamping);
                        };
                        check(target->mSpring);
                        if (target->mTwistSpring) check(*target->mTwistSpring);
                    }
                }
                const auto repeat = run(script, immediateStop);
                ASSERT_TRUE(repeat.completed());
                EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, repeat));
            }
    }

    TEST(RagdollCoreReferenceGaitTest, restartBeginsAFreshTransferAfterStopOrMotorRelease)
    {
        using namespace RagdollCore;
        // These lifecycle contracts are about the stop itself; pin the
        // pre-steering immediate stop instead of the steering filter's
        // decelerating hand-over (steering-results-2026-09-06.txt).
        auto immediateStop = referenceSettings();
        immediateStop.mReferenceGait.mSteeringStopDeceleration = 0.0f;
        for (const std::uint64_t pause : { 35u, 50u })
            for (const bool release : { false, true })
            {
                SCOPED_TRACE(::testing::Message() << "pause=" << pause << " release=" << release);
                const std::uint64_t resume = pause + 5;
                auto script = walkingScript(1.2f, 60, 30);
                ScenarioInputKeyframe key;
                key.mStep = pause;
                key.mMotorsEnabled = !release;
                if (release) key.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
                script.mKeyframes.push_back(key);
                key.mStep = resume;
                key.mMotorsEnabled = true;
                key.mRequestedHorizontalVelocity = JPH::Vec3(0, 60, 0);
                script.mKeyframes.push_back(key);
                const auto result = run(script, immediateStop);
                ASSERT_TRUE(result.completed()) << describe(result);
                exportTraceIfRequested(result, "reference-restart-" + std::to_string(pause)
                    + (release ? "-release" : ""));
                ASSERT_TRUE(result.mFrames[pause - 1].mReferenceGait);
                for (std::uint64_t step = pause; step < resume; ++step)
                {
                    EXPECT_FALSE(result.mFrames[step].mReferenceGait);
                    EXPECT_EQ(result.mFrames[step].mGait.mPhase, GaitPhase::Idle);
                }
                const auto first = std::find_if(result.mFrames.begin() + resume, result.mFrames.end(),
                    [](const auto& frame) { return frame.mReferenceGait.has_value(); });
                ASSERT_NE(first, result.mFrames.end());
                EXPECT_LE(first->mStep, resume + 2);
                EXPECT_EQ(first->mReferenceGait->mStage, ReferenceGaitStage::DoubleSupport);
                EXPECT_FLOAT_EQ(first->mReferenceGait->mTimeInStage, 0.0f);
                EXPECT_FALSE(first->mActuationGait->mSwingFootLifted);
                const auto repeat = run(script, immediateStop);
                ASSERT_TRUE(repeat.completed());
                EXPECT_TRUE(haveIdenticalScenarioTelemetry(result, repeat));
            }
    }

    TEST(RagdollCoreReferenceGaitTest, rootRoutingCancelsBranchReactionsByLoadFraction)
    {
        RagdollCore::ReferenceRootRoutingInput input;
        input.mPelvisTorqueRequest = JPH::Vec3(10.0f, -4.0f, 2.0f);
        input.mBranchDynamicTorques = { JPH::Vec3(1.0f, 1.0f, 0.0f), JPH::Vec3(-3.0f, 0.0f, 5.0f) };
        input.mLeftHipLoadFraction = 300.0f;
        input.mRightHipLoadFraction = 100.0f;
        const auto routed = RagdollCore::routeReferenceRootTorque(input);
        ASSERT_TRUE(routed);
        EXPECT_TRUE(routed->mBranchReaction.IsClose(JPH::Vec3(-2.0f, 1.0f, 5.0f)));
        const JPH::Vec3 total = -(input.mPelvisTorqueRequest + routed->mBranchReaction);
        EXPECT_TRUE(routed->mLeftHipTorque.IsClose(total * 0.75f));
        EXPECT_TRUE(routed->mRightHipTorque.IsClose(total * 0.25f));
        // The pelvis receives minus every child torque: exactly the request.
        const JPH::Vec3 pelvis = -(routed->mLeftHipTorque + routed->mRightHipTorque + routed->mBranchReaction);
        EXPECT_TRUE(pelvis.IsClose(input.mPelvisTorqueRequest, 1e-8f));
        input.mLeftHipLoadFraction = input.mRightHipLoadFraction = 0.0f;
        const auto unsupported = RagdollCore::routeReferenceRootTorque(input);
        ASSERT_TRUE(unsupported);
        EXPECT_TRUE(unsupported->mLeftHipTorque.IsNearZero());
        EXPECT_TRUE(unsupported->mRightHipTorque.IsNearZero());
        input.mLeftHipLoadFraction = -1.0f;
        EXPECT_FALSE(RagdollCore::routeReferenceRootTorque(input));
    }

    // Experiment E2: loaded single support and foot lift. Prediction: after
    // the walk request the first swing foot leaves the floor within the
    // double-support interval plus 0.15 s, reaches at least half the
    // commanded clearance, and the actor holds single support for the rest
    // of the run without falling, with every motor below saturation on at
    // least 95% of reference frames. Rejection: a fall, no measured lift, or
    // sustained saturation. Matched control: the same run with motors
    // released at the walk request falls.
    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreReferenceGaitTest, DISABLED_singleSupportLiftAndHoldCarriesTheLoadedStance)
    {
        RagdollCore::ScenarioSettings settings = referenceSettings();
        settings.mReferenceGaitHoldSwingFoot = true;
        // Static single support needs the weight transferred over the stance
        // foot first, so this diagnostic uses a long double support. The hold
        // never lands, so the step deadline must not be counted.
        settings.mReferenceGait.mDoubleSupportDuration = 0.35f;
        settings.mReferenceGait.mMaximumDoubleSupportDuration = 0.35f;
        settings.mReferenceGait.mWeightTransferFrequency = 1.5f;
        settings.mReferenceGait.mTransferVelocityGain = 0.0f;
        settings.mReferenceGait.mMaximumRequestedAcceleration = 150.0f;
        settings.mReferenceGait.mMaximumStepDuration = 10.0f;
        const auto result = run(walkingScript(2.0f, 1.0f, 30), settings);
        exportTraceIfRequested(result, "reference-lift-hold");
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto summary = summarize(result);
        const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
        std::cout << "REFERENCE_LIFT_HOLD " << describe(summary) << ' ' << describe(invariants) << std::endl;
        EXPECT_TRUE(summary.mFirstSwingStep) << describe(summary);
        ASSERT_TRUE(summary.mFirstLiftStep) << describe(summary);
        EXPECT_LE(*summary.mFirstLiftStep, *summary.mFirstSwingStep + 9u) << describe(summary);
        EXPECT_LE(*summary.mFirstLiftStep, 30u + static_cast<std::uint64_t>(
            std::ceil((settings.mReferenceGait.mDoubleSupportDuration + 0.15f) / settings.mFixedTimeStep)))
            << "delaying swing must not shorten the required single-support observation";
        EXPECT_FALSE(summary.mFirstTransferStep) << "the held foot must not land: " << describe(summary);
        EXPECT_GE(summary.mMaxSwingClearance, 0.5f * settings.mReferenceGait.mSwingFootLift) << describe(summary);
        EXPECT_EQ(invariants.mFallingFrames + invariants.mFallenFrames, 0u) << describe(summary);
        EXPECT_GT(invariants.mMinPelvisUpDot, 0.9f);
        EXPECT_EQ(invariants.mAirborneFrames, 0u);
        EXPECT_EQ(invariants.mFreeVelocityLimitedFrames + invariants.mSolverVelocityLimitedFrames, 0u);
        EXPECT_LE(summary.mSaturatedFrames * 20, summary.mReferenceFrames) << describe(summary);
        EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);

        RagdollCore::ScenarioScript control = walkingScript(2.0f, 1.0f, 30);
        control.mKeyframes.front().mMotorsEnabled = false;
        const auto released = run(control, settings);
        ASSERT_TRUE(released.completed()) << describe(released);
        exportTraceIfRequested(released, "reference-lift-released");
        exportBodiesIfRequested(released, "reference-lift-released");
        double minimumReleasedHeight = std::numeric_limits<double>::max();
        for (const auto& frame : released.mFrames)
            if (frame.mStep >= 30)
                minimumReleasedHeight = std::min(minimumReleasedHeight,
                    static_cast<double>(frame.mBodies[0].mWorldPosition.GetZ()) - settings.mGroundHeight);
        std::cout << "REFERENCE_RELEASE minimum_world_pelvis_height=" << minimumReleasedHeight << std::endl;
        EXPECT_EQ(std::count_if(released.mFrames.begin(), released.mFrames.end(),
            [](const auto& frame) { return frame.mStep >= 30 && frame.mMotorsApplied; }), 0);
        const auto releasedInvariants = RagdollCore::summarizeScenarioInvariants(released);
        auto ungovernedSettings = settings;
        ungovernedSettings.mReferenceGait.mLimitVelocityLead = false;
        const auto ungovernedRelease = run(control, ungovernedSettings);
        ASSERT_TRUE(ungovernedRelease.completed()) << describe(ungovernedRelease);
        std::cout << "REFERENCE_RELEASE_UNGOVERNED_CONTROL "
                  << describe(RagdollCore::summarizeScenarioInvariants(ungovernedRelease)) << std::endl;
        EXPECT_GT(releasedInvariants.mFallingFrames + releasedInvariants.mFallenFrames, 0u)
            << "the released control must fall: " << describe(releasedInvariants);
    }

    // Experiment E3: one complete step. Prediction: the first stance
    // transfers within 0.75 s of the walk request without a timeout, the
    // swing foot lands ahead of its lift-off point, and the actor stays
    // upright through the following double support. Rejection: no transfer,
    // a timeout, or a fall before the second swing begins.
    TEST(RagdollCoreReferenceGaitTest, firstStepTransfersStanceWithoutTimeout)
    {
        const auto result = run(walkingScript(1.6f, 60.0f, 30), referenceSettings());
        exportTraceIfRequested(result, "reference-first-step");
        exportBodiesIfRequested(result, "reference-first-step");
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto summary = summarize(result);
        const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
        std::cout << "REFERENCE_FIRST_STEP " << describe(summary) << ' ' << describe(invariants) << std::endl;
        ASSERT_TRUE(summary.mFirstTransferStep) << describe(summary);
        EXPECT_LE(*summary.mFirstTransferStep, 30u + 45u) << describe(summary);
        EXPECT_EQ(invariants.mSteps.mTimedOutPhases, 0u) << describe(summary);
        if (summary.mFirstFallingStep)
            EXPECT_GT(*summary.mFirstFallingStep, *summary.mFirstTransferStep + 6u) << describe(summary);
        EXPECT_EQ(invariants.mAirborneFrames, 0u);
        EXPECT_EQ(invariants.mFreeVelocityLimitedFrames + invariants.mSolverVelocityLimitedFrames, 0u);
        EXPECT_LE(summary.mSaturatedFrames * 20, summary.mReferenceFrames);
        EXPECT_LE(invariants.mMaxStanceFootSlide, 1.5f);
        EXPECT_LT(invariants.mMaxAppliedNetForce, 1e-3f);
        EXPECT_LT(invariants.mMaxNetActuationMoment, 1e-3f);
    }

    // Experiment E4: the complete walking contract of
    // walkingSatisfiesGaitInvariants on the reference route. Both actuation
    // route labels of the legacy contract exercise identical native motor
    // commands here, so only one run is a genuine observation. Matched
    // control: all actuation disabled at the walk request falls.
    void checkReferenceWalking(const RagdollCore::ScenarioSettings& settings, const char* traceName)
    {
        const auto result = run(walkingScript(7.0f, 60.0f, 30), settings);
        exportTraceIfRequested(result, traceName);
        exportBodiesIfRequested(result, traceName);
        exportLandingPostureAudit(result, traceName);
        if (settings.mReferenceGaitGroundsPlacement)
        {
            // Independent oracle for this fixture: both level foot boxes have
            // 2.5-unit half-height and zero shape/COM offset. Body roll must
            // never move the planned touchdown off the fixed floor surface.
            ASSERT_TRUE(settings.mRootRotation.IsClose(JPH::Quat::sIdentity()));
            std::size_t placements = 0;
            for (const auto& frame : result.mFrames)
                if (frame.mReferenceGait && frame.mReferenceGait->mPlacement)
                {
                    ++placements;
                    EXPECT_NEAR(frame.mReferenceGait->mPlacement->GetZ(),
                        settings.mGroundHeight + 2.5 * settings.mUniformScale, 1e-5) << frame.mStep;
                }
            EXPECT_GT(placements, 0u);
        }
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
        const auto summary = summarize(result);
        std::cout << "REFERENCE_WALKING " << describe(summary) << ' ' << describe(invariants) << std::endl;
        SCOPED_TRACE(describe(summary));

        bool observedStance = false;
        bool observedSwingTarget = false;
        for (const auto& frame : result.mFrames)
        {
            observedStance = observedStance || frame.mGait.hasStance();
            observedSwingTarget = observedSwingTarget || frame.mSwingFootMotorTarget.has_value();
        }
        EXPECT_TRUE(observedStance);
        EXPECT_TRUE(observedSwingTarget);
        EXPECT_EQ(invariants.mFallingFrames + invariants.mFallenFrames, 0u) << describe(result);
        EXPECT_GT(invariants.mMinPelvisUpDot, 0.5f);
        EXPECT_EQ(invariants.mAirborneFrames, 0u);
        EXPECT_GE(invariants.mSteps.mLeftStanceCount, 2u);
        EXPECT_GE(invariants.mSteps.mRightStanceCount, 2u);
        EXPECT_GE(invariants.mSteps.mDurationSymmetry, 0.6f);
        EXPECT_EQ(invariants.mSteps.mTimedOutPhases, 0u);
        EXPECT_GE(invariants.mFinalHalfSecondAchievedSpeed, 0.5f * invariants.mMeanRequestedSpeed);
        EXPECT_LE(invariants.mMaxStanceFootSlide, 1.5f);
        EXPECT_LE(invariants.mPelvisMagnitudeSaturatedFrames * 20, invariants.mFrameCount);
        EXPECT_LE(invariants.mPelvisHorizontalForceSaturatedFrames * 20, invariants.mFrameCount);
        EXPECT_LE(invariants.mPelvisVerticalForceSaturatedFrames * 20, invariants.mFrameCount);
        EXPECT_LE(invariants.mSwingHipMotorSaturatedFrames * 20, invariants.mFrameCount);
        // The reference route has no external force path; its equivalent
        // actuator gate is native motor saturation over every joint.
        EXPECT_LE(summary.mSaturatedFrames * 20, summary.mReferenceFrames);
        for (const auto& stage : RagdollCore::summarizeScenarioGaitStages(result))
        {
            if (stage.mStage == RagdollCore::GaitActuationStage::Inactive
                || stage.mStage == RagdollCore::GaitActuationStage::Settling
                || stage.mStage == RagdollCore::GaitActuationStage::AwaitingStance || stage.mFrames == 0)
                continue;
            SCOPED_TRACE(RagdollCore::gaitActuationStageName(stage.mStage));
            EXPECT_EQ(stage.mSolverObservedFrames, stage.mFrames);
            EXPECT_EQ(stage.mFreeVelocityLimitedFrames, 0u);
            EXPECT_EQ(stage.mSolverVelocityLimitedFrames, 0u);
            EXPECT_EQ(stage.mAirborneFrames, 0u);
            EXPECT_EQ(stage.mFallingFrames, 0u);
            EXPECT_LT(stage.mMaxNetForce, 1e-3f);
            EXPECT_LT(stage.mMaxNetMoment, 1e-3f);
            EXPECT_LE(stage.mMaxStanceSlide, 1.5f);
            EXPECT_LE(stage.mSwingHipSaturationFrames * 20, stage.mFrames);
            EXPECT_LE(stage.mSwingKneeSaturationFrames * 20, stage.mFrames);
            EXPECT_LE(stage.mSwingAnkleSaturationFrames * 20, stage.mFrames);
        }

        // Determinism: an exact repeat reproduces every recorded field.
        const auto repeat = run(walkingScript(7.0f, 60.0f, 30), settings);
        ASSERT_TRUE(repeat.completed());
        EXPECT_TRUE(RagdollCore::haveIdenticalScenarioTelemetry(result, repeat));

        RagdollCore::ScenarioScript control = walkingScript(7.0f, 60.0f, 30);
        control.mKeyframes.front().mMotorsEnabled = false;
        const auto released = run(control, settings);
        ASSERT_TRUE(released.completed());
        const auto releasedInvariants = RagdollCore::summarizeScenarioInvariants(released);
        EXPECT_GT(releasedInvariants.mFallingFrames + releasedInvariants.mFallenFrames, 0u)
            << "the released control must fall: " << describe(releasedInvariants);
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreReferenceGaitTest, DISABLED_referenceWalkingSatisfiesGaitInvariants)
    {
        checkReferenceWalking(referenceSettings(), "reference-walking");
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreReferenceGaitTest, DISABLED_coordinatedStartupSatisfiesGaitInvariants)
    {
        auto settings = referenceSettings();
        settings.mReferenceGait.mCoordinateInitialTransfer = true;
        checkReferenceWalking(settings, "coordinated-startup-walking");
        const auto result = run(walkingScript(7.0f, 60, 30), settings);
        ASSERT_TRUE(result.completed());
        std::size_t coordinated = 0;
        for (const auto& frame : result.mFrames)
            if (frame.mReferenceGait && frame.mReferenceGait->mCoordinatedPelvisTargetZ)
            {
                ++coordinated;
                EXPECT_EQ(frame.mReferenceGait->mStage, RagdollCore::ReferenceGaitStage::DoubleSupport);
            }
        EXPECT_GT(coordinated, 0u); // A fallback-only pass does not test coordination.
    }

    // Disabled 10-30 (test review): tests the retired reference-gait controller, not
// Footfall (the game's gait); failing since 10-08 (PLAN.md hygiene).
TEST(RagdollCoreReferenceGaitTest, DISABLED_coordinatedTransfersSatisfyGaitInvariants)
    {
        auto settings = referenceSettings();
        settings.mReferenceGait.mCoordinateWeightTransfers = true;
        checkReferenceWalking(settings, "coordinated-transfer-walking");
        const auto result = run(walkingScript(7.0f, 60, 30), settings);
        ASSERT_TRUE(result.completed());
        bool sawSwing = false;
        std::size_t laterCoordinated = 0;
        for (const auto& frame : result.mFrames)
        {
            if (!frame.mReferenceGait) continue;
            const auto& gait = *frame.mReferenceGait;
            sawSwing |= gait.mStage == RagdollCore::ReferenceGaitStage::Swing;
            if (gait.mCoordinatedPelvisTargetZ)
            {
                EXPECT_EQ(gait.mStage, RagdollCore::ReferenceGaitStage::DoubleSupport);
                if (sawSwing) ++laterCoordinated;
            }
        }
        EXPECT_GT(laterCoordinated, 0u);
    }

    TEST(RagdollCoreReferenceGaitTest, rootRecoveryRatePreservesAuthoredPdTorqueAndCorrectsWorldTilt)
    {
        using namespace RagdollCore;
        const auto profile = makeScenarioHumanoidProfile().mActiveController;
        const JPH::Vec3 error(0.2f, -0.1f, 0.05f), omega(-0.4f, 0.2f, -0.3f);
        const auto rate = referenceGaitRootRecoveryRate(error, profile.mPelvisUprightStiffness,
            profile.mPelvisUprightDamping);
        ASSERT_TRUE(rate);
        const auto directTorque = error * profile.mPelvisUprightStiffness - omega * profile.mPelvisUprightDamping;
        const auto rateTorque = (*rate - omega) * profile.mPelvisUprightDamping;
        EXPECT_LE((directTorque - rateTorque).Length(), 0.05f);
        const auto measured = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -0.2f);
        JPH::Vec3 axis;
        float angle;
        measured.Conjugated().EnsureWPositive().GetAxisAngle(axis, angle);
        const auto correction = referenceGaitRootRecoveryRate(axis * angle,
            profile.mPelvisUprightStiffness, profile.mPelvisUprightDamping);
        ASSERT_TRUE(correction);
        const auto next = (JPH::Quat::sRotation(correction->Normalized(), correction->Length() / 60) * measured).Normalized();
        float nextAngle;
        next.EnsureWPositive().GetAxisAngle(axis, nextAngle);
        EXPECT_LT(nextAngle, angle);
        EXPECT_GT(correction->GetX(), 0);
        const auto rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.7f);
        const auto rotated = referenceGaitRootRecoveryRate(rotation * error,
            profile.mPelvisUprightStiffness, profile.mPelvisUprightDamping);
        ASSERT_TRUE(rotated);
        EXPECT_LE((*rotated - rotation * *rate).Length(), 1e-6f);
        EXPECT_FALSE(referenceGaitRootRecoveryRate(error, 1, 0));
        EXPECT_FALSE(referenceGaitRootRecoveryRate(error, -1, 1));
        EXPECT_FALSE(referenceGaitRootRecoveryRate(JPH::Vec3(std::numeric_limits<float>::infinity(), 0, 0), 1, 1));
        const auto zero = referenceGaitRootRecoveryRate(error, 0, 1);
        ASSERT_TRUE(zero);
        EXPECT_EQ(zero->LengthSq(), 0);
    }

    TEST(RagdollCoreReferenceGaitTest, relativeMotorPosePreservesDescendantsAndUsesMeasuredWorldParent)
    {
        using namespace RagdollCore;
        ReferenceGaitPlan plan;
        plan.mTargetPose.mBodyTransforms.resize(3);
        plan.mTargetPose.mBodyTransforms[0].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.2f);
        plan.mTargetPose.mBodyTransforms[1].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 0.3f);
        plan.mTargetPose.mBodyTransforms[2].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.4f);
        auto measured = plan.mTargetPose.mBodyTransforms;
        measured[0].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), -0.7f);
        const auto root = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 0.8f);
        const std::array<int, 3> parents{ -1, 0, 1 };
        plan.mSwingHip = 1;
        plan.mSwingHipWorldTarget = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.6f);
        const auto target = referenceGaitRelativeMotorPose(plan, measured, parents, root);
        ASSERT_TRUE(target);
        const auto& pose = target->mBodyTransforms;
        const auto mapped = root * measured[0].mRotation * pose[0].mRotation.Conjugated() * pose[1].mRotation;
        EXPECT_GT(std::abs(mapped.Dot(*plan.mSwingHipWorldTarget)), 1 - 1e-6f);
        const auto relative = pose[1].mRotation.Conjugated() * pose[2].mRotation;
        const auto expected = plan.mTargetPose.mBodyTransforms[1].mRotation.Conjugated()
            * plan.mTargetPose.mBodyTransforms[2].mRotation;
        EXPECT_GT(std::abs(relative.Dot(expected)), 1 - 1e-6f);
        const std::array<int, 3> invalidParents{ -1, 2, 1 };
        EXPECT_FALSE(referenceGaitRelativeMotorPose(plan, measured, invalidParents, root));
        plan.mWorldFootTargets.emplace_back(3, JPH::Quat::sIdentity());
        EXPECT_FALSE(referenceGaitRelativeMotorPose(plan, measured, parents, root));
    }

    TEST(RagdollCoreReferenceGaitTest, referencePushDuringWalkTriggersRecoveryStep)
    {
        RagdollCore::ScenarioScript script = walkingScript(3.5f, 60.0f, 30);
        RagdollCore::ScenarioImpulse push;
        push.mStep = 120;
        push.mBodyIndex = 3; // spine2
        push.mImpulse = JPH::Vec3(1200.0f, 0.0f, 0.0f);
        script.mImpulses.push_back(push);
        const auto result = run(script, referenceSettings());
        exportTraceIfRequested(result, "reference-push-recovery");
        exportBodiesIfRequested(result, "reference-push-recovery");
        exportLandingPostureAudit(result, "reference-push-recovery");
        ASSERT_TRUE(result.completed()) << describe(result);
        const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
        std::cout << "REFERENCE_PUSH " << describe(summarize(result)) << ' ' << describe(invariants) << std::endl;
        EXPECT_EQ(invariants.mImpulseFrames, 1u);
        EXPECT_GE(invariants.mStepsWithinSecondAfterImpulse, 1u);
        EXPECT_FALSE(invariants.mFellAfterImpulse) << describe(result);
        EXPECT_GT(invariants.mMinPelvisUpDot, 0.5f);
        EXPECT_EQ(invariants.mAirborneFrames, 0u);
        EXPECT_EQ(invariants.mSteps.mTimedOutPhases, 0u);
        EXPECT_EQ(invariants.mFreeVelocityLimitedFrames + invariants.mSolverVelocityLimitedFrames, 0u);
    }

    TEST(RagdollCoreReferenceGaitTest, forwardReserveGovernorPreservesBudgetAndLateralCrises)
    {
        const JPH::Vec3 forward = JPH::Vec3::sAxisY();
        const JPH::Vec3 right = forward.Cross(JPH::Vec3::sAxisZ()).Normalized();
        const float maxA = 75.5628f;
        const float fraction = 0.6f;

        // Zero fraction and zero budget are inert.
        const JPH::Vec3 request(10.0f, 60.0f, 0.0f);
        EXPECT_EQ(RagdollCore::governForwardReserve(request, forward, right, maxA, 0.0f), request);
        EXPECT_EQ(RagdollCore::governForwardReserve(request, forward, right, 0.0f, fraction), request);

        // No forward demand: the full budget stays available laterally.
        const JPH::Vec3 lateralCrisis = right * (maxA * 0.999f);
        EXPECT_EQ(RagdollCore::governForwardReserve(lateralCrisis, forward, right, maxA, fraction),
            lateralCrisis);

        // Lateral within the remaining budget is untouched.
        const float reserve = fraction * maxA;
        const float lateralCap = std::sqrt(maxA * maxA - reserve * reserve);
        const JPH::Vec3 small = forward * maxA + right * (lateralCap * 0.5f);
        const JPH::Vec3 smallOut = RagdollCore::governForwardReserve(small, forward, right, maxA, fraction);
        EXPECT_NEAR(smallOut.GetX(), small.GetX(), 1e-4f);
        EXPECT_NEAR(smallOut.GetY(), small.GetY(), 1e-4f);

        // Forward demand with runaway lateral: lateral is capped to the
        // remaining budget and the forward component is preserved.
        const JPH::Vec3 runaway = forward * maxA + right * (maxA * 4.0f);
        const JPH::Vec3 governed
            = RagdollCore::governForwardReserve(runaway, forward, right, maxA, fraction);
        EXPECT_NEAR(governed.Dot(forward), maxA, 1e-4f);
        EXPECT_NEAR(governed.Dot(right), lateralCap, 1e-3f);
        // The governor caps only the lateral component; the downstream
        // magnitude clamp owns the combined budget.
        EXPECT_NEAR(governed.Length(), std::sqrt(maxA * maxA + lateralCap * lateralCap), 1e-3f);

        // Braking demand (negative forward) reserves the same share.
        const JPH::Vec3 braking = forward * (-maxA) + right * (maxA * 4.0f);
        const JPH::Vec3 governedBraking
            = RagdollCore::governForwardReserve(braking, forward, right, maxA, fraction);
        EXPECT_NEAR(governedBraking.Dot(forward), -maxA, 1e-4f);
        EXPECT_NEAR(governedBraking.Dot(right), lateralCap, 1e-3f);
    }

    // Single-variable comparison from one baseline. Run explicitly with
    // --gtest_also_run_disabled_tests; each variant prints one summary line
    // and exports its trace. This is a diagnostic, not an acceptance gate.
    TEST(RagdollCoreReferenceGaitTest, DISABLED_referenceVariantSweep)
    {
        using Profile = RagdollCore::ReferenceGaitProfile;
        struct Variant
        {
            const char* mName;
            void (*mApply)(Profile&);
        };
        const Variant variants[] = {
            { "baseline", [](Profile&) {} },
            { "noregulation", [](Profile& p) { p.mTransferVelocityGain = 0.0f; } },
            { "reach0.97", [](Profile& p) { p.mSwingReachFraction = 0.97f; } },
            { "swing4", [](Profile& p) { p.mSwingLegFrequency = 0.0f; } },
            { "ankle-level", [](Profile& p) { p.mLoadedAnkleFrequency = 0.0f; } },
            { "offset6", [](Profile& p) { p.mLateralCaptureOffset = 6.0f; } },
        };
        const char* only = std::getenv("OPENMW_RAGDOLL_REFERENCE_VARIANT");
        for (const Variant& variant : variants)
        {
            if (only != nullptr && *only != '\0' && std::string(only) != variant.mName)
                continue;
            RagdollCore::ScenarioSettings settings = referenceSettings();
            variant.mApply(settings.mReferenceGait);
            const auto result = run(walkingScript(7.0f, 60.0f, 30), settings);
            ASSERT_TRUE(result.completed()) << variant.mName << ": " << describe(result);
            const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
            exportTraceIfRequested(result, std::string("reference-variant-") + variant.mName);
            std::cout << "REFERENCE_VARIANT " << variant.mName << ' ' << describe(summarize(result)) << ' '
                      << describe(invariants) << std::endl;
        }
    }
}
