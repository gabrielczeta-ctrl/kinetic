// Offline reference-gait evaluation driver. Runs the seven-second walking
// contract of referenceWalkingSatisfiesGaitInvariants with profile overrides
// supplied on the command line and prints one JSON summary line, so
// parameter sweeps do not need a rebuild. Diagnostic tooling only: it
// changes no controller behaviour and no acceptance gate.

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
// clang-format on

#include "nativebody.hpp"
#include "recoverypose.hpp"
#include "recoveryclips.hpp"
#include "hybridcatch.hpp"

#include <ragdollcore/referencegait.hpp>
#include <ragdollcore/reactionmechanics.hpp>
#include <ragdollcore/reactionpipeline.hpp>
#include <ragdollcore/scenario.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <ragdollcore/replay.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace
{
    using RagdollCore::ReferenceGaitProfile;
    using RagdollCore::ScenarioSettings;

    struct FloatField
    {
        const char* mName;
        float ReferenceGaitProfile::*mMember;
    };

    const FloatField sProfileFields[] = {
        { "double_support", &ReferenceGaitProfile::mDoubleSupportDuration },
        { "swing_duration", &ReferenceGaitProfile::mSwingDuration },
        { "minimum_swing", &ReferenceGaitProfile::mMinimumSwingDuration },
        { "contact_confirmation", &ReferenceGaitProfile::mContactConfirmationDuration },
        { "transfer_load", &ReferenceGaitProfile::mTransferLoadFraction },
        { "swing_start_load", &ReferenceGaitProfile::mSwingStartLoadFraction },
        { "maximum_double_support", &ReferenceGaitProfile::mMaximumDoubleSupportDuration },
        { "maximum_step", &ReferenceGaitProfile::mMaximumStepDuration },
        { "swing_lift", &ReferenceGaitProfile::mSwingFootLift },
        { "press_speed", &ReferenceGaitProfile::mTouchdownPressSpeed },
        { "press_depth", &ReferenceGaitProfile::mMaximumTouchdownPressDepth },
        { "lateral_offset", &ReferenceGaitProfile::mLateralCaptureOffset },
        { "forward_error_gain", &ReferenceGaitProfile::mForwardVelocityErrorGain },
        { "minimum_width", &ReferenceGaitProfile::mMinimumStepWidth },
        { "maximum_width", &ReferenceGaitProfile::mMaximumStepWidth },
        { "minimum_forward", &ReferenceGaitProfile::mMinimumForwardPlacement },
        { "maximum_reach", &ReferenceGaitProfile::mMaximumStepReach },
        { "velocity_gain", &ReferenceGaitProfile::mVelocityGain },
        { "unrestricted_request_speed", &ReferenceGaitProfile::mUnrestrictedRequestSpeed },
        { "velocity_lead_fraction", &ReferenceGaitProfile::mVelocityLeadAccelerationFraction },
        { "transfer_frequency", &ReferenceGaitProfile::mWeightTransferFrequency },
        { "transfer_damping", &ReferenceGaitProfile::mWeightTransferDamping },
        { "transfer_velocity_gain", &ReferenceGaitProfile::mTransferVelocityGain },
        { "maximum_acceleration", &ReferenceGaitProfile::mMaximumRequestedAcceleration },
        { "swing_frequency", &ReferenceGaitProfile::mSwingLegFrequency },
        { "swing_damping", &ReferenceGaitProfile::mSwingLegDamping },
        { "stance_frequency", &ReferenceGaitProfile::mStanceLegFrequency },
        { "stance_damping", &ReferenceGaitProfile::mStanceLegDamping },
        { "stance_knee_flexion", &ReferenceGaitProfile::mStanceKneeFlexion },
        { "placement_latch", &ReferenceGaitProfile::mPlacementLatchFraction },
        { "trunk_frequency", &ReferenceGaitProfile::mTrunkFrequency },
        { "trunk_damping", &ReferenceGaitProfile::mTrunkDamping },
        { "loaded_ankle_frequency", &ReferenceGaitProfile::mLoadedAnkleFrequency },
        { "loaded_ankle_damping", &ReferenceGaitProfile::mLoadedAnkleDamping },
        { "swing_reach", &ReferenceGaitProfile::mSwingReachFraction },
        { "support_hull_scale", &ReferenceGaitProfile::mSupportHullScale },
        { "lost_landing_swap", &ReferenceGaitProfile::mLostLandingSwapDelay },
        { "landing_press", &ReferenceGaitProfile::mLandingPressDepth },
        { "rise_end", &ReferenceGaitProfile::mSwingRiseEndFraction },
        { "travel_end", &ReferenceGaitProfile::mSwingTravelEndFraction },
        { "press_start", &ReferenceGaitProfile::mTouchdownPressStartFraction },
        { "stance_hip_frequency", &ReferenceGaitProfile::mStanceHipFrequency },
        { "stance_hip_damping", &ReferenceGaitProfile::mStanceHipDamping },
        { "travel_start", &ReferenceGaitProfile::mSwingTravelStartFraction },
        { "swing_inertia_ff", &ReferenceGaitProfile::mSwingInertiaFeedForward },
        { "swing_ff_accel_limit", &ReferenceGaitProfile::mSwingFeedForwardAccelerationLimit },
        { "descent_end", &ReferenceGaitProfile::mSwingDescentEndFraction },
        { "emergency_margin", &ReferenceGaitProfile::mEmergencyCaptureMargin },
        { "emergency_stance_load", &ReferenceGaitProfile::mEmergencyStanceLoadFraction },
        { "turn_rate", &ReferenceGaitProfile::mSteeringTurnRate },
        { "pelvis_twist", &ReferenceGaitProfile::mSteeringPelvisTwist },
        { "speed_rate", &ReferenceGaitProfile::mSteeringSpeedChangeRate },
        { "stop_deceleration", &ReferenceGaitProfile::mSteeringStopDeceleration },
        { "stop_speed", &ReferenceGaitProfile::mSteeringStopSpeed },
        { "turn_slowdown", &ReferenceGaitProfile::mSteeringTurnSlowdown },
        { "stopping_gain", &ReferenceGaitProfile::mStoppingVelocityErrorGain },
        { "stop_timeout", &ReferenceGaitProfile::mSteeringStopTimeout },
        { "handover", &ReferenceGaitProfile::mStandingHandoverDuration },
        { "feasibility_margin_stance", &ReferenceGaitProfile::mFinalCommandFeasibilityStanceMargin },
        { "feasibility_margin_swing", &ReferenceGaitProfile::mFinalCommandFeasibilitySwingMargin },
        { "stance_knee_ease", &ReferenceGaitProfile::mStanceKneeEaseDuration },
        { "push_off", &ReferenceGaitProfile::mStanceAnklePushOff },
        { "push_off_start", &ReferenceGaitProfile::mStanceAnklePushOffStartFraction },
        { "terminal_push", &ReferenceGaitProfile::mTerminalStancePush },
        { "terminal_push_start", &ReferenceGaitProfile::mTerminalStancePushStartFraction },
        { "terminal_push_lat_limit", &ReferenceGaitProfile::mTerminalStancePushLateralLimit },
        { "lateral_request_fraction", &ReferenceGaitProfile::mMaximumLateralRequestFraction },
#ifdef RAGDOLLCORE_WALKSEARCH_EXTRA_FIELDS
        RAGDOLLCORE_WALKSEARCH_EXTRA_FIELDS
#endif
    };

    struct BoolProfileField
    {
        const char* mName;
        bool ReferenceGaitProfile::*mMember;
    };

    const BoolProfileField sBoolProfileFields[] = {
        { "initial_support_transfer", &ReferenceGaitProfile::mRequireInitialSupportTransfer },
        { "coordinate_initial_transfer", &ReferenceGaitProfile::mCoordinateInitialTransfer },
        { "coordinate_weight_transfers", &ReferenceGaitProfile::mCoordinateWeightTransfers },
        { "predict_placement", &ReferenceGaitProfile::mPredictPlacement },
        { "swing_velocity_feedforward", &ReferenceGaitProfile::mSwingVelocityFeedForward },
        { "stance_hip_rate_hold", &ReferenceGaitProfile::mStanceHipRateHold },
        { "limit_velocity_lead", &ReferenceGaitProfile::mLimitVelocityLead },
        { "push_off_trailing", &ReferenceGaitProfile::mStanceAnklePushOffTrailing },
    };

    struct BoolSetting
    {
        const char* mName;
        bool ScenarioSettings::*mMember;
    };

    const BoolSetting sBoolSettings[] = {
        { "player_input", &ScenarioSettings::mReferenceGaitPlayerInput },
        { "mesh_ground", &ScenarioSettings::mTriangleMeshGround },
        { "grounds_placement", &ScenarioSettings::mReferenceGaitGroundsPlacement },
        { "coordinates_leg_pose", &ScenarioSettings::mReferenceGaitCoordinatesLegPose },
        { "hold_swing_foot", &ScenarioSettings::mReferenceGaitHoldSwingFoot },
        // The hip strategy, so it can be measured against the same screens
        // with and without it rather than by rebuilding between runs.
        { "hip_strategy", &ScenarioSettings::mReferenceHipStrategy },
    };

    // Active-controller floats. The balance bandwidth lives here rather than
    // on the gait profile, and it decides whether the body can answer a
    // disturbance inside the step that caused it. Collected while parsing and
    // applied where the profile is built, since ScenarioSettings does not
    // carry the controller profile.
    struct ActiveFloatField
    {
        const char* mName;
        float RagdollCore::ActiveControllerProfile::*mMember;
    };

    const ActiveFloatField sActiveFloatFields[] = {
        { "com_frequency", &RagdollCore::ActiveControllerProfile::mCenterOfMassFrequency },
        { "com_damping", &RagdollCore::ActiveControllerProfile::mCenterOfMassDamping },
    };

    std::vector<std::pair<const ActiveFloatField*, float>> sActiveOverrides;

    struct Options
    {
        // --native-body <context.txt> <profile.yaml>: run the authored
        // scenario on the skeleton captured from the game (replay_context.py
        // creation block) with the game's YAML profile instead of the
        // authored scenario humanoid. 2026-09-11 (strafe-native-input): the
        // scenario body has legs 15% shorter and a pelvis 10 units lower
        // than the game's, so numbers scored on it did not transfer.
        std::string mNativeContext;
        std::string mNativeProfile;
        float mSeconds = 7.0f;
        float mSpeed = 60.0f;
        std::uint64_t mStartStep = 30;
        std::string mTraceDirectory;
        std::string mTraceName = "walksearch";
        bool mListFields = false;
        // Explicit opt-in RXN-01 controller/fixture. The ordinary walker path
        // and its defaults remain unchanged.
        bool mReactionEpisode = false;
        std::string mReactionScene;
        bool mReactionsDisabled = false;
        // Selected by the fixed RXN-01 expression-v2 physical screen.
        float mReactionExpressionScale = 1.2f;
        float mReactionDurationScale = 1.5f;
        // Opt-in until the bounded recovery screen passes. Zero is legacy.
        float mRecoveryPreviewDuration = 0.0f;
        std::filesystem::path mRecoveryArchive;
        float mHybridImpulse = 1000;
        std::string mHybridControl;
        std::uint64_t mReactionTrial = 0;
        std::vector<RagdollCore::ScenarioImpulse> mImpulses;
        // Additional request keyframes (--request step,vx,vy) for turns,
        // speed changes and stops after the start keyframe.
        std::vector<RagdollCore::ScenarioInputKeyframe> mRequests;
        std::vector<std::pair<std::uint64_t, JPH::Vec3>> mFacingRequests;
        // Facing applied to every keyframe; zero faces the travel direction.
        JPH::Vec3 mFacing = JPH::Vec3::sZero();
        // Multiplies the humanoid's total mass (--mass-scale) for robustness sweeps.
        float mMassScale = 1.0f;
        // Fable i56: authored pose cycle cut from a powered replay's animation
        // targets (--cycle replay start count [offset]).
        std::string mCyclePath;
        std::size_t mCycleStart = 0, mCycleCount = 0, mCycleOffset = 0;
        // Fixed simulation rate (--rate hz). Every step-valued argument
        // (--start, --impulse, --request, --facing-at) stays in 60 Hz step
        // units and is rescaled to the same simulated time; absent or 60
        // leaves the run byte-identical to the historical 60 Hz harness.
        float mRate = 60.0f;
        ScenarioSettings mSettings;
    };

    bool assignProfile(ScenarioSettings& settings, const std::string& key, const std::string& value)
    {
        char* end = nullptr;
        const float number = std::strtof(value.c_str(), &end);
        for (const auto& field : sProfileFields)
            if (key == field.mName)
            {
                if (end == value.c_str() || *end != '\0' || !std::isfinite(number))
                    return false;
                settings.mReferenceGait.*(field.mMember) = number;
                return true;
            }
        for (const auto& field : sBoolProfileFields)
            if (key == field.mName)
            {
                settings.mReferenceGait.*(field.mMember) = value == "1" || value == "true";
                return true;
            }
        for (const auto& field : sBoolSettings)
            if (key == field.mName)
            {
                settings.*(field.mMember) = value == "1" || value == "true";
                return true;
            }
        for (const auto& field : sActiveFloatFields)
            if (key == field.mName)
            {
                if (end == value.c_str() || *end != 0 || !std::isfinite(number))
                    return false;
                sActiveOverrides.push_back({ &field, number });
                return true;
            }
        return false;
    }

    std::optional<Options> parse(int argc, char** argv)
    {
        Options options;
        for (int i = 1; i < argc; ++i)
        {
            const std::string argument = argv[i];
            const auto next = [&]() -> std::optional<std::string> {
                if (i + 1 >= argc)
                    return std::nullopt;
                return std::string(argv[++i]);
            };
            if (argument == "--seconds")
            {
                const auto value = next();
                if (!value)
                    return std::nullopt;
                options.mSeconds = std::strtof(value->c_str(), nullptr);
            }
            else if (argument == "--speed")
            {
                const auto value = next();
                if (!value)
                    return std::nullopt;
                options.mSpeed = std::strtof(value->c_str(), nullptr);
            }
            else if (argument == "--start")
            {
                const auto value = next();
                if (!value)
                    return std::nullopt;
                options.mStartStep = std::strtoull(value->c_str(), nullptr, 10);
            }
            else if (argument == "--trace")
            {
                const auto value = next();
                if (!value)
                    return std::nullopt;
                options.mTraceDirectory = *value;
            }
            else if (argument == "--name")
            {
                const auto value = next();
                if (!value)
                    return std::nullopt;
                options.mTraceName = *value;
            }
            else if (argument == "--cycle")
            {
                const auto path = next();
                const auto start = next();
                const auto count = next();
                if (!path || !start || !count)
                    return std::nullopt;
                options.mCyclePath = *path;
                options.mCycleStart = static_cast<std::size_t>(std::strtoull(start->c_str(), nullptr, 10));
                options.mCycleCount = static_cast<std::size_t>(std::strtoull(count->c_str(), nullptr, 10));
                if (i + 1 < argc && std::string(argv[i + 1]).find("--") != 0
                    && std::string(argv[i + 1]).find('=') == std::string::npos)
                    options.mCycleOffset = static_cast<std::size_t>(std::strtoull(argv[++i], nullptr, 10));
            }
            else if (argument == "--impulse")
            {
                // step,body,x,y,z (world impulse at the body center of mass)
                const auto value = next();
                if (!value)
                    return std::nullopt;
                std::stringstream stream(*value);
                std::string part;
                std::vector<std::string> parts;
                while (std::getline(stream, part, ','))
                    parts.push_back(part);
                if (parts.size() != 5)
                    return std::nullopt;
                RagdollCore::ScenarioImpulse impulse;
                impulse.mStep = std::strtoull(parts[0].c_str(), nullptr, 10);
                impulse.mBodyIndex = std::strtoull(parts[1].c_str(), nullptr, 10);
                impulse.mImpulse = JPH::Vec3(std::strtof(parts[2].c_str(), nullptr),
                    std::strtof(parts[3].c_str(), nullptr), std::strtof(parts[4].c_str(), nullptr));
                options.mImpulses.push_back(impulse);
            }
            else if (argument == "--request" || argument == "--facing-at")
            {
                // step,vx,vy: a new horizontal velocity request from that step
                const auto value = next();
                if (!value)
                    return std::nullopt;
                std::stringstream stream(*value);
                std::string part;
                std::vector<std::string> parts;
                while (std::getline(stream, part, ','))
                    parts.push_back(part);
                if (parts.size() != 3)
                    return std::nullopt;
                RagdollCore::ScenarioInputKeyframe keyframe;
                keyframe.mStep = std::strtoull(parts[0].c_str(), nullptr, 10);
                keyframe.mRequestedHorizontalVelocity
                    = JPH::Vec3(std::strtof(parts[1].c_str(), nullptr), std::strtof(parts[2].c_str(), nullptr), 0.0f);
                if (argument == "--facing-at")
                    options.mFacingRequests.emplace_back(keyframe.mStep, keyframe.mRequestedHorizontalVelocity);
                else
                    options.mRequests.push_back(keyframe);
            }
            else if (argument == "--native-body")
            {
                const auto contextPath = next();
                const auto profilePath = next();
                if (!contextPath || !profilePath)
                    return std::nullopt;
                options.mNativeContext = *contextPath;
                options.mNativeProfile = *profilePath;
            }
            else if (argument == "--facing")
            {
                // x,y: where the body points, independent of where it travels
                // (plan item 2.3a). Zero, the default, faces the travel
                // direction and reproduces the pre-split behaviour exactly.
                const auto value = next();
                if (!value)
                    return std::nullopt;
                std::stringstream stream(*value);
                std::string part;
                std::vector<std::string> parts;
                while (std::getline(stream, part, ','))
                    parts.push_back(part);
                if (parts.size() != 2)
                    return std::nullopt;
                options.mFacing = JPH::Vec3(
                    std::strtof(parts[0].c_str(), nullptr), std::strtof(parts[1].c_str(), nullptr), 0.0f);
                if (!std::isfinite(options.mFacing.GetX()) || !std::isfinite(options.mFacing.GetY()))
                    return std::nullopt;
            }
            else if (argument == "--mass-scale")
            {
                const auto value = next();
                if (!value)
                    return std::nullopt;
                options.mMassScale = std::strtof(value->c_str(), nullptr);
                if (!std::isfinite(options.mMassScale) || options.mMassScale <= 0.0f)
                    return std::nullopt;
            }
            else if (argument == "--rxn01-episode")
                options.mReactionEpisode = true;
            else if (argument == "--rxn01-scene")
            {
                const auto value = next();
                const std::array<std::string, 7> scenes{
                    "face_guard", "arm_cradle", "limp", "doubled_over", "stumble_flail", "knockdown", "hybrid_catch" };
                if (!value || std::find(scenes.begin(), scenes.end(), *value) == scenes.end())
                    return std::nullopt;
                options.mReactionScene = *value;
                options.mReactionEpisode = true;
            }
            else if (argument == "--rxn01-reactions-off")
                options.mReactionsDisabled = true;
            else if (argument == "--rxn01-hybrid-impulse")
            {
                const auto value = next();
                if (!value) return std::nullopt;
                options.mHybridImpulse = std::strtof(value->c_str(), nullptr);
                if (!std::isfinite(options.mHybridImpulse) || options.mHybridImpulse < 0 || options.mHybridImpulse > 6000)
                    return std::nullopt;
            }
            else if (argument == "--rxn01-hybrid-control")
            {
                const auto value = next();
                if (!value || (*value != "no-hit" && *value != "motors-off")) return std::nullopt;
                options.mHybridControl = *value;
            }
            else if (argument == "--rxn01-recovery-archive")
            {
                const auto value = next();
                if (!value) return std::nullopt;
                options.mRecoveryArchive = std::filesystem::u8path(*value);
            }
            else if (argument == "--rxn01-getup")
            {
                const auto value = next();
                if (!value) return std::nullopt;
                options.mRecoveryPreviewDuration = std::strtof(value->c_str(), nullptr);
                if (!std::isfinite(options.mRecoveryPreviewDuration)
                    || options.mRecoveryPreviewDuration < 0.0f || options.mRecoveryPreviewDuration > 6.0f)
                    return std::nullopt;
            }
            else if (argument == "--rxn01-expression" || argument == "--rxn01-duration")
            {
                const auto value = next();
                if (!value)
                    return std::nullopt;
                const float scale = std::strtof(value->c_str(), nullptr);
                if (!std::isfinite(scale) || scale <= 0.0f
                    || scale > (argument == "--rxn01-expression" ? 1.2f : 2.0f))
                    return std::nullopt;
                (argument == "--rxn01-expression" ? options.mReactionExpressionScale
                                                  : options.mReactionDurationScale) = scale;
            }
            else if (argument == "--rxn01-trial")
            {
                const auto value = next();
                if (!value)
                    return std::nullopt;
                options.mReactionTrial = std::strtoull(value->c_str(), nullptr, 10);
            }
            else if (argument == "--rate")
            {
                const auto value = next();
                if (!value)
                    return std::nullopt;
                options.mRate = std::strtof(value->c_str(), nullptr);
                if (!std::isfinite(options.mRate) || options.mRate <= 0.0f)
                    return std::nullopt;
            }
            else if (argument == "--velocity-steps")
            {
                const auto value = next();
                if (!value)
                    return std::nullopt;
                options.mSettings.mVelocitySolverSteps = static_cast<std::uint32_t>(std::strtoul(value->c_str(), nullptr, 10));
            }
            else if (argument == "--list")
                options.mListFields = true;
            else if (argument.rfind("--", 0) == 0)
            {
                std::cerr << "unknown option " << argument << '\n';
                return std::nullopt;
            }
            else
            {
                const auto equals = argument.find('=');
                if (equals == std::string::npos
                    || !assignProfile(options.mSettings, argument.substr(0, equals), argument.substr(equals + 1)))
                {
                    std::cerr << "bad override " << argument << '\n';
                    return std::nullopt;
                }
            }
        }
        return options;
    }

    std::string quoted(const std::string& text)
    {
        std::string out = "\"";
        for (const char c : text)
        {
            if (c == '"' || c == '\\')
                out += '\\';
            out += c;
        }
        return out + "\"";
    }

    std::size_t reactionBodyIndex(
        const RagdollCore::RagdollProfile& profile, const std::string& name)
    {
        const auto found = std::find_if(profile.mBodies.begin(), profile.mBodies.end(),
            [&](const RagdollCore::BodyProfile& body) { return body.mName == name; });
        return found == profile.mBodies.end() ? std::numeric_limits<std::size_t>::max()
                                              : static_cast<std::size_t>(found - profile.mBodies.begin());
    }

    RagdollCore::RagdollPose reactionBindPose(const RagdollCore::SkeletonDescriptor& skeleton,
        const RagdollCore::RagdollProfile& profile)
    {
        RagdollCore::RagdollPose pose;
        pose.mBodyTransforms.reserve(profile.mBodies.size());
        for (const RagdollCore::BodyProfile& body : profile.mBodies)
        {
            const auto bone = std::find_if(skeleton.mBones.begin(), skeleton.mBones.end(),
                [&](const RagdollCore::SkeletonBone& candidate) { return candidate.mName == body.mBone; });
            if (bone == skeleton.mBones.end())
                return {};
            pose.mBodyTransforms.push_back(bone->mModelTransform);
        }
        return pose;
    }

    RagdollCore::ReactionBodyMode reactionMode(const RagdollCore::ScenarioFrame* previous)
    {
        if (!previous)
            return RagdollCore::ReactionBodyMode::Standing;
        switch (previous->mController.mMode)
        {
            case RagdollCore::ActiveControllerMode::GettingUp:
            case RagdollCore::ActiveControllerMode::BlendBack:
                return RagdollCore::ReactionBodyMode::Recovery;
            case RagdollCore::ActiveControllerMode::Falling:
            case RagdollCore::ActiveControllerMode::Fallen:
                return RagdollCore::ReactionBodyMode::Passive;
            case RagdollCore::ActiveControllerMode::Standing:
            case RagdollCore::ActiveControllerMode::Bracing:
            case RagdollCore::ActiveControllerMode::Stumbling:
                return previous->mRequestedHorizontalVelocity.IsNearZero()
                    ? RagdollCore::ReactionBodyMode::Standing : RagdollCore::ReactionBodyMode::Walking;
        }
        return RagdollCore::ReactionBodyMode::Idle;
    }

    RagdollCore::ReactionSupportState reactionSupport(const RagdollCore::ScenarioFrame* previous)
    {
        RagdollCore::ReactionSupportState support;
        if (!previous)
            return support;
        support.mLeftFootSupported = previous->mSupport.mLeftFootSupported;
        support.mRightFootSupported = previous->mSupport.mRightFootSupported;
        support.mBothFeetSupported = support.mLeftFootSupported && support.mRightFootSupported;
        support.mSupportEstablished = previous->mSupport.hasSupport();
        support.mCaptureMargin = previous->mSupport.mCenterOfMassDistance;
        return support;
    }
}

int runDriver(int argc, char** argv);
int runPlay01Worker(int argc, char** argv);

#include "replaydriver.hpp"

namespace
{
    // Creation skeleton + game profile from a replay context (the same
    // header the replay driver reads); the rest of the context is ignored.
    using WalkSearch::loadNativeBody;
}

int main(int argc, char** argv)
{
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory;
    JPH::RegisterTypes();
    const int code = runDriver(argc, argv);
    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
    return code;
}

int runDriver(int argc, char** argv)
{
    // PLAY-01 captured-body tracking worker (play01worker.cpp).
    if (argc >= 2 && std::string(argv[1]) == "--play01-worker")
        return runPlay01Worker(argc - 1, argv + 1);
    // Native scheduler twins: compare every captured physical state through
    // the shared replay driver. This does not claim offline reconstruction.
    if (argc == 4 && std::string(argv[1]) == "--compare-captures")
    {
        const auto load = [](const char* path) {
            std::ifstream file(path, std::ios::binary | std::ios::ate);
            if (!file || file.tellg() <= 0) throw std::runtime_error("Missing comparison capture");
            std::vector<std::byte> bytes(static_cast<std::size_t>(file.tellg()));
            file.seekg(0); file.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
            const auto replay = RagdollCore::deserializeReplay(bytes);
            if (!replay) throw std::runtime_error("Invalid comparison capture");
            return *replay;
        };
        const auto expected = load(argv[2]), actual = load(argv[3]);
        const auto result = RagdollCore::compareReplays(expected, actual);
        const bool compared = result.mStatus == RagdollCore::ReplayComparisonStatus::Match
            || result.mStatus == RagdollCore::ReplayComparisonStatus::Diverged;
        const bool physical = compared && result.mMaximumTranslationError == 0
            && result.mMaximumRotationAngleError == 0 && result.mMaximumLinearVelocityError == 0
            && result.mMaximumAngularVelocityError == 0;
        std::cout << std::setprecision(17) << "{\"comparison_scope\":\"native_capture_physical_state\",\"equivalence_proven\":false"
            << ",\"zero_physical_divergence\":" << (physical ? "true" : "false")
            << ",\"expected_frames\":" << expected.mFrames.size() << ",\"actual_frames\":" << actual.mFrames.size()
            << ",\"status\":" << static_cast<int>(result.mStatus)
            << ",\"translation_error\":" << result.mMaximumTranslationError
            << ",\"rotation_error\":" << result.mMaximumRotationAngleError
            << ",\"linear_velocity_error\":" << result.mMaximumLinearVelocityError
            << ",\"angular_velocity_error\":" << result.mMaximumAngularVelocityError
            << ",\"first_divergent_step\":" << (result.mFirstDivergentStep ? std::to_string(*result.mFirstDivergentStep) : "null")
            << "}\n";
        return physical ? 0 : 1;
    }
    // Fable: dump a powered replay's animation target poses (model space) so
    // an authored walk cycle can be inspected and cut into a pose cycle.
    if (argc >= 4 && std::string(argv[1]) == "--dump-animation")
    {
        std::ifstream binary(argv[2], std::ios::binary | std::ios::ate);
        if (!binary || binary.tellg() <= 0) { std::cerr << "Missing replay capture\n"; return 2; }
        std::vector<std::byte> bytes(static_cast<std::size_t>(binary.tellg()));
        binary.seekg(0);
        binary.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        const auto captured = RagdollCore::deserializeReplay(bytes);
        if (!captured) { std::cerr << "Invalid replay capture\n"; return 2; }
        std::ofstream out(argv[3], std::ios::binary | std::ios::trunc);
        out.precision(7);
        out << "frame,body,tx,ty,tz,qx,qy,qz,qw,req_x,req_y\n";
        for (std::size_t f = 0; f < captured->mFrames.size(); ++f)
        {
            const auto input = RagdollCore::decodePoweredReplayInput(*captured, captured->mFrames[f]);
            if (!input)
                continue;
            for (std::size_t b = 0; b < input->mAnimationTargetPose.size(); ++b)
            {
                const auto& t = input->mAnimationTargetPose[b];
                out << f << ',' << b << ',' << t.mTranslation.GetX() << ',' << t.mTranslation.GetY() << ','
                    << t.mTranslation.GetZ() << ',' << t.mRotation.GetX() << ',' << t.mRotation.GetY() << ','
                    << t.mRotation.GetZ() << ',' << t.mRotation.GetW() << ','
                    << input->mRequestedHorizontalVelocity.GetX() << ',' << input->mRequestedHorizontalVelocity.GetY() << '\n';
            }
        }
        std::cout << "frames=" << captured->mFrames.size() << " bodies=" << captured->mBodyCount << '\n';
        return 0;
    }
    if (argc >= 6 && std::string(argv[1]) == "--replay-capture")
    {
        try { return runCapturedScenario(argv[2], argv[3], argv[4], argv[5], argc - 6, argv + 6); }
        catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 2; }
    }
    const auto options = parse(argc, argv);
    if (!options)
    {
        std::cerr << "usage: ragdollcore-walksearch [--seconds S] [--speed V] [--start STEP] [--trace DIR] [--name N]"
                     " [--impulse step,body,x,y,z] [--request step,vx,vy] [--facing x,y]"
                     " [--facing-at step,x,y] [--mass-scale f] [--rate hz]"
                     " [--native-body context profile] [--rxn01-episode] [--rxn01-trial N]"
                     " [--rxn01-scene face_guard|arm_cradle|limp|doubled_over|stumble_flail|knockdown]"
                     " [--rxn01-reactions-off]"
                     " [--rxn01-expression scale] [--rxn01-duration scale]"
                     " [--list] [field=value ...]\n";
        return 2;
    }
    if (options->mListFields)
    {
        for (const auto& field : sProfileFields)
            std::cout << field.mName << '=' << ScenarioSettings{}.mReferenceGait.*(field.mMember) << '\n';
        for (const auto& field : sBoolProfileFields)
            std::cout << field.mName << '=' << (ScenarioSettings{}.mReferenceGait.*(field.mMember) ? 1 : 0)
                      << std::endl;
        for (const auto& field : sBoolSettings)
            std::cout << field.mName << '=' << (ScenarioSettings{}.*(field.mMember) ? 1 : 0) << '\n';
        for (const auto& field : sActiveFloatFields)
            std::cout << field.mName << '=' << RagdollCore::ActiveControllerProfile{}.*(field.mMember) << '\n';
        return 0;
    }
    ScenarioSettings settings = options->mSettings;
    // Same path as the scenario FPS fixtures (testscenario.cpp halfRate):
    // only ScenarioSettings::mFixedTimeStep changes. Steps given on the
    // command line are 60 Hz steps; rescale them to the chosen rate.
    const bool customRate = options->mRate != 60.0f;
    if (customRate)
        settings.mFixedTimeStep = 1.0f / options->mRate;
    const auto rescale = [&](std::uint64_t step) -> std::uint64_t {
        if (!customRate)
            return step;
        return static_cast<std::uint64_t>(std::llround(static_cast<double>(step) * options->mRate / 60.0));
    };
    const std::uint64_t startStep = rescale(options->mStartStep);
    settings.mReferenceGait.mContinuousSwingTrajectory = std::getenv("OPENMW_RAGDOLL_CONTINUOUS_SWING") != nullptr;
    settings.mLocomotionController = RagdollCore::ScenarioLocomotionController::Reference;
    settings.mTotalMass *= options->mMassScale;

    RagdollCore::ScenarioScript script;
    script.mDurationSeconds = options->mSeconds;
    if (!options->mCyclePath.empty())
    {
        std::ifstream binary(options->mCyclePath, std::ios::binary | std::ios::ate);
        if (!binary || binary.tellg() <= 0) { std::cerr << "Missing cycle replay\n"; return 2; }
        std::vector<std::byte> bytes(static_cast<std::size_t>(binary.tellg()));
        binary.seekg(0);
        binary.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        const auto captured = RagdollCore::deserializeReplay(bytes);
        if (!captured || options->mCycleCount < 2 || options->mCycleStart + options->mCycleCount > captured->mFrames.size()
            || options->mCycleOffset >= options->mCycleCount)
        { std::cerr << "Invalid cycle replay or window\n"; return 2; }
        for (std::size_t sample = 0; sample < options->mCycleCount; ++sample)
        {
            const std::size_t frameIndex = options->mCycleStart + (options->mCycleOffset + sample) % options->mCycleCount;
            auto input = RagdollCore::decodePoweredReplayInput(*captured, captured->mFrames[frameIndex]);
            if (!input) { std::cerr << "Undecodable cycle frame\n"; return 2; }
            RagdollCore::RagdollPose pose;
            pose.mBodyTransforms = std::move(input->mAnimationTargetPose);
            script.mGaitPoseCycle.push_back(std::move(pose));
        }
    }
    // Merge independent travel/facing timelines without a facing-only event
    // accidentally stopping movement, or a travel event clearing body aim.
    struct InputChange
    {
        std::optional<JPH::Vec3> mTravel;
        std::optional<JPH::Vec3> mFacing;
    };
    std::map<std::uint64_t, InputChange> changes;
    changes[startStep].mTravel = JPH::Vec3(0.0f, options->mSpeed, 0.0f);
    if (!options->mFacing.IsNearZero())
        changes[0].mFacing = options->mFacing;
    for (const auto& request : options->mRequests)
        changes[rescale(request.mStep)].mTravel = request.mRequestedHorizontalVelocity;
    for (const auto& [step, facing] : options->mFacingRequests)
        changes[rescale(step)].mFacing = facing;
    RagdollCore::ScenarioInputKeyframe command;
    for (const auto& [step, change] : changes)
    {
        command.mStep = step;
        if (change.mTravel)
            command.mRequestedHorizontalVelocity = *change.mTravel;
        if (change.mFacing)
            command.mFacing = *change.mFacing;
        script.mKeyframes.push_back(command);
    }
    script.mImpulses = options->mImpulses;
    for (auto& impulse : script.mImpulses)
        impulse.mStep = rescale(impulse.mStep);

    // The scenario profile is authored parity with humanoid-v1.yaml; the
    // controller switches under test are applied on top of it here so a term
    // can be screened with and without rebuilding.
    RagdollCore::RagdollProfile scenarioProfile = RagdollCore::makeScenarioHumanoidProfile();
    RagdollCore::SkeletonDescriptor scenarioSkeleton = RagdollCore::makeScenarioHumanoidSkeleton();
    const bool nativeBody = !options->mNativeContext.empty();
    if (nativeBody)
    {
        try
        {
            auto body = loadNativeBody(options->mNativeContext, options->mNativeProfile, settings);
            scenarioSkeleton = std::move(body.mSkeleton);
            scenarioProfile = std::move(body.mProfile);
        }
        catch (const std::exception& error)
        {
            std::cerr << error.what() << '\n';
            return 2;
        }
    }
    scenarioProfile.mActiveController.mReferenceHipStrategy = settings.mReferenceHipStrategy;
    for (const auto& [field, value] : sActiveOverrides)
        scenarioProfile.mActiveController.*(field->mMember) = value;

    struct ReactionEpisode
    {
        RagdollCore::ScenarioResult mScenario;
        std::vector<RagdollCore::ReactionOutputs> mReactions;
    };
    RagdollCore::ScenarioResult result;
    std::vector<RagdollCore::ReactionOutputs> reactionOutputs;
    std::optional<RagdollCore::ScenarioResult> passiveReactionControl;
    std::array<std::string, 2> recoveryGroups;
    constexpr std::array<const char*, 3> impactBodyNames{ "spine", "spine1", "spine2" };
    std::string impactBodyName = impactBodyNames[options->mReactionTrial % impactBodyNames.size()];
    float impactMagnitude = 5200.0f + 100.0f * static_cast<float>((options->mReactionTrial / 3) % 5);
    auto impactRegion = options->mReactionTrial % 3 == 0
        ? RagdollCore::AnatomicalRegion::Abdomen : RagdollCore::AnatomicalRegion::Chest;
    auto expectedBehavior = RagdollCore::BehaviorId::DoubledOver;
    std::optional<float> fixtureWound;
    const std::string& scene = options->mReactionScene;
    const bool hybridScene = scene == "hybrid_catch";
    const bool hybridHit = options->mHybridControl != "no-hit";
    if (scene == "face_guard")
    {
        impactBodyName = "head";
        impactRegion = RagdollCore::AnatomicalRegion::Head;
        impactMagnitude = 1400.0f;
        expectedBehavior = RagdollCore::BehaviorId::FaceGuard;
    }
    else if (scene == "arm_cradle")
    {
        impactBodyName = "left_forearm";
        impactRegion = RagdollCore::AnatomicalRegion::LeftForearm;
        impactMagnitude = 300.0f;
        fixtureWound = 0.55f;
        expectedBehavior = RagdollCore::BehaviorId::ArmCradle;
    }
    else if (scene == "limp")
    {
        impactBodyName = "left_calf";
        impactRegion = RagdollCore::AnatomicalRegion::LeftCalf;
        impactMagnitude = 300.0f;
        fixtureWound = 0.6f;
        expectedBehavior = RagdollCore::BehaviorId::Limp;
    }
    else if (scene == "doubled_over")
        impactMagnitude = 2200.0f;
    else if (scene == "stumble_flail")
    {
        impactBodyName = "right_upper_arm";
        impactRegion = RagdollCore::AnatomicalRegion::RightUpperArm;
        impactMagnitude = 2500.0f;
        expectedBehavior = RagdollCore::BehaviorId::StumbleFlail;
    }
    else if (hybridScene)
    {
        impactBodyName = "spine";
        impactRegion = RagdollCore::AnatomicalRegion::Chest;
        impactMagnitude = options->mHybridImpulse;
    }
    const std::size_t impactBody = reactionBodyIndex(scenarioProfile, impactBodyName.c_str());
    const std::uint64_t impactStep = (hybridScene ? 270 : startStep + 90) + options->mReactionTrial % 61;
    if (options->mReactionEpisode)
    {
        if (!nativeBody || options->mRate != 60.0f)
        {
            std::cerr << "--rxn01-episode requires --native-body and --rate 60\n";
            return 2;
        }
        const RagdollCore::RagdollPose standing = reactionBindPose(scenarioSkeleton, scenarioProfile);
        const auto lowPose = RecoveryPreview::crouch(standing, scenarioProfile);
        std::optional<std::array<WalkSearch::RecoveryClip, 2>> recoveryClips;
        std::optional<WalkSearch::RecoveryClip> walkingClip;
        if (hybridScene && options->mRecoveryArchive.empty())
        { std::cerr << "Hybrid catch needs --rxn01-recovery-archive for the real walk cycle\n"; return 2; }
        if (!options->mRecoveryArchive.empty())
        {
            if (options->mRecoveryPreviewDuration != 0.0f)
            { std::cerr << "Select either native clips or the rejected procedural preview\n"; return 2; }
            try
            {
                recoveryClips = WalkSearch::loadRecoveryClips(options->mRecoveryArchive, scenarioProfile);
                if (hybridScene) walkingClip = WalkSearch::loadWalkingClip(options->mRecoveryArchive, scenarioProfile);
            }
            catch (const std::exception& error)
            { std::cerr << "Recovery animation load failed: " << error.what() << '\n'; return 2; }
            for (std::size_t i = 0; i < recoveryGroups.size(); ++i)
                recoveryGroups[i] = (*recoveryClips)[i].mGroup;
        }
        const float impactAngle = 2.0f * std::numbers::pi_v<float>
            * static_cast<float>(options->mReactionTrial % 16) / 16.0f;
        const JPH::Vec3 impactImpulse(
            impactMagnitude * std::cos(impactAngle), impactMagnitude * std::sin(impactAngle), 0.0f);
        auto poseTargets = RagdollCore::makeReactionPoseTargets(scenarioProfile);
        poseTargets.mExpressionScale = options->mReactionExpressionScale;
        if (standing.mBodyTransforms.size() != scenarioProfile.mBodies.size()
            || impactBody >= scenarioProfile.mBodies.size())
        {
            std::cerr << "RXN-01 body is missing its bind pose or spine2 landmark\n";
            return 2;
        }
        RagdollCore::ScenarioScript episodeScript;
        episodeScript.mDurationSeconds = std::max(options->mSeconds, 12.0f);
        const auto runEpisode = [&](bool recoveryEnabled) {
            ReactionEpisode episode;
            RagdollCore::ReactionPipelineState reactionState;
            RagdollCore::ReactionPipelineProfile reactionProfile;
            reactionProfile.mEnabled = !options->mReactionsDisabled;
            reactionProfile.mBehavior.mEnabled = true;
            reactionProfile.mBehavior.mFaceGuardDuration *= options->mReactionDurationScale;
            reactionProfile.mBehavior.mDoubledOverDuration *= options->mReactionDurationScale;
            reactionProfile.mBehavior.mStumbleDuration *= options->mReactionDurationScale;
            std::optional<RagdollCore::RagdollPose> recoveryStart;
            bool wasGettingUp = false;
            bool hasRecovered = false;
            float resumeTime = 0.0f;
            float recoveryPoseTime = 0.0f;
            float crouchWaitTime = 0.0f;
            bool recoveryRising = false;
            std::size_t recoveryClipIndex = 0;
            WalkSearch::HybridCatch hybrid;
            float walkTime = 0;
            JPH::Vec3 animationOffset = JPH::Vec3::sZero();
            std::optional<RagdollCore::RagdollPose> catchStart, returnStart;
            JPH::Vec3 returnVelocity = JPH::Vec3::sZero();
            const RagdollCore::ScenarioInputProvider provider = [&](std::uint64_t step,
                                                                       const RagdollCore::ScenarioFrame* previous)
                -> std::optional<RagdollCore::ScenarioInputKeyframe> {
                RagdollCore::ReactionContractInput input;
                input.mFramePhysicsStep = step + 1;
                input.mFixedStepDuration = settings.mFixedTimeStep;
                input.mMode = reactionMode(previous);
                input.mWalking = input.mMode == RagdollCore::ReactionBodyMode::Walking;
                input.mMoving = input.mWalking;
                input.mSupport = reactionSupport(previous);
                input.mAnimationSampleValid = true;
                input.mOwned.mRecoveryBrace = input.mMode == RagdollCore::ReactionBodyMode::Recovery;
                if (step == impactStep + 1 && (!hybridScene || hybridHit))
                {
                    input.mContact.mValid = true;
                    input.mContact.mContactId = 1;
                    input.mContact.mAttackerActorIdentity = 2;
                    input.mContact.mTargetActorIdentity = 1;
                    input.mContact.mTargetRegistryGeneration = 1;
                    input.mContact.mSwingIdentity = 1;
                    input.mContact.mPhysicsStep = impactStep;
                    input.mContact.mTargetBodyIndex = static_cast<std::uint32_t>(impactBody);
                    input.mContact.mRegion = impactRegion;
                    input.mImpulse.mProvenance = RagdollCore::ImpulseProvenance::AppliedExternal;
                    input.mImpulse.mMagnitude = impactMagnitude;
                    input.mImpulse.mAlreadyApplied = true;
                    if (fixtureWound)
                    {
                        // Explicit fixture state supplied by the offline adapter.
                        // It is not damage inferred from the physical impulse.
                        input.mReceipt = RagdollCore::ReactionReceipt{ 1, 0.0f, 0.0f };
                        input.mWoundSeverity = fixtureWound;
                    }
                }
                const RagdollCore::ReactionOutputs reaction
                    = RagdollCore::stepReactionPipeline(reactionState, input, reactionProfile);
                episode.mReactions.push_back(reaction);

                RagdollCore::ScenarioInputKeyframe key;
                key.mStep = step;
                key.mRequestedHorizontalVelocity = step >= startStep
                    ? JPH::Vec3(0.0f, options->mSpeed * reaction.mSpeedScale, 0.0f)
                    : JPH::Vec3::sZero();
                key.mStabilizationStrength = reaction.mStabilizationStrength;
                RagdollCore::RagdollPose reacted = standing;
                RagdollCore::applyReactionPose(
                    standing.mBodyTransforms, reaction, poseTargets, reacted.mBodyTransforms);
                key.mMotorTarget = std::move(reacted);
                key.mRecoveryMotorTarget = standing;
                if (recoveryEnabled && recoveryClips && previous)
                {
                    const auto mode = previous->mController.mMode;
                    const bool gettingUp = mode == RagdollCore::ActiveControllerMode::GettingUp;
                    const bool recovering = gettingUp || mode == RagdollCore::ActiveControllerMode::BlendBack;
                    if (gettingUp && !wasGettingUp)
                    {
                        recoveryPoseTime = 0.0f;
                        recoveryClipIndex = previous->mController.mRecoveryDirection
                            == RagdollCore::RecoveryDirection::FaceDown ? 1 : 0;
                    }
                    wasGettingUp = gettingUp;
                    if (recovering)
                    {
                        const auto& clip = (*recoveryClips)[recoveryClipIndex];
                        recoveryPoseTime += settings.mFixedTimeStep;
                        const auto sample = std::min(clip.mFrames.size() - 1,
                            static_cast<std::size_t>(recoveryPoseTime * 60));
                        // After the authored rise, settle into the physical gait's
                        // standing reference before releasing the pelvis.
                        const float handover = RecoveryPreview::ease((recoveryPoseTime - clip.mDuration) / 0.75f);
                        key.mRecoveryMotorTarget = handover == 0.0f ? clip.mFrames[sample]
                            : RecoveryPreview::blend(clip.mFrames.back(), standing, poseTargets.mParents, handover);
                        key.mKinematicRecovery = true;
                        key.mKinematicRecoveryTerminalPose = standing;
                        key.mRecoveryAnimationFinished = recoveryPoseTime >= clip.mDuration + 0.75f;
                        hasRecovered = true;
                        resumeTime = 0.0f;
                        key.mRequestedHorizontalVelocity = JPH::Vec3::sZero();
                    }
                    else if (hasRecovered)
                    {
                        const bool upright = mode == RagdollCore::ActiveControllerMode::Standing
                            || mode == RagdollCore::ActiveControllerMode::Bracing;
                        resumeTime = upright ? resumeTime + settings.mFixedTimeStep : 0.0f;
                        key.mRequestedHorizontalVelocity *= RecoveryPreview::ease((resumeTime - 0.75f) / 0.75f);
                    }
                }
                else if (recoveryEnabled && options->mRecoveryPreviewDuration > 0.0f && previous)
                {
                    const auto mode = previous->mController.mMode;
                    const bool gettingUp = mode == RagdollCore::ActiveControllerMode::GettingUp;
                    const bool recovering = gettingUp || mode == RagdollCore::ActiveControllerMode::BlendBack;
                    if (gettingUp && !wasGettingUp)
                    {
                        recoveryPoseTime = 0.0f;
                        crouchWaitTime = 0.0f;
                        recoveryRising = false;
                        recoveryStart = standing;
                        for (std::size_t b = 0; b < previous->mBodies.size(); ++b)
                            recoveryStart->mBodyTransforms[b] = previous->mBodies[b].mModelTransform;
                    }
                    wasGettingUp = gettingUp;
                    if (gettingUp && recoveryStart)
                    {
                        const float gatherEnd = 0.55f * options->mRecoveryPreviewDuration;
                        const bool readyToRise = previous->mPelvisUpDot >= 0.80f
                            && previous->mSupport.hasSupport();
                        if (recoveryPoseTime >= gatherEnd && (readyToRise || crouchWaitTime >= 2.0f))
                            recoveryRising = true;
                        if (recoveryPoseTime < gatherEnd || recoveryRising)
                            recoveryPoseTime += settings.mFixedTimeStep;
                        else
                            crouchWaitTime += settings.mFixedTimeStep;
                        const float phase = recoveryPoseTime / options->mRecoveryPreviewDuration;
                        key.mRecoveryMotorTarget = phase < 0.55f
                            ? RecoveryPreview::blend(*recoveryStart, lowPose, poseTargets.mParents,
                                  RecoveryPreview::ease(phase / 0.55f))
                            : RecoveryPreview::blend(lowPose, standing, poseTargets.mParents,
                                  RecoveryPreview::ease((phase - 0.55f) / 0.45f));
                        key.mRecoveryPelvisHeightScale = phase < 0.55f
                            ? 0.15f + 0.30f * RecoveryPreview::ease(phase / 0.55f)
                            : 0.45f + 0.55f * RecoveryPreview::ease((phase - 0.55f) / 0.45f);
                        key.mRecoveryAnimationFinished = phase >= 1.0f;
                    }
                    if (recovering)
                    {
                        hasRecovered = true;
                        resumeTime = 0.0f;
                        key.mRequestedHorizontalVelocity = JPH::Vec3::sZero();
                    }
                    else if (hasRecovered)
                    {
                        const bool upright = mode == RagdollCore::ActiveControllerMode::Standing
                            || mode == RagdollCore::ActiveControllerMode::Bracing;
                        resumeTime = upright ? resumeTime + settings.mFixedTimeStep : 0.0f;
                        key.mRequestedHorizontalVelocity *= RecoveryPreview::ease((resumeTime - 0.75f) / 0.75f);
                    }
                }
                if (hybridScene)
                {
                    const bool catchEnabled = recoveryEnabled && options->mHybridControl != "motors-off";
                    const auto priorStage = hybrid.mStage;
                    hybrid.step(previous, step == impactStep && hybridHit, catchEnabled, settings.mFixedTimeStep);
                    key.mHybridStage = hybrid.mStage;
                    const auto measured = [&] {
                        auto pose = standing;
                        if (previous)
                            for (std::size_t b = 0; b < previous->mBodies.size(); ++b)
                                pose.mBodyTransforms[b] = previous->mBodies[b].mModelTransform;
                        return pose;
                    };
                    if (hybrid.mStage == 2 && priorStage != 2) catchStart = measured();
                    if (hybrid.mStage == 3 && priorStage != 3)
                    {
                        returnStart = measured();
                        walkTime = WalkSearch::matchWalkPhase(*walkingClip, *returnStart, poseTargets);
                        const auto target = WalkSearch::sampleWalk(*walkingClip, walkTime, poseTargets);
                        animationOffset = returnStart->mBodyTransforms[0].mTranslation - target.mBodyTransforms[0].mTranslation;
                        returnVelocity = settings.mRootRotation.Conjugated() * previous->mBodies[0].mLinearVelocity
                            / settings.mUniformScale;
                    }
                    if (hybrid.mStage == 1 || hybrid.mStage == 3)
                    {
                        key.mKinematicLocomotion = true;
                        key.mKinematicRecovery = false;
                        key.mRequestedHorizontalVelocity = step >= 90 ? JPH::Vec3(0, options->mSpeed, 0) : JPH::Vec3::sZero();
                        auto animated = standing;
                        if (step >= 90)
                        {
                            walkTime += settings.mFixedTimeStep * options->mSpeed / walkingClip->mSpeed;
                            animated = WalkSearch::sampleWalk(*walkingClip, walkTime, poseTargets);
                            if (!returnStart)
                                animated = RecoveryPreview::blend(standing, animated, poseTargets.mParents,
                                    RecoveryPreview::ease(static_cast<float>(step - 90) / 45));
                            animationOffset += JPH::Vec3(0, options->mSpeed * settings.mFixedTimeStep, 0);
                        }
                        for (auto& body : animated.mBodyTransforms) body.mTranslation += animationOffset;
                        if (returnStart)
                        {
                            auto movingStart = *returnStart;
                            for (auto& body : movingStart.mBodyTransforms) body.mTranslation += returnVelocity * hybrid.mTime;
                            animated = RecoveryPreview::blend(movingStart, animated, poseTargets.mParents,
                                RecoveryPreview::ease(hybrid.mTime / 0.75f));
                        }
                        key.mMotorTarget = std::move(animated);
                    }
                    else if (catchStart && hybrid.mTime < 0.35f && !key.mKinematicRecovery)
                        key.mMotorTarget = RecoveryPreview::blend(*catchStart, *key.mMotorTarget, poseTargets.mParents,
                            RecoveryPreview::ease(hybrid.mTime / 0.35f));
                    // Let the catch take real PGC steps, then use its normal
                    // stop/standing handover to earn a stable return pose.
                    // A walking COM need not remain inside a static foot hull.
                    if (hybrid.mStage == 2 && hybrid.mTime >= 1.0f)
                        key.mRequestedHorizontalVelocity = JPH::Vec3::sZero();
                    if (!catchEnabled && step >= impactStep && hybridHit)
                    {
                        key.mKinematicLocomotion = key.mKinematicRecovery = false;
                        key.mMotorsEnabled = key.mTranslationActuationEnabled = false;
                        key.mSwingFootActuationEnabled = key.mUprightTorqueEnabled = false;
                    }
                }
                if (step == impactStep && (!hybridScene || hybridHit))
                {
                    std::optional<JPH::RVec3> point;
                    if (previous && impactBody < previous->mBodies.size())
                    {
                        const float verticalOffset = 2.0f
                            * static_cast<float>(static_cast<int>(options->mReactionTrial % 3) - 1);
                        point = previous->mBodies[impactBody].mWorldPosition
                            + JPH::Vec3(0.0f, 0.0f, verticalOffset);
                    }
                    key.mImpulses.push_back(
                        { step, impactBody, impactImpulse, point });
                }
                if (previous && previous->mController.mMode == RagdollCore::ActiveControllerMode::Fallen)
                    key.mRecoveryRequested = recoveryEnabled;
                if (previous
                    && options->mRecoveryPreviewDuration == 0.0f
                    && !recoveryClips
                    && previous->mController.mMode == RagdollCore::ActiveControllerMode::GettingUp
                    && previous->mController.mTimeInMode >= 1.0f)
                    key.mRecoveryAnimationFinished = recoveryEnabled;
                if (!recoveryEnabled && step >= impactStep)
                {
                    key.mMotorsEnabled = false;
                    key.mTranslationActuationEnabled = false;
                    key.mSwingFootActuationEnabled = false;
                    key.mUprightTorqueEnabled = false;
                }
                return key;
            };
            episode.mReactions.reserve(static_cast<std::size_t>(
                episodeScript.mDurationSeconds / settings.mFixedTimeStep));
            episode.mScenario = RagdollCore::runRagdollScenario(
                scenarioSkeleton, scenarioProfile, settings, episodeScript, provider);
            return episode;
        };
        ReactionEpisode active = runEpisode(true);
        ReactionEpisode passive = runEpisode(false);
        result = std::move(active.mScenario);
        for (const auto& error : result.mErrors) std::cerr << error << '\n';
        reactionOutputs = std::move(active.mReactions);
        passiveReactionControl = std::move(passive.mScenario);
    }
    else
        result = RagdollCore::runRagdollScenario(scenarioSkeleton, scenarioProfile, settings, script);
    const auto budgetRatio = [&](const RagdollCore::ScenarioFrame& frame) {
        float ratio = 0.0f;
        for (std::size_t b = 0; b < frame.mJointMotorTargets.size() && b < scenarioProfile.mBodies.size(); ++b)
        {
            const auto& target = frame.mJointMotorTargets[b];
            const auto& constraint = scenarioProfile.mBodies[b].mConstraint;
            if (!target || !target->mDrive || !constraint)
                continue;
            const float authored = constraint->mMotor.mMaximumTorque;
            const auto torque = target->mDrive->mMaximumTorque;
            const float maximum = std::max({ torque.GetX(), torque.GetY(), torque.GetZ() });
            if (authored > 0.0f)
                ratio = std::max(ratio, maximum / authored);
            else if (maximum > 0.0f)
                return std::numeric_limits<float>::infinity();
        }
        return ratio;
    };
    if (!options->mTraceDirectory.empty())
    {
        std::filesystem::path directory(options->mTraceDirectory);
        std::error_code error;
        std::filesystem::create_directories(directory, error);
        std::ofstream trace(directory / (options->mTraceName + ".csv"), std::ios::binary | std::ios::trunc);
        trace << RagdollCore::formatScenarioTraceCsv(result);
        if (options->mReactionEpisode)
        {
            std::ofstream reactions(directory / (options->mTraceName + "-reactions.csv"));
            reactions << "step,behavior,reaction_phase,reaction_intensity,pose_authority,stabilization_strength,"
                         "applied_stabilization_strength,speed_scale,limp_bias,cradle_left,cradle_right,limp_left,limp_right,"
                         "cover_face,served_contact,served_receipt,impact,impact_body,impact_magnitude,"
                         "motor_budget_ratio,controller_state,actual_speed,requested_speed\n";
            constexpr std::array modes{ "standing", "bracing", "stumbling", "falling", "fallen", "getting_up", "blend_back" };
            constexpr std::array phases{ "idle", "impact", "stagger", "catch", "fall", "get_up", "resuming" };
            for (std::size_t i = 0; i < result.mFrames.size() && i < reactionOutputs.size(); ++i)
            {
                const auto& frame = result.mFrames[i];
                const auto& reaction = reactionOutputs[i];
                reactions << frame.mStep << ',' << RagdollCore::behaviorName(reaction.mBehavior)
                    << ',' << phases[static_cast<std::size_t>(reaction.mPhase)]
                    << ',' << reaction.mReactionIntensity << ',' << reaction.mPoseCorrectionAuthority
                    << ',' << reaction.mStabilizationStrength << ',' << frame.mStabilizationStrength
                    << ',' << reaction.mSpeedScale << ',' << reaction.mLimpBias
                    << ',' << reaction.mCradleLeft << ',' << reaction.mCradleRight
                    << ',' << reaction.mLimpLeft << ',' << reaction.mLimpRight << ',' << reaction.mCoverFace
                    << ',' << reaction.mServedContactId << ',' << reaction.mServedReceiptId
                    << ',' << frame.mImpulseCount << ',' << impactBody << ',' << (frame.mStep == impactStep ? impactMagnitude : 0)
                    << ',' << budgetRatio(frame) << ',' << modes[static_cast<std::size_t>(frame.mController.mMode)]
                    << ',' << std::hypot(frame.mCenterOfMassVelocity.GetX(), frame.mCenterOfMassVelocity.GetY())
                    << ',' << frame.mRequestedHorizontalVelocity.Length() << '\n';
            }
        }
        std::ofstream stages(directory / (options->mTraceName + "-stages.csv"), std::ios::binary | std::ios::trunc);
        stages << RagdollCore::formatScenarioGaitStagesCsv(result);
        // Body positions per step for tools/offline-viewer (same columns as
        // the ragdollcore_tests export, positions and rotations only).
        std::ofstream bodies(directory / (options->mTraceName + "-bodies.csv"), std::ios::binary | std::ios::trunc);
        // Ground contact points per step (same columns as the replay
        // driver's contacts.csv) so floor geometry can be compared.
        std::ofstream contacts(directory / (options->mTraceName + "-contacts.csv"), std::ios::binary | std::ios::trunc);
        contacts << "step,point_order,body,x,y,z,nx,ny,nz,depth,normal_impulse,friction_impulse,friction,supported\n"
                 << std::setprecision(17);
        for (std::size_t step = 0; step < result.mFrames.size(); ++step)
            for (std::size_t pointIndex = 0; pointIndex < result.mFrames[step].mContacts.size(); ++pointIndex)
            {
                const auto& c = result.mFrames[step].mContacts[pointIndex];
                contacts << step << ',' << pointIndex << ',' << c.mBodyIndex
                    << ',' << c.mPosition.GetX() << ',' << c.mPosition.GetY() << ',' << c.mPosition.GetZ()
                    << ',' << c.mNormal.GetX() << ',' << c.mNormal.GetY() << ',' << c.mNormal.GetZ()
                    << ',' << c.mPenetrationDepth << ',';
                if (c.mNormalImpulse) contacts << *c.mNormalImpulse;
                contacts << ',';
                if (c.mFrictionImpulseMagnitude) contacts << *c.mFrictionImpulseMagnitude;
                contacts << ',';
                if (c.mFriction) contacts << *c.mFriction;
                contacts << ',' << c.hasSupport() << '\n';
            }
        bodies.precision(9);
        bodies << "step,body,x,y,z,vx,vy,vz,wx,wy,wz,qx,qy,qz,qw\n";
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
                       << body.mWorldRotation.GetZ() << ',' << body.mWorldRotation.GetW() << '\n';
            }
    }

    if (options->mReactionEpisode)
    {
        const std::size_t recordedImpactBody = impactBody;
        const float recordedImpactMagnitude = impactMagnitude;
        bool walkedBeforeImpact = false;
        bool physicalImpact = false;
        bool reducedBudget = false;
        bool scaledMotorBudget = false;
        bool catchOrFall = false;
        bool fallen = false;
        bool gettingUp = false;
        bool blendBack = false;
        bool resumedStanding = false;
        bool resumedWalking = false;
        std::size_t stablePostImpactWalkingFrames = 0;
        std::uint64_t standingAgainStep = 0;
        const std::size_t spine = reactionBodyIndex(scenarioProfile, "spine");
        const bool aligned = reactionOutputs.size() == result.mFrames.size();
        for (std::size_t i = 0; aligned && i < result.mFrames.size(); ++i)
        {
            const RagdollCore::ScenarioFrame& frame = result.mFrames[i];
            const RagdollCore::ReactionOutputs& reaction = reactionOutputs[i];
            if (frame.mStep < impactStep && frame.mReferenceGait && frame.mGait.hasStance()
                && frame.mCenterOfMass.GetY() > 4.0)
                walkedBeforeImpact = true;
            if (frame.mStep == impactStep)
                physicalImpact = frame.mImpulseCount == 1;
            if (reaction.mStabilizationStrength < 0.999f
                && frame.mStabilizationStrength == reaction.mStabilizationStrength)
            {
                reducedBudget = true;
                if (spine < frame.mJointMotorTargets.size() && spine < scenarioProfile.mBodies.size()
                    && scenarioProfile.mBodies[spine].mConstraint && frame.mJointMotorTargets[spine]
                    && frame.mJointMotorTargets[spine]->mDrive)
                {
                    const float expected = scenarioProfile.mBodies[spine].mConstraint->mMotor.mMaximumTorque
                        * reaction.mStabilizationStrength;
                    scaledMotorBudget = std::abs(
                        frame.mJointMotorTargets[spine]->mDrive->mMaximumTorque.GetX() - expected) < 1e-3f;
                }
            }
            catchOrFall = catchOrFall || reaction.mPhase == RagdollCore::ReactionPhase::Catch
                || reaction.mPhase == RagdollCore::ReactionPhase::Fall
                || frame.mController.mMode == RagdollCore::ActiveControllerMode::Falling;
            fallen = fallen || frame.mController.mMode == RagdollCore::ActiveControllerMode::Fallen;
            gettingUp = gettingUp || frame.mController.mMode == RagdollCore::ActiveControllerMode::GettingUp;
            blendBack = blendBack || frame.mController.mMode == RagdollCore::ActiveControllerMode::BlendBack;
            if (blendBack && frame.mController.mMode == RagdollCore::ActiveControllerMode::Standing)
            {
                resumedStanding = true;
                if (standingAgainStep == 0)
                    standingAgainStep = frame.mStep;
            }
            if (resumedStanding && frame.mStep > standingAgainStep + 30 && frame.mGait.hasStance()
                && !frame.mRequestedHorizontalVelocity.IsNearZero())
                resumedWalking = true;
            if (frame.mStep > impactStep + 120 && frame.mGait.hasStance()
                && !frame.mRequestedHorizontalVelocity.IsNearZero()
                && frame.mController.mMode != RagdollCore::ActiveControllerMode::Falling
                && frame.mController.mMode != RagdollCore::ActiveControllerMode::Fallen)
                ++stablePostImpactWalkingFrames;
        }
        bool passiveFell = false;
        bool passiveRecovered = false;
        if (passiveReactionControl)
        {
            for (const RagdollCore::ScenarioFrame& frame : passiveReactionControl->mFrames)
            {
                passiveFell = passiveFell || frame.mController.mMode == RagdollCore::ActiveControllerMode::Fallen;
                if (passiveFell && (frame.mController.mMode == RagdollCore::ActiveControllerMode::GettingUp
                        || frame.mController.mMode == RagdollCore::ActiveControllerMode::BlendBack))
                    passiveRecovered = true;
            }
        }
        const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
        const bool caughtWithoutFall = !fallen && stablePostImpactWalkingFrames >= 60;
        catchOrFall = catchOrFall || caughtWithoutFall;
        const bool completedOutcome = caughtWithoutFall
            || (fallen && gettingUp && blendBack && resumedStanding && resumedWalking);
        const bool internalWrench = invariants.mMaxAppliedNetForce < 1e-3f
            && invariants.mMaxNetActuationMoment < 1e-3f;
        if (!scene.empty())
        {
            std::size_t behaviorFrames = 0, impulseCount = 0, contactCount = 0;
            float maximumBudgetRatio = 0.0f;
            bool strengthConsumed = aligned;
            for (std::size_t i = 0; aligned && i < result.mFrames.size(); ++i)
            {
                const auto& frame = result.mFrames[i];
                const auto& reaction = reactionOutputs[i];
                behaviorFrames += reaction.mBehavior == (options->mReactionsDisabled
                    ? RagdollCore::BehaviorId::None : expectedBehavior);
                impulseCount += frame.mImpulseCount;
                contactCount += reaction.mServedContactId != 0;
                maximumBudgetRatio = std::max(maximumBudgetRatio, budgetRatio(frame));
                strengthConsumed &= std::abs(frame.mStabilizationStrength - reaction.mStabilizationStrength) < 1e-5f;
            }
            const bool behaviorObserved = hybridScene || behaviorFrames >= (fixtureWound ? 120u : 1u);
            const unsigned expectedImpulses = hybridScene && !hybridHit ? 0 : 1;
            const bool checks = result.completed() && aligned && (physicalImpact == (expectedImpulses != 0)) && behaviorObserved
                && impulseCount == expectedImpulses && contactCount == (options->mReactionsDisabled ? 0u : expectedImpulses)
                && strengthConsumed && maximumBudgetRatio <= 1.00001f && internalWrench;
            std::cout << "{\"mode\":\"rxn01_review\",\"scene\":\"" << scene
                << "\",\"checks_passed\":" << (checks ? "true" : "false")
                << ",\"reactions_enabled\":" << (options->mReactionsDisabled ? "false" : "true")
                << ",\"expression_scale\":" << options->mReactionExpressionScale
                << ",\"duration_scale\":" << options->mReactionDurationScale
                << ",\"recovery_preview_duration\":" << options->mRecoveryPreviewDuration
                << ",\"kinematic_recovery\":" << (!options->mRecoveryArchive.empty() ? "true" : "false")
                << ",\"face_up_clip\":\"" << recoveryGroups[0] << "\",\"face_down_clip\":\"" << recoveryGroups[1] << "\""
                << ",\"frames\":" << result.mFrames.size() << ",\"behavior_frames\":" << behaviorFrames
                << ",\"impulse_count\":" << impulseCount << ",\"served_contacts\":" << contactCount
                << ",\"max_motor_budget_ratio\":" << maximumBudgetRatio
                << ",\"strength_consumed\":" << (strengthConsumed ? "true" : "false")
                << ",\"fallen\":" << (fallen ? "true" : "false")
                << ",\"resumed_walking\":" << (resumedWalking ? "true" : "false")
                << ",\"caught_without_fall\":" << (caughtWithoutFall ? "true" : "false")
                << ",\"passive_fell\":" << (passiveFell ? "true" : "false")
                << ",\"passive_recovered\":" << (passiveRecovered ? "true" : "false")
                << ",\"max_net_force\":" << invariants.mMaxAppliedNetForce
                << ",\"max_net_moment\":" << invariants.mMaxNetActuationMoment << "}" << std::endl;
            return checks ? 0 : 1;
        }
        const bool pass = result.completed() && passiveReactionControl
            && passiveReactionControl->completed() && aligned && walkedBeforeImpact && physicalImpact
            && reducedBudget && scaledMotorBudget && catchOrFall && completedOutcome
            && passiveFell && !passiveRecovered && internalWrench;
        std::cout << "{\"pass\":" << (pass ? "true" : "false")
                  << ",\"mode\":\"" << (options->mRecoveryArchive.empty() ? "rxn01_episode" : "rxn01_hybrid_episode")
                  << "\",\"captured_body\":true"
                  << ",\"kinematic_recovery\":" << (!options->mRecoveryArchive.empty() ? "true" : "false")
                  << ",\"face_up_clip\":\"" << recoveryGroups[0] << "\",\"face_down_clip\":\"" << recoveryGroups[1] << "\""
                  << ",\"trial\":" << options->mReactionTrial
                  << ",\"impact_body\":" << recordedImpactBody
                  << ",\"impact_step\":" << impactStep
                  << ",\"impact_magnitude\":" << recordedImpactMagnitude
                  << ",\"triangle_mesh_ground\":" << (result.mSettings.mTriangleMeshGround ? "true" : "false")
                  << ",\"physics_hz\":60,\"walked_before_impact\":" << (walkedBeforeImpact ? "true" : "false")
                  << ",\"physical_impact\":" << (physicalImpact ? "true" : "false")
                  << ",\"reduced_budget\":" << (reducedBudget ? "true" : "false")
                  << ",\"scaled_motor_budget\":" << (scaledMotorBudget ? "true" : "false")
                  << ",\"catch_or_fall\":" << (catchOrFall ? "true" : "false")
                  << ",\"caught_without_fall\":" << (caughtWithoutFall ? "true" : "false")
                  << ",\"fallen\":" << (fallen ? "true" : "false")
                  << ",\"getting_up\":" << (gettingUp ? "true" : "false")
                  << ",\"blend_back\":" << (blendBack ? "true" : "false")
                  << ",\"resumed_standing\":" << (resumedStanding ? "true" : "false")
                  << ",\"resumed_walking\":" << (resumedWalking ? "true" : "false")
                  << ",\"passive_fell\":" << (passiveFell ? "true" : "false")
                  << ",\"passive_recovered\":" << (passiveRecovered ? "true" : "false")
                  << ",\"max_net_force\":" << invariants.mMaxAppliedNetForce
                  << ",\"max_net_moment\":" << invariants.mMaxNetActuationMoment
                  << ",\"frames\":" << result.mFrames.size() << "}" << std::endl;
        return pass ? 0 : 1;
    }

    std::vector<std::string> violations;
    const auto check = [&](bool condition, const std::string& text) {
        if (!condition)
            violations.push_back(text);
    };
    std::size_t referenceFrames = 0, saturatedFrames = 0;
    std::size_t feasibilityClamped = 0, feasibilityFrames = 0;
    float maxStanceKneeJump = 0.0f;
    std::size_t stanceKneeJumpFrames = 0;
    std::optional<std::uint64_t> firstFall;
    std::vector<std::uint64_t> transfers;
    bool observedStance = false, observedSwingTarget = false;
    float maxUtilization = 0.0f;
    std::map<std::size_t, std::size_t> saturationByBody;
    // Lateral sway follows the current request, including scripted turns.
    double lateralSquared = 0.0;
    std::size_t lateralFrames = 0;
    double qualityLateralSquared = 0.0, headingSquared = 0.0, maximumHeading = 0.0;
    double landingWidth = 0.0, landingLength = 0.0;
    std::size_t qualityFrames = 0, measuredLandings = 0;
    auto previousStage = RagdollCore::ReferenceGaitStage::Idle;
    std::size_t commandIndex = 0;
    JPH::Vec3 commandedFacing = JPH::Vec3::sZero();
    std::uint32_t emergencySteps = 0;
    for (const auto& frame : result.mFrames)
    {
        while (commandIndex < script.mKeyframes.size() && script.mKeyframes[commandIndex].mStep <= frame.mStep)
            commandedFacing = script.mKeyframes[commandIndex++].mFacing;
        if ((frame.mController.mMode == RagdollCore::ActiveControllerMode::Falling
                || frame.mController.mMode == RagdollCore::ActiveControllerMode::Fallen)
            && !firstFall)
            firstFall = frame.mStep;
        if (frame.mGait.mTransitioned && frame.mGait.hasStance() && frame.mReferenceGait)
            transfers.push_back(frame.mStep);
        observedStance = observedStance || frame.mGait.hasStance();
        observedSwingTarget = observedSwingTarget || frame.mSwingFootMotorTarget.has_value();
        if (!frame.mReferenceGait)
        {
            previousStage = RagdollCore::ReferenceGaitStage::Idle;
            continue;
        }
        ++referenceFrames;
        feasibilityClamped += frame.mReferenceGait->mFeasibilityClampedCount;
        if (frame.mReferenceGait->mFeasibilityClampedCount > 0)
            ++feasibilityFrames;
        if (frame.mReferenceGait->mStanceKneeTargetJump >= 0.0f)
        {
            maxStanceKneeJump = std::max(maxStanceKneeJump, frame.mReferenceGait->mStanceKneeTargetJump);
            if (frame.mReferenceGait->mStanceKneeTargetJump > 0.05f)
                ++stanceKneeJumpFrames;
        }
        JPH::Vec3 forward = frame.mRequestedHorizontalVelocity;
        forward.SetZ(0.0f);
        if (frame.mGait.hasStance() && !forward.IsNearZero())
        {
            forward = forward.Normalized();
            const JPH::Vec3 right(forward.GetY(), -forward.GetX(), 0.0f);
            const double lateral = frame.mCenterOfMassVelocity.Dot(right);
            lateralSquared += lateral * lateral;
            ++lateralFrames;
            // Match the live quality window: exclude the first two seconds
            // after movement is requested. These are diagnostics, not gates.
            if (frame.mTime >= startStep * settings.mFixedTimeStep + 2.0)
            {
                JPH::Vec3 facing = commandedFacing.IsNearZero() ? forward : commandedFacing.Normalized();
                const JPH::Vec3 facingRight(facing.GetY(), -facing.GetX(), 0.0f);
                const double qualityLateral = frame.mCenterOfMassVelocity.Dot(facingRight);
                qualityLateralSquared += qualityLateral * qualityLateral;
                // The synthetic fixture's pelvis has identity bind rotation
                // and local +Y forward; this is body yaw, not velocity heading.
                JPH::Vec3 pelvisForward = frame.mBodies[result.mLandmarks.mPelvis].mWorldRotation * JPH::Vec3::sAxisY();
                pelvisForward.SetZ(0.0f);
                const double heading = std::atan2(facing.Cross(pelvisForward).GetZ(), facing.Dot(pelvisForward));
                headingSquared += heading * heading;
                maximumHeading = std::max(maximumHeading, std::abs(heading));
                ++qualityFrames;
                if (previousStage == RagdollCore::ReferenceGaitStage::Swing
                    && frame.mReferenceGait->mStage == RagdollCore::ReferenceGaitStage::DoubleSupport
                    && frame.mSupport.mLeftFootSupported && frame.mSupport.mRightFootSupported)
                {
                    const JPH::Vec3 separation(frame.mBodies[result.mLandmarks.mRightFoot].mWorldPosition
                        - frame.mBodies[result.mLandmarks.mLeftFoot].mWorldPosition);
                    landingWidth += std::abs(separation.Dot(facingRight));
                    landingLength += std::abs(separation.Dot(facing));
                    ++measuredLandings;
                }
            }
        }
        previousStage = frame.mReferenceGait->mStage;
        maxUtilization = std::max(maxUtilization, frame.mReferenceGait->mMaxMotorUtilization);
        emergencySteps = std::max(emergencySteps, frame.mReferenceGait->mEmergencyStepCount);
        if (frame.mReferenceGait->mMaxMotorUtilization >= 0.999f)
        {
            ++saturatedFrames;
            ++saturationByBody[frame.mReferenceGait->mMaxMotorUtilizationBody];
        }
    }
    for (const auto& error : result.mErrors)
        violations.push_back("error: " + error);
    const auto invariants = RagdollCore::summarizeScenarioInvariants(result);
    check(observedStance, "no stance");
    check(observedSwingTarget, "no swing target");
    check(invariants.mFallingFrames + invariants.mFallenFrames == 0, "fell");
    check(invariants.mMinPelvisUpDot > 0.5f, "pelvis tilt");
    // Fable i51: a run/hop experiment leaves the ground on purpose.
    const bool runExperiment = std::getenv("OPENMW_RAGDOLL_RUN_LIFT") != nullptr;
    check(runExperiment || invariants.mAirborneFrames == 0, "airborne");
    check(invariants.mSteps.mLeftStanceCount >= 2, "left stances");
    check(invariants.mSteps.mRightStanceCount >= 2, "right stances");
    check(invariants.mSteps.mDurationSymmetry >= 0.6f, "symmetry");
    check(invariants.mSteps.mTimedOutPhases == 0, "timed out");
    // Final half second against the request in force then (not the run's mean
    // request, which a turn or stop changes): walking runs must hold half of
    // it along its heading; a stopped request must leave the body standing
    // still and upright.
    double finalRequested = 0.0, finalAchieved = 0.0, finalSpeed = 0.0, finalHeadingError = 0.0;
    std::size_t finalCount = 0, finalRequestedCount = 0;
    // The final half second: 30 frames at 60 Hz, rescaled with --rate.
    const std::size_t finalWindow = customRate
        ? static_cast<std::size_t>(std::max<long long>(1, std::llround(0.5 * options->mRate)))
        : 30;
    for (std::size_t i = result.mFrames.size() >= finalWindow ? result.mFrames.size() - finalWindow : 0;
         i < result.mFrames.size(); ++i)
    {
        const auto& frame = result.mFrames[i];
        const JPH::Vec3 requested(frame.mRequestedHorizontalVelocity.GetX(), frame.mRequestedHorizontalVelocity.GetY(), 0.0f);
        const JPH::Vec3 velocity(frame.mCenterOfMassVelocity.GetX(), frame.mCenterOfMassVelocity.GetY(), 0.0f);
        ++finalCount;
        finalSpeed += velocity.Length();
        if (requested.Length() > 1e-3f)
        {
            ++finalRequestedCount;
            finalRequested += requested.Length();
            finalAchieved += velocity.Dot(requested.Normalized());
            if (velocity.Length() > 1e-3f)
                finalHeadingError += std::acos(std::clamp(velocity.Normalized().Dot(requested.Normalized()), -1.0f, 1.0f));
        }
    }
    if (finalCount)
        finalSpeed /= static_cast<double>(finalCount);
    if (finalRequestedCount)
    {
        finalRequested /= static_cast<double>(finalRequestedCount);
        finalAchieved /= static_cast<double>(finalRequestedCount);
        finalHeadingError = finalHeadingError / static_cast<double>(finalRequestedCount) * 180.0 / std::numbers::pi;
    }
    const bool finalStop = finalRequestedCount == 0;
    if (finalStop)
        check(finalSpeed <= 5.0, "stopped speed");
    else
        check(finalAchieved >= 0.5 * finalRequested, "final speed");
    check(invariants.mMaxStanceFootSlide <= 1.5f, "slide");
    check(invariants.mSwingHipMotorSaturatedFrames * 20 <= invariants.mFrameCount, "swing hip saturation");
    check(saturatedFrames * 20 <= referenceFrames, "motor saturation");
    for (const auto& stage : RagdollCore::summarizeScenarioGaitStages(result))
    {
        if (stage.mStage == RagdollCore::GaitActuationStage::Inactive
            || stage.mStage == RagdollCore::GaitActuationStage::Settling
            || stage.mStage == RagdollCore::GaitActuationStage::AwaitingStance || stage.mFrames == 0)
            continue;
        const std::string name = RagdollCore::gaitActuationStageName(stage.mStage);
        check(stage.mSolverObservedFrames == stage.mFrames, name + ": solver observed");
        check(stage.mFreeVelocityLimitedFrames == 0, name + ": free velocity limit");
        check(stage.mSolverVelocityLimitedFrames == 0, name + ": solver velocity limit");
        check(runExperiment || stage.mAirborneFrames == 0, name + ": airborne");
        check(stage.mFallingFrames == 0, name + ": falling");
        check(stage.mMaxNetForce < 1e-3f, name + ": net force");
        check(stage.mMaxNetMoment < 1e-3f, name + ": net moment");
        check(stage.mMaxStanceSlide <= 1.5f, name + ": slide " + std::to_string(stage.mMaxStanceSlide));
        check(stage.mSwingHipSaturationFrames * 20 <= stage.mFrames, name + ": swing hip saturation");
        check(stage.mSwingKneeSaturationFrames * 20 <= stage.mFrames, name + ": swing knee saturation");
        check(stage.mSwingAnkleSaturationFrames * 20 <= stage.mFrames, name + ": swing ankle saturation");
    }
    const std::size_t frames = result.mFrames.size();
    const auto coordinatedFrames = std::count_if(result.mFrames.begin(), result.mFrames.end(), [](const auto& frame) {
        return frame.mReferenceGait && frame.mReferenceGait->mCoordinatedPelvisTargetZ.has_value();
    });
    const double survival = frames == 0 ? 0.0 : static_cast<double>(firstFall ? *firstFall : frames) / frames;
    const bool pass = violations.empty();
    const double score = survival + (pass ? 1.0 : 0.0) - 0.02 * static_cast<double>(violations.size());

    std::cout.precision(6);
    std::cout << "{\"pass\":" << (pass ? "true" : "false") << ",\"score\":" << score << ",\"frames\":" << frames
              << ",\"gait_profile_hash\":\"" << RagdollCore::hashReferenceGaitProfile(result.mSettings.mReferenceGait) << "\""
              << (customRate ? ",\"rate\":" + std::to_string(options->mRate) : std::string())
              << ",\"triangle_mesh_ground\":" << (result.mSettings.mTriangleMeshGround ? "true" : "false")
              << ",\"native_body\":" << (nativeBody ? "true" : "false") << ",\"first_fall\":" << (firstFall ? std::to_string(*firstFall) : "null") << ",\"transfers\":[";
    for (std::size_t i = 0; i < transfers.size(); ++i)
        std::cout << (i ? "," : "") << transfers[i];
    std::cout << "],\"reference_frames\":" << referenceFrames
              << ",\"feasibility_clamped\":" << feasibilityClamped << ",\"feasibility_frames\":" << feasibilityFrames
              << ",\"stance_knee_max_jump\":" << maxStanceKneeJump
              << ",\"stance_knee_jump_frames\":" << stanceKneeJumpFrames
              << ",\"saturated\":" << saturatedFrames
              << ",\"max_utilization\":" << maxUtilization
              << ",\"coordinated_pose_frames\":" << coordinatedFrames
              << ",\"emergency_steps\":" << emergencySteps
              << ",\"final_request\":" << finalRequested << ",\"final_along\":" << finalAchieved
              << ",\"final_com_speed\":" << finalSpeed << ",\"final_heading_error_deg\":" << finalHeadingError
              << ",\"lateral_rms\":" << (lateralFrames ? std::sqrt(lateralSquared / static_cast<double>(lateralFrames)) : 0.0)
              << ",\"quality_frames\":" << qualityFrames
              << ",\"quality_lateral_rms\":" << (qualityFrames ? std::sqrt(qualityLateralSquared / qualityFrames) : -1.0)
              << ",\"pelvis_heading_rms_degrees\":" << (qualityFrames ? std::sqrt(headingSquared / qualityFrames) * 180.0 / std::numbers::pi : -1.0)
              << ",\"maximum_pelvis_heading_degrees\":" << (qualityFrames ? maximumHeading * 180.0 / std::numbers::pi : -1.0)
              << ",\"measured_landings\":" << measuredLandings
              << ",\"mean_landing_width\":" << (measuredLandings ? landingWidth / measuredLandings : -1.0)
              << ",\"mean_landing_fore_aft_separation\":" << (measuredLandings ? landingLength / measuredLandings : -1.0)
              << ",\"mean_speed\":" << invariants.mMeanAchievedSpeed
              << ",\"final_speed\":" << invariants.mFinalHalfSecondAchievedSpeed
              << ",\"max_slide\":" << invariants.mMaxStanceFootSlide << ",\"left\":" << invariants.mSteps.mLeftStanceCount
              << ",\"right\":" << invariants.mSteps.mRightStanceCount
              << ",\"symmetry\":" << invariants.mSteps.mDurationSymmetry
              << ",\"timed_out\":" << invariants.mSteps.mTimedOutPhases << ",\"min_up\":" << invariants.mMinPelvisUpDot
              << ",\"airborne\":" << invariants.mAirborneFrames << ",\"saturation_by_body\":{";
    bool first = true;
    for (const auto& [body, count] : saturationByBody)
    {
        std::cout << (first ? "" : ",") << '"' << body << "\":" << count;
        first = false;
    }
    std::cout << "},\"violations\":[";
    for (std::size_t i = 0; i < violations.size(); ++i)
        std::cout << (i ? "," : "") << quoted(violations[i]);
    std::cout << "]}" << std::endl;
    return pass ? 0 : 1;
}
