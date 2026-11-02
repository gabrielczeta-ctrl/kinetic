// PLAY-01 captured-body tracking worker.
//
// A persistent process that runs one offline scenario per episode on the
// captured game body (creation descriptor + YAML profile) and exchanges a
// fixed-layout observation for a bounded residual action every physics step
// over stdin/stdout. The controllers exposed include:
//
//   * "reference": the current production walker (ScenarioLocomotionController
//     ::Reference) driven only by the scheduled travel/facing commands. The
//     per-step exchange carries observations; actions are acknowledgements.
//   * "direct": ScenarioLocomotionController::DirectMotorTracking. The caller
//     supplies the model-space reference pose and 49 normalized residual
//     scalars each step; this worker maps them to bounded parent-local rotation
//     offsets, derives the composed-target angular velocity and hands both to
//     the existing seam. No core controller code is changed here.
//   * "production_residual": the production walker with bounded pose-only
//     corrections. Production keeps its support, load, world-target and safety
//     decisions. The packet's generated pose is validated but not actuated.
//
// Framing: little-endian uint32 tag, uint32 byte length, payload.
//   tag 1 = UTF-8 JSON text (worker -> caller only)
//   tag 2 = float32 array (both directions)
// The worker never writes anything else to stdout. Diagnostics go to stderr.
//
// This is diagnostic tooling. It records no tracker verdict.

// clang-format off
#include <Jolt/Jolt.h>
// clang-format on

#include "nativebody.hpp"

#include <ragdollcore/scenario.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace
{
    using namespace RagdollCore;

    constexpr std::uint32_t sTagJson = 1;
    constexpr std::uint32_t sTagFloats = 2;
    constexpr float sMaximumOffset = ScenarioInputKeyframe::sMaximumDirectMotorRotationOffset;
    constexpr float sMaximumProductionOffset = ScenarioInputKeyframe::sMaximumProductionMotorRotationOffset;
    constexpr const char* sProtocol = "play01-worker-v1";

    // ---------------------------------------------------------------- framing

    void writeAll(const void* data, std::size_t size)
    {
        if (size && std::fwrite(data, 1, size, stdout) != size)
            throw std::runtime_error("stdout closed");
    }

    void writeFrame(std::uint32_t tag, const void* data, std::size_t size)
    {
        const std::uint32_t length = static_cast<std::uint32_t>(size);
        writeAll(&tag, sizeof tag);
        writeAll(&length, sizeof length);
        writeAll(data, size);
        std::fflush(stdout);
    }

    void writeJson(const std::string& text) { writeFrame(sTagJson, text.data(), text.size()); }

    void writeFloats(const std::vector<float>& values)
    {
        writeFrame(sTagFloats, values.data(), values.size() * sizeof(float));
    }

    bool readAll(void* data, std::size_t size)
    {
        return size == 0 || std::fread(data, 1, size, stdin) == size;
    }

    // Returns false on a clean end of input before a frame starts.
    bool readFloats(std::vector<float>& out)
    {
        std::uint32_t tag = 0, length = 0;
        if (!readAll(&tag, sizeof tag))
            return false;
        if (!readAll(&length, sizeof length))
            throw std::runtime_error("truncated frame header");
        if (tag != sTagFloats || length % sizeof(float) != 0 || length > (64u << 20))
            throw std::runtime_error("unexpected input frame");
        out.resize(length / sizeof(float));
        if (!readAll(out.data(), length))
            throw std::runtime_error("truncated float frame");
        return true;
    }

    // ------------------------------------------------------------------ math

    JPH::Vec3 rotationVector(JPH::QuatArg rotation)
    {
        JPH::Quat q = rotation.Normalized();
        if (q.GetW() < 0.0f)
            q = -q;
        const JPH::Vec3 v = q.GetXYZ();
        const float length = v.Length();
        if (length < 1e-9f)
            return JPH::Vec3::sZero();
        const float angle = 2.0f * std::atan2(length, q.GetW());
        return v * (angle / length);
    }

    JPH::Quat rotationFromVector(JPH::Vec3Arg vector)
    {
        const float angle = vector.Length();
        if (angle < 1e-9f)
            return JPH::Quat::sIdentity();
        return JPH::Quat::sRotation(vector / angle, angle);
    }

    bool finite(JPH::Vec3Arg v) { return std::isfinite(v.GetX()) && std::isfinite(v.GetY()) && std::isfinite(v.GetZ()); }
    bool finite(JPH::QuatArg q) { return finite(q.GetXYZ()) && std::isfinite(q.GetW()); }

    std::string hex(std::uint64_t value)
    {
        std::ostringstream out;
        out << std::hex << std::setw(16) << std::setfill('0') << value;
        return out.str();
    }

    std::string number(double value)
    {
        if (!std::isfinite(value))
            return "null";
        std::ostringstream out;
        out << std::setprecision(9) << value;
        return out.str();
    }

    std::string quoted(const std::string& text)
    {
        std::string out = "\"";
        for (const char c : text)
        {
            if (c == '"' || c == '\\')
                out += '\\';
            if (c == '\n')
                out += "\\n";
            else
                out += c;
        }
        return out + "\"";
    }

    // ------------------------------------------------------------ manifest

    struct JointAction
    {
        std::size_t mBody = 0;
        std::size_t mFirstScalar = 0;
        std::size_t mDimensions = 0;
        // Hinge axis in the parent body's local axes; unused for swing-twist.
        JPH::Vec3 mHingeAxis = JPH::Vec3::sZero();
    };

    struct Layout
    {
        std::vector<std::pair<std::string, std::size_t>> mBlocks;
        std::size_t mTotal = 0;
        void add(const std::string& name, std::size_t count)
        {
            mBlocks.emplace_back(name, count);
            mTotal += count;
        }
    };

    Layout observationLayout(std::size_t bodies)
    {
        Layout layout;
        layout.add("valid", 1);
        layout.add("step", 1);
        layout.add("time", 1);
        layout.add("body_world_position", bodies * 3);
        layout.add("body_world_rotation_xyzw", bodies * 4);
        layout.add("body_linear_velocity", bodies * 3);
        layout.add("body_angular_velocity", bodies * 3);
        layout.add("com_position", 3);
        layout.add("com_velocity", 3);
        layout.add("pelvis_up_dot", 1);
        layout.add("torso_up_dot", 1);
        layout.add("pelvis_height", 1);
        layout.add("controller_mode", 1);
        layout.add("motors_applied", 1);
        layout.add("left_foot_supported", 1);
        layout.add("right_foot_supported", 1);
        layout.add("foot_contact_count", 1);
        layout.add("ground_contact_count", 1);
        layout.add("nonfoot_support", 1);
        layout.add("left_foot_contact_tangential_speed_mean", 1);
        layout.add("right_foot_contact_tangential_speed_mean", 1);
        layout.add("left_foot_supported_contact_count", 1);
        layout.add("right_foot_supported_contact_count", 1);
        layout.add("free_velocity_limit_touched", 1);
        layout.add("solver_velocity_limit_touched", 1);
        layout.add("motor_utilization", bodies);
        layout.add("any_motor_saturated", 1);
        // Per body: 4 = swing-twist constraint-space target quaternion or
        // [hinge angle, 0, 0, 0]; then angular error and limit clamp delta.
        layout.add("delivered_motor_target", bodies * 6);
        layout.add("driven_motor_local_rotation_xyzw", bodies * 4);
        layout.add("applied_offset_parent_local", bodies * 3);
        layout.add("applied_net_force_magnitude", 1);
        layout.add("applied_pelvis_force_magnitude", 1);
        layout.add("applied_pelvis_torque_magnitude", 1);
        layout.add("requested_velocity", 2);
        return layout;
    }

    struct WorkerConfig
    {
        std::string mContext;
        std::string mProfile;
        std::string mFloor = "tiled";
    };

    struct Body
    {
        WalkSearch::NativeBody mNative;
        ScenarioSettings mSettings;
        std::vector<JointAction> mActions;
        std::size_t mActionCount = 0;
        std::size_t mPelvis = 0, mTorso = 0, mLeftFoot = 0, mRightFoot = 0, mLeftHand = 0, mRightHand = 0;
        Layout mLayout;
    };

    std::size_t bodyIndex(const RagdollCore::RagdollProfile& profile, const std::string& name)
    {
        for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
            if (profile.mBodies[i].mName == name)
                return i;
        throw std::runtime_error("profile body not found: " + name);
    }

    // Same derivation as builder.cpp createConstraintFrame (derive_frame) so
    // a scalar hinge action rotates about the axis Jolt actually solves.
    JPH::Vec3 hingeAxisParentLocal(const RagdollCore::RagdollProfile& profile, const SkeletonDescriptor& skeleton, std::size_t body)
    {
        const auto& constraint = *profile.mBodies[body].mConstraint;
        if (!constraint.mDeriveFrameFromSkeleton)
            return constraint.mParentPlaneAxis.Cross(constraint.mParentTwistAxis).Normalized();
        const auto bone = [&](const std::string& name) -> const SkeletonBone& {
            for (const auto& candidate : skeleton.mBones)
                if (candidate.mName == name)
                    return candidate;
            throw std::runtime_error("skeleton bone not found: " + name);
        };
        const SkeletonBone& parentBone = bone(profile.mBodies[bodyIndex(profile, *profile.mBodies[body].mParentBody)].mBone);
        const SkeletonBone& childBone = bone(profile.mBodies[body].mBone);
        const SkeletonBone* frameAxisBone = nullptr;
        if (constraint.mFrameAxisBone)
            for (const auto& other : profile.mBodies)
                if (other.mName == *constraint.mFrameAxisBone)
                    frameAxisBone = &bone(other.mBone);
        const JPH::Vec3 modelOffset = childBone.mModelTransform.mTranslation - parentBone.mModelTransform.mTranslation;
        const JPH::Vec3 modelAxis = frameAxisBone
            ? frameAxisBone->mModelTransform.mTranslation - childBone.mModelTransform.mTranslation
            : modelOffset;
        const JPH::Vec3 modelTwist = modelAxis.Normalized();
        const JPH::Vec3 reference
            = std::abs(modelTwist.Dot(JPH::Vec3::sAxisZ())) < 0.9f ? JPH::Vec3::sAxisZ() : JPH::Vec3::sAxisX();
        const JPH::Vec3 modelPlane = modelTwist.Cross(reference).Normalized();
        const JPH::Quat parentInverse = parentBone.mModelTransform.mRotation.Conjugated();
        const JPH::Vec3 twist1 = parentInverse * modelTwist;
        const JPH::Vec3 plane1 = parentInverse * modelPlane;
        return plane1.Cross(twist1).Normalized();
    }

    Body loadBody(const WorkerConfig& config)
    {
        Body body;
        body.mNative = WalkSearch::loadNativeBody(config.mContext, config.mProfile, body.mSettings);
        const auto& profile = body.mNative.mProfile;
        const auto& skeleton = body.mNative.mSkeleton;
        if (skeleton.mBones.size() != profile.mBodies.size())
            throw std::runtime_error("skeleton/profile size mismatch");
        for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
            if (skeleton.mBones[i].mName != profile.mBodies[i].mBone)
                throw std::runtime_error("skeleton bone order differs from profile body order");
        for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
        {
            const auto& constraint = profile.mBodies[i].mConstraint;
            if (!constraint || constraint->mMotor.mMaximumTorque <= 0.0f)
                continue;
            if (!profile.mBodies[i].mParentBody)
                throw std::runtime_error("constrained body without a parent");
            JointAction action;
            action.mBody = i;
            action.mFirstScalar = body.mActionCount;
            action.mDimensions = constraint->mType == ConstraintType::Hinge ? 1 : 3;
            if (constraint->mType == ConstraintType::Hinge)
                action.mHingeAxis = hingeAxisParentLocal(profile, skeleton, i);
            body.mActionCount += action.mDimensions;
            body.mActions.push_back(action);
        }
        body.mPelvis = bodyIndex(profile, profile.mLandmarks.mPelvis);
        body.mTorso = profile.mLandmarks.mSpine.empty() ? body.mPelvis : bodyIndex(profile, profile.mLandmarks.mSpine.back());
        body.mLeftFoot = bodyIndex(profile, profile.mLandmarks.mLeftFoot.value_or("left_foot"));
        body.mRightFoot = bodyIndex(profile, profile.mLandmarks.mRightFoot.value_or("right_foot"));
        body.mLeftHand = bodyIndex(profile, profile.mLandmarks.mLeftHand.value_or("left_hand"));
        body.mRightHand = bodyIndex(profile, profile.mLandmarks.mRightHand.value_or("right_hand"));
        body.mLayout = observationLayout(profile.mBodies.size());

        // Floor: the calibration contract's tiled mesh by default. The mesh
        // grid is read from the environment by the scenario, so set it here
        // deterministically and record it in the manifest.
        if (config.mFloor == "tiled")
        {
            body.mSettings.mTriangleMeshGround = true;
#ifdef _WIN32
            _putenv_s("OPENMW_RAGDOLL_MESH_GRID", "64");
#else
            setenv("OPENMW_RAGDOLL_MESH_GRID", "64", 1);
#endif
        }
        else if (config.mFloor == "mesh")
        {
            body.mSettings.mTriangleMeshGround = true;
#ifdef _WIN32
            _putenv_s("OPENMW_RAGDOLL_MESH_GRID", "");
#else
            unsetenv("OPENMW_RAGDOLL_MESH_GRID");
#endif
        }
        else if (config.mFloor == "box")
            body.mSettings.mTriangleMeshGround = false;
        else
            throw std::runtime_error("unknown floor: " + config.mFloor);
        return body;
    }

    std::string manifestJson(const Body& body, const WorkerConfig& config)
    {
        const auto& profile = body.mNative.mProfile;
        const auto& skeleton = body.mNative.mSkeleton;
        std::ostringstream out;
        out << std::setprecision(9);
        out << "{\"kind\":\"manifest\",\"protocol\":" << quoted(sProtocol)
            << ",\"context_path\":" << quoted(config.mContext) << ",\"profile_path\":" << quoted(config.mProfile)
            << ",\"profile_id\":" << quoted(profile.mId) << ",\"creation_profile_hash\":" << body.mNative.mProfileHash
            << ",\"total_mass\":" << number(body.mNative.mTotalMass) << ",\"uniform_scale\":" << number(body.mNative.mUniformScale)
            << ",\"floor\":" << quoted(config.mFloor)
            << ",\"triangle_mesh_ground\":" << (body.mSettings.mTriangleMeshGround ? "true" : "false")
            << ",\"mesh_grid_env\":" << quoted(std::getenv("OPENMW_RAGDOLL_MESH_GRID") ? std::getenv("OPENMW_RAGDOLL_MESH_GRID") : "")
            << ",\"manifold_reduction_env\":" << quoted(std::getenv("OPENMW_RAGDOLL_PHYS_MANIFOLD") ? std::getenv("OPENMW_RAGDOLL_PHYS_MANIFOLD") : "")
            << ",\"fixed_time_step\":" << number(body.mSettings.mFixedTimeStep)
            << ",\"collision_steps\":" << body.mSettings.mCollisionSteps
            << ",\"ground_friction\":" << number(body.mSettings.mGroundFriction)
            << ",\"gravity\":" << number(body.mSettings.mGravity)
            << ",\"maximum_offset_radians\":" << number(sMaximumOffset)
            << ",\"maximum_production_residual_offset_radians\":" << number(sMaximumProductionOffset)
            << ",\"controller_contracts\":{"
            << "\"reference\":{\"code\":0,\"action\":\"acknowledgement\"},"
            << "\"direct\":{\"code\":1,\"maximum_offset_radians\":" << number(sMaximumOffset) << "},"
            << "\"seam\":{\"code\":2,\"action\":\"zero_only\"},"
            << "\"production_residual\":{\"code\":3,\"maximum_offset_radians\":" << number(sMaximumProductionOffset)
            << ",\"pose_input\":\"current_final_production_pose\",\"packet_pose\":\"validated_ignored\","
            << "\"velocity_targets\":\"production_owned\",\"forces_and_loads\":\"production_owned\","
            << "\"world_targets_and_released_axes\":\"production_owned_may_override_pose_channels\","
            << "\"applied_offset\":\"bounded_pose_input_not_guaranteed_delivered_target\","
            << "\"zero_identity\":\"exact_physical_frames_distinct_controller_setting\"}}"
            << ",\"action_basis\":\"parent_body_axes\",\"action_count\":" << body.mActionCount
            << ",\"landmarks\":{\"pelvis\":" << body.mPelvis << ",\"torso\":" << body.mTorso << ",\"left_foot\":" << body.mLeftFoot
            << ",\"right_foot\":" << body.mRightFoot << ",\"left_hand\":" << body.mLeftHand << ",\"right_hand\":" << body.mRightHand << "}";
        out << ",\"bodies\":[";
        for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
        {
            const auto& b = profile.mBodies[i];
            const auto& bone = skeleton.mBones[i];
            const auto& t = bone.mModelTransform;
            out << (i ? "," : "") << "{\"index\":" << i << ",\"name\":" << quoted(b.mName) << ",\"bone\":" << quoted(b.mBone)
                << ",\"parent\":" << bone.mParent << ",\"mass_fraction\":" << number(b.mMassFraction)
                << ",\"constraint\":";
            if (!b.mConstraint)
                out << "null";
            else
            {
                const auto& c = *b.mConstraint;
                out << "{\"type\":" << (c.mType == ConstraintType::Hinge ? "\"hinge\"" : "\"swing_twist\"")
                    << ",\"maximum_torque\":" << number(c.mMotor.mMaximumTorque) << ",\"frequency\":" << number(c.mMotor.mFrequency)
                    << ",\"damping\":" << number(c.mMotor.mDamping)
                    << ",\"normal_half_cone\":" << number(c.mNormalHalfConeAngle) << ",\"plane_half_cone\":" << number(c.mPlaneHalfConeAngle)
                    << ",\"twist_min\":" << number(c.mTwistMinAngle) << ",\"twist_max\":" << number(c.mTwistMaxAngle)
                    << ",\"hinge_min\":" << number(c.mHingeMinAngle) << ",\"hinge_max\":" << number(c.mHingeMaxAngle)
                    << ",\"derive_frame\":" << (c.mDeriveFrameFromSkeleton ? "true" : "false")
                    << ",\"frame_axis_bone\":" << (c.mFrameAxisBone ? quoted(*c.mFrameAxisBone) : "null") << "}";
            }
            out << ",\"creation_translation\":[" << number(t.mTranslation.GetX()) << "," << number(t.mTranslation.GetY()) << ","
                << number(t.mTranslation.GetZ()) << "],\"creation_rotation_xyzw\":[" << number(t.mRotation.GetX()) << ","
                << number(t.mRotation.GetY()) << "," << number(t.mRotation.GetZ()) << "," << number(t.mRotation.GetW()) << "]}";
        }
        out << "],\"actions\":[";
        for (std::size_t i = 0; i < body.mActions.size(); ++i)
        {
            const auto& a = body.mActions[i];
            out << (i ? "," : "") << "{\"body\":" << a.mBody << ",\"first_scalar\":" << a.mFirstScalar << ",\"dimensions\":" << a.mDimensions;
            if (a.mDimensions == 1)
                out << ",\"hinge_axis_parent_local\":[" << number(a.mHingeAxis.GetX()) << "," << number(a.mHingeAxis.GetY()) << ","
                    << number(a.mHingeAxis.GetZ()) << "]";
            out << "}";
        }
        out << "],\"observation_layout\":[";
        for (std::size_t i = 0; i < body.mLayout.mBlocks.size(); ++i)
            out << (i ? "," : "") << "[" << quoted(body.mLayout.mBlocks[i].first) << "," << body.mLayout.mBlocks[i].second << "]";
        out << "],\"observation_size\":" << body.mLayout.mTotal
            << ",\"action_message_size\":" << (2 + body.mActionCount + profile.mBodies.size() * 7)
            << ",\"controller_modes\":[\"standing\",\"bracing\",\"stumbling\",\"falling\",\"fallen\",\"getting_up\",\"blend_back\"]}";
        return out.str();
    }

    // ------------------------------------------------------------- episode

    struct Command
    {
        std::uint64_t mStep = 0;
        JPH::Vec3 mVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mFacing = JPH::Vec3::sZero();
    };

    struct EpisodeConfig
    {
        bool mDirect = false;
        bool mProductionResidual = false;
        // Controller code 2: direct tracking that leaves the offset vector
        // empty (the seam's exact no-correction path) and rejects any nonzero
        // action, so zero-action identity compares the two seam paths.
        bool mReferencePath = false;
        std::uint64_t mSteps = 0;
        std::optional<ScenarioImpulse> mImpulse;
        std::optional<std::uint64_t> mMotorsOffStep;
        std::vector<Command> mCommands;
    };

    // Episode header float32 layout:
    //   [0] controller (0 production walker, 1 direct, 2 direct via the
    //       empty-offset seam path, zero actions only, 3 production residual)
    //   [1] steps
    //   [2] impulse step (-1 none) [3] impulse body [4..6] impulse vector
    //   [7] motors-off step (-1 none)            [8] command count N
    //   then N * (step, vx, vy, fx, fy)
    EpisodeConfig parseEpisode(const std::vector<float>& values, const Body& body)
    {
        if (values.size() < 9)
            throw std::runtime_error("episode header too short");
        for (const float v : values)
            if (!std::isfinite(v))
                throw std::runtime_error("non-finite episode header");
        EpisodeConfig config;
        config.mDirect = values[0] == 1.0f || values[0] == 2.0f;
        config.mProductionResidual = values[0] == 3.0f;
        config.mReferencePath = values[0] == 2.0f;
        if (values[0] != 0.0f && values[0] != 1.0f && values[0] != 2.0f && values[0] != 3.0f)
            throw std::runtime_error("unknown controller");
        config.mSteps = static_cast<std::uint64_t>(values[1]);
        if (config.mSteps < 1 || config.mSteps > 36000)
            throw std::runtime_error("episode step count out of range");
        if (values[2] >= 0.0f)
        {
            ScenarioImpulse impulse;
            impulse.mStep = static_cast<std::uint64_t>(values[2]);
            impulse.mBodyIndex = static_cast<std::size_t>(values[3]);
            impulse.mImpulse = JPH::Vec3(values[4], values[5], values[6]);
            if (impulse.mBodyIndex >= body.mNative.mProfile.mBodies.size() || impulse.mStep >= config.mSteps)
                throw std::runtime_error("impulse body or step out of range");
            config.mImpulse = impulse;
        }
        if (values[7] >= 0.0f)
            config.mMotorsOffStep = static_cast<std::uint64_t>(values[7]);
        const std::size_t count = static_cast<std::size_t>(values[8]);
        if (values.size() != 9 + count * 5 || count == 0)
            throw std::runtime_error("episode command list size mismatch");
        std::uint64_t previous = 0;
        for (std::size_t i = 0; i < count; ++i)
        {
            Command command;
            command.mStep = static_cast<std::uint64_t>(values[9 + i * 5]);
            command.mVelocity = JPH::Vec3(values[10 + i * 5], values[11 + i * 5], 0.0f);
            command.mFacing = JPH::Vec3(values[12 + i * 5], values[13 + i * 5], 0.0f);
            if ((i && command.mStep <= previous) || (!i && command.mStep != 0))
                throw std::runtime_error("commands must start at step 0 and increase");
            previous = command.mStep;
            config.mCommands.push_back(command);
        }
        return config;
    }

    const Command& commandAt(const EpisodeConfig& config, std::uint64_t step)
    {
        const Command* current = &config.mCommands.front();
        for (const auto& command : config.mCommands)
            if (command.mStep <= step)
                current = &command;
        return *current;
    }

    struct EpisodeState
    {
        // Composed parent-local targets from the previous accepted action, for
        // the finite-difference velocity target.
        std::vector<JPH::Quat> mPreviousComposed;
        bool mHaveComposed = false;
        std::vector<JPH::Vec3> mAppliedOffsets;
        std::uint64_t mFallStep = std::numeric_limits<std::uint64_t>::max();
    };

    void appendVec3(std::vector<float>& out, JPH::Vec3Arg v)
    {
        out.push_back(v.GetX());
        out.push_back(v.GetY());
        out.push_back(v.GetZ());
    }
    void appendRVec3(std::vector<float>& out, JPH::RVec3Arg v)
    {
        out.push_back(static_cast<float>(v.GetX()));
        out.push_back(static_cast<float>(v.GetY()));
        out.push_back(static_cast<float>(v.GetZ()));
    }
    void appendQuat(std::vector<float>& out, JPH::QuatArg q)
    {
        out.push_back(q.GetX());
        out.push_back(q.GetY());
        out.push_back(q.GetZ());
        out.push_back(q.GetW());
    }

    std::vector<float> observation(const Body& body, const ScenarioFrame* frame, std::uint64_t step,
        const EpisodeState& state, const Command& command)
    {
        const std::size_t bodies = body.mNative.mProfile.mBodies.size();
        std::vector<float> out;
        out.reserve(body.mLayout.mTotal);
        if (!frame)
        {
            out.assign(body.mLayout.mTotal, 0.0f);
            out[1] = static_cast<float>(step);
            return out;
        }
        out.push_back(1.0f);
        out.push_back(static_cast<float>(step));
        out.push_back(frame->mTime);
        if (frame->mBodies.size() != bodies)
            throw std::runtime_error("frame body count mismatch");
        for (const auto& b : frame->mBodies)
            appendRVec3(out, b.mWorldPosition);
        for (const auto& b : frame->mBodies)
            appendQuat(out, b.mWorldRotation);
        for (const auto& b : frame->mBodies)
            appendVec3(out, b.mLinearVelocity);
        for (const auto& b : frame->mBodies)
            appendVec3(out, b.mAngularVelocity);
        appendRVec3(out, frame->mCenterOfMass);
        appendVec3(out, frame->mCenterOfMassVelocity);
        out.push_back(frame->mPelvisUpDot);
        out.push_back(frame->mTorsoUpDot);
        out.push_back(frame->mPelvisHeight);
        out.push_back(static_cast<float>(static_cast<int>(frame->mController.mMode)));
        out.push_back(frame->mMotorsApplied ? 1.0f : 0.0f);
        out.push_back(frame->mSupport.mLeftFootSupported ? 1.0f : 0.0f);
        out.push_back(frame->mSupport.mRightFootSupported ? 1.0f : 0.0f);
        out.push_back(static_cast<float>(frame->mSupport.mFootContactCount));
        out.push_back(static_cast<float>(frame->mSupport.mGroundContactCount));
        bool nonfoot = false;
        double leftSpeed = 0, rightSpeed = 0;
        std::size_t leftCount = 0, rightCount = 0;
        for (const auto& contact : frame->mContacts)
        {
            if (!contact.hasSupport())
                continue;
            if (contact.mBodyIndex != body.mLeftFoot && contact.mBodyIndex != body.mRightFoot)
                nonfoot = true;
            if (!contact.mPointVelocity)
                continue;
            const double speed = std::hypot(contact.mPointVelocity->GetX(), contact.mPointVelocity->GetY());
            if (contact.mBodyIndex == body.mLeftFoot)
            {
                leftSpeed += speed;
                ++leftCount;
            }
            else if (contact.mBodyIndex == body.mRightFoot)
            {
                rightSpeed += speed;
                ++rightCount;
            }
        }
        out.push_back(nonfoot ? 1.0f : 0.0f);
        out.push_back(leftCount ? static_cast<float>(leftSpeed / leftCount) : 0.0f);
        out.push_back(rightCount ? static_cast<float>(rightSpeed / rightCount) : 0.0f);
        out.push_back(static_cast<float>(leftCount));
        out.push_back(static_cast<float>(rightCount));
        out.push_back(frame->mSolverImpulse && frame->mSolverImpulse->mFreeVelocityLimitTouched ? 1.0f : 0.0f);
        out.push_back(frame->mSolverImpulse && frame->mSolverImpulse->mVelocityLimitTouched ? 1.0f : 0.0f);
        float peak = 0.0f;
        for (std::size_t i = 0; i < bodies; ++i)
        {
            const float utilization = i < frame->mJointSupportMotorUtilizations.size() ? frame->mJointSupportMotorUtilizations[i] : 0.0f;
            peak = std::max(peak, utilization);
            out.push_back(utilization);
        }
        out.push_back(peak >= 0.999f ? 1.0f : 0.0f);
        for (std::size_t i = 0; i < bodies; ++i)
        {
            const auto* target = i < frame->mJointMotorTargets.size() && frame->mJointMotorTargets[i]
                ? &*frame->mJointMotorTargets[i] : nullptr;
            if (!target)
            {
                for (int k = 0; k < 6; ++k)
                    out.push_back(0.0f);
                continue;
            }
            if (target->mConstraintType == ConstraintType::Hinge)
            {
                out.push_back(target->mTargetHingeAngle.value_or(0.0f));
                out.push_back(0.0f);
                out.push_back(0.0f);
                out.push_back(0.0f);
            }
            else if (target->mTargetOrientation)
                appendQuat(out, *target->mTargetOrientation);
            else
                for (int k = 0; k < 4; ++k)
                    out.push_back(0.0f);
            out.push_back(target->mAngularError);
            out.push_back(target->mTargetClampDelta);
        }
        if (frame->mDrivenMotorLocalRotations.size() == bodies)
            for (const auto rotation : frame->mDrivenMotorLocalRotations)
                appendQuat(out, rotation);
        else
            for (std::size_t i = 0; i < bodies; ++i)
                appendQuat(out, JPH::Quat(0.0f, 0.0f, 0.0f, 0.0f));
        for (std::size_t i = 0; i < bodies; ++i)
            appendVec3(out, i < state.mAppliedOffsets.size() ? state.mAppliedOffsets[i] : JPH::Vec3::sZero());
        out.push_back(frame->mAppliedNetForce.Length());
        out.push_back(frame->mAppliedPelvisForce.Length());
        out.push_back(frame->mAppliedPelvisTorque.Length());
        out.push_back(command.mVelocity.GetX());
        out.push_back(command.mVelocity.GetY());
        if (out.size() != body.mLayout.mTotal)
            throw std::runtime_error("observation layout mismatch");
        return out;
    }

    // Action float32 layout (direct and production-residual modes):
    //   [0] step  [1] motors enabled (0/1)  [2 .. 2+A) normalized residuals in
    //   manifest order, already tanh-bounded and filtered by the caller;
    //   then B * (tx, ty, tz, qx, qy, qz, qw) model-space reference pose.
    // Reference mode: [0] step only.
    std::optional<ScenarioInputKeyframe> buildKeyframe(const Body& body, const EpisodeConfig& config,
        const std::vector<float>& action, std::uint64_t step, const ScenarioFrame* previous, EpisodeState& state)
    {
        const auto& profile = body.mNative.mProfile;
        const std::size_t bodies = profile.mBodies.size();
        if (action.empty() || !std::isfinite(action[0]) || action[0] != static_cast<float>(step))
            throw std::runtime_error("stale or missing action step");
        const Command& command = commandAt(config, step);
        if (!config.mDirect && !config.mProductionResidual)
        {
            if (action.size() != 1)
                throw std::runtime_error("reference mode expects an acknowledgement only");
            return std::nullopt;
        }
        const std::size_t expected = 2 + body.mActionCount + bodies * 7;
        if (action.size() != expected)
            throw std::runtime_error("action message has the wrong dimension");
        for (const float value : action)
            if (!std::isfinite(value))
                throw std::runtime_error("non-finite action");
        if (action[1] != 0.0f && action[1] != 1.0f)
            throw std::runtime_error("motors flag must be 0 or 1");
        const bool motorsEnabled = action[1] == 1.0f
            && (!config.mProductionResidual || !config.mMotorsOffStep || step < *config.mMotorsOffStep);
        for (std::size_t i = 0; i < body.mActionCount; ++i)
            if (std::abs(action[2 + i]) > 1.0f)
                throw std::runtime_error("normalized action outside [-1, 1]");

        ScenarioInputKeyframe key;
        key.mStep = step;
        key.mRequestedHorizontalVelocity = command.mVelocity;
        key.mFacing = command.mFacing;
        key.mMotorsEnabled = motorsEnabled;
        RagdollPose pose;
        pose.mBodyTransforms.resize(bodies);
        const std::size_t poseOffset = 2 + body.mActionCount;
        for (std::size_t i = 0; i < bodies; ++i)
        {
            const float* p = &action[poseOffset + i * 7];
            Transform t;
            t.mTranslation = JPH::Vec3(p[0], p[1], p[2]);
            JPH::Quat q(p[3], p[4], p[5], p[6]);
            const float length = q.Length();
            if (!(length > 0.99f && length < 1.01f))
                throw std::runtime_error("reference rotation is not a unit quaternion");
            t.mRotation = q / length;
            pose.mBodyTransforms[i] = t;
        }
        // Residual mapping: swing-twist three-vectors are radially projected
        // to the unit ball, hinges scale their derived parent-local axis.
        const float maximumOffset = config.mProductionResidual ? sMaximumProductionOffset : sMaximumOffset;
        std::vector<JPH::Vec3> offsets(bodies, JPH::Vec3::sZero());
        for (const auto& joint : body.mActions)
        {
            JPH::Vec3 v;
            if (joint.mDimensions == 3)
            {
                v = JPH::Vec3(action[2 + joint.mFirstScalar], action[2 + joint.mFirstScalar + 1], action[2 + joint.mFirstScalar + 2]);
                const float length = v.Length();
                if (length > 1.0f)
                    v /= length;
            }
            else
                v = joint.mHingeAxis * action[2 + joint.mFirstScalar];
            offsets[joint.mBody] = v * maximumOffset;
            // Guard the seam bound against float rounding of a unit vector.
            const float length = offsets[joint.mBody].Length();
            if (length > maximumOffset)
                offsets[joint.mBody] *= maximumOffset / length;
            if (config.mProductionResidual && offsets[joint.mBody].LengthSq() > maximumOffset * maximumOffset)
                offsets[joint.mBody] *= std::nextafter(maximumOffset, 0.0f) / offsets[joint.mBody].Length();
            // Multiplying a negative hinge-axis component by zero produces
            // -0.0. Production's absent offsets are +0.0; canonicalize only
            // this new path so zero-residual observation bytes match too.
            if (config.mProductionResidual && offsets[joint.mBody].LengthSq() == 0.0f)
                offsets[joint.mBody] = JPH::Vec3::sZero();
        }
        state.mAppliedOffsets = offsets;
        if (config.mReferencePath)
        {
            for (const auto& offset : offsets)
                if (!offset.IsNearZero())
                    throw std::runtime_error("reference-path episodes accept zero actions only");
        }
        else
            key.mMotorTargetRotationOffsets = offsets;
        if (config.mProductionResidual)
        {
            // Neither the generated pose nor its finite-difference velocity
            // may replace any part of the production baseline. Pose offsets
            // are composed by the scenario only when production drives.
            return key;
        }
        key.mMotorTarget = pose;

        // Composed parent-local target and its finite-difference velocity,
        // converted to the world child-minus-parent convention of the seam.
        std::vector<JPH::Quat> composed(bodies, JPH::Quat::sIdentity());
        for (const auto& joint : body.mActions)
        {
            const int parent = body.mNative.mSkeleton.mBones[joint.mBody].mParent;
            if (parent < 0)
                throw std::runtime_error("actuated joint without parent");
            const JPH::Quat relative = pose.mBodyTransforms[parent].mRotation.Conjugated() * pose.mBodyTransforms[joint.mBody].mRotation;
            composed[joint.mBody] = (rotationFromVector(offsets[joint.mBody]) * relative).Normalized();
        }
        if (motorsEnabled)
        {
            std::vector<JPH::Vec3> velocities(bodies, JPH::Vec3::sZero());
            if (state.mHaveComposed)
            {
                const float dt = body.mSettings.mFixedTimeStep;
                for (const auto& joint : body.mActions)
                {
                    const int parent = body.mNative.mSkeleton.mBones[joint.mBody].mParent;
                    const JPH::Vec3 local = rotationVector(composed[joint.mBody] * state.mPreviousComposed[joint.mBody].Conjugated()) / dt;
                    const JPH::Quat parentWorld = previous && static_cast<std::size_t>(parent) < previous->mBodies.size()
                        ? previous->mBodies[parent].mWorldRotation
                        : body.mSettings.mRootRotation * body.mNative.mSkeleton.mBones[parent].mModelTransform.mRotation;
                    velocities[joint.mBody] = parentWorld * local;
                    if (!finite(velocities[joint.mBody]))
                        throw std::runtime_error("non-finite velocity target");
                }
            }
            key.mMotorTargetAngularVelocities = velocities;
        }
        state.mPreviousComposed = composed;
        state.mHaveComposed = true;
        return key;
    }

    std::string endJson(const ScenarioResult& result, const Body& body, const EpisodeState& state, double seconds)
    {
        const auto invariants = summarizeScenarioInvariants(result);
        std::size_t falling = 0, fallen = 0, standing = 0;
        std::size_t nonfootFrames = 0, airborne = 0;
        for (const auto& frame : result.mFrames)
        {
            const auto mode = frame.mController.mMode;
            standing += mode == ActiveControllerMode::Standing;
            falling += mode == ActiveControllerMode::Falling;
            fallen += mode == ActiveControllerMode::Fallen;
            bool support = false, nonfoot = false;
            for (const auto& contact : frame.mContacts)
                if (contact.hasSupport())
                {
                    support = true;
                    if (contact.mBodyIndex != body.mLeftFoot && contact.mBodyIndex != body.mRightFoot)
                        nonfoot = true;
                }
            nonfootFrames += nonfoot;
            airborne += !support;
        }
        std::ostringstream out;
        out << std::setprecision(9) << "{\"kind\":\"end\",\"completed\":" << (result.completed() ? "true" : "false")
            << ",\"frames\":" << result.mFrames.size() << ",\"step_count\":" << result.mStepCount
            << ",\"telemetry_hash\":" << quoted(hex(hashScenarioTelemetry(result)))
            << ",\"profile_hash\":" << result.mProfileHash
            << ",\"fall_step\":" << (state.mFallStep == std::numeric_limits<std::uint64_t>::max() ? std::string("null") : std::to_string(state.mFallStep))
            << ",\"standing_frames\":" << standing << ",\"falling_frames\":" << falling << ",\"fallen_frames\":" << fallen
            << ",\"nonfoot_support_frames\":" << nonfootFrames << ",\"airborne_frames\":" << airborne
            << ",\"free_velocity_limited_frames\":" << invariants.mFreeVelocityLimitedFrames
            << ",\"solver_velocity_limited_frames\":" << invariants.mSolverVelocityLimitedFrames
            << ",\"max_stance_foot_slide\":" << number(invariants.mMaxStanceFootSlide)
            << ",\"total_stance_foot_slide\":" << number(invariants.mTotalStanceFootSlide)
            << ",\"mean_achieved_speed\":" << number(invariants.mMeanAchievedSpeed)
            << ",\"final_half_second_speed\":" << number(invariants.mFinalHalfSecondAchievedSpeed)
            << ",\"max_applied_net_force\":" << number(invariants.mMaxAppliedNetForce)
            << ",\"max_net_actuation_moment\":" << number(invariants.mMaxNetActuationMoment)
            << ",\"landmarks\":{\"pelvis\":" << result.mLandmarks.mPelvis << ",\"torso\":" << result.mLandmarks.mTorso
            << ",\"left_foot\":" << result.mLandmarks.mLeftFoot << ",\"right_foot\":" << result.mLandmarks.mRightFoot << "}"
            << ",\"target_pelvis_height\":" << number(result.mTargetPelvisHeight)
            << ",\"wall_seconds\":" << number(seconds) << ",\"errors\":[";
        for (std::size_t i = 0; i < result.mErrors.size(); ++i)
            out << (i ? "," : "") << quoted(result.mErrors[i]);
        out << "]}";
        return out.str();
    }

    void runEpisode(const Body& body, const EpisodeConfig& config)
    {
        ScenarioSettings settings = body.mSettings;
        settings.mLocomotionController = config.mProductionResidual ? ScenarioLocomotionController::ProductionResidual
            : config.mDirect ? ScenarioLocomotionController::DirectMotorTracking : ScenarioLocomotionController::Reference;
        ScenarioScript script;
        script.mDurationSeconds = static_cast<float>(config.mSteps) * settings.mFixedTimeStep;
        for (const auto& command : config.mCommands)
        {
            ScenarioInputKeyframe key;
            key.mStep = command.mStep;
            key.mRequestedHorizontalVelocity = command.mVelocity;
            key.mFacing = command.mFacing;
            script.mKeyframes.push_back(key);
        }
        if (config.mMotorsOffStep && !config.mDirect)
        {
            // Reference mode: a scheduled release keyframe copies the active command.
            ScenarioInputKeyframe key = script.mKeyframes.front();
            for (const auto& existing : script.mKeyframes)
                if (existing.mStep <= *config.mMotorsOffStep)
                    key = existing;
            key.mStep = *config.mMotorsOffStep;
            key.mMotorsEnabled = false;
            auto at = std::lower_bound(script.mKeyframes.begin(), script.mKeyframes.end(), key.mStep,
                [](const auto& k, auto value) { return k.mStep < value; });
            if (at != script.mKeyframes.end() && at->mStep == key.mStep)
                *at = key;
            else
                script.mKeyframes.insert(at, key);
            // Later scheduled commands would re-enable motors; keep them off.
            for (auto& existing : script.mKeyframes)
                if (existing.mStep > key.mStep)
                    existing.mMotorsEnabled = false;
        }
        if (config.mImpulse)
            script.mImpulses.push_back(*config.mImpulse);

        EpisodeState state;
        std::vector<float> action;
        const auto started = std::chrono::steady_clock::now();
        const auto provider = [&](std::uint64_t step, const ScenarioFrame* previous) -> std::optional<ScenarioInputKeyframe> {
            if (previous && state.mFallStep == std::numeric_limits<std::uint64_t>::max()
                && (previous->mController.mMode == ActiveControllerMode::Falling
                    || previous->mController.mMode == ActiveControllerMode::Fallen))
                state.mFallStep = previous->mStep;
            writeFloats(observation(body, previous, step, state, commandAt(config, step)));
            if (!readFloats(action))
                throw std::runtime_error("action input closed mid-episode");
            return buildKeyframe(body, config, action, step, previous, state);
        };
        const ScenarioResult result
            = runRagdollScenario(body.mNative.mSkeleton, body.mNative.mProfile, settings, script, provider);
        if (!result.mFrames.empty() && state.mFallStep == std::numeric_limits<std::uint64_t>::max())
        {
            const auto& last = result.mFrames.back();
            if (last.mController.mMode == ActiveControllerMode::Falling || last.mController.mMode == ActiveControllerMode::Fallen)
                state.mFallStep = last.mStep;
        }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        // The final frame is not observed through the provider; send it once
        // so callers see the last completed physical state before the end.
        writeFloats(observation(body, result.mFrames.empty() ? nullptr : &result.mFrames.back(),
            result.mFrames.size(), state, commandAt(config, result.mFrames.empty() ? 0 : result.mFrames.size())));
        writeJson(endJson(result, body, state, seconds));
    }

    int worker(int argc, char** argv)
    {
        WorkerConfig config;
        if (argc < 3)
        {
            std::cerr << "usage: ragdollcore-walksearch --play01-worker <context.creation.txt> <profile.yaml> [--floor tiled|mesh|box]\n";
            return 2;
        }
        config.mContext = argv[1];
        config.mProfile = argv[2];
        for (int i = 3; i + 1 < argc; i += 2)
            if (std::string(argv[i]) == "--floor")
                config.mFloor = argv[i + 1];
            else
            {
                std::cerr << "unknown option " << argv[i] << '\n';
                return 2;
            }
#ifdef _WIN32
        _setmode(_fileno(stdin), _O_BINARY);
        _setmode(_fileno(stdout), _O_BINARY);
#endif
        Body body = loadBody(config);
        writeJson(manifestJson(body, config));
        std::vector<float> header;
        while (readFloats(header))
        {
            const EpisodeConfig episode = parseEpisode(header, body);
            runEpisode(body, episode);
        }
        return 0;
    }
}

int runPlay01Worker(int argc, char** argv)
{
    try
    {
        return worker(argc, argv);
    }
    catch (const std::exception& error)
    {
        // Rejections happen before the affected physics solve; report them
        // on both channels so the caller can attribute the stop.
        std::cerr << "play01-worker: " << error.what() << std::endl;
        try
        {
            writeJson(std::string("{\"kind\":\"error\",\"message\":") + quoted(error.what()) + "}");
        }
        catch (...)
        {
        }
        return 2;
    }
}
