#include <cstdio>
#include <cstdlib>
#ifndef OPENMW_WALKSEARCH_REPLAYDRIVER_H
#define OPENMW_WALKSEARCH_REPLAYDRIVER_H
#include <ragdollcore/replay.hpp>
#include <components/ragdollprofile/loader.hpp>
#include <cmath>
#include <iomanip>
#include <stdexcept>

// Diagnostic reconstruction, never an implicit claim of native/offline parity.
static int runCapturedScenario(const char* capturePath, const char* contextPath, const char* outputPath, const char* profilePath, int overrideCount, char** overrides)
{
    using namespace RagdollCore;
    std::ifstream binary(capturePath, std::ios::binary | std::ios::ate);
    if (!binary || binary.tellg() <= 0) throw std::runtime_error("Missing replay capture");
    std::vector<std::byte> bytes(static_cast<std::size_t>(binary.tellg()));
    binary.seekg(0); binary.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
    auto captured = deserializeReplay(bytes);
    if (!captured || captured->mFrames.empty()) throw std::runtime_error("Invalid replay capture");
    // Input-substitution probes (2026-09-11, strafe-native-input): replace one
    // recorded input channel to find which native-only input causes a fall
    // that walksearch's authored scenario does not show. Env-gated, inert
    // when unset; the JSON reports them so a probed run is never mistaken
    // for a faithful replay.
    std::optional<std::size_t> freezeSource;
    std::optional<JPH::Vec3> probeFacing;
    std::optional<RagdollPose> frozenPose;
    const char* freezeTargetsText = std::getenv("OPENMW_RAGDOLL_REPLAY_FREEZE_TARGETS");
    const char* facingText = std::getenv("OPENMW_RAGDOLL_REPLAY_FACING");
    if ((freezeTargetsText || facingText) && captured->mInputCount >= 10)
    {
        std::optional<JPH::Vec3> forcedFacing;
        if (facingText)
        {
            float fx = 0.0f, fy = 0.0f;
            if (std::sscanf(facingText, "%f,%f", &fx, &fy) != 2) throw std::runtime_error("OPENMW_RAGDOLL_REPLAY_FACING wants x,y");
            forcedFacing = JPH::Vec3(fx, fy, 0.0f);
        }
        if (freezeTargetsText)
            freezeSource = std::strtoul(freezeTargetsText, nullptr, 10);
        probeFacing = forcedFacing;
        std::cerr << "replay probe: " << (freezeSource ? "targets frozen " : "") << (forcedFacing ? "facing forced " : "") << std::endl;
    }
    std::ifstream context(contextPath);
    const auto point = [&]() { double x = 0, y = 0, z = 0;
        if (!(context >> x >> y >> z)) throw std::runtime_error("Truncated context vector");
        return JPH::RVec3(x, y, z); };
    const auto vector = [&]() { return JPH::Vec3(point()); };
    const auto rotation = [&]() { float x = 0, y = 0, z = 0, w = 1;
        if (!(context >> x >> y >> z >> w)) throw std::runtime_error("Truncated context rotation");
        return JPH::Quat(x, y, z, w); };
    unsigned version = 0; std::uint64_t profileHash = 0; std::size_t warmup = 0, boneCount = 0, frameCount = 0;
    std::ifstream profileStream(profilePath);
    auto loaded = ::RagdollProfile::load(profileStream);
    if (!loaded.isValid()) throw std::runtime_error("Cannot load captured native YAML profile");
    auto profile = std::move(*loaded.mProfile);
    ScenarioSettings settings;
    settings.mReferenceGait.mContinuousSwingTrajectory = std::getenv("OPENMW_RAGDOLL_CONTINUOUS_SWING") != nullptr;
    std::string profileId;
    context >> version >> std::quoted(profileId) >> profileHash >> warmup;
    if ((version < 1 || version > 5) || profileId != profile.mId || hashProfile(profile) != profileHash || profileHash != captured->mMetadata.mProfileHash)
        throw std::runtime_error("Creation profile does not match the authored offline profile");
    context >> settings.mTotalMass >> settings.mUniformScale >> settings.mGroundHeight;
    settings.mRootPosition = point(); settings.mRootRotation = rotation();
    settings.mInitialActorMotion.mLinearVelocity = vector();
    settings.mInitialActorMotion.mAngularVelocity = vector();
    context >> boneCount;
    if (boneCount != profile.mBodies.size()) throw std::runtime_error("Unexpected creation skeleton dimensions");
    SkeletonDescriptor skeleton; skeleton.mId = "captured-native-creation";
    for (std::size_t i = 0; i < boneCount; ++i)
    {
        SkeletonBone bone; context >> std::quoted(bone.mName) >> bone.mParent;
        bone.mModelTransform.mTranslation = vector(); bone.mModelTransform.mRotation = rotation();
        skeleton.mBones.push_back(bone);
    }
    if (version >= 3)
    {
        settings.mReplayBodyIds.resize(boneCount);
        for (auto& id : settings.mReplayBodyIds) context >> id;
    }
    if (version >= 4)
    {
        std::uint32_t groundId = 0; context >> groundId; settings.mReplayGroundId = groundId;
        settings.mReplayGroundPosition = point(); settings.mReplayGroundRotation = rotation();
        std::string path; context >> settings.mGroundFriction >> settings.mReplayGroundRestitution >> std::quoted(path);
        std::ifstream shape(path, std::ios::binary | std::ios::ate);
        if (!shape || shape.tellg() <= 0 || shape.tellg() > 16 * 1024 * 1024)
            throw std::runtime_error("Missing or oversized ground shape capture");
        settings.mReplayGroundShape.resize(static_cast<std::size_t>(shape.tellg()));
        shape.seekg(0); shape.read(reinterpret_cast<char*>(settings.mReplayGroundShape.data()), settings.mReplayGroundShape.size());
        if (!shape) throw std::runtime_error("Truncated ground shape capture");
        // Probe (2026-09-11, strafe-native-input): replace the captured
        // native floor by the scenario BoxShape at the same nominal height,
        // to test whether the native floor geometry is what differs from
        // the authored scenario. Env-gated; unset is byte-identical.
        if (std::getenv("OPENMW_RAGDOLL_REPLAY_BOX_GROUND"))
        {
            settings.mReplayGroundId = {};
            settings.mReplayGroundShape.clear();
            std::cerr << "replay probe: native ground replaced by box" << std::endl;
        }
    }
    context >> frameCount;
    if (frameCount != captured->mFrames.size()) throw std::runtime_error("Context/replay frame mismatch");
    settings.mFixedTimeStep = captured->mMetadata.mFixedTimeStep;
    settings.mCollisionSteps = captured->mMetadata.mCollisionSteps;
    settings.mLocomotionController = ScenarioLocomotionController::Reference;
    settings.mReferenceInputsAreFiltered = true;
    settings.mReferenceGait.mBrakeLateralTransfer
        = std::getenv("OPENMW_RAGDOLL_REPLAY_TRANSFER_BRAKING") != nullptr
        && std::getenv("OPENMW_RAGDOLL_REPLAY_LEGACY_TRANSFER") == nullptr;
    // Reuse walksearch's named float profile fields without overriding captured
    // geometry, timing, or adapter settings. Reject typos instead of silently
    // screening a different controller. The scenario/profile hashes include them.
    std::map<std::string, float> profileOverrides;
    for (int index = 0; index < overrideCount; ++index)
    {
        const std::string argument(overrides[index]);
        const auto separator = argument.find('=');
        const auto key = argument.substr(0, separator);
        const bool known = std::any_of(std::begin(sProfileFields), std::end(sProfileFields),
            [&](const auto& field) { return key == field.mName; });
        if (separator == std::string::npos || !known || profileOverrides.contains(key)
            || !assignProfile(settings, key, argument.substr(separator + 1)))
            throw std::runtime_error("Invalid or duplicate replay profile override: " + argument);
        for (const auto& field : sProfileFields)
            if (key == field.mName) profileOverrides.emplace(key, settings.mReferenceGait.*(field.mMember));
    }
    const bool reconstructPlayerTravel = std::getenv("OPENMW_RAGDOLL_REPLAY_PLAYER_RAW_TRAVEL") != nullptr;
    const bool reconstructNpcTravel = std::getenv("OPENMW_RAGDOLL_REPLAY_NPC_RAW_TRAVEL") != nullptr;
    if (reconstructPlayerTravel && reconstructNpcTravel)
        throw std::runtime_error("Select either player or NPC raw travel reconstruction, not both");
    if (reconstructPlayerTravel || reconstructNpcTravel)
    {
        settings.mReferenceInputsAreFiltered = false;
        settings.mReferenceGaitPlayerInput = reconstructPlayerTravel;
        settings.mReferenceGaitMeasuredStopping = true;
    }
    settings.mReplayPreserveContactOrder = true;
    settings.mReplayDiscardSupportOnReferenceReset = std::getenv("OPENMW_RAGDOLL_REPLAY_NATIVE_SUPPORT_RESET") != nullptr;
    settings.mReplayObserveRestartRequest = std::getenv("OPENMW_RAGDOLL_REPLAY_RESTART_OBSERVATION") != nullptr;
    settings.mReplayNativeBraceLimits = std::getenv("OPENMW_RAGDOLL_REPLAY_SKIP_BRACE_LIMITS") == nullptr;
    settings.mReplayAutomaticRecoveryRequest = std::getenv("OPENMW_RAGDOLL_REPLAY_SKIP_RECOVERY_REQUEST") == nullptr;
    settings.mReplayBraceTargets = settings.mReplayNativeBraceLimits
        && std::getenv("OPENMW_RAGDOLL_REPLAY_SKIP_BRACE_TARGETS") == nullptr;
    settings.mReferenceGaitGroundsPlacement = true;
    settings.mTriangleMeshGround = std::getenv("OPENMW_RAGDOLL_REPLAY_BOX_GROUND") == nullptr;
    ScenarioScript script;
    script.mDurationSeconds = static_cast<float>(frameCount + warmup) * settings.mFixedTimeStep;
    Replay expected = *captured;
    const bool compareSwingHistory = expected.mControllerStateCount == ContinuousSwingReplayControllerStateChannels;
    if (compareSwingHistory) settings.mReferenceGait.mContinuousSwingTrajectory = true;
    std::optional<std::size_t> firstSwingHistoryDifference;
    const bool movingReference = version >= 2 && !std::getenv("OPENMW_RAGDOLL_REPLAY_FIXED_ROOT");
    settings.mReplayComputeHorizontalRoot = movingReference
        && std::getenv("OPENMW_RAGDOLL_REPLAY_COMPUTE_HORIZONTAL_ROOT") != nullptr;
    const auto inverse = settings.mRootRotation.Conjugated();
    const auto convert = [&](Transform t, JPH::RVec3Arg root, JPH::QuatArg q) {
        t.mTranslation = (inverse * JPH::Vec3(root - settings.mRootPosition)
            + inverse * q * (t.mTranslation * settings.mUniformScale)) / settings.mUniformScale;
        // Preserve captured quaternion components. Normalizing here changes
        // native motor inputs by a few ULPs even when both roots are identity.
        t.mRotation = inverse * q * t.mRotation;
        return t;
    };
    for (std::size_t i = 0; i < warmup + frameCount; ++i)
    {
        std::size_t index; context >> index;
        if (index != i) throw std::runtime_error("Non-contiguous consumed input timeline");
        const auto root = point(); const auto q = rotation();
        if (movingReference) settings.mReplayReferenceFrames.push_back({ root, q });
        ScenarioInputKeyframe key; key.mStep = i; key.mRequestedHorizontalVelocity = vector(); key.mFacing = vector();
        if (probeFacing && !key.mRequestedHorizontalVelocity.IsNearZero()) key.mFacing = *probeFacing;
        if ((reconstructPlayerTravel || reconstructNpcTravel) && i > warmup)
        {
            // Native observes steering after Update. The command consumed by
            // this solve comes from the previous captured raw input.
            const auto previous = decodePoweredReplayInput(*captured, captured->mFrames[i - warmup - 1]);
            if (!previous || !previous->mRawRequestedHorizontalVelocity)
                throw std::runtime_error("Raw travel reconstruction requires captured raw channels");
            key.mRequestedHorizontalVelocity = *previous->mRawRequestedHorizontalVelocity;
        }
        RagdollPose pose;
        if (version >= 2)
        {
            const auto consumedRoot = point();
            const auto consumedRotation = rotation();
            for (std::size_t body = 0; body < boneCount; ++body)
            {
                Transform t; t.mTranslation = vector(); t.mRotation = rotation();
                pose.mBodyTransforms.push_back(movingReference ? t : convert(t, consumedRoot, consumedRotation));
            }
        }
        else if (i < warmup)
            for (const auto& bone : skeleton.mBones) pose.mBodyTransforms.push_back(bone.mModelTransform);
        else
        {
            const auto decoded = decodePoweredReplayInput(*captured, captured->mFrames[i - warmup]);
            if (!decoded) throw std::runtime_error("Missing powered pose channels");
            for (const auto& t : decoded->mAnimationTargetPose) pose.mBodyTransforms.push_back(convert(t, root, q));
        }
        // Probe: mirror-average the left_/right_ leg body targets so the
        // animation-derived targets (and the base pose taken from them) are
        // laterally symmetric like a symmetrized creation skeleton.
        static const bool symmetricTargets = std::getenv("OPENMW_RAGDOLL_REPLAY_SYMMETRIC_TARGETS") != nullptr;
        if (symmetricTargets && pose.mBodyTransforms.size() == skeleton.mBones.size())
        {
            for (const char* part : { "thigh", "calf", "foot" })
            {
                std::optional<std::size_t> left, right;
                for (std::size_t b = 0; b < skeleton.mBones.size(); ++b)
                {
                    if (skeleton.mBones[b].mName == std::string("left_") + part) left = b;
                    if (skeleton.mBones[b].mName == std::string("right_") + part) right = b;
                }
                if (!left || !right) continue;
                Transform& l = pose.mBodyTransforms[*left];
                Transform& r = pose.mBodyTransforms[*right];
                const JPH::Vec3 average(0.5f * (l.mTranslation.GetX() - r.mTranslation.GetX()),
                    0.5f * (l.mTranslation.GetY() + r.mTranslation.GetY()), 0.5f * (l.mTranslation.GetZ() + r.mTranslation.GetZ()));
                JPH::Quat rm(r.mRotation.GetX(), -r.mRotation.GetY(), -r.mRotation.GetZ(), r.mRotation.GetW());
                if (l.mRotation.Dot(rm) < 0.0f) rm = -rm;
                const JPH::Quat q = (l.mRotation + rm).Normalized();
                l.mTranslation = average; l.mRotation = q;
                r.mTranslation = JPH::Vec3(-average.GetX(), average.GetY(), average.GetZ());
                r.mRotation = JPH::Quat(q.GetX(), -q.GetY(), -q.GetZ(), q.GetW());
            }
        }
        if (freezeSource && i == *freezeSource) frozenPose = pose;
        if (frozenPose && i > *freezeSource && pose.mBodyTransforms.size() == frozenPose->mBodyTransforms.size()) pose = *frozenPose;
        if (i >= warmup)
        {
            const auto decoded = decodePoweredReplayInput(*captured, captured->mFrames[i - warmup]);
            if (!decoded) throw std::runtime_error("Invalid powered recovery inputs");
            // The event is consumed by post-solve controller observation in both
            // adapters. Unlike raw travel, it belongs to this captured step.
            if (decoded->mRecoveryAnimationFinished)
                key.mRecoveryAnimationFinished = *decoded->mRecoveryAnimationFinished;
        }
        if (i >= warmup && !movingReference)
            for (auto& body : expected.mFrames[i - warmup].mBodies) body.mTransform = convert(body.mTransform, root, q);
        if (version >= 5)
        {
            RagdollPose animation;
            for (std::size_t body = 0; body < boneCount; ++body)
            {
                Transform t; t.mTranslation = vector(); t.mRotation = rotation();
                animation.mBodyTransforms.push_back(t);
            }
            if (!movingReference) throw std::runtime_error("Paired poses require moving reference replay");
            key.mRecoveryMotorTarget = std::move(animation);
        }
        key.mMotorTarget = std::move(pose); script.mKeyframes.push_back(std::move(key));
    }
    if (!context) throw std::runtime_error("Truncated reconstruction context");
    context >> std::ws;
    bool seenRecoveryFinish = false;
    bool seenDirectImpulses = false;
    bool seenRecoveryRequests = false;
    while (context.peek() != std::char_traits<char>::eof())
    {
        std::string tag; std::size_t count = 0;
        context >> tag >> count;
        if (!context)
            throw std::runtime_error("Truncated reconstruction context trailer");
        if (tag == "recovery_finish_events_v1")
        {
            if (seenRecoveryFinish || count > warmup + frameCount)
                throw std::runtime_error("Invalid recovery event extension");
            seenRecoveryFinish = true;
            settings.mReplayRecoveryFinishEventsCaptured = true;
            settings.mReplayRecoveryFinishSteps.resize(count);
            for (auto& step : settings.mReplayRecoveryFinishSteps)
                context >> step;
        }
        else if (tag == "direct_impulses_v1")
        {
            if (seenDirectImpulses || count > 10000)
                throw std::runtime_error("Invalid direct impulse trailer");
            seenDirectImpulses = true;
            for (std::size_t i = 0; i < count; ++i)
            {
                ScenarioImpulse impulse;
                context >> impulse.mStep >> impulse.mBodyIndex;
                impulse.mImpulse = vector();
                unsigned hasPoint = 0; context >> hasPoint;
                const auto worldPoint = point();
                if (!context || hasPoint > 1 || impulse.mStep >= script.mKeyframes.size())
                    throw std::runtime_error("Invalid direct impulse event");
                if (hasPoint) impulse.mWorldPoint = worldPoint;
                script.mKeyframes[impulse.mStep].mPrePlanningImpulses.push_back(impulse);
            }
        }
        else if (tag == "recovery_requests_v1")
        {
            if (seenRecoveryRequests || count > frameCount + warmup)
                throw std::runtime_error("Invalid recovery request trailer");
            seenRecoveryRequests = true;
            settings.mReplayRecoveryRequestSteps.resize(count);
            for (auto& step : settings.mReplayRecoveryRequestSteps)
                context >> step;
        }
        else
            throw std::runtime_error("Unexpected reconstruction context trailer: " + tag);
        if (!context)
            throw std::runtime_error("Truncated reconstruction context trailer: " + tag);
        context >> std::ws;
    }
    const auto result = runRagdollScenario(skeleton, profile, settings, script);
    std::filesystem::create_directories(outputPath);
    const std::filesystem::path output(outputPath);
    std::ofstream initial(output / "initial-bodies.csv");
    std::ofstream commands(output / "startup-commands.csv");
    commands << "step,body,qx,qy,qz,qw,hinge,vx,vy,vz,stiffness_mode,spring,damping,"
        "constraint_type,world_orientation,orientation_present,hinge_present,angular_error,target_clamp_delta,support_torque_x,support_torque_y,support_torque_z,"
        "drive_known,swing_or_hinge_state,twist_state,min_torque_x,min_torque_y,min_torque_z,max_torque_x,max_torque_y,max_torque_z,"
        "drive_stiffness_mode,drive_spring,drive_damping,twist_stiffness_mode,twist_spring,twist_damping,child_world_qx,child_world_qy,child_world_qz,child_world_qw,parent_world_qx,parent_world_qy,parent_world_qz,parent_world_qw\n"
        << std::setprecision(17);
    for (std::size_t step = 0; step < result.mStartupMotorTargets.size(); ++step)
        for (std::size_t body = 1; body < result.mStartupMotorTargets[step].size(); ++body)
        {
            const auto& m = result.mStartupMotorTargets[step][body];
            const auto q = m.mTargetOrientation.value_or(JPH::Quat::sIdentity()); const auto v = m.mTargetAngularVelocity;
            commands << step << ',' << body << ',' << q.GetX() << ',' << q.GetY() << ',' << q.GetZ() << ',' << q.GetW()
                << ',' << m.mTargetHingeAngle.value_or(0) << ',' << v.GetX() << ',' << v.GetY() << ',' << v.GetZ()
                << ',' << m.mSpring.mStiffnessMode << ',' << m.mSpring.mFrequencyOrStiffness << ',' << m.mSpring.mDamping
                << ',' << static_cast<unsigned>(m.mConstraintType) << ',' << m.mWorldOrientation
                << ',' << m.mTargetOrientation.has_value() << ',' << m.mTargetHingeAngle.has_value()
                << ',' << m.mAngularError << ',' << m.mTargetClampDelta;
            if (step < result.mFrames.size() && result.mFrames[step].mJointSupport
                && body < result.mFrames[step].mJointSupport->mJointTorques.size())
            {
                const auto torque = result.mFrames[step].mJointSupport->mJointTorques[body];
                commands << ',' << torque.GetX() << ',' << torque.GetY() << ',' << torque.GetZ();
            }
            else commands << ",,,";
            commands << ',' << m.mDrive.has_value();
            if (m.mDrive)
            {
                const auto& d = *m.mDrive;
                commands << ',' << d.mSwingOrHingeState << ',' << d.mTwistState;
                for (int axis = 0; axis < 3; ++axis) commands << ',' << d.mMinimumTorque[axis];
                for (int axis = 0; axis < 3; ++axis) commands << ',' << d.mMaximumTorque[axis];
                commands << ',' << d.mSwingOrHingeSpring.mStiffnessMode
                    << ',' << d.mSwingOrHingeSpring.mFrequencyOrStiffness << ',' << d.mSwingOrHingeSpring.mDamping
                    << ',' << d.mTwistSpring.mStiffnessMode
                    << ',' << d.mTwistSpring.mFrequencyOrStiffness << ',' << d.mTwistSpring.mDamping;
            }
            else commands << ",,,,,,,,,,,,,,";
            for (const auto& world : { m.mChildWorldTarget, m.mParentWorldTarget })
            {
                if (world) commands << ',' << world->GetX() << ',' << world->GetY() << ',' << world->GetZ() << ',' << world->GetW();
                else commands << ",,,,";
            }
            commands << '\n';
        }
    initial << "body,x,y,z,qx,qy,qz,qw,vx,vy,vz\n" << std::setprecision(17);
    for (std::size_t i = 0; i < result.mInitialTelemetry.mBodies.size(); ++i)
    {
        const auto& b = result.mInitialTelemetry.mBodies[i];
        initial << profile.mBodies[i].mName << ',' << b.mPosition.GetX() << ',' << b.mPosition.GetY() << ',' << b.mPosition.GetZ()
            << ',' << b.mRotation.GetX() << ',' << b.mRotation.GetY() << ',' << b.mRotation.GetZ() << ',' << b.mRotation.GetW()
            << ',' << b.mMotion.mLinearVelocity.GetX() << ',' << b.mMotion.mLinearVelocity.GetY() << ',' << b.mMotion.mLinearVelocity.GetZ() << '\n';
    }
    std::ofstream(output / "offline.csv") << formatScenarioTraceCsv(result);
    std::ofstream contacts(output / "contacts.csv");
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
    if (!result.completed() || result.mFrames.size() < frameCount + warmup)
    {
        for (const auto& error : result.mErrors) std::cerr << error << '\n';
        throw std::runtime_error("Reconstructed scenario did not complete");
    }
    std::ofstream swingHistory(output / "swing-history.csv");
    swingHistory << "frame";
    for (unsigned channel = 0; channel < SwingTrajectoryHistoryChannels; ++channel) swingHistory << ",state_" << channel;
    swingHistory << '\n' << std::setprecision(9);
    Replay actual = expected;
    std::ofstream initialDifferences(output / "first-frame-bodies.csv");
    initialDifferences << "body,translation_difference,linear_velocity_difference,angular_velocity_difference,native_z,offline_z\n";
    std::optional<std::size_t> firstBodyDifference, firstFall;
    std::ofstream errorCurve(output / "physical-error-curve.csv");
    std::ofstream worldCurve(output / "world-position-error-curve.csv");
    worldCurve << "frame,max_world_position_error\n" << std::setprecision(17);
    // Cross-capture comparisons must not mistake a different actor reference
    // frame for different physical motion. Export full world-space states.
    std::ofstream worldBodies(output / "world-bodies.csv");
    worldBodies << "frame,body,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n" << std::setprecision(17);
    errorCurve << "frame,max_translation_error,max_linear_velocity_error,max_angular_velocity_error,worst_translation_body,max_quaternion_component_error\n"
               << std::setprecision(17);
    for (std::size_t i = 0; i < frameCount; ++i)
    {
        const auto& frame = result.mFrames[i + warmup]; auto& destination = actual.mFrames[i];
        if (!firstFall && (frame.mController.mMode == ActiveControllerMode::Falling
                || frame.mController.mMode == ActiveControllerMode::Fallen)) firstFall = i;
        const auto history = encodeSwingTrajectoryHistory(frame.mSwingTrajectoryHistory);
        if (!history) throw std::runtime_error("Invalid reconstructed swing history");
        if (compareSwingHistory)
        {
            const auto channels = std::span<const float>(captured->mFrames[i].mControllerState)
                .subspan(PoweredReplayControllerStateChannels);
            std::optional<SwingTrajectoryHistory> decodedHistory;
            if (!decodeSwingTrajectoryHistory(channels, decodedHistory))
                throw std::runtime_error("Invalid captured swing history");
            for (std::size_t channel = 0; channel < history->size(); ++channel)
                if (channels[channel] != (*history)[channel] && !firstSwingHistoryDifference)
                    firstSwingHistoryDifference = i;
        }
        swingHistory << i;
        for (float value : *history) swingHistory << ',' << value;
        swingHistory << '\n';
        destination.mControllerState.clear(); // compare physical state separately from different controller layouts
        double maxWorldError = 0;
        for (std::size_t body = 0; body < frame.mBodies.size(); ++body)
        {
            const auto& state = frame.mBodies[body];
            worldBodies << i << ',' << body << ',' << state.mWorldPosition.GetX() << ','
                << state.mWorldPosition.GetY() << ',' << state.mWorldPosition.GetZ();
            const auto q = state.mWorldRotation;
            worldBodies << ',' << q.GetX() << ',' << q.GetY() << ',' << q.GetZ() << ',' << q.GetW();
            for (const auto& v : { state.mLinearVelocity, state.mAngularVelocity })
                worldBodies << ',' << v.GetX() << ',' << v.GetY() << ',' << v.GetZ();
            worldBodies << '\n';
            if (movingReference)
            {
                const auto& reference = settings.mReplayReferenceFrames[i + warmup];
                const auto nativeWorld = reference.mPosition + JPH::RVec3(reference.mRotation
                    * (expected.mFrames[i].mBodies[body].mTransform.mTranslation * settings.mUniformScale));
                maxWorldError = std::max(maxWorldError, (frame.mBodies[body].mWorldPosition - nativeWorld).Length());
            }
            destination.mBodies[body].mTransform = frame.mBodies[body].mModelTransform;
            destination.mBodies[body].mLinearVelocity = frame.mBodies[body].mLinearVelocity;
            destination.mBodies[body].mAngularVelocity = frame.mBodies[body].mAngularVelocity;
            if (i == 0)
            {
                const auto& native = expected.mFrames[i].mBodies[body];
                const auto& offline = destination.mBodies[body];
                initialDifferences << std::setprecision(9) << profile.mBodies[body].mName << ','
                    << (native.mTransform.mTranslation - offline.mTransform.mTranslation).Length() << ','
                    << (native.mLinearVelocity - offline.mLinearVelocity).Length() << ','
                    << (native.mAngularVelocity - offline.mAngularVelocity).Length() << ','
                    << native.mTransform.mTranslation.GetZ() << ',' << offline.mTransform.mTranslation.GetZ() << '\n';
            }
            if (!firstBodyDifference && (destination.mBodies[body].mTransform.mTranslation
                    - expected.mFrames[i].mBodies[body].mTransform.mTranslation).Length() > 0.01f)
                firstBodyDifference = i;
        }
        if (movingReference) worldCurve << i << ',' << maxWorldError << '\n';
        expected.mFrames[i].mControllerState.clear();
        float translation = 0, linear = 0, angular = 0, rotation = 0;
        std::size_t worstBody = 0;
        for (std::size_t body = 0; body < destination.mBodies.size(); ++body)
        {
            const auto& native = expected.mFrames[i].mBodies[body];
            const auto& offline = destination.mBodies[body];
            const float distance = (native.mTransform.mTranslation - offline.mTransform.mTranslation).Length();
            if (distance > translation) { translation = distance; worstBody = body; }
            linear = std::max(linear, (native.mLinearVelocity - offline.mLinearVelocity).Length());
            angular = std::max(angular, (native.mAngularVelocity - offline.mAngularVelocity).Length());
            const auto a = native.mTransform.mRotation;
            const auto b = offline.mTransform.mRotation;
            const float same = std::max({ std::abs(a.GetX() - b.GetX()), std::abs(a.GetY() - b.GetY()),
                std::abs(a.GetZ() - b.GetZ()), std::abs(a.GetW() - b.GetW()) });
            const float opposite = std::max({ std::abs(a.GetX() + b.GetX()), std::abs(a.GetY() + b.GetY()),
                std::abs(a.GetZ() + b.GetZ()), std::abs(a.GetW() + b.GetW()) });
            rotation = std::max(rotation, std::min(same, opposite));
        }
        errorCurve << i << ',' << translation << ',' << linear << ',' << angular << ','
                   << profile.mBodies[worstBody].mName << ',' << rotation << '\n';
        // Physical-only comparison: inputs are verified separately against STEP_TRACE.
        destination.mInputs.clear(); expected.mFrames[i].mInputs.clear();
    }
    expected.mControllerStateCount = actual.mControllerStateCount = 0;
    expected.mInputCount = actual.mInputCount = 0;
    // Explicit normalization for tolerance comparison only, not a bitwise parity claim.
    actual.mMetadata.mWorkerThreadCount = expected.mMetadata.mWorkerThreadCount = 1;
    ReplayComparisonTolerances tolerance;
    tolerance.mTranslation = 0.01f; tolerance.mRotationAngle = 0.001f;
    tolerance.mLinearVelocity = 0.01f; tolerance.mAngularVelocity = 0.001f;
    const auto comparison = compareReplays(expected, actual, tolerance);
    if (comparison.mFirstDivergentStep)
    {
        const auto frame = *comparison.mFirstDivergentStep;
        std::ofstream differences(output / "first-divergence-bodies.csv");
        differences << "body,translation_difference,linear_velocity_difference,angular_velocity_difference\n" << std::setprecision(17);
        for (std::size_t body = 0; body < profile.mBodies.size(); ++body)
        {
            const auto& native = expected.mFrames[frame].mBodies[body];
            const auto& offline = actual.mFrames[frame].mBodies[body];
            differences << profile.mBodies[body].mName << ','
                << (native.mTransform.mTranslation - offline.mTransform.mTranslation).Length() << ','
                << (native.mLinearVelocity - offline.mLinearVelocity).Length() << ','
                << (native.mAngularVelocity - offline.mAngularVelocity).Length() << '\n';
        }
    }
    for (const auto& pair : { std::pair{ "native-normalized.omwrdpl", &expected },
             std::pair{ "offline-physical.omwrdpl", &actual } })
    {
        const auto data = serializeReplay(*pair.second);
        if (!data) throw std::runtime_error("Invalid physical comparison replay");
        std::ofstream f(output / pair.first, std::ios::binary);
        f.write(reinterpret_cast<const char*>(data->data()), data->size());
    }
    std::cout << "{\"frames\":" << frameCount << ",\"warmup_frames\":" << warmup
        << ",\"moving_reference_root\":" << (movingReference ? "true" : "false")
        << ",\"computed_horizontal_root\":" << (settings.mReplayComputeHorizontalRoot ? "true" : "false")
        << ",\"recovery_finished_input_captured\":"
        << (decodePoweredReplayInput(*captured, captured->mFrames.front())->mRecoveryAnimationFinished.has_value() ? "true" : "false")
        << ",\"gait_profile_hash\":" << hashReferenceGaitProfile(settings.mReferenceGait)
        << ",\"recovery_finish_events_captured\":" << (settings.mReplayRecoveryFinishEventsCaptured ? "true" : "false")
        << ",\"recovery_finish_event_count\":" << settings.mReplayRecoveryFinishSteps.size()
        << ",\"restart_observation_probe\":"
        << (settings.mReplayObserveRestartRequest ? "true" : "false")
        << ",\"native_support_reset_probe\":"
        << (settings.mReplayDiscardSupportOnReferenceReset ? "true" : "false")
        << ",\"lateral_transfer_braking\":" << (settings.mReferenceGait.mBrakeLateralTransfer ? "true" : "false")
        << ",\"reconstructed_npc_travel\":" << (reconstructNpcTravel ? "true" : "false")
        << ",\"reconstructed_player_travel\":" << (reconstructPlayerTravel ? "true" : "false")
        << ",\"native_brace_limits\":" << (settings.mReplayNativeBraceLimits ? "true" : "false")
        << ",\"contact_root_observation_replay\":false"
        << ",\"contact_root_input_count\":0"
        << ",\"automatic_recovery_request\":" << (settings.mReplayAutomaticRecoveryRequest ? "true" : "false")
        << ",\"brace_targets\":" << (settings.mReplayBraceTargets ? "true" : "false")
        << ",\"swing_history_compared\":" << (compareSwingHistory ? "true" : "false")
        << ",\"first_swing_history_difference_frame\":"
        << (firstSwingHistoryDifference ? std::to_string(*firstSwingHistoryDifference) : "null")
        << ",\"comparison_scope\":\"physical_state\""
        << ",\"native_workers\":" << captured->mMetadata.mWorkerThreadCount
        << ",\"offline_workers\":1,\"first_divergent_frame\":"
        << (comparison.mFirstDivergentStep ? std::to_string(*comparison.mFirstDivergentStep) : "null")
        << ",\"first_translation_difference_frame\":"
        << (firstBodyDifference ? std::to_string(*firstBodyDifference) : "null")
        << ",\"offline_first_fall_frame\":" << (firstFall ? std::to_string(*firstFall) : "null")
        << ",\"equivalence_proven\":false,\"profile_overrides\":{";
    bool firstOverride = true;
    for (const auto& [key, value] : profileOverrides)
    {
        if (!firstOverride) std::cout << ',';
        std::cout << '\"' << key << "\":" << value;
        firstOverride = false;
    }
    std::cout << "}}\n";
    return 0;
}
#endif
