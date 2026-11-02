#include <gtest/gtest.h>

#include <ragdollcore/reactionpipeline.hpp>
#include <ragdollcore/scenario.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace RagdollCore;

namespace
{
    constexpr float Pi = 3.14159265358979323846f;

    std::string number(float value, int precision = 3)
    {
        if (!std::isfinite(value))
            return "0";
        std::ostringstream out;
        out.setf(std::ios::fixed);
        out.precision(precision);
        out << value;
        return out.str();
    }

    std::string point(const JPH::Vec3& value)
    {
        return "[" + number(value.GetX()) + "," + number(value.GetY()) + "," + number(value.GetZ()) + "]";
    }

    std::string point(const JPH::RVec3& value)
    {
        return "[" + number(static_cast<float>(value.GetX())) + ","
            + number(static_cast<float>(value.GetY())) + ","
            + number(static_cast<float>(value.GetZ())) + "]";
    }

    int findBone(const std::vector<std::string>& names, const std::string& wanted)
    {
        for (std::size_t i = 0; i < names.size(); ++i)
            if (names[i] == wanted)
                return static_cast<int>(i);
        return -1;
    }

    // Simple model-space forward kinematics over the rest offsets so a rotated
    // parent visibly carries its children (arms, head) in the viewer.
    std::vector<JPH::Vec3> forwardKinematics(const std::vector<Transform>& pose,
        const std::vector<JPH::Vec3>& restPositions, const std::vector<int>& parents)
    {
        std::vector<JPH::Vec3> world = restPositions;
        for (std::size_t i = 0; i < pose.size(); ++i)
        {
            const int parent = i < parents.size() ? parents[i] : -1;
            if (parent < 0)
                continue;
            const JPH::Vec3 localOffset = restPositions[i] - restPositions[parent];
            const JPH::Quat rotation = pose[parent].mRotation;
            world[i] = world[parent] + rotation * localOffset;
        }
        return world;
    }

    ReactionContractInput baselineInput(std::uint64_t step)
    {
        ReactionContractInput input;
        input.mFramePhysicsStep = step;
        input.mFixedStepDuration = 1.0f / 60.0f;
        input.mMode = ReactionBodyMode::Walking;
        input.mWalking = true;
        input.mMoving = true;
        input.mSupport.mSupportEstablished = true;
        input.mSupport.mBothFeetSupported = false;
        input.mSupport.mLeftFootSupported = true;
        input.mSupport.mRightFootSupported = false;
        return input;
    }

    void addHit(ReactionContractInput& input, std::uint64_t contactId, AnatomicalRegion region,
        float impulse, bool withReceipt, float healthLoss, float severity)
    {
        input.mContact.mValid = true;
        input.mContact.mContactId = contactId;
        input.mContact.mRegion = region;
        input.mContact.mTargetBodyIndex = 0;
        input.mImpulse.mProvenance = ImpulseProvenance::SolverMeasured;
        input.mImpulse.mMagnitude = impulse;
        input.mImpulse.mAlreadyApplied = true;
        if (withReceipt)
        {
            input.mReceipt = ReactionReceipt{ contactId, healthLoss, 0.0f };
            input.mWoundSeverity = severity;
        }
    }
}

TEST(RagdollCoreReactionViewerTest, DISABLED_writesOfflineViewerTrace)
{
    const char* outputPath = std::getenv("OPENMW_RAGDOLL_REACTION_VIEWER");
    if (outputPath == nullptr || *outputPath == '\0')
        GTEST_SKIP() << "set OPENMW_RAGDOLL_REACTION_VIEWER to an output json path";

    // ---- Walker scene: the real reference controller on the synthetic body.
    ScenarioScript script;
    script.mDurationSeconds = 6.0f;
    ScenarioInputKeyframe walk;
    walk.mStep = 60;
    walk.mRequestedHorizontalVelocity = JPH::Vec3(0.0f, 45.0f, 0.0f);
    script.mKeyframes.push_back(walk);
    ScenarioImpulse push;
    push.mStep = 300;
    push.mBodyIndex = 5;
    push.mImpulse = JPH::Vec3(1600.0f, 0.0f, 0.0f);
    script.mImpulses.push_back(push);

    const SkeletonDescriptor skeleton = makeScenarioHumanoidSkeleton();
    const ScenarioResult walker = runRagdollScenario(
        skeleton, makeScenarioHumanoidProfile(), ScenarioSettings{}, script);
    ASSERT_TRUE(walker.completed()) << (walker.mErrors.empty() ? "" : walker.mErrors.front());

    // ---- Reaction scene: the real pipeline on the same body, with FK so the
    //      guard/cradle/limp offsets read in the render.
    std::vector<Transform> rest;
    std::vector<JPH::Vec3> restPositions;
    std::vector<int> parents(skeleton.mBones.size(), -1);
    for (std::size_t i = 0; i < skeleton.mBones.size(); ++i)
        parents[i] = skeleton.mBones[i].mParent;
    if (!walker.mFrames.empty() && walker.mFrames.front().mBodies.size() == skeleton.mBones.size())
    {
        for (const auto& body : walker.mFrames.front().mBodies)
        {
            rest.push_back(body.mModelTransform);
            restPositions.push_back(body.mModelTransform.mTranslation);
        }
    }
    else
    {
        for (const auto& bone : skeleton.mBones)
        {
            rest.push_back(bone.mModelTransform);
            restPositions.push_back(bone.mModelTransform.mTranslation);
        }
    }

    ReactionPoseTargets targets;
    targets.mSpine = static_cast<std::size_t>(findBone(walker.mBodyNames, "spine"));
    targets.mNeck = static_cast<std::size_t>(findBone(walker.mBodyNames, "neck"));
    targets.mLeftUpperArm = static_cast<std::size_t>(findBone(walker.mBodyNames, "left_upper_arm"));
    targets.mRightUpperArm = static_cast<std::size_t>(findBone(walker.mBodyNames, "right_upper_arm"));

    ReactionPipelineProfile reactionProfile;
    reactionProfile.mEnabled = true;
    reactionProfile.mBehavior.mEnabled = true;
    ReactionPipelineState reactionState;

    std::ostringstream json;
    json << "{\n";
    json << "\"bodyNames\":[";
    for (std::size_t i = 0; i < walker.mBodyNames.size(); ++i)
        json << (i ? "," : "") << "\"" << walker.mBodyNames[i] << "\"";
    json << "],\n\"parents\":[";
    for (std::size_t i = 0; i < parents.size(); ++i)
        json << (i ? "," : "") << parents[i];
    json << "],\n";

    json << "\"walker\":{\"dt\":0.0167,\"frames\":[";
    bool first = true;
    for (std::size_t f = 0; f < walker.mFrames.size(); f += 2)
    {
        const ScenarioFrame& frame = walker.mFrames[f];
        if (frame.mBodies.size() != walker.mBodyNames.size())
            continue;
        json << (first ? "" : ",") << "\n{\"t\":" << number(frame.mTime)
             << ",\"stage\":" << static_cast<int>(frame.mActuationStage)
             << ",\"left\":" << (frame.mSupport.mLeftFootSupported ? 1 : 0)
             << ",\"right\":" << (frame.mSupport.mRightFootSupported ? 1 : 0)
             << ",\"speed\":" << number(frame.mRequestedHorizontalVelocity.Length())
             << ",\"bodies\":[";
        for (std::size_t b = 0; b < frame.mBodies.size(); ++b)
            json << (b ? "," : "") << point(frame.mBodies[b].mWorldPosition);
        json << "]";
        if (frame.mMomentum)
            json << ",\"com\":" << point(frame.mMomentum->mCenterOfMass);
        else
            json << ",\"com\":[0,0,0]";
        json << "}";
        first = false;
    }
    json << "]},\n";

    json << "\"reactions\":{\"dt\":0.0167,\"targets\":{\"spine\":" << static_cast<long long>(targets.mSpine)
         << ",\"neck\":" << static_cast<long long>(targets.mNeck)
         << ",\"leftUpperArm\":" << static_cast<long long>(targets.mLeftUpperArm)
         << ",\"rightUpperArm\":" << static_cast<long long>(targets.mRightUpperArm) << "},\"events\":[";
    json << "{\"t\":1.0,\"kind\":\"head\"},{\"t\":2.33,\"kind\":\"left_forearm\"},{\"t\":3.0,\"kind\":\"left_calf\"}],";
    json << "\"frames\":[";
    first = true;
    constexpr int reactionFrames = 240;
    for (int i = 0; i < reactionFrames; ++i)
    {
        const std::uint64_t step = static_cast<std::uint64_t>(i + 1);
        ReactionContractInput input = baselineInput(step);
        if (i == 60)
            addHit(input, 1, AnatomicalRegion::Head, 2500.0f, false, 0.0f, 0.0f);
        if (i == 140)
            addHit(input, 2, AnatomicalRegion::LeftForearm, 900.0f, true, 3.0f, 0.55f);
        if (i == 180)
            addHit(input, 3, AnatomicalRegion::LeftCalf, 900.0f, true, 4.0f, 0.60f);
        if (i >= 220)
        {
            input.mMode = ReactionBodyMode::Recovery;
            input.mOwned.mRecoveryBrace = true;
            input.mContact = ReactionContact{};
            input.mImpulse = ReactionImpulse{};
        }
        const ReactionOutputs output = stepReactionPipeline(reactionState, input, reactionProfile);
        std::vector<Transform> composed = rest;
        applyReactionPose(rest, output, targets, composed);
        const std::vector<JPH::Vec3> world = forwardKinematics(composed, restPositions, parents);
        json << (first ? "" : ",") << "\n{\"t\":" << number(static_cast<float>(i) / 60.0f)
             << ",\"behavior\":\"" << behaviorName(output.mBehavior)
             << "\",\"phase\":" << static_cast<int>(output.mPhase)
             << ",\"authority\":" << number(output.mPoseCorrectionAuthority)
             << ",\"stability\":" << number(output.mStabilizationStrength)
             << ",\"speed\":" << number(output.mSpeedScale)
             << ",\"intensity\":" << number(output.mReactionIntensity)
             << ",\"cover\":" << (output.mCoverFace ? 1 : 0)
             << ",\"cradleL\":" << (output.mCradleLeft ? 1 : 0)
             << ",\"cradleR\":" << (output.mCradleRight ? 1 : 0)
             << ",\"limpL\":" << (output.mLimpLeft ? 1 : 0)
             << ",\"limpR\":" << (output.mLimpRight ? 1 : 0)
             << ",\"bodies\":[";
        for (std::size_t b = 0; b < world.size(); ++b)
            json << (b ? "," : "") << point(world[b]);
        json << "]}";
        first = false;
    }
    json << "]}\n}\n";

    std::ofstream stream(outputPath, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(stream.is_open()) << "cannot write " << outputPath;
    stream << json.str();
    stream.close();
    SUCCEED() << "viewer trace written to " << outputPath;
}
