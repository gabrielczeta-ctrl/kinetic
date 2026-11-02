#include <ragdollcore/scenario.hpp>

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
// clang-format on

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>

namespace
{
    using namespace RagdollCore;
    using Action = std::array<float, 6>;
    struct Episode
    {
        float speed = 60, pushX = 0, pushY = 0;
        int pushStep = -1, releaseStep = -1;
    };

    ScenarioScript scriptFor(const Episode& episode)
    {
        ScenarioScript script;
        script.mDurationSeconds = 7;
        ScenarioInputKeyframe walk;
        walk.mStep = 30;
        walk.mRequestedHorizontalVelocity = JPH::Vec3(0, episode.speed, 0);
        script.mKeyframes.push_back(walk);
        if (episode.releaseStep >= 0)
        {
            auto release = walk;
            release.mStep = static_cast<std::uint64_t>(episode.releaseStep);
            release.mMotorsEnabled = false;
            script.mKeyframes.push_back(release);
        }
        if (episode.pushStep >= 0)
            script.mImpulses.push_back({ static_cast<std::uint64_t>(episode.pushStep), 3,
                JPH::Vec3(episode.pushX, episode.pushY, 0), std::nullopt });
        return script;
    }

    RagdollPose basePose()
    {
        RagdollPose pose;
        for (const auto& bone : makeScenarioHumanoidSkeleton().mBones)
            pose.mBodyTransforms.push_back(bone.mModelTransform);
        return pose;
    }

    std::optional<ScenarioInputKeyframe> keyFor(std::uint64_t step, const Episode& episode,
        const Action& action, const RagdollPose& base)
    {
        for (float value : action)
            if (!std::isfinite(value) || std::abs(value) > 1)
                throw std::runtime_error("non-finite or out-of-range action");
        if (step < 30)
            return std::nullopt;
        ScenarioInputKeyframe key;
        key.mStep = step;
        key.mRequestedHorizontalVelocity = JPH::Vec3(0, episode.speed, 0);
        key.mMotorsEnabled = episode.releaseStep < 0 || step < static_cast<std::uint64_t>(episode.releaseStep);
        key.mMotorTarget = base;
        auto& pose = key.mMotorTarget->mBodyTransforms;
        pose[15].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), .25f * action[0]) * base.mBodyTransforms[15].mRotation;
        pose[18].mRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), .25f * action[1]) * base.mBodyTransforms[18].mRotation;
        for (const auto [body, first, scale] : { std::tuple{ 1u, 2u, .15f }, std::tuple{ 0u, 4u, .10f } })
            pose[body].mRotation = (JPH::Quat::sRotation(JPH::Vec3::sAxisX(), scale * action[first])
                * JPH::Quat::sRotation(JPH::Vec3::sAxisY(), scale * action[first + 1])
                * base.mBodyTransforms[body].mRotation).Normalized();
        return key;
    }

    ScenarioResult simulate(const ScenarioScript& script, const ScenarioInputProvider& provider = {})
    {
        ScenarioSettings settings;
        settings.mLocomotionController = ScenarioLocomotionController::Reference;
        auto result = runRagdollScenario(makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), settings, script, provider);
        if (!result.completed())
            throw std::runtime_error(result.mErrors.empty() ? "scenario incomplete" : result.mErrors.front());
        return result;
    }

    bool supported(const ScenarioFrame& frame, std::size_t body)
    {
        return std::any_of(frame.mContacts.begin(), frame.mContacts.end(),
            [body](const auto& contact) { return contact.mBodyIndex == body && contact.hasSupport(); });
    }

    std::vector<float> observation(const ScenarioFrame* frame, float speed)
    {
        if (!frame)
        {
            std::vector<float> values(57, 0);
            values.back() = speed / 60;
            return values;
        }
        std::vector<float> values;
        const auto vector = [&](JPH::Vec3Arg v) { values.insert(values.end(), { v.GetX(), v.GetY(), v.GetZ() }); };
        const auto quaternion = [&](JPH::QuatArg rotation) {
            const auto q = rotation.EnsureWPositive();
            values.insert(values.end(), { q.GetX(), q.GetY(), q.GetZ(), q.GetW() });
        };
        vector(frame->mCenterOfMassVelocity / 60);
        vector(frame->mBodies[0].mWorldRotation * JPH::Vec3::sAxisZ());
        vector(frame->mBodies[0].mAngularVelocity / 5);
        values.push_back(static_cast<float>(frame->mBodies[0].mWorldPosition.GetZ()) / 66 - 1);
        for (const auto foot : { 16u, 19u })
        {
            vector(JPH::Vec3(frame->mBodies[foot].mWorldPosition - frame->mCenterOfMass) / 66);
            vector(frame->mBodies[foot].mLinearVelocity / 60);
            values.push_back(supported(*frame, foot) ? 1.0f : 0.0f);
        }
        for (const auto [child, parent] : { std::pair{ 14u, 0u }, { 15u, 14u }, { 16u, 15u },
                 { 17u, 0u }, { 18u, 17u }, { 19u, 18u }, { 1u, 0u } })
            quaternion((frame->mBodies[parent].mWorldRotation.Conjugated() * frame->mBodies[child].mWorldRotation).Normalized());
        for (const auto phase : { GaitPhase::Idle, GaitPhase::LeftStance, GaitPhase::RightStance })
            values.push_back(frame->mGait.mPhase == phase ? 1.0f : 0.0f);
        values.push_back(frame->mReferenceGait ? frame->mReferenceGait->mTimeInStage / .35f : 0);
        values.push_back(speed / 60);
        if (values.size() != 57 || std::any_of(values.begin(), values.end(), [](float x) { return !std::isfinite(x); }))
            throw std::runtime_error("invalid observation");
        return values;
    }

    float reward(const ScenarioFrame* frame, const ScenarioFrame* before, float speed)
    {
        if (!frame || frame->mStep < 30)
            return 0;
        if (frame->mController.mMode == ActiveControllerMode::Falling || frame->mController.mMode == ActiveControllerMode::Fallen)
            return -3;
        const float error = (frame->mCenterOfMassVelocity.GetY() - speed) / speed;
        const float lateral = frame->mCenterOfMassVelocity.GetX() / 60;
        const float up = std::max(0.0f, frame->mPelvisUpDot);
        float value = .2f + std::exp(-2.5f * error * error) * up * up - .25f * lateral * lateral
            - .25f * (1 - up) * (1 - up);
        float peak = 0;
        for (float utilization : frame->mJointSupportMotorUtilizations)
            peak = std::max(peak, utilization);
        if (peak >= .999f) value -= .5f;
        if (std::none_of(frame->mContacts.begin(), frame->mContacts.end(),
                [](const auto& contact) { return contact.hasSupport(); })) value -= 2;
        if (frame->mSolverImpulse && (frame->mSolverImpulse->mFreeVelocityLimitTouched || frame->mSolverImpulse->mVelocityLimitTouched))
            value -= 4;
        if (before)
            for (const auto foot : { 16u, 19u })
                if (supported(*frame, foot) && supported(*before, foot))
                {
                    auto delta = frame->mBodies[foot].mWorldPosition - before->mBodies[foot].mWorldPosition;
                    delta.SetZ(0);
                    value -= .25f * static_cast<float>(std::min(10.0, delta.LengthSq() / 2.25));
                }
        return value;
    }

    void summary(const ScenarioResult& result)
    {
        const auto m = summarizeScenarioInvariants(result);
        int falling = -1;
        std::size_t transfers = 0, saturated = 0, reference = 0;
        for (const auto& frame : result.mFrames)
        {
            if (falling < 0 && (frame.mController.mMode == ActiveControllerMode::Falling || frame.mController.mMode == ActiveControllerMode::Fallen))
                falling = static_cast<int>(frame.mStep);
            if (frame.mReferenceGait)
            {
                ++reference;
                saturated += frame.mReferenceGait->mMaxMotorUtilization >= .999f;
                transfers += frame.mGait.hasStance() && frame.mGait.mTransitioned;
            }
        }
        std::cout << "{\"steps\":" << result.mFrames.size() << ",\"hash\":\"" << hashScenarioTelemetry(result)
                  << "\",\"fall_step\":" << falling << ",\"transfers\":" << transfers << ",\"saturated\":" << saturated
                  << ",\"reference_frames\":" << reference << ",\"slide\":" << m.mMaxStanceFootSlide
                  << ",\"airborne\":" << m.mAirborneFrames << ",\"caps\":" << m.mFreeVelocityLimitedFrames + m.mSolverVelocityLimitedFrames
                  << ",\"final_speed\":" << m.mFinalHalfSecondAchievedSpeed << ",\"mean_speed\":" << m.mMeanAchievedSpeed
                  << ",\"net_force\":" << m.mMaxAppliedNetForce << ",\"net_moment\":" << m.mMaxNetActuationMoment << "}";
    }

    void selfTest()
    {
        const Episode episode;
        const auto script = scriptFor(episode);
        const auto base = basePose();
        const auto baseline = simulate(script);
        const auto zero = simulate(script, [&](std::uint64_t step, const ScenarioFrame*) { return keyFor(step, episode, {}, base); });
        if (!haveIdenticalScenarioTelemetry(baseline, zero)) throw std::runtime_error("zero action changed baseline");
        auto releaseEpisode = episode;
        releaseEpisode.releaseStep = 80;
        auto recorded = scriptFor(releaseEpisode);
        const auto feedbackScript = recorded;
        const auto controlled = simulate(feedbackScript, [&](std::uint64_t step, const ScenarioFrame*) {
            const auto key = keyFor(step, releaseEpisode, { .2f, -.1f, .1f, -.2f, .1f, 0 }, base);
            if (key)
            {
                auto at = std::lower_bound(recorded.mKeyframes.begin(), recorded.mKeyframes.end(), step,
                    [](const auto& k, auto value) { return k.mStep < value; });
                if (at != recorded.mKeyframes.end() && at->mStep == step) *at = *key;
                else recorded.mKeyframes.insert(at, *key);
            }
            return key;
        });
        const auto replay = simulate(recorded);
        if (!haveIdenticalScenarioTelemetry(controlled, replay) || !haveIdenticalScenarioTelemetry(replay, simulate(recorded)))
            throw std::runtime_error("action replay differs");
        for (const auto& frame : controlled.mFrames)
            if (frame.mStep >= 80 && frame.mMotorsApplied) throw std::runtime_error("action overrode motor release");
        if (controlled.mFrames[40].mBodies[15].mWorldRotation.IsClose(baseline.mFrames[40].mBodies[15].mWorldRotation, 1e-8f))
            throw std::runtime_error("nonzero actions had no measured effect");
        std::cout << "SELF_TEST "; summary(baseline); std::cout << std::endl;
    }

    void worker()
    {
        const auto base = basePose();
        std::string line;
        std::cout << "READY 57 6" << std::endl;
        while (std::getline(std::cin, line) && line != "QUIT")
        {
            Episode episode;
            std::string command, extra;
            std::istringstream input(line);
            if (!(input >> command >> episode.speed >> episode.pushX >> episode.pushY >> episode.pushStep >> episode.releaseStep)
                || command != "EP" || input >> extra || !std::isfinite(episode.speed) || episode.speed <= 0 || episode.speed > 120
                || !std::isfinite(episode.pushX) || !std::isfinite(episode.pushY)
                || std::abs(episode.pushX) > 2400 || std::abs(episode.pushY) > 2400
                || episode.pushStep < -1 || episode.pushStep >= 420
                || (episode.releaseStep != -1 && (episode.releaseStep <= 30 || episode.releaseStep >= 420)))
                throw std::runtime_error("invalid episode request");
            const ScenarioFrame* before = nullptr;
            const auto result = simulate(scriptFor(episode), [&](std::uint64_t step, const ScenarioFrame* previous) {
                std::cout << "STEP " << step << ' ' << reward(previous, before, episode.speed);
                for (float value : observation(previous, episode.speed)) std::cout << ' ' << value;
                std::cout << std::endl;
                before = previous;
                if (!std::getline(std::cin, line)) throw std::runtime_error("action input closed");
                std::istringstream actionInput(line);
                Action action;
                for (float& value : action) if (!(actionInput >> value)) throw std::runtime_error("missing action");
                if (actionInput >> extra) throw std::runtime_error("extra action fields");
                return keyFor(step, episode, action, base);
            });
            std::cout << "END " << reward(&result.mFrames.back(), &result.mFrames[result.mFrames.size() - 2], episode.speed) << ' ';
            summary(result); std::cout << std::endl;
        }
    }
}

int runFeedbackPilot(int argc, char** argv)
{
    std::cout << std::setprecision(9);
    int status = 0;
    try
    {
        if (argc == 2 && std::string(argv[1]) == "--self-test") selfTest();
        else if (argc == 2 && std::string(argv[1]) == "--baseline-csv")
            std::cout << RagdollCore::formatScenarioTraceCsv(simulate(scriptFor({})));
        else if (argc == 1) worker();
        else throw std::runtime_error("usage: ragdollcore-tests --feedback-pilot [--self-test|--baseline-csv]");
    }
    catch (const std::exception& e) { std::cerr << e.what() << std::endl; status = 2; }
    return status;
}
