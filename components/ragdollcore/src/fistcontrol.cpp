#include <ragdollcore/fistcontrol.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace RagdollCore
{
    namespace
    {
        constexpr float sPi = std::numbers::pi_v<float>;

        float smooth(float t)
        {
            t = std::clamp(t, 0.0f, 1.0f);
            return t * t * (3.0f - 2.0f * t);
        }

        // Hand, pole and turn along keys by s (0..1), eased within each span.
        void along(const std::vector<ChamberKey>& keys, float s, FistPunchPose& pose)
        {
            if (keys.empty())
                return;
            if (keys.size() == 1 || s <= keys.front().mS)
            {
                pose.mHand = keys.front().mHand;
                pose.mPole = keys.front().mPole;
                pose.mTwistDegrees = keys.front().mTwistDegrees;
                return;
            }
            std::size_t i = 0;
            while (i + 2 < keys.size() && s > keys[i + 1].mS)
                ++i;
            const ChamberKey& a = keys[i];
            const ChamberKey& b = keys[i + 1];
            const float t = smooth((s - a.mS) / std::max(1e-4f, b.mS - a.mS));
            pose.mHand = a.mHand + (b.mHand - a.mHand) * t;
            pose.mPole = (a.mPole + (b.mPole - a.mPole) * t).NormalizedOr(a.mPole);
            pose.mTwistDegrees = a.mTwistDegrees + (b.mTwistDegrees - a.mTwistDegrees) * t;
        }
    }

    std::optional<FlickPunch> classifyFlick(
        float right, float up, bool crouched, FistHand nextStraight, const FlickSettings& settings)
    {
        const float length = std::hypot(right, up);
        if (!(length >= settings.mPick) || !std::isfinite(length))
            return std::nullopt;
        FlickPunch punch;
        punch.mPower = std::clamp((length - settings.mPick) / std::max(1.0f, settings.mFull - settings.mPick), 0.0f, 1.0f);
        // Eight sectors of 45 degrees, 0 = right, counter-clockwise.
        float angle = std::atan2(up, right) * (180.0f / sPi);
        if (angle < 0.0f)
            angle += 360.0f;
        const int sector = static_cast<int>(std::floor((angle + 22.5f) / 45.0f)) % 8;
        switch (sector)
        {
            case 0: // right: the left hook, thrown across from the left
                punch = { FistHand::Left, PunchKind::Hook, punch.mPower };
                break;
            case 1: // up-right: the right cross
                punch = { FistHand::Right, PunchKind::Straight, punch.mPower };
                break;
            case 2: // up: the next straight
                punch = { nextStraight, PunchKind::Straight, punch.mPower };
                break;
            case 3: // up-left: the left jab
                punch = { FistHand::Left, PunchKind::Straight, punch.mPower };
                break;
            case 4: // left: the right hook
                punch = { FistHand::Right, PunchKind::Hook, punch.mPower };
                break;
            case 5: // down-left: the left uppercut
                punch = { FistHand::Left, PunchKind::Uppercut, punch.mPower };
                break;
            case 6: // down: the next body shot
                punch = { nextStraight, PunchKind::Body, punch.mPower };
                break;
            default: // down-right: the right uppercut
                punch = { FistHand::Right, PunchKind::Uppercut, punch.mPower };
                break;
        }
        if (crouched && punch.mKind == PunchKind::Straight)
            punch.mKind = PunchKind::Body;
        return punch;
    }

    JPH::Vec3 aimFistTarget(JPH::Vec3 target, JPH::Vec3 rayOrigin, JPH::Vec3 rayDirection,
        float reach, float weight)
    {
        const auto finite = [](JPH::Vec3 v) {
            return std::isfinite(v.GetX()) && std::isfinite(v.GetY()) && std::isfinite(v.GetZ());
        };
        if (!finite(target) || !finite(rayOrigin) || !finite(rayDirection)
            || !std::isfinite(reach) || reach <= 0.0f || !std::isfinite(weight) || weight <= 0.0f
            || rayDirection.LengthSq() < 1e-6f)
            return target;
        const float radius = std::min(target.Length(), reach);
        if (radius < 1e-3f)
            return target;
        target = target.Normalized() * radius;
        const JPH::Vec3 ray = rayDirection.Normalized();
        const float along = -rayOrigin.Dot(ray);
        const JPH::Vec3 nearest = rayOrigin + ray * std::max(0.0f, along);
        const float discriminant = radius * radius - (rayOrigin + ray * along).LengthSq();
        JPH::Vec3 wanted = nearest.NormalizedOr(target.Normalized()) * radius;
        if (discriminant >= 0.0f)
        {
            const float far = along + std::sqrt(discriminant);
            if (far >= 0.0f)
                wanted = rayOrigin + ray * far;
        }
        return target + (wanted - target) * std::clamp(weight, 0.0f, 1.0f);
    }

    JPH::Vec3 easeFistToHold(JPH::Vec3 from, JPH::Vec3 hold, float elapsed, float seconds)
    {
        if (!(seconds > 0.0f) || !std::isfinite(elapsed) || elapsed >= seconds || from.IsNaN())
            return hold;
        return from + (hold - from) * smooth(elapsed / seconds);
    }

    std::string_view fistLineName(PunchKind kind)
    {
        switch (kind)
        {
            case PunchKind::Hook:
                return "hook";
            case PunchKind::Uppercut:
                return "uppercut";
            case PunchKind::Body:
                return "body";
            default:
                return "cross";
        }
    }

    const AttackLine* fistLine(PunchKind kind)
    {
        const std::string_view name = fistLineName(kind);
        for (const AttackLine& line : fistLines())
            if (line.mName == name)
                return &line;
        return nullptr;
    }

    ChamberKey mirroredKey(const ChamberKey& key)
    {
        ChamberKey result = key;
        const auto flip = [](JPH::Vec3 v) { return JPH::Vec3(-v.GetX(), v.GetY(), v.GetZ()); };
        result.mHand = flip(key.mHand);
        result.mBlade = flip(key.mBlade);
        result.mPole = flip(key.mPole);
        result.mEdge = flip(key.mEdge);
        result.mTwistDegrees = -key.mTwistDegrees;
        return result;
    }

    AttackLine mirroredLine(const AttackLine& line)
    {
        AttackLine result = line;
        result.mDragRight = -line.mDragRight;
        for (ChamberKey& key : result.mWind)
            key = mirroredKey(key);
        for (ChamberKey& key : result.mCut)
            key = mirroredKey(key);
        return result;
    }

    PunchTiming punchTiming(PunchKind kind, float power, float control)
    {
        PunchTiming timing;
        switch (kind)
        {
            case PunchKind::Hook:
                timing = { 0.14f, 0.16f, 0.30f, 1.2f };
                break;
            case PunchKind::Uppercut:
                timing = { 0.14f, 0.14f, 0.30f, 1.2f };
                break;
            default: // straights and body shots
                timing = { 0.10f, 0.12f, 0.25f, 1.0f };
                break;
        }
        const float p = std::isfinite(power) ? std::clamp(power, 0.0f, 1.0f) : 0.0f;
        const float slow = 1.0f + 0.3f * (1.0f - (std::isfinite(control) ? std::clamp(control, 0.0f, 1.0f) : 1.0f));
        timing.mWind *= (1.0f + 1.5f * p) * slow;
        timing.mCut *= (1.0f - 0.25f * p) * slow;
        timing.mBack *= (1.0f + 0.4f * p) * slow;
        timing.mFatigue *= 1.0f + 2.0f * p;
        return timing;
    }

    bool woundPunchPaced(PunchKind kind)
    {
        return kind == PunchKind::Hook || kind == PunchKind::Uppercut;
    }

    float woundStrikeSeconds(float cut, PunchKind kind, float seconds)
    {
        return woundPunchPaced(kind) && seconds > 0.0f ? std::max(cut, seconds) : cut;
    }

    float woundWindShare(PunchKind kind, float share)
    {
        return woundPunchPaced(kind) && share > 0.0f ? share : 0.5f;
    }

    FistPunchPose fistPunchPose(PunchKind kind, FistHand hand, const PunchTiming& timing, float time)
    {
        const AttackLine* right = fistLine(kind);
        if (!right)
        {
            FistPunchPose pose;
            pose.mDone = true;
            return pose;
        }
        return fistLinePunchPose(*right, hand, timing, time);
    }

    FistPunchPose fistLinePunchPose(const AttackLine& rightLine, FistHand hand, const PunchTiming& timing, float time,
        const JPH::Vec3* from)
    {
        FistPunchPose pose;
        const AttackLine line = hand == FistHand::Left ? mirroredLine(rightLine) : rightLine;
        const float wind = std::max(1e-3f, timing.mWind), cut = std::max(1e-3f, timing.mCut),
                    back = std::max(1e-3f, timing.mBack);
        if (time < wind)
        {
            along(line.mWind, time / wind, pose);
            if (from)
            {
                // From the held fist's place to the line's chamber.
                FistPunchPose chamber;
                along(line.mWind, 1.0f, chamber);
                const float t = smooth(time / wind);
                pose.mHand = *from + (chamber.mHand - *from) * t;
                pose.mPole = chamber.mPole;
                pose.mTwistDegrees = chamber.mTwistDegrees;
            }
        }
        else if (time < wind + cut)
        {
            along(line.mCut, (time - wind) / cut, pose);
            // The fist strikes from a third of the way through.
            pose.mStriking = (time - wind) / cut >= 0.3f;
        }
        else
        {
            FistPunchPose end, guard;
            along(line.mCut, 1.0f, end);
            along(line.mWind, 0.0f, guard);
            const float t = smooth((time - wind - cut) / back);
            pose.mHand = end.mHand + (guard.mHand - end.mHand) * t;
            pose.mPole = (end.mPole + (guard.mPole - end.mPole) * t).NormalizedOr(guard.mPole);
            pose.mTwistDegrees = end.mTwistDegrees + (guard.mTwistDegrees - end.mTwistDegrees) * t;
            // Still striking just past the end (a punch thrown through).
            pose.mStriking = time - wind - cut < 0.25f * back;
            pose.mDone = time >= wind + cut + back;
        }
        return pose;
    }
}
