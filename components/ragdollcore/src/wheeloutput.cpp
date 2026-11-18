#include <ragdollcore/wheeloutput.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace RagdollCore
{
    void clearFirstPersonView(ChamberPose& pose, WheelStage stage, float wind)
    {
        constexpr float clear = 12.0f * std::numbers::pi_v<float> / 180.0f;
        const float share = stage == WheelStage::Guard || stage == WheelStage::Recover ? 1.0f
            : stage == WheelStage::Wind ? std::clamp(1.0f - 2.0f * wind, 0.0f, 1.0f)
                                         : 0.0f;
        if (share <= 0.0f)
            return;
        const JPH::Quat turn = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), -clear * share);
        const JPH::Quat lean = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), 1.5f * clear * share);
        pose.mHand = turn * pose.mHand;
        pose.mRotation = (lean * turn * pose.mRotation).Normalized();
    }

    float keepHaftOutOfBody(ChamberPose& pose, const JPH::Vec3& bladeAxis, float butt)
    {
        constexpr float radius = 20.0f, top = 10.0f, bottom = -80.0f, forward = 6.0f;
        if (!(butt > 15.0f) || !std::isfinite(butt))
            return 0.0f;
        const JPH::Vec3 axisTop(0.0f, forward, top), axisBottom(0.0f, forward, bottom);
        const JPH::Vec3 hand = pose.mHand;
        float turned = 0.0f;
        for (int pass = 0; pass < 3; ++pass)
        {
            const JPH::Vec3 blade = (pose.mRotation * bladeAxis).NormalizedOr(JPH::Vec3::sAxisY());
            const JPH::Vec3 end = hand - blade * butt;
            // The closest points of the haft (hand -> butt) and the axis.
            const JPH::Vec3 d1 = end - hand, d2 = axisBottom - axisTop, r = hand - axisTop;
            const float a = d1.Dot(d1), e = d2.Dot(d2), f = d2.Dot(r), c = d1.Dot(r), b = d1.Dot(d2);
            const float denominator = a * e - b * b;
            float s = denominator > 1e-4f ? std::clamp((b * f - c * e) / denominator, 0.0f, 1.0f) : 0.0f;
            const float t = std::clamp((b * s + f) / e, 0.0f, 1.0f);
            s = std::clamp((b * t - c) / a, 0.0f, 1.0f);
            const JPH::Vec3 onHaft = hand + d1 * s, onAxis = axisTop + d2 * t;
            JPH::Vec3 away = onHaft - onAxis;
            away.SetZ(0.0f);
            const float distance = away.Length();
            if (distance >= radius || s < 0.05f)
                break;
            away = distance > 1e-3f ? away / distance : JPH::Vec3::sAxisX();
            // Out so the crossing clears; the nearer the hand, the more turn.
            const float push = (radius - distance + 1.0f) / std::max(s, 0.25f);
            const JPH::Vec3 newBlade = (hand - (end + away * push)).NormalizedOr(blade);
            if (!std::isfinite(newBlade.GetX()))
                break;
            pose.mRotation = (JPH::Quat::sFromTo(blade, newBlade) * pose.mRotation).Normalized();
            turned += std::acos(std::clamp(blade.Dot(newBlade), -1.0f, 1.0f));
        }
        return turned;
    }

    float haftGripTarget(const HaftGripSettings& settings, const JPH::Vec3& primary, const JPH::Vec3& authored,
        const JPH::Vec3& shoulder, float reach, float butt)
    {
        const JPH::Vec3 along = authored - primary;
        const float authoredSpacing = along.Length();
        if (!(settings.mShare > 0.0f) || authoredSpacing < 1e-3f || !std::isfinite(authoredSpacing))
            return authoredSpacing;
        const JPH::Vec3 axis = along / authoredSpacing;
        // The haft's point nearest the shoulder, and how far off the haft it is.
        const float nearest = (shoulder - primary).Dot(axis);
        const float offHaft = (shoulder - (primary + axis * nearest)).Length();
        float wanted = authoredSpacing + std::min(settings.mShare, 1.0f) * (nearest - authoredSpacing);
        // Within the arm's reach of the shoulder: toward the nearest point.
        if (reach > 0.0f && std::isfinite(reach))
        {
            const float span = offHaft < reach ? std::sqrt(reach * reach - offHaft * offHaft) : 0.0f;
            wanted = std::clamp(wanted, nearest - span, nearest + span);
        }
        const float most = std::max(settings.mMinimum, butt);
        return std::isfinite(wanted) ? std::clamp(wanted, settings.mMinimum, most) : authoredSpacing;
    }

    float haftGripStep(float current, float target, float speed, float dt)
    {
        const float most = std::max(0.0f, speed) * std::max(0.0f, dt);
        return current + std::clamp(target - current, -most, most);
    }

    JPH::Vec3 pivotBetweenHands(
        ChamberPose& pose, const JPH::Quat& reference, const JPH::Vec3& bladeAxis, float support, float share)
    {
        if (!(share > 0.0f) || !std::isfinite(support) || std::abs(support) < 1e-3f)
            return JPH::Vec3::sZero();
        const JPH::Vec3 shift
            = (reference * bladeAxis - pose.mRotation * bladeAxis) * (0.5f * support * std::min(share, 1.0f));
        if (!std::isfinite(shift.GetX()) || !std::isfinite(shift.GetY()) || !std::isfinite(shift.GetZ()))
            return JPH::Vec3::sZero();
        pose.mHand += shift;
        return shift;
    }

    JPH::Vec3 NpcGuardHold::step(bool resting, float dt, JPH::RVec3Arg actorPosition, float actorYaw,
        JPH::RVec3& target, JPH::Quat& rotation)
    {
        constexpr float seconds = 0.35f, reseat = 45.0f;
        if (!resting || !(dt > 0.0f))
        {
            mLocal.reset();
            return JPH::Vec3::sZero();
        }
        const JPH::Quat rootTurn = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), -actorYaw);
        const JPH::Quat toLocal = rootTurn.Conjugated();
        const JPH::Vec3 wantedLocal = toLocal * JPH::Vec3(target - actorPosition);
        const JPH::Quat turnLocal = (toLocal * rotation).Normalized();
        if (!mLocal || (wantedLocal - *mLocal).Length() > reseat)
        {
            mLocal = wantedLocal;
            mTurn = turnLocal;
        }
        const float k = std::min(1.0f, dt / seconds);
        *mLocal += (wantedLocal - *mLocal) * k;
        mTurn = mTurn.SLERP(turnLocal, k).Normalized(); // Jolt takes the short way
        const JPH::RVec3 held = actorPosition + JPH::RVec3(rootTurn * *mLocal);
        const JPH::Vec3 shift(held - target);
        target = held;
        rotation = (rootTurn * mTurn).Normalized();
        return shift;
    }
}
