#include <ragdollcore/wheelaim.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace RagdollCore
{
    namespace
    {
        constexpr float sDegree = std::numbers::pi_v<float> / 180.0f;
        // The game's values (play-2026-11-03-7).
        constexpr float sAssistFull = 20.0f * sDegree; // HSC_AIM_ASSIST
        constexpr float sAssistFade = 35.0f * sDegree;
        constexpr float sAssistClick = 1.0f;
        constexpr float sLowStance = 0.9f; // HSC_LOW[0]: the lowest aim
        constexpr float sLowStart = 0.2f; // HSC_LOW[3]
        constexpr float sDownAim = 1.3f; // HSC_DOWN_AIM
        constexpr float sAimReach = 35.0f; // HSC_AIM_REACH
        constexpr float sAimHold = 0.2f; // HSC_AIM_HOLD
        constexpr float sShortReachPast = 5.0f; // HSC_SHORT_REACH
        constexpr float sShortReachMost = 25.0f;
        constexpr float sThrustReach = 6.0f; // HSC_THRUST_LUNGE[2]
        constexpr float sThrustDrawBack = 0.35f; // HSC_THRUST_LUNGE[3]

        float smoothStep(float x)
        {
            x = std::clamp(x, 0.0f, 1.0f);
            return x * x * (3.0f - 2.0f * x);
        }

        void finish(WheelAim& aim, float angle)
        {
            const float ca = std::cos(aim.mAzimuth), sa = std::sin(aim.mAzimuth);
            const float ce = std::cos(aim.mElevation), se = std::sin(aim.mElevation);
            aim.mDirection = JPH::Vec3(sa * ce, ca * ce, se);
            // Spanned by the aim and the stroke (the wind-up's opposite).
            const float sx = -std::cos(angle), sy = -std::sin(angle);
            const JPH::Vec3 screenRight(ca, -sa, 0.0f);
            const JPH::Vec3 screenUp(-sa * se, -ca * se, ce);
            const JPH::Vec3 stroke = screenRight * sx + screenUp * sy;
            aim.mPlaneNormal = stroke.Cross(aim.mDirection).NormalizedOr(JPH::Vec3::sAxisX());
        }
    }

    WheelAim solveWheelAim(const WheelAimInput& input, float angle, float tip)
    {
        WheelAim aim;
        aim.mThrustness = std::clamp((-std::sin(angle) - 0.85f) / 0.15f, 0.0f, 1.0f);
        float elevation = -std::clamp(input.mLookPitch, -0.6f, 0.6f);
        aim.mLowest = std::min(sLowStance, 1.3f);
        aim.mFloor = input.mFoeDowned ? std::max(aim.mLowest, sDownAim) : aim.mLowest;
        float baseAzimuth = 0.0f;
        // The crosshair point: where the ray passes the foe's upright axis, or
        // one at the blade's sweet spot ahead of the chest.
        if (input.mPlayer && input.mRayOrigin && input.mRayDirection)
        {
            const JPH::Vec3 ray = input.mRayDirection->NormalizedOr(JPH::Vec3::sAxisY());
            const JPH::Vec3 up = JPH::Vec3::sAxisZ();
            const JPH::Vec3 flatRay = ray - up * ray.Dot(up);
            float sweet = 35.0f + 0.55f * tip;
            if (input.mPointReach > sweet)
                sweet += (input.mPointReach - sweet) * aim.mThrustness;
            const JPH::Vec3 axis = input.mFoe ? *input.mFoe : flatRay.NormalizedOr(JPH::Vec3::sAxisY()) * sweet;
            const JPH::Vec3 toAxis = axis - *input.mRayOrigin;
            float depth = toAxis.Dot(ray);
            if (flatRay.LengthSq() > 0.01f)
                depth = std::min((toAxis - up * toAxis.Dot(up)).Dot(flatRay) / flatRay.LengthSq(), 2.0f * toAxis.Length());
            const JPH::Vec3 local = *input.mRayOrigin + ray * std::max(depth, 30.0f);
            const float flat = std::hypot(local.GetX(), local.GetY());
            if (local.GetY() > 1.0f && flat > 1.0f)
            {
                const float azimuth = std::atan2(local.GetX(), local.GetY());
                const float pitch = std::atan2(local.GetZ(), flat);
                if (std::isfinite(azimuth) && std::isfinite(pitch))
                {
                    baseAzimuth = std::clamp(azimuth, -0.6f, 0.6f);
                    elevation = std::clamp(pitch, -aim.mFloor, 0.6f);
                    aim.mDistance = local.Length();
                }
            }
        }
        // The foe assist: a foe in reach near the aim takes it, its height
        // kept on the foe's body from the shins to the head.
        float azimuthOut = baseAzimuth;
        if (input.mFoe)
        {
            const JPH::Vec3 foe = *input.mFoe;
            const float flat = std::hypot(foe.GetX(), foe.GetY());
            aim.mLungeGap = flat - (input.mArmLength + 0.6f * tip + 12.0f);
            const float reach = input.mAssistReach > 0.0f ? input.mAssistReach : tip + 40.0f;
            const float azimuth = std::atan2(foe.GetX(), foe.GetY());
            if (flat > 1.0f && flat < reach)
            {
                aim.mFoeAssist = std::clamp(
                    (sAssistFade - std::abs(azimuth - baseAzimuth)) / (sAssistFade - sAssistFull), 0.0f, 1.0f);
                azimuthOut = baseAzimuth + (azimuth - baseAzimuth) * aim.mFoeAssist;
                const bool downed = aim.mFloor > aim.mLowest;
                const float low = std::atan2(foe.GetZ() - (downed ? 15.0f : 95.0f), flat);
                const float high = std::atan2(foe.GetZ() + (downed ? 15.0f : 25.0f), flat);
                // min(max()), not std::clamp: a foe whose head is below the floor
                // gives a band upside down, and the old clamp (undefined then)
                // returned its top in the game - kept.
                const float kept = std::min(std::max(elevation, std::max(low, -aim.mFloor)), std::min(high, 0.6f));
                elevation += (kept - elevation) * aim.mFoeAssist;
            }
            if (!std::isfinite(azimuthOut) || !std::isfinite(elevation) || !std::isfinite(aim.mFoeAssist))
            {
                azimuthOut = aim.mFoeAssist = 0.0f;
                elevation = -std::clamp(input.mLookPitch, -0.6f, 0.6f);
            }
        }
        aim.mAzimuth = azimuthOut;
        aim.mElevation = elevation;
        finish(aim, angle);
        return aim;
    }

    void steerWheelAim(WheelAim& aim, float angle, float steerX, float steerY, bool autoAttack)
    {
        aim.mAzimuth = std::clamp(aim.mAzimuth + steerX, -0.8f, 0.8f);
        aim.mElevation = std::clamp(aim.mElevation + steerY, -aim.mFloor, 0.6f);
        // A low aim takes the low stance where the plane carries the cut.
        const float span = std::max(0.05f, aim.mLowest - sLowStart);
        const bool planed = (!autoAttack || aim.mFoeAssist > 0.0f) && aim.mThrustness < 1.0f;
        aim.mLowWant = planed ? smoothStep((-aim.mElevation - sLowStart) / span) : 0.0f;
        finish(aim, angle);
    }

    float planeWeight(const WheelAim& aim, float angle, bool autoAttack, float u, bool asCut, bool asWind)
    {
        if (aim.mThrustness >= 1.0f || (autoAttack && aim.mFoeAssist <= 0.0f))
            return 0.0f;
        const float share = (1.0f - aim.mThrustness) * (autoAttack ? aim.mFoeAssist * sAssistClick : 1.0f);
        // Low chambers stay out of it; their cuts go in over the cut's first
        // 0.4, and an aimed foe keeps the plane longer.
        const float low = std::clamp((std::sin(angle) + 0.3f) / 0.6f, 0.0f, 1.0f);
        if (asCut && u >= 1.0f)
        {
            const float cu = u - 1.0f;
            const float fadeFrom = 0.65f + sAimHold * aim.mFoeAssist;
            return share * (low + (1.0f - low) * smoothStep(cu / 0.4f)) * (1.0f - smoothStep((cu - fadeFrom) / 0.25f));
        }
        if (asWind)
            return share * smoothStep((u - 0.5f) / 0.5f) * low;
        return 0.0f;
    }

    void putInPlane(ChamberPose& pose, const WheelAim& aim, float weight, const JPH::Vec3& bladeAxis, float tip)
    {
        if (weight <= 0.0f)
            return;
        const JPH::Vec3 normal = aim.mPlaneNormal;
        const JPH::Vec3 blade = (pose.mRotation * bladeAxis).NormalizedOr(JPH::Vec3::sAxisY());
        // The blade only where it already lies near the plane (one along its
        // normal flipped: FLOW-01).
        const float across = std::abs(normal.Dot(blade));
        const float bladeWeight = weight * smoothStep((0.8f - across) / 0.4f);
        const JPH::Vec3 planeBlade = (blade - normal * (normal.Dot(blade) * bladeWeight)).NormalizedOr(blade);
        pose.mRotation = (JPH::Quat::sFromTo(blade, planeBlade) * pose.mRotation).Normalized();
        pose.mHand -= normal * (normal.Dot(pose.mHand) * weight);
        if (aim.mDistance > 0.0f)
        {
            const JPH::Vec3 tipAt = pose.mHand + planeBlade * tip;
            const float along = tipAt.Dot(aim.mDirection);
            const float align = smoothStep((tipAt.NormalizedOr(JPH::Vec3::sAxisY()).Dot(aim.mDirection) - 0.5f) / 0.4f);
            const float out = std::clamp(aim.mDistance - along, 0.0f, sAimReach) * align * weight;
            if (std::isfinite(out))
                pose.mHand += aim.mDirection * out;
        }
    }

    void turnThrust(ChamberPose& pose, const WheelAim& aim, float share)
    {
        if (!(share > 0.0f) || !std::isfinite(aim.mAzimuth) || !std::isfinite(aim.mElevation))
            return;
        const JPH::Quat turn = (JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), -aim.mAzimuth * share)
            * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), aim.mElevation * share))
                                   .Normalized();
        pose.mHand = turn * pose.mHand;
        pose.mRotation = (turn * pose.mRotation).Normalized();
    }

    void reachShort(ChamberPose& pose, const WheelAim& aim, float shownU, float tip)
    {
        if (!(aim.mDistance > 0.0f) || !(aim.mThrustness > 0.0f))
            return;
        const float share = aim.mThrustness * smoothStep((shownU - 1.3f) / 0.5f);
        const float tipAlong = pose.mHand.Dot(aim.mDirection) + tip;
        const float out = std::clamp(aim.mDistance + sShortReachPast - tipAlong, 0.0f, sShortReachMost) * share;
        if (std::isfinite(out))
            pose.mHand += aim.mDirection * out;
    }

    float thrustDrive(const WheelAim& aim, bool player, float u, bool asCut)
    {
        if (aim.mThrustness <= 0.0f || !player)
            return 0.0f;
        if (asCut)
            return aim.mThrustness * smoothStep((u - 1.0f) / 0.5f);
        return -aim.mThrustness * sThrustDrawBack * smoothStep((u - 0.5f) / 0.5f);
    }

    void reachThrust(ChamberPose& pose, float drive, const JPH::Vec3& bladeAxis, bool shortStriker)
    {
        if (drive <= 0.0f || shortStriker)
            return;
        const JPH::Vec3 along = (pose.mRotation * bladeAxis).NormalizedOr(JPH::Vec3::sAxisY());
        pose.mHand += along * (sThrustReach * drive);
    }

    void pointThrust(ChamberPose& pose, const WheelAim& aim, float share, const JPH::Vec3& bladeAxis, float tip)
    {
        if (!(share > 0.0f) || !(tip > 1.0f))
            return;
        const JPH::Vec3 blade = (pose.mRotation * bladeAxis).NormalizedOr(JPH::Vec3::sAxisY());
        const float depth = (pose.mHand + blade * tip).Dot(aim.mDirection);
        if (!std::isfinite(depth) || depth < 0.5f * tip)
            return;
        const JPH::Vec3 wanted = (aim.mDirection * depth - pose.mHand).NormalizedOr(blade);
        JPH::Vec3 axis;
        float angle = 0.0f;
        JPH::Quat::sFromTo(blade, wanted).GetAxisAngle(axis, angle);
        angle = std::min(angle, 45.0f * sDegree) * std::min(share, 1.0f);
        if (!(angle > 1e-5f) || !std::isfinite(angle))
            return;
        pose.mRotation = (JPH::Quat::sRotation(axis, angle) * pose.mRotation).Normalized();
    }
}
