#include <ragdollcore/activebody.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <utility>

namespace RagdollCore
{
    namespace
    {
        bool finite(JPH::Vec3Arg v)
        {
            return std::isfinite(v.GetX()) && std::isfinite(v.GetY()) && std::isfinite(v.GetZ());
        }

        // Stable PD (Tan et al.): gains that act on the error predicted one
        // step ahead with implicit damping, so a stiff spring cannot ring at
        // the physics step. Returns per-unit-mass (stiffness, damping).
        std::pair<float, float> stableGains(float hz, float dampingRatio, float h)
        {
            const float w = 2.0f * JPH::JPH_PI * std::max(hz, 0.0f);
            const float k = w * w;
            const float d = 2.0f * dampingRatio * w;
            const float den = 1.0f + d * h + k * h * h;
            return { k / den, (d + k * h) / den };
        }

        JPH::Vec3 clampLength(JPH::Vec3Arg v, float cap, bool& saturated)
        {
            const float length = v.Length();
            if (length > cap && length > 0.0f)
            {
                saturated = true;
                return v * (cap / length);
            }
            return v;
        }
    }

    ActiveBodyOutput computeActiveBodyWrench(const ActiveBodySettings& s, const ActiveBodyInput& in)
    {
        ActiveBodyOutput out;
        const std::size_t count = in.mParts.size();
        out.mForces.assign(count, JPH::Vec3::sZero());
        out.mAngularAccelerations.assign(count, JPH::Vec3::sZero());
        const float g = in.mGravity.Length();
        const float h = in.mDuration;
        if (count == 0 || !(g > 0.0f) || !(h > 0.0f))
            return out;
        const JPH::Vec3 up = -in.mGravity / g;

        float totalMass = 0.0f, upperMass = 0.0f, torsoMass = 0.0f;
        JPH::Vec3 upperMomentum = JPH::Vec3::sZero();
        JPH::Vec3 torsoSpin = JPH::Vec3::sZero();
        std::size_t grounded = 0;
        for (const ActiveBodyPart& part : in.mParts)
        {
            totalMass += part.mMass;
            if (part.mUpper)
            {
                upperMass += part.mMass;
                upperMomentum += part.mVelocity * part.mMass;
            }
            if (part.mTorso)
            {
                torsoMass += part.mMass;
                torsoSpin += part.mAngularVelocity * part.mMass;
            }
            if (part.mFoot && part.mGrounded)
                ++grounded;
        }
        if (!(totalMass > 0.0f) || !(upperMass > 0.0f) || !(torsoMass > 0.0f))
            return out;
        // Damping reads the mass-weighted motion of the parts that receive the
        // wrench, never one light body (the pelvis is ~14% of the mass).
        const JPH::Vec3 velocity = upperMomentum / upperMass;
        const JPH::Vec3 angularVelocity = torsoSpin / torsoMass;
        const float strength = std::clamp(in.mStrength, 0.0f, 1.0f);
        const float support = std::clamp(in.mSupport, 0.0f, 1.0f);

        // Translation: whole-body acceleration toward the anchor.
        const JPH::Vec3 error = in.mTargetPosition - in.mRootPosition;
        const JPH::Vec3 velocityError = in.mTargetVelocity - velocity;
        const float errorUp = error.Dot(up);
        const float velocityErrorUp = velocityError.Dot(up);
        const auto [kh, dh] = stableGains(s.mHorizontalFrequency, s.mDampingRatio, h);
        const auto [kv, dv] = stableGains(s.mVerticalFrequency, s.mDampingRatio, h);
        bool saturated = false;
        const JPH::Vec3 horizontal = clampLength(
            (error - up * errorUp) * kh + (velocityError - up * velocityErrorUp) * dh,
            s.mHorizontalCap * g * strength, saturated) * std::clamp(in.mHorizontalScale, 0.0f, 1.0f);
        float vertical = errorUp * kv + velocityErrorUp * dv;
        const float upCap = s.mUpCap * std::max(1.0f, in.mUpCapScale) * g * strength * support;
        const float downCap = s.mDownCap * std::max(1.0f, in.mUpCapScale) * g * strength;
        if (vertical > upCap || vertical < -downCap)
        {
            vertical = std::clamp(vertical, -downCap, upCap);
            saturated = true;
        }
        // The upper parts carry the whole-body correction (scaled total/upper,
        // so the legs follow through the hips) plus their own weight.
        const JPH::Vec3 correction = (horizontal + up * vertical) * (totalMass / upperMass);
        const JPH::Vec3 weight = up * (g * s.mWeightSupport * support * strength);

        // Ground routing: this fraction of the upper-body force is taken back
        // out of the grounded feet. With no foot on the ground that fraction
        // does not exist at all - nothing holds a body in the air.
        // Air catch: a body with no grounded foot for longer than a stride's
        // flight gets its external support back.
        const float airCatch = s.mAirCatchFade > 0.0f
            ? std::clamp((in.mAirborneTime - s.mAirCatchDelay) / s.mAirCatchFade, 0.0f, 1.0f)
            : (in.mAirborneTime > s.mAirCatchDelay ? 1.0f : 0.0f);
        const float routingUp = std::clamp(s.mGroundRouting, 0.0f, 1.0f) * (1.0f - airCatch);
        const float routingSide = std::clamp(s.mGroundRoutingHorizontal, 0.0f, 1.0f) * (1.0f - airCatch);
        JPH::Vec3 upperTotal = JPH::Vec3::sZero();
        for (std::size_t i = 0; i < count; ++i)
        {
            const ActiveBodyPart& part = in.mParts[i];
            if (!part.mUpper)
                continue;
            JPH::Vec3 force = (correction + weight) * part.mMass;
            if (grounded == 0)
            {
                // In the air the routed share has nothing to push against.
                const float forceUp = force.Dot(up);
                force = (force - up * forceUp) * (1.0f - routingSide) + up * (forceUp * (1.0f - routingUp));
            }
            out.mForces[i] = force;
            upperTotal += force;
        }
        if (grounded > 0 && (routingUp > 0.0f || routingSide > 0.0f))
        {
            const float totalUp = upperTotal.Dot(up);
            const JPH::Vec3 routed = (upperTotal - up * totalUp) * routingSide + up * (totalUp * routingUp);
            const JPH::Vec3 reaction = -routed / static_cast<float>(grounded);
            for (std::size_t i = 0; i < count; ++i)
                if (in.mParts[i].mFoot && in.mParts[i].mGrounded)
                    out.mForces[i] += reaction;
        }
        for (const JPH::Vec3& f : out.mForces)
            out.mNetForce += f;

        // Rotation: separate yaw and tilt springs on the torso chain.
        JPH::Quat rotationError = in.mTargetRotation * in.mRootRotation.Conjugated();
        if (rotationError.GetW() < 0.0f)
            rotationError = -rotationError;
        JPH::Vec3 axis = JPH::Vec3::sAxisZ();
        float angle = 0.0f;
        rotationError.GetAxisAngle(axis, angle);
        if (!std::isfinite(angle) || angle < 1e-6f)
            angle = 0.0f;
        const JPH::Vec3 rotationVector = axis * angle;
        const JPH::Vec3 spinError = in.mTargetAngularVelocity - angularVelocity;
        const JPH::Vec3 yawError = up * rotationVector.Dot(up);
        const JPH::Vec3 yawSpinError = up * spinError.Dot(up);
        const auto [kr, dr] = stableGains(s.mYawFrequency, s.mDampingRatio, h);
        const auto [kt, dt] = stableGains(s.mTiltFrequency, s.mDampingRatio, h);
        const float angularScale = g / std::max(s.mGyrationRadius, 1.0f) * strength;
        const JPH::Vec3 yawAcceleration
            = clampLength(yawError * kr + yawSpinError * dr, s.mYawCap * angularScale, saturated);
        const JPH::Vec3 tiltAcceleration = clampLength(
            (rotationVector - yawError) * kt + (spinError - yawSpinError) * dt, s.mTiltCap * angularScale, saturated);
        const JPH::Vec3 angularAcceleration = yawAcceleration + tiltAcceleration;
        for (std::size_t i = 0; i < count; ++i)
            if (in.mParts[i].mTorso)
                out.mAngularAccelerations[i]
                    = in.mParts[i].mRoot ? yawAcceleration * s.mRootYawBoost + tiltAcceleration * s.mRootTiltBoost : angularAcceleration;

        out.mPositionError = error.Length();
        out.mAngleError = angle;
        out.mSaturated = saturated;
        out.mValid = finite(out.mNetForce) && finite(angularAcceleration);
        return out;
    }

    bool ActiveBodySettings::set(const std::string& key, float value)
    {
        if (!std::isfinite(value))
            return false;
        static const std::map<std::string, float ActiveBodySettings::*> fields = {
            { "horizontal_frequency", &ActiveBodySettings::mHorizontalFrequency },
            { "vertical_frequency", &ActiveBodySettings::mVerticalFrequency },
            { "yaw_frequency", &ActiveBodySettings::mYawFrequency },
            { "tilt_frequency", &ActiveBodySettings::mTiltFrequency },
            { "damping_ratio", &ActiveBodySettings::mDampingRatio },
            { "horizontal_cap", &ActiveBodySettings::mHorizontalCap },
            { "up_cap", &ActiveBodySettings::mUpCap },
            { "down_cap", &ActiveBodySettings::mDownCap },
            { "yaw_cap", &ActiveBodySettings::mYawCap },
            { "tilt_cap", &ActiveBodySettings::mTiltCap },
            { "gyration_radius", &ActiveBodySettings::mGyrationRadius },
            { "root_tilt_boost", &ActiveBodySettings::mRootTiltBoost },
            { "root_yaw_boost", &ActiveBodySettings::mRootYawBoost },
            { "capsule_acceleration", &ActiveBodySettings::mCapsuleAcceleration },
            { "max_yaw_rate", &ActiveBodySettings::mMaximumYawRate },
            { "rotation_constraint", &ActiveBodySettings::mRotationConstraint },
            { "anchor_frequency", &ActiveBodySettings::mAnchorFrequency },
            { "anchor_torque", &ActiveBodySettings::mAnchorTorque },
            { "weight_support", &ActiveBodySettings::mWeightSupport },
            { "ground_routing", &ActiveBodySettings::mGroundRouting },
            { "ground_routing_horizontal", &ActiveBodySettings::mGroundRoutingHorizontal },
            { "air_catch_delay", &ActiveBodySettings::mAirCatchDelay },
            { "air_catch_fade", &ActiveBodySettings::mAirCatchFade },
            { "hit_zero_at", &ActiveBodySettings::mHitZeroAt },
            { "hit_recovery_per_second", &ActiveBodySettings::mHitRecoveryPerSecond },
            { "hit_floor", &ActiveBodySettings::mHitFloor },
            { "topple_up_dot", &ActiveBodySettings::mToppleUpDot },
            { "shove_speed", &ActiveBodySettings::mShoveSpeed },
            { "stumble_seconds", &ActiveBodySettings::mStumbleSeconds },
            { "stumble_threshold", &ActiveBodySettings::mStumbleThreshold },
            { "get_up_strength", &ActiveBodySettings::mGetUpStrength },
        };
        const auto found = fields.find(key);
        if (found == fields.end())
            return false;
        this->*(found->second) = value;
        return true;
    }

    std::vector<std::string> ActiveBodySettings::load(const std::string& path)
    {
        std::vector<std::string> problems;
        std::ifstream file(path);
        if (!file)
        {
            problems.push_back("cannot open " + path);
            return problems;
        }
        std::string line;
        int number = 0;
        while (std::getline(file, line))
        {
            ++number;
            const auto hash = line.find('#');
            if (hash != std::string::npos)
                line.erase(hash);
            const auto equals = line.find('=');
            if (equals == std::string::npos)
            {
                if (line.find_first_not_of(" \t\r") != std::string::npos)
                    problems.push_back(path + ":" + std::to_string(number) + ": expected key = value");
                continue;
            }
            auto trim = [](std::string text) {
                const auto first = text.find_first_not_of(" \t\r");
                const auto last = text.find_last_not_of(" \t\r");
                return first == std::string::npos ? std::string() : text.substr(first, last - first + 1);
            };
            const std::string key = trim(line.substr(0, equals));
            const std::string text = trim(line.substr(equals + 1));
            std::istringstream parse(text);
            float value = 0.0f;
            if (!(parse >> value) || !set(key, value))
                problems.push_back(path + ":" + std::to_string(number) + ": bad setting '" + key + "'");
        }
        return problems;
    }
}
