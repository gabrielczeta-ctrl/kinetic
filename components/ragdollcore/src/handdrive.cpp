#include <ragdollcore/handdrive.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace RagdollCore
{
    HandDriveGains handDriveGains(float weaponWeight, float control, float response)
    {
        HandDriveGains gains;
        const float carried = std::clamp(weaponWeight * 0.08f, 0.4f, 4.0f);
        const float slower = std::sqrt(carried / 1.6f);
        const float r = std::clamp(response, 0.25f, 4.0f);
        gains.mFrequency *= r / slower;
        gains.mRotationFrequency *= r / slower;
        gains.mMaxAcceleration *= r * r;
        gains.mMaxAngularAcceleration *= r * r;
        gains.mStrength = 0.6f + 0.4f * std::clamp(control, 0.0f, 1.0f);
        return gains;
    }

    JPH::Quat nearestEdgeRotation(JPH::QuatArg goal, JPH::QuatArg current, JPH::Vec3Arg bladeAxis)
    {
        const JPH::Quat other = (goal * JPH::Quat::sRotation(bladeAxis.NormalizedOr(JPH::Vec3::sAxisZ()),
                                            std::numbers::pi_v<float>))
                                    .Normalized();
        return std::abs(other.Dot(current)) > std::abs(goal.Dot(current)) ? other : goal;
    }

    HandDriveOutput solveHandDrive(const HandChain& chain, const HandDriveGoal& goal, const HandDriveGains& gains,
        float gravity)
    {
        HandDriveOutput out;
        constexpr float tau = 2.0f * std::numbers::pi_v<float>;
        // The mass the grip carries: the hand and weapon whole, the forearm
        // half, the upper arm a quarter (their share of the grip's motion).
        out.mEffectiveMass = chain.mBody[2].mMass + chain.mBody[3].mMass + 0.5f * chain.mBody[1].mMass
            + 0.25f * chain.mBody[0].mMass;
        float rigidInertia = 0.0f; // the hand and weapon's about their centre
        if (chain.mWeaponInertia > 0.0f)
        {
            // The hand (a point) and the weapon as one rigid body: pushed at
            // the grip, 1/m = 1/M + r^2/I (r the grip's lever about its centre).
            const auto& h = chain.mBody[2];
            const auto& w = chain.mBody[3];
            const float total = h.mMass + w.mMass;
            if (total > 0.0f)
            {
                const JPH::RVec3 centre = (h.mCenterOfMass * h.mMass + w.mCenterOfMass * w.mMass) / total;
                const float inertia = chain.mWeaponInertia
                    + w.mMass * static_cast<float>((w.mCenterOfMass - centre).LengthSq())
                    + h.mMass * static_cast<float>((h.mCenterOfMass - centre).LengthSq());
                rigidInertia = inertia;
                const float lever = static_cast<float>((chain.mGrip - centre).LengthSq());
                const float rigid = 1.0f / (1.0f / total + lever / std::max(inertia, 1e-6f));
                out.mEffectiveMass = rigid + 0.5f * chain.mBody[1].mMass + 0.25f * chain.mBody[0].mMass;
            }
        }

        // The grip's PD in task space.
        const float omega = tau * std::max(0.0f, gains.mFrequency);
        const JPH::Vec3 error(goal.mGrip - chain.mGrip);
        const JPH::Vec3 velocityError = goal.mGripVelocity - chain.mGripVelocity;
        // Stable PD: the spring's pull taken at the end of the step.
        const float kp = omega * omega, kd = 2.0f * gains.mDamping * omega, dt = std::max(0.0f, gains.mDt);
        JPH::Vec3 acceleration = (error * kp + velocityError * (kd + kp * dt)) / (1.0f + kd * dt + kp * dt * dt);
        if (const float a = acceleration.Length(); a > gains.mMaxAcceleration && a > 0.0f)
            acceleration *= gains.mMaxAcceleration / a;
        out.mForce = acceleration * out.mEffectiveMass;

        // The hand's rotation: a PD on the rotation error, about the wrist.
        JPH::Vec3 rotationTorque = JPH::Vec3::sZero();
        if (gains.mRotationTorque || gains.mComputeRotation)
        {
            JPH::Vec3 rotationError = JPH::Vec3::sZero();
            if (goal.mBladeAxis)
            {
                // The blade's direction, then the roll about it the nearer way.
                const JPH::Vec3 local = goal.mBladeAxis->NormalizedOr(JPH::Vec3::sAxisZ());
                const JPH::Vec3 bladeNow = chain.mHandRotation * local;
                const JPH::Vec3 bladeWant = goal.mRotation * local;
                const JPH::Quat swing = JPH::Quat::sFromTo(bladeNow, bladeWant);
                JPH::Vec3 swingAxis;
                float swingAngle = 0.0f;
                swing.GetAxisAngle(swingAxis, swingAngle);
                if (swingAngle > 1e-6f)
                    rotationError += swingAxis * swingAngle;
                JPH::Quat roll = (goal.mRotation * (swing * chain.mHandRotation).Conjugated()).Normalized();
                if (roll.GetW() < 0.0f)
                    roll = -roll;
                JPH::Vec3 rollAxis;
                float rollAngle = 0.0f;
                roll.GetAxisAngle(rollAxis, rollAngle);
                if (rollAngle > 1e-6f)
                {
                    float signedRoll = rollAxis.Dot(bladeWant) >= 0.0f ? rollAngle : -rollAngle;
                    constexpr float pi = std::numbers::pi_v<float>;
                    if (signedRoll > 0.5f * pi)
                        signedRoll -= pi;
                    else if (signedRoll < -0.5f * pi)
                        signedRoll += pi;
                    rotationError += bladeWant * (signedRoll * goal.mRollShare);
                }
            }
            else
            {
                JPH::Quat delta = (goal.mRotation * chain.mHandRotation.Conjugated()).Normalized();
                if (delta.GetW() < 0.0f)
                    delta = -delta;
                JPH::Vec3 axis;
                float angle = 0.0f;
                delta.GetAxisAngle(axis, angle);
                if (angle > 1e-6f)
                    rotationError = axis * angle;
            }
            const float rotOmega = tau * std::max(0.0f, gains.mRotationFrequency);
            float inertia = chain.mHandInertia;
            // A torque on a body held by a soft wrist turns the hand and weapon
            // about their own centre: their inertia there, not the wrist's
            // (point masses at their centres about the wrist over-drove a
            // weapon with its centre far out by up to 2x: VARIANT-01 11-07).
            if (!(inertia > 0.0f) && rigidInertia > 0.0f && chain.mRigidTurn)
                inertia = rigidInertia;
            if (!(inertia > 0.0f))
            {
                // The weapon's and the hand's centres about the wrist (point
                // masses; the caller gives the real inertia when it has it).
                const float lever = static_cast<float>((chain.mBody[3].mCenterOfMass - chain.mJoint[2]).Length());
                const float hand = static_cast<float>((chain.mBody[2].mCenterOfMass - chain.mJoint[2]).Length());
                inertia = chain.mBody[3].mMass * lever * lever + chain.mBody[2].mMass * hand * hand
                    + std::max(0.0f, chain.mTurnInertia);
            }
            const float rkp = rotOmega * rotOmega, rkd = 2.0f * gains.mRotationDamping * rotOmega;
            JPH::Vec3 angular = (rotationError * rkp
                                    + (goal.mAngularVelocity - chain.mHandAngularVelocity) * (rkd + rkp * dt))
                / (1.0f + rkd * dt + rkp * dt * dt);
            if (const float a = angular.Length(); a > gains.mMaxAngularAcceleration && a > 0.0f)
                angular *= gains.mMaxAngularAcceleration / a;
            rotationTorque = angular * inertia;
        }

        // The elbow's lead toward its pole: a force at the elbow across the
        // shoulder-grip line, carried by the shoulder alone.
        JPH::Vec3 poleTorque = JPH::Vec3::sZero();
        if (goal.mPole && gains.mPoleFrequency > 0.0f)
        {
            const JPH::Vec3 line(chain.mGrip - chain.mJoint[0]);
            const JPH::Vec3 elbow(chain.mJoint[1] - chain.mJoint[0]);
            const JPH::Vec3 axis = line.NormalizedOr(JPH::Vec3::sAxisY());
            const JPH::Vec3 bend = elbow - axis * elbow.Dot(axis);
            JPH::Vec3 want = *goal.mPole - axis * goal.mPole->Dot(axis);
            if (bend.Length() > 1e-3f && want.Length() > 1e-3f)
            {
                want = want.Normalized() * bend.Length();
                const float poleOmega = tau * gains.mPoleFrequency;
                const JPH::Vec3 force = (want - bend) * (poleOmega * poleOmega * (chain.mBody[0].mMass + chain.mBody[1].mMass));
                poleTorque = elbow.Cross(force);
            }
        }

        out.mRotationTorque = rotationTorque;

        // Gravity on everything distal to each joint, cancelled.
        const JPH::Vec3 up(0.0f, 0.0f, 1.0f);
        const std::array<float, 3> cap{ chain.mStrength[0] * gains.mStrength, chain.mStrength[1] * gains.mStrength,
            chain.mStrength[2] * gains.mStrength };
        for (std::size_t j = 0; j < 3; ++j)
        {
            const JPH::Vec3 lever(chain.mGrip - chain.mJoint[j]);
            JPH::Vec3 torque = lever.Cross(out.mForce) + (gains.mRotationTorque ? rotationTorque : JPH::Vec3::sZero());
            if (j == 0)
                torque += poleTorque;
            if (gains.mGravity)
                for (std::size_t b = j; b < chain.mBody.size(); ++b)
                {
                    const JPH::Vec3 arm(chain.mBody[b].mCenterOfMass - chain.mJoint[j]);
                    torque += arm.Cross(up * (chain.mBody[b].mMass * gravity));
                }
            const float magnitude = torque.Length();
            out.mUtilization[j] = cap[j] > 0.0f ? magnitude / cap[j] : 0.0f;
            if (cap[j] > 0.0f && magnitude > cap[j])
                torque *= cap[j] / magnitude;
            out.mTorque[j] = torque;
        }
        return out;
    }
}
