#include <ragdollcore/haftfit.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    float HaftFit::worst() const
    {
        return std::max({ mWeaponReach, mSupportReach, mInTrunk, mCrossed });
    }

    HaftFit haftFit(const ChamberPose& pose, const JPH::Vec3& bladeAxis, const HaftBody& body)
    {
        HaftFit fit;
        const JPH::Vec3 blade = (pose.mRotation * bladeAxis).NormalizedOr(JPH::Vec3::sAxisY());
        fit.mSupport = pose.mHand - blade * body.mGrip;
        // The chest's turn: negative brings the weapon shoulder forward.
        const float c = std::cos(pose.mTwist), s = std::sin(pose.mTwist);
        const JPH::Vec3 weaponShoulder(body.mShoulderOut * c, -body.mShoulderOut * s, body.mShoulderUp);
        const JPH::Vec3 supportShoulder(-body.mShoulderOut * c, body.mShoulderOut * s, body.mShoulderUp);
        const float limit = 0.97f * body.mArm;
        fit.mWeaponReach = std::max(0.0f, (pose.mHand - weaponShoulder).Length() - limit);
        fit.mSupportReach = std::max(0.0f, (fit.mSupport - supportShoulder).Length() - limit);
        // The trunk's capsule.
        const float z = std::clamp(fit.mSupport.GetZ(), -body.mTrunkBottom, body.mTrunkTop);
        const float gap = (fit.mSupport - JPH::Vec3(0.0f, 0.0f, z)).Length();
        fit.mInTrunk = std::max(0.0f, body.mTrunkRadius - gap);
        // Across the chest's own right (the shoulder line).
        const JPH::Vec3 right(c, -s, 0.0f);
        fit.mCrossed = std::max(0.0f, (fit.mSupport - pose.mHand).Dot(right) - 0.5f * body.mGrip);
        return fit;
    }

    HaftLineFit haftLineFit(const AttackLine& line, const WeaponStrikeAxes& axes, const HaftBody& body)
    {
        HaftLineFit result;
        constexpr int places = 32;
        for (int i = 0; i <= places; ++i)
        {
            const float u = 2.0f * static_cast<float>(i) / places;
            const auto pose = linePose(line, axes, u);
            if (!pose)
                continue;
            const HaftFit fit = haftFit(*pose, axes.mLongAxis, body);
            if (fit.worst() > result.mWorst.worst())
            {
                result.mWorst = fit;
                result.mU = u;
            }
        }
        return result;
    }
}
