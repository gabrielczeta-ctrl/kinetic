#include <ragdollcore/fallprotectpose.hpp>

namespace RagdollCore
{
    void applyFallProtectPose(std::vector<Transform>& target, const FallProtectCommand& command,
        const FallPoseChains& chains, const JPH::Quat& chestWorldRotation)
    {
        const auto& arms = chains.mArms;
        JPH::Vec3 modelForward = JPH::Vec3::sAxisY();
        {
            JPH::Vec3 across = target[arms[1].mFrame].mTranslation - target[arms[0].mFrame].mTranslation;
            across.SetZ(0.0f);
            if (!across.IsNearZero())
                modelForward = JPH::Vec3::sAxisZ().Cross(across.Normalized());
        }
        const JPH::Vec3 bend = JPH::Vec3::sAxisZ().Cross(modelForward);
        const auto turn = [&](std::size_t index, const JPH::Quat& q, JPH::Vec3Arg pivot) {
            auto& transform = target[index];
            transform.mTranslation = pivot + q * (transform.mTranslation - pivot);
            transform.mRotation = (q * transform.mRotation).Normalized();
        };
        const auto armPart = [&](std::size_t i) {
            for (const auto& arm : arms)
                if (i == arm.mShoulder || i == arm.mElbow || i == arm.mHand)
                    return true;
            return false;
        };
        const auto& spine = chains.mSpine;
        if (command.mSpineCurl != 0.0f && !spine.empty())
        {
            const JPH::Quat full = JPH::Quat::sRotation(bend, command.mSpineCurl);
            const JPH::Vec3 pivot = target[spine.front()].mTranslation;
            for (std::size_t k = 0; k < spine.size(); ++k)
                turn(spine[k],
                    JPH::Quat::sIdentity().SLERP(full, static_cast<float>(k + 1) / static_cast<float>(spine.size())),
                    pivot);
            for (std::size_t i = 0; i < target.size(); ++i)
                if (i == chains.mHead || (chains.mNeck && i == *chains.mNeck) || armPart(i) || i == arms[0].mFrame
                    || i == arms[1].mFrame)
                    turn(i, full, pivot);
        }
        for (const auto& leg : chains.mLegs)
        {
            if (command.mHipBend != 0.0f)
            {
                const JPH::Quat q = JPH::Quat::sRotation(bend, -command.mHipBend);
                const JPH::Vec3 pivot = target[leg.mHip].mTranslation;
                for (const std::size_t index : { leg.mHip, leg.mKnee, leg.mFoot })
                    turn(index, q, pivot);
            }
            if (command.mKneeBend != 0.0f)
            {
                const JPH::Quat q = JPH::Quat::sRotation(bend, command.mKneeBend);
                const JPH::Vec3 pivot = target[leg.mKnee].mTranslation;
                for (const std::size_t index : { leg.mKnee, leg.mFoot })
                    turn(index, q, pivot);
            }
        }
        if (command.mHeadPitch != 0.0f && chains.mNeck)
            turn(chains.mHead, JPH::Quat::sRotation(bend, command.mHeadPitch), target[*chains.mNeck].mTranslation);
        if (spine.empty())
            return;
        const std::size_t chest = spine.back();
        const JPH::Quat worldToModel = target[chest].mRotation * chestWorldRotation.Conjugated();
        for (std::size_t side = 0; side < 2; ++side)
        {
            const float weight = command.mArmWeight[side];
            const FallVec3& own = command.mArmDirections[side];
            const JPH::Vec3 reach = worldToModel * JPH::Vec3(own.mX, own.mY, own.mZ);
            const JPH::Vec3 axis = target[arms[side].mElbow].mTranslation - target[arms[side].mShoulder].mTranslation;
            if (weight <= 0.0f || axis.IsNearZero())
                continue;
            const JPH::Vec3 from = axis.Normalized();
            const JPH::Vec3 blend = from * (1.0f - weight) + reach * weight;
            if (blend.IsNearZero())
                continue;
            const JPH::Quat q = JPH::Quat::sFromTo(from, blend.Normalized());
            const JPH::Vec3 pivot = target[arms[side].mShoulder].mTranslation;
            for (const std::size_t index : { arms[side].mShoulder, arms[side].mElbow, arms[side].mHand })
                turn(index, q, pivot);
        }
    }

    std::optional<float> fallContactRadius(AnatomicalRegion region, bool legContacts)
    {
        using R = AnatomicalRegion;
        if (region == R::Unknown || region == R::LeftFoot || region == R::RightFoot)
            return std::nullopt;
        if (!legContacts
            && (region == R::LeftCalf || region == R::RightCalf || region == R::LeftThigh || region == R::RightThigh))
            return std::nullopt;
        return region == R::Head ? 9.0f
            : region == R::LeftHand || region == R::RightHand || region == R::LeftForearm || region == R::RightForearm
            ? 4.0f
            : region == R::LeftCalf || region == R::RightCalf || region == R::LeftUpperArm || region == R::RightUpperArm
            ? 5.0f
            : region == R::LeftThigh || region == R::RightThigh ? 6.0f
                                                                : 10.0f;
    }

    bool fallBodyContact(AnatomicalRegion region)
    {
        using R = AnatomicalRegion;
        return region != R::LeftHand && region != R::RightHand && region != R::LeftForearm && region != R::RightForearm;
    }

    float fallSpringScale(const FallProtectCommand& command, bool armPart, bool leftArm, bool leg)
    {
        return armPart ? command.mArmStiffnesses[leftArm ? 0 : 1] : leg ? command.mLegStiffness : command.mUpperStiffness;
    }

    BraceDecision decideBraceArms(const BraceInput& in)
    {
        using M = ActiveControllerMode;
        BraceDecision d;
        const bool fallingEntry = in.mMode && !in.mRootKinematic && *in.mMode == M::Falling && in.mTimeInMode <= 0.20f;
        const bool supportedRecovery = in.mGroundContact && in.mMode
            && (*in.mMode == M::GettingUp || (in.mFallenBrace && *in.mMode == M::Fallen && in.mRecoveryRequested));
        d.mWanted = in.mEnabled && (fallingEntry || supportedRecovery);
        d.mProtectiveFall = d.mWanted && *in.mMode == M::Falling;
        if (d.mProtectiveFall && in.mCenterOfMassVelocity)
        {
            d.mFallDirection = JPH::Vec3(in.mCenterOfMassVelocity->GetX(), in.mCenterOfMassVelocity->GetY(), 0.0f);
            if (!d.mFallDirection.IsNearZero())
                d.mFallDirection = d.mFallDirection.Normalized();
        }
        return d;
    }
}
