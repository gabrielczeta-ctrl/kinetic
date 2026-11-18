#include <ragdollcore/lifecycle.hpp>

#include <ragdollcore/tuning.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string_view>
#include <vector>

namespace RagdollCore
{
    DeathCarry deathCarry(std::span<const JPH::Vec3> partVelocities, const DeathCarrySettings& settings)
    {
        DeathCarry carry;
        JPH::Vec3 common = JPH::Vec3::sZero();
        for (const JPH::Vec3& v : partVelocities)
            common += v;
        if (!partVelocities.empty())
            common /= static_cast<float>(partVelocities.size());
        carry.mUncapped = common;
        if (settings.mCap > 0.0f && common.Length() > settings.mCap)
            common = common.Normalized() * settings.mCap;
        carry.mCommon = common;
        carry.mPartBase = settings.mTrueCap ? carry.mUncapped : common;
        carry.mShare = settings.mShare;
        return carry;
    }

    DeathMotion deathMotion(const DeathMotion& old, std::span<const float, 3> momentum)
    {
        if (!(momentum[0] > 0.0f))
            return old;
        DeathMotion motion = old;
        motion.mCarry.mShare = std::min(momentum[0], 1.0f);
        motion.mCarry.mCap = std::max(momentum[1], 0.0f);
        motion.mCarry.mTrueCap = true;
        motion.mDragRate = std::max(momentum[2], 0.0f);
        return motion;
    }

    CorpseBuild corpseBuild(bool setting, bool carried, bool livingKnown, float blendSeconds)
    {
        CorpseBuild build;
        build.mLivingFrames = setting && carried && livingKnown;
        build.mBlendSeconds = setting && carried ? 0.0f : blendSeconds;
        return build;
    }

    namespace
    {
        // Past where the joint is, so it starts inside and not on its limit.
        constexpr float sDeathRangeMargin = 0.03f;

        float swingAngle(const JointAt& at)
        {
            return 2.0f * std::asin(std::min(1.0f, std::hypot(at.mSwingY, at.mSwingZ)));
        }
    }

    float jointExcess(const JointRange& range, const JointAt& at)
    {
        if (range.mHinge)
            return std::max({ 0.0f, range.mHingeMin - at.mHinge, at.mHinge - range.mHingeMax });
        float excess = std::max({ 0.0f, range.mTwistMin - at.mTwist, at.mTwist - range.mTwistMax });
        const float length = std::hypot(at.mSwingY, at.mSwingZ);
        if (length > 1e-6f)
        {
            // The ellipse's radius toward where the swing is. A locked axis
            // counts as a degree wide, so the noise on it reads as inside.
            const auto axis = [](float halfCone) {
                return std::max(0.01f, std::sin(0.5f * std::clamp(halfCone, 0.0f, JPH::JPH_PI)));
            };
            const float planeAxis = axis(range.mPlaneHalfCone);
            const float normalAxis = axis(range.mNormalHalfCone);
            const float radius
                = 1.0f / std::hypot(at.mSwingY / length / planeAxis, at.mSwingZ / length / normalAxis);
            excess = std::max(excess, swingAngle(at) - 2.0f * std::asin(std::min(1.0f, radius)));
        }
        return excess;
    }

    JointRange deathRangeOpened(const JointRange& authored, const JointAt& at)
    {
        JointRange opened = authored;
        if (authored.mHinge)
        {
            opened.mHingeMin = std::max(-JPH::JPH_PI, std::min(authored.mHingeMin, at.mHinge - sDeathRangeMargin));
            opened.mHingeMax = std::min(JPH::JPH_PI, std::max(authored.mHingeMax, at.mHinge + sDeathRangeMargin));
            return opened;
        }
        opened.mTwistMin = std::max(-JPH::JPH_PI, std::min(authored.mTwistMin, at.mTwist - sDeathRangeMargin));
        opened.mTwistMax = std::min(JPH::JPH_PI, std::max(authored.mTwistMax, at.mTwist + sDeathRangeMargin));
        // Both half cones as wide as the swing: the ellipse then holds it
        // whichever way it points.
        if (jointExcess({ false, authored.mNormalHalfCone, authored.mPlaneHalfCone, -JPH::JPH_PI, JPH::JPH_PI }, at)
            > 0.0f)
        {
            const float cone = std::min(JPH::JPH_PI, swingAngle(at) + sDeathRangeMargin);
            opened.mNormalHalfCone = std::max(authored.mNormalHalfCone, cone);
            opened.mPlaneHalfCone = std::max(authored.mPlaneHalfCone, cone);
        }
        return opened;
    }

    JointRange deathRangeEased(const JointRange& authored, const JointRange& opened, float progress)
    {
        const float t = std::clamp(progress, 0.0f, 1.0f);
        const auto ease = [t](float from, float to) { return from + (to - from) * t; };
        JointRange range = authored;
        range.mNormalHalfCone = ease(opened.mNormalHalfCone, authored.mNormalHalfCone);
        range.mPlaneHalfCone = ease(opened.mPlaneHalfCone, authored.mPlaneHalfCone);
        range.mTwistMin = ease(opened.mTwistMin, authored.mTwistMin);
        range.mTwistMax = ease(opened.mTwistMax, authored.mTwistMax);
        range.mHingeMin = ease(opened.mHingeMin, authored.mHingeMin);
        range.mHingeMax = ease(opened.mHingeMax, authored.mHingeMax);
        return range;
    }

    int deathDragSteps(float rate, float seconds, float dt)
    {
        return rate > 0.0f && dt > 0.0f ? static_cast<int>(seconds / dt) : 0;
    }

    JPH::Vec3 deathDragDrain(JPH::Vec3Arg meanVelocity, float rate, float dt)
    {
        const float keep = std::exp(-std::max(0.0f, rate) * dt);
        JPH::Vec3 drain = meanVelocity * (1.0f - keep);
        drain.SetZ(0.0f);
        return drain;
    }

    std::optional<JPH::RVec3> corpseFollowTarget(JPH::RVec3Arg root, JPH::RVec3Arg reference,
        std::optional<JPH::Real> floorZ, JPH::RVec3Arg actor)
    {
        JPH::RVec3 next(root.GetX(), root.GetY(), std::min(reference.GetZ(), root.GetZ()));
        if (floorZ)
            next.SetZ(*floorZ);
        // The actor's position is compared in floats, as the engine keeps it.
        const JPH::Vec3 nextActor(static_cast<float>(next.GetX()), static_cast<float>(next.GetY()),
            static_cast<float>(next.GetZ()));
        const JPH::Vec3 actorF(
            static_cast<float>(actor.GetX()), static_cast<float>(actor.GetY()), static_cast<float>(actor.GetZ()));
        if (JPH::Vec3(next - reference).LengthSq() < 0.25f * 0.25f && (actorF - nextActor).LengthSq() < 0.25f * 0.25f)
            return std::nullopt;
        return next;
    }

    std::optional<CorpseLoadPose> corpseLoadPose(float yaw, JPH::RVec3Arg actor, JPH::RVec3Arg root, JPH::Real floorZ)
    {
        // Only a body still standing: one already low was laid by its own pose.
        if (!(root.GetZ() - floorZ > 30.0))
            return std::nullopt;
        // Turned about its right axis so the up axis points back: on its back,
        // the chest up. Turned about the pelvis set down over the actor:
        // CORPSE_FOLLOW saved the actor under the lying pelvis, so that is
        // where the corpse lay (turned about the floor point the pelvis landed
        // its own height behind, and every load moved the corpse again).
        const JPH::Vec3 forward(std::sin(yaw), std::cos(yaw), 0.0f);
        CorpseLoadPose pose;
        pose.mTurn = JPH::Quat::sRotation(JPH::Vec3::sAxisZ().Cross(-forward).Normalized(), 0.5f * JPH::JPH_PI);
        pose.mPivot = JPH::RVec3(actor.GetX(), actor.GetY(), floorZ + 8.0);
        return pose;
    }

    BuildFrames buildFrames(const RagdollProfile& profile, const RagdollProfile& runtimeProfile,
        const SkeletonDescriptor& skeleton, bool onGroundStill)
    {
        const auto findBone = [&](const std::string& name) -> const Transform* {
            for (const auto& bone : skeleton.mBones)
                if (bone.mName == name)
                    return &bone.mModelTransform;
            return nullptr;
        };
        const auto boneOfBody = [&](const std::string& bodyName) -> std::string {
            for (const auto& body : runtimeProfile.mBodies)
                if (body.mName == bodyName)
                    return body.mBone;
            return {};
        };
        const auto parentOfBody = [&](const std::string& bodyName) -> std::string {
            for (const auto& body : runtimeProfile.mBodies)
                if (body.mName == bodyName && body.mParentBody)
                    return *body.mParentBody;
            return {};
        };
        const LandmarkProfile& landmarks = profile.mLandmarks;
        std::vector<std::string> frameBodies{ landmarks.mPelvis };
        frameBodies.insert(frameBodies.end(), landmarks.mSpine.begin(), landmarks.mSpine.end());
        if (landmarks.mHead)
            frameBodies.push_back(*landmarks.mHead);
        for (const auto& footName : { landmarks.mLeftFoot, landmarks.mRightFoot })
            if (footName)
            {
                const std::string calf = parentOfBody(*footName);
                frameBodies.insert(frameBodies.end(), { *footName, calf, parentOfBody(calf) });
            }
        // K2_LEFT_ARM_FRAMES: the arm that holds no weapon, from the standing pose too.
        static const auto leftArm_tuned = tuned<bool>([] {
            const char* text = tuningText("OPENMW_RAGDOLL_K2_LEFT_ARM_FRAMES");
            return text && std::atof(text) > 0.0;
        });
        if (leftArm_tuned.get() && landmarks.mLeftHand)
            for (std::string body = *landmarks.mLeftHand; !body.empty() && body != landmarks.mSpine.back();
                 body = parentOfBody(body))
                frameBodies.push_back(body);
        BuildFrames result;
        result.mFrames.mId = skeleton.mId;
        result.mKey = skeleton.mId;
        for (const auto& bone : skeleton.mBones)
            result.mKey += ";" + bone.mName;
        for (const auto& footName : { landmarks.mLeftFoot, landmarks.mRightFoot })
            if (footName)
            {
                const std::string calf = parentOfBody(*footName);
                const Transform* a = findBone(boneOfBody(parentOfBody(calf)));
                const Transform* b = findBone(boneOfBody(calf));
                const Transform* c = findBone(boneOfBody(*footName));
                if (a && b && c)
                    result.mKey += ":" + std::to_string(std::lround((b->mTranslation - a->mTranslation).Length()))
                        + "," + std::to_string(std::lround((c->mTranslation - b->mTranslation).Length()));
            }
        for (const auto& bone : skeleton.mBones)
            for (const std::string& body : frameBodies)
                if (!body.empty() && boneOfBody(body) == bone.mName)
                {
                    result.mFrames.mBones.push_back(bone);
                    result.mFrames.mBones.back().mParent = -1;
                    break;
                }
        const auto modelBone = [&](const std::optional<std::string>& body) -> const Transform* {
            return body ? findBone(boneOfBody(*body)) : nullptr;
        };
        const Transform* pelvis = findBone(boneOfBody(landmarks.mPelvis));
        const Transform* left = modelBone(landmarks.mLeftFoot);
        const Transform* right = modelBone(landmarks.mRightFoot);
        const Transform* chest = landmarks.mSpine.empty() ? nullptr : findBone(boneOfBody(landmarks.mSpine.back()));
        result.mStanding = pelvis && left && right && chest && onGroundStill;
        if (result.mStanding)
        {
            const auto flat = [](JPH::Vec3Arg a, JPH::Vec3Arg b) {
                return std::hypot(a.GetX() - b.GetX(), a.GetY() - b.GetY());
            };
            result.mStanding = flat(left->mTranslation, pelvis->mTranslation) < 15.0f
                && flat(right->mTranslation, pelvis->mTranslation) < 15.0f
                && std::abs(left->mTranslation.GetZ() - right->mTranslation.GetZ()) < 4.0f
                && flat(chest->mTranslation, pelvis->mTranslation) < 8.0f;
        }
        return result;
    }

    bool rebuildForRig(const RebuildInput& in)
    {
        return (in.mRigChanged || in.mUnready || in.mRegrip) && !in.mBusy && in.mStanding && in.mRootKinematic;
    }

    bool regripAfterRelease(const RegripInput& in, float delay)
    {
        return delay > 0.0f && in.mReleased && in.mBroke && in.mStillEquipped && in.mSinceRelease >= delay;
    }

    bool rebuildKeepsHeight(bool setting, bool heightLearned)
    {
        return setting && heightLearned;
    }

    bool beginPhysicalDeath(PhysicalDeathState& state, const PhysicalDeathSettings& settings, bool powered,
        std::optional<ActiveControllerMode> mode)
    {
        using M = ActiveControllerMode;
        if (!settings.mOn || !powered || state.mDying || !mode
            || !(*mode == M::Standing || *mode == M::Bracing || *mode == M::Stumbling))
            return false;
        state.mDying = true;
        state.mDone = false;
        state.mTime = 0.0f;
        return true;
    }

    bool stepPhysicalDeath(
        PhysicalDeathState& state, const PhysicalDeathSettings& settings, bool lying, bool powered, float dt)
    {
        if (!state.mDying || state.mDone)
            return false;
        state.mTime += dt;
        if (lying || state.mTime >= std::max(0.2f, settings.mMost) || !powered)
        {
            state.mDone = true;
            return true;
        }
        return false;
    }
}
