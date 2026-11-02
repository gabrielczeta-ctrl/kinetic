#ifndef OPENMW_RAGDOLLCORE_WALKSEARCH_NATIVEBODY_H
#define OPENMW_RAGDOLLCORE_WALKSEARCH_NATIVEBODY_H

// Captured game body loader shared by the walksearch driver modes. The
// creation descriptor (replay_context.py creation block) supplies the
// skeleton the game built the ragdoll from; the YAML profile supplies the
// bodies, constraints and motor limits. Only the header and skeleton prefix
// of the context file are read; replay frames are not part of body creation.

#include <ragdollcore/profile.hpp>
#include <ragdollcore/scenario.hpp>

#include <components/ragdollprofile/loader.hpp>

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace WalkSearch
{
    struct NativeBody
    {
        RagdollCore::SkeletonDescriptor mSkeleton;
        RagdollCore::RagdollProfile mProfile;
        // Header values recorded by the game at creation.
        std::uint64_t mProfileHash = 0;
        float mTotalMass = 0.0f;
        float mUniformScale = 1.0f;
        float mGroundHeight = 0.0f;
    };

    inline NativeBody loadNativeBody(
        const std::string& contextPath, const std::string& profilePath, RagdollCore::ScenarioSettings& settings)
    {
        std::ifstream profileStream(profilePath);
        auto loaded = ::RagdollProfile::load(profileStream);
        if (!loaded.isValid())
            throw std::runtime_error("Cannot load the native YAML profile");
        NativeBody body;
        body.mProfile = std::move(*loaded.mProfile);
        std::ifstream context(contextPath);
        const auto point = [&]() {
            double x = 0, y = 0, z = 0;
            if (!(context >> x >> y >> z))
                throw std::runtime_error("Truncated context vector");
            return JPH::RVec3(x, y, z);
        };
        const auto vector = [&]() { return JPH::Vec3(point()); };
        const auto rotation = [&]() {
            float x = 0, y = 0, z = 0, w = 1;
            if (!(context >> x >> y >> z >> w))
                throw std::runtime_error("Truncated context rotation");
            return JPH::Quat(x, y, z, w);
        };
        unsigned version = 0;
        std::uint64_t profileHash = 0;
        std::size_t warmup = 0, boneCount = 0;
        std::string profileId;
        context >> version >> std::quoted(profileId) >> profileHash >> warmup;
        if (!context || profileId != body.mProfile.mId)
            throw std::runtime_error("Context profile id does not match the YAML profile");
        float totalMass = 0.0f, uniformScale = 1.0f, groundHeight = 0.0f;
        context >> totalMass >> uniformScale >> groundHeight;
        (void)point();
        (void)rotation();
        (void)vector();
        (void)vector();
        context >> boneCount;
        if (!context || boneCount != body.mProfile.mBodies.size())
            throw std::runtime_error("Unexpected creation skeleton dimensions");
        body.mSkeleton.mId = "captured-native-creation";
        for (std::size_t i = 0; i < boneCount; ++i)
        {
            RagdollCore::SkeletonBone bone;
            context >> std::quoted(bone.mName) >> bone.mParent;
            bone.mModelTransform.mTranslation = vector();
            bone.mModelTransform.mRotation = rotation();
            body.mSkeleton.mBones.push_back(bone);
        }
        body.mProfileHash = profileHash;
        body.mTotalMass = totalMass;
        body.mUniformScale = uniformScale;
        body.mGroundHeight = groundHeight;
        settings.mTotalMass = totalMass;
        settings.mUniformScale = uniformScale;
        return body;
    }
}

#endif
