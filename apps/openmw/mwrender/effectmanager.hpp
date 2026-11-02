#ifndef OPENMW_MWRENDER_EFFECTMANAGER_H
#define OPENMW_MWRENDER_EFFECTMANAGER_H

#include <memory>
#include <map>
#include <vector>

#include <osg/Vec3f>

#include <osg/ref_ptr>

#include <components/vfs/pathutil.hpp>

namespace osg
{
    class Group;
    class Vec3f;
    class PositionAttitudeTransform;
    class Geometry;
    class Node;
}

namespace Resource
{
    class ResourceSystem;
}

namespace MWRender
{
    class EffectAnimationTime;

    // Note: effects attached to another object should be managed by MWRender::Animation::addEffect.
    // This class manages "free" effects, i.e. attached to a dedicated scene node in the world.
    class EffectManager
    {
    public:
        EffectManager(osg::ref_ptr<osg::Group> parent, Resource::ResourceSystem* resourceSystem);
        EffectManager(const EffectManager&) = delete;
        ~EffectManager();

        /// Add an effect. When it's finished playing, it will be removed automatically.
        void addEffect(VFS::Path::NormalizedView model, std::string_view textureOverride,
            const osg::Vec3f& worldPosition, float scale, bool isMagicVFX = true, bool useAmbientLight = true,
            std::string_view effectId = {}, bool loop = false);

        void removeEffect(std::string_view effectId);

        /// A burst of bright streaks (metal sparks): thrown off around normal,
        /// falling under gravity, fading from white-yellow to orange.
        void addSparks(const osg::Vec3f& worldPosition, const osg::Vec3f& normal, float strength);

        /// TRAIL-01 (user 10-29: "a visible subtle whoosh at the tip of the
        /// weapon so the player knows the range"): this frame's blades, the
        /// last part of each held weapon (base) to its tip, world space. A
        /// faint ribbon trails a blade while its tip moves fast; a blade not
        /// given for a frame fades out.
        struct WeaponBlade
        {
            const void* mKey = nullptr;
            osg::Vec3f mBase, mTip;
        };
        void setWeaponBlades(const std::vector<WeaponBlade>& blades);

        void update(float dt);

        /// Remove all effects
        void clear();

    private:
        struct Effect
        {
            std::string mEffectId;
            float mMaxControllerLength;
            bool mLoop;
            std::shared_ptr<EffectAnimationTime> mAnimTime;
            osg::ref_ptr<osg::PositionAttitudeTransform> mTransform;
        };

        std::vector<Effect> mEffects;

        struct SparkBurst
        {
            osg::ref_ptr<osg::Node> mNode;
            osg::ref_ptr<osg::Geometry> mGeometry;
            std::vector<osg::Vec3f> mPositions, mVelocities;
            std::vector<float> mLifetimes;
            float mAge = 0.0f;
        };
        std::vector<SparkBurst> mSparks;
        void updateSparks(float dt);

        struct TrailSample
        {
            osg::Vec3f mBase, mTip;
            float mAge = 0.0f;
            float mStrength = 0.0f; // 0..1 by the tip's speed when it was taken
        };
        struct WeaponTrail
        {
            osg::ref_ptr<osg::Node> mNode;
            osg::ref_ptr<osg::Geometry> mGeometry;
            std::vector<TrailSample> mSamples; // newest first
            bool mSeen = false;
        };
        std::map<const void*, WeaponTrail> mTrails;
        std::vector<WeaponBlade> mPendingBlades;
        void updateTrails(float dt);

        osg::ref_ptr<osg::Group> mParentNode;
        Resource::ResourceSystem* mResourceSystem;
    };

}

#endif
