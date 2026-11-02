#include "effectmanager.hpp"

#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/PositionAttitudeTransform>

#include <components/misc/rng.hpp>
#include <ragdollcore/tuning.hpp>
#include <components/sceneutil/depth.hpp>
#include <components/sceneutil/material.hpp>

#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>

#include <components/sceneutil/controller.hpp>

#include "animation.hpp"
#include "util.hpp"
#include "vismask.hpp"

#include <algorithm>

namespace MWRender
{

    EffectManager::EffectManager(osg::ref_ptr<osg::Group> parent, Resource::ResourceSystem* resourceSystem)
        : mParentNode(std::move(parent))
        , mResourceSystem(resourceSystem)
    {
    }

    EffectManager::~EffectManager()
    {
        clear();
    }

    void EffectManager::addEffect(VFS::Path::NormalizedView model, std::string_view textureOverride,
        const osg::Vec3f& worldPosition, float scale, bool isMagicVFX, bool useAmbientLight, std::string_view effectId,
        bool loop)
    {
        osg::ref_ptr<osg::Node> node = mResourceSystem->getSceneManager()->getInstance(model);

        node->setNodeMask(Mask_Effect);

        Effect effect;
        effect.mAnimTime = std::make_shared<EffectAnimationTime>();
        effect.mLoop = loop;
        effect.mEffectId = effectId;

        SceneUtil::FindMaxControllerLengthVisitor findMaxLengthVisitor;
        node->accept(findMaxLengthVisitor);
        effect.mMaxControllerLength = findMaxLengthVisitor.getMaxLength();

        osg::ref_ptr<osg::PositionAttitudeTransform> trans = new osg::PositionAttitudeTransform;
        trans->setPosition(worldPosition);
        trans->setScale(osg::Vec3f(scale, scale, scale));
        trans->addChild(node);

        effect.mTransform = trans;

        SceneUtil::AssignControllerSourcesVisitor assignVisitor(effect.mAnimTime);
        node->accept(assignVisitor);

        if (isMagicVFX)
            overrideFirstRootTexture(VFS::Path::toNormalized(textureOverride), mResourceSystem, *node);
        else
            overrideTexture(VFS::Path::toNormalized(textureOverride), mResourceSystem, *node);

        mParentNode->addChild(trans);

        if (useAmbientLight)
        {
            // Morrowind has a white ambient light attached to the root VFX node of the scenegraph
            SceneUtil::configureSunAmbientOverride(osg::Vec4f(1, 1, 1, 1), node->getOrCreateStateSet());
        }

        mResourceSystem->getSceneManager()->setUpNormalsRTForStateSet(node->getOrCreateStateSet(), false);

        mEffects.push_back(std::move(effect));
    }

    void EffectManager::removeEffect(std::string_view effectId)
    {
        mEffects.erase(std::remove_if(mEffects.begin(), mEffects.end(),
                           [effectId, this](Effect& effect) {
                               if (effectId == effect.mEffectId)
                               {
                                   mParentNode->removeChild(effect.mTransform);
                                   return true;
                               }

                               return false;
                           }),
            mEffects.end());
    }

    void EffectManager::addSparks(const osg::Vec3f& worldPosition, const osg::Vec3f& normal, float strength)
    {
        strength = std::clamp(strength, 0.0f, 1.0f);
        osg::Vec3f n = normal;
        if (n.length2() < 1e-6f)
            n = osg::Vec3f(0, 0, 1);
        n.normalize();
        // Two directions across the normal for the cone.
        osg::Vec3f a = std::abs(n.z()) < 0.9f ? n ^ osg::Vec3f(0, 0, 1) : n ^ osg::Vec3f(1, 0, 0);
        a.normalize();
        const osg::Vec3f b = n ^ a;
        SparkBurst burst;
        // 2026-10-01 (screenshots): thin pale streaks were easy to miss on a
        // light floor - more of them, longer tails, a saturated orange tail,
        // and a starburst flash (very fast, very short-lived) at the impact.
        const int flash = 8;
        const int count = flash + 16 + static_cast<int>(26.0f * strength);
        for (int i = 0; i < count; ++i)
        {
            if (i < flash)
            {
                const float angle = 6.2831853f * (i + Misc::Rng::rollClosedProbability() * 0.5f) / flash;
                osg::Vec3f dir = n * 0.35f + a * std::cos(angle) + b * std::sin(angle);
                dir.normalize();
                burst.mPositions.push_back(worldPosition);
                burst.mVelocities.push_back(dir * (900.0f + 300.0f * strength));
                burst.mLifetimes.push_back(0.06f);
                continue;
            }
            // Thrown in a wide cone off the surface, mostly sideways.
            const float spread = 1.4f * Misc::Rng::rollClosedProbability();
            const float angle = 6.2831853f * Misc::Rng::rollClosedProbability();
            osg::Vec3f dir = n * std::cos(spread) + (a * std::cos(angle) + b * std::sin(angle)) * std::sin(spread);
            dir.normalize();
            const float speed = (180.0f + 320.0f * Misc::Rng::rollClosedProbability()) * (0.6f + 0.6f * strength);
            burst.mPositions.push_back(worldPosition);
            burst.mVelocities.push_back(dir * speed);
            burst.mLifetimes.push_back(0.15f + 0.3f * Misc::Rng::rollClosedProbability());
        }
        burst.mGeometry = new osg::Geometry;
        burst.mGeometry->setDataVariance(osg::Object::DYNAMIC);
        burst.mGeometry->setUseDisplayList(false);
        burst.mGeometry->setUseVertexBufferObjects(true);
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(2 * count);
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(2 * count);
        burst.mGeometry->setVertexArray(vertices);
        burst.mGeometry->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
        burst.mGeometry->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2 * count));
        burst.mGeometry->setCullingActive(false);
        osg::StateSet* state = burst.mGeometry->getOrCreateStateSet();
        state->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
        state->setMode(GL_BLEND, osg::StateAttribute::ON);
        state->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE));
        // Reversed depth aware (a plain osg::Depth LESS hid every spark), no write.
        state->setAttributeAndModes(new SceneUtil::AutoDepth(osg::Depth::LESS, 0.0, 1.0, false));
        osg::ref_ptr<SceneUtil::Material> material = new SceneUtil::Material;
        material->setVertexColorMode(SceneUtil::VertexColorModes::AmbientAndDiffuse);
        material->setStateSet(state);
        state->setAttributeAndModes(new osg::LineWidth(2.0f));
        state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(burst.mGeometry);
        geode->setNodeMask(Mask_Effect);
        burst.mNode = geode;
        // Without OpenMW's shaders the lines were not drawn at all.
        mResourceSystem->getSceneManager()->recreateShaders(geode, "debug");
        mParentNode->addChild(geode);
        mSparks.push_back(std::move(burst));
        updateSparks(0.0f);
    }

    namespace
    {
        // OPENMW_RAGDOLL_TIP_TRAIL = "on, speed u/s, full speed u/s, seconds,
        // alpha, glint": the ribbon shows above `speed` (tip), full at
        // `full speed`, each sample fading over `seconds`; `alpha` its
        // strongest; `glint` a tiny mark on a slow tip (0 = none).
        struct TrailSettings
        {
            bool mOn = true;
            float mSpeed = 250.0f, mFullSpeed = 900.0f, mSeconds = 0.15f, mAlpha = 0.28f, mGlint = 0.0f;
        };
        const TrailSettings& trailSettings()
        {
            static const auto value_tuned = RagdollCore::tuned<TrailSettings>([] {
                TrailSettings value;
                if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_TIP_TRAIL"))
                {
                    float on = 1.0f;
                    std::sscanf(text, "%f,%f,%f,%f,%f,%f", &on, &value.mSpeed, &value.mFullSpeed, &value.mSeconds,
                        &value.mAlpha, &value.mGlint);
                    value.mOn = on > 0.0f;
                }
                return value;
            });
            return value_tuned.get();
        }
    }

    void EffectManager::setWeaponBlades(const std::vector<WeaponBlade>& blades)
    {
        mPendingBlades = blades;
    }

    void EffectManager::updateTrails(float dt)
    {
        const TrailSettings& s = trailSettings();
        for (auto& [key, trail] : mTrails)
            trail.mSeen = false;
        if (s.mOn && dt > 0.0f)
            for (const WeaponBlade& blade : mPendingBlades)
            {
                WeaponTrail& trail = mTrails[blade.mKey];
                trail.mSeen = true;
                if (!trail.mGeometry)
                {
                    trail.mGeometry = new osg::Geometry;
                    trail.mGeometry->setDataVariance(osg::Object::DYNAMIC);
                    trail.mGeometry->setUseDisplayList(false);
                    trail.mGeometry->setUseVertexBufferObjects(true);
                    trail.mGeometry->setVertexArray(new osg::Vec3Array);
                    trail.mGeometry->setColorArray(new osg::Vec4Array, osg::Array::BIND_PER_VERTEX);
                    trail.mGeometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLE_STRIP, 0, 0));
                    trail.mGeometry->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 0));
                    trail.mGeometry->setCullingActive(false);
                    osg::StateSet* state = trail.mGeometry->getOrCreateStateSet();
                    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
                    state->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
                    state->setMode(GL_BLEND, osg::StateAttribute::ON);
                    // Additive, as the sparks: a pale shimmer, never a dark band.
                    state->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE));
                    state->setAttributeAndModes(new SceneUtil::AutoDepth(osg::Depth::LESS, 0.0, 1.0, false));
                    osg::ref_ptr<SceneUtil::Material> material = new SceneUtil::Material;
                    material->setVertexColorMode(SceneUtil::VertexColorModes::AmbientAndDiffuse);
                    material->setStateSet(state);
                    state->setAttributeAndModes(new osg::LineWidth(1.5f));
                    state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                    geode->addDrawable(trail.mGeometry);
                    geode->setNodeMask(Mask_Effect);
                    trail.mNode = geode;
                    mResourceSystem->getSceneManager()->recreateShaders(geode, "debug");
                    mParentNode->addChild(geode);
                }
                // The tip's speed since the last frame decides how much shows.
                float speed = 0.0f;
                if (!trail.mSamples.empty())
                    speed = (blade.mTip - trail.mSamples.front().mTip).length() / dt;
                const float k = std::clamp((speed - s.mSpeed) / std::max(1.0f, s.mFullSpeed - s.mSpeed), 0.0f, 1.0f);
                trail.mSamples.insert(trail.mSamples.begin(), TrailSample{ blade.mBase, blade.mTip, 0.0f, k });
            }
        mPendingBlades.clear();
        for (auto it = mTrails.begin(); it != mTrails.end();)
        {
            WeaponTrail& trail = it->second;
            for (TrailSample& sample : trail.mSamples)
                sample.mAge += dt;
            // The newest sample stays (the next frame's speed is measured from it).
            while (trail.mSamples.size() > 1 && trail.mSamples.back().mAge > s.mSeconds)
                trail.mSamples.pop_back();
            if (!trail.mSeen && (trail.mSamples.empty() || trail.mSamples.front().mAge > s.mSeconds))
            {
                mParentNode->removeChild(trail.mNode);
                it = mTrails.erase(it);
                continue;
            }
            auto* vertices = static_cast<osg::Vec3Array*>(trail.mGeometry->getVertexArray());
            auto* colors = static_cast<osg::Vec4Array*>(trail.mGeometry->getColorArray());
            vertices->clear();
            colors->clear();
            // The ribbon: base and tip of each sample, brightest at the tip
            // and newest, gone by `seconds` old.
            for (const TrailSample& sample : trail.mSamples)
            {
                const float fade = std::clamp(1.0f - sample.mAge / s.mSeconds, 0.0f, 1.0f);
                const float a = s.mAlpha * sample.mStrength * fade * fade;
                vertices->push_back(sample.mBase);
                colors->push_back(osg::Vec4f(0.85f, 0.88f, 0.92f, 0.0f));
                vertices->push_back(sample.mTip);
                colors->push_back(osg::Vec4f(0.95f, 0.96f, 1.0f, a));
            }
            const auto ribbon = static_cast<GLsizei>(vertices->size());
            // The glint: a tiny cross on a slow tip, so the reach shows at rest.
            GLsizei glint = 0;
            if (trail.mSeen && s.mGlint > 0.0f && !trail.mSamples.empty())
            {
                const TrailSample& now = trail.mSamples.front();
                const float a = s.mGlint * (1.0f - now.mStrength);
                if (a > 0.01f)
                {
                    osg::Vec3f along = now.mTip - now.mBase;
                    along.normalize();
                    osg::Vec3f side = along ^ osg::Vec3f(0, 0, 1);
                    if (side.length2() < 1e-4f)
                        side = along ^ osg::Vec3f(1, 0, 0);
                    side.normalize();
                    const osg::Vec3f up = along ^ side;
                    for (const osg::Vec3f& d : { side, up })
                    {
                        vertices->push_back(now.mTip - d * 1.5f);
                        colors->push_back(osg::Vec4f(1.0f, 0.93f, 0.75f, a));
                        vertices->push_back(now.mTip + d * 1.5f);
                        colors->push_back(osg::Vec4f(1.0f, 0.93f, 0.75f, a));
                    }
                    glint = 4;
                }
            }
            static_cast<osg::DrawArrays*>(trail.mGeometry->getPrimitiveSet(0))->setCount(ribbon >= 4 ? ribbon : 0);
            auto* lines = static_cast<osg::DrawArrays*>(trail.mGeometry->getPrimitiveSet(1));
            lines->setFirst(ribbon);
            lines->setCount(glint);
            vertices->dirty();
            colors->dirty();
            trail.mGeometry->dirtyBound();
            ++it;
        }
    }

    void EffectManager::updateSparks(float dt)
    {
        for (auto& burst : mSparks)
        {
            burst.mAge += dt;
            auto* vertices = static_cast<osg::Vec3Array*>(burst.mGeometry->getVertexArray());
            auto* colors = static_cast<osg::Vec4Array*>(burst.mGeometry->getColorArray());
            for (std::size_t i = 0; i < burst.mPositions.size(); ++i)
            {
                burst.mVelocities[i].z() -= 627.0f * dt; // Morrowind gravity, units/s^2
                burst.mVelocities[i] *= std::max(0.0f, 1.0f - 1.5f * dt);
                burst.mPositions[i] += burst.mVelocities[i] * dt;
                const float life = std::clamp(1.0f - burst.mAge / burst.mLifetimes[i], 0.0f, 1.0f);
                // A streak: the spark's last few milliseconds of travel.
                (*vertices)[2 * i] = burst.mPositions[i];
                (*vertices)[2 * i + 1] = burst.mPositions[i] - burst.mVelocities[i] * 0.035f;
                (*colors)[2 * i] = osg::Vec4f(1.0f, 0.9f * (0.7f + 0.3f * life), 0.5f * life, life);
                (*colors)[2 * i + 1] = osg::Vec4f(1.0f, 0.35f + 0.3f * life, 0.0f, 0.6f * life);
            }
            vertices->dirty();
            colors->dirty();
            burst.mGeometry->dirtyBound();
        }
        mSparks.erase(std::remove_if(mSparks.begin(), mSparks.end(),
                          [this](const SparkBurst& burst) {
                              if (burst.mAge < 0.5f)
                                  return false;
                              mParentNode->removeChild(burst.mNode);
                              return true;
                          }),
            mSparks.end());
    }

    void EffectManager::update(float dt)
    {
        updateSparks(dt);
        updateTrails(dt);
        mEffects.erase(std::remove_if(mEffects.begin(), mEffects.end(),
                           [dt, this](Effect& effect) {
                               bool remove = false;
                               effect.mAnimTime->addTime(dt);
                               if (effect.mAnimTime->getTime() >= effect.mMaxControllerLength)
                               {
                                   if (effect.mLoop)
                                   {
                                       float remainder = effect.mAnimTime->getTime() - effect.mMaxControllerLength;
                                       effect.mAnimTime->resetTime(remainder);
                                   }
                                   else
                                   {
                                       mParentNode->removeChild(effect.mTransform);
                                       remove = true;
                                   }
                               }

                               return remove;
                           }),
            mEffects.end());
    }

    void EffectManager::clear()
    {
        for (const auto& burst : mSparks)
            mParentNode->removeChild(burst.mNode);
        mSparks.clear();
        for (const auto& effect : mEffects)
        {
            mParentNode->removeChild(effect.mTransform);
        }
        mEffects.clear();
    }

}
