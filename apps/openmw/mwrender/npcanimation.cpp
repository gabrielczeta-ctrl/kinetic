#include "npcanimation.hpp"
#include <ragdollcore/tuning.hpp>

#include <limits>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include <osg/Depth>
#include <osg/Transform>
#include <osg/MatrixTransform>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osg/BlendFunc>
#include <osg/PolygonOffset>
#include <osg/Program>
#include <osg/Shader>
#include <osg/TriangleFunctor>
#include <osg/UserDataContainer>

#include <osgUtil/CullVisitor>
#include <osgUtil/RenderBin>

#include <components/debug/debuglog.hpp>

#include <components/misc/rng.hpp>

#include <components/misc/resourcehelpers.hpp>

#include <components/esm3/loadbody.hpp>
#include <components/esm3/loadarmo.hpp>
#include <components/esm3/loadmgef.hpp>
#include <components/esm3/loadrace.hpp>
#include <components/esm3/loadskil.hpp>
#include <components/misc/strings/algorithm.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/sceneutil/depth.hpp>
#include <components/sceneutil/keyframe.hpp>
#include <components/sceneutil/lightcommon.hpp>
#include <components/sceneutil/visitor.hpp>
#include <components/sceneutil/riggeometry.hpp>
#include <components/sceneutil/morphgeometry.hpp>
#include <components/settings/values.hpp>

#include <components/vfs/manager.hpp>

#include "../mwworld/class.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/inventorystore.hpp"

#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/npcstats.hpp"
#include "../mwmechanics/weapontype.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/soundmanager.hpp"
#include "../mwbase/world.hpp"

#include "actorutil.hpp"
#include "postprocessor.hpp"
#include "renderbin.hpp"
#include "renderingmanager.hpp"
#include "rotatecontroller.hpp"
#include "vismask.hpp"
#include "woundfx.hpp"

namespace
{
    // WOUNDFX-01 FX3: is this armour piece leather-like (a matte, scuffed cut
    // with a frayed edge) rather than metal (a bright scored line and a dent)?
    // A word in its name or model first (leather beats a metal word: studded
    // leather is leather), then the weight class: light armour is
    // leather-like unless it names a hard material (glass, elven...);
    // medium and heavy are metal.
    bool isSoftArmour(const MWWorld::ConstPtr& item)
    {
        const ESM::Armor* armor = item.get<ESM::Armor>()->mBase;
        const std::string_view name = armor->mName;
        const std::string_view model = armor->mModel.getOriginal();
        const auto mentions = [&](std::string_view word) {
            return Misc::StringUtils::ciFind(name, word) != std::string_view::npos
                || Misc::StringUtils::ciFind(model, word) != std::string_view::npos;
        };
        static constexpr std::string_view soft[] = { "leather", "netch", "fur", "hide", "pelt", "bear", "wolf",
            "chitin", "padded", "quilted", "gambeson", "cloth" };
        static constexpr std::string_view hard[] = { "glass", "elven", "mithril", "silver", "steel", "iron", "chain",
            "mail", "plate", "scale", "ebony", "daedric", "dwemer", "dwarven", "orcish", "adamant", "bronze", "metal" };
        for (const std::string_view word : soft)
            if (mentions(word))
                return true;
        for (const std::string_view word : hard)
            if (mentions(word))
                return false;
        return item.getClass().getEquipmentSkill(item) == ESM::Skill::LightArmor;
    }

    // WOUNDFX-01 FX4: the bone a region's skin moves with: a skinned part's
    // frame for its wounds' blood running downhill (a rigid part's is its
    // parent node).
    std::string_view regionBone(std::string_view region)
    {
        static constexpr std::pair<std::string_view, std::string_view> bones[] = {
            { "head", "bip01 head" },
            { "neck", "bip01 neck" },
            { "chest", "bip01 spine2" },
            { "abdomen", "bip01 spine1" },
            { "pelvis", "bip01 pelvis" },
            { "left_clavicle", "bip01 l clavicle" },
            { "right_clavicle", "bip01 r clavicle" },
            { "left_upper_arm", "bip01 l upperarm" },
            { "right_upper_arm", "bip01 r upperarm" },
            { "left_forearm", "bip01 l forearm" },
            { "right_forearm", "bip01 r forearm" },
            { "left_hand", "bip01 l hand" },
            { "right_hand", "bip01 r hand" },
            { "left_thigh", "bip01 l thigh" },
            { "right_thigh", "bip01 r thigh" },
            { "left_calf", "bip01 l calf" },
            { "right_calf", "bip01 r calf" },
            { "left_foot", "bip01 l foot" },
            { "right_foot", "bip01 r foot" },
        };
        for (const auto& [name, bone] : bones)
            if (name == region)
                return bone;
        return {};
    }

    // WOUNDFX-01 FX4: a held weapon's rigid drawables, each with its frame
    // into the bone it hangs on (the weapon node's parent: its origin is the
    // grip).
    class WeaponDrawableVisitor : public osg::NodeVisitor
    {
    public:
        struct Found
        {
            osg::ref_ptr<osg::Geometry> mGeometry;
            osg::ref_ptr<osg::Group> mParent;
            osg::Matrix mToBone;
        };
        std::vector<Found> mFound;
        osg::BoundingBox mBox; // every vertex in the bone's frame

        WeaponDrawableVisitor()
            : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
        {
        }

        void apply(osg::Drawable& drawable) override
        {
            bool isWound = false;
            drawable.getUserValue("localizedWound", isWound);
            osg::Geometry* geometry = drawable.asGeometry();
            const auto* vertices = geometry ? dynamic_cast<const osg::Vec3Array*>(geometry->getVertexArray()) : nullptr;
            const osg::NodePath& path = getNodePath();
            if (isWound || !vertices || vertices->empty() || path.size() < 2)
                return;
            osg::Group* parent = path[path.size() - 2]->asGroup();
            if (!parent)
                return;
            const osg::Matrix toBone = osg::computeLocalToWorld(path);
            for (const osg::Vec3f& vertex : *vertices)
                mBox.expandBy(vertex * toBone);
            mFound.push_back({ geometry, parent, toBone });
        }
    };

    // Search the actual last-rendered surface, not the ragdoll collision box.
    // The resulting UV projector is baked per vertex; the overlay then uses
    // precisely the original mesh's weights and topology during skinning.
    struct WoundSurface
    {
        osg::Vec3f mPoint, mNormal, mHit;
        float mDistance = 12.0f;
        bool mFound = false;
        void operator()(const osg::Vec3& a, const osg::Vec3& b, const osg::Vec3& c, bool = false)
        {
            const osg::Vec3 e1 = b - a, e2 = c - a;
            osg::Vec3 n = e1 ^ e2;
            const float normalLength = n.normalize();
            if (normalLength < 1e-7f || n * mNormal < 0.15f)
                return;
            // Prefer projection along the physical contact normal. Thin limbs
            // can sit beside that line inside their collision shape, so use
            // the closest point on a facing triangle when the line misses.
            // Both paths retain the same 12-unit Euclidean distance bound.
            const osg::Vec3 p = mNormal ^ e2;
            const float det = e1 * p;
            osg::Vec3 hit;
            bool onRay = false;
            if (std::abs(det) >= 1e-7f)
            {
                const osg::Vec3 s = mPoint - a;
                const float u = (s * p) / det;
                const osg::Vec3 q = s ^ e1;
                const float v = (mNormal * q) / det;
                const float t = (e2 * q) / det;
                onRay = u >= 0 && v >= 0 && u + v <= 1;
                if (onRay) hit = mPoint + mNormal * t;
            }
            if (!onRay)
            {
                hit = mPoint - n * ((mPoint - a) * n);
                const bool inside = ((b - a) ^ (hit - a)) * n >= 0
                    && ((c - b) ^ (hit - b)) * n >= 0 && ((a - c) ^ (hit - c)) * n >= 0;
                if (!inside)
                {
                    float best = std::numeric_limits<float>::max();
                    for (const auto& edge : {std::pair{a, b}, std::pair{b, c}, std::pair{c, a}})
                    {
                        const osg::Vec3 direction = edge.second - edge.first;
                        const float square = direction.length2();
                        const float t = square > 1e-8f
                            ? std::clamp(((mPoint - edge.first) * direction) / square, 0.0f, 1.0f) : 0.0f;
                        const osg::Vec3 candidate = edge.first + direction * t;
                        const float distance = (candidate - mPoint).length2();
                        if (distance < best) { best = distance; hit = candidate; }
                    }
                }
            }
            const float distance = (hit - mPoint).length();
            if (distance >= mDistance)
                return;
            mDistance = distance;
            mHit = hit;
            mFound = true;
        }
    };

    class WoundSurfaceVisitor : public osg::NodeVisitor
    {
    public:
        osg::Vec3f mPoint, mNormal;
        float mDistance = 12.0f;
        osg::ref_ptr<osg::Drawable> mDrawable;
        osg::ref_ptr<osg::Group> mParent;
        osg::ref_ptr<osg::Geometry> mPosed;
        osg::Matrix mMatrix;
        osg::Vec3f mHit;
        // WOUNDFX-01 FX2: every bare-skin drawable of the region, for a wound
        // spilling over the struck drawable's edge (the vanilla forearm ends
        // mid-arm and the wrist piece goes on).
        struct Candidate
        {
            osg::ref_ptr<osg::Drawable> mDrawable;
            osg::ref_ptr<osg::Group> mParent;
            osg::ref_ptr<osg::Geometry> mPosed;
            osg::Matrix mMatrix;
            int mSurface = 0; // MWRender::WoundFxSet::Surface
        };
        bool mCollect = false;
        int mSurface = 0; // the surface of the part being visited (FX3)
        std::vector<Candidate> mCandidates;
        WoundSurfaceVisitor(const osg::Vec3f& point, const osg::Vec3f& normal)
            : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), mPoint(point), mNormal(normal) {}
        void apply(osg::Drawable& drawable) override
        {
            bool isWound = false;
            drawable.getUserValue("localizedWound", isWound);
            if (isWound)
                return;
            auto* rig = dynamic_cast<SceneUtil::RigGeometry*>(&drawable);
            auto* morph = dynamic_cast<SceneUtil::MorphGeometry*>(&drawable);
            osg::Geometry* plain = drawable.asGeometry();
            if (!rig && !morph && !plain)
                return;
            osg::ref_ptr<osg::Geometry> posed = rig ? rig->copyDeformedGeometry()
                : morph ? morph->copyDeformedGeometry() : osg::ref_ptr<osg::Geometry>(plain);
            const auto* vertices = dynamic_cast<const osg::Vec3Array*>(posed->getVertexArray());
            const auto& path = getNodePath();
            if (!vertices || path.empty())
                return;
            // Static body-part drawables are shared by multiple actors. Use
            // this part's traversal path, never another instance's parent.
            const auto roots = path.front()->getParentalNodePaths();
            if (roots.empty())
                return;
            auto worldPath = roots.front();
            worldPath.insert(worldPath.end(), path.begin() + 1, path.end());
            auto* parent = worldPath.size() >= 2 ? worldPath[worldPath.size() - 2]->asGroup() : nullptr;
            if (!parent)
                return;
            const osg::Matrix matrix = osg::computeLocalToWorld(worldPath);
            if (mCollect)
                mCandidates.push_back({ &drawable, parent, posed, matrix, mSurface });
            const osg::Matrix inverse = osg::Matrix::inverse(matrix);
            osg::TriangleFunctor<WoundSurface> query;
            query.mPoint = mPoint * inverse;
            query.mNormal = osg::Matrix::transform3x3(mNormal, inverse);
            const float scale = query.mNormal.normalize();
            if (scale <= 0)
                return;
            query.mDistance = mDistance * scale;
            posed->accept(query);
            if (!query.mFound)
                return;
            mDistance = query.mDistance / scale;
            mHit = query.mHit;
            mMatrix = matrix;
            mDrawable = &drawable;
            mParent = parent;
            mPosed = posed;
        }
    };

    osg::ref_ptr<osg::StateSet> woundState(int kind, float severity)
    {
        static const osg::ref_ptr<osg::Program> sharedProgram = [] {
        osg::ref_ptr<osg::Program> program = new osg::Program;
        program->addShader(new osg::Shader(osg::Shader::VERTEX, R"glsl(
#version 120
uniform mat4 projectionMatrix;
varying vec4 woundCoord;
void main() {
    // OpenMW's projection (reversed depth): the fixed-function matrix put
    // the marks in front of everything (user 2026-10-01).
    gl_Position = projectionMatrix * (gl_ModelViewMatrix * gl_Vertex);
    woundCoord = gl_MultiTexCoord0;
}
)glsl"));
        program->addShader(new osg::Shader(osg::Shader::FRAGMENT, R"glsl(
#version 120
varying vec4 woundCoord;
uniform int woundKind;
uniform float woundStrength;
uniform float alpha;
uniform float actorFade;
void main() {
    vec2 p = woundCoord.xy;
    if (abs(woundCoord.z) > 1.8 || woundCoord.w < 0.15) discard;
    float edge = 1.0 + 0.07 * sin(p.x * 19.0 + p.y * 11.0) + 0.035 * sin(p.y * 31.0);
    float r = length(p);
    float opacity = (1.0 - smoothstep(0.48, edge, r)) * (0.60 + 0.30 * woundStrength);
    vec3 color;
    if (woundKind == 1) {
        color = mix(vec3(0.22, 0.12, 0.21), vec3(0.10, 0.045, 0.12), 1.0 - min(r, 1.0));
        opacity *= 0.8;
    } else if (woundKind == 4) {
        color = vec3(0.28, 0.025, 0.035); // blood stain on cloth, no open flesh
        opacity *= 0.8;
    } else {
        float slit = 1.0 - smoothstep(0.12, 0.48, abs(p.y + 0.035 * sin(p.x * 15.0)));
        color = mix(vec3(0.42, 0.025, 0.04), vec3(0.08, 0.012, 0.018), slit);
    }
    opacity *= alpha * actorFade;
    if (opacity < 0.015) discard;
    gl_FragColor = vec4(color, opacity);
}
)glsl"));
        return program;
        }();
        osg::ref_ptr<osg::StateSet> state = new osg::StateSet;
        const auto mode = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED;
        state->setAttributeAndModes(sharedProgram, mode);
        state->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), mode);
        state->setAttributeAndModes(new SceneUtil::AutoDepth(osg::Depth::LEQUAL, 0, 1, false), mode);
        const float bias = SceneUtil::AutoDepth::isReversed() ? 1.0f : -1.0f;
        state->setAttributeAndModes(new osg::PolygonOffset(bias, bias), mode);
        state->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
        state->addUniform(new osg::Uniform("woundKind", kind));
        state->addUniform(new osg::Uniform("woundStrength", severity));
        state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        return state;
    }

    VFS::Path::Normalized getVampireHead(ESM::RefId race, bool female)
    {
        static std::map<std::pair<ESM::RefId, bool>, const ESM::BodyPart*> sVampireMapping;

        const std::pair<ESM::RefId, bool> key(race, female);
        const ESM::BodyPart* vampireHead = nullptr;

        if (const auto it = sVampireMapping.find(key); it == sVampireMapping.end())
        {
            const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();

            for (const ESM::BodyPart& bodyPart : store.get<ESM::BodyPart>())
            {
                if (!bodyPart.mData.mVampire)
                    continue;
                if (bodyPart.mData.mType != ESM::BodyPart::MT_Skin)
                    continue;
                if (bodyPart.mData.mPart != ESM::BodyPart::MP_Head)
                    continue;
                if (female != (bodyPart.mData.mFlags & ESM::BodyPart::BPF_Female))
                    continue;
                if (!(bodyPart.mRace == race))
                    continue;

                vampireHead = &bodyPart;
            }

            sVampireMapping.emplace_hint(it, key, vampireHead);
        }
        else
        {
            vampireHead = it->second;
        }

        if (vampireHead == nullptr)
            return {};

        return Misc::ResourceHelpers::correctMeshPath(vampireHead->mModel.getNormalized());
    }

}

namespace MWRender
{
    class HeadAnimationTime : public SceneUtil::ControllerSource
    {
    private:
        MWWorld::Ptr mReference;
        float mTalkStart;
        float mTalkStop;
        float mBlinkStart;
        float mBlinkStop;

        float mBlinkTimer;

        bool mEnabled;

        float mValue;

    private:
        void resetBlinkTimer();

    public:
        HeadAnimationTime(const MWWorld::Ptr& reference);

        void updatePtr(const MWWorld::Ptr& updated);

        void update(float dt);

        void setEnabled(bool enabled);

        void setTalkStart(float value);
        void setTalkStop(float value);
        void setBlinkStart(float value);
        void setBlinkStop(float value);

        float getValue(osg::NodeVisitor* nv) override;
    };

    // --------------------------------------------------------------------------------------------------------------

    HeadAnimationTime::HeadAnimationTime(const MWWorld::Ptr& reference)
        : mReference(reference)
        , mTalkStart(0)
        , mTalkStop(0)
        , mBlinkStart(0)
        , mBlinkStop(0)
        , mEnabled(true)
        , mValue(0)
    {
        resetBlinkTimer();
    }

    void HeadAnimationTime::updatePtr(const MWWorld::Ptr& updated)
    {
        mReference = updated;
    }

    void HeadAnimationTime::setEnabled(bool enabled)
    {
        mEnabled = enabled;
    }

    void HeadAnimationTime::resetBlinkTimer()
    {
        auto& prng = MWBase::Environment::get().getWorld()->getPrng();
        mBlinkTimer = -(2.0f + Misc::Rng::rollDice(6, prng));
    }

    void HeadAnimationTime::update(float dt)
    {
        if (!mEnabled)
            return;

        if (dt == 0.f)
            return;

        if (!MWBase::Environment::get().getSoundManager()->sayActive(mReference))
        {
            mBlinkTimer += dt;

            float duration = mBlinkStop - mBlinkStart;

            if (mBlinkTimer >= 0 && mBlinkTimer <= duration)
            {
                mValue = mBlinkStart + mBlinkTimer;
            }
            else
                mValue = mBlinkStop;

            if (mBlinkTimer > duration)
                resetBlinkTimer();
        }
        else
        {
            // FIXME: would be nice to hold on to the SoundPtr so we don't have to retrieve it every frame
            mValue = mTalkStart
                + (mTalkStop - mTalkStart)
                    * std::min(1.f,
                        MWBase::Environment::get().getSoundManager()->getSaySoundLoudness(mReference)
                            * 2); // Rescale a bit (most voices are not very loud)
        }
    }

    float HeadAnimationTime::getValue(osg::NodeVisitor*)
    {
        return mValue;
    }

    void HeadAnimationTime::setTalkStart(float value)
    {
        mTalkStart = value;
    }

    void HeadAnimationTime::setTalkStop(float value)
    {
        mTalkStop = value;
    }

    void HeadAnimationTime::setBlinkStart(float value)
    {
        mBlinkStart = value;
    }

    void HeadAnimationTime::setBlinkStop(float value)
    {
        mBlinkStop = value;
    }

    // ----------------------------------------------------

    NpcAnimation::NpcType NpcAnimation::getNpcType() const
    {
        const MWWorld::Class& cls = mPtr.getClass();
        // Dead vampires should typically stay vampires.
        if (mNpcType == Type_Vampire && cls.getNpcStats(mPtr).isDead() && !cls.getNpcStats(mPtr).isWerewolf())
            return mNpcType;
        return getNpcType(mPtr);
    }

    NpcAnimation::NpcType NpcAnimation::getNpcType(const MWWorld::Ptr& ptr)
    {
        const MWWorld::Class& cls = ptr.getClass();
        NpcAnimation::NpcType curType = Type_Normal;
        if (cls.getCreatureStats(ptr).getMagicEffects().getOrDefault(ESM::MagicEffect::Vampirism).getMagnitude() > 0)
            curType = Type_Vampire;
        if (cls.getNpcStats(ptr).isWerewolf())
            curType = Type_Werewolf;

        return curType;
    }

    static const inline NpcAnimation::PartBoneMap createPartListMap()
    {
        return { { ESM::PRT_Head, "Head" },
            { ESM::PRT_Hair, "Head" }, // note it uses "Head" as attach bone, but "Hair" as filter
            { ESM::PRT_Neck, "Neck" }, { ESM::PRT_Cuirass, "Chest" }, { ESM::PRT_Groin, "Groin" },
            { ESM::PRT_Skirt, "Groin" }, { ESM::PRT_RHand, "Right Hand" }, { ESM::PRT_LHand, "Left Hand" },
            { ESM::PRT_RWrist, "Right Wrist" }, { ESM::PRT_LWrist, "Left Wrist" }, { ESM::PRT_Shield, "Shield Bone" },
            { ESM::PRT_RForearm, "Right Forearm" }, { ESM::PRT_LForearm, "Left Forearm" },
            { ESM::PRT_RUpperarm, "Right Upper Arm" }, { ESM::PRT_LUpperarm, "Left Upper Arm" },
            { ESM::PRT_RFoot, "Right Foot" }, { ESM::PRT_LFoot, "Left Foot" }, { ESM::PRT_RAnkle, "Right Ankle" },
            { ESM::PRT_LAnkle, "Left Ankle" }, { ESM::PRT_RKnee, "Right Knee" }, { ESM::PRT_LKnee, "Left Knee" },
            { ESM::PRT_RLeg, "Right Upper Leg" }, { ESM::PRT_LLeg, "Left Upper Leg" },
            { ESM::PRT_RPauldron, "Right Clavicle" }, { ESM::PRT_LPauldron, "Left Clavicle" },
            { ESM::PRT_Weapon, "Weapon Bone" }, // Fallback. The real node name depends on the current weapon type.
            { ESM::PRT_Tail, "Tail" } };
    }
    const NpcAnimation::PartBoneMap NpcAnimation::sPartList = createPartListMap();

    NpcAnimation::~NpcAnimation()
    {
        mAmmunition.reset();
    }

    NpcAnimation::NpcAnimation(const MWWorld::Ptr& ptr, osg::ref_ptr<osg::Group> parentNode,
        Resource::ResourceSystem* resourceSystem, bool disableSounds, ViewMode viewMode, float firstPersonFieldOfView)
        : ActorAnimation(ptr, std::move(parentNode), resourceSystem)
        , mViewMode(viewMode)
        , mShowWeapons(false)
        , mShowCarriedLeft(true)
        , mNpcType(getNpcType(ptr))
        , mFirstPersonFieldOfView(firstPersonFieldOfView)
        , mSoundsDisabled(disableSounds)
        , mAccurateAiming(false)
        , mAimingFactor(0.f)
    {
        mNpc = mPtr.get<ESM::NPC>()->mBase;

        mHeadAnimationTime = std::make_shared<HeadAnimationTime>(mPtr);
        mWeaponAnimationTime = std::make_shared<WeaponAnimationTime>(this);

        for (size_t i = 0; i < ESM::PRT_Count; i++)
        {
            mPartslots[i] = -1; // each slot is empty
            mPartPriorities[i] = 0;
        }

        std::fill(mSounds.begin(), mSounds.end(), nullptr);

        updateNpcBase();
    }

    void NpcAnimation::setViewMode(NpcAnimation::ViewMode viewMode)
    {
        assert(viewMode != VM_HeadOnly);
        if (mViewMode == viewMode)
            return;
        // Both views share the complete skeleton and its physical binding.
        // Rebuilding on a camera toggle needlessly releases support grips and
        // restarts active attacks. Only the visible head and traversal mask change.
        bool viewChange = mViewMode == VM_FirstPerson || viewMode == VM_FirstPerson;
        mViewMode = viewMode;
        mObjectRoot->setNodeMask(mViewMode == VM_FirstPerson ? Mask_FirstPerson : ~0u);
        for (const auto part : { ESM::PRT_Head, ESM::PRT_Hair, ESM::PRT_Neck })
            if (mObjectParts[part])
                mObjectParts[part]->getNode()->setNodeMask(mViewMode == VM_FirstPerson ? 0u : ~0u);
        setRenderBin();
        applyNearDissolve();

        if (viewChange && Settings::game().mShieldSheathing)
        {
            ESM::RefId weaponTypeId;
            MWMechanics::getActiveWeapon(mPtr, &weaponTypeId);
            showCarriedLeft(updateCarriedLeftVisible(weaponTypeId));
        }
    }

    /// @brief A RenderBin callback to clear the depth buffer before rendering.
    /// Switches depth attachments to a proxy renderbuffer, reattaches original depth then redraws first person root.
    /// This gives a complete depth buffer which can be used for postprocessing, buffer resolves as if depth was never
    /// cleared.
    class DepthClearCallback : public osgUtil::RenderBin::DrawCallback
    {
    public:
        DepthClearCallback()
        {
            mDepth = new SceneUtil::AutoDepth;
            mDepth->setWriteMask(true);

            mStateSet = new osg::StateSet;
            mStateSet->setAttributeAndModes(new osg::ColorMask(false, false, false, false), osg::StateAttribute::ON);
        }

        void drawImplementation(
            osgUtil::RenderBin* bin, osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous) override
        {
            osg::State* state = renderInfo.getState();

            PostProcessor* postProcessor = static_cast<PostProcessor*>(renderInfo.getCurrentCamera()->getUserData());

            state->applyAttribute(mDepth);

            unsigned int frameId = state->getFrameStamp()->getFrameNumber() % 2;

            postProcessor->getFbo(PostProcessor::FBO_FirstPerson, frameId)->apply(*state);
            glClear(GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
            // color accumulation pass
            bin->drawImplementation(renderInfo, previous);

            auto primaryFBO = postProcessor->getPrimaryFbo(frameId);
            primaryFBO->apply(*state);

            postProcessor->getFbo(PostProcessor::FBO_OpaqueDepth, frameId)->apply(*state);

            // depth accumulation pass
            osg::ref_ptr<osg::StateSet> restore = bin->getStateSet();
            bin->setStateSet(mStateSet);
            bin->drawImplementation(renderInfo, previous);
            bin->setStateSet(restore);

            primaryFBO->apply(*state);

            state->checkGLErrors("after DepthClearCallback::drawImplementation");
        }

        osg::ref_ptr<osg::Depth> mDepth;
        osg::ref_ptr<osg::StateSet> mStateSet;
    };

    /// Overrides Field of View to given value for rendering the subgraph.
    /// Must be added as cull callback.
    class OverrideFieldOfViewCallback : public osg::NodeCallback
    {
    public:
        OverrideFieldOfViewCallback(float fov, RenderingManager* renderingManager)
            : mFov(fov)
            , mRenderingManager(renderingManager)
        {
        }

        void operator()(osg::Node* node, osg::NodeVisitor* nv) override
        {
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(nv);
            float fov, aspect, zNear, zFar;
            if (cv->getProjectionMatrix()->getPerspective(fov, aspect, zNear, zFar) && std::abs(fov - mFov) > 0.001)
            {
                fov = mFov;
                osg::ref_ptr<osg::RefMatrix> newProjectionMatrix = new osg::RefMatrix();
                newProjectionMatrix->makePerspective(fov, aspect, zNear, zFar);

                osg::Vec2f offset = mRenderingManager->getProjectionOffset();

                double offsetX = (offset.x() / cv->getViewport()->width()) * 2.0;
                double offsetY = (offset.y() / cv->getViewport()->height()) * 2.0;

                const osg::Matrix translation = osg::Matrix::translate(offsetX, offsetY, 0.0);
                newProjectionMatrix->postMult(translation);

                osg::ref_ptr<osg::RefMatrix> invertedOldMatrix = cv->getProjectionMatrix();
                invertedOldMatrix = new osg::RefMatrix(osg::RefMatrix::inverse(*invertedOldMatrix));
                osg::ref_ptr<osg::RefMatrix> viewMatrix = new osg::RefMatrix(*cv->getModelViewMatrix());
                viewMatrix->postMult(*newProjectionMatrix);
                viewMatrix->postMult(*invertedOldMatrix);
                cv->pushModelViewMatrix(viewMatrix, osg::Transform::ReferenceFrame::ABSOLUTE_RF);
                traverse(node, nv);
                cv->popModelViewMatrix();
            }
            else
                traverse(node, nv);
        }

    private:
        float mFov;
        RenderingManager* mRenderingManager;
    };

    void NpcAnimation::setRenderBin()
    {
        // The first-person body occupies the same world space as the actor.
        // Clearing depth would draw the legs and torso through nearby scenery.
        if (osg::StateSet* stateset = mObjectRoot->getStateSet())
            stateset->setRenderBinToInherit();
    }

    void NpcAnimation::applyNearDissolve()
    {
        // KINETIC (10-09, user: "we need some way to also make the character
        // model not block the player vision ... dissolve the body"): in first
        // person the player's own arms, hands and weapon dissolve with a
        // screen-door dither where they come close to the eye, more so near
        // the middle of the view. OPENMW_RAGDOLL_FP_DISSOLVE = "gone, solid,
        // centre" (world units, units, extra share of solid toward the
        // centre); 0 = off. Uniform defaults to off at the scene root.
        static const auto settings_tuned = RagdollCore::tuned<osg::Vec3f>([] {
            osg::Vec3f value(10.0f, 22.0f, 0.8f);
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_FP_DISSOLVE"))
            {
                float a = 0, b = 0, c = 0;
                const int read = std::sscanf(text, "%f,%f,%f", &a, &b, &c);
                value = read >= 2 && b > a ? osg::Vec3f(a, b, read == 3 ? c : 0.0f) : osg::Vec3f();
            }
            return value;
        });
        static const auto& settings = settings_tuned.get();
        if (!mObjectRoot)
            return;
        const osg::Vec3f value = mViewMode == VM_FirstPerson ? settings : osg::Vec3f();
        osg::StateSet* stateset = mObjectRoot->getOrCreateStateSet();
        if (osg::Uniform* uniform = stateset->getUniform("nearDissolve"))
            uniform->set(value);
        else if (value != osg::Vec3f())
        {
            osg::ref_ptr<osg::Uniform> uniform = new osg::Uniform("nearDissolve", value);
            uniform->setDataVariance(osg::Object::DYNAMIC);
            stateset->addUniform(uniform);
        }
    }

    void NpcAnimation::rebuild()
    {
        mScabbard.reset();
        mHolsteredShield.reset();
        updateNpcBase();

        MWBase::Environment::get().getMechanicsManager()->forceStateUpdate(mPtr);
    }

    int NpcAnimation::getSlot(const osg::NodePath& path) const
    {
        for (int i = 0; i < ESM::PRT_Count; ++i)
        {
            const PartHolder* const part = mObjectParts[i].get();
            if (part == nullptr)
                continue;
            if (std::find(path.begin(), path.end(), part->getNode().get()) != path.end())
            {
                return mPartslots[i];
            }
        }
        return -1;
    }

    void NpcAnimation::updateNpcBase()
    {
        clearAnimSources();
        for (size_t i = 0; i < ESM::PRT_Count; i++)
            removeIndividualPart((ESM::PartReferenceType)i);

        const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();
        const ESM::Race* race = store.get<ESM::Race>().find(mNpc->mRace);
        NpcType curType = getNpcType();
        bool isWerewolf = (curType == Type_Werewolf);
        bool isVampire = (curType == Type_Vampire);
        bool isFemale = !mNpc->isMale();

        mHeadModel.clear();
        mHairModel.clear();

        const ESM::RefId headName = isWerewolf ? ESM::RefId::stringRefId("WerewolfHead") : mNpc->mHead;
        const ESM::RefId hairName = isWerewolf ? ESM::RefId::stringRefId("WerewolfHair") : mNpc->mHair;

        if (!headName.empty())
        {
            const ESM::BodyPart* bp = store.get<ESM::BodyPart>().search(headName);
            if (bp)
                mHeadModel = Misc::ResourceHelpers::correctMeshPath(bp->mModel.getNormalized());
            else
                Log(Debug::Warning) << "Warning: Failed to load body part '" << headName << "'";
        }

        if (!hairName.empty())
        {
            const ESM::BodyPart* bp = store.get<ESM::BodyPart>().search(hairName);
            if (bp)
                mHairModel = Misc::ResourceHelpers::correctMeshPath(bp->mModel.getNormalized());
            else
                Log(Debug::Warning) << "Warning: Failed to load body part '" << hairName << "'";
        }

        if (!isWerewolf && isVampire)
            if (VFS::Path::Normalized vampireHead = getVampireHead(mNpc->mRace, isFemale); !vampireHead.empty())
                mHeadModel = std::move(vampireHead);

        // Both perspectives use the complete animated skeleton. The arms-only
        // skeleton has neither a usable lower-body pose nor matching body skins.
        bool isBeast = (race->mData.mFlags & ESM::Race::Beast) != 0;

        std::string_view base;
        if (!isWerewolf)
            base = Settings::models().mXbaseanim.get().value();

        const std::string defaultSkeleton = Misc::ResourceHelpers::correctActorModelPath(
            VFS::Path::toNormalized(getActorSkeleton(false, isFemale, isBeast, isWerewolf)),
            mResourceSystem->getVFS());

        std::string smodel = defaultSkeleton;
        bool isCustomModel = false;
        if (!isWerewolf && !mNpc->mModel.empty())
        {
            const VFS::Path::Normalized model = Misc::ResourceHelpers::correctMeshPath(mNpc->mModel.getNormalized());
            isCustomModel = !isDefaultActorSkeleton(model);
            smodel = Misc::ResourceHelpers::correctActorModelPath(model, mResourceSystem->getVFS());
        }

        setObjectRoot(smodel, true, true, false);

        updateParts();

        if (!base.empty())
            addAnimSource(base, smodel);

        if (defaultSkeleton != base)
            addAnimSource(defaultSkeleton, smodel);

        if (isCustomModel)
            addAnimSource(smodel, smodel);

        const bool customArgonianSwim = !isWerewolf && isBeast && mNpc->mRace.contains("argonian");
        if (customArgonianSwim)
            addAnimSource(Settings::models().mXargonianswimkna.get().value(), smodel);

        if (mViewMode == VM_FirstPerson)
            mObjectRoot->setNodeMask(Mask_FirstPerson);
        applyNearDissolve();

        mWeaponAnimationTime->updateStartTime();
    }

    std::string NpcAnimation::getSheathedShieldMesh(const MWWorld::ConstPtr& shield) const
    {
        std::string mesh = getShieldMesh(shield, !mNpc->isMale());

        if (mesh.empty())
            return std::string();

        const VFS::Path::Normalized holsteredName(addSuffixBeforeExtension(mesh, "_sh"));
        if (mResourceSystem->getVFS()->exists(holsteredName))
        {
            osg::ref_ptr<osg::Node> shieldTemplate = mResourceSystem->getSceneManager()->getInstance(holsteredName);
            SceneUtil::FindByNameVisitor findVisitor("Bip01 Sheath");
            shieldTemplate->accept(findVisitor);
            osg::ref_ptr<osg::Node> sheathNode = findVisitor.mFoundNode;
            if (!sheathNode)
                return std::string();
        }

        return mesh;
    }

    void NpcAnimation::updateParts()
    {
        if (!mObjectRoot.get())
            return;

        NpcType curType = getNpcType();
        if (curType != mNpcType)
        {
            mNpcType = curType;
            rebuild();
            return;
        }

        static const struct
        {
            int mSlot;
            int mBasePriority;
        } slotlist[] = { // FIXME: Priority is based on the number of reserved slots. There should be a better way.
            { MWWorld::InventoryStore::Slot_Robe, 11 }, { MWWorld::InventoryStore::Slot_Skirt, 3 },
            { MWWorld::InventoryStore::Slot_Helmet, 0 }, { MWWorld::InventoryStore::Slot_Cuirass, 0 },
            { MWWorld::InventoryStore::Slot_Greaves, 0 }, { MWWorld::InventoryStore::Slot_LeftPauldron, 0 },
            { MWWorld::InventoryStore::Slot_RightPauldron, 0 }, { MWWorld::InventoryStore::Slot_Boots, 0 },
            { MWWorld::InventoryStore::Slot_LeftGauntlet, 0 }, { MWWorld::InventoryStore::Slot_RightGauntlet, 0 },
            { MWWorld::InventoryStore::Slot_Shirt, 0 }, { MWWorld::InventoryStore::Slot_Pants, 0 },
            { MWWorld::InventoryStore::Slot_CarriedLeft, 0 }, { MWWorld::InventoryStore::Slot_CarriedRight, 0 }
        };
        static const size_t slotlistsize = sizeof(slotlist) / sizeof(slotlist[0]);

        bool wasArrowAttached = isArrowAttached();
        mAmmunition.reset();

        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        for (size_t i = 0; i < slotlistsize && mViewMode != VM_HeadOnly; i++)
        {
            MWWorld::ConstContainerStoreIterator store = inv.getSlot(slotlist[i].mSlot);

            removePartGroup(slotlist[i].mSlot);

            if (store == inv.end())
                continue;

            if (slotlist[i].mSlot == MWWorld::InventoryStore::Slot_Helmet)
                removeIndividualPart(ESM::PRT_Hair);

            int prio = 1;
            bool enchantedGlow = !store->getClass().getEnchantment(*store).empty();
            osg::Vec4f glowColor = store->getClass().getEnchantmentColor(*store);
            if (store->getType() == ESM::Clothing::sRecordId)
            {
                prio = ((slotlist[i].mBasePriority + 1) << 1) + 0;
                const ESM::Clothing* clothes = store->get<ESM::Clothing>()->mBase;
                addPartGroup(slotlist[i].mSlot, prio, clothes->mParts.mParts, enchantedGlow, &glowColor);
            }
            else if (store->getType() == ESM::Armor::sRecordId)
            {
                prio = ((slotlist[i].mBasePriority + 1) << 1) + 1;
                const ESM::Armor* armor = store->get<ESM::Armor>()->mBase;
                addPartGroup(slotlist[i].mSlot, prio, armor->mParts.mParts, enchantedGlow, &glowColor);
            }

            if (slotlist[i].mSlot == MWWorld::InventoryStore::Slot_Robe)
            {
                ESM::PartReferenceType parts[] = { ESM::PRT_Groin, ESM::PRT_Skirt, ESM::PRT_RLeg, ESM::PRT_LLeg,
                    ESM::PRT_RUpperarm, ESM::PRT_LUpperarm, ESM::PRT_RKnee, ESM::PRT_LKnee, ESM::PRT_RForearm,
                    ESM::PRT_LForearm, ESM::PRT_Cuirass };
                const size_t partsSize = sizeof(parts) / sizeof(parts[0]);
                for (size_t p = 0; p < partsSize; ++p)
                    reserveIndividualPart(parts[p], slotlist[i].mSlot, prio);
            }
            else if (slotlist[i].mSlot == MWWorld::InventoryStore::Slot_Skirt)
            {
                reserveIndividualPart(ESM::PRT_Groin, slotlist[i].mSlot, prio);
                reserveIndividualPart(ESM::PRT_RLeg, slotlist[i].mSlot, prio);
                reserveIndividualPart(ESM::PRT_LLeg, slotlist[i].mSlot, prio);
            }
        }

        {
            if (mPartPriorities[ESM::PRT_Head] < 1 && !mHeadModel.empty())
                addOrReplaceIndividualPart(ESM::PRT_Head, -1, 1, mHeadModel);
            if (mPartPriorities[ESM::PRT_Hair] < 1 && mPartPriorities[ESM::PRT_Head] <= 1 && !mHairModel.empty())
                addOrReplaceIndividualPart(ESM::PRT_Hair, -1, 1, mHairModel);
        }
        if (mViewMode == VM_HeadOnly)
            return;

        if (mPartPriorities[ESM::PRT_Shield] < 1)
        {
            MWWorld::ConstContainerStoreIterator store = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
            MWWorld::ConstPtr part;
            if (store != inv.end() && (part = *store).getType() == ESM::Light::sRecordId)
            {
                const ESM::Light* light = part.get<ESM::Light>()->mBase;
                addOrReplaceIndividualPart(ESM::PRT_Shield, MWWorld::InventoryStore::Slot_CarriedLeft, 1,
                    Misc::ResourceHelpers::correctMeshPath(light->mModel.getNormalized()), false, nullptr, true);
                if (mObjectParts[ESM::PRT_Shield])
                    addExtraLight(mObjectParts[ESM::PRT_Shield]->getNode()->asGroup(), SceneUtil::LightCommon(*light));
            }
        }

        showWeapons(mShowWeapons);
        showCarriedLeft(mShowCarriedLeft);

        bool isWerewolf = (getNpcType() == Type_Werewolf);
        ESM::RefId race = (isWerewolf ? ESM::RefId::stringRefId("werewolf") : mNpc->mRace);

        const std::vector<const ESM::BodyPart*>& parts
            = getBodyParts(race, !mNpc->isMale(), false, isWerewolf);
        for (int part = ESM::PRT_Neck; part < ESM::PRT_Count; ++part)
        {
            if (mPartPriorities[part] < 1)
            {
                if (const ESM::BodyPart* bodypart = parts[part])
                    addOrReplaceIndividualPart(static_cast<ESM::PartReferenceType>(part), -1, 1,
                        Misc::ResourceHelpers::correctMeshPath(bodypart->mModel.getNormalized()));
            }
        }

        if (wasArrowAttached)
            attachArrow();
    }

    PartHolderPtr NpcAnimation::insertBoundedPart(VFS::Path::NormalizedView model, std::string_view bonename,
        std::string_view bonefilter, bool enchantedGlow, osg::Vec4f* glowColor, bool isLight)
    {
        osg::ref_ptr<osg::Node> attached = attach(model, bonename, bonefilter, isLight);
        if (enchantedGlow)
            mGlowUpdater = SceneUtil::addEnchantedGlow(attached, mResourceSystem, *glowColor);

        return std::make_unique<PartHolder>(std::move(attached));
    }

    osg::Vec3f NpcAnimation::runAnimation(float timepassed)
    {
        // Live tuning: the options page moved a slider (FP_DISSOLVE).
        if (const std::uint64_t generation = RagdollCore::tuningGeneration(); generation != mDissolveTuning)
        {
            mDissolveTuning = generation;
            applyNearDissolve();
        }
        // Wounds queued off the main thread (FX on): placed here.
        std::vector<PendingWound> pending;
        std::vector<PendingWeaponBlood> pendingBlood;
        {
            std::lock_guard<std::mutex> lock(mPendingWoundsMutex);
            pending.swap(mPendingWounds);
            pendingBlood.swap(mPendingWeaponBlood);
        }
        for (const PendingWound& wound : pending)
            placeWound(wound.mRegion, wound.mKind, wound.mPoint, wound.mNormal, wound.mTangent, wound.mSeverity,
                0.0f, 0.0f);
        // FX4: blood on the held weapon, queued like the wounds.
        for (const PendingWeaponBlood& blood : pendingBlood)
            placeWeaponBlood(blood);
        // Restore a rebuilt part's marks two frames later, once it is posed.
        for (int part = 0; part < ESM::PRT_Count; ++part)
            if (mWoundRestoreFrames[part] > 0 && --mWoundRestoreFrames[part] == 0)
            {
                restoreWoundSeeds(static_cast<ESM::PartReferenceType>(part));
                // FX4: a weapon drawn again gets its blood back.
                if (part == ESM::PRT_Weapon)
                    applyWeaponBlood(nullptr);
            }
        // FX4: fresh wounds' blood keeps running downhill.
        if (mWoundFx)
            mWoundFx->update();
        osg::Vec3f ret = Animation::runAnimation(timepassed);

        mHeadAnimationTime->update(timepassed);

        if (mFirstPersonNeckController)
        {
            if (mAccurateAiming)
                mAimingFactor = 1.f;
            else
                mAimingFactor = std::max(0.f, mAimingFactor - timepassed * 0.5f);

            float rotateFactor = 0.75f + 0.25f * mAimingFactor;

            mFirstPersonNeckController->setRotate(
                osg::Quat(mPtr.getRefData().getPosition().rot[0] * rotateFactor, osg::Vec3f(-1, 0, 0)));
            mFirstPersonNeckController->setOffset(mFirstPersonOffset);
        }

        WeaponAnimation::configureControllers(mPtr.getRefData().getPosition().rot[0] + getBodyPitchRadians());

        static const auto partTrace_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_FP_PART_TRACE") != nullptr; });
        static const auto& partTrace = partTrace_tuned.get();
        if (partTrace && mViewMode == VM_FirstPerson && ++mPartTraceCounter % 30 == 0)
        {
            struct RigLister : osg::NodeVisitor
            {
                RigLister() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) {}
                void apply(osg::Drawable& drawable) override
                {
                    const osg::BoundingBox box = drawable.getBoundingBox();
                    mText += " [" + drawable.getName() + " " + drawable.className()
                        + " mask=" + std::to_string(drawable.getNodeMask()) + " box="
                        + (box.valid() ? std::to_string(box.center().x()) + "," + std::to_string(box.center().y()) + ","
                                    + std::to_string(box.center().z()) + " r=" + std::to_string(box.radius())
                                       : std::string("invalid"))
                        + "]";
                }
                std::string mText;
            };
            const auto worldOf = [](osg::Node* node) {
                const osg::MatrixList matrices = node->getWorldMatrices();
                return matrices.empty() ? osg::Matrix() : matrices.front();
            };
            for (const auto& [type, label] : { std::pair{ ESM::PRT_LHand, "LHand" }, { ESM::PRT_RHand, "RHand" },
                     { ESM::PRT_LWrist, "LWrist" }, { ESM::PRT_RWrist, "RWrist" }, { ESM::PRT_LForearm, "LForearm" },
                     { ESM::PRT_LUpperarm, "LUpperarm" }, { ESM::PRT_Head, "Head" }, { ESM::PRT_Hair, "Hair" },
                     { ESM::PRT_Cuirass, "Chest" }, { ESM::PRT_Groin, "Groin" },
                     { ESM::PRT_LLeg, "LLeg" }, { ESM::PRT_LFoot, "LFoot" }, { ESM::PRT_RFoot, "RFoot" } })
            {
                if (!mObjectParts[type])
                {
                    Log(Debug::Info) << "RAGDOLL_FP_PART " << label << " absent priority=" << mPartPriorities[type];
                    continue;
                }
                osg::Node* node = mObjectParts[type]->getNode();
                RigLister lister;
                node->accept(lister);
                const osg::Vec3d origin = worldOf(node).getTrans();
                Log(Debug::Info) << "RAGDOLL_FP_PART " << label << " node=" << node->getName() << " mask="
                                 << node->getNodeMask() << " parents=" << node->getNumParents() << " world=" << origin.x()
                                 << "," << origin.y() << "," << origin.z() << lister.mText;
            }
            for (const char* bone : { "bip01 l hand", "bip01 r hand", "bip01 l forearm", "bip01 l finger0", "bip01 head" })
            {
                const auto found = mNodeMap.find(bone);
                if (found == mNodeMap.end())
                {
                    Log(Debug::Info) << "RAGDOLL_FP_BONE " << bone << " missing";
                    continue;
                }
                const osg::Matrix world = worldOf(found->second.get());
                const osg::Vec3d p = world.getTrans();
                const osg::Vec3d s = world.getScale();
                const double det = osg::Vec3d(world(0, 0), world(0, 1), world(0, 2))
                    * (osg::Vec3d(world(1, 0), world(1, 1), world(1, 2)) ^ osg::Vec3d(world(2, 0), world(2, 1), world(2, 2)));
                Log(Debug::Info) << "RAGDOLL_FP_BONE " << bone << " world=" << p.x() << "," << p.y() << "," << p.z()
                                 << " scale=" << s.x() << "," << s.y() << "," << s.z() << " det=" << det;
            }
        }

        return ret;
    }

    void NpcAnimation::removeIndividualPart(ESM::PartReferenceType type)
    {
        // Cosmetic attachments never outlive the mesh they were projected on.
        // Session-only v1 clears that part on equipment/model replacement.
        for (auto it = mWoundOverlays.begin(); it != mWoundOverlays.end();)
        {
            if (it->mPart != type)
            {
                ++it;
                continue;
            }
            it->mParent->removeChild(it->mNode);
            it = mWoundOverlays.erase(it);
        }
        mPartPriorities[type] = 0;
        mPartslots[type] = -1;

        mObjectParts[type].reset();
        if (mSounds[type] != nullptr && !mSoundsDisabled)
        {
            MWBase::Environment::get().getSoundManager()->stopSound(mSounds[type]);
            mSounds[type] = nullptr;
        }
    }

    void NpcAnimation::reserveIndividualPart(ESM::PartReferenceType type, int group, int priority)
    {
        if (priority > mPartPriorities[type])
        {
            removeIndividualPart(type);
            mPartPriorities[type] = priority;
            mPartslots[type] = group;
        }
    }

    void NpcAnimation::removePartGroup(int group)
    {
        for (int i = 0; i < ESM::PRT_Count; i++)
        {
            if (mPartslots[i] == group)
                removeIndividualPart((ESM::PartReferenceType)i);
        }
    }

    bool NpcAnimation::isFemalePart(const ESM::BodyPart* bodypart)
    {
        return bodypart->mData.mFlags & ESM::BodyPart::BPF_Female;
    }

    bool NpcAnimation::addOrReplaceIndividualPart(ESM::PartReferenceType type, int group, int priority,
        VFS::Path::NormalizedView mesh, bool enchantedGlow, osg::Vec4f* glowColor, bool isLight)
    {
        if (priority <= mPartPriorities[type])
            return false;

        removeIndividualPart(type);
        mPartslots[type] = group;
        mPartPriorities[type] = priority;
        try
        {
            std::string_view bonename = sPartList.at(type);
            if (type == ESM::PRT_Weapon)
            {
                const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
                MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                if (weapon != inv.end() && weapon->getType() == ESM::Weapon::sRecordId)
                {
                    const ESM::RefId weaponTypeId = weapon->get<ESM::Weapon>()->mBase->mData.mType;
                    const std::string& weaponBonename = MWMechanics::getWeaponType(weaponTypeId)->mAttachBone;

                    if (weaponBonename != bonename)
                    {
                        const NodeMap& nodeMap = getNodeMap();
                        NodeMap::const_iterator found = nodeMap.find(weaponBonename);
                        if (found != nodeMap.end())
                            bonename = weaponBonename;
                    }
                }
            }

            // PRT_Hair seems to be the only type that breaks consistency and uses a filter that's different from the
            // attachment bone
            const std::string_view bonefilter = (type == ESM::PRT_Hair) ? std::string_view{ "hair" } : bonename;
            mObjectParts[type] = insertBoundedPart(mesh, bonename, bonefilter, enchantedGlow, glowColor, isLight);
        }
        catch (std::exception& e)
        {
            Log(Debug::Error) << "Error adding NPC part: " << e.what();
            return false;
        }

        if (!mSoundsDisabled && group == MWWorld::InventoryStore::Slot_CarriedLeft)
        {
            const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
            MWWorld::ConstContainerStoreIterator csi = inv.getSlot(group);
            if (csi != inv.end())
            {
                const auto soundId = csi->getClass().getSound(*csi);
                if (!soundId.empty())
                {
                    mSounds[type] = MWBase::Environment::get().getSoundManager()->playSound3D(
                        mPtr, soundId, 1.0f, 1.0f, MWSound::Type::Sfx, MWSound::PlayMode::Loop);
                }
            }
        }

        osg::Node* node = mObjectParts[type]->getNode();
        if (node->getNumChildrenRequiringUpdateTraversal() > 0)
        {
            std::shared_ptr<SceneUtil::ControllerSource> src;
            if (type == ESM::PRT_Head)
            {
                src = mHeadAnimationTime;

                if (node->getUserDataContainer())
                {
                    for (unsigned int i = 0; i < node->getUserDataContainer()->getNumUserObjects(); ++i)
                    {
                        osg::Object* obj = node->getUserDataContainer()->getUserObject(i);
                        if (SceneUtil::TextKeyMapHolder* keys = dynamic_cast<SceneUtil::TextKeyMapHolder*>(obj))
                        {
                            for (const auto& key : keys->mTextKeys)
                            {
                                if (Misc::StringUtils::ciEqual(key.second, "talk: start"))
                                    mHeadAnimationTime->setTalkStart(key.first);
                                if (Misc::StringUtils::ciEqual(key.second, "talk: stop"))
                                    mHeadAnimationTime->setTalkStop(key.first);
                                if (Misc::StringUtils::ciEqual(key.second, "blink: start"))
                                    mHeadAnimationTime->setBlinkStart(key.first);
                                if (Misc::StringUtils::ciEqual(key.second, "blink: stop"))
                                    mHeadAnimationTime->setBlinkStop(key.first);
                            }

                            break;
                        }
                    }
                }
                SceneUtil::ForceControllerSourcesVisitor assignVisitor(std::move(src));
                node->accept(assignVisitor);
            }
            else
            {
                if (type == ESM::PRT_Weapon)
                    src = mWeaponAnimationTime;
                else
                    src = mAnimationTimePtr[0];
                SceneUtil::AssignControllerSourcesVisitor assignVisitor(std::move(src));
                node->accept(assignVisitor);
            }
        }

        // An equipment change replaces the part node; the limb under it is
        // still the damaged one.
        if (mPartDamageTint[type] > 0.0f)
            applyPartDamageTint(type);
        // Posed geometry exists only after the next skinning update.
        mWoundRestoreFrames[type] = 3;

        // Hide only the attached skins (including helmets), never the head
        // bones used by the camera and physics. Equipment changes take this path too.
        if (mViewMode == VM_FirstPerson && (type == ESM::PRT_Head || type == ESM::PRT_Hair || type == ESM::PRT_Neck))
            node->setNodeMask(0);

        return true;
    }

    bool NpcAnimation::addLocalizedWound(std::string_view region, int kind, const osg::Vec3f& point,
        const osg::Vec3f& normal, const osg::Vec3f& tangent, float severity)
    {
        mLastWoundSurface = -1;
        if (WoundFxSet::enabled())
        {
            if (!point.valid() || !normal.valid() || !tangent.valid() || !std::isfinite(severity) || severity <= 0
                || kind < 1 || kind > 3)
                return false;
            // The surface lookup only reads (as the marks always did here):
            // no surface, no wound - only the placement waits.
            mProbeWound = true;
            mProbedSurface = -1;
            const bool found = placeWound(region, kind, point, normal, tangent, severity, 0.0f, 0.0f);
            mProbeWound = false;
            if (!found)
                return false;
            mLastWoundSurface = mProbedSurface;
            std::lock_guard<std::mutex> lock(mPendingWoundsMutex);
            if (mPendingWounds.size() < 32)
                mPendingWounds.push_back({ std::string(region), kind, severity, point, normal, tangent });
            return true;
        }
        return placeWound(region, kind, point, normal, tangent, severity, 0.0f, 0.0f);
    }

    WoundFxSet& NpcAnimation::woundFx()
    {
        if (!mWoundFx)
            mWoundFx = std::make_unique<WoundFxSet>();
        // FX4: the actor's blood (a skeleton's dust, a Dwemer's metal).
        mWoundFx->setBloodType(mNpc ? mNpc->mBloodType : 0);
        return *mWoundFx;
    }

    bool NpcAnimation::addWeaponBlood(float alongFrom, float alongTo, float strength, int bloodType)
    {
        if (!WoundFxSet::bloodEnabled() || !std::isfinite(alongFrom) || !std::isfinite(alongTo)
            || !std::isfinite(strength) || !(strength > 0) || bloodType == WoundFxSet::Blood_Dwemer)
            return false;
        // Any thread (a hit receipt can come from the Lua thread): queued,
        // drawn in runAnimation on the main thread.
        std::lock_guard<std::mutex> lock(mPendingWoundsMutex);
        if (mPendingWeaponBlood.size() >= 8)
            return false;
        mPendingWeaponBlood.push_back({ alongFrom, alongTo, std::min(strength, 1.0f), bloodType });
        return true;
    }

    const void* NpcAnimation::currentWeaponItem() const
    {
        if (!mPtr.getClass().hasInventoryStore(mPtr))
            return nullptr;
        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        const MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (weapon == inv.end() || weapon->getType() != ESM::Weapon::sRecordId)
            return nullptr;
        return weapon->mRef;
    }

    void NpcAnimation::placeWeaponBlood(const PendingWeaponBlood& blood)
    {
        const void* item = currentWeaponItem();
        if (!item)
            return;
        // Up to WoundFxSet::sMaxBladeMarks per weapon (its oldest gives way),
        // and a few weapons' worth in all.
        const auto onItem = [item](const WeaponBlood& mark) { return mark.mItem == item; };
        while (static_cast<unsigned int>(std::count_if(mWeaponBlood.begin(), mWeaponBlood.end(), onItem))
            >= WoundFxSet::sMaxBladeMarks)
            mWeaponBlood.erase(std::find_if(mWeaponBlood.begin(), mWeaponBlood.end(), onItem));
        while (mWeaponBlood.size() >= 4 * WoundFxSet::sMaxBladeMarks)
            mWeaponBlood.erase(mWeaponBlood.begin());
        const float seed = woundFx().newSeed();
        mWeaponBlood.push_back({ item, blood.mFrom, blood.mTo, blood.mStrength, WoundFxSet::now(), seed,
            blood.mBloodType });
        applyWeaponBlood(&mWeaponBlood.back());
    }

    void NpcAnimation::applyWeaponBlood(const WeaponBlood* only)
    {
        if (!WoundFxSet::bloodEnabled() || mWeaponBlood.empty())
            return;
        osg::Node* node = getWeaponNode();
        const void* item = currentWeaponItem();
        if (!node || !item || (only && only->mItem != item))
            return;
        const bool freshNode = mBloodiedWeaponNode.get() != node;
        if (!only && !freshNode)
            return; // its marks are on it already
        // The blade in the bone's frame: its longest axis, from the grip (the
        // bone's origin, Morrowind's weapon meshes are held there) to the far
        // end, which is the tip (the pommel side is the shorter).
        WeaponDrawableVisitor visitor;
        node->accept(visitor);
        if (visitor.mFound.empty() || !visitor.mBox.valid())
            return;
        const osg::BoundingBox& box = visitor.mBox;
        const osg::Vec3f extent = box._max - box._min;
        const int axis = extent.x() >= extent.y() && extent.x() >= extent.z() ? 0 : extent.y() >= extent.z() ? 1 : 2;
        if (!(extent[axis] > 1.0f))
            return;
        osg::Vec3f grip = box.center();
        osg::Vec3f tip = box.center();
        grip[axis] = std::clamp(0.0f, box._min[axis], box._max[axis]);
        tip[axis] = std::abs(box._max[axis]) >= std::abs(box._min[axis]) ? box._max[axis] : box._min[axis];
        const float bladeLength = std::abs(tip[axis] - grip[axis]);
        if (!(bladeLength > 1.0f))
            return;
        Shader::ShaderManager& shaderManager = mResourceSystem->getSceneManager()->getShaderManager();
        WoundFxSet& fx = woundFx();
        std::vector<const WeaponBlood*> marks;
        for (const WeaponBlood& mark : mWeaponBlood)
            if (mark.mItem == item && (freshNode || &mark == only))
                marks.push_back(&mark);
        std::vector<bool> placed(marks.size(), false);
        for (const auto& found : visitor.mFound)
        {
            // Only the drawables the bloodied range reaches (the blade, not
            // the pommel).
            const osg::BoundingBox own = found.mGeometry->getBoundingBox();
            osg::Matrix toLocal;
            if (!own.valid() || !toLocal.invert(found.mToBone))
                continue;
            float lowest = std::numeric_limits<float>::max();
            float highest = -std::numeric_limits<float>::max();
            for (unsigned int corner = 0; corner < 8; ++corner)
            {
                const float along
                    = ((own.corner(corner) * found.mToBone)[axis] - grip[axis]) / (tip[axis] - grip[axis]);
                lowest = std::min(lowest, along);
                highest = std::max(highest, along);
            }
            // The first mark on a shared (template) geometry swaps in this
            // actor's copy under the same parent: the next ones go on that.
            const unsigned int index = found.mParent->getChildIndex(found.mGeometry.get());
            for (std::size_t i = 0; i < marks.size(); ++i)
            {
                const WeaponBlood& mark = *marks[i];
                if (highest < mark.mFrom - 0.3f || lowest > mark.mTo + 0.3f)
                    continue;
                osg::Drawable* target = index < found.mParent->getNumChildren()
                    ? found.mParent->getChild(index)->asDrawable()
                    : nullptr;
                if (target
                    && fx.addBladeBlood(*target, found.mParent.get(), grip * toLocal, tip * toLocal, mark.mFrom,
                        mark.mTo, mark.mStrength, mark.mBirth, mark.mSeed, mark.mBloodType, shaderManager))
                    placed[i] = true;
            }
        }
        for (const bool took : placed)
            if (took)
                ++mWeaponBloodPlaced;
        mBloodiedWeaponNode = node;
    }

    bool NpcAnimation::addWoundFx(std::string_view region, const osg::Vec3f& point, const osg::Vec3f& normal,
        const osg::Vec3f& direction, float length, float halfWidth, float severity, int kind)
    {
        mLastWoundFxKind = 0;
        if (!WoundFxSet::enabled() || !(length > 0) || !(halfWidth > 0) || !std::isfinite(length)
            || !std::isfinite(halfWidth) || kind < WoundFxSet::Kind_Bruise || kind > WoundFxSet::Kind_Chop)
            return false;
        return placeWound(region, kind, point, normal, direction, severity, length, halfWidth);
    }

    std::size_t NpcAnimation::getWoundFxCount() const
    {
        return mWoundFx ? mWoundFx->getCount() : 0;
    }

    bool NpcAnimation::placeWound(std::string_view region, int kind, const osg::Vec3f& point,
        const osg::Vec3f& normal, const osg::Vec3f& tangent, float severity, float fxLength, float fxHalfWidth)
    {
        // Kinds: 1 bruise, 2 slash, 3 stab (the receipt kinds); 4 chop only
        // from the WOUNDFX look check (a receipt's chop is a heavy slash).
        if (!point.valid() || !normal.valid() || !tangent.valid() || normal.length2() < 0.5f
            || !std::isfinite(severity) || severity <= 0 || kind < 1 || kind > 4)
            return false;
        using Parts = std::vector<ESM::PartReferenceType>;
        static const std::pair<std::string_view, Parts> regions[] = {
            { "head", { ESM::PRT_Head } }, { "neck", { ESM::PRT_Neck } },
            // The stock torso mesh spans the abdomen and clavicles too.
            // Physics regions do not map one-to-one to equipment part slots.
            { "chest", { ESM::PRT_Cuirass } },
            { "abdomen", { ESM::PRT_Cuirass, ESM::PRT_Groin, ESM::PRT_Skirt } },
            { "pelvis", { ESM::PRT_Groin, ESM::PRT_Skirt } },
            { "left_clavicle", { ESM::PRT_LPauldron, ESM::PRT_Cuirass } },
            { "right_clavicle", { ESM::PRT_RPauldron, ESM::PRT_Cuirass } },
            { "left_upper_arm", { ESM::PRT_LUpperarm } }, { "right_upper_arm", { ESM::PRT_RUpperarm } },
            { "left_forearm", { ESM::PRT_LForearm, ESM::PRT_LWrist } },
            { "right_forearm", { ESM::PRT_RForearm, ESM::PRT_RWrist } },
            { "left_hand", { ESM::PRT_LHand } }, { "right_hand", { ESM::PRT_RHand } },
            { "left_thigh", { ESM::PRT_LLeg } }, { "right_thigh", { ESM::PRT_RLeg } },
            { "left_calf", { ESM::PRT_LKnee, ESM::PRT_LAnkle } },
            { "right_calf", { ESM::PRT_RKnee, ESM::PRT_RAnkle } },
            { "left_foot", { ESM::PRT_LFoot } }, { "right_foot", { ESM::PRT_RFoot } },
        };
        WoundSurfaceVisitor query(point, normal);
        ESM::PartReferenceType part = ESM::PRT_Head;
        // What the struck part's slot holds (WOUNDFX-01 FX3): nothing (bare
        // skin), clothing (shirts, pants, robes, skirts: anything not armour)
        // or armour. Below level 3 plate is passed over (no flesh openings on
        // it); bruises always pass through clothes. A restored wound keeps to
        // armour or off it: a dent does not come back as a cut on the chest
        // once the cuirass is off, nor an old cut as a scratch on a cuirass
        // put on over it.
        const bool surfaces = WoundFxSet::level() >= 3;
        const bool reseedArmour = mReseedingWounds && mReseedSurface == WoundFxSet::Surface_Armour;
        int surface = WoundFxSet::Surface_Skin;
        bool softArmour = false; // the struck armour is leather-like (FX3)
        for (const auto& [name, parts] : regions)
        {
            if (name != region)
                continue;
            for (const auto candidate : parts)
            {
                if (!mObjectParts[candidate])
                    continue;
                int here = WoundFxSet::Surface_Skin;
                bool softHere = false;
                if (mPartslots[candidate] >= 0)
                {
                    const auto& inv = mPtr.getClass().getInventoryStore(mPtr);
                    const auto item = inv.getSlot(mPartslots[candidate]);
                    if (item != inv.end())
                    {
                        const bool armour = item->getType() == ESM::Armor::sRecordId;
                        if (armour ? !surfaces || (mReseedingWounds && !reseedArmour) : kind == 1)
                            continue; // no flesh openings on plate, or bruises through clothes
                        here = armour ? WoundFxSet::Surface_Armour : WoundFxSet::Surface_Cloth;
                        softHere = armour && isSoftArmour(*item);
                    }
                }
                if (surfaces && reseedArmour && here != WoundFxSet::Surface_Armour)
                    continue;
                const float previous = query.mDistance;
                // Spill-over candidates: bare skin, and at FX3 every surface
                // (a wound spills only onto its own surface).
                query.mCollect = surfaces || here == WoundFxSet::Surface_Skin;
                query.mSurface = here;
                mObjectParts[candidate]->getNode()->accept(query);
                if (query.mDistance < previous)
                {
                    part = candidate;
                    surface = here;
                    softArmour = softHere;
                }
            }
            break;
        }
        if (mProbeWound)
        {
            const bool found = query.mDrawable && query.mParent && query.mPosed;
            mProbedSurface = found ? surface : -1;
            return found;
        }
        if (!query.mDrawable || !query.mParent || !query.mPosed)
        {
            if (fxLength > 0)
                Log(Debug::Warning) << "WOUNDFX01 no surface near the point for " << region << " (part "
                                    << static_cast<int>(part) << ", distance " << query.mDistance << ")";
            return false;
        }

        osg::Vec3f n = normal;
        n.normalize();
        osg::Vec3f u = tangent - n * (tangent * n);
        if (u.length2() < 0.001f)
            u = n ^ osg::Vec3f(1, 0, 0);
        if (u.length2() < 0.001f)
            u = n ^ osg::Vec3f(0, 0, 1);
        u.normalize();
        const osg::Vec3f v = n ^ u;
        const osg::Vec3f center = query.mHit * query.mMatrix;
        // Bounded like the marks below: every projection input, in its part
        // node's frame, so a rebuilt part gets its wounds back.
        constexpr std::size_t maximumMarks = 48;
        const auto rememberSeed = [&] {
            if (mReseedingWounds || !mObjectParts[part] || !mObjectParts[part]->getNode())
                return;
            const auto paths = mObjectParts[part]->getNode()->getParentalNodePaths();
            if (paths.empty())
                return;
            const osg::Matrix toWorld = osg::computeLocalToWorld(paths.front());
            const osg::Matrix toLocal = osg::Matrix::inverse(toWorld);
            while (mWoundSeeds.size() >= maximumMarks)
                mWoundSeeds.erase(mWoundSeeds.begin());
            mWoundSeeds.push_back({ part, std::string(region), kind, severity, point * toLocal,
                osg::Matrix::transform3x3(normal, toLocal), osg::Matrix::transform3x3(tangent, toLocal), fxLength,
                fxHalfWidth, surface });
        };
        // WOUNDFX-01 FX1/FX2: every wound on bare skin becomes a wound volume
        // in the skin shader instead of a mesh-copy mark: slashes, stabs,
        // chops (a slash of severity 0.8 or more) and bruises. FX3 (level 3):
        // the same record on clothing is a blood stain, on armour a scratch
        // or a dent; below it clothing keeps the marks.
        const int stroke = WoundFxSet::enabled() ? WoundFxSet::drawnKind(kind, severity) : 0;
        if (fxLength > 0 && stroke == 0)
        {
            Log(Debug::Warning) << "WOUNDFX01 kind " << kind << " has no volume at OPENMW_RAGDOLL_WOUND_FX="
                                << WoundFxSet::level();
            return false; // a test shape has no mesh-copy fallback
        }
        // FX4 (level 4): a Dwemer's skin is metal (blood type 2: sparks, no
        // blood): a cut, stab or blow on it is a scratch or a dent, as on
        // armour.
        const bool metalSkin = WoundFxSet::bloodEnabled() && surface == WoundFxSet::Surface_Skin && mNpc
            && mNpc->mBloodType == WoundFxSet::Blood_Dwemer;
        const int drawnKind = WoundFxSet::surfaceKind(stroke, metalSkin ? WoundFxSet::Surface_Armour : surface);
        if (drawnKind != 0)
        {
            // Skinned parts, morphed heads and rigid parts (the vanilla
            // forearms are a plain Geometry on the bone).
            if (dynamic_cast<SceneUtil::RigGeometry*>(query.mDrawable.get())
                || dynamic_cast<SceneUtil::MorphGeometry*>(query.mDrawable.get()) || query.mDrawable->asGeometry())
            {
                // Shapes by kind (WOUNDFX-01 section 1), world units: a cut's
                // length and half gape; a stab's slit about a blade wide; a
                // chop gaping wide; a bruise's radius round a short smear.
                // A stain or a scratch takes its stroke's shape.
                const float strength = std::clamp(severity, 0.0f, 1.0f);
                float length = 3.5f + 4.5f * strength;
                float halfWidth = 0.3f + 0.45f * strength;
                if (stroke == WoundFxSet::Kind_Stab)
                {
                    length = 1.4f + 1.2f * strength;
                    halfWidth = 0.2f + 0.22f * strength;
                }
                else if (stroke == WoundFxSet::Kind_Chop)
                {
                    length = 4.5f + 4.5f * strength;
                    halfWidth = 1.7f * (0.3f + 0.45f * strength);
                }
                else if (stroke == WoundFxSet::Kind_Bruise)
                {
                    halfWidth = 1.0f + 1.0f * strength;
                    length = 0.6f * halfWidth;
                }
                if (fxLength > 0)
                    length = fxLength;
                if (fxHalfWidth > 0)
                    halfWidth = fxHalfWidth;
                // Armour marks read at fighting distance (ARMOUR_MARKS): a
                // scratch or a dent drawn larger.
                if (drawnKind == WoundFxSet::Kind_Scratch)
                {
                    length *= WoundFxSet::armourMarkScale();
                    halfWidth *= WoundFxSet::armourMarkScale();
                }
                WoundFxSet& fx = woundFx();
                Shader::ShaderManager& shaderManager = mResourceSystem->getSceneManager()->getShaderManager();
                const float seed = fx.newSeed();
                // FX4: the frame the skin under the wound moves with, for its
                // blood running downhill: a skinned part's region bone, a
                // rigid part's (or a head's) parent node.
                osg::ref_ptr<osg::Node> regionFrame;
                if (WoundFxSet::bloodEnabled())
                {
                    const std::string_view bone = regionBone(region);
                    const NodeMap& nodes = getNodeMap();
                    if (const auto found = bone.empty() ? nodes.end() : nodes.find(std::string(bone));
                        found != nodes.end())
                        regionFrame = found->second;
                }
                const auto runFrame = [&](osg::Drawable* drawable, osg::Group* parent) -> osg::Node* {
                    if (!WoundFxSet::bloodEnabled())
                        return nullptr;
                    if (dynamic_cast<SceneUtil::RigGeometry*>(drawable))
                        return regionFrame.get();
                    return parent;
                };
                if (fx.addWound(*query.mDrawable, query.mParent.get(), *query.mPosed, query.mMatrix, query.mHit, n,
                        u, drawnKind, length, halfWidth, strength, seed, shaderManager, stroke, softArmour,
                        runFrame(query.mDrawable.get(), query.mParent.get())))
                {
                    // The same wound on the region's other drawables of its
                    // surface it reaches (the forearm piece ends mid-arm, the
                    // wrist piece goes on; a shirt or a cuirass of several
                    // meshes): no straight seam cuts it off. Same middle, axes
                    // and seed, each in its own bind space. (FX4: a stain sags
                    // downhill up to 0.6 of its radius more.)
                    const float extent = drawnKind == WoundFxSet::Kind_Stain
                        ? 0.5f * length
                            + (WoundFxSet::bloodEnabled() ? 2.25f : 1.65f)
                                * WoundFxSet::stainRadius(stroke, halfWidth, strength)
                            + 0.3f
                        : 0.5f * length + 3.6f * halfWidth + 0.3f;
                    for (const auto& other : query.mCandidates)
                    {
                        if (other.mDrawable == query.mDrawable || !other.mPosed || !other.mParent
                            || other.mSurface != surface)
                            continue;
                        const osg::Matrix inverse = osg::Matrix::inverse(other.mMatrix);
                        osg::TriangleFunctor<WoundSurface> spill;
                        spill.mPoint = center * inverse;
                        spill.mNormal = osg::Matrix::transform3x3(n, inverse);
                        const float scale = spill.mNormal.normalize();
                        if (!(scale > 0))
                            continue;
                        spill.mDistance = extent * scale;
                        other.mPosed->accept(spill);
                        if (spill.mFound)
                            fx.addWound(*other.mDrawable, other.mParent.get(), *other.mPosed, other.mMatrix,
                                spill.mPoint, n, u, drawnKind, length, halfWidth, strength, seed, shaderManager,
                                stroke, softArmour, runFrame(other.mDrawable.get(), other.mParent.get()));
                    }
                    // A hard blow on armour bleeds out from under it
                    // (ARMOUR_MARKS): a blood stain welling from the mark and
                    // sagging down the plate (FX4 runs stains downhill), on
                    // the struck piece just below the mark. Red blood only.
                    if (drawnKind == WoundFxSet::Kind_Scratch && WoundFxSet::bloodEnabled() && !metalSkin
                        && fx.getBloodType() == WoundFxSet::Blood_Red
                        && strength >= WoundFxSet::armourBleedSeverity())
                    {
                        osg::Vec3f down = osg::Vec3f(0, 0, -1) - n * (-n.z());
                        if (down.length2() < 0.05f)
                            down = v;
                        down.normalize();
                        const osg::Vec3f welling = center + down * (0.35f * length + 1.0f);
                        const float ooze = 0.6f + 0.4f * std::clamp(
                            (strength - WoundFxSet::armourBleedSeverity()) / 0.4f, 0.0f, 1.0f);
                        fx.addWound(*query.mDrawable, query.mParent.get(), *query.mPosed, query.mMatrix,
                            welling * osg::Matrix::inverse(query.mMatrix), n, down, WoundFxSet::Kind_Stain,
                            0.5f * length, 0.9f * halfWidth * ooze, strength, fx.newSeed(), shaderManager,
                            WoundFxSet::Kind_Bruise, false, runFrame(query.mDrawable.get(), query.mParent.get()));
                    }
                    mLastWoundFxKind = drawnKind;
                    rememberSeed();
                    return true;
                }
            }
            if (fxLength > 0)
            {
                Log(Debug::Warning) << "WOUNDFX01 not placed on " << region << ": surface=" << surface << " rig="
                                    << (dynamic_cast<SceneUtil::RigGeometry*>(query.mDrawable.get()) != nullptr)
                                    << " drawable=" << query.mDrawable->className();
                return false; // a test shape has no mesh-copy fallback
            }
        }
        // Armour has no mesh-copy mark (blood on plate); it reaches here only
        // at FX3 when the scratch did not take.
        if (surface == WoundFxSet::Surface_Armour || metalSkin)
            return false;
        // The mesh-copy marks know no chop: a cut.
        const int markKind = kind == 4 ? 2 : kind;
        const float length = markKind == 1 ? 3.0f + 2.0f * severity
            : markKind == 2 ? 3.0f + 3.0f * severity : 1.0f + 0.6f * severity;
        const float width = markKind == 1 ? length * 0.8f : markKind == 2 ? 0.75f : 0.6f;
        const auto* posedVertices = dynamic_cast<const osg::Vec3Array*>(query.mPosed->getVertexArray());
        const auto* posedNormals = dynamic_cast<const osg::Vec3Array*>(query.mPosed->getNormalArray());
        auto* rig = dynamic_cast<SceneUtil::RigGeometry*>(query.mDrawable.get());
        auto* morph = dynamic_cast<SceneUtil::MorphGeometry*>(query.mDrawable.get());
        osg::Geometry* source = rig ? rig->getSourceGeometry().get()
            : morph ? morph->getSourceGeometry().get() : query.mDrawable->asGeometry();
        if (!posedVertices || !source || posedVertices->size() != source->getVertexArray()->getNumElements())
            return false;
        osg::ref_ptr<osg::Geometry> overlay = new osg::Geometry(*source, osg::CopyOp::DEEP_COPY_ALL);
        osg::ref_ptr<osg::Vec4Array> coordinates = new osg::Vec4Array;
        coordinates->reserve(posedVertices->size());
        for (std::size_t i = 0; i < posedVertices->size(); ++i)
        {
            const osg::Vec3f delta = (*posedVertices)[i] * query.mMatrix - center;
            osg::Vec3f surfaceNormal = n;
            if (posedNormals && i < posedNormals->size())
            {
                surfaceNormal = osg::Matrix::transform3x3((*posedNormals)[i], query.mMatrix);
                surfaceNormal.normalize();
            }
            coordinates->push_back(osg::Vec4f((delta * u) / length, (delta * v) / width,
                delta * n, surfaceNormal * n));
        }
        overlay->setTexCoordArray(0, coordinates, osg::Array::BIND_PER_VERTEX);
        overlay->setName("localized wound");
        overlay->setUserValue("localizedWound", true);
        overlay->setStateSet(woundState(surface == WoundFxSet::Surface_Cloth ? 4 : markKind, severity));
        overlay->setUpdateCallback(nullptr);
        overlay->setCullCallback(nullptr);
        osg::ref_ptr<osg::Drawable> drawable = overlay;
        if (rig)
        {
            osg::ref_ptr<SceneUtil::RigGeometry> skinned = new SceneUtil::RigGeometry(*rig, osg::CopyOp::SHALLOW_COPY);
            skinned->setSourceGeometry(overlay);
            // RigGeometry uses its original shape name to cancel skin-root
            // transforms. Renaming the clone would change its deformation.
            skinned->setUserDataContainer(osg::clone(rig->getUserDataContainer(), osg::CopyOp::SHALLOW_COPY));
            skinned->setUserValue("localizedWound", true);
            skinned->setStateSet(overlay->getStateSet());
            skinned->setUpdateCallback(nullptr);
            skinned->setCullCallback(nullptr);
            drawable = skinned;
        }
        else if (morph)
        {
            osg::ref_ptr<SceneUtil::MorphGeometry> face = new SceneUtil::MorphGeometry(*morph, osg::CopyOp::SHALLOW_COPY);
            face->setSourceGeometry(overlay);
            face->followMorphWeights(morph);
            face->setUserDataContainer(osg::clone(morph->getUserDataContainer(), osg::CopyOp::SHALLOW_COPY));
            face->setUserValue("localizedWound", true);
            face->setStateSet(overlay->getStateSet());
            face->setUpdateCallback(nullptr);
            face->setCullCallback(nullptr);
            drawable = face;
        }
        // One bounded mesh overlay per successful projection. Old cosmetic
        // marks can expire without changing health or the damage ledger.
        // Bounded: each mark copies its part's mesh. 12 filled within one
        // fight once every weapon left marks; 48 lasts a long duel.
        while (mWoundOverlays.size() >= maximumMarks)
        {
            mWoundOverlays.front().mParent->removeChild(mWoundOverlays.front().mNode);
            mWoundOverlays.erase(mWoundOverlays.begin());
        }
        query.mParent->addChild(drawable);
        mWoundOverlays.push_back({ query.mParent, drawable, part });
        rememberSeed();
        return true;
    }

    void NpcAnimation::restoreWoundSeeds(ESM::PartReferenceType type)
    {
        if (mWoundSeeds.empty() || !mObjectParts[type] || !mObjectParts[type]->getNode())
            return;
        const auto paths = mObjectParts[type]->getNode()->getParentalNodePaths();
        if (paths.empty())
            return;
        const osg::Matrix toWorld = osg::computeLocalToWorld(paths.front());
        mReseedingWounds = true;
        for (const auto& seed : mWoundSeeds)
        {
            if (seed.mPart != type)
                continue;
            mReseedSurface = seed.mSurface;
            placeWound(seed.mRegion, seed.mKind, seed.mPoint * toWorld,
                osg::Matrix::transform3x3(seed.mNormal, toWorld), osg::Matrix::transform3x3(seed.mTangent, toWorld),
                seed.mSeverity, seed.mLength, seed.mHalfWidth);
        }
        mReseedingWounds = false;
    }

    void NpcAnimation::applyPartDamageTint(ESM::PartReferenceType type)
    {
        if (!mObjectParts[type] || !mObjectParts[type]->getNode())
            return;
        osg::Node* node = mObjectParts[type]->getNode();
        const float tint = std::clamp(mPartDamageTint[type], 0.0f, 1.0f);
        // Always a fresh state set: the one on the node may be in use by the
        // draw traversal of the previous frame.
        osg::ref_ptr<osg::StateSet> stateset = node->getStateSet()
            ? new osg::StateSet(*node->getStateSet(), osg::CopyOp::SHALLOW_COPY)
            : new osg::StateSet;
        // Material emission is multiplied by the texture and clamped with the
        // lighting. On a lit sleeve it can disappear entirely. Blend the final
        // surface colour instead, without replacing the authored material.
        const osg::StateAttribute::GLModeValue mode = osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE;
        stateset->addUniform(new osg::Uniform("anatomicalDamageTint", tint), mode);
        node->setStateSet(stateset);
    }

    void NpcAnimation::setAnatomicalDamageTint(std::string_view region, float intensity)
    {
        // Own the elements: an initializer_list copied into std::pair would
        // retain pointers to a temporary backing array after initialization.
        using Parts = std::vector<ESM::PartReferenceType>;
        static const std::pair<std::string_view, Parts> sRegionParts[] = {
            { "head", { ESM::PRT_Head, ESM::PRT_Hair } },
            { "neck", { ESM::PRT_Neck } },
            { "chest", { ESM::PRT_Cuirass } },
            { "abdomen", { ESM::PRT_Groin, ESM::PRT_Skirt } },
            { "pelvis", { ESM::PRT_Groin, ESM::PRT_Skirt, ESM::PRT_Tail } },
            { "left_clavicle", { ESM::PRT_LPauldron } },
            { "right_clavicle", { ESM::PRT_RPauldron } },
            { "left_upper_arm", { ESM::PRT_LUpperarm } },
            { "right_upper_arm", { ESM::PRT_RUpperarm } },
            { "left_forearm", { ESM::PRT_LForearm, ESM::PRT_LWrist } },
            { "right_forearm", { ESM::PRT_RForearm, ESM::PRT_RWrist } },
            { "left_hand", { ESM::PRT_LHand } },
            { "right_hand", { ESM::PRT_RHand } },
            { "left_thigh", { ESM::PRT_LLeg } },
            { "right_thigh", { ESM::PRT_RLeg } },
            { "left_calf", { ESM::PRT_LKnee, ESM::PRT_LAnkle } },
            { "right_calf", { ESM::PRT_RKnee, ESM::PRT_RAnkle } },
            { "left_foot", { ESM::PRT_LFoot } },
            { "right_foot", { ESM::PRT_RFoot } },
        };
        if (!std::isfinite(intensity))
            return;
        for (const auto& [name, parts] : sRegionParts)
        {
            if (name != region)
                continue;
            for (const ESM::PartReferenceType part : parts)
            {
                // Several regions can share a part (abdomen and pelvis both
                // show on the groin): the part keeps the strongest tint.
                const float wanted = std::clamp(intensity, 0.0f, 1.0f);
                const float value = wanted <= 0.0f ? 0.0f : std::max(mPartDamageTint[part], wanted);
                if (value == mPartDamageTint[part])
                    continue;
                mPartDamageTint[part] = value;
                applyPartDamageTint(part);
            }
            return;
        }
    }

    void NpcAnimation::clearAnatomicalDamageTints()
    {
        for (const auto& mark : mWoundOverlays)
            mark.mParent->removeChild(mark.mNode);
        mWoundOverlays.clear();
        mWoundSeeds.clear();
        if (mWoundFx)
            mWoundFx->clear();
        // FX4: the weapon's blood goes with the reset too.
        mWeaponBlood.clear();
        mBloodiedWeaponNode = nullptr;
        {
            std::lock_guard<std::mutex> lock(mPendingWoundsMutex);
            mPendingWeaponBlood.clear();
        }
        for (int part = 0; part < ESM::PRT_Count; ++part)
        {
            if (mPartDamageTint[part] <= 0.0f)
                continue;
            mPartDamageTint[part] = 0.0f;
            applyPartDamageTint(static_cast<ESM::PartReferenceType>(part));
        }
    }

    void NpcAnimation::addPartGroup(int group, int priority, const std::vector<ESM::PartReference>& parts,
        bool enchantedGlow, osg::Vec4f* glowColor)
    {
        const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();
        const MWWorld::Store<ESM::BodyPart>& partStore = store.get<ESM::BodyPart>();

        for (const ESM::PartReference& part : parts)
        {
            const ESM::BodyPart* bodypart = nullptr;
            if (!mNpc->isMale() && !part.mFemale.empty())
            {
                bodypart = partStore.search(part.mFemale);
                if (!bodypart)
                    Log(Debug::Warning) << "Warning: Failed to find body part '" << part.mFemale << "'";
            }
            if (!bodypart && !part.mMale.empty())
            {
                bodypart = partStore.search(part.mMale);
                if (!bodypart)
                    Log(Debug::Warning) << "Warning: Failed to find body part '" << part.mMale << "'";
            }

            if (bodypart)
                addOrReplaceIndividualPart(static_cast<ESM::PartReferenceType>(part.mPart), group, priority,
                    Misc::ResourceHelpers::correctMeshPath(bodypart->mModel.getNormalized()), enchantedGlow, glowColor);
            else
                reserveIndividualPart((ESM::PartReferenceType)part.mPart, group, priority);
        }
    }

    void NpcAnimation::addControllers()
    {
        Animation::addControllers();

        mFirstPersonNeckController = nullptr;
        WeaponAnimation::deleteControllers();

        if (mViewMode == VM_Normal || mViewMode == VM_FirstPerson)
        {
            // Looking down must leave the torso and feet under the camera.
            WeaponAnimation::addControllers(mNodeMap, mActiveControllers, mObjectRoot.get());
        }
    }

    void NpcAnimation::showWeapons(bool showWeapon)
    {
        mShowWeapons = showWeapon;
        mAmmunition.reset();
        if (showWeapon)
        {
            const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
            MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            if (weapon != inv.end())
            {
                osg::Vec4f glowColor = weapon->getClass().getEnchantmentColor(*weapon);
                const VFS::Path::Normalized mesh = weapon->getClass().getCorrectedModel(*weapon);
                addOrReplaceIndividualPart(ESM::PRT_Weapon, MWWorld::InventoryStore::Slot_CarriedRight, 1, mesh,
                    !weapon->getClass().getEnchantment(*weapon).empty(), &glowColor);

                // Crossbows start out with a bolt attached
                if (weapon->getType() == ESM::Weapon::sRecordId
                    && weapon->get<ESM::Weapon>()->mBase->mData.mType == ESM::WeaponType::MarksmanCrossbow)
                {
                    const ESM::RefId ammotype
                        = MWMechanics::getWeaponType(ESM::WeaponType::MarksmanCrossbow)->mAmmoType;
                    MWWorld::ConstContainerStoreIterator ammo = inv.getSlot(MWWorld::InventoryStore::Slot_Ammunition);
                    if (ammo != inv.end() && ammo->get<ESM::Weapon>()->mBase->mData.mType == ammotype)
                        attachArrow();
                }
            }
        }
        else
        {
            removeIndividualPart(ESM::PRT_Weapon);
            // If we remove/hide weapon from player, we should reset attack animation as well
            if (mPtr == MWMechanics::getPlayer())
                mPtr.getClass().getCreatureStats(mPtr).setAttackingOrSpell(false);
        }

        updateHolsteredWeapon(!mShowWeapons);
        updateQuiver();
    }

    bool NpcAnimation::updateCarriedLeftVisible(ESM::RefId weaptype) const
    {
        if (Settings::game().mShieldSheathing)
        {
            const MWWorld::Class& cls = mPtr.getClass();
            MWMechanics::CreatureStats& stats = cls.getCreatureStats(mPtr);
            if (stats.getDrawState() == MWMechanics::DrawState::Nothing)
            {
                SceneUtil::FindByNameVisitor findVisitor("Bip01 AttachShield");
                mObjectRoot->accept(findVisitor);
                if (findVisitor.mFoundNode || mViewMode == VM_FirstPerson)
                {
                    const MWWorld::InventoryStore& inv = cls.getInventoryStore(mPtr);
                    const MWWorld::ConstContainerStoreIterator shield
                        = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
                    if (shield != inv.end() && shield->getType() == ESM::Armor::sRecordId
                        && !getSheathedShieldMesh(*shield).empty())
                        return false;
                }
            }
        }

        return !(MWMechanics::getWeaponType(weaptype)->mFlags & ESM::WeaponType::TwoHanded);
    }

    void NpcAnimation::showCarriedLeft(bool show)
    {
        mShowCarriedLeft = show;
        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator iter = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        if (show && iter != inv.end())
        {
            osg::Vec4f glowColor = iter->getClass().getEnchantmentColor(*iter);
            VFS::Path::Normalized mesh = iter->getClass().getCorrectedModel(*iter);
            // For shields we must try to use the body part model
            if (iter->getType() == ESM::Armor::sRecordId)
            {
                mesh = getShieldMesh(*iter, !mNpc->isMale());
            }
            if (mesh.empty()
                || addOrReplaceIndividualPart(ESM::PRT_Shield, MWWorld::InventoryStore::Slot_CarriedLeft, 1, mesh,
                    !iter->getClass().getEnchantment(*iter).empty(), &glowColor,
                    iter->getType() == ESM::Light::sRecordId))
            {
                if (mesh.empty())
                    reserveIndividualPart(ESM::PRT_Shield, MWWorld::InventoryStore::Slot_CarriedLeft, 1);
                if (iter->getType() == ESM::Light::sRecordId && mObjectParts[ESM::PRT_Shield])
                    addExtraLight(mObjectParts[ESM::PRT_Shield]->getNode()->asGroup(),
                        SceneUtil::LightCommon(*iter->get<ESM::Light>()->mBase));
            }
        }
        else
            removeIndividualPart(ESM::PRT_Shield);

        updateHolsteredShield(mShowCarriedLeft);
    }

    void NpcAnimation::attachArrow()
    {
        WeaponAnimation::attachArrow(mPtr);

        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator ammo = inv.getSlot(MWWorld::InventoryStore::Slot_Ammunition);
        if (ammo != inv.end() && !ammo->getClass().getEnchantment(*ammo).empty())
        {
            osg::Group* bone = getArrowBone();
            if (bone != nullptr && bone->getNumChildren())
                SceneUtil::addEnchantedGlow(
                    bone->getChild(0), mResourceSystem, ammo->getClass().getEnchantmentColor(*ammo));
        }

        updateQuiver();
    }

    void NpcAnimation::detachArrow()
    {
        WeaponAnimation::detachArrow(mPtr);
        updateQuiver();
    }

    void NpcAnimation::releaseArrow(float attackStrength, float attackWindUp)
    {
        WeaponAnimation::releaseArrow(mPtr, attackStrength, attackWindUp);
        updateQuiver();
    }

    osg::Group* NpcAnimation::getArrowBone()
    {
        const PartHolder* const part = mObjectParts[ESM::PRT_Weapon].get();
        if (part == nullptr)
            return nullptr;

        const MWWorld::InventoryStore& inv = mPtr.getClass().getInventoryStore(mPtr);
        MWWorld::ConstContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (weapon == inv.end() || weapon->getType() != ESM::Weapon::sRecordId)
            return nullptr;

        const ESM::RefId type = weapon->get<ESM::Weapon>()->mBase->mData.mType;
        const ESM::RefId ammoType = MWMechanics::getWeaponType(type)->mAmmoType;
        if (ammoType.empty())
            return nullptr;

        // Try to find and attachment bone in actor's skeleton, otherwise fall back to the ArrowBone in weapon's mesh
        osg::Group* bone = getBoneByName(MWMechanics::getWeaponType(ammoType)->mAttachBone);
        if (bone == nullptr)
        {
            SceneUtil::FindByNameVisitor findVisitor("ArrowBone");
            part->getNode()->accept(findVisitor);
            bone = findVisitor.mFoundNode;
        }
        return bone;
    }

    osg::Node* NpcAnimation::getWeaponNode()
    {
        const PartHolder* const part = mObjectParts[ESM::PRT_Weapon].get();
        if (part == nullptr)
            return nullptr;
        return part->getNode();
    }

    void NpcAnimation::setWeaponGripTurn(const std::optional<osg::Matrixf>& handFrame)
    {
        osg::Node* weapon = getWeaponNode();
        // Unwrap when the turn ends or the weapon part changed (a new part is
        // attached to the bone again; the old one left the wrapper empty).
        if (mWeaponGripNode && (!handFrame || weapon == nullptr || mWeaponGripChild.get() != weapon))
        {
            const osg::ref_ptr<osg::MatrixTransform> wrapper = mWeaponGripNode;
            mWeaponGripNode = nullptr;
            osg::ref_ptr<osg::Node> child;
            if (mWeaponGripChild.lock(child) && child.get() == weapon)
            {
                while (wrapper->getNumParents() > 0)
                    wrapper->getParent(0)->replaceChild(wrapper, child);
            }
            else
            {
                while (wrapper->getNumParents() > 0)
                    wrapper->getParent(0)->removeChild(wrapper);
            }
            mWeaponGripChild = nullptr;
        }
        if (!handFrame || weapon == nullptr)
            return;
        const osg::Node* hand = getNode("Bip01 R Hand");
        if (hand == nullptr)
            return;
        if (!mWeaponGripNode)
        {
            if (weapon->getNumParents() != 1)
                return;
            osg::Group* parent = weapon->getParent(0);
            mWeaponGripNode = new osg::MatrixTransform;
            mWeaponGripNode->setName("KINETIC grip turn");
            parent->replaceChild(weapon, mWeaponGripNode);
            mWeaponGripNode->addChild(weapon);
            mWeaponGripChild = weapon;
        }
        if (mWeaponGripNode->getNumParents() != 1)
            return;
        // The turn is in the hand's frame; the wrapper sits in the frame of the
        // weapon's own parent (the weapon bone, which may be offset from it).
        const auto parentPaths = mWeaponGripNode->getParent(0)->getParentalNodePaths(getObjectRoot());
        const auto handPaths = hand->getParentalNodePaths(getObjectRoot());
        if (parentPaths.size() != 1 || handPaths.size() != 1)
            return;
        const osg::Matrixf parentToHand
            = osg::computeLocalToWorld(parentPaths.front()) * osg::Matrixf::inverse(osg::computeLocalToWorld(handPaths.front()));
        mWeaponGripNode->setMatrix(parentToHand * *handFrame * osg::Matrixf::inverse(parentToHand));
    }

    osg::Node* NpcAnimation::getShieldNode()
    {
        const PartHolder* const part = mObjectParts[ESM::PRT_Shield].get();
        if (part == nullptr)
            return nullptr;
        return part->getNode();
    }

    Resource::ResourceSystem* NpcAnimation::getResourceSystem()
    {
        return mResourceSystem;
    }

    void NpcAnimation::enableHeadAnimation(bool enable)
    {
        mHeadAnimationTime->setEnabled(enable);
    }

    void NpcAnimation::setWeaponGroup(const std::string& group, bool relativeDuration)
    {
        mWeaponAnimationTime->setGroup(group, relativeDuration);
    }

    void NpcAnimation::equipmentChanged()
    {
        if (Settings::game().mShieldSheathing)
        {
            ESM::RefId weaponTypeId;
            MWMechanics::getActiveWeapon(mPtr, &weaponTypeId);
            showCarriedLeft(updateCarriedLeftVisible(weaponTypeId));
        }

        updateParts();
    }

    void NpcAnimation::setVampire(bool vampire)
    {
        if (mNpcType == Type_Werewolf) // we can't have werewolf vampires, can we
            return;
        if ((mNpcType == Type_Vampire) != vampire)
        {
            if (mPtr == MWMechanics::getPlayer())
                MWBase::Environment::get().getWorld()->reattachPlayerCamera();
            else
                rebuild();
        }
    }

    void NpcAnimation::setFirstPersonOffset(const osg::Vec3f& offset)
    {
        mFirstPersonOffset = offset;
    }

    void NpcAnimation::updatePtr(const MWWorld::Ptr& updated)
    {
        Animation::updatePtr(updated);
        mHeadAnimationTime->updatePtr(updated);
    }

    // Remember body parts so we only have to search through the store once for each race/gender/viewmode combination
    typedef std::map<std::pair<ESM::RefId, int>, std::vector<const ESM::BodyPart*>> RaceMapping;
    static RaceMapping sRaceMapping;

    const std::vector<const ESM::BodyPart*>& NpcAnimation::getBodyParts(
        const ESM::RefId& race, bool female, bool firstPerson, bool werewolf)
    {
        constexpr int flagFirstPerson = 1 << 1;
        constexpr int flagFemale = 1 << 0;

        int flags = (werewolf ? -1 : 0);
        if (female)
            flags |= flagFemale;
        if (firstPerson)
            flags |= flagFirstPerson;

        RaceMapping::iterator found = sRaceMapping.find(std::make_pair(race, flags));
        if (found != sRaceMapping.end())
            return found->second;
        else
        {
            std::vector<const ESM::BodyPart*>& parts = sRaceMapping[std::make_pair(race, flags)];

            typedef std::multimap<ESM::BodyPart::MeshPart, ESM::PartReferenceType> BodyPartMapType;
            static const BodyPartMapType sBodyPartMap = { { ESM::BodyPart::MP_Neck, ESM::PRT_Neck },
                { ESM::BodyPart::MP_Chest, ESM::PRT_Cuirass }, { ESM::BodyPart::MP_Groin, ESM::PRT_Groin },
                { ESM::BodyPart::MP_Hand, ESM::PRT_RHand }, { ESM::BodyPart::MP_Hand, ESM::PRT_LHand },
                { ESM::BodyPart::MP_Wrist, ESM::PRT_RWrist }, { ESM::BodyPart::MP_Wrist, ESM::PRT_LWrist },
                { ESM::BodyPart::MP_Forearm, ESM::PRT_RForearm }, { ESM::BodyPart::MP_Forearm, ESM::PRT_LForearm },
                { ESM::BodyPart::MP_Upperarm, ESM::PRT_RUpperarm }, { ESM::BodyPart::MP_Upperarm, ESM::PRT_LUpperarm },
                { ESM::BodyPart::MP_Foot, ESM::PRT_RFoot }, { ESM::BodyPart::MP_Foot, ESM::PRT_LFoot },
                { ESM::BodyPart::MP_Ankle, ESM::PRT_RAnkle }, { ESM::BodyPart::MP_Ankle, ESM::PRT_LAnkle },
                { ESM::BodyPart::MP_Knee, ESM::PRT_RKnee }, { ESM::BodyPart::MP_Knee, ESM::PRT_LKnee },
                { ESM::BodyPart::MP_Upperleg, ESM::PRT_RLeg }, { ESM::BodyPart::MP_Upperleg, ESM::PRT_LLeg },
                { ESM::BodyPart::MP_Tail, ESM::PRT_Tail } };

            parts.resize(ESM::PRT_Count, nullptr);

            if (werewolf)
                return parts;

            const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();

            for (const ESM::BodyPart& bodypart : store.get<ESM::BodyPart>())
            {
                if (bodypart.mData.mFlags & ESM::BodyPart::BPF_NotPlayable)
                    continue;
                if (bodypart.mData.mType != ESM::BodyPart::MT_Skin)
                    continue;

                if (!(bodypart.mRace == race))
                    continue;

                const bool partFirstPerson = ESM::isFirstPersonBodyPart(bodypart);

                bool isHand = bodypart.mData.mPart == ESM::BodyPart::MP_Hand
                    || bodypart.mData.mPart == ESM::BodyPart::MP_Wrist
                    || bodypart.mData.mPart == ESM::BodyPart::MP_Forearm
                    || bodypart.mData.mPart == ESM::BodyPart::MP_Upperarm;

                bool isSameGender = isFemalePart(&bodypart) == female;

                /* A fallback for the arms if 1st person is missing:
                 1. Try to use 3d person skin for same gender
                 2. Try to use 1st person skin for male, if female == true
                 3. Try to use 3d person skin for male, if female == true

                 A fallback in another cases: allow to use male bodyparts, if female == true
                */
                if (firstPerson && isHand && !partFirstPerson)
                {
                    // Allow 3rd person skins as a fallback for the arms if 1st person is missing
                    BodyPartMapType::const_iterator bIt
                        = sBodyPartMap.lower_bound(BodyPartMapType::key_type(bodypart.mData.mPart));
                    while (bIt != sBodyPartMap.end() && bIt->first == bodypart.mData.mPart)
                    {
                        // If we have no fallback bodypart now and bodypart is for same gender (1)
                        if (!parts[bIt->second] && isSameGender)
                            parts[bIt->second] = &bodypart;

                        // If we have fallback bodypart for other gender and found fallback for current gender (1)
                        else if (isSameGender && isFemalePart(parts[bIt->second]) != female)
                            parts[bIt->second] = &bodypart;

                        // If we have no fallback bodypart and searching for female bodyparts (3)
                        else if (!parts[bIt->second] && female)
                            parts[bIt->second] = &bodypart;

                        ++bIt;
                    }

                    continue;
                }

                // Don't allow to use podyparts for a different view
                if (partFirstPerson != firstPerson)
                    continue;

                if (female && !isFemalePart(&bodypart))
                {
                    // Allow male parts as fallback for females if female parts are missing
                    BodyPartMapType::const_iterator bIt
                        = sBodyPartMap.lower_bound(BodyPartMapType::key_type(bodypart.mData.mPart));
                    while (bIt != sBodyPartMap.end() && bIt->first == bodypart.mData.mPart)
                    {
                        // If we have no fallback bodypart now
                        if (!parts[bIt->second])
                            parts[bIt->second] = &bodypart;

                        // If we have 3d person fallback bodypart for hand and 1st person fallback found (2)
                        else if (isHand && !ESM::isFirstPersonBodyPart(*parts[bIt->second]) && partFirstPerson)
                            parts[bIt->second] = &bodypart;

                        ++bIt;
                    }

                    continue;
                }

                // Don't allow to use podyparts for another gender
                if (female != isFemalePart(&bodypart))
                    continue;

                // Use properly found bodypart, replacing fallbacks
                BodyPartMapType::const_iterator bIt
                    = sBodyPartMap.lower_bound(BodyPartMapType::key_type(bodypart.mData.mPart));
                while (bIt != sBodyPartMap.end() && bIt->first == bodypart.mData.mPart)
                {
                    parts[bIt->second] = &bodypart;
                    ++bIt;
                }
            }
            return parts;
        }
    }

    void NpcAnimation::setAccurateAiming(bool enabled)
    {
        mAccurateAiming = enabled;
    }

    bool NpcAnimation::isArrowAttached() const
    {
        return mAmmunition != nullptr;
    }

}
