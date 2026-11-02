#include "physicssystem.hpp"
#include <ragdollcore/tuning.hpp>
#include "../manualdiagnostics.hpp"
#include "../fpsfixture.hpp"
#include "../teststepscheduler.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <bit>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <iomanip>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <map>
#include <sstream>
#include <fstream>
#include <functional>
#include <Jolt/Core/StreamWrapper.h>
#include <sstream>
#include <mutex>
#include <string_view>
#include <vector>

#include <osg/Group>
#include <osg/ComputeBoundsVisitor>
#include <osg/MatrixTransform>
#include <osg/Stats>
#include <osg/Timer>
#include <osg/Drawable>
#include <osg/TriangleFunctor>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <components/debug/debuglog.hpp>
#include <components/esm3/loadarmo.hpp>
#include <components/esm3/loadcrea.hpp>
#include <components/esm3/loadgmst.hpp>
#include <components/misc/rng.hpp>
#include <components/esm3/loadligh.hpp>
#include <components/esm3/loadnpc.hpp>
#include <components/esm3/loadskil.hpp>
#include <components/esm3/loadmgef.hpp>
#include <components/esm3/loadweap.hpp>
#include <components/misc/convert.hpp>
#include <components/misc/resourcehelpers.hpp>
#include <components/misc/strings/algorithm.hpp>
#include <components/misc/strings/conversion.hpp>
#include <components/ragdollprofile/loader.hpp>
#include <components/ragdollprofile/skeletonadapter.hpp>
#include <components/resource/physicsshapemanager.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/sceneutil/attach.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/settings/values.hpp>

#include <ragdollcore/anatomy.hpp>
#include <ragdollcore/anatomicalhinge.hpp>
#include <ragdollcore/armtarget.hpp>
#include <ragdollcore/balanceauthority.hpp>
#include <ragdollcore/blow.hpp>
#include <ragdollcore/duelread.hpp>
#include <ragdollcore/bodylease.hpp>
#include <ragdollcore/contactoutcome.hpp>
#include <ragdollcore/reactioncontroller.hpp>
#include <ragdollcore/builder.hpp>
#include <ragdollcore/balancecontacts.hpp>
#include <ragdollcore/contact.hpp>
#include <ragdollcore/controller.hpp>
#include <ragdollcore/balancehelper.hpp>
#include <ragdollcore/footfall.hpp>
#include <ragdollcore/recoverycontrol.hpp>
#include <ragdollcore/fallprotect.hpp>

#include "../mwlua/kineticbindings.hpp"
#include <ragdollcore/footfallgait.hpp>
#include <ragdollcore/gaitcontroller.hpp>
#include <ragdollcore/behavior.hpp> // the region helpers (regionIsArm, ...)
#include <ragdollcore/hitreaction.hpp>
#include <ragdollcore/impactdamage.hpp>
#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/posecomposer.hpp>
#include <ragdollcore/situation.hpp>
#include <ragdollcore/staggerplan.hpp>
#include <ragdollcore/steparbiter.hpp>
#include <ragdollcore/physicaldefense.hpp>
#include <ragdollcore/referencegait.hpp>
#include <ragdollcore/replay.hpp>
#include <ragdollcore/strikecooldown.hpp>
#include <ragdollcore/supportgrip.hpp>
#include <ragdollcore/unitmodel.hpp>
#include <ragdollcore/weaponcontact.hpp>
#include <ragdollcore/weaponalignment.hpp>
#include <ragdollcore/weaponswing.hpp>

#include "../mwbase/dialoguemanager.hpp"
#include "../mwbase/environment.hpp"
#include "../mwbase/luamanager.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/soundmanager.hpp"
#include "../mwbase/world.hpp"

#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/combat.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwmechanics/aisequence.hpp"
#include "../mwmechanics/movement.hpp"
#include "../mwmechanics/ragdollattackstate.hpp"
#include "../mwmechanics/weapontype.hpp"
#include "../mwsound/sound.hpp"

#include "../mwworld/cellstore.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/globals.hpp"
#include "../mwworld/inventorystore.hpp"
#include "../mwworld/player.hpp"

#include "../mwrender/joltdebugdraw.hpp"
#include "../mwrender/animation.hpp"
#include "../mwrender/camera.hpp"
#include "../mwrender/creatureanimation.hpp"
#include "../mwrender/npcanimation.hpp"

#include "../mwworld/class.hpp"

#include "actor.hpp"
#include "joltlayers.hpp"

#include "heightfield.hpp"
#include "movementsolver.hpp"
#include "mtphysics.hpp"
#include "object.hpp"
#include "projectile.hpp"
#include "water.hpp"

#include "joltcallbacks.hpp"
#include "joltfilters.hpp"
#include "joltlisteners.hpp"

using namespace JPH;

// Callback for traces
static void TraceImpl(const char* inFMT, ...)
{
    // Format the message
    va_list list;
    va_start(list, inFMT);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), inFMT, list);
    va_end(list);

    Log(Debug::Info) << "Jolt Trace: " << buffer;
}

#ifdef JPH_ENABLE_ASSERTS

// Callback for Jolt asserts
static bool AssertFailedImpl(const char* inExpression, const char* inMessage, const char* inFile, uint inLine)
{
    Log(Debug::Error) << "Jolt Assert: " << inFile << ":" << inLine << ": (" << inExpression << ") "
                      << (inMessage != nullptr ? inMessage : "");

    // Prevent breakpoint, better to log than exit/crash in debug mode usually
    // Jolt has a tendancy to complain about alot of things even if they work fine
    return true;
}

#endif

namespace
{
    class AlwaysCollideSimShapeFilter final : public JPH::SimShapeFilter
    {
    public:
        bool ShouldCollide(const JPH::Body&, const JPH::Shape*, const JPH::SubShapeID&, const JPH::Body&,
            const JPH::Shape*, const JPH::SubShapeID&) const override
        {
            return true;
        }
    };

    AlwaysCollideSimShapeFilter sSimShapeFilter;

    // Soft root (wobbly body) settings. Temporary environment switches for
    // tuning; they move into the ragdoll settings file once tuned.
    //   OPENMW_RAGDOLL_SOFT_ROOT=1
    //   OPENMW_RAGDOLL_SOFT_ROOT_HZ=horizontal,vertical,yaw,tilt
    //   OPENMW_RAGDOLL_SOFT_ROOT_CAP=horizontal,up,down,yawTorque,tiltTorque (x body weight)
    //   OPENMW_RAGDOLL_SOFT_ROOT_ZETA=damping ratio
    //   OPENMW_RAGDOLL_SOFT_ROOT_WEIGHT=gravity support (x body weight)
    //   OPENMW_RAGDOLL_SOFT_ROOT_RADIUS=gyration radius (units)
    //   OPENMW_RAGDOLL_SOFT_ROOT_HIT=zeroAt,recoveryPerSecond,floor,toppleUpDot
    struct SoftRootHitSettings
    {
        float mZeroAt = 5000.0f; // impulse that takes all balance
        float mRecoveryPerSecond = 1.2f;
        float mFloor = 0.15f;
        float mToppleUpDot = 0.5f; // torso/pelvis tilt (cos) that counts as toppled
        float mShoveSpeed = 160.0f; // whole-body velocity change at the fall band (u/s)
        float mStumbleSeconds = 0.4f; // involuntary step time per band above the threshold
        float mStumbleThreshold = 0.35f; // weaker hits flinch without stepping
        float mGetUpStrength = 0.8f; // balance strength while getting up
    };
    //   OPENMW_RAGDOLL_SOFT_ROOT_INERTIA=maximum target acceleration (u/s^2, 0 = off)
    float softRootInertiaSettings()
    {
        static const auto value_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SOFT_ROOT_INERTIA");
            return text != nullptr ? static_cast<float>(std::atof(text)) : 0.0f;
        });
        static const auto& value = value_tuned.get();
        return value;
    }

    //   OPENMW_RAGDOLL_SOFT_ROOT_YIELD=deadZone,gain,maximumSpeed
    struct SoftRootYieldSettings
    {
        float mDeadZone = 4.0f; // units of pelvis offset before the anchor moves
        // Off by default: with the root spring's velocity feed-forward the
        // anchor and the body chased each other at mMaximumSpeed (user
        // session 2026-09-23: capsule at 250 u/s for 2.5 s with no input).
        float mGain = 0.0f; // capsule speed per unit of excess offset (1/s)
        float mMaximumSpeed = 250.0f;
    };
    const SoftRootYieldSettings& softRootYieldSettings()
    {
        static const auto settings_tuned = RagdollCore::tuned<SoftRootYieldSettings>([] {
            SoftRootYieldSettings s;
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SOFT_ROOT_YIELD"))
                std::sscanf(text, "%f,%f,%f", &s.mDeadZone, &s.mGain, &s.mMaximumSpeed);
            return s;
        });
        static const auto& settings = settings_tuned.get();
        return settings;
    }

    // Active body (soft root) settings: OPENMW_RAGDOLL_SOFT_ROOT=1 enables it;
    // OPENMW_RAGDOLL_ACTIVE_BODY names the settings file (key = value lines,
    // see files/data/ragdoll/active-body.cfg). Read once at startup.
    const RagdollCore::PassiveRagdoll::SoftRootSettings& softRootSettings()
    {
        static const auto settings_tuned = RagdollCore::tuned<RagdollCore::PassiveRagdoll::SoftRootSettings>([] {
            RagdollCore::PassiveRagdoll::SoftRootSettings s;
            const char* enabled = RagdollCore::tuningText("OPENMW_RAGDOLL_SOFT_ROOT");
            s.mEnabled = enabled != nullptr && std::atoi(enabled) != 0;
            if (const char* path = RagdollCore::tuningText("OPENMW_RAGDOLL_ACTIVE_BODY"))
            {
                for (const std::string& problem : s.mBody.load(path))
                    Log(Debug::Warning) << "Active body settings: " << problem;
                Log(Debug::Info) << "Active body settings loaded from " << path;
            }
            return s;
        });
        static const auto& settings = settings_tuned.get();
        return settings;
    }

    const RagdollCore::ActiveBodySettings& softRootHitSettings()
    {
        return softRootSettings().mBody;
    }

    void simCollideBodyVsBody(const JPH::Body& inBody1, const JPH::Body& inBody2,
        JPH::Mat44Arg inCenterOfMassTransform1, JPH::Mat44Arg inCenterOfMassTransform2,
        JPH::CollideShapeSettings& ioCollideShapeSettings, JPH::CollideShapeCollector& ioCollector,
        const JPH::ShapeFilter& inShapeFilter)
    {
        // Sensors only need overlap information, so a single deepest hit is enough.
        if (inBody1.IsSensor() || inBody2.IsSensor())
        {
            JPH::ClosestHitCollisionCollector<JPH::CollideShapeCollector> closestHitCollector;
            JPH::PhysicsSystem::sDefaultSimCollideBodyVsBody(inBody1, inBody2, inCenterOfMassTransform1,
                inCenterOfMassTransform2, ioCollideShapeSettings, closestHitCollector, inShapeFilter);
            if (closestHitCollector.HadHit())
                ioCollector.AddHit(closestHitCollector.mHit);
            return;
        }

        JPH::PhysicsSystem::sDefaultSimCollideBodyVsBody(inBody1, inBody2, inCenterOfMassTransform1,
            inCenterOfMassTransform2, ioCollideShapeSettings, ioCollector, inShapeFilter);
    }

    constexpr std::array<JPH::ObjectLayer, 12> sRaycastObjectLayers = { MWPhysics::Layers::WORLD,
        MWPhysics::Layers::DOOR, MWPhysics::Layers::ACTOR, MWPhysics::Layers::HEIGHTMAP, MWPhysics::Layers::PROJECTILE,
        MWPhysics::Layers::WATER, MWPhysics::Layers::CAMERA_ONLY, MWPhysics::Layers::VISUAL_ONLY,
        MWPhysics::Layers::DYNAMIC_WORLD, MWPhysics::Layers::DEBRIS, MWPhysics::Layers::SENSOR,
        MWPhysics::Layers::POWERED_ACTOR };

    int getQueryObjectMask(int mask, int group)
    {
        int result = 0;
        MWPhysics::JoltObjectLayerPairFilter layerPairFilter;

        for (const JPH::ObjectLayer objectLayer : sRaycastObjectLayers)
        {
            if ((mask & static_cast<int>(objectLayer)) == 0)
                continue;

            if (group != 0xff)
            {
                bool groupMatches = false;
                for (const JPH::ObjectLayer queryLayer : sRaycastObjectLayers)
                {
                    if ((group & static_cast<int>(queryLayer)) == 0)
                        continue;
                    if (layerPairFilter.ShouldCollide(queryLayer, objectLayer))
                    {
                        groupMatches = true;
                        break;
                    }
                }

                if (!groupMatches)
                    continue;
            }

            result |= static_cast<int>(objectLayer);
        }

        return result;
    }

    std::uint32_t getQueryBroadPhaseMask(int objectMask, const MWPhysics::JoltBPLayerInterface& bpLayerInterface)
    {
        std::uint32_t result = 0;
        for (const JPH::ObjectLayer objectLayer : sRaycastObjectLayers)
        {
            if ((objectMask & static_cast<int>(objectLayer)) == 0)
                continue;
            const JPH::BroadPhaseLayer broadphaseLayer = bpLayerInterface.GetBroadPhaseLayer(objectLayer);
            const auto broadPhaseLayerIndex = static_cast<std::uint32_t>(broadphaseLayer.GetValue());
            if (broadPhaseLayerIndex < 32)
                result |= 1u << broadPhaseLayerIndex;
        }

        return result;
    }

    void handleJump(const MWWorld::Ptr& ptr)
    {
        if (!ptr.getClass().isActor())
            return;
        if (ptr.getClass().getMovementSettings(ptr).mPosition[2] == 0)
            return;
        const bool isPlayer = (ptr == MWMechanics::getPlayer());
        // Advance acrobatics and set flag for GetPCJumping
        if (isPlayer)
        {
            ptr.getClass().skillUsageSucceeded(ptr, ESM::Skill::Acrobatics, ESM::Skill::Acrobatics_Jump);
            MWBase::Environment::get().getWorld()->getPlayer().setJumping(true);
        }

        // Decrease fatigue
        if (!isPlayer || !MWBase::Environment::get().getWorld()->getGodModeState())
        {
            const MWWorld::Store<ESM::GameSetting>& gmst
                = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
            const float fFatigueJumpBase = gmst.find("fFatigueJumpBase")->mValue.getFloat();
            const float fFatigueJumpMult = gmst.find("fFatigueJumpMult")->mValue.getFloat();
            const float normalizedEncumbrance = std::min(1.f, ptr.getClass().getNormalizedEncumbrance(ptr));
            const float fatigueDecrease = fFatigueJumpBase + normalizedEncumbrance * fFatigueJumpMult;
            MWMechanics::DynamicStat<float> fatigue = ptr.getClass().getCreatureStats(ptr).getFatigue();
            fatigue.setCurrent(fatigue.getCurrent() - fatigueDecrease);
            ptr.getClass().getCreatureStats(ptr).setFatigue(fatigue);
        }
        ptr.getClass().getMovementSettings(ptr).mPosition[2] = 0;
    }

}

namespace MWPhysics
{
    namespace
    {
        std::string_view recoveryDirectionName(RagdollCore::RecoveryDirection direction)
        {
            switch (direction)
            {
                case RagdollCore::RecoveryDirection::FaceUp:
                    return "face_up";
                case RagdollCore::RecoveryDirection::FaceDown:
                    return "face_down";
            }
            return "unknown";
        }

        std::string_view gaitPhaseName(RagdollCore::GaitPhase phase)
        {
            switch (phase)
            {
                case RagdollCore::GaitPhase::Idle:
                    return "idle";
                case RagdollCore::GaitPhase::LeftStance:
                    return "left_stance";
                case RagdollCore::GaitPhase::RightStance:
                    return "right_stance";
            }
            return "unknown";
        }
    }

    // Diagnostic: force every get-up to use one animation, so the two can be
    // compared on the same fall instead of whichever the noise picked.
    std::optional<RagdollCore::RecoveryDirection> sForcedRecoveryDirection;

    // The direction an actor is facing in the world, in the same convention
    // the player steering already uses.
    osg::Vec3f actorLookDirection(const MWWorld::ConstPtr& ptr)
    {
        const float yaw = ptr.getRefData().getPosition().rot[2];
        const osg::Vec3f look = osg::Quat(yaw, osg::Vec3(0, 0, -1)) * osg::Vec3f(0, 1, 0);
        return look.valid() ? look : osg::Vec3f(0, 1, 0);
    }

    osg::Quat poweredActorRotation(const MWWorld::ConstPtr& ptr)
    {
        // Capsule rotation may stay identity: Actor deliberately skips its
        // updates for rotationally invariant collision shapes. A ragdoll is
        // not rotationally invariant and must use the visible actor's frame.
        if (const auto* node = ptr.getRefData().getBaseNode())
            return node->getAttitude();
        return osg::Quat(ptr.getRefData().getPosition().rot[2], osg::Vec3f(0, 0, -1));
    }

    struct RagdollLegChain
    {
        std::size_t mHip = 0;
        std::size_t mKnee = 0;
        std::size_t mFoot = 0;
    };

    // Shoulder to hand, plus the clavicle that carries it. The clavicle plays
    // the part the pelvis plays for a leg: it is the frame the arm is solved
    // relative to, not a body the solver writes.
    struct RagdollArmChain
    {
        std::size_t mFrame = 0;
        std::size_t mShoulder = 0;
        std::size_t mElbow = 0;
        std::size_t mHand = 0;
    };

    struct LiveWeaponAttachment
    {
        std::string mRecordId;
        std::size_t mHandBody = 0;
        std::size_t mWeaponBody = 0;
        JPH::Vec3 mCenterFromHand = JPH::Vec3::sZero();
        float mItemWeight = 0;
        float mItemSpeed = 1;
        float mPhysicalMass = 1;
        float mLength = RagdollCore::UnitModel::fromMeters(0.8f);
        std::uint32_t mVelocitySolverSteps = 40;
        std::uint64_t mContacts = 0;
        std::uint64_t mGroundContacts = 0;
        std::uint64_t mOtherRagdollContacts = 0;
        // Jolt reports native impulses (kg*world-unit/s and
        // kg*world-unit^2/s). These finite limits are deliberately above the
        // static one-kilogram training-bar load and below a hard impact.
        float mLinearBreakImpulse = 10000.0f;
        float mAngularBreakImpulse = 250000.0f;
        float mLastLinearImpulse = 0.0f;
        float mLastAngularImpulse = 0.0f;
        bool mAttached = true;
        bool mCallbackArmed = true;
        std::optional<PassiveRagdollWeaponReleaseReason> mReleaseReason;
        std::uint64_t mReleaseRequests = 0;
        std::uint64_t mReleaseCount = 0;
        JPH::Vec3 mBladeAxis = JPH::Vec3::sAxisY();
        JPH::Vec3 mEdgeAxis = JPH::Vec3::sAxisX();
        JPH::Vec3 mFlatAxis = JPH::Vec3::sAxisZ();
        bool mUsesBladeSurface = false;
        // How this weapon wounds (cut, stab, bruise); every melee class.
        RagdollCore::WoundProfile mWoundProfile;
        bool mTwoHanded = false;
        // Shoulder-rest two-handed stances (long blade, close blunt, axe): a
        // drag presents a forward guard instead of carrying the rest angle.
        bool mShoulderRestStance = false;
        // A spear's hands are ~40 units apart; tip control stretched that
        // grip past a unit in 12% of dragged steps, so it keeps hand control.
        bool mWideSpearGrip = false;
        bool mShortBlade = false; // a dagger or short sword (ShortBladeOneHand)
        // A bow, crossbow or thrown weapon: a body in the hand, but no striker
        // (PassiveRagdollEntry::rangedWeaponDrawn).
        bool mRanged = false;
#ifdef OPENMW_RAGDOLL_DEV_API
        std::uint64_t mGeometryTraceId = 0;
#endif

        float tipDistanceFromHand() const { return mCenterFromHand.Dot(mBladeAxis) + 0.5f * mLength; }
    };

    // A carried shield (Alpha 1.0 "shields"): its own box body, fixed to the
    // left forearm where the mesh straps on, sized from the shield mesh. It
    // used to be only a rule (a blow on the left hand or forearm of an actor
    // carrying a shield counted as a shield block), so a blow that met the
    // shield plate but not the arm went through to the body.
    struct LiveShieldAttachment
    {
        std::string mRecordId;
        std::size_t mForearmBody = 0;
        std::size_t mShieldBody = 0;
        JPH::Vec3 mCenterFromForearm = JPH::Vec3::sZero();
    };

    // The shield record carried left, or empty; a powered rig is rebuilt
    // when it no longer matches the one it was created with.
    std::string equippedShieldId(const MWWorld::Ptr& ptr)
    {
        if (!ptr.getClass().hasInventoryStore(ptr))
            return {};
        MWWorld::InventoryStore& inventory = ptr.getClass().getInventoryStore(ptr);
        const auto left = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        if (left == inventory.end() || left->getType() != ESM::Armor::sRecordId)
            return {};
        return left->getCellRef().getRefId().serializeText();
    }

    // The carried shield's mesh: an NPC's, or an armed creature's (CREATURE-02).
    osg::Node* liveShieldNode(MWRender::Animation* animation)
    {
        if (auto* npc = dynamic_cast<MWRender::NpcAnimation*>(animation))
            return npc->getShieldNode();
        if (auto* creature = dynamic_cast<MWRender::CreatureWeaponAnimation*>(animation))
            return creature->getShieldNode();
        return nullptr;
    }

    std::optional<LiveShieldAttachment> appendLiveShield(RagdollCore::SkeletonDescriptor& skeleton,
        RagdollCore::RagdollProfile& profile, std::string recordId, float itemWeight,
        MWRender::Animation* animation)
    {
        using namespace RagdollCore;
        if (!profile.mLandmarks.mLeftHand || animation == nullptr)
            return std::nullopt;
        const auto findBody = [&](std::string_view name) -> std::optional<std::size_t> {
            const auto found = std::find_if(profile.mBodies.begin(), profile.mBodies.end(),
                [&](const BodyProfile& body) { return body.mName == name; });
            if (found == profile.mBodies.end())
                return std::nullopt;
            return static_cast<std::size_t>(std::distance(profile.mBodies.begin(), found));
        };
        const auto hand = findBody(*profile.mLandmarks.mLeftHand);
        if (!hand || !profile.mBodies[*hand].mParentBody)
            return std::nullopt;
        const auto forearm = findBody(*profile.mBodies[*hand].mParentBody);
        if (!forearm)
            return std::nullopt;
        const auto forearmBone = std::find_if(skeleton.mBones.begin(), skeleton.mBones.end(),
            [&](const SkeletonBone& bone) { return bone.mName == profile.mBodies[*forearm].mBone; });
        if (forearmBone == skeleton.mBones.end())
            return std::nullopt;
        osg::Node* shieldNode = liveShieldNode(animation);
        const osg::Node* forearmNode = animation->getNode(forearmBone->mName);
        if (!forearmNode)
            for (const auto& aliases : profile.mBoneAliases)
                if (aliases.mCanonical == forearmBone->mName)
                    for (const auto& alias : aliases.mAliases)
                        if ((forearmNode = animation->getNode(alias)))
                            break;
        if (!shieldNode || !forearmNode)
        {
            Log(Debug::Info) << "RAGDOLL_SHIELD_MESH record=" << recordId << " shield_node=" << (shieldNode != nullptr)
                             << " forearm_node=" << (forearmNode != nullptr);
            return std::nullopt;
        }
        const auto shieldPaths = shieldNode->getParentalNodePaths(animation->getObjectRoot());
        const auto forearmPaths = forearmNode->getParentalNodePaths(animation->getObjectRoot());
        if (shieldPaths.size() != 1 || forearmPaths.size() != 1)
            return std::nullopt;
        auto parentPath = shieldPaths.front();
        parentPath.pop_back(); // the visitor applies the shield node's own transform
        osg::Matrix toForearm = osg::computeLocalToWorld(parentPath)
            * osg::Matrix::inverse(osg::computeLocalToWorld(forearmPaths.front()));
        osg::ComputeBoundsVisitor visitor;
        visitor.setNodeMaskOverride(~0u);
        visitor.pushMatrix(toForearm);
        shieldNode->accept(visitor);
        const osg::BoundingBox& bounds = visitor.getBoundingBox();
        const osg::Vec3f size = bounds._max - bounds._min;
        if (!bounds.valid() || !size.valid() || size.length() < 5.0f || size.length() >= UnitModel::fromMeters(3.0f))
        {
            Log(Debug::Info) << "RAGDOLL_SHIELD_BOUNDS record=" << recordId << " rejected size=" << size.x() << ','
                             << size.y() << ',' << size.z();
            return std::nullopt;
        }
        const JPH::Vec3 offset = Misc::Convert::toJolt<JPH::Vec3>(bounds.center());
        const JPH::Vec3 halfExtents(
            std::max(size.x() * 0.5f, 1.0f), std::max(size.y() * 0.5f, 1.0f), std::max(size.z() * 0.5f, 1.0f));

        // Weighs what the record says, at the weapon scale (0.08 kg per unit:
        // a 15-unit iron shield is 1.2 kg), taken from the body's 75 kg.
        constexpr float actorMass = 75.0f;
        const float shieldMass = std::clamp(itemWeight * 0.08f, 0.5f, 3.0f);
        float bodyFractions = 0.0f;
        for (const auto& body : profile.mBodies)
            bodyFractions += body.mMassFraction;
        const float share = shieldMass / (actorMass + shieldMass);
        for (auto& body : profile.mBodies)
            body.mMassFraction *= 1.0f - share;

        SkeletonBone bone;
        bone.mName = "equipped_shield";
        bone.mParent = static_cast<int>(std::distance(skeleton.mBones.begin(), forearmBone));
        bone.mModelTransform = forearmBone->mModelTransform;
        bone.mModelTransform.mTranslation += bone.mModelTransform.mRotation * offset;
        skeleton.mBones.push_back(bone);

        BodyProfile shield;
        shield.mName = shield.mBone = bone.mName;
        shield.mParentBody = profile.mBodies[*forearm].mName;
        shield.mMassFraction = share * bodyFractions;
        shield.mShape.mType = ShapeType::Box;
        shield.mShape.mHalfExtents = halfExtents;
        // No contact with its own arm (the forearm it straps to, the hand
        // inside it); the torso and legs still meet it.
        shield.mSelfCollisionGroup = profile.mBodies[*forearm].mSelfCollisionGroup;
        shield.mConstraint = ConstraintProfile{};
        shield.mConstraint->mChildAnchor = -offset;
        shield.mConstraint->mNormalHalfConeAngle = 0;
        shield.mConstraint->mPlaneHalfConeAngle = 0;
        shield.mConstraint->mTwistMinAngle = 0;
        shield.mConstraint->mTwistMaxAngle = 0;
        shield.mConstraint->mMotor.mMaximumTorque = 0;
        profile.mBodies.push_back(shield);

        // The left arm carries the plate: holding torque for its weight at
        // each joint's lever, as the weapon arm gets for its weapon.
        const float lever = offset.Length() / UnitModel::UnitsPerMeter;
        std::optional<std::size_t> link = forearm;
        for (const float radius : { lever, lever + 0.28f, lever + 0.4f })
        {
            if (!link)
                break;
            auto& body = profile.mBodies[*link];
            if (body.mConstraint)
                body.mConstraint->mMotor.mMaximumTorque += shieldMass
                    * (UnitModel::GravityMetersPerSecondSquared * radius + 40.0f * radius * radius)
                    * UnitModel::UnitsPerMeter * UnitModel::UnitsPerMeter;
            link = body.mParentBody ? findBody(*body.mParentBody) : std::nullopt;
        }
        Log(Debug::Info) << "RAGDOLL_SHIELD_PROXY record=" << recordId << " mass=" << shieldMass
                         << " center=" << offset.GetX() << ',' << offset.GetY() << ',' << offset.GetZ()
                         << " half_extents=" << halfExtents.GetX() << ',' << halfExtents.GetY() << ','
                         << halfExtents.GetZ();
        return LiveShieldAttachment{ std::move(recordId), *forearm, profile.mBodies.size() - 1, offset };
    }

    std::string_view weaponReleaseReasonName(PassiveRagdollWeaponReleaseReason reason)
    {
        switch (reason)
        {
            case PassiveRagdollWeaponReleaseReason::Break: return "break";
            case PassiveRagdollWeaponReleaseReason::Drop: return "drop";
            case PassiveRagdollWeaponReleaseReason::Unequip: return "unequip";
            case PassiveRagdollWeaponReleaseReason::Death: return "death";
            case PassiveRagdollWeaponReleaseReason::Cleanup: return "cleanup";
        }
        return "unknown";
    }


#ifdef OPENMW_RAGDOLL_DEV_API
    // Read-only dump of the same drawable path used by ComputeBoundsVisitor.
    // Actual triangles are included because scene optimization can bake NIF
    // transforms into vertices. The collector supplies diagnostics and the
    // opt-in mesh-derived collision hull in the same hand-local frame.
    struct WeaponGeometryTriangleTrace
    {
        std::uint64_t mId = 0;
        std::size_t mDrawable = 0;
        std::size_t* mTriangleCount = nullptr;
        std::vector<JPH::Vec3>* mHullPoints = nullptr;
        osg::Matrix mToHand;

        void operator()(const osg::Vec3& a, const osg::Vec3& b, const osg::Vec3& c, bool = false)
        {
            const std::size_t triangle = (*mTriangleCount)++;
            if (mHullPoints && triangle < 16384)
                for (const osg::Vec3& point : { a, b, c })
                {
                    const osg::Vec3d hand = osg::Vec3d(point) * mToHand;
                    mHullPoints->emplace_back(hand.x(), hand.y(), hand.z());
                }
            if (mId == 0 || triangle >= 4096)
                return;
            std::ostringstream row;
            row << std::dec << std::setprecision(17)
                << "RAGDOLL_WEAPON_GEOMETRY_TRIANGLE schema=1 geometry_id=" << mId
                << " drawable=" << mDrawable << " triangle=" << triangle;
            std::size_t index = 0;
            for (const osg::Vec3& point : { a, b, c })
            {
                const osg::Vec3d hand = osg::Vec3d(point) * mToHand;
                row << " local" << index << '=' << point.x() << ',' << point.y() << ',' << point.z()
                    << " hand" << index << '=' << hand.x() << ',' << hand.y() << ',' << hand.z();
                ++index;
            }
            Log(Debug::Info) << row.str();
        }
    };

    class WeaponGeometryTraceVisitor final : public osg::NodeVisitor
    {
    public:
        WeaponGeometryTraceVisitor(std::uint64_t id, const osg::Matrix& parentToHand,
            std::vector<JPH::Vec3>* hullPoints)
            : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
            , mId(id)
            , mParentToHand(parentToHand)
            , mHullPoints(hullPoints)
        {
            setNodeMaskOverride(~0u);
        }

        void apply(osg::Drawable& drawable) override
        {
            const std::size_t index = mDrawables++;
            // OSG row-vector convention: drawable local -> visited root parent -> hand.
            const osg::Matrix toHand = osg::computeLocalToWorld(getNodePath()) * mParentToHand;
            std::ostringstream row;
            row << std::dec << std::setprecision(17)
                << "RAGDOLL_WEAPON_GEOMETRY_DRAWABLE schema=1 geometry_id=" << mId
                << " drawable=" << index << " name=" << std::quoted(drawable.getName())
                << " matrix_order=row_major_row_vector local_to_hand=";
            for (unsigned r = 0; r < 4; ++r)
                for (unsigned c = 0; c < 4; ++c)
                    row << (r == 0 && c == 0 ? "" : ",") << toHand(r, c);
            if (mId != 0)
                Log(Debug::Info) << row.str();
            osg::TriangleFunctor<WeaponGeometryTriangleTrace> triangles;
            triangles.mId = mId;
            triangles.mDrawable = index;
            triangles.mTriangleCount = &mTriangles;
            triangles.mHullPoints = mHullPoints;
            triangles.mToHand = toHand;
            const std::size_t firstTriangle = mTriangles;
            drawable.accept(triangles);
            if (mTriangles == firstTriangle && drawable.getBoundingBox().valid())
                ++mUncapturedDrawables;
        }

        std::uint64_t mId;
        osg::Matrix mParentToHand;
        std::vector<JPH::Vec3>* mHullPoints;
        std::size_t mDrawables = 0;
        std::size_t mTriangles = 0;
        std::size_t mUncapturedDrawables = 0;
    };

    std::uint64_t traceWeaponGeometry(osg::Node& node, const osg::Matrix& parentToHand,
        std::string_view record, std::string_view model, float uniformScale, std::string_view source,
        std::vector<JPH::Vec3>* hullPoints)
    {
        const bool trace = RagdollCore::tuningText("OPENMW_RAGDOLL_WEAPON_GEOMETRY_TRACE") != nullptr;
        if (!trace && !hullPoints)
            return 0;
        // Actor/mesh creation already runs on the game thread. Sequence belongs
        // only to this diagnostic and never changes an actor identity.
        static std::uint64_t nextId = 1;
        const std::uint64_t id = trace ? nextId++ : 0;
        if (trace) Log(Debug::Info) << std::dec << std::setprecision(17)
            << "RAGDOLL_WEAPON_GEOMETRY_BEGIN schema=1 geometry_id=" << id
            << " record=" << std::quoted(std::string(record))
            << " model=" << std::quoted(std::string(model)) << " source=" << source
            << " uniform_scale=" << uniformScale << " triangle_cap=4096";
        WeaponGeometryTraceVisitor visitor(id, parentToHand, hullPoints);
        node.accept(visitor);
        if (hullPoints && (visitor.mTriangles > 16384 || visitor.mUncapturedDrawables != 0))
        {
            hullPoints->clear();
            Log(Debug::Warning) << "Weapon hull capture rejected: oversized or incomplete triangle geometry for " << record;
        }
        if (trace) Log(Debug::Info) << std::dec << "RAGDOLL_WEAPON_GEOMETRY_END schema=1 geometry_id=" << id
            << " drawables=" << visitor.mDrawables << " triangles=" << visitor.mTriangles
            << " uncaptured_drawables=" << visitor.mUncapturedDrawables
            << " emitted=" << std::min<std::size_t>(visitor.mTriangles, 4096)
            << " truncated=" << (visitor.mTriangles > 4096);
        return id;
    }
#endif

    // OPENMW_RAGDOLL_SWING_RIG: 1 = the tip-controller stack (candidate 37),
    // 2 = swing rig v2 (docs/ragdoll/designs/PLAY-04-swing-rig-v2.md).
    int swingRigVersion()
    {
        static const auto version_tuned = RagdollCore::tuned<int>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SWING_RIG");
            return text ? std::atoi(text) : 1;
        });
        static const auto& version = version_tuned.get();
        return version;
    }

    std::optional<LiveWeaponAttachment> appendLiveTrainingWeapon(RagdollCore::SkeletonDescriptor& skeleton,
        RagdollCore::RagdollProfile& profile, RagdollCore::BuildOptions& options, std::string recordId,
        float itemWeight, float itemSpeed, float weaponReach, MWRender::Animation* actorAnimation)
    {
        // An NPC's or an armed creature's animation (CREATURE-02): both carry
        // the weapon mesh.
        osg::Node* liveWeapon = nullptr;
        Resource::ResourceSystem* resources = nullptr;
        if (auto* npc = dynamic_cast<MWRender::NpcAnimation*>(actorAnimation))
        {
            liveWeapon = npc->getWeaponNode();
            resources = npc->getResourceSystem();
        }
        else if (auto* creature = dynamic_cast<MWRender::CreatureWeaponAnimation*>(actorAnimation))
        {
            liveWeapon = creature->getWeaponNode();
            resources = creature->getResourceSystem();
        }
        const bool animation = resources != nullptr;
        using namespace RagdollCore;
        if (!profile.mLandmarks.mRightHand)
            return std::nullopt;
        const auto findBody = [&](std::string_view name) -> std::optional<std::size_t> {
            const auto found = std::find_if(profile.mBodies.begin(), profile.mBodies.end(),
                [&](const BodyProfile& body) { return body.mName == name; });
            if (found == profile.mBodies.end())
                return std::nullopt;
            return static_cast<std::size_t>(std::distance(profile.mBodies.begin(), found));
        };
        const auto hand = findBody(*profile.mLandmarks.mRightHand);
        if (!hand || *hand >= skeleton.mBones.size())
            return std::nullopt;
        const auto handBone = std::find_if(skeleton.mBones.begin(), skeleton.mBones.end(),
            [&](const SkeletonBone& bone) { return bone.mName == profile.mBodies[*hand].mBone; });
        if (handBone == skeleton.mBones.end())
            return std::nullopt;

        constexpr float actorMass = 75.0f;
        // Swing rig v2 (docs/ragdoll/designs/PLAY-04-swing-rig-v2.md): the
        // weapon weighs what its record says, OPENMW_RAGDOLL_WEAPON_MASS_SCALE
        // kg per weight unit (default 0.08: dagger 0.4, longsword 1.6, claymore
        // 2.2, warhammer 2.6). At 0.12 the long blades' wrists were pushed
        // 20-26 degrees past their limits. Before, every weapon was 1 kg and
        // weight existed only as a limit on the swing command.
        static const auto massScale_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_WEAPON_MASS_SCALE");
            const float value = text ? static_cast<float>(std::atof(text)) : 0.08f;
            return std::isfinite(value) && value > 0.0f ? value : 0.08f;
        });
        static const auto& massScale = massScale_tuned.get();
        const float weaponMass = swingRigVersion() >= 2 ? std::clamp(itemWeight * massScale, 0.4f, 4.0f) : 1.0f;
        const float totalMass = actorMass + weaponMass;
        // Reach is a gameplay multiplier, not a mesh dimension. Retain the
        // old bounded approximation only when no equipped mesh is available.
        const float fallbackLength = UnitModel::fromMeters(std::clamp(weaponReach, 0.3f, 1.6f));
        JPH::Vec3 halfExtents(UnitModel::fromMeters(0.02f), fallbackLength * 0.5f,
            UnitModel::fromMeters(0.01f));
        JPH::Vec3 offset(0, fallbackLength * 0.44f, 0);
        bool meshBounds = false;
        bool templateBounds = false;
#ifdef OPENMW_RAGDOLL_DEV_API
        std::uint64_t geometryTraceId = 0;
        const char* hullOption = RagdollCore::tuningText("OPENMW_RAGDOLL_WEAPON_HULL");
        const bool useHull = !hullOption || std::string_view(hullOption) != "0";
        std::vector<JPH::Vec3> hullPoints;
#endif
        const auto* record = MWBase::Environment::get().getESMStore()->get<ESM::Weapon>().search(
            ESM::RefId::deserializeText(recordId));
        const auto useBounds = [&](const osg::BoundingBox& bounds) {
            const osg::Vec3f size = bounds._max - bounds._min;
            if (!bounds.valid() || !bounds.center().valid() || !size.valid()
                || size.y() <= 0.1f || size.length() >= UnitModel::fromMeters(8.0f))
                return false;
            offset = Misc::Convert::toJolt<JPH::Vec3>(bounds.center());
            halfExtents = JPH::Vec3(std::max(size.x() * 0.5f, 0.1f),
                size.y() * 0.5f, std::max(size.z() * 0.5f, 0.1f));
            return true;
        };
        if (animation)
        {
            osg::Node* weaponNode = liveWeapon;
            const osg::Node* handNode = actorAnimation->getNode(handBone->mName);
            if (!handNode)
                for (const auto& aliases : profile.mBoneAliases)
                    if (aliases.mCanonical == handBone->mName)
                        for (const auto& alias : aliases.mAliases)
                            if ((handNode = actorAnimation->getNode(alias)))
                                break;
            if (weaponNode && handNode)
            {
                const auto weaponPaths = weaponNode->getParentalNodePaths(actorAnimation->getObjectRoot());
                const auto handPaths = handNode->getParentalNodePaths(actorAnimation->getObjectRoot());
                Log(Debug::Info) << "RAGDOLL_WEAPON_MESH record=" << recordId
                                << " weapon_paths=" << weaponPaths.size() << " hand_paths=" << handPaths.size();
                if (weaponPaths.size() == 1 && handPaths.size() == 1)
                {
                    auto parentPath = weaponPaths.front();
                    parentPath.pop_back(); // the visitor applies weaponNode's transform
                    osg::Matrix toHand = osg::computeLocalToWorld(parentPath)
                        * osg::Matrix::inverse(osg::computeLocalToWorld(handPaths.front()));
                    osg::ComputeBoundsVisitor visitor;
                    visitor.setNodeMaskOverride(~0u);
                    visitor.pushMatrix(toHand);
                    weaponNode->accept(visitor);
                    const auto& bounds = visitor.getBoundingBox();
                    const osg::Vec3f size = bounds._max - bounds._min;
                    Log(Debug::Info) << "RAGDOLL_WEAPON_BOUNDS record=" << recordId << " valid=" << bounds.valid()
                                    << " size=" << size.x() << ',' << size.y() << ',' << size.z();
                    meshBounds = useBounds(bounds);
#ifdef OPENMW_RAGDOLL_DEV_API
                    if (meshBounds)
                        geometryTraceId = traceWeaponGeometry(*weaponNode, toHand, recordId,
                            record ? std::string_view(record->mModel.getNormalized().value()) : std::string_view(),
                            options.mUniformScale, "mesh", useHull ? &hullPoints : nullptr);
#endif
                }
            }
            else
                Log(Debug::Info) << "RAGDOLL_WEAPON_MESH record=" << recordId
                                << " weapon_node=" << (weaponNode != nullptr) << " hand_node=" << (handNode != nullptr);
            // Population activation can precede the draw-animation's mesh
            // attachment. Measure a private instance using the same attachment
            // routine (including BoneOffset), without changing the live scene.
            if (!meshBounds && handNode && record && !record->mModel.getNormalized().empty())
            {
                const std::string& mountName = MWMechanics::getWeaponType(record->mData.mType)->mAttachBone;
                const osg::Node* mount = actorAnimation->getNode(mountName);
                if (!mount) mount = actorAnimation->getNode("Weapon Bone");
                if (mount)
                {
                    const auto mountPaths = mount->getParentalNodePaths(actorAnimation->getObjectRoot());
                    const auto handPaths = handNode->getParentalNodePaths(actorAnimation->getObjectRoot());
                    if (mountPaths.size() == 1 && handPaths.size() == 1)
                    {
                        try
                        {
                            osg::ref_ptr<osg::Group> root = new osg::Group;
                            osg::ref_ptr<osg::MatrixTransform> attachment = new osg::MatrixTransform;
                            attachment->setName(mount->getName());
                            attachment->setMatrix(osg::computeLocalToWorld(mountPaths.front())
                                * osg::Matrix::inverse(osg::computeLocalToWorld(handPaths.front())));
                            root->addChild(attachment);
                            auto* sceneManager = resources->getSceneManager();
                            const auto mesh = sceneManager->getTemplate(
                                Misc::ResourceHelpers::correctMeshPath(record->mModel.getNormalized()));
                            SceneUtil::attach(mesh, root, mountName, attachment, sceneManager);
                            osg::ComputeBoundsVisitor visitor;
                            visitor.setNodeMaskOverride(~0u);
                            root->accept(visitor);
                            meshBounds = templateBounds = useBounds(visitor.getBoundingBox());
#ifdef OPENMW_RAGDOLL_DEV_API
                            if (meshBounds)
                                geometryTraceId = traceWeaponGeometry(*root, osg::Matrix::identity(), recordId,
                                    record->mModel.getNormalized().value(), options.mUniformScale, "mesh_template",
                                    useHull ? &hullPoints : nullptr);
#endif
                        }
                        catch (const std::exception& error)
                        {
                            Log(Debug::Warning) << "Weapon mesh bounds unavailable for " << recordId << ": " << error.what();
                        }
                    }
                }
            }
        }
        std::array<unsigned, 3> axes{ 0, 1, 2 };
        std::stable_sort(axes.begin(), axes.end(), [&](unsigned a, unsigned b) { return halfExtents[a] > halfExtents[b]; });
        const auto axisVector = [](unsigned axis) {
            return axis == 0 ? JPH::Vec3::sAxisX() : axis == 1 ? JPH::Vec3::sAxisY() : JPH::Vec3::sAxisZ();
        };
        const JPH::Vec3 bladeAxis = axisVector(axes[0]) * (offset[axes[0]] < 0 ? -1.0f : 1.0f);
        const float weaponLength = halfExtents[axes[0]] * 2 / UnitModel::UnitsPerMeter;
        const float weaponWidth = halfExtents[axes[1]] * 2 / UnitModel::UnitsPerMeter;
        const float gripToCenter = offset.Length() / UnitModel::UnitsPerMeter;
        Log(Debug::Info) << "RAGDOLL_WEAPON_PROXY record=" << recordId
                        << " source=" << (templateBounds ? "mesh_template" : meshBounds ? "mesh" : "reach_fallback")
                        << " length=" << UnitModel::fromMeters(weaponLength) << " long_axis=" << axes[0]
                        << " center=" << offset.GetX() << ',' << offset.GetY() << ',' << offset.GetZ()
                        << " half_extents=" << halfExtents.GetX() << ',' << halfExtents.GetY() << ','
                        << halfExtents.GetZ();
        for (auto& body : profile.mBodies)
            body.mMassFraction *= actorMass / totalMass;

        SkeletonBone bone;
        bone.mName = "equipped_weapon";
        bone.mParent = static_cast<int>(std::distance(skeleton.mBones.begin(), handBone));
        bone.mModelTransform = skeleton.mBones[bone.mParent].mModelTransform;
        bone.mModelTransform.mTranslation += bone.mModelTransform.mRotation * offset;
        skeleton.mBones.push_back(bone);

        BodyProfile weapon;
        weapon.mName = weapon.mBone = bone.mName;
        weapon.mParentBody = profile.mBodies[*hand].mName;
        weapon.mMassFraction = weaponMass / totalMass;
        weapon.mShape.mType = ShapeType::Box;
        weapon.mShape.mHalfExtents = halfExtents;
#ifdef OPENMW_RAGDOLL_DEV_API
        if (useHull)
        {
            if (!meshBounds || hullPoints.empty())
            {
                Log(Debug::Warning) << "Weapon hull creation rejected: no complete mesh geometry for " << recordId;
                return std::nullopt;
            }
            // Same body origin and grip anchors. Exact mesh points replace
            // empty AABB volume; shape-derived inertia keeps the original mass.
            weapon.mShape.mType = ShapeType::ConvexHull;
            weapon.mShape.mPoints.reserve(hullPoints.size());
            for (const auto& point : hullPoints)
                weapon.mShape.mPoints.push_back(point - offset);
            Log(Debug::Info) << "RAGDOLL_WEAPON_HULL record=" << std::quoted(recordId)
                            << " points=" << hullPoints.size() << " geometry_id=" << geometryTraceId;
        }
#endif
        weapon.mConstraint = ConstraintProfile{};
        weapon.mConstraint->mChildAnchor = -offset;
        weapon.mConstraint->mNormalHalfConeAngle = 0;
        weapon.mConstraint->mPlaneHalfConeAngle = 0;
        weapon.mConstraint->mTwistMinAngle = 0;
        weapon.mConstraint->mTwistMaxAngle = 0;
        weapon.mConstraint->mMotor.mMaximumTorque = 0;
        profile.mBodies.push_back(weapon);

        std::array<std::size_t, 4> arm{};
        arm.back() = *hand;
        for (std::size_t slot = arm.size() - 1; slot > 0; --slot)
        {
            const auto& child = profile.mBodies[arm[slot]];
            if (!child.mParentBody)
                return std::nullopt;
            const auto parent = findBody(*child.mParentBody);
            if (!parent)
                return std::nullopt;
            arm[slot - 1] = *parent;
        }
        const float inertia = weaponMass * (weaponLength * weaponLength + weaponWidth * weaponWidth) / 12.0f;
        const std::array<float, 4> radii{ 1.0f, 0.9f, 0.6f, gripToCenter };
        // 8 rad/s^2 left the wrist and upper arm saturated just holding a
        // claymore at its ready stance; 40 holds it at under 0.8 utilization.
        float holdAcceleration = 40.0f;
        // Stiffness of the weapon arm's holding springs over the authored,
        // inertia-compensated value (frequency multiplier).
        float holdSpring = 1.0f;
#ifdef OPENMW_RAGDOLL_DEV_API
        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_WEAPON_HOLD_SPRING"))
            if (const float value = static_cast<float>(std::atof(text)); value > 0)
                holdSpring = value;
        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_WEAPON_HOLD_ACCEL"))
            if (const float value = static_cast<float>(std::atof(text)); value > 0)
                holdAcceleration = value;
#endif
        for (std::size_t i = 0; i < arm.size(); ++i)
        {
            auto& constraint = profile.mBodies[arm[i]].mConstraint;
            if (!constraint)
                return std::nullopt;
            const float radius = radii[i];
            const float allowance = weaponMass * UnitModel::GravityMetersPerSecondSquared * radius
                + (inertia + weaponMass * radius * radius) * holdAcceleration;
            constraint->mMotor.mMaximumTorque += allowance * UnitModel::UnitsPerMeter * UnitModel::UnitsPerMeter;
        }

        // The authored wrist (cone 0.45/0.35, twist 0.30 rad) is a corpse's:
        // it is centred on whatever pose the body was built from, and the
        // original weapon stances cock the wrist 90 degrees and more from a
        // relaxed hand. Measured at the claymore ready stance, the forearm
        // followed the animation within 9 degrees while the hand sat 115
        // degrees from it, its motor saturated against the limit, the blade
        // level instead of upright, and the support hand sent to a grip point
        // defined on that misplaced blade. A hand that holds a weapon gets the
        // wrist range its stances use; both hands for a two-handed weapon.
        {
            float normalCone = 2.0f, planeCone = 2.0f, twist = 1.6f;
#ifdef OPENMW_RAGDOLL_DEV_API
            if (const char* spec = RagdollCore::tuningText("OPENMW_RAGDOLL_WEAPON_WRIST"))
            {
                float a = 0, b = 0, c = 0;
                if (std::sscanf(spec, "%f,%f,%f", &a, &b, &c) == 3 && a > 0 && b > 0 && c > 0)
                    normalCone = a, planeCone = b, twist = c;
            }
#endif
            const auto* weaponType = record ? MWMechanics::getWeaponType(record->mData.mType) : nullptr;
            const bool bothHands = weaponType && weaponType->mWeaponClass == ESM::WeaponType::Melee
                && (weaponType->mFlags & ESM::WeaponType::TwoHanded);
            const auto openWrist = [&](std::optional<std::size_t> body) {
                if (!body || !profile.mBodies[*body].mConstraint)
                    return;
                auto& wrist = *profile.mBodies[*body].mConstraint;
                wrist.mNormalHalfConeAngle = std::max(wrist.mNormalHalfConeAngle, normalCone);
                wrist.mPlaneHalfConeAngle = std::max(wrist.mPlaneHalfConeAngle, planeCone);
                wrist.mTwistMinAngle = std::min(wrist.mTwistMinAngle, -twist);
                wrist.mTwistMaxAngle = std::max(wrist.mTwistMaxAngle, twist);
            };
            // The clavicle too (authored cone 0.25/0.20, twist 0.15): at the
            // longsword and claymore ready stances it sat 34 degrees from the
            // animation with the spine within 9, and carried the whole arm
            // chain round with it. The shoulder itself was tried and made no
            // difference to any measurement, so it keeps its authored cone.
            // Half cone in radians; zero keeps the authored clavicle. A light
            // body between a compliant trunk and a strongly driven arm: opened
            // to 0.8 it fixed the stance and then swung 74 degrees off while
            // an attack was being braked.
            static const auto clavicleCone_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_WEAPON_CLAVICLE");
                const float value = text ? static_cast<float>(std::atof(text)) : -1.0f;
                return value >= 0.0f ? value : 0.5f;
            });
            static const auto& clavicleCone = clavicleCone_tuned.get();
            const auto openClavicle = [&](std::optional<std::size_t> body) {
                for (int level = 0; level < 3 && body; ++level)
                    body = profile.mBodies[*body].mParentBody ? findBody(*profile.mBodies[*body].mParentBody)
                                                              : std::nullopt;
                if (!body || !profile.mBodies[*body].mConstraint)
                    return;
                auto& clavicle = *profile.mBodies[*body].mConstraint;
                clavicle.mNormalHalfConeAngle = std::max(clavicle.mNormalHalfConeAngle, clavicleCone);
                clavicle.mPlaneHalfConeAngle = std::max(clavicle.mPlaneHalfConeAngle, clavicleCone);
                clavicle.mTwistMinAngle = std::min(clavicle.mTwistMinAngle, -0.8f * clavicleCone);
                clavicle.mTwistMaxAngle = std::max(clavicle.mTwistMaxAngle, 0.8f * clavicleCone);
            };
            const bool clavicleOpened = clavicleCone > 0.0f;
            openWrist(hand);
            if (clavicleOpened)
                openClavicle(hand);
            if (bothHands && profile.mLandmarks.mLeftHand)
            {
                openWrist(findBody(*profile.mLandmarks.mLeftHand));
                if (clavicleOpened)
                    openClavicle(findBody(*profile.mLandmarks.mLeftHand));
            }
        }

        options.mTotalMass = totalMass;
        const auto built = buildRagdollSettings(skeleton, profile, options);
        if (!built.isValid())
            return std::nullopt;
        const auto& parts = built.mSettings->mParts;
        for (const std::size_t joint : arm)
        {
            const auto pivot = parts[joint].mPosition;
            JPH::Mat44 subtreeInertia = JPH::Mat44::sZero();
            for (std::size_t body = 0; body < parts.size(); ++body)
            {
                int ancestor = static_cast<int>(body);
                while (ancestor >= 0 && ancestor != static_cast<int>(joint))
                    ancestor = skeleton.mBones[ancestor].mParent;
                if (ancestor < 0)
                    continue;
                auto mass = parts[body].GetMassProperties();
                mass.Rotate(JPH::Mat44::sRotation(parts[body].mRotation));
                mass.Translate(JPH::Vec3(
                    parts[body].mPosition + parts[body].mRotation * parts[body].GetShape()->GetCenterOfMass() - pivot));
                subtreeInertia += mass.mInertia;
            }
            const auto worldInertia = [&](std::size_t body) {
                auto mass = parts[body].GetMassProperties();
                mass.Rotate(JPH::Mat44::sRotation(parts[body].mRotation));
                return mass.mInertia;
            };
            const int parent = skeleton.mBones[joint].mParent;
            if (parent < 0)
                return std::nullopt;
            const JPH::Mat44 inverse
                = worldInertia(joint).Inversed3x3() + worldInertia(static_cast<std::size_t>(parent)).Inversed3x3();
            // About the world vertical, so it depends on the pose the body is
            // built from: the weapon arm is built from the ready stance
            // (samplePassiveRagdollMotion), where the tuning was measured.
            const float ratio = subtreeInertia.Multiply3x3(JPH::Vec3::sAxisZ()).GetZ()
                * inverse.Multiply3x3(JPH::Vec3::sAxisZ()).GetZ();
            if (!(ratio > 0) || !std::isfinite(ratio))
                return std::nullopt;
            auto& motor = profile.mBodies[joint].mConstraint->mMotor;
            motor.mFrequency *= std::sqrt(ratio) * holdSpring;
            motor.mDamping *= std::sqrt(ratio);
            // Only a two-handed grip carries the factor into the swing drive:
            // the second hand closes the loop and steadies the stiffer wrist
            // (without it a claymore still flipped 170-180 degrees in drags
            // and after an attack). A single arm rang with any of it: after a
            // longsword tap the hand kept 400-1000 units/s for half a second,
            // against 12-22 steps to settle with none.
            const auto* loadType = record ? MWMechanics::getWeaponType(record->mData.mType) : nullptr;
            // A one-handed weapon keeps it at the wrist alone: in a dragged
            // swing the flat spring there let the blade droop and trail below
            // the hand, which is the joint that decides where a blade points.
            static const auto oneHandedWrist_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_ONE_HAND_WRIST_LOAD");
                return !(text && std::atoi(text) == 0);
            });
            static const auto& oneHandedWrist = oneHandedWrist_tuned.get();
            const bool twoHandedLoad = loadType && loadType->mWeaponClass == ESM::WeaponType::Melee
                && (loadType->mFlags & ESM::WeaponType::TwoHanded);
            if (twoHandedLoad || (oneHandedWrist && joint == *hand))
                motor.mLoadScale = std::sqrt(ratio);
            Log(Debug::Info) << "RAGDOLL_WEAPON_ARM_LOAD record=" << recordId << " body=" << profile.mBodies[joint].mName
                            << " load_scale=" << motor.mLoadScale << " frequency=" << motor.mFrequency;
        }
        LiveWeaponAttachment attached{ recordId, *hand, profile.mBodies.size() - 1, offset, itemWeight,
            itemSpeed, weaponMass, UnitModel::fromMeters(weaponLength), 40 };
#ifdef OPENMW_RAGDOLL_DEV_API
        attached.mGeometryTraceId = geometryTraceId;
#endif
        attached.mBladeAxis = bladeAxis;
        attached.mEdgeAxis = axisVector(axes[1]);
        attached.mFlatAxis = axisVector(axes[2]);
        if (record)
        {
            attached.mUsesBladeSurface = record->mData.mType == ESM::WeaponType::ShortBladeOneHand
                || record->mData.mType == ESM::WeaponType::LongBladeOneHand
                || record->mData.mType == ESM::WeaponType::LongBladeTwoHand;
            const auto* type = MWMechanics::getWeaponType(record->mData.mType);
            attached.mTwoHanded = type->mWeaponClass == ESM::WeaponType::Melee
                && (type->mFlags & ESM::WeaponType::TwoHanded);
            attached.mShoulderRestStance = attached.mTwoHanded
                && (record->mData.mType == ESM::WeaponType::LongBladeTwoHand
                    || record->mData.mType == ESM::WeaponType::BluntTwoClose
                    || record->mData.mType == ESM::WeaponType::AxeTwoHand);
            attached.mWideSpearGrip = record->mData.mType == ESM::WeaponType::SpearTwoWide;
            attached.mShortBlade = record->mData.mType == ESM::WeaponType::ShortBladeOneHand;
            attached.mRanged = type->mWeaponClass != ESM::WeaponType::Melee;
            using WoundClass = RagdollCore::WoundClass;
            const auto& weaponType = record->mData.mType;
            if (weaponType == ESM::WeaponType::ShortBladeOneHand || weaponType == ESM::WeaponType::LongBladeOneHand
                || weaponType == ESM::WeaponType::LongBladeTwoHand)
                attached.mWoundProfile = { WoundClass::Blade, 0.10f, 0.85f };
            // The head is the outer third of the haft.
            else if (weaponType == ESM::WeaponType::AxeOneHand || weaponType == ESM::WeaponType::AxeTwoHand)
                attached.mWoundProfile = { WoundClass::Axe, 0.65f, 0.95f };
            else if (weaponType == ESM::WeaponType::SpearTwoWide)
                attached.mWoundProfile = { WoundClass::Spear, 0.10f, 0.85f };
            else if (weaponType == ESM::WeaponType::BluntOneHand || weaponType == ESM::WeaponType::BluntTwoClose
                || weaponType == ESM::WeaponType::BluntTwoWide)
                attached.mWoundProfile = { WoundClass::Blunt, 0.0f, 1.0f };
            // The two measured stock blades keep their authored bands.
            if (const auto authored = RagdollCore::bladeSurfaceProfile(attached.mRecordId))
                attached.mWoundProfile = { WoundClass::Blade, authored->mBladeStart, authored->mTipStart };
        }
        return attached;
    }

    // Raise an arm's motor limits for the duration of a swing, or put them
    // back. Never leaves them raised: an arm left stiff after the swing would
    // fight the balance controller for the rest of the actor's life.
    void setArmSwingMotors(RagdollCore::PassiveRagdoll& ragdoll, const RagdollCore::RagdollProfile& profile,
        const RagdollArmChain& arm, const RagdollCore::WeaponSwingProfile& swing, bool swinging,
        bool strengthenFrame = false)
    {
        for (const std::size_t body : { arm.mFrame, arm.mShoulder, arm.mElbow, arm.mHand })
        {
            if (body >= profile.mBodies.size() || !profile.mBodies[body].mConstraint)
                continue;
            const auto& motor = profile.mBodies[body].mConstraint->mMotor;
            // The carrying clavicle follows the arm's spring response, but
            // retains its authored torque budget and angular limits: on a
            // dynamic root the shoulder reaction has to be held by the trunk.
            // With a kinematic root the trunk is anchored, so an attack that
            // follows the animation can give the clavicle the same budget;
            // otherwise the shoulder's reaction spins the light clavicle
            // instead of moving the arm.
            const float scale = swinging && (body != arm.mFrame || strengthenFrame)
                ? std::max(swing.mMotorTorqueScale, 1.0f)
                : 1.0f;
            ragdoll.setMotorMaximumTorque(body, motor.mMaximumTorque * scale);
            if (swinging)
            {
                float frequency = swing.mSpringFrequency;
                float damping = swing.mSpringDamping;
#ifdef OPENMW_RAGDOLL_DEV_API
                // Dev A/B of the strike spring: OPENMW_RAGDOLL_STRIKE_SPRING="hz,damping".
                if (strengthenFrame)
                    if (const char* spec = RagdollCore::tuningText("OPENMW_RAGDOLL_STRIKE_SPRING"))
                    {
                        float f = 0.0f, d = 0.0f;
                        if (std::sscanf(spec, "%f,%f", &f, &d) == 2 && f > 0.0f && d > 0.0f)
                        {
                            frequency = f;
                            damping = d;
                        }
                    }
#endif
                // The swing spring is stated for the arm it moves. On a joint
                // that carries a weapon Jolt would size it for the bare pair:
                // a flat 35 Hz at the wrist of a longsword behaved like 1-2
                // Hz, softer than the idle motor it replaced, and the blade
                // swung 150 degrees out after every attack without any motor
                // saturating. With load factor s the effective frequency is
                // f/s and the effective damping ratio is zeta/s (0.07 at a
                // claymore wrist). Restoring the full stiffness (f*s) with the
                // swing's torque budget rang violently, the idle hand moving
                // at 850 units/s, and sqrt(s) with critical damping still rang
                // on a single arm. A quarter power of s and an effective
                // damping ratio of 0.3 is what the two-handed grips settle
                // with (8-16 steps after an attack, no flips).
                static const auto loadRule_tuned = RagdollCore::tuned<std::pair<float, float>>([] {
                    // 0.9 (2026-10-03): at 0.3 the loaded arm rang after every
                    // stroke (FLOW-01 chained attacks: warhammer 13 blade jolts
                    // and 10 elbow snaps, battle axe 25, spear 24; at 0.9: 3, 0,
                    // 4, 0), with the sentinel's falls, duels and FIX-08 as at 0.3.
                    std::pair<float, float> rule{ 0.25f, 0.9f }; // frequency exponent, effective damping ratio
                    if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SWING_LOAD"))
                    {
                        float exponent = 0, zeta = 0;
                        if (std::sscanf(text, "%f,%f", &exponent, &zeta) == 2 && exponent >= 0 && zeta >= 0)
                            rule = { exponent, zeta };
                    }
                    return rule;
                });
                static const auto& loadRule = loadRule_tuned.get();
                const float load = std::max(motor.mLoadScale, 1.0f);
                if (load > 1.0f)
                {
                    frequency *= std::pow(load, loadRule.first);
                    damping = std::max(damping, loadRule.second * load);
                }
                ragdoll.setMotorSpring(body, frequency, damping);
            }
            else
                ragdoll.setMotorSpring(body, motor.mFrequency, motor.mDamping);
            // The shoulder alone. Its authored cone is what stops a hand
            // reaching in front of the chest, and widening it for everyone
            // was measured to topple a swinging actor, shorten the brace and
            // break the stop, because looser shoulders change the body's
            // angular momentum everywhere. Held open only while swinging.
            if (body != arm.mShoulder)
                continue;
            const auto& constraint = *profile.mBodies[body].mConstraint;
            if (swinging)
                ragdoll.setSwingTwistJointLimits(body, swing.mShoulderNormalHalfCone, swing.mShoulderPlaneHalfCone,
                    -swing.mShoulderTwistLimit, swing.mShoulderTwistLimit);
            else
                ragdoll.setSwingTwistJointLimits(body, constraint.mNormalHalfConeAngle,
                    constraint.mPlaneHalfConeAngle, constraint.mTwistMinAngle, constraint.mTwistMaxAngle);
        }
    }

    // A standing request to put one hand somewhere. Everything the arms are
    // wanted for - bracing on the way up, guarding, reaching - is this plus a
    // rule for where the point goes, so the weight ramp and the world-to-model
    // conversion are written once here rather than per behaviour.
    struct RagdollArmRequest
    {
        JPH::RVec3 mWorldTarget = JPH::RVec3::sZero();
        // Which way the elbow points away from the shoulder-to-hand line.
        JPH::Vec3 mWorldBend = JPH::Vec3(0.0f, 0.0f, -1.0f);
        std::optional<JPH::Quat> mHandRotation;
        std::optional<std::array<JPH::Quat, 3>> mRotationBlendHistory;
        std::optional<JPH::Vec3> mSolvedElbowPosition;
        std::optional<JPH::Quat> mSolvedElbowRotation;
        JPH::Vec3 mHandAimOffset = JPH::Vec3::sZero();
        bool mWeaponSupport = false;
        // A support hand's grip in the weapon's own frame (model units). When
        // set, the arm is solved toward where the weapon is being driven, not
        // toward where it physically is: a target rebuilt every step from the
        // measured weapon never asks the weapon to move, so a strong support
        // arm only braked the weapon hand that was trying to raise the blade.
        std::optional<JPH::Vec3> mWeaponLocalOffset;
        JPH::Quat mWeaponLocalRotation = JPH::Quat::sIdentity();
        JPH::Vec3 mWeaponLocalBend = JPH::Vec3(0.0f, 0.0f, -1.0f);
        // The core can orient the wrist explicitly; nothing needs it yet, so
        // the hand keeps the angle it holds against its own forearm.
        // Where the blend is going, where it is, and how fast it travels. An
        // arm that snapped to a target would fight the motors it shares with
        // the balance controller; a ramp lets both settle.
        float mRequestedWeight = 1.0f;
        float mWeight = 0.0f;
        float mBlendRate = 6.0f;
        // A released request fades to nothing and is then dropped, so a brace
        // never ends by dumping the arm.
        bool mReleasing = false;
        // Set on requests the brace owns. It only ever releases its own, so a
        // hand aimed by anything else outlives the actor standing up.
        bool mBrace = false;
        // Last solved distance between the request and what the arm reached.
        float mReachError = 0.0f;
        float mReachWithoutShoulder = -1.0f;
        bool mApplied = false;
    };

    // Development builds may override the prototype contact work profile
    // for calibration runs: "minimum impact J,full impact J,minimum cut
    // units/s,full cut units/s". Anything malformed keeps the defaults.
    // CONNECT-01 (build 61 bookmarks: 12 of 20 "didn't connect"; the
    // recording showed 75-200 eligible blade contacts in such cuts and no
    // strike): a hand control swing identity was a button press, so a chain
    // of cuts in one hold struck a foe once; any touch at 1 u/s - the blade
    // resting on the foe in the guard or brushing it in the wind-up - spent
    // that one strike, or for the longsword the contact episode's one impact;
    // and the earliest contact won over the real blow a few steps later.
    // On: every cut is its own swing and re-arms the episodes; only the cut
    // and `trailing` seconds after it (the physical blade trails the command
    // by 115-230 units) can strike; the strike is the strongest contact over
    // `window` steps from the first; a contact must approach at `approach`
    // u/s; a later impact `topUp` times the energy already paid in an
    // episode pays the difference.
    // `windCloses` 1: a new wind-up shuts the last cut's trailing window
    // (CONNECT-02, 10-07: a two-hander holds its cut stage 1 s after the
    // release, so the next press's backswing struck the foe 50 ms into the
    // wind-up - a full-price blow and a hit-stop mid wind-up, on most
    // point-blank backhands).
    // OPENMW_RAGDOLL_CONNECT = "window steps, approach, trailing s, top-up,
    // windCloses" (0 = off).
    struct ConnectSettings
    {
        bool mOn = false;
        std::uint64_t mWindow = 0;
        float mApproach = 0.0f;
        float mTrailing = 0.0f;
        float mTopUp = 0.0f;
        bool mWindCloses = false;
    };
    const ConnectSettings& connectSettings()
    {
        static const auto settings_tuned = RagdollCore::tuned<ConnectSettings>([] {
            ConnectSettings value;
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CONNECT"))
            {
                float window = 0.0f, approach = 60.0f, trailing = 0.35f, topUp = 2.0f, windCloses = 0.0f;
                if (std::sscanf(text, "%f,%f,%f,%f,%f", &window, &approach, &trailing, &topUp, &windCloses) >= 1
                    && window > 0.0f)
                {
                    value.mOn = true;
                    value.mWindow = static_cast<std::uint64_t>(std::clamp(window, 1.0f, 12.0f));
                    value.mApproach = std::clamp(approach, 0.0f, 400.0f);
                    value.mTrailing = std::clamp(trailing, 0.0f, 1.0f);
                    value.mTopUp = std::max(0.0f, topUp);
                    value.mWindCloses = windCloses > 0.0f;
                }
            }
            return value;
        });
        static const auto& settings = settings_tuned.get();
        return settings;
    }

    // Gore (stage 6, build 59 bookmark 9 "nastier cuts? bleeding?", build 61
    // bookmark 8 "more gruesome"): a cut or stab bursts `burst` x (1-3, by
    // severity) of Morrowind's blood effects at the wound, and from
    // `bleedFrom` severity up keeps bleeding for `seconds` x severity (3-9 s),
    // dropping blood every 0.5 s and taking `health` per second (never below
    // 1). OPENMW_RAGDOLL_GORE = "burst, bleed from, seconds, health" (0 = off).
    const std::array<float, 4>& goreSettings()
    {
        static const auto value_tuned = RagdollCore::tuned<std::array<float, 4>>([] {
            std::array<float, 4> v{ 0.0f, 0.35f, 6.0f, 0.0f };
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_GORE"))
                std::sscanf(text, "%f,%f,%f,%f", &v[0], &v[1], &v[2], &v[3]);
            return v;
        });
        static const auto& value = value_tuned.get();
        return value;
    }

    void spawnRagdollBlood(const MWWorld::Ptr& ptr, const osg::Vec3f& point, float scale)
    {
        const auto& settings = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
        static unsigned variant = 0;
        const ESM::GameSetting* model
            = settings.search("Blood_Model_" + std::to_string(variant++ % 3));
        if (!model || model->mValue.getString().empty())
            return;
        int bloodType = 0;
        if (ptr.getType() == ESM::REC_NPC_)
            bloodType = ptr.get<ESM::NPC>()->mBase->mBloodType;
        else if (ptr.getType() == ESM::REC_CREA)
            bloodType = ptr.get<ESM::Creature>()->mBase->mBloodType;
        const ESM::GameSetting* texture = settings.search("Blood_Texture_" + std::to_string(bloodType));
        if (!texture || texture->mValue.getString().empty())
            texture = settings.search("Blood_Texture_0");
        const VFS::Path::Normalized path("meshes/" + model->mValue.getString());
        MWBase::Environment::get().getWorld()->spawnEffect(
            path, texture ? texture->mValue.getString() : std::string(), point, scale, false, false);
    }

    bool hitSoundsEnabled()
    {
        static const auto value_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HIT_SOUNDS");
            return text && std::atoi(text) != 0;
        });
        static const auto& value = value_tuned.get();
        return value;
    }

    RagdollCore::ContactWorkProfile contactWorkProfileFromEnvironment()
    {
        RagdollCore::ContactWorkProfile profile;
        profile.mTopUpFactor = connectSettings().mTopUp;
#ifdef OPENMW_RAGDOLL_DEV_API
        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CONTACT_WORK_PROFILE"))
        {
            std::istringstream stream(text);
            RagdollCore::ContactWorkProfile parsed;
            char comma = 0;
            if (stream >> parsed.mMinimumImpactEnergy >> comma >> parsed.mFullImpactEnergy >> comma
                >> parsed.mMinimumCutSpeed >> comma >> parsed.mFullCutSpeed)
            {
                parsed.mTopUpFactor = profile.mTopUpFactor;
                profile = parsed;
            }
        }
#endif
        return profile;
    }

    // WPN-07: seconds a counted hit on one target is held before the same
    // attacker may charge condition, skill progress or a hit reaction again.
    // Contact chatter, several victim bodies and cutting installments all
    // belong to the same blow; without the gate one swing can wear a weapon
    // and grant skill many times. Dev A/B: seconds, 0 disables.
    float contactHitCooldownSeconds()
    {
        static const auto seconds_tuned = RagdollCore::tuned<float>([] {
#ifdef OPENMW_RAGDOLL_DEV_API
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HIT_COOLDOWN"))
            {
                const float value = static_cast<float>(std::atof(text));
                if (std::isfinite(value) && value >= 0.0f)
                    return std::min(value, 5.0f);
            }
#endif
            return RagdollCore::StrikeCooldown::sDefaultCooldownSeconds;
        });
        static const auto& seconds = seconds_tuned.get();
        return seconds;
    }

    float contactAxisEffectiveMass(const JPH::Body& first, const JPH::Body& second,
        JPH::RVec3Arg point, JPH::Vec3Arg normal)
    {
        const auto inverseContribution = [&](const JPH::Body& body) {
            if (!body.IsDynamic())
                return 0.0f;
            const JPH::MotionProperties* motion = body.GetMotionPropertiesUnchecked();
            if (motion == nullptr)
                return 0.0f;
            const JPH::Vec3 lever(point - body.GetCenterOfMassPosition());
            const JPH::Vec3 leverCrossNormal = lever.Cross(normal);
            return motion->GetInverseMass()
                + body.GetInverseInertia().Multiply3x3(leverCrossNormal).Dot(leverCrossNormal);
        };
        const float inverseMass = inverseContribution(first) + inverseContribution(second);
        return std::isfinite(inverseMass) && inverseMass > 0.0f ? 1.0f / inverseMass : 0.0f;
    }

    namespace
    {
        // Hit-region tuning. Flatter than a full anatomical model on purpose:
        // skill should still matter more than location, but a head blow reads
        // heavier than a forearm graze and the legs are what can drop a body.
        // User bookmark 11 (2026-10-02): "hits in arms and legs should do
        // less damage, more the closer the hit was to the chest; head hits a
        // bit more damage and more stagger". OPENMW_RAGDOLL_REGION_DAMAGE=1 =
        // the older flat table (head 1.25 ... hands 0.7).
        // User 10-29: "squeeze everything so it's closer to 1, maybe 1.2"
        // (the clash and guard already stop many blows): the 10-02 table
        // mapped linearly from 0.4-1.5 onto 1.0-1.2, the order kept - hands
        // 1.0, head 1.2. On the 10-29 Ashinabi run the player's 53 blows
        // would have dealt x1.13. OPENMW_RAGDOLL_REGION_DAMAGE = 1 the old
        // flat table, 2 the 10-02 table.
        float regionDamageScale(RagdollCore::AnatomicalRegion region) noexcept
        {
            static const auto table_tuned = RagdollCore::tuned<int>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_REGION_DAMAGE");
                return text ? std::atoi(text) : 0;
            });
            const int table = table_tuned.get();
            using R = RagdollCore::AnatomicalRegion;
            if (table == 0)
                switch (region)
                {
                    case R::Head: return 1.2f;
                    case R::Neck: return 1.18f;
                    case R::Chest: return 1.15f;
                    case R::Abdomen: return 1.13f;
                    case R::Pelvis: return 1.11f;
                    case R::LeftClavicle:
                    case R::RightClavicle: return 1.08f;
                    case R::LeftUpperArm:
                    case R::RightUpperArm:
                    case R::LeftThigh:
                    case R::RightThigh: return 1.05f;
                    case R::LeftCalf:
                    case R::RightCalf: return 1.03f;
                    case R::LeftForearm:
                    case R::RightForearm: return 1.02f;
                    case R::LeftHand:
                    case R::RightHand:
                    case R::LeftFoot:
                    case R::RightFoot: return 1.0f;
                    case R::Unknown: return 1.0f;
                }
            const bool steep = table != 1;
            switch (region)
            {
                case R::Head: return steep ? 1.5f : 1.25f;
                case R::Neck: return steep ? 1.4f : 1.2f;
                case R::Chest: return steep ? 1.2f : 1.1f;
                case R::Abdomen: return steep ? 1.1f : 1.0f;
                case R::Pelvis: return 1.0f;
                case R::LeftClavicle:
                case R::RightClavicle: return 0.85f;
                case R::LeftUpperArm:
                case R::RightUpperArm: return steep ? 0.65f : 0.8f;
                case R::LeftForearm:
                case R::RightForearm: return steep ? 0.5f : 0.75f;
                case R::LeftHand:
                case R::RightHand: return steep ? 0.4f : 0.7f;
                case R::LeftThigh:
                case R::RightThigh: return steep ? 0.7f : 0.75f;
                case R::LeftCalf:
                case R::RightCalf: return steep ? 0.55f : 0.7f;
                case R::LeftFoot:
                case R::RightFoot: return steep ? 0.45f : 0.7f;
                case R::Unknown: return 1.0f;
            }
            return 1.0f;
        }

        // How much a partial blow pays (user 10-29: "lets let partial cuts pay
        // a larger share"; fights took 9-20 landed blows, 40-100 s, against
        // vanilla's ~5-8): a glancing approach (the blade meets the body at a
        // shallow angle, was 0.25), a contact glance (it skids or nicks off,
        // was 0.3), and the floor of a landed blow's whole partial scale (they
        // multiplied down to 0.04-0.075). Blocks and parries are untouched; the
        // failed roll's share is OPENMW_RAGDOLL_GLANCE_SHARE (launcher).
        // OPENMW_RAGDOLL_PARTIAL_PAY = "glancing, contact glance, floor"
        // (0.25,0.3,0 = before 10-29; 0.5,0.5,0.3 = before 10-30, when the user
        // asked that "all hits tend to 100%": now 0.75,0.75,0.6).
        const std::array<float, 3>& partialPaySettings()
        {
            static const auto value_tuned = RagdollCore::tuned<std::array<float, 3>>([] {
                std::array<float, 3> value{ 0.75f, 0.75f, 0.6f };
                if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_PARTIAL_PAY"))
                    std::sscanf(text, "%f,%f,%f", &value[0], &value[1], &value[2]);
                for (float& v : value)
                    v = std::clamp(v, 0.0f, 1.0f);
                return value;
            });
            return value_tuned.get();
        }

        float regionReactionScale(RagdollCore::AnatomicalRegion region) noexcept
        {
            switch (region)
            {
                case RagdollCore::AnatomicalRegion::Head: return 1.5f;
                case RagdollCore::AnatomicalRegion::Neck: return 1.4f;
                case RagdollCore::AnatomicalRegion::Chest: return 1.15f;
                case RagdollCore::AnatomicalRegion::Abdomen: return 1.1f;
                case RagdollCore::AnatomicalRegion::Pelvis: return 1.25f;
                case RagdollCore::AnatomicalRegion::LeftClavicle:
                case RagdollCore::AnatomicalRegion::RightClavicle: return 0.9f;
                case RagdollCore::AnatomicalRegion::LeftUpperArm:
                case RagdollCore::AnatomicalRegion::RightUpperArm: return 0.9f;
                case RagdollCore::AnatomicalRegion::LeftForearm:
                case RagdollCore::AnatomicalRegion::RightForearm: return 0.85f;
                case RagdollCore::AnatomicalRegion::LeftHand:
                case RagdollCore::AnatomicalRegion::RightHand: return 0.8f;
                case RagdollCore::AnatomicalRegion::LeftThigh:
                case RagdollCore::AnatomicalRegion::RightThigh: return 1.1f;
                case RagdollCore::AnatomicalRegion::LeftCalf:
                case RagdollCore::AnatomicalRegion::RightCalf: return 1.0f;
                case RagdollCore::AnatomicalRegion::LeftFoot:
                case RagdollCore::AnatomicalRegion::RightFoot: return 0.95f;
                case RagdollCore::AnatomicalRegion::Unknown: return 1.0f;
            }
            return 1.0f;
        }

        // How hard the armour worn over a struck region is, as the share of
        // the blade's approach speed it throws back (restitution): heavy 0.8,
        // medium 0.55, light 0.3, cloth or skin 0.
        // Sparks where metal meets metal (user 2026-09-30: "add sparks when
        // metals connect"): bright streaks thrown off the struck surface,
        // sized by the approach speed. OPENMW_RAGDOLL_SPARKS=0 = off.
        void spawnSparks(JPH::RVec3Arg point, JPH::Vec3Arg normal, float approach)
        {
            static const auto enabled_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SPARKS");
                return !(text && std::atoi(text) == 0);
            });
            static const auto& enabled = enabled_tuned.get();
            if (!enabled || approach < 120.0f)
                return;
            // A burst of bright streaks off the struck surface (user 2026-09-30:
            // the blood-burst puffs with a spark texture "look like smoke").
            // Strength by approach speed: 120 u/s a few sparks, 900+ a shower.
            const float strength = std::clamp((approach - 120.0f) / 780.0f, 0.0f, 1.0f);
            MWBase::Environment::get().getWorld()->spawnSparks(
                osg::Vec3f(static_cast<float>(point.GetX()), static_cast<float>(point.GetY()), static_cast<float>(point.GetZ())),
                osg::Vec3f(normal.GetX(), normal.GetY(), normal.GetZ()), strength);
        }

        float armorHardnessAt(const MWWorld::Ptr& victim, RagdollCore::AnatomicalRegion region)
        {
            using Region = RagdollCore::AnatomicalRegion;
            if (victim.isEmpty() || !victim.getClass().hasInventoryStore(victim))
                return 0.0f;
            int slot = -1;
            switch (region)
            {
                case Region::Head:
                case Region::Neck: slot = MWWorld::InventoryStore::Slot_Helmet; break;
                case Region::Chest:
                case Region::Abdomen: slot = MWWorld::InventoryStore::Slot_Cuirass; break;
                case Region::Pelvis:
                case Region::LeftThigh:
                case Region::RightThigh:
                case Region::LeftCalf:
                case Region::RightCalf: slot = MWWorld::InventoryStore::Slot_Greaves; break;
                case Region::LeftClavicle:
                case Region::LeftUpperArm: slot = MWWorld::InventoryStore::Slot_LeftPauldron; break;
                case Region::RightClavicle:
                case Region::RightUpperArm: slot = MWWorld::InventoryStore::Slot_RightPauldron; break;
                case Region::LeftForearm:
                case Region::LeftHand: slot = MWWorld::InventoryStore::Slot_LeftGauntlet; break;
                case Region::RightForearm:
                case Region::RightHand: slot = MWWorld::InventoryStore::Slot_RightGauntlet; break;
                case Region::LeftFoot:
                case Region::RightFoot: slot = MWWorld::InventoryStore::Slot_Boots; break;
                case Region::Unknown: return 0.0f;
            }
            MWWorld::InventoryStore& inventory = victim.getClass().getInventoryStore(victim);
            const auto item = inventory.getSlot(slot);
            if (item == inventory.end() || item->getType() != ESM::Armor::sRecordId)
                return 0.0f;
            const ESM::RefId skill = item->getClass().getEquipmentSkill(*item);
            if (skill == ESM::Skill::HeavyArmor)
                return 0.8f;
            if (skill == ESM::Skill::MediumArmor)
                return 0.55f;
            return 0.3f;
        }

        // Impact feel (user 2026-09-29). OPENMW_RAGDOLL_IMPACT_BOUNCE =
        // "scale,cap": a landed blow throws the weapon back at hardness x scale
        // of its approach speed (weapon body mass x speed, capped); 0 = off.
        // OPENMW_RAGDOLL_HITSTOP = "seconds,min speed": a blow at least that
        // fast holds the weapon (both, blade on blade) still that long.
        // OPENMW_RAGDOLL_IMPACT_PUSH = "scale,max body speed": the struck body
        // part takes the weapon's carried mass x approach speed x scale, at
        // most that change of speed; 0 = off.
        const std::array<float, 2>& impactBounceSetting()
        {
            static const auto value_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
                std::array<float, 2> v{ 1.0f, 900.0f };
                if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_IMPACT_BOUNCE"))
                    std::sscanf(text, "%f,%f", &v[0], &v[1]);
                return v;
            });
            static const auto& value = value_tuned.get();
            return value;
        }
        const std::array<float, 2>& hitStopSetting()
        {
            static const auto value_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
                std::array<float, 2> v{ 0.05f, 150.0f };
                if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HITSTOP"))
                    std::sscanf(text, "%f,%f", &v[0], &v[1]);
                return v;
            });
            static const auto& value = value_tuned.get();
            return value;
        }
        const std::array<float, 2>& impactPushSetting()
        {
            static const auto value_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
                // 2,1500 since 2026-10-02 (alpha 0.8 force): the 250 u/s cap
                // held a head blow under ~1250, which moves a standing body's
                // head about 4 units (PUSH-01: 3000 -> 9, 30000 -> 72 and a
                // fall); FIX-08 guided hits 15/15. 4,3000 since 2026-10-02
                // (user bookmark 10 "hits landed should push the enemy ragdoll
                // in that spot more"): FIX-08 drags x2 struck spot 2-15 ->
                // 2-42 units, chest 0.1-8; 6,4500 shoved a dummy 95 units and
                // the next blows missed.
                std::array<float, 2> v{ 4.0f, 3000.0f };
                if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_IMPACT_PUSH"))
                    std::sscanf(text, "%f,%f", &v[0], &v[1]);
                return v;
            });
            static const auto& value = value_tuned.get();
            return value;
        }
    }

#include "passiveragdollentry.inl"

    struct PassiveRagdollMotionHistory
    {
        SceneUtil::Skeleton* mSkeleton = nullptr;
        std::unique_ptr<RagdollProfile::SkeletonPoseBinding> mPoseBinding;
        RagdollCore::MotionPose mPreviousPose;
        std::vector<RagdollCore::BodyMotion> mLatestMotion;
    };

    struct PassiveRagdollReplayRecording
    {
        const MWWorld::LiveCellRefBase* mRef = nullptr;
        RagdollCore::Replay mReplay;
        std::optional<std::size_t> mMaximumFrames;
        std::int64_t mFirstFallingFrame = -1;
        bool mFailed = false;
    };
}

namespace
{
    void logRagdollIssues(const MWWorld::Ptr& ptr, std::string_view stage,
        const std::vector<RagdollProfile::SkeletonSnapshotIssue>& issues)
    {
        for (const auto& issue : issues)
            Log(Debug::Warning) << "Passive ragdoll " << stage << " failed for " << ptr.getCellRef().getRefId()
                                << " at " << issue.mPath << ": " << issue.mMessage;
    }

    void logRagdollIssues(
        const MWWorld::Ptr& ptr, std::string_view stage, const RagdollCore::ValidationResult& validation)
    {
        for (const auto& issue : validation.mIssues)
            Log(Debug::Warning) << "Passive ragdoll " << stage << " failed for " << ptr.getCellRef().getRefId()
                                << " at " << issue.mPath << ": " << issue.mMessage;
    }

    std::optional<RagdollCore::MotionPose> makeMotionPose(
        const MWWorld::Ptr& ptr, const MWPhysics::Actor& actor, std::vector<RagdollCore::Transform> bodyTransforms)
    {
        const osg::Vec3f position = ptr.getRefData().getPosition().asVec3();
        const osg::Vec3f scale = actor.getScale();
        if (!position.valid() || !scale.valid() || scale.x() <= 0.0f || scale.y() <= 0.0f || scale.z() <= 0.0f
            || std::abs(scale.x() - scale.y()) > 1e-3f || std::abs(scale.x() - scale.z()) > 1e-3f)
            return std::nullopt;

        RagdollCore::MotionPose result;
        result.mRootTransform.mTranslation = Misc::Convert::toJolt<JPH::Vec3>(position);
        result.mRootTransform.mRotation = Misc::Convert::toJolt(actor.getRotation());
        result.mUniformScale = scale.x();
        result.mBodyTransforms = std::move(bodyTransforms);
        return result;
    }
}

namespace MWPhysics
{
    // If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the
    // simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
    static const int cCollisionSteps = 1;

    // This is the max amount of rigid bodies that you can add to the physics system
    static const unsigned int cMaxBodies = 65536;

    // This determines how many mutexes to allocate to protect rigid bodies from concurrent access. Set it to 0 for the
    // default settings.
    static const unsigned int cNumBodyMutexes = 0;

    // This is the max amount of body pairs that can be queued at any time (the broad phase will detect overlapping
    // body pairs based on their bounding boxes and will insert them into a queue for the narrowphase). If you make this
    // buffer too small the queue will fill up and the broad phase jobs will start to do narrow phase work. This is
    // slightly less efficient.
    static const unsigned int cMaxBodyPairs = 65536;

    // This is the maximum size of the contact constraint buffer. If more contacts (collisions between bodies) are
    // detected than this number then these contacts will be ignored and bodies will start interpenetrating / fall
    // through the world.
    static const unsigned int cMaxContactConstraints = 10240;

    namespace
    {
        LockingPolicy detectLockingPolicy()
        {
            if (Settings::physics().mAsyncNumThreads < 1)
                return LockingPolicy::NoLocks;

            return LockingPolicy::AllowSharedLocks;
        }

        unsigned getNumThreads(LockingPolicy lockingPolicy)
        {
            switch (lockingPolicy)
            {
                case LockingPolicy::NoLocks:
                    return 0;
                case LockingPolicy::AllowSharedLocks:
                {
                    const int configuredThreads = Settings::physics().mAsyncNumThreads;
                    const unsigned requested = configuredThreads > 0 ? static_cast<unsigned>(configuredThreads) : 0u;
                    return std::min(requested, std::thread::hardware_concurrency());
                }
            }

            throw std::runtime_error("Unsupported LockingPolicy: "
                + std::to_string(static_cast<std::underlying_type_t<LockingPolicy>>(lockingPolicy)));
        }
    }

    // One settings file for the ragdoll (plan phase 1): OPENMW_RAGDOLL_SETTINGS
    // names a file of "OPENMW_RAGDOLL_X = value" lines ('#' comments). Each
    // entry becomes that environment variable unless it is already set, so
    // every existing knob reads it and an explicit variable still wins. Runs
    // before any setting is first read.
    void PhysicsSystem::loadRagdollSettingsFile()
    {
        const auto load = [](const std::filesystem::path& path, bool required) {
            std::ifstream file(path);
            if (!file)
            {
                if (required)
                    Log(Debug::Warning) << "Ragdoll settings file not found: " << path;
                return;
            }
            std::string line;
            int applied = 0;
            while (std::getline(file, line))
            {
                const auto comment = line.find('#');
                if (comment != std::string::npos)
                    line.erase(comment);
                const auto equals = line.find('=');
                if (equals == std::string::npos)
                    continue;
                const auto trim = [](std::string text) {
                    const auto first = text.find_first_not_of(" \t\r");
                    const auto last = text.find_last_not_of(" \t\r");
                    return first == std::string::npos ? std::string() : text.substr(first, last - first + 1);
                };
                const std::string key = trim(line.substr(0, equals));
                const std::string value = trim(line.substr(equals + 1));
                if (key.rfind("OPENMW_RAGDOLL_", 0) != 0 || std::getenv(key.c_str()) != nullptr)
                    continue;
#ifdef _WIN32
                _putenv_s(key.c_str(), value.c_str());
#else
                setenv(key.c_str(), value.c_str(), 0);
#endif
                ++applied;
            }
            Log(Debug::Info) << "Ragdoll settings: " << applied << " values from " << path;
        };
        // The in-game options page's switches first (KINETIC menu), so they
        // win over the settings file; an environment variable wins over both.
        if (const auto switches = MWLua::kineticSwitchesPath(); !switches.empty())
            load(switches, false);
        const char* path = RagdollCore::tuningText("OPENMW_RAGDOLL_SETTINGS");
        if (path != nullptr && *path != '\0')
            load(std::filesystem::path(path), true);
        // The options page's sliders win over all of it (applied at once).
        MWLua::loadKineticTuning();
        RagdollCore::refreshTuning();
        // A list switch with the wrong number of fields (RagdollCore::tuningList).
        RagdollCore::setTuningWarningSink([](const std::string& message) { Log(Debug::Warning) << message; });
    }

    PhysicsSystem::PhysicsSystem(Resource::ResourceSystem* resourceSystem, osg::ref_ptr<osg::Group> parentNode)
        : mShapeManager(std::make_unique<Resource::PhysicsShapeManager>(resourceSystem->getVFS(),
              resourceSystem->getSceneManager(), resourceSystem->getNifFileManager(),
              Settings::cells().mCacheExpiryDelay))
        , mResourceSystem(resourceSystem)
        , mDebugDrawEnabled(false)
        , mTimeAccum(0.0f)
        , mProjectileId(0)
        , mWaterHeight(0)
        , mWaterEnabled(false)
        , mParentNode(std::move(parentNode))
        , mPhysicsDt(1.f / 60.f)
    {
        loadRagdollSettingsFile();
        mResourceSystem->addResourceManager(mShapeManager.get());

        // Register allocation hook and callbacks
        JPH::RegisterDefaultAllocator();
        JPH::Trace = TraceImpl;
        JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

        // Create a factory
        JPH::Factory::sInstance = new JPH::Factory();

        // Register all Jolt physics types
        JPH::RegisterTypes();

        // Mark this as main thread for profiling
        JPH_PROFILE_START("Main");

        // Check if a user decided to override a physics system FPS
        if (const char* env = getenv("OPENMW_PHYSICS_FPS"))
        {
            if (const auto physFramerate = Misc::StringUtils::toNumeric<float>(env);
                physFramerate.has_value() && *physFramerate > 0)
            {
                mPhysicsDt = 1.f / *physFramerate;
                Log(Debug::Warning) << "Warning: using custom physics framerate (" << *physFramerate << " FPS).";
            }
        }

        // We need a temp allocator for temporary allocations during the physics update.
        // Pre-allocatings 25 MB to avoid having to do allocations during the physics update.
        mMemoryAllocator = std::make_unique<JPH::TempAllocatorImpl>(25 * 1024 * 1024);

        // Now we can create the actual physics system.
        mPhysicsSystem.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, mBPLayerInterface,
            mObjectVsBPLayerFilter, mObjectVsObjectLayerFilter);
        mPhysicsSystem.SetSimShapeFilter(&sSimShapeFilter);
        mPhysicsSystem.SetSimCollideBodyVsBody(simCollideBodyVsBody);
        mPhysicsSystem.SetContactListener(&mContactListener);

        // The Jolt port keeps simulation coordinates in OpenMW world units,
        // while Jolt's default length and linear-velocity tolerances are
        // specified in metres. Scale every dimensional PhysicsSettings value
        // once at the engine boundary; angular and time values remain unchanged.
        JPH::PhysicsSettings physicsSettings = mPhysicsSystem.GetPhysicsSettings();
        constexpr float unitScale = Constants::UnitsPerMeter;
        constexpr float unitScaleSquared = unitScale * unitScale;
        physicsSettings.mSpeculativeContactDistance *= unitScale;
        physicsSettings.mPenetrationSlop *= unitScale;
        physicsSettings.mManifoldTolerance *= unitScale;
        physicsSettings.mMaxPenetrationDistance *= unitScale;
        physicsSettings.mBodyPairCacheMaxDeltaPositionSq *= unitScaleSquared;
        physicsSettings.mContactPointPreserveLambdaMaxDistSq *= unitScaleSquared;
        physicsSettings.mMinVelocityForRestitution *= unitScale;
        physicsSettings.mPointVelocitySleepThreshold *= unitScale;
        // ENV-GATED (default unchanged, 2026-09-11, strafe-native-input):
        // Jolt's manifold reduction merges the per-triangle manifolds of a
        // foot straddling a coplanar floor edge into one reduced manifold
        // that loses the foot's far corners; the walker then loads that
        // foot wrongly and its weight transfer stalls. Offline grid-3 at
        // 60 Hz on a mesh floor: 45% falls with reduction, 23% without
        // (tiled floor 60% -> 22%); the box floor is unaffected (2%).
        // Default OFF since 2026-09-11 (tiled floor 25.7% -> 1.7% falls offline on the game body); OPENMW_RAGDOLL_PHYS_MANIFOLD=1 restores Jolt's default.
        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_PHYS_MANIFOLD"); !(text && std::atoi(text) != 0))
            physicsSettings.mUseManifoldReduction = false;
#ifdef OPENMW_RAGDOLL_DEV_API
        // Match the offline velocity-step experiment without changing a
        // production default or the fixed timestep. Log achieved settings so
        // native evidence identifies the actual solver budget.
        if (const char* env = std::getenv("OPENMW_JOLT_VELOCITY_STEPS"))
        {
            const auto steps = Misc::StringUtils::toNumeric<unsigned>(env);
            if (steps && *steps >= 1 && *steps <= 64)
                physicsSettings.mNumVelocitySteps = *steps;
            else
                Log(Debug::Warning) << "Ignoring invalid OPENMW_JOLT_VELOCITY_STEPS (expected 1..64).";
        }
        Log(Debug::Info) << "Jolt solver budget: velocity=" << physicsSettings.mNumVelocitySteps
                         << " position=" << physicsSettings.mNumPositionSteps;
#endif
        mPhysicsSystem.SetPhysicsSettings(physicsSettings);

        // OpenMW is Z-up and GravityConst is a positive magnitude. Jolt expects
        // the signed acceleration vector, so gravity must point toward -Z.
        mPhysicsSystem.SetGravity(JPH::Vec3(0, 0, -Constants::GravityConst * Constants::UnitsPerMeter));

        // Debug helper
        mJoltDebugDrawer = std::make_unique<MWRender::JoltDebugDrawer>(mParentNode, &mPhysicsSystem, mDebugDrawEnabled);

        // Detect number of wanted async threads, if 0 then use single threaded job system
        unsigned numThreads = getNumThreads(detectLockingPolicy());
        if (numThreads > 0)
        {
            mPhysicsJobSystem
                = std::make_unique<JPH::JobSystemThreadPool>(cMaxPhysicsJobs, cMaxPhysicsBarriers, numThreads);
        }
        else
        {
            mPhysicsJobSystem = std::make_unique<JPH::JobSystemSingleThreaded>(cMaxPhysicsJobs);
        }

        // Create a job scheduler responsible for simulating the game world (actors etc)
        mTaskScheduler = std::make_unique<PhysicsTaskScheduler>(
            mPhysicsDt, &mPhysicsSystem, mJoltDebugDrawer.get(), mPhysicsJobSystem.get());
    }

    PhysicsSystem::~PhysicsSystem()
    {
        Log(Debug::Info) << "Destroying physics system...";

        mResourceSystem->removeResourceManager(mShapeManager.get());
        mWaterInstance.reset();
        mTaskScheduler->releaseSharedStates();
        mHeightFields.clear();
        mObjects.clear();
        for (auto& [_, entry] : mPassiveRagdolls)
            entry->invalidateBalanceSnapshot("physics-world-cleared");
        mPassiveRagdolls.clear();
        mPassiveRagdollMotionHistory.clear();
        mActors.clear();
        mProjectiles.clear();

        // Pre-emptive end jobs system
        mPhysicsJobSystem.reset();

        // Unregisters all types with the factory and cleans up the defaults
        UnregisterTypes();

        // Destroy the factory
        delete Factory::sInstance;
        Factory::sInstance = nullptr;

        // Signal to end profiling
        JPH_PROFILE_END();
    }

    Resource::PhysicsShapeManager* PhysicsSystem::getShapeManager()
    {
        return mShapeManager.get();
    }

    bool PhysicsSystem::toggleDebugRendering()
    {
        mDebugDrawEnabled = !mDebugDrawEnabled;

        mJoltDebugDrawer->setDebugMode(mDebugDrawEnabled);
        return mDebugDrawEnabled;
    }

    void PhysicsSystem::markAsNonSolid(const MWWorld::ConstPtr& ptr)
    {
        ObjectMap::iterator found = mObjects.find(ptr.mRef);
        if (found == mObjects.end())
            return;

        found->second->setSolid(false);
    }

    bool PhysicsSystem::isOnSolidGround(const MWWorld::Ptr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (!physactor || !physactor->getOnGround() || !physactor->getCollisionMode())
            return false;

        const auto obj = physactor->getStandingOnPtr();
        if (obj.isEmpty())
            return true; // assume standing on terrain (which is a non-object, so not collision tracked)

        ObjectMap::const_iterator foundObj = mObjects.find(obj.mRef);
        if (foundObj == mObjects.end())
            return false;

        if (!foundObj->second->isSolid())
            return false;

        return true;
    }

    RayCastingResult PhysicsSystem::castRay(const osg::Vec3f& from, const osg::Vec3f& to,
        const std::vector<MWWorld::ConstPtr>& ignore, const std::vector<MWWorld::Ptr>& targets, int mask,
        int group) const
    {
        if (from == to)
        {
            RayCastingResult result;
            result.mHit = false;
            return result;
        }

        JoltTargetBodiesFilter bodyFilter;

        for (const auto& ptr : ignore)
        {
            if (!ptr.isEmpty())
            {
                const Actor* actor = getActor(ptr);
                if (actor)
                    bodyFilter.IgnoreBody(actor->getPhysicsBody());
                else
                {
                    const Object* object = getObject(ptr);
                    if (object)
                        bodyFilter.IgnoreBody(object->getPhysicsBody());
                }
            }
        }

        if (!targets.empty())
        {
            for (const MWWorld::Ptr& target : targets)
            {
                const Actor* actor = getActor(target);
                if (actor)
                    bodyFilter.PushTarget(actor->getPhysicsBody());
            }
        }

        JPH::RVec3 rayOrigin = Misc::Convert::toJolt<JPH::RVec3>(from);
        auto diff = to - from;
        JPH::RRayCast ray(rayOrigin, Misc::Convert::toJolt<JPH::Vec3>(diff));

        const int queryObjectMask = getQueryObjectMask(mask, group);
        if (queryObjectMask == 0)
        {
            RayCastingResult result;
            result.mHit = false;
            return result;
        }

        const std::uint32_t queryBroadPhaseMask = getQueryBroadPhaseMask(queryObjectMask, mBPLayerInterface);
        if (queryBroadPhaseMask == 0)
        {
            RayCastingResult result;
            result.mHit = false;
            return result;
        }
        MaskedBroadPhaseLayerFilter broadphaseLayerFilter(queryBroadPhaseMask);
        MaskedObjectLayerFilter objectLayerFilter(queryObjectMask);

        // Cast ray and return closest hit
        JPH::RayCastResult ioHit;
        const bool didRayHit = mPhysicsSystem.GetNarrowPhaseQuery().CastRay(
            ray, ioHit, broadphaseLayerFilter, objectLayerFilter, bodyFilter);

        RayCastingResult result;
        result.mHit = didRayHit;
        if (result.mHit)
        {
            auto outPosition = ray.GetPointOnRay(ioHit.mFraction);
            result.mHitPos = Misc::Convert::toOsg(outPosition);
            JPH::BodyLockRead lock(mPhysicsSystem.GetBodyLockInterface(), ioHit.mBodyID);
            if (lock.Succeeded())
            {
                const JPH::Body& hitBody = lock.GetBody();
                result.mHitNormal
                    = Misc::Convert::toOsg(hitBody.GetWorldSpaceSurfaceNormal(ioHit.mSubShapeID2, outPosition));

                PtrHolder* ptrHolder = Misc::Convert::toPointerFromUserData<PtrHolder>(hitBody.GetUserData());
                if (ptrHolder)
                    result.mHitObject = ptrHolder->getPtr();
            }
        }
        return result;
    }

    RayCastingResult PhysicsSystem::castSphere(
        const osg::Vec3f& from, const osg::Vec3f& to, float radius, int mask, int group) const
    {
        JPH::SphereShape sphere(radius);
        sphere.SetEmbedded();

        JPH::RMat44 transFrom = JPH::RMat44::sIdentity();
        transFrom.SetTranslation(Misc::Convert::toJolt<JPH::RVec3>(from));

        JPH::ShapeCastSettings settings;
        settings.mUseShrunkenShapeAndConvexRadius = true;
        settings.mBackFaceModeTriangles = JPH::EBackFaceMode::IgnoreBackFaces;
        settings.mBackFaceModeConvex = JPH::EBackFaceMode::IgnoreBackFaces;

        JPH::Vec3 scale = JPH::Vec3::sReplicate(1.0f);
        JPH::RShapeCast shapeCast(&sphere, scale, transFrom, Misc::Convert::toJolt<JPH::Vec3>(to - from));

        const int queryObjectMask = getQueryObjectMask(mask, group);
        if (queryObjectMask == 0)
        {
            RayCastingResult result;
            result.mHit = false;
            return result;
        }

        const std::uint32_t queryBroadPhaseMask = getQueryBroadPhaseMask(queryObjectMask, mBPLayerInterface);
        if (queryBroadPhaseMask == 0)
        {
            RayCastingResult result;
            result.mHit = false;
            return result;
        }
        MaskedBroadPhaseLayerFilter broadphaseLayerFilter(queryBroadPhaseMask);
        MaskedObjectLayerFilter objectLayerFilter(queryObjectMask);

        ClosestConvexResultCallback callback(transFrom.GetTranslation());
        mPhysicsSystem.GetNarrowPhaseQuery().CastShape(
            shapeCast, settings, transFrom.GetTranslation(), callback, broadphaseLayerFilter, objectLayerFilter);

        RayCastingResult result;
        result.mHit = callback.hasHit();
        if (result.mHit)
        {
            result.mHitPos = callback.mHitPointWorld;
            result.mHitNormal = Misc::Convert::toOsg(callback.mHitNormalWorld);
            if (auto* ptrHolder = static_cast<PtrHolder*>(mTaskScheduler->getUserPointer(callback.mHitCollisionObject)))
                result.mHitObject = ptrHolder->getPtr();
        }
        return result;
    }

    bool PhysicsSystem::getLineOfSight(const MWWorld::ConstPtr& actor1, const MWWorld::ConstPtr& actor2) const
    {
        if (actor1 == actor2)
            return true;

        const auto it1 = mActors.find(actor1.mRef);
        const auto it2 = mActors.find(actor2.mRef);
        if (it1 == mActors.end() || it2 == mActors.end())
            return false;

        return mTaskScheduler->getLineOfSight(it1->second, it2->second);
    }

    bool PhysicsSystem::isOnGround(const MWWorld::Ptr& actor)
    {
        Actor* physactor = getActor(actor);
        return physactor && physactor->getOnGround() && physactor->getCollisionMode();
    }

    bool PhysicsSystem::canMoveToWaterSurface(const MWWorld::ConstPtr& actor, const float waterlevel)
    {
        const auto* physactor = getActor(actor);
        return physactor && physactor->canMoveToWaterSurface(waterlevel, &mPhysicsSystem);
    }

    osg::Vec3f PhysicsSystem::getHalfExtents(const MWWorld::ConstPtr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (physactor)
            return physactor->getHalfExtents();
        else
            return osg::Vec3f();
    }

    osg::Vec3f PhysicsSystem::getOriginalHalfExtents(const MWWorld::ConstPtr& actor) const
    {
        if (const Actor* physactor = getActor(actor))
            return physactor->getOriginalHalfExtents();
        else
            return osg::Vec3f();
    }

    osg::Vec3f PhysicsSystem::getRenderingHalfExtents(const MWWorld::ConstPtr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (physactor)
            return physactor->getRenderingHalfExtents();
        else
            return osg::Vec3f();
    }

    osg::BoundingBox PhysicsSystem::getBoundingBox(const MWWorld::ConstPtr& object) const
    {
        const Object* physobject = getObject(object);
        if (!physobject)
            return osg::BoundingBox();

        JPH::BodyID bodyId = physobject->getPhysicsBody();
        if (bodyId.IsInvalid())
            return osg::BoundingBox();

        JPH::BodyLockRead lock(mPhysicsSystem.GetBodyLockInterface(), bodyId);
        if (!lock.Succeeded())
            return osg::BoundingBox();

        const JPH::Body& body = lock.GetBody();

        // Get body world space translation + shape local space bound
        osg::Vec3f translation = Misc::Convert::toOsg(body.GetCenterOfMassTransform().GetTranslation());
        JPH::AABox bounds = body.GetShape()->GetLocalBounds();

        // Convert local space bounds to world space
        osg::Vec3f min = osg::Vec3f(bounds.mMin.GetX(), bounds.mMin.GetY(), bounds.mMin.GetZ()) + translation;
        osg::Vec3f max = osg::Vec3f(bounds.mMax.GetX(), bounds.mMax.GetY(), bounds.mMax.GetZ()) + translation;

        return osg::BoundingBox(min, max);
    }

    osg::Vec3f PhysicsSystem::getCollisionObjectPosition(const MWWorld::ConstPtr& actor) const
    {
        const Actor* physactor = getActor(actor);
        if (physactor)
            return physactor->getCollisionObjectPosition();
        else
            return osg::Vec3f();
    }

    std::vector<ContactPoint> PhysicsSystem::getCollisionsPoints(
        const MWWorld::ConstPtr& ptr, int collisionGroup, int collisionMask) const
    {
        JPH::BodyID me;

        auto found = mObjects.find(ptr.mRef);
        if (found != mObjects.end())
            me = found->second->getPhysicsBody();
        else
            return {};

        if (me.IsInvalid())
            return {};

        JPH::ShapeRefC shape;
        JPH::RMat44 transform;

        // Scoped lock so we can read then discard
        {
            JPH::BodyLockRead lock(mPhysicsSystem.GetBodyLockInterface(), me);
            if (!lock.Succeeded())
                return {};

            const JPH::Body& body = lock.GetBody();
            transform = body.GetCenterOfMassTransform();
            shape = body.GetShape();
        }

        // This sets layer filters to avoid collisions with static geometry in those cases, and allows with
        // actors->actors etc if needed this is important to prevent Jolt comlaining that two triangle mesh shapes
        // cannot collide
        const JPH::ObjectLayer collisionLayer = static_cast<JPH::ObjectLayer>(
            std::clamp(collisionGroup, 0, static_cast<int>(std::numeric_limits<JPH::ObjectLayer>::max())));
        JPH::DefaultBroadPhaseLayerFilter broadphaseLayerFilter
            = mPhysicsSystem.GetDefaultBroadPhaseLayerFilter(collisionLayer);
        MaskedObjectLayerFilter objectLayerFilter(collisionMask);

        JPH::CollideShapeSettings settings;
        settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideWithAll;
        settings.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;
        settings.mCollectFacesMode = JPH::ECollectFacesMode::NoFaces;

        // WARNING: you cannot collide mesh->mesh shapes in Jolt, so this should filter to avoid that (only collide with
        // actors etc)
        auto scale = JPH::Vec3::sReplicate(1.0f);
        ContactTestResultCallback resultCallback(&mPhysicsSystem, me);
        mPhysicsSystem.GetNarrowPhaseQuery().CollideShape(shape, scale, transform, settings, JPH::RVec3::sZero(),
            resultCallback, broadphaseLayerFilter, objectLayerFilter);
        return resultCallback.mResult;
    }

    std::vector<MWWorld::Ptr> PhysicsSystem::getCollisions(
        const MWWorld::ConstPtr& ptr, int collisionGroup, int collisionMask) const
    {
        std::vector<MWWorld::Ptr> actors;
        for (auto& [actor, point, normal] : getCollisionsPoints(ptr, collisionGroup, collisionMask))
            actors.emplace_back(actor);
        return actors;
    }

    osg::Vec3f PhysicsSystem::traceDown(const MWWorld::Ptr& ptr, const osg::Vec3f& position, float maxHeight)
    {
        ActorMap::iterator found = mActors.find(ptr.mRef);
        if (found == mActors.end())
            return ptr.getRefData().getPosition().asVec3();
        return MovementSolver::traceDown(ptr, position, found->second.get(), &mPhysicsSystem, maxHeight);
    }

    void PhysicsSystem::optimize()
    {
        mPhysicsSystem.OptimizeBroadPhase();
    }

    void PhysicsSystem::addHeightField(
        const float* heights, int x, int y, int size, int verts, float minH, float maxH, const osg::Object* holdObject)
    {
        mHeightFields[std::make_pair(x, y)]
            = std::make_unique<HeightField>(heights, x, y, size, verts, minH, maxH, holdObject, mTaskScheduler.get());
    }

    void PhysicsSystem::removeHeightField(int x, int y)
    {
        HeightFieldMap::iterator heightfield = mHeightFields.find(std::make_pair(x, y));
        if (heightfield != mHeightFields.end())
            mHeightFields.erase(heightfield);
    }

    const HeightField* PhysicsSystem::getHeightField(int x, int y) const
    {
        const auto heightField = mHeightFields.find(std::make_pair(x, y));
        if (heightField == mHeightFields.end())
            return nullptr;
        return heightField->second.get();
    }

    void PhysicsSystem::addObject(
        const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh, osg::Quat rotation, int collisionType)
    {
        if (ptr.mRef->mData.mPhysicsPostponed)
            return;

        VFS::Path::Normalized animationMesh(mesh);
        if (ptr.getClass().useAnim())
            animationMesh = Misc::ResourceHelpers::correctActorModelPath(mesh, mResourceSystem->getVFS());
        osg::ref_ptr<Resource::PhysicsShapeInstance> shapeInstance = mShapeManager->getInstance(animationMesh);
        if (!shapeInstance || !shapeInstance->mCollisionShape)
        {
            return;
        }

        assert(!getObject(ptr));

        // Override collision type based on shape content.
        switch (shapeInstance->mVisualCollisionType)
        {
            case Resource::VisualCollisionType::None:
                break;
            case Resource::VisualCollisionType::Default:
                collisionType = Layers::VISUAL_ONLY;
                break;
            case Resource::VisualCollisionType::Camera:
                collisionType = Layers::CAMERA_ONLY;
                break;
        }

        auto obj = std::make_shared<Object>(ptr, shapeInstance, rotation, collisionType, mTaskScheduler.get());
        mObjects.emplace(ptr.mRef, obj);

        if (obj->isAnimated())
            mAnimatedObjects.emplace(obj.get(), false);
    }

    void PhysicsSystem::remove(const MWWorld::Ptr& ptr)
    {
        mPendingRagdollImpacts.erase(ptr.mRef);
        mPreparedRagdollWeapons.erase(ptr.mRef);
        mPassiveRagdollMotionHistory.erase(ptr.mRef);
        // Movement jobs keep only raw user-data pointers for objects hit by a
        // sweep. Keep PtrHolder instances alive until every shared movement
        // visitor has finished using those copied pointers.
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        if (mPassiveRagdolls.contains(ptr.mRef))
            erasePassiveRagdoll(ptr.mRef);
        if (auto foundObject = mObjects.find(ptr.mRef); foundObject != mObjects.end())
        {
            mAnimatedObjects.erase(foundObject->second.get());

            mObjects.erase(foundObject);
        }
        else if (auto foundActor = mActors.find(ptr.mRef); foundActor != mActors.end())
        {
            mActors.erase(foundActor);
        }
    }

    void PhysicsSystem::removeProjectile(const int projectileId)
    {
        ProjectileMap::iterator foundProjectile = mProjectiles.find(projectileId);
        if (foundProjectile != mProjectiles.end())
            mProjectiles.erase(foundProjectile);
    }

    void PhysicsSystem::updatePtr(const MWWorld::Ptr& old, const MWWorld::Ptr& updated)
    {
        if (auto foundObject = mObjects.find(old.mRef); foundObject != mObjects.end())
            foundObject->second->updatePtr(updated);
        else if (auto foundActor = mActors.find(old.mRef); foundActor != mActors.end())
            foundActor->second->updatePtr(updated);

        if (auto foundRagdoll = mPassiveRagdolls.find(old.mRef); foundRagdoll != mPassiveRagdolls.end())
        {
            foundRagdoll->second->updatePtr(updated);
            foundRagdoll->second->invalidateBalanceSnapshot("actor-pointer-update");
        }

        for (auto& [_, actor] : mActors)
        {
            if (actor->getStandingOnPtr() == old)
                actor->setStandingOnPtr(updated);
        }

        for (auto& [_, projectile] : mProjectiles)
        {
            if (projectile->getCaster() == old)
                projectile->setCaster(updated);
        }
    }

    Actor* PhysicsSystem::getActor(const MWWorld::Ptr& ptr)
    {
        ActorMap::iterator found = mActors.find(ptr.mRef);
        if (found != mActors.end())
            return found->second.get();
        return nullptr;
    }

    const Actor* PhysicsSystem::getActor(const MWWorld::ConstPtr& ptr) const
    {
        ActorMap::const_iterator found = mActors.find(ptr.mRef);
        if (found != mActors.end())
            return found->second.get();
        return nullptr;
    }

    const Object* PhysicsSystem::getObject(const MWWorld::ConstPtr& ptr) const
    {
        ObjectMap::const_iterator found = mObjects.find(ptr.mRef);
        if (found != mObjects.end())
            return found->second.get();
        return nullptr;
    }

    Projectile* PhysicsSystem::getProjectile(int projectileId) const
    {
        ProjectileMap::const_iterator found = mProjectiles.find(projectileId);
        if (found != mProjectiles.end())
            return found->second.get();
        return nullptr;
    }

    void PhysicsSystem::updateScale(const MWWorld::Ptr& ptr)
    {
        if (const auto ragdoll = mPassiveRagdolls.find(ptr.mRef); ragdoll != mPassiveRagdolls.end())
            ragdoll->second->invalidateBalanceSnapshot("actor-scale-update");
        if (auto foundObject = mObjects.find(ptr.mRef); foundObject != mObjects.end())
        {
            float scale = ptr.getCellRef().getScale();
            foundObject->second->setScale(scale);
        }
        else if (auto foundActor = mActors.find(ptr.mRef); foundActor != mActors.end())
        {
            const float referenceScale = ptr.getCellRef().getScale();
            osg::Vec3f adjustedScale(referenceScale, referenceScale, referenceScale);
            ptr.getClass().adjustScale(ptr, adjustedScale, false);
            const bool physicalScaleChanged = adjustedScale != foundActor->second->getScale();
            {
                std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
                if (const auto entry = mPassiveRagdolls.find(ptr.mRef); entry != mPassiveRagdolls.end()
                    && entry->second->mSupportGripEnabled && physicalScaleChanged)
                {
                    entry->second->clearNativeSupportGrip("scale-changed");
                    // The persistent ragdoll is not resized by Actor::updateScale.
                    // Wait for its normal rebuild before accepting COM anchors.
                    // Camera rebuilds reapply the same scale; they must not
                    // invalidate an unchanged physical rig.
                    entry->second->mSupportGripScaleInvalidated = true;
                }
            }
            foundActor->second->updateScale();
        }
    }

    void PhysicsSystem::updateRotation(const MWWorld::Ptr& ptr, osg::Quat rotate)
    {
        if (auto foundObject = mObjects.find(ptr.mRef); foundObject != mObjects.end())
        {
            foundObject->second->setRotation(rotate);
        }
        else if (auto foundActor = mActors.find(ptr.mRef); foundActor != mActors.end())
        {
            if (!foundActor->second->isRotationallyInvariant())
            {
                foundActor->second->setRotation(rotate);
            }
        }
    }

    void PhysicsSystem::updatePosition(const MWWorld::Ptr& ptr)
    {
        if (const auto ragdoll = mPassiveRagdolls.find(ptr.mRef); ragdoll != mPassiveRagdolls.end())
        {
            ragdoll->second->invalidateBalanceSnapshot("actor-position-update");
            // A teleport (door, coc, script) moved the actor but not its
            // powered body, which stayed at the old place and dragged the
            // actor back: through a door into an interior the player landed
            // at the exterior's XY at the interior's height and fell into the
            // void every frame (play 2026-10-01, "can't go into interior
            // cells"). Far from its body, the actor drops the body now and is
            // powered again half a second after it stands at the new place
            // (the swim fallback's resume).
            auto& entry = *ragdoll->second;
            if (entry.mPowered && entry.mRagdoll && entry.mRagdoll->getBodyCount() > 0)
            {
                const osg::Vec3f body = Misc::Convert::toOsg(
                    mPhysicsSystem.GetBodyInterface().GetCenterOfMassPosition(entry.mRagdoll->getBodyId(0)));
                const osg::Vec3f actor = ptr.getRefData().getPosition().asVec3();
                const float distance = osg::Vec2f(body.x() - actor.x(), body.y() - actor.y()).length();
                if (distance > 500.0f || std::abs(body.z() - actor.z()) > 300.0f)
                {
                    Log(Debug::Info) << "RAGDOLL_TELEPORT actor=" << ptr.getCellRef().getRefId()
                                     << " distance=" << distance << " repower=1";
                    removePassiveRagdoll(ptr);
                    mSwimSuspended[ptr.mRef] = -1.0;
                    if (const auto found = mActors.find(ptr.mRef); found != mActors.end())
                        found->second->enableCollisionBody(true);
                }
            }
        }
        if (auto foundObject = mObjects.find(ptr.mRef); foundObject != mObjects.end())
        {
            foundObject->second->updatePosition();
        }
        else if (auto foundActor = mActors.find(ptr.mRef); foundActor != mActors.end())
        {
            foundActor->second->updatePosition();
        }
    }

    void PhysicsSystem::addActor(const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh)
    {
        VFS::Path::Normalized animationMesh
            = Misc::ResourceHelpers::correctActorModelPath(mesh, mResourceSystem->getVFS());
        osg::ref_ptr<const Resource::PhysicsShape> shape = mShapeManager->getShape(animationMesh);

        // Try to get shape from basic model as fallback for creatures
        if (!ptr.getClass().isNpc() && shape && shape->mCollisionBox.mExtents.length2() == 0)
        {
            if (animationMesh != mesh)
            {
                shape = mShapeManager->getShape(mesh);
            }
        }

        if (!shape)
            return;

        // check if Actor should spawn above water
        const MWMechanics::MagicEffects& effects = ptr.getClass().getCreatureStats(ptr).getMagicEffects();
        const bool canWaterWalk = effects.getOrDefault(ESM::MagicEffect::WaterWalking).getMagnitude() > 0;

        auto actor = std::make_shared<Actor>(
            ptr, shape, mTaskScheduler.get(), canWaterWalk, Settings::game().mActorCollisionShapeType);

        mActors.emplace(ptr.mRef, std::move(actor));
    }

    const RagdollCore::RagdollProfile* PhysicsSystem::getPassiveRagdollProfile()
    {
        if (mPassiveRagdollProfileLoadAttempted)
            return mPassiveRagdollProfile.get();

        mPassiveRagdollProfileLoadAttempted = true;
        const VFS::Manager* vfs = mResourceSystem->getVFS();
        if (vfs == nullptr)
        {
            Log(Debug::Warning) << "Passive ragdoll profile load failed: VFS is unavailable";
            return nullptr;
        }

        RagdollProfile::LoadResult loaded
            = RagdollProfile::load(*vfs, VFS::Path::NormalizedView("ragdoll/humanoid-v1.yaml"));
        if (!loaded.isValid())
        {
            for (const RagdollProfile::LoadIssue& issue : loaded.mIssues)
                Log(Debug::Warning) << "Passive ragdoll profile load failed at " << issue.mPath << ": "
                                    << issue.mMessage;
            return nullptr;
        }

        mPassiveRagdollProfile = std::make_unique<RagdollCore::RagdollProfile>(std::move(*loaded.mProfile));
        return mPassiveRagdollProfile.get();
    }

    void PhysicsSystem::samplePassiveRagdollMotion(
        const MWWorld::Ptr& ptr, SceneUtil::Skeleton& skeleton, float duration)
    {
        // How long the weapon has been drawn, with or without a body (the
        // ready-stance rebuild below).
        // A weapon swapped while drawn starts again (the play hub swaps the
        // dagger for the longsword without lowering the stance).
        if (ptr.getClass().isActor())
        {
            std::string held;
            if (ptr.getClass().hasInventoryStore(ptr))
            {
                const auto& inventory = ptr.getClass().getInventoryStore(ptr);
                const auto right = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                if (right != inventory.end())
                    held = right->getCellRef().getRefId().serializeText();
            }
            if (ptr.getClass().getCreatureStats(ptr).getDrawState() == MWMechanics::DrawState::Weapon)
            {
                auto& drawn = mWeaponDrawnFor[ptr.mRef];
                if (drawn.first != held)
                    drawn = { held, 0.0f };
                drawn.second += duration;
            }
            else
                mWeaponDrawnFor.erase(ptr.mRef);
        }
        if (const auto ragdoll = mPassiveRagdolls.find(ptr.mRef); ragdoll != mPassiveRagdolls.end())
        {
            // The development rig includes the grip in its creation profile.
            // Reconcile equipment changes at a quiet standing boundary, rather
            // than silently keeping the old fist/weapon contact surface. Never
            // rebuild a falling/recovering body or an attack in progress.
            // NPCs too: with OPENMW_RAGDOLL_POWER_NPCS an NPC is powered as
            // soon as it is near, often before its weapon is equipped, and
            // then fought bare-handed with a claymore in its hand (2026-09-24
            // play test Q3: every NPC strike was a fist, `bare=1`).
            // The weapon arm is built from the ready stance (2026-10-05). The
            // body is built from the animation pose of the moment, and the
            // wrist's range is centred on it and the arm's springs are sized
            // from it: rebuilt at an equipment change mid-draw or sheathed,
            // the user's longsword wrist got a load of 19-26 against the 7.2
            // of every test (built from the ready stance), and in play the
            // guard stuck with the wrist backwards and wind-ups spun to the
            // wrong side (builds 47-48; REPLAY-08 reproduces it). The player's
            // rig under hand control built sheathed or mid-draw is rebuilt
            // once the weapon has been drawn READY seconds. An equipment
            // change still rebuilds at once (deferring it left FIX-08's
            // first-person hits and a warhammer test without the weapon's rig
            // for 0.8 s: sentinel combat 7 vs 4, walk 9 vs 4 over 4 / 3 runs).
            // OPENMW_RAGDOLL_READY_REBUILD = READY (0.8 s; 0 = off).
            static const auto readyAfter_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_READY_REBUILD");
                return text ? static_cast<float>(std::atof(text)) : 0.8f;
            });
            static const auto& readyAfter = readyAfter_tuned.get();
            if (ragdoll->second->mPowered && ptr.getClass().hasInventoryStore(ptr))
            {
                auto& entry = *ragdoll->second;
                const float drawnFor = weaponDrawnFor(ptr);
                const bool readyNow = readyAfter > 0.0f && drawnFor >= readyAfter;
                const bool handControl = PassiveRagdollEntry::halfSwordMode() > 0
                    && (ptr == MWMechanics::getPlayer()
                        || (PassiveRagdollEntry::halfSwordNpcs() && PassiveRagdollEntry::kineticHumanoid(ptr)));
                const auto& inventory = ptr.getClass().getInventoryStore(ptr);
                const auto equipped = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                const std::string equippedId = equipped != inventory.end()
                        && equipped->getType() == ESM::Weapon::sRecordId
                    ? equipped->getCellRef().getRefId().serializeText() : std::string{};
                const std::string shieldId = equippedShieldId(ptr);
                const bool changed = equippedId != entry.mEquippedWeaponAtCreation
                    || shieldId != entry.mEquippedShieldAtCreation;
                const bool unready = handControl && readyNow && !entry.mBuiltReady && !equippedId.empty();
                if ((changed || unready)
                    && !entry.mSwing.mSwinging && !entry.mAiStrikeActive
                    && !entry.mSwingInput.mHeld && !entry.mHitStun.active() && entry.mControllerOutput
                    && entry.mControllerOutput->mMode == RagdollCore::ActiveControllerMode::Standing
                    && entry.mRagdoll->isRootKinematic())
                {
                    const auto damage = entry.mRegionDamage;
                    {
                        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
                        erasePassiveRagdoll(ptr.mRef, PassiveRagdollWeaponReleaseReason::Unequip);
                    }
                    prepareEquippedRagdollWeapon(ptr);
                    if (addRagdoll(ptr, skeleton, true))
                    {
                        auto& replacement = *mPassiveRagdolls.at(ptr.mRef);
                        // Accept the rig as matching even when this weapon could
                        // not get a body (no weight, no mesh): otherwise it was
                        // erased and rebuilt every standing frame.
                        replacement.mEquippedWeaponAtCreation = equippedId;
                        replacement.mEquippedShieldAtCreation = shieldId;
                        replacement.mBuiltReady = readyNow || equippedId.empty() || readyAfter <= 0.0f || !handControl;
                        if (unready)
                            Log(Debug::Info) << "RAGDOLL_READY_REBUILD actor=" << ptr.getCellRef().getRefId()
                                             << " weapon=" << equippedId << " drawn_for=" << drawnFor;
                        for (std::size_t region = 0; region < damage.size(); ++region)
                            replacement.addRegionDamage(static_cast<RagdollCore::AnatomicalRegion>(region), damage[region]);
                    }
                    else if (const auto actor = mActors.find(ptr.mRef); actor != mActors.end())
                        actor->second->enableCollisionBody(true);
                    return;
                }
            }
            if (ragdoll->second->mWeapon && ragdoll->second->mWeapon->mAttached)
            {
                MWWorld::InventoryStore& inventory = ptr.getClass().getInventoryStore(ptr);
                const auto equipped = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                const std::string equippedId = equipped != inventory.end()
                    ? equipped->getCellRef().getRefId().serializeText()
                    : std::string{};
                if (equippedId != ragdoll->second->mWeapon->mRecordId)
                {
                    const ESM::RefId weaponId
                        = ESM::RefId::stringRefId(ragdoll->second->mWeapon->mRecordId);
                    const PassiveRagdollWeaponReleaseReason reason = inventory.search(weaponId).isEmpty()
                        ? PassiveRagdollWeaponReleaseReason::Drop
                        : PassiveRagdollWeaponReleaseReason::Unequip;
                    std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
                    releaseWeapon(*ragdoll->second, reason);
                }
            }
            if (ragdoll->second->mPowered && !ragdoll->second->capturePoweredTarget(mJoltStepSerial))
            {
                // An animation fallback can replace the live skeleton while
                // the persistent physics body remains valid. Bind the new
                // skeleton before treating the missing animation target as a
                // controller failure. If traversal has not completed yet,
                // retain the last validated target and retry next frame.
                const RagdollCore::RagdollProfile* profile = getPassiveRagdollProfile();
                RagdollProfile::SkeletonSnapshotResult snapshot;
                if (profile != nullptr)
                    snapshot = RagdollProfile::snapshotSkeleton(*profile, skeleton);
                if (snapshot.isValid() && snapshot.mSkeleton->mBones.size() == ragdoll->second->mPoseBindingBodyCount)
                {
                    ragdoll->second->mSupportGripResetPending = true;
                    ragdoll->second->mPoseBinding = std::move(snapshot.mPoseBinding);
                    ragdoll->second->mPoweredTargetCaptureFailureCount = 0;
                    if (ragdoll->second->capturePoweredTarget(mJoltStepSerial))
                        return;
                }
                if (!ragdoll->second->mMotorTargetCaptured || !ragdoll->second->mCurrentPhysicsPoseCaptured)
                    ragdoll->second->failPoweredControl(__LINE__);
            }
            return;
        }
        const auto foundActor = mActors.find(ptr.mRef);
        const RagdollCore::RagdollProfile* profile = getPassiveRagdollProfile();
        if (foundActor == mActors.end() || profile == nullptr)
            return;

        auto found = mPassiveRagdollMotionHistory.find(ptr.mRef);
        if (found != mPassiveRagdollMotionHistory.end() && found->second->mSkeleton != &skeleton)
        {
            mPassiveRagdollMotionHistory.erase(found);
            found = mPassiveRagdollMotionHistory.end();
        }

        if (found == mPassiveRagdollMotionHistory.end())
        {
            auto history = std::make_unique<PassiveRagdollMotionHistory>();
            history->mSkeleton = &skeleton;
            RagdollProfile::SkeletonSnapshotResult snapshot = RagdollProfile::snapshotSkeleton(*profile, skeleton);
            if (snapshot.isValid())
            {
                std::vector<RagdollCore::Transform> bodyTransforms;
                bodyTransforms.reserve(snapshot.mSkeleton->mBones.size());
                for (const RagdollCore::SkeletonBone& bone : snapshot.mSkeleton->mBones)
                    bodyTransforms.push_back(bone.mModelTransform);
                if (auto pose = makeMotionPose(ptr, *foundActor->second, std::move(bodyTransforms)))
                {
                    history->mPreviousPose = std::move(*pose);
                    history->mPoseBinding = std::move(snapshot.mPoseBinding);
                    ++mPassiveRagdollMotionSamples;
                }
            }
            mPassiveRagdollMotionHistory.emplace(ptr.mRef, std::move(history));
            return;
        }

        PassiveRagdollMotionHistory& history = *found->second;
        if (!history.mPoseBinding)
            return;
        std::vector<RagdollCore::Transform> bodyTransforms;
        if (!history.mPoseBinding->capturePose(bodyTransforms))
        {
            mPassiveRagdollMotionHistory.erase(found);
            return;
        }
        std::optional<RagdollCore::MotionPose> current
            = makeMotionPose(ptr, *foundActor->second, std::move(bodyTransforms));
        if (!current)
        {
            history.mLatestMotion.clear();
            return;
        }
        if (auto motion
            = RagdollCore::estimateBodyMotions(history.mPreviousPose, *current, duration, profile->mMotionTransfer))
            history.mLatestMotion = std::move(*motion);
        else
            history.mLatestMotion.clear();
        history.mPreviousPose = std::move(*current);
        ++mPassiveRagdollMotionSamples;
    }

    std::vector<MWWorld::Ptr> PhysicsSystem::takeFinishedDeaths()
    {
        std::vector<MWWorld::Ptr> done;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        for (auto& [ref, entry] : mPassiveRagdolls)
            if (entry->mPowered && entry->mDying && entry->mDyingDone)
            {
                entry->mDying = false;
                done.push_back(entry->getPtr());
            }
        return done;
    }

    bool PhysicsSystem::addPassiveRagdoll(const MWWorld::Ptr& ptr, SceneUtil::Skeleton& skeleton, bool force)
    {
        // A physical death (OPENMW_RAGDOLL_DEATH_PHYSICAL): the powered body
        // dies on its feet first; takeFinishedDeaths hands it back here.
        if (!force)
            if (const auto dying = mPassiveRagdolls.find(ptr.mRef);
                dying != mPassiveRagdolls.end() && dying->second->mPowered)
            {
                std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
                if (dying->second->mDying)
                    return true;
                osg::Vec3f direction;
                if (const auto impact = mPendingRagdollImpacts.find(ptr.mRef); impact != mPendingRagdollImpacts.end())
                    direction = impact->second.mDirection;
                if (dying->second->beginDying(direction))
                    return true;
            }
        // Death of a powered body (user 2026-09-30 bookmark: "on death characters
        // dont maintain their current ragdoll position they get restarted as
        // standing then ragdoll"): the limp ragdoll was built from the
        // skeleton's animation pose. The powered bodies' world state (by body
        // name) is carried over, so the body goes limp where it is.
        // OPENMW_RAGDOLL_DEATH_CARRY=0 = the old rebuild.
        struct BodyState
        {
            JPH::RVec3 mPosition;
            JPH::Quat mRotation;
            JPH::Vec3 mLinear, mAngular;
        };
        std::map<std::string, BodyState> carried;
        static const auto carry_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DEATH_CARRY");
            return !(text && std::atoi(text) == 0);
        });
        static const auto& carry = carry_tuned.get();
        if (const auto existing = mPassiveRagdolls.find(ptr.mRef); existing != mPassiveRagdolls.end())
        {
            if (!existing->second->mPowered)
                return true;
            std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
            const PassiveRagdollEntry& old = *existing->second;
            const RagdollCore::RagdollProfile* profile
                = old.mRuntimeProfile ? old.mRuntimeProfile.get() : mPassiveRagdollProfile.get();
            if (carry && old.mRagdoll && profile)
            {
                JPH::BodyInterface& bodies = mPhysicsSystem.GetBodyInterface();
                for (std::size_t i = 0; i < old.mRagdoll->getBodyCount() && i < profile->mBodies.size(); ++i)
                {
                    const JPH::BodyID id = old.mRagdoll->getBodyId(i);
                    BodyState state;
                    bodies.GetPositionAndRotation(id, state.mPosition, state.mRotation);
                    state.mLinear = bodies.GetLinearVelocity(id);
                    state.mAngular = bodies.GetAngularVelocity(id);
                    carried[profile->mBodies[i].mName] = state;
                }
            }
            erasePassiveRagdoll(ptr.mRef, PassiveRagdollWeaponReleaseReason::Death);
        }
        const bool added = addRagdoll(ptr, skeleton, false);
        if (added && !carried.empty())
        {
            std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
            const auto created = mPassiveRagdolls.find(ptr.mRef);
            if (created != mPassiveRagdolls.end() && created->second->mRagdoll)
            {
                const PassiveRagdollEntry& fresh = *created->second;
                const RagdollCore::RagdollProfile* profile
                    = fresh.mRuntimeProfile ? fresh.mRuntimeProfile.get() : mPassiveRagdollProfile.get();
                JPH::BodyInterface& bodies = mPhysicsSystem.GetBodyInterface();
                std::size_t placed = 0;
                float snap = 0.0f; // how far the rebuilt pose was from the living body
                // The dying shuffle (user bookmark 20, 2026-09-30: "they don't
                // just die, they have some sort of momentum"): the living
                // bodies' velocities carried the root's drag and the killing
                // blow's kick. The body keeps its common velocity and a share of
                // each part's own, capped. OPENMW_RAGDOLL_DEATH_CARRY_VELOCITY =
                // "share,cap" (1,0 = all, uncapped: the old carry).
                static const auto carryVelocity_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
                    // 0.3, 50 since 2026-10-01: SOAK-02 (10 min of duels, 38-52
                    // deaths each) median/max slide in the first 1.5 s dead:
                    // 1,0 (old) 93/240 u, 0.3,100 72/113, 0.3,50 51/88,
                    // 0,1 (no carry_tuned.get()) 17/41.
                    // 0.15, 15 since 10-30 (user bookmark: "upon dying the npc
                    // corpse will travel/be drag the entire ragdoll around a
                    // bit before stopping"): 29 of 30 deaths in the 10-30 play
                    // carried the full 50 u/s cap (the killing blow's stagger)
                    // and slid 12-126 units, median 50.
                    std::array<float, 2> value{ 0.15f, 15.0f };
                    if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DEATH_CARRY_VELOCITY"))
                        std::sscanf(text, "%f,%f", &value[0], &value[1]);
                    return value;
                });
                static const auto& carryVelocity = carryVelocity_tuned.get();
                JPH::Vec3 common = JPH::Vec3::sZero();
                for (const auto& [name, state] : carried)
                    common += state.mLinear;
                if (!carried.empty())
                    common /= static_cast<float>(carried.size());
                if (carryVelocity[1] > 0.0f && common.Length() > carryVelocity[1])
                    common = common.Normalized() * carryVelocity[1];
                for (std::size_t i = 0; profile && i < fresh.mRagdoll->getBodyCount() && i < profile->mBodies.size(); ++i)
                {
                    const auto found = carried.find(profile->mBodies[i].mName);
                    if (found == carried.end())
                        continue;
                    const JPH::BodyID id = fresh.mRagdoll->getBodyId(i);
                    snap = std::max(snap, static_cast<float>((bodies.GetPosition(id) - found->second.mPosition).Length()));
                    const JPH::Vec3 linear = common + (found->second.mLinear - common) * carryVelocity[0];
                    bodies.SetPositionRotationAndVelocity(id, found->second.mPosition, found->second.mRotation,
                        linear, found->second.mAngular * carryVelocity[0]);
                    ++placed;
                }
                Log(Debug::Info) << "RAGDOLL_DEATH_CARRY actor=" << ptr.getCellRef().getRefId() << " bodies=" << placed
                                 << " snap_avoided=" << snap << " common_speed=" << common.Length();
                created->second->mDeathWatchStart = created->second->mRagdoll->getBodyCount() > 0
                    ? std::optional(bodies.GetCenterOfMassPosition(fresh.mRagdoll->getBodyId(0)))
                    : std::nullopt;
                created->second->mDeathWatchStep = mJoltStepSerial;
                created->second->mDeathWatchActorStart = ptr.getRefData().getPosition().asVec3();
                // The corpse's slide (10-30 play: the pelvis travelled a median
                // 32 units, 86 at p90, in the first 1.5 s dead, with only 15 u/s
                // carried): the body's common horizontal velocity is drained
                // at `rate` per second for `seconds`; the limbs keep their own
                // motion, so it still crumples and falls as before.
                // OPENMW_RAGDOLL_DEATH_DRAG = "rate, seconds" (0 = off).
                static const auto deathDrag_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
                    std::array<float, 2> value{ 3.0f, 2.5f };
                    if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DEATH_DRAG"))
                        std::sscanf(text, "%f,%f", &value[0], &value[1]);
                    return value;
                });
                const auto& deathDrag = deathDrag_tuned.get();
                created->second->mDeathDragSteps = deathDrag[0] > 0.0f && mPhysicsDt > 0.0f
                    ? static_cast<int>(deathDrag[1] / mPhysicsDt) : 0;
            }
        }
        return added;
    }

    bool PhysicsSystem::addPoweredRagdoll(const MWWorld::Ptr& ptr, SceneUtil::Skeleton& skeleton)
    {
        if (const auto existing = mPassiveRagdolls.find(ptr.mRef); existing != mPassiveRagdolls.end())
            return existing->second->mPowered;
        // An armed actor's ragdoll needs its weapon body from the start: it is
        // the physical strike surface, and without it a powered AI has nothing
        // to land blows with once the animation hit path is off.
        prepareEquippedRagdollWeapon(ptr);
        return addRagdoll(ptr, skeleton, true);
    }

    bool PhysicsSystem::prepareEquippedRagdollWeapon(const MWWorld::Ptr& ptr)
    {
        if (ptr.isEmpty() || mPassiveRagdolls.contains(ptr.mRef))
            return false;
        MWWorld::InventoryStore& inventory = ptr.getClass().getInventoryStore(ptr);
        const auto weapon = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (weapon == inventory.end() || weapon->getType() != ESM::Weapon::sRecordId)
            return false;
        const float itemWeight = weapon->getClass().getWeight(*weapon);
        if (!std::isfinite(itemWeight) || itemWeight <= 0)
            return false;
        const auto& data = weapon->get<ESM::Weapon>()->mBase->mData;
        const float speed = data.mSpeed;
        const float reach = data.mReach;
        if (!std::isfinite(speed) || speed <= 0.0f)
            return false;
        mPreparedRagdollWeapons.insert_or_assign(ptr.mRef,
            PreparedRagdollWeapon{ weapon->getCellRef().getRefId().serializeText(), itemWeight, speed,
                std::isfinite(reach) && reach > 0.0f ? reach : 0.8f });
        return true;
    }

    bool PhysicsSystem::releasePassiveRagdollWeapon(
        const MWWorld::ConstPtr& ptr, PassiveRagdollWeaponReleaseReason reason)
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mWeapon)
            return false;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        return releaseWeapon(*found->second, reason);
    }

    bool PhysicsSystem::addRagdoll(const MWWorld::Ptr& ptr, SceneUtil::Skeleton& skeleton, bool powered)
    {
        if (!powered && Settings::game().mPassiveRagdollCap <= 0)
            return false;

        const auto foundActor = mActors.find(ptr.mRef);
        if (foundActor == mActors.end())
            return false;

        const RagdollCore::RagdollProfile* profile = getPassiveRagdollProfile();
        if (profile == nullptr)
            return false;

        RagdollProfile::SkeletonSnapshotResult snapshot = RagdollProfile::snapshotSkeleton(*profile, skeleton);
        if (!snapshot.isValid())
        {
            logRagdollIssues(ptr, "skeleton snapshot", snapshot.mIssues);
            return false;
        }

        const osg::Vec3f scale = foundActor->second->getScale();
        if (!std::isfinite(scale.x()) || !std::isfinite(scale.y()) || !std::isfinite(scale.z()) || scale.x() <= 0.0f
            || scale.y() <= 0.0f || scale.z() <= 0.0f || std::abs(scale.x() - scale.y()) > 1e-3f
            || std::abs(scale.x() - scale.z()) > 1e-3f)
        {
            Log(Debug::Warning) << "Passive ragdoll creation failed for " << ptr.getCellRef().getRefId()
                                << ": actor scale must be finite, positive, and uniform";
            return false;
        }

        RagdollCore::BuildOptions buildOptions;
        buildOptions.mTotalMass = 75.0f;
        buildOptions.mUniformScale = scale.x();
        buildOptions.mRootPosition = Misc::Convert::toJolt<JPH::RVec3>(ptr.getRefData().getPosition().asVec3());
        buildOptions.mRootRotation = Misc::Convert::toJolt(
            powered ? poweredActorRotation(ptr) : foundActor->second->getRotation());
        // A powered body is a fighter and has to be able to reach another
        // one; a corpse is debris and should keep touching nothing but the
        // floor. Choosing here rather than switching later means a body never
        // changes what it can hit midway through its own simulation.
        buildOptions.mObjectLayer = powered ? Layers::RAGDOLL : Layers::DEBRIS;

        RagdollCore::SkeletonDescriptor creationSkeleton = *snapshot.mSkeleton;
        const RagdollCore::RagdollProfile* creationProfile = profile;
        std::unique_ptr<RagdollCore::RagdollProfile> runtimeProfile;
        std::optional<LiveWeaponAttachment> weaponAttachment;
        std::optional<LiveShieldAttachment> shieldAttachment;
        std::string frameKey;
        if (powered)
        {
            runtimeProfile = std::make_unique<RagdollCore::RagdollProfile>(*profile);
            const auto findProfileBody = [&](const std::string& name) -> RagdollCore::BodyProfile* {
                for (auto& body : runtimeProfile->mBodies)
                    if (body.mName == name) return &body;
                return nullptr;
            };
            const auto findBone = [&](const std::string& name) -> const RagdollCore::Transform* {
                for (const auto& bone : creationSkeleton.mBones)
                    if (bone.mName == name) return &bone.mModelTransform;
                return nullptr;
            };
            for (const auto& handName : { profile->mLandmarks.mLeftHand, profile->mLandmarks.mRightHand })
            {
                auto* hand = handName ? findProfileBody(*handName) : nullptr;
                auto* lower = hand && hand->mParentBody ? findProfileBody(*hand->mParentBody) : nullptr;
                auto* upper = lower && lower->mParentBody ? findProfileBody(*lower->mParentBody) : nullptr;
                if (!upper || !lower->mConstraint || !lower->mConstraint->mDeriveFrameFromSkeleton
                    || lower->mConstraint->mType != RagdollCore::ConstraintType::Hinge)
                    continue;
                const auto* a = findBone(upper->mBone);
                const auto* b = findBone(lower->mBone);
                const auto* c = findBone(hand->mBone);
                if (!a || !b || !c || !RagdollCore::setAnatomicalHingeFrame(*lower->mConstraint,
                        *a, *b, *c, upper->mCenterOfMass, lower->mCenterOfMass))
                {
                    Log(Debug::Warning) << "Powered ragdoll creation failed: invalid elbow frame for "
                                        << ptr.getCellRef().getRefId();
                    return false;
                }
            }
            bool anatomicalKnees = true;
#ifdef OPENMW_RAGDOLL_DEV_API
            if (const char* option = RagdollCore::tuningText("OPENMW_RAGDOLL_ANATOMICAL_KNEES"))
                anatomicalKnees = std::string_view(option) != "0";
#endif
            if (profile->mActiveController.mLocomotionRoute == RagdollCore::LocomotionRoute::KinematicRoot
                && anatomicalKnees)
            {
                std::array<std::array<RagdollCore::BodyProfile*, 3>, 2> legs;
                std::size_t side = 0;
                for (const auto& footName : { profile->mLandmarks.mLeftFoot, profile->mLandmarks.mRightFoot })
                {
                    auto* foot = footName ? findProfileBody(*footName) : nullptr;
                    auto* lower = foot && foot->mParentBody ? findProfileBody(*foot->mParentBody) : nullptr;
                    auto* upper = lower && lower->mParentBody ? findProfileBody(*lower->mParentBody) : nullptr;
                    if (!upper || !lower->mConstraint)
                        return false;
                    legs[side++] = { upper, lower, foot };
                }
                const auto* leftHip = findBone(legs[0][0]->mBone);
                const auto* rightHip = findBone(legs[1][0]->mBone);
                if (!leftHip || !rightHip)
                    return false;
                const JPH::Vec3 lateral = rightHip->mTranslation - leftHip->mTranslation;
                for (const auto& leg : legs)
                {
                    auto* upper = leg[0];
                    auto* lower = leg[1];
                    auto* foot = leg[2];
                    // Authored frames retain their owner. Correct only the
                    // generic frame that otherwise calls this bent pose zero.
                    if (!lower->mConstraint->mDeriveFrameFromSkeleton)
                        continue;
                    const auto* a = findBone(upper->mBone);
                    const auto* b = findBone(lower->mBone);
                    const auto* c = findBone(foot->mBone);
                    if (!a || !b || !c || !RagdollCore::setAnatomicalHingeFrame(*lower->mConstraint,
                            *a, *b, *c, upper->mCenterOfMass, lower->mCenterOfMass, -1.0f, lateral))
                    {
                        Log(Debug::Warning) << "Powered ragdoll creation failed: invalid anatomical knee frame for "
                                            << ptr.getCellRef().getRefId();
                        return false;
                    }
                }
            }
            creationProfile = runtimeProfile.get();
            // Joint frames from a standing pose (user 11-01 bookmarks 5-11,
            // "my right foot is always forward" until the next cell change:
            // the body rebuilt at a door while running had its hips, ankles
            // and spine take that stride as their zero, so the limit cones
            // kept the right foot 27 units ahead of where the legs asked and
            // the torso tilted, and the tilt hold stiffened the root for good).
            // The legs' and spine's skeleton-derived frames come from a
            // standing pose of the same skeleton: the first powered build of
            // it standing still (feet under the pelvis, torso upright) is kept
            // and used by every later build. Arms keep the build pose (the
            // weapon arm was tuned on it). No standing pose seen yet: the
            // build pose, as before. OPENMW_RAGDOLL_BUILD_FRAMES=0 = off.
            static const auto buildFrames_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_BUILD_FRAMES");
                return !(text && std::atoi(text) == 0);
            });
            if (buildFrames_tuned.get())
            {
                const auto boneOfBody = [&](const std::string& bodyName) -> std::string {
                    for (const auto& body : runtimeProfile->mBodies)
                        if (body.mName == bodyName)
                            return body.mBone;
                    return {};
                };
                const auto parentOfBody = [&](const std::string& bodyName) -> std::string {
                    for (const auto& body : runtimeProfile->mBodies)
                        if (body.mName == bodyName && body.mParentBody)
                            return *body.mParentBody;
                    return {};
                };
                std::vector<std::string> frameBodies{ profile->mLandmarks.mPelvis };
                frameBodies.insert(frameBodies.end(), profile->mLandmarks.mSpine.begin(), profile->mLandmarks.mSpine.end());
                if (profile->mLandmarks.mHead)
                    frameBodies.push_back(*profile->mLandmarks.mHead);
                for (const auto& footName : { profile->mLandmarks.mLeftFoot, profile->mLandmarks.mRightFoot })
                    if (footName)
                    {
                        const std::string calf = parentOfBody(*footName);
                        frameBodies.insert(frameBodies.end(), { *footName, calf, parentOfBody(calf) });
                    }
                RagdollCore::SkeletonDescriptor frames;
                frames.mId = creationSkeleton.mId;
                // The skeleton's bones and leg lengths name it (the same rig at
                // the same size shares one standing pose). Only thigh-calf and
                // calf-foot: no joint between them, so the length does not
                // change with the pose (across the neck it did, and a running
                // build never found the standing one).
                std::string key = creationSkeleton.mId;
                for (const auto& bone : creationSkeleton.mBones)
                    key += ";" + bone.mName;
                for (const auto& footName : { profile->mLandmarks.mLeftFoot, profile->mLandmarks.mRightFoot })
                    if (footName)
                    {
                        const std::string calf = parentOfBody(*footName);
                        const auto* a = findBone(boneOfBody(parentOfBody(calf)));
                        const auto* b = findBone(boneOfBody(calf));
                        const auto* c = findBone(boneOfBody(*footName));
                        if (a && b && c)
                            key += ":" + std::to_string(std::lround((b->mTranslation - a->mTranslation).Length()))
                                + "," + std::to_string(std::lround((c->mTranslation - b->mTranslation).Length()));
                    }
                for (const auto& bone : creationSkeleton.mBones)
                {
                    for (const std::string& body : frameBodies)
                        if (!body.empty() && boneOfBody(body) == bone.mName)
                        {
                            frames.mBones.push_back(bone);
                            frames.mBones.back().mParent = -1;
                            break;
                        }
                }
                const auto modelBone = [&](const std::optional<std::string>& body) -> const RagdollCore::Transform* {
                    return body ? findBone(boneOfBody(*body)) : nullptr;
                };
                const RagdollCore::Transform* pelvis = findBone(boneOfBody(profile->mLandmarks.mPelvis));
                const RagdollCore::Transform* left = modelBone(profile->mLandmarks.mLeftFoot);
                const RagdollCore::Transform* right = modelBone(profile->mLandmarks.mRightFoot);
                const RagdollCore::Transform* chest = profile->mLandmarks.mSpine.empty()
                    ? nullptr : findBone(boneOfBody(profile->mLandmarks.mSpine.back()));
                bool standing = pelvis && left && right && chest && foundActor->second->getOnGround()
                    && foundActor->second->getQueuedVelocity().length2() < 1.0f;
                if (standing)
                {
                    const auto flat = [](JPH::Vec3Arg a, JPH::Vec3Arg b) {
                        return std::hypot(a.GetX() - b.GetX(), a.GetY() - b.GetY());
                    };
                    standing = flat(left->mTranslation, pelvis->mTranslation) < 15.0f
                        && flat(right->mTranslation, pelvis->mTranslation) < 15.0f
                        && std::abs(left->mTranslation.GetZ() - right->mTranslation.GetZ()) < 4.0f
                        && flat(chest->mTranslation, pelvis->mTranslation) < 8.0f;
                }
                auto reference = mStandingFrameSkeletons.find(key);
                const bool captured = standing && reference == mStandingFrameSkeletons.end();
                if (captured)
                    reference = mStandingFrameSkeletons.emplace(key, std::move(frames)).first;
                if (reference != mStandingFrameSkeletons.end())
                    buildOptions.mFrameSkeleton = &reference->second;
                frameKey = key;
                Log(Debug::Info) << "RAGDOLL_BUILD_FRAMES actor=" << ptr.getCellRef().getRefId()
                                 << " standing=" << standing << " captured=" << captured
                                 << " source=" << (buildOptions.mFrameSkeleton ? "standing" : "build");
            }
            // Diagnostics (2026-10-05): a body built before the weapon is at its
            // ready stance (drawn under 0.8 s), as play builds it mid-draw,
            // gets its weapon hand turned by "x,y,z" degrees (hand frame): the
            // test stand-in for play's mid-draw build pose (REPLAY-08).
            // OPENMW_RAGDOLL_BUILD_WRIST_TEST.
            if (const char* spec = RagdollCore::tuningText("OPENMW_RAGDOLL_BUILD_WRIST_TEST");
                spec && ptr == MWMechanics::getPlayer() && weaponDrawnFor(ptr) < 0.8f && profile->mLandmarks.mRightHand)
            {
                float x = 0, y = 0, z = 0, w = 0;
                const int read = std::sscanf(spec, "%f,%f,%f,%f", &x, &y, &z, &w);
                if (read >= 3)
                    for (const auto& body : runtimeProfile->mBodies)
                        if (body.mName == *profile->mLandmarks.mRightHand)
                            for (auto& bone : creationSkeleton.mBones)
                                if (bone.mName == body.mBone)
                                {
                                    // three values: degrees about x, y, z; four: a quaternion
                                    const JPH::Quat delta = read == 4 ? JPH::Quat(x, y, z, w).Normalized()
                                        : JPH::Quat::sEulerAngles(JPH::Vec3(osg::DegreesToRadians(x),
                                            osg::DegreesToRadians(y), osg::DegreesToRadians(z)));
                                    bone.mModelTransform.mRotation = (bone.mModelTransform.mRotation * delta).Normalized();
                                    Log(Debug::Info) << "RAGDOLL_BUILD_WRIST_TEST turned=" << spec
                                                     << " drawn_for=" << weaponDrawnFor(ptr);
                                }
            }
            // The whole weapon arm as play built it (REPLAY-08): "u:q;f:q;h:q",
            // each the upper arm's / forearm's / hand's rotation from its parent
            // (torso, upper arm, forearm) as quaternions; the bones below move
            // with their parents. OPENMW_RAGDOLL_BUILD_ARM_TEST.
            if (const char* spec = RagdollCore::tuningText("OPENMW_RAGDOLL_BUILD_ARM_TEST");
                spec && ptr == MWMechanics::getPlayer() && weaponDrawnFor(ptr) < 0.8f && profile->mLandmarks.mRightHand)
            {
                std::array<JPH::Quat, 3> rel{ JPH::Quat::sIdentity(), JPH::Quat::sIdentity(), JPH::Quat::sIdentity() };
                std::array<bool, 3> have{};
                for (std::string part : { std::string(spec) })
                {
                    std::size_t from = 0;
                    while (from < part.size())
                    {
                        const std::size_t to = std::min(part.find(';', from), part.size());
                        char key = 0;
                        float x, y, z, w;
                        if (std::sscanf(part.substr(from, to - from).c_str(), "%c:%f,%f,%f,%f", &key, &x, &y, &z, &w) == 5)
                        {
                            const int i = key == 'u' ? 0 : key == 'f' ? 1 : key == 'h' ? 2 : -1;
                            if (i >= 0)
                            {
                                rel[i] = JPH::Quat(x, y, z, w).Normalized();
                                have[i] = true;
                            }
                        }
                        from = to + 1;
                    }
                }
                // the chain torso -> upper arm -> forearm -> hand, by body
                const auto boneOf = [&](const std::string& bodyName) -> RagdollCore::SkeletonBone* {
                    for (const auto& body : runtimeProfile->mBodies)
                        if (body.mName == bodyName)
                            for (auto& bone : creationSkeleton.mBones)
                                if (bone.mName == body.mBone)
                                    return &bone;
                    return nullptr;
                };
                const auto parentBody = [&](const std::string& bodyName) -> std::string {
                    for (const auto& body : runtimeProfile->mBodies)
                        if (body.mName == bodyName && body.mParentBody)
                            return *body.mParentBody;
                    return {};
                };
                const std::string hand = *profile->mLandmarks.mRightHand;
                const std::string forearm = parentBody(hand);
                const std::string upper = parentBody(forearm);
                const std::string torso = profile->mLandmarks.mSpine.empty() ? std::string() : profile->mLandmarks.mSpine.back();
                std::array<RagdollCore::SkeletonBone*, 4> chain{ boneOf(torso), boneOf(upper), boneOf(forearm), boneOf(hand) };
                if (chain[0] && chain[1] && chain[2] && chain[3])
                {
                    std::array<RagdollCore::Transform, 4> old;
                    for (int i = 0; i < 4; ++i)
                        old[i] = chain[i]->mModelTransform;
                    for (int i = 1; i < 4; ++i)
                    {
                        const JPH::Quat parentNew = chain[i - 1]->mModelTransform.mRotation;
                        if (i > 1)
                            chain[i]->mModelTransform.mTranslation = chain[i - 1]->mModelTransform.mTranslation
                                + parentNew * (old[i - 1].mRotation.Conjugated() * (old[i].mTranslation - old[i - 1].mTranslation));
                        if (have[i - 1])
                            chain[i]->mModelTransform.mRotation = (parentNew * rel[i - 1]).Normalized();
                        else
                            chain[i]->mModelTransform.mRotation
                                = (parentNew * (old[i - 1].mRotation.Conjugated() * old[i].mRotation)).Normalized();
                    }
                    Log(Debug::Info) << "RAGDOLL_BUILD_ARM_TEST drawn_for=" << weaponDrawnFor(ptr);
                }
            }
            if (const auto prepared = mPreparedRagdollWeapons.find(ptr.mRef); prepared != mPreparedRagdollWeapons.end())
            {
                weaponAttachment = appendLiveTrainingWeapon(creationSkeleton, *runtimeProfile, buildOptions,
                    prepared->second.mRecordId, prepared->second.mItemWeight, prepared->second.mSpeed,
                    prepared->second.mReach,
                    MWBase::Environment::get().getWorld()->getAnimation(ptr));
                if (!weaponAttachment)
                {
                    Log(Debug::Warning) << "Powered ragdoll weapon creation failed for " << ptr.getCellRef().getRefId()
                                        << ": live training fixture is invalid";
                    return false;
                }
                creationProfile = runtimeProfile.get();
            }
            // OPENMW_RAGDOLL_SHIELD_BODY=0: no shield body (the left arm
            // alone counts as the shield, as before).
            static const auto shieldBody_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SHIELD_BODY");
                return !(text && std::atoi(text) == 0);
            });
            static const auto& shieldBody = shieldBody_tuned.get();
            if (shieldBody && ptr.getClass().hasInventoryStore(ptr))
            {
                MWWorld::InventoryStore& inventory = ptr.getClass().getInventoryStore(ptr);
                const auto left = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
                if (left != inventory.end() && left->getType() == ESM::Armor::sRecordId)
                    shieldAttachment = appendLiveShield(creationSkeleton, *runtimeProfile,
                        left->getCellRef().getRefId().serializeText(), left->getClass().getWeight(*left),
                        MWBase::Environment::get().getWorld()->getAnimation(ptr));
            }
        }

        RagdollCore::PassiveRagdollInitialState initialState;
        initialState.mActorMotion.mLinearVelocity
            = Misc::Convert::toJolt<JPH::Vec3>(foundActor->second->getQueuedWorldVelocity());
        if (const auto history = mPassiveRagdollMotionHistory.find(ptr.mRef); !powered
            && history != mPassiveRagdollMotionHistory.end()
            && history->second->mLatestMotion.size() == profile->mBodies.size())
            initialState.mBodyMotions = history->second->mLatestMotion;
        mPassiveRagdollMotionHistory.erase(ptr.mRef);
        if (!powered)
        {
            if (const auto impact = mPendingRagdollImpacts.find(ptr.mRef); impact != mPendingRagdollImpacts.end())
            {
                initialState.mKillingImpulse = RagdollCore::makeKillingImpulse(*snapshot.mSkeleton, *profile,
                    buildOptions, Misc::Convert::toJolt<JPH::RVec3>(impact->second.mWorldPoint),
                    Misc::Convert::toJolt<JPH::Vec3>(impact->second.mDirection), impact->second.mHealthDamage);
            }
        }

        auto entry = std::make_unique<PassiveRagdollEntry>(ptr, mNextPassiveRagdollSerial++, mJoltDebugDrawer.get());
        entry->mSurfaceQuery = [this, ptr](JPH::RVec3Arg origin, JPH::Vec3Arg direction, float distance)
            -> std::optional<std::pair<JPH::RVec3, JPH::Vec3>> {
            const osg::Vec3f from = Misc::Convert::toOsg(origin);
            const osg::Vec3f to = from + Misc::Convert::toOsg(direction) * distance;
            // Terrain is its own layer: without HEIGHTMAP every foot target
            // outdoors missed the ground and fell back to the actor's height,
            // so on Balmora's slopes the feet stepped on a flat plane (the
            // uphill foot in the ground, the downhill one hovering; play test
            // 2026-09-28). The test floor is a WORLD mesh, so tests never saw it.
            // OPENMW_RAGDOLL_FOOT_RAY_TERRAIN=0 restores the old mask (comparison).
            static const auto terrain_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_FOOT_RAY_TERRAIN");
                return !(text && std::atoi(text) == 0);
            });
            static const auto& terrain = terrain_tuned.get();
            const auto hit = castRay(from, to, { ptr }, {},
                MWPhysics::Layers::WORLD | MWPhysics::Layers::DOOR | (terrain ? MWPhysics::Layers::HEIGHTMAP : 0));
            if (!hit.mHit || !hit.mHitPos.valid() || !hit.mHitNormal.valid())
                return std::nullopt;
            return std::make_pair(Misc::Convert::toJolt<JPH::RVec3>(hit.mHitPos),
                Misc::Convert::toJolt<JPH::Vec3>(hit.mHitNormal));
        };
        entry->mPowered = powered;
        entry->mStrikeCooldown = RagdollCore::StrikeCooldown(contactHitCooldownSeconds(), mPhysicsDt);
        entry->mReplayCreationSkeleton = creationSkeleton;
        entry->mReplayCreationOptions = buildOptions;
        entry->mFrameKey = frameKey;
        entry->mFramesFromStanding = buildOptions.mFrameSkeleton != nullptr;
        if (buildOptions.mFrameSkeleton != nullptr)
            for (std::size_t i = 0; i < creationProfile->mBodies.size(); ++i)
                for (const auto& bone : buildOptions.mFrameSkeleton->mBones)
                    if (bone.mName == creationProfile->mBodies[i].mBone)
                        entry->mStandingBodyRotations[i] = bone.mModelTransform.mRotation;
        entry->mReplayCreationMotion = initialState.mActorMotion;
        entry->mExperimentalReferenceRotation = buildOptions.mRootRotation;
        entry->mExperimentalUniformScale = buildOptions.mUniformScale;
        entry->mTransitionSourcePose.mBodyTransforms.reserve(creationSkeleton.mBones.size());
        for (const RagdollCore::SkeletonBone& bone : creationSkeleton.mBones)
            entry->mTransitionSourcePose.mBodyTransforms.push_back(bone.mModelTransform);
        if (powered)
        {
            entry->mMotorTargetPose = entry->mTransitionSourcePose;
            entry->mMotorTargetCaptured = true;
        }
        entry->mModeTransitionDuration = profile->mPoseTransition.mDeathBlendDuration;
        const std::uint64_t userData = static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(entry.get()));
        const JPH::CollisionGroup::GroupID group = mNextPassiveRagdollGroup++;
        if (mNextPassiveRagdollGroup == 0)
            mNextPassiveRagdollGroup = 1;

        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        RagdollCore::PassiveRagdollCreateResult created = RagdollCore::createPassiveRagdoll(
            mPhysicsSystem, creationSkeleton, *creationProfile, buildOptions, initialState, group, userData);
        if (!created.isValid())
        {
            logRagdollIssues(ptr, "Jolt creation", created.mValidation);
            return false;
        }

        entry->mRagdoll = std::move(created.mRagdoll);
        if (profile->mActiveController.mLocomotionRoute == RagdollCore::LocomotionRoute::KinematicRoot)
            entry->mRagdoll->setPoweredPositionSolverSteps(
                PassiveRagdollEntry::sKinematicRootPositionSolverSteps);
        entry->mPoseBinding = std::move(snapshot.mPoseBinding);
        entry->mPoseBindingBodyCount = snapshot.mSkeleton->mBones.size();
        entry->mRuntimeProfile = std::move(runtimeProfile);
        entry->mWeapon = std::move(weaponAttachment);
        entry->mShield = std::move(shieldAttachment);
#ifdef OPENMW_RAGDOLL_DEV_API
        if (entry->mWeapon && entry->mWeapon->mGeometryTraceId != 0)
        {
            const auto& weapon = *entry->mWeapon;
            Log(Debug::Info) << std::dec << std::setprecision(17)
                << "RAGDOLL_WEAPON_GEOMETRY_BIND schema=1 geometry_id=" << weapon.mGeometryTraceId
                << " actor_serial=" << entry->mSerial << " record=" << std::quoted(weapon.mRecordId)
                << " weapon_index=" << weapon.mWeaponBody
                << " weapon_body_id=" << entry->mRagdoll->getBodyId(weapon.mWeaponBody).GetIndexAndSequenceNumber()
                << " hand_index=" << weapon.mHandBody
                << " hand_body_id=" << entry->mRagdoll->getBodyId(weapon.mHandBody).GetIndexAndSequenceNumber()
                << " uniform_scale=" << buildOptions.mUniformScale
                << " unscaled_body_origin_in_hand=" << weapon.mCenterFromHand.GetX() << ','
                << weapon.mCenterFromHand.GetY() << ',' << weapon.mCenterFromHand.GetZ();
        }
#endif
        bool supportGripEnabled = true;
#ifdef OPENMW_RAGDOLL_DEV_API
        if (const char* supportGrip = RagdollCore::tuningText("OPENMW_RAGDOLL_TWO_HAND_GRIP"))
            supportGripEnabled = std::string_view(supportGrip) != "0";
#endif
        // NPCs too with NPC_PARITY (their left hand had only an IK target and
        // drifted off the hilt: STEADY-02 claymore grip 0%).
        // GRIP-05: a one-handed weapon gets the support hand's grip too when a
        // grip that uses it can be taken (half-sword, murder-stroke).
        const bool gripTwoHands = PassiveRagdollEntry::gripHalfSettings()[0] > 0.0f
            || PassiveRagdollEntry::gripMurderSettings()[0] > 0.0f;
        entry->mSupportGripEnabled = supportGripEnabled
            && powered && (ptr == MWMechanics::getPlayer() || PassiveRagdollEntry::npcParity()) && entry->mWeapon
            && (entry->mWeapon->mTwoHanded || gripTwoHands)
            && profile->mActiveController.mLocomotionRoute == RagdollCore::LocomotionRoute::KinematicRoot;
        entry->mAnimationAttackOwned = entry->mWeapon.has_value() && ptr == MWMechanics::getPlayer();
        entry->mEquippedWeaponAtCreation = entry->mWeapon ? entry->mWeapon->mRecordId : std::string{};
        entry->mBuiltReady = !entry->mWeapon || weaponDrawnFor(ptr) >= 0.8f;
        entry->mEquippedShieldAtCreation = powered ? equippedShieldId(ptr) : std::string{};
        if (entry->mWeapon)
        {
            entry->mRagdoll->setPoweredVelocitySolverSteps(entry->mWeapon->mVelocitySolverSteps);
            if (!entry->mRagdoll->setConstraintPriority(entry->mWeapon->mWeaponBody, 1)
                || !entry->mRagdoll->setConstraintSolverSteps(entry->mWeapon->mWeaponBody,
                    PassiveRagdollEntry::sWeaponGripSolverSteps, PassiveRagdollEntry::sWeaponGripSolverSteps))
                return false;
        }
        entry->mInheritedAnimationMotion = !initialState.mBodyMotions.empty();
        const auto findBodyIndex = [&](std::string_view name) -> std::optional<std::size_t> {
            const auto body = std::find_if(creationProfile->mBodies.begin(), creationProfile->mBodies.end(),
                [&](const RagdollCore::BodyProfile& value) { return value.mName == name; });
            if (body == creationProfile->mBodies.end())
                return std::nullopt;
            return static_cast<std::size_t>(std::distance(creationProfile->mBodies.begin(), body));
        };
        if (creationProfile->mLandmarks.mLeftFoot)
            entry->mLeftFootBodyIndex = findBodyIndex(*creationProfile->mLandmarks.mLeftFoot);
        if (creationProfile->mLandmarks.mRightFoot)
            entry->mRightFootBodyIndex = findBodyIndex(*creationProfile->mLandmarks.mRightFoot);
        const auto findLegChain = [&](std::optional<std::size_t> footIndex) -> std::optional<RagdollLegChain> {
            if (!footIndex || *footIndex >= creationProfile->mBodies.size())
                return std::nullopt;
            const auto& foot = creationProfile->mBodies[*footIndex];
            if (!foot.mParentBody)
                return std::nullopt;
            const auto kneeIndex = findBodyIndex(*foot.mParentBody);
            if (!kneeIndex || *kneeIndex >= creationProfile->mBodies.size())
                return std::nullopt;
            const auto& knee = creationProfile->mBodies[*kneeIndex];
            if (!knee.mParentBody)
                return std::nullopt;
            const auto hipIndex = findBodyIndex(*knee.mParentBody);
            if (!hipIndex)
                return std::nullopt;
            return RagdollLegChain{ *hipIndex, *kneeIndex, *footIndex };
        };
        entry->mLeftLeg = findLegChain(entry->mLeftFootBodyIndex);
        entry->mRightLeg = findLegChain(entry->mRightFootBodyIndex);
#ifdef OPENMW_RAGDOLL_DEV_API
        // Experiment (DUEL-A01): the calf-foot joint opens 2-2.9 units when a
        // falling foot hits the floor. OPENMW_RAGDOLL_ANKLE_SOLVE =
        // "priority,velocitySteps,positionSteps" solves the ankles after the
        // rest of the leg with their own iterations, as the weapon grip is.
        if (powered)
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_ANKLE_SOLVE"))
            {
                unsigned priority = 0, velocity = 0, position = 0;
                if (std::sscanf(text, "%u,%u,%u", &priority, &velocity, &position) == 3)
                    for (const auto& foot : { entry->mLeftFootBodyIndex, entry->mRightFootBodyIndex })
                        if (foot)
                        {
                            entry->mRagdoll->setConstraintPriority(*foot, priority);
                            entry->mRagdoll->setConstraintSolverSteps(*foot, velocity, position);
                        }
            }
#endif
        // The arm walks the same parent links as the leg, one further: hand,
        // forearm, upper arm, and the clavicle those hang from. The clavicle
        // is the frame, not part of the chain, so an arm on a profile that
        // attaches the shoulder straight to the spine still resolves.
        const auto findArmChain = [&](const std::optional<std::string>& handName)
            -> std::optional<RagdollArmChain> {
            if (!handName)
                return std::nullopt;
            const auto handIndex = findBodyIndex(*handName);
            if (!handIndex || *handIndex >= creationProfile->mBodies.size())
                return std::nullopt;
            const auto& hand = creationProfile->mBodies[*handIndex];
            if (!hand.mParentBody)
                return std::nullopt;
            const auto elbowIndex = findBodyIndex(*hand.mParentBody);
            if (!elbowIndex || *elbowIndex >= creationProfile->mBodies.size())
                return std::nullopt;
            const auto& elbow = creationProfile->mBodies[*elbowIndex];
            if (!elbow.mParentBody)
                return std::nullopt;
            const auto shoulderIndex = findBodyIndex(*elbow.mParentBody);
            if (!shoulderIndex || *shoulderIndex >= creationProfile->mBodies.size())
                return std::nullopt;
            const auto& shoulder = creationProfile->mBodies[*shoulderIndex];
            if (!shoulder.mParentBody)
                return std::nullopt;
            const auto frameIndex = findBodyIndex(*shoulder.mParentBody);
            if (!frameIndex)
                return std::nullopt;
            return RagdollArmChain{ *frameIndex, *shoulderIndex, *elbowIndex, *handIndex };
        };
        entry->mLeftArm = findArmChain(creationProfile->mLandmarks.mLeftHand);
        entry->mRightArm = findArmChain(creationProfile->mLandmarks.mRightHand);
        entry->mPelvisBodyIndex = findBodyIndex(creationProfile->mLandmarks.mPelvis);
        if (creationProfile->mLandmarks.mHead)
            entry->mHeadBodyIndex = findBodyIndex(*creationProfile->mLandmarks.mHead);
        entry->mTorsoBodyIndex = creationProfile->mLandmarks.mSpine.empty()
            ? entry->mPelvisBodyIndex
            : findBodyIndex(creationProfile->mLandmarks.mSpine.back());
        entry->mSpineBodyIndices.clear();
        entry->mChestCarried.clear();
        for (const auto& name : creationProfile->mLandmarks.mSpine)
            if (const auto index = findBodyIndex(name))
                entry->mSpineBodyIndices.push_back(*index);
        if (entry->mTorsoBodyIndex)
        {
            const std::string& chest = creationProfile->mBodies[*entry->mTorsoBodyIndex].mName;
            std::optional<std::string> neck;
            if (entry->mHeadBodyIndex)
                neck = creationProfile->mBodies[*entry->mHeadBodyIndex].mParentBody;
            for (std::size_t i = 0; i < creationProfile->mBodies.size(); ++i)
            {
                std::optional<std::string> parent = creationProfile->mBodies[i].mParentBody;
                for (int depth = 0; parent && depth < 12; ++depth)
                {
                    if (*parent == chest)
                    {
                        const bool head = (entry->mHeadBodyIndex && i == *entry->mHeadBodyIndex)
                            || (neck && creationProfile->mBodies[i].mName == *neck);
                        entry->mChestCarried.emplace_back(i, head);
                        break;
                    }
                    const auto next = findBodyIndex(*parent);
                    parent = next ? creationProfile->mBodies[*next].mParentBody : std::nullopt;
                }
            }
        }
        // DMG-01 anatomical registry snapshot: resolved once here so the
        // contact callback only reads immutable data. The appended weapon body
        // has no anatomical name and resolves to Unknown, which keeps the
        // legacy fallback for unsupported contacts.
        entry->mBodyRegions.clear();
        entry->mBodyRegions.reserve(creationProfile->mBodies.size());
        for (const auto& body : creationProfile->mBodies)
            entry->mBodyRegions.push_back(RagdollCore::regionForBodyName(body.mName));
        if (!entry->mRagdoll->addToPhysicsSystem())
            return false;
        if (!entry->updateGraspCollisionFilters(false))
        {
            entry->mRagdoll->removeFromPhysicsSystem();
            return false;
        }
        entry->initializeControllerObservation();
        entry->mCurrentPhysicsPoseCaptured = entry->mRagdoll->captureModelPose(entry->mCurrentPhysicsPose);
        if (!entry->mCurrentPhysicsPoseCaptured)
        {
            entry->mRagdoll->removeFromPhysicsSystem();
            return false;
        }
        // The kinematic-root route takes the root immediately and upright:
        // the game actor owns locomotion from the first tick, and the captured
        // insertion rotation must not be able to latch a physical fall before
        // the first root move.
        if (powered && profile->mActiveController.mLocomotionRoute
                == RagdollCore::LocomotionRoute::KinematicRoot
            && !entry->mCurrentPhysicsPose.mBodyTransforms.empty())
        {
            // The root keeps the pelvis bind orientation: the controller's
            // local-up axis is derived from it, so an identity rotation would
            // report an upright body as fallen. The per-tick update replaces
            // both values with the live animation pelvis.
            entry->mKinematicRootLocalOffset
                = entry->mCurrentPhysicsPose.mBodyTransforms[0].mTranslation;
            entry->mKinematicRootLocalRotation
                = entry->mCurrentPhysicsPose.mBodyTransforms[0].mRotation;
            entry->mRagdoll->setSoftRoot(softRootSettings());
            entry->mRagdoll->setSoftRootAnchorLayer(Layers::VISUAL_ONLY);
            if (mPassiveRagdollProfile)
            {
                std::vector<bool> upper, torso, feet;
                // The masks must cover the rig as built. An equipped weapon
                // (drawn or sheathed) appends a body to the runtime profile
                // only; masks from the base profile were one short, and
                // applySoftRootWrench then fell back for every body: legs
                // counted as upper parts (their weight carried by the balance
                // force) and no foot ever grounded (no ground routing, air
                // catch always on). Armed walking in every direction had both
                // feet >9 units up 26-37% of the time against 0-1% unarmed,
                // stance feet 8.6 units up against 2.9, the swing foot 33-36
                // high against the clip's 21 and 45-52 ahead of the pelvis
                // against 28 ("steps too long", 2026-09-24 play test R3).
                // OPENMW_RAGDOLL_SOFT_ROOT_RIG_MASKS=0 restores the old masks.
                static const auto rigMasks_tuned = RagdollCore::tuned<bool>([] {
                    const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SOFT_ROOT_RIG_MASKS");
                    return !(text && std::atoi(text) == 0);
                });
                static const auto& rigMasks = rigMasks_tuned.get();
                const RagdollCore::RagdollProfile& maskProfile = rigMasks && entry->mRuntimeProfile
                    ? *entry->mRuntimeProfile
                    : *mPassiveRagdollProfile;
                for (const auto& body : maskProfile.mBodies)
                {
                    const std::string& name = body.mName;
                    const bool leg = name.find("thigh") != std::string::npos || name.find("calf") != std::string::npos
                        || name.find("foot") != std::string::npos;
                    upper.push_back(!leg);
                    // OPENMW_RAGDOLL_SOFT_ROOT_TORSO_CHAIN = 0: the balance torque
                    // turns the pelvis alone, the spine motors carry the chest to
                    // the animation's lean (test knob, 2026-09-28).
                    static const auto torsoChain_tuned = RagdollCore::tuned<bool>([] {
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SOFT_ROOT_TORSO_CHAIN");
                        return !(text && std::atoi(text) == 0);
                    });
                    static const auto& torsoChain = torsoChain_tuned.get();
                    torso.push_back(name == "pelvis"
                        || (torsoChain && (name == "spine" || name == "spine1" || name == "spine2")));
                    feet.push_back(name.find("foot") != std::string::npos);
                }
                std::vector<JPH::BodyID> core;
                for (std::size_t i = 0; i < mPassiveRagdollProfile->mBodies.size()
                     && i < entry->mRagdoll->getBodyCount(); ++i)
                {
                    const std::string& name = mPassiveRagdollProfile->mBodies[i].mName;
                    // Hands (fists), weapons (not ragdoll bodies) and the torso
                    // and head touch another fighter, so bodies cannot pass
                    // through each other; arms and legs do not (arms caught the
                    // other body's legs and tripped both at walking speed).
                    static const auto torsoContact_tuned = RagdollCore::tuned<bool>([] {
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_TORSO_CONTACT");
                        return !(text && std::atoi(text) == 0);
                    });
                    static const auto& torsoContact = torsoContact_tuned.get();
                    const bool torsoPart = name == "pelvis" || name == "spine" || name == "spine1"
                        || name == "spine2" || name == "neck" || name == "head";
                    const bool fighting = name.find("hand") != std::string::npos || (torsoContact && torsoPart);
                    if (!fighting)
                        core.push_back(entry->mRagdoll->getBodyId(i));
                }
                std::sort(core.begin(), core.end());
                entry->mSoftRootCoreBodies = std::move(core);
                entry->mRagdoll->setSoftRootBodies(std::move(upper), std::move(torso), std::move(feet));
            }
            entry->mRagdoll->setRootKinematic(true);
            const JPH::RVec3 rootTarget = buildOptions.mRootPosition
                + JPH::RVec3(buildOptions.mRootRotation * *entry->mKinematicRootLocalOffset);
            mPhysicsSystem.GetBodyInterface().SetPositionAndRotation(
                entry->mRagdoll->getBodyId(0), rootTarget,
                (buildOptions.mRootRotation * *entry->mKinematicRootLocalRotation).Normalized(),
                JPH::EActivation::Activate);
        }
        entry->mPreviousPhysicsPose = entry->mCurrentPhysicsPose;
        if (entry->mModeTransitionDuration > 0.0f)
            entry->mRenderPose = entry->mTransitionSourcePose;
        else
        {
            entry->mRenderPose = entry->mCurrentPhysicsPose;
            entry->mModeTransitionFactor = 1.0f;
        }
        entry->mRenderPosePrepared = true;

        // Transaction commit: only switch off the ordinary actor collider after
        // the complete ragdoll has entered broadphase successfully. The
        // kinematic-root route keeps the capsule: gameplay owns locomotion and
        // needs the ordinary world collision for stairs, slopes and walls.
        // The layer matrix already keeps RAGDOLL from colliding with ACTOR.
        {
            const bool keepActorCapsule = powered
                && profile->mActiveController.mLocomotionRoute
                    == RagdollCore::LocomotionRoute::KinematicRoot;
            // Powered kinematic bodies keep only world collision (stairs,
            // slopes, walls); other actor capsules no longer block them, so
            // physical limbs own body-to-body contact.
            foundActor->second->enableCollisionBody(keepActorCapsule, false);
        }
        const bool inheritedAnimationMotion = entry->mInheritedAnimationMotion;
        const bool weaponCreated = entry->mWeapon.has_value();
        mPassiveRagdolls.emplace(ptr.mRef, std::move(entry));
        mPreparedRagdollWeapons.erase(ptr.mRef);
#ifdef OPENMW_RAGDOLL_DEV_API
        if (powered && OMW::FpsFixture::requestedRate())
            OMW::FpsFixture::armed = true;
#endif
        ++mPassiveRagdollsCreated;
        if (weaponCreated)
        {
            ++mWeaponBodiesCreated;
            ++mWeaponCallbacksArmed;
        }
        mPassiveRagdollMotionTransfers += inheritedAnimationMotion;
        mPendingRagdollImpacts.erase(ptr.mRef);
        if (!powered)
            enforcePassiveRagdollCap();
        return true;
    }

#ifdef OPENMW_RAGDOLL_DEV_API
    std::uint64_t PhysicsSystem::stageWoundReceipt(const MWWorld::Ptr& ptr, std::string_view body, int kind,
        const osg::Vec3f& point, const osg::Vec3f& normal)
    {
        // Native diagnostic seam: exercise the real Lua receipt, renderer and
        // controller without claiming the fixture is a measured weapon hit.
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || kind < 1 || kind > 3 || !point.valid()
            || !normal.valid() || normal.length2() < 0.5f)
            return 0;
        auto& entry = *found->second;
        const auto region = RagdollCore::regionForBodyName(body);
        if (region == RagdollCore::AnatomicalRegion::Unknown)
            return 0;
        const auto bodyIt = std::find(entry.mBodyRegions.begin(), entry.mBodyRegions.end(), region);
        if (bodyIt == entry.mBodyRegions.end())
            return 0;
        const auto index = static_cast<std::uint32_t>(bodyIt - entry.mBodyRegions.begin());
        auto& bodies = mPhysicsSystem.GetBodyInterface();
        const auto bodyId = entry.mRagdoll->getBodyId(index);
        const auto inverse = bodies.GetRotation(bodyId).Conjugated();
        RagdollCore::WoundContactSnapshot wound;
        wound.mKind = static_cast<RagdollCore::WoundKind>(kind);
        wound.mTargetLocalPoint = inverse * JPH::Vec3(Misc::Convert::toJolt<JPH::RVec3>(point)
            - bodies.GetCenterOfMassPosition(bodyId));
        wound.mTargetLocalNormal = inverse * Misc::Convert::toJolt<JPH::Vec3>(normal);
        wound.mTargetLocalTangent = inverse * JPH::Vec3::sAxisX();
        static std::uint64_t nextTestId = std::uint64_t{1} << 48;
        const auto id = ++nextTestId;
        while (entry.mPendingWounds.size() >= 32)
            entry.mPendingWounds.erase(entry.mPendingWounds.begin());
        entry.mPendingWounds.emplace(id, PassiveRagdollEntry::PendingWound{ RagdollCore::WeaponContactEvent(
            id, entry.mSerial + 1, entry.mSerial, "diagnostic", 0, bodyId.GetIndexAndSequenceNumber(),
            mJoltStepSerial, id, true, Misc::Convert::toJolt<JPH::RVec3>(point),
            -Misc::Convert::toJolt<JPH::Vec3>(normal), 1, 1, 1, index, region, entry.mSerial, wound) });
        return id;
    }
#endif

    bool PhysicsSystem::beginPhysicalHitReceipt(const MWWorld::Ptr& ptr, std::uint64_t id)
    {
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return false;
        auto& entry = *found->second;
        const auto pending = entry.mPendingWounds.find(id);
        if (pending == entry.mPendingWounds.end() || pending->second.mClaimed)
            return false;
        if (!pending->second.mContact.matchesGeneration(entry.mSerial)
            || mJoltStepSerial - pending->second.mContact.physicsStep() > 300)
        {
            entry.mPendingWounds.erase(pending);
            return false;
        }
        pending->second.mClaimed = true;
        return true;
    }

    void PhysicsSystem::finishPhysicalHitReceipt(
        const MWWorld::Ptr& ptr, std::uint64_t id, float health, float fatigue)
    {
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return;
        auto& entry = *found->second;
        const auto pending = entry.mPendingWounds.find(id);
        if (pending == entry.mPendingWounds.end() || !pending->second.mClaimed)
            return;
        const auto contact = pending->second.mContact;
        entry.mPendingWounds.erase(pending);
        const auto& wound = contact.wound();
        // A blow without a wound kind (no blade surface profile) is priced by
        // its health damage alone, like a slash: it still hurts and reacts.
        const bool woundKind = wound.mKind != RagdollCore::WoundKind::None;
        const float severity = woundKind ? RagdollCore::woundSeverity(wound.mKind, health, fatigue)
            : RagdollCore::woundSeverity(RagdollCore::WoundKind::Slash, health, 0.0f);
        ++entry.mWoundReceiptCount;
        entry.mLastWoundHealth = health;
        entry.mLastWoundFatigue = fatigue;
        entry.mLastWoundKind = RagdollCore::woundKindName(wound.mKind);
        entry.mLastWoundRegion = RagdollCore::regionName(contact.region());
        if (severity <= 0 || contact.targetBodyIndex() >= entry.mRagdoll->getBodyCount())
            return;

        auto& bodies = mPhysicsSystem.GetBodyInterface();
        const auto bodyId = entry.mRagdoll->getBodyId(contact.targetBodyIndex());
        const JPH::Quat rotation = bodies.GetRotation(bodyId);
        const JPH::RVec3 point = bodies.GetCenterOfMassPosition(bodyId) + rotation * wound.mTargetLocalPoint;
        // Without a wound snapshot the contact's own normal (world space).
        const JPH::Vec3 normal = woundKind ? JPH::Vec3(rotation * wound.mTargetLocalNormal) : -contact.normal();
        const JPH::Vec3 tangent = rotation * wound.mTargetLocalTangent;
        // Gore (stage 6): a burst at a cut or stab, a bleed from a deep one.
        // Queued: the receipt may come from the Lua thread, and effects are
        // scene changes (spawned in updatePassiveRagdolls).
        if (goreSettings()[0] > 0.0f && woundKind && wound.mKind != RagdollCore::WoundKind::Bruise)
        {
            const int bursts = 1 + std::min(2, static_cast<int>(severity * 2.0f));
            for (int k = 0; k < bursts; ++k)
                entry.mPendingBlood.push_back({ point + JPH::RVec3(normal * (2.0f * k)),
                    goreSettings()[0] * (0.8f + 0.4f * std::min(1.0f, severity)) });
            if (severity >= goreSettings()[1] && entry.mBleeds.size() < 6)
                entry.mBleeds.push_back({ contact.targetBodyIndex(), wound.mTargetLocalPoint,
                    goreSettings()[2] * std::min(1.5f, severity), 0.2f, severity });
        }
        // OPENMW_RAGDOLL_WOUND_MARKS=0: no marks drawn (diagnostics, 10-06:
        // a hub reset crash in RigGeometry::cull, see PLAN.md).
        static const auto woundMarks_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_WOUND_MARKS");
            return !(text && std::atoi(text) == 0);
        });
        static const auto& woundMarks = woundMarks_tuned.get();
        if (!woundKind || !woundMarks)
            ; // no mark: the renderer needs a cut, stab or bruise
        else if (auto* animation = MWBase::Environment::get().getWorld()->getAnimation(ptr))
            if (animation->addLocalizedWound(RagdollCore::regionName(contact.region()),
                    static_cast<int>(wound.mKind), Misc::Convert::toOsg(point),
                    Misc::Convert::toOsg(normal), Misc::Convert::toOsg(tangent), severity))
            {
                ++entry.mWoundMarkCount;
                // WOUNDFX FX4 (level 4): the blade that cut flesh or cloth
                // carries the blood where it went in (0 grip .. 1 tip, the
                // contact code's measure; the pose at the receipt).
                const int surface = animation->getLastWoundSurface(); // 0 skin, 1 cloth, 2 armour
                const auto attacker = std::find_if(mPassiveRagdolls.begin(), mPassiveRagdolls.end(),
                    [&](const auto& c) { return c.second->mSerial == contact.attackerActorIdentity(); });
                if ((surface == 0 || surface == 1) && wound.mKind != RagdollCore::WoundKind::Bruise
                    && attacker != mPassiveRagdolls.end() && attacker->second->mWeapon
                    && attacker->second->mWeapon->mAttached && attacker->second->mWeapon->tipDistanceFromHand() > 1.0f)
                    if (auto* blade = MWBase::Environment::get().getWorld()->getAnimation(attacker->second->getPtr()))
                    {
                        const auto& weapon = *attacker->second->mWeapon;
                        const JPH::BodyID id = attacker->second->mRagdoll->getBodyId(weapon.mWeaponBody);
                        const float along = (bodies.GetRotation(id).Conjugated()
                                                    * JPH::Vec3(contact.point() - bodies.GetCenterOfMassPosition(id))
                                                + weapon.mCenterFromHand)
                                                .Dot(weapon.mBladeAxis)
                            / weapon.tipDistanceFromHand();
                        const float spread = 0.06f + 0.1f * std::min(severity, 1.0f);
                        const int bloodType = ptr.getType() == ESM::REC_NPC_ ? ptr.get<ESM::NPC>()->mBase->mBloodType
                            : ptr.getType() == ESM::REC_CREA ? ptr.get<ESM::Creature>()->mBase->mBloodType
                                                              : 0;
                        if (std::isfinite(along))
                            blade->addWeaponBlood(along - spread,
                                wound.mKind == RagdollCore::WoundKind::Puncture ? 1.0f : along + spread,
                                std::min(severity, 1.0f) * (surface == 1 ? 0.5f : 1.0f), bloodType);
                    }
            }

        // A pose flinch is local, and does not feed generic whole-body stun.
        const auto index = static_cast<std::size_t>(contact.region());
        const auto body = contact.targetBodyIndex();
        if (index > 0 && index < entry.mWoundReactions.size()
            && body < entry.mCurrentPhysicsPose.mBodyTransforms.size())
        {
            const auto& pose = entry.mCurrentPhysicsPose.mBodyTransforms;
            const int parent = entry.mReplayCreationSkeleton.mBones[body].mParent;
            const JPH::Vec3 lever = parent >= 0
                ? pose[body].mTranslation - pose[parent].mTranslation : JPH::Vec3::sAxisZ();
            const auto push = entry.mRagdoll->getModelVector(-normal);
            JPH::Vec3 axis = push ? lever.Cross(*push) : JPH::Vec3::sZero();
            if (axis.LengthSq() < 1e-4f)
                axis = pose[body].mRotation * JPH::Vec3::sAxisX();
            entry.mWoundAxes[index] = pose[body].mRotation.Conjugated() * axis.Normalized();
            entry.mWoundBodies[index] = body;
            // A small resolved hit still needs a readable withdrawal. The
            // square-root response curve preserves the regional angle caps
            // and existing motor budgets without amplifying actual damage.
            entry.mWoundReactions[index].begin(std::sqrt(severity));
        }
        entry.addRegionDamage(contact.region(), health + fatigue);
        Log(Debug::Info) << "RAGDOLL_WOUND_RECEIPT id=" << id << " actor=" << ptr.getCellRef().getRefId()
            << " kind=" << entry.mLastWoundKind << " region=" << entry.mLastWoundRegion
            << " health=" << health << " fatigue=" << fatigue << " marks=" << entry.mWoundMarkCount;
    }

    // RXN-02 M2: everything one counted blow does to the struck body, in one
    // place, read from the blow record (the strike loop and the scripted
    // test blow both come here). Returns the push given to the struck part.
    // Guard blows (user 10-07: "a good hit to the weapon/shield pushes away
    // correspondingly. A good hit with the hammer vs a shield should push back
    // and cause stagger"): a blow that lands on a weapon or a shield is a blow
    // like any other - through the one entry point, struck on the arm that
    // holds it, its push on the braced trunk - instead of the old band
    // impulses. Blocked: at most a flinch, unless the guard was beaten.
    // OPENMW_RAGDOLL_GUARD_BLOWS=1 (with REACTION_V2).
    // CROWD (user 10-09: "better AI when fighting multiple enemies, they start
    // overlapping each other because they grab the same path"; chosen: all
    // attack, spaced). OPENMW_RAGDOLL_CROWD=1.
    bool PhysicsSystem::crowdEnabled()
    {
        static const auto enabled_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CROWD");
            return text && std::atoi(text) != 0;
        });
        static const auto& enabled = enabled_tuned.get();
        return enabled;
    }

    // CLASH-V2 (user 10-09): see the clash branch in the strike loop.
    bool PhysicsSystem::clashV2Enabled()
    {
        static const auto enabled_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CLASH_V2");
            return text && std::atoi(text) != 0;
        });
        static const auto& enabled = enabled_tuned.get();
        return enabled;
    }

    // The arm behind a blade in a clash: Strength / 50 (0.6-1.5); a guard also
    // by how fresh it is (0.5 + 0.5 x fatigue share).
    float PhysicsSystem::clashArmFactor(const MWWorld::Ptr& actor, bool guard)
    {
        if (actor.isEmpty() || !actor.getClass().isActor())
            return 1.0f;
        // OPENMW_RAGDOLL_CLASH_ARM = how much Strength counts (an exponent on
        // the Strength factor, 1 = fully, 0.5 = its square root).
        static const auto armWeight_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CLASH_ARM");
            return text ? std::clamp(static_cast<float>(std::atof(text)), 0.0f, 2.0f) : 1.0f;
        });
        static const auto& armWeight = armWeight_tuned.get();
        const auto& stats = actor.getClass().getCreatureStats(actor);
        float factor = std::pow(
            std::clamp(stats.getAttribute(ESM::Attribute::Strength).getModified() / 50.0f, 0.6f, 1.5f), armWeight);
        if (guard)
        {
            const auto& fatigue = stats.getFatigue();
            const float fresh = fatigue.getModified() > 0.0f
                ? std::clamp(fatigue.getCurrent() / fatigue.getModified(), 0.0f, 1.0f)
                : 1.0f;
            factor *= 0.5f + 0.5f * fresh;
        }
        return factor;
    }

    // Vanilla's block fatigue (MWMechanics::blockMeleeAttack), for a blade
    // that stopped or turned a blow.
    void PhysicsSystem::payBlockFatigue(const MWWorld::Ptr& blocker, const MWWorld::Ptr& attacker, float attackStrength)
    {
        if (blocker.isEmpty() || !blocker.getClass().isActor())
            return;
        const auto& gmst = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
        static const float base = gmst.find("fFatigueBlockBase")->mValue.getFloat();
        static const float mult = gmst.find("fFatigueBlockMult")->mValue.getFloat();
        static const float weaponMult = gmst.find("fWeaponFatigueBlockMult")->mValue.getFloat();
        float loss = base + std::min(1.0f, blocker.getClass().getNormalizedEncumbrance(blocker)) * mult;
        if (!attacker.isEmpty() && attacker.getClass().hasInventoryStore(attacker))
        {
            auto& store = attacker.getClass().getInventoryStore(attacker);
            const auto held = store.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            if (held != store.end())
                loss += held->getClass().getWeight(*held) * attackStrength * weaponMult;
        }
        auto& stats = blocker.getClass().getCreatureStats(blocker);
        MWMechanics::DynamicStat<float> fatigue = stats.getFatigue();
        fatigue.setCurrent(fatigue.getCurrent() - loss);
        stats.setFatigue(fatigue);
        Log(Debug::Info) << "RAGDOLL_BLOCK_FATIGUE blocker=" << blocker.getCellRef().getRefId() << " loss=" << loss
                         << " left=" << fatigue.getCurrent();
    }

    bool PhysicsSystem::guardBlowsEnabled()
    {
        static const auto on_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_GUARD_BLOWS");
            return text && std::atoi(text) != 0;
        });
        static const auto& on = on_tuned.get();
        return on;
    }

    float PhysicsSystem::applyGuardBlow(PassiveRagdollEntry& side, const PassiveRagdollEntry& other,
        JPH::RVec3Arg point, JPH::Vec3Arg direction, float momentum, bool shield, bool beaten, bool blunt,
        std::uint64_t physicsStep)
    {
        if (!guardBlowsEnabled() || !side.mRagdoll || !side.mTorsoBodyIndex || !(momentum > 0.0f)
            || !std::isfinite(momentum))
            return 0.0f;
        RagdollCore::Blow blow;
        blow.mId = RagdollCore::nextBlowId();
        blow.mPhysicsStep = physicsStep;
        blow.mAttacker = other.mSerial;
        blow.mVictim = side.mSerial;
        blow.mKind = blunt ? RagdollCore::BlowKind::Blunt : RagdollCore::BlowKind::Edge;
        blow.mRegion = shield ? RagdollCore::AnatomicalRegion::LeftForearm : RagdollCore::AnatomicalRegion::RightForearm;
        blow.mBodyIndex = static_cast<std::uint32_t>(*side.mTorsoBodyIndex);
        blow.mPoint = point;
        blow.mDirection = direction.IsNearZero() ? JPH::Vec3::sAxisY() : direction.Normalized();
        const JPH::Vec3 apart(side.mRagdoll->getReferencePosition() - other.mRagdoll->getReferencePosition());
        blow.mAttackerToVictim = JPH::Vec3(apart.GetX(), apart.GetY(), 0.0f);
        blow.mApproachSpeed = momentum;
        blow.mCarriedMass = 1.0f;
        blow.mBlocked = true;
        blow.mGuardBeaten = beaten;
        blow.mVictimState = side.blowVictimState();
        const float pushed = applyBlowReaction(side, blow);
        if (!shield)
            side.knockWeapon(std::clamp(momentum / 2500.0f, 0.0f, 1.0f));
        Log(Debug::Info) << "RAGDOLL_GUARD_BLOW actor=" << side.getPtr().getCellRef().getRefId()
                         << " from=" << other.getPtr().getCellRef().getRefId() << " shield=" << shield
                         << " momentum=" << momentum << " beaten=" << beaten << " blunt=" << blunt << " pushed=" << pushed;
        return pushed;
    }

    float PhysicsSystem::applyBlowReaction(PassiveRagdollEntry& victim, const RagdollCore::Blow& blow)
    {
        // NET-01: the puppet's parts are kinematic and replay the remote body;
        // that body takes its reactions in its own game.
        if (isNetPuppetEntry(victim))
            return 0.0f;
        const bool canKnockDown = RagdollCore::regionCanKnockDown(blow.mRegion);
        // A physical contact is a hit like any other: its region-scaled
        // reaction impulse goes to the victim's banding so it flinches,
        // staggers or falls by where it was struck, and the kinematic root is
        // pushed a bounded few units so the reaction reads on screen. Every
        // blow, also the ones that leave a wound mark: only unmarked blows
        // shoved the body, so the two profiled blades never did, and since
        // every weapon marks (2026-10-02) no blow would have (user: "hits
        // landed should push the enemy more"). The weapon-class share
        // (HIT_PUSH) still keeps swords from throwing people.
        // OPENMW_RAGDOLL_HIT_REACT_ALL=0 = unmarked only.
        static const auto reactAll_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HIT_REACT_ALL");
            return !(text && std::atoi(text) == 0);
        });
        static const auto& reactAll = reactAll_tuned.get();
        // The struck part takes the blow's real momentum (carried mass x
        // approach speed), so a head snaps and a torso folds - the 1 kg
        // weapon body itself carried almost none of it.
        // V2 (RXN-02 M4/M5): the blow's real momentum with a modest scale,
        // the struck part's speed change capped where a body part can go
        // (a straight punch moves the head ~4 m/s): the give, the snap and
        // the reflex make it read, not a x4 shove that folds the spine.
        // OPENMW_RAGDOLL_REACTION_V2_PUSH = "scale, part speed cap u/s".
        static const auto pushV2_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
            std::array<float, 2> value{ 1.5f, 560.0f };
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_REACTION_V2_PUSH"))
                std::sscanf(text, "%f,%f", &value[0], &value[1]);
            // Visibility (REACTION_V2_VIS) pushes harder too.
            value[0] *= PassiveRagdollEntry::reactionVisibility();
            value[1] *= PassiveRagdollEntry::reactionVisibility();
            return value;
        });
        static const auto& pushV2 = pushV2_tuned.get();
        const auto& push = pushV2;
        const auto& masses = victim.mRagdoll->getBodyMasses();
        const bool pushable = push[0] > 0.0f && blow.mBodyIndex < masses.size();
        // STEP-01 S1: the struck part's push is the only one, x the scale.
        const auto& step01 = PassiveRagdollEntry::step01Settings();
        const float stepScale = step01.mOneShove ? std::max(0.0f, step01.mPushScale) : 1.0f;
        const float pushed = pushable
            ? std::max(0.0f, stepScale * std::min(push[0] * blow.momentum(), masses[blow.mBodyIndex] * push[1])) : 0.0f;
        float bodyMass = 0.0f;
        for (std::size_t i = 0; i < masses.size(); ++i)
            if (!victim.isDetachedWeaponBody(i))
                bodyMass += masses[i];
        // The balance bands read the blow: priced from its damage (the
        // reaction impulse), or under V2 from what it did to the body's
        // momentum - its speed change (u/s) x K, so the bands follow the
        // physics (K 22: the stagger band at 0.8 m/s, the controller's
        // stagger tier). OPENMW_RAGDOLL_REACTION_V2_BALANCE = K (0 = damage).
        static const auto balanceK_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_REACTION_V2_BALANCE");
            return text ? std::max(0.0f, static_cast<float>(std::atof(text))) : 0.0f;
        });
        static const auto& balanceK = balanceK_tuned.get();
        const float bandImpulse = balanceK > 0.0f && bodyMass > 0.0f
            ? pushed / bodyMass * balanceK : blow.mReactionImpulse;
        // A physical contact is a hit like any other: its reaction impulse
        // goes to the victim's banding so it flinches, staggers or falls by
        // where it was struck, and the kinematic root is pushed a bounded few
        // units so the reaction reads on screen. Every blow, also the ones
        // that leave a wound mark: only unmarked blows shoved the body, so
        // the two profiled blades never did, and since every weapon marks
        // (2026-10-02) no blow would have (user: "hits landed should push the
        // enemy more"). The weapon-class share (HIT_PUSH) still keeps swords
        // from throwing people. OPENMW_RAGDOLL_HIT_REACT_ALL=0 = unmarked only.
        if (!blow.mLocalizedWound || reactAll)
        {
            victim.registerBlowV2(bandImpulse, canKnockDown,
                balanceK > 0.0f && bodyMass > 0.0f ? pushed / bodyMass : -1.0f);
            victim.noteKnockingBlow(blow.mDirection, canKnockDown);
            if (!blow.mBlocked)
                victim.beginDaze(blow.mRegion, blow.mReactionImpulse);
            // A receipted blow adds its limb damage on receipt.
            if (!blow.mBlocked && !blow.mReceipted)
                victim.addRegionDamage(blow.mRegion, blow.mDamage);
            victim.addKinematicRootReactionKick(blow.mAttackerToVictim, bandImpulse);
        }
        if (!(pushed > 0.0f))
            return 0.0f;
        // The push follows the weapon's travel, but not steeply up: a rising
        // cut pushed the struck part upward (up to 1360 u/s with the play
        // visibility) and lifted the whole body off its feet (user 10-29:
        // "some hits make the enemies just bounce up"). The upward part is
        // capped at `lift` (the sine of the steepest angle, 0.25 = 14 deg) and
        // the rest goes along the ground; straight up, along the line from the
        // attacker. Down is untouched. OPENMW_RAGDOLL_HIT_LIFT (1 = off).
        static const auto hitLift_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HIT_LIFT");
            return text ? std::clamp(static_cast<float>(std::atof(text)), 0.0f, 1.0f) : 0.25f;
        });
        JPH::Vec3 normal = blow.mDirection;
        if (const float lift = hitLift_tuned.get(); normal.GetZ() > lift)
        {
            JPH::Vec3 flat(normal.GetX(), normal.GetY(), 0.0f);
            if (flat.Length() < 0.3f)
                flat = JPH::Vec3(blow.mAttackerToVictim.GetX(), blow.mAttackerToVictim.GetY(), 0.0f);
            flat = flat.NormalizedOr(JPH::Vec3::sZero());
            if (!flat.IsNearZero())
                normal = flat * std::sqrt(1.0f - lift * lift) + JPH::Vec3(0.0f, 0.0f, lift);
        }
        JPH::BodyInterface& bodies = mPhysicsSystem.GetBodyInterface();
        const JPH::BodyID struck = victim.mRagdoll->getBodyId(blow.mBodyIndex);
        const JPH::RVec3 centre = bodies.GetCenterOfMassPosition(struck);
        JPH::Vec3 lever(blow.mPoint - centre);
        // Hit reactions (stage 7): a longer lever on the chest and shoulders
        // turns the trunk off a blow off its centre line.
        const auto& react = PassiveRagdollEntry::hitReactSettings();
        const bool trunk = blow.mRegion == RagdollCore::AnatomicalRegion::Chest
            || blow.mRegion == RagdollCore::AnatomicalRegion::LeftClavicle
            || blow.mRegion == RagdollCore::AnatomicalRegion::RightClavicle;
        const float leverCap = trunk ? std::max(10.0f, react[1]) : 10.0f;
        if (lever.Length() > leverCap)
            lever *= leverCap / lever.Length();
        bodies.AddImpulse(struck, normal * pushed, centre + lever);
        {
            // V2: the reaction controller reads what the push did to the
            // body's momentum; it gives the struck chain, snaps, reflexes
            // and interrupts (the head snap, the give and the interrupt
            // below are the old path's).
            JPH::RVec3 weighted = JPH::RVec3::sZero();
            for (std::size_t i = 0; i < masses.size(); ++i)
                if (!victim.isDetachedWeaponBody(i))
                    weighted += bodies.GetCenterOfMassPosition(victim.mRagdoll->getBodyId(i)) * masses[i];
            const JPH::RVec3 com = bodyMass > 0.0f ? weighted / bodyMass : centre;
            const auto reaction = victim.onBlowV2(blow, normal * pushed, com, bodyMass, masses[blow.mBodyIndex]);
            victim.rollVanillaKnockdown(blow, reaction.mTier);
            // A blow on the weapon arm knocks the weapon off line (WEAPON_KNOCK).
            const bool weaponArm = blow.mRegion == RagdollCore::AnatomicalRegion::RightHand
                || blow.mRegion == RagdollCore::AnatomicalRegion::RightForearm
                || blow.mRegion == RagdollCore::AnatomicalRegion::RightUpperArm;
            if (weaponArm && !blow.mBlocked && victim.mWeapon && victim.mWeapon->mAttached)
            {
                const float strength = pushed / std::max(1.0f, masses[blow.mBodyIndex] * push[1]);
                victim.knockWeapon(strength);
                if (PassiveRagdollEntry::weaponKnockSettings()[0] > 0.0f
                    && victim.mWeapon->mWeaponBody < victim.mRagdoll->getBodyCount())
                    bodies.AddImpulse(victim.mRagdoll->getBodyId(victim.mWeapon->mWeaponBody), normal * (0.5f * pushed));
            }
        }
        // An NPC hit hard in the head covers up.
        if (react[4] > 0.0f && blow.mRegion == RagdollCore::AnatomicalRegion::Head
            && victim.getPtr() != MWMechanics::getPlayer() && blow.mApproachSpeed >= react[4] && victim.mHscActive
            && victim.mAttackStage != PassiveRagdollEntry::AttackStage::Wind
            && victim.mAttackStage != PassiveRagdollEntry::AttackStage::Cut && !victim.mGuardActive)
        {
            victim.mGuardActive = true;
            victim.mGuardStartPhysicsStep = mJoltStepSerial;
            victim.mAiGuardLowerIn = 0.6f;
            victim.mAiGuardRaiseIn = -1.0f;
            victim.mCoverTime = 0.6f;
        }
        return pushed;
    }

    void PhysicsSystem::recordKillingBlow(
        const MWWorld::Ptr& ptr, const osg::Vec3f& worldPoint, const osg::Vec3f& direction, float healthDamage)
    {
        if (!worldPoint.valid() || !direction.valid() || direction.length2() <= 1e-6f || !std::isfinite(healthDamage)
            || healthDamage <= 0.0f)
            return;
        mPendingRagdollImpacts.insert_or_assign(ptr.mRef, PendingRagdollImpact{ worldPoint, direction, healthDamage });
    }

    bool PhysicsSystem::releaseWeapon(PassiveRagdollEntry& entry, PassiveRagdollWeaponReleaseReason reason)
    {
        // Even duplicate/failed primary release must never retain the off-hand.
        entry.clearNativeSupportGrip("primary-release");
        entry.dropMurderGrip(); // GRIP-05: a reversed weapon goes too
        if (!entry.mWeapon)
            return false;
        LiveWeaponAttachment& weapon = *entry.mWeapon;
        ++weapon.mReleaseRequests;
        if (!weapon.mAttached)
        {
            if (!entry.updateGraspCollisionFilters(false, true))
                entry.failPoweredControl(__LINE__);
            ++mWeaponDuplicateReleaseRequests;
            return false;
        }
        const std::optional<bool> released = entry.mRagdoll->releaseConstraint(weapon.mWeaponBody);
        if (!released || !*released)
        {
            entry.failPoweredControl(__LINE__);
            return false;
        }

        weapon.mAttached = false;
        entry.invalidateBalanceSnapshot("weapon-released");
        if (!entry.updateGraspCollisionFilters(false, true))
            entry.failPoweredControl(__LINE__);
        weapon.mReleaseReason = reason;
        ++weapon.mReleaseCount;
        ++mWeaponReleases;
        if (weapon.mCallbackArmed)
        {
            weapon.mCallbackArmed = false;
            ++mWeaponCallbacksDisarmed;
        }
        entry.mRagdoll->setPoweredVelocitySolverSteps(0);
        entry.mWeaponFixtureYaw.reset();
        entry.mAnimationAttack.mPhase = MWMechanics::RagdollAttackPhase::Idle;
        entry.mNativeStrikeHeld = false;
        entry.mClickExtensionRemaining = 0;
        entry.mPunchCycle = {};
        entry.mSwingHandRotation.reset();
        entry.mSwing = {};
        entry.mSwingInput = {};
        entry.mSwingInputActive = false;
        entry.mPlayerStrikeAimDepth.reset();
        entry.mSwingRestOffset.reset();
        entry.mWeaponStrikeFilter.reset();
        entry.mContactDamage.reset();
        entry.mStrikeCooldown.reset();
        if (entry.mSwingMotorsActive && mPassiveRagdollProfile)
        {
            const auto& arm = entry.mSwingArmIsRight ? entry.mRightArm : entry.mLeftArm;
            if (arm)
                setArmSwingMotors(*entry.mRagdoll,
                    entry.mRuntimeProfile ? *entry.mRuntimeProfile : *mPassiveRagdollProfile,
                    *arm, mRagdollWeaponSwing, false);
        }
        entry.mSwingMotorsActive = false;
        if (mPassiveRagdollProfile)
        {
            const auto& profile = entry.mRuntimeProfile ? *entry.mRuntimeProfile : *mPassiveRagdollProfile;
            if (entry.mAnimationAttackMotors && entry.mRightArm)
                setArmSwingMotors(*entry.mRagdoll, profile, *entry.mRightArm, mRagdollWeaponSwing, false);
            if (entry.mWeaponSupportActive && entry.mLeftArm)
            {
                setArmSwingMotors(*entry.mRagdoll, profile, *entry.mLeftArm, mRagdollWeaponSwing, false);
                entry.mLeftShoulderOpen = false;
            }
        }
        entry.mAnimationAttackMotors = false;
        entry.mWeaponSupportActive = false;
        if (entry.mLeftArmTarget && entry.mLeftArmTarget->mWeaponSupport && !entry.mLeftArmTarget->mBrace)
            entry.mLeftArmTarget.reset();
        auto& request = entry.mSwingArmIsRight ? entry.mRightArmTarget : entry.mLeftArmTarget;
        if (request && !request->mBrace)
            request->mReleasing = true;
        return true;
    }

    void PhysicsSystem::erasePassiveRagdoll(
        const MWWorld::LiveCellRefBase* ref, PassiveRagdollWeaponReleaseReason reason)
    {
        const auto found = mPassiveRagdolls.find(ref);
        if (found == mPassiveRagdolls.end())
            return;
        if (found->second->mWeapon)
        {
            releaseWeapon(*found->second, reason);
            ++mWeaponBodiesDestroyed;
        }
        if (const auto recording = mPassiveRagdollReplays.find(ref); recording != mPassiveRagdollReplays.end())
            recording->second->mFailed = true;
        found->second->invalidateBalanceSnapshot("actor-erased");
        mPassiveRagdolls.erase(found);
        ++mPassiveRagdollsRemoved;
    }

    void PhysicsSystem::removePassiveRagdoll(const MWWorld::Ptr& ptr)
    {
        // An explicit unpower also ends a swim suspension (the fallback
        // re-marks the actor right after its own call).
        mSwimSuspended.erase(ptr.mRef);
        mPendingRagdollImpacts.erase(ptr.mRef);
        mPassiveRagdollMotionHistory.erase(ptr.mRef);
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return;

        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        erasePassiveRagdoll(ptr.mRef);
        if (const auto actor = mActors.find(ptr.mRef); actor != mActors.end())
            actor->second->enableCollisionBody(true);
    }

    bool PhysicsSystem::hasPassiveRagdoll(const MWWorld::ConstPtr& ptr) const
    {
        return mPassiveRagdolls.contains(ptr.mRef);
    }

    void PhysicsSystem::updateStuckFallback()
    {
        // A stuck NPC walks on the vanilla capsule (user 10-30 playthrough:
        // "the guard that greets you right after Jiub wasnt able to climb the
        // stairs" - the chargen escort; SHIP-01's powered walker climbs those
        // stairs as fast as a vanilla one, so the stall was not reproduced):
        // a powered NPC whose movement is requested but whose actor stays
        // within `units` of one spot for `seconds` is unpowered, and powered
        // again once it has stood still for 6 s or walked 300 units on.
        // OPENMW_RAGDOLL_NPC_STUCK = "seconds, units" (0 = off).
        static const auto settings_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
            std::array<float, 2> value{ 3.0f, 20.0f };
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_NPC_STUCK"))
                std::sscanf(text, "%f,%f", &value[0], &value[1]);
            return value;
        });
        const auto& settings = settings_tuned.get();
        // Steep ground walks vanilla too (user 11-01 playthrough: the guard
        // after Jiub "walks very weirdly and also cant get on the ramp", the
        // prison ship's 40 deg ramp; RAMP-01): an AI-walked body's swing foot
        // lifts 2 u at most (the player's 9), scuffs into a ramp's lip and the
        // body braces there; the feet leash then holds the capsule, so NPC_STUCK
        // fires late or, while the AI shuffles aside, never (powered guard 2 of
        // 3 stalled, deck 10.8-35.6 s; vanilla 8.3 s). A powered NPC out of
        // combat whose ground under the capsule - or, walking, `ahead` units
        // ahead - is steeper than `degrees` is unpowered before it stalls, and
        // powered again (as NPC_STUCK: 300 units on or 6 s still) only off
        // the steep ground. Ordinary hills (Balmora's ~30 deg) stay physical.
        // OPENMW_RAGDOLL_NPC_STEEP = "degrees, ahead u" (0 = off).
        static const auto steep_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
            std::array<float, 2> value{ 0.0f, 35.0f };
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_NPC_STEEP"))
                std::sscanf(text, "%f,%f", &value[0], &value[1]);
            return value;
        });
        const auto& steep = steep_tuned.get();
        if (!(settings[0] > 0.0f) && !(steep[0] > 0.0f))
            return;
        const float steepUp = std::cos(osg::DegreesToRadians(std::clamp(steep[0], 0.0f, 89.0f)));
        const auto steepAt = [&](const MWWorld::Ptr& ptr, const osg::Vec3f& at) {
            const auto hit = castRay(at + osg::Vec3f(0.0f, 0.0f, 40.0f), at - osg::Vec3f(0.0f, 0.0f, 80.0f), { ptr }, {},
                MWPhysics::Layers::WORLD | MWPhysics::Layers::HEIGHTMAP);
            return hit.mHit && hit.mHitNormal.valid() && hit.mHitNormal.z() < steepUp;
        };
        const double now
            = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        const MWWorld::Ptr player = MWMechanics::getPlayer();
        std::vector<std::pair<MWWorld::Ptr, bool>> stuck; // the actor, let go by NPC_STEEP
        for (const auto& [ref, entry] : mPassiveRagdolls)
        {
            if (!entry->mPowered)
                continue;
            const MWWorld::Ptr ptr = entry->getPtr();
            if (ptr.isEmpty() || ptr == player || !ptr.getClass().isActor()
                || ptr.getClass().getCreatureStats(ptr).isDead()
                // A fighter pressing into its foe is not stuck.
                || ptr.getClass().getCreatureStats(ptr).getAiSequence().isInCombat())
            {
                mStuckWatch.erase(ref);
                continue;
            }
            // Its own walking request (the movement settings are cleared
            // by the character controller before this reads them).
            const osg::Vec3f request(
                entry->mRawTargetHorizontalVelocity.GetX(), entry->mRawTargetHorizontalVelocity.GetY(), 0.0f);
            if (steep[0] > 0.0f)
            {
                const osg::Vec3f position = ptr.getRefData().getPosition().asVec3();
                if (steepAt(ptr, position)
                    || (request.length() >= 20.0f && steepAt(ptr, position + request * (steep[1] / request.length()))))
                {
                    Log(Debug::Info) << "RAGDOLL_NPC_STEEP actor=" << ptr.getCellRef().getRefId() << " vanilla=1";
                    stuck.emplace_back(ptr, true);
                    continue;
                }
            }
            if (!(settings[0] > 0.0f) || request.length() < 20.0f)
            {
                mStuckWatch.erase(ref);
                continue;
            }
            const osg::Vec3f position = ptr.getRefData().getPosition().asVec3();
            auto found = mStuckWatch.find(ref);
            if (found == mStuckWatch.end() || (position - found->second.mAnchor).length() > settings[1])
            {
                mStuckWatch[ref] = { position, now };
                continue;
            }
            if (now - found->second.mSince >= settings[0])
                stuck.emplace_back(ptr, false);
        }
        for (const auto& [ptr, steepLetGo] : stuck)
        {
            Log(Debug::Info) << "RAGDOLL_NPC_STUCK actor=" << ptr.getCellRef().getRefId() << " vanilla=1";
            mStuckWatch.erase(ptr.mRef);
            removePassiveRagdoll(ptr);
            mStuckSuspended[ptr.mRef] = { ptr.getRefData().getPosition().asVec3(), -1.0,
                ptr.getRefData().getPosition().asVec3(), steepLetGo };
            if (const auto actor = mActors.find(ptr.mRef); actor != mActors.end())
                actor->second->enableCollisionBody(true);
        }
        for (auto it = mStuckSuspended.begin(); it != mStuckSuspended.end();)
        {
            const auto actor = mActors.find(it->first);
            if (actor == mActors.end())
            {
                it = mStuckSuspended.erase(it);
                continue;
            }
            const MWWorld::Ptr ptr = actor->second->getPtr();
            if (ptr.isEmpty() || ptr.getClass().getCreatureStats(ptr).isDead())
            {
                it = mStuckSuspended.erase(it);
                continue;
            }
            // Standing still: the actor has not moved 5 units since mSince
            // (the last place is kept in mLast).
            const osg::Vec3f position = ptr.getRefData().getPosition().asVec3();
            if (it->second.mSince < 0.0 || (position - it->second.mLast).length() > 5.0f)
            {
                it->second.mLast = position;
                it->second.mSince = now;
            }
            const bool walkedOn = (position - it->second.mAnchor).length() >= 300.0f;
            const bool settled = now - it->second.mSince >= 6.0;
            // NPC_STEEP: never powered again on the steep ground itself, and
            // only standing still (2 s): powered mid-stride where the ramp
            // ends, the guard fell at once (RAMP-01, 1 of 2).
            if (steep[0] > 0.0f && (steepAt(ptr, position) || (it->second.mSteep && now - it->second.mSince < 2.0)))
            {
                ++it;
                continue;
            }
            const MWRender::Animation* animation = MWBase::Environment::get().getWorld()->getAnimation(ptr);
            if ((walkedOn || settled) && animation != nullptr && animation->getSkeleton() != nullptr
                && addPoweredRagdoll(ptr, *animation->getSkeleton()))
            {
                Log(Debug::Info) << "RAGDOLL_NPC_STUCK actor=" << ptr.getCellRef().getRefId() << " powered=1";
                it = mStuckSuspended.erase(it);
                continue;
            }
            ++it;
        }
    }

    void PhysicsSystem::updateSwimFallback()
    {
        // Swimming falls back to vanilla (Alpha 1.0; SWIM-01: in water the
        // powered body sat in falling/bracing, fighting the water). A powered
        // actor that swims is unpowered and remembered; back on dry ground
        // for half a second it is powered again. OPENMW_RAGDOLL_SWIM_FALLBACK=0
        // keeps the body powered in water.
        static const auto enabled_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SWIM_FALLBACK");
            return !(text && std::atoi(text) == 0);
        });
        static const auto& enabled = enabled_tuned.get();
        MWBase::World* world = MWBase::Environment::get().getWorld();
        std::vector<MWWorld::Ptr> suspend;
        for (const auto& [ref, entry] : mPassiveRagdolls)
        {
            if (!enabled)
                break;
            if (!entry->mPowered)
                continue;
            const MWWorld::Ptr ptr = entry->getPtr();
            if (!ptr.isEmpty() && ptr.getClass().isActor() && world->isSwimming(ptr))
                suspend.push_back(ptr);
        }
        for (const MWWorld::Ptr& ptr : suspend)
        {
            Log(Debug::Info) << "RAGDOLL_SWIM_SUSPEND actor=" << ptr.getCellRef().getRefId();
            removePassiveRagdoll(ptr);
            mSwimSuspended[ptr.mRef] = -1.0;
        }
        // Called once per frame, so dry time is wall-clock seconds.
        const double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        for (auto it = mSwimSuspended.begin(); it != mSwimSuspended.end();)
        {
            const auto actor = mActors.find(it->first);
            if (actor == mActors.end())
            {
                it = mSwimSuspended.erase(it);
                continue;
            }
            const MWWorld::Ptr ptr = actor->second->getPtr();
            if (ptr.isEmpty() || ptr.getClass().getCreatureStats(ptr).isDead())
            {
                it = mSwimSuspended.erase(it);
                continue;
            }
            const bool dry = !world->isSwimming(ptr) && world->isOnGround(ptr);
            if (!dry)
                it->second = -1.0;
            else if (it->second < 0.0)
                it->second = now;
            const MWRender::Animation* animation = world->getAnimation(ptr);
            if (it->second >= 0.0 && now - it->second >= 0.5 && animation != nullptr && animation->getSkeleton() != nullptr
                && addPoweredRagdoll(ptr, *animation->getSkeleton()))
            {
                Log(Debug::Info) << "RAGDOLL_SWIM_RESUME actor=" << ptr.getCellRef().getRefId();
                it = mSwimSuspended.erase(it);
                continue;
            }
            ++it;
        }
    }

    bool PhysicsSystem::playerOnChargenShip()
    {
        // The player's body and the NPC bodies wait until the player is off
        // the prison ship (user 11-01: "turn on the ragdoll bodies until after
        // the player leaves the boat"; the hold's 40 deg ramp and the chargen
        // escort, RAMP-01). On the ship = character creation running and the
        // player in its hold, or standing on the ship, its plank or the
        // chargen props on the deck (the exterior's "chargen boat",
        // "chargen_plank", "CharGen..." references). Airborne keeps the last
        // answer. Bodies already powered stay. OPENMW_RAGDOLL_BOAT_START=0 = off.
        static const auto enabled_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_BOAT_START");
            return !(text && std::atoi(text) == 0);
        });
        if (!enabled_tuned.get())
            return false;
        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Ptr player = MWMechanics::getPlayer();
        if (player.isEmpty() || player.getCell() == nullptr
            || world->getGlobalFloat(MWWorld::Globals::sCharGenState) == -1)
            return mOnChargenShip = false;
        if (!player.getCell()->isExterior())
            return mOnChargenShip
                = Misc::StringUtils::ciEqual(player.getCell()->getCell()->getNameId(), "Imperial Prison Ship");
        const auto actor = mActors.find(player.mRef);
        if (actor == mActors.end() || !actor->second->getOnGround())
            return mOnChargenShip;
        const MWWorld::Ptr standingOn = actor->second->getStandingOnPtr();
        mOnChargenShip = !standingOn.isEmpty()
            && (standingOn.getCellRef().getRefId().startsWith("chargen")
                || standingOn.getCellRef().getRefId().startsWith("ex_de_ship"));
        return mOnChargenShip;
    }

    void PhysicsSystem::updatePoweredRagdollPopulation()
    {
        updateSwimFallback();
        updateStuckFallback();
        const bool onShip = playerOnChargenShip();
        {
            static bool logged = false;
            if (onShip != logged)
                Log(Debug::Info) << "RAGDOLL_BOAT_START waiting=" << onShip;
            logged = onShip;
        }
        // The player's physical body without the development scripts (Beta
        // package): OPENMW_RAGDOLL_POWER_PLAYER=1 powers a living, dry player
        // whenever it is not powered (load, death and resurrection included;
        // swimming has its own fallback). Tried twice a second.
        static const auto powerPlayer_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_POWER_PLAYER");
            return text && std::atoi(text) != 0;
        });
        static const auto& powerPlayer = powerPlayer_tuned.get();
        if (powerPlayer && !onShip && getPassiveRagdollProfile())
        {
            static std::uint32_t playerTries = 0;
            const MWWorld::Ptr self = MWMechanics::getPlayer();
            if (!self.isEmpty() && ++playerTries % 30 == 0 && !mPassiveRagdolls.contains(self.mRef)
                && !mSwimSuspended.contains(self.mRef) && !self.getClass().getCreatureStats(self).isDead()
                && !MWBase::Environment::get().getWorld()->isSwimming(self))
            {
                const MWRender::Animation* animation = MWBase::Environment::get().getWorld()->getAnimation(self);
                if (animation != nullptr && animation->getSkeleton() != nullptr
                    && addPoweredRagdoll(self, *animation->getSkeleton()))
                    Log(Debug::Info) << "RAGDOLL_POWER_PLAYER powered";
            }
        }
        // A stranded body (user 10-30 bookmark "I fell down and I cant get
        // up", right after the Smuggler Den door into Ashinabi: the body was
        // rebuilt, lay 40 units under the floor the player stood on, and the
        // player walked 180 units away from it for 20 s). When the player's
        // body has been more than `far` units from the player (flat), or
        // `below` units under its feet, for `seconds`, it is dropped and
        // powered again where the player stands (the door's resume).
        // OPENMW_RAGDOLL_STRANDED = "far, below, seconds" (0 = off).
        static const auto stranded_tuned = RagdollCore::tuned<std::array<float, 3>>([] {
            std::array<float, 3> value{ 150.0f, 60.0f, 2.0f };
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_STRANDED"))
                std::sscanf(text, "%f,%f,%f", &value[0], &value[1], &value[2]);
            return value;
        });
        if (const auto& stranded = stranded_tuned.get(); stranded[0] > 0.0f)
        {
            const MWWorld::Ptr self = MWMechanics::getPlayer();
            const auto found = self.isEmpty() ? mPassiveRagdolls.end() : mPassiveRagdolls.find(self.mRef);
            bool apart = false;
            if (found != mPassiveRagdolls.end() && found->second->mRagdoll
                && found->second->mRagdoll->getBodyCount() > 0 && !self.getClass().getCreatureStats(self).isDead())
            {
                const osg::Vec3f body = Misc::Convert::toOsg(
                    mPhysicsSystem.GetBodyInterface().GetCenterOfMassPosition(found->second->mRagdoll->getBodyId(0)));
                const osg::Vec3f actor = self.getRefData().getPosition().asVec3();
                apart = osg::Vec2f(body.x() - actor.x(), body.y() - actor.y()).length() > stranded[0]
                    || body.z() < actor.z() - stranded[1];
                // Not in a jump's flight: a cell load hitch in an Icarian
                // Flight jump (ICARIAN-01 acro 1000, 5000 u/s) left the body
                // 300-575 units behind for 2 s while it caught up, and the
                // rig was dropped mid-air.
                if (found->second->mJumpCommanded > 0.0f && found->second->mJumpAirborne)
                    apart = false;
            }
            const double now
                = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            if (!apart)
                mStrandedSince = -1.0;
            else if (mStrandedSince < 0.0)
                mStrandedSince = now;
            else if (now - mStrandedSince >= stranded[2])
            {
                Log(Debug::Info) << "RAGDOLL_STRANDED actor=player powered=" << found->second->mPowered
                                 << " repower=1";
                mStrandedSince = -1.0;
                removePassiveRagdoll(self);
                mSwimSuspended[self.mRef] = -1.0;
                if (const auto actor = mActors.find(self.mRef); actor != mActors.end())
                    actor->second->enableCollisionBody(true);
            }
        }
        // BUILD_FRAMES: a body built mid-stride before its skeleton's
        // standing pose was known is rebuilt once the actor has stood still
        // on the ground for a second and that pose is known (through the
        // swim fallback's resume, which powers it again where it stands).
        {
            const double now
                = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            std::vector<MWWorld::Ptr> reframe;
            std::vector<MWWorld::Ptr> failing;
            for (const auto& [ref, entry] : mPassiveRagdolls)
            {
                if (entry->mPowered && !entry->mDying && entry->mLegsFailing)
                {
                    failing.push_back(entry->getPtr());
                    continue;
                }
                if (!entry->mPowered || entry->mDying || entry->mFramesFromStanding || entry->mFrameKey.empty())
                    continue;
                const MWWorld::Ptr ptr = entry->getPtr();
                const auto actor = mActors.find(ref);
                const bool still = !ptr.isEmpty() && ptr.getClass().isActor()
                    && !ptr.getClass().getCreatureStats(ptr).isDead() && actor != mActors.end()
                    && actor->second->getOnGround() && actor->second->getQueuedVelocity().length2() < 1.0f
                    && mStandingFrameSkeletons.contains(entry->mFrameKey);
                if (!still)
                    entry->mReframeSince = -1.0;
                else if (entry->mReframeSince < 0.0)
                    entry->mReframeSince = now;
                else if (now - entry->mReframeSince >= 1.0)
                    reframe.push_back(ptr);
            }
            for (const MWWorld::Ptr& ptr : reframe)
            {
                Log(Debug::Info) << "RAGDOLL_BUILD_FRAMES rebuild actor=" << ptr.getCellRef().getRefId();
                removePassiveRagdoll(ptr);
                mSwimSuspended[ptr.mRef] = -1.0;
                if (const auto actor = mActors.find(ptr.mRef); actor != mActors.end())
                    actor->second->enableCollisionBody(true);
            }
            // LEGS_HEALTH (PassiveRagdollEntry::checkLegsHealth): rebuilt where
            // it stands; failing again within 20 s, 3 s on the default
            // animation first (the swim resume waits until `now` + 2.5).
            for (const MWWorld::Ptr& ptr : failing)
            {
                if (ptr.isEmpty())
                    continue;
                const auto last = mLegsHealthRebuilt.find(ptr.mRef);
                const bool again = last != mLegsHealthRebuilt.end() && now - last->second < 20.0;
                Log(Debug::Info) << "RAGDOLL_LEGS_HEALTH rebuild actor=" << ptr.getCellRef().getRefId()
                                 << " fallback=" << again;
                mLegsHealthRebuilt[ptr.mRef] = now;
                removePassiveRagdoll(ptr);
                mSwimSuspended[ptr.mRef] = again ? now + 2.5 : -1.0;
                if (const auto actor = mActors.find(ptr.mRef); actor != mActors.end())
                    actor->second->enableCollisionBody(true);
            }
        }
        static const auto enabled_tuned = RagdollCore::tuned<bool>([] {
            // "0" turns it off (the shipped ragdoll.cfg sets it; the lifecycle
            // tests need living NPCs without a ragdoll).
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_POWER_NPCS");
            return text != nullptr && std::atoi(text) != 0;
        });
        static const auto& enabled = enabled_tuned.get();
        // Loads the profile on first use: without the development scripts
        // nothing else did, and no NPC was ever powered (Beta package smoke).
        if (!enabled || onShip || !getPassiveRagdollProfile())
            return;
        const MWWorld::Ptr player = MWMechanics::getPlayer();
        if (player.isEmpty())
            return;
        const osg::Vec3f playerPosition = player.getRefData().getPosition().asVec3();
        // Hysteresis: acquire close, release farther out, so an actor pacing
        // the boundary does not flip between the two states every frame. The
        // budget bounds the solver cost until the population policy is tuned.
        constexpr float acquireRadius = 2000.0f;
        constexpr float releaseRadius = 3500.0f;
        constexpr std::size_t maximumPowered = 10;
        std::vector<MWWorld::Ptr> release;
        std::size_t poweredCount = 0;
        for (const auto& [ref, entry] : mPassiveRagdolls)
        {
            if (!entry->mPowered)
                continue;
            ++poweredCount;
            const MWWorld::Ptr& ptr = entry->getPtr();
            if (ptr.isEmpty() || ptr == player)
                continue;
            const bool dead
                = !ptr.getClass().isActor() || ptr.getClass().getCreatureStats(ptr).isDead();
            const float distance = (ptr.getRefData().getPosition().asVec3() - playerPosition).length();
            if (dead || distance > releaseRadius)
                release.push_back(ptr);
        }
        for (const MWWorld::Ptr& ptr : release)
        {
            removePassiveRagdoll(ptr);
            --poweredCount;
        }
        if (poweredCount >= maximumPowered)
            return;
        for (const auto& [ref, actor] : mActors)
        {
            if (poweredCount >= maximumPowered)
                break;
            const MWWorld::Ptr ptr = actor->getPtr();
            if (ptr.isEmpty() || ptr == player || !PassiveRagdollEntry::kineticHumanoid(ptr))
                continue;
            if (mPassiveRagdolls.contains(ptr.mRef) || mSwimSuspended.contains(ptr.mRef)
                || mStuckSuspended.contains(ptr.mRef)
                || ptr.getClass().getCreatureStats(ptr).isDead()
                || MWBase::Environment::get().getWorld()->isSwimming(ptr))
                continue;
            const float distance = (ptr.getRefData().getPosition().asVec3() - playerPosition).length();
            if (distance > acquireRadius)
                continue;
            const MWRender::Animation* animation = MWBase::Environment::get().getWorld()->getAnimation(ptr);
            if (animation == nullptr || animation->getSkeleton() == nullptr)
                continue;
            if (addPoweredRagdoll(ptr, *animation->getSkeleton()))
                ++poweredCount;
        }
    }

    void PhysicsSystem::updateAutoDuel()
    {
        static const auto enabled_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_AUTO_DUEL") != nullptr; });
        static const auto& enabled = enabled_tuned.get();
        if (!enabled)
            return;
        // Dev harness: keep a fight running without console input. Every two
        // seconds the two nearest living powered NPCs that are not already
        // fighting each other are paired up; when one dies the next pair takes
        // over, so the range keeps producing duels to the death.
        static std::uint32_t frameCounter = 0;
        if (++frameCounter % 120 != 0)
            return;
        const MWWorld::Ptr player = MWMechanics::getPlayer();
        if (player.isEmpty())
            return;
        const osg::Vec3f playerPosition = player.getRefData().getPosition().asVec3();
        std::vector<const PassiveRagdollEntry*> fighters;
        for (const auto& [ref, entry] : mPassiveRagdolls)
        {
            const MWWorld::Ptr& ptr = entry->getPtr();
            if (!entry->mPowered || ptr.isEmpty() || ptr == player || !ptr.getClass().isActor()
                || ptr.getClass().getCreatureStats(ptr).isDead())
                continue;
            const float distance = (ptr.getRefData().getPosition().asVec3() - playerPosition).length();
            if (distance > 3000.0f)
                continue;
            fighters.push_back(entry.get());
        }
        if (fighters.size() < 2)
            return;
        std::sort(fighters.begin(), fighters.end(),
            [&](const PassiveRagdollEntry* left, const PassiveRagdollEntry* right) {
                return (left->getPtr().getRefData().getPosition().asVec3() - playerPosition).length2()
                    < (right->getPtr().getRefData().getPosition().asVec3() - playerPosition).length2();
            });
        const MWWorld::Ptr first = fighters[0]->getPtr();
        const MWWorld::Ptr second = fighters[1]->getPtr();
        const MWMechanics::CreatureStats& firstStats = first.getClass().getCreatureStats(first);
        const MWMechanics::CreatureStats& secondStats = second.getClass().getCreatureStats(second);
        // Harness puppets disable their own AI; a duel needs it back or the
        // combat package is added and then never acted on.
        const auto enableAi = [](const MWWorld::Ptr& ptr) {
            if (auto* controls = MWBase::Environment::get().getLuaManager()->getActorControls(ptr))
            {
                controls->mDisableAI = false;
                controls->mChanged = true;
            }
        };
        enableAi(first);
        enableAi(second);
        const bool firstInCombat = firstStats.getAiSequence().isInCombat(second);
        const bool secondInCombat = secondStats.getAiSequence().isInCombat(first);
        if (!firstInCombat)
            MWBase::Environment::get().getMechanicsManager()->startCombat(first, second, nullptr);
        if (!secondInCombat)
            MWBase::Environment::get().getMechanicsManager()->startCombat(second, first, nullptr);
        if (RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_TRACE") != nullptr)
        {
            static unsigned duelTraceRows = 0;
            if (duelTraceRows < 200)
            {
                ++duelTraceRows;
                Log(Debug::Info) << "RAGDOLL_AUTO_DUEL first=" << first.getCellRef().getRefId()
                    << " second=" << second.getCellRef().getRefId()
                    << " alreadyFighting=" << (firstInCombat && secondInCombat);
            }
        }
    }

    void PhysicsSystem::updatePlayerStrikeAim(PassiveRagdollEntry& entry)
    {
        entry.mPlayerStrikeAimPoint.reset();
        static const auto aimAssistEnabled_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_PLAYER_AIM_ASSIST");
            return !(text && std::atoi(text) == 0);
        });
        static const auto& aimAssistEnabled = aimAssistEnabled_tuned.get();
        // Both strike routes need the point: the physical drag while it is
        // held, and the native quick/charged attack from wind-up to return.
        const bool nativeAttack
            = entry.mAnimationAttack.mPhase != MWMechanics::RagdollAttackPhase::Idle;
        const bool dragStrike = entry.mSwingInputActive && entry.mSwingInput.mHeld;
        if (!aimAssistEnabled || (!dragStrike && !nativeAttack)
            || !entry.mCurrentPhysicsPoseCaptured || entry.mAiStrikeActive)
        {
            entry.mPlayerStrikeAimDepth.reset();
            return;
        }
        const auto& arm = entry.mSwingArmIsRight ? entry.mRightArm : entry.mLeftArm;
        if (!arm)
            return;
        const auto shoulder = entry.mRagdoll->getWorldBodyPosition(entry.mCurrentPhysicsPose, arm->mShoulder);
        const auto geometry = entry.mRagdoll->getHingeLegGeometry(arm->mShoulder, arm->mElbow, arm->mHand);
        const auto reach = geometry ? RagdollCore::hingeChainReachRange(*geometry) : std::nullopt;
        const auto worldUnit = entry.mRagdoll->getWorldVector(JPH::Vec3::sAxisX());
        if (!shoulder || !reach || !worldUnit)
            return;
        float strikeRange = reach->second * worldUnit->Length();
        if (entry.mWeapon && entry.mWeapon->mAttached)
            strikeRange += std::max(0.0f, entry.mWeapon->tipDistanceFromHand());
        // Engage a little before the blow can land, so closing in on an
        // opponent already turns the stroke toward it.
        const float assistRange = strikeRange + 40.0f;
        // How far from the crosshair ray a body part may be and still be
        // what the player is looking at, and the cone the fallback accepts.
        constexpr float rayTolerance = 16.0f;
        constexpr float fallbackConeCosine = 0.80f;
        // The facing is where the player is looking now. A native attack does
        // not run the drag path that normally refreshes mSwingForward, so the
        // camera ray is the live source; the stored forward is the fallback.
        JPH::Vec3 facing = entry.mSwingForward;
        {
            JPH::Vec3 flat = entry.mSwingAimRayDirection;
            flat.SetZ(0.0f);
            if (flat.LengthSq() > 1e-6f)
                facing = flat.Normalized();
        }

        std::optional<JPH::RVec3> rayChoice;
        std::string_view rayRegion = "none";
        float rayScore = rayTolerance;
        std::optional<JPH::RVec3> coneChoice;
        float coneScore = fallbackConeCosine;
        const auto consider = [&](JPH::RVec3Arg point, bool torso, std::string_view region) {
            const JPH::Vec3 offset(point - *shoulder);
            const float distance = offset.Length();
            if (distance < 1e-3f || distance > assistRange || offset.Dot(facing) < 0.35f * distance)
                return;
            if (entry.mSwingAimRayOrigin)
            {
                const JPH::Vec3 fromEye(point - *entry.mSwingAimRayOrigin);
                const float along = fromEye.Dot(entry.mSwingAimRayDirection);
                if (along > 0.0f)
                {
                    const float miss = (fromEye - entry.mSwingAimRayDirection * along).Length();
                    if (miss < rayScore)
                    {
                        rayScore = miss;
                        rayChoice = point;
                        rayRegion = region;
                    }
                }
            }
            if (torso)
            {
                JPH::Vec3 flat(offset.GetX(), offset.GetY(), 0.0f);
                const float flatLength = flat.Length();
                const float cosine = flatLength > 1e-3f ? flat.Dot(facing) / flatLength : 0.0f;
                if (cosine > coneScore)
                {
                    coneScore = cosine;
                    coneChoice = point;
                }
            }
        };

        const MWWorld::Ptr self = entry.getPtr();
        for (const auto& [_, other] : mPassiveRagdolls)
        {
            if (other.get() == &entry || !other->mCurrentPhysicsPoseCaptured)
                continue;
            const MWWorld::Ptr actor = other->getPtr();
            if (actor.isEmpty() || !actor.getClass().isActor()
                || actor.getClass().getCreatureStats(actor).isDead())
                continue;
            const std::size_t bodies = std::min(other->mRagdoll->getBodyCount(), other->mBodyRegions.size());
            for (std::size_t body = 0; body < bodies; ++body)
            {
                if (other->mBodyRegions[body] == RagdollCore::AnatomicalRegion::Unknown)
                    continue;
                if (const auto position = other->mRagdoll->getWorldBodyPosition(other->mCurrentPhysicsPose, body))
                    consider(*position, other->mTorsoBodyIndex && body == *other->mTorsoBodyIndex,
                        RagdollCore::regionName(other->mBodyRegions[body]));
            }
        }
        // Actors without a physical body: aim at the middle of the capsule.
        for (const auto& [ref, actor] : mActors)
        {
            if (!actor || mPassiveRagdolls.contains(ref))
                continue;
            const MWWorld::Ptr ptr = actor->getPtr();
            if (ptr.isEmpty() || ptr == self || !ptr.getClass().isActor()
                || ptr.getClass().getCreatureStats(ptr).isDead())
                continue;
            consider(Misc::Convert::toJolt<JPH::RVec3>(actor->getCollisionObjectPosition()), true, "capsule");
        }
        entry.mPlayerStrikeAimPoint = rayChoice;
        if (entry.mWeapon && entry.mWeapon->mAttached && entry.mSwingAimRayOrigin)
        {
            // Acquire depth once per gesture, not a new limb centre each
            // physics step. Moving arms crossing a stationary crosshair used
            // to displace the target by 10-20 units in a single step. Keep the
            // point on the actual eye ray; camera movement still aims it.
            if (!entry.mPlayerStrikeAimDepth)
            {
                const JPH::Vec3 fromEye((rayChoice ? *rayChoice : *shoulder)
                    - *entry.mSwingAimRayOrigin);
                entry.mPlayerStrikeAimDepth = std::max(1.0f,
                    fromEye.Dot(entry.mSwingAimRayDirection) + (rayChoice ? 0.0f : strikeRange));
            }
            entry.mPlayerStrikeAimPoint = *entry.mSwingAimRayOrigin
                + entry.mSwingAimRayDirection * *entry.mPlayerStrikeAimDepth;
        }
        if (!entry.mSwingAimRayOrigin)
            entry.mPlayerStrikeAimPoint = coneChoice;
        else if (!entry.mPlayerStrikeAimPoint && nativeAttack)
        {
            // A quick attack also follows the crosshair in empty space. The
            // old no-target path fell back to the clip's low fixed arc, while
            // the broad torso cone could redirect a deliberate near miss.
            // Place an aim point one strike reach beyond this shoulder's
            // projection onto the actual eye ray; IK still limits the hand.
            const JPH::Vec3 fromEye(*shoulder - *entry.mSwingAimRayOrigin);
            const float distance = std::max(1.0f, fromEye.Dot(entry.mSwingAimRayDirection) + strikeRange);
            entry.mPlayerStrikeAimPoint = *entry.mSwingAimRayOrigin + entry.mSwingAimRayDirection * distance;
        }
        if (RagdollCore::tuningText("OPENMW_RAGDOLL_ARM_TOOL_CAPTURE") && entry.mPlayerStrikeAimPoint)
        {
            static unsigned aimRows = 0;
            if (aimRows++ < 256)
            {
                const auto vector = [](auto point) {
                    std::ostringstream text;
                    text << std::setprecision(9) << point.GetX() << ',' << point.GetY() << ',' << point.GetZ();
                    return text.str();
                };
                Log(Debug::Info) << "ARM_AIM_TARGET step=" << entry.mCurrentContactPhysicsStep
                    << " phase=" << static_cast<unsigned>(entry.mAnimationAttack.mPhase)
                    << " identity=" << entry.mSwingIdentity << " held=" << entry.mSwingInput.mHeld
                    << " region=" << rayRegion << " point=" << vector(*entry.mPlayerStrikeAimPoint)
                    << " shoulder=" << vector(*shoulder) << " ray=" << vector(entry.mSwingAimRayDirection);
            }
        }
    }

    void PhysicsSystem::updateAiGuard(PassiveRagdollEntry& entry, const MWWorld::Ptr& actor)
    {
        static const auto scale_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_AI_GUARD");
            return text ? static_cast<float>(std::atof(text)) : 1.0f;
        });
        static const auto& scale = scale_tuned.get();
        if (scale <= 0.0f)
            return;
        const MWMechanics::CreatureStats& stats = actor.getClass().getCreatureStats(actor);
        const bool able = entry.mWeapon && entry.mWeapon->mAttached && !entry.rangedWeaponDrawn()
            && entry.mCurrentPhysicsPoseCaptured
            && stats.getDrawState() == MWMechanics::DrawState::Weapon && !stats.getAttackingOrSpell()
            && entry.mAnimationAttack.mPhase == MWMechanics::RagdollAttackPhase::Idle && entry.mControllerOutput
            && RagdollCore::isDrivenControllerMode(entry.mControllerOutput->mMode);
        // AI-02 item 6: the guard's zone from the incoming blade (duelread.hpp),
        // read when the guard is decided and, by a skilled defender, again as
        // it rises (a feint fools the rest). Error by guard skill (Block,
        // Agility, Luck, fatigue). OPENMW_RAGDOLL_AI_READ = "error deg at
        // guard skill 0, at 1, re-read above skill" (unset = off: the middle).
        static const auto read_tuned = RagdollCore::tuned<std::array<float, 3>>([] {
            // Unset or "0" = off; missing fields take the start values (review 10-08).
            std::array<float, 3> value{ -1.0f, 10.0f, 0.6f };
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_AI_READ");
            if (text && std::string_view(text) != "0")
            {
                value[0] = 50.0f;
                std::sscanf(text, "%f,%f,%f", &value[0], &value[1], &value[2]);
            }
            return value;
        });
        static const auto& read = read_tuned.get();
        const auto guardSkill = [&] {
            const float block = actor.getClass().getSkill(actor, ESM::Skill::Block);
            const float agility = stats.getAttribute(ESM::Attribute::Agility).getModified();
            const float luck = stats.getAttribute(ESM::Attribute::Luck).getModified();
            return std::clamp((block + agility / 5.0f + luck / 10.0f) * stats.getFatigueTerm() / 100.0f, 0.0f, 1.0f);
        };
        // Returns the incoming blade's arrival (s, -1 = not read or slow).
        const auto readZone = [&](const PassiveRagdollEntry& attacker, std::uint64_t hash, const char* when) -> float {
            if (!(read[0] >= 0.0f) || !attacker.mWeapon || !attacker.mWeapon->mAttached || !entry.mTorsoBodyIndex
                || !attacker.mCurrentPhysicsPoseCaptured)
                return -1.0f;
            const auto chest = entry.mRagdoll->getWorldBodyPosition(entry.mCurrentPhysicsPose, *entry.mTorsoBodyIndex);
            const auto blade
                = attacker.mRagdoll->getWorldBodyPosition(attacker.mCurrentPhysicsPose, attacker.mWeapon->mWeaponBody);
            if (!chest || !blade)
                return -1.0f;
            const float yaw = actor.getRefData().getPosition().rot[2];
            const JPH::Vec3 forward(std::sin(yaw), std::cos(yaw), 0.0f), right(std::cos(yaw), -std::sin(yaw), 0.0f);
            const JPH::Vec3 at(*blade - *chest);
            const JPH::Vec3& v = attacker.mDuelBladeVelocity;
            const float skill = guardSkill();
            RagdollCore::GuardReadInput in;
            in.mX = at.Dot(right);
            in.mY = at.Dot(forward);
            in.mZ = at.GetZ();
            in.mVX = v.Dot(right);
            in.mVY = v.Dot(forward);
            in.mVZ = v.GetZ();
            in.mErrorDegrees = read[0] + (read[1] - read[0]) * skill;
            in.mDraw = static_cast<float>((hash >> 23) % 2001) / 1000.0f - 1.0f;
            float arrival = -1.0f;
            const RagdollCore::GuardRead zone = RagdollCore::readGuardZone(in);
            // The arrival from the tip and its true velocity (rotation
            // included), not the weapon body's centre (review 10-08).
            if (attacker.mDuelTipPoint)
            {
                const JPH::Vec3 tipAt(*attacker.mDuelTipPoint - *chest);
                const JPH::Vec3& tv = attacker.mDuelTipVelocity;
                RagdollCore::GuardReadInput tipIn;
                tipIn.mX = tipAt.Dot(right);
                tipIn.mY = tipAt.Dot(forward);
                tipIn.mZ = tipAt.GetZ();
                tipIn.mVX = tv.Dot(right);
                tipIn.mVY = tv.Dot(forward);
                tipIn.mVZ = tv.GetZ();
                RagdollCore::readGuardZone(tipIn, &arrival);
            }
            constexpr float counts = 400.0f;
            switch (zone)
            {
                case RagdollCore::GuardRead::High: entry.moveGuardZone(0.0f, counts, true); break;
                case RagdollCore::GuardRead::Low: entry.moveGuardZone(0.0f, -counts, true); break;
                case RagdollCore::GuardRead::Left: entry.moveGuardZone(-counts, 0.0f, true); break;
                case RagdollCore::GuardRead::Right: entry.moveGuardZone(counts, 0.0f, true); break;
                default: entry.moveGuardZone(0.0f, 0.0f, true); break;
            }
            static const auto trace_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_AI_TRACE");
                return text && std::string_view(text) != "0";
            });
            static const auto& trace = trace_tuned.get();
            if (trace)
                Log(Debug::Info) << "RAGDOLL_AI2 actor=" << actor.getCellRef().getRefId() << " what=read when=" << when
                                 << " zone=" << RagdollCore::guardReadName(zone)
                                 << " line=" << (attacker.mHscActive ? osg::RadiansToDegrees(attacker.mAttackAngle) : -1.0f)
                                 << " blade=" << in.mX << "," << in.mY << "," << in.mZ << " speed=" << v.Length()
                                 << " skill=" << skill << " err=" << in.mErrorDegrees * in.mDraw;
            return arrival;
        };
        // OPENMW_RAGDOLL_AI_PARRY = "guard skill above, lead s" (unset = off).
        static const auto parry_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
            std::array<float, 2> value{ 2.0f, 0.06f };
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_AI_PARRY");
            if (text && std::string_view(text) != "0")
            {
                value[0] = 0.6f;
                std::sscanf(text, "%f,%f", &value[0], &value[1]);
            }
            return value;
        });
        static const auto& parry = parry_tuned.get();
        const auto lower = [&] {
            entry.mGuardActive = false;
            entry.mAiParryWait = nullptr;
            entry.mAiGuardRaiseIn = entry.mAiGuardLowerIn = -1.0f;
            if (read[0] >= 0.0f)
                entry.moveGuardZone(0.0f, 0.0f, true);
        };
        if (!able)
        {
            entry.mAiGuardHoldFor = 0.0f;
            entry.mAiParryWait = nullptr;
            if (entry.mAiGuardRaiseIn >= 0.0f || entry.mAiGuardLowerIn >= 0.0f)
                lower();
            return;
        }
        // The nearest combat target that is swinging at us.
        std::vector<MWWorld::Ptr> targets;
        stats.getAiSequence().getCombatTargets(targets);
        const osg::Vec3f self = actor.getRefData().getPosition().asVec3();
        const PassiveRagdollEntry* threat = nullptr;
        osg::Vec3f toThreat;
        for (const MWWorld::Ptr& target : targets)
        {
            const auto found = target.isEmpty() ? mPassiveRagdolls.end() : mPassiveRagdolls.find(target.mRef);
            if (found == mPassiveRagdolls.end())
                continue;
            const PassiveRagdollEntry& other = *found->second;
            // Tip-controller strikes too (the player's, and NPC long weapons
            // since alpha 0.4 item 3): they never set the swing or the
            // animation attack, so no NPC raised a guard against them.
            bool swinging = other.mSwing.mSwinging
                || other.mAnimationAttack.mPhase != MWMechanics::RagdollAttackPhase::Idle
                || other.tipStriking();
            // A steady NPC (OPENMW_RAGDOLL_NPC_STEADY) guards against a real
            // attack on the wheel - past half its wind-up, or its cut - not a
            // held button: holding the mouse in the guard raised the NPC's
            // block again and again (build 61 bookmarks 9-10).
            static const auto steadyGuard_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_NPC_STEADY");
                return text && std::atof(text) > 0.0;
            });
            static const auto& steadyGuard = steadyGuard_tuned.get();
            if (steadyGuard && other.mHscActive)
                swinging = other.mAttackStage == PassiveRagdollEntry::AttackStage::Cut
                    || (other.mAttackStage == PassiveRagdollEntry::AttackStage::Wind && other.mAttackWind >= 0.5f);
            const osg::Vec3f offset = target.getRefData().getPosition().asVec3() - self;
            if (swinging && offset.length() < 170.0f)
            {
                threat = &other;
                toThreat = offset;
                break;
            }
        }
        // Learning (user 10-05, build 57: "after repeated hits of one kind,
        // guard that side, block or dodge"; play 10-05/06: 15 guarded of 1227
        // contacts at the duelists' low Block): each blow taken lately adds
        // `hit` to the chance, each lately taken on the incoming swing's wheel
        // line `line` more, up to `cap`; the memory halves every `half` s.
        // OPENMW_RAGDOLL_AI_GUARD_LEARN = "hit, line, cap, half" (0 = off).
        static const auto learn_tuned = RagdollCore::tuned<std::array<float, 4>>([] {
            std::array<float, 4> value{ 0.12f, 0.2f, 0.85f, 6.0f };
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_AI_GUARD_LEARN"))
                std::sscanf(text, "%f,%f,%f,%f", &value[0], &value[1], &value[2], &value[3]);
            return value;
        });
        static const auto& learn = learn_tuned.get();
        {
            const float keep = learn[3] > 0.0f ? std::exp2(-mPhysicsDt / learn[3]) : 0.0f;
            entry.mAiHitsTaken *= keep;
            for (float& value : entry.mAiLineHits)
                value *= keep;
        }
        if (threat && (threat != entry.mAiGuardDecidedAttacker || threat->mSwingIdentity != entry.mAiGuardDecidedSwing))
        {
            // Decide once per enemy swing: chance from the Block skill.
            entry.mAiGuardDecidedAttacker = threat;
            entry.mAiGuardDecidedSwing = threat->mSwingIdentity;
            const float skill = actor.getClass().getSkill(actor, ESM::Skill::Block);
            float learned = learn[0] * entry.mAiHitsTaken;
            if (threat->mHscActive)
            {
                const int line = static_cast<int>(std::lround(threat->mAttackAngle / (osg::PIf * 0.25f))) & 7;
                learned += learn[1] * entry.mAiLineHits[static_cast<std::size_t>(line)];
            }
            const float chance = std::max(std::clamp(scale * skill / 100.0f, 0.0f, 0.9f),
                std::min(learn[2], scale * skill / 100.0f + learned));
            if (RagdollCore::tuningText("OPENMW_RAGDOLL_AI_GUARD_TRACE"))
                Log(Debug::Info) << "RAGDOLL_AI_GUARD actor=" << actor.getCellRef().getRefId() << " chance=" << chance
                                 << " learned=" << learned << " hits=" << entry.mAiHitsTaken;
            const std::uint64_t hash = (entry.mSerial * 0x9E3779B97F4A7C15ull) ^ (threat->mSwingIdentity * 0xBF58476D1CE4E5B9ull);
            const float roll = static_cast<float>((hash >> 11) % 10000) / 10000.0f;
            if (roll < chance && !entry.mGuardActive)
            {
                entry.mAiGuardRaiseIn = 0.12f;
                readZone(*threat, hash, "decide");
                // AI-02 timed parry: a skilled guard holds its raise back from
                // the wind-up to the cut and rises just before the blade
                // arrives (the active parry window is 9 steps), so its block
                // can knock the blow away and open a riposte. Raised 0.12 s
                // after the read, NPC guards never made one (RIPOSTE-01: 0
                // knockaways; deciding in the wind-up the blade has no arrival
                // yet, so the first version never fired).
                if (parry[0] <= 1.0f && guardSkill() > parry[0])
                {
                    entry.mAiGuardRaiseIn = -1.0f;
                    entry.mAiParryWait = threat;
                    entry.mAiParryWaitLeft = 1.2f;
                }
            }
        }
        // The timed parry's wait (AI_PARRY): the cut starts (or the blade
        // flies) -> rise at its arrival less the lead; the swing gone or the
        // wait over -> rise now.
        if (entry.mAiParryWait)
        {
            entry.mAiParryWaitLeft -= mPhysicsDt;
            const PassiveRagdollEntry* attacker = entry.mAiParryWait;
            if (threat != attacker)
                entry.mAiParryWait = nullptr; // the swing is gone: nothing to meet (review 10-08)
            else if (entry.mAiParryWaitLeft <= 0.0f)
            {
                entry.mAiParryWait = nullptr;
                if (!entry.mGuardActive)
                    entry.mAiGuardRaiseIn = 0.0f;
            }
            else if ((attacker->mHscActive && attacker->mAttackStage == PassiveRagdollEntry::AttackStage::Cut)
                || attacker->mDuelTipSpeed > 300.0f)
            {
                // Read each step until the tip gives an arrival, then rise
                // that long less the lead (or at once if it is closer).
                const float arrival
                    = readZone(*attacker, entry.mAiGuardDecidedSwing * 0x94D049BB133111EBull, "parry");
                if (arrival > 0.0f || entry.mAiParryWaitLeft < 0.3f)
                {
                    entry.mAiParryWait = nullptr;
                    entry.mAiGuardRaiseIn = arrival > 0.0f ? std::max(0.0f, arrival - parry[1]) : 0.0f;
                    if (RagdollCore::tuningText("OPENMW_RAGDOLL_AI_TRACE"))
                        Log(Debug::Info) << "RAGDOLL_AI2 actor=" << actor.getCellRef().getRefId()
                                         << " what=parry arrival=" << arrival << " raise=" << entry.mAiGuardRaiseIn;
                }
            }
        }
        // The AI's own hold (a backpedal from a charge): up, and kept up - but
        // never into its own attack on the wheel (review 10-08).
        if (entry.mHscActive && entry.mAttackStage != PassiveRagdollEntry::AttackStage::Guard)
            entry.mAiGuardHoldFor = 0.0f;
        if (entry.mAiGuardHoldFor > 0.0f)
        {
            entry.mAiGuardHoldFor -= mPhysicsDt;
            if (!entry.mGuardActive && entry.mAiGuardRaiseIn < 0.0f)
                entry.mAiGuardRaiseIn = 0.12f;
            else if (entry.mGuardActive)
                entry.mAiGuardLowerIn = std::max(entry.mAiGuardLowerIn, 0.3f);
        }
        if (entry.mAiGuardRaiseIn >= 0.0f)
        {
            entry.mAiGuardRaiseIn -= mPhysicsDt;
            if (entry.mAiGuardRaiseIn < 0.0f && entry.mClashLockout <= 0.0f)
            {
                if (threat && read[0] >= 0.0f && guardSkill() > read[2])
                    readZone(*threat, entry.mAiGuardDecidedSwing * 0x94D049BB133111EBull, "raise");
                JPH::Vec3 forward(toThreat.x(), toThreat.y(), 0.0f);
                if (!threat || forward.LengthSq() <= 1e-6f)
                    forward = JPH::Vec3(std::sin(actor.getRefData().getPosition().rot[2]),
                        std::cos(actor.getRefData().getPosition().rot[2]), 0.0f);
                entry.mGuardForward = forward.Normalized();
                entry.mGuardStartPhysicsStep = mJoltStepSerial;
                entry.mGuardActive = true;
                entry.mAiGuardLowerIn = 0.6f;
            }
        }
        else if (entry.mAiGuardLowerIn >= 0.0f)
        {
            entry.mAiGuardLowerIn -= mPhysicsDt;
            if (entry.mAiGuardLowerIn < 0.0f)
                lower();
        }
    }

    // Active guard (alpha 0.4 item 6): a guarding body on the tip controller
    // turns its blade toward the nearest armed opponent's weapon, so the
    // guard meets the blow instead of holding one fixed high block (GUARD-01:
    // about 2 of 7 claymore strikes met the fixed guard).
    void PhysicsSystem::updateGuardThreats()
    {
        // Off by default: tracking the other blade kept the guard spinning
        // (GUARD-01 weapon spin p50 6.2 vs 2.5 rad/s) and the damage taken
        // rose (226 vs 101 over 3 runs) though fewer blows got through (15
        // vs 20). OPENMW_RAGDOLL_GUARD_TRACK=1 turns it on.
        static const auto active_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_GUARD_TRACK");
            return text && std::atoi(text) != 0;
        });
        static const auto& active = active_tuned.get();
        for (auto& [ref, entryPtr] : mPassiveRagdolls)
        {
            PassiveRagdollEntry& entry = *entryPtr;
            entry.mGuardThreat.reset();
            // An NPC's block holds the line it was raised against (C3,
            // OPENMW_RAGDOLL_FEINT): a feint draws it, the real cut comes past.
            const bool latching = PassiveRagdollEntry::feintSettings()[0] > 0.0f && entry.mHscActive
                && entry.getPtr() != MWMechanics::getPlayer();
            if (!entry.mGuardActive || !latching)
                entry.mGuardThreatLatch.reset();
            if (latching && entry.mGuardActive && entry.mGuardThreatLatch)
            {
                entry.mGuardThreat = entry.mGuardThreatLatch;
                continue;
            }
            // The incoming blade, for the tip controller's guard and the
            // attack path's block.
            // The wheel's block (a blend toward a block turned half toward the
            // blade) always takes it: the spin was the tip controller's.
            if (!(active || entry.mHscActive) || !entry.mGuardActive || !(entry.mTip.mActive || entry.mHscActive) || !entry.mPowered
                || !entry.mCurrentPhysicsPoseCaptured || !entry.mTorsoBodyIndex)
                continue;
            const auto chest = entry.mRagdoll->getWorldBodyPosition(entry.mCurrentPhysicsPose, *entry.mTorsoBodyIndex);
            if (!chest)
                continue;
            float best = 250.0f;
            for (auto& [otherRef, otherPtr] : mPassiveRagdolls)
            {
                const PassiveRagdollEntry& other = *otherPtr;
                if (&other == &entry || !other.mPowered || !other.mWeapon || !other.mWeapon->mAttached
                    || !other.mCurrentPhysicsPoseCaptured)
                    continue;
                const auto blade = other.mRagdoll->getWorldBodyPosition(other.mCurrentPhysicsPose, other.mWeapon->mWeaponBody);
                if (!blade)
                    continue;
                const float distance = static_cast<float>(JPH::Vec3(*blade - *chest).Length());
                if (distance < best)
                {
                    best = distance;
                    entry.mGuardThreat = *blade;
                }
            }
            if (latching && entry.mGuardThreat)
                entry.mGuardThreatLatch = entry.mGuardThreat;
        }
    }

    // Capsule strikes (ALPHA 1.0 gate, CREATURE-01: a rat and a guar took no
    // damage from three cuts each - the blade's contacts only ever counted
    // powered bodies, so creatures and unpowered NPCs could not be hit by the
    // attack wheel at all). In a cut (and its trailing window), the physical
    // blade is swept against the collision capsule of every actor without a
    // powered body; one that it passes through, with the tip faster than
    // `speed` u/s, takes the blow once per cut through the same damage
    // pipeline (Morrowind's band by the blade's speed, armour, the combat
    // script's blood, sound and hit recovery).
    // OPENMW_RAGDOLL_CAPSULE_STRIKE = speed (0 = off).
    void PhysicsSystem::strikeCapsuleActors(PassiveRagdollEntry& entry)
    {
        static const auto minimumSpeed_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CAPSULE_STRIKE");
            return text ? static_cast<float>(std::atof(text)) : 0.0f;
        });
        static const auto& minimumSpeed = minimumSpeed_tuned.get();
        if (!(minimumSpeed > 0.0f) || !entry.mHscActive || !entry.mPowered || !entry.mWeapon
            || !entry.mWeapon->mAttached || !entry.mCurrentPhysicsPoseCaptured || !entry.hscStrikeOpen()
            || entry.mWeapon->mWeaponBody >= entry.mRagdoll->getBodyCount()
            || entry.mWeapon->mHandBody >= entry.mRagdoll->getBodyCount())
            return;
        if (entry.mCapsuleStruckSwing != entry.mSwingIdentity)
        {
            entry.mCapsuleStruckSwing = entry.mSwingIdentity;
            entry.mCapsuleStruck.clear();
        }
        const MWWorld::Ptr attacker = entry.getPtr();
        if (attacker.isEmpty() || !attacker.getClass().hasInventoryStore(attacker))
            return;
        MWWorld::InventoryStore& inventory = attacker.getClass().getInventoryStore(attacker);
        const auto equipped = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
        if (equipped == inventory.end())
            return;
        JPH::BodyInterface& bodies = mPhysicsSystem.GetBodyInterface();
        const JPH::BodyID hand = entry.mRagdoll->getBodyId(entry.mWeapon->mHandBody);
        const JPH::BodyID weapon = entry.mRagdoll->getBodyId(entry.mWeapon->mWeaponBody);
        const JPH::RVec3 handPos = bodies.GetCenterOfMassPosition(hand);
        const JPH::Quat handTurn = bodies.GetRotation(hand);
        const float reach = std::max(10.0f, entry.mWeapon->tipDistanceFromHand());
        const JPH::Vec3 blade = (handTurn * entry.mWeapon->mBladeAxis).NormalizedOr(JPH::Vec3::sAxisY());
        const JPH::RVec3 tip = handPos + JPH::RVec3(blade * reach);
        const float tipSpeed = bodies.GetPointVelocity(weapon, tip).Length();
        if (!(tipSpeed >= minimumSpeed))
            return;
        for (const auto& [ref, actor] : mActors)
        {
            if (!actor)
                continue;
            const auto powered = mPassiveRagdolls.find(ref);
            if (powered != mPassiveRagdolls.end() && powered->second->mPowered)
                continue;
            const MWWorld::Ptr victim = actor->getPtr();
            if (victim.isEmpty() || victim == attacker || !victim.getClass().isActor()
                || victim.getClass().getCreatureStats(victim).isDead())
                continue;
            if (std::find(entry.mCapsuleStruck.begin(), entry.mCapsuleStruck.end(), ref) != entry.mCapsuleStruck.end())
                continue;
            const osg::Vec3f centre = actor->getCollisionObjectPosition();
            const osg::Vec3f half = actor->getHalfExtents();
            const float radius = std::max(half.x(), half.y());
            const float bottom = centre.z() - half.z() + radius, top = centre.z() + half.z() - radius;
            float best = 1e9f;
            JPH::RVec3 at = tip;
            for (int k = 2; k <= 10; ++k)
            {
                const JPH::RVec3 p = handPos + JPH::RVec3(blade * (reach * static_cast<float>(k) / 10.0f));
                const float z = std::clamp(static_cast<float>(p.GetZ()), bottom, std::max(bottom, top));
                const float d = std::hypot(static_cast<float>(p.GetX()) - centre.x(),
                    static_cast<float>(p.GetY()) - centre.y(), static_cast<float>(p.GetZ()) - z);
                if (d < best)
                {
                    best = d;
                    at = p;
                }
            }
            if (best > radius)
                continue;
            entry.mCapsuleStruck.push_back(ref);
            const float speed = bodies.GetPointVelocity(weapon, at).Length();
            const auto damage = MWMechanics::applyPhysicalWeaponStrike(attacker, victim, *equipped,
                Misc::Convert::toOsg(at), speed, 1.0f, RagdollCore::PhysicalDefenseOutcome::Hit);
            Log(Debug::Info) << "RAGDOLL_CAPSULE_STRIKE attacker=" << attacker.getCellRef().getRefId()
                             << " victim=" << victim.getCellRef().getRefId() << " speed=" << speed
                             << " damage=" << (damage ? damage->mFinalDamage : -1.0f);
        }
    }

    // The attack wheel's foe (roadmap 10-05, whiffs): for a body on hand
    // control, the nearest living powered body in front (within 200 units and
    // 60 deg of its facing, nearer and more central first). Its chest is what a
    // committed cut bends toward (stepHalfSwordControl, the cut plane).
    void PhysicsSystem::updateHscFoes()
    {
        for (auto& [ref, entryPtr] : mPassiveRagdolls)
        {
            PassiveRagdollEntry& entry = *entryPtr;
            entry.mHscFoe.reset();
            entry.mHscFoeGuardZone = -1;
            entry.mHscFoeBlade.reset();
            if (!entry.mHscActive || !entry.mPowered || !entry.mCurrentPhysicsPoseCaptured || !entry.mTorsoBodyIndex)
                continue;
            const auto chest = entry.mRagdoll->getWorldBodyPosition(entry.mCurrentPhysicsPose, *entry.mTorsoBodyIndex);
            const auto finite = [](JPH::RVec3Arg v) {
                return std::isfinite(v.GetX()) && std::isfinite(v.GetY()) && std::isfinite(v.GetZ());
            };
            if (!chest || !finite(*chest))
                continue;
            const float yaw = entry.getPtr().getRefData().getPosition().rot[2];
            const JPH::Vec3 facing(std::sin(yaw), std::cos(yaw), 0.0f);
            float best = std::numeric_limits<float>::max();
            for (auto& [otherRef, otherPtr] : mPassiveRagdolls)
            {
                const PassiveRagdollEntry& other = *otherPtr;
                if (&other == &entry || !other.mPowered || !other.mCurrentPhysicsPoseCaptured || !other.mTorsoBodyIndex)
                    continue;
                const MWWorld::Ptr otherPtrRef = other.getPtr();
                if (otherPtrRef.isEmpty() || !otherPtrRef.getClass().isActor()
                    || otherPtrRef.getClass().getCreatureStats(otherPtrRef).isDead())
                    continue;
                // CROWD (user 10-09, groups): an NPC aims, turns and picks its
                // line only at someone it fights - an ally in front of it was
                // its "foe" in a crowd. The player keeps any body in front.
                {
                    const MWWorld::Ptr self = entry.getPtr();
                    if (crowdEnabled() && self != MWMechanics::getPlayer() && self.getClass().isActor()
                        && !self.getClass().getCreatureStats(self).getAiSequence().isInCombat(otherPtrRef))
                        continue;
                }
                const auto otherChest
                    = other.mRagdoll->getWorldBodyPosition(other.mCurrentPhysicsPose, *other.mTorsoBodyIndex);
                // A body being placed (a duelist spawned at a hub reset) can
                // report a non-finite point: never aim at it (a NaN height went
                // through the cut plane into the arm's targets).
                if (!otherChest || !finite(*otherChest))
                    continue;
                JPH::Vec3 flat(*otherChest - *chest);
                flat.SetZ(0.0f);
                const float distance = flat.Length();
                if (distance > 200.0f || distance < 1.0f)
                    continue;
                const float alignment = flat.Dot(facing) / distance;
                if (alignment < 0.5f)
                    continue;
                const float score = distance * (2.0f - alignment);
                if (score < best)
                {
                    best = score;
                    entry.mHscFoe = *otherChest;
                    // AI_LEAD (user 10-09: "the enemies should have a much
                    // better hit rate on the player"): an NPC cuts where its foe
                    // will be when the blade arrives - the foe's ground velocity
                    // times the time left (`wind` seconds before the cut,
                    // `cut` in it). A strafe at a walk moved the target 50-100
                    // units in the 0.4-0.7 s from decision to contact.
                    // OPENMW_RAGDOLL_AI_LEAD = "wind, cut" seconds (0 = off).
                    static const auto aiLead_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
                        std::array<float, 2> value{ 0.0f, 0.0f };
                        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_AI_LEAD"))
                            std::sscanf(text, "%f,%f", &value[0], &value[1]);
                        return value;
                    });
                    static const auto& aiLead = aiLead_tuned.get();
                    if ((aiLead[0] > 0.0f || aiLead[1] > 0.0f) && entry.getPtr() != MWMechanics::getPlayer())
                    {
                        JPH::Vec3 velocity = mPhysicsSystem.GetBodyInterface().GetLinearVelocity(
                            other.mRagdoll->getBodyId(*other.mTorsoBodyIndex));
                        velocity.SetZ(0.0f);
                        if (velocity.Length() > 400.0f)
                            velocity = velocity.Normalized() * 400.0f;
                        const bool cutting = entry.mAttackStage == decltype(entry.mAttackStage)::Cut;
                        entry.mHscFoe = *otherChest + JPH::RVec3(velocity * (cutting ? aiLead[1] : aiLead[0]));
                    }
                    // AI-02 item 4: what the foe guards with - its guard zone
                    // and its blade, from its chest in this body's frame.
                    entry.mHscFoeGuardZone = other.mGuardActive ? static_cast<int>(other.guardZone()) : -1;
                    entry.mHscFoeBlade.reset();
                    if (other.mWeapon && other.mWeapon->mAttached)
                        if (const auto blade
                            = other.mRagdoll->getWorldBodyPosition(other.mCurrentPhysicsPose, other.mWeapon->mWeaponBody))
                        {
                            const JPH::Vec3 rel(*blade - *otherChest);
                            const JPH::Vec3 right(std::cos(yaw), -std::sin(yaw), 0.0f);
                            entry.mHscFoeBlade = JPH::Vec3(rel.Dot(right), rel.Dot(facing), rel.GetZ());
                        }
                }
            }
        }
    }

    // A corpse carries its actor (user 10-31: "on death the npc bodies should
    // be released into a ragdoll, right now they do some movement and drags
    // the ragdoll around"): a corpse is drawn under its actor's scene node,
    // relative to where the actor stood when it died, and the death clip's
    // root motion went on moving the actor, so the drawn body slid with it
    // while its physics bodies lay still. The death watch never saw it: it
    // measures the physics pelvis. Now the actor takes no animation movement
    // (queueObjectMovement) and follows the corpse each step: over the floor
    // under its root body, the reference and poses rebased so the body is
    // drawn where it lies; looting and saves find it there too.
    // OPENMW_RAGDOLL_CORPSE_FOLLOW=0 = the old corpse.
    static bool corpseFollow()
    {
        static const auto follow_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CORPSE_FOLLOW");
            return !(text && std::atoi(text) == 0);
        });
        return follow_tuned.get();
    }

    static void followCorpse(PassiveRagdollEntry& entry, Actor& actor, const JPH::BodyInterface& bodies)
    {
        if (entry.mPowered || !entry.mRagdoll || entry.mRagdoll->getBodyCount() == 0)
            return;
        const JPH::RVec3 root = bodies.GetCenterOfMassPosition(entry.mRagdoll->getBodyId(0));
        const JPH::RVec3 reference = entry.mRagdoll->getReferencePosition();
        // Over the floor under the root body; with no floor in reach (a fall
        // off a ledge) no higher than the root body.
        JPH::RVec3 next(root.GetX(), root.GetY(), std::min(reference.GetZ(), root.GetZ()));
        if (entry.mSurfaceQuery)
            if (const auto floor = entry.mSurfaceQuery(root + JPH::RVec3(0.0, 0.0, 10.0), -JPH::Vec3::sAxisZ(), 200.0f))
                next.SetZ(floor->first.GetZ());
        // Still, and the actor where it was put (leftover inertia moves it).
        const osg::Vec3f nextActor(static_cast<float>(next.GetX()), static_cast<float>(next.GetY()),
            static_cast<float>(next.GetZ()));
        if (JPH::Vec3(next - reference).LengthSq() < 0.25f * 0.25f
            && (actor.getPhysicsPosition() - nextActor).length2() < 0.25f * 0.25f)
            return;
        const JPH::Quat rotation = entry.mExperimentalReferenceRotation;
        RagdollCore::RagdollPose previous = entry.mPreviousPhysicsPose;
        RagdollCore::RagdollPose current = entry.mCurrentPhysicsPose;
        RagdollCore::RagdollPose source = entry.mTransitionSourcePose;
        if ((!previous.mBodyTransforms.empty() && !entry.mRagdoll->rebaseModelPose(previous, next, rotation))
            || (entry.mCurrentPhysicsPoseCaptured && !entry.mRagdoll->rebaseModelPose(current, next, rotation))
            || (!source.mBodyTransforms.empty() && !entry.mRagdoll->rebaseModelPose(source, next, rotation))
            || !entry.mRagdoll->setReferenceTransform(next, rotation))
            return;
        entry.mPreviousPhysicsPose = std::move(previous);
        entry.mCurrentPhysicsPose = std::move(current);
        entry.mTransitionSourcePose = std::move(source);
        actor.setPoweredRagdollPosition(nextActor);
        const osg::Vec3f synchronized = actor.getPhysicsPosition();
        entry.mSynchronizedActorPosition = JPH::RVec3(synchronized.x(), synchronized.y(), synchronized.z());
    }

    void PhysicsSystem::stepDeathDrag(float dt)
    {
        static const auto rate_tuned = RagdollCore::tuned<float>([] {
            std::array<float, 2> value{ 3.0f, 2.5f };
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DEATH_DRAG"))
                std::sscanf(text, "%f,%f", &value[0], &value[1]);
            return value[0];
        });
        const float keep = std::exp(-std::max(0.0f, rate_tuned.get()) * dt);
        JPH::BodyInterface& bodies = mPhysicsSystem.GetBodyInterface();
        for (auto& [_, entry] : mPassiveRagdolls)
        {
            if (entry->mDeathDragSteps <= 0 || entry->mPowered || !entry->mRagdoll)
                continue;
            --entry->mDeathDragSteps;
            const auto& masses = entry->mRagdoll->getBodyMasses();
            JPH::Vec3 momentum = JPH::Vec3::sZero();
            float mass = 0.0f;
            for (std::size_t i = 0; i < entry->mRagdoll->getBodyCount() && i < masses.size(); ++i)
            {
                momentum += bodies.GetLinearVelocity(entry->mRagdoll->getBodyId(i)) * masses[i];
                mass += masses[i];
            }
            if (!(mass > 0.0f))
                continue;
            JPH::Vec3 drain = momentum / mass * (1.0f - keep);
            drain.SetZ(0.0f);
            for (std::size_t i = 0; i < entry->mRagdoll->getBodyCount(); ++i)
            {
                const JPH::BodyID id = entry->mRagdoll->getBodyId(i);
                bodies.SetLinearVelocity(id, bodies.GetLinearVelocity(id) - drain);
            }
        }
    }

    namespace
    {
        // SND-01 stage 1: the swing's whoosh follows the blade (user 11-01:
        // "only add sounds if they fit with the Morrowind sound design"). One
        // Morrowind swish per acceleration of the weapon's tip (the striking
        // hand bare-handed), measured against the body so running carries no
        // whoosh: it starts when the tip passes `on` x the weapon's full-swing
        // speed, follows the tip speed (volume and pitch, full at `top` x it)
        // while it plays and re-arms below `off` x it. SwishS for fists and
        // short blades, SwishL for two-handers, SwishM (Weapon Swish's file)
        // the rest; heavier weapons lower. It replaces the swish at a cut's
        // start and the one Morrowind's attack clip played at its release.
        // OPENMW_RAGDOLL_SWING_VOICE = "on,off,top" shares (unset or 0 = off:
        // the old swishes). Measured 11-01 (CLICK-02, CUT-01, CONNECT-01,
        // FIST-01): blows peak at 2-3x (dagger, fists) to 10-13x (claymore);
        // 1.8,0.7,4 gave one whoosh per blow, none running armed.
        std::array<float, 3> swingVoiceSettings()
        {
            static const auto settings_tuned = RagdollCore::tuned<std::array<float, 3>>([] {
                std::array<float, 3> value{ 0.0f, 0.7f, 4.0f };
                if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SWING_VOICE"))
                    std::sscanf(text, "%f,%f,%f", &value[0], &value[1], &value[2]);
                for (float& v : value)
                    if (!std::isfinite(v))
                        v = 0.0f;
                value[1] = std::clamp(value[1], 0.0f, value[0]);
                value[2] = std::max(value[2], value[0] + 0.1f);
                return value;
            });
            return settings_tuned.get();
        }

        bool swingVoiceEnabled()
        {
            return swingVoiceSettings()[0] > 0.0f;
        }

        void updateSwingVoice(PassiveRagdollEntry& entry, const MWWorld::Ptr& actor, float dt, JPH::BodyInterface& bodies)
        {
            MWBase::SoundManager& sounds = *MWBase::Environment::get().getSoundManager();
            entry.mSwingVoiceAge += dt;
            // The sound object is the sound manager's: touch it only while it plays.
            if (entry.mSwingVoice && (actor.isEmpty() || !sounds.getSoundPlaying(actor, entry.mSwingVoiceId)))
                entry.mSwingVoice = nullptr;
            if (actor.isEmpty() || !actor.getClass().isActor() || !entry.mPowered || !entry.mRagdoll
                || entry.mRagdoll->getBodyCount() == 0 || entry.rangedWeaponDrawn()
                || actor.getClass().getCreatureStats(actor).getDrawState() != MWMechanics::DrawState::Weapon)
            {
                entry.mSwingVoiceArmed = true;
                entry.mSwingVoicePeak = 0.0f;
                return;
            }
            MWWorld::Ptr weapon;
            if (actor.getClass().hasInventoryStore(actor))
            {
                MWWorld::InventoryStore& inventory = actor.getClass().getInventoryStore(actor);
                const auto equipped = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                if (equipped != inventory.end() && equipped->getType() == ESM::Weapon::sRecordId)
                    weapon = *equipped;
            }
            const float weight = weapon.isEmpty() ? 0.0f : weapon.getClass().getWeight(weapon);
            const float full = RagdollCore::swingQualityProfile(weight).mFullSpeed;
            // The tip against the body: the blade's travel less the root's,
            // plus its spin (trackDuelMotion's tip speed less the blade's).
            const float travel = entry.mDuelBladeVelocity.Length();
            const JPH::Vec3 root = bodies.GetLinearVelocity(entry.mRagdoll->getBodyId(0));
            const float speed = (entry.mDuelBladeVelocity - root).Length() + std::max(0.0f, entry.mDuelTipSpeed - travel);
            const auto settings = swingVoiceSettings();
            const float on = settings[0] * full;
            const float off = settings[1] * full;
            const float top = settings[2] * full;
            const float size = std::clamp((speed - on) / (top - on), 0.0f, 1.0f);
            const float heavy = std::clamp(1.15f - weight / 100.0f, 0.85f, 1.15f);
            const float gain = 0.45f + 0.55f * size;
            const float pitch = heavy * (0.85f + 0.45f * size);
            if (speed > off)
                entry.mSwingVoicePeak = std::max(entry.mSwingVoicePeak, speed);
            // Only a blow whooshes: the wheel's cut, a committed drag or a
            // punch. Wind-ups, returns and guard moves reached 1.3-2.7x full
            // and whooshed as often as the cuts (combat cases, 11-01); cuts
            // peak at 3-13x. A body off the wheel (vanilla attack clips) goes
            // by speed alone. One whoosh per swing: a claymore's long cut
            // wobbled across the threshold and whooshed 2-4 times (CLICK-02
            // claymore 33 whooshes, the longsword 9).
            const bool swung = entry.mHscActive && entry.mSincePunch >= 0.3f
                && entry.mSwingVoiceSwing == entry.mSwingIdentity;
            const bool blow = !swung
                && (!entry.mHscActive || entry.mAttackStage == PassiveRagdollEntry::AttackStage::Cut
                    || entry.mTip.mCommitTimer > 0.0f || entry.mSincePunch < 0.3f);
            if (entry.mSwingVoiceArmed && blow && speed > on && entry.mSwingVoiceAge > 0.12f)
            {
                static const ESM::RefId small = ESM::RefId::stringRefId("SwishS");
                static const ESM::RefId medium = ESM::RefId::stringRefId("SwishM");
                static const ESM::RefId large = ESM::RefId::stringRefId("SwishL");
                const ESM::RefId* id = &medium;
                if (weapon.isEmpty())
                    id = &small;
                else
                {
                    const ESM::RefId& type = weapon.get<ESM::Weapon>()->mBase->mData.mType;
                    if (type == ESM::WeaponType::ShortBladeOneHand)
                        id = &small;
                    else if (const ESM::WeaponType* info = MWMechanics::getWeaponType(type);
                             info && (info->mFlags & ESM::WeaponType::TwoHanded))
                        id = &large;
                }
                entry.mSwingVoice = sounds.playSound3D(actor, *id, gain, pitch);
                entry.mSwingVoiceId = *id;
                entry.mSwingVoiceSwing = entry.mSwingIdentity;
                entry.mSwingVoiceAge = 0.0f;
                entry.mSwingVoiceArmed = false;
            }
            else if (entry.mSwingVoice && entry.mSwingVoiceAge < 0.3f)
            {
                entry.mSwingVoice->setVolume(gain);
                entry.mSwingVoice->setPitch(pitch);
            }
            if (speed < off && entry.mSwingVoicePeak > 0.0f)
            {
                // One line per motion past `off`: its peak, and whether it whooshed.
                Log(Debug::Info) << "RAGDOLL_SWING_VOICE actor=" << actor.getCellRef().getRefId()
                                 << " peak=" << entry.mSwingVoicePeak << " full=" << full << " on=" << on
                                 << " whoosh=" << !entry.mSwingVoiceArmed << " hsc=" << entry.mHscActive;
                entry.mSwingVoicePeak = 0.0f;
                entry.mSwingVoiceArmed = true;
            }
        }
    }

    void PhysicsSystem::updatePoweredAiStrikes()
    {
        updateGuardThreats();
        updateHscFoes();
        const MWWorld::Ptr player = MWMechanics::getPlayer();
        for (auto& [_, entry] : mPassiveRagdolls)
        {
            const MWWorld::Ptr attacker = entry->getPtr();
            // How far a body slid in its first 1.5 s dead (the dying shuffle).
            if (entry->mDeathWatchStart && entry->mRagdoll && entry->mRagdoll->getBodyCount() > 0
                && mJoltStepSerial - entry->mDeathWatchStep >= 90)
            {
                const JPH::RVec3 now
                    = mPhysicsSystem.GetBodyInterface().GetCenterOfMassPosition(entry->mRagdoll->getBodyId(0));
                JPH::Vec3 moved(now - *entry->mDeathWatchStart);
                const float vertical = moved.GetZ();
                moved.SetZ(0.0f);
                // Sunk (10-30 bookmark: "the npc body clipped through the floor
                // on death"): the deepest body centre under the collision
                // surface straight above it (units; negative = above it).
                float under = -1.0e9f;
                if (entry->mSurfaceQuery)
                    for (std::size_t i = 0; i < entry->mRagdoll->getBodyCount(); ++i)
                    {
                        const JPH::RVec3 centre
                            = mPhysicsSystem.GetBodyInterface().GetCenterOfMassPosition(entry->mRagdoll->getBodyId(i));
                        if (const auto hit = entry->mSurfaceQuery(
                                centre + JPH::RVec3(0.0, 0.0, 80.0), -JPH::Vec3::sAxisZ(), 160.0f))
                            under = std::max(under, static_cast<float>(hit->first.GetZ() - centre.GetZ()));
                    }
                Log(Debug::Info) << "RAGDOLL_DEATH_SLIDE actor=" << (attacker.isEmpty() ? std::string("?") : attacker.getCellRef().getRefId().toDebugString())
                                 << " horizontal=" << moved.Length() << " vertical=" << vertical << " under=" << under;
                entry->mDeathWatchStart.reset();
            }
            // The first 4 s dead, drawn: the largest drag of the drawn body off
            // its physics place, and how far the actor moved (CORPSE_FOLLOW).
            if (entry->mDeathWatchActorStart && mJoltStepSerial - entry->mDeathWatchStep >= 240)
            {
                const float actorMoved = attacker.isEmpty() ? 0.0f
                    : (attacker.getRefData().getPosition().asVec3() - *entry->mDeathWatchActorStart).length();
                Log(Debug::Info) << "RAGDOLL_DEATH_DRAWN actor="
                                 << (attacker.isEmpty() ? std::string("?") : attacker.getCellRef().getRefId().toDebugString())
                                 << " drawn_off=" << entry->mDeathDrawnOff << " actor_moved=" << actorMoved
                                 << " follow=" << corpseFollow();
                entry->mDeathWatchActorStart.reset();
            }
            // DMG-04: skill as weapon control, for everyone who swings (the
            // player's tip controller included).
            if (entry->mPowered && !attacker.isEmpty() && attacker.getClass().isActor())
            {
                MWWorld::Ptr weapon;
                if (attacker.getClass().hasInventoryStore(attacker))
                {
                    MWWorld::InventoryStore& inventory = attacker.getClass().getInventoryStore(attacker);
                    const auto equipped = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                    if (equipped != inventory.end() && equipped->getType() == ESM::Weapon::sRecordId)
                        weapon = *equipped;
                }
                entry->mWeaponControl = MWMechanics::physicalWeaponControl(attacker, weapon);
                entry->mControlWanderTime += mPhysicsDt;
            }
            // Fatigue for the swing (alpha 0.4 item 9): the ratio the tip
            // controller scales by, and the plain drags' cost - each half
            // circle of drag OPENMW_RAGDOLL_DRAG_FATIGUE of an attack's
            // fatigue (0 = off; committed strokes and clicks pay a full attack).
            if (!attacker.isEmpty() && attacker.getClass().isActor())
            {
                const auto& fatigue = attacker.getClass().getCreatureStats(attacker).getFatigue();
                entry->mFatigueRatio = fatigue.getModified() > 0.0f
                    ? std::clamp(fatigue.getCurrent() / fatigue.getModified(), 0.0f, 1.0f) : 1.0f;
                static const auto dragCost_tuned = RagdollCore::tuned<float>([] {
                    const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DRAG_FATIGUE");
                    return text ? static_cast<float>(std::atof(text)) : 0.7f;
                });
                static const auto& dragCost = dragCost_tuned.get();
                if (dragCost > 0.0f && entry->mDragTravel >= osg::PIf)
                {
                    MWWorld::Ptr weapon;
                    if (attacker.getClass().hasInventoryStore(attacker))
                    {
                        MWWorld::InventoryStore& inventory = attacker.getClass().getInventoryStore(attacker);
                        const auto equipped = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                        if (equipped != inventory.end() && equipped->getType() == ESM::Weapon::sRecordId)
                            weapon = *equipped;
                    }
                    const float before = attacker.getClass().getCreatureStats(attacker).getFatigue().getCurrent();
                    MWMechanics::applyFatigueLoss(attacker, weapon, dragCost * entry->mDragTravel / osg::PIf);
                    Log(Debug::Info) << "RAGDOLL_DRAG_FATIGUE actor=" << attacker.getCellRef().getRefId()
                                     << " travel=" << entry->mDragTravel << " fatigue=" << before << "->"
                                     << attacker.getClass().getCreatureStats(attacker).getFatigue().getCurrent();
                    entry->mDragTravel = 0.0f;
                }
            }
            // A drag past the grip's side limit turns the player (TIP_EDGE_TURN).
            if (const float overflow = std::exchange(entry->mTipEdgeOverflow, 0.0f);
                overflow != 0.0f && !attacker.isEmpty() && attacker == MWMechanics::getPlayer())
                MWBase::Environment::get().getWorld()->getPlayer().yaw(overflow);
            // A committed swing (tip controller) whooshes like a vanilla attack.
            if (entry->mCommitSwishPlayed != entry->mCommitSwishRequests)
            {
                entry->mCommitSwishPlayed = entry->mCommitSwishRequests;
                if (!attacker.isEmpty() && attacker.getClass().isActor())
                {
                    // The whoosh follows the swing's size (user 2026-09-28): a
                    // bigger, faster stroke is louder and higher.
                    static const ESM::RefId weaponSwish = ESM::RefId::stringRefId("Weapon Swish");
                    const float size = std::clamp(entry->mCommitSwishSize, 0.0f, 1.0f);
                    if (!swingVoiceEnabled()) // SND-01: the swing voice whooshes instead
                        MWBase::Environment::get().getSoundManager()->playSound3D(
                            attacker, weaponSwish, 0.45f + 0.55f * size, 0.85f + 0.45f * size);
                    // A committed stroke is an attack: it costs a full attack's
                    // fatigue, like a click (slow positioning drags cost none).
                    MWWorld::Ptr weapon;
                    if (attacker.getClass().hasInventoryStore(attacker))
                    {
                        MWWorld::InventoryStore& inventory = attacker.getClass().getInventoryStore(attacker);
                        const auto equipped = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                        if (equipped != inventory.end() && equipped->getType() == ESM::Weapon::sRecordId)
                            weapon = *equipped;
                    }
                    // A chain that flowed pays less (HSC_CHAIN_FLOW; 1 otherwise).
                    MWMechanics::applyFatigueLoss(attacker, weapon, std::clamp(entry->mCommitFatigue, 0.2f, 1.0f));
                }
            }
            if (swingVoiceEnabled())
                updateSwingVoice(*entry, attacker, mPhysicsDt, mPhysicsSystem.GetBodyInterface());
            if (!entry->mPowered || attacker.isEmpty() || attacker == player || !attacker.getClass().isActor()
                || !entry->mRightArm || entry->rangedWeaponDrawn())
            {
                entry->mAiStrikeActive = false;
                entry->mAiStrikeAimPoint.reset();
                continue;
            }
            // Scripted puppets (the harness disables their AI) keep their
            // authored attacks: driving a physical swing for them changes how
            // the scripted scenarios balance and is not wanted there.
            if (const auto* controls = MWBase::Environment::get().getLuaManager()->getActorControls(attacker);
                controls && controls->mDisableAI != 0)
            {
                if (!entry->mAiStrikeActive)
                    continue;
                entry->mAiStrikeActive = false;
                entry->mAiStrikeAimPoint.reset();
            }
            const MWMechanics::CreatureStats& stats = attacker.getClass().getCreatureStats(attacker);
            updateAiGuard(*entry, attacker);
            // Vanilla hand speed: how fast the live attack animation moves the
            // hand. The physical arm should be driven at the same speed.
            if (entry->mRightArm && entry->mCurrentPhysicsPoseCaptured)
            {
                if (const auto animHand = entry->mRagdoll->getWorldBodyPosition(
                        entry->mMotorTargetPose, entry->mRightArm->mHand);
                    animHand && entry->mAiHandTraceSeeded && mPhysicsDt > 0.0f)
                {
                    entry->mAiAnimHandSpeed
                        = static_cast<float>((*animHand - entry->mAiLastAnimHand).Length()) / mPhysicsDt;
                    entry->mAiLastAnimHand = *animHand;
                }
                else if (animHand)
                {
                    entry->mAiLastAnimHand = *animHand;
                    entry->mAiHandTraceSeeded = true;
                }
                if (const auto physHand = entry->mRagdoll->getWorldBodyPosition(
                        entry->mCurrentPhysicsPose, entry->mRightArm->mHand))
                {
                    if (entry->mAiHandTraceSeeded && mPhysicsDt > 0.0f)
                        entry->mAiPhysicalHandSpeed
                            = static_cast<float>((*physHand - entry->mAiLastPhysHand).Length()) / mPhysicsDt;
                    entry->mAiLastPhysHand = *physHand;
                    if (const auto animHand = entry->mRagdoll->getWorldBodyPosition(
                            entry->mMotorTargetPose, entry->mRightArm->mHand))
                        entry->mAiHandTrackingError = static_cast<float>((*physHand - *animHand).Length());
                }
                if (entry->mLeftArm)
                {
                    const auto animOffHand = entry->mRagdoll->getWorldBodyPosition(
                        entry->mMotorTargetPose, entry->mLeftArm->mHand);
                    const auto physOffHand = entry->mRagdoll->getWorldBodyPosition(
                        entry->mCurrentPhysicsPose, entry->mLeftArm->mHand);
                    if (animOffHand && mPhysicsDt > 0.0f)
                    {
                        entry->mAiAnimOffHandSpeed = static_cast<float>(
                            (*animOffHand - entry->mAiLastAnimOffHand).Length()) / mPhysicsDt;
                        entry->mAiLastAnimOffHand = *animOffHand;
                    }
                    if (animOffHand && physOffHand)
                        entry->mAiOffHandTrackingError
                            = static_cast<float>((*physOffHand - *animOffHand).Length());
                }
            }
            // The strike window is the whole attack animation. The attack
            // input only covers the wind-up; the hand travels fastest in the
            // release and follow-through, and the arm has to keep its strike
            // authority through those or the punch never happens.
            const bool attackAnimation
                = MWBase::Environment::get().getMechanicsManager()->isAttackAnimationActive(attacker);
            // The NPC's own attack, for the tip controller (OPENMW_RAGDOLL_TIP_NPC):
            // the animation attack state is only published for the player, so
            // NPC tip strikes never launched (alpha 0.4 item 3).
            entry->mAiAttackIntent = stats.getAttackingOrSpell() || attackAnimation;
            std::optional<JPH::Vec3> strikeDirection;
            std::optional<JPH::RVec3> strikeAim;
            [[maybe_unused]] const PassiveRagdollEntry* nearestTarget = nullptr;
            [[maybe_unused]] float nearestTargetDistance = -1.0f;
            if ((stats.getAttackingOrSpell() || attackAnimation) && entry->mCurrentPhysicsPoseCaptured)
            {
                const auto shoulder = entry->mRagdoll->getWorldBodyPosition(
                    entry->mCurrentPhysicsPose, entry->mRightArm->mShoulder);
                if (shoulder)
                {
                    std::vector<MWWorld::Ptr> targets;
                    stats.getAiSequence().getCombatTargets(targets);
                    float bestDistance = std::numeric_limits<float>::max();
                    for (const MWWorld::Ptr& target : targets)
                    {
                        if (target.isEmpty() || target == attacker)
                            continue;
                        // Aim at the torso when it is a powered body, so a
                        // downed opponent is struck from above.
                        JPH::RVec3 aim
                            = Misc::Convert::toJolt<JPH::RVec3>(target.getRefData().getPosition().asVec3());
                        if (const auto foundTarget = mPassiveRagdolls.find(target.mRef);
                            foundTarget != mPassiveRagdolls.end()
                            && foundTarget->second->mTorsoBodyIndex
                            && foundTarget->second->mCurrentPhysicsPoseCaptured)
                        {
                            if (const auto torso = foundTarget->second->mRagdoll->getWorldBodyPosition(
                                    foundTarget->second->mCurrentPhysicsPose,
                                    *foundTarget->second->mTorsoBodyIndex))
                                aim = *torso;
                        }
                        const JPH::Vec3 offset = JPH::Vec3(aim - *shoulder);
                        const float distance = offset.Length();
                        if (distance > 1e-4f && distance < bestDistance)
                        {
                            bestDistance = distance;
                            strikeDirection = offset / distance;
                            strikeAim = aim;
                            const auto nearestEntry = mPassiveRagdolls.find(target.mRef);
                            if (nearestEntry != mPassiveRagdolls.end())
                                nearestTarget = nearestEntry->second.get();
                            nearestTargetDistance = distance;
                        }
                    }
                    if (bestDistance > 150.0f)
                        strikeDirection.reset();
                    if (strikeDirection)
                    {
                        // DMG-04: a less skilled fighter's aim wanders up to
                        // 40 units off the target at no control.
                        const auto [wanderSide, wanderUp] = entry->controlWander();
                        const float wander = (1.0f - std::clamp(entry->mWeaponControl, 0.2f, 1.0f)) * 40.0f;
                        const JPH::Vec3 side = strikeDirection->Cross(JPH::Vec3::sAxisZ()).NormalizedOr(JPH::Vec3::sAxisX());
                        entry->mAiStrikeAimPoint = *strikeAim
                            + JPH::RVec3(side * (wander * wanderSide) + JPH::Vec3::sAxisZ() * (wander * wanderUp));
                    }
                    else
                        entry->mAiStrikeAimPoint.reset();                }
            }
#ifdef OPENMW_RAGDOLL_DEV_API
            if (RagdollCore::tuningText("OPENMW_RAGDOLL_AI_STRIKE_TRACE"))
            {
                static unsigned strikeTraceRows = 0;
                if (strikeTraceRows < 4000)
                {
                    ++strikeTraceRows;
                    Log(Debug::Info) << std::dec << std::setprecision(4)
                        << "RAGDOLL_AI_STRIKE attacker=" << attacker.getCellRef().getRefId()
                        << " attacking=" << stats.getAttackingOrSpell()
                        << " dir=" << (strikeDirection.has_value() ? 1 : 0)
                        << " active=" << entry->mAiStrikeActive
                        << " swinging=" << entry->mSwing.mSwinging
                        << " swing_speed=" << entry->mSwing.mVelocity.Length()
                        << " contacts=" << entry->mConsumedWeaponContactCount
                        << " strikes=" << entry->mConsumedWeaponStrikeCount
                        << " tgt_dist=" << nearestTargetDistance
                        << " anim_hand=" << entry->mAiAnimHandSpeed
                        << " phys_hand=" << entry->mAiPhysicalHandSpeed
                        << " hand_err=" << entry->mAiHandTrackingError
                        << " anim_lhand=" << entry->mAiAnimOffHandSpeed
                        << " lhand_err=" << entry->mAiOffHandTrackingError
                        << " attack_anim=" << attackAnimation
                        << " drive_vs_anim=" << [&] {
                               // How far the pose actually handed to the motors
                               // sits from the animation hand, and from the
                               // physical hand.
                               if (!entry->mAdaptedMotorTargetCaptured || !entry->mRightArm)
                                   return std::string("na");
                               const auto drive = entry->mRagdoll->getWorldBodyPosition(
                                   entry->mAdaptedMotorTargetPose, entry->mRightArm->mHand);
                               const auto anim = entry->mRagdoll->getWorldBodyPosition(
                                   entry->mMotorTargetPose, entry->mRightArm->mHand);
                               const auto phys = entry->mRagdoll->getWorldBodyPosition(
                                   entry->mCurrentPhysicsPose, entry->mRightArm->mHand);
                               if (!drive || !anim || !phys)
                                   return std::string("na");
                               std::ostringstream out;
                               out << JPH::Vec3(*drive - *anim).Length() << "/phys" << JPH::Vec3(*drive - *phys).Length();
                               return out.str();
                           }()
                        << " sh=" << [&] {
                               // error/clamp/frequency/damping/utilization per joint,
                               // from the previous step's drive.
                               std::ostringstream out;
                               const auto joint = [&](std::size_t body) {
                                   RagdollCore::MotorTargetTelemetry t;
                                   if (entry->mRagdoll->captureMotorTargetTelemetry(body, t))
                                   {
                                       out << t.mAngularError << '/' << t.mTargetClampDelta << '/'
                                           << t.mSpring.mFrequencyOrStiffness << '/' << t.mSpring.mDamping
                                           << "/w" << t.mTargetAngularVelocity.Length() << "/a"
                                           << (entry->mAnimationVelocity.count(body)
                                                   ? entry->mAnimationVelocity[body].mAngularVelocity.Length()
                                                   : -1.0f);
                                       if (t.mCurrentHingeAngle && t.mTargetHingeAngle)
                                           out << "/h" << *t.mCurrentHingeAngle << ':' << *t.mTargetHingeAngle << ':'
                                               << t.mHingeMinimum.value_or(0.0f) << ':' << t.mHingeMaximum.value_or(0.0f);
                                   }
                                   else
                                       out << "na";
                                   JPH::Vec3 impulse;
                                   float utilization = 0.0f;
                                   if (entry->mRagdoll->getMotorAngularImpulse(body, mPhysicsDt, impulse, utilization))
                                       out << "/u" << utilization;
                               };
                               joint(entry->mRightArm->mShoulder);
                               out << " fr=";
                               joint(entry->mRightArm->mFrame);
                               out << " el=";
                               joint(entry->mRightArm->mElbow);
                               out << " hd=";
                               joint(entry->mRightArm->mHand);
                               return out.str();
                           }()
                        << " swing_speed=" << entry->mSwing.mVelocity.Length()
                        << " reach_error=" << (entry->mRightArmTarget
                                ? entry->mRightArmTarget->mReachError : -1.0f)
                        << " target_d=" << (entry->mRightArmTarget
                                ? entry->mRightArmTarget->mReachWithoutShoulder : -1.0f);
                    if (nearestTarget && entry->mRightArm && nearestTarget->mTorsoBodyIndex
                        && entry->mCurrentPhysicsPoseCaptured && nearestTarget->mCurrentPhysicsPoseCaptured)
                    {
                        const auto hand = entry->mRagdoll->getWorldBodyPosition(
                            entry->mCurrentPhysicsPose, entry->mRightArm->mHand);
                        const auto torso = nearestTarget->mRagdoll->getWorldBodyPosition(
                            nearestTarget->mCurrentPhysicsPose, *nearestTarget->mTorsoBodyIndex);
                        if (hand && torso)
                            Log(Debug::Info) << "RAGDOLL_AI_STRIKE_HAND hand_torso="
                                << JPH::Vec3(*torso - *hand).Length();
                    }
                }
            }
#endif
            if (strikeDirection)
            {
                entry->mAiStrikeDirection = *strikeDirection;
                // The swing input moves the hand along the basis's "up" axis
                // (the second input axis), so the strike direction is passed
                // there and the first stroke pushes the hand out to the arm's
                // reach along it.
                const JPH::Vec3 lateral = strikeDirection->Cross(JPH::Vec3::sAxisZ());
                const JPH::Vec3 right
                    = lateral.LengthSq() > 1e-6f ? lateral.Normalized() : JPH::Vec3::sAxisX();
                const JPH::Vec3 forward = right.Cross(*strikeDirection).Normalized();
                // The swing is only the container that arms the contact
                // pipeline (and gives the strike its identity); the arm itself
                // follows the live attack animation's joint rotations, with the
                // swing motors giving it the authority to track them.
                if (!driveRagdollWeaponSwing(attacker, true, Misc::Convert::toOsg(right),
                        Misc::Convert::toOsg(forward), Misc::Convert::toOsg(*strikeDirection),
                        0.0f, 0.0f, true, mPhysicsDt))
                {
                    entry->mAiStrikeActive = false;
                    entry->mAiStrikeAimPoint.reset();
                }
                else
                    entry->mAiStrikeActive = true;
            }
            else if (entry->mAiStrikeActive && attackAnimation)
            {
                // The victim left the aiming range mid-swing. The attack
                // animation is still running, so the arm keeps following it
                // with the strike armed along the last direction.
                const JPH::Vec3 lateral = entry->mAiStrikeDirection.Cross(JPH::Vec3::sAxisZ());
                const JPH::Vec3 right
                    = lateral.LengthSq() > 1e-6f ? lateral.Normalized() : JPH::Vec3::sAxisX();
                const JPH::Vec3 forward = right.Cross(entry->mAiStrikeDirection).Normalized();
                if (!driveRagdollWeaponSwing(attacker, true, Misc::Convert::toOsg(right),
                        Misc::Convert::toOsg(forward), Misc::Convert::toOsg(entry->mAiStrikeDirection),
                        0.0f, 0.0f, true, mPhysicsDt))
                {
                    entry->mAiStrikeActive = false;
                    entry->mAiStrikeAimPoint.reset();
                }
            }
            else if (entry->mAiStrikeActive)
            {
                const JPH::Vec3 lateral = entry->mAiStrikeDirection.Cross(JPH::Vec3::sAxisZ());
                const JPH::Vec3 right
                    = lateral.LengthSq() > 1e-6f ? lateral.Normalized() : JPH::Vec3::sAxisX();
                const JPH::Vec3 forward = right.Cross(entry->mAiStrikeDirection).Normalized();
                driveRagdollWeaponSwing(attacker, true, Misc::Convert::toOsg(right),
                    Misc::Convert::toOsg(forward), Misc::Convert::toOsg(entry->mAiStrikeDirection),
                    0.0f, 0.0f, false, mPhysicsDt);
                entry->mAiStrikeActive = false;
                entry->mAiStrikeAimPoint.reset();
            }
        }
    }

    void PhysicsSystem::updatePassiveRagdolls()
    {
        // Scene changes asked for from other threads (clearRegionDamage).
        for (auto& [_, entry] : mPassiveRagdolls)
            entry->applyPendingTintClear();
        // Gore (stage 6): bursts queued by the hit receipts, and bleeding.
        if (goreSettings()[0] > 0.0f)
        {
            static auto lastBleed = std::chrono::steady_clock::now();
            const auto now = std::chrono::steady_clock::now();
            const float elapsed
                = std::clamp(std::chrono::duration<float>(now - lastBleed).count(), 0.0f, 0.1f);
            lastBleed = now;
            std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
            for (auto& [_, entry] : mPassiveRagdolls)
            {
                const MWWorld::Ptr ptr = entry->getPtr();
                if (ptr.isEmpty())
                {
                    entry->mPendingBlood.clear();
                    entry->mBleeds.clear();
                    continue;
                }
                for (const auto& burst : entry->mPendingBlood)
                    spawnRagdollBlood(ptr, Misc::Convert::toOsg(burst.mPoint), burst.mScale);
                entry->mPendingBlood.clear();
                const bool dead = ptr.getClass().isActor() && ptr.getClass().getCreatureStats(ptr).isDead();
                float bled = 0.0f;
                for (auto& bleed : entry->mBleeds)
                {
                    bleed.mRemaining -= elapsed;
                    bleed.mNextDrop -= elapsed;
                    if (!dead)
                        bled += goreSettings()[3] * bleed.mSeverity * elapsed;
                    if (bleed.mNextDrop <= 0.0f && bleed.mBody < entry->mRagdoll->getBodyCount())
                    {
                        bleed.mNextDrop = 0.5f;
                        JPH::BodyInterface& bodies = mPhysicsSystem.GetBodyInterface();
                        const JPH::BodyID id = entry->mRagdoll->getBodyId(bleed.mBody);
                        const JPH::RVec3 at = bodies.GetCenterOfMassPosition(id) + bodies.GetRotation(id) * bleed.mLocalPoint;
                        spawnRagdollBlood(ptr, Misc::Convert::toOsg(at), 0.45f * goreSettings()[0]);
                    }
                }
                std::erase_if(entry->mBleeds, [](const auto& bleed) { return bleed.mRemaining <= 0.0f; });
                if (bled > 0.0f && ptr.getClass().isActor())
                {
                    auto& stats = ptr.getClass().getCreatureStats(ptr);
                    MWMechanics::DynamicStat<float> health = stats.getHealth();
                    if (health.getCurrent() > 1.0f)
                    {
                        health.setCurrent(std::max(1.0f, health.getCurrent() - bled));
                        stats.setHealth(health);
                    }
                }
            }
        }
        updatePoweredRagdollPopulation();
        updateAutoDuel();
        updatePoweredAiStrikes();
        const float interpolationFactor = std::clamp(mTimeAccumJolt / mPhysicsDt, 0.0f, 1.0f);
        {
            std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
            for (auto& [_, entry] : mPassiveRagdolls)
            {
                entry->mRenderPosePrepared = entry->mCurrentPhysicsPoseCaptured
                    && RagdollCore::interpolateRagdollPoses(entry->mPreviousPhysicsPose, entry->mCurrentPhysicsPose,
                        interpolationFactor, entry->mInterpolatedPhysicsPose);
                if (!entry->mRenderPosePrepared)
                    continue;

                entry->mModeTransitionFactor = entry->mModeTransitionDuration > 0.0f
                    ? std::clamp(entry->mModeTransitionElapsed / entry->mModeTransitionDuration, 0.0f, 1.0f)
                    : 1.0f;
                if (entry->mModeTransitionFactor < 1.0f)
                    entry->mRenderPosePrepared = RagdollCore::interpolateRagdollPoses(entry->mTransitionSourcePose,
                        entry->mInterpolatedPhysicsPose, entry->mModeTransitionFactor, entry->mRenderPose);
                else
                    entry->mRenderPose = entry->mInterpolatedPhysicsPose;
                // Powered recovery remains rendered from the physical pose.
                // The settling phase must not conceal a failed rise by blending
                // the visible skeleton toward its animation motor target.
                // The skeleton is drawn at the actor's interpolated position
                // (its scene node only; moveActors placed it on the stepped
                // one), and the pose, relative to its Jolt step, is shifted by
                // the difference, so the body is drawn where the physics put it
                // at any frame rate. Above 60 fps the body shook 5.7 units a
                // frame against the camera (120 fps, RENDER_JITTER); 0.04 with
                // this (60 fps 0.17, unchanged). The actor's own position is
                // left alone (drawing actors interpolated also moved their
                // gameplay position and the Balmora hill stop fell 5 of 10).
                // A released root draws at its reference and needs none.
                // OPENMW_RAGDOLL_RENDER_REBASE=0 = off.
                static const auto renderRebase_tuned = RagdollCore::tuned<bool>([] {
                    const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_RENDER_REBASE");
                    return !(text && std::atoi(text) == 0);
                });
                entry->mRenderShift = JPH::Vec3::sZero();
                if (entry->mRenderPosePrepared && renderRebase_tuned.get() && entry->mSynchronizedActorPosition
                    && !(entry->mPowered && !entry->mRagdoll->isRootKinematic()))
                    if (const auto actor = mActors.find(entry->getPtr().mRef); actor != mActors.end())
                    {
                        // The position the pose was rebased to at its Jolt step
                        // (the actor's own stepped position can be a frame off
                        // with the async actor physics).
                        const osg::Vec3f stepped = Misc::Convert::toOsg(*entry->mSynchronizedActorPosition);
                        const osg::Vec3f drawn = actor->second->getInterpolatedPosition();
                        const osg::Vec3f shift = stepped - drawn;
                        SceneUtil::PositionAttitudeTransform* node = entry->getPtr().getRefData().getBaseNode();
                        if (node && shift.length2() > 1e-8f && shift.length2() < 100.0f * 100.0f)
                            if (const auto model = entry->mRagdoll->getModelVector(Misc::Convert::toJolt<JPH::Vec3>(shift)))
                            {
                                for (auto& transform : entry->mRenderPose.mBodyTransforms)
                                    transform.mTranslation += *model;
                                entry->mRenderShift = Misc::Convert::toJolt<JPH::Vec3>(shift);
                                // Drawn there only: the actor's position stays stepped.
                                node->setPosition(drawn);
                            }
                    }
                if (entry->mRenderPosePrepared)
                {
                    entry->mRenderInterpolationFactor = interpolationFactor;
                    entry->mRenderDiagnosticPhysicsStep = mJoltStepSerial;
                    if (RagdollCore::tuningText("OPENMW_RAGDOLL_RENDER_TRACE"))
                    {
                        entry->mRenderDiagnosticWorldPositions.clear();
                        for (std::size_t body = 0; body < entry->mRagdoll->getBodyCount(); ++body)
                        {
                            const auto world = entry->mRagdoll->getWorldBodyPosition(entry->mRenderPose, body);
                            if (!world) break;
                            entry->mRenderDiagnosticWorldPositions.push_back(*world);
                        }
                    }
                }
            }
        }

#ifdef OPENMW_RAGDOLL_DEV_API
        // OPENMW_RAGDOLL_RENDER_JITTER: once a rendered frame, the player's
        // drawn pelvis (its scene graph world matrix, as last drawn) and the
        // camera, with the frame time and the step interpolation factor
        // (>60 fps body shake in third person, 10-30 playthrough).
        static const auto renderJitter_tuned = RagdollCore::tuned<bool>([] {
            return RagdollCore::tuningText("OPENMW_RAGDOLL_RENDER_JITTER") != nullptr;
        });
        if (renderJitter_tuned.get())
        {
            const MWWorld::Ptr player = MWMechanics::getPlayer();
            MWBase::World* world = MWBase::Environment::get().getWorld();
            MWRender::Animation* animation = player.isEmpty() ? nullptr : world->getAnimation(player);
            const osg::Node* pelvisNode = animation ? animation->getNode("bip01 pelvis") : nullptr;
            static osg::Timer_t lastFrame = osg::Timer::instance()->tick();
            const osg::Timer_t now = osg::Timer::instance()->tick();
            const double dt = osg::Timer::instance()->delta_s(lastFrame, now);
            lastFrame = now;
            if (pelvisNode && !pelvisNode->getWorldMatrices().empty() && world->getCamera())
            {
                const osg::Vec3f pelvis = pelvisNode->getWorldMatrices().front().getTrans();
                const osg::Vec3d camera = world->getCamera()->getPosition();
                const osg::Vec3f actor = player.getRefData().getPosition().asVec3();
                Log(Debug::Info) << std::fixed << std::setprecision(3) << "RAGDOLL_RENDER_JITTER dt=" << dt
                                 << " alpha=" << interpolationFactor << " pelvis=" << pelvis.x() << ',' << pelvis.y()
                                 << ',' << pelvis.z() << " camera=" << camera.x() << ',' << camera.y() << ','
                                 << camera.z() << " actor=" << actor.x() << ',' << actor.y() << ',' << actor.z();
            }
            // The nearest powered NPC's drawn pelvis in the world (its
            // smoothness frame to frame; the camera does not follow it).
            float nearest = 2000.0f;
            const osg::Node* npcPelvis = nullptr;
            for (const auto& [_, entry] : mPassiveRagdolls)
            {
                const MWWorld::Ptr npc = entry->getPtr();
                if (!entry->mPowered || npc.isEmpty() || npc == player)
                    continue;
                const float distance = (npc.getRefData().getPosition().asVec3()
                    - player.getRefData().getPosition().asVec3()).length();
                MWRender::Animation* npcAnimation = distance < nearest ? world->getAnimation(npc) : nullptr;
                if (const osg::Node* node = npcAnimation ? npcAnimation->getNode("bip01 pelvis") : nullptr)
                {
                    nearest = distance;
                    npcPelvis = node;
                }
            }
            if (npcPelvis && !npcPelvis->getWorldMatrices().empty())
            {
                const osg::Vec3f pelvis = npcPelvis->getWorldMatrices().front().getTrans();
                Log(Debug::Info) << std::fixed << std::setprecision(3) << "RAGDOLL_RENDER_JITTER_NPC dt=" << dt
                                 << " pelvis=" << pelvis.x() << ',' << pelvis.y() << ',' << pelvis.z();
            }
        }
#endif
        std::vector<std::pair<MWWorld::Ptr, bool>> failed;
        for (auto& [_, entry] : mPassiveRagdolls)
        {
            const std::span<const RagdollCore::Transform> renderBodies(entry->mRenderPose.mBodyTransforms.data(),
                std::min(entry->mPoseBindingBodyCount, entry->mRenderPose.mBodyTransforms.size()));
            const bool poseApplied = entry->mRenderPosePrepared && renderBodies.size() == entry->mPoseBindingBodyCount
                && entry->mPoseBinding->applyPose(renderBodies);
            // Death watch, drawn: the root bone where the scene graph draws
            // it against where the physics put it; a change of that offset
            // is the corpse dragged by its actor (RAGDOLL_DEATH_DRAWN).
            if (poseApplied && !entry->mPowered && entry->mDeathWatchActorStart)
            {
                MWRender::Animation* animation = MWBase::Environment::get().getWorld()->getAnimation(entry->getPtr());
                const osg::Node* rootBone = animation ? animation->getNode("bip01 pelvis") : nullptr;
                const auto placed = entry->mRagdoll->getWorldBodyPosition(entry->mRenderPose, 0);
                if (rootBone && placed && !rootBone->getParentalNodePaths().empty())
                {
                    const osg::Vec3f drawn = osg::computeLocalToWorld(rootBone->getParentalNodePaths().front()).getTrans();
                    const osg::Vec3f offset = drawn - Misc::Convert::toOsg(*placed - JPH::RVec3(entry->mRenderShift));
                    if (!entry->mDeathDrawnOrigin)
                        entry->mDeathDrawnOrigin = offset;
                    entry->mDeathDrawnOff = std::max(entry->mDeathDrawnOff, (offset - *entry->mDeathDrawnOrigin).length());
                }
            }
            if (!entry->mPoweredControlFailureLine && poseApplied)
                continue;
            // Animation traversal may rebuild a player skeleton on the same
            // frame that a fall or retry changes animation state. A prepared
            // physics pose remains authoritative for a powered body even if
            // this frame's animation skeleton cannot accept it. Retain the
            // body and retry on later frames; actual controller failures and
            // missing prepared poses still use the safe actor fallback below.
            if (entry->mPowered && !entry->mPoweredControlFailureLine && entry->mRenderPosePrepared)
                continue;
            ++mPassiveRagdollPoseFailures;
            Log(Debug::Warning) << (entry->mPowered ? "Powered" : "Passive")
                                << " ragdoll synchronization failed for " << entry->getPtr().getCellRef().getRefId()
                                << "; control failure line=" << std::to_string(entry->mPoweredControlFailureLine)
                                << ", render pose prepared=" << entry->mRenderPosePrepared
                                << "; restoring "
                                << (entry->mPowered ? "ordinary actor collision" : "corpse fallback");
            failed.emplace_back(entry->getPtr(), entry->mPowered);
        }
        for (const auto& [ptr, powered] : failed)
        {
            std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
            erasePassiveRagdoll(ptr.mRef);
            if (const auto actor = mActors.find(ptr.mRef); actor != mActors.end())
                actor->second->enableCollisionBody(powered);
        }
    }

    void PhysicsSystem::enforcePassiveRagdollCap()
    {
        const std::size_t cap = static_cast<std::size_t>(std::max(0, Settings::game().mPassiveRagdollCap.get()));
        // The duel puppet is unpowered but no corpse.
        const auto passiveCount = [&] {
            return static_cast<std::size_t>(std::count_if(mPassiveRagdolls.begin(), mPassiveRagdolls.end(),
                [&](const auto& item) { return !item.second->mPowered && !isNetPuppetEntry(*item.second); }));
        };
        while (passiveCount() > cap)
        {
            auto candidate = mPassiveRagdolls.end();
            for (auto it = mPassiveRagdolls.begin(); it != mPassiveRagdolls.end(); ++it)
            {
                if (it->second->mPowered || isNetPuppetEntry(*it->second))
                    continue;
                if (candidate == mPassiveRagdolls.end()
                    || (candidate->second->mRagdoll->isActive() && !it->second->mRagdoll->isActive())
                    || (candidate->second->mRagdoll->isActive() == it->second->mRagdoll->isActive()
                        && it->second->mSerial < candidate->second->mSerial))
                    candidate = it;
            }
            ++mPassiveRagdollsEvicted;
            erasePassiveRagdoll(candidate->first);
        }
    }

    bool PhysicsSystem::isNetPuppetEntry(const PassiveRagdollEntry& entry) const
    {
        return mNetPuppetRef != nullptr && entry.mSerial == mNetPuppetSerial;
    }

    bool PhysicsSystem::isNetPuppet(const MWWorld::ConstPtr& ptr) const
    {
        if (mNetPuppetRef == nullptr || ptr.isEmpty() || ptr.mRef != mNetPuppetRef)
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        return found != mPassiveRagdolls.end() && isNetPuppetEntry(*found->second);
    }

    MWWorld::Ptr PhysicsSystem::getNetPuppet() const
    {
        if (mNetPuppetRef == nullptr)
            return {};
        const auto found = mPassiveRagdolls.find(mNetPuppetRef);
        if (found == mPassiveRagdolls.end() || !isNetPuppetEntry(*found->second))
            return {};
        return found->second->getPtr();
    }

    bool PhysicsSystem::setNetPuppet(const MWWorld::Ptr& ptr)
    {
        if (ptr.isEmpty() || !ptr.getClass().isActor() || ptr == MWMechanics::getPlayer())
            return false;
        if (isNetPuppet(ptr))
            return true;
        // Built fresh, with the weapon and shield it holds now: its parts must
        // match the remote body's one for one.
        if (mPassiveRagdolls.contains(ptr.mRef))
        {
            std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
            erasePassiveRagdoll(ptr.mRef);
        }
        MWRender::Animation* animation = MWBase::Environment::get().getWorld()->getAnimation(ptr);
        if (animation == nullptr || animation->getSkeleton() == nullptr
            || !addPoweredRagdoll(ptr, *animation->getSkeleton()))
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mRagdoll)
            return false;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        PassiveRagdollEntry& entry = *found->second;
        // Unpowered: no controller, balance, AI strike or population policy
        // touches it; the corpse path draws it and keeps its actor under it.
        entry.mPowered = false;
        entry.mRagdoll->disableMotors();
        JPH::BodyInterface& bodies = mPhysicsSystem.GetBodyInterface();
        for (std::size_t i = 0; i < entry.mRagdoll->getBodyCount(); ++i)
            bodies.SetMotionType(entry.mRagdoll->getBodyId(i), JPH::EMotionType::Kinematic, JPH::EActivation::Activate);
        mNetPuppetRef = ptr.mRef;
        mNetPuppetSerial = entry.mSerial;
        mNetPuppetPoses.clear();
        mNetPlayback = -1;
        mNetBodyCountWarned = false;
        Log(Debug::Info) << "KINETIC_NET puppet actor=" << ptr.getCellRef().getRefId()
                         << " bodies=" << entry.mRagdoll->getBodyCount();
        return true;
    }

    void PhysicsSystem::clearNetPuppet()
    {
        if (mNetPuppetRef != nullptr && mPassiveRagdolls.contains(mNetPuppetRef))
        {
            std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
            if (isNetPuppetEntry(*mPassiveRagdolls.at(mNetPuppetRef)))
                erasePassiveRagdoll(mNetPuppetRef);
        }
        mNetPuppetRef = nullptr;
        mNetPuppetSerial = 0;
        mNetPuppetPoses.clear();
        mNetPlayback = -1;
    }

    void PhysicsSystem::setNetActive(bool active)
    {
        mNetActive = active;
        if (!active)
        {
            mNetPlayerPoses.clear();
            mNetBlowsOut.clear();
            mNetBlowsIn.clear();
        }
    }

    void PhysicsSystem::receiveNetPoses(std::vector<MWWorld::NetPose> poses)
    {
        for (auto& pose : poses)
        {
            if (!mNetPuppetPoses.empty() && pose.mStep <= mNetPuppetPoses.back().mStep)
            {
                // A restarted sender counts its steps from zero again.
                if (mNetPuppetPoses.back().mStep - pose.mStep < 600)
                    continue;
                mNetPuppetPoses.clear();
                mNetPlayback = -1;
            }
            mNetPuppetPoses.push_back(std::move(pose));
        }
        while (mNetPuppetPoses.size() > 120)
            mNetPuppetPoses.pop_front();
    }

    void PhysicsSystem::receiveNetBlows(std::vector<MWWorld::NetBlow> blows)
    {
        mNetBlowsIn.insert(mNetBlowsIn.end(), blows.begin(), blows.end());
    }

    std::vector<MWWorld::NetPose> PhysicsSystem::takeNetPlayerPoses()
    {
        return std::exchange(mNetPlayerPoses, {});
    }

    std::vector<MWWorld::NetBlow> PhysicsSystem::takeNetBlows()
    {
        return std::exchange(mNetBlowsOut, {});
    }

    void PhysicsSystem::queueNetBlow(const RagdollCore::Blow& blow)
    {
        MWWorld::NetBlow net;
        net.mKind = static_cast<std::uint8_t>(blow.mKind);
        net.mRegion = static_cast<std::uint8_t>(blow.mRegion);
        net.mBodyIndex = blow.mBodyIndex;
        net.mPoint = { static_cast<float>(blow.mPoint.GetX()), static_cast<float>(blow.mPoint.GetY()),
            static_cast<float>(blow.mPoint.GetZ()) };
        net.mDirection = { blow.mDirection.GetX(), blow.mDirection.GetY(), blow.mDirection.GetZ() };
        net.mAttackerToVictim
            = { blow.mAttackerToVictim.GetX(), blow.mAttackerToVictim.GetY(), blow.mAttackerToVictim.GetZ() };
        net.mApproachSpeed = blow.mApproachSpeed;
        net.mCarriedMass = blow.mCarriedMass;
        net.mDamage = blow.mDamage;
        net.mReactionImpulse = blow.mReactionImpulse;
        net.mBlocked = blow.mBlocked;
        net.mParried = blow.mParried;
        net.mGuardBeaten = blow.mGuardBeaten;
        mNetBlowsOut.push_back(net);
    }

    void PhysicsSystem::captureNetPlayerPose()
    {
        if (!mNetActive)
            return;
        const MWWorld::Ptr player = MWMechanics::getPlayer();
        if (player.isEmpty())
            return;
        const auto found = mPassiveRagdolls.find(player.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mRagdoll)
            return;
        const PassiveRagdollEntry& entry = *found->second;
        const JPH::BodyInterface& bodies = mPhysicsSystem.GetBodyInterface();
        MWWorld::NetPose pose;
        pose.mStep = static_cast<std::uint32_t>(mJoltStepSerial);
        const osg::Vec3f actor = player.getRefData().getPosition().asVec3();
        pose.mActor = { actor.x(), actor.y(), actor.z() };
        pose.mGuard = entry.mGuardActive;
        pose.mGuardForward = { entry.mGuardForward.GetX(), entry.mGuardForward.GetY() };
        pose.mBodies.resize(entry.mRagdoll->getBodyCount());
        for (std::size_t i = 0; i < pose.mBodies.size(); ++i)
        {
            JPH::RVec3 position;
            JPH::Quat rotation;
            bodies.GetPositionAndRotation(entry.mRagdoll->getBodyId(i), position, rotation);
            pose.mBodies[i].mPosition = { static_cast<float>(position.GetX()), static_cast<float>(position.GetY()),
                static_cast<float>(position.GetZ()) };
            pose.mBodies[i].mRotation = { rotation.GetX(), rotation.GetY(), rotation.GetZ(), rotation.GetW() };
        }
        mNetPlayerPoses.push_back(std::move(pose));
        if (mNetPlayerPoses.size() > 16)
            mNetPlayerPoses.erase(mNetPlayerPoses.begin());
    }

    void PhysicsSystem::stepNetPuppet()
    {
        if (mNetPuppetRef == nullptr && mNetBlowsIn.empty())
            return;
        // Blows the remote game landed on the player: the player's body takes
        // them here, through the one entry point (damage comes as a Hit).
        if (!mNetBlowsIn.empty())
        {
            const MWWorld::Ptr player = MWMechanics::getPlayer();
            const auto self = player.isEmpty() ? mPassiveRagdolls.end() : mPassiveRagdolls.find(player.mRef);
            for (const MWWorld::NetBlow& net : mNetBlowsIn)
            {
                if (self == mPassiveRagdolls.end() || !self->second->mPowered)
                    break;
                PassiveRagdollEntry& victim = *self->second;
                RagdollCore::Blow blow;
                blow.mId = RagdollCore::nextBlowId();
                blow.mPhysicsStep = mJoltStepSerial;
                blow.mAttacker = mNetPuppetSerial;
                blow.mVictim = victim.mSerial;
                blow.mKind = static_cast<RagdollCore::BlowKind>(
                    std::min<int>(net.mKind, static_cast<int>(RagdollCore::BlowKind::Scripted)));
                blow.mRegion = static_cast<RagdollCore::AnatomicalRegion>(
                    std::min<int>(net.mRegion, static_cast<int>(RagdollCore::AnatomicalRegion::RightFoot)));
                blow.mBodyIndex = net.mBodyIndex < victim.mRagdoll->getBodyCount()
                    ? net.mBodyIndex : std::numeric_limits<std::uint32_t>::max();
                blow.mPoint = JPH::RVec3(net.mPoint[0], net.mPoint[1], net.mPoint[2]);
                blow.mDirection = JPH::Vec3(net.mDirection[0], net.mDirection[1], net.mDirection[2]);
                blow.mAttackerToVictim
                    = JPH::Vec3(net.mAttackerToVictim[0], net.mAttackerToVictim[1], net.mAttackerToVictim[2]);
                blow.mApproachSpeed = net.mApproachSpeed;
                blow.mCarriedMass = net.mCarriedMass;
                blow.mDamage = net.mDamage;
                blow.mReactionImpulse = net.mReactionImpulse;
                blow.mBlocked = net.mBlocked;
                blow.mParried = net.mParried;
                blow.mGuardBeaten = net.mGuardBeaten;
                blow.mVictimState = victim.blowVictimState();
                const float pushed = blow.valid() ? applyBlowReaction(victim, blow) : -1.0f;
                Log(Debug::Info) << "KINETIC_NET blow received region=" << static_cast<int>(net.mRegion)
                                 << " approach=" << net.mApproachSpeed << " damage=" << net.mDamage
                                 << " blocked=" << net.mBlocked << " pushed=" << pushed;
            }
            mNetBlowsIn.clear();
        }
        const auto found = mNetPuppetRef ? mPassiveRagdolls.find(mNetPuppetRef) : mPassiveRagdolls.end();
        if (found == mPassiveRagdolls.end() || !isNetPuppetEntry(*found->second) || mNetPuppetPoses.empty())
            return;
        PassiveRagdollEntry& entry = *found->second;
        // Playback runs one sender step per step, 6 steps (100 ms at 60 Hz)
        // behind the newest pose, so a late packet still has a pose after it.
        constexpr double delay = 6.0;
        const double wanted = static_cast<double>(mNetPuppetPoses.back().mStep) - delay;
        if (mNetPlayback < 0.0 || std::abs(mNetPlayback - wanted) > 30.0)
            mNetPlayback = wanted;
        else
            mNetPlayback += 1.0 + std::clamp((wanted - mNetPlayback) * 0.05, -0.25, 0.25);
        while (mNetPuppetPoses.size() > 1 && mNetPuppetPoses[1].mStep <= mNetPlayback)
            mNetPuppetPoses.pop_front();
        const MWWorld::NetPose& from = mNetPuppetPoses.front();
        const MWWorld::NetPose& to = mNetPuppetPoses.size() > 1 ? mNetPuppetPoses[1] : from;
        const std::size_t count = entry.mRagdoll->getBodyCount();
        if (from.mBodies.size() != count || to.mBodies.size() != count)
        {
            if (!mNetBodyCountWarned)
                Log(Debug::Warning) << "KINETIC_NET puppet has " << count << " parts, the remote body "
                                    << from.mBodies.size() << " (different weapon or shield?)";
            mNetBodyCountWarned = true;
            return;
        }
        float t = to.mStep > from.mStep
            ? static_cast<float>(std::clamp((mNetPlayback - from.mStep) / (to.mStep - from.mStep), 0.0, 1.0))
            : 0.0f;
        const auto position = [](const MWWorld::NetBody& body) {
            return JPH::RVec3(body.mPosition[0], body.mPosition[1], body.mPosition[2]);
        };
        // A teleport (a round's reset) is not drawn as a flight.
        if (count > 0 && JPH::Vec3(position(to.mBodies[0]) - position(from.mBodies[0])).Length() > 150.0f)
            t = 1.0f;
        JPH::BodyInterface& bodies = mPhysicsSystem.GetBodyInterface();
        for (std::size_t i = 0; i < count; ++i)
        {
            const MWWorld::NetBody& a = from.mBodies[i];
            const MWWorld::NetBody& b = to.mBodies[i];
            const JPH::RVec3 target = position(a) + JPH::RVec3(JPH::Vec3(position(b) - position(a)) * t);
            const JPH::Quat qa(a.mRotation[0], a.mRotation[1], a.mRotation[2], a.mRotation[3]);
            JPH::Quat qb(b.mRotation[0], b.mRotation[1], b.mRotation[2], b.mRotation[3]);
            if (qa.Dot(qb) < 0.0f)
                qb = -qb;
            const JPH::Quat rotation = qa.SLERP(qb, t).Normalized();
            const JPH::BodyID id = entry.mRagdoll->getBodyId(i);
            if (JPH::Vec3(bodies.GetPosition(id) - target).Length() > 150.0f)
                bodies.SetPositionAndRotation(id, target, rotation, JPH::EActivation::Activate);
            else
                bodies.MoveKinematic(id, target, rotation, mPhysicsDt);
        }
        const MWWorld::NetPose& guard = t < 0.5f ? from : to;
        if (guard.mGuard && !entry.mGuardActive)
            entry.mGuardStartPhysicsStep = mJoltStepSerial;
        entry.mGuardActive = guard.mGuard;
        const JPH::Vec3 forward(guard.mGuardForward[0], guard.mGuardForward[1], 0.0f);
        if (forward.LengthSq() > 1e-6f)
            entry.mGuardForward = forward.Normalized();
    }

    std::optional<PassiveRagdollBalanceState> PhysicsSystem::getPassiveRagdollBalanceState(
        const MWWorld::ConstPtr& ptr) const
    {
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        return found == mPassiveRagdolls.end() ? std::nullopt : found->second->mBalanceSnapshot;
    }

    std::optional<PassiveRagdollDebugState> PhysicsSystem::getPassiveRagdollDebugState(
        const MWWorld::ConstPtr& ptr) const
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return std::nullopt;

        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        PassiveRagdollDebugState result;
        // RXN-01 (the old reaction layer) was removed 10-07: its debug state
        // stays at the defaults (off).
        result.mBalance = found->second->mBalanceSnapshot;
        result.mStrikeRightHand = found->second->mSwingArmIsRight;
        result.mThrustInput = found->second->mNativeThrust && found->second->mNativeStrikeHeld;
        result.mPlayerPressCount = found->second->mPlayerPressCount;
        result.mWristOutsideRight = found->second->wristOutsideDegrees(found->second->mRightArm, nullptr);
        result.mWristOutsideLeft = found->second->wristOutsideDegrees(found->second->mLeftArm, nullptr);
        result.mTipActive = found->second->mTip.mActive;
        result.mHscActive = found->second->mHscActive;
        result.mHscEl = found->second->mHscEl;
        result.mHscBladeEl = found->second->mHscBladeEl;
        result.mChamberS = found->second->mChamberS;
        result.mChamberHandError = found->second->mChamberHandError;
        result.mChamberWristError = found->second->mChamberWristError;
        result.mHscGrip = Misc::Convert::toOsg(found->second->mHscGrip);
        result.mChamberLine = found->second->mAttackLine;
        result.mHscStage = static_cast<int>(found->second->mAttackStage);
        result.mHscStanceX = found->second->mStanceX;
        result.mHscStanceY = found->second->mStanceY;
        result.mHscAngle = found->second->mAttackAngle;
        result.mHscDrivePhase = found->second->mHscDrivePhase;
        result.mHscPlaneMiss = found->second->mCutPlaneMiss;
        result.mHscSnap = found->second->mSnapCut;
        result.mHscChain = found->second->mChainFrom.has_value();
        result.mHscAuto = found->second->mAutoAttack;
        result.mHscRiposte = found->second->mPlayerRiposte;
        result.mHscFlow = found->second->mChainFlowGain;
        result.mHscSinceParryWon = found->second->mAiSinceParryWon;
        result.mHscGripMode = static_cast<int>(found->second->mGrip);
        result.mHscJabs = found->second->mJabCount;
        result.mHscJab = found->second->leftJabExtension();
        result.mHscRoute = found->second->mFlowU;
        result.mHscReseat = found->second->mReseatEase;
        result.mHscReseats = found->second->mReseatCount;
        result.mHscTrajectory.reserve(found->second->mHscPreview.size());
        for (const JPH::RVec3& point : found->second->mHscPreview)
            result.mHscTrajectory.push_back(Misc::Convert::toOsg(point));
        result.mArmOwnerLeft = found->second->armOwnerName(found->second->mArmOwner[0]);
        result.mOwnerLegs = found->second->armOwnerName(found->second->mBodyOwner[PassiveRagdollEntry::ChannelLegs]);
        result.mOwnerTrunk = found->second->armOwnerName(found->second->mBodyOwner[PassiveRagdollEntry::ChannelTrunk]);
        result.mOwnerHead = found->second->armOwnerName(found->second->mBodyOwner[PassiveRagdollEntry::ChannelHead]);
        result.mArmOwnerRight = found->second->armOwnerName(found->second->mArmOwner[1]);
        result.mHscBlade = Misc::Convert::toOsg(found->second->mHscBlade);
        if (found->second->mRightArmTarget)
            result.mHscPole = Misc::Convert::toOsg(found->second->mRightArmTarget->mWorldBend);
        result.mTipAz = found->second->mTip.mAz;
        result.mTipGap = found->second->mTipGap;
        result.mTipCommand = Misc::Convert::toOsg(found->second->mTipDesiredTip);
        result.mBlockStaggers = found->second->mBlockStaggerCount;
        result.mDazeTime = found->second->mDazeTime;
        result.mDazeDrop = found->second->mDazeDrop;
        result.mDazeCount = found->second->mDazeCount;
        result.mTipEl = found->second->mTip.mEl;
        result.mTipRadius = found->second->mTip.mRadius;
        result.mTipBodyTwist = found->second->mTip.mBodyTwist;
        result.mTipBodyLean = found->second->mTip.mBodyLean;
        const auto& attack = found->second->mAnimationAttack;
        switch (attack.mPhase)
        {
            case MWMechanics::RagdollAttackPhase::Idle: break;
            case MWMechanics::RagdollAttackPhase::WindUp: result.mAttackPhase = "wind_up"; break;
            case MWMechanics::RagdollAttackPhase::Release: result.mAttackPhase = "release"; break;
            case MWMechanics::RagdollAttackPhase::FollowThrough: result.mAttackPhase = "follow_through"; break;
        }
        result.mAttackId = attack.mAttackId;
        result.mAttackStrength = attack.mStrength;
        result.mAttackType = attack.mAttackType;
        result.mPhysicsStep = mJoltStepSerial;
        result.mWoundReceiptCount = found->second->mWoundReceiptCount;
        result.mWoundMarkCount = found->second->mWoundMarkCount;
        result.mLastWoundKind = found->second->mLastWoundKind;
        result.mLastWoundRegion = found->second->mLastWoundRegion;
        result.mLastWoundHealth = found->second->mLastWoundHealth;
        result.mLastWoundFatigue = found->second->mLastWoundFatigue;
        result.mLocalReactionAngle = found->second->mLocalReactionAngle;
        result.mStepQuality = found->second->mStepQuality;
        result.mPendingImpact = mPendingRagdollImpacts.contains(ptr.mRef);
        result.mInheritedAnimationMotion = found->second->mInheritedAnimationMotion;
        result.mPowered = found->second->mPowered;
        result.mCombatReactionCount = found->second->mCombatReactions.count();
        result.mLastCombatReaction = found->second->mCombatReactions.last();
        result.mReferencePosition = Misc::Convert::toOsg(found->second->mRagdoll->getReferencePosition());
        if (found->second->mSynchronizedActorPosition)
            result.mPhysicsActorPosition = Misc::Convert::toOsg(*found->second->mSynchronizedActorPosition);
        RagdollCore::RagdollTelemetry telemetry;
        if (!found->second->mRagdoll->captureTelemetry(telemetry))
            return std::nullopt;
        const RagdollCore::RagdollProfile* debugProfile
            = found->second->mRuntimeProfile ? found->second->mRuntimeProfile.get() : mPassiveRagdollProfile.get();
        if (found->second->mWeapon)
        {
            const LiveWeaponAttachment& attached = *found->second->mWeapon;
            if (attached.mHandBody >= telemetry.mBodies.size() || attached.mWeaponBody >= telemetry.mBodies.size())
                return std::nullopt;
            const auto& hand = telemetry.mBodies[attached.mHandBody];
            const auto& weapon = telemetry.mBodies[attached.mWeaponBody];
            PassiveRagdollWeaponState item;
            item.mRecordId = attached.mRecordId;
            item.mBodyIndex = attached.mWeaponBody;
            item.mHandBodyIndex = attached.mHandBody;
            item.mItemWeight = attached.mItemWeight;
            item.mItemSpeed = attached.mItemSpeed;
            item.mPhysicalMass = weapon.mMass;
            item.mLength = attached.mLength;
            const JPH::RVec3 grip = weapon.mPosition - weapon.mRotation * attached.mCenterFromHand;
            item.mGripSeparation = JPH::Vec3(grip - hand.mPosition).Length();
            item.mGripAngularError
                = 2 * std::acos(std::clamp(std::abs(hand.mRotation.Dot(weapon.mRotation)), 0.0f, 1.0f));
            item.mTwoHanded = attached.mTwoHanded;
            item.mSupportActive = found->second->mWeaponSupportActive;
            if (found->second->mSupportGripEnabled)
                item.mSupportGrip = found->second->mSupportGripTelemetry;
            if (const auto& arm = found->second->mLeftArm)
            {
                item.mSupportHandBodyIndex = arm->mHand;
                if (const auto& request = found->second->mLeftArmTarget;
                    request && request->mWeaponSupport && item.mSupportActive)
                {
                    item.mSupportTarget = Misc::Convert::toOsg(request->mWorldTarget);
                    item.mSupportPositionError = JPH::Vec3(
                        telemetry.mBodies[arm->mHand].mPosition - request->mWorldTarget).Length();
                    const auto& entry = *found->second;
                    const auto& support = entry.mSupportGripTelemetry;
                    if (entry.mSupportGripEnabled && entry.mSupportGripFrame
                        && entry.mSupportGrip && entry.mSupportGrip->attached()
                        && support.mAttached && support.mConstraintActive && support.mSolvedObservation
                        && mJoltStepSerial > 0 && support.mObservationPhysicsStep == mJoltStepSerial - 1)
                    {
                        // Read both ends from this one current telemetry capture.
                        // The existing command still refers to the pre-solve pose;
                        // its tracking error includes ordinary weapon travel.
                        const auto& frame = *entry.mSupportGripFrame;
                        const JPH::RVec3 target
                            = weapon.mPosition + weapon.mRotation * (frame.mOffset * frame.mScale);
                        item.mSupportPhysical = PassiveRagdollWeaponState::SupportPhysicalObservation{
                            Misc::Convert::toOsg(target),
                            JPH::Vec3(telemetry.mBodies[arm->mHand].mPosition - target).Length(),
                            support.mObservationPhysicsStep };
                    }
                    if (request->mHandRotation && found->second->mCurrentPhysicsPoseCaptured)
                        item.mSupportRotationError = 2 * std::acos(std::clamp(std::abs(
                            found->second->mCurrentPhysicsPose.mBodyTransforms[arm->mHand].mRotation.Dot(
                                *request->mHandRotation)), 0.0f, 1.0f));
                }
            }
            item.mVelocitySolverSteps = attached.mAttached ? attached.mVelocitySolverSteps : 0;
            item.mLinearBreakImpulse = attached.mLinearBreakImpulse;
            item.mAngularBreakImpulse = attached.mAngularBreakImpulse;
            item.mLastLinearImpulse = attached.mLastLinearImpulse;
            item.mLastAngularImpulse = attached.mLastAngularImpulse;
            item.mAttached = attached.mAttached;
            item.mConstraintEnabled
                = found->second->mRagdoll->isConstraintEnabled(attached.mWeaponBody).value_or(false);
            item.mCallbackArmed = attached.mCallbackArmed;
            item.mBodyRetained = !attached.mAttached;
            item.mReleaseReason = attached.mReleaseReason
                ? std::string(weaponReleaseReasonName(*attached.mReleaseReason))
                : "attached";
            item.mReleaseRequests = attached.mReleaseRequests;
            item.mReleaseCount = attached.mReleaseCount;
            {
                std::scoped_lock contactLock(found->second->mContactMutex);
                item.mContacts = attached.mContacts;
                item.mGroundContacts = attached.mGroundContacts;
                item.mOtherRagdollContacts = attached.mOtherRagdollContacts;
            }
            item.mContactEventCount = found->second->mConsumedWeaponContactCount;
            item.mStrikeEventCount = found->second->mConsumedWeaponStrikeCount;
            item.mDamageEventCount = found->second->mConsumedWeaponDamageCount;
            item.mLastImpactFraction = found->second->mLastImpactFraction;
            item.mLastAuthoredDamage = found->second->mLastAuthoredDamage;
            item.mLastFinalDamage = found->second->mLastFinalDamage;
            item.mLastDamageBlocked = found->second->mLastDamageBlocked;
            item.mDefenseOutcomeCount = found->second->mConsumedWeaponDefenseOutcomeCount;
            if (const auto& defense = found->second->mLastPhysicalDefense)
            {
                item.mLastDefenseOutcome = RagdollCore::physicalDefenseOutcomeName(defense->mOutcome);
                item.mLastContactDisposition = RagdollCore::physicalContactDispositionName(defense->mDisposition);
                item.mLastApproachAlignment = defense->mApproachAlignment;
                item.mLastGuardFacingAlignment = defense->mGuardFacingAlignment;
                item.mLastDefenseDamageScale = defense->mDamageScale;
                item.mLastDefenseImpulseScale = defense->mImpulseScale;
                item.mLastRedirectScale = defense->mRedirectScale;
            }
            item.mLastRedirectImpulse = found->second->mLastWeaponRedirectImpulse;
            const auto copyContact = [](const RagdollCore::WeaponContactEvent& contact) {
                PassiveRagdollWeaponState::ContactEvent event;
                event.mContactIdentity = contact.contactIdentity();
                event.mAttackerActorIdentity = contact.attackerActorIdentity();
                event.mTargetActorIdentity = contact.targetActorIdentity();
                event.mWeaponRecordId = contact.weaponRecordId();
                event.mWeaponBodyIdentity = contact.weaponBodyIdentity();
                event.mTargetBodyIdentity = contact.targetBodyIdentity();
                event.mPhysicsStep = contact.physicsStep();
                event.mSwingIdentity = contact.swingIdentity();
                event.mSwingActive = contact.swingActive();
                event.mPoint = Misc::Convert::toOsg(contact.point());
                event.mNormal = Misc::Convert::toOsg(contact.normal());
                event.mRelativeNormalSpeed = contact.relativeNormalSpeed();
                event.mRelativeSpeed = contact.relativeSpeed();
                event.mEffectiveMass = contact.effectiveMass();
                event.mNormalImpulse = contact.normalImpulse();
                return event;
            };
#ifdef OPENMW_RAGDOLL_DEV_API
            if (found->second->mContactWindowAuditEnabled)
            {
                item.mContactWindows.emplace();
                item.mContactWindowsEvictedCount = found->second->mContactWindowsEvictedCount;
                for (const auto& sample : found->second->mContactWindowObservations)
                {
                    PassiveRagdollWeaponState::ContactWindowObservation row;
                    row.mPhysicsStep = sample.mPhysicsStep;
                    row.mActorSerial = sample.mActorSerial;
                    row.mLifecycleEpoch = sample.mLifecycleEpoch;
                    row.mWeaponBodyIdentity = sample.mWeaponBodyIdentity;
                    row.mAttackId = sample.mAttack.mAttackId;
                    row.mSwingIdentity = sample.mSwingIdentity;
                    static constexpr const char* phases[] = { "idle", "wind_up", "release", "follow_through" };
                    row.mPhase = phases[static_cast<unsigned>(sample.mAttack.mPhase)];
                    row.mNativeOwned = sample.mNativeOwned;
                    row.mOverflow = sample.mOverflow;
                    for (const auto& contact : sample.mContacts) row.mContacts.push_back(copyContact(contact));
                    for (const auto& contact : sample.mStrikes) row.mStrikes.push_back(copyContact(contact));
                    for (const auto& contact : sample.mDispatches) row.mDispatches.push_back(copyContact(contact));
                    row.mPhysicalHitIds = sample.mPhysicalHitIds;
                    item.mContactWindows->push_back(std::move(row));
                }
            }
#endif
            if (const auto& contact = found->second->mLastConsumedWeaponContact)
                item.mLastContactEvent = copyContact(*contact);
            if (const auto& contact = found->second->mLastConsumedWeaponStrike)
                item.mLastStrikeEvent = copyContact(*contact);
            result.mWeapon = std::move(item);
        }
        if (found->second->mHorizontalBodyWidth <= 0)
        {
            JPH::AABox bounds;
            for (std::size_t i = 0; i < found->second->mRagdoll->getBodyCount(); ++i)
                bounds.Encapsulate(mPhysicsSystem.GetBodyInterface().GetTransformedShape(found->second->mRagdoll->getBodyId(i)).GetWorldSpaceBounds());
            const auto size = bounds.GetSize();
            found->second->mHorizontalBodyWidth = std::max(size.GetX(), size.GetY());
        }
        result.mHorizontalBodyWidth = found->second->mHorizontalBodyWidth;
        result.mGuardActive = found->second->mGuardActive;
        result.mGuardZone = PassiveRagdollEntry::guardZoneName(found->second->guardZone());
        {
            const auto balance = found->second->hudBalance();
            result.mBalanceRight = balance[0];
            result.mBalanceForward = balance[1];
            result.mBalanceTip = balance[2];
            result.mAttackReady = found->second->attackReadiness();
        }
        result.mGuardStartPhysicsStep = found->second->mGuardStartPhysicsStep;
        result.mCenterOfMass = Misc::Convert::toOsg(telemetry.mCenterOfMass);
        result.mTotalMass = telemetry.mTotalMass;
        if (found->second->mRagdoll->getSoftRoot().mEnabled)
        {
            const auto& soft = found->second->mRagdoll->getSoftRootTelemetry();
            PassiveRagdollDebugState::SoftRootState softRoot;
            softRoot.mForce = Misc::Convert::toOsg(soft.mForce);
            softRoot.mTorque = Misc::Convert::toOsg(soft.mTorque);
            softRoot.mPositionError = soft.mPositionError;
            softRoot.mAngleError = soft.mAngleError;
            softRoot.mStrength = found->second->mRagdoll->getSoftRootStrength();
            softRoot.mSaturated = soft.mSaturated;
            result.mSoftRoot = softRoot;
        }
        result.mRenderInterpolationFactor = found->second->mRenderInterpolationFactor;
        result.mModeTransitionFactor = found->second->mModeTransitionFactor;
        {
            std::scoped_lock contactLock(found->second->mContactMutex);
            result.mOtherRagdollContacts = found->second->mOtherRagdollContacts;
            for (const auto& contact : RagdollCore::resolveGroundContacts(found->second->mStepContacts, mPhysicsSystem))
                result.mContacts.push_back({ contact.mBodyIndex, Misc::Convert::toOsg(contact.mPosition),
                    Misc::Convert::toOsg(contact.mNormal), contact.mPenetrationDepth, contact.mNormalImpulse, contact.mFriction });
        }
        if (const auto& support = found->second->mSupportEstimate)
        {
            PassiveRagdollSupportState item;
            item.mLeftFootSupported = support->mLeftFootSupported;
            item.mRightFootSupported = support->mRightFootSupported;
            item.mCenterOfMassSupported = support->mCenterOfMassSupported;
            item.mFootContactCount = support->mFootContactCount;
            item.mGroundContactCount = support->mGroundContactCount;
            item.mCenter = Misc::Convert::toOsg(support->mCenter);
            item.mGroundCenter = Misc::Convert::toOsg(support->mGroundCenter);
            item.mCenterOfMassDistance = support->mCenterOfMassDistance;
            item.mCenterOfMassHeight = support->mCenterOfMassHeight;
            item.mFriction = support->mFriction;
            result.mSupport = item;
        }
        if (found->second->mControllerInput && found->second->mControllerOutput)
        {
            PassiveRagdollControllerState item;
            item.mMode = controllerModeName(found->second->mControllerOutput->mMode);
            if (found->second->mControllerOutput->mRecoveryDirection)
            {
                item.mRecoveryDirection = recoveryDirectionName(*found->second->mControllerOutput->mRecoveryDirection);
            }
            item.mTimeInMode = found->second->mControllerOutput->mTimeInMode;
            item.mFallingExitCandidateTime = found->second->mControllerOutput->mFallingExitCandidateTime;
            item.mPelvisUpDot = found->second->mControllerInput->mPelvisUpDot;
            if (found->second->mUprightControllerInput)
            {
                item.mTargetPelvisUpDot = std::clamp(
                    found->second->mUprightControllerInput->mTargetUp.Dot(JPH::Vec3::sAxisZ()), -1.0f, 1.0f);
            }
            item.mTorsoUpDot = found->second->mControllerInput->mTorsoUpDot;
            item.mTorsoFacingUpDot = found->second->mControllerInput->mTorsoFacingUpDot;
            item.mPelvisHeight = found->second->mControllerInput->mPelvisHeight;
            item.mCenterOfMassVelocity = Misc::Convert::toOsg(found->second->mControllerInput->mCenterOfMassVelocity);
            item.mTargetHorizontalVelocity
                = Misc::Convert::toOsg(found->second->mControllerInput->mTargetHorizontalVelocity);
            item.mRawTargetHorizontalVelocity = Misc::Convert::toOsg(found->second->mRawTargetHorizontalVelocity);
            item.mRootExternallyOwned = found->second->mControllerInput->mRootExternallyOwned;
            item.mOwnedRootVerticalVelocity = found->second->mControllerInput->mOwnedRootVerticalVelocity;
            item.mCenterOfMassSpeed = item.mCenterOfMassVelocity.length();
            item.mVelocityTrackingError = (item.mCenterOfMassVelocity - item.mTargetHorizontalVelocity).length();
            item.mDriveJointMotors = found->second->mControllerOutput->mDriveJointMotors;
            item.mJointMotorsApplied = found->second->mJointMotorsApplied;
            item.mJointSupportControlApplied = found->second->mJointSupportControlApplied;
            item.mJointSupportContact = found->second->mJointSupportContact;
            item.mJointSupportMotorUtilization = found->second->mJointSupportMotorUtilization;
            item.mJointMotorPeakUtilization = found->second->mJointMotorPeakUtilization;
            item.mPelvisHeightCandidateTime = found->second->mPelvisHeightOutput.mCandidateTime;
            item.mPelvisHeightCandidateValid = found->second->mPelvisHeightOutput.mCandidateValid;
            item.mPelvisHeightSettled = found->second->mPelvisHeightOutput.mSettled;
            item.mPelvisHeightFrozen = found->second->mPelvisHeightOutput.mFrozen;
            item.mTargetPelvisHeight = found->second->mPelvisHeightOutput.mTargetHeight;
            if (const auto& upright = found->second->mUprightControllerInput)
            {
                item.mUprightTargetPelvisHeight = upright->mTargetPelvisHeight;
                item.mRecoveryUnweight = upright->mRecoveryUnweight;
            }
            item.mRecoveryRequested = found->second->mRecoveryRequested;
            item.mRequestHeadingTravel = found->second->mRequestHeadingTravel;
            item.mCommandHeadingTravel = found->second->mCommandHeadingTravel;
            item.mReferenceYawTravel = found->second->mReferenceYawTravel;
            item.mBasePoseTravel = found->second->mBasePoseTravel;
            item.mReferenceHeightTravel = found->second->mReferenceHeightTravel;
            item.mReferenceGroundTravel = found->second->mReferenceGroundTravel;
            item.mFixedSteps = found->second->mFixedSteps;
            item.mRecoveryAnimationStarted = found->second->mRecoveryAnimationStarted;
            item.mRecoveryAnimationFinished = found->second->mRecoveryAnimationFinished;
            item.mGaitStage = RagdollCore::gaitActuationStageName(RagdollCore::classifyGaitActuationStage(
                !found->second->mTargetHorizontalVelocity.IsNearZero(), found->second->mPelvisHeightOutput.mSettled,
                found->second->mGaitOutput.value_or(RagdollCore::GaitControllerOutput{})));
            item.mGaitActuationStage = RagdollCore::gaitActuationStageName(found->second->mMotorActuationStage);
            item.mStanceHipAxisReleased = found->second->mStanceHipAxisReleased;
            item.mSwingHipAxisReleased = found->second->mSwingHipAxisReleased;
            item.mUprightActuationStage = RagdollCore::gaitActuationStageName(found->second->mUprightActuationStage);
            if (const auto& applied = found->second->mAppliedUprightControllerOutput)
            {
                item.mAppliedPelvisForce = Misc::Convert::toOsg(applied->mPelvisForce);
                item.mAppliedVerticalForceSaturated = applied->mPelvisVerticalForceSaturated;
                item.mAppliedSupportMomentSaturated = applied->mPelvisSupportMomentSaturated;
            }
            if (found->second->mGaitOutput)
            {
                item.mGaitPhase = gaitPhaseName(found->second->mGaitOutput->mPhase);
                item.mGaitTimeInPhase = found->second->mGaitOutput->mTimeInPhase;
                item.mGaitPhaseProgress = found->second->mGaitOutput->mPhaseProgress;
                item.mGaitSwingTimeSinceLift = found->second->mGaitOutput->mSwingTimeSinceLift;
                item.mGaitSwingContactCandidateTime = found->second->mGaitOutput->mSwingContactCandidateTime;
                item.mGaitStanceReady = found->second->mGaitOutput->mStanceReady;
                item.mGaitSwingFootLifted = found->second->mGaitOutput->mSwingFootLifted;
                item.mGaitSwingTouchdownCandidate = found->second->mGaitOutput->mSwingTouchdownCandidate;
                item.mGaitTransitioned = found->second->mGaitOutput->mTransitioned;
                item.mGaitPhaseTimedOut = found->second->mGaitOutput->mPhaseTimedOut;
            }
            if (found->second->mSwingFootTarget)
                item.mSwingFootTarget = Misc::Convert::toOsg(*found->second->mSwingFootTarget);
            item.mBackwardRecoveryStep = found->second->mBackwardRecoveryStep;
            if (found->second->mSwingFootTrajectoryModel)
                item.mSwingFootTrajectoryModel = Misc::Convert::toOsg(*found->second->mSwingFootTrajectoryModel);
            if (found->second->mSwingFootResolvedModel)
                item.mSwingFootResolvedModel = Misc::Convert::toOsg(*found->second->mSwingFootResolvedModel);
            if (found->second->mSwingFootPoseModel)
                item.mSwingFootPoseModel = Misc::Convert::toOsg(*found->second->mSwingFootPoseModel);
            if (found->second->mSwingFootControllerWorldTarget)
            {
                item.mSwingFootControllerWorldTarget
                    = Misc::Convert::toOsg(*found->second->mSwingFootControllerWorldTarget);
            }
            const auto copyMotorTarget = [](const std::optional<RagdollCore::MotorTargetTelemetry>& source,
                                             std::optional<float>& error, std::optional<float>& clamp) {
                if (source)
                {
                    error = source->mAngularError;
                    clamp = source->mTargetClampDelta;
                }
            };
            copyMotorTarget(found->second->mSwingHipMotorTarget, item.mSwingHipMotorError, item.mSwingHipMotorClamp);
            copyMotorTarget(
                found->second->mSwingKneeMotorTarget, item.mSwingKneeMotorError, item.mSwingKneeMotorClamp);
            if (found->second->mSwingKneeMotorTarget)
            {
                item.mSwingKneeCurrentAngle = found->second->mSwingKneeMotorTarget->mCurrentHingeAngle;
                item.mSwingKneeTargetAngle = found->second->mSwingKneeMotorTarget->mTargetHingeAngle;
            }
            copyMotorTarget(
                found->second->mSwingAnkleMotorTarget, item.mSwingAnkleMotorError, item.mSwingAnkleMotorClamp);
            item.mSwingAnkleWorldMotor = found->second->mSwingAnkleMotorTarget
                && found->second->mSwingAnkleMotorTarget->mWorldOrientation;
            item.mSwingHipMotorUtilization = found->second->mSwingHipMotorUtilization;
            item.mSwingKneeMotorUtilization = found->second->mSwingKneeMotorUtilization;
            item.mSwingAnkleMotorUtilization = found->second->mSwingAnkleMotorUtilization;
            if (found->second->mSwingFootControllerOutput)
            {
                item.mSwingFootForce
                    = Misc::Convert::toOsg(found->second->mSwingFootControllerOutput->mForce);
                item.mSwingFootForceSaturated = found->second->mSwingFootControllerOutput->mSaturated;
                item.mSwingFootControlApplied = found->second->mSwingFootControlApplied;
            }
            item.mPreLiftSwingUnloadForce = found->second->mPreLiftSwingUnloadForce;
            if (found->second->mUprightControllerInput && found->second->mUprightControllerOutput)
            {
                item.mPelvisForce = Misc::Convert::toOsg(found->second->mUprightControllerOutput->mPelvisForce);
                item.mPelvisTorque = Misc::Convert::toOsg(found->second->mUprightControllerOutput->mPelvisTorque);
                item.mSpineTorque = Misc::Convert::toOsg(found->second->mUprightControllerOutput->mSpineTorque);
                item.mPelvisForceSaturated = found->second->mUprightControllerOutput->mPelvisForceSaturated;
                item.mPelvisMagnitudeSaturated = found->second->mUprightControllerOutput->mPelvisMagnitudeSaturated;
                item.mPelvisHorizontalForceSaturated
                    = found->second->mUprightControllerOutput->mPelvisHorizontalForceSaturated;
                item.mPelvisVerticalForceSaturated
                    = found->second->mUprightControllerOutput->mPelvisVerticalForceSaturated;
                item.mPelvisTorqueSaturated = found->second->mUprightControllerOutput->mPelvisTorqueSaturated;
                item.mSpineTorqueSaturated = found->second->mUprightControllerOutput->mSpineTorqueSaturated;
                item.mUprightControlApplied = found->second->mUprightControlApplied;
            }
            item.mAnimationAssistedActuation = found->second->mAnimationAssistedActuation;
            item.mLocomotionInterrupted = found->second->mLocomotionInterrupted;
            result.mController = std::move(item);
        }
        result.mConstraints.reserve(telemetry.mConstraints.size());
        for (const RagdollCore::ConstraintTelemetry& constraint : telemetry.mConstraints)
        {
            PassiveRagdollConstraintState item;
            if (debugProfile && constraint.mParentBodyIndex < debugProfile->mBodies.size())
                item.mParentBody = debugProfile->mBodies[constraint.mParentBodyIndex].mName;
            if (debugProfile && constraint.mChildBodyIndex < debugProfile->mBodies.size())
                item.mChildBody = debugProfile->mBodies[constraint.mChildBodyIndex].mName;
            item.mAnchorSeparation = constraint.mAnchorSeparation;
            item.mHingeAngle = constraint.mHingeAngle;
            item.mHingeMinimum = constraint.mHingeMinimum;
            item.mHingeMaximum = constraint.mHingeMaximum;
            result.mConstraints.push_back(std::move(item));
        }
        if (telemetry.mBodies.size() != found->second->mRagdoll->getBodyCount())
            return std::nullopt;
        RagdollCore::RagdollPose diagnosticSkeleton;
        const bool renderTrace = RagdollCore::tuningText("OPENMW_RAGDOLL_RENDER_TRACE") != nullptr;
        const bool skeletonCaptured = found->second->mPoseBinding
            && found->second->mPoseBinding->captureEvaluatedPose(diagnosticSkeleton.mBodyTransforms);
        result.mRenderPhysicsStep = found->second->mRenderDiagnosticPhysicsStep;
        result.mBodies.reserve(telemetry.mBodies.size());
        for (std::size_t i = 0; i < telemetry.mBodies.size(); ++i)
        {
            const RagdollCore::BodyTelemetry& telemetryBody = telemetry.mBodies[i];
            const auto renderPosition = found->second->mRagdoll->getWorldBodyPosition(found->second->mRenderPose, i);
            const auto physicsRenderPosition = found->second->mRagdoll->getWorldBodyPosition(
                found->second->mInterpolatedPhysicsPose, i);
            const RagdollCore::RagdollPose& targetPose = found->second->mPowered
                ? (found->second->mAdaptedMotorTargetCaptured
                    ? found->second->mAdaptedMotorTargetPose : found->second->mMotorTargetPose)
                : found->second->mTransitionSourcePose;
            const auto targetPosition = found->second->mRagdoll->getWorldBodyPosition(targetPose, i);
            if (!found->second->mRenderPosePrepared || !renderPosition || !targetPosition || !physicsRenderPosition)
                return std::nullopt;
            PassiveRagdollBodyState body;
            if (debugProfile && i < debugProfile->mBodies.size())
                body.mName = debugProfile->mBodies[i].mName;
            body.mPosition = Misc::Convert::toOsg(telemetryBody.mPosition);
            body.mCenterOfMass = Misc::Convert::toOsg(telemetryBody.mCenterOfMass);
            body.mTargetPosition = Misc::Convert::toOsg(*targetPosition);
            if (const auto targetRotation = found->second->mRagdoll->getWorldBodyRotation(targetPose, i))
                body.mTargetRotation = osg::Quat(targetRotation->GetX(), targetRotation->GetY(),
                    targetRotation->GetZ(), targetRotation->GetW());
            if (i < found->second->mJointMotorUtilizations.size())
                body.mMotorUtilization = found->second->mJointMotorUtilizations[i];
            body.mRenderPosition = Misc::Convert::toOsg(*renderPosition);
            body.mPhysicsRenderPosition = Misc::Convert::toOsg(*physicsRenderPosition);
            if (skeletonCaptured)
                if (const auto position = found->second->mRagdoll->getWorldBodyPosition(diagnosticSkeleton, i))
                    body.mSkeletonPosition = Misc::Convert::toOsg(*position);
            if (renderTrace
                && i < found->second->mRenderDiagnosticWorldPositions.size())
            {
                static unsigned samples = 0;
                static float maximumError = -1;
                const auto prepared = found->second->mRenderDiagnosticWorldPositions[i];
                const float reframe = JPH::Vec3(*renderPosition - prepared).Length();
                const float error = JPH::Vec3(*renderPosition - telemetryBody.mPosition).Length();
                if (error > maximumError && samples < 80)
                {
                    ++samples;
                    maximumError = error;
                    const auto previous = found->second->mRagdoll->getWorldBodyPosition(found->second->mPreviousPhysicsPose, i);
                    const auto current = found->second->mRagdoll->getWorldBodyPosition(found->second->mCurrentPhysicsPose, i);
                    const auto skeleton = skeletonCaptured
                        ? found->second->mRagdoll->getWorldBodyPosition(diagnosticSkeleton, i) : std::nullopt;
                    Log(Debug::Info) << "RAGDOLL_RENDER_FRAME body=" << body.mName
                        << " physics=" << mJoltStepSerial << " prepared_physics=" << found->second->mRenderDiagnosticPhysicsStep
                        << " reference_delta=" << reframe
                        << " reported_error=" << error
                        << " prepared_error=" << JPH::Vec3(prepared - telemetryBody.mPosition).Length()
                        << " current_error=" << (current ? JPH::Vec3(*current - telemetryBody.mPosition).Length() : -1)
                        << " step_distance=" << (previous && current ? JPH::Vec3(*previous - *current).Length() : -1)
                        << " interpolation=" << found->second->mRenderInterpolationFactor
                        << " interpolation_error=" << (previous && current ? JPH::Vec3(*renderPosition
                            - (*previous + (*current - *previous) * found->second->mRenderInterpolationFactor)).Length() : -1)
                        << " skeleton_error=" << (skeleton ? JPH::Vec3(*skeleton - *renderPosition).Length() : -1);
                }
            }
            body.mRotation = osg::Quat(telemetryBody.mRotation.GetX(), telemetryBody.mRotation.GetY(),
                telemetryBody.mRotation.GetZ(), telemetryBody.mRotation.GetW());
            body.mLinearVelocity = osg::Vec3f(telemetryBody.mMotion.mLinearVelocity.GetX(),
                telemetryBody.mMotion.mLinearVelocity.GetY(), telemetryBody.mMotion.mLinearVelocity.GetZ());
            body.mAngularVelocity = osg::Vec3f(telemetryBody.mMotion.mAngularVelocity.GetX(),
                telemetryBody.mMotion.mAngularVelocity.GetY(), telemetryBody.mMotion.mAngularVelocity.GetZ());
            body.mMass = telemetryBody.mMass;
            body.mActive = telemetryBody.mActive;
            result.mBodies.push_back(std::move(body));
        }
        return result;
    }

    bool PhysicsSystem::isPoweredRagdoll(const MWWorld::ConstPtr& ptr) const
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        return found != mPassiveRagdolls.end() && found->second->mPowered;
    }

    bool PhysicsSystem::hasAttachedRagdollWeapon(const MWWorld::ConstPtr& ptr) const
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        return found != mPassiveRagdolls.end() && found->second->mPowered && found->second->mWeapon
            && found->second->mWeapon->mAttached;
    }

    std::optional<osg::Vec3f> PhysicsSystem::getPassiveRagdollCameraPosition(const MWWorld::ConstPtr& ptr) const
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || mPassiveRagdollProfile == nullptr)
            return std::nullopt;

        const std::string& bodyName = mPassiveRagdollProfile->mLandmarks.mHead
            ? *mPassiveRagdollProfile->mLandmarks.mHead
            : mPassiveRagdollProfile->mLandmarks.mPelvis;
        const auto body = std::find_if(mPassiveRagdollProfile->mBodies.begin(), mPassiveRagdollProfile->mBodies.end(),
            [&](const RagdollCore::BodyProfile& value) { return value.mName == bodyName; });
        if (body == mPassiveRagdollProfile->mBodies.end())
            return std::nullopt;
        const std::size_t bodyIndex
            = static_cast<std::size_t>(std::distance(mPassiveRagdollProfile->mBodies.begin(), body));
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const PassiveRagdollEntry& entry = *found->second;
        if (!entry.mRenderPosePrepared)
            return std::nullopt;
        // The render pose carries the frame's shift to where the actor is
        // drawn (RENDER_REBASE); the camera takes it back out (it follows the
        // physical body, not the actor node).
        const auto shifted = entry.mRagdoll->getWorldBodyPosition(entry.mRenderPose, bodyIndex);
        if (!shifted)
            return std::nullopt;
        const std::optional<JPH::RVec3> position = *shifted - JPH::RVec3(entry.mRenderShift);
        // Active body: while the fighter is on its feet the camera ignores the
        // whole body's sway off its balance target (the pelvis offset), which
        // is exactly the rigid root's camera; a camera riding the soft body
        // read as floating on a harness. It blends to the plain physical head
        // when the body really falls.
        if (entry.mCameraPhysicalBlend < 1.0f && entry.mSoftRootLastTarget)
        {
            if (const auto pelvis = entry.mRagdoll->getWorldBodyPosition(entry.mRenderPose, 0))
            {
                // The balance target between its last two steps, as the pose
                // is (a stepped target moved the camera every other frame
                // above 60 fps).
                // Plus the smoothed absorb's offset (OPENMW_RAGDOLL_STOP_CAMERA):
                // the absorb is taken in one step at a stop.
                const float factor = entry.mRenderInterpolationFactor;
                const JPH::RVec3 target = entry.mSoftRootPrevTarget
                    + (*entry.mSoftRootLastTarget - entry.mSoftRootPrevTarget) * static_cast<double>(factor)
                    + JPH::RVec3(entry.mCameraTargetOffsetPrevious
                        + (entry.mCameraTargetOffset - entry.mCameraTargetOffsetPrevious) * factor);
                const JPH::RVec3 sway = *pelvis - JPH::RVec3(entry.mRenderShift) - target;
                return Misc::Convert::toOsg(
                    *position - sway * static_cast<double>(1.0f - entry.mCameraPhysicalBlend));
            }
        }
        return Misc::Convert::toOsg(*position);
    }

    PassiveRagdollDebugCounters PhysicsSystem::getPassiveRagdollDebugCounters() const
    {
        return { mPassiveRagdolls.size(), mPendingRagdollImpacts.size(), mPassiveRagdollsCreated,
            mPassiveRagdollsRemoved, mPassiveRagdollsEvicted, mPassiveRagdollPoseFailures, mPassiveRagdollMotionSamples,
            mPassiveRagdollMotionTransfers, mWeaponBodiesCreated, mWeaponBodiesDestroyed, mWeaponReleases,
            mWeaponCallbacksArmed, mWeaponCallbacksDisarmed, mWeaponDuplicateReleaseRequests };
    }

    bool PhysicsSystem::applyPassiveRagdollImpulse(const MWWorld::ConstPtr& ptr, std::size_t bodyIndex,
        const osg::Vec3f& impulse, const std::optional<osg::Vec3f>& worldPoint)
    {
        if (!impulse.valid() || (worldPoint && !worldPoint->valid()))
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || bodyIndex >= found->second->mRagdoll->getBodyCount())
            return false;

        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        JPH::BodyInterface& bodyInterface = mPhysicsSystem.GetBodyInterface();
        const JPH::BodyID bodyId = found->second->mRagdoll->getBodyId(bodyIndex);
        const JPH::Vec3 joltImpulse = Misc::Convert::toJolt<JPH::Vec3>(impulse);
        if (worldPoint)
            bodyInterface.AddImpulse(bodyId, joltImpulse, Misc::Convert::toJolt<JPH::RVec3>(*worldPoint));
        else
            bodyInterface.AddImpulse(bodyId, joltImpulse);
        // Probe/spell pushes can soften the struck part like a blow
        // (OPENMW_RAGDOLL_HIT_GIVE_SCRIPTED=1; off so the push tests keep
        // their meaning).
        static const auto scriptedGive_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HIT_GIVE_SCRIPTED");
            return text && std::atoi(text) == 1;
        });
        static const auto& scriptedGive = scriptedGive_tuned.get();
        if (scriptedGive)
        {
            const auto& masses = found->second->mRagdoll->getBodyMasses();
            if (bodyIndex < masses.size())
                found->second->beginHitGive(
                    bodyIndex, impulse.length() / std::max(1e-3f, masses[bodyIndex] * impactPushSetting()[1]));
        }
        if (mPassiveRagdollReplays.contains(ptr.mRef))
        {
            const auto point = worldPoint.value_or(osg::Vec3f());
            Log(Debug::Info) << std::dec << std::setprecision(17)
                << "RAGDOLL_REPLAY_EXTERNAL_IMPULSE schema=1 integer_radix=10"
                << " player=" << (ptr == MWMechanics::getPlayer())
                << " next_step=" << mJoltStepSerial << " body=" << bodyIndex
                << " impulse=" << impulse.x() << ',' << impulse.y() << ',' << impulse.z()
                << " point_present=" << worldPoint.has_value()
                << " point=" << point.x() << ',' << point.y() << ',' << point.z();
        }
        return true;
    }

    std::optional<osg::Vec3f> PhysicsSystem::pickPassiveRagdollAimPoint(const MWWorld::ConstPtr& ptr,
        const osg::Vec3f& rayFrom, const osg::Vec3f& rayDirection, float maxDistance) const
    {
        if (!rayFrom.valid() || !rayDirection.valid() || !std::isfinite(maxDistance) || maxDistance <= 0.0f)
            return std::nullopt;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return std::nullopt;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const JPH::BodyInterface& bodyInterface = mPhysicsSystem.GetBodyInterface();
        const JPH::RVec3 from = Misc::Convert::toJolt<JPH::RVec3>(rayFrom);
        JPH::Vec3 direction = Misc::Convert::toJolt<JPH::Vec3>(rayDirection);
        if (direction.IsNearZero())
            return std::nullopt;
        direction = direction.Normalized();
        std::optional<JPH::RVec3> best;
        float bestDistance = maxDistance;
        for (std::size_t body = 0; body < found->second->mRagdoll->getBodyCount(); ++body)
        {
            const JPH::RVec3 center = bodyInterface.GetCenterOfMassPosition(found->second->mRagdoll->getBodyId(body));
            const JPH::Vec3 offset(center - from);
            const float along = offset.Dot(direction);
            if (along < 0.0f)
                continue;
            const float distance = (offset - direction * along).Length();
            if (distance < bestDistance)
            {
                bestDistance = distance;
                // The point ON THE RAY beside that body: it reads as the spot
                // the attacker aimed at, and the impulse application maps it
                // back to the nearest body with its own lever-arm limit.
                best = from + direction * along;
            }
        }
        if (!best)
            return std::nullopt;
        return osg::Vec3f(static_cast<float>(best->GetX()), static_cast<float>(best->GetY()),
            static_cast<float>(best->GetZ()));
    }

    bool PhysicsSystem::grabPassiveRagdoll(const MWWorld::ConstPtr& ptr, const osg::Vec3f& rayFrom,
        const osg::Vec3f& rayDirection, float maxForce)
    {
        if (!rayFrom.valid() || !rayDirection.valid() || !std::isfinite(maxForce) || maxForce <= 0.0f)
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return false;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        JPH::BodyInterface& bodyInterface = mPhysicsSystem.GetBodyInterface();
        const JPH::RVec3 from = Misc::Convert::toJolt<JPH::RVec3>(rayFrom);
        JPH::Vec3 direction = Misc::Convert::toJolt<JPH::Vec3>(rayDirection);
        if (direction.IsNearZero())
            return false;
        direction = direction.Normalized();
        std::optional<std::size_t> nearest;
        float nearestDistance = 60.0f;
        JPH::RVec3 nearestCenter = JPH::RVec3::sZero();
        for (std::size_t body = 0; body < found->second->mRagdoll->getBodyCount(); ++body)
        {
            const JPH::RVec3 center = bodyInterface.GetCenterOfMassPosition(found->second->mRagdoll->getBodyId(body));
            const JPH::Vec3 offset(center - from);
            const float along = offset.Dot(direction);
            if (along < 0.0f)
                continue;
            const float distance = (offset - direction * along).Length();
            if (distance < nearestDistance)
            {
                nearestDistance = distance;
                nearest = body;
                nearestCenter = center;
            }
        }
        if (!nearest)
            return false;
        found->second->mGrab = PassiveRagdollEntry::Grab{ *nearest, nearestCenter, maxForce };
        return true;
    }

    bool PhysicsSystem::updatePassiveRagdollGrab(const MWWorld::ConstPtr& ptr, const osg::Vec3f& anchor)
    {
        if (!anchor.valid())
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mGrab)
            return false;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        found->second->mGrab->mAnchor = Misc::Convert::toJolt<JPH::RVec3>(anchor);
        return true;
    }

    bool PhysicsSystem::setPassiveRagdollArmTarget(const MWWorld::ConstPtr& ptr, bool rightArm,
        const osg::Vec3f& worldTarget, float weight, float blendRate)
    {
        if (!worldTarget.valid() || !std::isfinite(weight) || !std::isfinite(blendRate) || blendRate < 0.0f)
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return false;
        auto& entry = *found->second;
        if (!(rightArm ? entry.mRightArm : entry.mLeftArm))
            return false;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        auto& request = rightArm ? entry.mRightArmTarget : entry.mLeftArmTarget;
        if (!request)
            request.emplace();
        request->mWorldTarget = Misc::Convert::toJolt<JPH::RVec3>(worldTarget);
        request->mRequestedWeight = std::clamp(weight, 0.0f, 1.0f);
        request->mBlendRate = blendRate;
        request->mReleasing = false;
        request->mBrace = false;
        request->mWeaponSupport = false;
        request->mHandRotation.reset();
        // Elbows hang away from the body. Without a bend preference the
        // solver is free to choose the inside-out arm that reaches the same
        // point, which is anatomically valid and looks broken.
        request->mWorldBend = JPH::Vec3(0.0f, 0.0f, -1.0f);
        return true;
    }

    bool PhysicsSystem::setPassiveRagdollWeaponFixtureYaw(const MWWorld::ConstPtr& ptr, float angle)
    {
        if (!std::isfinite(angle))
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mWeapon || !found->second->mRightArm)
            return false;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        found->second->mWeaponFixtureYaw = angle;
        return true;
    }

    void PhysicsSystem::releasePassiveRagdollArmTarget(const MWWorld::ConstPtr& ptr, bool rightArm)
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        auto& request = rightArm ? found->second->mRightArmTarget : found->second->mLeftArmTarget;
        if (request)
            request->mReleasing = true;
    }

    bool PhysicsSystem::drivePassiveRagdollGuardZone(
        const MWWorld::ConstPtr& ptr, float countsRight, float countsUp, bool reset)
    {
        if (!std::isfinite(countsRight) || !std::isfinite(countsUp))
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return false;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        found->second->moveGuardZone(countsRight, countsUp, reset);
        return true;
    }

    bool PhysicsSystem::setPassiveRagdollGuard(const MWWorld::ConstPtr& ptr, bool active,
        const std::optional<osg::Vec3f>& worldFacing)
    {
        if (worldFacing && !worldFacing->valid())
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return false;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        auto& entry = *found->second;
        // A guard broken in a clash cannot come back up yet (M7).
        if (entry.mClashLockout > 0.0f)
            active = false;
        if (active && !entry.mGuardActive)
            entry.mGuardStartPhysicsStep = mJoltStepSerial;
        if (active)
        {
            const osg::Vec3f requested = worldFacing.value_or(actorLookDirection(ptr));
            JPH::Vec3 forward(requested.x(), requested.y(), 0.0f);
            if (forward.LengthSq() <= 1e-6f)
                return false;
            entry.mGuardForward = forward.Normalized();
        }
        entry.mGuardActive = active;
        return true;
    }

    bool PhysicsSystem::isPassiveRagdollHandControlActive(const MWWorld::ConstPtr& ptr) const
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        return found != mPassiveRagdolls.end() && found->second->mHscActive;
    }

    std::optional<float> PhysicsSystem::getPassiveRagdollArmReach(
        const MWWorld::ConstPtr& ptr, bool rightArm) const
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return std::nullopt;
        const auto& request = rightArm ? found->second->mRightArmTarget : found->second->mLeftArmTarget;
        static unsigned armTraceSamples = 0;
        if (RagdollCore::tuningText("OPENMW_RAGDOLL_ARM_TRACE") && armTraceSamples++ < 64)
        {
            const auto& arm = rightArm ? found->second->mRightArm : found->second->mLeftArm;
            if (arm)
            {
                const auto shoulder = found->second->mRagdoll->getWorldBodyPosition(
                    found->second->mCurrentPhysicsPose, arm->mShoulder);
                if (shoulder && request && request->mApplied)
                {
                    const JPH::Vec3 offset(request->mWorldTarget - *shoulder);
                    Log(Debug::Info) << "ARM_TARGET right=" << rightArm << " offset=" << offset.GetX()
                        << ',' << offset.GetY() << ',' << offset.GetZ() << " reach_error=" << request->mReachError;
                }
                for (const auto body : { arm->mFrame, arm->mShoulder, arm->mElbow, arm->mHand })
                {
                    RagdollCore::MotorTargetTelemetry motor;
                    if (found->second->mRagdoll->captureMotorTargetTelemetry(body, motor))
                        Log(Debug::Info) << "ARM_MOTOR body=" << body << " clamp=" << motor.mTargetClampDelta
                            << " error=" << motor.mAngularError << " hinge=" << motor.mCurrentHingeAngle.value_or(0)
                            << " target=" << motor.mTargetHingeAngle.value_or(0)
                            << " has_orientation=" << motor.mTargetOrientation.has_value()
                            << " orientation=" << motor.mTargetOrientation.value_or(JPH::Quat::sIdentity()).GetX() << ','
                            << motor.mTargetOrientation.value_or(JPH::Quat::sIdentity()).GetY() << ','
                            << motor.mTargetOrientation.value_or(JPH::Quat::sIdentity()).GetZ() << ','
                            << motor.mTargetOrientation.value_or(JPH::Quat::sIdentity()).GetW()
                            << " target_speed=" << motor.mTargetAngularVelocity.Length()
                            << " spring=" << motor.mSpring.mFrequencyOrStiffness
                            << " damping=" << motor.mSpring.mDamping
                            << " stiffness_mode=" << motor.mSpring.mStiffnessMode
                            << " arm_target_active=" << bool(request && request->mApplied);
                    JPH::Vec3 impulse;
                    float utilization;
                    if (found->second->mRagdoll->getMotorAngularImpulse(body, mPhysicsDt, impulse, utilization))
                        Log(Debug::Info) << "ARM_LOAD body=" << body << " step=" << mJoltStepSerial
                            << " impulse=" << impulse.GetX() << ',' << impulse.GetY() << ',' << impulse.GetZ()
                            << " utilization=" << utilization;
                }
            }
        }
        return request && request->mApplied ? std::optional<float>(request->mReachError) : std::nullopt;
    }

    std::optional<float> PhysicsSystem::driveRagdollWeaponSwing(const MWWorld::ConstPtr& ptr, bool rightArm,
        const osg::Vec3f& aimRight, const osg::Vec3f& aimForward, const osg::Vec3f& aimUp, float deltaRight,
        float deltaUp, bool held, float duration, bool playerInput, bool thrust)
    {
        if (!aimRight.valid() || !aimForward.valid() || !aimUp.valid() || !std::isfinite(deltaRight)
            || !std::isfinite(deltaUp) || !std::isfinite(duration) || duration <= 0.0f)
            return std::nullopt;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return std::nullopt;
        auto& entry = *found->second;
        const bool native = playerInput && ptr == MWMechanics::getPlayer();
        const bool bare = !entry.mWeapon || !entry.mWeapon->mAttached;
        const bool pressed = native && held && !entry.mNativeStrikeHeld;
        if (native && bare && thrust)
            rightArm = entry.mSwingArmIsRight; // fixed-step punch sequence owns alternation
        const auto& arm = rightArm ? entry.mRightArm : entry.mLeftArm;
        if (!arm || !entry.mCurrentPhysicsPoseCaptured || !entry.mPowered)
            return std::nullopt;

        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        if (entry.mAnimationAttack.mPhase != MWMechanics::RagdollAttackPhase::Idle
            || entry.mAnimationAttackMotors)
        {
            entry.mAnimationAttack.mPhase = MWMechanics::RagdollAttackPhase::Idle;
            if (entry.mAnimationAttackMotors && entry.mRightArm && mPassiveRagdollProfile)
                setArmSwingMotors(*entry.mRagdoll,
                    entry.mRuntimeProfile ? *entry.mRuntimeProfile : *mPassiveRagdollProfile,
                    *entry.mRightArm, mRagdollWeaponSwing, false);
            entry.mAnimationAttackMotors = false;
            entry.mSwing = {};
        }
        if (native)
        {
            entry.mNativeStrikeInput = true;
            if (pressed)
            {
                ++entry.mPlayerPressCount;
                if (bare && thrust) entry.mPunchCycle.mPendingPress = true;
                entry.mSwingHandRotation.reset();
                entry.mClickExtensionRemaining = bare || thrust ? 0.16f : 0;
                // Every press starts from this hand's measured pose. It must
                // never inherit the other arm's unfinished mouse trajectory.
                if (!bare || !thrust)
                {
                    entry.mSwing = {};
                    entry.mSwingInput = {};
                    entry.mSwingRestOffset.reset();
                    entry.mSwingExtension = 0;
                    entry.mPlayerAttackType.clear();
                }
            }
            if (held && thrust != entry.mNativeThrust) entry.mPlayerAttackType.clear();
            if (pressed)
                entry.mNativeStrikePressLatch = true;
            entry.mNativeStrikeHeld = held;
            // Keep orientation through withdrawal after a released thrust.
            if (held) entry.mNativeThrust = thrust;
        }
        else entry.mNativeStrikeInput = false;
        // An explicit physical gesture is an independent input route. Its
        // contact-work rules remain available after cancelling native charge.
        entry.mAnimationAttackOwned = false;
        entry.mAnimationPlaybackComplete = false;
        // Switching hands mid-swing would carry one arm's arc onto the other.
        if (entry.mSwingArmIsRight != rightArm)
        {
            const auto& previousArm = entry.mSwingArmIsRight ? entry.mRightArm : entry.mLeftArm;
            if (previousArm && mPassiveRagdollProfile)
                setArmSwingMotors(*entry.mRagdoll,
                    entry.mRuntimeProfile ? *entry.mRuntimeProfile : *mPassiveRagdollProfile,
                    *previousArm, mRagdollWeaponSwing, false);
            entry.mSwingMotorsActive = false;
            entry.mSwing = RagdollCore::WeaponSwingState{};
            entry.mSwingArmIsRight = rightArm;
        }
        // Keep held motion directional, including bare hands. Stability is
        // enforced by the bounded physical target, not by discarding input.
        entry.mSwingInput.mAimRight += deltaRight;
        entry.mSwingInput.mAimUp += deltaUp;
        entry.mSwingInput.mHeld = held;
        entry.mSwingRight = Misc::Convert::toJolt<JPH::Vec3>(aimRight);
        entry.mSwingForward = Misc::Convert::toJolt<JPH::Vec3>(aimForward);
        entry.mSwingUp = Misc::Convert::toJolt<JPH::Vec3>(aimUp);
        entry.mSwingInputActive = true;
        return entry.mSwing.mVelocity.Length();
    }

    void PhysicsSystem::setRagdollAttackAnimation(
        const MWWorld::ConstPtr& ptr, const MWMechanics::RagdollAttackState& state)
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        auto& entry = *found->second;
        using Phase = MWMechanics::RagdollAttackPhase;
        const bool animationAttackMode = entry.mControllerOutput
            && RagdollCore::isDrivenControllerMode(entry.mControllerOutput->mMode);
        if (state.mPhase == Phase::Idle)
        {
            // An unconditional cancel must not erase a subsequent explicit
            // thrust. Only the animation-owned state is released here.
            if (state.mAttackId == 0 || state.mAttackId == entry.mAnimationAttack.mAttackId)
            {
                entry.mAnimationPlaybackComplete = state.mPlaybackComplete
                    && (entry.mAnimationAttack.mPhase == Phase::FollowThrough || entry.mAnimationPlaybackComplete);
                entry.mAnimationAttack.mPhase = Phase::Idle;
            }
            return;
        }
        const bool matchingWeapon = entry.mWeapon && entry.mWeapon->mAttached
            ? entry.mWeapon->mRecordId == state.mWeaponRecordId : state.mWeaponRecordId.empty();
        if (!entry.mPowered || !entry.mRightArm || !matchingWeapon || state.mAttackId == 0
            || !std::isfinite(state.mStrength) || !animationAttackMode)
        {
            entry.mAnimationPlaybackComplete = false;
            entry.mAnimationAttack.mPhase = Phase::Idle;
            return;
        }
        if (state.mAttackId != entry.mAnimationAttack.mAttackId)
        {
            if (state.mPhase != Phase::WindUp)
                return;
            entry.mAnimationPlaybackComplete = false;
            ++entry.mSwingIdentity;
            ++entry.mPlayerPressCount;
            entry.mSwing = {};
            entry.mSwingInput = {};
            entry.mSwingInputActive = false;
            entry.mNativeStrikeInput = false;
            entry.mNativeStrikeHeld = false;
            entry.mClickExtensionRemaining = 0;
            entry.mPunchCycle = {};
            entry.mSwingHandRotation.reset();
            entry.mSwingRestOffset.reset();
            entry.mSwingExtension = 0;
            entry.mSwingMotorsActive = false;
        }
        else if (entry.mAnimationAttack.mPhase == Phase::Idle)
            return; // A cancelled identity cannot be revived by a late text key.
        entry.mAnimationAttack = state;
        entry.mAnimationAttackOwned = true;
        entry.mAnimationAttack.mStrength = std::clamp(state.mStrength, 0.0f, 1.0f);
    }

    void PhysicsSystem::setRagdollSwingAimRay(
        const MWWorld::ConstPtr& ptr, const osg::Vec3f& origin, const osg::Vec3f& direction)
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        if (!origin.valid() || !direction.valid() || direction.length2() < 1e-6f)
        {
            found->second->mSwingAimRayOrigin.reset();
            return;
        }
        found->second->mSwingAimRayOrigin = Misc::Convert::toJolt<JPH::RVec3>(origin);
        found->second->mSwingAimRayDirection = Misc::Convert::toJolt<JPH::Vec3>(direction).Normalized();
    }

    std::vector<std::pair<std::string, float>> PhysicsSystem::getRagdollLimbDamage(const MWWorld::ConstPtr& ptr) const
    {
        std::vector<std::pair<std::string, float>> result;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return result;
        const auto& ledger = found->second->mRegionDamage;
        for (std::size_t index = 0; index < ledger.size(); ++index)
            if (ledger[index] > 0.0f)
                result.emplace_back(
                    RagdollCore::regionName(static_cast<RagdollCore::AnatomicalRegion>(index)), ledger[index]);
        return result;
    }

    std::pair<float, float> PhysicsSystem::takePlayerCameraKick()
    {
        static const auto degrees_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CAMERA_KICK");
            return text ? static_cast<float>(std::atof(text)) : 0.0f;
        });
        static const auto& degrees = degrees_tuned.get();
        if (!(degrees > 0.0f))
            return { 0.0f, 0.0f };
        const MWWorld::Ptr player = MWMechanics::getPlayer();
        if (player.isEmpty())
            return { 0.0f, 0.0f };
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(player.mRef);
        if (found == mPassiveRagdolls.end())
            return { 0.0f, 0.0f };
        const float kick = std::clamp(found->second->mCameraKick, 0.0f, 1.0f);
        const float roll = std::clamp(found->second->mCameraRoll, -1.0f, 1.0f);
        found->second->mCameraKick = 0.0f;
        found->second->mCameraRoll = 0.0f;
        static const auto trace_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_CAMERA_KICK_TRACE") != nullptr; });
        static const auto& trace = trace_tuned.get();
        if (trace && (kick > 0.0f || roll != 0.0f))
            Log(Debug::Info) << "RAGDOLL_CAMERA_KICK nod=" << kick << " roll=" << roll;
        return { osg::DegreesToRadians(degrees) * kick, osg::DegreesToRadians(degrees) * roll };
    }

    void PhysicsSystem::addRagdollLimbDamage(const MWWorld::ConstPtr& ptr, std::string_view region, float damage)
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return;
        for (std::size_t index = 1; index < found->second->mRegionDamage.size(); ++index)
        {
            const auto value = static_cast<RagdollCore::AnatomicalRegion>(index);
            if (RagdollCore::regionName(value) == region)
                found->second->addRegionDamage(value, damage);
        }
    }

    void PhysicsSystem::clearRagdollLimbDamage()
    {
        for (auto& [_, entry] : mPassiveRagdolls)
            entry->clearRegionDamage();
    }

    void PhysicsSystem::setRagdollSwingStrength(float scale)
    {
        if (std::isfinite(scale) && scale >= 1.0f)
            mRagdollWeaponSwing.mMotorTorqueScale = scale;
    }

    float PhysicsSystem::getRagdollSwingStrength() const
    {
        return mRagdollWeaponSwing.mMotorTorqueScale;
    }

    std::optional<float> PhysicsSystem::getRagdollWeaponSwingSpeed(const MWWorld::ConstPtr& ptr) const
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end()
            || (!found->second->mSwing.mSwinging && !found->second->hasPhysicalFollowThrough()))
            return std::nullopt;
        return found->second->mSwing.mVelocity.Length();
    }

    std::optional<float> PhysicsSystem::getPassiveRagdollHeadingError(const MWWorld::ConstPtr& ptr) const
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mControllerObservationInitialized)
            return std::nullopt;
        return found->second->mHeadingError;
    }

    void PhysicsSystem::setRagdollRecoveryDirection(std::optional<bool> faceUp)
    {
        sForcedRecoveryDirection = faceUp
            ? std::optional<RagdollCore::RecoveryDirection>(
                  *faceUp ? RagdollCore::RecoveryDirection::FaceUp : RagdollCore::RecoveryDirection::FaceDown)
            : std::nullopt;
    }

    void PhysicsSystem::setRagdollBracingArms(bool bracing)
    {
        if (mPassiveRagdollProfile)
            mPassiveRagdollProfile->mActiveController.mBracingArms = bracing;
    }

    bool PhysicsSystem::getRagdollBracingArms() const
    {
        return mPassiveRagdollProfile && mPassiveRagdollProfile->mActiveController.mBracingArms;
    }

    std::optional<PhysicsSystem::RagdollArmDebug> PhysicsSystem::getPassiveRagdollArmDebug(
        const MWWorld::ConstPtr& ptr, bool rightArm) const
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return std::nullopt;
        const auto& request = rightArm ? found->second->mRightArmTarget : found->second->mLeftArmTarget;
        if (!request)
            return std::nullopt;
        RagdollArmDebug debug;
        debug.mReachError = request->mReachError;
        debug.mWeight = request->mWeight;
        debug.mApplied = request->mApplied;
        debug.mBrace = request->mBrace;
        debug.mTargetWorld = Misc::Convert::toOsg(request->mWorldTarget);
        debug.mReachWithoutShoulderModel = request->mReachWithoutShoulder;
        const auto& entry = *found->second;
        const auto& arm = rightArm ? entry.mRightArm : entry.mLeftArm;
        if (arm && entry.mCurrentPhysicsPoseCaptured)
        {
            const auto shoulder = entry.mRagdoll->getWorldBodyPosition(entry.mCurrentPhysicsPose, arm->mShoulder);
            const auto hand = entry.mRagdoll->getWorldBodyPosition(entry.mCurrentPhysicsPose, arm->mHand);
            if (hand)
                debug.mMeasuredErrorWorld = static_cast<float>((request->mWorldTarget - *hand).Length());
            if (shoulder)
            {
                const auto offset = entry.mRagdoll->getModelVector(JPH::Vec3(request->mWorldTarget - *shoulder));
                if (offset)
                    debug.mTargetDistanceModel = offset->Length();
            }
            const auto geometry = entry.mRagdoll->getHingeLegGeometry(arm->mShoulder, arm->mElbow, arm->mHand);
            const auto range = geometry ? RagdollCore::hingeChainReachRange(*geometry) : std::nullopt;
            if (range)
            {
                debug.mMinimumReachModel = range->first;
                debug.mMaximumReachModel = range->second;
            }
        }
        return debug;
    }

    void PhysicsSystem::setAutomaticRagdollRecovery(bool automatic)
    {
        if (mPassiveRagdollProfile)
            mPassiveRagdollProfile->mActiveController.mAutomaticRecovery = automatic;
    }

    bool PhysicsSystem::getAutomaticRagdollRecovery() const
    {
        return mPassiveRagdollProfile && mPassiveRagdollProfile->mActiveController.mAutomaticRecovery;
    }

    void PhysicsSystem::releasePassiveRagdollGrab(const MWWorld::ConstPtr& ptr)
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        found->second->mGrab.reset();
    }

    // REACT-02: a blow delivered by a test straight to the reaction entry
    // point (applyBlowReaction), as if `mass` units of weapon struck the
    // named body at `speed` u/s along `direction` and paid `damage`; the
    // reaction impulse is priced the way combat.cpp prices a weapon blow
    // (damage x reaction per damage x the region's reaction scale).
    namespace
    {
        // SHOVE-01 (user 10-29: a shove on a double right click; then "make
        // shove push much less, make the force depend on how many times the
        // player clicked the RMB quickly ... drive the player forward a bit and
        // take a stamina hit, make all of these scale with stats").
        // OPENMW_RAGDOLL_SHOVE = "on, reach u, cone deg, speed u/s, mass kg,
        // hands s, cooldown s, fatigue, lunge s, power per click, extra clicks":
        // the body shoved is the nearest one whose centre of mass is within
        // `reach` (flat) and `cone` of the facing; the push is a blow on its
        // chest of `speed` x `mass` x Strength / 50 (0.6-1.4) x (1 + `power
        // per click` for each rapid right click after the double click, up to
        // `extra clicks`), x 0.6 into a raised guard. It lands when a hand
        // gets to the chest (not before 0.15 s, so the clicks count), at most
        // `hands` s after. The shover lunges forward (its own walk) for `lunge`
        // x (0.6 + Agility / 100) s (0.15-0.5), 0.08 s more a click; it costs
        // `fatigue` x (1.4 - Endurance / 100) (0.6-1.4), again for each click.
        struct ShoveSettings
        {
            bool mOn = true;
            float mReach = 75.0f, mCone = 50.0f, mSpeed = 110.0f, mMass = 12.0f;
            float mHands = 0.35f, mCooldown = 0.8f, mFatigue = 10.0f;
            float mLunge = 0.25f, mPowerPerClick = 0.5f;
            int mExtraClicks = 2;
        };
        const ShoveSettings& shoveSettings()
        {
            static const auto value_tuned = RagdollCore::tuned<ShoveSettings>([] {
                ShoveSettings value;
                if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SHOVE"))
                {
                    float on = 1.0f, extra = 2.0f;
                    std::sscanf(text, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &on, &value.mReach, &value.mCone,
                        &value.mSpeed, &value.mMass, &value.mHands, &value.mCooldown, &value.mFatigue, &value.mLunge,
                        &value.mPowerPerClick, &extra);
                    value.mOn = on > 0.0f;
                    value.mExtraClicks = std::clamp(static_cast<int>(extra), 0, 5);
                }
                return value;
            });
            return value_tuned.get();
        }

        std::optional<std::uint32_t> chestBody(const PassiveRagdollEntry& entry)
        {
            for (std::size_t i = 0; i < entry.mBodyRegions.size(); ++i)
                if (entry.mBodyRegions[i] == RagdollCore::AnatomicalRegion::Chest && !entry.isDetachedWeaponBody(i)
                    && i < entry.mRagdoll->getBodyCount())
                    return static_cast<std::uint32_t>(i);
            return std::nullopt;
        }
    }

    bool PhysicsSystem::shovePassiveRagdoll(const MWWorld::ConstPtr& ptr)
    {
        const ShoveSettings& s = shoveSettings();
        if (!s.mOn)
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return false;
        PassiveRagdollEntry& entry = *found->second;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const MWWorld::Ptr actor = entry.getPtr();
        auto& stats = actor.getClass().getCreatureStats(actor);
        const auto attribute = [&](ESM::RefId id) { return stats.getAttribute(id).getModified(); };
        // The stamina a push costs: less with Endurance.
        const float cost = s.mFatigue * std::clamp(1.4f - attribute(ESM::Attribute::Endurance) / 100.0f, 0.6f, 1.4f);
        const auto spend = [&] {
            if (!(cost > 0.0f))
                return;
            auto fatigue = stats.getFatigue();
            fatigue.setCurrent(fatigue.getCurrent() - cost);
            stats.setFatigue(fatigue);
        };
        // A rapid click while the hands are on their way: more power.
        for (PendingShove& pending : mPendingShoves)
            if (pending.mShover == ptr.mRef)
            {
                if (pending.mExtraClicks >= s.mExtraClicks)
                    return true;
                ++pending.mExtraClicks;
                spend();
                entry.mSoftRootStumbleTime += 0.08f;
                Log(Debug::Info) << "RAGDOLL_SHOVE_POWER actor=" << ptr.getCellRef().getRefId()
                                 << " clicks=" << pending.mExtraClicks << " power="
                                 << 1.0f + s.mPowerPerClick * static_cast<float>(pending.mExtraClicks);
                return true;
            }
        if (!entry.mPowered || !entry.mSituationKnown || entry.mShoveCooldown > 0.0f || entry.mShoveArms > 0.0f)
            return false;
        const JPH::Vec3 facing = entry.reactionForward();
        const float cone = std::cos(s.mCone * 3.14159265f / 180.0f);
        const MWWorld::LiveCellRefBase* best = nullptr;
        float nearest = s.mReach;
        for (const auto& [ref, other] : mPassiveRagdolls)
        {
            if (other.get() == &entry || !other->mPowered || !other->mSituationKnown)
                continue;
            const JPH::Vec3 d(other->mSituationCom - entry.mSituationCom);
            const float flat = std::hypot(d.GetX(), d.GetY());
            if (flat < 1.0f || flat > nearest || std::abs(d.GetZ()) > 60.0f
                || (d.GetX() * facing.GetX() + d.GetY() * facing.GetY()) / flat < cone)
                continue;
            nearest = flat;
            best = ref;
        }
        if (!best)
        {
            Log(Debug::Info) << "RAGDOLL_SHOVE actor=" << ptr.getCellRef().getRefId() << " target=- (nobody in reach)";
            return false;
        }
        const float strength = std::clamp(attribute(ESM::Attribute::Strength) / 50.0f, 0.6f, 1.4f);
        spend();
        // The lunge: the shover's own walk carries it forward (the stumble
        // path a shoved body steps with), longer with Agility.
        if (s.mLunge > 0.0f)
        {
            entry.mSoftRootStumbleDirection = entry.reactionForward();
            entry.mSoftRootStumbleTime = std::max(entry.mSoftRootStumbleTime,
                std::clamp(s.mLunge * (0.6f + attribute(ESM::Attribute::Agility) / 100.0f), 0.15f, 0.5f));
        }
        entry.mShoveArms = s.mHands + 0.3f;
        entry.mShoveCooldown = s.mCooldown;
        entry.mShoveTarget = best;
        entry.mShoveAt.reset();
        mPendingShoves.push_back({ ptr.mRef, best, s.mHands, strength, 0 });
        Log(Debug::Info) << "RAGDOLL_SHOVE actor=" << ptr.getCellRef().getRefId() << " target="
                         << mPassiveRagdolls.at(best)->getPtr().getCellRef().getRefId() << " distance=" << nearest
                         << " strength=" << strength;
        return true;
    }

    std::vector<std::pair<MWWorld::Ptr, std::optional<osg::Matrixf>>> PhysicsSystem::getWeaponGripTurns()
    {
        std::vector<std::pair<MWWorld::Ptr, std::optional<osg::Matrixf>>> out;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        for (const auto& [ref, entry] : mPassiveRagdolls)
        {
            const bool turned = entry->mMurder.has_value() && entry->mWeapon;
            if (!turned && !entry->mGripTurnShown)
                continue;
            entry->mGripTurnShown = turned;
            if (!turned)
            {
                out.emplace_back(entry->getPtr(), std::nullopt);
                continue;
            }
            // The weapon as drawn: the hold point to the hand, turned half
            // round about the blade's flat (as reconcileMurderGrip holds it).
            const auto& murder = *entry->mMurder;
            const osg::Vec3f hold = Misc::Convert::toOsg(murder.mBladeAxis * murder.mHold);
            const JPH::Quat& q = entry->mMurderTurn;
            const osg::Quat turn(q.GetX(), q.GetY(), q.GetZ(), q.GetW());
            out.emplace_back(entry->getPtr(), osg::Matrixf::translate(-hold) * osg::Matrixf::rotate(turn));
        }
        return out;
    }

    std::vector<std::pair<const void*, std::pair<osg::Vec3f, osg::Vec3f>>> PhysicsSystem::getWeaponBlades()
    {
        std::vector<std::pair<const void*, std::pair<osg::Vec3f, osg::Vec3f>>> out;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        JPH::BodyInterface& bodies = mPhysicsSystem.GetBodyInterface();
        for (const auto& [ref, entry] : mPassiveRagdolls)
        {
            if (!entry->mPowered || !entry->mWeapon || !entry->mWeapon->mAttached || !entry->mRagdoll
                || entry->mWeapon->mHandBody >= entry->mRagdoll->getBodyCount()
                // A sheathed weapon's body stays on the hand, unseen: no trail
                // (user 10-30: "when the weapon is sheathed it still shows the
                // weapon whoosh visual").
                || entry->mContactWeaponSheathed.load(std::memory_order_relaxed))
                continue;
            // As the sparks: the hand and the blade axis it holds - from the
            // pose the frame draws (between the last two steps), not the
            // newest step, which ran ahead of the drawn blade.
            const JPH::BodyID hand = entry->mRagdoll->getBodyId(entry->mWeapon->mHandBody);
            std::optional<JPH::RVec3> drawnPos;
            std::optional<JPH::Quat> drawnTurn;
            if (entry->mRenderPosePrepared)
            {
                drawnPos = entry->mRagdoll->getWorldBodyPosition(entry->mRenderPose, entry->mWeapon->mHandBody);
                drawnTurn = entry->mRagdoll->getWorldBodyRotation(entry->mRenderPose, entry->mWeapon->mHandBody);
            }
            const JPH::RVec3 handPos = drawnPos ? *drawnPos : bodies.GetCenterOfMassPosition(hand);
            const JPH::Vec3 blade = ((drawnTurn ? *drawnTurn : bodies.GetRotation(hand)) * entry->mWeapon->mBladeAxis)
                                        .NormalizedOr(JPH::Vec3::sAxisY());
            const float reach = std::max(10.0f, entry->mWeapon->tipDistanceFromHand());
            const JPH::RVec3 tip = handPos + JPH::RVec3(blade * reach);
            const JPH::RVec3 base = tip - JPH::RVec3(blade * (0.4f * entry->mWeapon->mLength));
            out.push_back({ ref, { Misc::Convert::toOsg(base), Misc::Convert::toOsg(tip) } });
        }
        return out;
    }

    void PhysicsSystem::stepPendingShoves(float dt)
    {
        // The shoving hands follow the chest they go to.
        for (auto& [ref, entry] : mPassiveRagdolls)
        {
            entry->mShoveCooldown = std::max(0.0f, entry->mShoveCooldown - dt);
            if (entry->mShoveArms <= 0.0f)
                continue;
            entry->mShoveArms = std::max(0.0f, entry->mShoveArms - dt);
            const auto target
                = mPassiveRagdolls.find(static_cast<const MWWorld::LiveCellRefBase*>(entry->mShoveTarget));
            if (entry->mShoveArms <= 0.0f || target == mPassiveRagdolls.end() || !target->second->mSituationKnown)
            {
                entry->mShoveAt.reset();
                continue;
            }
            const JPH::Vec3 d(target->second->mSituationCom - entry->mSituationCom);
            const float flat = std::hypot(d.GetX(), d.GetY());
            if (flat < 1.0f)
                continue;
            const JPH::Vec3 toward(-d.GetX() / flat, -d.GetY() / flat, 0.0f);
            entry->mShoveAt = std::make_pair(
                target->second->mSituationCom + JPH::RVec3(toward * 14.0f) + JPH::RVec3(0.0, 0.0, 22.0), toward);
        }
        const ShoveSettings& s = shoveSettings();
        for (auto it = mPendingShoves.begin(); it != mPendingShoves.end();)
        {
            it->mLeft -= dt;
            const auto shover = mPassiveRagdolls.find(it->mShover);
            const auto target = mPassiveRagdolls.find(it->mTarget);
            if (shover == mPassiveRagdolls.end() || target == mPassiveRagdolls.end())
            {
                it = mPendingShoves.erase(it);
                continue;
            }
            // The push lands as a hand gets to the chest (or when the time is up).
            const std::optional<float> handGap = shover->second->shoveHandGap();
            if (it->mLeft > 0.0f && !(handGap && *handGap < 14.0f && s.mHands - it->mLeft >= 0.15f))
            {
                ++it;
                continue;
            }
            const PendingShove shove = *it;
            it = mPendingShoves.erase(it);
            PassiveRagdollEntry& from = *shover->second;
            PassiveRagdollEntry& victim = *target->second;
            const JPH::Vec3 d(victim.mSituationCom - from.mSituationCom);
            const float flat = std::hypot(d.GetX(), d.GetY());
            const auto chest = chestBody(victim);
            // Out of reach by the time the hands get there (it stepped away).
            if (flat < 1.0f || flat > s.mReach + 20.0f || !victim.mPowered || !chest)
            {
                Log(Debug::Info) << "RAGDOLL_SHOVE_MISS actor=" << from.getPtr().getCellRef().getRefId()
                                 << " distance=" << flat;
                continue;
            }
            // Flat along the line between the bodies, a little up.
            const JPH::Vec3 direction = JPH::Vec3(d.GetX() / flat, d.GetY() / flat, 0.15f).Normalized();
            RagdollCore::Blow blow;
            blow.mId = RagdollCore::nextBlowId();
            blow.mPhysicsStep = mJoltStepSerial;
            blow.mVictim = victim.mSerial;
            // No weapon: a push of the hands, no wound.
            blow.mKind = RagdollCore::BlowKind::Scripted;
            blow.mRegion = RagdollCore::AnatomicalRegion::Chest;
            blow.mBodyIndex = *chest;
            blow.mDirection = direction;
            const JPH::RVec3 centre
                = mPhysicsSystem.GetBodyInterface().GetCenterOfMassPosition(victim.mRagdoll->getBodyId(*chest));
            blow.mPoint = centre - JPH::RVec3(direction * 6.0f);
            blow.mAttackerToVictim = JPH::Vec3(direction.GetX(), direction.GetY(), 0.0f).Normalized();
            // A raised guard takes some of it on the arms.
            const float power = 1.0f + s.mPowerPerClick * static_cast<float>(shove.mExtraClicks);
            blow.mApproachSpeed = s.mSpeed * shove.mStrength * power * (victim.mGuardActive ? 0.6f : 1.0f);
            blow.mCarriedMass = s.mMass;
            blow.mDamage = 0.0f;
            blow.mReactionImpulse = 0.0f;
            blow.mVictimState = victim.blowVictimState();
            const float pushed = applyBlowReaction(victim, blow);
            Log(Debug::Info) << "RAGDOLL_SHOVE_HIT actor=" << from.getPtr().getCellRef().getRefId()
                             << " victim=" << victim.getPtr().getCellRef().getRefId() << " distance=" << flat
                             << " speed=" << blow.mApproachSpeed << " guard=" << victim.mGuardActive
                             << " pushed=" << pushed << " clicks=" << shove.mExtraClicks << " hand_gap=" << handGap.value_or(-1.0f)
                             << " after=" << shoveSettings().mHands - shove.mLeft;
        }
    }

    bool PhysicsSystem::applyTestBlow(const MWWorld::ConstPtr& ptr, std::string_view bodyName,
        const osg::Vec3f& direction, float speed, float mass, float damage)
    {
        if (!direction.valid() || direction.length2() <= 1e-6f || !std::isfinite(speed) || speed < 0.0f
            || !std::isfinite(mass) || mass < 0.0f || !std::isfinite(damage) || damage < 0.0f)
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return false;
        PassiveRagdollEntry& entry = *found->second;
        const RagdollCore::RagdollProfile* profile
            = entry.mRuntimeProfile ? entry.mRuntimeProfile.get() : mPassiveRagdollProfile.get();
        if (!profile)
            return false;
        std::optional<std::uint32_t> body;
        for (std::size_t i = 0; i < profile->mBodies.size(); ++i)
            if (profile->mBodies[i].mName == bodyName)
                body = static_cast<std::uint32_t>(i);
        if (!body || *body >= entry.mRagdoll->getBodyCount())
            return false;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        osg::Vec3f unit = direction;
        unit.normalize();
        RagdollCore::Blow blow;
        blow.mId = RagdollCore::nextBlowId();
        blow.mPhysicsStep = mJoltStepSerial;
        blow.mVictim = entry.mSerial;
        blow.mKind = RagdollCore::BlowKind::Scripted;
        blow.mRegion = RagdollCore::regionForBodyName(bodyName);
        blow.mBodyIndex = *body;
        blow.mDirection = Misc::Convert::toJolt<JPH::Vec3>(unit);
        // The surface facing the blow: a few units back from the centre.
        const JPH::RVec3 centre = mPhysicsSystem.GetBodyInterface().GetCenterOfMassPosition(
            entry.mRagdoll->getBodyId(*body));
        blow.mPoint = centre - JPH::RVec3(blow.mDirection * 6.0f);
        blow.mAttackerToVictim = JPH::Vec3(blow.mDirection.GetX(), blow.mDirection.GetY(), 0.0f);
        blow.mApproachSpeed = speed;
        blow.mCarriedMass = mass;
        blow.mDamage = damage;
        blow.mReactionImpulse
            = damage * MWMechanics::getPhysicalReactionPerDamage() * regionReactionScale(blow.mRegion);
        blow.mVictimState = entry.blowVictimState();
        const float pushed = applyBlowReaction(entry, blow);
        Log(Debug::Info) << "RAGDOLL_TEST_BLOW victim=" << ptr.getCellRef().getRefId() << " body=" << bodyName
                         << " region=" << RagdollCore::regionName(blow.mRegion) << " speed=" << speed
                         << " mass=" << mass << " damage=" << damage << " reaction=" << blow.mReactionImpulse
                         << " pushed=" << pushed;
        return true;
    }

    bool PhysicsSystem::applyPassiveRagdollImpulseAtPoint(
        const MWWorld::ConstPtr& ptr, const osg::Vec3f& impulse, const osg::Vec3f& worldPoint)
    {
        if (!impulse.valid() || !worldPoint.valid())
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return false;

        // A qualifying hit interrupts the victim's own movement for a moment.
        // During recovery the same measured impulse also releases the recovery
        // motors so a later request starts from the body's new physical state.
        // The reaction itself is registered below, once the struck body (and
        // therefore the hit region) is known.
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        JPH::BodyInterface& bodyInterface = mPhysicsSystem.GetBodyInterface();
        if (impulse.length() >= found->second->mExperimentalReferenceProfile.mHitStunMinimumImpulse
            && found->second->mControllerOutput
            && (found->second->mControllerOutput->mMode == RagdollCore::ActiveControllerMode::GettingUp
                || found->second->mControllerOutput->mMode == RagdollCore::ActiveControllerMode::BlendBack))
            found->second->mRecoveryInterrupted = true;
        const JPH::RVec3 point = Misc::Convert::toJolt<JPH::RVec3>(worldPoint);
        std::optional<JPH::BodyID> nearest;
        std::size_t nearestIndex = 0;
        JPH::RVec3 nearestCenter = JPH::RVec3::sZero();
        float nearestDistanceSq = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < found->second->mRagdoll->getBodyCount(); ++i)
        {
            const JPH::BodyID bodyId = found->second->mRagdoll->getBodyId(i);
            const JPH::RVec3 center = bodyInterface.GetCenterOfMassPosition(bodyId);
            const float distanceSq = static_cast<float>((center - point).LengthSq());
            if (distanceSq < nearestDistanceSq)
            {
                nearestDistanceSq = distanceSq;
                nearest = bodyId;
                nearestIndex = i;
                nearestCenter = center;
            }
        }
        if (!nearest)
            return false;
        // Register the hit against the region that was actually struck: the
        // same scaling and tier caps the physical contact path uses, so an
        // animation punch to the head flinches and one to the shin can drop.
        {
            RagdollCore::AnatomicalRegion region = RagdollCore::AnatomicalRegion::Unknown;
            if (mPassiveRagdollProfile && nearestIndex < mPassiveRagdollProfile->mBodies.size())
                region = RagdollCore::regionForBodyName(
                    mPassiveRagdollProfile->mBodies[nearestIndex].mName);
            const float reactionImpulse = impulse.length() * regionReactionScale(region);
            found->second->registerBlowV2(reactionImpulse, RagdollCore::regionCanKnockDown(region));
            found->second->noteKnockingBlow(
                JPH::Vec3(impulse.x(), impulse.y(), impulse.z()), RagdollCore::regionCanKnockDown(region));
            found->second->beginDaze(region, reactionImpulse);
            // The animation hit path only carries its impulse; price the
            // ledger entry with the same impulse-per-damage the physical
            // strikes use, so both paths tint alike.
            if (const float perDamage = MWMechanics::getPhysicalReactionPerDamage(); perDamage > 0.0f)
                found->second->addRegionDamage(region, impulse.length() / perDamage);
            found->second->addKinematicRootReactionKick(
                JPH::Vec3(impulse.x(), impulse.y(), 0.0f), reactionImpulse);
        }
        // The contact sample can sit on a plane in front of the actor; limit
        // the lever arm so the impulse cannot spin a small body absurdly.
        constexpr float maximumLeverArm = 10.0f;
        JPH::Vec3 lever = JPH::Vec3(point - nearestCenter);
        const float leverLength = lever.Length();
        if (leverLength > maximumLeverArm)
            lever *= maximumLeverArm / leverLength;
        bodyInterface.AddImpulse(*nearest, Misc::Convert::toJolt<JPH::Vec3>(impulse), nearestCenter + lever);
        found->second->observeBalanceDisturbance(mJoltStepSerial, impulse.length(), *nearest);
        if (mPassiveRagdollReplays.contains(ptr.mRef))
        {
            const auto appliedPoint = nearestCenter + lever;
            Log(Debug::Info) << std::dec << std::setprecision(17)
                << "RAGDOLL_REPLAY_COMBAT_IMPULSE schema=1 integer_radix=10"
                << " player=" << (ptr == MWMechanics::getPlayer()) << " next_step=" << mJoltStepSerial
                << " body=" << nearestIndex << " impulse=" << impulse.x() << ',' << impulse.y() << ',' << impulse.z()
                << " point=" << appliedPoint.GetX() << ',' << appliedPoint.GetY() << ',' << appliedPoint.GetZ()
                << " requested_point=" << worldPoint.x() << ',' << worldPoint.y() << ',' << worldPoint.z()
                << " stun_remaining=" << found->second->mHitStun.mRemaining;
        }
        return true;
    }

    std::uint64_t PhysicsSystem::nextCombatReactionId()
    {
        if (mNextCombatReactionSerial == 0)
            mNextCombatReactionSerial = 1;
        return mNextCombatReactionSerial++;
    }

    bool PhysicsSystem::applyCombatReactionAtPoint(const MWWorld::ConstPtr& ptr, std::uint64_t reactionId,
        const osg::Vec3f& impulse, const osg::Vec3f& worldPoint)
    {
        if (!impulse.valid() || !worldPoint.valid())
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mPowered || !found->second->mControllerOutput)
            return false;
        auto preview = found->second->mHitStun;
        preview.begin(impulse.length(), found->second->mExperimentalReferenceProfile);
        const RagdollCore::CombatReactionCommand command{
            reactionId,
            { worldPoint.x(), worldPoint.y(), worldPoint.z() },
            { impulse.x(), impulse.y(), impulse.z() },
            preview.mRemaining,
        };
        if (!found->second->mCombatReactions.accept(
                command, found->second->mControllerOutput->mMode, mJoltStepSerial))
            return false;
        return applyPassiveRagdollImpulseAtPoint(ptr, impulse, worldPoint);
    }

    bool PhysicsSystem::setPassiveRagdollActive(const MWWorld::ConstPtr& ptr, bool active)
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return false;

        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        found->second->invalidateBalanceSnapshot("body-activation-changed");
        JPH::BodyInterface& bodyInterface = mPhysicsSystem.GetBodyInterface();
        for (std::size_t i = 0; i < found->second->mRagdoll->getBodyCount(); ++i)
        {
            const JPH::BodyID bodyId = found->second->mRagdoll->getBodyId(i);
            if (active)
                bodyInterface.ActivateBody(bodyId);
            else
                bodyInterface.DeactivateBody(bodyId);
        }
        return true;
    }

    bool PhysicsSystem::startPassiveRagdollReplay(
        const MWWorld::ConstPtr& ptr, std::uint64_t seed, std::optional<std::size_t> maximumFrames, bool concurrent)
    {
        if ((!concurrent && !mPassiveRagdollReplays.empty()) || mPassiveRagdollReplays.contains(ptr.mRef)
            || mPassiveRagdollReplays.size() >= 8 || mPassiveRagdollProfile == nullptr
            || (maximumFrames && (*maximumFrames == 0 || *maximumFrames > RagdollCore::MaximumReplayFrames)))
            return false;
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return false;

        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        auto recording = std::make_unique<PassiveRagdollReplayRecording>();
        recording->mRef = ptr.mRef;
        recording->mMaximumFrames = maximumFrames;
        recording->mReplay.mMetadata.mSeed = seed;
        recording->mReplay.mMetadata.mProfileHash = RagdollCore::hashProfile(*mPassiveRagdollProfile);
        recording->mReplay.mMetadata.mFixedTimeStep = mPhysicsDt;
        recording->mReplay.mMetadata.mWorkerThreadCount = mPhysicsJobSystem->GetMaxConcurrency();
        recording->mReplay.mMetadata.mCollisionSteps = cCollisionSteps;
        recording->mReplay.mBodyCount = static_cast<std::uint32_t>(found->second->mRagdoll->getBodyCount());
        if (found->second->mPowered)
        {
            const std::uint64_t inputCount = RagdollCore::PoweredReplayGlobalInputChannels
                + static_cast<std::uint64_t>(recording->mReplay.mBodyCount)
                    * RagdollCore::PoweredReplayPoseChannelsPerBody;
            if (inputCount > RagdollCore::MaximumReplayChannels)
                return false;
            recording->mReplay.mInputCount = static_cast<std::uint32_t>(inputCount);
            recording->mReplay.mControllerStateCount = (found->second->mExperimentalReferenceProfile.mContinuousSwingTrajectory
                || RagdollCore::tuningText("OPENMW_RAGDOLL_CONTINUOUS_SWING") != nullptr)
                ? RagdollCore::ContinuousSwingReplayControllerStateChannels : RagdollCore::PoweredReplayControllerStateChannels;
        }
        const auto& captured = *found->second;
        const auto& options = captured.mReplayCreationOptions;
        const auto& root = options.mRootPosition;
        const auto& rotation = options.mRootRotation;
        const auto& motion = captured.mReplayCreationMotion;
        Log(Debug::Info) << std::dec << std::setprecision(17)
            << "RAGDOLL_REPLAY_RIG schema=1 recovery_event_schema=2 direct_event_schema=1 seed=" << seed
            << " start_step=" << mJoltStepSerial << " controller_steps=" << captured.mFixedSteps
            << " profile_id=" << mPassiveRagdollProfile->mId
            << " profile_hash=" << RagdollCore::hashProfile(*mPassiveRagdollProfile)
            << " mass=" << options.mTotalMass << " scale=" << options.mUniformScale
            << " root=" << root.GetX() << ',' << root.GetY() << ',' << root.GetZ()
            << " rotation=" << rotation.GetX() << ',' << rotation.GetY() << ',' << rotation.GetZ() << ',' << rotation.GetW()
            << " velocity=" << motion.mLinearVelocity.GetX() << ',' << motion.mLinearVelocity.GetY() << ',' << motion.mLinearVelocity.GetZ()
            << " angular_velocity=" << motion.mAngularVelocity.GetX() << ',' << motion.mAngularVelocity.GetY() << ',' << motion.mAngularVelocity.GetZ();
        for (std::size_t i = 0; i < captured.mReplayCreationSkeleton.mBones.size(); ++i)
        {
            const auto& bone = captured.mReplayCreationSkeleton.mBones[i];
            const auto& t = bone.mModelTransform;
            Log(Debug::Info) << std::dec << std::setprecision(17)
                << "RAGDOLL_REPLAY_BONE index=" << i << " name=" << std::quoted(bone.mName)
                << " parent=" << bone.mParent
                << " position=" << t.mTranslation.GetX() << ',' << t.mTranslation.GetY() << ',' << t.mTranslation.GetZ()
                << " rotation=" << t.mRotation.GetX() << ',' << t.mRotation.GetY() << ',' << t.mRotation.GetZ() << ',' << t.mRotation.GetW();
        }
        mPassiveRagdollReplays.emplace(ptr.mRef, std::move(recording));
        return true;
    }

    std::optional<PassiveRagdollReplayResult> PhysicsSystem::finishPassiveRagdollReplay(const MWWorld::ConstPtr* ptr)
    {
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        if (!ptr && mPassiveRagdollReplays.size() != 1)
            return std::nullopt;
        const auto found = ptr ? mPassiveRagdollReplays.find(ptr->mRef) : mPassiveRagdollReplays.begin();
        if (found == mPassiveRagdollReplays.end())
            return std::nullopt;
        auto recording = std::move(found->second);
        mPassiveRagdollReplays.erase(found);
        if (recording->mFailed)
            return std::nullopt;
        auto data = RagdollCore::serializeReplay(recording->mReplay);
        if (!data)
            return std::nullopt;

        const RagdollCore::ReplayMetadata& metadata = recording->mReplay.mMetadata;
        PassiveRagdollReplayResult result;
        result.mFirstFallingFrame = recording->mFirstFallingFrame;
        result.mVersion = metadata.mVersion;
        result.mSeed = metadata.mSeed;
        result.mProfileHash = metadata.mProfileHash;
        result.mFixedTimeStep = metadata.mFixedTimeStep;
        result.mWorkerThreadCount = metadata.mWorkerThreadCount;
        result.mCollisionSteps = metadata.mCollisionSteps;
        result.mInputCount = recording->mReplay.mInputCount;
        result.mControllerStateCount = recording->mReplay.mControllerStateCount;
        result.mBodyCount = recording->mReplay.mBodyCount;
        result.mFrameCount = recording->mReplay.mFrames.size();
        result.mData = std::move(*data);
        return result;
    }

#ifdef OPENMW_RAGDOLL_DEV_API
    RagdollProfile::AnimationCycleSample PhysicsSystem::sampleRagdollAnimationCycle(const MWWorld::ConstPtr& ptr,
        const SceneUtil::KeyframeHolder& source, std::string_view group, std::string_view accumulationBone,
        const osg::Vec3f& accumulationAxes, std::size_t sampleCount) const
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mPoseBinding)
        {
            RagdollProfile::AnimationCycleSample result;
            result.mError = "Actor has no ragdoll skeleton binding";
            return result;
        }
        return found->second->mPoseBinding->sampleAnimationCycle(source, group, accumulationBone,
            Misc::Convert::toJolt<JPH::Vec3>(accumulationAxes), sampleCount);
    }

    bool PhysicsSystem::samplePoweredRecoveryTarget(const MWWorld::ConstPtr& ptr,
        const SceneUtil::KeyframeHolder& source, float time, std::string_view accumulationBone,
        const osg::Vec3f& accumulationAxes)
    {
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mPowered || !found->second->mPoseBinding)
            return false;
        auto& entry = *found->second;
        entry.mSampledRecoveryTarget.reset();
        std::vector<RagdollCore::Transform> pose;
        if (!entry.mPoseBinding->sampleAnimationPose(source, time, pose, accumulationBone,
                Misc::Convert::toJolt<JPH::Vec3>(accumulationAxes))
            || pose.size() != entry.mRagdoll->getBodyCount())
            return false;
        entry.mSampledRecoveryTarget = std::move(pose);
        if (entry.mSampledRecoveryTraceCount++ < 120)
            Log(Debug::Warning) << "RAGDOLL_RECOVERY_SOURCE time=" << time;
        return true;
    }

    bool PhysicsSystem::requestPoweredRagdollRecovery(const MWWorld::ConstPtr& ptr)
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        if (found == mPassiveRagdolls.end() || !found->second->mPowered || !found->second->mControllerOutput
            || found->second->mControllerOutput->mMode != RagdollCore::ActiveControllerMode::Fallen)
            return false;
        auto& entry = *found->second;
        // A previous standing/stop handover is not the source pose of a new
        // rise. Otherwise the ordinary motor path overwrites the recovery's
        // leg targets with cached pre-fall joint angles every fixed step.
        entry.mStandingLegPose.reset();
        entry.mHandover.reset();
        entry.mRecoveryHandoverPending = false;
        entry.mRecoveryRequested = true;
        if (mPassiveRagdollReplays.contains(ptr.mRef))
            Log(Debug::Info) << std::dec
                << "RAGDOLL_REPLAY_RECOVERY_REQUEST schema=1 integer_radix=10"
                << " player=" << (ptr == MWMechanics::getPlayer()) << " next_step=" << mJoltStepSerial;
        return true;
    }

    std::optional<RagdollCore::CombatState> PhysicsSystem::getRagdollCombatState(
        const MWWorld::ConstPtr& ptr) const
    {
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end())
            return std::nullopt;
        const auto& entry = *found->second;
        return RagdollCore::makeCombatState(entry.mPowered,
            entry.mControllerOutput ? &*entry.mControllerOutput : nullptr, mJoltStepSerial);
    }

    void PhysicsSystem::requestRagdollAiRiposte(const MWWorld::ConstPtr& ptr)
    {
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found != mPassiveRagdolls.end())
            found->second->mRiposteGranted = 0.5f;
    }

    bool PhysicsSystem::requestRagdollDodge(const MWWorld::ConstPtr& ptr, std::optional<osg::Vec3f> direction)
    {
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mPowered)
            return false;
        std::optional<JPH::Vec3> wanted;
        if (direction)
            wanted = JPH::Vec3(direction->x(), direction->y(), 0.0f);
        found->second->requestDodge(wanted);
        return true;
    }

    bool PhysicsSystem::cycleRagdollGrip(const MWWorld::ConstPtr& ptr)
    {
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        return found != mPassiveRagdolls.end() && found->second->cycleGrip();
    }

    void PhysicsSystem::markRagdollParryWon(const MWWorld::ConstPtr& ptr)
    {
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found != mPassiveRagdolls.end())
            found->second->mAiSinceParryWon = 0.0f;
    }

    void PhysicsSystem::requestRagdollAiGuard(const MWWorld::ConstPtr& ptr, float seconds)
    {
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found != mPassiveRagdolls.end() && std::isfinite(seconds))
            found->second->mAiGuardHoldFor = std::max(found->second->mAiGuardHoldFor, seconds);
    }

    std::optional<RagdollCore::DuelBodyView> PhysicsSystem::getRagdollDuelView(const MWWorld::ConstPtr& ptr) const
    {
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mPowered)
            return std::nullopt;
        const auto& entry = *found->second;
        RagdollCore::DuelBodyView view;
        view.mTipSpeed = entry.mDuelTipSpeed;
        view.mSinceFast = entry.mDuelSinceFast;
        view.mSinceBounce = entry.mDuelSinceImpact;
        view.mArmed = entry.mWeapon && entry.mWeapon->mAttached;
        view.mWeaponReach = view.mArmed ? std::max(entry.mWeapon->tipDistanceFromHand(), 0.0f) : 0.0f;
        view.mWheel = entry.mHscActive;
        view.mHitsTaken = entry.mAiHitsTaken;
        view.mWindingUp = entry.mHscActive && entry.mAttackStage == PassiveRagdollEntry::AttackStage::Wind;
        // AI-02 item 1a: the wheel attack outlasts the AI's attack flag (the
        // cut came 0.4-0.8 s after it, while stepping out): the tactics read it.
        if (entry.mHscActive)
        {
            using Stage = PassiveRagdollEntry::AttackStage;
            view.mWheelStage = entry.mAttackStage == Stage::Wind ? 1
                : entry.mAttackStage == Stage::Cut               ? 2
                : entry.mAttackStage == Stage::Recover           ? 3
                                                                 : 0;
        }
        view.mGuardBroken = entry.mClashLockout > 0.0f;
        view.mSinceParryWon = entry.mAiSinceParryWon;
        // The blade's side (NPC dodges): its tip across the body's facing.
        if (entry.mDuelTipPoint && entry.mPelvisBodyIndex && entry.mCurrentPhysicsPoseCaptured)
            if (const auto pelvis = entry.mRagdoll->getWorldBodyPosition(entry.mCurrentPhysicsPose, *entry.mPelvisBodyIndex))
            {
                const float yaw = ptr.getRefData().getPosition().rot[2];
                const JPH::Vec3 right(std::cos(yaw), -std::sin(yaw), 0.0f);
                view.mTipSide = JPH::Vec3(*entry.mDuelTipPoint - *pelvis).Dot(right);
            }
        view.mStaggered = entry.mReactionV2Step.mTier >= RagdollCore::ReactionTier::Stagger
            || entry.mReactionV2Step.mWeaponArmBusy;
        if (entry.mControllerOutput)
        {
            using Mode = RagdollCore::ActiveControllerMode;
            const Mode mode = entry.mControllerOutput->mMode;
            view.mOffBalance = mode == Mode::Stumbling;
            // RXN-02 M6: a leg giving out (REACTION_V2_LEG) is an opening.
            view.mOffBalance = view.mOffBalance
                    || std::max(entry.mReactionV2Step.mLegGiveOut[0], entry.mReactionV2Step.mLegGiveOut[1]) >= 0.5f;
            view.mDown = mode == Mode::Falling || mode == Mode::Fallen || mode == Mode::GettingUp
                || mode == Mode::BlendBack;
        }
        return view;
    }

    std::optional<PoweredRagdollRecoveryState> PhysicsSystem::getPoweredRagdollRecoveryState(
        const MWWorld::ConstPtr& ptr) const
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mPowered)
            return std::nullopt;
        PoweredRagdollRecoveryState result;
        if (found->second->mControllerOutput && found->second->mControllerOutput->mRecoveryDirection)
        {
            result.mFaceUp
                = *found->second->mControllerOutput->mRecoveryDirection == RagdollCore::RecoveryDirection::FaceUp;
        }
        result.mAnimationStarted = found->second->mRecoveryAnimationStarted;
        result.mAnimationFinished = found->second->mRecoveryAnimationFinished;
        return result;
    }

    bool PhysicsSystem::setPoweredRagdollRecoveryAnimationStarted(const MWWorld::ConstPtr& ptr)
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mPowered || !found->second->mControllerOutput
            || !found->second->mControllerOutput->mRecoveryDirection || found->second->mRecoveryAnimationStarted)
            return false;
        found->second->mRecoveryAnimationStarted = true;
        return true;
    }

    bool PhysicsSystem::setPoweredRagdollRecoveryAnimationFinished(const MWWorld::ConstPtr& ptr)
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mPowered || !found->second->mRecoveryAnimationStarted
            || found->second->mRecoveryAnimationFinished)
            return false;
        found->second->mRecoveryAnimationFinished = true;
        return true;
    }

    std::optional<float> PhysicsSystem::finishPoweredRagdollRecovery(const MWWorld::Ptr& ptr)
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || !found->second->mPowered
            || !found->second->mRecoveryAnimationFinished || !found->second->mControllerOutput
            || found->second->mControllerOutput->mMode != RagdollCore::ActiveControllerMode::Standing
            || !found->second->mCurrentPhysicsPoseCaptured)
            return std::nullopt;
        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        auto& entry = *found->second;
        // The controller requires sustained supported standing. A timeout
        // cannot transfer ownership to animation or resurrect the capsule.
        if (mPassiveRagdollReplays.contains(ptr.mRef))
            Log(Debug::Info) << std::dec << "RAGDOLL_REPLAY_RECOVERY_FINISH integer_radix=10"
                << " player=" << (ptr == MWMechanics::getPlayer())
                << " next_step=" << mJoltStepSerial;
        entry.mStandingLegPose = entry.mCurrentPhysicsPose;
        entry.mRecoveryHandoverPending = true;
        entry.mRecoveryRequested = false;
        entry.mRecoveryAnimationStarted = false;
        entry.mRecoveryAnimationFinished = false;
        float yaw = ptr.getRefData().getPosition().rot[2];
        // Player facing is input intent (and camera direction), not a pose
        // to inherit from a fallen body. The persistent rig turns toward that
        // intent through its normal locomotion controller after recovery.
        if (entry.mControllerObservationInitialized && std::isfinite(entry.mHeadingError))
        {
            const float physicalYaw = yaw - entry.mHeadingError;
            if (ptr == MWMechanics::getPlayer())
                entry.mSteering.mFacing = JPH::Vec3(std::sin(physicalYaw), std::cos(physicalYaw), 0.0f);
            else
                yaw = physicalYaw;
        }
        return yaw;
    }

    void PhysicsSystem::failPoweredRagdollControl(const MWWorld::ConstPtr& ptr)
    {
        if (const auto found = mPassiveRagdolls.find(ptr.mRef);
            found != mPassiveRagdolls.end() && found->second->mPowered)
            found->second->failPoweredControl(__LINE__);
    }
#endif

    void PhysicsSystem::recordPassiveRagdollReplayFrame()
    {
        const auto record = [&](PassiveRagdollReplayRecording& recording) {
            if (recording.mFailed)
                return;
            if (recording.mMaximumFrames
                && recording.mReplay.mFrames.size() >= *recording.mMaximumFrames)
                return;
            const auto found = mPassiveRagdolls.find(recording.mRef);
            if (found == mPassiveRagdolls.end() || !found->second->mCurrentPhysicsPoseCaptured)
            {
                recording.mFailed = true;
                return;
            }

            const PassiveRagdollEntry& entry = *found->second;
            if (entry.mCurrentPhysicsPose.mBodyTransforms.size() != entry.mRagdoll->getBodyCount())
            {
                recording.mFailed = true;
                return;
            }

            RagdollCore::ReplayFrame frame;
            frame.mStepIndex = recording.mReplay.mFrames.size();
            if (recording.mFirstFallingFrame < 0 && entry.mControllerOutput
                && (entry.mControllerOutput->mMode == RagdollCore::ActiveControllerMode::Falling
                    || entry.mControllerOutput->mMode == RagdollCore::ActiveControllerMode::Fallen
                    || entry.mControllerOutput->mMode == RagdollCore::ActiveControllerMode::GettingUp))
                recording.mFirstFallingFrame = static_cast<std::int64_t>(frame.mStepIndex);
            if (entry.mPowered)
            {
                if (!entry.mMotorTargetCaptured
                    || entry.mMotorTargetPose.mBodyTransforms.size() != entry.mRagdoll->getBodyCount()
                    || !entry.mControllerOutput || !entry.mGaitOutput || !entry.mSupportEstimate)
                {
                    recording.mFailed = true;
                    return;
                }
                const auto appendVec3 = [](std::vector<float>& values, JPH::Vec3Arg value) {
                    values.push_back(value.GetX());
                    values.push_back(value.GetY());
                    values.push_back(value.GetZ());
                };
                frame.mInputs.reserve(recording.mReplay.mInputCount);
                appendVec3(frame.mInputs, entry.mTargetHorizontalVelocity);
                appendVec3(frame.mInputs, entry.mRawTargetHorizontalVelocity);
                appendVec3(frame.mInputs, entry.mSteering.facing());
                frame.mInputs.push_back(entry.mLastRecoveryAnimationFinishedInput ? 1.0f : 0.0f);
                for (const RagdollCore::Transform& target : entry.mMotorTargetPose.mBodyTransforms)
                {
                    appendVec3(frame.mInputs, target.mTranslation);
                    frame.mInputs.push_back(target.mRotation.GetX());
                    frame.mInputs.push_back(target.mRotation.GetY());
                    frame.mInputs.push_back(target.mRotation.GetZ());
                    frame.mInputs.push_back(target.mRotation.GetW());
                }

                frame.mControllerState.resize(RagdollCore::PoweredReplayControllerStateChannels);
                const auto setControllerState = [&](RagdollCore::PoweredReplayControllerChannel channel, float value) {
                    frame.mControllerState[static_cast<std::size_t>(channel)] = value;
                };
                using Channel = RagdollCore::PoweredReplayControllerChannel;
                setControllerState(Channel::Mode, static_cast<float>(entry.mControllerOutput->mMode));
                setControllerState(Channel::TimeInMode, entry.mControllerOutput->mTimeInMode);
                setControllerState(Channel::GaitPhase, static_cast<float>(entry.mGaitOutput->mPhase));
                setControllerState(Channel::TimeInGaitPhase, entry.mGaitOutput->mTimeInPhase);
                setControllerState(Channel::GaitPhaseProgress, entry.mGaitOutput->mPhaseProgress);
                setControllerState(Channel::SwingFootLifted, entry.mGaitOutput->mSwingFootLifted ? 1.0f : 0.0f);
                setControllerState(Channel::LeftFootSupported, entry.mSupportEstimate->mLeftFootSupported ? 1.0f : 0.0f);
                setControllerState(Channel::RightFootSupported, entry.mSupportEstimate->mRightFootSupported ? 1.0f : 0.0f);
                setControllerState(
                    Channel::CenterOfMassSupported, entry.mSupportEstimate->mCenterOfMassSupported ? 1.0f : 0.0f);
                setControllerState(Channel::GaitPhaseTimedOut, entry.mGaitOutput->mPhaseTimedOut ? 1.0f : 0.0f);
                if (recording.mReplay.mControllerStateCount == RagdollCore::ContinuousSwingReplayControllerStateChannels)
                {
                    // Only the removed reference walker kept a swing history.
                    const auto history = RagdollCore::encodeSwingTrajectoryHistory(std::nullopt);
                    if (!history) { recording.mFailed = true; return; }
                    frame.mControllerState.insert(frame.mControllerState.end(), history->begin(), history->end());
                }
            }
            frame.mBodies.reserve(entry.mRagdoll->getBodyCount());
            const JPH::BodyInterface& bodies = mPhysicsSystem.GetBodyInterface();
            for (std::size_t i = 0; i < entry.mRagdoll->getBodyCount(); ++i)
            {
                const JPH::BodyID bodyId = entry.mRagdoll->getBodyId(i);
                if (bodyId.IsInvalid())
                {
                    recording.mFailed = true;
                    return;
                }
                RagdollCore::ReplayBodyState body;
                body.mTransform = entry.mCurrentPhysicsPose.mBodyTransforms[i];
                body.mLinearVelocity = bodies.GetLinearVelocity(bodyId);
                body.mAngularVelocity = bodies.GetAngularVelocity(bodyId);
                frame.mBodies.push_back(body);
            }
            const auto replayRoot = entry.mRagdoll->getReferencePosition();
            const auto replayRotation = entry.mExperimentalReferenceRotation;
            Log(Debug::Info) << std::dec << std::setprecision(17)
                << "RAGDOLL_REPLAY_FRAME frame=" << frame.mStepIndex << " step=" << mJoltStepSerial << " seed=" << recording.mReplay.mMetadata.mSeed
                << " root=" << replayRoot.GetX() << ',' << replayRoot.GetY() << ',' << replayRoot.GetZ()
                << " rotation=" << replayRotation.GetX() << ',' << replayRotation.GetY() << ',' << replayRotation.GetZ() << ',' << replayRotation.GetW();
            recording.mReplay.mFrames.push_back(std::move(frame));
        };
        for (auto& [ref, recording] : mPassiveRagdollReplays)
            record(*recording);
    }

    int PhysicsSystem::addProjectile(
        const MWWorld::Ptr& caster, const osg::Vec3f& position, VFS::Path::NormalizedView mesh, bool computeRadius)
    {
        osg::ref_ptr<Resource::PhysicsShapeInstance> shapeInstance = mShapeManager->getInstance(mesh);
        assert(shapeInstance);
        float radius = computeRadius ? shapeInstance->mCollisionBox.mExtents.length() / 2.f : 1.f;

        mProjectileId++;

        auto projectile = std::make_shared<Projectile>(caster, position, radius, mTaskScheduler.get(), this);
        mProjectiles.emplace(mProjectileId, std::move(projectile));

        return mProjectileId;
    }

    void PhysicsSystem::setCaster(int projectileId, const MWWorld::Ptr& caster)
    {
        const auto foundProjectile = mProjectiles.find(projectileId);
        assert(foundProjectile != mProjectiles.end());
        auto* projectile = foundProjectile->second.get();

        projectile->setCaster(caster);
    }

    bool PhysicsSystem::toggleCollisionMode()
    {
        ActorMap::iterator found = mActors.find(MWMechanics::getPlayer().mRef);
        if (found != mActors.end())
        {
            bool cmode = found->second->getCollisionMode();
            cmode = !cmode;
            found->second->enableCollisionMode(cmode);
            // NB: Collision body isn't disabled for vanilla TCL compatibility
            return cmode;
        }

        return false;
    }

    osg::Vec3f PhysicsSystem::getPassiveRagdollStumble(const MWWorld::ConstPtr& ptr) const
    {
        const auto found = mPassiveRagdolls.find(ptr.mRef);
        if (found == mPassiveRagdolls.end() || found->second->mSoftRootStumbleTime <= 0.0f
            || !found->second->mRagdoll || !found->second->mRagdoll->isRootKinematic())
            return osg::Vec3f();
        const JPH::Vec3 d = found->second->mSoftRootStumbleDirection;
        return osg::Vec3f(d.GetX(), d.GetY(), 0.0f);
    }

    void PhysicsSystem::queueObjectMovement(const MWWorld::Ptr& ptr, const osg::Vec3f& velocity)
    {
        ActorMap::iterator found = mActors.find(ptr.mRef);
        if (found == mActors.end())
            return;
        osg::Vec3f total = velocity;
        // A corpse carries its actor; the death clip does not (CORPSE_FOLLOW).
        if (const auto ragdoll = mPassiveRagdolls.find(ptr.mRef);
            ragdoll != mPassiveRagdolls.end() && !ragdoll->second->mPowered && corpseFollow())
            total = osg::Vec3f();
        if (const auto ragdoll = mPassiveRagdolls.find(ptr.mRef); ragdoll != mPassiveRagdolls.end()
            && ragdoll->second->mRagdoll && ragdoll->second->mRagdoll->getSoftRoot().mEnabled
            && ragdoll->second->mRagdoll->isRootKinematic())
        {
            PassiveRagdollEntry& entry = *ragdoll->second;
            // The solver rotates this actor-local velocity by the actor's yaw.
            const osg::Quat yaw(ptr.getRefData().getPosition().rot[2], osg::Vec3f(0, 0, -1));
            osg::Vec3f world = yaw * total;
            // Getting up is a crawl (user 2026-09-28: "when standing up movement
            // speed should be fixed to some very slow amount - so it's like the
            // player is crawling trying to get up"): the movement input is scaled
            // while getting up and blending back. OPENMW_RAGDOLL_GETUP_MOVE =
            // share of the speed (1 = unchanged).
            static const auto getUpMove_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_GETUP_MOVE");
                const float value = text ? static_cast<float>(std::atof(text)) : 0.15f;
                return std::isfinite(value) ? std::clamp(value, 0.0f, 1.0f) : 0.15f;
            });
            static const auto& getUpMove = getUpMove_tuned.get();
            if (entry.mControllerOutput
                && (entry.mControllerOutput->mMode == RagdollCore::ActiveControllerMode::GettingUp
                    || entry.mControllerOutput->mMode == RagdollCore::ActiveControllerMode::BlendBack))
            {
                world.x() *= getUpMove;
                world.y() *= getUpMove;
            }
            const JPH::Vec3 knockback = entry.mSoftRootKnockback;
            world += osg::Vec3f(knockback.GetX(), knockback.GetY(), 0.0f);
            // Jump launch (user 11-01: the Scroll of Icarian Flight, Fortify
            // Acrobatics 1000, "only move a little forward"): Morrowind's
            // running jump launches as fast forward as up (~2700 u/s each
            // with that scroll); the momentum bound below cut the forward
            // part to the run speed and the flight carried 640 units, not
            // vanilla's ~20000. A launch faster than the capsule already moves
            // by more than a margin passes unbounded into the jump's inertia;
            // the walking velocity the bound follows stays the one before the
            // jump. Ordinary launches (~200 u/s, ~460 at Acrobatics 100) are
            // bounded as before, also while a run is still speeding up.
            // OPENMW_RAGDOLL_JUMP_LAUNCH_KEEPS = margin u/s (0 = off).
            static const auto launchMargin_tuned = RagdollCore::tuned<float>(
                [] { return PassiveRagdollEntry::weaponDriveSetting("OPENMW_RAGDOLL_JUMP_LAUNCH_KEEPS", 300.0f); });
            const float launchMargin = launchMargin_tuned.get();
            const bool launch = total.z() > 0.0f && found->second->getOnGround()
                && !MWBase::Environment::get().getWorld()->isFlying(ptr)
                && !MWBase::Environment::get().getWorld()->isSwimming(ptr);
            if (launch)
                entry.mJumpLaunch = JPH::Vec3(world.x(), world.y(), world.z());
            const float launchSpeed = std::hypot(world.x(), world.y());
            const float walkSpeed
                = entry.mCapsuleVelocity ? std::hypot(entry.mCapsuleVelocity->x(), entry.mCapsuleVelocity->y()) : 0.0f;
            if (launch && launchMargin > 0.0f && launchSpeed > walkSpeed + launchMargin)
            {
                entry.mCapsuleStepSerial = mJoltStepSerial;
                found->second->setVelocity(yaw.inverse() * world);
                return;
            }
            // Physical momentum: bound how fast the capsule's horizontal
            // velocity may change, over the fixed steps since the last call.
            const float limit = entry.mRagdoll->getSoftRoot().mBody.mCapsuleAcceleration;
            const float elapsed = static_cast<float>(mJoltStepSerial - entry.mCapsuleStepSerial) * mPhysicsDt;
            entry.mCapsuleStepSerial = mJoltStepSerial;
            if (limit > 0.0f && entry.mCapsuleVelocity && elapsed > 0.0f && elapsed < 0.25f)
            {
                osg::Vec3f change(world.x() - entry.mCapsuleVelocity->x(), world.y() - entry.mCapsuleVelocity->y(), 0.0f);
                const float maximum = limit * elapsed;
                if (change.length() > maximum)
                {
                    change *= maximum / change.length();
                    world.x() = entry.mCapsuleVelocity->x() + change.x();
                    world.y() = entry.mCapsuleVelocity->y() + change.y();
                }
            }
            entry.mCapsuleVelocity = osg::Vec3f(world.x(), world.y(), 0.0f);
            total = yaw.inverse() * world;
        }
        found->second->setVelocity(total);
    }

    void PhysicsSystem::updateAnimatedCollisionShape(const MWWorld::Ptr& ptr)
    {
        auto foundObject = mObjects.find(ptr.mRef);
        if (foundObject == mObjects.end())
            return;

        Object* object = foundObject->second.get();
        auto animatedObject = mAnimatedObjects.find(object);
        if (animatedObject == mAnimatedObjects.end())
            return;

        std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
        animatedObject->second = object->animateCollisionShapes();
    }

    void PhysicsSystem::clearQueuedMovement()
    {
        for (const auto& [_, actor] : mActors)
        {
            actor->setVelocity(osg::Vec3f());
            actor->setInertialForce(osg::Vec3f());
        }
    }

    void PhysicsSystem::prepareSimulation(bool willSimulate, std::vector<Simulation>& simulations)
    {
        assert(simulations.empty());
        simulations.reserve(mActors.size() + mProjectiles.size());
        const MWBase::World* world = MWBase::Environment::get().getWorld();
        for (const auto& [ref, physicActor] : mActors)
        {
            if (!physicActor->isActive())
                continue;

            auto ptr = physicActor->getPtr();
            if (!ptr.getClass().isMobile(ptr))
                continue;
            float waterlevel = -std::numeric_limits<float>::max();
            const MWWorld::CellStore* cell = ptr.getCell();
            if (cell->getCell()->hasWater())
                waterlevel = cell->getWaterLevel();

            const auto& stats = ptr.getClass().getCreatureStats(ptr);
            const MWMechanics::MagicEffects& effects = stats.getMagicEffects();

            bool waterCollision = false;
            if (cell->getCell()->hasWater() && effects.getOrDefault(ESM::MagicEffect::WaterWalking).getMagnitude() > 0)
            {
                if (physicActor->getCollisionMode()
                    || !world->isUnderwater(ptr.getCell(), ptr.getRefData().getPosition().asVec3()))
                    waterCollision = true;
            }

            physicActor->setCanWaterWalk(waterCollision);

            // Slow fall reduces fall speed by a factor of (effect magnitude / 200)
            const float slowFall
                = 1.f - std::clamp(effects.getOrDefault(ESM::MagicEffect::SlowFall).getMagnitude() * 0.005f, 0.f, 1.f);
            const bool isPlayer = ptr == world->getPlayerConstPtr();
            const bool godmode = isPlayer && world->getGodModeState();
            const bool inert = stats.isDead()
                || (!godmode && stats.getMagicEffects().getOrDefault(ESM::MagicEffect::Paralyze).getModifier() > 0);

            simulations.emplace_back(ActorSimulation{
                physicActor, ActorFrameData{ *physicActor, inert, waterCollision, slowFall, waterlevel, isPlayer } });

            // Powered kinematic bodies buffer a requested jump through a short
            // crouch: hold the movement request while the ragdoll dips, then
            // release it so the takeoff follows the anticipation.
            if (willSimulate && mPassiveRagdollProfile
                && mPassiveRagdollProfile->mActiveController.mLocomotionRoute
                    == RagdollCore::LocomotionRoute::KinematicRoot)
            {
                const auto ragdollEntry = mPassiveRagdolls.find(ptr.mRef);
                if (ragdollEntry != mPassiveRagdolls.end() && ragdollEntry->second->mPowered)
                {
                    auto& entry = *ragdollEntry->second;
                    auto& movement = ptr.getClass().getMovementSettings(ptr);
                    // A body that is getting up must not walk at the same
                    // time: the locomotion request drags the actor (and the
                    // animation-owned root) out from under the get-up clip,
                    // the rise fails and the body falls again. Hold every
                    // movement channel until the recovery hands back to
                    // standing.
                    if (entry.mControllerOutput
                        && (entry.mControllerOutput->mMode == RagdollCore::ActiveControllerMode::GettingUp
                            || entry.mControllerOutput->mMode == RagdollCore::ActiveControllerMode::BlendBack))
                    {
                        movement.mPosition[0] = movement.mPosition[1] = movement.mPosition[2] = 0.0f;
                        handleJump(ptr);
                        continue;
                    }
                    // The active body does not buffer the jump into a crouch:
                    // the capsule launched at once regardless, the crouch pulled
                    // the body's target 14 units down while the capsule rose,
                    // and the body stayed on the floor then was flung up late
                    // (play test 2026-09-24 M8 "jumping has no animation").
                    if (entry.mRagdoll->getSoftRoot().mEnabled)
                    {
                        entry.mJumpAnticipation = 0.0f;
                        entry.mJumpReleasePending = false;
                        if (movement.mPosition[2] != 0 && entry.mSoftRootJumpWindow <= 0.0f)
                        {
                            entry.mSoftRootJumpWindow = 0.8f;
                            entry.mJumpCommanded = 1.2f;
                        }
                    }
                    else if (entry.mJumpAnticipation > 0.0f)
                        movement.mPosition[2] = 0;
                    else if (entry.mJumpReleasePending)
                    {
                        movement.mPosition[2] = 1;
                        entry.mJumpReleasePending = false;
                    }
                    else if (movement.mPosition[2] != 0 && entry.mJumpCrouch <= 0.0f
                        && !entry.mRagdoll->getSoftRoot().mEnabled)
                    {
                        entry.mJumpAnticipation
                            = mPassiveRagdollProfile->mActiveController.mKinematicRootJumpAnticipation;
                        movement.mPosition[2] = 0;
                    }
                }
            }

            // if the simulation will run, a jump request will be fulfilled. Update mechanics accordingly.
            if (willSimulate)
                handleJump(ptr);
        }
    }

    void PhysicsSystem::stepSimulation(
        float dt, bool skipSimulation, osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats)
    {
        // We cannot modify shapes at runtime while there is a body query going on (such as actor collision)
        // so we must update all animated objects first when we are guaranteed to not be having any physics queries
        {
            std::scoped_lock lock(mTaskScheduler->getSimulationMutex());
            // Live tuning (the KINETIC options page): queued switch changes
            // take effect here, where no physics reader runs.
            for (const std::string& name : RagdollCore::refreshTuning())
                Log(Debug::Info) << "KINETIC live " << name << " = "
                                 << (RagdollCore::tuningText(name) ? RagdollCore::tuningText(name) : "(default)");
            for (auto& [animatedObject, changed] : mAnimatedObjects)
            {
                if (animatedObject->animateCollisionShapes())
                {
                    assert(mObjects.find(animatedObject->getPtr().mRef) != mObjects.end());
                    changed = true;
                }
                else
                {
                    changed = false;
                }
            }
        }

        for (const MWWorld::LiveCellRefBase* ref : Object::consumeCollidedObjects())
        {
            if (const auto it = mObjects.find(ref); it != mObjects.end())
                it->second->resetCollisions();
        }

        mTimeAccum += dt;
        mTimeAccumJolt += dt;
        if (dt > 0.0f)
        {
            for (auto& [_, entry] : mPassiveRagdolls)
            {
                entry->mModeTransitionElapsed
                    = std::min(entry->mModeTransitionElapsed + dt, entry->mModeTransitionDuration);
            }
        }

        // Capture the gameplay-requested gait velocity before ActorFrameData
        // consumes the movement queue. All fixed Jolt steps in this rendered
        // frame then see the same deterministic target.
        for (auto& [ref, entry] : mPassiveRagdolls)
        {
            if (!entry->mPowered)
                continue;
            const auto actor = mActors.find(ref);
            if (actor == mActors.end())
            {
                entry->clearPendingBalanceIntent("actor-queue-unavailable");
                entry->failPoweredControl(__LINE__);
            }
            else if (!entry->captureLocomotionTarget(*actor->second, dt, mJoltStepSerial, frameNumber))
                entry->failPoweredControl(__LINE__);
#ifdef OPENMW_RAGDOLL_DEV_API
            else if (RagdollCore::tuningText("OPENMW_RAGDOLL_ANIMATION_INPUT_TRACE") && entry->mAnimationInputSerial <= 3600)
            {
                const auto raw = entry->mRawTargetHorizontalVelocity;
                Log(Debug::Info) << std::dec << std::setprecision(17)
                    << "RAGDOLL_ANIMATION_RAW schema=1 player=" << (entry->getPtr() == MWMechanics::getPlayer())
                    << " actor=" << entry->getPtr().getCellRef().getRefId()
                    << " serial=" << entry->mAnimationInputSerial << " next_step=" << mJoltStepSerial
                    << " previous_known=" << entry->mPreviousMotorTargetPose.has_value()
                    << " render=" << frameNumber << " raw=" << raw.GetX() << ',' << raw.GetY() << ',' << raw.GetZ()
                    << " interrupted=" << entry->mLocomotionInterrupted << " render_dt=" << dt;
            }
#endif
        }

        if (skipSimulation)
        {
            mTaskScheduler->resetSimulation(mActors);
            // A discard frame accompanies world/cell changes. Do not advance
            // dynamic Jolt bodies through stale geometry or catch this time up
            // on the following frame.
            mTimeAccumJolt = 0.0f;
        }
        else
        {
            std::vector<Simulation>& simulations = mSimulations[mSimulationsCounter++ % mSimulations.size()];
            prepareSimulation(mTimeAccum >= mPhysicsDt, simulations);

            // Runs world simulation for required steps, modifies mTimeAccum
            mTaskScheduler->applyQueuedMovements(mTimeAccum, simulations, frameStart, frameNumber, stats);
        }

        // Synchronize/commit all transform updates for actors and objects
        updatePtrHolders();

        // Run dynamic body sim, broadphase updates etc. Drop excess backlog
        // rather than allowing a slow frame to create an unbounded catch-up
        // spiral on the next one.
        constexpr unsigned maxJoltStepsPerFrame = 10;
        unsigned joltSteps = 0;
#ifdef OPENMW_RAGDOLL_DEV_API
        // Bounded scalar diagnostics only. Distinguish the command consumed by
        // this step from the steering/controller result prepared after it.
        static const auto stepTrace_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_STEP_TRACE") != nullptr; });
        static const auto& stepTrace = stepTrace_tuned.get();
        // Row budget per process. The historical 5400 covers ~40 s of one
        // walker; a play session records several runs, and a fall after
        // the budget cannot be rebuilt offline (2026-09-12). A numeric
        // value of the variable above 1 is the budget.
        static const auto stepTraceBudget_tuned = RagdollCore::tuned<unsigned>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_STEP_TRACE");
            const unsigned long parsed = text ? std::strtoul(text, nullptr, 10) : 0;
            return parsed > 1 ? static_cast<unsigned>(parsed) : 5400u;
        });
        static const auto& stepTraceBudget = stepTraceBudget_tuned.get();
        static unsigned stepTraceRows = 0;
        const auto traceStep = [&](const char* phase) {
            if (!stepTrace || stepTraceRows >= stepTraceBudget)
                return;
            for (const auto& [ref, entry] : mPassiveRagdolls)
            {
                if (!entry->mPowered || stepTraceRows >= stepTraceBudget)
                    continue;
                ++stepTraceRows;
                const auto raw = entry->mRawTargetHorizontalVelocity;
                const auto filtered = entry->mTargetHorizontalVelocity;
                const auto facing = entry->mSteering.facing();
                const auto support = entry->mSupportEstimate;
                RagdollCore::RagdollTelemetry telemetry;
                const bool measured = entry->mRagdoll->captureTelemetry(telemetry) && telemetry.mTotalMass > 0;
                if (phase[0] == 'i' && entry->mFixedSteps == 0 && measured
                    && RagdollCore::tuningText("OPENMW_RAGDOLL_REPLAY_POSE_TRACE"))
                    for (std::size_t body = 0; body < telemetry.mBodies.size(); ++body)
                    {
                        const auto& b = telemetry.mBodies[body];
                        Log(Debug::Info) << std::dec << std::setprecision(17)
                            << "RAGDOLL_REPLAY_INITIAL step=" << mJoltStepSerial << " body=" << body
                            << " body_id=" << entry->mRagdoll->getBodyId(body).GetIndexAndSequenceNumber()
                            << " player=" << (entry->getPtr() == MWMechanics::getPlayer())
                            << " position=" << b.mPosition.GetX() << ',' << b.mPosition.GetY() << ',' << b.mPosition.GetZ()
                            << " rotation=" << b.mRotation.GetX() << ',' << b.mRotation.GetY() << ',' << b.mRotation.GetZ() << ',' << b.mRotation.GetW()
                            << " velocity=" << b.mMotion.mLinearVelocity.GetX() << ',' << b.mMotion.mLinearVelocity.GetY() << ',' << b.mMotion.mLinearVelocity.GetZ()
                            << " angular_velocity=" << b.mMotion.mAngularVelocity.GetX() << ',' << b.mMotion.mAngularVelocity.GetY() << ',' << b.mMotion.mAngularVelocity.GetZ();
                    }
                // Identify changes in the animation/reference input separately
                // from measured physics. This is not a hash of the final motor
                // command, which may also contain gait, arm and load corrections.
                std::uint64_t basePoseHash = 14695981039346656037ull;
                const auto hashScalar = [&](auto value) {
                    for (const auto byte : std::bit_cast<std::array<std::uint8_t, sizeof(value)>>(value))
                    {
                        basePoseHash ^= byte;
                        basePoseHash *= 1099511628211ull;
                    }
                };
                for (const auto& transform : entry->motorBasePose().mBodyTransforms)
                {
                    hashScalar(transform.mTranslation.GetX());
                    hashScalar(transform.mTranslation.GetY());
                    hashScalar(transform.mTranslation.GetZ());
                    hashScalar(transform.mRotation.GetX());
                    hashScalar(transform.mRotation.GetY());
                    hashScalar(transform.mRotation.GetZ());
                    hashScalar(transform.mRotation.GetW());
                }
                const auto rootPosition = entry->mRagdoll->getReferencePosition();
                const auto rootForward = entry->mRagdoll->getWorldVector(JPH::Vec3::sAxisY());
                const auto rootForwardValue = rootForward.value_or(JPH::Vec3::sZero());
                // Opt-in replay sidecar: capture the consumed reference, including
                // startup before Lua can begin recording. Animation channels alone
                // do not represent the frozen locomotion base used by this path.
                if (phase[0] == 'a' && RagdollCore::tuningText("OPENMW_RAGDOLL_REPLAY_POSE_TRACE"))
                {
                    const auto& pose = entry->motorBasePose().mBodyTransforms;
                    for (std::size_t body = 0; body < pose.size(); ++body)
                    {
                        const auto& t = pose[body];
                        Log(Debug::Info) << std::dec << std::setprecision(17)
                            << "RAGDOLL_REPLAY_BASE step=" << mJoltStepSerial
                            << " player=" << (entry->getPtr() == MWMechanics::getPlayer())
                            << " body=" << body
                            << " position=" << t.mTranslation.GetX() << ',' << t.mTranslation.GetY() << ',' << t.mTranslation.GetZ()
                            << " rotation=" << t.mRotation.GetX() << ',' << t.mRotation.GetY() << ',' << t.mRotation.GetZ() << ',' << t.mRotation.GetW();
                    }
                }
                JPH::Vec3 velocity = JPH::Vec3::sZero();
                if (measured)
                    for (const auto& body : telemetry.mBodies)
                        velocity += body.mMotion.mLinearVelocity * (body.mMass / telemetry.mTotalMass);
                // The reference-walker fields (stage, placement, foot loads,
                // requested acceleration) keep their columns for the trace
                // readers; with that walker removed they hold its idle values.
                const RagdollCore::ReferenceGaitState idleReference;
                const auto load = [&](const std::optional<std::size_t>& foot) {
                    return measured && foot ? 0.0 : -1.0;
                };
                const JPH::Vec3 requested = JPH::Vec3::sZero();
                const auto budgeted = entry->mJointSupportControllerOutput
                    ? entry->mJointSupportControllerOutput->mAcceleration : JPH::Vec3::sZero();
                const auto hullDelta = entry->mJointSupportControllerOutput
                    ? entry->mJointSupportControllerOutput->mHullProjectionDelta : std::nullopt;
                Log(Debug::Info) << std::dec << "RAGDOLL_STEP phase=" << phase << " integer_radix=10"
                    << " actor=" << entry->getPtr().getCellRef().getRefId()
                    << " player=" << (entry->getPtr() == MWMechanics::getPlayer())
                    << " render=" << frameNumber << " step=" << mJoltStepSerial
                    << " substep=" << joltSteps << " render_dt=" << dt << " physics_dt=" << mPhysicsDt
                    << " backlog=" << mTimeAccumJolt
                    << " raw=" << raw.GetX() << ',' << raw.GetY() << ',' << raw.GetZ()
                    << " filtered=" << filtered.GetX() << ',' << filtered.GetY() << ',' << filtered.GetZ()
                    << " facing=" << facing.GetX() << ',' << facing.GetY() << ',' << facing.GetZ()
                    << " facing_request=" << entry->mRawFacingRequest.GetX() << ','
                    << entry->mRawFacingRequest.GetY() << ',' << entry->mRawFacingRequest.GetZ()
                    << " mode=" << (entry->mControllerOutput ? static_cast<int>(entry->mControllerOutput->mMode) : -1)
                    << " reference_stage=" << static_cast<int>(idleReference.mStage)
                    << " reference_phase=" << static_cast<int>(idleReference.mPhase)
                    << " reference_time=" << idleReference.mTimeInStage
                    << " pelvis_height=" << (entry->mControllerInput ? entry->mControllerInput->mPelvisHeight : -1)
                    << " placement_known=0 placement=0,0,0 coordinated_pose=0 feasibility_clamped=0"
                    << " gait_profile_hash=" << RagdollCore::hashReferenceGaitProfile(entry->mExperimentalReferenceProfile)
                    << " left=" << (support ? static_cast<int>(support->mLeftFootSupported) : -1)
                    << " right=" << (support ? static_cast<int>(support->mRightFootSupported) : -1)
                    << " com_supported=" << (support ? static_cast<int>(support->mCenterOfMassSupported) : -1)
                    << " motors=" << entry->mJointMotorsApplied
                    << " joint_motor_peak_utilization=" << entry->mJointMotorPeakUtilization
                    << " motor_stage=" << RagdollCore::gaitActuationStageName(entry->mMotorActuationStage)
                    << " upright_stage=" << RagdollCore::gaitActuationStageName(entry->mUprightActuationStage)
                    << " measured=" << measured
                    << " velocity=" << velocity.GetX() << ',' << velocity.GetY() << ',' << velocity.GetZ()
                    << " left_load=" << load(entry->mLeftFootBodyIndex)
                    << " right_load=" << load(entry->mRightFootBodyIndex)
                    << " request_valid=0"
                    << " base_pose_hash=" << basePoseHash
                    << " root=" << rootPosition.GetX() << ',' << rootPosition.GetY() << ',' << rootPosition.GetZ()
                    << " root_forward_valid=" << rootForward.has_value()
                    << " root_forward=" << rootForwardValue.GetX() << ',' << rootForwardValue.GetY()
                    << ',' << rootForwardValue.GetZ()
                    << " requested_accel=" << requested.GetX() << ',' << requested.GetY() << ',' << requested.GetZ()
                    << " support_valid=" << entry->mJointSupportControllerOutput.has_value()
                    << " support_applied=" << entry->mJointSupportControlApplied
                    << " animation_assisted=" << entry->mAnimationAssistedActuation
                    // The removed animation-assisted routes' external pelvis
                    // assist, heading reference and stance anchor columns stay
                    // at their idle values so existing log readers still parse.
                    << " external_assist=0 external_assist_saturated=0"
                    << " external_assist_requested_force=0,0,0 external_assist_force=0,0,0"
                    << " external_assist_requested_tilt_torque=0,0,0"
                    << " external_assist_requested_yaw_torque=0,0,0 external_assist_torque=0,0,0"
                    << " external_assist_torque_saturated=0 assisted_heading_known=0"
                    << " coherent_reference_phase=0 coherent_stance=0 coherent_handoff_weight=0"
                    << " coherent_translation_allowed=0 coherent_anchor_body=-1"
                    << " coherent_position_anchor_known=0 coherent_position_anchor=0,0,0"
                    << " coherent_position_error=0,0,0 coherent_position_correction=0,0,0"
                    << " coherent_position_saturated=0 coherent_position_ik=0"
                    << " physical_yaw=0 desired_yaw=0 assisted_reference_yaw=0"
                    << " assisted_reference_yaw_rate=0 assisted_reference_yaw_acceleration=0"
                    << " physical_yaw_rate=0"
                    << " locomotion_interrupted=" << entry->mLocomotionInterrupted
                    << " hull_known=" << hullDelta.has_value()
                    << " hull_delta=" << (hullDelta ? hullDelta->GetX() : 0)
                    << ',' << (hullDelta ? hullDelta->GetY() : 0)
                    << " friction_limited=" << (entry->mJointSupportControllerOutput
                        && entry->mJointSupportControllerOutput->mFrictionLimited)
                    << " budgeted_accel=" << budgeted.GetX() << ',' << budgeted.GetY() << ',' << budgeted.GetZ();
                // Same bounded trace window, after all command overrides and
                if (phase[0] == 'a' && entry->mFixedSteps < 16
                    && RagdollCore::tuningText("OPENMW_RAGDOLL_REPLAY_POSE_TRACE"))
                    for (std::size_t body = 1; body < entry->mRagdoll->getBodyCount(); ++body)
                    {
                        RagdollCore::MotorTargetTelemetry motor;
                        if (!entry->mRagdoll->captureMotorTargetTelemetry(body, motor)) continue;
                        const auto q = motor.mTargetOrientation.value_or(JPH::Quat::sIdentity());
                        const auto v = motor.mTargetAngularVelocity;
                        Log(Debug::Info) << std::dec << std::setprecision(17)
                            << "RAGDOLL_STARTUP_COMMAND step=" << mJoltStepSerial << " body=" << body
                            << " target=" << q.GetX() << ',' << q.GetY() << ',' << q.GetZ() << ',' << q.GetW()
                            << " hinge=" << motor.mTargetHingeAngle.value_or(0)
                            << " velocity=" << v.GetX() << ',' << v.GetY() << ',' << v.GetZ()
                            << " stiffness_mode=" << motor.mSpring.mStiffnessMode
                            << " spring=" << motor.mSpring.mFrequencyOrStiffness << " damping=" << motor.mSpring.mDamping;
                    }
                // Same bounded trace window, after all command overrides and
                // before Jolt Update. Identify joints by body, not by a role
                // which changes at touchdown. Angles are radians; quaternion
                // order is XYZW in the reported constraint/world target space.
                if (phase[0] == 'a' && entry->mLeftLeg && entry->mRightLeg)
                    for (const auto* leg : { &*entry->mLeftLeg, &*entry->mRightLeg })
                        for (const auto body : { leg->mHip, leg->mKnee, leg->mFoot })
                        {
                            RagdollCore::MotorTargetTelemetry motor;
                            const bool valid = entry->mRagdoll->captureMotorTargetTelemetry(body, motor);
                            const auto current = motor.mCurrentOrientation.value_or(JPH::Quat::sIdentity());
                            const auto target = motor.mTargetOrientation.value_or(JPH::Quat::sIdentity());
                            std::optional<float> nativeClamp;
                            if (valid && !motor.mWorldOrientation && motor.mRequestedOrientation)
                                if (const auto geometry = entry->mRagdoll->getSwingTwistJointGeometry(body))
                                {
                                    const auto requested = (geometry->mParentFrame * *motor.mRequestedOrientation
                                        * geometry->mChildFrame.Conjugated()).Normalized();
                                    if (const auto limited = RagdollCore::clampSwingTwistJointOrientation(
                                            *geometry, JPH::Quat::sIdentity(), requested))
                                    {
                                        const auto delta = (requested * limited->Conjugated()).Normalized();
                                        nativeClamp = 2 * std::atan2(delta.GetXYZ().Length(), std::abs(delta.GetW()));
                                    }
                                }
                            Log(Debug::Info) << std::dec << "RAGDOLL_LEG_COMMAND integer_radix=10 actor=" << entry->getPtr().getCellRef().getRefId()
                                << " step=" << mJoltStepSerial << " body=" << body
                                << " valid=" << valid << " motors=" << entry->mJointMotorsApplied
                                << " stage=" << static_cast<int>(RagdollCore::ReferenceGaitState{}.mStage)
                                << " stance=" << static_cast<int>(RagdollCore::ReferenceGaitState{}.mPhase)
                                << " world=" << motor.mWorldOrientation
                                << " orientation_known=" << (motor.mCurrentOrientation && motor.mTargetOrientation)
                                << " current=" << current.GetX() << ',' << current.GetY() << ',' << current.GetZ() << ',' << current.GetW()
                                << " target=" << target.GetX() << ',' << target.GetY() << ',' << target.GetZ() << ',' << target.GetW()
                                // Reference-walker columns kept for trace readers.
                                << " load_torque_known=0 load_torque=0,0,0"
                                << " hinge_known=" << (motor.mCurrentHingeAngle && motor.mTargetHingeAngle)
                                << " hinge_current=" << motor.mCurrentHingeAngle.value_or(0)
                                << " hinge_target=" << motor.mTargetHingeAngle.value_or(0)
                                << " error=" << motor.mAngularError
                                // Submitted-to-final change may also include
                                // later overrides. It is not a limit-only metric.
                                << " target_adjustment=" << motor.mTargetClampDelta
                                << " native_limit_known=" << nativeClamp.has_value()
                                << " native_limit_clamp=" << nativeClamp.value_or(0)
                                << " stiffness_mode=" << motor.mSpring.mStiffnessMode
                                << " spring=" << motor.mSpring.mFrequencyOrStiffness
                                << " damping=" << motor.mSpring.mDamping;
                        }
            }
        };
#endif
        while (!skipSimulation && mTimeAccumJolt >= mPhysicsDt && joltSteps < maxJoltStepsPerFrame)
        {
            ++joltSteps;
            mTimeAccumJolt -= mPhysicsDt;
#ifdef OPENMW_RAGDOLL_DEV_API
            traceStep("input");
#endif
            stepPendingShoves(mPhysicsDt);
            stepDeathDrag(mPhysicsDt);
            stepNetPuppet();
            // PGC-03 D13: the nearest other body within 70 units of each
            // powered one (last step's situations), for a hand to catch.
            for (auto& [ref, entry] : mPassiveRagdolls)
            {
                entry->mPersonNear.reset();
                if (!entry->mPowered || !entry->mSituationKnown)
                    continue;
                float nearest = 70.0f;
                for (auto& [otherRef, other] : mPassiveRagdolls)
                {
                    if (other.get() == entry.get() || !other->mSituationKnown)
                        continue;
                    const JPH::Vec3 d(other->mSituationCom - entry->mSituationCom);
                    const float flat = std::hypot(d.GetX(), d.GetY());
                    if (flat > 1.0f && flat < nearest && std::abs(d.GetZ()) < 60.0f)
                    {
                        nearest = flat;
                        const JPH::Vec3 facing(-d.GetX() / flat, -d.GetY() / flat, 0.0f);
                        entry->mPersonNear = std::make_pair(
                            other->mSituationCom + JPH::RVec3(facing * 14.0f) + JPH::RVec3(0.0, 0.0, 22.0), facing);
                    }
                }
            }
            for (auto& [ref, entry] : mPassiveRagdolls)
            {
                if (entry->mCurrentPhysicsPoseCaptured)
                    entry->mPreviousPhysicsPose = entry->mCurrentPhysicsPose;
                entry->beginContactStep(mJoltStepSerial, mPhysicsDt);
                // The actor's position is written once a rendered frame,
                // after the steps: new on the frame's first step only.
                entry->updateSituation(mPhysicsDt, joltSteps == 1);
                entry->stepWeaponImpact(mPhysicsDt, mPhysicsSystem.GetBodyInterface());
                entry->trackDuelMotion(mPhysicsDt, mPhysicsSystem.GetBodyInterface());
                entry->stepDying(mPhysicsDt);
                entry->stepReactionExtras(mPhysicsDt);
                entry->mJointMotorsApplied = false;
                entry->mMotorActuationStage = RagdollCore::GaitActuationStage::Inactive;
                entry->mStanceHipAxisReleased = false;
                entry->mSwingHipAxisReleased = false;
                entry->mUprightActuationStage = RagdollCore::GaitActuationStage::Inactive;
                entry->mAppliedUprightControllerOutput.reset();
                entry->mUprightControlApplied = false;
                entry->mAnimationAssistedActuation = false;
                entry->mAdaptedMotorTargetCaptured = false;
                entry->mJointSupportControlApplied = false;
                entry->mJointSupportContact = false;
                entry->mJointSupportMotorUtilization = 0;
                entry->mJointMotorPeakUtilization = 0;
                entry->mSwingFootTrajectoryModel.reset();
                entry->mSwingFootResolvedModel.reset();
                entry->mSwingFootPoseModel.reset();
                entry->mSwingFootControllerWorldTarget.reset();
                entry->mSwingHipMotorTarget.reset();
                entry->mSwingKneeMotorTarget.reset();
                entry->mSwingAnkleMotorTarget.reset();
                entry->mSwingFootControllerOutput.reset();
                entry->mSwingHipMotorUtilization.reset();
                entry->mSwingKneeMotorUtilization.reset();
                entry->mSwingAnkleMotorUtilization.reset();
                entry->mSwingFootControlApplied = false;
                entry->mPreLiftSwingUnloadForce = 0.0f;
                if (entry->mGrab && entry->mGrab->mBody < entry->mRagdoll->getBodyCount())
                {
                    // Manual grab: a spring-damper toward the anchor, full
                    // force at 20 units of separation, capped at the grab
                    // strength; the balance controller sees an external force.
                    JPH::BodyInterface& bodyInterface = mPhysicsSystem.GetBodyInterface();
                    const JPH::BodyID bodyId = entry->mRagdoll->getBodyId(entry->mGrab->mBody);
                    const JPH::Vec3 offset(entry->mGrab->mAnchor - bodyInterface.GetCenterOfMassPosition(bodyId));
                    const JPH::Vec3 velocity = bodyInterface.GetLinearVelocity(bodyId);
                    const float stiffness = entry->mGrab->mMaxForce / 20.0f;
                    JPH::Vec3 force = offset * stiffness - velocity * (stiffness * 0.05f);
                    const float length = force.Length();
                    if (length > entry->mGrab->mMaxForce)
                        force *= entry->mGrab->mMaxForce / length;
                    if (!force.IsNearZero())
                        bodyInterface.AddForce(bodyId, force, JPH::EActivation::Activate);
                }
                if (entry->mPowered)
                {
                    // A hit stun suspends the pose drive entirely: holding the
                    // authored pose while the impulse plays out damps the
                    // reaction into a slow lean, which reads as a delayed hit.
                    // With the motors released the body takes the impulse at
                    // once and the controller decides on the way out.
                    // A hit stun keeps the drive for the absorbed and
                    // staggering bands so the legs can step under the body;
                    // only an expected knock-down releases the motors
                    // entirely. The stagger band masks the upper body at the
                    // drive call, so the reaction reads instantly.
                    bool drive = !entry->mControllerOutput || entry->mControllerOutput->mDriveJointMotors;
                     bool protectiveArmDrive = false;
                    // A body that is falling, down or still getting up cannot
                    // swing: the AI keeps driving attacks through the fall and
                    // the animation hit path lands them from the floor.
                    if (entry->mPowered && entry->mControllerOutput)
                    {
                        const auto mode = entry->mControllerOutput->mMode;
                        // An animation-held (kinematic) root is not falling, even
                        // when the controller mode transiently says so during an
                        // attack; the protective arm mask is for dynamic falls.
                        protectiveArmDrive = !entry->mRagdoll->isRootKinematic()
                            && RagdollCore::protectiveArmDriveEnabled(mode,
                            entry->mLeftArmTarget && entry->mLeftArmTarget->mBrace,
                            entry->mRightArmTarget && entry->mRightArmTarget->mBrace);
                        if (protectiveArmDrive)
                            drive = true;
                        if (mode == RagdollCore::ActiveControllerMode::Falling
                            || mode == RagdollCore::ActiveControllerMode::Fallen
                            || mode == RagdollCore::ActiveControllerMode::GettingUp
                            || mode == RagdollCore::ActiveControllerMode::BlendBack)
                        {
                            auto& stats = entry->getPtr().getClass().getCreatureStats(entry->getPtr());
                            if (stats.getAttackingOrSpell())
                                stats.setAttackingOrSpell(false);
                        }
                    }
                    // A fallen actor tries to get back up on its own. The
                    // controller still decides when it can: it waits out its
                    // own recovery delay and requires the body to have
                    // settled, so this only removes the need for an external
                    // request that nothing in ordinary gameplay issues.
                    if (mPassiveRagdollProfile && mPassiveRagdollProfile->mActiveController.mAutomaticRecovery
                        && entry->mPowered && !entry->mRecoveryRequested && entry->mControllerOutput
                        && entry->mControllerOutput->mMode == RagdollCore::ActiveControllerMode::Fallen)
                        entry->mRecoveryRequested = true;
                    // Hold the get-up's own final pose through the settle
                    // phase. The live animation returns to the idle clip the
                    // moment the get-up completes, and driving that (while the
                    // upright assist fights for the root) jerks the freshly
                    // risen body back down before the recovery can latch.
                    if (mPassiveRagdollProfile
                        && mPassiveRagdollProfile->mActiveController.mLocomotionRoute
                            == RagdollCore::LocomotionRoute::KinematicRoot
                        && entry->mControllerOutput && entry->mMotorTargetCaptured)
                    {
                        const auto mode = entry->mControllerOutput->mMode;
                        if (mode == RagdollCore::ActiveControllerMode::GettingUp)
                            entry->mRecoveryReferencePose = entry->mMotorTargetPose;
                        else if (mode != RagdollCore::ActiveControllerMode::BlendBack)
                            entry->mRecoveryReferencePose.reset();
                    }
#ifdef OPENMW_RAGDOLL_DEV_API
                    if (RagdollCore::tuningText("OPENMW_RAGDOLL_RECOVERY_TRACE"))
                    {
                        static unsigned recoveryTraceSamples = 0;
                        if (recoveryTraceSamples < 20000)
                        {
                            ++recoveryTraceSamples;
                            const auto recoveryDirection = entry->mControllerOutput
                                    && entry->mControllerOutput->mRecoveryDirection
                                ? (*entry->mControllerOutput->mRecoveryDirection
                                        == RagdollCore::RecoveryDirection::FaceUp
                                    ? 1 : 0)
                                : -1;
                            Log(Debug::Info) << std::dec << std::setprecision(4)
                                << "RAGDOLL_RECOVERY step=" << mJoltStepSerial
                                << " serial=" << entry->mSerial
                                << " player=" << (entry->getPtr() == MWMechanics::getPlayer() ? 1 : 0)
                                << " actor=" << entry->getPtr().getCellRef().getRefId()
                                << " mode=" << (entry->mControllerOutput
                                        ? static_cast<int>(entry->mControllerOutput->mMode) : -1)
                                << " mode_time=" << (entry->mControllerOutput
                                        ? entry->mControllerOutput->mTimeInMode : 0.0f)
                                << " requested=" << entry->mRecoveryRequested
                                << " started=" << entry->mRecoveryAnimationStarted
                                << " finished=" << entry->mRecoveryAnimationFinished
                                << " handover=" << entry->mRecoveryHandoverPending
                                << " face_up=" << recoveryDirection
                                << " up=" << (entry->mControllerInput
                                        ? entry->mControllerInput->mPelvisUpDot : -2.0f)
                                << " torso=" << (entry->mControllerInput
                                        ? entry->mControllerInput->mTorsoUpDot : -2.0f)
                                << " com_speed=" << (entry->mControllerInput
                                        ? entry->mControllerInput->mCenterOfMassVelocity.Length() : -1.0f)
                                << " feet=" << (entry->mSupportEstimate
                                        ? static_cast<int>(entry->mSupportEstimate->mLeftFootSupported)
                                            + static_cast<int>(entry->mSupportEstimate->mRightFootSupported)
                                        : -1)
                                << " upright=" << entry->mUprightControlApplied
                                << " have_upright=" << (entry->mUprightControllerOutput ? 1 : 0)
                                << " have_gait=" << (entry->mGaitActuationDecision ? 1 : 0)
                                << " use_support=" << (entry->mGaitActuationDecision
                                        ? (entry->mGaitActuationDecision->mUseJointSupportControl ? 1 : 0)
                                        : -1)
                                << " root_kin=" << (entry->mRagdoll->isRootKinematic() ? 1 : 0)
                                << " drive=" << (entry->mControllerOutput
                                        ? (entry->mControllerOutput->mDriveJointMotors ? 1 : 0)
                                        : -1)
                                << " fault=" << entry->mPoweredControlFailureLine
                                << " pforce=" << (entry->mAppliedUprightControllerOutput
                                        ? entry->mAppliedUprightControllerOutput->mPelvisForce.Length()
                                        : -1.0f)
                                << " tup2=" << [&] {
                                       if (entry->mTorsoBodyIndex)
                                           if (const auto rotation = entry->mRagdoll->getWorldBodyRotation(
                                                   entry->mMotorTargetPose, *entry->mTorsoBodyIndex))
                                               return (*rotation * entry->mTorsoLocalUp).GetZ();
                                       return 0.0f;
                                   }();
                        }
                    }
#endif
                    if (mPassiveRagdollProfile)
                    {
                        if (entry->getPtr() == MWMechanics::getPlayer())
                            updatePlayerStrikeAim(*entry);
                        entry->stepWeaponSwing(mPhysicsDt, mRagdollWeaponSwing,
                            entry->mRuntimeProfile ? *entry->mRuntimeProfile : *mPassiveRagdollProfile);
                        entry->publishContactState();
                    }
                    // An AI strike follows the vanilla attack animation. The
                    // striking arm keeps the animation's joint rotations that
                    // are already in the drive pose (applyArmTargets leaves it
                    // alone while the strike runs) instead of chasing a solved
                    // hand position, and the swing motors give both arms the
                    // authority to track the fast release. When the clip cannot
                    // reach what the strike is aimed at - a prone victim below
                    // the swing, an elevated one above it - the target is
                    // biased toward the aim point, clamped to the arm's reach,
                    // so the blow still arrives.
                    if (entry->mAiStrikeActive && entry->mRightArm && entry->mRightArmTarget
                        && entry->mAiStrikeAimPoint && entry->mCurrentPhysicsPoseCaptured && !entry->mTip.mActive)
                    {
                        if (const auto target = entry->strikeAssistTarget(*entry->mAiStrikeAimPoint))
                            entry->mRightArmTarget->mWorldTarget = *target;
                    }
#ifdef OPENMW_RAGDOLL_DEV_API
                    // Dev A/B: extra velocity solver iterations while an AI
                    // strike follows the animation on a kinematic root, so a
                    // stiff shoulder spring can drag the whole arm chain.
                    if (const char* spec = RagdollCore::tuningText("OPENMW_RAGDOLL_STRIKE_SOLVER_STEPS"))
                    {
                        const bool wanted = entry->mAiStrikeActive && entry->mRagdoll->isRootKinematic();
                        const std::uint32_t steps = wanted ? static_cast<std::uint32_t>(std::atoi(spec))
                                                           : (entry->mWeapon ? entry->mWeapon->mVelocitySolverSteps : 0u);
                        entry->mRagdoll->setPoweredVelocitySolverSteps(steps);
                    }
#endif
                    if (mPassiveRagdollProfile && entry->mLeftArm)
                    {
                        // Only on the kinematic root: an opened shoulder changes
                        // a dynamic body's angular momentum, which was measured
                        // to topple the reference walker.
                        const bool wanted = entry->mAiStrikeActive && entry->mSwingArmIsRight
                            && entry->mRagdoll->isRootKinematic();
                        if (wanted != entry->mAiStrikeOffArmMotorsActive)
                        {
                            const auto& rigProfile
                                = entry->mRuntimeProfile ? *entry->mRuntimeProfile : *mPassiveRagdollProfile;
                            entry->requestArmMotors(rigProfile, *entry->mLeftArm, mRagdollWeaponSwing, wanted, wanted);
                            entry->mAiStrikeOffArmMotorsActive = wanted;
                            // Restoring the authored limits must not close a
                            // shoulder the brace logic is holding open.
                            if (!wanted && entry->mLeftShoulderOpen
                                && entry->mLeftArm->mShoulder < rigProfile.mBodies.size()
                                && rigProfile.mBodies[entry->mLeftArm->mShoulder].mConstraint)
                            {
                                const auto& active = rigProfile.mActiveController;
                                entry->mRagdoll->setSwingTwistJointLimits(entry->mLeftArm->mShoulder,
                                    active.mBraceShoulderNormalHalfCone, active.mBraceShoulderPlaneHalfCone,
                                    -active.mBraceShoulderTwistLimit, active.mBraceShoulderTwistLimit);
                            }
                        }
                    }
                    if (mPassiveRagdollProfile)
                    {
                        entry->updateWeaponSupport(mPhysicsSystem, mPhysicsDt, mJoltStepSerial, mRagdollWeaponSwing,
                            entry->mRuntimeProfile ? *entry->mRuntimeProfile : *mPassiveRagdollProfile);
                        entry->updateBraceArms(mPassiveRagdollProfile->mActiveController);
                        entry->updateSupportReach(mPassiveRagdollProfile->mActiveController);
                        entry->updateMantle(mPhysicsDt, mPassiveRagdollProfile->mActiveController);
                        entry->updateFallGrab(mPassiveRagdollProfile->mActiveController);
                        entry->updateDropRoll(mPhysicsDt, mPassiveRagdollProfile->mActiveController);
                        entry->watchArmsBehind(mPhysicsDt);
                    }
                    if (mPassiveRagdollProfile)
                    {
                        entry->updatePlayerGuard(mRagdollWeaponSwing,
                            entry->mRuntimeProfile ? *entry->mRuntimeProfile : *mPassiveRagdollProfile);
                        entry->updateNpcOffHand(
                            entry->mRuntimeProfile ? *entry->mRuntimeProfile : *mPassiveRagdollProfile);
                        entry->updateFirstPersonWeaponRest(mRagdollWeaponSwing);
                        // The arms' motors, once (the motor layer).
                        entry->flushArmMotors();
                        entry->stepArmTargets(
                            mPhysicsDt, entry->mRuntimeProfile ? *entry->mRuntimeProfile : *mPassiveRagdollProfile);
                        // RXN-02 V2: the reaction controller's step; its snaps
                        // on the trunk and the head (never past the head cap).
                        {
                            const auto [trunkSnap, headSnap] = entry->stepReactionV2(mPhysicsDt);
                            JPH::BodyInterface& snapBodies = mPhysicsSystem.GetBodyInterface();
                            const auto snap = [&](std::optional<std::size_t> index, JPH::Vec3Arg add, float cap) {
                                if (!index || add.LengthSq() < 1e-8f || *index >= entry->mRagdoll->getBodyCount())
                                    return;
                                const JPH::BodyID body = entry->mRagdoll->getBodyId(*index);
                                const JPH::Vec3 before = snapBodies.GetAngularVelocity(body);
                                JPH::Vec3 after = before + add;
                                if (after.Length() > cap)
                                    after = before.Length() >= cap ? before : after * (cap / after.Length());
                                snapBodies.SetAngularVelocity(body, after);
                            };
                            snap(entry->mTorsoBodyIndex, trunkSnap, 12.0f);
                            snap(entry->mHeadBodyIndex, headSnap, entry->mReactionV2.settings().mHeadSpinCap);
                        }
                    }
                    // Advance the standing hand-over blend once per fixed step;
                    // it only runs while standing drives the motors.
                    if (!drive
                        || (entry->mControllerOutput
                            && (entry->mControllerOutput->mMode == RagdollCore::ActiveControllerMode::GettingUp
                                || entry->mControllerOutput->mMode == RagdollCore::ActiveControllerMode::BlendBack)))
                        entry->mHandover.reset();
                    else
                        entry->mHandover.step(mPhysicsDt, entry->mExperimentalReferenceProfile);
                    const bool animationKinematicRootRoute = mPassiveRagdollProfile
                        && mPassiveRagdollProfile->mActiveController.mLocomotionRoute
                            == RagdollCore::LocomotionRoute::KinematicRoot;
                    // A held root takes absorbed and staggering hits on its
                    // feet: the limbs keep following the live animation, the
                    // struck body takes the physical impulse, the root recoils,
                    // and a stagger lets the arms and head go slack. Releasing
                    // every motor for the length of the stun leaves a held body
                    // hanging from its pelvis: it folds at the waist on every
                    // landed blow.
                    const bool heldRootReaction = animationKinematicRootRoute && entry->mLocomotionInterrupted
                        && entry->mRagdoll->isRootKinematic();
                    // Any other interruption (a released root, a Fall hit)
                    // removes the pose motors so the impact controller can
                    // decide whether the unassisted body braces or falls.
                    // A falling body keeps its motors with a protective pose
                    // (PassiveRagdollEntry::stepFallProtect, FALL_PROTECT).
                    if (mPassiveRagdollProfile
                        && entry->stepFallProtect(mPhysicsDt,
                            entry->mRuntimeProfile ? *entry->mRuntimeProfile : *mPassiveRagdollProfile,
                            animationKinematicRootRoute && entry->mLocomotionInterrupted && !heldRootReaction))
                    {
                        entry->mJointSupportControllerOutput.reset();
                        continue;
                    }
                    if (animationKinematicRootRoute && entry->mLocomotionInterrupted && !heldRootReaction)
                    {
                        entry->mJointSupportControllerOutput.reset();
                        entry->mRagdoll->beginActuationStep(false);
                        entry->mRagdoll->disableMotors();
                        continue;
                    }
                    const bool animationAssisted = mPassiveRagdollProfile && entry->mMotorTargetCaptured
                        && entry->mControllerOutput && entry->mUprightControllerOutput
                        && entry->mPelvisBodyIndex
                        && RagdollCore::shouldUseAnimationAssistedActuation(
                            mPassiveRagdollProfile->mActiveController.mLocomotionRoute,
                            entry->mControllerOutput->mMode, drive,
                            entry->mLocomotionInterrupted && !heldRootReaction);
                    if (animationAssisted)
                    {
                        // Kinematic-root route. Vanilla-faithful limbs: the
                        // game's own walk, run and combat clips already match
                        // its movement speed and the kinematic root follows
                        // the actor, so the limbs follow the live animation
                        // directly.
                        const auto& activeProfile = mPassiveRagdollProfile->mActiveController;
                        RagdollCore::RagdollPose animationTargetPose = entry->mMotorTargetPose;
                        entry->mJointSupportControllerOutput.reset();
                        entry->mRagdoll->beginActuationStep(protectiveArmDrive);
                        RagdollCore::RagdollPose drivePose = animationTargetPose;
                        if (!entry->mCurrentPhysicsPoseCaptured
                            || !RagdollCore::interpolateRagdollPoses(entry->mCurrentPhysicsPose,
                                animationTargetPose, activeProfile.mKinematicRootPoseTrackingBlend, drivePose))
                        {
                            entry->failPoweredControl(__LINE__);
                            continue;
                        }
                        // RXN-02 M1: each pose writer records its layer; the
                        // leases compose them (composeEnd, before the drive).
                        using Owner = RagdollCore::BodyOwner;
                        const auto gated = [&](std::optional<Owner> owner, auto&& write) {
                            entry->gateBegin(drivePose);
                            write();
                            entry->gateEnd(drivePose, owner);
                        };
                        entry->composeBegin(drivePose);
                        gated(std::nullopt, [&] { entry->applySwingBodyPose(drivePose); });
                        gated(std::nullopt,
                            [&] { entry->applyArmTargets(drivePose, entry->armSolveBase(animationTargetPose)); });
                        gated(std::nullopt, [&] { entry->applyWeaponFixtureYaw(drivePose); });
                        gated(Owner::Balance,
                            [&] { entry->applyBalanceLegCorrection(drivePose, animationTargetPose, activeProfile); });
                        gated(Owner::Locomotion, [&] {
                            entry->applyLegTravelYaw(drivePose, mPhysicsDt);
                            entry->applyRunBackKick(drivePose);
                            entry->applyFootGrounding(drivePose, mPhysicsDt);
                            entry->applySwingClearance(drivePose);
                            entry->applyFootfall(drivePose, animationTargetPose, mPhysicsDt);
                            // Jump tuck in the locomotion branch too (jumps run here).
                            entry->applyJumpTuck(drivePose, mPhysicsDt);
                            entry->applyLandingReach(drivePose);
                        });
                        entry->mAdaptedMotorTargetPose = drivePose;
                        entry->mAdaptedMotorTargetCaptured = true;
                        std::vector<std::size_t> slackBodies;
                        entry->gateBegin(drivePose);
                        entry->applyReflexPoseV2(drivePose);
                        entry->gateEnd(drivePose, RagdollCore::BodyOwner::Reaction);
                        entry->composeEnd(drivePose);
                        entry->mJointMotorsApplied = entry->driveBalanceObservedPose(drivePose, "animation-drive", slackBodies);
                        entry->mAnimationAssistedActuation = entry->mJointMotorsApplied;
                        // Head stabilisation (user 10-07: "the enemies still wiggle
                        // their heads ... too much, especially when running";
                        // STEADY-02: head spin p90 5.5 rad/s running, 2.1 walking):
                        // the head is driven to its pose's orientation in the world,
                        // not relative to the bouncing chest (people hold the head
                        // steady in space when running - the vestibulo-collic
                        // reflex), as the feet are. The pose carries the reaction
                        // reflex, so a blow still turns the head; snaps are added
                        // as velocity on top. NPCs only. OPENMW_RAGDOLL_HEAD_STABILIZE=1.
                        static const auto headStabilize_tuned = RagdollCore::tuned<bool>([] {
                            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HEAD_STABILIZE");
                            return text && std::atoi(text) != 0;
                        });
                        static const auto& headStabilize = headStabilize_tuned.get();
                        if (headStabilize && entry->mJointMotorsApplied && entry->mHeadBodyIndex
                            && entry->getPtr() != MWMechanics::getPlayer()
                            && std::find(slackBodies.begin(), slackBodies.end(), *entry->mHeadBodyIndex) == slackBodies.end())
                            if (const auto headTarget = entry->mRagdoll->getWorldBodyRotation(drivePose, *entry->mHeadBodyIndex))
                                entry->mRagdoll->driveMotorToWorldOrientation(*entry->mHeadBodyIndex, *headTarget, mPhysicsDt);
                        if (entry->mJointMotorsApplied && entry->mAiStrikeActive
                            && RagdollCore::tuningText("OPENMW_RAGDOLL_STRIKE_JOINT_FOLLOW")
                            && !RagdollCore::tuningText("OPENMW_RAGDOLL_STRIKE_NO_FEEDFORWARD"))
                            entry->applyStrikeArmFeedForward(mPhysicsDt);
                        else if (entry->mJointMotorsApplied && entry->mAnimationAttackMotors
                            && PassiveRagdollEntry::nativeAttackDrive().mFeedForward)
                            entry->applyStrikeArmFeedForward(mPhysicsDt, &drivePose);
                        else if (!entry->mAiStrikeActive)
                            entry->mAnimationVelocity.clear();
                        if (entry->mJointMotorsApplied && entry->mRagdoll->isRootKinematic())
                            entry->applyLegFeedForward(mPhysicsDt);
                        else
                            entry->mLegAnimationVelocity.clear();
                        if (!entry->mJointMotorsApplied)
                        {
                            entry->failPoweredControl(__LINE__);
                            continue;
                        }
                        entry->applyWeaponWeightCompensation(mPhysicsDt);
                        entry->stepGuardStiffness(mPhysicsDt);
                        entry->applyPlayerWeaponDrive(mPhysicsDt);
                        // The architecture base declares the root external
                        // while it is kinematic: the pose motors drive the
                        // limbs and the actor owns the root transform. Pose
                        // motors alone, however, lag a moving animation
                        // badly enough that the torso pitches back tens of
                        // degrees, so the upright controller's spine torque
                        // stays applied with its reaction grounded in the
                        // support feet. While the root is released (falling,
                        // fallen, getting up, settling) the pelvis must be
                        // driven again: every animation pose is relative to
                        // it, so without pelvis drive the get-up clip
                        // finishes while the body is still on the ground
                        // and walking never resumes.
                        entry->mUprightControlApplied = false;
                        if (entry->mUprightControllerOutput && entry->mPelvisBodyIndex
                            && entry->mTorsoBodyIndex && entry->mLeftLeg && entry->mRightLeg
                            && entry->mGaitActuationDecision)
                        {
                            RagdollCore::UprightControllerOutput control
                                = *entry->mUprightControllerOutput;
                            if (entry->mRagdoll->isRootKinematic())
                            {
                                control.mPelvisForce = JPH::Vec3::sZero();
                                control.mPelvisTorque = JPH::Vec3::sZero();
                                // The spine torque holds the chest to world up: under
                                // the held root it pulled against the animation's
                                // lean every step (chest 4-11 deg behind the running
                                // lean, rocking ~3x the animation's pitch, and the
                                // spine springs could not win; 2026-09-28).
                                // OPENMW_RAGDOLL_HELD_SPINE_UPRIGHT scales it (1 = old).
                                static const auto heldSpineUpright_tuned = RagdollCore::tuned<float>([] {
                                    const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HELD_SPINE_UPRIGHT");
                                    return text ? static_cast<float>(std::atof(text)) : 1.0f;
                                });
                                static const auto& heldSpineUpright = heldSpineUpright_tuned.get();
                                control.mSpineTorque *= heldSpineUpright;
                            }
                            const auto reactions = RagdollCore::selectGaitReactionBodies(
                                *entry->mGaitActuationDecision,
                                entry->mSupportEstimate.value_or(RagdollCore::SupportEstimate{}),
                                { entry->mLeftLeg->mFoot, entry->mRightLeg->mFoot },
                                entry->mLeftLeg->mHip, entry->mRightLeg->mHip,
                                entry->mControllerOutput
                                    && entry->mControllerOutput->mMode
                                        == RagdollCore::ActiveControllerMode::BlendBack);
                            if (reactions.mHorizontal.empty())
                            {
                                control.mPelvisForce.SetX(0.0f);
                                control.mPelvisForce.SetY(0.0f);
                            }
                            if (reactions.mVertical.empty())
                                control.mPelvisForce.SetZ(0.0f);
                            entry->mUprightControlApplied = entry->mRagdoll->applyUprightControl(control,
                                *entry->mPelvisBodyIndex, *entry->mTorsoBodyIndex,
                                reactions.mVertical, reactions.mHorizontal, activeProfile,
                                reactions.mAngular,
                                entry->mGaitActuationDecision->mLocomoting,
                                reactions.mSecondaryAngular);
                            if (!entry->mUprightControlApplied)
                                entry->failPoweredControl(__LINE__);
                            else
                                entry->mAppliedUprightControllerOutput = control;
                        }
                        continue;
                    }
                    // Observation computes horizontal control before advancing
                    // the gait clock. Motor policy must use the newly advanced
                    // phase, as the offline adapter does at actuation time.
                    if (mPassiveRagdollProfile)
                    {
                        RagdollCore::GaitActuationDecisionInput decisionInput;
                        decisionInput.mLocomotionRequested = !entry->mTargetHorizontalVelocity.IsNearZero();
                        decisionInput.mPelvisHeightSettled = entry->mPelvisHeightOutput.mSettled;
                        decisionInput.mDriveJointMotors = drive;
                        decisionInput.mRecovering = entry->mControllerOutput
                            && (entry->mControllerOutput->mMode == RagdollCore::ActiveControllerMode::GettingUp
                                || entry->mControllerOutput->mMode == RagdollCore::ActiveControllerMode::BlendBack);
                        decisionInput.mBothFeetSupported = entry->mSupportEstimate
                            && entry->mSupportEstimate->mLeftFootSupported
                            && entry->mSupportEstimate->mRightFootSupported;
                        decisionInput.mRequestedHorizontalVelocity = entry->mTargetHorizontalVelocity;
                        if (entry->mControllerInput)
                            decisionInput.mMeasuredHorizontalVelocity = entry->mControllerInput->mCenterOfMassVelocity;
                        decisionInput.mSwingTargetAvailable = entry->mSwingFootTarget.has_value();
                        decisionInput.mStanceFootSupported = entry->mSupportEstimate && entry->mGaitOutput
                            && entry->mGaitOutput->hasStance()
                            && (entry->mGaitOutput->leftFootIsStance()
                                    ? entry->mSupportEstimate->mLeftFootSupported
                                    : entry->mSupportEstimate->mRightFootSupported);
                        if (entry->mGaitOutput)
                            decisionInput.mGait = *entry->mGaitOutput;
                        entry->mGaitActuationDecision = RagdollCore::calculateGaitActuationDecision(
                            decisionInput, mPassiveRagdollProfile->mActiveController.mGait);
                        if (!entry->mGaitActuationDecision)
                        {
                            entry->failPoweredControl(__LINE__);
                            continue;
                        }
                    }
                    entry->mRagdoll->beginActuationStep(entry->mGaitActuationDecision
                        && entry->mGaitActuationDecision->mUseJointSupportControl);
                    if (drive && entry->mMotorTargetCaptured)
                    {
                        if (entry->mGaitActuationDecision)
                            entry->mMotorActuationStage = entry->mGaitActuationDecision->mStage;
                        const RagdollCore::RagdollPose& basePose
                            = entry->motorBasePose();
                        RagdollCore::RagdollPose standingPose = basePose;
                        if (entry->mStandingLegPose && entry->mPelvisBodyIndex && entry->mLeftLeg && entry->mRightLeg)
                        {
                            const std::array<std::size_t, 6> legs{ entry->mLeftLeg->mHip, entry->mLeftLeg->mKnee,
                                entry->mLeftLeg->mFoot, entry->mRightLeg->mHip, entry->mRightLeg->mKnee,
                                entry->mRightLeg->mFoot };
                            if (!RagdollCore::applyGaitMeasuredLegTargets(standingPose.mBodyTransforms,
                                    entry->mStandingLegPose->mBodyTransforms, *entry->mPelvisBodyIndex, legs,
                                    RagdollCore::GaitMeasuredPoseFrame::Joint))
                            {
                                entry->failPoweredControl(__LINE__);
                                continue;
                            }
                        }
                        RagdollCore::RagdollPose drivePose = standingPose;
                        if (entry->mHandover.active()
                            && !entry->mHandover.blendPose(standingPose, entry->mExperimentalReferenceProfile, drivePose))
                        {
                            entry->failPoweredControl(__LINE__);
                            continue;
                        }
                        if (entry->mGaitActuationDecision
                            && entry->mGaitActuationDecision->mPoseSource
                                == RagdollCore::GaitPoseSource::MeasuredSwingLeg
                            && entry->mGaitOutput && entry->mLeftLeg && entry->mRightLeg
                            && entry->mCurrentPhysicsPoseCaptured)
                        {
                            const RagdollLegChain& swingLeg
                                = entry->mGaitOutput->leftFootIsStance() ? *entry->mRightLeg : *entry->mLeftLeg;
                            const std::array<std::size_t, 3> chain{ swingLeg.mHip, swingLeg.mKnee, swingLeg.mFoot };
                            if (!entry->mPelvisBodyIndex || !RagdollCore::applyGaitMeasuredLegTargets(
                                    drivePose.mBodyTransforms, entry->mCurrentPhysicsPose.mBodyTransforms,
                                    *entry->mPelvisBodyIndex, chain, entry->mGaitActuationDecision->mMeasuredPoseFrame))
                            {
                                entry->failPoweredControl(__LINE__);
                                continue;
                            }
                        }
                        if (mPassiveRagdollProfile && entry->mGaitActuationDecision
                            && entry->mGaitActuationDecision->mApplyPreLiftSwingUnload && entry->mGaitOutput
                            && entry->mSupportEstimate && entry->mLeftLeg && entry->mRightLeg)
                        {
                            const RagdollLegChain& stanceLeg
                                = entry->mGaitOutput->leftFootIsStance() ? *entry->mLeftLeg : *entry->mRightLeg;
                            const RagdollLegChain& swingLeg
                                = entry->mGaitOutput->leftFootIsStance() ? *entry->mRightLeg : *entry->mLeftLeg;
                            RagdollCore::RagdollTelemetry telemetry;
                            if (!entry->mRagdoll->captureTelemetry(telemetry)
                                || swingLeg.mHip >= telemetry.mBodies.size()
                                || swingLeg.mKnee >= telemetry.mBodies.size()
                                || swingLeg.mFoot >= telemetry.mBodies.size())
                            {
                                entry->failPoweredControl(__LINE__);
                                continue;
                            }
                            const float swingLegMass = telemetry.mBodies[swingLeg.mHip].mMass
                                + telemetry.mBodies[swingLeg.mKnee].mMass
                                + telemetry.mBodies[swingLeg.mFoot].mMass;
                            const float bodyWeight
                                = telemetry.mTotalMass * RagdollCore::UnitModel::GravityUnitsPerSecondSquared;
                            const auto unload = RagdollCore::calculatePreLiftSwingUnloadForce(
                                *entry->mGaitActuationDecision, telemetry.mTotalMass, swingLegMass,
                                RagdollCore::UnitModel::GravityUnitsPerSecondSquared,
                                mPassiveRagdollProfile->mActiveController.mGait);
                            if (!unload)
                            {
                                entry->failPoweredControl(__LINE__);
                                continue;
                            }
                            const float unloadForce = *unload;
                            if (unloadForce > 0.0f
                                && !entry->mRagdoll->applyBodyForcePair(swingLeg.mFoot, stanceLeg.mFoot,
                                    JPH::Vec3(0.0f, 0.0f, unloadForce), bodyWeight))
                            {
                                entry->failPoweredControl(__LINE__);
                                continue;
                            }
                            entry->mPreLiftSwingUnloadForce = unloadForce;
                        }
                        if (mPassiveRagdollProfile && entry->mGaitOutput && entry->mGaitOutput->hasStance()
                            && entry->mGaitOutput->mStanceReady && entry->mSwingFootTarget
                            && entry->mSwingStartModel && entry->mLeftLeg && entry->mRightLeg && entry->mPelvisBodyIndex
                            && entry->mCurrentPhysicsPoseCaptured)
                        {
                            const RagdollLegChain& swingLeg
                                = entry->mGaitOutput->leftFootIsStance() ? *entry->mRightLeg : *entry->mLeftLeg;
                            const RagdollLegChain& stanceLeg
                                = entry->mGaitOutput->leftFootIsStance() ? *entry->mLeftLeg : *entry->mRightLeg;
                            const auto targetModel = entry->mRagdoll->getModelPosition(*entry->mSwingFootTarget);
                            if (targetModel && *entry->mPelvisBodyIndex < basePose.mBodyTransforms.size()
                                && *entry->mPelvisBodyIndex < entry->mCurrentPhysicsPose.mBodyTransforms.size()
                                && swingLeg.mHip < basePose.mBodyTransforms.size()
                                && swingLeg.mKnee < basePose.mBodyTransforms.size()
                                && swingLeg.mFoot < basePose.mBodyTransforms.size()
                                && swingLeg.mHip < entry->mCurrentPhysicsPose.mBodyTransforms.size()
                                && swingLeg.mKnee < entry->mCurrentPhysicsPose.mBodyTransforms.size())
                            {
                                JPH::Vec3 placementModel = *targetModel;
                                placementModel.SetZ(entry->mSwingStartModel->GetZ());
                                // Lateral foot spacing is not a knee-bend heading.
                                // Use the same travel direction as the core harness.
                                const auto travelModel = entry->mRagdoll->getModelVector(entry->mTargetHorizontalVelocity);
                                if (!travelModel)
                                {
                                    entry->failPoweredControl(__LINE__);
                                    continue;
                                }
                                const JPH::Vec3 bend
                                    = travelModel->IsNearZero() ? JPH::Vec3::sAxisY() : travelModel->Normalized();
                                RagdollCore::SwingFootTrajectoryInput trajectoryInput;
                                trajectoryInput.mLiftOffPosition = *entry->mSwingStartModel;
                                trajectoryInput.mPlacementPosition = placementModel;
                                trajectoryInput.mGait = *entry->mGaitOutput;
                                const auto footTrajectory = RagdollCore::calculateSwingFootTrajectoryState(
                                    trajectoryInput, mPassiveRagdollProfile->mActiveController.mGait);
                                if (!footTrajectory)
                                {
                                    entry->failPoweredControl(__LINE__);
                                    continue;
                                }
                                const JPH::Vec3& footTarget = footTrajectory->mPosition;
                                entry->mSwingFootTrajectoryModel = footTarget;
                                const RagdollCore::TwoBoneLegTargetInput legInput{ swingLeg.mHip, swingLeg.mKnee,
                                    swingLeg.mFoot, footTarget, bend,
                                    entry->mRagdoll->getHingeLegGeometry(swingLeg.mHip, swingLeg.mKnee, swingLeg.mFoot) };
                                const auto legPose = RagdollCore::applySwingLegPose(drivePose.mBodyTransforms,
                                    basePose.mBodyTransforms, entry->mCurrentPhysicsPose.mBodyTransforms,
                                    *entry->mPelvisBodyIndex, legInput,
                                    entry->mGaitActuationDecision.value_or(RagdollCore::GaitActuationDecision{}));
                                if (!legPose)
                                {
                                    entry->failPoweredControl(__LINE__);
                                    continue;
                                }
                                entry->mSwingFootResolvedModel = legPose->mFrame.mTargetFootPosition;
                                if (legPose->mApplied)
                                {
                                    entry->mSwingFootPoseModel = legPose->mFootPosition;
                                    RagdollCore::RagdollPose worldTargetPose = drivePose;
                                    worldTargetPose.mBodyTransforms[swingLeg.mFoot].mTranslation = footTarget;
                                    const auto targetWorld
                                        = entry->mRagdoll->getWorldBodyPosition(worldTargetPose, swingLeg.mFoot);
                                    if (targetWorld)
                                        entry->mSwingFootControllerWorldTarget = *targetWorld;
                                    RagdollCore::RagdollTelemetry telemetry;
                                    if (targetWorld && entry->mRagdoll->captureTelemetry(telemetry)
                                        && swingLeg.mHip < telemetry.mBodies.size()
                                        && swingLeg.mKnee < telemetry.mBodies.size()
                                        && swingLeg.mFoot < telemetry.mBodies.size())
                                    {
                                        RagdollCore::SwingFootControllerInput input;
                                        input.mCurrentPosition = telemetry.mBodies[swingLeg.mFoot].mPosition;
                                        input.mTargetPosition = *targetWorld;
                                        input.mCurrentVelocity
                                            = telemetry.mBodies[swingLeg.mFoot].mMotion.mLinearVelocity;
                                        const auto targetVelocity
                                            = entry->mRagdoll->getWorldVector(footTrajectory->mLinearVelocity);
                                        if (!targetVelocity)
                                        {
                                            entry->failPoweredControl(__LINE__);
                                            continue;
                                        }
                                        input.mTargetVelocity = *targetVelocity;
                                        input.mEffectiveMass = telemetry.mBodies[swingLeg.mHip].mMass
                                            + telemetry.mBodies[swingLeg.mKnee].mMass
                                            + telemetry.mBodies[swingLeg.mFoot].mMass;
                                        input.mFootLifted = entry->mGaitActuationDecision->mSwingFootInFlight;
                                        if (const auto control = RagdollCore::calculateSwingFootControl(
                                                input, mPassiveRagdollProfile->mActiveController.mGait))
                                        {
                                            entry->mSwingFootControllerOutput = *control;
                                            const JPH::Vec3 force = control->mForce;
                                            if (!force.IsNearZero())
                                            {
                                                const std::size_t reactionBody
                                                    = entry->mGaitActuationDecision->mSwingReactionThroughStanceFoot
                                                    ? stanceLeg.mFoot : swingLeg.mHip;
                                                entry->mSwingFootControlApplied
                                                    = entry->mRagdoll->applyBodyForcePair(swingLeg.mFoot, reactionBody,
                                                        force,
                                                        std::hypot(mPassiveRagdollProfile->mActiveController.mGait
                                                                       .mMaximumSwingFootForce,
                                                            mPassiveRagdollProfile->mActiveController.mGait
                                                                .mMaximumSwingFootHorizontalForce));
                                                if (!entry->mSwingFootControlApplied)
                                                    entry->failPoweredControl(__LINE__);
                                                if (!entry->mGaitActuationDecision->mSwingReactionThroughStanceFoot
                                                    && entry->mPelvisBodyIndex)
                                                {
                                                    // Counter the airborne leg's internal horizontal
                                                    // reaction through the planted leg. Both operations
                                                    // are zero-net force pairs; forward momentum can only
                                                    // come from the stance foot's ground contact.
                                                    JPH::Vec3 horizontalForce = force;
                                                    horizontalForce.SetZ(0.0f);
                                                    if (!horizontalForce.IsNearZero()
                                                        && !entry->mRagdoll->applyBodyForcePair(
                                                            *entry->mPelvisBodyIndex, stanceLeg.mFoot,
                                                            horizontalForce,
                                                            mPassiveRagdollProfile->mActiveController.mGait
                                                                .mMaximumSwingFootHorizontalForce))
                                                        entry->failPoweredControl(__LINE__);
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        if (mPassiveRagdollProfile && entry->mLeftLeg && entry->mRightLeg)
                        {
                            const auto setLegTorqueScale = [&](const RagdollLegChain& leg, float scale) {
                                for (const std::size_t bodyIndex : { leg.mHip, leg.mKnee, leg.mFoot })
                                {
                                    if (bodyIndex >= mPassiveRagdollProfile->mBodies.size()
                                        || !mPassiveRagdollProfile->mBodies[bodyIndex].mConstraint
                                        || !entry->mRagdoll->setMotorMaximumTorque(bodyIndex,
                                            mPassiveRagdollProfile->mBodies[bodyIndex]
                                                    .mConstraint->mMotor.mMaximumTorque
                                                * scale))
                                        return false;
                                }
                                return true;
                            };
                            // A wounded leg is weaker (alpha 0.4 item 7): a limp.
                            // M1 (LEASE_COMPLIANCE): the profile torque here, the
                            // scales as Locomotion leases (commitLeaseComplianceV2).
                            const bool viaLeases = PassiveRagdollEntry::leaseCompliance();
                            std::array<float, 2> legScale{ entry->legWoundScale(true), entry->legWoundScale(false) };
                            const bool restored = setLegTorqueScale(*entry->mLeftLeg, viaLeases ? 1.0f : legScale[0])
                                && setLegTorqueScale(*entry->mRightLeg, viaLeases ? 1.0f : legScale[1]);
                            bool swingScaled = restored;
                            if (swingScaled && entry->mGaitActuationDecision && entry->mGaitOutput
                                && entry->mGaitOutput->hasStance()
                                && entry->mGaitActuationDecision->mSwingLegMotorTorqueScale > 1.0f)
                            {
                                const bool leftStance = entry->mGaitOutput->leftFootIsStance();
                                const RagdollLegChain& swingLeg = leftStance ? *entry->mRightLeg : *entry->mLeftLeg;
                                // (The swing leg's boost replaces its wound scale.)
                                legScale[leftStance ? 1 : 0] = entry->mGaitActuationDecision->mSwingLegMotorTorqueScale;
                                if (!viaLeases)
                                    swingScaled = setLegTorqueScale(
                                        swingLeg, entry->mGaitActuationDecision->mSwingLegMotorTorqueScale);
                            }
                            if (viaLeases && swingScaled)
                                entry->setLocomotionLegScales(legScale[0], legScale[1]);
                            if (!swingScaled)
                            {
                                entry->failPoweredControl(__LINE__);
                                continue;
                            }
                        }
                        // The route a fallen or standing actor takes. Arms
                        // are placed last so they win over the authored pose
                        // without disturbing how it was assembled.
                        // RXN-02 M1: each pose writer records its layer; the
                        // leases compose them (composeEnd, before the drive).
                        using Owner = RagdollCore::BodyOwner;
                        const auto gated = [&](std::optional<Owner> owner, auto&& write) {
                            entry->gateBegin(drivePose);
                            write();
                            entry->gateEnd(drivePose, owner);
                        };
                        entry->composeBegin(drivePose);
                        gated(Owner::Protect, [&] {
                            entry->applySettlingSupportPose(drivePose);
                            entry->applyGetUpLunge(drivePose);
                        });
                        gated(Owner::Locomotion, [&] {
                            entry->applyJumpTuck(drivePose, mPhysicsDt);
                            entry->applyLandingReach(drivePose);
                        });
                        gated(std::nullopt, [&] { entry->applyBracingPgc(drivePose, basePose, mPhysicsDt); });
                        gated(Owner::Reaction, [&] {
                            entry->applyImpactCounterbalance(drivePose);
                        });
                        // The chest's turn toward the weapon goes into the
                        // spine targets before the arms are solved, so the arm
                        // reaches from the turned shoulders (alpha 0.4 item 1).
                        gated(std::nullopt, [&] { entry->applySwingBodyPose(drivePose); });
                        gated(std::nullopt, [&] { entry->applyArmTargets(drivePose, entry->armSolveBase(basePose)); });
                        gated(std::nullopt, [&] { entry->applyWeaponFixtureYaw(drivePose); });
                        // Kinematic-root idle is intentionally owned by the
                        // ordinary standing path rather than the animation-
                        // assisted branch above. Apply the same Stage C leg
                        // adapter after that path has composed its final pose,
                        // using the authored target as the deviation baseline.
                        const auto& balanceProfile = entry->mRuntimeProfile
                            ? entry->mRuntimeProfile->mActiveController
                            : mPassiveRagdollProfile->mActiveController;
                        if (balanceProfile.mLocomotionRoute == RagdollCore::LocomotionRoute::KinematicRoot)
                            gated(Owner::Balance, [&] {
                                entry->applyBalanceLegCorrection(drivePose, entry->mMotorTargetPose, balanceProfile);
                            });
                        std::vector<std::size_t> stunMaskedBodies;
                        // Protective falling is an arm-only exception. The
                        // controller output disables the ordinary pose drive;
                        // do not re-enable the torso and legs merely because
                        // a brace target exists. Mask every body except the
                        // shoulder/elbow/hand chains that own a live brace.
                        std::vector<std::size_t> protectiveMaskedBodies;
                        if (protectiveArmDrive && entry->mRagdoll)
                        {
                            std::array<bool, 3> leftArmBodies{ false, false, false };
                            std::array<bool, 3> rightArmBodies{ false, false, false };
                            if (entry->mLeftArmTarget && entry->mLeftArmTarget->mBrace && entry->mLeftArm)
                                leftArmBodies = { true, true, true };
                            if (entry->mRightArmTarget && entry->mRightArmTarget->mBrace && entry->mRightArm)
                                rightArmBodies = { true, true, true };
                            protectiveMaskedBodies.reserve(entry->mRagdoll->getBodyCount());
                            for (std::size_t body = 0; body < entry->mRagdoll->getBodyCount(); ++body)
                            {
                                // The pelvis root and released equipment have no
                                // joint motor; masking them rejects the whole drive.
                                if ((entry->mPelvisBodyIndex && body == *entry->mPelvisBodyIndex)
                                    || entry->isDetachedWeaponBody(body))
                                    continue;
                                const bool armBody = (entry->mLeftArm &&
                                        (body == entry->mLeftArm->mShoulder || body == entry->mLeftArm->mElbow
                                            || body == entry->mLeftArm->mHand) && leftArmBodies[0])
                                    || (entry->mRightArm &&
                                        (body == entry->mRightArm->mShoulder || body == entry->mRightArm->mElbow
                                            || body == entry->mRightArm->mHand) && rightArmBodies[0]);
                                if (!armBody)
                                    protectiveMaskedBodies.push_back(body);
                            }
                        }
                        const auto& motorMask = protectiveArmDrive ? protectiveMaskedBodies : stunMaskedBodies;
                        entry->gateBegin(drivePose);
                        entry->applyReflexPoseV2(drivePose);
                        entry->gateEnd(drivePose, RagdollCore::BodyOwner::Reaction);
                        entry->composeEnd(drivePose);
                        entry->mJointMotorsApplied = entry->driveBalanceObservedPose(drivePose, "standing-drive", motorMask);
                        if (entry->mJointMotorsApplied && entry->mGaitActuationDecision
                            && entry->mGaitActuationDecision->mUseJointSupportControl)
                        {
                            bool applied = true;
                            for (std::size_t i = 1; i < entry->mRagdoll->getBodyCount(); ++i)
                            {
                                // Released equipment retains its physical body,
                                // but its disconnected joint is no longer a
                                // controlled actor motor.
                                if (entry->isDetachedWeaponBody(i))
                                    continue;
                                // Stagger deliberately releases these joints.
                                // A load-drive request against their disabled
                                // motors fails and used to tear down the whole rig.
                                if (std::find(motorMask.begin(), motorMask.end(), i) != motorMask.end())
                                    continue;
                                const auto torque = entry->mJointSupportControllerOutput
                                    ? entry->mJointSupportControllerOutput->mJointTorques[i] : JPH::Vec3::sZero();
                                if (!entry->mRagdoll->driveMotorWithLoadTorque(i, torque, mPhysicsDt))
                                {
                                    applied = false;
                                    break;
                                }
                            }
                            entry->mJointSupportControlApplied = applied;
                            entry->mJointSupportContact = applied && entry->mJointSupportControllerOutput
                                && entry->mJointSupportControllerOutput->mSupported;
                            if (!applied)
                                entry->failPoweredControl(__LINE__);
                        }
                        if (entry->mJointMotorsApplied)
                            entry->applyWeaponWeightCompensation(mPhysicsDt);
                        entry->stepGuardStiffness(mPhysicsDt);
                        entry->applyPlayerWeaponDrive(mPhysicsDt);
                        if (!entry->mJointMotorsApplied)
                            entry->failPoweredControl(__LINE__);
                        else if (mPassiveRagdollProfile && entry->mGaitOutput && entry->mGaitOutput->hasStance()
                            && entry->mLeftLeg && entry->mRightLeg && entry->mGaitActuationDecision
                            && !entry->mGaitActuationDecision->mUseJointSupportControl)
                        {
                            const RagdollLegChain& swingLeg
                                = entry->mGaitOutput->leftFootIsStance() ? *entry->mRightLeg : *entry->mLeftLeg;
                            if (!entry->mRagdoll->limitMotorTargetStep(swingLeg.mHip,
                                    mPassiveRagdollProfile->mActiveController.mGait.mMaximumSwingHipTargetSpeed
                                        * mPhysicsDt))
                                entry->failPoweredControl(__LINE__);
                            else
                            {
                                RagdollCore::MotorTargetTelemetry hip;
                                RagdollCore::MotorTargetTelemetry knee;
                                RagdollCore::MotorTargetTelemetry ankle;
                                if (entry->mRagdoll->captureMotorTargetTelemetry(swingLeg.mHip, hip))
                                    entry->mSwingHipMotorTarget = hip;
                                if (entry->mRagdoll->captureMotorTargetTelemetry(swingLeg.mKnee, knee))
                                    entry->mSwingKneeMotorTarget = knee;
                                if (entry->mRagdoll->captureMotorTargetTelemetry(swingLeg.mFoot, ankle))
                                    entry->mSwingAnkleMotorTarget = ankle;
                                if (entry->mGaitActuationDecision)
                                {
                                    const auto& decision = *entry->mGaitActuationDecision;
                                    const auto& stanceLeg = entry->mGaitOutput->leftFootIsStance()
                                        ? *entry->mLeftLeg : *entry->mRightLeg;
                                    if ((decision.mReleaseStanceHipAxis
                                            && !entry->mRagdoll->driveMotorWithFreeRotationAxis(
                                                stanceLeg.mHip, decision.mFreeHipWorldAxis, mPhysicsDt))
                                        || (decision.mReleaseSwingHipAxis
                                            && !entry->mRagdoll->driveMotorWithFreeRotationAxis(
                                                swingLeg.mHip, decision.mFreeHipWorldAxis, mPhysicsDt)))
                                        entry->failPoweredControl(__LINE__);
                                    else
                                    {
                                        entry->mStanceHipAxisReleased = decision.mReleaseStanceHipAxis;
                                        entry->mSwingHipAxisReleased = decision.mReleaseSwingHipAxis;
                                    }
                                    for (const std::size_t foot : RagdollCore::selectGaitWorldFootMotors(decision,
                                             entry->mSupportEstimate.value_or(RagdollCore::SupportEstimate{}),
                                             { entry->mLeftLeg->mFoot, entry->mRightLeg->mFoot }))
                                    {
                                        const auto target = entry->mRagdoll->getWorldBodyRotation(basePose, foot);
                                        if (!target || !entry->mRagdoll->driveMotorToWorldOrientation(
                                                foot, *target, mPhysicsDt))
                                            entry->failPoweredControl(__LINE__);
                                        else if (foot == swingLeg.mFoot)
                                        {
                                            RagdollCore::MotorTargetTelemetry worldAnkle;
                                            if (entry->mRagdoll->captureMotorTargetTelemetry(foot, worldAnkle))
                                                entry->mSwingAnkleMotorTarget = worldAnkle;
                                            else
                                                entry->failPoweredControl(__LINE__);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    else
                        entry->mRagdoll->disableMotors();

                    if (RagdollCore::tuningText("OPENMW_RAGDOLL_RECOVERY_TARGET_TRACE") && drive
                        && entry->mControllerOutput && entry->mLeftLeg && entry->mRightLeg
                        && (entry->mControllerOutput->mMode == RagdollCore::ActiveControllerMode::GettingUp
                            || entry->mControllerOutput->mMode == RagdollCore::ActiveControllerMode::BlendBack)
                        && mJoltStepSerial % 6 == 0)
                    {
                        static unsigned samples = 0;
                        for (const auto& leg : { *entry->mLeftLeg, *entry->mRightLeg })
                            for (const auto body : { leg.mHip, leg.mKnee, leg.mFoot })
                            {
                                if (samples >= 900) break;
                                RagdollCore::MotorTargetTelemetry target;
                                if (!entry->mRagdoll->captureMotorTargetTelemetry(body, target)) continue;
                                float nativeClamp = -1;
                                if (!target.mWorldOrientation && target.mRequestedOrientation)
                                    if (const auto geometry = entry->mRagdoll->getSwingTwistJointGeometry(body))
                                    {
                                        const auto requested = (geometry->mParentFrame * *target.mRequestedOrientation
                                            * geometry->mChildFrame.Conjugated()).Normalized();
                                        if (const auto limited = RagdollCore::clampSwingTwistJointOrientation(
                                                *geometry, JPH::Quat::sIdentity(), requested))
                                        {
                                            const auto delta = (requested * limited->Conjugated()).Normalized();
                                            nativeClamp = 2 * std::atan2(delta.GetXYZ().Length(), std::abs(delta.GetW()));
                                        }
                                    }
                                ++samples;
                                Log(Debug::Info) << "RAGDOLL_RECOVERY_TARGET step=" << mJoltStepSerial
                                    << " body=" << body << " mode=" << controllerModeName(entry->mControllerOutput->mMode)
                                    << " mode_time=" << entry->mControllerOutput->mTimeInMode
                                    << " world=" << target.mWorldOrientation << " error=" << target.mAngularError
                                    << " native_limit_clamp=" << nativeClamp
                                    << " clamp=" << (target.mConstraintType == RagdollCore::ConstraintType::SwingTwist
                                        ? target.mTargetClampDelta : -1.0f)
                                    << " hinge_current=" << target.mCurrentHingeAngle.value_or(-99)
                                    << " hinge_target=" << target.mTargetHingeAngle.value_or(-99)
                                    << " frequency=" << target.mSpring.mFrequencyOrStiffness
                                    << " pelvis_up=" << (entry->mControllerInput ? entry->mControllerInput->mPelvisUpDot : -2)
                                    << " support_distance=" << (entry->mSupportEstimate ? entry->mSupportEstimate->mCenterOfMassDistance : -1);
                            }
                    }

                    // Joint support and external upright assistance are
                    // mutually exclusive by contract; the live suite asserts
                    // it ("joint support was combined with external upright
                    // assistance"). A hip strategy therefore cannot be added
                    // here - it has to become joint torque inside the support
                    // solution. See the offline probe and its measurement in
                    // docs/ragdoll/history/hip-strategy-2026-09-07.txt.
                    // The kinematic-root route routes joint support nowhere
                    // (its load blocks belong to other routes); while a
                    // recovery has released the root, the upright controller
                    // is the only pelvis drive the body gets, so it applies
                    // even when the gait decision asks for joint support.
                    const bool uprightAllowedByContract = !entry->mGaitActuationDecision
                        || !entry->mGaitActuationDecision->mUseJointSupportControl
                        || (mPassiveRagdollProfile
                            && mPassiveRagdollProfile->mActiveController.mLocomotionRoute
                                == RagdollCore::LocomotionRoute::KinematicRoot
                            && entry->mRagdoll && !entry->mRagdoll->isRootKinematic());
                    if (drive && entry->mUprightControllerOutput && entry->mPelvisBodyIndex && entry->mTorsoBodyIndex
                        && mPassiveRagdollProfile && entry->mGaitActuationDecision
                        && uprightAllowedByContract)
                    {
                        if (!entry->mGaitActuationDecision || !entry->mLeftLeg || !entry->mRightLeg)
                        {
                            entry->failPoweredControl(__LINE__);
                            continue;
                        }
                        const auto reactions = RagdollCore::selectGaitReactionBodies(
                            *entry->mGaitActuationDecision,
                            entry->mSupportEstimate.value_or(RagdollCore::SupportEstimate{}),
                            { entry->mLeftLeg->mFoot, entry->mRightLeg->mFoot },
                            entry->mLeftLeg->mHip, entry->mRightLeg->mHip,
                            entry->mControllerOutput
                                && entry->mControllerOutput->mMode == RagdollCore::ActiveControllerMode::BlendBack);
                        entry->mUprightActuationStage = entry->mUprightPolicyStage;
                        entry->mAppliedUprightControllerOutput = entry->mUprightControllerOutput;
                        if (reactions.mHorizontal.empty())
                        {
                            entry->mAppliedUprightControllerOutput->mPelvisForce.SetX(0.0f);
                            entry->mAppliedUprightControllerOutput->mPelvisForce.SetY(0.0f);
                        }
                        if (reactions.mVertical.empty())
                            entry->mAppliedUprightControllerOutput->mPelvisForce.SetZ(0.0f);
                        entry->mUprightControlApplied = entry->mRagdoll->applyUprightControl(
                            *entry->mUprightControllerOutput, *entry->mPelvisBodyIndex, *entry->mTorsoBodyIndex,
                            reactions.mVertical, reactions.mHorizontal,
                            mPassiveRagdollProfile->mActiveController, reactions.mAngular,
                            entry->mGaitActuationDecision->mLocomoting, reactions.mSecondaryAngular);
                        if (!entry->mUprightControlApplied)
                            entry->failPoweredControl(__LINE__);
#ifdef OPENMW_RAGDOLL_DEV_API
                        if (RagdollCore::tuningText("OPENMW_RAGDOLL_RECOVERY_TRACE") && entry->mControllerOutput
                            && (entry->mControllerOutput->mMode == RagdollCore::ActiveControllerMode::GettingUp
                                || entry->mControllerOutput->mMode == RagdollCore::ActiveControllerMode::BlendBack)
                            && mJoltStepSerial % 6 == 0)
                        {
                            static unsigned applySamples = 0;
                            if (applySamples < 400)
                            {
                                ++applySamples;
                                Log(Debug::Info) << std::dec << std::setprecision(4)
                                    << "RAGDOLL_RECOVERY_APPLY step=" << mJoltStepSerial
                                    << " player=" << (entry->getPtr() == MWMechanics::getPlayer() ? 1 : 0)
                                    << " mode=" << static_cast<int>(entry->mControllerOutput->mMode)
                                    << " applied=" << entry->mUprightControlApplied
                                    << " pforce=" << entry->mUprightControllerOutput->mPelvisForce.Length()
                                    << " ptorque=" << entry->mUprightControllerOutput->mPelvisTorque.Length()
                                    << " vertical_reactions=" << reactions.mVertical.size()
                                    << " horizontal_reactions=" << reactions.mHorizontal.size();
                            }
                        }
#endif
                    }
                }
            }
#ifdef OPENMW_RAGDOLL_DEV_API
            traceStep("applied");
#endif
            for (auto& [_, entry] : mPassiveRagdolls)
            {
                entry->prepareBalanceStep(mPhysicsSystem);
                entry->limitReactionMotors();
                entry->commitLeaseComplianceV2();
            }
            const auto updateError
                = mPhysicsSystem.Update(mPhysicsDt, cCollisionSteps, mMemoryAllocator.get(), mPhysicsJobSystem.get());
            for (auto& [_, entry] : mPassiveRagdolls)
                entry->restoreReactionMotors();
            if (mPassiveRagdollProfile)
                for (auto& [_, entry] : mPassiveRagdolls)
                    entry->captureBalanceStep(mPhysicsSystem,
                        entry->mRuntimeProfile ? entry->mRuntimeProfile->mActiveController
                                               : mPassiveRagdollProfile->mActiveController, updateError);
            for (auto& [_, entry] : mPassiveRagdolls)
            {
                // NET-01: the puppet's weapon strikes in the remote game; here
                // it only blocks and clashes.
                const bool netPuppet = isNetPuppetEntry(*entry);
                if (!netPuppet)
                    strikeCapsuleActors(*entry);
                const auto strikes = entry->consumeWeaponContacts(mPhysicsSystem);
                // A missing manifold is not separation. Rearm only after the
                // weapon/hand has cleared every body of this victim by the
                // material's margin. Shape snapshots keep queries outside locks.
                for (const auto& pair : entry->mContactDamage.activePairs())
                {
                    const auto target = std::find_if(mPassiveRagdolls.begin(), mPassiveRagdolls.end(),
                        [&](const auto& candidate) { return candidate.second->mSerial == std::get<2>(pair); });
                    bool near = false;
                    if (target != mPassiveRagdolls.end() && target->second->mSerial == std::get<3>(pair))
                    {
                        JPH::TransformedShape weaponShape;
                        {
                            JPH::BodyLockRead lock(mPhysicsSystem.GetBodyLockInterface(), JPH::BodyID(std::get<1>(pair)));
                            if (lock.Succeeded()) weaponShape = lock.GetBody().GetTransformedShape();
                        }
                        for (std::size_t body = 0; weaponShape.mShape && body < target->second->mRagdoll->getBodyCount(); ++body)
                        {
                            JPH::TransformedShape targetShape;
                            {
                                JPH::BodyLockRead lock(mPhysicsSystem.GetBodyLockInterface(), target->second->mRagdoll->getBodyId(body));
                                if (lock.Succeeded()) targetShape = lock.GetBody().GetTransformedShape();
                            }
                            if (RagdollCore::contactShapesWithinDistance(weaponShape, targetShape,
                                    entry->mContactDamage.profile().mSeparationDistance))
                            {
                                near = true;
                                break;
                            }
                        }
                    }
                    if (!near) entry->mContactDamage.confirmSeparation(pair);
                }
                for (const auto& strike : strikes)
                {
                    if (netPuppet || !strike.targetActorIdentity())
                        continue;
                    const auto target = std::find_if(mPassiveRagdolls.begin(), mPassiveRagdolls.end(),
                        [&](const auto& candidate) {
                            return candidate.second->mSerial == *strike.targetActorIdentity();
                        });
                    if (target == mPassiveRagdolls.end())
                        continue;
                    // DMG-01 stale-generation rejection: an event resolved
                    // against a body that no longer exists must not apply its
                    // region to the fresh body that inherited the actor.
                    if (!strike.matchesGeneration(target->second->mSerial))
                        continue;
                    MWWorld::Ptr attacker = entry->getPtr();
                    MWWorld::Ptr victim = target->second->getPtr();
                    // Stale generation: a late event from a body that was
                    // replaced must never apply a region or damage to the
                    // fresh body that inherited the actor.
                    if (!strike.matchesGeneration(target->second->mSerial))
                        continue;
                    MWWorld::InventoryStore& inventory = attacker.getClass().getInventoryStore(attacker);
                    const auto equipped = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                    // An empty record is a fist strike: there is no weapon to
                    // match, the contact itself is the blow.
                    const bool unarmedStrike = strike.weaponRecordId().empty();
                    if (!unarmedStrike
                        && (equipped == inventory.end()
                            || equipped->getCellRef().getRefId().serializeText() != strike.weaponRecordId()))
                        continue;

                    if (!entry->permitsNativeAttackContact(strike))
                        continue;

                    // Blade on blade, blade on shield (user 2026-09-28: "hitting
                    // a sword, i.e. parrying, makes both swords bounce back; also
                    // if it hits a shield, it should bounce"). A contact with the
                    // defender's weapon body - or, carrying a shield, its shield
                    // arm - is no blow: no damage (a weapon body has no region
                    // and was priced like one), the blades are knocked apart by
                    // the approach (effective mass x speed x factor, capped), and
                    // the attacker's committed swing ends instead of driving back
                    // through. OPENMW_RAGDOLL_PARRY_BOUNCE = "restitution,cap":
                    // after the hit-stop the attacker's weapon is thrown back at
                    // that share of its approach speed (weapon body mass x speed,
                    // capped), the defender's at 0.8 of it (0 = off). It was
                    // effective mass x speed x 1.6, capped at 600: a short knock
                    // the tip controller pulled straight back ("if weapons clash,
                    // the weapon bounces further", user 2026-09-29).
                    {
                        auto& defender = *target->second;
                        const std::uint32_t struck = strike.targetBodyIndex();
                        const bool bladeOnBlade = !unarmedStrike && defender.mWeapon && defender.mWeapon->mAttached
                            && struck == defender.mWeapon->mWeaponBody;
                        bool onShield = !bladeOnBlade && !unarmedStrike && defender.mShield
                            && struck == defender.mShield->mShieldBody;
                        if (!onShield && !bladeOnBlade && !unarmedStrike && defender.mLeftArm
                            && (struck == defender.mLeftArm->mHand || struck == defender.mLeftArm->mElbow)
                            && victim.getClass().hasInventoryStore(victim))
                        {
                            MWWorld::InventoryStore& defenderInventory = victim.getClass().getInventoryStore(victim);
                            const auto left = defenderInventory.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
                            onShield = left != defenderInventory.end() && left->getType() == ESM::Armor::sRecordId;
                        }
                        static const auto parryBounce_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
                            // 1.3, 3000 since 2026-09-30 (user: "the parries should make
                            // the weapons bounce much more"); was 0.9, 1400.
                            std::array<float, 2> value{ 1.3f, 3000.0f };
                            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_PARRY_BOUNCE"))
                                std::sscanf(text, "%f,%f", &value[0], &value[1]);
                            return value;
                        });
                        static const auto& parryBounce = parryBounce_tuned.get();
                        if ((bladeOnBlade || onShield) && parryBounce[0] > 0.0f)
                        {
                            // CLASH-V2 (user 10-09: "it is very hard to hit when the
                            // weapons clash, making fights too long"; chosen: "the
                            // stronger blade decides, a clash makes an opening"):
                            // a clash used to arm the attacker's BLOW cooldown on the
                            // defender, so a cut that broke the guard, slid off it or
                            // pushed through could not wound for 0.35 s - after a
                            // won clash the next hit came a median 4.6 s later (play
                            // 10-09). Clashes now have their own gate.
                            // OPENMW_RAGDOLL_CLASH_V2=1 (0 = before).
                            auto& clashGate = clashV2Enabled() ? entry->mClashCooldown : entry->mStrikeCooldown;
                            if (!clashGate.allows(*strike.targetActorIdentity(), strike.physicsStep()))
                                continue;
                            clashGate.commit(*strike.targetActorIdentity(), strike.physicsStep());
                            JPH::Vec3 normal = strike.normal();
                            if (normal.LengthSq() > 1e-6f)
                                normal /= normal.Length();
                            const float approach = std::max(0.0f, strike.relativeNormalSpeed());
                            const float weaponMass = entry->mWeapon ? entry->mWeapon->mPhysicalMass : 1.0f;
                            const float impulse = std::clamp(weaponMass * approach * parryBounce[0], 0.0f, parryBounce[1]);
                            const float freeze = approach >= hitStopSetting()[1] ? hitStopSetting()[0] : 0.0f;
                            // Parry roles (alpha 0.4 item 5, user 2026-09-30: "the
                            // slower weapon at the contact is the parry and bounces
                            // little, the attacker's more; holding block shrinks it
                            // further"). Each blade's speed at the contact point
                            // decides: the faster is the attack (bounced at the
                            // restitution), the slower the parry (bounced at
                            // `parry` of it, x `block` while its owner guards). One
                            // clash is also reported by both bodies in the same
                            // step: it is handled once. Before, the reporting body
                            // always took 0.9 and the other 0.8 - twice when both
                            // reported. OPENMW_RAGDOLL_PARRY_ROLES="parry,block"
                            // (0 = the old split).
                            static const auto roles_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
                                std::array<float, 2> value{ 0.3f, 0.5f };
                                if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_PARRY_ROLES"))
                                    std::sscanf(text, "%f,%f", &value[0], &value[1]);
                                return value;
                            });
                            static const auto& roles = roles_tuned.get();
                            JPH::BodyInterface& parryBodies = mPhysicsSystem.GetBodyInterface();
                            const auto pointSpeed = [&](PassiveRagdollEntry& side) {
                                if (!side.mWeapon || !side.mWeapon->mAttached)
                                    return 0.0f;
                                return parryBodies.GetPointVelocity(
                                    side.mRagdoll->getBodyId(side.mWeapon->mWeaponBody), strike.point()).Length();
                            };
                            const float mySpeed = pointSpeed(*entry);
                            const float theirSpeed = bladeOnBlade ? pointSpeed(defender) : 0.0f;
                            if (bladeOnBlade && roles[0] > 0.0f && defender.mLastParryStep == strike.physicsStep()
                                && defender.mLastParryWith == entry->mSerial)
                                continue; // the other side already handled this clash
                            entry->mLastParryStep = strike.physicsStep();
                            entry->mLastParryWith = defender.mSerial;
                            float myImpulse = impulse, theirImpulse = impulse * 0.8f;
                            float myHardness = std::min(1.0f, parryBounce[0]), theirHardness = myHardness * 0.6f;
                            bool iAttack = true;
                            if (bladeOnBlade && roles[0] > 0.0f)
                            {
                                iAttack = mySpeed >= theirSpeed;
                                const float theirMass = defender.mWeapon ? defender.mWeapon->mPhysicalMass : 1.0f;
                                // The parry is struck by the attack's momentum, not its
                                // own (user 2026-09-30: "a strong heavy wind up hammer
                                // attack should be able to make a sword fly away"):
                                // a hammer's weight on a light blade throws it far.
                                const float attackMass = iAttack ? weaponMass : theirMass;
                                const auto kick = [&](float mass, bool attacking, bool guarding) {
                                    float share = parryBounce[0] * (attacking ? 1.0f : roles[0]);
                                    if (!attacking && guarding)
                                        share *= roles[1];
                                    const float momentumMass = attacking ? mass : std::max(mass, attackMass);
                                    return std::clamp(momentumMass * approach * share, 0.0f, parryBounce[1]);
                                };
                                myImpulse = kick(weaponMass, iAttack, entry->mGuardActive);
                                theirImpulse = kick(theirMass, !iAttack, defender.mGuardActive);
                                const float hard = std::min(1.0f, parryBounce[0]);
                                myHardness = hard * (iAttack ? 1.0f : roles[0]);
                                theirHardness = hard * (iAttack ? roles[0] : 1.0f);
                            }
                            spawnSparks(strike.point(), strike.normal(), approach);
                            // Weight in the clash (C1, build 59 bookmarks 10/13: a chop
                            // with a giant claymore "should be slowed down by the staff
                            // but it should at least do some damage"): the attack's
                            // weight (x1.25 two-handed) against the parry's hold (its
                            // weight, x1.4 two-handed, weaker the further from its
                            // hands it is struck). At `threshold` times the hold or
                            // more the attack pushes through: its own bounce and recoil
                            // shrink by the ratio and its stroke goes on (slowed by the
                            // contact itself), the parry is knocked up to `push` times
                            // harder, and its bearer staggers by `stagger` per unit of
                            // kick. OPENMW_RAGDOLL_CLASH_WEIGHT = "threshold, push,
                            // stagger" (0 = off).
                            float pushThrough = 1.0f;
                            bool clashBrokeGuard = false; // the clash table broke (or took) the parry's guard
                            std::optional<RagdollCore::ClashOutcome> clashOutcome;
                            {
                                static const auto clashWeight_tuned = RagdollCore::tuned<std::array<float, 3>>([] {
                                    std::array<float, 3> value{ 0.0f, 2.5f, 3.0f };
                                    if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CLASH_WEIGHT"))
                                        std::sscanf(text, "%f,%f,%f", &value[0], &value[1], &value[2]);
                                    return value;
                                });
                                static const auto& clashWeight = clashWeight_tuned.get();
                                if (bladeOnBlade && clashWeight[0] > 0.0f)
                                {
                                    PassiveRagdollEntry& attackSide = iAttack ? *entry : defender;
                                    PassiveRagdollEntry& parrySide = iAttack ? defender : *entry;
                                    const auto weight = [](const PassiveRagdollEntry& side) {
                                        return side.mWeapon
                                            ? side.mWeapon->mPhysicalMass * (side.mWeapon->mTwoHanded ? 1.25f : 1.0f)
                                            : 1.0f;
                                    };
                                    float hold = parrySide.mWeapon
                                        ? parrySide.mWeapon->mPhysicalMass * (parrySide.mWeapon->mTwoHanded ? 1.4f : 1.0f)
                                        : 1.0f;
                                    if (parrySide.mWeapon && parrySide.mWeapon->mAttached
                                        && parrySide.mWeapon->mHandBody < parrySide.mRagdoll->getBodyCount())
                                        if (const auto hand = parrySide.mRagdoll->getWorldBodyPosition(
                                                parrySide.mCurrentPhysicsPose, parrySide.mWeapon->mHandBody))
                                        {
                                            const float reach = std::max(10.0f, parrySide.mWeapon->tipDistanceFromHand());
                                            const float lever = std::clamp(
                                                static_cast<float>((strike.point() - *hand).Length()) / reach, 0.0f, 1.0f);
                                            hold *= 1.0f - 0.5f * lever;
                                        }
                                    const float ratio = weight(attackSide) / std::max(0.05f, hold);
                                    if (std::isfinite(ratio) && ratio >= clashWeight[0])
                                    {
                                        pushThrough = ratio;
                                        float& attackKick = iAttack ? myImpulse : theirImpulse;
                                        float& parryKick = iAttack ? theirImpulse : myImpulse;
                                        attackKick /= ratio;
                                        parryKick *= std::min(ratio, std::max(1.0f, clashWeight[1]));
                                        if (clashWeight[2] > 0.0f && !guardBlowsEnabled())
                                            parrySide.registerHitReaction(std::min(parryKick * clashWeight[2], 1800.0f), false);
                                        Log(Debug::Info) << "RAGDOLL_CLASH_WEIGHT attacker="
                                                         << attackSide.getPtr().getCellRef().getRefId()
                                                         << " parry=" << parrySide.getPtr().getCellRef().getRefId()
                                                         << " ratio=" << ratio << " parry_kick=" << parryKick;
                                    }
                                }
                            }
                            // RXN-02 M7, the clash table (a physical clash
                            // table): the attack's momentum against the guard's
                            // firmness (its weight and lever, x a reference speed,
                            // more when it moved into the blow) and the blades'
                            // angle decide deflect, bounce, knockaway or a broken
                            // guard. OPENMW_RAGDOLL_CONTACT_OUTCOME=1.
                            static const auto clashTable_tuned = RagdollCore::tuned<bool>([] {
                                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CONTACT_OUTCOME");
                                return text && std::atoi(text) != 0;
                            });
                            static const auto& clashTable = clashTable_tuned.get();
                            if (clashTable && bladeOnBlade && roles[0] > 0.0f && defender.mWeapon && entry->mWeapon)
                            {
                                PassiveRagdollEntry& attackSide = iAttack ? *entry : defender;
                                PassiveRagdollEntry& parrySide = iAttack ? defender : *entry;
                                const float attackSpeed = iAttack ? mySpeed : theirSpeed;
                                const float parrySpeed = iAttack ? theirSpeed : mySpeed;
                                const float attackMass = attackSide.mWeapon->mPhysicalMass
                                    * (attackSide.mWeapon->mTwoHanded ? 1.25f : 1.0f);
                                float hold = parrySide.mWeapon->mPhysicalMass * (parrySide.mWeapon->mTwoHanded ? 1.4f : 1.0f);
                                if (parrySide.mWeapon->mHandBody < parrySide.mRagdoll->getBodyCount())
                                    if (const auto hand = parrySide.mRagdoll->getWorldBodyPosition(
                                            parrySide.mCurrentPhysicsPose, parrySide.mWeapon->mHandBody))
                                    {
                                        const float reach = std::max(10.0f, parrySide.mWeapon->tipDistanceFromHand());
                                        hold *= 1.0f - 0.5f * std::clamp(
                                            static_cast<float>((strike.point() - *hand).Length()) / reach, 0.0f, 1.0f);
                                    }
                                const bool active = parrySpeed >= 200.0f
                                    || (parrySide.mGuardActive
                                        && strike.physicsStep() - parrySide.mGuardStartPhysicsStep <= 9);
                                const auto bladeOf = [](PassiveRagdollEntry& side) -> std::optional<JPH::Vec3> {
                                    if (side.mWeapon->mHandBody >= side.mRagdoll->getBodyCount())
                                        return std::nullopt;
                                    const auto turn = side.mRagdoll->getWorldBodyRotation(
                                        side.mCurrentPhysicsPose, side.mWeapon->mHandBody);
                                    if (!turn)
                                        return std::nullopt;
                                    return (*turn * side.mWeapon->mBladeAxis).NormalizedOr(JPH::Vec3::sAxisZ());
                                };
                                const auto attackBlade = bladeOf(attackSide);
                                const auto parryBlade = bladeOf(parrySide);
                                RagdollCore::ClashInput input;
                                input.mAttackMomentum = attackMass * attackSpeed;
                                input.mGuardFirmness = hold * (350.0f + (active ? parrySpeed : 0.0f));
                                if (clashV2Enabled())
                                {
                                    // The arms behind the blades: the attack by its
                                    // bearer's Strength, the guard by Strength and how
                                    // fresh it is (a tired guard wavers; every block
                                    // costs vanilla block fatigue, below).
                                    input.mAttackMomentum *= clashArmFactor(attackSide.getPtr(), false);
                                    input.mGuardFirmness *= clashArmFactor(parrySide.getPtr(), true);
                                }
                                input.mBladeAlignment
                                    = attackBlade && parryBlade ? std::abs(attackBlade->Dot(*parryBlade)) : 0.0f;
                                input.mActiveParry = active;
                                const auto clash = RagdollCore::classifyClash(input);
                                clashOutcome = clash.mOutcome;
                                float& attackKick = iAttack ? myImpulse : theirImpulse;
                                float& parryKick = iAttack ? theirImpulse : myImpulse;
                                switch (clash.mOutcome)
                                {
                                    case RagdollCore::ClashOutcome::Deflect:
                                        // It slides off along the guard: a small knock, the
                                        // stroke goes on.
                                        attackKick *= 0.3f;
                                        pushThrough = std::max(pushThrough, 2.0f);
                                        break;
                                    case RagdollCore::ClashOutcome::Knockaway:
                                    case RagdollCore::ClashOutcome::Disarm:
                                        if (clash.mAttackerLockout > 0.0f || clash.mAttackerDisarmed)
                                        {
                                            attackKick *= 1.5f;
                                            attackSide.mClashLockout = std::max(attackSide.mClashLockout,
                                                std::max(clash.mAttackerLockout, 0.4f));
                                            // AI-02 item 5: the parry won - a riposte window.
                                            if (!clash.mDefenderDisarmed)
                                                parrySide.mAiSinceParryWon = 0.0f;
                                        }
                                        if (clash.mDefenderDisarmed)
                                        {
                                            clashBrokeGuard = true;
                                            pushThrough = std::max(pushThrough, clash.mRatio);
                                            parryKick *= 1.5f;
                                            parrySide.mGuardActive = false;
                                            parrySide.mClashLockout = std::max(parrySide.mClashLockout, 0.55f);
                                        }
                                        break;
                                    case RagdollCore::ClashOutcome::BrokenGuard:
                                        clashBrokeGuard = true;
                                        pushThrough = std::max(pushThrough, clash.mRatio);
                                        parryKick *= 1.5f;
                                        parrySide.mGuardActive = false;
                                        parrySide.mClashLockout
                                            = std::max(parrySide.mClashLockout, clash.mDefenderLockout);
                                        break;
                                    case RagdollCore::ClashOutcome::Bounce:
                                        // The blow stopped on the guard: a parry won
                                        // too (AI-02 item 5; NPC guards rise 0.12 s
                                        // after the read, so they almost never make
                                        // the active parry a knockaway needs -
                                        // RIPOSTE-01: 0 knockaways in 16 clashes).
                                        if (!parrySide.blowVictimState().mAttacking)
                                            parrySide.mAiSinceParryWon = 0.0f;
                                        break;
                                }
                                Log(Debug::Info) << "RAGDOLL_CLASH attacker=" << attackSide.getPtr().getCellRef().getRefId()
                                                 << " parry=" << parrySide.getPtr().getCellRef().getRefId()
                                                 << " outcome=" << RagdollCore::clashOutcomeName(clash.mOutcome)
                                                 << " ratio=" << clash.mRatio << " angle=" << input.mBladeAlignment
                                                 << " active=" << active << " attack_speed=" << attackSpeed
                                                 << " parry_speed=" << parrySpeed;
                            }
                            if (impulse > 0.0f && entry->mWeapon && entry->mWeapon->mAttached)
                            {
                                entry->beginWeaponImpact(freeze, -normal * myImpulse, myHardness, strike.point());
                                if (bladeOnBlade)
                                    defender.beginWeaponImpact(freeze, normal * theirImpulse, theirHardness, strike.point());
                            }
                            // The parry is knocked along the attack's travel
                            // (PassiveRagdollEntry::beginParryKnock).
                            if (bladeOnBlade && roles[0] > 0.0f && PassiveRagdollEntry::parryKnockSettings()[0] > 0.0f)
                            {
                                PassiveRagdollEntry& attackSide = iAttack ? *entry : defender;
                                PassiveRagdollEntry& parrySide = iAttack ? defender : *entry;
                                JPH::Vec3 travel = attackSide.mWeapon && attackSide.mWeapon->mAttached
                                    ? parryBodies.GetPointVelocity(
                                        attackSide.mRagdoll->getBodyId(attackSide.mWeapon->mWeaponBody), strike.point())
                                    : JPH::Vec3::sZero();
                                if (travel.LengthSq() < 1.0f)
                                    travel = iAttack ? normal : -normal;
                                const float parryKick = iAttack ? theirImpulse : myImpulse;
                                const float units = PassiveRagdollEntry::parryKnockSettings()[0]
                                    * std::clamp(parryKick / std::max(1.0f, PassiveRagdollEntry::parryKnockSettings()[2]),
                                        0.0f, 1.5f)
                                    * (parrySide.mGuardActive ? roles[1] * 1.4f : 1.0f);
                                parrySide.beginParryKnock(travel, units);
                                Log(Debug::Info) << "RAGDOLL_PARRY_KNOCK parry=" << parrySide.getPtr().getCellRef().getRefId()
                                                 << " units=" << units << " kick=" << parryKick
                                                 << " guard=" << parrySide.mGuardActive;
                            }
                            // The attack wheel's recoil (PassiveRagdollEntry::beginHscKnock):
                            // the attacking hand is thrown back between the way it came
                            // and the bounce, its stroke spent; a parrying blade is
                            // knocked along the attack's travel. Sized by the kicks
                            // (play 10-05/06, 1346 clashes: attack kick median 388, p90
                            // 1077; parry 149 / 518). OPENMW_RAGDOLL_HSC_CLASH_KNOCK =
                            // "attack units, parry units, attack reference kick, parry
                            // reference kick" (units at the reference, 0.4-1.6x of it;
                            // 0 = off).
                            {
                                static const auto clashKnock_tuned = RagdollCore::tuned<std::array<float, 4>>([] {
                                    std::array<float, 4> value{ 36.0f, 26.0f, 400.0f, 150.0f };
                                    if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HSC_CLASH_KNOCK"))
                                        std::sscanf(text, "%f,%f,%f,%f", &value[0], &value[1], &value[2], &value[3]);
                                    return value;
                                });
                                static const auto& clashKnock = clashKnock_tuned.get();
                                PassiveRagdollEntry& attackSide = (bladeOnBlade && !iAttack) ? defender : *entry;
                                const float attackKick = (bladeOnBlade && !iAttack) ? theirImpulse : myImpulse;
                                const JPH::Vec3 bounceDir = &attackSide == entry.get() ? -normal : normal;
                                JPH::Vec3 travel = attackSide.mWeapon && attackSide.mWeapon->mAttached
                                    ? parryBodies.GetPointVelocity(
                                        attackSide.mRagdoll->getBodyId(attackSide.mWeapon->mWeaponBody), strike.point())
                                    : JPH::Vec3::sZero();
                                const JPH::Vec3 travelDir = travel.NormalizedOr(-bounceDir);
                                if (clashKnock[0] > 0.0f)
                                {
                                    const float units = clashKnock[0]
                                        * std::clamp(attackKick / std::max(1.0f, clashKnock[2]), 0.4f, 1.6f) / pushThrough;
                                    // Pushing through, the stroke goes on (C1). CLASH-V2: an
                                    // even bounce ends the stroke but keeps the chain - a
                                    // held drag cuts on from the rebound, a short recovery.
                                    const bool chain = clashV2Enabled() && pushThrough <= 1.0f
                                        && clashOutcome != RagdollCore::ClashOutcome::Knockaway
                                        && clashOutcome != RagdollCore::ClashOutcome::Disarm;
                                    attackSide.beginHscKnock((bounceDir - travelDir).NormalizedOr(bounceDir) * units
                                            * (chain ? 0.6f : 1.0f),
                                        pushThrough <= 1.0f, chain);
                                }
                                if (bladeOnBlade && clashKnock[1] > 0.0f)
                                {
                                    PassiveRagdollEntry& parrySide = iAttack ? defender : *entry;
                                    const float parryKick = iAttack ? theirImpulse : myImpulse;
                                    const float units = clashKnock[1]
                                        * std::clamp(parryKick / std::max(1.0f, clashKnock[3]), 0.4f, 1.6f)
                                        * (parrySide.mGuardActive ? roles[1] * 1.4f : 1.0f);
                                    parrySide.beginHscKnock(travelDir * units, false);
                                }
                            }
                            // A hard block staggers the attacker (alpha 0.4 item 8):
                            // its blade stopped on a held guard or a shield, the
                            // shock goes into its balance like a blow of its own
                            // kick x OPENMW_RAGDOLL_BLOCK_STAGGER (0 = off).
                            {
                                static const auto stagger_tuned = RagdollCore::tuned<float>([] {
                                    const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_BLOCK_STAGGER");
                                    return text ? static_cast<float>(std::atof(text)) : 25.0f;
                                });
                                static const auto& stagger = stagger_tuned.get();
                                PassiveRagdollEntry* striker = iAttack ? entry.get() : &defender;
                                const PassiveRagdollEntry* blocker = iAttack ? &defender : entry.get();
                                const bool hardBlock = onShield || (bladeOnBlade && blocker->mGuardActive);
                                if (hardBlock)
                                {
                                    const float kick = iAttack ? myImpulse : theirImpulse;
                                    // Capped: a heavy blade on a shield must not floor
                                    // the attacker outright.
                                    if (stagger > 0.0f && !guardBlowsEnabled())
                                        striker->registerHitReaction(std::min(kick * stagger, 2500.0f), false);
                                    ++striker->mBlockStaggerCount;
                                    // The shield takes it too (user 10-05, build 57: "shields
                                    // need balance (a stagger when hit?)"): the bearer is
                                    // rocked by the blow at `stagger` per unit of kick
                                    // (capped) and pays fatigue `base` + `per` per unit.
                                    // OPENMW_RAGDOLL_SHIELD_COST = "stagger, cap, base, per"
                                    // (0 = off).
                                    static const auto shieldCost_tuned = RagdollCore::tuned<std::array<float, 4>>([] {
                                        std::array<float, 4> value{ 6.0f, 1600.0f, 4.0f, 0.012f };
                                        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SHIELD_COST"))
                                            std::sscanf(text, "%f,%f,%f,%f", &value[0], &value[1], &value[2], &value[3]);
                                        return value;
                                    });
                                    static const auto& shieldCost = shieldCost_tuned.get();
                                    if (onShield)
                                    {
                                        PassiveRagdollEntry* bearer = iAttack ? &defender : entry.get();
                                        if (shieldCost[0] > 0.0f && !guardBlowsEnabled())
                                            bearer->registerHitReaction(std::min(kick * shieldCost[0], shieldCost[1]), false);
                                        const MWWorld::Ptr bearerPtr = bearer->getPtr();
                                        if (!bearerPtr.isEmpty() && bearerPtr.getClass().isActor()
                                            && (shieldCost[2] > 0.0f || shieldCost[3] > 0.0f))
                                        {
                                            auto& bearerStats = bearerPtr.getClass().getCreatureStats(bearerPtr);
                                            MWMechanics::DynamicStat<float> fatigue = bearerStats.getFatigue();
                                            fatigue.setCurrent(std::max(0.0f, fatigue.getCurrent() - (shieldCost[2] + shieldCost[3] * kick)));
                                            bearerStats.setFatigue(fatigue);
                                        }
                                        Log(Debug::Info) << "RAGDOLL_SHIELD_COST bearer=" << bearerPtr.getCellRef().getRefId()
                                                         << " reaction=" << std::min(kick * shieldCost[0], shieldCost[1])
                                                         << " fatigue=" << (shieldCost[2] + shieldCost[3] * kick);
                                    }
                                    Log(Debug::Info) << "RAGDOLL_BLOCK_STAGGER striker=" << striker->getPtr().getCellRef().getRefId()
                                                     << " reaction=" << kick * stagger << " shield=" << onShield;
                                }
                            }
                            if (guardBlowsEnabled())
                            {
                                PassiveRagdollEntry& attackSide = iAttack ? *entry : defender;
                                PassiveRagdollEntry& guardSide = iAttack ? defender : *entry;
                                const float attackSpeed = iAttack ? mySpeed : theirSpeed;
                                const float attackMass = attackSide.mWeapon
                                    ? attackSide.mWeapon->mPhysicalMass * (attackSide.mWeapon->mTwoHanded ? 1.25f : 1.0f)
                                    : 1.0f;
                                const float momentum = attackMass * std::max(attackSpeed, approach);
                                JPH::Vec3 travel = attackSide.mWeapon && attackSide.mWeapon->mAttached
                                    ? parryBodies.GetPointVelocity(
                                        attackSide.mRagdoll->getBodyId(attackSide.mWeapon->mWeaponBody), strike.point())
                                    : JPH::Vec3::sZero();
                                travel.SetZ(travel.GetZ() * 0.3f);
                                if (travel.LengthSq() < 1.0f)
                                    travel = JPH::Vec3(guardSide.mRagdoll->getReferencePosition()
                                        - attackSide.mRagdoll->getReferencePosition());
                                bool blunt = false;
                                {
                                    const MWWorld::Ptr attackPtr = attackSide.getPtr();
                                    if (attackPtr.getClass().hasInventoryStore(attackPtr))
                                    {
                                        auto& store = attackPtr.getClass().getInventoryStore(attackPtr);
                                        const auto held = store.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                                        blunt = held != store.end() && held->getType() == ESM::Weapon::sRecordId
                                            && held->getClass().getEquipmentSkill(*held) == ESM::Skill::BluntWeapon;
                                    }
                                }
                                // A shield holds about a strapped arm's worth (1050
                                // kg u/s: a solid longsword cut) when the bearer is
                                // fresh and of middling strength; tired or weak it
                                // holds less (user 10-07: "stagger logic based on
                                // stamina and health"). 1.2x that beats it.
                                float firmness = 1050.0f;
                                {
                                    const MWWorld::Ptr guardPtr = guardSide.getPtr();
                                    const auto& stats = guardPtr.getClass().getCreatureStats(guardPtr);
                                    const auto& fatigue = stats.getFatigue();
                                    const float fresh = fatigue.getModified() > 0.0f
                                        ? std::clamp(fatigue.getCurrent() / fatigue.getModified(), 0.0f, 1.0f) : 1.0f;
                                    const float strength = std::clamp(
                                        stats.getAttribute(ESM::Attribute::Strength).getModified() / 50.0f, 0.6f, 1.5f);
                                    firmness *= (0.4f + 0.6f * fresh) * strength;
                                }
                                // A blade guard is beaten when the clash table broke it.
                                const bool beaten = onShield && !bladeOnBlade ? momentum > 1.2f * firmness : clashBrokeGuard;
                                applyGuardBlow(guardSide, attackSide, strike.point(), travel, momentum, onShield && !bladeOnBlade,
                                    beaten, blunt, strike.physicsStep());
                                // The striker feels a hard block in its own arms
                                // (CLASH-V2: only when the guard won - not on a cut
                                // that slid off it or pushed through).
                                const bool guardWon = !clashV2Enabled() || !clashOutcome
                                    || *clashOutcome == RagdollCore::ClashOutcome::Bounce
                                    || *clashOutcome == RagdollCore::ClashOutcome::Knockaway;
                                const bool hard = onShield || guardSide.mGuardActive;
                                if (hard && !beaten && guardWon && (!clashV2Enabled() || pushThrough <= 1.0f))
                                    applyGuardBlow(attackSide, guardSide, strike.point(), -travel, momentum * 0.4f, false, false,
                                        false, strike.physicsStep());
                            }
                            Log(Debug::Info) << "RAGDOLL_PARRY_ROLES reporter=" << attacker.getCellRef().getRefId()
                                             << " speed=" << mySpeed << " other_speed=" << theirSpeed
                                             << " reporter_attacks=" << iAttack << " kick=" << myImpulse
                                             << " other_kick=" << theirImpulse << " reporter_guard=" << entry->mGuardActive
                                             << " other_guard=" << defender.mGuardActive << " blade=" << bladeOnBlade;
                            if (clashV2Enabled() && bladeOnBlade
                                && (pushThrough > 1.0f || clashOutcome == RagdollCore::ClashOutcome::Deflect
                                    || clashOutcome == RagdollCore::ClashOutcome::BrokenGuard))
                            {
                                PassiveRagdollEntry& attackSide = iAttack ? *entry : defender;
                                PassiveRagdollEntry& parrySide = iAttack ? defender : *entry;
                                parrySide.mCoverPassedBy = attackSide.mSerial;
                                parrySide.mCoverPassedUntil = strike.physicsStep() + 24; // 0.4 s
                            }
                            if (!clashV2Enabled() || !iAttack || pushThrough <= 1.0f)
                            {
                                entry->mTip.mCommitTimer = 0.0f;
                                if (entry->mTip.mMode == decltype(entry->mTip)::Mode::Strike)
                                    entry->mTip.mMode = decltype(entry->mTip)::Mode::Guard;
                            }
                            // CLASH-V2: a blade that stopped or turned a blow pays
                            // vanilla block fatigue (fFatigueBlockBase + encumbrance
                            // x fFatigueBlockMult + the attacking weapon's weight x
                            // attack strength x fWeaponFatigueBlockMult).
                            if (clashV2Enabled() && bladeOnBlade && clashOutcome
                                && *clashOutcome != RagdollCore::ClashOutcome::BrokenGuard)
                            {
                                PassiveRagdollEntry& attackSide = iAttack ? *entry : defender;
                                PassiveRagdollEntry& parrySide = iAttack ? defender : *entry;
                                const float attackSpeed = iAttack ? mySpeed : theirSpeed;
                                payBlockFatigue(parrySide.getPtr(), attackSide.getPtr(),
                                    std::clamp(attackSpeed / 1000.0f, 0.1f, 1.0f));
                            }
                            static const ESM::RefId clang = ESM::RefId::stringRefId("Heavy Armor Hit");
                            static const ESM::RefId shieldHit = ESM::RefId::stringRefId("Medium Armor Hit");
                            MWBase::Environment::get().getSoundManager()->playSound3D(
                                victim, bladeOnBlade ? clang : shieldHit, 1.0f, bladeOnBlade ? 1.2f : 1.0f);
                            ++entry->mParryBounceCount;
                            Log(Debug::Info) << "RAGDOLL_PARRY kind=" << (bladeOnBlade ? "blade" : "shield")
                                             << " attacker=" << attacker.getCellRef().getRefId()
                                             << " defender=" << victim.getCellRef().getRefId()
                                             << " approach=" << strike.relativeNormalSpeed()
                                             << " mass=" << strike.effectiveMass() << " impulse=" << impulse;
                            continue;
                        }
                    }

                    JPH::Vec3 defenderForward = target->second->mGuardActive
                        ? target->second->mGuardForward : JPH::Vec3::sAxisY();
                    JPH::RVec3 defenderCenter = target->second->mRagdoll->getReferencePosition();
                    if (target->second->mTorsoBodyIndex)
                    {
                        const JPH::BodyID torso
                            = target->second->mRagdoll->getBodyId(*target->second->mTorsoBodyIndex);
                        JPH::BodyInterface& bodies = mPhysicsSystem.GetBodyInterface();
                        if (!target->second->mGuardActive)
                            defenderForward = bodies.GetRotation(torso) * target->second->mTorsoLocalForward;
                        defenderCenter = bodies.GetCenterOfMassPosition(torso);
                    }
                    defenderForward.SetZ(0.0f);
                    if (defenderForward.LengthSq() > 1e-6f)
                        defenderForward /= defenderForward.Length();
                    JPH::Vec3 contactOffset(strike.point() - defenderCenter);
                    contactOffset.SetZ(0.0f);
                    JPH::Vec3 attackerOffset = Misc::Convert::toJolt<JPH::Vec3>(
                        attacker.getRefData().getPosition().asVec3()
                        - victim.getRefData().getPosition().asVec3());
                    attackerOffset.SetZ(0.0f);
                    float guardFacingAlignment = -1.0f;
                    if (attackerOffset.LengthSq() > 1e-6f && defenderForward.LengthSq() > 1e-6f)
                        guardFacingAlignment = attackerOffset.Normalized().Dot(defenderForward);

                    // A raised guard covers only what it is in front of
                    // (GUARD-COVER, build 63 play: 12 of 12 "no hit" bookmarks,
                    // 22 of 151 cuts - the blade got past the guard to the
                    // thigh, calf, neck or chest and the guard's timing window
                    // still called it a block, no damage). On, a contact
                    // counts as guarded only within `radius` units of the
                    // defender's weapon (hand to tip), shield or bare hands;
                    // blade on blade and shield hits are parries above.
                    // OPENMW_RAGDOLL_GUARD_COVER = radius (0 = off).
                    static const auto guardCoverRadius_tuned = RagdollCore::tuned<float>([] {
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_GUARD_COVER");
                        return text ? std::max(0.0f, static_cast<float>(std::atof(text))) : 0.0f;
                    });
                    static const auto& guardCoverRadius = guardCoverRadius_tuned.get();
                    float guardCover = -1.0f;
                    bool guardCovers = target->second->mGuardActive;
                    // CLASH-V2: past the guard it just beat or slid off.
                    if (clashV2Enabled() && target->second->mCoverPassedBy == entry->mSerial
                        && strike.physicsStep() <= target->second->mCoverPassedUntil)
                        guardCovers = false;
                    if (guardCovers && guardCoverRadius > 0.0f)
                    {
                        guardCover = target->second->guardCoverDistance(strike.point());
                        guardCovers = guardCover <= guardCoverRadius;
                    }
                    const RagdollCore::PhysicalDefenseInput defenseInput{
                        strike.valid(), MWMechanics::isFriendlyPhysicalContact(attacker, victim),
                        guardCovers, strike.physicsStep(), target->second->mGuardStartPhysicsStep,
                        strike.damageSpeed(), std::max(strike.damageSpeed(), strike.relativeSpeed()), guardFacingAlignment
                    };
                    // A held guard keeps blocking what it covers (10-08, NPCHIT-02:
                    // the right-click guard held past 0.6 s let every blow off the
                    // blade through as a hit, cover 15-23 units). Needs GUARD_COVER
                    // (the cover is then by distance, not by time).
                    // OPENMW_RAGDOLL_GUARD_HELD=1.
                    static const auto defenseProfile_tuned = RagdollCore::tuned<RagdollCore::PhysicalDefenseProfile>([] {
                        RagdollCore::PhysicalDefenseProfile value;
                        value.mGlancingDamageScale = partialPaySettings()[0];
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_GUARD_HELD");
                        value.mHeldGuardBlocks = guardCoverRadius_tuned.get() > 0.0f && text && std::atoi(text) != 0;
                        return value;
                    });
                    static const auto& defenseProfile = defenseProfile_tuned.get();
                    // A body that is falling, down or getting up does not
                    // strike (user 10-30 bookmark: "when standing up hits
                    // shouldnt connect" - an NPC getting up from a knockdown
                    // cut the player's thigh and calf for 20 and 24 with its
                    // flailing blade: held blades injure from contact alone).
                    // OPENMW_RAGDOLL_DOWN_NO_STRIKE=0 = off.
                    static const auto downNoStrike_tuned = RagdollCore::tuned<bool>([] {
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DOWN_NO_STRIKE");
                        return !(text && std::atoi(text) == 0);
                    });
                    if (downNoStrike_tuned.get() && entry->mControllerOutput)
                    {
                        using Mode = RagdollCore::ActiveControllerMode;
                        const Mode mode = entry->mControllerOutput->mMode;
                        if (mode == Mode::Falling || mode == Mode::Fallen || mode == Mode::GettingUp
                            || mode == Mode::BlendBack)
                        {
                            // One line per attacker and second (play_metrics.py counts them).
                            if (mJoltStepSerial - entry->mDownStrikeLogged >= 60)
                            {
                                entry->mDownStrikeLogged = mJoltStepSerial;
                                Log(Debug::Info) << "RAGDOLL_DOWN_NO_STRIKE attacker=" << attacker.getCellRef().getRefId()
                                                 << " victim=" << victim.getCellRef().getRefId()
                                                 << " mode=" << static_cast<int>(mode);
                            }
                            continue;
                        }
                    }
                    const auto defense = RagdollCore::classifyPhysicalDefense(defenseInput, defenseProfile);
                    entry->mLastPhysicalDefense = defense;
                    entry->mLastWeaponRedirectImpulse = 0.0f;
                    if (!defense.emitsOutcome())
                        continue;
                    Log(Debug::Info) << "RAGDOLL_DEFENSE outcome=" << RagdollCore::physicalDefenseOutcomeName(defense.mOutcome)
                                     << " guard=" << target->second->mGuardActive << " cover=" << guardCover
                                     << " facing=" << guardFacingAlignment
                                     << " damage_speed=" << strike.damageSpeed() << " relative_speed=" << strike.relativeSpeed()
                                     << " tip=" << entry->mTip.mActive << " ai=" << entry->mAiStrikeActive;
                    ++entry->mConsumedWeaponDefenseOutcomeCount;
                    // The struck NPC learns (updateAiGuard): a blow taken, by the
                    // attacker's wheel line.
                    if (defense.mOutcome == RagdollCore::PhysicalDefenseOutcome::Hit && victim != MWMechanics::getPlayer())
                    {
                        target->second->mAiHitsTaken += 1.0f;
                        if (entry->mHscActive)
                        {
                            const int line = static_cast<int>(std::lround(entry->mAttackCutAngle / (osg::PIf * 0.25f))) & 7;
                            target->second->mAiLineHits[static_cast<std::size_t>(line)] += 1.0f;
                        }
                    }
                    // A supported blade may injure from the victim's relative motion.
                    // Unsupported weapons retain their legacy intent gate.
                    if (!RagdollCore::bladeSurfaceProfile(strike.weaponRecordId())
                        && !entry->mAiStrikeActive && !entry->mPlayerStrikeActive && !entry->mSwing.mSwinging
                        && !entry->mLeftJabStriking
                        && !entry->hasPhysicalFollowThrough())
                        continue;

                    // Coarse surface inference from the held mesh. Blades keep
                    // the generic longest-axis frame; the authored shape
                    // profiles add the axe and hammer heads, whose striking
                    // part is a distal block rather than the whole length.
                    float contactLocalY = 0.0f;
                    float contactEdge = 0.0f;
                    float contactFlat = 0.0f;
                    float contactBluntScale = 1.0f;
                    bool contactOnBlade = true;
                    const auto shape = RagdollCore::weaponShapeProfile(strike.weaponRecordId());
                    if (!unarmedStrike && !RagdollCore::bladeSurfaceProfile(strike.weaponRecordId())
                        && entry->mWeapon && entry->mWeapon->mAttached
                        && (entry->mWeapon->mUsesBladeSurface || shape)
                        && entry->mWeapon->mWeaponBody < entry->mRagdoll->getBodyCount())
                    {
                        const JPH::Vec3 strikeAxis = shape ? shape->mStrikeAxis : entry->mWeapon->mBladeAxis;
                        const JPH::Vec3 edgeAxis = shape ? shape->mEdgeAxis : entry->mWeapon->mEdgeAxis;
                        const JPH::Vec3 flatAxis
                            = shape ? strikeAxis.Cross(edgeAxis).Normalized() : entry->mWeapon->mFlatAxis;
                        JPH::BodyInterface& contactBodies = mPhysicsSystem.GetBodyInterface();
                        const JPH::BodyID weaponId
                            = entry->mRagdoll->getBodyId(entry->mWeapon->mWeaponBody);
                        const JPH::RVec3 weaponCenter = contactBodies.GetCenterOfMassPosition(weaponId);
                        const JPH::Quat weaponRotation = contactBodies.GetRotation(weaponId);
                        const JPH::Vec3 contactLocal
                            = weaponRotation.Conjugated() * JPH::Vec3(strike.point() - weaponCenter);
                        JPH::Vec3 contactNormal = weaponRotation.Conjugated() * strike.normal();
                        if (contactNormal.LengthSq() > 1e-8f)
                            contactNormal = contactNormal.Normalized();
                        contactLocalY = contactLocal.Dot(strikeAxis);
                        contactEdge = std::abs(contactNormal.Dot(edgeAxis));
                        contactFlat = std::abs(contactNormal.Dot(flatAxis));
                        // Forward of the strike start the contact is blade or
                        // head; behind it (haft, guard, butt) only smacks. A
                        // thrust does not care how the blade is rolled, and a
                        // flat slap still bruises.
                        const float handToContact
                            = contactLocalY + entry->mWeapon->mCenterFromHand.Dot(entry->mWeapon->mBladeAxis);
                        const float strikeStart = shape ? shape->mStrikeStart : 0.0f;
                        contactOnBlade = handToContact > strikeStart * entry->mWeapon->tipDistanceFromHand();
                        if (!contactOnBlade)
                            contactBluntScale = 0.5f;
                        else if (std::abs(contactNormal.Dot(strikeAxis)) > 0.7f)
                            contactBluntScale = 1.0f; // stab or head-first
                        else if (contactEdge >= contactFlat)
                            contactBluntScale = 1.0f; // edge cuts
                        else
                            contactBluntScale = 0.5f; // flat smack
                    }
                    const float effectiveDamageScale = defense.mDamageScale * contactBluntScale;

                    if (defense.mRedirectScale > 0.0f && entry->mWeapon && entry->mWeapon->mAttached)
                    {
                        JPH::Vec3 normal = strike.normal();
                        if (normal.LengthSq() > 1e-6f)
                            normal /= normal.Length();
                        JPH::Vec3 right = defenderForward.Cross(JPH::Vec3::sAxisZ());
                        if (right.LengthSq() <= 1e-6f)
                            right = JPH::Vec3::sAxisX();
                        else
                            right /= right.Length();
                        const float side = contactOffset.Dot(right) < 0.0f ? -1.0f : 1.0f;
                        const JPH::Vec3 outward = side * right;
                        JPH::Vec3 redirectDirection
                            = defense.mOutcome == RagdollCore::PhysicalDefenseOutcome::Parry
                            ? outward * 0.8f - normal * 0.2f
                            : outward * 0.2f - normal * 0.8f;
                        if (redirectDirection.LengthSq() > 1e-6f)
                            redirectDirection /= redirectDirection.Length();
                        const float redirectImpulse = std::clamp(strike.effectiveMass()
                            * strike.relativeNormalSpeed() * defense.mRedirectScale, 0.0f, 40.0f);
                        if (redirectImpulse > 0.0f)
                        {
                            mPhysicsSystem.GetBodyInterface().AddImpulse(
                                entry->mRagdoll->getBodyId(entry->mWeapon->mWeaponBody),
                                redirectDirection * redirectImpulse, strike.point());
                            entry->mLastWeaponRedirectImpulse = redirectImpulse;
                        }
                    }
                    // WPN-07: one blow may keep touching for many steps and
                    // several bodies. Once a target has counted a hit, hold the
                    // RPG transaction (damage, condition, skill, reaction and
                    // hit feedback) for the cooldown window. The physical
                    // defense redirect above is unaffected.
                    if (!entry->mStrikeCooldown.allows(*strike.targetActorIdentity(), strike.physicsStep()))
                    {
                        ++entry->mConsumedWeaponCooldownSuppressedCount;
                        continue;
                    }
                    // Alpha 1.0 skill parity: vanilla's hit roll, once per blow
                    // per target. A failed roll is a glancing blow - the
                    // contact stays physical (bounce, defense), nothing is paid,
                    // and the blow's gate is armed so its other contacts do not
                    // roll again. OPENMW_RAGDOLL_HIT_ROLL=1 (0 = off).
                    // On (user: "softened roll"): duels 75 of 109 rolls bit.
                    static const auto hitRoll_tuned = RagdollCore::tuned<bool>([] {
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HIT_ROLL");
                        return !(text && std::atoi(text) == 0);
                    });
                    static const auto& hitRoll = hitRoll_tuned.get();
                    // A failed roll still lands as a glancing blow paying this
                    // share (build 61 bookmark 11: "enemy hits won't connect" -
                    // the duel daedra's blade reached the player and its 35%
                    // roll failed, so a visible contact paid nothing).
                    // OPENMW_RAGDOLL_GLANCE_SHARE (0 = a failed roll pays nothing).
                    static const auto glanceShare_tuned = RagdollCore::tuned<float>([] {
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_GLANCE_SHARE");
                        return text ? std::clamp(static_cast<float>(std::atof(text)), 0.0f, 1.0f) : 0.0f;
                    });
                    static const auto& glanceShare = glanceShare_tuned.get();
                    float glanceScale = 1.0f;
                    if (hitRoll)
                    {
                        float chance = 100.0f;
                        const bool bites = MWMechanics::rollPhysicalHit(
                            attacker, victim, unarmedStrike ? MWWorld::Ptr() : *equipped, &chance);
                        static const auto rollTrace_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_DAMAGE_TRACE") != nullptr; });
                        static const auto& rollTrace = rollTrace_tuned.get();
                        if (rollTrace)
                            Log(Debug::Info) << "RAGDOLL_HIT_ROLL attacker=" << attacker.getCellRef().getRefId()
                                             << " victim=" << victim.getCellRef().getRefId() << " chance=" << chance
                                             << " bites=" << bites;
                        if (!bites)
                        {
                            ++entry->mGlancingBlows;
                            if (glanceShare <= 0.0f)
                            {
                                entry->mStrikeCooldown.commit(*strike.targetActorIdentity(), strike.physicsStep());
                                continue;
                            }
                            glanceScale = glanceShare;
                        }
                    }
                    // Hit region from the contact snapshot: the callback
                    // resolved it while it owned stable reads, and the stale
                    // generation was rejected above. Unknown bodies keep the
                    // legacy fallback. The region scales both the damage and
                    // the reaction, and the torso and head cannot reach the
                    // knockdown tier: they flinch and stagger, while the legs
                    // and pelvis are what can fail.
                    RagdollCore::AnatomicalRegion targetRegion = strike.region();
                    if (targetRegion == RagdollCore::AnatomicalRegion::Unknown
                        && strike.targetBodyIndex() != RagdollCore::WeaponContactEvent::sInvalidBodyIndex)
                    {
                        if (const RagdollCore::RagdollProfile* targetProfile = target->second->mRuntimeProfile
                                ? target->second->mRuntimeProfile.get()
                                : mPassiveRagdollProfile.get())
                        {
                            if (strike.targetBodyIndex() < targetProfile->mBodies.size())
                                targetRegion = RagdollCore::regionForBodyName(
                                    targetProfile->mBodies[strike.targetBodyIndex()].mName);
                        }
                    }
                    // DMG-04: measured contacts pay their share of one blow (the
                    // regional split of an impact, or a cut installment); the
                    // legacy fraction stays a plain multiplier on the blow.
                    const MWMechanics::PhysicalBlowShare blowShare = strike.hasMeasuredWork()
                        ? MWMechanics::PhysicalBlowShare{ strike.damageShare(), strike.damageIsCut() }
                        : MWMechanics::PhysicalBlowShare{};
                    const float measuredWorkFraction = strike.hasMeasuredWork() ? strike.damageFraction() : -1.0f;
                    const float regionImpact = regionDamageScale(targetRegion)
                        * (strike.hasMeasuredWork() ? 1.0f : strike.damageFraction());
                    const float regionReaction = regionReactionScale(targetRegion);
                    const bool localizedWound = strike.wound().mKind != RagdollCore::WoundKind::None
                        && targetRegion != RagdollCore::AnatomicalRegion::Unknown;
                    // Every blow on a known body part is a hit receipt: the
                    // reaction layer, limb damage and the local flinch read it.
                    // Only blades with a surface profile have a wound kind (cut,
                    // stab, bruise mark); the others (claymore, glass, daedric...)
                    // used to get no receipt at all - 0 reactions in 70 blows on
                    // hub opponents (play 2026-10-01). OPENMW_RAGDOLL_RECEIPT_ALL=0
                    // = wound-kind blows only.
                    static const auto receiptAll_tuned = RagdollCore::tuned<bool>([] {
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_RECEIPT_ALL");
                        return !(text && std::atoi(text) == 0);
                    });
                    static const auto& receiptAll = receiptAll_tuned.get();
                    const bool receipted = localizedWound
                        || (receiptAll && targetRegion != RagdollCore::AnatomicalRegion::Unknown);
                    static std::uint64_t nextPhysicalHitId = 0;
                    const std::uint64_t physicalHitId = receipted ? ++nextPhysicalHitId : 0;
                    if (receipted)
                    {
                        auto& pending = target->second->mPendingWounds;
                        while (pending.size() >= 32)
                            pending.erase(pending.begin());
                        pending.emplace(physicalHitId, PassiveRagdollEntry::PendingWound{ strike });
                    }
                    // A bruise from an edged weapon is its flat or hilt (half
                    // the blow); from a hammer, mace or staff it is the blow.
                    const bool bluntWeapon = entry->mWeapon
                        && entry->mWeapon->mWoundProfile.mClass == RagdollCore::WoundClass::Blunt;
                    if (localizedWound && !unarmedStrike && !bluntWeapon)
                        contactBluntScale = strike.wound().mKind == RagdollCore::WoundKind::Bruise ? 0.5f : 1.0f;
                    // GRIP-05 murder-stroke: the hilt and pommel are the hammer's
                    // head, the whole blow (its wound profile is blunt already).
                    const bool murderStroke = entry->murderStrokeGrip() && entry->mMurder.has_value();
                    if (murderStroke)
                        contactBluntScale = 1.0f;
                    // RXN-02 M7: what the contact did - glance, bite, stop or
                    // cleave - from its speeds, edge, carried mass and the
                    // armour at the struck slot (no dice). A glance pays a
                    // share; the outcome also sets the blade's response below.
                    // OPENMW_RAGDOLL_CONTACT_OUTCOME=1 (0 = off).
                    static const auto contactOutcomes_tuned = RagdollCore::tuned<bool>([] {
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CONTACT_OUTCOME");
                        return text && std::atoi(text) != 0;
                    });
                    static const auto& contactOutcomes = contactOutcomes_tuned.get();
                    std::optional<RagdollCore::BodyContactResult> bodyOutcome;
                    if (contactOutcomes && defense.mOutcome == RagdollCore::PhysicalDefenseOutcome::Hit)
                    {
                        RagdollCore::BodyContactInput input;
                        const bool bluntSkill = !unarmedStrike
                            && equipped->getClass().getEquipmentSkill(*equipped) == ESM::Skill::BluntWeapon;
                        input.mKind = unarmedStrike ? RagdollCore::BlowKind::Fist
                            : murderStroke ? RagdollCore::BlowKind::Blunt
                            : strike.wound().mKind == RagdollCore::WoundKind::Puncture ? RagdollCore::BlowKind::Point
                            : bluntSkill ? RagdollCore::BlowKind::Blunt
                                         : RagdollCore::BlowKind::Edge;
                        JPH::Vec3 contactNormal = strike.normal();
                        if (contactNormal.LengthSq() > 1e-6f)
                            contactNormal /= contactNormal.Length();
                        const JPH::Vec3 relative = strike.wound().mRelativeVelocity;
                        // The effective speed (a slice's sliding share, a measured
                        // cut's speed) drives the energy; a slice is not shallow.
                        input.mNormalSpeed = std::max(0.0f, std::max(strike.relativeNormalSpeed(), strike.damageSpeed()));
                        input.mTangentSpeed = strike.isSlice()
                            ? 0.0f : (relative - contactNormal * relative.Dot(contactNormal)).Length();
                        // The flat (a bruise from an edged weapon) meets the body square-on.
                        input.mEdgeAlignment = strike.wound().mKind == RagdollCore::WoundKind::Bruise ? 0.2f : 1.0f;
                        input.mEffectiveMass = unarmedStrike
                            ? 1.0f
                            : std::clamp((entry->mWeapon ? entry->mWeapon->mItemWeight : 0.0f) * 0.08f, 0.4f, 4.0f);
                        input.mHardness = armorHardnessAt(victim, targetRegion);
                        // GRIP-05: a half-sword point is aimed at the gaps, so it
                        // meets the armour at `armour share` of its hardness.
                        if (input.mKind == RagdollCore::BlowKind::Point && entry->halfSwordGrip())
                            input.mHardness *= PassiveRagdollEntry::gripHalfSettings()[3];
                        input.mSoftPart = RagdollCore::regionIsArm(targetRegion) || RagdollCore::regionIsLeg(targetRegion)
                            || targetRegion == RagdollCore::AnatomicalRegion::Neck;
                        bodyOutcome = RagdollCore::classifyBodyContact(input);
                        Log(Debug::Info) << "RAGDOLL_CONTACT_OUTCOME attacker=" << attacker.getCellRef().getRefId()
                                         << " victim=" << victim.getCellRef().getRefId()
                                         << " region=" << RagdollCore::regionName(targetRegion)
                                         << " outcome=" << RagdollCore::bodyContactOutcomeName(bodyOutcome->mOutcome)
                                         << " energy=" << bodyOutcome->mEnergy << " cut=" << bodyOutcome->mCutEnergy
                                         << " vn=" << input.mNormalSpeed << " vt=" << input.mTangentSpeed
                                         << " hardness=" << input.mHardness << " hold=" << bodyOutcome->mHoldTime;
                    }
                    // A contact glance (it skids or nicks off) pays the partial
                    // share; a landed blow never less than the floor (PARTIAL_PAY).
                    const float contactScale = !bodyOutcome ? 1.0f
                        : bodyOutcome->mOutcome == RagdollCore::BodyContactOutcome::Glance ? partialPaySettings()[1]
                                                                                            : bodyOutcome->mDamageScale;
                    float woundDamageScale = (localizedWound ? defense.mDamageScale * contactBluntScale
                                                             : effectiveDamageScale) * glanceScale * contactScale
                        * (murderStroke ? PassiveRagdollEntry::gripMurderSettings()[3] : 1.0f);
                    if (defense.mOutcome == RagdollCore::PhysicalDefenseOutcome::Hit && woundDamageScale > 0.0f)
                        woundDamageScale = std::max(woundDamageScale, partialPaySettings()[2]);
                    const auto contactPhase = entry->contactAttackPhase();
                    const bool nativeRelease = contactPhase == MWMechanics::RagdollAttackPhase::Release
                        || contactPhase == MWMechanics::RagdollAttackPhase::FollowThrough;
                    const int woundAttackType = nativeRelease
                        ? (entry->mAnimationAttack.mAttackType == "thrust" ? ESM::Weapon::AT_Thrust
                            : entry->mAnimationAttack.mAttackType == "slash" ? ESM::Weapon::AT_Slash
                                                                                 : ESM::Weapon::AT_Chop)
                        : !localizedWound || unarmedStrike ? -1
                        : strike.wound().mKind == RagdollCore::WoundKind::Puncture ? ESM::Weapon::AT_Thrust
                                                                                                : ESM::Weapon::AT_Slash;
                    const auto damage = unarmedStrike
                        ? MWMechanics::applyPhysicalHandStrike(attacker, victim,
                            Misc::Convert::toOsg(strike.point()), strike.damageSpeed(), woundDamageScale,
                            defense.mOutcome, regionImpact, physicalHitId, blowShare)
                        : MWMechanics::applyPhysicalWeaponStrike(attacker, victim, *equipped,
                            Misc::Convert::toOsg(strike.point()), strike.damageSpeed(), woundDamageScale,
                            defense.mOutcome, regionImpact, physicalHitId, woundAttackType, blowShare,
                            nativeRelease ? std::optional(entry->mAnimationAttack.mStrength) : std::nullopt);
                    if (!damage)
                    {
                        target->second->mPendingWounds.erase(physicalHitId);
#ifdef OPENMW_RAGDOLL_DEV_API
                        if (RagdollCore::tuningText("OPENMW_RAGDOLL_AI_STRIKE_TRACE"))
                        {
                            static unsigned noDamageRows = 0;
                            if (noDamageRows < 200)
                            {
                                ++noDamageRows;
                                Log(Debug::Info) << "RAGDOLL_PHYSICAL_STRIKE attacker="
                                    << attacker.getCellRef().getRefId() << " victim="
                                    << victim.getCellRef().getRefId() << " applied=0";
                            }
                        }
#endif
                        continue;
                    }
                    if (localizedWound)
                        if (const auto paid = target->second->mPendingWounds.find(physicalHitId);
                            paid != target->second->mPendingWounds.end())
                            paid->second.mLocalReaction = damage->mLocalReactionImpulse;
                    // The blow landed and was paid for: arm the gate so its
                    // remaining contact and cutting installments cannot charge
                    // the RPG pipeline again.
                    entry->mStrikeCooldown.commit(*strike.targetActorIdentity(), strike.physicsStep());
#ifdef OPENMW_RAGDOLL_DEV_API
                    if (entry->mContactWindowAuditEnabled && !entry->mContactWindowObservations.empty())
                    {
                        auto& sample = entry->mContactWindowObservations.back();
                        if (sample.mPhysicsStep != mJoltStepSerial || sample.mDispatches.size() == 64)
                            sample.mOverflow = true;
                        else
                        {
                            sample.mDispatches.push_back(strike);
                            sample.mPhysicalHitIds.push_back(physicalHitId);
                        }
                    }
#endif
                    ++entry->mConsumedWeaponDamageCount;
                    // RXN-02 M2: the blow, once. Every reaction reads this record.
                    RagdollCore::Blow blow;
                    {
                        blow.mId = RagdollCore::nextBlowId();
                        blow.mPhysicsStep = strike.physicsStep();
                        blow.mAttacker = entry->mSerial;
                        blow.mVictim = target->second->mSerial;
                        const bool bluntSkill = !unarmedStrike
                            && equipped->getClass().getEquipmentSkill(*equipped) == ESM::Skill::BluntWeapon;
                        blow.mKind = unarmedStrike ? RagdollCore::BlowKind::Fist
                            : strike.wound().mKind == RagdollCore::WoundKind::Puncture ? RagdollCore::BlowKind::Point
                            : bluntSkill ? RagdollCore::BlowKind::Blunt
                                         : RagdollCore::BlowKind::Edge;
                        blow.mRegion = targetRegion;
                        blow.mBodyIndex = strike.targetBodyIndex();
                        blow.mPoint = strike.point();
                        const JPH::Vec3 normal = strike.normal();
                        blow.mDirection = normal.LengthSq() > 1e-6f ? normal.Normalized() : normal;
                        const osg::Vec3f toVictim = victim.getRefData().getPosition().asVec3()
                            - attacker.getRefData().getPosition().asVec3();
                        blow.mAttackerToVictim = JPH::Vec3(toVictim.x(), toVictim.y(), 0.0f);
                        blow.mApproachSpeed = std::max(0.0f, strike.relativeNormalSpeed());
                        blow.mCarriedMass = unarmedStrike
                            ? 1.0f
                            : std::clamp((entry->mWeapon ? entry->mWeapon->mItemWeight : 0.0f) * 0.08f, 0.4f, 4.0f);
                        blow.mDamage = damage->mFinalDamage; // paid; a beaten guard pays a share
                        // The region-scaled reaction impulse the banding reads.
                        blow.mReactionImpulse = damage->mReactionImpulse * regionReaction;
                        blow.mBlocked = damage->mBlocked;
                        blow.mParried = defense.mOutcome == RagdollCore::PhysicalDefenseOutcome::Parry;
                        blow.mLocalizedWound = localizedWound;
                        blow.mReceipted = receipted;
                        blow.mVictimState = target->second->blowVictimState();
                    }
                    // Impact feel (user 2026-09-29: "if the hit connects, if the
                    // opponent is armored, the weapon bounces back, and the enemy
                    // reacts accordingly"). A clean blow holds the weapon still
                    // for the hit-stop; armour, a guard or a blunt head then
                    // throws it back; and the struck body part takes the weapon's
                    // real momentum (carried mass x approach speed), so a head
                    // snaps and a torso folds - the 1 kg weapon body itself
                    // carried almost none of it.
                    {
                        const float approach = std::max(0.0f, strike.relativeNormalSpeed());
                        JPH::Vec3 normal = strike.normal();
                        if (normal.LengthSq() > 1e-6f)
                            normal /= normal.Length();
                        const float armor = armorHardnessAt(victim, targetRegion);
                        float hardness = armor;
                        if (damage->mBlocked || defense.mOutcome == RagdollCore::PhysicalDefenseOutcome::Parry)
                            hardness = std::max(hardness, 0.7f);
                        if (!unarmedStrike && equipped->getClass().getEquipmentSkill(*equipped) == ESM::Skill::BluntWeapon)
                            hardness = std::min(1.0f, std::max(hardness * 1.25f, 0.25f));
                        const auto& bounceSetting = impactBounceSetting();
                        const float weaponMass = entry->mWeapon ? entry->mWeapon->mPhysicalMass : 1.0f;
                        const float bounce = unarmedStrike || !(bounceSetting[0] > 0.0f)
                            ? 0.0f
                            : std::min(bounceSetting[1], weaponMass * approach * hardness * bounceSetting[0]);
                        float freeze = approach >= hitStopSetting()[1] ? hitStopSetting()[0] : 0.0f;
                        float recoil = bounce;
                        // M7: the outcome is the hit-stop - a bite holds the
                        // blade for its bite, a stop freezes and throws it back
                        // (as before), a cleave and a glance go on.
                        if (bodyOutcome)
                            switch (bodyOutcome->mOutcome)
                            {
                                case RagdollCore::BodyContactOutcome::Bite:
                                    freeze = bodyOutcome->mHoldTime;
                                    recoil = 0.0f;
                                    break;
                                case RagdollCore::BodyContactOutcome::Cleave:
                                case RagdollCore::BodyContactOutcome::Glance:
                                    freeze = 0.0f;
                                    recoil = 0.0f;
                                    break;
                                case RagdollCore::BodyContactOutcome::Stop:
                                    break;
                            }
                        if (freeze > 0.0f || recoil > 0.0f)
                            entry->beginWeaponImpact(freeze, -normal * recoil, hardness, strike.point());
                        // Medium or heavy armour is metal enough to spark.
                        if (!unarmedStrike && armor >= 0.5f)
                            spawnSparks(strike.point(), strike.normal(), approach);
                        // The struck body's reactions, through the one entry point.
                        // NET-01: a puppet's body is in the remote game; the blow
                        // goes there (the kick here still reads it as landed).
                        const bool remoteVictim = isNetPuppetEntry(*target->second);
                        if (remoteVictim)
                            queueNetBlow(blow);
                        const float pushed = remoteVictim ? std::max(1.0f, blow.momentum())
                                                          : applyBlowReaction(*target->second, blow);
                        // The player feels a blow land: a camera kick by
                        // its damage (a clean cut on flesh kicked nothing).
                        // V2: by what the blade did - a bite or a cleave by
                        // its damage, a stop by the hardness it met (the
                        // jar of a blade stopped by plate), a glance barely.
                        // (Before build 67 this needed HIT_REACT's head snap
                        // on as well as CAMERA_KICK, and never fired in play.)
                        if (pushed > 0.0f && attacker == MWMechanics::getPlayer())
                        {
                            float kick = 0.0f;
                            if (bodyOutcome)
                                switch (bodyOutcome->mOutcome)
                                {
                                    case RagdollCore::BodyContactOutcome::Bite:
                                    case RagdollCore::BodyContactOutcome::Cleave:
                                        kick = std::clamp(blow.mDamage / 30.0f, 0.15f, 0.7f);
                                        break;
                                    case RagdollCore::BodyContactOutcome::Stop:
                                        kick = std::clamp(0.3f + 0.5f * hardness, 0.3f, 0.8f);
                                        break;
                                    case RagdollCore::BodyContactOutcome::Glance:
                                        kick = 0.1f;
                                        break;
                                }
                            else
                                kick = std::clamp(blow.mDamage / 30.0f, 0.15f, 0.7f);
                            entry->mCameraKick = std::max(entry->mCameraKick, kick);
                        }
                        // Hit sounds by weapon and surface (stage 6, build 61
                        // bookmark 8: "slashes sound exactly the same as hammer
                        // hits" - every blow on flesh played Morrowind's one
                        // "Health Damage"). On armour the plate, mail or leather
                        // rings (a blade higher, a hammer lower); on flesh a blade
                        // cuts (higher, a deep cut also "critical damage"), a
                        // hammer thuds, a point stabs, a fist smacks. The combat
                        // script's own sound is muted for these blows.
                        // OPENMW_RAGDOLL_HIT_SOUNDS=1 (off by default).
                        if (hitSoundsEnabled() && damage && !damage->mBlocked)
                        {
                            const bool blunt = !unarmedStrike
                                && equipped->getClass().getEquipmentSkill(*equipped) == ESM::Skill::BluntWeapon;
                            const bool point = !unarmedStrike && strike.wound().mKind == RagdollCore::WoundKind::Puncture;
                            const float loud = std::clamp(approach / 600.0f, 0.4f, 1.0f);
                            const float shade = blunt ? 0.8f : unarmedStrike ? 1.0f : 1.15f;
                            MWBase::SoundManager& sounds = *MWBase::Environment::get().getSoundManager();
                            static const ESM::RefId heavy = ESM::RefId::stringRefId("Heavy Armor Hit");
                            static const ESM::RefId medium = ESM::RefId::stringRefId("Medium Armor Hit");
                            static const ESM::RefId light = ESM::RefId::stringRefId("Light Armor Hit");
                            static const ESM::RefId flesh = ESM::RefId::stringRefId("Health Damage");
                            static const ESM::RefId thud = ESM::RefId::stringRefId("Hand To Hand Hit");
                            static const ESM::RefId deep = ESM::RefId::stringRefId("critical damage");
                            if (armor >= 0.75f)
                                sounds.playSound3D(victim, heavy, loud, shade);
                            else if (armor >= 0.45f)
                                sounds.playSound3D(victim, medium, loud, shade);
                            else if (armor >= 0.15f)
                                sounds.playSound3D(victim, light, loud, shade);
                            else if (unarmedStrike)
                                sounds.playSound3D(victim, thud, loud, 1.1f);
                            else if (blunt)
                            {
                                sounds.playSound3D(victim, thud, loud, 0.65f);
                                sounds.playSound3D(victim, flesh, 0.5f * loud, 0.75f);
                            }
                            else if (point)
                                sounds.playSound3D(victim, flesh, 0.85f * loud, 1.45f);
                            else
                            {
                                sounds.playSound3D(victim, flesh, loud, 1.2f);
                                if (damage->mFinalDamage >= 12.0f)
                                    sounds.playSound3D(victim, deep, 0.55f * loud, 1.1f);
                            }
                        }
                        // A blow to the head always cries out (user bookmark
                        // 2026-10-31 hub session: "make sure a hurt sound effect
                        // is played on headshots"; the "hit" voice was only
                        // Morrowind's iVoiceHitOdds roll, the same for every
                        // region). The roll in onHit, queued behind this, finds
                        // the voice already playing and stays quiet.
                        // OPENMW_RAGDOLL_HEAD_VOICE = 0 leaves it to the roll.
                        static const auto headVoice_tuned = RagdollCore::tuned<bool>([] {
                            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HEAD_VOICE");
                            return !text || std::atoi(text) != 0;
                        });
                        static const auto& headVoice = headVoice_tuned.get();
                        if (headVoice && targetRegion == RagdollCore::AnatomicalRegion::Head && damage
                            && !damage->mBlocked && damage->mFinalDamage > 0.0f && victim.getClass().isNpc()
                            && !victim.getClass().getCreatureStats(victim).isDead())
                        {
                            const bool said = MWBase::Environment::get().getDialogueManager()->say(
                                victim, ESM::RefId::stringRefId("hit"));
                            Log(Debug::Info) << "RAGDOLL_HEAD_VOICE victim=" << victim.getCellRef().getRefId()
                                             << " damage=" << damage->mFinalDamage << " said=" << said;
                        }
                        static const auto traceImpact_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_DAMAGE_TRACE") != nullptr; });
                        static const auto& traceImpact = traceImpact_tuned.get();
                        if (traceImpact)
                            Log(Debug::Info) << std::dec << std::fixed << std::setprecision(1)
                                             << "RAGDOLL_IMPACT attacker=" << attacker.getCellRef().getRefId()
                                             << " victim=" << victim.getCellRef().getRefId()
                                             << " region=" << RagdollCore::regionName(targetRegion)
                                             << " armor=" << armor << " hardness=" << hardness
                                             << " approach=" << approach << " bounce=" << bounce
                                             << " freeze=" << freeze << " push=" << pushed
                                             << " blocked=" << damage->mBlocked;
                    }
#ifdef OPENMW_RAGDOLL_DEV_API
                    if (RagdollCore::tuningText("OPENMW_RAGDOLL_AI_STRIKE_TRACE"))
                    {
                        static unsigned damageRows = 0;
                        if (damageRows < 200)
                        {
                            ++damageRows;
                            Log(Debug::Info) << std::dec << std::setprecision(4)
                                << "RAGDOLL_PHYSICAL_STRIKE attacker="
                                << attacker.getCellRef().getRefId() << " victim="
                                << victim.getCellRef().getRefId() << " applied=1 final="
                                << damage->mFinalDamage << " blocked=" << damage->mBlocked
                                << " step=" << mJoltStepSerial << " victim_serial=" << target->second->mSerial
                                << " scale=" << defense.mDamageScale
                                << " work=" << measuredWorkFraction
                                << " speed=" << strike.damageSpeed()
                                << " approach=" << strike.relativeNormalSpeed()
                                << " mass=" << strike.effectiveMass()
                                << " blunt=" << contactBluntScale
                                << " blade=" << contactOnBlade
                                << " localy=" << contactLocalY
                                << " edge=" << contactEdge
                                << " flat=" << contactFlat
                                << " region=" << RagdollCore::regionName(targetRegion)
                                << " atk=" << attacker.getClass().getCreatureStats(attacker).getAttackType()
                                << " reaction=" << damage->mReactionImpulse
                                << " band="
                                << target->second->hitBandName()
                                << " ko=" << victim.getClass().getCreatureStats(victim).getKnockedDown();
                        }
                    }
#endif
                    entry->mLastImpactFraction = damage->mImpactFraction;
                    entry->mLastAuthoredDamage = damage->mAuthoredDamage;
                    entry->mLastFinalDamage = damage->mFinalDamage;
                    entry->mLastDamageBlocked = damage->mBlocked;
                }
                if (entry->mWeapon && entry->mWeapon->mAttached)
                {
                    RagdollCore::ConstraintImpulse impulse;
                    // GRIP-05 murder-stroke: the weapon's grip joint is set
                    // aside; the reversed hold carries the load (both hands on
                    // the blade: it does not break).
                    if (entry->mMurder && entry->mReverseGrip && entry->mReverseGrip->attached())
                    {
                        const auto observed = entry->mReverseGrip->observe();
                        entry->mWeapon->mLastLinearImpulse = observed ? observed->mImpulseOnSecond.Length() : 0.0f;
                        entry->mWeapon->mLastAngularImpulse = 0.0f;
                        if (mPassiveRagdollProfile)
                            entry->observeSupportGripAfterStep(mJoltStepSerial, mRagdollWeaponSwing,
                                entry->mRuntimeProfile ? *entry->mRuntimeProfile : *mPassiveRagdollProfile);
                    }
                    else if (entry->mRagdoll->getConstraintImpulse(entry->mWeapon->mWeaponBody, impulse))
                    {
                        entry->mWeapon->mLastLinearImpulse = impulse.mLinear;
                        entry->mWeapon->mLastAngularImpulse = impulse.mAngular;
                        if (mPassiveRagdollProfile)
                            entry->observeSupportGripAfterStep(mJoltStepSerial, mRagdollWeaponSwing,
                                entry->mRuntimeProfile ? *entry->mRuntimeProfile : *mPassiveRagdollProfile);
                        if (impulse.mLinear > entry->mWeapon->mLinearBreakImpulse
                            || impulse.mAngular > entry->mWeapon->mAngularBreakImpulse)
                            releaseWeapon(*entry, PassiveRagdollWeaponReleaseReason::Break);
                    }
                    else if (entry->mSupportGrip && entry->mSupportGrip->attached())
                    {
                        // No independent second budget when the primary load
                        // cannot be measured; remove the link before rollback.
                        entry->clearNativeSupportGrip("missing-primary-impulse");
                        if (mPassiveRagdollProfile)
                            entry->clearWeaponSupportControl(mRagdollWeaponSwing,
                                entry->mRuntimeProfile ? *entry->mRuntimeProfile : *mPassiveRagdollProfile);
                        entry->failPoweredControl(__LINE__);
                    }
                }
                if (entry->mJointSupportControlApplied)
                {
                    for (std::size_t i = 1; i < entry->mRagdoll->getBodyCount(); ++i)
                    {
                        JPH::Vec3 impulse;
                        float utilization = 0;
                        if (entry->isDetachedWeaponBody(i))
                            continue;
                        const bool measured = entry->mRagdoll->getMotorAngularImpulse(i, mPhysicsDt, impulse, utilization);
                        if (!measured)
                            entry->failPoweredControl(__LINE__);
                        entry->mJointSupportMotorUtilization = std::max(entry->mJointSupportMotorUtilization, utilization);
                    }
                }
                entry->mJointMotorUtilizations.assign(entry->mRagdoll->getBodyCount(), -1.0f);
                if (entry->mJointMotorsApplied)
                    for (std::size_t i = 1; i < entry->mRagdoll->getBodyCount(); ++i)
                    {
                        if (entry->isDetachedWeaponBody(i))
                            continue;
                        JPH::Vec3 impulse;
                        float utilization = 0.0f;
                        if (!entry->mRagdoll->getMotorAngularImpulse(i, mPhysicsDt, impulse, utilization))
                            entry->failPoweredControl(__LINE__);
                        else
                        {
                            entry->mJointMotorPeakUtilization
                                = std::max(entry->mJointMotorPeakUtilization, utilization);
                            entry->mJointMotorUtilizations[i] = utilization;
                        }
                    }
                if (entry->mPowered)
                {
                    if (entry->mJointMotorsApplied && entry->mGaitOutput && entry->mGaitOutput->hasStance()
                        && entry->mLeftLeg && entry->mRightLeg)
                    {
                        const RagdollLegChain& swingLeg
                            = entry->mGaitOutput->leftFootIsStance() ? *entry->mRightLeg : *entry->mLeftLeg;
                        const auto captureUtilization = [&](std::size_t bodyIndex, std::optional<float>& result) {
                            JPH::Vec3 impulse;
                            float utilization = 0.0f;
                            if (entry->mRagdoll->getMotorAngularImpulse(
                                    bodyIndex, mPhysicsDt, impulse, utilization))
                                result = utilization;
                        };
                        captureUtilization(swingLeg.mHip, entry->mSwingHipMotorUtilization);
                        captureUtilization(swingLeg.mKnee, entry->mSwingKneeMotorUtilization);
                        captureUtilization(swingLeg.mFoot, entry->mSwingAnkleMotorUtilization);
                    }
                    const auto actor = mActors.find(entry->getPtr().mRef);
                    if (actor == mActors.end() || !entry->synchronizePoweredRoot(*actor->second, mPhysicsDt))
                        entry->failPoweredControl(__LINE__);
                }
                else if (corpseFollow())
                    if (const auto actor = mActors.find(entry->getPtr().mRef); actor != mActors.end())
                        followCorpse(*entry, *actor->second, mPhysicsSystem.GetBodyInterface());
                entry->mCurrentPhysicsPoseCaptured = entry->mRagdoll->captureModelPose(entry->mCurrentPhysicsPose);
                if (mPassiveRagdollProfile)
                {
                    entry->updateControllerObservation(mPassiveRagdollProfile->mActiveController, mPhysicsDt, mPhysicsSystem);
                    if (entry->mSupportGripEnabled && entry->mControllerOutput
                        && !RagdollCore::isDrivenControllerMode(entry->mControllerOutput->mMode))
                    {
                        entry->clearNativeSupportGrip("physical-recovery");
                        entry->clearWeaponSupportControl(mRagdollWeaponSwing,
                            entry->mRuntimeProfile ? *entry->mRuntimeProfile : *mPassiveRagdollProfile);
                    }
                    // Kinematic locomotion root: driven once per fixed step
                    // after observation and before the solver, so the actor
                    // transform for this step is the one the solver sees.
                    entry->updateKinematicRoot(
                        mPassiveRagdollProfile->mActiveController, mPhysicsDt, mPhysicsSystem);
                }
            }
#ifdef OPENMW_RAGDOLL_DEV_API
            traceStep("observed");
#endif
            recordPassiveRagdollReplayFrame();
            captureNetPlayerPose();
            if (ManualDiagnostics::recorder().enabled())
            {
                const auto found = mPassiveRagdolls.find(MWMechanics::getPlayer().mRef);
                ManualDiagnostics::Frame frame;
                frame.tick = mJoltStepSerial;
                frame.dt = mPhysicsDt;
                if (found != mPassiveRagdolls.end())
                {
                    const auto& entry = *found->second;
                    frame.actor = entry.mSerial;
                    frame.attack = entry.mSwingIdentity;
                    frame.press = entry.mPlayerPressCount;
                    frame.phase = static_cast<int>(entry.mAnimationAttack.mPhase);
                    frame.held = entry.mSwingInput.mHeld;
                    const auto add = [&](std::size_t body, const char* name, int parent) {
                        ManualDiagnostics::Body b;
                        b.name = name; b.parent = parent;
                        const auto pose = [&](const RagdollCore::RagdollPose& source,
                            osg::Vec3f& position, osg::Quat& rotation) {
                            const auto p = entry.mRagdoll->getWorldBodyPosition(source, body);
                            const auto q = entry.mRagdoll->getWorldBodyRotation(source, body);
                            if (!p || !q) return false;
                            position = Misc::Convert::toOsg(*p);
                            rotation = osg::Quat(q->GetX(), q->GetY(), q->GetZ(), q->GetW());
                            return true;
                        };
                        b.animationValid = pose(entry.mMotorTargetPose, b.animation, b.animationRotation);
                        b.commandValid = entry.mManualMotorPoseApplied && entry.mManualMotorPoseTick == mJoltStepSerial
                            && std::find(entry.mManualDisabledMotors.begin(), entry.mManualDisabledMotors.end(), body)
                                == entry.mManualDisabledMotors.end()
                            && pose(entry.mManualMotorPose, b.command, b.commandRotation);
                        b.physicalValid = entry.mCurrentPhysicsPoseCaptured
                            && pose(entry.mCurrentPhysicsPose, b.physical, b.physicalRotation);
                        if (body < entry.mJointMotorUtilizations.size()) b.utilization = entry.mJointMotorUtilizations[body];
                        frame.bodies.push_back(std::move(b));
                    };
                    if (entry.mTorsoBodyIndex) add(*entry.mTorsoBodyIndex, "torso", -1);
                    const int torso = frame.bodies.empty() ? -1 : 0;
                    {
                        std::ostringstream body;
                        body << std::fixed << std::setprecision(2);
                        if (entry.mControllerOutput)
                            body << "mode=" << static_cast<int>(entry.mControllerOutput->mMode)
                                 << " t=" << entry.mControllerOutput->mTimeInMode;
                        if (entry.mRagdoll->getSoftRoot().mEnabled)
                        {
                            const auto& soft = entry.mRagdoll->getSoftRootTelemetry();
                            body << " sr=" << entry.mRagdoll->getSoftRootStrength() << " srF=" << soft.mForce.Length()
                                 << " srErr=" << soft.mPositionError << " srAng=" << soft.mAngleError
                                 << " kin=" << entry.mRagdoll->isRootKinematic();
                        }
                        body << " pin=" << entry.footfallPinnedAt(0) << entry.footfallPinnedAt(1)
                             << " swing=" << entry.footfallSwingingAt(0) << entry.footfallSwingingAt(1)
                             << " yield=" << entry.mAnchorYielding;
                        frame.body = body.str();
                    }
                    {
                        const osg::Vec3f playerPosition
                            = MWMechanics::getPlayer().getRefData().getPosition().asVec3();
                        const PassiveRagdollEntry* nearest = nullptr;
                        float nearestDistance = 1500.0f;
                        for (const auto& [otherRef, other] : mPassiveRagdolls)
                        {
                            if (!other->mPowered || other.get() == &entry || other->getPtr().isEmpty())
                                continue;
                            const float distance
                                = (other->getPtr().getRefData().getPosition().asVec3() - playerPosition).length();
                            if (distance < nearestDistance)
                            {
                                nearestDistance = distance;
                                nearest = other.get();
                            }
                        }
                        if (nearest && nearest->mControllerOutput && nearest->mCurrentPhysicsPoseCaptured
                            && nearest->mPelvisBodyIndex)
                        {
                            std::ostringstream npc;
                            npc << std::fixed << std::setprecision(1) << "id=" << nearest->getPtr().getCellRef().getRefId()
                                << " mode=" << static_cast<int>(nearest->mControllerOutput->mMode)
                                << " t=" << nearest->mControllerOutput->mTimeInMode;
                            if (const auto pelvis = nearest->mRagdoll->getWorldBodyPosition(
                                    nearest->mCurrentPhysicsPose, *nearest->mPelvisBodyIndex))
                                npc << " pelvis=" << pelvis->GetX() << ',' << pelvis->GetY() << ',' << pelvis->GetZ();
                            float deepest = -1.0e9f;
                            for (std::size_t i = 0; nearest->mSurfaceQuery
                                 && i < nearest->mCurrentPhysicsPose.mBodyTransforms.size(); ++i)
                                if (const auto centre
                                    = nearest->mRagdoll->getWorldBodyPosition(nearest->mCurrentPhysicsPose, i))
                                    if (const auto hit = nearest->mSurfaceQuery(
                                            JPH::RVec3(centre->GetX(), centre->GetY(), centre->GetZ() + 80.0),
                                            -JPH::Vec3::sAxisZ(), 160.0f))
                                        deepest = std::max(deepest, static_cast<float>(hit->first.GetZ() - centre->GetZ()));
                            if (deepest > -1.0e8f)
                                npc << " sink=" << deepest;
                            // Its weapon tip, attack phase and AI strike (user
                            // 2026-09-29: "the opponent swinging the weapon
                            // wildly"; the rows had nothing of its weapon).
                            if (nearest->mWeapon && nearest->mWeapon->mAttached && nearest->mRightArm)
                            {
                                const auto hand = nearest->mRagdoll->getWorldBodyPosition(
                                    nearest->mCurrentPhysicsPose, nearest->mRightArm->mHand);
                                const auto handRotation = nearest->mRagdoll->getWorldBodyRotation(
                                    nearest->mCurrentPhysicsPose, nearest->mRightArm->mHand);
                                if (hand && handRotation)
                                {
                                    const JPH::RVec3 tip = *hand
                                        + JPH::RVec3(*handRotation * nearest->mWeapon->mBladeAxis
                                            * std::max(nearest->mWeapon->tipDistanceFromHand(), 0.0f));
                                    npc << " wtip=" << tip.GetX() << ',' << tip.GetY() << ',' << tip.GetZ();
                                }
                                npc << " atk=" << static_cast<int>(nearest->mAnimationAttack.mPhase)
                                    << " strike=" << nearest->mAiStrikeActive << " tipc=" << nearest->mTip.mActive;
                            }
                            frame.npc = npc.str();
                        }
                    }
                    for (int side = 0; side < 2; ++side)
                    {
                        const auto& arm = side == 0 ? entry.mLeftArm : entry.mRightArm;
                        const auto& request = side == 0 ? entry.mLeftArmTarget : entry.mRightArmTarget;
                        if (!arm) continue;
                        const int shoulder = static_cast<int>(frame.bodies.size());
                        add(arm->mShoulder, side == 0 ? "left_shoulder" : "right_shoulder", torso);
                        add(arm->mElbow, side == 0 ? "left_elbow" : "right_elbow", shoulder);
                        add(arm->mHand, side == 0 ? "left_hand" : "right_hand", shoulder + 1);
                        if (request && frame.bodies.back().physicalValid)
                        {
                            frame.targets[side] = Misc::Convert::toOsg(request->mWorldTarget);
                            frame.handError[side] = (frame.targets[side] - frame.bodies.back().physical).length();
                        }
                    }
                    if (entry.mWeapon && entry.mWeapon->mAttached)
                    {
                        const auto& weapon = *entry.mWeapon;
                        frame.weapon = weapon.mRecordId;
                        frame.weaponAxis = Misc::Convert::toOsg(weapon.mBladeAxis);
                        frame.weaponLength = weapon.mLength;
                        add(weapon.mWeaponBody, "weapon", -1);
                        auto& b = frame.bodies.back();
                        // The weapon is carried by its grip constraint, not a
                        // driven arm joint. Its inherited animation-body slot
                        // is not the guided command. Show the primary hand's
                        // rigid-grip prediction and leave motor load unavailable.
                        b.utilization = -1;
                        b.animationValid = b.commandValid = false;
                        for (const auto& hand : frame.bodies)
                            if (hand.name == "right_hand")
                            {
                                b.animationValid = hand.animationValid;
                                b.commandValid = hand.commandValid;
                                b.animationRotation = hand.animationRotation;
                                b.commandRotation = hand.commandRotation;
                                b.animation = hand.animation + hand.animationRotation * Misc::Convert::toOsg(weapon.mCenterFromHand);
                                b.command = hand.command + hand.commandRotation * Misc::Convert::toOsg(weapon.mCenterFromHand);
                                break;
                            }
                        if (const auto hand = entry.mRagdoll->getWorldBodyPosition(entry.mCurrentPhysicsPose, weapon.mHandBody);
                            hand && b.physicalValid)
                            frame.grip[1] = (b.physical - b.physicalRotation * Misc::Convert::toOsg(weapon.mCenterFromHand)
                                - Misc::Convert::toOsg(*hand)).length();
                        if (entry.mSupportGripTelemetry.mAttached)
                            frame.grip[0] = entry.mSupportGripTelemetry.mAnchorSeparation;
                        // Tip: grip point plus the blade axis to its far end,
                        // for the physical weapon and the commanded hand.
                        const osg::Vec3f toTip = Misc::Convert::toOsg(weapon.mBladeAxis * weapon.tipDistanceFromHand());
                        const osg::Vec3f centre = Misc::Convert::toOsg(weapon.mCenterFromHand);
                        if (b.physicalValid)
                        {
                            frame.tip = b.physical + b.physicalRotation * (toTip - centre);
                            frame.tipValid = true;
                        }
                        for (const auto& hand : frame.bodies)
                            if (hand.name == "right_hand" && hand.commandValid)
                            {
                                frame.tipCommand = hand.command + hand.commandRotation * toTip;
                                frame.tipCommandValid = true;
                            }
                        // The tip controller's own command, when it runs.
                        if (entry.mTip.mActive && entry.mTipDesiredRotation)
                        {
                            frame.tipCommand = Misc::Convert::toOsg(entry.mTipDesiredTip);
                            frame.tipCommandValid = true;
                        }
                    }
                    // A bare fist under the tip controller: its tip is the hand.
                    if (!(entry.mWeapon && entry.mWeapon->mAttached) && entry.mTip.mActive && entry.mRightArm)
                        for (const auto& hand : frame.bodies)
                            if (hand.name == "right_hand" && hand.physicalValid)
                            {
                                frame.tip = hand.physical;
                                frame.tipValid = true;
                                frame.tipCommand = Misc::Convert::toOsg(entry.mTipDesiredTip);
                                frame.tipCommandValid = true;
                            }
                    // Wrist past its current constraint limit, degrees.
                    if (entry.mCurrentPhysicsPoseCaptured)
                        for (int side = 0; side < 2; ++side)
                        {
                            const auto& arm = side == 0 ? entry.mLeftArm : entry.mRightArm;
                            const auto& pose = entry.mCurrentPhysicsPose.mBodyTransforms;
                            if (!arm || arm->mHand >= pose.size() || arm->mElbow >= pose.size())
                                continue;
                            const auto geometry = entry.mRagdoll->getSwingTwistJointGeometry(arm->mHand);
                            if (!geometry)
                                continue;
                            const JPH::Quat child = pose[arm->mHand].mRotation;
                            const auto clamped = RagdollCore::clampSwingTwistJointOrientation(
                                *geometry, pose[arm->mElbow].mRotation, child);
                            if (clamped)
                                frame.wristExcess[side] = osg::RadiansToDegrees(
                                    2.0f * std::acos(std::clamp(std::abs(clamped->Dot(child)), 0.0f, 1.0f)));
                        }
                    // Appended after the arms and weapon (tools index those by name).
                    if (entry.mPelvisBodyIndex) add(*entry.mPelvisBodyIndex, "pelvis", -1);
                    if (entry.mLeftLeg) add(entry.mLeftLeg->mFoot, "left_foot", -1);
                    if (entry.mRightLeg) add(entry.mRightLeg->mFoot, "right_foot", -1);
                    if (entry.mSwingAimRayOrigin)
                    {
                        frame.eyeOrigin = Misc::Convert::toOsg(*entry.mSwingAimRayOrigin);
                        frame.eyeDirection = Misc::Convert::toOsg(entry.mSwingAimRayDirection);
                    }
                    if (entry.mPlayerStrikeAimPoint)
                    {
                        frame.aimPoint = Misc::Convert::toOsg(*entry.mPlayerStrikeAimPoint);
                        frame.aimValid = true;
                    }
                }
                {
                    const MWWorld::Ptr player = MWMechanics::getPlayer();
                    // (side, forward, running) as the controls ask; the
                    // character controller clears the movement settings
                    // before the physics step reads them.
                    if (const auto* controls = MWBase::Environment::get().getLuaManager()->getActorControls(player))
                        frame.move = osg::Vec3f(controls->mSideMovement, controls->mMovement, controls->mRun ? 1.0f : 0.0f);
                    frame.actorPosition = player.getRefData().getPosition().asVec3();
                    frame.actorYaw = player.getRefData().getPosition().rot[2];
                    frame.firstPerson = MWBase::Environment::get().getWorld()->isFirstPerson();
                    frame.drawState = static_cast<int>(player.getClass().getCreatureStats(player).getDrawState());
                }
                if (found != mPassiveRagdolls.end())
                {
                    frame.driveActive = found->second->mWeaponDriveActive;
                    frame.tipMode = found->second->mTip.mActive ? static_cast<int>(found->second->mTip.mMode) : -1;
                    const auto& support = found->second->mSupportGripTelemetry;
                    frame.support = std::string(support.mState) + ':' + std::string(support.mReason)
                        + (found->second->mSupportGripEnabled ? ":on" : ":off")
                        + (found->second->mWeaponSupportActive ? ":arm" : "");
                    // Hand control's view of the mouse (tools/body/mouse_view.py).
                    const auto& e = *found->second;
                    std::ostringstream hsc;
                    hsc.imbue(std::locale::classic());
                    hsc << std::setprecision(6);
                    const auto vec = [&](const char* name, double x, double y, double z) {
                        hsc << ",\"" << name << "\":[" << x << ',' << y << ',' << z << ']';
                    };
                    hsc << "{\"on\":" << e.mHscActive << ",\"stage\":" << static_cast<int>(e.mAttackStage)
                        << ",\"line\":" << e.mAttackLine << ",\"ang\":" << e.mAttackAngle
                        << ",\"cut_ang\":" << e.mAttackCutAngle << ",\"sx\":" << e.mStanceX
                        << ",\"sy\":" << e.mStanceY << ",\"vx\":" << e.mStrokeVX << ",\"vy\":" << e.mStrokeVY
                        << ",\"wind\":" << e.mAttackWind << ",\"cut\":" << e.mAttackCut
                        << ",\"u\":" << e.mFlowU << ",\"uv\":" << e.mFlowV << ",\"s\":" << e.mChamberS
                        << ",\"stroke\":[" << e.mCutStrokeX << ',' << e.mCutStrokeY << ']'
                        << ",\"snap\":" << e.mSnapCut << ",\"chain\":" << e.mChainFrom.has_value()
                        << ",\"chain_t\":" << e.mChainTime << ",\"overflow\":" << e.mCutOverflow
                        << ",\"inward\":" << e.mInwardPending << ",\"wait\":" << e.mWaitRecover
                        << ",\"auto\":" << e.mAutoAttack << ",\"held\":" << e.mHscWasHeld
                        << ",\"recover\":" << e.mAttackRecover << ",\"drive_phase\":" << e.mHscDrivePhase
                        << ",\"plane_miss\":" << e.mCutPlaneMiss << ",\"foe\":" << e.mHscFoeAssist << ",\"hand_err\":" << e.mChamberHandError
                        << ",\"wrist_err\":" << e.mChamberWristError;
                    vec("grip", e.mHscGrip.GetX(), e.mHscGrip.GetY(), e.mHscGrip.GetZ());
                    vec("blade", e.mHscBlade.GetX(), e.mHscBlade.GetY(), e.mHscBlade.GetZ());
                    if (e.mRightArmTarget)
                        vec("pole", e.mRightArmTarget->mWorldBend.GetX(), e.mRightArmTarget->mWorldBend.GetY(),
                            e.mRightArmTarget->mWorldBend.GetZ());
                    // Build 48: what the arm has to work with (the limp arm in
                    // play that tests never showed).
                    hsc << ",\"yield\":" << e.mHscDriveYield << ",\"ramp\":" << e.mWeaponDriveRamp
                        << ",\"guard_k\":" << e.mGuardStiffness << ",\"wound\":" << e.armWoundScale()
                        << ",\"tired\":" << e.tiredSwingScale() << ",\"fatigue\":" << e.mFatigueRatio
                        << ",\"drive_on\":" << e.mWeaponDriveActive << ",\"implausible\":" << e.mImplausibleTime
                        << ",\"reseat\":" << e.mReseatEase << ",\"reseats\":" << e.mReseatCount
                        << ",\"elbow_open\":" << e.mHscElbowOpen;
                    const auto motors = [&](const char* name, const auto& arm) {
                        if (!arm)
                            return;
                        hsc << ",\"" << name << "\":[";
                        bool first = true;
                        for (const std::size_t body : { arm->mFrame, arm->mShoulder, arm->mElbow, arm->mHand })
                        {
                            hsc << (first ? "" : ",");
                            first = false;
                            if (const auto m = e.mRagdoll->getMotorDebug(body))
                                hsc << '[' << m->mMaximumTorque << ',' << m->mFrequency << ',' << m->mDamping << ','
                                    << m->mState << ',' << osg::RadiansToDegrees(m->mTargetError) << ']';
                            else
                                hsc << "null";
                        }
                        hsc << ']';
                    };
                    motors("motors_r", e.mRightArm);
                    motors("motors_l", e.mLeftArm);
                    if (e.mRightArm)
                    {
                        if (const auto g = e.mRagdoll->getSwingTwistJointGeometry(e.mRightArm->mHand))
                            hsc << ",\"wrist_lim\":[" << osg::RadiansToDegrees(g->mNormalHalfConeAngle) << ','
                                << osg::RadiansToDegrees(g->mPlaneHalfConeAngle) << ','
                                << osg::RadiansToDegrees(g->mTwistMinimum) << ',' << osg::RadiansToDegrees(g->mTwistMaximum)
                                << ']';
                        if (const auto h = e.mRagdoll->getHingeJointLimits(e.mRightArm->mElbow))
                            hsc << ",\"elbow_lim\":[" << osg::RadiansToDegrees(h->first) << ','
                                << osg::RadiansToDegrees(h->second) << ']';
                        if (const auto g = e.mRagdoll->getSwingTwistJointGeometry(e.mRightArm->mShoulder))
                            hsc << ",\"shoulder_lim\":[" << osg::RadiansToDegrees(g->mNormalHalfConeAngle) << ','
                                << osg::RadiansToDegrees(g->mPlaneHalfConeAngle) << ','
                                << osg::RadiansToDegrees(g->mTwistMinimum) << ',' << osg::RadiansToDegrees(g->mTwistMaximum)
                                << ']';
                    }
                    hsc << ",\"own\":[\"" << e.armOwnerName(e.mBodyOwner[PassiveRagdollEntry::ChannelTrunk])
                        << "\",\"" << e.armOwnerName(e.mArmOwner[0]) << "\",\""
                        << e.armOwnerName(e.mArmOwner[1]) << "\"]}";
                    frame.hsc = hsc.str();
                }
                ManualDiagnostics::recorder().frame(std::move(frame));
            }
            ++mJoltStepSerial;
        }
        if (joltSteps == maxJoltStepsPerFrame && mTimeAccumJolt >= mPhysicsDt)
            mTimeAccumJolt = std::fmod(mTimeAccumJolt, mPhysicsDt);

#ifdef JPH_PROFILE_ENABLED
        JPH_PROFILE_NEXTFRAME();
#endif
    }

    void PhysicsSystem::updatePtrHolders()
    {
        for (auto& [_, object] : mObjects)
            object->commitPositionChange();
    }

    void PhysicsSystem::moveActors()
    {
        auto* player = getActor(MWMechanics::getPlayer());
        const auto world = MWBase::Environment::get().getWorld();
        const auto rootPosition = [this](const Actor& actor) -> osg::Vec3f {
            const auto ragdoll = mPassiveRagdolls.find(actor.getPtr().mRef);
            if (ragdoll != mPassiveRagdolls.end() && ragdoll->second->mPowered
                && ragdoll->second->mRagdoll
                // Only a released root owns the position. While the root is
                // kinematic the actor's own movement (player input or AI)
                // owns it; publishing the ragdoll reference instead pinned
                // every powered NPC in place, because the reference is taken
                // from the actor and the loop never advanced.
                && !ragdoll->second->mRagdoll->isRootKinematic())
            {
                // The rendered model pose is rebased to this reference. The
                // ordinary movement scheduler can still publish a cached root
                // on frames without a Jolt step; using it would translate the
                // whole skeleton back toward its pre-fall position.
                return Misc::Convert::toOsg(ragdoll->second->mRagdoll->getReferencePosition());
            }
            return actor.getSimulationPosition();
        };

        // copy new ptr position in temporary vector. player is handled separately as its movement might change active
        // cell.
        mActorsPositions.clear();
        if (!mActors.empty())
            mActorsPositions.reserve(mActors.size() - 1);
        for (const auto& [ptr, physicActor] : mActors)
        {
            if (physicActor.get() == player)
                continue;
            mActorsPositions.emplace_back(physicActor->getPtr(), rootPosition(*physicActor));
        }

        for (const auto& [ptr, pos] : mActorsPositions)
            world->moveObject(ptr, pos, false, false);

        if (player != nullptr)
            world->moveObject(player->getPtr(), rootPosition(*player), false, false);
    }

    namespace
    {
        // Reference loops (tools-workspace/mixamo/reference_loops.py): in-place
        // joint tracks in the character frame (x right, y forward, z up,
        // metres, floor at 0), the left lift frame lined up with Footfall's
        // clock 0.
        struct ReferenceLoop
        {
            int mFrames = 0;
            float mFps = 30.0f, mLeg = 0.9f;
            int mLiftFrame = 0;
            std::vector<std::array<osg::Vec3f, 18>> mPoses;
        };

        const std::map<std::string, ReferenceLoop>& referenceLoops()
        {
            static const auto loops_tuned = RagdollCore::tuned<std::map<std::string, ReferenceLoop>>([] {
                std::map<std::string, ReferenceLoop> result;
                const char* settings = RagdollCore::tuningText("OPENMW_RAGDOLL_SETTINGS");
                if (!settings)
                    return result;
                std::ifstream in(std::filesystem::path(settings).parent_path() / "reference-loops.txt");
                std::string text;
                ReferenceLoop* current = nullptr;
                int remaining = 0;
                while (std::getline(in, text))
                {
                    if (text.empty() || text[0] == '#')
                        continue;
                    std::istringstream row(text);
                    if (text.rfind("clip ", 0) == 0)
                    {
                        std::string word, key;
                        ReferenceLoop loop;
                        row >> word >> key >> loop.mFrames >> loop.mFps >> loop.mLeg >> loop.mLiftFrame;
                        current = &(result[key] = loop);
                        remaining = loop.mFrames;
                        continue;
                    }
                    if (!current || remaining <= 0)
                        continue;
                    std::array<osg::Vec3f, 18> pose;
                    for (auto& joint : pose)
                        row >> joint.x() >> joint.y() >> joint.z();
                    current->mPoses.push_back(pose);
                    --remaining;
                }
                return result;
            });
            static const auto& loops = loops_tuned.get();
            return loops;
        }

        template <class Line>
        void drawReferenceGhost(const PassiveRagdollEntry& entry, const MWWorld::Ptr& ptr, Line&& line)
        {
            if (!entry.mRagdoll || !entry.mLeftLeg || !entry.mRightLeg)
                return;
            const auto& loops = referenceLoops();
            const ESM::Position& position = ptr.getRefData().getPosition();
            const float yaw = position.rot[2];
            const osg::Vec3f forward(std::sin(yaw), std::cos(yaw), 0.0f);
            const osg::Vec3f right(std::cos(yaw), -std::sin(yaw), 0.0f);
            const osg::Vec3f up(0.0f, 0.0f, 1.0f);
            const JPH::Vec3 velocity = entry.mTargetHorizontalVelocity;
            const osg::Vec3f move(velocity.GetX(), velocity.GetY(), 0.0f);
            const float speed = move.length();
            // Which loop: walk or run, direction against the facing, armed or not.
            const float along = move * forward, across = move * right;
            std::string direction = "fwd";
            bool reversed = false;
            if (speed > 15.0f)
            {
                const float angle = std::atan2(across, along) * 180.0f / osg::PIf;
                if (std::abs(angle) > 135.0f)
                    direction = "back";
                else if (angle > 45.0f)
                    direction = "right";
                else if (angle < -45.0f)
                    direction = "left";
            }
            const bool armed = ptr.getClass().getCreatureStats(ptr).getDrawState() == MWMechanics::DrawState::Weapon
                && entry.mWeapon && entry.mWeapon->mAttached;
            const std::string gait = speed > 170.0f ? "run" : "walk";
            std::string key = std::string(armed ? "armed" : "") + gait + "_" + direction;
            if (!loops.count(key))
                key = gait + "_" + direction;
            if (!loops.count(key) && direction == "back")
            {
                key = gait + "_fwd";
                reversed = true;
            }
            if (!loops.count(key))
                key = "walk_fwd";
            const auto found = loops.find(key);
            if (found == loops.end() || found->second.mPoses.empty())
                return;
            const ReferenceLoop& loop = found->second;
            // Scale to the player's leg (hip to ankle).
            const auto geometry = entry.mRagdoll->getHingeLegGeometry(
                entry.mLeftLeg->mHip, entry.mLeftLeg->mKnee, entry.mLeftLeg->mFoot);
            const float legUnits = geometry ? geometry->mUpperOffset.Length() + geometry->mLowerOffset.Length() : 72.0f;
            const float scale = legUnits / std::max(loop.mLeg, 0.1f);
            // Phase: Footfall's clock while stepping (0 = left lift), else the loop's own time.
            static float freeTime = 0.0f;
            freeTime += 1.0f / 60.0f;
            float phase = entry.mFootfallLibrary && entry.mFootfallLibrary->walking()
                ? entry.mFootfallLibrary->clock()
                : std::fmod(freeTime * loop.mFps / loop.mFrames, 1.0f);
            if (reversed)
                phase = 1.0f - phase;
            float frame = loop.mLiftFrame + phase * loop.mFrames;
            frame = std::fmod(frame, static_cast<float>(loop.mFrames));
            const int a = static_cast<int>(frame) % loop.mFrames;
            const int b = (a + 1) % loop.mFrames;
            const float t = frame - std::floor(frame);
            const osg::Vec3f floor(position.pos[0], position.pos[1], position.pos[2]);
            std::array<osg::Vec3f, 18> world;
            for (std::size_t j = 0; j < world.size(); ++j)
            {
                const osg::Vec3f p = loop.mPoses[a][j] * (1.0f - t) + loop.mPoses[b][j] * t;
                world[j] = floor + (right * p.x() + forward * p.y() + up * p.z()) * scale;
            }
            const osg::Vec4f white(1, 1, 1, 1), leftTint(0.75f, 0.85f, 1, 1);
            static const std::array<std::array<int, 2>, 17> bones{ { { 0, 1 }, { 1, 2 }, { 2, 3 }, { 0, 4 },
                { 4, 5 }, { 5, 6 }, { 6, 7 }, { 0, 8 }, { 8, 9 }, { 9, 10 }, { 10, 11 }, { 1, 12 }, { 12, 13 },
                { 13, 14 }, { 1, 15 }, { 15, 16 }, { 16, 17 } } };
            for (const auto& bone : bones)
            {
                const bool left = (bone[1] >= 4 && bone[1] <= 7) || (bone[1] >= 12 && bone[1] <= 14);
                line(world[bone[0]], world[bone[1]], left ? leftTint : white);
            }
            // The physical legs, green, over it.
            const osg::Vec4f green(0.2f, 1, 0.3f, 1);
            if (entry.mPelvisBodyIndex)
                for (const auto* leg : { &*entry.mLeftLeg, &*entry.mRightLeg })
                {
                    std::optional<osg::Vec3f> previous;
                    for (const std::size_t body : { *entry.mPelvisBodyIndex, leg->mHip, leg->mKnee, leg->mFoot })
                    {
                        const auto at = entry.mRagdoll->getWorldBodyPosition(entry.mCurrentPhysicsPose, body);
                        if (!at)
                            break;
                        const osg::Vec3f p = Misc::Convert::toOsg(*at);
                        if (previous)
                            line(*previous, p, green);
                        previous = p;
                    }
                }
        }

        // The vanilla animation pose (the captured motor target; the legs'
        // stepping targets live in a separate drive pose) as a white stick
        // figure, left side tinted, and the physical body in green over it.
        template <class Line>
        void drawVanillaSkeleton(const PassiveRagdollEntry& entry, Line&& line)
        {
            if (!entry.mRagdoll || !entry.mRuntimeProfile)
                return;
            const auto& bodies = entry.mRuntimeProfile->mBodies;
            std::map<std::string, std::size_t> index;
            for (std::size_t i = 0; i < bodies.size(); ++i)
                index[bodies[i].mName] = i;
            const osg::Vec4f white(1, 1, 1, 1), leftTint(0.75f, 0.85f, 1, 1), green(0.2f, 1, 0.3f, 1);
            const auto at = [&](const RagdollCore::RagdollPose& pose, std::size_t body) -> std::optional<osg::Vec3f> {
                const auto p = entry.mRagdoll->getWorldBodyPosition(pose, body);
                if (!p)
                    return std::nullopt;
                return Misc::Convert::toOsg(*p);
            };
            for (std::size_t i = 0; i < bodies.size(); ++i)
            {
                if (!bodies[i].mParentBody)
                    continue;
                const auto parent = index.find(*bodies[i].mParentBody);
                if (parent == index.end())
                    continue;
                const bool left = bodies[i].mName.rfind("left_", 0) == 0;
                if (entry.mMotorTargetCaptured)
                    if (const auto a = at(entry.mMotorTargetPose, parent->second))
                        if (const auto b = at(entry.mMotorTargetPose, i))
                            line(*a, *b, left ? leftTint : white);
                if (entry.mCurrentPhysicsPoseCaptured)
                    if (const auto a = at(entry.mCurrentPhysicsPose, parent->second))
                        if (const auto b = at(entry.mCurrentPhysicsPose, i))
                            line(*a, *b, green);
            }
        }
    }

    void PhysicsSystem::debugDraw()
    {
        if (ManualDiagnostics::recorder().enabled())
        {
            const auto status = ManualDiagnostics::recorder().status();
            std::vector<MWRender::JoltDebugDrawer::ObservationLine> lines;
            const auto& frame = status.frame;
            const auto line = [&](const osg::Vec3f& a, const osg::Vec3f& b, const osg::Vec4f& color) {
                lines.push_back({ a, b, color });
            };
            const osg::Vec4f cyan(0, 0.85f, 1, 1), purple(0.85f, 0.35f, 1, 1), yellow(1, 1, 0, 1);
            // Modes 1 ("vanilla") and 2 ("targets"): the whole vanilla pose as
            // lines with the physical body over it, drawn like the reference
            // (user 2026-09-27/28: "the first V press to have the vanilla
            // morrowind animation as lines"; the recorder's frame has the torso
            // and arms only, which left the legs out).
            if (status.mode == 1 || status.mode == 2)
            {
                const MWWorld::Ptr playerPtr = MWMechanics::getPlayer();
                if (!playerPtr.isEmpty())
                    if (const auto found = mPassiveRagdolls.find(playerPtr.mRef); found != mPassiveRagdolls.end())
                        drawVanillaSkeleton(*found->second, line);
            }
            if (status.mode > 1)
                for (const auto& body : frame.bodies)
                {
                    const osg::Vec4f physical = body.utilization < 0 ? osg::Vec4f(0.5f, 0.5f, 0.5f, 1)
                        : osg::Vec4f(std::clamp(body.utilization, 0.0f, 1.0f),
                            1 - std::clamp(body.utilization, 0.0f, 1.0f), 0.15f, 1);
                    if (body.parent >= 0 && static_cast<std::size_t>(body.parent) < frame.bodies.size())
                    {
                        const auto& parent = frame.bodies[body.parent];
                        if (body.animationValid && parent.animationValid) line(parent.animation, body.animation, cyan);
                        if (body.commandValid && parent.commandValid) line(parent.command, body.command, purple);
                        if (body.physicalValid && parent.physicalValid) line(parent.physical, body.physical, physical);
                    }
                    if (body.name == "weapon")
                    {
                        const auto axis = frame.weaponAxis * (0.5f * frame.weaponLength);
                        if (body.animationValid) line(body.animation - body.animationRotation * axis,
                            body.animation + body.animationRotation * axis, cyan);
                        if (body.commandValid) line(body.command - body.commandRotation * axis,
                            body.command + body.commandRotation * axis, purple);
                        if (body.physicalValid) line(body.physical - body.physicalRotation * axis,
                            body.physical + body.physicalRotation * axis, physical);
                    }
                    if (status.mode == 2 && (body.name == "left_hand" || body.name == "right_hand"))
                    {
                        const int side = body.name == "left_hand" ? 0 : 1;
                        if (frame.handError[side] >= 0)
                        {
                            const auto p = frame.targets[side];
                            if (body.physicalValid) line(body.physical, p, yellow);
                            for (int axis = 0; axis < 3; ++axis)
                            {
                                osg::Vec3f offset; offset[axis] = 2;
                                line(p - offset, p + offset, yellow);
                            }
                        }
                    }
                }
            // Mode 3, "reference": the downloaded loop the legs follow, as a
            // white stick figure standing where the player stands, scaled to
            // its legs and stepping on Footfall's clock, with the physical legs
            // in green over it (user 2026-09-27: "put them as reference in the
            // game like some sort of loop ... so I can compare"). Loops from
            // tools-workspace/mixamo/reference_loops.py, next to the settings.
            if (status.mode == 3)
            {
                const MWWorld::Ptr playerPtr = MWMechanics::getPlayer();
                if (!playerPtr.isEmpty())
                    if (const auto found = mPassiveRagdolls.find(playerPtr.mRef); found != mPassiveRagdolls.end())
                        drawReferenceGhost(*found->second, playerPtr, line);
            }
            // Aim marker (aiming review 2026-09-30), any mode, weapon drawn:
            // an orange cross where a click aims the blade's sweet spot - the
            // crosshair at the depth of what it is on (a target out of reach
            // is swapped for the crosshair at the sweet spot's reach).
            // OPENMW_RAGDOLL_AIM_MARKER=1.
            static const auto aimMarker_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_AIM_MARKER");
                return text && std::atoi(text) != 0;
            });
            static const auto& aimMarker = aimMarker_tuned.get();
            if (aimMarker && frame.aimValid && frame.drawState == 1)
            {
                const osg::Vec4f orange(1, 0.55f, 0.1f, 1);
                for (int axis = 0; axis < 3; ++axis)
                {
                    osg::Vec3f offset;
                    offset[axis] = 4;
                    line(frame.aimPoint - offset, frame.aimPoint + offset, orange);
                }
            }
            mJoltDebugDrawer->setObservationLines(lines);
        }
        if (mDebugDrawEnabled)
        {
            std::vector<std::pair<osg::Vec3f, osg::Vec3f>> targetLines;
            for (const auto& [_, entry] : mPassiveRagdolls)
            {
                if (!entry->mCurrentPhysicsPoseCaptured)
                    continue;
                const std::size_t bodyCount = entry->mRagdoll->getBodyCount();
                for (std::size_t i = 0; i < bodyCount; ++i)
                {
                    const auto target = entry->mRagdoll->getWorldBodyPosition(entry->mTransitionSourcePose, i);
                    const auto physics = entry->mRagdoll->getWorldBodyPosition(entry->mCurrentPhysicsPose, i);
                    if (target && physics)
                        targetLines.emplace_back(Misc::Convert::toOsg(*target), Misc::Convert::toOsg(*physics));
                }
            }
            mJoltDebugDrawer->setRagdollTargetLines(std::move(targetLines));
            mTaskScheduler->debugDraw();
        }
    }

    bool PhysicsSystem::isActorStandingOn(const MWWorld::Ptr& actor, const MWWorld::ConstPtr& object) const
    {
        const auto physActor = mActors.find(actor.mRef);
        if (physActor != mActors.end())
            return physActor->second->getStandingOnPtr() == object;
        return false;
    }

    void PhysicsSystem::getActorsStandingOn(const MWWorld::ConstPtr& object, std::vector<MWWorld::Ptr>& out) const
    {
        for (const auto& [_, actor] : mActors)
        {
            if (actor->getStandingOnPtr() == object)
                out.emplace_back(actor->getPtr());
        }
    }

    bool PhysicsSystem::isObjectCollidingWith(const MWWorld::ConstPtr& object, ScriptedCollisionType type) const
    {
        auto found = mObjects.find(object.mRef);
        if (found != mObjects.end())
            return found->second->collidedWith(type);
        return false;
    }

    void PhysicsSystem::getActorsCollidingWith(const MWWorld::ConstPtr& object, std::vector<MWWorld::Ptr>& out) const
    {
        std::vector<MWWorld::Ptr> collisions
            = getCollisions(object, Layers::WORLD, Layers::ACTOR | Layers::POWERED_ACTOR);
        out.insert(out.end(), collisions.begin(), collisions.end());
    }

    void PhysicsSystem::disableWater()
    {
        if (mWaterEnabled)
        {
            mWaterEnabled = false;
            updateWater();
        }
    }

    void PhysicsSystem::enableWater(float height)
    {
        if (!mWaterEnabled || mWaterHeight != height)
        {
            mWaterEnabled = true;
            mWaterHeight = height;
            updateWater();
        }
    }

    void PhysicsSystem::setWaterHeight(float height)
    {
        if (mWaterHeight != height)
        {
            mWaterHeight = height;
            updateWater();
        }
    }

    void PhysicsSystem::updateWater()
    {
        if (!mWaterEnabled)
        {
            mWaterInstance.reset();
            return;
        }

        mWaterInstance = std::make_unique<MWWater>(mTaskScheduler.get(), mWaterHeight);
    }

    bool PhysicsSystem::isAreaOccupiedByOtherActor(const osg::Vec3f& position, const float radius,
        std::span<const MWWorld::ConstPtr> ignore, std::vector<MWWorld::Ptr>* occupyingActors) const
    {
        // Build list of ignored body IDs from theire actor ptrs
        std::vector<JPH::BodyID> ignoredObjects;
        ignoredObjects.reserve(ignore.size());
        for (const auto& v : ignore)
            if (const auto it = mActors.find(v.mRef); it != mActors.end())
                ignoredObjects.push_back(it->second->getPhysicsBody());
        std::sort(ignoredObjects.begin(), ignoredObjects.end());
        ignoredObjects.erase(std::unique(ignoredObjects.begin(), ignoredObjects.end()), ignoredObjects.end());

        // Define a tiny private class here as its not used anywhere else
        class AreaActorCollector : public JPH::CollideShapeBodyCollector
        {
        public:
            AreaActorCollector(const JPH::PhysicsSystem* inSystem, std::vector<JPH::BodyID>* ignoredBodies,
                std::vector<MWWorld::Ptr>* occupiedResult)
                : mSystem(inSystem)
                , mIgnoredBodies(ignoredBodies)
                , mOccupiedResult(occupiedResult)
            {
            }

            virtual void AddHit(const JPH::BodyID& inBodyID) override
            {
                // Check if we should ignore this body
                if (std::binary_search(mIgnoredBodies->begin(), mIgnoredBodies->end(), inBodyID))
                    return;

                // Bool-only callers need to know just if any actor occupies the area.
                if (mOccupiedResult == nullptr)
                {
                    mHasHit = true;
                    ForceEarlyOut();
                    return;
                }

                // If an output array is specified, get body ptrholder
                if (mOccupiedResult)
                {
                    JPH::BodyLockRead lock(mSystem->GetBodyLockInterface(), inBodyID);
                    if (lock.Succeeded())
                    {
                        const JPH::Body& body = lock.GetBody();
                        if (PtrHolder* holder = Misc::Convert::toPointerFromUserData<PtrHolder>(body.GetUserData()))
                            mOccupiedResult->push_back(holder->getPtr());
                    }
                }

                mHasHit = true;
            }

            bool hasHit() { return mHasHit; }

        private:
            const JPH::PhysicsSystem* mSystem;
            std::vector<JPH::BodyID>* mIgnoredBodies;
            std::vector<MWWorld::Ptr>* mOccupiedResult;
            bool mHasHit = false;
        };

        // Setup collector and collide body AABBs by the input sphere
        AreaActorCollector collector(&mPhysicsSystem, &ignoredObjects, occupyingActors);
        JPH::DefaultBroadPhaseLayerFilter broadphaseLayerFilter
            = mPhysicsSystem.GetDefaultBroadPhaseLayerFilter(Layers::ACTOR | Layers::POWERED_ACTOR);
        mPhysicsSystem.GetBroadPhaseQuery().CollideSphere(Misc::Convert::toJolt<JPH::Vec3>(position), radius, collector,
            broadphaseLayerFilter, JPH::SpecifiedObjectLayerFilter(Layers::ACTOR | Layers::POWERED_ACTOR));

        return collector.hasHit();
    }

    void PhysicsSystem::reportStats(unsigned int frameNumber, osg::Stats& stats) const
    {
        stats.setAttribute(frameNumber, "Physics Actors", static_cast<double>(mActors.size()));
        stats.setAttribute(frameNumber, "Physics Objects", static_cast<double>(mObjects.size()));
        stats.setAttribute(frameNumber, "Physics Projectiles", static_cast<double>(mProjectiles.size()));
        stats.setAttribute(frameNumber, "Physics HeightFields", static_cast<double>(mHeightFields.size()));
    }

    void PhysicsSystem::reportCollision(const osg::Vec3f& position, const osg::Vec3f& normal)
    {
        if (mDebugDrawEnabled)
            mJoltDebugDrawer->addCollision(position, normal);
    }

    const JPH::BodyLockInterfaceLocking& PhysicsSystem::getBodyLockInterface() const
    {
        return mPhysicsSystem.GetBodyLockInterface();
    }

    const JPH::BodyInterface& PhysicsSystem::getBodyInterface() const
    {
        return mPhysicsSystem.GetBodyInterface();
    }

    ActorFrameData::ActorFrameData(
        Actor& actor, bool inert, bool waterCollision, float slowFall, float waterlevel, bool isPlayer)
        : mPosition()
        , mIsOnGround(actor.getOnGround())
        , mIsOnSlope(actor.getOnSlope())
        , mWalkingOnWater(false)
        , mInert(inert)
        , mPhysicsBody(actor.getPhysicsBody())
        , mSwimLevel(waterlevel
              - (actor.getRenderingHalfExtents().z() * 2
                  * MWBase::Environment::get()
                      .getESMStore()
                      ->get<ESM::GameSetting>()
                      .find("fSwimHeightScale")
                      ->mValue.getFloat()))
        , mSlowFall(slowFall)
        , mRotation()
        , mMovement(actor.velocity())
        , mWaterlevel(waterlevel)
        , mHalfExtentsZ(actor.getHalfExtents().z())
        , mOldHeight(0)
        , mStuckFrames(0)
        , mFlying(MWBase::Environment::get().getWorld()->isFlying(actor.getPtr()))
        , mWasOnGround(actor.getOnGround())
        , mIsAquatic(actor.getPtr().getClass().isPureWaterCreature(actor.getPtr()))
        , mWaterCollision(waterCollision)
        , mSkipCollisionDetection(!actor.getCollisionMode())
        , mIsPlayer(isPlayer)
        , mCollisionMask(actor.getCollisionMask())
    {
    }

    WorldFrameData::WorldFrameData()
        : mIsInStorm(MWBase::Environment::get().getWorld()->isInStorm())
        , mStormDirection(MWBase::Environment::get().getWorld()->getStormDirection())
    {
    }

    LOSRequest::LOSRequest(const std::weak_ptr<Actor>& a1, const std::weak_ptr<Actor>& a2)
        : mResult(false)
        , mStale(false)
        , mAge(0)
    {
        // we use raw actor pointer pair to uniquely identify request
        // sort the pointer value in ascending order to not duplicate equivalent requests, eg. getLOS(A, B) and
        // getLOS(B, A)
        auto* raw1 = a1.lock().get();
        auto* raw2 = a2.lock().get();
        assert(raw1 != raw2);
        if (raw1 < raw2)
        {
            mActors = { a1, a2 };
            mRawActors = { raw1, raw2 };
        }
        else
        {
            mActors = { a2, a1 };
            mRawActors = { raw2, raw1 };
        }
    }

    bool operator==(const LOSRequest& lhs, const LOSRequest& rhs) noexcept
    {
        return lhs.mRawActors == rhs.mRawActors;
    }
}
