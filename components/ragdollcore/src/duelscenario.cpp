#include <ragdollcore/duelscenario.hpp>
#include <ragdollcore/tuning.hpp>

#include <ragdollcore/builder.hpp>
#include <ragdollcore/contact.hpp>
#include <ragdollcore/controller.hpp>
#include <ragdollcore/gaitcontroller.hpp>
#include <ragdollcore/impactdamage.hpp>
#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/reactionmechanics.hpp>
#include <ragdollcore/reactionpipeline.hpp>
#include <ragdollcore/unitmodel.hpp>

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
// clang-format on

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string_view>
#include <tuple>
#include <unordered_map>

namespace RagdollCore
{
    namespace
    {
        constexpr JPH::ObjectLayer sGroundLayer = 0;
        constexpr JPH::ObjectLayer sLeftLayer = 1;
        constexpr JPH::ObjectLayer sRightLayer = 2;
        // Weapons live on their own layers: they hit the opponent but never
        // the ground. A rigid weapon pinned through the floor otherwise drags
        // or launches the body through the solver.
        constexpr JPH::ObjectLayer sWeaponLeftLayer = 3;
        constexpr JPH::ObjectLayer sWeaponRightLayer = 4;
        // A corpse only collides with the ground: the killer's weapon cannot
        // drag or toss a body that is already down, and a dropped weapon is
        // inert.
        constexpr JPH::ObjectLayer sCorpseLayer = 5;
        constexpr JPH::ObjectLayer sInertLayer = 6;

        bool isFinite(float value) { return std::isfinite(value); }

        class DuelBroadPhaseLayers final : public JPH::BroadPhaseLayerInterface
        {
        public:
            JPH::uint GetNumBroadPhaseLayers() const override { return 2; }
            JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override
            {
                return JPH::BroadPhaseLayer(static_cast<JPH::BroadPhaseLayer::Type>(
                    layer == sGroundLayer ? 0 : 1));
            }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
            const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override
            {
                return layer == JPH::BroadPhaseLayer(0) ? "ground" : "ragdoll";
            }
#endif
        };

        class DuelObjectVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer broadPhase) const override
            {
                return layer != sGroundLayer || broadPhase != JPH::BroadPhaseLayer(0);
            }
        };

        // Ground collides with everything; the two fighters are on separate
        // layers so a weapon never hits its owner, while every cross-fighter
        // pair (weapon/body) collides.
        class DuelObjectPairs final : public JPH::ObjectLayerPairFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer first, JPH::ObjectLayer second) const override
            {
                if (first == sInertLayer || second == sInertLayer)
                    return false;
                if (first == sGroundLayer)
                    return second == sLeftLayer || second == sRightLayer || second == sCorpseLayer;
                if (second == sGroundLayer)
                    return first == sLeftLayer || first == sRightLayer || first == sCorpseLayer;
                if (first == sCorpseLayer || second == sCorpseLayer)
                    return false;
                if (first == second)
                    return false; // no self collisions within a fighter
                const bool firstWeapon
                    = first == sWeaponLeftLayer || first == sWeaponRightLayer;
                const bool secondWeapon
                    = second == sWeaponLeftLayer || second == sWeaponRightLayer;
                if (firstWeapon && secondWeapon)
                    return true; // weapons may clash
                if (firstWeapon)
                    return first == (second == sLeftLayer ? sWeaponRightLayer : sWeaponLeftLayer);
                if (secondWeapon)
                    return second == (first == sLeftLayer ? sWeaponRightLayer : sWeaponLeftLayer);
                return true; // left body against right body
            }
        };

        struct StrikeContact
        {
            int mAttacker = 0;
            std::size_t mAttackerBody = 0;
            std::size_t mDefenderBody = 0;
            bool mWeapon = false;
            JPH::RVec3 mPoint = JPH::RVec3::sZero();
            JPH::Vec3 mNormal = JPH::Vec3::sAxisY();
            float mApproach = 0.0f;
            float mEffectiveMass = 0.0f;
        };

        class DuelContactListener final : public JPH::ContactListener
        {
        public:
            struct Registry
            {
                std::unordered_map<std::uint32_t, std::pair<int, std::size_t>> mBody;
                std::array<std::size_t, 2> mWeapon{};
                std::array<std::size_t, 2> mHand{};
            };

            void setRegistry(Registry registry) { mRegistry = std::move(registry); }

            void beginStep()
            {
                mContacts.clear();
                mGround[0].clear();
                mGround[1].clear();
            }

            const std::vector<StrikeContact>& contacts() const { return mContacts; }

            const std::vector<GroundContactObservation>& ground(int actor) const
            {
                return mGround[static_cast<std::size_t>(actor)];
            }

            void OnContactAdded(const JPH::Body& body1, const JPH::Body& body2,
                const JPH::ContactManifold& manifold, JPH::ContactSettings& settings) override
            {
                observe(body1, body2, manifold, settings.mCombinedFriction);
            }

            void OnContactPersisted(const JPH::Body& body1, const JPH::Body& body2,
                const JPH::ContactManifold& manifold, JPH::ContactSettings& settings) override
            {
                observe(body1, body2, manifold, settings.mCombinedFriction);
            }

        private:
            void observe(const JPH::Body& body1, const JPH::Body& body2,
                const JPH::ContactManifold& manifold, float friction)
            {
                const auto first = mRegistry.mBody.find(body1.GetID().GetIndexAndSequenceNumber());
                const auto second = mRegistry.mBody.find(body2.GetID().GetIndexAndSequenceNumber());
                if (first == mRegistry.mBody.end() && second == mRegistry.mBody.end())
                    return;
                if (first != mRegistry.mBody.end() && second != mRegistry.mBody.end())
                {
                    if (first->second.first == second->second.first)
                        return;
                }
                else
                {
                    // One registered body against the world: a ground contact
                    // for that fighter (foot support, weapon scraping).
                    const bool firstBody = first != mRegistry.mBody.end();
                    const auto& entry = firstBody ? *first : *second;
                    appendGroundContactObservations(mGround[static_cast<std::size_t>(entry.second.first)],
                        entry.second.second, body1, body2, manifold, firstBody, friction);
                    return;
                }

                const auto strikeKind = [&](const std::pair<const std::uint32_t,
                                               std::pair<int, std::size_t>>& entry, bool& weapon) {
                    const int actor = entry.second.first;
                    const std::size_t index = entry.second.second;
                    if (index == mRegistry.mWeapon[actor])
                    {
                        weapon = true;
                        return true;
                    }
                    if (index == mRegistry.mHand[actor])
                    {
                        weapon = false;
                        return true;
                    }
                    return false;
                };
                bool firstWeapon = false;
                bool secondWeapon = false;
                const bool firstStrikes = strikeKind(*first, firstWeapon);
                const bool secondStrikes = strikeKind(*second, secondWeapon);
                if (firstStrikes == secondStrikes)
                    return; // two striking bodies meeting is a clash, not a hit
                int attacker = -1;
                std::size_t attackerBody = 0;
                std::size_t defenderBody = 0;
                bool weapon = false;
                if (firstStrikes)
                {
                    attacker = first->second.first;
                    attackerBody = first->second.second;
                    defenderBody = second->second.second;
                    weapon = firstWeapon;
                }
                else
                {
                    attacker = second->second.first;
                    attackerBody = second->second.second;
                    defenderBody = first->second.second;
                    weapon = secondWeapon;
                }

                JPH::RVec3 point = manifold.mBaseOffset;
                if (!manifold.mRelativeContactPointsOn1.empty() && !manifold.mRelativeContactPointsOn2.empty())
                {
                    const JPH::RVec3 onFirst = manifold.GetWorldSpaceContactPointOn1(0);
                    const JPH::RVec3 onSecond = manifold.GetWorldSpaceContactPointOn2(0);
                    point = onFirst + (onSecond - onFirst) * 0.5f;
                }
                JPH::Vec3 normal = manifold.mWorldSpaceNormal;
                const JPH::RVec3 firstCenter = body1.GetCenterOfMassPosition();
                const JPH::RVec3 secondCenter = body2.GetCenterOfMassPosition();
                if (normal.Dot(JPH::Vec3(secondCenter - firstCenter)) < 0.0f)
                    normal = -normal;

                const JPH::Vec3 firstVelocity = body1.GetPointVelocity(point);
                const JPH::Vec3 secondVelocity = body2.GetPointVelocity(point);
                const bool attackerIsFirst = attacker == first->second.first;
                const JPH::Vec3 attackerVelocity = attackerIsFirst ? firstVelocity : secondVelocity;
                const JPH::Vec3 defenderVelocity = attackerIsFirst ? secondVelocity : firstVelocity;
                const JPH::Vec3 towardDefender = attackerIsFirst ? normal : -normal;
                const float approach = (attackerVelocity - defenderVelocity).Dot(towardDefender);
                if (!(approach > 0.0f) || !std::isfinite(approach))
                    return;

                const float inverseMassFirst = body1.GetMotionProperties()->GetInverseMass();
                const float inverseMassSecond = body2.GetMotionProperties()->GetInverseMass();
                const float inverseTotal = inverseMassFirst + inverseMassSecond;
                const float effectiveMass = inverseTotal > 0.0f ? 1.0f / inverseTotal : 0.0f;

                StrikeContact contact;
                contact.mAttacker = attacker;
                contact.mAttackerBody = attackerBody;
                contact.mDefenderBody = defenderBody;
                contact.mWeapon = weapon;
                contact.mPoint = point;
                contact.mNormal = towardDefender;
                contact.mApproach = approach;
                contact.mEffectiveMass = effectiveMass;
                mContacts.push_back(contact);
            }

            Registry mRegistry;
            std::vector<StrikeContact> mContacts;
            std::array<std::vector<GroundContactObservation>, 2> mGround;
        };

        std::size_t findBody(const RagdollProfile& profile, std::string_view name)
        {
            for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
                if (profile.mBodies[i].mName == name)
                    return i;
            return std::numeric_limits<std::size_t>::max();
        }

        std::size_t findBone(const SkeletonDescriptor& skeleton, std::string_view name)
        {
            for (std::size_t i = 0; i < skeleton.mBones.size(); ++i)
                if (skeleton.mBones[i].mName == name)
                    return i;
            return std::numeric_limits<std::size_t>::max();
        }

        // Appends a rigid training weapon to the right hand: a bone for the
        // grip, a thin box body and a zero-cone joint, exactly like the live
        // training weapon path.
        bool appendDuelWeapon(SkeletonDescriptor& skeleton, RagdollProfile& profile, BuildOptions& options,
            const DuelWeaponProfile& weapon, float actorMass)
        {
            if (!weapon.mArmed || !profile.mLandmarks.mRightHand)
                return !weapon.mArmed;
            const std::size_t hand = findBody(profile, *profile.mLandmarks.mRightHand);
            if (hand == std::numeric_limits<std::size_t>::max() || hand >= skeleton.mBones.size())
                return false;
            const std::size_t handBoneIndex = findBone(skeleton, profile.mBodies[hand].mBone);
            if (handBoneIndex == std::numeric_limits<std::size_t>::max())
                return false;

            const float weaponMass = std::max(0.1f, weapon.mMass);
            const float totalMass = std::max(1.0f, actorMass) + weaponMass;
            const float length = std::max(4.0f, weapon.mLength);
            const float thickness = std::max(0.5f, weapon.mThickness);
            const float gripToCenter = 0.44f * length;
            for (auto& body : profile.mBodies)
                body.mMassFraction *= actorMass / totalMass;

            SkeletonBone bone;
            bone.mName = "equipped_weapon";
            bone.mParent = static_cast<int>(handBoneIndex);
            bone.mModelTransform = skeleton.mBones[handBoneIndex].mModelTransform;
            const JPH::Vec3 offset(0.0f, gripToCenter, 0.0f);
            bone.mModelTransform.mTranslation
                += bone.mModelTransform.mRotation * offset;
            skeleton.mBones.push_back(bone);

            BodyProfile weaponBody;
            weaponBody.mName = weaponBody.mBone = bone.mName;
            weaponBody.mParentBody = profile.mBodies[hand].mName;
            weaponBody.mMassFraction = weaponMass / totalMass;
            weaponBody.mShape.mType = ShapeType::Box;
            weaponBody.mShape.mHalfExtents
                = JPH::Vec3(thickness * 0.5f, length * 0.5f, thickness * 0.5f);
            weaponBody.mConstraint = ConstraintProfile{};
            weaponBody.mConstraint->mChildAnchor = -offset;
            weaponBody.mConstraint->mNormalHalfConeAngle = 0.0f;
            weaponBody.mConstraint->mPlaneHalfConeAngle = 0.0f;
            weaponBody.mConstraint->mTwistMinAngle = 0.0f;
            weaponBody.mConstraint->mTwistMaxAngle = 0.0f;
            weaponBody.mConstraint->mMotor.mMaximumTorque = 0.0f;
            profile.mBodies.push_back(weaponBody);
            options.mTotalMass = totalMass;
            return true;
        }

        struct Duelist
        {
            SkeletonDescriptor mSkeleton;
            RagdollProfile mProfile;
            BuildOptions mOptions;
            std::unique_ptr<PassiveRagdoll> mRagdoll;
            std::vector<std::string> mBodyNames;
            RagdollPose mBindPose;
            std::vector<JPH::Vec3> mBindPositions;
            std::vector<int> mParents;
            std::size_t mWeaponBody = 0;
            std::size_t mHandBody = 0;
            std::size_t mUpperArmBody = 0;
            std::size_t mForearmBody = 0;
            std::vector<std::size_t> mConstraintBodies;
            JPH::RVec3 mRootPosition = JPH::RVec3::sZero();
            JPH::Quat mRootRotation = JPH::Quat::sIdentity();
            std::size_t mPelvisBody = 0;
            std::size_t mTorsoBody = 0;
            FootBodyIndices mFeet;
            JPH::Vec3 mPelvisLocalUp = JPH::Vec3::sAxisZ();
            JPH::Vec3 mTorsoLocalUp = JPH::Vec3::sAxisZ();
            float mTargetPelvisHeight = 66.0f;
            std::vector<GroundContact> mContacts;
            std::optional<JointSupportOutput> mSupport;
            std::optional<SupportEstimate> mSupportEstimate;
            std::optional<UprightControllerOutput> mUpright;
            bool mPelvisHeightLatched = false;
            float mHealth = 0.0f;
            bool mAlive = true;
            bool mDeathHandled = false;
            std::uint64_t mDeathStep = 0;
            JPH::RVec3 mDeathPosition = JPH::RVec3::sZero();
            JPH::Quat mDeathRotation = JPH::Quat::sIdentity();
            JPH::Quat mCollapseRotation = JPH::Quat::sIdentity();
            ReactionPipelineState mReaction;
            ReactionOutputs mReactionOutputs;
            std::int64_t mLastHitSwing = -1;
            float mLatestHealthLoss = 0.0f;
            AnatomicalRegion mLatestRegion = AnatomicalRegion::Unknown;
            float mLatestImpulse = 0.0f;
            bool mHitThisStep = false;
        };

        float smoothStep(float value)
        {
            const float x = std::clamp(value, 0.0f, 1.0f);
            return x * x * (3.0f - 2.0f * x);
        }

        // Thrust profile: draw back, extend through the target, recover.
        void swingAngles(float progress, float& shoulderAngle, float& elbowAngle)
        {
            if (progress < 0.30f)
            {
                const float t = smoothStep(progress / 0.30f);
                shoulderAngle = -0.35f * t;
                elbowAngle = -1.05f * t;
            }
            else if (progress < 0.55f)
            {
                const float t = smoothStep((progress - 0.30f) / 0.25f);
                shoulderAngle = -0.35f + (1.55f + 0.35f) * t;
                elbowAngle = -1.05f + 1.05f * t;
            }
            else
            {
                const float t = smoothStep((progress - 0.55f) / 0.45f);
                shoulderAngle = 1.55f * (1.0f - t);
                elbowAngle = 0.0f;
            }
        }

        void rotateChain(std::vector<Transform>& pose, const std::vector<std::size_t>& chain,
            std::size_t pivot, JPH::QuatArg delta)
        {
            if (pivot >= pose.size())
                return;
            const JPH::Vec3 pivotPosition = pose[pivot].mTranslation;
            for (const std::size_t body : chain)
            {
                if (body >= pose.size())
                    continue;
                pose[body].mRotation = (delta * pose[body].mRotation).Normalized();
                pose[body].mTranslation = pivotPosition + delta * (pose[body].mTranslation - pivotPosition);
            }
        }
    }

    DuelResult runDuelScenario(const SkeletonDescriptor& skeleton, const RagdollProfile& profile,
        const DuelSettings& settings)
    {
        DuelResult result;
        if (!isFinite(settings.mDurationSeconds) || settings.mDurationSeconds <= 0.0f
            || !isFinite(settings.mFixedTimeStep) || settings.mFixedTimeStep <= 0.0f
            || !isFinite(settings.mSeparation) || settings.mSeparation <= 0.0f)
        {
            result.mErrors.push_back("Duel settings require a positive duration, fixed step and separation.");
            return result;
        }

        std::array<Duelist, 2> duelists;
        const std::array<const DuelActorSettings*, 2> actorSettings{ &settings.mLeft, &settings.mRight };
        for (int actor = 0; actor < 2; ++actor)
        {
            if (actorSettings[actor]->mScriptedWoundSeverity
                && (!isFinite(*actorSettings[actor]->mScriptedWoundSeverity)
                    || *actorSettings[actor]->mScriptedWoundSeverity < 0.0f
                    || *actorSettings[actor]->mScriptedWoundSeverity > 1.0f))
            {
                result.mErrors.push_back("Duel scripted wound severity must be inside [0, 1].");
                return result;
            }
            Duelist& duelist = duelists[actor];
            duelist.mSkeleton = skeleton;
            duelist.mProfile = profile;
            duelist.mOptions.mTotalMass = settings.mTotalMass;
            duelist.mOptions.mUseContinuousCollision = true;
            duelist.mOptions.mObjectLayer = actor == 0 ? sLeftLayer : sRightLayer;
            duelist.mRootPosition = JPH::RVec3(0.0, actor == 0 ? -0.5 * settings.mSeparation
                                                               : 0.5 * settings.mSeparation,
                0.0);
            duelist.mRootRotation
                = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), actor == 0 ? 0.0f : 3.14159265358979323846f);
            duelist.mOptions.mRootPosition = duelist.mRootPosition;
            duelist.mOptions.mRootRotation = duelist.mRootRotation;
            duelist.mHealth = std::max(1.0f, actorSettings[actor]->mHealth); // NOLINT
            if (!appendDuelWeapon(duelist.mSkeleton, duelist.mProfile, duelist.mOptions,
                    actorSettings[actor]->mWeapon, settings.mTotalMass))
            {
                result.mErrors.push_back("Duel weapon could not be appended.");
                return result;
            }
            duelist.mWeaponBody = actorSettings[actor]->mWeapon.mArmed
                ? duelist.mProfile.mBodies.size() - 1
                : std::numeric_limits<std::size_t>::max();
            const std::size_t hand = findBody(duelist.mProfile, duelist.mProfile.mLandmarks.mRightHand.value_or(""));
            const std::size_t forearm = findBody(duelist.mProfile, "right_forearm");
            const std::size_t upperArm = findBody(duelist.mProfile, "right_upper_arm");
            if (hand == std::numeric_limits<std::size_t>::max()
                || forearm == std::numeric_limits<std::size_t>::max()
                || upperArm == std::numeric_limits<std::size_t>::max())
            {
                result.mErrors.push_back("Duel requires the right hand, forearm and upper arm bodies.");
                return result;
            }
            duelist.mHandBody = hand;
            duelist.mForearmBody = forearm;
            duelist.mUpperArmBody = upperArm;
            for (std::size_t i = 0; i < duelist.mProfile.mBodies.size(); ++i)
            {
                duelist.mBodyNames.push_back(duelist.mProfile.mBodies[i].mName);
                const std::size_t bone = findBone(duelist.mSkeleton, duelist.mProfile.mBodies[i].mBone);
                if (bone == std::numeric_limits<std::size_t>::max())
                {
                    result.mErrors.push_back("Duel body has no matching skeleton bone.");
                    return result;
                }
                duelist.mBindPose.mBodyTransforms.push_back(duelist.mSkeleton.mBones[bone].mModelTransform);
                duelist.mBindPositions.push_back(duelist.mSkeleton.mBones[bone].mModelTransform.mTranslation);
                duelist.mParents.push_back(duelist.mSkeleton.mBones[bone].mParent);
                if (duelist.mProfile.mBodies[i].mConstraint)
                    duelist.mConstraintBodies.push_back(i);
            }
            duelist.mPelvisBody = findBody(duelist.mProfile, duelist.mProfile.mLandmarks.mPelvis);
            duelist.mTorsoBody = duelist.mProfile.mLandmarks.mSpine.empty()
                ? duelist.mPelvisBody
                : findBody(duelist.mProfile, duelist.mProfile.mLandmarks.mSpine.back());
            duelist.mFeet.mLeft = findBody(duelist.mProfile, "left_foot");
            duelist.mFeet.mRight = findBody(duelist.mProfile, "right_foot");
            if (duelist.mPelvisBody == std::numeric_limits<std::size_t>::max()
                || duelist.mTorsoBody == std::numeric_limits<std::size_t>::max()
                || duelist.mFeet.mLeft == std::numeric_limits<std::size_t>::max()
                || duelist.mFeet.mRight == std::numeric_limits<std::size_t>::max())
            {
                result.mErrors.push_back("Duel requires pelvis, torso and both feet bodies.");
                return result;
            }
            const JPH::Vec3 pelvisUp
                = duelist.mBindPose.mBodyTransforms[duelist.mPelvisBody].mRotation * JPH::Vec3::sAxisZ();
            const JPH::Vec3 torsoUp
                = duelist.mBindPose.mBodyTransforms[duelist.mTorsoBody].mRotation * JPH::Vec3::sAxisZ();
            duelist.mPelvisLocalUp
                = duelist.mBindPose.mBodyTransforms[duelist.mPelvisBody].mRotation.Conjugated() * pelvisUp;
            duelist.mTorsoLocalUp
                = duelist.mBindPose.mBodyTransforms[duelist.mTorsoBody].mRotation.Conjugated() * torsoUp;
            const float footHeight = std::min(
                duelist.mBindPose.mBodyTransforms[duelist.mFeet.mLeft].mTranslation.GetZ(),
                duelist.mBindPose.mBodyTransforms[duelist.mFeet.mRight].mTranslation.GetZ());
            duelist.mTargetPelvisHeight
                = duelist.mBindPose.mBodyTransforms[duelist.mPelvisBody].mTranslation.GetZ() - footHeight;
            // The root body is the pelvis: aiming the kinematic root at
            // standing height keeps the rig above the floor. Creation must
            // keep the model origin on the floor -- the builder adds the
            // bind-pose height itself, so passing the pelvis height here
            // spawns the whole rig one body-height too high.
            duelist.mRootPosition = JPH::RVec3(duelist.mRootPosition.GetX(),
                duelist.mRootPosition.GetY(), duelist.mTargetPelvisHeight);
            duelist.mOptions.mRootPosition = JPH::RVec3(duelist.mRootPosition.GetX(),
                duelist.mRootPosition.GetY(), 0.0);
        }

        DuelBroadPhaseLayers broadPhaseLayers;
        DuelObjectVsBroadPhase objectVsBroadPhase;
        DuelObjectPairs objectPairs;
        JPH::PhysicsSystem physics;
        physics.Init(256, 0, 1024, 1024, broadPhaseLayers, objectVsBroadPhase, objectPairs);
        // Jolt defaults to Y-up gravity; this world stands on the XY plane,
        // so an unset gravity drags every body sideways along -Y.
        physics.SetGravity(JPH::Vec3(0.0f, 0.0f, -UnitModel::GravityUnitsPerSecondSquared));
        {
            JPH::PhysicsSettings solver = physics.GetPhysicsSettings();
            solver.mNumVelocitySteps = 12;
            solver.mNumPositionSteps = 4;
            physics.SetPhysicsSettings(solver);
        }
        JPH::TempAllocatorImpl allocator(8 * 1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);

        DuelContactListener listener;
        physics.SetContactListener(&listener);

        JPH::BodyInterface& bodies = physics.GetBodyInterface();
        const JPH::BoxShapeSettings groundShape(JPH::Vec3(2000.0f, 2000.0f, 10.0f));
        JPH::BodyCreationSettings groundSettings(
            groundShape.Create().Get(), JPH::RVec3(0.0, 0.0, -10.0), JPH::Quat::sIdentity(),
            JPH::EMotionType::Static, sGroundLayer);
        const JPH::BodyID ground = bodies.CreateAndAddBody(groundSettings, JPH::EActivation::DontActivate);
        if (ground.IsInvalid())
        {
            result.mErrors.push_back("Jolt could not create the duel ground.");
            return result;
        }

        for (int actor = 0; actor < 2; ++actor)
        {
            Duelist& duelist = duelists[actor];
            PassiveRagdollInitialState initialState;
            PassiveRagdollCreateResult created = createPassiveRagdoll(physics, duelist.mSkeleton,
                duelist.mProfile, duelist.mOptions, initialState,
                static_cast<JPH::CollisionGroup::GroupID>(actor), 0);
            if (!created.isValid())
            {
                result.mErrors.push_back("Duel ragdoll creation failed.");
                bodies.RemoveBody(ground);
                bodies.DestroyBody(ground);
                return result;
            }
            duelist.mRagdoll = std::move(created.mRagdoll);
            if (!duelist.mRagdoll->addToPhysicsSystem())
            {
                result.mErrors.push_back("Duel ragdoll could not enter the physics system.");
                duelist.mRagdoll.reset();
                bodies.RemoveBody(ground);
                bodies.DestroyBody(ground);
                return result;
            }
            if (!settings.mPhysicalBalance && !duelist.mRagdoll->setRootKinematic(true))
            {
                result.mErrors.push_back("Duel root could not become kinematic.");
                return result;
            }
            if (actorSettings[actor]->mWeapon.mArmed && duelist.mWeaponBody < duelist.mBodyNames.size())
            {
                bodies.SetObjectLayer(duelist.mRagdoll->getBodyId(duelist.mWeaponBody),
                    actor == 0 ? sWeaponLeftLayer : sWeaponRightLayer);
            }
        }

        {
            DuelContactListener::Registry registry;
            for (int actor = 0; actor < 2; ++actor)
            {
                for (std::size_t i = 0; i < duelists[actor].mRagdoll->getBodyCount(); ++i)
                    registry.mBody.emplace(duelists[actor].mRagdoll->getBodyId(i).GetIndexAndSequenceNumber(),
                        std::make_pair(actor, i));
                registry.mWeapon[actor] = duelists[actor].mWeaponBody;
                registry.mHand[actor] = duelists[actor].mHandBody;
            }
            listener.setRegistry(std::move(registry));
        }

        std::array<std::int64_t, 2> lastHitSwing{ -1, -1 };
        result.mWeaponNames[0] = duelists[0].mWeaponBody < duelists[0].mBodyNames.size()
            ? duelists[0].mBodyNames[duelists[0].mWeaponBody] : std::string();
        result.mWeaponNames[1] = duelists[1].mWeaponBody < duelists[1].mBodyNames.size()
            ? duelists[1].mBodyNames[duelists[1].mWeaponBody] : std::string();
        result.mBodyCount[0] = duelists[0].mBodyNames.size();
        result.mBodyCount[1] = duelists[1].mBodyNames.size();
        result.mBodyNames[0] = duelists[0].mBodyNames;
        result.mBodyNames[1] = duelists[1].mBodyNames;
        result.mBodyParents[0] = duelists[0].mParents;
        result.mBodyParents[1] = duelists[1].mParents;

        const float dt = settings.mFixedTimeStep;
        const std::uint64_t steps = static_cast<std::uint64_t>(settings.mDurationSeconds / dt);
        result.mStepCount = steps;
        result.mFrames.reserve(steps);

        ReactionPipelineProfile reactionProfile;
        reactionProfile.mEnabled = settings.mReactionsEnabled;
        reactionProfile.mBehavior = settings.mBehavior;

        for (std::uint64_t step = 0; step < steps; ++step)
        {
            const float time = static_cast<float>(step) * dt;
            DuelFrame frame;
            frame.mTime = time;
            frame.mStep = step;

            std::array<std::int64_t, 2> swingIds{ -1, -1 };
            std::array<bool, 2> swinging{ false, false };
            std::array<float, 2> swingProgress{ 0.0f, 0.0f };

            for (int actor = 0; actor < 2; ++actor)
            {
                Duelist& duelist = duelists[actor];
                const DuelActorSettings& actorSettingsValue = *actorSettings[actor];
                RagdollPose drivePose = duelist.mBindPose;
                if (duelist.mAlive)
                {
                    const float first = std::max(0.0f, actorSettingsValue.mFirstAttackSeconds);
                    const float interval = std::max(0.1f, actorSettingsValue.mSwingIntervalSeconds);
                    const float duration = std::clamp(actorSettingsValue.mSwingDurationSeconds,
                        0.05f, interval);
                    const std::uint64_t firstStep = static_cast<std::uint64_t>(first / dt);
                    if (actorSettingsValue.mAttacksEnabled && step >= firstStep)
                    {
                        const std::uint64_t since = step - firstStep;
                        const auto periodSteps = static_cast<std::uint64_t>(std::max(1.0f, interval / dt));
                        const auto swingSteps = static_cast<std::uint64_t>(std::max(1.0f, duration / dt));
                        const std::uint64_t within = since % periodSteps;
                        if (within < swingSteps)
                        {
                            swinging[actor] = true;
                            swingProgress[actor] = static_cast<float>(within) / static_cast<float>(swingSteps);
                            swingIds[actor] = static_cast<std::int64_t>(since / periodSteps);
                        }
                    }
                    if (swinging[actor])
                    {
                        float shoulderAngle = 0.0f;
                        float elbowAngle = 0.0f;
                        swingAngles(swingProgress[actor], shoulderAngle, elbowAngle);
                        const std::vector<std::size_t> shoulderChain{ duelist.mUpperArmBody,
                            duelist.mForearmBody, duelist.mHandBody, duelist.mWeaponBody };
                        rotateChain(drivePose.mBodyTransforms, shoulderChain, duelist.mUpperArmBody,
                            JPH::Quat::sRotation(JPH::Vec3::sAxisX(), shoulderAngle));
                        const std::vector<std::size_t> elbowChain{ duelist.mForearmBody,
                            duelist.mHandBody, duelist.mWeaponBody };
                        rotateChain(drivePose.mBodyTransforms, elbowChain, duelist.mForearmBody,
                            JPH::Quat::sRotation(JPH::Vec3::sAxisX(), elbowAngle));
                    }
                    const bool reactionActive = duelist.mReactionOutputs.mBehavior != BehaviorId::None
                        || duelist.mReactionOutputs.mCradleLeft || duelist.mReactionOutputs.mCradleRight
                        || duelist.mReactionOutputs.mLimpLeft || duelist.mReactionOutputs.mLimpRight;
                    if (settings.mReactionsEnabled && reactionActive)
                    {
                        ReactionPoseTargets targets;
                        targets.mSpine = findBody(duelist.mProfile, "spine");
                        targets.mNeck = findBody(duelist.mProfile, "neck");
                        targets.mLeftUpperArm = findBody(duelist.mProfile, "left_upper_arm");
                        targets.mRightUpperArm = duelist.mUpperArmBody;
                        applyReactionPose(drivePose.mBodyTransforms, duelist.mReactionOutputs,
                            targets, drivePose.mBodyTransforms);
                    }
                    if (settings.mPhysicalBalance)
                    {
                        // Reference standing: measured contacts feed the shared
                        // support estimate and the joint load plan; the root
                        // stays dynamic so balance is the physics' decision.
                        RagdollTelemetry telemetry;
                        if (!duelist.mRagdoll->captureTelemetry(telemetry))
                        {
                            result.mErrors.push_back("Duel telemetry capture failed.");
                            return result;
                        }
                        duelist.mSupportEstimate = estimateFootSupport(telemetry.mCenterOfMass,
                            duelist.mContacts, duelist.mFeet, duelist.mProfile.mActiveController);
                        if (duelist.mSupportEstimate && duelist.mSupportEstimate->hasSupport())
                        {
                            duelist.mSupport = calculateStandingJointSupport(telemetry,
                                duelist.mContacts, duelist.mFeet,
                                UnitModel::GravityUnitsPerSecondSquared, 0.0f,
                                duelist.mProfile.mActiveController);
                            // Upright control supplies the root channel the
                            // standing load plan omits: measured pelvis height
                            // and COM are corrected with bounded pelvis and
                            // spine forces, so a dynamic root settles instead
                            // of drifting.
                            JPH::Vec3 centerOfMassVelocity = JPH::Vec3::sZero();
                            for (const BodyTelemetry& body : telemetry.mBodies)
                                centerOfMassVelocity += body.mMotion.mLinearVelocity * body.mMass;
                            if (telemetry.mTotalMass > 0.0f)
                                centerOfMassVelocity /= telemetry.mTotalMass;
                            const float measuredPelvisHeight = std::max(0.0f,
                                static_cast<float>(telemetry.mBodies[duelist.mPelvisBody].mCenterOfMass.GetZ()
                                    - duelist.mSupportEstimate->mGroundCenter.GetZ()));
                            // Latch the measured settled height: an authored
                            // target taller than the rig makes the loop chase
                            // a height the joints cannot hold, and the excess
                            // force pumps the body upward.
                            if (!duelist.mPelvisHeightLatched && measuredPelvisHeight > 1.0f)
                            {
                                duelist.mTargetPelvisHeight = measuredPelvisHeight;
                                duelist.mPelvisHeightLatched = true;
                            }
                            UprightControllerInput upright;
                            upright.mTargetPelvisHeight = duelist.mTargetPelvisHeight;
                            upright.mPelvisHeight = measuredPelvisHeight;
                            upright.mPelvisVerticalVelocity
                                = telemetry.mBodies[duelist.mPelvisBody].mMotion.mLinearVelocity
                                    .Dot(JPH::Vec3::sAxisZ());
                            upright.mCenterOfMassOffset = JPH::Vec3(
                                telemetry.mCenterOfMass - duelist.mSupportEstimate->mCenter);
                            upright.mCenterOfMassVelocity = centerOfMassVelocity;
                            upright.mPelvisUp = telemetry.mBodies[duelist.mPelvisBody].mRotation
                                * duelist.mPelvisLocalUp;
                            upright.mPelvisAngularVelocity
                                = telemetry.mBodies[duelist.mPelvisBody].mMotion.mAngularVelocity;
                            upright.mTorsoUp = telemetry.mBodies[duelist.mTorsoBody].mRotation
                                * duelist.mTorsoLocalUp;
                            upright.mTorsoAngularVelocity
                                = telemetry.mBodies[duelist.mTorsoBody].mMotion.mAngularVelocity;
                            upright.mTotalMass = telemetry.mTotalMass;
                            upright.mSupportFriction = duelist.mSupportEstimate->mFriction.value_or(0.0f);
                            upright.mGravityMagnitude = UnitModel::GravityUnitsPerSecondSquared;
                            upright.mCenterOfMassPosition = telemetry.mCenterOfMass;
                            upright.mCenterOfMassHeight = duelist.mSupportEstimate->mCenterOfMassHeight;
                            duelist.mUpright = calculateUprightControl(
                                upright, duelist.mProfile.mActiveController);
                            if (duelist.mUpright)
                            {
                                // Authored force ceilings are sized for the
                                // live actuator stack; a catching fall would
                                // launch the body. Budget the duel assistance
                                // against actual body weight instead.
                                const float weight = telemetry.mTotalMass
                                    * UnitModel::GravityUnitsPerSecondSquared;
                                UprightControllerOutput& control = *duelist.mUpright;
                                const float stabilization = std::clamp(
                                    duelist.mReactionOutputs.mStabilizationStrength, 0.0f, 1.0f);
                                control.mPelvisForce *= stabilization;
                                control.mPelvisTorque *= stabilization;
                                control.mSpineTorque *= stabilization;
                                control.mPelvisForce.SetZ(std::clamp(control.mPelvisForce.GetZ(),
                                    -0.5f * weight, 1.5f * weight));
                                JPH::Vec3 horizontalForce(control.mPelvisForce.GetX(),
                                    control.mPelvisForce.GetY(), 0.0f);
                                const float maximumHorizontal = 0.6f * weight;
                                if (horizontalForce.Length() > maximumHorizontal)
                                {
                                    horizontalForce = horizontalForce.Normalized() * maximumHorizontal;
                                    control.mPelvisForce.SetX(horizontalForce.GetX());
                                    control.mPelvisForce.SetY(horizontalForce.GetY());
                                }
                            }
                            if (RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_BALANCE_TRACE")
                                && (step < 24 || (step % 60) == 0))
                            {
                                const JPH::Vec3 force = duelist.mUpright
                                    ? duelist.mUpright->mPelvisForce : JPH::Vec3::sZero();
                                std::fprintf(stderr,
                                    "DUEL_BALANCE actor=%d step=%llu z=%.1f height=%.1f up=%.3f force=(%.0f,%.0f,%.0f) sat=%d/%d/%d\n",
                                    actor, static_cast<unsigned long long>(step),
                                    telemetry.mBodies[duelist.mPelvisBody].mCenterOfMass.GetZ(),
                                    measuredPelvisHeight,
                                    upright.mPelvisUp.Dot(JPH::Vec3::sAxisZ()),
                                    force.GetX(), force.GetY(), force.GetZ(),
                                    duelist.mUpright ? duelist.mUpright->mPelvisVerticalForceSaturated : 0,
                                    duelist.mUpright ? duelist.mUpright->mPelvisHorizontalForceSaturated : 0,
                                    duelist.mUpright ? duelist.mUpright->mPelvisForceSaturated : 0);
                            }
                        }
                        else
                        {
                            duelist.mSupport.reset();
                            duelist.mUpright.reset();
                        }
                    }
                    if (!duelist.mRagdoll->driveToPoseUsingMotors(drivePose))
                    {
                        result.mErrors.push_back("Duel pose drive failed.");
                        return result;
                    }
                    if (RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_ARM_TRACE")
                        && (step % 30) == 0 && actor == 0 && swinging[0])
                    {
                        MotorTargetTelemetry telemetry;
                        if (duelist.mRagdoll->captureMotorTargetTelemetry(duelist.mUpperArmBody, telemetry))
                        {
                            std::fprintf(stderr,
                                "DUEL_ARM step=%llu p=%.2f error=%.3f clamp=%.3f\n",
                                static_cast<unsigned long long>(step), swingProgress[0],
                                telemetry.mAngularError, telemetry.mTargetClampDelta);
                        }
                    }
                    if (duelist.mSupport)
                    {
                        for (std::size_t body = 1; body < duelist.mProfile.mBodies.size(); ++body)
                        {
                            if (!duelist.mProfile.mBodies[body].mConstraint)
                                continue;
                            if (body >= duelist.mSupport->mJointTorques.size())
                                return result.mErrors.emplace_back("Duel support torque mismatch"), result;
                            if (!duelist.mRagdoll->driveMotorWithLoadTorque(body,
                                    duelist.mSupport->mJointTorques[body]
                                        * std::clamp(duelist.mReactionOutputs.mStabilizationStrength, 0.0f, 1.0f),
                                    dt))
                            {
                                result.mErrors.push_back("Duel support load motor rejected.");
                                return result;
                            }
                        }
                    }
                    if (!settings.mPhysicalBalance)
                    {
                        float lunge = 0.0f;
                        if (swinging[actor])
                        {
                            const float p = swingProgress[actor];
                            if (p >= 0.30f && p < 0.60f)
                                lunge = 14.0f * smoothStep((p - 0.30f) / 0.30f);
                            else if (p >= 0.60f && p < 0.90f)
                                lunge = 14.0f * (1.0f - smoothStep((p - 0.60f) / 0.30f));
                        }
                        const JPH::Vec3 forward
                            = duelist.mRootRotation * JPH::Vec3(0.0f, lunge, 0.0f);
                        duelist.mRagdoll->moveRootKinematic(duelist.mRootPosition + JPH::RVec3(forward),
                            duelist.mRootRotation, dt);
                    }
                    else if (duelist.mUpright)
                    {
                        // Support feet receive the upright reaction; the root
                        // stays dynamic and physics owns the balance outcome.
                        std::vector<std::size_t> reactionBodies;
                        for (const GroundContact& contact : duelist.mContacts)
                        {
                            if (contact.mNormal.GetZ() <= 0.0f)
                                continue;
                            if (std::find(reactionBodies.begin(), reactionBodies.end(),
                                    contact.mBodyIndex) == reactionBodies.end())
                                reactionBodies.push_back(contact.mBodyIndex);
                        }
                        if (!reactionBodies.empty()
                            && !duelist.mRagdoll->applyUprightControl(*duelist.mUpright,
                                duelist.mPelvisBody, duelist.mTorsoBody, reactionBodies,
                                reactionBodies, duelist.mProfile.mActiveController,
                                reactionBodies.front(), true,
                                reactionBodies.size() > 1
                                    ? std::optional<std::size_t>(reactionBodies[1]) : std::nullopt))
                        {
                            result.mErrors.push_back("Duel upright control rejected.");
                            return result;
                        }
                    }
                }
                else if (!duelist.mDeathHandled)
                {
                    duelist.mDeathHandled = true;
                    duelist.mDeathStep = step;
                    for (std::size_t body = 0; body < duelist.mBodyNames.size(); ++body)
                    {
                        const bool droppedWeapon
                            = body == duelist.mWeaponBody && duelist.mWeaponBody < duelist.mBodyNames.size();
                        bodies.SetObjectLayer(duelist.mRagdoll->getBodyId(body),
                            droppedWeapon ? sInertLayer : sCorpseLayer);
                    }
                    if (!settings.mPhysicalBalance)
                    {
                        // Scripted root: the corpse eases into a lying pose.
                        // A released dynamic root keeps sliding on leftover
                        // contacts, so the dead stay on the scripted root.
                        duelist.mDeathPosition = bodies.GetPosition(duelist.mRagdoll->getBodyId(0));
                        duelist.mDeathRotation = bodies.GetRotation(duelist.mRagdoll->getBodyId(0));
                        // Interim: the corpse holds its pose where it died.
                        // The joint drive targets relative angles through the
                        // standing reference frame, so a rigid lying re-tip
                        // does not carry the limbs; a real lying pose needs an
                        // authored per-joint collapse (tracked as a follow-up).
                        duelist.mCollapseRotation = duelist.mDeathRotation;
                    }
                    if (!settings.mPhysicalBalance)
                    {
                        // Hold the bind pose so the corpse holds its ground
                        // instead of letting limp limbs hang through it.
                        duelist.mRagdoll->driveToPoseUsingMotors(duelist.mBindPose);
                    }
                    else
                    {
                        RagdollPose current;
                        const bool captured = duelist.mRagdoll->captureModelPose(current);
                        bool motorsDisabled = false;
                        if (captured)
                            motorsDisabled
                                = duelist.mRagdoll->driveToPoseUsingMotors(current, duelist.mConstraintBodies);
                        // Zero every joint motor torque unconditionally: the
                        // corpse must not keep any scripted target, and a
                        // rejected disabled-list drive must not leave one
                        // behind.
                        for (std::size_t body = 1; body < duelist.mProfile.mBodies.size(); ++body)
                        {
                            if (duelist.mProfile.mBodies[body].mConstraint)
                                duelist.mRagdoll->setMotorMaximumTorque(body, 0.0f);
                        }
                        if (RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_DEATH_TRACE"))
                            std::fprintf(stderr,
                                "DUEL_DEATH_SETUP actor=%d captured=%d disabled=%d rootLayer=%u weaponLayer=%u\n",
                                actor, captured ? 1 : 0, motorsDisabled ? 1 : 0,
                                static_cast<unsigned>(bodies.GetObjectLayer(duelist.mRagdoll->getBodyId(0))),
                                static_cast<unsigned>(bodies.GetObjectLayer(
                                    duelist.mRagdoll->getBodyId(duelist.mWeaponBody))));
                    }
                }
                else if (!settings.mPhysicalBalance)
                {
                    // Jolt kinematic bodies keep their velocity: steer the
                    // corpse to the collapse pose every step until it rests.
                    const JPH::RVec3 target(
                        duelist.mDeathPosition.GetX(),
                        duelist.mDeathPosition.GetY(),
                        duelist.mDeathPosition.GetZ());
                    duelist.mRagdoll->moveRootKinematic(
                        target, duelist.mCollapseRotation, std::max(0.001f, dt));
                    duelist.mRagdoll->driveToPoseUsingMotors(duelist.mBindPose);
                }
            }

            listener.beginStep();
            const auto updateError = physics.Update(dt, 1, &allocator, &jobs);
            if (updateError != JPH::EPhysicsUpdateError::None)
            {
                result.mErrors.push_back("Jolt duel update failed.");
                return result;
            }

            if (settings.mPhysicalBalance)
            {
                for (int actor = 0; actor < 2; ++actor)
                    duelists[actor].mContacts = resolveGroundContacts(listener.ground(actor), physics);
            }

            // Deterministic strike processing: sort contacts before deducting.
            std::vector<StrikeContact> contacts = listener.contacts();
            std::sort(contacts.begin(), contacts.end(), [](const StrikeContact& a, const StrikeContact& b) {
                return std::make_tuple(a.mAttacker, a.mDefenderBody, a.mAttackerBody,
                           static_cast<double>(a.mPoint.GetX()), static_cast<double>(a.mPoint.GetY()),
                           static_cast<double>(a.mPoint.GetZ()))
                    < std::make_tuple(b.mAttacker, b.mDefenderBody, b.mAttackerBody,
                        static_cast<double>(b.mPoint.GetX()), static_cast<double>(b.mPoint.GetY()),
                        static_cast<double>(b.mPoint.GetZ()));
            });

            for (int actor = 0; actor < 2; ++actor)
            {
                duelists[actor].mHitThisStep = false;
                duelists[actor].mLatestHealthLoss = 0.0f;
            }

            for (const StrikeContact& contact : contacts)
            {
                if (contact.mApproach < settings.mMinimumApproachSpeed)
                    continue;
                Duelist& defender = duelists[1 - contact.mAttacker];
                if (!defender.mAlive)
                    continue;
                const std::int64_t swingId = swingIds[contact.mAttacker];
                if (swingId >= 0 && swingId == lastHitSwing[contact.mAttacker]
                    && lastHitSwing[contact.mAttacker] >= 0)
                    continue;
                const DuelActorSettings& attackerSettings = *actorSettings[contact.mAttacker];
                const float damage = contact.mWeapon
                    ? physicalImpactWeaponDamage(attackerSettings.mWeapon.mMinimumDamage,
                        attackerSettings.mWeapon.mMaximumDamage, contact.mApproach,
                        contact.mEffectiveMass)
                    : physicalImpactWeaponDamage(attackerSettings.mFistMinimumDamage,
                        attackerSettings.mFistMaximumDamage, contact.mApproach, contact.mEffectiveMass);
                if (!(damage > 0.0f))
                    continue;
                if (swingId >= 0)
                    lastHitSwing[contact.mAttacker] = swingId;

                defender.mHealth -= damage;
                if (contact.mAttacker == 0)
                {
                    ++result.mLeftHits;
                    result.mLeftDamageDealt += damage;
                }
                else
                {
                    ++result.mRightHits;
                    result.mRightDamageDealt += damage;
                }
                if (defender.mHealth <= 0.0f)
                {
                    defender.mHealth = 0.0f;
                    defender.mAlive = false;
                }
                defender.mHitThisStep = true;
                defender.mLatestHealthLoss += damage;
                if (contact.mDefenderBody < defender.mBodyNames.size())
                    defender.mLatestRegion = regionForBodyName(defender.mBodyNames[contact.mDefenderBody]);
                defender.mLatestImpulse = contact.mEffectiveMass * contact.mApproach;

                if (settings.mReactionsEnabled)
                {
                    const int defenderIndex = 1 - contact.mAttacker;
                    ReactionContractInput input;
                    input.mFramePhysicsStep = step;
                    input.mFixedStepDuration = dt;
                    input.mContact.mValid = true;
                    input.mContact.mContactId = step * 4 + static_cast<std::uint64_t>(contact.mAttacker) * 2 + 1;
                    input.mContact.mAttackerActorIdentity = static_cast<std::uint64_t>(contact.mAttacker + 1);
                    input.mContact.mTargetActorIdentity = static_cast<std::uint64_t>(2 - contact.mAttacker);
                    input.mContact.mTargetRegistryGeneration = 1;
                    input.mContact.mSwingIdentity = static_cast<std::uint64_t>(std::max<std::int64_t>(0, swingId));
                    input.mContact.mPhysicsStep = step;
                    input.mContact.mRegion = defender.mLatestRegion;
                    input.mContact.mTargetBodyIndex = static_cast<std::uint32_t>(contact.mDefenderBody);
                    input.mImpulse.mProvenance = ImpulseProvenance::ContactEstimate;
                    input.mImpulse.mMagnitude = defender.mLatestImpulse;
                    input.mImpulse.mAlreadyApplied = true;
                    input.mReceipt = ReactionReceipt{ input.mContact.mContactId, damage, 0.0f };
                    input.mWoundSeverity = actorSettings[defenderIndex]->mScriptedWoundSeverity.value_or(0.0f);
                    input.mRegionCapable = true;
                    input.mMode = ReactionBodyMode::Standing;
                    if (settings.mPhysicalBalance && defender.mSupportEstimate)
                    {
                        input.mSupport.mLeftFootSupported = defender.mSupportEstimate->mLeftFootSupported;
                        input.mSupport.mRightFootSupported = defender.mSupportEstimate->mRightFootSupported;
                        input.mSupport.mSupportEstablished = defender.mSupportEstimate->hasSupport();
                        input.mSupport.mBothFeetSupported
                            = defender.mSupportEstimate->mLeftFootSupported
                            && defender.mSupportEstimate->mRightFootSupported;
                    }
                    else
                    {
                        input.mSupport.mSupportEstablished = true;
                        input.mSupport.mBothFeetSupported = true;
                    }
                    input.mOwned.mRightArm = swinging[defenderIndex];
                    input.mOwned.mWeaponAim = actorSettings[defenderIndex]->mWeapon.mArmed;
                    defender.mReactionOutputs = stepReactionPipeline(defender.mReaction, input, reactionProfile);
                }
                else
                {
                    defender.mReactionOutputs.mBehavior = BehaviorId::None;
                }
            }

            for (int actor = 0; actor < 2; ++actor)
            {
                Duelist& duelist = duelists[actor];
                if (RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_DEATH_TRACE") && !duelist.mAlive
                    && (step % 30) == 0)
                {
                    const JPH::BodyID rootBody = duelist.mRagdoll->getBodyId(0);
                    const JPH::Vec3 velocity = bodies.GetLinearVelocity(rootBody);
                    std::fprintf(stderr,
                        "DUEL_DEATH actor=%d step=%llu kinematic=%d velocity=(%.1f,%.1f,%.1f)\n",
                        actor, static_cast<unsigned long long>(step),
                        duelist.mRagdoll->isRootKinematic() ? 1 : 0,
                        velocity.GetX(), velocity.GetY(), velocity.GetZ());
                }
                DuelActorFrame actorFrame;
                actorFrame.mAlive = duelist.mAlive;
                actorFrame.mHealth = duelist.mHealth;
                actorFrame.mBehavior = duelist.mReactionOutputs.mBehavior;
                actorFrame.mPhase = duelist.mReactionOutputs.mPhase;
                actorFrame.mAuthority = duelist.mReactionOutputs.mPoseCorrectionAuthority;
                actorFrame.mStability = duelist.mReactionOutputs.mStabilizationStrength;
                actorFrame.mCoverFace = duelist.mReactionOutputs.mCoverFace;
                actorFrame.mCradleLeft = duelist.mReactionOutputs.mCradleLeft;
                actorFrame.mCradleRight = duelist.mReactionOutputs.mCradleRight;
                actorFrame.mLimpLeft = duelist.mReactionOutputs.mLimpLeft;
                actorFrame.mLimpRight = duelist.mReactionOutputs.mLimpRight;
                actorFrame.mSwinging = swinging[actor];
                actorFrame.mSwingProgress = swingProgress[actor];
                actorFrame.mHitThisStep = duelist.mHitThisStep;
                actorFrame.mLatestHealthLoss = duelist.mLatestHealthLoss;
                actorFrame.mLatestImpulse = duelist.mLatestImpulse;
                actorFrame.mLatestRegion = duelist.mLatestRegion;

                RagdollPose pose;
                if (duelist.mRagdoll->captureModelPose(pose))
                {
                    for (std::size_t body = 0; body < duelist.mBodyNames.size(); ++body)
                    {
                        const auto position = duelist.mRagdoll->getWorldBodyPosition(pose, body);
                        const auto rotation = duelist.mRagdoll->getWorldBodyRotation(pose, body);
                        if (!position || !rotation)
                        {
                            result.mErrors.push_back("Duel pose capture failed.");
                            return result;
                        }
                        actorFrame.mBodyPositions.push_back(JPH::Vec3(*position));
                        actorFrame.mBodyRotations.push_back(*rotation);
                    }
                    const auto weaponPosition = duelist.mWeaponBody < duelist.mBodyNames.size()
                        ? duelist.mRagdoll->getWorldBodyPosition(pose, duelist.mWeaponBody)
                        : std::optional<JPH::RVec3>{};
                    const auto weaponRotation
                        = duelist.mRagdoll->getWorldBodyRotation(pose, duelist.mWeaponBody);
                    if (weaponPosition && weaponRotation)
                    {
                        actorFrame.mWeaponGrip = JPH::Vec3(*weaponPosition);
                        const JPH::Vec3 direction = *weaponRotation * JPH::Vec3::sAxisY();
                        actorFrame.mWeaponTip = JPH::Vec3(*weaponPosition)
                            + direction * (actorSettings[actor]->mWeapon.mLength * 0.5f);
                    }
                }
                if (actor == 0)
                    frame.mLeft = std::move(actorFrame);
                else
                    frame.mRight = std::move(actorFrame);
            }

            if (step == 0 || step == steps - 1 || swinging[0] || swinging[1] || duelists[0].mHitThisStep
                || duelists[1].mHitThisStep)
            {
                // Keep every step for determinism; the viewer downsamples.
            }
            result.mFrames.push_back(std::move(frame));
        }

        result.mLeftDead = !duelists[0].mAlive;
        result.mRightDead = !duelists[1].mAlive;
        result.mSimulatedSeconds = static_cast<float>(result.mFrames.size()) * dt;

        for (int actor = 0; actor < 2; ++actor)
        {
            if (duelists[actor].mRagdoll)
                duelists[actor].mRagdoll->removeFromPhysicsSystem();
            duelists[actor].mRagdoll.reset();
        }
        physics.SetContactListener(nullptr);
        bodies.RemoveBody(ground);
        bodies.DestroyBody(ground);
        return result;
    }
}
