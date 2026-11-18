#include <ragdollcore/stridesim.hpp>
#include <ragdollcore/tuning.hpp>
#include <limits>

#include <ragdollcore/activebody.hpp>
#include <ragdollcore/contact.hpp>
#include <ragdollcore/gaitcontroller.hpp>
#include <ragdollcore/runtimeprofile.hpp>
#include <ragdollcore/scenario.hpp>
#include <ragdollcore/roottarget.hpp>
#include <ragdollcore/uprightholder.hpp>

// clang-format off
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <sstream>
#include <unordered_map>

namespace RagdollCore
{
    namespace
    {
        constexpr JPH::ObjectLayer sGround = 0;
        constexpr JPH::ObjectLayer sBody = 1;
        constexpr JPH::ObjectLayer sAnchor = 2; // the soft root's anchor touches nothing
        constexpr float sStep = 1.0f / 60.0f;

        class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterface
        {
        public:
            JPH::uint GetNumBroadPhaseLayers() const override { return 2; }
            JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override
            {
                return JPH::BroadPhaseLayer(layer == sGround ? 0 : 1);
            }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
            const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override { return "stridesim"; }
#endif
        };

        class ObjectVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer broadPhase) const override
            {
                if (layer == sAnchor)
                    return false;
                if (layer == sGround)
                    return broadPhase == JPH::BroadPhaseLayer(1);
                return true;
            }
        };

        class ObjectPairs final : public JPH::ObjectLayerPairFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override
            {
                if (a == sAnchor || b == sAnchor)
                    return false;
                return a == sBody || b == sBody;
            }
        };
    }

    // The ragdoll's ground contacts per step (the crowd scenario's listener,
    // one actor).
    class GroundListener final : public JPH::ContactListener
    {
    public:
        void registerRagdoll(const PassiveRagdoll& ragdoll)
        {
            for (std::size_t i = 0; i < ragdoll.getBodyCount(); ++i)
                mBodies.emplace(ragdoll.getBodyId(i).GetIndexAndSequenceNumber(), i);
        }
        std::vector<GroundContact> take(const JPH::PhysicsSystem& physics)
        {
            std::scoped_lock lock(mMutex);
            std::vector<GroundContact> contacts = resolveGroundContacts(mObservations, physics);
            mObservations.clear();
            return contacts;
        }
        JPH::ValidateResult OnContactValidate(const JPH::Body& a, const JPH::Body& b, JPH::RVec3Arg baseOffset,
            const JPH::CollideShapeResult& result) override
        {
            if (mRejectUnderside && rejectsFloorUnderside(a, b, baseOffset, result))
                return JPH::ValidateResult::RejectContact;
            return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
        }
        void OnContactAdded(const JPH::Body& a, const JPH::Body& b, const JPH::ContactManifold& manifold,
            JPH::ContactSettings& settings) override
        {
            observe(a, b, manifold, settings.mCombinedFriction);
        }
        bool mRejectUnderside = false;
        void OnContactPersisted(const JPH::Body& a, const JPH::Body& b, const JPH::ContactManifold& manifold,
            JPH::ContactSettings& settings) override
        {
            observe(a, b, manifold, settings.mCombinedFriction);
        }

    private:
        void observe(const JPH::Body& a, const JPH::Body& b, const JPH::ContactManifold& manifold, float friction)
        {
            const auto first = mBodies.find(a.GetID().GetIndexAndSequenceNumber());
            const auto second = mBodies.find(b.GetID().GetIndexAndSequenceNumber());
            if ((first != mBodies.end()) == (second != mBodies.end()))
                return;
            std::scoped_lock lock(mMutex);
            const bool firstBody = first != mBodies.end();
            appendGroundContactObservations(
                mObservations, firstBody ? first->second : second->second, a, b, manifold, firstBody, friction);
        }
        std::unordered_map<std::uint32_t, std::size_t> mBodies;
        std::vector<GroundContactObservation> mObservations;
        std::mutex mMutex;
    };

    struct StrideSim::World
    {
        GroundListener mListener;
        std::vector<GroundContact> mContacts;
        BroadPhaseLayers mBroadPhase;
        ObjectVsBroadPhase mObjectVsBroadPhase;
        ObjectPairs mPairs;
        JPH::PhysicsSystem mPhysics;
        JPH::TempAllocatorImpl mAllocator{ 8 * 1024 * 1024 };
        JPH::JobSystemSingleThreaded mJobs{ JPH::cMaxPhysicsJobs };
        JPH::BodyID mGround;
        JPH::BodyID mSlab;
        ~World()
        {
            if (!mSlab.IsInvalid())
            {
                mPhysics.GetBodyInterface().RemoveBody(mSlab);
                mPhysics.GetBodyInterface().DestroyBody(mSlab);
            }
            if (!mGround.IsInvalid())
            {
                mPhysics.GetBodyInterface().RemoveBody(mGround);
                mPhysics.GetBodyInterface().DestroyBody(mGround);
            }
        }
    };

    StrideSim::StrideSim(const Settings& settings)
        : mWorld(std::make_unique<World>())
        , mSettings(settings)
    {
        mProfile = settings.mProfile ? *settings.mProfile : makeScenarioHumanoidProfile();
        float uniformScale = 1.0f;
        if (!settings.mSkeletonPath.empty())
        {
            auto dump = readSkeletonDump(settings.mSkeletonPath);
            if (!dump)
            {
                mError = "the skeleton dump could not be read: " + settings.mSkeletonPath;
                return;
            }
            mSkeleton = std::move(dump->mSkeleton);
            mFrames = std::move(dump->mFrames);
            uniformScale = dump->mUniformScale;
            mSettings.mTotalMass = dump->mTotalMass;
            if (const auto failed = applyAnatomicalFrames(mProfile, mSkeleton,
                    mProfile.mActiveController.mLocomotionRoute == LocomotionRoute::KinematicRoot))
            {
                mError = std::string("the dumped skeleton's ") + std::string(*failed) + " could not be framed";
                return;
            }
        }
        else
            mSkeleton = makeScenarioHumanoidSkeleton();
        const SkeletonDescriptor& sSkeleton = mSkeleton;
        const RagdollProfile& sProfile = mProfile;
        JPH::PhysicsSystem& physics = mWorld->mPhysics;
        physics.Init(256, 0, 1024, 1024, mWorld->mBroadPhase, mWorld->mObjectVsBroadPhase, mWorld->mPairs);
        physics.SetGravity(JPH::Vec3(0.0f, 0.0f, -UnitModel::GravityUnitsPerSecondSquared));
        physics.SetContactListener(&mWorld->mListener);
        mWorld->mListener.mRejectUnderside = settings.mRejectFloorUnderside;
        if (settings.mGameContacts)
            configureScenarioPhysics(physics, 0);
        const bool slab = settings.mFloor == Settings::Floor::HoldSlab;
        JPH::BodyCreationSettings ground(new JPH::BoxShape(JPH::Vec3(8192.0f, 8192.0f, 16.0f)),
            JPH::RVec3(0.0, 0.0, slab ? -31.0 : -16.0), JPH::Quat::sIdentity(), JPH::EMotionType::Static, sGround);
        ground.mFriction = settings.mGroundFriction;
        mWorld->mGround = physics.GetBodyInterface().CreateAndAddBody(ground, JPH::EActivation::DontActivate);
        if (slab)
        {
            // In_prison_ship.NIF's hold floor (ship x, y; the top at -105.3,
            // the underside at -105.9), each section's top and underside.
            constexpr float x0 = -229.9f, x1 = 251.4f;
            constexpr std::array<float, 5> ys{ -313.2f, -187.3f, 412.3f, 1011.9f, 1271.0f };
            const float sx = settings.mSlabStart[0], sy = settings.mSlabStart[1];
            const auto at = [&](float x, float y, float z) { return JPH::Float3(x - sx, y - sy, z); };
            JPH::TriangleList triangles;
            for (std::size_t i = 0; i + 1 < ys.size(); ++i)
            {
                const float a = ys[i], b = ys[i + 1];
                triangles.push_back({ at(x1, a, 0.0f), at(x1, b, 0.0f), at(x0, a, 0.0f) });
                triangles.push_back({ at(x0, b, 0.0f), at(x0, a, 0.0f), at(x1, b, 0.0f) });
                triangles.push_back({ at(x1, b, -0.6f), at(x1, a, -0.6f), at(x0, b, -0.6f) });
                triangles.push_back({ at(x0, a, -0.6f), at(x0, b, -0.6f), at(x1, a, -0.6f) });
            }
            JPH::MeshShapeSettings mesh(triangles);
            const auto shape = mesh.Create();
            if (shape.IsValid())
            {
                JPH::BodyCreationSettings floor(shape.Get(), JPH::RVec3::sZero(), JPH::Quat::sIdentity(),
                    JPH::EMotionType::Static, sGround);
                floor.mFriction = settings.mGroundFriction;
                mWorld->mSlab = physics.GetBodyInterface().CreateAndAddBody(floor, JPH::EActivation::DontActivate);
            }
        }

        mLimp = settings.mLyingRotation.has_value();
        BuildOptions options;
        options.mTotalMass = mSettings.mTotalMass;
        options.mRootPosition = JPH::RVec3(0.0, 0.0, settings.mActorHeight);
        options.mUniformScale = uniformScale;
        options.mFrameSkeleton = mFrames ? &*mFrames : nullptr;
        options.mObjectLayer = sBody;
        if (mLimp)
        {
            options.mRootRotation = *settings.mLyingRotation;
            options.mRootPosition = JPH::RVec3(0.0, 0.0, settings.mLyingHeight);
        }
        PassiveRagdollCreateResult created = createPassiveRagdoll(physics, sSkeleton, sProfile, options, {}, 1, 1);
        if (!created.isValid() || !created.mRagdoll->addToPhysicsSystem())
        {
            mError = "the scenario humanoid could not be created";
            return;
        }
        mRagdoll = std::move(created.mRagdoll);
        mWorld->mListener.registerRagdoll(*mRagdoll);
        // The game's kinematic-root body solves its powered joints with 32
        // position steps (physicssystem.cpp, sKinematicRootPositionSolverSteps).
        if (settings.mGameContacts)
            mRagdoll->setPoweredPositionSolverSteps(32);

        const auto find = [&](const std::string& name) -> std::optional<std::size_t> {
            for (std::size_t i = 0; i < sProfile.mBodies.size(); ++i)
                if (sProfile.mBodies[i].mName == name)
                    return i;
            return std::nullopt;
        };
        const auto parent = [&](std::size_t body) -> std::optional<std::size_t> {
            const auto& name = sProfile.mBodies[body].mParentBody;
            return name ? find(*name) : std::nullopt;
        };
        std::unordered_map<std::string, const SkeletonBone*> bones;
        for (const SkeletonBone& bone : sSkeleton.mBones)
            bones.emplace(bone.mName, &bone);
        for (const BodyProfile& body : sProfile.mBodies)
        {
            mBasePose.mBodyTransforms.push_back(bones.at(body.mBone)->mModelTransform);
            if (body.mConstraint)
                mProfileSprings.emplace_back(body.mConstraint->mMotor.mFrequency, body.mConstraint->mMotor.mDamping);
            else
                mProfileSprings.emplace_back(0.0f, 0.0f);
        }
        const auto pelvis = find(sProfile.mLandmarks.mPelvis);
        const auto head = sProfile.mLandmarks.mHead ? find(*sProfile.mLandmarks.mHead) : std::nullopt;
        const auto leftFoot = sProfile.mLandmarks.mLeftFoot ? find(*sProfile.mLandmarks.mLeftFoot) : std::nullopt;
        const auto rightFoot = sProfile.mLandmarks.mRightFoot ? find(*sProfile.mLandmarks.mRightFoot) : std::nullopt;
        if (!pelvis || !head || !leftFoot || !rightFoot)
        {
            mError = "the profile lacks landmarks";
            return;
        }
        mPelvis = *pelvis;
        mHead = *head;
        const std::array<std::size_t, 2> feet{ *leftFoot, *rightFoot };
        for (std::size_t side = 0; side < 2; ++side)
        {
            const auto knee = parent(feet[side]);
            const auto hip = knee ? parent(*knee) : std::nullopt;
            if (!hip)
            {
                mError = "a leg chain is incomplete";
                return;
            }
            mLegs[side] = FootfallLeg{ *hip, *knee, feet[side], mRagdoll->getHingeLegGeometry(*hip, *knee, feet[side]) };
        }
        for (const std::string& name : sProfile.mLandmarks.mSpine)
            if (const auto index = find(name))
                mSpine.push_back(*index);
        if (!mSpine.empty())
        {
            const std::string& chest = sProfile.mBodies[mSpine.back()].mName;
            const auto neck = sProfile.mBodies[mHead].mParentBody;
            for (std::size_t i = 0; i < sProfile.mBodies.size(); ++i)
            {
                auto up = sProfile.mBodies[i].mParentBody;
                for (int depth = 0; up && depth < 12; ++depth)
                {
                    if (*up == chest)
                    {
                        mChestCarried.emplace_back(i, i == mHead || (neck && sProfile.mBodies[i].mName == *neck));
                        break;
                    }
                    const auto next = find(*up);
                    up = next ? sProfile.mBodies[*next].mParentBody : std::nullopt;
                }
            }
        }
        if (!settings.mClipTrackPath.empty())
        {
            mClipTrack = readClipTrack(settings.mClipTrackPath);
            if (!mClipTrack || mClipTrack->mPoses.front().size() != mBasePose.mBodyTransforms.size())
            {
                mError = "the clip track could not be read or has another body count: " + settings.mClipTrackPath;
                return;
            }
            while (mClipTrackStart + 1 < mClipTrack->mRequestSpeed.size()
                && mClipTrack->mRequestSpeed[mClipTrackStart] < 1.0f)
                ++mClipTrackStart;
        }
        // The skeleton stands with its feet on z = 0 in model space.
        mStandingPelvisHeight = mBasePose.mBodyTransforms[mPelvis].mTranslation.GetZ() * uniformScale;
        if (!mSpine.empty())
            mTorsoLocalUp
                = (mBasePose.mBodyTransforms[mSpine.back()].mRotation.Conjugated() * JPH::Vec3::sAxisZ()).Normalized();

        if (!mLimp || settings.mLimpAssist)
            setUpSoftRoot();
        if (mLimp && settings.mLimpAssist)
            mRagdoll->setSoftRootStrength(0.0f);
        if (!mLimp)
        {
            if (!settings.mGaitTablePath.empty())
                mGaitTable = FootfallGaitTable::load(settings.mGaitTablePath);
            mFootfallController = std::make_unique<FootfallController>(settings.mFootfall);
        }
        mValid = true;
    }

    StrideSim::~StrideSim()
    {
        mRagdoll.reset();
    }

    // The game's soft root (physicssystem.cpp): the active-body settings, the
    // legs out of the upper mask, the pelvis and spine as the torso, the feet.
    void StrideSim::setUpSoftRoot()
    {
        PassiveRagdoll::SoftRootSettings soft;
        soft.mEnabled = true;
        if (!mSettings.mActiveBodyPath.empty())
            for (const std::string& problem : soft.mBody.load(mSettings.mActiveBodyPath))
                mError += problem + "; ";
        mRagdoll->setSoftRoot(soft);
        mRagdoll->setSoftRootAnchorLayer(sAnchor);
        std::vector<bool> upper, torso, footMask;
        for (const BodyProfile& body : profile().mBodies)
        {
            const std::string& name = body.mName;
            const bool leg = name.find("thigh") != std::string::npos || name.find("calf") != std::string::npos
                || name.find("foot") != std::string::npos;
            upper.push_back(!leg);
            torso.push_back(name == "pelvis" || name == "spine" || name == "spine1" || name == "spine2");
            footMask.push_back(name.find("foot") != std::string::npos);
        }
        mRagdoll->setSoftRootBodies(std::move(upper), std::move(torso), std::move(footMask));
        mRagdoll->setRootKinematic(true);
    }

    void StrideSim::setAssist(JPH::RVec3Arg target, JPH::QuatArg rotation, float strength)
    {
        mAssistTarget = target;
        mAssistRotation = rotation;
        mAssistStrength = std::clamp(strength, 0.0f, 1.0f);
    }

    const std::vector<GroundContact>& StrideSim::groundContacts() const
    {
        return mWorld->mContacts;
    }

    std::size_t StrideSim::bodyCount() const
    {
        return mRagdoll ? mRagdoll->getBodyCount() : 0;
    }

    const RagdollProfile& StrideSim::profile() const
    {
        return mProfile;
    }

    JPH::RVec3 StrideSim::bodyPosition(std::size_t body) const
    {
        return mWorld->mPhysics.GetBodyInterface().GetCenterOfMassPosition(mRagdoll->getBodyId(body));
    }

    JPH::Quat StrideSim::bodyRotation(std::size_t body) const
    {
        return mWorld->mPhysics.GetBodyInterface().GetRotation(mRagdoll->getBodyId(body));
    }

    JPH::Vec3 StrideSim::bodyVelocity(std::size_t body) const
    {
        return mWorld->mPhysics.GetBodyInterface().GetLinearVelocity(mRagdoll->getBodyId(body));
    }

    std::string StrideSim::bodyMassesLine() const
    {
        std::ostringstream out;
        out << "RAGDOLL_BODY_MASSES";
        if (!mRagdoll)
            return out.str();
        const auto& mass = mRagdoll->getBodyMasses();
        const JPH::BodyInterface& bodies = mWorld->mPhysics.GetBodyInterface();
        for (std::size_t body = 0; body < mRagdoll->getBodyCount(); ++body)
        {
            const JPH::Vec3 size = bodies.GetShape(mRagdoll->getBodyId(body))->GetLocalBounds().GetSize();
            out << ' ' << body << ':' << (body < mass.size() ? mass[body] : -1.0f) << '/' << size.GetX() << 'x'
                << size.GetY() << 'x' << size.GetZ() << '/'
                << static_cast<int>(bodies.GetShape(mRagdoll->getBodyId(body))->GetSubType());
        }
        return out.str();
    }

    bool StrideSim::setGameState(const std::string& line, float yaw)
    {
        if (!mRagdoll)
            return false;
        std::istringstream in(line);
        double ox = 0, oy = 0, oz = 0;
        float gameYaw = 0.0f;
        std::size_t count = 0;
        if (!(in >> ox >> oy >> oz >> gameYaw >> count) || count != mRagdoll->getBodyCount())
            return false;
        struct State
        {
            JPH::RVec3 mPosition;
            JPH::Quat mRotation;
            JPH::Vec3 mLinear, mAngular;
        };
        std::vector<State> states(count);
        // Morrowind's yaw turns clockwise (forward = (sin, cos)): turning the
        // game's frame counter-clockwise by gameYaw - yaw faces it along `yaw`.
        const JPH::Quat turn = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), gameYaw - yaw);
        const JPH::RVec3 from(ox, oy, oz);
        const JPH::RVec3 to(mCapsule.GetX(), mCapsule.GetY(), mSettings.mActorHeight);
        for (State& state : states)
        {
            double px, py, pz;
            float qx, qy, qz, qw, vx, vy, vz, wx, wy, wz;
            if (!(in >> px >> py >> pz >> qx >> qy >> qz >> qw >> vx >> vy >> vz >> wx >> wy >> wz))
                return false;
            state.mPosition = to + JPH::RVec3(turn * JPH::Vec3(JPH::RVec3(px, py, pz) - from));
            state.mRotation = (turn * JPH::Quat(qx, qy, qz, qw)).Normalized();
            state.mLinear = turn * JPH::Vec3(vx, vy, vz);
            state.mAngular = turn * JPH::Vec3(wx, wy, wz);
        }
        JPH::BodyInterface& bodies = mWorld->mPhysics.GetBodyInterface();
        for (std::size_t body = 0; body < count; ++body)
        {
            const JPH::BodyID id = mRagdoll->getBodyId(body);
            const State& state = states[body];
            // Jolt places a body by its origin: the centre of mass less the
            // shape's own offset, turned.
            const JPH::RVec3 origin
                = state.mPosition - JPH::RVec3(state.mRotation * bodies.GetShape(id)->GetCenterOfMass());
            bodies.SetPositionRotationAndVelocity(id, origin, state.mRotation, state.mLinear, state.mAngular);
        }
        return true;
    }

    void StrideSim::setLimpTarget(const RagdollPose& target, float stiffness)
    {
        mLimpTarget = target;
        mLimpStiffness = std::clamp(stiffness, 0.0f, 1.0f);
    }

    void StrideSim::plan(float yaw)
    {
        RagdollMomentum momentum;
        if (!mRagdoll->captureMomentum(momentum) || !(momentum.mTotalMass > 0.0f))
            return;
        const auto& feet = mFootfallController->feet();
        FootfallCommand command;
        bool any = false;
        for (std::size_t side = 0; side < 2; ++side)
        {
            mPlans[side].reset();
            const auto& swing = feet[side];
            const auto& stance = feet[1 - side];
            // Aimed in the swing's first half only (as the game's planner).
            if (!swing.mSwinging || stance.mSwinging || swing.mPhase > mSettings.mPlanUntil)
                continue;
            const JPH::RVec3 com = momentum.mCenterOfMass;
            const JPH::Vec3 p = momentum.mLinearMomentum;
            const JPH::Vec3 l = momentum.mAngularMomentum;
            // The physical stance foot on the floor: its pin can be stale
            // when the planted foot drags (a dragged foot made the momentum
            // about a far pin, and landings missed by hundreds of units).
            const JPH::RVec3 standing = bodyPosition(mLegs[1 - side].mFoot);
            const JPH::RVec3 contact(standing.GetX(), standing.GetY(), 0.0);
            StrideInput input;
            input.mState.mCenterOfMass = { static_cast<float>(com.GetX()), static_cast<float>(com.GetY()) };
            input.mState.mVelocity = { p.GetX() / momentum.mTotalMass, p.GetY() / momentum.mTotalMass };
            input.mState.mAngularMomentum = strideAngularMomentumAbout(
                { static_cast<float>(com.GetX()), static_cast<float>(com.GetY()), static_cast<float>(com.GetZ()) },
                { p.GetX(), p.GetY(), p.GetZ() }, { l.GetX(), l.GetY(), l.GetZ() },
                { static_cast<float>(contact.GetX()), static_cast<float>(contact.GetY()),
                    static_cast<float>(contact.GetZ()) });
            input.mState.mMass = momentum.mTotalMass;
            input.mState.mHeight = std::max(20.0f, static_cast<float>(com.GetZ() - contact.GetZ()));
            input.mState.mGravity = UnitModel::GravityUnitsPerSecondSquared;
            input.mStanceFoot = { static_cast<float>(contact.GetX()), static_cast<float>(contact.GetY()) };
            const JPH::RVec3 swinging = bodyPosition(mLegs[side].mFoot);
            input.mSwingFoot = { static_cast<float>(swinging.GetX()), static_cast<float>(swinging.GetY()) };
            input.mSwingIsLeft = side == 0;
            input.mForward = { std::sin(yaw), std::cos(yaw) };
            input.mDesiredVelocity = { mRequest.GetX(), mRequest.GetY() };
            input.mElapsed = std::max(0.0f, swing.mPhase * swing.mDuration);
            input.mLimits = strideLimitsFromGait(swing.mDuration, 0.6f, 6.0f, mSettings.mPlannerLimits);
            input.mLimits.mMaximumSwingTime = std::max(input.mLimits.mMaximumSwingTime, swing.mDuration + 0.1f);
            mPlans[side] = planStride(input);
            if (!mPlans[side])
                continue;
            mRecoveryPlans += mPlans[side]->mRecovery ? 1 : 0;
            mGiveUps += mPlans[side]->mGiveUp ? 1 : 0;
            command.mLanding[side] = JPH::RVec3(mPlans[side]->mLanding.mX, mPlans[side]->mLanding.mY, 0.0);
            any = true;
        }
        if (any)
            mFootfallController->command(command);
    }

    // The stance force as the game's applyLegsLead puts it on: spread over
    // the non-leg bodies by mass, its reaction on the planted feet.
    void StrideSim::updateLegsCruising()
    {
        if (mSettings.mLegsLeadCruiseError <= 0.0f || mRequest.Length() < 15.0f)
        {
            mLegsCruising = false;
            return;
        }
        // The centre of mass's velocity, as the game's applyLegsLead reads it
        // (W1 11-05: the pelvis's swings within a step dropped the sim out of
        // the lead at its second swing, the root shoving at 1.5 x weight where
        // the game's stayed at 0).
        JPH::Vec3 body(bodyVelocity(mPelvis).GetX(), bodyVelocity(mPelvis).GetY(), 0.0f);
        if (RagdollMomentum momentum; mRagdoll->captureMomentum(momentum) && momentum.mTotalMass > 0.0f)
            body = JPH::Vec3(momentum.mLinearMomentum.GetX(), momentum.mLinearMomentum.GetY(), 0.0f) / momentum.mTotalMass;
        const float error = (JPH::Vec3(mRequest.GetX(), mRequest.GetY(), 0.0f) - body).Length() / mRequest.Length();
        if (!mLegsCruising && error <= mSettings.mLegsLeadCruiseError)
            mLegsCruising = true;
        else if (mLegsCruising && error > 2.0f * mSettings.mLegsLeadCruiseError)
            mLegsCruising = false;
    }

    void StrideSim::legsLead(const std::array<bool, 2>& grounded)
    {
        RagdollMomentum momentum;
        if (!mRagdoll->captureMomentum(momentum) || !(momentum.mTotalMass > 0.0f))
            return;
        StanceForceInput stance;
        stance.mCenterOfMass
            = { static_cast<float>(momentum.mCenterOfMass.GetX()), static_cast<float>(momentum.mCenterOfMass.GetY()) };
        stance.mVelocity = { momentum.mLinearMomentum.GetX() / momentum.mTotalMass,
            momentum.mLinearMomentum.GetY() / momentum.mTotalMass };
        stance.mRequestVelocity = { mRequest.GetX(), mRequest.GetY() };
        const auto& feet = mFootfallController->feet();
        for (std::size_t side = 0; side < 2; ++side)
            if (grounded[side] && !feet[side].mSwinging)
            {
                const JPH::RVec3 foot = bodyPosition(mLegs[side].mFoot);
                stance.mPlantedFeet[side] = BalanceVec2{ static_cast<float>(foot.GetX()), static_cast<float>(foot.GetY()) };
            }
        stance.mTotalMass = momentum.mTotalMass;
        stance.mGravity = UnitModel::GravityUnitsPerSecondSquared;
        stance.mSettings = mSettings.mStanceForce;
        // Each foot's measured load and the contacts' friction (mGripMargin).
        float friction = 1e9f;
        for (std::size_t side = 0; side < 2; ++side)
            stance.mFootLoad[side] = 0.0f;
        static const auto plantedGrip_tuned = tuned<bool>([] {
            const char* text = tuningText("OPENMW_RAGDOLL_K2_STRIDE_GRIP");
            return text && std::atoi(text) != 0;
        });
        const bool plantedGrip = plantedGrip_tuned.get();
        for (const auto& contact : mWorld->mContacts)
            for (std::size_t side = 0; side < 2; ++side)
                if (contact.mBodyIndex == mLegs[side].mFoot && contact.mNormalImpulse)
                {
                    stance.mFootLoad[side] += *contact.mNormalImpulse * contact.mNormal.GetZ() / sStep;
                    // K2_STRIDE_GRIP: the planted feet's grip only (a swinging
                    // foot brushing the floor capped the push at its 0.05 grip).
                    if (contact.mFriction && (!plantedGrip || stance.mPlantedFeet[side]))
                        friction = std::min(friction, *contact.mFriction);
                }
        stance.mFriction = friction < 1e8f ? friction : 0.0f;
        const auto force = computeStanceForce(stance);
        // The stance force each step (RAGDOLL_LEGS_FORCE: acceleration given,
        // u/s^2 along the asked direction, capped, planted feet).
        if (mSettings.mTrace)
        {
            const JPH::Vec3 asked(mRequest.GetX(), mRequest.GetY(), 0.0f);
            const float along = force && force->mActive && !asked.IsNearZero()
                ? (force->mBodyForce.mX * asked.GetX() + force->mBodyForce.mY * asked.GetY()) / asked.Length()
                    / momentum.mTotalMass
                : 0.0f;
            char line[200];
            std::snprintf(line, sizeof(line),
                "RAGDOLL_LEGS_FORCE accel=%.0f capped=%d planted=%d,%d friction=%.2f load=%.2f,%.2f actor=\"sim\"",
                along, force && force->mCapped ? 1 : 0, stance.mPlantedFeet[0] ? 1 : 0, stance.mPlantedFeet[1] ? 1 : 0,
                stance.mFriction, stance.mFootLoad[0] / (stance.mTotalMass * stance.mGravity),
                stance.mFootLoad[1] / (stance.mTotalMass * stance.mGravity));
            mSettings.mTrace(line);
        }
        if (!force || !force->mActive)
            return;
        std::vector<bool> leg(bodyCount(), false);
        for (const FootfallLeg& chain : mLegs)
            for (const std::size_t body : { chain.mHip, chain.mKnee, chain.mFoot })
                leg[body] = true;
        const auto& masses = mRagdoll->getBodyMasses();
        float upper = 0.0f;
        for (std::size_t i = 0; i < leg.size() && i < masses.size(); ++i)
            upper += leg[i] ? 0.0f : masses[i];
        if (!(upper > 0.0f))
            return;
        for (std::size_t i = 0; i < leg.size() && i < masses.size(); ++i)
            if (!leg[i])
                mRagdoll->applyExternalBodyWrench(i,
                    JPH::Vec3(force->mBodyForce.mX, force->mBodyForce.mY, 0.0f) * (masses[i] / upper), JPH::Vec3::sZero());
        for (std::size_t side = 0; side < 2; ++side)
            if (stance.mPlantedFeet[side])
                mRagdoll->applyExternalBodyWrench(mLegs[side].mFoot,
                    JPH::Vec3(force->mFootForce[side].mX, force->mFootForce[side].mY, 0.0f), JPH::Vec3::sZero());
    }

    void StrideSim::step()
    {
        if (!mValid)
            return;
        JPH::PhysicsSystem& physics = mWorld->mPhysics;
        if (mLimp)
        {
            if (mLimpTarget)
            {
                for (std::size_t i = 0; i < mProfileSprings.size(); ++i)
                    if (mProfileSprings[i].first > 0.0f)
                        mRagdoll->setMotorSpring(
                            i, std::max(0.5f, mProfileSprings[i].first * mLimpStiffness), mProfileSprings[i].second);
                mRagdoll->driveToPoseUsingMotors(*mLimpTarget);
            }
            if (mSettings.mLimpAssist)
            {
                mRagdoll->setSoftRootStrength(mAssistStrength);
                mRagdoll->setSoftRootSupport(1.0f);
                mRagdoll->moveRootKinematic(mAssistTarget, mAssistRotation, sStep);
            }
            physics.Update(sStep, 1, &mWorld->mAllocator, &mWorld->mJobs);
            mWorld->mContacts = mWorld->mListener.take(physics);
            ++mSteps;
            return;
        }

        updateLegsCruising();
        // The capsule: Morrowind's actor, moved at the asked velocity on the
        // floor - or, the legs leading, following the body's pelvis.
        const JPH::RVec3 before = mCapsule;
        if (legsLeadNow() && mSettings.mLegsLeadLeash > 0.0f)
        {
            const JPH::RVec3 pelvis = bodyPosition(mPelvis);
            mCapsule += JPH::RVec3(mRequest * sStep);
            JPH::Vec3 ahead(mCapsule - JPH::RVec3(pelvis.GetX(), pelvis.GetY(), 0.0));
            ahead.SetZ(0.0f);
            if (ahead.Length() > mSettings.mLegsLeadLeash)
                mCapsule = JPH::RVec3(pelvis.GetX(), pelvis.GetY(), 0.0)
                    + JPH::RVec3(ahead.Normalized() * mSettings.mLegsLeadLeash);
        }
        else if (legsLeadNow())
        {
            const JPH::RVec3 pelvis = bodyPosition(mPelvis);
            mCapsule = JPH::RVec3(pelvis.GetX(), pelvis.GetY(), 0.0);
        }
        else
            mCapsule += JPH::RVec3(mRequest * sStep);
        mCapsule.SetZ(mSettings.mActorHeight);
        // Smoothed as the game smooths its kinematic root's actual velocity
        // (0.65 kept a step): raw, the capsule that follows the pelvis while the
        // legs lead carried the gait's wobble into Footfall's command, and the
        // clip sync measured strides along a wandering direction (W1 11-05).
        mCapsuleVelocity = mCapsuleVelocity * 0.65f + (JPH::Vec3(mCapsule - before) / sStep) * 0.35f;
        const float yaw = mFacing ? *mFacing
            : mRequest.IsNearZero()   ? 0.0f
                                      : std::atan2(mRequest.GetX(), mRequest.GetY());
        const JPH::Quat facing = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), -yaw);
        mRagdoll->setReferenceTransform(mCapsule, facing);
        mRagdoll->captureModelPose(mPhysicsPose);

        // Ground contact per foot (its lowest bound near the floor).
        std::array<bool, 2> grounded{};
        for (std::size_t side = 0; side < 2; ++side)
            if (const auto low = mRagdoll->getPoseBodyFloorHeight(mPhysicsPose, mLegs[side].mFoot))
                grounded[side] = *low < 2.0f;
        std::array<bool, 2> routed = grounded;
        if (mSettings.mGameRootSupport)
        {
            for (std::size_t side = 0; side < 2; ++side)
            {
                routed[side] = false;
                for (const auto& contact : mWorld->mContacts)
                    if (contact.mBodyIndex == mLegs[side].mFoot && contact.hasSupport())
                        routed[side] = true;
            }
            if (mFootfallController && mRequest.Length() < 15.0f && routed[0] && routed[1])
            {
                const auto& feet = mFootfallController->feet();
                if (feet[0].mSwinging && !feet[1].mSwinging)
                    routed[0] = false;
                else if (feet[1].mSwinging && !feet[0].mSwinging)
                    routed[1] = false;
            }
        }
        for (std::size_t side = 0; side < 2; ++side)
            mRagdoll->setSoftRootGrounded(mLegs[side].mFoot, routed[side]);
        // The walking share, eased over 0.3 s as the game's (11-06 parity: the
        // sim switched it in one step, and a run slowing to a walk lost 0.6 of
        // its support at once).
        {
            const float asked = mRequest.Length();
            const bool walking = asked >= 15.0f && asked <= 150.0f && (grounded[0] || grounded[1]);
            const float want = walking ? mSettings.mWalkSupport : 1.0f;
            mRootWalkSupport += std::clamp(want - mRootWalkSupport, -sStep / 0.3f, sStep / 0.3f);
        }
        mRagdoll->setSoftRootSupport(
            (mSettings.mGameRootSupport || grounded[0] || grounded[1] ? 1.0f : 0.0f) * mRootWalkSupport);

        // The PGC budget: whole strength while fast (only the horizontal hold
        // budgeted), the standing or moving share otherwise.
        const float bodySpeed = JPH::Vec3(bodyVelocity(mPelvis).GetX(), bodyVelocity(mPelvis).GetY(), 0.0f).Length();
        const float speed = std::max(mRequest.Length(), bodySpeed);
        const bool moving = speed > 15.0f;
        if (legsLeadNow())
        {
            mRagdoll->setSoftRootStrength(mSettings.mMovingStrength);
            float share = mSettings.mLegsLeadRootShare;
            if (mSettings.mLegsLeadErrorShare > 0.0f)
            {
                const JPH::Vec3 body(bodyVelocity(mPelvis).GetX(), bodyVelocity(mPelvis).GetY(), 0.0f);
                const float error = (JPH::Vec3(mRequest.GetX(), mRequest.GetY(), 0.0f) - body).Length();
                share = std::min(1.0f, share + mSettings.mLegsLeadErrorShare * error / std::max(mRequest.Length(), 1.0f));
            }
            mRagdoll->setSoftRootHorizontalScale(share);
        }
        else if (speed > mSettings.mMoveVertical)
        {
            mRagdoll->setSoftRootStrength(1.0f);
            mRagdoll->setSoftRootHorizontalScale(mSettings.mMovingStrength);
        }
        else
        {
            mRagdoll->setSoftRootStrength(moving ? mSettings.mMovingStrength : mSettings.mStandingStrength);
            mRagdoll->setSoftRootHorizontalScale(1.0f);
        }

        if (mClipTrack)
        {
            const long long frame = static_cast<long long>(mClipTrackStart) + static_cast<long long>(mSteps)
                - static_cast<long long>(mSettings.mClipTrackAlign);
            const auto last = static_cast<long long>(mClipTrack->mPoses.size()) - 1;
            mBasePose.mBodyTransforms = mClipTrack->mPoses[static_cast<std::size_t>(std::clamp(frame, 0LL, last))];
        }
        FootfallInput input;
        input.mDt = sStep;
        input.mEligible = true;
        input.mPelvis = mPelvis;
        input.mLegs = { mLegs[0], mLegs[1] };
        input.mSpine = mSpine;
        input.mChestCarried = mChestCarried;
        input.mPhysicsPose = &mPhysicsPose;
        input.mClipPose = &mBasePose;
        input.mFrame.mReferencePosition = mRagdoll->getReferencePosition();
        input.mFrame.mInverseReferenceRotation = mRagdoll->getInverseReferenceRotation();
        input.mFrame.mInverseUniformScale = mRagdoll->getInverseUniformScale();
        input.mGround = [](const JPH::RVec3&, double) -> std::optional<double> { return 0.0; };
        input.mGroundNormal = [](const JPH::RVec3&, double) -> std::optional<JPH::Vec3> { return JPH::Vec3::sAxisZ(); };
        input.mActorZ = mCapsule.GetZ();
        input.mYaw = yaw;
        // The legs step for the capsule's velocity, as in the game.
        input.mCommandVelocity = mSettings.mLegsLead && !mSettings.mLegsLeadStepForIntent
            ? JPH::Vec3(mCapsuleVelocity.GetX(), mCapsuleVelocity.GetY(), 0.0f)
            : JPH::Vec3(mRequest.GetX(), mRequest.GetY(), 0.0f);
        input.mGaitTable = mGaitTable ? &*mGaitTable : nullptr;
        input.mFootContact = grounded;
        // Each foot's contact slip, as the game measures it (_DRAG_LIFT).
        std::array<float, 2> slip{ 0.0f, 0.0f };
        for (const auto& contact : mWorld->mContacts)
            for (std::size_t side = 0; side < 2; ++side)
                if (contact.mBodyIndex == mLegs[side].mFoot && contact.hasSupport() && contact.mPointVelocity)
                    slip[side] = std::max(slip[side],
                        std::hypot(contact.mPointVelocity->GetX(), contact.mPointVelocity->GetY()));
        input.mFootSlip = slip;
        if (mSettings.mTrace)
            input.mTrace = [this](const std::string& line) { mSettings.mTrace(line + " actor=\"sim\""); };
        if (mSettings.mPlanner)
            plan(yaw);
        const bool legsLeading = legsLeadNow();
        FootfallCommand command;
        bool commanded = false;
        if (mSettings.mTransfer && legsLeading && !mSettings.mPlanner)
        {
            command.mHoldClock = true;
            commanded = true;
            const auto& feet = mFootfallController->feet();
            if (feet[0].mSwinging || feet[1].mSwinging)
            {
                mNextSwing.reset();
                mTransferElapsed = 0.0f;
            }
            else
            {
                if (!mNextSwing)
                {
                    const JPH::RVec3 left = bodyPosition(mLegs[0].mFoot), right = bodyPosition(mLegs[1].mFoot);
                    mNextSwing = strideFirstFootIsLeft({ static_cast<float>(left.GetX()), static_cast<float>(left.GetY()) },
                                     { static_cast<float>(right.GetX()), static_cast<float>(right.GetY()) },
                                     { mRequest.GetX(), mRequest.GetY() }, false)
                        ? 0
                        : 1;
                }
                mTransferElapsed += sStep;
                const float load = calculateMeasuredFootNormalLoadFraction(mWorld->mContacts, mLegs[*mNextSwing].mFoot,
                    mSettings.mTotalMass, UnitModel::GravityUnitsPerSecondSquared, sStep)
                                       .value_or(1.0f);
                const bool unloaded = load < mSettings.mTransferLoad;
                if (unloaded || mTransferElapsed >= mSettings.mTransferTime)
                {
                    command.mSwingNow = *mNextSwing;
                    command.mSwingTime = mSettings.mTransferSwingTime;
                    (unloaded ? mTransferLifts : mTransferTimeouts) += 1;
                }
            }
        }
        if (mSettings.mLegsLeadAimSwing && mSettings.mLegsLeadLeash > 0.0f && !mSettings.mPlanner)
        {
            const JPH::RVec3 pelvis = bodyPosition(mPelvis);
            const JPH::RVec3 lead(mCapsule.GetX() - pelvis.GetX(), mCapsule.GetY() - pelvis.GetY(), 0.0);
            for (std::size_t side = 0; side < 2; ++side)
            {
                const auto& foot = mFootfallController->feet()[side];
                if (!foot.mSwinging)
                    mSwingAimed[side] = false;
                else if (!mSwingAimed[side])
                {
                    command.mLanding[side] = foot.mTarget + lead;
                    mSwingAimed[side] = commanded = true;
                }
            }
        }
        if (commanded)
            mFootfallController->command(command);
        mDrivePose = mBasePose;
        mFootfallOutput = {};
        mFootfallController->step(input, mDrivePose, mFootfallOutput);

        // The leg springs Footfall asks for (as applyFootfallLibrarySprings).
        for (std::size_t side = 0; side < 2; ++side)
        {
            const auto& springs = mFootfallOutput.mSprings[side];
            for (const std::size_t body : { mLegs[side].mHip, mLegs[side].mKnee })
            {
                const float give = body == mLegs[side].mKnee ? springs.mKneeGive : 1.0f;
                mRagdoll->setMotorSpring(body, mProfileSprings[body].first * springs.mStiffness * give,
                    mProfileSprings[body].second * springs.mDamping);
            }
        }

        // The root target: the drive pose's pelvis, lowered by the gait's drop.
        if (const auto target = mRagdoll->getWorldBodyPosition(mDrivePose, mPelvis))
        {
            JPH::RVec3 at = *target;
            // The game's run bob (OPENMW_RAGDOLL_RUN_BOB, roottarget): running,
            // the root's target keeps a share of the clip pelvis's bob and sits
            // lower (11-06 parity: without it the sim's runner rode 3-4 u high
            // on the root, its feet loaded on a third of the frames).
            if (mSettings.mRunBob)
            {
                const float offset = static_cast<float>(at.GetZ() - mCapsule.GetZ());
                at.SetZ(mCapsule.GetZ()
                    + applyRunBob(mRunBob, offset, JPH::Vec3(mRequest.GetX(), mRequest.GetY(), 0.0f), true,
                        *mSettings.mRunBob, sStep));
            }
            at.SetZ(at.GetZ() - mFootfallController->pelvisDrop() - mSettings.mRootLower);
            const auto rotation = mRagdoll->getWorldBodyRotation(mDrivePose, mPelvis);
            mRagdoll->moveRootKinematic(at, rotation ? *rotation : facing, sStep);
        }
        if (mSettings.mGameFootGrip)
        {
            static const auto pinnedGrip_tuned = tuned<float>([] {
                const char* text = tuningText("OPENMW_RAGDOLL_PINNED_FOOT_FRICTION");
                return text ? static_cast<float>(std::atof(text)) : 2.0f;
            });
            for (std::size_t side = 0; side < 2; ++side)
                mRagdoll->setBodyFrictionScale(mLegs[side].mFoot,
                    mFootfallOutput.mPinned[side]         ? pinnedGrip_tuned.get()
                        : mFootfallOutput.mSwinging[side] ? 0.05f
                                                          : 1.0f);
        }
        mRagdoll->driveToPoseUsingMotors(mDrivePose);
        if (mSettings.mSwingFeedForward)
        {
            static const auto feed_tuned = tuned<float>([] {
                const char* text = tuningText("OPENMW_RAGDOLL_FOOTFALL_FF");
                return text ? static_cast<float>(std::atof(text)) : 0.5f;
            });
            static const auto gain_tuned = tuned<float>([] {
                const char* text = tuningText("OPENMW_RAGDOLL_LEG_FEEDFORWARD");
                const float value = text ? static_cast<float>(std::atof(text)) : 1.0f;
                return std::isfinite(value) ? std::clamp(value, -2.0f, 2.0f) : 1.0f;
            });
            const auto& velocities = mFootfallOutput.mAngularVelocity;
            const auto at = [&](std::size_t body) {
                const auto found = velocities.find(body);
                return found == velocities.end() ? JPH::Vec3::sZero() : found->second * feed_tuned.get();
            };
            const auto feed = [&](std::size_t body, JPH::Vec3 relative) {
                relative *= gain_tuned.get();
                if (relative.Length() > 12.0f)
                    relative *= 12.0f / relative.Length();
                if (const auto world = mRagdoll->getWorldVector(relative))
                    mRagdoll->setMotorTargetAngularVelocity(body, *world);
            };
            for (std::size_t side = 0; side < 2; ++side)
                if (mFootfallOutput.mPinned[side] || mFootfallOutput.mSwinging[side])
                {
                    feed(mLegs[side].mHip, at(mLegs[side].mHip) - at(mPelvis));
                    feed(mLegs[side].mKnee, at(mLegs[side].mKnee) - at(mLegs[side].mHip));
                    feed(mLegs[side].mFoot, at(mLegs[side].mFoot) - at(mLegs[side].mKnee));
                }
        }
        if (legsLeadNow() && !mSettings.mLegsLeadThroughSupport)
            legsLead(grounded);
        // The game plans the joint support only while no movement is asked
        // for (GaitActuationDecision: mDriveJointMotors && !locomotion requested).
        // The legs leading the game's way (the stance force, legsLead above)
        // add no joint support: only the through-support variant walks on it
        // (W1 11-05: with it the sim's walking pelvis rode 6 u over the game's).
        if (mSettings.mJointSupport && (!mSettings.mGameRootSupport || mRequest.IsNearZero()
                || (legsLeadNow() && mSettings.mLegsLeadThroughSupport)))
        {
            // The legs leading through the support: a centre-of-mass target
            // that moves at the asked velocity from where the body was.
            std::optional<SupportMotionReference> motion;
            const bool leading
                = mSettings.mLegsLead && mSettings.mLegsLeadThroughSupport && mRequest.Length() >= 15.0f;
            RagdollTelemetry telemetry;
            const bool measured = mRagdoll->captureTelemetry(telemetry);
            if (leading && measured && mSettings.mLegsLeadPush)
            {
                const JPH::Vec3 travel = JPH::Vec3(mRequest.GetX(), mRequest.GetY(), 0.0f).Normalized();
                const JPH::Vec3 across(travel.GetY(), -travel.GetX(), 0.0f);
                const JPH::Vec3 com(telemetry.mCenterOfMass);
                JPH::Vec3 middle((bodyPosition(mLegs[0].mFoot) + bodyPosition(mLegs[1].mFoot)) * 0.5);
                // Stepping by events: across the travel, over the stance foot
                // (the one not swinging, or not lifting next); a swing's
                // planned loads go on the stance foot alone.
                std::optional<std::size_t> supportFoot;
                if (mSettings.mTransfer)
                {
                    const auto& feet = mFootfallController->feet();
                    std::optional<std::size_t> stance;
                    if (feet[0].mSwinging != feet[1].mSwinging)
                        stance = feet[0].mSwinging ? 1 : 0;
                    else if (mNextSwing)
                        stance = 1 - *mNextSwing;
                    if (stance)
                    {
                        middle = JPH::Vec3(bodyPosition(mLegs[*stance].mFoot));
                        if (feet[1 - *stance].mSwinging)
                            supportFoot = mLegs[*stance].mFoot;
                    }
                }
                const JPH::Vec3 at = travel * com.Dot(travel) + across * middle.Dot(across)
                    + JPH::Vec3(0.0f, 0.0f, com.GetZ());
                motion = SupportMotionReference{ JPH::RVec3(at), JPH::Vec3(mRequest.GetX(), mRequest.GetY(), 0.0f),
                    JPH::Vec3::sZero(), supportFoot };
            }
            else if (leading && measured)
            {
                if (!mMotionTarget)
                    mMotionTarget = telemetry.mCenterOfMass;
                *mMotionTarget += JPH::RVec3(JPH::Vec3(mRequest.GetX(), mRequest.GetY(), 0.0f) * sStep);
                // Never further than a stride ahead of the body.
                JPH::Vec3 ahead(*mMotionTarget - telemetry.mCenterOfMass);
                ahead.SetZ(0.0f);
                if (ahead.Length() > 40.0f)
                    *mMotionTarget = telemetry.mCenterOfMass + JPH::RVec3(ahead.Normalized() * 40.0f);
                motion = SupportMotionReference{ *mMotionTarget, JPH::Vec3(mRequest.GetX(), mRequest.GetY(), 0.0f),
                    JPH::Vec3::sZero(), std::nullopt };
            }
            else
                mMotionTarget.reset();
            if (measured)
            {
                // The game hands the support the contacts' lowest combined friction.
                float friction = mSettings.mGroundFriction;
                if (mSettings.mGameFootGrip)
                {
                    float lowest = std::numeric_limits<float>::max();
                    for (const auto& contact : mWorld->mContacts)
                        if (contact.mFriction
                            && (contact.mBodyIndex == mLegs[0].mFoot || contact.mBodyIndex == mLegs[1].mFoot))
                            lowest = std::min(lowest, *contact.mFriction);
                    if (lowest < std::numeric_limits<float>::max())
                        friction = lowest;
                }
                if (const auto support = calculateStandingJointSupport(telemetry, mWorld->mContacts,
                        FootBodyIndices{ mLegs[0].mFoot, mLegs[1].mFoot }, UnitModel::GravityUnitsPerSecondSquared,
                        friction, profile().mActiveController, motion))
                    for (std::size_t body = 1; body < support->mJointTorques.size(); ++body)
                        mRagdoll->driveMotorWithLoadTorque(body, support->mJointTorques[body], sStep);
            }
        }
        // The swing spring (strideswing).
        if (mSettings.mStrideSwing.mFrequency > 0.0f)
        {
            const auto& masses = mRagdoll->getBodyMasses();
            const auto& feet = mFootfallController->feet();
            const JPH::BodyInterface& bodies = mWorld->mPhysics.GetBodyInterface();
            StrideSwingInput swing;
            swing.mGravity = UnitModel::GravityUnitsPerSecondSquared;
            swing.mSettings = mSettings.mStrideSwing;
            const JPH::Vec3 pelvisVelocity = bodyVelocity(mPelvis);
            swing.mPelvisVelocity = { pelvisVelocity.GetX(), pelvisVelocity.GetY() };
            for (std::size_t side = 0; side < 2; ++side)
            {
                const FootfallLeg& leg = mLegs[side];
                StrideSwingFoot& foot = swing.mFeet[side];
                foot.mSwinging = feet[side].mSwinging;
                foot.mPhase = feet[side].mPhase;
                foot.mDuration = feet[side].mDuration;
                foot.mLiftOff = { static_cast<float>(feet[side].mPin.GetX()), static_cast<float>(feet[side].mPin.GetY()) };
                foot.mLanding = { static_cast<float>(feet[side].mTarget.GetX()), static_cast<float>(feet[side].mTarget.GetY()) };
                foot.mLandingHeight = static_cast<float>(feet[side].mTarget.GetZ());
                const JPH::RVec3 at = bodyPosition(leg.mFoot);
                const JPH::Vec3 velocity = bodyVelocity(leg.mFoot);
                foot.mFoot = { static_cast<float>(at.GetX()), static_cast<float>(at.GetY()) };
                foot.mFootVelocity = { velocity.GetX(), velocity.GetY() };
                const JPH::RVec3 hip = bodies.GetPosition(mRagdoll->getBodyId(leg.mHip));
                foot.mHip = { static_cast<float>(hip.GetX()), static_cast<float>(hip.GetY()), static_cast<float>(hip.GetZ()) };
                foot.mLegLength = leg.mGeometry
                    ? leg.mGeometry->mUpperOffset.Length() + leg.mGeometry->mLowerOffset.Length() : 0.0f;
                foot.mLegMass = masses[leg.mHip] + masses[leg.mKnee] + masses[leg.mFoot];
                const JPH::RVec3 footOrigin = bodies.GetPosition(mRagdoll->getBodyId(leg.mFoot));
                foot.mFootHeight = static_cast<float>(footOrigin.GetZ());
                foot.mFootVerticalVelocity = velocity.GetZ();
                foot.mLift = feet[side].mLift;
            }
            if (const auto out = computeStrideSwing(swing))
            {
                for (std::size_t side = 0; side < 2; ++side)
                    if (const auto& force = out->mFootForce[side])
                        mRagdoll->applyExternalBodyWrench(mLegs[side].mFoot,
                            JPH::Vec3(force->mX, force->mY, out->mFootLift[side]), JPH::Vec3::sZero());
                if (out->mFootForce[0] || out->mFootForce[1])
                    mRagdoll->applyExternalBodyWrench(mPelvis,
                        JPH::Vec3(out->mPelvisForce.mX, out->mPelvisForce.mY, out->mPelvisLift), JPH::Vec3::sZero());
            }
        }
        // The game's upright spine torque (mUprightSpine): under the held root
        // only the chest's tilt spring reaches the body (composeUprightControl),
        // its reaction on the stance leg's thigh.
        if (mSettings.mUprightSpine && !mSpine.empty() && !mLimp)
        {
            const auto& feet = mFootfallController->feet();
            if (feet[0].mSwinging != feet[1].mSwinging)
                mLastStance = feet[0].mSwinging ? 1 : 0;
            UprightControllerInput upright;
            upright.mTotalMass = mSettings.mTotalMass;
            upright.mGravityMagnitude = UnitModel::GravityUnitsPerSecondSquared;
            upright.mPelvisHeight = upright.mTargetPelvisHeight = static_cast<float>(bodyPosition(mPelvis).GetZ());
            upright.mPelvisUp = (bodyRotation(mPelvis) * JPH::Vec3::sAxisZ()).Normalized();
            upright.mPelvisAngularVelocity
                = mWorld->mPhysics.GetBodyInterface().GetAngularVelocity(mRagdoll->getBodyId(mPelvis));
            upright.mTorsoUp = (bodyRotation(mSpine.back()) * mTorsoLocalUp).Normalized();
            upright.mTorsoAngularVelocity
                = mWorld->mPhysics.GetBodyInterface().GetAngularVelocity(mRagdoll->getBodyId(mSpine.back()));
            upright.mCenterOfMassPosition = bodyPosition(mPelvis);
            upright.mCenterOfMassHeight = upright.mPelvisHeight;
            if (const auto correction = calculateUprightControl(upright, profile().mActiveController))
            {
                const UprightControllerOutput control
                    = composeUprightControl(*correction, true, 1.0f, false, false);
                mRagdoll->applyUprightControl(control, mPelvis, mSpine.back(), {}, {}, profile().mActiveController,
                    mLegs[mLastStance].mHip, true, std::nullopt);
            }
        }
        physics.Update(sStep, 1, &mWorld->mAllocator, &mWorld->mJobs);
        mWorld->mContacts = mWorld->mListener.take(physics);
        ++mSteps;
        // Who carries the weight, as the game's RAGDOLL_LOAD (MOVE-01 W1):
        // each foot's vertical ground force and the root's force in body
        // weights, the loaded contacts' slide, the pelvis over the lower foot.
        if (mSettings.mTrace)
        {
            const float weight = mSettings.mTotalMass * UnitModel::GravityUnitsPerSecondSquared;
            float load[2] = { 0.0f, 0.0f };
            float slip[2] = { 0.0f, 0.0f };
            int footContacts = 0, withImpulse = 0;
            for (const auto& contact : mWorld->mContacts)
            {
                const int side = contact.mBodyIndex == mLegs[0].mFoot ? 0 : contact.mBodyIndex == mLegs[1].mFoot ? 1 : -1;
                if (side >= 0)
                {
                    ++footContacts;
                    withImpulse += contact.mNormalImpulse ? 1 : 0;
                }
                if (side < 0 || !contact.mNormalImpulse || *contact.mNormalImpulse <= 0.0f)
                    continue;
                load[side] += *contact.mNormalImpulse * contact.mNormal.GetZ() / sStep;
                if (contact.mPointVelocity)
                    slip[side] = std::max(slip[side],
                        std::hypot(contact.mPointVelocity->GetX(), contact.mPointVelocity->GetY()));
            }
            const JPH::Vec3 force = mRagdoll->getSoftRootTelemetry().mForce;
            // Body origins, as the game's trace reads them (telemetry
            // mPosition; bodyPosition() is the centre of mass).
            const auto origin = [&](std::size_t body) {
                return mWorld->mPhysics.GetBodyInterface().GetPosition(mRagdoll->getBodyId(body));
            };
            const double lowFoot = std::min(origin(mLegs[0].mFoot).GetZ(), origin(mLegs[1].mFoot).GetZ());
            char line[256];
            std::snprintf(line, sizeof(line),
                "RAGDOLL_LOAD left=%.3f right=%.3f rootV=%.3f rootH=%.3f slipL=%.1f slipR=%.1f pelvisH=%.1f "
                "errZ=%.1f actor=\"sim\"",
                load[0] / weight, load[1] / weight, force.GetZ() / weight,
                std::hypot(force.GetX(), force.GetY()) / weight, slip[0], slip[1],
                origin(mPelvis).GetZ() - lowFoot, mRagdoll->getSoftRootTelemetry().mPositionErrorUp);
            std::string text = line;
            text.insert(text.rfind(" actor="), " contacts=" + std::to_string(footContacts) + "/" + std::to_string(withImpulse));
            {
                char part[48];
                if (const auto drive = mRagdoll->getWorldBodyPosition(mDrivePose, mLegs[0].mFoot))
                {
                    char extra[48];
                    std::snprintf(extra, sizeof(extra), " targetZL=%.1f", static_cast<double>(drive->GetZ()));
                    text.insert(text.rfind(" actor="), extra);
                }
                std::snprintf(part, sizeof(part), " hipDZ=%.3f",
                    static_cast<double>(origin(mLegs[0].mHip).GetZ() - origin(mLegs[1].mHip).GetZ()));
                text.insert(text.rfind(" actor="), part);
            }
            // The stance leg's motors, as the game's RAGDOLL_LEG_DRIVE prints them.
            for (const std::size_t body : { mLegs[0].mHip, mLegs[0].mKnee })
            {
                MotorTargetTelemetry motor;
                mRagdoll->captureMotorTargetTelemetry(body, motor);
                char part[96];
                std::snprintf(part, sizeof(part), " %s=%.1f,%.1f,%.2f", body == mLegs[0].mHip ? "hip" : "knee",
                    motor.mDrive ? motor.mDrive->mSwingOrHingeSpring.mFrequencyOrStiffness : -1.0f,
                    motor.mDrive ? motor.mDrive->mMaximumTorque.GetX() : -1.0f,
                    motor.mDrive ? motor.mDrive->mSwingOrHingeSpring.mDamping : -1.0f);
                text.insert(text.rfind(" actor="), part);
                if (body == mLegs[0].mHip && motor.mCurrentOrientation && motor.mTargetOrientation)
                {
                    std::snprintf(part, sizeof(part), " hipE=%.1f",
                        JPH::RadiansToDegrees(2.0f * std::acos(std::clamp(
                            std::abs(motor.mCurrentOrientation->Dot(*motor.mTargetOrientation)), 0.0f, 1.0f))));
                    text.insert(text.rfind(" actor="), part);
                }
                if (body == mLegs[0].mKnee && motor.mCurrentHingeAngle && motor.mTargetHingeAngle)
                {
                    std::snprintf(part, sizeof(part), " kneeA=%.1f,%.1f swingL=%d",
                        JPH::RadiansToDegrees(*motor.mCurrentHingeAngle), JPH::RadiansToDegrees(*motor.mTargetHingeAngle),
                        mFootfallOutput.mSwinging[0] ? 1 : 0);
                    text.insert(text.rfind(" actor="), part);
                }
            }
            // The knees' motor utilization (impulse over its cap), as the game's kneeU.
            {
                float u[2] = { -1.0f, -1.0f };
                for (std::size_t side = 0; side < 2; ++side)
                {
                    JPH::Vec3 impulse;
                    float used = 0.0f;
                    if (mRagdoll->getMotorAngularImpulse(mLegs[side].mKnee, sStep, impulse, used))
                        u[side] = used;
                }
                char part[48];
                std::snprintf(part, sizeof(part), " kneeU=%.3f,%.3f", u[0], u[1]);
                text.insert(text.rfind(" actor="), part);
            }
            // The right knee too (current, target), as the game's side=1 line.
            {
                MotorTargetTelemetry motor;
                mRagdoll->captureMotorTargetTelemetry(mLegs[1].mKnee, motor);
                if (motor.mCurrentHingeAngle && motor.mTargetHingeAngle)
                {
                    char part[64];
                    std::snprintf(part, sizeof(part), " kneeAR=%.1f,%.1f swingR=%d",
                        JPH::RadiansToDegrees(*motor.mCurrentHingeAngle), JPH::RadiansToDegrees(*motor.mTargetHingeAngle),
                        mFootfallOutput.mSwinging[1] ? 1 : 0);
                    text.insert(text.rfind(" actor="), part);
                }
            }
            mSettings.mTrace(text);
        }
    }
}
