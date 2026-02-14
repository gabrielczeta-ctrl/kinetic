#ifndef OPENMW_MWPHYSICS_PHYSICSSYSTEM_H
#define OPENMW_MWPHYSICS_PHYSICSSYSTEM_H

#include <algorithm>
#include <array>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <unordered_map>
#include <variant>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>

#include <osg/BoundingBox>
#include <osg/Quat>
#include <osg/Timer>
#include <osg/ref_ptr>

#include <components/vfs/pathutil.hpp>

#include "../mwworld/ptr.hpp"

#include "joltlisteners.hpp"
#include "raycasting.hpp"

namespace JPH
{
    class JobSystem;
    class TempAllocatorImpl;
    class PhysicsSystem;
    class BodyInterface;
    class BodyLockInterfaceLocking;
}

namespace osg
{
    class Group;
    class Object;
    class Stats;
}

namespace MWRender
{
    class JoltDebugDrawer;
}

namespace Resource
{
    class PhysicsShapeManager;
    class ResourceSystem;
}

namespace MWPhysics
{
    class MWWater;
    class HeightField;
    class Object;
    class Actor;
    class PhysicsTaskScheduler;
    class Projectile;
    enum ScriptedCollisionType : char;

    using ActorMap = std::unordered_map<const MWWorld::LiveCellRefBase*, std::shared_ptr<Actor>>;

    struct ContactPoint
    {
        MWWorld::Ptr mObject;
        osg::Vec3f mPoint;
        osg::Vec3f mNormal;
    };

    struct LOSRequest
    {
        LOSRequest(const std::weak_ptr<Actor>& a1, const std::weak_ptr<Actor>& a2);
        std::array<std::weak_ptr<Actor>, 2> mActors;
        std::array<const Actor*, 2> mRawActors;
        bool mResult;
        bool mStale;
        int mAge;
    };
    bool operator==(const LOSRequest& lhs, const LOSRequest& rhs) noexcept;

    struct ActorFrameData
    {
        ActorFrameData(Actor& actor, bool inert, bool waterCollision, float slowFall, float waterlevel, bool isPlayer);
        osg::Vec3f mPosition;
        osg::Vec3f mInertia;
        bool mIsOnGround;
        bool mIsOnSlope;
        bool mWalkingOnWater;
        const bool mInert;
        JPH::BodyID mStandingOn;
        JPH::BodyID mPhysicsBody;
        const float mSwimLevel;
        const float mSlowFall;
        osg::Vec2f mRotation;
        osg::Vec3f mMovement;
        osg::Vec3f mLastStuckPosition;
        const float mWaterlevel;
        const float mHalfExtentsZ;
        float mOldHeight;
        unsigned int mStuckFrames;
        const bool mFlying;
        const bool mWasOnGround;
        const bool mIsAquatic;
        const bool mWaterCollision;
        const bool mSkipCollisionDetection;
        const bool mIsPlayer;
        JPH::ObjectLayer mCollisionMask;
    };

    struct WorldFrameData
    {
        WorldFrameData();
        bool mIsInStorm;
        osg::Vec3f mStormDirection;
    };

    template <class Ptr, class FrameData>
    class SimulationImpl
    {
    public:
        explicit SimulationImpl(const std::weak_ptr<Ptr>& ptr, FrameData&& data)
            : mPtr(ptr)
            , mData(data)
        {
        }

        std::optional<std::pair<std::shared_ptr<Ptr>, std::reference_wrapper<FrameData>>> lock()
        {
            if (auto locked = mPtr.lock())
                return { { std::move(locked), std::ref(mData) } };
            return std::nullopt;
        }

    private:
        std::weak_ptr<Ptr> mPtr;
        FrameData mData;
    };

    using ActorSimulation = SimulationImpl<Actor, ActorFrameData>;
    using Simulation = std::variant<ActorSimulation>;

    class PhysicsSystem : public RayCastingInterface
    {
    public:
        PhysicsSystem(Resource::ResourceSystem* resourceSystem, osg::ref_ptr<osg::Group> parentNode);
        virtual ~PhysicsSystem();

        Resource::PhysicsShapeManager* getShapeManager();

        void enableWater(float height);
        void setWaterHeight(float height);
        void disableWater();

        void addObject(const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh, osg::Quat rotation,
            int collisionType = Layers::WORLD);
        void addActor(const MWWorld::Ptr& ptr, VFS::Path::NormalizedView mesh);

        int addProjectile(
            const MWWorld::Ptr& caster, const osg::Vec3f& position, VFS::Path::NormalizedView mesh, bool computeRadius);
        void setCaster(int projectileId, const MWWorld::Ptr& caster);
        void removeProjectile(const int projectileId);

        void updatePtr(const MWWorld::Ptr& old, const MWWorld::Ptr& updated);

        Actor* getActor(const MWWorld::Ptr& ptr);
        const Actor* getActor(const MWWorld::ConstPtr& ptr) const;

        const Object* getObject(const MWWorld::ConstPtr& ptr) const;

        Projectile* getProjectile(int projectileId) const;

        // Object or Actor
        void remove(const MWWorld::Ptr& ptr);

        void updateScale(const MWWorld::Ptr& ptr);
        void updateRotation(const MWWorld::Ptr& ptr, osg::Quat rotate);
        void updatePosition(const MWWorld::Ptr& ptr);

        void addHeightField(const float* heights, int x, int y, int size, int verts, float minH, float maxH,
            const osg::Object* holdObject);

        void removeHeightField(int x, int y);

        const HeightField* getHeightField(int x, int y) const;

        bool toggleCollisionMode();

        /// Determine new position based on all queued movements, then clear the list.
        void stepSimulation(
            float dt, bool skipSimulation, osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats);

        /// Apply new positions to actors
        void moveActors();
        void debugDraw();

        std::vector<MWWorld::Ptr> getCollisions(const MWWorld::ConstPtr& ptr, int collisionGroup,
            int collisionMask) const; ///< get handles this object collides with
        std::vector<ContactPoint> getCollisionsPoints(
            const MWWorld::ConstPtr& ptr, int collisionGroup, int collisionMask) const;
        osg::Vec3f traceDown(const MWWorld::Ptr& ptr, const osg::Vec3f& position, float maxHeight);
        void optimize();

        /// @param ignore Optional, a list of Ptr to ignore in the list of results. targets are actors to filter for,
        /// ignoring all other actors.
        RayCastingResult castRay(const osg::Vec3f& from, const osg::Vec3f& to,
            const std::vector<MWWorld::ConstPtr>& ignore = {}, const std::vector<MWWorld::Ptr>& targets = {},
            int mask = CollisionMask_Default, int group = 0xff) const override;
        using RayCastingInterface::castRay;

        RayCastingResult castSphere(const osg::Vec3f& from, const osg::Vec3f& to, float radius,
            int mask = CollisionMask_Default, int group = 0xff) const override;

        /// Return true if actor1 can see actor2.
        bool getLineOfSight(const MWWorld::ConstPtr& actor1, const MWWorld::ConstPtr& actor2) const override;

        bool isOnGround(const MWWorld::Ptr& actor);

        bool canMoveToWaterSurface(const MWWorld::ConstPtr& actor, const float waterlevel);

        /// Get physical half extents (scaled) of the given actor.
        osg::Vec3f getHalfExtents(const MWWorld::ConstPtr& actor) const;

        /// Get physical half extents (not scaled) of the given actor.
        osg::Vec3f getOriginalHalfExtents(const MWWorld::ConstPtr& actor) const;

        /// @see MWPhysics::Actor::getRenderingHalfExtents
        osg::Vec3f getRenderingHalfExtents(const MWWorld::ConstPtr& actor) const;

        /// Get the position of the collision shape for the actor. Use together with getHalfExtents() to get the
        /// collision bounds in world space.
        /// @note The collision shape's origin is in its center, so the position returned can be described as center of
        /// the actor collision box in world space.
        osg::Vec3f getCollisionObjectPosition(const MWWorld::ConstPtr& actor) const;

        /// Get bounding box in world space of the given object.
        osg::BoundingBox getBoundingBox(const MWWorld::ConstPtr& object) const;

        /// Queues velocity movement for a Ptr. If a Ptr is already queued, its velocity will
        /// be overwritten. Valid until the next call to stepSimulation
        void queueObjectMovement(const MWWorld::Ptr& ptr, const osg::Vec3f& velocity);

        /// Force a collision-shape animation update for a single object.
        void updateAnimatedCollisionShape(const MWWorld::Ptr& ptr);

        /// Clear the queued movements list without applying.
        void clearQueuedMovement();

        /// Return true if \a actor has been standing on \a object in this frame
        /// This will trigger whenever the object is directly below the actor.
        /// It doesn't matter if the actor is stationary or moving.
        bool isActorStandingOn(const MWWorld::Ptr& actor, const MWWorld::ConstPtr& object) const;

        /// Get the handle of all actors standing on \a object in this frame.
        void getActorsStandingOn(const MWWorld::ConstPtr& object, std::vector<MWWorld::Ptr>& out) const;

        /// Return true if an object of the given type has collided with this object
        bool isObjectCollidingWith(const MWWorld::ConstPtr& object, ScriptedCollisionType type) const;

        /// Get the handle of all actors colliding with \a object in this frame.
        void getActorsCollidingWith(const MWWorld::ConstPtr& object, std::vector<MWWorld::Ptr>& out) const;

        bool toggleDebugRendering();

        void reportCollision(const osg::Vec3f& position, const osg::Vec3f& normal);

        /// Mark the given object as a 'non-solid' object. A non-solid object means that
        /// \a isOnSolidGround will return false for actors standing on that object.
        void markAsNonSolid(const MWWorld::ConstPtr& ptr);

        bool isOnSolidGround(const MWWorld::Ptr& actor) const;

        template <class Function>
        void forEachAnimatedObject(Function&& function) const
        {
            std::for_each(mAnimatedObjects.begin(), mAnimatedObjects.end(), function);
        }

        bool isAreaOccupiedByOtherActor(const osg::Vec3f& position, const float radius,
            std::span<const MWWorld::ConstPtr> ignore, std::vector<MWWorld::Ptr>* occupyingActors) const;

        float getPhysicsDt() const { return mPhysicsDt; }

        void reportStats(unsigned int frameNumber, osg::Stats& stats) const;

        inline const JPH::BodyLockInterfaceLocking& getBodyLockInterface() const;

        const JPH::BodyInterface& getBodyInterface() const;

    private:
        void updateWater();
        void updatePtrHolders();

        void prepareSimulation(bool willSimulate, std::vector<Simulation>& simulations);

        JoltContactListener mContactListener;
        JoltBPLayerInterface mBPLayerInterface;
        JoltObjectVsBroadPhaseLayerFilter mObjectVsBPLayerFilter;
        JoltObjectLayerPairFilter mObjectVsObjectLayerFilter;

        JPH::PhysicsSystem mPhysicsSystem;
        std::unique_ptr<PhysicsTaskScheduler> mTaskScheduler;
        std::unique_ptr<JPH::TempAllocatorImpl> mMemoryAllocator;
        std::unique_ptr<JPH::JobSystem> mPhysicsJobSystem;
        std::unique_ptr<Resource::PhysicsShapeManager> mShapeManager;
        Resource::ResourceSystem* mResourceSystem;

        using ObjectMap = std::unordered_map<const MWWorld::LiveCellRefBase*, std::shared_ptr<Object>>;
        ObjectMap mObjects;

        std::map<Object*, bool> mAnimatedObjects; // stores pointers to elements in mObjects

        ActorMap mActors;

        using ProjectileMap = std::map<int, std::shared_ptr<Projectile>>;
        ProjectileMap mProjectiles;

        using HeightFieldMap = std::map<std::pair<int, int>, std::unique_ptr<HeightField>>;
        HeightFieldMap mHeightFields;

        bool mDebugDrawEnabled;

        float mTimeAccum;

        unsigned int mProjectileId;

        float mTimeAccumJolt = 0.0f;
        float mWaterHeight;
        bool mWaterEnabled;

        std::unique_ptr<MWWater> mWaterInstance;

        std::unique_ptr<MWRender::JoltDebugDrawer> mJoltDebugDrawer;

        osg::ref_ptr<osg::Group> mParentNode;

        float mPhysicsDt;

        std::size_t mSimulationsCounter = 0;
        std::array<std::vector<Simulation>, 2> mSimulations;
        std::vector<std::pair<MWWorld::Ptr, osg::Vec3f>> mActorsPositions;

        PhysicsSystem(const PhysicsSystem&);
        PhysicsSystem& operator=(const PhysicsSystem&);
    };
}

#endif
