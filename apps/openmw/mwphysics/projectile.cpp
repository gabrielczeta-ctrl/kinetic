#include <memory>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include <components/misc/convert.hpp>
#include <components/physicshelpers/collisionobject.hpp>

#include "actor.hpp"
#include "joltlayers.hpp"
#include "mtphysics.hpp"
#include "object.hpp"
#include "projectile.hpp"

namespace MWPhysics
{
    Projectile::Projectile(const MWWorld::Ptr& caster, const osg::Vec3f& position, float radius,
        PhysicsTaskScheduler* scheduler, PhysicsSystem* physicssystem)
        : PtrHolder(MWWorld::Ptr(), position)
        , mHitWater(false)
        , mActive(true)
        , mPhysics(physicssystem)
        , mTaskScheduler(scheduler)
    {
        mPosition = position;
        mPreviousPosition = position;
        setCaster(caster);

        JPH::BodyCreationSettings bodyCreationSettings
            = PhysicsSystemHelpers::makePhysicsBodySettings(new JPH::SphereShape(radius), mPosition,
                osg::Quat(1.0f, 0.0f, 0.0f, 0.0f), Layers::PROJECTILE, JPH::EMotionType::Dynamic);

        bodyCreationSettings.mEnhancedInternalEdgeRemoval = true;
        bodyCreationSettings.mMotionQuality
            = JPH::EMotionQuality::LinearCast; // Important for accurate collision detection at speed/small radius
        bodyCreationSettings.mMassPropertiesOverride.mMass = 10000.0f; // Very high mass so it cant be moved
        bodyCreationSettings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
        bodyCreationSettings.mGravityFactor = 0.0f; // Gravity managed manually
        bodyCreationSettings.mLinearDamping = 0.0f;
        bodyCreationSettings.mFriction = 0.0f;
        bodyCreationSettings.mRestitution = 0.0f;
        bodyCreationSettings.mMaxLinearVelocity = 10000.0f;

        mPhysicsBody = mTaskScheduler->createPhysicsBody(bodyCreationSettings);
        mPhysicsBody->SetUserData(reinterpret_cast<uintptr_t>(this));
        mTaskScheduler->addCollisionObject(mPhysicsBody);
    }

    Projectile::~Projectile()
    {
        if (!mActive)
            mPhysics->reportCollision(mHitPosition, mHitNormal);

        mTaskScheduler->removeCollisionObject(mPhysicsBody);
        mTaskScheduler->destroyCollisionObject(mPhysicsBody);
    }

    osg::Vec3f Projectile::getSimulationPosition() const
    {
        // OPTIMIZATION: dont do each call to here, but each update frame?
        // right now its ok because this method is called only once per projectile update
        JPH::BodyLockRead lock(mTaskScheduler->getBodyLockInterface(), getPhysicsBody());
        if (lock.Succeeded())
        {
            const JPH::Body& body = lock.GetBody();
            auto targetTransform = body.GetCenterOfMassTransform();
            return Misc::Convert::toOsg(targetTransform.GetTranslation());
        }

        return osg::Vec3f(0.0f, 0.0f, 0.0f);
    }

    bool Projectile::onContactValidate(const JPH::Body& withBody)
    {
        // if inactive while still in simulation, skip all collisions
        if (!isActive())
            return false;

        // don't hit the caster
        if (withBody.GetID() == mCasterColObj)
            return false;

        // Check if projectile or actor and if we should skip collision with them
        // typically useful for NPCs whos projectiles cannot collide with someone they arent targetting
        switch (withBody.GetObjectLayer())
        {
            case Layers::PROJECTILE:
            {
                Projectile* projectileHolder = Misc::Convert::toPointerFromUserData<Projectile>(withBody.GetUserData());
                if (projectileHolder)
                {
                    if (!projectileHolder->isActive())
                        return false;
                    if (!isValidTarget(projectileHolder->getCasterCollisionObject()))
                        return false;
                }
                break;
            }
            case Layers::ACTOR:
            {
                if (!isValidTarget(withBody.GetID()))
                    return false;
                break;
            }
        }

        // Allow the collisions and gather contacts
        return true;
    }

    void Projectile::onContactAdded(
        const JPH::Body& withBody, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings)
    {
        // If inactive while still in simulation, skip all future contacts
        if (!isActive())
            return;

        osg::Vec3f mHitPointWorld = Misc::Convert::toOsg(inManifold.mBaseOffset);
        osg::Vec3f mHitNormalWorld = Misc::Convert::toOsg(inManifold.mWorldSpaceNormal);

        // If hit projectile, set that projectile as hit with me early at same hit point in world
        // Otherwise if water, we want to set the hit water flag
        switch (withBody.GetObjectLayer())
        {
            case Layers::PROJECTILE:
            {
                Projectile* target = Misc::Convert::toPointerFromUserData<Projectile>(withBody.GetUserData());
                if (target)
                    target->hit(getPhysicsBody(), mHitPointWorld, mHitNormalWorld);
                break;
            }
            case Layers::WATER:
            {
                setHitWater();
                break;
            }
        }

        // Register hit of the target
        hit(withBody.GetID(), mHitPointWorld, mHitNormalWorld);
    }

    void Projectile::setVelocity(osg::Vec3f velocity)
    {
        PtrHolder::setVelocity(velocity);

        if (!getPhysicsBody().IsInvalid())
        {
            JPH::BodyInterface& bodyInterface = mTaskScheduler->getBodyInterface();
            bodyInterface.SetLinearVelocity(getPhysicsBody(), Misc::Convert::toJolt<JPH::Vec3>(velocity));
        }
    }

    void Projectile::hit(const JPH::BodyID target, osg::Vec3f pos, osg::Vec3f normal)
    {
        bool active = true;
        if (!mActive.compare_exchange_strong(active, false, std::memory_order_relaxed) || !active)
            return;

        std::scoped_lock lock(mMutex);
        mHitTarget = target;
        mHitPosition = pos;
        mHitNormal = normal;
    }

    MWWorld::Ptr Projectile::getTarget() const
    {
        assert(!mActive);
        void* uPtr = mTaskScheduler->getUserPointer(mHitTarget);
        if (uPtr != nullptr)
        {
            auto* target = reinterpret_cast<PtrHolder*>(uPtr);
            return target ? target->getPtr() : MWWorld::Ptr();
        }
        return MWWorld::Ptr();
    }

    MWWorld::Ptr Projectile::getCaster() const
    {
        return mCaster;
    }

    void Projectile::setCaster(const MWWorld::Ptr& caster)
    {
        mCaster = caster;
        mCasterColObj = [this, &caster]() -> const JPH::BodyID {
            JPH::BodyID invalid;
            const Actor* actor = mPhysics->getActor(caster);
            if (actor)
                return actor->getPhysicsBody();
            const Object* object = mPhysics->getObject(caster);
            if (object)
                return object->getPhysicsBody();
            return invalid;
        }();
    }

    void Projectile::setValidTargets(const std::vector<MWWorld::Ptr>& targets)
    {
        std::scoped_lock lock(mMutex);
        mValidTargets.clear();
        for (const auto& ptr : targets)
        {
            const auto* physicActor = mPhysics->getActor(ptr);
            if (physicActor)
                mValidTargets.push_back(physicActor->getPhysicsBody());
        }
    }

    bool Projectile::isValidTarget(const JPH::BodyID target) const
    {
        if (target.IsInvalid())
            return false;

        std::scoped_lock lock(mMutex);
        if (mCasterColObj == target)
            return false;

        if (mValidTargets.empty())
            return true;

        return std::any_of(
            mValidTargets.begin(), mValidTargets.end(), [target](const JPH::BodyID actor) { return target == actor; });
    }

}
