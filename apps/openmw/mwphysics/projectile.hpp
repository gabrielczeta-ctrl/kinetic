#ifndef OPENMW_MWPHYSICS_PROJECTILE_H
#define OPENMW_MWPHYSICS_PROJECTILE_H

#include <atomic>
#include <memory>
#include <mutex>

#include "ptrholder.hpp"

namespace osg
{
    class Vec3f;
}

namespace JPH
{
    class Body;
}

namespace MWPhysics
{
    class PhysicsTaskScheduler;
    class PhysicsSystem;

    class Projectile final : public PtrHolder
    {
    public:
        Projectile(const MWWorld::Ptr& caster, const osg::Vec3f& position, float radius,
            PhysicsTaskScheduler* scheduler, PhysicsSystem* physicssystem);
        ~Projectile() override;

        bool isActive() const { return mActive.load(std::memory_order_acquire); }

        MWWorld::Ptr getTarget() const;

        MWWorld::Ptr getCaster() const;
        void setCaster(const MWWorld::Ptr& caster);
        const JPH::BodyID getCasterCollisionObject() const { return mCasterColObj; }

        void setHitWater() { mHitWater = true; }

        bool getHitWater() const { return mHitWater.load(std::memory_order_acquire); }

        void hit(const JPH::BodyID target, osg::Vec3f pos, osg::Vec3f normal);

        void setValidTargets(const std::vector<MWWorld::Ptr>& targets);
        bool isValidTarget(const JPH::BodyID target) const;

        osg::Vec3f getHitPosition() const { return mHitPosition; }
        osg::Vec3f getHitNormal() const { return mHitNormal; }

        void onContactAdded(const JPH::Body& withBody, const JPH::ContactManifold& inManifold,
            JPH::ContactSettings& ioSettings) override;
        bool onContactValidate(const JPH::Body& withBody) override;
        void setVelocity(osg::Vec3f velocity) override;

        osg::Vec3f getSimulationPosition() const override;

    private:
        std::atomic<bool> mHitWater;
        std::atomic<bool> mActive;
        MWWorld::Ptr mCaster;
        JPH::BodyID mCasterColObj;
        JPH::BodyID mHitTarget;
        osg::Vec3f mHitPosition;
        osg::Vec3f mHitNormal;

        std::vector<JPH::BodyID> mValidTargets;

        mutable std::mutex mMutex;

        PhysicsSystem* mPhysics;
        PhysicsTaskScheduler* mTaskScheduler;

        Projectile(const Projectile&);
        Projectile& operator=(const Projectile&);
    };

}

#endif
