#ifndef OPENMW_MWPHYSICS_PTRHOLDER_H
#define OPENMW_MWPHYSICS_PTRHOLDER_H

#include <memory>
#include <mutex>
#include <utility>

#include <osg/Vec3d>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include "../mwworld/ptr.hpp"

namespace MWPhysics
{
    class PtrHolder
    {
    public:
        explicit PtrHolder(const MWWorld::Ptr& ptr, const osg::Vec3f& position)
            : mPtr(ptr)
            , mPosition(position)
            , mPreviousPosition(position)
        {
        }

        virtual ~PtrHolder() = default;

        void updatePtr(const MWWorld::Ptr& updated) { mPtr = updated; }

        MWWorld::Ptr getPtr() const { return mPtr; }

        const JPH::BodyID getPhysicsBody() const
        {
            assert(mPhysicsBody != nullptr);
            if (mPhysicsBody == nullptr)
            {
                return JPH::BodyID();
            }
            return mPhysicsBody->GetID();
        }

        virtual void setVelocity(osg::Vec3f velocity) { mVelocity = velocity; }

        osg::Vec3f velocity() { return std::exchange(mVelocity, osg::Vec3f()); }

        // Assumed static by default, override if not
        virtual osg::Vec3f getSimulationPosition() const { return mPosition; }

        void setPosition(const osg::Vec3f& position)
        {
            mPreviousPosition = mPosition;
            mPosition = position;
        }

        osg::Vec3d getPosition() const { return mPosition; }

        osg::Vec3d getPreviousPosition() const { return mPreviousPosition; }

        virtual void onContactAdded(
            const JPH::Body& withBody, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings)
        {
        }

        virtual bool onContactValidate(const JPH::Body& withBody) { return true; }

    protected:
        MWWorld::Ptr mPtr;
        JPH::Body* mPhysicsBody; // NOTE: memory is managed by Jolt!
        osg::Vec3f mVelocity;
        osg::Vec3d mPosition;
        osg::Vec3d mPreviousPosition;
    };
}

#endif
