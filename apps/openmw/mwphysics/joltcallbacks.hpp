#ifndef OPENMW_MWPHYSICS_JOLTCALLBACKS_H
#define OPENMW_MWPHYSICS_JOLTCALLBACKS_H

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <osg/Vec3f>

namespace MWPhysics
{
    class ClosestConvexResultCallback : public JPH::CastShapeCollector
    {
    public:
        JPH::BodyID mHitCollisionObject;
        float mClosestHitFraction;
        JPH::Vec3 mHitNormalWorld;
        osg::Vec3f mHitPointWorld;

        explicit ClosestConvexResultCallback(JPH::RVec3 origin)
            : mOrigin(Misc::Convert::toOsg(origin))
        {
        }

        virtual void AddHit(const JPH::ShapeCastResult& inResult) override
        {
            // Test if this collision is closer/deeper than the previous one
            float earlyOut = inResult.GetEarlyOutFraction();
            if (earlyOut < GetEarlyOutFraction())
            {
                // Update early out fraction to this hit
                UpdateEarlyOutFraction(earlyOut);

                // Get the contact properties
                mHitCollisionObject = inResult.mBodyID2;
                mHitPointWorld = mOrigin + Misc::Convert::toOsg(inResult.mContactPointOn2);
                mHitNormalWorld = -inResult.mPenetrationAxis.Normalized();
                mClosestHitFraction = inResult.mFraction;
            }
        }

        bool hasHit() const { return !mHitCollisionObject.IsInvalid(); }

    protected:
        osg::Vec3f mOrigin;
    };

    class ContactTestResultCallback : public JPH::CollideShapeCollector
    {
    public:
        explicit ContactTestResultCallback(const JPH::PhysicsSystem* physicsSystem, JPH::BodyID& me)
            : mPhysicsSystem(physicsSystem)
            , mMe(me)
        {
        }

        virtual void AddHit(const JPH::CollideShapeResult& inResult) override
        {
            // If self, skip
            if (inResult.mBodyID2 == mMe)
                return;

            JPH::BodyLockRead lock(mPhysicsSystem->GetBodyLockInterface(), inResult.mBodyID2);
            if (!lock.Succeeded())
                return;

            const JPH::Body& body = lock.GetBody();

            // Skip sensors, unlikely to happen but just incase
            if (body.IsSensor())
                return;

            // If object or actor (PtrHolder) then store hit as a result
            PtrHolder* holder = Misc::Convert::toPointerFromUserData<PtrHolder>(body.GetUserData());
            if (holder)
                mResult.emplace_back(ContactPoint{ holder->getPtr(), Misc::Convert::toOsg(inResult.mContactPointOn2),
                    Misc::Convert::toOsg(inResult.mPenetrationAxis.Normalized()) });

            // NOTE: unlike other jolt collectors, dont call UpdateEarlyOutFraction in this method as we want ALL HITS
        }

        std::vector<ContactPoint> mResult;

    protected:
        const JPH::PhysicsSystem* mPhysicsSystem;
        JPH::BodyID mMe;
    };
}

#endif
