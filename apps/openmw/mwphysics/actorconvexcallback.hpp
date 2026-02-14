#ifndef OPENMW_MWPHYSICS_ACTORCONVEXCALLBACK_H
#define OPENMW_MWPHYSICS_ACTORCONVEXCALLBACK_H

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>

#include "../mwrender/joltdebugdraw.hpp"

#include "joltlayers.hpp"
#include "physicssystem.hpp"

namespace MWPhysics
{
    class ActorConvexCallback : public JPH::CastShapeCollector
    {
    public:
        ActorConvexCallback()
            : mClosestHitFraction(0.0f)
            , mMinCollisionDot(0.0f)
            , mHitCollisionLayer(0)
            , mHitPointWorld(JPH::RVec3(0.0, 0.0, 0.0))
            , mHitNormalWorld(JPH::Vec3(0.0, 0.0, 0.0))
            , mMotion(JPH::Vec3(0.0, 0.0, 0.0))
            , mPhysicsSystem(nullptr)
            , mOrigin(JPH::RVec3(0.0, 0.0, 0.0))
        {
        }

        ActorConvexCallback(const JPH::BodyID actor, const JPH::PhysicsSystem* inPhysicsSystem, JPH::RVec3 origin,
            float minCollisionDot, JPH::Vec3 motion)
            : mMinCollisionDot(minCollisionDot)
            , mMe(actor)
            , mHitCollisionLayer(0)
            , mHitPointWorld(JPH::RVec3(0.0, 0.0, 0.0))
            , mHitNormalWorld(JPH::RVec3(0.0, 0.0, 0.0))
            , mMotion(motion)
            , mPhysicsSystem(inPhysicsSystem)
            , mOrigin(origin)
        {
        }

        virtual void AddHit(const JPH::ShapeCastResult& inResult) override
        {
            // If self, skip
            if (inResult.mBodyID2 == mMe)
            {
                return;
            }

            JPH::BodyLockRead lock(mPhysicsSystem->GetBodyLockInterfaceNoLock(), inResult.mBodyID2);
            if (!lock.Succeeded())
                return;

            const JPH::Body* body = &lock.GetBody();

            if (body->IsSensor())
                return;

            JPH::BodyLockRead myLock(mPhysicsSystem->GetBodyLockInterfaceNoLock(), mMe);
            assert(myLock.Succeeded());
            if (!myLock.Succeeded())
                return;

            const JPH::Body* myBody = &myLock.GetBody();

            const JPH::ObjectLayer collisionGroup = body->GetObjectLayer();

            JPH::Vec3 hitNormalWorld = -inResult.mPenetrationAxis.Normalized();

            // override data for actor-actor collisions
            // vanilla Morrowind seems to make overlapping actors collide as though they are both cylinders with a
            // diameter of the distance between them For some reason this doesn't work as well as it should when using
            // capsules, but it still helps a lot.
            float earlyOut = inResult.GetEarlyOutFraction();
            if (collisionGroup == Layers::ACTOR)
            {
                bool isOverlapping = inResult.mPenetrationDepth != 0.0f;
                if (isOverlapping)
                {
                    auto originA = Misc::Convert::toOsg(myBody->GetCenterOfMassTransform().GetTranslation());
                    auto originB = Misc::Convert::toOsg(body->GetCenterOfMassTransform().GetTranslation());
                    osg::Vec3f motion = Misc::Convert::toOsg(mMotion);
                    osg::Vec3f normal = (originA - originB);
                    normal.z() = 0;
                    motion.z() = 0;

                    // Prefer the vanilla-like horizontal actor-vs-actor response when moving into another actor.
                    // If actors are vertically stacked (normal almost zero), fall back to the standard contact normal
                    // handling below to avoid dropping contacts.
                    if (normal.length2() > 0.0001f && motion.length2() > 0.0001f)
                    {
                        normal.normalize();
                        if (normal * motion < 0.0f)
                        {
                            // Get the contact properties
                            mHitCollisionObject = body;
                            mHitCollisionLayer = collisionGroup;
                            mHitPointWorld = mOrigin + inResult.mContactPointOn2;
                            mHitNormalWorld = Misc::Convert::toJolt<JPH::Vec3>(normal);
                            mClosestHitFraction = 0.0f;
                            ForceEarlyOut();
                            return;
                        }
                    }
                }
            }

            if (collisionGroup == Layers::PROJECTILE)
                return;

            // dot product of the motion vector against the collision contact normal
            const float approachDot = -mMotion.Dot(hitNormalWorld);
            if (approachDot <= mMinCollisionDot)
                return;

            // Test if this collision is closer/deeper than the previous one
            if (earlyOut < GetEarlyOutFraction())
            {
                // Update early out fraction to this hit
                UpdateEarlyOutFraction(earlyOut);

                // Get the contact properties
                mHitCollisionObject = body;
                mHitCollisionLayer = collisionGroup;
                mHitPointWorld = mOrigin + inResult.mContactPointOn2;
                mHitNormalWorld = hitNormalWorld;
                mClosestHitFraction = inResult.mFraction;
            }
        }

        bool hasHit() const { return mHitCollisionObject != nullptr; }

        float mClosestHitFraction;
        const float mMinCollisionDot;
        const JPH::BodyID mMe;
        const JPH::Body* mHitCollisionObject = nullptr;

        JPH::ObjectLayer mHitCollisionLayer;
        JPH::RVec3 mHitPointWorld;
        JPH::Vec3 mHitNormalWorld;
        JPH::Vec3 mMotion;

    private:
        const JPH::PhysicsSystem* mPhysicsSystem;
        JPH::RVec3 mOrigin;
    };
}

#endif
