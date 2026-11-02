#ifndef OPENMW_MWPHYSICS_ACTORCONVEXCALLBACK_H
#define OPENMW_MWPHYSICS_ACTORCONVEXCALLBACK_H

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>

#include <osg/Vec3f>

#include <cstdint>

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
            , mHitPointWorld(0.0f, 0.0f, 0.0f)
            , mHitNormalWorld(JPH::Vec3(0.0, 0.0, 0.0))
            , mMotion(JPH::Vec3(0.0, 0.0, 0.0))
            , mPhysicsSystem(nullptr)
            , mSelfCenterWorld(0.0f, 0.0f, 0.0f)
            , mOrigin(0.0f, 0.0f, 0.0f)
        {
        }

        ActorConvexCallback(const JPH::BodyID actor, const JPH::PhysicsSystem* inPhysicsSystem,
            JPH::RVec3 selfCenterWorld, JPH::RVec3 origin, float minCollisionDot, JPH::Vec3 motion)
            : mMinCollisionDot(minCollisionDot)
            , mMe(actor)
            , mHitCollisionLayer(0)
            , mHitPointWorld(0.0f, 0.0f, 0.0f)
            , mHitNormalWorld(JPH::Vec3(0.0f, 0.0f, 0.0f))
            , mMotion(motion)
            , mPhysicsSystem(inPhysicsSystem)
            , mSelfCenterWorld(Misc::Convert::toOsg(selfCenterWorld))
            , mOrigin(Misc::Convert::toOsg(origin))
        {
        }

        virtual void AddHit(const JPH::ShapeCastResult& inResult) override
        {
            // If self, skip
            if (inResult.mBodyID2 == mMe)
            {
                return;
            }

            // NarrowPhaseQuery releases its candidate-body lock before calling
            // the collector. Lock explicitly and copy all metadata needed by
            // the movement solver; never let a Jolt Body pointer escape.
            JPH::BodyLockRead lock(mPhysicsSystem->GetBodyLockInterface(), inResult.mBodyID2);
            if (!lock.Succeeded())
                return;

            const JPH::Body& body = lock.GetBody();

            if (body.IsSensor())
                return;

            const JPH::ObjectLayer collisionGroup = body.GetObjectLayer();
            const std::uint64_t userData = body.GetUserData();

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
                    auto originA = mSelfCenterWorld;
                    auto originB = Misc::Convert::toOsg(body.GetCenterOfMassTransform().GetTranslation());
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
                            mHitCollisionObject = inResult.mBodyID2;
                            mHitCollisionLayer = collisionGroup;
                            mHitUserData = userData;
                            mHitPointWorld = mOrigin + Misc::Convert::toOsg(inResult.mContactPointOn2);
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
                mHitCollisionObject = inResult.mBodyID2;
                mHitCollisionLayer = collisionGroup;
                mHitUserData = userData;
                mHitPointWorld = mOrigin + Misc::Convert::toOsg(inResult.mContactPointOn2);
                mHitNormalWorld = hitNormalWorld;
                mClosestHitFraction = inResult.mFraction;
            }
        }

        bool hasHit() const { return !mHitCollisionObject.IsInvalid(); }

        float mClosestHitFraction;
        const float mMinCollisionDot;
        const JPH::BodyID mMe;
        JPH::BodyID mHitCollisionObject;

        JPH::ObjectLayer mHitCollisionLayer;
        std::uint64_t mHitUserData = 0;
        osg::Vec3f mHitPointWorld;
        JPH::Vec3 mHitNormalWorld;
        JPH::Vec3 mMotion;

    private:
        const JPH::PhysicsSystem* mPhysicsSystem;
        osg::Vec3f mSelfCenterWorld;
        osg::Vec3f mOrigin;
    };
}

#endif
