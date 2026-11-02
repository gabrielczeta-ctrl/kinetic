#ifndef OENGINE_PHYSICS_ACTOR_TRACE_H
#define OENGINE_PHYSICS_ACTOR_TRACE_H

#include <osg/Vec3f>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

#include <cstdint>

namespace JPH
{
    class PhysicsSystem;
}

namespace MWPhysics
{
    class Actor;

    struct ActorTracer
    {
        osg::Vec3f mEndPos;
        osg::Vec3f mPlaneNormal;
        osg::Vec3f mHitPoint;
        JPH::ObjectLayer mHitObjectLayer;
        JPH::BodyID mHitObject;
        std::uint64_t mHitObjectUserData = 0;

        float mFraction;

        void doTrace(const JPH::BodyID actor, const osg::Vec3f& start, const osg::Vec3f& end,
            const JPH::PhysicsSystem* world, const int collisionMask, bool attemptShortTrace = false);
        void findGround(const Actor* actor, const osg::Vec3f& start, const osg::Vec3f& end,
            const JPH::PhysicsSystem* physicsSystem);
    };
}

#endif
