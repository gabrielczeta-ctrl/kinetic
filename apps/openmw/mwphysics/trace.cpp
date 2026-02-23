#include "trace.h"

#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include "actor.hpp"
#include "actorconvexcallback.hpp"
#include "joltfilters.hpp"
#include "joltlayers.hpp"

namespace MWPhysics
{
    ActorConvexCallback sweepHelper(const JPH::BodyID actor, const JPH::RVec3& from, const JPH::RVec3& to,
        const JPH::PhysicsSystem* physicsSystem, bool actorFilter, const int collisionMask,
        JPH::EBackFaceMode trianglesBackFaceMode)
    {
        const JPH::RVec3 correctMotion = to - from;

        JPH::BodyLockRead lock(physicsSystem->GetBodyLockInterface(), actor);
        if (lock.Succeeded())
        {
            const JPH::Body& actorBody = lock.GetBody();
            const JPH::ShapeRefC shapeRef = actorBody.GetShape();
            const JPH::RMat44& trans = actorBody.GetWorldTransform();

            JPH::RMat44 transFrom(trans);
            transFrom.SetTranslation(from);

            // Convex shapes (actors) should always check backfaces. Triangle behavior depends on query type.
            JPH::ShapeCastSettings settings;
            settings.mBackFaceModeTriangles = trianglesBackFaceMode;
            settings.mBackFaceModeConvex = JPH::EBackFaceMode::CollideWithBackFaces;

            JPH::Vec3 scale = JPH::Vec3::sReplicate(1.0f);
            JPH::RShapeCast shapeCast(
                shapeRef.GetPtr(), scale, transFrom, Misc::Convert::toJolt<JPH::Vec3>(correctMotion));

            // Inherit the actor's collision mask
            int actualMask = collisionMask;
            if (actorFilter)
                actualMask &= ~Layers::ACTOR;

            // Inherit the actor's collision group
            JPH::DefaultBroadPhaseLayerFilter broadphaseLayerFilter
                = physicsSystem->GetDefaultBroadPhaseLayerFilter(actorBody.GetObjectLayer());
            MaskedObjectLayerFilter objectLayerFilter(actualMask);

            // Ignore actor's own body
            JPH::IgnoreSingleBodyFilter bodyFilter(actor);

            const JPH::RVec3 castStart = shapeCast.mCenterOfMassStart.GetTranslation();
            ActorConvexCallback collector(
                actor, physicsSystem, castStart, castStart, 0.0, Misc::Convert::toJolt<JPH::Vec3>(correctMotion));
            lock.ReleaseLock();
            physicsSystem->GetNarrowPhaseQuery().CastShape(shapeCast, settings,
                shapeCast.mCenterOfMassStart.GetTranslation(), collector, broadphaseLayerFilter, objectLayerFilter,
                bodyFilter);

            return collector;
        }

        // Shouldn't happen, but just incase lock fails (body invalid or something bad happens)
        Log(Debug::Error) << "Unable to lock body for sweep helper";
        ActorConvexCallback collector;
        return collector;
    }

    void ActorTracer::doTrace(const JPH::BodyID actor, const osg::Vec3f& start, const osg::Vec3f& end,
        const JPH::PhysicsSystem* physicsSystem, const int collisionMask, bool attemptShortTrace)
    {
        (void)attemptShortTrace;
        const JPH::RVec3 jphStart = Misc::Convert::toJolt<JPH::RVec3>(start);
        JPH::RVec3 jphEnd = Misc::Convert::toJolt<JPH::RVec3>(end);

        const auto traceCallback = sweepHelper(
            actor, jphStart, jphEnd, physicsSystem, false, collisionMask, JPH::EBackFaceMode::IgnoreBackFaces);

        // Copy the hit data over to our trace results struct:
        if (traceCallback.hasHit())
        {
            mFraction = traceCallback.mClosestHitFraction;
            mPlaneNormal = Misc::Convert::toOsg(traceCallback.mHitNormalWorld);
            mEndPos = (end - start) * mFraction + start;
            mHitPoint = traceCallback.mHitPointWorld;
            mHitObject = traceCallback.mHitCollisionObject;
            mHitObjectLayer = traceCallback.mHitCollisionLayer;
        }
        else
        {
            // fallthrough
            mEndPos = end;
            mPlaneNormal = osg::Vec3f(0.0f, 0.0f, 1.0f);
            mFraction = 1.0f;
            mHitPoint = end;
            mHitObject = nullptr;
            mHitObjectLayer = 0;
        }
    }

    void ActorTracer::findGround(
        const Actor* actor, const osg::Vec3f& start, const osg::Vec3f& end, const JPH::PhysicsSystem* physicsSystem)
    {
        const auto traceCallback = sweepHelper(actor->getPhysicsBody(), Misc::Convert::toJolt<JPH::RVec3>(start),
            Misc::Convert::toJolt<JPH::RVec3>(end), physicsSystem, true, actor->getCollisionMask(),
            JPH::EBackFaceMode::CollideWithBackFaces);
        if (traceCallback.hasHit())
        {
            mFraction = traceCallback.mClosestHitFraction;
            mPlaneNormal = Misc::Convert::toOsg(traceCallback.mHitNormalWorld);
            mEndPos = (end - start) * mFraction + start;
        }
        else
        {
            mEndPos = end;
            mPlaneNormal = osg::Vec3f(0.0f, 0.0f, 1.0f);
            mFraction = 1.0f;
        }
    }

}
