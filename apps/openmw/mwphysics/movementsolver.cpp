#include "movementsolver.hpp"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>

#include <components/debug/debuglog.hpp>
#include <components/esm3/loadgmst.hpp>
#include <components/misc/convert.hpp>

#include "../mwbase/environment.hpp"

#include "../mwworld/esmstore.hpp"

#include "actor.hpp"
#include "constants.hpp"
#include "joltfilters.hpp"
#include "joltlayers.hpp"
#include "object.hpp"
#include "physicssystem.hpp"
#include "projectile.hpp"
#include "stepper.hpp"
#include "trace.h"

#include <cmath>

namespace MWPhysics
{
    static bool isActor(const JPH::Body* obj)
    {
        assert(obj);
        return obj->GetObjectLayer() == Layers::ACTOR;
    }

    namespace
    {
        // Collector that checks if there is anything in the way while switching to inShape
        class ContactCollectionCallback : public JPH::CollideShapeCollector
        {
        public:
            explicit ContactCollectionCallback(const osg::Vec3f& velocity)
                : mVelocity(Misc::Convert::toJolt<JPH::Vec3>(velocity))
            {
            }

            virtual void AddHit(const JPH::CollideShapeResult& inResult) override
            {
                // ignore overlap if we're moving in the same direction as it would push us out (don't change this to
                // >=, that would break detection when not moving)
                JPH::Vec3 worldSpaceNormal = -inResult.mPenetrationAxis.Normalized();
                if (worldSpaceNormal.Dot(mVelocity) > 0.0)
                    return;

                auto delta = worldSpaceNormal * inResult.mPenetrationDepth;
                mContactSum += delta;
                mMaxX = std::max(std::abs(delta.GetX()), mMaxX);
                mMaxY = std::max(std::abs(delta.GetY()), mMaxY);
                mMaxZ = std::max(std::abs(delta.GetZ()), mMaxZ);
                if (-inResult.mPenetrationDepth < mDistance)
                {
                    mDistance = -inResult.mPenetrationDepth;
                    mNormal = worldSpaceNormal;
                    mDelta = delta;
                }

                float early_out = inResult.GetEarlyOutFraction();
                if (early_out < GetEarlyOutFraction())
                {
                    // Update early out fraction to this hit
                    UpdateEarlyOutFraction(early_out);
                }
            }

            float mMaxX = 0.0;
            float mMaxY = 0.0;
            float mMaxZ = 0.0;
            JPH::Vec3 mContactSum{ 0.0, 0.0, 0.0 };
            JPH::Vec3 mNormal{ 0.0, 0.0, 0.0 }; // points towards "me"
            JPH::Vec3 mDelta{ 0.0, 0.0, 0.0 }; // points towards "me"
            float mDistance = 0.0; // negative or zero

        protected:
            JPH::Vec3 mVelocity;
        };
    }

    osg::Vec3f MovementSolver::traceDown(const MWWorld::Ptr& ptr, const osg::Vec3f& position, Actor* actor,
        JPH::PhysicsSystem* physicsSystem, float maxHeight)
    {
        osg::Vec3f offset = actor->getCollisionObjectPosition() - ptr.getRefData().getPosition().asVec3();

        ActorTracer tracer;
        tracer.findGround(actor, position + offset, position + offset - osg::Vec3f(0, 0, maxHeight), physicsSystem);
        if (tracer.mFraction >= 1.0f)
        {
            actor->setOnGround(false);
            return position;
        }

        actor->setOnGround(true);

        // Check if we actually found a valid spawn point (use an infinitely thin ray this time).
        // Required for some broken door destinations in Morrowind.esm, where the spawn point
        // intersects with other geometry if the actor's base is taken into account
        JPH::RVec3 rayOrigin = Misc::Convert::toJolt<JPH::RVec3>(position);
        JPH::RRayCast ray(rayOrigin, JPH::Vec3(0.0f, 0.0f, -maxHeight));

        MultiBroadPhaseLayerFilter broadphaseLayerFilter({ BroadPhaseLayers::WORLD });
        MultiObjectLayerFilter objectLayerFilter({ Layers::WORLD, Layers::HEIGHTMAP });

        // It is important to ignore backfaces for this check, as all actor collision should
        JPH::RayCastSettings settings;
        settings.SetBackFaceMode(JPH::EBackFaceMode::IgnoreBackFaces);

        // Cast ray and return closest hit
        class TraceHitCollector : public JPH::CastRayCollector
        {
        public:
            virtual void AddHit(const JPH::RayCastResult& inResult) override
            {
                mFraction = inResult.mFraction;
                mSubShapeID2 = inResult.mSubShapeID2;
                mBodyID = inResult.mBodyID;
                mHit = true;
                ForceEarlyOut(); // Only collect a single hit
            }

            bool mHit = false;
            float mFraction = 1.0f;
            JPH::BodyID mBodyID;
            JPH::SubShapeID mSubShapeID2;
        };

        TraceHitCollector collector;
        physicsSystem->GetNarrowPhaseQuery().CastRay(
            ray, settings, collector, broadphaseLayerFilter, objectLayerFilter);

        if (collector.mHit)
        {
            JPH::RVec3 hitPointWorld = ray.GetPointOnRay(collector.mFraction);
            if (((Misc::Convert::toOsg(hitPointWorld) - tracer.mEndPos + offset).length2() > 35 * 35
                    || !isWalkableSlope(tracer.mPlaneNormal)))
            {
                JPH::BodyLockRead lock(physicsSystem->GetBodyLockInterface(), collector.mBodyID);
                if (lock.Succeeded())
                {
                    const JPH::Body& hitBody = lock.GetBody();
                    const JPH::Vec3 normal = hitBody.GetWorldSpaceSurfaceNormal(collector.mSubShapeID2, hitPointWorld);
                    actor->setOnSlope(!isWalkableSlope(Misc::Convert::toOsg(normal)));
                    return Misc::Convert::toOsg(hitPointWorld) + osg::Vec3f(0.f, 0.f, sGroundOffset);
                }
            }
        }

        actor->setOnSlope(!isWalkableSlope(tracer.mPlaneNormal));

        return tracer.mEndPos - offset + osg::Vec3f(0.f, 0.f, sGroundOffset);
    }

    void MovementSolver::move(
        ActorFrameData& actor, float time, const JPH::PhysicsSystem* physicsSystem, const WorldFrameData& worldData)
    {
        const int collisionMask = actor.mCollisionMask;

        // Reset per-frame data
        actor.mWalkingOnWater = false;
        // Anything to collide with?
        if (actor.mSkipCollisionDetection)
        {
            actor.mPosition += (osg::Quat(actor.mRotation.x(), osg::Vec3f(-1, 0, 0))
                                   * osg::Quat(actor.mRotation.y(), osg::Vec3f(0, 0, -1)))
                * actor.mMovement * time;
            return;
        }

        // Adjust for collision mesh offset relative to actor's "location"
        // (doTrace doesn't take local/interior collision shape translation into account, so we have to do it on our
        // own) for compatibility with vanilla assets, we have to derive this from the vertical half extent instead of
        // from internal hull translation if not for this hack, the "correct" collision hull position would be
        // physicActor->getScaledMeshTranslation()
        actor.mPosition.z() += actor.mHalfExtentsZ; // vanilla-accurate

        float swimlevel = actor.mSwimLevel + actor.mHalfExtentsZ;

        ActorTracer tracer;

        osg::Vec3f velocity;

        // Dead and paralyzed actors underwater will float to the surface,
        // if the CharacterController tells us to do so
        if (actor.mMovement.z() > 0 && actor.mInert && actor.mPosition.z() < swimlevel)
        {
            velocity = osg::Vec3f(0, 0, 1) * 25;
        }
        else if (actor.mPosition.z() < swimlevel || actor.mFlying)
        {
            velocity = (osg::Quat(actor.mRotation.x(), osg::Vec3f(-1, 0, 0))
                           * osg::Quat(actor.mRotation.y(), osg::Vec3f(0, 0, -1)))
                * actor.mMovement;
        }
        else
        {
            velocity = (osg::Quat(actor.mRotation.y(), osg::Vec3f(0, 0, -1))) * actor.mMovement;

            if ((velocity.z() > 0.f && actor.mIsOnGround && !actor.mIsOnSlope)
                || (velocity.z() > 0.f && velocity.z() + actor.mInertia.z() <= -velocity.z() && actor.mIsOnSlope))
                actor.mInertia = velocity;
            else if (!actor.mIsOnGround || actor.mIsOnSlope)
                velocity = velocity + actor.mInertia;
        }

        // Now that we have the effective movement vector, apply wind forces to it
        if (worldData.mIsInStorm && velocity.length() > 0)
        {
            osg::Vec3f stormDirection = worldData.mStormDirection;
            float angleDegrees = osg::RadiansToDegrees(
                std::acos(stormDirection * velocity / (stormDirection.length() * velocity.length())));
            static const float fStromWalkMult = MWBase::Environment::get()
                                                    .getESMStore()
                                                    ->get<ESM::GameSetting>()
                                                    .find("fStromWalkMult")
                                                    ->mValue.getFloat();
            velocity *= 1.f - (fStromWalkMult * (angleDegrees / 180.f));
        }

        Stepper stepper(physicsSystem, actor.mPhysicsBody);
        osg::Vec3f origVelocity = velocity;
        osg::Vec3f newPosition = actor.mPosition;
        /*
         * A loop to find newPosition using tracer, if successful different from the starting position.
         * nextpos is the local variable used to find potential newPosition, using velocity and remainingTime
         * The initial velocity was set earlier (see above).
         */
        float remainingTime = time;

        int numTimesSlid = 0;
        osg::Vec3f lastSlideNormal(0, 0, 1);
        osg::Vec3f lastSlideNormalFallback(0, 0, 1);
        bool forceGroundTest = false;

        for (int iterations = 0; iterations < sMaxIterations && remainingTime > 0.0001f; ++iterations)
        {
            osg::Vec3f nextpos = newPosition + velocity * remainingTime;
            bool underwater = newPosition.z() < swimlevel;

            // If not able to fly, don't allow to swim up into the air
            if (!actor.mFlying && nextpos.z() > swimlevel && underwater)
            {
                const osg::Vec3f down(0, 0, -1);
                velocity = reject(velocity, down);
                // NOTE: remainingTime is unchanged before the loop continues
                continue; // velocity updated, calculate nextpos again
            }

            if ((newPosition - nextpos).length2() > 0.0001)
            {
                // trace to where character would go if there were no obstructions
                tracer.doTrace(
                    actor.mPhysicsBody, newPosition, nextpos, physicsSystem, collisionMask, actor.mIsOnGround);

                // check for obstructions
                if (tracer.mFraction >= 1.0f)
                {
                    newPosition = tracer.mEndPos; // ok to move, so set newPosition
                    break;
                }
            }
            else
            {
                // The current position and next position are nearly the same, so just exit.
                // Since we aren't performing any collision detection, we want to reject the next
                // position, so that we don't slowly move inside another object.
                break;
            }

            bool seenGround = !actor.mFlying && !underwater
                && ((actor.mIsOnGround && !actor.mIsOnSlope) || isWalkableSlope(tracer.mPlaneNormal));

            // We hit something. Check if we can step up.
            float hitHeight = tracer.mHitPoint.z() - tracer.mEndPos.z() + actor.mHalfExtentsZ;
            osg::Vec3f oldPosition = newPosition;
            bool usedStepLogic = false;

            if (!isActor(tracer.mHitObject))
            {
                if (hitHeight < Constants::sStepSizeUp)
                {
                    // Try to step up onto it.
                    // NOTE: this modifies newPosition and velocity on its own if successful
                    usedStepLogic = stepper.step(
                        newPosition, velocity, remainingTime, seenGround, iterations == 0, collisionMask);
                }

                if (tracer.mHitObject->GetObjectLayer() != Layers::WATER)
                {
                    Object* hitObject = Misc::Convert::toPointerFromUserData<Object>(tracer.mHitObject->GetUserData());
                    if (hitObject != nullptr)
                    {
                        hitObject->addCollision(
                            actor.mIsPlayer ? ScriptedCollisionType_Player : ScriptedCollisionType_Actor);
                    }
                }
            }
            if (usedStepLogic)
            {
                if (actor.mIsAquatic && newPosition.z() + actor.mHalfExtentsZ > actor.mWaterlevel)
                    newPosition = oldPosition;
                else if (!actor.mFlying && actor.mPosition.z() >= swimlevel)
                    forceGroundTest = true;
            }
            else
            {
                // Can't step up, so slide against what we ran into
                remainingTime *= (1.0f - tracer.mFraction);

                auto planeNormal = tracer.mPlaneNormal;
                // need to know the unadjusted normal to handle certain types of seams properly
                const auto origPlaneNormal = planeNormal;

                // If we touched the ground this frame, and whatever we ran into is a wall of some sort,
                // pretend that its collision normal is pointing horizontally
                // (fixes snagging on slightly downward-facing walls, and crawling up the bases of very steep walls
                // because of the collision margin)
                if (seenGround && !isWalkableSlope(planeNormal) && planeNormal.z() != 0)
                {
                    planeNormal.z() = 0;
                    planeNormal.normalize();
                }

                // Move up to what we ran into (with a bit of a collision margin)
                if ((newPosition - tracer.mEndPos).length2() > sCollisionMargin * sCollisionMargin)
                {
                    auto direction = velocity;
                    direction.normalize();
                    newPosition = tracer.mEndPos;
                    newPosition -= direction * sCollisionMargin;
                }

                osg::Vec3f newVelocity = (velocity * planeNormal <= 0.0) ? reject(velocity, planeNormal) : velocity;
                bool usedSeamLogic = false;

                // check for the current and previous collision planes forming an acute angle; slide along the seam if
                // they do for this, we want to use the original plane normal, or else certain types of geometry will
                // snag
                if (numTimesSlid > 0)
                {
                    auto dotA = lastSlideNormal * origPlaneNormal;
                    auto dotB = lastSlideNormalFallback * origPlaneNormal;
                    if (numTimesSlid <= 1) // ignore fallback normal if this is only the first or second slide
                        dotB = 1.0;
                    if (dotA <= 0.0 || dotB <= 0.0)
                    {
                        osg::Vec3f bestNormal = lastSlideNormal;
                        // use previous-to-previous collision plane if it's acute with current plane but actual previous
                        // plane isn't
                        if (dotB < dotA)
                        {
                            bestNormal = lastSlideNormalFallback;
                            lastSlideNormal = lastSlideNormalFallback;
                        }

                        auto constraintVector = bestNormal ^ origPlaneNormal; // cross product
                        if (constraintVector.length2() > 0) // only if it's not zero length
                        {
                            constraintVector.normalize();
                            newVelocity = project(velocity, constraintVector);

                            // version of surface rejection for acute crevices/seams
                            auto averageNormal = bestNormal + origPlaneNormal;
                            averageNormal.normalize();
                            tracer.doTrace(actor.mPhysicsBody, newPosition,
                                newPosition + averageNormal * (sCollisionMargin * 2.0), physicsSystem, collisionMask);
                            newPosition = (newPosition + tracer.mEndPos) / 2.0;

                            usedSeamLogic = true;
                        }
                    }
                }
                // otherwise just keep the normal vector rejection

                // move away from the collision plane slightly, if possible
                // this reduces getting stuck in some concave geometry, like the gaps above the railings in some
                // ald'ruhn buildings this is different from the normal collision margin, because the normal collision
                // margin is along the movement path, but this is along the collision normal
                if (!usedSeamLogic)
                {
                    tracer.doTrace(actor.mPhysicsBody, newPosition,
                        newPosition + planeNormal * (sCollisionMargin * 2.0), physicsSystem, collisionMask);
                    newPosition = (newPosition + tracer.mEndPos) / 2.0;
                }

                // short circuit if we went backwards, but only if it was mostly horizontal and we're on the ground
                if (seenGround && newVelocity * origVelocity <= 0.0f)
                {
                    auto perpendicular = newVelocity ^ origVelocity;
                    if (perpendicular.length2() > 0.0f)
                    {
                        perpendicular.normalize();
                        if (std::abs(perpendicular.z()) > 0.7071f)
                            break;
                    }
                }

                // Do not allow sliding up steep slopes if there is gravity.
                // The purpose of this is to prevent air control from letting you slide up tall, unwalkable slopes.
                // For that purpose, it is not necessary to do it when trying to slide along acute seams/crevices (i.e.
                // usedSeamLogic) and doing so would actually break air control in some situations where vanilla allows
                // air control. Vanilla actually allows you to slide up slopes as long as you're in the "walking"
                // animation, which can be true even in the air, so allowing this for seams isn't a compatibility break.
                if (newPosition.z() >= swimlevel && !actor.mFlying && !isWalkableSlope(planeNormal) && !usedSeamLogic)
                    newVelocity.z() = std::min(newVelocity.z(), velocity.z());

                numTimesSlid += 1;
                lastSlideNormalFallback = lastSlideNormal;
                lastSlideNormal = origPlaneNormal;
                velocity = newVelocity;
            }
        }

        bool isOnGround = false;
        bool isOnSlope = false;
        if (forceGroundTest || (actor.mInertia.z() <= 0.f && newPosition.z() >= swimlevel))
        {
            auto dropDistance = 2 * sGroundOffset + (actor.mIsOnGround ? sStepSizeDown : 0);
            osg::Vec3f to = newPosition - osg::Vec3f(0, 0, dropDistance);
            tracer.doTrace(actor.mPhysicsBody, newPosition, to, physicsSystem, collisionMask, actor.mIsOnGround);
            if (tracer.mFraction < 1.0f)
            {
                if (!isActor(tracer.mHitObject))
                {
                    isOnGround = true;
                    isOnSlope = !isWalkableSlope(tracer.mPlaneNormal);
                    actor.mStandingOn = tracer.mHitObject->GetID();

                    if (tracer.mHitObject->GetObjectLayer() == Layers::WATER)
                        actor.mWalkingOnWater = true;
                    if (!actor.mFlying && !isOnSlope)
                    {
                        if (tracer.mFraction * dropDistance > sGroundOffset)
                            newPosition.z() = tracer.mEndPos.z() + sGroundOffset;
                        else
                        {
                            newPosition.z() = tracer.mEndPos.z();
                            tracer.doTrace(actor.mPhysicsBody, newPosition,
                                newPosition + osg::Vec3f(0, 0, 2 * sGroundOffset), physicsSystem, collisionMask);
                            newPosition = (newPosition + tracer.mEndPos) / 2.0;
                        }
                    }
                }
                else
                {
                    // Vanilla allows actors to float on top of other actors. Do not push them off.
                    if (!actor.mFlying && isWalkableSlope(tracer.mPlaneNormal)
                        && tracer.mEndPos.z() + sGroundOffset <= newPosition.z())
                        newPosition.z() = tracer.mEndPos.z() + sGroundOffset;

                    isOnGround = false;
                }
            }
            // forcibly treat stuck actors as if they're on flat ground because buggy collisions when inside of things
            // can/will break ground detection
            if (actor.mStuckFrames > 0)
            {
                isOnGround = true;
                isOnSlope = false;
            }
        }

        if ((isOnGround && !isOnSlope) || newPosition.z() < swimlevel || actor.mFlying)
            actor.mInertia = osg::Vec3f(0.f, 0.f, 0.f);
        else
        {
            actor.mInertia.z() -= time * Constants::GravityConst * Constants::UnitsPerMeter;
            if (actor.mInertia.z() < 0)
                actor.mInertia.z() *= actor.mSlowFall;
            if (actor.mSlowFall < 1.f)
            {
                actor.mInertia.x() *= actor.mSlowFall;
                actor.mInertia.y() *= actor.mSlowFall;
            }
        }
        actor.mIsOnGround = isOnGround;
        actor.mIsOnSlope = isOnSlope;

        actor.mPosition = newPosition;
        // remove what was added earlier in compensating for doTrace not taking interior transformation into account
        actor.mPosition.z() -= actor.mHalfExtentsZ; // vanilla-accurate
    }

    JPH::Vec3 addMarginToDelta(JPH::Vec3 delta)
    {
        if (delta.LengthSq() == 0.0)
            return delta;
        return delta + delta.Normalized() * sCollisionMargin;
    }

    void MovementSolver::unstuck(ActorFrameData& actor, JPH::PhysicsSystem* physicsSystem)
    {
        if (actor.mSkipCollisionDetection) // noclipping/tcl
            return;

        if (actor.mMovement.length2() == 0) // no AI nor player attempted to move, current position is assumed correct
            return;

        auto tempPosition = actor.mPosition;

        if (actor.mStuckFrames >= 10)
        {
            if ((actor.mLastStuckPosition - actor.mPosition).length2() < 100)
                return;
            else
            {
                actor.mStuckFrames = 0;
                actor.mLastStuckPosition = { 0, 0, 0 };
            }
        }

        // use vanilla-accurate collision hull position hack (do same hitbox offset hack as movement solver)
        // if vanilla compatibility didn't matter, the "correct" collision hull position would be
        // physicActor->getScaledMeshTranslation()
        const auto verticalHalfExtent = osg::Vec3f(0.0, 0.0, actor.mHalfExtentsZ);

        // use a 3d approximation of the movement vector to better judge player intent
        auto velocity = (osg::Quat(actor.mRotation.x(), osg::Vec3f(-1, 0, 0))
                            * osg::Quat(actor.mRotation.y(), osg::Vec3f(0, 0, -1)))
            * actor.mMovement;
        // try to pop outside of the world before doing anything else if we're inside of it
        if (!actor.mIsOnGround || actor.mIsOnSlope)
            velocity += actor.mInertia;

        // because of the internal collision box offset hack, and the fact that we're moving the collision box manually,
        // we need to replicate part of the collision box's transform process from scratch
        osg::Vec3f refPosition = tempPosition + verticalHalfExtent;
        osg::Vec3f goodPosition = refPosition;

        JPH::BodyLockRead lock(physicsSystem->GetBodyLockInterface(), actor.mPhysicsBody);
        if (lock.Succeeded())
        {
            const JPH::Body& actorBody = lock.GetBody();
            const JPH::ShapeRefC shape = actorBody.GetShape();
            const JPH::RMat44& trans = actorBody.GetWorldTransform();

            JPH::RMat44 oldTransformJolt(trans);
            JPH::RMat44 newTransformJolt(oldTransformJolt);
            const JPH::Vec3 scale = JPH::Vec3::sReplicate(1.0f);

            // Create a mask from the actor, excluding projectiles.
            // Keep actor collisions enabled here so overlap recovery can separate stacked actors.
            int collisionMask = actor.mCollisionMask;
            collisionMask = collisionMask & ~Layers::PROJECTILE;

            // Filter out layers
            JPH::DefaultBroadPhaseLayerFilter broadphaseLayerFilter
                = physicsSystem->GetDefaultBroadPhaseLayerFilter(Layers::ACTOR);
            MaskedObjectLayerFilter objectLayerFilter(collisionMask);
            lock.ReleaseLock();

            JPH::BodyInterface& bodyInterface = physicsSystem->GetBodyInterface();

            auto gatherContacts = [&](JPH::Vec3 newOffset) -> ContactCollectionCallback {
                goodPosition = refPosition + Misc::Convert::toOsg(addMarginToDelta(newOffset));
                newTransformJolt.SetTranslation(Misc::Convert::toJolt<JPH::RVec3>(goodPosition));
                bodyInterface.SetPosition(
                    actor.mPhysicsBody, newTransformJolt.GetTranslation(), JPH::EActivation::Activate);

                // Collide with all edges, dont collect face data and ignore backfaces
                JPH::CollideShapeSettings settings;
                settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideWithAll;
                settings.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;
                settings.mCollectFacesMode = JPH::ECollectFacesMode::NoFaces;

                // Ignore actors own body
                JPH::IgnoreSingleBodyFilter bodyFilter(actor.mPhysicsBody);

                // Query against actor shape
                ContactCollectionCallback ioCollector(velocity);
                physicsSystem->GetNarrowPhaseQuery().CollideShape(shape, scale, newTransformJolt, settings,
                    JPH::RVec3::sZero(), ioCollector, broadphaseLayerFilter, objectLayerFilter, bodyFilter);
                return ioCollector;
            };

            // check whether we're inside the world with our collision box with manually-derived offset
            auto contactCallback = gatherContacts({ 0.0, 0.0, 0.0 });
            if (contactCallback.mDistance < -sAllowedPenetration)
            {
                ++actor.mStuckFrames;
                actor.mLastStuckPosition = actor.mPosition;
                // we are; try moving it out of the world
                auto positionDelta = contactCallback.mContactSum;
                // limit rejection delta to the largest known individual rejections
                if (std::abs(positionDelta.GetX()) > contactCallback.mMaxX)
                    positionDelta *= contactCallback.mMaxX / std::abs(positionDelta.GetX());
                if (std::abs(positionDelta.GetY()) > contactCallback.mMaxY)
                    positionDelta *= contactCallback.mMaxY / std::abs(positionDelta.GetY());
                if (std::abs(positionDelta.GetZ()) > contactCallback.mMaxZ)
                    positionDelta *= contactCallback.mMaxZ / std::abs(positionDelta.GetZ());

                auto contactCallback2 = gatherContacts(positionDelta);
                // successfully moved further out from contact (does not have to be in open space, just less inside of
                // things)
                if (contactCallback2.mDistance > contactCallback.mDistance)
                    tempPosition = goodPosition - verticalHalfExtent;
                // try again but only upwards (fixes some bad coc floors)
                else
                {
                    // upwards-only offset
                    auto contactCallback3 = gatherContacts({ 0.0, 0.0, std::abs(positionDelta.GetZ()) });
                    // success
                    if (contactCallback3.mDistance > contactCallback.mDistance)
                        tempPosition = goodPosition - verticalHalfExtent;
                    else
                    // try again but fixed distance up
                    {
                        auto contactCallback4 = gatherContacts({ 0.0, 0.0, 10.0 });
                        // success
                        if (contactCallback4.mDistance > contactCallback.mDistance)
                            tempPosition = goodPosition - verticalHalfExtent;
                    }
                }
            }
            else
            {
                actor.mStuckFrames = 0;
                actor.mLastStuckPosition = { 0, 0, 0 };
            }

            bodyInterface.SetPosition(
                actor.mPhysicsBody, oldTransformJolt.GetTranslation(), JPH::EActivation::Activate);
            actor.mPosition = tempPosition;
        }
        else
        {
            assert(false);
        }
    }
}
