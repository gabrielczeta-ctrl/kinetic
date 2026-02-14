#include "actor.hpp"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>

#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>
#include <components/physicshelpers/collisionobject.hpp>
#include <components/resource/physicsshape.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>

#include "../mwmechanics/creaturestats.hpp"
#include "../mwworld/class.hpp"

#include "joltlayers.hpp"
#include "mtphysics.hpp"
#include "trace.h"

#include <cmath>

namespace MWPhysics
{

    Actor::Actor(const MWWorld::Ptr& ptr, const Resource::PhysicsShape* shape, PhysicsTaskScheduler* scheduler,
        bool canWaterWalk, DetourNavigator::CollisionShapeType collisionShapeType)
        : PtrHolder(ptr, ptr.getRefData().getPosition().asVec3())
        , mStandingOnPtr(nullptr)
        , mCanWaterWalk(canWaterWalk)
        , mWalkingOnWater(false)
        , mMeshTranslation(shape->mCollisionBox.mCenter)
        , mOriginalHalfExtents(shape->mCollisionBox.mExtents)
        , mScale({ 1.0f, 1.0f, 1.0f })
        , mStuckFrames(0)
        , mLastStuckPosition{ 0, 0, 0 }
        , mForce(0.f, 0.f, 0.f)
        , mOnGround(ptr.getClass().getCreatureStats(ptr).getFallHeight() == 0)
        , mOnSlope(false)
        , mInternalCollisionMode(true)
        , mExternalCollisionMode(true)
        , mActive(false)
        , mTaskScheduler(scheduler)
    {
        // We can not create actor without collisions - he will fall through the ground.
        // In this case we should autogenerate collision box based on mesh shape
        // (NPCs have bodyparts and use a different approach)
        if (!ptr.getClass().isNpc() && mOriginalHalfExtents.length2() == 0.f)
        {
            if (shape->mCollisionShape)
            {
                auto shapeRef = shape->mCollisionShape;
                JPH::AABox bounds = shapeRef->GetLocalBounds();
                osg::Vec3f min = osg::Vec3f(bounds.mMin.GetX(), bounds.mMin.GetY(), bounds.mMin.GetZ());
                osg::Vec3f max = osg::Vec3f(bounds.mMax.GetX(), bounds.mMax.GetY(), bounds.mMax.GetZ());

                mOriginalHalfExtents.x() = (max[0] - min[0]) / 2.f;
                mOriginalHalfExtents.y() = (max[1] - min[1]) / 2.f;
                mOriginalHalfExtents.z() = (max[2] - min[2]) / 2.f;

                mMeshTranslation = osg::Vec3f(0.f, 0.f, mOriginalHalfExtents.z());
            }

            if (mOriginalHalfExtents.length2() == 0.f)
                Log(Debug::Error) << "Error: Failed to calculate bounding box for actor \""
                                  << ptr.getCellRef().getRefId() << "\".";
        }

        if ((mMeshTranslation.x() == 0.0 && mMeshTranslation.y() == 0.0)
            && std::fabs(mOriginalHalfExtents.x() - mOriginalHalfExtents.y()) < 2.2)
        {
            switch (collisionShapeType)
            {
                default:
                case DetourNavigator::CollisionShapeType::Aabb:
                    mBasePhysicsShape = new JPH::BoxShape(Misc::Convert::toJolt<JPH::Vec3>(mOriginalHalfExtents));
                    mRotationallyInvariant = true;
                    break;
                case DetourNavigator::CollisionShapeType::RotatingBox:
                    mBasePhysicsShape = new JPH::BoxShape(Misc::Convert::toJolt<JPH::Vec3>(mOriginalHalfExtents));
                    mRotationallyInvariant = false;
                    break;
                case DetourNavigator::CollisionShapeType::Cylinder:
                    JPH::Quat shapeRotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), JPH::DegreesToRadians(90.0f));
                    mBasePhysicsShape = new JPH::RotatedTranslatedShape(JPH::Vec3(0.0f, 0.0f, 0.0f), shapeRotation,
                        new JPH::CylinderShape(mOriginalHalfExtents.z(), mOriginalHalfExtents.x()));
                    mRotationallyInvariant = true;
                    break;
            }
            mCollisionShapeType = collisionShapeType;
        }
        else
        {
            mBasePhysicsShape = new JPH::BoxShape(Misc::Convert::toJolt<JPH::Vec3>(mOriginalHalfExtents));
            mRotationallyInvariant = false;
            mCollisionShapeType = DetourNavigator::CollisionShapeType::RotatingBox;
        }

        updateScaleUnsafe();

        if (!mRotationallyInvariant)
        {
            const SceneUtil::PositionAttitudeTransform* baseNode = mPtr.getRefData().getBaseNode();
            if (baseNode)
                mRotation = baseNode->getAttitude();
        }

        // Jolt specific
        // NOTE: player must always have scaled shape to prevent runtime cost of updating body/shape if scale requested
        mPhysicsShape = new JPH::ScaledShape(mBasePhysicsShape, Misc::Convert::toJolt<JPH::Vec3>(mScale));
        mScaleUpdated = false; // We set the scale just now so cancel update flag set in updateScaleUnsafe

        JPH::BodyCreationSettings bodyCreationSettings = PhysicsSystemHelpers::makePhysicsBodySettings(mPhysicsShape,
            getScaledMeshTranslation() + mPosition, mRotation, Layers::ACTOR, JPH::EMotionType::Kinematic);
        bodyCreationSettings.mEnhancedInternalEdgeRemoval = true;

        mPhysicsBody = mTaskScheduler->createPhysicsBody(bodyCreationSettings);
        mPhysicsBody->SetUserData(reinterpret_cast<uintptr_t>(this));
        mTaskScheduler->addCollisionObject(mPhysicsBody, true);
    }

    Actor::~Actor()
    {
        mTaskScheduler->removeCollisionObject(mPhysicsBody);
        mTaskScheduler->destroyCollisionObject(mPhysicsBody);
    }

    void Actor::enableCollisionMode(bool collision)
    {
        mInternalCollisionMode = collision;
    }

    void Actor::enableCollisionBody(bool collision)
    {
        std::scoped_lock lock(mMutex);
        if (mExternalCollisionMode != collision)
        {
            mExternalCollisionMode = collision;

            // If collision is intended to be disabled on this object (i,e for a corpse)
            // then we need to update the body to a new layer to prevent other things colliding with it
            JPH::BodyInterface& bodyInterface = mTaskScheduler->getBodyInterface();
            bodyInterface.SetObjectLayer(getPhysicsBody(), mExternalCollisionMode ? Layers::ACTOR : Layers::DEBRIS);
        }
    }

    JPH::ObjectLayer Actor::getCollisionMask() const
    {
        std::scoped_lock lock(mMutex);
        JPH::ObjectLayer collisionMask = Layers::WORLD | Layers::HEIGHTMAP;
        if (mExternalCollisionMode)
            collisionMask |= Layers::ACTOR | Layers::PROJECTILE | Layers::DOOR;
        if (mCanWaterWalk)
            collisionMask |= Layers::WATER;
        return collisionMask;
    }

    void Actor::updatePosition()
    {
        std::scoped_lock lock(mMutex);
        const auto worldPosition = mPtr.getRefData().getPosition().asVec3();
        mPreviousPosition = worldPosition;
        mPosition = worldPosition;
        mSimulationPosition = worldPosition;
        mPositionOffset = osg::Vec3f();
        mStandingOnPtr = nullptr;
        mSkipSimulation = true;
    }

    void Actor::setSimulationPosition(const osg::Vec3f& position)
    {
        if (!std::exchange(mSkipSimulation, false))
            mSimulationPosition = position;
    }

    osg::Vec3f Actor::getScaledMeshTranslation() const
    {
        return mRotation * osg::componentMultiply(mMeshTranslation, mScale);
    }

    void Actor::updateCollisionObjectPosition()
    {
        std::scoped_lock lock(mMutex);
        updateCollisionObjectPositionUnsafe();
    }

    void Actor::updateCollisionObjectPositionUnsafe()
    {
        if (getPhysicsBody().IsInvalid())
            return;

        JPH::BodyInterface& bodyInterface = mTaskScheduler->getBodyInterface();

        // If scale was updated, we need to re-create a scaled shape
        // This code assumes mPhysicsShape is a ScaledShape and not equal to mBasePhysicsShape (which is fine for now)
        if (mScaleUpdated)
        {
            // Jolt handles destroying of the original shape when SetShape is called!
            mPhysicsShape = new JPH::ScaledShape(mBasePhysicsShape, Misc::Convert::toJolt<JPH::Vec3>(mScale));
            bodyInterface.SetShape(getPhysicsBody(), mPhysicsShape, false, JPH::EActivation::DontActivate);
            mScaleUpdated = false;
        }

        osg::Vec3f newPosition = getScaledMeshTranslation() + mPosition;

        // NOTE: SetPositionAndRotation is thread safe to call
        bodyInterface.SetPositionAndRotation(getPhysicsBody(), Misc::Convert::toJolt<JPH::RVec3>(newPosition),
            Misc::Convert::toJolt(mRotation), JPH::EActivation::Activate);
    }

    osg::Vec3f Actor::getCollisionObjectPosition() const
    {
        std::scoped_lock lock(mMutex);
        return getScaledMeshTranslation() + mPosition;
    }

    bool Actor::setPosition(const osg::Vec3f& position)
    {
        std::scoped_lock lock(mMutex);
        const bool worldPositionChanged = mPositionOffset.length2() != 0;
        applyOffsetChange();
        if (worldPositionChanged || mSkipSimulation)
            return true;
        mPreviousPosition = mPosition;
        mPosition = position;
        return mPreviousPosition != mPosition;
    }

    void Actor::adjustPosition(const osg::Vec3f& offset)
    {
        std::scoped_lock lock(mMutex);
        mPositionOffset += offset;
    }

    osg::Vec3f Actor::applyOffsetChange()
    {
        if (mPositionOffset.length2() != 0)
        {
            mPosition += mPositionOffset;
            mPreviousPosition += mPositionOffset;
            mSimulationPosition += mPositionOffset;
            mPositionOffset = osg::Vec3f();
        }
        return mPosition;
    }

    void Actor::setRotation(osg::Quat quat)
    {
        std::scoped_lock lock(mMutex);
        mRotation = quat;
    }

    bool Actor::isRotationallyInvariant() const
    {
        return mRotationallyInvariant;
    }

    void Actor::updateScale()
    {
        std::scoped_lock lock(mMutex);
        updateScaleUnsafe();
    }

    void Actor::updateScaleUnsafe()
    {
        auto prevScale = mScale;
        float scale = mPtr.getCellRef().getScale();
        osg::Vec3f scaleVec(scale, scale, scale);

        mPtr.getClass().adjustScale(mPtr, scaleVec, false);
        mScale = scaleVec;
        mHalfExtents = osg::componentMultiply(mOriginalHalfExtents, scaleVec);

        scaleVec = osg::Vec3f(scale, scale, scale);
        mPtr.getClass().adjustScale(mPtr, scaleVec, true);
        mRenderingHalfExtents = osg::componentMultiply(mOriginalHalfExtents, scaleVec);
        mScaleUpdated = mScale != prevScale;
    }

    osg::Vec3f Actor::getHalfExtents() const
    {
        return mHalfExtents;
    }

    osg::Vec3f Actor::getOriginalHalfExtents() const
    {
        return mOriginalHalfExtents;
    }

    osg::Vec3f Actor::getRenderingHalfExtents() const
    {
        return mRenderingHalfExtents;
    }

    void Actor::setInertialForce(const osg::Vec3f& force)
    {
        mForce = force;
    }

    void Actor::setOnGround(bool grounded)
    {
        mOnGround = grounded;
    }

    void Actor::setOnSlope(bool slope)
    {
        mOnSlope = slope;
    }

    bool Actor::isWalkingOnWater() const
    {
        return mWalkingOnWater;
    }

    void Actor::setWalkingOnWater(bool walkingOnWater)
    {
        mWalkingOnWater = walkingOnWater;
    }

    void Actor::setCanWaterWalk(bool waterWalk)
    {
        std::scoped_lock lock(mMutex);
        if (waterWalk != mCanWaterWalk)
        {
            mCanWaterWalk = waterWalk;
        }
    }

    MWWorld::Ptr Actor::getStandingOnPtr() const
    {
        std::scoped_lock lock(mMutex);
        return mStandingOnPtr;
    }

    void Actor::setStandingOnPtr(const MWWorld::Ptr& ptr)
    {
        std::scoped_lock lock(mMutex);
        mStandingOnPtr = ptr;
    }

    bool Actor::canMoveToWaterSurface(float waterlevel, const JPH::PhysicsSystem* physicsSystem) const
    {
        const float halfZ = getHalfExtents().z();
        const osg::Vec3f actorPosition = getPosition();
        const osg::Vec3f startingPosition(actorPosition.x(), actorPosition.y(), actorPosition.z() + halfZ);
        const osg::Vec3f destinationPosition(actorPosition.x(), actorPosition.y(), waterlevel + halfZ);
        MWPhysics::ActorTracer tracer;
        tracer.doTrace(getPhysicsBody(), startingPosition, destinationPosition, physicsSystem, getCollisionMask());
        return (tracer.mFraction >= 1.0f);
    }

}
