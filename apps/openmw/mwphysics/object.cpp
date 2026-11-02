#include "object.hpp"
#include "mtphysics.hpp"

#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>
#include <components/nifosg/nifloader.hpp>
#include <components/nifosg/particle.hpp>
#include <components/physicshelpers/collisionobject.hpp>
#include <components/resource/physicsshape.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>

#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>

namespace MWPhysics
{
    namespace
    {
        std::mutex sCollidedObjectsMutex;
        std::vector<const MWWorld::LiveCellRefBase*> sCollidedObjects;
    }

    Object::Object(const MWWorld::Ptr& ptr, osg::ref_ptr<Resource::PhysicsShapeInstance> shapeInstance,
        osg::Quat rotation, int collisionType, PhysicsTaskScheduler* scheduler)
        : PtrHolder(ptr, osg::Vec3f())
        , mShapeInstance(std::move(shapeInstance))
        , mSolid(true)
        , mScale(ptr.getCellRef().getScale(), ptr.getCellRef().getScale(), ptr.getCellRef().getScale())
        , mPosition(ptr.getRefData().getPosition().asVec3())
        , mRotation(rotation)
        , mTaskScheduler(scheduler)
        , mCollidedWith(ScriptedCollisionType_None)
    {
        // Create a new shape instance from the settings
        // mBasePhysicsShape = mShapeInstance->mCollisionShape.get()->Create().Get();
        mBasePhysicsShape = mShapeInstance->mCollisionShape.GetPtr();
        mUsesScaledShape = !isScaleIdentity();

        JPH::Shape* finalShape = mUsesScaledShape
            ? new JPH::ScaledShape(mBasePhysicsShape.GetPtr(), Misc::Convert::toJolt<JPH::Vec3>(mScale))
            : mBasePhysicsShape.GetPtr();

        JPH::BodyCreationSettings bodyCreationSettings = PhysicsSystemHelpers::makePhysicsBodySettings(
            finalShape, mPosition, rotation, static_cast<JPH::ObjectLayer>(collisionType));
        mPhysicsBody = mTaskScheduler->createPhysicsBody(bodyCreationSettings);
        mPhysicsBody->SetUserData(reinterpret_cast<uintptr_t>(this));
        mTaskScheduler->addCollisionObject(mPhysicsBody, false);
    }

    Object::~Object()
    {
        mTaskScheduler->removeCollisionObject(mPhysicsBody);
        mTaskScheduler->destroyCollisionObject(mPhysicsBody);
    }

    const Resource::PhysicsShapeInstance* Object::getShapeInstance() const
    {
        return mShapeInstance.get();
    }

    void Object::setScale(float scale)
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        osg::Vec3f newScale = { scale, scale, scale };
        if (mScale != newScale)
        {
            mScale = { scale, scale, scale };
            mScaleUpdatePending = true;
        }
    }

    void Object::setRotation(osg::Quat quat)
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mRotation = quat;
        mTransformUpdatePending = true;
    }

    void Object::updatePosition()
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mPosition = mPtr.getRefData().getPosition().asVec3();
        mTransformUpdatePending = true;
    }

    void Object::commitPositionChange()
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        if (mScaleUpdatePending)
        {
            JPH::BodyInterface& bodyInterface = mTaskScheduler->getBodyInterface();
            JPH::ScaledShape* newShape;
            JPH::ShapeRefC shapeRef;
            shapeRef = mPhysicsBody->GetShape();
            if (!mUsesScaledShape) // Was not originally a scaled shape
            {
                newShape = new JPH::ScaledShape(shapeRef.GetPtr(), Misc::Convert::toJolt<JPH::Vec3>(mScale));
            }
            else
            {
                const JPH::ScaledShape* bodyShape = reinterpret_cast<const JPH::ScaledShape*>(shapeRef.GetPtr());
                const JPH::Shape* innerShape = bodyShape->GetInnerShape();
                newShape = new JPH::ScaledShape(innerShape, Misc::Convert::toJolt<JPH::Vec3>(mScale));
            }

            mUsesScaledShape = true;

            // NOTE: SetShape will destroy the original shape if required, no need to do it after
            bodyInterface.SetShape(getPhysicsBody(), newShape, false, JPH::EActivation::DontActivate);
            mScaleUpdatePending = false;
        }

        if (mTransformUpdatePending)
        {
            // SetPositionAndRotation is thread safe
            JPH::BodyInterface& bodyInterface = mTaskScheduler->getBodyInterface();
            bodyInterface.SetPositionAndRotation(getPhysicsBody(), Misc::Convert::toJolt<JPH::RVec3>(mPosition),
                Misc::Convert::toJolt(mRotation), JPH::EActivation::Activate);

            mTransformUpdatePending = false;
        }
    }

    osg::Matrixd Object::getTransform() const
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        osg::Matrixd trans;
        trans.makeRotate(mRotation);
        trans.setTrans(mPosition);
        return osg::Matrixd::scale(mScale) * trans;
    }

    bool Object::isSolid() const
    {
        return mSolid;
    }

    void Object::setSolid(bool solid)
    {
        mSolid = solid;
    }

    bool Object::isAnimated() const
    {
        return mShapeInstance->isAnimated();
    }

    bool Object::animateCollisionShapes()
    {
        if (mShapeInstance->mAnimatedShapes.empty())
            return false;

        if (!mPtr.getRefData().getBaseNode())
            return false;

        JPH::BodyLockWrite lock(mTaskScheduler->getBodyLockInterface(), getPhysicsBody());
        assert(lock.Succeeded());
        if (!lock.Succeeded())
            return false;

        JPH::MutableCompoundShape* compound = static_cast<JPH::MutableCompoundShape*>(mBasePhysicsShape.GetPtr());

        const auto getNodePathByRecordIndex = [this](int recordIndex) -> osg::NodePath* {
            auto nodePathFound = mRecIndexToNodePath.find(recordIndex);
            if (nodePathFound == mRecIndexToNodePath.end())
            {
                NifOsg::FindGroupByRecordIndex visitor(static_cast<unsigned int>(recordIndex));
                mPtr.getRefData().getBaseNode()->accept(visitor);
                if (!visitor.mFound)
                    return nullptr;

                osg::NodePath nodePath = visitor.mFoundPath;
                nodePath.erase(nodePath.begin());
                nodePathFound = mRecIndexToNodePath.emplace(recordIndex, std::move(nodePath)).first;
            }

            return &nodePathFound->second;
        };

        bool result = false;
        for (const auto& [recordIndex, shapeIndex] : mShapeInstance->mAnimatedShapes)
        {
            osg::NodePath* nodePath = getNodePathByRecordIndex(recordIndex);
            if (!nodePath)
            {
                Log(Debug::Warning) << "Warning: animateCollisionShapes can't find node " << recordIndex << " for "
                                    << mPtr.getCellRef().getRefId();

                // Remove nonexistent nodes from animated shapes map and early out
                mShapeInstance->mAnimatedShapes.erase(recordIndex);
                return false;
            }

            assert(static_cast<int>(compound->GetNumSubShapes()) > shapeIndex);

            const JPH::uint shapeIndexUnsigned = static_cast<JPH::uint>(shapeIndex);
            const JPH::CompoundShape::SubShape& subShape = compound->GetSubShape(shapeIndexUnsigned);
            JPH::RefConst<JPH::Shape> childShape = subShape.mShape;
            auto subShapePosition = subShape.GetPositionCOM();
            auto subShapeRotation = subShape.GetRotation();

            if (const auto collisionSwitch = mShapeInstance->mCollisionSwitches.find(recordIndex);
                collisionSwitch != mShapeInstance->mCollisionSwitches.end())
            {
                osg::NodePath* switchNodePath = getNodePathByRecordIndex(collisionSwitch->second);
                if (switchNodePath && !switchNodePath->empty()
                    && switchNodePath->back()->getNodeMask() == NifOsg::Loader::getIntersectionDisabledNodeMask())
                {
                    static const JPH::Vec3 disabledPosition(1.0e8f, 1.0e8f, 1.0e8f);
                    static const JPH::Quat disabledRotation = JPH::Quat::sIdentity();
                    if (subShapePosition != disabledPosition || subShapeRotation != disabledRotation)
                    {
                        compound->ModifyShape(shapeIndexUnsigned, disabledPosition, disabledRotation);
                        result = true;
                    }
                    continue;
                }
            }

            osg::Matrixf matrix = osg::computeLocalToWorld(*nodePath);
            osg::Vec3f scale = matrix.getScale();
            matrix.orthoNormalize(matrix);

            auto origin = Misc::Convert::toJolt<JPH::Vec3>(matrix.getTrans());
            auto rotation = Misc::Convert::toJolt(matrix.getRotate());

            const JPH::ScaledShape* scaledPtr = dynamic_cast<const JPH::ScaledShape*>(subShape.mShape.GetPtr());
            const bool isScaledSubshape = scaledPtr != nullptr;
            const JPH::Vec3 currentScale = isScaledSubshape ? scaledPtr->GetScale() : JPH::Vec3(1.0f, 1.0f, 1.0f);
            const JPH::Vec3 newScale = Misc::Convert::toJolt<JPH::Vec3>(scale);

            // In Jolt to change the scale requires creating a new ScaledShape and replacing
            // any position/origin change is also applied at the same time
            if (currentScale != newScale)
            {
                const JPH::Shape* baseSubshape = isScaledSubshape ? scaledPtr->GetInnerShape() : childShape.GetPtr();
                JPH::ScaledShape* newShape = new JPH::ScaledShape(baseSubshape, newScale);
                compound->ModifyShape(shapeIndexUnsigned, origin, rotation, newShape);
                result = true;
            }
            else if (subShapeRotation != rotation || origin != subShapePosition)
            {
                compound->ModifyShape(shapeIndexUnsigned, origin, rotation);
                result = true;
            }
        }
        return result;
    }

    bool Object::collidedWith(ScriptedCollisionType type) const
    {
        return mCollidedWith & type;
    }

    void Object::addCollision(ScriptedCollisionType type)
    {
        bool firstCollisionForThisFrame = false;
        {
            std::unique_lock<std::mutex> lock(mPositionMutex);
            firstCollisionForThisFrame = mCollidedWith == ScriptedCollisionType_None;
            mCollidedWith |= type;
        }

        if (firstCollisionForThisFrame)
        {
            std::unique_lock<std::mutex> lock(sCollidedObjectsMutex);
            sCollidedObjects.push_back(mPtr.mRef);
        }
    }

    void Object::resetCollisions()
    {
        std::unique_lock<std::mutex> lock(mPositionMutex);
        mCollidedWith = ScriptedCollisionType_None;
    }

    std::vector<const MWWorld::LiveCellRefBase*> Object::consumeCollidedObjects()
    {
        std::unique_lock<std::mutex> lock(sCollidedObjectsMutex);
        auto result = std::move(sCollidedObjects);
        sCollidedObjects.clear();
        return result;
    }
}
