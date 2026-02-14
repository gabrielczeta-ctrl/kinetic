#include "physicsshape.hpp"

#include <stdexcept>
#include <string>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <components/debug/debuglog.hpp>

namespace Resource
{
    PhysicsShape::PhysicsShape(const PhysicsShape& other, const osg::CopyOp& copyOp)
        : Object(other, copyOp)
        , mCollisionShape(other.mCollisionShape)
        , mAvoidCollisionShape(other.mAvoidCollisionShape)
        , mCollisionBox(other.mCollisionBox)
        , mAnimatedShapes(other.mAnimatedShapes)
        , mCollisionSwitches(other.mCollisionSwitches)
        , mFileName(other.mFileName)
        , mFileHash(other.mFileHash)
        , mVisualCollisionType(other.mVisualCollisionType)
    {
    }

    osg::ref_ptr<PhysicsShapeInstance> makeInstance(osg::ref_ptr<const PhysicsShape> source)
    {
        return { new PhysicsShapeInstance(std::move(source)) };
    }

    PhysicsShapeInstance::PhysicsShapeInstance(osg::ref_ptr<const PhysicsShape> source)
        : PhysicsShape(*source)
        , mSource(std::move(source))
    {
    }
}
