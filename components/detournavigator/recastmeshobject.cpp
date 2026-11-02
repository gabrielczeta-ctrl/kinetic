#include "recastmeshobject.hpp"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>

#include <cassert>

namespace DetourNavigator
{
    namespace
    {
        bool updateCompoundObject(
            const JPH::CompoundShape& shape, const AreaType areaType, std::vector<ChildRecastMeshObject>& children)
        {
            assert(static_cast<std::size_t>(shape.GetNumSubShapes()) == children.size());
            bool result = false;
            const JPH::uint num = shape.GetNumSubShapes();
            for (JPH::uint i = 0; i < num; ++i)
            {
                assert(shape.GetSubShape(i).mShape
                    == std::addressof(*children[static_cast<std::size_t>(i)].getShape().GetPtr()));
                auto shapeTransform = getSubShapeTransform(shape.GetSubShape(i));
                result = children[static_cast<std::size_t>(i)].update(shapeTransform, areaType) || result;
            }
            return result;
        }

        std::vector<ChildRecastMeshObject> makeChildrenObjects(const JPH::CompoundShape& shape, const AreaType areaType)
        {
            std::vector<ChildRecastMeshObject> result;
            const JPH::uint num = shape.GetNumSubShapes();
            for (JPH::uint i = 0; i < num; ++i)
            {
                auto shapeTransform = getSubShapeTransform(shape.GetSubShape(i));
                result.emplace_back(shape.GetSubShape(i), shapeTransform, areaType);
            }
            return result;
        }

        std::vector<ChildRecastMeshObject> makeChildrenObjects(const JPH::Shape& shape, const AreaType areaType)
        {
            if (dynamic_cast<const JPH::CompoundShape*>(&shape))
                return makeChildrenObjects(static_cast<const JPH::CompoundShape&>(shape), areaType);
            return {};
        }
    }

    ChildRecastMeshObject::ChildRecastMeshObject(
        const JPH::CompoundShape::SubShape& shape, const osg::Matrixd& transform, const AreaType areaType)
        : mShape(shape)
        , mTransform(transform)
        , mAreaType(areaType)
        , mLocalScaling(Misc::Convert::toJolt<JPH::Vec3>(transform.getScale()))
        , mChildren(makeChildrenObjects(*shape.mShape.GetPtr(), mAreaType))
    {
    }

    inline bool updateMeshObject(const JPH::Shape& actualShape, const osg::Matrixd& transform, const AreaType areaType,
        osg::Matrixd& mTransform, AreaType& mAreaType, JPH::Vec3& mLocalScaling,
        std::vector<ChildRecastMeshObject>& mChildren)
    {
        bool result = false;
        if (!(mTransform == transform))
        {
            mTransform = transform;
            result = true;
        }
        if (mAreaType != areaType)
        {
            mAreaType = areaType;
            result = true;
        }

        const JPH::ScaledShape* scaledShape = dynamic_cast<const JPH::ScaledShape*>(&actualShape);
        if (scaledShape)
        {
            auto shapeScale = scaledShape->GetScale();
            if (mLocalScaling != shapeScale)
            {
                mLocalScaling = shapeScale;
                result = true;
            }
        }

        if (dynamic_cast<const JPH::CompoundShape*>(&actualShape))
            result = updateCompoundObject(static_cast<const JPH::CompoundShape&>(actualShape), mAreaType, mChildren)
                || result;

        return result;
    }

    bool ChildRecastMeshObject::update(const osg::Matrixd& transform, const AreaType areaType)
    {
        const JPH::RefConst<JPH::Shape> actualShape = mShape.get().mShape;
        return updateMeshObject(
            *actualShape.GetPtr(), transform, areaType, mTransform, mAreaType, mLocalScaling, mChildren);
    }

    bool RecastMeshObject::update(const osg::Matrixd& transform, const AreaType areaType)
    {
        return updateMeshObject(mShape, transform, areaType, mTransform, mAreaType, mLocalScaling, mChildren);
    }

    RecastMeshObject::RecastMeshObject(
        const CollisionShape& shape, const osg::Matrixd& transform, const AreaType areaType)
        : mInstance(shape.getInstance())
        , mObjectTransform(shape.getObjectTransform())
        , mShape(shape.getShape())
        , mTransform(transform)
        , mAreaType(areaType)
        , mLocalScaling(Misc::Convert::toJolt<JPH::Vec3>(transform.getScale()))
        , mChildren(makeChildrenObjects(mShape, mAreaType))
    {
    }
}
