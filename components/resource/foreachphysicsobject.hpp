#ifndef OPENMW_COMPONENTS_RESOURCE_FOREACHPHYSICSOBJECT_H
#define OPENMW_COMPONENTS_RESOURCE_FOREACHPHYSICSOBJECT_H

#include <components/esm/position.hpp>
#include <components/resource/physicsshape.hpp>

#include <osg/ref_ptr>

#include <functional>
#include <vector>

namespace ESM
{
    class ReadersCache;
    struct Cell;
}

namespace VFS
{
    class Manager;
}

namespace Resource
{
    class PhysicsShapeManager;
}

namespace EsmLoader
{
    struct EsmData;
}

namespace Resource
{
    struct PhysicsObject
    {
        osg::ref_ptr<const Resource::PhysicsShape> mShape;
        ESM::Position mPosition;
        float mScale;
    };

    void forEachPhysicsObject(ESM::ReadersCache& readers, const VFS::Manager& vfs,
        Resource::PhysicsShapeManager& physicsShapeManager, const EsmLoader::EsmData& esmData,
        std::function<void(const ESM::Cell&, const PhysicsObject& object)> callback);
}

#endif
