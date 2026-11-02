#ifndef OPENMW_NAVMESHTOOL_WORLDSPACEDATA_H
#define OPENMW_NAVMESHTOOL_WORLDSPACEDATA_H

#include <components/detournavigator/settings.hpp>
#include <components/detournavigator/tilecachedrecastmeshmanager.hpp>
#include <components/esm3/loadland.hpp>
#include <components/misc/convert.hpp>
#include <memory>
#include <regex>
#include <span>
#include <vector>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include <components/detournavigator/tilecachedrecastmeshmanager.hpp>
#include <components/esm3/loadland.hpp>
#include <components/misc/convert.hpp>
#include <components/physicshelpers/collisionobject.hpp>
#include <components/resource/physicsshape.hpp>

namespace ESM
{
    class ESMReader;
    class ReadersCache;
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

namespace NavMeshTool
{
    using DetourNavigator::ObjectTransform;
    using DetourNavigator::RecastSettings;
    using DetourNavigator::TileCachedRecastMeshManager;

    class PhysicsObject
    {
    public:
        PhysicsObject(osg::ref_ptr<Resource::PhysicsShapeInstance>&& shapeInstance, const ESM::Position& position,
            float localScaling)
            : mShapeInstance(std::move(shapeInstance))
            , mObjectTransform{ position, localScaling }
        {
            worldTransform.setTrans(position.asVec3());
            worldTransform.setRotate(Misc::Convert::makeOsgQuat(position));
            worldTransform = osg::Matrixd::scale(osg::Vec3f(localScaling, localScaling, localScaling)) * worldTransform;
        }

        const osg::ref_ptr<Resource::PhysicsShapeInstance>& getShapeInstance() const noexcept { return mShapeInstance; }
        const DetourNavigator::ObjectTransform& getObjectTransform() const noexcept { return mObjectTransform; }
        JPH::Shape& getShape() const noexcept { return *mShapeInstance->mCollisionShape.GetPtr(); }
        const osg::Matrixd& getWorldTransform() const noexcept { return worldTransform; }

    private:
        osg::ref_ptr<Resource::PhysicsShapeInstance> mShapeInstance;
        DetourNavigator::ObjectTransform mObjectTransform;
        osg::Matrixd worldTransform;
    };

    struct TilesData
    {
        const RecastSettings mSettings;
        TileCachedRecastMeshManager mTileCachedRecastMeshManager;
        std::vector<PhysicsObject> mObjects;
        std::vector<std::unique_ptr<ESM::Land::LandData>> mLandData;
        std::vector<std::vector<float>> mHeightfields;

        explicit TilesData(const RecastSettings& settings)
            : mSettings(settings)
            , mTileCachedRecastMeshManager(mSettings)
        {
        }
    };

    struct WorldspaceData
    {
        ESM::RefId mWorldspace;
        JPH::AABox mAabb;
        bool mAabbInitialized = false;
        std::vector<DetourNavigator::TilePosition> mTiles;
        std::shared_ptr<TilesData> mTilesData;

        WorldspaceData(ESM::RefId worldspace, const RecastSettings& settings);
    };

    std::unordered_map<ESM::RefId, std::vector<std::size_t>> collectWorldspaceCells(
        const EsmLoader::EsmData& esmData, bool processInteriorCells, const std::regex& worldspaceFilter);

    WorldspaceData gatherWorldspaceData(const DetourNavigator::Settings& settings, ESM::ReadersCache& readers,
        const VFS::Manager& vfs, Resource::PhysicsShapeManager& physicsShapeManager, const EsmLoader::EsmData& esmData,
        bool writeBinaryLog, ESM::RefId worldspace, std::span<const std::size_t> cells);
}

#endif
