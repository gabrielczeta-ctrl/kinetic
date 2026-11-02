#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESHBUILDER_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_RECASTMESHBUILDER_H

#include "recastmesh.hpp"
#include "tilebounds.hpp"

#include <components/resource/physicsshape.hpp>

#include <osg/Vec3f>

#include <array>
#include <functional>
#include <memory>
#include <tuple>
#include <vector>

namespace JPH
{
    class Shape;
    class ConvexShape;
    class MeshShape;
    class CompoundShape;
    class HeightFieldShape;
    class RotatedTranslatedShape;
}

namespace DetourNavigator
{
    struct RecastMeshTriangle
    {
        AreaType mAreaType;
        std::array<osg::Vec3f, 3> mVertices;

        friend inline bool operator<(const RecastMeshTriangle& lhs, const RecastMeshTriangle& rhs)
        {
            return std::tie(lhs.mAreaType, lhs.mVertices) < std::tie(rhs.mAreaType, rhs.mVertices);
        }
    };

    using TriangleProcessFunc = std::function<void(JPH::Float3* triangle, int partId, int triangleIndex)>;
    using TriangleWalkerFunc = std::function<void(JPH::Float3& v1, JPH::Float3& v2, JPH::Float3& v3, int)>;

    class RecastMeshBuilder
    {
    public:
        explicit RecastMeshBuilder(const TileBounds& bounds) noexcept;

        void addObject(const JPH::Shape& shape, const osg::Matrixd& transform, const AreaType areaType,
            osg::ref_ptr<const Resource::PhysicsShape> source, const ObjectTransform& objectTransform);

        void addWater(const osg::Vec2i& cellPosition, const Water& water);

        void addHeightfield(const osg::Vec2i& cellPosition, int cellSize, float height);

        void addHeightfield(const osg::Vec2i& cellPosition, int cellSize, const float* heights, std::size_t size,
            float minHeight, float maxHeight);

        std::shared_ptr<RecastMesh> create(const Version& version) &&;

    private:
        const TileBounds mBounds;
        std::vector<RecastMeshTriangle> mTriangles;
        std::vector<CellWater> mWater;
        std::vector<Heightfield> mHeightfields;
        std::vector<FlatHeightfield> mFlatHeightfields;
        std::vector<MeshSource> mSources;

        inline void addObject(const JPH::Shape& shape, const osg::Matrixd& transform, const AreaType areaType);

        void addObject(const JPH::Shape& shape, const osg::Matrixd& transform, TriangleProcessFunc& processTriangle);

        void addObject(
            const JPH::HeightFieldShape& shape, const osg::Matrixd& transform, TriangleProcessFunc& processTriangle);

        void addObject(
            const JPH::RotatedTranslatedShape& shape, const osg::Matrixd& transform, const AreaType areaType);

        void addObject(const JPH::CompoundShape& shape, const osg::Matrixd& transform, const AreaType areaType);

        void addObject(const JPH::HeightFieldShape& shape, const osg::Matrixd& transform, const AreaType areaType);
    };

    Mesh makeMesh(std::vector<RecastMeshTriangle>&& triangles, const osg::Vec3f& shift = osg::Vec3f());

    Mesh makeMesh(const Heightfield& heightfield);
}

#endif
