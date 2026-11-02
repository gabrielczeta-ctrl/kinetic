#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_HEIGHTFIELDMESHBUILDER_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_HEIGHTFIELDMESHBUILDER_H

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
    class BoxShape;
    class MeshShape;
    class CompoundShape;
    class HeightFieldShape;
    class Vec3;
}

namespace DetourNavigator
{
    using TriangleProcessFunc = std::function<void(JPH::Float3* triangle, int partId, int triangleIndex)>;

    class HeightfieldMeshBuilder
    {
    public:
        explicit HeightfieldMeshBuilder(int heightStickWidth, int heightStickLength, const float* heightfieldData,
            double minHeight, double maxHeight);
        ~HeightfieldMeshBuilder() {}

        void initialize(int heightStickWidth, int heightStickLength, const void* heightfieldData, double minHeight,
            double maxHeight, bool flipQuadEdges);

        void getAabb(JPH::Vec3& aabbMin, JPH::Vec3& aabbMax) const;

        void processAllTriangles(
            TriangleProcessFunc& callback, const JPH::Vec3& aabbMin, const JPH::Vec3& aabbMax) const;

        void setLocalScaling(const JPH::Vec3& scaling);

        const JPH::Vec3& getLocalScaling() const;

        void getVertex(int x, int y, JPH::Float3& vertex) const;

        void quantizeWithClamp(int* out, const JPH::Vec3& point, int isMax) const;

        float getRawHeightFieldValue(int x, int y) const;

    private:
        JPH::Vec3 m_localAabbMin;
        JPH::Vec3 m_localAabbMax;
        JPH::Vec3 m_localScaling;
        JPH::Vec3 m_localOrigin;

        int m_heightStickWidth;
        int m_heightStickLength;
        double m_minHeight;
        double m_maxHeight;
        double m_width;
        double m_length;
        const float* m_heightfieldDataFloat;

        bool m_flipQuadEdges;
        bool m_useDiamondSubdivision;
        bool m_useZigzagSubdivision;
    };
}

#endif
