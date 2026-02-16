/*
    Original code from btHeightfieldShape - modified slightly

    Bullet Continuous Collision Detection and Physics Library
    Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

    This software is provided 'as-is', without any express or implied warranty.
    In no event will the authors be held liable for any damages arising from the use of this software.
    Permission is granted to anyone to use this software for any purpose,
    including commercial applications, and to alter it and redistribute it freely,
    subject to the following restrictions:

    1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software.
   If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not
   required.
    2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original
   software.
    3. This notice may not be removed or altered from any source distribution.
*/

#include "heightfieldmeshbuilder.hpp"

#include <components/misc/convert.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Real.h>

#include <algorithm>

namespace DetourNavigator
{
    HeightfieldMeshBuilder::HeightfieldMeshBuilder(
        int heightStickWidth, int heightStickLength, const float* heightfieldData, double minHeight, double maxHeight)
    {
        initialize(heightStickWidth, heightStickLength, heightfieldData, minHeight, maxHeight, false);
    }

    void HeightfieldMeshBuilder::initialize(int heightStickWidth, int heightStickLength, const void* heightfieldData,
        double minHeight, double maxHeight, bool flipQuadEdges)
    {
        assert(heightStickWidth > 1); // && "bad width");
        assert(heightStickLength > 1); // && "bad length");
        assert(heightfieldData); // && "null heightfield data");
        assert(minHeight <= maxHeight);

        // initialize member variables
        m_heightStickWidth = heightStickWidth;
        m_heightStickLength = heightStickLength;
        m_minHeight = minHeight;
        m_maxHeight = maxHeight;
        m_width = (double)(heightStickWidth - 1);
        m_length = (double)(heightStickLength - 1);
        m_heightfieldDataFloat = static_cast<const float*>(heightfieldData);
        m_flipQuadEdges = flipQuadEdges;
        m_useDiamondSubdivision = false;
        m_useZigzagSubdivision = false;
        m_localScaling.Set(1.0f, 1.0f, 1.0f);
        m_localAabbMin.Set(0.0f, 0.0f, static_cast<float>(m_minHeight));
        m_localAabbMax.Set(static_cast<float>(m_width), static_cast<float>(m_length), static_cast<float>(m_maxHeight));

        // remember origin (defined as exact middle of aabb)
        m_localOrigin = 0.5f * (m_localAabbMin + m_localAabbMax);
    }

    void HeightfieldMeshBuilder::getAabb(JPH::Vec3& aabbMin, JPH::Vec3& aabbMax) const
    {
        JPH::Vec3 halfExtents = (m_localAabbMax - m_localAabbMin) * m_localScaling * 0.5f;
        aabbMin = -halfExtents;
        aabbMax = halfExtents;
    }

    float HeightfieldMeshBuilder::getRawHeightFieldValue(int x, int y) const
    {
        return m_heightfieldDataFloat[(y * m_heightStickWidth) + x];
    }

    /// this returns the vertex in local coordinates
    void HeightfieldMeshBuilder::getVertex(int x, int y, JPH::Float3& vertex) const
    {
        assert(x >= 0);
        assert(y >= 0);
        assert(x < m_heightStickWidth);
        assert(y < m_heightStickLength);

        const float height = getRawHeightFieldValue(x, y);
        const float xOffset = static_cast<float>((-m_width / 2.0) + static_cast<double>(x));
        const float yOffset = static_cast<float>((-m_length / 2.0) + static_cast<double>(y));

        vertex.x = m_localScaling.GetX() * xOffset;
        vertex.y = m_localScaling.GetY() * yOffset;
        vertex.z = m_localScaling.GetZ() * (height - m_localOrigin.GetZ());
    }

    static inline int getQuantized(double x)
    {
        if (x < 0.0)
        {
            return (int)(x - 0.5);
        }
        return (int)(x + 0.5);
    }

    /// given input vector, return quantized version
    /**
    This routine is basically determining the gridpoint indices for a given
    input vector, answering the question: "which gridpoint is closest to the
    provided point?".

    "with clamp" means that we restrict the point to be in the heightfield's
    axis-aligned bounding box.
    */
    void HeightfieldMeshBuilder::quantizeWithClamp(int* out, const JPH::Vec3& point, int /*isMax*/) const
    {
        JPH::Vec3 clampedPoint(point);
        clampedPoint.Set(std::max(m_localAabbMin.GetX(), clampedPoint.GetX()),
            std::max(m_localAabbMin.GetY(), clampedPoint.GetY()), std::max(m_localAabbMin.GetZ(), clampedPoint.GetZ()));

        clampedPoint.Set(std::min(m_localAabbMax.GetX(), clampedPoint.GetX()),
            std::min(m_localAabbMax.GetY(), clampedPoint.GetY()), std::min(m_localAabbMax.GetZ(), clampedPoint.GetZ()));

        out[0] = getQuantized(clampedPoint.GetX());
        out[1] = getQuantized(clampedPoint.GetY());
        out[2] = getQuantized(clampedPoint.GetZ());
    }

    /// process all triangles within the provided axis-aligned bounding box
    /**
    basic algorithm:
        - convert input aabb to local coordinates (scale down and shift for local origin)
        - convert input aabb to a range of heightfield grid points (quantize)
        - iterate over all triangles in that subset of the grid
    */
    void HeightfieldMeshBuilder::processAllTriangles(
        TriangleProcessFunc& callback, const JPH::Vec3& aabbMin, const JPH::Vec3& aabbMax) const
    {
        // scale down the input aabb's so they are in local (non-scaled) coordinates
        JPH::Vec3 localScale = JPH::Vec3(1.f / m_localScaling[0], 1.f / m_localScaling[1], 1.f / m_localScaling[2]);
        JPH::Vec3 localAabbMin = aabbMin * localScale;
        JPH::Vec3 localAabbMax = aabbMax * localScale;

        // account for local origin
        localAabbMin += m_localOrigin;
        localAabbMax += m_localOrigin;

        // quantize the aabbMin and aabbMax, and adjust the start/end ranges
        int quantizedAabbMin[3];
        int quantizedAabbMax[3];
        quantizeWithClamp(quantizedAabbMin, localAabbMin, 0);
        quantizeWithClamp(quantizedAabbMax, localAabbMax, 1);

        // expand the min/max quantized values
        // this is to catch the case where the input aabb falls between grid points!
        for (int i = 0; i < 3; ++i)
        {
            quantizedAabbMin[i]--;
            quantizedAabbMax[i]++;
        }

        int startX = 0;
        int endX = m_heightStickWidth - 1;
        int startJ = 0;
        int endJ = m_heightStickLength - 1;

        if (quantizedAabbMin[0] > startX)
            startX = quantizedAabbMin[0];
        if (quantizedAabbMax[0] < endX)
            endX = quantizedAabbMax[0];
        if (quantizedAabbMin[1] > startJ)
            startJ = quantizedAabbMin[1];
        if (quantizedAabbMax[1] < endJ)
            endJ = quantizedAabbMax[1];

        const float queryMinHeight = localAabbMin.GetZ();
        const float queryMaxHeight = localAabbMax.GetZ();

        for (int j = startJ; j < endJ; j++)
        {
            for (int x = startX; x < endX; x++)
            {
                const float height00 = getRawHeightFieldValue(x, j);
                const float height01 = getRawHeightFieldValue(x, j + 1);
                const float height10 = getRawHeightFieldValue(x + 1, j);
                const float height11 = getRawHeightFieldValue(x + 1, j + 1);
                const float minHeight = std::min(std::min(height00, height01), std::min(height10, height11));
                const float maxHeight = std::max(std::max(height00, height01), std::max(height10, height11));
                if (maxHeight < queryMinHeight || minHeight > queryMaxHeight)
                    continue;

                JPH::Float3 vertices[3];
                int indices[3] = { 0, 1, 2 };

                if (m_flipQuadEdges || (m_useDiamondSubdivision && !((j + x) & 1))
                    || (m_useZigzagSubdivision && !(j & 1)))
                {
                    getVertex(x, j, vertices[indices[0]]);
                    getVertex(x, j + 1, vertices[indices[1]]);
                    getVertex(x + 1, j + 1, vertices[indices[2]]);
                    callback(vertices, 2 * x, j);

                    vertices[indices[1]] = vertices[indices[2]];

                    getVertex(x + 1, j, vertices[indices[2]]);
                    callback(vertices, 2 * x + 1, j);
                }
                else
                {
                    getVertex(x, j, vertices[indices[0]]);
                    getVertex(x, j + 1, vertices[indices[1]]);
                    getVertex(x + 1, j, vertices[indices[2]]);
                    callback(vertices, 2 * x, j);

                    vertices[indices[0]] = vertices[indices[2]];

                    getVertex(x + 1, j + 1, vertices[indices[2]]);
                    callback(vertices, 2 * x + 1, j);
                }
            }
        }
    }

    void HeightfieldMeshBuilder::setLocalScaling(const JPH::Vec3& scaling)
    {
        m_localScaling = scaling;
    }

    const JPH::Vec3& HeightfieldMeshBuilder::getLocalScaling() const
    {
        return m_localScaling;
    }
}
