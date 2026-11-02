#ifndef OPENMW_COMPONENTS_JOLTHELPERS_TRANSFORMBOUNDINGBOX_H
#define OPENMW_COMPONENTS_JOLTHELPERS_TRANSFORMBOUNDINGBOX_H

#include <algorithm>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Mat44.h>
#include <Jolt/Math/Vec3.h>

namespace PhysicsSystemHelpers
{
    inline JPH::Vec3 min(const JPH::Vec3& a, const JPH::Vec3& b)
    {
        return JPH::Vec3(std::min(a.GetX(), b.GetX()), std::min(a.GetY(), b.GetY()), std::min(a.GetZ(), b.GetZ()));
    }

    inline JPH::Vec3 max(const JPH::Vec3& a, const JPH::Vec3& b)
    {
        return JPH::Vec3(std::max(a.GetX(), b.GetX()), std::max(a.GetY(), b.GetY()), std::max(a.GetZ(), b.GetZ()));
    }

    // Adaptation of bounding box transformation for Jolt Physics
    inline void transformBoundingBox(const JPH::RMat44& transform, JPH::Vec3& aabbMin, JPH::Vec3& aabbMax)
    {
        auto transformFloat = transform.ToMat44();
        const JPH::Vec3 xa = transform.Multiply3x3(JPH::Vec3(aabbMin.GetX(), 0, 0));
        const JPH::Vec3 xb = transform.Multiply3x3(JPH::Vec3(aabbMax.GetX(), 0, 0));

        const JPH::Vec3 ya = transform.Multiply3x3(JPH::Vec3(0, aabbMin.GetY(), 0));
        const JPH::Vec3 yb = transform.Multiply3x3(JPH::Vec3(0, aabbMax.GetY(), 0));

        const JPH::Vec3 za = transform.Multiply3x3(JPH::Vec3(0, 0, aabbMin.GetZ()));
        const JPH::Vec3 zb = transform.Multiply3x3(JPH::Vec3(0, 0, aabbMax.GetZ()));

        aabbMin = min(xa, xb) + min(ya, yb) + min(za, zb) + transformFloat.GetTranslation();
        aabbMax = max(xa, xb) + max(ya, yb) + max(za, zb) + transformFloat.GetTranslation();
    }
}

#endif
