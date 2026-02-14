#ifndef OPENMW_COMPONENTS_MISC_CONVERT_H
#define OPENMW_COMPONENTS_MISC_CONVERT_H

#include <components/esm/position.hpp>
#include <components/esm3/loadpgrd.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Float3.h>
#include <Jolt/Math/Mat44.h>
#include <Jolt/Math/Real.h>

#include <osg/Matrixd>
#include <osg/Quat>
#include <osg/Vec3f>

namespace Misc::Convert
{
    inline osg::Vec3f makeOsgVec3f(const float* values)
    {
        return osg::Vec3f(values[0], values[1], values[2]);
    }

    inline osg::Vec3f makeOsgVec3f(const ESM::Pathgrid::Point& value)
    {
        return osg::Vec3f(value.mX, value.mY, value.mZ);
    }

    template <typename T>
    inline T toJolt(const osg::Vec3f& vec)
    {
        return T(vec.x(), vec.y(), vec.z());
    }

    template <typename T>
    inline T toJolt(const osg::Vec3d& vec)
    {
        return T(vec.x(), vec.y(), vec.z());
    }

    template <typename T>
    inline T toJolt(const JPH::RVec3& vec)
    {
        return T(vec.GetX(), vec.GetY(), vec.GetZ());
    }

    template <typename T>
    inline T toJolt(const JPH::Float3& vec)
    {
        return T(vec.x, vec.y, vec.z);
    }

    inline JPH::Quat toJolt(const osg::Quat& quat)
    {
        return JPH::Quat(quat.x(), quat.y(), quat.z(), quat.w());
    }

    inline osg::Matrixd toOsgNoScale(const JPH::Mat44& joltMatrix)
    {
        osg::Matrixd mat;
        auto inPosition = joltMatrix.GetTranslation();
        auto subRot = joltMatrix.GetRotation().GetQuaternion();
        mat.makeRotate(osg::Quat(subRot.GetX(), subRot.GetY(), subRot.GetZ(), subRot.GetW()));
        mat.setTrans(osg::Vec3f(inPosition.GetX(), inPosition.GetY(), inPosition.GetZ()));
        return mat;
    }

    inline JPH::RMat44 toJoltNoScale(const osg::Matrixd& mat)
    {
        JPH::Quat quat = toJolt(mat.getRotate()).Normalized();
        osg::Vec3d trans = mat.getTrans();
        return JPH::RMat44::sRotationTranslation(quat, toJolt<JPH::RVec3>(trans));
    }

    inline osg::Vec3f toOsg(const JPH::RVec3& vec)
    {
        return osg::Vec3f(vec.GetX(), vec.GetY(), vec.GetZ());
    }

#ifdef JPH_DOUBLE_PRECISION
    inline osg::Vec3f toOsg(const JPH::Vec3& vec)
    {
        return osg::Vec3f(vec.GetX(), vec.GetY(), vec.GetZ());
    }
#endif

    template <typename T>
    inline T* toPointerFromUserData(uint64_t userData)
    {
        if (userData > 0)
        {
            // Converts from userdata uintptr to class type
            // You should know the userdata points to the correct type!
            T* ptr = reinterpret_cast<T*>(static_cast<uintptr_t>(userData));
            return ptr;
        }
        return nullptr;
    }

    inline osg::Vec3f toOsg(const JPH::Float3& vec)
    {
        return osg::Vec3f(vec.x, vec.y, vec.z);
    }

    inline osg::Quat makeOsgQuat(const float (&rotation)[3])
    {
        return osg::Quat(rotation[2], osg::Vec3f(0, 0, -1)) * osg::Quat(rotation[1], osg::Vec3f(0, -1, 0))
            * osg::Quat(rotation[0], osg::Vec3f(-1, 0, 0));
    }

    inline osg::Quat makeOsgQuat(const ESM::Position& position)
    {
        return makeOsgQuat(position.rot);
    }

    inline osg::Quat makeQuaternion(const float (&rotation)[3])
    {
        return osg::Quat(0, 0, -1, rotation[2]) * osg::Quat(0, -1, 0, rotation[1]) * osg::Quat(-1, 0, 0, rotation[0]);
    }

    inline osg::Quat makeQuaternion(const ESM::Position& position)
    {
        return makeQuaternion(position.rot);
    }

    inline osg::Matrixd makeOSGTransform(const ESM::Position& position)
    {
        osg::Matrixd mat;
        mat.setRotate(makeQuaternion(position));
        mat.setTrans(position.asVec3());
        return mat;
    }

    inline osg::Vec2f toOsgXY(const osg::Vec3f& value)
    {
        return osg::Vec2f(static_cast<float>(value.x()), static_cast<float>(value.y()));
    }

    inline osg::Vec2f toOsgXY(const JPH::RVec3& value)
    {
        return osg::Vec2f(static_cast<float>(value.GetX()), static_cast<float>(value.GetY()));
    }

#ifdef JPH_DOUBLE_PRECISION
    inline osg::Vec2f toOsgXY(const JPH::Vec3& value)
    {
        return osg::Vec2f(static_cast<float>(value.GetX()), static_cast<float>(value.GetY()));
    }
#endif
}

#endif
