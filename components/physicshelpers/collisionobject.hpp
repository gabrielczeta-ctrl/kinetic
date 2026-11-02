#ifndef OPENMW_COMPONENTS_JOLTHELPERS_COLLISIONOBJECT_H
#define OPENMW_COMPONENTS_JOLTHELPERS_COLLISIONOBJECT_H

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>

#include <memory>

namespace PhysicsSystemHelpers
{
    inline JPH::BodyCreationSettings makePhysicsBodySettings(const JPH::Shape* shape, const osg::Vec3f& position,
        const osg::Quat& rotation, const JPH::ObjectLayer collisionLayer,
        const JPH::EMotionType motionType = JPH::EMotionType::Static)
    {
        JPH::BodyCreationSettings settings(shape, Misc::Convert::toJolt<JPH::RVec3>(position),
            Misc::Convert::toJolt(rotation), motionType, collisionLayer);
        return settings;
    }

    inline JPH::BodyCreationSettings makePhysicsBodySettings(JPH::ShapeSettings* shape, const osg::Vec3f& position,
        const osg::Quat& rotation, const JPH::ObjectLayer collisionLayer,
        const JPH::EMotionType motionType = JPH::EMotionType::Static)
    {
        auto createRes = shape->Create();
        if (createRes.HasError())
        {
            Log(Debug::Error) << "makePhysicsBodySettings shape create has error: " << createRes.GetError();
        }

        auto shapeRef = createRes.Get();
        return makePhysicsBodySettings(shapeRef, position, rotation, collisionLayer, motionType);
    }
}

#endif
