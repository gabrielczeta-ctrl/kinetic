#include <type_traits>

#include <osg/Object>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>

#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>
#include <components/physicshelpers/heightfield.hpp>

#include "heightfield.hpp"
#include "mtphysics.hpp"

namespace MWPhysics
{
    HeightField::HeightField(const float* heights, int x, int y, int size, int verts, float minH, float maxH,
        const osg::Object* holdObject, PhysicsTaskScheduler* scheduler)
        : mHoldObject(holdObject)
        , mTaskScheduler(scheduler)
    {
        const float scaling = static_cast<float>(size) / static_cast<float>(verts - 1);
        mWorldOrigin = PhysicsSystemHelpers::getHeightfieldShift(x, y, size, minH, maxH);

        // Determine scale and offset
        JPH::Vec3 terrainOffset = JPH::Vec3(float(size) / 2.0f - float(size), 0.0f, float(size) / 2.0f - float(size));
        JPH::Vec3 terrainScale(scaling, 1.0f, scaling); // NOTE: jolt heightfield is Y up, its rotated below

        // Create height field
        JPH::HeightFieldShapeSettings shapeSettings(
            heights, terrainOffset, terrainScale, static_cast<JPH::uint32>(verts));
        shapeSettings.mMinHeightValue = minH;
        shapeSettings.mMaxHeightValue = maxH;
        shapeSettings.mBitsPerSample = 8;
        shapeSettings.mBlockSize = 4;

        // Must flip on Z axis
        mShapeReference = new JPH::ScaledShape(shapeSettings.Create().Get(), JPH::Vec3(1.0f, 1.0f, -1.0f));

        // Create a quaternion representing a rotation of 90 degrees around the X-axis
        JPH::Quat rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), JPH::DegreesToRadians(90.0f));

        JPH::BodyCreationSettings bodyCreationSettings(mShapeReference,
            JPH::RVec3(mWorldOrigin.x(), mWorldOrigin.y(), 0.0f), rotation, JPH::EMotionType::Static,
            Layers::HEIGHTMAP);

        mPhysicsBody = mTaskScheduler->createPhysicsBody(bodyCreationSettings);
        mTaskScheduler->addCollisionObject(mPhysicsBody);
    }

    HeightField::~HeightField()
    {
        mTaskScheduler->removeCollisionObject(mPhysicsBody);
    }
}
