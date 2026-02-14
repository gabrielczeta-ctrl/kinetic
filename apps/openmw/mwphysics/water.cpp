#include "water.hpp"
#include "mtphysics.hpp"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>

#include <components/misc/convert.hpp>
#include <components/physicshelpers/collisionobject.hpp>

namespace MWPhysics
{
    MWWater::MWWater(PhysicsTaskScheduler* scheduler, float height)
        : mTaskScheduler(scheduler)
    {
        constexpr float waterPlaneHalfExtent = 1'000'000.f;
        const auto waterPlane = JPH::Plane::sFromPointAndNormal(JPH::Vec3::sZero(), JPH::Vec3::sAxisZ());
        JPH::BodyCreationSettings bodyCreationSettings = PhysicsSystemHelpers::makePhysicsBodySettings(
            new JPH::PlaneShape(waterPlane, nullptr, waterPlaneHalfExtent), osg::Vec3f(0.f, 0.f, height),
            osg::Quat(1.f, 0.f, 0.f, 0.f), Layers::WATER);

        mPhysicsBody = mTaskScheduler->createPhysicsBody(bodyCreationSettings);
        mTaskScheduler->addCollisionObject(mPhysicsBody, false);
    }

    MWWater::~MWWater()
    {
        mTaskScheduler->removeCollisionObject(mPhysicsBody);
        mTaskScheduler->destroyCollisionObject(mPhysicsBody);
    }
}
