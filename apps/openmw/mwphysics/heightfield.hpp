#ifndef OPENMW_MWPHYSICS_HEIGHTFIELD_H
#define OPENMW_MWPHYSICS_HEIGHTFIELD_H

#include <osg/ref_ptr>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <memory>
#include <vector>

namespace osg
{
    class Object;
}

namespace MWPhysics
{
    class PhysicsTaskScheduler;

    class HeightField
    {
    public:
        HeightField(const float* heights, int x, int y, int size, int verts, float minH, float maxH,
            const osg::Object* holdObject, PhysicsTaskScheduler* scheduler);
        ~HeightField();

        const osg::Vec3f& getOrigin() const { return mWorldOrigin; }

    private:
        JPH::Body* mPhysicsBody; // NOTE: memory is managed by Jolt!
        JPH::ShapeRefC mShapeReference;
        osg::Vec3f mWorldOrigin;
        osg::ref_ptr<const osg::Object> mHoldObject;

        PhysicsTaskScheduler* mTaskScheduler;

        void operator=(const HeightField&);
        HeightField(const HeightField&);
    };
}

#endif
