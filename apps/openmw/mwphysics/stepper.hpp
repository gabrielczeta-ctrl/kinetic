#ifndef OPENMW_MWPHYSICS_STEPPER_H
#define OPENMW_MWPHYSICS_STEPPER_H

#include "trace.h"

namespace osg
{
    class Vec3f;
}

namespace MWPhysics
{
    class Stepper
    {
    private:
        const JPH::PhysicsSystem* mColWorld;
        JPH::BodyID mColObj;

        ActorTracer mTracer, mUpStepper, mDownStepper;

    public:
        Stepper(const JPH::PhysicsSystem* colWorld, JPH::BodyID colObj);

        bool step(osg::Vec3f& position, osg::Vec3f& velocity, float& remainingTime, const bool& onGround,
            bool firstIteration, const int collisionMask);
    };
}

#endif
