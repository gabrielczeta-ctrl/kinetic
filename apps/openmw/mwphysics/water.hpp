#ifndef OPENMW_MWWATER_OBJECT_H
#define OPENMW_MWWATER_OBJECT_H

namespace JPH
{
    class Body;
}

namespace MWPhysics
{
    class PhysicsTaskScheduler;
    class MWWater
    {
    public:
        MWWater(PhysicsTaskScheduler* scheduler, float height);
        ~MWWater();

    private:
        JPH::Body* mPhysicsBody;
        PhysicsTaskScheduler* mTaskScheduler;
    };
}

#endif
