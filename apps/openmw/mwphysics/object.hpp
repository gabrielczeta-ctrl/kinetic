#ifndef OPENMW_MWPHYSICS_OBJECT_H
#define OPENMW_MWPHYSICS_OBJECT_H

#include "ptrholder.hpp"

#include <osg/Node>

#include <map>
#include <mutex>
#include <vector>

namespace Resource
{
    class PhysicsShapeInstance;
}

namespace MWPhysics
{
    class PhysicsTaskScheduler;

    enum ScriptedCollisionType : char
    {
        ScriptedCollisionType_None = 0,
        ScriptedCollisionType_Actor = 1,
        // Note that this isn't 3, colliding with a player doesn't count as colliding with an actor
        ScriptedCollisionType_Player = 2
    };

    class Object final : public PtrHolder
    {
    public:
        Object(const MWWorld::Ptr& ptr, osg::ref_ptr<Resource::PhysicsShapeInstance> shapeInstance, osg::Quat rotation,
            int collisionType, PhysicsTaskScheduler* scheduler);
        ~Object() override;

        const Resource::PhysicsShapeInstance* getShapeInstance() const;
        void setScale(float scale);
        void setRotation(osg::Quat quat);
        void updatePosition();
        void commitPositionChange();
        osg::Matrixd getTransform() const;
        /// Return solid flag. Not used by the object itself, true by default.
        bool isSolid() const;
        void setSolid(bool solid);
        bool isAnimated() const;
        /// @brief update object shape
        /// @return true if shape changed
        bool animateCollisionShapes();
        bool collidedWith(ScriptedCollisionType type) const;
        void addCollision(ScriptedCollisionType type);
        void resetCollisions();
        static std::vector<const MWWorld::LiveCellRefBase*> consumeCollidedObjects();

        bool isScaleIdentity() const { return mScale == osg::Vec3f(1.0f, 1.0f, 1.0f); }

    private:
        osg::ref_ptr<Resource::PhysicsShapeInstance> mShapeInstance;
        JPH::Ref<JPH::Shape> mBasePhysicsShape;
        std::map<int, osg::NodePath> mRecIndexToNodePath;
        bool mSolid;
        osg::Vec3f mScale;
        osg::Vec3f mPosition;
        osg::Quat mRotation;
        bool mUsesScaledShape = false;
        bool mScaleUpdatePending = false;
        bool mTransformUpdatePending = false;
        mutable std::mutex mPositionMutex;
        PhysicsTaskScheduler* mTaskScheduler;
        char mCollidedWith;
    };
}

#endif
