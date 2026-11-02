#ifndef OPENMW_COMPONENTS_RESOURCE_PHYSICSSHAPE_H
#define OPENMW_COMPONENTS_RESOURCE_PHYSICSSHAPE_H

#include <array>
#include <map>
#include <memory>

#include <osg/Matrixd>
#include <osg/Object>
#include <osg/Vec3f>
#include <osg/ref_ptr>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>

#include <components/vfs/pathutil.hpp>

namespace NifJolt
{
    class JoltNifLoader;
}

namespace Resource
{
    using CollisionShapePtr = JPH::Ref<JPH::Shape>;

    struct CollisionBox
    {
        osg::Vec3f mExtents;
        osg::Vec3f mCenter;
    };

    enum class VisualCollisionType
    {
        None,
        Default,
        Camera
    };

    struct PhysicsShape : public osg::Object
    {
        CollisionShapePtr mCollisionShape;
        CollisionShapePtr mAvoidCollisionShape;

        // Used for actors and projectiles. mCollisionShape is used for actors only when we need to autogenerate
        // collision box for creatures. For now, use one file <-> one resource for simplicity.
        CollisionBox mCollisionBox;

        // Stores animated collision shapes.
        // mCollisionShape is a JPH::MutableCompoundShape (which consists of one or more child shapes).
        // In this map, for each animated collision shape,
        // we store the node's record index mapped to the child index of the shape in the JPH::MutableCompoundShape.
        std::map<int, int> mAnimatedShapes;
        // Maps shape record index to controlling NiCollisionSwitch record index.
        std::map<int, int> mCollisionSwitches;

        VFS::Path::Normalized mFileName;
        std::string mFileHash;

        VisualCollisionType mVisualCollisionType = VisualCollisionType::None;

        PhysicsShape() = default;
        // Note this is always a shallow copy and the copy will not autodelete underlying vertex data
        PhysicsShape(const PhysicsShape& other, const osg::CopyOp& copyOp = osg::CopyOp());

        META_Object(Resource, PhysicsShape)

        bool isAnimated() const { return !mAnimatedShapes.empty(); }
    };

    // An instance of a PhysicsShape that may have its own unique scaling set on collision shapes.
    // Vertex data is shallow-copied where possible. A ref_ptr to the original shape is held to keep vertex pointers
    // intact.
    class PhysicsShapeInstance : public PhysicsShape
    {
    public:
        explicit PhysicsShapeInstance(osg::ref_ptr<const PhysicsShape> source);

        const osg::ref_ptr<const PhysicsShape>& getSource() const { return mSource; }

    private:
        osg::ref_ptr<const PhysicsShape> mSource;
    };

    osg::ref_ptr<PhysicsShapeInstance> makeInstance(osg::ref_ptr<const PhysicsShape> source);

    struct TriangleMeshShape
    {
        JPH::MeshShapeSettings* m_meshInterface;
        TriangleMeshShape(JPH::MeshShapeSettings* meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true)
        {
            m_meshInterface = meshInterface;
        }

        virtual ~TriangleMeshShape() { delete m_meshInterface; }
    };
}

#endif
