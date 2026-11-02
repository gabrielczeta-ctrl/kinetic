#ifndef OPENMW_MWRENDER_JOLTDEBUGDRAW_H
#define OPENMW_MWRENDER_JOLTDEBUGDRAW_H

#include <atomic>
#include <chrono>
#include <mutex>
#include <vector>

#include <osg/Array>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/PrimitiveSet>
#include <osg/ref_ptr>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Renderer/DebugRenderer.h>

#include <Jolt/Core/FixedSizeFreeList.h>
#include <Jolt/Core/Mutex.h>
#include <Jolt/Core/Reference.h>
#include <Jolt/Core/UnorderedMap.h>

namespace MWRender
{
    class JoltDebugDrawer final : public JPH::DebugRenderer
    {
    private:
        struct CollisionView
        {
            osg::Vec3f mOrig;
            osg::Vec3f mEnd;
            std::chrono::time_point<std::chrono::steady_clock> mCreated;
            CollisionView(osg::Vec3f orig, osg::Vec3f normal)
                : mOrig(orig)
                , mEnd(orig + normal * 20)
                , mCreated(std::chrono::steady_clock::now())
            {
            }
        };

        class BatchImpl : public JPH::RefTargetVirtual
        {
        public:
            JPH_OVERRIDE_NEW_DELETE

            BatchImpl() {}

            BatchImpl(osg::ref_ptr<osg::Geometry> geom)
                : mGeometry(geom)
            {
            }

            virtual void AddRef() override { ++mRefCount; }
            virtual void Release() override
            {
                if (--mRefCount == 0)
                    delete this;
            }

            std::atomic<uint32_t> mRefCount = 0;
            osg::ref_ptr<osg::Geometry> mGeometry;
        };

        osg::ref_ptr<osg::Group> mParentNode;
        osg::ref_ptr<osg::Geometry> mLinesGeometry;
        osg::ref_ptr<osg::Geometry> mTrisGeometry;
        osg::ref_ptr<osg::Vec3Array> mLinesVertices;
        osg::ref_ptr<osg::Vec3Array> mTrisVertices;
        osg::ref_ptr<osg::Vec4Array> mLinesColors;
        osg::ref_ptr<osg::DrawArrays> mLinesDrawArrays;
        osg::ref_ptr<osg::DrawArrays> mTrisDrawArrays;
        osg::ref_ptr<osg::StateSet> stateSet;
        Batch mEmptyBatch;

        bool mDebugOn = false;

        std::mutex mCollisionMutex;
        JPH::Mutex mPrimitivesLock;

        JPH::BodyManager::DrawSettings mBodyDrawSettings;

        JPH::PhysicsSystem* mPhysicsSystem;
        std::vector<CollisionView> mCollisionViews;
        osg::ref_ptr<osg::Group> mShapesRoot;

        void createGeometry();
        void destroyGeometry();

    public:
        JPH_OVERRIDE_NEW_DELETE

        JoltDebugDrawer(osg::ref_ptr<osg::Group> parentNode, JPH::PhysicsSystem* physicsSystem, int debugMode = 1);
        ~JoltDebugDrawer();

        void step();

        void addCollision(const osg::Vec3f& orig, const osg::Vec3f& normal);

        void showCollisions();

        // 0 for off, anything else for on.
        void setDebugMode(int isOn);

        // 0 for off, anything else for on.
        int getDebugMode() const;

        // Jolt overrides
        virtual void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor);

        virtual void DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor,
            JPH::DebugRenderer::ECastShadow inCastShadow = JPH::DebugRenderer::ECastShadow::Off);

        virtual JPH::DebugRenderer::Batch CreateTriangleBatch(
            const JPH::DebugRenderer::Triangle* inTriangles, int inTriangleCount);

        virtual JPH::DebugRenderer::Batch CreateTriangleBatch(const JPH::DebugRenderer::Vertex* inVertices,
            int inVertexCount, const uint32_t* inIndices, int inIndexCount);

        virtual void DrawGeometry(JPH::RMat44Arg inModelMatrix, const JPH::AABox& inWorldSpaceBounds,
            float inLODScaleSq, JPH::ColorArg inModelColor, const JPH::DebugRenderer::GeometryRef& inGeometry,
            JPH::DebugRenderer::ECullMode inCullMode, JPH::DebugRenderer::ECastShadow inCastShadow,
            JPH::DebugRenderer::EDrawMode inDrawMode);

        virtual void DrawText3D(
            JPH::RVec3Arg inPosition, const std::string_view& inString, JPH::ColorArg inColor, float inHeight);
    };

}

#endif
