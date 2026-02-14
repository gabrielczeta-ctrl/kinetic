#include "physicsshapemanager.hpp"

#include <array>
#include <cstring>
#include <cstdint>
#include <limits>

#include <osg/Drawable>
#include <osg/Geometry>
#include <osg/NodeVisitor>
#include <osg/Transform>
#include <osg/TriangleIndexFunctor>
#include <osg/TriangleFunctor>

#include <components/misc/osguservalues.hpp>
#include <components/misc/pathhelpers.hpp>
#include <components/sceneutil/visitor.hpp>
#include <components/vfs/manager.hpp>
#include <components/vfs/pathutil.hpp>

#include <components/nifjolt/joltnifloader.hpp>

#include <components/misc/convert.hpp>

#include "multiobjectcache.hpp"
#include "niffilemanager.hpp"
#include "objectcache.hpp"
#include "physicsshape.hpp"
#include "scenemanager.hpp"

namespace Resource
{
    struct CollectTriangleIndices
    {
        void operator()(unsigned int i0, unsigned int i1, unsigned int i2)
        {
            mTriangles.push_back({ i0, i1, i2 });
        }

        const std::vector<std::array<unsigned int, 3>>& getTriangles() const { return mTriangles; }

    private:
        std::vector<std::array<unsigned int, 3>> mTriangles;
    };

    bool appendIndexedTriangles(
        const osg::Geometry& geometry, const osg::Matrixf& matrix, JPH::MeshShapeSettings& triangleMesh)
    {
        const osg::Vec3Array* vertices = dynamic_cast<const osg::Vec3Array*>(geometry.getVertexArray());
        if (vertices == nullptr || vertices->empty())
            return false;

        osg::TriangleIndexFunctor<CollectTriangleIndices> indexFunctor;
        geometry.accept(indexFunctor);
        const auto& triangles = indexFunctor.getTriangles();
        if (triangles.empty())
            return false;

        const std::size_t baseVertex = triangleMesh.mTriangleVertices.size();
        if (baseVertex > std::numeric_limits<std::uint32_t>::max())
            return false;
        if (vertices->size() > std::numeric_limits<std::uint32_t>::max() - baseVertex)
            return false;

        triangleMesh.mTriangleVertices.reserve(baseVertex + vertices->size());
        for (const osg::Vec3& vertex : *vertices)
            triangleMesh.mTriangleVertices.push_back(Misc::Convert::toJolt<JPH::Float3>(matrix.preMult(vertex)));

        triangleMesh.mIndexedTriangles.reserve(triangleMesh.mIndexedTriangles.size() + triangles.size());
        const auto indexBase = static_cast<std::uint32_t>(baseVertex);
        for (const std::array<unsigned int, 3>& triangle : triangles)
            triangleMesh.mIndexedTriangles.emplace_back(indexBase + triangle[0], indexBase + triangle[1],
                indexBase + triangle[2]);

        return true;
    }

    struct GetTriangleFunctor
    {
        GetTriangleFunctor()
            : mTriMesh(nullptr)
        {
        }

        void setTriMesh(JPH::MeshShapeSettings* triMesh) { mTriMesh = triMesh; }

        void setMatrix(const osg::Matrixf& matrix) { mMatrix = matrix; }

        void inline operator()(const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3,
            bool _temp = false) // Note: unused temp argument left here for OSG versions less than 3.5.6
        {
            if (mTriMesh)
            {
                uint32_t sizeC = mTriMesh->mTriangleVertices.size();
                mTriMesh->mTriangleVertices.push_back(Misc::Convert::toJolt<JPH::Float3>(mMatrix.preMult(v1)));
                mTriMesh->mTriangleVertices.push_back(Misc::Convert::toJolt<JPH::Float3>(mMatrix.preMult(v2)));
                mTriMesh->mTriangleVertices.push_back(Misc::Convert::toJolt<JPH::Float3>(mMatrix.preMult(v3)));
                mTriMesh->mIndexedTriangles.push_back(JPH::IndexedTriangle(sizeC + 0, sizeC + 1, sizeC + 2));
            }
        }

        JPH::MeshShapeSettings* mTriMesh;
        osg::Matrixf mMatrix;
    };

    /// Creates a PhysicsShape out of a Node hierarchy.
    class NodeToShapeVisitor : public osg::NodeVisitor
    {
    public:
        NodeToShapeVisitor()
            : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
            , mTriangleMesh(nullptr)
        {
        }

        void apply(osg::Drawable& drawable) override
        {
            // NOTE: this assumes OSG-based physics shapes are not animated
            if (!mTriangleMesh)
                mTriangleMesh.reset(new JPH::MeshShapeSettings);

            osg::Matrixf worldMat = osg::computeLocalToWorld(getNodePath());

            if (const osg::Geometry* geometry = drawable.asGeometry())
                if (appendIndexedTriangles(*geometry, worldMat, *mTriangleMesh))
                    return;

            osg::TriangleFunctor<GetTriangleFunctor> functor;
            functor.setTriMesh(mTriangleMesh.get());
            functor.setMatrix(worldMat);
            drawable.accept(functor);
        }

        osg::ref_ptr<PhysicsShape> getShape()
        {
            if (!mTriangleMesh || mTriangleMesh->mTriangleVertices.size() == 0)
                return osg::ref_ptr<PhysicsShape>();

            osg::ref_ptr<PhysicsShape> shape(new PhysicsShape);

            auto triangleMeshShape = std::make_unique<TriangleMeshShape>(mTriangleMesh.release(), true);

            auto meshInterface = triangleMeshShape.release()->m_meshInterface;

            // Try create shape, Jolt will validate and give error if it failed (it shouldnt usually)
            auto createdRef = meshInterface->Create();
            if (createdRef.HasError())
            {
                // Remove degenerate and duplicate triangles, then try again
                meshInterface->ClearCachedResult();
                meshInterface->Sanitize();

                createdRef = meshInterface->Create();
                if (createdRef.HasError())
                {
                    Log(Debug::Error) << "PhysicsShapeManager mesh error: " << createdRef.GetError();
                    return osg::ref_ptr<PhysicsShape>();
                }
            }

            shape->mCollisionShape = createdRef.Get();

            JPH::AABox bounds = shape->mCollisionShape->GetLocalBounds();
            osg::Vec3f aabbMin = osg::Vec3f(bounds.mMin.GetX(), bounds.mMin.GetY(), bounds.mMin.GetZ());
            osg::Vec3f aabbMax = osg::Vec3f(bounds.mMax.GetX(), bounds.mMax.GetY(), bounds.mMax.GetZ());
            shape->mCollisionBox.mExtents[0] = (aabbMax[0] - aabbMin[0]) / 2.0f;
            shape->mCollisionBox.mExtents[1] = (aabbMax[1] - aabbMin[1]) / 2.0f;
            shape->mCollisionBox.mExtents[2] = (aabbMax[2] - aabbMin[2]) / 2.0f;
            shape->mCollisionBox.mCenter = osg::Vec3f(
                (aabbMax[0] + aabbMin[0]) / 2.0f, (aabbMax[1] + aabbMin[1]) / 2.0f, (aabbMax[2] + aabbMin[2]) / 2.0f);

            return shape;
        }

    private:
        std::unique_ptr<JPH::MeshShapeSettings> mTriangleMesh;
    };

    PhysicsShapeManager::PhysicsShapeManager(
        const VFS::Manager* vfs, SceneManager* sceneMgr, NifFileManager* nifFileManager, double expiryDelay)
        : ResourceManager(vfs, expiryDelay)
        , mInstanceCache(new MultiObjectCache)
        , mSceneManager(sceneMgr)
        , mNifFileManager(nifFileManager)
    {
    }

    PhysicsShapeManager::~PhysicsShapeManager() = default;

    osg::ref_ptr<const PhysicsShape> PhysicsShapeManager::getShape(VFS::Path::NormalizedView name)
    {
        if (osg::ref_ptr<osg::Object> obj = mCache->getRefFromObjectCache(name))
            return osg::ref_ptr<PhysicsShape>(static_cast<PhysicsShape*>(obj.get()));

        osg::ref_ptr<PhysicsShape> shape;

        if (Misc::getFileExtension(name.value()) == "nif")
        {
            NifJolt::JoltNifLoader loader(getVFS());
            shape = loader.load(*mNifFileManager->get(name));
        }
        else
        {
            osg::ref_ptr<const osg::Node> constNode(mSceneManager->getTemplate(name));
            // const-trickery required because there is no const version of NodeVisitor
            osg::ref_ptr<osg::Node> node(const_cast<osg::Node*>(constNode.get()));

            // Check first if there's a custom collision node
            unsigned int visitAllNodesMask = 0xffffffff;
            SceneUtil::FindByNameVisitor nameFinder("Collision");
            nameFinder.setTraversalMask(visitAllNodesMask);
            nameFinder.setNodeMaskOverride(visitAllNodesMask);
            node->accept(nameFinder);
            if (nameFinder.mFoundNode)
            {
                NodeToShapeVisitor visitor;
                visitor.setTraversalMask(visitAllNodesMask);
                visitor.setNodeMaskOverride(visitAllNodesMask);
                nameFinder.mFoundNode->accept(visitor);
                shape = visitor.getShape();
            }

            // Generate a collision shape from the mesh
            if (!shape)
            {
                NodeToShapeVisitor visitor;
                node->accept(visitor);
                shape = visitor.getShape();
                if (!shape)
                    return osg::ref_ptr<PhysicsShape>();
            }

            if (shape != nullptr)
            {
                shape->mFileName = name;
                constNode->getUserValue(Misc::OsgUserValues::sFileHash, shape->mFileHash);
            }
        }

        mCache->addEntryToObjectCache(name.value(), shape);

        return shape;
    }

    osg::ref_ptr<PhysicsShapeInstance> PhysicsShapeManager::cacheInstance(VFS::Path::NormalizedView name)
    {
        osg::ref_ptr<PhysicsShapeInstance> instance = createInstance(name);
        if (instance != nullptr)
            mInstanceCache->addEntryToObjectCache(name, instance.get());
        return instance;
    }

    osg::ref_ptr<PhysicsShapeInstance> PhysicsShapeManager::getInstance(VFS::Path::NormalizedView name)
    {
        if (osg::ref_ptr<osg::Object> obj = mInstanceCache->takeFromObjectCache(name))
            return static_cast<PhysicsShapeInstance*>(obj.get());
        return createInstance(name);
    }

    osg::ref_ptr<PhysicsShapeInstance> PhysicsShapeManager::createInstance(VFS::Path::NormalizedView name)
    {
        if (osg::ref_ptr<const PhysicsShape> shape = getShape(name))
            return makeInstance(std::move(shape));
        return osg::ref_ptr<PhysicsShapeInstance>();
    }

    void PhysicsShapeManager::updateCache(double referenceTime)
    {
        ResourceManager::updateCache(referenceTime);

        mInstanceCache->removeUnreferencedObjectsInCache();
    }

    void PhysicsShapeManager::clearCache()
    {
        ResourceManager::clearCache();

        mInstanceCache->clear();
    }

    void PhysicsShapeManager::reportStats(unsigned int frameNumber, osg::Stats* stats) const
    {
        Resource::reportStats("Shape", frameNumber, mCache->getStats(), *stats);
        Resource::reportStats("Shape Instance", frameNumber, mInstanceCache->getStats(), *stats);
    }

}
