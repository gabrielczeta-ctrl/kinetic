#include <algorithm>

#include <osg/Geometry>
#include <osg/Group>
#include <osg/Material>

#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>
#include <components/sceneutil/depth.hpp>
#include <osg/PolygonMode>
#include <osg/PolygonOffset>
#include <osg/ShapeDrawable>
#include <osg/StateSet>

#include "joltdebugdraw.hpp"
#include "vismask.hpp"

#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>

#include "../mwbase/environment.hpp"

using namespace JPH;

namespace MWRender
{
    JoltDebugDrawer::JoltDebugDrawer(
        osg::ref_ptr<osg::Group> parentNode, JPH::PhysicsSystem* physicsSystem, int debugMode)
        : mParentNode(std::move(parentNode))
        , mPhysicsSystem(physicsSystem)
    {
        JoltDebugDrawer::setDebugMode(debugMode);

        JPH::DebugRenderer::Initialize();

        // Keep expensive/verbose overlays disabled by default for frame-time stability.
        mBodyDrawSettings.mDrawCenterOfMassTransform = false;
        mBodyDrawSettings.mDrawGetSupportFunction = false;
        mBodyDrawSettings.mDrawSupportDirection = false;
        mBodyDrawSettings.mDrawGetSupportingFace = false;
        mBodyDrawSettings.mDrawBoundingBox = false;
        mBodyDrawSettings.mDrawWorldTransform = false;
        mBodyDrawSettings.mDrawMassAndInertia = false;
        mBodyDrawSettings.mDrawSleepStats = false;
        mBodyDrawSettings.mDrawShape = true;
    }

    JoltDebugDrawer::~JoltDebugDrawer()
    {
        destroyGeometry();
    }

    void JoltDebugDrawer::createGeometry()
    {
        if (!mLinesGeometry)
        {
            stateSet = new osg::StateSet;
            mLinesGeometry = new osg::Geometry;
            mTrisGeometry = new osg::Geometry;
            mLinesGeometry->setNodeMask(Mask_Debug);
            mTrisGeometry->setNodeMask(Mask_Debug);

            mLinesVertices = new osg::Vec3Array;
            mTrisVertices = new osg::Vec3Array;
            mLinesColors = new osg::Vec4Array;

            mLinesDrawArrays = new osg::DrawArrays(osg::PrimitiveSet::LINES);
            mTrisDrawArrays = new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES);

            mLinesGeometry->setUseDisplayList(false);
            mLinesGeometry->setVertexArray(mLinesVertices);
            mLinesGeometry->setColorArray(mLinesColors);
            mLinesGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
            mLinesGeometry->setDataVariance(osg::Object::DYNAMIC);
            mLinesGeometry->addPrimitiveSet(mLinesDrawArrays);

            mTrisGeometry->setUseDisplayList(false);
            mTrisGeometry->setVertexArray(mTrisVertices);
            mTrisGeometry->setDataVariance(osg::Object::DYNAMIC);
            mTrisGeometry->addPrimitiveSet(mTrisDrawArrays);

            mParentNode->addChild(mLinesGeometry);
            mParentNode->addChild(mTrisGeometry);

            stateSet->setAttributeAndModes(
                new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE),
                osg::StateAttribute::ON);
            stateSet->setAttributeAndModes(new osg::PolygonOffset(
                SceneUtil::AutoDepth::isReversed() ? 1.0 : -1.0, SceneUtil::AutoDepth::isReversed() ? 1.0 : -1.0));
            osg::ref_ptr<osg::Material> material = new osg::Material;
            material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
            stateSet->setAttribute(material);
            mLinesGeometry->setStateSet(stateSet);
            mTrisGeometry->setStateSet(stateSet);
            mShapesRoot = new osg::Group;
            mShapesRoot->setStateSet(stateSet);
            mShapesRoot->setDataVariance(osg::Object::DYNAMIC);
            mShapesRoot->setNodeMask(Mask_Debug);
            mParentNode->addChild(mShapesRoot);

            MWBase::Environment::get().getResourceSystem()->getSceneManager()->recreateShaders(mLinesGeometry, "debug");
            MWBase::Environment::get().getResourceSystem()->getSceneManager()->recreateShaders(mTrisGeometry, "debug");
            MWBase::Environment::get().getResourceSystem()->getSceneManager()->recreateShaders(mShapesRoot, "debug");

            // Create empty batch of triagnles
            Vertex emptyVertex{ JPH::Float3(0, 0, 0), JPH::Float3(1, 0, 0), JPH::Float2(0, 0), Color::sWhite };
            uint32_t emptyIndices[] = { 0, 0, 0 };
            mEmptyBatch = CreateTriangleBatch(&emptyVertex, 1, emptyIndices, 3);
        }
    }

    void JoltDebugDrawer::destroyGeometry()
    {
        if (mLinesGeometry)
        {
            mParentNode->removeChild(mLinesGeometry);
            mParentNode->removeChild(mTrisGeometry);
            mParentNode->removeChild(mShapesRoot);
            mLinesGeometry = nullptr;
            mLinesVertices = nullptr;
            mLinesColors = nullptr;
            mLinesDrawArrays = nullptr;
            mTrisGeometry = nullptr;
            mTrisVertices = nullptr;
            mTrisDrawArrays = nullptr;
        }
    }

    void JoltDebugDrawer::addCollision(const osg::Vec3f& orig, const osg::Vec3f& normal)
    {
        std::scoped_lock lock(mCollisionMutex);
        mCollisionViews.emplace_back(orig, normal);
    }

    void JoltDebugDrawer::showCollisions()
    {
        std::scoped_lock lock(mCollisionMutex);
        const auto now = std::chrono::steady_clock::now();
        for (auto& [from, to, created] : mCollisionViews)
        {
            if (now - created < std::chrono::seconds(2))
            {
                mLinesVertices->push_back(from);
                mLinesVertices->push_back(to);
                mLinesColors->push_back({ 1, 0, 0, 1 });
                mLinesColors->push_back({ 1, 0, 0, 1 });
            }
        }
        mCollisionViews.erase(
            std::remove_if(mCollisionViews.begin(), mCollisionViews.end(),
                [&now](const CollisionView& view) { return now - view.mCreated >= std::chrono::seconds(2); }),
            mCollisionViews.end());
    }

    void JoltDebugDrawer::step()
    {
        if (mDebugOn)
        {
            mLinesVertices->clear();
            mTrisVertices->clear();
            mLinesColors->clear();
            mShapesRoot->removeChildren(0, mShapesRoot->getNumChildren());
            mPhysicsSystem->DrawBodies(mBodyDrawSettings, static_cast<JPH::DebugRenderer*>(this));
            mPhysicsSystem->DrawConstraints(static_cast<JPH::DebugRenderer*>(this));
            mPhysicsSystem->DrawConstraintLimits(static_cast<JPH::DebugRenderer*>(this));
            mPhysicsSystem->DrawConstraintReferenceFrame(static_cast<JPH::DebugRenderer*>(this));
            showCollisions();
            mLinesDrawArrays->setCount(mLinesVertices->size());
            mTrisDrawArrays->setCount(mTrisVertices->size());
            mLinesVertices->dirty();
            mTrisVertices->dirty();
            mLinesColors->dirty();
            mLinesGeometry->dirtyBound();
            mTrisGeometry->dirtyBound();
        }
    }

    void JoltDebugDrawer::setDebugMode(int isOn)
    {
        mDebugOn = (isOn != 0);

        if (!mDebugOn)
            destroyGeometry();
        else
            createGeometry();
    }

    int JoltDebugDrawer::getDebugMode() const
    {
        return mDebugOn;
    }

    void JoltDebugDrawer::DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor)
    {
        mLinesVertices->push_back(osg::Vec3f(inFrom.GetX(), inFrom.GetY(), inFrom.GetZ()));
        mLinesVertices->push_back(osg::Vec3f(inTo.GetX(), inTo.GetY(), inTo.GetZ()));
        mLinesColors->push_back({ (float)inColor.r / 255.0f, (float)inColor.g / 255.0f, (float)inColor.b / 255.0f, 1 });
        mLinesColors->push_back({ (float)inColor.r / 255.0f, (float)inColor.g / 255.0f, (float)inColor.b / 255.0f, 1 });
    }

    void JoltDebugDrawer::DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3,
        JPH::ColorArg inColor, JPH::DebugRenderer::ECastShadow inCastShadow)
    {
        mTrisVertices->push_back(Misc::Convert::toOsg(inV1));
        mTrisVertices->push_back(Misc::Convert::toOsg(inV2));
        mTrisVertices->push_back(Misc::Convert::toOsg(inV3));
    }

    JPH::DebugRenderer::Batch JoltDebugDrawer::CreateTriangleBatch(
        const JPH::DebugRenderer::Triangle* inTriangles, int inTriangleCount)
    {
        if (inTriangles == nullptr || inTriangleCount == 0)
            return mEmptyBatch;

        // Create a new OSG geometry object
        BatchImpl* primitive = new BatchImpl();
        primitive->mGeometry = new osg::Geometry();

        // Each triangle has 3 vertices
        int inVertexCount = inTriangleCount * 3;

        // Create a vertex array
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(inVertexCount);
        osg::ref_ptr<osg::DrawElementsUInt> indices
            = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, inVertexCount);

        for (int i = 0; i < inTriangleCount; ++i)
        {
            const JPH::DebugRenderer::Triangle& triangle = inTriangles[i];
            for (int j = 0; j < 3; ++j) // Each triangle has 3 vertices
            {
                const JPH::DebugRenderer::Vertex& vertex = triangle.mV[j];
                (*vertices)[i * 3 + j].set(vertex.mPosition.x, vertex.mPosition.y, vertex.mPosition.z);
                (*indices)[i * 3 + j] = i * 3 + j; // Direct mapping of vertex to index
            }
        }

        primitive->mGeometry->setVertexArray(vertices.get());
        primitive->mGeometry->addPrimitiveSet(indices.get());
        primitive->mGeometry->setUseDisplayList(false);
        primitive->mGeometry->setDataVariance(osg::Object::STATIC);
        primitive->mGeometry->setStateSet(stateSet);

        MWBase::Environment::get().getResourceSystem()->getSceneManager()->recreateShaders(
            primitive->mGeometry, "debug");

        return primitive;
    }

    JPH::DebugRenderer::Batch JoltDebugDrawer::CreateTriangleBatch(
        const JPH::DebugRenderer::Vertex* inVertices, int inVertexCount, const uint32_t* inIndices, int inIndexCount)
    {
        if (inVertices == nullptr || inVertexCount == 0 || inIndices == nullptr || inIndexCount == 0)
            return mEmptyBatch;

        // Create a new OSG geometry object
        BatchImpl* primitive = new BatchImpl(); // always a triangle list topology
        primitive->mGeometry = new osg::Geometry();

        // Create a vertex array and fill it with the vertices from inVertices
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(inVertexCount);
        for (int i = 0; i < inVertexCount; ++i)
        {
            const JPH::DebugRenderer::Vertex& vertex = inVertices[i];
            (*vertices)[i].set(vertex.mPosition.x, vertex.mPosition.y, vertex.mPosition.z);
        }

        // Create a new DrawElementsUInt object for the indices
        osg::ref_ptr<osg::DrawElementsUInt> indices
            = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, inIndexCount);
        for (int i = 0; i < inIndexCount; ++i)
        {
            (*indices)[i] = inIndices[i];
        }

        primitive->mGeometry->setVertexArray(vertices.get());
        primitive->mGeometry->addPrimitiveSet(indices.get());
        primitive->mGeometry->setUseDisplayList(false);
        primitive->mGeometry->setDataVariance(osg::Object::STATIC);
        primitive->mGeometry->setStateSet(stateSet);

        MWBase::Environment::get().getResourceSystem()->getSceneManager()->recreateShaders(
            primitive->mGeometry, "debug");

        return primitive;
    }

    void JoltDebugDrawer::DrawGeometry(JPH::RMat44Arg inModelMatrix, const JPH::AABox& inWorldSpaceBounds,
        float inLODScaleSq, JPH::ColorArg inModelColor, const JPH::DebugRenderer::GeometryRef& inGeometry,
        JPH::DebugRenderer::ECullMode inCullMode, JPH::DebugRenderer::ECastShadow inCastShadow,
        JPH::DebugRenderer::EDrawMode inDrawMode)
    {
        Mat44 model_matrix = inModelMatrix.ToMat44();

        // Draw all geometry LODs
        const Array<LOD>& geometry_lods = inGeometry->mLODs;
        for (size_t lod = 0; lod < geometry_lods.size(); ++lod)
        {
            // Handle for a batch of triangles
            BatchImpl* batchPtr = static_cast<BatchImpl*>(geometry_lods[lod].mTriangleBatch.GetPtr());
            osg::ref_ptr<osg::MatrixTransform> transformNode = new osg::MatrixTransform();

            // Set the transformation matrix for this instance
            osg::Matrix mat = osg::Matrix::identity();
            for (uint8_t x = 0; x < 4; x++)
                for (uint8_t y = 0; y < 4; y++)
                    mat(x, y) = model_matrix(y, x);

            transformNode->setMatrix(mat);
            transformNode->addChild(batchPtr->mGeometry.get());
            mShapesRoot->addChild(transformNode.get());
        }
    }

    void JoltDebugDrawer::DrawText3D(
        JPH::RVec3Arg inPosition, const std::string_view& inString, JPH::ColorArg inColor, float inHeight)
    {
        // No implementation
    }
}
