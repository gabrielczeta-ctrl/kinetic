#include "joltnifloader.hpp"

#include <cassert>
#include <sstream>
#include <tuple>
#include <variant>
#include <vector>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>

#include <components/debug/debuglog.hpp>
#include <components/files/conversion.hpp>
#include <components/misc/convert.hpp>
#include <components/misc/strings/algorithm.hpp>
#include <components/nif/extra.hpp>
#include <components/nif/nifstream.hpp>
#include <components/nif/node.hpp>
#include <components/nif/parent.hpp>

namespace
{

    bool pathFileNameStartsWithX(const std::string& path)
    {
        const std::size_t slashpos = path.find_last_of("/\\");
        const std::size_t letterPos = slashpos == std::string::npos ? 0 : slashpos + 1;
        return letterPos < path.size() && (path[letterPos] == 'x' || path[letterPos] == 'X');
    }

}

namespace NifJolt
{
    // Some nif scales aren't exactly 1.0f, 1.0f, 1.0f but within a small epsilon
    // detect that so we can skip potentially expensive scaling
    bool isScaleUniformAndCloseToOne(const osg::Vec3& localScale)
    {
        const float targetScale = 1.0f;
        const float epsilon = 0.0001f;

        // Check if each component of localScale is within epsilon of targetScale
        bool scaleXCloseToOne = std::abs(localScale.x() - targetScale) < epsilon;
        bool scaleYCloseToOne = std::abs(localScale.y() - targetScale) < epsilon;
        bool scaleZCloseToOne = std::abs(localScale.z() - targetScale) < epsilon;

        // Return true only if all components are within epsilon of 1.0f
        return scaleXCloseToOne && scaleYCloseToOne && scaleZCloseToOne;
    }

    osg::ref_ptr<Resource::PhysicsShape> JoltNifLoader::load(Nif::FileView nif)
    {
        mShape = new Resource::PhysicsShape;

        mCompoundShape.reset();
        mAvoidCompoundShape.reset();

        mShape->mFileHash = nif.getHash();

        const size_t numRoots = nif.numRoots();
        std::vector<const Nif::NiAVObject*> roots;
        for (size_t i = 0; i < numRoots; ++i)
        {
            const Nif::Record* r = nif.getRoot(i);
            if (!r)
                continue;
            const Nif::NiAVObject* node = dynamic_cast<const Nif::NiAVObject*>(r);
            if (node)
                roots.emplace_back(node);
        }
        mShape->mFileName = nif.getFilename();
        if (roots.empty())
        {
            warn("Found no root nodes in NIF file " + mShape->mFileName.value());
            return mShape;
        }

        for (const Nif::NiAVObject* node : roots)
            if (findBoundingBox(*node))
                break;

        HandleNodeArgs args;

        // files with the name convention xmodel.nif usually have keyframes stored in a separate file xmodel.kf (see
        // Animation::addAnimSource). assume all nodes in the file will be animated
        // TODO: investigate whether this should and could be optimized.
        args.mAnimated = pathFileNameStartsWithX(mShape->mFileName);

        for (const Nif::NiAVObject* node : roots)
            handleRoot(nif, *node, args);

        if (mCompoundShape)
            mShape->mCollisionShape = mCompoundShape.get()->Create().Get();

        if (mAvoidCompoundShape)
            mShape->mAvoidCollisionShape = mAvoidCompoundShape.get()->Create().Get();

        return mShape;
    }

    // Find a bounding box in the node hierarchy to use for actor collision
    bool JoltNifLoader::findBoundingBox(const Nif::NiAVObject& node)
    {
        if (Misc::StringUtils::ciEqual(node.mName, "Bounding Box"))
        {
            if (node.mBounds.mType == Nif::BoundingVolume::Type::BOX_BV)
            {
                mShape->mCollisionBox.mExtents = node.mBounds.mBox.mExtents;
                mShape->mCollisionBox.mCenter = node.mBounds.mBox.mCenter;
            }
            else
            {
                warn("Invalid Bounding Box node bounds in file " + mShape->mFileName.value());
            }
            return true;
        }

        if (auto ninode = dynamic_cast<const Nif::NiNode*>(&node))
            for (const auto& child : ninode->mChildren)
                if (!child.empty() && findBoundingBox(child.get()))
                    return true;

        return false;
    }

    void JoltNifLoader::handleRoot(Nif::FileView nif, const Nif::NiAVObject& node, HandleNodeArgs args)
    {
        // Gamebryo/Bethbryo meshes
        if (nif.getVersion() >= Nif::NIFStream::generateVersion(10, 0, 1, 0))
        {
            // Handle BSXFlags
            const Nif::NiIntegerExtraData* bsxFlags = nullptr;
            for (const auto& e : node.getExtraList())
            {
                if (e->mRecordType == Nif::RC_BSXFlags)
                {
                    bsxFlags = static_cast<const Nif::NiIntegerExtraData*>(e.getPtr());
                    break;
                }
            }

            // Collision flag
            if (!bsxFlags || !(bsxFlags->mData & 2))
                return;

            // Editor marker flag
            if (bsxFlags->mData & 32)
                args.mHasMarkers = true;

            // FIXME: hack, using rendered geometry instead of Bethesda Havok data
            args.mAutogenerated = true;
        }
        // Pre-Gamebryo meshes
        else
        {
            // Handle RootCollisionNode
            const Nif::NiNode* colNode = nullptr;
            if (const Nif::NiNode* ninode = dynamic_cast<const Nif::NiNode*>(&node))
            {
                for (const auto& child : ninode->mChildren)
                {
                    if (!child.empty() && child.getPtr()->mRecordType == Nif::RC_RootCollisionNode)
                    {
                        colNode = static_cast<const Nif::NiNode*>(child.getPtr());
                        break;
                    }
                }
            }

            args.mAutogenerated = colNode == nullptr;

            // Check for extra data
            for (const auto& e : node.getExtraList())
            {
                if (e->mRecordType == Nif::RC_NiStringExtraData)
                {
                    // String markers may contain important information
                    // affecting the entire subtree of this node
                    auto sd = static_cast<const Nif::NiStringExtraData*>(e.getPtr());

                    // Editor marker flag
                    if (sd->mData == "MRK")
                        args.mHasTriMarkers = true;
                    else if (Misc::StringUtils::ciStartsWith(sd->mData, "NC"))
                    {
                        // NC prefix is case-insensitive but the second C in NCC flag needs be uppercase.

                        // Collide only with camera.
                        if (sd->mData.length() > 2 && sd->mData[2] == 'C')
                            mShape->mVisualCollisionType = Resource::VisualCollisionType::Camera;
                        // No collision.
                        else
                            mShape->mVisualCollisionType = Resource::VisualCollisionType::Default;
                    }
                }
            }

            // FIXME: JoltNifLoader should never have to provide rendered geometry for camera collision
            if (colNode && colNode->mChildren.empty())
            {
                args.mAutogenerated = true;
                mShape->mVisualCollisionType = Resource::VisualCollisionType::Camera;
            }
        }

        handleNode(node, nullptr, args);
    }

    void JoltNifLoader::handleNode(const Nif::NiAVObject& node, const Nif::Parent* parent, HandleNodeArgs args)
    {
        // TODO: allow on-the fly collision switching via toggling this flag
        if (node.mRecordType == Nif::RC_NiCollisionSwitch && !node.collisionActive())
            return;

        for (Nif::NiTimeControllerPtr ctrl = node.mController; !ctrl.empty(); ctrl = ctrl->mNext)
        {
            if (args.mAnimated)
                break;
            if (!ctrl->isActive())
                continue;
            switch (ctrl->mRecordType)
            {
                case Nif::RC_NiKeyframeController:
                case Nif::RC_NiPathController:
                case Nif::RC_NiRollController:
                    args.mAnimated = true;
                    break;
                default:
                    continue;
            }
        }

        if (node.mRecordType == Nif::RC_RootCollisionNode)
        {
            if (args.mAutogenerated)
            {
                // Encountered a RootCollisionNode inside an autogenerated mesh.

                // We treat empty RootCollisionNodes as NCC flag (set collisionType to `Camera`)
                // and generate the camera collision shape based on rendered geometry.
                if (mShape->mVisualCollisionType == Resource::VisualCollisionType::Camera)
                    return;

                // Otherwise we'll want to notify the user.
                Log(Debug::Info) << "JoltNifLoader: RootCollisionNode is not attached to the root node in "
                                 << mShape->mFileName << ". Treating it as a NiNode.";
            }
            else
            {
                args.mIsCollisionNode = true;
            }
        }

        // Don't collide with AvoidNode shapes
        if (node.mRecordType == Nif::RC_AvoidNode)
            args.mAvoid = true;

        if (args.mAutogenerated || args.mIsCollisionNode)
        {
            auto geometry = dynamic_cast<const Nif::NiGeometry*>(&node);
            if (geometry)
                handleGeometry(*geometry, parent, args);
        }

        // For NiNodes, loop through children
        if (const Nif::NiNode* ninode = dynamic_cast<const Nif::NiNode*>(&node))
        {
            const Nif::Parent currentParent{ *ninode, parent };
            for (const auto& child : ninode->mChildren)
            {
                if (!child.empty())
                {
                    assert(std::find(child->mParents.begin(), child->mParents.end(), ninode) != child->mParents.end());
                    handleNode(child.get(), &currentParent, args);
                }
                // For NiSwitchNodes and NiFltAnimationNodes, only use the first child
                // TODO: must synchronize with the rendering scene graph somehow
                // Doing this for NiLODNodes is unsafe (the first level might not be the closest)
                if (node.mRecordType == Nif::RC_NiSwitchNode || node.mRecordType == Nif::RC_NiFltAnimationNode)
                    break;
            }
        }
    }

    void JoltNifLoader::handleGeometry(
        const Nif::NiGeometry& niGeometry, const Nif::Parent* nodeParent, HandleNodeArgs args)
    {
        // This flag comes from BSXFlags
        if (args.mHasMarkers && Misc::StringUtils::ciStartsWith(niGeometry.mName, "EditorMarker"))
            return;

        // This flag comes from Morrowind
        if (args.mHasTriMarkers && Misc::StringUtils::ciStartsWith(niGeometry.mName, "Tri EditorMarker"))
            return;

        if (!niGeometry.mSkin.empty())
            args.mAnimated = false;

        std::unique_ptr<JPH::MeshShapeSettings> childShape = niGeometry.getCollisionShape();
        if (childShape == nullptr)
            return;

        osg::Matrixf transform = niGeometry.mTransform.toMatrix();
        for (const Nif::Parent* parent = nodeParent; parent != nullptr; parent = parent->mParent)
            transform *= parent->mNiNode.mTransform.toMatrix();

        osg::Vec3f localScale = transform.getScale();
        transform.orthoNormalize(transform);

        const osg::Vec3f& osgPos = transform.getTrans();
        const osg::Quat& osgQuat = transform.getRotate();

        // Try create shape, Jolt will validate and give error if it failed (it shouldnt usually)
        auto createdRef = childShape->Create();
        if (createdRef.HasError())
        {
            // Remove degenerate and duplicate triangles, then try again
            childShape->ClearCachedResult();
            childShape->Sanitize();

            createdRef = childShape->Create();
            if (createdRef.HasError())
            {
                Log(Debug::Error) << "JoltNifLoader mesh error: " << createdRef.GetError();
                return;
            }
        }

        // Scaled shapes require wrapping the initial shape in a ScaledShape
        auto finalShape = createdRef.Get();
        bool isScaledShape = !isScaleUniformAndCloseToOne(localScale);
        if (isScaledShape)
        {
            JPH::ScaledShapeSettings scaledSettings(createdRef.Get(), Misc::Convert::toJolt<JPH::Vec3>(localScale));
            createdRef = scaledSettings.Create();
        }

        // TODO: determine if has any animation in collision object at all so parent can be mutable
        mShapeMutable = true;

        if (!args.mAvoid)
        {
            if (!mCompoundShape)
            {
                if (mShapeMutable)
                {
                    // This shape is optimized for adding / removing and changing the rotation / translation of sub
                    // shapes but is less efficient in querying.
                    mCompoundShape.reset(new JPH::MutableCompoundShapeSettings);
                }
                else
                {
                    mCompoundShape.reset(new JPH::StaticCompoundShapeSettings);
                }
            }

            if (args.mAnimated)
                mShape->mAnimatedShapes.emplace(niGeometry.mRecordIndex, mCompoundShape->mSubShapes.size());

            mCompoundShape->AddShape(JPH::Vec3(osgPos.x(), osgPos.y(), osgPos.z()),
                JPH::Quat(osgQuat.x(), osgQuat.y(), osgQuat.z(), osgQuat.w()), createdRef.Get());
        }
        else
        {
            if (!mAvoidCompoundShape)
            {
                if (mShapeMutable)
                {
                    // This shape is optimized for adding / removing and changing the rotation / translation of sub
                    // shapes but is less efficient in querying.
                    mAvoidCompoundShape.reset(new JPH::MutableCompoundShapeSettings);
                }
                else
                {
                    mAvoidCompoundShape.reset(new JPH::StaticCompoundShapeSettings);
                }
            }

            mAvoidCompoundShape->AddShape(JPH::Vec3(osgPos.x(), osgPos.y(), osgPos.z()),
                JPH::Quat(osgQuat.x(), osgQuat.y(), osgQuat.z(), osgQuat.w()), createdRef.Get());
        }

        // TODO: maybe we should delete childShape here instead of just forgetting about it
        std::ignore = childShape.release();
    }

} // namespace NifJolt
