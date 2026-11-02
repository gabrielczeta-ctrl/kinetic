#include "../nif/node.hpp"

#include <components/nif/data.hpp>
#include <components/nif/extra.hpp>
#include <components/nif/node.hpp>
#include <components/nifjolt/joltnifloader.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <ranges>

namespace
{
    using namespace Nif::Testing;

    constexpr VFS::Path::NormalizedView testNif("test.nif");

    struct TestJoltNifLoader : testing::Test
    {
        NifJolt::JoltNifLoader mLoader;
        Nif::NiNode mRoot;
        Nif::NiNode mNode2;
        Nif::NiNode mNode3;
        Nif::NiTriShapeData mTriData1;
        Nif::NiTriShape mTri1;
        Nif::NiTriShapeData mTriData2;
        Nif::NiTriShape mTri2;
        Nif::NiStringExtraData mRcnExtraData;

        TestJoltNifLoader()
        {
            init(mRoot);
            init(mNode2);
            init(mNode3);
            init(mTri1);
            init(mTri2);
            init(mRcnExtraData);

            mTriData1.mRecordType = Nif::RC_NiTriShapeData;
            mTriData1.mVertices = { osg::Vec3f(0, 0, 0), osg::Vec3f(1, 0, 0), osg::Vec3f(1, 1, 0) };
            mTriData1.mNumTriangles = 1;
            mTriData1.mTriangles = { 0, 1, 2 };
            mTri1.mData = Nif::NiGeometryDataPtr(&mTriData1);

            mTriData2.mRecordType = Nif::RC_NiTriShapeData;
            mTriData2.mVertices = { osg::Vec3f(0, 0, 1), osg::Vec3f(1, 0, 1), osg::Vec3f(1, 1, 1) };
            mTriData2.mNumTriangles = 1;
            mTriData2.mTriangles = { 0, 1, 2 };
            mTri2.mData = Nif::NiGeometryDataPtr(&mTriData2);

            mRcnExtraData.mData = "RCN";
            mRcnExtraData.mRecordType = Nif::RC_NiStringExtraData;
        }

        static void attach(Nif::NiNode& parent, Nif::NiAVObject& child)
        {
            parent.mChildren.emplace_back(&child);
            child.mParents.push_back(&parent);
        }

        static void expectCollisionBounds(const osg::ref_ptr<Resource::PhysicsShape>& result,
            JPH::uint expectedSubShapes, float expectedMinZ, float expectedMaxZ)
        {
            ASSERT_NE(result->mCollisionShape, nullptr);
            const JPH::Shape& shape = *result->mCollisionShape;
            const auto* compound = dynamic_cast<const JPH::CompoundShape*>(&shape);
            EXPECT_EQ(compound ? compound->GetNumSubShapes() : 1, expectedSubShapes);

            const JPH::AABox bounds = shape.GetLocalBounds();
            EXPECT_NEAR(bounds.mMin.GetZ(), expectedMinZ, 1e-4f);
            EXPECT_NEAR(bounds.mMax.GetZ(), expectedMaxZ, 1e-4f);
        }

        osg::ref_ptr<Resource::PhysicsShape> load()
        {
            Nif::NIFFile file(testNif);
            file.mRoots.push_back(&mRoot);
            return mLoader.load(file);
        }
    };

    TEST_F(TestJoltNifLoader, dont_assign_invalid_bounding_box_extents)
    {
        mTri2.mName = "Bounding Box";
        mTri2.mBounds.mType = Nif::BoundingVolume::Type::BOX_BV;
        mTri2.mBounds.mBox.mExtents = osg::Vec3f(-1, -2, -3);
        attach(mRoot, mTri1);
        attach(mRoot, mTri2);

        const auto result = load();

        EXPECT_TRUE(std::ranges::all_of(result->mCollisionBox.mExtents._v, [](float extent) { return extent == 0.f; }));
    }

    TEST_F(TestJoltNifLoader, no_rcn_flag_with_immediate_rcn_uses_rcn_for_collision)
    {
        Nif::NiNode rcn;
        init(rcn);
        rcn.mRecordType = Nif::RC_RootCollisionNode;
        attach(mRoot, mTri1);
        attach(mRoot, rcn);
        attach(rcn, mTri2);

        expectCollisionBounds(load(), 1, 1, 1);
    }

    TEST_F(TestJoltNifLoader, no_rcn_flag_with_nested_rcn_ignores_rcn_and_uses_all_for_collision)
    {
        Nif::NiNode rcn;
        init(rcn);
        rcn.mRecordType = Nif::RC_RootCollisionNode;
        attach(mRoot, mTri1);
        attach(mRoot, mNode2);
        attach(mNode2, rcn);
        attach(rcn, mTri2);

        expectCollisionBounds(load(), 2, 0, 1);
    }

    TEST_F(TestJoltNifLoader, rcn_flag_with_nested_rcn_uses_nested_rcn_for_collision)
    {
        Nif::NiNode rcn;
        init(rcn);
        rcn.mRecordType = Nif::RC_RootCollisionNode;
        mRoot.mExtra = Nif::ExtraPtr(&mRcnExtraData);
        attach(mRoot, mTri1);
        attach(mRoot, mNode2);
        attach(mNode2, rcn);
        attach(rcn, mTri2);

        expectCollisionBounds(load(), 1, 1, 1);
    }

    TEST_F(TestJoltNifLoader, rcn_flag_with_multiple_nested_rcns_uses_last_rcn)
    {
        Nif::NiNode rcn1;
        init(rcn1);
        rcn1.mRecordType = Nif::RC_RootCollisionNode;
        Nif::NiNode rcn2;
        init(rcn2);
        rcn2.mRecordType = Nif::RC_RootCollisionNode;
        mRoot.mExtra = Nif::ExtraPtr(&mRcnExtraData);
        attach(mRoot, mNode2);
        attach(mRoot, mNode3);
        attach(mNode2, rcn1);
        attach(rcn1, mTri1);
        attach(mNode3, rcn2);
        attach(rcn2, mTri2);

        expectCollisionBounds(load(), 1, 1, 1);
    }
}
