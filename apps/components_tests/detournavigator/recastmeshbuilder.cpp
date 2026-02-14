#include "operators.hpp"

#include <components/detournavigator/recastmesh.hpp>
#include <components/detournavigator/recastmeshbuilder.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>

#include <DetourCommon.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>

namespace DetourNavigator
{
    static inline bool operator==(const Water& lhs, const Water& rhs)
    {
        const auto tie = [](const Water& v) { return std::tie(v.mCellSize, v.mLevel); };
        return tie(lhs) == tie(rhs);
    }

    static inline bool operator==(const CellWater& lhs, const CellWater& rhs)
    {
        const auto tie = [](const CellWater& v) { return std::tie(v.mCellPosition, v.mWater); };
        return tie(lhs) == tie(rhs);
    }

    static inline bool operator==(const Heightfield& lhs, const Heightfield& rhs)
    {
        return makeTuple(lhs) == makeTuple(rhs);
    }

    static inline bool operator==(const FlatHeightfield& lhs, const FlatHeightfield& rhs)
    {
        const auto tie = [](const FlatHeightfield& v) { return std::tie(v.mCellPosition, v.mCellSize, v.mHeight); };
        return tie(lhs) == tie(rhs);
    }
}

namespace
{
    using namespace testing;
    using namespace DetourNavigator;

    struct DetourNavigatorRecastMeshBuilderTest : Test
    {
        TileBounds mBounds;
        const Version mVersion{ 0, 0 };
        const osg::ref_ptr<const Resource::PhysicsShape> mSource{ nullptr };
        const ObjectTransform mObjectTransform{ ESM::Position{ { 0, 0, 0 }, { 0, 0, 0 } }, 0.0f };

        DetourNavigatorRecastMeshBuilderTest()
        {
            mBounds.mMin = osg::Vec2f(-std::numeric_limits<float>::max() * std::numeric_limits<float>::epsilon(),
                -std::numeric_limits<float>::max() * std::numeric_limits<float>::epsilon());
            mBounds.mMax = osg::Vec2f(std::numeric_limits<float>::max() * std::numeric_limits<float>::epsilon(),
                std::numeric_limits<float>::max() * std::numeric_limits<float>::epsilon());
        }
    };

    TEST_F(DetourNavigatorRecastMeshBuilderTest, create_for_empty_should_return_empty)
    {
        RecastMeshBuilder builder(mBounds);
        const auto recastMesh = std::move(builder).create(mVersion);
        EXPECT_EQ(recastMesh->getMesh().getVertices(), std::vector<float>());
        EXPECT_EQ(recastMesh->getMesh().getIndices(), std::vector<int>());
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>());
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_bhv_triangle_mesh_shape)
    {
        JPH::MeshShapeSettings settings;
        settings.SetEmbedded();
        settings.mTriangleVertices.push_back(JPH::Float3(-1, -1, 0));
        settings.mTriangleVertices.push_back(JPH::Float3(-1, 1, 0));
        settings.mTriangleVertices.push_back(JPH::Float3(1, -1, 0));
        settings.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        const auto shape = settings.Create().Get();

        RecastMeshBuilder builder(mBounds);
        builder.addObject(static_cast<const JPH::Shape&>(*shape), osg::Matrixd::identity(), AreaType_ground, mSource,
            mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);
        EXPECT_EQ(recastMesh->getMesh().getVertices(),
            std::vector<float>({
                -1, -1, 0, // vertex 0
                -1, 1, 0, // vertex 1
                1, -1, 0, // vertex 2
            }))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(), std::vector<int>({ 2, 1, 0 }));
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>({ AreaType_ground }));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_transformed_bhv_triangle_mesh_shape)
    {
        JPH::MeshShapeSettings settings;
        settings.SetEmbedded();
        settings.mTriangleVertices.push_back(JPH::Float3(-1, -1, 0));
        settings.mTriangleVertices.push_back(JPH::Float3(-1, 1, 0));
        settings.mTriangleVertices.push_back(JPH::Float3(1, -1, 0));
        settings.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        const auto shape = settings.Create().Get();

        osg::Matrixd transform
            = osg::Matrixd::scale(osg::Vec3f(1, 2, 3)) * osg::Matrixd::translate(osg::Vec3f(1, 2, 3));

        RecastMeshBuilder builder(mBounds);
        builder.addObject(
            static_cast<const JPH::Shape&>(*shape), transform, AreaType_ground, mSource, mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);
        EXPECT_EQ(recastMesh->getMesh().getVertices(),
            std::vector<float>({
                0, 0, 3, // vertex 0
                0, 4, 3, // vertex 1
                2, 0, 3, // vertex 2
            }))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(), std::vector<int>({ 2, 1, 0 }));
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>({ AreaType_ground }));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_heightfield_terrain_shape)
    {
        // NOTE: this is the smallest Jolt heightfield we can create
        const std::array<float, 16> heightfieldData{ { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

        JPH::Vec3 terrainOffset(-1.5f, 0.0f, -1.5f);
        JPH::Vec3 terrainScale = JPH::Vec3::sReplicate(1.0f);
        JPH::HeightFieldShapeSettings shapeSettings(heightfieldData.data(), terrainOffset, terrainScale, 4);
        shapeSettings.mBlockSize = 2;

        auto createdRes = shapeSettings.Create();
        EXPECT_FALSE(createdRes.HasError());
        JPH::ShapeRefC shape = createdRes.Get();

        RecastMeshBuilder builder(mBounds);
        builder.addObject(static_cast<const JPH::Shape&>(*shape), osg::Matrixd::identity(), AreaType_ground, mSource,
            mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);

        EXPECT_EQ(recastMesh->getMesh().getVertices(),
            std::vector<float>({
                -1.5, 0.0, -1.5, // vertex 0
                -1.5, 0.0, -0.5, // vertex 1
                -1.5, 0.0, 0.5, // vertex 2
                -1.5, 0.0, 1.5, // vertex 3
                -0.5, 0.0, -1.5, // vertex 4
                -0.5, 0.0, -0.5, // vertex 5
                -0.5, 0.0, 0.5, // vertex 6
                -0.5, 0.0, 1.5, // vertex 7
                0.5, 0.0, -1.5, // vertex 8
                0.5, 0.0, -0.5, // vertex 9
                0.5, 0.0, 0.5, // vertex 10
                0.5, 0.0, 1.5, // vertex 11
                1.5, 0.0, -1.5, // vertex 12
                1.5, 0.0, -0.5, // vertex 13
                1.5, 0.0, 0.5, // vertex 14
                1.5, 0.0, 1.5, // vertex 15
            }))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(),
            std::vector<int>({ 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 4, 5, 9, 4, 9, 8, 5, 6, 10, 5, 10,
                9, 6, 7, 11, 6, 11, 10, 8, 9, 13, 8, 13, 12, 9, 10, 14, 9, 14, 13, 10, 11, 15, 10, 15, 14 }));
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>(18, AreaType_ground));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_box_shape_should_produce_12_triangles)
    {
        JPH::BoxShape shape(JPH::Vec3(1, 1, 2));
        shape.SetEmbedded();

        RecastMeshBuilder builder(mBounds);
        builder.addObject(static_cast<const JPH::Shape&>(shape), osg::Matrixd::identity(), AreaType_ground, mSource,
            mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);

        EXPECT_EQ(recastMesh->getMesh().getVertices(),
            std::vector<float>({
                -1, -1, -2, // vertex 0
                -1, -1, 2, // vertex 1
                -1, 1, -2, // vertex 2
                -1, 1, 2, // vertex 3
                1, -1, -2, // vertex 4
                1, -1, 2, // vertex 5
                1, 1, -2, // vertex 6
                1, 1, 2, // vertex 7
            }))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(),
            std::vector<int>({
                0, 4, 2, // triangle 0
                1, 0, 2, // triangle 1
                1, 5, 0, // triangle 2
                3, 1, 2, // triangle 3
                4, 5, 7, // triangle 4
                4, 6, 2, // triangle 5
                5, 1, 3, // triangle 6
                5, 4, 0, // triangle 7
                6, 4, 7, // triangle 8
                6, 7, 2, // triangle 9
                7, 3, 2, // triangle 10
                7, 5, 3, // triangle 11
            }))
            << recastMesh->getMesh().getIndices();
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>(12, AreaType_ground));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_compound_shape)
    {
        JPH::BoxShape box(JPH::Vec3(1, 1, 2));
        box.SetEmbedded();

        JPH::MeshShapeSettings settingsTri1;
        settingsTri1.SetEmbedded();
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, -1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, 1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(1, -1, 0));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        JPH::MeshShapeSettings settingsTri2;
        settingsTri2.SetEmbedded();
        settingsTri2.mTriangleVertices.push_back(JPH::Float3(1, 1, 0));
        settingsTri2.mTriangleVertices.push_back(JPH::Float3(-1, 1, 0));
        settingsTri2.mTriangleVertices.push_back(JPH::Float3(1, -1, 0));
        settingsTri2.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        const auto triangle1 = settingsTri1.Create().Get();
        const auto triangle2 = settingsTri2.Create().Get();

        JPH::StaticCompoundShapeSettings compoundSettings;
        compoundSettings.SetEmbedded();
        compoundSettings.AddShape(JPH::Vec3(0, 0, 0), JPH::Quat::sIdentity(), triangle1);
        compoundSettings.AddShape(JPH::Vec3(0, 0, 0), JPH::Quat::sIdentity(), &box);
        compoundSettings.AddShape(JPH::Vec3(0, 0, 0), JPH::Quat::sIdentity(), triangle2);
        const auto shape = compoundSettings.Create().Get();

        RecastMeshBuilder builder(mBounds);
        builder.addObject(static_cast<const JPH::Shape&>(*shape), osg::Matrixd::identity(), AreaType_ground, mSource,
            mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);

        EXPECT_EQ(recastMesh->getMesh().getVertices(),
            std::vector<float>({
                -1, -1, -2, // vertex 0
                -1, -1, 0, // vertex 1
                -1, -1, 2, // vertex 2
                -1, 1, -2, // vertex 3
                -1, 1, 0, // vertex 4
                -1, 1, 2, // vertex 5
                1, -1, -2, // vertex 6
                1, -1, 0, // vertex 7
                1, -1, 2, // vertex 8
                1, 1, -2, // vertex 9
                1, 1, 0, // vertex 10
                1, 1, 2, // vertex 11
            }))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(),
            std::vector<int>({
                0, 6, 3, // triangle 0
                2, 0, 3, // triangle 1
                2, 8, 0, // triangle 2
                5, 2, 3, // triangle 3
                6, 8, 11, // triangle 4
                6, 9, 3, // triangle 5
                7, 4, 1, // triangle 6
                7, 4, 10, // triangle 7
                8, 2, 5, // triangle 8
                8, 6, 0, // triangle 9
                9, 6, 11, // triangle 10
                9, 11, 3, // triangle 11
                11, 5, 3, // triangle 12
                11, 8, 5, // triangle 13
            }))
            << recastMesh->getMesh().getIndices();
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>(14, AreaType_ground));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_transformed_compound_shape)
    {
        JPH::MeshShapeSettings settingsTri1;
        settingsTri1.SetEmbedded();
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, -1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, 1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(1, -1, 0));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        const auto triangle1 = settingsTri1.Create().Get();

        JPH::StaticCompoundShapeSettings compoundSettings;
        compoundSettings.SetEmbedded();
        compoundSettings.AddShape(JPH::Vec3(0, 0, 0), JPH::Quat::sIdentity(), triangle1);
        const auto shape = compoundSettings.Create().Get();

        osg::Matrixd transform
            = osg::Matrixd::scale(osg::Vec3f(1, 2, 3)) * osg::Matrixd::translate(osg::Vec3f(1, 2, 3));
        RecastMeshBuilder builder(mBounds);
        builder.addObject(
            static_cast<const JPH::Shape&>(*shape), transform, AreaType_ground, mSource, mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);
        EXPECT_EQ(recastMesh->getMesh().getVertices(),
            std::vector<float>({
                0, 0, 3, // vertex 0
                0, 4, 3, // vertex 1
                2, 0, 3, // vertex 2
            }))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(), std::vector<int>({ 2, 1, 0 }));
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>({ AreaType_ground }));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_transformed_compound_shape_with_transformed_bhv_triangle_shape)
    {
        JPH::MeshShapeSettings settingsTri1;
        settingsTri1.SetEmbedded();
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, -1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, 1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(1, -1, 0));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        const auto triangle1 = settingsTri1.Create().Get();

        JPH::StaticCompoundShapeSettings compoundSettings;
        compoundSettings.SetEmbedded();
        compoundSettings.AddShape(
            JPH::Vec3(1, 2, 3), JPH::Quat::sIdentity(), new JPH::ScaledShape(triangle1, JPH::Vec3(1, 2, 3)));
        const auto shape = compoundSettings.Create().Get();

        osg::Matrixd transform
            = osg::Matrixd::scale(osg::Vec3f(1, 2, 3)) * osg::Matrixd::translate(osg::Vec3f(1, 2, 3));

        RecastMeshBuilder builder(mBounds);
        builder.addObject(
            static_cast<const JPH::Shape&>(*shape), transform, AreaType_ground, mSource, mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);
        EXPECT_EQ(recastMesh->getMesh().getVertices(),
            std::vector<float>({
                1, 2, 12, // vertex 0
                1, 10, 12, // vertex 1
                3, 2, 12, // vertex 2
            }))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(), std::vector<int>({ 2, 1, 0 }));
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>({ AreaType_ground }));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, without_bounds_add_bhv_triangle_shape_should_not_filter_by_bounds)
    {
        JPH::MeshShapeSettings settingsTri1;
        settingsTri1.SetEmbedded();
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, -1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, 1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(1, -1, 0));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-3, -3, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-3, -2, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-2, -3, 0));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(3, 4, 5));

        const auto shape = settingsTri1.Create().Get();

        RecastMeshBuilder builder(mBounds);
        builder.addObject(static_cast<const JPH::Shape&>(*shape), osg::Matrixd::identity(), AreaType_ground, mSource,
            mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);

        // NOTE: jolt mesh shape triangle walk causes this inprecision
        EXPECT_EQ(recastMesh->getMesh().getVertices(),
            std::vector<float>({
                -3, -3, 0, // vertex 0
                -3, -1.999999523162841796875, 0, // vertex 1
                -1.999999523162841796875, -3, 0, // vertex 2
                -0.99999904632568359375, -0.99999904632568359375, 0, // vertex 3
                -0.99999904632568359375, 1, 0, // vertex 4
                1, -0.99999904632568359375, 0, // vertex 5
            }))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(), std::vector<int>({ 2, 1, 0, 5, 4, 3 }));
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>(2, AreaType_ground));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, with_bounds_add_bhv_triangle_shape_should_filter_by_bounds)
    {
        mBounds.mMin = osg::Vec2f(-3, -3);
        mBounds.mMax = osg::Vec2f(-2, -2);

        JPH::MeshShapeSettings settingsTri1;
        settingsTri1.SetEmbedded();
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, -1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, 1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(1, -1, 0));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-3, -3, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-3, -2, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-2, -3, 0));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(3, 4, 5));

        const auto shape = settingsTri1.Create().Get();

        RecastMeshBuilder builder(mBounds);
        builder.addObject(static_cast<const JPH::Shape&>(*shape), osg::Matrixd::identity(), AreaType_ground, mSource,
            mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);

        // NOTE: jolt mesh shape triangle walk causes this inprecision
        EXPECT_EQ(recastMesh->getMesh().getVertices(),
            std::vector<float>({
                -3, -3, 0, // vertex 0
                -3, -1.999999523162841796875, 0, // vertex 1
                -1.999999523162841796875, -3, 0, // vertex 2
            }))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(), std::vector<int>({ 2, 1, 0 }));
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>({ AreaType_ground }));
    }

    TEST_F(
        DetourNavigatorRecastMeshBuilderTest, with_bounds_add_rotated_by_x_bhv_triangle_shape_should_filter_by_bounds)
    {
        mBounds.mMin = osg::Vec2f(-5, -5);
        mBounds.mMax = osg::Vec2f(5, -2);

        JPH::MeshShapeSettings settingsTri1;
        settingsTri1.SetEmbedded();
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(0, -1, -1));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(0, -1, 1));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(0, 1, -1));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        settingsTri1.mTriangleVertices.push_back(JPH::Float3(0, -3, -3));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(0, -3, -2));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(0, -2, -3));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(3, 4, 5));

        const auto shape = settingsTri1.Create().Get();
        osg::Matrixd transform = osg::Matrixd::rotate(osg::Quat(static_cast<float>(-osg::PI_4), osg::Vec3f(1, 0, 0)));

        RecastMeshBuilder builder(mBounds);
        builder.addObject(
            static_cast<const JPH::Shape&>(*shape), transform, AreaType_ground, mSource, mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);
        EXPECT_THAT(recastMesh->getMesh().getVertices(),
            Pointwise(FloatNear(1e-5f),
                std::vector<float>({
                    0, -4.24264049530029296875f, 4.44089209850062616169452667236328125e-16f, // vertex 0
                    0, -3.535533905029296875f, -0.707106769084930419921875f, // vertex 1
                    0, -3.535533905029296875f, 0.707106769084930419921875f, // vertex 2
                })))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(), std::vector<int>({ 1, 2, 0 }));
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>({ AreaType_ground }));
    }

    TEST_F(
        DetourNavigatorRecastMeshBuilderTest, with_bounds_add_rotated_by_y_bhv_triangle_shape_should_filter_by_bounds)
    {
        mBounds.mMin = osg::Vec2f(-5, -5);
        mBounds.mMax = osg::Vec2f(-3, 5);

        JPH::MeshShapeSettings settingsTri1;
        settingsTri1.SetEmbedded();
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, 0, -1));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, 0, 1));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(1, 0, -1));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-3, 0, -3));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-3, 0, -2));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-2, 0, -3));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(3, 4, 5));

        const auto shape = settingsTri1.Create().Get();
        osg::Matrixd transform = osg::Matrixd::rotate(osg::Quat(static_cast<float>(osg::PI_4), osg::Vec3f(0, 1, 0)));

        RecastMeshBuilder builder(mBounds);
        builder.addObject(
            static_cast<const JPH::Shape&>(*shape), transform, AreaType_ground, mSource, mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);
        EXPECT_THAT(recastMesh->getMesh().getVertices(),
            Pointwise(FloatNear(1e-5f),
                std::vector<float>({
                    -4.24264049530029296875f, 0, 4.44089209850062616169452667236328125e-16f, // vertex 0
                    -3.535533905029296875f, 0, -0.707106769084930419921875f, // vertex 1
                    -3.535533905029296875f, 0, 0.707106769084930419921875f, // vertex 2
                })))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(), std::vector<int>({ 1, 2, 0 }));
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>({ AreaType_ground }));
    }

    TEST_F(
        DetourNavigatorRecastMeshBuilderTest, with_bounds_add_rotated_by_z_bhv_triangle_shape_should_filter_by_bounds)
    {
        mBounds.mMin = osg::Vec2f(-5, -5);
        mBounds.mMax = osg::Vec2f(-1, -1);

        JPH::MeshShapeSettings settingsTri1;
        settingsTri1.SetEmbedded();
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, -1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, 1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(1, -1, 0));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-3, -3, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-3, -2, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-2, -3, 0));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(3, 4, 5));

        const auto shape = settingsTri1.Create().Get();
        osg::Matrixd transform = osg::Matrixd::rotate(osg::Quat(static_cast<float>(osg::PI_4), osg::Vec3f(0, 0, 1)));

        RecastMeshBuilder builder(mBounds);
        builder.addObject(
            static_cast<const JPH::Shape&>(*shape), transform, AreaType_ground, mSource, mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);
        EXPECT_THAT(recastMesh->getMesh().getVertices(),
            Pointwise(FloatNear(1e-5f),
                std::vector<float>({
                    -1.41421353816986083984375f, -1.1102230246251565404236316680908203125e-16f, 0, // vertex 0
                    1.1102230246251565404236316680908203125e-16f, -1.41421353816986083984375f, 0, // vertex 1
                    1.41421353816986083984375f, 1.1102230246251565404236316680908203125e-16f, 0, // vertex 2
                })))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(), std::vector<int>({ 2, 0, 1 }));
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>({ AreaType_ground }));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, flags_values_should_be_corresponding_to_added_objects)
    {
        JPH::MeshShapeSettings settingsTri1;
        settingsTri1.SetEmbedded();
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, -1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, 1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(1, -1, 0));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        JPH::MeshShapeSettings settingsTri2;
        settingsTri2.mTriangleVertices.push_back(JPH::Float3(-3, -3, 0));
        settingsTri2.mTriangleVertices.push_back(JPH::Float3(-3, -2, 0));
        settingsTri2.mTriangleVertices.push_back(JPH::Float3(-2, -3, 0));
        settingsTri2.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        const auto shape1 = settingsTri1.Create().Get();
        const auto shape2 = settingsTri2.Create().Get();

        RecastMeshBuilder builder(mBounds);
        builder.addObject(static_cast<const JPH::Shape&>(*shape1), osg::Matrixd::identity(), AreaType_ground, mSource,
            mObjectTransform);
        builder.addObject(static_cast<const JPH::Shape&>(*shape2), osg::Matrixd::identity(), AreaType_null, mSource,
            mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);
        EXPECT_EQ(recastMesh->getMesh().getVertices(),
            std::vector<float>({
                -3, -3, 0, // vertex 0
                -3, -2, 0, // vertex 1
                -2, -3, 0, // vertex 2
                -1, -1, 0, // vertex 3
                -1, 1, 0, // vertex 4
                1, -1, 0, // vertex 5
            }))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(), std::vector<int>({ 2, 1, 0, 5, 4, 3 }));
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>({ AreaType_null, AreaType_ground }));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_water_then_get_water_should_return_it)
    {
        RecastMeshBuilder builder(mBounds);
        builder.addWater(osg::Vec2i(1, 2), Water{ 1000, 300.0f });
        const auto recastMesh = std::move(builder).create(mVersion);
        EXPECT_EQ(
            recastMesh->getWater(), std::vector<CellWater>({ CellWater{ osg::Vec2i(1, 2), Water{ 1000, 300.0f } } }));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_bhv_triangle_mesh_shape_with_duplicated_vertices)
    {
        JPH::MeshShapeSettings settingsTri1;
        settingsTri1.SetEmbedded();
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, -1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, 1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(1, -1, 0));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(0, 1, 2));

        settingsTri1.mTriangleVertices.push_back(JPH::Float3(1, 1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(-1, 1, 0));
        settingsTri1.mTriangleVertices.push_back(JPH::Float3(1, -1, 0));
        settingsTri1.mIndexedTriangles.push_back(JPH::IndexedTriangle(3, 4, 5));

        const auto shape = settingsTri1.Create().Get();

        RecastMeshBuilder builder(mBounds);
        builder.addObject(static_cast<const JPH::Shape&>(*shape), osg::Matrixd::identity(), AreaType_ground, mSource,
            mObjectTransform);
        const auto recastMesh = std::move(builder).create(mVersion);
        EXPECT_EQ(recastMesh->getMesh().getVertices(),
            std::vector<float>({
                -1, -1, 0, // vertex 0
                -1, 1, 0, // vertex 1
                1, -1, 0, // vertex 2
                1, 1, 0, // vertex 3
            }))
            << recastMesh->getMesh().getVertices();
        EXPECT_EQ(recastMesh->getMesh().getIndices(), std::vector<int>({ 2, 1, 0, 2, 1, 3 }));
        EXPECT_EQ(recastMesh->getMesh().getAreaTypes(), std::vector<AreaType>({ AreaType_ground, AreaType_ground }));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_flat_heightfield_should_add_intersection)
    {
        const osg::Vec2i cellPosition(0, 0);
        const int cellSize = 1000;
        const float height = 10;
        mBounds.mMin = osg::Vec2f(100, 100);
        RecastMeshBuilder builder(mBounds);
        builder.addHeightfield(cellPosition, cellSize, height);
        const auto recastMesh = std::move(builder).create(mVersion);
        EXPECT_EQ(recastMesh->getFlatHeightfields(),
            std::vector<FlatHeightfield>({
                FlatHeightfield{ cellPosition, cellSize, height },
            }));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_heightfield_inside_tile)
    {
        constexpr std::size_t size = 3;
        constexpr std::array<float, size * size> heights{ {
            0, 1, 2, // row 0
            3, 4, 5, // row 1
            6, 7, 8, // row 2
        } };
        const osg::Vec2i cellPosition(0, 0);
        const int cellSize = 1000;
        const float minHeight = 0;
        const float maxHeight = 8;
        RecastMeshBuilder builder(mBounds);
        builder.addHeightfield(cellPosition, cellSize, heights.data(), size, minHeight, maxHeight);
        const auto recastMesh = std::move(builder).create(mVersion);
        Heightfield expected;
        expected.mCellPosition = cellPosition;
        expected.mCellSize = cellSize;
        expected.mLength = size;
        expected.mMinHeight = minHeight;
        expected.mMaxHeight = maxHeight;
        expected.mHeights.assign(heights.begin(), heights.end());
        expected.mOriginalSize = 3;
        expected.mMinX = 0;
        expected.mMinY = 0;
        EXPECT_EQ(recastMesh->getHeightfields(), std::vector<Heightfield>({ expected }));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_heightfield_to_shifted_cell_inside_tile)
    {
        constexpr std::size_t size = 3;
        constexpr std::array<float, size * size> heights{ {
            0, 1, 2, // row 0
            3, 4, 5, // row 1
            6, 7, 8, // row 2
        } };
        const osg::Vec2i cellPosition(1, 2);
        const int cellSize = 1000;
        const float minHeight = 0;
        const float maxHeight = 8;
        RecastMeshBuilder builder(maxCellTileBounds(cellPosition, cellSize));
        builder.addHeightfield(cellPosition, cellSize, heights.data(), size, minHeight, maxHeight);
        const auto recastMesh = std::move(builder).create(mVersion);
        Heightfield expected;
        expected.mCellPosition = cellPosition;
        expected.mCellSize = cellSize;
        expected.mLength = size;
        expected.mMinHeight = minHeight;
        expected.mMaxHeight = maxHeight;
        expected.mHeights.assign(heights.begin(), heights.end());
        expected.mOriginalSize = 3;
        expected.mMinX = 0;
        expected.mMinY = 0;
        EXPECT_EQ(recastMesh->getHeightfields(), std::vector<Heightfield>({ expected }));
    }

    TEST_F(DetourNavigatorRecastMeshBuilderTest, add_heightfield_should_add_intersection)
    {
        constexpr std::size_t size = 3;
        constexpr std::array<float, 3 * 3> heights{ {
            0, 1, 2, // row 0
            3, 4, 5, // row 1
            6, 7, 8, // row 2
        } };
        const osg::Vec2i cellPosition(0, 0);
        const int cellSize = 1000;
        const float minHeight = 0;
        const float maxHeight = 8;
        mBounds.mMin = osg::Vec2f(750, 750);
        RecastMeshBuilder builder(mBounds);
        builder.addHeightfield(cellPosition, cellSize, heights.data(), size, minHeight, maxHeight);
        const auto recastMesh = std::move(builder).create(mVersion);
        Heightfield expected;
        expected.mCellPosition = cellPosition;
        expected.mCellSize = cellSize;
        expected.mLength = 2;
        expected.mMinHeight = 0;
        expected.mMaxHeight = 8;
        expected.mHeights = {
            4, 5, // row 0
            7, 8, // row 1
        };
        expected.mOriginalSize = 3;
        expected.mMinX = 1;
        expected.mMinY = 1;
        EXPECT_EQ(recastMesh->getHeightfields(), std::vector<Heightfield>({ expected }));
    }
}
