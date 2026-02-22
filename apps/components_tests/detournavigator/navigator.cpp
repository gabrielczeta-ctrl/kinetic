#include "operators.hpp"
#include "settings.hpp"

#include <components/detournavigator/navigatorimpl.hpp>
#include <components/detournavigator/navigatorutils.hpp>
#include <components/detournavigator/navmeshdb.hpp>
#include <components/esm3/loadland.hpp>
#include <components/loadinglistener/loadinglistener.hpp>
#include <components/misc/rng.hpp>
#include <components/physicshelpers/heightfield.hpp>
#include <components/resource/physicsshape.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>

#include <osg/io_utils>
#include <osg/ref_ptr>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <condition_variable>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

MATCHER_P3(Vec3fEq, x, y, z, "")
{
    return std::abs(arg.x() - x) < 1e-3 && std::abs(arg.y() - y) < 1e-3 && std::abs(arg.z() - z) < 1e-3;
}

MATCHER_P4(Vec3fEq, x, y, z, precision, "")
{
    return std::abs(arg.x() - x) < precision && std::abs(arg.y() - y) < precision && std::abs(arg.z() - z) < precision;
}

namespace
{
    using namespace testing;
    using namespace DetourNavigator;
    using namespace DetourNavigator::Tests;

    constexpr int heightfieldTileSize = ESM::Land::REAL_SIZE / (ESM::Land::LAND_SIZE - 1);

    struct DetourNavigatorNavigatorTest : Test
    {
        Settings mSettings = makeSettings();
        std::unique_ptr<Navigator> mNavigator = std::make_unique<NavigatorImpl>(
            mSettings, std::make_unique<NavMeshDb>(":memory:", std::numeric_limits<std::uint64_t>::max()));
        const osg::Vec3f mPlayerPosition{ 256, 256, 0 };
        const ESM::RefId mWorldspace = ESM::RefId::stringRefId("sys::default");
        const AgentBounds mAgentBounds{ CollisionShapeType::Aabb, { 29, 29, 66 } };
        osg::Vec3f mStart{ 52, 460, 1 };
        osg::Vec3f mEnd{ 460, 52, 1 };
        std::deque<osg::Vec3f> mPath;
        std::back_insert_iterator<std::deque<osg::Vec3f>> mOut{ mPath };
        AreaCosts mAreaCosts;
        Loading::Listener mListener;
        const osg::Vec2i mCellPosition{ 0, 0 };
        const float mEndTolerance = 0;
        const osg::Matrixd mTransform = osg::Matrixd::translate(osg::Vec3f(256, 256, 0));
        const ObjectTransform mObjectTransform{ ESM::Position{ { 256, 256, 0 }, { 0, 0, 0 } }, 0.0f };
    };

    constexpr std::array<float, 5 * 5> defaultHeightfieldData{ {
        0, 0, 0, 0, 0, // row 0
        0, -25, -25, -25, -25, // row 1
        0, -25, -100, -100, -100, // row 2
        0, -25, -100, -100, -100, // row 3
        0, -25, -100, -100, -100, // row 4
    } };

    template <std::size_t size>
    std::unique_ptr<JPH::RotatedTranslatedShape> makeSquareHeightfieldTerrainShape(
        const std::array<float, size>& values)
    {
        const int width = static_cast<int>(std::sqrt(size));
        const float min = *std::min_element(values.begin(), values.end());
        const float max = *std::max_element(values.begin(), values.end());
        const float greater = std::max(std::abs(min), std::abs(max));

        JPH::Vec3 terrainOffset(0.0f, 0, 0.0f);
        JPH::Vec3 terrainScale(128.0f, 1.0f, -128.0f); // NOTE: jolt heightfield is Y up, its rotated below

        JPH::HeightFieldShapeSettings settings(values.data(), terrainOffset, terrainScale, width);
        settings.mMinHeightValue = -greater;
        settings.mMaxHeightValue = greater;
        settings.mBlockSize = 2;

        // Create a quaternion representing a rotation of 90 degrees around the X-axis
        JPH::Quat rotation = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), JPH::DegreesToRadians(90.0f));

        // Must flip on Z axis (scale) then rotate
        JPH::Ref<JPH::Shape> createdRes = settings.Create().Get();
        createdRes->SetEmbedded();
        return std::make_unique<JPH::RotatedTranslatedShape>(JPH::Vec3(-256.0f, -256.0f, 0.0f), rotation, createdRes);
    }

    template <std::size_t size>
    HeightfieldSurface makeSquareHeightfieldSurface(const std::array<float, size>& values)
    {
        const auto [min, max] = std::minmax_element(values.begin(), values.end());
        const float greater = std::max(std::abs(*min), std::abs(*max));
        HeightfieldSurface surface;
        surface.mHeights = values.data();
        surface.mMinHeight = -greater;
        surface.mMaxHeight = greater;
        surface.mSize = static_cast<int>(std::sqrt(size));
        return surface;
    }

    template <class T>
    osg::ref_ptr<const Resource::PhysicsShapeInstance> makePhysicsShapeInstance(std::unique_ptr<T>&& shape)
    {
        osg::ref_ptr<Resource::PhysicsShape> physicsShape(new Resource::PhysicsShape);
        physicsShape->mCollisionShape = std::move(shape).release();
        return new Resource::PhysicsShapeInstance(physicsShape);
    }

    template <class T>
    class CollisionShapeInstance
    {
    public:
        CollisionShapeInstance(std::unique_ptr<T>&& shape)
            : mInstance(makePhysicsShapeInstance(std::move(shape)))
        {
        }

        T& shape() const { return static_cast<T&>(*mInstance->mCollisionShape); }
        const osg::ref_ptr<const Resource::PhysicsShapeInstance>& instance() const { return mInstance; }

    private:
        osg::ref_ptr<const Resource::PhysicsShapeInstance> mInstance;
    };

    inline osg::Vec3f getHeightfieldShift(
        const osg::Vec2i& cellPosition, int cellSize, float minHeight, float maxHeight)
    {
        return PhysicsSystemHelpers::getHeightfieldShift(
            cellPosition.x(), cellPosition.x(), cellSize, minHeight, maxHeight);
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_for_empty_should_return_empty)
    {
        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::NavMeshNotFound);
        EXPECT_EQ(mPath, std::deque<osg::Vec3f>());
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_for_existing_agent_with_no_navmesh_should_throw_exception)
    {
        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::StartPolygonNotFound);
    }

    TEST_F(DetourNavigatorNavigatorTest, add_agent_should_count_each_agent)
    {
        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->removeAgent(mAgentBounds);
        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::StartPolygonNotFound);
    }

    TEST_F(DetourNavigatorNavigatorTest, update_then_find_path_should_return_path)
    {
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        auto updateGuard = mNavigator->makeUpdateGuard();
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, updateGuard.get());
        mNavigator->update(mPlayerPosition, updateGuard.get());
        updateGuard.reset();
        mNavigator->wait(WaitConditionType::requiredTilesPresent, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_to_the_start_position_should_contain_single_point)
    {
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        auto updateGuard = mNavigator->makeUpdateGuard();
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, updateGuard.get());
        mNavigator->update(mPlayerPosition, updateGuard.get());
        updateGuard.reset();
        mNavigator->wait(WaitConditionType::requiredTilesPresent, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mStart, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath, ElementsAre(Vec3fEq(56.66666412353515625, 460, 1.99998295307159423828125))) << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, add_object_should_change_navmesh)
    {
        mSettings.mWaitUntilMinDistanceToPlayer = 0;
        mNavigator.reset(new NavigatorImpl(
            mSettings, std::make_unique<NavMeshDb>(":memory:", std::numeric_limits<std::uint64_t>::max())));

        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        std::unique_ptr<JPH::Shape> ptr;
        JPH::StaticCompoundShapeSettings settings;
        settings.AddShape(JPH::Vec3(0, 0, 0), JPH::Quat::sIdentity(), new JPH::BoxShape(JPH::Vec3(20, 20, 100)));
        ptr.reset(settings.Create().Get().GetPtr());

        CollisionShapeInstance compound(std::move(ptr));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66666412353515625, 460, 1.99998295307159423828125),
                Vec3fEq(460, 56.66666412353515625, 1.99998295307159423828125)))
            << mPath;

        {
            auto updateGuard = mNavigator->makeUpdateGuard();
            mNavigator->addObject(ObjectId(&compound.shape()), ObjectShapes(compound.instance(), mObjectTransform),
                mTransform, updateGuard.get());
            mNavigator->update(mPlayerPosition, updateGuard.get());
        }
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mPath.clear();
        mOut = std::back_inserter(mPath);
        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(181.33331298828125, 215.33331298828125, -20.6666717529296875),
                Vec3fEq(215.33331298828125, 181.33331298828125, -20.6666717529296875),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, update_changed_object_should_change_navmesh)
    {
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        std::unique_ptr<JPH::Shape> ptr;
        JPH::MutableCompoundShapeSettings settings;
        settings.AddShape(JPH::Vec3(0, 0, 0), JPH::Quat::sIdentity(), new JPH::BoxShape(JPH::Vec3(20, 20, 100)));
        auto shapeRef = settings.Create().Get();
        ptr.reset(shapeRef.GetPtr());

        CollisionShapeInstance compound(std::move(ptr));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->addObject(
            ObjectId(&compound.shape()), ObjectShapes(compound.instance(), mObjectTransform), mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(181.33331298828125, 215.33331298828125, -20.6666717529296875),
                Vec3fEq(215.33331298828125, 181.33331298828125, -20.6666717529296875),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;

        static_cast<JPH::MutableCompoundShape*>(shapeRef.GetPtr())
            ->ModifyShape(0, JPH::Vec3(1000, 0, 0), JPH::Quat::sIdentity());

        mNavigator->updateObject(
            ObjectId(&compound.shape()), ObjectShapes(compound.instance(), mObjectTransform), mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mPath.clear();
        mOut = std::back_inserter(mPath);
        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, for_overlapping_heightfields_objects_should_use_higher)
    {
        CollisionShapeInstance heightfield1(makeSquareHeightfieldTerrainShape(defaultHeightfieldData));
        heightfield1.shape().SetEmbedded();

        const std::array<float, 5 * 5> heightfieldData2{ {
            -25, -25, -25, -25, -25, // row 0
            -25, -25, -25, -25, -25, // row 1
            -25, -25, -25, -25, -25, // row 2
            -25, -25, -25, -25, -25, // row 3
            -25, -25, -25, -25, -25, // row 4
        } };
        CollisionShapeInstance heightfield2(makeSquareHeightfieldTerrainShape(heightfieldData2));
        heightfield2.shape().SetEmbedded();

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addObject(ObjectId(&heightfield1.shape()), ObjectShapes(heightfield1.instance(), mObjectTransform),
            mTransform, nullptr);
        mNavigator->addObject(ObjectId(&heightfield2.shape()), ObjectShapes(heightfield2.instance(), mObjectTransform),
            mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, only_one_heightfield_per_cell_is_allowed)
    {
        const HeightfieldSurface surface1 = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize1 = heightfieldTileSize * static_cast<int>(surface1.mSize - 1);

        const std::array<float, 5 * 5> heightfieldData2{ {
            -25, -25, -25, -25, -25, // row 0
            -25, -25, -25, -25, -25, // row 1
            -25, -25, -25, -25, -25, // row 2
            -25, -25, -25, -25, -25, // row 3
            -25, -25, -25, -25, -25, // row 4
        } };
        const HeightfieldSurface surface2 = makeSquareHeightfieldSurface(heightfieldData2);
        const int cellSize2 = heightfieldTileSize * static_cast<int>(surface2.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize1, surface1, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const Version version = mNavigator->getNavMesh(mAgentBounds)->lockConst()->getVersion();

        mNavigator->addHeightfield(mCellPosition, cellSize2, surface2, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(mNavigator->getNavMesh(mAgentBounds)->lockConst()->getVersion(), version);
    }

    TEST_F(DetourNavigatorNavigatorTest, path_should_be_around_avoid_shape)
    {
        osg::ref_ptr<Resource::PhysicsShape> physicsShape(new Resource::PhysicsShape);

        std::unique_ptr<JPH::RotatedTranslatedShape> shapePtr
            = makeSquareHeightfieldTerrainShape(defaultHeightfieldData);
        physicsShape->mCollisionShape = shapePtr.release();

        std::array<float, 5 * 5> heightfieldDataAvoid{ {
            -25, -25, -25, -25, -25, // row 0
            -25, -25, -25, -25, -25, // row 1
            -25, -25, -25, -25, -25, // row 2
            -25, -25, -25, -25, -25, // row 3
            -25, -25, -25, -25, -25, // row 4
        } };
        std::unique_ptr<JPH::RotatedTranslatedShape> shapeAvoidPtr
            = makeSquareHeightfieldTerrainShape(heightfieldDataAvoid);
        physicsShape->mAvoidCollisionShape = shapeAvoidPtr.release();

        osg::ref_ptr<const Resource::PhysicsShapeInstance> instance(new Resource::PhysicsShapeInstance(physicsShape));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addObject(ObjectId(instance->mCollisionShape.GetPtr()), ObjectShapes(instance, mObjectTransform),
            mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(158.6666412353515625, 192.666656494140625, -20.6666717529296875),
                Vec3fEq(192.666656494140625, 158.6666412353515625, -20.6666717529296875),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, path_should_be_over_water_ground_lower_than_water_with_only_swim_flag)
    {
        std::array<float, 5 * 5> heightfieldData{ {
            -50, -50, -50, -50, 0, // row 0
            -50, -100, -150, -100, -50, // row 1
            -50, -150, -200, -150, -100, // row 2
            -50, -100, -150, -100, -100, // row 3
            0, -50, -100, -100, -100, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addWater(mCellPosition, cellSize, 300, nullptr);
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mStart.x() = 256;
        mStart.z() = 300;
        mEnd.x() = 256;
        mEnd.z() = 300;

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_swim, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(256, 460, 185.3333282470703125), Vec3fEq(256, 56.66664886474609375, 185.3333282470703125)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, path_should_be_over_water_when_ground_cross_water_with_swim_and_walk_flags)
    {
        std::array<float, 7 * 7> heightfieldData{ {
            0, 0, 0, 0, 0, 0, 0, // row 0
            0, -100, -100, -100, -100, -100, 0, // row 1
            0, -100, -150, -150, -150, -100, 0, // row 2
            0, -100, -150, -200, -150, -100, 0, // row 3
            0, -100, -150, -150, -150, -100, 0, // row 4
            0, -100, -100, -100, -100, -100, 0, // row 5
            0, 0, 0, 0, 0, 0, 0, // row 6
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addWater(mCellPosition, cellSize, -25, nullptr);
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mStart.x() = 256;
        mEnd.x() = 256;

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_swim | Flag_walk, mAreaCosts, mEndTolerance,
                      {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(256, 460, -129.4098663330078125), Vec3fEq(256, 56.66664886474609375, -30.0000133514404296875)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest,
        path_should_be_over_water_when_ground_cross_water_with_max_int_cells_size_and_swim_and_walk_flags)
    {
        std::array<float, 7 * 7> heightfieldData{ {
            0, 0, 0, 0, 0, 0, 0, // row 0
            0, -100, -100, -100, -100, -100, 0, // row 1
            0, -100, -150, -150, -150, -100, 0, // row 2
            0, -100, -150, -200, -150, -100, 0, // row 3
            0, -100, -150, -150, -150, -100, 0, // row 4
            0, -100, -100, -100, -100, -100, 0, // row 5
            0, 0, 0, 0, 0, 0, 0, // row 6
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->addWater(mCellPosition, std::numeric_limits<int>::max(), -25, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mStart.x() = 256;
        mEnd.x() = 256;

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_swim | Flag_walk, mAreaCosts, mEndTolerance,
                      {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre(
                Vec3fEq(256, 460, -129.4098663330078125), Vec3fEq(256, 56.66664886474609375, -30.0000133514404296875)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, path_should_be_over_ground_when_ground_cross_water_with_only_walk_flag)
    {
        std::array<float, 7 * 7> heightfieldData{ {
            0, 0, 0, 0, 0, 0, 0, // row 0
            0, -100, -100, -100, -100, -100, 0, // row 1
            0, -100, -150, -150, -150, -100, 0, // row 2
            0, -100, -150, -200, -150, -100, 0, // row 3
            0, -100, -150, -150, -150, -100, 0, // row 4
            0, -100, -100, -100, -100, -100, 0, // row 5
            0, 0, 0, 0, 0, 0, 0, // row 6
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addWater(mCellPosition, cellSize, -25, nullptr);
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mStart.x() = 256;
        mEnd.x() = 256;

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(256, 460, -129.4098663330078125), Vec3fEq(256, 56.66664886474609375, -30.0000133514404296875)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, update_object_remove_and_update_then_find_path_should_return_path)
    {
        CollisionShapeInstance heightfield(makeSquareHeightfieldTerrainShape(defaultHeightfieldData));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addObject(ObjectId(&heightfield.shape()), ObjectShapes(heightfield.instance(), mObjectTransform),
            mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mNavigator->removeObject(ObjectId(&heightfield.shape()), nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mNavigator->addObject(ObjectId(&heightfield.shape()), ObjectShapes(heightfield.instance(), mObjectTransform),
            mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, update_heightfield_remove_and_update_then_find_path_should_return_path)
    {
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mNavigator->removeHeightfield(mCellPosition, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, update_then_find_random_point_around_circle_should_return_position)
    {
        const std::array<float, 6 * 6> heightfieldData{ {
            0, 0, 0, 0, 0, 0, // row 0
            0, -25, -25, -25, -25, -25, // row 1
            0, -25, -1000, -1000, -100, -100, // row 2
            0, -25, -1000, -1000, -100, -100, // row 3
            0, -25, -100, -100, -100, -100, // row 4
            0, -25, -100, -100, -100, -100, // row 5
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        Misc::Rng::init(42);

        const auto result = findRandomPointAroundCircle(
            *mNavigator, mAgentBounds, mStart, 100.0, Flag_walk, []() { return Misc::Rng::rollClosedProbability(); });

        ASSERT_TRUE(result.has_value());

        EXPECT_THAT(*result, Vec3fEq(70.35845947265625, 335.592041015625, -2.6667339801788330078125, 1)) << *result;

        const auto distance = (*result - mStart).length();

        EXPECT_NEAR(distance, 125.80865478515625, 1) << distance;
    }

    TEST_F(DetourNavigatorNavigatorTest, multiple_threads_should_lock_tiles)
    {
        mSettings.mAsyncNavMeshUpdaterThreads = 2;
        mNavigator.reset(new NavigatorImpl(
            mSettings, std::make_unique<NavMeshDb>(":memory:", std::numeric_limits<std::uint64_t>::max())));

        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);
        const osg::Vec3f shift = getHeightfieldShift(mCellPosition, cellSize, surface.mMinHeight, surface.mMaxHeight);

        std::vector<CollisionShapeInstance<JPH::BoxShape>> boxes;
        std::generate_n(
            std::back_inserter(boxes), 100, [] { return std::make_unique<JPH::BoxShape>(JPH::Vec3(20, 20, 100)); });

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));

        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);

        for (std::size_t i = 0; i < boxes.size(); ++i)
        {
            boxes[i].shape().SetEmbedded();
            const double offset = static_cast<double>(i) * 10.0;
            const osg::Matrixd transform
                = osg::Matrixd::translate(osg::Vec3d(shift.x() + offset, shift.y() + offset, offset));
            mNavigator->addObject(
                ObjectId(&boxes[i].shape()), ObjectShapes(boxes[i].instance(), mObjectTransform), transform, nullptr);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(1));

        for (std::size_t i = 0; i < boxes.size(); ++i)
        {
            const double offset = static_cast<double>(i) * 10.0 + 1.0;
            const osg::Matrixd transform
                = osg::Matrixd::translate(osg::Vec3d(shift.x() + offset, shift.y() + offset, offset));
            mNavigator->updateObject(
                ObjectId(&boxes[i].shape()), ObjectShapes(boxes[i].instance(), mObjectTransform), transform, nullptr);
        }

        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 1.99999392032623291015625),
                Vec3fEq(181.33331298828125, 215.33331298828125, -20.6666717529296875),
                Vec3fEq(215.33331298828125, 181.33331298828125, -20.6666717529296875),
                Vec3fEq(460, 56.66664886474609375, 1.99999392032623291015625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, update_changed_multiple_times_object_should_delay_navmesh_change)
    {
        std::vector<CollisionShapeInstance<JPH::BoxShape>> shapes;
        std::generate_n(
            std::back_inserter(shapes), 100, [] { return std::make_unique<JPH::BoxShape>(JPH::Vec3(64, 64, 64)); });

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));

        for (std::size_t i = 0; i < shapes.size(); ++i)
        {
            shapes[i].shape().SetEmbedded();
            const float offset = static_cast<float>(i) * 32.0f;
            const osg::Matrixd transform = osg::Matrixd::translate(osg::Vec3f(offset, offset, offset));
            mNavigator->addObject(
                ObjectId(&shapes[i].shape()), ObjectShapes(shapes[i].instance(), mObjectTransform), transform, nullptr);
        }
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const auto start = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < shapes.size(); ++i)
        {
            const float offset = static_cast<float>(i) * 32.0f + 1.0f;
            const osg::Matrixd transform = osg::Matrixd::translate(osg::Vec3f(offset, offset, offset));
            mNavigator->updateObject(
                ObjectId(&shapes[i].shape()), ObjectShapes(shapes[i].instance(), mObjectTransform), transform, nullptr);
        }
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        for (std::size_t i = 0; i < shapes.size(); ++i)
        {
            const float offset = static_cast<float>(i) * 32.0f + 2.0f;
            const osg::Matrixd transform = osg::Matrixd::translate(osg::Vec3f(offset, offset, offset));
            mNavigator->updateObject(
                ObjectId(&shapes[i].shape()), ObjectShapes(shapes[i].instance(), mObjectTransform), transform, nullptr);
        }
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const auto duration = std::chrono::steady_clock::now() - start;

        EXPECT_GT(duration, mSettings.mMinUpdateInterval)
            << std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(duration).count() << " ms";
    }

    TEST_F(DetourNavigatorNavigatorTest, update_then_raycast_should_return_position)
    {
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const osg::Vec3f start(57, 460, 1);
        const osg::Vec3f end(460, 57, 1);
        const auto result = raycast(*mNavigator, mAgentBounds, start, end, Flag_walk);

        ASSERT_THAT(result, Optional(Vec3fEq(end.x(), end.y(), 1.95257937908172607421875)))
            << (result ? *result : osg::Vec3f());
    }

    TEST_F(DetourNavigatorNavigatorTest,
        update_for_oscillating_object_that_does_not_change_navmesh_should_not_trigger_navmesh_update)
    {
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        CollisionShapeInstance oscillatingBox(std::make_unique<JPH::BoxShape>(JPH::Vec3(20, 20, 20)));
        const osg::Vec3f oscillatingBoxShapePosition(288, 288, 400);
        CollisionShapeInstance borderBox(std::make_unique<JPH::BoxShape>(JPH::Vec3(50, 50, 50)));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->addObject(ObjectId(&oscillatingBox.shape()),
            ObjectShapes(oscillatingBox.instance(), mObjectTransform),
            osg::Matrixd::translate(oscillatingBoxShapePosition), nullptr);
        // add this box to make navmesh bound box independent from oscillatingBoxShape rotations
        mNavigator->addObject(ObjectId(&borderBox.shape()), ObjectShapes(borderBox.instance(), mObjectTransform),
            osg::Matrixd::translate(oscillatingBoxShapePosition + osg::Vec3f(0, 0, 200)), nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const Version expectedVersion{ 1, 4 };

        const auto navMeshes = mNavigator->getNavMeshes();
        ASSERT_EQ(navMeshes.size(), 1);
        ASSERT_EQ(navMeshes.begin()->second->lockConst()->getVersion(), expectedVersion);

        for (int n = 0; n < 10; ++n)
        {
            osg::Matrixd transform = osg::Matrixd::rotate(osg::Quat(n * 2 * osg::PI / 10, osg::Vec3f(0, 0, 1)));
            transform.setTrans(oscillatingBoxShapePosition);

            mNavigator->updateObject(ObjectId(&oscillatingBox.shape()),
                ObjectShapes(oscillatingBox.instance(), mObjectTransform), transform, nullptr);
            mNavigator->update(mPlayerPosition, nullptr);
            mNavigator->wait(WaitConditionType::allJobsDone, &mListener);
        }

        ASSERT_EQ(navMeshes.size(), 1);
        ASSERT_EQ(navMeshes.begin()->second->lockConst()->getVersion(), expectedVersion);
    }

    TEST_F(DetourNavigatorNavigatorTest, should_provide_path_over_flat_heightfield)
    {
        const HeightfieldPlane plane{ 100 };
        const int cellSize = heightfieldTileSize * 4;

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, plane, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::requiredTilesPresent, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, 102), Vec3fEq(460, 56.66664886474609375, 102)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, for_not_reachable_destination_find_path_should_provide_partial_path)
    {
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        std::unique_ptr<JPH::Shape> ptr;
        JPH::StaticCompoundShapeSettings settings;
        settings.AddShape(
            JPH::Vec3(204, -204, 0), JPH::Quat::sIdentity(), new JPH::BoxShape(JPH::Vec3(200, 200, 1000)));
        ptr.reset(settings.Create().Get().GetPtr());

        CollisionShapeInstance compound(std::move(ptr));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->addObject(
            ObjectId(&compound.shape()), ObjectShapes(compound.instance(), mObjectTransform), mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::PartialPath);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, -2.5371119976043701171875),
                Vec3fEq(222, 290, -71.33342742919921875)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, end_tolerance_should_extent_available_destinations)
    {
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        std::unique_ptr<JPH::Shape> ptr;
        JPH::StaticCompoundShapeSettings settings;
        settings.AddShape(
            JPH::Vec3(204, -204, 0), JPH::Quat::sIdentity(), new JPH::BoxShape(JPH::Vec3(100, 100, 1000)));
        ptr.reset(settings.Create().Get().GetPtr());

        CollisionShapeInstance compound(std::move(ptr));

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->addObject(
            ObjectId(&compound.shape()), ObjectShapes(compound.instance(), mObjectTransform), mTransform, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const float endTolerance = 1000.0f;

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, mStart, mEnd, Flag_walk, mAreaCosts, endTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 460, -2.5371119976043701171875),
                Vec3fEq(305.999969482421875, 56.66664886474609375, -2.6667406558990478515625)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, only_one_water_per_cell_is_allowed)
    {
        const int cellSize1 = 100;
        const float level1 = 1;
        const int cellSize2 = 200;
        const float level2 = 2;

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addWater(mCellPosition, cellSize1, level1, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const Version version = mNavigator->getNavMesh(mAgentBounds)->lockConst()->getVersion();

        mNavigator->addWater(mCellPosition, cellSize2, level2, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        EXPECT_EQ(mNavigator->getNavMesh(mAgentBounds)->lockConst()->getVersion(), version);
    }

    std::pair<TilePosition, TilePosition> getMinMax(const RecastMeshTiles& tiles)
    {
        const auto lessByX = [](const auto& l, const auto& r) { return l.first.x() < r.first.x(); };
        const auto lessByY = [](const auto& l, const auto& r) { return l.first.y() < r.first.y(); };

        const auto [minX, maxX] = std::ranges::minmax_element(tiles, lessByX);
        const auto [minY, maxY] = std::ranges::minmax_element(tiles, lessByY);

        return { TilePosition(minX->first.x(), minY->first.y()), TilePosition(maxX->first.x(), maxY->first.y()) };
    }

    TEST_F(DetourNavigatorNavigatorTest, update_for_very_big_object_should_be_limited)
    {
        const float size = static_cast<float>((1 << 22) - 1);
        CollisionShapeInstance bigBox(std::make_unique<JPH::BoxShape>(JPH::Vec3(size, size, 1)));
        bigBox.shape().SetEmbedded();
        const ObjectTransform objectTransform{
            .mPosition = ESM::Position{ .pos = { 0, 0, 0 }, .rot{ 0, 0, 0 } },
            .mScale = 1.0f,
        };
        const std::optional<CellGridBounds> cellGridBounds = std::nullopt;
        const osg::Vec3f playerPosition(32, 1024, 0);

        mNavigator->updateBounds(mWorldspace, cellGridBounds, playerPosition, nullptr);
        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addObject(ObjectId(&bigBox.shape()), ObjectShapes(bigBox.instance(), objectTransform),
            osg::Matrixd::identity(), nullptr);

        bool updated = false;
        std::condition_variable updateFinished;
        std::mutex mutex;

        std::thread thread([&] {
            auto guard = mNavigator->makeUpdateGuard();
            mNavigator->update(playerPosition, guard.get());
            std::lock_guard lock(mutex);
            updated = true;
            updateFinished.notify_all();
        });

        {
            std::unique_lock lock(mutex);
            updateFinished.wait_for(lock, std::chrono::seconds(10), [&] { return updated; });
            ASSERT_TRUE(updated);
        }

        thread.join();

        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const auto recastMeshTiles = mNavigator->getRecastMeshTiles();
        ASSERT_EQ(recastMeshTiles.size(), 1033);
        EXPECT_EQ(getMinMax(recastMeshTiles), std::pair(TilePosition(-18, -17), TilePosition(18, 19)));

        const auto navMesh = mNavigator->getNavMesh(mAgentBounds);
        ASSERT_NE(navMesh, nullptr);

        std::size_t usedNavMeshTiles = 0;
        navMesh->lockConst()->forEachUsedTile([&](const auto&...) { ++usedNavMeshTiles; });
        EXPECT_EQ(usedNavMeshTiles, 1024);
    }

    TEST_F(DetourNavigatorNavigatorTest, update_should_be_limited_by_cell_grid_bounds)
    {
        const float size = static_cast<float>((1 << 22) - 1);
        CollisionShapeInstance bigBox(std::make_unique<JPH::BoxShape>(JPH::Vec3(size, size, 1)));
        const ObjectTransform objectTransform{
            .mPosition = ESM::Position{ .pos = { 0, 0, 0 }, .rot{ 0, 0, 0 } },
            .mScale = 1.0f,
        };
        const CellGridBounds cellGridBounds{
            .mCenter = osg::Vec2i(0, 0),
            .mHalfSize = 1,
        };
        const osg::Vec3f playerPosition(32, 1024, 0);

        mNavigator->updateBounds(mWorldspace, cellGridBounds, playerPosition, nullptr);
        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addObject(ObjectId(&bigBox.shape()), ObjectShapes(bigBox.instance(), objectTransform),
            osg::Matrixd::identity(), nullptr);

        bool updated = false;
        std::condition_variable updateFinished;
        std::mutex mutex;

        std::thread thread([&] {
            auto guard = mNavigator->makeUpdateGuard();
            mNavigator->update(playerPosition, guard.get());
            std::lock_guard lock(mutex);
            updated = true;
            updateFinished.notify_all();
        });

        {
            std::unique_lock lock(mutex);
            updateFinished.wait_for(lock, std::chrono::seconds(10), [&] { return updated; });
            ASSERT_TRUE(updated);
        }

        thread.join();

        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const auto recastMeshTiles = mNavigator->getRecastMeshTiles();
        ASSERT_EQ(recastMeshTiles.size(), 854);
        EXPECT_EQ(getMinMax(recastMeshTiles), std::pair(TilePosition(-12, -12), TilePosition(18, 19)));

        const auto navMesh = mNavigator->getNavMesh(mAgentBounds);
        ASSERT_NE(navMesh, nullptr);

        std::size_t usedNavMeshTiles = 0;
        navMesh->lockConst()->forEachUsedTile([&](const auto&...) { ++usedNavMeshTiles; });
        EXPECT_EQ(usedNavMeshTiles, 854);
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_should_return_path_around_steep_mountains)
    {
        const std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, 0, 0, 0, 0, // row 1
            0, 0, 1000, 0, 0, // row 2
            0, 0, 1000, 0, 0, // row 3
            0, 0, 0, 0, 0, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const osg::Vec3f start(56, 56, 12);
        const osg::Vec3f end(464, 464, 12);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, start, end, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 56.66664886474609375, 11.33333301544189453125),
                Vec3fEq(396.666656494140625, 79.33331298828125, 11.33333301544189453125),
                Vec3fEq(430.666656494140625, 113.33331298828125, 11.33333301544189453125),
                Vec3fEq(463.999969482421875, 463.999969482421875, 11.33333301544189453125)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_should_return_path_around_steep_cliffs)
    {
        const std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, 0, 0, 0, 0, // row 1
            0, 0, -1000, 0, 0, // row 2
            0, 0, -1000, 0, 0, // row 3
            0, 0, 0, 0, 0, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const osg::Vec3f start(56, 56, 12);
        const osg::Vec3f end(464, 464, 12);

        EXPECT_EQ(findPath(*mNavigator, mAgentBounds, start, end, Flag_walk, mAreaCosts, mEndTolerance, {}, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 56.66664886474609375, 8.66659259796142578125),
                Vec3fEq(385.33331298828125, 79.33331298828125, 8.66659259796142578125),
                Vec3fEq(430.666656494140625, 124.66664886474609375, 8.66659259796142578125),
                Vec3fEq(463.999969482421875, 463.999969482421875, 8.66659259796142578125)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_should_return_path_with_checkpoints)
    {
        const std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, 0, 0, 0, 0, // row 1
            0, 0, 1000, 0, 0, // row 2
            0, 0, 1000, 0, 0, // row 3
            0, 0, 0, 0, 0, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const std::vector<osg::Vec3f> checkpoints = {
            osg::Vec3f(400, 70, 12),
        };

        const osg::Vec3f start(56, 56, 12);
        const osg::Vec3f end(464, 464, 12);

        EXPECT_EQ(
            findPath(*mNavigator, mAgentBounds, start, end, Flag_walk, mAreaCosts, mEndTolerance, checkpoints, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 56.66664886474609375, 11.33333301544189453125),
                Vec3fEq(400, 70, 11.33333301544189453125),
                Vec3fEq(430.666656494140625, 113.33331298828125, 11.33333301544189453125),
                Vec3fEq(463.999969482421875, 463.999969482421875, 11.33333301544189453125)))
            << mPath;
    }

    TEST_F(DetourNavigatorNavigatorTest, find_path_should_skip_unreachable_checkpoints)
    {
        const std::array<float, 5 * 5> heightfieldData{ {
            0, 0, 0, 0, 0, // row 0
            0, 0, 0, 0, 0, // row 1
            0, 0, 1000, 0, 0, // row 2
            0, 0, 1000, 0, 0, // row 3
            0, 0, 0, 0, 0, // row 4
        } };
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(heightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, nullptr);
        mNavigator->update(mPlayerPosition, nullptr);
        mNavigator->wait(WaitConditionType::allJobsDone, &mListener);

        const std::vector<osg::Vec3f> checkpoints = {
            osg::Vec3f(400, 70, 10000),
            osg::Vec3f(256, 256, 1000),
            osg::Vec3f(-1000, -1000, 0),
        };

        const osg::Vec3f start(56, 56, 12);
        const osg::Vec3f end(464, 464, 12);

        EXPECT_EQ(
            findPath(*mNavigator, mAgentBounds, start, end, Flag_walk, mAreaCosts, mEndTolerance, checkpoints, mOut),
            Status::Success);

        EXPECT_THAT(mPath,
            ElementsAre( //
                Vec3fEq(56.66664886474609375, 56.66664886474609375, 11.33333301544189453125),
                Vec3fEq(396.666656494140625, 79.33331298828125, 11.33333301544189453125),
                Vec3fEq(430.666656494140625, 113.33331298828125, 11.33333301544189453125),
                Vec3fEq(463.999969482421875, 463.999969482421875, 11.33333301544189453125)))
            << mPath;
    }

    struct DetourNavigatorNavigatorNotSupportedAgentBoundsTest : TestWithParam<AgentBounds>
    {
    };

    TEST_P(DetourNavigatorNavigatorNotSupportedAgentBoundsTest, on_add_agent)
    {
        const Settings settings = makeSettings();
        NavigatorImpl navigator(settings, nullptr);
        EXPECT_FALSE(navigator.addAgent(GetParam()));
    }

    const std::array notSupportedAgentBounds = {
        AgentBounds{ .mShapeType = CollisionShapeType::Aabb, .mHalfExtents = osg::Vec3f(0, 0, 0) },
        AgentBounds{ .mShapeType = CollisionShapeType::RotatingBox, .mHalfExtents = osg::Vec3f(0, 0, 0) },
        AgentBounds{ .mShapeType = CollisionShapeType::Cylinder, .mHalfExtents = osg::Vec3f(0, 0, 0) },
        AgentBounds{ .mShapeType = CollisionShapeType::Aabb, .mHalfExtents = osg::Vec3f(0, 0, 11.34f) },
        AgentBounds{ .mShapeType = CollisionShapeType::RotatingBox, .mHalfExtents = osg::Vec3f(0, 11.34f, 11.34f) },
        AgentBounds{ .mShapeType = CollisionShapeType::Cylinder, .mHalfExtents = osg::Vec3f(0, 0, 11.34f) },
        AgentBounds{ .mShapeType = CollisionShapeType::Aabb, .mHalfExtents = osg::Vec3f(1, 1, 0) },
        AgentBounds{ .mShapeType = CollisionShapeType::RotatingBox, .mHalfExtents = osg::Vec3f(1, 1, 0) },
        AgentBounds{ .mShapeType = CollisionShapeType::Cylinder, .mHalfExtents = osg::Vec3f(1, 1, 0) },
        AgentBounds{ .mShapeType = CollisionShapeType::Aabb, .mHalfExtents = osg::Vec3f(1, 1, 11.33f) },
        AgentBounds{ .mShapeType = CollisionShapeType::RotatingBox, .mHalfExtents = osg::Vec3f(1, 1, 11.33f) },
        AgentBounds{ .mShapeType = CollisionShapeType::Cylinder, .mHalfExtents = osg::Vec3f(1, 1, 11.33f) },
        AgentBounds{ .mShapeType = CollisionShapeType::Aabb, .mHalfExtents = osg::Vec3f(2043.54f, 2043.54f, 11.34f) },
        AgentBounds{ .mShapeType = CollisionShapeType::RotatingBox, .mHalfExtents = osg::Vec3f(2890, 1, 11.34f) },
        AgentBounds{ .mShapeType = CollisionShapeType::Cylinder, .mHalfExtents = osg::Vec3f(2890, 2890, 11.34f) },
    };

    INSTANTIATE_TEST_SUITE_P(NotSupportedAgentBounds, DetourNavigatorNavigatorNotSupportedAgentBoundsTest,
        ValuesIn(notSupportedAgentBounds));

    TEST_F(DetourNavigatorNavigatorTest, find_nearest_nav_mesh_position_should_return_nav_mesh_position)
    {
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        auto updateGuard = mNavigator->makeUpdateGuard();
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, updateGuard.get());
        mNavigator->update(mPlayerPosition, updateGuard.get());
        updateGuard.reset();
        mNavigator->wait(WaitConditionType::requiredTilesPresent, &mListener);

        const osg::Vec3f position(250, 250, 0);
        const osg::Vec3f searchAreaHalfExtents(1000, 1000, 1000);
        EXPECT_THAT(findNearestNavMeshPosition(*mNavigator, mAgentBounds, position, searchAreaHalfExtents, Flag_walk),
            Optional(Vec3fEq(250, 250, -62.5186)));
    }

    TEST_F(DetourNavigatorNavigatorTest, find_nearest_nav_mesh_position_should_return_nullopt_when_too_far)
    {
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        auto updateGuard = mNavigator->makeUpdateGuard();
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, updateGuard.get());
        mNavigator->update(mPlayerPosition, updateGuard.get());
        updateGuard.reset();
        mNavigator->wait(WaitConditionType::requiredTilesPresent, &mListener);

        const osg::Vec3f position(250, 250, 250);
        const osg::Vec3f searchAreaHalfExtents(100, 100, 100);
        EXPECT_EQ(findNearestNavMeshPosition(*mNavigator, mAgentBounds, position, searchAreaHalfExtents, Flag_walk),
            std::nullopt);
    }

    TEST_F(DetourNavigatorNavigatorTest, find_nearest_nav_mesh_position_should_return_nullopt_when_flags_do_not_match)
    {
        const HeightfieldSurface surface = makeSquareHeightfieldSurface(defaultHeightfieldData);
        const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);

        ASSERT_TRUE(mNavigator->addAgent(mAgentBounds));
        auto updateGuard = mNavigator->makeUpdateGuard();
        mNavigator->addHeightfield(mCellPosition, cellSize, surface, updateGuard.get());
        mNavigator->update(mPlayerPosition, updateGuard.get());
        updateGuard.reset();
        mNavigator->wait(WaitConditionType::requiredTilesPresent, &mListener);

        const osg::Vec3f position(250, 250, 0);
        const osg::Vec3f searchAreaHalfExtents(1000, 1000, 1000);
        EXPECT_EQ(findNearestNavMeshPosition(*mNavigator, mAgentBounds, position, searchAreaHalfExtents, Flag_swim),
            std::nullopt);
    }

    struct DetourNavigatorUpdateTest : TestWithParam<std::function<void(Navigator&)>>
    {
    };

    std::vector<TilePosition> getUsedTiles(const NavMeshCacheItem& navMesh)
    {
        std::vector<TilePosition> result;
        navMesh.forEachUsedTile([&](const TilePosition& position, const auto&...) { result.push_back(position); });
        return result;
    }

    TEST_P(DetourNavigatorUpdateTest, update_should_change_covered_area_when_player_moves)
    {
        Loading::Listener listener;
        Settings settings = makeSettings();
        settings.mMaxTilesNumber = 5;
        NavigatorImpl navigator(settings, nullptr);
        const AgentBounds agentBounds{ CollisionShapeType::Aabb, { 29, 29, 66 } };
        ASSERT_TRUE(navigator.addAgent(agentBounds));

        GetParam()(navigator);

        {
            auto updateGuard = navigator.makeUpdateGuard();
            navigator.update(osg::Vec3f(3000, 3000, 0), updateGuard.get());
        }

        navigator.wait(WaitConditionType::allJobsDone, &listener);

        {
            const auto navMesh = navigator.getNavMesh(agentBounds);
            ASSERT_NE(navMesh, nullptr);

            const TilePosition expectedTiles[] = { { 3, 4 }, { 4, 3 }, { 4, 4 }, { 4, 5 }, { 5, 4 } };
            const auto usedTiles = getUsedTiles(*navMesh->lockConst());
            EXPECT_THAT(usedTiles, UnorderedElementsAreArray(expectedTiles)) << usedTiles;
        }

        {
            auto updateGuard = navigator.makeUpdateGuard();
            navigator.update(osg::Vec3f(4000, 3000, 0), updateGuard.get());
        }

        navigator.wait(WaitConditionType::allJobsDone, &listener);

        {
            const auto navMesh = navigator.getNavMesh(agentBounds);
            ASSERT_NE(navMesh, nullptr);

            const TilePosition expectedTiles[] = { { 4, 4 }, { 5, 3 }, { 5, 4 }, { 5, 5 }, { 6, 4 } };
            const auto usedTiles = getUsedTiles(*navMesh->lockConst());
            EXPECT_THAT(usedTiles, UnorderedElementsAreArray(expectedTiles)) << usedTiles;
        }
    }

    TEST_P(DetourNavigatorUpdateTest, update_should_change_covered_area_when_player_moves_without_waiting_for_all)
    {
        Loading::Listener listener;
        Settings settings = makeSettings();
        settings.mMaxTilesNumber = 1;
        settings.mWaitUntilMinDistanceToPlayer = 1;
        NavigatorImpl navigator(settings, nullptr);
        const AgentBounds agentBounds{ CollisionShapeType::Aabb, { 29, 29, 66 } };
        ASSERT_TRUE(navigator.addAgent(agentBounds));

        GetParam()(navigator);

        {
            auto updateGuard = navigator.makeUpdateGuard();
            navigator.update(osg::Vec3f(3000, 3000, 0), updateGuard.get());
        }

        navigator.wait(WaitConditionType::requiredTilesPresent, &listener);

        {
            const auto navMesh = navigator.getNavMesh(agentBounds);
            ASSERT_NE(navMesh, nullptr);

            const TilePosition expectedTile(4, 4);
            const auto usedTiles = getUsedTiles(*navMesh->lockConst());
            EXPECT_THAT(usedTiles, Contains(expectedTile)) << usedTiles;
        }

        {
            auto updateGuard = navigator.makeUpdateGuard();
            navigator.update(osg::Vec3f(6000, 3000, 0), updateGuard.get());
        }

        navigator.wait(WaitConditionType::requiredTilesPresent, &listener);

        {
            const auto navMesh = navigator.getNavMesh(agentBounds);
            ASSERT_NE(navMesh, nullptr);

            const TilePosition expectedTile(8, 4);
            const auto usedTiles = getUsedTiles(*navMesh->lockConst());
            EXPECT_THAT(usedTiles, Contains(expectedTile)) << usedTiles;
        }
    }

    TEST_P(DetourNavigatorUpdateTest, update_should_change_covered_area_when_player_moves_with_db)
    {
        Loading::Listener listener;
        Settings settings = makeSettings();
        settings.mMaxTilesNumber = 1;
        settings.mWaitUntilMinDistanceToPlayer = 1;
        NavigatorImpl navigator(settings, std::make_unique<NavMeshDb>(":memory:", settings.mMaxDbFileSize));
        const AgentBounds agentBounds{ CollisionShapeType::Aabb, { 29, 29, 66 } };
        ASSERT_TRUE(navigator.addAgent(agentBounds));

        GetParam()(navigator);

        {
            auto updateGuard = navigator.makeUpdateGuard();
            navigator.update(osg::Vec3f(3000, 3000, 0), updateGuard.get());
        }

        navigator.wait(WaitConditionType::requiredTilesPresent, &listener);

        {
            const auto navMesh = navigator.getNavMesh(agentBounds);
            ASSERT_NE(navMesh, nullptr);

            const TilePosition expectedTile(4, 4);
            const auto usedTiles = getUsedTiles(*navMesh->lockConst());
            EXPECT_THAT(usedTiles, Contains(expectedTile)) << usedTiles;
        }

        {
            auto updateGuard = navigator.makeUpdateGuard();
            navigator.update(osg::Vec3f(6000, 3000, 0), updateGuard.get());
        }

        navigator.wait(WaitConditionType::requiredTilesPresent, &listener);

        {
            const auto navMesh = navigator.getNavMesh(agentBounds);
            ASSERT_NE(navMesh, nullptr);

            const TilePosition expectedTile(8, 4);
            const auto usedTiles = getUsedTiles(*navMesh->lockConst());
            EXPECT_THAT(usedTiles, Contains(expectedTile)) << usedTiles;
        }
    }

    struct AddHeightfieldSurface
    {
        static constexpr std::size_t sSize = 65;
        static constexpr float sHeights[sSize * sSize]{};

        void operator()(Navigator& navigator) const
        {
            const osg::Vec2i cellPosition(0, 0);
            const HeightfieldSurface surface{
                .mHeights = sHeights,
                .mSize = sSize,
                .mMinHeight = -1,
                .mMaxHeight = 1,
            };
            const int cellSize = heightfieldTileSize * static_cast<int>(surface.mSize - 1);
            navigator.addHeightfield(cellPosition, cellSize, surface, nullptr);
        }
    };

    struct AddHeightfieldPlane
    {
        void operator()(Navigator& navigator) const
        {
            const osg::Vec2i cellPosition(0, 0);
            const HeightfieldPlane plane{ .mHeight = 0 };
            const int cellSize = 8192;
            navigator.addHeightfield(cellPosition, cellSize, plane, nullptr);
        }
    };

    struct AddWater
    {
        void operator()(Navigator& navigator) const
        {
            const osg::Vec2i cellPosition(0, 0);
            const float level = 0;
            const int cellSize = 8192;
            navigator.addWater(cellPosition, cellSize, level, nullptr);
        }
    };

    struct AddObject
    {
        const float mSize = 8192;
        const ObjectTransform mTransform{
            .mPosition = ESM::Position{ .pos = { 0, 0, 0 }, .rot{ 0, 0, 0 } },
            .mScale = 1.0f,
        };

        void operator()(Navigator& navigator) const
        {
            CollisionShapeInstance<JPH::BoxShape> mBox{ std::make_unique<JPH::BoxShape>(JPH::Vec3(mSize, mSize, 1)) };
            mBox.shape().SetEmbedded();
            navigator.addObject(
                ObjectId(&mBox.shape()), ObjectShapes(mBox.instance(), mTransform), osg::Matrixd::identity(), nullptr);
        }
    };

    struct AddAll
    {
        AddHeightfieldSurface mAddHeightfieldSurface;
        AddHeightfieldPlane mAddHeightfieldPlane;
        AddWater mAddWater;
        AddObject mAddObject;

        void operator()(Navigator& navigator) const
        {
            mAddHeightfieldSurface(navigator);
            mAddHeightfieldPlane(navigator);
            mAddWater(navigator);
            mAddObject(navigator);
        }
    };

    const std::function<void(Navigator&)> addNavMeshData[] = {
        AddHeightfieldSurface{},
        AddHeightfieldPlane{},
        AddWater{},
        AddObject{},
        AddAll{},
    };

    INSTANTIATE_TEST_SUITE_P(DifferentNavMeshData, DetourNavigatorUpdateTest, ValuesIn(addNavMeshData));
}
