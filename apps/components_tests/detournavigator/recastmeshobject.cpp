
#include <components/detournavigator/recastmeshobject.hpp>
#include <components/misc/convert.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>

#include <gtest/gtest.h>

namespace
{
    using namespace testing;
    using namespace DetourNavigator;

    struct DetourNavigatorRecastMeshObjectTest : Test
    {
        JPH::BoxShape* mBoxShapeImpl;
        JPH::Ref<JPH::MutableCompoundShape> mCompoundShapeImpl;
        JPH::MutableCompoundShapeSettings mCompoundShapeSettings;

        CollisionShape* mBoxShape;
        CollisionShape* mCompoundShape;
        const ObjectTransform mObjectTransform{ ESM::Position{ { 1, 2, 3 }, { 1, 2, 3 } }, 0.5f };
        osg::Matrixd mTransform{ Misc::Convert::makeOSGTransform(mObjectTransform.mPosition) };

        DetourNavigatorRecastMeshObjectTest()
        {
            mBoxShapeImpl = new JPH::BoxShape(JPH::Vec3(1, 2, 3));
            mCompoundShapeSettings.AddShape(Misc::Convert::toJolt<JPH::Vec3>(mTransform.getTrans()),
                Misc::Convert::toJolt(mTransform.getRotate()), mBoxShapeImpl);

            auto createdRes = mCompoundShapeSettings.Create();
            EXPECT_FALSE(createdRes.HasError());
            JPH::Ref<JPH::Shape> shape = createdRes.Get();
            mCompoundShapeImpl = static_cast<JPH::MutableCompoundShape*>(shape.GetPtr());
            mCompoundShape = new CollisionShape(nullptr, *mCompoundShapeImpl, mObjectTransform);
            mBoxShape = new CollisionShape(nullptr, *mBoxShapeImpl, mObjectTransform);
        }
    };

    TEST_F(DetourNavigatorRecastMeshObjectTest, constructed_object_should_have_shape_and_transform)
    {
        const RecastMeshObject object(*mBoxShape, mTransform, AreaType_ground);
        EXPECT_EQ(std::addressof(object.getShape()), std::addressof(*const_cast<const JPH::BoxShape*>(mBoxShapeImpl)));
        EXPECT_EQ(object.getTransform(), mTransform);
    }

    TEST_F(DetourNavigatorRecastMeshObjectTest, update_with_same_transform_for_not_compound_shape_should_return_false)
    {
        RecastMeshObject object(*mBoxShape, mTransform, AreaType_ground);
        EXPECT_FALSE(object.update(mTransform, AreaType_ground));
    }

    TEST_F(DetourNavigatorRecastMeshObjectTest, update_with_different_transform_should_return_true)
    {
        RecastMeshObject object(*mBoxShape, mTransform, AreaType_ground);
        EXPECT_TRUE(object.update(osg::Matrixd::identity(), AreaType_ground));
    }

    TEST_F(DetourNavigatorRecastMeshObjectTest, update_with_different_flags_should_return_true)
    {
        RecastMeshObject object(*mBoxShape, mTransform, AreaType_ground);
        EXPECT_TRUE(object.update(mTransform, AreaType_null));
    }

    TEST_F(DetourNavigatorRecastMeshObjectTest,
        update_for_compound_shape_with_same_transform_and_not_changed_child_transform_should_return_false)
    {
        RecastMeshObject object(*mCompoundShape, mTransform, AreaType_ground);
        EXPECT_FALSE(object.update(mTransform, AreaType_ground));
    }

    TEST_F(DetourNavigatorRecastMeshObjectTest,
        update_for_compound_shape_with_same_transform_and_changed_child_transform_should_return_true)
    {
        osg::Matrixd idTransform = osg::Matrixd::identity();
        RecastMeshObject object(*mCompoundShape, mTransform, AreaType_ground);
        mCompoundShapeImpl->ModifyShape(0, Misc::Convert::toJolt<JPH::Vec3>(idTransform.getTrans()),
            Misc::Convert::toJolt(idTransform.getRotate()));
        EXPECT_TRUE(object.update(mTransform, AreaType_ground));
    }

    TEST_F(DetourNavigatorRecastMeshObjectTest, repeated_update_for_compound_shape_without_changes_should_return_false)
    {
        osg::Matrixd idTransform = osg::Matrixd::identity();
        RecastMeshObject object(*mCompoundShape, mTransform, AreaType_ground);
        mCompoundShapeImpl->ModifyShape(0, Misc::Convert::toJolt<JPH::Vec3>(idTransform.getTrans()),
            Misc::Convert::toJolt(idTransform.getRotate()));
        object.update(mTransform, AreaType_ground);
        EXPECT_FALSE(object.update(mTransform, AreaType_ground));
    }
}
