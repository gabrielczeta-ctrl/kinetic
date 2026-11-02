#include <components/ragdollprofile/skeletonadapter.hpp>
#include <components/sceneutil/skeleton.hpp>
#include <components/sceneutil/keyframe.hpp>
#include <components/nifosg/controller.hpp>

#include <osg/MatrixTransform>
#include <osg/ref_ptr>
#include <osgUtil/UpdateVisitor>

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace
{
    RagdollCore::RagdollProfile makeProfile()
    {
        RagdollCore::RagdollProfile profile;
        profile.mId = "adapter_fixture";
        profile.mBoneAliases = {
            { "pelvis", { "Bip01 Pelvis" } },
            { "spine", { "Bip01 Spine" } },
        };

        RagdollCore::BodyProfile pelvis;
        pelvis.mName = "pelvis";
        pelvis.mBone = "pelvis";
        pelvis.mShape.mType = RagdollCore::ShapeType::Sphere;
        pelvis.mShape.mRadius = 2.0f;
        pelvis.mMassFraction = 0.6f;

        RagdollCore::BodyProfile spine;
        spine.mName = "spine";
        spine.mBone = "spine";
        spine.mParentBody = "pelvis";
        spine.mShape.mType = RagdollCore::ShapeType::Capsule;
        spine.mShape.mRadius = 1.0f;
        spine.mShape.mHalfHeight = 2.0f;
        spine.mMassFraction = 0.4f;
        spine.mConstraint = RagdollCore::ConstraintProfile{};

        profile.mBodies = { pelvis, spine };
        profile.mLandmarks.mPelvis = "pelvis";
        profile.mLandmarks.mSpine = { "spine" };
        return profile;
    }

    osg::ref_ptr<osg::MatrixTransform> makeBone(const std::string& name, const osg::Vec3f& translation)
    {
        osg::ref_ptr<osg::MatrixTransform> result = new osg::MatrixTransform;
        result->setName(name);
        result->setMatrix(osg::Matrixf::translate(translation));
        return result;
    }

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4250)
#endif
    class SamplingController final : public NifOsg::KeyframeController
    {
    public:
        SamplingController() = default;
        SamplingController(const SamplingController& copy, const osg::CopyOp& op)
            : osg::Object(copy, op), NifOsg::KeyframeController(copy, op), mInvalid(copy.mInvalid) {}
        META_Object(RagdollTest, SamplingController)
        bool mInvalid = false;
        KfTransform getCurrentTransformation(osg::NodeVisitor* visitor) override
        {
            KfTransform result;
            result.mTranslation = osg::Vec3f(mInvalid ? std::numeric_limits<float>::quiet_NaN()
                : getInputValue(visitor) * 10.0f, 0, 0);
            return result;
        }
    };
#ifdef _MSC_VER
#pragma warning(pop)
#endif

    TEST(RagdollSkeletonAdapterTest, samplesSourceTimeWithoutMutatingOrReadingBackPhysicalPose)
    {
        auto skeleton = osg::ref_ptr<SceneUtil::Skeleton>(new SceneUtil::Skeleton);
        auto root = makeBone("Bip01", { 10, 0, 0 });
        auto pelvis = makeBone("Bip01 Pelvis", { 2, 0, 0 });
        auto spine = makeBone("Bip01 Spine", { 3, 0, 0 });
        skeleton->addChild(root);
        root->addChild(pelvis);
        pelvis->addChild(spine);
        auto binding = RagdollProfile::snapshotSkeleton(makeProfile(), *skeleton);
        ASSERT_TRUE(binding.isValid());
        auto controller = osg::ref_ptr<SamplingController>(new SamplingController);
        auto originalSource = std::make_shared<SceneUtil::FrameTimeSource>();
        controller->setSource(originalSource);
        *controller->mTime = 123;
        auto source = osg::ref_ptr<SceneUtil::KeyframeHolder>(new SceneUtil::KeyframeHolder);
        source->mKeyframeControllers.emplace("BIP01", controller);
        std::vector<RagdollCore::Transform> sampled;
        ASSERT_TRUE(binding.mPoseBinding->sampleAnimationPose(*source, 0.25f, sampled));
        ASSERT_EQ(sampled.size(), 2u);
        EXPECT_FLOAT_EQ(sampled[0].mTranslation.GetX(), 4.5f);
        EXPECT_FLOAT_EQ(sampled[1].mTranslation.GetX(), 7.5f);

        auto physical = sampled;
        physical[0].mTranslation.SetX(100);
        physical[1].mTranslation.SetX(110);
        ASSERT_TRUE(binding.mPoseBinding->applyPose(physical));
        const auto beforePelvis = pelvis->getMatrix();
        const auto beforeSpine = spine->getMatrix();
        auto* const callback = skeleton->getUpdateCallback();
        std::vector<RagdollCore::Transform> repeated;
        ASSERT_TRUE(binding.mPoseBinding->sampleAnimationPose(*source, 0.25f, repeated));
        ASSERT_EQ(repeated.size(), sampled.size());
        for (std::size_t i = 0; i < sampled.size(); ++i)
        {
            EXPECT_EQ(repeated[i].mTranslation, sampled[i].mTranslation);
            EXPECT_EQ(repeated[i].mRotation, sampled[i].mRotation);
        }
        EXPECT_EQ(pelvis->getMatrix(), beforePelvis);
        EXPECT_EQ(spine->getMatrix(), beforeSpine);
        EXPECT_EQ(skeleton->getUpdateCallback(), callback);
        EXPECT_EQ(pelvis->getNumParents(), 1u);
        EXPECT_EQ(spine->getNumParents(), 1u);
        EXPECT_EQ(controller->getSource(), originalSource);
        EXPECT_FLOAT_EQ(*controller->mTime, 123);

        source->mTextKeys.emplace(0, "walkforward: start");
        source->mTextKeys.emplace(1, "walkforward: stop");
        const auto cycle = binding.mPoseBinding->sampleAnimationCycle(*source, "walkforward", "Bip01",
            JPH::Vec3(1, 1, 0), 8);
        ASSERT_TRUE(cycle.isValid()) << cycle.mError;
        ASSERT_EQ(cycle.mFrames.size(), 8u);
        EXPECT_FLOAT_EQ(cycle.mMaximumSeamTranslation, 0);
        for (const auto& frame : cycle.mFrames)
        {
            EXPECT_FLOAT_EQ(frame[0].mTranslation.GetX(), 2);
            EXPECT_FLOAT_EQ(frame[1].mTranslation.GetX(), 5);
        }
        EXPECT_FALSE(binding.mPoseBinding->sampleAnimationCycle(*source, "walkforward", "missing root",
            JPH::Vec3(1, 1, 0), 8).isValid());
        source->mKeyframeControllers.emplace("Bip01 Spine", controller);
        const auto invalidSeam = binding.mPoseBinding->sampleAnimationCycle(*source, "walkforward", "Bip01",
            JPH::Vec3(1, 1, 0), 8);
        EXPECT_FALSE(invalidSeam.isValid());
        EXPECT_TRUE(invalidSeam.mFrames.empty());
        EXPECT_GE(invalidSeam.mMaximumSeamTranslation, 0.05f);
        source->mKeyframeControllers.erase("Bip01 Spine");

        EXPECT_FALSE(binding.mPoseBinding->sampleAnimationPose(*source,
            std::numeric_limits<float>::quiet_NaN(), repeated));
        controller->mInvalid = true;
        EXPECT_FALSE(binding.mPoseBinding->sampleAnimationPose(*source, 0.5f, repeated));
        EXPECT_EQ(repeated[0].mTranslation, sampled[0].mTranslation);
        source->mKeyframeControllers.emplace("bip01", controller);
        EXPECT_FALSE(binding.mPoseBinding->sampleAnimationPose(*source, 0.25f, repeated));
    }

    TEST(RagdollSkeletonAdapterTest, resolvesAliasesAndSnapshotsModelSpaceHierarchy)
    {
        osg::ref_ptr<SceneUtil::Skeleton> skeleton = new SceneUtil::Skeleton;
        skeleton->setName("fixture_skeleton");
        osg::ref_ptr<osg::MatrixTransform> root = makeBone("Bip01", { 10.0f, 0.0f, 0.0f });
        osg::ref_ptr<osg::MatrixTransform> pelvis = makeBone("BIP01 PELVIS", { 2.0f, 0.0f, 0.0f });
        osg::ref_ptr<osg::MatrixTransform> spine = makeBone("bip01 spine", { 3.0f, 0.0f, 0.0f });
        skeleton->addChild(root);
        root->addChild(pelvis);
        pelvis->addChild(spine);

        const auto result = RagdollProfile::snapshotSkeleton(makeProfile(), *skeleton);
        ASSERT_TRUE(result.isValid());
        ASSERT_EQ(result.mSkeleton->mBones.size(), 2u);
        EXPECT_EQ(result.mSkeleton->mId, "fixture_skeleton");
        EXPECT_EQ(result.mSkeleton->mBones[0].mName, "pelvis");
        EXPECT_EQ(result.mSkeleton->mBones[0].mParent, -1);
        EXPECT_FLOAT_EQ(result.mSkeleton->mBones[0].mModelTransform.mTranslation.GetX(), 12.0f);
        EXPECT_EQ(result.mSkeleton->mBones[1].mParent, 0);
        EXPECT_FLOAT_EQ(result.mSkeleton->mBones[1].mModelTransform.mTranslation.GetX(), 15.0f);
    }

    TEST(RagdollSkeletonAdapterTest, evaluatedPhysicalPoseDoesNotMirrorNormalizeLegs)
    {
        auto profile = makeProfile();
        auto foot = profile.mBodies.back();
        profile.mBodies.resize(1);
        foot.mName = foot.mBone = "left_foot";
        foot.mMassFraction = 0.2f;
        profile.mBodies.push_back(foot);
        foot.mName = foot.mBone = "right_foot";
        profile.mBodies.push_back(foot);
        profile.mLandmarks.mSpine.clear();
        auto skeleton = osg::ref_ptr<SceneUtil::Skeleton>(new SceneUtil::Skeleton);
        auto pelvis = makeBone("Bip01 Pelvis", {});
        skeleton->addChild(pelvis);
        pelvis->addChild(makeBone("left_foot", { -5, 0, -20 }));
        pelvis->addChild(makeBone("right_foot", { 10, 4, -18 }));
        auto result = RagdollProfile::snapshotSkeleton(profile, *skeleton);
        ASSERT_TRUE(result.isValid());

        const std::vector<RagdollCore::Transform> physical = {
            { JPH::Vec3(100, 0, 0), JPH::Quat::sIdentity() },
            { JPH::Vec3(90, 12, 5), JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.2f) },
            { JPH::Vec3(120, -8, 7), JPH::Quat::sRotation(JPH::Vec3::sAxisY(), -0.3f) },
        };
        ASSERT_TRUE(result.mPoseBinding->applyPose(physical));
        std::vector<RagdollCore::Transform> evaluated, targets, animation;
        ASSERT_TRUE(result.mPoseBinding->captureEvaluatedPose(evaluated));
        ASSERT_TRUE(result.mPoseBinding->capturePose(targets));
        ASSERT_TRUE(result.mPoseBinding->captureAnimationPose(animation));
        ASSERT_EQ(evaluated.size(), physical.size());
        for (std::size_t i = 0; i < physical.size(); ++i)
        {
            EXPECT_LT((evaluated[i].mTranslation - physical[i].mTranslation).Length(), 1e-5f);
            EXPECT_NEAR(std::abs(evaluated[i].mRotation.Dot(physical[i].mRotation)), 1.0f, 1e-5f);
        }
        // Target normalization remains enabled at the current default. The
        // actual nodes and their readback must retain their asymmetric pose.
        EXPECT_LT((targets[1].mTranslation - JPH::Vec3(-15, 2, 6)).Length(), 1e-5f);
        EXPECT_LT((targets[2].mTranslation - JPH::Vec3(15, 2, 6)).Length(), 1e-5f);
        EXPECT_LT((animation[1].mTranslation - JPH::Vec3(-7.5f, 2, -19)).Length(), 1e-5f);
        ASSERT_TRUE(result.mPoseBinding->captureEvaluatedPose(evaluated));
        EXPECT_LT((evaluated[1].mTranslation - physical[1].mTranslation).Length(), 1e-5f);
    }

    TEST(RagdollSkeletonAdapterTest, kinematicRootRouteKeepsAsymmetricCreationLegs)
    {
        // A body created in a staggered stance: the averaged snapshot would
        // draw both feet at the same fore-aft position while the rigid leg
        // meshes keep pointing at the real ones, which shows as detached
        // limbs. The held-root route has no use for the symmetric joint zero.
        auto profile = makeProfile();
        profile.mActiveController.mLocomotionRoute = RagdollCore::LocomotionRoute::KinematicRoot;
        auto foot = profile.mBodies.back();
        profile.mBodies.resize(1);
        foot.mName = foot.mBone = "left_foot";
        foot.mMassFraction = 0.2f;
        profile.mBodies.push_back(foot);
        foot.mName = foot.mBone = "right_foot";
        profile.mBodies.push_back(foot);
        profile.mLandmarks.mSpine.clear();
        auto skeleton = osg::ref_ptr<SceneUtil::Skeleton>(new SceneUtil::Skeleton);
        auto pelvis = makeBone("Bip01 Pelvis", {});
        skeleton->addChild(pelvis);
        pelvis->addChild(makeBone("left_foot", { -5, 14, -20 }));
        pelvis->addChild(makeBone("right_foot", { 10, -9, -18 }));
        auto result = RagdollProfile::snapshotSkeleton(profile, *skeleton);
        ASSERT_TRUE(result.isValid());
        ASSERT_EQ(result.mSkeleton->mBones.size(), 3u);
        EXPECT_LT((result.mSkeleton->mBones[1].mModelTransform.mTranslation - JPH::Vec3(-5, 14, -20)).Length(), 1e-5f);
        EXPECT_LT((result.mSkeleton->mBones[2].mModelTransform.mTranslation - JPH::Vec3(10, -9, -18)).Length(), 1e-5f);
        std::vector<RagdollCore::Transform> targets;
        ASSERT_TRUE(result.mPoseBinding->capturePose(targets));
        EXPECT_LT((targets[1].mTranslation - JPH::Vec3(-5, 14, -20)).Length(), 1e-5f);
        EXPECT_LT((targets[2].mTranslation - JPH::Vec3(10, -9, -18)).Length(), 1e-5f);
    }

    TEST(RagdollSkeletonAdapterTest, reportsMissingRequiredBone)
    {
        osg::ref_ptr<SceneUtil::Skeleton> skeleton = new SceneUtil::Skeleton;
        skeleton->addChild(makeBone("Bip01 Pelvis", {}).get());

        const auto result = RagdollProfile::snapshotSkeleton(makeProfile(), *skeleton);
        EXPECT_FALSE(result.isValid());
        EXPECT_FALSE(result.mSkeleton.has_value());
        ASSERT_EQ(result.mIssues.size(), 1u);
        EXPECT_NE(result.mIssues.front().mMessage.find("spine"), std::string::npos);
    }

    TEST(RagdollSkeletonAdapterTest, rejectsAmbiguousAliasMatch)
    {
        osg::ref_ptr<SceneUtil::Skeleton> skeleton = new SceneUtil::Skeleton;
        osg::ref_ptr<osg::MatrixTransform> first = makeBone("Bip01 Pelvis", {});
        osg::ref_ptr<osg::MatrixTransform> second = makeBone("BIP01 PELVIS", {});
        osg::ref_ptr<osg::MatrixTransform> spine = makeBone("Bip01 Spine", {});
        skeleton->addChild(first);
        skeleton->addChild(second);
        first->addChild(spine);

        const auto result = RagdollProfile::snapshotSkeleton(makeProfile(), *skeleton);
        EXPECT_FALSE(result.isValid());
        EXPECT_FALSE(result.mSkeleton.has_value());
        ASSERT_FALSE(result.mIssues.empty());
        EXPECT_NE(result.mIssues.front().mMessage.find("ambiguous"), std::string::npos);
    }

    TEST(RagdollSkeletonAdapterTest, prefersShallowerRigBoneOverNestedAttachmentAlias)
    {
        osg::ref_ptr<SceneUtil::Skeleton> skeleton = new SceneUtil::Skeleton;
        osg::ref_ptr<osg::MatrixTransform> pelvis = makeBone("Bip01 Pelvis", { 2.0f, 0.0f, 0.0f });
        osg::ref_ptr<osg::MatrixTransform> attachment = makeBone("pelvis", { 50.0f, 0.0f, 0.0f });
        osg::ref_ptr<osg::MatrixTransform> spine = makeBone("Bip01 Spine", { 3.0f, 0.0f, 0.0f });
        skeleton->addChild(pelvis);
        pelvis->addChild(attachment);
        pelvis->addChild(spine);

        const auto result = RagdollProfile::snapshotSkeleton(makeProfile(), *skeleton);
        ASSERT_TRUE(result.isValid());
        ASSERT_EQ(result.mSkeleton->mBones.size(), 2u);
        EXPECT_FLOAT_EQ(result.mSkeleton->mBones[0].mModelTransform.mTranslation.GetX(), 2.0f);
        EXPECT_FLOAT_EQ(result.mSkeleton->mBones[1].mModelTransform.mTranslation.GetX(), 5.0f);
    }

    TEST(RagdollSkeletonAdapterTest, reportsProfileHierarchyThatDisagreesWithSkeleton)
    {
        osg::ref_ptr<SceneUtil::Skeleton> skeleton = new SceneUtil::Skeleton;
        skeleton->addChild(makeBone("Bip01 Pelvis", {}).get());
        skeleton->addChild(makeBone("Bip01 Spine", {}).get());

        const auto result = RagdollProfile::snapshotSkeleton(makeProfile(), *skeleton);
        EXPECT_FALSE(result.isValid());
        ASSERT_TRUE(result.mSkeleton.has_value());
        bool foundHierarchyIssue = false;
        for (const RagdollProfile::SkeletonSnapshotIssue& issue : result.mIssues)
            foundHierarchyIssue = foundHierarchyIssue
                || issue.mValidationCode == RagdollCore::ValidationCode::InvalidHierarchy;
        EXPECT_TRUE(foundHierarchyIssue);
    }

    TEST(RagdollSkeletonAdapterTest, appliesCompleteModelPoseThroughCurrentParentTransforms)
    {
        osg::ref_ptr<SceneUtil::Skeleton> skeleton = new SceneUtil::Skeleton;
        osg::ref_ptr<osg::MatrixTransform> root = makeBone("Bip01", { 10.0f, 0.0f, 0.0f });
        osg::ref_ptr<osg::MatrixTransform> pelvis = makeBone("Bip01 Pelvis", { 2.0f, 0.0f, 0.0f });
        osg::ref_ptr<osg::MatrixTransform> spine = makeBone("Bip01 Spine", { 3.0f, 0.0f, 0.0f });
        skeleton->addChild(root);
        root->addChild(pelvis);
        pelvis->addChild(spine);

        auto result = RagdollProfile::snapshotSkeleton(makeProfile(), *skeleton);
        ASSERT_TRUE(result.isValid());
        EXPECT_EQ(skeleton->getUpdateCallback(), nullptr);
        std::vector<RagdollCore::Transform> captured;
        ASSERT_TRUE(result.mPoseBinding->capturePose(captured));
        ASSERT_EQ(captured.size(), 2u);
        EXPECT_FLOAT_EQ(captured[0].mTranslation.GetX(), 12.0f);
        EXPECT_FLOAT_EQ(captured[1].mTranslation.GetX(), 15.0f);
        const std::vector<RagdollCore::Transform> pose = {
            { JPH::Vec3(20.0f, 0.0f, 0.0f), JPH::Quat::sIdentity() },
            { JPH::Vec3(25.0f, 0.0f, 0.0f), JPH::Quat::sIdentity() },
        };
        ASSERT_TRUE(result.mPoseBinding->applyPose(pose));
        EXPECT_DOUBLE_EQ(pelvis->getMatrix().getTrans().x(), 10.0);
        EXPECT_DOUBLE_EQ(spine->getMatrix().getTrans().x(), 5.0);

        osg::NodePath pelvisPath{ skeleton.get(), root.get(), pelvis.get() };
        osg::NodePath spinePath{ skeleton.get(), root.get(), pelvis.get(), spine.get() };
        EXPECT_DOUBLE_EQ(osg::computeLocalToWorld(pelvisPath).getTrans().x(), 20.0);
        EXPECT_DOUBLE_EQ(osg::computeLocalToWorld(spinePath).getTrans().x(), 25.0);

        // Animation controllers run during update traversal. The binding's
        // post-traversal callback reapplies the published physics pose after
        // those controllers have written their animation transforms.
        pelvis->setMatrix(osg::Matrixf::translate(2.0f, 0.0f, 0.0f));
        spine->setMatrix(osg::Matrixf::translate(3.0f, 0.0f, 0.0f));
        osgUtil::UpdateVisitor updateVisitor;
        skeleton->accept(updateVisitor);
        EXPECT_DOUBLE_EQ(osg::computeLocalToWorld(pelvisPath).getTrans().x(), 20.0);
        EXPECT_DOUBLE_EQ(osg::computeLocalToWorld(spinePath).getTrans().x(), 25.0);
        std::vector<RagdollCore::Transform> animationPose;
        ASSERT_TRUE(result.mPoseBinding->captureAnimationPose(animationPose));
        ASSERT_EQ(animationPose.size(), 2u);
        EXPECT_FLOAT_EQ(animationPose[0].mTranslation.GetX(), 12.0f);
        EXPECT_FLOAT_EQ(animationPose[1].mTranslation.GetX(), 15.0f);

        ASSERT_NE(skeleton->getUpdateCallback(), nullptr);
        result.mPoseBinding.reset();
        EXPECT_EQ(skeleton->getUpdateCallback(), nullptr);
    }

    TEST(RagdollSkeletonAdapterTest, invalidatesSameFrameSkinningMatricesAfterApplyingPose)
    {
        osg::ref_ptr<SceneUtil::Skeleton> skeleton = new SceneUtil::Skeleton;
        osg::ref_ptr<osg::MatrixTransform> root = makeBone("Bip01", { 10.0f, 0.0f, 0.0f });
        osg::ref_ptr<osg::MatrixTransform> pelvis = makeBone("Bip01 Pelvis", { 2.0f, 0.0f, 0.0f });
        osg::ref_ptr<osg::MatrixTransform> spine = makeBone("Bip01 Spine", { 3.0f, 0.0f, 0.0f });
        skeleton->addChild(root);
        root->addChild(pelvis);
        pelvis->addChild(spine);

        auto result = RagdollProfile::snapshotSkeleton(makeProfile(), *skeleton);
        ASSERT_TRUE(result.isValid());
        SceneUtil::Bone* skinnedSpine = skeleton->getBone("Bip01 Spine");
        ASSERT_NE(skinnedSpine, nullptr);
        skeleton->updateBoneMatrices(7);
        EXPECT_DOUBLE_EQ(skinnedSpine->mMatrixInSkeletonSpace.getTrans().x(), 15.0);

        const std::vector<RagdollCore::Transform> pose = {
            { JPH::Vec3(20.0f, 0.0f, 0.0f), JPH::Quat::sIdentity() },
            { JPH::Vec3(25.0f, 0.0f, 0.0f), JPH::Quat::sIdentity() },
        };
        ASSERT_TRUE(result.mPoseBinding->applyPose(pose));

        // RigGeometry can request matrices again during cull using the same
        // traversal number. It must see the newly published ragdoll pose.
        skeleton->updateBoneMatrices(7);
        EXPECT_DOUBLE_EQ(skinnedSpine->mMatrixInSkeletonSpace.getTrans().x(), 25.0);
    }

    TEST(RagdollSkeletonAdapterTest, rejectsInvalidPoseBeforeChangingAnyNode)
    {
        osg::ref_ptr<SceneUtil::Skeleton> skeleton = new SceneUtil::Skeleton;
        osg::ref_ptr<osg::MatrixTransform> pelvis = makeBone("Bip01 Pelvis", { 2.0f, 0.0f, 0.0f });
        osg::ref_ptr<osg::MatrixTransform> spine = makeBone("Bip01 Spine", { 3.0f, 0.0f, 0.0f });
        skeleton->addChild(pelvis);
        pelvis->addChild(spine);
        auto result = RagdollProfile::snapshotSkeleton(makeProfile(), *skeleton);
        ASSERT_TRUE(result.isValid());

        std::vector<RagdollCore::Transform> pose(2);
        pose[1].mTranslation.SetX(std::numeric_limits<float>::quiet_NaN());
        EXPECT_FALSE(result.mPoseBinding->applyPose(pose));
        EXPECT_DOUBLE_EQ(pelvis->getMatrix().getTrans().x(), 2.0);
        EXPECT_DOUBLE_EQ(spine->getMatrix().getTrans().x(), 3.0);
    }

    TEST(RagdollSkeletonAdapterTest, rejectsPoseAccessAfterAnIntermediateAncestorIsDestroyed)
    {
        osg::ref_ptr<SceneUtil::Skeleton> skeleton = new SceneUtil::Skeleton;
        osg::ref_ptr<osg::MatrixTransform> root = makeBone("Bip01", { 10.0f, 0.0f, 0.0f });
        osg::ref_ptr<osg::MatrixTransform> pelvis = makeBone("Bip01 Pelvis", { 2.0f, 0.0f, 0.0f });
        osg::ref_ptr<osg::MatrixTransform> spine = makeBone("Bip01 Spine", { 3.0f, 0.0f, 0.0f });
        skeleton->addChild(root);
        root->addChild(pelvis);
        pelvis->addChild(spine);

        auto result = RagdollProfile::snapshotSkeleton(makeProfile(), *skeleton);
        ASSERT_TRUE(result.isValid());

        ASSERT_TRUE(skeleton->removeChild(root));
        root = nullptr;
        ASSERT_TRUE(pelvis.valid());
        ASSERT_TRUE(spine.valid());

        std::vector<RagdollCore::Transform> captured = {
            { JPH::Vec3(123.0f, 0.0f, 0.0f), JPH::Quat::sIdentity() },
        };
        EXPECT_FALSE(result.mPoseBinding->capturePose(captured));
        ASSERT_EQ(captured.size(), 1u);
        EXPECT_FLOAT_EQ(captured.front().mTranslation.GetX(), 123.0f);

        const std::vector<RagdollCore::Transform> pose = {
            { JPH::Vec3(20.0f, 0.0f, 0.0f), JPH::Quat::sIdentity() },
            { JPH::Vec3(25.0f, 0.0f, 0.0f), JPH::Quat::sIdentity() },
        };
        EXPECT_FALSE(result.mPoseBinding->applyPose(pose));
        EXPECT_DOUBLE_EQ(pelvis->getMatrix().getTrans().x(), 2.0);
        EXPECT_DOUBLE_EQ(spine->getMatrix().getTrans().x(), 3.0);
    }
}
