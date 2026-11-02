#ifndef OPENMW_APPS_RAGDOLLCORE_TESTS_FIXTURES_H
#define OPENMW_APPS_RAGDOLLCORE_TESTS_FIXTURES_H

#include <ragdollcore/profile.hpp>

namespace RagdollCoreTest
{
    inline RagdollCore::SkeletonDescriptor makeSkeleton()
    {
        RagdollCore::SkeletonDescriptor result;
        result.mId = "test_humanoid";
        result.mBones = {
            { "pelvis", -1, {} },
            { "torso", 0, { JPH::Vec3(0.0f, 10.0f, 0.0f), JPH::Quat::sIdentity() } },
        };
        return result;
    }

    inline RagdollCore::RagdollProfile makeProfile()
    {
        RagdollCore::BodyProfile pelvis;
        pelvis.mName = "pelvis";
        pelvis.mBone = "pelvis";
        pelvis.mShape.mType = RagdollCore::ShapeType::Box;
        pelvis.mShape.mHalfExtents = JPH::Vec3(5.0f, 3.0f, 4.0f);
        pelvis.mMassFraction = 0.6f;

        RagdollCore::BodyProfile torso;
        torso.mName = "torso";
        torso.mBone = "torso";
        torso.mParentBody = "pelvis";
        torso.mShape.mType = RagdollCore::ShapeType::Capsule;
        torso.mShape.mRadius = 2.5f;
        torso.mShape.mHalfHeight = 4.0f;
        torso.mMassFraction = 0.4f;
        torso.mConstraint = RagdollCore::ConstraintProfile{};

        RagdollCore::RagdollProfile result;
        result.mId = "test_humanoid";
        result.mBodies = { pelvis, torso };
        result.mLandmarks.mPelvis = "pelvis";
        result.mLandmarks.mSpine = { "torso" };
        return result;
    }
}

#endif
