#include <ragdollcore/armsim.hpp>

#include <ragdollcore/armtarget.hpp>
#include <ragdollcore/builder.hpp>
#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/runtimeprofile.hpp>
#include <ragdollcore/scenario.hpp>
#include <ragdollcore/unitmodel.hpp>

// clang-format off
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/PhysicsSystem.h>
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <unordered_map>

namespace RagdollCore
{
    namespace
    {
        constexpr JPH::ObjectLayer sBody = 1;
        constexpr float sStep = 1.0f / 60.0f;

        class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterface
        {
        public:
            JPH::uint GetNumBroadPhaseLayers() const override { return 1; }
            JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer) const override { return JPH::BroadPhaseLayer(0); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
            const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override { return "armsim"; }
#endif
        };

        class ObjectVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer, JPH::BroadPhaseLayer) const override { return true; }
        };

        class ObjectPairs final : public JPH::ObjectLayerPairFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer, JPH::ObjectLayer) const override { return true; }
        };

        std::optional<std::size_t> findBody(const RagdollProfile& profile, const std::string& name)
        {
            for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
                if (profile.mBodies[i].mName == name)
                    return i;
            return std::nullopt;
        }
    }

    struct ArmSim::World
    {
        BroadPhaseLayers mBroadPhase;
        ObjectVsBroadPhase mObjectVsBroadPhase;
        ObjectPairs mPairs;
        JPH::PhysicsSystem mPhysics;
        JPH::TempAllocatorImpl mAllocator{ 8 * 1024 * 1024 };
        JPH::JobSystemSingleThreaded mJobs{ JPH::cMaxPhysicsJobs };
    };

    struct ArmSim::Pose
    {
        RagdollPose mPose;
    };

    ArmSim::ArmSim(const Settings& settings)
        : mWorld(std::make_unique<World>())
        , mSettings(settings)
        , mProfile(settings.mProfile)
        , mBase(std::make_unique<Pose>())
    {
        auto dump = readSkeletonDump(settings.mSkeletonPath);
        if (!dump)
        {
            mError = "the skeleton dump could not be read: " + settings.mSkeletonPath;
            return;
        }
        SkeletonDescriptor skeleton = std::move(dump->mSkeleton);
        if (const auto failed = applyAnatomicalFrames(mProfile, skeleton,
                mProfile.mActiveController.mLocomotionRoute == LocomotionRoute::KinematicRoot))
        {
            mError = std::string("the dumped skeleton's ") + std::string(*failed) + " could not be framed";
            return;
        }
        const auto hand = findBody(mProfile, "right_hand");
        const auto forearm = findBody(mProfile, "right_forearm");
        const auto upper = findBody(mProfile, "right_upper_arm");
        const auto clavicle = findBody(mProfile, "right_clavicle");
        const auto pelvis = findBody(mProfile, mProfile.mLandmarks.mPelvis);
        if (!hand || !forearm || !upper || !clavicle || !pelvis)
        {
            mError = "the profile lacks the right arm or the pelvis";
            return;
        }
        mHand = *hand;
        mForearm = *forearm;
        mUpperArm = *upper;
        mClavicle = *clavicle;
        mPelvis = *pelvis;

        // The weapon as the engine builds it (appendLiveTrainingWeapon): a box
        // along the blade (the hand bone's +Z), its centre `tip - length / 2`
        // along it from the grip in the palm, fixed to the hand, its mass a
        // share of the body's.
        const Weapon& weapon = settings.mWeapon;
        const float actorMass = dump->mTotalMass;
        const float totalMass = actorMass + weapon.mMass;
        const auto handBone = std::find_if(skeleton.mBones.begin(), skeleton.mBones.end(),
            [&](const SkeletonBone& bone) { return bone.mName == mProfile.mBodies[mHand].mBone; });
        if (handBone == skeleton.mBones.end())
        {
            mError = "the skeleton lacks the right hand's bone";
            return;
        }
        const float centre = weapon.mTip - 0.5f * weapon.mLength;
        const JPH::Vec3 offset = (gripOffset() + bladeAxis() * centre) / dump->mUniformScale;
        SkeletonBone bone;
        bone.mName = "equipped_weapon";
        bone.mParent = static_cast<int>(std::distance(skeleton.mBones.begin(), handBone));
        bone.mModelTransform = handBone->mModelTransform;
        bone.mModelTransform.mTranslation += bone.mModelTransform.mRotation * offset;
        skeleton.mBones.push_back(bone);
        for (BodyProfile& body : mProfile.mBodies)
            body.mMassFraction *= actorMass / totalMass;
        BodyProfile body;
        body.mName = body.mBone = bone.mName;
        body.mParentBody = mProfile.mBodies[mHand].mName;
        body.mMassFraction = weapon.mMass / totalMass;
        body.mShape.mType = ShapeType::Box;
        body.mShape.mHalfExtents
            = JPH::Vec3(0.5f * weapon.mWidth, 0.35f * weapon.mWidth, 0.5f * weapon.mLength) / dump->mUniformScale;
        body.mConstraint = ConstraintProfile{};
        body.mConstraint->mChildAnchor = -offset;
        body.mConstraint->mNormalHalfConeAngle = 0;
        body.mConstraint->mPlaneHalfConeAngle = 0;
        body.mConstraint->mTwistMinAngle = 0;
        body.mConstraint->mTwistMaxAngle = 0;
        body.mConstraint->mMotor.mMaximumTorque = 0;
        mProfile.mBodies.push_back(body);
        mWeaponBody = mProfile.mBodies.size() - 1;
        // The arm's holding allowance (the engine's: gravity on the weapon at
        // each joint's radius plus 40 rad/s^2 of it, metres).
        {
            const float length = weapon.mLength / UnitModel::UnitsPerMeter;
            const float width = weapon.mWidth / UnitModel::UnitsPerMeter;
            const float gripToCenter = offset.Length() * dump->mUniformScale / UnitModel::UnitsPerMeter;
            const float inertia = weapon.mMass * (length * length + width * width) / 12.0f;
            const std::array<std::size_t, 4> arm{ mClavicle, mUpperArm, mForearm, mHand };
            const std::array<float, 4> radii{ 1.0f, 0.9f, 0.6f, gripToCenter };
            for (std::size_t i = 0; i < arm.size(); ++i)
                if (auto& constraint = mProfile.mBodies[arm[i]].mConstraint)
                {
                    const float allowance = weapon.mMass * UnitModel::GravityMetersPerSecondSquared * radii[i]
                        + (inertia + weapon.mMass * radii[i] * radii[i]) * 40.0f;
                    constraint->mMotor.mMaximumTorque
                        += allowance * UnitModel::UnitsPerMeter * UnitModel::UnitsPerMeter;
                }
            for (std::size_t j = 0; j < 3; ++j)
            {
                const auto& constraint = mProfile.mBodies[arm[j + 1]].mConstraint;
                mStrength[j] = constraint ? constraint->mMotor.mMaximumTorque : 0.0f;
            }
        }
        // The weapon hand's wrist and clavicle opened as the engine opens them
        // (the authored wrist is a corpse's: the stances cock it 90 deg and
        // more): wrist cones 2.0, twist 1.6; clavicle cones 0.5, twist 0.4.
        if (auto& wrist = mProfile.mBodies[mHand].mConstraint)
        {
            wrist->mNormalHalfConeAngle = std::max(wrist->mNormalHalfConeAngle, 2.0f);
            wrist->mPlaneHalfConeAngle = std::max(wrist->mPlaneHalfConeAngle, 2.0f);
            wrist->mTwistMinAngle = std::min(wrist->mTwistMinAngle, -1.6f);
            wrist->mTwistMaxAngle = std::max(wrist->mTwistMaxAngle, 1.6f);
        }
        if (auto& collar = mProfile.mBodies[mClavicle].mConstraint)
        {
            collar->mNormalHalfConeAngle = std::max(collar->mNormalHalfConeAngle, 0.5f);
            collar->mPlaneHalfConeAngle = std::max(collar->mPlaneHalfConeAngle, 0.5f);
            collar->mTwistMinAngle = std::min(collar->mTwistMinAngle, -0.4f);
            collar->mTwistMaxAngle = std::max(collar->mTwistMaxAngle, 0.4f);
        }

        JPH::PhysicsSystem& physics = mWorld->mPhysics;
        physics.Init(256, 0, 1024, 1024, mWorld->mBroadPhase, mWorld->mObjectVsBroadPhase, mWorld->mPairs);
        physics.SetGravity(JPH::Vec3(0.0f, 0.0f, -UnitModel::GravityUnitsPerSecondSquared));
        configureScenarioPhysics(physics, 0);
        BuildOptions options;
        options.mTotalMass = totalMass;
        options.mUniformScale = dump->mUniformScale;
        options.mFrameSkeleton = dump->mFrames ? &*dump->mFrames : nullptr;
        options.mObjectLayer = sBody;
        PassiveRagdollCreateResult created = createPassiveRagdoll(physics, skeleton, mProfile, options, {}, 1, 1);
        if (!created.isValid() || !created.mRagdoll->addToPhysicsSystem())
        {
            mError = "the body could not be created";
            return;
        }
        mRagdoll = std::move(created.mRagdoll);
        std::unordered_map<std::string, const SkeletonBone*> bones;
        for (const SkeletonBone& b : skeleton.mBones)
            bones.emplace(b.mName, &b);
        for (const BodyProfile& b : mProfile.mBodies)
            mBase->mPose.mBodyTransforms.push_back(bones.at(b.mBone)->mModelTransform);
        mDisabled = { mUpperArm, mForearm, mHand };
        mRagdoll->setRootKinematic(true);
        // Bench diagnostics: OPENMW_ARM_NO_SELF=1 lets the weapon pass through its own body.
        if (std::getenv("OPENMW_ARM_NO_SELF"))
            for (std::size_t i = 0; i < mRagdoll->getBodyCount(); ++i)
                if (i != mWeaponBody)
                    mRagdoll->setBodyPairCollisionEnabled(mWeaponBody, i, false);
        JPH::BodyInterface& bodies = physics.GetBodyInterface();
        mRootPosition = bodies.GetPosition(mRagdoll->getBodyId(0));
        mRootRotation = bodies.GetRotation(mRagdoll->getBodyId(0));

        // The chest-centre frame from the shoulders.
        const auto left = findBody(mProfile, "left_upper_arm");
        if (!left)
        {
            mError = "the profile lacks the left upper arm";
            return;
        }
        const JPH::RVec3 rightShoulder = bodies.GetPosition(mRagdoll->getBodyId(mUpperArm));
        const JPH::RVec3 leftShoulder = bodies.GetPosition(mRagdoll->getBodyId(*left));
        JPH::Vec3 right(rightShoulder - leftShoulder);
        right.SetZ(0.0f);
        right = right.NormalizedOr(JPH::Vec3::sAxisX());
        const JPH::Vec3 up = JPH::Vec3::sAxisZ();
        const JPH::Vec3 forward = up.Cross(right);
        mCentre = leftShoulder + (rightShoulder - leftShoulder) * 0.5 - JPH::RVec3(up * 6.0f);
        mBodyToWorld = JPH::Mat44(JPH::Vec4(right, 0.0f), JPH::Vec4(forward, 0.0f), JPH::Vec4(up, 0.0f),
            JPH::Vec4(0.0f, 0.0f, 0.0f, 1.0f))
                           .GetQuaternion()
                           .Normalized();
        const JPH::RVec3 elbow = bodies.GetPosition(mRagdoll->getBodyId(mForearm));
        const JPH::RVec3 wrist = bodies.GetPosition(mRagdoll->getBodyId(mHand));
        mArmLength = static_cast<float>((elbow - rightShoulder).Length() + (wrist - elbow).Length());
        mValid = true;
    }

    ArmSim::~ArmSim()
    {
        mRagdoll.reset();
    }

    ArmSim::Step ArmSim::stepPosed(const HandDriveGoal& goal, const HandDriveGains& gains, float strength)
    {
        Step result;
        if (!mValid)
            return result;
        result.mChain = readChain();
        HandDriveGains feed = gains;
        feed.mRotationTorque = false;
        result.mDrive = solveHandDrive(result.mChain, goal, feed, UnitModel::GravityUnitsPerSecondSquared);

        // The arm solved as the game solves it (applyArmTargets, the chamber path's settings).
        RagdollPose measured;
        mRagdoll->captureModelPose(measured);
        RagdollPose target = mBase->mPose;
        ArmTargetInput input;
        input.mShoulderBodyIndex = mUpperArm;
        input.mElbowBodyIndex = mForearm;
        input.mHandBodyIndex = mHand;
        input.mTargetHandPosition = JPH::Vec3(goal.mGrip);
        input.mPreferredBendDirection = goal.mPole ? *goal.mPole : -JPH::Vec3::sAxisZ();
        input.mHandRotation = goal.mBladeAxis
            ? nearestEdgeRotation(goal.mRotation, result.mChain.mHandRotation, *goal.mBladeAxis)
            : goal.mRotation;
        input.mHingeGeometry = mRagdoll->getHingeLegGeometry(mUpperArm, mForearm, mHand);
        input.mShoulderGeometry = mRagdoll->getSwingTwistJointGeometry(mUpperArm);
        input.mHandGeometry = mRagdoll->getSwingTwistJointGeometry(mHand);
        input.mHandAimOffset = bladeAxis() * mSettings.mWeapon.mTip;
        input.mTipReferenceLever = 20.0f;
        input.mMaximumElbowStep = 250.0f * sStep;
        input.mPreviousElbowPosition = mPreviousElbow;
        input.mRollContinuityWeight = 0.02f;
        input.mBendPreferenceWeight = 300.0f;
        input.mBendCrossingWeight = 400.0f;
        input.mReachTolerance = 12.0f;
        input.mStraightElbowRelease = 3.0f;
        const auto solved = applyArmTargetPose(target.mBodyTransforms, mBase->mPose.mBodyTransforms,
            measured.mBodyTransforms, mClavicle, input);
        if (solved && solved->mElbowPosition)
            mPreviousElbow = solved->mElbowPosition;
        if (solved && solved->mApplied)
        {
            const JPH::Vec3 asked = goal.mRotation * bladeAxis();
            const JPH::Vec3 posed = target.mBodyTransforms[mHand].mRotation * bladeAxis();
            result.mSolvedBladeError = std::acos(std::clamp(asked.Dot(posed), -1.0f, 1.0f)) * 57.2958f;
            result.mSolvedReachError = solved->mReachError;
        }

        mRagdoll->moveRootKinematic(mRootPosition, mRootRotation, sStep);
        mRagdoll->driveToPoseUsingMotors(target);
        const std::array<std::size_t, 3> arm{ mUpperArm, mForearm, mHand };
        for (std::size_t j = 0; j < 3; ++j)
            mRagdoll->setMotorMaximumTorque(arm[j], mStrength[j] * strength * gains.mStrength);
        const std::array<std::pair<std::size_t, std::size_t>, 3> joints{ std::pair{ mUpperArm, mClavicle },
            std::pair{ mForearm, mUpperArm }, std::pair{ mHand, mForearm } };
        for (std::size_t j = 0; j < 3; ++j)
        {
            mRagdoll->applyExternalBodyWrench(joints[j].first, JPH::Vec3::sZero(), result.mDrive.mTorque[j]);
            mRagdoll->applyExternalBodyWrench(joints[j].second, JPH::Vec3::sZero(), -result.mDrive.mTorque[j]);
        }
        // Bench: the blade's turn helped from outside the body (the old
        // weapon spring's way), on top of the solved arm.
        if (gains.mComputeRotation)
            mRagdoll->applyExternalBodyWrench(mWeaponBody, JPH::Vec3::sZero(), result.mDrive.mRotationTorque);
        mWorld->mPhysics.Update(sStep, 1, &mWorld->mAllocator, &mWorld->mJobs);
        result.mGrip = grip();
        result.mHandRotation = handRotation();
        result.mTip = result.mGrip
            + JPH::RVec3(result.mHandRotation * (gripOffset() + bladeAxis() * mSettings.mWeapon.mTip));
        return result;
    }

    std::string ArmSim::jointReport() const
    {
        std::string out;
        JPH::BodyInterface& bodies = mWorld->mPhysics.GetBodyInterface();
        const std::array<std::pair<const char*, std::pair<std::size_t, std::size_t>>, 3> joints{
            std::pair{ "shoulder", std::pair{ mUpperArm, mClavicle } },
            std::pair{ "elbow", std::pair{ mForearm, mUpperArm } }, std::pair{ "wrist", std::pair{ mHand, mForearm } } };
        for (const auto& [name, pair] : joints)
        {
            const auto geometry = mRagdoll->getSwingTwistJointGeometry(pair.first);
            char line[256];
            if (!geometry)
            {
                std::snprintf(line, sizeof(line), "%s: not swing-twist; ", name);
                out += line;
                continue;
            }
            const JPH::Quat parent = bodies.GetRotation(mRagdoll->getBodyId(pair.second));
            const JPH::Quat child = bodies.GetRotation(mRagdoll->getBodyId(pair.first));
            const auto clamped = clampSwingTwistJointOrientation(*geometry, parent, child);
            float beyond = -1.0f;
            if (clamped)
                beyond = 2.0f * std::acos(std::clamp(std::abs(clamped->Dot(child)), 0.0f, 1.0f)) * 57.2958f;
            std::snprintf(line, sizeof(line), "%s: cones %.2f/%.2f twist %.2f..%.2f at-limit %.1f deg; ", name,
                geometry->mNormalHalfConeAngle, geometry->mPlaneHalfConeAngle, geometry->mTwistMinimum,
                geometry->mTwistMaximum, beyond);
            out += line;
        }
        return out;
    }

    JPH::RVec3 ArmSim::grip() const
    {
        return mWorld->mPhysics.GetBodyInterface().GetPosition(mRagdoll->getBodyId(mHand));
    }

    JPH::Quat ArmSim::handRotation() const
    {
        return mWorld->mPhysics.GetBodyInterface().GetRotation(mRagdoll->getBodyId(mHand));
    }

    HandChain ArmSim::readChain() const
    {
        HandChain chain;
        RagdollTelemetry telemetry;
        if (!mRagdoll->captureTelemetry(telemetry))
            return chain;
        const std::array<std::size_t, 3> children{ mUpperArm, mForearm, mHand };
        for (std::size_t j = 0; j < 3; ++j)
            for (const ConstraintTelemetry& constraint : telemetry.mConstraints)
                if (constraint.mChildBodyIndex == children[j])
                    chain.mJoint[j] = constraint.mPosition;
        const std::array<std::size_t, 4> bodies{ mUpperArm, mForearm, mHand, mWeaponBody };
        for (std::size_t b = 0; b < 4; ++b)
            chain.mBody[b] = { telemetry.mBodies[bodies[b]].mCenterOfMass, telemetry.mBodies[bodies[b]].mMass };
        const BodyTelemetry& hand = telemetry.mBodies[mHand];
        chain.mGrip = hand.mPosition;
        chain.mGripVelocity = hand.mMotion.mLinearVelocity
            + hand.mMotion.mAngularVelocity.Cross(JPH::Vec3(hand.mPosition - hand.mCenterOfMass));
        chain.mHandRotation = hand.mRotation;
        chain.mHandAngularVelocity = hand.mMotion.mAngularVelocity;
        chain.mStrength = mStrength;
        return chain;
    }

    ArmSim::Step ArmSim::step(const HandDriveGoal& goal, const HandDriveGains& gains)
    {
        Step result;
        if (!mValid)
            return result;
        result.mChain = readChain();
        result.mDrive = solveHandDrive(result.mChain, goal, gains, UnitModel::GravityUnitsPerSecondSquared);
        const std::array<std::pair<std::size_t, std::size_t>, 3> joints{ std::pair{ mUpperArm, mClavicle },
            std::pair{ mForearm, mUpperArm }, std::pair{ mHand, mForearm } };
        mRagdoll->moveRootKinematic(mRootPosition, mRootRotation, sStep);
        // The wrist's own motor turns the hand when the drive leaves the
        // rotation to it, at the wrist's strength.
        const bool wristMotor = !gains.mRotationTorque && !gains.mComputeRotation;
        if (wristMotor)
        {
            const std::array<std::size_t, 2> armOnly{ mUpperArm, mForearm };
            mRagdoll->driveToPoseUsingMotors(mBase->mPose, armOnly);
            mRagdoll->setMotorMaximumTorque(mHand, mStrength[2] * gains.mStrength);
            // Bench: OPENMW_ARM_WRIST="frequency,damping" for the wrist's spring.
            if (const char* text = std::getenv("OPENMW_ARM_WRIST"))
            {
                float frequency = 0.0f, damping = 0.0f;
                if (std::sscanf(text, "%f,%f", &frequency, &damping) == 2 && frequency > 0.0f)
                    mRagdoll->setMotorSpring(mHand, frequency, damping);
            }
            const JPH::Quat want = goal.mBladeAxis
                ? nearestEdgeRotation(goal.mRotation, result.mChain.mHandRotation, *goal.mBladeAxis)
                : goal.mRotation;
            mRagdoll->driveMotorToWorldOrientation(mHand, want, sStep);
            // Following, not only chasing: the route's own turn rate, relative
            // to the forearm (the motor is heavily damped).
            const JPH::Vec3 forearmSpin
                = mWorld->mPhysics.GetBodyInterface().GetAngularVelocity(mRagdoll->getBodyId(mForearm));
            mRagdoll->setMotorTargetAngularVelocity(mHand, goal.mAngularVelocity - forearmSpin);
        }
        else
            mRagdoll->driveToPoseUsingMotors(mBase->mPose, mDisabled);
        // The wrist's torque turns the hand and the weapon fixed in it: shared
        // by their inertia about the wrist, so the light hand is not spun
        // ahead of the weapon its fixed joint has to drag after it.
        const HandChain& chain = result.mChain;
        const auto aboutWrist = [&](std::size_t b) {
            const float r = static_cast<float>((chain.mBody[b].mCenterOfMass - chain.mJoint[2]).Length());
            return chain.mBody[b].mMass * (r * r + 1.0f);
        };
        const float weaponShare = aboutWrist(3) / std::max(1e-6f, aboutWrist(2) + aboutWrist(3));
        for (std::size_t j = 0; j < 3; ++j)
        {
            if (j == 2)
            {
                mRagdoll->applyExternalBodyWrench(mHand, JPH::Vec3::sZero(), result.mDrive.mTorque[j] * (1.0f - weaponShare));
                mRagdoll->applyExternalBodyWrench(mWeaponBody, JPH::Vec3::sZero(), result.mDrive.mTorque[j] * weaponShare);
            }
            else
                mRagdoll->applyExternalBodyWrench(joints[j].first, JPH::Vec3::sZero(), result.mDrive.mTorque[j]);
            mRagdoll->applyExternalBodyWrench(joints[j].second, JPH::Vec3::sZero(), -result.mDrive.mTorque[j]);
        }
        // Bench: the blade turned by the rotation's torque on the hand and
        // weapon alone, with no reaction in the body (the old weapon spring's
        // way, here as a budget to measure).
        if (gains.mComputeRotation && !gains.mRotationTorque)
        {
            mRagdoll->applyExternalBodyWrench(
                mHand, JPH::Vec3::sZero(), result.mDrive.mRotationTorque * (1.0f - weaponShare));
            mRagdoll->applyExternalBodyWrench(
                mWeaponBody, JPH::Vec3::sZero(), result.mDrive.mRotationTorque * weaponShare);
        }
        mWorld->mPhysics.Update(sStep, 1, &mWorld->mAllocator, &mWorld->mJobs);
        result.mGrip = grip();
        result.mHandRotation = handRotation();
        result.mTip = result.mGrip
            + JPH::RVec3(result.mHandRotation * (gripOffset() + bladeAxis() * mSettings.mWeapon.mTip));
        return result;
    }
}
