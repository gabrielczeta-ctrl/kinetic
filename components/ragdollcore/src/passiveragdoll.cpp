#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/controller.hpp>

// clang-format off
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Body/MotionProperties.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Constraints/ConstraintPart/RotationEulerConstraintPart.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Skeleton/SkeletonPose.h>
// clang-format on

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <unordered_map>
#include <utility>

namespace RagdollCore
{
    namespace
    {
        bool isFinite(JPH::Vec3Arg value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        }

        bool isFinite(JPH::RVec3Arg value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        }

        bool isFinite(JPH::QuatArg value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ())
                && std::isfinite(value.GetW()) && value.IsNormalized(1e-3f);
        }

        float requestedHingeAngle(const JPH::HingeConstraint& hinge, JPH::QuatArg relative)
        {
            // Match HingeConstraint::SetTargetOrientationBS without calling a
            // setter: the frame is reconstructed from Jolt's public local
            // axes using the same helper as its constructor.
            const auto inverseInitial = JPH::RotationEulerConstraintPart::sGetInvInitialOrientationXZ(
                hinge.GetLocalSpaceNormalAxis1(), hinge.GetLocalSpaceHingeAxis1(),
                hinge.GetLocalSpaceNormalAxis2(), hinge.GetLocalSpaceHingeAxis2());
            return (relative * inverseInitial).GetRotationAngle(hinge.GetLocalSpaceHingeAxis1());
        }

        void addInitialStateIssue(ValidationResult& validation, std::string path, std::string message)
        {
            validation.mIssues.push_back(
                { ValidationCode::InvalidInitialState, std::move(path), std::move(message) });
        }

        bool isFinite(const BodyMotion& motion)
        {
            return isFinite(motion.mLinearVelocity) && isFinite(motion.mAngularVelocity);
        }

        bool isFinite(const Transform& transform)
        {
            return isFinite(transform.mTranslation) && std::isfinite(transform.mRotation.GetX())
                && std::isfinite(transform.mRotation.GetY()) && std::isfinite(transform.mRotation.GetZ())
                && std::isfinite(transform.mRotation.GetW()) && transform.mRotation.IsNormalized(1e-3f);
        }

        JPH::Vec3 clampLength(JPH::Vec3Arg value, float maximum)
        {
            const float lengthSquared = value.LengthSq();
            if (lengthSquared <= maximum * maximum)
                return value;
            return value * (maximum / std::sqrt(lengthSquared));
        }

        bool isWithinMagnitude(JPH::Vec3Arg value, float maximum)
        {
            if (!isFinite(value) || !std::isfinite(maximum) || maximum < 0.0f)
                return false;
            const float length = value.Length();
            const float tolerance = std::max(1e-3f, maximum * 1e-5f);
            return std::isfinite(length) && length <= maximum + tolerance;
        }

        void validateInitialState(
            const PassiveRagdollInitialState& state, std::size_t bodyCount, ValidationResult& validation)
        {
            if (!isFinite(state.mActorMotion))
                addInitialStateIssue(validation, "initialState.actorMotion", "Actor velocities must be finite.");
            if (!state.mBodyMotions.empty() && state.mBodyMotions.size() != bodyCount)
                addInitialStateIssue(validation, "initialState.bodyMotions",
                    "Per-body motion must be empty or contain exactly one entry per ragdoll body.");
            for (std::size_t i = 0; i < state.mBodyMotions.size(); ++i)
            {
                if (!isFinite(state.mBodyMotions[i]))
                    addInitialStateIssue(validation, "initialState.bodyMotions[" + std::to_string(i) + "]",
                        "Body velocities must be finite.");
            }
            if (state.mKillingImpulse)
            {
                if (state.mKillingImpulse->mBodyIndex >= bodyCount)
                    addInitialStateIssue(validation, "initialState.killingImpulse.bodyIndex",
                        "Killing impulse body index is outside the ragdoll.");
                if (!isFinite(state.mKillingImpulse->mImpulse) || !isFinite(state.mKillingImpulse->mWorldPoint))
                    addInitialStateIssue(validation, "initialState.killingImpulse",
                        "Killing impulse and world point must be finite.");
            }
        }
    }

    struct PassiveRagdoll::ForceMotor
    {
        std::size_t mDriven;
        std::size_t mReaction;
        JPH::Vec3 mForce = JPH::Vec3::sZero();
        JPH::Vec3 mTorque = JPH::Vec3::sZero();
        JPH::Ref<JPH::SixDOFConstraint> mConstraint;
    };

    void PassiveRagdoll::clearForceMotors()
    {
        for (auto& motor : mForceMotors)
        {
            motor->mForce = JPH::Vec3::sZero();
            motor->mTorque = JPH::Vec3::sZero();
            motor->mConstraint->SetEnabled(false);
        }
    }

    void PassiveRagdoll::beginActuationStep(bool useSolverWrenches)
    {
        clearForceMotors();
        mUseSolverWrenches = useSolverWrenches;
    }

    void PassiveRagdoll::applySolverWrench(std::size_t driven, std::size_t reaction,
        JPH::Vec3Arg force, JPH::Vec3Arg torque)
    {
        if (driven == reaction || (force.IsNearZero() && torque.IsNearZero()))
            return;
        auto found = std::find_if(mForceMotors.begin(), mForceMotors.end(), [&](const auto& motor) {
            return motor->mDriven == driven && motor->mReaction == reaction;
        });
        if (found == mForceMotors.end())
        {
            auto motor = std::make_unique<ForceMotor>();
            motor->mDriven = driven;
            motor->mReaction = reaction;
            JPH::SixDOFConstraintSettings settings;
            // All six axes stay free. Jolt's translation motor applies its
            // force and reaction at the body-2 anchor, even after the bodies
            // separate. Keeping that anchor at the driven COM preserves the
            // common-point wrench without a positional weld.
            settings.mPosition1 = settings.mPosition2
                = mPhysicsSystem->GetBodyInterface().GetCenterOfMassPosition(getBodyId(driven));
            motor->mConstraint = static_cast<JPH::SixDOFConstraint*>(mPhysicsSystem->GetBodyInterface()
                    .CreateConstraint(&settings, getBodyId(reaction), getBodyId(driven)));
            mPhysicsSystem->AddConstraint(motor->mConstraint);
            mForceMotors.push_back(std::move(motor));
            found = std::prev(mForceMotors.end());
        }
        auto& motor = **found;
        motor.mForce += force;
        motor.mTorque += torque;
        const auto& constraint = *motor.mConstraint;
        const JPH::Quat forceFrame = constraint.GetBody1()->GetRotation()
            * constraint.GetConstraintToBody1Matrix().GetQuaternion();
        // Jolt solves translation in the parent frame and angular motors in
        // the child frame. Capture both before the integration step.
        const JPH::Quat torqueFrame = constraint.GetBody2()->GetRotation()
            * constraint.GetConstraintToBody2Matrix().GetQuaternion();
        const JPH::Vec3 localForce = forceFrame.InverseRotate(motor.mForce);
        const JPH::Vec3 localTorque = torqueFrame.InverseRotate(motor.mTorque);
        for (int i = 0; i < 3; ++i)
        {
            const auto axis = static_cast<JPH::SixDOFConstraint::EAxis>(i);
            auto& settings = motor.mConstraint->GetMotorSettings(axis);
            settings.mMinForceLimit = settings.mMaxForceLimit = localForce[i];
            motor.mConstraint->SetMotorState(axis, JPH::EMotorState::Velocity);
            const auto rotationAxis = static_cast<JPH::SixDOFConstraint::EAxis>(i + 3);
            auto& rotationSettings = motor.mConstraint->GetMotorSettings(rotationAxis);
            rotationSettings.mMinTorqueLimit = rotationSettings.mMaxTorqueLimit = localTorque[i];
            motor.mConstraint->SetMotorState(rotationAxis, JPH::EMotorState::Velocity);
        }
        motor.mConstraint->SetEnabled(true);
        mPhysicsSystem->GetBodyInterface().ActivateBody(getBodyId(driven));
        mPhysicsSystem->GetBodyInterface().ActivateBody(getBodyId(reaction));
    }

    PassiveRagdoll::PassiveRagdoll(JPH::PhysicsSystem& physicsSystem, std::unique_ptr<JPH::Ragdoll> ragdoll,
        JPH::RVec3Arg referencePosition, JPH::QuatArg referenceRotation, float uniformScale,
        std::vector<float> bodyMasses, std::optional<KillingImpulse> pendingKillingImpulse)
        : mPhysicsSystem(&physicsSystem)
        , mRagdoll(std::move(ragdoll))
        , mReferencePosition(referencePosition)
        , mInverseReferenceRotation(referenceRotation.Conjugated())
        , mInverseUniformScale(1.0f / uniformScale)
        , mBodyMasses(std::move(bodyMasses))
        , mPendingKillingImpulse(std::move(pendingKillingImpulse))
        , mPoseScratch(mRagdoll->GetBodyCount())
        , mMotorPose(std::make_unique<JPH::SkeletonPose>())
        , mRequestedHingeAngles(mRagdoll->GetBodyCount())
    {
        mMotorPose->SetSkeleton(mRagdoll->GetRagdollSettings()->GetSkeleton());
    }

    PassiveRagdoll::~PassiveRagdoll()
    {
        removeFromPhysicsSystem();
    }

    bool PassiveRagdoll::addToPhysicsSystem(JPH::EActivation activation)
    {
        if (mAdded)
            return false;
        // Creation starts passive. Apply its solver settings before the first
        // impact; the live powered branch does not visit ordinary corpses.
        disableMotors();
        mRagdoll->AddToPhysicsSystem(activation);
        mAdded = true;
        if (mPendingKillingImpulse)
        {
            const KillingImpulse& impulse = *mPendingKillingImpulse;
            const JPH::BodyID bodyId = getBodyId(impulse.mBodyIndex);
            if (activation == JPH::EActivation::Activate)
                mPhysicsSystem->GetBodyInterface().AddImpulse(bodyId, impulse.mImpulse, impulse.mWorldPoint);
            else
            {
                JPH::BodyLockWrite lock(mPhysicsSystem->GetBodyLockInterface(), bodyId);
                if (lock.Succeeded())
                    lock.GetBody().AddImpulse(impulse.mImpulse, impulse.mWorldPoint);
            }
            mPendingKillingImpulse.reset();
        }
        return true;
    }

    void PassiveRagdoll::removeFromPhysicsSystem()
    {
        if (!mAdded)
            return;
        destroySoftAnchor();
        for (const auto& motor : mForceMotors)
            mPhysicsSystem->RemoveConstraint(motor->mConstraint);
        mForceMotors.clear();
        mRagdoll->RemoveFromPhysicsSystem();
        mAdded = false;
    }

    bool PassiveRagdoll::isActive() const
    {
        return mAdded && mRagdoll->IsActive();
    }

    std::size_t PassiveRagdoll::getBodyCount() const
    {
        return mRagdoll->GetBodyCount();
    }

    JPH::BodyID PassiveRagdoll::getBodyId(std::size_t index) const
    {
        if (index >= getBodyCount())
            return {};
        return mRagdoll->GetBodyID(static_cast<int>(index));
    }

    bool PassiveRagdoll::setRootKinematic(bool kinematic)
    {
        if (!mAdded || mPhysicsSystem == nullptr || getBodyCount() == 0 || mRootKinematic == kinematic)
            return false;
        if (mSoftRoot.mEnabled)
        {
            // Held, but the body stays dynamic; the spring does the holding.
            destroySoftAnchor();
            mRootKinematic = kinematic;
            mSoftRootPreviousPosition.reset();
            mSoftRootPreviousRotation.reset();
            return true;
        }
        JPH::BodyInterface& bodyInterface = mPhysicsSystem->GetBodyInterface();
        const JPH::BodyID root = getBodyId(0);
        bodyInterface.SetMotionType(root,
            kinematic ? JPH::EMotionType::Kinematic : JPH::EMotionType::Dynamic,
            JPH::EActivation::Activate);
        mRootKinematic = kinematic;
        return true;
    }

    bool PassiveRagdoll::moveRootKinematic(
        JPH::RVec3Arg position, JPH::QuatArg rotation, float duration)
    {
        if (!mAdded || mPhysicsSystem == nullptr || !mRootKinematic || getBodyCount() == 0
            || !isFinite(position) || !isFinite(rotation) || !rotation.IsNormalized(1e-3f)
            || !std::isfinite(duration) || duration <= 0.0f)
            return false;
        if (mSoftRoot.mEnabled)
            return applySoftRootWrench(position, rotation, duration);
        mPhysicsSystem->GetBodyInterface().MoveKinematic(getBodyId(0), position, rotation, duration);
        return true;
    }

    void PassiveRagdoll::setSoftRoot(const SoftRootSettings& settings)
    {
        const bool wasHeldKinematic = mRootKinematic && !mSoftRoot.mEnabled;
        mSoftRoot = settings;
        mSoftRootPreviousPosition.reset();
        mSoftRootPreviousRotation.reset();
        if (wasHeldKinematic && settings.mEnabled && mAdded && mPhysicsSystem != nullptr && getBodyCount() > 0)
            mPhysicsSystem->GetBodyInterface().SetMotionType(
                getBodyId(0), JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
    }

    void PassiveRagdoll::setSoftRootStrength(float strength)
    {
        if (std::isfinite(strength))
            mSoftRootStrength = std::clamp(strength, 0.0f, 1.0f);
    }

    void PassiveRagdoll::setSoftRootSupport(float support)
    {
        if (std::isfinite(support))
            mSoftRootSupport = std::clamp(support, 0.0f, 1.0f);
    }

    bool PassiveRagdoll::addLinearVelocity(JPH::Vec3Arg velocity)
    {
        if (!mAdded || mPhysicsSystem == nullptr || !isFinite(velocity))
            return false;
        JPH::BodyInterface& bodies = mPhysicsSystem->GetBodyInterface();
        for (std::size_t i = 0; i < getBodyCount(); ++i)
            if (bodies.GetMotionType(getBodyId(i)) == JPH::EMotionType::Dynamic)
                bodies.AddLinearVelocity(getBodyId(i), velocity);
        return true;
    }

    bool PassiveRagdoll::addLinearVelocity(JPH::Vec3Arg velocity, const std::vector<float>& scale)
    {
        if (!mAdded || mPhysicsSystem == nullptr || !isFinite(velocity))
            return false;
        JPH::BodyInterface& bodies = mPhysicsSystem->GetBodyInterface();
        const auto share = [&](std::size_t i) { return i < scale.size() ? std::max(scale[i], 0.0f) : 1.0f; };
        float total = 0.0f, weighted = 0.0f;
        for (std::size_t i = 0; i < getBodyCount() && i < mBodyMasses.size(); ++i)
            if (bodies.GetMotionType(getBodyId(i)) == JPH::EMotionType::Dynamic)
            {
                total += mBodyMasses[i];
                weighted += mBodyMasses[i] * share(i);
            }
        if (!(weighted > 0.0f))
            return false;
        for (std::size_t i = 0; i < getBodyCount() && i < mBodyMasses.size(); ++i)
            if (bodies.GetMotionType(getBodyId(i)) == JPH::EMotionType::Dynamic)
                bodies.AddLinearVelocity(getBodyId(i), velocity * (share(i) * total / weighted));
        return true;
    }

    bool PassiveRagdoll::setBodyWorldTransform(
        std::size_t bodyIndex, JPH::RVec3Arg position, JPH::QuatArg rotation, std::size_t velocityFrom)
    {
        if (!mAdded || mPhysicsSystem == nullptr || bodyIndex >= getBodyCount() || velocityFrom >= getBodyCount()
            || !isFinite(position) || !isFinite(rotation) || !rotation.IsNormalized())
            return false;
        JPH::BodyInterface& bodies = mPhysicsSystem->GetBodyInterface();
        const JPH::BodyID id = getBodyId(bodyIndex);
        if (bodies.GetMotionType(id) != JPH::EMotionType::Dynamic)
            return false;
        const JPH::Vec3 velocity = bodies.GetLinearVelocity(getBodyId(velocityFrom));
        bodies.SetPositionRotationAndVelocity(id, position, rotation, velocity, JPH::Vec3::sZero());
        return true;
    }

    void PassiveRagdoll::destroySoftAnchor()
    {
        if (mPhysicsSystem == nullptr)
            return;
        if (mSoftAnchorConstraint)
        {
            mPhysicsSystem->RemoveConstraint(mSoftAnchorConstraint);
            mSoftAnchorConstraint = nullptr;
        }
        if (!mSoftAnchor.IsInvalid())
        {
            JPH::BodyInterface& bodies = mPhysicsSystem->GetBodyInterface();
            bodies.RemoveBody(mSoftAnchor);
            bodies.DestroyBody(mSoftAnchor);
            mSoftAnchor = JPH::BodyID();
        }
    }

    bool PassiveRagdoll::driveSoftAnchor(JPH::RVec3Arg position, JPH::QuatArg rotation, float duration, float torqueLimit)
    {
        if (!mSoftAnchorLayer || mPhysicsSystem == nullptr)
            return false;
        JPH::BodyInterface& bodies = mPhysicsSystem->GetBodyInterface();
        const JPH::BodyID pelvis = getBodyId(0);
        if (mSoftAnchor.IsInvalid())
        {
            // The anchor starts at the pelvis pose, so the motor's rest
            // orientation is the pelvis's own; moving the anchor to the target
            // then pulls the pelvis after it.
            JPH::RVec3 pelvisPosition;
            JPH::Quat pelvisRotation;
            bodies.GetPositionAndRotation(pelvis, pelvisPosition, pelvisRotation);
            JPH::BodyCreationSettings anchor(new JPH::SphereShape(0.05f), pelvisPosition, pelvisRotation,
                JPH::EMotionType::Kinematic, *mSoftAnchorLayer);
            anchor.mIsSensor = true;
            mSoftAnchor = bodies.CreateAndAddBody(anchor, JPH::EActivation::Activate);
            if (mSoftAnchor.IsInvalid())
                return false;
            JPH::SixDOFConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::WorldSpace;
            settings.mPosition1 = settings.mPosition2 = bodies.GetCenterOfMassPosition(pelvis);
            settings.mAxisX1 = settings.mAxisX2 = pelvisRotation.RotateAxisX();
            settings.mAxisY1 = settings.mAxisY2 = pelvisRotation.RotateAxisY();
            for (int i = 0; i < 6; ++i)
                settings.MakeFreeAxis(static_cast<JPH::SixDOFConstraintSettings::EAxis>(i));
            mSoftAnchorConstraint = static_cast<JPH::SixDOFConstraint*>(
                bodies.CreateConstraint(&settings, mSoftAnchor, pelvis));
            if (!mSoftAnchorConstraint)
                return false;
            mPhysicsSystem->AddConstraint(mSoftAnchorConstraint);
        }
        bodies.MoveKinematic(mSoftAnchor, position, rotation, duration);
        for (int i = 3; i < 6; ++i)
        {
            const auto axis = static_cast<JPH::SixDOFConstraint::EAxis>(i);
            JPH::MotorSettings& motor = mSoftAnchorConstraint->GetMotorSettings(axis);
            motor.mSpringSettings.mMode = JPH::ESpringMode::FrequencyAndDamping;
            motor.mSpringSettings.mFrequency = mSoftAnchorFrequencyCap > 0.0f
                ? std::min(mSoftRoot.mBody.mAnchorFrequency, mSoftAnchorFrequencyCap)
                : mSoftRoot.mBody.mAnchorFrequency;
            motor.mSpringSettings.mDamping = mSoftRoot.mBody.mDampingRatio;
            motor.SetTorqueLimit(torqueLimit);
            mSoftAnchorConstraint->SetMotorState(axis, JPH::EMotorState::Position);
        }
        mSoftAnchorConstraint->SetTargetOrientationCS(JPH::Quat::sIdentity());
        bodies.ActivateBody(pelvis);
        return true;
    }

    bool PassiveRagdoll::applySoftRootWrench(JPH::RVec3Arg position, JPH::QuatArg rotation, float duration)
    {
        JPH::BodyInterface& bodies = mPhysicsSystem->GetBodyInterface();
        const std::size_t count = getBodyCount();
        const JPH::BodyID root = getBodyId(0);
        if (bodies.GetMotionType(root) != JPH::EMotionType::Dynamic)
            bodies.SetMotionType(root, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
        if (mBodyMasses.size() != count || count == 0)
            return false;
        const auto flag = [&](const std::vector<bool>& mask, std::size_t i, bool fallback) {
            return mask.size() == count ? bool(mask[i]) : fallback;
        };

        ActiveBodyInput input;
        input.mDuration = duration;
        input.mGravity = mPhysicsSystem->GetGravity();
        input.mStrength = mSoftRootStrength;
        input.mHorizontalScale = mSoftRootHorizontalScale;
        input.mSupport = mSoftRootSupport;
        JPH::RVec3 current;
        JPH::Quat currentRotation;
        bodies.GetPositionAndRotation(root, current, currentRotation);
        // Positions relative to the current root keep float precision.
        input.mRootPosition = JPH::Vec3::sZero();
        input.mRootRotation = currentRotation;
        input.mTargetPosition = JPH::Vec3(position - current);
        input.mTargetRotation = rotation;
        if (mSoftRootPreviousPosition)
            input.mTargetVelocity = JPH::Vec3(position - *mSoftRootPreviousPosition) / duration;
        if (mSoftRootPreviousRotation)
        {
            JPH::Quat delta = rotation * mSoftRootPreviousRotation->Conjugated();
            if (delta.GetW() < 0.0f)
                delta = -delta;
            JPH::Vec3 axis;
            float angle;
            delta.GetAxisAngle(axis, angle);
            if (std::isfinite(angle) && angle > 1e-6f)
                input.mTargetAngularVelocity = axis * (angle / duration);
        }
        mSoftRootPreviousPosition = position;
        mSoftRootPreviousRotation = rotation;
        // A teleport-sized target jump (respawn, cell change) is not motion.
        if (input.mTargetVelocity.Length() > (mSoftRootJumping ? mSoftRootJumpTeleportSpeed : 2000.0f))
            input.mTargetVelocity = JPH::Vec3::sZero();
        if (isFinite(mSoftRootVelocityBias))
            input.mTargetVelocity -= mSoftRootVelocityBias;
        input.mParts.resize(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            ActiveBodyPart& part = input.mParts[i];
            const JPH::BodyID id = getBodyId(i);
            part.mMass = mBodyMasses[i];
            part.mVelocity = bodies.GetLinearVelocity(id);
            part.mAngularVelocity = bodies.GetAngularVelocity(id);
            part.mUpper = flag(mSoftRootUpper, i, true);
            part.mTorso = flag(mSoftRootTorso, i, i == 0);
            part.mRoot = i == 0;
            part.mFoot = flag(mSoftRootFeet, i, false);
            part.mGrounded = part.mFoot && flag(mSoftRootGrounded, i, false);
        }
        bool anyGrounded = false;
        for (const ActiveBodyPart& part : input.mParts)
            anyGrounded = anyGrounded || part.mGrounded;
        mSoftRootAirborneTime = anyGrounded ? 0.0f : mSoftRootAirborneTime + duration;
        input.mAirborneTime = mSoftRootAirborneTime;
        if (mSoftRootJumping)
        {
            for (ActiveBodyPart& part : input.mParts)
                part.mGrounded = false;
            input.mAirborneTime = 1000.0f;
            input.mUpCapScale = mSoftRootJumpCap;
        }
        ActiveBodyOutput output = computeActiveBodyWrench(mSoftRoot.mBody, input);
        if (!output.mValid)
            return false;
        if (mSoftRoot.mBody.mRotationConstraint > 0.0f)
        {
            float totalMass = 0.0f;
            for (float mass : mBodyMasses)
                totalMass += mass;
            const float torqueLimit = mSoftRoot.mBody.mAnchorTorque * totalMass
                * mPhysicsSystem->GetGravity().Length() * mSoftRoot.mBody.mGyrationRadius
                * std::clamp(mSoftRootStrength, 0.0f, 1.0f);
            if (driveSoftAnchor(position, rotation, duration, torqueLimit))
                std::fill(output.mAngularAccelerations.begin(), output.mAngularAccelerations.end(),
                    JPH::Vec3::sZero());
        }
        else
            destroySoftAnchor();

        JPH::Vec3 totalTorque = JPH::Vec3::sZero();
        for (std::size_t i = 0; i < count; ++i)
        {
            const JPH::BodyID id = getBodyId(i);
            if (!output.mForces[i].IsNearZero())
                bodies.AddForce(id, output.mForces[i], JPH::EActivation::Activate);
            if (output.mAngularAccelerations[i].IsNearZero())
                continue;
            // I * alpha in the body's own inertia frame.
            JPH::Vec3 torque = JPH::Vec3::sZero();
            {
                JPH::BodyLockRead lock(mPhysicsSystem->GetBodyLockInterface(), id);
                if (!lock.Succeeded())
                    continue;
                const JPH::Body& body = lock.GetBody();
                const JPH::MotionProperties* motion = body.GetMotionProperties();
                if (motion == nullptr)
                    continue;
                const JPH::Vec3 inverseInertia = motion->GetInverseInertiaDiagonal();
                if (!(inverseInertia.GetX() > 0.0f && inverseInertia.GetY() > 0.0f && inverseInertia.GetZ() > 0.0f))
                    continue;
                const JPH::Quat frame = body.GetRotation() * motion->GetInertiaRotation();
                torque = frame * (frame.InverseRotate(output.mAngularAccelerations[i]) / inverseInertia);
            }
            bodies.AddTorque(id, torque, JPH::EActivation::Activate);
            totalTorque += torque;
        }
        mSoftRootTelemetry
            = { output.mNetForce, totalTorque, output.mPositionError, output.mAngleError, output.mSaturated };
        return true;
    }

    void PassiveRagdoll::setSoftRootBodies(std::vector<bool> upper, std::vector<bool> torso, std::vector<bool> feet)
    {
        mSoftRootUpper = std::move(upper);
        mSoftRootTorso = std::move(torso);
        mSoftRootFeet = std::move(feet);
        mSoftRootGrounded.assign(mSoftRootFeet.size(), false);
    }

    void PassiveRagdoll::setSoftRootGrounded(std::size_t bodyIndex, bool grounded)
    {
        if (bodyIndex < mSoftRootGrounded.size())
            mSoftRootGrounded[bodyIndex] = grounded;
    }

    bool PassiveRagdoll::releaseRootKinematic(JPH::Vec3Arg velocity)
    {
        if (!mAdded || mPhysicsSystem == nullptr || !mRootKinematic || getBodyCount() == 0)
            return false;
        if (mSoftRoot.mEnabled)
        {
            // The body is already moving physically; keep its own momentum.
            destroySoftAnchor();
            mRootKinematic = false;
            mSoftRootPreviousPosition.reset();
            mSoftRootPreviousRotation.reset();
            return true;
        }
        JPH::BodyInterface& bodyInterface = mPhysicsSystem->GetBodyInterface();
        const JPH::BodyID root = getBodyId(0);
        bodyInterface.SetMotionType(root, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
        if (isFinite(velocity))
            bodyInterface.SetLinearVelocity(root, velocity);
        mRootKinematic = false;
        return true;
    }

    bool PassiveRagdoll::setBodyMass(std::size_t bodyIndex, float mass)
    {
        if (!mAdded || mPhysicsSystem == nullptr || bodyIndex >= getBodyCount() || !std::isfinite(mass) || mass <= 0.0f)
            return false;
        JPH::BodyLockWrite lock(mPhysicsSystem->GetBodyLockInterface(), getBodyId(bodyIndex));
        if (!lock.Succeeded() || !lock.GetBody().IsDynamic())
            return false;
        lock.GetBody().GetMotionProperties()->ScaleToMass(mass);
        if (bodyIndex < mBodyMasses.size())
            mBodyMasses[bodyIndex] = mass;
        return true;
    }

    bool PassiveRagdoll::setBodyFrictionScale(std::size_t bodyIndex, float scale)
    {
        if (!mAdded || mPhysicsSystem == nullptr || bodyIndex >= getBodyCount() || !std::isfinite(scale)
            || scale < 0.0f || scale > 4.0f)
            return false;
        const auto& parts = mRagdoll->GetRagdollSettings()->mParts;
        if (bodyIndex >= parts.size())
            return false;
        const float authored = parts[bodyIndex].mFriction;
        mPhysicsSystem->GetBodyInterface().SetFriction(getBodyId(bodyIndex), authored * scale);
        return true;
    }

    std::optional<bool> PassiveRagdoll::isBodyPairCollisionEnabled(std::size_t first, std::size_t second) const
    {
        if (!mAdded || !mPhysicsSystem || first == second || first >= getBodyCount() || second >= getBodyCount())
            return std::nullopt;
        const std::array ids{ getBodyId(first), getBodyId(second) };
        JPH::BodyLockMultiRead lock(mPhysicsSystem->GetBodyLockInterface(), ids.data(), static_cast<int>(ids.size()));
        const auto* firstBody = lock.GetBody(0);
        const auto* secondBody = lock.GetBody(1);
        if (!firstBody || !secondBody)
            return std::nullopt;
        const auto& firstGroup = firstBody->GetCollisionGroup();
        const auto& secondGroup = secondBody->GetCollisionGroup();
        const auto* filter = JPH::DynamicCast<JPH::GroupFilterTable>(firstGroup.GetGroupFilter());
        if (!filter || firstGroup.GetGroupFilter() != secondGroup.GetGroupFilter()
            || firstGroup.GetGroupID() == JPH::CollisionGroup::cInvalidGroup
            || firstGroup.GetGroupID() != secondGroup.GetGroupID()
            || firstGroup.GetSubGroupID() != first || secondGroup.GetSubGroupID() != second)
            return std::nullopt;
        return filter->IsCollisionEnabled(firstGroup.GetSubGroupID(), secondGroup.GetSubGroupID());
    }

    bool PassiveRagdoll::setBodyPairCollisionEnabled(std::size_t first, std::size_t second, bool enabled)
    {
        if (!mAdded || !mPhysicsSystem || first == second || first >= getBodyCount() || second >= getBodyCount())
            return false;
        const auto& ids = mRagdoll->GetBodyIDs();
        {
            // Validate every body before changing any group. Holding all of
            // this rig's locks avoids a partly reassigned live filter table.
            JPH::BodyLockMultiWrite lock(mPhysicsSystem->GetBodyLockInterface(), ids.data(), static_cast<int>(ids.size()));
            std::vector<JPH::CollisionGroup> groups;
            groups.reserve(ids.size());
            for (std::size_t i = 0; i < ids.size(); ++i)
            {
                const auto* body = lock.GetBody(static_cast<int>(i));
                if (!body)
                    return false;
                groups.push_back(body->GetCollisionGroup());
                if (groups.back().GetSubGroupID() != i
                    || groups.back().GetGroupID() == JPH::CollisionGroup::cInvalidGroup
                    || groups.back().GetGroupID() != groups.front().GetGroupID()
                    || groups.back().GetGroupFilter() != groups.front().GetGroupFilter())
                    return false;
            }
            const auto* filter = JPH::DynamicCast<JPH::GroupFilterTable>(groups.front().GetGroupFilter());
            if (!filter)
                return false;
            const auto firstSubgroup = groups[first].GetSubGroupID();
            const auto secondSubgroup = groups[second].GetSubGroupID();
            if (filter->IsCollisionEnabled(firstSubgroup, secondSubgroup) == enabled)
                return true;
            // Copy on every effective change, including restoration. A filter
            // can be retained by settings, snapshots or another instance; none
            // of those owners may observe this ragdoll's temporary override.
            JPH::Ref<JPH::GroupFilterTable> changed = new JPH::GroupFilterTable(*filter);
            if (enabled)
                changed->EnableCollision(firstSubgroup, secondSubgroup);
            else
                changed->DisableCollision(firstSubgroup, secondSubgroup);
            for (std::size_t i = 0; i < ids.size(); ++i)
            {
                groups[i].SetGroupFilter(changed);
                lock.GetBody(static_cast<int>(i))->SetCollisionGroup(groups[i]);
            }
        }
        // Cached collision pairs must be reconsidered even though positions,
        // broadphase layers and all other collision rules are unchanged.
        auto& bodies = mPhysicsSystem->GetBodyInterface();
        bodies.InvalidateContactCache(ids[first]);
        bodies.InvalidateContactCache(ids[second]);
        return true;
    }

    bool PassiveRagdoll::captureModelPose(RagdollPose& out)
    {
        if (!mAdded)
            return false;

        JPH::RVec3 rootOffset;
        mRagdoll->GetPose(rootOffset, mPoseScratch.data());

        out.mBodyTransforms.resize(mPoseScratch.size());
        for (std::size_t i = 0; i < mPoseScratch.size(); ++i)
        {
            const JPH::RVec3 worldPosition = rootOffset + mPoseScratch[i].GetTranslation();
            Transform& transform = out.mBodyTransforms[i];
            transform.mTranslation
                = (mInverseReferenceRotation * JPH::Vec3(worldPosition - mReferencePosition)) * mInverseUniformScale;
            transform.mRotation = mInverseReferenceRotation * mPoseScratch[i].GetQuaternion();
        }
        return true;
    }

    std::optional<float> PassiveRagdoll::getPoseFloorHeight(
        const RagdollPose& pose, std::size_t bodyIndex) const
    {
        if (pose.mBodyTransforms.size() != getBodyCount() || bodyIndex >= getBodyCount())
            return std::nullopt;
        const auto& bodies = mPhysicsSystem->GetBodyInterface();
        const auto origin = getWorldBodyPosition(pose, bodyIndex);
        const auto rotation = getWorldBodyRotation(pose, bodyIndex);
        const auto shape = bodies.GetShape(getBodyId(bodyIndex));
        if (!origin || !rotation || !shape) return std::nullopt;
        const JPH::RVec3 center = *origin + *rotation * shape->GetCenterOfMass();
        float lowest = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < getBodyCount(); ++i)
        {
            const auto position = getWorldBodyPosition(pose, i);
            const auto orientation = getWorldBodyRotation(pose, i);
            const auto bodyShape = bodies.GetShape(getBodyId(i));
            if (!position || !orientation || !bodyShape) return std::nullopt;
            // Work relative to the queried COM to avoid subtracting large
            // world coordinates after conversion to single-precision bounds.
            const JPH::Vec3 offset(*position - center);
            const auto transform = JPH::Mat44::sRotationTranslation(*orientation,
                offset + *orientation * bodyShape->GetCenterOfMass());
            const auto bounds = bodyShape->GetWorldSpaceBounds(transform, JPH::Vec3::sOne());
            if (!isFinite(bounds.mMin) || !isFinite(bounds.mMax)) return std::nullopt;
            lowest = std::min(lowest, bounds.mMin.GetZ());
        }
        return std::isfinite(lowest) ? std::optional<float>(-lowest) : std::nullopt;
    }

    std::optional<float> PassiveRagdoll::getPoseBodyFloorHeight(
        const RagdollPose& pose, std::size_t bodyIndex) const
    {
        if (pose.mBodyTransforms.size() != getBodyCount() || bodyIndex >= getBodyCount())
            return std::nullopt;
        const auto origin = getWorldBodyPosition(pose, bodyIndex);
        const auto rotation = getWorldBodyRotation(pose, bodyIndex);
        const auto shape = mPhysicsSystem->GetBodyInterface().GetShape(getBodyId(bodyIndex));
        if (!origin || !rotation || !shape)
            return std::nullopt;
        // Bounds relative to this body's own center of mass, then lifted to
        // world by the measured COM height.
        const JPH::Vec3 centerOffset = *rotation * shape->GetCenterOfMass();
        const auto transform = JPH::Mat44::sRotationTranslation(*rotation, centerOffset);
        const auto bounds = shape->GetWorldSpaceBounds(transform, JPH::Vec3::sOne());
        if (!isFinite(bounds.mMin))
            return std::nullopt;
        return static_cast<float>((*origin).GetZ() + centerOffset.GetZ() + bounds.mMin.GetZ());
    }

    std::optional<JPH::RVec3> PassiveRagdoll::getWorldBodyPosition(
        const RagdollPose& pose, std::size_t bodyIndex) const
    {
        if (bodyIndex >= pose.mBodyTransforms.size()
            || !isFinite(pose.mBodyTransforms[bodyIndex].mTranslation))
            return std::nullopt;

        const JPH::Vec3 scaledModelPosition
            = pose.mBodyTransforms[bodyIndex].mTranslation / mInverseUniformScale;
        return mReferencePosition + mInverseReferenceRotation.Conjugated() * scaledModelPosition;
    }

    std::optional<JPH::Quat> PassiveRagdoll::getWorldBodyRotation(
        const RagdollPose& pose, std::size_t bodyIndex) const
    {
        if (bodyIndex >= pose.mBodyTransforms.size()
            || !isFinite(pose.mBodyTransforms[bodyIndex].mRotation))
            return std::nullopt;
        const JPH::Quat result
            = mInverseReferenceRotation.Conjugated() * pose.mBodyTransforms[bodyIndex].mRotation;
        if (!isFinite(result) || !result.IsNormalized())
            return std::nullopt;
        return result;
    }

    std::optional<JPH::Quat> clampSwingTwistJointOrientation(const SwingTwistJointGeometry& geometry,
        JPH::QuatArg parentRotation, JPH::QuatArg childRotation)
    {
        for (const auto& rotation : { geometry.mParentFrame, geometry.mChildFrame, parentRotation, childRotation })
            if (!isFinite(rotation) || !rotation.IsNormalized())
                return std::nullopt;
        const float pi = std::numbers::pi_v<float>;
        if (!std::isfinite(geometry.mNormalHalfConeAngle) || geometry.mNormalHalfConeAngle < 0
            || geometry.mNormalHalfConeAngle > pi || !std::isfinite(geometry.mPlaneHalfConeAngle)
            || geometry.mPlaneHalfConeAngle < 0 || geometry.mPlaneHalfConeAngle > pi
            || !std::isfinite(geometry.mTwistMinimum) || !std::isfinite(geometry.mTwistMaximum)
            || geometry.mTwistMinimum < -pi || geometry.mTwistMaximum > pi
            || geometry.mTwistMinimum > geometry.mTwistMaximum)
            return std::nullopt;
        JPH::SwingTwistConstraintPart limits;
        limits.SetSwingType(geometry.mPyramid ? JPH::ESwingType::Pyramid : JPH::ESwingType::Cone);
        limits.SetLimits(geometry.mTwistMinimum, geometry.mTwistMaximum,
            -geometry.mPlaneHalfConeAngle, geometry.mPlaneHalfConeAngle,
            -geometry.mNormalHalfConeAngle, geometry.mNormalHalfConeAngle);
        const auto frame = (parentRotation * geometry.mParentFrame).Normalized();
        const auto request = (frame.Conjugated() * childRotation * geometry.mChildFrame).Normalized();
        JPH::Quat swing, twist;
        request.GetSwingTwist(swing, twist);
        JPH::uint clamped = 0;
        limits.ClampSwingTwist(swing, twist, clamped);
        if (clamped == 0)
            return childRotation;
        return (frame * (swing * twist) * geometry.mChildFrame.Conjugated()).Normalized();
    }

    std::optional<SwingTwistJointGeometry> PassiveRagdoll::getSwingTwistJointGeometry(std::size_t bodyIndex) const
    {
        if (bodyIndex >= getBodyCount())
            return std::nullopt;
        const int index = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0)
            return std::nullopt;
        const auto* joint = mRagdoll->GetConstraint(index);
        if (joint->GetSubType() != JPH::EConstraintSubType::SwingTwist)
            return std::nullopt;
        const auto* motor = static_cast<const JPH::SwingTwistConstraint*>(joint);
        const auto settings = motor->GetConstraintSettings();
        const auto* swingSettings = static_cast<const JPH::SwingTwistConstraintSettings*>(settings.GetPtr());
        return SwingTwistJointGeometry{ motor->GetConstraintToBody1(), motor->GetConstraintToBody2(),
            motor->GetNormalHalfConeAngle(), motor->GetPlaneHalfConeAngle(), motor->GetTwistMinAngle(),
            motor->GetTwistMaxAngle(), swingSettings->mSwingType == JPH::ESwingType::Pyramid };
    }

    std::optional<HingeLegGeometry> PassiveRagdoll::getHingeLegGeometry(
        std::size_t hip, std::size_t knee, std::size_t foot) const
    {
        if (hip >= getBodyCount() || knee >= getBodyCount() || foot >= getBodyCount()
            || hip == knee || hip == foot || knee == foot)
            return std::nullopt;
        const auto* settings = mRagdoll->GetRagdollSettings();
        const int kneeConstraint = settings->GetConstraintIndexForBodyIndex(static_cast<int>(knee));
        const int footConstraint = settings->GetConstraintIndexForBodyIndex(static_cast<int>(foot));
        if (kneeConstraint < 0 || footConstraint < 0)
            return std::nullopt;
        const auto* joint = mRagdoll->GetConstraint(kneeConstraint);
        const auto* ankle = mRagdoll->GetConstraint(footConstraint);
        if (joint->GetSubType() != JPH::EConstraintSubType::Hinge
            || joint->GetBody1()->GetID() != getBodyId(hip)
            || joint->GetBody2()->GetID() != getBodyId(knee)
            || ankle->GetBody1()->GetID() != getBodyId(knee)
            || ankle->GetBody2()->GetID() != getBodyId(foot))
            return std::nullopt;
        const auto* hinge = static_cast<const JPH::HingeConstraint*>(joint);
        const auto& upper = settings->mParts[hip];
        const auto& lower = settings->mParts[knee];
        const auto& end = settings->mParts[foot];
        HingeLegGeometry geometry;
        const auto inverseUpper = upper.mRotation.Conjugated();
        geometry.mUpperOffset = inverseUpper * JPH::Vec3(lower.mPosition - upper.mPosition) * mInverseUniformScale;
        geometry.mLowerOffset = inverseUpper * JPH::Vec3(end.mPosition - lower.mPosition) * mInverseUniformScale;
        geometry.mLowerRotation = (inverseUpper * lower.mRotation).Normalized();
        // This solver's endpoints are bone origins. Offset joint anchors
        // require different kinematics and must not silently use this model.
        const auto atOrigin = [](const JPH::TwoBodyConstraint* constraint) {
            return (constraint->GetConstraintToBody2Matrix().GetTranslation()
                + constraint->GetBody2()->GetShape()->GetCenterOfMass()).IsNearZero(1e-4f);
        };
        if (!atOrigin(joint) || !atOrigin(ankle))
            return std::nullopt;
        geometry.mHingeAxis = hinge->GetLocalSpaceHingeAxis1();
        const auto otherAxis = geometry.mLowerRotation * hinge->GetLocalSpaceHingeAxis2();
        const auto normal1 = hinge->GetLocalSpaceNormalAxis1();
        const auto normal2 = geometry.mLowerRotation * hinge->GetLocalSpaceNormalAxis2();
        if (geometry.mHingeAxis.Dot(otherAxis) < 0.9999f)
            return std::nullopt;
        const float referenceAngle = std::atan2(geometry.mHingeAxis.Dot(normal1.Cross(normal2)), normal1.Dot(normal2));
        geometry.mMinimumAngle = hinge->GetLimitsMin() - referenceAngle;
        geometry.mMaximumAngle = hinge->GetLimitsMax() - referenceAngle;
        return geometry;
    }

    std::optional<float> PassiveRagdoll::getPoseHingeAngle(
        const RagdollPose& pose, std::size_t bodyIndex) const
    {
        if (bodyIndex >= getBodyCount() || pose.mBodyTransforms.size() != getBodyCount())
            return std::nullopt;
        const auto* settings = mRagdoll->GetRagdollSettings();
        const int index = settings->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        const int parent = settings->mSkeleton->GetJoint(static_cast<int>(bodyIndex)).mParentJointIndex;
        if (index < 0 || parent < 0 || static_cast<std::size_t>(parent) >= pose.mBodyTransforms.size())
            return std::nullopt;
        const auto* joint = mRagdoll->GetConstraint(index);
        if (joint->GetSubType() != JPH::EConstraintSubType::Hinge
            || joint->GetBody1()->GetID() != getBodyId(static_cast<std::size_t>(parent))
            || joint->GetBody2()->GetID() != getBodyId(bodyIndex))
            return std::nullopt;
        const auto& upper = pose.mBodyTransforms[parent];
        const auto& lower = pose.mBodyTransforms[bodyIndex];
        if (!isFinite(upper) || !isFinite(lower))
            return std::nullopt;
        // Keep SkeletonPose::CalculateJointStates' exact conversion. In
        // particular, do not assume bind rotations or knee zero are identity.
        const auto upperMatrix = JPH::Mat44::sRotationTranslation(upper.mRotation, upper.mTranslation);
        const auto lowerMatrix = JPH::Mat44::sRotationTranslation(lower.mRotation, lower.mTranslation);
        const auto relative = (upperMatrix.Inversed() * lowerMatrix).GetQuaternion();
        const float angle = requestedHingeAngle(*static_cast<const JPH::HingeConstraint*>(joint), relative);
        return std::isfinite(angle) ? std::optional<float>(angle) : std::nullopt;
    }

    std::optional<JPH::Vec3> PassiveRagdoll::getWorldVector(JPH::Vec3Arg modelVector) const
    {
        if (!isFinite(modelVector))
            return std::nullopt;
        const JPH::Vec3 result
            = mInverseReferenceRotation.Conjugated() * (modelVector / mInverseUniformScale);
        return isFinite(result) ? std::optional<JPH::Vec3>(result) : std::nullopt;
    }

    std::optional<JPH::Vec3> PassiveRagdoll::getModelVector(JPH::Vec3Arg worldVector) const
    {
        if (!isFinite(worldVector))
            return std::nullopt;
        const JPH::Vec3 result = (mInverseReferenceRotation * worldVector) * mInverseUniformScale;
        return isFinite(result) ? std::optional<JPH::Vec3>(result) : std::nullopt;
    }

    std::optional<JPH::Quat> PassiveRagdoll::getModelRotation(JPH::QuatArg worldRotation) const
    {
        if (!isFinite(worldRotation) || !worldRotation.IsNormalized())
            return std::nullopt;
        return (mInverseReferenceRotation * worldRotation).Normalized();
    }

    std::optional<JPH::Vec3> PassiveRagdoll::getModelPosition(JPH::RVec3Arg worldPosition) const
    {
        if (!isFinite(worldPosition))
            return std::nullopt;
        const JPH::Vec3 result = (mInverseReferenceRotation
            * JPH::Vec3(worldPosition - mReferencePosition)) * mInverseUniformScale;
        if (!isFinite(result))
            return std::nullopt;
        return result;
    }

    std::optional<JPH::Vec3> PassiveRagdoll::rebaseModelPosition(JPH::Vec3Arg modelPosition,
        JPH::RVec3Arg referencePosition, JPH::QuatArg referenceRotation) const
    {
        if (!isFinite(modelPosition) || !isFinite(referencePosition) || !isFinite(referenceRotation))
            return std::nullopt;
        const JPH::Vec3 scaledModelPosition = modelPosition / mInverseUniformScale;
        const JPH::RVec3 worldPosition
            = mReferencePosition + mInverseReferenceRotation.Conjugated() * scaledModelPosition;
        const JPH::Vec3 result
            = (referenceRotation.Conjugated() * JPH::Vec3(worldPosition - referencePosition))
            * mInverseUniformScale;
        return isFinite(result) ? std::optional<JPH::Vec3>(result) : std::nullopt;
    }

    bool PassiveRagdoll::rebaseModelPose(
        RagdollPose& pose, JPH::RVec3Arg referencePosition, JPH::QuatArg referenceRotation) const
    {
        if (!isFinite(referencePosition) || !isFinite(referenceRotation))
            return false;

        const JPH::Quat oldReferenceRotation = mInverseReferenceRotation.Conjugated();
        const JPH::Quat inverseNewReferenceRotation = referenceRotation.Conjugated();
        RagdollPose rebased;
        rebased.mBodyTransforms.reserve(pose.mBodyTransforms.size());
        for (const Transform& transform : pose.mBodyTransforms)
        {
            if (!isFinite(transform))
                return false;
            const JPH::Vec3 scaledModelPosition = transform.mTranslation / mInverseUniformScale;
            const JPH::RVec3 worldPosition
                = mReferencePosition + oldReferenceRotation * scaledModelPosition;
            Transform item;
            item.mTranslation = (inverseNewReferenceRotation
                * JPH::Vec3(worldPosition - referencePosition)) * mInverseUniformScale;
            item.mRotation = inverseNewReferenceRotation
                * (oldReferenceRotation * transform.mRotation);
            if (!isFinite(item))
                return false;
            rebased.mBodyTransforms.push_back(item);
        }
        pose = std::move(rebased);
        return true;
    }

    bool PassiveRagdoll::setReferenceTransform(
        JPH::RVec3Arg referencePosition, JPH::QuatArg referenceRotation)
    {
        if (!isFinite(referencePosition) || !isFinite(referenceRotation))
            return false;
        mReferencePosition = referencePosition;
        mInverseReferenceRotation = referenceRotation.Conjugated();
        return true;
    }

    bool PassiveRagdoll::driveToPoseUsingMotors(
        const RagdollPose& target, std::span<const std::size_t> disabledMotorBodies)
    {
        if (!mAdded || target.mBodyTransforms.size() != getBodyCount())
            return false;

        for (const Transform& transform : target.mBodyTransforms)
        {
            if (!isFinite(transform))
                return false;
        }
        const JPH::RagdollSettings* settings = mRagdoll->GetRagdollSettings();
        for (std::size_t i = 0; i < disabledMotorBodies.size(); ++i)
        {
            const std::size_t bodyIndex = disabledMotorBodies[i];
            if (bodyIndex >= getBodyCount()
                || settings->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex)) < 0)
                return false;
            for (std::size_t j = 0; j < i; ++j)
            {
                if (disabledMotorBodies[j] == bodyIndex)
                    return false;
            }
        }

        std::vector<std::array<JPH::Vec3, 3>> motorAxes;
        motorAxes.reserve(mRagdoll->GetConstraintCount());
        for (std::size_t i = 0; i < mRagdoll->GetConstraintCount(); ++i)
        {
            const auto* constraint = mRagdoll->GetConstraint(static_cast<int>(i));
            std::array<JPH::Vec3, 3> axes{ JPH::Vec3::sZero(), JPH::Vec3::sZero(), JPH::Vec3::sZero() };
            if (constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
            {
                const auto* swingTwist = static_cast<const JPH::SwingTwistConstraint*>(constraint);
                const JPH::Quat frame = constraint->GetBody2()->GetRotation() * swingTwist->GetConstraintToBody2();
                axes = { frame * JPH::Vec3::sAxisX(), frame * JPH::Vec3::sAxisY(), frame * JPH::Vec3::sAxisZ() };
            }
            else if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
            {
                const auto* hinge = static_cast<const JPH::HingeConstraint*>(constraint);
                axes[0] = constraint->GetBody1()->GetRotation() * hinge->GetLocalSpaceHingeAxis1();
            }
            else
                return false;
            motorAxes.push_back(axes);
        }
        mMotorImpulseWorldAxes = std::move(motorAxes);
        mTorqueMotorImpulses.clear();
        auto& matrices = mMotorPose->GetJointMatrices();
        for (std::size_t i = 0; i < target.mBodyTransforms.size(); ++i)
        {
            const Transform& transform = target.mBodyTransforms[i];
            matrices[i] = JPH::Mat44::sRotationTranslation(transform.mRotation, transform.mTranslation);
        }
        mMotorPose->CalculateJointStates();
        std::fill(mRequestedHingeAngles.begin(), mRequestedHingeAngles.end(), std::nullopt);
        for (std::size_t body = 0; body < getBodyCount(); ++body)
        {
            const int index = settings->GetConstraintIndexForBodyIndex(static_cast<int>(body));
            if (index < 0)
                continue;
            const auto* joint = mRagdoll->GetConstraint(index);
            if (joint->GetSubType() != JPH::EConstraintSubType::Hinge)
                continue;
            const float requested = requestedHingeAngle(*static_cast<const JPH::HingeConstraint*>(joint),
                mMotorPose->GetJoint(static_cast<int>(body)).mRotation);
            if (std::isfinite(requested))
                mRequestedHingeAngles[body] = requested;
        }
        // Restore this ragdoll's powered budgets after passive mode. A held-
        // root adapter can keep additional position steps through dynamic
        // falls and reacquisition without changing other locomotion routes.
        for (std::size_t i = 0; i < mRagdoll->GetConstraintCount(); ++i)
        {
            mRagdoll->GetConstraint(static_cast<int>(i))->SetNumPositionStepsOverride(mPoweredPositionSolverSteps);
            mRagdoll->GetConstraint(static_cast<int>(i))->SetNumVelocityStepsOverride(mPoweredVelocitySolverSteps);
        }
        for (std::size_t i = 0; i < mRagdoll->GetConstraintCount(); ++i)
        {
            auto* joint = mRagdoll->GetConstraint(static_cast<int>(i));
            if (joint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
            {
                auto* motor = static_cast<JPH::SwingTwistConstraint*>(joint);
                motor->SetMotorLoadTorque(JPH::Vec3::sZero());
                motor->SetTargetAngularVelocityCS(JPH::Vec3::sZero());
                motor->SetPositionMotorWorldVelocityReference(false);
            }
            else if (joint->GetSubType() == JPH::EConstraintSubType::Hinge)
            {
                auto* motor = static_cast<JPH::HingeConstraint*>(joint);
                motor->SetMotorLoadTorque(JPH::Vec3::sZero());
                motor->SetTargetAngularVelocity(0);
            }
        }
        mRagdoll->DriveToPoseUsingMotors(*mMotorPose);
        for (const std::size_t bodyIndex : disabledMotorBodies)
        {
            const int constraintIndex
                = settings->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
            JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(constraintIndex);
            if (constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
            {
                auto* swingTwist = static_cast<JPH::SwingTwistConstraint*>(constraint);
                swingTwist->SetSwingMotorState(JPH::EMotorState::Off);
                swingTwist->SetTwistMotorState(JPH::EMotorState::Off);
            }
            else if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
                static_cast<JPH::HingeConstraint*>(constraint)->SetMotorState(JPH::EMotorState::Off);
        }
        mRagdoll->Activate();
        return true;
    }

    void PassiveRagdoll::setPoweredVelocitySolverSteps(std::uint32_t steps)
    {
        mPoweredVelocitySolverSteps = steps;
        for (std::size_t i = 0; i < mRagdoll->GetConstraintCount(); ++i)
            mRagdoll->GetConstraint(static_cast<int>(i))->SetNumVelocityStepsOverride(steps);
    }

    void PassiveRagdoll::setPoweredPositionSolverSteps(std::uint32_t steps)
    {
        mPoweredPositionSolverSteps = steps;
        for (std::size_t i = 0; i < mRagdoll->GetConstraintCount(); ++i)
            mRagdoll->GetConstraint(static_cast<int>(i))->SetNumPositionStepsOverride(steps);
    }

    bool PassiveRagdoll::setConstraintPriority(std::size_t bodyIndex, std::uint32_t priority)
    {
        if (bodyIndex >= getBodyCount())
            return false;
        const int index
            = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0)
            return false;
        mRagdoll->GetConstraint(index)->SetConstraintPriority(priority);
        return true;
    }

    bool PassiveRagdoll::setConstraintSolverSteps(
        std::size_t bodyIndex, std::uint32_t velocitySteps, std::uint32_t positionSteps)
    {
        if (bodyIndex >= getBodyCount() || velocitySteps >= 256 || positionSteps >= 256)
            return false;
        const int index
            = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0)
            return false;
        auto* constraint = mRagdoll->GetConstraint(index);
        constraint->SetNumVelocityStepsOverride(velocitySteps);
        constraint->SetNumPositionStepsOverride(positionSteps);
        return true;
    }

    bool PassiveRagdoll::getConstraintImpulse(std::size_t bodyIndex, ConstraintImpulse& out) const
    {
        if (!mAdded || bodyIndex >= getBodyCount())
            return false;
        const int index
            = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0)
            return false;
        const auto* constraint = mRagdoll->GetConstraint(index);
        if (!constraint->GetEnabled() || constraint->GetSubType() != JPH::EConstraintSubType::SwingTwist)
            return false;
        const auto* swingTwist = static_cast<const JPH::SwingTwistConstraint*>(constraint);
        const float linear = swingTwist->GetTotalLambdaPosition().Length();
        const float angular = JPH::Vec3(swingTwist->GetTotalLambdaTwist(),
            swingTwist->GetTotalLambdaSwingY(), swingTwist->GetTotalLambdaSwingZ()).Length();
        if (!std::isfinite(linear) || !std::isfinite(angular))
            return false;
        out = ConstraintImpulse{ linear, angular };
        return true;
    }

    std::optional<bool> PassiveRagdoll::releaseConstraint(std::size_t bodyIndex)
    {
        if (bodyIndex >= getBodyCount())
            return std::nullopt;
        const int index
            = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0)
            return std::nullopt;
        auto* constraint = mRagdoll->GetConstraint(index);
        if (!constraint->GetEnabled())
            return false;
        constraint->SetEnabled(false);
        mRagdoll->Activate();
        return true;
    }

    std::optional<bool> PassiveRagdoll::restoreConstraint(std::size_t bodyIndex)
    {
        if (bodyIndex >= getBodyCount())
            return std::nullopt;
        const int index
            = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0)
            return std::nullopt;
        auto* constraint = mRagdoll->GetConstraint(index);
        if (constraint->GetEnabled())
            return false;
        constraint->SetEnabled(true);
        mRagdoll->Activate();
        return true;
    }

    std::optional<bool> PassiveRagdoll::isConstraintEnabled(std::size_t bodyIndex) const
    {
        if (bodyIndex >= getBodyCount())
            return std::nullopt;
        const int index
            = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0)
            return std::nullopt;
        return mRagdoll->GetConstraint(index)->GetEnabled();
    }

    bool PassiveRagdoll::diagnosticOverrideMotorPose(std::size_t bodyIndex,
        JPH::QuatArg relative, float hingeAngle, const std::optional<JPH::Vec3>& velocity)
    {
        if (!mAdded || bodyIndex >= getBodyCount() || !isFinite(relative)
            || std::abs(relative.LengthSq() - 1) > 1e-4f || !std::isfinite(hingeAngle) || (velocity && !isFinite(*velocity))) return false;
        const int index = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0) return false;
        auto* constraint = mRagdoll->GetConstraint(index);
        if (constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
        {
            auto* joint = static_cast<JPH::SwingTwistConstraint*>(constraint);
            if (joint->GetSwingMotorState() != JPH::EMotorState::Position
                || joint->GetTwistMotorState() != JPH::EMotorState::Position) return false;
            joint->SetTargetOrientationCS((joint->GetConstraintToBody1().Conjugated()
                * relative * joint->GetConstraintToBody2()).Normalized());
            if (velocity)
                joint->SetTargetAngularVelocityCS((joint->GetBody2()->GetRotation()
                    * joint->GetConstraintToBody2()).Conjugated() * *velocity);
            mMotorPose->GetJoint(static_cast<int>(bodyIndex)).mRotation = relative;
            for (auto& entry : mTorqueMotorImpulses)
                if (entry.mBodyIndex == bodyIndex && entry.mWorldTarget)
                {
                    entry.mRequestedWorldTarget = (joint->GetBody1()->GetRotation() * relative).Normalized();
                    entry.mWorldTarget = (joint->GetBody1()->GetRotation() * joint->GetConstraintToBody1()
                        * joint->GetTargetOrientationCS() * joint->GetConstraintToBody2().Conjugated()).Normalized();
                }
            return true;
        }
        if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
        {
            auto* joint = static_cast<JPH::HingeConstraint*>(constraint);
            if (joint->GetMotorState() != JPH::EMotorState::Position) return false;
            joint->SetTargetAngle(std::clamp(hingeAngle, joint->GetLimitsMin(), joint->GetLimitsMax()));
            mRequestedHingeAngles[bodyIndex] = hingeAngle;
            if (velocity) joint->SetTargetAngularVelocity(velocity->Dot(
                joint->GetBody1()->GetRotation() * joint->GetLocalSpaceHingeAxis1()));
            return true;
        }
        return false;
    }

    bool PassiveRagdoll::limitMotorTargetStep(std::size_t bodyIndex, float maximumAngularStep)
    {
        if (!mAdded || bodyIndex >= getBodyCount() || !std::isfinite(maximumAngularStep)
            || maximumAngularStep <= 0.0f)
            return false;
        const JPH::RagdollSettings* settings = mRagdoll->GetRagdollSettings();
        const int constraintIndex = settings->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (constraintIndex < 0)
            return false;

        JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(constraintIndex);
        if (constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
        {
            auto* swingTwist = static_cast<JPH::SwingTwistConstraint*>(constraint);
            if (swingTwist->GetSwingMotorState() != JPH::EMotorState::Position
                || swingTwist->GetTwistMotorState() != JPH::EMotorState::Position)
                return false;
            const JPH::Quat current = swingTwist->GetRotationInConstraintSpace();
            const JPH::Quat target = swingTwist->GetTargetOrientationCS();
            const float cosine = std::clamp(std::abs(current.Dot(target)), 0.0f, 1.0f);
            const float angle = 2.0f * std::acos(cosine);
            if (angle > maximumAngularStep)
            {
                swingTwist->SetTargetOrientationCS(current.SLERP(target, maximumAngularStep / angle));
                const auto recorded = std::find_if(mTorqueMotorImpulses.begin(), mTorqueMotorImpulses.end(),
                    [bodyIndex](const auto& entry) { return entry.mBodyIndex == bodyIndex && entry.mWorldTarget; });
                if (recorded != mTorqueMotorImpulses.end())
                {
                    if (!recorded->mRequestedWorldTarget)
                        recorded->mRequestedWorldTarget = recorded->mWorldTarget;
                    const auto parentFrame = swingTwist->GetBody1()->GetRotation() * swingTwist->GetConstraintToBody1();
                    recorded->mWorldTarget = (parentFrame * swingTwist->GetTargetOrientationCS()
                        * swingTwist->GetConstraintToBody2().Conjugated()).Normalized();
                }
            }
            return true;
        }
        if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
        {
            auto* hinge = static_cast<JPH::HingeConstraint*>(constraint);
            if (hinge->GetMotorState() != JPH::EMotorState::Position)
                return false;
            const float current = hinge->GetCurrentAngle();
            const float target = hinge->GetTargetAngle();
            hinge->SetTargetAngle(current
                + std::clamp(target - current, -maximumAngularStep, maximumAngularStep));
            return true;
        }
        return false;
    }

    bool PassiveRagdoll::setSwingTwistJointLimits(
        std::size_t bodyIndex, float normalHalfCone, float planeHalfCone, float twistMin, float twistMax)
    {
        if (!mAdded || bodyIndex >= getBodyCount() || !std::isfinite(normalHalfCone) || normalHalfCone < 0.0f
            || !std::isfinite(planeHalfCone) || planeHalfCone < 0.0f || !std::isfinite(twistMin)
            || !std::isfinite(twistMax) || twistMin > twistMax)
            return false;
        const JPH::RagdollSettings* settings = mRagdoll->GetRagdollSettings();
        const int constraintIndex = settings->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (constraintIndex < 0)
            return false;
        JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(constraintIndex);
        if (constraint->GetSubType() != JPH::EConstraintSubType::SwingTwist)
            return false;
        auto* swingTwist = static_cast<JPH::SwingTwistConstraint*>(constraint);
        swingTwist->SetNormalHalfConeAngle(normalHalfCone);
        swingTwist->SetPlaneHalfConeAngle(planeHalfCone);
        swingTwist->SetTwistMinAngle(twistMin);
        swingTwist->SetTwistMaxAngle(twistMax);
        return true;
    }

    bool PassiveRagdoll::setHingeJointLimits(std::size_t bodyIndex, float minimum, float maximum)
    {
        if (!mAdded || bodyIndex >= getBodyCount() || !std::isfinite(minimum) || !std::isfinite(maximum)
            || minimum > maximum)
            return false;
        const JPH::RagdollSettings* settings = mRagdoll->GetRagdollSettings();
        const int constraintIndex = settings->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (constraintIndex < 0)
            return false;
        JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(constraintIndex);
        if (constraint->GetSubType() != JPH::EConstraintSubType::Hinge)
            return false;
        static_cast<JPH::HingeConstraint*>(constraint)->SetLimits(minimum, maximum);
        return true;
    }

    std::optional<std::pair<float, float>> PassiveRagdoll::getHingeJointLimits(std::size_t bodyIndex) const
    {
        if (!mAdded || bodyIndex >= getBodyCount())
            return std::nullopt;
        const JPH::RagdollSettings* settings = mRagdoll->GetRagdollSettings();
        const int constraintIndex = settings->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (constraintIndex < 0)
            return std::nullopt;
        const JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(constraintIndex);
        if (constraint->GetSubType() != JPH::EConstraintSubType::Hinge)
            return std::nullopt;
        const auto* hinge = static_cast<const JPH::HingeConstraint*>(constraint);
        return std::pair<float, float>{ hinge->GetLimitsMin(), hinge->GetLimitsMax() };
    }

    std::optional<PassiveRagdoll::MotorDebug> PassiveRagdoll::getMotorDebug(std::size_t bodyIndex) const
    {
        if (!mAdded || bodyIndex >= getBodyCount())
            return std::nullopt;
        const int constraintIndex
            = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (constraintIndex < 0)
            return std::nullopt;
        const JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(constraintIndex);
        MotorDebug result;
        const auto fill = [&](const JPH::MotorSettings& motor) {
            result.mMaximumTorque = motor.mMaxTorqueLimit;
            result.mFrequency = motor.mSpringSettings.mFrequency;
            result.mDamping = motor.mSpringSettings.mDamping;
        };
        if (constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
        {
            const auto* joint = static_cast<const JPH::SwingTwistConstraint*>(constraint);
            fill(joint->GetSwingMotorSettings());
            result.mState = static_cast<int>(joint->GetSwingMotorState());
            const JPH::Quat current = ((constraint->GetBody1()->GetRotation() * joint->GetConstraintToBody1()).Conjugated()
                * (constraint->GetBody2()->GetRotation() * joint->GetConstraintToBody2())).Normalized();
            const float dot = std::abs(current.Dot(joint->GetTargetOrientationCS()));
            result.mTargetError = 2.0f * std::acos(std::clamp(dot, 0.0f, 1.0f));
            return result;
        }
        if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
        {
            const auto* joint = static_cast<const JPH::HingeConstraint*>(constraint);
            fill(joint->GetMotorSettings());
            result.mState = static_cast<int>(joint->GetMotorState());
            result.mTargetError = std::abs(joint->GetCurrentAngle() - joint->GetTargetAngle());
            return result;
        }
        return std::nullopt;
    }

    bool PassiveRagdoll::setMotorMaximumTorque(std::size_t bodyIndex, float maximumTorque)
    {
        if (!mAdded || bodyIndex >= getBodyCount() || !std::isfinite(maximumTorque) || maximumTorque < 0.0f)
            return false;
        const JPH::RagdollSettings* settings = mRagdoll->GetRagdollSettings();
        const int constraintIndex = settings->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (constraintIndex < 0)
            return false;

        JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(constraintIndex);
        const auto setLimit = [maximumTorque](JPH::MotorSettings& motor) {
            motor.mMinTorqueLimit = -maximumTorque;
            motor.mMaxTorqueLimit = maximumTorque;
        };
        if (constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
        {
            auto* swingTwist = static_cast<JPH::SwingTwistConstraint*>(constraint);
            setLimit(swingTwist->GetSwingMotorSettings());
            setLimit(swingTwist->GetTwistMotorSettings());
            return true;
        }
        if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
        {
            setLimit(static_cast<JPH::HingeConstraint*>(constraint)->GetMotorSettings());
            return true;
        }
        return false;
    }

    bool PassiveRagdoll::setMotorTorqueLimits(
        std::size_t bodyIndex, JPH::Vec3Arg minimum, JPH::Vec3Arg maximum)
    {
        if (!mAdded || bodyIndex >= getBodyCount() || !isFinite(minimum) || !isFinite(maximum))
            return false;
        for (unsigned axis = 0; axis < 3; ++axis)
            if (minimum[axis] > 0 || maximum[axis] < 0 || minimum[axis] > maximum[axis])
                return false;
        const int index = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0)
            return false;
        auto* constraint = mRagdoll->GetConstraint(index);
        const auto set = [&](JPH::MotorSettings& motor, unsigned axis) {
            motor.mMinTorqueLimit = minimum[axis];
            motor.mMaxTorqueLimit = maximum[axis];
        };
        if (constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
        {
            if (minimum.GetY() != minimum.GetZ() || maximum.GetY() != maximum.GetZ())
                return false;
            auto* joint = static_cast<JPH::SwingTwistConstraint*>(constraint);
            set(joint->GetTwistMotorSettings(), 0);
            set(joint->GetSwingMotorSettings(), 1);
            return true;
        }
        if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
        {
            if (minimum.GetY() != 0 || minimum.GetZ() != 0 || maximum.GetY() != 0 || maximum.GetZ() != 0)
                return false;
            set(static_cast<JPH::HingeConstraint*>(constraint)->GetMotorSettings(), 0);
            return true;
        }
        return false;
    }

    bool PassiveRagdoll::setMotorSpring(std::size_t bodyIndex, float frequency, float damping)
    {
        if (!mAdded || bodyIndex >= getBodyCount() || !std::isfinite(frequency) || frequency <= 0.0f
            || !std::isfinite(damping) || damping < 0.0f)
            return false;
        const JPH::RagdollSettings* settings = mRagdoll->GetRagdollSettings();
        const int constraintIndex = settings->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (constraintIndex < 0)
            return false;

        JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(constraintIndex);
        const auto setSpring = [frequency, damping](JPH::MotorSettings& motor) {
            motor.mSpringSettings.mMode = JPH::ESpringMode::FrequencyAndDamping;
            motor.mSpringSettings.mFrequency = frequency;
            motor.mSpringSettings.mDamping = damping;
        };
        if (constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
        {
            auto* swingTwist = static_cast<JPH::SwingTwistConstraint*>(constraint);
            setSpring(swingTwist->GetSwingMotorSettings());
            setSpring(swingTwist->GetTwistMotorSettings());
            return true;
        }
        if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
        {
            setSpring(static_cast<JPH::HingeConstraint*>(constraint)->GetMotorSettings());
            return true;
        }
        return false;
    }

    bool PassiveRagdoll::driveMotorWithFreeRotationAxis(
        std::size_t bodyIndex, JPH::Vec3Arg freeWorldAxis, float fixedTimeStep)
    {
        return driveMotorWithFreeAxisServo(bodyIndex, fixedTimeStep, freeWorldAxis);
    }

    bool PassiveRagdoll::driveMotorToWorldOrientation(
        std::size_t bodyIndex, JPH::QuatArg targetWorldRotation, float fixedTimeStep)
    {
        return driveMotorWithLoadTorque(bodyIndex, JPH::Vec3::sZero(), fixedTimeStep, std::nullopt, targetWorldRotation);
    }

    bool PassiveRagdoll::driveMotorWithFreeAxisServo(std::size_t bodyIndex, float fixedTimeStep,
        JPH::Vec3Arg freeWorldAxis, const std::optional<JPH::Vec3>& loadTorque)
    {
        if (!mAdded || bodyIndex >= getBodyCount()
            || !isFinite(freeWorldAxis) || !freeWorldAxis.IsNormalized()
            || (loadTorque && !isFinite(*loadTorque))
            || !std::isfinite(fixedTimeStep) || fixedTimeStep <= 0.0f
            || std::any_of(mTorqueMotorImpulses.begin(), mTorqueMotorImpulses.end(),
                [bodyIndex](const auto& entry) { return entry.mBodyIndex == bodyIndex; }))
            return false;
        const int index = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0)
            return false;
        JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(index);
        if (constraint->GetSubType() != JPH::EConstraintSubType::SwingTwist)
            return false;
        auto* joint = static_cast<JPH::SwingTwistConstraint*>(constraint);
        if (joint->GetSwingMotorState() != JPH::EMotorState::Position
            || joint->GetTwistMotorState() != JPH::EMotorState::Position || joint->GetMaxFrictionTorque() != 0.0f)
            return false;
        const auto& swing = joint->GetSwingMotorSettings();
        const auto& twist = joint->GetTwistMotorSettings();
        const auto& spring = swing.mSpringSettings;
        if (spring.mMode != JPH::ESpringMode::FrequencyAndDamping
            || twist.mSpringSettings.mMode != spring.mMode
            || twist.mSpringSettings.mFrequency != spring.mFrequency
            || twist.mSpringSettings.mDamping != spring.mDamping
            || !std::isfinite(spring.mFrequency) || spring.mFrequency <= 0.0f
            || !std::isfinite(spring.mDamping) || spring.mDamping < 0.0f)
            return false;
        const float maximumTorque = std::min({ swing.mMaxTorqueLimit, -swing.mMinTorqueLimit,
            twist.mMaxTorqueLimit, -twist.mMinTorqueLimit });
        if (!std::isfinite(maximumTorque) || maximumTorque < 0.0f)
            return false;

        const JPH::Body& parent = *joint->GetBody1();
        const JPH::Body& child = *joint->GetBody2();
        const JPH::Quat constraintToWorld = child.GetRotation() * joint->GetConstraintToBody2();
        const JPH::Quat current = joint->GetRotationInConstraintSpace();
        JPH::Quat target = joint->GetTargetOrientationCS();
        if (current.Dot(target) < 0.0f)
            target = -target;
        const JPH::Vec3 error = constraintToWorld * (-2.0f * (current.Conjugated() * target).GetXYZ());
        const JPH::Vec3 velocity = child.GetAngularVelocity() - parent.GetAngularVelocity();
        // A world basis for the controlled plane avoids giving the released
        // direction torque through an anisotropic inertia tensor.
        const JPH::Vec3 firstAxis = freeWorldAxis.GetNormalizedPerpendicular();
        const std::array<JPH::Vec3, 2> axes{ firstAxis, freeWorldAxis.Cross(firstAxis) };
        const float omega = 2.0f * std::numbers::pi_v<float> * spring.mFrequency;
        const float stiffness = omega * omega;
        const float damping = 2.0f * spring.mDamping * omega;
        const float denominator = 1.0f + fixedTimeStep * damping + fixedTimeStep * fixedTimeStep * stiffness;
        JPH::Vec3 torque = JPH::Vec3::sZero();
        for (std::size_t axisIndex = 0; axisIndex < axes.size(); ++axisIndex)
        {
            const JPH::Vec3 axis = axes[axisIndex];
            const auto inverseInertia = [&](const JPH::Body& body) {
                return body.IsDynamic()
                    ? body.GetMotionProperties()->MultiplyWorldSpaceInverseInertiaByVector(body.GetRotation(), axis)
                    : JPH::Vec3::sZero();
            };
            const float inverseEffectiveInertia = axis.Dot(inverseInertia(child) + inverseInertia(parent));
            if (!std::isfinite(inverseEffectiveInertia) || inverseEffectiveInertia <= 0.0f)
                return false;
            // Implicit spring integration includes the step's numerical
            // damping, using the same frequency/damping convention as Jolt.
            const float axisTorque = -(stiffness * error.Dot(axis)
                + (damping + fixedTimeStep * stiffness) * velocity.Dot(axis))
                / (inverseEffectiveInertia * denominator);
            torque += axis * axisTorque;
        }
        if (loadTorque)
            torque += *loadTorque;
        const float magnitude = torque.Length();
        if (!isFinite(torque) || !std::isfinite(magnitude))
            return false;
        float utilization = 0;
        if (loadTorque)
        {
            // Pose release removes stiffness/damping, not the torque carrying
            // gravity or planned contact loads. Clip the sum once. Uniform
            // scaling retains its direction even when the released world axis
            // is oblique to asymmetric native motor limits.
            float scale = 1;
            const std::array<JPH::Vec3, 3> nativeAxes{ constraintToWorld * JPH::Vec3::sAxisX(),
                constraintToWorld * JPH::Vec3::sAxisY(), constraintToWorld * JPH::Vec3::sAxisZ() };
            for (std::size_t i = 0; i < nativeAxes.size(); ++i)
            {
                const auto& motor = i == 0 ? twist : swing;
                if (!std::isfinite(motor.mMinTorqueLimit) || motor.mMinTorqueLimit > 0
                    || !std::isfinite(motor.mMaxTorqueLimit) || motor.mMaxTorqueLimit < 0)
                    return false;
                const float requested = torque.Dot(nativeAxes[i]);
                const float capacity = requested >= 0 ? motor.mMaxTorqueLimit : -motor.mMinTorqueLimit;
                if (requested != 0)
                {
                    scale = std::min(scale, capacity / std::abs(requested));
                    utilization = std::max(utilization,
                        capacity > 0 ? std::min(std::abs(requested) / capacity, 1.0f) : 1.0f);
                }
            }
            torque *= scale;
        }
        else
        {
            if (magnitude > maximumTorque)
                torque *= maximumTorque / magnitude;
            utilization = maximumTorque > 0.0f ? std::min(magnitude / maximumTorque, 1.0f)
                : magnitude > 0.0f ? 1.0f : 0.0f;
        }
        const bool useSolver = mUseSolverWrenches || loadTorque.has_value();
        const int parentIndex = mRagdoll->GetRagdollSettings()->GetSkeleton()
            ->GetJoint(static_cast<int>(bodyIndex)).mParentJointIndex;
        if (useSolver && (parentIndex < 0 || static_cast<std::size_t>(parentIndex) >= getBodyCount()
                || getBodyId(static_cast<std::size_t>(parentIndex)) != parent.GetID()))
            return false;
        joint->SetSwingMotorState(JPH::EMotorState::Off);
        joint->SetTwistMotorState(JPH::EMotorState::Off);
        joint->SetTargetAngularVelocityCS(JPH::Vec3::sZero());
        if (useSolver)
        {
            applySolverWrench(bodyIndex, static_cast<std::size_t>(parentIndex), JPH::Vec3::sZero(), torque);
        }
        else
        {
            mPhysicsSystem->GetBodyInterface().AddTorque(child.GetID(), torque, JPH::EActivation::Activate);
            mPhysicsSystem->GetBodyInterface().AddTorque(parent.GetID(), -torque, JPH::EActivation::Activate);
        }
        mTorqueMotorImpulses.push_back({ bodyIndex, torque * fixedTimeStep, fixedTimeStep, utilization,
            std::nullopt, loadTorque.has_value(), false });
        return true;
    }

    bool PassiveRagdoll::driveMotorWithLoadTorque(
        std::size_t bodyIndex, JPH::Vec3Arg loadTorque, float fixedTimeStep,
        const std::optional<JPH::Vec3>& freeWorldAxis, const std::optional<JPH::Quat>& targetWorldRotation,
        const std::optional<JPH::Vec3>& targetWorldAngularVelocity)
    {
        if (freeWorldAxis)
            return !targetWorldRotation && !targetWorldAngularVelocity
                && driveMotorWithFreeAxisServo(bodyIndex, fixedTimeStep, *freeWorldAxis, loadTorque);
        MotorTargetTelemetry target;
        if (!mAdded || !isFinite(loadTorque) || !std::isfinite(fixedTimeStep) || fixedTimeStep <= 0
            || (targetWorldRotation && (!isFinite(*targetWorldRotation) || !targetWorldRotation->IsNormalized()))
            || (targetWorldAngularVelocity && !isFinite(*targetWorldAngularVelocity))
            || !captureMotorTargetTelemetry(bodyIndex, target)
            || std::any_of(mTorqueMotorImpulses.begin(), mTorqueMotorImpulses.end(),
                [bodyIndex](const auto& entry) { return entry.mBodyIndex == bodyIndex; }))
            return false;
        const int index = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0)
            return false;
        auto* joint = mRagdoll->GetConstraint(index);
        if (!joint->GetEnabled())
            return false;
        const auto validSpring = [](const JPH::MotorSettings& motor) {
            const auto& spring = motor.mSpringSettings;
            return std::isfinite(motor.mMinTorqueLimit) && motor.mMinTorqueLimit <= 0
                && std::isfinite(motor.mMaxTorqueLimit) && motor.mMaxTorqueLimit >= 0
                && std::isfinite(spring.mDamping) && spring.mDamping >= 0
                && (spring.mMode == JPH::ESpringMode::FrequencyAndDamping
                    ? std::isfinite(spring.mFrequency) && spring.mFrequency > 0
                    : std::isfinite(spring.mStiffness) && spring.mStiffness > 0);
        };
        std::optional<JPH::Quat> worldTarget;
        std::optional<JPH::Quat> requestedWorldTarget;
        if (joint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
        {
            auto* motor = static_cast<JPH::SwingTwistConstraint*>(joint);
            if (!validSpring(motor->GetSwingMotorSettings()) || !validSpring(motor->GetTwistMotorSettings()))
                return false;
            if (targetWorldRotation)
            {
                const auto parentFrame = motor->GetBody1()->GetRotation() * motor->GetConstraintToBody1();
                const auto requested = (parentFrame.Conjugated() * *targetWorldRotation
                    * motor->GetConstraintToBody2()).Normalized();
                const auto clamped = motor->GetClampedTargetOrientationCS(requested);
                if (!isFinite(motor->GetBody1()->GetAngularVelocity())
                    || !isFinite(motor->GetBody2()->GetAngularVelocity()))
                    return false;
                worldTarget = targetWorldRotation;
                if (clamped != requested)
                {
                    requestedWorldTarget = targetWorldRotation;
                    worldTarget = (parentFrame * clamped * motor->GetConstraintToBody2().Conjugated()).Normalized();
                }
                motor->SetTargetOrientationCS(clamped);
                motor->SetTargetAngularVelocityCS(JPH::Vec3::sZero());
            }
            motor->SetPositionMotorWorldVelocityReference(targetWorldRotation.has_value());
            if (targetWorldAngularVelocity)
            {
                // With a world target the damping observes the child's world
                // rate, so this is the child's absolute rate; otherwise it is
                // the child-minus-parent relative rate, as in
                // setMotorTargetAngularVelocity. Both are expressed in the
                // child's constraint space.
                const auto frame = motor->GetBody2()->GetRotation() * motor->GetConstraintToBody2();
                const auto velocity = frame.Conjugated() * *targetWorldAngularVelocity;
                if (!isFinite(velocity))
                    return false;
                motor->SetTargetAngularVelocityCS(velocity);
            }
            motor->SetMotorLoadTorque(loadTorque);
        }
        else if (joint->GetSubType() == JPH::EConstraintSubType::Hinge)
        {
            auto* motor = static_cast<JPH::HingeConstraint*>(joint);
            if (targetWorldRotation || !validSpring(motor->GetMotorSettings()))
                return false;
            if (targetWorldAngularVelocity)
            {
                const auto axis = motor->GetBody1()->GetRotation() * motor->GetLocalSpaceHingeAxis1();
                const float velocity = targetWorldAngularVelocity->Dot(axis);
                if (!std::isfinite(velocity))
                    return false;
                motor->SetTargetAngularVelocity(velocity);
            }
            motor->SetMotorLoadTorque(loadTorque);
        }
        else
            return false;
        mTorqueMotorImpulses.push_back({ bodyIndex, JPH::Vec3::sZero(), fixedTimeStep, 0,
            worldTarget, !worldTarget.has_value(), true, requestedWorldTarget, loadTorque });
        return true;
    }

    bool PassiveRagdoll::addMotorLoadTorque(std::size_t bodyIndex, JPH::Vec3Arg extra)
    {
        if (!mAdded || !isFinite(extra))
            return false;
        const auto found = std::find_if(mTorqueMotorImpulses.begin(), mTorqueMotorImpulses.end(),
            [bodyIndex](const auto& entry) { return entry.mBodyIndex == bodyIndex && entry.mNativeLoad; });
        if (found == mTorqueMotorImpulses.end())
            return false;
        const int index = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0)
            return false;
        auto* joint = mRagdoll->GetConstraint(index);
        const JPH::Vec3 total = found->mLoadTorque + extra;
        if (joint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
            static_cast<JPH::SwingTwistConstraint*>(joint)->SetMotorLoadTorque(total);
        else if (joint->GetSubType() == JPH::EConstraintSubType::Hinge)
            static_cast<JPH::HingeConstraint*>(joint)->SetMotorLoadTorque(total);
        else
            return false;
        found->mLoadTorque = total;
        return true;
    }
    bool PassiveRagdoll::setMotorTargetAngularVelocity(std::size_t bodyIndex, JPH::Vec3Arg relativeWorldVelocity)
    {
        if (!mAdded || bodyIndex >= getBodyCount() || !isFinite(relativeWorldVelocity)) return false;
        const int index = mRagdoll->GetRagdollSettings()->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (index < 0) return false;
        auto* joint = mRagdoll->GetConstraint(index);
        if (!joint->GetEnabled()) return false;
        if (joint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
        {
            auto* motor = static_cast<JPH::SwingTwistConstraint*>(joint);
            if (motor->GetSwingMotorState() != JPH::EMotorState::Position
                || motor->GetTwistMotorState() != JPH::EMotorState::Position
                || motor->GetPositionMotorWorldVelocityReference()) return false;
            const auto frame = motor->GetBody2()->GetRotation() * motor->GetConstraintToBody2();
            const auto velocity = frame.Conjugated() * relativeWorldVelocity;
            if (!isFinite(velocity)) return false;
            motor->SetTargetAngularVelocityCS(velocity);
            return true;
        }
        if (joint->GetSubType() == JPH::EConstraintSubType::Hinge)
        {
            auto* motor = static_cast<JPH::HingeConstraint*>(joint);
            if (motor->GetMotorState() != JPH::EMotorState::Position) return false;
            const auto axis = motor->GetBody1()->GetRotation() * motor->GetLocalSpaceHingeAxis1();
            const float velocity = relativeWorldVelocity.Dot(axis);
            if (!std::isfinite(velocity)) return false;
            motor->SetTargetAngularVelocity(velocity);
            return true;
        }
        return false;
    }

    bool PassiveRagdoll::captureMotorTargetTelemetry(
        std::size_t bodyIndex, MotorTargetTelemetry& out) const
    {
        if (!mAdded || bodyIndex >= getBodyCount())
            return false;
        const JPH::RagdollSettings* settings = mRagdoll->GetRagdollSettings();
        const int constraintIndex = settings->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (constraintIndex < 0)
            return false;

        const JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(constraintIndex);
        MotorTargetTelemetry telemetry;
        const auto angularDistance = [](JPH::QuatArg first, JPH::QuatArg second) {
            // Dot/acos magnifies quaternion norm roundoff near zero and can
            // hide actual small rotations. Use the shortest relative rotation.
            const auto relative = (first * second.Conjugated()).Normalized();
            return 2.0f * std::atan2(relative.GetXYZ().Length(), std::abs(relative.GetW()));
        };
        const auto captureSpring = [](const JPH::MotorSettings& motor) {
            const auto& spring = motor.mSpringSettings;
            return MotorSpringTelemetry{ spring.mMode == JPH::ESpringMode::StiffnessAndDamping,
                spring.mFrequency, spring.mDamping };
        };
        if (constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
        {
            const auto* joint = static_cast<const JPH::SwingTwistConstraint*>(constraint);
            telemetry.mSpring = captureSpring(joint->GetSwingMotorSettings());
            telemetry.mTwistSpring = captureSpring(joint->GetTwistMotorSettings());
            const auto& swing = joint->GetSwingMotorSettings();
            const auto& twist = joint->GetTwistMotorSettings();
            telemetry.mDrive = MotorDriveTelemetry{ static_cast<int>(joint->GetSwingMotorState()),
                static_cast<int>(joint->GetTwistMotorState()),
                JPH::Vec3(twist.mMinTorqueLimit, swing.mMinTorqueLimit, swing.mMinTorqueLimit),
                JPH::Vec3(twist.mMaxTorqueLimit, swing.mMaxTorqueLimit, swing.mMaxTorqueLimit),
                telemetry.mSpring, *telemetry.mTwistSpring };
            telemetry.mTargetAngularVelocity = (joint->GetBody2()->GetRotation() * joint->GetConstraintToBody2())
                * joint->GetTargetAngularVelocityCS();
        }
        else if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
        {
            telemetry.mConstraintType = ConstraintType::Hinge;
            telemetry.mSpring = captureSpring(static_cast<const JPH::HingeConstraint*>(constraint)->GetMotorSettings());
            const auto* joint = static_cast<const JPH::HingeConstraint*>(constraint);
            const auto& motor = joint->GetMotorSettings();
            telemetry.mDrive = MotorDriveTelemetry{ static_cast<int>(joint->GetMotorState()), 0,
                JPH::Vec3(motor.mMinTorqueLimit, 0, 0), JPH::Vec3(motor.mMaxTorqueLimit, 0, 0),
                telemetry.mSpring, {} };
            telemetry.mTargetAngularVelocity = (joint->GetBody1()->GetRotation() * joint->GetLocalSpaceHingeAxis1())
                * joint->GetTargetAngularVelocity();
        }
        else
            return false;
        out.mDrive = telemetry.mDrive;
        if (!isFinite(telemetry.mTargetAngularVelocity)) return false;
        const auto replacement = std::find_if(mTorqueMotorImpulses.begin(), mTorqueMotorImpulses.end(),
            [bodyIndex](const auto& entry) { return entry.mBodyIndex == bodyIndex; });
        if (replacement != mTorqueMotorImpulses.end() && replacement->mWorldTarget)
        {
            const JPH::Quat current = constraint->GetBody2()->GetRotation();
            telemetry.mWorldOrientation = true;
            telemetry.mCurrentOrientation = current;
            telemetry.mRequestedOrientation = replacement->mRequestedWorldTarget.value_or(*replacement->mWorldTarget);
            telemetry.mTargetOrientation = replacement->mWorldTarget;
            telemetry.mChildWorldTarget = replacement->mWorldTarget;
            if (replacement->mRequestedWorldTarget)
                telemetry.mTargetClampDelta = angularDistance(
                    *replacement->mRequestedWorldTarget, *replacement->mWorldTarget);
            telemetry.mAngularError = angularDistance(current, *replacement->mWorldTarget);
            out = std::move(telemetry);
            return true;
        }
        if (constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
        {
            const auto* swingTwist = static_cast<const JPH::SwingTwistConstraint*>(constraint);
            if ((swingTwist->GetSwingMotorState() != JPH::EMotorState::Position
                    || swingTwist->GetTwistMotorState() != JPH::EMotorState::Position)
                && (replacement == mTorqueMotorImpulses.end() || !replacement->mJointSpaceTarget))
                return false;
            const JPH::Quat current = swingTwist->GetRotationInConstraintSpace();
            const JPH::Quat requested = swingTwist->GetConstraintToBody1().Conjugated()
                * mMotorPose->GetJoint(static_cast<int>(bodyIndex)).mRotation
                * swingTwist->GetConstraintToBody2();
            const JPH::Quat target = swingTwist->GetTargetOrientationCS();
            if (!isFinite(current) || !isFinite(requested) || !isFinite(target))
                return false;
            telemetry.mConstraintType = ConstraintType::SwingTwist;
            telemetry.mCurrentOrientation = current;
            telemetry.mRequestedOrientation = requested;
            telemetry.mTargetOrientation = target;
            const JPH::Quat childRelative = (swingTwist->GetConstraintToBody1()
                * target * swingTwist->GetConstraintToBody2().Conjugated()).Normalized();
            telemetry.mChildWorldTarget = (swingTwist->GetBody1()->GetRotation() * childRelative).Normalized();
            telemetry.mParentWorldTarget = (swingTwist->GetBody2()->GetRotation() * childRelative.Conjugated()).Normalized();
            telemetry.mNormalHalfConeAngle = swingTwist->GetNormalHalfConeAngle();
            telemetry.mPlaneHalfConeAngle = swingTwist->GetPlaneHalfConeAngle();
            telemetry.mTwistMinimum = swingTwist->GetTwistMinAngle();
            telemetry.mTwistMaximum = swingTwist->GetTwistMaxAngle();
            telemetry.mAngularError = angularDistance(current, target);
            telemetry.mTargetClampDelta = angularDistance(requested, target);
        }
        else if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
        {
            const auto* hinge = static_cast<const JPH::HingeConstraint*>(constraint);
            if (hinge->GetMotorState() != JPH::EMotorState::Position
                && (replacement == mTorqueMotorImpulses.end() || !replacement->mJointSpaceTarget))
                return false;
            const float current = hinge->GetCurrentAngle();
            const float target = hinge->GetTargetAngle();
            const float minimum = hinge->GetLimitsMin();
            const float maximum = hinge->GetLimitsMax();
            if (!std::isfinite(current) || !std::isfinite(target) || !std::isfinite(minimum)
                || !std::isfinite(maximum))
                return false;
            telemetry.mConstraintType = ConstraintType::Hinge;
            telemetry.mCurrentHingeAngle = current;
            telemetry.mRequestedHingeAngle = mRequestedHingeAngles[bodyIndex];
            telemetry.mTargetHingeAngle = target;
            telemetry.mHingeMinimum = minimum;
            telemetry.mHingeMaximum = maximum;
            telemetry.mAngularError = std::abs(target - current);
            if (telemetry.mRequestedHingeAngle)
                telemetry.mTargetClampDelta = std::abs(*telemetry.mRequestedHingeAngle - target);
        }
        else
            return false;
        const auto finiteOptional = [](const std::optional<float>& value) {
            return !value || std::isfinite(*value);
        };
        if (!std::isfinite(telemetry.mAngularError) || !std::isfinite(telemetry.mTargetClampDelta)
            || !finiteOptional(telemetry.mNormalHalfConeAngle) || !finiteOptional(telemetry.mPlaneHalfConeAngle)
            || !finiteOptional(telemetry.mTwistMinimum) || !finiteOptional(telemetry.mTwistMaximum))
            return false;
        out = std::move(telemetry);
        return true;
    }

    void PassiveRagdoll::disableMotors()
    {
        std::fill(mRequestedHingeAngles.begin(), mRequestedHingeAngles.end(), std::nullopt);
        clearForceMotors();
        mTorqueMotorImpulses.clear();
        for (std::size_t i = 0; i < mRagdoll->GetConstraintCount(); ++i)
        {
            JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(static_cast<int>(i));
            // Resolve impact stretch in the passive constraint island with
            // the authored masses, instead of redistributing body mass.
            // Powered pose drive restores the world's default iteration counts.
            constraint->SetNumPositionStepsOverride(32);
            constraint->SetNumVelocityStepsOverride(32);
            if (constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
            {
                auto* swingTwist = static_cast<JPH::SwingTwistConstraint*>(constraint);
                swingTwist->SetSwingMotorState(JPH::EMotorState::Off);
                swingTwist->SetTwistMotorState(JPH::EMotorState::Off);
                swingTwist->SetTargetAngularVelocityCS(JPH::Vec3::sZero());
            }
            else if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
            {
                auto* hinge = static_cast<JPH::HingeConstraint*>(constraint);
                hinge->SetMotorState(JPH::EMotorState::Off);
                hinge->SetTargetAngularVelocity(0);
            }
        }
    }

    bool PassiveRagdoll::getMotorAngularImpulse(std::size_t bodyIndex, float fixedTimeStep,
        JPH::Vec3& outImpulse, float& outPeakAxisUtilization) const
    {
        if (!mAdded || bodyIndex >= getBodyCount() || !std::isfinite(fixedTimeStep)
            || fixedTimeStep <= 0.0f)
            return false;
        const JPH::RagdollSettings* settings = mRagdoll->GetRagdollSettings();
        const int constraintIndex = settings->GetConstraintIndexForBodyIndex(static_cast<int>(bodyIndex));
        if (constraintIndex < 0)
            return false;

        const auto replacement = std::find_if(mTorqueMotorImpulses.begin(), mTorqueMotorImpulses.end(),
            [bodyIndex](const auto& entry) { return entry.mBodyIndex == bodyIndex; });
        if (replacement != mTorqueMotorImpulses.end() && fixedTimeStep != replacement->mDuration)
            return false;
        if (replacement != mTorqueMotorImpulses.end() && !replacement->mNativeLoad)
        {
            outImpulse = replacement->mImpulse;
            outPeakAxisUtilization = replacement->mUtilization;
            return true;
        }
        const JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(constraintIndex);
        if (static_cast<std::size_t>(constraintIndex) >= mMotorImpulseWorldAxes.size())
            return false;
        const auto& axes = mMotorImpulseWorldAxes[constraintIndex];
        JPH::Vec3 impulse;
        float peakAxisUtilization = 0.0f;
        const auto includeAxis = [&](float lambda, const JPH::MotorSettings& motor) {
            const float maximumTorque
                = std::max(std::abs(motor.mMinTorqueLimit), std::abs(motor.mMaxTorqueLimit));
            const float maximumImpulse = maximumTorque * fixedTimeStep;
            if (!std::isfinite(maximumImpulse) || maximumImpulse < 0.0f)
                return false;
            if (maximumImpulse == 0.0f)
                return std::abs(lambda) <= std::numeric_limits<float>::epsilon();
            peakAxisUtilization = std::max(peakAxisUtilization, std::abs(lambda) / maximumImpulse);
            return true;
        };
        if (constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist)
        {
            const auto* swingTwist = static_cast<const JPH::SwingTwistConstraint*>(constraint);
            const JPH::Vec3 lambda = swingTwist->GetTotalLambdaMotor();
            if (!includeAxis(lambda.GetX(), swingTwist->GetTwistMotorSettings())
                || !includeAxis(lambda.GetY(), swingTwist->GetSwingMotorSettings())
                || !includeAxis(lambda.GetZ(), swingTwist->GetSwingMotorSettings()))
                return false;
            impulse = axes[0] * lambda.GetX() + axes[1] * lambda.GetY() + axes[2] * lambda.GetZ();
        }
        else if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
        {
            const auto* hinge = static_cast<const JPH::HingeConstraint*>(constraint);
            const float lambda = hinge->GetTotalLambdaMotor();
            if (!includeAxis(lambda, hinge->GetMotorSettings()))
                return false;
            impulse = axes[0] * lambda;
        }
        else
            return false;
        if (!isFinite(impulse) || !std::isfinite(peakAxisUtilization))
            return false;
        outImpulse = impulse;
        outPeakAxisUtilization = peakAxisUtilization;
        return true;
    }

    bool PassiveRagdoll::applyUprightControl(const UprightControllerOutput& control,
        std::size_t pelvisBodyIndex, std::size_t spineBodyIndex,
        std::span<const std::size_t> verticalReactionBodyIndices,
        std::span<const std::size_t> horizontalReactionBodyIndices,
        const ActiveControllerProfile& profile,
        std::optional<std::size_t> angularReactionBodyIndex, bool centerPelvisForce,
        std::optional<std::size_t> secondaryAngularReactionBodyIndex)
    {
        if (!mAdded || pelvisBodyIndex >= getBodyCount() || spineBodyIndex >= getBodyCount()
            || pelvisBodyIndex == spineBodyIndex
            || !isWithinMagnitude(control.mPelvisForce, profile.mMaximumPelvisForce)
            || !isWithinMagnitude(control.mPelvisTorque, profile.mMaximumPelvisTorque)
            || !isWithinMagnitude(control.mSpineTorque, profile.mMaximumSpineTorque))
            return false;
        const auto validReactionBodies = [&](std::span<const std::size_t> bodyIndices) {
            for (std::size_t i = 0; i < bodyIndices.size(); ++i)
            {
                if (bodyIndices[i] >= getBodyCount() || bodyIndices[i] == pelvisBodyIndex)
                    return false;
                for (std::size_t j = 0; j < i; ++j)
                {
                    if (bodyIndices[j] == bodyIndices[i])
                        return false;
                }
            }
            return true;
        };
        std::array<std::size_t, 2> angularBodies{};
        std::size_t angularCount = 0;
        if (angularReactionBodyIndex)
            angularBodies[angularCount++] = *angularReactionBodyIndex;
        if (secondaryAngularReactionBodyIndex)
        {
            if (!angularReactionBodyIndex)
                return false;
            angularBodies[angularCount++] = *secondaryAngularReactionBodyIndex;
        }
        const std::span<const std::size_t> angularReactions(angularBodies.data(), angularCount);
        if (!validReactionBodies(verticalReactionBodyIndices)
            || !validReactionBodies(horizontalReactionBodyIndices)
            || !validReactionBodies(angularReactions))
            return false;

        JPH::BodyInterface& bodyInterface = mPhysicsSystem->GetBodyInterface();
        const JPH::BodyID pelvisBody = getBodyId(pelvisBodyIndex);
        const JPH::BodyID spineBody = getBodyId(spineBodyIndex);
        const JPH::Vec3 verticalForce(0.0f, 0.0f, control.mPelvisForce.GetZ());
        const JPH::Vec3 horizontalForce
            = control.mPelvisForce - verticalForce;
        const auto applyForcePairs = [&](JPH::Vec3Arg force,
                                         std::span<const std::size_t> reactionBodyIndices) {
            if (force.IsNearZero() || reactionBodyIndices.empty())
                return;
            const JPH::Vec3 share = force / static_cast<float>(reactionBodyIndices.size());
            const JPH::RVec3 pelvisCenter = bodyInterface.GetCenterOfMassPosition(pelvisBody);
            for (const std::size_t bodyIndex : reactionBodyIndices)
            {
                const JPH::BodyID reactionBody = getBodyId(bodyIndex);
                // In gait, keep the requested pelvis wrench intact. Standing
                // and recovery use the existing symmetric force distribution.
                const JPH::RVec3 reactionCenter = bodyInterface.GetCenterOfMassPosition(reactionBody);
                const JPH::RVec3 applicationPoint = centerPelvisForce
                    ? pelvisCenter : pelvisCenter + 0.5 * (reactionCenter - pelvisCenter);
                if (mUseSolverWrenches)
                {
                    applySolverWrench(pelvisBodyIndex, bodyIndex, share,
                        JPH::Vec3(applicationPoint - pelvisCenter).Cross(share));
                    continue;
                }
                bodyInterface.AddForce(pelvisBody, share, applicationPoint, JPH::EActivation::Activate);
                bodyInterface.AddForce(reactionBody, -share, applicationPoint, JPH::EActivation::Activate);
            }
        };
        applyForcePairs(verticalForce, verticalReactionBodyIndices);
        applyForcePairs(horizontalForce, horizontalReactionBodyIndices);
        if (mUseSolverWrenches && angularReactionBodyIndex)
        {
            for (const std::size_t reaction : angularReactions)
            {
                applySolverWrench(pelvisBodyIndex, reaction,
                    JPH::Vec3::sZero(), control.mPelvisTorque / static_cast<float>(angularCount));
                applySolverWrench(spineBodyIndex, reaction,
                    JPH::Vec3::sZero(), control.mSpineTorque / static_cast<float>(angularCount));
            }
        }
        else
        {
            bodyInterface.AddTorque(pelvisBody, control.mPelvisTorque, JPH::EActivation::Activate);
            bodyInterface.AddTorque(spineBody, control.mSpineTorque, JPH::EActivation::Activate);
            for (const std::size_t reaction : angularReactions)
                bodyInterface.AddTorque(getBodyId(reaction),
                    -(control.mPelvisTorque + control.mSpineTorque) / static_cast<float>(angularCount),
                    JPH::EActivation::Activate);
        }
        return true;
    }

    bool PassiveRagdoll::applyBodyForcePair(std::size_t drivenBodyIndex,
        std::size_t reactionBodyIndex, JPH::Vec3Arg force, float maximumForce)
    {
        if (!mAdded || drivenBodyIndex >= getBodyCount() || reactionBodyIndex >= getBodyCount()
            || drivenBodyIndex == reactionBodyIndex || !isFinite(force)
            || !isWithinMagnitude(force, maximumForce))
            return false;

        JPH::BodyInterface& bodyInterface = mPhysicsSystem->GetBodyInterface();
        const JPH::BodyID drivenBody = getBodyId(drivenBodyIndex);
        const JPH::BodyID reactionBody = getBodyId(reactionBodyIndex);
        const JPH::RVec3 drivenCenter = bodyInterface.GetCenterOfMassPosition(drivenBody);
        const JPH::RVec3 reactionCenter = bodyInterface.GetCenterOfMassPosition(reactionBody);
        const JPH::RVec3 applicationPoint = drivenCenter + 0.5 * (reactionCenter - drivenCenter);
        if (mUseSolverWrenches)
        {
            applySolverWrench(drivenBodyIndex, reactionBodyIndex, force,
                JPH::Vec3(applicationPoint - drivenCenter).Cross(force));
            return true;
        }
        bodyInterface.AddForce(drivenBody, force, applicationPoint, JPH::EActivation::Activate);
        bodyInterface.AddForce(reactionBody, -force, applicationPoint, JPH::EActivation::Activate);
        return true;
    }

    bool PassiveRagdoll::applyExternalPelvisAssist(
        std::size_t pelvisBodyIndex, JPH::Vec3Arg force, float maximumForce,
        JPH::Vec3Arg torque, float maximumTorque)
    {
        if (!mAdded || pelvisBodyIndex >= getBodyCount() || !isFinite(force) || !isFinite(torque)
            || !std::isfinite(maximumForce) || maximumForce < 0.0f
            || !std::isfinite(maximumTorque) || maximumTorque < 0.0f
            || !isWithinMagnitude(force, maximumForce) || !isWithinMagnitude(torque, maximumTorque))
            return false;
        JPH::BodyInterface& bodyInterface = mPhysicsSystem->GetBodyInterface();
        const JPH::BodyID pelvis = getBodyId(pelvisBodyIndex);
        if (!force.IsNearZero())
            bodyInterface.AddForce(pelvis, force, JPH::EActivation::Activate);
        if (!torque.IsNearZero())
            bodyInterface.AddTorque(pelvis, torque, JPH::EActivation::Activate);
        return true;
    }

    bool PassiveRagdoll::applyExternalBodyWrench(std::size_t bodyIndex, JPH::Vec3Arg force, JPH::Vec3Arg torque)
    {
        if (!mAdded || bodyIndex >= getBodyCount() || !isFinite(force) || !isFinite(torque))
            return false;
        JPH::BodyInterface& bodyInterface = mPhysicsSystem->GetBodyInterface();
        const JPH::BodyID body = getBodyId(bodyIndex);
        if (bodyInterface.GetMotionType(body) != JPH::EMotionType::Dynamic)
            return false;
        if (!force.IsNearZero())
            bodyInterface.AddForce(body, force, JPH::EActivation::Activate);
        if (!torque.IsNearZero())
            bodyInterface.AddTorque(body, torque, JPH::EActivation::Activate);
        return true;
    }

    bool PassiveRagdoll::applyBodyAngularImpulsePair(std::size_t drivenBodyIndex,
        std::size_t reactionBodyIndex, JPH::Vec3Arg angularImpulse, float maximumAngularImpulse)
    {
        if (!mAdded || drivenBodyIndex >= getBodyCount() || reactionBodyIndex >= getBodyCount()
            || drivenBodyIndex == reactionBodyIndex || !isFinite(angularImpulse)
            || !isWithinMagnitude(angularImpulse, maximumAngularImpulse))
            return false;

        JPH::BodyInterface& bodyInterface = mPhysicsSystem->GetBodyInterface();
        bodyInterface.AddAngularImpulse(getBodyId(drivenBodyIndex), angularImpulse);
        bodyInterface.AddAngularImpulse(getBodyId(reactionBodyIndex), -angularImpulse);
        return true;
    }

    bool PassiveRagdoll::applyBodyTorquePair(std::size_t drivenBodyIndex,
        std::size_t reactionBodyIndex, JPH::Vec3Arg torque, float maximumTorque)
    {
        if (!mAdded || drivenBodyIndex >= getBodyCount() || reactionBodyIndex >= getBodyCount()
            || drivenBodyIndex == reactionBodyIndex || !isFinite(torque)
            || !isWithinMagnitude(torque, maximumTorque))
            return false;

        if (mUseSolverWrenches)
        {
            applySolverWrench(drivenBodyIndex, reactionBodyIndex, JPH::Vec3::sZero(), torque);
            return true;
        }
        JPH::BodyInterface& bodyInterface = mPhysicsSystem->GetBodyInterface();
        bodyInterface.AddTorque(getBodyId(drivenBodyIndex), torque, JPH::EActivation::Activate);
        bodyInterface.AddTorque(getBodyId(reactionBodyIndex), -torque, JPH::EActivation::Activate);
        return true;
    }

    std::optional<JPH::RVec3> estimateHorizontalPlaneCenterOfPressure(
        const RagdollSolverImpulse& impulse, double planeHeight)
    {
        if (impulse.mVelocityLimitTouched || impulse.mActivationChanged || impulse.mContinuousCollisionEnabled
            || !std::isfinite(planeHeight)
            || !isFinite(impulse.mReferenceCenterOfMass) || !isFinite(impulse.mLinearImpulse)
            || !isFinite(impulse.mAngularImpulse) || impulse.mLinearImpulse.GetZ() <= 0.0f)
            return std::nullopt;
        const double height = impulse.mReferenceCenterOfMass.GetZ() - planeHeight;
        const JPH::RVec3 result(impulse.mReferenceCenterOfMass.GetX()
                - (impulse.mAngularImpulse.GetY() + height * impulse.mLinearImpulse.GetX()) / impulse.mLinearImpulse.GetZ(),
            impulse.mReferenceCenterOfMass.GetY()
                + (impulse.mAngularImpulse.GetX() - height * impulse.mLinearImpulse.GetY()) / impulse.mLinearImpulse.GetZ(),
            planeHeight);
        return isFinite(result) ? std::optional<JPH::RVec3>(result) : std::nullopt;
    }

    bool PassiveRagdoll::beginSolverImpulseMeasurement(float fixedTimeStep)
    {
        mSolverImpulseSnapshot.reset();
        if (!mAdded || !std::isfinite(fixedTimeStep) || fixedTimeStep <= 0.0f)
            return false;
        SolverImpulseSnapshot snapshot;
        snapshot.mDuration = fixedTimeStep;
        float totalMass = 0.0f;
        for (std::size_t i = 0; i < getBodyCount(); ++i)
        {
            JPH::BodyLockRead lock(mPhysicsSystem->GetBodyLockInterface(), getBodyId(i));
            if (!lock.Succeeded() || !lock.GetBody().IsDynamic())
                return false;
            const JPH::Body& body = lock.GetBody();
            const auto& properties = *body.GetMotionProperties();
            snapshot.mContinuousCollisionEnabled = snapshot.mContinuousCollisionEnabled
                || properties.GetMotionQuality() == JPH::EMotionQuality::LinearCast;
            if (properties.GetAllowedDOFs() != JPH::EAllowedDOFs::All)
                return false;
            const JPH::Vec3 inverseInertia = properties.GetInverseInertiaDiagonal();
            if (!isFinite(inverseInertia) || inverseInertia.GetX() <= 0.0f
                || inverseInertia.GetY() <= 0.0f || inverseInertia.GetZ() <= 0.0f
                || properties.GetInverseMass() <= 0.0f)
                return false;
            JPH::MotionProperties predicted = properties;
            JPH::MotionProperties unclamped = properties;
            const float unlimited = std::sqrt(std::numeric_limits<float>::max());
            unclamped.SetMaxLinearVelocity(unlimited);
            unclamped.SetMaxAngularVelocity(unlimited);
            if (body.IsActive())
            {
                if (body.GetApplyGyroscopicForce())
                {
                    predicted.ApplyGyroscopicForceInternal(body.GetRotation(), fixedTimeStep);
                    unclamped.ApplyGyroscopicForceInternal(body.GetRotation(), fixedTimeStep);
                }
                predicted.ApplyForceTorqueAndDragInternal(body.GetRotation(), mPhysicsSystem->GetGravity(), fixedTimeStep);
                unclamped.ApplyForceTorqueAndDragInternal(body.GetRotation(), mPhysicsSystem->GetGravity(), fixedTimeStep);
            }
            SolverBodySnapshot entry;
            entry.mCenterOfMass = body.GetCenterOfMassPosition();
            entry.mInertiaRotation = body.GetRotation() * properties.GetInertiaRotation();
            entry.mInertiaDiagonal = JPH::Vec3::sOne() / inverseInertia;
            entry.mMass = 1.0f / properties.GetInverseMass();
            entry.mActive = body.IsActive();
            entry.mFreeMotion = { predicted.GetLinearVelocity(), predicted.GetAngularVelocity() };
            entry.mUnclampedMotion = { unclamped.GetLinearVelocity(), unclamped.GetAngularVelocity() };
            if (!isFinite(entry.mCenterOfMass) || !isFinite(entry.mInertiaRotation)
                || !isFinite(entry.mInertiaDiagonal) || !isFinite(entry.mFreeMotion)
                || !isFinite(entry.mUnclampedMotion)
                || !std::isfinite(entry.mMass))
                return false;
            snapshot.mCenterOfMass += entry.mCenterOfMass * entry.mMass;
            totalMass += entry.mMass;
            snapshot.mBodies.push_back(entry);
        }
        if (!std::isfinite(totalMass) || totalMass <= 0.0f)
            return false;
        snapshot.mCenterOfMass /= totalMass;
        for (std::size_t i = 0; i < mRagdoll->GetConstraintCount(); ++i)
        {
            const auto* constraint = mRagdoll->GetConstraint(static_cast<int>(i));
            if (constraint->GetSubType() != JPH::EConstraintSubType::SwingTwist
                && constraint->GetSubType() != JPH::EConstraintSubType::Hinge)
                return false;
            const auto parent = constraint->GetBody1()->GetCenterOfMassTransform()
                * constraint->GetConstraintToBody1Matrix();
            const auto child = constraint->GetBody2()->GetCenterOfMassTransform()
                * constraint->GetConstraintToBody2Matrix();
            // Sleeping or disabled constraints retain their previous lambda,
            // but apply no impulse in this solve. Waking is flagged below.
            snapshot.mAnchorDifferences.emplace_back(constraint->IsActive()
                    ? JPH::Vec3(child.GetTranslation() - parent.GetTranslation()) : JPH::Vec3::sZero());
        }
        mSolverImpulseSnapshot = std::move(snapshot);
        return true;
    }

    bool PassiveRagdoll::endSolverImpulseMeasurement(float fixedTimeStep, RagdollSolverImpulse& out) const
    {
        if (!mAdded || !mSolverImpulseSnapshot || fixedTimeStep != mSolverImpulseSnapshot->mDuration
            || mSolverImpulseSnapshot->mBodies.size() != getBodyCount()
            || mSolverImpulseSnapshot->mAnchorDifferences.size() != mRagdoll->GetConstraintCount())
            return false;
        const auto& snapshot = *mSolverImpulseSnapshot;
        RagdollSolverImpulse result;
        result.mReferenceCenterOfMass = snapshot.mCenterOfMass;
        result.mContinuousCollisionEnabled = snapshot.mContinuousCollisionEnabled;
        for (std::size_t i = 0; i < getBodyCount(); ++i)
        {
            JPH::BodyLockRead lock(mPhysicsSystem->GetBodyLockInterface(), getBodyId(i));
            if (!lock.Succeeded() || !lock.GetBody().IsDynamic())
                return false;
            const auto& body = lock.GetBody();
            const auto& before = snapshot.mBodies[i];
            const JPH::Vec3 limitedLinear = before.mFreeMotion.mLinearVelocity - before.mUnclampedMotion.mLinearVelocity;
            const JPH::Vec3 limitedAngular = before.mFreeMotion.mAngularVelocity - before.mUnclampedMotion.mAngularVelocity;
            const JPH::Vec3 limitImpulse = limitedLinear * before.mMass;
            result.mFreeLinearLimitImpulse += limitImpulse;
            result.mFreeAngularLimitImpulse += before.mInertiaRotation
                * (before.mInertiaDiagonal * before.mInertiaRotation.InverseRotate(limitedAngular))
                + JPH::Vec3(before.mCenterOfMass - snapshot.mCenterOfMass).Cross(limitImpulse);
            result.mFreeVelocityLimitTouched = result.mFreeVelocityLimitTouched
                || limitedLinear != JPH::Vec3::sZero() || limitedAngular != JPH::Vec3::sZero();
            const JPH::Vec3 impulse = (body.GetLinearVelocity() - before.mFreeMotion.mLinearVelocity) * before.mMass;
            const JPH::Vec3 angularDelta = body.GetAngularVelocity() - before.mFreeMotion.mAngularVelocity;
            const JPH::Vec3 angularImpulse = before.mInertiaRotation
                * (before.mInertiaDiagonal * before.mInertiaRotation.InverseRotate(angularDelta));
            result.mLinearImpulse += impulse;
            result.mAngularImpulse += angularImpulse
                + JPH::Vec3(before.mCenterOfMass - snapshot.mCenterOfMass).Cross(impulse);
            const auto& properties = *body.GetMotionProperties();
            const float linearLimit = properties.GetMaxLinearVelocity();
            const float angularLimit = properties.GetMaxAngularVelocity();
            result.mVelocityLimitTouched = result.mVelocityLimitTouched
                || body.GetLinearVelocity().LengthSq() >= std::nextafter(linearLimit * linearLimit, 0.0f)
                || body.GetAngularVelocity().LengthSq() >= std::nextafter(angularLimit * angularLimit, 0.0f);
            result.mActivationChanged = result.mActivationChanged || body.IsActive() != before.mActive;
        }
        for (std::size_t i = 0; i < mRagdoll->GetConstraintCount(); ++i)
        {
            const auto* constraint = mRagdoll->GetConstraint(static_cast<int>(i));
            const JPH::Vec3 impulse = constraint->GetSubType() == JPH::EConstraintSubType::SwingTwist
                ? static_cast<const JPH::SwingTwistConstraint*>(constraint)->GetTotalLambdaPosition()
                : static_cast<const JPH::HingeConstraint*>(constraint)->GetTotalLambdaPosition();
            result.mInternalAnchorImpulseMoment += snapshot.mAnchorDifferences[i].Cross(impulse);
        }
        result.mAngularImpulse -= result.mInternalAnchorImpulseMoment;
        if (!isFinite(result.mLinearImpulse) || !isFinite(result.mAngularImpulse)
            || !isFinite(result.mFreeLinearLimitImpulse) || !isFinite(result.mFreeAngularLimitImpulse)
            || !isFinite(result.mInternalAnchorImpulseMoment))
            return false;
        out = result;
        return true;
    }

    bool PassiveRagdoll::captureMomentum(RagdollMomentum& out) const
    {
        if (!mAdded || getBodyCount() == 0)
            return false;
        RagdollMomentum result;
        JPH::RVec3 origin = JPH::RVec3::sZero();
        JPH::RVec3 weightedOffset = JPH::RVec3::sZero();
        for (std::size_t i = 0; i < getBodyCount(); ++i)
        {
            JPH::BodyLockRead lock(mPhysicsSystem->GetBodyLockInterface(), getBodyId(i));
            if (!lock.Succeeded() || !lock.GetBody().IsDynamic())
                return false;
            const auto& body = lock.GetBody();
            const auto& properties = *body.GetMotionProperties();
            const JPH::Vec3 inverseInertia = properties.GetInverseInertiaDiagonal();
            if (properties.GetAllowedDOFs() != JPH::EAllowedDOFs::All
                || properties.GetInverseMass() <= 0.0f || !isFinite(inverseInertia)
                || inverseInertia.GetX() <= 0.0f || inverseInertia.GetY() <= 0.0f || inverseInertia.GetZ() <= 0.0f)
                return false;
            if (i == 0)
                origin = body.GetCenterOfMassPosition();
            const float mass = 1.0f / properties.GetInverseMass();
            const JPH::RVec3 offset = body.GetCenterOfMassPosition() - origin;
            const JPH::Quat inertiaFrame = body.GetRotation() * properties.GetInertiaRotation();
            const JPH::Vec3 spin = inertiaFrame
                * (inertiaFrame.InverseRotate(body.GetAngularVelocity()) / inverseInertia);
            const JPH::Vec3 linear = body.GetLinearVelocity() * mass;
            result.mTotalMass += mass;
            weightedOffset += offset * mass;
            result.mLinearMomentum += linear;
            result.mAngularMomentum += spin + JPH::Vec3(offset).Cross(linear);
        }
        if (!std::isfinite(result.mTotalMass) || result.mTotalMass <= 0.0f)
            return false;
        const JPH::RVec3 centerOffset = weightedOffset / result.mTotalMass;
        result.mCenterOfMass = origin + centerOffset;
        result.mAngularMomentum -= JPH::Vec3(centerOffset).Cross(result.mLinearMomentum);
        if (!isFinite(result.mCenterOfMass) || !isFinite(result.mLinearMomentum) || !isFinite(result.mAngularMomentum))
            return false;
        out = result;
        return true;
    }

    bool PassiveRagdoll::captureTelemetry(RagdollTelemetry& out) const
    {
        if (!mAdded || mPhysicsSystem == nullptr || mBodyMasses.size() != getBodyCount())
            return false;

        const JPH::BodyInterface& bodyInterface = mPhysicsSystem->GetBodyInterface();
        RagdollTelemetry captured;
        JPH::RVec3 weightedCenter = JPH::RVec3::sZero();
        captured.mBodies.reserve(getBodyCount());
        for (std::size_t i = 0; i < getBodyCount(); ++i)
        {
            const JPH::BodyID bodyId = getBodyId(i);
            BodyTelemetry body;
            body.mPosition = bodyInterface.GetPosition(bodyId);
            body.mCenterOfMass = bodyInterface.GetCenterOfMassPosition(bodyId);
            body.mRotation = bodyInterface.GetRotation(bodyId);
            body.mMotion.mLinearVelocity = bodyInterface.GetLinearVelocity(bodyId);
            body.mMotion.mAngularVelocity = bodyInterface.GetAngularVelocity(bodyId);
            body.mMass = mBodyMasses[i];
            body.mActive = bodyInterface.IsActive(bodyId);
            if (!std::isfinite(body.mMass) || body.mMass <= 0.0f || !isFinite(body.mPosition)
                || !isFinite(body.mCenterOfMass) || !isFinite(body.mRotation) || !isFinite(body.mMotion))
                return false;
            weightedCenter += body.mCenterOfMass * body.mMass;
            captured.mTotalMass += body.mMass;
            captured.mBodies.push_back(body);
        }
        if (!std::isfinite(captured.mTotalMass) || captured.mTotalMass <= 0.0f)
            return false;
        captured.mCenterOfMass = weightedCenter / captured.mTotalMass;

        const auto findBodyIndex = [&](JPH::BodyID id) -> std::optional<std::size_t> {
            for (std::size_t i = 0; i < getBodyCount(); ++i)
            {
                if (getBodyId(i) == id)
                    return i;
            }
            return std::nullopt;
        };

        captured.mConstraints.reserve(mRagdoll->GetConstraintCount());
        for (std::size_t i = 0; i < mRagdoll->GetConstraintCount(); ++i)
        {
            const JPH::TwoBodyConstraint* constraint = mRagdoll->GetConstraint(static_cast<int>(i));
            const std::optional<std::size_t> parent = findBodyIndex(constraint->GetBody1()->GetID());
            const std::optional<std::size_t> child = findBodyIndex(constraint->GetBody2()->GetID());
            if (!parent || !child)
                return false;

            const JPH::RMat44 parentFrame = bodyInterface.GetCenterOfMassTransform(getBodyId(*parent))
                * constraint->GetConstraintToBody1Matrix();
            const JPH::RMat44 childFrame = bodyInterface.GetCenterOfMassTransform(getBodyId(*child))
                * constraint->GetConstraintToBody2Matrix();
            ConstraintTelemetry item;
            item.mParentBodyIndex = *parent;
            item.mChildBodyIndex = *child;
            item.mPosition = childFrame.GetTranslation();
            item.mAnchorSeparation = static_cast<float>(
                (parentFrame.GetTranslation() - childFrame.GetTranslation()).Length());
            if (!std::isfinite(item.mAnchorSeparation))
                return false;

            if (constraint->GetSubType() == JPH::EConstraintSubType::Hinge)
            {
                const auto* hinge = static_cast<const JPH::HingeConstraint*>(constraint);
                item.mHingeAngle = hinge->GetCurrentAngle();
                item.mHingeMinimum = hinge->GetLimitsMin();
                item.mHingeMaximum = hinge->GetLimitsMax();
                if (!std::isfinite(*item.mHingeAngle) || !std::isfinite(*item.mHingeMinimum)
                    || !std::isfinite(*item.mHingeMaximum))
                    return false;
            }
            captured.mConstraints.push_back(item);
        }

        if (!isFinite(captured.mCenterOfMass))
            return false;
        out = std::move(captured);
        return true;
    }

    PassiveRagdollCreateResult createPassiveRagdoll(JPH::PhysicsSystem& physicsSystem,
        const SkeletonDescriptor& skeleton, const RagdollProfile& profile, const BuildOptions& buildOptions,
        const PassiveRagdollInitialState& initialState, JPH::CollisionGroup::GroupID collisionGroup,
        std::uint64_t userData)
    {
        PassiveRagdollCreateResult result;
        BuildResult build = buildRagdollSettings(skeleton, profile, buildOptions);
        result.mValidation = std::move(build.mValidation);
        validateInitialState(initialState, profile.mBodies.size(), result.mValidation);
        if (!result.mValidation.isValid() || build.mSettings == nullptr)
            return result;

        std::unique_ptr<JPH::Ragdoll> ragdoll(
            build.mSettings->CreateRagdoll(collisionGroup, userData, &physicsSystem));
        if (!ragdoll)
        {
            result.mValidation.mIssues.push_back({ ValidationCode::RuntimeCreationFailed, "physicsSystem",
                "Jolt could not allocate all passive-ragdoll bodies." });
            return result;
        }

        JPH::BodyInterface& bodyInterface = physicsSystem.GetBodyInterface();
        for (std::size_t i = 0; i < ragdoll->GetBodyCount(); ++i)
        {
            const BodyMotion& motion
                = initialState.mBodyMotions.empty() ? initialState.mActorMotion : initialState.mBodyMotions[i];
            const auto bodyId = ragdoll->GetBodyID(static_cast<int>(i));
            const JPH::Vec3 offset(bodyInterface.GetCenterOfMassPosition(bodyId) - bodyInterface.GetPosition(bodyId));
            const JPH::Vec3 centerVelocity = motion.mLinearVelocity + motion.mAngularVelocity.Cross(offset);
            if (!isFinite(centerVelocity))
            {
                addInitialStateIssue(result.mValidation, "initialState.bodyMotions",
                    "Transferring body-origin motion produced a non-finite mass-center velocity.");
                return result;
            }
            bodyInterface.SetLinearAndAngularVelocity(bodyId, centerVelocity, motion.mAngularVelocity);
        }
        std::vector<float> bodyMasses;
        bodyMasses.reserve(build.mSettings->mParts.size());
        for (const JPH::RagdollSettings::Part& part : build.mSettings->mParts)
            bodyMasses.push_back(part.mMassPropertiesOverride.mMass);
        result.mRagdoll = std::unique_ptr<PassiveRagdoll>(new PassiveRagdoll(physicsSystem, std::move(ragdoll),
            buildOptions.mRootPosition, buildOptions.mRootRotation, buildOptions.mUniformScale,
            std::move(bodyMasses), initialState.mKillingImpulse));
        return result;
    }

    std::optional<std::vector<BodyMotion>> estimateBodyMotions(
        const MotionPose& previous, const MotionPose& current, float duration, const MotionTransferProfile& profile)
    {
        if (!std::isfinite(duration) || duration <= 0.0f || !std::isfinite(profile.mMaximumSampleDuration)
            || duration > profile.mMaximumSampleDuration || !std::isfinite(profile.mMaximumLinearSpeed)
            || profile.mMaximumLinearSpeed <= 0.0f || !std::isfinite(profile.mMaximumAngularSpeed)
            || profile.mMaximumAngularSpeed <= 0.0f || !std::isfinite(previous.mUniformScale)
            || previous.mUniformScale <= 0.0f || !std::isfinite(current.mUniformScale)
            || current.mUniformScale <= 0.0f || !isFinite(previous.mRootTransform)
            || !isFinite(current.mRootTransform) || previous.mBodyTransforms.size() != current.mBodyTransforms.size()
            || current.mBodyTransforms.empty())
            return std::nullopt;

        std::vector<BodyMotion> result;
        result.reserve(current.mBodyTransforms.size());
        for (std::size_t i = 0; i < current.mBodyTransforms.size(); ++i)
        {
            const Transform& previousBody = previous.mBodyTransforms[i];
            const Transform& currentBody = current.mBodyTransforms[i];
            if (!isFinite(previousBody) || !isFinite(currentBody))
                return std::nullopt;

            const JPH::Vec3 previousPosition = previous.mRootTransform.mTranslation
                + previous.mRootTransform.mRotation * (previousBody.mTranslation * previous.mUniformScale);
            const JPH::Vec3 currentPosition = current.mRootTransform.mTranslation
                + current.mRootTransform.mRotation * (currentBody.mTranslation * current.mUniformScale);
            const JPH::Quat previousRotation = previous.mRootTransform.mRotation * previousBody.mRotation;
            const JPH::Quat currentRotation = current.mRootTransform.mRotation * currentBody.mRotation;

            BodyMotion motion;
            motion.mLinearVelocity
                = clampLength((currentPosition - previousPosition) / duration, profile.mMaximumLinearSpeed);
            const JPH::Quat delta = (currentRotation * previousRotation.Conjugated()).Normalized();
            JPH::Vec3 axis;
            float angle = 0.0f;
            delta.GetAxisAngle(axis, angle);
            motion.mAngularVelocity = clampLength(axis * (angle / duration), profile.mMaximumAngularSpeed);
            if (!isFinite(motion))
                return std::nullopt;
            result.push_back(motion);
        }
        return result;
    }

    bool interpolateRagdollPoses(
        const RagdollPose& previous, const RagdollPose& current, float factor, RagdollPose& out)
    {
        if (!std::isfinite(factor) || factor < 0.0f || factor > 1.0f || previous.mBodyTransforms.empty()
            || previous.mBodyTransforms.size() != current.mBodyTransforms.size())
            return false;

        RagdollPose interpolated;
        interpolated.mBodyTransforms.reserve(previous.mBodyTransforms.size());
        for (std::size_t i = 0; i < previous.mBodyTransforms.size(); ++i)
        {
            const Transform& from = previous.mBodyTransforms[i];
            const Transform& to = current.mBodyTransforms[i];
            if (!isFinite(from) || !isFinite(to))
                return false;
            Transform transform;
            transform.mTranslation = from.mTranslation + (to.mTranslation - from.mTranslation) * factor;
            transform.mRotation = from.mRotation.SLERP(to.mRotation, factor);
            if (!isFinite(transform))
                return false;
            interpolated.mBodyTransforms.push_back(transform);
        }
        out = std::move(interpolated);
        return true;
    }

    std::optional<KillingImpulse> makeKillingImpulse(const SkeletonDescriptor& skeleton, const RagdollProfile& profile,
        const BuildOptions& buildOptions, JPH::RVec3Arg worldPoint, JPH::Vec3Arg direction, float healthDamage)
    {
        const float directionLengthSquared = direction.LengthSq();
        if (!isFinite(worldPoint) || !isFinite(direction) || !std::isfinite(healthDamage) || healthDamage <= 0.0f
            || !std::isfinite(directionLengthSquared) || directionLengthSquared <= 1e-6f || profile.mBodies.empty())
            return std::nullopt;

        const ImpactResponseProfile& response = profile.mImpactResponse;
        if (!std::isfinite(response.mImpulsePerDamage) || !std::isfinite(response.mMinimumImpulse)
            || !std::isfinite(response.mMaximumImpulse) || !std::isfinite(response.mUpwardBias)
            || response.mImpulsePerDamage < 0.0f || response.mMinimumImpulse < 0.0f
            || response.mMaximumImpulse < response.mMinimumImpulse || response.mUpwardBias < 0.0f
            || response.mUpwardBias > 1.0f)
            return std::nullopt;

        std::unordered_map<std::string_view, const SkeletonBone*> bones;
        bones.reserve(skeleton.mBones.size());
        for (const SkeletonBone& bone : skeleton.mBones)
            bones.emplace(bone.mName, &bone);

        std::size_t closestBody = 0;
        double closestDistanceSquared = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
        {
            const auto found = bones.find(profile.mBodies[i].mBone);
            if (found == bones.end())
                return std::nullopt;
            const JPH::Vec3 modelPosition = found->second->mModelTransform.mTranslation * buildOptions.mUniformScale;
            const JPH::RVec3 bodyPosition = buildOptions.mRootPosition + buildOptions.mRootRotation * modelPosition;
            const double distanceSquared = (bodyPosition - worldPoint).LengthSq();
            if (distanceSquared < closestDistanceSquared)
            {
                closestDistanceSquared = distanceSquared;
                closestBody = i;
            }
        }

        JPH::Vec3 impulseDirection = direction / std::sqrt(directionLengthSquared);
        impulseDirection += JPH::Vec3(0.0f, 0.0f, response.mUpwardBias);
        const float biasedLengthSquared = impulseDirection.LengthSq();
        if (!std::isfinite(biasedLengthSquared) || biasedLengthSquared <= 1e-6f)
            return std::nullopt;
        impulseDirection /= std::sqrt(biasedLengthSquared);
        const float magnitude
            = std::clamp(healthDamage * response.mImpulsePerDamage, response.mMinimumImpulse, response.mMaximumImpulse);
        return KillingImpulse{ closestBody, impulseDirection * magnitude, worldPoint };
    }
}
