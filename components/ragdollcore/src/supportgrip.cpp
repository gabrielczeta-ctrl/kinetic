#include <ragdollcore/supportgrip.hpp>

// clang-format off
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
// clang-format on

#include <array>
#include <cmath>
#include <utility>

namespace RagdollCore
{
    namespace
    {
        template <class Vector>
        bool finite(const Vector& value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        }

        bool validLimit(float limit)
        {
            return std::isfinite(limit) && limit >= 0;
        }

        SupportGripObservation observation(const SupportGripRequest& request,
            const JPH::Body& first, const JPH::Body& second)
        {
            SupportGripObservation result;
            result.mFirstBody = request.mFirstBody;
            result.mSecondBody = request.mSecondBody;
            result.mFirstWorldAnchor = first.GetCenterOfMassTransform() * request.mFirstLocalAnchor;
            result.mSecondWorldAnchor = second.GetCenterOfMassTransform() * request.mSecondLocalAnchor;
            result.mAnchorSeparation = JPH::Vec3(result.mSecondWorldAnchor - result.mFirstWorldAnchor).Length();
            result.mRelativeAnchorSpeed = (second.GetPointVelocity(result.mSecondWorldAnchor)
                - first.GetPointVelocity(result.mFirstWorldAnchor)).Length();
            return result;
        }

        bool validObservation(const SupportGripObservation& value)
        {
            return finite(value.mFirstWorldAnchor) && finite(value.mSecondWorldAnchor)
                && validLimit(value.mAnchorSeparation) && validLimit(value.mRelativeAnchorSpeed);
        }
    }

    struct SupportGrip::Attachment
    {
        SupportGripRequest mRequest;
        JPH::Ref<JPH::PointConstraint> mConstraint;
        JPH::Ref<JPH::SixDOFConstraint> mHold;
    };

    SupportGrip::SupportGrip(JPH::PhysicsSystem& physics)
        : mPhysics(physics)
    {
    }

    SupportGrip::~SupportGrip()
    {
        clear();
    }

    SupportGripAcquireResult SupportGrip::acquire(
        const SupportGripRequest& request, const SupportGripAcquisitionLimits& limits)
    {
        if (mAttachment)
            return SupportGripAcquireResult::AlreadyAttached;
        if (!finite(request.mFirstLocalAnchor) || !finite(request.mSecondLocalAnchor)
            || !validLimit(limits.mMaximumAnchorSeparation) || !validLimit(limits.mMaximumRelativeAnchorSpeed))
            return SupportGripAcquireResult::InvalidRequest;
        if (request.mFirstBody.IsInvalid() || request.mSecondBody.IsInvalid()
            || request.mFirstBody == request.mSecondBody)
            return SupportGripAcquireResult::InvalidBodies;

        const std::array ids{ request.mFirstBody, request.mSecondBody };
        auto attachment = std::make_unique<Attachment>();
        attachment->mRequest = request;
        {
            JPH::BodyLockMultiWrite lock(mPhysics.GetBodyLockInterface(), ids.data(), static_cast<int>(ids.size()));
            auto* first = lock.GetBody(0);
            auto* second = lock.GetBody(1);
            if (!first || !second || !first->IsRigidBody() || !second->IsRigidBody()
                || !first->IsDynamic() || !second->IsDynamic()
                || !first->IsInBroadPhase() || !second->IsInBroadPhase())
                return SupportGripAcquireResult::InvalidBodies;
            const auto state = observation(request, *first, *second);
            if (!validObservation(state))
                return SupportGripAcquireResult::InvalidRequest;
            if (state.mAnchorSeparation > limits.mMaximumAnchorSeparation)
                return SupportGripAcquireResult::TooFar;
            if (state.mRelativeAnchorSpeed > limits.mMaximumRelativeAnchorSpeed)
                return SupportGripAcquireResult::TooFast;

            JPH::PointConstraintSettings settings;
            settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
            settings.mPoint1 = JPH::RVec3(request.mFirstLocalAnchor);
            settings.mPoint2 = JPH::RVec3(request.mSecondLocalAnchor);
            attachment->mConstraint = static_cast<JPH::PointConstraint*>(settings.Create(*first, *second));
            attachment->mConstraint->SetNumVelocityStepsOverride(request.mVelocitySolverSteps);
            attachment->mConstraint->SetNumPositionStepsOverride(request.mPositionSolverSteps);
            attachment->mConstraint->SetConstraintPriority(request.mConstraintPriority);
            if (request.mHoldRelation && std::isfinite(request.mHoldTorque) && request.mHoldTorque > 0.0f
                && request.mHoldRelation->IsNormalized(1.0e-3f))
            {
                // Constraint space: the first body's own axes, and on the
                // second body those axes turned by the relation, so the
                // constraint's rotation is identity when the relation holds.
                const JPH::Quat relation = request.mHoldRelation->Normalized();
                JPH::SixDOFConstraintSettings hold;
                hold.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
                hold.mPosition1 = JPH::RVec3(request.mFirstLocalAnchor);
                hold.mPosition2 = JPH::RVec3(request.mSecondLocalAnchor);
                hold.mAxisX1 = JPH::Vec3::sAxisX();
                hold.mAxisY1 = JPH::Vec3::sAxisY();
                hold.mAxisX2 = relation * JPH::Vec3::sAxisX();
                hold.mAxisY2 = relation * JPH::Vec3::sAxisY();
                using Axis = JPH::SixDOFConstraintSettings::EAxis;
                for (const Axis axis : { Axis::TranslationX, Axis::TranslationY, Axis::TranslationZ,
                         Axis::RotationX, Axis::RotationY, Axis::RotationZ })
                    hold.MakeFreeAxis(axis);
                for (const Axis axis : { Axis::RotationX, Axis::RotationY, Axis::RotationZ })
                    hold.mMotorSettings[axis] = JPH::MotorSettings(request.mHoldFrequency, request.mHoldDamping,
                        0.0f, request.mHoldTorque);
                attachment->mHold = static_cast<JPH::SixDOFConstraint*>(hold.Create(*first, *second));
                for (const Axis axis : { Axis::RotationX, Axis::RotationY, Axis::RotationZ })
                    attachment->mHold->SetMotorState(axis, JPH::EMotorState::Position);
                attachment->mHold->SetTargetOrientationCS(JPH::Quat::sIdentity());
                attachment->mHold->SetNumVelocityStepsOverride(request.mVelocitySolverSteps);
                attachment->mHold->SetNumPositionStepsOverride(request.mPositionSolverSteps);
                attachment->mHold->SetConstraintPriority(request.mConstraintPriority);
            }
        }
        // No BodyInterface call while holding body locks. External serialization
        // prevents removal between validation, creation and registration.
        mPhysics.AddConstraint(attachment->mConstraint);
        mPhysics.GetBodyInterface().ActivateConstraint(attachment->mConstraint);
        if (attachment->mHold)
            mPhysics.AddConstraint(attachment->mHold);
        mAttachment = std::move(attachment);
        return SupportGripAcquireResult::Acquired;
    }

    bool SupportGrip::attached() const
    {
        return mAttachment != nullptr;
    }

    bool SupportGrip::setSecondLocalAnchor(JPH::Vec3Arg anchor)
    {
        if (!mAttachment || !finite(anchor))
            return false;
        mAttachment->mRequest.mSecondLocalAnchor = anchor;
        mAttachment->mConstraint->SetPoint2(JPH::EConstraintSpace::LocalToBodyCOM, JPH::RVec3(anchor));
        mPhysics.GetBodyInterface().ActivateConstraint(mAttachment->mConstraint);
        return true;
    }

    bool SupportGrip::clear()
    {
        if (!mAttachment)
            return false;
        mPhysics.RemoveConstraint(mAttachment->mConstraint);
        mPhysics.GetBodyInterface().ActivateConstraint(mAttachment->mConstraint);
        if (mAttachment->mHold)
            mPhysics.RemoveConstraint(mAttachment->mHold);
        mAttachment.reset();
        return true;
    }

    std::optional<SupportGripObservation> SupportGrip::observe() const
    {
        if (!mAttachment)
            return std::nullopt;
        const auto& request = mAttachment->mRequest;
        const std::array ids{ request.mFirstBody, request.mSecondBody };
        JPH::BodyLockMultiRead lock(mPhysics.GetBodyLockInterface(), ids.data(), static_cast<int>(ids.size()));
        const auto* first = lock.GetBody(0);
        const auto* second = lock.GetBody(1);
        if (!first || !second || !first->IsInBroadPhase() || !second->IsInBroadPhase())
            return std::nullopt;
        auto result = observation(request, *first, *second);
        result.mConstraintActive = mAttachment->mConstraint->IsActive();
        if (result.mConstraintActive)
            result.mImpulseOnSecond = mAttachment->mConstraint->GetTotalLambdaPosition();
        if (!validObservation(result) || !finite(result.mImpulseOnSecond))
            return std::nullopt;
        return result;
    }

    std::optional<SupportGripBreakResult> SupportGrip::releaseIfOverloaded(float maximumLinearImpulse)
    {
        if (!validLimit(maximumLinearImpulse))
            return std::nullopt;
        const auto state = observe();
        if (!state)
            return std::nullopt;
        const bool overloaded = state->mImpulseOnSecond.Length() > maximumLinearImpulse;
        if (overloaded)
            clear();
        return SupportGripBreakResult{ *state, overloaded };
    }
}
