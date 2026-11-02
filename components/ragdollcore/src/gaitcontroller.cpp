#include <ragdollcore/gaitcontroller.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace RagdollCore
{
    namespace
    {
        bool isFinite(float value)
        {
            return std::isfinite(value);
        }

        bool isFinite(JPH::RVec3Arg value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        }

        bool isFinite(JPH::Vec3Arg value)
        {
            return isFinite(value.GetX()) && isFinite(value.GetY()) && isFinite(value.GetZ());
        }

        bool isFinite(JPH::QuatArg value)
        {
            return isFinite(value.GetX()) && isFinite(value.GetY()) && isFinite(value.GetZ()) && isFinite(value.GetW());
        }

        bool validProfile(const GaitControllerProfile& profile)
        {
            return isFinite(profile.mMinimumStepDuration) && profile.mMinimumStepDuration >= 0.0f
                && isFinite(profile.mNominalStepDuration) && profile.mNominalStepDuration > 0.0f
                && profile.mNominalStepDuration >= profile.mMinimumStepDuration
                && isFinite(profile.mMaximumStepDuration)
                && profile.mMaximumStepDuration >= profile.mNominalStepDuration
                && isFinite(profile.mContactConfirmationDuration) && profile.mContactConfirmationDuration >= 0.0f
                && isFinite(profile.mStanceConfirmationDuration) && profile.mStanceConfirmationDuration >= 0.0f
                && isFinite(profile.mMaximumTouchdownHorizontalDistance)
                && profile.mMaximumTouchdownHorizontalDistance >= 0.0f
                && isFinite(profile.mMaximumLandingApproachVerticalDistance)
                && profile.mMaximumLandingApproachVerticalDistance >= 0.0f
                && isFinite(profile.mMaximumTouchdownRelativeVerticalSpeed)
                && profile.mMaximumTouchdownRelativeVerticalSpeed >= 0.0f
                && isFinite(profile.mMaximumStepReach) && profile.mMaximumStepReach > 0.0f
                && isFinite(profile.mNominalStepWidth) && profile.mNominalStepWidth >= 0.0f
                && profile.mNominalStepWidth <= 2.0f * profile.mMaximumStepReach && isFinite(profile.mCapturePointBlend)
                && profile.mCapturePointBlend >= 0.0f && profile.mCapturePointBlend <= 1.0f
                && isFinite(profile.mSwingFootLift) && profile.mSwingFootLift >= 0.0f
                && isFinite(profile.mSwingFootFrequency) && profile.mSwingFootFrequency >= 0.0f
                && isFinite(profile.mSwingFootDamping) && profile.mSwingFootDamping >= 0.0f
                && isFinite(profile.mMaximumSwingFootForce) && profile.mMaximumSwingFootForce >= 0.0f
                && isFinite(profile.mMaximumSwingFootHorizontalForce)
                && profile.mMaximumSwingFootHorizontalForce >= 0.0f
                && isFinite(profile.mMaximumSwingHipTargetSpeed) && profile.mMaximumSwingHipTargetSpeed > 0.0f
                && isFinite(profile.mSwingHipWorldStiffness) && profile.mSwingHipWorldStiffness >= 0.0f
                && isFinite(profile.mSwingHipWorldDamping) && profile.mSwingHipWorldDamping >= 0.0f
                && isFinite(profile.mMaximumSwingHipWorldTorque) && profile.mMaximumSwingHipWorldTorque >= 0.0f;
        }

        GaitPhase chooseInitialStance(const GaitControllerInput& input)
        {
            if (input.mLeftFootSupported)
                return GaitPhase::LeftStance;
            if (input.mRightFootSupported)
                return GaitPhase::RightStance;
            return GaitPhase::Idle;
        }

        bool swingFootSupported(GaitPhase phase, const GaitControllerInput& input)
        {
            if (phase == GaitPhase::LeftStance)
                return input.mRightFootSupported;
            if (phase == GaitPhase::RightStance)
                return input.mLeftFootSupported;
            return false;
        }

        bool stanceFootSupported(GaitPhase phase, const GaitControllerInput& input)
        {
            if (phase == GaitPhase::LeftStance)
                return input.mLeftFootSupported;
            if (phase == GaitPhase::RightStance)
                return input.mRightFootSupported;
            return false;
        }

        float swingFootRelativeHeight(GaitPhase phase, const GaitControllerInput& input)
        {
            if (phase == GaitPhase::LeftStance)
                return input.mRightFootHeight - input.mLeftFootHeight;
            if (phase == GaitPhase::RightStance)
                return input.mLeftFootHeight - input.mRightFootHeight;
            return 0.0f;
        }

        float swingFootRelativeVerticalVelocity(GaitPhase phase, const GaitControllerInput& input)
        {
            if (phase == GaitPhase::LeftStance)
                return input.mRightFootVerticalVelocity - input.mLeftFootVerticalVelocity;
            if (phase == GaitPhase::RightStance)
                return input.mLeftFootVerticalVelocity - input.mRightFootVerticalVelocity;
            return 0.0f;
        }

        float swingFootVerticalVelocity(GaitPhase phase, const GaitControllerInput& input)
        {
            if (phase == GaitPhase::LeftStance)
                return input.mRightFootVerticalVelocity;
            if (phase == GaitPhase::RightStance)
                return input.mLeftFootVerticalVelocity;
            return 0.0f;
        }

        GaitPhase oppositeStance(GaitPhase phase)
        {
            return phase == GaitPhase::LeftStance ? GaitPhase::RightStance : GaitPhase::LeftStance;
        }
    }

    GaitActuationStage classifyGaitActuationStage(
        bool locomotionRequested, bool pelvisHeightSettled, const GaitControllerOutput& gait)
    {
        if (!locomotionRequested)
            return GaitActuationStage::Inactive;
        if (!pelvisHeightSettled)
            return GaitActuationStage::Settling;
        if (!gait.hasStance())
            return GaitActuationStage::AwaitingStance;
        if (!gait.mStanceReady)
            return GaitActuationStage::WeightTransfer;
        if (gait.mSwingTouchdownCandidate)
            return GaitActuationStage::Touchdown;
        return gait.mSwingFootLifted ? GaitActuationStage::Swing : GaitActuationStage::Unloading;
    }

    const char* gaitActuationStageName(GaitActuationStage stage)
    {
        switch (stage)
        {
            case GaitActuationStage::Inactive: return "inactive";
            case GaitActuationStage::Settling: return "settling";
            case GaitActuationStage::AwaitingStance: return "awaiting_stance";
            case GaitActuationStage::WeightTransfer: return "weight_transfer";
            case GaitActuationStage::Unloading: return "unloading";
            case GaitActuationStage::Swing: return "swing";
            case GaitActuationStage::Touchdown: return "touchdown";
        }
        return "invalid";
    }

    std::optional<GaitActuationDecision> calculateGaitActuationDecision(
        const GaitActuationDecisionInput& input, const GaitControllerProfile& profile)
    {
        if (!validProfile(profile) || !isFinite(input.mRequestedHorizontalVelocity)
            || !isFinite(input.mMeasuredHorizontalVelocity)
            || !isFinite(profile.mPreLiftSwingUnloadFraction) || profile.mPreLiftSwingUnloadFraction < 0.0f
            || profile.mPreLiftSwingUnloadFraction > 1.0f || !isFinite(profile.mSwingLegMotorTorqueScale)
            || profile.mSwingLegMotorTorqueScale < 1.0f)
            return std::nullopt;

        JPH::Vec3 requested = input.mRequestedHorizontalVelocity;
        requested.SetZ(0.0f);
        if (input.mLocomotionRequested && requested.IsNearZero())
            return std::nullopt;

        GaitActuationDecision result;
        if (input.mReferenceCorrectiveStep)
        {
            // A stationary corrective step belongs to the reference joint-load
            // route. Do not invent travel velocity or enable legacy free axes.
            if (input.mLocomotionRequested || !requested.IsNearZero() || !input.mDriveJointMotors)
                return std::nullopt;
            result.mUseJointSupportControl = true;
            result.mLocomoting = true;
            result.mStage = classifyGaitActuationStage(true, true, input.mGait);
            result.mPhase = input.mGait.mPhase;
            return result;
        }
        result.mUseJointSupportControl = input.mDriveJointMotors && !input.mLocomotionRequested && !input.mRecovering;
        if (input.mRecovering)
        {
            result.mRequireInternalAngularReaction = true;
            return result;
        }
        result.mStage = classifyGaitActuationStage(
            input.mLocomotionRequested, input.mPelvisHeightSettled, input.mGait);
        result.mPhase = input.mGait.mPhase;
        if (!input.mLocomotionRequested)
            return result;

        if (!input.mPelvisHeightSettled)
        {
            return result;
        }

        result.mLocomoting = true;
        if (!input.mGait.hasStance())
        {
            if (input.mCorrectTranslation)
                result.mTargetHorizontalVelocity = requested;
            return result;
        }

        result.mFreeHipWorldAxis = JPH::Vec3::sAxisZ().Cross(requested.Normalized());
        result.mUseSupportedFootWorldMotors = input.mDriveJointMotors;
        result.mReleaseStanceHipAxis = input.mDriveJointMotors && input.mStanceFootSupported;
        result.mReleaseSwingHipAxis = input.mDriveJointMotors && input.mBothFeetSupported
            && (!input.mGait.mStanceReady || input.mGait.mSwingTouchdownCandidate);
        result.mUseStanceSupportContacts = input.mGait.mSwingFootLifted;
        result.mSwingReactionThroughStanceFoot = !input.mGait.mSwingFootLifted;
        result.mSwingFootInFlight = input.mGait.mSwingFootLifted && input.mGait.mSwingContactCandidateTime <= 0.0f;
        result.mUseStanceVerticalReaction = input.mStanceFootSupported && input.mSwingTargetAvailable
            && input.mGait.mStanceReady;
        result.mUseStanceHorizontalReaction = input.mStanceFootSupported && input.mSwingTargetAvailable
            && input.mGait.mSwingFootLifted;
        if (!input.mGait.mStanceReady)
        {
            result.mPoseSource = GaitPoseSource::MeasuredSwingLeg;
            result.mHorizontalControl = GaitHorizontalControl::LateralStance;
            if (input.mCorrectTranslation)
            {
                // During transfer, preserve earned forward motion while the
                // lateral capture state becomes ready. The request remains
                // the speed ceiling; forward acceleration resumes at readiness.
                const JPH::Vec3 forward = requested.Normalized();
                const float speed = std::clamp(input.mMeasuredHorizontalVelocity.Dot(forward),
                    0.0f, requested.Length());
                result.mTargetHorizontalVelocity = forward * speed;
            }
            result.mApplyPreLiftSwingUnload = input.mDriveJointMotors && input.mBothFeetSupported
                && profile.mPreLiftSwingUnloadFraction > 0.0f;
            return result;
        }

        result.mHorizontalControl = GaitHorizontalControl::LateralStance;
        result.mSwingLegMotorTorqueScale = profile.mSwingLegMotorTorqueScale;
        if (input.mCorrectTranslation)
            result.mTargetHorizontalVelocity = requested;
        // Approach detection can precede contact. Keep commanding the landing
        // pose until measured support exists; holding the current leg while
        // still airborne can suspend it above the floor indefinitely.
        if (input.mGait.mSwingTouchdownCandidate
            && (input.mBothFeetSupported || input.mGait.mSwingContactCandidateTime > 0.0f))
        {
            result.mPoseSource = GaitPoseSource::MeasuredTouchdown;
        }
        else
        {
            result.mPoseSource = GaitPoseSource::GaitCycle;
        }
        return result;
    }

    std::optional<JointSupportOutput> calculateStandingJointSupport(const RagdollTelemetry& telemetry,
        std::span<const GroundContact> contacts, FootBodyIndices feet, float gravity, float friction,
        const ActiveControllerProfile& profile, const std::optional<SupportMotionReference>& motion)
    {
        if (!isFinite(telemetry.mTotalMass) || telemetry.mTotalMass <= 0
            || !isFinite(profile.mCenterOfMassFrequency) || profile.mCenterOfMassFrequency < 0
            || !isFinite(profile.mCenterOfMassDamping) || profile.mCenterOfMassDamping < 0
            || (motion && (!isFinite(motion->mPosition) || !isFinite(motion->mVelocity) || !isFinite(motion->mAcceleration))))
            return std::nullopt;
        std::vector<GroundContact> selectedContacts;
        if (motion && motion->mSupportFoot)
        {
            if (*motion->mSupportFoot != feet.mLeft && *motion->mSupportFoot != feet.mRight)
                return std::nullopt;
            for (const auto& contact : contacts)
                if (contact.mBodyIndex == *motion->mSupportFoot)
                    selectedContacts.push_back(contact);
            contacts = selectedContacts;
        }
        const auto support = estimateFootSupport(telemetry.mCenterOfMass, contacts, feet, profile);
        if (!support)
            return std::nullopt;
        std::vector<SupportJointBody> bodies;
        JPH::Vec3 velocity = JPH::Vec3::sZero();
        for (const auto& body : telemetry.mBodies)
        {
            bodies.push_back({ -1, body.mCenterOfMass, body.mPosition, body.mMass });
            velocity += body.mMotion.mLinearVelocity * (body.mMass / telemetry.mTotalMass);
        }
        for (const auto& joint : telemetry.mConstraints)
        {
            if (joint.mChildBodyIndex == 0 || joint.mChildBodyIndex >= bodies.size()
                || joint.mParentBodyIndex >= joint.mChildBodyIndex
                || bodies[joint.mChildBodyIndex].mParent != -1)
                return std::nullopt;
            bodies[joint.mChildBodyIndex].mParent = static_cast<int>(joint.mParentBodyIndex);
            bodies[joint.mChildBodyIndex].mJointPosition = joint.mPosition;
        }
        auto offset = JPH::Vec3(telemetry.mCenterOfMass - (motion ? motion->mPosition : support->mCenter));
        offset.SetZ(0);
        if (motion)
            velocity -= motion->mVelocity;
        velocity.SetZ(0);
        const float omega = 2.0f * JPH::JPH_PI * profile.mCenterOfMassFrequency;
        JointSupportInput input;
        input.mBodies = bodies;
        input.mContacts = contacts;
        input.mFeet = feet;
        input.mGravity = gravity;
        input.mFriction = support->mFriction.value_or(friction);
        input.mRequestedAcceleration = -offset * (omega * omega)
            - velocity * (2.0f * omega * profile.mCenterOfMassDamping);
        if (motion)
            input.mRequestedAcceleration += JPH::Vec3(motion->mAcceleration.GetX(), motion->mAcceleration.GetY(), 0);
        return calculateJointSupportControl(input, profile);
    }

    std::optional<float> calculateMeasuredFootNormalLoadFraction(
        std::span<const GroundContact> contacts, std::size_t footBody,
        float actorMass, float gravity, float duration)
    {
        if (!isFinite(actorMass) || actorMass <= 0.0f || !isFinite(gravity) || gravity <= 0.0f
            || !isFinite(duration) || duration <= 0.0f)
            return std::nullopt;
        float impulse = 0.0f;
        for (const auto& contact : contacts)
        {
            if (contact.mBodyIndex != footBody || !contact.hasSupport() || !contact.mNormalImpulse)
                continue;
            if (!isFinite(*contact.mNormalImpulse) || *contact.mNormalImpulse < 0.0f)
                return std::nullopt;
            impulse += *contact.mNormalImpulse;
        }
        const float fraction = impulse / (actorMass * gravity * duration);
        return isFinite(fraction) ? std::optional<float>(fraction) : std::nullopt;
    }

    std::optional<JointSupportOutput> blendJointSupportOutputs(
        const JointSupportOutput& bilateral, const JointSupportOutput& unilateral, float unilateralWeight)
    {
        if (!bilateral.mSupported || !unilateral.mSupported || !isFinite(unilateralWeight)
            || unilateralWeight < 0.0f || unilateralWeight > 1.0f
            || bilateral.mJointTorques.size() != unilateral.mJointTorques.size())
            return std::nullopt;
        const float bilateralWeight = 1.0f - unilateralWeight;
        const auto blend = [&](JPH::Vec3Arg first, JPH::Vec3Arg second) {
            return first * bilateralWeight + second * unilateralWeight;
        };
        if (!isFinite(bilateral.mAcceleration) || !isFinite(unilateral.mAcceleration)
            || !isFinite(bilateral.mCenterOfPressure) || !isFinite(unilateral.mCenterOfPressure)
            || !isFinite(bilateral.mRootForceResidual) || !isFinite(unilateral.mRootForceResidual)
            || !isFinite(bilateral.mRootMomentResidual) || !isFinite(unilateral.mRootMomentResidual))
            return std::nullopt;
        JointSupportOutput result;
        result.mSupported = true;
        result.mFrictionLimited = bilateral.mFrictionLimited || unilateral.mFrictionLimited;
        result.mLegAxisApplied = bilateral.mLegAxisApplied || unilateral.mLegAxisApplied;
        result.mAcceleration = blend(bilateral.mAcceleration, unilateral.mAcceleration);
        result.mCenterOfPressure = bilateral.mCenterOfPressure * bilateralWeight
            + unilateral.mCenterOfPressure * unilateralWeight;
        result.mRootForceResidual = blend(bilateral.mRootForceResidual, unilateral.mRootForceResidual);
        result.mRootMomentResidual = blend(bilateral.mRootMomentResidual, unilateral.mRootMomentResidual);
        if (bilateral.mHullProjectionDelta && unilateral.mHullProjectionDelta)
            result.mHullProjectionDelta = blend(*bilateral.mHullProjectionDelta, *unilateral.mHullProjectionDelta);
        else if (unilateralWeight <= 0.0f)
            result.mHullProjectionDelta = bilateral.mHullProjectionDelta;
        else if (unilateralWeight >= 1.0f)
            result.mHullProjectionDelta = unilateral.mHullProjectionDelta;
        result.mJointTorques.reserve(bilateral.mJointTorques.size());
        for (std::size_t i = 0; i < bilateral.mJointTorques.size(); ++i)
        {
            if (!isFinite(bilateral.mJointTorques[i]) || !isFinite(unilateral.mJointTorques[i]))
                return std::nullopt;
            result.mJointTorques.push_back(blend(bilateral.mJointTorques[i], unilateral.mJointTorques[i]));
        }
        result.mContactForces.reserve(
            bilateral.mContactForces.size() + unilateral.mContactForces.size());
        for (const auto& contact : bilateral.mContactForces)
        {
            if (!isFinite(contact.mForce))
                return std::nullopt;
            auto weighted = contact;
            weighted.mForce *= bilateralWeight;
            result.mContactForces.push_back(weighted);
        }
        for (const auto& contact : unilateral.mContactForces)
        {
            if (!isFinite(contact.mForce))
                return std::nullopt;
            auto weighted = contact;
            weighted.mForce *= unilateralWeight;
            result.mContactForces.push_back(weighted);
        }
        return result;
    }

    std::optional<JPH::Vec3> selectGaitCenterOfMassOffset(const GaitActuationDecision& decision,
        JPH::Vec3Arg requestedHorizontalVelocity, JPH::Vec3Arg defaultSupportOffset,
        const std::optional<JPH::Vec3>& stanceOffset)
    {
        if (!isFinite(requestedHorizontalVelocity) || !isFinite(defaultSupportOffset)
            || (stanceOffset && !isFinite(*stanceOffset)))
            return std::nullopt;

        if (decision.mHorizontalControl == GaitHorizontalControl::DefaultSupport || !stanceOffset)
            return defaultSupportOffset;
        if (decision.mHorizontalControl == GaitHorizontalControl::FullStance)
            return *stanceOffset;

        JPH::Vec3 forward = requestedHorizontalVelocity;
        forward.SetZ(0.0f);
        if (forward.IsNearZero())
            return std::nullopt;
        forward = forward.Normalized();
        const JPH::Vec3 lateral = JPH::Vec3::sAxisZ().Cross(forward).Normalized();
        return lateral * stanceOffset->Dot(lateral);
    }

    std::optional<GaitStanceSupport> estimateGaitStanceSupport(GaitPhase phase,
        JPH::RVec3Arg centerOfMass, JPH::Vec3Arg velocity, JPH::Vec3Arg requestedVelocity,
        float height, float gravity, std::span<const GroundContact> contacts,
        FootBodyIndices feet, const ActiveControllerProfile& profile, bool useCapturePoint,
        const std::optional<GaitCaptureStepInput>& captureStep)
    {
        if ((phase != GaitPhase::Idle && phase != GaitPhase::LeftStance && phase != GaitPhase::RightStance)
            || !validProfile(profile.mGait) || !isFinite(centerOfMass) || !isFinite(velocity) || !isFinite(requestedVelocity)
            || (captureStep && (!isFinite(captureStep->mStanceFootPosition)
                || !isFinite(captureStep->mTimeInPhase) || captureStep->mTimeInPhase < 0.0f)))
            return std::nullopt;
        GaitStanceSupport result;
        if (phase == GaitPhase::Idle)
            return result;
        const std::size_t stanceFoot = phase == GaitPhase::LeftStance ? feet.mLeft : feet.mRight;
        std::vector<GroundContact> stanceContacts;
        for (const GroundContact& contact : contacts)
            if (contact.mBodyIndex == stanceFoot)
                stanceContacts.push_back(contact);
        const auto support = estimateFootSupport(centerOfMass, stanceContacts, feet, profile);
        if (!support)
            return std::nullopt;
        result.mSupport = *support;
        if (!support->hasSupport())
            return result;
        JPH::RVec3 readinessPoint = centerOfMass;
        if (useCapturePoint)
        {
            const auto capture = calculateCapturePoint({ centerOfMass, velocity, height, gravity });
            if (!capture)
                return result;
            JPH::Vec3 forward = requestedVelocity;
            forward.SetZ(0.0f);
            if (!forward.IsNearZero())
            {
                forward = forward.Normalized();
                const JPH::Vec3 lateral = JPH::Vec3::sAxisZ().Cross(forward);
                float minimumLateral = std::numeric_limits<float>::max();
                float maximumLateral = std::numeric_limits<float>::lowest();
                float minimumForward = std::numeric_limits<float>::max();
                float maximumForward = std::numeric_limits<float>::lowest();
                for (const GroundContact& contact : stanceContacts)
                {
                    if (!contact.hasSupport()
                        || contact.mNormal.Normalized().GetZ() < profile.mSupportNormalMinimumDot)
                        continue;
                    const JPH::Vec3 relative(contact.mPosition - centerOfMass);
                    minimumLateral = std::min(minimumLateral, relative.Dot(lateral));
                    maximumLateral = std::max(maximumLateral, relative.Dot(lateral));
                    minimumForward = std::min(minimumForward, relative.Dot(forward));
                    maximumForward = std::max(maximumForward, relative.Dot(forward));
                }
                const JPH::Vec3 captureOffset(*capture - centerOfMass);
                const float lateralCapture = captureOffset.Dot(lateral);
                const float forwardCapture = captureOffset.Dot(forward);
                // Lateral capture must lie strictly within measured support.
                // Forward motion is caught by the next step, whose authored
                // reach bounds the admissible extrapolation. A backward escape
                // cannot be justified by a forward step request.
                result.mReady = lateralCapture >= minimumLateral && lateralCapture <= maximumLateral
                    && forwardCapture >= minimumForward
                    && forwardCapture <= maximumForward + profile.mGait.mMaximumStepReach;
                if (captureStep && lateralCapture >= minimumLateral && lateralCapture <= maximumLateral
                    && forwardCapture < minimumForward)
                {
                    // Assume lateral capture is held by a CoP on its measured hull
                    // section. The rear endpoint minimizes backward divergence.
                    // All chords lie in the convex support hull; their extrema
                    // give its exact intersection without assuming a rectangle.
                    float rear = std::numeric_limits<float>::max();
                    for (const GroundContact& a : stanceContacts)
                    {
                        if (!a.hasSupport()
                            || a.mNormal.Normalized().GetZ() < profile.mSupportNormalMinimumDot)
                            continue;
                        const JPH::Vec3 pa(a.mPosition - centerOfMass);
                        const float la = pa.Dot(lateral);
                        if (la == lateralCapture)
                            rear = std::min(rear, pa.Dot(forward));
                        for (const GroundContact& b : stanceContacts)
                        {
                            if (!b.hasSupport()
                                || b.mNormal.Normalized().GetZ() < profile.mSupportNormalMinimumDot)
                                continue;
                            const JPH::Vec3 pb(b.mPosition - centerOfMass);
                            const float lb = pb.Dot(lateral);
                            if ((la < lateralCapture && lb > lateralCapture)
                                || (la > lateralCapture && lb < lateralCapture))
                            {
                                const float t = (lateralCapture - la) / (lb - la);
                                rear = std::min(rear, pa.Dot(forward) + t * (pb - pa).Dot(forward));
                            }
                        }
                    }
                    if (rear != std::numeric_limits<float>::max())
                    {
                        const float remainingLift = std::max(0.0f,
                            0.5f * profile.mGait.mNominalStepDuration - captureStep->mTimeInPhase);
                        const float horizon = remainingLift + profile.mGait.mNominalStepDuration
                            + profile.mGait.mContactConfirmationDuration + profile.mGait.mStanceConfirmationDuration;
                        // xi_dot = omega * (xi - CoP), under the same LIPM
                        // assumptions as capture readiness. Do not silently
                        // clamp an unreachable recovery into an accepted step.
                        const float predicted = rear + (forwardCapture - rear)
                            * std::exp(std::sqrt(gravity / height) * horizon);
                        const JPH::Vec3 stanceOffset(captureStep->mStanceFootPosition - centerOfMass);
                        const float side = phase == GaitPhase::LeftStance ? -1.0f : 1.0f;
                        JPH::RVec3 target = centerOfMass + forward * predicted
                            + lateral * (stanceOffset.Dot(lateral) + side * profile.mGait.mNominalStepWidth);
                        target.SetZ(captureStep->mStanceFootPosition.GetZ());
                        JPH::Vec3 reach(target - captureStep->mStanceFootPosition);
                        reach.SetZ(0.0f);
                        if (isFinite(target) && reach.LengthSq()
                            <= profile.mGait.mMaximumStepReach * profile.mGait.mMaximumStepReach)
                        {
                            result.mBackwardRecoveryTarget = target;
                            result.mReady = true;
                        }
                    }
                }
                return result;
            }
            else
                readinessPoint = *capture;
        }
        const auto readiness = estimateFootSupport(readinessPoint, stanceContacts, feet, profile);
        result.mReady = readiness && readiness->hasSupport() && readiness->mCenterOfMassDistance <= 0.0f;
        return result;
    }

    void configureGaitSupportBudget(const GaitActuationDecision& decision, bool correctTranslation,
        std::span<const GroundContact> contacts, FootBodyIndices feet, UprightControllerInput& upright)
    {
        upright.mLimitToSupportFriction = decision.mLocomoting && correctTranslation;
        upright.mSupportContactPoints.clear();
        const std::size_t stanceFoot = decision.mPhase == GaitPhase::LeftStance ? feet.mLeft : feet.mRight;
        for (const GroundContact& contact : contacts)
        {
            const bool foot = contact.mBodyIndex == feet.mLeft || contact.mBodyIndex == feet.mRight;
            if (foot && (!decision.mUseStanceSupportContacts || contact.mBodyIndex == stanceFoot)
                && contact.hasSupport())
                upright.mSupportContactPoints.push_back(contact.mPosition);
        }
        upright.mLimitToSupportMoment = upright.mLimitToSupportFriction
            && upright.mCenterOfMassHeight > 1e-5f && !upright.mSupportContactPoints.empty();
    }

    std::vector<std::size_t> selectGaitWorldFootMotors(const GaitActuationDecision& decision,
        const SupportEstimate& support, FootBodyIndices feet)
    {
        std::vector<std::size_t> result;
        if (!decision.mUseSupportedFootWorldMotors)
            return result;
        if (support.mLeftFootSupported)
            result.push_back(feet.mLeft);
        if (support.mRightFootSupported)
            result.push_back(feet.mRight);
        return result;
    }

    GaitReactionBodies selectGaitReactionBodies(const GaitActuationDecision& decision,
        const SupportEstimate& support, FootBodyIndices feet, std::size_t leftHip, std::size_t rightHip,
        bool settlingRecovery)
    {
        GaitReactionBodies result;
        if (support.mLeftFootSupported)
            result.mVertical.push_back(feet.mLeft);
        if (support.mRightFootSupported)
            result.mVertical.push_back(feet.mRight);
        result.mHorizontal = result.mVertical;
        const bool leftStance = decision.mPhase == GaitPhase::LeftStance;
        const bool supported = leftStance ? support.mLeftFootSupported : support.mRightFootSupported;
        // An internal torque reaction does not require ground contact. Losing
        // contact must never turn orientation control into an external torque.
        if (decision.mPhase != GaitPhase::Idle)
            result.mAngular = leftStance ? leftHip : rightHip;
        else if (decision.mRequireInternalAngularReaction)
        {
            if (settlingRecovery && support.hasSupport())
            {
                result.mAngular = support.mLeftFootSupported ? feet.mLeft : feet.mRight;
                if (support.mLeftFootSupported && support.mRightFootSupported)
                    result.mSecondaryAngular = feet.mRight;
            }
            else
            {
                result.mAngular = support.mRightFootSupported && !support.mLeftFootSupported ? rightHip : leftHip;
                if (support.mLeftFootSupported && support.mRightFootSupported)
                    result.mSecondaryAngular = rightHip;
            }
        }
        if (decision.mPhase != GaitPhase::Idle && supported)
        {
            if (decision.mUseStanceVerticalReaction)
            {
                result.mVertical = { leftStance ? feet.mLeft : feet.mRight };
                result.mAngular = leftStance ? leftHip : rightHip;
            }
            if (decision.mUseStanceHorizontalReaction)
                result.mHorizontal = { leftStance ? leftHip : rightHip };
        }
        return result;
    }

    std::optional<float> calculatePreLiftSwingUnloadForce(const GaitActuationDecision& decision,
        float totalMass, float swingLegMass, float gravity, const GaitControllerProfile& profile)
    {
        if (!isFinite(totalMass) || totalMass <= 0.0f || !isFinite(swingLegMass) || swingLegMass <= 0.0f
            || swingLegMass > totalMass || !isFinite(gravity) || gravity <= 0.0f
            || !isFinite(profile.mPreLiftSwingUnloadFraction) || profile.mPreLiftSwingUnloadFraction < 0.0f
            || profile.mPreLiftSwingUnloadFraction > 1.0f)
            return std::nullopt;
        const float weight = totalMass * gravity;
        if (!isFinite(weight))
            return std::nullopt;
        return decision.mApplyPreLiftSwingUnload
            ? std::min(weight, profile.mPreLiftSwingUnloadFraction * swingLegMass * gravity) : 0.0f;
    }

    bool GaitPhaseController::step(const GaitControllerInput& input, float duration,
        const GaitControllerProfile& profile, GaitControllerOutput& output)
    {
        if (!isFinite(duration) || duration <= 0.0f || !validProfile(profile) || !isFinite(input.mLeftFootHeight)
            || !isFinite(input.mRightFootHeight) || !isFinite(input.mLeftFootVerticalVelocity)
            || !isFinite(input.mRightFootVerticalVelocity) || !isFinite(input.mSwingFootHorizontalPlacementError)
            || input.mSwingFootHorizontalPlacementError < 0.0f
            || !isFinite(input.mSwingFootVerticalPlacementError) || input.mSwingFootVerticalPlacementError < 0.0f
            || !isFinite(input.mSwingFootRelativeVerticalSpeed) || input.mSwingFootRelativeVerticalSpeed < 0.0f)
            return false;

        bool transitioned = false;
        if (!input.mEnabled)
        {
            reset();
            output = getOutput();
            return true;
        }

        if (mPhase == GaitPhase::Idle)
        {
            mPhase = chooseInitialStance(input);
            mTimeInPhase = 0.0f;
            mSwingContactCandidateTime = 0.0f;
            mPhaseProgress = 0.0f;
            mSwingFootStartRelativeHeight = swingFootRelativeHeight(mPhase, input);
            mStanceSupportCandidateTime = 0.0f;
            mStanceSupportLossCandidateTime = 0.0f;
            mStanceReady = false;
            mSwingFootLifted = false;
            mSwingTouchdownCandidate = false;
            mPhaseTimedOut = false;
            transitioned = mPhase != GaitPhase::Idle;
        }
        else
        {
            const bool swingSupported = swingFootSupported(mPhase, input);
            const float minimumClearance = std::max(1e-3f, 0.1f * profile.mSwingFootLift);
            const float relativeClearance = swingFootRelativeHeight(mPhase, input) - mSwingFootStartRelativeHeight;
            const bool measuredLift = !swingSupported && relativeClearance >= minimumClearance
                && swingFootRelativeVerticalVelocity(mPhase, input) > 0.0f
                && swingFootVerticalVelocity(mPhase, input) > 0.0f;
            // Consume the current physical observation before deciding that
            // an armed step is still planted. Capture loss in the same solve
            // must not retroactively cancel an already observed lift-off.
            if (mStanceReady && measuredLift)
                mSwingFootLifted = true;
            // Hold the beginning of the stance while the balance controller
            // transfers weight onto it. Phase time owns both the sampled walk
            // pose and the swing trajectory, so freezing it here prevents
            // either system from trying to lift an unloaded foot prematurely.
            if (!mSwingFootLifted)
            {
                const bool stanceSupported = stanceFootSupported(mPhase, input);
                const bool validStance = stanceSupported && input.mStanceSupportsCenterOfMass;
                if (mStanceReady)
                {
                    // The shared observer already permits reachable forward
                    // progress outside the static hull. Do not also ignore
                    // measured lateral escape or unreachable capture while
                    // the other foot has not lifted. Missing contacts retain
                    // their existing grace; observed invalid geometry does not.
                    if (stanceSupported && !input.mStanceSupportsCenterOfMass)
                    {
                        mStanceSupportCandidateTime = 0.0f;
                        mStanceSupportLossCandidateTime = 0.0f;
                        mStanceReady = false;
                    }
                    else if (stanceSupported)
                        mStanceSupportLossCandidateTime = 0.0f;
                    else
                    {
                        mStanceSupportLossCandidateTime += duration;
                        if (mStanceSupportLossCandidateTime >= profile.mStanceConfirmationDuration)
                        {
                            mStanceSupportCandidateTime = 0.0f;
                            mStanceSupportLossCandidateTime = 0.0f;
                            mStanceReady = false;
                        }
                    }
                }
                else if (validStance)
                {
                    mStanceSupportLossCandidateTime = 0.0f;
                    mStanceSupportCandidateTime += duration;
                    mStanceReady = mStanceSupportCandidateTime >= profile.mStanceConfirmationDuration;
                }
                else
                {
                    mStanceSupportCandidateTime = 0.0f;
                    mStanceSupportLossCandidateTime = 0.0f;
                }
            }
            if (!mSwingFootLifted && !mStanceReady)
            {
                mTimeInPhase = 0.0f;
                mSwingContactCandidateTime = 0.0f;
                mSwingTimeSinceLift = 0.0f;
                mPhaseProgress = 0.0f;
                mSwingTouchdownCandidate = false;
                mPhaseTimedOut = false;
                output = getOutput();
                return true;
            }

            mTimeInPhase += duration;
            if (!swingSupported)
            {
                if (measuredLift)
                    mSwingFootLifted = true;
                if (!mSwingFootLifted)
                    mSwingTimeSinceLift = 0.0f;
                else
                    mSwingTimeSinceLift += duration;
                mSwingContactCandidateTime = 0.0f;
            }
            else if (mSwingFootLifted)
            {
                mSwingTimeSinceLift += duration;
                mSwingContactCandidateTime += duration;
            }
            else
                mSwingContactCandidateTime = 0.0f;

            const bool descendingTowardSupport = swingFootRelativeVerticalVelocity(mPhase, input) <= 0.0f
                && swingFootVerticalVelocity(mPhase, input) <= 0.0f;
            mSwingTouchdownCandidate = mSwingFootLifted
                && input.mSwingFootHorizontalPlacementError <= profile.mMaximumTouchdownHorizontalDistance
                && (swingSupported
                    || (descendingTowardSupport
                        && input.mSwingFootVerticalPlacementError
                            <= profile.mMaximumLandingApproachVerticalDistance));

            if (mTimeInPhase >= profile.mMinimumStepDuration && mSwingFootLifted && swingSupported
                && mSwingContactCandidateTime >= profile.mContactConfirmationDuration
                && input.mSwingFootHorizontalPlacementError <= profile.mMaximumTouchdownHorizontalDistance
                && input.mSwingFootRelativeVerticalSpeed <= profile.mMaximumTouchdownRelativeVerticalSpeed)
            {
                mPhase = oppositeStance(mPhase);
                mTimeInPhase = 0.0f;
                mSwingContactCandidateTime = 0.0f;
                mSwingTimeSinceLift = 0.0f;
                mSwingFootStartRelativeHeight = swingFootRelativeHeight(mPhase, input);
                mStanceSupportCandidateTime = 0.0f;
                mStanceSupportLossCandidateTime = 0.0f;
                mStanceReady = false;
                mSwingFootLifted = false;
                mSwingTouchdownCandidate = false;
                transitioned = true;
            }
            mPhaseProgress = std::clamp(mTimeInPhase / profile.mNominalStepDuration, 0.0f, 1.0f);
            // Bound both failure modes independently. A foot that never
            // lifts must still time out, but a legitimate delayed lift must
            // receive a complete swing interval instead of inheriting the
            // time already spent unloading it.
            mPhaseTimedOut = mSwingFootLifted ? mSwingTimeSinceLift >= profile.mMaximumStepDuration
                                              : mTimeInPhase >= profile.mMaximumStepDuration;
        }

        output = getOutput();
        output.mTransitioned = transitioned;
        return true;
    }

    void GaitPhaseController::reset()
    {
        mPhase = GaitPhase::Idle;
        mTimeInPhase = 0.0f;
        mSwingContactCandidateTime = 0.0f;
        mSwingTimeSinceLift = 0.0f;
        mPhaseProgress = 0.0f;
        mSwingFootStartRelativeHeight = 0.0f;
        mStanceSupportCandidateTime = 0.0f;
        mStanceSupportLossCandidateTime = 0.0f;
        mStanceReady = false;
        mSwingFootLifted = false;
        mSwingTouchdownCandidate = false;
        mPhaseTimedOut = false;
    }

    GaitControllerOutput GaitPhaseController::getOutput() const
    {
        GaitControllerOutput result;
        result.mPhase = mPhase;
        result.mTimeInPhase = mTimeInPhase;
        result.mPhaseProgress = mPhaseProgress;
        result.mSwingContactCandidateTime = mSwingContactCandidateTime;
        result.mSwingTimeSinceLift = mSwingTimeSinceLift;
        result.mStanceReady = mStanceReady;
        result.mSwingFootLifted = mSwingFootLifted;
        result.mSwingTouchdownCandidate = mSwingTouchdownCandidate;
        result.mPhaseTimedOut = mPhaseTimedOut;
        return result;
    }

    std::optional<float> calculateGaitCyclePhase(const GaitControllerOutput& gait)
    {
        if (!gait.hasStance() || !isFinite(gait.mPhaseProgress) || gait.mPhaseProgress < 0.0f
            || gait.mPhaseProgress > 1.0f)
            return std::nullopt;
        const float stanceOffset = gait.rightFootIsStance() ? 0.5f : 0.0f;
        return stanceOffset + 0.5f * gait.mPhaseProgress;
    }

    bool sampleGaitPoseCycle(std::span<const RagdollPose> samples, const GaitControllerOutput& gait, RagdollPose& out)
    {
        const auto phase = calculateGaitCyclePhase(gait);
        if (!phase || samples.size() < 2)
            return false;
        const std::size_t bodyCount = samples.front().mBodyTransforms.size();
        if (bodyCount == 0)
            return false;
        for (const RagdollPose& sample : samples)
        {
            if (sample.mBodyTransforms.size() != bodyCount)
                return false;
        }

        const float samplePosition = *phase * static_cast<float>(samples.size());
        const float sampleFloor = std::floor(samplePosition);
        const std::size_t from = static_cast<std::size_t>(sampleFloor) % samples.size();
        const std::size_t to = (from + 1) % samples.size();
        const float factor = samplePosition - sampleFloor;
        return interpolateRagdollPoses(samples[from], samples[to], factor, out);
    }

    std::optional<JPH::RVec3> calculateCapturePoint(const CapturePointInput& input)
    {
        if (!isFinite(input.mCenterOfMassPosition) || !isFinite(input.mCenterOfMassVelocity)
            || !isFinite(input.mCenterOfMassHeight) || input.mCenterOfMassHeight <= 0.0f || !isFinite(input.mGravity)
            || input.mGravity <= 0.0f)
            return std::nullopt;

        JPH::Vec3 horizontalVelocity = input.mCenterOfMassVelocity;
        horizontalVelocity.SetZ(0.0f);
        const float timeConstant = std::sqrt(input.mCenterOfMassHeight / input.mGravity);
        const JPH::RVec3 result = input.mCenterOfMassPosition + horizontalVelocity * timeConstant;
        if (!isFinite(result))
            return std::nullopt;
        return result;
    }

    std::optional<JPH::RVec3> calculateSwingFootTarget(
        const SwingFootTargetInput& input, const GaitControllerProfile& profile)
    {
        if ((input.mPhase != GaitPhase::LeftStance && input.mPhase != GaitPhase::RightStance) || !validProfile(profile)
            || !isFinite(input.mCenterOfMassPosition) || !isFinite(input.mCenterOfMassVelocity)
            || !isFinite(input.mTargetHorizontalVelocity) || !isFinite(input.mStanceFootPosition)
            || !isFinite(input.mForward) || std::abs(input.mForward.GetZ()) > 1e-4f)
            return std::nullopt;

        JPH::Vec3 forward = input.mForward;
        const float forwardLengthSquared = forward.LengthSq();
        if (!isFinite(forwardLengthSquared) || forwardLengthSquared <= std::numeric_limits<float>::epsilon())
            return std::nullopt;
        forward /= std::sqrt(forwardLengthSquared);
        const JPH::Vec3 right = forward.Cross(JPH::Vec3::sAxisZ());

        const auto capturePoint = calculateCapturePoint(
            { input.mCenterOfMassPosition, input.mCenterOfMassVelocity, input.mCenterOfMassHeight, input.mGravity });
        if (!capturePoint)
            return std::nullopt;

        JPH::Vec3 targetVelocity = input.mTargetHorizontalVelocity;
        targetVelocity.SetZ(0.0f);
        JPH::RVec3 nominal
            = input.mCenterOfMassPosition + targetVelocity * (0.5f * profile.mNominalStepDuration);
        // Step width is the desired distance between the two foot centers,
        // not the distance from the COM to either foot. During deliberate
        // weight transfer the COM approaches the stance foot, so centering
        // the lateral target on the COM can command the swing foot across the
        // stance leg. Anchor the lateral coordinate to the measured stance
        // foot and preserve the COM-derived coordinate only along forward.
        const float stanceLateral = JPH::Vec3(input.mStanceFootPosition).Dot(right);
        const float nominalLateral = stanceLateral
            + (input.mPhase == GaitPhase::LeftStance ? 1.0f : -1.0f) * profile.mNominalStepWidth;
        nominal += right * (nominalLateral - JPH::Vec3(nominal).Dot(right));
        // Lateral capture feedback is useful during an external shove, but it
        // must not pull the swing leg across the stance leg during ordinary
        // weight transfer. Accept only the outward half of the correction.
        const JPH::Vec3 captureCorrection = JPH::Vec3(*capturePoint - nominal);
        const float lateralCorrection = captureCorrection.Dot(right);
        const float swingSide = input.mPhase == GaitPhase::LeftStance ? 1.0f : -1.0f;
        const float outwardCorrection = std::max(0.0f, swingSide * lateralCorrection);
        JPH::RVec3 target = nominal + forward * (captureCorrection.Dot(forward) * profile.mCapturePointBlend)
            + right * (swingSide * outwardCorrection * profile.mCapturePointBlend);
        target.SetZ(input.mStanceFootPosition.GetZ());

        JPH::Vec3 offset = JPH::Vec3(target - input.mStanceFootPosition);
        offset.SetZ(0.0f);
        const float distanceSquared = offset.LengthSq();
        if (!isFinite(distanceSquared))
            return std::nullopt;
        if (distanceSquared > profile.mMaximumStepReach * profile.mMaximumStepReach)
        {
            offset *= profile.mMaximumStepReach / std::sqrt(distanceSquared);
            target.SetX(input.mStanceFootPosition.GetX() + offset.GetX());
            target.SetY(input.mStanceFootPosition.GetY() + offset.GetY());
        }
        return target;
    }

    std::optional<GaitSwingPlacement> selectGaitSwingPlacement(const GaitControllerOutput& gait,
        const SwingFootTargetInput& input, const std::optional<JPH::RVec3>& backwardRecoveryTarget,
        const GaitSwingPlacement& previous, const GaitControllerProfile& profile)
    {
        if (!validProfile(profile) || (previous.mTarget && !isFinite(*previous.mTarget))
            || (previous.mBackwardRecovery && !previous.mTarget)
            || (backwardRecoveryTarget && !isFinite(*backwardRecoveryTarget)))
            return std::nullopt;
        if (!gait.hasStance())
            return GaitSwingPlacement{};
        if (input.mPhase != gait.mPhase)
            return std::nullopt;
        GaitSwingPlacement result = gait.mTransitioned ? GaitSwingPlacement{} : previous;
        if (gait.mSwingFootLifted)
            return result.mTarget ? std::optional<GaitSwingPlacement>(result) : std::nullopt;
        if (backwardRecoveryTarget && !gait.mTransitioned)
            return GaitSwingPlacement{ backwardRecoveryTarget, true };
        if (result.mBackwardRecovery)
            return result;
        result.mTarget = calculateSwingFootTarget(input, profile);
        return result.mTarget ? std::optional<GaitSwingPlacement>(result) : std::nullopt;
    }

    std::optional<SwingFootTrajectoryOutput> calculateSwingFootTrajectoryState(
        const SwingFootTrajectoryInput& input, const GaitControllerProfile& profile)
    {
        if (!validProfile(profile) || !input.mGait.hasStance() || !isFinite(input.mLiftOffPosition)
            || !isFinite(input.mPlacementPosition) || !isFinite(input.mGait.mTimeInPhase)
            || input.mGait.mTimeInPhase < 0.0f || !isFinite(input.mGait.mSwingTimeSinceLift)
            || input.mGait.mSwingTimeSinceLift < 0.0f
            || (!input.mGait.mSwingFootLifted && input.mGait.mSwingTimeSinceLift != 0.0f)
            || (input.mGait.mSwingTouchdownCandidate && !input.mGait.mSwingFootLifted)
            || input.mGait.mSwingTimeSinceLift > input.mGait.mTimeInPhase + 1e-6f)
            return std::nullopt;

        const float halfDuration = 0.5f * profile.mNominalStepDuration;
        const auto smoothStep = [](float value) {
            const float clamped = std::clamp(value, 0.0f, 1.0f);
            return clamped * clamped * (3.0f - 2.0f * clamped);
        };
        const auto smoothStepRate = [](float value, float duration) {
            if (value <= 0.0f || value >= 1.0f)
                return 0.0f;
            return 6.0f * value * (1.0f - value) / duration;
        };
        const float apexZ
            = std::max(input.mLiftOffPosition.GetZ(), input.mPlacementPosition.GetZ()) + profile.mSwingFootLift;
        SwingFootTrajectoryOutput result;
        result.mPosition = input.mLiftOffPosition;
        if (input.mGait.mSwingTouchdownCandidate)
        {
            result.mPosition = input.mPlacementPosition;
            return result;
        }
        // Contact loss starts horizontal transfer; it does not finish the
        // ongoing rise. Keep its position and velocity continuous at lift-off.
        const float normalizedTime = input.mGait.mTimeInPhase / halfDuration;
        const float lift = smoothStep(normalizedTime);
        const float liftRate = smoothStepRate(normalizedTime, halfDuration);
        const float rise = (apexZ - input.mLiftOffPosition.GetZ()) * lift;
        const float riseVelocity = (apexZ - input.mLiftOffPosition.GetZ()) * liftRate;
        if (!input.mGait.mSwingFootLifted)
        {
            result.mPosition = input.mLiftOffPosition;
            result.mPosition.SetZ(input.mLiftOffPosition.GetZ() + rise);
            result.mLinearVelocity.SetZ(riseVelocity);
        }
        else
        {
            // Finish horizontal transfer at clearance before descending. If
            // translation and descent share the same short interval, a
            // saturated leg can touch down near its lift-off point and the
            // contact-driven phase machine will accept a false step.
            const float transferTime = input.mGait.mSwingTimeSinceLift / halfDuration;
            const float transfer = smoothStep(transferTime);
            const float transferRate = smoothStepRate(transferTime, halfDuration);
            const float descentTime = (input.mGait.mSwingTimeSinceLift - halfDuration) / halfDuration;
            const float descent = smoothStep(descentTime);
            const float descentRate = smoothStepRate(descentTime, halfDuration);
            result.mPosition
                = input.mLiftOffPosition + (input.mPlacementPosition - input.mLiftOffPosition) * transfer;
            result.mPosition.SetZ(input.mLiftOffPosition.GetZ() + rise
                + (input.mPlacementPosition.GetZ() - apexZ) * descent);
            result.mLinearVelocity = (input.mPlacementPosition - input.mLiftOffPosition) * transferRate;
            result.mLinearVelocity.SetZ(riseVelocity
                + (input.mPlacementPosition.GetZ() - apexZ) * descentRate);
        }
        return isFinite(result.mPosition) && isFinite(result.mLinearVelocity)
            ? std::optional<SwingFootTrajectoryOutput>(result)
            : std::nullopt;
    }

    std::optional<JPH::Vec3> calculateSwingFootTrajectory(
        const SwingFootTrajectoryInput& input, const GaitControllerProfile& profile)
    {
        const auto state = calculateSwingFootTrajectoryState(input, profile);
        return state ? std::optional<JPH::Vec3>(state->mPosition) : std::nullopt;
    }

    bool applyGaitMeasuredLegTargets(std::span<Transform> target, std::span<const Transform> measured,
        std::size_t pelvis, std::span<const std::size_t> leg, GaitMeasuredPoseFrame frame)
    {
        if (pelvis >= target.size() || pelvis >= measured.size() || leg.empty())
            return false;
        const auto valid = [](JPH::QuatArg rotation) { return isFinite(rotation) && rotation.IsNormalized(); };
        if (!valid(target[pelvis].mRotation) || !valid(measured[pelvis].mRotation))
            return false;
        for (std::size_t i = 0; i < leg.size(); ++i)
        {
            if (leg[i] == pelvis || leg[i] >= target.size() || leg[i] >= measured.size()
                || !valid(measured[leg[i]].mRotation))
                return false;
            for (std::size_t j = 0; j < i; ++j)
                if (leg[i] == leg[j])
                    return false;
        }
        const JPH::Quat delta = (target[pelvis].mRotation * measured[pelvis].mRotation.Conjugated()).Normalized();
        for (const std::size_t body : leg)
            target[body].mRotation = frame == GaitMeasuredPoseFrame::Joint
                ? (delta * measured[body].mRotation).Normalized() : measured[body].mRotation;
        return true;
    }

    bool applyTwoBoneLegTarget(std::span<Transform> pose, const TwoBoneLegTargetInput& input)
    {
        if (input.mHipBodyIndex >= pose.size() || input.mKneeBodyIndex >= pose.size()
            || input.mFootBodyIndex >= pose.size() || input.mHipBodyIndex == input.mKneeBodyIndex
            || input.mHipBodyIndex == input.mFootBodyIndex || input.mKneeBodyIndex == input.mFootBodyIndex
            || !isFinite(input.mTargetFootPosition) || !isFinite(input.mPreferredBendDirection))
            return false;

        const Transform& hip = pose[input.mHipBodyIndex];
        const Transform& knee = pose[input.mKneeBodyIndex];
        const Transform& foot = pose[input.mFootBodyIndex];
        if (!isFinite(hip.mTranslation) || !isFinite(knee.mTranslation) || !isFinite(foot.mTranslation)
            || !isFinite(hip.mRotation) || !hip.mRotation.IsNormalized() || !isFinite(knee.mRotation)
            || !knee.mRotation.IsNormalized() || !isFinite(foot.mRotation) || !foot.mRotation.IsNormalized())
            return false;

        HingeLegGeometry geometry;
        if (input.mHingeGeometry)
            geometry = *input.mHingeGeometry;
        else
        {
            // Generic callers have no native joint. Infer a planar hinge from
            // the supplied pose and use the same solver, without joint limits.
            const auto inverseHip = hip.mRotation.Conjugated();
            geometry.mUpperOffset = inverseHip * (knee.mTranslation - hip.mTranslation);
            geometry.mLowerOffset = inverseHip * (foot.mTranslation - knee.mTranslation);
            geometry.mLowerRotation = (inverseHip * knee.mRotation).Normalized();
            auto axis = geometry.mUpperOffset.Cross(geometry.mLowerOffset);
            if (axis.IsNearZero())
                axis = geometry.mUpperOffset.Cross(inverseHip * input.mPreferredBendDirection);
            if (axis.IsNearZero())
            {
                const auto reference = std::abs(geometry.mUpperOffset.NormalizedOr(JPH::Vec3::sAxisZ()).GetZ()) < 0.9f
                    ? JPH::Vec3::sAxisZ() : JPH::Vec3::sAxisX();
                axis = geometry.mUpperOffset.Cross(reference);
            }
            if (axis.IsNearZero())
                return false;
            geometry.mHingeAxis = axis.Normalized();
            geometry.mMinimumAngle = -std::numbers::pi_v<float>;
            geometry.mMaximumAngle = std::numbers::pi_v<float>;
        }
        const auto& upper = geometry.mUpperOffset;
        const auto& lower = geometry.mLowerOffset;
        const auto& axis = geometry.mHingeAxis;
        if (!isFinite(upper) || !isFinite(lower) || upper.LengthSq() <= 1e-8f || lower.LengthSq() <= 1e-8f
            || !isFinite(axis) || !axis.IsNormalized() || !isFinite(geometry.mLowerRotation)
            || !geometry.mLowerRotation.IsNormalized() || !isFinite(geometry.mMinimumAngle)
            || !isFinite(geometry.mMaximumAngle) || geometry.mMinimumAngle > geometry.mMaximumAngle
            || geometry.mMaximumAngle - geometry.mMinimumAngle > 2.0f * std::numbers::pi_v<float>
            || geometry.mMinimumAngle < -2.0f * std::numbers::pi_v<float>
            || geometry.mMaximumAngle > 2.0f * std::numbers::pi_v<float>)
            return false;
        const auto target = input.mTargetFootPosition - hip.mTranslation;
        float distance = target.Length();
        if (!isFinite(distance) || distance <= 1e-4f)
            return false;
        const auto direction = target / distance;
        if (!input.mHingeGeometry)
            distance = std::clamp(distance, std::abs(upper.Length() - lower.Length()) + 1e-4f,
                upper.Length() + lower.Length() - 1e-4f);
        const auto upperPerpendicular = upper - axis * upper.Dot(axis);
        const auto lowerPerpendicular = lower - axis * lower.Dot(axis);
        const float amplitude = 2.0f * upperPerpendicular.Length() * lowerPerpendicular.Length();
        if (amplitude <= 1e-8f)
            return false;
        // The axial offsets are fixed by the actual hinge. A planar
        // triangle's sum/difference radii are not this chain's reach.
        const float base = upper.LengthSq() + lower.LengthSq() + 2.0f * upper.Dot(axis) * lower.Dot(axis);
        const float reference = std::atan2(axis.Dot(upperPerpendicular.Cross(lowerPerpendicular)),
            upperPerpendicular.Dot(lowerPerpendicular));
        const float opening = std::acos(std::clamp((distance * distance - base) / amplitude, -1.0f, 1.0f));
        float angle = geometry.mMinimumAngle;
        float bestError = std::numeric_limits<float>::max();
        const auto consider = [&](float candidate) {
            candidate = std::clamp(candidate, geometry.mMinimumAngle, geometry.mMaximumAngle);
            const float error = std::abs(base + amplitude * std::cos(reference + candidate) - distance * distance);
            if (error < bestError - 1e-5f || (std::abs(error - bestError) <= 1e-5f && std::abs(candidate) < std::abs(angle)))
            {
                bestError = error;
                angle = candidate;
            }
        };
        consider(geometry.mMaximumAngle);
        consider(geometry.mMinimumAngle);
        for (const float sign : { -1.0f, 1.0f })
            for (const int turn : { -1, 0, 1 })
                consider(sign * opening - reference + turn * (2.0f * std::numbers::pi_v<float>));
        const auto flexion = JPH::Quat::sRotation(axis, angle);
        const auto reach = upper + flexion * lower;
        if (reach.IsNearZero())
            return false;
        auto rotation = (JPH::Quat::sFromTo((hip.mRotation * reach).Normalized(), direction)
            * hip.mRotation).Normalized();
        auto kneeDirection = rotation * upper;
        kneeDirection -= direction * kneeDirection.Dot(direction);
        auto bend = input.mPreferredBendDirection - direction * input.mPreferredBendDirection.Dot(direction);
        if (input.mHingeGeometry)
        {
            // Fixed offsets along the hinge axis are not knee flexion. Near
            // extension they can dominate the knee's projected position and
            // invent a quarter turn at the hip. Orient the hinge plane itself;
            // its sign is immaterial, so preserve the nearest orientation.
            kneeDirection = rotation * axis;
            kneeDirection -= direction * kneeDirection.Dot(direction);
            bend = direction.Cross(input.mPreferredBendDirection);
            if (kneeDirection.Dot(bend) < 0)
                bend = -bend;
        }
        if (!kneeDirection.IsNearZero() && !bend.IsNearZero())
        {
            kneeDirection = kneeDirection.Normalized();
            bend = bend.Normalized();
            const float twist = std::atan2(direction.Dot(kneeDirection.Cross(bend)), kneeDirection.Dot(bend));
            rotation = (JPH::Quat::sRotation(direction, twist) * rotation).Normalized();
        }
        const auto kneeRotation = (rotation * flexion * geometry.mLowerRotation).Normalized();
        const auto kneePosition = hip.mTranslation + rotation * upper;
        const auto footPosition = kneePosition + rotation * (flexion * lower);
        if (!isFinite(rotation) || !isFinite(kneeRotation) || !isFinite(kneePosition) || !isFinite(footPosition))
            return false;
        pose[input.mHipBodyIndex].mRotation = rotation;
        pose[input.mKneeBodyIndex].mRotation = kneeRotation;
        pose[input.mKneeBodyIndex].mTranslation = kneePosition;
        pose[input.mFootBodyIndex].mTranslation = footPosition;
        return true;
    }

    bool applyMeasuredSeededTwoBoneLegTarget(std::span<Transform> target,
        std::span<const Transform> measured, std::size_t pelvis, const TwoBoneLegTargetInput& input)
    {
        const std::array<std::size_t, 3> leg{
            input.mHipBodyIndex, input.mKneeBodyIndex, input.mFootBodyIndex };
        if (pelvis >= target.size() || pelvis >= measured.size())
            return false;
        for (const std::size_t body : leg)
            if (body >= target.size())
                return false;

        const std::array<Transform, 3> previous{
            target[leg[0]], target[leg[1]], target[leg[2]] };
        if (applyGaitMeasuredLegTargets(target, measured, pelvis, leg, GaitMeasuredPoseFrame::Joint)
            && applyTwoBoneLegTarget(target, input))
            return true;

        for (std::size_t i = 0; i < leg.size(); ++i)
            target[leg[i]] = previous[i];
        return false;
    }

    std::optional<LegFrameTargetOutput> resolveLegFrameTarget(const LegFrameTargetInput& input)
    {
        const auto validTransform = [](const Transform& transform) {
            return isFinite(transform.mTranslation) && isFinite(transform.mRotation)
                && transform.mRotation.IsNormalized();
        };
        if (!validTransform(input.mBasePelvis) || !validTransform(input.mBaseHip)
            || !validTransform(input.mMeasuredPelvis) || !validTransform(input.mMeasuredHip)
            || !isFinite(input.mTargetFootPosition) || !isFinite(input.mPreferredBendDirection))
            return std::nullopt;

        LegFrameTargetOutput result;
        result.mPelvisRotationDelta
            = (input.mMeasuredPelvis.mRotation * input.mBasePelvis.mRotation.Conjugated()).Normalized();
        const JPH::Quat inverseDelta = result.mPelvisRotationDelta.Conjugated();
        const JPH::Vec3 offset = input.mTargetFootPosition - input.mMeasuredHip.mTranslation;
        result.mTargetFootPosition = input.mBaseHip.mTranslation + inverseDelta * offset;
        result.mPreferredBendDirection = inverseDelta * input.mPreferredBendDirection;
        if (!isFinite(result.mPelvisRotationDelta) || !result.mPelvisRotationDelta.IsNormalized()
            || !isFinite(result.mTargetFootPosition) || !isFinite(result.mPreferredBendDirection))
            return std::nullopt;
        return result;
    }

    std::optional<JPH::Quat> calculateLevelFootTarget(const LevelFootTargetInput& input)
    {
        if (!isFinite(input.mBaseFootRotation) || !input.mBaseFootRotation.IsNormalized()
            || !isFinite(input.mTargetCalfRotation) || !input.mTargetCalfRotation.IsNormalized()
            || !isFinite(input.mMeasuredCalfRotation) || !input.mMeasuredCalfRotation.IsNormalized()
            || !isFinite(input.mPelvisRotationDelta) || !input.mPelvisRotationDelta.IsNormalized())
            return std::nullopt;

        JPH::Vec3 forward = input.mPelvisRotationDelta * JPH::Vec3::sAxisY();
        forward.SetZ(0.0f);
        JPH::Quat yaw = JPH::Quat::sIdentity();
        const float forwardLengthSquared = forward.LengthSq();
        if (isFinite(forwardLengthSquared) && forwardLengthSquared > 1e-6f)
            yaw = JPH::Quat::sFromTo(JPH::Vec3::sAxisY(), forward / std::sqrt(forwardLengthSquared));

        const JPH::Quat result
            = (input.mTargetCalfRotation * input.mMeasuredCalfRotation.Conjugated() * yaw * input.mBaseFootRotation)
                  .Normalized();
        if (!isFinite(result) || !result.IsNormalized())
            return std::nullopt;
        return result;
    }

    std::optional<SwingLegPoseOutput> applySwingLegPose(std::span<Transform> target,
        std::span<const Transform> base, std::span<const Transform> measured, std::size_t pelvis,
        TwoBoneLegTargetInput input, const GaitActuationDecision& decision)
    {
        const std::array<std::size_t, 4> bodies{ pelvis, input.mHipBodyIndex, input.mKneeBodyIndex, input.mFootBodyIndex };
        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            if (bodies[i] >= target.size() || bodies[i] >= base.size() || bodies[i] >= measured.size())
                return std::nullopt;
            for (std::size_t j = 0; j < i; ++j)
                if (bodies[i] == bodies[j])
                    return std::nullopt;
        }
        const auto frame = resolveLegFrameTarget({ base[pelvis], base[input.mHipBodyIndex], measured[pelvis],
            measured[input.mHipBodyIndex], input.mTargetFootPosition, input.mPreferredBendDirection });
        if (!frame)
            return std::nullopt;
        SwingLegPoseOutput result;
        result.mFrame = *frame;
        input.mTargetFootPosition = frame->mTargetFootPosition;
        input.mPreferredBendDirection = frame->mPreferredBendDirection;
        std::vector<Transform> candidate(target.begin(), target.end());
        if (!applyTwoBoneLegTarget(candidate, input))
            return result;
        result.mFootPosition = candidate[input.mFootBodyIndex].mTranslation;
        const auto foot = calculateLevelFootTarget({ base[input.mFootBodyIndex].mRotation,
            candidate[input.mKneeBodyIndex].mRotation, measured[input.mKneeBodyIndex].mRotation,
            frame->mPelvisRotationDelta });
        if (foot)
            candidate[input.mFootBodyIndex].mRotation = *foot;
        if (decision.mPoseSource == GaitPoseSource::MeasuredTouchdown)
        {
            const std::array<std::size_t, 3> chain{ input.mHipBodyIndex, input.mKneeBodyIndex, input.mFootBodyIndex };
            if (!applyGaitMeasuredLegTargets(candidate, measured, pelvis, chain, decision.mMeasuredPoseFrame))
                return std::nullopt;
        }
        result.mHipModelRotation
            = (frame->mPelvisRotationDelta * candidate[input.mHipBodyIndex].mRotation).Normalized();
        result.mApplied = true;
        for (const auto body : { input.mHipBodyIndex, input.mKneeBodyIndex, input.mFootBodyIndex })
            target[body] = candidate[body];
        return result;
    }

    std::optional<WorldOrientationControllerOutput> calculateSwingHipWorldControl(
        const WorldOrientationControllerInput& input, const GaitControllerProfile& profile)
    {
        if (!validProfile(profile) || !isFinite(input.mCurrentRotation) || !input.mCurrentRotation.IsNormalized()
            || !isFinite(input.mTargetRotation) || !input.mTargetRotation.IsNormalized()
            || !isFinite(input.mCurrentAngularVelocity))
            return std::nullopt;

        // q and -q encode the same orientation. Canonicalizing W keeps the
        // axis-angle conversion on the shortest arc instead of occasionally
        // requesting a nearly full revolution.
        const JPH::Quat error
            = (input.mTargetRotation * input.mCurrentRotation.Conjugated()).Normalized().EnsureWPositive();
        JPH::Vec3 axis;
        float angle = 0.0f;
        error.GetAxisAngle(axis, angle);
        JPH::Vec3 torque = axis * (angle * profile.mSwingHipWorldStiffness)
            - input.mCurrentAngularVelocity * profile.mSwingHipWorldDamping;
        WorldOrientationControllerOutput result;
        const float length = torque.Length();
        if (!isFinite(length))
            return std::nullopt;
        if (length > profile.mMaximumSwingHipWorldTorque && length > 0.0f)
        {
            torque *= profile.mMaximumSwingHipWorldTorque / length;
            result.mSaturated = true;
        }
        result.mTorque = torque;
        return result;
    }

    std::optional<SwingFootControllerOutput> calculateSwingFootControl(
        const SwingFootControllerInput& input, const GaitControllerProfile& profile)
    {
        if (!validProfile(profile) || !isFinite(input.mCurrentPosition) || !isFinite(input.mTargetPosition)
            || !isFinite(input.mCurrentVelocity) || !isFinite(input.mTargetVelocity) || !isFinite(input.mEffectiveMass)
            || input.mEffectiveMass <= 0.0f)
            return std::nullopt;

        constexpr float twoPi = 6.2831853071795864769f;
        const float omega = twoPi * profile.mSwingFootFrequency;
        const JPH::Vec3 positionError = JPH::Vec3(input.mTargetPosition - input.mCurrentPosition);
        JPH::Vec3 force = input.mEffectiveMass
            * (positionError * (omega * omega)
                + (input.mTargetVelocity - input.mCurrentVelocity) * (2.0f * profile.mSwingFootDamping * omega));
        // Before release, do not turn a lift request into lateral drag across
        // the ground. In flight, the equal-and-opposite swing-chain pair may
        // correct all three axes without injecting net body momentum.
        if (!input.mFootLifted)
        {
            force.SetX(0.0f);
            force.SetY(0.0f);
        }
        JPH::Vec3 horizontalForce(force.GetX(), force.GetY(), 0.0f);
        const float horizontalLengthSquared = horizontalForce.LengthSq();
        if (!isFinite(horizontalLengthSquared) || !isFinite(force.GetZ()))
            return std::nullopt;

        SwingFootControllerOutput result;
        const float maximumHorizontalSquared
            = profile.mMaximumSwingFootHorizontalForce * profile.mMaximumSwingFootHorizontalForce;
        if (horizontalLengthSquared > maximumHorizontalSquared && horizontalLengthSquared > 0.0f)
        {
            horizontalForce *= profile.mMaximumSwingFootHorizontalForce / std::sqrt(horizontalLengthSquared);
            result.mSaturated = true;
        }
        const float verticalForce = std::clamp(
            force.GetZ(), -profile.mMaximumSwingFootForce, profile.mMaximumSwingFootForce);
        result.mSaturated = result.mSaturated || verticalForce != force.GetZ();
        force = horizontalForce + JPH::Vec3::sAxisZ() * verticalForce;
        result.mForce = force;
        return result;
    }
}
