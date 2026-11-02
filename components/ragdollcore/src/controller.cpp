#include <ragdollcore/controller.hpp>
#include <ragdollcore/tuning.hpp>
#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/unitmodel.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

namespace RagdollCore
{
    namespace
    {
        struct Point2
        {
            // Contacts can form a very thin patch when a foot rolls onto an
            // edge. Keep projection and barycentric reconstruction in the
            // same precision; rounding a projected point to float can put it
            // outside every triangle of an otherwise valid support hull.
            double mX = 0.0;
            double mY = 0.0;
        };

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
            return isFinite(value.GetX()) && isFinite(value.GetY())
                && isFinite(value.GetZ()) && isFinite(value.GetW());
        }

        bool validProfile(const ActiveControllerProfile& profile)
        {
            return isFinite(profile.mSupportNormalMinimumDot) && profile.mSupportNormalMinimumDot >= 0.0f
                && profile.mSupportNormalMinimumDot <= 1.0f && isFinite(profile.mSupportMargin)
                && profile.mSupportMargin >= 0.0f && isFinite(profile.mStandingMinimumUpDot)
                && profile.mStandingMinimumUpDot >= -1.0f && profile.mStandingMinimumUpDot <= 1.0f
                && isFinite(profile.mBracingMinimumUpDot) && profile.mBracingMinimumUpDot >= -1.0f
                && profile.mBracingMinimumUpDot <= profile.mStandingMinimumUpDot
                && isFinite(profile.mStandingMaximumSpeed) && profile.mStandingMaximumSpeed >= 0.0f
                && isFinite(profile.mSettledMaximumSpeed) && profile.mSettledMaximumSpeed >= 0.0f
                && profile.mSettledMaximumSpeed <= profile.mStandingMaximumSpeed
                && isFinite(profile.mFallenMaximumPelvisHeight) && profile.mFallenMaximumPelvisHeight >= 0.0f
                && isFinite(profile.mMinimumStateDuration) && profile.mMinimumStateDuration >= 0.0f
                && isFinite(profile.mFallingExitSupportDuration) && profile.mFallingExitSupportDuration >= 0.0f
                && isFinite(profile.mFallingExitMaximumVerticalSpeed)
                && profile.mFallingExitMaximumVerticalSpeed >= 0.0f
                && isFinite(profile.mRecoveryDelay) && profile.mRecoveryDelay >= 0.0f
                && isFinite(profile.mBlendBackDuration) && profile.mBlendBackDuration >= 0.0f
                && isFinite(profile.mPelvisHeightSettleDuration)
                && profile.mPelvisHeightSettleDuration >= 0.0f
                && isFinite(profile.mPelvisHeightSettleMaximumVerticalSpeed)
                && profile.mPelvisHeightSettleMaximumVerticalSpeed >= 0.0f
                && isFinite(profile.mActivationSupportGraceDuration)
                && profile.mActivationSupportGraceDuration >= 0.0f;
        }

        bool validUprightProfile(const ActiveControllerProfile& profile)
        {
            return validProfile(profile) && isFinite(profile.mPelvisHeightFrequency)
                && profile.mPelvisHeightFrequency >= 0.0f && isFinite(profile.mPelvisHeightDamping)
                && profile.mPelvisHeightDamping >= 0.0f && isFinite(profile.mCenterOfMassFrequency)
                && profile.mCenterOfMassFrequency >= 0.0f && isFinite(profile.mCenterOfMassDamping)
                && profile.mCenterOfMassDamping >= 0.0f && isFinite(profile.mMaximumLocomotionSpeed)
                && profile.mMaximumLocomotionSpeed >= 0.0f
                && isFinite(profile.mLocomotionSupportGraceDuration)
                && profile.mLocomotionSupportGraceDuration >= 0.0f && isFinite(profile.mMaximumPelvisForce)
                && profile.mMaximumPelvisForce >= 0.0f && isFinite(profile.mRecoveryUnweightFactor)
                && isFinite(profile.mAnimationAssistHorizontalScale)
                && profile.mAnimationAssistHorizontalScale >= 0.0f
                && profile.mAnimationAssistHorizontalScale <= 1.0f
                && isFinite(profile.mAnimationAssistVerticalScale)
                && profile.mAnimationAssistVerticalScale >= 0.0f
                && profile.mAnimationAssistVerticalScale <= 1.0f
                && isFinite(profile.mMaximumAnimationAssistForce)
                && profile.mMaximumAnimationAssistForce >= 0.0f
                && isFinite(profile.mAnimationAssistUprightScale)
                && profile.mAnimationAssistUprightScale >= 0.0f
                && profile.mAnimationAssistUprightScale <= 1.0f
                && isFinite(profile.mMaximumAnimationAssistTorque)
                && profile.mMaximumAnimationAssistTorque >= 0.0f
                && isFinite(profile.mAnimationAssistYawReferenceMaximumRate)
                && profile.mAnimationAssistYawReferenceMaximumRate > 0.0f
                && isFinite(profile.mAnimationAssistYawReferenceMaximumAcceleration)
                && profile.mAnimationAssistYawReferenceMaximumAcceleration > 0.0f
                && isFinite(profile.mAnimationAssistYawStiffness)
                && profile.mAnimationAssistYawStiffness >= 0.0f
                && isFinite(profile.mAnimationAssistYawDamping)
                && profile.mAnimationAssistYawDamping >= 0.0f
                && isFinite(profile.mMaximumAnimationAssistYawTorque)
                && profile.mMaximumAnimationAssistYawTorque >= 0.0f
                && profile.mMaximumAnimationAssistYawTorque <= profile.mMaximumAnimationAssistTorque
                && isFinite(profile.mAnimationAssistPoseTrackingBlend)
                && profile.mAnimationAssistPoseTrackingBlend >= 0.0f
                && profile.mAnimationAssistPoseTrackingBlend <= 1.0f
                && isFinite(profile.mAnimationAssistStanceAnchorCompliance)
                && profile.mAnimationAssistStanceAnchorCompliance >= 0.0f
                && profile.mAnimationAssistStanceAnchorCompliance <= 1.0f
                && isFinite(profile.mAnimationAssistSupportLossHandoffDuration)
                && profile.mAnimationAssistSupportLossHandoffDuration > 0.0f
                && isFinite(profile.mAnimationAssistStancePositionCorrection)
                && profile.mAnimationAssistStancePositionCorrection >= 0.0f
                && profile.mAnimationAssistStancePositionCorrection <= 1.0f
                && isFinite(profile.mAnimationAssistStanceMaximumEndpointCorrection)
                && profile.mAnimationAssistStanceMaximumEndpointCorrection > 0.0f
                && isFinite(profile.mAnimationAssistSupportMotionFrequency)
                && profile.mAnimationAssistSupportMotionFrequency > 0.0f
                && isFinite(profile.mAnimationAssistSupportMotionDamping)
                && profile.mAnimationAssistSupportMotionDamping >= 0.0f
                && isFinite(profile.mAnimationAssistSupportPositionLeash)
                && profile.mAnimationAssistSupportPositionLeash > 0.0f
                && profile.mAnimationAssistContactConfirmationTicks > 0
                && profile.mAnimationAssistContactConfirmationTicks <= 60
                && isFinite(profile.mAnimationAssistLoadTransferDuration)
                && profile.mAnimationAssistLoadTransferDuration > 0.0f
                && profile.mRecoveryUnweightFactor >= 0.0f && profile.mRecoveryUnweightFactor <= 3.0f
                && isFinite(profile.mPelvisUprightStiffness)
                && profile.mPelvisUprightStiffness >= 0.0f && isFinite(profile.mPelvisUprightDamping)
                && profile.mPelvisUprightDamping >= 0.0f && isFinite(profile.mMaximumPelvisTorque)
                && profile.mMaximumPelvisTorque >= 0.0f && isFinite(profile.mSpineUprightStiffness)
                && profile.mSpineUprightStiffness >= 0.0f && isFinite(profile.mSpineUprightDamping)
                && profile.mSpineUprightDamping >= 0.0f && isFinite(profile.mMaximumSpineTorque)
                && profile.mMaximumSpineTorque >= 0.0f;
        }

        JPH::Vec3 clampMagnitude(JPH::Vec3Arg value, float maximum, bool& saturated)
        {
            const float lengthSquared = value.LengthSq();
            if (lengthSquared <= maximum * maximum || lengthSquared <= std::numeric_limits<float>::epsilon())
                return value;
            saturated = true;
            return value * (maximum / std::sqrt(lengthSquared));
        }

        JPH::Vec3 clampHorizontalVelocity(JPH::Vec3Arg value, float maximum)
        {
            JPH::Vec3 horizontal(value.GetX(), value.GetY(), 0.0f);
            bool saturated = false;
            return clampMagnitude(horizontal, maximum, saturated);
        }

        JPH::Vec3 uprightTorque(JPH::Vec3Arg currentUp, JPH::Vec3Arg targetUp,
            JPH::Vec3Arg angularVelocity, float stiffness, float damping)
        {
            JPH::Vec3 tiltError = currentUp.Cross(targetUp);
            const float sine = tiltError.Length();
            const float cosine = std::clamp(currentUp.Dot(targetUp), -1.0f, 1.0f);
            if (sine > 1e-6f)
                tiltError *= std::atan2(sine, cosine) / sine;
            else if (cosine < 0.0f)
            {
                const JPH::Vec3 reference
                    = std::abs(currentUp.GetX()) < 0.9f ? JPH::Vec3::sAxisX() : JPH::Vec3::sAxisY();
                tiltError = currentUp.Cross(reference).Normalized() * 3.14159265358979323846f;
            }
            const JPH::Vec3 tiltAngularVelocity
                = angularVelocity - targetUp * angularVelocity.Dot(targetUp);
            return tiltError * stiffness - tiltAngularVelocity * damping;
        }

        double cross(const Point2& origin, const Point2& a, const Point2& b)
        {
            return (a.mX - origin.mX) * (b.mY - origin.mY)
                - (a.mY - origin.mY) * (b.mX - origin.mX);
        }

        double squaredDistance(const Point2& a, const Point2& b)
        {
            const double x = a.mX - b.mX;
            const double y = a.mY - b.mY;
            return x * x + y * y;
        }

        Point2 closestPointOnSegment(const Point2& point, const Point2& a, const Point2& b)
        {
            const double lengthSquared = squaredDistance(a, b);
            if (lengthSquared <= std::numeric_limits<float>::epsilon())
                return a;
            const double projection = std::clamp(
                ((point.mX - a.mX) * (b.mX - a.mX) + (point.mY - a.mY) * (b.mY - a.mY))
                    / lengthSquared,
                0.0, 1.0);
            return { a.mX + (b.mX - a.mX) * projection, a.mY + (b.mY - a.mY) * projection };
        }

        std::vector<Point2> convexHull(std::vector<Point2> points)
        {
            std::sort(points.begin(), points.end(), [](const Point2& a, const Point2& b) {
                return a.mX < b.mX || (a.mX == b.mX && a.mY < b.mY);
            });
            points.erase(std::unique(points.begin(), points.end(), [](const Point2& a, const Point2& b) {
                return a.mX == b.mX && a.mY == b.mY;
            }), points.end());
            if (points.size() <= 2)
                return points;

            std::vector<Point2> hull;
            hull.reserve(points.size() * 2);
            for (const Point2& point : points)
            {
                while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), point) <= 0.0f)
                    hull.pop_back();
                hull.push_back(point);
            }
            const std::size_t lowerSize = hull.size();
            for (auto it = points.rbegin() + 1; it != points.rend(); ++it)
            {
                while (hull.size() > lowerSize && cross(hull[hull.size() - 2], hull.back(), *it) <= 0.0f)
                    hull.pop_back();
                hull.push_back(*it);
            }
            hull.pop_back();
            return hull;
        }

        // Intersect the measured support hull with the line through the COM
        // opposite the requested horizontal force. Under constant height and
        // zero angular-momentum rate, CoP = COM - height * force / weight.
        // Projecting just the furthest vertex ignores transverse separation
        // and can claim a center of pressure that is outside the hull.
        std::optional<std::pair<float, float>> supportForceLeverInterval(
            const UprightControllerInput& input, JPH::Vec3Arg direction)
        {
            const JPH::Vec3 lateral = JPH::Vec3::sAxisZ().Cross(direction);
            std::vector<Point2> points;
            for (const JPH::RVec3& contact : input.mSupportContactPoints)
            {
                const JPH::Vec3 relative(contact - input.mCenterOfMassPosition);
                points.push_back({ relative.Dot(direction), relative.Dot(lateral) });
            }
            const auto hull = convexHull(std::move(points));
            float minimum = std::numeric_limits<float>::infinity();
            float maximum = -std::numeric_limits<float>::infinity();
            const auto include = [&](float x) {
                minimum = std::min(minimum, x);
                maximum = std::max(maximum, x);
            };
            for (std::size_t i = 0; i < hull.size(); ++i)
            {
                const Point2& a = hull[i];
                const Point2& b = hull[(i + 1) % hull.size()];
                // This is only a float round-off tolerance in world units.
                if (std::abs(a.mY) <= 1e-6f)
                    include(static_cast<float>(a.mX));
                if ((a.mY < 0.0f && b.mY > 0.0f) || (a.mY > 0.0f && b.mY < 0.0f))
                    include(static_cast<float>(a.mX - a.mY * (b.mX - a.mX) / (b.mY - a.mY)));
            }
            if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum > 0.0f)
                return std::nullopt;
            return std::pair{ std::max(0.0f, -maximum), -minimum };
        }

        struct HullProjection
        {
            Point2 mPoint;
            double mDistance = 0.0;
        };

        HullProjection projectToHull(const Point2& point, const std::vector<Point2>& hull)
        {
            if (hull.size() == 1)
                return { hull[0], std::sqrt(squaredDistance(point, hull[0])) };
            if (hull.size() == 2)
            {
                const Point2 closest = closestPointOnSegment(point, hull[0], hull[1]);
                return { closest, std::sqrt(squaredDistance(point, closest)) };
            }

            bool inside = true;
            double distance = std::numeric_limits<double>::infinity();
            Point2 closest = point;
            for (std::size_t i = 0; i < hull.size(); ++i)
            {
                const Point2& a = hull[i];
                const Point2& b = hull[(i + 1) % hull.size()];
                if (cross(a, b, point) < 0.0f)
                    inside = false;
                const Point2 candidate = closestPointOnSegment(point, a, b);
                const double candidateDistance = std::sqrt(squaredDistance(point, candidate));
                if (candidateDistance < distance)
                {
                    distance = candidateDistance;
                    closest = candidate;
                }
            }
            return inside ? HullProjection{ point, 0.0f } : HullProjection{ closest, distance };
        }

        bool validSupport(const SupportEstimate& support)
        {
            const bool hasFoot = support.hasSupport();
            return support.mFootContactCount <= support.mGroundContactCount
                && (hasFoot == (support.mFootContactCount > 0))
                && (!support.mCenterOfMassSupported || hasFoot)
                && isFinite(support.mCenter) && isFinite(support.mGroundCenter) && isFinite(support.mClosestPoint)
                && isFinite(support.mCenterOfMassDistance)
                && support.mCenterOfMassDistance >= 0.0f && isFinite(support.mCenterOfMassHeight)
                && (!support.mFriction || (isFinite(*support.mFriction) && *support.mFriction >= 0));
        }
    }

    std::optional<SupportEstimate> estimateFootSupport(JPH::RVec3Arg centerOfMass,
        std::span<const GroundContact> contacts, FootBodyIndices feet, const ActiveControllerProfile& profile)
    {
        if (!isFinite(centerOfMass) || feet.mLeft == feet.mRight || !validProfile(profile))
            return std::nullopt;

        SupportEstimate result;
        bool frictionKnown = true;
        float minimumFriction = std::numeric_limits<float>::max();
        std::vector<Point2> points;
        for (const GroundContact& contact : contacts)
        {
            if (!isFinite(contact.mPosition) || !isFinite(contact.mNormal)
                || !isFinite(contact.mPenetrationDepth)
                || (contact.mNormalImpulse && (!isFinite(*contact.mNormalImpulse) || *contact.mNormalImpulse < 0))
                || (contact.mFriction && (!isFinite(*contact.mFriction) || *contact.mFriction < 0)))
                return std::nullopt;
            if (!contact.hasSupport())
                continue;
            const float normalLengthSquared = contact.mNormal.LengthSq();
            if (!isFinite(normalLengthSquared) || normalLengthSquared <= 1e-8f)
                return std::nullopt;
            if (contact.mNormal.GetZ() / std::sqrt(normalLengthSquared) < profile.mSupportNormalMinimumDot)
                continue;
            ++result.mGroundContactCount;
            result.mGroundCenter += contact.mPosition;
            if (contact.mFriction)
                minimumFriction = std::min(minimumFriction, *contact.mFriction);
            else
                frictionKnown = false;

            const bool left = contact.mBodyIndex == feet.mLeft;
            const bool right = contact.mBodyIndex == feet.mRight;
            if (!left && !right)
                continue;

            result.mLeftFootSupported = result.mLeftFootSupported || left;
            result.mRightFootSupported = result.mRightFootSupported || right;
            points.push_back({ contact.mPosition.GetX(), contact.mPosition.GetY() });
            result.mCenter += contact.mPosition;
            ++result.mFootContactCount;
        }

        if (result.mGroundContactCount == 0)
            return result;
        if (frictionKnown)
            result.mFriction = minimumFriction;
        result.mGroundCenter /= static_cast<double>(result.mGroundContactCount);
        result.mCenterOfMassHeight
            = static_cast<float>(centerOfMass.GetZ() - result.mGroundCenter.GetZ());
        if (points.empty())
            return result;
        result.mCenter /= static_cast<double>(points.size());
        const auto hull = convexHull(std::move(points));
        // Contact sampling density is a collision implementation detail. Use
        // the area's centroid so extra mesh manifolds, duplicate vertices, or
        // interior samples cannot move the balance target. Work relative to a
        // hull vertex to retain precision away from the world origin.
        Point2 center = hull.front();
        if (hull.size() == 2)
            center = { (hull[0].mX + hull[1].mX) * 0.5, (hull[0].mY + hull[1].mY) * 0.5 };
        else if (hull.size() > 2)
        {
            double area = 0, x = 0, y = 0;
            for (std::size_t i = 1; i + 1 < hull.size(); ++i)
            {
                const double triangleArea = cross(hull[0], hull[i], hull[i + 1]);
                area += triangleArea;
                x += triangleArea * ((hull[i].mX - hull[0].mX) + (hull[i + 1].mX - hull[0].mX)) / 3.0;
                y += triangleArea * ((hull[i].mY - hull[0].mY) + (hull[i + 1].mY - hull[0].mY)) / 3.0;
            }
            if (!(area > 0) || !std::isfinite(area) || !std::isfinite(x) || !std::isfinite(y))
                return std::nullopt;
            center = { hull[0].mX + x / area, hull[0].mY + y / area };
        }
        result.mCenter.SetX(center.mX);
        result.mCenter.SetY(center.mY);
        const Point2 projectedCenterOfMass{ centerOfMass.GetX(), centerOfMass.GetY() };
        const HullProjection projection = projectToHull(projectedCenterOfMass, hull);
        result.mClosestPoint = JPH::RVec3(projection.mPoint.mX, projection.mPoint.mY, result.mGroundCenter.GetZ());
        result.mCenterOfMassDistance = static_cast<float>(projection.mDistance);
        result.mCenterOfMassSupported = result.mCenterOfMassDistance <= profile.mSupportMargin;
        return result;
    }

    std::optional<RootTorqueDistribution> distributeRootTorque(std::span<const JointTorqueRequest> joints,
        std::size_t leftHip, std::size_t rightHip, float leftFootLoad, float rightFootLoad,
        JPH::Vec3Arg requestedRootTorque)
    {
        if (joints.empty() || leftHip == 0 || rightHip == 0 || leftHip == rightHip
            || leftHip >= joints.size() || rightHip >= joints.size()
            || joints[leftHip].mParent != 0 || joints[rightHip].mParent != 0
            || !isFinite(leftFootLoad) || leftFootLoad < 0
            || !isFinite(rightFootLoad) || rightFootLoad < 0 || !isFinite(requestedRootTorque))
            return std::nullopt;
        RootTorqueDistribution result;
        result.mJointTorques.reserve(joints.size());
        for (std::size_t i = 0; i < joints.size(); ++i)
        {
            const auto& joint = joints[i];
            const auto& q = joint.mMotorFrame;
            if ((i == 0 ? joint.mParent != -1 || joint.mTorque != JPH::Vec3::sZero()
                        : joint.mParent < 0 || static_cast<std::size_t>(joint.mParent) >= i)
                || !isFinite(joint.mTorque) || !isFinite(joint.mMaximumAxisTorque)
                || joint.mMaximumAxisTorque.GetX() < 0 || joint.mMaximumAxisTorque.GetY() < 0
                || joint.mMaximumAxisTorque.GetZ() < 0
                || !isFinite(q.GetX()) || !isFinite(q.GetY()) || !isFinite(q.GetZ()) || !isFinite(q.GetW())
                || !q.IsNormalized())
                return std::nullopt;
            result.mJointTorques.push_back(joint.mTorque);
        }
        const auto limit = [&](std::size_t i) {
            const auto& joint = joints[i];
            const auto local = joint.mMotorFrame.InverseRotate(result.mJointTorques[i]);
            if (!isFinite(local))
                return false;
            const auto maximum = joint.mMaximumAxisTorque;
            const JPH::Vec3 bounded(std::clamp(local.GetX(), -maximum.GetX(), maximum.GetX()),
                std::clamp(local.GetY(), -maximum.GetY(), maximum.GetY()),
                std::clamp(local.GetZ(), -maximum.GetZ(), maximum.GetZ()));
            if (bounded != local)
            {
                result.mSaturated = true;
                result.mJointTorques[i] = joint.mMotorFrame * bounded;
            }
            return isFinite(result.mJointTorques[i]);
        };
        // Other root joints must contribute their actual bounded reaction.
        // Hip limits follow composition, so their complete budget is shared.
        for (std::size_t i = 1; i < joints.size(); ++i)
            if (i != leftHip && i != rightHip && !limit(i))
                return std::nullopt;
        const double load = static_cast<double>(leftFootLoad) + rightFootLoad;
        result.mSupported = load > 0;
        if (result.mSupported)
        {
            JPH::Vec3 correction = -requestedRootTorque;
            for (std::size_t i = 1; i < joints.size(); ++i)
                if (joints[i].mParent == 0)
                    correction -= result.mJointTorques[i];
            if (!isFinite(correction))
                return std::nullopt;
            const float leftFraction = static_cast<float>(leftFootLoad / load);
            const float rightFraction = static_cast<float>(rightFootLoad / load);
            result.mJointTorques[leftHip] += correction * leftFraction;
            result.mJointTorques[rightHip] += correction * rightFraction;
        }
        if (!limit(leftHip) || !limit(rightHip))
            return std::nullopt;
        for (std::size_t i = 1; i < joints.size(); ++i)
            if (joints[i].mParent == 0)
                result.mAppliedRootTorque -= result.mJointTorques[i];
        result.mRootTorqueResidual = requestedRootTorque - result.mAppliedRootTorque;
        return isFinite(result.mRootTorqueResidual) ? std::optional<RootTorqueDistribution>(std::move(result))
                                                   : std::nullopt;
    }

    std::optional<JointSupportOutput> calculateJointSupportControl(
        const JointSupportInput& input, const ActiveControllerProfile& profile)
    {
        const auto count = input.mBodies.size();
        if (count == 0 || input.mFeet.mLeft >= count || input.mFeet.mRight >= count
            || input.mFeet.mLeft == input.mFeet.mRight || !isFinite(input.mGravity) || input.mGravity <= 0.0f
            || !isFinite(input.mFriction) || input.mFriction < 0.0f || !validProfile(profile)
            || !isFinite(input.mRequestedAcceleration) || input.mRequestedAcceleration.GetZ() != 0.0f
            || !isFinite(input.mRequestedVerticalAcceleration)
            || input.mRequestedVerticalAcceleration <= -input.mGravity
            || (input.mPushShift != 0.0f && (!isFinite(input.mPushShift) || !(input.mPushShift > 0.0f)
                || !isFinite(input.mPushDirection) || std::abs(input.mPushDirection.GetZ()) > 1e-3f
                || std::abs(input.mPushDirection.Length() - 1.0f) > 1e-2f))
            || !isFinite(input.mCounterRotationAcceleration) || input.mCounterRotationAcceleration < 0.0f)
            return std::nullopt;
        JointSupportOutput result;
        result.mJointTorques.assign(count, JPH::Vec3::sZero());
        double mass = 0;
        const auto origin = input.mBodies.front().mCenterOfMass;
        JPH::RVec3 weightedCenter = JPH::RVec3::sZero();
        for (std::size_t i = 0; i < count; ++i)
        {
            const auto& body = input.mBodies[i];
            if (!isFinite(body.mMass) || body.mMass <= 0.0f || !isFinite(body.mCenterOfMass)
                || !isFinite(body.mJointPosition) || (i == 0 ? body.mParent != -1
                    : body.mParent < 0 || static_cast<std::size_t>(body.mParent) >= i))
                return std::nullopt;
            mass += body.mMass;
            weightedCenter += (body.mCenterOfMass - origin) * body.mMass;
        }
        const auto center = origin + weightedCenter / mass;
        if (!isFinite(center) || mass > std::numeric_limits<float>::max())
            return std::nullopt;
        std::vector<GroundContact> contacts;
        std::vector<Point2> points;
        double groundHeight = 0;
        for (const auto& contact : input.mContacts)
        {
            if (contact.mBodyIndex >= count || !isFinite(contact.mPosition) || !isFinite(contact.mNormal)
                || !isFinite(contact.mPenetrationDepth) || !isFinite(contact.mNormal.LengthSq())
                || contact.mNormal.IsNearZero()
                || (contact.mNormalImpulse && (!isFinite(*contact.mNormalImpulse) || *contact.mNormalImpulse < 0))
                || (contact.mFriction && (!isFinite(*contact.mFriction) || *contact.mFriction < 0)))
                return std::nullopt;
            if (!contact.hasSupport()
                || contact.mNormal.Normalized().GetZ() < profile.mSupportNormalMinimumDot
                || (contact.mBodyIndex != input.mFeet.mLeft && contact.mBodyIndex != input.mFeet.mRight))
                continue;
            if (std::any_of(contacts.begin(), contacts.end(), [&](const auto& previous) {
                    return previous.mBodyIndex == contact.mBodyIndex && previous.mPosition == contact.mPosition;
                }))
                continue;
            contacts.push_back(contact);
            points.push_back({ contact.mPosition.GetX() - center.GetX(), contact.mPosition.GetY() - center.GetY() });
            if (!std::isfinite(points.back().mX) || !std::isfinite(points.back().mY))
                return std::nullopt;
            groundHeight += contact.mPosition.GetZ();
        }
        if (contacts.empty())
            return result;
        groundHeight /= contacts.size();
        const float height = static_cast<float>(center.GetZ() - groundHeight);
        if (!isFinite(height))
            return std::nullopt;
        if (!(height > 0.0f))
            return result;
        bool limited = false;
        const auto request = clampMagnitude(input.mRequestedAcceleration, input.mFriction * input.mGravity, limited);
        const auto hull = convexHull(points);
        // Vertical request channel: N = m(g + az), so the CoP that realizes a
        // horizontal acceleration is d = h * a_h / (g + az). az == 0 reduces
        // exactly to the historical constant-height mapping (float x + 0.0f
        // is exact), preserving byte-identical default behavior.
        const float supportNormal = input.mGravity + input.mRequestedVerticalAcceleration;
        const Point2 desired{ -height * request.GetX() / supportNormal, -height * request.GetY() / supportNormal };
        auto pressure = projectToHull(desired, hull).mPoint;
        result.mHullProjectionDelta = JPH::Vec3(static_cast<float>(pressure.mX - desired.mX),
            static_cast<float>(pressure.mY - desired.mY), 0.0f);
        result.mFrictionLimited = limited;
        if (input.mPushShift != 0.0f)
        {
            // Push-off (trailing-leg) CoP shift: move the solved pressure
            // point aft along the caller-supplied direction and re-project
            // to the hull. Only the fore-aft coordinate moves, so the
            // lateral transfer keeps its solved share of the CoP. The
            // friction-disk clamp below then covers the shifted point.
            const Point2 shifted{ pressure.mX
                    + static_cast<double>(input.mPushShift) * input.mPushDirection.GetX(),
                pressure.mY + static_cast<double>(input.mPushShift) * input.mPushDirection.GetY() };
            pressure = projectToHull(shifted, hull).mPoint;
        }
        const auto nearest = projectToHull({ 0, 0 }, hull).mPoint;
        const float radius = height * input.mFriction;
        const float radiusSquared = radius * radius;
        if (squaredDistance(nearest, { 0, 0 }) > radiusSquared)
            return result;
        if (squaredDistance(pressure, { 0, 0 }) > radiusSquared)
        {
            result.mFrictionLimited = true;
            // The segment stays in the convex hull. Intersect it with the
            // friction disk to keep both support and friction constraints.
            const double dx = pressure.mX - nearest.mX, dy = pressure.mY - nearest.mY;
            const double a = dx * dx + dy * dy;
            const double b = 2.0 * (nearest.mX * dx + nearest.mY * dy);
            const double c = squaredDistance(nearest, { 0, 0 }) - radiusSquared;
            const double t = a > 0 ? std::clamp((-b + std::sqrt(std::max(0.0, b * b - 4.0 * a * c)))
                    / (2.0 * a), 0.0, 1.0) : 0.0;
            pressure = { nearest.mX + t * dx, nearest.mY + t * dy };
        }
        std::vector<double> hullWeights(hull.size(), 0.0);
        if (hull.size() == 1)
            hullWeights[0] = 1;
        else if (hull.size() == 2)
        {
            const double distance = squaredDistance(hull[0], hull[1]);
            const double t = std::clamp(((pressure.mX - hull[0].mX) * (hull[1].mX - hull[0].mX)
                + (pressure.mY - hull[0].mY) * (hull[1].mY - hull[0].mY)) / distance, 0.0, 1.0);
            hullWeights[0] = 1.0 - t;
            hullWeights[1] = t;
        }
        else
        {
            // A fan about the vertex mean gives continuous nonnegative
            // coordinates, including a uniform distribution at its center.
            Point2 hub;
            for (const auto& point : hull)
            {
                hub.mX += point.mX / static_cast<double>(hull.size());
                hub.mY += point.mY / static_cast<double>(hull.size());
            }
            constexpr double roundoff = 8.0 * std::numeric_limits<float>::epsilon();
            bool found = false;
            for (std::size_t i = 0; i < hull.size(); ++i)
            {
                const auto j = (i + 1) % hull.size();
                const double area = cross(hub, hull[i], hull[j]);
                if (!(area > 0))
                    continue;
                const double atHub = cross(pressure, hull[i], hull[j]) / area;
                const double atI = cross(pressure, hull[j], hub) / area;
                const double atJ = cross(pressure, hub, hull[i]) / area;
                if (atHub < -roundoff || atI < -roundoff || atJ < -roundoff)
                    continue;
                const double total = std::max(0.0, atHub) + std::max(0.0, atI) + std::max(0.0, atJ);
                std::fill(hullWeights.begin(), hullWeights.end(), std::max(0.0, atHub) / (total * hull.size()));
                hullWeights[i] += std::max(0.0, atI) / total;
                hullWeights[j] += std::max(0.0, atJ) / total;
                found = true;
                break;
            }
            if (!found)
                return std::nullopt;
        }
        std::vector<double> weights(contacts.size(), 0.0);
        JPH::RVec3 pressureOffset = JPH::RVec3::sZero();
        for (std::size_t i = 0; i < hull.size(); ++i)
        {
            const auto same = [&](const Point2& point) { return point.mX == hull[i].mX && point.mY == hull[i].mY; };
            const auto matches = std::count_if(points.begin(), points.end(), same);
            for (std::size_t j = 0; j < contacts.size(); ++j)
                if (same(points[j]))
                    weights[j] += hullWeights[i] / matches;
        }
        for (std::size_t i = 0; i < contacts.size(); ++i)
            pressureOffset += (contacts[i].mPosition - center) * weights[i];
        const float actualHeight = -static_cast<float>(pressureOffset.GetZ());
        if (!(actualHeight > 0.0f))
            return result;
        result.mAcceleration = JPH::Vec3(-static_cast<float>(pressureOffset.GetX()),
            -static_cast<float>(pressureOffset.GetY()), 0) * (supportNormal / actualHeight)
            + JPH::Vec3(0, 0, input.mRequestedVerticalAcceleration);
        result.mCenterOfPressure = center + pressureOffset;
        // Angular-momentum borrow (the hip strategy inside the solver;
        // hip-moment-borrow-2026-09-11.txt): realize up to
        // mCounterRotationAcceleration of the shortfall the clamped CoP
        // cannot deliver, as a net moment about the COM absorbed by
        // trunk/arm counter-rotation (applied in the moment loop below).
        // The cone check and residuals below see the augmented force, so
        // the borrow can exceed the moment cap but never friction.
        JPH::Vec3 borrowMoment = JPH::Vec3::sZero();
        if (input.mCounterRotationAcceleration > 0.0f)
        {
            const JPH::Vec3 zeroMoment
                = result.mAcceleration - JPH::Vec3(0, 0, input.mRequestedVerticalAcceleration);
            JPH::Vec3 shortfall = request - zeroMoment;
            shortfall.SetZ(0.0f);
            const float shortfallLength = shortfall.Length();
            if (shortfallLength > 1e-6f)
            {
                const float extra = std::min(shortfallLength, input.mCounterRotationAcceleration);
                const JPH::Vec3 deltaA = shortfall * (extra / shortfallLength);
                result.mAcceleration += deltaA;
                borrowMoment = JPH::Vec3(actualHeight * static_cast<float>(mass) * deltaA.GetY(),
                    -actualHeight * static_cast<float>(mass) * deltaA.GetX(), 0.0f);
            }
        }
        // Fable i52: leg-axis force allocation (see header). Replaces the
        // parallel distribution below when feasible; otherwise falls through.
        if (input.mLegAxisFeet && input.mLegAxisHips[0] < count && input.mLegAxisHips[1] < count)
        {
            std::array<std::vector<std::size_t>, 2> footContacts;
            for (std::size_t i = 0; i < contacts.size(); ++i)
                for (std::size_t k = 0; k < 2; ++k)
                    if (contacts[i].mBodyIndex == (*input.mLegAxisFeet)[k])
                        footContacts[k].push_back(i);
            if (!footContacts[0].empty() && !footContacts[1].empty())
            {
                std::array<JPH::RVec3, 2> centroid{ { JPH::RVec3::sZero(), JPH::RVec3::sZero() } };
                std::array<JPH::Vec3, 2> axis;
                std::array<JPH::Vec3, 2> normal{ { JPH::Vec3::sZero(), JPH::Vec3::sZero() } };
                bool geometryOk = true;
                for (std::size_t k = 0; k < 2; ++k)
                {
                    for (const std::size_t i : footContacts[k])
                    {
                        centroid[k] += contacts[i].mPosition / static_cast<double>(footContacts[k].size());
                        normal[k] += contacts[i].mNormal.Normalized();
                    }
                    normal[k] = normal[k].NormalizedOr(JPH::Vec3::sAxisZ());
                    const JPH::Vec3 leg(input.mBodies[input.mLegAxisHips[k]].mJointPosition - centroid[k]);
                    if (!isFinite(leg) || leg.LengthSq() < 1.0f || leg.GetZ() <= 0.0f)
                        geometryOk = false;
                    else
                        axis[k] = leg.Normalized();
                }
                JPH::Vec3 forwardAxis(centroid[1] - centroid[0]);
                forwardAxis.SetZ(0.0f);
                if (geometryOk && forwardAxis.LengthSq() > 1.0f)
                {
                    forwardAxis = forwardAxis.Normalized();
                    const JPH::Vec3 required = (JPH::Vec3(0, 0, input.mGravity)
                        + JPH::Vec3(request.GetX(), request.GetY(), input.mRequestedVerticalAcceleration))
                        * static_cast<float>(mass);
                    const auto sag = [&](JPH::Vec3Arg v) { return std::pair<float, float>(v.Dot(forwardAxis), v.GetZ()); };
                    const auto [tx, tz] = sag(axis[0]);
                    const auto [lx, lz] = sag(axis[1]);
                    const auto [fx, fz] = sag(required);
                    const float det = tx * lz - tz * lx;
                    if (std::abs(det) > 1e-4f)
                    {
                        const float sTrail = (fx * lz - fz * lx) / det;
                        const float sLead = (tx * fz - tz * fx) / det;
                        if (sTrail >= 0.0f && sLead >= 0.0f)
                        {
                            std::array<JPH::Vec3, 2> footForce{ { axis[0] * sTrail, axis[1] * sLead } };
                            // Whatever the two leg lines cannot carry (the lateral
                            // component) is split by contact weight.
                            JPH::Vec3 remainder = required - footForce[0] - footForce[1];
                            double w0 = 0.0, w1 = 0.0;
                            for (const std::size_t i : footContacts[0]) w0 += weights[i];
                            for (const std::size_t i : footContacts[1]) w1 += weights[i];
                            const float share0 = (w0 + w1) > 0.0 ? static_cast<float>(w0 / (w0 + w1)) : 0.5f;
                            footForce[0] += remainder * share0;
                            footForce[1] += remainder * (1.0f - share0);
                            bool frictionOk = true;
                            for (std::size_t k = 0; k < 2; ++k)
                            {
                                const float n = footForce[k].Dot(normal[k]);
                                const JPH::Vec3 t = footForce[k] - normal[k] * n;
                                if (!(n > 0.0f) || t.LengthSq() > input.mFriction * input.mFriction * n * n)
                                    frictionOk = false;
                            }
                            if (frictionOk)
                            {
                                result.mContactForces.clear();
                                for (std::size_t k = 0; k < 2; ++k)
                                    for (const std::size_t i : footContacts[k])
                                        result.mContactForces.push_back({ contacts[i].mBodyIndex, contacts[i].mPosition,
                                            footForce[k] / static_cast<float>(footContacts[k].size()) });
                                result.mAcceleration = JPH::Vec3(request.GetX(), request.GetY(), input.mRequestedVerticalAcceleration);
                                const float n0 = std::max(footForce[0].GetZ(), 0.0f), n1 = std::max(footForce[1].GetZ(), 0.0f);
                                if (n0 + n1 > 0.0f)
                                    result.mCenterOfPressure = centroid[0] * (n0 / (n0 + n1)) + JPH::RVec3(centroid[1]) * (n1 / (n0 + n1));
                                result.mHullProjectionDelta = JPH::Vec3::sZero();
                                result.mLegAxisApplied = true;
                                if (RagdollCore::tuningText("OPENMW_RAGDOLL_PUSHOFF_DEBUG"))
                                    std::fprintf(stderr, "legaxis sTrail=%.0f sLead=%.0f trailAngle=%.1f leadAngle=%.1f req=(%.0f,%.0f)\n",
                                        sTrail, sLead, std::atan2(tx, tz) * 57.2958f, std::atan2(lx, lz) * 57.2958f, fx, fz);
                            }
                        }
                    }
                }
            }
        }
        const auto effectiveGravity = JPH::Vec3(0, 0, -input.mGravity) - result.mAcceleration;
        const auto totalForce = -effectiveGravity * static_cast<float>(mass);
        for (std::size_t i = 0; i < contacts.size(); ++i)
        {
            if (result.mLegAxisApplied || weights[i] <= 0)
                continue;
            const auto normal = contacts[i].mNormal.Normalized();
            const float normalForce = totalForce.Dot(normal);
            const auto tangent = totalForce - normal * normalForce;
            constexpr float rounding = 16.0f * std::numeric_limits<float>::epsilon();
            if (normalForce <= 0 || tangent.LengthSq() > input.mFriction * input.mFriction
                    * normalForce * normalForce * (1.0f + rounding)
                    + rounding * rounding * totalForce.LengthSq())
                return JointSupportOutput{ .mJointTorques = result.mJointTorques };
            result.mContactForces.push_back({ contacts[i].mBodyIndex, contacts[i].mPosition,
                totalForce * static_cast<float>(weights[i]) });
        }
        std::vector<JPH::Vec3> forces(count), moments(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            forces[i] = effectiveGravity * input.mBodies[i].mMass;
            moments[i] = JPH::Vec3(input.mBodies[i].mCenterOfMass - input.mBodies[i].mJointPosition).Cross(forces[i]);
        }
        if (!borrowMoment.IsNearZero() && isFinite(borrowMoment))
        {
            // Counter-rotation set: every body that is neither a foot nor on
            // a foot's parent chain (scenario rig: trunk, neck, head, arms).
            // Pelvis and legs keep support duty. Point-mass inertia about the
            // COM, same form as the swing inertia feed-forward. The added
            // d'Alembert moments -I*alpha keep the root moment residual at
            // zero and become spine/hip joint torques through the ordinary
            // propagation - internal torques with reactions, not a free
            // external torque.
            std::vector<bool> isSupport(count, false);
            for (const std::size_t foot : { input.mFeet.mLeft, input.mFeet.mRight })
            {
                std::size_t body = foot;
                while (true)
                {
                    isSupport[body] = true;
                    const int parent = input.mBodies[body].mParent;
                    if (parent < 0)
                        break;
                    body = static_cast<std::size_t>(parent);
                }
            }
            const JPH::Vec3 axis = borrowMoment.Normalized();
            double inertiaSum = 0.0;
            for (std::size_t i = 0; i < count; ++i)
            {
                if (isSupport[i])
                    continue;
                const JPH::Vec3 r(input.mBodies[i].mCenterOfMass - center);
                const float radial = r.Dot(axis);
                inertiaSum += input.mBodies[i].mMass * (r.LengthSq() - radial * radial);
            }
            if (inertiaSum > 0.0)
            {
                const JPH::Vec3 alpha = axis * (borrowMoment.Length() / static_cast<float>(inertiaSum));
                for (std::size_t i = 0; i < count; ++i)
                {
                    if (isSupport[i])
                        continue;
                    const JPH::Vec3 r(input.mBodies[i].mCenterOfMass - center);
                    const float bodyMass = input.mBodies[i].mMass;
                    moments[i] -= bodyMass * (r.LengthSq() * alpha - r * r.Dot(alpha));
                }
            }
        }
        for (const auto& contact : result.mContactForces)
        {
            forces[contact.mBodyIndex] += contact.mForce;
            moments[contact.mBodyIndex] += JPH::Vec3(contact.mPosition - input.mBodies[contact.mBodyIndex].mJointPosition)
                .Cross(contact.mForce);
        }
        for (std::size_t i = count - 1; i > 0; --i)
        {
            const auto parent = static_cast<std::size_t>(input.mBodies[i].mParent);
            result.mJointTorques[i] = -moments[i];
            moments[parent] += moments[i]
                + JPH::Vec3(input.mBodies[i].mJointPosition - input.mBodies[parent].mJointPosition).Cross(forces[i]);
            forces[parent] += forces[i];
        }
        result.mRootForceResidual = forces[0];
        result.mRootMomentResidual = moments[0] - JPH::Vec3(center - input.mBodies[0].mJointPosition).Cross(forces[0]);
        if (!isFinite(result.mRootForceResidual) || !isFinite(result.mRootMomentResidual)
            || !isFinite(result.mAcceleration) || !isFinite(result.mCenterOfPressure)
            || std::any_of(result.mJointTorques.begin(), result.mJointTorques.end(),
                [](JPH::Vec3Arg torque) { return !isFinite(torque); }))
            return std::nullopt;
        result.mSupported = true;
        return result;
    }

    std::optional<UprightControllerOutput> calculateUprightControl(
        const UprightControllerInput& input, const ActiveControllerProfile& profile)
    {
        if (!validUprightProfile(profile) || !isFinite(input.mTargetPelvisHeight)
            || input.mTargetPelvisHeight < 0.0f || !isFinite(input.mPelvisHeight) || input.mPelvisHeight < 0.0f
            || !isFinite(input.mPelvisVerticalVelocity) || !isFinite(input.mCenterOfMassOffset)
            || !isFinite(input.mCenterOfMassVelocity) || !isFinite(input.mTargetHorizontalVelocity)
            || !isFinite(input.mTargetUp)
            || !input.mTargetUp.IsNormalized() || !isFinite(input.mPelvisUp) || !input.mPelvisUp.IsNormalized()
            || !isFinite(input.mPelvisAngularVelocity) || !isFinite(input.mTorsoUp)
            || !input.mTorsoUp.IsNormalized() || !isFinite(input.mTorsoAngularVelocity)
            || !isFinite(input.mTotalMass) || input.mTotalMass <= 0.0f
            || (input.mLimitToSupportFriction
                && (!isFinite(input.mSupportFriction) || input.mSupportFriction < 0.0f
                    || !isFinite(input.mGravityMagnitude) || input.mGravityMagnitude <= 0.0f))
            || (input.mLimitToSupportMoment
                && (!input.mLimitToSupportFriction || !isFinite(input.mCenterOfMassPosition)
                    || !isFinite(input.mCenterOfMassHeight) || input.mCenterOfMassHeight <= 0.0f
                    || input.mSupportContactPoints.empty())))
            return std::nullopt;
        if (input.mLimitToSupportMoment)
        {
            for (const JPH::RVec3& point : input.mSupportContactPoints)
            {
                if (!isFinite(point))
                    return std::nullopt;
            }
        }

        constexpr float twoPi = 6.2831853071795864769f;
        const float heightOmega = twoPi * profile.mPelvisHeightFrequency;
        const float centerOfMassOmega = twoPi * profile.mCenterOfMassFrequency;
        const JPH::Vec3 targetVelocity
            = clampHorizontalVelocity(input.mTargetHorizontalVelocity, profile.mMaximumLocomotionSpeed);
        // Height, gravity and support contacts use world Z. The authored
        // recovery orientation can be horizontal; it must steer torques
        // without rotating lift, COM feedback or the friction budget.
        const JPH::Vec3 vertical = JPH::Vec3::sAxisZ();
        JPH::Vec3 horizontalVelocityError = input.mCenterOfMassVelocity - targetVelocity;
        horizontalVelocityError -= vertical * horizontalVelocityError.Dot(vertical);
        JPH::Vec3 acceleration = -input.mCenterOfMassOffset * (centerOfMassOmega * centerOfMassOmega)
            - horizontalVelocityError * (2.0f * profile.mCenterOfMassDamping * centerOfMassOmega);
        acceleration -= vertical * acceleration.Dot(vertical);
        acceleration += vertical
            * ((input.mTargetPelvisHeight - input.mPelvisHeight) * heightOmega * heightOmega
                - input.mPelvisVerticalVelocity * 2.0f * profile.mPelvisHeightDamping * heightOmega);
        if (input.mRecoveryUnweight)
            acceleration = vertical
                * (UnitModel::GravityUnitsPerSecondSquared * profile.mRecoveryUnweightFactor);

        UprightControllerOutput result;
        result.mPelvisForce = clampMagnitude(
            acceleration * input.mTotalMass, profile.mMaximumPelvisForce, result.mPelvisMagnitudeSaturated);
        if (input.mLimitToSupportFriction)
        {
            const float weight = input.mTotalMass * input.mGravityMagnitude;
            JPH::Vec3 horizontalForce = result.mPelvisForce
                - vertical * result.mPelvisForce.Dot(vertical);
            bool horizontalLimited = false;
            horizontalForce = clampMagnitude(
                horizontalForce, input.mSupportFriction * weight, horizontalLimited);
            const float verticalForce = std::clamp(
                result.mPelvisForce.Dot(vertical), -weight, weight);
            const bool verticalLimited
                = verticalForce != result.mPelvisForce.Dot(vertical);
            result.mPelvisForce = horizontalForce + vertical * verticalForce;
            result.mPelvisHorizontalForceSaturated = horizontalLimited;
            result.mPelvisVerticalForceSaturated = verticalLimited;
            if (input.mLimitToSupportMoment && !horizontalForce.IsNearZero())
            {
                const float horizontalLength = horizontalForce.Length();
                const JPH::Vec3 direction = horizontalForce / horizontalLength;
                const auto lever = supportForceLeverInterval(input, direction);
                // Weight is the constant-height model's normal-load budget;
                // internal height force is not a measured contact normal load.
                const float forcePerLever = weight / input.mCenterOfMassHeight;
                const float minimumForce = lever ? lever->first * forcePerLever : 0.0f;
                const float maximumForce = lever ? lever->second * forcePerLever : 0.0f;
                // A clamp may reduce a request; it cannot invent the extra
                // acceleration needed when the COM has already left support.
                // In that case a recovery step or angular momentum is needed.
                const float boundedForce = !lever || horizontalLength < minimumForce
                    ? 0.0f : std::min(horizontalLength, maximumForce);
                if (horizontalLength != boundedForce)
                {
                    horizontalForce *= boundedForce / horizontalLength;
                    result.mPelvisForce = horizontalForce + vertical * verticalForce;
                    result.mPelvisSupportMomentSaturated = true;
                }
            }
        }
        result.mPelvisForceSaturated = result.mPelvisMagnitudeSaturated
            || result.mPelvisHorizontalForceSaturated || result.mPelvisSupportMomentSaturated
            || result.mPelvisVerticalForceSaturated;
        result.mPelvisTorque = clampMagnitude(
            uprightTorque(input.mPelvisUp, input.mTargetUp, input.mPelvisAngularVelocity,
                profile.mPelvisUprightStiffness, profile.mPelvisUprightDamping),
            profile.mMaximumPelvisTorque, result.mPelvisTorqueSaturated);
        result.mSpineTorque = clampMagnitude(
            uprightTorque(input.mTorsoUp, input.mTargetUp, input.mTorsoAngularVelocity,
                profile.mSpineUprightStiffness, profile.mSpineUprightDamping),
            profile.mMaximumSpineTorque, result.mSpineTorqueSaturated);
        if (!isFinite(result.mPelvisForce) || !isFinite(result.mPelvisTorque) || !isFinite(result.mSpineTorque))
            return std::nullopt;
        return result;
    }

    bool shouldUseAnimationAssistedActuation(LocomotionRoute route, ActiveControllerMode mode,
        bool driveJointMotors, bool locomotionInterrupted)
    {
        return route == LocomotionRoute::KinematicRoot
            && driveJointMotors
            && !locomotionInterrupted
            && (mode == ActiveControllerMode::Standing || mode == ActiveControllerMode::Bracing
                || mode == ActiveControllerMode::Stumbling);
    }

    std::optional<StanceContactPositionOutput> calculateStanceContactPositionTarget(
        const StanceContactPositionInput& input)
    {
        if (!isFinite(input.mAnchorPosition) || !isFinite(input.mDesiredFootOrigin)
            || !isFinite(input.mDesiredFootRotation) || !input.mDesiredFootRotation.IsNormalized()
            || !isFinite(input.mContactOffsetInFoot) || !isFinite(input.mCorrectionFraction)
            || input.mCorrectionFraction < 0.0f || input.mCorrectionFraction > 1.0f
            || !isFinite(input.mMaximumEndpointCorrection) || input.mMaximumEndpointCorrection <= 0.0f)
            return std::nullopt;
        StanceContactPositionOutput result;
        result.mDesiredContactPosition
            = input.mDesiredFootOrigin + input.mDesiredFootRotation * input.mContactOffsetInFoot;
        result.mContactError = JPH::Vec3(input.mAnchorPosition - result.mDesiredContactPosition);
        const float errorLength = result.mContactError.Length();
        if (!isFinite(errorLength))
            return std::nullopt;
        JPH::Vec3 bounded = result.mContactError;
        if (errorLength > input.mMaximumEndpointCorrection)
        {
            bounded *= input.mMaximumEndpointCorrection / errorLength;
            result.mSaturated = true;
        }
        result.mEndpointCorrection = bounded * input.mCorrectionFraction;
        result.mTargetFootOrigin = input.mDesiredFootOrigin + result.mEndpointCorrection;
        if (!isFinite(result.mEndpointCorrection) || !isFinite(result.mTargetFootOrigin))
            return std::nullopt;
        return result;
    }

    bool SettledPelvisHeightTracker::step(const SettledPelvisHeightInput& input, float duration,
        const ActiveControllerProfile& profile, SettledPelvisHeightOutput& output)
    {
        if (!isFinite(duration) || duration <= 0.0f || !validProfile(profile)
            || !validSupport(input.mSupport) || !isFinite(input.mPelvisHeight) || input.mPelvisHeight < 0.0f
            || !isFinite(input.mPelvisVerticalVelocity))
            return false;

        std::optional<float> targetHeight = mTargetHeight;
        float candidateTime = mCandidateTime;
        bool candidateValid = mCandidateValid;
        bool settled = mSettled;
        bool frozen = mFrozen;
        if (!frozen && input.mLocomotionRequested && settled)
            frozen = true;
        if (!frozen)
        {
            const bool supported = input.mSupport.hasSupport() && input.mSupport.mCenterOfMassSupported;
            // Contact begins while a newly inserted body is still descending
            // and may briefly be classified as bracing. Publish the first
            // physically supported height so upright damping can arrest that
            // motion; requiring an already quiet standing body here creates a
            // circular dependency and lets the body collapse before settling.
            if (!targetHeight && supported)
                targetHeight = input.mPelvisHeight;
            // Feet on the ground are enough to calibrate the standing height:
            // with a warhammer held out the centre of mass sat past the support
            // hull, the height never settled, locomotion (gated on it) never
            // engaged and the legs ran stiff - feet lifting 10-12 units against
            // the clip's 47 - until a redraw happened to settle it (play test
            // 2026-09-24 N9).
            candidateValid = input.mSupport.hasSupport() && input.mMode == ActiveControllerMode::Standing
                && std::abs(input.mPelvisVerticalVelocity)
                    <= profile.mPelvisHeightSettleMaximumVerticalSpeed;
            if (candidateValid)
            {
                candidateTime += duration;
                targetHeight = input.mPelvisHeight;
                settled = candidateTime >= profile.mPelvisHeightSettleDuration;
            }
            else
            {
                candidateTime = 0.0f;
                settled = false;
            }
            // An early movement request cannot freeze an invalid insertion
            // height. Keep observing the stationary physical pose until a
            // settled target exists; adapters can withhold gait actuation in
            // that short interval.
            if (input.mLocomotionRequested && settled)
                frozen = true;
        }

        mTargetHeight = targetHeight;
        mCandidateTime = candidateTime;
        mCandidateValid = candidateValid;
        mSettled = settled;
        mFrozen = frozen;
        output = getOutput();
        return true;
    }

    void SettledPelvisHeightTracker::reset()
    {
        mTargetHeight.reset();
        mCandidateTime = 0.0f;
        mCandidateValid = false;
        mSettled = false;
        mFrozen = false;
    }

    SettledPelvisHeightOutput SettledPelvisHeightTracker::getOutput() const
    {
        SettledPelvisHeightOutput result;
        result.mTargetHeight = mTargetHeight;
        result.mCandidateTime = mCandidateTime;
        result.mCandidateValid = mCandidateValid;
        result.mSettled = mSettled;
        result.mFrozen = mFrozen;
        return result;
    }

    bool ActiveRagdollController::step(const ActiveControllerInput& input, float duration,
        const ActiveControllerProfile& profile, ActiveControllerOutput& output)
    {
        if (!isFinite(duration) || duration <= 0.0f || !validProfile(profile) || !validSupport(input.mSupport)
            || !isFinite(input.mPelvisUpDot) || input.mPelvisUpDot < -1.0f || input.mPelvisUpDot > 1.0f
            || !isFinite(input.mTorsoUpDot) || input.mTorsoUpDot < -1.0f || input.mTorsoUpDot > 1.0f
            || !isFinite(input.mTorsoFacingUpDot) || input.mTorsoFacingUpDot < -1.0f || input.mTorsoFacingUpDot > 1.0f
            || !isFinite(input.mPelvisHeight) || input.mPelvisHeight < 0.0f
            || !isFinite(input.mCenterOfMassVelocity) || !isFinite(input.mTargetHorizontalVelocity)
            || (input.mOwnedRootVerticalVelocity && !isFinite(*input.mOwnedRootVerticalVelocity)))
            return false;

        mTimeInMode += duration;
        const JPH::Vec3 targetVelocity
            = clampHorizontalVelocity(input.mTargetHorizontalVelocity, profile.mMaximumLocomotionSpeed);
        const float centerOfMassSpeed = input.mCenterOfMassVelocity.Length();
        const float trackingErrorSpeed = (input.mCenterOfMassVelocity - targetVelocity).Length();
        float standingTrackingErrorSpeed = trackingErrorSpeed;
        if (profile.mLocomotionRoute == LocomotionRoute::KinematicRoot
            && input.mRootExternallyOwned && input.mOwnedRootVerticalVelocity)
        {
            // The animation owns vertical root motion. Compare the body's
            // response to that measured motion, retaining the original
            // horizontal command error and all physical-route observations.
            JPH::Vec3 error = input.mCenterOfMassVelocity - targetVelocity;
            error.SetZ(error.GetZ() - *input.mOwnedRootVerticalVelocity);
            standingTrackingErrorSpeed = error.Length();
        }
        const bool locomoting = !targetVelocity.IsNearZero();
        const bool footSupported = input.mSupport.hasSupport();
        const bool grounded = input.mSupport.hasGroundContact();
        const bool gaitSupported = locomoting && grounded;
        if (footSupported)
            mHasObservedFootSupport = true;
        if (footSupported || gaitSupported)
            mUnsupportedTime = 0.0f;
        else
            mUnsupportedTime += duration;
        const bool activationSupportGrace = !mHasObservedFootSupport
            && mUnsupportedTime <= profile.mActivationSupportGraceDuration;
        const bool gaitSupportGrace = locomoting
            && mUnsupportedTime <= profile.mLocomotionSupportGraceDuration;
        // The kinematic-root route is externally stabilized: the actor owns
        // the root, so neither a support gap nor an animation-driven torso
        // lean is a physical fall. A knock-down is decided by the hit
        // reaction band, which releases the root to physics; only then do the
        // physical observations below apply. While the root is still held, a
        // hit stun is not a fall either: the feet of a held body carry little
        // load, and reading that as lost support dropped the victim of every
        // landed blow (35 releases in 164 s of duelling, 2026-09-18).
        const bool kinematicRootUpright
            = profile.mLocomotionRoute == LocomotionRoute::KinematicRoot
            && (input.mRootExternallyOwned || !input.mActuationInterrupted);
        const bool standing = (footSupported && input.mSupport.mCenterOfMassSupported
                && input.mPelvisUpDot >= profile.mStandingMinimumUpDot
                && standingTrackingErrorSpeed <= profile.mStandingMaximumSpeed)
            // The kinematic route holds the root externally; the animation is
            // the authority on whether the actor stands, and contact-derived
            // support can read zero while the body is animation-held upright.
            || (input.mRootExternallyOwned && input.mPelvisUpDot >= profile.mStandingMinimumUpDot
                && standingTrackingErrorSpeed <= profile.mStandingMaximumSpeed);
        const bool bracing = (footSupported || gaitSupported || activationSupportGrace || gaitSupportGrace
                                 || kinematicRootUpright)
            && input.mPelvisUpDot >= profile.mBracingMinimumUpDot;
        const bool recoveryStanding = (footSupported && input.mSupport.mCenterOfMassSupported
                // The ordinary standing margin tolerates a small overhang, but
                // recovery must not release assistance outside the contact hull.
                && input.mSupport.mCenterOfMassDistance == 0.0f
                && input.mPelvisUpDot >= profile.mStandingMinimumUpDot
                && centerOfMassSpeed <= std::min(profile.mStandingMaximumSpeed, profile.mSettledMaximumSpeed)
                && input.mPelvisHeight > profile.mFallenMaximumPelvisHeight)
            // An externally owned root is held by the animation itself; the
            // body is standing even while the physical feet carry no load.
            // The measured pelvis height is contact-derived and reads zero
            // exactly then, so only the upright axis is required.
            || (input.mRootExternallyOwned && input.mPelvisUpDot >= profile.mStandingMinimumUpDot)
            // An actor already walking upright on supported feet is recovered
            // even though the strict settle margin is unmet: locomotion intent
            // is external and can arrive before the blend has come to rest.
            // Without this, a walking actor never leaves BlendBack and its
            // recovery never completes (the duel opponent loops forever).
            // Kinematic-root route only: there the actor holds the body up.
            // On the dynamic routes a speed that is fine for locomoting
            // Standing is still too fast to release recovery assistance
            // (recoveryQualifiesMeasuredRestRatherThanMatchingWalkingInput).
            || (profile.mLocomotionRoute == LocomotionRoute::KinematicRoot && locomoting && footSupported
                && input.mSupport.mCenterOfMassSupported
                && input.mPelvisUpDot >= profile.mStandingMinimumUpDot
                && input.mPelvisHeight > profile.mFallenMaximumPelvisHeight);
        if (mMode == ActiveControllerMode::BlendBack && recoveryStanding)
            mRecoveryStandingTime += duration;
        else
            mRecoveryStandingTime = 0.0f;
        const bool settled = grounded && centerOfMassSpeed <= profile.mSettledMaximumSpeed
            && input.mPelvisHeight <= profile.mFallenMaximumPelvisHeight;
        const bool recoveryStallCandidate = mMode == ActiveControllerMode::BlendBack && grounded
            && centerOfMassSpeed <= profile.mSettledMaximumSpeed
            && !recoveryStanding;
        // GettingUp follows the authored animation and may briefly clear the
        // floor. Once physical settling owns the pose, sustained loss of all
        // measured ground contact means the rise was interrupted. Release the
        // motors after the bounded recovery timeout so the body can fall and a
        // later recovery request can start a clean retry.
        const bool recoverySupportLost = mMode == ActiveControllerMode::BlendBack && !grounded
            && mUnsupportedTime >= profile.mRecoveryHandoverTimeout;
        bool recoveryProgress = false;
        if (mMode == ActiveControllerMode::BlendBack && input.mPelvisUpDot > mRecoveryProgressUp + 0.01f)
        {
            mRecoveryProgressUp = input.mPelvisUpDot;
            recoveryProgress = true;
        }
        if (mMode == ActiveControllerMode::BlendBack && footSupported
            && (!mRecoveryProgressDistance
                || input.mSupport.mCenterOfMassDistance < *mRecoveryProgressDistance - 0.1f))
        {
            mRecoveryProgressDistance = input.mSupport.mCenterOfMassDistance;
            recoveryProgress = true;
        }
        if (recoveryStallCandidate && !recoveryProgress)
            mRecoveryStallTime += duration;
        else
            mRecoveryStallTime = 0.0f;
        const bool fallingExitCandidate = footSupported && input.mSupport.mCenterOfMassSupported
            && input.mPelvisUpDot >= profile.mBracingMinimumUpDot
            && input.mPelvisHeight > profile.mFallenMaximumPelvisHeight
            && std::abs(input.mCenterOfMassVelocity.GetZ()) <= profile.mFallingExitMaximumVerticalSpeed;
        if (mMode == ActiveControllerMode::Falling && fallingExitCandidate)
            mFallingExitCandidateTime += duration;
        else
            mFallingExitCandidateTime = 0.0f;
        const bool mayExitFalling
            = mFallingExitCandidateTime >= profile.mFallingExitSupportDuration;
        if ((mMode == ActiveControllerMode::GettingUp || mMode == ActiveControllerMode::BlendBack)
            && input.mPelvisUpDot >= profile.mBracingMinimumUpDot)
            mRecoveryHeightAssist = true;

        ActiveControllerMode requested = mMode;
        std::optional<RecoveryDirection> requestedRecoveryDirection;
        switch (mMode)
        {
            case ActiveControllerMode::Standing:
            case ActiveControllerMode::Bracing:
            case ActiveControllerMode::Stumbling:
                if (standing)
                    requested = ActiveControllerMode::Standing;
                else if (settled)
                    requested = ActiveControllerMode::Fallen;
                else if (bracing)
                    requested = ActiveControllerMode::Bracing;
                // A foot can remain on the ground after the pelvis has tipped
                // horizontal or inverted. That contact cannot justify driving
                // the standing pose; leave authored get-up/blend states alone.
                else if (footSupported && input.mPelvisUpDot > 0.0f)
                    requested = ActiveControllerMode::Stumbling;
                else
                    requested = ActiveControllerMode::Falling;
                break;
            case ActiveControllerMode::Falling:
            {
                // A body propped up off the floor (half sitting, draped over a
                // step or a body) is never "settled" (pelvis above 30 units)
                // and cannot stand: it stayed Falling for 12 s with no get-up
                // (build 61 bookmark 19, "stuck lying down"; the stuck stumble
                // after a hard landing). After `seconds` grounded, slower than
                // `speed` and not standing, it counts as lying and may get up.
                // OPENMW_RAGDOLL_FALLING_TIMEOUT = "seconds, speed" (0 = off).
                static const auto fallingTimeout_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
                    std::array<float, 2> value{ 0.0f, 60.0f };
                    if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_FALLING_TIMEOUT"))
                        std::sscanf(text, "%f,%f", &value[0], &value[1]);
                    return value;
                });
                static const auto& fallingTimeout = fallingTimeout_tuned.get();
                if (settled)
                    requested = ActiveControllerMode::Fallen;
                else if (mayExitFalling && standing)
                    requested = ActiveControllerMode::Standing;
                else if (mayExitFalling && bracing)
                    requested = ActiveControllerMode::Bracing;
                else if (fallingTimeout[0] > 0.0f && mTimeInMode >= fallingTimeout[0] && grounded && !standing
                    && centerOfMassSpeed <= fallingTimeout[1])
                    requested = ActiveControllerMode::Fallen;
                break;
            }
            case ActiveControllerMode::Fallen:
                if (!grounded)
                    requested = ActiveControllerMode::Falling;
                else if (input.mRecoveryRequested && mTimeInMode >= profile.mRecoveryDelay)
                {
                    requested = ActiveControllerMode::GettingUp;
                    requestedRecoveryDirection = input.mForcedRecoveryDirection
                        ? *input.mForcedRecoveryDirection
                        : (input.mTorsoFacingUpDot >= 0.0f ? RecoveryDirection::FaceUp : RecoveryDirection::FaceDown);
                }
                break;
            case ActiveControllerMode::GettingUp:
                // A get-up animation can legitimately lift every body clear of
                // the ground for a few fixed steps. The animation completion
                // signal, rather than transient contact loss, owns this phase.
                if (input.mRecoveryInterrupted)
                    requested = ActiveControllerMode::Falling;
                else if (input.mRecoveryAnimationFinished)
                    requested = ActiveControllerMode::BlendBack;
                break;
            case ActiveControllerMode::BlendBack:
                // This legacy-named phase is physical settling. Require
                // continuous supported standing, not elapsed animation time
                // plus one briefly upright observation.
                if (settled && input.mPelvisUpDot < profile.mBracingMinimumUpDot
                    && mTimeInMode >= profile.mRecoveryDelay)
                    requested = ActiveControllerMode::Fallen;
                else if (input.mRecoveryInterrupted || recoverySupportLost)
                    requested = ActiveControllerMode::Falling;
                else if (recoveryStallCandidate && mRecoveryStallTime > 0.0f
                    && mRecoveryStallTime >= std::max(profile.mRecoveryDelay, profile.mBlendBackDuration))
                    requested = ActiveControllerMode::Falling;
                else if (recoveryStanding && mRecoveryStandingTime >= profile.mBlendBackDuration)
                    requested = ActiveControllerMode::Standing;
                break;
        }

        if (requested != mMode && mTimeInMode >= profile.mMinimumStateDuration)
        {
            mMode = requested;
            mTimeInMode = 0.0f;
            mFallingExitCandidateTime = 0.0f;
            mRecoveryStandingTime = 0.0f;
            mRecoveryStallTime = 0.0f;
            mRecoveryProgressUp = -1.0f;
            mRecoveryProgressDistance.reset();
            if (mMode == ActiveControllerMode::GettingUp)
            {
                mRecoveryHeightAssist = false;
                mRecoveryDirection = requestedRecoveryDirection;
            }
            else if (mMode != ActiveControllerMode::BlendBack)
            {
                mRecoveryHeightAssist = false;
                mRecoveryDirection.reset();
            }
        }
        output = getOutput();
        return true;
    }

    void ActiveRagdollController::reset(ActiveControllerMode mode)
    {
        mMode = mode;
        mTimeInMode = 0.0f;
        mUnsupportedTime = 0.0f;
        mHasObservedFootSupport = false;
        mFallingExitCandidateTime = 0.0f;
        mRecoveryHeightAssist = false;
        mRecoveryStandingTime = 0.0f;
        mRecoveryStallTime = 0.0f;
        mRecoveryProgressUp = -1.0f;
        mRecoveryProgressDistance.reset();
        mRecoveryDirection.reset();
    }

    ActiveControllerOutput ActiveRagdollController::getOutput() const
    {
        ActiveControllerOutput result;
        result.mMode = mMode;
        result.mTimeInMode = mTimeInMode;
        result.mFallingExitCandidateTime = mFallingExitCandidateTime;
        result.mDriveJointMotors = mMode != ActiveControllerMode::Falling && mMode != ActiveControllerMode::Fallen;
        result.mRecoveryHeightAssist = mRecoveryHeightAssist;
        result.mRecoveryDirection = mRecoveryDirection;
        return result;
    }
}
