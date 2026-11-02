#include <ragdollcore/limbclearance.hpp>

#include <Jolt/Math/Mat44.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>

#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

namespace RagdollCore
{
    namespace
    {
        bool valid(const LimbShapePose& pose)
        {
            if (!pose.mShape || !std::isfinite(pose.mPosition.GetX())
                || !std::isfinite(pose.mPosition.GetY()) || !std::isfinite(pose.mPosition.GetZ()))
                return false;
            const float norm = pose.mRotation.LengthSq();
            return std::isfinite(norm) && std::abs(norm - 1.0f) < 1e-4f;
        }
    }

    std::optional<LimbClearance> measureLimbClearance(std::span<const LimbShapePose> first,
        std::span<const LimbShapePose> second, float margin)
    {
        if (first.empty() || second.empty() || !std::isfinite(margin) || margin < 0.0f)
            return std::nullopt;
        for (const auto& pose : first) if (!valid(pose)) return std::nullopt;
        for (const auto& pose : second) if (!valid(pose)) return std::nullopt;
        JPH::CollideShapeSettings settings;
        settings.mMaxSeparationDistance = margin;
        LimbClearance result;
        for (std::size_t i = 0; i < first.size(); ++i)
            for (std::size_t j = 0; j < second.size(); ++j)
            {
                const auto& a = first[i];
                const auto& b = second[j];
                // Subtract a nearby origin BEFORE narrowing to Jolt float
                // collision coordinates, preserving large-world precision.
                const JPH::Vec3 localA = a.mRotation * a.mShape->GetCenterOfMass();
                const JPH::Vec3 localB = JPH::Vec3(b.mPosition - a.mPosition)
                    + b.mRotation * b.mShape->GetCenterOfMass();
                if (!localA.IsNaN() && !localB.IsNaN()
                    && std::isfinite(localA.LengthSq()) && std::isfinite(localB.LengthSq()))
                {
                    JPH::ClosestHitCollisionCollector<JPH::CollideShapeCollector> collector;
                    JPH::CollisionDispatch::sCollideShapeVsShape(a.mShape, b.mShape,
                        JPH::Vec3::sReplicate(1.0f), JPH::Vec3::sReplicate(1.0f),
                        JPH::Mat44::sRotationTranslation(a.mRotation, localA),
                        JPH::Mat44::sRotationTranslation(b.mRotation, localB), {}, {}, settings, collector);
                    ++result.mTestedPairs;
                    if (collector.HadHit())
                    {
                        const float gap = -collector.mHit.mPenetrationDepth;
                        if (!std::isfinite(gap)) return std::nullopt;
                        result.mClear = false;
                        if (!result.mMinimumReportedGap || gap < *result.mMinimumReportedGap)
                        {
                            result.mMinimumReportedGap = gap;
                            result.mFirstIndex = i;
                            result.mSecondIndex = j;
                        }
                    }
                }
                else return std::nullopt;
            }
        return result;
    }

    static std::optional<ClearLimbPose> findLimbPoseImpl(const LimbPoseSearchInput& input,
        std::span<const LimbShapePose> support, float margin, float maximumFootDisplacement,
        const LimbPoseContinuity* continuity, std::vector<ClearLimbPose>* alternatives)
    {
        if (support.empty() || !std::isfinite(margin) || margin < 0
            || (input.mMinimumWorldZ && !std::isfinite(*input.mMinimumWorldZ))) return std::nullopt;
        for (const auto* shape : input.mShapes)
            if (!valid({ shape, input.mHipPosition, input.mHipRotation })) return std::nullopt;
        for (const auto& pose : support) if (!valid(pose)) return std::nullopt;
        const auto& leg = input.mLeg;
        const auto finiteVector = [](JPH::Vec3Arg value) {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        };
        const auto unitRotation = [](JPH::QuatArg value) {
            return std::isfinite(value.LengthSq()) && std::abs(value.LengthSq() - 1.0f) < 1e-4f;
        };
        if (!std::isfinite(maximumFootDisplacement) || maximumFootDisplacement < 0
            || !std::isfinite(input.mScale) || input.mScale <= 0
            || !std::isfinite(input.mKneeAngleFromReference) || !std::isfinite(leg.mMinimumAngle)
            || !std::isfinite(leg.mMaximumAngle) || leg.mMinimumAngle > leg.mMaximumAngle
            || !finiteVector(leg.mUpperOffset) || !finiteVector(leg.mLowerOffset)
            || !finiteVector(leg.mHingeAxis) || std::abs(leg.mHingeAxis.LengthSq() - 1.0f) > 1e-4f
            || !unitRotation(leg.mLowerRotation) || !unitRotation(input.mParentRotation)
            || !unitRotation(input.mHipRotation) || !unitRotation(input.mFootRotation))
            return std::nullopt;
        const float length = (leg.mUpperOffset.Length() + leg.mLowerOffset.Length()) * input.mScale;
        if (!std::isfinite(length) || length <= 0) return std::nullopt;
        const JPH::Vec3 lower = leg.mLowerRotation.Conjugated() * leg.mLowerOffset;
        const auto nominalCalf = (input.mHipRotation * JPH::Quat::sRotation(leg.mHingeAxis,
            input.mKneeAngleFromReference) * leg.mLowerRotation).Normalized();
        const auto requestedKnee = input.mHipPosition + JPH::RVec3(input.mHipRotation * leg.mUpperOffset * input.mScale);
        const auto requestedFoot = requestedKnee + JPH::RVec3(nominalCalf * lower * input.mScale);
        std::optional<ClearLimbPose> best;
        float bestDistance2 = std::numeric_limits<float>::infinity();
        std::size_t evaluated = 0;
        const auto evaluate = [&](JPH::QuatArg desiredHip, float desiredKnee) {
            ++evaluated;
            const auto hip = clampSwingTwistJointOrientation(input.mHipLimits, input.mParentRotation, desiredHip);
            if (!hip) return;
            const float knee = std::clamp(desiredKnee, leg.mMinimumAngle, leg.mMaximumAngle);
            const auto calf = (*hip * JPH::Quat::sRotation(leg.mHingeAxis, knee) * leg.mLowerRotation).Normalized();
            const auto ankle = clampSwingTwistJointOrientation(input.mAnkleLimits, calf, input.mFootRotation);
            if (!ankle) return;
            ClearLimbPose candidate;
            candidate.mKneeAngleFromReference = knee;
            candidate.mPoses[0] = { input.mShapes[0], input.mHipPosition, *hip };
            candidate.mPoses[1] = { input.mShapes[1], input.mHipPosition
                + JPH::RVec3(*hip * leg.mUpperOffset * input.mScale), calf };
            candidate.mPoses[2] = { input.mShapes[2], candidate.mPoses[1].mPosition
                + JPH::RVec3(calf * lower * input.mScale), *ankle };
            const float distance2 = JPH::Vec3(candidate.mPoses[2].mPosition - requestedFoot).LengthSq();
            if (!std::isfinite(distance2) || distance2 > maximumFootDisplacement * maximumFootDisplacement
                || (!alternatives && distance2 >= bestDistance2)) return;
            if (input.mMinimumWorldZ)
                for (const auto& pose : candidate.mPoses)
                {
                    const auto bounds = pose.mShape->GetWorldSpaceBounds(
                        JPH::Mat44::sRotationTranslation(pose.mRotation, pose.mRotation * pose.mShape->GetCenterOfMass()),
                        JPH::Vec3::sReplicate(1.0f));
                    if (pose.mPosition.GetZ() + bounds.mMin.GetZ() < *input.mMinimumWorldZ) return;
                }
            const auto clearance = measureLimbClearance(candidate.mPoses, support, margin);
            if (!clearance || !clearance->mClear) return;
            if (continuity && !isLimbTransitionClear(input, candidate, support, margin, *continuity)) return;
            candidate.mFootDisplacement = std::sqrt(distance2);
            if (alternatives) alternatives->push_back(candidate);
            if (distance2 < bestDistance2)
            {
                best = candidate;
                bestDistance2 = distance2;
            }
        };
        evaluate(input.mHipRotation, input.mKneeAngleFromReference);
        if (!alternatives && best && bestDistance2 == 0) { best->mEvaluatedCandidates = evaluated; return best; }
        const float radius = std::min(0.5f, maximumFootDisplacement / length);
        for (int ring = 1; ring <= 4; ++ring)
            for (int x = -1; x <= 1; ++x)
                for (int y = -1; y <= 1; ++y)
                    for (int z = -1; z <= 1; ++z)
                        for (int bend = -1; bend <= 1; ++bend)
                        {
                            if (x == 0 && y == 0 && z == 0 && bend == 0) continue;
                            const JPH::Vec3 vector(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
                            const float angle = radius * ring / 4.0f;
                            const auto rotation = vector.IsNearZero() ? JPH::Quat::sIdentity()
                                : JPH::Quat::sRotation(vector.Normalized(), angle);
                            evaluate((input.mHipRotation * rotation).Normalized(),
                                input.mKneeAngleFromReference + bend * 2.0f * angle);
                        }
        if (alternatives)
            for (auto& candidate : *alternatives) candidate.mEvaluatedCandidates = evaluated;
        if (best) best->mEvaluatedCandidates = evaluated;
        return best;
    }


    std::optional<ClearLimbPose> findClearLimbPose(const LimbPoseSearchInput& input,
        std::span<const LimbShapePose> support, float margin, float maximumFootDisplacement,
        const LimbPoseContinuity* continuity)
    {
        return findLimbPoseImpl(input, support, margin, maximumFootDisplacement, continuity, nullptr);
    }

    std::vector<ClearLimbPose> enumerateClearLimbPoses(const LimbPoseSearchInput& input,
        std::span<const LimbShapePose> support, float margin, float maximumFootDisplacement)
    {
        std::vector<ClearLimbPose> result;
        findLimbPoseImpl(input, support, margin, maximumFootDisplacement, nullptr, &result);
        return result;
    }

    std::optional<LimbPoseRates> calculateLimbPoseRates(
        const ClearLimbPose& from, const ClearLimbPose& to, float interval)
    {
        if (!std::isfinite(interval) || interval <= 0
            || !std::isfinite(from.mKneeAngleFromReference) || !std::isfinite(to.mKneeAngleFromReference))
            return std::nullopt;
        for (std::size_t i = 0; i < 3; ++i)
            if (!valid(from.mPoses[i]) || !valid(to.mPoses[i])
                || from.mPoses[i].mShape != to.mPoses[i].mShape) return std::nullopt;
        const auto rate = [&](JPH::QuatArg a, JPH::QuatArg b) {
            JPH::Quat delta = (b * a.Conjugated()).Normalized();
            if (delta.GetW() < 0) delta = -delta;
            // atan2 retains small-angle precision; acos(w) can round a
            // near-identity float quaternion to zero before dividing by dt.
            const JPH::Vec3 vector(delta.GetX(), delta.GetY(), delta.GetZ());
            const float length = vector.Length();
            return length > 0 ? vector * (2.0f * std::atan2(length, delta.GetW()) / (length * interval))
                              : JPH::Vec3::sZero();
        };
        LimbPoseRates result;
        result.mHipWorld = rate(from.mPoses[0].mRotation, to.mPoses[0].mRotation);
        result.mKnee = (to.mKneeAngleFromReference - from.mKneeAngleFromReference) / interval;
        result.mAnkleParent = rate(
            (from.mPoses[1].mRotation.Conjugated() * from.mPoses[2].mRotation).Normalized(),
            (to.mPoses[1].mRotation.Conjugated() * to.mPoses[2].mRotation).Normalized());
        if (!std::isfinite(result.mHipWorld.LengthSq()) || !std::isfinite(result.mKnee)
            || !std::isfinite(result.mAnkleParent.LengthSq())) return std::nullopt;
        return result;
    }

    bool isLimbTransitionClear(const LimbPoseSearchInput& input, const ClearLimbPose& candidate,
        std::span<const LimbShapePose> support, float margin, const LimbPoseContinuity& continuity)
    {
        const auto& previous = continuity.mPrevious;
        const auto& leg = input.mLeg;
        const auto finite = [](JPH::Vec3Arg v) {
            return std::isfinite(v.GetX()) && std::isfinite(v.GetY()) && std::isfinite(v.GetZ());
        };
        const auto normalized = [](JPH::QuatArg q) {
            return std::isfinite(q.LengthSq()) && std::abs(q.LengthSq() - 1) < 1e-4f;
        };
        if (support.empty() || support.size() != continuity.mPreviousSupport.size()
            || continuity.mIntervals < 2 || continuity.mIntervals > 64
            || !std::isfinite(margin) || margin < 0 || !std::isfinite(input.mScale) || input.mScale <= 0
            || !std::isfinite(continuity.mMaximumFootTravel) || continuity.mMaximumFootTravel < 0
            || !std::isfinite(continuity.mMaximumJointRotation) || continuity.mMaximumJointRotation < 0
            || !std::isfinite(previous.mKneeAngleFromReference) || !std::isfinite(candidate.mKneeAngleFromReference)
            || !std::isfinite(leg.mMinimumAngle) || !std::isfinite(leg.mMaximumAngle)
            || leg.mMinimumAngle > leg.mMaximumAngle || !finite(leg.mUpperOffset) || !finite(leg.mLowerOffset)
            || !finite(leg.mHingeAxis) || std::abs(leg.mHingeAxis.LengthSq() - 1) > 1e-4f
            || !normalized(leg.mLowerRotation) || !normalized(input.mParentRotation)
            || !normalized(continuity.mPreviousParentRotation)
            || (input.mMinimumWorldZ && !std::isfinite(*input.mMinimumWorldZ))) return false;
        for (std::size_t i = 0; i < 3; ++i)
            if (!valid(previous.mPoses[i]) || !valid(candidate.mPoses[i])
                || previous.mPoses[i].mShape != candidate.mPoses[i].mShape
                || input.mShapes[i] != candidate.mPoses[i].mShape) return false;
        for (std::size_t i = 0; i < support.size(); ++i)
            if (!valid(support[i]) || !valid(continuity.mPreviousSupport[i])
                || support[i].mShape != continuity.mPreviousSupport[i].mShape) return false;
        const auto angle = [](JPH::Quat a, JPH::Quat b) {
            return 2.0f * std::acos(std::clamp(std::abs(a.Normalized().Dot(b.Normalized())), 0.0f, 1.0f));
        };
        const float footTravel = JPH::Vec3(candidate.mPoses[2].mPosition - previous.mPoses[2].mPosition).Length();
        if (!std::isfinite(footTravel) || footTravel > continuity.mMaximumFootTravel
            || std::abs(candidate.mKneeAngleFromReference - previous.mKneeAngleFromReference)
                > continuity.mMaximumJointRotation
            || angle(continuity.mPreviousParentRotation.Conjugated() * previous.mPoses[0].mRotation,
                input.mParentRotation.Conjugated() * candidate.mPoses[0].mRotation) > continuity.mMaximumJointRotation
            || angle(previous.mPoses[1].mRotation.Conjugated() * previous.mPoses[2].mRotation,
                candidate.mPoses[1].mRotation.Conjugated() * candidate.mPoses[2].mRotation)
                > continuity.mMaximumJointRotation) return false;
        const auto unchanged = [](const std::optional<JPH::Quat>& clamped, JPH::QuatArg rotation) {
            return clamped && (clamped->IsClose(rotation, 1e-10f) || clamped->IsClose(-rotation, 1e-10f));
        };
        const auto lower = leg.mLowerRotation.Conjugated() * leg.mLowerOffset;
        std::vector<LimbShapePose> interpolatedSupport(support.size());
        for (unsigned sample = 0; sample <= continuity.mIntervals; ++sample)
        {
            const float t = static_cast<float>(sample) / continuity.mIntervals;
            const auto parent = continuity.mPreviousParentRotation.SLERP(input.mParentRotation, t).Normalized();
            const auto hip = previous.mPoses[0].mRotation.SLERP(candidate.mPoses[0].mRotation, t).Normalized();
            const float knee = std::lerp(previous.mKneeAngleFromReference, candidate.mKneeAngleFromReference, t);
            if (knee < leg.mMinimumAngle || knee > leg.mMaximumAngle
                || !unchanged(clampSwingTwistJointOrientation(input.mHipLimits, parent, hip), hip)) return false;
            const auto calf = (hip * JPH::Quat::sRotation(leg.mHingeAxis, knee) * leg.mLowerRotation).Normalized();
            // Interpolate the ankle in the SAME parent space as its velocity
            // reference. World-foot SLERP would describe a different path.
            const auto ankleFrom = (previous.mPoses[1].mRotation.Conjugated()
                * previous.mPoses[2].mRotation).Normalized();
            const auto ankleTo = (candidate.mPoses[1].mRotation.Conjugated()
                * candidate.mPoses[2].mRotation).Normalized();
            const auto ankle = (calf * ankleFrom.SLERP(ankleTo, t)).Normalized();
            if (!unchanged(clampSwingTwistJointOrientation(input.mAnkleLimits, calf, ankle), ankle)) return false;
            std::array<LimbShapePose, 3> poses;
            poses[0] = { input.mShapes[0], previous.mPoses[0].mPosition
                + (candidate.mPoses[0].mPosition - previous.mPoses[0].mPosition) * t, hip };
            poses[1] = { input.mShapes[1], poses[0].mPosition + JPH::RVec3(hip * leg.mUpperOffset * input.mScale), calf };
            poses[2] = { input.mShapes[2], poses[1].mPosition + JPH::RVec3(calf * lower * input.mScale), ankle };
            if (sample == 0 || sample == continuity.mIntervals)
            {
                const auto& endpoint = sample == 0 ? previous : candidate;
                for (std::size_t i = 0; i < 3; ++i)
                    if (JPH::Vec3(poses[i].mPosition - endpoint.mPoses[i].mPosition).Length() > 1e-4f
                        || !(poses[i].mRotation.IsClose(endpoint.mPoses[i].mRotation, 1e-8f)
                            || poses[i].mRotation.IsClose(-endpoint.mPoses[i].mRotation, 1e-8f))) return false;
            }
            if (input.mMinimumWorldZ)
                for (const auto& pose : poses)
                {
                    const auto bounds = pose.mShape->GetWorldSpaceBounds(JPH::Mat44::sRotationTranslation(
                        pose.mRotation, pose.mRotation * pose.mShape->GetCenterOfMass()), JPH::Vec3::sReplicate(1.0f));
                    if (pose.mPosition.GetZ() + bounds.mMin.GetZ() < *input.mMinimumWorldZ) return false;
                }
            for (std::size_t i = 0; i < support.size(); ++i)
                interpolatedSupport[i] = { support[i].mShape, continuity.mPreviousSupport[i].mPosition
                    + (support[i].mPosition - continuity.mPreviousSupport[i].mPosition) * t,
                    continuity.mPreviousSupport[i].mRotation.SLERP(support[i].mRotation, t).Normalized() };
            const auto result = measureLimbClearance(poses, interpolatedSupport, margin);
            if (!result || !result->mClear) return false;
        }
        return true;
    }

    namespace
    {
        float quatAngle(JPH::QuatArg a, JPH::QuatArg b)
        {
            return 2.0f * std::acos(std::clamp(std::abs(a.Normalized().Dot(b.Normalized())), 0.0f, 1.0f));
        }

        float floorMargin(const LimbPoseSearchInput& input, std::span<const LimbShapePose> poses)
        {
            if (!input.mMinimumWorldZ) return std::numeric_limits<float>::quiet_NaN();
            float margin = std::numeric_limits<float>::infinity();
            for (const auto& pose : poses)
            {
                const auto bounds = pose.mShape->GetWorldSpaceBounds(JPH::Mat44::sRotationTranslation(
                    pose.mRotation, pose.mRotation * pose.mShape->GetCenterOfMass()), JPH::Vec3::sReplicate(1.0f));
                margin = std::min(margin,
                    static_cast<float>(pose.mPosition.GetZ() + bounds.mMin.GetZ() - *input.mMinimumWorldZ));
            }
            return margin;
        }
    }

    LimbPoseDiagnosis diagnoseLimbPose(const LimbPoseSearchInput& input,
        std::span<const LimbShapePose> support, float margin)
    {
        // Mirrors the exact-requested candidate evaluation in findLimbPoseImpl;
        // reports each subcondition independently instead of early-rejecting.
        LimbPoseDiagnosis result;
        if (support.empty() || !std::isfinite(margin) || margin < 0)
            return result;
        const auto& leg = input.mLeg;
        result.mValidInput = true;
        const JPH::Vec3 lower = leg.mLowerRotation.Conjugated() * leg.mLowerOffset;
        const auto requestedCalf = (input.mHipRotation * JPH::Quat::sRotation(leg.mHingeAxis,
            input.mKneeAngleFromReference) * leg.mLowerRotation).Normalized();
        const auto requestedKnee = input.mHipPosition + JPH::RVec3(input.mHipRotation * leg.mUpperOffset * input.mScale);
        const auto requestedFoot = requestedKnee + JPH::RVec3(requestedCalf * lower * input.mScale);
        const auto hip = clampSwingTwistJointOrientation(input.mHipLimits, input.mParentRotation, input.mHipRotation);
        if (hip)
            result.mHipClampDelta = quatAngle(*hip, input.mHipRotation);
        result.mHipClamped = !hip || result.mHipClampDelta > 1e-10f;
        const float knee = std::clamp(input.mKneeAngleFromReference, leg.mMinimumAngle, leg.mMaximumAngle);
        result.mKneeClampDelta = std::abs(knee - input.mKneeAngleFromReference);
        result.mKneeClamped = result.mKneeClampDelta > 0;
        const auto effectiveHip = hip.value_or(input.mHipRotation);
        const auto calf = (effectiveHip * JPH::Quat::sRotation(leg.mHingeAxis, knee) * leg.mLowerRotation).Normalized();
        const auto ankle = clampSwingTwistJointOrientation(input.mAnkleLimits, calf, input.mFootRotation);
        if (ankle)
            result.mAnkleClampDelta = quatAngle(*ankle, input.mFootRotation);
        result.mAnkleClamped = !ankle || result.mAnkleClampDelta > 1e-10f;
        std::array<LimbShapePose, 3> poses;
        poses[0] = { input.mShapes[0], input.mHipPosition, effectiveHip };
        poses[1] = { input.mShapes[1], input.mHipPosition + JPH::RVec3(effectiveHip * leg.mUpperOffset * input.mScale), calf };
        poses[2] = { input.mShapes[2], poses[1].mPosition + JPH::RVec3(calf * lower * input.mScale),
            ankle.value_or(input.mFootRotation) };
        result.mExactFootDisplacement = JPH::Vec3(poses[2].mPosition - requestedFoot).Length();
        result.mFloorMargin = floorMargin(input, poses);
        result.mFloorClear = !input.mMinimumWorldZ || result.mFloorMargin >= 0;
        if (const auto clearance = measureLimbClearance(poses, support, margin))
        {
            result.mSupportClear = clearance->mClear;
            result.mSupportGap = clearance->mMinimumReportedGap.value_or(
                std::numeric_limits<float>::quiet_NaN());
        }
        else
            result.mValidInput = false;
        return result;
    }

    LimbTransitionDiagnosis diagnoseLimbTransition(const LimbPoseSearchInput& input,
        const ClearLimbPose& candidate, std::span<const LimbShapePose> support, float margin,
        const LimbPoseContinuity& continuity)
    {
        // Mirrors isLimbTransitionClear; reports the first failing check instead
        // of a bare false. Bound failures leave mFirstFailingSample at -1.
        LimbTransitionDiagnosis result;
        const auto& previous = continuity.mPrevious;
        const auto& leg = input.mLeg;
        if (support.empty() || support.size() != continuity.mPreviousSupport.size()
            || continuity.mIntervals < 2 || continuity.mIntervals > 64)
            return result;
        result.mFootTravel = JPH::Vec3(candidate.mPoses[2].mPosition - previous.mPoses[2].mPosition).Length();
        result.mKneeChange = std::abs(candidate.mKneeAngleFromReference - previous.mKneeAngleFromReference);
        result.mHipChange = quatAngle(continuity.mPreviousParentRotation.Conjugated() * previous.mPoses[0].mRotation,
            input.mParentRotation.Conjugated() * candidate.mPoses[0].mRotation);
        result.mAnkleChange = quatAngle(previous.mPoses[1].mRotation.Conjugated() * previous.mPoses[2].mRotation,
            candidate.mPoses[1].mRotation.Conjugated() * candidate.mPoses[2].mRotation);
        if (result.mFootTravel > continuity.mMaximumFootTravel)
        {
            result.mFailure = LimbTransitionFailure::FootTravel;
            return result;
        }
        if (result.mKneeChange > continuity.mMaximumJointRotation)
        {
            result.mFailure = LimbTransitionFailure::KneeChange;
            return result;
        }
        if (result.mHipChange > continuity.mMaximumJointRotation)
        {
            result.mFailure = LimbTransitionFailure::HipChange;
            return result;
        }
        if (result.mAnkleChange > continuity.mMaximumJointRotation)
        {
            result.mFailure = LimbTransitionFailure::AnkleChange;
            return result;
        }
        const auto unchanged = [](const std::optional<JPH::Quat>& clamped, JPH::QuatArg rotation) {
            return clamped && (clamped->IsClose(rotation, 1e-10f) || clamped->IsClose(-rotation, 1e-10f));
        };
        const auto lower = leg.mLowerRotation.Conjugated() * leg.mLowerOffset;
        std::vector<LimbShapePose> interpolatedSupport(support.size());
        for (unsigned sample = 0; sample <= continuity.mIntervals; ++sample)
        {
            const float t = static_cast<float>(sample) / continuity.mIntervals;
            const auto parent = continuity.mPreviousParentRotation.SLERP(input.mParentRotation, t).Normalized();
            const auto hip = previous.mPoses[0].mRotation.SLERP(candidate.mPoses[0].mRotation, t).Normalized();
            const float knee = std::lerp(previous.mKneeAngleFromReference, candidate.mKneeAngleFromReference, t);
            result.mFirstFailingSample = static_cast<int>(sample);
            if (knee < leg.mMinimumAngle || knee > leg.mMaximumAngle)
            {
                result.mFailure = LimbTransitionFailure::SampleKneeRange;
                return result;
            }
            if (!unchanged(clampSwingTwistJointOrientation(input.mHipLimits, parent, hip), hip))
            {
                result.mFailure = LimbTransitionFailure::SampleHipClamp;
                return result;
            }
            const auto calf = (hip * JPH::Quat::sRotation(leg.mHingeAxis, knee) * leg.mLowerRotation).Normalized();
            const auto ankleFrom = (previous.mPoses[1].mRotation.Conjugated()
                * previous.mPoses[2].mRotation).Normalized();
            const auto ankleTo = (candidate.mPoses[1].mRotation.Conjugated()
                * candidate.mPoses[2].mRotation).Normalized();
            const auto ankle = (calf * ankleFrom.SLERP(ankleTo, t)).Normalized();
            if (!unchanged(clampSwingTwistJointOrientation(input.mAnkleLimits, calf, ankle), ankle))
            {
                result.mFailure = LimbTransitionFailure::SampleAnkleClamp;
                return result;
            }
            std::array<LimbShapePose, 3> poses;
            poses[0] = { input.mShapes[0], previous.mPoses[0].mPosition
                + (candidate.mPoses[0].mPosition - previous.mPoses[0].mPosition) * t, hip };
            poses[1] = { input.mShapes[1], poses[0].mPosition + JPH::RVec3(hip * leg.mUpperOffset * input.mScale), calf };
            poses[2] = { input.mShapes[2], poses[1].mPosition + JPH::RVec3(calf * lower * input.mScale), ankle };
            if (sample == 0 || sample == continuity.mIntervals)
            {
                const auto& endpoint = sample == 0 ? previous : candidate;
                for (std::size_t i = 0; i < 3; ++i)
                    if (JPH::Vec3(poses[i].mPosition - endpoint.mPoses[i].mPosition).Length() > 1e-4f
                        || !(poses[i].mRotation.IsClose(endpoint.mPoses[i].mRotation, 1e-8f)
                            || poses[i].mRotation.IsClose(-endpoint.mPoses[i].mRotation, 1e-8f)))
                {
                    result.mFailure = LimbTransitionFailure::SampleEndpoint;
                    return result;
                }
            }
            result.mSampleFloorMargin = floorMargin(input, poses);
            if (input.mMinimumWorldZ && result.mSampleFloorMargin < 0)
            {
                result.mFailure = LimbTransitionFailure::SampleFloor;
                return result;
            }
            for (std::size_t i = 0; i < support.size(); ++i)
                interpolatedSupport[i] = { support[i].mShape, continuity.mPreviousSupport[i].mPosition
                    + (support[i].mPosition - continuity.mPreviousSupport[i].mPosition) * t,
                    continuity.mPreviousSupport[i].mRotation.SLERP(support[i].mRotation, t).Normalized() };
            const auto clearance = measureLimbClearance(poses, interpolatedSupport, margin);
            if (!clearance)
            {
                result.mFailure = LimbTransitionFailure::InvalidInput;
                return result;
            }
            result.mSampleSupportGap = clearance->mMinimumReportedGap.value_or(
                std::numeric_limits<float>::quiet_NaN());
            if (!clearance->mClear)
            {
                result.mFailure = LimbTransitionFailure::SampleClearance;
                return result;
            }
        }
        result.mFailure = LimbTransitionFailure::None;
        result.mFirstFailingSample = -1;
        return result;
    }

}
