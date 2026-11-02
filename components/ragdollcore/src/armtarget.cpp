#include <ragdollcore/armtarget.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <limits>
#include <utility>
#include <vector>

namespace RagdollCore
{
    bool isVerticalBraceSurface(JPH::Vec3Arg surfaceNormal)
    {
        const float length = surfaceNormal.Length();
        return std::isfinite(length) && length > 1e-5f
            && std::abs(surfaceNormal.GetZ() / length) < 0.35f;
    }

    bool protectiveArmDriveEnabled(ActiveControllerMode mode, bool leftBrace, bool rightBrace)
    {
        return (mode == ActiveControllerMode::Falling || mode == ActiveControllerMode::Fallen)
            && (leftBrace || rightBrace);
    }

    std::optional<BraceHandTarget> calculateBraceHandTarget(JPH::RVec3Arg pelvis,
        JPH::RVec3Arg shoulder, JPH::RVec3Arg elbow, JPH::RVec3Arg hand,
        double groundHeight, const std::optional<HingeLegGeometry>& geometry,
        const ActiveControllerProfile& profile)
    {
        JPH::Vec3 outward(static_cast<float>(shoulder.GetX() - pelvis.GetX()),
            static_cast<float>(shoulder.GetY() - pelvis.GetY()), 0.0f);
        outward = outward.IsNearZero() ? JPH::Vec3::sZero() : outward.Normalized();
        const JPH::RVec3 ground(shoulder.GetX() + outward.GetX() * profile.mBraceHandOffset,
            shoulder.GetY() + outward.GetY() * profile.mBraceHandOffset,
            groundHeight);
        const auto result = calculateSurfaceBraceHandTarget(pelvis, shoulder, elbow, hand,
            ground, JPH::Vec3::sAxisZ(), geometry, profile);
        if (!result)
            return std::nullopt;
        return result;
    }

    std::optional<BraceHandTarget> calculateSurfaceBraceHandTarget(JPH::RVec3Arg pelvis,
        JPH::RVec3Arg shoulder, JPH::RVec3Arg elbow, JPH::RVec3Arg hand,
        JPH::RVec3Arg surfacePoint, JPH::Vec3Arg surfaceNormal,
        const std::optional<HingeLegGeometry>& geometry,
        const ActiveControllerProfile& profile)
    {
        const float normalLength = surfaceNormal.Length();
        if (!(normalLength > 1e-5f) || !std::isfinite(normalLength))
            return std::nullopt;
        const JPH::Vec3 normal = surfaceNormal / normalLength;
        const JPH::Vec3 shoulderOut(static_cast<float>(shoulder.GetX() - pelvis.GetX()),
            static_cast<float>(shoulder.GetY() - pelvis.GetY()), 0.0f);
        const JPH::Vec3 outward = shoulderOut.IsNearZero() ? -normal : shoulderOut.Normalized();
        const JPH::RVec3 target = surfacePoint + normal * profile.mBraceHandClearance;
        const auto range = geometry ? hingeChainReachRange(*geometry) : std::nullopt;
        const float modelSpan = geometry ? geometry->mUpperOffset.Length() + geometry->mLowerOffset.Length() : 0.0f;
        const float span = static_cast<float>((elbow - shoulder).Length()) + static_cast<float>((hand - elbow).Length());
        if (!range || !(modelSpan > 0.0f) || !(span > 0.0f))
            return std::nullopt;
        const float margin = std::clamp(profile.mBraceReachMargin, 0.0f, 0.49f);
        const float width = range->second - range->first;
        const float nearest = (range->first + width * margin) / modelSpan * span;
        const float furthest = (range->second - width * margin) / modelSpan * span;
        JPH::Vec3 offset(static_cast<float>(target.GetX() - shoulder.GetX()),
            static_cast<float>(target.GetY() - shoulder.GetY()), static_cast<float>(target.GetZ() - shoulder.GetZ()));
        const float distance = offset.Length();
        if (distance > furthest)
            return std::nullopt;
        if (distance < nearest)
        {
            const float height = static_cast<float>(shoulder.GetZ() - target.GetZ());
            const float radius = std::sqrt(std::max(nearest * nearest - height * height, 0.0f));
            JPH::Vec3 flat(offset.GetX(), offset.GetY(), 0.0f);
            if (flat.IsNearZero()) flat = outward.IsNearZero() ? JPH::Vec3::sAxisY() : outward;
            if (flat.IsNearZero() || !(radius > 0.0f))
                return std::nullopt;
            flat = flat.Normalized() * radius;
            return BraceHandTarget{JPH::RVec3(shoulder.GetX() + flat.GetX(), shoulder.GetY() + flat.GetY(),
                target.GetZ()), outward};
        }
        return BraceHandTarget{ target, outward };
    }

    namespace
    {
        bool isFinite(float value)
        {
            return std::isfinite(value);
        }

        bool isFinite(JPH::Vec3Arg value)
        {
            return isFinite(value.GetX()) && isFinite(value.GetY()) && isFinite(value.GetZ());
        }

        bool isFinite(JPH::QuatArg value)
        {
            return isFinite(value.GetX()) && isFinite(value.GetY()) && isFinite(value.GetZ()) && isFinite(value.GetW());
        }

        bool validTransform(const Transform& transform)
        {
            return isFinite(transform.mTranslation) && isFinite(transform.mRotation)
                && transform.mRotation.IsNormalized();
        }

        // Start on the shortest arc, then retain a supplied path across steps.
        // Re-normalize every command before it reaches a motor.
        JPH::Quat blendRotation(JPH::QuatArg from, JPH::QuatArg to, float weight,
            const std::optional<JPH::Quat>& previous, JPH::Quat& history)
        {
            auto delta = (from.Conjugated() * to).Normalized();
            if (previous ? delta.Dot(*previous) < 0.0f : delta.GetW() < 0.0f)
                delta = -delta;
            history = delta;
            // Jolt SLERP always reselects the shortest arc. Follow the stored
            // relative path instead, including smoothly through 180 degrees.
            const float sine = delta.GetXYZ().Length();
            const auto fallback = previous
                ? previous->GetXYZ().NormalizedOr(JPH::Vec3::sAxisX()) : JPH::Vec3::sAxisX();
            const auto axis = delta.GetXYZ().NormalizedOr(fallback);
            const float angle = 2.0f * std::atan2(sine, delta.GetW());
            const JPH::Quat result = from * JPH::Quat::sRotation(axis, angle * weight);
            if (!isFinite(result) || result.LengthSq() <= 1e-8f)
                return from;
            return result.Normalized();
        }
    }

    std::optional<std::pair<float, float>> hingeChainReachRange(const HingeLegGeometry& geometry)
    {
        const auto& upper = geometry.mUpperOffset;
        const auto& lower = geometry.mLowerOffset;
        const auto& axis = geometry.mHingeAxis;
        if (!isFinite(upper) || !isFinite(lower) || upper.LengthSq() <= 1e-8f || lower.LengthSq() <= 1e-8f
            || !isFinite(axis) || !axis.IsNormalized() || !isFinite(geometry.mMinimumAngle)
            || !isFinite(geometry.mMaximumAngle) || geometry.mMinimumAngle > geometry.mMaximumAngle)
            return std::nullopt;

        // The same closure the solver inverts: with the joint opened by
        // `angle`, the squared shoulder-to-hand distance is
        //   base + amplitude * cos(reference + angle)
        // so the reachable distances are found by extremizing that cosine
        // over the joint's own limits rather than over a full turn.
        const auto upperPerpendicular = upper - axis * upper.Dot(axis);
        const auto lowerPerpendicular = lower - axis * lower.Dot(axis);
        const float amplitude = 2.0f * upperPerpendicular.Length() * lowerPerpendicular.Length();
        const float base = upper.LengthSq() + lower.LengthSq() + 2.0f * upper.Dot(axis) * lower.Dot(axis);
        const float reference = std::atan2(axis.Dot(upperPerpendicular.Cross(lowerPerpendicular)),
            upperPerpendicular.Dot(lowerPerpendicular));
        if (!isFinite(amplitude) || !isFinite(base) || !isFinite(reference))
            return std::nullopt;

        const float from = reference + geometry.mMinimumAngle;
        const float to = reference + geometry.mMaximumAngle;
        float lowestCosine = std::min(std::cos(from), std::cos(to));
        float highestCosine = std::max(std::cos(from), std::cos(to));
        // A turning point of the cosine inside the joint's travel beats both
        // endpoints. Straight-arm and fully folded are exactly these.
        const float turns = std::numbers::pi_v<float>;
        const float firstBelow = std::ceil(from / turns);
        for (float multiple = firstBelow; multiple * turns <= to; multiple += 1.0f)
        {
            if (static_cast<int>(std::fmod(std::abs(multiple), 2.0f)) == 0)
                highestCosine = 1.0f;
            else
                lowestCosine = -1.0f;
        }

        const float minimum = std::sqrt(std::max(base + amplitude * lowestCosine, 0.0f));
        const float maximum = std::sqrt(std::max(base + amplitude * highestCosine, 0.0f));
        if (!isFinite(minimum) || !isFinite(maximum) || maximum < minimum)
            return std::nullopt;
        return std::pair<float, float>{ minimum, maximum };
    }

    std::optional<ArmTargetOutput> applyArmTargetPose(std::span<Transform> target,
        std::span<const Transform> base, std::span<const Transform> measured, std::size_t frameBody,
        ArmTargetInput input)
    {
        const std::array<std::size_t, 4> bodies{ frameBody, input.mShoulderBodyIndex, input.mElbowBodyIndex,
            input.mHandBodyIndex };
        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            if (bodies[i] >= target.size() || bodies[i] >= base.size() || bodies[i] >= measured.size())
                return std::nullopt;
            for (std::size_t j = 0; j < i; ++j)
                if (bodies[i] == bodies[j])
                    return std::nullopt;
        }
        for (const std::size_t body : bodies)
            if (!validTransform(target[body]) || !validTransform(base[body]) || !validTransform(measured[body]))
                return std::nullopt;
        if (!isFinite(input.mTargetHandPosition) || !isFinite(input.mPreferredBendDirection)
            || !isFinite(input.mWeight) || !isFinite(input.mHandAimOffset))
            return std::nullopt;
        if (input.mHandRotation && (!isFinite(*input.mHandRotation) || !input.mHandRotation->IsNormalized()))
            return std::nullopt;
        if (input.mRotationBlendHistory)
            for (const auto& rotation : *input.mRotationBlendHistory)
                if (!isFinite(rotation) || !rotation.IsNormalized())
                    return std::nullopt;
        if (input.mPreservedPositionAxis
            && (!isFinite(*input.mPreservedPositionAxis) || input.mPreservedPositionAxis->IsNearZero()))
            return std::nullopt;

        // A leg's frame is its pelvis and a hip; an arm's is its clavicle and
        // a shoulder. The routine only uses the pair as "the body carrying the
        // chain" and "the chain's root", so it transfers unchanged.
        const auto frame = resolveLegFrameTarget({ base[frameBody], base[input.mShoulderBodyIndex],
            measured[frameBody], measured[input.mShoulderBodyIndex], input.mTargetHandPosition,
            input.mPreferredBendDirection });
        if (!frame)
            return std::nullopt;

        if (input.mExactPole)
        {
            input.mPreviousElbowPosition.reset();
            input.mPreviousElbowRotation.reset();
            input.mAnchorTwistToMeasured = false;
            input.mRollContinuityWeight = input.mTwistContinuityWeight = 0.0f;
            input.mMaximumElbowStep = 0.0f;
        }
        const auto previousElbowRotation = input.mAnchorTwistToMeasured
            ? std::optional((frame->mPelvisRotationDelta.Conjugated()
                * measured[input.mElbowBodyIndex].mRotation).Normalized())
            : input.mPreviousElbowRotation;
        const auto wantedHand = input.mHandRotation
            ? std::optional((frame->mPelvisRotationDelta.Conjugated() * *input.mHandRotation).Normalized())
            : std::nullopt;

        ArmTargetOutput result;
        result.mWeight = std::clamp(input.mWeight, 0.0f, 1.0f);

        TwoBoneLegTargetInput solve;
        solve.mHipBodyIndex = input.mShoulderBodyIndex;
        solve.mKneeBodyIndex = input.mElbowBodyIndex;
        solve.mFootBodyIndex = input.mHandBodyIndex;
        solve.mTargetFootPosition = frame->mTargetFootPosition;
        solve.mPreferredBendDirection = frame->mPreferredBendDirection;
        solve.mHingeGeometry = input.mHingeGeometry;

        std::vector<Transform> candidate(target.begin(), target.end());
        if (!applyTwoBoneLegTarget(candidate, solve))
            return result;

        if (input.mShoulderGeometry)
        {
            // Two-bone IK leaves one degree of freedom: roll around the
            // shoulder-to-hand ray. Choose that roll inside the real shoulder
            // limits instead of asking Jolt to clip an unrelated pose later.
            const auto shoulder = candidate[input.mShoulderBodyIndex];
            const auto elbow = candidate[input.mElbowBodyIndex];
            const auto hand = candidate[input.mHandBodyIndex];
            const auto direction = (hand.mTranslation - shoulder.mTranslation).NormalizedOr(JPH::Vec3::sAxisZ());
            const auto upper = shoulder.mRotation.Conjugated() * (elbow.mTranslation - shoulder.mTranslation);
            const auto lower = elbow.mRotation.Conjugated() * (hand.mTranslation - elbow.mTranslation);
            const auto relativeElbow = (shoulder.mRotation.Conjugated() * elbow.mRotation).Normalized();
            const auto measuredShoulder = (frame->mPelvisRotationDelta.Conjugated()
                * measured[input.mShoulderBodyIndex].mRotation).Normalized();
            // Reach alone cannot distinguish elbow rolls about the hand ray.
            // Preserve the requested pole when choosing between reachable
            // shoulder orientations, rather than erasing it after two-bone IK.
            const auto preferredPole = (frame->mPreferredBendDirection
                - direction * frame->mPreferredBendDirection.Dot(direction)).NormalizedOr(JPH::Vec3::sZero());
            const auto preservedAxis = input.mPreservedPositionAxis
                ? (frame->mPelvisRotationDelta.Conjugated() * *input.mPreservedPositionAxis).Normalized()
                : JPH::Vec3::sZero();
            const auto candidateScore = [&](JPH::Vec3Arg elbowPosition, JPH::Vec3Arg handPosition,
                                            JPH::QuatArg rotation, JPH::QuatArg elbowRotation) {
                const auto upperDirection = elbowPosition - shoulder.mTranslation;
                const auto pole = (upperDirection - direction * upperDirection.Dot(direction))
                                      .NormalizedOr(JPH::Vec3::sZero());
                const float bendError = 1.0f - std::clamp(pole.Dot(preferredPole), -1.0f, 1.0f);
                const float motion = input.mExactPole ? 0.0f : 1.0f - std::abs(rotation.Dot(measuredShoulder));
                // Small tie-break terms: positional reach remains primary;
                // the measured pose resolves ambiguity only after the pole.
                const auto error = handPosition - frame->mTargetFootPosition;
                const float depthError = error.Dot(preservedAxis);
                float tipError = 0.0f;
                if (wantedHand && input.mHandGeometry && !input.mHandAimOffset.IsNearZero())
                {
                    const auto legalHand = clampSwingTwistJointOrientation(
                        *input.mHandGeometry, elbowRotation, *wantedHand);
                    if (!legalHand)
                        return std::numeric_limits<float>::max();
                    tipError = (*legalHand * input.mHandAimOffset - *wantedHand * input.mHandAimOffset).LengthSq();
                    // Price the tip on a fixed lever, i.e. as an angle: on the
                    // real lever a 60-90 unit warhammer or claymore made any
                    // wrist-limited orientation cost thousands, dwarfing the
                    // elbow continuity terms, and the elbow flipped to the
                    // other side mid-swing and spun the weapon (play test
                    // 2026-09-24). Short blades are unchanged.
                    const float lever = input.mHandAimOffset.Length();
                    if (input.mTipReferenceLever > 0.0f && lever > input.mTipReferenceLever)
                        tipError *= (input.mTipReferenceLever * input.mTipReferenceLever) / (lever * lever);
                }
                float continuity = input.mPreviousElbowPosition
                    ? input.mRollContinuityWeight * (elbowPosition - *input.mPreviousElbowPosition).LengthSq()
                    : 0.0f;
                // A previous elbow on the shoulder-hand line (a straight arm)
                // has no side to flip from: bending out of it is free, or the
                // step limit holds the arm straight for good.
                bool straightRelease = false;
                if (input.mPreviousElbowPosition && input.mStraightElbowRelease > 0.0f)
                {
                    const auto previous = *input.mPreviousElbowPosition - shoulder.mTranslation;
                    straightRelease = (previous - direction * previous.Dot(direction)).Length()
                        < input.mStraightElbowRelease;
                }
                if (input.mPreviousElbowPosition && input.mMaximumElbowStep > 0.0f && !straightRelease)
                {
                    const float excess
                        = (elbowPosition - *input.mPreviousElbowPosition).Length() - input.mMaximumElbowStep;
                    if (excess > 0.0f)
                        continuity += 1000.0f * excess * excess;
                }
                if (previousElbowRotation)
                    continuity += input.mTwistContinuityWeight
                        * (1.0f - std::abs(elbowRotation.Dot(*previousElbowRotation)));
                float reach = error.LengthSq();
                if (input.mReachTolerance > 0.0f)
                {
                    const float over = error.Length() - input.mReachTolerance;
                    if (over > 0.0f)
                        reach += 1000.0f * over * over;
                }
                return reach + 15.0f * depthError * depthError
                    + tipError + continuity + input.mBendPreferenceWeight * bendError
                    + input.mBendCrossingWeight * std::max(0.0f, bendError - input.mBendCrossingKnee) + 1e-6f * motion;
            };
            float bestScore = std::numeric_limits<float>::max();
            float bestRoll = 0.0f;
            const auto consider = [&](float roll) {
                const auto raw = (JPH::Quat::sRotation(direction, roll) * shoulder.mRotation).Normalized();
                const auto rotation = clampSwingTwistJointOrientation(*input.mShoulderGeometry,
                    base[frameBody].mRotation, raw);
                if (!rotation)
                    return;
                const auto elbowRotation = (*rotation * relativeElbow).Normalized();
                const auto elbowPosition = shoulder.mTranslation + *rotation * upper;
                const auto handPosition = elbowPosition + elbowRotation * lower;
                const float score = candidateScore(elbowPosition, handPosition, *rotation, elbowRotation);
                if (score < bestScore)
                {
                    bestScore = score;
                    bestRoll = roll;
                    candidate[input.mShoulderBodyIndex].mRotation = *rotation;
                    candidate[input.mElbowBodyIndex].mRotation = elbowRotation;
                    candidate[input.mElbowBodyIndex].mTranslation = elbowPosition;
                    candidate[input.mHandBodyIndex].mTranslation = handPosition;
                }
            };
            constexpr int samples = 64;
            constexpr float turn = 2.0f * std::numbers::pi_v<float>;
            for (int i = 0; i < samples; ++i)
                consider(turn * i / samples);
            if (!std::isfinite(bestScore) || bestScore == std::numeric_limits<float>::max())
                return std::nullopt;
            float radius = turn / samples;
            for (int refinement = 0; refinement < 10; ++refinement)
            {
                const float center = bestRoll;
                consider(center - radius);
                consider(center + radius);
                radius *= 0.5f;
            }
            if (input.mHingeGeometry && bestScore > 1e-4f)
            {
                // Once a shoulder limit clips the ray solution, the elbow
                // angle chosen for its original distance is no longer optimal.
                // Refine the actual constrained chain, retaining the ray solve
                // whenever no feasible candidate improves it.
                const auto& hinge = *input.mHingeGeometry;
                const auto flexion = (candidate[input.mShoulderBodyIndex].mRotation.Conjugated()
                    * candidate[input.mElbowBodyIndex].mRotation * hinge.mLowerRotation.Conjugated()).Normalized();
                const JPH::Vec3 imaginary(flexion.GetX(), flexion.GetY(), flexion.GetZ());
                float angle = 2.0f * std::atan2(imaginary.Dot(hinge.mHingeAxis), flexion.GetW());
                if (angle > std::numbers::pi_v<float>) angle -= turn;
                if (angle < -std::numbers::pi_v<float>) angle += turn;
                angle = std::clamp(angle, hinge.mMinimumAngle, hinge.mMaximumAngle);
                const auto improve = [&](JPH::QuatArg raw, float requestedAngle) {
                    const auto rotation = clampSwingTwistJointOrientation(*input.mShoulderGeometry,
                        base[frameBody].mRotation, raw);
                    if (!rotation)
                        return;
                    const float trialAngle = std::clamp(requestedAngle, hinge.mMinimumAngle, hinge.mMaximumAngle);
                    const auto flex = JPH::Quat::sRotation(hinge.mHingeAxis, trialAngle);
                    const auto elbowPosition = shoulder.mTranslation + *rotation * hinge.mUpperOffset;
                    const auto handPosition = elbowPosition + *rotation * (flex * hinge.mLowerOffset);
                    const auto elbowRotation = (*rotation * flex * hinge.mLowerRotation).Normalized();
                    const float score = candidateScore(elbowPosition, handPosition, *rotation, elbowRotation);
                    if (score < bestScore)
                    {
                        bestScore = score;
                        angle = trialAngle;
                        candidate[input.mShoulderBodyIndex].mRotation = *rotation;
                        candidate[input.mElbowBodyIndex].mRotation = elbowRotation;
                        candidate[input.mElbowBodyIndex].mTranslation = elbowPosition;
                        candidate[input.mHandBodyIndex].mTranslation = handPosition;
                    }
                };
                const auto refine = [&] {
                    float step = 0.2f;
                    for (int iteration = 0; iteration < 48 && step >= 0.0005f; ++iteration)
                    {
                        const float previousScore = bestScore;
                        for (const auto axis : { JPH::Vec3::sAxisX(), JPH::Vec3::sAxisY(), JPH::Vec3::sAxisZ() })
                        {
                            const auto center = candidate[input.mShoulderBodyIndex].mRotation;
                            improve((JPH::Quat::sRotation(axis, step) * center).Normalized(), angle);
                            improve((JPH::Quat::sRotation(axis, -step) * center).Normalized(), angle);
                        }
                        const auto center = candidate[input.mShoulderBodyIndex].mRotation;
                        const float centerAngle = angle;
                        improve(center, centerAngle + step);
                        improve(center, centerAngle - step);
                        if (previousScore - bestScore < 1e-6f)
                            step *= 0.5f;
                    }
                };
                refine();
                // The ray seed can land in a different constrained minimum
                // between adjacent frames. Also refine from the measured arm,
                // then retain whichever legal solution reaches better.
                if (!input.mExactPole)
                {
                    const auto rayCandidate = candidate;
                    const float rayScore = bestScore;
                    const auto measuredFlexion = (measured[input.mShoulderBodyIndex].mRotation.Conjugated()
                        * measured[input.mElbowBodyIndex].mRotation * hinge.mLowerRotation.Conjugated()).Normalized();
                    float measuredAngle = 2.0f * std::atan2(measuredFlexion.GetXYZ().Dot(hinge.mHingeAxis), measuredFlexion.GetW());
                    if (measuredAngle > std::numbers::pi_v<float>) measuredAngle -= turn;
                    if (measuredAngle < -std::numbers::pi_v<float>) measuredAngle += turn;
                    bestScore = std::numeric_limits<float>::max();
                    improve(measuredShoulder, measuredAngle);
                    refine();
                    if (rayScore < bestScore)
                    {
                        candidate = rayCandidate;
                        bestScore = rayScore;
                    }
                }

            }
        }

        result.mHandPosition = candidate[input.mHandBodyIndex].mTranslation;
        result.mElbowPosition = candidate[input.mElbowBodyIndex].mTranslation;
        result.mElbowRotation = candidate[input.mElbowBodyIndex].mRotation;
        result.mReachError = (frame->mTargetFootPosition - result.mHandPosition).Length();
        if (!isFinite(result.mReachError))
            return result;

        // The solver places the wrist but never rotates it: for a leg that job
        // belongs to calculateLevelFootTarget, which flattens a sole against
        // the ground and would twist a hand into a pose nothing asked for. An
        // explicit orientation arrives in the same measured frame as the hand
        // target, so it is mapped back the same way; with none, the hand keeps
        // the angle it holds against its forearm in the base pose.
        const JPH::Quat relative
            = (base[input.mElbowBodyIndex].mRotation.Conjugated() * base[input.mHandBodyIndex].mRotation).Normalized();
        JPH::Quat hand = wantedHand
            ? *wantedHand
            : (candidate[input.mElbowBodyIndex].mRotation * relative).Normalized();
        if (input.mHandGeometry)
        {
            const auto legal = clampSwingTwistJointOrientation(
                *input.mHandGeometry, candidate[input.mElbowBodyIndex].mRotation, hand);
            if (!legal)
                return std::nullopt;
            hand = *legal;
        }
        if (!isFinite(hand) || !hand.IsNormalized())
            return result;
        candidate[input.mHandBodyIndex].mRotation = hand;

        if (result.mWeight <= 0.0f)
        {
            // Nothing is asserted, but the solve still ran and its reach is
            // still the honest answer to "could the arm get there", which is
            // what a caller ramping a brace in needs before it commits.
            result.mApplied = true;
            return result;
        }

        for (const std::size_t body : { input.mShoulderBodyIndex, input.mElbowBodyIndex, input.mHandBodyIndex })
        {
            const Transform& solved = candidate[body];
            if (!validTransform(solved))
                return result;
        }
        result.mRotationBlendHistory.emplace();
        std::size_t blendIndex = 0;
        for (const std::size_t body : { input.mShoulderBodyIndex, input.mElbowBodyIndex, input.mHandBodyIndex })
        {
            Transform& written = target[body];
            const Transform& solved = candidate[body];
            const auto previous = input.mRotationBlendHistory
                ? std::optional((*input.mRotationBlendHistory)[blendIndex]) : std::nullopt;
            written.mRotation = blendRotation(written.mRotation, solved.mRotation, result.mWeight,
                previous, (*result.mRotationBlendHistory)[blendIndex++]);
            written.mTranslation = written.mTranslation
                + (solved.mTranslation - written.mTranslation) * result.mWeight;
        }
        result.mApplied = true;
        return result;
    }

    bool preferOppositeArmToolRoll(std::span<const Transform> target, std::span<const Transform> base,
        std::span<const Transform> measured, std::size_t frameBody, ArmTargetInput input)
    {
        if (!input.mHandRotation || input.mHandAimOffset.IsNearZero())
            return false;
        input.mWeight = 1.0f;
        std::vector<Transform> first(target.begin(), target.end()), opposite = first;
        const auto initial = applyArmTargetPose(first, base, measured, frameBody, input);
        if (!initial || !initial->mApplied)
            return false;
        const auto frame = resolveLegFrameTarget({ base[frameBody], base[input.mShoulderBodyIndex],
            measured[frameBody], measured[input.mShoulderBodyIndex], input.mTargetHandPosition,
            input.mPreferredBendDirection });
        if (!frame)
            return false;
        const auto wanted = (frame->mPelvisRotationDelta.Conjugated() * *input.mHandRotation).Normalized();
        const auto desiredTip = frame->mTargetFootPosition + wanted * input.mHandAimOffset;
        const auto tipError = [&](const auto& pose, const auto& result) {
            return (result.mHandPosition + pose[input.mHandBodyIndex].mRotation * input.mHandAimOffset
                - desiredTip).LengthSq();
        };
        input.mHandRotation = (*input.mHandRotation
            * JPH::Quat::sRotation(input.mHandAimOffset.Normalized(), std::numbers::pi_v<float>)).Normalized();
        const auto alternative = applyArmTargetPose(opposite, base, measured, frameBody, input);
        // The equivalent blade plane may remove a wrist limit conflict. Never
        // exchange accurate reach for aim, or accurate aim for reach.
        return alternative && alternative->mApplied
            && alternative->mReachError < initial->mReachError
            && tipError(opposite, *alternative) < tipError(first, *initial);
    }
}
