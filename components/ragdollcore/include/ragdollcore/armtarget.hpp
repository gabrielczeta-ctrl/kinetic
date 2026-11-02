#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_ARMTARGET_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_ARMTARGET_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <ragdollcore/gaitcontroller.hpp>
#include <ragdollcore/combatstate.hpp>
#include <ragdollcore/passiveragdoll.hpp>

#include <cstddef>
#include <array>
#include <optional>
#include <utility>
#include <span>

namespace RagdollCore
{
    bool protectiveArmDriveEnabled(ActiveControllerMode mode, bool leftBrace, bool rightBrace);

    struct BraceHandTarget
    {
        JPH::RVec3 mWorldTarget;
        JPH::Vec3 mWorldBend;
    };

    // Wall braces require a measured surface whose normal is mostly
    // horizontal. Floors and ceilings are handled by the ground brace path.
    bool isVerticalBraceSurface(JPH::Vec3Arg surfaceNormal);

    // Shared native/replay brace geometry. Null means release the brace:
    // the floor cannot be reached within the elbow's admissible reach band.
    std::optional<BraceHandTarget> calculateBraceHandTarget(JPH::RVec3Arg pelvis,
        JPH::RVec3Arg shoulder, JPH::RVec3Arg elbow, JPH::RVec3Arg hand,
        double groundHeight, const std::optional<HingeLegGeometry>& geometry,
        const ActiveControllerProfile& profile);

    // Projects a protective hand brace onto any measured surface. The point
    // is the contact point on the surface and the normal points away from it;
    // the returned target is offset by the configured hand clearance. This is
    // deliberately geometry-only: callers must provide a real solver/query
    // contact, so a wall brace cannot invent support from a speculative ray.
    std::optional<BraceHandTarget> calculateSurfaceBraceHandTarget(JPH::RVec3Arg pelvis,
        JPH::RVec3Arg shoulder, JPH::RVec3Arg elbow, JPH::RVec3Arg hand,
        JPH::RVec3Arg surfacePoint, JPH::Vec3Arg surfaceNormal,
        const std::optional<HingeLegGeometry>& geometry,
        const ActiveControllerProfile& profile);

    // Places one hand at a model-space point by editing the shoulder, elbow
    // and wrist motor targets of an otherwise unmodified pose. Everything the
    // arms need above this - a brace while rising, a flinch, a weapon swing -
    // differs only in where the point is and how strongly it is asserted, so
    // the geometry lives here once and the intent lives in the callers.
    //
    // The chain is solved by the same two-bone routine the legs use. That is
    // not an analogy: the real rig gives the elbow a Hinge exactly as it gives
    // the knee one, with the same body ordering, so PassiveRagdoll's hinge
    // geometry query and applyTwoBoneLegTarget both apply unchanged. What does
    // NOT carry over is calculateLevelFootTarget, which exists to keep a sole
    // flat against the ground; levelling a wrist that way would rotate the
    // hand into a pose nothing asked for. The hand is oriented explicitly or
    // left to follow the forearm.
    struct ArmTargetInput
    {
        std::size_t mShoulderBodyIndex = 0;
        std::size_t mElbowBodyIndex = 0;
        std::size_t mHandBodyIndex = 0;
        // Model-space point the hand origin should reach. Targets beyond the
        // arm's reach are clamped by the solver rather than rejected, so a
        // caller may aim past a surface and get a fully extended arm.
        JPH::Vec3 mTargetHandPosition = JPH::Vec3::sZero();
        // Which way the elbow should point away from the shoulder-to-hand
        // line. For a brace or a guard this is normally down and outward.
        JPH::Vec3 mPreferredBendDirection = JPH::Vec3::sAxisZ();
        // Optional model-space undriven axis (e.g. mouse gesture depth).
        // Constrained reach prioritizes preserving this coordinate when the
        // requested point is outside the shoulder/elbow workspace.
        std::optional<JPH::Vec3> mPreservedPositionAxis;
        // Model-space hand orientation. Empty keeps the hand's base rotation
        // relative to its forearm, so the wrist neither leads nor lags the
        // arm it is attached to.
        std::optional<JPH::Quat> mHandRotation;
        // How much of the solved arm to assert over the incoming pose, in
        // [0, 1]. Zero leaves the pose untouched, which is what lets a brace
        // fade in and out instead of snapping on.
        float mWeight = 1.0f;
        // Relative rotation paths from the previous solve. Retaining their
        // quaternion hemisphere prevents a partial blend flipping its arc
        // when the incoming and solved poses cross a half turn.
        std::optional<std::array<JPH::Quat, 3>> mRotationBlendHistory;
        std::optional<HingeLegGeometry> mHingeGeometry;
        // Native shoulder cone/twist limits participate in the hand solve.
        std::optional<SwingTwistJointGeometry> mShoulderGeometry;
        // For an oriented tool, solve the arm with the real wrist limits too.
        // The local aim offset prices orientation error as tool-tip distance,
        // in the same model units as the hand's positional error.
        std::optional<SwingTwistJointGeometry> mHandGeometry;
        JPH::Vec3 mHandAimOffset = JPH::Vec3::sZero();
        // Previous solve's elbow (ArmTargetOutput::mElbowPosition). A hand
        // point leaves the arm's roll about the shoulder-to-hand ray free;
        // without an orientation goal to fix it, near-equal rolls alternate
        // between steps and spin the whole arm. The weight prices squared
        // elbow displacement against squared hand error (both model units).
        // A nearly straight arm keeps its elbow on that ray, so the forearm's
        // previous rotation is priced as well: (1 - |dot|) times
        // mTwistContinuityWeight.
        std::optional<JPH::Vec3> mPreviousElbowPosition;
        std::optional<JPH::Quat> mPreviousElbowRotation;
        float mRollContinuityWeight = 0.0f;
        float mTwistContinuityWeight = 0.0f;
        // Price forearm twist against the measured forearm instead of the
        // previous command. The roll is free for the hand point, so letting
        // the physical arm choose it keeps the command on the arm's real
        // branch; a command that keeps its own roll can stay half a turn
        // from a body that was knocked into the other one.
        bool mAnchorTwistToMeasured = false;
        // Price of an elbow away from mPreferredBendDirection, as
        // (1 - cos) of the roll about the shoulder-hand ray. The default is a
        // tie-break only. Continuity alone keeps a wound elbow wherever a
        // gesture left it; a released arm raises this so the pole pulls the
        // elbow back at a rate bounded by the continuity price.
        float mBendPreferenceWeight = 1e-4f;
        // When > 0, the wrist/tip error is priced on a lever of this length
        // instead of the real aim offset (long weapons), so the elbow's
        // continuity terms are not dwarfed by the weapon's length.
        float mTipReferenceLever = 0.0f;
        // When > 0 and mPreviousElbowPosition is set: the solved elbow may
        // move at most this far from it (units); farther candidates pay a
        // steep price, so the elbow turns over rather than jumping sides.
        float mMaximumElbowStep = 0.0f;
        // When > 0: hand reach error up to this many units is priced as usual
        // (squared) and beyond it steeply, so a strong bend preference may
        // carry the elbow round to its side at the cost of a bounded hand
        // drift, never an unbounded one.
        float mReachTolerance = 0.0f;
        // Extra price of the elbow's roll beyond mBendCrossingKnee (1 - cos;
        // 0.3 is about 45 deg) from the preferred pole: strong enough to carry
        // an elbow from the wrong side, without the whole preference outbidding
        // the hand's reach once the elbow is on its side.
        float mBendCrossingWeight = 0.0f;
        float mBendCrossingKnee = 0.3f;
        // When > 0: a previous elbow closer than this (units) to the
        // shoulder-hand line waives mMaximumElbowStep, so a straightened arm
        // can bend again.
        float mStraightElbowRelease = 0.0f;
        // The elbow is the request's alone: the solved arm depends only on
        // the target, the pole and the limits, never on the previous solve or
        // the measured arm. No continuity, no step limit and no refine from
        // the measured arm, so a body knocked or lagging into the other elbow
        // branch is pulled back to the requested one instead of the command
        // adopting it (fast rising cuts, 2026-10-03). The pole is priced by
        // mBendPreferenceWeight as usual.
        bool mExactPole = false;
    };

    struct ArmTargetOutput
    {
        // False when the frame or the chain could not be solved. The target
        // pose is never partially edited: either the complete arm is written
        // or nothing is.
        bool mApplied = false;
        // Where the hand actually ended up, before the weight blend. Compare
        // against the request to see whether the target was out of reach.
        JPH::Vec3 mHandPosition = JPH::Vec3::sZero();
        // Distance between the request and what the arm could reach. Callers
        // that aim at a surface use this to tell a brace that found the floor
        // from one that is still waving at it.
        float mReachError = 0.0f;
        // The weight actually applied, after clamping.
        float mWeight = 0.0f;
        std::optional<std::array<JPH::Quat, 3>> mRotationBlendHistory;
        // Solved elbow in the solver's frame, for the next step's
        // mPreviousElbowPosition.
        std::optional<JPH::Vec3> mElbowPosition;
        std::optional<JPH::Quat> mElbowRotation;
    };

    // The distances from the shoulder that this chain can actually hold,
    // in the same model units as the geometry. A hinge has a minimum as well
    // as a maximum: an elbow cannot fold past its limit, so a hand aimed
    // closer than the minimum is as unreachable as one aimed past the span,
    // and a caller that only checks the far end will place targets the arm
    // silently fails to meet. Derived from the same closure equation the
    // solver uses, so the two cannot disagree.
    std::optional<std::pair<float, float>> hingeChainReachRange(const HingeLegGeometry& geometry);

    // Re-expresses the model-space hand target at the measured frame and
    // shoulder, solves the arm, then blends the result into target by weight.
    // The frame body is the shoulder's parent - the clavicle on the humanoid
    // profile - and plays the part the pelvis plays for a leg: it supplies the
    // rotation that maps the measured arm back onto the base pose the solver
    // works in. Failure never edits target.
    std::optional<ArmTargetOutput> applyArmTargetPose(std::span<Transform> target,
        std::span<const Transform> base, std::span<const Transform> measured, std::size_t frameBody,
        ArmTargetInput input);

    // For a tool with equivalent opposing edges/faces, compare the two rolls
    // about its aim axis using the actual arm limits. True only when a half
    // turn improves both hand reach and tip accuracy. Call at gesture start
    // and retain that branch through reversals; this does not edit any pose.
    bool preferOppositeArmToolRoll(std::span<const Transform> target, std::span<const Transform> base,
        std::span<const Transform> measured, std::size_t frameBody, ArmTargetInput input);
}

#endif
