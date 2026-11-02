#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_SUPPORTGRIP_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_SUPPORTGRIP_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
// clang-format on

#include <memory>
#include <optional>
#include <cstdint>

namespace JPH
{
    class PhysicsSystem;
}

namespace RagdollCore
{
    // WPN-05 support-grip constraints use physical anchors, not
    // animation targets. Coordinates are local to each body's center of mass
    // in scaled physics units; callers convert model/bone origins explicitly.
    struct SupportGripRequest
    {
        JPH::BodyID mFirstBody;
        JPH::BodyID mSecondBody;
        JPH::Vec3 mFirstLocalAnchor = JPH::Vec3::sZero();
        JPH::Vec3 mSecondLocalAnchor = JPH::Vec3::sZero();
        // Zero keeps the world default. Native weapon integration supplies the
        // same per-assembly budgets used by the primary grip joints.
        std::uint32_t mVelocitySolverSteps = 0;
        std::uint32_t mPositionSolverSteps = 0;
        std::uint32_t mConstraintPriority = 0;
        // GRIP-02 (10-09): an optional rotational hold beside the point. A
        // second constraint (translation free) drives the first body toward
        // second * mHoldRelation (rotations of the bodies themselves) with
        // spring motors bounded by mHoldTorque. Without it the two hands of a
        // two-handed weapon left the weapon free to roll on the line between
        // them and the support hand free to turn off the handle.
        std::optional<JPH::Quat> mHoldRelation;
        float mHoldFrequency = 10.0f;
        float mHoldDamping = 1.0f;
        float mHoldTorque = 0.0f;
    };

    struct SupportGripAcquisitionLimits
    {
        float mMaximumAnchorSeparation;
        float mMaximumRelativeAnchorSpeed;
    };

    enum class SupportGripAcquireResult
    {
        Acquired,
        AlreadyAttached,
        InvalidRequest,
        InvalidBodies,
        TooFar,
        TooFast,
    };

    struct SupportGripObservation
    {
        JPH::BodyID mFirstBody;
        JPH::BodyID mSecondBody;
        JPH::RVec3 mFirstWorldAnchor = JPH::RVec3::sZero();
        JPH::RVec3 mSecondWorldAnchor = JPH::RVec3::sZero();
        float mAnchorSeparation = 0;
        float mRelativeAnchorSpeed = 0;
        bool mConstraintActive = false;
        // Jolt's last accumulated velocity-solver lambda, in world axes.
        // First body receives its negative. Sleeping constraints report zero
        // rather than a stale previous lambda. Read after a completed Update.
        JPH::Vec3 mImpulseOnSecond = JPH::Vec3::sZero();
        // Anchor positions are sampled now. Angular impulse accounting must
        // also capture pre-step moment arms; do not pair post-step positions
        // with this lambda and assume they were its solver application points.
    };

    struct SupportGripBreakResult
    {
        SupportGripObservation mObservation;
        bool mReleased = false;
    };

    // Owns one passive point constraint between two dynamic rigid bodies. It
    // adds no motors, collision filtering, or pose edits. A caller may copy
    // existing assembly-local solver overrides into the request.
    // All operations (including destruction) must be serialized with Jolt
    // Update and body removal, outside contact callbacks/body locks. The
    // PhysicsSystem must outlive this object; clear before either body is
    // removed/destroyed. Native integration owns eligibility and break latches.
    class SupportGrip
    {
    public:
        explicit SupportGrip(JPH::PhysicsSystem& physics);
        ~SupportGrip();

        SupportGrip(const SupportGrip&) = delete;
        SupportGrip& operator=(const SupportGrip&) = delete;
        SupportGrip(SupportGrip&&) = delete;
        SupportGrip& operator=(SupportGrip&&) = delete;

        SupportGripAcquireResult acquire(
            const SupportGripRequest& request, const SupportGripAcquisitionLimits& limits);
        bool attached() const;
        // Moves the weapon-side point of an existing grip in body-COM local
        // coordinates. The caller bounds each step and owns overload release.
        bool setSecondLocalAnchor(JPH::Vec3Arg anchor);
        // Returns true only when an existing constraint was removed.
        bool clear();
        std::optional<SupportGripObservation> observe() const;
        // Rejects non-finite/negative limits without mutation. A successful
        // overload report retains the final observation after clearing the
        // constraint; caller decides when acquisition is eligible again.
        std::optional<SupportGripBreakResult> releaseIfOverloaded(float maximumLinearImpulse);

    private:
        struct Attachment;
        JPH::PhysicsSystem& mPhysics;
        std::unique_ptr<Attachment> mAttachment;
    };
}

#endif
