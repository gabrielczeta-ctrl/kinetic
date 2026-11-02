#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_BALANCEMETRICS_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_BALANCEMETRICS_H

#include <ragdollcore/controller.hpp>

namespace RagdollCore
{
    // BAL-01 measurements only. These helpers neither alter a controller nor
    // invent contacts. Positions/distances use OpenMW world units, velocities
    // units/second, mass the existing physical mass, and time seconds.
    struct BalanceMassMeasurement
    {
        float mTotalMass = 0.0f;
        JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
        JPH::Vec3 mCenterOfMassVelocity = JPH::Vec3::sZero();
        std::optional<std::size_t> mKinematicRootBody;
    };

    // Use a complete, single-physics-step body snapshot. BodyTelemetry::mMass
    // is the declared body mass retained by captureTelemetry even when the
    // root is kinematic. Include all bodies, including sleeping ones; mActive
    // is deliberately irrelevant. Missing/nonpositive mass invalidates the
    // measurement instead of silently changing the represented body assembly.
    std::optional<BalanceMassMeasurement> measureBalanceMass(std::span<const BodyTelemetry> bodies,
        std::optional<std::size_t> kinematicRootBody = std::nullopt);

    struct BalanceContactPoint
    {
        GroundContact mContact;
        // Post-step velocity of the opposing surface's material point, at the
        // same contact as mContact.mPointVelocity. GroundContact currently
        // exposes only the ragdoll side. Unknown must remain unknown; callers
        // may supply zero only for an observed stationary surface.
        std::optional<JPH::Vec3> mSurfacePointVelocity;
    };

    enum class BalanceSupportAvailability
    {
        Unknown,
        Unsupported,
        Supported,
    };

    struct BalanceSupportMeasurement
    {
        BalanceSupportAvailability mAvailability = BalanceSupportAvailability::Unknown;
        bool mLeftFootSupported = false;
        bool mRightFootSupported = false;
        // Unsigned distance outside the measured horizontal convex hull;
        // exactly zero inside/on it. This is not a signed stability margin.
        std::optional<float> mHorizontalDistance;
        // Nearest hull point in XY, retaining the query's Z. No inferred ground
        // plane or arithmetic-mean contact height is encoded in this point.
        std::optional<JPH::RVec3> mClosestPoint;
        std::optional<double> mMinimumContactHeight;
        std::optional<double> mMaximumContactHeight;
    };

    // Reuses estimateFootSupport's convex-hull projection with only observed
    // upward foot contacts that delivered positive normal impulse this step.
    // Geometry alone, foot origins, walls and zero-load touching do not qualify.
    // contactsComplete must describe the whole foot-contact snapshot, not just
    // whether a callback happened. Empty complete input is Unsupported; absent
    // relevant impulse data or an incomplete snapshot is Unknown. Neither has
    // a numeric distance. Duplicate/interior points do not expand the hull.
    BalanceSupportMeasurement measureBalanceSupport(JPH::RVec3Arg query,
        std::span<const BalanceContactPoint> contacts, FootBodyIndices feet,
        const ActiveControllerProfile& profile, bool contactsComplete);

    // Horizontal linear-inverted-pendulum diagnostic for a declared stationary
    // horizontal support plane. The height must come from measured support,
    // not a foot-origin fallback. Sloped/moving support needs an explicit frame
    // interpretation at the caller; this helper does not fabricate one. The
    // result lies at supportPlaneHeight. Gravity is positive world units/s^2.
    // A missing/invalid mass snapshot or nonpositive COM height yields no value.
    std::optional<JPH::RVec3> calculateBalanceCapturePoint(const BalanceMassMeasurement& mass,
        double supportPlaneHeight, float gravity);

    struct BalanceFootSlipMeasurement
    {
        BalanceSupportAvailability mAvailability = BalanceSupportAvailability::Unknown;
        // Maximum across qualified points on this foot of the contact-normal
        // tangent component of (foot point velocity - surface point velocity).
        // A rotating foot and a moving/rotating support are therefore measured
        // at their real contact points, not by either body's COM velocity.
        std::optional<float> mMaximumTangentSpeed;
        // Maximum speed * duration, once per foot per physics step. This is a
        // conservative per-step slip estimate, not a summed manifold distance
        // or persistent episode accumulator. Sample once per fixed step.
        std::optional<float> mMaximumTangentDistanceThisStep;
    };

    // Support availability and velocity availability are distinct: Supported
    // with absent numbers means at least one qualified point velocity is
    // unknown/invalid. Unsupported also has no slip number, never a false zero.
    BalanceFootSlipMeasurement measureBalanceFootSlip(std::span<const BalanceContactPoint> contacts,
        std::size_t footBody, float minimumNormalDot, bool contactsComplete, float duration);
}

#endif
