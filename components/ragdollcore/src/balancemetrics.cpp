#include <ragdollcore/balancemetrics.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace RagdollCore
{
    namespace
    {
        template <class Vector>
        bool finiteVector(const Vector& value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        }

        bool validMinimumNormalDot(float value)
        {
            return std::isfinite(value) && value >= 0.0f && value <= 1.0f;
        }

        enum class ContactQualification
        {
            Unknown,
            Unloaded,
            Loaded,
        };

        ContactQualification qualify(const GroundContact& contact, float minimumNormalDot)
        {
            if (!finiteVector(contact.mPosition) || !finiteVector(contact.mNormal)
                || !std::isfinite(contact.mPenetrationDepth))
                return ContactQualification::Unknown;
            const float normalLengthSquared = contact.mNormal.LengthSq();
            if (!std::isfinite(normalLengthSquared) || normalLengthSquared <= 1e-8f)
                return ContactQualification::Unknown;
            if (contact.mNormal.GetZ() / std::sqrt(normalLengthSquared) < minimumNormalDot)
                return ContactQualification::Unloaded;
            if (!contact.mNormalImpulse || !std::isfinite(*contact.mNormalImpulse)
                || *contact.mNormalImpulse < 0.0f)
                return ContactQualification::Unknown;
            return *contact.mNormalImpulse > 0.0f ? ContactQualification::Loaded : ContactQualification::Unloaded;
        }
    }

    std::optional<BalanceMassMeasurement> measureBalanceMass(std::span<const BodyTelemetry> bodies,
        std::optional<std::size_t> kinematicRootBody)
    {
        if (bodies.empty() || (kinematicRootBody && *kinematicRootBody >= bodies.size()))
            return std::nullopt;

        const JPH::RVec3 origin = bodies.front().mCenterOfMass;
        if (!finiteVector(origin))
            return std::nullopt;
        double mass = 0.0;
        double position[3]{};
        double velocity[3]{};
        for (const BodyTelemetry& body : bodies)
        {
            if (!std::isfinite(body.mMass) || body.mMass <= 0.0f || !finiteVector(body.mCenterOfMass)
                || !finiteVector(body.mMotion.mLinearVelocity))
                return std::nullopt;
            mass += body.mMass;
            const JPH::RVec3 offset = body.mCenterOfMass - origin;
            position[0] += offset.GetX() * static_cast<double>(body.mMass);
            position[1] += offset.GetY() * static_cast<double>(body.mMass);
            position[2] += offset.GetZ() * static_cast<double>(body.mMass);
            velocity[0] += body.mMotion.mLinearVelocity.GetX() * static_cast<double>(body.mMass);
            velocity[1] += body.mMotion.mLinearVelocity.GetY() * static_cast<double>(body.mMass);
            velocity[2] += body.mMotion.mLinearVelocity.GetZ() * static_cast<double>(body.mMass);
        }
        if (!std::isfinite(mass) || mass <= 0.0 || mass > std::numeric_limits<float>::max())
            return std::nullopt;

        BalanceMassMeasurement result;
        result.mTotalMass = static_cast<float>(mass);
        result.mCenterOfMass = origin + JPH::RVec3(position[0] / mass, position[1] / mass, position[2] / mass);
        result.mCenterOfMassVelocity = JPH::Vec3(static_cast<float>(velocity[0] / mass),
            static_cast<float>(velocity[1] / mass), static_cast<float>(velocity[2] / mass));
        result.mKinematicRootBody = kinematicRootBody;
        if (!finiteVector(result.mCenterOfMass) || !finiteVector(result.mCenterOfMassVelocity))
            return std::nullopt;
        return result;
    }

    BalanceSupportMeasurement measureBalanceSupport(JPH::RVec3Arg query,
        std::span<const BalanceContactPoint> contacts, FootBodyIndices feet,
        const ActiveControllerProfile& profile, bool contactsComplete)
    {
        BalanceSupportMeasurement result;
        if (!contactsComplete || !finiteVector(query) || feet.mLeft == feet.mRight
            || !validMinimumNormalDot(profile.mSupportNormalMinimumDot))
            return result;

        std::vector<GroundContact> qualified;
        double minimumHeight = std::numeric_limits<double>::max();
        double maximumHeight = std::numeric_limits<double>::lowest();
        for (const BalanceContactPoint& point : contacts)
        {
            const GroundContact& contact = point.mContact;
            if (contact.mBodyIndex != feet.mLeft && contact.mBodyIndex != feet.mRight)
                continue;
            switch (qualify(contact, profile.mSupportNormalMinimumDot))
            {
                case ContactQualification::Unknown:
                    return result;
                case ContactQualification::Unloaded:
                    continue;
                case ContactQualification::Loaded:
                    break;
            }
            // Friction and velocity availability do not change the observed
            // support geometry. The projection helper validates friction when
            // present, so pass only the geometry/impulse it needs here.
            GroundContact geometry = contact;
            geometry.mFriction.reset();
            qualified.push_back(geometry);
            minimumHeight = std::min(minimumHeight, static_cast<double>(contact.mPosition.GetZ()));
            maximumHeight = std::max(maximumHeight, static_cast<double>(contact.mPosition.GetZ()));
        }

        const auto projection = estimateFootSupport(query, qualified, feet, profile);
        if (!projection)
            return result;
        if (!projection->hasSupport())
        {
            result.mAvailability = BalanceSupportAvailability::Unsupported;
            return result;
        }
        if (!std::isfinite(projection->mCenterOfMassDistance) || projection->mCenterOfMassDistance < 0.0f
            || !finiteVector(projection->mClosestPoint))
            return result;
        result.mAvailability = BalanceSupportAvailability::Supported;
        result.mLeftFootSupported = projection->mLeftFootSupported;
        result.mRightFootSupported = projection->mRightFootSupported;
        result.mHorizontalDistance = projection->mCenterOfMassDistance;
        result.mClosestPoint = JPH::RVec3(projection->mClosestPoint.GetX(), projection->mClosestPoint.GetY(), query.GetZ());
        result.mMinimumContactHeight = minimumHeight;
        result.mMaximumContactHeight = maximumHeight;
        return result;
    }

    std::optional<JPH::RVec3> calculateBalanceCapturePoint(const BalanceMassMeasurement& mass,
        double supportPlaneHeight, float gravity)
    {
        if (!std::isfinite(mass.mTotalMass) || mass.mTotalMass <= 0.0f || !finiteVector(mass.mCenterOfMass)
            || !finiteVector(mass.mCenterOfMassVelocity) || !std::isfinite(supportPlaneHeight)
            || !std::isfinite(gravity) || gravity <= 0.0f)
            return std::nullopt;
        const double height = mass.mCenterOfMass.GetZ() - supportPlaneHeight;
        if (!std::isfinite(height) || height <= 0.0)
            return std::nullopt;
        const double timeConstant = std::sqrt(height / gravity);
        const JPH::RVec3 result(mass.mCenterOfMass.GetX() + mass.mCenterOfMassVelocity.GetX() * timeConstant,
            mass.mCenterOfMass.GetY() + mass.mCenterOfMassVelocity.GetY() * timeConstant, supportPlaneHeight);
        return finiteVector(result) ? std::optional<JPH::RVec3>(result) : std::nullopt;
    }

    BalanceFootSlipMeasurement measureBalanceFootSlip(std::span<const BalanceContactPoint> contacts,
        std::size_t footBody, float minimumNormalDot, bool contactsComplete, float duration)
    {
        BalanceFootSlipMeasurement result;
        if (!contactsComplete || !validMinimumNormalDot(minimumNormalDot))
            return result;

        bool supported = false;
        bool velocityKnown = true;
        float maximumSpeed = 0.0f;
        for (const BalanceContactPoint& point : contacts)
        {
            const GroundContact& contact = point.mContact;
            if (contact.mBodyIndex != footBody)
                continue;
            switch (qualify(contact, minimumNormalDot))
            {
                case ContactQualification::Unknown:
                    return result;
                case ContactQualification::Unloaded:
                    continue;
                case ContactQualification::Loaded:
                    break;
            }
            supported = true;
            if (!contact.mPointVelocity || !point.mSurfacePointVelocity
                || !finiteVector(*contact.mPointVelocity) || !finiteVector(*point.mSurfacePointVelocity))
            {
                velocityKnown = false;
                continue;
            }
            const JPH::Vec3 normal = contact.mNormal.Normalized();
            const JPH::Vec3 relative = *contact.mPointVelocity - *point.mSurfacePointVelocity;
            const float speed = (relative - normal * relative.Dot(normal)).Length();
            if (!std::isfinite(speed))
                velocityKnown = false;
            else
                maximumSpeed = std::max(maximumSpeed, speed);
        }
        result.mAvailability = supported ? BalanceSupportAvailability::Supported : BalanceSupportAvailability::Unsupported;
        if (!supported || !velocityKnown)
            return result;
        result.mMaximumTangentSpeed = maximumSpeed;
        if (std::isfinite(duration) && duration > 0.0f)
        {
            const float distance = maximumSpeed * duration;
            if (std::isfinite(distance))
                result.mMaximumTangentDistanceThisStep = distance;
        }
        return result;
    }
}
