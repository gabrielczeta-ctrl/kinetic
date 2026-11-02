#include <ragdollcore/balancecontacts.hpp>

// clang-format off
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Physics/PhysicsSystem.h>
// clang-format on

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        template <class Vector>
        bool finiteVector(const Vector& value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        }
    }

    bool replaceBalanceContactObservations(std::vector<BalanceContactObservation>& out,
        std::size_t bodyIndex, const JPH::Body& body1, const JPH::Body& body2,
        const JPH::ContactManifold& manifold, bool firstBody, float friction)
    {
        const JPH::SubShapeIDPair pair(body1.GetID(), manifold.mSubShapeID1, body2.GetID(), manifold.mSubShapeID2);
        std::erase_if(out, [&](const auto& previous) {
            return previous.mGround.mPair == pair && previous.mGround.mFirstBody == firstBody;
        });
        const auto count = manifold.mRelativeContactPointsOn1.size();
        if (count == 0 || count != manifold.mRelativeContactPointsOn2.size()
            || !finiteVector(manifold.mBaseOffset) || !finiteVector(manifold.mWorldSpaceNormal)
            || manifold.mWorldSpaceNormal.LengthSq() <= 1e-8f || !std::isfinite(manifold.mPenetrationDepth))
            return false;
        const auto& self = firstBody ? body1 : body2;
        const auto& surface = firstBody ? body2 : body1;
        const auto normal = firstBody ? -manifold.mWorldSpaceNormal : manifold.mWorldSpaceNormal;
        std::vector<BalanceContactObservation> replacement;
        replacement.reserve(count);
        for (JPH::uint point = 0; point < count; ++point)
        {
            const auto one = manifold.GetWorldSpaceContactPointOn1(point);
            const auto two = manifold.GetWorldSpaceContactPointOn2(point);
            const auto selfLocal = JPH::Vec3(self.GetInverseCenterOfMassTransform() * (firstBody ? one : two));
            const auto surfaceLocal = JPH::Vec3(surface.GetInverseCenterOfMassTransform() * (firstBody ? two : one));
            const auto midpoint = one + (two - one) * 0.5f;
            if (!finiteVector(one) || !finiteVector(two) || !finiteVector(midpoint)
                || !finiteVector(selfLocal) || !finiteVector(surfaceLocal))
                return false;
            BalanceContactObservation observation;
            observation.mGround = { { bodyIndex, midpoint, normal, manifold.mPenetrationDepth,
                std::nullopt, friction }, pair, point, firstBody, selfLocal };
            observation.mSurfaceLocalPoint = surfaceLocal;
            replacement.push_back(observation);
        }
        out.insert(out.end(), replacement.begin(), replacement.end());
        return true;
    }

    BalanceContactResolution resolveBalanceContacts(
        std::span<const BalanceContactObservation> observations, const JPH::PhysicsSystem& physics)
    {
        BalanceContactResolution result;
        result.mContacts.reserve(observations.size());
        for (const auto& observation : observations)
        {
            const auto& ground = observation.mGround;
            BalanceContactPoint point;
            point.mContact = ground.mContact;
            point.mContact.mNormalImpulse.reset();
            point.mContact.mFrictionImpulseMagnitude.reset();
            point.mContact.mPointVelocity.reset();
            float impulse = 0;
            if (physics.GetContactNormalImpulse(ground.mPair, ground.mPointIndex, impulse))
                point.mContact.mNormalImpulse = impulse;
            if (physics.GetContactFrictionImpulse(ground.mPair, ground.mPointIndex, impulse))
                point.mContact.mFrictionImpulseMagnitude = impulse;
            const JPH::BodyID ids[]{
                ground.mFirstBody ? ground.mPair.GetBody1ID() : ground.mPair.GetBody2ID(),
                ground.mFirstBody ? ground.mPair.GetBody2ID() : ground.mPair.GetBody1ID(),
            };
            JPH::BodyLockMultiRead lock(physics.GetBodyLockInterface(), ids, 2);
            const auto* self = lock.GetBody(0);
            const auto* surface = lock.GetBody(1);
            std::optional<JPH::RVec3> selfPosition;
            std::optional<JPH::RVec3> surfacePosition;
            if (self && self->IsInBroadPhase() && ground.mLocalPoint && finiteVector(*ground.mLocalPoint))
            {
                const auto offset = self->GetRotation() * *ground.mLocalPoint;
                const auto position = self->GetCenterOfMassPosition() + JPH::RVec3(offset);
                const auto velocity = self->GetPointVelocityCOM(offset);
                if (finiteVector(position))
                    selfPosition = position;
                if (finiteVector(velocity))
                    point.mContact.mPointVelocity = velocity;
            }
            if (surface && surface->IsInBroadPhase() && finiteVector(observation.mSurfaceLocalPoint))
            {
                const auto offset = surface->GetRotation() * observation.mSurfaceLocalPoint;
                const auto position = surface->GetCenterOfMassPosition() + JPH::RVec3(offset);
                const auto velocity = surface->GetPointVelocityCOM(offset);
                if (finiteVector(position))
                    surfacePosition = position;
                if (finiteVector(velocity))
                    point.mSurfacePointVelocity = velocity;
            }
            bool geometryComplete = false;
            if (selfPosition && surfacePosition)
            {
                const auto midpoint = *selfPosition + (*surfacePosition - *selfPosition) * 0.5f;
                if (finiteVector(midpoint))
                {
                    point.mContact.mPosition = midpoint;
                    geometryComplete = true;
                }
            }
            if (!geometryComplete
                && std::find(result.mIncompleteBodyIndices.begin(), result.mIncompleteBodyIndices.end(),
                    ground.mContact.mBodyIndex) == result.mIncompleteBodyIndices.end())
                result.mIncompleteBodyIndices.push_back(ground.mContact.mBodyIndex);
            result.mContacts.push_back(point);
        }
        return result;
    }
}
