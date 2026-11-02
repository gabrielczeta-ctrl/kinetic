#include <ragdollcore/contact.hpp>

// clang-format off
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
// clang-format on

#include <algorithm>

namespace RagdollCore
{
    bool contactShapesWithinDistance(const JPH::TransformedShape& first,
        const JPH::TransformedShape& second, float distance)
    {
        if (!first.mShape || !second.mShape) return false;
        JPH::CollideShapeSettings settings;
        settings.mMaxSeparationDistance = distance;
        JPH::AnyHitCollisionCollector<JPH::CollideShapeCollector> collector;
        const auto origin = first.mShapePositionCOM;
        JPH::CollisionDispatch::sCollideShapeVsShape(first.mShape, second.mShape,
            first.GetShapeScale(), second.GetShapeScale(),
            first.GetCenterOfMassTransform().PostTranslated(-origin).ToMat44(),
            second.GetCenterOfMassTransform().PostTranslated(-origin).ToMat44(),
            {}, {}, settings, collector);
        return collector.HadHit();
    }

    void appendGroundContactObservations(std::vector<GroundContactObservation>& out, std::size_t bodyIndex,
        const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold& manifold, bool firstBody, float friction)
    {
        const JPH::SubShapeIDPair pair(body1.GetID(), manifold.mSubShapeID1, body2.GetID(), manifold.mSubShapeID2);
        const auto normal = firstBody ? -manifold.mWorldSpaceNormal : manifold.mWorldSpaceNormal;
        const auto count = std::min(manifold.mRelativeContactPointsOn1.size(), manifold.mRelativeContactPointsOn2.size());
        if (count == 0)
            out.push_back({ { bodyIndex, manifold.mBaseOffset, normal, manifold.mPenetrationDepth, std::nullopt, friction }, pair, 0, firstBody, std::nullopt });
        for (JPH::uint i = 0; i < count; ++i)
        {
            const auto first = manifold.GetWorldSpaceContactPointOn1(i);
            const auto second = manifold.GetWorldSpaceContactPointOn2(i);
            const auto& body = firstBody ? body1 : body2;
            const auto point = firstBody ? first : second;
            const auto localPoint = JPH::Vec3(body.GetInverseCenterOfMassTransform() * point);
            out.push_back({ { bodyIndex, first + (second - first) * 0.5f, normal, manifold.mPenetrationDepth, std::nullopt, friction },
                pair, i, firstBody, localPoint });
        }
    }

    std::vector<GroundContact> resolveGroundContacts(
        std::span<const GroundContactObservation> observations, const JPH::PhysicsSystem& physics)
    {
        std::vector<GroundContact> result;
        result.reserve(observations.size());
        for (const auto& observation : observations)
        {
            auto contact = observation.mContact;
            contact.mPointVelocity.reset();
            float impulse;
            if (physics.GetContactNormalImpulse(observation.mPair, observation.mPointIndex, impulse))
                contact.mNormalImpulse = impulse;
            contact.mFrictionImpulseMagnitude.reset();
            if (physics.GetContactFrictionImpulse(observation.mPair, observation.mPointIndex, impulse))
                contact.mFrictionImpulseMagnitude = impulse;
            if (observation.mLocalPoint)
            {
                const auto id = observation.mFirstBody ? observation.mPair.GetBody1ID() : observation.mPair.GetBody2ID();
                JPH::BodyLockRead lock(physics.GetBodyLockInterface(), id);
                if (lock.Succeeded())
                {
                    const auto& body = lock.GetBody();
                    contact.mPointVelocity = body.GetPointVelocityCOM(body.GetRotation() * *observation.mLocalPoint);
                }
            }
            result.push_back(contact);
        }
        return result;
    }
}
