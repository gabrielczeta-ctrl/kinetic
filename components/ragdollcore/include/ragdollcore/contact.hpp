#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_CONTACT_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_CONTACT_H

#include <ragdollcore/controller.hpp>

// clang-format off
#include <Jolt/Physics/Collision/ContactListener.h>
// clang-format on

namespace JPH { class PhysicsSystem; class TransformedShape; }

namespace RagdollCore
{
    // Includes a real geometric clearance margin, independent of contact-cache
    // lifetime. Call with current, owned shape snapshots after Update.
    bool contactShapesWithinDistance(const JPH::TransformedShape& first,
        const JPH::TransformedShape& second, float distance);

    struct GroundContactObservation
    {
        GroundContact mContact;
        JPH::SubShapeIDPair mPair;
        JPH::uint mPointIndex = 0;
        bool mFirstBody = true;
        std::optional<JPH::Vec3> mLocalPoint;
    };

    // Call while bodies are locked in the contact callback. Preserve callback
    // order: it is also the order of the solver's cached manifold points.
    void appendGroundContactObservations(std::vector<GroundContactObservation>& out, std::size_t bodyIndex,
        const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold& manifold, bool firstBody, float friction);

    // Call only after PhysicsSystem::Update has finished. An unavailable impulse
    // (for example a CCD-only callback) remains explicitly unknown.
    std::vector<GroundContact> resolveGroundContacts(
        std::span<const GroundContactObservation> observations, const JPH::PhysicsSystem& physics);
}

#endif
