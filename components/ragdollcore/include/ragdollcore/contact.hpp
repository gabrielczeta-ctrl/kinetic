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

    // Whether a contact may stand for the ground under a body
    // (OPENMW_RAGDOLL_K2_HELD_NOT_GROUND): never when either side of it is a
    // held weapon or shield. A blade pressed on a blade at chest height has
    // an upward normal like a floor, and counted as ground it put the
    // "ground" above the pelvis: the body read as lying and was let go.
    inline bool countsAsGroundContact(bool selfHeldItem, bool otherHeldItem, bool heldItemsExcluded)
    {
        return !heldItemsExcluded || (!selfHeldItem && !otherHeldItem);
    }

    // Call while bodies are locked in the contact callback. Preserve callback
    // order: it is also the order of the solver's cached manifold points.
    void appendGroundContactObservations(std::vector<GroundContactObservation>& out, std::size_t bodyIndex,
        const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold& manifold, bool firstBody, float friction);

    // Call only after PhysicsSystem::Update has finished. An unavailable impulse
    // (for example a CCD-only callback) remains explicitly unknown.
    std::vector<GroundContact> resolveGroundContacts(
        std::span<const GroundContactObservation> observations, const JPH::PhysicsSystem& physics);

    // A thin floor's underside (MOVE-01 W2, the prison ship's hold: a slab
    // 0.6 units thick). Jolt lets a body sink by its penetration slop (1.4
    // units in world units) before correcting, so a planted foot reaches past
    // the slab's down-facing underside, which then pushes it down: the leg
    // stops carrying, the root takes the weight, the foot cannot lift (the
    // "jolly walk"). True for a contact between a static and a moving body
    // that pushes the moving one down at a point below its own centre - a
    // floor reached from above, never a ceiling met from below.
    bool rejectsFloorUnderside(const JPH::Body& body1, const JPH::Body& body2, JPH::RVec3Arg baseOffset,
        const JPH::CollideShapeResult& result);
}

#endif
