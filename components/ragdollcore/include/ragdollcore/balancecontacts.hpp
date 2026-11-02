#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_BALANCECONTACTS_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_BALANCECONTACTS_H

#include <ragdollcore/balancemetrics.hpp>
#include <ragdollcore/contact.hpp>

namespace RagdollCore
{
    struct BalanceContactObservation
    {
        GroundContactObservation mGround;
        JPH::Vec3 mSurfaceLocalPoint = JPH::Vec3::sZero();
    };

    // Call while the callback owns both body locks; body1/body2 and firstBody
    // have the same meaning as appendGroundContactObservations. Capture both
    // COM-local material points without acquiring locks or changing physics.
    //
    // Replace the whole previous manifold for this body/subshape pair and
    // observed side, preserving the latest solver point order. An empty,
    // mismatched or invalid manifold removes that prior manifold and returns
    // false; the caller must retain an incomplete-contact flag for this body
    // until the next physics step. Never substitute the manifold base offset.
    bool replaceBalanceContactObservations(std::vector<BalanceContactObservation>& out,
        std::size_t bodyIndex, const JPH::Body& body1, const JPH::Body& body2,
        const JPH::ContactManifold& manifold, bool firstBody, float friction);

    struct BalanceContactResolution
    {
        std::vector<BalanceContactPoint> mContacts;
        // Original ragdoll indices, unique in first-observed order. A missing,
        // removed or invalid material point cannot yield a complete support
        // patch. The caller combines this with pre-step activity/update status;
        // successful resolution alone never proves the callback set complete.
        std::vector<std::size_t> mIncompleteBodyIndices;
    };

    // Call after PhysicsSystem::Update has joined all callbacks, before other
    // gameplay impulses/mutations. Positions are the midpoint of the two saved
    // material points transformed by the solved body poses. Normals retain the
    // observed solver normal. Both velocities include angular motion at their
    // respective point, in world units/second. Unknown impulse/velocity remains
    // absent; zero surface velocity is valid only when read from a real body.
    // This measures rigid-body surface motion, not ContactSettings conveyors.
    BalanceContactResolution resolveBalanceContacts(
        std::span<const BalanceContactObservation> observations, const JPH::PhysicsSystem& physics);
}

#endif
