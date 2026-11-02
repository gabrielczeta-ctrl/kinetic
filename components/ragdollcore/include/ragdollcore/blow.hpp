#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_BLOW_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_BLOW_H

// RXN-02 M2: one record per counted blow (docs/ragdoll/designs/
// RXN-02-reaction-controller.md). The strike loop fills it once, after the
// defense outcome and the RPG price are known; every reaction reads it and
// nothing else reads raw contacts for reactions. A value snapshot: no game,
// Lua or rendering types.

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <ragdollcore/anatomy.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

namespace RagdollCore
{
    // What struck: decides how the struck part gives (a cut bites, a blunt
    // blow shoves, a point drives in) and what the attacker's hand feels.
    enum class BlowKind : std::uint8_t
    {
        Edge,
        Blunt,
        Point,
        Fist,
        Scripted, // a test or a script, no weapon
    };

    const char* blowKindName(BlowKind kind);

    // What the struck body was doing when the blow landed.
    struct BlowVictimState
    {
        bool mAttacking = false; // its own cut or wind-up was running
        bool mWindingUp = false;
        bool mGuarding = false;
        bool mMoving = false; // locomotion requested
        bool mReacting = false; // a stun from an earlier blow is still running
        bool mDown = false; // falling, lying or getting up
        float mSpeed = 0.0f; // horizontal body speed, u/s
    };

    struct Blow
    {
        std::uint64_t mId = 0; // unique per process, 0 = none
        std::uint64_t mPhysicsStep = 0;
        std::uint64_t mAttacker = 0; // actor serials
        std::uint64_t mVictim = 0;
        BlowKind mKind = BlowKind::Scripted;

        // Where and which way. The direction is unit length and points into
        // the victim (the push direction); the point is in world space.
        AnatomicalRegion mRegion = AnatomicalRegion::Unknown;
        std::uint32_t mBodyIndex = std::numeric_limits<std::uint32_t>::max();
        JPH::RVec3 mPoint = JPH::RVec3::sZero();
        JPH::Vec3 mDirection = JPH::Vec3::sZero();
        // Horizontal attacker-to-victim direction (not normalised: zero when
        // unknown). A stagger travels this way, whatever the contact normal.
        JPH::Vec3 mAttackerToVictim = JPH::Vec3::sZero();

        // How hard. Speeds in u/s; the carried mass is what the blow brings
        // to the struck part (the weapon and the arm behind it), not the
        // weapon body's own mass.
        float mApproachSpeed = 0.0f;
        float mCarriedMass = 0.0f;
        // The RPG side: damage paid and the reaction impulse priced from it
        // (combat.cpp: damage x reaction per damage x weapon class push,
        // then the region's reaction scale). Band units.
        float mDamage = 0.0f;
        float mReactionImpulse = 0.0f;

        // The defense outcome. A blocked or parried blow still moves the
        // body (it struck something) but dazes nobody and adds no limb
        // damage; mDamage is what was paid (a beaten guard pays a share).
        bool mBlocked = false;
        bool mParried = false;
        // A blocked blow that beat the guard (a hammer through a shield, a
        // broken guard): it may stagger like a blow to the body.
        bool mGuardBeaten = false;
        // A blow with a wound kind (cut, stab or bruise mark) on a known part.
        bool mLocalizedWound = false;
        // A wound receipt follows from the combat script (it adds the limb
        // damage and the local flinch when it arrives).
        bool mReceipted = false;

        BlowVictimState mVictimState;

        bool valid() const
        {
            return mId != 0 && std::isfinite(mApproachSpeed) && mApproachSpeed >= 0.0f
                && std::isfinite(mCarriedMass) && mCarriedMass >= 0.0f && std::isfinite(mDamage)
                && mDamage >= 0.0f && std::isfinite(mReactionImpulse) && mReactionImpulse >= 0.0f
                && std::isfinite(mDirection.GetX()) && std::isfinite(mDirection.GetY())
                && std::isfinite(mDirection.GetZ());
        }
        // Momentum the blow brings, carried mass x approach speed.
        float momentum() const { return mCarriedMass * mApproachSpeed; }
    };

    // The next process-wide blow id (never 0).
    std::uint64_t nextBlowId();
}

#endif
