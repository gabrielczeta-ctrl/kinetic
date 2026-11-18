#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_NPCPOPULATION_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_NPCPOPULATION_HPP

#include <cstddef>
#include <span>
#include <vector>

namespace RagdollCore
{
    // Which NPCs near the player have physical bodies (KINETIC-2 lifecycle).
    // The engine lists every humanoid NPC with its distance to the player;
    // the plan says which bodies to release and which NPCs may get one.
    //
    // OPENMW_RAGDOLL_NPC_BODIES = "count, radius": at most `count` NPC bodies
    // at once (the player's own body is not counted), given to NPCs within
    // `radius` units, nearest first; a body is released beyond
    // radius x sReleaseFactor (hysteresis: an NPC pacing the edge does not
    // flip every frame), when its NPC dies, or, the farthest first, when the
    // count is lowered below the bodies in use. Read live: the options page
    // slider applies on the next physics step. count 0 = no NPC bodies.
    //
    // Pure: no Jolt or engine types.
    struct NpcPopulationLimits
    {
        std::size_t mMaximum = 10;
        float mAcquireRadius = 2000.0f;
        float mReleaseRadius = 3500.0f;
    };

    // The release radius over the acquire radius (3500 / 2000 since 09-17).
    inline constexpr float sReleaseFactor = 1.75f;

    // A full population gives a nearer unpowered NPC the farthest NPC's body
    // only when the newcomer is this many units nearer, one swap an update,
    // so bodies do not trade back and forth between two NPCs at similar range.
    inline constexpr float sSwapMargin = 500.0f;

    // The limits in effect (OPENMW_RAGDOLL_NPC_BODIES, its registered default
    // when unset). Values out of range are clamped.
    NpcPopulationLimits npcPopulationLimits();

    struct NpcPopulationCandidate
    {
        std::size_t mId = 0; // the caller's index
        float mDistance = 0.0f; // to the player, units
        bool mPowered = false; // has a body now
        // May have a body: alive, humanoid, not swimming or suspended.
        // A powered NPC that may not is released.
        bool mEligible = true;
        // Powered but not judged (no reference yet, or dying on its feet):
        // keeps its body and slot.
        bool mHeld = false;
        // Powered by a script (tests, duels): takes a
        // slot and goes when dead or past the release radius, but never for
        // the count or a swap.
        bool mPinned = false;
    };

    struct NpcPopulationPlan
    {
        // Bodies to release, in no particular order.
        std::vector<std::size_t> mRelease;
        // NPCs that may get a body, nearest first; the caller powers them in
        // order until `mFreeSlots` succeeded (a failed build costs no slot).
        std::vector<std::size_t> mAcquire;
        std::size_t mFreeSlots = 0;
    };

    NpcPopulationPlan planNpcPopulation(
        std::span<const NpcPopulationCandidate> candidates, const NpcPopulationLimits& limits);
}

#endif
