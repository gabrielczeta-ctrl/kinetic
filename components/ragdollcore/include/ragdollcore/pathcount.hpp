#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_PATHCOUNT_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_PATHCOUNT_HPP

#include <cstdint>
#include <string>

namespace RagdollCore
{
    // KINETIC-2 step 3: how often the paths the 11-02 audit suspected of
    // never mattering in play actually run. Each counter counts physics steps
    // of one powered actor; Steps is the denominator. A path that stays at 0
    // over the player's normal sessions goes on the deletion list, and these
    // counters go with it.
    //
    // Pure: no Jolt or engine types. Thread-safe (relaxed atomics).
    enum class CountedPath : std::uint8_t
    {
        Steps, // a powered actor's weapon/arm step (the denominator)
        TipControl, // the old tip controller drove the arm (HSC declined)
        BracingPgc, // PGC_BRACE drove the legs of a Bracing body
        GaitClock, // the old GaitPhaseController stepped (Footfall has its own clock)
        AiStrikeOverHsc, // the AI strike drove the arm while HSC owned it
        LegTravelYawWrote, // the four pre-Footfall leg writers: changed the pose...
        LegTravelYawKept, // ...and their value reached the motors (Footfall did not rewrite it)
        RunBackKickWrote,
        RunBackKickKept,
        FootGroundingWrote,
        FootGroundingKept,
        SwingClearanceWrote,
        SwingClearanceKept,
        Count
    };

    void countPath(CountedPath path, bool player);

    // One line, "player steps=N tip=N ... | npc steps=N ...", nonzero
    // counters only; counts since the start of the process.
    std::string pathCountReport();
}

#endif
