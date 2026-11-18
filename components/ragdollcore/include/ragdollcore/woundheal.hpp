#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_WOUNDHEAL_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_WOUNDHEAL_HPP

namespace RagdollCore
{
    // K2_WOUND_HEAL (registry; WND-01 section 2.5): wounds follow the pool
    // they were paid from. A body's wounds (the limb damage, the marks on
    // skin and the stains on clothing) follow its health; the marks on worn
    // armour follow that armour's condition. One rule for both:
    //
    // - When the pool rises, the share of the missing pool that came back is
    //   the share of the remaining wounds that heals: half the missing health
    //   restored heals half of what is left, a full pool leaves none.
    // - While the pool stays full, what is left (bruises from blows that cost
    //   fatigue only, a scratch on a piece that took no wear) fades over
    //   `mFullHours` of game time.
    //
    // Whatever restores the pool heals: a spell, a potion, an altar, rest; a
    // hammer or a smith for armour. Nothing here is saved.
    //
    // Pure: no Jolt or engine types.
    struct WoundHealSettings
    {
        bool mOn = true;
        // Game hours for a full pool's remaining wounds to go; 0 = they stay.
        float mFullHours = 1.0f;
    };

    // "on, full hours" (the K2_WOUND_HEAL value); null or unreadable fields
    // keep the defaults above.
    WoundHealSettings parseWoundHealSettings(const char* text);

    // One per pool (an actor's health, an actor's worn armour). A default
    // state has seen nothing: its first step only takes the readings.
    struct WoundHealState
    {
        float mLastPool = -1.0f;
        double mLastHours = 0.0;
        float mCarry = 0.0f; // marks healed in part, not yet removed
    };

    // The share (0..1) of the remaining wounds that heals this step, from
    // the pool now, its maximum and the game clock in hours. An empty or dead
    // pool (0 or less) heals nothing; a clock that ran backwards (a loaded
    // game) is not time passed.
    float woundHealShare(WoundHealState& state, float pool, float poolMax, double gameHours,
        const WoundHealSettings& settings);

    // How many of `marks` to remove now, the oldest first, for that share.
    // Fractions carry over, so a slow restore removes a mark at a time.
    int woundMarksHealed(WoundHealState& state, int marks, float share);
}

#endif
