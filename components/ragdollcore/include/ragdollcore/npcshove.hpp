#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_NPCSHOVE_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_NPCSHOVE_HPP

namespace RagdollCore
{
    // SHOVE-02 (user 11-07: "enemies should be able to shove the player as
    // well"): when an NPC in melee shoves its foe, with the player's own shove
    // (SHOVE-01: both hands to the chest, a no-damage push through the blow
    // reaction). Close in and facing the foe, between its own attacks, rested
    // enough, it shoves at a rate - higher when the foe hides behind a raised
    // guard (the shove that opens a turtle) - stronger NPCs more often; never
    // twice within the cooldown. Pure: the caller rolls the dice.

    struct NpcShoveSettings
    {
        float mGuardRate = 0.35f; // shoves per second into a raised guard, in reach
        float mCloseRate = 0.12f; // per second at close range otherwise
        float mCooldown = 4.0f; // s between one NPC's shoves
        float mMinFatigue = 0.3f; // its fatigue share below which it never shoves
        float mReach = 65.0f; // flat units, the chests' distance
        float mClose = 50.0f; // closer than this counts as close range
        float mCone = 0.8f; // cos of the angle off its facing
    };

    struct NpcShoveInput
    {
        float mDistance = 1e9f; // flat units between the chests
        float mFacing = 0.0f; // cos of the foe off the NPC's facing
        bool mFoeGuarding = false;
        bool mAttacking = false; // its own attack under way
        float mFatigue = 1.0f; // 0..1
        float mStrength = 50.0f; // the attribute
        float mSinceLast = 1e9f; // s since its last shove
        float mDt = 0.0f; // s this decision covers
        float mRoll = 1.0f; // uniform 0..1
    };

    // The chance this decision shoves (0 when it cannot), and the decision
    // against the roll.
    float npcShoveChance(const NpcShoveInput& input, const NpcShoveSettings& settings);
    bool npcShouldShove(const NpcShoveInput& input, const NpcShoveSettings& settings);
}

#endif
