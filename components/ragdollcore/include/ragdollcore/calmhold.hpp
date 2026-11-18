#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_CALMHOLD_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_CALMHOLD_HPP

namespace RagdollCore
{
    // K2_CALM (registry): a calm NPC's body is held to the game's animation -
    // the root near its whole strength with its whole sideways hold and
    // weight support, the legs on the clip instead of the stepping
    // controller - and is let go to stand on its own legs when something
    // bothers it (a fight, a blow, a shove, a hard bump). `level` is the
    // blend: 1 held, 0 the kinetic body. A held body is drawn as the
    // animation itself (`draw`), so a calm NPC looks as in the original game;
    // the physical body under it stays where a blow or a shove finds it.
    //
    // Pure: no engine types.

    struct CalmHoldSettings
    {
        float mStrength = 0.0f; // the held root's strength, 0-1 (0 = off)
        float mEaseDown = 0.2f; // seconds from held to kinetic
        float mQuiet = 4.0f; // seconds unbothered and upright before the hold returns
        float mEaseUp = 1.0f; // seconds from kinetic to held
        // The pelvis this far (u, horizontal) off where the animation has it
        // while held = something pushes the body: bothered.
        float mBump = 14.0f;
        float mBumpFor = 0.15f; // seconds it must stay that far (a walker's start or turn is shorter)
    };
    CalmHoldSettings calmHoldSettings(); // OPENMW_RAGDOLL_K2_CALM

    struct CalmHoldInput
    {
        bool mAllowed = false; // an NPC's powered body (never the player's)
        bool mBothered = false; // fighting, armed, hit, staggering, falling, getting up, in the air
        bool mUpright = true; // standing on a held root
        float mOffTarget = 0.0f; // the pelvis off the animation's, horizontal u
    };

    struct CalmHoldState
    {
        bool mStarted = false;
        float mLevel = 0.0f;
        float mQuietFor = 0.0f;
        float mOffFor = 0.0f;
        bool mLetGo = false; // bumped: let go all the way before the quiet time counts
        float mDraw = 0.0f;
    };

    struct CalmHold
    {
        float mLevel = 0.0f; // 0 kinetic .. 1 held
        bool mLegsOnClip = false; // the legs follow the animation this step
        bool mBumped = false; // let go by a bump this step
        // The animation's share of the drawn pose, 0-1: the level, except
        // that it never drops at once (a blow lets the body go in one step;
        // the picture eases to the physical body over the ease-down time).
        float mDraw = 0.0f;
    };

    // A body that starts unbothered starts held (an NPC walking into the
    // player's neighbourhood is not built mid-stride onto its own legs).
    CalmHold updateCalmHold(const CalmHoldInput& input, const CalmHoldSettings& settings, CalmHoldState& state, float dt);

    // Let go at once (a blow: its push must meet the kinetic body).
    void breakCalmHold(CalmHoldState& state);
}

#endif
