#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_ROOTSTRENGTH_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_ROOTSTRENGTH_HPP

#include <ragdollcore/uprightholder.hpp>

#include <array>
#include <cstdint>

namespace RagdollCore
{
    // KINETIC-2 phase 4, root strength (PLAN.md "KINETIC-2": "root strength
    // becomes one function with named budget terms"). The soft root holds the
    // pelvis to the capsule with springs; three numbers say how hard:
    // - strength: scales every soft-root term (horizontal, tilt, vertical,
    //   weight support);
    // - horizontal: the horizontal hold's share on top of the strength;
    // - support: the share of the body's weight the root carries.
    // Each comes from named terms, every one reported (RootStrength), so a
    // trace says which term set the number. Moved from the old integration's
    // updateKinematicRoot behind OPENMW_RAGDOLL_K2_ROOT; the old path stays
    // beside it until the user's verdict. The weight budget (the support
    // handed to the legs) keeps its values: it waits on the legs carrying the
    // weight (MOVE-01 W3-W5).
    //
    // Pure: no engine types.

    // The drop roll's phases as the root sees them (None also while armed).
    enum class RootRollPhase : std::uint8_t
    {
        None,
        Absorb,
        Roll,
        Rise,
    };

    struct RootStrengthSettings
    {
        // A get-up's strength cap (the hit settings' get-up strength), reached
        // through the ramp below.
        float mGetUpStrength = 1.0f;
        // Getting up, and after a topple rehold, the strength grows from
        // [0] to full over [1] seconds: at full strength from the clip's first
        // frame it yanked the lying body to the clip's pose (2026-09-26).
        // OPENMW_RAGDOLL_GETUP_RAMP ([1] 0 = no ramp).
        std::array<float, 2> mGetUpRamp{ 0.2f, 1.2f };
        // PGC knob A, the root budget: standing, moving, braking (the body
        // 20+ u/s faster than its request). OPENMW_RAGDOLL_PGC_BUDGET.
        bool mBudget = true;
        std::array<float, 3> mBudgetShares{ 0.15f, 0.3f, 0.45f };
        // Above this speed (asked or the body's) the budget weakens only the
        // horizontal hold; weight and vertical stay whole (unarmed running
        // "choppy", 2026-09-30). OPENMW_RAGDOLL_PGC_MOVE_VERTICAL (0 = off).
        float mMoveVertical = 180.0f;
        // Climbing faster than this (u/s) the root keeps its whole strength
        // (STAIRS-01). OPENMW_RAGDOLL_PGC_CLIMB_HOLD (0 = off).
        float mClimbHold = 100.0f;
        TiltHoldSettings mTiltHold;
        // K2_UNEVEN: on uneven ground the lift's strength returns as the
        // pelvis hangs from [0] to [1] u under its target ([1] <= [0]: off).
        std::array<float, 2> mSagHold{ 0.0f, 0.0f };
        // The horizontal hold while a catch step runs
        // (OPENMW_RAGDOLL_CATCH_HOLD) and just after a jump lands
        // (OPENMW_RAGDOLL_LANDING_HOLD); 1 = whole.
        float mCatchHold = 1.0f;
        float mLandingHold = 1.0f;
        // LEGS_LEAD: the legs carry the body, the root only steadies it.
        float mLegsLeadShare = 0.0f;
        // The drop roll (DROP-01): the root's strength while rolling.
        float mRollStrength = 0.35f;
        // K2_CALM: the strength of a root holding a calm NPC to the
        // animation (RootStrengthInput::mCalm blends it in).
        float mCalmStrength = 0.0f;
    };

    struct RootStrengthInput
    {
        // The strength the hit rules leave (recoverRootStrength), 0-1.
        float mHitStrength = 1.0f;
        bool mRecoveryOwns = false; // getting up or blending back
        float mRequestSpeed = 0.0f; // asked, horizontal u/s
        float mBodySpeed = 0.0f; // the centre of mass's, horizontal u/s
        float mLandingFloor = 0.0f; // the PGC landing's floor on the strength
        bool mAirborne = false;
        float mVerticalVelocity = 0.0f; // the capsule's, u/s
        TiltHoldInput mTilt;
        bool mCatchRecovering = false;
        bool mLanding = false; // a jump's landing window runs
        float mStumble = 0.0f; // the bearings' stumble share (PGC_FALLS)
        bool mLegsLeading = false;
        bool mUneven = false; // slope, stairs or rough ground (K2_UNEVEN)
        float mSag = 0.0f; // the pelvis under its target, u
        float mCalm = 0.0f; // the calm hold's level, 0-1 (calmhold.hpp)
        RootRollPhase mRoll = RootRollPhase::None;
        float mRollPhaseTime = 0.0f;
    };

    struct RootStrengthState
    {
        float mGetUpElapsed = 0.0f; // seconds in the current get-up
        float mReholdElapsed = 1000.0f; // seconds since a topple rehold
        float mHorizontalBudget = 1.0f; // the budget's horizontal share
        TiltHoldState mTilt;
    };

    // The terms, in the order they apply.
    struct RootStrength
    {
        float mGetUpRamp = 1.0f;
        float mReholdRamp = 1.0f;
        float mHeld = 1.0f; // min(hit strength, rehold ramp)
        float mBudgeted = 1.0f; // after the budget, landing floor and climb hold
        bool mMovingVertical = false; // the budget weakened only the horizontal hold
        bool mClimbing = false;
        float mSagHold = 0.0f; // the share of the held strength a hanging pelvis got back (K2_UNEVEN)
        float mTiltShare = 0.0f;
        float mStrength = 1.0f; // final, after recovery and the roll
        float mHorizontalBudget = 1.0f;
        float mHorizontalHold = 1.0f; // catch, landing and stumble
        float mHorizontal = 1.0f; // final
    };

    // Hit recovery: the strength a hit took grows back at `perSecond`,
    // slower at low weapon control (user 2026-09-30: "takes more to recover
    // from a hit"; OPENMW_RAGDOLL_HIT_RECOVERY_SKILL = the share lost at no
    // control).
    float recoverRootStrength(
        float strength, float perSecond, float weaponControl, float recoverySkill, float dt);

    RootStrength updateRootStrength(
        const RootStrengthInput& input, const RootStrengthSettings& settings, RootStrengthState& state, float dt);

    // The weight the root carries (setSoftRootSupport), on a held root only.
    struct RootSupportSettings
    {
        // Weight goes to the ground only through a foot: with no ground
        // contact the support fades within 0.1 s (off: a held root is on its
        // feet, and the fade dropped jumps). OPENMW_RAGDOLL_SOFT_ROOT_AIRFADE.
        bool mAirFade = false;
        // Standing on a wounded leg alone the root carries less, so the leg
        // gives (a limp). OPENMW_RAGDOLL_ROOT_BUDGET_LIMP (0 = off).
        float mLimpBudget = 0.0f;
        // The weight budget (MOVE-01 4.6 `support`, W5): walking on the
        // ground (15-150 u/s asked) the root holds this share and the legs the
        // rest, eased over 0.3 s. OPENMW_RAGDOLL_ROOT_WALK_SUPPORT (1 = whole).
        float mWalkSupport = 1.0f;
        float mRollSupport = 0.3f; // the drop roll's
    };

    struct RootSupportInput
    {
        bool mGroundContact = false;
        bool mFootfall = false; // Footfall drives the legs
        std::array<bool, 2> mPinned{}; // left, right: Footfall's pins
        std::array<float, 2> mWound{}; // left, right leg wound 0-1
        float mRequestSpeed = 0.0f;
        bool mAirborne = false;
        // On a slope, stairs or rough ground (K2_UNEVEN): the walk keeps the
        // root's whole lift there.
        bool mUneven = false;
        float mCalm = 0.0f; // the calm hold's level: the root carries the whole weight at 1
        RootRollPhase mRoll = RootRollPhase::None;
        float mRollPhaseTime = 0.0f;
    };

    struct RootSupportState
    {
        float mAir = 1.0f; // the air fade's share
        float mLimp = 1.0f; // eased
        float mWalk = 1.0f; // eased
    };

    float updateRootSupport(
        const RootSupportInput& input, const RootSupportSettings& settings, RootSupportState& state, float dt);
}

#endif
