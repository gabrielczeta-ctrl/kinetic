#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_CONTACTOUTCOME_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_CONTACTOUTCOME_H

// RXN-02 M7: what a counted contact does, from measured quantities, no dice
// (docs/ragdoll/designs/RXN-02-reaction-controller.md 2.2-2.3).
//  - A blade on a body: glance, bite, stop or cleave.
//  - A blade on a blade: deflect, bounce, knockaway, broken guard, disarm.
// Pure functions; speeds in u/s (converted with sContactUnitsPerMetre),
// masses in kg, energies in J.

#include <ragdollcore/blow.hpp>

#include <cstdint>

namespace RagdollCore
{
    inline constexpr float sContactUnitsPerMetre = 70.0f;

    enum class BodyContactOutcome : std::uint8_t
    {
        Glance, // slides off: chip damage, Impact only
        Bite, // the edge or point goes in and is held briefly
        Stop, // a blunt blow, or a hard slot the edge cannot open: all momentum into the body
        Cleave, // through a soft part; the blade goes on, slower
    };
    const char* bodyContactOutcomeName(BodyContactOutcome outcome);

    struct BodyContactInput
    {
        BlowKind mKind = BlowKind::Edge;
        float mNormalSpeed = 0.0f; // closing speed along the contact normal, u/s
        float mTangentSpeed = 0.0f; // sliding speed, u/s
        // How square the edge (or point) meets the travel: 1 = edge first,
        // 0 = the flat. Ignored for blunt and fists.
        float mEdgeAlignment = 1.0f;
        float mEffectiveMass = 1.0f; // kg the blow brings (weapon and arm)
        float mHardness = 0.0f; // the struck slot: 0 flesh, ~0.3 leather, ~0.6 mail, ~0.9 plate
        bool mSoftPart = true; // a limb or the neck (cleavable), not the trunk or head
    };

    struct BodyContactSettings
    {
        float mShallowRatio = 0.5f; // normal/tangent under this: a glance
        float mMisalignedEdge = 0.45f; // edge alignment under this on a hard slot: a glance
        // Energy (J) an edge needs to cut, at hardness 0 and 1 (between: by
        // hardness squared - mail and plate resist far more than leather).
        float mCutEnergySoft = 8.0f;
        float mCutEnergyHard = 1000.0f;
        // A point needs this share of the cut energy (it concentrates).
        float mPointShare = 0.6f;
        // Energy (J) over the cut energy that goes clean through a soft part.
        float mCleaveEnergy = 120.0f;
        float mCleaveAlignment = 0.7f;
        // How long a bite holds the blade (s), by the energy over the cut.
        float mBiteHoldMin = 0.04f;
        float mBiteHoldMax = 0.08f;
        // Share of the cut energy a cleave spends going through.
        float mCleaveSpend = 2.0f;
    };

    struct BodyContactResult
    {
        BodyContactOutcome mOutcome = BodyContactOutcome::Glance;
        float mEnergy = 0.0f; // J along the normal
        float mCutEnergy = 0.0f; // J the slot needed
        float mHoldTime = 0.0f; // s, a bite
        // The blade's speed after a cleave (u/s), else 0 (bite/stop) or the
        // sliding speed (glance).
        float mSpeedAfter = 0.0f;
        // A multiplier for the RPG price: a glance chips, the rest pay.
        float mDamageScale = 1.0f;
    };

    BodyContactResult classifyBodyContact(const BodyContactInput& input, const BodyContactSettings& settings = {});

    enum class ClashOutcome : std::uint8_t
    {
        Deflect, // the attack slides off along the guard and keeps going
        Bounce, // square and even: the attack is driven back
        Knockaway, // an active parry wins clearly: the attacker's arm is thrown off line
        BrokenGuard, // the attack wins clearly: it goes on, slowed; the guard cannot block for a while
        Disarm, // a clear win over a weak grip: the loser's weapon leaves the hand
    };
    const char* clashOutcomeName(ClashOutcome outcome);

    struct ClashInput
    {
        // The attacking blade's momentum at the contact (kg u/s) and the
        // guard's firmness, as the momentum it can stop (kg u/s): hand and
        // wrist drive x grip (two hands, shield) x lever (strong near the
        // hilt) - the caller measures both.
        float mAttackMomentum = 0.0f;
        float mGuardFirmness = 0.0f;
        // |dot| of the two blades' directions: 0 square, 1 parallel.
        float mBladeAlignment = 0.0f;
        bool mActiveParry = false; // the guard moved into the blade in the parry window
        float mAttackerGrip = 1.0f; // 0-1 (skill, fatigue)
        float mDefenderGrip = 1.0f;
    };

    struct ClashSettings
    {
        float mDeflectAlignment = 0.25f; // above: shallow, the blade slides
        float mEven = 1.5f; // within 1/even .. even: an even exchange
        float mDisarmMargin = 3.0f;
        float mWeakGrip = 0.5f;
        float mKnockawayLockout = 0.4f; // s without a block (the loser)
        float mBrokenGuardLockout = 0.55f;
    };

    struct ClashResult
    {
        ClashOutcome mOutcome = ClashOutcome::Bounce;
        float mRatio = 1.0f; // attack / guard
        // Seconds the attacker or the defender cannot block or start a cut.
        float mAttackerLockout = 0.0f;
        float mDefenderLockout = 0.0f;
        // The share of the attack's speed it keeps going on (deflect,
        // broken guard); 0 when it is stopped or thrown back.
        float mAttackContinues = 0.0f;
        bool mAttackerDisarmed = false;
        bool mDefenderDisarmed = false;
    };

    ClashResult classifyClash(const ClashInput& input, const ClashSettings& settings = {});
}

#endif
