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
        // K2_EDGE_BITE: an edge stopped by armour it cannot open sticks in it
        // this long (s) instead of being thrown back (0 = it is thrown back).
        float mEdgeStick = 0.0f;
    };

    struct BodyContactResult
    {
        BodyContactOutcome mOutcome = BodyContactOutcome::Glance;
        float mEnergy = 0.0f; // J along the normal
        float mCutEnergy = 0.0f; // J the slot needed
        float mHoldTime = 0.0f; // s, a bite, or an edge stuck in armour (a stop that holds)
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

    // K2_BLOW_PRESS (user 11-05: "the sword should push them away, but the
    // sword shouldnt bounce tf off the other body"): a blade that bites into,
    // or is stopped by, flesh presses on - no hit-stop and no throw-back, the
    // blade's own contact drives the struck body away while the arm's drive
    // carries the stroke through. Plate struck by a blunt head or the flat
    // still rebounds, and an edge stuck in armour still sticks.
    struct BlowPressSettings
    {
        bool mOn = false;
        // Share of the attacker's mass that rides with the blade, by how much
        // of the blade's closing speed is the attacker's own body moving that
        // way (a jumping chop, a step into the cut; user bookmark "body
        // momentum ... the inertia helps it build up"). 0 = the weapon's mass.
        float mBodyShare = 0.0f;
        float mMassCap = 8.0f; // kg, the most a blow carries
        // Hardness from which a stop rebounds as before (mail and plate).
        float mHardFrom = 0.5f;
    };

    // The hit-stop and throw-back a body contact makes (`freeze`, `recoil` as
    // the old rule set them), under press: zero on flesh, kept on a hard slot.
    struct ImpactHold
    {
        float mFreeze = 0.0f;
        float mRecoil = 0.0f;
    };
    ImpactHold pressImpact(BodyContactOutcome outcome, float hardness, float holdTime, ImpactHold before,
        const BlowPressSettings& settings);

    // The mass a blow carries (kg): the weapon's, plus the riding share of the
    // attacker's body when it moves along the blow. `bodyAlong` = the
    // attacker's centre-of-mass speed along the blow (u/s), `approach` = the
    // blade's closing speed (u/s).
    float pressCarriedMass(float weaponMass, float bodyMass, float bodyAlong, float approach,
        const BlowPressSettings& settings);

    // K2_PUSH_KICK (user bookmark 11-05: "design a much better forward kick ...
    // at very high strength a kick should be able to set them flying,
    // strength + something else"). The front push kick: the hips drive the
    // planted sole into the foe's middle, so what lands is the leg plus the
    // share of the body that technique couples through it, at the foot's
    // speed plus the body's own run-in. Strength drives the foot (Strength /
    // 50, no longer capped at 1.4 - fortified Strength launches); Hand-to-hand
    // is the "something else": how much of the body is behind the sole.
    struct PushKickSettings
    {
        bool mOn = false;
        float mBodyBase = 0.1f; // share of the kicker's mass behind the sole at Hand-to-hand 0
        float mBodySkill = 0.25f; // added at Hand-to-hand 100
        float mStrengthCap = 4.0f; // Strength / 50 at most
        // The whole body thrown: this share of the push's momentum over the
        // victim's mass, as a speed every part takes, up to the cap (u/s).
        // The struck part's own push is capped at what a body part can move
        // at, so without this a kick of any strength moved its target the
        // same 45-50 u (KICK-01 11-09). 0 = the struck part only.
        float mCarry = 0.0f, mCarryCap = 0.0f;
        // The kicker thrown back: this share of the carry's momentum.
        float mRecoil = 0.0f;
        // A carry of this speed or more fells an unguarded victim (u/s; 0 =
        // never): carried at 320 u/s a held body slid 130-170 u and stood.
        float mFell = 0.0f;
    };
    struct PushKickInput
    {
        float mStrength = 50.0f, mHandToHand = 50.0f;
        float mKickerMass = 75.0f; // kg
        float mKickerAlong = 0.0f; // the kicker's speed toward the foe, u/s
        float mFootSpeed = 0.0f; // u/s at the touch
        float mLegMass = 14.0f; // kg (KICK's push kg)
        float mBaseSpeed = 240.0f; // u/s (KICK's push u/s)
        bool mGuarded = false;
        float mVictimMass = 75.0f; // kg
    };
    struct PushKick
    {
        float mSpeed = 0.0f; // u/s
        float mMass = 0.0f; // kg
        float mCarry = 0.0f; // u/s the victim's whole body takes
        float mRecoil = 0.0f; // u/s the kicker's whole body takes, backward
        bool mFells = false; // the victim goes down
    };
    PushKick pushKick(const PushKickInput& input, const PushKickSettings& settings);
    // The share of a blow's body carry (Blow::mBodyCarry) a part takes: the
    // trunk, head and arms all of it, the legs less toward the feet, so a
    // thrown body goes over backward off its feet (carried evenly it slid
    // 170 u upright, KICK-01 11-09).
    float bodyCarryShare(AnatomicalRegion region);
}

#endif
