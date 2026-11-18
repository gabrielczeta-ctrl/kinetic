#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_LODGE_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_LODGE_H

// K2_LODGE (user 11-16: "is it possible to add a behavior so the stab weapons
// physically enter the body? and pin the tip somewhat into the body? ... what
// would be an equivalent for slashing?", then "deploy stab and pin and also
// something for chopping vs slashing"). What a blow that was paid for does to
// the weapon in the body, after contactoutcome has said glance, bite, stop or
// cleave:
//  - Stab: a point goes in as deep as its damage earned and is held on its own
//    line - the body's entry point slides along the blade, not sideways, so
//    the struck body is pinned on the weapon until it is drawn out.
//  - Chop: a square edge lodges a little way in along its line of entry and
//    has to be pulled free (the weapon's contact point slides on a line fixed
//    in the body).
//  - Pass: a drawn cut, or a cleave, goes on through; the part is dragged
//    along the stroke.
// A blunt head, a fist, a glance and an edge that armour stopped keep what
// they did before. Pure functions; distances in world units (u), speeds in
// u/s, masses in kg, impulses in kg u/s.

#include <ragdollcore/contactoutcome.hpp>

#include <Jolt/Jolt.h>

#include <Jolt/Math/Vec3.h>

#include <cstdint>

namespace RagdollCore
{
    enum class LodgeKind : std::uint8_t
    {
        None,
        Stab,
        Chop,
        Pass,
    };
    const char* lodgeKindName(LodgeKind kind);

    struct LodgeSettings
    {
        bool mStab = false, mChop = false, mPass = false;
        // Stab: depth = damage x per-damage, within min..max (u; 1 u is
        // 1.4 cm: 5 damage a thumb's length, 25 a hand and a half).
        float mStabDepthPerDamage = 0.6f, mStabDepthMin = 2.0f, mStabDepthMax = 20.0f;
        float mStabHold = 1.5f; // s at most
        // Chop: shallower and shorter.
        float mChopDepthPerDamage = 0.25f, mChopDepthMin = 1.5f, mChopDepthMax = 7.0f;
        float mChopHold = 0.5f;
        // An edge's bite is a chop when it closes at least this many times
        // faster than it slides; under it the cut is drawn: a pass.
        float mChopRatio = 1.0f;
        // Armour from this hardness: an edge neither lodges nor passes (the
        // armour's own stick or bounce stays). A point that paid still goes in.
        float mHardFrom = 0.5f;
        // Pass: how long the blade goes through the body (s), the share of
        // its sliding momentum the struck part takes along the stroke, and
        // the most of it (kg u/s).
        float mPassTime = 0.2f, mPassDrag = 0.5f, mPassDragCap = 900.0f;
        // The pin. The most force it makes (kg u/s^2; 70 of them a newton),
        // the share of the sideways error taken out a step, the friction
        // along the line at the surface and per unit of depth, and how far
        // off the line (u) tears it out.
        // LODGE-01/02 11-16 at 45000, 3 kg of arm and 12 u: a driven thrust
        // went 17-20 u through a 7 u stop and tore off its line in 0.03-0.37 s,
        // a chop 14 u through a 4 u one.
        float mPinForce = 150000.0f;
        float mPinPull = 0.3f;
        float mFriction = 4000.0f, mFrictionPerDepth = 500.0f;
        float mTearDistance = 25.0f;
        // The weapon is the arm's: this much of the attacker (kg) is behind
        // it in the pin's reduced mass.
        float mArmMass = 12.0f;
        // The weapon drive's share while held (its frequency and its caps;
        // 1 = whole). At 0.25 the hold was no longer (torn in 0.05-0.2 s
        // either way) and SPEAR-01 failed (its thrusts' median 2.19, one
        // run); whole it passed (10.81, one run). LODGE-01's second and
        // fourth thrusts paid 0-2.2 either way (four runs): open.
        float mDriveYield = 1.0f;
        // After a lodge lets go the weapon still goes through the body until
        // it is out, this long at most (s): never a solid blade inside a body.
        float mClearTime = 1.0f;
    };

    struct LodgeInput
    {
        BlowKind mKind = BlowKind::Edge;
        BodyContactOutcome mOutcome = BodyContactOutcome::Glance;
        float mDamage = 0.0f; // health paid
        float mHardness = 0.0f; // the struck slot (contactoutcome's)
        float mNormalSpeed = 0.0f, mTangentSpeed = 0.0f; // u/s
        float mCarriedMass = 1.0f; // kg the blow brings
    };

    struct LodgePlan
    {
        LodgeKind mKind = LodgeKind::None;
        float mDepth = 0.0f; // u, the deepest it may go
        float mHold = 0.0f; // s
        float mDragImpulse = 0.0f; // a pass: kg u/s along the stroke, once
    };
    LodgePlan planLodge(const LodgeInput& input, const LodgeSettings& settings);

    // One step of the pin: a point fixed in one body held on a line fixed in
    // the other. Stab: the point is the body's entry, the line the blade's
    // (origin the tip, axis from the tip toward the grip). Chop: the point is
    // the edge's contact, the line the entry's (origin the entry, axis into
    // the body). Depth is the point's place along the axis.
    struct LodgeStepInput
    {
        JPH::Vec3 mOffset = JPH::Vec3::sZero(); // point - line origin, world
        JPH::Vec3 mAxis = JPH::Vec3::sAxisX(); // unit
        // The point's body's velocity at the point minus the line's body's.
        JPH::Vec3 mRelativeVelocity = JPH::Vec3::sZero();
        float mEffectiveMass = 1.0f; // kg, the two bodies' reduced mass at the point
        float mDepthMax = 0.0f;
        float mAge = 0.0f, mHold = 0.0f, mDt = 1.0f / 60.0f;
    };
    enum class LodgeRelease : std::uint8_t
    {
        Held,
        Out, // drawn out
        Time,
        Torn, // wrenched off its line
    };
    const char* lodgeReleaseName(LodgeRelease release);
    struct LodgeStep
    {
        // On the point's body at the point; its opposite on the line's body.
        JPH::Vec3 mImpulse = JPH::Vec3::sZero();
        float mDepth = 0.0f;
        float mOffLine = 0.0f; // u
        LodgeRelease mRelease = LodgeRelease::Held;
    };
    LodgeStep stepLodge(const LodgeStepInput& input, const LodgeSettings& settings);
}

#endif
