#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_WHEELOUTPUT_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_WHEELOUTPUT_HPP

#include <ragdollcore/wheelplanner.hpp>

#include <Jolt/Math/Real.h>

#include <optional>

namespace RagdollCore
{
    // KINETIC-2 phase 1, the cut wheel's last touches on the arm request
    // (part 4): applied to the pose sent to the arm, not to the planner's own
    // (its next step starts from where it planned, as in the old path).

    // First person: the guard and the return turned 12 deg right about the
    // chest and the tip leaned out right, faded out over the wind-up's first
    // half, so the blade keeps clear of the crosshair (FP_CLEAR, FPV-01).
    void clearFirstPersonView(ChamberPose& pose, WheelStage stage, float wind);

    // The haft stays out of the body: a staff or spear held by its middle
    // (`butt` units of it behind the hand; only past 15) has its butt turned
    // out about the hand where it would pass within 20 of the body's upright
    // axis (from 10 above the chest centre to 80 below, 6 ahead).
    // HSC_HAFT_CLEAR. Returns the turn (radians).
    float keepHaftOutOfBody(ChamberPose& pose, const JPH::Vec3& bladeAxis, float butt);

    // The support hand on a long haft (spear, staff) slides along it
    // (K2_HAFT_GRIP; user 11-06: "a left hand that slides along the haft"; the
    // 18:19 session: the left hand stayed ~22 u behind the right the whole
    // time, and when the haft turned across the body it was carried over the
    // right forearm - the forearms crossed in 9.5% of staff frames, 0.6% for
    // the longsword). Its point on the haft moves from the authored one
    // `share` of the way to the point nearest the support shoulder, within
    // the arm's `reach` of it, never nearer the weapon hand than `minimum`
    // nor past `butt` (the haft behind the weapon hand, less a hand's half).
    // All positions in one frame (the weapon body's, model units); returns the
    // point's distance from the weapon hand along the haft.
    struct HaftGripSettings
    {
        float mShare = 0.0f; // 0 = off: the authored point
        float mSpeed = 60.0f; // units/s the point moves at (haftGripStep)
        float mMinimum = 8.0f; // a hand's width
    };
    float haftGripTarget(const HaftGripSettings& settings, const JPH::Vec3& primary, const JPH::Vec3& authored,
        const JPH::Vec3& shoulder, float reach, float butt);
    // `current` moved toward `target` by at most speed x dt.
    float haftGripStep(float current, float target, float speed, float dt);

    // A two-handed haft turns about the point between the hands (K2_HAFT_PIVOT;
    // user 11-06: "turning around the point between the hands"): the wheel
    // plans a sword's hand, which turns the weapon about the weapon hand, so a
    // staff's support hand swung through a wide arc and the arms crossed. The
    // weapon hand is moved so the haft's midpoint between the hands stays
    // where it would be at the `reference` turn (the guard's):
    // hand += (reference - R) x bladeAxis x support / 2 x share.
    // `support`: the support grip along the blade from the weapon hand
    // (units; negative behind it, toward the butt). Returns the shift
    // (chest-centre frame).
    JPH::Vec3 pivotBetweenHands(
        ChamberPose& pose, const JPH::Quat& reference, const JPH::Vec3& bladeAxis, float support, float share);

    // A steady NPC guard: out of an attack its weapon hand is held in the
    // actor's own frame (position and facing), following the wobbly body
    // only over 0.35 s; a target 45 units off re-seats it (NPC_STEADY,
    // NPC_STEADY_ROOT; build 61: guards bounced, tips at 319 u/s).
    class NpcGuardHold
    {
    public:
        // `target` and `rotation` (world) are replaced by the held ones while
        // `resting`; returns the shift applied to the target.
        JPH::Vec3 step(bool resting, float dt, JPH::RVec3Arg actorPosition, float actorYaw, JPH::RVec3& target,
            JPH::Quat& rotation);
        void reset() { mLocal.reset(); }

    private:
        std::optional<JPH::Vec3> mLocal;
        JPH::Quat mTurn = JPH::Quat::sIdentity();
    };
}

#endif
