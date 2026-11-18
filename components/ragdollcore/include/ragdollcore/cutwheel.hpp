#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_CUTWHEEL_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_CUTWHEEL_HPP

#include <ragdollcore/weaponalignment.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>

#include <array>
#include <optional>
#include <string_view>
#include <vector>

namespace RagdollCore
{
    // KINETIC-2 phase 1, the cut-wheel planner (docs/ragdoll/designs/
    // K2-WEAPON-phase1.md; the rules it reproduces are listed there, "The
    // wheel's rules"). This first part holds what has no state: the eight
    // lines' keys, the routes through them, blending, the pace of an attack
    // and the route's flow. The numbers are the game's (play-2026-11-03-7)
    // and are constants, not switches.
    //
    // Frame: the chest centre (between the shoulders, 6 units down), x right,
    // y forward, z up, units.
    //
    // Pure: no engine types.

    struct ChamberKey
    {
        float mS = 0.0f; // along its route, 0..1
        JPH::Vec3 mHand = JPH::Vec3::sZero();
        JPH::Vec3 mBlade = JPH::Vec3::sAxisY(); // the blade (a fist: the knuckle line)
        JPH::Vec3 mPole = -JPH::Vec3::sAxisZ(); // the elbow's bend direction
        float mTwistDegrees = 0.0f; // the chest's turn
        JPH::Vec3 mEdge = JPH::Vec3::sZero(); // a named edge facing; zero = from the motion
    };

    // One attack: a wind-up route from the guard to its chamber, a cut route
    // from the chamber through the follow-through.
    struct AttackLine
    {
        std::string_view mName;
        float mDragRight = 0.0f, mDragUp = 0.0f; // the wind-up drag's direction
        std::string_view mAttackType; // Morrowind's damage type: chop, slash, thrust
        std::vector<ChamberKey> mWind, mCut;
        // A chain into this line's wind-up (from the last cut, from the
        // point guard) is blended in over this share of it, not all of it:
        // a chamber the far side of the body (K2_HAFT_CHOP's, the point
        // behind the back) is reached along the line, not by the short turn
        // from wherever the weapon was.
        float mChainSpan = 1.0f;
        // The drag angle on the wheel, 0..2pi (0 right, pi/2 up).
        float angle() const;
    };

    using WheelLines = std::array<AttackLine, 8>;

    // Ordered round the wheel by angle: right, up-right, up, up-left, left,
    // down-left, down, down-right.
    const WheelLines& weaponLines();
    const WheelLines& fistLines();
    // K2_SPEAR_SWEEP: the sword lines with a long haft's own left-to-right sweep.
    const WheelLines& spearLines();
    // K2_HAFT_LINES: a long haft's own swings, authored by both hands (the
    // thrust is the sword's). K2_HAFT_CHOP's `deepChop`: the chop wound up
    // until the point hangs behind the back. K2_HAFT_THRUST's `ownThrust`:
    // the dragged thrust side-on, the support hand beside its own hip.
    // K2_HAFT_GUARD's `ownGuard`: the guard with the butt beside the support hip.
    const WheelLines& haftLines(bool deepChop = false, bool ownThrust = false, bool ownGuard = false);

    struct ChamberPose
    {
        JPH::Vec3 mHand = JPH::Vec3::sZero();
        JPH::Vec3 mPole = -JPH::Vec3::sAxisZ();
        JPH::Quat mRotation = JPH::Quat::sIdentity(); // the hand's, in the chest-centre frame
        float mTwist = 0.0f; // the chest's turn, radians
    };

    // The hand rotation at each key. The cut route's edges face the way the
    // blade moves toward the follow-through, the wind-up's back toward the
    // guard; the wind-up's last key may be given (the cut's first), so the
    // turn from one route into the other is seamless. All in one hemisphere.
    std::optional<std::vector<JPH::Quat>> routeRotations(const std::vector<ChamberKey>& keys,
        const WeaponStrikeAxes& axes, bool towardEnd, const std::optional<JPH::Quat>& last);

    // The pose at `s` along a route: hand, twist, elbow pole and rotation on
    // one Catmull-Rom spline through the keys (a slerp per segment changed
    // the turn rate at every key: FLOW-01).
    ChamberPose routePose(const std::vector<ChamberKey>& keys, const std::vector<JPH::Quat>& rotations, float s);

    // From a to b by w: the hand and twist mixed, the elbow pole turned along
    // the shorter arc (mixed straight, two nearly opposite poles flipped:
    // FLOW-01), the rotation slerped the short way.
    ChamberPose blendPose(const ChamberPose& a, const ChamberPose& b, float w);

    // One line at route position u (0..1 the wind-up, 1..2 the cut).
    std::optional<ChamberPose> linePose(const AttackLine& line, const WeaponStrikeAxes& axes, float u);

    // The wheel at `angle`: the two lines either side, blended.
    struct WheelSlot
    {
        std::size_t mFirst = 0, mSecond = 1;
        float mWeight = 0.0f; // toward mSecond
    };
    WheelSlot wheelSlot(float angle);
    std::optional<ChamberPose> wheelPose(const WheelLines& lines, const WeaponStrikeAxes& axes, float angle, float u);

    // The guard: the first line's wind-up start.
    std::optional<ChamberPose> guardPose(const WheelLines& lines, const WeaponStrikeAxes& axes);

    // An attack's timing (the wheel's rules, "Pace" and "The return").
    struct PaceInput
    {
        float mWeight = 20.0f; // the weapon's Morrowind weight (fists: 0)
        bool mTwoHanded = false;
        bool mShort = false; // fists and daggers
        bool mPlayer = true;
        float mControl = 1.0f; // weapon control 0..1 (skill, Agility, Luck, fatigue)
        bool mPlayerRiposte = false; // the player's click within the riposte window
        bool mNpcRiposte = false; // an NPC's riposte (its parry won within the window)
        float mChainFlowGain = 0.0f; // 0..1, how well a chain flows (Meyer's square)
        // The weapon's response (K2_WEAPON_PACE, user 11-04: "50% more
        // responsive"): x the attack speed - the wind-up, cut and return
        // times / it, the route's top speed x it and its acceleration x it^2.
        // The holds (the NPC's tell, the end of a released attack) stay.
        float mResponse = 1.0f;
        // K2_MOMENTUM_WIND: the wind-up this many times as fast (1 = none).
        float mMomentumHaste = 1.0f;
        // K2_DAGGER_THRUST: the player's dagger thrust, nearly uncapped.
        bool mDaggerThrust = false;
        // K2_SPEAR_THRUST_PACE: the player's spear / staff thrust's wind-up,
        // cut and return seconds (/ the response); a wind-up of 0 = the
        // weapon's weight pace.
        float mSpearThrustWind = 0.0f, mSpearThrustCut = 0.0f, mSpearThrustReturn = 0.0f;
    };

    struct Pace
    {
        float mAttackSpeed = 1.0f; // the divisor of the click times, with the player's factor
        float mCarried = 1.6f; // clamp(weight x 0.08, 0.4, 4): the weapon's inertia share
        float mWeightPace = 1.0f / 1.5f; // sqrt(carried / 1.6) / attack speed (a committed cut's floor)
        float mWind = 0.35f; // a click's wind-up, seconds
        float mCut = 0.25f; // a click's cut, seconds
        float mChamberHold = 0.0f; // an NPC's hold in the chamber before the cut (the tell)
        float mReleaseHold = 0.3f; // the hold at the end of a released attack
        float mReturn = 0.4f; // back to the guard
        float mWindHaste = 1.0f; // a riposte's or flowing chain's wind-up speed-up (route flow; any actor)
    };

    Pace attackPace(const PaceInput& input);

    // The weight the wheel and the hand drive pace a weapon at (K2_PACE_FLOOR):
    // its own, never below `floorShare` of the iron weapon of its kind
    // (dagger 3, longsword 20, claymore 27, mace 15, warhammer 32, staff 8,
    // spear 14, war axe 24, battle axe 30). Every weapon body is 1 kg and the
    // arm as strong, so a lighter blade's quicker command outran the body:
    // the glass claymore (16.2) was commanded 70 deg/step and swung 22-25,
    // the steel (27) 57 and 29-33 (FLOW-01, 11-06). Heavier stays slower.
    enum class PacedWeaponKind
    {
        Other,
        ShortBlade,
        LongBlade,
        LongBladeTwoHand,
        BluntOneHand,
        BluntTwoClose,
        BluntTwoWide,
        Spear,
        AxeOneHand,
        AxeTwoHand,
    };
    float pacedWeaponWeight(float weight, PacedWeaponKind kind, float floorShare);

    // The route's place, following the stage's target acceleration-limited.
    struct RouteFlow
    {
        float mU = 0.0f; // 0..1 the wind-up, 1..2 the cut
        float mV = 0.0f; // route units/s
    };

    // At most 40 x attack speed^2 x (1.6 / carried) route units/s^2 (x haste^2),
    // never faster than it can still brake to the target, at most 5 x attack
    // speed x sqrt(1.6 / carried) (x haste); close in it settles first-order.
    void stepRouteFlow(RouteFlow& flow, float target, float dt, const Pace& pace, float haste);

    // The stance circle (counts at mouse sensitivity 1).
    inline constexpr float sStanceRadius = 300.0f;
    inline constexpr float sCommitCounts = 1500.0f;
    inline constexpr float sStanceCutCounts = 3000.0f;

    // The wind-up place of a stance point `rn` counts from the guard: the
    // circle is its first half; past it the rest eases into the chamber with
    // the same slope at the circle (FLOW-01: no second speed peak).
    float windAt(float rn, float stanceRadius = sStanceRadius);
}

#endif
