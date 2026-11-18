#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_FISTCONTROL_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_FISTCONTROL_HPP

#include <ragdollcore/cutwheel.hpp>

#include <optional>
#include <string_view>

namespace RagdollCore
{
    // FIST-05 (user 11-07: "implement better hand to hand fighting ... can we
    // try something like fight night fistfighting system?"): Fight Night's
    // punch control on the mouse. A flick - the press's drag - picks the
    // punch by its direction and the hand by its side, and its length is the
    // punch's power; both hands throw every punch (the left mirrors the right
    // hand's fist lines across the body). Pure: no engine types.
    //
    // Frame: the cut wheel's (the chest centre, x right, y forward, z up).

    enum class FistHand
    {
        Left,
        Right,
    };

    enum class PunchKind
    {
        Straight, // the jab (left) or the cross (right)
        Hook,
        Uppercut,
        Body, // a straight to the body
    };

    struct FlickPunch
    {
        FistHand mHand = FistHand::Left;
        PunchKind mKind = PunchKind::Straight;
        float mPower = 0.0f; // 0 a quick punch .. 1 a haymaker
    };

    struct FlickSettings
    {
        float mPick = 120.0f; // counts (mouse sensitivity 1) before a press is a flick
        float mFull = 900.0f; // counts at full power
    };

    // The punch a flick throws (`right`, `up`: its drag in counts):
    //   up-left: the left jab; up-right: the right cross;
    //   straight up: the next straight of the alternation (`nextStraight`);
    //   to the left: the right hook (thrown across from the right), to the
    //   right: the left hook;
    //   down-left / down-right: that hand's uppercut;
    //   straight down: the next body shot of the alternation.
    // Crouched, the straights go to the body. None below the pick.
    std::optional<FlickPunch> classifyFlick(
        float right, float up, bool crouched, FistHand nextStraight, const FlickSettings& settings = {});

    // Align a commanded fist with the current camera ray, in shoulder-relative
    // coordinates. Keeps its authored extension and the arm's reach. When the
    // ray misses that reach sphere, aims at its nearest reachable point.
    // This changes a motor target only; collisions still decide whether it lands.
    JPH::Vec3 aimFistTarget(JPH::Vec3 target, JPH::Vec3 rayOrigin, JPH::Vec3 rayDirection,
        float reach, float weight);

    // A fist's commanded place `elapsed` s after its punch ended: from where
    // the punch left it to where it is held now, eased over `seconds` (the
    // right fist's hold eases in over 0.12 s on the wheel; the left's took
    // its held place at once). `seconds` 0 or less = the held place.
    JPH::Vec3 easeFistToHold(JPH::Vec3 from, JPH::Vec3 hold, float elapsed, float seconds);

    // The right hand's fist line a punch takes on the wheel (fistLines()).
    std::string_view fistLineName(PunchKind kind);
    const AttackLine* fistLine(PunchKind kind);

    // The left hand's version of a right-hand key or line: mirrored across
    // the body (x and the chest's turn reversed, the pole's x reversed).
    ChamberKey mirroredKey(const ChamberKey& key);
    AttackLine mirroredLine(const AttackLine& line);

    // A punch's timing: wind-up, the strike, the way back (s), and its
    // fatigue as a share of a click's. A haymaker (power 1) winds up 2.5
    // times as long, strikes 25% quicker and costs 3 times the fatigue; low
    // control (0..1: Hand-to-hand, Agility, fatigue) is up to 30% slower.
    struct PunchTiming
    {
        float mWind = 0.1f;
        float mCut = 0.12f;
        float mBack = 0.25f;
        float mFatigue = 1.0f;
    };
    PunchTiming punchTiming(PunchKind kind, float power, float control);

    // K2_FIST_WOUND: a hook or an uppercut thrown from a held fist goes
    // round a curve, and its strike was commanded at 750-900 u/s over
    // 0.10-0.12 s where the hand peaks at 200-460 u/s (the user's 11-17
    // play: the hand covered 40-70% of the strike). Its strike takes at
    // least `seconds` (never quicker than `cut`; 0 = `cut`), and its wind-up
    // to the chamber this share of a quick punch's (0 = the half it has).
    // Straights and body shots keep their times.
    bool woundPunchPaced(PunchKind kind);
    float woundStrikeSeconds(float cut, PunchKind kind, float seconds);
    float woundWindShare(PunchKind kind, float share);

    // A hand's punch: where its fist is `time` s in (the chest-centre frame),
    // whether it strikes, and whether it is over. Through the wind-up and the
    // strike it runs the line's route; then it eases back to the line's guard.
    struct FistPunchPose
    {
        JPH::Vec3 mHand = JPH::Vec3::sZero();
        JPH::Vec3 mPole = -JPH::Vec3::sAxisZ();
        float mTwistDegrees = 0.0f;
        bool mStriking = false;
        bool mDone = false;
    };
    FistPunchPose fistPunchPose(PunchKind kind, FistHand hand, const PunchTiming& timing, float time);
    // The same on any of the right hand's fist lines (FIST-06); `from`: the
    // wind-up starts there (a held fist's place), not at the line's guard.
    FistPunchPose fistLinePunchPose(const AttackLine& rightLine, FistHand hand, const PunchTiming& timing, float time,
        const JPH::Vec3* from = nullptr);
}

#endif
