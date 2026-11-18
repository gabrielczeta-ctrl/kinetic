#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_WHEELAIM_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_WHEELAIM_HPP

#include <ragdollcore/cutwheel.hpp>

#include <optional>

namespace RagdollCore
{
    // KINETIC-2 phase 1, the cut wheel's aim (part 3; docs/ragdoll/designs/
    // K2-WEAPON-phase1.md, "Aim" and "The plane's weight"): where an attack
    // goes - the crosshair point, the assist toward a foe near it, the
    // steering - and how the keyed lines are bent there: the cut plane
    // through the chest centre spanned by the aim and the stroke, the reach
    // out to the aim point, thrusts turned to the aim, fists' and daggers'
    // reach, the player's thrust driven by the body. Chest-centre frame (x
    // right, y forward, z up). Pure.

    struct WheelAimInput
    {
        float mLookPitch = 0.0f; // the actor's pitch, positive looking down
        bool mPlayer = true;
        // The player's crosshair ray (HSC_AIM_POINT): origin and direction in
        // the chest-centre frame (the origin relative to the centre).
        std::optional<JPH::Vec3> mRayOrigin, mRayDirection;
        std::optional<JPH::Vec3> mFoe; // the foe's chest, chest-centre frame
        bool mFoeDowned = false; // lying or on its knees, within reach
        float mArmLength = 55.0f;
        // K2_SPEAR_THRUST: with no foe on the ray, a thrust's crosshair point
        // lies this far ahead of the chest (the point's own reach) instead
        // of at the blade's sweet spot: aimed from the chest at a point 72
        // units along a ray that starts at the eye, 20 units higher, a spear
        // pointed 15 degrees above the view. 0: the sweet spot.
        float mPointReach = 0.0f;
        // K2_FIST_STEP: the foe assist takes a foe this near (flat, from the
        // chest) instead of the tip's length + 40 u - the fists step in to a
        // foe further off than that. 0: the tip's length + 40.
        float mAssistReach = 0.0f;
    };

    // The step's aim.
    struct WheelAim
    {
        float mAzimuth = 0.0f; // radians, positive right
        float mElevation = 0.0f; // radians, positive up
        float mDistance = 0.0f; // to the aim point (0: none)
        float mFoeAssist = 0.0f; // 0..1
        float mFloor = 0.6f; // the lowest elevation allowed (a downed foe: lower)
        float mLowest = 0.9f; // the standing low stance's floor
        float mLowWant = 0.0f; // the low stance wanted 0..1 (the trunk bends, part 4)
        float mThrustness = 0.0f; // 0..1, how much the attack's line is the thrust
        float mLungeGap = -1.0f; // the foe beyond the blade's sweet reach (units; -1 none)
        JPH::Vec3 mDirection = JPH::Vec3::sAxisY();
        JPH::Vec3 mPlaneNormal = JPH::Vec3::sAxisX();
    };

    // The aim before steering: the crosshair point (the player's), the foe
    // assist, the plane's normal for the attack at `angle`.
    WheelAim solveWheelAim(const WheelAimInput& input, float angle, float tip);

    // Adds the steering offset (radians) and recomputes the direction, the
    // plane and the low stance's want.
    void steerWheelAim(WheelAim& aim, float angle, float steerX, float steerY, bool autoAttack);

    // The plane's weight at route position u (0..1 wind-up, 1..2 cut), for a
    // pose taken as the cut's (asCut) or the wind-up's (asWind). A click
    // keeps its keyed line unless it is aimed at a foe.
    float planeWeight(const WheelAim& aim, float angle, bool autoAttack, float u, bool asCut, bool asWind);

    // Puts the pose into the cut plane by `weight` (the blade only where it
    // already lies near it) and reaches the hand out along the aim until the
    // tip meets the aim point (at most 35 units, HSC_AIM_REACH).
    void putInPlane(ChamberPose& pose, const WheelAim& aim, float weight, const JPH::Vec3& bladeAxis, float tip);

    // A thrust's pose turned about the chest toward the aim by `share`
    // (HSC_THRUST_AIM).
    void turnThrust(ChamberPose& pose, const WheelAim& aim, float share);

    // The player's fists and daggers reach out to the aim point plus 5, at
    // most 25 units (HSC_SHORT_REACH), late in a thrusting cut.
    void reachShort(ChamberPose& pose, const WheelAim& aim, float shownU, float tip);

    // The player's thrust drive (THRUST-02): the body's share, -0.35 x the
    // drive drawn back late in the wind-up, up to 1 over the cut's first half.
    float thrustDrive(const WheelAim& aim, bool player, float u, bool asCut);

    // The hand 6 units further along the blade at full drive (not fists or
    // daggers).
    void reachThrust(ChamberPose& pose, float drive, const JPH::Vec3& bladeAxis, bool shortStriker);

    // K2_SPEAR_THRUST: a long haft's thrust points at the aim - the blade
    // turned about the hand, by `share`, until its tip lies on the aim's line
    // from the chest centre (at most 45 degrees; a tip not ahead is left).
    void pointThrust(ChamberPose& pose, const WheelAim& aim, float share, const JPH::Vec3& bladeAxis, float tip);
}

#endif
