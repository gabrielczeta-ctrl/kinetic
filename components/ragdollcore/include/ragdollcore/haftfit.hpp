#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_HAFTFIT_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_HAFTFIT_HPP

#include <ragdollcore/cutwheel.hpp>

namespace RagdollCore
{
    // Whether two hands can hold a long haft (spear, staff) in a pose of the
    // cut wheel (user 11-12: "swinging makes the arms get in weird
    // positions"). The wheel plans the weapon hand only; the support hand
    // holds the haft `mGrip` behind it, so a pose asks the support arm for a
    // place nobody chose: inside the trunk, past its reach, or across the
    // weapon arm. Chest-centre frame (cutwheel.hpp), units. Pure.

    struct HaftBody
    {
        // The shoulders from the chest centre before the chest's turn (the
        // weapon arm's on +x), and the arm from shoulder to grip
        // (THRUST-01's rows, 11-12: shoulders 10-12 out and 4-9 up, the arm
        // 34.9).
        float mShoulderOut = 11.0f, mShoulderUp = 6.0f;
        float mArm = 34.9f;
        // The trunk: an upright capsule about the chest centre's axis, from
        // `mTrunkTop` above it to `mTrunkBottom` below, a hand's centre no
        // nearer than `mTrunkRadius`.
        float mTrunkRadius = 9.0f, mTrunkTop = 6.0f, mTrunkBottom = 34.0f;
        // The support hand's place on the haft, behind the weapon hand.
        float mGrip = 21.0f;
    };

    struct HaftFit
    {
        JPH::Vec3 mSupport = JPH::Vec3::sZero(); // where the support hand must be
        float mWeaponReach = 0.0f; // the weapon hand past 0.97 of its arm (units; 0 = within)
        float mSupportReach = 0.0f; // the support hand past 0.97 of its arm
        float mInTrunk = 0.0f; // the support hand inside the trunk (units deep)
        // The support hand further to the weapon side than the weapon hand,
        // seen from the turned chest (units; the forearms cross).
        float mCrossed = 0.0f;
        // The worst of the four.
        float worst() const;
    };

    HaftFit haftFit(const ChamberPose& pose, const JPH::Vec3& bladeAxis, const HaftBody& body = {});

    // The worst fit along a line's two routes (17 places on each) and where
    // (route position u: 0..1 the wind-up, 1..2 the cut).
    struct HaftLineFit
    {
        HaftFit mWorst;
        float mU = 0.0f;
    };
    HaftLineFit haftLineFit(const AttackLine& line, const WeaponStrikeAxes& axes, const HaftBody& body = {});
}

#endif
