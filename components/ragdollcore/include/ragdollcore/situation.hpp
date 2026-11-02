#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_SITUATION_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_SITUATION_H

// PGC-03 A2: one reading of the body's situation per physics step, shared by
// every balance behaviour (stagger, landing, slope stance, edge teeter, hand
// down, crawl). It replaces the single downward ray per foot: a fan of
// ground rays under and ahead of each foot and ahead of the body gives the
// ground's height, slope and edges; with the feet's contacts and the centre
// of mass it gives the support area and where the body's momentum carries it.
//
// Pure: no Jolt bodies or engine types; the ground comes from a callback.

#include <ragdollcore/balancehelper.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Vec3.h>

#include <array>
#include <functional>
#include <optional>
#include <vector>

namespace RagdollCore
{
    struct GroundHit
    {
        JPH::RVec3 mPoint;
        JPH::Vec3 mNormal = JPH::Vec3::sAxisZ();
    };
    // Straight down from `from`, at most `length` units.
    using GroundRay = std::function<std::optional<GroundHit>(const JPH::RVec3& from, float length)>;

    struct SituationInput
    {
        std::array<JPH::RVec3, 2> mFeet{};
        std::array<bool, 2> mFootContact{ false, false };
        JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
        JPH::Vec3 mVelocity = JPH::Vec3::sZero(); // the centre of mass's
        JPH::Vec3 mCommandVelocity = JPH::Vec3::sZero(); // what the actor asks for (horizontal)
        JPH::Vec3 mFacing = JPH::Vec3::sAxisY(); // horizontal unit
        JPH::RVec3 mCapsule = JPH::RVec3::sZero(); // the gameplay actor's position
        // The capsule position is new this step (the game writes it once a
        // rendered frame, so the other fixed steps of a frame repeat it).
        bool mCapsuleFresh = true;
        bool mCapsuleOnGround = true;
        float mGravity = 627.2f; // units/s^2
        float mStepHeight = 34.0f; // drops beyond this ahead are edges
        float mFootHalfLength = 8.0f; // the support area around a contact foot
        float mDt = 1.0f / 60.0f;
    };

    struct FootReading
    {
        bool mGround = false; // ground under the foot within reach
        float mHeight = 0.0f; // ground height under the foot
        float mClearance = 0.0f; // the foot's height above it
        float mSlopeDegrees = 0.0f;
        JPH::Vec3 mDownhill = JPH::Vec3::sZero(); // horizontal, unit (zero on flat ground)
        float mDropAhead = 0.0f; // the largest fall of the ground within the fan ahead (units)
        float mEdgeDistance = -1.0f; // ahead along the travel, -1 = no edge in the fan
    };

    struct Situation
    {
        std::array<FootReading, 2> mFeet;
        // Support: the contact feet's area (a convex polygon, counter-clockwise).
        std::vector<BalanceVec2> mSupport;
        bool mSupported = false;
        // Where the momentum carries the body (linear inverted pendulum) and
        // its signed distance from the support area (positive = inside).
        BalanceVec2 mCapturePoint;
        float mCaptureMargin = 0.0f;
        // Ground under the centre of mass.
        bool mGroundUnderBody = false;
        float mGroundHeight = 0.0f;
        float mBodyAboveGround = 0.0f;
        float mSlopeDegrees = 0.0f; // under the body (the steeper foot when both stand)
        JPH::Vec3 mDownhill = JPH::Vec3::sZero();
        // An edge ahead of the body along its travel (or facing).
        bool mEdgeAhead = false;
        float mEdgeDistance = 0.0f; // from the centre of mass, horizontal
        float mEdgeDrop = 0.0f; // how far the ground falls there (300 = none found)
        JPH::Vec3 mTravel = JPH::Vec3::sAxisY(); // the direction scanned
        // Airborne: no foot in contact and the feet clear of the ground.
        bool mAirborne = false;
        float mAirTime = 0.0f;
        float mTimeToTouchdown = -1.0f; // predicted from the lowest foot's clearance; -1 = not falling
        float mTouchdownSpeed = 0.0f; // vertical, at that touchdown (u/s, positive down)
        // The capsule moves where the actor did not ask (a slope sliding it,
        // a ledge it walked off): its horizontal speed beyond the command.
        bool mCapsuleSliding = false;
        float mCapsuleSlip = 0.0f;
        float mCapsuleVerticalSpeed = 0.0f;
    };

    // Keeps the times and the capsule's last place between steps.
    class SituationReader
    {
    public:
        const Situation& read(const SituationInput& input, const GroundRay& ground);
        const Situation& current() const { return mSituation; }
        void reset();
        // How many rays the last read cast (budget checks).
        int raysCast() const { return mRays; }

    private:
        Situation mSituation;
        std::optional<JPH::RVec3> mLastCapsule;
        float mCapsuleTime = 0.0f; // since mLastCapsule
        float mSlipFiltered = 0.0f;
        float mSlipDownFiltered = 0.0f; // the part of it down the slope
        int mRays = 0;
    };

    // Signed distance from p to a convex polygon (positive inside).
    float signedDistanceToPolygon(const std::vector<BalanceVec2>& polygon, const BalanceVec2& p);
}

#endif
