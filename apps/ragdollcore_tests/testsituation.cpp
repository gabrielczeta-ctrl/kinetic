#include <ragdollcore/situation.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
    using namespace RagdollCore;

    // Flat ground at z = 0 for x < edgeX, nothing (a drop) beyond.
    GroundRay floorWithEdge(float edgeX, float slopeDegrees = 0.0f)
    {
        return [edgeX, slopeDegrees](const JPH::RVec3& from, float length) -> std::optional<GroundHit> {
            if (from.GetX() >= edgeX)
                return std::nullopt;
            // A slope rising along +X.
            const float rise = std::tan(slopeDegrees * 3.14159265f / 180.0f);
            const double z = from.GetX() * rise;
            if (from.GetZ() - z > length || from.GetZ() < z)
                return std::nullopt;
            const JPH::Vec3 n = JPH::Vec3(-rise, 0.0f, 1.0f).Normalized();
            return GroundHit{ JPH::RVec3(from.GetX(), from.GetY(), z), n };
        };
    }

    SituationInput standing()
    {
        SituationInput in;
        in.mFeet = { JPH::RVec3(-6, 0, 2), JPH::RVec3(6, 0, 2) };
        in.mFootContact = { true, true };
        in.mCenterOfMass = JPH::RVec3(0, 0, 90);
        in.mFacing = JPH::Vec3::sAxisY();
        return in;
    }

    TEST(RagdollCoreSituation, standingOnFlatGroundIsSupportedAndCaptured)
    {
        SituationReader reader;
        const auto& s = reader.read(standing(), floorWithEdge(1e6f));
        EXPECT_TRUE(s.mSupported);
        EXPECT_GT(s.mCaptureMargin, 0.0f);
        EXPECT_FALSE(s.mAirborne);
        EXPECT_FALSE(s.mEdgeAhead);
        EXPECT_NEAR(s.mSlopeDegrees, 0.0f, 0.1f);
        EXPECT_TRUE(s.mFeet[0].mGround);
        EXPECT_NEAR(s.mFeet[0].mClearance, 2.0f, 1e-3f);
    }

    TEST(RagdollCoreSituation, aFastPushCarriesTheCapturePointOutOfTheSupport)
    {
        SituationReader reader;
        auto in = standing();
        in.mVelocity = JPH::Vec3(150, 0, 0);
        const auto& s = reader.read(in, floorWithEdge(1e6f));
        EXPECT_LT(s.mCaptureMargin, 0.0f);
    }

    TEST(RagdollCoreSituation, anEdgeAheadAlongTheTravelIsFound)
    {
        SituationReader reader;
        auto in = standing();
        in.mFacing = JPH::Vec3::sAxisX();
        const auto& s = reader.read(in, floorWithEdge(40.0f));
        EXPECT_TRUE(s.mEdgeAhead);
        EXPECT_NEAR(s.mEdgeDistance, 50.0f, 1e-3f);
        EXPECT_GT(s.mEdgeDrop, 100.0f);
        EXPECT_GT(s.mFeet[1].mDropAhead, 34.0f);
        EXPECT_GT(s.mFeet[1].mEdgeDistance, 0.0f);
    }

    TEST(RagdollCoreSituation, aSlopeUnderTheFeetGivesItsAngleAndDownhill)
    {
        SituationReader reader;
        auto in = standing();
        in.mFeet = { JPH::RVec3(-6, 0, -6 * std::tan(0.5236f) + 1), JPH::RVec3(6, 0, 6 * std::tan(0.5236f) + 1) };
        const auto& s = reader.read(in, floorWithEdge(1e6f, 30.0f));
        EXPECT_NEAR(s.mSlopeDegrees, 30.0f, 0.5f);
        EXPECT_LT(s.mDownhill.GetX(), -0.9f);
    }

    TEST(RagdollCoreSituation, aFallingBodyPredictsItsTouchdown)
    {
        SituationReader reader;
        auto in = standing();
        in.mFootContact = { false, false };
        in.mCapsuleOnGround = false;
        in.mFeet = { JPH::RVec3(-6, 0, 100), JPH::RVec3(6, 0, 100) };
        in.mCenterOfMass = JPH::RVec3(0, 0, 190);
        in.mVelocity = JPH::Vec3(0, 0, -100);
        const auto& s = reader.read(in, floorWithEdge(1e6f));
        EXPECT_TRUE(s.mAirborne);
        // 100 = 100 t + 627.2 t^2 / 2 -> t ~ 0.427 s, speed ~ 368 u/s.
        EXPECT_NEAR(s.mTimeToTouchdown, 0.427f, 0.01f);
        EXPECT_NEAR(s.mTouchdownSpeed, 368.0f, 3.0f);
    }

    TEST(RagdollCoreSituation, aCapsuleMovingWithoutTheCommandOnASlopeIsSliding)
    {
        SituationReader reader;
        auto in = standing();
        in.mFeet = { JPH::RVec3(-6, 0, -6 * std::tan(0.6981f) + 1), JPH::RVec3(6, 0, 6 * std::tan(0.6981f) + 1) };
        for (int i = 0; i < 20; ++i)
        {
            in.mCapsule += JPH::RVec3(-2.0, 0, 0); // 120 u/s downhill with no command
            reader.read(in, floorWithEdge(1e6f, 40.0f));
        }
        EXPECT_TRUE(reader.current().mCapsuleSliding);
        EXPECT_GT(reader.current().mCapsuleSlip, 80.0f);
    }

    TEST(RagdollCoreSituation, walkingUpASlopeSlowerThanAskedIsNotSliding)
    {
        // The slope eats speed: asked 127 u/s up a 30 degree ramp, the
        // capsule makes 60. Its slip (67 u/s) points uphill, not down.
        SituationReader reader;
        auto in = standing();
        in.mFeet = { JPH::RVec3(-6, 0, -6 * std::tan(0.5236f) + 1), JPH::RVec3(6, 0, 6 * std::tan(0.5236f) + 1) };
        in.mCommandVelocity = JPH::Vec3(127, 0, 0);
        for (int i = 0; i < 20; ++i)
        {
            in.mCapsule += JPH::RVec3(1.0, 0, 0); // 60 u/s uphill
            reader.read(in, floorWithEdge(1e6f, 30.0f));
        }
        EXPECT_GT(reader.current().mCapsuleSlip, 50.0f);
        EXPECT_FALSE(reader.current().mCapsuleSliding);
    }

    TEST(RagdollCoreSituation, aCapsuleWrittenOnceAFrameFollowingTheCommandDoesNotSlip)
    {
        // 30 fps, two fixed steps a frame: the capsule moves 5 u on the
        // first (fresh) step and repeats on the second, walking exactly at
        // the 150 u/s command up a 40 degree slope.
        SituationReader reader;
        auto in = standing();
        in.mFeet = { JPH::RVec3(-6, 0, -6 * std::tan(0.6981f) + 1), JPH::RVec3(6, 0, 6 * std::tan(0.6981f) + 1) };
        in.mCommandVelocity = JPH::Vec3(150, 0, 0);
        for (int i = 0; i < 40; ++i)
        {
            in.mCapsuleFresh = i % 2 == 0;
            if (in.mCapsuleFresh)
                in.mCapsule += JPH::RVec3(5.0, 0, 0);
            reader.read(in, floorWithEdge(1e6f, 40.0f));
        }
        EXPECT_LT(reader.current().mCapsuleSlip, 5.0f);
        EXPECT_FALSE(reader.current().mCapsuleSliding);
    }

    TEST(RagdollCoreSituation, aShortTeleportIsNotASlide)
    {
        SituationReader reader;
        auto in = standing();
        in.mFeet = { JPH::RVec3(-6, 0, -6 * std::tan(0.6981f) + 1), JPH::RVec3(6, 0, 6 * std::tan(0.6981f) + 1) };
        reader.read(in, floorWithEdge(1e6f, 40.0f));
        in.mCapsule += JPH::RVec3(200.0, 0, 0);
        reader.read(in, floorWithEdge(1e6f, 40.0f));
        EXPECT_LT(reader.current().mCapsuleSlip, 1.0f);
        EXPECT_FALSE(reader.current().mCapsuleSliding);
    }

    TEST(RagdollCoreSituation, theSlopeAheadIsNotAnEdge)
    {
        SituationReader reader;
        auto in = standing();
        in.mFacing = -JPH::Vec3::sAxisX(); // facing downhill
        in.mFeet = { JPH::RVec3(0, -6, 1), JPH::RVec3(0, 6, 1) };
        const auto& s = reader.read(in, floorWithEdge(1e6f, 40.0f));
        EXPECT_FALSE(s.mEdgeAhead);
    }

    TEST(RagdollCoreSituation, signedDistanceToASquare)
    {
        const std::vector<BalanceVec2> square{ { 0, 0 }, { 10, 0 }, { 10, 10 }, { 0, 10 } };
        EXPECT_NEAR(signedDistanceToPolygon(square, { 5, 5 }), 5.0f, 1e-4f);
        EXPECT_NEAR(signedDistanceToPolygon(square, { 15, 5 }), -5.0f, 1e-4f);
    }
}
