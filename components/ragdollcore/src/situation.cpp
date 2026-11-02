#include <ragdollcore/situation.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        constexpr float sRayFromAbove = 40.0f; // rays start this far above the reference height
        constexpr float sFootRay = 120.0f;
        constexpr float sBodyRay = 340.0f;

        float horizontalLength(JPH::Vec3Arg v)
        {
            return std::hypot(v.GetX(), v.GetY());
        }

        JPH::Vec3 horizontalUnit(JPH::Vec3Arg v, JPH::Vec3Arg fallback)
        {
            const float l = horizontalLength(v);
            return l > 1e-4f ? JPH::Vec3(v.GetX() / l, v.GetY() / l, 0.0f) : fallback;
        }

        void slopeOf(const GroundHit& hit, float& degrees, JPH::Vec3& downhill)
        {
            const JPH::Vec3 n = hit.mNormal.NormalizedOr(JPH::Vec3::sAxisZ());
            degrees = std::acos(std::clamp(n.GetZ(), -1.0f, 1.0f)) * 57.29578f;
            downhill = horizontalLength(n) > 1e-3f ? horizontalUnit(n, JPH::Vec3::sZero()) : JPH::Vec3::sZero();
        }

        float cross(const BalanceVec2& o, const BalanceVec2& a, const BalanceVec2& b)
        {
            return (a.mX - o.mX) * (b.mY - o.mY) - (a.mY - o.mY) * (b.mX - o.mX);
        }

        // Andrew's monotone chain, counter-clockwise.
        std::vector<BalanceVec2> hull(std::vector<BalanceVec2> points)
        {
            if (points.size() < 3)
                return points;
            std::sort(points.begin(), points.end(), [](const BalanceVec2& a, const BalanceVec2& b) {
                return a.mX < b.mX || (a.mX == b.mX && a.mY < b.mY);
            });
            std::vector<BalanceVec2> h(2 * points.size());
            std::size_t k = 0;
            for (const auto& p : points)
            {
                while (k >= 2 && cross(h[k - 2], h[k - 1], p) <= 0.0f)
                    --k;
                h[k++] = p;
            }
            for (std::size_t i = points.size() - 1, t = k + 1; i-- > 0;)
            {
                const auto& p = points[i];
                while (k >= t && cross(h[k - 2], h[k - 1], p) <= 0.0f)
                    --k;
                h[k++] = p;
            }
            h.resize(k - 1);
            return h;
        }
    }

    float signedDistanceToPolygon(const std::vector<BalanceVec2>& polygon, const BalanceVec2& p)
    {
        if (polygon.empty())
            return -1e9f;
        if (polygon.size() == 1)
            return -std::hypot(p.mX - polygon[0].mX, p.mY - polygon[0].mY);
        bool inside = polygon.size() >= 3;
        float best = 1e9f;
        for (std::size_t i = 0; i < polygon.size(); ++i)
        {
            const auto& a = polygon[i];
            const auto& b = polygon[(i + 1) % polygon.size()];
            const float ex = b.mX - a.mX, ey = b.mY - a.mY;
            const float len2 = ex * ex + ey * ey;
            float t = len2 > 0.0f ? ((p.mX - a.mX) * ex + (p.mY - a.mY) * ey) / len2 : 0.0f;
            t = std::clamp(t, 0.0f, 1.0f);
            best = std::min(best, std::hypot(p.mX - (a.mX + ex * t), p.mY - (a.mY + ey * t)));
            if (cross(a, b, p) < 0.0f)
                inside = false;
        }
        return inside ? best : -best;
    }

    void SituationReader::reset()
    {
        mSituation = {};
        mLastCapsule.reset();
        mCapsuleTime = 0.0f;
        mSlipFiltered = 0.0f;
    }

    const Situation& SituationReader::read(const SituationInput& in, const GroundRay& ground)
    {
        Situation s;
        mRays = 0;
        const auto cast = [&](const JPH::RVec3& from, float length) -> std::optional<GroundHit> {
            ++mRays;
            return ground ? ground(from, length) : std::nullopt;
        };
        const JPH::Vec3 flatVelocity(in.mVelocity.GetX(), in.mVelocity.GetY(), 0.0f);
        s.mTravel = horizontalLength(flatVelocity) > 25.0f ? horizontalUnit(flatVelocity, in.mFacing)
                                                           : horizontalUnit(in.mFacing, JPH::Vec3::sAxisY());

        // Under and ahead of each foot.
        float lowestClearance = 1e9f;
        bool anyContact = false;
        for (std::size_t side = 0; side < 2; ++side)
        {
            FootReading& f = s.mFeet[side];
            const JPH::RVec3 foot = in.mFeet[side];
            anyContact = anyContact || in.mFootContact[side];
            const auto under = cast(foot + JPH::RVec3(0, 0, sRayFromAbove), sFootRay);
            if (under)
            {
                f.mGround = true;
                f.mHeight = static_cast<float>(under->mPoint.GetZ());
                f.mClearance = static_cast<float>(foot.GetZ() - under->mPoint.GetZ());
                slopeOf(*under, f.mSlopeDegrees, f.mDownhill);
                lowestClearance = std::min(lowestClearance, f.mClearance);
            }
            for (const float d : { 14.0f, 28.0f, 42.0f })
            {
                const JPH::RVec3 at = foot + JPH::RVec3(s.mTravel * d);
                const auto hit = cast(at + JPH::RVec3(0, 0, sRayFromAbove), sFootRay);
                const float groundHere = f.mGround ? f.mHeight : static_cast<float>(foot.GetZ());
                const float drop = hit ? groundHere - static_cast<float>(hit->mPoint.GetZ()) : sFootRay;
                const float slopeFall = d * std::tan(std::min(f.mSlopeDegrees, 80.0f) * 0.0174533f)
                    * std::max(0.0f, s.mTravel.Dot(f.mDownhill));
                f.mDropAhead = std::max(f.mDropAhead, drop);
                if (f.mEdgeDistance < 0.0f && drop - slopeFall > in.mStepHeight)
                    f.mEdgeDistance = d;
            }
        }

        // Support area: the contact feet, each a short segment along the
        // facing (heel to toe), as one convex polygon.
        std::vector<BalanceVec2> points;
        for (std::size_t side = 0; side < 2; ++side)
        {
            if (!in.mFootContact[side])
                continue;
            const JPH::RVec3 foot = in.mFeet[side];
            const JPH::Vec3 along = in.mFacing * in.mFootHalfLength;
            const JPH::Vec3 across(-in.mFacing.GetY() * 3.0f, in.mFacing.GetX() * 3.0f, 0.0f);
            for (const JPH::Vec3& o : { along + across, along - across, -along + across, -along - across })
                points.push_back({ static_cast<float>(foot.GetX() + o.GetX()), static_cast<float>(foot.GetY() + o.GetY()) });
        }
        s.mSupport = hull(points);
        s.mSupported = !s.mSupport.empty();

        // Ground under the body and the edge ahead of it.
        const auto underBody = cast(in.mCenterOfMass, sBodyRay);
        if (underBody)
        {
            s.mGroundUnderBody = true;
            s.mGroundHeight = static_cast<float>(underBody->mPoint.GetZ());
            s.mBodyAboveGround = static_cast<float>(in.mCenterOfMass.GetZ() - underBody->mPoint.GetZ());
        }
        // The body's slope: the steeper foot that stands, else the body's ray.
        for (const auto& f : s.mFeet)
            if (f.mGround && f.mClearance < 12.0f && f.mSlopeDegrees > s.mSlopeDegrees)
            {
                s.mSlopeDegrees = f.mSlopeDegrees;
                s.mDownhill = f.mDownhill;
            }
        if (s.mSlopeDegrees <= 0.0f && underBody)
            slopeOf(*underBody, s.mSlopeDegrees, s.mDownhill);
        {
            const float reference = s.mGroundUnderBody ? s.mGroundHeight
                : static_cast<float>(std::min(in.mFeet[0].GetZ(), in.mFeet[1].GetZ()));
            s.mEdgeDrop = 0.0f;
            for (const float d : { 20.0f, 35.0f, 50.0f, 65.0f })
            {
                const JPH::RVec3 at = in.mCenterOfMass + JPH::RVec3(s.mTravel * d);
                const auto hit = cast(at, sBodyRay);
                const float drop = hit ? reference - static_cast<float>(hit->mPoint.GetZ()) : 300.0f;
                // On a slope the ground ahead falls by the slope itself: only
                // a drop beyond that is an edge.
                const float slopeFall = d * std::tan(std::min(s.mSlopeDegrees, 80.0f) * 0.0174533f)
                    * std::max(0.0f, s.mTravel.Dot(s.mDownhill));
                if (drop - slopeFall > in.mStepHeight)
                {
                    s.mEdgeAhead = true;
                    s.mEdgeDistance = d;
                    s.mEdgeDrop = drop;
                    break;
                }
            }
        }

        // Momentum: the capture point over the support.
        {
            BalanceState state;
            state.mCenterOfMass = { static_cast<float>(in.mCenterOfMass.GetX()), static_cast<float>(in.mCenterOfMass.GetY()) };
            state.mVelocity = { in.mVelocity.GetX(), in.mVelocity.GetY() };
            state.mHeight = std::max(20.0f, s.mGroundUnderBody ? s.mBodyAboveGround : 90.0f);
            state.mGravity = in.mGravity;
            if (const auto cp = capturePoint(state))
            {
                s.mCapturePoint = *cp;
                s.mCaptureMargin = signedDistanceToPolygon(s.mSupport, *cp);
            }
        }

        // A foot too high for its own ray (a fall): its clearance from the
        // ground under the body.
        if (lowestClearance >= 1e8f && s.mGroundUnderBody)
            lowestClearance = static_cast<float>(std::min(in.mFeet[0].GetZ(), in.mFeet[1].GetZ())) - s.mGroundHeight;
        // Airborne and the touchdown ahead.
        const bool clear = lowestClearance > 6.0f && lowestClearance < 1e8f;
        s.mAirborne = !anyContact && (clear || !in.mCapsuleOnGround);
        s.mAirTime = s.mAirborne ? mSituation.mAirTime + in.mDt : 0.0f;
        if (s.mAirborne && lowestClearance < 1e8f)
        {
            const float vz = in.mVelocity.GetZ();
            const float h = std::max(0.0f, lowestClearance);
            const float g = std::max(in.mGravity, 1.0f);
            const float t = (vz + std::sqrt(std::max(0.0f, vz * vz + 2.0f * g * h))) / g;
            s.mTimeToTouchdown = t;
            s.mTouchdownSpeed = std::max(0.0f, g * t - vz);
        }

        // The capsule's slip: its horizontal speed beyond the command, over
        // the time since its position last came in (per fixed step it
        // aliased: at 30 fps a capsule walking exactly at 150 u/s read 300
        // then 0, a slip of 150 - 'sliding' on any slope). A jump beyond
        // 2000 u/s is a teleport: a new start, no slip.
        mCapsuleTime += std::max(in.mDt, 0.0f);
        s.mCapsuleVerticalSpeed = mSituation.mCapsuleVerticalSpeed;
        if (in.mCapsuleFresh || !mLastCapsule)
        {
            if (mLastCapsule && mCapsuleTime > 0.0f)
            {
                const JPH::Vec3 v = JPH::Vec3(in.mCapsule - *mLastCapsule) / mCapsuleTime;
                if (horizontalLength(v) <= 2000.0f && std::abs(v.GetZ()) <= 2000.0f)
                {
                    const JPH::Vec3 slip(
                        v.GetX() - in.mCommandVelocity.GetX(), v.GetY() - in.mCommandVelocity.GetY(), 0.0f);
                    mSlipFiltered
                        += (horizontalLength(slip) - mSlipFiltered) * std::min(1.0f, mCapsuleTime / 0.1f);
                    // A slide is the capsule going downhill faster than asked.
                    // Walking up a ramp it falls short of the command (the
                    // slope eats speed), and that shortfall read as sliding -
                    // the body led, did not step up, stalled halfway up a 30
                    // degree ramp and slid back down (user 10-29: "walking
                    // uphill gets buggy"; SLOPE-01 walk up 30, build 85 too).
                    const JPH::Vec3 flatV(v.GetX(), v.GetY(), 0.0f);
                    const JPH::Vec3 flatCommand(in.mCommandVelocity.GetX(), in.mCommandVelocity.GetY(), 0.0f);
                    const float down
                        = std::max(0.0f, flatV.Dot(s.mDownhill) - std::max(0.0f, flatCommand.Dot(s.mDownhill)));
                    mSlipDownFiltered += (down - mSlipDownFiltered) * std::min(1.0f, mCapsuleTime / 0.1f);
                    s.mCapsuleVerticalSpeed = v.GetZ();
                }
                else
                    s.mCapsuleVerticalSpeed = 0.0f;
            }
            mLastCapsule = in.mCapsule;
            mCapsuleTime = 0.0f;
        }
        s.mCapsuleSlip = mSlipFiltered;
        // Sliding: slipping on steep ground (a running stop also slips, on
        // the flat, and is not a slide).
        s.mCapsuleSliding = in.mCapsuleOnGround && mSlipDownFiltered > 40.0f && s.mSlopeDegrees > 25.0f;
        mSituation = s;
        return mSituation;
    }
}
