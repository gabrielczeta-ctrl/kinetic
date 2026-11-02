#include "fixtures.hpp"
#include <Jolt/Physics/Collision/ManifoldBetweenTwoFaces.h>
#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>

TEST(ContactReduction, tiltedRectangleRetainsFourSupportCorners)
{
    if (!std::getenv("OPENMW_JOLT_GEOMETRIC_SECOND_CONTACT"))
        GTEST_SKIP() << "A9 candidate is opt-in; baseline red result is archived";
    // Two deep adjacent corners, two shallow corners and an intermediate edge point.
    JPH::ContactPoints a, b;
    const JPH::Vec3 points[] = {{-2,-8,-1.7f},{-2,8,-1.7f},{2,8,-1.6f},
        {2,0,-1.6f},{2,-8,-1.6f}};
    for (auto p : points) { a.push_back(p); b.push_back({p.GetX(),p.GetY(),0}); }
    JPH::PruneContactPoints(JPH::Vec3::sAxisZ(),a,b JPH_IF_DEBUG_RENDERER(, JPH::RVec3::sZero()));
    ASSERT_EQ(a.size(),4u);
    for (unsigned i=0;i<a.size();++i)
    {
        EXPECT_EQ(std::abs(a[i].GetX()),2);
        EXPECT_EQ(std::abs(a[i].GetY()),8);
        EXPECT_EQ(a[i].GetX(),b[i].GetX());
        EXPECT_EQ(a[i].GetY(),b[i].GetY());
    }
}
