#include <gtest/gtest.h>
#include <ragdollcore/wound.hpp>
#include <limits>

using namespace RagdollCore;

TEST(RagdollCoreWoundTest, TipRequiresDistalContactAndForwardMotion)
{
    DaggerContact hit{ 0.98f, 1, 0, 0, 30, 30, 30 };
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::Puncture);
    hit.mAlong = 0.4f;
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::Bruise);
    hit.mAlong = 0.98f;
    hit.mVelocityTip = -30;
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::Bruise);
    hit.mVelocityTip = 0;
    hit.mNormalTip = 0;
    hit.mNormalEdge = 1;
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::Slash);
}

TEST(RagdollCoreWoundTest, BothEdgesCutButFlatAndHandleDoNot)
{
    DaggerContact hit{ 0.5f, 0, 1, 0, 0, 20, 20 };
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::Slash);
    hit.mNormalEdge = -1;
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::Slash);
    hit.mNormalFlat = 1;
    hit.mNormalEdge = 0;
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::Bruise);
    hit.mAlong = -0.1f;
    hit.mNormalEdge = 1;
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::Bruise);
}

TEST(RagdollCoreWoundTest, RestSeparatingAndInvalidContactsDoNotWound)
{
    DaggerContact hit{ 1, 1, 0, 0, 30, 30, 0.2f };
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::None);
    hit.mApproach = -30;
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::None);
    hit.mApproach = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::None);
}

TEST(RagdollCoreWoundTest, EdgeDrawCutsWithNegligibleApproach)
{
    DaggerContact hit{ 0.5f, 0, 1, 0, 20, 20, 0 };
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::Slash);
    hit.mNormalEdge = 0;
    hit.mNormalFlat = 1;
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::None);
    hit.mSpeed = 0;
    EXPECT_EQ(classifyIronDagger(hit), WoundKind::None);
}

TEST(RagdollCoreWoundTest, ReceiptsSeparateFatigueBruisesFromOpenWounds)
{
    EXPECT_GT(woundSeverity(WoundKind::Bruise, 0, 10), 0);
    EXPECT_EQ(woundSeverity(WoundKind::Slash, 0, 10), 0);
    EXPECT_EQ(woundSeverity(WoundKind::Puncture, 0, 0), 0);
    EXPECT_EQ(woundSeverity(WoundKind::Bruise, -4, -10), 0);
    EXPECT_EQ(woundSeverity(WoundKind::None, 10, 10), 0);
    EXPECT_GT(woundSeverity(WoundKind::Slash, 3, 0), 0);
    EXPECT_EQ(woundSeverity(WoundKind::Puncture, 1000, 0), 1);
}

TEST(RagdollCoreWoundTest, ReactionIsBoundedMirroredAndRecovers)
{
    LocalWoundReaction reaction;
    EXPECT_EQ(reaction.angle(AnatomicalRegion::LeftForearm), 0);
    reaction.begin(0.8f);
    EXPECT_EQ(reaction.angle(AnatomicalRegion::LeftForearm), 0);
    reaction.step(0.08f);
    EXPECT_GT(reaction.angle(AnatomicalRegion::LeftForearm), 0);
    EXPECT_EQ(reaction.angle(AnatomicalRegion::LeftForearm), reaction.angle(AnatomicalRegion::RightForearm));
    EXPECT_LT(reaction.angle(AnatomicalRegion::LeftCalf), reaction.angle(AnatomicalRegion::LeftForearm));
    for (int i = 0; i < 100; ++i)
    {
        reaction.begin(100);
        reaction.step(0.08f);
        EXPECT_LE(reaction.angle(AnatomicalRegion::LeftForearm), 0.28f);
    }
    reaction.step(0.5f);
    EXPECT_EQ(reaction.angle(AnatomicalRegion::LeftForearm), 0);
}

TEST(RagdollCoreLongswordTest, EdgeFlatGuardAndDistalTipHaveSeparateSemantics)
{
    const auto profile = bladeSurfaceProfile("iron longsword");
    ASSERT_TRUE(profile);
    EXPECT_FALSE(bladeSurfaceProfile("iron warhammer"));
    for (float side : {-1.f, 1.f})
        EXPECT_EQ(classifyBladeContact({0.5f, 0, side, 0, 0, 80, 80}, *profile), WoundKind::Slash);
    EXPECT_EQ(classifyBladeContact({0.5f, 0, 0, 1, 0, 80, 80}, *profile), WoundKind::Bruise);
    for (float along : {-0.1f, 0.f, 0.1f})
        EXPECT_EQ(classifyBladeContact({along, 0, 1, 0, 0, 80, 80}, *profile), WoundKind::Bruise);
    EXPECT_EQ(classifyBladeContact({0.98f, 1, 0, 0, 80, 80, 80}, *profile), WoundKind::Puncture);
    EXPECT_EQ(classifyBladeContact({0.85f, 1, 0, 0, 80, 80, 80}, *profile), WoundKind::Bruise);
    EXPECT_EQ(classifyBladeContact({0.98f, 1, 0, 0, -80, 80, 80}, *profile), WoundKind::Bruise);
    EXPECT_EQ(classifyBladeContact({0.98f, 1, 0, 0, 0, 0, 0}, *profile), WoundKind::None);
    EXPECT_EQ(classifyBladeContact({0.98f, 1, 0, 0, 80, 80, -80}, *profile), WoundKind::None);
}

TEST(RagdollCoreWoundTest, ASpearPointLeadingTheMotionPuncturesWithTheSetting)
{
    using namespace RagdollCore;
    // A thrust whose hull normal reads the spearhead's side (K2_POINT_LEAD).
    BladeContact hit;
    hit.mAlong = 0.92f;
    hit.mNormalTip = 0.25f;
    hit.mNormalEdge = 0.2f;
    hit.mNormalFlat = 0.95f;
    hit.mVelocityTip = 330.0f;
    hit.mSpeed = 350.0f;
    hit.mApproach = 0.4f; // sliding in: the side normal sees almost no approach
    WoundProfile spear{ WoundClass::Spear, 0.10f, 0.85f };
    EXPECT_EQ(classifyWeaponContact(hit, spear), WoundKind::None);
    hit.mApproach = 120.0f;
    EXPECT_EQ(classifyWeaponContact(hit, spear), WoundKind::Bruise);
    spear.mPointLead = 0.7f;
    EXPECT_EQ(classifyWeaponContact(hit, spear), WoundKind::Puncture);
    hit.mApproach = 0.4f;
    EXPECT_EQ(classifyWeaponContact(hit, spear), WoundKind::Puncture);
    hit.mApproach = 120.0f;
    // The shaft still bruises; a point carried sideways or drawn back still
    // bruises; a point moving away is nothing.
    hit.mAlong = 0.5f;
    EXPECT_EQ(classifyWeaponContact(hit, spear), WoundKind::Bruise);
    hit.mAlong = 0.92f;
    hit.mVelocityTip = 100.0f;
    EXPECT_EQ(classifyWeaponContact(hit, spear), WoundKind::Bruise);
    hit.mVelocityTip = -330.0f;
    EXPECT_EQ(classifyWeaponContact(hit, spear), WoundKind::Bruise);
    hit.mVelocityTip = 330.0f;
    hit.mApproach = -20.0f;
    EXPECT_EQ(classifyWeaponContact(hit, spear), WoundKind::None);
    // A square thrust punctures either way.
    hit.mApproach = 300.0f;
    hit.mNormalTip = 0.9f;
    spear.mPointLead = 0.0f;
    EXPECT_EQ(classifyWeaponContact(hit, spear), WoundKind::Puncture);
}

TEST(RagdollCoreWoundTest, AnEdgeLeadingTheMotionCutsWithTheSetting)
{
    using namespace RagdollCore;
    // An axe head's contact whose hull normal reads the flat, the motion
    // along the edge (K2_EDGE_BITE).
    BladeContact hit;
    hit.mAlong = 0.85f;
    hit.mNormalTip = 0.1f;
    hit.mNormalEdge = 0.3f;
    hit.mNormalFlat = 0.9f;
    hit.mSpeed = 800.0f;
    hit.mApproach = 500.0f;
    hit.mVelocityEdge = -700.0f;
    WoundProfile axe{ WoundClass::Axe, 0.65f, 0.95f };
    EXPECT_EQ(classifyWeaponContact(hit, axe), WoundKind::Bruise);
    axe.mEdgeLead = 0.6f;
    EXPECT_EQ(classifyWeaponContact(hit, axe), WoundKind::Slash);
    // The haft still bruises; the flat leading still bruises.
    hit.mAlong = 0.4f;
    EXPECT_EQ(classifyWeaponContact(hit, axe), WoundKind::Bruise);
    hit.mAlong = 0.85f;
    hit.mVelocityEdge = 200.0f;
    EXPECT_EQ(classifyWeaponContact(hit, axe), WoundKind::Bruise);
    // A blade too.
    hit.mVelocityEdge = 700.0f;
    WoundProfile blade{ WoundClass::Blade, 0.10f, 0.95f, 0.6f };
    EXPECT_EQ(classifyWeaponContact(hit, blade), WoundKind::Slash);
}
