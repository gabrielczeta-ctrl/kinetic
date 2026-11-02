#include <gtest/gtest.h>

#include <ragdollcore/behavior.hpp>

#include <cmath>
#include <limits>

using namespace RagdollCore;

namespace
{
    BehaviorProfile enabledProfile()
    {
        BehaviorProfile profile;
        profile.mEnabled = true;
        return profile;
    }

    BehaviorInput hit(AnatomicalRegion region, float impulse, bool walking = false)
    {
        BehaviorInput input;
        input.mRegion = region;
        input.mImpulse = impulse;
        input.mWalking = walking;
        input.mMoving = walking;
        input.mDuration = 1.0f / 60.0f;
        return input;
    }

    BehaviorOutput apply(BehaviorState& state, const BehaviorProfile& profile, BehaviorInput input,
        int steps = 1, float duration = 1.0f / 60.0f)
    {
        BehaviorOutput output;
        for (int i = 0; i < steps; ++i)
        {
            input.mDuration = duration;
            output = stepBehavior(state, input, profile);
        }
        return output;
    }
}

TEST(RagdollCoreBehaviorTest, RegionHelpersClassify)
{
    EXPECT_TRUE(regionIsHead(AnatomicalRegion::Head));
    EXPECT_TRUE(regionIsHead(AnatomicalRegion::Neck));
    EXPECT_FALSE(regionIsHead(AnatomicalRegion::Chest));
    EXPECT_TRUE(regionIsTorso(AnatomicalRegion::Chest));
    EXPECT_TRUE(regionIsTorso(AnatomicalRegion::Abdomen));
    EXPECT_TRUE(regionIsArm(AnatomicalRegion::LeftForearm));
    EXPECT_TRUE(regionIsArm(AnatomicalRegion::RightHand));
    EXPECT_FALSE(regionIsArm(AnatomicalRegion::LeftThigh));
    EXPECT_TRUE(regionIsLeg(AnatomicalRegion::RightCalf));
    EXPECT_TRUE(regionIsLeft(AnatomicalRegion::LeftHand));
    EXPECT_FALSE(regionIsLeft(AnatomicalRegion::RightHand));
}

TEST(RagdollCoreBehaviorTest, DisabledProfileIsInert)
{
    BehaviorState state;
    BehaviorProfile profile;
    const BehaviorOutput output = apply(state, profile, hit(AnatomicalRegion::Head, 5000.0f), 10);
    EXPECT_EQ(output.mBehavior, BehaviorId::None);
    EXPECT_FLOAT_EQ(output.mStabilizationStrength, 1.0f);
    EXPECT_FLOAT_EQ(output.mAuthorityDropRequest, 0.0f);
    EXPECT_FALSE(output.mCoverFace);
}

TEST(RagdollCoreBehaviorTest, FaceGuardTriggersOnHeadHitAndExpires)
{
    BehaviorProfile profile = enabledProfile();
    BehaviorState state;
    const BehaviorInput input = hit(AnatomicalRegion::Head, profile.mFaceGuardFullImpulse);
    const BehaviorOutput triggered = apply(state, profile, input);
    EXPECT_EQ(triggered.mBehavior, BehaviorId::FaceGuard);
    EXPECT_TRUE(triggered.mCoverFace);
    EXPECT_GT(triggered.mTorsoFlexion, 0.0f);
    EXPECT_LE(triggered.mTorsoFlexion, profile.mMaximumTorsoFlexion);
    EXPECT_GT(triggered.mHeadRecoil, 0.0f);
    EXPECT_LE(triggered.mHeadRecoil, profile.mMaximumHeadRecoil);
    EXPECT_GT(triggered.mAuthorityDropRequest, 0.0f);
    // Expires after the configured duration without a new hit.
    const BehaviorOutput after = apply(state, profile, hit(AnatomicalRegion::Unknown, 0.0f),
        static_cast<int>(profile.mFaceGuardDuration * 60.0f) + 2);
    EXPECT_EQ(after.mBehavior, BehaviorId::None);
    EXPECT_FALSE(after.mCoverFace);
    EXPECT_FLOAT_EQ(after.mAuthorityDropRequest, 0.0f);
}

TEST(RagdollCoreBehaviorTest, FaceGuardNeedsEnoughImpulse)
{
    BehaviorProfile profile = enabledProfile();
    BehaviorState state;
    const BehaviorOutput below = apply(state, profile,
        hit(AnatomicalRegion::Head, profile.mFaceGuardImpulse * 0.5f));
    EXPECT_EQ(below.mBehavior, BehaviorId::None);
}

TEST(RagdollCoreBehaviorTest, DoubledOverOutranksFaceGuard)
{
    BehaviorProfile profile = enabledProfile();
    BehaviorState state;
    const BehaviorOutput output = apply(state, profile,
        hit(AnatomicalRegion::Chest, profile.mDoubledOverFullImpulse));
    EXPECT_EQ(output.mBehavior, BehaviorId::DoubledOver);
    EXPECT_GT(output.mTorsoFlexion, 0.0f);
    EXPECT_GT(output.mAuthorityDropRequest, 0.5f);
    EXPECT_FALSE(output.mCoverFace);
}

TEST(RagdollCoreBehaviorTest, StumbleRequiresMotionAndNonHeadRegion)
{
    BehaviorProfile profile = enabledProfile();
    {
        BehaviorState state;
        const BehaviorOutput walking = apply(state, profile,
            hit(AnatomicalRegion::LeftCalf, profile.mStumbleFullImpulse, true));
        EXPECT_EQ(walking.mBehavior, BehaviorId::StumbleFlail);
    }
    {
        BehaviorState state;
        const BehaviorOutput standing = apply(state, profile,
            hit(AnatomicalRegion::LeftCalf, profile.mStumbleFullImpulse, false));
        EXPECT_EQ(standing.mBehavior, BehaviorId::None);
    }
    {
        // A head hit while walking prefers the guard.
        BehaviorState state;
        const BehaviorOutput head = apply(state, profile,
            hit(AnatomicalRegion::Head, profile.mStumbleFullImpulse, true));
        EXPECT_EQ(head.mBehavior, BehaviorId::FaceGuard);
    }
}

TEST(RagdollCoreBehaviorTest, RecoveryBraceSuppressesCosmeticBehavior)
{
    BehaviorProfile profile = enabledProfile();
    BehaviorState state;
    BehaviorInput input = hit(AnatomicalRegion::Head, profile.mFaceGuardFullImpulse);
    input.mOwned.mRecoveryBrace = true;
    const BehaviorOutput output = apply(state, profile, input);
    EXPECT_EQ(output.mBehavior, BehaviorId::None);
    EXPECT_FALSE(output.mCoverFace);
    EXPECT_FLOAT_EQ(output.mTorsoFlexion, 0.0f);
}

TEST(RagdollCoreBehaviorTest, CradleNeedsReceiptAndRespectsSidesAndOwnership)
{
    BehaviorProfile profile = enabledProfile();
    {
        BehaviorState state;
        BehaviorInput input = hit(AnatomicalRegion::LeftForearm, 0.0f);
        input.mHasReceipt = true;
        input.mWoundSeverity = 0.5f;
        const BehaviorOutput output = apply(state, profile, input, 2);
        EXPECT_TRUE(output.mCradleLeft);
        EXPECT_FALSE(output.mCradleRight);
        EXPECT_TRUE(output.mDisabledLimbs.mLeftArm);
        EXPECT_FALSE(output.mDisabledLimbs.mRightArm);
    }
    {
        // No receipt: transient only, never a persistent cradle.
        BehaviorState state;
        BehaviorInput input = hit(AnatomicalRegion::LeftForearm, 0.0f);
        input.mWoundSeverity = 0.9f;
        const BehaviorOutput output = apply(state, profile, input, 2);
        EXPECT_FALSE(output.mCradleLeft);
    }
    {
        // A weapon-owned arm is never stolen.
        BehaviorState state;
        BehaviorInput input = hit(AnatomicalRegion::RightForearm, 0.0f);
        input.mHasReceipt = true;
        input.mWoundSeverity = 0.9f;
        input.mOwned.mRightArm = true;
        input.mOwned.mWeaponAim = true;
        const BehaviorOutput output = apply(state, profile, input, 2);
        EXPECT_FALSE(output.mCradleRight);
        EXPECT_TRUE(output.mDisabledLimbs.mRightArm);
    }
}

TEST(RagdollCoreBehaviorTest, LimpCapsSpeedAndRespectsSide)
{
    BehaviorProfile profile = enabledProfile();
    for (const bool left : { true, false })
    {
        BehaviorState state;
        BehaviorInput input = hit(left ? AnatomicalRegion::LeftCalf : AnatomicalRegion::RightFoot, 0.0f);
        input.mHasReceipt = true;
        input.mWoundSeverity = 0.8f;
        const BehaviorOutput output = apply(state, profile, input, 2);
        EXPECT_EQ(output.mLimpLeft, left);
        EXPECT_EQ(output.mLimpRight, !left);
        EXPECT_GT(output.mLimpBias, 0.0f);
        EXPECT_LT(output.mSpeedScale, 1.0f);
        EXPECT_GE(output.mSpeedScale, profile.mLimpSpeedScale - 1e-4f);
    }
}

TEST(RagdollCoreBehaviorTest, LostCapabilityDisablesTheLimb)
{
    BehaviorProfile profile = enabledProfile();
    BehaviorState state;
    BehaviorInput input = hit(AnatomicalRegion::RightHand, 0.0f);
    input.mRegionCapable = false;
    input.mHasReceipt = true;
    const BehaviorOutput output = apply(state, profile, input, 2);
    EXPECT_TRUE(output.mDisabledLimbs.mRightArm);
    EXPECT_TRUE(output.mCradleRight);
}

TEST(RagdollCoreBehaviorTest, DegenerateInputsStayBounded)
{
    BehaviorProfile profile = enabledProfile();
    BehaviorState state;
    BehaviorInput input = hit(AnatomicalRegion::Head, std::numeric_limits<float>::quiet_NaN());
    input.mWoundSeverity = std::numeric_limits<float>::infinity();
    const BehaviorOutput output = apply(state, profile, input);
    EXPECT_EQ(output.mBehavior, BehaviorId::None);
    EXPECT_TRUE(std::isfinite(output.mTorsoFlexion));
    EXPECT_TRUE(std::isfinite(output.mSpeedScale));
    EXPECT_TRUE(std::isfinite(output.mStabilizationStrength));
    EXPECT_GE(output.mStabilizationStrength, profile.mMinimumStabilizationStrength);
    EXPECT_LE(output.mStabilizationStrength, 1.0f);
}

TEST(RagdollCoreBehaviorTest, StabilizationStrengthRespectsFloor)
{
    BehaviorProfile profile = enabledProfile();
    BehaviorState state;
    const BehaviorOutput output = apply(state, profile,
        hit(AnatomicalRegion::Abdomen, profile.mDoubledOverFullImpulse), 4);
    EXPECT_LT(output.mStabilizationStrength, 1.0f);
    EXPECT_GE(output.mStabilizationStrength, profile.mMinimumStabilizationStrength);
}
