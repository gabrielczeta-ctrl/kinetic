#include <ragdollcore/pipeline.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <string_view>
#include <vector>

namespace
{
    using namespace RagdollCore;

    MovedAreas everyArea()
    {
        MovedAreas moved;
        for (Area area : { Area::Lifecycle, Area::Root, Area::Legs, Area::Upright, Area::Reaction, Area::Weapon })
            moved.with(area);
        return moved;
    }

    OwnershipInput input(BodyState state, bool weaponReady, MovedAreas moved = everyArea())
    {
        OwnershipInput result;
        result.mState = state;
        result.mWeaponReady = weaponReady;
        result.mMoved = moved;
        return result;
    }

    TEST(PipelineOwnershipTest, NothingMovedIsAllLegacy)
    {
        for (BodyState state : { BodyState::Standing, BodyState::Airborne, BodyState::Staggered, BodyState::Falling,
                 BodyState::Downed, BodyState::GettingUp, BodyState::Dead })
            for (bool weapon : { false, true })
                for (Area area : resolveOwnership(input(state, weapon, MovedAreas{})))
                    EXPECT_EQ(area, Area::Legacy);
    }

    TEST(PipelineOwnershipTest, StandingWithAWeapon)
    {
        // MOVE-01 4.8: the weapon owns the arms and the trunk's turn, the
        // legs the legs and pelvis height, the upright holder the tilt.
        const Ownership owners = resolveOwnership(input(BodyState::Standing, true));
        EXPECT_EQ(owner(owners, Channel::Root), Area::Root);
        EXPECT_EQ(owner(owners, Channel::PelvisHeight), Area::Legs);
        EXPECT_EQ(owner(owners, Channel::LeftLeg), Area::Legs);
        EXPECT_EQ(owner(owners, Channel::RightLeg), Area::Legs);
        EXPECT_EQ(owner(owners, Channel::TrunkTilt), Area::Upright);
        EXPECT_EQ(owner(owners, Channel::Head), Area::Upright);
        EXPECT_EQ(owner(owners, Channel::TrunkTurn), Area::Weapon);
        EXPECT_EQ(owner(owners, Channel::LeftArm), Area::Weapon);
        EXPECT_EQ(owner(owners, Channel::RightArm), Area::Weapon);
    }

    TEST(PipelineOwnershipTest, StandingEmptyHandedTheGaitHasTheArmsAndTurn)
    {
        const Ownership owners = resolveOwnership(input(BodyState::Standing, false));
        EXPECT_EQ(owner(owners, Channel::TrunkTurn), Area::Legs);
        EXPECT_EQ(owner(owners, Channel::LeftArm), Area::Legs);
        EXPECT_EQ(owner(owners, Channel::RightArm), Area::Legs);
    }

    TEST(PipelineOwnershipTest, StaggeredTheReactionHasTheTiltAndFreeArm)
    {
        OwnershipInput staggered = input(BodyState::Staggered, true);
        Ownership owners = resolveOwnership(staggered);
        EXPECT_EQ(owner(owners, Channel::TrunkTilt), Area::Reaction);
        EXPECT_EQ(owner(owners, Channel::Head), Area::Reaction);
        EXPECT_EQ(owner(owners, Channel::LeftArm), Area::Reaction);
        EXPECT_EQ(owner(owners, Channel::RightArm), Area::Weapon);
        EXPECT_EQ(owner(owners, Channel::TrunkTurn), Area::Weapon);
        // Catch steps are steps.
        EXPECT_EQ(owner(owners, Channel::LeftLeg), Area::Legs);

        staggered.mReactionHasWeaponArm = true;
        owners = resolveOwnership(staggered);
        EXPECT_EQ(owner(owners, Channel::RightArm), Area::Reaction);
        EXPECT_EQ(owner(owners, Channel::TrunkTurn), Area::Reaction);

        staggered.mWeaponRight = false;
        owners = resolveOwnership(staggered);
        EXPECT_EQ(owner(owners, Channel::LeftArm), Area::Reaction);
        EXPECT_EQ(owner(owners, Channel::RightArm), Area::Reaction);
    }

    TEST(PipelineOwnershipTest, DownAndDead)
    {
        for (BodyState state : { BodyState::Falling, BodyState::Downed, BodyState::GettingUp })
        {
            const Ownership owners = resolveOwnership(input(state, true));
            for (std::size_t c = 0; c < sChannelCount; ++c)
                EXPECT_EQ(owners[c], static_cast<Channel>(c) == Channel::Root ? Area::Root : Area::Reaction)
                    << channelName(static_cast<Channel>(c));
        }
        for (Area area : resolveOwnership(input(BodyState::Dead, true)))
            EXPECT_EQ(area, Area::Lifecycle);
    }

    TEST(PipelineOwnershipTest, OnlyMovedAreasOwn)
    {
        // The weapon moved first (phase 1): it owns its channels, the rest
        // stays with the old code.
        const Ownership owners = resolveOwnership(input(BodyState::Standing, true, MovedAreas{}.with(Area::Weapon)));
        EXPECT_EQ(owner(owners, Channel::RightArm), Area::Weapon);
        EXPECT_EQ(owner(owners, Channel::TrunkTurn), Area::Weapon);
        EXPECT_EQ(owner(owners, Channel::LeftLeg), Area::Legacy);
        EXPECT_EQ(owner(owners, Channel::TrunkTilt), Area::Legacy);
        EXPECT_EQ(owner(owners, Channel::Root), Area::Legacy);
    }

    struct Body
    {
        const char* mName;
        float mX, mY, mZ;
    };

    constexpr Body sBodies[] = {
        { "pelvis", 0, 0, 90 },
        { "spine", 0, 0, 100 },
        { "spine1", 0, 0, 110 },
        { "spine2", 0, 0, 120 },
        { "neck", 0, 0, 130 },
        { "head", 0, 2, 140 },
        { "left_thigh", -10, 0, 70 },
        { "right_thigh", 10, 0, 70 },
        { "left_upper_arm", -20, 0, 120 },
        { "right_hand", 30, 10, 100 },
        { "weapon", 40, 10, 100 },
    };

    std::vector<std::string_view> names()
    {
        std::vector<std::string_view> result;
        for (const Body& body : sBodies)
            result.push_back(body.mName);
        return result;
    }

    RagdollPose pose()
    {
        RagdollPose result;
        for (const Body& body : sBodies)
            result.mBodyTransforms.push_back({ JPH::Vec3(body.mX, body.mY, body.mZ), JPH::Quat::sIdentity() });
        return result;
    }

    TEST(PipelineComposerTest, MapsBodiesToChannels)
    {
        const std::vector<std::string_view> bodyNames = names();
        const ChannelMap map = channelMapFromNames(bodyNames);
        ASSERT_EQ(map.mBodyChannel.size(), std::size(sBodies));
        EXPECT_EQ(map.mPelvis, std::optional<std::size_t>(0));
        EXPECT_EQ(map.mBodyChannel[0], Channel::Root);
        EXPECT_EQ(map.mBodyChannel[3], Channel::TrunkTilt);
        EXPECT_EQ(map.mBodyChannel[5], Channel::Head);
        EXPECT_EQ(map.mBodyChannel[6], Channel::LeftLeg);
        EXPECT_EQ(map.mBodyChannel[8], Channel::LeftArm);
        EXPECT_EQ(map.mBodyChannel[9], Channel::RightArm);
        EXPECT_FALSE(map.mBodyChannel[10].has_value());
        EXPECT_FLOAT_EQ(map.mTurnShare[1], 0.3f);
        EXPECT_FLOAT_EQ(map.mTurnShare[3], 1.0f);
        EXPECT_FLOAT_EQ(map.mTurnShare[5], 1.0f);
        EXPECT_FLOAT_EQ(map.mTurnShare[9], 0.0f);
    }

    TEST(PipelineComposerTest, TakesWritesFromTheOwnerOnly)
    {
        const std::vector<std::string_view> bodyNames = names();
        const ChannelMap map = channelMapFromNames(bodyNames);
        const Ownership owners = resolveOwnership(input(BodyState::Standing, true, MovedAreas{}.with(Area::Weapon)));
        StepComposer composer;
        composer.begin(pose(), map, owners);
        const Transform hand{ JPH::Vec3(35, 20, 110), JPH::Quat::sIdentity() };
        EXPECT_TRUE(composer.writeBody(Area::Weapon, 9, hand));
        // The old AI strike driving the arm the weapon owns: refused.
        EXPECT_FALSE(composer.writeBody(Area::Legacy, 9, Transform{}));
        // The weapon writing a leg: refused.
        EXPECT_FALSE(composer.writeBody(Area::Weapon, 6, Transform{}));
        // The old code still owns the legs.
        EXPECT_TRUE(composer.writeBody(Area::Legacy, 6, { JPH::Vec3(-12, 3, 70), JPH::Quat::sIdentity() }));
        // A body with no channel (the weapon) is not a pose channel.
        EXPECT_FALSE(composer.writeBody(Area::Weapon, 10, Transform{}));
        EXPECT_FALSE(composer.writeRoot(Area::Weapon, JPH::Vec3::sZero()));
        EXPECT_TRUE(composer.writeRoot(Area::Legacy, JPH::Vec3(1, 0, 0)));

        const RagdollPose result = composer.compose();
        EXPECT_TRUE(result.mBodyTransforms[9].mTranslation.IsClose(hand.mTranslation));
        EXPECT_TRUE(result.mBodyTransforms[6].mTranslation.IsClose(JPH::Vec3(-12, 3, 70)));
        const auto& rejected = composer.rejections().mCount;
        EXPECT_EQ(rejected[static_cast<std::size_t>(Area::Legacy)][static_cast<std::size_t>(Channel::RightArm)], 1u);
        EXPECT_EQ(rejected[static_cast<std::size_t>(Area::Weapon)][static_cast<std::size_t>(Channel::LeftLeg)], 1u);
        EXPECT_EQ(rejected[static_cast<std::size_t>(Area::Weapon)][static_cast<std::size_t>(Channel::Root)], 1u);
        EXPECT_EQ(composer.rejections().total(), 3u);
        composer.clearRejections();
        EXPECT_EQ(composer.rejections().total(), 0u);
    }

    TEST(PipelineComposerTest, TurnsTheTrunkOverItsTilt)
    {
        const std::vector<std::string_view> bodyNames = names();
        const ChannelMap map = channelMapFromNames(bodyNames);
        const Ownership owners = resolveOwnership(input(BodyState::Standing, true));
        StepComposer composer;
        composer.begin(pose(), map, owners);
        // The upright holder leans the chest forward; the weapon turns it.
        const Transform leaned{ JPH::Vec3(0, 4, 120), JPH::Quat::sRotation(JPH::Vec3::sAxisX(), 0.2f) };
        ASSERT_TRUE(composer.writeBody(Area::Upright, 3, leaned));
        EXPECT_FALSE(composer.writeTrunkTurn(Area::Upright, 1.0f));
        const float turn = 0.5f * JPH::JPH_PI;
        ASSERT_TRUE(composer.writeTrunkTurn(Area::Weapon, turn));
        const RagdollPose result = composer.compose();

        // spine2 (share 1): its lean point (0, 4) turned a quarter about the
        // pelvis's vertical goes to (-4, 0), its height kept, the turn
        // applied over the lean.
        const Transform& chest = result.mBodyTransforms[3];
        EXPECT_TRUE(chest.mTranslation.IsClose(JPH::Vec3(-4, 0, 120), 1.0e-6f));
        const JPH::Quat expected = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), turn) * leaned.mRotation;
        EXPECT_TRUE(chest.mRotation.IsClose(expected, 1.0e-6f));
        // The head rides the chest; the lowest spine turns its share.
        EXPECT_TRUE(result.mBodyTransforms[5].mTranslation.IsClose(JPH::Vec3(-2, 0, 140), 1.0e-6f));
        const JPH::Quat lowSpine = result.mBodyTransforms[1].mRotation;
        EXPECT_NEAR(2.0f * std::acos(std::abs(lowSpine.GetW())), 0.3f * turn, 1.0e-4f);
        // Pelvis, legs and arms are not turned.
        EXPECT_TRUE(result.mBodyTransforms[0].mRotation.IsClose(JPH::Quat::sIdentity()));
        EXPECT_TRUE(result.mBodyTransforms[6].mTranslation.IsClose(JPH::Vec3(-10, 0, 70)));
        EXPECT_TRUE(result.mBodyTransforms[9].mTranslation.IsClose(JPH::Vec3(30, 10, 100)));
    }
}
