#include "../../openmw/mwmechanics/ragdollattackstate.hpp"
#include <gtest/gtest.h>
#include <array>
#include <limits>

namespace
{
    using MWMechanics::RagdollAttackPhase;
    constexpr std::array phases{RagdollAttackPhase::Idle, RagdollAttackPhase::WindUp,
        RagdollAttackPhase::Release, RagdollAttackPhase::FollowThrough};

    TEST(NativeAttackContactTest, ownedWindowRequiresCurrentActiveSnapshot)
    {
        for (auto phase : phases)
            for (bool active : {false, true})
                for (std::uint64_t captured : {40ull, 41ull, 42ull})
                {
                    const bool eligible = phase == RagdollAttackPhase::Release
                        || phase == RagdollAttackPhase::FollowThrough;
                    EXPECT_EQ(MWMechanics::permitsRagdollAttackContact(true, phase, active, captured, 41),
                        eligible && active && captured == 41);
                }
    }
    TEST(NativeAttackContactTest, transitionToIdleAndLaterSwingCannotReviveOldContact)
    {
        EXPECT_TRUE(MWMechanics::permitsRagdollAttackContact(true, RagdollAttackPhase::FollowThrough,true,7,7));
        EXPECT_FALSE(MWMechanics::permitsRagdollAttackContact(true, RagdollAttackPhase::Idle,true,7,7));
        EXPECT_FALSE(MWMechanics::permitsRagdollAttackContact(true, RagdollAttackPhase::Release,true,7,8));
        EXPECT_FALSE(MWMechanics::permitsRagdollAttackContact(true, RagdollAttackPhase::FollowThrough,true,7,8));
        constexpr std::uint64_t id = (1ull << 53) + 1;
        EXPECT_FALSE(MWMechanics::permitsRagdollAttackContact(true, RagdollAttackPhase::Release,true,id,id+1));
        EXPECT_TRUE(MWMechanics::permitsRagdollAttackContact(true, RagdollAttackPhase::Release,true,id,id));
    }
    TEST(NativeAttackContactTest, legacyGesturesRetainTheirExistingDownstreamAuthority)
    {
        for (auto phase : phases)
            for (bool active : {false, true})
                EXPECT_TRUE(MWMechanics::permitsRagdollAttackContact(false, phase, active, 7, 8));
    }
}
