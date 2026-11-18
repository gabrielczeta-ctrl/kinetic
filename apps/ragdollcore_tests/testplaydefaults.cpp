#include <ragdollcore/playdefaults.hpp>

#include <gtest/gtest.h>

#include <string_view>

namespace
{
    using namespace RagdollCore;

    TEST(PlayDefaultsTest, NamesAreRagdollSwitchesSortedAndUnique)
    {
        const auto table = playDefaults();
        ASSERT_FALSE(table.empty());
        for (std::size_t i = 0; i < table.size(); ++i)
        {
            EXPECT_TRUE(table[i].mName.starts_with("OPENMW_RAGDOLL_")) << table[i].mName;
            EXPECT_FALSE(table[i].mValue.empty()) << table[i].mName;
            if (i > 0)
                EXPECT_LT(table[i - 1].mName, table[i].mName) << "not sorted or a duplicate";
        }
    }

    TEST(PlayDefaultsTest, HoldsNoPathsOrTraces)
    {
        // Paths and traces are the launcher's business, not the game's.
        for (const PlayDefault& entry : playDefaults())
        {
            EXPECT_FALSE(entry.mName.ends_with("_TRACE")) << entry.mName;
            EXPECT_NE(entry.mName, std::string_view("OPENMW_RAGDOLL_SETTINGS"));
            EXPECT_NE(entry.mName, std::string_view("OPENMW_RAGDOLL_ACTIVE_BODY"));
            EXPECT_NE(entry.mName, std::string_view("OPENMW_RAGDOLL_PLAY_DEFAULTS"));
        }
    }
}
