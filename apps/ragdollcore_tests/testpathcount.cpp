#include <gtest/gtest.h>

#include <ragdollcore/pathcount.hpp>

using namespace RagdollCore;

TEST(RagdollCorePathCountTest, CountsPlayerAndNpcApartAndListsNonzeroPaths)
{
    const std::string before = pathCountReport();
    countPath(CountedPath::Steps, true);
    countPath(CountedPath::TipControl, true);
    countPath(CountedPath::BracingPgc, false);
    countPath(CountedPath::Count, true); // out of range: ignored
    const std::string after = pathCountReport();
    EXPECT_NE(before, after);
    const auto split = after.find(" | npc");
    ASSERT_NE(split, std::string::npos);
    EXPECT_NE(after.substr(0, split).find(" tip="), std::string::npos);
    EXPECT_NE(after.substr(split).find(" brace_pgc="), std::string::npos);
    EXPECT_NE(after.substr(split).find(" steps="), std::string::npos);
}
