#include <ragdollcore/tuning.hpp>

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

namespace
{
    using namespace RagdollCore;

    struct TuningListTest : ::testing::Test
    {
        std::vector<std::string> mWarnings;
        void SetUp() override
        {
            setTuningWarningSink([this](const std::string& message) { mWarnings.push_back(message); });
        }
        void TearDown() override
        {
            setTuningWarningSink({});
        }
        static void set(const char* name, const char* value)
        {
            setTuningOverride(name, std::string(value));
            refreshTuning();
        }
    };

    TEST_F(TuningListTest, readsEveryFieldWithoutAWarning)
    {
        set("OPENMW_RAGDOLL_TEST_LIST_FULL", "1.5,2,-3,4e1");
        std::array<float, 4> v{ 0.0f, 0.0f, 0.0f, 0.0f };
        EXPECT_EQ(tuningList("OPENMW_RAGDOLL_TEST_LIST_FULL", v), 4u);
        EXPECT_EQ(v, (std::array<float, 4>{ 1.5f, 2.0f, -3.0f, 40.0f }));
        EXPECT_TRUE(mWarnings.empty());
    }

    TEST_F(TuningListTest, aShortValueKeepsTheDefaultsAndWarnsOnce)
    {
        set("OPENMW_RAGDOLL_TEST_LIST_SHORT", "7,8,9");
        std::array<float, 4> v{ 1.0f, 2.0f, 3.0f, 4.0f };
        EXPECT_EQ(tuningList("OPENMW_RAGDOLL_TEST_LIST_SHORT", v), 3u);
        EXPECT_EQ(v, (std::array<float, 4>{ 7.0f, 8.0f, 9.0f, 4.0f }));
        tuningList("OPENMW_RAGDOLL_TEST_LIST_SHORT", v);
        ASSERT_EQ(mWarnings.size(), 1u);
        EXPECT_NE(mWarnings[0].find("OPENMW_RAGDOLL_TEST_LIST_SHORT has 3 fields, expected 4"), std::string::npos);
    }

    TEST_F(TuningListTest, extraFieldsAreIgnoredWithAWarning)
    {
        set("OPENMW_RAGDOLL_TEST_LIST_LONG", "1,2,3");
        std::array<float, 2> v{ 0.0f, 0.0f };
        EXPECT_EQ(tuningList("OPENMW_RAGDOLL_TEST_LIST_LONG", v), 3u);
        EXPECT_EQ(v, (std::array<float, 2>{ 1.0f, 2.0f }));
        ASSERT_EQ(mWarnings.size(), 1u);
        EXPECT_NE(mWarnings[0].find("extra fields are ignored"), std::string::npos);
    }

    TEST_F(TuningListTest, anUnsetOrUnreadableFieldKeepsItsDefault)
    {
        std::array<float, 2> v{ 5.0f, 6.0f };
        EXPECT_EQ(tuningList("OPENMW_RAGDOLL_TEST_LIST_UNSET", v), 0u);
        EXPECT_EQ(v, (std::array<float, 2>{ 5.0f, 6.0f }));
        set("OPENMW_RAGDOLL_TEST_LIST_HOLE", "x,9");
        EXPECT_EQ(tuningList("OPENMW_RAGDOLL_TEST_LIST_HOLE", v), 2u);
        EXPECT_EQ(v, (std::array<float, 2>{ 5.0f, 9.0f }));
        EXPECT_TRUE(mWarnings.empty());
    }
}
