#include <ragdollcore/switchregistry.hpp>
#include <ragdollcore/tuning.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

namespace
{
    using namespace RagdollCore;

    TEST(SwitchRegistryTest, IsSound)
    {
        for (const std::string& problem : checkSwitchRegistry())
            ADD_FAILURE() << problem;
    }

    TEST(SwitchRegistryTest, FindsRegisteredNamesOnly)
    {
        for (const SwitchSpec& spec : switchRegistry())
        {
            const SwitchSpec* found = findSwitch(spec.mName);
            ASSERT_NE(found, nullptr) << spec.mName;
            EXPECT_EQ(found->mName, spec.mName);
        }
        EXPECT_EQ(findSwitch("OPENMW_RAGDOLL_NOT_A_SWITCH"), nullptr);
        EXPECT_EQ(findSwitch(""), nullptr);
    }

    TEST(SwitchRegistryTest, UnsetSwitchReadsItsRegisteredDefault)
    {
        // The code's default is the game: nothing set, tuningText gives the
        // registered value; the environment and an override still win.
        const SwitchSpec* spec = findSwitch("OPENMW_RAGDOLL_K2_LOCKOUT");
        ASSERT_NE(spec, nullptr);
        ASSERT_EQ(std::getenv("OPENMW_RAGDOLL_K2_LOCKOUT"), nullptr) << "unset it to run this test";
        const char* text = tuningText("OPENMW_RAGDOLL_K2_LOCKOUT");
        ASSERT_NE(text, nullptr);
        EXPECT_EQ(std::string(text), std::string(spec->mDefault));

        setTuningOverride("OPENMW_RAGDOLL_K2_LOCKOUT", std::string("1"));
        refreshTuning();
        EXPECT_EQ(std::string(tuningText("OPENMW_RAGDOLL_K2_LOCKOUT")), "1");
        setTuningOverride("OPENMW_RAGDOLL_K2_LOCKOUT", std::nullopt);
        refreshTuning();
        EXPECT_EQ(std::string(tuningText("OPENMW_RAGDOLL_K2_LOCKOUT")), std::string(spec->mDefault));
    }

    TEST(SwitchRegistryTest, UnregisteredUnsetSwitchStaysNull)
    {
        EXPECT_EQ(tuningText("OPENMW_RAGDOLL_NOT_A_SWITCH"), nullptr);
    }
}
