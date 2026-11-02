#include <gtest/gtest.h>
#include <string_view>

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
// clang-format on

int runFeedbackPilot(int argc, char** argv);

int main(int argc, char** argv)
{
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory;
    JPH::RegisterTypes();
    int result;
    if (argc >= 2 && std::string_view(argv[1]) == "--feedback-pilot")
        result = runFeedbackPilot(argc - 1, argv + 1);
    else
    {
        testing::InitGoogleTest(&argc, argv);
        result = RUN_ALL_TESTS();
    }
    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
    return result;
}
