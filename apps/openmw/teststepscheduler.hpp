#ifndef OPENMW_TESTSTEPSCHEDULER_HPP
#define OPENMW_TESTSTEPSCHEDULER_HPP
#ifdef OPENMW_RAGDOLL_DEV_API
#include <atomic>
#include <ragdollcore/tuning.hpp>
#include <charconv>
#include <cstdlib>
#include <stdexcept>
#include <string_view>

namespace OMW::TestStepScheduler
{
    // Zero distinguishes an unset switch from the explicit k=1 control.
    inline unsigned requested()
    {
        static const auto value_tuned = RagdollCore::tuned<unsigned>([] {
            const char* env = RagdollCore::tuningText("OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER");
            if (!env) return 0u;
            const std::string_view text(env);
            unsigned result = 0;
            const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
            if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || result < 1 || result > 16)
                throw std::runtime_error("OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER must be an integer from 1 to 16");
            return result;
        });
        static const auto& value = value_tuned.get();
        return value;
    }
    inline std::atomic<bool> active{ false };
    inline std::atomic<unsigned> replaySeed{ 0 };
    // Engine-thread state, snapshotted before the Lua worker starts the tick.
    inline bool drawFrame = true;
}
#endif
#endif
