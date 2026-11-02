#ifndef OPENMW_FPSFIXTURE_HPP
#define OPENMW_FPSFIXTURE_HPP

#ifdef OPENMW_RAGDOLL_DEV_API
#include <charconv>
#include <ragdollcore/tuning.hpp>
#include <cstdlib>
#include <stdexcept>
#include <string_view>

namespace OMW::FpsFixture
{
    // Test-only, main-thread clock. All arms create the first powered actor
    // during the same 60 Hz prelude. An optional post-power prelude isolates
    // later sampling changes while preserving the full earlier history.
    inline bool armed = false;
    inline unsigned armedFrames = 0;
    inline bool cadenceSelected = false;

    inline unsigned postArmPreludeFrames()
    {
        static const auto frames_tuned = RagdollCore::tuned<unsigned>([] {
            const char* value = RagdollCore::tuningText("OPENMW_RAGDOLL_TEST_FPS_PRELUDE_FRAMES");
            if (!value) return 0u;
            const std::string_view text(value);
            unsigned result = 0;
            const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
            if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size())
                throw std::runtime_error("OPENMW_RAGDOLL_TEST_FPS_PRELUDE_FRAMES must be a nonnegative integer");
            return result;
        });
        static const auto& frames = frames_tuned.get();
        return frames;
    }

    inline int requestedRate()
    {
        static const auto rate_tuned = RagdollCore::tuned<int>([] {
            const char* value = RagdollCore::tuningText("OPENMW_RAGDOLL_TEST_RENDER_FPS");
            if (!value)
                return 0;
            const std::string_view text(value);
            if (text == "30") return 30;
            if (text == "60") return 60;
            if (text == "120") return 120;
            throw std::runtime_error("OPENMW_RAGDOLL_TEST_RENDER_FPS must be 30, 60 or 120");
        });
        static const auto& rate = rate_tuned.get();
        return rate;
    }

    inline int currentRate()
    {
        if (!armed) return 60;
        if (armedFrames < postArmPreludeFrames())
        {
            ++armedFrames;
            return 60;
        }
        cadenceSelected = true;
        return requestedRate();
    }
}
#endif
#endif
