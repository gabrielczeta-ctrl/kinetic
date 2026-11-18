#include <ragdollcore/pathcount.hpp>

#include <array>
#include <atomic>
#include <cstddef>
#include <sstream>

namespace RagdollCore
{
    namespace
    {
        constexpr std::size_t sPathCount = static_cast<std::size_t>(CountedPath::Count);

        constexpr std::array<const char*, sPathCount> sNames{ "steps", "tip", "brace_pgc", "gait_clock",
            "ai_strike_over_hsc", "leg_travel_yaw_wrote", "leg_travel_yaw_kept", "run_back_kick_wrote",
            "run_back_kick_kept", "foot_grounding_wrote", "foot_grounding_kept", "swing_clearance_wrote",
            "swing_clearance_kept" };

        std::array<std::array<std::atomic<std::uint64_t>, sPathCount>, 2> sCounts{};
    }

    void countPath(CountedPath path, bool player)
    {
        const auto index = static_cast<std::size_t>(path);
        if (index < sPathCount)
            sCounts[player ? 0 : 1][index].fetch_add(1, std::memory_order_relaxed);
    }

    std::string pathCountReport()
    {
        std::ostringstream out;
        for (std::size_t who = 0; who < 2; ++who)
        {
            out << (who == 0 ? "player" : " | npc");
            for (std::size_t index = 0; index < sPathCount; ++index)
            {
                const std::uint64_t count = sCounts[who][index].load(std::memory_order_relaxed);
                if (count > 0 || index == 0)
                    out << ' ' << sNames[index] << '=' << count;
            }
        }
        return out.str();
    }
}
