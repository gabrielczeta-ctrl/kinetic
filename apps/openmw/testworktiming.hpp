#ifndef OPENMW_TESTWORKTIMING_HPP
#define OPENMW_TESTWORKTIMING_HPP

#ifdef OPENMW_RAGDOLL_DEV_API
#include <chrono>
#include <ragdollcore/tuning.hpp>
#include <cstdlib>
#include <iomanip>
#include <stdexcept>
#include <components/debug/debuglog.hpp>
#ifdef _WIN32
#include <components/misc/windows.hpp>
#else
#include <time.h>
#endif

namespace OMW::TestWorkTiming
{
    inline bool enabled()
    {
        static const auto value_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_TEST_WORK_TIMING") != nullptr; });
        static const auto& value = value_tuned.get();
        return value;
    }

    struct Reading { double wall = 0, process = 0, thread = 0; };
    inline Reading read()
    {
        Reading result;
        result.wall = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
#ifdef _WIN32
        const auto cpu = [](HANDLE handle, bool process) {
            FILETIME created, exited, kernel, user;
            const BOOL ok = process ? GetProcessTimes(handle, &created, &exited, &kernel, &user)
                                    : GetThreadTimes(handle, &created, &exited, &kernel, &user);
            if (!ok) throw std::runtime_error("native work timing: CPU clock failed");
            const auto ticks = [](FILETIME value) {
                return (static_cast<unsigned long long>(value.dwHighDateTime) << 32) | value.dwLowDateTime;
            };
            return static_cast<double>(ticks(kernel) + ticks(user)) / 10000.0;
        };
        result.process = cpu(GetCurrentProcess(), true);
        result.thread = cpu(GetCurrentThread(), false);
#else
        const auto cpu = [](clockid_t clock) {
            timespec value;
            if (clock_gettime(clock, &value)) throw std::runtime_error("native work timing: CPU clock failed");
            return value.tv_sec * 1000.0 + value.tv_nsec / 1000000.0;
        };
        result.process = cpu(CLOCK_PROCESS_CPUTIME_ID);
        result.thread = cpu(CLOCK_THREAD_CPUTIME_ID);
#endif
        return result;
    }

    inline Reading operator-(Reading a, Reading b)
    { return {a.wall - b.wall, a.process - b.process, a.thread - b.thread}; }

    inline Reading physics, render, limiter;
    // Scopes run on the engine thread. Process CPU includes all concurrent
    // workers during the interval; it is not exclusive subsystem attribution.
    struct Scope
    {
        Reading& target;
        Reading start;
        explicit Scope(Reading& value) : target(value) { if (enabled()) start = read(); }
        ~Scope() { if (enabled()) target = read() - start; }
    };

    inline void emit(unsigned frame, Reading total)
    {
        Log(Debug::Info) << std::dec << std::setprecision(12) << "RAGDOLL_FRAME_WORK schema=1 frame=" << frame
            << " wall=" << total.wall << " process=" << total.process << " thread=" << total.thread
            << " physics_wall=" << physics.wall << " physics_process=" << physics.process
            << " physics_thread=" << physics.thread << " render_wall=" << render.wall
            << " render_process=" << render.process << " render_thread=" << render.thread
            << " limiter_wall=" << limiter.wall << " limiter_process=" << limiter.process
            << " limiter_thread=" << limiter.thread;
    }
}
#endif
#endif
