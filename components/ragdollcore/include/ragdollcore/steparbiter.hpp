#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_STEPARBITER_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_STEPARBITER_HPP

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

namespace RagdollCore
{
    // Who commands the legs' next step (STEPPING audit 2026-11-01, item 13).
    // Several balance behaviours may each want to step a foot; before this,
    // the first in the engine's code order took it and nothing said which.
    // The arbiter holds the declared priority of the balance catches, the
    // owner of the current step and why, so the engine runs them in that order
    // and logs each change of owner. Footfall's own steps (the gait clock,
    // overrun, settle) are the default when no behaviour claims a step; their
    // reasons are in its own trace (RAGDOLL_FOOTFALL_LIFT/_SETTLE).
    //
    // Pure: no Jolt or engine types.
    enum class StepSource : std::size_t
    {
        GaveUp, // a stagger gave up: no step, the body falls (always first)
        Stagger, // a stagger plan's rushed steps after a blow (always second)
        Dodge, // a dodge's committed step (always third)
        Stuck, // a still body stuck leaning steps under itself
        Landing, // a landing runs out its carried speed
        MovingCatch, // walking or running, a catch beside the gait
        StandingCatch, // standing, the capture-point catch
        StageCatch, // after a hit, the stage-only catch
        Count
    };

    const char* stepSourceName(StepSource source);

    class StepArbiter
    {
    public:
        static constexpr std::size_t sCatches = 5; // Stuck .. StageCatch
        using Order = std::array<StepSource, sCatches>;

        // The engine's code order of 2026-11-01.
        static Order defaultOrder();
        // "landing,stuck,moving,standing,stage": every catch exactly once
        // (names as stepSourceName without the prefix); nullopt otherwise.
        static std::optional<Order> parseOrder(std::string_view text);

        explicit StepArbiter(Order order = defaultOrder())
            : mOrder(order)
        {
        }

        const Order& order() const { return mOrder; }

        // This step belongs to `source` (it commanded a step, held the clock
        // or released the body). True when the owner changed.
        bool claim(StepSource source, std::string_view reason);
        // Nobody claimed this step: Footfall's own gait. True when it changed.
        bool idle();

        std::optional<StepSource> owner() const { return mOwner; }
        const std::string& reason() const { return mReason; }
        // Steps each source has owned (diagnostics).
        unsigned steps(StepSource source) const { return mSteps[static_cast<std::size_t>(source)]; }

    private:
        Order mOrder;
        std::optional<StepSource> mOwner;
        std::string mReason;
        std::array<unsigned, static_cast<std::size_t>(StepSource::Count)> mSteps{};
    };
}

#endif
