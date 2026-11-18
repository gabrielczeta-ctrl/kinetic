#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_SWITCHREGISTRY_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_SWITCHREGISTRY_HPP

#include <cstddef>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace RagdollCore
{
    // KINETIC-2's switch registry (PLAN.md "KINETIC-2", phase 0). Every
    // switch the rewritten code reads is declared here once: what kind it
    // is, which area owns it, the value the game plays with, the range of
    // each field, why it exists and, for an experiment, by when it is
    // promoted or deleted. tuningText() falls back to the registered default
    // when nothing else sets a switch, so the code's default is the game:
    // the scripted tests and a fresh clone play what the player plays.
    //
    // The old integration's switches are not here; they go with their area
    // (tools/body/switch_census.py counts what is left). A switch is not
    // both here and in playDefaults().
    //
    // Pure: no Jolt or engine types.
    enum class SwitchKind : unsigned char
    {
        Tunable, // a real dial, set by feel; counts against the budget
        Experiment, // a new path behind a switch until the user's verdict
        Debug, // a trace or a diagnostic; never changes behaviour
    };

    enum class SwitchArea : unsigned char
    {
        Pipeline,
        Weapon,
        Legs,
        Upright,
        Reaction,
        Root,
        Lifecycle,
        Ai,
        Hud,
    };

    struct SwitchSpec
    {
        std::string_view mName;
        SwitchKind mKind = SwitchKind::Tunable;
        SwitchArea mArea = SwitchArea::Pipeline;
        // The value the game plays with ("a,b,c" for a list); empty = unset.
        std::string_view mDefault;
        // Every field of a set value lies in [mMin, mMax].
        float mMin = 0.0f;
        float mMax = 1.0f;
        std::string_view mReason;
        // Experiments: the phase or date by which it is promoted to a
        // tunable or a constant, or deleted with its losing path.
        std::string_view mExpires;
    };

    // About 50 real tunables in the finished game (PLAN.md rules).
    inline constexpr std::size_t sTunableBudget = 50;

    // Sorted by name.
    std::span<const SwitchSpec> switchRegistry();

    // nullptr when `name` is not registered.
    const SwitchSpec* findSwitch(std::string_view name);

    // Every rule the registry breaks (empty = sound): unsorted or duplicate
    // names, a missing reason, an experiment with no expiry, a default whose
    // fields are not numbers inside the range, more tunables than the
    // budget, a name also in playDefaults().
    std::vector<std::string> checkSwitchRegistry();

    std::string_view switchAreaName(SwitchArea area);
}

#endif
