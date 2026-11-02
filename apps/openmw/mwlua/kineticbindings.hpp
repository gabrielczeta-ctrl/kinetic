#ifndef MWLUA_KINETICBINDINGS_H
#define MWLUA_KINETICBINDINGS_H

#include <filesystem>
#include <sol/forward.hpp>

namespace MWLua
{
    struct Context;

    // KINETIC switches (Beta settings menu): a few whitelisted
    // OPENMW_RAGDOLL_* switches the in-game options page writes to a file,
    // applied at the next start as environment values (an explicit
    // environment variable still wins). The file is OPENMW_KINETIC_SWITCHES,
    // else kinetic-switches.cfg in the user data folder.
    void setKineticSwitchesDefaultPath(const std::filesystem::path& path);
    std::filesystem::path kineticSwitchesPath();

    // Live tuning (the KINETIC page's sliders): overrides of curated
    // switches that apply on the next physics step and are saved to
    // kinetic-tuning.cfg beside the switches file. Unlike the switches they
    // win over the launcher's environment, at once and at the next start.
    std::filesystem::path kineticTuningPath();
    // Queues the saved overrides (call once at start, before the first step).
    void loadKineticTuning();

    // openmw.kinetic for player scripts: switches(), setSwitch(key, value or
    // nil), tunables(), setLive(env, value or nil), and the actor queries.
    sol::table initKineticPackage(const Context& context);

    // openmw.kinetic for actor scripts (NPC, CREATURE): the actor queries
    // only, API_VERSION and isPowered(actor). Compatibility scripts for other
    // combat mods use them (docs/ragdoll/designs/COMPAT-01-combat-mods.md).
    sol::table initKineticLocalPackage(const Context& context);
    void addKineticActorQueries(sol::table& api);
}

#endif
