#include "kineticbindings.hpp"
#include <ragdollcore/tuning.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include <string_view>

#include <components/debug/debuglog.hpp>
#include <components/lua/luastate.hpp>

#include "apps/openmw/mwbase/environment.hpp"
#include "apps/openmw/mwbase/world.hpp"
#include "apps/openmw/mwphysics/physicssystem.hpp"
#include "apps/openmw/mwworld/worldimp.hpp"

#include "context.hpp"
#include "object.hpp"

namespace MWLua
{
    namespace
    {
        std::filesystem::path sDefaultPath;

        // The switches the options page may write. Anything else is refused.
        constexpr std::array<std::string_view, 9> sAllowed{
            "OPENMW_RAGDOLL_RMB_GUARD",
            "OPENMW_RAGDOLL_FALL_PROTECT",
            "OPENMW_RAGDOLL_PGC",
            "OPENMW_RAGDOLL_HEAD_DAZE",
            "OPENMW_RAGDOLL_REACTION_V2_VIS",
            "OPENMW_RAGDOLL_PARRY_KNOCK",
            "OPENMW_RAGDOLL_TIP_CHAMBER",
            "OPENMW_RAGDOLL_SPARKS",
            "OPENMW_RAGDOLL_CAMERA_KICK",
        };

        bool allowed(std::string_view key)
        {
            return std::find(sAllowed.begin(), sAllowed.end(), key) != sAllowed.end();
        }

        bool safeValue(std::string_view value);

        // Live tuning: the switches the page shows as sliders. Each component
        // is one slider; `fallback` is the code's default when the switch is
        // unset (the launcher may set another value, which the page shows).
        // Only switches every reader takes live are listed (Tuned values, or
        // read per use; per-actor copies refresh on the tuning generation).
        struct TunableComponent
        {
            const char* mLabel;
            float mFallback, mMin, mMax, mStep;
        };
        struct Tunable
        {
            const char* mEnv;
            const char* mGroup;
            const char* mName;
            const char* mDescription;
            std::vector<TunableComponent> mComponents;
        };
        const std::vector<Tunable>& tunables()
        {
            static const std::vector<Tunable> list{
                { "OPENMW_RAGDOLL_REACTION_V2_VIS", "Hit reactions", "Reaction size",
                    "Sizes, snaps and push of every hit reaction (1 = the measured base).",
                    { { "k", 1.0f, 0.5f, 3.0f, 0.1f } } },
                { "OPENMW_RAGDOLL_REACTION_V2_PUSH", "Hit reactions", "Hit push",
                    "Push = scale x the blow's momentum, capped at a body-part speed (u/s).",
                    { { "scale", 1.5f, 0.0f, 4.0f, 0.1f }, { "part speed cap", 560.0f, 100.0f, 1500.0f, 10.0f } } },
                { "OPENMW_RAGDOLL_REACTION_V2_COVER_HOLD", "Hit reactions", "Head cover hold",
                    "Seconds the arms stay up over the face after a head blow.",
                    { { "seconds", 0.35f, 0.05f, 3.0f, 0.05f } } },
                { "OPENMW_RAGDOLL_BEARINGS_STUMBLE", "Hit reactions", "Stagger stumble",
                    "Share of the balance helper's sideways hold a stagger takes away (more = bigger stumbles).",
                    { { "share", 0.0f, 0.0f, 0.95f, 0.05f } } },
                { "OPENMW_RAGDOLL_HEAD_DAZE", "Hit reactions", "Head daze",
                    "Daze seconds per 2500 impulse, longest daze, knee sink (units), balance kept.",
                    { { "s per impulse", 1.2f, 0.0f, 4.0f, 0.1f }, { "max s", 1.8f, 0.0f, 5.0f, 0.1f },
                        { "sink", 12.0f, 0.0f, 40.0f, 1.0f }, { "balance", 1.0f, 0.0f, 1.0f, 0.05f } } },
                { "OPENMW_RAGDOLL_CAMERA_KICK", "Hit reactions", "Camera kick",
                    "Degrees the view jolts when you are hit (0 = off).",
                    { { "degrees", 0.0f, 0.0f, 8.0f, 0.5f } } },
                { "OPENMW_RAGDOLL_FP_DISSOLVE", "First person", "Body dissolve",
                    "Your own body fades near the eye: gone within, solid beyond (units), extra toward the centre.",
                    { { "gone", 10.0f, 0.0f, 40.0f, 1.0f }, { "solid", 22.0f, 1.0f, 60.0f, 1.0f },
                        { "centre", 0.8f, 0.0f, 2.0f, 0.1f } } },
                { "OPENMW_RAGDOLL_FP_DOWN_FORWARD", "First person", "Look-down eye shift",
                    "Units the eye slides forward over the chest when you look all the way down (0 = off).",
                    { { "units", 8.0f, 0.0f, 30.0f, 1.0f } } },
                { "OPENMW_RAGDOLL_FP_CLEAR", "First person", "Guard turned clear",
                    "Degrees the guard blade turns right out of the first-person view.",
                    { { "degrees", 0.0f, 0.0f, 30.0f, 1.0f } } },
                { "OPENMW_RAGDOLL_HSC_STANCE_RADIUS", "Your weapon", "Guard circle",
                    "Mouse counts you can move inside the guard before a drag winds up.",
                    { { "counts", 300.0f, 100.0f, 800.0f, 10.0f } } },
                { "OPENMW_RAGDOLL_HSC_CUT_SPEED", "Your weapon", "Cut flick speed",
                    "Inward mouse speed (counts/s) that starts a cut.",
                    { { "counts/s", 2500.0f, 500.0f, 6000.0f, 100.0f } } },
                { "OPENMW_RAGDOLL_HSC_STEER", "Your weapon", "Cut steering",
                    "Mouse counts per radian a cut bends, and the most it bends (radians; 0 = off).",
                    { { "counts/rad", 0.0f, 0.0f, 4000.0f, 100.0f }, { "max rad", 0.5f, 0.0f, 1.5f, 0.05f } } },
                { "OPENMW_RAGDOLL_HSC_AIM_REACH", "Your weapon", "Aim reach",
                    "Units the hand reaches toward the crosshair point.",
                    { { "units", 0.0f, 0.0f, 80.0f, 1.0f } } },
                { "OPENMW_RAGDOLL_FIST_POWER", "Your weapon", "Punch from the body",
                    "Trunk turn into a punch (degrees) and weight forward (units).",
                    { { "twist", 0.0f, 0.0f, 45.0f, 1.0f }, { "forward", 0.0f, 0.0f, 20.0f, 1.0f } } },
                { "OPENMW_RAGDOLL_CLASH_ARM", "Clashes", "Strength in clashes",
                    "How much Strength decides a clash (1 = fully, 0.5 = its square root).",
                    { { "weight", 1.0f, 0.0f, 2.0f, 0.05f } } },
                { "OPENMW_RAGDOLL_WEAPON_TORQUE", "Clashes", "Off-centre blade turn",
                    "Share of a blow's turn a struck blade keeps, and its recoil seconds.",
                    { { "share", 0.0f, 0.0f, 0.95f, 0.05f }, { "recoil s", 0.25f, 0.02f, 1.0f, 0.01f } } },
                { "OPENMW_RAGDOLL_WEAPON_KNOCK", "Clashes", "Weapon knock",
                    "A struck weapon's drive eases: seconds at full, drive left.",
                    { { "seconds", 0.0f, 0.0f, 1.5f, 0.05f }, { "drive left", 0.15f, 0.0f, 1.0f, 0.05f } } },
                { "OPENMW_RAGDOLL_PARRY_KNOCK", "Clashes", "Parry knock",
                    "Units a parried blade is thrown, recovery seconds at no skill, reference kick.",
                    { { "units", 35.0f, 0.0f, 100.0f, 1.0f }, { "recovery s", 0.7f, 0.0f, 3.0f, 0.05f },
                        { "reference", 300.0f, 50.0f, 1000.0f, 10.0f } } },
                { "OPENMW_RAGDOLL_FEINT", "Clashes", "Feints",
                    "Fatigue a pulled-back cut costs, and an NPC's feint chance at skill 100.",
                    { { "fatigue", 0.0f, 0.0f, 20.0f, 0.5f }, { "npc chance", 0.0f, 0.0f, 1.0f, 0.05f } } },
                { "OPENMW_RAGDOLL_GUARD_COVER", "Clashes", "Guard cover radius",
                    "Units around the defender's weapon, shield or hands a hit counts as guarded (0 = off).",
                    { { "units", 0.0f, 0.0f, 40.0f, 1.0f } } },
                { "OPENMW_RAGDOLL_DUEL_COMPOSED", "NPC fighters", "Patience",
                    "NPC patience before attacking (x), back-off chance at skill 0 and at skill 100.",
                    { { "patience", 1.0f, 0.2f, 3.0f, 0.05f }, { "back-off unskilled", 0.0f, 0.0f, 1.0f, 0.05f },
                        { "back-off skilled", 0.0f, 0.0f, 1.0f, 0.05f } } },
                { "OPENMW_RAGDOLL_NPC_STEADY", "NPC fighters", "Steady hands",
                    "Seconds an NPC's weapon settles before it may strike (0 = off).",
                    { { "seconds", 0.0f, 0.0f, 1.0f, 0.05f } } },
            };
            return list;
        }

        const Tunable* findTunable(std::string_view env)
        {
            for (const Tunable& tunable : tunables())
                if (env == tunable.mEnv)
                    return &tunable;
            return nullptr;
        }

        std::map<std::string, std::string> readTuningFile(const std::filesystem::path& path)
        {
            std::map<std::string, std::string> result;
            std::ifstream file(path);
            std::string line;
            while (std::getline(file, line))
            {
                const auto equals = line.find('=');
                if (line.empty() || line[0] == '#' || equals == std::string::npos)
                    continue;
                const auto trim = [](std::string text) {
                    const auto first = text.find_first_not_of(" \t\r");
                    const auto last = text.find_last_not_of(" \t\r");
                    return first == std::string::npos ? std::string() : text.substr(first, last - first + 1);
                };
                const std::string key = trim(line.substr(0, equals));
                const std::string value = trim(line.substr(equals + 1));
                if (findTunable(key) != nullptr && safeValue(value))
                    result[key] = value;
            }
            return result;
        }

        bool safeValue(std::string_view value)
        {
            return value.size() <= 64 && std::all_of(value.begin(), value.end(), [](char c) {
                return (c >= '0' && c <= '9') || c == '.' || c == ',' || c == '-';
            });
        }

        std::map<std::string, std::string> readFile(const std::filesystem::path& path)
        {
            std::map<std::string, std::string> result;
            std::ifstream file(path);
            std::string line;
            while (std::getline(file, line))
            {
                const auto equals = line.find('=');
                if (line.empty() || line[0] == '#' || equals == std::string::npos)
                    continue;
                const auto trim = [](std::string text) {
                    const auto first = text.find_first_not_of(" \t\r");
                    const auto last = text.find_last_not_of(" \t\r");
                    return first == std::string::npos ? std::string() : text.substr(first, last - first + 1);
                };
                const std::string key = trim(line.substr(0, equals));
                const std::string value = trim(line.substr(equals + 1));
                if (allowed(key) && safeValue(value))
                    result[key] = value;
            }
            return result;
        }

        bool writeFile(const std::filesystem::path& path, const std::map<std::string, std::string>& values)
        {
            std::error_code error;
            if (path.has_parent_path())
                std::filesystem::create_directories(path.parent_path(), error);
            std::ofstream file(path, std::ios::trunc);
            if (!file)
                return false;
            file << "# KINETIC switches, written by the in-game options page.\n"
                    "# Applied at the next start; an environment variable of the same name wins.\n";
            for (const auto& [key, value] : values)
                file << key << " = " << value << '\n';
            return static_cast<bool>(file);
        }
    }

    void setKineticSwitchesDefaultPath(const std::filesystem::path& path)
    {
        sDefaultPath = path;
    }

    std::filesystem::path kineticSwitchesPath()
    {
        if (const char* text = RagdollCore::tuningText("OPENMW_KINETIC_SWITCHES"); text && *text)
            return std::filesystem::path(text);
        return sDefaultPath;
    }

    std::filesystem::path kineticTuningPath()
    {
        const std::filesystem::path switches = kineticSwitchesPath();
        if (switches.empty())
            return {};
        return switches.parent_path() / "kinetic-tuning.cfg";
    }

    void loadKineticTuning()
    {
        const auto path = kineticTuningPath();
        if (path.empty())
            return;
        const auto values = readTuningFile(path);
        for (const auto& [key, value] : values)
            RagdollCore::setTuningOverride(key, value);
        if (!values.empty())
            Log(Debug::Info) << "KINETIC live tuning: " << values.size() << " values from " << path;
    }

    sol::table initKineticPackage(const Context& context)
    {
        sol::state_view lua = context.sol();
        sol::table api(lua, sol::create);
        // The brass reticle's feed (HUD-01) in every build: the player's hand
        // control, guard, balance and attack readiness, as
        // kinetic_hud_global.lua reads them from openmw.ragdolltest - which a
        // release engine does not have, so the reticle never showed there
        // (user 10-31, the release test: "I dont have the diamond reticle").
        // nil while the player has no powered body.
        api["hud"] = [lua]() -> sol::object {
            auto* world
                = dynamic_cast<MWWorld::World*>(static_cast<MWBase::World*>(MWBase::Environment::get().getWorld()));
            if (world == nullptr)
                return sol::nil;
            const MWWorld::Ptr player = world->getPlayerPtr();
            if (player.isEmpty())
                return sol::nil;
            const auto state = world->getPhysicsSystem().getPassiveRagdollDebugState(player);
            if (!state)
                return sol::nil;
            sol::table result(lua, sol::create);
            if (state->mHscActive)
            {
                sol::table hud(lua, sol::create);
                hud["stage"] = state->mHscStage;
                hud["x"] = state->mHscStanceX;
                hud["y"] = state->mHscStanceY;
                hud["angle"] = state->mHscAngle;
                hud["line"] = state->mChamberLine;
                if (!state->mHscTrajectory.empty())
                {
                    sol::table trajectory(lua, sol::create);
                    for (std::size_t i = 0; i < state->mHscTrajectory.size(); ++i)
                        trajectory[i + 1] = state->mHscTrajectory[i];
                    hud["trajectory"] = trajectory;
                }
                result["hud"] = hud;
            }
            result["guard"] = state->mGuardActive;
            result["guardZone"] = state->mGuardZone;
            sol::table balance(lua, sol::create);
            balance["x"] = state->mBalanceRight;
            balance["y"] = state->mBalanceForward;
            balance["tip"] = state->mBalanceTip;
            result["balance"] = balance;
            result["ready"] = state->mAttackReady;
            return result;
        };
        // Every allowed switch: the value in effect now (environment, '' if
        // unset) and the value saved for the next start ('' if none).
        api["switches"] = [lua]() {
            sol::table result(lua, sol::create);
            const auto saved = readFile(kineticSwitchesPath());
            for (const std::string_view key : sAllowed)
            {
                sol::table entry(lua, sol::create);
                const std::string name(key);
                const char* current = RagdollCore::tuningText(name.c_str());
                entry["current"] = std::string(current ? current : "");
                const auto found = saved.find(name);
                entry["saved"] = found == saved.end() ? std::string() : found->second;
                result[name] = entry;
            }
            return result;
        };
        // Saves (or with nil/'' removes) one switch for the next start.
        api["setSwitch"] = [](std::string_view key, sol::optional<std::string> value) {
            if (!allowed(key) || (value && !safeValue(*value)))
                return false;
            const auto path = kineticSwitchesPath();
            if (path.empty())
                return false;
            auto values = readFile(path);
            if (value && !value->empty())
                values[std::string(key)] = *value;
            else
                values.erase(std::string(key));
            const bool written = writeFile(path, values);
            Log(Debug::Info) << "KINETIC switch " << key << " = " << (value ? *value : std::string("(default)"))
                             << (written ? "" : " (not written)") << " -> " << path;
            return written;
        };
        // Live tuning: every slider switch with its components, the value in
        // effect ('' = unset: the components' fallbacks) and the value
        // without the page's override (the launcher's, '' if none).
        api["tunables"] = [lua]() {
            sol::table result(lua, sol::create);
            int index = 1;
            for (const Tunable& tunable : tunables())
            {
                sol::table entry(lua, sol::create);
                entry["env"] = std::string(tunable.mEnv);
                entry["group"] = std::string(tunable.mGroup);
                entry["name"] = std::string(tunable.mName);
                entry["description"] = std::string(tunable.mDescription);
                const char* current = RagdollCore::tuningText(tunable.mEnv);
                entry["current"] = std::string(current ? current : "");
                const char* base = std::getenv(tunable.mEnv);
                entry["base"] = std::string(base ? base : "");
                entry["overridden"] = RagdollCore::tuningOverride(tunable.mEnv).has_value();
                sol::table components(lua, sol::create);
                int c = 1;
                for (const TunableComponent& component : tunable.mComponents)
                {
                    sol::table item(lua, sol::create);
                    item["label"] = std::string(component.mLabel);
                    item["fallback"] = component.mFallback;
                    item["min"] = component.mMin;
                    item["max"] = component.mMax;
                    item["step"] = component.mStep;
                    components[c++] = item;
                }
                entry["components"] = components;
                result[index++] = entry;
            }
            return result;
        };
        // Sets (or with nil/'' drops) a slider switch's live value: it applies
        // on the next physics step and is saved for the next start.
        api["setLive"] = [](std::string_view env, sol::optional<std::string> value) {
            if (findTunable(env) == nullptr || (value && !safeValue(*value)))
                return false;
            const bool drop = !value || value->empty();
            RagdollCore::setTuningOverride(env, drop ? std::nullopt : std::optional<std::string>(*value));
            const auto path = kineticTuningPath();
            if (path.empty())
                return true;
            auto values = readTuningFile(path);
            if (drop)
                values.erase(std::string(env));
            else
                values[std::string(env)] = *value;
            std::error_code error;
            if (path.has_parent_path())
                std::filesystem::create_directories(path.parent_path(), error);
            std::ofstream file(path, std::ios::trunc);
            file << "# KINETIC live tuning, written by the in-game options page's sliders.\n"
                    "# Applied at start over the launcher's values; delete a line to go back to them.\n";
            for (const auto& [key, text] : values)
                file << key << " = " << text << '\n';
            return static_cast<bool>(file);
        };
        addKineticActorQueries(api);
        return LuaUtil::makeReadOnly(api);
    }

    void addKineticActorQueries(sol::table& api)
    {
        // Bumped when the queries change; other mods test it to find KINETIC
        // (`pcall(require, 'openmw.kinetic')`) and what it offers.
        api["API_VERSION"] = 1;
        // True while the actor's body is KINETIC's powered ragdoll: its melee
        // is physical and its fighting AI is KINETIC's (an NPC swimming, or
        // walking free of a snag on the vanilla capsule, is not).
        api["isPowered"] = [](const LObject& actor) {
            return MWBase::Environment::get().getWorld()->isPoweredRagdoll(actor.ptr());
        };
    }

    sol::table initKineticLocalPackage(const Context& context)
    {
        sol::table api(context.sol(), sol::create);
        addKineticActorQueries(api);
        return LuaUtil::makeReadOnly(api);
    }
}
