#include "kineticnetbindings.hpp"

#include <cstdlib>

#include <components/lua/luastate.hpp>

#include "apps/openmw/mwbase/environment.hpp"
#include "apps/openmw/mwbase/world.hpp"
#include "apps/openmw/mwworld/kineticnet.hpp"

#include "context.hpp"
#include "luamanagerimp.hpp"
#include "object.hpp"

namespace MWLua
{
    sol::table initKineticNetPackage(const Context& context, bool global)
    {
        sol::state_view lua = context.sol();
        sol::table api(lua, sol::create);
        using MWWorld::KineticNet;
        // 'off' (no duel), 'waiting' (hosting, no opponent), 'connecting',
        // 'connected' or 'lost'.
        api["state"] = [] { return std::string(KineticNet::stateName(KineticNet::get().state())); };
        api["isHost"] = [] { return KineticNet::get().isHost(); };
        // Round trip in milliseconds, -1 until measured.
        api["ping"] = [] { return KineticNet::get().pingMs(); };
        api["localName"] = [] { return KineticNet::get().localName(); };
        api["remoteName"] = [] { return KineticNet::get().remoteName(); };
        api["endpoint"] = [] { return KineticNet::get().endpoint(); };
        api["port"] = [] { return KineticNet::get().port(); };
        api["lastError"] = [] { return KineticNet::get().lastError(); };
        // A new number is a new connection: the scripts set the duel up again.
        api["session"] = [] { return KineticNet::get().session(); };
        // Reliable, in order; up to 60000 bytes.
        api["send"] = [](std::string_view text) { KineticNet::get().sendMessage(std::string(text)); };
        // OPENMW_KINETIC_DUEL_BOT (tests): both fighters start close and the
        // player's script clicks attacks by itself.
        api["bot"] = [] {
            const char* bot = std::getenv("OPENMW_KINETIC_DUEL_BOT");
            return bot != nullptr && *bot != '\0' && *bot != '0';
        };
        if (!global)
        {
            // A scripted attack click (the bot): press, then release.
            api["click"] = [context](bool held) {
                context.mLuaManager->addAction(
                    [held] { MWBase::Environment::get().getWorld()->driveActorRagdollSwing(0.0f, 0.0f, held); },
                    "KineticNetClickAction");
            };
            return LuaUtil::makeReadOnly(api);
        }

        api["receive"] = [](sol::this_state state) {
            sol::state_view view(state);
            sol::table result = view.create_table();
            int index = 1;
            for (std::string& text : KineticNet::get().takeMessages())
                result[index++] = std::move(text);
            return result;
        };
        // The actor that replays the opponent's body: give it the opponent's
        // weapon (drawn) and shield first, its parts are built from them.
        api["setPuppet"] = [context](const GObject& actor) {
            context.mLuaManager->addAction(
                [actor = GObject(actor)] { MWBase::Environment::get().getWorld()->setNetPuppet(actor.ptr()); },
                "KineticNetSetPuppetAction");
        };
        api["clearPuppet"] = [context] {
            context.mLuaManager->addAction(
                [] { MWBase::Environment::get().getWorld()->clearNetPuppet(); }, "KineticNetClearPuppetAction");
        };
        api["isPuppet"] = [](const GObject& actor) {
            return MWBase::Environment::get().getWorld()->isNetPuppet(actor.ptr());
        };
        return LuaUtil::makeReadOnly(api);
    }
}
