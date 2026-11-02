#ifndef MWLUA_KINETICNETBINDINGS_H
#define MWLUA_KINETICNETBINDINGS_H

#include <sol/forward.hpp>

namespace MWLua
{
    struct Context;

    // openmw.kineticnet (NET-01 duel, mwworld/kineticnet.hpp): the link's
    // state and text messages; global scripts also receive the messages and
    // choose the puppet (setPuppet).
    sol::table initKineticNetPackage(const Context& context, bool global);
}

#endif
