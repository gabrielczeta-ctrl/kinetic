#ifndef OPENMW_MWLUA_RAGDOLLTESTBINDINGS_H
#define OPENMW_MWLUA_RAGDOLLTESTBINDINGS_H

#include <sol/forward.hpp>

namespace MWLua
{
    struct Context;

    sol::table initRagdollTestPackage(const Context& context);
    sol::table initManualObservationPackage(const Context& context);
}

#endif
