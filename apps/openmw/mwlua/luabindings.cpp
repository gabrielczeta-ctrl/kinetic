#include "luabindings.hpp"

#include <components/lua/asyncpackage.hpp>
#include <components/lua/utilpackage.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwworld/datetimemanager.hpp"

#include "animationbindings.hpp"
#include "camerabindings.hpp"
#include "cellbindings.hpp"
#include "contentbindings.hpp"
#include "corebindings.hpp"
#include "debugbindings.hpp"
#include "inputbindings.hpp"
#include "kineticbindings.hpp"
#include "kineticnetbindings.hpp"
#include "localscripts.hpp"
#include "markupbindings.hpp"
#include "menuscripts.hpp"
#include "nearbybindings.hpp"
#include "objectbindings.hpp"
#include "postprocessingbindings.hpp"
#include "ragdolltestbindings.hpp"
#include "soundbindings.hpp"
#include "types/types.hpp"
#include "uibindings.hpp"
#include "vfsbindings.hpp"
#include "worldbindings.hpp"

namespace MWLua
{
    std::map<std::string, sol::object> initCommonPackages(const Context& context)
    {
        sol::state_view lua = context.mLua->unsafeState();
        MWWorld::DateTimeManager* tm = MWBase::Environment::get().getWorld()->getTimeManager();
        return {
            { "openmw.async",
                LuaUtil::getAsyncPackageInitializer(
                    lua, [tm] { return tm->getSimulationTime(); }, [tm] { return tm->getGameTime(); }) },
            { "openmw.markup", initMarkupPackage(context) },
            { "openmw.util", LuaUtil::initUtilPackage(lua) },
            { "openmw.vfs", initVFSPackage(context) },
        };
    }

    std::map<std::string, sol::object> initGlobalPackages(const Context& context)
    {
        initObjectBindingsForGlobalScripts(context);
        initCellBindingsForGlobalScripts(context);
        std::map<std::string, sol::object> packages = {
            { "openmw.core", initCorePackage(context) },
            { "openmw.types", initTypesPackage(context) },
            { "openmw.world", initWorldPackage(context) },
            { "openmw.kineticnet", initKineticNetPackage(context, true) },
        };
#ifdef OPENMW_RAGDOLL_TEST_LUA
        packages.emplace("openmw.ragdolltest", initRagdollTestPackage(context));
#endif
        return packages;
    }

    std::map<std::string, sol::object> initLocalPackages(const Context& context)
    {
        initObjectBindingsForLocalScripts(context);
        initCellBindingsForLocalScripts(context);
        LocalScripts::initializeSelfPackage(context);
        return {
            { "openmw.animation", initAnimationPackage(context) },
            { "openmw.core", initCorePackage(context) },
            { "openmw.types", initTypesPackage(context) },
            { "openmw.nearby", initNearbyPackage(context) },
            // Player scripts keep their own, fuller openmw.kinetic (a player
            // package of the same name is not replaced by this one).
            { "openmw.kinetic", initKineticLocalPackage(context) },
        };
    }

    std::map<std::string, sol::object> initPlayerPackages(const Context& context)
    {
        std::map<std::string, sol::object> packages = {
            { "openmw.ambient", initAmbientPackage(context) },
            { "openmw.camera", initCameraPackage(context.sol()) },
            { "openmw.debug", initDebugPackage(context) },
            { "openmw.input", initInputPackage(context) },
            { "openmw.kinetic", initKineticPackage(context) },
            { "openmw.kineticnet", initKineticNetPackage(context, false) },
            { "openmw.postprocessing", initPostprocessingPackage(context) },
            { "openmw.ui", initUserInterfacePackage(context) },
        };
#ifdef OPENMW_RAGDOLL_TEST_LUA
        packages.emplace("openmw.manualobservation", initManualObservationPackage(context));
#endif
        return packages;
    }

    std::map<std::string, sol::object> initMenuPackages(const Context& context)
    {
        return {
            { "openmw.core", initCorePackage(context) },
            { "openmw.ambient", initAmbientPackage(context) },
            { "openmw.ui", initUserInterfacePackage(context) },
            { "openmw.menu", initMenuPackage(context) },
            { "openmw.input", initInputPackage(context) },
        };
    }

    std::map<std::string, sol::object> initLoadPackages(const Context& context)
    {
        return {
            { "openmw.core", initCorePackage(context) },
            { "openmw.content", initContentPackage(context) },
        };
    }
}
