set(RAGDOLLCORE_SOURCES
    src/activebody.cpp
    src/footfall.cpp
    src/footfallgait.cpp
    src/armtarget.cpp
    src/balanceauthority.cpp
    src/balancehelper.cpp
    src/balancemetrics.cpp
    src/balancecontacts.cpp
    src/balanceepisodes.cpp
    src/behavior.cpp
    src/blow.cpp
    src/bodylease.cpp
    src/contactoutcome.cpp
    src/reactioncontroller.cpp
    src/builder.cpp
    src/combatstate.cpp
    src/contact.cpp
    src/controller.cpp
    src/duelscenario.cpp
    src/dueltactics.cpp
    src/recoverycontrol.cpp
    src/fallprotect.cpp
    src/gaitcontroller.cpp
    src/impactdamage.cpp
    src/limbclearance.cpp
    src/passiveragdoll.cpp
    src/posecomposer.cpp
    src/physicaldefense.cpp
    src/profile.cpp
    src/reactionmechanics.cpp
    src/reactionpipeline.cpp
    src/referencegait.cpp
    src/behavior.cpp
    src/reactionauthority.cpp
    src/reactionpipeline.cpp
    src/reactionmechanics.cpp
    src/replay.cpp
    src/scenario.cpp
    src/situation.cpp
    src/staggerplan.cpp
    src/steparbiter.cpp
    src/swingtrajectory.cpp
    src/tuning.cpp
    src/supportgrip.cpp
    src/strikecooldown.cpp
    src/weaponcontact.cpp
    src/weaponalignment.cpp
    src/weaponswing.cpp
)

set(RAGDOLLCORE_HEADERS
    include/ragdollcore/anatomicalhinge.hpp
    include/ragdollcore/tuning.hpp
    include/ragdollcore/anatomy.hpp
    include/ragdollcore/activebody.hpp
    include/ragdollcore/footfall.hpp
    include/ragdollcore/footfallgait.hpp
    include/ragdollcore/armtarget.hpp
    include/ragdollcore/balanceauthority.hpp
    include/ragdollcore/balancehelper.hpp
    include/ragdollcore/balancemetrics.hpp
    include/ragdollcore/balancecontacts.hpp
    include/ragdollcore/balanceepisodes.hpp
    include/ragdollcore/behavior.hpp
    include/ragdollcore/blow.hpp
    include/ragdollcore/bodylease.hpp
    include/ragdollcore/contactoutcome.hpp
    include/ragdollcore/reactioncontroller.hpp
    include/ragdollcore/builder.hpp
    include/ragdollcore/combatstate.hpp
    include/ragdollcore/contact.hpp
    include/ragdollcore/controller.hpp
    include/ragdollcore/duelscenario.hpp
    include/ragdollcore/dueltactics.hpp
    include/ragdollcore/duelread.hpp
    include/ragdollcore/recoverycontrol.hpp
    include/ragdollcore/fallprotect.hpp
    include/ragdollcore/gaitcontroller.hpp
    include/ragdollcore/hitreaction.hpp
    include/ragdollcore/impactdamage.hpp
    include/ragdollcore/limbclearance.hpp
    include/ragdollcore/passiveragdoll.hpp
    include/ragdollcore/posecomposer.hpp
    include/ragdollcore/physicaldefense.hpp
    include/ragdollcore/profile.hpp
    include/ragdollcore/reactioncontract.hpp
    include/ragdollcore/reactionmechanics.hpp
    include/ragdollcore/reactionpipeline.hpp
    include/ragdollcore/referencegait.hpp
    include/ragdollcore/behavior.hpp
    include/ragdollcore/reactionauthority.hpp
    include/ragdollcore/reactioncontract.hpp
    include/ragdollcore/reactionpipeline.hpp
    include/ragdollcore/reactionmechanics.hpp
    include/ragdollcore/replay.hpp
    include/ragdollcore/scenario.hpp
    include/ragdollcore/situation.hpp
    include/ragdollcore/staggerplan.hpp
    include/ragdollcore/steparbiter.hpp
    include/ragdollcore/swingtrajectory.hpp
    include/ragdollcore/supportgrip.hpp
    include/ragdollcore/strikecooldown.hpp
    include/ragdollcore/unitmodel.hpp
    include/ragdollcore/weaponcontact.hpp
    include/ragdollcore/weaponalignment.hpp
    include/ragdollcore/wound.hpp
    include/ragdollcore/weaponswing.hpp
)

# Keep the controller boundary enforceable even when a developer accidentally
# has broad include directories in their local build.
foreach(source IN LISTS RAGDOLLCORE_SOURCES RAGDOLLCORE_HEADERS)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/${source}" contents)
    if(contents MATCHES "#[ \t]*include[ \t]*[<\"](osg|components/|apps/)"
        OR contents MATCHES "MWWorld::|VFS::|MyGUI::|sol::")
        message(FATAL_ERROR "components/ragdollcore architecture violation in ${source}")
    endif()
endforeach()

add_library(ragdollcore STATIC ${RAGDOLLCORE_SOURCES} ${RAGDOLLCORE_HEADERS})
# These numerical routines rely on finite checks and residual-controlled double
# arithmetic. Scope the floating-point mode to them; native Jolt code retains
# its existing configuration and solver behavior.
if (MSVC)
    set_source_files_properties(src/behavior.cpp src/reactionauthority.cpp src/reactionpipeline.cpp src/reactionmechanics.cpp src/balanceauthority.cpp src/balancemetrics.cpp src/balancecontacts.cpp src/balanceepisodes.cpp src/supportgrip.cpp
        PROPERTIES COMPILE_OPTIONS "/fp:precise")
else()
    set_source_files_properties(src/behavior.cpp src/reactionauthority.cpp src/reactionpipeline.cpp src/reactionmechanics.cpp src/balanceauthority.cpp src/balancemetrics.cpp src/balancecontacts.cpp src/balanceepisodes.cpp src/supportgrip.cpp
        PROPERTIES COMPILE_OPTIONS "-fno-fast-math")
endif()
target_compile_features(ragdollcore PUBLIC cxx_std_20)
target_include_directories(ragdollcore PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(ragdollcore PUBLIC Jolt)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${RAGDOLLCORE_SOURCES} ${RAGDOLLCORE_HEADERS})
