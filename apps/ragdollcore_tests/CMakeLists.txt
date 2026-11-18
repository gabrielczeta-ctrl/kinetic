openmw_add_executable(ragdollcore-tests
    main.cpp
    testactivebody.cpp
    testfootfall.cpp
    testfootfallgait.cpp
    testanatomicalhinge.cpp
    testarmtarget.cpp
    testbalanceauthority.cpp
    testbalancehelper.cpp
    testbalancemetrics.cpp
    testbalancecontacts.cpp
    testbalanceepisodes.cpp
    testbehavior.cpp
    testbodylease.cpp
    testposecomposer.cpp
    testsituation.cpp
    teststaggerplan.cpp
    testnpcpopulation.cpp
    testpathcount.cpp
    testpipeline.cpp
    testkinetic2phase34.cpp
    testkinetic2phase34b.cpp
    testkinetic2stride.cpp
    testplaydefaults.cpp
    teststeparbiter.cpp
    teststepplanner.cpp
    testswitchregistry.cpp
    teststridesim.cpp
    testwheelaim.cpp
    testwheeloutput.cpp
    testwheelplanner.cpp
    testcontactoutcome.cpp
    testlodge.cpp
    testcutwheel.cpp
    testhaftfit.cpp
    testfistcontrol.cpp
    testfiststeer.cpp
    testpalmparry.cpp
    testnpcshove.cpp
    testhanddrive.cpp
    testreactioncontroller.cpp
    testbuilder.cpp
    testcombatstate.cpp
    testcontactreduction.cpp
    testcontroller.cpp
    testduelscenario.cpp
    testgaitcontroller.cpp
    testhitreaction.cpp
    testimpactdamage.cpp
    testlimbclearance.cpp
    testlocalizedinjury.cpp
    testpassiveragdoll.cpp
    testphysicaldefense.cpp
    testdueltactics.cpp
    testarmstrain.cpp
    testduelread.cpp
    testrecoverycontrol.cpp
    teststrideswing.cpp
    testfallprotect.cpp
    testprofile.cpp
    testreactionloop.cpp
    testreactionmechanics.cpp
    testreactionpipeline.cpp
    testreactionviewer.cpp
    testreferencegait.cpp
    testreferencematrix.cpp
    testbehavior.cpp
    testreactionpipeline.cpp
    testreactionmechanics.cpp
    testreplay.cpp
    testscenario.cpp
    testscenarioinput.cpp
    teststrikecooldown.cpp
    testswingtrajectory.cpp
    testsupportgrip.cpp
    testunitmodel.cpp
    testtuning.cpp
    testweaponcontact.cpp
    testweaponalignment.cpp
    testwound.cpp
    testwoundheal.cpp
    testweaponswing.cpp
    feedbackpilot.cpp
)

target_link_libraries(ragdollcore-tests
    GTest::GTest
    ragdollcore
)

# The shipped ragdoll data (the gait table sanity test reads it).
target_compile_definitions(ragdollcore-tests PRIVATE
    RAGDOLLCORE_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/files/data/ragdoll")

# These tests intentionally exercise unavailable/invalid floating-point input.
if (MSVC)
    set_source_files_properties(testbehavior.cpp testreactionpipeline.cpp testreactionmechanics.cpp testanatomicalhinge.cpp testbalanceauthority.cpp testbalancecontacts.cpp testbalanceepisodes.cpp testsupportgrip.cpp
        PROPERTIES COMPILE_OPTIONS "/fp:precise")
else()
    set_source_files_properties(testbehavior.cpp testreactionpipeline.cpp testreactionmechanics.cpp testanatomicalhinge.cpp testbalanceauthority.cpp testbalancecontacts.cpp testbalanceepisodes.cpp testsupportgrip.cpp
        PROPERTIES COMPILE_OPTIONS "-fno-fast-math")
endif()

add_test(NAME ragdollcore-tests COMMAND ragdollcore-tests --gtest_brief=1)


if (WIN32)
    target_sources(ragdollcore-tests PRIVATE ${CMAKE_SOURCE_DIR}/files/windows/other-apps.manifest)
endif()

# The MOVE-01 training pilot (offline only): StrideSim's walking course and
# the get-up trainer (trainer.cpp).
openmw_add_executable(ragdollcore-trainer trainer.cpp)
# The course loads the game's authored profile (components/ragdollprofile).
target_link_libraries(ragdollcore-trainer ragdollcore components)
