set(PHYSICSMESHTOOL
    main.cpp
)
source_group(apps\\physicsobjecttool FILES ${PHYSICSMESHTOOL})

openmw_add_executable(openmw-physicsobjecttool ${PHYSICSMESHTOOL})

target_link_libraries(openmw-physicsobjecttool
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    components
)

if (BUILD_WITH_CODE_COVERAGE)
    target_compile_options(openmw-physicsobjecttool PRIVATE --coverage)
    target_link_libraries(openmw-physicsobjecttool gcov)
endif()

if (WIN32)
    install(TARGETS openmw-physicsobjecttool RUNTIME DESTINATION ".")
endif()

if (MSVC AND PRECOMPILE_HEADERS_WITH_MSVC)
    target_precompile_headers(openmw-physicsobjecttool PRIVATE
        <string>
        <vector>
    )
endif()
