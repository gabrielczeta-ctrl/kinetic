openmw_add_executable(ragdollcore-walksearch
    main.cpp
    nativebody.hpp
    play01worker.cpp
)

target_link_libraries(ragdollcore-walksearch
    ragdollcore
    components
)

if (WIN32)
    target_sources(ragdollcore-walksearch PRIVATE ${CMAKE_SOURCE_DIR}/files/windows/other-apps.manifest)
endif()
