geolio_begin_file()

find_package(Geogram REQUIRED)

if(Geogram_FOUND)
    message(STATUS "Geogram found in ${GEOGRAM_INCLUDE_DIR}")
    include_directories(${GEOGRAM_INCLUDE_DIR})

    if(WIN32) # this macro should normally be defined when configuring the geogram, but it appears not in some cases...
        add_compile_definitions(GEOGRAM_USE_BUILTIN_DEPS)
        add_compile_definitions(GEO_DYNAMIC_LIBS) # ref https://github.com/BrunoLevy/geogram/discussions/376
    endif()

    if(WIN32)
        # Force /MD (dynamic MultiThreadedDLL) in all configurations to match
        # geogram.dll which is built with /MD. FindGeogram.cmake defaults to
        # VORPALINE_BUILD_DYNAMIC=FALSE which forces /MT (static CRT), creating
        # a _ITERATOR_DEBUG_LEVEL mismatch that corrupts std::string data when
        # exceptions cross the DLL boundary.
        foreach(config DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
            string(REPLACE "/MT" "/MD" CMAKE_CXX_FLAGS_${config} "${CMAKE_CXX_FLAGS_${config}}")
            string(REPLACE "/MT" "/MD" CMAKE_C_FLAGS_${config} "${CMAKE_C_FLAGS_${config}}")
        endforeach()
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
    endif()

    if(WIN32) # copy geogram's dlls to the execute file path
        message(STATUS "Copying geogram's release dll -> ${CMAKE_BINARY_DIR}/bin")
        file(GLOB GEOGRAM_RELEASE_DLL_FILES "${GEOGRAM_PATH}/build/Windows/bin/Release/*.dll")
        file(COPY ${GEOGRAM_RELEASE_DLL_FILES} DESTINATION ${CMAKE_BINARY_DIR}/bin/Release)


        message(STATUS "Copying geogram's debug dll -> ${CMAKE_BINARY_DIR}/bin")
        file(GLOB GEOGRAM_DEBUG_DLL_FILES "${GEOGRAM_PATH}/build/Windows/bin/Debug/*.dll")
        file(COPY ${GEOGRAM_DEBUG_DLL_FILES} DESTINATION ${CMAKE_BINARY_DIR}/bin/Debug)
    endif()
else()
    message(WARNING "Geogram not found!")
endif()

geolio_end_file()