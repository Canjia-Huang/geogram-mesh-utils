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

    if(WIN32) # copy geogram's runtime DLLs next to the executables (build/bin/<config>)
        if(GEOGRAM_LIBRARY)
            # Derive the geogram build directory (e.g. build/Windows) from the
            # located import library instead of relying on GEOGRAM_PATH, which
            # is not set by FindGeogram.cmake and made this copy a no-op.
            get_filename_component(_GEOGRAM_LIB_DIR "${GEOGRAM_LIBRARY}" DIRECTORY)
            get_filename_component(_GEOGRAM_LIB_PARENT "${_GEOGRAM_LIB_DIR}" DIRECTORY)
            get_filename_component(GEOGRAM_BUILD_DIR "${_GEOGRAM_LIB_PARENT}" DIRECTORY)
            # Copy the whole DLL set (geogram.dll, geogram_gfx.dll, glfw.dll,
            # ...) for every configuration that exists in the geogram build.
            foreach(cfg Release Debug RelWithDebInfo MinSizeRel)
                file(GLOB _GEOGRAM_CFG_DLL_FILES "${GEOGRAM_BUILD_DIR}/bin/${cfg}/*.dll")
                if(_GEOGRAM_CFG_DLL_FILES)
                    message(STATUS "Copying geogram ${cfg} dlls -> ${CMAKE_BINARY_DIR}/bin/${cfg}")
                    file(COPY ${_GEOGRAM_CFG_DLL_FILES} DESTINATION "${CMAKE_BINARY_DIR}/bin/${cfg}")
                endif()
            endforeach()
        endif()
    endif()
else()
    message(WARNING "Geogram not found!")
endif()

geolio_end_file()