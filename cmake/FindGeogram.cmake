if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()

# Find Geogram
# ------------
#
# Find Geogram include dirs and libraries
#
# This module defines the following variables:
#
#   Geogram_FOUND        - True if geogram has been found.
#   Geogram::geogram     - Imported target for the main Geogram library.
#   Geogram::geogram_gfx - Imported target for Geogram graphics library.
#
# This module reads hints about the Geogram location from the following
# environment variables:
#
#   GEOGRAM_INSTALL_PREFIX - Directory where Geogram is installed.
#
# Authors: Jeremie Dumas
#          Pierre Moulon
#          Bruno Levy

set (GEOGRAM_SEARCH_PATHS
        "${GEOGRAM_DIR}"
        "$ENV{GEOGRAM_DIR}"
        "${GEOGRAM_PATH}"
        "$ENV{GEOGRAM_PATH}"
        "${GEOGRAM_INSTALL_PREFIX}"
        "$ENV{GEOGRAM_INSTALL_PREFIX}"
        "/usr/local/"
        "$ENV{PROGRAMFILES}/Geogram"
        "$ENV{PROGRAMW6432}/Geogram"
        "../geogram"
)

set (GEOGRAM_SEARCH_PATHS_SYSTEM
        "/usr/lib"
        "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
)

# Geogram library search suffixes. The macOS/Linux builds produced by geogram's
# ./configure.sh are always Release, so those paths are used regardless of the
# top-level build type (unchanged). On Windows geogram is a multi-config Visual
# Studio build under build/Windows, so the library directory depends on the
# configuration that was actually built: prefer the one matching the top-level
# build type (the CI passes CMAKE_BUILD_TYPE at configure time) and fall back to
# the other one if only it is present.
set (GEOGRAM_LIBRARY_PATH_SUFFIXES
        build/Darwin-clang-dynamic-Release/lib
        build/Linux64-gcc-dynamic-Release/lib
)
if(WIN32)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        list(APPEND GEOGRAM_LIBRARY_PATH_SUFFIXES
                build/Windows/lib/Debug
                build/Windows/lib/Release
        )
    else()
        list(APPEND GEOGRAM_LIBRARY_PATH_SUFFIXES
                build/Windows/lib/Release
                build/Windows/lib/Debug
        )
    endif()
endif()

find_path (GEOGRAM_INCLUDE_DIR
        geogram/basic/common.h
        PATHS ${GEOGRAM_SEARCH_PATHS}
        PATH_SUFFIXES src/lib
)
message(STATUS "GEOGRAM_INCLUDE_DIR: ${GEOGRAM_INCLUDE_DIR}")

find_library (GEOGRAM_LIBRARY
        NAMES geogram
        PATHS ${GEOGRAM_SEARCH_PATHS}
        PATH_SUFFIXES ${GEOGRAM_LIBRARY_PATH_SUFFIXES}
)
message(STATUS "GEOGRAM_LIBRARY: ${GEOGRAM_LIBRARY}")

find_library (GEOGRAM_GFX_LIBRARY
        NAMES geogram_gfx
        PATHS ${GEOGRAM_SEARCH_PATHS}
        PATH_SUFFIXES ${GEOGRAM_LIBRARY_PATH_SUFFIXES}
)
message(STATUS "GEOGRAM_GFX_LIBRARY: ${GEOGRAM_GFX_LIBRARY}")

# This one we search in both Geogram search path and
# system search path since it may be already installed
# in the system
find_library (GEOGRAM_GLFW3_LIBRARY
        NAMES glfw3 glfw geogram_glfw3 glfw3dll glfwdll
        PATHS ${GEOGRAM_SEARCH_PATHS} ${GEOGRAM_SEARCH_PATHS_SYSTEM}
        PATH_SUFFIXES ${GEOGRAM_LIBRARY_PATH_SUFFIXES}
)
message(STATUS "GEOGRAM_GLFW3_LIBRARY: ${GEOGRAM_GLFW3_LIBRARY}")

# Find TBB (transitive dependency of Geogram). Geogram headers may inline TBB calls,
# so consumers of Geogram::geogram must also link TBB on Linux.
find_library (GEOGRAM_TBB_LIBRARY
        NAMES tbb
        PATHS ${GEOGRAM_SEARCH_PATHS_SYSTEM}
)
message(STATUS "GEOGRAM_TBB_LIBRARY: ${GEOGRAM_TBB_LIBRARY}")

find_library (GEOGRAM_TBB_MALLOC_LIBRARY
        NAMES tbbmalloc
        PATHS ${GEOGRAM_SEARCH_PATHS_SYSTEM}
)
message(STATUS "GEOGRAM_TBB_MALLOC_LIBRARY: ${GEOGRAM_TBB_MALLOC_LIBRARY}")

# On Windows, locate the geogram DLL so consumers can copy it for runtime.
if(WIN32 AND GEOGRAM_LIBRARY)
    get_filename_component(_GEOGRAM_LIB_DIR "${GEOGRAM_LIBRARY}" DIRECTORY)
    get_filename_component(_GEOGRAM_CONFIG "${_GEOGRAM_LIB_DIR}" NAME)
    get_filename_component(_GEOGRAM_LIB_PARENT "${_GEOGRAM_LIB_DIR}" DIRECTORY)
    get_filename_component(_GEOGRAM_BUILD_DIR "${_GEOGRAM_LIB_PARENT}" DIRECTORY)
    find_file(GEOGRAM_DLL
        NAMES geogram.dll
        HINTS
            "${_GEOGRAM_LIB_DIR}"
            "${_GEOGRAM_BUILD_DIR}/bin"
        PATH_SUFFIXES
            .
            "${_GEOGRAM_CONFIG}"
    )
    mark_as_advanced(GEOGRAM_DLL)
    if(GEOGRAM_DLL)
        message(STATUS "GEOGRAM_DLL: ${GEOGRAM_DLL}")
    else()
        message(STATUS "GEOGRAM_DLL: not found")
    endif()
endif()

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args(
        Geogram DEFAULT_MSG GEOGRAM_LIBRARY GEOGRAM_INCLUDE_DIR
)

# Create an imported target for Geogram
If (GEOGRAM_FOUND)

    set(GEOGRAM_INSTALL_PREFIX ${GEOGRAM_INCLUDE_DIR}/../..)

    if (NOT TARGET Geogram::geogram)
        add_library (Geogram::geogram UNKNOWN IMPORTED)

        # Interface include directory
        set_target_Properties(Geogram::geogram PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${GEOGRAM_INCLUDE_DIR}"
        )

        # On Windows Geogram is built with its bundled third-party deps and as
        # dynamic libraries. Both are compile-time requirements for any target
        # that consumes geogram headers (GEOGRAM_USE_BUILTIN_DEPS switches the
        # zlib include to geogram's bundled copy, otherwise <zlib.h> is expected
        # from the system). Exposing them on the interface lets add_subdirectory()
        # consumers inherit them; directory-scoped add_compile_definitions() in
        # Geogram.cmake alone does not propagate outside geolio.
        if(WIN32)
            set_target_properties(Geogram::geogram PROPERTIES
                    INTERFACE_COMPILE_DEFINITIONS
                    "GEOGRAM_USE_BUILTIN_DEPS;GEO_DYNAMIC_LIBS;NOMINMAX;WIN32_LEAN_AND_MEAN;VC_EXTRALEAN;_USE_MATH_DEFINES"
            )
        endif()

        # Link to library file
        Set_Target_Properties(Geogram::geogram PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                IMPORTED_LOCATION "${GEOGRAM_LIBRARY}"
        )

        # TBB is a transitive dependency of Geogram. Geogram headers may inline
        # TBB calls, requiring consumers to link it explicitly on Linux.
        set(GEOGRAM_TBB_LIBRARIES "")
        if(GEOGRAM_TBB_LIBRARY)
            list(APPEND GEOGRAM_TBB_LIBRARIES "${GEOGRAM_TBB_LIBRARY}")
        endif()
        if(GEOGRAM_TBB_MALLOC_LIBRARY)
            list(APPEND GEOGRAM_TBB_LIBRARIES "${GEOGRAM_TBB_MALLOC_LIBRARY}")
        endif()
        if(GEOGRAM_TBB_LIBRARIES)
            set_target_properties(Geogram::geogram PROPERTIES
                INTERFACE_LINK_LIBRARIES "${GEOGRAM_TBB_LIBRARIES}"
            )
        endif()
    endif ()

    if (NOT TARGET Geogram::geogram_gfx)
        add_library (Geogram::geogram_gfx UNKNOWN IMPORTED)

        set_target_properties(Geogram::geogram_gfx PROPERTIES
                INTERFACE_LINK_LIBRARIES ${GEOGRAM_GLFW3_LIBRARY}
        )

        # Interface include directory
        set_target_properties(Geogram::geogram_gfx PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${GEOGRAM_INCLUDE_DIR}"
        )

        # Link to library file
        set_target_properties(Geogram::geogram_gfx PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
                IMPORTED_LOCATION "${GEOGRAM_GFX_LIBRARY}"
        )

    endif ()


endif ()

# Hide variables from the default CMake-Gui options
mark_as_advanced (GEOGRAM_LIBRARY GEOGRAM_GFX_LIBRARY GEOGRAM_INCLUDE_DIR)

# Some specific settings for Windows

if(WIN32)
    # Default mode for Windows uses static libraries. Use this variable to
    # link with geogram compiled as DLLs.
    set(VORPALINE_BUILD_DYNAMIC FALSE CACHE BOOL "Installed Geogram uses DLLs")

    # remove warning for multiply defined symbols (caused by multiple
    # instanciations of STL templates)
    add_definitions(/wd4251)

    # remove all unused stuff from windows.h
    add_definitions(-DWIN32_LEAN_AND_MEAN)
    add_definitions(-DVC_EXTRALEAN)

    # do not define a min() and a max() macro, breaks
    # std::min() and std::max() !!
    add_definitions(-DNOMINMAX)

    # we want M_PI etc...
    add_definitions(-D_USE_MATH_DEFINES)

    if(NOT VORPALINE_BUILD_DYNAMIC)
        # If we use static library, we link with the static C++ runtime.
        foreach(config ${CMAKE_CONFIGURATION_TYPES})
            string(TOUPPER ${config} config)
            string(REPLACE /MD /MT CMAKE_C_FLAGS_${config} "${CMAKE_C_FLAGS_${config}}")
            string(REPLACE /MD /MT CMAKE_CXX_FLAGS_${config} "${CMAKE_CXX_FLAGS_${config}}")
        endforeach()
    endif()

endif()

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()