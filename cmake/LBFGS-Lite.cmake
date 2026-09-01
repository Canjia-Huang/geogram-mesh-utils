if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()

include(FetchContent)
FetchContent_Declare(
        lbfgs_lite
        GIT_REPOSITORY https://github.com/ZJU-FAST-Lab/LBFGS-Lite.git
        GIT_TAG        v2.3
        # Header-only dependency: SOURCE_SUBDIR points at a directory that has no
        # CMakeLists.txt, so FetchContent_MakeAvailable() downloads the sources but
        # skips add_subdirectory() — LBFGS-Lite's own CMakeLists only builds its
        # example and would rewrite global CMAKE_CXX_FLAGS / CMAKE_BUILD_TYPE.
        SOURCE_SUBDIR  include
)
FetchContent_MakeAvailable(lbfgs_lite)
message(STATUS "LBFGS-Lite v2.3 fetched at ${lbfgs_lite_SOURCE_DIR}")

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()