if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()
set(CURRENT_LIB_NAME ${PROJECT_NAME}_third_party)
set(CURRENT_LIB_ALIAS_NAME ${PROJECT_NAME}::third_party)

add_library(${CURRENT_LIB_NAME} INTERFACE)
add_library(${CURRENT_LIB_ALIAS_NAME} ALIAS ${CURRENT_LIB_NAME})

# Optional header-only LBFGS-Lite optimizer (https://github.com/ZJU-FAST-Lab/LBFGS-Lite).
# Downloaded at configure time via FetchContent when GEOLIO_ENABLE_LBFGS_LITE is ON.
if(GEOLIO_ENABLE_LBFGS_LITE)
    include(LBFGS-Lite)
    # SYSTEM: treat LBFGS-Lite as a third-party include dir (suppresses warnings
    # from its headers). lbfgs.hpp is header-only but uses Eigen, which is already
    # exposed to this target below.
    target_include_directories(${CURRENT_LIB_NAME} SYSTEM INTERFACE
            "${lbfgs_lite_SOURCE_DIR}/../"
    )

    # Feature macro: defined when the LBFGS-Lite optimizer is enabled (see the
    # GEOLIO_ENABLE_LBFGS_LITE option in the top-level CMakeLists.txt). PUBLIC so the
    # macro is also visible to consumers of geolio::common in their own translation
    # units; code may guard LBFGS-Lite-specific includes/implementations with
    #   #if GEOLIO_ENABLE_LBFGS_LITE
    if(GEOLIO_ENABLE_LBFGS_LITE)
        target_compile_definitions(${CURRENT_LIB_NAME} INTERFACE GEOLIO_ENABLE_LBFGS_LITE=1)
    endif()
endif()

target_include_directories(${CURRENT_LIB_NAME} INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/CLI11/include"
#        "${CMAKE_CURRENT_SOURCE_DIR}/json/include"
#        "${CMAKE_CURRENT_SOURCE_DIR}/libTimekeeper/src"
        "${CMAKE_CURRENT_SOURCE_DIR}/imoguizmo"
        "${CMAKE_CURRENT_SOURCE_DIR}/spdlog/include"
        "${GEOGRAM_INCLUDE_DIR}/geogram_gfx/third_party/imgui"
)

include(Dependencies)
target_link_libraries(${CURRENT_LIB_NAME} INTERFACE
        Geogram::geogram
        Geogram::geogram_gfx
        Eigen3::Eigen
)

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()