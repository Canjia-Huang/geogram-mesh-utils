########################################################################################################################
# CMake options
########################################################################################################################
cmake_minimum_required(VERSION 3.30)
project(Geolio
        VERSION 0.1.0
        DESCRIPTION "Geometry library"
        HOMEPAGE_URL "https://github.com/Canjia-Huang/geolio"
        LANGUAGES CXX
)

# == Options ==
option(BUILD_TESTS "Build tests" ON)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    # standalone
else()
    # as submodule
    set(BUILD_TESTS OFF)
endif()

if(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8")
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

########################################################################################################################
# CMake module path
########################################################################################################################
list(PREPEND CMAKE_MODULE_PATH
        ${CMAKE_CURRENT_LIST_DIR}/cmake)
include(LoggingHelper)
include(MyAddLibrary)

########################################################################################################################
# Include
########################################################################################################################
include(Dependencies)
add_subdirectory(third_party)
add_subdirectory(src)

########################################################################################################################
# Add subdirectories
########################################################################################################################
if (BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

########################################################################################################################
# Install
########################################################################################################################
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

configure_package_config_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/GeolioConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/GeolioConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Geolio
)

write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/GeolioConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
)

install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/GeolioConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/GeolioConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Geolio
)
