geolio_begin_file()
set(CURRENT_LIB_NAME ${PROJECT_NAME})
set(CURRENT_LIB_ALIAS_NAME ${PROJECT_NAME}::geolio)

add_library(${CURRENT_LIB_NAME}
        SHARED
)
add_library(${CURRENT_LIB_ALIAS_NAME} ALIAS ${CURRENT_LIB_NAME})

target_include_directories(${CURRENT_LIB_NAME}
        PUBLIC
        # Build
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/>
        # Install
        $<INSTALL_INTERFACE:include>
)

# Add sources to the library
file(GLOB SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
target_sources(${CURRENT_LIB_NAME}
        PRIVATE
        ${SOURCE_FILES}
)

# Add subdirectory (component library)
add_subdirectory(local_operation_optimization)

target_link_libraries(${CURRENT_LIB_NAME}
        PRIVATE
        ${PROJECT_NAME}::local_operation_optimization
        ${PROJECT_NAME}::third_party
        Geogram::geogram
)

# Public headers for install
file(GLOB PUBLIC_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/geolio/*.h
)
set_target_properties(${CURRENT_LIB_NAME} PROPERTIES
        PUBLIC_HEADER "${PUBLIC_HEADERS}"
)

# Install
include(GNUInstallDirs)
install(TARGETS ${CURRENT_LIB_NAME}
        EXPORT GeolioTargets
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/geolio
)

install(EXPORT GeolioTargets
        FILE GeolioTargets.cmake
        NAMESPACE Geolio::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Geolio
)

geolio_end_file()
