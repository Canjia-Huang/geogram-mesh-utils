geolio_begin_file()
set(CURRENT_LIB_NAME ${PROJECT_NAME})

add_library(${CURRENT_LIB_NAME}
        SHARED
)

target_include_directories(${CURRENT_LIB_NAME}
        PUBLIC
        # Build
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/>
        # Install
        $<INSTALL_INTERFACE:include>
)

# add sources to the library
file(GLOB SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cxx)
target_sources(${CURRENT_LIB_NAME}
        PRIVATE
        ${SOURCE_FILES}
)

target_link_libraries(${CURRENT_LIB_NAME}
        PUBLIC
        Geogram::geogram
)

geolio_end_file()