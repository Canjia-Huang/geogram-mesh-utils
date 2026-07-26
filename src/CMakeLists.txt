geolio_begin_file()

add_library(${CURRENT_LIB_NAME})

target_include_directories(${CURRENT_LIB_NAME}
        PRIVATE
        "${PROJECT_SOURCE_DIR}/include/"
)

# configure sources
file(GLOB SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cxx)

# add sources to the library
target_sources(${CURRENT_LIB_NAME}
        PRIVATE
        ${SOURCE_FILES}
)
target_link_libraries(${CURRENT_LIB_NAME}
        PUBLIC
        Geogram::geogram
)

geolio_end_file()