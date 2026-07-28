geolio_begin_file()
set(CURRENT_LIB_NAME ${PROJECT_NAME}_THIRD_PARTY)

add_library(${CURRENT_LIB_NAME} INTERFACE)

target_include_directories(${CURRENT_LIB_NAME} INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/CLI11/include"
#        "${CMAKE_CURRENT_SOURCE_DIR}/json/include"
#        "${CMAKE_CURRENT_SOURCE_DIR}/libTimekeeper/src"
        "${CMAKE_CURRENT_SOURCE_DIR}/spdlog/include"
)

geolio_end_file()