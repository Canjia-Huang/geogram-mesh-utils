########################################################################################################################
# Download and install googletest
########################################################################################################################
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

########################################################################################################################
# Build tests
########################################################################################################################
file(GLOB TESTS_SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

add_executable(${PROJECT_NAME}_TESTS
        ${TESTS_SOURCE})

target_link_libraries(${PROJECT_NAME}_TESTS
        ${PROJECT_NAME}::geolio
        ${PROJECT_NAME}::third_party
        Geogram::geogram
        GTest::gtest_main
)

add_compile_definitions(TEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/")

if(WIN32)
    # The test executable needs geolio.dll (and potentially geogram.dll) at runtime.
    # Copy them alongside the test executable so Windows can find them.
    add_custom_command(TARGET ${PROJECT_NAME}_TESTS POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:geolio>"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}_TESTS>"
        COMMENT "Copying geolio.dll to test directory"
    )
    if(GEOGRAM_DLL)
        add_custom_command(TARGET ${PROJECT_NAME}_TESTS POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${GEOGRAM_DLL}"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}_TESTS>"
            COMMENT "Copying geogram.dll to test directory"
        )
    endif()
endif()

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}_TESTS)