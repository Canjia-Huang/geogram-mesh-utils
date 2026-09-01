if(COMMAND geolio_begin_file)
    geolio_begin_file()
endif()

find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3")

if(COMMAND geolio_end_file)
    geolio_end_file()
endif()