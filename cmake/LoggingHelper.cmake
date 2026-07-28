macro(geolio_begin_file)
    get_filename_component(CURRENT_FILE_NAME "${CMAKE_CURRENT_LIST_FILE}" NAME)

    list(APPEND CMAKE_MESSAGE_INDENT "[${CMAKE_CURRENT_LIST_FILE}]: ")
    message(STATUS "")
endmacro()

macro(geolio_end_file)
    list(POP_BACK CMAKE_MESSAGE_INDENT)
endmacro()