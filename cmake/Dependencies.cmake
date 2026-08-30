geolio_begin_file()

if (NOT TARGET CGAL::CGAL)
    include(CGAL)
endif()
if (NOT TARGET Geogram::geogram)
    include(Geogram)
endif()

geolio_end_file()