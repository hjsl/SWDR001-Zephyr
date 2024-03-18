zephyr_include_directories(.)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/apps_common.c
	${CMAKE_CURRENT_SOURCE_DIR}/smtc_dbpsk.c)

add_subdirectory(printers)
