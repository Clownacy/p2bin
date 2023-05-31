cmake_minimum_required(VERSION 3.0)

project(p2bin LANGUAGES C)

add_executable(p2bin
	"main.c"
	"lz_comp2/LZSS.c"
	"lz_comp2/LZSS.h"
)

set_target_properties(p2bin PROPERTIES
	C_STANDARD 90
	C_STANDARD_REQUIRED NO
	C_EXTENSIONS OFF
)

add_subdirectory("accurate-kosinski" EXCLUDE_FROM_ALL)
target_link_libraries(p2bin PRIVATE kosinski-compressor)

add_subdirectory("clownlzss" EXCLUDE_FROM_ALL)
target_link_libraries(p2bin PRIVATE clownlzss-kosinski clownlzss-kosinskiplus clownlzss-saxman)
