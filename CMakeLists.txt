cmake_minimum_required(VERSION 3.0)

project(p2bin LANGUAGES C)

add_executable(p2bin "main.c")

set_target_properties(p2bin PROPERTIES
	C_STANDARD 90
	C_STANDARD_REQUIRED NO
	C_EXTENSIONS OFF
)
