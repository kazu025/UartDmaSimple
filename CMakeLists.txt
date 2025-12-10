cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/pico_sdk_import.cmake)
project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()


add_executable(main
	src/main.c
)

include_directories(${PROJECT_SOURCE_DIR}/include)

# pull in common dependencies
target_link_libraries(main
	pico_stdlib
	hardware_uart
	hardware_dma
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(main pico_cyw43_arch_none)
endif()

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
# create map/bin/hex file etc.
pico_add_extra_outputs(main)
