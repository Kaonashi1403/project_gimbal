cmake_minimum_required(VERSION 3.13)
add_library(pid STATIC pid.cpp)

target_include_directories(pid PUBLIC include)

target_link_libraries(pid PUBLIC
    pico_stdlib
    hardware_spi
    hardware_gpio
)