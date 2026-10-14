cmake_minimum_required(VERSION 3.13)
add_library(mpu STATIC mpu.c)

target_include_directories(mpu PUBLIC include)

target_link_libraries(mpu PUBLIC
    pico_stdlib
    hardware_spi
    hardware_gpio
)