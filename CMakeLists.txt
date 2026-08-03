cmake_minimum_required(VERSION 3.16)
project(motor_data_producer LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

include_directories(../../rpi-gpio/resmgr/public)

add_executable(motor_data_producer
    motor_controller.c config.c cJSON.c rpi_gpio.c rpi_spi.c
)
target_link_libraries(motor_data_producer PRIVATE m)
