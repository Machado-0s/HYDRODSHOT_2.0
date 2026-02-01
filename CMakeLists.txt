cmake_minimum_required(VERSION 3.10)

project(HydroDSHOT VERSION 2.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(HYDROLIB_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

set(MCU_FAMILY F4)

add_subdirectory(Drivers/Hydrodrivers)
add_subdirectory(Drivers/STM32F4xx_HAL_Driver)

add_subdirectory(Core/Hw)
add_subdirectory(Core)
