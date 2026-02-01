cmake_minimum_required(VERSION 3.10)

project(HydroDSHOT VERSION 2.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(HYDROLIB_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

set(MCU_FAMILY F4)

add_subdirectory(Drivers)
add_subdirectory(Core)

add_custom_target(compdb ALL
    COMMAND /bin/sh -c "compdb -p ${CMAKE_BINARY_DIR} list > compile_commands.json && mv compile_commands.json ${CMAKE_BINARY_DIR}/compile_commands.json"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    VERBATIM
)