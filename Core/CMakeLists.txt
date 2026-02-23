
set(LIBRARY_NAME Periph)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(${LIBRARY_NAME} 
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/system_stm32f4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/dshot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/dshot_A.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/pwm.c  
    )
target_include_directories(${LIBRARY_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
target_link_libraries(${LIBRARY_NAME} HAL_drivers)



set(TARGET_NAME HydrvDshot)
    
add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/Src/main.cpp)
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Inc)
target_link_libraries(${TARGET_NAME} Periph)
target_link_libraries(${TARGET_NAME}
    HydrodriversClockCPP HydrodriversUART HydrodriversGPIO
    HydrolibBusDatalink HydrolibBusApplication)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)