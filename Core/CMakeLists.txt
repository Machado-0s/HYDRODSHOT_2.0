set(TARGET_NAME HydrvDshot)

add_library(core_headers 
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/byteProtocol_tx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/byteProtocol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/dshot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/dshot_A.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/status_tx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/uart_1_cmd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/uart_cmd.c
    )
    


add_executable(${TARGET_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/Src/main.c)
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Inc)
target_link_libraries(${TARGET_NAME} Periph core_headers)


target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)