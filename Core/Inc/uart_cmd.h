/*
 * uart_cmd.h
 *
 *  Created on: Aug 19, 2025
 *      Author: Chard Ethern
 */


#ifndef __UART_CMD_H
#define __UART_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "dshot.h" // Include dshot.h to get MOTORS_COUNT
#include "dshot_A.h"




#define UART_RX2_BUFFER_SIZE 128

extern volatile float pid_target_speed_rpms[MOTORS_COUNT];

extern uint8_t uart_rx2_buffer[UART_RX2_BUFFER_SIZE];
extern volatile uint16_t uart_rx2_write_pos;
extern volatile bool uart_new_data_available;

void UART_CMD_Init(UART_HandleTypeDef *huart);
void process_uart_command(void);
void Debug_Send_DMA(const char* format, ...);
extern volatile bool uart_tx2_busy;

#ifdef __cplusplus
}
#endif

#endif /* __UART_ONLY_CMD_H */
