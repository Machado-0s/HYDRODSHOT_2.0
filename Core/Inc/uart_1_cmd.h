/*
 * uart_1_cmd.h
 *
 *  Created on: Nov 28, 2025
 *      Author: Chard Ethern
 */

#ifndef INC_UART_1_CMD_H_
#define INC_UART_1_CMD_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "dshot.h"
#include "dshot_A.h"

#define UART_1_RX_BUFFER_SIZE 128

extern volatile float pid_target_speed_rpms[MOTORS_COUNT];

extern uint8_t uart_1_rx_buffer[UART_1_RX_BUFFER_SIZE];
extern volatile uint16_t uart_1_rx_write_pos;
extern volatile bool uart_1_new_data_available;
//extern volatile float received_1_numeric_value;


void UART_1_CMD_Init(UART_HandleTypeDef *huart);
void process_uart_1_command(void);
void Debug_Send_DMA_1(const char* format, ...);
extern volatile bool uart_1_tx_busy;

#ifdef __cplusplus
}
#endif

#endif /* INC_UART_1_CMD_H_ */
