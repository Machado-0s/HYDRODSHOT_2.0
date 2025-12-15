/*
 * byteProtocol.h
 *
 *  Created on: Dec 8, 2025
 *      Author: Chard Ethern
 */

#ifndef BYTE_PROTOCOL_H
#define BYTE_PROTOCOL_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>


#define PROTOCOL_HEADER_1     0xFF
#define PROTOCOL_HEADER_2     0xFD
#define HEADER_SIZE           2
#define TOTAL_MOTOR_COUNT     14
#define PACKET_SIZE           (HEADER_SIZE + TOTAL_MOTOR_COUNT)

#define DSHOT_MIN_RPM        -6000
#define DSHOT_MAX_RPM         6000

#define PWM_MIN_USEC          1000
#define PWM_MAX_USEC          2000

#define PWM_MOTOR_COUNT       4

#define RX_BUFFER_SIZE        64

extern volatile bool debug_tx_busy;

void ByteProtocol_Init(void);
void ByteProtocol_Process(void);
void ByteProtocol_RxCpltCallback(UART_HandleTypeDef *huart);
void ByteProtocol_IdleLineCallback(UART_HandleTypeDef *huart);

extern void ByteProtocol_DShotUpdate(uint8_t motor_idx, float rpm);
extern void ByteProtocol_DShotUpdateInt(uint8_t motor_idx, int32_t rpm);
extern void ByteProtocol_PWMUpdate(uint8_t pwm_idx, uint16_t pulse_us);

void Debug_Print(const char* format, ...);

#endif
