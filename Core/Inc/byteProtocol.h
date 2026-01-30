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

#define MAGIC_BYTE            0x00
#define HEADER_SIZE           2
#define TOTAL_MOTOR_COUNT     14
#define DATA_SIZE             TOTAL_MOTOR_COUNT
#define CRC_SIZE              1


#define PACKET_SIZE           (HEADER_SIZE + DATA_SIZE + CRC_SIZE)
#define MAX_SCAN_BYTES        256

#define DSHOT_MIN_RPM        -6000
#define DSHOT_MAX_RPM         6000
#define RX_BUFFER_SIZE        128

/* Function Prototypes */
void ByteProtocol_Init(void);
void ByteProtocol_IdleLineCallback(UART_HandleTypeDef *huart);
void ByteProtocol_DShotUpdateInt(uint8_t motor_idx, int32_t rpm);
void ByteProtocol_PWMUpdate(uint8_t pwm_idx, uint16_t pulse_us);

/* Utility Functions */
void COBS_Decode(uint8_t *data, uint16_t length);
uint8_t Calculate_CRC8(const uint8_t *data, uint16_t length);
void Debug_Print(const char* format, ...);

extern volatile bool debug_tx_busy;

#endif /* BYTE_PROTOCOL_H */
