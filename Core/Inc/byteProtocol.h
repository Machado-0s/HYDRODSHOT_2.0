#ifndef BYTE_PROTOCOL_H
#define BYTE_PROTOCOL_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Headers */
#define HEADER_BYTE_1         0xFF
#define HEADER_BYTE_2         0xFD

#define HEADER_SIZE           2
#define TOTAL_MOTOR_COUNT     14
#define DATA_SIZE             TOTAL_MOTOR_COUNT
#define CRC_SIZE              1

/* Total Packet: 2 Headers + 14 Data + 1 CRC = 17 Bytes */
#define PACKET_SIZE           (HEADER_SIZE + DATA_SIZE + CRC_SIZE)

#define DSHOT_MAX_RPM         6000
#define RX_BUFFER_SIZE        128
#define RING_BUFFER_SIZE      256

extern volatile bool debug_tx_busy;

/* Function Prototypes */
void ByteProtocol_Init(void);
void ByteProtocol_DShotUpdateInt(uint8_t motor_idx, int32_t rpm);
void ByteProtocol_PWMUpdate(uint8_t pwm_idx, uint16_t pulse_us);
uint8_t Calculate_CRC8(const uint8_t *data, uint16_t length);


#endif
