/*
 * status_tx.h
 *
 *  Created on: Dec 14, 2025
 *      Author: Chard Ethern
 */

#ifndef STATUS_TX_H
#define STATUS_TX_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== PACKET STRUCTURE ==================== */
// Single 8-byte packet containing everything
// Format: [0xFF][0xFD][BAT1][BAT2][KILL][ERRORS][STATUS][UNUSED]
#define STATUS_HEADER_1     0xFF
#define STATUS_HEADER_2     0xFD
#define STATUS_PACKET_SIZE  8

/* ==================== ERROR FLAGS BITMASK ==================== */
// Byte 5: Error flags (PC14 LED will set bit 3)
#define ERROR_NONE          0x00
#define ERROR_PC14_LED      0x08  // Bit 3: PC14 LED is ON (your indicator)
// Other bits available for future use:
// #define ERROR_UART_OVERRUN  0x01  // Bit 0
// #define ERROR_MOTOR_COMM    0x02  // Bit 1
// #define ERROR_BATTERY_LOW   0x04  // Bit 2
// #define ERROR_OTHER         0x10  // Bit 4
// #define ERROR_OTHER2        0x20  // Bit 5
// #define ERROR_OTHER3        0x40  // Bit 6
// #define ERROR_OTHER4        0x80  // Bit 7

/* ==================== SYSTEM STATUS ==================== */
// Byte 6: System status codes
#define STATUS_BOOTING      0
#define STATUS_CALIBRATING  1
#define STATUS_READY        2
#define STATUS_RUNNING      3
#define STATUS_ERROR        4

/* ==================== PUBLIC API ==================== */
void StatusTx_Init(UART_HandleTypeDef *huart);
void StatusTx_SendPacket(void);
void StatusTx_UpdateAll(float bat1_volts, float bat2_volts, uint8_t killswitch_state);

// Direct setters (optional)
void StatusTx_SetBattery1(float voltage);
void StatusTx_SetBattery2(float voltage);
void StatusTx_SetKillswitch(uint8_t state);
void StatusTx_SetErrorFlag(uint8_t flag);
void StatusTx_ClearErrorFlag(uint8_t flag);
void StatusTx_SetSystemStatus(uint8_t status);

extern volatile bool debug_tx_busy;

#endif /* STATUS_TX_H */
