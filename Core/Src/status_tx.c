/*
 * status_tx.c
 *
 *  Created on: Dec 14, 2025
 *      Author: Chard Ethern
 */

#include "status_tx.h"
#include <string.h>
#include <string.h>
#include <stdbool.h>

/* ==================== PRIVATE VARIABLES ==================== */
static UART_HandleTypeDef *status_huart = NULL;

// Single packet containing all status
static uint8_t status_packet[STATUS_PACKET_SIZE] = {
    STATUS_HEADER_1,    // Byte 0: Header 1
    STATUS_HEADER_2,    // Byte 1: Header 2
    0,                  // Byte 2: Battery 1 (0-255 = 0-25.5V)
    0,                  // Byte 3: Battery 2
    0,                  // Byte 4: Killswitch (0=OFF, 1=ON)
    0,                  // Byte 5: Error flags
    0,                  // Byte 6: System status
    0                   // Byte 7: Unused (reserved)
};

/* ==================== INITIALIZATION ==================== */
void StatusTx_Init(UART_HandleTypeDef *huart)
{
    status_huart = huart;
    debug_tx_busy = false;

    // Initialize packet
    status_packet[0] = STATUS_HEADER_1;
    status_packet[1] = STATUS_HEADER_2;
    status_packet[2] = 0;  // Battery 1
    status_packet[3] = 0;  // Battery 2
    status_packet[4] = 0;  // Killswitch OFF
    status_packet[5] = 0;  // No errors
    status_packet[6] = STATUS_BOOTING;
    status_packet[7] = 0;  // Reserved
}

/* ==================== COMPLETE UPDATE ==================== */
void StatusTx_UpdateAll(float bat1_volts, float bat2_volts, uint8_t killswitch_state)
{
    // Battery 1: Convert voltage to byte (0-255 = 0-25.5V, 0.1V resolution)
    uint8_t bat1_byte = (uint8_t)(bat1_volts * 10.0f);
    if (bat1_byte > 255) bat1_byte = 255;
    status_packet[2] = bat1_byte;

    // Battery 2
    uint8_t bat2_byte = (uint8_t)(bat2_volts * 10.0f);
    if (bat2_byte > 255) bat2_byte = 255;
    status_packet[3] = bat2_byte;

    // Killswitch
    status_packet[4] = (killswitch_state != 0) ? 1 : 0;

    // Error flags (PC14 LED state will be updated separately)
    // Byte 5 already contains error flags from other functions

    // System status (set separately if needed)
    // Byte 6 already contains system status
}

/* ==================== INDIVIDUAL SETTERS ==================== */
void StatusTx_SetBattery1(float voltage)
{
    uint8_t byte_val = (uint8_t)(voltage * 10.0f);
    if (byte_val > 255) byte_val = 255;
    status_packet[2] = byte_val;
}

void StatusTx_SetBattery2(float voltage)
{
    uint8_t byte_val = (uint8_t)(voltage * 10.0f);
    if (byte_val > 255) byte_val = 255;
    status_packet[3] = byte_val;
}

void StatusTx_SetKillswitch(uint8_t state)
{
    status_packet[4] = (state != 0) ? 1 : 0;
}

void StatusTx_SetErrorFlag(uint8_t flag)
{
    status_packet[5] |= flag;
}

void StatusTx_ClearErrorFlag(uint8_t flag)
{
    status_packet[5] &= ~flag;
}

void StatusTx_SetSystemStatus(uint8_t status)
{
    if (status <= STATUS_ERROR) {
        status_packet[6] = status;
    }
}

/* ==================== TRANSMIT SINGLE PACKET ==================== */
void StatusTx_SendPacket(void)
{
    if (!status_huart || debug_tx_busy) {
        return;
    }

    debug_tx_busy = true;
    HAL_UART_Transmit_IT(status_huart, status_packet, STATUS_PACKET_SIZE);
}


