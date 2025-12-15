/*
 * status_tx.c
 *
 *  Created on: Dec 14, 2025
 *      Author: Chard Ethern
 */
#include "byteProtocol_tx.h"
#include "main.h"
#include <string.h>


extern UART_HandleTypeDef huart1;

static uint8_t tx_buffer[7];
volatile bool debug_tx_busy = false;

void ByteProtocol_TX_Init(void)
{
    debug_tx_busy = false;
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

void ByteProtocol_TX_SendBatteryData(uint16_t vbat1_adc, uint16_t vbat2_adc, bool killswitch_state)
{
    if (debug_tx_busy) return;

    tx_buffer[0] = TX_HEADER_1;
    tx_buffer[1] = TX_HEADER_2;
    tx_buffer[2] = vbat1_adc & 0xFF;
    tx_buffer[3] = (vbat1_adc >> 8) & 0xFF;
    tx_buffer[4] = vbat2_adc & 0xFF;
    tx_buffer[5] = (vbat2_adc >> 8) & 0xFF;
    tx_buffer[6] = killswitch_state ? 0x01 : 0x00;

    debug_tx_busy = true;
    HAL_UART_Transmit_DMA(&huart1, tx_buffer, sizeof(tx_buffer));
}

