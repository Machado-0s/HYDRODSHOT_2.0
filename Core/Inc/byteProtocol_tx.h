/*
 * status_tx.h
 *
 *  Created on: Dec 14, 2025
 *      Author: Chard Ethern
 */

#ifndef BYTEPROTOCOL_TX_H
#define BYTEPROTOCOL_TX_H

#include <stdint.h>
#include <stdbool.h>
#include "byteProtocol.h"

#define TX_HEADER_1 0xFF
#define TX_HEADER_2 0xFD

typedef struct {
    uint16_t vbat1_adc;
    uint16_t vbat2_adc;
    bool killswitch_state;
} BatteryData_t;

extern volatile bool debug_tx_busy;

void ByteProtocol_TX_Init(void);
void ByteProtocol_TX_SendBatteryData(uint16_t vbat1_adc, uint16_t vbat2_adc, bool killswitch_state);

#endif // BYTEPROTOCOL_TX_H
