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

typedef struct {
    uint16_t vbat1_adc;
    uint16_t vbat2_adc;
    bool killswitch_state;
} BatteryData_t;


#endif // BYTEPROTOCOL_TX_H
