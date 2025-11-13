/*
 * dshot_A.h
 *
 *  Created on: Nov 7, 2025
 *      Author: Chard Ethern
 */

#ifndef DSHOT_A_H
#define DSHOT_A_H

// Constants
#define TIMER_CLK                        168000000
#define MOTORS_COUNT                     10
#define MOTOR_POLES_NUMBER               14

#define DSHOT_MODE                       600

#define DSHOT_BB_FRAME_LENGTH            (TIMER_CLK / (DSHOT_MODE * 1000))
#define DSHOT_BB_FRAME_SECTIONS          8
#define DSHOT_BB_SECTION_LENGTH          (DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS)
#define DSHOT_BB_1_LENGTH                ((DSHOT_BB_FRAME_SECTIONS * 3) / 4)  // 75%
#define DSHOT_BB_0_LENGTH                ((DSHOT_BB_FRAME_SECTIONS * 2) / 4)  // 50%
#define DSHOT_BB_BUFFER_BITS             BITS_PER_FRAME
#define DSHOT_BB_BUFFER_LENGTH           (DSHOT_BB_BUFFER_BITS * DSHOT_BB_FRAME_SECTIONS + 2)
#define BITS_PER_FRAME                   16

#define BDSHOT_RESPONSE_BITRATE          (DSHOT_MODE * 4/3)
#define BDSHOT_RESPONSE_LENGTH           21
#define BDSHOT_RESPONSE_OVERSAMPLING     3

#define DSHOT_BASE_COMMAND               48


#include "dshot.h"
#include <main.h>


// TIM1_UP - DMA2 Stream 5 - PA5, PA6
extern volatile uint32_t dshot_bb_buffer_A[DSHOT_BB_BUFFER_LENGTH];
extern volatile uint32_t dshot_bb_buffer_rx_A[(int)(31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
extern bool bdshot_reception_A;

void arm_bdshot_rx_capture_A(void);
void dshot_A_tx_configuration(void);
void dshot_A_rx_processing(void);

// TIM1_CH4/TRIG/COM - DMA2 Stream 4 - PB0, PB10
extern volatile uint32_t dshot_bb_buffer_B[DSHOT_BB_BUFFER_LENGTH];
extern volatile uint32_t dshot_bb_buffer_rx_B[(int)(31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
extern bool bdshot_reception_B;

void arm_bdshot_rx_capture_B(void);
void dshot_B_tx_configuration(void);
void dshot_B_rx_processing(void);

// TIM1_CH1 - DMA2 Stream 3 - PC8, PC6
extern volatile uint32_t dshot_bb_buffer_C[DSHOT_BB_BUFFER_LENGTH];
extern volatile uint32_t dshot_bb_buffer_rx_C[(int)(31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
extern bool bdshot_reception_C;

void arm_bdshot_rx_capture_C(void);
void dshot_C_tx_configuration(void);
void dshot_C_rx_processing(void);

// TIM1_CH3 - DMA2 Stream 6 - PD12, PD14
extern volatile uint32_t dshot_bb_buffer_D[DSHOT_BB_BUFFER_LENGTH];
extern volatile uint32_t dshot_bb_buffer_rx_D[(int)(31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
extern bool bdshot_reception_D;

void arm_bdshot_rx_capture_D(void);
void dshot_D_tx_configuration(void);
void dshot_D_rx_processing(void);

// TIM8_UP - DMA2 Stream 1 - PE9, PE13
extern volatile uint32_t dshot_bb_buffer_E[DSHOT_BB_BUFFER_LENGTH];
extern volatile uint32_t dshot_bb_buffer_rx_E[(int)(31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
extern bool bdshot_reception_E;

void arm_bdshot_rx_capture_E(void);
void dshot_E_tx_configuration(void);
void dshot_E_rx_processing(void);

extern volatile uint8_t tim1_rx_done_count;

#endif // DSHOT_A_H
