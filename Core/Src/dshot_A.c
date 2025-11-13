/*
 * dshot_A.c
 *
 *  Created on: Nov 7, 2025
 *      Author: Chard Ethern
 */

#include "dshot_A.h"
#include "dshot.h"
#include <string.h>
#include <stdio.h>
#include "uart_cmd.h"
#include "math.h"

// TIM1_UP - DMA2 Stream 5 - PA5, PA6
volatile uint32_t dshot_bb_buffer_A[DSHOT_BB_BUFFER_LENGTH] __attribute__((aligned(4)));
volatile uint32_t dshot_bb_buffer_rx_A[(int)(31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
bool bdshot_reception_A = true;

static inline uint32_t get_port_A_mask(void)
{
    uint32_t mask = 0;
    for (uint8_t m = 0; m < MOTORS_COUNT; m++) {
        if (motor_gpio_port[m] == 0 && (motor_gpio_pin_numbers[m] == 5 || motor_gpio_pin_numbers[m] == 6)) {
            mask |= (1u << motor_gpio_pin_numbers[m]);
        }
    }
    return mask;
}

void arm_bdshot_rx_capture_A(void)
{
    const uint32_t pins_mask = (GPIO_MODER_MODER5 | GPIO_MODER_MODER6);
    const uint32_t pupdr_clear_mask = ((3U << (5*2)) | (3U << (6*2)));
    const uint32_t pupdr_pullup_mask = ((1U << (5*2)) | (1U << (6*2)));

    GPIOA->MODER &= ~pins_mask;
    GPIOA->PUPDR &= ~pupdr_clear_mask;
    GPIOA->PUPDR |= pupdr_pullup_mask;

    uint32_t sample_period = (DSHOT_BB_FRAME_LENGTH * DSHOT_MODE) / (BDSHOT_RESPONSE_BITRATE * BDSHOT_RESPONSE_OVERSAMPLING);
    if (sample_period == 0) sample_period = 1;

    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->ARR = sample_period - 1;
    TIM1->CCR1 = sample_period/2;
    TIM1->EGR |= TIM_EGR_UG;

    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream5->CR & DMA_SxCR_EN) { __NOP(); }

    DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                  DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;

    DMA2_Stream5->PAR = (uint32_t)(&GPIOA->IDR);
    DMA2_Stream5->M0AR = (uint32_t)dshot_bb_buffer_rx_A;

    uint32_t ndtr = (uint32_t)((31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING);
    if (ndtr > 0xFFFF) ndtr = 0xFFFF;
    DMA2_Stream5->NDTR = ndtr;

    uint32_t cr_val = (6 << DMA_SxCR_CHSEL_Pos)
                    | DMA_SxCR_PL
                    | DMA_SxCR_MSIZE_1
                    | DMA_SxCR_PSIZE_1
                    | DMA_SxCR_MINC
                    //| DMA_SxCR_DIR_0
                    | DMA_SxCR_TCIE;
    DMA2_Stream5->CR = cr_val;

    __DSB(); __ISB();
    DMA2_Stream5->CR |= DMA_SxCR_EN;
    TIM1->CNT = 0;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void dshot_A_tx_configuration(void)
{
    GPIOA->MODER |= (GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0);
    GPIOA->MODER &= ~(GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR5_0 | GPIO_PUPDR_PUPDR6_0);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5_1 | GPIO_PUPDR_PUPDR6_1);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6);

    TIM1->CR1 &= ~TIM_CR1_CEN;
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream5->CR & DMA_SxCR_EN) { __NOP(); }
    DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;

    DMA2_Stream5->PAR = (uint32_t)(&GPIOA->BSRR);
    DMA2_Stream5->M0AR = (uint32_t)(dshot_bb_buffer_A);
    DMA2_Stream5->NDTR = DSHOT_BB_BUFFER_LENGTH;

    uint32_t cr_val_A = (6 << DMA_SxCR_CHSEL_Pos)
                      | DMA_SxCR_PL
                      | DMA_SxCR_MSIZE_1
                      | DMA_SxCR_PSIZE_1
                      | DMA_SxCR_MINC
                      //| DMA_SxCR_DIR_0
                      | DMA_SxCR_TCIE;
    DMA2_Stream5->CR = cr_val_A | DMA_SxCR_DIR_0;
}

void dshot_A_rx_processing(void)
{
    for (int m = 0; m < MOTORS_COUNT; m++) {
        if (motor_gpio_port[m] == 0 && (motor_gpio_pin_numbers[m] == 5 || motor_gpio_pin_numbers[m] == 6)) {
            uint8_t pin = motor_gpio_pin_numbers[m];
            uint32_t decoded_gcr_value = get_BDshot_response((uint32_t*)dshot_bb_buffer_rx_A, pin);
            read_BDshot_response(decoded_gcr_value, m);
        }
    }
}



// TIM1_CH4/TRIG/COM - DMA2 Stream 4 - PB0, PB10
volatile uint32_t dshot_bb_buffer_B[DSHOT_BB_BUFFER_LENGTH] __attribute__((aligned(4)));
volatile uint32_t dshot_bb_buffer_rx_B[(int)(31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
bool bdshot_reception_B = true;

static inline uint32_t get_port_B_mask(void)
{
    uint32_t mask = 0;
    for (uint8_t m = 0; m < MOTORS_COUNT; m++) {
        if (motor_gpio_port[m] == 1 && (motor_gpio_pin_numbers[m] == 0 || motor_gpio_pin_numbers[m] == 10)) {
            mask |= (1u << motor_gpio_pin_numbers[m]);
        }
    }
    return mask;
}

void arm_bdshot_rx_capture_B(void)
{
    const uint32_t pins_mask = (GPIO_MODER_MODER0 | GPIO_MODER_MODER10);
    const uint32_t pupdr_clear_mask = ((3U << (0*2)) | (3U << (10*2)));
    const uint32_t pupdr_pullup_mask = ((1U << (0*2)) | (1U << (10*2)));

    GPIOB->MODER &= ~pins_mask;
    GPIOB->PUPDR &= ~pupdr_clear_mask;
    GPIOB->PUPDR |= pupdr_pullup_mask;

    uint32_t sample_period = (DSHOT_BB_FRAME_LENGTH * DSHOT_MODE) / (BDSHOT_RESPONSE_BITRATE * BDSHOT_RESPONSE_OVERSAMPLING);
    if (sample_period == 0) sample_period = 1;

    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->ARR = sample_period - 1;
    TIM1->CCR4 = sample_period/2;
    TIM1->EGR |= TIM_EGR_UG;

    DMA2_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream4->CR & DMA_SxCR_EN) { __NOP(); }

    DMA2->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 |
                  DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;

    DMA2_Stream4->PAR = (uint32_t)(&GPIOB->IDR);
    DMA2_Stream4->M0AR = (uint32_t)dshot_bb_buffer_rx_B;

    uint32_t ndtr = (uint32_t)((31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING);
    if (ndtr > 0xFFFF) ndtr = 0xFFFF;
    DMA2_Stream4->NDTR = ndtr;

    uint32_t cr_val = (6 << DMA_SxCR_CHSEL_Pos)
                    | DMA_SxCR_PL
                    | DMA_SxCR_MSIZE_1
                    | DMA_SxCR_PSIZE_1
                    | DMA_SxCR_MINC
                   // | DMA_SxCR_DIR_0
                    | DMA_SxCR_TCIE;
    DMA2_Stream4->CR = cr_val;

    __DSB(); __ISB();
    DMA2_Stream4->CR |= DMA_SxCR_EN;
    TIM1->CNT = 0;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void dshot_B_tx_configuration(void)
{
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER10_0);
    GPIOB->MODER &= ~(GPIO_MODER_MODER0_1 | GPIO_MODER_MODER10_1);
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT10);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR10_0);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR10_1);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR10);

    DMA2_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream4->CR & DMA_SxCR_EN) { __NOP(); }
    DMA2->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;

    DMA2_Stream4->PAR = (uint32_t)(&GPIOB->BSRR);
    DMA2_Stream4->M0AR = (uint32_t)(dshot_bb_buffer_B);
    DMA2_Stream4->NDTR = DSHOT_BB_BUFFER_LENGTH;

    uint32_t cr_val_B = (6 << DMA_SxCR_CHSEL_Pos)
                      | DMA_SxCR_PL
                      | DMA_SxCR_MSIZE_1
                      | DMA_SxCR_PSIZE_1
                      | DMA_SxCR_MINC
                     /// | DMA_SxCR_DIR_0
                      | DMA_SxCR_TCIE;
    DMA2_Stream4->CR = cr_val_B | DMA_SxCR_DIR_0;
}

void dshot_B_rx_processing(void)
{
    for (int m = 0; m < MOTORS_COUNT; m++) {
        if (motor_gpio_port[m] == 1 && (motor_gpio_pin_numbers[m] == 0 || motor_gpio_pin_numbers[m] == 10)) {
            uint8_t pin = motor_gpio_pin_numbers[m];
            uint32_t decoded_gcr_value = get_BDshot_response((uint32_t*)dshot_bb_buffer_rx_B, pin);
            read_BDshot_response(decoded_gcr_value, m);
        }
    }
}


// TIM1_CH1 - DMA2 Stream 3 - PC8, PC6
volatile uint32_t dshot_bb_buffer_C[DSHOT_BB_BUFFER_LENGTH] __attribute__((aligned(4)));
volatile uint32_t dshot_bb_buffer_rx_C[(int)(31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
bool bdshot_reception_C = true;

static inline uint32_t get_port_C_mask(void)
{
    uint32_t mask = 0;
    for (uint8_t m = 0; m < MOTORS_COUNT; m++) {
        if (motor_gpio_port[m] == 2 && (motor_gpio_pin_numbers[m] == 6 || motor_gpio_pin_numbers[m] == 8)) {
            mask |= (1u << motor_gpio_pin_numbers[m]);
        }
    }
    return mask;
}

void arm_bdshot_rx_capture_C(void)
{
    const uint32_t pins_mask = (GPIO_MODER_MODER6 | GPIO_MODER_MODER8);
    const uint32_t pupdr_clear_mask = ((3U << (6*2)) | (3U << (8*2)));
    const uint32_t pupdr_pullup_mask = ((1U << (6*2)) | (1U << (8*2)));

    GPIOC->MODER &= ~pins_mask;
    GPIOC->PUPDR &= ~pupdr_clear_mask;
    GPIOC->PUPDR |= pupdr_pullup_mask;

    uint32_t sample_period = (DSHOT_BB_FRAME_LENGTH * DSHOT_MODE) / (BDSHOT_RESPONSE_BITRATE * BDSHOT_RESPONSE_OVERSAMPLING);
    if (sample_period == 0) sample_period = 1;

    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->ARR = sample_period - 1;
    TIM1->CCR1 = sample_period/2;
    TIM1->EGR |= TIM_EGR_UG;

    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream3->CR & DMA_SxCR_EN) { __NOP(); }

    DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
                  DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;

    DMA2_Stream3->PAR = (uint32_t)(&GPIOC->IDR);
    DMA2_Stream3->M0AR = (uint32_t)dshot_bb_buffer_rx_C;

    uint32_t ndtr = (uint32_t)((31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING);
    if (ndtr > 0xFFFF) ndtr = 0xFFFF;
    DMA2_Stream3->NDTR = ndtr;

    uint32_t cr_val = (6 << DMA_SxCR_CHSEL_Pos)
                    | DMA_SxCR_PL
                    | DMA_SxCR_MSIZE_1
                    | DMA_SxCR_PSIZE_1
                    | DMA_SxCR_MINC
                   // | DMA_SxCR_DIR_0
                    | DMA_SxCR_TCIE;
    DMA2_Stream3->CR = cr_val;

    __DSB(); __ISB();
    DMA2_Stream3->CR |= DMA_SxCR_EN;
    TIM1->CNT = 0;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void dshot_C_tx_configuration(void)
{
    GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER8_0);
    GPIOC->MODER &= ~(GPIO_MODER_MODER6_1 | GPIO_MODER_MODER8_1);
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT6 | GPIO_OTYPER_OT8);
    GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR8_0);
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_1 | GPIO_PUPDR_PUPDR8_1);
    GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR8);

    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream3->CR & DMA_SxCR_EN) { __NOP(); }
    DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;

    DMA2_Stream3->PAR = (uint32_t)(&GPIOC->BSRR);
    DMA2_Stream3->M0AR = (uint32_t)(dshot_bb_buffer_C);
    DMA2_Stream3->NDTR = DSHOT_BB_BUFFER_LENGTH;

    uint32_t cr_val_C = (6 << DMA_SxCR_CHSEL_Pos)
                      | DMA_SxCR_PL
                      | DMA_SxCR_MSIZE_1
                      | DMA_SxCR_PSIZE_1
                      | DMA_SxCR_MINC
                      //| DMA_SxCR_DIR_0
                      | DMA_SxCR_TCIE;
    DMA2_Stream3->CR = cr_val_C | DMA_SxCR_DIR_0;
}

void dshot_C_rx_processing(void)
{
    for (int m = 0; m < MOTORS_COUNT; m++) {
        if (motor_gpio_port[m] == 2 && (motor_gpio_pin_numbers[m] == 6 || motor_gpio_pin_numbers[m] == 8)) {
            uint8_t pin = motor_gpio_pin_numbers[m];
            uint32_t decoded_gcr_value = get_BDshot_response((uint32_t*)dshot_bb_buffer_rx_C, pin);
            read_BDshot_response(decoded_gcr_value, m);
        }
    }
}


// TIM1_CH3 - DMA2 Stream 6 - PD12, PD14
volatile uint32_t dshot_bb_buffer_D[DSHOT_BB_BUFFER_LENGTH] __attribute__((aligned(4)));
volatile uint32_t dshot_bb_buffer_rx_D[(int)(31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
bool bdshot_reception_D = true;

static inline uint32_t get_port_D_mask(void)
{
    uint32_t mask = 0;
    for (uint8_t m = 0; m < MOTORS_COUNT; m++) {
        if (motor_gpio_port[m] == 3 && (motor_gpio_pin_numbers[m] == 12 || motor_gpio_pin_numbers[m] == 14)) {
            mask |= (1u << motor_gpio_pin_numbers[m]);
        }
    }
    return mask;
}

void arm_bdshot_rx_capture_D(void)
{
    const uint32_t pins_mask = (GPIO_MODER_MODER12 | GPIO_MODER_MODER14);
    const uint32_t pupdr_clear_mask = ((3U << (12*2)) | (3U << (14*2)));
    const uint32_t pupdr_pullup_mask = ((1U << (12*2)) | (1U << (14*2)));

    GPIOD->MODER &= ~pins_mask;
    GPIOD->PUPDR &= ~pupdr_clear_mask;
    GPIOD->PUPDR |= pupdr_pullup_mask;

    uint32_t sample_period = (DSHOT_BB_FRAME_LENGTH * DSHOT_MODE) / (BDSHOT_RESPONSE_BITRATE * BDSHOT_RESPONSE_OVERSAMPLING);
    if (sample_period == 0) sample_period = 1;

    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->ARR = sample_period - 1;
    TIM1->CCR3 = sample_period/2;
    TIM1->EGR |= TIM_EGR_UG;

    DMA2_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream6->CR & DMA_SxCR_EN) { __NOP(); }

    DMA2->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |
                  DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;

    DMA2_Stream6->PAR = (uint32_t)(&GPIOD->IDR);
    DMA2_Stream6->M0AR = (uint32_t)dshot_bb_buffer_rx_D;

    uint32_t ndtr = (uint32_t)((31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING);
    if (ndtr > 0xFFFF) ndtr = 0xFFFF;
    DMA2_Stream6->NDTR = ndtr;

    uint32_t cr_val = (6 << DMA_SxCR_CHSEL_Pos)
                    | DMA_SxCR_PL
                    | DMA_SxCR_MSIZE_1
                    | DMA_SxCR_PSIZE_1
                    | DMA_SxCR_MINC
                    //| DMA_SxCR_DIR_0
                    | DMA_SxCR_TCIE;
    DMA2_Stream6->CR = cr_val;

    __DSB(); __ISB();
    DMA2_Stream6->CR |= DMA_SxCR_EN;
    TIM1->CNT = 0;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void dshot_D_tx_configuration(void)
{
    GPIOD->MODER |= (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER14_0);
    GPIOD->MODER &= ~(GPIO_MODER_MODER12_1 | GPIO_MODER_MODER14_1);
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT12 | GPIO_OTYPER_OT14);
    GPIOD->PUPDR |= (GPIO_PUPDR_PUPDR12_0 | GPIO_PUPDR_PUPDR14_0);
    GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPDR12_1 | GPIO_PUPDR_PUPDR14_1);
    GPIOD->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR14);

    DMA2_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream6->CR & DMA_SxCR_EN) { __NOP(); }
    DMA2->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;

    DMA2_Stream6->PAR = (uint32_t)(&GPIOD->BSRR);
    DMA2_Stream6->M0AR = (uint32_t)(dshot_bb_buffer_D);
    DMA2_Stream6->NDTR = DSHOT_BB_BUFFER_LENGTH;

    uint32_t cr_val_D = (6 << DMA_SxCR_CHSEL_Pos)
                      | DMA_SxCR_PL
                      | DMA_SxCR_MSIZE_1
                      | DMA_SxCR_PSIZE_1
                      | DMA_SxCR_MINC
                      //| DMA_SxCR_DIR_0
                      | DMA_SxCR_TCIE;
    DMA2_Stream6->CR = cr_val_D | DMA_SxCR_DIR_0;
}

void dshot_D_rx_processing(void)
{
    for (int m = 0; m < MOTORS_COUNT; m++) {
        if (motor_gpio_port[m] == 3 && (motor_gpio_pin_numbers[m] == 12 || motor_gpio_pin_numbers[m] == 14)) {
            uint8_t pin = motor_gpio_pin_numbers[m];
            uint32_t decoded_gcr_value = get_BDshot_response((uint32_t*)dshot_bb_buffer_rx_D, pin);
            read_BDshot_response(decoded_gcr_value, m);
        }
    }
}



// TIM8_UP - DMA2 Stream 1 - PE9, PE13
volatile uint32_t dshot_bb_buffer_E[DSHOT_BB_BUFFER_LENGTH] __attribute__((aligned(4)));
volatile uint32_t dshot_bb_buffer_rx_E[(int)(31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
bool bdshot_reception_E = true;

static inline uint32_t get_port_E_mask(void)
{
    uint32_t mask = 0;
    for (uint8_t m = 0; m < MOTORS_COUNT; m++) {
        if (motor_gpio_port[m] == 4 && (motor_gpio_pin_numbers[m] == 9 || motor_gpio_pin_numbers[m] == 13)) {
            mask |= (1u << motor_gpio_pin_numbers[m]);
        }
    }
    return mask;
}

void arm_bdshot_rx_capture_E(void)
{
    const uint32_t pins_mask = (GPIO_MODER_MODER9 | GPIO_MODER_MODER13);
    const uint32_t pupdr_clear_mask = ((3U << (9*2)) | (3U << (13*2)));
    const uint32_t pupdr_pullup_mask = ((1U << (9*2)) | (1U << (13*2)));

    GPIOE->MODER &= ~pins_mask;
    GPIOE->PUPDR &= ~pupdr_clear_mask;
    GPIOE->PUPDR |= pupdr_pullup_mask;

    uint32_t sample_period = (DSHOT_BB_FRAME_LENGTH * DSHOT_MODE) / (BDSHOT_RESPONSE_BITRATE * BDSHOT_RESPONSE_OVERSAMPLING);
    if (sample_period == 0) sample_period = 1;

    TIM8->CR1 &= ~TIM_CR1_CEN;
    TIM8->ARR = sample_period - 1;
    TIM8->EGR |= TIM_EGR_UG;

    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream1->CR & DMA_SxCR_EN) { __NOP(); }

    DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |
                  DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

    DMA2_Stream1->PAR = (uint32_t)(&GPIOE->IDR);
    DMA2_Stream1->M0AR = (uint32_t)dshot_bb_buffer_rx_E;

    uint32_t ndtr = (uint32_t)((31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING);
    if (ndtr > 0xFFFF) ndtr = 0xFFFF;
    DMA2_Stream1->NDTR = ndtr;

    uint32_t cr_val = (7 << DMA_SxCR_CHSEL_Pos)  // Channel 7 for TIM8_UP
                    | DMA_SxCR_PL
                    | DMA_SxCR_MSIZE_1
                    | DMA_SxCR_PSIZE_1
                    | DMA_SxCR_MINC
                    //| DMA_SxCR_DIR_0
                    | DMA_SxCR_TCIE;
    DMA2_Stream1->CR = cr_val;

    __DSB(); __ISB();
    DMA2_Stream1->CR |= DMA_SxCR_EN;
    TIM8->CNT = 0;
    TIM8->CR1 |= TIM_CR1_CEN;
}

void dshot_E_tx_configuration(void)
{
    GPIOE->MODER |= (GPIO_MODER_MODER9_0 | GPIO_MODER_MODER13_0);
    GPIOE->MODER &= ~(GPIO_MODER_MODER9_1 | GPIO_MODER_MODER13_1);
    GPIOE->OTYPER &= ~(GPIO_OTYPER_OT9 | GPIO_OTYPER_OT13);
    GPIOE->PUPDR |= (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR13_0);
    GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPDR9_1 | GPIO_PUPDR_PUPDR13_1);
    GPIOE->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR13);

    TIM8->CR1 &= ~TIM_CR1_CEN;
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream1->CR & DMA_SxCR_EN) { __NOP(); }
    DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

    DMA2_Stream1->PAR = (uint32_t)(&GPIOE->BSRR);
    DMA2_Stream1->M0AR = (uint32_t)(dshot_bb_buffer_E);
    DMA2_Stream1->NDTR = DSHOT_BB_BUFFER_LENGTH;

    uint32_t cr_val_E = (7 << DMA_SxCR_CHSEL_Pos)  // Channel 7 for TIM8_UP
                      | DMA_SxCR_PL
                      | DMA_SxCR_MSIZE_1
                      | DMA_SxCR_PSIZE_1
                      | DMA_SxCR_MINC
                     // | DMA_SxCR_DIR_0
                      | DMA_SxCR_TCIE;
    DMA2_Stream1->CR = cr_val_E | DMA_SxCR_DIR_0;
}

void dshot_E_rx_processing(void)
{
    for (int m = 0; m < MOTORS_COUNT; m++) {
        if (motor_gpio_port[m] == 4 && (motor_gpio_pin_numbers[m] == 9 || motor_gpio_pin_numbers[m] == 13)) {
            uint8_t pin = motor_gpio_pin_numbers[m];
            uint32_t decoded_gcr_value = get_BDshot_response((uint32_t*)dshot_bb_buffer_rx_E, pin);
            read_BDshot_response(decoded_gcr_value, m);
        }
    }
}

volatile uint8_t tim1_rx_done_count = 0;

void DMA2_Stream5_IRQHandler(void)
{
    if (DMA2->HISR & DMA_HISR_TCIF5) {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF5;

        if (bdshot_reception_A) {
            // Tx Completion -> Start Rx
            arm_bdshot_rx_capture_A();

            bdshot_reception_A = false;
        } else {
            // Rx Completion -> Cleanup
            DMA2_Stream5->CR &= ~DMA_SxCR_EN;
            while (DMA2_Stream5->CR & DMA_SxCR_EN) { __NOP(); }
            DMA2->HIFCR |= DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                           DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;

            telemetry_done_flag |= 0x01; // Signal main loop

            __disable_irq();
            tim1_rx_done_count++;
            if (tim1_rx_done_count == 4) {
                TIM1->CR1 &= ~TIM_CR1_CEN; // Stop TIM1 only when all 4 streams are done
                tim1_rx_done_count = 0;
            }
            __enable_irq();

            bdshot_reception_A = true; // Reset state for next Tx cycle
        }
    }

    if (DMA2->HISR & DMA_HISR_HTIF5) DMA2->HIFCR |= DMA_HIFCR_CHTIF5;
    if (DMA2->HISR & DMA_HISR_TEIF5) DMA2->HIFCR |= DMA_HIFCR_CTEIF5;
    if (DMA2->HISR & DMA_HISR_DMEIF5) DMA2->HIFCR |= DMA_HIFCR_CDMEIF5;
    if (DMA2->HISR & DMA_HISR_FEIF5) DMA2->HIFCR |= DMA_HIFCR_CFEIF5;
}

void DMA2_Stream4_IRQHandler(void)
{
    if (DMA2->HISR & DMA_HISR_TCIF4) {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF4;

        if (bdshot_reception_B) {
            // Tx Completion -> Start Rx
            arm_bdshot_rx_capture_B();
            for(volatile int i = 0; i < 100; i++) { __NOP(); }
            bdshot_reception_B = false;
        } else {
            // Rx Completion -> Cleanup
            DMA2_Stream4->CR &= ~DMA_SxCR_EN;
            while (DMA2_Stream4->CR & DMA_SxCR_EN) { __NOP(); }
            DMA2->HIFCR |= DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 |
                           DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;

            telemetry_done_flag |= 0x02; // Signal main loop

            __disable_irq();
            tim1_rx_done_count++;
            if (tim1_rx_done_count == 4) {
                TIM1->CR1 &= ~TIM_CR1_CEN; // Stop TIM1 only when all 4 streams are done
                tim1_rx_done_count = 0;
            }
            __enable_irq();

            bdshot_reception_B = true; // Reset state for next Tx cycle
        }
    }

    if (DMA2->HISR & DMA_HISR_HTIF4) DMA2->HIFCR |= DMA_HIFCR_CHTIF4;
    if (DMA2->HISR & DMA_HISR_TEIF4) DMA2->HIFCR |= DMA_HIFCR_CTEIF4;
    if (DMA2->HISR & DMA_HISR_DMEIF4) DMA2->HIFCR |= DMA_HIFCR_CDMEIF4;
    if (DMA2->HISR & DMA_HISR_FEIF4) DMA2->HIFCR |= DMA_HIFCR_CFEIF4;
}

void DMA2_Stream3_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF3) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF3;

        if (bdshot_reception_C) {
            // Tx Completion -> Start Rx
            arm_bdshot_rx_capture_C();
            bdshot_reception_C = false;
        } else {
            // Rx Completion -> Cleanup
            DMA2_Stream3->CR &= ~DMA_SxCR_EN;
            while (DMA2_Stream3->CR & DMA_SxCR_EN) { __NOP(); }
            DMA2->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
                           DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;

            telemetry_done_flag |= 0x04; // Signal main loop

            __disable_irq();
            tim1_rx_done_count++;
            if (tim1_rx_done_count == 4) {
                TIM1->CR1 &= ~TIM_CR1_CEN; // Stop TIM1 only when all 4 streams are done
                tim1_rx_done_count = 0;
            }
            __enable_irq();

            bdshot_reception_C = true; // Reset state for next Tx cycle
        }
    }

    if (DMA2->LISR & DMA_LISR_HTIF3) DMA2->LIFCR |= DMA_LIFCR_CHTIF3;
    if (DMA2->LISR & DMA_LISR_TEIF3) DMA2->LIFCR |= DMA_LIFCR_CTEIF3;
    if (DMA2->LISR & DMA_LISR_DMEIF3) DMA2->LIFCR |= DMA_LIFCR_CDMEIF3;
    if (DMA2->LISR & DMA_LISR_FEIF3) DMA2->LIFCR |= DMA_LIFCR_CFEIF3;
}

void DMA2_Stream6_IRQHandler(void)
{
    if (DMA2->HISR & DMA_HISR_TCIF6) {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF6;

        if (bdshot_reception_D) {
            // Tx Completion -> Start Rx
            arm_bdshot_rx_capture_D();
            bdshot_reception_D = false;
        } else {
            // Rx Completion -> Cleanup
            DMA2_Stream6->CR &= ~DMA_SxCR_EN;
            while (DMA2_Stream6->CR & DMA_SxCR_EN) { __NOP(); }
            DMA2->HIFCR |= DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |
                           DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;

            telemetry_done_flag |= 0x08; // Signal main loop

            __disable_irq();
            tim1_rx_done_count++;
            if (tim1_rx_done_count == 4) {
                TIM1->CR1 &= ~TIM_CR1_CEN; // Stop TIM1 only when all 4 streams are done
                tim1_rx_done_count = 0;
            }
            __enable_irq();

            bdshot_reception_D = true; // Reset state for next Tx cycle
        }
    }

    if (DMA2->HISR & DMA_HISR_HTIF6) DMA2->HIFCR |= DMA_HIFCR_CHTIF6;
    if (DMA2->HISR & DMA_HISR_TEIF6) DMA2->HIFCR |= DMA_HIFCR_CTEIF6;
    if (DMA2->HISR & DMA_HISR_DMEIF6) DMA2->HIFCR |= DMA_HIFCR_CDMEIF6;
    if (DMA2->HISR & DMA_HISR_FEIF6) DMA2->HIFCR |= DMA_HIFCR_CFEIF6;
}

void DMA2_Stream1_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF1) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF1;

        if (bdshot_reception_E) {
            // Tx Completion -> Start Rx
            arm_bdshot_rx_capture_E();
            bdshot_reception_E = false;
        } else {
            // Rx Completion -> Cleanup
            TIM8->CR1 &= ~TIM_CR1_CEN; // Stop TIM8 immediately (not shared)
            DMA2_Stream1->CR &= ~DMA_SxCR_EN;
            while (DMA2_Stream1->CR & DMA_SxCR_EN) { __NOP(); }
            DMA2->LIFCR |= DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |
                           DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

            telemetry_done_flag |= 0x10; // Signal main loop
            bdshot_reception_E = true; // Reset state for next Tx cycle
        }
    }

    if (DMA2->LISR & DMA_LISR_HTIF1) DMA2->LIFCR |= DMA_LIFCR_CHTIF1;
    if (DMA2->LISR & DMA_LISR_TEIF1) DMA2->LIFCR |= DMA_LIFCR_CTEIF1;
    if (DMA2->LISR & DMA_LISR_DMEIF1) DMA2->LIFCR |= DMA_LIFCR_CDMEIF1;
    if (DMA2->LISR & DMA_LISR_FEIF1) DMA2->LIFCR |= DMA_LIFCR_CFEIF1;
}

