/*
 * pwm.c
 *
 *  Created on: Jul 16, 2025
 *      Author: Chard Ethern
 */
#include "pwm.h"

// Экспортируемые таймеры
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;

void PWM_Init(void)
{
    // Запуск PWM каналов
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); // PE6
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); // PE5

     //Start 100Hz PWM channels (TIM3)
      HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); // e.g., PB14- 100Hz
      HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); // e.g., PB15- 100Hz
}

void PWM_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us)
{
    // Clamp pulse to servo-safe range
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;

    __HAL_TIM_SET_COMPARE(htim, channel, pulse_us);
}

