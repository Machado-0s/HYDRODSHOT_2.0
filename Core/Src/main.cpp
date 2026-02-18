/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Merged Main program body (C++)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
/* Wrap C headers in extern "C" to allow linking with C++ */
extern "C" {
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "dshot.h"
#include "pwm.h"
#include "dshot_A.h"
#include <string.h>
#include <stdio.h>
}

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>

#include "hydrolib_bus_application_slave.hpp"
#include "hydrolib_bus_datalink_stream.hpp"
#include "hydrolib_bus_application_master.hpp"
#include "hal_uart_adapter.hpp"
#include "hydrolib_return_codes.hpp"
#include <cstring>
#include "byteProtocol_tx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern "C" {
extern UART_HandleTypeDef huart1;
}
#define BUFFER_LENGTH 14
#define DSHOT_MAX_RPM 6000

float value = 0.0;
uint16_t pwm_targets[4] = {1500, 1500, 1500, 1500};
volatile float pid_target_speed_rpms[MOTORS_COUNT] = {0};
uint8_t pid_target_speed_rpm_conversion[MOTORS_COUNT] = {0};
 uint8_t pwm_targets_conversion[4] = {150, 150, 150, 150};

uint8_t pinState = 0;
uint16_t adc_buffer[2];
volatile bool battery_data_ready = false;
uint16_t bat1 = 0;
uint16_t bat2 = 0;
static uint8_t tx_buffer_bat[5];
BatteryData_t battery_data;

 
class Memory
{
public:
    hydrolib::ReturnCode Read(void *buffer, unsigned address, unsigned length)
    {
        if (length + address > BUFFER_LENGTH)
        {
            return hydrolib::ReturnCode::FAIL;
        }
        memcpy(buffer, buffer_ + address, length);
        return hydrolib::ReturnCode::OK;
    }

    hydrolib::ReturnCode Write(const void *buffer, unsigned address,
                               unsigned length)
    {
        if (address + length > BUFFER_LENGTH)
        {
            return hydrolib::ReturnCode::FAIL;
        }
        memcpy(buffer_ + address, buffer, length);
        return hydrolib::ReturnCode::OK;
    }

    uint32_t Size() { return BUFFER_LENGTH; }

private:
    uint8_t buffer_[BUFFER_LENGTH] = {};
};



hydrolib::bus::datalink::StreamManager manager(1, huart1);
hydrolib::bus::datalink::Stream stream(manager, 0xFD);

Memory memory;

hydrolib::bus::application::Slave slave(stream, memory);
hydrolib::bus::application::Master<hydrolib::bus::datalink::Stream<UART_HandleTypeDef>&> master(stream);

void getCommmands(void){

      
       for(int i =0;i<10;i++){
       if (memory.Read(&pid_target_speed_rpm_conversion, 0, 10) != hydrolib::ReturnCode::OK)
    continue;

        if (pid_target_speed_rpm_conversion[i] >= 100 && pid_target_speed_rpm_conversion[i] <= 200) {
            int32_t signed_val = (int32_t)pid_target_speed_rpm_conversion[i] - 150;
            int32_t rpm_int = (signed_val * DSHOT_MAX_RPM) / 50.0f;
         pid_target_speed_rpms[i] = rpm_int;
        }
    }

    for (int i =1;i<4;i++){
           if (memory.Read(&pwm_targets_conversion, 10 , 4) != hydrolib::ReturnCode::OK)
    continue;

        if (pwm_targets_conversion[i] >= 100 && pwm_targets_conversion[i] <= 200) {
            uint16_t pulse_us = 1000 + ((uint16_t)(pwm_targets_conversion[i] - 100) * 1000 / 100);
            pwm_targets[i] = pulse_us;
        }
    }
  
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
extern "C" {
void SystemClock_Config(void);
}
/* USER CODE BEGIN PFP */
void adc_start(void);
void calibration(void);
void quick_battery_read(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void adc_start(void) {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);
}


void ByteProtocol_TX_SendBatteryData(const BatteryData_t* data)
{
    memset(tx_buffer_bat, 0, sizeof(tx_buffer_bat));

    tx_buffer_bat[0] =data->vbat1_adc & 0xFF;
    tx_buffer_bat[1] =(data->vbat1_adc >> 8) & 0xFF;
    tx_buffer_bat[2] =data-> vbat2_adc & 0xFF;
    tx_buffer_bat[3] =(data->vbat2_adc >> 8) & 0xFF;
    tx_buffer_bat[4] =data->killswitch_state ? 0x01 : 0x00;

   master.Write(tx_buffer_bat, 0, 5);
}

void quick_battery_read(void){
	battery_data.vbat1_adc = adc_buffer[0];
	battery_data.vbat2_adc = adc_buffer[1]; 
}

void send_battery_data(void) {

    battery_data.vbat1_adc = bat1;
    battery_data.vbat2_adc = bat2;

    ByteProtocol_TX_SendBatteryData(&battery_data);
}


void calibration(void){
    for (int i = 0; i < MOTORS_COUNT; i++) {
        pid_target_speed_rpms[i] = value;
    }

    for (int i = 0; i < MOTORS_COUNT; i++) {
        motor_values[i] = prepare_Dshot_package(0, false);
    }

    uint32_t calibration_start_time = HAL_GetTick();
    while (HAL_GetTick() - calibration_start_time < 2000) {
        update_motors_Tx_Only();
        for (volatile int i = 0; i < 100; i++);
    }

    for (int i = 0; i < MOTORS_COUNT; i++) {
	    motor_values[i] = prepare_Dshot_package(10, false);
    }

    for (int t = 0; t < 6; t++){
        update_motors_Tx_Only();
    }

    for (int i = 0; i < MOTORS_COUNT; i++) {
   	    motor_values[i] = prepare_Dshot_package(12, false);
    }

    for (int t = 0; t < 6; t++){
        update_motors_Tx_Only();
    }

    HAL_Delay(40);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HalUartAdapter_Init(&huart1);
  
  PWM_Init();
  adc_start();

  setup_Dshot_Tx_Only();
  preset_bb_Dshot_buffers();
  pid_reset_all();
  calibration();

  uint32_t last_50hz_time = 0;
  uint32_t last_100hz_time = 0;
  uint32_t last_battery_tx = 0;
  uint32_t now2 = HAL_GetTick();

  uint16_t count = 0;
  uint16_t err = 0;

  GPIOC->MODER |= GPIO_MODER_MODER13_0;
  GPIOC->MODER |= GPIO_MODER_MODER14_0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   battery_data.killswitch_state =  (GPIOA->IDR & GPIO_PIN_3) ? true : false;

	 if(pinState == 0 && battery_data.killswitch_state ==1 ){
		 HAL_Delay(200);
		calibration();
		 pinState = battery_data.killswitch_state;
	  }

    manager.Process();
    slave.Process();
    master.Process();

    getCommmands();
  

	if(battery_data_ready){

         quick_battery_read();
         battery_data_ready = false;

	  }
   

	if (telemetry_done_flag) {
        process_telemetry_with_new_method();

	    for (int m = 0; m < MOTORS_COUNT; m++) {
	        uint32_t current_rpm = 0;
	        float target_rpm = pid_target_speed_rpms[m];

	        if (motor_telemetry_data[m].valid_rpm) {
	            current_rpm = motor_telemetry_data[m].raw_rpm_value;
	        } else {
	            current_rpm = 0;
	        }

	        float dt = 0.005f;
	        uint16_t new_command = pid_calculate_command(m, current_rpm, target_rpm, dt);
	        motor_values[m] = prepare_Dshot_package(new_command, true);
	    }

	    update_motors_Tx_Only();
	    GPIOC->ODR |= GPIO_ODR_OD13;
	    GPIOC->ODR &= ~GPIO_ODR_OD14;
        err = 0;
	} else {
		if(err > 1000){
	        GPIOC->ODR &= ~GPIO_ODR_OD13;
	        GPIOC->ODR |= GPIO_ODR_OD14;
	    }
		err++;
	}

    now2 = HAL_GetTick();

	if (now2 - last_50hz_time >= 20) {
	  	if(count == 0){
	  	    PWM_SetDuty(&htim9, TIM_CHANNEL_1, 1000); // PE5
	  	    PWM_SetDuty(&htim9, TIM_CHANNEL_2, 1000); // PE6
	  	    count++;
	  	} else {
	  	    PWM_SetDuty(&htim9, TIM_CHANNEL_1, pwm_targets[0]); // PE5
	  	    PWM_SetDuty(&htim9, TIM_CHANNEL_2, pwm_targets[1]); // PE6
	  	}
	  	last_50hz_time = now2;
	}

	if (now2 - last_100hz_time >= 10) {
	    PWM_SetDuty(&htim12, TIM_CHANNEL_1, pwm_targets[2]); // PB14
	    PWM_SetDuty(&htim12, TIM_CHANNEL_2, pwm_targets[3]); // PB15
	    last_100hz_time = now2;
	}

	  
    if (now2 - last_battery_tx >= 100){  
         send_battery_data();
        last_battery_tx = now2;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
         battery_data_ready = true;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
extern "C" void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}