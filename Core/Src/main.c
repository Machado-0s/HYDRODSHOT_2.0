/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dshot.h"
#include <string.h>
#include <stdio.h>
#include "pwm.h"
#include "dshot_A.h"
#include "byteProtocol.h"
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

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

volatile bool uart_tx_ready = true;

float value = 0.0;
uint16_t pwm_targets[4] = {1500, 1500, 1500, 1500};
volatile float pid_target_speed_rpms[MOTORS_COUNT] = {0};

uint8_t pinState = 0;
uint16_t adc_buffer[2];
volatile bool battery_data_ready = false;
uint16_t bat1 = 0;
uint16_t bat2 = 0;
BatteryData_t battery_data;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void adc_start(void) {

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);
}


void ByteProtocol_DShotUpdateInt(uint8_t motor_idx, int32_t rpm) {
    if (motor_idx < MOTORS_COUNT) {
    	pid_target_speed_rpms[motor_idx] = (float)rpm;

    }
}

void ByteProtocol_PWMUpdate(uint8_t pwm_idx, uint16_t pulse_us) {
    if (pwm_idx < 4) {
    	pwm_targets[pwm_idx] = pulse_us;
    }
}


void quick_battery_read(void){
	bat1 = adc_buffer[0];
	bat2 = adc_buffer[1];
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
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
  PWM_Init();
  adc_start();

  ByteProtocol_TX_Init();
  ByteProtocol_Init();

  setup_Dshot_Tx_Only();
  preset_bb_Dshot_buffers();
  pid_reset_all();
  calibration();

  uint32_t last_50hz_time = 0;
  uint32_t last_100hz_time = 0;
  uint32_t last_battery_tx_time = 0;
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
	          uint16_t new_command = pid_calculate_command(m, current_rpm, target_rpm,dt);

	          motor_values[m] = prepare_Dshot_package(new_command, true);
	      }


	      update_motors_Tx_Only();
	      GPIOC->ODR |= GPIO_ODR_OD13;
	      GPIOC->ODR &= ~GPIO_ODR_OD14;

               err=0;
	      }else{

		  if(err>1000){
	           GPIOC->ODR &= ~GPIO_ODR_OD13;
	           GPIOC->ODR |= GPIO_ODR_OD14;

	        }
		  err++;
	  }


 now2 = HAL_GetTick();

	if (now2 - last_50hz_time >= 20) {
	  	  if(count==0){

	  	    PWM_SetDuty(&htim9, TIM_CHANNEL_1, 1000); // PE5
	  	    PWM_SetDuty(&htim9, TIM_CHANNEL_2, 1000); // PE6
	  	    count++;

	  	     }else{
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

	  if (now2 - last_battery_tx_time >= 100) {
	  	     send_battery_data();
	  	      last_battery_tx_time = now2;
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
         battery_data_ready = true;
    }
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
