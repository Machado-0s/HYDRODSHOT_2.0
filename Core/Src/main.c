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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dshot.h"
#include <string.h>
#include <stdio.h>
#include "uart_cmd.h"
#include "pwm.h"
#include "dshot_A.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define DWT_CYCCNT ((volatile uint32_t *)0xE0001004)
void DWT_Delay(uint32_t microseconds) {
    uint32_t start = *DWT_CYCCNT;
    uint32_t cycles = microseconds * (SystemCoreClock / 1000000);
    while((*DWT_CYCCNT - start) < cycles);
}

static inline void short_delay_us(uint32_t us)
{
    volatile uint32_t count = us * (168); // Roughly 1 cycle per loop at 84 MHz
    while (count--) {
        __NOP();
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint16_t current_motor_rpms[MOTORS_COUNT];
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;

volatile uint8_t dshot_send_flag = 0;
volatile bool uart_tx_ready = true; // TX flag

float value = 100.0;
uint16_t pwm_targets[4] = {1500, 1500, 1500, 1500};
volatile float pid_target_speed_rpms[MOTORS_COUNT] = {0};

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
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  UART_CMD_Init(&huart2);
  PWM_Init();

  setup_Dshot_Tx_Only();
  preset_bb_Dshot_buffers();
  pid_reset_all();

     //DWT_Delay(100);

   // Initialize all motor target RPMs to 0.0 (stop)
       for (int i = 0; i < MOTORS_COUNT; i++) {
           pid_target_speed_rpms[i] = value; // All motors initially stopped
       }

         for (int i = 0; i < MOTORS_COUNT; i++) {
              motor_values[i] = prepare_Dshot_package(0, false); // Send 0 throttle (disarmed)
          }

     // Send this 0 throttle for 200ms
     uint32_t calibration_start_time = HAL_GetTick();
     while (HAL_GetTick() - calibration_start_time < 2000) {
         update_motors_Tx_Only();
         // Keep the small delay to ensure signal integrity during calibration phase too
        for (volatile int i = 0; i < 100; i++);

     }


    // HAL_Delay(260);

     for (int i = 0; i < MOTORS_COUNT; i++) {
   	  motor_values[i] = prepare_Dshot_package(10, false); //11
        }

      for (int t = 0; t < 6; t++){
      update_motors_Tx_Only();

      }


      for (int i = 0; i < MOTORS_COUNT; i++) {
      	  motor_values[i] = prepare_Dshot_package(12, false); //20
           }

      for (int t = 0; t < 6; t++){

      update_motors_Tx_Only();

      }

      HAL_Delay(40);

 // Debug_Send_DMA("--- STM32 DShot Controller Started ---\r\n");




      uint32_t last_50hz_time = 0;
      uint32_t last_100hz_time = 0;
      uint32_t now2 = HAL_GetTick();


    // uint32_t check = 0;
    // uint32_t now1 = HAL_GetTick();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if (uart_new_data_available) {

	      process_uart_command();
	  }


	  if (telemetry_done_flag) {



	      // --- Step 1: Decode and store telemetry results ---
	      process_telemetry_with_new_method();

	      // --- Step 2: Run PID for each motor using measured dt ---
	      for (int m = 0; m < MOTORS_COUNT; m++) {

	          uint32_t current_rpm = 0;
	          float target_rpm = pid_target_speed_rpms[m];

	          // Use telemetry if valid, else conservative fallback
	          if (motor_telemetry_data[m].valid_rpm) {
	              current_rpm = motor_telemetry_data[m].raw_rpm_value;
	          } else {
	              current_rpm = 0; // ESC didn't respond
	          }
	         // Debug_Send_DMA("v: %d %d %.2f \r\n", m, current_rpm,target_rpm);
	          // Compute PID output — pass dt into your modified PID function
	          float dt = 0.005f;  // 5 ms control loop
	          uint16_t new_command = pid_calculate_command(m, current_rpm, target_rpm,dt);
	         // Debug_Send_DMA("hello");

	          // Always encode as valid DShot frame
	          motor_values[m] = prepare_Dshot_package(new_command, true);
	      }

	      // --- Step 3: Transmit new DShot commands ---
	      //for (volatile int i = 0; i < 100; i++);   // short delay
	      update_motors_Tx_Only();
	  }

	  /*
	  now1 = HAL_GetTick();
	  if (now1 - check >= 2000) {
	      Debug_Send_DMA(
	          "M%d err=%.1f T: %.0f OUT=%d\r\n",
	          pid_debug.motor_index,
	          pid_debug.error,
	          pid_debug.target,
	          pid_debug.output
	      );
	      check = now1;
	  }
*/


	  	      // ---- SERVO PWM UPDATE - Different frequencies
	  	      now2 = HAL_GetTick();

	  	      // Update 50Hz PWM motors every 20ms
	  	      if (now2 - last_50hz_time >= 20) {
	  	    	  PWM_SetDuty(&htim9, TIM_CHANNEL_1, pwm_targets[0]); // PE5
	  	    	  PWM_SetDuty(&htim9, TIM_CHANNEL_2, pwm_targets[1]); // PE6
	  	    	  last_50hz_time = now2;
	  	      }

	  	      // Update 100Hz PWM motors every 10ms
	  	      if (now2 - last_100hz_time >= 10) {

	  	    	  PWM_SetDuty(&htim12, TIM_CHANNEL_1, pwm_targets[2]); // PB14
	  	    	  PWM_SetDuty(&htim12, TIM_CHANNEL_2, pwm_targets[3]); // PB15
	  	    	  last_100hz_time = now2;


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
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // CRITICAL: Clear the consistent busy flag when DMA is complete
        uart_tx_busy = false;
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
