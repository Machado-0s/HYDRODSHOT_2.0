/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Merged Main program body (C++) 
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
extern "C" {
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
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
#include <stdbool.h>

#include "hydrv_uart.hpp"
#include "hydrv_gpio_low.hpp"
#include "hydrolib_bus_application_slave.hpp"
#include "hydrolib_bus_datalink_stream.hpp"
#include "hydrolib_bus_application_master.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrolib_log_distributor.hpp"
#include "hydrolib_logger.hpp"
#include <cstring>
#include <chrono>
#include <ctime>

extern "C" {
    #include <sys/time.h>
    int _gettimeofday(struct timeval *tv, void *tz) {
        if (tv) {
            tv->tv_sec = HAL_GetTick() / 1000;
            tv->tv_usec = (HAL_GetTick() % 1000) * 1000;
        }
        return 0;
    }
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern "C" {
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
}
#define BUFFER_LENGTH 20
#define DSHOT_MAX_RPM 6000

typedef struct {
    uint16_t vbat1_adc;
    uint16_t vbat2_adc;
    bool killswitch_state;
} BatteryData_t;

float value = 0.0;
uint16_t pwm_targets[4] = {1500, 1500, 1500, 1500};
volatile float pid_target_speed_rpms[MOTORS_COUNT] = {0};
uint8_t pid_target_speed_rpm_conversion[MOTORS_COUNT] = {0};
uint8_t pwm_targets_conversion[4] = {150, 150, 150, 150};

uint8_t pinState = 0;
volatile bool battery_data_ready = false;
BatteryData_t battery_data;

class Memory {
public:
    hydrolib::ReturnCode Read(void *buffer, unsigned address, unsigned length) {
        if (length + address > BUFFER_LENGTH) return hydrolib::ReturnCode::FAIL;
        memcpy(buffer, buffer_ + address, length);
        return hydrolib::ReturnCode::OK;
    }
    hydrolib::ReturnCode Write(const void *buffer, unsigned address, unsigned length) {
        if (address + length > BUFFER_LENGTH) return hydrolib::ReturnCode::FAIL;
        memcpy(buffer_ + address, buffer, length);
        return hydrolib::ReturnCode::OK;
    }
    uint32_t Size() { return BUFFER_LENGTH; }
private:
    uint8_t buffer_[BUFFER_LENGTH] = {};
};

constinit hydrv::GPIO::GPIOLow rx_pin1(hydrv::GPIO::GPIOLow::GPIOA_port, 10,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin1(hydrv::GPIO::GPIOLow::GPIOA_port, 9,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255>
    uart1(hydrv::UART::UARTLow::USART1_115200_LOW, rx_pin1, tx_pin1, 7);

constinit hydrv::GPIO::GPIOLow rx_pin2(hydrv::GPIO::GPIOLow::GPIOD_port, 6,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin2(hydrv::GPIO::GPIOLow::GPIOD_port, 5,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255>
    uart2(hydrv::UART::UARTLow::USART2_115200_LOW, rx_pin2, tx_pin2, 7);


    char log_format[] = "[%s] [%l] %m\n\r";

constinit hydrolib::logger::LogDistributor<hydrv::UART::UART<255, 255>> distributor(log_format, uart2);

constinit hydrolib::logger::Logger<hydrolib::logger::LogDistributor<hydrv::UART::UART<255, 255>>> logger("SerialProtocol", 1, distributor);

hydrolib::bus::datalink::StreamManager manager(1, uart1, logger);

hydrolib::bus::datalink::Stream stream(manager, 2);

Memory memory;

hydrolib::bus::application::Slave slave(stream, memory, logger);

hydrolib::bus::application::Master master(stream, logger);

void getCommmands(void){
    memory.Read(&pid_target_speed_rpm_conversion, 0 , 10) ;
    memory.Read(&pwm_targets_conversion, 10 , 4) ;
    for(int i =0;i<10;i++){
        if (pid_target_speed_rpm_conversion[i] >= 100 && pid_target_speed_rpm_conversion[i] <= 200) {
            int32_t signed_val = (int32_t)pid_target_speed_rpm_conversion[i] - 150;
            pid_target_speed_rpms[i] = (float)signed_val * 120.0f;
        }
    }
    for (int i=0; i<4;i++){
        if (pwm_targets_conversion[i] >= 100 && pwm_targets_conversion[i] <= 200) {
            pwm_targets[i] = (uint16_t)(pwm_targets_conversion[i] * 10);
        }
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
extern "C" {
     void SystemClock_Config(void); 
     void adc_start(void);
}

void calibration(void);


/* Private user code ---------------------------------------------------------*/

void read_battery_polling(void) {
    HAL_ADC_Start(&hadc1);                    

    
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {  
        battery_data.vbat1_adc = HAL_ADC_GetValue(&hadc1);
    }

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        battery_data.vbat2_adc = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);  
}

uint8_t tx_buffer_bat[5];
void ByteProtocol_TX_SendBatteryData(const BatteryData_t* data) {
    
    tx_buffer_bat[0] = data->vbat1_adc & 0xFF;
    tx_buffer_bat[1] = (data->vbat1_adc >> 8) & 0xFF;
    tx_buffer_bat[2] = data->vbat2_adc & 0xFF;
    tx_buffer_bat[3] = (data->vbat2_adc >> 8) & 0xFF;
    tx_buffer_bat[4] = data->killswitch_state ? 0x01 : 0x00;
    memory.Write(tx_buffer_bat, 14, 5);
}


void calibration(void) {
    for (int i = 0; i < MOTORS_COUNT; i++) { pid_target_speed_rpms[i] = value; }
    for (int i = 0; i < MOTORS_COUNT; i++) { motor_values[i] = prepare_Dshot_package(0, false); }
    uint32_t calibration_start_time = HAL_GetTick();
    while (HAL_GetTick() - calibration_start_time < 2000) {
        update_motors_Tx_Only();
        for (volatile int i = 0; i < 100; i++);
    }
    for (int i = 0; i < MOTORS_COUNT; i++) { motor_values[i] = prepare_Dshot_package(10, false); }
    for (int t = 0; t < 6; t++) { update_motors_Tx_Only(); }
    for (int i = 0; i < MOTORS_COUNT; i++) { motor_values[i] = prepare_Dshot_package(12, false); }
    for (int t = 0; t < 6; t++) { update_motors_Tx_Only(); }
    HAL_Delay(40);
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  PWM_Init();
  setup_Dshot_Tx_Only();
  preset_bb_Dshot_buffers();
  pid_reset_all();
  calibration();
  

  NVIC_SetPriorityGrouping(0);
  uart1.Init();
  uart2.Init();

  uint32_t last_50hz_time = 0;
  uint32_t last_100hz_time = 0;
  uint32_t last_battery_tx = 0;
  uint16_t count = 0;
  uint16_t err = 0;

  GPIOC->MODER |= GPIO_MODER_MODER13_0;
  GPIOC->MODER |= GPIO_MODER_MODER14_0;

  while (1) {
    battery_data.killswitch_state = (GPIOA->IDR & GPIO_PIN_3) ? true : false;
    if(pinState == 0 && battery_data.killswitch_state == 1) {
        HAL_Delay(200);
        pid_reset_all();
        calibration();
        pinState = battery_data.killswitch_state;
    }

    manager.Process();
    slave.Process();
    master.Process();
    getCommmands();

    uint32_t now = HAL_GetTick();

if (now - last_battery_tx >= 100) {  
    read_battery_polling();                                       
    ByteProtocol_TX_SendBatteryData(&battery_data);
    last_battery_tx = now;
}
  


    if (telemetry_done_flag) {
        process_telemetry_with_new_method();
        for (int m = 0; m < MOTORS_COUNT; m++) {
            uint32_t current_rpm = motor_telemetry_data[m].valid_rpm ? motor_telemetry_data[m].raw_rpm_value : 0;
            float dt = 0.005f;
            uint16_t new_command = pid_calculate_command(m, current_rpm, pid_target_speed_rpms[m], dt);
            motor_values[m] = prepare_Dshot_package(new_command, true);
        }
        update_motors_Tx_Only();
        GPIOC->ODR |= GPIO_ODR_OD13;
        GPIOC->ODR &= ~GPIO_ODR_OD14;
        err = 0;
    } else {
        if(err > 1000) {
            GPIOC->ODR &= ~GPIO_ODR_OD13;
            GPIOC->ODR |= GPIO_ODR_OD14;
        }
        err++;
    }

    uint32_t now2 = HAL_GetTick();
    if (now2 - last_50hz_time >= 20) {
        if(count == 0) {
            PWM_SetDuty(&htim9, TIM_CHANNEL_1, 1000);
            PWM_SetDuty(&htim9, TIM_CHANNEL_2, 1000);
            count++;
        } else {
            PWM_SetDuty(&htim9, TIM_CHANNEL_1, pwm_targets[0]);
            PWM_SetDuty(&htim9, TIM_CHANNEL_2, pwm_targets[1]);
        }
        last_50hz_time = now2;
    }

    if (now2 - last_100hz_time >= 10) {
        PWM_SetDuty(&htim12, TIM_CHANNEL_1, pwm_targets[2]);
        PWM_SetDuty(&htim12, TIM_CHANNEL_2, pwm_targets[3]);
        last_100hz_time = now2;
    }

  
  }
}

extern "C" void SystemClock_Config(void) {
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
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

extern "C"
{
   
    void USART2_IRQHandler(void) { uart2.IRQCallback(); }
    void USART1_IRQHandler(void) { uart1.IRQCallback(); }
}


extern "C" void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}