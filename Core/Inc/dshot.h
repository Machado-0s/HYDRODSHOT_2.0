#ifndef DSHOT_H
#define DSHOT_H



#include <main.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>


// Include the pair files
#include "dshot_A.h"
#include "uart_cmd.h"

typedef struct {
    uint8_t  motor_index;
    float    error;
    float    target;
    uint16_t output;
    uint8_t  pending;
} pid_debug_t;

extern volatile pid_debug_t pid_debug;

// Type definitions
typedef struct {
    float i_term;
    float last_error;
    float filtered_error;
    uint16_t last_dshot_command;
    float last_valid_rpm;
} PID_State_t;

typedef struct {
    bool valid_rpm;
    bool valid_voltage;
    uint16_t raw_rpm_value;
} MotorTelemetry_t;

// Global variables
extern volatile uint16_t last_sent_dshot_command;
extern PID_State_t pid_states[MOTORS_COUNT];
extern float pid_integral;
extern float pid_last_error;
extern float pid_filtered_error;
extern float PID_KP;
extern float PID_KI;
extern float PID_KD;

extern volatile float pid_target_speed_rpms[MOTORS_COUNT];

extern UART_HandleTypeDef huart1;
extern uint16_t motor_command_dshot_value;
extern volatile bool telemetry_data_ready_flag;
extern uint16_t motor_values[MOTORS_COUNT];
extern volatile uint8_t telemetry_done_flag;
extern MotorTelemetry_t motor_telemetry_data[MOTORS_COUNT];
extern volatile bool request_voltage_next;
extern const uint8_t motor_gpio_port[MOTORS_COUNT];
extern const uint8_t motor_gpio_pin_numbers[MOTORS_COUNT];

// Common function declarations
void preset_bb_Dshot_buffers(void);
void fill_bb_Dshot_buffers(const uint16_t motor_packets[MOTORS_COUNT]);
uint16_t prepare_Dshot_package(uint16_t value, bool telemetry);
void set_idle_on_ports(void);
void update_motors_Tx_Only(void);
void setup_Dshot_Tx_Only(void);
void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream4_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
uint32_t get_BDshot_response(uint32_t raw_buffer[], const uint8_t motor_shift);
bool BDshot_check_checksum(uint32_t decoded_value);
void read_BDshot_response(uint32_t value, uint8_t motor);
void process_telemetry_with_new_method(void);
uint16_t get_rpm_from_telemetry(uint16_t raw_value);
void pid_reset_all(void);
void pid_reset_motor(uint8_t motor_index);
uint16_t pid_calculate_command(uint8_t motor_index, uint32_t current_rpm_unsigned, float target_rpm_signed,float dt);

// Helper macros
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#endif // DSHOT_H
