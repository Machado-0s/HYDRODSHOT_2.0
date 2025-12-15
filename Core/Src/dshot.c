#include "dshot.h"
#include "dshot_A.h"
#include <string.h>
#include <stdio.h>
#include "uart_cmd.h"
#include "math.h"

// --- Global Variables ---
volatile uint16_t last_sent_dshot_command = 0;
PID_State_t pid_states[MOTORS_COUNT] = {0};

float pid_integral = 0;
float pid_last_error = 0;
float pid_filtered_error = 0;

float PID_KP = 0.5f;
float PID_KI = 1.2f;
float PID_KD = 0.01f;

uint16_t motor_values[MOTORS_COUNT] = {0};

volatile uint8_t telemetry_done_flag = 0;
MotorTelemetry_t motor_telemetry_data[MOTORS_COUNT];

// Updated motor mapping according to timer assignments
const uint8_t motor_gpio_port[MOTORS_COUNT] = {4, 4, 0, 1, 0, 1, 3, 3, 2, 2};
const uint8_t motor_gpio_pin_numbers[MOTORS_COUNT] = {9, 13, 5, 10, 6, 0, 12, 14, 6, 8};


volatile pid_debug_t pid_debug = {0};


// --- Helper Functions ---
static inline uint32_t get_port_mask(uint8_t port)
{
    uint32_t mask = 0;
    for (uint8_t m = 0; m < MOTORS_COUNT; m++) {
        if (motor_gpio_port[m] == port) {
            mask |= (1u << motor_gpio_pin_numbers[m]);
        }
    }
    return mask;
}

static inline void get_all_ports_masks(uint32_t *maskA, uint32_t *maskB, uint32_t *maskC, uint32_t *maskD, uint32_t *maskE)
{
    *maskA = get_port_mask(0);  // PA5, PA6
    *maskB = get_port_mask(1);  // PB0, PB10
    *maskC = get_port_mask(2);  // PC6, PC8
    *maskD = get_port_mask(3);  // PD12, PD14
    *maskE = get_port_mask(4);  // PE9, PE13
}

void preset_bb_Dshot_buffers(void)
{
    memset((void*)dshot_bb_buffer_A, 0, sizeof(dshot_bb_buffer_A));
    memset((void*)dshot_bb_buffer_B, 0, sizeof(dshot_bb_buffer_B));
    memset((void*)dshot_bb_buffer_C, 0, sizeof(dshot_bb_buffer_C));
    memset((void*)dshot_bb_buffer_D, 0, sizeof(dshot_bb_buffer_D));
    memset((void*)dshot_bb_buffer_E, 0, sizeof(dshot_bb_buffer_E));

    uint32_t maskA, maskB, maskC, maskD, maskE;
    get_all_ports_masks(&maskA, &maskB, &maskC, &maskD, &maskE);

    dshot_bb_buffer_A[0] = maskA;
    dshot_bb_buffer_B[0] = maskB;
    dshot_bb_buffer_C[0] = maskC;
    dshot_bb_buffer_D[0] = maskD;
    dshot_bb_buffer_E[0] = maskE;

    for (uint16_t bit = 0; bit < DSHOT_BB_BUFFER_BITS; bit++) {
        uint16_t idx_base = (bit * DSHOT_BB_FRAME_SECTIONS) + 1;
        uint16_t idx_end = idx_base + DSHOT_BB_1_LENGTH;

        if (idx_base < DSHOT_BB_BUFFER_LENGTH) {
            dshot_bb_buffer_A[idx_base] |= (maskA << 16);
            dshot_bb_buffer_B[idx_base] |= (maskB << 16);
            dshot_bb_buffer_C[idx_base] |= (maskC << 16);
            dshot_bb_buffer_D[idx_base] |= (maskD << 16);
            dshot_bb_buffer_E[idx_base] |= (maskE << 16);
        }

        if (idx_end < DSHOT_BB_BUFFER_LENGTH) {
            dshot_bb_buffer_A[idx_end] |= maskA;
            dshot_bb_buffer_B[idx_end] |= maskB;
            dshot_bb_buffer_C[idx_end] |= maskC;
            dshot_bb_buffer_D[idx_end] |= maskD;
            dshot_bb_buffer_E[idx_end] |= maskE;
        }
    }

    dshot_bb_buffer_A[DSHOT_BB_BUFFER_LENGTH - 1] |= get_port_mask(0);
    dshot_bb_buffer_B[DSHOT_BB_BUFFER_LENGTH - 1] |= get_port_mask(1);
    dshot_bb_buffer_C[DSHOT_BB_BUFFER_LENGTH - 1] |= get_port_mask(2);
    dshot_bb_buffer_D[DSHOT_BB_BUFFER_LENGTH - 1] |= get_port_mask(3);
    dshot_bb_buffer_E[DSHOT_BB_BUFFER_LENGTH - 1] |= get_port_mask(4);
}

void fill_bb_Dshot_buffers(const uint16_t motor_packets[MOTORS_COUNT])
{
    preset_bb_Dshot_buffers();

    for (uint8_t bit = 0; bit < BITS_PER_FRAME; bit++) {
        uint16_t idx_base = bit * DSHOT_BB_FRAME_SECTIONS;
        uint8_t bit_pos = 15 - bit;
        uint16_t idx_one = idx_base + DSHOT_BB_1_LENGTH;
        uint16_t idx_zero = idx_base + DSHOT_BB_0_LENGTH;

        for (uint8_t m = 0; m < MOTORS_COUNT; m++) {
            uint8_t port = motor_gpio_port[m];
            uint32_t pin_mask = (1u << motor_gpio_pin_numbers[m]);
            bool is_one = (motor_packets[m] >> bit_pos) & 1;

            if (!is_one) {
                if (idx_one < DSHOT_BB_BUFFER_LENGTH) {
                    switch(port) {
                        case 0: dshot_bb_buffer_A[idx_one] &= ~pin_mask; break;
                        case 1: dshot_bb_buffer_B[idx_one] &= ~pin_mask; break;
                        case 2: dshot_bb_buffer_C[idx_one] &= ~pin_mask; break;
                        case 3: dshot_bb_buffer_D[idx_one] &= ~pin_mask; break;
                        case 4: dshot_bb_buffer_E[idx_one] &= ~pin_mask; break;
                    }
                }
                if (idx_zero < DSHOT_BB_BUFFER_LENGTH) {
                    switch(port) {
                        case 0: dshot_bb_buffer_A[idx_zero] |= pin_mask; break;
                        case 1: dshot_bb_buffer_B[idx_zero] |= pin_mask; break;
                        case 2: dshot_bb_buffer_C[idx_zero] |= pin_mask; break;
                        case 3: dshot_bb_buffer_D[idx_zero] |= pin_mask; break;
                        case 4: dshot_bb_buffer_E[idx_zero] |= pin_mask; break;
                    }
                }
            }
        }
    }
}

uint16_t prepare_Dshot_package(uint16_t value, bool telemetry)
{
    value = (value << 1) | (telemetry ? 1 : 0);
    uint16_t crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
    return (value << 4) | crc;
}

void set_idle_on_ports(void)
{
    uint32_t maskA, maskB, maskC, maskD, maskE;
    get_all_ports_masks(&maskA, &maskB, &maskC, &maskD, &maskE);

    if (maskA) GPIOA->BSRR = maskA;
    if (maskB) GPIOB->BSRR = maskB;
    if (maskC) GPIOC->BSRR = maskC;
    if (maskD) GPIOD->BSRR = maskD;
    if (maskE) GPIOE->BSRR = maskE;
}

// --- Updated update_motors_Tx_Only (Coordinates all pairs A-E) ---
void update_motors_Tx_Only(void)
{
    set_idle_on_ports();
    fill_bb_Dshot_buffers(motor_values);

    // Reset reception flags
    bdshot_reception_A = true;
    bdshot_reception_B = true;
    bdshot_reception_C = true;
    bdshot_reception_D = true;
    bdshot_reception_E = true;

    // Configure all pairs
    dshot_A_tx_configuration();
    dshot_B_tx_configuration();
    dshot_C_tx_configuration();
    dshot_D_tx_configuration();
    dshot_E_tx_configuration();

    // --- Start Sequence for TIM1 pairs (A, B, C, D) ---
    TIM1->ARR = DSHOT_BB_SECTION_LENGTH - 1;
    TIM1->CCR1 = 1; // Trigger CC1 DMA
    TIM1->CCR3 = 1; // Trigger CC3 DMA
    TIM1->CCR4 = 1; // Trigger CC4/COM DMA

    TIM1->EGR |= TIM_EGR_UG | TIM_EGR_COMG;
    //TIM1->CNT = 0;             // Ensure counter is explicitly at 0
    // Enable all required DMA triggers for TIM1
    TIM1->DIER |= TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC3DE | TIM_DIER_CC4DE;// 4

    // --- Start Sequence for TIM8 pair (E) ---
    TIM8->ARR = DSHOT_BB_SECTION_LENGTH - 1;
    TIM8->EGR |= TIM_EGR_UG;
    TIM8->DIER |= TIM_DIER_UDE; // 4

    __DSB(); __ISB();

    // Enable Timers SECOND
       TIM1->CR1 |= TIM_CR1_CEN;
       TIM8->CR1 |= TIM_CR1_CEN;

    // Enable DMA Streams FIRST
    DMA2_Stream5->CR |= DMA_SxCR_EN; // Port A (TIM1_UP)
    DMA2_Stream4->CR |= DMA_SxCR_EN; // Port B (TIM1_CH4/COM)
    DMA2_Stream3->CR |= DMA_SxCR_EN; // Port C (TIM1_CH1)
    DMA2_Stream6->CR |= DMA_SxCR_EN; // Port D (TIM1_CH3)
    DMA2_Stream1->CR |= DMA_SxCR_EN; // Port E (TIM8_UP)


}

// --- Updated setup_Dshot_Tx_Only (Configuration for all pairs) ---
void setup_Dshot_Tx_Only(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // --- TIM1 SETUP (For pairs A, B, C, D) ---
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_URS;
    TIM1->PSC = 0;
    TIM1->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;
    TIM1->CCR1 = (DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS);///2;
    TIM1->CCR3 = 1; // For Port D
    TIM1->CCR4 = 1; // For Port B

    TIM1->CR2 = TIM_CR2_MMS_1; // Update Event is TRGO
    TIM1->DIER |= TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC3DE | TIM_DIER_CC4DE;

    // Configure Commutation Event for Port B
    TIM1->CCMR2 &= ~TIM_CCMR2_OC4M;
    TIM1->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos);
    TIM1->CCER |= TIM_CCER_CC4E;
    TIM1->BDTR |= TIM_BDTR_MOE;

    TIM1->EGR |= TIM_EGR_UG;
    TIM1->SR &= ~TIM_SR_UIF;

    // --- TIM8 SETUP (For pair E) ---
    TIM8->CR1 = TIM_CR1_ARPE | TIM_CR1_URS;
    TIM8->PSC = 0;
    TIM8->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;
    TIM8->BDTR |= TIM_BDTR_MOE;
    TIM8->EGR |= TIM_EGR_UG;
    TIM8->SR &= ~TIM_SR_UIF;

    // --- DMA Configurations ---

    // DMA2 Stream 5, Channel 6 (TIM1_UP) - PORT A
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream5->CR & DMA_SxCR_EN);
    //DMA2->HIFCR = 0xFFFFFFFF;
    DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
    DMA2_Stream5->CR = (6 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_DIR_0;
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    NVIC_SetPriority(DMA2_Stream5_IRQn, 3);

    // DMA2 Stream 4, Channel 6 (TIM1_CH4/COM) - PORT B
    DMA2_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream4->CR & DMA_SxCR_EN);
    //DMA2->HIFCR = 0xFFFFFFFF;
    DMA2->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;
    DMA2_Stream4->CR = (6 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_DIR_0;
    NVIC_EnableIRQ(DMA2_Stream4_IRQn);
    NVIC_SetPriority(DMA2_Stream4_IRQn, 3);

    // DMA2 Stream 3, Channel 6 (TIM1_CH1) - PORT C
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream3->CR & DMA_SxCR_EN);
    //DMA2->LIFCR = 0xFFFFFFFF;
    DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;
    DMA2_Stream3->CR = (6 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_DIR_0;
    NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    NVIC_SetPriority(DMA2_Stream3_IRQn, 3);

    // DMA2 Stream 6, Channel 6 (TIM1_CH3) - PORT D
    DMA2_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream6->CR & DMA_SxCR_EN);
    //DMA2->HIFCR = 0xFFFFFFFF;
    DMA2->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
    DMA2_Stream6->CR = (6 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_DIR_0;
    NVIC_EnableIRQ(DMA2_Stream6_IRQn);
    NVIC_SetPriority(DMA2_Stream6_IRQn, 3);

    // DMA2 Stream 1, Channel 7 (TIM8_UP) - PORT E
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream1->CR & DMA_SxCR_EN);
    //DMA2->LIFCR = 0xFFFFFFFF;
    DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;
    DMA2_Stream1->CR = (7 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_DIR_0;
    NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    NVIC_SetPriority(DMA2_Stream1_IRQn, 3);
}



#define iv 0xFFFFFFFF
static const uint32_t GCR_table[32] = {
    iv, iv, iv, iv, iv, iv, iv, iv, iv, 9, 10, 11, iv, 13, 14, 15,
    iv, iv, 2, 3, iv, 5, 6, 7, iv, 0, 8, 1, iv, 4, 12, iv };

uint32_t get_BDshot_response(uint32_t raw_buffer[], const uint8_t motor_shift)
{
    uint32_t* buffer_end = raw_buffer + 31 * BDSHOT_RESPONSE_BITRATE / 1000 * BDSHOT_RESPONSE_OVERSAMPLING;
    while (raw_buffer < buffer_end)
    {
        if (__builtin_expect(!(*raw_buffer++ & 1 << motor_shift), 0) ||
            __builtin_expect(!(*raw_buffer++ & 1 << motor_shift), 0) ||
            __builtin_expect(!(*raw_buffer++ & 1 << motor_shift), 0) ||
            __builtin_expect(!(*raw_buffer++ & 1 << motor_shift), 0))
        {
            uint32_t* buffer_previous = raw_buffer - 1;
            buffer_end = raw_buffer + BDSHOT_RESPONSE_LENGTH * BDSHOT_RESPONSE_OVERSAMPLING;
            uint32_t motor_response = 0;
            uint8_t bits = 0;
            while (raw_buffer <= buffer_end)
            {
                if (__builtin_expect((*raw_buffer++ & (1 << motor_shift)), 0) ||
                    __builtin_expect((*raw_buffer++ & (1 << motor_shift)), 0) ||
                    __builtin_expect((*raw_buffer++ & (1 << motor_shift)), 0) ||
                    __builtin_expect((*raw_buffer++ & (1 << motor_shift)), 0))
                {
                    if (raw_buffer <= buffer_end) {
                        uint8_t len = MAX((raw_buffer - buffer_previous) / BDSHOT_RESPONSE_OVERSAMPLING, 1);
                        bits += len;
                        motor_response <<= len;
                        buffer_previous = raw_buffer - 1;
                        while (raw_buffer < buffer_end)
                        {
                            if (__builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0) ||
                                __builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0) ||
                                __builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0) ||
                                __builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0)) {
                                if (raw_buffer <= buffer_end) {
                                    len = MAX((raw_buffer - buffer_previous) / BDSHOT_RESPONSE_OVERSAMPLING, 1);
                                    bits += len;
                                    motor_response <<= len;
                                    motor_response |= 0x1FFFFF >> (BDSHOT_RESPONSE_LENGTH - len);
                                    buffer_previous = raw_buffer - 1;
                                }
                                break;
                            }
                        }
                    }
                }
            }
            motor_response <<= (BDSHOT_RESPONSE_LENGTH - bits);
            if (*buffer_previous & (1 << motor_shift)) {
                motor_response |= 0x1FFFFF >> bits;
            }
            return motor_response;
        }
    }
    return 0xFFFFFFFF;
}

bool BDshot_check_checksum(uint32_t decoded_value) {
    uint8_t crc = (decoded_value & 0x0F);
    uint16_t value = (decoded_value >> 4);
    uint8_t calculated_crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
    return (crc == calculated_crc);
}

void read_BDshot_response(uint32_t value, uint8_t motor){
    if (value < 0xFFFFFFF) {
        value = (value ^ (value >> 1));

        uint32_t nibble1 = (value & 0x1F);
        uint32_t nibble2 = ((value >> 5) & 0x1F);
        uint32_t nibble3 = ((value >> 10) & 0x1F);
        uint32_t nibble4 = ((value >> 15) & 0x1F);

        if (GCR_table[nibble1] == iv || GCR_table[nibble2] == iv || GCR_table[nibble3] == iv || GCR_table[nibble4] == iv) {
            motor_telemetry_data[motor].valid_rpm = false;
            motor_telemetry_data[motor].valid_voltage = false;
            return;
        }

        uint32_t decoded_value = GCR_table[nibble1];
        decoded_value |= GCR_table[nibble2] << 4;
        decoded_value |= GCR_table[nibble3] << 8;
        decoded_value |= GCR_table[nibble4] << 12;

        if (BDshot_check_checksum(decoded_value))
        {
            motor_telemetry_data[motor].valid_rpm = true;
            motor_telemetry_data[motor].raw_rpm_value = ((decoded_value & 0x1FF0) >> 4) << (decoded_value >> 13);
            motor_telemetry_data[motor].raw_rpm_value = 60 * 1000000 / motor_telemetry_data[motor].raw_rpm_value * 2 / MOTOR_POLES_NUMBER;

        } else {
            motor_telemetry_data[motor].valid_rpm = false;
        }
    } else {
        motor_telemetry_data[motor].valid_rpm = false;
    }
}

void process_telemetry_with_new_method(void) {
    // Process telemetry from all pairs
    dshot_A_rx_processing();
    dshot_B_rx_processing();
    dshot_C_rx_processing();
    dshot_D_rx_processing();
    dshot_E_rx_processing();

    telemetry_done_flag = 0;
}


void pid_reset_all(void)
{
    for (int i = 0; i < MOTORS_COUNT; i++) {
        pid_states[i].i_term = 0.0f;
        pid_states[i].last_error = 0.0f;
        pid_states[i].filtered_error = 0.0f;
        pid_states[i].last_dshot_command = 0;
    }
}

void pid_reset_motor(uint8_t motor_index)
{
    if (motor_index < MOTORS_COUNT) {
        pid_states[motor_index].i_term = 0.0f;
        pid_states[motor_index].last_error = 0.0f;
        pid_states[motor_index].filtered_error = 0.0f;
        pid_states[motor_index].last_dshot_command = 0;
    }
}



uint32_t counter =0;
#define DEBUG_PRINT_MOTOR 1
#define DEBUG_CYCLE_DIVIDER 300  // adjust for 2000 Hz loop -> ~10 Hz debug


uint16_t pid_calculate_command(uint8_t motor_index,
                               uint32_t current_rpm_unsigned,
                               float target_rpm_signed,
                               float dt)
{
    if (motor_index >= MOTORS_COUNT || dt <= 0.0f)
        return 0;

    float current_rpm = (float)current_rpm_unsigned;
    float target_rpm = target_rpm_signed;

    uint8_t count = 0;
    if(target_rpm < 0.0f) {
        count = 2; // reverse
        target_rpm = -target_rpm;
    } else if(target_rpm > 0.0f) {
        count = 1; // forward
    }


    if (fabsf(target_rpm) > 0.0f) {
        if (current_rpm_unsigned == 0 && pid_states[motor_index].last_valid_rpm > 0.0f) {

            current_rpm = pid_states[motor_index].last_valid_rpm;
        }


        if (current_rpm_unsigned > 0) {
            pid_states[motor_index].last_valid_rpm = current_rpm;
        }
    }


    float error = target_rpm - current_rpm;


    float p_term = PID_KP * error;

    // Integral: per motor
    if (fabsf(target_rpm) > 0.0f && fabsf(error) < fabsf(target_rpm) * 1.2f) {
        pid_states[motor_index].i_term += PID_KI * error * dt;
        if(pid_states[motor_index].i_term > 500.0f) pid_states[motor_index].i_term = 500.0f;
        if(pid_states[motor_index].i_term < -500.0f) pid_states[motor_index].i_term = -500.0f;
    } else {
        pid_states[motor_index].i_term *= 0.95f;
    }


    float error_derivative = error - pid_states[motor_index].last_error;
    pid_states[motor_index].last_error = error;
    pid_states[motor_index].filtered_error = 0.4f * error_derivative + 0.6f * pid_states[motor_index].filtered_error;
    float d_term = PID_KD * pid_states[motor_index].filtered_error;

    float pid_output = p_term + pid_states[motor_index].i_term + d_term;


    float ff_from_target = fabsf(target_rpm) * 0.15f;
    float pid_correction = pid_output * 0.15f;
    float throttle_value = ff_from_target + pid_correction;


    const float DSHOT_NEUTRAL = 1048.0f; // Minimum DShot value for forward thrust
    const float DSHOT_MIN = 48.0f;       // Minimum DShot value for reverse thrust/idle
    const float DSHOT_MAX = 2047.0f;

    float dshot_f = 0;

    if(count == 1) {
        dshot_f = DSHOT_NEUTRAL + throttle_value; // forward
    } else if(count == 2) {
        dshot_f = DSHOT_MIN + throttle_value; // reverse
    } else {
        // Dead stop: reset all states
        pid_states[motor_index].i_term = 0.0f;
        pid_states[motor_index].last_error = 0.0f;
        pid_states[motor_index].filtered_error = 0.0f;
        pid_states[motor_index].last_valid_rpm = 0.0f; // Reset valid RPM on stop
        return 0;
    }


    // This section prevents the forward throttle from dropping into the reverse range.
    if(count == 1) {

        if(dshot_f < DSHOT_NEUTRAL) dshot_f = DSHOT_NEUTRAL;
    } else if (count == 2) {

        if(dshot_f < DSHOT_MIN) dshot_f = DSHOT_MIN;
    }


    if(dshot_f > DSHOT_MAX) dshot_f = DSHOT_MAX;


    uint16_t dshot_command = (uint16_t)(dshot_f + 0.5f);
    pid_states[motor_index].last_dshot_command = dshot_command;

    // --- Debug ---
    static uint16_t debug_counter = 0;
    if(motor_index == DEBUG_PRINT_MOTOR) {
        debug_counter++;
        if(debug_counter >= DEBUG_CYCLE_DIVIDER) {
            pid_debug.motor_index = motor_index;
            pid_debug.error = error;
            pid_debug.target = target_rpm;
            pid_debug.output = dshot_command;
            pid_debug.pending = 1;
            debug_counter = 0;
        }
    }

    return dshot_command;
}
