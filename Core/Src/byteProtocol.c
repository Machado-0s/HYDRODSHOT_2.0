#include "byteProtocol.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart1;

static uint8_t rx_dma_buffer[RX_BUFFER_SIZE];
static uint8_t ring_buffer[RING_BUFFER_SIZE];
static volatile uint16_t ring_head = 0;
static volatile uint16_t ring_tail = 0;

static volatile uint32_t packet_count   = 0;
static volatile uint32_t error_count    = 0;
static volatile uint32_t bytes_received = 0;



static uint16_t ring_available(void) {
    if (ring_head >= ring_tail) return ring_head - ring_tail;
    return RING_BUFFER_SIZE - ring_tail + ring_head;
}

static uint16_t ring_free(void) {
    return RING_BUFFER_SIZE - ring_available() - 1;
}

static void ring_push(const uint8_t *data, uint16_t len) {
    if (len > ring_free()) {
        error_count++;
        return;
    }
    for (uint16_t i = 0; i < len; i++) {
        ring_buffer[ring_head++] = data[i];
        if (ring_head == RING_BUFFER_SIZE) ring_head = 0;
    }
}


uint8_t Calculate_CRC8(const uint8_t *data, uint16_t length) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
    return crc;
}


static void process_complete_packet(uint8_t* buffer) {

    uint8_t received_crc = buffer[PACKET_SIZE - 1];
    uint8_t computed_crc = Calculate_CRC8(&buffer[HEADER_SIZE], DATA_SIZE);

    if (received_crc != computed_crc) {
        error_count++;
        return;
    }

    uint8_t *motor_bytes = &buffer[HEADER_SIZE];


    for (int i = 0; i < 10; i++) {
        uint8_t byte_val = motor_bytes[i];
        if (byte_val >= 100 && byte_val <= 200) {
            int32_t signed_val = (int32_t)byte_val - 150;
            int32_t rpm_int = (signed_val * DSHOT_MAX_RPM) / 50;
            ByteProtocol_DShotUpdateInt(i, rpm_int);
        }
    }


    for (int i = 0; i < 4; i++) {
        uint8_t pwm_byte = motor_bytes[10 + i];
        uint16_t pulse_us = 1000 + ((uint16_t)(pwm_byte - 100) * 1000 / 100);
        ByteProtocol_PWMUpdate(i, pulse_us);
    }

    packet_count++;
}


static void parse_ring_buffer(void) {

    while (ring_available() >= PACKET_SIZE) {

        uint16_t h1_idx = ring_tail;
        uint16_t h2_idx = (ring_tail + 1) % RING_BUFFER_SIZE;


        if (ring_buffer[h1_idx] != HEADER_BYTE_1 || ring_buffer[h2_idx] != HEADER_BYTE_2) {
            ring_tail = (ring_tail + 1) % RING_BUFFER_SIZE;
            continue;
        }


        uint8_t packet[PACKET_SIZE];
        for (int i = 0; i < PACKET_SIZE; i++) {
            packet[i] = ring_buffer[(ring_tail + i) % RING_BUFFER_SIZE];
        }

        process_complete_packet(packet);


        ring_tail = (ring_tail + PACKET_SIZE) % RING_BUFFER_SIZE;
    }
}


void ByteProtocol_Init(void) {
    memset(rx_dma_buffer, 0, sizeof(rx_dma_buffer));
    ring_head = ring_tail = 0;
    packet_count = 0;
    error_count = 0;
    bytes_received = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buffer, RX_BUFFER_SIZE);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance != USART1) return;
    bytes_received += Size;

    ring_push(rx_dma_buffer, Size);
    parse_ring_buffer();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) return;
    if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
        error_count++;
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
        ByteProtocol_Init();
    }
}


