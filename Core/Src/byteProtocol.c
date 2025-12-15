/*
 * byteProtocol.c
 *
 *  Created on: Dec 8, 2025
 *      Author: Chard Ethern
 */

#include "byteProtocol.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "dshot.h"

extern UART_HandleTypeDef huart1;

/* ==================== CONFIG ==================== */

#define MAX_SCAN_BYTES   256//64   /* Hard ISR CPU budget */

/* ==================== RX DMA ==================== */

static uint8_t  rx_dma_buffer[RX_BUFFER_SIZE];
static uint16_t last_dma_read_idx = 0;

/* ==================== STATS ==================== */

static volatile uint32_t packet_count   = 0;
static volatile uint32_t error_count    = 0;
static volatile uint32_t bytes_received = 0;

/* ==================== DEBUG ==================== */

static char debug_buffer[128];


/* ==================== INTERNAL ==================== */

static void process_complete_packet_from_buffer(const uint8_t* buffer);

/* ================================================= */

/* ================================================= */
/* ================= DEBUG TX ====================== */
/* ================================================= */

void Debug_Print(const char* format, ...)
{
    if (debug_tx_busy)
        return;

    va_list args;
    va_start(args, format);
    int len = vsnprintf(debug_buffer, sizeof(debug_buffer), format, args);
    va_end(args);

    if (len > 0 && len < (int)sizeof(debug_buffer)) {
        debug_tx_busy = true;
        HAL_UART_Transmit_IT(&huart1, (uint8_t*)debug_buffer, len);
    }
}

/* ================================================= */

static void process_complete_packet_from_buffer(const uint8_t* buffer)
{
    const uint8_t *motor_bytes = buffer + HEADER_SIZE;

    for (int i = 0; i < MOTORS_COUNT; i++) {

        uint8_t byte_val = motor_bytes[i];
        int32_t rpm_int = 0;

        if (byte_val >= 100 && byte_val <= 200) {
            int32_t signed_val = (int32_t)byte_val - 150;

            if (signed_val > 0) {
                rpm_int = (signed_val * DSHOT_MAX_RPM) / 50;
                if (rpm_int > DSHOT_MAX_RPM) rpm_int = DSHOT_MAX_RPM;
            } else if (signed_val < 0) {
                rpm_int = (signed_val * DSHOT_MAX_RPM) / 50;
                if (rpm_int < -DSHOT_MAX_RPM) rpm_int = -DSHOT_MAX_RPM;
            }
        } else {
            continue;
        }

        ByteProtocol_DShotUpdateInt(i, rpm_int);
    }

    for (int i = 0; i < 4; i++) {
        uint8_t pwm_byte = motor_bytes[10 + i];
        uint16_t pulse_us =  1000 + ((uint16_t)(pwm_byte-100) * 1000 / 100);
        ByteProtocol_PWMUpdate(i, pulse_us);
    }

    packet_count++;
}
/* ================================================= */

void ByteProtocol_Init(void)
{
    memset(rx_dma_buffer, 0, sizeof(rx_dma_buffer));

    last_dma_read_idx = 0;
    packet_count     = 0;
    error_count      = 0;
    bytes_received   = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buffer, RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/* ================================================= */
/* ================= IDLE ISR ====================== */
/* ================================================= */

void ByteProtocol_IdleLineCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    uint16_t dma_write_idx =
        RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    uint16_t unread =
        (dma_write_idx >= last_dma_read_idx) ?
        (dma_write_idx - last_dma_read_idx) :
        (RX_BUFFER_SIZE - last_dma_read_idx + dma_write_idx);

    bytes_received += unread;

    /* ================= OVERFLOW DETECTION ================= */

    if (unread > (RX_BUFFER_SIZE - PACKET_SIZE)) {
        error_count++;
        last_dma_read_idx = dma_write_idx;
        GPIOC->ODR |= GPIO_ODR_OD14;
        return;
    }

    /* ================= BACKWARD HEAD-CHASE SCAN ================= */

    uint16_t scan_count = unread;
    if (scan_count > MAX_SCAN_BYTES)
        scan_count = MAX_SCAN_BYTES;

    int32_t scan_idx = (int32_t)dma_write_idx - 1;
    if (scan_idx < 0)
        scan_idx += RX_BUFFER_SIZE;

    uint16_t packet_start = 0xFFFF;

    while (scan_count--) {

        uint16_t idx = (uint16_t)scan_idx;

        /* Fast reject */
        if (rx_dma_buffer[idx] == PROTOCOL_HEADER_1) {

            uint16_t idx2 = (idx + 1) % RX_BUFFER_SIZE;

            if (rx_dma_buffer[idx2] == PROTOCOL_HEADER_2) {

                /* Ensure full packet is already written */
                uint16_t packet_end =
                    (idx + PACKET_SIZE) % RX_BUFFER_SIZE;

                bool packet_complete =
                    (dma_write_idx > idx) ?
                    (dma_write_idx >= idx + PACKET_SIZE) :
                    (packet_end <= dma_write_idx);

                if (packet_complete) {
                    packet_start = idx;
                    break;  /* NEWEST packet found */
                }
            }
        }

        scan_idx--;
        if (scan_idx < 0)
            scan_idx += RX_BUFFER_SIZE;
    }

    /* ================= APPLY LATEST PACKET ================= */

    if (packet_start != 0xFFFF) {

        uint8_t packet_data[PACKET_SIZE];

        for (int i = 0; i < PACKET_SIZE; i++) {
            packet_data[i] =
                rx_dma_buffer[(packet_start + i) % RX_BUFFER_SIZE];
        }

        process_complete_packet_from_buffer(packet_data);

        last_dma_read_idx =
            (packet_start + PACKET_SIZE) % RX_BUFFER_SIZE;

        GPIOC->ODR &= ~GPIO_ODR_OD14;

    } else {
        /* Nothing valid found → drop unread */
        last_dma_read_idx = dma_write_idx;
        error_count++;
    }
}


/* ================================================= */
/* ================= UART ERR ====================== */
/* ================================================= */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    if (huart->ErrorCode & HAL_UART_ERROR_ORE) {

        error_count++;

        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);

        last_dma_read_idx = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(
            huart, rx_dma_buffer, RX_BUFFER_SIZE);

        GPIOC->ODR |= GPIO_ODR_OD14;
    }
}

