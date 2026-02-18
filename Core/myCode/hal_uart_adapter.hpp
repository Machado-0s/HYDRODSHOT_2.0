/*
 * hal_uart_adapter.hpp
 *
 *  Created on: 16 Feb 2026
 *      Author: Chard Ethern
 *
 *  
 */

#ifndef MYCODE_HAL_UART_ADAPTER_HPP_
#define MYCODE_HAL_UART_ADAPTER_HPP_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdint.h>

#define UART_RX_DMA_BUFFER_SIZE    256    
#define UART_RING_BUFFER_SIZE      512    


static uint8_t  uart_rx_dma_buffer[UART_RX_DMA_BUFFER_SIZE];
static uint8_t  uart_ring_buffer[UART_RING_BUFFER_SIZE];
static volatile uint16_t uart_ring_head = 0;
static volatile uint16_t uart_ring_tail = 0;

static volatile uint32_t uart_bytes_received = 0;
static volatile uint32_t uart_error_count    = 0;

static inline uint16_t ring_available(void) {
    if (uart_ring_head >= uart_ring_tail) {
        return uart_ring_head - uart_ring_tail;
    }
    return UART_RING_BUFFER_SIZE - uart_ring_tail + uart_ring_head;
}

static inline uint16_t ring_free(void) {
    return UART_RING_BUFFER_SIZE - ring_available() - 1;
}

static void ring_push(const uint8_t *data, uint16_t len) {
    if (len > ring_free()) {
        uart_error_count++;
        return;
    }
    for (uint16_t i = 0; i < len; i++) {
        uart_ring_buffer[uart_ring_head++] = data[i];
        if (uart_ring_head == UART_RING_BUFFER_SIZE) uart_ring_head = 0;
    }
}



int read(UART_HandleTypeDef &huart, void *dest, unsigned length)
{
    (void)huart;  

    uint16_t avail = ring_available();
    if (avail == 0) return 0;

    unsigned to_read = (length < avail) ? length : avail;

    // Copy from ring buffer to destination
    for (unsigned i = 0; i < to_read; i++) {
        ((uint8_t*)dest)[i] = uart_ring_buffer[uart_ring_tail++];
        if (uart_ring_tail == UART_RING_BUFFER_SIZE) uart_ring_tail = 0;
    }

    return static_cast<int>(to_read);
}



/*int write(UART_HandleTypeDef &huart, const void *src, unsigned length)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit(
        &huart,
        const_cast<uint8_t*>(static_cast<const uint8_t*>(src)),
        length,
        HAL_MAX_DELAY
    );

    if (status == HAL_OK) return static_cast<int>(length);
    return -1;
}
*/

static volatile bool uart_tx_done = true;


int write(UART_HandleTypeDef &huart, const void *src, unsigned length)
{
    if (length == 0)
        return 0;
      

    uart_tx_done = false;

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
        &huart,
        const_cast<uint8_t*>(static_cast<const uint8_t*>(src)),
        length
    );

    if (status != HAL_OK)
    {
        uart_tx_done = true;
        return -1;
    }
    
    return static_cast<int>(length);
}


void HalUartAdapter_Init(UART_HandleTypeDef *huart)
{
    memset(uart_rx_dma_buffer, 0, sizeof(uart_rx_dma_buffer));
    memset(uart_ring_buffer,   0, sizeof(uart_ring_buffer));
    uart_ring_head = uart_ring_tail = 0;
    uart_bytes_received = 0;
    uart_error_count = 0;

    uart_tx_done = true;
   
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_rx_dma_buffer, UART_RX_DMA_BUFFER_SIZE);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART1) return;

    uart_bytes_received += Size;

    ring_push(uart_rx_dma_buffer, Size);

}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
        uart_error_count++;
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
       
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_rx_dma_buffer, UART_RX_DMA_BUFFER_SIZE);
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_tx_done = true;
    }
}
#endif /* MYCODE_HAL_UART_ADAPTER_HPP_ */
