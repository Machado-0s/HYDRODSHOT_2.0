#pragma once

#include <cstring>

#include "hydrolib_common.h"
#include "hydrolib_func_concepts.hpp"

#include "hydrv_uart_low.hpp"

namespace hydrv::UART
{
template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY,
          typename CallbackType =
              decltype(&hydrolib::concepts::func::DummyFunc<void>)>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
class UART
{
private:
    static constexpr unsigned REAL_RX_BUFFER_CAPACITY_ = RX_BUFFER_CAPACITY + 1;
    static constexpr unsigned REAL_TX_BUFFER_CAPACITY_ = TX_BUFFER_CAPACITY + 1;

public:
    consteval UART(const UARTLow::UARTPreset &UART_preset,
                   hydrv::GPIO::GPIOLow &rx_pin, hydrv::GPIO::GPIOLow &tx_pin,
                   unsigned IRQ_priority,
                   CallbackType rx_callback = hydrolib::concepts::func::DummyFunc<void>);

    void Init();

    void IRQCallback();

    int Transmit(const void *data, unsigned data_length);

    int Read(void *data, unsigned data_length);
    void ClearRx();

    unsigned GetRxLength() const;
    unsigned GetTxLength() const;

private:
    void ProcessRx_();
    void ProcessTx_();

private:
    UARTLow UART_handler_;

    uint8_t rx_buffer_[REAL_RX_BUFFER_CAPACITY_];
    unsigned rx_head_;
    volatile unsigned rx_tail_;

    uint8_t tx_buffer_[REAL_TX_BUFFER_CAPACITY_];
    volatile unsigned tx_head_;
    unsigned tx_tail_;

    hydrolib_ReturnCode status_;

    CallbackType rx_callback_;
};

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
int read(UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType> &stream,
         void *dest, unsigned length);

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
int write(UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType> &stream,
          const void *dest, unsigned length);

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
consteval UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::UART(
    const UARTLow::UARTPreset &UART_preset, hydrv::GPIO::GPIOLow &rx_pin,
    hydrv::GPIO::GPIOLow &tx_pin, unsigned IRQ_priority,
    CallbackType rx_callback)
    : UART_handler_(UART_preset, rx_pin, tx_pin, IRQ_priority),
      rx_buffer_{},
      rx_head_(0),
      rx_tail_(0),
      tx_buffer_{},
      tx_head_(0),
      tx_tail_(0),
      status_(HYDROLIB_RETURN_OK),
      rx_callback_(rx_callback)
{
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::Init()
{
    UART_handler_.Init();
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::IRQCallback()
{
    ProcessRx_();
    ProcessTx_();
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
int UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::Transmit(
    const void *data, unsigned data_length)
{
    unsigned length = GetTxLength();

    if (length + data_length > TX_BUFFER_CAPACITY)
    {
        data_length = TX_BUFFER_CAPACITY - length;
    }

    unsigned forward_length = REAL_TX_BUFFER_CAPACITY_ - tx_tail_;
    if (forward_length >= data_length)
    {
        memcpy(tx_buffer_ + tx_tail_, data, data_length);
    }
    else
    {
        memcpy(tx_buffer_ + tx_tail_, data, forward_length);
        memcpy(tx_buffer_, static_cast<const uint8_t *>(data) + forward_length,
               data_length - forward_length);
    }
    tx_tail_ = (tx_tail_ + data_length) % REAL_TX_BUFFER_CAPACITY_;

    UART_handler_.EnableTxInterruption();

    return data_length;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
int UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::Read(
    void *data, unsigned data_length)
{
    unsigned length = GetRxLength();

    if (data_length > length)
    {
        data_length = length;
    }
    unsigned forward_length = RX_BUFFER_CAPACITY - rx_head_;
    if (data_length > forward_length)
    {
        memcpy(data, rx_buffer_ + rx_head_, forward_length);
        memcpy(static_cast<uint8_t *>(data) + forward_length, rx_buffer_,
               data_length - forward_length);
    }
    else
    {
        memcpy(data, rx_buffer_ + rx_head_, data_length);
    }
    rx_head_ = (rx_head_ + data_length) % REAL_RX_BUFFER_CAPACITY_;
    return data_length;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::ClearRx()
{
    rx_head_ = rx_tail_;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
unsigned
UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::GetRxLength() const
{
    unsigned tail = rx_tail_;
    if (tail >= rx_head_)
    {
        return tail - rx_head_;
    }
    else
    {
        return tail + REAL_RX_BUFFER_CAPACITY_ - rx_head_;
    }
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
unsigned
UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::GetTxLength() const
{
    unsigned head = tx_head_;
    if (tx_tail_ >= head)
    {
        return tx_tail_ - head;
    }
    else
    {
        return tx_tail_ + REAL_TX_BUFFER_CAPACITY_ - head;
    }
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::ProcessRx_()
{
    if (!UART_handler_.IsRxDone())
    {
        return;
    }

    unsigned next_tail = (rx_tail_ + 1) % REAL_RX_BUFFER_CAPACITY_;

    if (next_tail == rx_head_)
    {
        UART_handler_.GetRx();
        status_ = HYDROLIB_RETURN_FAIL;
        rx_callback_();
        return;
    }

    rx_buffer_[rx_tail_] = UART_handler_.GetRx();
    rx_tail_ = next_tail;

    rx_callback_();
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
requires hydrolib::concepts::func::FuncConcept<CallbackType, void>
void UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType>::ProcessTx_()
{
    if (!UART_handler_.IsTxDone())
    {
        return;
    }

    if (tx_head_ == tx_tail_)
    {
        UART_handler_.DisableTxInterruption();
        return;
    }

    UART_handler_.SetTx(tx_buffer_[tx_head_]);
    tx_head_ = (tx_head_ + 1) % REAL_TX_BUFFER_CAPACITY_;
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
int read(UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType> &stream,
         void *dest, unsigned length)
{
    return stream.Read(dest, length);
}

template <int RX_BUFFER_CAPACITY, int TX_BUFFER_CAPACITY, typename CallbackType>
int write(UART<RX_BUFFER_CAPACITY, TX_BUFFER_CAPACITY, CallbackType> &stream,
          const void *dest, unsigned length)
{
    return stream.Transmit(dest, length);
}

} // namespace hydrv::UART
