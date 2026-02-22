#include "hydrolib_return_codes.hpp"
#include "hydrv_clock.hpp"
#include "hydrv_gpio_low.hpp"
#include "hydrv_uart.hpp"

#include "hydrolib_bus_application_slave.hpp"
#include "hydrolib_bus_datalink_stream.hpp"
#include "hydrolib_log_distributor.hpp"

#include <cstring>

#include <chrono>
#include <ctime>

#define BUFFER_LENGTH 5

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
    uint8_t buffer_[BUFFER_LENGTH];
};

// constinit hydrv::clock::Clock clock(hydrv::clock::Clock::HSI_DEFAULT);
constinit hydrv::GPIO::GPIOLow led_pin(hydrv::GPIO::GPIOLow::GPIOD_port, 15,
                                       hydrv::GPIO::GPIOLow::GPIO_Output);
constinit hydrv::GPIO::GPIOLow rx_pin1(hydrv::GPIO::GPIOLow::GPIOB_port, 7,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin1(hydrv::GPIO::GPIOLow::GPIOB_port, 6,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255>
    uart1(hydrv::UART::UARTLow::USART1_115200_LOW, rx_pin1, tx_pin1, 7);

constinit hydrv::GPIO::GPIOLow rx_pin3(hydrv::GPIO::GPIOLow::GPIOB_port, 11,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_RX);
constinit hydrv::GPIO::GPIOLow tx_pin3(hydrv::GPIO::GPIOLow::GPIOB_port, 10,
                                       hydrv::GPIO::GPIOLow::GPIO_UART_TX);
constinit hydrv::UART::UART<255, 255>
    uart3(hydrv::UART::UARTLow::USART3_115200_LOW, rx_pin3, tx_pin3, 7);

constinit hydrolib::logger::LogDistributor<decltype(uart3)>
    distributor("[%s] [%l] %m\n\r", uart3);
// hydrolib::logger::LogDistributor distributor("%m", uart3);
constinit hydrolib::logger::Logger<decltype(distributor)>
    logger("SerialProtocol", 1, distributor);

hydrolib::bus::datalink::StreamManager manager(1, uart1, logger);
hydrolib::bus::datalink::Stream stream(manager, 2);

Memory memory;

hydrolib::bus::application::Slave slave(stream, memory, logger);

int main(void)
{
    hydrv::clock::Clock::Init(hydrv::clock::Clock::HSI_DEFAULT);
    NVIC_SetPriorityGrouping(0);
    led_pin.Init();
    uart1.Init();
    uart3.Init();

    while (1)
    {
        manager.Process();
        slave.Process();
        uint8_t byte;
        if (memory.Read(&byte, 0, 1) == hydrolib::ReturnCode::OK)
        {
            if (byte == 'a')
            {
                led_pin.Set();
            }
            else
            {
                led_pin.Reset();
            }
        }
    }
}

extern "C"
{
    void SysTick_Handler(void) { hydrv::clock::Clock::SysTickHandler(); }
    void USART3_IRQHandler(void) { uart3.IRQCallback(); }
    void USART1_IRQHandler(void) { uart1.IRQCallback(); }
}
