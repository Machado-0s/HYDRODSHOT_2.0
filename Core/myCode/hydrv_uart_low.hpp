#pragma once

#include <cstdint>
#include <cstddef>

extern "C"
{
#include "stm32f4xx.h"
}

#include "hydrv_gpio_low.hpp"


namespace hydrv::UART
{
class UARTLow
{
public:
    struct UARTPreset
    {
        const uint32_t USARTx;

        const uint8_t GPIO_alt_func;

        const uint32_t RCC_APBENR_UARTxEN;
        const uint32_t RCC_address;

        const IRQn_Type USARTx_IRQn;

        unsigned mantissa;
        unsigned fraction;
    };

public:
    static constexpr UARTPreset USART1_115200_LOW{
        USART1_BASE, 7, RCC_APB2ENR_USART1EN, RCC_BASE + offsetof(RCC_TypeDef, APB2ENR), USART1_IRQn, 45, 9};

    static constexpr UARTPreset USART1_921600_LOW{
        USART1_BASE, 7, RCC_APB2ENR_USART1EN, RCC_BASE + offsetof(RCC_TypeDef, APB2ENR), USART1_IRQn, 5, 11};

    // USART2 added (APB1 - 42MHz)
    static constexpr UARTPreset USART2_115200_LOW{
        USART2_BASE, 7, RCC_APB1ENR_USART2EN, RCC_BASE + offsetof(RCC_TypeDef, APB1ENR), USART2_IRQn, 22, 13};

    static constexpr UARTPreset USART2_921600_LOW{
        USART2_BASE, 7, RCC_APB1ENR_USART2EN, RCC_BASE + offsetof(RCC_TypeDef, APB1ENR), USART2_IRQn, 2, 14};

    static constexpr UARTPreset USART3_115200_LOW{
        USART3_BASE, 7, RCC_APB1ENR_USART3EN, RCC_BASE + offsetof(RCC_TypeDef, APB1ENR), USART3_IRQn, 22, 13};

    static constexpr UARTPreset USART3_921600_LOW{
        USART3_BASE, 7, RCC_APB1ENR_USART3EN, RCC_BASE + offsetof(RCC_TypeDef, APB1ENR), USART3_IRQn, 2, 14};

public:
    consteval UARTLow(const UARTPreset &preset, hydrv::GPIO::GPIOLow &rx_pin,
                      hydrv::GPIO::GPIOLow &tx_pin, unsigned IRQ_priority);

public:
    void Init();
    bool IsRxDone();
    bool IsTxDone();

    uint8_t GetRx();
    void SetTx(uint8_t byte);

    void EnableTxInterruption();
    void DisableTxInterruption();
    void EnableRxInterruption();
    void DisableRxInterruption();

    void EnableDMATransmit();
    void EnableDMAReceive();

private:
    static constexpr uint32_t CountCR1Mask_();
    static constexpr uint32_t CountCR2Mask_();
    static constexpr uint32_t CountBRRMask_(const UARTPreset &preset);
    
    static void EnableUARTClock_(uint32_t rcc_address, uint32_t en_bit);
    static constexpr uint32_t USARTBRRDIVFractionVal_(uint32_t val);
    static constexpr uint32_t USARTBRRDIVMantissaVal_(uint32_t val);
    static constexpr uint32_t USARTCR2Stop1bit_();

private:
    UARTPreset preset_;
    unsigned IRQ_priority_;
    hydrv::GPIO::GPIOLow &rx_pin_;
    hydrv::GPIO::GPIOLow &tx_pin_;

    const uint32_t cr1_;
    const uint32_t cr2_;
    const uint32_t brr_;
};

consteval UARTLow::UARTLow(const UARTPreset &preset,
                           hydrv::GPIO::GPIOLow &rx_pin,
                           hydrv::GPIO::GPIOLow &tx_pin, unsigned IRQ_priority)
    : preset_(preset),
      IRQ_priority_(IRQ_priority),
      rx_pin_(rx_pin),
      tx_pin_(tx_pin),
      cr1_(CountCR1Mask_()),
      cr2_(CountCR2Mask_()),
      brr_(CountBRRMask_(preset))
{
}

inline void UARTLow::Init()
{
    EnableUARTClock_(preset_.RCC_address, preset_.RCC_APBENR_UARTxEN);
    NVIC_SetPriority(preset_.USARTx_IRQn, IRQ_priority_);
    NVIC_EnableIRQ(preset_.USARTx_IRQn);

    CLEAR_BIT(reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->CR1, USART_CR1_UE);

    reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->CR1 = cr1_;
    reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->CR2 = cr2_;
    reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->BRR = brr_;

    SET_BIT(reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->CR1, USART_CR1_UE);

    rx_pin_.Init(preset_.GPIO_alt_func);
    tx_pin_.Init(preset_.GPIO_alt_func);
}

inline bool UARTLow::IsRxDone() { return READ_BIT(reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->SR, USART_SR_RXNE); }
inline bool UARTLow::IsTxDone() { return READ_BIT(reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->SR, USART_SR_TC); }
inline uint8_t UARTLow::GetRx() { return reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->DR; }
inline void UARTLow::SetTx(uint8_t byte) { reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->DR = byte; }
inline void UARTLow::EnableTxInterruption()
{
    SET_BIT(reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->CR1, USART_CR1_TCIE);
}
inline void UARTLow::DisableTxInterruption()
{
    CLEAR_BIT(reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->CR1, USART_CR1_TCIE);
}
inline void UARTLow::EnableRxInterruption()
{
    SET_BIT(reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->CR1, USART_CR1_RXNEIE);
}
inline void UARTLow::DisableRxInterruption()
{
    CLEAR_BIT(reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->CR1, USART_CR1_RXNEIE);
}

inline void UARTLow::EnableDMATransmit()
{
    SET_BIT(reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->CR3, USART_CR3_DMAT);
}

inline void UARTLow::EnableDMAReceive()
{
    SET_BIT(reinterpret_cast<USART_TypeDef*>(preset_.USARTx)->CR3, USART_CR3_DMAR);
}

constexpr uint32_t UARTLow::CountCR1Mask_()
{
    uint32_t cr1 = 0;
    CLEAR_BIT(cr1, USART_CR1_M);     // 8 bits including parity
    CLEAR_BIT(cr1, USART_CR1_PCE);   // parity disable
    SET_BIT(cr1, USART_CR1_PS);      // odd parity
    CLEAR_BIT(cr1, USART_CR1_OVER8); // 16-bit oversampling
    SET_BIT(cr1, USART_CR1_TE);
    SET_BIT(cr1, USART_CR1_RE);
    SET_BIT(cr1, USART_CR1_RXNEIE);

    return cr1;
}

constexpr uint32_t UARTLow::CountCR2Mask_()
{
    uint32_t cr2 = 0;
    MODIFY_REG(cr2, USART_CR2_STOP, USARTCR2Stop1bit_());
    return cr2;
}

constexpr uint32_t UARTLow::CountBRRMask_(const UARTPreset &preset)
{
    uint32_t brr = 0;
    MODIFY_REG(brr, USART_BRR_DIV_Fraction,
               USARTBRRDIVFractionVal_(preset.fraction));
    MODIFY_REG(brr, USART_BRR_DIV_Mantissa,
               USARTBRRDIVMantissaVal_(preset.mantissa));
    return brr;
}

inline void UARTLow::EnableUARTClock_(uint32_t rcc_address, uint32_t en_bit)
{
    volatile uint32_t* rcc_reg = reinterpret_cast<volatile uint32_t*>(rcc_address);
    __IO uint32_t tmpreg = 0x00U;
    SET_BIT(*rcc_reg, en_bit); /* Delay after an RCC peripheral clock enabling */
    tmpreg = READ_BIT(*rcc_reg, en_bit);
    (void)tmpreg;
}

constexpr uint32_t UARTLow::USARTBRRDIVFractionVal_(uint32_t val)
{
    return val << USART_BRR_DIV_Fraction_Pos;
}

constexpr uint32_t UARTLow::USARTBRRDIVMantissaVal_(uint32_t val)
{
    return val << USART_BRR_DIV_Mantissa_Pos;
}

constexpr uint32_t UARTLow::USARTCR2Stop1bit_()
{
    return 0x0UL << USART_CR2_STOP_Pos;
}

} // namespace hydrv::UART