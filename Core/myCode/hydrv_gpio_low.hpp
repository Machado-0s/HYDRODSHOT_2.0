#pragma once

#include <cstddef>
#include <cstdint>

extern "C"
{
#include "stm32f4xx.h"

#include "hydrolib_common.h"
}

namespace hydrv::GPIO
{
class GPIOLow
{
public:
    enum GPIOFunc
    {
        OUTPUT = 0,
        UART,
        TIMER
    };
    struct GPIOPort
    {
    public:
        static constexpr std::size_t PIN_COUNT = 16;

    public:
        const uint32_t GPIOx_;
        const uint32_t RCC_AHB1ENR_GPIOxEN_;

        bool *const inited_pins_;
    };

    struct GPIOPreset
    {
        const GPIOFunc pin_function;
    };

private:
    static constinit bool GPIOA_inited_pins_[GPIOPort::PIN_COUNT];
    static constinit bool GPIOB_inited_pins_[GPIOPort::PIN_COUNT];
    static constinit bool GPIOC_inited_pins_[GPIOPort::PIN_COUNT];
    static constinit bool GPIOD_inited_pins_[GPIOPort::PIN_COUNT];

public:
    static constexpr GPIOPort GPIOA_port{GPIOA_BASE, RCC_AHB1ENR_GPIOAEN,
                                         GPIOA_inited_pins_};
    static constexpr GPIOPort GPIOB_port{GPIOB_BASE, RCC_AHB1ENR_GPIOBEN,
                                         GPIOB_inited_pins_};
    static constexpr GPIOPort GPIOC_port{GPIOC_BASE, RCC_AHB1ENR_GPIOCEN,
                                         GPIOC_inited_pins_};
    static constexpr GPIOPort GPIOD_port{GPIOD_BASE, RCC_AHB1ENR_GPIODEN,
                                         GPIOD_inited_pins_};
    static constexpr GPIOPreset GPIO_Output{OUTPUT};
    static constexpr GPIOPreset GPIO_UART_TX{UART};
    static constexpr GPIOPreset GPIO_UART_RX{UART};
    static constexpr GPIOPreset GPIO_Timer{TIMER};

public:
    consteval GPIOLow(const GPIOPort &GPIO_group, unsigned pin,
                      GPIOPreset preset);

public:
    hydrolib_ReturnCode Init(uint32_t altfunc);

    bool IsInited();

    void Set();

    void Reset();

private:
    bool &is_inited_;
    const uint32_t GPIOx_;
    const unsigned pin_;
    const GPIOFunc pin_func_;
    const uint32_t RCC_AHB1ENR_GPIOxEN_;

    const uint32_t output_speed_reg_mask_;
    const uint32_t output_speed_reg_value_low_;
    const uint32_t output_speed_reg_value_very_high_;

    const uint32_t output_type_reg_mask_;

    const uint32_t push_pull_reg_mask_;
    const uint32_t push_pull_reg_value_no_;

    const uint32_t mode_reg_mask_;
    const uint32_t mode_reg_value_output_;
    const uint32_t mode_reg_value_altfunc_;

    const uint32_t altfunc_reg_low_mask_;
    const uint32_t altfunc_reg_high_mask_;

    const uint32_t set_reg_mask_;
    const uint32_t reset_reg_mask_;

private:
    static void EnableGPIOxClock_(const uint32_t RCC_AHB1ENR_GPIOxEN);

    void SetPinConfig_(uint32_t speed, bool is_open_drain, uint32_t push_pull);

    void SetModeOutput_();

    void SetModeAltfunc_(uint32_t altfunc);

    uint32_t GetAltfuncRegLowValue_(uint32_t func);

    uint32_t GetAltfuncRegHighValue_(uint32_t func);
};

constinit inline bool GPIOLow::GPIOA_inited_pins_[GPIOPort::PIN_COUNT] = {};
constinit inline bool GPIOLow::GPIOB_inited_pins_[GPIOPort::PIN_COUNT] = {};
constinit inline bool GPIOLow::GPIOC_inited_pins_[GPIOPort::PIN_COUNT] = {};
constinit inline bool GPIOLow::GPIOD_inited_pins_[GPIOPort::PIN_COUNT] = {};

consteval inline GPIOLow::GPIOLow(const GPIOPort &GPIO_group, unsigned pin,
                                  GPIOPreset preset)
    : is_inited_(GPIO_group.inited_pins_[pin]),
      GPIOx_(GPIO_group.GPIOx_),
      pin_(pin),
      pin_func_(preset.pin_function),
      RCC_AHB1ENR_GPIOxEN_(GPIO_group.RCC_AHB1ENR_GPIOxEN_),
      output_speed_reg_mask_(0x3UL << (2 * pin)),
      output_speed_reg_value_low_(0x0UL),
      output_speed_reg_value_very_high_(0x3UL << (2 * pin)),
      output_type_reg_mask_(0x1UL << pin),
      push_pull_reg_mask_(0x3UL << (2 * pin)),
      push_pull_reg_value_no_(0x0UL),
      mode_reg_mask_(0x3UL << (2 * pin)),
      mode_reg_value_output_(0x1UL << (2 * pin)),
      mode_reg_value_altfunc_(0x2UL << (2 * pin)),
      altfunc_reg_low_mask_(pin < 8 ? 0xFUL << (4 * pin) : 0x0UL),
      altfunc_reg_high_mask_(pin < 8 ? 0x0UL : 0xFUL << (4 * (pin - 8))),
      set_reg_mask_(0x1UL << pin),
      reset_reg_mask_(0x1UL << (pin + GPIO_BSRR_BR0_Pos))
{
}

inline hydrolib_ReturnCode GPIOLow::Init(uint32_t altfunc = 0)
{
    if (is_inited_)
    {
        return HYDROLIB_RETURN_FAIL;
    }

    EnableGPIOxClock_(RCC_AHB1ENR_GPIOxEN_);

    switch (pin_func_)
    {
    case UART:
        SetPinConfig_(output_speed_reg_value_very_high_, false,
                      push_pull_reg_value_no_);
        SetModeAltfunc_(altfunc);
        break;
    case TIMER:
        SetPinConfig_(output_speed_reg_value_low_, false,
                      push_pull_reg_value_no_);
        SetModeAltfunc_(altfunc);
        break;
    case OUTPUT:
        SetPinConfig_(output_speed_reg_value_low_, false,
                      push_pull_reg_value_no_);
        SetModeOutput_();
        break;
    default:
        return HYDROLIB_RETURN_FAIL;
    }
    // TODO move to constructor output_speed_reg_value_low_, false,
    // push_pull_reg_value_no_
    is_inited_ = true;

    return HYDROLIB_RETURN_OK;
}

inline bool GPIOLow::IsInited() { return is_inited_; }

inline void GPIOLow::Set()
{
    CLEAR_BIT(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->BSRR, reset_reg_mask_);
    SET_BIT(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->BSRR, set_reg_mask_);
}

inline void GPIOLow::Reset()
{
    CLEAR_BIT(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->BSRR, set_reg_mask_);
    SET_BIT(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->BSRR, reset_reg_mask_);
}

inline void GPIOLow::EnableGPIOxClock_(const uint32_t RCC_AHB1ENR_GPIOxEN)
{
    __IO uint32_t tmpreg = 0x00U;
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOxEN); /* Delay after an RCC peripheral
                                                   clock enabling */
    tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOxEN);
    (void)tmpreg;
}

inline void GPIOLow::SetPinConfig_(uint32_t speed, bool is_open_drain,
                                   uint32_t push_pull)
{
    MODIFY_REG(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->OSPEEDR, output_speed_reg_mask_, speed);
    if (is_open_drain)
    {
        SET_BIT(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->OTYPER, output_type_reg_mask_);
    }
    else
    {
        CLEAR_BIT(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->OTYPER, output_type_reg_mask_);
    }
    MODIFY_REG(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->PUPDR, push_pull_reg_mask_, push_pull);
}

inline void GPIOLow::SetModeOutput_()
{
    MODIFY_REG(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->MODER, mode_reg_mask_, mode_reg_value_output_);
}

inline void GPIOLow::SetModeAltfunc_(uint32_t altfunc)
{
    if (pin_ > 7)
    {
        MODIFY_REG(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->AFR[1], altfunc_reg_high_mask_,
                   GetAltfuncRegHighValue_(altfunc));
    }
    else
    {
        MODIFY_REG(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->AFR[0], altfunc_reg_low_mask_,
                   GetAltfuncRegLowValue_(altfunc));
    }

    MODIFY_REG(reinterpret_cast<GPIO_TypeDef*>(GPIOx_)->MODER, mode_reg_mask_, mode_reg_value_altfunc_);
}

inline uint32_t GPIOLow::GetAltfuncRegLowValue_(uint32_t func)
{
    return func << (4 * pin_);
}

inline uint32_t GPIOLow::GetAltfuncRegHighValue_(uint32_t func)
{
    return func << (4 * (pin_ - 8));
}

} // namespace hydrv::GPIO
