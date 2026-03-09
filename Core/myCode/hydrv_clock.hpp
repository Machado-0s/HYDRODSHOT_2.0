#pragma once

#include <cstdint>
#include <stdbool.h>
#include <stdint.h>

#include "hydrolib_return_codes.hpp"

extern "C"
{
#include "stm32f407xx.h"
#include "stm32f4xx.h"
}

namespace hydrv::clock
{

class Clock
{
public:
    enum PLLsource
    {
        HSE = RCC_PLLCFGR_PLLSRC_HSE,
        HSI = RCC_PLLCFGR_PLLSRC_HSI
    };

    struct ClockPreset
    {
        enum PLLsource source;
        uint32_t M;
        uint32_t N;
        uint32_t P;
        unsigned frequency_hse_mhz;
    };

    static constexpr ClockPreset HSI_DEFAULT{
        .source = HSI,
        .M = 8,
        .N = 168,
        .P = 2,
        .frequency_hse_mhz = 0,
    };

    static constexpr ClockPreset HSE_DEFAULT{
        .source = HSE,
        .M = 4,
        .N = 168,
        .P = 2,
        .frequency_hse_mhz = 8,
    };

    static constexpr unsigned TIMEOUT_MS = 1000;

    static hydrolib::ReturnCode Init(ClockPreset preset);

    static void SysTickHandler(void);
    static uint32_t GetSystemTime(void);
    static void Delay(uint32_t time_ms);

    static bool IsDefaultTickFailed();
    static bool IsHSIFailed();
    static bool IsHSEFailed();
    static bool IsPLLFailed();
    static bool IsSysTickFailed();

    static int GetSystemClockMHz();

private:
    static void EnablePowerClock_(void);
    static void SetPowerVoltageScale_(void);

    static hydrolib::ReturnCode EnableHSI_(void);
    static hydrolib::ReturnCode EnableHSE_(void);
    static void ConfigureSystemClock_(void);
    static hydrolib::ReturnCode ConfigurePLL_(uint32_t pllcfgr_value);

    static uint32_t GetSystickCounter_(void);
    static bool IsHSIReady_();
    static bool IsHSEReady_();
    static bool IsPLLReady_();

    static constexpr unsigned
    CalculateSystemClockMHz_(const ClockPreset &preset);
    static constexpr uint32_t CalculatePLLCFGRValue_(const ClockPreset &preset);

private:
    static constexpr uint32_t PWR_REGULATOR_VOLTAGE_SCALE1 = PWR_CR_VOS;
    static constexpr uint32_t PWR_REGULATOR_VOLTAGE_SCALE2 = 0;
    static constexpr unsigned FREQUENCY_HSI_MHZ = 16;
    static constexpr unsigned FREQUENCY_HSE_DEFAULT_MHZ = 8;
    static constexpr unsigned MhzToKhz_(unsigned freq);

    static inline unsigned systick_counter_ = 0;
    static inline bool default_tick_failed_ = false;
    static inline bool hsi_failed_ = false;
    static inline bool hse_failed_ = false;
    static inline bool pll_failed_ = false;
    static inline bool sys_tick_failed_ = false;
    static inline int system_clock_mhz_ = 0;
};

constexpr unsigned Clock::MhzToKhz_(unsigned freq) { return freq * 1000; }

inline hydrolib::ReturnCode Clock::Init(ClockPreset preset)
{
    const uint32_t pllcfgr_value = CalculatePLLCFGRValue_(preset);
    system_clock_mhz_ = CalculateSystemClockMHz_(preset);
    const uint32_t systick_reload_value = MhzToKhz_(system_clock_mhz_);

    default_tick_failed_ = SysTick_Config(MhzToKhz_(FREQUENCY_HSI_MHZ));
    if (default_tick_failed_)
    {
        return hydrolib::ReturnCode::ERROR;
    }

    EnablePowerClock_();
    SetPowerVoltageScale_();

    if (preset.source == HSI)
    {
        hydrolib::ReturnCode hsi_rc = EnableHSI_();
        hsi_failed_ = hsi_rc != hydrolib::ReturnCode::OK;
        if (hsi_failed_)
        {
            return hydrolib::ReturnCode::ERROR;
        }
    }
    else
    {
        hydrolib::ReturnCode hse_rc = EnableHSE_();
        hse_failed_ = hse_rc != hydrolib::ReturnCode::OK;
        if (hse_failed_)
        {
            return hydrolib::ReturnCode::ERROR;
        }
    }

    hydrolib::ReturnCode pll_rc = ConfigurePLL_(pllcfgr_value);
    pll_failed_ = pll_rc != hydrolib::ReturnCode::OK;
    if (pll_failed_)
    {
        return hydrolib::ReturnCode::ERROR;
    }

    ConfigureSystemClock_();

    sys_tick_failed_ = SysTick_Config(systick_reload_value);
    if (sys_tick_failed_)
    {
        return hydrolib::ReturnCode::ERROR;
    }

    return hydrolib::ReturnCode::OK;
}

inline void Clock::SysTickHandler() { systick_counter_++; }

inline uint32_t Clock::GetSystemTime(void) { return systick_counter_; }

inline void Clock::Delay(uint32_t time_ms)
{
    uint32_t start_counter = GetSystickCounter_();
    volatile uint32_t current_counter = GetSystickCounter_();
    while (current_counter - start_counter < time_ms)
    {
        current_counter = GetSystickCounter_();
    }
}

inline bool Clock::IsDefaultTickFailed() { return default_tick_failed_; }

inline bool Clock::IsHSIFailed() { return hsi_failed_; }

inline bool Clock::IsHSEFailed() { return hse_failed_; }

inline bool Clock::IsPLLFailed() { return pll_failed_; }

inline bool Clock::IsSysTickFailed() { return sys_tick_failed_; }

inline int Clock::GetSystemClockMHz() { return system_clock_mhz_; }

inline void Clock::EnablePowerClock_(void)
{
    volatile uint32_t tmpreg = 0x00U;
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    (void)tmpreg;
}

inline void Clock::SetPowerVoltageScale_(void)
{
    volatile uint32_t tmpreg = 0x00U;
    MODIFY_REG(PWR->CR, PWR_CR_VOS, PWR_REGULATOR_VOLTAGE_SCALE1);
    tmpreg = READ_BIT(PWR->CR, PWR_CR_VOS);
    (void)tmpreg;
}

inline hydrolib::ReturnCode Clock::EnableHSI_(void)
{
    SET_BIT(RCC->CR, RCC_CR_HSION);

    uint32_t start = GetSystickCounter_();
    while (!IsHSIReady_())
    {
        if (GetSystickCounter_() - start > TIMEOUT_MS)
        {
            return hydrolib::ReturnCode::FAIL;
        }
    }
    return hydrolib::ReturnCode::OK;
}

inline hydrolib::ReturnCode Clock::EnableHSE_(void)
{
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    uint32_t start = GetSystickCounter_();
    while (!IsHSEReady_())
    {
        if (GetSystickCounter_() - start > TIMEOUT_MS)
        {
            return hydrolib::ReturnCode::FAIL;
        }
    }
    return hydrolib::ReturnCode::OK;
}

inline void Clock::ConfigureSystemClock_(void)
{
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_5WS);

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1,
               RCC_CFGR_PPRE1_DIV16); // TODO: Precalculate CFGR
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV16);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);

    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV4);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV2);
}

inline hydrolib::ReturnCode Clock::ConfigurePLL_(uint32_t pllcfgr_value)
{
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
    RCC->PLLCFGR = pllcfgr_value;
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    uint32_t start = GetSystickCounter_();
    while (!IsPLLReady_())
    {
        if (GetSystickCounter_() - start > TIMEOUT_MS)
        {
            return hydrolib::ReturnCode::FAIL;
        }
    }

    return hydrolib::ReturnCode::OK;
}

inline uint32_t Clock::GetSystickCounter_(void)
{
    return Clock::systick_counter_;
}

inline bool Clock::IsHSIReady_() { return READ_BIT(RCC->CR, RCC_CR_HSIRDY); }

inline bool Clock::IsHSEReady_() { return READ_BIT(RCC->CR, RCC_CR_HSERDY); }

inline bool Clock::IsPLLReady_() { return READ_BIT(RCC->CR, RCC_CR_PLLRDY); }

constexpr unsigned Clock::CalculateSystemClockMHz_(const ClockPreset &preset)
{
    unsigned input_mhz = 0;
    switch (preset.source)
    {
    case HSE:
        input_mhz = preset.frequency_hse_mhz;
        break;
    case HSI:
        input_mhz = FREQUENCY_HSI_MHZ;
        break;
    }
    return input_mhz * preset.N / preset.P / preset.M;
}

constexpr uint32_t Clock::CalculatePLLCFGRValue_(const ClockPreset &preset)
{
    return preset.source | (preset.M << RCC_PLLCFGR_PLLM_Pos) |
           (preset.N << RCC_PLLCFGR_PLLN_Pos) |
           (((preset.P >> 1) - 1) << RCC_PLLCFGR_PLLP_Pos);
}

} // namespace hydrv::clock