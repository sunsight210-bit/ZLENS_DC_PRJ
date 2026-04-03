// App/Inc/hw_constants.hpp
#pragma once
#include <cstdint>

namespace zlens {
namespace hw {

// --- ADC / DAC ---
static constexpr uint16_t ADC_RESOLUTION      = 4095;   // 12-bit max
static constexpr uint16_t DAC_RESOLUTION      = 4095;   // 12-bit max
static constexpr uint16_t VREF_MV             = 3300;   // MCU VDDA 参考电压 3.3V (非 A4950 VREF)

// --- Encoder (TIM4, 16-bit, AS5311 on PB6/PB7) ---
static constexpr uint32_t TIMER_PERIOD        = 65536;  // 2^16

// --- FRAM SPI Commands ---
static constexpr uint8_t  FRAM_CMD_WREN       = 0x06;
static constexpr uint8_t  FRAM_CMD_WRITE      = 0x02;
static constexpr uint8_t  FRAM_CMD_READ       = 0x03;

// --- FRAM GPIO ---
static constexpr uint16_t FRAM_CS_PIN         = 1 << 12;  // PB12
static constexpr uint16_t FRAM_WP_PIN         = 1 << 11;  // PB11

// --- Flash Programming ---
static constexpr uint8_t  FLASH_HALFWORD_SIZE = 2;    // STM32 Flash 半字编程粒度 (bytes)

// --- Voltage Divider (PB0, ADC1_CH8) ---
// R_top=30kΩ, R_bottom=4.7kΩ
// V_adc = V_in × 4.7/(30+4.7) = V_in × 0.1354
// Verified: 12V supply → PB0 = 1.65V (multimeter) → ADC ≈ 2048
// V_mV = ADC × VREF_MV × (R_top + R_bot) / R_bot / ADC_RESOLUTION
//      = ADC × 3300 × 34700 / 4700 / 4095 = ADC × 24363 / 4095
static constexpr uint16_t VDIV_R_TOP          = 30000;  // 上分压电阻 30kΩ
static constexpr uint16_t VDIV_R_BOTTOM       = 4700;   // 下分压电阻 4.7kΩ
static constexpr uint16_t VOLTAGE_SCALE_NUM   = VREF_MV * (VDIV_R_TOP + VDIV_R_BOTTOM) / VDIV_R_BOTTOM;

} // namespace hw
} // namespace zlens
