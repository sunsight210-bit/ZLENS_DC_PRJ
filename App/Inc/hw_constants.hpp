// App/Inc/hw_constants.hpp
#pragma once
#include <cstdint>

namespace zlens {
namespace hw {

// --- ADC / DAC ---
static constexpr uint16_t ADC_RESOLUTION      = 4095;   // 12-bit max
static constexpr uint16_t DAC_RESOLUTION      = 4095;   // 12-bit max
static constexpr uint16_t VREF_MV             = 3300;   // 参考电压 3.3V

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

// --- Voltage Divider ---
// 推导：V_in = V_adc × (R_top + R_bot) / R_bot
//       V_mV = ADC × VREF_MV / ADC_RESOLUTION × (30000 + 4700) / 4700
//            = ADC × 24363 / ADC_RESOLUTION
static constexpr uint16_t VDIV_R_TOP          = 30000;  // 上分压电阻 30kΩ
static constexpr uint16_t VDIV_R_BOTTOM       = 4700;   // 下分压电阻 4.7kΩ
static constexpr uint16_t VOLTAGE_SCALE_NUM   = VREF_MV * (VDIV_R_TOP + VDIV_R_BOTTOM) / VDIV_R_BOTTOM;

} // namespace hw
} // namespace zlens
