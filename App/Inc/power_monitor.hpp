// App/Inc/power_monitor.hpp
#pragma once
#include <cstdint>

namespace zlens {

class PowerMonitor {
public:
    // Voltage divider on PB0 (ADC1_CH8):
    // 12V supply → PB0 = 1.65V (measured by multimeter)
    // ADC reference = 3.3V, 12-bit (0~4095)
    // ADC = 1.65 / 3.3 × 4095 = 2048 → 2048 raw ≈ 12V
    // Power-down threshold: 9V → ADC = 9/12 × 2048 = 1536
    static constexpr uint16_t POWER_DOWN_THRESHOLD = 1536;

    void init();
    bool is_power_down(uint16_t adc_value) const;
    static uint32_t adc_to_voltage_mv(uint16_t adc_value);
};

} // namespace zlens
