// App/Src/power_monitor.cpp
#include "power_monitor.hpp"
#include "hw_constants.hpp"

namespace zlens {

void PowerMonitor::init() {}

bool PowerMonitor::is_power_down(uint16_t adc_value) const {
    return adc_value < POWER_DOWN_THRESHOLD;
}

uint32_t PowerMonitor::adc_to_voltage_mv(uint16_t adc_value) {
    // R_top=30kΩ, R_bottom=4.7kΩ → V_in = V_adc × 34.7/4.7
    // Verified: 12V → PB0=1.65V (multimeter) → ADC≈2048
    // Formula: adc × 24363 / 4095
    // Max: 4095 × 24363 = 99,766,485 < UINT32_MAX ✓
    return static_cast<uint32_t>(adc_value) * hw::VOLTAGE_SCALE_NUM / hw::ADC_RESOLUTION;
}

} // namespace zlens
