// App/Src/power_monitor.cpp
#include "power_monitor.hpp"

namespace zlens {

void PowerMonitor::init() {}

bool PowerMonitor::is_power_down(uint16_t adc_value) const {
    return adc_value < POWER_DOWN_THRESHOLD;
}

uint32_t PowerMonitor::adc_to_voltage_mv(uint16_t adc_value) {
    // Divider ratio 1:4, so V_in = ADC * 3300 / 4095 * 4
    return static_cast<uint32_t>(adc_value) * 3300 * 4 / 4095;
}

} // namespace zlens
