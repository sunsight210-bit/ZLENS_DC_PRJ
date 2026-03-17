// App/Inc/power_monitor.hpp
#pragma once
#include <cstdint>

namespace zlens {

class PowerMonitor {
public:
    static constexpr uint16_t POWER_DOWN_THRESHOLD = 3103; // ADC value for 10V

    void init();
    bool is_power_down(uint16_t adc_value) const;
    static uint32_t adc_to_voltage_mv(uint16_t adc_value);
};

} // namespace zlens
