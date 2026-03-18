// App/Inc/power_monitor.hpp
#pragma once
#include <cstdint>

namespace zlens {

class PowerMonitor {
public:
    // R_top=30kΩ, R_bottom=4.7kΩ, ratio=347/47
    // ADC: 2017 raw ≈ 12V, 1394 raw ≈ 8.3V
    static constexpr uint16_t POWER_DOWN_THRESHOLD = 1394;

    void init();
    bool is_power_down(uint16_t adc_value) const;
    static uint32_t adc_to_voltage_mv(uint16_t adc_value);
};

} // namespace zlens
