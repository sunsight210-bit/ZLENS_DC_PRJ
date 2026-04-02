// App/Inc/adc_filter.hpp
#pragma once
#include <cstdint>

namespace zlens {

class AdcFilter {
public:
    static constexpr uint8_t ALPHA_SHIFT = 4; // alpha = 1/16

    void reset(uint16_t iInitial = 0);
    uint16_t update(uint16_t iRawValue);
    uint16_t get_filtered() const { return m_iFiltered; }

private:
    uint32_t m_iAccumulator = 0;
    uint16_t m_iFiltered = 0;
};

} // namespace zlens
