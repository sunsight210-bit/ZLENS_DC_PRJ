// App/Src/adc_filter.cpp
#include "adc_filter.hpp"

namespace zlens {

void AdcFilter::reset(uint16_t iInitial) {
    m_iAccumulator = static_cast<uint32_t>(iInitial) << ALPHA_SHIFT;
    m_iFiltered = iInitial;
}

uint16_t AdcFilter::update(uint16_t iRawValue) {
    m_iAccumulator += iRawValue - (m_iAccumulator >> ALPHA_SHIFT);
    m_iFiltered = static_cast<uint16_t>(m_iAccumulator >> ALPHA_SHIFT);
    return m_iFiltered;
}

} // namespace zlens
