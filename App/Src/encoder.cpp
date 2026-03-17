// App/Src/encoder.cpp
#include "encoder.hpp"

namespace zlens {

void Encoder::init() {
    m_iOverflow = 0;
    m_iZPosition = 0;
    TIM8->CNT = 0;
}

int32_t Encoder::get_position() const {
    int32_t ovf = m_iOverflow;
    uint16_t cnt = static_cast<uint16_t>(TIM8->CNT);
    return ovf * 65536 + cnt;
}

void Encoder::set_position(int32_t pos) {
    if (pos >= 0) {
        m_iOverflow = pos / 65536;
        TIM8->CNT = pos % 65536;
    } else {
        // Handle negative: e.g. -100 => overflow=-1, CNT=65436
        m_iOverflow = (pos - 65535) / 65536;
        TIM8->CNT = static_cast<uint16_t>(pos - m_iOverflow * 65536);
    }
}

void Encoder::reset() {
    m_iOverflow = 0;
    m_iZPosition = 0;
    TIM8->CNT = 0;
}

void Encoder::handle_overflow(bool up_direction) {
    if (up_direction) {
        m_iOverflow++;
    } else {
        m_iOverflow--;
    }
}

void Encoder::handle_z_pulse() {
    m_iZPosition = get_position();
}

} // namespace zlens
