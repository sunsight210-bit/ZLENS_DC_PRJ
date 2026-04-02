// App/Src/encoder.cpp
#include "encoder.hpp"
#include "hw_constants.hpp"
#include <cstdlib>

namespace zlens {

void Encoder::init() {
    m_iOverflow = 0;
    m_iZPosition = 0;
    TIM4->CNT = 0;
}

int32_t Encoder::get_position() const {
    // Retry loop to avoid race between reading overflow and CNT
    int32_t ovf;
    uint16_t cnt;
    do {
        ovf = m_iOverflow;
        cnt = static_cast<uint16_t>(TIM4->CNT);
    } while (ovf != m_iOverflow);
    return ovf * static_cast<int32_t>(hw::TIMER_PERIOD) + cnt;
}

void Encoder::set_position(int32_t pos) {
    // Disable TIM4 update interrupt to prevent race during reset
    TIM4->DIER &= ~TIM_DIER_UIE;

    if (pos >= 0) {
        m_iOverflow = pos / static_cast<int32_t>(hw::TIMER_PERIOD);
        TIM4->CNT = pos % static_cast<int32_t>(hw::TIMER_PERIOD);
    } else {
        // Handle negative: e.g. -100 => overflow=-1, CNT=65436
        const int32_t iPeriod = static_cast<int32_t>(hw::TIMER_PERIOD);
        m_iOverflow = (pos - (iPeriod - 1)) / iPeriod;
        TIM4->CNT = static_cast<uint16_t>(pos - m_iOverflow * iPeriod);
    }

    // Clear any pending update flag, then re-enable
    TIM4->SR = ~TIM_SR_UIF;
    TIM4->DIER |= TIM_DIER_UIE;
}

void Encoder::reset() {
    TIM4->DIER &= ~TIM_DIER_UIE;
    m_iOverflow = 0;
    m_iZPosition = 0;
    TIM4->CNT = 0;
    TIM4->SR = ~TIM_SR_UIF;
    TIM4->DIER |= TIM_DIER_UIE;
    reset_index_tracking();
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

    if (!m_bFirstIndexSet) {
        m_iFirstIndexPos = m_iZPosition;
        m_bFirstIndexSet = true;
        m_iIndexCount = 0;
        return;
    }

    m_iIndexCount++;
    int32_t iExpected = m_iFirstIndexPos +
        static_cast<int32_t>(m_iIndexCount) * 1024;
    int16_t iDrift = static_cast<int16_t>(m_iZPosition - iExpected);

    if (std::abs(iDrift) > DRIFT_THRESHOLD) {
        m_bDriftDetected = true;
        m_iDriftError = iDrift;
    }
}

void Encoder::reset_index_tracking() {
    m_bFirstIndexSet = false;
    m_iIndexCount    = 0;
    m_bDriftDetected = false;
    m_iDriftError    = 0;
}

} // namespace zlens

#ifndef BUILD_TESTING
// C-linkage wrapper called from HAL_TIM_PeriodElapsedCallback (TIM4 overflow ISR)
namespace zlens { extern Encoder g_Encoder; }

extern "C" void encoder_overflow_handler(int up) {
    zlens::g_Encoder.handle_overflow(up != 0);
}

extern "C" void encoder_z_pulse_handler(void) {
    zlens::g_Encoder.handle_z_pulse();
}
#endif
