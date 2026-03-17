// App/Src/stall_detect.cpp
#include "stall_detect.hpp"

namespace zlens {

void StallDetect::init() { reset(); }

void StallDetect::reset() {
    m_iBlankingCount = 0;
    m_iStallCount = 0;
    m_iOvercurrentCount = 0;
    m_bStallConfirmed = false;
    m_bOvercurrent = false;
    m_bMotorRunning = false;
    m_iEncoderNoChangeCount = 0;
    m_bEncoderStall = false;
    m_iLastEncoderPos = 0;
}

void StallDetect::start_motor() {
    reset();
    m_bMotorRunning = true;
}

void StallDetect::update(uint16_t adc_current, int32_t encoder_pos, uint32_t) {
    if (!m_bMotorRunning || m_bStallConfirmed || m_bOvercurrent) return;

    // Blanking window
    if (m_iBlankingCount < BLANKING_TICKS) {
        m_iBlankingCount++;
        m_iLastEncoderPos = encoder_pos;
        return;
    }

    // Overcurrent check (fast response)
    if (adc_current > OVERCURRENT_THRESHOLD) {
        m_iOvercurrentCount++;
        if (m_iOvercurrentCount >= OVERCURRENT_CONFIRM) {
            m_bOvercurrent = true;
            return;
        }
    } else {
        m_iOvercurrentCount = 0;
    }

    // Stall current check
    if (adc_current > STALL_THRESHOLD) {
        m_iStallCount++;
        if (m_iStallCount >= STALL_CONFIRM_COUNT) {
            m_bStallConfirmed = true;
        }
    } else {
        m_iStallCount = 0;
    }

    // Encoder stall check
    if (encoder_pos == m_iLastEncoderPos) {
        m_iEncoderNoChangeCount++;
        if (m_iEncoderNoChangeCount >= ENCODER_STALL_TICKS) {
            m_bEncoderStall = true;
        }
    } else {
        m_iEncoderNoChangeCount = 0;
        m_iLastEncoderPos = encoder_pos;
    }
}

StallDetect::LimitType StallDetect::get_stall_limit_type() const {
    return (m_eDirection == Direction::FORWARD) ? LimitType::LIMIT_MAX : LimitType::LIMIT_MIN;
}

} // namespace zlens
