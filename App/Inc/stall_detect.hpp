// App/Inc/stall_detect.hpp
#pragma once
#include <cstdint>

namespace zlens {

class StallDetect {
public:
    enum class Direction { FORWARD, REVERSE };
    enum class LimitType { LIMIT_MIN, LIMIT_MAX };

    static constexpr uint16_t STALL_THRESHOLD = 200;
    static constexpr uint16_t OVERCURRENT_THRESHOLD = 350;
    static constexpr uint16_t BLANKING_TICKS = 200;
    static constexpr uint16_t STALL_CONFIRM_COUNT = 1000;
    static constexpr uint16_t OVERCURRENT_CONFIRM = 10;
    static constexpr uint16_t ENCODER_STALL_TICKS = 500;

    void init();
    void reset();
    void start_motor();
    void update(uint16_t adc_current, int32_t encoder_pos, uint32_t tick);

    bool is_stalled() const { return m_bStallConfirmed; }
    bool is_overcurrent() const { return m_bOvercurrent; }
    bool encoder_stalled() const { return m_bEncoderStall; }

    void set_direction(Direction d) { m_eDirection = d; }
    LimitType get_stall_limit_type() const;

private:
    Direction m_eDirection = Direction::FORWARD;
    uint16_t m_iBlankingCount = 0;
    uint16_t m_iStallCount = 0;
    uint16_t m_iOvercurrentCount = 0;
    bool m_bStallConfirmed = false;
    bool m_bOvercurrent = false;
    bool m_bMotorRunning = false;

    // Encoder stall detection
    int32_t m_iLastEncoderPos = 0;
    uint16_t m_iEncoderNoChangeCount = 0;
    bool m_bEncoderStall = false;
};

} // namespace zlens
