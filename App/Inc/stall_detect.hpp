// App/Inc/stall_detect.hpp
#pragma once
#include <cstdint>

namespace zlens {

class StallDetect {
public:
    enum class Direction { FORWARD, REVERSE };
    enum class LimitType { LIMIT_MIN, LIMIT_MAX };

    // --- 电流检测 (使用 IIR 滤波后的 ADC 值, alpha=1/16) ---
    // 实测 (VREF=2.5V, 12V 供电, A4950 驱动):
    //   空载: filt≈36   正常运动: filt 平均239 峰值942
    //   堵转 (30% PWM, MAX_SPEED=1280): filt avg≈998, raw max=4089
    //   堵转 (50% PWM): filt avg≈1722   堵转 (80% PWM): filt avg≈2747
    //   filt=3000 对应外部仪器实测总电流约 800mA
    //
    // 过流保护: filt > 3000 连续 50ms → 紧急停止 (硬件异常/短路保护)
    // 电流堵转: filt > 1200 连续 1000ms → 堵转 (备份, 30% 堵转 avg=998 不触发)
    // 编码器堵转: 位置不变连续 500ms → 堵转 (主要生效的检测方式)
    // 消隐期: 启动后前 200ms 不检测, 避免启动浪涌误判
    static constexpr uint16_t STALL_THRESHOLD = 1200;
    static constexpr uint16_t OVERCURRENT_THRESHOLD = 3000;
    static constexpr uint16_t BLANKING_TICKS = 200;         // 启动消隐 200ms
    static constexpr uint16_t STALL_CONFIRM_COUNT = 1000;   // 电流堵转确认 1000ms
    static constexpr uint16_t OVERCURRENT_CONFIRM = 50;     // 过流确认 50ms
    static constexpr uint16_t ENCODER_STALL_TICKS = 500;    // 编码器堵转确认 500ms

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
