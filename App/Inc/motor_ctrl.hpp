// App/Inc/motor_ctrl.hpp
#pragma once
#include <cstdint>
#include "encoder.hpp"

#ifdef BUILD_TESTING
#include "mock_hal.hpp"
#else
#include "stm32f1xx_hal.h"
#endif

namespace zlens {

enum class MOTOR_STATE_E { IDLE, ACCELERATING, CONSTANT, DECELERATING, BRAKING, STALLED, APPROACHING, SETTLING };
enum class DIRECTION_E { FORWARD, REVERSE };

class MotorCtrl {
public:
    static constexpr uint16_t PWM_ARR = 4266;
    static constexpr uint16_t MAX_SPEED = 2400;    // ~56% duty — avoid overcurrent
    static constexpr uint16_t MIN_SPEED = 480;     // ~10% duty
    static constexpr uint16_t ACCEL_STEP = 10;     // PWM increment per ms
    static constexpr int32_t  DEADZONE = 1000;      // +/-1000 counts (matches gearbox play)
    static constexpr int32_t  FINE_DEADZONE = 100;  // Phase2 fine positioning deadzone
    static constexpr int32_t  DECEL_DISTANCE = 15000;
    static constexpr uint16_t SETTLE_TICKS = 100;   // 100ms settle after brake
    static constexpr uint8_t  MAX_CORRECTIONS = 1;
    static constexpr uint16_t MIN_CORRECTION_SPEED = 480;   // ~11% duty, VREF=2V
    static constexpr int32_t  BACKLASH_MARGIN = 200;        // overshoot margin for backlash compensation

    void init(TIM_HandleTypeDef* htim, DAC_HandleTypeDef* hdac, Encoder* encoder);
    void move_to(int32_t target);
    void stop();
    void emergency_stop();
    void update();

    void set_vref_mv(uint16_t mv);
    MOTOR_STATE_E get_state() const { return m_eState; }
    DIRECTION_E get_direction() const { return m_eDirection; }
    uint16_t get_current_speed() const { return m_iCurrentSpeed; }
    int32_t get_target() const { return m_iTarget; }

    void set_max_speed(uint16_t speed) { m_iMaxSpeed = speed; }
    void set_min_speed(uint16_t speed) { m_iMinSpeed = speed; }
    void set_speed_limit(uint16_t iLimit) { m_iMaxSpeed = iLimit; }
    void set_backlash(int16_t iCounts) { m_iBacklash = iCounts; }
    void set_backlash_enabled(bool bEnabled) { m_bBacklashEnabled = bEnabled; }
    int16_t get_backlash() const { return m_iBacklash; }
    bool is_backlash_enabled() const { return m_bBacklashEnabled; }

private:
    TIM_HandleTypeDef* m_pHtim = nullptr;
    DAC_HandleTypeDef* m_pHdac = nullptr;
    Encoder* m_pEncoder = nullptr;

    MOTOR_STATE_E m_eState = MOTOR_STATE_E::IDLE;
    DIRECTION_E m_eDirection = DIRECTION_E::FORWARD;
    int32_t m_iTarget = 0;
    uint16_t m_iCurrentSpeed = 0;
    uint16_t m_iMaxSpeed = MAX_SPEED;
    uint16_t m_iMinSpeed = MIN_SPEED;

    int16_t m_iBacklash = 200;
    bool m_bBacklashEnabled = false;
    int32_t m_iFinalTarget = 0;
    uint16_t m_iSettleCount = 0;
    uint8_t m_iCorrectionCount = 0;
    bool m_bPhase2Active = false;

    void set_pwm(DIRECTION_E dir, uint16_t speed);
    void brake();
    void coast();
    void start_correction();
};

} // namespace zlens
