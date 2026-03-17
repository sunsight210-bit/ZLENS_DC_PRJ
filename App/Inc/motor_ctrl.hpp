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

enum class MOTOR_STATE_E { IDLE, ACCELERATING, CONSTANT, DECELERATING, BRAKING, STALLED };
enum class DIRECTION_E { FORWARD, REVERSE };

class MotorCtrl {
public:
    static constexpr uint16_t PWM_ARR = 4266;
    static constexpr uint16_t MAX_SPEED = 3600;    // ~75% duty
    static constexpr uint16_t MIN_SPEED = 480;     // ~10% duty
    static constexpr uint16_t ACCEL_STEP = 10;     // PWM increment per ms
    static constexpr int32_t  DEADZONE = 50;       // +/-50 counts
    static constexpr int32_t  DECEL_DISTANCE = 5000;

    void init(TIM_HandleTypeDef* htim, DAC_HandleTypeDef* hdac, Encoder* encoder);
    void move_to(int32_t target);
    void stop();
    void emergency_stop();
    void update();

    void set_current_limit(uint16_t milliamps);
    MOTOR_STATE_E get_state() const { return m_eState; }
    DIRECTION_E get_direction() const { return m_eDirection; }
    uint16_t get_current_speed() const { return m_iCurrentSpeed; }
    int32_t get_target() const { return m_iTarget; }

    void set_max_speed(uint16_t speed) { m_iMaxSpeed = speed; }
    void set_min_speed(uint16_t speed) { m_iMinSpeed = speed; }

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

    void set_pwm(DIRECTION_E dir, uint16_t speed);
    void brake();
    void coast();
};

} // namespace zlens
