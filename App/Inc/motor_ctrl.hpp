// App/Inc/motor_ctrl.hpp
#pragma once
#include <cstdint>
#include "encoder.hpp"
#include "pid_ctrl.hpp"

#ifdef BUILD_TESTING
#include "mock_hal.hpp"
#else
#include "stm32f1xx_hal.h"
#endif

namespace zlens {

enum class MOTOR_STATE_E { IDLE, RUNNING, STALLED };
enum class DIRECTION_E { FORWARD, REVERSE };

class MotorCtrl {
public:
    static constexpr uint16_t PWM_ARR = 4266;
    static constexpr uint16_t MAX_SPEED = 1200;
    static constexpr uint16_t MIN_SPEED = 200;
    static constexpr int32_t  DEADZONE = 3;            // ~6um
    static constexpr int32_t  SAFE_LIMIT_MIN = 64;     // = HOME_OFFSET / 2
    static constexpr uint16_t ENCODER_TIMEOUT_TICKS = 500;  // 500ms
    static constexpr uint16_t SETTLE_COUNT = 100;      // 100ms brake settle before IDLE

    void init(TIM_HandleTypeDef* htim, DAC_HandleTypeDef* hdac, Encoder* encoder);
    void move_to(int32_t target);
    void stop();
    void emergency_stop();
    void update();

    void set_vref_mv(uint16_t mv);
    MOTOR_STATE_E get_state() const { return m_eState; }
    DIRECTION_E get_direction() const { return m_eDirection; }
    int32_t get_target() const { return m_iTarget; }

    static constexpr uint16_t duty_to_pwm(uint16_t iDuty_x10) {
        return static_cast<uint16_t>(static_cast<uint32_t>(iDuty_x10) * PWM_ARR / 1000);
    }
    static constexpr uint16_t pwm_to_duty(uint16_t iPwm) {
        return static_cast<uint16_t>(static_cast<uint32_t>(iPwm) * 1000 / PWM_ARR);
    }

    void set_soft_limit_min(int32_t iMin) { m_iSoftLimitMin = iMin; }
    void set_safe_limit_max(int32_t iMax) { m_iSafeLimitMax = iMax; }

    void set_pwm_test(DIRECTION_E dir, uint16_t speed) { set_pwm(dir, speed); }
    void brake_test() { brake(); }

private:
    TIM_HandleTypeDef* m_pHtim = nullptr;
    DAC_HandleTypeDef* m_pHdac = nullptr;
    Encoder* m_pEncoder = nullptr;

    MOTOR_STATE_E m_eState = MOTOR_STATE_E::IDLE;
    DIRECTION_E m_eDirection = DIRECTION_E::FORWARD;
    int32_t m_iTarget = 0;
    int32_t m_iSoftLimitMin = 0;
    int32_t m_iSafeLimitMax = 65536 - SAFE_LIMIT_MIN;

    PidCtrl m_Pid;
    int32_t m_iLastPos = 0;
    int32_t m_iLastPosBeforeUpdate = 0;
    uint16_t m_iNoMoveCount = 0;
    uint16_t m_iSettleCount = 0;

    void set_pwm(DIRECTION_E dir, uint16_t speed);
    void brake();
    void coast();
};

} // namespace zlens
