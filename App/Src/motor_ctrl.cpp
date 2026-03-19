// App/Src/motor_ctrl.cpp
#include "motor_ctrl.hpp"
#include <cstdlib>

namespace zlens {

void MotorCtrl::init(TIM_HandleTypeDef* htim, DAC_HandleTypeDef* hdac, Encoder* encoder) {
    m_pHtim = htim;
    m_pHdac = hdac;
    m_pEncoder = encoder;
    m_eState = MOTOR_STATE_E::IDLE;
    m_iCurrentSpeed = 0;
    coast();
}

void MotorCtrl::set_pwm(DIRECTION_E dir, uint16_t speed) {
    if (dir == DIRECTION_E::FORWARD) {
        m_pHtim->Instance->CCR1 = speed;
        m_pHtim->Instance->CCR2 = 0;
    } else {
        m_pHtim->Instance->CCR1 = 0;
        m_pHtim->Instance->CCR2 = speed;
    }
}

void MotorCtrl::brake() {
    m_pHtim->Instance->CCR1 = PWM_ARR;
    m_pHtim->Instance->CCR2 = PWM_ARR;
}

void MotorCtrl::coast() {
    m_pHtim->Instance->CCR1 = 0;
    m_pHtim->Instance->CCR2 = 0;
}

void MotorCtrl::move_to(int32_t target) {
    m_iTarget = target;
    int32_t pos = m_pEncoder->get_position();
    int32_t diff = target - pos;

    if (std::abs(diff) <= DEADZONE) {
        m_eState = MOTOR_STATE_E::IDLE;
        return;
    }

    m_eDirection = (diff > 0) ? DIRECTION_E::FORWARD : DIRECTION_E::REVERSE;
    m_iCurrentSpeed = m_iMinSpeed;
    m_eState = MOTOR_STATE_E::ACCELERATING;
    set_pwm(m_eDirection, m_iCurrentSpeed);
}

void MotorCtrl::stop() {
    if (m_eState == MOTOR_STATE_E::IDLE) return;
    m_eState = MOTOR_STATE_E::DECELERATING;
}

void MotorCtrl::emergency_stop() {
    brake();
    m_iCurrentSpeed = 0;
    m_eState = MOTOR_STATE_E::IDLE;
}

void MotorCtrl::set_current_limit(uint16_t milliamps) {
    // I_max = VREF / 2 => VREF = I_max * 2
    // DAC 12-bit: 0-4095 maps to 0-3.3V
    uint32_t vref_mv = milliamps * 2; // mV
    uint32_t dac_val = vref_mv * 4095 / 3300;
    if (dac_val > 4095) dac_val = 4095;
    HAL_DAC_SetValue(m_pHdac, 2, 0, dac_val);
}

void MotorCtrl::update() {
    if (m_eState == MOTOR_STATE_E::IDLE || m_eState == MOTOR_STATE_E::STALLED) return;

    int32_t pos = m_pEncoder->get_position();
    int32_t diff = m_iTarget - pos;
    int32_t remaining = std::abs(diff);

    // Check arrival or overshoot (passed target beyond DEADZONE)
    bool bOvershot = (m_eDirection == DIRECTION_E::FORWARD && diff < -DEADZONE) ||
                     (m_eDirection == DIRECTION_E::REVERSE && diff > DEADZONE);
    if (remaining <= DEADZONE || bOvershot) {
        brake();
        m_iCurrentSpeed = 0;
        m_eState = MOTOR_STATE_E::IDLE;
        return;
    }

    switch (m_eState) {
    case MOTOR_STATE_E::ACCELERATING:
        if (m_iCurrentSpeed < m_iMaxSpeed) {
            m_iCurrentSpeed += ACCEL_STEP;
            if (m_iCurrentSpeed > m_iMaxSpeed) m_iCurrentSpeed = m_iMaxSpeed;
        } else {
            m_eState = MOTOR_STATE_E::CONSTANT;
        }
        // Only decelerate if we've actually accelerated above min speed,
        // otherwise short moves (distance < DECEL_DISTANCE) get stuck at min speed
        if (remaining < DECEL_DISTANCE && m_iCurrentSpeed > m_iMinSpeed + ACCEL_STEP) {
            m_eState = MOTOR_STATE_E::DECELERATING;
        }
        break;

    case MOTOR_STATE_E::CONSTANT:
        if (remaining < DECEL_DISTANCE) {
            m_eState = MOTOR_STATE_E::DECELERATING;
        }
        break;

    case MOTOR_STATE_E::DECELERATING:
        if (m_iCurrentSpeed > m_iMinSpeed) {
            m_iCurrentSpeed -= ACCEL_STEP;
            if (m_iCurrentSpeed < m_iMinSpeed) m_iCurrentSpeed = m_iMinSpeed;
        }
        break;

    case MOTOR_STATE_E::BRAKING:
        brake();
        m_iCurrentSpeed = 0;
        m_eState = MOTOR_STATE_E::IDLE;
        return;

    default:
        break;
    }

    set_pwm(m_eDirection, m_iCurrentSpeed);
}

} // namespace zlens
