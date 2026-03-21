// App/Src/motor_ctrl.cpp
#include "motor_ctrl.hpp"
#include "hw_constants.hpp"
#include <cstdlib>

namespace zlens {

void MotorCtrl::init(TIM_HandleTypeDef* htim, DAC_HandleTypeDef* hdac, Encoder* encoder) {
    m_pHtim = htim;
    m_pHdac = hdac;
    m_pEncoder = encoder;
    m_eState = MOTOR_STATE_E::IDLE;
    m_iCurrentSpeed = 0;
    m_iCorrectionCount = 0;
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
    m_iCorrectionCount = 0;  // new move resets corrections
    int32_t pos = m_pEncoder->get_position();
    int32_t diff = target - pos;

    if (std::abs(diff) <= DEADZONE) {
        m_bPhase2Active = false;
        m_eState = MOTOR_STATE_E::IDLE;
        return;
    }

    if (m_bBacklashEnabled && m_iBacklash > 0 && diff < 0) {
        // Reverse move: two-phase approach
        m_iFinalTarget = target;
        int32_t iOvershoot = target - m_iBacklash - BACKLASH_MARGIN;
        if (iOvershoot < FINE_DEADZONE) {
            iOvershoot = FINE_DEADZONE;  // clamp to safe distance from zero
        }
        m_iTarget = iOvershoot;
    } else {
        m_iTarget = target;
        m_iFinalTarget = target;
    }

    m_eDirection = (m_iTarget - pos > 0) ? DIRECTION_E::FORWARD : DIRECTION_E::REVERSE;
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
    m_iCorrectionCount = 0;
    m_bPhase2Active = false;
    m_eState = MOTOR_STATE_E::IDLE;
}

void MotorCtrl::set_vref_mv(uint16_t mv) {
    uint32_t dac_val = static_cast<uint32_t>(mv) * hw::DAC_RESOLUTION / hw::VREF_MV;
    if (dac_val > hw::DAC_RESOLUTION) dac_val = hw::DAC_RESOLUTION;
    HAL_DAC_SetValue(m_pHdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_val);
}

// Start a slow correction move directly to final target (no backlash compensation)
void MotorCtrl::start_correction() {
    int32_t pos = m_pEncoder->get_position();
    int32_t diff = m_iFinalTarget - pos;
    int32_t iCorrDeadzone = m_bPhase2Active ? FINE_DEADZONE : DEADZONE;
    if (std::abs(diff) <= iCorrDeadzone) {
        m_bPhase2Active = false;
        m_eState = MOTOR_STATE_E::IDLE;
        return;
    }
    m_iTarget = m_iFinalTarget;
    m_eDirection = (diff > 0) ? DIRECTION_E::FORWARD : DIRECTION_E::REVERSE;
    // Correction runs at MIN_CORRECTION_SPEED to ensure gearbox can move
    m_iCurrentSpeed = MIN_CORRECTION_SPEED;
    m_eState = MOTOR_STATE_E::CONSTANT;  // skip accel/decel — short distance, low speed
    set_pwm(m_eDirection, m_iCurrentSpeed);
}

void MotorCtrl::update() {
    if (m_eState == MOTOR_STATE_E::IDLE || m_eState == MOTOR_STATE_E::STALLED) return;

    // Handle APPROACHING: start phase 2 (forward to final target)
    if (m_eState == MOTOR_STATE_E::APPROACHING) {
        m_bPhase2Active = true;
        m_iTarget = m_iFinalTarget;
        m_eDirection = DIRECTION_E::FORWARD;
        m_iCurrentSpeed = MIN_CORRECTION_SPEED;
        m_eState = MOTOR_STATE_E::CONSTANT;
        m_iCorrectionCount = 1;  // prevent immediate decel in CONSTANT branch
        set_pwm(m_eDirection, m_iCurrentSpeed);
        return;
    }

    // Handle SETTLING: wait for motor to physically stop, then check position
    if (m_eState == MOTOR_STATE_E::SETTLING) {
        if (++m_iSettleCount < SETTLE_TICKS) return;
        // Settled — check position vs final target
        int32_t pos = m_pEncoder->get_position();
        int32_t iFinalDiff = std::abs(pos - m_iFinalTarget);
        int32_t iSettleDeadzone = m_bPhase2Active ? FINE_DEADZONE : DEADZONE;
        if (iFinalDiff > iSettleDeadzone && m_iCorrectionCount < MAX_CORRECTIONS) {
            m_iCorrectionCount++;
            start_correction();
        } else {
            m_bPhase2Active = false;
            m_eState = MOTOR_STATE_E::IDLE;
        }
        return;
    }

    int32_t pos = m_pEncoder->get_position();
    int32_t diff = m_iTarget - pos;
    int32_t remaining = std::abs(diff);

    // Check arrival or overshoot
    int32_t iActiveDeadzone = m_bPhase2Active ? FINE_DEADZONE : DEADZONE;
    bool bOvershot = (m_eDirection == DIRECTION_E::FORWARD && diff < -iActiveDeadzone) ||
                     (m_eDirection == DIRECTION_E::REVERSE && diff > iActiveDeadzone);
    if (remaining <= iActiveDeadzone || bOvershot) {
        brake();
        m_iCurrentSpeed = 0;
        // Check if this was phase 1 of backlash compensation
        if (m_bBacklashEnabled && m_iTarget != m_iFinalTarget) {
            m_eState = MOTOR_STATE_E::APPROACHING;
            return;
        }
        // Enter settling to wait for motor to physically stop
        m_iSettleCount = 0;
        m_eState = MOTOR_STATE_E::SETTLING;
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
        if (remaining < DECEL_DISTANCE && m_iCurrentSpeed > m_iMinSpeed + ACCEL_STEP) {
            m_eState = MOTOR_STATE_E::DECELERATING;
        }
        break;

    case MOTOR_STATE_E::CONSTANT:
        if (remaining < DECEL_DISTANCE && m_iCorrectionCount == 0) {
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
