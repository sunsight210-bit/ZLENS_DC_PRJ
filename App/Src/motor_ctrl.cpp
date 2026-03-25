// App/Src/motor_ctrl.cpp
#include "motor_ctrl.hpp"
#include "zoom_table.hpp"
#include <cstdlib>

namespace zlens {

static_assert(MotorCtrl::SAFE_LIMIT_MIN == ZoomTable::HOME_OFFSET / 2,
              "SAFE_LIMIT_MIN must be HOME_OFFSET / 2");

void MotorCtrl::init(TIM_HandleTypeDef* htim, DAC_HandleTypeDef* hdac, Encoder* encoder) {
    m_pHtim = htim;
    m_pHdac = hdac;
    m_pEncoder = encoder;
    m_eState = MOTOR_STATE_E::IDLE;
    m_iTarget = 0;
    m_iNoMoveCount = 0;
    m_iCoastCount = 0;
    m_Pid.reset(0);
    coast();
}

void MotorCtrl::move_to(int32_t target) {
    if (target < SAFE_LIMIT_MIN) target = SAFE_LIMIT_MIN;
    if (target > m_iSafeLimitMax) target = m_iSafeLimitMax;

    m_iNoMoveCount = 0;
    m_iCoastCount = 0;

    int32_t pos = m_pEncoder->get_position();
    m_Pid.reset(pos);

    if (std::abs(target - pos) <= DEADZONE) {
        m_eState = MOTOR_STATE_E::IDLE;
        return;
    }

    m_iTarget = target;
    m_eState = MOTOR_STATE_E::RUNNING;
}

void MotorCtrl::stop() {
    brake();
    m_Pid.reset(m_pEncoder ? m_pEncoder->get_position() : 0);
    m_eState = MOTOR_STATE_E::IDLE;
}

void MotorCtrl::emergency_stop() {
    brake();
    m_Pid.reset(0);
    m_iNoMoveCount = 0;
    m_iCoastCount = 0;
    m_eState = MOTOR_STATE_E::IDLE;
}

void MotorCtrl::update() {
    if (m_eState != MOTOR_STATE_E::RUNNING) return;

    int32_t pos = m_pEncoder->get_position();
    int32_t iError = m_iTarget - pos;

    // Safety limits
    if (pos <= SAFE_LIMIT_MIN && iError < 0) {
        brake(); m_Pid.reset(pos); m_eState = MOTOR_STATE_E::IDLE; return;
    }
    if (pos >= m_iSafeLimitMax && iError > 0) {
        brake(); m_Pid.reset(pos); m_eState = MOTOR_STATE_E::IDLE; return;
    }

    // Encoder timeout (stall detection)
    if (pos == m_iLastPos) {
        if (++m_iNoMoveCount >= ENCODER_TIMEOUT_TICKS) {
            brake(); m_Pid.reset(pos); m_iNoMoveCount = 0;
            m_eState = MOTOR_STATE_E::STALLED; return;
        }
    } else {
        m_iNoMoveCount = 0;
    }
    m_iLastPos = pos;

    // Deadzone check
    if (std::abs(iError) <= DEADZONE) {
        brake(); m_Pid.reset(pos); m_eState = MOTOR_STATE_E::IDLE; return;
    }

    // PID compute
    int16_t iOutput = m_Pid.compute(iError, pos);
    uint16_t iSpeed = static_cast<uint16_t>(std::abs(iOutput));

    // Coast zone with timeout nudge
    if (iSpeed < MIN_SPEED) {
        if (std::abs(iError) <= COAST_ZONE) {
            if (++m_iCoastCount < COAST_TIMEOUT) {
                coast();
                m_iNoMoveCount = 0;
                return;
            }
            m_iCoastCount = 0;
        }
        iSpeed = MIN_SPEED;
    } else {
        m_iCoastCount = 0;
    }

    m_eDirection = (iOutput >= 0) ? DIRECTION_E::FORWARD : DIRECTION_E::REVERSE;
    set_pwm(m_eDirection, iSpeed);

    // Drift detection (no interruption, log only)
    if (m_pEncoder->is_drift_detected()) {
#ifdef PID_TUNE_LOG
        extern void swo_printf(const char* fmt, ...);
        swo_printf("[DRIFT]%ld\n", static_cast<int32_t>(m_pEncoder->get_drift_error()));
#endif
    }

#ifdef PID_TUNE_LOG
    extern void swo_printf(const char* fmt, ...);
    swo_printf("[PID]%lu,%ld,%ld,%ld,%d\n",
               HAL_GetTick(), m_iTarget, pos, iError, iOutput);
#endif
}

void MotorCtrl::set_vref_mv(uint16_t mv) {
    if (!m_pHdac) return;
    uint32_t dac_val = static_cast<uint32_t>(mv) * 4095 / 3300;
    HAL_DAC_SetValue(m_pHdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_val);
}

void MotorCtrl::set_pwm(DIRECTION_E dir, uint16_t speed) {
    if (!m_pHtim) return;
    if (dir == DIRECTION_E::FORWARD) {
        m_pHtim->Instance->CCR1 = speed;
        m_pHtim->Instance->CCR2 = 0;
    } else {
        m_pHtim->Instance->CCR1 = 0;
        m_pHtim->Instance->CCR2 = speed;
    }
}

void MotorCtrl::brake() {
    if (!m_pHtim) return;
    m_pHtim->Instance->CCR1 = PWM_ARR;
    m_pHtim->Instance->CCR2 = PWM_ARR;
}

void MotorCtrl::coast() {
    if (!m_pHtim) return;
    m_pHtim->Instance->CCR1 = 0;
    m_pHtim->Instance->CCR2 = 0;
}

} // namespace zlens
