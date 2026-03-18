// App/Src/self_test.cpp
#include "self_test.hpp"
#include <cstring>

namespace zlens {

void SelfTest::init(PowerMonitor* pPm, FramStorage* pFram, Encoder* pEncoder,
                    MotorCtrl* pMotor, StallDetect* pStall, ZoomTable* pZoom,
                    uint16_t* pAdcVoltage, uint16_t* pAdcCurrent) {
    m_pPm = pPm;
    m_pFram = pFram;
    m_pEncoder = pEncoder;
    m_pMotor = pMotor;
    m_pStall = pStall;
    m_pZoom = pZoom;
    m_pAdcVoltage = pAdcVoltage;
    m_pAdcCurrent = pAdcCurrent;
    m_ePhase = SELF_TEST_PHASE_E::IDLE;
    std::memset(&m_stResult, 0, sizeof(m_stResult));
}

void SelfTest::start() {
    std::memset(&m_stResult, 0, sizeof(m_stResult));
    m_bHomingNotified = false;
    m_bHomingSuccess = false;
    m_iHomingTotalRange = 0;
    m_bEncoderReversed = false;
    m_ePhase = SELF_TEST_PHASE_E::VOLTAGE;
}

void SelfTest::set_item_pass(SELF_TEST_ITEM_E eItem, bool bPass) {
    m_stResult.aPass[static_cast<uint8_t>(eItem)] = bPass;
}

void SelfTest::notify_homing_done(bool bSuccess, int32_t iTotalRange) {
    m_bHomingNotified = true;
    m_bHomingSuccess = bSuccess;
    m_iHomingTotalRange = iTotalRange;
}

bool SelfTest::step(uint32_t iTick) {
    switch (m_ePhase) {
    case SELF_TEST_PHASE_E::VOLTAGE: {
        uint16_t iAdcVal = m_pAdcVoltage ? *m_pAdcVoltage : 0;
        m_stResult.iMeasuredVoltage = iAdcVal;
        bool bPass = !m_pPm->is_power_down(iAdcVal);
        set_item_pass(SELF_TEST_ITEM_E::VOLTAGE, bPass);
        if (!bPass) { m_ePhase = SELF_TEST_PHASE_E::DONE; finalize(); return true; }
        m_ePhase = SELF_TEST_PHASE_E::BASELINE;
        break;
    }
    case SELF_TEST_PHASE_E::BASELINE: {
        uint16_t iAdcVal = m_pAdcCurrent ? *m_pAdcCurrent : 0;
        m_stResult.iMeasuredBaseline = iAdcVal;
        bool bPass = iAdcVal < BASELINE_MAX;
        set_item_pass(SELF_TEST_ITEM_E::BASELINE, bPass);
        if (!bPass) { m_ePhase = SELF_TEST_PHASE_E::DONE; finalize(); return true; }
        m_ePhase = SELF_TEST_PHASE_E::FRAM_RW;
        break;
    }
    case SELF_TEST_PHASE_E::FRAM_RW: {
        bool bPass = m_pFram->test_rw(FRAM_TEST_ADDR, FRAM_TEST_BYTE);
        set_item_pass(SELF_TEST_ITEM_E::FRAM_RW, bPass);
        if (!bPass) { m_ePhase = SELF_TEST_PHASE_E::DONE; finalize(); return true; }
        m_ePhase = SELF_TEST_PHASE_E::ENCODER_DIR_START;
        break;
    }
    case SELF_TEST_PHASE_E::ENCODER_DIR_START: {
        m_iEncoderStartPos = m_pEncoder->get_position();
        m_pStall->set_direction(StallDetect::Direction::FORWARD);
        m_pStall->start_motor();
        m_pMotor->move_to(m_iEncoderStartPos + ENCODER_DIR_MOVE);
        m_ePhase = SELF_TEST_PHASE_E::ENCODER_DIR_WAIT;
        break;
    }
    case SELF_TEST_PHASE_E::ENCODER_DIR_WAIT: {
        m_pMotor->update();
        if (m_pMotor->get_state() != MOTOR_STATE_E::IDLE) break;
        int32_t iDelta = m_pEncoder->get_position() - m_iEncoderStartPos;
        if (iDelta > ENCODER_DIR_THRESHOLD) {
            // Direction correct
            set_item_pass(SELF_TEST_ITEM_E::ENCODER_DIR, true);
            m_pStall->reset();
            m_ePhase = SELF_TEST_PHASE_E::HOMING_START;
        } else if (!m_bEncoderReversed) {
            // Try reversing encoder polarity
            m_bEncoderReversed = true;
            TIM8->CCER ^= TIM_CCER_CC1P;
            m_pEncoder->set_position(0);
            m_ePhase = SELF_TEST_PHASE_E::ENCODER_DIR_REVERSE;
        } else {
            // Already tried reversing, fail
            set_item_pass(SELF_TEST_ITEM_E::ENCODER_DIR, false);
            m_pStall->reset();
            m_ePhase = SELF_TEST_PHASE_E::DONE;
            finalize();
            return true;
        }
        break;
    }
    case SELF_TEST_PHASE_E::ENCODER_DIR_REVERSE: {
        m_iEncoderStartPos = m_pEncoder->get_position();
        m_pStall->start_motor();
        m_pMotor->move_to(m_iEncoderStartPos + ENCODER_DIR_MOVE);
        m_ePhase = SELF_TEST_PHASE_E::ENCODER_DIR_REVERSE_WAIT;
        break;
    }
    case SELF_TEST_PHASE_E::ENCODER_DIR_REVERSE_WAIT: {
        m_pMotor->update();
        if (m_pMotor->get_state() != MOTOR_STATE_E::IDLE) break;
        int32_t iDelta = m_pEncoder->get_position() - m_iEncoderStartPos;
        if (iDelta > ENCODER_DIR_THRESHOLD) {
            set_item_pass(SELF_TEST_ITEM_E::ENCODER_DIR, true);
            m_stResult.bEncoderCompensated = true;
        } else {
            set_item_pass(SELF_TEST_ITEM_E::ENCODER_DIR, false);
            m_pStall->reset();
            m_ePhase = SELF_TEST_PHASE_E::DONE;
            finalize();
            return true;
        }
        m_pStall->reset();
        m_ePhase = SELF_TEST_PHASE_E::HOMING_START;
        break;
    }
    case SELF_TEST_PHASE_E::HOMING_START: {
        // Caller (MonitorTask) will send the HOMING command
        m_iHomingStartTick = iTick;
        m_bHomingNotified = false;
        m_ePhase = SELF_TEST_PHASE_E::HOMING_WAIT;
        break;
    }
    case SELF_TEST_PHASE_E::HOMING_WAIT: {
        if (m_bHomingNotified) {
            set_item_pass(SELF_TEST_ITEM_E::HOMING, m_bHomingSuccess);
            if (!m_bHomingSuccess) {
                m_ePhase = SELF_TEST_PHASE_E::DONE;
                finalize();
                return true;
            }
            m_stResult.iTotalRange = m_iHomingTotalRange;
            m_ePhase = SELF_TEST_PHASE_E::RANGE_CHECK;
        } else if (iTick - m_iHomingStartTick > HOMING_TIMEOUT_MS) {
            set_item_pass(SELF_TEST_ITEM_E::HOMING, false);
            m_ePhase = SELF_TEST_PHASE_E::DONE;
            finalize();
            return true;
        }
        break;
    }
    case SELF_TEST_PHASE_E::RANGE_CHECK: {
        bool bPass = m_iHomingTotalRange > MIN_VALID_RANGE;
        set_item_pass(SELF_TEST_ITEM_E::RANGE_CHECK, bPass);
        if (!bPass) { m_ePhase = SELF_TEST_PHASE_E::DONE; finalize(); return true; }
        m_ePhase = SELF_TEST_PHASE_E::LIMITS_CHECK;
        break;
    }
    case SELF_TEST_PHASE_E::LIMITS_CHECK: {
        int32_t iSoftMin = MotorCtrl::DEADZONE;
        int32_t iSoftMax = m_iHomingTotalRange - MotorCtrl::DEADZONE;
        bool bPass = iSoftMin < iSoftMax;
        set_item_pass(SELF_TEST_ITEM_E::LIMITS_CHECK, bPass);
        if (!bPass) { m_ePhase = SELF_TEST_PHASE_E::DONE; finalize(); return true; }
        m_ePhase = SELF_TEST_PHASE_E::FRAM_SAVE;
        break;
    }
    case SELF_TEST_PHASE_E::FRAM_SAVE: {
        FRAM_PARAMS_S stParams{};
        stParams.magic_number = FramStorage::MAGIC;
        stParams.version = 1;
        stParams.current_position = m_pEncoder->get_position();
        stParams.total_range = m_iHomingTotalRange;
        stParams.homing_done = 1;
        stParams.position_valid = 0xFF;
        stParams.encoder_compensated = m_bEncoderReversed ? 1 : 0;
        stParams.soft_limit_offset = 200; // matches MotorTask::SOFT_LIMIT_OFFSET
        bool bSaved = m_pFram->save_params(stParams);
        // Verify by reading back
        FRAM_PARAMS_S stVerify{};
        bool bLoaded = m_pFram->load_params(stVerify);
        bool bPass = bSaved && bLoaded && FramStorage::verify_crc(stVerify);
        set_item_pass(SELF_TEST_ITEM_E::FRAM_SAVE, bPass);
        m_ePhase = SELF_TEST_PHASE_E::DONE;
        finalize();
        return true;
    }
    case SELF_TEST_PHASE_E::IDLE:
    case SELF_TEST_PHASE_E::DONE:
        return true;
    }
    return false;
}

void SelfTest::finalize() {
    m_stResult.bAllPassed = true;
    for (uint8_t i = 0; i < static_cast<uint8_t>(SELF_TEST_ITEM_E::COUNT); ++i) {
        if (!m_stResult.aPass[i]) {
            m_stResult.bAllPassed = false;
            break;
        }
    }
}

} // namespace zlens
