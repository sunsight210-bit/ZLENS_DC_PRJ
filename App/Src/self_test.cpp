// App/Src/self_test.cpp
#include "self_test.hpp"
#include "swo_debug.hpp"
#include "task_config.hpp"
#include "zoom_table.hpp"
#include <cstring>

namespace zlens {

void SelfTest::init(PowerMonitor* pPm, FramStorage* pFram, Encoder* pEncoder,
                    MotorCtrl* pMotor, StallDetect* pStall,
                    uint16_t* pAdcVoltage, uint16_t* pAdcCurrent) {
    m_pPm = pPm;
    m_pFram = pFram;
    m_pEncoder = pEncoder;
    m_pMotor = pMotor;
    m_pStall = pStall;
    m_pAdcVoltage = pAdcVoltage;
    m_pAdcCurrent = pAdcCurrent;
    m_ePhase = SELF_TEST_PHASE_E::IDLE;
    std::memset(&m_stResult, 0, sizeof(m_stResult));
}

void SelfTest::start() {
    m_pMotor->emergency_stop();
    m_pStall->reset();
    std::memset(&m_stResult, 0, sizeof(m_stResult));
    m_bEncoderReversed = false;
    m_ePhase = SELF_TEST_PHASE_E::VOLTAGE;
}

void SelfTest::set_item_pass(SELF_TEST_ITEM_E eItem, bool bPass) {
    m_stResult.aPass[static_cast<uint8_t>(eItem)] = bPass;
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
        m_iEncoderDirStartTick = iTick;
        m_pStall->set_direction(StallDetect::Direction::FORWARD);
        m_pStall->start_motor();
        m_pMotor->move_to(m_iEncoderStartPos + ENCODER_DIR_MOVE);
        m_ePhase = SELF_TEST_PHASE_E::ENCODER_DIR_WAIT;
        break;
    }
    case SELF_TEST_PHASE_E::ENCODER_DIR_WAIT: {
        m_pMotor->update();
        if (m_pMotor->get_state() != MOTOR_STATE_E::IDLE) {
            if (iTick - m_iEncoderDirStartTick > ENCODER_DIR_TIMEOUT_MS) {
                m_pMotor->emergency_stop();
            }
            break;
        }
        int32_t iDelta = m_pEncoder->get_position() - m_iEncoderStartPos;
        if (iDelta > ENCODER_DIR_THRESHOLD) {
            // Direction correct, not at positive limit
            set_item_pass(SELF_TEST_ITEM_E::ENCODER_DIR, true);
            m_pStall->reset();
            m_ePhase = SELF_TEST_PHASE_E::DONE;
            finalize();
            return true;
        } else {
            // Forward motion insufficient — try reverse to distinguish
            // positive-limit vs encoder-reversed vs hardware-fail
            m_ePhase = SELF_TEST_PHASE_E::ENCODER_DIR_REVERSE;
        }
        break;
    }
    case SELF_TEST_PHASE_E::ENCODER_DIR_REVERSE: {

        m_iEncoderStartPos = m_pEncoder->get_position();
        m_iEncoderDirStartTick = iTick;
        m_pStall->set_direction(StallDetect::Direction::REVERSE);
        m_pStall->start_motor();
        m_pMotor->move_to(m_iEncoderStartPos - ENCODER_DIR_MOVE);
        m_ePhase = SELF_TEST_PHASE_E::ENCODER_DIR_REVERSE_WAIT;
        break;
    }
    case SELF_TEST_PHASE_E::ENCODER_DIR_REVERSE_WAIT: {
        m_pMotor->update();
        if (m_pMotor->get_state() != MOTOR_STATE_E::IDLE) {
            if (iTick - m_iEncoderDirStartTick > ENCODER_DIR_TIMEOUT_MS) {
                m_pMotor->emergency_stop();
            }
            break;
        }
        int32_t iDelta = m_pEncoder->get_position() - m_iEncoderStartPos;
        if (iDelta < -ENCODER_DIR_THRESHOLD) {
            // Reverse moved, encoder decreased -> direction correct, at positive limit
            set_item_pass(SELF_TEST_ITEM_E::ENCODER_DIR, true);
        } else if (iDelta > ENCODER_DIR_THRESHOLD) {
            // Reverse moved but encoder increased -> polarity reversed
            m_bEncoderReversed = true;
            TIM8->CCER ^= TIM_CCER_CC1P;
            set_item_pass(SELF_TEST_ITEM_E::ENCODER_DIR, true);
            m_stResult.bEncoderCompensated = true;
        } else {
            // Neither direction produced movement -> hardware fault
            set_item_pass(SELF_TEST_ITEM_E::ENCODER_DIR, false);
            m_pStall->reset();
            m_ePhase = SELF_TEST_PHASE_E::DONE;
            finalize();
            return true;
        }
        m_pStall->reset();
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
    print_report();
}

void SelfTest::print_report() {
    // Read live ADC values
    uint16_t iVRaw = m_pAdcVoltage ? *m_pAdcVoltage : 0;
    uint16_t iCRaw = m_pAdcCurrent ? *m_pAdcCurrent : 0;
    uint32_t iVmv = PowerMonitor::adc_to_voltage_mv(iVRaw);
    uint32_t iThreshVmv = PowerMonitor::adc_to_voltage_mv(
        PowerMonitor::POWER_DOWN_THRESHOLD);

    constexpr uint16_t FW_VERSION = rsp::FW_VERSION;
    constexpr uint16_t LENS_TYPE  = rsp::LENS_TYPE;

    swo_printf("\n============ ZLENS_DC SELF-TEST REPORT ============\n");
    swo_printf("RESULT: %s         FW: v%u.%u.%u  Lens: 0x%02X\n",
               m_stResult.bAllPassed ? "ALL PASSED" : "SOME FAILED",
               (FW_VERSION >> 12) & 0xF, (FW_VERSION >> 8) & 0xF,
               (FW_VERSION >> 4) & 0xF, LENS_TYPE);

    // --- Test Items ---
    swo_printf("\n[Test Items]\n");

    // VOLTAGE
    {
        uint32_t iMeasVmv = PowerMonitor::adc_to_voltage_mv(m_stResult.iMeasuredVoltage);
        swo_printf(" VOLTAGE ...... %s  %lu.%luV / ADC=%u (min %lu.%luV/%u)\n",
                   m_stResult.aPass[0] ? "PASS" : "FAIL",
                   iMeasVmv / 1000, (iMeasVmv % 1000) / 100,
                   m_stResult.iMeasuredVoltage,
                   iThreshVmv / 1000, (iThreshVmv % 1000) / 100,
                   PowerMonitor::POWER_DOWN_THRESHOLD);
    }
    // BASELINE
    swo_printf(" BASELINE ..... %s  ADC=%u (max %u)\n",
               m_stResult.aPass[1] ? "PASS" : "FAIL",
               m_stResult.iMeasuredBaseline, BASELINE_MAX);
    // FRAM_RW
    swo_printf(" FRAM_RW ...... %s  addr=0x%04X pattern=0x%02X\n",
               m_stResult.aPass[2] ? "PASS" : "FAIL",
               FRAM_TEST_ADDR, FRAM_TEST_BYTE);
    // ENCODER_DIR
    swo_printf(" ENCODER_DIR .. %s  compensated=%s\n",
               m_stResult.aPass[3] ? "PASS" : "FAIL",
               m_stResult.bEncoderCompensated ? "YES" : "NO");

    // --- Mechanical ---
    {
        constexpr int32_t SOFT_LIMIT_OFFSET = ZoomTable::HOME_OFFSET;
        constexpr int32_t HOMING_RETRACT = homing::RETRACT_DISTANCE;
        int32_t iRange = ZoomTable::TOTAL_RANGE;
        swo_printf("\n[Mechanical]\n");
        swo_printf(" Total range     %ld counts\n", static_cast<long>(iRange));
        swo_printf(" Hard limits     [0, %ld]\n", static_cast<long>(iRange));
        swo_printf(" Soft limits     [%ld, %ld]  offset=%ld\n",
                   static_cast<long>(SOFT_LIMIT_OFFSET),
                   static_cast<long>(iRange - SOFT_LIMIT_OFFSET),
                   static_cast<long>(SOFT_LIMIT_OFFSET));
        swo_printf(" Homing retract  %ld counts\n", static_cast<long>(HOMING_RETRACT));
        swo_printf(" Encoder         512PPR x4 = 2048/rev  ratio=104.77:1 x gear 4:1 = 419.08:1\n");
    }

    // --- Motor ---
    {
        uint32_t iMaxPct = static_cast<uint32_t>(MotorCtrl::MAX_SPEED) * 100
                           / MotorCtrl::PWM_ARR;
        uint32_t iMinPct = static_cast<uint32_t>(MotorCtrl::MIN_SPEED) * 100
                           / MotorCtrl::PWM_ARR;
        swo_printf("\n[Motor]\n");
        swo_printf(" PWM             max=%u/%u(%lu%%) min=%u/%u(%lu%%)\n",
                   MotorCtrl::MAX_SPEED, MotorCtrl::PWM_ARR, iMaxPct,
                   MotorCtrl::MIN_SPEED, MotorCtrl::PWM_ARR, iMinPct);
        swo_printf(" Accel           %u/ms  decel=%ld counts\n",
                   MotorCtrl::ACCEL_STEP,
                   static_cast<long>(MotorCtrl::DECEL_DISTANCE));
        swo_printf(" Deadzone        +/-%ld counts\n",
                   static_cast<long>(MotorCtrl::DEADZONE));
    }

    // --- Protection Thresholds ---
    {
        swo_printf("\n[Protection Thresholds]\n");
        swo_printf(" Stall           ADC>=%u  confirm=%u ticks\n",
                   StallDetect::STALL_THRESHOLD,
                   StallDetect::STALL_CONFIRM_COUNT);
        swo_printf(" Overcurrent     ADC>=%u  confirm=%u ticks\n",
                   StallDetect::OVERCURRENT_THRESHOLD,
                   StallDetect::OVERCURRENT_CONFIRM);
        swo_printf(" Blanking        %u ticks\n", StallDetect::BLANKING_TICKS);
        swo_printf(" Encoder stall   %u ticks no movement\n",
                   StallDetect::ENCODER_STALL_TICKS);
        swo_printf(" Power-down      <%lu.%luV (ADC<%u)\n",
                   iThreshVmv / 1000, (iThreshVmv % 1000) / 100,
                   PowerMonitor::POWER_DOWN_THRESHOLD);
    }

    // --- Live Readings ---
    {
        static const char* const STATE_NAMES[] = {
            "IDLE", "ACCEL", "CONST", "DECEL", "BRAKE", "STALL"
        };
        uint8_t iStateIdx = static_cast<uint8_t>(m_pMotor->get_state());
        const char* pState = (iStateIdx < sizeof(STATE_NAMES) / sizeof(STATE_NAMES[0]))
                             ? STATE_NAMES[iStateIdx] : "?";
        swo_printf("\n[Live Readings]\n");
        swo_printf(" Voltage         %lu.%luV / ADC=%u\n",
                   iVmv / 1000, (iVmv % 1000) / 100, iVRaw);
        swo_printf(" Current ADC     %u\n", iCRaw);
        swo_printf(" Encoder pos     %ld\n",
                   static_cast<long>(m_pEncoder->get_position()));
        swo_printf(" Motor state     %s\n", pState);
    }
    swo_printf("===================================================\n");
}

} // namespace zlens
