// App/Src/self_test.cpp
#include "self_test.hpp"
#include "swo_debug.hpp"
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
    m_pMotor->emergency_stop();
    m_pStall->reset();
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
            m_ePhase = SELF_TEST_PHASE_E::HOMING_START;
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
            // Reverse moved, encoder decreased → direction correct, at positive limit
            set_item_pass(SELF_TEST_ITEM_E::ENCODER_DIR, true);
        } else if (iDelta > ENCODER_DIR_THRESHOLD) {
            // Reverse moved but encoder increased → polarity reversed
            m_bEncoderReversed = true;
            TIM8->CCER ^= TIM_CCER_CC1P;
            set_item_pass(SELF_TEST_ITEM_E::ENCODER_DIR, true);
            m_stResult.bEncoderCompensated = true;
        } else {
            // Neither direction produced movement → hardware fault
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
        stParams.homing_done = 1;
        stParams.position_valid = 0xFF;
        stParams.encoder_compensated = m_bEncoderReversed ? 1 : 0;
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
    print_report();
}

void SelfTest::print_report() {
    // --- Helper lambdas for ADC conversions ---
    // ADC→mA: R_sense=0.2Ω, Vref=3.3V, 12-bit ADC
    // V_sense = ADC * 3300mV / 4095
    // I = V_sense / R_sense = V_sense / 0.2Ω = V_sense * 5  (mV→mA when R in Ω)
    // I_mA = ADC * 3300 * 5 / 4095 = ADC * 16500 / 4095
    auto adc_to_current_ma = [](uint16_t adc) -> uint32_t {
        return static_cast<uint32_t>(adc) * 16500 / 4095;
    };

    // Read live ADC values
    uint16_t iVRaw = m_pAdcVoltage ? *m_pAdcVoltage : 0;
    uint16_t iCRaw = m_pAdcCurrent ? *m_pAdcCurrent : 0;
    uint32_t iVmv = PowerMonitor::adc_to_voltage_mv(iVRaw);
    uint32_t iCma = adc_to_current_ma(iCRaw);
    uint32_t iThreshVmv = PowerMonitor::adc_to_voltage_mv(
        PowerMonitor::POWER_DOWN_THRESHOLD);

    // Constants from task_config.hpp rsp:: namespace (avoid include due to FreeRTOS deps)
    constexpr uint16_t FW_VERSION = 0x1000;  // rsp::FW_VERSION
    constexpr uint16_t LENS_TYPE  = 0x0004;  // rsp::LENS_TYPE

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
    {
        uint32_t iBaseMa = adc_to_current_ma(m_stResult.iMeasuredBaseline);
        uint32_t iMaxMa = adc_to_current_ma(BASELINE_MAX);
        swo_printf(" BASELINE ..... %s  %lumA / ADC=%u (max %lumA/%u)\n",
                   m_stResult.aPass[1] ? "PASS" : "FAIL",
                   iBaseMa, m_stResult.iMeasuredBaseline,
                   iMaxMa, BASELINE_MAX);
    }
    // FRAM_RW
    swo_printf(" FRAM_RW ...... %s  addr=0x%04X pattern=0x%02X\n",
               m_stResult.aPass[2] ? "PASS" : "FAIL",
               FRAM_TEST_ADDR, FRAM_TEST_BYTE);
    // ENCODER_DIR
    swo_printf(" ENCODER_DIR .. %s  compensated=%s\n",
               m_stResult.aPass[3] ? "PASS" : "FAIL",
               m_stResult.bEncoderCompensated ? "YES" : "NO");
    // HOMING
    swo_printf(" HOMING ....... %s  range=%ld counts\n",
               m_stResult.aPass[4] ? "PASS" : "FAIL",
               static_cast<long>(m_stResult.iTotalRange));
    // RANGE_CHECK
    swo_printf(" RANGE_CHECK .. %s  %ld > %ld\n",
               m_stResult.aPass[5] ? "PASS" : "FAIL",
               static_cast<long>(m_stResult.iTotalRange),
               static_cast<long>(MIN_VALID_RANGE));
    // LIMITS_CHECK
    {
        // SOFT_LIMIT_OFFSET=800 from MotorTask
        constexpr int32_t SOFT_LIMIT_OFFSET = 800;
        int32_t iSoftMin = SOFT_LIMIT_OFFSET;
        int32_t iSoftMax = m_stResult.iTotalRange - SOFT_LIMIT_OFFSET;
        swo_printf(" LIMITS_CHECK . %s  soft=[%ld, %ld]\n",
                   m_stResult.aPass[6] ? "PASS" : "FAIL",
                   static_cast<long>(iSoftMin), static_cast<long>(iSoftMax));
    }
    // FRAM_SAVE
    swo_printf(" FRAM_SAVE .... %s  CRC verified\n",
               m_stResult.aPass[7] ? "PASS" : "FAIL");

    // --- Lens Zoom ---
    {
        uint16_t iMinZ = m_pZoom->get_min_zoom();
        uint16_t iMaxZ = m_pZoom->get_max_zoom();
        uint8_t iCount = m_pZoom->get_entry_count();
        swo_printf("\n[Lens Zoom]\n");
        swo_printf(" Optical range   %u.%ux ~ %u.%ux\n",
                   iMinZ / 10, iMinZ % 10, iMaxZ / 10, iMaxZ % 10);
        swo_printf(" Zoom presets    %u (", iCount);
        // Print preset list from zoom table min to max
        bool bFirst = true;
        uint16_t iZoom = iMinZ;
        for (uint8_t i = 0; i < iCount; ++i) {
            if (!bFirst) swo_printf(",");
            swo_printf("%u", iZoom);
            bFirst = false;
            iZoom = m_pZoom->get_next_zoom(iZoom, 1);
            if (iZoom == 0) break;
        }
        swo_printf(")\n");
    }

    // --- Mechanical ---
    if (m_stResult.iTotalRange > 0) {
        constexpr int32_t SOFT_LIMIT_OFFSET = 800;  // from MotorTask
        constexpr int32_t HOMING_RETRACT = 8000;     // from MotorTask
        int32_t iRange = m_stResult.iTotalRange;
        swo_printf("\n[Mechanical]\n");
        swo_printf(" Total range     %ld counts\n", static_cast<long>(iRange));
        swo_printf(" Hard limits     [0, %ld]\n", static_cast<long>(iRange));
        swo_printf(" Soft limits     [%ld, %ld]  offset=%ld\n",
                   static_cast<long>(SOFT_LIMIT_OFFSET),
                   static_cast<long>(iRange - SOFT_LIMIT_OFFSET),
                   static_cast<long>(SOFT_LIMIT_OFFSET));
        swo_printf(" Homing retract  %ld counts\n", static_cast<long>(HOMING_RETRACT));
        swo_printf(" Encoder         512PPR x4 = 2048/rev  ratio=104.77:1\n");
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
        uint32_t iStallMa = adc_to_current_ma(StallDetect::STALL_THRESHOLD);
        uint32_t iOcMa = adc_to_current_ma(StallDetect::OVERCURRENT_THRESHOLD);
        swo_printf("\n[Protection Thresholds]\n");
        swo_printf(" Stall           ADC>=%u (~%lumA)  confirm=%u ticks\n",
                   StallDetect::STALL_THRESHOLD, iStallMa,
                   StallDetect::STALL_CONFIRM_COUNT);
        swo_printf(" Overcurrent     ADC>=%u (~%lumA) confirm=%u ticks\n",
                   StallDetect::OVERCURRENT_THRESHOLD, iOcMa,
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
        const char* pState = (iStateIdx < 6) ? STATE_NAMES[iStateIdx] : "?";
        swo_printf("\n[Live Readings]\n");
        swo_printf(" Voltage         %lu.%luV / ADC=%u\n",
                   iVmv / 1000, (iVmv % 1000) / 100, iVRaw);
        swo_printf(" Current         %lumA / ADC=%u\n",
                   iCma, iCRaw);
        swo_printf(" Encoder pos     %ld\n",
                   static_cast<long>(m_pEncoder->get_position()));
        swo_printf(" Motor state     %s\n", pState);
    }
    swo_printf("===================================================\n");
}

} // namespace zlens
