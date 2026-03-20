// Tasks/Src/motor_task.cpp
#include "motor_task.hpp"
#ifndef BUILD_TESTING
#include "app_instances.hpp"
#include "swo_debug.hpp"
#endif

namespace zlens {

void MotorTask::init(MotorCtrl* pMotor, Encoder* pEncoder, StallDetect* pStall,
                     ZoomTable* pZoom, FramStorage* pFram, SystemManager* pSm,
                     QueueHandle_t cmdQ, QueueHandle_t rspQ, QueueHandle_t saveQ,
                     uint16_t* pAdcCurrent) {
    m_pMotor = pMotor;
    m_pEncoder = pEncoder;
    m_pStall = pStall;
    m_pZoom = pZoom;
    m_pFram = pFram;
    m_pSm = pSm;
    m_cmdQueue = cmdQ;
    m_rspQueue = rspQ;
    m_saveQueue = saveQ;
    m_pAdcCurrent = pAdcCurrent;
    m_eTaskState = TASK_STATE_E::IDLE;
    m_bHomingDone = false;
    m_bFullDiagnostics = false;
    m_AdcCurrentFilter.reset(0);
}

void MotorTask::run_once() {
    // Check for power-down notification (highest priority)
    uint32_t iNotifyVal = 0;
    if (xTaskNotifyWait(0, 0x02, &iNotifyVal, 0) == pdTRUE) {
        if (iNotifyVal & 0x02) {
            handle_power_down();
            return;
        }
    }

    int32_t iPos = m_pEncoder->get_position();
    uint16_t iAdcRaw = m_pAdcCurrent ? *m_pAdcCurrent : 0;
    uint16_t iAdcFiltered = m_AdcCurrentFilter.update(iAdcRaw);

    m_pStall->update(iAdcFiltered, iPos, HAL_GetTick());
    m_pMotor->update();

    // Overcurrent: highest priority (any state except IDLE)
    if (m_pStall->is_overcurrent() && m_eTaskState != TASK_STATE_E::IDLE) {
        handle_overcurrent();
        return;
    }

    // Stall detection
    if (m_pStall->is_stalled()) {
        handle_stall();
        return;
    }

    // Non-blocking command queue check
    CMD_MESSAGE_S stCmd;
    if (xQueueReceive(m_cmdQueue, &stCmd, 0) == pdTRUE) {
        dispatch_command(stCmd);
    }

    // State processing
    switch (m_eTaskState) {
    case TASK_STATE_E::MOVING:
        process_moving();
        break;
    case TASK_STATE_E::HOMING_FAST:
    case TASK_STATE_E::HOMING_RETRACT:
    case TASK_STATE_E::HOMING_SLOW:
    case TASK_STATE_E::HOMING_SETTLE:
        process_homing();
        break;
    case TASK_STATE_E::CYCLING:
        process_cycling();
        break;
    case TASK_STATE_E::BACKLASH_MEASURE:
        process_backlash_measure();
        break;
    case TASK_STATE_E::ACCURACY_TEST:
        process_accuracy_test();
        break;
    default:
        break;
    }
}

void MotorTask::dispatch_command(const CMD_MESSAGE_S& stCmd) {
#ifndef BUILD_TESTING
    swo_printf("[MOTOR] CMD 0x%02X param=0x%04X state=%d\n",
               stCmd.cmd, stCmd.param, (int)m_eTaskState);
#endif
    switch (stCmd.cmd) {
    case cmd::SET_ZOOM: {
        if (m_eTaskState != TASK_STATE_E::IDLE) break;
        m_pSm->transition_to(SYSTEM_STATE_E::BUSY);
        int32_t iTarget = m_pZoom->get_position(stCmd.param);
        m_pStall->set_direction(iTarget > m_pEncoder->get_position()
            ? StallDetect::Direction::FORWARD : StallDetect::Direction::REVERSE);
        m_pStall->start_motor();
        m_pMotor->move_to(iTarget);
        m_eTaskState = TASK_STATE_E::MOVING;
        break;
    }
    case cmd::HOMING:
        if (m_eTaskState != TASK_STATE_E::IDLE) break;
        m_pSm->transition_to(SYSTEM_STATE_E::HOMING);
        start_homing(stCmd.param == 1);
        break;
    case cmd::FORCE_STOP:
        m_pMotor->emergency_stop();
        m_pStall->reset();
        if (m_eTaskState == TASK_STATE_E::CYCLING) {
            m_bCycleWaiting = false;
        }
        m_eTaskState = TASK_STATE_E::IDLE;
        m_pSm->transition_to(SYSTEM_STATE_E::READY);
        send_response(cmd::FORCE_STOP, rsp::OK);
        break;
    case cmd::ZOOM_INC: {
        if (m_eTaskState != TASK_STATE_E::IDLE) break;
        m_pSm->transition_to(SYSTEM_STATE_E::BUSY);
        int32_t iPos = m_pEncoder->get_position();
        uint16_t iCurZoom = m_pZoom->get_nearest_zoom(iPos);
        uint16_t iNewZoom = iCurZoom + stCmd.param;
        uint16_t iMaxZoom = m_pZoom->get_max_zoom();
        if (iNewZoom > iMaxZoom) iNewZoom = iMaxZoom;
        int32_t iTarget = m_pZoom->get_position(iNewZoom);
        m_pStall->set_direction(iTarget > iPos
            ? StallDetect::Direction::FORWARD : StallDetect::Direction::REVERSE);
        m_pStall->start_motor();
        m_pMotor->move_to(iTarget);
        m_eTaskState = TASK_STATE_E::MOVING;
        break;
    }
    case cmd::ZOOM_DEC: {
        if (m_eTaskState != TASK_STATE_E::IDLE) break;
        m_pSm->transition_to(SYSTEM_STATE_E::BUSY);
        int32_t iPos = m_pEncoder->get_position();
        uint16_t iCurZoom = m_pZoom->get_nearest_zoom(iPos);
        uint16_t iMinZoom = m_pZoom->get_min_zoom();
        uint16_t iNewZoom = (stCmd.param >= iCurZoom) ? iMinZoom
                            : (iCurZoom - stCmd.param < iMinZoom ? iMinZoom : iCurZoom - stCmd.param);
        int32_t iTarget = m_pZoom->get_position(iNewZoom);
        m_pStall->set_direction(iTarget > iPos
            ? StallDetect::Direction::FORWARD : StallDetect::Direction::REVERSE);
        m_pStall->start_motor();
        m_pMotor->move_to(iTarget);
        m_eTaskState = TASK_STATE_E::MOVING;
        break;
    }
    case cmd::CYCLE_START:
        if (m_eTaskState != TASK_STATE_E::IDLE) break;
        m_pSm->transition_to(SYSTEM_STATE_E::BUSY);
        start_cycle(static_cast<int8_t>(stCmd.param >> 8),
                    static_cast<uint8_t>(stCmd.param & 0xFF));
        break;
    case cmd::CYCLE_STOP:
        stop_cycle();
        break;
    default:
        break;
    }
}

void MotorTask::start_homing(bool bFullDiag) {
    m_bHomingDone = false;
    m_bFullDiagnostics = bFullDiag;
    m_pMotor->set_backlash_enabled(false);
    m_pMotor->set_speed_limit(HOMING_FAST_SPEED);
    m_eTaskState = TASK_STATE_E::HOMING_FAST;
    m_pStall->set_direction(StallDetect::Direction::REVERSE);
    m_pStall->start_motor();
    m_pMotor->move_to(-HOMING_FAR_DISTANCE);
#ifndef BUILD_TESTING
    swo_printf("[INFO] Homing started: fast reverse to min limit\n");
#endif
}

void MotorTask::process_moving() {
    if (m_pMotor->get_state() == MOTOR_STATE_E::IDLE) {
        int32_t iPos = m_pEncoder->get_position();
        uint16_t iZoom = m_pZoom->get_nearest_zoom(iPos);
        m_pStall->reset();
        send_response(rsp_cmd::ZOOM, iZoom);
        send_response(rsp_cmd::ARRIVED, rsp::ARRIVED_PARAM);
        send_save(save_reason::ARRIVED);
        m_eTaskState = TASK_STATE_E::IDLE;
        m_pSm->transition_to(SYSTEM_STATE_E::READY);
    }
}

void MotorTask::process_homing() {
    switch (m_eTaskState) {
    case TASK_STATE_E::HOMING_RETRACT:
        if (m_pMotor->get_state() == MOTOR_STATE_E::IDLE) {
            // Retract done → start slow reverse
            m_pMotor->set_speed_limit(HOMING_SLOW_SPEED);
            m_eTaskState = TASK_STATE_E::HOMING_SLOW;
            m_pStall->set_direction(StallDetect::Direction::REVERSE);
            m_pStall->start_motor();
            m_pMotor->move_to(-HOMING_FAR_DISTANCE);
#ifndef BUILD_TESTING
            swo_printf("[INFO] Homing: retract done, starting slow reverse\n");
#endif
        }
        break;
    case TASK_STATE_E::HOMING_SETTLE:
        if (m_pMotor->get_state() == MOTOR_STATE_E::IDLE) {
            // Settle done → homing complete
            m_pMotor->set_speed_limit(MotorCtrl::MAX_SPEED);
            m_pMotor->set_backlash_enabled(true);
            m_bHomingDone = true;
            m_pStall->reset();
            int32_t iPos = m_pEncoder->get_position();
            uint16_t iZoom = m_pZoom->get_nearest_zoom(iPos);
            send_response(rsp_cmd::ZOOM, iZoom);
            send_response(rsp_cmd::HOMING_DONE, rsp::HOMING_DONE_PARAM);
            send_save(save_reason::ARRIVED);

            if (m_bFullDiagnostics) {
                // Chain: homing → backlash measurement
                start_backlash_measure();
            } else {
                m_eTaskState = TASK_STATE_E::IDLE;
                m_pSm->transition_to(SYSTEM_STATE_E::READY);
            }
#ifndef BUILD_TESTING
            swo_printf("[PASS] Homing complete: pos=%ld fullDiag=%d\n",
                       static_cast<long>(m_pEncoder->get_position()),
                       m_bFullDiagnostics ? 1 : 0);
#endif
        }
        break;
    default:
        break;
    }
}

void MotorTask::handle_stall() {
    switch (m_eTaskState) {
    case TASK_STATE_E::HOMING_FAST:
        // Coarse negative limit found
        m_pMotor->emergency_stop();
        m_pEncoder->set_position(0);
        m_pStall->reset();
        m_pMotor->set_speed_limit(HOMING_SLOW_SPEED);
        m_eTaskState = TASK_STATE_E::HOMING_RETRACT;
        m_pStall->set_direction(StallDetect::Direction::FORWARD);
        m_pStall->start_motor();
        m_pMotor->move_to(HOMING_RETRACT_DISTANCE);
#ifndef BUILD_TESTING
        swo_printf("[INFO] Homing: coarse limit found, retracting %ld counts\n",
                   static_cast<long>(HOMING_RETRACT_DISTANCE));
#endif
        break;
    case TASK_STATE_E::HOMING_SLOW:
        // Precise negative limit found
        m_pMotor->emergency_stop();
        m_pEncoder->set_position(0);
        m_pStall->reset();
        m_eTaskState = TASK_STATE_E::HOMING_SETTLE;
        m_pStall->set_direction(StallDetect::Direction::FORWARD);
        m_pStall->start_motor();
        m_pMotor->move_to(HOMING_SETTLE_DISTANCE);
#ifndef BUILD_TESTING
        swo_printf("[INFO] Homing: precise limit found, settling %ld counts\n",
                   static_cast<long>(HOMING_SETTLE_DISTANCE));
#endif
        break;
    case TASK_STATE_E::HOMING_RETRACT:
        // Stall during retract — unusual, skip to slow
        m_pMotor->emergency_stop();
        m_pStall->reset();
        m_eTaskState = TASK_STATE_E::HOMING_SLOW;
        m_pStall->set_direction(StallDetect::Direction::REVERSE);
        m_pStall->start_motor();
        m_pMotor->move_to(-HOMING_FAR_DISTANCE);
#ifndef BUILD_TESTING
        swo_printf("[WARN] Homing: stall during retract, proceeding to slow reverse\n");
#endif
        break;
    default:
        // Unexpected stall during normal operation
        m_pMotor->emergency_stop();
        m_pStall->reset();
        m_eTaskState = TASK_STATE_E::IDLE;
        m_pSm->transition_to(SYSTEM_STATE_E::READY);
        send_response(rsp_cmd::STALL_STOP, 0);
        send_save(save_reason::STALL);
#ifndef BUILD_TESTING
        swo_printf("[FAIL] Unexpected stall during operation\n");
#endif
        break;
    }
}

void MotorTask::handle_power_down() {
    m_pMotor->emergency_stop();
    g_bSpiEmergency = true;
    m_pFram->emergency_save(m_pEncoder->get_position());
    m_pStall->reset();
    m_eTaskState = TASK_STATE_E::IDLE;
    m_pSm->transition_to(SYSTEM_STATE_E::READY);
#ifndef BUILD_TESTING
    swo_printf("[WARN] Power down detected, emergency save position=%ld\n",
               static_cast<long>(m_pEncoder->get_position()));
#endif
}

void MotorTask::handle_overcurrent() {
    // During homing, overcurrent at physical limit is expected — treat as stall
    if (m_eTaskState == TASK_STATE_E::HOMING_FAST ||
        m_eTaskState == TASK_STATE_E::HOMING_SLOW ||
        m_eTaskState == TASK_STATE_E::HOMING_RETRACT ||
        m_eTaskState == TASK_STATE_E::HOMING_SETTLE) {
        handle_stall();
        return;
    }

    m_pMotor->emergency_stop();
    m_pStall->reset();
    m_eTaskState = TASK_STATE_E::IDLE;
    m_pSm->transition_to(SYSTEM_STATE_E::READY);
    send_response(rsp_cmd::OVERCURRENT, 0);
    send_save(save_reason::STALL);
#ifndef BUILD_TESTING
    uint16_t iAdcRaw = m_pAdcCurrent ? *m_pAdcCurrent : 0;
    uint16_t iAdcFilt = m_AdcCurrentFilter.get_filtered();
    swo_printf("[FAIL] Overcurrent detected, raw=%u filt=%u (threshold=%u), emergency stop\n",
               iAdcRaw, iAdcFilt, StallDetect::OVERCURRENT_THRESHOLD);
#endif
}

void MotorTask::start_cycle(int8_t iStep, uint8_t iDwell_x100ms) {
    m_iCycleStep = iStep;
    m_iCycleDwell_x100ms = iDwell_x100ms;
    m_iCycleDwellCount = 0;
    m_bCycleWaiting = false;

    // Start from current position's nearest zoom
    int32_t iPos = m_pEncoder->get_position();
    m_iCurrentZoom = m_pZoom->get_nearest_zoom(iPos);

    // Move to first target
    uint16_t iNextZoom = m_pZoom->get_next_zoom(m_iCurrentZoom, m_iCycleStep);
    if (iNextZoom == m_iCurrentZoom) {
        // At limit, reverse
        m_iCycleStep = -m_iCycleStep;
        iNextZoom = m_pZoom->get_next_zoom(m_iCurrentZoom, m_iCycleStep);
    }

    int32_t iTarget = m_pZoom->get_position(iNextZoom);
    m_pStall->set_direction(iTarget > iPos
        ? StallDetect::Direction::FORWARD : StallDetect::Direction::REVERSE);
    m_pStall->start_motor();
    m_pMotor->move_to(iTarget);
    m_iCurrentZoom = iNextZoom;
    m_eTaskState = TASK_STATE_E::CYCLING;
}

void MotorTask::stop_cycle() {
    if (m_eTaskState == TASK_STATE_E::CYCLING) {
        m_pMotor->emergency_stop();
        m_pStall->reset();
        m_eTaskState = TASK_STATE_E::IDLE;
        m_pSm->transition_to(SYSTEM_STATE_E::READY);
        send_response(cmd::CYCLE_STOP, rsp::OK);
    }
}

void MotorTask::process_cycling() {
    if (m_bCycleWaiting) {
        // Dwelling at current position
        m_iCycleDwellCount++;
        uint16_t iDwellTicks = static_cast<uint16_t>(m_iCycleDwell_x100ms) * 100;
        if (m_iCycleDwellCount >= iDwellTicks) {
            m_bCycleWaiting = false;
            // Move to next zoom
            uint16_t iNextZoom = m_pZoom->get_next_zoom(m_iCurrentZoom, m_iCycleStep);
            if (iNextZoom == m_iCurrentZoom) {
                // At limit, reverse direction
                m_iCycleStep = -m_iCycleStep;
                iNextZoom = m_pZoom->get_next_zoom(m_iCurrentZoom, m_iCycleStep);
            }
            int32_t iTarget = m_pZoom->get_position(iNextZoom);
            int32_t iPos = m_pEncoder->get_position();
            m_pStall->set_direction(iTarget > iPos
                ? StallDetect::Direction::FORWARD : StallDetect::Direction::REVERSE);
            m_pStall->start_motor();
            m_pMotor->move_to(iTarget);
            m_iCurrentZoom = iNextZoom;
        }
    } else {
        // Moving to target
        if (m_pMotor->get_state() == MOTOR_STATE_E::IDLE) {
            // Arrived at target, start dwell
            m_bCycleWaiting = true;
            m_iCycleDwellCount = 0;
            m_pStall->reset();
        }
    }
}

void MotorTask::start_backlash_measure() {
    m_pMotor->set_backlash_enabled(false);
    m_pMotor->set_speed_limit(HOMING_SLOW_SPEED);
    m_eBLPhase = BL_MEASURE_PHASE_E::MOVE_TO_MID;
    m_iBLSampleIdx = 0;
    m_eTaskState = TASK_STATE_E::BACKLASH_MEASURE;
    m_pStall->set_direction(StallDetect::Direction::FORWARD);
    m_pStall->start_motor();
    m_pMotor->move_to(BL_MEASURE_MID);
}

void MotorTask::process_backlash_measure() {
    if (m_pMotor->get_state() != MOTOR_STATE_E::IDLE) return;

    switch (m_eBLPhase) {
    case BL_MEASURE_PHASE_E::MOVE_TO_MID:
        // Arrived at middle, record reference position
        m_iBLRefPos = m_pEncoder->get_position();
        m_eBLPhase = BL_MEASURE_PHASE_E::REVERSE;
        m_pStall->set_direction(StallDetect::Direction::REVERSE);
        m_pStall->start_motor();
        m_pMotor->move_to(m_iBLRefPos - BL_REVERSE_DIST);
        break;

    case BL_MEASURE_PHASE_E::REVERSE:
        // Reversed, now go forward back to reference
        m_eBLPhase = BL_MEASURE_PHASE_E::FORWARD;
        m_pStall->set_direction(StallDetect::Direction::FORWARD);
        m_pStall->start_motor();
        m_pMotor->move_to(m_iBLRefPos);
        break;

    case BL_MEASURE_PHASE_E::FORWARD: {
        // Arrived, measure error
        int32_t iActual = m_pEncoder->get_position();
        int32_t iError = iActual - m_iBLRefPos;
        if (iError < 0) iError = -iError;
        m_aBLSamples[m_iBLSampleIdx++] = iError;

        if (m_iBLSampleIdx < 3) {
            // Next cycle: reverse again
            m_eBLPhase = BL_MEASURE_PHASE_E::REVERSE;
            m_pStall->set_direction(StallDetect::Direction::REVERSE);
            m_pStall->start_motor();
            m_pMotor->move_to(m_iBLRefPos - BL_REVERSE_DIST);
        } else {
            // Calculate average backlash, subtract DEADZONE
            int32_t iSum = m_aBLSamples[0] + m_aBLSamples[1] + m_aBLSamples[2];
            int32_t iAvg = iSum / 3;
            int16_t iBacklash = static_cast<int16_t>(
                iAvg > MotorCtrl::DEADZONE ? iAvg - MotorCtrl::DEADZONE : 0);
            m_pMotor->set_backlash(iBacklash);

            // Save backlash to FRAM
            {
                int32_t iPos = m_pEncoder->get_position();
                uint16_t iZoom = m_pZoom->get_nearest_zoom(iPos);
                SAVE_MESSAGE_S stSave = {iPos, iZoom, save_reason::ARRIVED,
                                          iBacklash, 0xFF};
                xQueueSend(m_saveQueue, &stSave, 0);
            }

            m_pMotor->set_speed_limit(MotorCtrl::MAX_SPEED);
            m_pStall->reset();

            if (m_bFullDiagnostics) {
                // Chain: backlash measurement → accuracy test
                start_accuracy_test();
            } else {
                m_eTaskState = TASK_STATE_E::IDLE;
                m_pSm->transition_to(SYSTEM_STATE_E::READY);
            }
        }
        break;
    }
    }
}

void MotorTask::start_accuracy_test() {
    m_pMotor->set_backlash_enabled(true);  // test WITH compensation
    m_eAccPhase = ACC_TEST_PHASE_E::MOVE_TO_START;
    m_iAccTripCount = 0;
    m_iAccErrorIdx = 0;
    m_eTaskState = TASK_STATE_E::ACCURACY_TEST;
    m_pStall->set_direction(StallDetect::Direction::FORWARD);
    m_pStall->start_motor();
    m_pMotor->move_to(ACC_START_POS);
}

void MotorTask::process_accuracy_test() {
    // Wait for full completion (including APPROACHING phase 2)
    if (m_pMotor->get_state() != MOTOR_STATE_E::IDLE) return;

    switch (m_eAccPhase) {
    case ACC_TEST_PHASE_E::MOVE_TO_START:
        m_iAccRefPos = m_pEncoder->get_position();
        m_eAccPhase = ACC_TEST_PHASE_E::MOVE_TO_END;
        m_pStall->set_direction(StallDetect::Direction::FORWARD);
        m_pStall->start_motor();
        m_pMotor->move_to(ACC_END_POS);
        break;

    case ACC_TEST_PHASE_E::MOVE_TO_END: {
        int32_t iError = m_pEncoder->get_position() - ACC_END_POS;
        m_aAccErrors[m_iAccErrorIdx++] = iError;
        m_eAccPhase = ACC_TEST_PHASE_E::MOVE_TO_START_RETURN;
        m_pStall->set_direction(StallDetect::Direction::REVERSE);
        m_pStall->start_motor();
        m_pMotor->move_to(ACC_START_POS);
        break;
    }

    case ACC_TEST_PHASE_E::MOVE_TO_START_RETURN: {
        int32_t iError = m_pEncoder->get_position() - ACC_START_POS;
        m_aAccErrors[m_iAccErrorIdx++] = iError;
        m_iAccTripCount++;
        if (m_iAccTripCount < ACC_NUM_TRIPS) {
            m_eAccPhase = ACC_TEST_PHASE_E::MOVE_TO_END;
            m_pStall->set_direction(StallDetect::Direction::FORWARD);
            m_pStall->start_motor();
            m_pMotor->move_to(ACC_END_POS);
        } else {
            print_accuracy_report();
            m_pStall->reset();
            m_eTaskState = TASK_STATE_E::IDLE;
            m_pSm->transition_to(SYSTEM_STATE_E::READY);
        }
        break;
    }
    }
}

void MotorTask::print_accuracy_report() {
#ifndef BUILD_TESTING
    int32_t iDrift = m_pEncoder->get_position() - m_iAccRefPos;
    int32_t iMaxErr = 0;
    bool bPass = true;

    swo_printf("\n============ ACCURACY TEST REPORT ============\n");
    swo_printf(" Backlash: %d counts\n", m_pMotor->get_backlash());

    for (uint8_t i = 0; i < m_iAccErrorIdx; ++i) {
        int32_t iErr = m_aAccErrors[i];
        if (iErr < 0 && -iErr > iMaxErr) iMaxErr = -iErr;
        if (iErr > 0 && iErr > iMaxErr) iMaxErr = iErr;
        if (iErr > MotorCtrl::DEADZONE || iErr < -MotorCtrl::DEADZONE) bPass = false;
        swo_printf(" Trip %u %s: error=%ld counts\n",
                   i / 2 + 1, (i % 2 == 0) ? "FWD" : "REV",
                   static_cast<long>(iErr));
    }

    if (iDrift > MotorCtrl::DEADZONE || iDrift < -MotorCtrl::DEADZONE) bPass = false;
    swo_printf(" Cumulative drift: %ld counts\n", static_cast<long>(iDrift));
    swo_printf(" Max error: %ld counts\n", static_cast<long>(iMaxErr));
    swo_printf(" RESULT: %s\n", bPass ? "PASS" : "FAIL");
    swo_printf("===============================================\n");
#endif
}

void MotorTask::send_response(uint8_t cmd, uint16_t param) {
    RSP_MESSAGE_S stRsp = {cmd, param};
    xQueueSend(m_rspQueue, &stRsp, 0);
}

void MotorTask::send_save(uint8_t reason) {
    int32_t iPos = m_pEncoder->get_position();
    uint16_t iZoom = m_pZoom->get_nearest_zoom(iPos);
    SAVE_MESSAGE_S stSave = {iPos, iZoom, reason, 0, 0};
    xQueueSend(m_saveQueue, &stSave, 0);
}

} // namespace zlens

extern "C" void motor_task_entry(void* params) {
#ifndef BUILD_TESTING
    (void)params;
    using namespace zlens;

    static MotorTask task;
    task.init(&g_Motor, &g_Encoder, &g_StallDetect, &g_ZoomTable,
              &g_FramStorage, &g_SystemManager, g_cmdQueue, g_rspQueue,
              g_saveQueue, const_cast<uint16_t*>(&g_aAdcDmaBuf[0]));

    swo_printf("[MOTOR] Task started\n");

    TickType_t xLastWake = xTaskGetTickCount();
    uint32_t iHeartbeat = 0;
    for (;;) {
        task.run_once();

        // Heartbeat every 5s
        if (++iHeartbeat >= 5000) {
            iHeartbeat = 0;
            swo_printf("[MOTOR] HB state=%d motor=%d pos=%ld\n",
                       (int)task.get_state(),
                       (int)g_Motor.get_state(),
                       static_cast<long>(g_Encoder.get_position()));
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(1));
    }
#else
    (void)params;
#endif
}
