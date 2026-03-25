// Tasks/Src/motor_task.cpp
#include "motor_task.hpp"
#ifndef BUILD_TESTING
#include "app_instances.hpp"
#include "swo_debug.hpp"
#endif

namespace zlens {

constexpr uint16_t MotorTask::STALL_TEST_SPEEDS[3];

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

    // Overcurrent: highest priority (any state except IDLE and STALL_CURRENT_TEST)
    if (m_pStall->is_overcurrent() && m_eTaskState != TASK_STATE_E::IDLE
        && m_eTaskState != TASK_STATE_E::STALL_CURRENT_TEST) {
        handle_overcurrent();
        return;
    }

    // Stall detection (skip during stall current test — we want to stay stalled)
    if (m_pStall->is_stalled() && m_eTaskState != TASK_STATE_E::STALL_CURRENT_TEST) {
#ifndef BUILD_TESTING
        swo_printf("[STALL] current stall state=%d pos=%ld adc=%u\n",
                   (int)m_eTaskState, static_cast<long>(iPos), iAdcFiltered);
#endif
        handle_stall();
        return;
    }

    // Encoder stall detection (homing only — mechanical limit stops encoder)
    if (m_pStall->encoder_stalled() &&
        (m_eTaskState == TASK_STATE_E::HOMING_FAST ||
         m_eTaskState == TASK_STATE_E::HOMING_SLOW)) {
#ifndef BUILD_TESTING
        swo_printf("[STALL] encoder stall state=%d pos=%ld adc=%u\n",
                   (int)m_eTaskState, static_cast<long>(iPos), iAdcFiltered);
#endif
        handle_stall();
        return;
    }

    // PID motor stall (non-homing — motor encoder timeout while PID-positioning)
    if (m_pMotor->get_state() == MOTOR_STATE_E::STALLED &&
        m_eTaskState != TASK_STATE_E::IDLE &&
        m_eTaskState != TASK_STATE_E::STALL_CURRENT_TEST) {
#ifndef BUILD_TESTING
        swo_printf("[STALL] motor timeout stall state=%d pos=%ld\n",
                   (int)m_eTaskState, static_cast<long>(iPos));
#endif
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
    case TASK_STATE_E::STALL_CURRENT_TEST:
        process_stall_current_test();
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
        if (!m_pZoom->is_valid_zoom(stCmd.param)) {
            send_response(rsp_cmd::ERR_PARAM, stCmd.param);
            break;
        }
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
        if (stCmd.param == 2) {
            // Stall current test: home first, then test
            m_pSm->transition_to(SYSTEM_STATE_E::HOMING);
            m_bStallTestPending = true;
            start_homing();
        } else {
            m_pSm->transition_to(SYSTEM_STATE_E::HOMING);
            start_homing();
        }
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
        int32_t iPos = m_pEncoder->get_position();
        uint16_t iCurZoom = m_pZoom->get_nearest_zoom(iPos);
        uint16_t iNewZoom = m_pZoom->get_next_zoom(iCurZoom, static_cast<int8_t>(stCmd.param));
        if (iNewZoom == iCurZoom) {
            send_response(rsp_cmd::ERR_PARAM, stCmd.param);
            break;
        }
        m_pSm->transition_to(SYSTEM_STATE_E::BUSY);
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
        int32_t iPos = m_pEncoder->get_position();
        uint16_t iCurZoom = m_pZoom->get_nearest_zoom(iPos);
        uint16_t iNewZoom = m_pZoom->get_next_zoom(iCurZoom, -static_cast<int8_t>(stCmd.param));
        if (iNewZoom == iCurZoom) {
            send_response(rsp_cmd::ERR_PARAM, stCmd.param);
            break;
        }
        m_pSm->transition_to(SYSTEM_STATE_E::BUSY);
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
    // 0x60-0x64: Speed commands
    case cmd::SET_SPEED: {
        uint16_t iDuty = stCmd.param;
        if (iDuty > 1000) iDuty = 1000;
        if (iDuty < m_iMinSpeedDuty) iDuty = m_iMinSpeedDuty;
        if (iDuty > m_iMaxSpeedDuty) iDuty = m_iMaxSpeedDuty;
        m_iSpeedDuty = iDuty;
        apply_speed_to_motor();
        send_response(rsp_cmd::SPEED, m_iSpeedDuty);
        send_save(save_reason::ARRIVED, 0, 0, true);
        break;
    }
    case cmd::SPEED_INC:
        if (m_iSpeedDuty < m_iMaxSpeedDuty) ++m_iSpeedDuty;
        apply_speed_to_motor();
        send_response(rsp_cmd::SPEED, m_iSpeedDuty);
        break;
    case cmd::SPEED_DEC:
        if (m_iSpeedDuty > m_iMinSpeedDuty) --m_iSpeedDuty;
        apply_speed_to_motor();
        send_response(rsp_cmd::SPEED, m_iSpeedDuty);
        break;
    case cmd::SET_MIN_SPEED: {
        uint16_t iDuty = stCmd.param;
        if (iDuty > 1000) iDuty = 1000;
        m_iMinSpeedDuty = iDuty;
        if (m_iSpeedDuty < m_iMinSpeedDuty) {
            m_iSpeedDuty = m_iMinSpeedDuty;
            apply_speed_to_motor();
        }
        send_response(rsp_cmd::SPEED, m_iMinSpeedDuty);
        send_save(save_reason::ARRIVED, 0, 0, true);
        break;
    }
    case cmd::SET_MAX_SPEED: {
        uint16_t iDuty = stCmd.param;
        if (iDuty > 1000) iDuty = 1000;
        m_iMaxSpeedDuty = iDuty;
        if (m_iSpeedDuty > m_iMaxSpeedDuty) {
            m_iSpeedDuty = m_iMaxSpeedDuty;
            apply_speed_to_motor();
        }
        send_response(rsp_cmd::SPEED, m_iMaxSpeedDuty);
        send_save(save_reason::ARRIVED, 0, 0, true);
        break;
    }
    case cmd::SELF_TEST:
        if (m_eTaskState != TASK_STATE_E::IDLE) break;
        m_pSm->transition_to(SYSTEM_STATE_E::HOMING);
        start_homing();
        break;
    default:
        break;
    }
}

void MotorTask::start_homing() {
    m_bHomingDone = false;
    m_eTaskState = TASK_STATE_E::HOMING_FAST;
    m_pStall->set_direction(StallDetect::Direction::REVERSE);
    m_pStall->start_motor();
    // Drive motor directly (not PID) — seeking physical limit
    m_pMotor->set_pwm_test(DIRECTION_E::REVERSE, MotorCtrl::MAX_SPEED / 2);
#ifndef BUILD_TESTING
    swo_printf("[INFO] Homing started: fast reverse, pos=%ld motor=%d\n",
               static_cast<long>(m_pEncoder->get_position()),
               (int)m_pMotor->get_state());
#endif
}

void MotorTask::process_moving() {
    if (m_pMotor->get_state() == MOTOR_STATE_E::IDLE) {
        int32_t iPos = m_pEncoder->get_position();
        uint16_t iZoom = m_pZoom->get_nearest_zoom(iPos);
        m_pStall->reset();
        send_response(rsp_cmd::ZOOM, iZoom);
        send_response(rsp_cmd::ARRIVED, rsp::ARRIVED_PARAM);
        send_save(save_reason::ARRIVED, 0, 0xFF);  // position_valid
        m_eTaskState = TASK_STATE_E::IDLE;
        m_pSm->transition_to(SYSTEM_STATE_E::READY);
    }
}

void MotorTask::process_homing() {
    switch (m_eTaskState) {
    case TASK_STATE_E::HOMING_RETRACT:
        if (m_pMotor->get_state() == MOTOR_STATE_E::IDLE) {
            // Retract done -> start slow reverse
            m_eTaskState = TASK_STATE_E::HOMING_SLOW;
            m_pStall->set_direction(StallDetect::Direction::REVERSE);
            m_pStall->start_motor();
            // Drive motor directly (not PID) — seeking physical limit
            m_pMotor->set_pwm_test(DIRECTION_E::REVERSE, MotorCtrl::MIN_SPEED);
#ifndef BUILD_TESTING
            swo_printf("[INFO] Homing: retract done pos=%ld, starting slow reverse\n",
                       static_cast<long>(m_pEncoder->get_position()));
#endif
        }
        break;
    case TASK_STATE_E::HOMING_SETTLE:
        if (m_pMotor->get_state() == MOTOR_STATE_E::IDLE) {
            // Settle done -> homing complete
            m_bHomingDone = true;
            m_pStall->reset();
            m_pEncoder->reset_index_tracking();
            int32_t iPos = m_pEncoder->get_position();
            uint16_t iZoom = m_pZoom->get_nearest_zoom(iPos);
            send_response(rsp_cmd::ZOOM, iZoom);
            send_response(rsp_cmd::HOMING_DONE, rsp::HOMING_DONE_PARAM);
            send_save(save_reason::ARRIVED, 1, 0xFF);  // homing_done + position_valid

            if (m_bStallTestPending) {
                m_bStallTestPending = false;
                start_stall_current_test();
            } else {
                m_eTaskState = TASK_STATE_E::IDLE;
                m_pSm->transition_to(SYSTEM_STATE_E::READY);
            }
#ifndef BUILD_TESTING
            swo_printf("[PASS] Homing complete: pos=%ld\n",
                       static_cast<long>(m_pEncoder->get_position()));
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
        m_pMotor->set_pwm_test(DIRECTION_E::REVERSE, MotorCtrl::MIN_SPEED);
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

// --- Settle detection helpers ---

void MotorTask::reset_settle() {
    m_iSettleCount = 0;
    m_iSettleLastPos = m_pEncoder->get_position();
    m_bSettling = true;
}

bool MotorTask::is_settled() {
    if (!m_bSettling) return true;
    int32_t iPos = m_pEncoder->get_position();
    if (iPos != m_iSettleLastPos) {
        m_iSettleLastPos = iPos;
        m_iSettleCount = 0;
        return false;
    }
    if (++m_iSettleCount >= SETTLE_STABLE_COUNT) {
        m_bSettling = false;
        return true;
    }
    return false;
}

// --- Stall current test ---

void MotorTask::start_stall_current_test() {
    m_iStallSpeedIdx = 0;
    m_eStallTestPhase = STALL_TEST_PHASE_E::MOVE_TO_2X;
    m_eTaskState = TASK_STATE_E::STALL_CURRENT_TEST;

    // Move to 2.0x position
    int32_t iTarget = m_pZoom->get_position(STALL_TEST_ZOOM_X10);  // 2.0x
    m_pStall->set_direction(StallDetect::Direction::FORWARD);
    m_pStall->start_motor();
    m_pMotor->move_to(iTarget);
#ifndef BUILD_TESTING
    swo_printf("\n============ STALL CURRENT TEST ============\n");
    swo_printf(" Speeds: 30%% (%u), 50%% (%u), 80%% (%u)\n",
               STALL_TEST_SPEEDS[0], STALL_TEST_SPEEDS[1], STALL_TEST_SPEEDS[2]);
    swo_printf(" Dwell: %u ms per speed\n", STALL_TEST_DWELL_MS);
    swo_printf(" Moving to 2.0x...\n");
#endif
}

void MotorTask::process_stall_current_test() {
    uint16_t iAdcRaw = m_pAdcCurrent ? *m_pAdcCurrent : 0;

    switch (m_eStallTestPhase) {
    case STALL_TEST_PHASE_E::MOVE_TO_2X:
        if (m_pMotor->get_state() != MOTOR_STATE_E::IDLE) return;
        reset_settle();
        m_eStallTestPhase = STALL_TEST_PHASE_E::SETTLE_2X;
        break;

    case STALL_TEST_PHASE_E::SETTLE_2X:
        if (!is_settled()) return;
        m_eStallTestPhase = STALL_TEST_PHASE_E::START_STALL;
        break;

    case STALL_TEST_PHASE_E::START_STALL: {
        // Set test speed and drive toward negative limit
        uint16_t iSpeed = STALL_TEST_SPEEDS[m_iStallSpeedIdx];
        m_pMotor->set_pwm_test(DIRECTION_E::REVERSE, iSpeed);
        m_pStall->set_direction(StallDetect::Direction::REVERSE);
        m_pStall->start_motor();

        // Reset ADC sampling
        m_iStallDwellCount = 0;
        m_iStallAdcSum = 0;
        m_iStallAdcMax = 0;
        m_iStallAdcMin = 0xFFFF;
        m_iStallAdcSamples = 0;

        m_eStallTestPhase = STALL_TEST_PHASE_E::STALL_DWELL;
#ifndef BUILD_TESTING
        uint32_t iPct = static_cast<uint32_t>(iSpeed) * 100 / MotorCtrl::PWM_ARR;
        swo_printf("\n--- Speed %u/%u: %u (%lu%%) ---\n",
                   m_iStallSpeedIdx + 1, STALL_TEST_NUM_SPEEDS, iSpeed, iPct);
        swo_printf(" Driving toward negative limit...\n");
#endif
        break;
    }

    case STALL_TEST_PHASE_E::STALL_DWELL:
        // Sample ADC every tick
        m_iStallAdcSum += iAdcRaw;
        m_iStallAdcSamples++;
        if (iAdcRaw > m_iStallAdcMax) m_iStallAdcMax = iAdcRaw;
        if (iAdcRaw < m_iStallAdcMin) m_iStallAdcMin = iAdcRaw;

        m_iStallDwellCount++;

        // Reset stall detector to prevent it from accumulating
        if (m_pStall->is_stalled()) {
            m_pStall->reset();
            m_pStall->set_direction(StallDetect::Direction::REVERSE);
            m_pStall->start_motor();
        }

#ifndef BUILD_TESTING
        // Print ADC every 500ms
        if (m_iStallDwellCount % STALL_PRINT_INTERVAL == 0) {
            uint16_t iAvgSoFar = static_cast<uint16_t>(m_iStallAdcSum / m_iStallAdcSamples);
            swo_printf(" [%us] ADC raw=%u avg=%u min=%u max=%u\n",
                       m_iStallDwellCount / 1000, iAdcRaw, iAvgSoFar,
                       m_iStallAdcMin, m_iStallAdcMax);
        }
#endif

        if (m_iStallDwellCount >= STALL_TEST_DWELL_MS) {
            // Dwell complete — record results
            uint16_t iAvg = static_cast<uint16_t>(m_iStallAdcSum / m_iStallAdcSamples);
            m_aStallResults[m_iStallSpeedIdx][0] = iAvg;
            m_aStallResults[m_iStallSpeedIdx][1] = m_iStallAdcMin;
            m_aStallResults[m_iStallSpeedIdx][2] = m_iStallAdcMax;

            // Stop motor, return to 2x
            m_pMotor->emergency_stop();
            m_pStall->reset();
            m_eStallTestPhase = STALL_TEST_PHASE_E::STOP_AND_RETURN;

#ifndef BUILD_TESTING
            swo_printf(" DONE: avg=%u min=%u max=%u (samples=%u)\n",
                       iAvg, m_iStallAdcMin, m_iStallAdcMax, m_iStallAdcSamples);
#endif
        }
        break;

    case STALL_TEST_PHASE_E::STOP_AND_RETURN: {
        // Return to 2.0x
        int32_t iTarget = m_pZoom->get_position(STALL_TEST_ZOOM_X10);
        m_pStall->set_direction(StallDetect::Direction::FORWARD);
        m_pStall->start_motor();
        m_pMotor->move_to(iTarget);
        m_eStallTestPhase = STALL_TEST_PHASE_E::SETTLE_RETURN;
        break;
    }

    case STALL_TEST_PHASE_E::SETTLE_RETURN:
        if (m_pMotor->get_state() != MOTOR_STATE_E::IDLE) return;
        reset_settle();

        m_iStallSpeedIdx++;
        if (m_iStallSpeedIdx < STALL_TEST_NUM_SPEEDS) {
            // Next speed
            m_eStallTestPhase = STALL_TEST_PHASE_E::START_STALL;
        } else {
            // All speeds done
            if (!is_settled()) return;
            print_stall_current_report();
            m_pStall->reset();
            m_eTaskState = TASK_STATE_E::IDLE;
            m_pSm->transition_to(SYSTEM_STATE_E::READY);
        }
        break;
    }
}

void MotorTask::print_stall_current_report() {
#ifndef BUILD_TESTING
    swo_printf("\n============ STALL CURRENT RESULTS ============\n");
    swo_printf(" %-8s  %-8s  %-8s  %-8s  %-8s\n",
               "Speed", "Duty%", "ADC_avg", "ADC_min", "ADC_max");
    for (uint8_t i = 0; i < STALL_TEST_NUM_SPEEDS; ++i) {
        uint32_t iPct = static_cast<uint32_t>(STALL_TEST_SPEEDS[i]) * 100
                        / MotorCtrl::PWM_ARR;
        swo_printf(" %-8u  %-8lu  %-8u  %-8u  %-8u\n",
                   STALL_TEST_SPEEDS[i], iPct,
                   m_aStallResults[i][0], m_aStallResults[i][1],
                   m_aStallResults[i][2]);
    }
    swo_printf("================================================\n");
#endif
}

void MotorTask::send_response(uint8_t cmd, uint16_t param) {
    RSP_MESSAGE_S stRsp = {cmd, param};
    xQueueSend(m_rspQueue, &stRsp, 0);
}

void MotorTask::apply_speed_to_motor() {
    // Speed commands are kept for protocol compatibility but PID controls actual motor speed.
    // This is a no-op in PID mode — the PID controller decides PWM based on position error.
}

void MotorTask::restore_speed(uint16_t iSpeedDuty, uint16_t iMinDuty, uint16_t iMaxDuty) {
    m_iMinSpeedDuty = (iMinDuty > 0) ? iMinDuty : rsp::DEFAULT_MIN_SPEED_DUTY;
    m_iMaxSpeedDuty = (iMaxDuty > 0) ? iMaxDuty : rsp::DEFAULT_MAX_SPEED_DUTY;
    m_iSpeedDuty = iSpeedDuty;
    if (m_iSpeedDuty < m_iMinSpeedDuty) m_iSpeedDuty = m_iMinSpeedDuty;
    if (m_iSpeedDuty > m_iMaxSpeedDuty) m_iSpeedDuty = m_iMaxSpeedDuty;
    apply_speed_to_motor();
}

void MotorTask::send_save(uint8_t reason, uint8_t homing_done,
                          uint8_t position_valid, bool bSaveSpeed) {
    int32_t iPos = m_pEncoder->get_position();
    uint16_t iZoom = m_pZoom->get_nearest_zoom(iPos);
    SAVE_MESSAGE_S stSave = {iPos, iZoom, reason, 0, 0, homing_done, position_valid,
                             m_iSpeedDuty, m_iMinSpeedDuty, m_iMaxSpeedDuty,
                             static_cast<uint8_t>(bSaveSpeed ? 1 : 0)};
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

    // Restore speed from FRAM (version >= 3)
    {
        FRAM_PARAMS_S stParams;
        if (g_FramStorage.load_params(stParams) &&
            FramStorage::verify_crc(stParams) && FramStorage::check_magic(stParams) &&
            stParams.version >= 3) {
            task.restore_speed(stParams.speed_duty, stParams.min_speed_duty,
                               stParams.max_speed_duty);
            swo_printf("[MOTOR] Speed restored: duty=%u min=%u max=%u\n",
                       stParams.speed_duty, stParams.min_speed_duty,
                       stParams.max_speed_duty);
        }
    }

    swo_printf("[MOTOR] Task started\n");

    TickType_t xLastWake = xTaskGetTickCount();
    uint32_t iHeartbeat = 0;
    for (;;) {
        task.run_once();

        // Heartbeat every 5s
        if (++iHeartbeat >= MotorTask::HEARTBEAT_INTERVAL) {
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
