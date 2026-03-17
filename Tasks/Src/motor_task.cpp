// Tasks/Src/motor_task.cpp
#include "motor_task.hpp"
#ifndef BUILD_TESTING
#include "app_instances.hpp"
#endif

namespace zlens {

void MotorTask::init(MotorCtrl* pMotor, Encoder* pEncoder, StallDetect* pStall,
                     ZoomTable* pZoom, QueueHandle_t cmdQ, QueueHandle_t rspQ,
                     QueueHandle_t saveQ, uint16_t* pAdcCurrent) {
    m_pMotor = pMotor;
    m_pEncoder = pEncoder;
    m_pStall = pStall;
    m_pZoom = pZoom;
    m_cmdQueue = cmdQ;
    m_rspQueue = rspQ;
    m_saveQueue = saveQ;
    m_pAdcCurrent = pAdcCurrent;
    m_eTaskState = TASK_STATE_E::IDLE;
    m_bHomingDone = false;
    m_iZOffset = 0;
    m_iTotalRange = 0;
}

void MotorTask::run_once() {
    int32_t iPos = m_pEncoder->get_position();
    uint16_t iAdcCurrent = m_pAdcCurrent ? *m_pAdcCurrent : 0;

    m_pStall->update(iAdcCurrent, iPos, HAL_GetTick());
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
    case TASK_STATE_E::HOMING_REVERSE:
    case TASK_STATE_E::HOMING_RETRACT:
    case TASK_STATE_E::HOMING_FIND_Z:
    case TASK_STATE_E::HOMING_FORWARD:
    case TASK_STATE_E::HOMING_TO_SOFT_MIN:
        process_homing();
        break;
    case TASK_STATE_E::CYCLING:
        process_cycling();
        break;
    default:
        break;
    }
}

void MotorTask::dispatch_command(const CMD_MESSAGE_S& stCmd) {
    switch (stCmd.cmd) {
    case cmd::SET_ZOOM: {
        if (m_eTaskState != TASK_STATE_E::IDLE) break;
        int32_t iTarget = m_pZoom->get_position(stCmd.param);
        iTarget = clamp_to_soft_limits(iTarget);
        m_pStall->set_direction(iTarget > m_pEncoder->get_position()
            ? StallDetect::Direction::FORWARD : StallDetect::Direction::REVERSE);
        m_pStall->start_motor();
        m_pMotor->move_to(iTarget);
        m_eTaskState = TASK_STATE_E::MOVING;
        break;
    }
    case cmd::HOMING:
        if (m_eTaskState != TASK_STATE_E::IDLE) break;
        start_homing();
        break;
    case cmd::FORCE_STOP:
        m_pMotor->emergency_stop();
        m_pStall->reset();
        if (m_eTaskState == TASK_STATE_E::CYCLING) {
            m_bCycleWaiting = false;
        }
        m_eTaskState = TASK_STATE_E::IDLE;
        send_response(cmd::FORCE_STOP, rsp::OK);
        break;
    case cmd::CYCLE_START:
        if (m_eTaskState != TASK_STATE_E::IDLE) break;
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

void MotorTask::start_homing() {
    m_bHomingDone = false;
    m_eTaskState = TASK_STATE_E::HOMING_REVERSE;
    m_pStall->set_direction(StallDetect::Direction::REVERSE);
    m_pStall->start_motor();
    m_pMotor->move_to(-HOMING_FAR_DISTANCE);
}

void MotorTask::process_moving() {
    if (m_pMotor->get_state() == MOTOR_STATE_E::IDLE) {
        int32_t iPos = m_pEncoder->get_position();
        uint16_t iZoom = m_pZoom->get_nearest_zoom(iPos);
        m_pStall->reset();
        send_response(cmd::SET_ZOOM, iZoom);
        send_save(save_reason::ARRIVED);
        m_eTaskState = TASK_STATE_E::IDLE;
    }
}

void MotorTask::process_homing() {
    switch (m_eTaskState) {
    case TASK_STATE_E::HOMING_RETRACT:
        if (m_pMotor->get_state() == MOTOR_STATE_E::IDLE) {
            // Retract done, record Z offset and start forward
            m_iZOffset = m_pEncoder->get_z_position();
            m_eTaskState = TASK_STATE_E::HOMING_FORWARD;
            m_pStall->set_direction(StallDetect::Direction::FORWARD);
            m_pStall->start_motor();
            m_pMotor->move_to(HOMING_FAR_DISTANCE);
        }
        break;
    case TASK_STATE_E::HOMING_TO_SOFT_MIN:
        if (m_pMotor->get_state() == MOTOR_STATE_E::IDLE) {
            // Homing complete
            m_pZoom->set_total_range(m_iTotalRange);
            m_bHomingDone = true;
            m_pStall->reset();
            send_response(cmd::HOMING, rsp::OK);
            send_save(save_reason::ARRIVED);
            m_eTaskState = TASK_STATE_E::IDLE;
        }
        break;
    default:
        break;
    }
}

void MotorTask::handle_stall() {
    switch (m_eTaskState) {
    case TASK_STATE_E::HOMING_REVERSE:
        // Expected stall: at min limit, set position to 0
        m_pMotor->emergency_stop();
        m_pEncoder->set_position(0);
        m_pStall->reset();
        m_eTaskState = TASK_STATE_E::HOMING_RETRACT;
        m_pMotor->move_to(HOMING_RETRACT_DISTANCE);
        break;
    case TASK_STATE_E::HOMING_FORWARD:
        // Expected stall: at max limit, record total range
        m_pMotor->emergency_stop();
        m_iTotalRange = m_pEncoder->get_position();
        m_pStall->reset();
        m_eTaskState = TASK_STATE_E::HOMING_TO_SOFT_MIN;
        m_pMotor->move_to(SOFT_LIMIT_OFFSET);
        break;
    default:
        // Unexpected stall during normal operation
        m_pMotor->emergency_stop();
        m_pStall->reset();
        m_eTaskState = TASK_STATE_E::IDLE;
        send_response(cmd::FORCE_STOP, rsp::STALL_ALARM);
        send_save(save_reason::STALL);
        break;
    }
}

void MotorTask::handle_overcurrent() {
    m_pMotor->emergency_stop();
    m_pStall->reset();
    m_eTaskState = TASK_STATE_E::IDLE;
    send_response(cmd::FORCE_STOP, rsp::OVERCURRENT);
    send_save(save_reason::STALL);
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
    iTarget = clamp_to_soft_limits(iTarget);
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
            iTarget = clamp_to_soft_limits(iTarget);
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

void MotorTask::send_response(uint8_t cmd, uint16_t param) {
    RSP_MESSAGE_S stRsp = {cmd, param};
    xQueueSend(m_rspQueue, &stRsp, 0);
}

void MotorTask::send_save(uint8_t reason) {
    int32_t iPos = m_pEncoder->get_position();
    uint16_t iZoom = m_pZoom->get_nearest_zoom(iPos);
    SAVE_MESSAGE_S stSave = {iPos, iZoom, reason};
    xQueueSend(m_saveQueue, &stSave, 0);
}

int32_t MotorTask::clamp_to_soft_limits(int32_t iTarget) const {
    if (!m_bHomingDone || m_iTotalRange == 0) return iTarget;
    int32_t iMin = SOFT_LIMIT_OFFSET;
    int32_t iMax = m_iTotalRange - SOFT_LIMIT_OFFSET;
    if (iTarget < iMin) return iMin;
    if (iTarget > iMax) return iMax;
    return iTarget;
}

} // namespace zlens

extern "C" void motor_task_entry(void* params) {
#ifndef BUILD_TESTING
    (void)params;
    using namespace zlens;

    static MotorTask task;
    task.init(&g_Motor, &g_Encoder, &g_StallDetect, &g_ZoomTable,
              g_cmdQueue, g_rspQueue, g_saveQueue,
              const_cast<uint16_t*>(&g_aAdcDmaBuf[0]));

    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        task.run_once();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(1));
    }
#else
    (void)params;
#endif
}
