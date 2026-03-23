// Tasks/Src/monitor_task.cpp
#include "monitor_task.hpp"
#ifndef BUILD_TESTING
#include "app_instances.hpp"
#include "swo_debug.hpp"
#endif

namespace zlens {

void MonitorTask::init(SystemManager* pSm, PowerMonitor* pPm,
                       FramStorage* pFram, TaskHandle_t hMotorTask,
                       IWDG_HandleTypeDef* pHiwdg, uint16_t* pAdcVoltage,
                       Encoder* pEncoder, MotorCtrl* pMotor, ZoomTable* pZoom) {
    m_pSm = pSm;
    m_pPm = pPm;
    m_pFram = pFram;
    m_hMotorTask = hMotorTask;
    m_pHiwdg = pHiwdg;
    m_pAdcVoltage = pAdcVoltage;
    m_pZoom = pZoom;
    m_pEncoder = pEncoder;
    m_pMotor = pMotor;
    m_bSelfTestDone = false;
    m_bSelfTestPassed = false;
    m_bBootDecided = false;
    m_bNormalBoot = false;
    m_bNormalBootStarted = false;
    m_bNeedHomingOnly = false;
    m_bHomingTriggered = false;
}

void MonitorTask::run_once() {
    SYSTEM_STATE_E eState = m_pSm->get_state();

    switch (eState) {
    case SYSTEM_STATE_E::SELF_TEST:
        feed_watchdog();

        if (!m_bBootDecided) {
            // Decide boot path: normal / homing-only / full self-test
            FRAM_PARAMS_S stParams{};
            bool bValid = m_pFram->load_params(stParams);
            if (bValid && FramStorage::check_magic(stParams) &&
                FramStorage::verify_crc(stParams) && stParams.version == 2) {
                if (stParams.position_valid == 0xFF && stParams.homing_done == 1) {
                    m_bNormalBoot = true;
                } else if (stParams.homing_done == 1) {
                    m_bNeedHomingOnly = true;
                }
            }
            m_bBootDecided = true;
        }

        if (m_bNormalBoot) {
            start_normal_boot();
        } else if (m_bNeedHomingOnly) {
            CMD_MESSAGE_S stCmd = {cmd::HOMING, 0};
            xQueueSend(g_cmdQueue, &stCmd, 0);
            m_pSm->transition_to(SYSTEM_STATE_E::HOMING);
#ifndef BUILD_TESTING
            swo_printf("[BOOT] Homing-only boot\n");
#endif
            m_bSelfTestDone = true;
            m_bSelfTestPassed = true;
        } else {
            // First boot (no valid FRAM): homing with default backlash
            m_pMotor->set_backlash(384);
            CMD_MESSAGE_S stCmd = {cmd::HOMING, 0};
            xQueueSend(g_cmdQueue, &stCmd, 0);
            m_pSm->transition_to(SYSTEM_STATE_E::HOMING);
#ifndef BUILD_TESTING
            swo_printf("[BOOT] First boot: homing with default backlash\n");
#endif
            m_bSelfTestDone = true;
            m_bSelfTestPassed = true;
        }
        break;

    case SYSTEM_STATE_E::HOMING:
    case SYSTEM_STATE_E::BUSY:
        feed_watchdog();
        check_voltage();
        break;

    case SYSTEM_STATE_E::READY:
        feed_watchdog();
        check_voltage();
        break;

    case SYSTEM_STATE_E::ERROR_STATE:
        feed_watchdog();
        break;

    default:
        break;
    }
}

void MonitorTask::start_normal_boot() {
    m_bNormalBootStarted = true;

    // Voltage check
    if (m_pAdcVoltage && m_pPm->is_power_down(*m_pAdcVoltage)) {
#ifndef BUILD_TESTING
        swo_printf("[FAIL] Normal boot: voltage too low\n");
#endif
        m_bSelfTestDone = true;
        m_bSelfTestPassed = false;
        m_pSm->transition_to(SYSTEM_STATE_E::ERROR_STATE);
        return;
    }

    // Position was already restored by StorageTask at startup (restore_params)
    // Find nearest zoom and check if we need to move
    int32_t iCurrentPos = m_pEncoder->get_position();
    uint16_t iNearestZoom = m_pZoom->get_nearest_zoom(iCurrentPos);
    int32_t iTargetPos = m_pZoom->get_position(iNearestZoom);
    int32_t iDelta = iCurrentPos - iTargetPos;
    if (iDelta < 0) iDelta = -iDelta;

    if (iDelta > MotorCtrl::DEADZONE) {
        // Move to nearest zoom position
        CMD_MESSAGE_S stCmd = {cmd::SET_ZOOM, iNearestZoom};
        xQueueSend(g_cmdQueue, &stCmd, 0);
#ifndef BUILD_TESTING
        swo_printf("[INFO] Normal boot: moving to zoom=%u.%u (delta=%ld)\n",
                   iNearestZoom / 10, iNearestZoom % 10, static_cast<long>(iDelta));
#endif
    }

    m_bSelfTestDone = true;
    m_bSelfTestPassed = true;
    m_pSm->transition_to(SYSTEM_STATE_E::READY);
#ifndef BUILD_TESTING
    swo_printf("[PASS] Normal boot: skipped self-test, READY\n");
#endif
}

void MonitorTask::run_normal_boot() {
    // Normal boot completes in start_normal_boot(), nothing to do here
}

void MonitorTask::feed_watchdog() {
    if (m_pHiwdg) {
        HAL_IWDG_Refresh(m_pHiwdg);
    }
}

void MonitorTask::check_voltage() {
    if (!m_pAdcVoltage) return;
    uint16_t iAdcVal = *m_pAdcVoltage;

    if (m_pPm->is_power_down(iAdcVal)) {
        xTaskNotify(m_hMotorTask, 2, eSetBits);
    }
}

} // namespace zlens

extern "C" void monitor_task_entry(void* params) {
#ifndef BUILD_TESTING
    using namespace zlens;

    TaskHandle_t hMotorTask = static_cast<TaskHandle_t>(params);
    extern IWDG_HandleTypeDef hiwdg;

    static MonitorTask task;
    task.init(&g_SystemManager, &g_PowerMonitor, &g_FramStorage,
              hMotorTask, &hiwdg,
              const_cast<uint16_t*>(&g_aAdcDmaBuf[1]),
              &g_Encoder, &g_Motor, &g_ZoomTable);

    g_SystemManager.transition_to(SYSTEM_STATE_E::SELF_TEST);
    swo_printf("[BOOT] ZLENS_DC v1.0 starting...\n");

    // Wait for ADC DMA to complete at least one conversion cycle
    vTaskDelay(pdMS_TO_TICKS(50));

    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        task.run_once();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(20));
    }
#else
    (void)params;
#endif
}
