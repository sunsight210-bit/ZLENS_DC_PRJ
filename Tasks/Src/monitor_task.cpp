// Tasks/Src/monitor_task.cpp
#include "monitor_task.hpp"
#ifndef BUILD_TESTING
#include "app_instances.hpp"
#include "swo_debug.hpp"
#endif

namespace zlens {

void MonitorTask::init(SystemManager* pSm, PowerMonitor* pPm,
                       FramStorage* pFram, TaskHandle_t hMotorTask,
                       IWDG_HandleTypeDef* pHiwdg, uint16_t* pAdcVoltage) {
    m_pSm = pSm;
    m_pPm = pPm;
    m_pFram = pFram;
    m_hMotorTask = hMotorTask;
    m_pHiwdg = pHiwdg;
    m_pAdcVoltage = pAdcVoltage;
    m_bSelfTestDone = false;
    m_bSelfTestPassed = false;
}

void MonitorTask::run_once() {
    SYSTEM_STATE_E eState = m_pSm->get_state();

    switch (eState) {
    case SYSTEM_STATE_E::SELF_TEST:
        if (!m_bSelfTestDone) {
            m_bSelfTestPassed = run_self_test();
            m_bSelfTestDone = true;
            if (m_bSelfTestPassed) {
#ifndef BUILD_TESTING
                swo_printf("[PASS] Self-test passed\n");
#endif
                m_pSm->transition_to(SYSTEM_STATE_E::HOMING);
                // Send HOMING command to motor task via queue
                CMD_MESSAGE_S stCmd = {cmd::HOMING, 0};
                xQueueSend(g_cmdQueue, &stCmd, 0);
            } else {
#ifndef BUILD_TESTING
                swo_printf("[FAIL] Self-test failed\n");
#endif
                m_pSm->transition_to(SYSTEM_STATE_E::ERROR_STATE);
            }
        }
        break;

    case SYSTEM_STATE_E::HOMING:
    case SYSTEM_STATE_E::READY:
    case SYSTEM_STATE_E::BUSY:
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

bool MonitorTask::run_self_test() {
    // Check FRAM connectivity
    if (!m_pFram->is_valid()) {
        // FRAM not readable - could be first boot, not fatal
    }

    // Check ADC baseline (voltage should be reasonable)
    if (m_pAdcVoltage) {
        uint16_t iAdcVal = *m_pAdcVoltage;
#ifndef BUILD_TESTING
        swo_printf("[INFO] Self-test ADC value: %u (threshold: %u)\n",
                   iAdcVal, PowerMonitor::POWER_DOWN_THRESHOLD);
#endif
        if (m_pPm->is_power_down(iAdcVal)) {
            return false; // Voltage too low
        }
    }

    return true;
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
        // Notify motor task of power-down (emergency stop)
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
              const_cast<uint16_t*>(&g_aAdcDmaBuf[1]));

    g_SystemManager.transition_to(SYSTEM_STATE_E::SELF_TEST);
    swo_printf("[PASS] FreeRTOS scheduler running\n");

    // Wait for ADC DMA to complete at least one conversion cycle
    vTaskDelay(pdMS_TO_TICKS(50));

    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        task.run_once();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(100));
    }
#else
    (void)params;
#endif
}
