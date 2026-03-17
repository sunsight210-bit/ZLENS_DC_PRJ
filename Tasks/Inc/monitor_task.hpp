// Tasks/Inc/monitor_task.hpp
#pragma once
#include "task_config.hpp"
#include "system_manager.hpp"
#include "power_monitor.hpp"
#include "fram_storage.hpp"

#ifdef BUILD_TESTING
#include "mock_hal.hpp"
#else
#include "stm32f1xx_hal.h"
#endif

namespace zlens {

class MonitorTask {
public:
    void init(SystemManager* pSm, PowerMonitor* pPm,
              FramStorage* pFram, TaskHandle_t hMotorTask,
              IWDG_HandleTypeDef* pHiwdg, uint16_t* pAdcVoltage);

    void run_once();
    bool run_self_test();

    bool is_self_test_done() const { return m_bSelfTestDone; }
    bool is_self_test_passed() const { return m_bSelfTestPassed; }

private:
    SystemManager* m_pSm = nullptr;
    PowerMonitor* m_pPm = nullptr;
    FramStorage* m_pFram = nullptr;
    TaskHandle_t m_hMotorTask = nullptr;
    IWDG_HandleTypeDef* m_pHiwdg = nullptr;
    uint16_t* m_pAdcVoltage = nullptr;

    bool m_bSelfTestDone = false;
    bool m_bSelfTestPassed = false;

    void feed_watchdog();
    void check_voltage();
};

} // namespace zlens

extern "C" void monitor_task_entry(void* params);
