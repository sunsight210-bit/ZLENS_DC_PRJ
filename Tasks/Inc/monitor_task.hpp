// Tasks/Inc/monitor_task.hpp
#pragma once
#include "task_config.hpp"
#include "system_manager.hpp"
#include "power_monitor.hpp"
#include "fram_storage.hpp"
#include "self_test.hpp"
#include "zoom_table.hpp"
#include "encoder.hpp"
#include "motor_ctrl.hpp"
#include "stall_detect.hpp"

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
              IWDG_HandleTypeDef* pHiwdg, uint16_t* pAdcVoltage,
              uint16_t* pAdcCurrent, Encoder* pEncoder,
              MotorCtrl* pMotor, StallDetect* pStall, ZoomTable* pZoom);

    void run_once();

    bool is_self_test_done() const { return m_bSelfTestDone; }
    bool is_self_test_passed() const { return m_bSelfTestPassed; }
    bool is_normal_boot() const { return m_bNormalBoot; }

    SelfTest& get_self_test() { return m_SelfTest; }

private:
    SystemManager* m_pSm = nullptr;
    PowerMonitor* m_pPm = nullptr;
    FramStorage* m_pFram = nullptr;
    TaskHandle_t m_hMotorTask = nullptr;
    IWDG_HandleTypeDef* m_pHiwdg = nullptr;
    uint16_t* m_pAdcVoltage = nullptr;
    ZoomTable* m_pZoom = nullptr;
    Encoder* m_pEncoder = nullptr;

    SelfTest m_SelfTest;
    bool m_bSelfTestDone = false;
    bool m_bSelfTestPassed = false;
    bool m_bBootDecided = false;
    bool m_bNormalBoot = false;
    bool m_bNormalBootStarted = false;
    bool m_bSelfTestActive = false;
    bool m_bNeedHomingOnly = false;
    bool m_bHomingTriggered = false;

    void feed_watchdog();
    void check_voltage();
    void start_normal_boot();
    void run_normal_boot();
};

} // namespace zlens

extern "C" void monitor_task_entry(void* params);
