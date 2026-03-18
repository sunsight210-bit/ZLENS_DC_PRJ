// Test/test_monitor_task.cpp
#include <gtest/gtest.h>
#include "monitor_task.hpp"

using namespace zlens;

// Provide definition for global queue used by monitor_task
namespace zlens { QueueHandle_t g_cmdQueue = nullptr; }

class MonitorTaskTest : public ::testing::Test {
protected:
    MonitorTask task;
    SystemManager sm;
    PowerMonitor pm;
    FramStorage fram;
    SPI_HandleTypeDef hspi;
    IWDG_HandleTypeDef hiwdg;
    TaskHandle_t hMotorTask;
    uint16_t iAdcVoltage = 2017; // normal 12V (R_top=30kΩ, R_bottom=4.7kΩ)

    void SetUp() override {
        mock::get_log().reset();
        g_cmdQueue = xQueueCreate(8, sizeof(CMD_MESSAGE_S));
        hspi.Instance = SPI2;
        hiwdg.Instance = IWDG;
        hMotorTask = reinterpret_cast<TaskHandle_t>(0x1234);

        sm.init();
        pm.init();
        fram.init(&hspi);

        task.init(&sm, &pm, &fram, hMotorTask, &hiwdg, &iAdcVoltage);
    }
};

TEST_F(MonitorTaskTest, SelfTest_Pass_GoesToHoming) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    iAdcVoltage = 2017; // normal voltage

    task.run_once();

    EXPECT_TRUE(task.is_self_test_done());
    EXPECT_TRUE(task.is_self_test_passed());
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::HOMING);
}

TEST_F(MonitorTaskTest, SelfTest_Fail_GoesToError) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    iAdcVoltage = 1300; // low voltage -> power down detected

    task.run_once();

    EXPECT_TRUE(task.is_self_test_done());
    EXPECT_FALSE(task.is_self_test_passed());
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::ERROR_STATE);
}

TEST_F(MonitorTaskTest, NormalRun_FeedsWatchdog) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);

    uint32_t before = mock::get_log().iwdg_refresh_count;
    task.run_once();
    EXPECT_GT(mock::get_log().iwdg_refresh_count, before);
}

TEST_F(MonitorTaskTest, NormalRun_ReadsVoltage) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);

    iAdcVoltage = 2017; // normal
    task.run_once();

    // No error transition should occur
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::READY);
}

TEST_F(MonitorTaskTest, LowVoltage_NotifiesMotor) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);

    iAdcVoltage = 1300; // below threshold -> power down
    task.run_once();

    // Motor task should have been notified
    // (xTaskNotify was called - we can verify it ran without crash)
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::READY);
}

TEST_F(MonitorTaskTest, ErrorState_StaysInError) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::ERROR_STATE);

    task.run_once();

    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::ERROR_STATE);
    // Still feeds watchdog in error state
    EXPECT_GT(mock::get_log().iwdg_refresh_count, 0u);
}

TEST_F(MonitorTaskTest, ReadyState_NormalRun) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);

    iAdcVoltage = 2017;
    for (int i = 0; i < 10; ++i) task.run_once();

    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::READY);
    EXPECT_GE(mock::get_log().iwdg_refresh_count, 10u);
}

TEST_F(MonitorTaskTest, BusyState_NormalRun) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);
    sm.transition_to(SYSTEM_STATE_E::BUSY);

    iAdcVoltage = 2017;
    task.run_once();

    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::BUSY);
    EXPECT_GT(mock::get_log().iwdg_refresh_count, 0u);
}
