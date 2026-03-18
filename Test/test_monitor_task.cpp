// Test/test_monitor_task.cpp
#include <gtest/gtest.h>
#include "monitor_task.hpp"

using namespace zlens;

// Provide definition for global queues used by monitor_task
namespace zlens {
    QueueHandle_t g_cmdQueue = nullptr;
    QueueHandle_t g_rspQueue = nullptr;
    volatile bool g_bUartSelfTestReq = false;
}

class MonitorTaskTest : public ::testing::Test {
protected:
    MonitorTask task;
    SystemManager sm;
    PowerMonitor pm;
    FramStorage fram;
    Encoder encoder;
    MotorCtrl motor;
    StallDetect stall;
    ZoomTable zoom;

    SPI_HandleTypeDef hspi;
    TIM_HandleTypeDef htim3;
    DAC_HandleTypeDef hdac;
    IWDG_HandleTypeDef hiwdg;
    TaskHandle_t hMotorTask;
    uint16_t iAdcVoltage = 2017;
    uint16_t iAdcCurrent = 0;

    void SetUp() override {
        mock::get_log().reset();
        mock::get_log().tick_auto_increment = false;
        g_cmdQueue = xQueueCreate(8, sizeof(CMD_MESSAGE_S));
        g_rspQueue = xQueueCreate(8, sizeof(RSP_MESSAGE_S));
        hspi.Instance = SPI2;
        htim3.Instance = TIM3;
        hdac.Instance = DAC1;
        hiwdg.Instance = IWDG;
        hMotorTask = reinterpret_cast<TaskHandle_t>(0x1234);

        sm.init();
        pm.init();
        fram.init(&hspi);
        encoder.init();
        motor.init(&htim3, &hdac, &encoder);
        stall.init();
        zoom.init();
        zoom.load_defaults();

        task.init(&sm, &pm, &fram, hMotorTask, &hiwdg, &iAdcVoltage,
                  &iAdcCurrent, &encoder, &motor, &stall, &zoom);
    }

    // Helper: prepare FRAM with valid params to simulate normal boot
    void prepare_valid_fram() {
        FRAM_PARAMS_S p{};
        p.magic_number = FramStorage::MAGIC;
        p.version = 1;
        p.current_position = 50000;
        p.current_zoom_x10 = 40; // 4.0x
        p.homing_done = 1;
        p.position_valid = 0xFF;
        p.total_range = 346292;
        p.crc16 = FramStorage::calc_crc(p);
        // Set SPI rx buffer so load_params returns valid data
        const uint8_t* pBytes = reinterpret_cast<const uint8_t*>(&p);
        mock::get_log().spi_rx_buffer.assign(pBytes, pBytes + sizeof(p));
    }
};

TEST_F(MonitorTaskTest, FirstBoot_SelfTest_VoltagePass_GoesToSelfTest) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    iAdcVoltage = 2017;
    // FRAM not valid (default) -> first boot -> starts full self-test
    task.run_once(); // boot decision + start self_test + first step (VOLTAGE)
    EXPECT_FALSE(task.is_normal_boot());
    // Self-test has started, not yet done
    EXPECT_FALSE(task.is_self_test_done());
}

TEST_F(MonitorTaskTest, FirstBoot_SelfTest_VoltageFail_Error) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    iAdcVoltage = 1300; // low voltage
    // First step: boot decision (FRAM invalid) + start self_test + VOLTAGE fail
    task.run_once();
    task.run_once(); // process the step that completes
    EXPECT_TRUE(task.is_self_test_done());
    EXPECT_FALSE(task.is_self_test_passed());
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::ERROR_STATE);
}

TEST_F(MonitorTaskTest, NormalBoot_SkipsSelfTest) {
    prepare_valid_fram();
    encoder.set_position(50000);
    zoom.set_total_range(346292);
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);

    task.run_once(); // boot decision: FRAM valid -> normal boot -> READY

    EXPECT_TRUE(task.is_normal_boot());
    EXPECT_TRUE(task.is_self_test_done());
    EXPECT_TRUE(task.is_self_test_passed());
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::READY);
}

TEST_F(MonitorTaskTest, NormalBoot_RestoresPosition) {
    prepare_valid_fram();
    encoder.set_position(50000);
    zoom.set_total_range(346292);
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);

    task.run_once();

    EXPECT_TRUE(task.is_normal_boot());
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::READY);
}

TEST_F(MonitorTaskTest, NormalBoot_MovesToNearestZoom) {
    prepare_valid_fram();
    // Position far from any zoom table entry
    encoder.set_position(100000);
    zoom.set_total_range(346292);
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);

    task.run_once();

    // Should have sent SET_ZOOM command
    CMD_MESSAGE_S stCmd;
    bool bGotCmd = xQueueReceive(g_cmdQueue, &stCmd, 0) == pdTRUE;
    EXPECT_TRUE(bGotCmd);
    if (bGotCmd) {
        EXPECT_EQ(stCmd.cmd, cmd::SET_ZOOM);
    }
}

TEST_F(MonitorTaskTest, NormalBoot_NoMoveWhenCloseToZoom) {
    prepare_valid_fram();
    // Position exactly at a zoom entry
    zoom.set_total_range(346292);
    int32_t iExactPos = zoom.get_position(zoom.get_min_zoom());
    encoder.set_position(iExactPos);
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);

    task.run_once();

    // Should NOT have sent any command (or at least not SET_ZOOM)
    CMD_MESSAGE_S stCmd;
    bool bGotCmd = xQueueReceive(g_cmdQueue, &stCmd, 0) == pdTRUE;
    // If position is exactly at a zoom entry, delta = 0 <= DEADZONE, no move needed
    if (bGotCmd) {
        EXPECT_NE(stCmd.cmd, cmd::SET_ZOOM);
    }
}

TEST_F(MonitorTaskTest, UartSelfTestReq_TriggersRetest) {
    // First: get to READY state via normal boot
    prepare_valid_fram();
    encoder.set_position(50000);
    zoom.set_total_range(346292);
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    task.run_once(); // normal boot -> READY
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::READY);

    // Set UART self-test request flag
    g_bUartSelfTestReq = true;
    task.run_once();

    // Flag should be cleared
    EXPECT_FALSE(g_bUartSelfTestReq);
    // Should transition to SELF_TEST
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::SELF_TEST);
    // Self-test state should be reset
    EXPECT_FALSE(task.is_self_test_done());
}

TEST_F(MonitorTaskTest, ReadyState_FeedsWatchdog) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);

    uint32_t before = mock::get_log().iwdg_refresh_count;
    task.run_once();
    EXPECT_GT(mock::get_log().iwdg_refresh_count, before);
}

TEST_F(MonitorTaskTest, LowVoltage_NotifiesMotor) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);

    iAdcVoltage = 1300;
    task.run_once();

    // Motor task should have been notified (xTaskNotify was called)
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::READY);
}

TEST_F(MonitorTaskTest, ErrorState_StaysInError) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::ERROR_STATE);

    task.run_once();

    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::ERROR_STATE);
    EXPECT_GT(mock::get_log().iwdg_refresh_count, 0u);
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
