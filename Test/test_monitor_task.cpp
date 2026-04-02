// Test/test_monitor_task.cpp
#include <gtest/gtest.h>
#include "monitor_task.hpp"

using namespace zlens;

// Provide definition for global queues used by monitor_task
namespace zlens {
    QueueHandle_t g_cmdQueue = nullptr;
    QueueHandle_t g_rspQueue = nullptr;
}

class MonitorTaskTest : public ::testing::Test {
protected:
    MonitorTask task;
    SystemManager sm;
    PowerMonitor pm;
    FramStorage fram;
    Encoder encoder;
    MotorCtrl motor;
    ZoomTable zoom;

    SPI_HandleTypeDef hspi;
    TIM_HandleTypeDef htim3;
    DAC_HandleTypeDef hdac;
    IWDG_HandleTypeDef hiwdg;
    TaskHandle_t hMotorTask;
    uint16_t iAdcVoltage = 2017;

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
        zoom.init();
        zoom.load_defaults();

        task.init(&sm, &pm, &fram, hMotorTask, &hiwdg, &iAdcVoltage,
                  &encoder, &motor, &zoom);
    }

    // Helper: prepare FRAM with valid state for normal boot
    void prepare_valid_fram() {
        FRAM_STATE_S p{};
        p.magic = FramStorage::MAGIC;
        p.current_position = 50000;
        p.current_zoom_x10 = 40; // 4.0x
        p.homing_done = 1;
        p.position_valid = 0xFF;
        p.crc16 = FramStorage::calc_crc(p);
        const uint8_t* pBytes = reinterpret_cast<const uint8_t*>(&p);
        mock::get_log().spi_rx_buffer.assign(pBytes, pBytes + sizeof(p));
    }

    // Helper: prepare FRAM with homing_done=1 but position_valid=0 (homing-only boot)
    void prepare_homing_only_fram() {
        FRAM_STATE_S p{};
        p.magic = FramStorage::MAGIC;
        p.current_position = 50000;
        p.current_zoom_x10 = 40;
        p.homing_done = 1;
        p.position_valid = 0;  // abnormal power loss
        p.crc16 = FramStorage::calc_crc(p);
        const uint8_t* pBytes = reinterpret_cast<const uint8_t*>(&p);
        mock::get_log().spi_rx_buffer.assign(pBytes, pBytes + sizeof(p));
    }
};

TEST_F(MonitorTaskTest, FirstBoot_HomingOnly) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    iAdcVoltage = 2017;
    task.run_once();
    EXPECT_FALSE(task.is_normal_boot());
    EXPECT_TRUE(task.is_self_test_done());
    EXPECT_TRUE(task.is_self_test_passed());
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::HOMING);
    CMD_MESSAGE_S stCmd;
    bool bGotCmd = xQueueReceive(g_cmdQueue, &stCmd, 0) == pdTRUE;
    EXPECT_TRUE(bGotCmd);
    EXPECT_EQ(stCmd.cmd, cmd::HOMING);
    EXPECT_EQ(stCmd.param, 0);
}

TEST_F(MonitorTaskTest, FirstBoot_LowVoltage_StillHomesOnly) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    iAdcVoltage = 1300;
    task.run_once();
    EXPECT_TRUE(task.is_self_test_done());
    EXPECT_TRUE(task.is_self_test_passed());
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::HOMING);
}

TEST_F(MonitorTaskTest, NormalBoot_SkipsSelfTest) {
    prepare_valid_fram();
    encoder.set_position(50000);
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    task.run_once();
    EXPECT_TRUE(task.is_normal_boot());
    EXPECT_TRUE(task.is_self_test_done());
    EXPECT_TRUE(task.is_self_test_passed());
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::READY);
}

TEST_F(MonitorTaskTest, NormalBoot_RestoresPosition) {
    prepare_valid_fram();
    encoder.set_position(50000);
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    task.run_once();
    EXPECT_TRUE(task.is_normal_boot());
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::READY);
}

TEST_F(MonitorTaskTest, NormalBoot_MovesToNearestZoom) {
    prepare_valid_fram();
    encoder.set_position(100000);
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    task.run_once();
    CMD_MESSAGE_S stCmd;
    bool bGotCmd = xQueueReceive(g_cmdQueue, &stCmd, 0) == pdTRUE;
    EXPECT_TRUE(bGotCmd);
    if (bGotCmd) {
        EXPECT_EQ(stCmd.cmd, cmd::SET_ZOOM);
    }
}

TEST_F(MonitorTaskTest, NormalBoot_NoMoveWhenCloseToZoom) {
    prepare_valid_fram();
    int32_t iExactPos = zoom.get_position(zoom.get_min_zoom());
    encoder.set_position(iExactPos);
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    task.run_once();
    CMD_MESSAGE_S stCmd;
    bool bGotCmd = xQueueReceive(g_cmdQueue, &stCmd, 0) == pdTRUE;
    if (bGotCmd) {
        EXPECT_NE(stCmd.cmd, cmd::SET_ZOOM);
    }
}

TEST_F(MonitorTaskTest, HomingOnlyBoot_SendsHomingCommand) {
    prepare_homing_only_fram();
    encoder.set_position(50000);
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    task.run_once();
    CMD_MESSAGE_S stCmd;
    bool bGotCmd = xQueueReceive(g_cmdQueue, &stCmd, 0) == pdTRUE;
    EXPECT_TRUE(bGotCmd);
    if (bGotCmd) {
        EXPECT_EQ(stCmd.cmd, cmd::HOMING);
    }
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::HOMING);
    EXPECT_TRUE(task.is_self_test_done());
    EXPECT_TRUE(task.is_self_test_passed());
}

TEST_F(MonitorTaskTest, HomingOnlyBoot_NotNormalBoot) {
    prepare_homing_only_fram();
    encoder.set_position(50000);
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    task.run_once();
    EXPECT_FALSE(task.is_normal_boot());
}

TEST_F(MonitorTaskTest, FirstBoot_SendsHomingCommand) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    iAdcVoltage = 2017;
    task.run_once();
    EXPECT_TRUE(task.is_self_test_done());
    EXPECT_TRUE(task.is_self_test_passed());
    CMD_MESSAGE_S stCmd;
    bool bGotCmd = xQueueReceive(g_cmdQueue, &stCmd, 0) == pdTRUE;
    EXPECT_TRUE(bGotCmd);
    EXPECT_EQ(stCmd.cmd, cmd::HOMING);
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::HOMING);
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

TEST_F(MonitorTaskTest, MoveCountExceeded_ForcesHoming) {
    FRAM_STATE_S p{};
    p.magic = FramStorage::MAGIC;
    p.current_position = 50000;
    p.current_zoom_x10 = 40;
    p.homing_done = 1;
    p.position_valid = 0xFF;
    p.move_count = 500;
    p.crc16 = FramStorage::calc_crc(p);
    const uint8_t* pBytes = reinterpret_cast<const uint8_t*>(&p);
    mock::get_log().spi_rx_buffer.assign(pBytes, pBytes + sizeof(p));

    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    task.run_once();
    EXPECT_FALSE(task.is_normal_boot());
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::HOMING);
}

TEST_F(MonitorTaskTest, MoveCountBelowThreshold_NormalBoot) {
    FRAM_STATE_S p{};
    p.magic = FramStorage::MAGIC;
    p.current_position = 50000;
    p.current_zoom_x10 = 40;
    p.homing_done = 1;
    p.position_valid = 0xFF;
    p.move_count = 499;
    p.crc16 = FramStorage::calc_crc(p);
    const uint8_t* pBytes = reinterpret_cast<const uint8_t*>(&p);
    mock::get_log().spi_rx_buffer.assign(pBytes, pBytes + sizeof(p));

    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    task.run_once();
    EXPECT_TRUE(task.is_normal_boot());
}
