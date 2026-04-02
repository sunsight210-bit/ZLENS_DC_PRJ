// Test/test_comm_task.cpp
#include <gtest/gtest.h>
#include "comm_task.hpp"
#include "crc16.hpp"

namespace zlens { }

using namespace zlens;

class CommTaskTest : public ::testing::Test {
protected:
    CommTask task;
    CommProtocol comm;
    SystemManager sm;
    ZoomTable zoom;
    UART_HandleTypeDef huart;
    QueueHandle_t cmdQ, rspQ;

    void SetUp() override {
        mock::get_log().reset();
        comm.init();
        sm.init();
        zoom.init();
        zoom.load_defaults();
        huart = {};

        cmdQ = xQueueCreate(8, sizeof(CMD_MESSAGE_S));
        rspQ = xQueueCreate(8, sizeof(RSP_MESSAGE_S));

        task.init(&comm, &sm, &zoom, cmdQ, rspQ, &huart);
    }

    void send_work_frame(uint8_t cmd, uint16_t param) {
        uint8_t frame[6];
        comm.build_work_frame(cmd, param, frame);
        task.on_frame_received(frame, 6);
    }

    void send_factory_frame(uint8_t cmd, uint16_t paramH, uint16_t paramL) {
        uint8_t frame[8];
        comm.build_factory_frame(cmd, paramH, paramL, frame);
        task.on_frame_received(frame, 8);
    }

    bool receive_cmd(CMD_MESSAGE_S& msg) {
        return xQueueReceive(cmdQ, &msg, 0) == pdTRUE;
    }

    size_t uart_tx_count() const {
        return mock::get_log().uart_tx_data.size();
    }

    // Parse the Nth UART TX frame (0-indexed)
    WorkFrameResult parse_tx_frame(size_t idx) {
        auto& tx = mock::get_log().uart_tx_data[idx];
        return comm.parse_work_frame(tx.data(), tx.size());
    }

    // Parse the last UART TX frame
    WorkFrameResult parse_last_tx() {
        return parse_tx_frame(uart_tx_count() - 1);
    }
};

// ============================================================
// Handshake: all valid frames echo back raw bytes first
// ============================================================

TEST_F(CommTaskTest, Handshake_QueryEchoesRawFrame) {
    task.set_current_zoom(100);
    send_work_frame(cmd::QUERY_ZOOM, 0);

    // Frame 0: handshake (raw echo), Frame 1: response
    ASSERT_GE(uart_tx_count(), 2u);

    // Handshake frame should be identical to sent frame
    auto& echo = mock::get_log().uart_tx_data[0];
    uint8_t expected[6];
    comm.build_work_frame(cmd::QUERY_ZOOM, 0, expected);
    EXPECT_EQ(echo.size(), 6u);
    for (size_t i = 0; i < 6; i++) {
        EXPECT_EQ(echo[i], expected[i]) << "byte " << i;
    }
}

TEST_F(CommTaskTest, Handshake_MotionCommandEchoes) {
    send_work_frame(cmd::SET_ZOOM, 60);

    // First TX should be handshake echo
    ASSERT_GE(uart_tx_count(), 1u);
    auto& echo = mock::get_log().uart_tx_data[0];
    uint8_t expected[6];
    comm.build_work_frame(cmd::SET_ZOOM, 60, expected);
    EXPECT_EQ(echo.size(), 6u);
    for (size_t i = 0; i < 6; i++) {
        EXPECT_EQ(echo[i], expected[i]);
    }
}

TEST_F(CommTaskTest, Handshake_FactoryModeEchoes) {
    comm.set_factory_mode(true);
    send_factory_frame(fcmd::SET_ENTRY, 0x001E, 0x510E);

    ASSERT_GE(uart_tx_count(), 1u);
    auto& echo = mock::get_log().uart_tx_data[0];
    uint8_t expected[8];
    comm.build_factory_frame(fcmd::SET_ENTRY, 0x001E, 0x510E, expected);
    EXPECT_EQ(echo.size(), 8u);
    for (size_t i = 0; i < 8; i++) {
        EXPECT_EQ(echo[i], expected[i]);
    }
}

// ============================================================
// Invalid frame: no echo, no action
// ============================================================

TEST_F(CommTaskTest, ParseInvalidCRC_NoAction) {
    uint8_t frame[] = {0xA5, cmd::SET_ZOOM, 0x00, 0x3C, 0x00, 0x00}; // bad CRC
    task.on_frame_received(frame, 6);

    CMD_MESSAGE_S msg;
    EXPECT_FALSE(receive_cmd(msg));
    EXPECT_EQ(uart_tx_count(), 0u);
}

// ============================================================
// Query commands: response uses 0x1X command bytes
// ============================================================

TEST_F(CommTaskTest, QueryZoom_Response0x10) {
    task.set_current_zoom(100); // 10.0x
    send_work_frame(cmd::QUERY_ZOOM, 0);

    CMD_MESSAGE_S msg;
    EXPECT_FALSE(receive_cmd(msg)); // not forwarded

    // Frame 0: echo, Frame 1: response
    ASSERT_GE(uart_tx_count(), 2u);
    auto result = parse_tx_frame(1);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, rsp_cmd::ZOOM);
    EXPECT_EQ(result.param, 100);
}

TEST_F(CommTaskTest, QueryStatus_Response0x11) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);

    send_work_frame(cmd::QUERY_STATUS, 0);

    ASSERT_GE(uart_tx_count(), 2u);
    auto result = parse_tx_frame(1);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, rsp_cmd::STATUS);
    EXPECT_EQ(result.param, 0x0000); // READY = stopped
}

TEST_F(CommTaskTest, QuerySpeed_Response0x12) {
    send_work_frame(cmd::QUERY_SPEED, 0);

    ASSERT_GE(uart_tx_count(), 2u);
    auto result = parse_tx_frame(1);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, rsp_cmd::SPEED);
    EXPECT_EQ(result.param, rsp::DEFAULT_SPEED_DUTY);
}

TEST_F(CommTaskTest, QueryType_Response0x13) {
    send_work_frame(cmd::QUERY_TYPE, 0);

    ASSERT_GE(uart_tx_count(), 2u);
    auto result = parse_tx_frame(1);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, rsp_cmd::TYPE);
    EXPECT_EQ(result.param, rsp::LENS_TYPE);
}

TEST_F(CommTaskTest, QueryRange_TwoFrames0x14_0x15) {
    send_work_frame(cmd::QUERY_RANGE, 0);

    // echo + min + max = 3 frames
    ASSERT_GE(uart_tx_count(), 3u);

    auto r1 = parse_tx_frame(1);
    EXPECT_TRUE(r1.valid);
    EXPECT_EQ(r1.cmd, rsp_cmd::MIN_ZOOM);
    EXPECT_EQ(r1.param, zoom.get_min_zoom());

    auto r2 = parse_tx_frame(2);
    EXPECT_TRUE(r2.valid);
    EXPECT_EQ(r2.cmd, rsp_cmd::MAX_ZOOM);
    EXPECT_EQ(r2.param, zoom.get_max_zoom());
}

TEST_F(CommTaskTest, QueryVersion_Response0x16) {
    send_work_frame(cmd::QUERY_VERSION, 0);

    ASSERT_GE(uart_tx_count(), 2u);
    auto result = parse_tx_frame(1);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, rsp_cmd::VERSION);
    EXPECT_EQ(result.param, rsp::FW_VERSION);
}

// ============================================================
// GET_STALL_CNT: direct response
// ============================================================

TEST_F(CommTaskTest, GetStallCount_Response0xE3) {
    task.set_stall_count(5);
    send_work_frame(cmd::GET_STALL_CNT, 0);

    CMD_MESSAGE_S msg;
    EXPECT_FALSE(receive_cmd(msg)); // not forwarded

    ASSERT_GE(uart_tx_count(), 2u); // echo + response
    auto result = parse_tx_frame(1);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, rsp_cmd::STALL_COUNT);
    EXPECT_EQ(result.param, 5);
}

// ============================================================
// BUSY reject: 0x03 + 0x000E
// ============================================================

TEST_F(CommTaskTest, BusyReject_Response0x03_0x000E) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);
    sm.transition_to(SYSTEM_STATE_E::BUSY);

    send_work_frame(cmd::SET_ZOOM, 60);

    CMD_MESSAGE_S msg;
    EXPECT_FALSE(receive_cmd(msg)); // not forwarded

    // echo + busy response
    ASSERT_GE(uart_tx_count(), 2u);
    auto result = parse_tx_frame(1);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, rsp_cmd::REQ_INVALID);
    EXPECT_EQ(result.param, rsp::REQ_INVALID_PARAM);
}

// ============================================================
// Motion commands forwarded to MotorTask
// ============================================================

TEST_F(CommTaskTest, SetZoom_ForwardsToMotor) {
    send_work_frame(cmd::SET_ZOOM, 100);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::SET_ZOOM);
    EXPECT_EQ(msg.param, 100);
}

TEST_F(CommTaskTest, Homing_ForwardsToMotor) {
    send_work_frame(cmd::HOMING, 0);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::HOMING);
}

TEST_F(CommTaskTest, ZoomInc_ForwardsToMotor) {
    send_work_frame(cmd::ZOOM_INC, 10);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::ZOOM_INC);
    EXPECT_EQ(msg.param, 10);
}

TEST_F(CommTaskTest, ZoomDec_ForwardsToMotor) {
    send_work_frame(cmd::ZOOM_DEC, 10);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::ZOOM_DEC);
    EXPECT_EQ(msg.param, 10);
}

TEST_F(CommTaskTest, CycleStart_ForwardsToMotor) {
    send_work_frame(cmd::CYCLE_START, 0x0105);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::CYCLE_START);
    EXPECT_EQ(msg.param, 0x0105);
}

TEST_F(CommTaskTest, CycleStop_ForwardsToMotor) {
    send_work_frame(cmd::CYCLE_STOP, 0);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::CYCLE_STOP);
}

// ============================================================
// is_motion_command: ZOOM_INC/DEC are motion commands
// ============================================================

TEST_F(CommTaskTest, ZoomInc_BusyReject) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);
    sm.transition_to(SYSTEM_STATE_E::BUSY);

    send_work_frame(cmd::ZOOM_INC, 10);

    CMD_MESSAGE_S msg;
    EXPECT_FALSE(receive_cmd(msg));

    ASSERT_GE(uart_tx_count(), 2u);
    auto result = parse_tx_frame(1);
    EXPECT_EQ(result.cmd, rsp_cmd::REQ_INVALID);
    EXPECT_EQ(result.param, rsp::REQ_INVALID_PARAM);
}

// ============================================================
// SwitchFactory at 0xFA
// ============================================================

TEST_F(CommTaskTest, SwitchFactory_EnterWithMagic) {
    EXPECT_FALSE(comm.is_factory_mode());

    send_work_frame(cmd::SWITCH_FACTORY, factory::MAGIC_PARAM);
    EXPECT_TRUE(comm.is_factory_mode());

    // echo + response
    ASSERT_GE(uart_tx_count(), 2u);
    auto result = parse_tx_frame(1);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.param, 0x0001); // factory mode ON
}

TEST_F(CommTaskTest, SwitchFactory_RejectWithoutMagic) {
    EXPECT_FALSE(comm.is_factory_mode());

    send_work_frame(cmd::SWITCH_FACTORY, 0x0000);
    EXPECT_FALSE(comm.is_factory_mode()); // should NOT enter factory mode
}

// ============================================================
// Factory command forwarding
// ============================================================

TEST_F(CommTaskTest, FactorySetEntry_UpdatesZoomTable) {
    comm.set_factory_mode(true);

    send_factory_frame(fcmd::SET_ENTRY, 0x001E, 0x510E);

    // SET_ENTRY should update ZoomTable directly, not forward to MotorTask
    CMD_MESSAGE_S msg;
    EXPECT_FALSE(receive_cmd(msg));
    EXPECT_TRUE(zoom.is_valid_zoom(0x001E));
}

// ============================================================
// Self-test
// ============================================================

TEST_F(CommTaskTest, SelfTest0x65_ForwardsToMotor) {
    send_work_frame(cmd::SELF_TEST, 0);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::SELF_TEST);
}

// ============================================================
// run_once: response queue forwarding
// ============================================================

TEST_F(CommTaskTest, RunOnce_ForwardsZoomResponse) {
    RSP_MESSAGE_S rsp_msg = {rsp_cmd::ZOOM, 100};
    xQueueSend(rspQ, &rsp_msg, 0);

    task.run_once();

    ASSERT_GE(uart_tx_count(), 1u);
    auto result = parse_last_tx();
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, rsp_cmd::ZOOM);
    EXPECT_EQ(result.param, 100);
}

TEST_F(CommTaskTest, RunOnce_ZoomResponseUpdatesCurrentZoom) {
    RSP_MESSAGE_S rsp_msg = {rsp_cmd::ZOOM, 45};
    xQueueSend(rspQ, &rsp_msg, 0);

    task.run_once();

    EXPECT_EQ(task.get_current_zoom(), 45);
}

TEST_F(CommTaskTest, RunOnce_ArrivedResponseNoZoomUpdate) {
    task.set_current_zoom(100);
    RSP_MESSAGE_S rsp_msg = {rsp_cmd::ARRIVED, rsp::ARRIVED_PARAM};
    xQueueSend(rspQ, &rsp_msg, 0);

    task.run_once();

    // current_zoom should NOT change for ARRIVED frame
    EXPECT_EQ(task.get_current_zoom(), 100);
}

// ============================================================
// Speed accessor
// ============================================================

TEST_F(CommTaskTest, SpeedAccessor_DefaultAndSet) {
    EXPECT_EQ(task.get_speed_duty(), rsp::DEFAULT_SPEED_DUTY);
    task.set_speed_duty(200);
    EXPECT_EQ(task.get_speed_duty(), 200);
}

// ============================================================
// Stall count accessor
// ============================================================

// ============================================================
// 0x60 Speed commands: forwarded, not blocked by busy
// ============================================================

TEST_F(CommTaskTest, SetSpeed_ForwardsToMotor) {
    send_work_frame(cmd::SET_SPEED, 500);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::SET_SPEED);
    EXPECT_EQ(msg.param, 500);
}

TEST_F(CommTaskTest, SpeedInc_ForwardsToMotor) {
    send_work_frame(cmd::SPEED_INC, 0);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::SPEED_INC);
}

TEST_F(CommTaskTest, SpeedDec_ForwardsToMotor) {
    send_work_frame(cmd::SPEED_DEC, 0);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::SPEED_DEC);
}

TEST_F(CommTaskTest, SetMinSpeed_ForwardsToMotor) {
    send_work_frame(cmd::SET_MIN_SPEED, 100);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::SET_MIN_SPEED);
    EXPECT_EQ(msg.param, 100);
}

TEST_F(CommTaskTest, SetMaxSpeed_ForwardsToMotor) {
    send_work_frame(cmd::SET_MAX_SPEED, 800);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::SET_MAX_SPEED);
    EXPECT_EQ(msg.param, 800);
}

TEST_F(CommTaskTest, SpeedCommands_NotBlockedByBusy) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);
    sm.transition_to(SYSTEM_STATE_E::BUSY);

    send_work_frame(cmd::SET_SPEED, 300);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::SET_SPEED);
}

TEST_F(CommTaskTest, RunOnce_SpeedResponseUpdatesCache) {
    RSP_MESSAGE_S rsp_msg = {rsp_cmd::SPEED, 500};
    xQueueSend(rspQ, &rsp_msg, 0);

    task.run_once();

    EXPECT_EQ(task.get_speed_duty(), 500);
}

// ============================================================
// Stall count accessor
// ============================================================

TEST_F(CommTaskTest, StallCountAccessor_DefaultAndSet) {
    EXPECT_EQ(task.get_stall_count(), 0);
    task.set_stall_count(3);
    EXPECT_EQ(task.get_stall_count(), 3);
}

// ============================================================
// Factory mode: full calibration flow
// ============================================================

TEST_F(CommTaskTest, FactoryEraseAll_ClearsZoomTable) {
    // ZoomTable has defaults loaded
    EXPECT_GT(zoom.get_entry_count(), 0);

    comm.set_factory_mode(true);
    send_factory_frame(fcmd::ERASE_ALL, factory::MAGIC_HIGH, factory::MAGIC_LOW);

    EXPECT_EQ(zoom.get_entry_count(), 0);
}

TEST_F(CommTaskTest, FactoryEraseAll_RejectBadMagic) {
    comm.set_factory_mode(true);
    uint16_t iCountBefore = zoom.get_entry_count();

    send_factory_frame(fcmd::ERASE_ALL, 0x1234, 0x5678);

    EXPECT_EQ(zoom.get_entry_count(), iCountBefore); // unchanged
}

TEST_F(CommTaskTest, FactoryExitToWork_SavesAndSwitches) {
    comm.set_factory_mode(true);

    // Erase + set 2 entries
    send_factory_frame(fcmd::ERASE_ALL, factory::MAGIC_HIGH, factory::MAGIC_LOW);
    send_factory_frame(fcmd::SET_ENTRY, 10, 7200);   // 1.0x = 72.00°
    send_factory_frame(fcmd::SET_ENTRY, 20, 14800);  // 2.0x = 148.00°
    EXPECT_EQ(zoom.get_entry_count(), 2);

    // Exit factory mode
    send_factory_frame(fcmd::SWITCH_TO_WORK, factory::MAGIC_HIGH, factory::MAGIC_LOW);

    EXPECT_FALSE(comm.is_factory_mode());
    // save_to_flash was called (flash_memory should have data)
    EXPECT_FALSE(mock::get_log().flash_memory.empty());
}

TEST_F(CommTaskTest, FactoryExitToWork_RejectBadMagic) {
    comm.set_factory_mode(true);

    send_factory_frame(fcmd::SWITCH_TO_WORK, 0x0000, 0x0000);

    EXPECT_TRUE(comm.is_factory_mode()); // still in factory mode
}

TEST_F(CommTaskTest, FactoryFullCalibrationFlow) {
    // Step 1: Enter factory mode from work mode
    send_work_frame(cmd::SWITCH_FACTORY, factory::MAGIC_PARAM);
    EXPECT_TRUE(comm.is_factory_mode());

    // Step 2: Erase old table
    send_factory_frame(fcmd::ERASE_ALL, factory::MAGIC_HIGH, factory::MAGIC_LOW);
    EXPECT_EQ(zoom.get_entry_count(), 0);

    // Step 3: Set 5 zoom entries
    send_factory_frame(fcmd::SET_ENTRY, 6,     0);      // 0.6x
    send_factory_frame(fcmd::SET_ENTRY, 10,  7200);     // 1.0x
    send_factory_frame(fcmd::SET_ENTRY, 20, 14800);     // 2.0x
    send_factory_frame(fcmd::SET_ENTRY, 40, 25750);     // 4.0x
    send_factory_frame(fcmd::SET_ENTRY, 70, 34700);     // 7.0x
    EXPECT_EQ(zoom.get_entry_count(), 5);

    // Step 4: Verify entries are valid and positions correct
    EXPECT_TRUE(zoom.is_valid_zoom(6));
    EXPECT_TRUE(zoom.is_valid_zoom(20));
    EXPECT_TRUE(zoom.is_valid_zoom(70));
    EXPECT_FALSE(zoom.is_valid_zoom(30));  // not in new table

    // Step 5: Exit factory mode (saves to flash)
    send_factory_frame(fcmd::SWITCH_TO_WORK, factory::MAGIC_HIGH, factory::MAGIC_LOW);
    EXPECT_FALSE(comm.is_factory_mode());

    // Step 6: Verify saved — position for 2.0x should be 128 + 14800*65536/36000 = 27075
    int32_t iPos = zoom.get_position(20);
    EXPECT_EQ(iPos, 128 + static_cast<int32_t>(
        static_cast<int64_t>(14800) * 65536 / 36000));
}

TEST_F(CommTaskTest, FactorySetEntry_OverwriteExisting) {
    comm.set_factory_mode(true);
    send_factory_frame(fcmd::ERASE_ALL, factory::MAGIC_HIGH, factory::MAGIC_LOW);

    send_factory_frame(fcmd::SET_ENTRY, 20, 14800);
    send_factory_frame(fcmd::SET_ENTRY, 20, 15000);  // overwrite same zoom

    EXPECT_EQ(zoom.get_entry_count(), 1);  // still 1 entry, not 2
    // Position should reflect the updated angle
    int32_t iPos = zoom.get_position(20);
    EXPECT_EQ(iPos, 128 + static_cast<int32_t>(
        static_cast<int64_t>(15000) * 65536 / 36000));
}
