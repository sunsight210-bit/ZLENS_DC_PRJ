// Test/test_comm_task.cpp
#include <gtest/gtest.h>
#include "comm_task.hpp"
#include "crc16.hpp"

namespace zlens { volatile bool g_bUartSelfTestReq = false; }

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
};

TEST_F(CommTaskTest, ParseValidFrame_SendsCmd) {
    send_work_frame(cmd::SET_ZOOM, 60);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, cmd::SET_ZOOM);
    EXPECT_EQ(msg.param, 60);
}

TEST_F(CommTaskTest, ParseInvalidCRC_NoAction) {
    uint8_t frame[] = {0xA5, cmd::SET_ZOOM, 0x00, 0x3C, 0x00, 0x00}; // bad CRC
    task.on_frame_received(frame, 6);

    CMD_MESSAGE_S msg;
    EXPECT_FALSE(receive_cmd(msg));
    EXPECT_EQ(uart_tx_count(), 0u);
}

TEST_F(CommTaskTest, QueryZoom_DirectResponse) {
    task.set_current_zoom(100); // 10.0x
    send_work_frame(cmd::QUERY_ZOOM, 0);

    // Query should be handled directly (UART response), not forwarded
    CMD_MESSAGE_S msg;
    EXPECT_FALSE(receive_cmd(msg)); // not forwarded to cmdQueue

    ASSERT_GE(uart_tx_count(), 1u);
    // Parse the response frame
    auto& tx = mock::get_log().uart_tx_data.back();
    auto result = comm.parse_work_frame(tx.data(), tx.size());
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, cmd::QUERY_ZOOM);
    EXPECT_EQ(result.param, 100);
}

TEST_F(CommTaskTest, QueryStatus_DirectResponse) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);

    send_work_frame(cmd::QUERY_STATUS, 0);

    ASSERT_GE(uart_tx_count(), 1u);
    auto& tx = mock::get_log().uart_tx_data.back();
    auto result = comm.parse_work_frame(tx.data(), tx.size());
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, cmd::QUERY_STATUS);
    EXPECT_EQ(result.param, 0x0000); // READY = stopped
}

TEST_F(CommTaskTest, QueryLensType_Returns0x0004) {
    send_work_frame(cmd::QUERY_TYPE, 0);

    ASSERT_GE(uart_tx_count(), 1u);
    auto& tx = mock::get_log().uart_tx_data.back();
    auto result = comm.parse_work_frame(tx.data(), tx.size());
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, cmd::QUERY_TYPE);
    EXPECT_EQ(result.param, rsp::LENS_TYPE);
}

TEST_F(CommTaskTest, QueryZoomRange_TwoFrames) {
    send_work_frame(cmd::QUERY_RANGE, 0);

    // Should send two frames: min zoom then max zoom
    ASSERT_GE(uart_tx_count(), 2u);

    auto& tx1 = mock::get_log().uart_tx_data[uart_tx_count() - 2];
    auto result1 = comm.parse_work_frame(tx1.data(), tx1.size());
    EXPECT_TRUE(result1.valid);
    EXPECT_EQ(result1.cmd, cmd::QUERY_RANGE);
    EXPECT_EQ(result1.param, zoom.get_min_zoom());

    auto& tx2 = mock::get_log().uart_tx_data[uart_tx_count() - 1];
    auto result2 = comm.parse_work_frame(tx2.data(), tx2.size());
    EXPECT_TRUE(result2.valid);
    EXPECT_EQ(result2.cmd, cmd::QUERY_RANGE);
    EXPECT_EQ(result2.param, zoom.get_max_zoom());
}

TEST_F(CommTaskTest, BusyReject_WhenBusy) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);
    sm.transition_to(SYSTEM_STATE_E::BUSY);

    send_work_frame(cmd::SET_ZOOM, 60);

    // Should NOT be forwarded
    CMD_MESSAGE_S msg;
    EXPECT_FALSE(receive_cmd(msg));

    // Should send BUSY response
    ASSERT_GE(uart_tx_count(), 1u);
    auto& tx = mock::get_log().uart_tx_data.back();
    auto result = comm.parse_work_frame(tx.data(), tx.size());
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.param, rsp::BUSY);
}

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

TEST_F(CommTaskTest, SwitchFactory_ToggleMode) {
    EXPECT_FALSE(comm.is_factory_mode());

    send_work_frame(cmd::SWITCH_FACTORY, 0);
    EXPECT_TRUE(comm.is_factory_mode());

    ASSERT_GE(uart_tx_count(), 1u);
    auto& tx = mock::get_log().uart_tx_data.back();
    auto result = comm.parse_work_frame(tx.data(), tx.size());
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.param, 0x0001); // factory mode ON
}

TEST_F(CommTaskTest, FactorySetAngle_Forwards) {
    comm.set_factory_mode(true);

    send_factory_frame(fcmd::SET_ANGLE, 0x001E, 0x510E);

    CMD_MESSAGE_S msg;
    EXPECT_TRUE(receive_cmd(msg));
    EXPECT_EQ(msg.cmd, fcmd::SET_ANGLE);
    EXPECT_EQ(msg.param, 0x510E); // param_low = angle_x100
}

TEST_F(CommTaskTest, SelfTest_SetsFlag_SendsAck) {
    g_bUartSelfTestReq = false;
    send_work_frame(cmd::SELF_TEST, 0);

    // Flag should be set
    EXPECT_TRUE(g_bUartSelfTestReq);

    // Should NOT be forwarded to cmdQueue
    CMD_MESSAGE_S msg;
    EXPECT_FALSE(receive_cmd(msg));

    // Should send ACK response
    ASSERT_GE(uart_tx_count(), 1u);
    auto& tx = mock::get_log().uart_tx_data.back();
    auto result = comm.parse_work_frame(tx.data(), tx.size());
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, cmd::SELF_TEST);
    EXPECT_EQ(result.param, rsp::OK);
}

TEST_F(CommTaskTest, CheckRspQueue_SendsUart) {
    RSP_MESSAGE_S rsp = {cmd::SET_ZOOM, 100};
    xQueueSend(rspQ, &rsp, 0);

    task.run_once();

    ASSERT_GE(uart_tx_count(), 1u);
    auto& tx = mock::get_log().uart_tx_data.back();
    auto result = comm.parse_work_frame(tx.data(), tx.size());
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, cmd::SET_ZOOM);
    EXPECT_EQ(result.param, 100);
}
