// Test/test_motor_task.cpp
#include <gtest/gtest.h>
#include "motor_task.hpp"

using namespace zlens;

namespace zlens { volatile bool g_bSpiEmergency = false; }

class MotorTaskTest : public ::testing::Test {
protected:
    MotorTask task;
    MotorCtrl motor;
    Encoder encoder;
    StallDetect stall;
    ZoomTable zoom;
    FramStorage fram;
    SystemManager sm;

    TIM_HandleTypeDef htim3;
    DAC_HandleTypeDef hdac;
    SPI_HandleTypeDef hspi;
    QueueHandle_t cmdQ, rspQ, saveQ;
    uint16_t iAdcCurrent = 0;

    void SetUp() override {
        mock::get_log().reset();
        mock::get_log().tick_auto_increment = true;
        htim3.Instance = TIM3;
        hdac.Instance = DAC1;
        hspi.Instance = SPI2;

        encoder.init();
        motor.init(&htim3, &hdac, &encoder);
        stall.init();
        zoom.init();
        zoom.load_defaults();
        fram.init(&hspi);

        cmdQ = xQueueCreate(8, sizeof(CMD_MESSAGE_S));
        rspQ = xQueueCreate(8, sizeof(RSP_MESSAGE_S));
        saveQ = xQueueCreate(8, sizeof(SAVE_MESSAGE_S));

        sm.init();
        sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
        sm.transition_to(SYSTEM_STATE_E::READY);

        iAdcCurrent = 0;
        task.init(&motor, &encoder, &stall, &zoom, &fram, &sm, cmdQ, rspQ, saveQ, &iAdcCurrent);
    }

    void send_cmd(uint8_t cmd, uint16_t param = 0) {
        CMD_MESSAGE_S msg = {cmd, param};
        xQueueSend(cmdQ, &msg, 0);
    }

    bool receive_rsp(RSP_MESSAGE_S& rsp) {
        return xQueueReceive(rspQ, &rsp, 0) == pdTRUE;
    }

    bool receive_save(SAVE_MESSAGE_S& save) {
        return xQueueReceive(saveQ, &save, 0) == pdTRUE;
    }

    // Drain all responses from rspQ into a vector
    std::vector<RSP_MESSAGE_S> drain_rsp() {
        std::vector<RSP_MESSAGE_S> results;
        RSP_MESSAGE_S rsp;
        while (receive_rsp(rsp)) {
            results.push_back(rsp);
        }
        return results;
    }

    void trigger_stall(uint16_t adc_val = 250) {
        iAdcCurrent = adc_val;
        for (int i = 0; i < 64 + StallDetect::BLANKING_TICKS + StallDetect::STALL_CONFIRM_COUNT + 10; ++i) {
            task.run_once();
        }
    }

    void trigger_overcurrent() {
        iAdcCurrent = 1300;
        for (int i = 0; i < 64 + StallDetect::BLANKING_TICKS + StallDetect::OVERCURRENT_CONFIRM + 10; ++i) {
            task.run_once();
        }
    }
};

TEST_F(MotorTaskTest, Init_StateIsIdle) {
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    EXPECT_FALSE(task.is_homing_done());
}

// ============================================================
// SET_ZOOM (0x10) — unchanged cmd, new response format
// ============================================================

TEST_F(MotorTaskTest, SetZoom_StartsMotor) {

    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorTaskTest, SetZoom_Arrived_TwoFrames) {

    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();

    int32_t iTarget = motor.get_target();
    encoder.set_position(iTarget);
    for (int i = 0; i < 10; ++i) task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);

    // Should send two responses: ZOOM + ARRIVED
    auto rsps = drain_rsp();
    ASSERT_GE(rsps.size(), 2u);
    EXPECT_EQ(rsps[0].cmd, rsp_cmd::ZOOM);
    EXPECT_EQ(rsps[1].cmd, rsp_cmd::ARRIVED);
    EXPECT_EQ(rsps[1].param, rsp::ARRIVED_PARAM);

    SAVE_MESSAGE_S save;
    EXPECT_TRUE(receive_save(save));
    EXPECT_EQ(save.reason, save_reason::ARRIVED);
}

// ============================================================
// HOMING (0x01) — 4-stage fast/slow homing
// ============================================================

TEST_F(MotorTaskTest, Homing_CmdIs0x01) {
    send_cmd(cmd::HOMING, 0);
    task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_FAST);
}

TEST_F(MotorTaskTest, HomingFast_StartsReverse) {
    task.start_homing();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_FAST);
}

TEST_F(MotorTaskTest, HomingFast_StallTriggersRetract) {
    task.start_homing();
    trigger_stall();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_RETRACT);
    EXPECT_EQ(motor.get_target(), MotorTask::HOMING_RETRACT_DISTANCE);
}

TEST_F(MotorTaskTest, HomingRetract_DoneTriggersSlow) {
    task.start_homing();
    trigger_stall(); // FAST → RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_SLOW);
}

TEST_F(MotorTaskTest, HomingSlow_StallTriggersSettle) {
    task.start_homing();
    trigger_stall(); // FAST → RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();
    // Now in SLOW
    trigger_stall(); // SLOW → SETTLE
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_SETTLE);
    EXPECT_EQ(motor.get_target(), MotorTask::HOMING_SETTLE_DISTANCE);
}

TEST_F(MotorTaskTest, HomingSettle_Complete) {
    task.start_homing();
    trigger_stall(); // FAST → RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();
    trigger_stall(); // SLOW → SETTLE
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    EXPECT_TRUE(task.is_homing_done());
    auto rsps = drain_rsp();
    ASSERT_GE(rsps.size(), 2u);
    EXPECT_EQ(rsps[0].cmd, rsp_cmd::ZOOM);
    EXPECT_EQ(rsps[1].cmd, rsp_cmd::HOMING_DONE);
}

TEST_F(MotorTaskTest, Homing_BacklashDisabled) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    task.start_homing();
    EXPECT_FALSE(motor.is_backlash_enabled());
}

TEST_F(MotorTaskTest, Homing_BacklashReenabledAfter) {
    motor.set_backlash(200);
    task.start_homing();
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();
    EXPECT_TRUE(motor.is_backlash_enabled());
}

// ============================================================
// FORCE_STOP (0x02)
// ============================================================

TEST_F(MotorTaskTest, ForceStop_CmdIs0x02) {

    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);

    send_cmd(cmd::FORCE_STOP);
    task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, cmd::FORCE_STOP);
}

// ============================================================
// Stall alarm: 0xE1
// ============================================================

TEST_F(MotorTaskTest, StallDetected_Response0xE1) {

    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);

    trigger_stall();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::STALL_STOP);
}

// ============================================================
// Overcurrent alarm: 0xE2
// ============================================================

TEST_F(MotorTaskTest, OvercurrentDetected_Response0xE2) {

    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();

    trigger_overcurrent();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::OVERCURRENT);
}

// ============================================================
// ZOOM_INC (0x11)
// ============================================================

TEST_F(MotorTaskTest, ZoomInc_RelativeIncrease) {

    int32_t iPos10 = zoom.get_position(10);
    encoder.set_position(iPos10);

    send_cmd(cmd::ZOOM_INC, 10); // +1.0x → target 2.0x
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);
}

TEST_F(MotorTaskTest, ZoomInc_ClampToMax) {

    int32_t iPos50 = zoom.get_position(50);
    encoder.set_position(iPos50);

    uint16_t iMaxZoom = zoom.get_max_zoom();
    send_cmd(cmd::ZOOM_INC, iMaxZoom);
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);
    int32_t iPosMax = zoom.get_position(iMaxZoom);
    EXPECT_EQ(motor.get_target(), iPosMax);
}

// ============================================================
// ZOOM_DEC (0x12)
// ============================================================

TEST_F(MotorTaskTest, ZoomDec_RelativeDecrease) {

    int32_t iPos60 = zoom.get_position(60);
    encoder.set_position(iPos60);

    send_cmd(cmd::ZOOM_DEC, 10); // -1.0x → target 5.0x
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);
}

TEST_F(MotorTaskTest, ZoomDec_ClampToMin) {

    int32_t iPos50 = zoom.get_position(50);
    encoder.set_position(iPos50);

    uint16_t iMinZoom = zoom.get_min_zoom();
    send_cmd(cmd::ZOOM_DEC, 700);
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);
    // Min zoom position is 0 — no soft limits anymore
    int32_t iPosMin = zoom.get_position(iMinZoom);
    EXPECT_EQ(motor.get_target(), iPosMin);
}

// ============================================================
// CYCLE_START (0x30) / CYCLE_STOP (0x31)
// ============================================================

TEST_F(MotorTaskTest, CycleStart_CmdIs0x30) {

    send_cmd(cmd::CYCLE_START, 0x0105); // step=1, dwell=5
    task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::CYCLING);
}

TEST_F(MotorTaskTest, CycleStop_CmdIs0x31) {

    task.start_cycle(1, 1);
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::CYCLING);

    send_cmd(cmd::CYCLE_STOP);
    task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, cmd::CYCLE_STOP);
}

// ============================================================
// Power down
// ============================================================

TEST_F(MotorTaskTest, PowerDown_EmergencyStop) {

    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);

    mock_rtos::set_notify_return(pdTRUE, 0x02);
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
    mock_rtos::set_notify_return(pdFALSE);
}

TEST_F(MotorTaskTest, PowerDown_EmergencySave) {
    encoder.set_position(12345);
    mock_rtos::set_notify_return(pdTRUE, 0x02);
    task.run_once();

    EXPECT_GE(mock::get_log().spi_tx_data.size(), 1u);
    mock_rtos::set_notify_return(pdFALSE);
}

TEST_F(MotorTaskTest, PowerDown_SetsSpiEmergency) {
    g_bSpiEmergency = false;
    mock_rtos::set_notify_return(pdTRUE, 0x02);
    task.run_once();

    EXPECT_TRUE(g_bSpiEmergency);

    mock_rtos::set_notify_return(pdFALSE);
    g_bSpiEmergency = false;
}

// ============================================================
// Cycle zoom sub-steps
// ============================================================

TEST_F(MotorTaskTest, CycleZoom_StepsThrough) {

    task.start_cycle(1, 1);
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::CYCLING);
}

TEST_F(MotorTaskTest, CycleZoom_ReversesAtLimit) {

    encoder.set_position(99000);
    task.start_cycle(1, 1);
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::CYCLING);
}
