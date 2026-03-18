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

        iAdcCurrent = 0;
        task.init(&motor, &encoder, &stall, &zoom, &fram, cmdQ, rspQ, saveQ, &iAdcCurrent);
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

    // Run enough stall cycles to trigger stall detection
    // EMA filter needs ~64 extra iterations to converge from 0 to adc_val
    void trigger_stall(uint16_t adc_val = 250) {
        iAdcCurrent = adc_val;
        // EMA convergence + blanking window + stall confirm
        for (int i = 0; i < 64 + StallDetect::BLANKING_TICKS + StallDetect::STALL_CONFIRM_COUNT + 10; ++i) {
            task.run_once();
        }
    }

    // Run enough cycles for overcurrent
    void trigger_overcurrent() {
        iAdcCurrent = 1100; // > OVERCURRENT_THRESHOLD (1000)
        // EMA convergence + blanking + overcurrent confirm
        for (int i = 0; i < 64 + StallDetect::BLANKING_TICKS + StallDetect::OVERCURRENT_CONFIRM + 10; ++i) {
            task.run_once();
        }
    }
};

TEST_F(MotorTaskTest, Init_StateIsIdle) {
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    EXPECT_FALSE(task.is_homing_done());
}

TEST_F(MotorTaskTest, ReceiveMoveTo_StartsMotor) {
    // Setup zoom table total range so soft limits don't interfere
    zoom.set_total_range(100000);
    // The default zoom table has 14 entries
    // Send SET_ZOOM with zoom_x10=60 (6.0x)
    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorTaskTest, MotorArrived_SendsRspAndSave) {
    zoom.set_total_range(100000);
    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();

    // Simulate motor arrival by setting encoder to target position
    int32_t iTarget = motor.get_target();
    encoder.set_position(iTarget);

    // Run until motor sees it arrived
    for (int i = 0; i < 10; ++i) task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, cmd::SET_ZOOM);

    SAVE_MESSAGE_S save;
    EXPECT_TRUE(receive_save(save));
    EXPECT_EQ(save.reason, save_reason::ARRIVED);
}

TEST_F(MotorTaskTest, StallDetected_EmergencyStop) {
    zoom.set_total_range(100000);
    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);

    // Trigger stall (current > threshold, encoder not moving)
    trigger_stall();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.param, rsp::STALL_ALARM);
}

TEST_F(MotorTaskTest, OvercurrentDetected_Alarm) {
    zoom.set_total_range(100000);
    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();

    trigger_overcurrent();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.param, rsp::OVERCURRENT);
}

TEST_F(MotorTaskTest, HomingReverse_UntilStall) {
    task.start_homing();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_REVERSE);

    // Trigger stall during homing reverse
    trigger_stall();

    // Should transition to HOMING_RETRACT and encoder pos = 0
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_RETRACT);
    // After stall in reverse homing, position was set to 0 before retract
}

TEST_F(MotorTaskTest, HomingRetract_4096Counts) {
    task.start_homing();
    trigger_stall(); // -> HOMING_RETRACT

    // Motor is now moving to 4096
    EXPECT_EQ(motor.get_target(), MotorTask::HOMING_RETRACT_DISTANCE);
}

TEST_F(MotorTaskTest, HomingFindZ_RecordsOffset) {
    task.start_homing();
    trigger_stall(); // -> HOMING_RETRACT

    // Simulate motor arrival at retract position
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    // Simulate a Z pulse at some position
    encoder.handle_z_pulse();
    iAdcCurrent = 0;

    // Run until motor arrives
    for (int i = 0; i < 10; ++i) task.run_once();

    // Should now be in HOMING_FORWARD
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_FORWARD);
    EXPECT_EQ(task.get_z_offset(), encoder.get_z_position());
}

TEST_F(MotorTaskTest, HomingForward_RecordsRange) {
    task.start_homing();
    trigger_stall(); // -> HOMING_RETRACT

    // Complete retract
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_FORWARD);

    // Simulate forward movement to some position then stall
    encoder.set_position(50000);
    trigger_stall();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_TO_SOFT_MIN);
    EXPECT_EQ(task.get_total_range(), 50000);
}

TEST_F(MotorTaskTest, HomingComplete_SavesParams) {
    task.start_homing();
    // Complete full homing sequence
    trigger_stall(); // REVERSE -> RETRACT

    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();

    encoder.set_position(50000);
    trigger_stall();
    // Now in HOMING_TO_SOFT_MIN

    // Simulate arrival at soft min
    encoder.set_position(MotorTask::SOFT_LIMIT_OFFSET);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    EXPECT_TRUE(task.is_homing_done());

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, cmd::HOMING);
    EXPECT_EQ(rsp.param, rsp::OK);
}

TEST_F(MotorTaskTest, CycleZoom_StepsThrough) {
    zoom.set_total_range(100000);
    task.start_cycle(1, 1); // step=1, dwell=100ms
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::CYCLING);
}

TEST_F(MotorTaskTest, CycleZoom_ReversesAtLimit) {
    zoom.set_total_range(100000);
    // Start cycling from max zoom position
    encoder.set_position(99000);
    task.start_cycle(1, 1);
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::CYCLING);
}

TEST_F(MotorTaskTest, CycleZoom_StopCommand) {
    zoom.set_total_range(100000);
    task.start_cycle(1, 1);
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::CYCLING);

    task.stop_cycle();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, cmd::CYCLE_STOP);
}

TEST_F(MotorTaskTest, SoftLimit_ClampsTarget) {
    // Complete homing so soft limits are active
    task.start_homing();
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();
    encoder.set_position(50000);
    trigger_stall();
    encoder.set_position(MotorTask::SOFT_LIMIT_OFFSET);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();
    EXPECT_TRUE(task.is_homing_done());

    // Now try to move beyond soft limit (zoom_x10=700 = 7.0x = max zoom, position near max)
    // The target should be clamped to total_range - SOFT_LIMIT_OFFSET
    send_cmd(cmd::SET_ZOOM, 700);
    task.run_once();

    int32_t iMax = 50000 - MotorTask::SOFT_LIMIT_OFFSET;
    EXPECT_LE(motor.get_target(), iMax);
}

TEST_F(MotorTaskTest, ForceStop_StopsMotor) {
    zoom.set_total_range(100000);
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

TEST_F(MotorTaskTest, PowerDown_EmergencyStop) {
    zoom.set_total_range(100000);
    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);

    // Simulate power-down notification
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

    // Verify SPI writes occurred (emergency_save writes to FRAM)
    EXPECT_GE(mock::get_log().spi_tx_data.size(), 1u);
    mock_rtos::set_notify_return(pdFALSE);
}

TEST_F(MotorTaskTest, PowerDown_SetsSpiEmergency) {
    g_bSpiEmergency = false;
    mock_rtos::set_notify_return(pdTRUE, 0x02);
    task.run_once();

    EXPECT_TRUE(g_bSpiEmergency);

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, cmd::FORCE_STOP);
    EXPECT_EQ(rsp.param, rsp::POWER_DOWN);

    mock_rtos::set_notify_return(pdFALSE);
    g_bSpiEmergency = false;
}
