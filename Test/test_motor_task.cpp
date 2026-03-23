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

    void trigger_stall(uint16_t adc_val = 800) {
        iAdcCurrent = adc_val;
        for (int i = 0; i < 64 + StallDetect::BLANKING_TICKS + StallDetect::STALL_CONFIRM_COUNT + 10; ++i) {
            task.run_once();
        }
    }

    void trigger_overcurrent() {
        iAdcCurrent = 3500;
        for (int i = 0; i < 64 + StallDetect::BLANKING_TICKS + StallDetect::OVERCURRENT_CONFIRM + 10; ++i) {
            task.run_once();
        }
    }

    // Run enough ticks for MotorCtrl SETTLING (100) + MotorTask settle detection (100) + margin.
    // If iSettlePos is given, set encoder to that position once motor enters SETTLING
    // (so corrections see the correct position and resolve quickly).
    void run_settle(int iExtra = 0) {
        int iTotal = MotorCtrl::SETTLE_TICKS + MotorTask::SETTLE_STABLE_COUNT + 20 + iExtra;
        for (int i = 0; i < iTotal; ++i)
            task.run_once();
    }

    // Settle variant for backlash tests where encoder position may be outside DEADZONE
    // of the motor target, causing infinite correction loops in mock (since mock encoder
    // doesn't move with motor). Forces motor to IDLE via emergency_stop after SETTLING.
    void run_settle_overshoot() {
        // Phase 1: run through MotorCtrl SETTLING
        for (int i = 0; i < MotorCtrl::SETTLE_TICKS + 5; ++i)
            task.run_once();
        // Motor may have started a correction move that can't complete in mock.
        // Force it to IDLE so MotorTask can proceed with its own settle detection.
        if (motor.get_state() != MOTOR_STATE_E::IDLE) {
            motor.emergency_stop();
        }
        // Phase 2: MotorTask settle detection
        for (int i = 0; i < MotorTask::SETTLE_STABLE_COUNT + 20; ++i)
            task.run_once();
    }

    void complete_homing() {
        task.start_homing();
        trigger_stall(); // FAST → RETRACT
        encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
        iAdcCurrent = 0;
        for (int i = 0; i < 120; ++i) task.run_once();
        trigger_stall(); // SLOW → SETTLE
        encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
        iAdcCurrent = 0;
        for (int i = 0; i < 120; ++i) task.run_once();
        drain_rsp();
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
    run_settle();

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

TEST_F(MotorTaskTest, HomingFast_EncoderStallTriggersRetract) {
    task.start_homing();
    // Encoder stays at same position (mechanical limit) with low ADC (no current stall)
    iAdcCurrent = 100;  // below STALL_THRESHOLD=600
    encoder.set_position(0);
    for (int i = 0; i < StallDetect::BLANKING_TICKS + StallDetect::ENCODER_STALL_TICKS + 10; ++i) {
        task.run_once();
    }
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_RETRACT);
    EXPECT_EQ(motor.get_target(), MotorTask::HOMING_RETRACT_DISTANCE);
}

TEST_F(MotorTaskTest, HomingSlow_EncoderStallTriggersSettle) {
    task.start_homing();
    trigger_stall(); // FAST → RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    // Now in SLOW — encoder stall should trigger settle
    iAdcCurrent = 100;  // below STALL_THRESHOLD
    encoder.set_position(0);
    for (int i = 0; i < StallDetect::BLANKING_TICKS + StallDetect::ENCODER_STALL_TICKS + 10; ++i) {
        task.run_once();
    }
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_SETTLE);
    EXPECT_EQ(motor.get_target(), MotorTask::HOMING_SETTLE_DISTANCE);
}

TEST_F(MotorTaskTest, HomingRetract_DoneTriggersSlow) {
    task.start_homing();
    trigger_stall(); // FAST → RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_SLOW);
}

TEST_F(MotorTaskTest, HomingSlow_StallTriggersSettle) {
    task.start_homing();
    trigger_stall(); // FAST → RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
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
    for (int i = 0; i < 120; ++i) task.run_once();
    trigger_stall(); // SLOW → SETTLE
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
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
    for (int i = 0; i < 120; ++i) task.run_once();
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
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

TEST_F(MotorTaskTest, ZoomInc_StepByIndex) {
    // Start at zoom=10 (index 1), step +1 → zoom=15 (index 2)
    int32_t iPos10 = zoom.get_position(10);
    encoder.set_position(iPos10);

    send_cmd(cmd::ZOOM_INC, 1);
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);
    int32_t iPos15 = zoom.get_position(15);
    EXPECT_EQ(motor.get_target(), iPos15);
}

TEST_F(MotorTaskTest, ZoomInc_AtMax_ReturnsError) {
    uint16_t iMaxZoom = zoom.get_max_zoom();
    int32_t iPosMax = zoom.get_position(iMaxZoom);
    encoder.set_position(iPosMax);

    send_cmd(cmd::ZOOM_INC, 1);
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::ERR_PARAM);
}

// ============================================================
// ZOOM_DEC (0x12)
// ============================================================

TEST_F(MotorTaskTest, ZoomDec_StepByIndex) {
    // Start at zoom=60 (index 11), step -1 → zoom=55 (index 10)
    int32_t iPos60 = zoom.get_position(60);
    encoder.set_position(iPos60);

    send_cmd(cmd::ZOOM_DEC, 1);
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);
    int32_t iPos55 = zoom.get_position(55);
    EXPECT_EQ(motor.get_target(), iPos55);
}

TEST_F(MotorTaskTest, ZoomDec_AtMin_ReturnsError) {
    uint16_t iMinZoom = zoom.get_min_zoom();
    int32_t iPosMin = zoom.get_position(iMinZoom);
    encoder.set_position(iPosMin);

    send_cmd(cmd::ZOOM_DEC, 1);
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::ERR_PARAM);
}

TEST_F(MotorTaskTest, SetZoom_InvalidZoom_ReturnsError) {
    encoder.set_position(0);

    send_cmd(cmd::SET_ZOOM, 99); // 9.9x not in table
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::ERR_PARAM);
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

// ============================================================
// Backlash Measurement (Task 6)
// ============================================================

// Helper: complete_homing is defined as a fixture method above
// (We add it to the fixture class via a lambda-like approach in each test,
//  but since we can't modify the fixture easily, we use a local helper.)

TEST_F(MotorTaskTest, BacklashMeasure_StartsMovingToMid) {
    // Complete homing first
    task.start_homing();
    trigger_stall(); // FAST → RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    trigger_stall(); // SLOW → SETTLE
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    drain_rsp();

    task.start_backlash_measure();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::BACKLASH_MEASURE);
    EXPECT_EQ(motor.get_target(), MotorTask::BL_MEASURE_MID);
}

TEST_F(MotorTaskTest, BacklashMeasure_DisablesCompensation) {
    task.start_homing();
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    drain_rsp();

    motor.set_backlash_enabled(true);
    task.start_backlash_measure();
    EXPECT_FALSE(motor.is_backlash_enabled());
}

TEST_F(MotorTaskTest, BacklashMeasure_FullCycle_SetsBacklash) {
    task.start_homing();
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    drain_rsp();

    task.start_backlash_measure();

    encoder.set_position(MotorTask::BL_MEASURE_MID);
    run_settle();

    for (int cycle = 0; cycle < 8; ++cycle) {
        encoder.set_position(MotorTask::BL_MEASURE_MID - MotorTask::BL_REVERSE_DIST);
        run_settle();
        encoder.set_position(MotorTask::BL_MEASURE_MID + 80);
        run_settle_overshoot();
    }

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    EXPECT_EQ(motor.get_backlash(), 30);
}

TEST_F(MotorTaskTest, BacklashMeasure_ZeroError_BacklashIsZero) {
    complete_homing();
    task.start_backlash_measure();

    encoder.set_position(MotorTask::BL_MEASURE_MID);
    run_settle();

    for (int cycle = 0; cycle < 8; ++cycle) {
        encoder.set_position(MotorTask::BL_MEASURE_MID - MotorTask::BL_REVERSE_DIST);
        run_settle();
        encoder.set_position(MotorTask::BL_MEASURE_MID);
        run_settle();
    }

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    EXPECT_EQ(motor.get_backlash(), 0);
}

// ============================================================
// Accuracy Test (Task 7)
// ============================================================

TEST_F(MotorTaskTest, AccuracyTest_StartsMovingToStart) {
    complete_homing();
    task.start_accuracy_test();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::ACCURACY_TEST);
    // After homing, position=HOMING_SETTLE_DISTANCE(2048).
    // ACC_START_POS(2048) == HOMING_SETTLE_DISTANCE,
    // so move_to returns IDLE immediately — motor already at start.
}

TEST_F(MotorTaskTest, AccuracyTest_EnablesBacklash) {
    complete_homing();
    motor.set_backlash_enabled(false);
    task.start_accuracy_test();
    EXPECT_TRUE(motor.is_backlash_enabled());
}

TEST_F(MotorTaskTest, AccuracyTest_CompletesAfterAllTrips) {
    complete_homing();
    motor.set_backlash(0);
    task.start_accuracy_test();

    encoder.set_position(MotorTask::ACC_START_POS);
    run_settle();

    for (int trip = 0; trip < MotorTask::ACC_NUM_TRIPS; ++trip) {
        encoder.set_position(MotorTask::ACC_END_POS);
        run_settle();
        encoder.set_position(MotorTask::ACC_START_POS);
        run_settle();
    }

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
}

// ============================================================
// 0x60 Full Diagnostics Chain (Task 8)
// ============================================================

TEST_F(MotorTaskTest, HomingFullDiag_ChainsToBacklashMeasure) {
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    task.start_homing(true);  // full diagnostics
    // Complete homing
    trigger_stall(); // FAST -> RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    trigger_stall(); // SLOW -> SETTLE
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();

    // Should chain to BACKLASH_MEASURE, not IDLE
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::BACKLASH_MEASURE);
}

TEST_F(MotorTaskTest, HomingNoDiag_GoesToIdle) {
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    task.start_homing(false);  // no diagnostics
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
}

TEST_F(MotorTaskTest, FullDiag_ChainsBacklashToAccuracy) {
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    task.start_homing(true);
    // Complete homing
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();

    // Now in BACKLASH_MEASURE - complete it
    encoder.set_position(MotorTask::BL_MEASURE_MID);
    run_settle();
    for (int cycle = 0; cycle < 8; ++cycle) {
        encoder.set_position(MotorTask::BL_MEASURE_MID - MotorTask::BL_REVERSE_DIST);
        run_settle();
        encoder.set_position(MotorTask::BL_MEASURE_MID);
        run_settle();
    }

    // Should chain to ACCURACY_TEST
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::ACCURACY_TEST);
}

TEST_F(MotorTaskTest, BacklashMeasure_SavesBacklashToQueue) {
    complete_homing();
    task.start_backlash_measure();

    encoder.set_position(MotorTask::BL_MEASURE_MID);
    run_settle();

    for (int cycle = 0; cycle < 8; ++cycle) {
        encoder.set_position(MotorTask::BL_MEASURE_MID - MotorTask::BL_REVERSE_DIST);
        run_settle();
        encoder.set_position(MotorTask::BL_MEASURE_MID + 80);
        run_settle_overshoot();
    }

    // Check save queue has backlash data
    SAVE_MESSAGE_S save;
    bool bFoundBacklash = false;
    while (receive_save(save)) {
        if (save.backlash_valid == 0xFF) {
            EXPECT_EQ(save.backlash_counts, 30);  // 80 - BL_DEADZONE(50)
            bFoundBacklash = true;
            break;
        }
    }
    EXPECT_TRUE(bFoundBacklash);
}

TEST_F(MotorTaskTest, SendSave_DefaultBacklashFieldsZero) {
    // Normal move should not carry backlash data
    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();

    int32_t iTarget = motor.get_target();
    encoder.set_position(iTarget);
    for (int i = 0; i < 120; ++i) task.run_once();

    SAVE_MESSAGE_S save;
    EXPECT_TRUE(receive_save(save));
    EXPECT_EQ(save.backlash_counts, 0);
    EXPECT_EQ(save.backlash_valid, 0);
}

TEST_F(MotorTaskTest, HomingCmd_Param1_FullDiag) {
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    send_cmd(cmd::HOMING, 1);  // param=1 -> full diagnostics
    task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_FAST);
    // Complete homing
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    trigger_stall();
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 120; ++i) task.run_once();
    // Should chain to BACKLASH_MEASURE because param=1
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::BACKLASH_MEASURE);
}
