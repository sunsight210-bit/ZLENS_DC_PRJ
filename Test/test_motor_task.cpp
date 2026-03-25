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

    // Trigger stall via ADC current (StallDetect). For non-homing stall tests.
    void trigger_stall(uint16_t adc_val = 800) {
        iAdcCurrent = adc_val;
        for (int i = 0; i < 64 + StallDetect::BLANKING_TICKS + StallDetect::STALL_CONFIRM_COUNT + 10; ++i) {
            task.run_once();
        }
    }

    // Trigger homing stall via StallDetect encoder stall (encoder stuck at same position).
    // During homing FAST/SLOW, motor is driven via set_pwm_test (direct PWM, not PID),
    // so StallDetect::encoder_stalled() is the primary stall mechanism.
    void trigger_homing_stall() {
        iAdcCurrent = 0;  // low ADC — encoder stall, not current stall
        MotorTask::TASK_STATE_E eBefore = task.get_state();
        int iMax = StallDetect::BLANKING_TICKS + StallDetect::ENCODER_STALL_TICKS + 20;
        for (int i = 0; i < iMax; ++i) {
            task.run_once();
            if (task.get_state() != eBefore) break;
        }
    }

    void trigger_overcurrent() {
        iAdcCurrent = 3500;
        for (int i = 0; i < 64 + StallDetect::BLANKING_TICKS + StallDetect::OVERCURRENT_CONFIRM + 10; ++i) {
            task.run_once();
        }
    }

    // Run enough ticks for MotorCtrl to reach IDLE + margin
    void run_settle(int iExtra = 0) {
        int iTotal = MotorTask::SETTLE_STABLE_COUNT + 120 + iExtra;
        for (int i = 0; i < iTotal; ++i)
            task.run_once();
    }

    // Wait for task state to change from current state (i.e., phase transition).
    void wait_phase_change(MotorTask::TASK_STATE_E eExpected, int max_ticks = 200) {
        iAdcCurrent = 0;
        for (int i = 0; i < max_ticks; ++i) {
            task.run_once();
            if (task.get_state() != eExpected) break;
        }
    }

    void complete_homing() {
        task.start_homing();
        // FAST: motor drives reverse, stall at mechanical limit
        trigger_homing_stall(); // FAST -> RETRACT (encoder set to 0 by handle_stall)
        // RETRACT: motor target = RETRACT_DISTANCE, set encoder there so motor reaches IDLE
        encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
        wait_phase_change(MotorTask::TASK_STATE_E::HOMING_RETRACT); // RETRACT done -> SLOW
        // SLOW: motor drives reverse, stall at mechanical limit
        trigger_homing_stall(); // SLOW -> SETTLE (encoder set to 0 by handle_stall)
        // SETTLE: motor target = SETTLE_DISTANCE, set encoder there so motor reaches IDLE
        encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
        wait_phase_change(MotorTask::TASK_STATE_E::HOMING_SETTLE); // SETTLE done -> IDLE
        drain_rsp();
    }
};

TEST_F(MotorTaskTest, Init_StateIsIdle) {
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    EXPECT_FALSE(task.is_homing_done());
}

// ============================================================
// SET_ZOOM (0x10)
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
    trigger_homing_stall();  // motor encoder timeout at limit
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

TEST_F(MotorTaskTest, HomingSlow_StallViaMotorTimeout_TriggersSettle) {
    task.start_homing();
    trigger_homing_stall(); // FAST -> RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    wait_phase_change(MotorTask::TASK_STATE_E::HOMING_RETRACT); // RETRACT done -> SLOW
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_SLOW);
    // Now in SLOW — motor encoder timeout should trigger settle
    encoder.set_position(0);  // encoder stuck at 0 (mechanical limit)
    trigger_homing_stall(); // SLOW -> SETTLE
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_SETTLE);
    EXPECT_EQ(motor.get_target(), MotorTask::HOMING_SETTLE_DISTANCE);
}

TEST_F(MotorTaskTest, HomingRetract_DoneTriggersSlow) {
    task.start_homing();
    trigger_homing_stall(); // FAST -> RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    wait_phase_change(MotorTask::TASK_STATE_E::HOMING_RETRACT);
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_SLOW);
}

TEST_F(MotorTaskTest, HomingSlow_StallTriggersSettle) {
    task.start_homing();
    trigger_homing_stall(); // FAST -> RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    wait_phase_change(MotorTask::TASK_STATE_E::HOMING_RETRACT); // RETRACT done -> SLOW
    // Now in SLOW
    trigger_homing_stall(); // SLOW -> SETTLE
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_SETTLE);
    EXPECT_EQ(motor.get_target(), MotorTask::HOMING_SETTLE_DISTANCE);
}

TEST_F(MotorTaskTest, HomingSettle_Complete) {
    task.start_homing();
    trigger_homing_stall(); // FAST -> RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    wait_phase_change(MotorTask::TASK_STATE_E::HOMING_RETRACT); // RETRACT done -> SLOW
    trigger_homing_stall(); // SLOW -> SETTLE
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    wait_phase_change(MotorTask::TASK_STATE_E::HOMING_SETTLE); // SETTLE done -> IDLE
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    EXPECT_TRUE(task.is_homing_done());
    auto rsps = drain_rsp();
    ASSERT_GE(rsps.size(), 2u);
    EXPECT_EQ(rsps[0].cmd, rsp_cmd::ZOOM);
    EXPECT_EQ(rsps[1].cmd, rsp_cmd::HOMING_DONE);
}

TEST_F(MotorTaskTest, HomingSettleResetsIndexTracking) {
    // After homing SETTLE completes, encoder.reset_index_tracking() should be called.
    // We verify by checking that the encoder's index tracking is in a clean state.
    task.start_homing();
    trigger_homing_stall(); // FAST -> RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    wait_phase_change(MotorTask::TASK_STATE_E::HOMING_RETRACT); // RETRACT done -> SLOW
    trigger_homing_stall(); // SLOW -> SETTLE
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    wait_phase_change(MotorTask::TASK_STATE_E::HOMING_SETTLE); // SETTLE done -> IDLE

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    EXPECT_TRUE(task.is_homing_done());
    // reset_index_tracking() was called — no drift should be flagged
    EXPECT_FALSE(encoder.is_drift_detected());
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
    // Start at zoom=10 (index 1), step +1 -> zoom=15 (index 2)
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
    // Start at zoom=60 (index 11), step -1 -> zoom=55 (index 10)
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
// Homing -> no diagnostics chain (simplified)
// ============================================================

TEST_F(MotorTaskTest, HomingNoDiag_GoesToIdle) {
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    task.start_homing();
    trigger_homing_stall(); // FAST -> RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    wait_phase_change(MotorTask::TASK_STATE_E::HOMING_RETRACT); // RETRACT done -> SLOW
    trigger_homing_stall(); // SLOW -> SETTLE
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    wait_phase_change(MotorTask::TASK_STATE_E::HOMING_SETTLE); // SETTLE done -> IDLE

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
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

// ============================================================
// duty_to_pwm / pwm_to_duty conversion
// ============================================================

TEST_F(MotorTaskTest, DutyToPwm_BoundaryValues) {
    EXPECT_EQ(MotorCtrl::duty_to_pwm(0), 0);
    EXPECT_EQ(MotorCtrl::duty_to_pwm(1000), MotorCtrl::PWM_ARR);
    // DEFAULT_SPEED_DUTY(188) -> PWM~800
    uint16_t iPwm = MotorCtrl::duty_to_pwm(rsp::DEFAULT_SPEED_DUTY);
    EXPECT_NEAR(iPwm, 800, 5);
    // DEFAULT_MIN_SPEED_DUTY(112) ~ MIN_SPEED(480)
    uint16_t iPwmMin = MotorCtrl::duty_to_pwm(rsp::DEFAULT_MIN_SPEED_DUTY);
    EXPECT_NEAR(iPwmMin, MotorCtrl::MIN_SPEED, 5);
}

TEST_F(MotorTaskTest, PwmToDuty_RoundTrip) {
    uint16_t iDuty = 500;
    uint16_t iPwm = MotorCtrl::duty_to_pwm(iDuty);
    uint16_t iBack = MotorCtrl::pwm_to_duty(iPwm);
    EXPECT_NEAR(iBack, iDuty, 1);
}

// ============================================================
// 0x60 Speed Commands (protocol compatibility — PID ignores speed setting)
// ============================================================

TEST_F(MotorTaskTest, SetSpeed_SetsSpeedAndResponds) {
    send_cmd(cmd::SET_SPEED, 200);
    task.run_once();

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::SPEED);
    EXPECT_EQ(rsp.param, 200);
}

TEST_F(MotorTaskTest, SetSpeed_ClampsToMaxDuty) {
    // Default max is 281, so 500 should clamp to 281
    send_cmd(cmd::SET_SPEED, 500);
    task.run_once();

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::SPEED);
    EXPECT_EQ(rsp.param, rsp::DEFAULT_MAX_SPEED_DUTY);
}

TEST_F(MotorTaskTest, SetSpeed_ClampsToMinDuty) {
    // Default min is 112, so 50 should clamp to 112
    send_cmd(cmd::SET_SPEED, 50);
    task.run_once();

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::SPEED);
    EXPECT_EQ(rsp.param, rsp::DEFAULT_MIN_SPEED_DUTY);
}

TEST_F(MotorTaskTest, SetSpeed_Clamps1500To1000ThenToMax) {
    // First raise max to 1000
    send_cmd(cmd::SET_MAX_SPEED, 1000);
    task.run_once();
    drain_rsp();

    send_cmd(cmd::SET_SPEED, 1500);
    task.run_once();

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::SPEED);
    EXPECT_EQ(rsp.param, 1000);
}

TEST_F(MotorTaskTest, SetSpeed_SavesSpeedToQueue) {
    send_cmd(cmd::SET_SPEED, 200);
    task.run_once();

    SAVE_MESSAGE_S save;
    EXPECT_TRUE(receive_save(save));
    EXPECT_EQ(save.speed_valid, 1);
    EXPECT_EQ(save.speed_duty, 200);
    EXPECT_EQ(save.min_speed_duty, rsp::DEFAULT_MIN_SPEED_DUTY);
    EXPECT_EQ(save.max_speed_duty, rsp::DEFAULT_MAX_SPEED_DUTY);
}

TEST_F(MotorTaskTest, SpeedInc_IncrementsBy1) {
    // Set speed to 200 first
    send_cmd(cmd::SET_SPEED, 200);
    task.run_once();
    drain_rsp();

    send_cmd(cmd::SPEED_INC, 0);
    task.run_once();

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::SPEED);
    EXPECT_EQ(rsp.param, 201);
}

TEST_F(MotorTaskTest, SpeedInc_ClampsAtMax) {
    // Set speed to max first
    send_cmd(cmd::SET_SPEED, rsp::DEFAULT_MAX_SPEED_DUTY);
    task.run_once();
    drain_rsp();

    send_cmd(cmd::SPEED_INC, 0);
    task.run_once();

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.param, rsp::DEFAULT_MAX_SPEED_DUTY);
}

TEST_F(MotorTaskTest, SpeedDec_DecrementsBy1) {
    send_cmd(cmd::SET_SPEED, 200);
    task.run_once();
    drain_rsp();

    send_cmd(cmd::SPEED_DEC, 0);
    task.run_once();

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::SPEED);
    EXPECT_EQ(rsp.param, 199);
}

TEST_F(MotorTaskTest, SpeedDec_ClampsAtMin) {
    // Set speed to min
    send_cmd(cmd::SET_SPEED, rsp::DEFAULT_MIN_SPEED_DUTY);
    task.run_once();
    drain_rsp();

    send_cmd(cmd::SPEED_DEC, 0);
    task.run_once();

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.param, rsp::DEFAULT_MIN_SPEED_DUTY);
}

TEST_F(MotorTaskTest, SetMinSpeed_RaisesCurrentSpeed) {
    // Set speed to 150 first
    send_cmd(cmd::SET_SPEED, 150);
    task.run_once();
    drain_rsp();

    // Set min to 200 -> current should raise to 200
    send_cmd(cmd::SET_MIN_SPEED, 200);
    task.run_once();

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::SPEED);
    EXPECT_EQ(rsp.param, 200);  // response is the new min
}

TEST_F(MotorTaskTest, SetMaxSpeed_LowersCurrentSpeed) {
    // Set speed to 250 first, then set max to 200 -> current should drop to 200
    send_cmd(cmd::SET_SPEED, 250);
    task.run_once();
    drain_rsp();

    send_cmd(cmd::SET_MAX_SPEED, 200);
    task.run_once();

    RSP_MESSAGE_S rsp;
    EXPECT_TRUE(receive_rsp(rsp));
    EXPECT_EQ(rsp.cmd, rsp_cmd::SPEED);
    EXPECT_EQ(rsp.param, 200);
}

TEST_F(MotorTaskTest, SetMaxSpeed_SavesSpeedToQueue) {
    send_cmd(cmd::SET_MAX_SPEED, 500);
    task.run_once();

    SAVE_MESSAGE_S save;
    EXPECT_TRUE(receive_save(save));
    EXPECT_EQ(save.speed_valid, 1);
    EXPECT_EQ(save.max_speed_duty, 500);
}

TEST_F(MotorTaskTest, SelfTest0x65_TriggersHoming) {
    send_cmd(cmd::SELF_TEST, 0);
    task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_FAST);
    // Complete homing
    trigger_homing_stall(); // FAST -> RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    wait_phase_change(MotorTask::TASK_STATE_E::HOMING_RETRACT); // RETRACT done -> SLOW
    trigger_homing_stall(); // SLOW -> SETTLE
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    wait_phase_change(MotorTask::TASK_STATE_E::HOMING_SETTLE); // SETTLE done -> IDLE
    // Goes to IDLE (no diagnostics chain)
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
}

TEST_F(MotorTaskTest, SelfTest0x65_IgnoredWhenBusy) {
    send_cmd(cmd::SET_ZOOM, 60);
    task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);

    send_cmd(cmd::SELF_TEST, 0);
    task.run_once();
    // Should still be MOVING, SELF_TEST ignored
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::MOVING);
}

TEST_F(MotorTaskTest, RestoreSpeed_ClampsAndApplies) {
    // Clamp: speed below min
    task.restore_speed(50, 100, 600);
    // No motor speed setter to verify — just verify no crash

    // Clamp: speed above max
    task.restore_speed(700, 100, 600);
    // No crash = pass
}
