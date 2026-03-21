// Test/test_motor_ctrl.cpp
#include <gtest/gtest.h>
#include "motor_ctrl.hpp"

using namespace zlens;

class MotorCtrlTest : public ::testing::Test {
protected:
    MotorCtrl motor;
    Encoder encoder;
    TIM_HandleTypeDef htim3;
    DAC_HandleTypeDef hdac;

    void SetUp() override {
        mock::get_log().reset();
        htim3.Instance = TIM3;
        hdac.Instance = DAC1;
        encoder.init();
        motor.init(&htim3, &hdac, &encoder);
    }
};

TEST_F(MotorCtrlTest, InitialState_IsIdle) {
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlTest, MoveTo_StartsAccelerating) {
    motor.move_to(10000);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::ACCELERATING);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::FORWARD);
}

TEST_F(MotorCtrlTest, MoveTo_Reverse) {
    encoder.set_position(50000);
    motor.move_to(10000);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::REVERSE);
}

TEST_F(MotorCtrlTest, Update_AcceleratesSpeed) {
    motor.move_to(100000);
    uint16_t speed1 = motor.get_current_speed();
    motor.update();
    uint16_t speed2 = motor.get_current_speed();
    EXPECT_GT(speed2, speed1);
}

TEST_F(MotorCtrlTest, EmergencyStop_GoesToIdle) {
    motor.move_to(10000);
    motor.emergency_stop();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlTest, ArrivedAtTarget_StopsAndIdle) {
    motor.move_to(5000);
    encoder.set_position(4500); // within deadzone (1000)
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlTest, SetVrefMv) {
    motor.set_vref_mv(1000);
    EXPECT_GE(mock::get_log().dac_sets.size(), 1u);
}

TEST_F(MotorCtrlTest, Stop_DeceleratesFirst) {
    motor.move_to(100000);
    for (int i = 0; i < 100; ++i) motor.update();
    motor.stop();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::DECELERATING);
}

// --- Backlash compensation tests ---

TEST_F(MotorCtrlTest, SetSpeedLimit_LimitsMaxSpeed) {
    motor.set_speed_limit(1000);
    motor.move_to(100000);
    for (int i = 0; i < 500; ++i) motor.update();
    EXPECT_LE(motor.get_current_speed(), 1000u);
}

TEST_F(MotorCtrlTest, Backlash_ForwardMove_NoCompensation) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(1000);
    motor.move_to(5000);
    EXPECT_EQ(motor.get_target(), 5000);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::FORWARD);
}

TEST_F(MotorCtrlTest, Backlash_Disabled_NoCompensation) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(false);
    encoder.set_position(10000);
    motor.move_to(5000);
    EXPECT_EQ(motor.get_target(), 5000);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::REVERSE);
}

TEST_F(MotorCtrlTest, Backlash_ReverseMove_OvershootTarget) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    // Phase 1: overshoot = 5000 - 200 - 200 = 4600
    EXPECT_EQ(motor.get_target(), 4600);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::REVERSE);
}

TEST_F(MotorCtrlTest, Backlash_ReverseMove_ApproachingState) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    // Simulate arrival at overshoot (4600)
    encoder.set_position(4600);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::APPROACHING) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::APPROACHING);
}

TEST_F(MotorCtrlTest, Backlash_ReverseMove_Phase2Forward) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    // Phase 1: arrive at overshoot (4600)
    encoder.set_position(4600);
    for (int i = 0; i < 500; ++i) motor.update();
    // Phase 2: should now target 5000, forward direction
    motor.update();
    EXPECT_EQ(motor.get_target(), 5000);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::FORWARD);
}

TEST_F(MotorCtrlTest, Backlash_ReverseMove_CompletesAtFinalTarget) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    // Phase 1
    encoder.set_position(4600);
    for (int i = 0; i < 500; ++i) motor.update();
    // Phase 2 — need to be within FINE_DEADZONE(100) of 5000
    encoder.set_position(4950);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlTest, Backlash_ReverseMove_NearZero_ClampsOvershoot) {
    motor.set_backlash(997);
    motor.set_backlash_enabled(true);
    encoder.set_position(2000);
    motor.move_to(800);
    // overshoot = 800 - 997 - 200 = -397 < FINE_DEADZONE(100), clamped to 100
    EXPECT_EQ(motor.get_target(), 100);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::REVERSE);
}

TEST_F(MotorCtrlTest, Backlash_ReverseMove_LargeBacklash) {
    motor.set_backlash(997);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    // overshoot = 5000 - 997 - 200 = 3803
    EXPECT_EQ(motor.get_target(), 3803);
}

TEST_F(MotorCtrlTest, Backlash_Phase2_StartsAtCorrectionSpeed) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    // Arrive at overshoot (4600) to trigger APPROACHING
    encoder.set_position(4600);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::APPROACHING) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::APPROACHING);
    // One update transitions out of APPROACHING
    motor.update();
    EXPECT_EQ(motor.get_current_speed(), MotorCtrl::MIN_CORRECTION_SPEED);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::CONSTANT);
}

TEST_F(MotorCtrlTest, Backlash_EmergencyStop_DuringApproach) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    encoder.set_position(4600);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::APPROACHING) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::APPROACHING);
    motor.emergency_stop();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
    motor.update();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

// --- Phase2 fine deadzone tests ---

TEST_F(MotorCtrlTest, Phase2_UsesFineDZone_NotCoarseDZone) {
    // Phase2 should stop within FINE_DEADZONE(100), not DEADZONE(1000)
    motor.set_backlash(257);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    // Phase 1: overshoot = 5000 - 257 - 200 = 4543
    EXPECT_EQ(motor.get_target(), 4543);
    // Arrive at overshoot → APPROACHING
    encoder.set_position(4543);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::APPROACHING) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::APPROACHING);
    // Enter Phase2
    motor.update();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::CONSTANT);
    EXPECT_EQ(motor.get_target(), 5000);
    // Position at 4500 — 500 from target, within old DEADZONE but NOT FINE_DEADZONE
    // Phase2 should NOT stop here
    encoder.set_position(4500);
    motor.update();
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::IDLE);
    // Position at 4950 — within FINE_DEADZONE(100) of 5000
    encoder.set_position(4950);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlTest, Phase2_NearZero_BacklashNotSkipped) {
    // 0.6x scenario: target=2048, backlash=257
    // overshoot = 2048 - 257 - 200 = 1591 >= FINE_DEADZONE(100), compensation NOT skipped
    motor.set_backlash(257);
    motor.set_backlash_enabled(true);
    encoder.set_position(829635);  // 7.0x position
    motor.move_to(2048);
    EXPECT_EQ(motor.get_target(), 1591);  // overshoot target
    EXPECT_NE(motor.get_target(), 2048);  // compensation was applied
}
