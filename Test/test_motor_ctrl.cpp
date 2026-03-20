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
    motor.move_to(100);
    encoder.set_position(80); // within deadzone
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlTest, SetCurrentLimit) {
    motor.set_current_limit(500);
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
    // Phase 1: overshoot = 5000 - 200 - 100 = 4700
    EXPECT_EQ(motor.get_target(), 4700);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::REVERSE);
}

TEST_F(MotorCtrlTest, Backlash_ReverseMove_ApproachingState) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    // Simulate arrival at overshoot
    encoder.set_position(4700);
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
    // Phase 1: arrive at overshoot
    encoder.set_position(4700);
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
    encoder.set_position(4700);
    for (int i = 0; i < 500; ++i) motor.update();
    // Phase 2
    encoder.set_position(5000);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlTest, Backlash_EmergencyStop_DuringApproach) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    encoder.set_position(4700);
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
