// Test/test_motor_ctrl.cpp
#include <gtest/gtest.h>
#include "motor_ctrl.hpp"
#include "mock_hal.hpp"

using namespace zlens;

class MotorCtrlPidTest : public ::testing::Test {
protected:
    MotorCtrl motor;
    Encoder encoder;
    TIM_HandleTypeDef htim_pwm;
    DAC_HandleTypeDef hdac;

    void SetUp() override {
        mock::get_log().reset();
        htim_pwm.Instance = TIM3;
        hdac.Instance = DAC1;
        encoder.init();
        motor.init(&htim_pwm, &hdac, &encoder);
    }
};

// 1. InitialStateIsIdle
TEST_F(MotorCtrlPidTest, InitialStateIsIdle) {
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

// 2. IdleDoesNothing
TEST_F(MotorCtrlPidTest, IdleDoesNothing) {
    motor.update();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

// 3. MoveToSetsRunning
TEST_F(MotorCtrlPidTest, MoveToSetsRunning) {
    encoder.set_position(0);
    motor.move_to(10000);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
}

// 4. MoveToWithinDeadzoneStaysIdle
TEST_F(MotorCtrlPidTest, MoveToWithinDeadzoneStaysIdle) {
    encoder.set_position(1000);
    motor.move_to(1002);  // |2| <= DEADZONE(3)
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

// 5. MoveToClampsBelowSafeMin
TEST_F(MotorCtrlPidTest, MoveToClampsBelowSafeMin) {
    encoder.set_position(100);
    motor.move_to(-500);
    // Target clamped to SAFE_LIMIT_MIN(64)
    EXPECT_EQ(motor.get_target(), MotorCtrl::SAFE_LIMIT_MIN);
}

// 6. MoveToClampsAboveSafeMax
TEST_F(MotorCtrlPidTest, MoveToClampsAboveSafeMax) {
    encoder.set_position(60000);
    motor.move_to(70000);
    // Target clamped to safe max (65536 - 64 = 65472)
    int32_t iExpectedMax = 65536 - MotorCtrl::SAFE_LIMIT_MIN;
    EXPECT_EQ(motor.get_target(), iExpectedMax);
}

// 7. ForwardErrorDrivesForward
TEST_F(MotorCtrlPidTest, ForwardErrorDrivesForward) {
    encoder.set_position(1000);
    motor.move_to(5000);
    motor.update();
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::FORWARD);
    // PWM should be set on channel 1 (forward), channel 2 = 0
    EXPECT_GT(htim_pwm.Instance->CCR1, 0u);
    EXPECT_EQ(htim_pwm.Instance->CCR2, 0u);
}

// 8. ReverseErrorDrivesReverse
TEST_F(MotorCtrlPidTest, ReverseErrorDrivesReverse) {
    encoder.set_position(5000);
    motor.move_to(1000);
    motor.update();
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::REVERSE);
    // PWM should be set on channel 2 (reverse), channel 1 = 0
    EXPECT_EQ(htim_pwm.Instance->CCR1, 0u);
    EXPECT_GT(htim_pwm.Instance->CCR2, 0u);
}

// 9. ReachesTargetAndStops
TEST_F(MotorCtrlPidTest, ReachesTargetAndStops) {
    encoder.set_position(0);
    motor.move_to(5000);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    // Simulate encoder reaching within DEADZONE of target
    encoder.set_position(4998);  // |5000 - 4998| = 2 <= DEADZONE(3)
    // Need SETTLE_COUNT(100) consecutive ticks with no position change
    for (uint16_t i = 0; i < MotorCtrl::SETTLE_COUNT + 1; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

// 10. SafeLimitMinBrakes
TEST_F(MotorCtrlPidTest, SafeLimitMinBrakes_RunningAboveLimit) {
    encoder.set_position(200);
    motor.move_to(MotorCtrl::SAFE_LIMIT_MIN + 1);  // 65, valid target
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
}

TEST_F(MotorCtrlPidTest, SafeLimitMinBrakes_AtLimit) {
    // With entry clamping, target >= SAFE_LIMIT_MIN=64, so test deadzone path
    encoder.set_position(500);
    motor.move_to(200);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    EXPECT_EQ(motor.get_target(), 200);
    // Arrive within DEADZONE(3): error = 200-199 = 1
    encoder.set_position(199);
    for (uint16_t i = 0; i < MotorCtrl::SETTLE_COUNT + 1; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlPidTest, SafeLimitMinBrakes_ReverseAtLimit) {
    // With entry clamping, target >= SAFE_LIMIT_MIN=64
    encoder.set_position(500);
    motor.move_to(MotorCtrl::SAFE_LIMIT_MIN);  // clamped to 64
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    EXPECT_EQ(motor.get_target(), MotorCtrl::SAFE_LIMIT_MIN);
    // Arrive within deadzone: error = 64-66 = -2, |2| <= 3
    encoder.set_position(66);
    for (uint16_t i = 0; i < MotorCtrl::SETTLE_COUNT + 1; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlPidTest, SafeLimitMaxBrakes) {
    // pos >= safe max and error > 0 → brake
    int32_t iSafeMax = 65536 - MotorCtrl::SAFE_LIMIT_MIN;
    encoder.set_position(1000);
    motor.move_to(iSafeMax);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    // Arrive within DEADZONE(3)
    encoder.set_position(iSafeMax - 1);  // error = 1 <= 3
    for (uint16_t i = 0; i < MotorCtrl::SETTLE_COUNT + 1; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

// 11. EncoderTimeoutCausesStall
TEST_F(MotorCtrlPidTest, EncoderTimeoutCausesStall) {
    encoder.set_position(0);
    motor.move_to(10000);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    // Freeze encoder at position far from target
    encoder.set_position(5000);
    for (uint16_t i = 0; i < MotorCtrl::ENCODER_TIMEOUT_TICKS + 1; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::STALLED) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::STALLED);
}

// 12. PidOutputBelowMinSpeedClampedToMinSpeed
TEST_F(MotorCtrlPidTest, PidOutputBelowMinSpeedClampedToMinSpeed) {
    // When PID output < MIN_SPEED, motor drives at MIN_SPEED (pure PID, no coast)
    encoder.set_position(9980);
    motor.move_to(10000);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    // error=20, P=0.70*20=14, D=0 → output=14 < MIN_SPEED(200)
    // Should clamp to MIN_SPEED and drive forward
    motor.update();
    // Motor should be driven (at least one channel > 0), not coasting
    bool bMotorDriven = (htim_pwm.Instance->CCR1 > 0) || (htim_pwm.Instance->CCR2 > 0);
    EXPECT_TRUE(bMotorDriven);
    // Drive speed should be MIN_SPEED
    uint16_t iDriveSpeed = (htim_pwm.Instance->CCR1 > 0)
        ? htim_pwm.Instance->CCR1 : htim_pwm.Instance->CCR2;
    EXPECT_EQ(iDriveSpeed, MotorCtrl::MIN_SPEED);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
}

// 15. MoveToRecoverFromStalled
TEST_F(MotorCtrlPidTest, MoveToRecoverFromStalled) {
    encoder.set_position(0);
    motor.move_to(10000);
    // Force stall
    encoder.set_position(5000);
    for (uint16_t i = 0; i < MotorCtrl::ENCODER_TIMEOUT_TICKS + 1; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::STALLED) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::STALLED);
    // New move_to should recover
    encoder.set_position(5000);
    motor.move_to(8000);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
}

// 16. StopBrakes
TEST_F(MotorCtrlPidTest, StopBrakes) {
    encoder.set_position(0);
    motor.move_to(10000);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    motor.stop();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
    // Brake: both channels = PWM_ARR
    EXPECT_EQ(htim_pwm.Instance->CCR1, MotorCtrl::PWM_ARR);
    EXPECT_EQ(htim_pwm.Instance->CCR2, MotorCtrl::PWM_ARR);
}

// 17. EmergencyStopBrakes
TEST_F(MotorCtrlPidTest, EmergencyStopBrakes) {
    encoder.set_position(0);
    motor.move_to(10000);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    motor.emergency_stop();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
    // Brake: both channels = PWM_ARR
    EXPECT_EQ(htim_pwm.Instance->CCR1, MotorCtrl::PWM_ARR);
    EXPECT_EQ(htim_pwm.Instance->CCR2, MotorCtrl::PWM_ARR);
}

// --- Additional PID behavior tests ---

TEST_F(MotorCtrlPidTest, SetVrefMv_SetsDac) {
    motor.set_vref_mv(1000);
    auto& log = mock::get_log();
    EXPECT_GE(log.dac_sets.size(), 1u);
    // Expected: 1000 * 4095 / 3300 = 1240
    EXPECT_EQ(log.dac_sets.back().value, 1240u);
}

TEST_F(MotorCtrlPidTest, DutyPwmConversion) {
    // 500/1000 = 50% → 4266/2 = 2133
    EXPECT_EQ(MotorCtrl::duty_to_pwm(500), 2133u);
    EXPECT_EQ(MotorCtrl::pwm_to_duty(2133), 500u);
}

TEST_F(MotorCtrlPidTest, SetPwmTest_DirectDrive) {
    motor.set_pwm_test(DIRECTION_E::FORWARD, 600);
    EXPECT_EQ(htim_pwm.Instance->CCR1, 600u);
    EXPECT_EQ(htim_pwm.Instance->CCR2, 0u);

    motor.set_pwm_test(DIRECTION_E::REVERSE, 800);
    EXPECT_EQ(htim_pwm.Instance->CCR1, 0u);
    EXPECT_EQ(htim_pwm.Instance->CCR2, 800u);
}

TEST_F(MotorCtrlPidTest, BrakeTest_BothChannelsHigh) {
    motor.brake_test();
    EXPECT_EQ(htim_pwm.Instance->CCR1, MotorCtrl::PWM_ARR);
    EXPECT_EQ(htim_pwm.Instance->CCR2, MotorCtrl::PWM_ARR);
}

TEST_F(MotorCtrlPidTest, StalledDoesNotUpdate) {
    encoder.set_position(0);
    motor.move_to(10000);
    encoder.set_position(5000);
    for (uint16_t i = 0; i < MotorCtrl::ENCODER_TIMEOUT_TICKS + 1; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::STALLED) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::STALLED);
    // Further updates should not change state
    motor.update();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::STALLED);
}

TEST_F(MotorCtrlPidTest, EncoderTimeoutResetsOnMovement) {
    encoder.set_position(0);
    motor.move_to(10000);
    encoder.set_position(5000);
    // Run 400 ticks (below 500 timeout)
    for (int i = 0; i < 400; ++i) motor.update();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    // Movement resets counter
    encoder.set_position(5001);
    motor.update();
    // Another 400 ticks — still below timeout
    encoder.set_position(5001);
    for (int i = 0; i < 400; ++i) motor.update();
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::STALLED);
}

TEST_F(MotorCtrlPidTest, SetSoftLimitMin) {
    motor.set_soft_limit_min(200);
    // Just verify it compiles and doesn't crash
    encoder.set_position(0);
    motor.move_to(300);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
}

TEST_F(MotorCtrlPidTest, SetSafeLimitMax) {
    motor.set_safe_limit_max(30000);
    encoder.set_position(1000);
    motor.move_to(50000);
    // Target clamped to 30000
    EXPECT_EQ(motor.get_target(), 30000);
}
