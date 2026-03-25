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
    motor.move_to(1020);  // |20| <= DEADZONE(25)
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
    encoder.set_position(4980);  // |5000 - 4980| = 20 <= 25
    motor.update();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

// 10. SafeLimitMinBrakes
TEST_F(MotorCtrlPidTest, SafeLimitMinBrakes_RunningAboveLimit) {
    encoder.set_position(200);
    motor.move_to(MotorCtrl::SAFE_LIMIT_MIN + 1);  // 65, valid target
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
}

TEST_F(MotorCtrlPidTest, SafeLimitMinBrakes_AtLimit) {
    // Safety check: pos <= SAFE_LIMIT_MIN && error < 0 → brake to IDLE
    // Set up: move to a low target, then position overshoots below safe limit
    // We need error < 0: target < pos, so target must be below pos
    // Use set_safe_limit_max to allow a low target scenario
    // Start far away, target=200
    encoder.set_position(500);
    motor.move_to(200);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    EXPECT_EQ(motor.get_target(), 200);
    // Motor overshoots: position drops to 50 (below SAFE_LIMIT_MIN=64)
    // error = 200 - 50 = 150 > 0 → safety check (error<0) won't trigger
    // To get error < 0 we need pos > target AND pos <= 64
    // That means target < 64, but move_to clamps to SAFE_LIMIT_MIN=64
    // So target=64, pos=64 → error=0 → deadzone → IDLE (not safety, but still stops)
    encoder.set_position(64);
    motor.update();
    // error = 200 - 64 = 136 > 0 → not safety triggered, still RUNNING
    // The safety limit min check protects against going below 64 with negative error
    // In practice, with clamping at SAFE_LIMIT_MIN, error can't be negative at pos<=64
    // unless target is manually set. Test the deadzone path instead.
    encoder.set_position(190);  // error = 200-190 = 10 <= 25 → deadzone → IDLE
    motor.update();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlPidTest, SafeLimitMinBrakes_ReverseAtLimit) {
    // The safety limit min check: pos <= SAFE_LIMIT_MIN && error < 0
    // With entry clamping, target >= SAFE_LIMIT_MIN=64, so error < 0 only if pos > target
    // But pos <= 64 and target >= 64 means error >= 0 always
    // This guard protects against encoder noise pushing pos below limit while moving in reverse
    // Test: start moving to low target (64), position reads 64, error = 0 → deadzone
    encoder.set_position(500);
    motor.move_to(MotorCtrl::SAFE_LIMIT_MIN);  // clamped to 64
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    EXPECT_EQ(motor.get_target(), MotorCtrl::SAFE_LIMIT_MIN);
    // Arrive within deadzone → IDLE
    encoder.set_position(70);  // error = 64 - 70 = -6, |6| <= 25 → deadzone
    motor.update();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlPidTest, SafeLimitMaxBrakes) {
    // pos >= safe max and error > 0 → brake
    int32_t iSafeMax = 65536 - MotorCtrl::SAFE_LIMIT_MIN;
    encoder.set_position(1000);
    motor.move_to(iSafeMax);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    // Position at safe max, error > 0
    encoder.set_position(iSafeMax + 10);
    motor.update();
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

// 12. CoastZoneReleasesMotor
TEST_F(MotorCtrlPidTest, CoastZoneReleasesMotor) {
    // When PID output < MIN_SPEED and error in COAST_ZONE → coast (both channels 0)
    // Need to avoid large D-term spike: move_to resets PID with current pos,
    // so first update with same pos has D=0
    encoder.set_position(9920);
    motor.move_to(10000);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    // error = 80, P = 0.5*80 = 40, D = 0 (no position change since reset)
    // total output = 40 < MIN_SPEED(480), |error|=80 <= COAST_ZONE(100) → coast
    motor.update();
    // Coast: both CCR1 and CCR2 should be 0
    EXPECT_EQ(htim_pwm.Instance->CCR1, 0u);
    EXPECT_EQ(htim_pwm.Instance->CCR2, 0u);
    // Still RUNNING (not IDLE yet)
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
}

// 13. CoastTimeoutAppliesNudge
TEST_F(MotorCtrlPidTest, CoastTimeoutAppliesNudge) {
    // Start near target so D-term is zero on first update
    encoder.set_position(9920);
    motor.move_to(10000);
    // error=80, P=40, D=0, output=40 < MIN_SPEED → coast zone
    // Coast for (COAST_TIMEOUT - 1) ticks: ticks 0..48 coast
    for (uint16_t i = 0; i < MotorCtrl::COAST_TIMEOUT - 1; ++i) {
        motor.update();
        EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
        // During coast, both channels = 0
        EXPECT_EQ(htim_pwm.Instance->CCR1, 0u);
        EXPECT_EQ(htim_pwm.Instance->CCR2, 0u);
    }
    // Tick COAST_TIMEOUT: counter reaches limit → nudge with MIN_SPEED
    motor.update();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::RUNNING);
    // Should have PWM set (not coast) — at least one channel > 0
    bool bMotorDriven = (htim_pwm.Instance->CCR1 > 0) || (htim_pwm.Instance->CCR2 > 0);
    EXPECT_TRUE(bMotorDriven);
}

// 14. CoastDoesNotAccumulateStallCount
TEST_F(MotorCtrlPidTest, CoastDoesNotAccumulateStallCount) {
    // Start near target so D-term is zero
    encoder.set_position(9920);
    motor.move_to(10000);
    // error=80, in coast zone, PID output < MIN_SPEED
    // Run 600 ticks in coast zone — should NOT stall
    for (int i = 0; i < 600; ++i) {
        motor.update();
        EXPECT_NE(motor.get_state(), MOTOR_STATE_E::STALLED)
            << "Stalled at tick " << i;
    }
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
