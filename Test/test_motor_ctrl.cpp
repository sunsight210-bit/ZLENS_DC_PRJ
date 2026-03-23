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
    // Phase 2 — within FINE_DEADZONE(500) of 5000
    encoder.set_position(4600);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlTest, Backlash_ReverseMove_LargeBacklash) {
    motor.set_backlash(997);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    // overshoot = 5000 - 997 - 200 = 3803
    EXPECT_EQ(motor.get_target(), 3803);
}

TEST_F(MotorCtrlTest, Backlash_Phase2_StartsAtPhase2Speed) {
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
    EXPECT_EQ(motor.get_current_speed(), MotorCtrl::PHASE2_SPEED);
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
    // Phase2 should stop within FINE_DEADZONE(500), not DEADZONE(1000)
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
    // Position at 4200 — 800 from target, outside FINE_DEADZONE(500) → keep moving
    encoder.set_position(4200);
    motor.update();
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::IDLE);
    // Position at 4600 — within FINE_DEADZONE(500) of 5000
    encoder.set_position(4600);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

// --- Settle: position verification (replaces old "always IDLE" behavior) ---
// See post-settle verification tests below for new behavior

// --- Encoder timeout tests ---

TEST_F(MotorCtrlTest, EncoderTimeout_MotorStuck_BrakesToSettling) {
    // Motor running but encoder not moving → timeout (2000 ticks) → brake → SETTLING
    motor.move_to(100000);
    // Advance past acceleration
    for (int i = 0; i < 50; ++i) {
        encoder.set_position(i * 100);
        motor.update();
    }
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::IDLE);
    // Now freeze encoder position — simulate stuck motor
    encoder.set_position(5000);
    for (int i = 0; i < 2100; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::SETTLING) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::SETTLING);
}

TEST_F(MotorCtrlTest, EncoderTimeout_ResetsOnMovement) {
    // Encoder timeout counter resets when position changes
    motor.move_to(100000);
    encoder.set_position(5000);
    // Run 1500 ticks with no movement (below timeout of 2000)
    for (int i = 0; i < 1500; ++i) motor.update();
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::SETTLING);
    // Move encoder — resets counter
    encoder.set_position(5001);
    motor.update();
    // Another 1500 ticks — still below timeout
    encoder.set_position(5001);
    for (int i = 0; i < 1500; ++i) motor.update();
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::SETTLING);
}

// --- Soft limit clamp tests ---

TEST_F(MotorCtrlTest, SoftLimit_ClampsOvershootTarget) {
    // 0.6x scenario: target=2048 (soft limit), backlash=512
    // Without clamp: overshoot = 2048 - 512 - 200 = 1336 (below soft limit)
    // With clamp: overshoot = 2048 (soft limit min)
    // Since overshoot >= target → skip compensation
    motor.set_backlash(512);
    motor.set_backlash_enabled(true);
    motor.set_soft_limit_min(2048);
    encoder.set_position(10000);
    motor.move_to(2048);
    // Compensation skipped: target = finalTarget = 2048
    EXPECT_EQ(motor.get_target(), 2048);
    EXPECT_EQ(motor.get_final_target(), 2048);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::REVERSE);
}

TEST_F(MotorCtrlTest, SoftLimit_CompensationWorksAboveLimit) {
    // target=5000, soft limit=2048, backlash=512
    // overshoot = 5000 - 512 - 200 = 4288 > 2048 → compensation normal
    motor.set_backlash(512);
    motor.set_backlash_enabled(true);
    motor.set_soft_limit_min(2048);
    encoder.set_position(10000);
    motor.move_to(5000);
    EXPECT_EQ(motor.get_target(), 4288);
    EXPECT_EQ(motor.get_final_target(), 5000);
}

TEST_F(MotorCtrlTest, SoftLimit_ClampsToLimit_PartialCompensation) {
    // target=3000, soft limit=2048, backlash=512
    // overshoot = 3000 - 512 - 200 = 2288 > 2048 → not clamped, compensation works
    motor.set_backlash(512);
    motor.set_backlash_enabled(true);
    motor.set_soft_limit_min(2048);
    encoder.set_position(10000);
    motor.move_to(3000);
    EXPECT_EQ(motor.get_target(), 2288);
    EXPECT_EQ(motor.get_final_target(), 3000);
}

TEST_F(MotorCtrlTest, SoftLimit_ClampedToLimit_StillCompensates) {
    // target=2500, soft limit=2048, backlash=512
    // overshoot = 2500 - 512 - 200 = 1788 < 2048 → clamped to 2048
    // 2048 < 2500 → compensation still applies with reduced margin
    motor.set_backlash(512);
    motor.set_backlash_enabled(true);
    motor.set_soft_limit_min(2048);
    encoder.set_position(10000);
    motor.move_to(2500);
    EXPECT_EQ(motor.get_target(), 2048);  // clamped to soft limit
    EXPECT_EQ(motor.get_final_target(), 2500);
}

TEST_F(MotorCtrlTest, Phase2_NearSoftLimit_SkipsCompensation) {
    // 0.6x scenario: target exactly at soft limit
    motor.set_backlash(257);
    motor.set_backlash_enabled(true);
    motor.set_soft_limit_min(2048);
    encoder.set_position(829635);  // 7.0x position
    motor.move_to(2048);
    // overshoot = 2048 - 257 - 200 = 1591 < 2048 → clamped to 2048
    // 2048 >= 2048 → skip compensation
    EXPECT_EQ(motor.get_target(), 2048);
    EXPECT_EQ(motor.get_final_target(), 2048);
}

// --- CRAWL speed tests ---

TEST_F(MotorCtrlTest, Crawl_ReducesSpeedNearTarget) {
    // When remaining < CRAWL_DISTANCE(5000), speed should cap at CRAWL_SPEED(320)
    motor.set_backlash_enabled(true);  // CRAWL requires backlash enabled
    motor.move_to(100000);
    // Accelerate to full speed
    for (int i = 0; i < 500; ++i) {
        encoder.set_position(i * 200);
        motor.update();
    }
    // Jump to within CRAWL_DISTANCE of target
    encoder.set_position(96000);  // 4000 from target
    motor.update();
    EXPECT_LE(motor.get_current_speed(), MotorCtrl::CRAWL_SPEED);
}

TEST_F(MotorCtrlTest, Crawl_NotAppliedOutsideCrawlDistance) {
    // When remaining > CRAWL_DISTANCE, speed should not be limited to CRAWL_SPEED
    motor.set_backlash_enabled(true);
    motor.move_to(100000);
    // Accelerate
    for (int i = 0; i < 300; ++i) {
        encoder.set_position(i * 200);
        motor.update();
    }
    // Position well outside CRAWL_DISTANCE
    encoder.set_position(50000);  // 50000 from target
    motor.update();
    EXPECT_GT(motor.get_current_speed(), MotorCtrl::CRAWL_SPEED);
}

TEST_F(MotorCtrlTest, Crawl_NotAppliedWhenBacklashDisabled) {
    // During homing (backlash disabled), CRAWL should NOT activate
    motor.set_backlash_enabled(false);
    motor.move_to(500000);
    // Accelerate without reaching target
    for (int i = 0; i < 200; ++i) {
        encoder.set_position(i * 200);
        motor.update();
    }
    // Jump to within CRAWL_DISTANCE of target
    encoder.set_position(496000);  // 4000 from target
    motor.update();
    // Speed should be > CRAWL_SPEED since backlash is disabled
    EXPECT_GT(motor.get_current_speed(), MotorCtrl::CRAWL_SPEED);
}

TEST_F(MotorCtrlTest, Crawl_AppliesToPhase2) {
    // Phase 2 (backlash forward approach) should also use CRAWL_SPEED
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    // Phase 1: arrive at overshoot (4600)
    encoder.set_position(4600);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::APPROACHING) break;
    }
    // Enter Phase 2
    motor.update();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::CONSTANT);
    // Phase 2 target=5000, pos=4600, remaining=400 < CRAWL_DISTANCE
    // After next update, speed should be capped to CRAWL_SPEED
    encoder.set_position(4600);
    motor.update();
    EXPECT_LE(motor.get_current_speed(), MotorCtrl::CRAWL_SPEED);
}

// --- Post-settle position verification tests ---
// Note: correction only triggers when backlash is enabled (precision positioning)

TEST_F(MotorCtrlTest, Settle_NoCorrectionWhenBacklashDisabled) {
    // During homing (backlash disabled), skip correction even if error is large
    motor.set_backlash_enabled(false);
    motor.move_to(50000);
    encoder.set_position(49500);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::SETTLING) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::SETTLING);
    encoder.set_position(53000);  // large error
    for (int i = 0; i < 200; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);  // no correction, straight to IDLE
}

TEST_F(MotorCtrlTest, Settle_CorrectionTriggered_WhenErrorExceedsTolerance) {
    // After settling, if pos is far from target, correction move should start
    motor.set_backlash_enabled(true);  // correction requires backlash enabled
    motor.move_to(50000);
    encoder.set_position(49500);  // within DEADZONE → triggers brake+settle
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::SETTLING) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::SETTLING);
    // Simulate overshoot: encoder drifts to 53000 during settling (err=3000)
    encoder.set_position(53000);
    // Run through SETTLE_TICKS
    for (int i = 0; i < 200; ++i) {
        motor.update();
        if (motor.get_state() != MOTOR_STATE_E::SETTLING) break;
    }
    // Should NOT be IDLE — correction move should have started
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlTest, Settle_NoCorrectionNeeded_WhenWithinTolerance) {
    // After settling with small error, go directly to IDLE
    motor.set_backlash_enabled(true);
    motor.move_to(50000);
    encoder.set_position(49500);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::SETTLING) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::SETTLING);
    // Position within POSITION_TOLERANCE(500)
    encoder.set_position(50200);
    for (int i = 0; i < 200; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlTest, Settle_MaxCorrectionsLimit) {
    // After MAX_CORRECTIONS(2) attempts, go to IDLE even if still off
    motor.set_backlash_enabled(true);
    motor.move_to(50000);

    // Correction 1: overshoot
    encoder.set_position(49500);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::SETTLING) break;
    }
    encoder.set_position(53000);  // large error
    for (int i = 0; i < 200; ++i) {
        motor.update();
        if (motor.get_state() != MOTOR_STATE_E::SETTLING) break;
    }
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::IDLE);  // correction started

    // Correction 2: still overshoots
    encoder.set_position(49500);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::SETTLING) break;
    }
    encoder.set_position(53000);  // still large error
    for (int i = 0; i < 200; ++i) {
        motor.update();
        if (motor.get_state() != MOTOR_STATE_E::SETTLING) break;
    }
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::IDLE);  // correction 2 started

    // Correction 3 attempt: should give up and go IDLE
    encoder.set_position(49500);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::SETTLING) break;
    }
    encoder.set_position(53000);  // still large error
    for (int i = 0; i < 200; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);  // gave up
}

TEST_F(MotorCtrlTest, Settle_CorrectionCountResetsOnNewMove) {
    // A new move_to() call resets the correction counter
    motor.set_backlash_enabled(true);
    motor.move_to(50000);
    // Force a correction cycle
    encoder.set_position(49500);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::SETTLING) break;
    }
    encoder.set_position(53000);
    for (int i = 0; i < 200; ++i) {
        motor.update();
        if (motor.get_state() != MOTOR_STATE_E::SETTLING) break;
    }
    // Correction started (count=1)
    // Now issue a completely new move — counter should reset
    encoder.set_position(10000);
    motor.move_to(20000);
    // First settle with error should still trigger correction (count was reset)
    encoder.set_position(19500);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::SETTLING) break;
    }
    encoder.set_position(23000);
    for (int i = 0; i < 200; ++i) {
        motor.update();
        if (motor.get_state() != MOTOR_STATE_E::SETTLING) break;
    }
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::IDLE);  // correction triggered (count was reset)
}

TEST_F(MotorCtrlTest, EmergencyStop_ResetsCorrectionCount) {
    motor.set_backlash_enabled(true);
    motor.move_to(50000);
    encoder.set_position(49500);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::SETTLING) break;
    }
    motor.emergency_stop();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
    // After emergency stop, new move should have full correction budget
    encoder.set_position(10000);  // start far from target
    motor.move_to(50000);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::ACCELERATING);
    // Simulate arrival
    encoder.set_position(49500);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::SETTLING) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::SETTLING);
    // Overshoot during settling
    encoder.set_position(53000);
    for (int i = 0; i < 200; ++i) {
        motor.update();
        if (motor.get_state() != MOTOR_STATE_E::SETTLING) break;
    }
    EXPECT_NE(motor.get_state(), MOTOR_STATE_E::IDLE);  // correction works
}
