// Test/test_stall_detect.cpp
#include <gtest/gtest.h>
#include "stall_detect.hpp"

using namespace zlens;

class StallDetectTest : public ::testing::Test {
protected:
    StallDetect sd;
    void SetUp() override {
        sd.init();
    }
};

TEST_F(StallDetectTest, InitialState_NoStall) {
    EXPECT_FALSE(sd.is_stalled());
    EXPECT_FALSE(sd.is_overcurrent());
}

TEST_F(StallDetectTest, BelowThreshold_NoStall) {
    sd.start_motor();
    for (int i = 0; i < 250; ++i) sd.update(50, 0, i * 100);
    EXPECT_FALSE(sd.is_stalled());
}

TEST_F(StallDetectTest, BlankingWindow_IgnoresHighCurrent) {
    sd.start_motor();
    for (int i = 0; i < 200; ++i) sd.update(800, 0, i * 100);
    EXPECT_FALSE(sd.is_stalled());
}

TEST_F(StallDetectTest, StallDetected_After1000Consecutive) {
    sd.start_motor();
    for (int i = 0; i < 200; ++i) sd.update(50, 0, i * 100);
    int32_t last_enc = 0;
    for (int i = 0; i < 1000; ++i) {
        sd.update(800, last_enc, (200 + i) * 100);
    }
    EXPECT_TRUE(sd.is_stalled());
}

TEST_F(StallDetectTest, StallResets_OnLowCurrent) {
    sd.start_motor();
    for (int i = 0; i < 200; ++i) sd.update(50, 0, i * 100);
    for (int i = 0; i < 500; ++i) sd.update(800, 0, (200 + i) * 100);
    sd.update(50, 0, 700 * 100);
    for (int i = 0; i < 500; ++i) sd.update(800, 0, (701 + i) * 100);
    EXPECT_FALSE(sd.is_stalled());
}

TEST_F(StallDetectTest, Overcurrent_FastResponse) {
    sd.start_motor();
    for (int i = 0; i < 200; ++i) sd.update(50, 0, i * 100);
    for (int i = 0; i < 100; ++i) sd.update(3500, 0, (200 + i) * 100);
    EXPECT_TRUE(sd.is_overcurrent());
}

TEST_F(StallDetectTest, EncoderStall_NoMovement500ms) {
    sd.start_motor();
    for (int i = 0; i < 200; ++i) sd.update(50, 0, i * 100);
    for (int i = 0; i < 500; ++i) sd.update(800, 0, (200 + i) * 100);
    EXPECT_TRUE(sd.encoder_stalled());
}

TEST_F(StallDetectTest, DirectionDetection) {
    sd.set_direction(StallDetect::Direction::FORWARD);
    EXPECT_EQ(sd.get_stall_limit_type(), StallDetect::LimitType::LIMIT_MAX);
    sd.set_direction(StallDetect::Direction::REVERSE);
    EXPECT_EQ(sd.get_stall_limit_type(), StallDetect::LimitType::LIMIT_MIN);
}

TEST_F(StallDetectTest, Reset_ClearsState) {
    sd.start_motor();
    for (int i = 0; i < 200; ++i) sd.update(50, 0, i * 100);
    for (int i = 0; i < 1000; ++i) sd.update(800, 0, (200 + i) * 100);
    EXPECT_TRUE(sd.is_stalled());
    sd.reset();
    EXPECT_FALSE(sd.is_stalled());
}
