// Test/test_encoder.cpp
#include <gtest/gtest.h>
#include "mock_hal.hpp"
#include "encoder.hpp"

using namespace zlens;

class EncoderTest : public ::testing::Test {
protected:
    Encoder enc;
    void SetUp() override {
        mock::get_log().reset();
        enc.init();
    }
};

TEST_F(EncoderTest, InitialPositionIsZero) {
    EXPECT_EQ(enc.get_position(), 0);
}

TEST_F(EncoderTest, SetAndGetPosition) {
    enc.set_position(12345);
    EXPECT_EQ(enc.get_position(), 12345);
}

TEST_F(EncoderTest, SetPosition_SplitsOverflowAndCNT) {
    enc.set_position(70000); // > 65536
    EXPECT_EQ(TIM4->CNT, 4464u);
    EXPECT_EQ(enc.get_position(), 70000);
}

TEST_F(EncoderTest, SetPosition_Negative) {
    enc.set_position(-100);
    EXPECT_EQ(enc.get_position(), -100);
}

TEST_F(EncoderTest, Reset) {
    enc.set_position(50000);
    enc.reset();
    EXPECT_EQ(enc.get_position(), 0);
    EXPECT_EQ(TIM4->CNT, 0u);
}

TEST_F(EncoderTest, OverflowIncrement) {
    enc.set_position(0);
    enc.handle_overflow(true);
    TIM4->CNT = 100;
    EXPECT_EQ(enc.get_position(), 65536 + 100);
}

TEST_F(EncoderTest, OverflowDecrement) {
    enc.set_position(70000);
    enc.handle_overflow(false);
    TIM4->CNT = 65000;
    EXPECT_EQ(enc.get_position(), 65000);
}

TEST_F(EncoderTest, ZPulseRecord) {
    enc.set_position(1000);
    TIM4->CNT = 1000;
    enc.handle_z_pulse();
    EXPECT_EQ(enc.get_z_position(), 1000);
}

TEST_F(EncoderTest, MultipleOverflows) {
    enc.set_position(0);
    enc.handle_overflow(true);
    enc.handle_overflow(true);
    enc.handle_overflow(true);
    TIM4->CNT = 500;
    EXPECT_EQ(enc.get_position(), 3 * 65536 + 500);
}

// --- INDEX drift detection tests ---

TEST_F(EncoderTest, FirstIndexSetsBaseline) {
    enc.set_position(1024);
    enc.handle_z_pulse();
    EXPECT_FALSE(enc.is_drift_detected());
}

TEST_F(EncoderTest, NoDriftWithinThreshold) {
    enc.set_position(1024);
    enc.handle_z_pulse();
    enc.set_position(2048);
    enc.handle_z_pulse();
    EXPECT_FALSE(enc.is_drift_detected());
}

TEST_F(EncoderTest, SmallDriftWithinThreshold) {
    enc.set_position(1024);
    enc.handle_z_pulse();
    enc.set_position(2051);  // drift = 3 < 4
    enc.handle_z_pulse();
    EXPECT_FALSE(enc.is_drift_detected());
}

TEST_F(EncoderTest, DriftExceedsThreshold) {
    enc.set_position(1024);
    enc.handle_z_pulse();
    enc.set_position(2053);  // drift = 5 > 4
    enc.handle_z_pulse();
    EXPECT_TRUE(enc.is_drift_detected());
    EXPECT_EQ(enc.get_drift_error(), 5);
}

TEST_F(EncoderTest, ClearDriftFlag) {
    enc.set_position(1024);
    enc.handle_z_pulse();
    enc.set_position(2053);
    enc.handle_z_pulse();
    EXPECT_TRUE(enc.is_drift_detected());
    enc.clear_drift_flag();
    EXPECT_FALSE(enc.is_drift_detected());
}

TEST_F(EncoderTest, ResetIndexTrackingClearsAll) {
    enc.set_position(1024);
    enc.handle_z_pulse();
    enc.set_position(2053);
    enc.handle_z_pulse();
    EXPECT_TRUE(enc.is_drift_detected());
    enc.reset_index_tracking();
    EXPECT_FALSE(enc.is_drift_detected());
    enc.set_position(5000);
    enc.handle_z_pulse();
    EXPECT_FALSE(enc.is_drift_detected());
}

TEST_F(EncoderTest, CumulativeDriftDetection) {
    enc.set_position(1000);
    enc.handle_z_pulse();
    for (int i = 1; i <= 4; ++i) {
        int32_t iExpected = 1000 + i * 1024;
        enc.set_position(iExpected + i);
        enc.handle_z_pulse();
    }
    EXPECT_FALSE(enc.is_drift_detected());
    enc.set_position(1000 + 5 * 1024 + 5);
    enc.handle_z_pulse();
    EXPECT_TRUE(enc.is_drift_detected());
    EXPECT_EQ(enc.get_drift_error(), 5);
}
