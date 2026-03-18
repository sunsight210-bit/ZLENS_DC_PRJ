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
    EXPECT_EQ(TIM8->CNT, 4464u);
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
    EXPECT_EQ(TIM8->CNT, 0u);
}

TEST_F(EncoderTest, OverflowIncrement) {
    enc.set_position(0);
    enc.handle_overflow(true);
    TIM8->CNT = 100;
    EXPECT_EQ(enc.get_position(), 65536 + 100);
}

TEST_F(EncoderTest, OverflowDecrement) {
    enc.set_position(70000);
    enc.handle_overflow(false);
    TIM8->CNT = 65000;
    EXPECT_EQ(enc.get_position(), 65000);
}

TEST_F(EncoderTest, ZPulseRecord) {
    enc.set_position(1000);
    TIM8->CNT = 1000;
    enc.handle_z_pulse();
    EXPECT_EQ(enc.get_z_position(), 1000);
}

TEST_F(EncoderTest, MultipleOverflows) {
    enc.set_position(0);
    enc.handle_overflow(true);
    enc.handle_overflow(true);
    enc.handle_overflow(true);
    TIM8->CNT = 500;
    EXPECT_EQ(enc.get_position(), 3 * 65536 + 500);
}
