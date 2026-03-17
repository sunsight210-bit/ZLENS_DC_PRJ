// Test/test_comm_protocol.cpp
#include <gtest/gtest.h>
#include "comm_protocol.hpp"

using namespace zlens;

class CommProtocolTest : public ::testing::Test {
protected:
    CommProtocol comm;
    void SetUp() override { comm.init(); }
};

// --- Frame parsing ---
TEST_F(CommProtocolTest, ParseValidFrame_SetZoom) {
    uint8_t frame[] = {0xA5, 0x10, 0x00, 0x2D, 0xE3, 0x30};
    auto result = comm.parse_work_frame(frame, 6);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, 0x10);
    EXPECT_EQ(result.param, 0x002D);
}

TEST_F(CommProtocolTest, ParseValidFrame_Homing) {
    uint8_t frame[] = {0xA5, 0x01, 0x00, 0x0F, 0x33, 0x2C};
    auto result = comm.parse_work_frame(frame, 6);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, 0x01);
}

TEST_F(CommProtocolTest, RejectBadHeader) {
    uint8_t frame[] = {0xBB, 0x10, 0x00, 0x2D, 0xE3, 0x30};
    auto result = comm.parse_work_frame(frame, 6);
    EXPECT_FALSE(result.valid);
}

TEST_F(CommProtocolTest, RejectBadCRC) {
    uint8_t frame[] = {0xA5, 0x10, 0x00, 0x2D, 0x00, 0x00};
    auto result = comm.parse_work_frame(frame, 6);
    EXPECT_FALSE(result.valid);
}

TEST_F(CommProtocolTest, RejectShortFrame) {
    uint8_t frame[] = {0xA5, 0x10, 0x00};
    auto result = comm.parse_work_frame(frame, 3);
    EXPECT_FALSE(result.valid);
}

// --- Build response frame ---
TEST_F(CommProtocolTest, BuildResponseFrame) {
    uint8_t out[6];
    comm.build_work_frame(0x10, 0x002D, out);
    EXPECT_EQ(out[0], 0xA5);
    EXPECT_EQ(out[1], 0x10);
    EXPECT_EQ(out[2], 0x00);
    EXPECT_EQ(out[3], 0x2D);
    EXPECT_EQ(out[4], 0xE3);
    EXPECT_EQ(out[5], 0x30);
}

// --- Factory mode frame (8 bytes) ---
TEST_F(CommProtocolTest, ParseFactoryFrame) {
    comm.set_factory_mode(true);
    uint8_t frame[8];
    comm.build_factory_frame(0xF1, 0x001E, 0x510E, frame);
    EXPECT_EQ(frame[0], 0xA5);
    EXPECT_EQ(frame[1], 0xF1);
    auto result = comm.parse_factory_frame(frame, 8);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(result.cmd, 0xF1);
    EXPECT_EQ(result.param_high, 0x001E);
    EXPECT_EQ(result.param_low, 0x510E);
}

// --- Mode switching ---
TEST_F(CommProtocolTest, ModeSwitch) {
    EXPECT_FALSE(comm.is_factory_mode());
    comm.set_factory_mode(true);
    EXPECT_TRUE(comm.is_factory_mode());
    comm.set_factory_mode(false);
    EXPECT_FALSE(comm.is_factory_mode());
}
