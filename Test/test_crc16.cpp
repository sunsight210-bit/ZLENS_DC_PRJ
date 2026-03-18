// Test/test_crc16.cpp
#include <gtest/gtest.h>
#include "crc16.hpp"

TEST(CRC16, KnownVector_A5_01_0000) {
    uint8_t data[] = {0xA5, 0x01, 0x00, 0x0F};
    uint16_t crc = zlens::crc16_modbus(data, 4);
    EXPECT_EQ(crc, 0x2C33);
}

TEST(CRC16, KnownVector_A5_10_002D) {
    uint8_t data[] = {0xA5, 0x10, 0x00, 0x2D};
    uint16_t crc = zlens::crc16_modbus(data, 4);
    EXPECT_EQ(crc, 0x30E3);
}

TEST(CRC16, KnownVector_A5_02_000A) {
    uint8_t data[] = {0xA5, 0x02, 0x00, 0x0A};
    uint16_t crc = zlens::crc16_modbus(data, 4);
    EXPECT_EQ(crc, 0x2F03);
}

TEST(CRC16, EmptyData) {
    uint16_t crc = zlens::crc16_modbus(nullptr, 0);
    EXPECT_EQ(crc, 0xFFFF);
}

TEST(CRC16, SingleByte) {
    uint8_t data[] = {0x00};
    uint16_t crc = zlens::crc16_modbus(data, 1);
    EXPECT_NE(crc, 0xFFFF);
}
