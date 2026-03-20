// Test/test_fram_storage.cpp
#include <gtest/gtest.h>
#include "fram_storage.hpp"

using namespace zlens;

class FramStorageTest : public ::testing::Test {
protected:
    FramStorage fram;
    SPI_HandleTypeDef hspi;
    void SetUp() override {
        mock::get_log().reset();
        hspi.Instance = SPI2;
        fram.init(&hspi);
    }
};

TEST_F(FramStorageTest, DefaultParams_Invalid) {
    EXPECT_FALSE(fram.is_valid());
}

TEST_F(FramStorageTest, SaveAndLoad) {
    FRAM_PARAMS_S p{};
    p.magic_number = FramStorage::MAGIC;
    p.version = 1;
    p.current_position = 50000;
    p.current_zoom_x10 = 20;
    p.homing_done = 1;
    p.position_valid = 0xFF;
    fram.save_params(p);
    EXPECT_GE(mock::get_log().spi_tx_data.size(), 2u);

    FRAM_PARAMS_S loaded{};
    fram.load_params_from_buffer(p, loaded);
    EXPECT_EQ(loaded.current_position, 50000);
    EXPECT_EQ(loaded.current_zoom_x10, 20);
}

TEST_F(FramStorageTest, EmergencySave_WritesPosition) {
    fram.emergency_save(12345);
    EXPECT_GE(mock::get_log().spi_tx_data.size(), 1u);
}

TEST_F(FramStorageTest, MagicCheck) {
    FRAM_PARAMS_S p{};
    p.magic_number = 0x0000;
    EXPECT_FALSE(FramStorage::check_magic(p));
    p.magic_number = FramStorage::MAGIC;
    EXPECT_TRUE(FramStorage::check_magic(p));
}

TEST_F(FramStorageTest, CRC_Validation) {
    FRAM_PARAMS_S p{};
    p.magic_number = FramStorage::MAGIC;
    p.version = 1;
    p.current_position = 100;
    uint16_t crc = FramStorage::calc_crc(p);
    EXPECT_NE(crc, 0);
    p.crc16 = crc;
    EXPECT_TRUE(FramStorage::verify_crc(p));
    p.current_position = 200; // tamper
    EXPECT_FALSE(FramStorage::verify_crc(p));
}

TEST(FramParamsTest, V2_StructSize_Is60Bytes) {
    EXPECT_EQ(sizeof(zlens::FRAM_PARAMS_S), 60u);
}

TEST(FramParamsTest, V2_BacklashFields_DefaultZero) {
    zlens::FRAM_PARAMS_S stParams{};
    EXPECT_EQ(stParams.backlash_counts, 0);
    EXPECT_EQ(stParams.backlash_valid, 0);
}
