// Test/test_flash_config.cpp
#include <gtest/gtest.h>
#include "flash_config.hpp"

using namespace zlens;

TEST(FlashConfigTest, StructSize_Is48Bytes) {
    EXPECT_EQ(sizeof(FLASH_CONFIG_S), 48u);
}

TEST(FlashConfigTest, LoadDefaults_ValidCRC) {
    FLASH_CONFIG_S cfg;
    FlashConfig::load_defaults(cfg);

    EXPECT_EQ(cfg.magic, FlashConfig::MAGIC);
    EXPECT_EQ(cfg.speed_duty, 300);
    EXPECT_EQ(cfg.min_speed_duty, 40);
    EXPECT_EQ(cfg.max_speed_duty, 300);
    EXPECT_EQ(cfg.min_zoom_x10, 6);
    EXPECT_EQ(cfg.max_zoom_x10, 70);
    EXPECT_EQ(cfg.zoom_step_x10, 0);
    EXPECT_EQ(cfg.lens_type, 4);
    EXPECT_TRUE(FlashConfig::verify_crc(cfg));
    EXPECT_TRUE(FlashConfig::check_magic(cfg));
}

TEST(FlashConfigTest, CRC_DetectsTamper) {
    FLASH_CONFIG_S cfg;
    FlashConfig::load_defaults(cfg);
    EXPECT_TRUE(FlashConfig::verify_crc(cfg));

    cfg.speed_duty = 999;
    EXPECT_FALSE(FlashConfig::verify_crc(cfg));
}

TEST(FlashConfigTest, MagicCheck) {
    FLASH_CONFIG_S cfg{};
    EXPECT_FALSE(FlashConfig::check_magic(cfg));
    cfg.magic = FlashConfig::MAGIC;
    EXPECT_TRUE(FlashConfig::check_magic(cfg));
}

TEST(FlashConfigTest, Load_ReturnsFalse_InTestMode) {
    FlashConfig fc;
    FLASH_CONFIG_S cfg;
    EXPECT_FALSE(fc.load(cfg));
}
