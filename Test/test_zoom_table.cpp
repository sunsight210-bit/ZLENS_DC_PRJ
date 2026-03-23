// Test/test_zoom_table.cpp
#include <gtest/gtest.h>
#include "mock_hal.hpp"
#include "zoom_table.hpp"

using namespace zlens;

class ZoomTableTest : public ::testing::Test {
protected:
    ZoomTable table;
    void SetUp() override {
        mock::get_log().reset();
        table.init();
        table.load_defaults();
        // TOTAL_RANGE=858592 (214648×4), HOME_OFFSET=2048
    }
};

TEST_F(ZoomTableTest, DefaultTableHas14Entries) {
    EXPECT_EQ(table.get_entry_count(), 14u);
}

TEST_F(ZoomTableTest, ValidZoom_InTable) {
    EXPECT_TRUE(table.is_valid_zoom(6));
    EXPECT_TRUE(table.is_valid_zoom(10));
    EXPECT_TRUE(table.is_valid_zoom(70));
}

TEST_F(ZoomTableTest, InvalidZoom_NotInTable) {
    EXPECT_FALSE(table.is_valid_zoom(7));
    EXPECT_FALSE(table.is_valid_zoom(100));
    EXPECT_FALSE(table.is_valid_zoom(0));
}

TEST_F(ZoomTableTest, GetPosition_FirstEntry) {
    // 0.6x @ 0°: HOME_OFFSET + 0 = 2048
    EXPECT_EQ(table.get_position(6), ZoomTable::HOME_OFFSET);
}

TEST_F(ZoomTableTest, GetPosition_LastEntry) {
    // 7.0x @ 347°: HOME_OFFSET + 34700*858592/36000 = 2048+827587 = 829635
    int32_t pos = table.get_position(70);
    EXPECT_GT(pos, 828000);
    EXPECT_LT(pos, 831000);
}

TEST_F(ZoomTableTest, GetPosition_InvalidZoom_ReturnsNeg1) {
    EXPECT_EQ(table.get_position(99), -1);
}

TEST_F(ZoomTableTest, GetNearestZoom) {
    EXPECT_EQ(table.get_nearest_zoom(0), 6);
    EXPECT_EQ(table.get_nearest_zoom(2048), 6);
    // Midpoint between 0.6x(2048) and 1.0x(173766) ≈ 87907
    EXPECT_EQ(table.get_nearest_zoom(87907), 6);
    EXPECT_EQ(table.get_nearest_zoom(173766), 10);
}

TEST_F(ZoomTableTest, GetNextZoom_Forward) {
    EXPECT_EQ(table.get_next_zoom(6, 1), 10);
    EXPECT_EQ(table.get_next_zoom(60, 1), 65);
}

TEST_F(ZoomTableTest, GetNextZoom_AtMax_Clamps) {
    EXPECT_EQ(table.get_next_zoom(70, 1), 70);
}

TEST_F(ZoomTableTest, GetNextZoom_Backward) {
    EXPECT_EQ(table.get_next_zoom(10, -1), 6);
    EXPECT_EQ(table.get_next_zoom(6, -1), 6);
}

TEST_F(ZoomTableTest, GetNextZoom_MultiStep) {
    // 6 → +3 → index 0+3 = index 3 → zoom_x10=20
    EXPECT_EQ(table.get_next_zoom(6, 3), 20);
}

TEST_F(ZoomTableTest, SetEntry_FactoryMode) {
    table.erase_all();
    EXPECT_EQ(table.get_entry_count(), 0u);
    table.set_entry(10, 3600);
    EXPECT_EQ(table.get_entry_count(), 1u);
    EXPECT_TRUE(table.is_valid_zoom(10));
}

TEST_F(ZoomTableTest, AngleToPositionConversion) {
    table.erase_all();
    table.set_entry(25, 18000);
    int32_t pos = table.get_position(25);
    // HOME_OFFSET + 18000 * 858592 / 36000 = 2048 + 429296 = 431344
    EXPECT_NEAR(pos, 431344, 2);
}

TEST_F(ZoomTableTest, MinMaxZoom) {
    EXPECT_EQ(table.get_min_zoom(), 6);
    EXPECT_EQ(table.get_max_zoom(), 70);
}

TEST_F(ZoomTableTest, TotalRange_IsFixedConstant) {
    ZoomTable zt;
    zt.init();
    zt.load_defaults();
    // 0.6x at 0°: HOME_OFFSET + 0 = 2048
    EXPECT_EQ(zt.get_position(6), ZoomTable::HOME_OFFSET);
    // 7.0x at 347°: HOME_OFFSET + 34700*858592/36000 = 829635
    int32_t iPos70 = zt.get_position(70);
    EXPECT_GT(iPos70, 828000);
    EXPECT_LT(iPos70, 831000);
}
