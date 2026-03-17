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
        table.set_total_range(206821);
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
    EXPECT_EQ(table.get_position(6), 0);
}

TEST_F(ZoomTableTest, GetPosition_LastEntry) {
    int32_t pos = table.get_position(70);
    EXPECT_GT(pos, 190000);
    EXPECT_LT(pos, 210000);
}

TEST_F(ZoomTableTest, GetPosition_InvalidZoom_ReturnsNeg1) {
    EXPECT_EQ(table.get_position(99), -1);
}

TEST_F(ZoomTableTest, GetNearestZoom) {
    EXPECT_EQ(table.get_nearest_zoom(0), 6);
    EXPECT_EQ(table.get_nearest_zoom(5000), 6);
    EXPECT_EQ(table.get_nearest_zoom(15000), 8);
}

TEST_F(ZoomTableTest, GetNextZoom_Forward) {
    EXPECT_EQ(table.get_next_zoom(6, 1), 8);
    EXPECT_EQ(table.get_next_zoom(60, 1), 70);
}

TEST_F(ZoomTableTest, GetNextZoom_AtMax_Clamps) {
    EXPECT_EQ(table.get_next_zoom(70, 1), 70);
}

TEST_F(ZoomTableTest, GetNextZoom_Backward) {
    EXPECT_EQ(table.get_next_zoom(10, -1), 8);
    EXPECT_EQ(table.get_next_zoom(6, -1), 6);
}

TEST_F(ZoomTableTest, GetNextZoom_MultiStep) {
    EXPECT_EQ(table.get_next_zoom(6, 3), 12);
}

TEST_F(ZoomTableTest, SetEntry_FactoryMode) {
    table.erase_all();
    EXPECT_EQ(table.get_entry_count(), 0u);
    table.set_entry(10, 3600);
    EXPECT_EQ(table.get_entry_count(), 1u);
    EXPECT_TRUE(table.is_valid_zoom(10));
}

TEST_F(ZoomTableTest, AngleToPositionConversion) {
    table.set_total_range(206821);
    table.erase_all();
    table.set_entry(25, 18000);
    int32_t pos = table.get_position(25);
    EXPECT_NEAR(pos, 103410, 2);
}

TEST_F(ZoomTableTest, MinMaxZoom) {
    EXPECT_EQ(table.get_min_zoom(), 6);
    EXPECT_EQ(table.get_max_zoom(), 70);
}
