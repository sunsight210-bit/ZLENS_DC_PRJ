// Test/test_power_monitor.cpp
#include <gtest/gtest.h>
#include "power_monitor.hpp"

using namespace zlens;

TEST(PowerMonitor, NormalVoltage_NoPowerDown) {
    PowerMonitor pm;
    pm.init();
    EXPECT_FALSE(pm.is_power_down(2017));  // ~12V
    EXPECT_FALSE(pm.is_power_down(1500));  // ~9V
}

TEST(PowerMonitor, LowVoltage_PowerDown) {
    PowerMonitor pm;
    pm.init();
    EXPECT_TRUE(pm.is_power_down(1300));
    EXPECT_TRUE(pm.is_power_down(0));
}

TEST(PowerMonitor, ThresholdBoundary) {
    PowerMonitor pm;
    pm.init();
    EXPECT_FALSE(pm.is_power_down(1394));  // exactly at threshold
    EXPECT_TRUE(pm.is_power_down(1393));   // just below
}

TEST(PowerMonitor, AdcToMillivolts) {
    // 2017 * 24363 / 4095 ≈ 12001
    EXPECT_NEAR(PowerMonitor::adc_to_voltage_mv(2017), 12000, 100);
    // 1394 * 24363 / 4095 ≈ 8293
    EXPECT_NEAR(PowerMonitor::adc_to_voltage_mv(1394), 8300, 100);
}
