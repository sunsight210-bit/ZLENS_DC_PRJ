// Test/test_power_monitor.cpp
#include <gtest/gtest.h>
#include "power_monitor.hpp"

using namespace zlens;

TEST(PowerMonitor, NormalVoltage_NoPowerDown) {
    PowerMonitor pm;
    pm.init();
    EXPECT_FALSE(pm.is_power_down(3723));
    EXPECT_FALSE(pm.is_power_down(3500));
}

TEST(PowerMonitor, LowVoltage_PowerDown) {
    PowerMonitor pm;
    pm.init();
    EXPECT_TRUE(pm.is_power_down(3000));
    EXPECT_TRUE(pm.is_power_down(0));
}

TEST(PowerMonitor, ThresholdBoundary) {
    PowerMonitor pm;
    pm.init();
    EXPECT_FALSE(pm.is_power_down(3103));
    EXPECT_TRUE(pm.is_power_down(3102));
}

TEST(PowerMonitor, AdcToMillivolts) {
    EXPECT_NEAR(PowerMonitor::adc_to_voltage_mv(3723), 12000, 50);
    EXPECT_NEAR(PowerMonitor::adc_to_voltage_mv(3103), 10000, 50);
}
