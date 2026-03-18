// Test/test_adc_filter.cpp
#include <gtest/gtest.h>
#include "adc_filter.hpp"

using namespace zlens;

TEST(AdcFilter, Reset_SetsInitialValue) {
    AdcFilter filter;
    filter.reset(1000);
    EXPECT_EQ(filter.get_filtered(), 1000);
}

TEST(AdcFilter, Reset_ZeroDefault) {
    AdcFilter filter;
    filter.reset();
    EXPECT_EQ(filter.get_filtered(), 0);
}

TEST(AdcFilter, ConstantInput_ConvergesToValue) {
    AdcFilter filter;
    filter.reset(0);

    // Feed constant 2000 for 64 samples (4x time constant)
    for (int i = 0; i < 64; ++i) {
        filter.update(2000);
    }

    // Should converge close to 2000 (EMA with alpha=1/16 needs ~64 samples for ~98%)
    EXPECT_NEAR(filter.get_filtered(), 2000, 50);
}

TEST(AdcFilter, ConstantInput_ExactConvergence) {
    AdcFilter filter;
    filter.reset(0);

    // Feed constant 4000 for 128 samples
    for (int i = 0; i < 128; ++i) {
        filter.update(4000);
    }

    // After many samples, should be very close
    EXPECT_NEAR(filter.get_filtered(), 4000, 5);
}

TEST(AdcFilter, PwmAliasing_25Percent) {
    // Simulate 25% duty cycle: 25% of samples at 4000, 75% at 0
    // Expected average: ~1000
    AdcFilter filter;
    filter.reset(0);

    for (int cycle = 0; cycle < 64; ++cycle) {
        // 1 sample high, 3 samples low (25% duty)
        filter.update(4000);
        filter.update(0);
        filter.update(0);
        filter.update(0);
    }

    EXPECT_NEAR(filter.get_filtered(), 1000, 100);
}

TEST(AdcFilter, PwmAliasing_50Percent) {
    // Simulate 50% duty cycle: 50% at 4000, 50% at 0
    // Expected average: ~2000
    AdcFilter filter;
    filter.reset(0);

    for (int cycle = 0; cycle < 64; ++cycle) {
        filter.update(4000);
        filter.update(0);
    }

    EXPECT_NEAR(filter.get_filtered(), 2000, 100);
}

TEST(AdcFilter, PwmAliasing_75Percent) {
    // Simulate 75% duty cycle: 75% at 4000, 25% at 0
    // Expected average: ~3000
    AdcFilter filter;
    filter.reset(0);

    for (int cycle = 0; cycle < 64; ++cycle) {
        filter.update(4000);
        filter.update(4000);
        filter.update(4000);
        filter.update(0);
    }

    EXPECT_NEAR(filter.get_filtered(), 3000, 100);
}

TEST(AdcFilter, ResetAfterUse_ClearsState) {
    AdcFilter filter;
    filter.reset(0);

    for (int i = 0; i < 64; ++i) filter.update(4000);
    EXPECT_GT(filter.get_filtered(), 3000);

    filter.reset(0);
    EXPECT_EQ(filter.get_filtered(), 0);
}

TEST(AdcFilter, StepResponse_MonotonicallyIncreasing) {
    AdcFilter filter;
    filter.reset(0);

    uint16_t iPrev = 0;
    for (int i = 0; i < 32; ++i) {
        uint16_t iVal = filter.update(4000);
        EXPECT_GE(iVal, iPrev);
        iPrev = iVal;
    }
}
