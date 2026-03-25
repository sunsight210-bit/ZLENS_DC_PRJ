#include <gtest/gtest.h>
#include "pid_ctrl.hpp"

using namespace zlens;

class PidCtrlTest : public ::testing::Test {
protected:
    PidCtrl m_Pid;
    void SetUp() override { m_Pid.reset(0); }
};

TEST_F(PidCtrlTest, PositiveErrorGivesPositiveOutput) {
    int16_t iOutput = m_Pid.compute(1000, 0);
    EXPECT_GT(iOutput, 0);
    EXPECT_EQ(iOutput, 500);  // KP=0.5 * 1000
}

TEST_F(PidCtrlTest, NegativeErrorGivesNegativeOutput) {
    int16_t iOutput = m_Pid.compute(-1000, 0);
    EXPECT_LT(iOutput, 0);
    EXPECT_EQ(iOutput, -500);
}

TEST_F(PidCtrlTest, ZeroErrorGivesZeroOutput) {
    m_Pid.reset(100);
    int16_t iOutput = m_Pid.compute(0, 100);  // D = -0.1*(100-100) = 0
    EXPECT_EQ(iOutput, 0);
}

TEST_F(PidCtrlTest, OutputSaturatesAtMaxSpeed) {
    int16_t iOutput = m_Pid.compute(50000, 0);
    EXPECT_EQ(iOutput, 1200);
    iOutput = m_Pid.compute(-50000, 0);
    EXPECT_EQ(iOutput, -1200);
}

TEST_F(PidCtrlTest, DerivativeOnMeasurementDampsMotion) {
    int16_t iOut1 = m_Pid.compute(100, 0);
    int16_t iOut2 = m_Pid.compute(50, 50);
    // P=25, D=-0.1*(50-0)=-5 → output=20
    EXPECT_EQ(iOut2, 20);
}

TEST_F(PidCtrlTest, ResetWithPositionPreventsDKick) {
    m_Pid.reset(30000);
    int16_t iOutput = m_Pid.compute(100, 30000);
    // P=50, D=-0.1*(30000-30000)=0 → output=50
    EXPECT_EQ(iOutput, 50);
}

TEST_F(PidCtrlTest, ResetToZeroFromHighPosCausesDKick) {
    m_Pid.reset(0);
    int16_t iOutput = m_Pid.compute(100, 30000);
    // P=50, D=-0.1*(30000-0)=-3000 → output=-2950 → clamp -1200
    EXPECT_EQ(iOutput, -1200);
}

TEST_F(PidCtrlTest, IntegralHasNoEffectWhenKiZero) {
    for (int i = 0; i < 1000; ++i) {
        m_Pid.compute(100, 0);
    }
    int16_t iOutput = m_Pid.compute(100, 0);
    EXPECT_EQ(iOutput, 50);
}
