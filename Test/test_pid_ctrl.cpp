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
    EXPECT_EQ(iOutput, 700);  // KP=0.70 * 1000
}

TEST_F(PidCtrlTest, NegativeErrorGivesNegativeOutput) {
    int16_t iOutput = m_Pid.compute(-1000, 0);
    EXPECT_LT(iOutput, 0);
    EXPECT_EQ(iOutput, -700);
}

TEST_F(PidCtrlTest, ZeroErrorGivesZeroOutput) {
    m_Pid.reset(100);
    int16_t iOutput = m_Pid.compute(0, 100);  // KD=0, no D-term
    EXPECT_EQ(iOutput, 0);
}

TEST_F(PidCtrlTest, OutputSaturatesAtMaxSpeed) {
    int16_t iOutput = m_Pid.compute(50000, 0);
    EXPECT_EQ(iOutput, 1200);
    iOutput = m_Pid.compute(-50000, 0);
    EXPECT_EQ(iOutput, -1200);
}

TEST_F(PidCtrlTest, DerivativeHasNoEffectWhenKdZero) {
    int16_t iOut1 = m_Pid.compute(100, 0);
    // P=0.70*100=70, D=0 (KD=0) → output=70
    EXPECT_EQ(iOut1, 70);
    int16_t iOut2 = m_Pid.compute(50, 50);
    // P=0.70*50=35, D=0 (KD=0) → output=35
    EXPECT_EQ(iOut2, 35);
}

TEST_F(PidCtrlTest, ResetWithPositionPreventsDKick) {
    m_Pid.reset(30000);
    int16_t iOutput = m_Pid.compute(100, 30000);
    // P=0.70*100=70, D=0 (KD=0) → output=70
    EXPECT_EQ(iOutput, 70);
}

TEST_F(PidCtrlTest, ResetToZeroFromHighPosNoDKick) {
    m_Pid.reset(0);
    int16_t iOutput = m_Pid.compute(100, 30000);
    // P=0.70*100=70, D=0 (KD=0) → output=70
    EXPECT_EQ(iOutput, 70);
}

TEST_F(PidCtrlTest, IntegralHasNoEffectWhenKiZero) {
    for (int i = 0; i < 1000; ++i) {
        m_Pid.compute(100, 0);
    }
    int16_t iOutput = m_Pid.compute(100, 0);
    // P=0.70*100=70, no I/D → output=70
    EXPECT_EQ(iOutput, 70);
}
