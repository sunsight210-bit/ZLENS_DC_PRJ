// Test/test_system_manager.cpp
#include <gtest/gtest.h>
#include "system_manager.hpp"

using namespace zlens;

class SystemManagerTest : public ::testing::Test {
protected:
    SystemManager sm;
    void SetUp() override { sm.init(); }
};

TEST_F(SystemManagerTest, InitialState_IsInit) {
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::INIT);
}

TEST_F(SystemManagerTest, Transition_Init_To_SelfTest) {
    EXPECT_TRUE(sm.transition_to(SYSTEM_STATE_E::SELF_TEST));
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::SELF_TEST);
}

TEST_F(SystemManagerTest, Transition_SelfTest_To_Homing) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    EXPECT_TRUE(sm.transition_to(SYSTEM_STATE_E::HOMING));
}

TEST_F(SystemManagerTest, Transition_Homing_To_Ready) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    EXPECT_TRUE(sm.transition_to(SYSTEM_STATE_E::READY));
}

TEST_F(SystemManagerTest, Transition_Ready_To_Busy) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);
    EXPECT_TRUE(sm.transition_to(SYSTEM_STATE_E::BUSY));
}

TEST_F(SystemManagerTest, Transition_Busy_To_Ready) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);
    sm.transition_to(SYSTEM_STATE_E::BUSY);
    EXPECT_TRUE(sm.transition_to(SYSTEM_STATE_E::READY));
}

TEST_F(SystemManagerTest, Transition_Ready_To_SelfTest) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);
    EXPECT_TRUE(sm.transition_to(SYSTEM_STATE_E::SELF_TEST));
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::SELF_TEST);
}

TEST_F(SystemManagerTest, AnyState_To_Error) {
    EXPECT_TRUE(sm.transition_to(SYSTEM_STATE_E::ERROR_STATE));
}

TEST_F(SystemManagerTest, InvalidTransition_Rejected) {
    EXPECT_FALSE(sm.transition_to(SYSTEM_STATE_E::READY));
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::INIT);
}

TEST_F(SystemManagerTest, IsReady) {
    EXPECT_FALSE(sm.is_ready());
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);
    EXPECT_TRUE(sm.is_ready());
}

TEST_F(SystemManagerTest, StatusCode) {
    sm.transition_to(SYSTEM_STATE_E::SELF_TEST);
    sm.transition_to(SYSTEM_STATE_E::HOMING);
    sm.transition_to(SYSTEM_STATE_E::READY);
    EXPECT_EQ(sm.get_status_code(), 0x0000);

    sm.transition_to(SYSTEM_STATE_E::BUSY);
    EXPECT_EQ(sm.get_status_code(), 0x0001);
}
