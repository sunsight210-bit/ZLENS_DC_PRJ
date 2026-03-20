// Test/test_self_test.cpp
#include <gtest/gtest.h>
#include "self_test.hpp"

using namespace zlens;

class SelfTestTest : public ::testing::Test {
protected:
    SelfTest selfTest;
    PowerMonitor pm;
    FramStorage fram;
    Encoder encoder;
    MotorCtrl motor;
    StallDetect stall;

    SPI_HandleTypeDef hspi;
    TIM_HandleTypeDef htim3;
    DAC_HandleTypeDef hdac;
    uint16_t iAdcVoltage = 2017;
    uint16_t iAdcCurrent = 0;
    uint32_t iTick = 0;

    void SetUp() override {
        mock::get_log().reset();
        mock::get_log().tick_auto_increment = false;
        hspi.Instance = SPI2;
        htim3.Instance = TIM3;
        hdac.Instance = DAC1;

        pm.init();
        fram.init(&hspi);
        encoder.init();
        motor.init(&htim3, &hdac, &encoder);
        stall.init();

        selfTest.init(&pm, &fram, &encoder, &motor, &stall,
                       &iAdcVoltage, &iAdcCurrent);
        iTick = 0;
    }

    uint32_t tick() { return iTick++; }

    // Advance through voltage, baseline, fram_rw to reach ENCODER_DIR_START
    // Then complete encoder dir check (normal direction) to reach DONE
    void advance_to_encoder_dir_done() {
        selfTest.start();
        selfTest.step(tick()); // VOLTAGE -> BASELINE
        selfTest.step(tick()); // BASELINE -> FRAM_RW
        selfTest.step(tick()); // FRAM_RW -> ENCODER_DIR_START
        selfTest.step(tick()); // ENCODER_DIR_START -> ENCODER_DIR_WAIT

        // Simulate encoder moving to target
        encoder.set_position(SelfTest::ENCODER_DIR_MOVE);
        bool bDone = selfTest.step(tick()); // ENCODER_DIR_WAIT -> DONE

        ASSERT_TRUE(bDone);
        ASSERT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::DONE);
    }
};

TEST_F(SelfTestTest, Init_PhaseIsIdle) {
    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::IDLE);
}

TEST_F(SelfTestTest, Start_BecomesVoltage) {
    selfTest.start();
    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::VOLTAGE);
}

TEST_F(SelfTestTest, Voltage_Pass) {
    iAdcVoltage = 2017;
    selfTest.start();
    selfTest.step(tick());
    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::BASELINE);
    EXPECT_TRUE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::VOLTAGE)]);
    EXPECT_EQ(selfTest.get_result().iMeasuredVoltage, 2017);
}

TEST_F(SelfTestTest, Voltage_Fail_LowVoltage) {
    iAdcVoltage = 1300;
    selfTest.start();
    bool bDone = selfTest.step(tick());
    EXPECT_TRUE(bDone);
    EXPECT_FALSE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::VOLTAGE)]);
    EXPECT_FALSE(selfTest.get_result().bAllPassed);
}

TEST_F(SelfTestTest, Baseline_Pass) {
    iAdcCurrent = 10;
    selfTest.start();
    selfTest.step(tick()); // voltage
    selfTest.step(tick()); // baseline
    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::FRAM_RW);
    EXPECT_TRUE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::BASELINE)]);
}

TEST_F(SelfTestTest, Baseline_Fail_HighCurrent) {
    iAdcCurrent = 100;
    selfTest.start();
    selfTest.step(tick()); // voltage
    bool bDone = selfTest.step(tick()); // baseline
    EXPECT_TRUE(bDone);
    EXPECT_FALSE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::BASELINE)]);
}

TEST_F(SelfTestTest, FramRW_Pass) {
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};
    selfTest.start();
    selfTest.step(tick()); // voltage
    selfTest.step(tick()); // baseline
    selfTest.step(tick()); // fram_rw
    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::ENCODER_DIR_START);
    EXPECT_TRUE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::FRAM_RW)]);
}

TEST_F(SelfTestTest, FramRW_Fail) {
    selfTest.start();
    selfTest.step(tick()); // voltage
    selfTest.step(tick()); // baseline
    bool bDone = selfTest.step(tick()); // fram_rw -> 0xFF != 0xA5
    EXPECT_TRUE(bDone);
    EXPECT_FALSE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::FRAM_RW)]);
}

TEST_F(SelfTestTest, EncoderDir_Pass_NormalDirection) {
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};
    selfTest.start();
    selfTest.step(tick()); // voltage
    selfTest.step(tick()); // baseline
    selfTest.step(tick()); // fram_rw
    selfTest.step(tick()); // encoder_dir_start -> motor commanded

    encoder.set_position(SelfTest::ENCODER_DIR_MOVE);
    bool bDone = selfTest.step(tick()); // encoder_dir_wait -> motor IDLE, dir OK -> DONE

    EXPECT_TRUE(bDone);
    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::DONE);
    EXPECT_TRUE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::ENCODER_DIR)]);
    EXPECT_FALSE(selfTest.get_result().bEncoderCompensated);
}

TEST_F(SelfTestTest, EncoderDir_AtPositiveLimit_Pass) {
    // Forward delta=0 (at positive limit), reverse delta < -500 -> direction correct
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};
    encoder.set_position(5000);
    selfTest.init(&pm, &fram, &encoder, &motor, &stall,
                   &iAdcVoltage, &iAdcCurrent);
    selfTest.start();
    selfTest.step(tick()); // voltage
    selfTest.step(tick()); // baseline
    selfTest.step(tick()); // fram_rw
    selfTest.step(tick()); // encoder_dir_start (start=5000, target=7000)

    // Forward doesn't move (at positive limit)
    encoder.set_position(5000);
    motor.emergency_stop();
    selfTest.step(tick()); // encoder_dir_wait -> delta=0 -> ENCODER_DIR_REVERSE

    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::ENCODER_DIR_REVERSE);

    // Reverse probe: motor moves backward, encoder decreases (direction correct)
    selfTest.step(tick()); // encoder_dir_reverse -> starts reverse move from 5000
    encoder.set_position(5000 - SelfTest::ENCODER_DIR_MOVE); // moved to 3000
    bool bDone = selfTest.step(tick()); // reverse_wait -> delta=-2000 < -500 -> PASS -> DONE

    EXPECT_TRUE(bDone);
    EXPECT_TRUE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::ENCODER_DIR)]);
    EXPECT_FALSE(selfTest.get_result().bEncoderCompensated);
    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::DONE);
}

TEST_F(SelfTestTest, EncoderDir_Compensated) {
    // Forward delta=0, reverse delta > 500 -> encoder reversed -> flip polarity
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};
    encoder.set_position(5000);
    selfTest.init(&pm, &fram, &encoder, &motor, &stall,
                   &iAdcVoltage, &iAdcCurrent);
    selfTest.start();
    selfTest.step(tick()); // voltage
    selfTest.step(tick()); // baseline
    selfTest.step(tick()); // fram_rw
    selfTest.step(tick()); // encoder_dir_start (start=5000)

    // Forward doesn't move
    encoder.set_position(5000);
    motor.emergency_stop();
    selfTest.step(tick()); // -> ENCODER_DIR_REVERSE

    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::ENCODER_DIR_REVERSE);

    // Reverse: motor moves backward but encoder increases (polarity wrong)
    selfTest.step(tick()); // encoder_dir_reverse -> starts reverse move
    encoder.set_position(5000 + SelfTest::ENCODER_DIR_MOVE); // encoder goes up (wrong)
    motor.emergency_stop(); // stall detect would stop motor in real HW
    bool bDone = selfTest.step(tick()); // reverse_wait -> delta=+2000 > 500 -> flip -> PASS(compensated) -> DONE

    EXPECT_TRUE(bDone);
    EXPECT_TRUE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::ENCODER_DIR)]);
    EXPECT_TRUE(selfTest.get_result().bEncoderCompensated);
    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::DONE);
}

TEST_F(SelfTestTest, EncoderDir_HardwareFail) {
    // Both directions delta ~= 0 -> hardware fault
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};
    encoder.set_position(5000);
    selfTest.init(&pm, &fram, &encoder, &motor, &stall,
                   &iAdcVoltage, &iAdcCurrent);
    selfTest.start();
    selfTest.step(tick()); // voltage
    selfTest.step(tick()); // baseline
    selfTest.step(tick()); // fram_rw
    selfTest.step(tick()); // encoder_dir_start

    // Forward doesn't move
    encoder.set_position(5000);
    motor.emergency_stop();
    selfTest.step(tick()); // -> ENCODER_DIR_REVERSE

    // Reverse also doesn't move
    selfTest.step(tick()); // encoder_dir_reverse -> starts reverse move
    encoder.set_position(5000); // no movement
    motor.emergency_stop();
    bool bDone = selfTest.step(tick()); // reverse_wait -> |delta|=0 -> FAIL

    EXPECT_TRUE(bDone);
    EXPECT_FALSE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::ENCODER_DIR)]);
}

TEST_F(SelfTestTest, FullSequence_AllPass) {
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};

    selfTest.start();
    selfTest.step(tick()); // VOLTAGE -> BASELINE
    selfTest.step(tick()); // BASELINE -> FRAM_RW
    selfTest.step(tick()); // FRAM_RW -> ENCODER_DIR_START
    selfTest.step(tick()); // ENCODER_DIR_START -> ENCODER_DIR_WAIT

    encoder.set_position(SelfTest::ENCODER_DIR_MOVE);
    bool bDone = selfTest.step(tick()); // ENCODER_DIR_WAIT -> DONE

    EXPECT_TRUE(bDone);

    auto& result = selfTest.get_result();
    EXPECT_TRUE(result.bAllPassed);
    EXPECT_EQ(result.iMeasuredVoltage, 2017);
    EXPECT_FALSE(result.bEncoderCompensated);
}

TEST_F(SelfTestTest, ItemCount_Is4) {
    EXPECT_EQ(static_cast<uint8_t>(SELF_TEST_ITEM_E::COUNT), 4);
}
