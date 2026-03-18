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
    ZoomTable zoom;

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
        zoom.init();
        zoom.load_defaults();

        selfTest.init(&pm, &fram, &encoder, &motor, &stall, &zoom,
                       &iAdcVoltage, &iAdcCurrent);
        iTick = 0;
    }

    uint32_t tick() { return iTick++; }

    // Advance through voltage, baseline, fram_rw, encoder_dir to reach HOMING_START
    // Requires: spi_rx_buffer set with FRAM_TEST_BYTE
    void advance_to_homing_start() {
        selfTest.start();
        selfTest.step(tick()); // VOLTAGE -> BASELINE
        selfTest.step(tick()); // BASELINE -> FRAM_RW
        selfTest.step(tick()); // FRAM_RW -> ENCODER_DIR_START
        selfTest.step(tick()); // ENCODER_DIR_START -> ENCODER_DIR_WAIT

        // Simulate encoder moving to target
        encoder.set_position(SelfTest::ENCODER_DIR_MOVE);
        selfTest.step(tick()); // ENCODER_DIR_WAIT -> motor IDLE, dir OK -> HOMING_START

        ASSERT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::HOMING_START);
    }

    // Advance through homing to reach RANGE_CHECK
    void advance_to_range_check(int32_t iTotalRange) {
        advance_to_homing_start();
        selfTest.step(tick()); // HOMING_START -> HOMING_WAIT
        selfTest.notify_homing_done(true, iTotalRange);
        selfTest.step(tick()); // HOMING_WAIT -> RANGE_CHECK
        ASSERT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::RANGE_CHECK);
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
    selfTest.step(tick()); // encoder_dir_wait -> motor IDLE, delta OK -> HOMING_START

    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::HOMING_START);
    EXPECT_TRUE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::ENCODER_DIR)]);
    EXPECT_FALSE(selfTest.get_result().bEncoderCompensated);
}

TEST_F(SelfTestTest, EncoderDir_Compensated) {
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};
    encoder.set_position(5000);
    selfTest.init(&pm, &fram, &encoder, &motor, &stall, &zoom,
                   &iAdcVoltage, &iAdcCurrent);
    selfTest.start();
    selfTest.step(tick()); // voltage
    selfTest.step(tick()); // baseline
    selfTest.step(tick()); // fram_rw
    selfTest.step(tick()); // encoder_dir_start (start=5000, target=7000)

    // Simulate encoder going backwards (wrong polarity)
    encoder.set_position(4000);
    motor.emergency_stop();
    selfTest.step(tick()); // encoder_dir_wait -> delta=-1000 < threshold -> ENCODER_DIR_REVERSE

    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::ENCODER_DIR_REVERSE);

    // After compensation, encoder moves correctly
    encoder.set_position(0);
    selfTest.step(tick()); // encoder_dir_reverse -> starts new move from 0
    int32_t iTarget = motor.get_target();
    encoder.set_position(iTarget);
    selfTest.step(tick()); // reverse_wait -> motor IDLE, delta OK -> pass

    EXPECT_TRUE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::ENCODER_DIR)]);
    EXPECT_TRUE(selfTest.get_result().bEncoderCompensated);
    EXPECT_EQ(selfTest.get_phase(), SELF_TEST_PHASE_E::HOMING_START);
}

TEST_F(SelfTestTest, EncoderDir_HardwareFail) {
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};
    encoder.set_position(5000);
    selfTest.init(&pm, &fram, &encoder, &motor, &stall, &zoom,
                   &iAdcVoltage, &iAdcCurrent);
    selfTest.start();
    selfTest.step(tick()); // voltage
    selfTest.step(tick()); // baseline
    selfTest.step(tick()); // fram_rw
    selfTest.step(tick()); // encoder_dir_start

    encoder.set_position(4000);
    motor.emergency_stop();
    selfTest.step(tick()); // -> ENCODER_DIR_REVERSE

    // After compensation still wrong
    encoder.set_position(0);
    selfTest.step(tick()); // encoder_dir_reverse -> starts move
    encoder.set_position(-1000);
    motor.emergency_stop();
    bool bDone = selfTest.step(tick()); // reverse_wait -> fail

    EXPECT_TRUE(bDone);
    EXPECT_FALSE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::ENCODER_DIR)]);
}

TEST_F(SelfTestTest, Homing_Pass) {
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};
    advance_to_homing_start();
    selfTest.step(tick()); // HOMING_START -> HOMING_WAIT

    selfTest.notify_homing_done(true, 346292);
    selfTest.step(tick()); // HOMING_WAIT -> sees notification -> RANGE_CHECK

    EXPECT_TRUE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::HOMING)]);
    EXPECT_EQ(selfTest.get_result().iTotalRange, 346292);
}

TEST_F(SelfTestTest, Homing_Timeout) {
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};
    advance_to_homing_start();

    uint32_t iStartTick = iTick;
    selfTest.step(iStartTick); // HOMING_START -> HOMING_WAIT (start tick recorded)
    iTick = iStartTick + 1;

    // Advance past timeout without notification
    bool bDone = selfTest.step(iStartTick + SelfTest::HOMING_TIMEOUT_MS + 1);
    EXPECT_TRUE(bDone);
    EXPECT_FALSE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::HOMING)]);
}

TEST_F(SelfTestTest, RangeCheck_Pass) {
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};
    advance_to_range_check(346292);
    selfTest.step(tick()); // RANGE_CHECK -> LIMITS_CHECK
    EXPECT_TRUE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::RANGE_CHECK)]);
}

TEST_F(SelfTestTest, RangeCheck_Fail_TooSmall) {
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};
    advance_to_range_check(5000);
    bool bDone = selfTest.step(tick());
    EXPECT_TRUE(bDone);
    EXPECT_FALSE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::RANGE_CHECK)]);
}

TEST_F(SelfTestTest, LimitsCheck_Pass) {
    mock::get_log().spi_rx_buffer = {SelfTest::FRAM_TEST_BYTE};
    advance_to_range_check(346292);
    selfTest.step(tick()); // RANGE_CHECK -> LIMITS_CHECK
    selfTest.step(tick()); // LIMITS_CHECK -> FRAM_SAVE
    EXPECT_TRUE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::LIMITS_CHECK)]);
}

TEST_F(SelfTestTest, FramSave_Pass) {
    // Prepare SPI rx buffer: first test_rw readback, then load_params readback
    FRAM_PARAMS_S stExpected{};
    stExpected.magic_number = FramStorage::MAGIC;
    stExpected.version = 1;
    stExpected.homing_done = 1;
    stExpected.position_valid = 0xFF;
    stExpected.total_range = 346292;
    stExpected.crc16 = FramStorage::calc_crc(stExpected);

    std::vector<uint8_t> rx_data;
    rx_data.push_back(SelfTest::FRAM_TEST_BYTE); // for test_rw
    const uint8_t* pBytes = reinterpret_cast<const uint8_t*>(&stExpected);
    rx_data.insert(rx_data.end(), pBytes, pBytes + sizeof(stExpected)); // for load_params
    mock::get_log().spi_rx_buffer = rx_data;

    advance_to_range_check(346292);
    selfTest.step(tick()); // RANGE_CHECK -> LIMITS_CHECK
    selfTest.step(tick()); // LIMITS_CHECK -> FRAM_SAVE
    bool bDone = selfTest.step(tick()); // FRAM_SAVE -> DONE
    EXPECT_TRUE(bDone);
    EXPECT_TRUE(selfTest.get_result().aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::FRAM_SAVE)]);
    EXPECT_TRUE(selfTest.get_result().bAllPassed);
}

TEST_F(SelfTestTest, FullSequence_AllPass) {
    FRAM_PARAMS_S stExpected{};
    stExpected.magic_number = FramStorage::MAGIC;
    stExpected.version = 1;
    stExpected.homing_done = 1;
    stExpected.position_valid = 0xFF;
    stExpected.total_range = 346292;
    stExpected.crc16 = FramStorage::calc_crc(stExpected);

    std::vector<uint8_t> rx_data;
    rx_data.push_back(SelfTest::FRAM_TEST_BYTE);
    const uint8_t* pBytes = reinterpret_cast<const uint8_t*>(&stExpected);
    rx_data.insert(rx_data.end(), pBytes, pBytes + sizeof(stExpected));
    mock::get_log().spi_rx_buffer = rx_data;

    advance_to_range_check(346292);
    selfTest.step(tick()); // RANGE_CHECK -> LIMITS_CHECK
    selfTest.step(tick()); // LIMITS_CHECK -> FRAM_SAVE
    bool bDone = selfTest.step(tick()); // FRAM_SAVE -> DONE
    EXPECT_TRUE(bDone);

    auto& result = selfTest.get_result();
    EXPECT_TRUE(result.bAllPassed);
    EXPECT_EQ(result.iTotalRange, 346292);
    EXPECT_EQ(result.iMeasuredVoltage, 2017);
    EXPECT_FALSE(result.bEncoderCompensated);
}
