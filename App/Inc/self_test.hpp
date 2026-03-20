// App/Inc/self_test.hpp
#pragma once
#include <cstdint>
#include "power_monitor.hpp"
#include "fram_storage.hpp"
#include "encoder.hpp"
#include "motor_ctrl.hpp"
#include "stall_detect.hpp"

#ifdef BUILD_TESTING
#include "mock_hal.hpp"
#else
#include "stm32f1xx_hal.h"
#endif

namespace zlens {

enum class SELF_TEST_ITEM_E : uint8_t {
    VOLTAGE = 0,
    BASELINE,
    FRAM_RW,
    ENCODER_DIR,
    COUNT  // now 4
};

struct SELF_TEST_RESULT_S {
    bool aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::COUNT)];
    uint16_t iMeasuredVoltage;
    uint16_t iMeasuredBaseline;
    bool bEncoderCompensated;
    bool bAllPassed;
};

enum class SELF_TEST_PHASE_E : uint8_t {
    IDLE,
    VOLTAGE,
    BASELINE,
    FRAM_RW,
    ENCODER_DIR_START,
    ENCODER_DIR_WAIT,
    ENCODER_DIR_REVERSE,
    ENCODER_DIR_REVERSE_WAIT,
    DONE
};

class SelfTest {
public:
    static constexpr uint16_t BASELINE_MAX = 50;
    static constexpr int32_t  ENCODER_DIR_MOVE = 2000;
    static constexpr int32_t  ENCODER_DIR_THRESHOLD = 500;
    static constexpr uint32_t ENCODER_DIR_TIMEOUT_MS = 3000;
    static constexpr uint16_t FRAM_TEST_ADDR = 0x00FE;
    static constexpr uint8_t  FRAM_TEST_BYTE = 0xA5;

    void init(PowerMonitor* pPm, FramStorage* pFram, Encoder* pEncoder,
              MotorCtrl* pMotor, StallDetect* pStall,
              uint16_t* pAdcVoltage, uint16_t* pAdcCurrent);

    void start();
    bool step(uint32_t iTick);  // returns true when done
    const SELF_TEST_RESULT_S& get_result() const { return m_stResult; }

    SELF_TEST_PHASE_E get_phase() const { return m_ePhase; }
    bool is_motor_idle() const { return m_pMotor->get_state() == MOTOR_STATE_E::IDLE; }

private:
    PowerMonitor* m_pPm = nullptr;
    FramStorage* m_pFram = nullptr;
    Encoder* m_pEncoder = nullptr;
    MotorCtrl* m_pMotor = nullptr;
    StallDetect* m_pStall = nullptr;
    uint16_t* m_pAdcVoltage = nullptr;
    uint16_t* m_pAdcCurrent = nullptr;

    SELF_TEST_PHASE_E m_ePhase = SELF_TEST_PHASE_E::IDLE;
    SELF_TEST_RESULT_S m_stResult{};

    int32_t m_iEncoderStartPos = 0;
    uint32_t m_iEncoderDirStartTick = 0;
    bool m_bEncoderReversed = false;

    void set_item_pass(SELF_TEST_ITEM_E eItem, bool bPass);
    void finalize();
    void print_report();
};

} // namespace zlens
