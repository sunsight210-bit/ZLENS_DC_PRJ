// Tasks/Inc/motor_task.hpp
#pragma once
#include "task_config.hpp"
#include "motor_ctrl.hpp"
#include "encoder.hpp"
#include "stall_detect.hpp"
#include "adc_filter.hpp"
#include "zoom_table.hpp"
#include "fram_storage.hpp"
#include "flash_config.hpp"
#include "system_manager.hpp"

namespace zlens {

class MotorTask {
public:
    enum class TASK_STATE_E {
        IDLE, MOVING,
        HOMING_FAST, HOMING_RETRACT, HOMING_SLOW, HOMING_SETTLE,
        CYCLING,
        STALL_CURRENT_TEST
    };

    void init(MotorCtrl* pMotor, Encoder* pEncoder, StallDetect* pStall,
              ZoomTable* pZoom, FramStorage* pFram, SystemManager* pSm,
              QueueHandle_t cmdQ, QueueHandle_t rspQ, QueueHandle_t saveQ,
              uint16_t* pAdcCurrent);

    void run_once();
    void start_homing();
    bool is_homing_done() const { return m_bHomingDone; }
    void start_cycle(int8_t iStep, uint8_t iDwell_x100ms);
    void stop_cycle();

    TASK_STATE_E get_state() const { return m_eTaskState; }

    static constexpr int32_t HOMING_RETRACT_DISTANCE = homing::RETRACT_DISTANCE;
    static constexpr int32_t HOMING_SETTLE_DISTANCE  = homing::SETTLE_DISTANCE;
    static constexpr int32_t HOMING_FAR_DISTANCE     = homing::FAR_DISTANCE;

    // Settle detection
    static constexpr uint16_t SETTLE_STABLE_COUNT = 100;  // 100ms at 1ms/tick

    // Move timeout & retry
    static constexpr uint16_t MOVE_TIMEOUT_TICKS = 5000;  // 5s total timeout
    static constexpr uint8_t  MAX_RETRY_COUNT = 2;

    void start_stall_current_test();
    void restore_speed(uint16_t iSpeedDuty, uint16_t iMinDuty, uint16_t iMaxDuty);

    // Stall current test constants
    static constexpr uint16_t STALL_TEST_DWELL_MS = 4000;
    static constexpr uint8_t  STALL_TEST_NUM_SPEEDS = 3;
    static constexpr uint16_t STALL_TEST_SPEEDS[3] = {1280, 2133, 3413};  // 30%, 50%, 80% of PWM_ARR
    static constexpr uint16_t STALL_TEST_ZOOM_X10  = 20;    // 2.0x 测试位置
    static constexpr uint16_t STALL_PRINT_INTERVAL = 500;   // ADC 打印间隔 (ms)
    static constexpr uint16_t HEARTBEAT_INTERVAL   = 500;   // 心跳间隔 (ms)

private:
    MotorCtrl* m_pMotor = nullptr;
    Encoder* m_pEncoder = nullptr;
    StallDetect* m_pStall = nullptr;
    ZoomTable* m_pZoom = nullptr;
    FramStorage* m_pFram = nullptr;
    SystemManager* m_pSm = nullptr;

    QueueHandle_t m_cmdQueue = nullptr;
    QueueHandle_t m_rspQueue = nullptr;
    QueueHandle_t m_saveQueue = nullptr;
    uint16_t* m_pAdcCurrent = nullptr;
    AdcFilter m_AdcCurrentFilter;

    TASK_STATE_E m_eTaskState = TASK_STATE_E::IDLE;
    bool m_bHomingDone = false;
    bool m_bStallTestPending = false;

    // Speed state (0x60 group)
    uint16_t m_iSpeedDuty    = rsp::DEFAULT_SPEED_DUTY;
    uint16_t m_iMinSpeedDuty = rsp::DEFAULT_MIN_SPEED_DUTY;
    uint16_t m_iMaxSpeedDuty = rsp::DEFAULT_MAX_SPEED_DUTY;

    // Cycling
    int8_t m_iCycleStep = 1;
    uint8_t m_iCycleDwell_x100ms = 0;
    uint16_t m_iCycleDwellCount = 0;
    uint16_t m_iCurrentZoom = 0;
    bool m_bCycleWaiting = false;

    // Settle detection
    uint16_t m_iSettleCount = 0;
    int32_t m_iSettleLastPos = 0;
    bool m_bSettling = false;

    // Move timeout & retry
    uint16_t m_iMoveTimeout = 0;
    uint8_t  m_iRetryCount = 0;
    int32_t  m_iRetryTarget = 0;

    void reset_settle();
    bool is_settled();
    int32_t get_settled_position() const { return m_iSettleLastPos; }

    // Stall current test sub-states
    enum class STALL_TEST_PHASE_E : uint8_t {
        MOVE_TO_2X, SETTLE_2X,
        START_STALL, STALL_DWELL,
        STOP_AND_RETURN, SETTLE_RETURN
    };
    STALL_TEST_PHASE_E m_eStallTestPhase = STALL_TEST_PHASE_E::MOVE_TO_2X;
    uint8_t m_iStallSpeedIdx = 0;
    uint16_t m_iStallDwellCount = 0;
    uint32_t m_iStallAdcSum = 0;
    uint16_t m_iStallAdcMax = 0;
    uint16_t m_iStallAdcMin = 0xFFFF;
    uint16_t m_iStallAdcSamples = 0;
    uint16_t m_aStallResults[3][3] = {};  // [speed_idx][avg, min, max]

    void dispatch_command(const CMD_MESSAGE_S& stCmd);
    void process_moving();
    void process_homing();
    void process_cycling();
    void process_stall_current_test();
    void print_stall_current_report();
    void handle_stall();
    void handle_overcurrent();
    void handle_power_down();
    void apply_speed_to_motor();
    void save_speed_to_flash();
    void send_response(uint8_t cmd, uint16_t param);
    void send_save(uint8_t reason, uint8_t homing_done = 0,
                   uint8_t position_valid = 0);
};

} // namespace zlens

extern "C" void motor_task_entry(void* params);
