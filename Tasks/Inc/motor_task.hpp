// Tasks/Inc/motor_task.hpp
#pragma once
#include "task_config.hpp"
#include "motor_ctrl.hpp"
#include "encoder.hpp"
#include "stall_detect.hpp"
#include "adc_filter.hpp"
#include "zoom_table.hpp"
#include "fram_storage.hpp"
#include "system_manager.hpp"

namespace zlens {

class MotorTask {
public:
    enum class TASK_STATE_E {
        IDLE, MOVING,
        HOMING_FAST, HOMING_RETRACT, HOMING_SLOW, HOMING_SETTLE,
        CYCLING,
        BACKLASH_MEASURE, ACCURACY_TEST  // for future Tasks 6/7
    };

    void init(MotorCtrl* pMotor, Encoder* pEncoder, StallDetect* pStall,
              ZoomTable* pZoom, FramStorage* pFram, SystemManager* pSm,
              QueueHandle_t cmdQ, QueueHandle_t rspQ, QueueHandle_t saveQ,
              uint16_t* pAdcCurrent);

    void run_once();
    void start_homing(bool bFullDiag = false);
    bool is_homing_done() const { return m_bHomingDone; }
    void start_cycle(int8_t iStep, uint8_t iDwell_x100ms);
    void stop_cycle();

    TASK_STATE_E get_state() const { return m_eTaskState; }

    static constexpr int32_t HOMING_RETRACT_DISTANCE = 4096;
    static constexpr int32_t HOMING_SETTLE_DISTANCE = 800;
    static constexpr int32_t HOMING_FAR_DISTANCE = 1000000;
    static constexpr uint16_t HOMING_FAST_SPEED = MotorCtrl::MAX_SPEED / 2;
    static constexpr uint16_t HOMING_SLOW_SPEED = MotorCtrl::MIN_SPEED;

    // Backlash measurement constants
    static constexpr int32_t BL_MEASURE_MID = 107000;
    static constexpr int32_t BL_REVERSE_DIST = 4096;

    void start_backlash_measure();

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
    bool m_bFullDiagnostics = false;

    // Cycling
    int8_t m_iCycleStep = 1;
    uint8_t m_iCycleDwell_x100ms = 0;
    uint16_t m_iCycleDwellCount = 0;
    uint16_t m_iCurrentZoom = 0;
    bool m_bCycleWaiting = false;

    // Backlash measurement sub-states
    enum class BL_MEASURE_PHASE_E : uint8_t {
        MOVE_TO_MID, REVERSE, FORWARD
    };
    BL_MEASURE_PHASE_E m_eBLPhase = BL_MEASURE_PHASE_E::MOVE_TO_MID;
    int32_t m_iBLRefPos = 0;
    int32_t m_aBLSamples[3] = {};
    uint8_t m_iBLSampleIdx = 0;

    void dispatch_command(const CMD_MESSAGE_S& stCmd);
    void process_moving();
    void process_homing();
    void process_cycling();
    void process_backlash_measure();
    void handle_stall();
    void handle_overcurrent();
    void handle_power_down();
    void send_response(uint8_t cmd, uint16_t param);
    void send_save(uint8_t reason);
};

} // namespace zlens

extern "C" void motor_task_entry(void* params);
