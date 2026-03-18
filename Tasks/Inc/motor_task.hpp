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
        HOMING_REVERSE, HOMING_RETRACT, HOMING_FIND_Z,
        HOMING_FORWARD, HOMING_TO_SOFT_MIN,
        CYCLING
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
    int32_t get_total_range() const { return m_iTotalRange; }
    int32_t get_z_offset() const { return m_iZOffset; }

    static constexpr int32_t SOFT_LIMIT_OFFSET = 200;
    static constexpr int32_t HOMING_RETRACT_DISTANCE = 4096;
    static constexpr int32_t HOMING_FAR_DISTANCE = 1000000;

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

    // Homing
    int32_t m_iZOffset = 0;
    int32_t m_iTotalRange = 0;

    // Cycling
    int8_t m_iCycleStep = 1;
    uint8_t m_iCycleDwell_x100ms = 0;
    uint16_t m_iCycleDwellCount = 0;
    uint16_t m_iCurrentZoom = 0;
    bool m_bCycleWaiting = false;

    void dispatch_command(const CMD_MESSAGE_S& stCmd);
    void process_moving();
    void process_homing();
    void process_cycling();
    void handle_stall();
    void handle_overcurrent();
    void handle_power_down();
    void send_response(uint8_t cmd, uint16_t param);
    void send_save(uint8_t reason);
    int32_t clamp_to_soft_limits(int32_t iTarget) const;
};

} // namespace zlens

extern "C" void motor_task_entry(void* params);
