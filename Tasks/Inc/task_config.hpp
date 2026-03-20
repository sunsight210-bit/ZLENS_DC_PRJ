// Tasks/Inc/task_config.hpp
#pragma once
#include <cstdint>

#ifdef BUILD_TESTING
#include "mock_freertos.hpp"
#else
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#endif

namespace zlens {

// --- Work mode command codes (Protocol v2.5) ---
namespace cmd {
    // 0x0X: Control commands
    constexpr uint8_t HOMING         = 0x01;
    constexpr uint8_t FORCE_STOP     = 0x02;
    constexpr uint8_t GET_STALL_CNT  = 0x03;
    // 0x1X: Zoom commands
    constexpr uint8_t SET_ZOOM       = 0x10;
    constexpr uint8_t ZOOM_INC       = 0x11;
    constexpr uint8_t ZOOM_DEC       = 0x12;
    // 0x2X: Query commands
    constexpr uint8_t QUERY_ZOOM     = 0x20;
    constexpr uint8_t QUERY_STATUS   = 0x21;
    constexpr uint8_t QUERY_SPEED    = 0x22;
    constexpr uint8_t QUERY_TYPE     = 0x23;
    constexpr uint8_t QUERY_RANGE    = 0x24;
    constexpr uint8_t QUERY_VERSION  = 0x25;
    // 0x3X: Cycle zoom
    constexpr uint8_t CYCLE_START    = 0x30;
    constexpr uint8_t CYCLE_STOP     = 0x31;
    // 0x6X: Self-test
    constexpr uint8_t SELF_TEST      = 0x60;
    // 0xFX: Mode switch
    constexpr uint8_t SWITCH_FACTORY = 0xFA;
} // namespace cmd

// --- Factory mode command codes ---
namespace fcmd {
    constexpr uint8_t SET_ANGLE = 0xF1;
} // namespace fcmd

// --- Response command bytes (Protocol v2.5 Table 2) ---
namespace rsp_cmd {
    constexpr uint8_t HOMING_DONE = 0x01;
    constexpr uint8_t ARRIVED     = 0x02;
    constexpr uint8_t REQ_INVALID = 0x03;
    constexpr uint8_t ZOOM        = 0x10;
    constexpr uint8_t STATUS      = 0x11;
    constexpr uint8_t SPEED       = 0x12;
    constexpr uint8_t TYPE        = 0x13;
    constexpr uint8_t MIN_ZOOM    = 0x14;
    constexpr uint8_t MAX_ZOOM    = 0x15;
    constexpr uint8_t VERSION     = 0x16;
    constexpr uint8_t ERR_PARAM   = 0xE0;
    constexpr uint8_t STALL_STOP  = 0xE1;
    constexpr uint8_t OVERCURRENT = 0xE2;
    constexpr uint8_t STALL_COUNT = 0xE3;
} // namespace rsp_cmd

// --- Response param constants ---
namespace rsp {
    constexpr uint16_t LENS_TYPE          = 0x0004;
    constexpr uint16_t OK                 = 0x0000;
    constexpr uint16_t HOMING_DONE_PARAM  = 0x000F;
    constexpr uint16_t ARRIVED_PARAM      = 0x000A;
    constexpr uint16_t REQ_INVALID_PARAM  = 0x000E;
    constexpr uint16_t FW_VERSION         = 0x1000;  // v1.000
    constexpr uint16_t DEFAULT_SPEED_KHZ  = 15;
    constexpr uint16_t POWER_DOWN         = 0x0005;
} // namespace rsp

// --- Save reason codes ---
namespace save_reason {
    constexpr uint8_t ARRIVED    = 0;
    constexpr uint8_t PERIODIC   = 1;
    constexpr uint8_t POWER_DOWN = 2;
    constexpr uint8_t STALL      = 3;
} // namespace save_reason

// Command message from CommTask to MotorTask
struct CMD_MESSAGE_S {
    uint8_t cmd;
    uint16_t param;
};

// Response message from MotorTask to CommTask
struct RSP_MESSAGE_S {
    uint8_t cmd;
    uint16_t param;
};

// Save message from MotorTask to StorageTask
struct SAVE_MESSAGE_S {
    int32_t position;
    uint16_t zoom_x10;
    uint8_t reason; // see save_reason namespace
    int16_t backlash_counts;    // 0 = don't update
    uint8_t backlash_valid;     // 0 = don't update, 0xFF = calibrated
};

// Task priorities
constexpr uint32_t MOTOR_TASK_PRIORITY   = 4;
constexpr uint32_t COMM_TASK_PRIORITY    = 3;
constexpr uint32_t STORAGE_TASK_PRIORITY = 2;
constexpr uint32_t MONITOR_TASK_PRIORITY = 1;

// Stack sizes (words)
constexpr uint32_t MOTOR_TASK_STACK   = 512;
constexpr uint32_t COMM_TASK_STACK    = 256;
constexpr uint32_t STORAGE_TASK_STACK = 256;
constexpr uint32_t MONITOR_TASK_STACK = 384;

// Global queue handles (defined in main.cpp, extern here)
extern QueueHandle_t g_cmdQueue;
extern QueueHandle_t g_rspQueue;
extern QueueHandle_t g_saveQueue;
extern volatile bool g_bSpiEmergency;
extern volatile bool g_bUartSelfTestReq;

} // namespace zlens
