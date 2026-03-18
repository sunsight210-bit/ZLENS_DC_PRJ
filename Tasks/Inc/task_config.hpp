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

// --- Work mode command codes ---
namespace cmd {
    constexpr uint8_t HANDSHAKE      = 0x01;
    constexpr uint8_t HOMING         = 0x02;
    constexpr uint8_t FORCE_STOP     = 0x03;
    constexpr uint8_t SET_ZOOM       = 0x10;
    constexpr uint8_t CYCLE_START    = 0x11;
    constexpr uint8_t CYCLE_STOP     = 0x12;
    constexpr uint8_t QUERY_ZOOM     = 0x20;
    constexpr uint8_t QUERY_STATUS   = 0x21;
    constexpr uint8_t QUERY_RANGE    = 0x22;
    constexpr uint8_t QUERY_TYPE     = 0x23;
    constexpr uint8_t SWITCH_FACTORY = 0x30;
    constexpr uint8_t SELF_TEST     = 0x60;
} // namespace cmd

// --- Factory mode command codes ---
namespace fcmd {
    constexpr uint8_t SET_ANGLE = 0xF1;
} // namespace fcmd

// --- Response param constants ---
namespace rsp {
    constexpr uint16_t LENS_TYPE       = 0x0004;
    constexpr uint16_t OK              = 0x0000;
    constexpr uint16_t BUSY            = 0xFFFF;
    constexpr uint16_t STALL_ALARM     = 0x0002;
    constexpr uint16_t OVERCURRENT     = 0x0003;
    constexpr uint16_t POWER_DOWN      = 0x0005;
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

} // namespace zlens
