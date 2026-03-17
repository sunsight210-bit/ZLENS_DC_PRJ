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
    uint8_t reason; // 0=arrived, 1=periodic, 2=power_down, 3=stall
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
constexpr uint32_t MONITOR_TASK_STACK = 256;

// Global queue handles (defined in main.cpp, extern here)
extern QueueHandle_t g_cmdQueue;
extern QueueHandle_t g_rspQueue;
extern QueueHandle_t g_saveQueue;
extern volatile bool g_bSpiEmergency;

} // namespace zlens
