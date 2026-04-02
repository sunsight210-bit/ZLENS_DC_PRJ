#pragma once

#ifndef BUILD_TESTING

#include "motor_ctrl.hpp"
#include "encoder.hpp"
#include "stall_detect.hpp"
#include "zoom_table.hpp"
#include "fram_storage.hpp"
#include "flash_config.hpp"
#include "comm_protocol.hpp"
#include "system_manager.hpp"
#include "power_monitor.hpp"
#include "task_config.hpp"

namespace zlens {

// Global App module instances
extern MotorCtrl g_Motor;
extern Encoder g_Encoder;
extern StallDetect g_StallDetect;
extern ZoomTable g_ZoomTable;
extern FramStorage g_FramStorage;
extern FlashConfig g_FlashConfig;
extern CommProtocol g_CommProtocol;
extern SystemManager g_SystemManager;
extern PowerMonitor g_PowerMonitor;

// ADC DMA buffer: [0]=IN10 current sense, [1]=IN8 voltage sense
extern volatile uint16_t g_aAdcDmaBuf[2];

} // namespace zlens

extern "C" void app_init(void);

#endif // BUILD_TESTING
