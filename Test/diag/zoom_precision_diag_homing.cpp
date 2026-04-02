// Test/diag/zoom_precision_diag.cpp — 3.0X 编码器偏移精度测试
// Build: cmake -B build/diag -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake
//        -DBUILD_DIAG=ON -DDIAG_SOURCE=zoom_precision_diag.cpp
// Purpose: homing → 走到 3.0X 及其 ±2/±5/±10/±20 count 偏移位置,
//          每个位置 SWO 报告后等待 UART 输入再继续, 供手动测量图像精度

#include "motor_ctrl.hpp"
#include "encoder.hpp"
#include "stall_detect.hpp"
#include "adc_filter.hpp"
#include "power_monitor.hpp"
#include "zoom_table.hpp"
#include "fram_storage.hpp"
#include "comm_protocol.hpp"
#include "system_manager.hpp"
#include "task_config.hpp"
#include "swo_debug.hpp"
#include <cstdio>
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern "C" {
    extern TIM_HandleTypeDef htim3;
    extern TIM_HandleTypeDef htim4;
    extern DAC_HandleTypeDef hdac;
    extern ADC_HandleTypeDef hadc1;
    extern SPI_HandleTypeDef hspi2;
    extern UART_HandleTypeDef huart2;
    extern IWDG_HandleTypeDef hiwdg;
}

namespace zlens {
QueueHandle_t g_cmdQueue = nullptr;
QueueHandle_t g_rspQueue = nullptr;
QueueHandle_t g_saveQueue = nullptr;
volatile bool g_bSpiEmergency = false;
MotorCtrl g_Motor;
Encoder g_Encoder;
StallDetect g_StallDetect;
ZoomTable g_ZoomTable;
FramStorage g_FramStorage;
CommProtocol g_CommProtocol;
SystemManager g_SystemManager;
PowerMonitor g_PowerMonitor;
volatile uint16_t g_aAdcDmaBuf[2] = {0, 0};
} // namespace zlens

namespace {

constexpr uint16_t HOMING_SPEED     = 480;
constexpr uint32_t STALL_TIMEOUT_MS = 500;

// 3.0X 编码器位置: HOME_OFFSET(128) + 21100 * 65536 / 36000 = 38539
constexpr int32_t POS_3X = 38539;

// 测试点列表: 基准 3.0X 排第一, 然后各偏移量 (counts)
constexpr int32_t OFFSETS[] = {0, -100, -50, -20, -10, -5, -2, +2, +5, +10, +20, +50, +100};
constexpr uint32_t NUM_OFFSETS = sizeof(OFFSETS) / sizeof(OFFSETS[0]);

void drive_to_stall(zlens::MotorCtrl& motor, zlens::Encoder& encoder,
                    zlens::DIRECTION_E eDir, const char* pLabel) {
    motor.set_pwm_test(eDir, HOMING_SPEED);
    int32_t iLastPos = encoder.get_position();
    uint32_t iStallCount = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        int32_t iPos = encoder.get_position();
        if (iPos == iLastPos) {
            if (++iStallCount >= STALL_TIMEOUT_MS) break;
        } else {
            iStallCount = 0;
            iLastPos = iPos;
        }
    }
    motor.brake_test();
    swo_printf("[HOME] %s stall pos=%ld\n", pLabel, static_cast<long>(encoder.get_position()));
}

bool pid_move_and_wait(zlens::MotorCtrl& motor, zlens::Encoder& encoder,
                       int32_t iTarget, uint32_t iTimeoutMs) {
    motor.move_to(iTarget);
    for (uint32_t t = 0; t < iTimeoutMs; ++t) {
        vTaskDelay(pdMS_TO_TICKS(1));
        motor.update();
        HAL_IWDG_Refresh(&hiwdg);
        if (motor.get_state() == zlens::MOTOR_STATE_E::IDLE) return true;
        if (motor.get_state() == zlens::MOTOR_STATE_E::STALLED) return false;
    }
    motor.stop();
    return false;
}

// 等待 UART 收到任意字节
void wait_for_uart_key(void) {
    uint8_t iBuf;
    while (HAL_UART_Receive(&huart2, &iBuf, 1, 100) != HAL_OK) {
        HAL_IWDG_Refresh(&hiwdg);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void uart_puts(const char* pStr) {
    HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t*>(pStr),
                      static_cast<uint16_t>(__builtin_strlen(pStr)), 200);
}

// 完整回零流程 (快回零 + 退回 + 慢回零 + 定位到 HOME_OFFSET)
void do_homing(zlens::MotorCtrl& motor, zlens::Encoder& encoder) {
    swo_printf("[HOME] Fast reverse...\n");
    drive_to_stall(motor, encoder, zlens::DIRECTION_E::REVERSE, "FAST");
    encoder.set_position(0);
    vTaskDelay(pdMS_TO_TICKS(300));

    swo_printf("[HOME] Retract to 256...\n");
    pid_move_and_wait(motor, encoder, 256, 3000);
    vTaskDelay(pdMS_TO_TICKS(200));

    swo_printf("[HOME] Slow reverse...\n");
    drive_to_stall(motor, encoder, zlens::DIRECTION_E::REVERSE, "SLOW");
    encoder.set_position(0);
    vTaskDelay(pdMS_TO_TICKS(300));

    swo_printf("[HOME] Settle to HOME_OFFSET=%d...\n", zlens::ZoomTable::HOME_OFFSET);
    pid_move_and_wait(motor, encoder, zlens::ZoomTable::HOME_OFFSET, 3000);
    vTaskDelay(pdMS_TO_TICKS(500));

    swo_printf("[HOME] Done. pos=%ld\n", static_cast<long>(encoder.get_position()));
}

} // anonymous namespace

static void diag_task_entry(void* params) {
    (void)params;
    using namespace zlens;

    MotorCtrl& motor = g_Motor;
    Encoder& encoder = g_Encoder;

    swo_printf("[DIAG] === 3.0X OFFSET PRECISION TEST ===\n");
    swo_printf("[DIAG] 3.0X pos=%ld, offsets: 0,-20,-10,-5,-2,+2,+5,+10,+20\n",
               static_cast<long>(POS_3X));
    uart_puts("\r\n=== 3.0X OFFSET PRECISION TEST ===\r\n");
    uart_puts("Each point: homing -> move to target -> wait for key.\r\n\r\n");

    // === 逐个偏移位置测试 (每次先回零) ===
    char aBuf[128];

    for (uint32_t i = 0; i < NUM_OFFSETS; ++i) {
        int32_t iTarget = POS_3X + OFFSETS[i];

        // --- 回零 ---
        snprintf(aBuf, sizeof(aBuf),
                 "\r\n[%lu/%lu] Homing before offset=%+ld ...\r\n",
                 static_cast<unsigned long>(i + 1),
                 static_cast<unsigned long>(NUM_OFFSETS),
                 static_cast<long>(OFFSETS[i]));
        uart_puts(aBuf);

        swo_printf("[DIAG] === Point %lu/%lu: homing before offset=%+ld ===\n",
                   static_cast<unsigned long>(i + 1),
                   static_cast<unsigned long>(NUM_OFFSETS),
                   static_cast<long>(OFFSETS[i]));

        do_homing(motor, encoder);

        // --- 正向走到目标 ---
        snprintf(aBuf, sizeof(aBuf),
                 "  Moving to 3.0X %+ld counts (target=%ld)...\r\n",
                 static_cast<long>(OFFSETS[i]),
                 static_cast<long>(iTarget));
        uart_puts(aBuf);

        swo_printf("[DIAG] Moving to offset=%+ld target=%ld\n",
                   static_cast<long>(OFFSETS[i]), static_cast<long>(iTarget));

        bool bOk = pid_move_and_wait(motor, encoder, iTarget, 5000);
        vTaskDelay(pdMS_TO_TICKS(300));

        int32_t iActual = encoder.get_position();
        int32_t iErr = iActual - iTarget;

        swo_printf("[DIAG] offset=%+ld target=%ld actual=%ld err=%ld %s\n",
                   static_cast<long>(OFFSETS[i]),
                   static_cast<long>(iTarget),
                   static_cast<long>(iActual),
                   static_cast<long>(iErr),
                   bOk ? "OK" : "FAIL");

        if (OFFSETS[i] == 0) {
            snprintf(aBuf, sizeof(aBuf),
                     "  >> [%lu/%lu] 3.0X BASELINE  actual=%ld err=%ld [%s]\r\n",
                     static_cast<unsigned long>(i + 1),
                     static_cast<unsigned long>(NUM_OFFSETS),
                     static_cast<long>(iActual),
                     static_cast<long>(iErr),
                     bOk ? "OK" : "FAIL");
        } else {
            snprintf(aBuf, sizeof(aBuf),
                     "  >> [%lu/%lu] 3.0X %+ld counts  target=%ld actual=%ld err=%ld [%s]\r\n",
                     static_cast<unsigned long>(i + 1),
                     static_cast<unsigned long>(NUM_OFFSETS),
                     static_cast<long>(OFFSETS[i]),
                     static_cast<long>(iTarget),
                     static_cast<long>(iActual),
                     static_cast<long>(iErr),
                     bOk ? "OK" : "FAIL");
        }
        uart_puts(aBuf);
        uart_puts("  >>> Press any key when done measuring <<<\r\n");

        wait_for_uart_key();
    }

    // === 完成 ===
    swo_printf("[DIAG] === ALL POSITIONS DONE ===\n");
    uart_puts("\r\n=== ALL POSITIONS DONE ===\r\n");

    for (;;) {
        HAL_IWDG_Refresh(&hiwdg);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_init(void) {
    using namespace zlens;

    g_cmdQueue  = xQueueCreate(8, sizeof(CMD_MESSAGE_S));
    g_rspQueue  = xQueueCreate(8, sizeof(RSP_MESSAGE_S));
    g_saveQueue = xQueueCreate(4, sizeof(SAVE_MESSAGE_S));

    g_Motor.init(&htim3, &hdac, &g_Encoder);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
    g_Motor.set_vref_mv(3300);
    g_Encoder.init();
    g_StallDetect.init();
    g_ZoomTable.init();
    g_ZoomTable.load_defaults();
    g_FramStorage.init(&hspi2);
    g_CommProtocol.init();
    g_SystemManager.init();
    g_PowerMonitor.init();

    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1,
                       reinterpret_cast<uint32_t*>(
                           const_cast<uint16_t*>(g_aAdcDmaBuf)), 2);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    xTaskCreate(diag_task_entry, "zprec", 1024, nullptr, 4, nullptr);
    swo_printf("[BOOT] ZLENS_DIAG Zoom Precision Test starting...\n");
}
