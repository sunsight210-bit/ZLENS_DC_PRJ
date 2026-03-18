// Test/diag/phase4_heat_diag.cpp — 驱动片发热排查：PA6/PA7 拉低 10s → 正反转
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
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern "C" {
    extern TIM_HandleTypeDef htim3;
    extern TIM_HandleTypeDef htim8;
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

// ============================================================
// Phase 1: PA6/PA7 强制拉低 10s（PWM 停止，输出低电平）
//   — 如果此阶段仍然发热，则问题在驱动片或外围电路
//   — 如果不发热，则问题在 PWM 信号
// Phase 2: 正反转运行
// ============================================================

static void diag_task_entry(void* params) {
    (void)params;
    using namespace zlens;

    volatile uint16_t* pAdcCur = &g_aAdcDmaBuf[0];
    volatile uint16_t* pAdcVol = &g_aAdcDmaBuf[1];

    swo_printf("[DIAG] === START: Heat Test ===\n");
    swo_printf("[DIAG] PWM freq = 64MHz / (0+1) / (4266+1) = 15.0 kHz\n");

    // ===================== Phase 1: 正反转 10s =====================
    swo_printf("[DIAG] Phase 1: Motor FWD 5s + REV 5s\n");

    // 启动 PWM
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    MotorCtrl motor;
    Encoder& encoder = g_Encoder;
    motor.init(&htim3, &hdac, &encoder);

    // FWD 5s
    swo_printf("[DIAG] FWD start\n");
    motor.move_to(encoder.get_position() + 500000);
    for (int i = 0; i < 5000; i++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        motor.update();
        if (i % 1000 == 0) {
            HAL_IWDG_Refresh(&hiwdg);
            swo_printf("[DIAG]   t=%ds pos=%ld cur=%u vol=%u\n",
                       i / 1000, static_cast<long>(encoder.get_position()),
                       *pAdcCur, *pAdcVol);
        }
    }
    motor.emergency_stop();
    swo_printf("[DIAG] FWD done pos=%ld\n", static_cast<long>(encoder.get_position()));

    // REV 5s
    swo_printf("[DIAG] REV start\n");
    motor.move_to(encoder.get_position() - 500000);
    for (int i = 0; i < 5000; i++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        motor.update();
        if (i % 1000 == 0) {
            HAL_IWDG_Refresh(&hiwdg);
            swo_printf("[DIAG]   t=%ds pos=%ld cur=%u vol=%u\n",
                       i / 1000, static_cast<long>(encoder.get_position()),
                       *pAdcCur, *pAdcVol);
        }
    }
    motor.emergency_stop();
    swo_printf("[DIAG] REV done pos=%ld\n", static_cast<long>(encoder.get_position()));

    // ===================== Phase 2: PA6/PA7 拉低 10s =====================
    swo_printf("[DIAG] Phase 2: PWM stop, PA6/PA7 = LOW for 10s\n");

    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        HAL_IWDG_Refresh(&hiwdg);
        swo_printf("[DIAG] idle t=%ds cur=%u vol=%u\n", i + 1, *pAdcCur, *pAdcVol);
    }

    swo_printf("[DIAG] === END ===\n");

    // 测试结束，喂狗待机
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        HAL_IWDG_Refresh(&hiwdg);
    }
}

extern "C" void app_init(void) {
    using namespace zlens;

    g_cmdQueue  = xQueueCreate(8, sizeof(CMD_MESSAGE_S));
    g_rspQueue  = xQueueCreate(8, sizeof(RSP_MESSAGE_S));
    g_saveQueue = xQueueCreate(4, sizeof(SAVE_MESSAGE_S));

    g_Motor.init(&htim3, &hdac, &g_Encoder);
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
                           const_cast<uint16_t*>(g_aAdcDmaBuf)),
                       2);

    // 不启动 PWM — 由 diag task 控制
    // 启动 TIM8 编码器
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TIM8_UP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);

    xTaskCreate(diag_task_entry, "diag", 1024, nullptr, 4, nullptr);
    swo_printf("[BOOT] ZLENS_DIAG Phase 4 Heat Test starting...\n");
}
