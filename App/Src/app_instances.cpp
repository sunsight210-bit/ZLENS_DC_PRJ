#ifndef BUILD_TESTING

#include "app_instances.hpp"
#include "swo_debug.hpp"
#include "motor_task.hpp"
#include "comm_task.hpp"
#include "storage_task.hpp"
#include "monitor_task.hpp"

// CubeMX-generated HAL handles (defined in main.c)
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

// Global queue handles (declared extern in task_config.hpp)
QueueHandle_t g_cmdQueue = nullptr;
QueueHandle_t g_rspQueue = nullptr;
QueueHandle_t g_saveQueue = nullptr;
volatile bool g_bSpiEmergency = false;

// Global App module instances
MotorCtrl g_Motor;
Encoder g_Encoder;
StallDetect g_StallDetect;
ZoomTable g_ZoomTable;
FramStorage g_FramStorage;
FlashConfig g_FlashConfig;
CommProtocol g_CommProtocol;
SystemManager g_SystemManager;
PowerMonitor g_PowerMonitor;

// ADC DMA buffer
volatile uint16_t g_aAdcDmaBuf[2] = {0, 0};

} // namespace zlens

extern "C" void app_init(void) {
    using namespace zlens;

    // Create inter-task queues
    g_cmdQueue  = xQueueCreate(8, sizeof(CMD_MESSAGE_S));
    g_rspQueue  = xQueueCreate(8, sizeof(RSP_MESSAGE_S));
    g_saveQueue = xQueueCreate(4, sizeof(SAVE_MESSAGE_S));

    // Init App modules
    g_Motor.init(&htim3, &hdac, &g_Encoder);
    g_Motor.set_soft_limit_min(ZoomTable::HOME_OFFSET);  // 128
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);   // enable DAC CH2 output
    g_Motor.set_vref_mv(2500);  // A4950 VREF=2.5V, 粗调精调全程固定
    g_Encoder.init();
    g_StallDetect.init();
    g_ZoomTable.init();
    if (!g_ZoomTable.load_from_flash()) {
        swo_printf("[BOOT] ZoomTable: Flash load failed, using defaults\n");
        g_ZoomTable.load_defaults();
    } else {
        swo_printf("[BOOT] ZoomTable: loaded %u entries from Flash (min=%u max=%u)\n",
                   g_ZoomTable.get_entry_count(),
                   g_ZoomTable.get_min_zoom(),
                   g_ZoomTable.get_max_zoom());
    }
    g_FramStorage.init(&hspi2);
    g_CommProtocol.init();
    g_SystemManager.init();
    g_PowerMonitor.init();

    // ADC self-calibration (must be done after HAL_ADC_Init, before start)
    HAL_ADCEx_Calibration_Start(&hadc1);

    // Start ADC DMA continuous conversion
    HAL_ADC_Start_DMA(&hadc1,
                       reinterpret_cast<uint32_t*>(
                           const_cast<uint16_t*>(g_aAdcDmaBuf)),
                       2);

    // Start TIM3 PWM (both channels, initially 0% duty)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // Start TIM4 encoder (AS5311) + enable overflow interrupt for 32-bit position
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    // Create FreeRTOS tasks
    TaskHandle_t hMotorTask = nullptr;
    xTaskCreate(motor_task_entry,   "motor",   MOTOR_TASK_STACK,
                nullptr, MOTOR_TASK_PRIORITY, &hMotorTask);
    xTaskCreate(comm_task_entry,    "comm",    COMM_TASK_STACK,
                nullptr, COMM_TASK_PRIORITY, nullptr);
    xTaskCreate(storage_task_entry, "storage", STORAGE_TASK_STACK,
                nullptr, STORAGE_TASK_PRIORITY, nullptr);
    xTaskCreate(monitor_task_entry, "monitor", MONITOR_TASK_STACK,
                static_cast<void*>(hMotorTask), MONITOR_TASK_PRIORITY, nullptr);

    swo_printf("[BOOT] ZLENS_DC v1.0 starting...\n");
}

#endif // BUILD_TESTING
