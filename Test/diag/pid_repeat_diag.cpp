// Test/diag/pid_repeat_diag.cpp — PID repeatability test
// Build: cmake -B build/diag -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake
//        -DBUILD_DIAG=ON -DDIAG_SOURCE=pid_repeat_diag.cpp
// Purpose: homing → 1.0X ↔ 2.0X 往复 N 趟, SWO 记录每次到位位置, 统计重复精度

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
constexpr uint32_t NUM_LAPS         = 10;

// 目标位置: HOME_OFFSET(128) + angle_x100 * 65536 / 36000
struct ZOOM_TARGET_S {
    const char* pLabel;
    int32_t     iPos;
};
constexpr ZOOM_TARGET_S TARGETS[] = {
    {"1.0X", 13235},  // 7200
    {"2.0X", 27070},  // 14800
    {"3.0X", 38539},  // 21100
    {"4.0X", 47004},  // 25750
    {"5.0X", 53739},  // 29450
    {"6.0X", 59292},  // 32500
    {"7.0X", 63295},  // 34700
};
constexpr uint32_t NUM_TARGETS = sizeof(TARGETS) / sizeof(TARGETS[0]);

// 开环驱动到限位 (用于 homing)
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

// PID move_to + 等到位
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

} // anonymous namespace

static void diag_task_entry(void* params) {
    (void)params;
    using namespace zlens;

    MotorCtrl& motor = g_Motor;
    Encoder& encoder = g_Encoder;

    swo_printf("[DIAG] === PID MULTI-ZOOM REPEATABILITY TEST ===\n");
    swo_printf("[DIAG] Zooms=%lu  Laps=%lu\n",
               static_cast<unsigned long>(NUM_TARGETS),
               static_cast<unsigned long>(NUM_LAPS));

    // === Homing ===
    swo_printf("[HOME] Fast reverse...\n");
    drive_to_stall(motor, encoder, DIRECTION_E::REVERSE, "FAST");
    encoder.set_position(0);
    vTaskDelay(pdMS_TO_TICKS(300));

    swo_printf("[HOME] Retract to 256...\n");
    pid_move_and_wait(motor, encoder, 256, 3000);
    vTaskDelay(pdMS_TO_TICKS(200));

    swo_printf("[HOME] Slow reverse...\n");
    drive_to_stall(motor, encoder, DIRECTION_E::REVERSE, "SLOW");
    encoder.set_position(0);
    vTaskDelay(pdMS_TO_TICKS(300));

    swo_printf("[HOME] Settle to HOME_OFFSET=%d...\n", ZoomTable::HOME_OFFSET);
    pid_move_and_wait(motor, encoder, ZoomTable::HOME_OFFSET, 3000);
    vTaskDelay(pdMS_TO_TICKS(500));

    int32_t iHomePos = encoder.get_position();
    swo_printf("[HOME] Done. pos=%ld\n", static_cast<long>(iHomePos));

    // === 统计数组 ===
    int32_t aMin[NUM_TARGETS], aMax[NUM_TARGETS];
    int64_t aSum[NUM_TARGETS];
    uint32_t aCount[NUM_TARGETS];
    for (uint32_t z = 0; z < NUM_TARGETS; ++z) {
        aMin[z] = 999999; aMax[z] = -999999; aSum[z] = 0; aCount[z] = 0;
    }

    // === 往复测试: 1X→7X→1X→7X... ===
    swo_printf("[DIAG] lap,zoom,target,actual,error\n");
    for (uint32_t iLap = 1; iLap <= NUM_LAPS; ++iLap) {
        // FWD: 1X → 7X
        for (uint32_t z = 0; z < NUM_TARGETS; ++z) {
            HAL_IWDG_Refresh(&hiwdg);
            bool bOk = pid_move_and_wait(motor, encoder, TARGETS[z].iPos, 5000);
            vTaskDelay(pdMS_TO_TICKS(200));
            int32_t iPos = encoder.get_position();
            int32_t iErr = iPos - TARGETS[z].iPos;
            swo_printf("[DIAG] %lu,%s,%ld,%ld,%ld\n",
                       static_cast<unsigned long>(iLap), TARGETS[z].pLabel,
                       static_cast<long>(TARGETS[z].iPos),
                       static_cast<long>(iPos), static_cast<long>(iErr));
            if (bOk) {
                aSum[z] += iPos; aCount[z]++;
                if (iPos < aMin[z]) aMin[z] = iPos;
                if (iPos > aMax[z]) aMax[z] = iPos;
            }
        }
        // REV: 7X → 1X
        for (int32_t z = NUM_TARGETS - 1; z >= 0; --z) {
            HAL_IWDG_Refresh(&hiwdg);
            bool bOk = pid_move_and_wait(motor, encoder, TARGETS[z].iPos, 5000);
            vTaskDelay(pdMS_TO_TICKS(200));
            int32_t iPos = encoder.get_position();
            int32_t iErr = iPos - TARGETS[z].iPos;
            swo_printf("[DIAG] %lu,%s,%ld,%ld,%ld\n",
                       static_cast<unsigned long>(iLap), TARGETS[z].pLabel,
                       static_cast<long>(TARGETS[z].iPos),
                       static_cast<long>(iPos), static_cast<long>(iErr));
            if (bOk) {
                aSum[z] += iPos; aCount[z]++;
                if (iPos < aMin[z]) aMin[z] = iPos;
                if (iPos > aMax[z]) aMax[z] = iPos;
            }
        }
    }

    // === 统计 ===
    swo_printf("\n[DIAG] === RESULTS ===\n");
    for (uint32_t z = 0; z < NUM_TARGETS; ++z) {
        if (aCount[z] > 0) {
            int32_t iAvg = static_cast<int32_t>(aSum[z] / aCount[z]);
            int32_t iRange = aMax[z] - aMin[z];
            swo_printf("[DIAG] %s: target=%ld avg=%ld min=%ld max=%ld range=%ld (%lu)\n",
                       TARGETS[z].pLabel,
                       static_cast<long>(TARGETS[z].iPos),
                       static_cast<long>(iAvg),
                       static_cast<long>(aMin[z]),
                       static_cast<long>(aMax[z]),
                       static_cast<long>(iRange),
                       static_cast<unsigned long>(aCount[z]));
        }
    }
    swo_printf("[DIAG] DEADZONE=%d\n", MotorCtrl::DEADZONE);
    swo_printf("[DIAG] === DONE ===\n");

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

    xTaskCreate(diag_task_entry, "repeat", 1024, nullptr, 4, nullptr);
    swo_printf("[BOOT] ZLENS_DIAG PID Repeat Test starting...\n");
}
