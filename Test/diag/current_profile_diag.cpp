// Test/diag/current_profile_diag.cpp — Accurate current profiling diagnostic
// Measures PC0 (CUR) ADC under 3 conditions:
//   1. Motor stopped (baseline)
//   2. Motor running at constant speed (per speed level, FWD + REV)
//   3. Motor stalled against limit (sustained stall current)
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
volatile bool g_bUartSelfTestReq = false;

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

constexpr uint16_t SPEEDS[] = {480, 1200, 2400, 3600};
constexpr uint32_t SPEED_COUNT = sizeof(SPEEDS) / sizeof(SPEEDS[0]);
constexpr int32_t  HOMING_FAR = 1000000;
constexpr int32_t  HOMING_RETRACT = 8000;
constexpr int32_t  SOFT_OFFSET = 2000;
constexpr uint32_t MOVE_TIMEOUT_MS = 15000;
constexpr uint32_t STALL_SAMPLE_MS = 500;    // stall sustained sampling duration
constexpr uint32_t BASELINE_SAMPLES = 50;
constexpr uint32_t BASELINE_INTERVAL = 40;   // ms

struct ADC_STATS_S {
    uint32_t iSum;
    uint16_t iMax;
    uint16_t iMin;
    uint32_t iCount;

    void reset() { iSum = 0; iMax = 0; iMin = 0xFFFF; iCount = 0; }

    void add(uint16_t v) {
        iSum += v;
        if (v > iMax) iMax = v;
        if (v < iMin) iMin = v;
        iCount++;
    }

    uint16_t avg() const { return iCount ? iSum / iCount : 0; }
};

// Convert ADC to mA (R_sense=0.2Ω)
uint32_t adc_to_ma(uint16_t adc) {
    return static_cast<uint32_t>(adc) * 16500 / 4095;
}

// Wait for motor IDLE with safety
bool safe_wait(zlens::MotorCtrl& motor, zlens::Encoder& encoder, uint32_t iTimeoutMs) {
    int32_t iLastPos = encoder.get_position();
    uint32_t iNoChange = 0;
    for (uint32_t t = 0; t < iTimeoutMs; t++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        motor.update();
        if (motor.get_state() == zlens::MOTOR_STATE_E::IDLE) return true;
        int32_t iPos = encoder.get_position();
        if (iPos != iLastPos) { iLastPos = iPos; iNoChange = 0; }
        else if (++iNoChange >= 3000) { motor.emergency_stop(); return false; }
    }
    motor.emergency_stop();
    return false;
}

} // anonymous namespace

static void diag_task_entry(void* params) {
    (void)params;
    using namespace zlens;

    MotorCtrl motor;
    Encoder& encoder = g_Encoder;
    StallDetect stall;
    AdcFilter filter;

    motor.init(&htim3, &hdac, &encoder);
    stall.init();
    filter.reset(0);

    volatile uint16_t* pAdcCur = &g_aAdcDmaBuf[0];
    volatile uint16_t* pAdcVol = &g_aAdcDmaBuf[1];

    swo_printf("\n========== CURRENT PROFILE DIAGNOSTIC ==========\n");

    // ===================== Phase 1: Baseline =====================
    swo_printf("[1/3] Baseline (motor stopped, %lu samples)...\n",
               static_cast<unsigned long>(BASELINE_SAMPLES));
    ADC_STATS_S stBaseline;
    ADC_STATS_S stBaselineVol;
    stBaseline.reset();
    stBaselineVol.reset();
    for (uint32_t i = 0; i < BASELINE_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(BASELINE_INTERVAL));
        HAL_IWDG_Refresh(&hiwdg);
        stBaseline.add(*pAdcCur);
        stBaselineVol.add(*pAdcVol);
    }
    swo_printf("  CUR: avg=%u min=%u max=%u  (%lumA)\n",
               stBaseline.avg(), stBaseline.iMin, stBaseline.iMax,
               adc_to_ma(stBaseline.avg()));
    swo_printf("  VOL: avg=%u  (%lumV)\n",
               stBaselineVol.avg(),
               PowerMonitor::adc_to_voltage_mv(stBaselineVol.avg()));

    // ===================== Phase 2: Homing =====================
    swo_printf("\n[2/3] Homing to find range...\n");

    // Reverse to min limit
    encoder.set_position(0);
    stall.set_direction(StallDetect::Direction::REVERSE);
    stall.start_motor();
    filter.reset(0);
    motor.move_to(-HOMING_FAR);

    for (uint32_t t = 0; t < MOVE_TIMEOUT_MS; t++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        uint16_t iFiltered = filter.update(*pAdcCur);
        stall.update(iFiltered, encoder.get_position(), HAL_GetTick());
        motor.update();
        if (stall.is_stalled() || stall.is_overcurrent()) {
            motor.emergency_stop();
            break;
        }
    }
    motor.emergency_stop();
    stall.reset();
    encoder.set_position(0);
    vTaskDelay(pdMS_TO_TICKS(500));
    HAL_IWDG_Refresh(&hiwdg);

    // Retract
    motor.move_to(HOMING_RETRACT);
    safe_wait(motor, encoder, MOVE_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(300));
    HAL_IWDG_Refresh(&hiwdg);

    // Forward to max limit
    stall.set_direction(StallDetect::Direction::FORWARD);
    stall.start_motor();
    filter.reset(0);
    motor.move_to(HOMING_FAR);

    int32_t iTotalRange = 0;
    for (uint32_t t = 0; t < MOVE_TIMEOUT_MS; t++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        uint16_t iFiltered = filter.update(*pAdcCur);
        stall.update(iFiltered, encoder.get_position(), HAL_GetTick());
        motor.update();
        if (stall.is_stalled() || stall.is_overcurrent()) {
            iTotalRange = encoder.get_position();
            motor.emergency_stop();
            break;
        }
    }
    motor.emergency_stop();
    stall.reset();

    int32_t iSoftMin = SOFT_OFFSET;
    int32_t iSoftMax = iTotalRange - SOFT_OFFSET;
    swo_printf("  Range=%ld counts, soft=[%ld, %ld]\n",
               static_cast<long>(iTotalRange),
               static_cast<long>(iSoftMin),
               static_cast<long>(iSoftMax));

    if (iTotalRange < 10000) {
        swo_printf("  ERROR: range too small, abort\n");
        for (;;) { vTaskDelay(pdMS_TO_TICKS(500)); HAL_IWDG_Refresh(&hiwdg); }
    }

    // Move to soft_min start position
    motor.move_to(iSoftMin);
    safe_wait(motor, encoder, MOVE_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(500));
    HAL_IWDG_Refresh(&hiwdg);

    // ===================== Phase 3: Run current at each speed =====================
    swo_printf("\n[3/3] Run current profiling (constant-speed phase only)...\n");
    swo_printf("  Sampling only in CONSTANT state, skipping accel/decel\n\n");

    // Results storage
    ADC_STATS_S aConstFwd[SPEED_COUNT];
    ADC_STATS_S aConstRev[SPEED_COUNT];
    ADC_STATS_S aAccelFwd[SPEED_COUNT];

    for (uint32_t s = 0; s < SPEED_COUNT; s++) {
        aConstFwd[s].reset();
        aConstRev[s].reset();
        aAccelFwd[s].reset();

        uint16_t iSpeed = SPEEDS[s];
        motor.set_max_speed(iSpeed);

        // --- Forward: soft_min → soft_max ---
        filter.reset(0);
        motor.move_to(iSoftMax);
        swo_printf("  @%u FWD: ", iSpeed);

        for (uint32_t t = 0; t < MOVE_TIMEOUT_MS; t++) {
            vTaskDelay(pdMS_TO_TICKS(1));
            HAL_IWDG_Refresh(&hiwdg);
            motor.update();
            uint16_t iFiltered = filter.update(*pAdcCur);

            MOTOR_STATE_E eState = motor.get_state();
            if (eState == MOTOR_STATE_E::CONSTANT) {
                aConstFwd[s].add(iFiltered);
            } else if (eState == MOTOR_STATE_E::ACCELERATING) {
                aAccelFwd[s].add(iFiltered);
            }

            if (eState == MOTOR_STATE_E::IDLE) break;

            // Encoder stall safety
            if (t > 0 && t % 3000 == 0) {
                int32_t iPos = encoder.get_position();
                if (iPos == encoder.get_position()) {
                    motor.emergency_stop();
                    swo_printf("ENCODER_STALL ");
                    break;
                }
            }
        }

        swo_printf("const: avg=%u min=%u max=%u (%lu samples, %lumA avg)\n",
                   aConstFwd[s].avg(), aConstFwd[s].iMin, aConstFwd[s].iMax,
                   aConstFwd[s].iCount, adc_to_ma(aConstFwd[s].avg()));

        vTaskDelay(pdMS_TO_TICKS(500));
        HAL_IWDG_Refresh(&hiwdg);

        // --- Reverse: soft_max → soft_min ---
        filter.reset(0);
        motor.move_to(iSoftMin);
        swo_printf("  @%u REV: ", iSpeed);

        for (uint32_t t = 0; t < MOVE_TIMEOUT_MS; t++) {
            vTaskDelay(pdMS_TO_TICKS(1));
            HAL_IWDG_Refresh(&hiwdg);
            motor.update();
            uint16_t iFiltered = filter.update(*pAdcCur);

            if (motor.get_state() == MOTOR_STATE_E::CONSTANT) {
                aConstRev[s].add(iFiltered);
            }
            if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
        }

        swo_printf("const: avg=%u min=%u max=%u (%lu samples, %lumA avg)\n",
                   aConstRev[s].avg(), aConstRev[s].iMin, aConstRev[s].iMax,
                   aConstRev[s].iCount, adc_to_ma(aConstRev[s].avg()));

        vTaskDelay(pdMS_TO_TICKS(500));
        HAL_IWDG_Refresh(&hiwdg);
    }

    // Restore max speed
    motor.set_max_speed(MotorCtrl::MAX_SPEED);

    // ===================== Phase 4: Stall current =====================
    swo_printf("\n  Stall current (sustained %lums sampling after stall)...\n",
               static_cast<unsigned long>(STALL_SAMPLE_MS));

    // Move to soft_min first
    motor.move_to(iSoftMin);
    safe_wait(motor, encoder, MOVE_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(500));
    HAL_IWDG_Refresh(&hiwdg);

    // --- Reverse stall (hit min limit) ---
    ADC_STATS_S stStallRev;
    stStallRev.reset();
    filter.reset(0);
    stall.set_direction(StallDetect::Direction::REVERSE);
    stall.start_motor();
    motor.move_to(-HOMING_FAR);

    bool bStallDetected = false;
    uint32_t iStallStartTick = 0;
    for (uint32_t t = 0; t < MOVE_TIMEOUT_MS; t++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        uint16_t iFiltered = filter.update(*pAdcCur);
        stall.update(iFiltered, encoder.get_position(), HAL_GetTick());
        motor.update();

        if (!bStallDetected && (stall.is_stalled() || stall.is_overcurrent())) {
            bStallDetected = true;
            iStallStartTick = t;
            // Don't stop motor — keep sampling stall current
        }
        if (bStallDetected) {
            stStallRev.add(iFiltered);
            if (t - iStallStartTick >= STALL_SAMPLE_MS) {
                motor.emergency_stop();
                break;
            }
        }
    }
    if (!bStallDetected) motor.emergency_stop();
    stall.reset();

    swo_printf("  REV stall: avg=%u min=%u max=%u (%lu samples, %lumA avg)\n",
               stStallRev.avg(), stStallRev.iMin, stStallRev.iMax,
               stStallRev.iCount, adc_to_ma(stStallRev.avg()));

    vTaskDelay(pdMS_TO_TICKS(500));
    HAL_IWDG_Refresh(&hiwdg);

    // Move away from limit
    encoder.set_position(0);
    motor.move_to(HOMING_RETRACT);
    safe_wait(motor, encoder, MOVE_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(500));
    HAL_IWDG_Refresh(&hiwdg);

    // --- Forward stall (hit max limit) ---
    ADC_STATS_S stStallFwd;
    stStallFwd.reset();
    filter.reset(0);
    stall.set_direction(StallDetect::Direction::FORWARD);
    stall.start_motor();
    motor.move_to(HOMING_FAR);

    bStallDetected = false;
    for (uint32_t t = 0; t < MOVE_TIMEOUT_MS; t++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        uint16_t iFiltered = filter.update(*pAdcCur);
        stall.update(iFiltered, encoder.get_position(), HAL_GetTick());
        motor.update();

        if (!bStallDetected && (stall.is_stalled() || stall.is_overcurrent())) {
            bStallDetected = true;
            iStallStartTick = t;
        }
        if (bStallDetected) {
            stStallFwd.add(iFiltered);
            if (t - iStallStartTick >= STALL_SAMPLE_MS) {
                motor.emergency_stop();
                break;
            }
        }
    }
    if (!bStallDetected) motor.emergency_stop();
    stall.reset();

    swo_printf("  FWD stall: avg=%u min=%u max=%u (%lu samples, %lumA avg)\n",
               stStallFwd.avg(), stStallFwd.iMin, stStallFwd.iMax,
               stStallFwd.iCount, adc_to_ma(stStallFwd.avg()));

    // ===================== Summary Table =====================
    swo_printf("\n============ SUMMARY ============\n");
    swo_printf("Condition        ADC avg  ADC min  ADC max  mA avg\n");
    swo_printf("-----------------------------------------------\n");
    swo_printf("Stopped          %-7u  %-7u  %-7u  %lu\n",
               stBaseline.avg(), stBaseline.iMin, stBaseline.iMax,
               adc_to_ma(stBaseline.avg()));

    for (uint32_t s = 0; s < SPEED_COUNT; s++) {
        swo_printf("FWD @%-4u const  %-7u  %-7u  %-7u  %lu\n",
                   SPEEDS[s],
                   aConstFwd[s].avg(), aConstFwd[s].iMin, aConstFwd[s].iMax,
                   adc_to_ma(aConstFwd[s].avg()));
        swo_printf("REV @%-4u const  %-7u  %-7u  %-7u  %lu\n",
                   SPEEDS[s],
                   aConstRev[s].avg(), aConstRev[s].iMin, aConstRev[s].iMax,
                   adc_to_ma(aConstRev[s].avg()));
    }

    swo_printf("Stall REV        %-7u  %-7u  %-7u  %lu\n",
               stStallRev.avg(), stStallRev.iMin, stStallRev.iMax,
               adc_to_ma(stStallRev.avg()));
    swo_printf("Stall FWD        %-7u  %-7u  %-7u  %lu\n",
               stStallFwd.avg(), stStallFwd.iMin, stStallFwd.iMax,
               adc_to_ma(stStallFwd.avg()));
    swo_printf("=================================\n");

    swo_printf("\nVoltage: %lumV (ADC=%u)\n",
               PowerMonitor::adc_to_voltage_mv(stBaselineVol.avg()),
               stBaselineVol.avg());
    swo_printf("Current thresholds: STALL=%u OVERCURRENT=%u\n",
               StallDetect::STALL_THRESHOLD,
               StallDetect::OVERCURRENT_THRESHOLD);
    swo_printf("========== DONE ==========\n");

    for (;;) { vTaskDelay(pdMS_TO_TICKS(500)); HAL_IWDG_Refresh(&hiwdg); }
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
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TIM8_UP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);

    xTaskCreate(diag_task_entry, "diag", 1024, nullptr, 4, nullptr);
    swo_printf("[BOOT] ZLENS_DIAG Current Profile starting...\n");
}
