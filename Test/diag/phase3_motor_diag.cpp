// Test/diag/phase3_motor_diag.cpp — Phase 3 board diagnostic: homing + stall + run current
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

// CubeMX-generated HAL handles
extern "C" {
    extern TIM_HandleTypeDef htim3;
    extern TIM_HandleTypeDef htim8;
    extern DAC_HandleTypeDef hdac;
    extern ADC_HandleTypeDef hadc1;
    extern SPI_HandleTypeDef hspi2;
    extern UART_HandleTypeDef huart2;
    extern IWDG_HandleTypeDef hiwdg;
}

// ---------- Global instances (same as product, needed by task entry points) ----------
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

// ---------- Diagnostic constants ----------
namespace {

constexpr uint32_t BASELINE_SAMPLES    = 20;
constexpr uint32_t BASELINE_INTERVAL   = 100;  // ms between baseline samples
constexpr uint16_t SPEEDS[]            = {800, 1600, 3200};
constexpr uint32_t SPEED_COUNT         = sizeof(SPEEDS) / sizeof(SPEEDS[0]);
constexpr int32_t  HOMING_FAR          = 1000000;
constexpr int32_t  HOMING_RETRACT      = 4096;
constexpr int32_t  SOFT_OFFSET         = 200;
constexpr uint32_t ENCODER_STALL_MS    = 2000; // encoder no change for 2s = stalled
constexpr uint32_t MOVE_TIMEOUT_MS     = 10000; // 10s max for any single move
constexpr uint32_t IDLE_STALL_MS       = 2000;  // 2s encoder no-change = stalled in wait-for-idle

// ADC conversion helpers
inline uint32_t adc_to_cur_mv(uint16_t adc) {
    return static_cast<uint32_t>(adc) * 3300 / 4095;
}

inline uint32_t adc_to_vol_mv(uint16_t adc) {
    return zlens::PowerMonitor::adc_to_voltage_mv(adc);
}

struct RUN_STATS_S {
    uint32_t iCurSum;
    uint16_t iCurMax;
    uint16_t iCurMin;
    uint32_t iVolSum;
    uint32_t iSampleCount;

    void reset() {
        iCurSum = 0; iCurMax = 0; iCurMin = 0xFFFF;
        iVolSum = 0; iSampleCount = 0;
    }

    void add(uint16_t iCur, uint16_t iVol) {
        iCurSum += iCur;
        if (iCur > iCurMax) iCurMax = iCur;
        if (iCur < iCurMin) iCurMin = iCur;
        iVolSum += iVol;
        iSampleCount++;
    }

    uint16_t cur_avg() const { return iSampleCount ? iCurSum / iSampleCount : 0; }
    uint16_t vol_avg() const { return iSampleCount ? iVolSum / iSampleCount : 0; }
};

// Safe move: wait for IDLE with timeout + encoder stall detection
// Returns true if reached target, false if timed out or encoder stalled
bool safe_move_wait(zlens::MotorCtrl& motor, zlens::Encoder& encoder,
                    IWDG_HandleTypeDef& hiwdg, uint32_t iTimeoutMs) {
    int32_t iLastPos = encoder.get_position();
    uint32_t iNoChangeCount = 0;
    uint32_t iTickCount = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        motor.update();
        iTickCount++;

        if (motor.get_state() == zlens::MOTOR_STATE_E::IDLE) return true;

        // Absolute timeout
        if (iTickCount >= iTimeoutMs) {
            motor.emergency_stop();
            swo_printf("[DIAG] SAFETY: move timeout (%lums)\n",
                       static_cast<unsigned long>(iTimeoutMs));
            return false;
        }

        // Encoder stall detection (motor running but encoder not moving)
        int32_t iPos = encoder.get_position();
        if (iPos != iLastPos) {
            iLastPos = iPos;
            iNoChangeCount = 0;
        } else {
            iNoChangeCount++;
            if (iNoChangeCount >= IDLE_STALL_MS) {
                motor.emergency_stop();
                swo_printf("[DIAG] SAFETY: encoder stall at pos=%ld\n",
                           static_cast<long>(iPos));
                return false;
            }
        }
    }
}

} // anonymous namespace

// ---------- Diagnostic task ----------
static void diag_task_entry(void* params) {
    (void)params;
    using namespace zlens;

    // Local modules for the diagnostic — independent from motor_task
    MotorCtrl motor;
    Encoder& encoder = g_Encoder;  // Shared encoder (hardware singleton)
    StallDetect stall;
    AdcFilter filter;

    motor.init(&htim3, &hdac, &encoder);
    stall.init();
    filter.reset(0);

    volatile uint16_t* pAdcCur = &g_aAdcDmaBuf[0];
    volatile uint16_t* pAdcVol = &g_aAdcDmaBuf[1];

    swo_printf("[DIAG] === START ===\n");

    // ===================== Test 0: Encoder direction check =====================
    swo_printf("[DIAG] Test 0: Encoder direction...\n");
    encoder.set_position(0);
    swo_printf("[DIAG] Init: pos=%ld cnt=%lu\n",
               encoder.get_position(), static_cast<unsigned long>(TIM8->CNT));

    // Brief forward move (500ms)
    motor.move_to(100000);
    for (int i = 0; i < 500; i++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        motor.update();
    }
    motor.emergency_stop();
    int32_t iFwdPos = encoder.get_position();
    swo_printf("[DIAG] After FWD 500ms: pos=%ld cnt=%lu\n",
               static_cast<long>(iFwdPos), static_cast<unsigned long>(TIM8->CNT));

    // Brief reverse move (500ms)
    motor.move_to(-100000);
    for (int i = 0; i < 500; i++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        motor.update();
    }
    motor.emergency_stop();
    int32_t iRevPos = encoder.get_position();
    swo_printf("[DIAG] After REV 500ms: pos=%ld cnt=%lu\n",
               static_cast<long>(iRevPos), static_cast<unsigned long>(TIM8->CNT));

    swo_printf("[DIAG] Direction: FWD=%s REV=%s\n",
               iFwdPos > 0 ? "OK" : "INVERTED",
               iRevPos < iFwdPos ? "OK" : "INVERTED");

    // Reset for subsequent tests
    encoder.set_position(0);
    vTaskDelay(pdMS_TO_TICKS(200));
    HAL_IWDG_Refresh(&hiwdg);

    // ===================== Test 1: Baseline (2s) =====================
    swo_printf("[DIAG] Test 1: Baseline sampling...\n");
    uint32_t iCurSum = 0, iVolSum = 0;
    for (uint32_t i = 0; i < BASELINE_SAMPLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(BASELINE_INTERVAL));
        HAL_IWDG_Refresh(&hiwdg);
        iCurSum += *pAdcCur;
        iVolSum += *pAdcVol;
    }
    uint16_t iBaselineCur = iCurSum / BASELINE_SAMPLES;
    uint16_t iBaselineVol = iVolSum / BASELINE_SAMPLES;
    swo_printf("[DIAG] Baseline: cur=%u(%lumV) vol=%u(%lumV)\n",
               iBaselineCur, adc_to_cur_mv(iBaselineCur),
               iBaselineVol, adc_to_vol_mv(iBaselineVol));

    // ===================== Test 2: Homing + Stall Current =====================
    swo_printf("[DIAG] Test 2: Homing + stall current...\n");

    // --- Phase A: Reverse to min limit ---
    stall.set_direction(StallDetect::Direction::REVERSE);
    stall.start_motor();
    motor.move_to(-HOMING_FAR);

    uint16_t iStallRev = 0;
    uint32_t iPhaseATick = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        iPhaseATick++;

        uint16_t iRaw = *pAdcCur;
        uint16_t iFiltered = filter.update(iRaw);
        int32_t iPos = encoder.get_position();
        stall.update(iFiltered, iPos, HAL_GetTick());
        motor.update();

        if (stall.is_stalled() || stall.is_overcurrent()) {
            iStallRev = iFiltered;
            motor.emergency_stop();
            swo_printf("[DIAG] PhaseA stall: pos=%ld cnt=%lu\n",
                       static_cast<long>(encoder.get_position()),
                       static_cast<unsigned long>(TIM8->CNT));
            encoder.set_position(0);
            stall.reset();
            break;
        }
        if (iPhaseATick >= MOVE_TIMEOUT_MS) {
            motor.emergency_stop();
            swo_printf("[DIAG] SAFETY: PhaseA timeout\n");
            encoder.set_position(0);
            stall.reset();
            break;
        }
    }

    // --- Phase B: Retract 4096 counts ---
    filter.reset(0);
    swo_printf("[DIAG] PhaseB retract: start pos=%ld\n",
               static_cast<long>(encoder.get_position()));
    motor.move_to(HOMING_RETRACT);
    if (!safe_move_wait(motor, encoder, hiwdg, MOVE_TIMEOUT_MS)) {
        swo_printf("[DIAG] PhaseB FAILED\n");
        swo_printf("[DIAG] === END ===\n");
        for (;;) { vTaskDelay(pdMS_TO_TICKS(500)); HAL_IWDG_Refresh(&hiwdg); }
    }
    swo_printf("[DIAG] PhaseB done: pos=%ld\n",
               static_cast<long>(encoder.get_position()));

    // --- Phase C: Forward to max limit ---
    stall.set_direction(StallDetect::Direction::FORWARD);
    stall.start_motor();
    filter.reset(0);
    motor.move_to(HOMING_FAR);

    uint16_t iStallFwd = 0;
    int32_t iTotalRange = 0;
    uint32_t iPhaseCTick = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        iPhaseCTick++;

        uint16_t iRaw = *pAdcCur;
        uint16_t iFiltered = filter.update(iRaw);
        int32_t iPos = encoder.get_position();
        stall.update(iFiltered, iPos, HAL_GetTick());
        motor.update();

        if (stall.is_stalled() || stall.is_overcurrent()) {
            iStallFwd = iFiltered;
            iTotalRange = encoder.get_position();
            motor.emergency_stop();
            swo_printf("[DIAG] PhaseC stall: pos=%ld cnt=%lu\n",
                       static_cast<long>(iTotalRange),
                       static_cast<unsigned long>(TIM8->CNT));
            stall.reset();
            break;
        }
        if (iPhaseCTick >= MOVE_TIMEOUT_MS) {
            motor.emergency_stop();
            iTotalRange = encoder.get_position();
            swo_printf("[DIAG] SAFETY: PhaseC timeout at pos=%ld\n",
                       static_cast<long>(iTotalRange));
            stall.reset();
            break;
        }
    }

    int32_t iSoftMin = SOFT_OFFSET;
    int32_t iSoftMax = iTotalRange - SOFT_OFFSET;

    swo_printf("[DIAG] Homing: range=%ld stall_rev=%u(%lumV) stall_fwd=%u(%lumV)\n",
               static_cast<long>(iTotalRange),
               iStallRev, adc_to_cur_mv(iStallRev),
               iStallFwd, adc_to_cur_mv(iStallFwd));

    // Move to soft_min as starting position
    filter.reset(0);
    motor.move_to(iSoftMin);
    if (!safe_move_wait(motor, encoder, hiwdg, MOVE_TIMEOUT_MS)) {
        swo_printf("[DIAG] Move to soft_min FAILED\n");
        swo_printf("[DIAG] === END ===\n");
        for (;;) { vTaskDelay(pdMS_TO_TICKS(500)); HAL_IWDG_Refresh(&hiwdg); }
    }

    // ===================== Test 3: Run Current at 3 speeds =====================
    swo_printf("[DIAG] Test 3: Run current profiling...\n");

    // Per-speed results: [speed_idx][0=fwd, 1=rev]
    RUN_STATS_S aRunStats[SPEED_COUNT][2];

    for (uint32_t s = 0; s < SPEED_COUNT; s++) {
        aRunStats[s][0].reset();
        aRunStats[s][1].reset();

        uint16_t iSpeed = SPEEDS[s];
        motor.set_max_speed(iSpeed);

        // Run helper: move motor with logging + encoder stall timeout
        // dir_idx: 0=fwd, 1=rev
        auto run_with_diag = [&](int32_t iTarget, uint32_t iDirIdx, const char* pDir) -> bool {
            filter.reset(0);
            int32_t iPosAtStart = encoder.get_position();
            swo_printf("[DIAG] @%u %s: pos=%ld target=%ld\n",
                       iSpeed, pDir,
                       static_cast<long>(iPosAtStart),
                       static_cast<long>(iTarget));
            motor.move_to(iTarget);

            uint32_t iTickCount = 0;
            int32_t iLastPos = iPosAtStart;
            uint32_t iNoChangeCount = 0;

            for (;;) {
                vTaskDelay(pdMS_TO_TICKS(1));
                HAL_IWDG_Refresh(&hiwdg);

                uint16_t iRaw = *pAdcCur;
                uint16_t iFiltered = filter.update(iRaw);
                int32_t iPos = encoder.get_position();
                motor.update();
                iTickCount++;

                // Sample after blanking period (200ms)
                if (iTickCount > 200) {
                    aRunStats[s][iDirIdx].add(iFiltered, *pAdcVol);
                }

                // Periodic debug log every 500ms
                if (iTickCount % 500 == 0) {
                    swo_printf("[DIAG]   t=%lu pos=%ld rem=%ld spd=%u st=%u\n",
                               iTickCount,
                               static_cast<long>(iPos),
                               static_cast<long>(iTarget - iPos),
                               motor.get_current_speed(),
                               static_cast<unsigned>(motor.get_state()));
                }

                // Absolute timeout
                if (iTickCount >= MOVE_TIMEOUT_MS) {
                    motor.emergency_stop();
                    swo_printf("[DIAG] @%u %s: TIMEOUT at pos=%ld (target=%ld)\n",
                               iSpeed, pDir,
                               static_cast<long>(iPos),
                               static_cast<long>(iTarget));
                    return false;
                }

                // Encoder stall timeout: no position change for 2s
                if (iPos != iLastPos) {
                    iLastPos = iPos;
                    iNoChangeCount = 0;
                } else {
                    iNoChangeCount++;
                    if (iNoChangeCount >= ENCODER_STALL_MS) {
                        motor.emergency_stop();
                        swo_printf("[DIAG] @%u %s: ENCODER STALL at pos=%ld (target=%ld)\n",
                                   iSpeed, pDir,
                                   static_cast<long>(iPos),
                                   static_cast<long>(iTarget));
                        return false;
                    }
                }

                if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
            }

            swo_printf("[DIAG] @%u %s: done pos=%ld\n",
                       iSpeed, pDir,
                       static_cast<long>(encoder.get_position()));
            return true;
        };

        // --- Forward: soft_min → soft_max ---
        bool bFwdOk = run_with_diag(iSoftMax, 0, "FWD");

        vTaskDelay(pdMS_TO_TICKS(200));
        HAL_IWDG_Refresh(&hiwdg);

        // --- Reverse: soft_max → soft_min ---
        bool bRevOk = run_with_diag(iSoftMin, 1, "REV");

        vTaskDelay(pdMS_TO_TICKS(200));
        HAL_IWDG_Refresh(&hiwdg);

        swo_printf("[DIAG] Run @%u: fwd_avg=%u max=%u rev_avg=%u max=%u vol=%u%s\n",
                   iSpeed,
                   aRunStats[s][0].cur_avg(), aRunStats[s][0].iCurMax,
                   aRunStats[s][1].cur_avg(), aRunStats[s][1].iCurMax,
                   aRunStats[s][0].vol_avg(),
                   (!bFwdOk || !bRevOk) ? " STALL" : "");
    }

    // Restore default max speed
    motor.set_max_speed(MotorCtrl::MAX_SPEED);

    // ===================== Summary =====================
    // Find global max running current
    uint16_t iRunMax = 0;
    for (uint32_t s = 0; s < SPEED_COUNT; s++) {
        if (aRunStats[s][0].iCurMax > iRunMax) iRunMax = aRunStats[s][0].iCurMax;
        if (aRunStats[s][1].iCurMax > iRunMax) iRunMax = aRunStats[s][1].iCurMax;
    }

    // Stall min = lower of the two stall values
    uint16_t iStallMin = (iStallRev < iStallFwd) ? iStallRev : iStallFwd;

    // Suggested thresholds
    uint16_t iSuggestedOvercurrent = iRunMax + (iStallMin - iRunMax) / 2;
    uint16_t iSuggestedPowerDown = iBaselineVol > 50 ? iBaselineVol - 50 : 100;

    swo_printf("[DIAG] === SUMMARY ===\n");
    swo_printf("[DIAG] Baseline: cur=%u vol=%u(%lumV)\n",
               iBaselineCur, iBaselineVol, adc_to_vol_mv(iBaselineVol));
    swo_printf("[DIAG] Stall: rev=%u(%lumV) fwd=%u(%lumV)\n",
               iStallRev, adc_to_cur_mv(iStallRev),
               iStallFwd, adc_to_cur_mv(iStallFwd));
    for (uint32_t s = 0; s < SPEED_COUNT; s++) {
        swo_printf("[DIAG] @%u: fwd=%u/%u rev=%u/%u vol=%u\n",
                   SPEEDS[s],
                   aRunStats[s][0].cur_avg(), aRunStats[s][0].iCurMax,
                   aRunStats[s][1].cur_avg(), aRunStats[s][1].iCurMax,
                   aRunStats[s][0].vol_avg());
    }
    swo_printf("[DIAG] Suggested: OVERCURRENT=%u POWER_DOWN=%u\n",
               iSuggestedOvercurrent, iSuggestedPowerDown);
    swo_printf("[DIAG] === END ===\n");

    // Done — idle forever with watchdog
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        HAL_IWDG_Refresh(&hiwdg);
    }
}

// ---------- app_init() — replaces product app_instances.cpp ----------
extern "C" void app_init(void) {
    using namespace zlens;

    // Create queues (needed by task infrastructure even if unused by diag)
    g_cmdQueue  = xQueueCreate(8, sizeof(CMD_MESSAGE_S));
    g_rspQueue  = xQueueCreate(8, sizeof(RSP_MESSAGE_S));
    g_saveQueue = xQueueCreate(4, sizeof(SAVE_MESSAGE_S));

    // Init App modules
    g_Motor.init(&htim3, &hdac, &g_Encoder);
    g_Encoder.init();
    g_StallDetect.init();
    g_ZoomTable.init();
    g_ZoomTable.load_defaults();
    g_FramStorage.init(&hspi2);
    g_CommProtocol.init();
    g_SystemManager.init();
    g_PowerMonitor.init();

    // ADC self-calibration
    HAL_ADCEx_Calibration_Start(&hadc1);

    // Start ADC DMA continuous conversion
    HAL_ADC_Start_DMA(&hadc1,
                       reinterpret_cast<uint32_t*>(
                           const_cast<uint16_t*>(g_aAdcDmaBuf)),
                       2);

    // Start TIM3 PWM (both channels, initially 0% duty)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // Start TIM8 encoder + enable overflow interrupt for 32-bit position
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TIM8_UP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);

    // Create diagnostic task only (no product tasks)
    xTaskCreate(diag_task_entry, "diag", 1024, nullptr, 4, nullptr);

    swo_printf("[BOOT] ZLENS_DIAG Phase 3 starting...\n");
}

