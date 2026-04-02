// Test/diag/pid_tune_diag.cpp — PID auto-tuning diagnostic (SWO output)
// Build: cmake -B build/diag -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake
//        -DBUILD_DIAG=ON -DDIAG_SOURCE=pid_tune_diag.cpp
// Flow: Home → KP sweep → KI sweep → KD sweep → MIN_SPEED sweep → final verify
// Output: SWO only, parsed by scripts/pid_tune_auto.py

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
#include <cstdlib>
#include <cmath>

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

// ============================================================================
// PID tuning runtime state
// ============================================================================
namespace {

struct PidParams {
    float fKp       = 0.50f;
    float fKi       = 0.00f;
    float fKd       = 0.10f;
    uint16_t iMinSpeed = 200;
};

struct PidState {
    float   fIntegral = 0.0f;
    int32_t iLastPos  = 0;
    void reset(int32_t iPos) { fIntegral = 0.0f; iLastPos = iPos; }
};

struct TestResult {
    int32_t iMeanFwd  = 0;
    int32_t iMaxFwd   = 0;
    int32_t iMeanRev  = 0;
    int32_t iMaxRev   = 0;
    uint32_t iCountFwd = 0;
    uint32_t iCountRev = 0;

    int32_t score() const {
        if (iCountFwd == 0 || iCountRev == 0) return 9999;
        int32_t iAbsMean = (std::abs(iMeanFwd) + std::abs(iMeanRev)) / 2;
        int32_t iMax = (iMaxFwd > iMaxRev) ? iMaxFwd : iMaxRev;
        return iMax * 2 + iAbsMean;
    }
};

PidParams g_stParams;
PidState  g_stPid;

constexpr float    INTEGRAL_MAX     = 500.0f;
constexpr float    MAX_OUTPUT       = 1200.0f;
constexpr uint16_t HOMING_SPEED     = 480;
constexpr uint32_t STALL_TIMEOUT_MS = 500;
constexpr uint16_t ENCODER_TIMEOUT  = 500;
constexpr int32_t  DEADZONE         = 3;
constexpr uint16_t SETTLE_COUNT     = 100;
constexpr uint32_t MOVE_TIMEOUT_MS  = 5000;
constexpr uint32_t REPS_PER_TEST    = 4;
constexpr uint32_t REPS_FINAL       = 10;

// 1.0X and 2.0X target positions
constexpr int32_t POS_1X = 13235;
constexpr int32_t POS_2X = 27070;

// Sweep parameter tables
constexpr int32_t KP_TABLE[] = { 30, 40, 50, 60, 70, 80, 100 };  // ×100
constexpr int32_t KI_TABLE[] = {  0,  1,  2,  3,  5,  8,  10 };  // ×100
constexpr int32_t KD_TABLE[] = {  0,  5, 10, 15, 20, 30 };       // ×100
constexpr int32_t MS_TABLE[] = { 100, 150, 200, 250, 300, 400 };  // raw PWM

constexpr uint32_t KP_COUNT = sizeof(KP_TABLE) / sizeof(KP_TABLE[0]);
constexpr uint32_t KI_COUNT = sizeof(KI_TABLE) / sizeof(KI_TABLE[0]);
constexpr uint32_t KD_COUNT = sizeof(KD_TABLE) / sizeof(KD_TABLE[0]);
constexpr uint32_t MS_COUNT = sizeof(MS_TABLE) / sizeof(MS_TABLE[0]);

// ============================================================================
// PID compute
// ============================================================================
int16_t pid_compute(int32_t iError, int32_t iPos) {
    float fP = g_stParams.fKp * static_cast<float>(iError);

    g_stPid.fIntegral += g_stParams.fKi * static_cast<float>(iError);
    if (g_stPid.fIntegral >  INTEGRAL_MAX) g_stPid.fIntegral =  INTEGRAL_MAX;
    if (g_stPid.fIntegral < -INTEGRAL_MAX) g_stPid.fIntegral = -INTEGRAL_MAX;

    float fD = -g_stParams.fKd * static_cast<float>(iPos - g_stPid.iLastPos);
    g_stPid.iLastPos = iPos;

    float fOutput = fP + g_stPid.fIntegral + fD;
    if (fOutput >  MAX_OUTPUT) fOutput =  MAX_OUTPUT;
    if (fOutput < -MAX_OUTPUT) fOutput = -MAX_OUTPUT;
    return static_cast<int16_t>(fOutput);
}

// ============================================================================
// Motor helpers
// ============================================================================
void motor_set_pwm(zlens::DIRECTION_E eDir, uint16_t iSpeed) {
    zlens::g_Motor.set_pwm_test(eDir, iSpeed);
}
void motor_brake() { zlens::g_Motor.brake_test(); }

// ============================================================================
// Homing
// ============================================================================
void drive_to_stall(zlens::DIRECTION_E eDir, uint16_t iSpeed, const char* pLabel) {
    motor_set_pwm(eDir, iSpeed);
    int32_t iLastPos = zlens::g_Encoder.get_position();
    uint32_t iStallCount = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        int32_t iPos = zlens::g_Encoder.get_position();
        if (iPos == iLastPos) {
            if (++iStallCount >= STALL_TIMEOUT_MS) break;
        } else {
            iStallCount = 0;
            iLastPos = iPos;
        }
    }
    motor_brake();
    swo_printf("[STALL] %s pos=%ld\n", pLabel,
               static_cast<long>(zlens::g_Encoder.get_position()));
}

// Simple open-loop + PID move for homing retract
void simple_move_to(int32_t iTarget, uint32_t iTimeoutMs) {
    g_stPid.reset(zlens::g_Encoder.get_position());
    for (uint32_t t = 0; t < iTimeoutMs; ++t) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);
        int32_t iPos = zlens::g_Encoder.get_position();
        int32_t iErr = iTarget - iPos;
        if (std::abs(iErr) <= DEADZONE) { motor_brake(); return; }
        int16_t iOut = pid_compute(iErr, iPos);
        uint16_t iSpd = static_cast<uint16_t>(std::abs(iOut));
        if (iSpd < g_stParams.iMinSpeed) iSpd = g_stParams.iMinSpeed;
        motor_set_pwm(iOut >= 0 ? zlens::DIRECTION_E::FORWARD
                                : zlens::DIRECTION_E::REVERSE, iSpd);
    }
    motor_brake();
}

void do_homing() {
    swo_printf("[HOME] Fast reverse...\n");
    drive_to_stall(zlens::DIRECTION_E::REVERSE, HOMING_SPEED, "FAST");
    zlens::g_Encoder.set_position(0);
    vTaskDelay(pdMS_TO_TICKS(300));

    swo_printf("[HOME] Retract to 256...\n");
    simple_move_to(256, 3000);
    vTaskDelay(pdMS_TO_TICKS(200));

    swo_printf("[HOME] Slow reverse...\n");
    drive_to_stall(zlens::DIRECTION_E::REVERSE, HOMING_SPEED, "SLOW");
    zlens::g_Encoder.set_position(0);
    vTaskDelay(pdMS_TO_TICKS(300));

    swo_printf("[HOME] Settle to HOME_OFFSET=%d...\n", zlens::ZoomTable::HOME_OFFSET);
    simple_move_to(zlens::ZoomTable::HOME_OFFSET, 3000);
    vTaskDelay(pdMS_TO_TICKS(500));

    swo_printf("[HOME] Done. pos=%ld\n",
               static_cast<long>(zlens::g_Encoder.get_position()));
}

// ============================================================================
// PID positioning (pure PID, NO coast/nudge)
// ============================================================================
enum class MoveResult { ARRIVED, STALLED, TIMEOUT };

struct MoveStats {
    MoveResult eResult;
    int32_t    iError;
    uint32_t   iElapsedMs;
};

MoveStats pid_move_to(int32_t iTarget) {
    MoveStats st = {};
    int32_t iPos = zlens::g_Encoder.get_position();
    g_stPid.reset(iPos);

    uint16_t iSettleCounter = 0;
    uint16_t iNoMoveCount   = 0;
    int32_t  iLastPosStall  = iPos;
    int32_t  iLastPosSettle = iPos;
    uint32_t iStartTick     = HAL_GetTick();

    for (uint32_t t = 0; t < MOVE_TIMEOUT_MS; ++t) {
        vTaskDelay(pdMS_TO_TICKS(1));
        HAL_IWDG_Refresh(&hiwdg);

        iPos = zlens::g_Encoder.get_position();
        int32_t iErr = iTarget - iPos;

        // Stall detection
        if (iPos == iLastPosStall) {
            if (++iNoMoveCount >= ENCODER_TIMEOUT) {
                motor_brake();
                st.eResult = MoveResult::STALLED;
                st.iError  = iErr;
                st.iElapsedMs = HAL_GetTick() - iStartTick;
                return st;
            }
        } else {
            iNoMoveCount  = 0;
            iLastPosStall = iPos;
        }

        // Deadzone → brake and settle
        if (std::abs(iErr) <= DEADZONE) {
            motor_brake();
            if (iPos == iLastPosSettle) {
                if (++iSettleCounter >= SETTLE_COUNT) {
                    st.eResult    = MoveResult::ARRIVED;
                    st.iError     = iErr;
                    st.iElapsedMs = HAL_GetTick() - iStartTick;
                    return st;
                }
            } else {
                iSettleCounter = 0;
            }
            iLastPosSettle = iPos;
            continue;
        }
        iSettleCounter = 0;
        iLastPosSettle = iPos;

        // PID compute — pure PID, no coast/nudge
        int16_t iOutput = pid_compute(iErr, iPos);
        uint16_t iSpeed = static_cast<uint16_t>(std::abs(iOutput));
        if (iSpeed < g_stParams.iMinSpeed) {
            iSpeed = g_stParams.iMinSpeed;
        }

        zlens::DIRECTION_E eDir = (iOutput >= 0)
            ? zlens::DIRECTION_E::FORWARD
            : zlens::DIRECTION_E::REVERSE;
        motor_set_pwm(eDir, iSpeed);
    }

    motor_brake();
    st.eResult    = MoveResult::TIMEOUT;
    st.iError     = iTarget - zlens::g_Encoder.get_position();
    st.iElapsedMs = HAL_GetTick() - iStartTick;
    return st;
}

// ============================================================================
// Run N laps between POS_1X and POS_2X, return aggregated result
// ============================================================================
TestResult run_laps(uint32_t iReps) {
    TestResult res = {};
    int64_t iSumFwd = 0, iSumRev = 0;

    for (uint32_t iLap = 1; iLap <= iReps; ++iLap) {
        HAL_IWDG_Refresh(&hiwdg);

        // FWD: 1X → 2X
        MoveStats stFwd = pid_move_to(POS_2X);
        vTaskDelay(pdMS_TO_TICKS(200));
        int32_t iActual = zlens::g_Encoder.get_position();
        int32_t iErr = iActual - POS_2X;
        swo_printf("[DATA] %lu,FWD,%ld,%ld,%lu\n",
                   static_cast<unsigned long>(iLap),
                   static_cast<long>(iActual),
                   static_cast<long>(iErr),
                   static_cast<unsigned long>(stFwd.iElapsedMs));
        if (stFwd.eResult == MoveResult::ARRIVED) {
            iSumFwd += iErr;
            res.iCountFwd++;
            if (std::abs(iErr) > res.iMaxFwd) res.iMaxFwd = std::abs(iErr);
        }

        HAL_IWDG_Refresh(&hiwdg);

        // REV: 2X → 1X
        MoveStats stRev = pid_move_to(POS_1X);
        vTaskDelay(pdMS_TO_TICKS(200));
        iActual = zlens::g_Encoder.get_position();
        iErr = iActual - POS_1X;
        swo_printf("[DATA] %lu,REV,%ld,%ld,%lu\n",
                   static_cast<unsigned long>(iLap),
                   static_cast<long>(iActual),
                   static_cast<long>(iErr),
                   static_cast<unsigned long>(stRev.iElapsedMs));
        if (stRev.eResult == MoveResult::ARRIVED) {
            iSumRev += iErr;
            res.iCountRev++;
            if (std::abs(iErr) > res.iMaxRev) res.iMaxRev = std::abs(iErr);
        }
    }

    res.iMeanFwd = (res.iCountFwd > 0) ? static_cast<int32_t>(iSumFwd / res.iCountFwd) : 0;
    res.iMeanRev = (res.iCountRev > 0) ? static_cast<int32_t>(iSumRev / res.iCountRev) : 0;
    return res;
}

// ============================================================================
// Run one parameter set test: set params, run laps, log result
// ============================================================================
TestResult test_params(int32_t iKp100, int32_t iKi100, int32_t iKd100, int32_t iMinSpd) {
    g_stParams.fKp       = static_cast<float>(iKp100) / 100.0f;
    g_stParams.fKi       = static_cast<float>(iKi100) / 100.0f;
    g_stParams.fKd       = static_cast<float>(iKd100) / 100.0f;
    g_stParams.iMinSpeed = static_cast<uint16_t>(iMinSpd);

    swo_printf("[TEST] kp=%ld ki=%ld kd=%ld ms=%ld\n",
               static_cast<long>(iKp100), static_cast<long>(iKi100),
               static_cast<long>(iKd100), static_cast<long>(iMinSpd));

    TestResult res = run_laps(REPS_PER_TEST);

    swo_printf("[RESULT] kp=%ld ki=%ld kd=%ld ms=%ld "
               "fwd_m=%ld fwd_x=%ld rev_m=%ld rev_x=%ld score=%ld\n",
               static_cast<long>(iKp100), static_cast<long>(iKi100),
               static_cast<long>(iKd100), static_cast<long>(iMinSpd),
               static_cast<long>(res.iMeanFwd), static_cast<long>(res.iMaxFwd),
               static_cast<long>(res.iMeanRev), static_cast<long>(res.iMaxRev),
               static_cast<long>(res.score()));
    return res;
}

} // anonymous namespace

// ============================================================================
// Main diagnostic task — autonomous staged sweep
// ============================================================================
static void diag_task_entry(void* params) {
    (void)params;
    using namespace zlens;

    swo_printf("[DIAG] === PID AUTO-TUNE ===\n");
    swo_printf("[DIAG] Targets: 1X=%ld 2X=%ld  Reps=%lu  Final=%lu\n",
               static_cast<long>(POS_1X), static_cast<long>(POS_2X),
               static_cast<unsigned long>(REPS_PER_TEST),
               static_cast<unsigned long>(REPS_FINAL));

    // === Homing ===
    do_homing();

    // Track best params across phases
    int32_t iBestKp100  = 50;
    int32_t iBestKi100  = 0;
    int32_t iBestKd100  = 10;
    int32_t iBestMinSpd = 200;

    // === Phase 1: KP sweep ===
    swo_printf("\n[SWEEP] PHASE=1 KP (ki=0 kd=10 ms=200)\n");
    {
        int32_t iBestScore = 9999;
        for (uint32_t i = 0; i < KP_COUNT; ++i) {
            TestResult r = test_params(KP_TABLE[i], 0, 10, 200);
            if (r.score() < iBestScore) {
                iBestScore = r.score();
                iBestKp100 = KP_TABLE[i];
            }
        }
        swo_printf("[BEST] PHASE=1 kp=%ld score=%ld\n",
                   static_cast<long>(iBestKp100), static_cast<long>(iBestScore));
    }

    // === Phase 2: KI sweep ===
    swo_printf("\n[SWEEP] PHASE=2 KI (kp=%ld kd=10 ms=200)\n",
               static_cast<long>(iBestKp100));
    {
        int32_t iBestScore = 9999;
        for (uint32_t i = 0; i < KI_COUNT; ++i) {
            TestResult r = test_params(iBestKp100, KI_TABLE[i], 10, 200);
            if (r.score() < iBestScore) {
                iBestScore = r.score();
                iBestKi100 = KI_TABLE[i];
            }
        }
        swo_printf("[BEST] PHASE=2 ki=%ld score=%ld\n",
                   static_cast<long>(iBestKi100), static_cast<long>(iBestScore));
    }

    // === Phase 3: KD sweep ===
    swo_printf("\n[SWEEP] PHASE=3 KD (kp=%ld ki=%ld ms=200)\n",
               static_cast<long>(iBestKp100), static_cast<long>(iBestKi100));
    {
        int32_t iBestScore = 9999;
        for (uint32_t i = 0; i < KD_COUNT; ++i) {
            TestResult r = test_params(iBestKp100, iBestKi100, KD_TABLE[i], 200);
            if (r.score() < iBestScore) {
                iBestScore = r.score();
                iBestKd100 = KD_TABLE[i];
            }
        }
        swo_printf("[BEST] PHASE=3 kd=%ld score=%ld\n",
                   static_cast<long>(iBestKd100), static_cast<long>(iBestScore));
    }

    // === Phase 4: MIN_SPEED sweep ===
    swo_printf("\n[SWEEP] PHASE=4 MIN_SPEED (kp=%ld ki=%ld kd=%ld)\n",
               static_cast<long>(iBestKp100), static_cast<long>(iBestKi100),
               static_cast<long>(iBestKd100));
    {
        int32_t iBestScore = 9999;
        for (uint32_t i = 0; i < MS_COUNT; ++i) {
            TestResult r = test_params(iBestKp100, iBestKi100, iBestKd100, MS_TABLE[i]);
            if (r.score() < iBestScore) {
                iBestScore = r.score();
                iBestMinSpd = MS_TABLE[i];
            }
        }
        swo_printf("[BEST] PHASE=4 ms=%ld score=%ld\n",
                   static_cast<long>(iBestMinSpd), static_cast<long>(iBestScore));
    }

    // === Final verification ===
    swo_printf("\n[SWEEP] FINAL VERIFY (kp=%ld ki=%ld kd=%ld ms=%ld) reps=%lu\n",
               static_cast<long>(iBestKp100), static_cast<long>(iBestKi100),
               static_cast<long>(iBestKd100), static_cast<long>(iBestMinSpd),
               static_cast<unsigned long>(REPS_FINAL));

    g_stParams.fKp       = static_cast<float>(iBestKp100) / 100.0f;
    g_stParams.fKi       = static_cast<float>(iBestKi100) / 100.0f;
    g_stParams.fKd       = static_cast<float>(iBestKd100) / 100.0f;
    g_stParams.iMinSpeed = static_cast<uint16_t>(iBestMinSpd);

    TestResult rFinal = run_laps(REPS_FINAL);

    swo_printf("\n[FINAL] KP=%ld KI=%ld KD=%ld MIN_SPD=%ld\n",
               static_cast<long>(iBestKp100), static_cast<long>(iBestKi100),
               static_cast<long>(iBestKd100), static_cast<long>(iBestMinSpd));
    swo_printf("[FINAL] FWD: mean=%ld max=%ld  REV: mean=%ld max=%ld  score=%ld\n",
               static_cast<long>(rFinal.iMeanFwd), static_cast<long>(rFinal.iMaxFwd),
               static_cast<long>(rFinal.iMeanRev), static_cast<long>(rFinal.iMaxRev),
               static_cast<long>(rFinal.score()));

    swo_printf("\n[DIAG] === DONE ===\n");

    for (;;) {
        HAL_IWDG_Refresh(&hiwdg);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// app_init
// ============================================================================
extern "C" void app_init(void) {
    using namespace zlens;

    g_cmdQueue  = xQueueCreate(8, sizeof(CMD_MESSAGE_S));
    g_rspQueue  = xQueueCreate(8, sizeof(RSP_MESSAGE_S));
    g_saveQueue = xQueueCreate(4, sizeof(SAVE_MESSAGE_S));

    g_Motor.init(&htim3, &hdac, &g_Encoder);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
    g_Motor.set_vref_mv(2000);
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

    xTaskCreate(diag_task_entry, "tune", 1024, nullptr, 4, nullptr);
    swo_printf("[BOOT] PID Tune Diag starting...\n");
}
