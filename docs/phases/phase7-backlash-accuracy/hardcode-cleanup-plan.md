# 硬编码清理实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 消除产品代码中的 magic number，提取为命名常量，提高可维护性

**Architecture:** 新建 `hw_constants.hpp` 集中硬件参数，归零参数搬入 `task_config.hpp`，各模块业务常量就地提取。同时重命名 `set_current_limit` → `set_vref_mv`，删除错误的电流换算函数。

**Tech Stack:** C++14, STM32 HAL, GoogleTest

**Spec:** `docs/phases/phase7-backlash-accuracy/hardcode-cleanup-spec.md`

**构建命令：**
```bash
# 单元测试
cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && ctest --test-dir build/test
# 固件编译
cmake -B build/fw -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake && cmake --build build/fw
```

---

### Task 1: 新建 `hw_constants.hpp`

**Files:**
- Create: `App/Inc/hw_constants.hpp`

- [ ] **Step 1: 创建 hw_constants.hpp**

```cpp
// App/Inc/hw_constants.hpp
#pragma once
#include <cstdint>

namespace zlens {
namespace hw {

// --- ADC / DAC ---
static constexpr uint16_t ADC_RESOLUTION      = 4095;   // 12-bit max
static constexpr uint16_t DAC_RESOLUTION      = 4095;   // 12-bit max
static constexpr uint16_t VREF_MV             = 3300;   // 参考电压 3.3V

// --- Encoder (TIM8, 16-bit) ---
static constexpr uint32_t TIMER_PERIOD        = 65536;  // 2^16

// --- FRAM SPI Commands ---
static constexpr uint8_t  FRAM_CMD_WREN       = 0x06;
static constexpr uint8_t  FRAM_CMD_WRITE      = 0x02;
static constexpr uint8_t  FRAM_CMD_READ       = 0x03;

// --- FRAM GPIO ---
static constexpr uint16_t FRAM_CS_PIN         = 1 << 12;  // PB12
static constexpr uint16_t FRAM_WP_PIN         = 1 << 11;  // PB11

// --- Flash Programming ---
static constexpr uint8_t  FLASH_HALFWORD_SIZE = 2;    // STM32 Flash 半字编程粒度 (bytes)

// --- Voltage Divider ---
// 推导：V_in = V_adc × (R_top + R_bot) / R_bot
//       V_mV = ADC × VREF_MV / ADC_RESOLUTION × (30000 + 4700) / 4700
//            = ADC × 24363 / ADC_RESOLUTION
static constexpr uint16_t VDIV_R_TOP          = 30000;  // 上分压电阻 30kΩ
static constexpr uint16_t VDIV_R_BOTTOM       = 4700;   // 下分压电阻 4.7kΩ
static constexpr uint16_t VOLTAGE_SCALE_NUM   = VREF_MV * (VDIV_R_TOP + VDIV_R_BOTTOM) / VDIV_R_BOTTOM;

} // namespace hw
} // namespace zlens
```

- [ ] **Step 2: 编译测试验证新头文件无语法错误**

Run: `cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test`
Expected: 编译通过（新文件尚未被 include，不影响现有代码）

- [ ] **Step 3: Commit**

```bash
git add App/Inc/hw_constants.hpp
git commit -m "feat: add hw_constants.hpp with hardware-level named constants"
```

---

### Task 2: 归零参数搬入 `task_config.hpp`

**Files:**
- Modify: `Tasks/Inc/task_config.hpp` — 末尾 `} // namespace zlens` 前新增 `homing` 命名空间
- Modify: `Tasks/Inc/motor_task.hpp:37-39` — 删除三个归零常量，改为引用
- Modify: `Tasks/Src/motor_task.cpp:270,285-288,686` — 更新引用

- [ ] **Step 1: 在 task_config.hpp 中新增 homing 命名空间**

在 `} // namespace zlens` 之前（第 124 行前）插入：

```cpp
// --- Homing mechanical parameters ---
namespace homing {
    constexpr int32_t RETRACT_DISTANCE = 4096;     // 粗定位后回退距离
    constexpr int32_t SETTLE_DISTANCE  = 2048;     // 归零稳定偏移 (= ZoomTable::HOME_OFFSET)
    constexpr int32_t FAR_DISTANCE     = 1000000;  // 归零远距离移动
} // namespace homing
```

注意：`task_config.hpp` 中已有的命名空间常量均使用 `constexpr`（非 `static constexpr`），此处保持一致。

- [ ] **Step 2: 修改 motor_task.hpp 删除原有常量，添加别名**

将 `motor_task.hpp:37-39` 的三行：
```cpp
    static constexpr int32_t HOMING_RETRACT_DISTANCE = 4096;
    static constexpr int32_t HOMING_SETTLE_DISTANCE = 2048;  // = ZoomTable::HOME_OFFSET
    static constexpr int32_t HOMING_FAR_DISTANCE = 1000000;
```
替换为：
```cpp
    static constexpr int32_t HOMING_RETRACT_DISTANCE = homing::RETRACT_DISTANCE;
    static constexpr int32_t HOMING_SETTLE_DISTANCE  = homing::SETTLE_DISTANCE;
    static constexpr int32_t HOMING_FAR_DISTANCE     = homing::FAR_DISTANCE;
```

注意：保留类内别名，这样 `MotorTask::HOMING_RETRACT_DISTANCE` 的所有引用（包括 26 处测试代码）无需修改。

- [ ] **Step 3: 运行测试验证**

Run: `cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && ctest --test-dir build/test`
Expected: 全部通过

- [ ] **Step 4: Commit**

```bash
git add Tasks/Inc/task_config.hpp Tasks/Inc/motor_task.hpp
git commit -m "refactor: move homing constants to task_config.hpp homing namespace"
```

---

### Task 3: `motor_ctrl` — `set_current_limit` → `set_vref_mv`

**Files:**
- Modify: `App/Inc/motor_ctrl.hpp:37` — 函数声明改名
- Modify: `App/Src/motor_ctrl.cpp:80-85` — 函数实现重写
- Modify: `App/Src/app_instances.cpp:56` — 调用处更新
- Modify: `Test/test_motor_ctrl.cpp:64` — 测试调用更新

- [ ] **Step 1: 修改 motor_ctrl.hpp 函数声明**

将 `motor_ctrl.hpp:37`：
```cpp
    void set_current_limit(uint16_t milliamps);
```
替换为：
```cpp
    void set_vref_mv(uint16_t mv);
```

- [ ] **Step 2: 修改 motor_ctrl.cpp 函数实现**

将 `motor_ctrl.cpp:80-85`：
```cpp
void MotorCtrl::set_current_limit(uint16_t milliamps) {
    uint32_t vref_mv = milliamps * 2;
    uint32_t dac_val = vref_mv * 4095 / 3300;
    if (dac_val > 4095) dac_val = 4095;
    HAL_DAC_SetValue(m_pHdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_val);
}
```
替换为：
```cpp
void MotorCtrl::set_vref_mv(uint16_t mv) {
    uint32_t dac_val = static_cast<uint32_t>(mv) * hw::DAC_RESOLUTION / hw::VREF_MV;
    if (dac_val > hw::DAC_RESOLUTION) dac_val = hw::DAC_RESOLUTION;
    HAL_DAC_SetValue(m_pHdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_val);
}
```

在 `motor_ctrl.cpp` 顶部添加 `#include "hw_constants.hpp"`。

- [ ] **Step 3: 修改 app_instances.cpp 调用处**

将 `app_instances.cpp:56`：
```cpp
    g_Motor.set_current_limit(1000);  // VREF=2.0V, I_trip=1A
```
替换为：
```cpp
    g_Motor.set_vref_mv(2000);  // A4950 VREF=2.0V
```

- [ ] **Step 4: 修改测试文件调用和测试用例名**

将 `Test/test_motor_ctrl.cpp` 中测试用例名：
```cpp
TEST_F(MotorCtrlTest, SetCurrentLimit) {
```
替换为：
```cpp
TEST_F(MotorCtrlTest, SetVrefMv) {
```

将调用 `test_motor_ctrl.cpp:64`：
```cpp
    motor.set_current_limit(500);
```
替换为：
```cpp
    motor.set_vref_mv(1000);
```

- [ ] **Step 5: 运行测试验证**

Run: `cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && ctest --test-dir build/test`
Expected: 全部通过

- [ ] **Step 6: Commit**

```bash
git add App/Inc/motor_ctrl.hpp App/Src/motor_ctrl.cpp App/Src/app_instances.cpp Test/test_motor_ctrl.cpp
git commit -m "refactor: rename set_current_limit to set_vref_mv, use hw:: constants"
```

---

### Task 4: `encoder.cpp` — 编码器溢出常量

**Files:**
- Modify: `App/Src/encoder.cpp:20,28-29,32-33` — 替换 65536/65535

- [ ] **Step 1: 添加 include 并替换常量**

在 `encoder.cpp` 顶部添加 `#include "hw_constants.hpp"`。

将所有 `65536` 替换为 `hw::TIMER_PERIOD`，`65535` 替换为 `hw::TIMER_PERIOD - 1`：

Line 20: `return ovf * hw::TIMER_PERIOD + cnt;`
Line 28: `m_iOverflow = pos / hw::TIMER_PERIOD;`
Line 29: `TIM8->CNT = pos % hw::TIMER_PERIOD;`
Line 32: `m_iOverflow = (pos - (hw::TIMER_PERIOD - 1)) / hw::TIMER_PERIOD;`
Line 33: `TIM8->CNT = static_cast<uint16_t>(pos - m_iOverflow * hw::TIMER_PERIOD);`

- [ ] **Step 2: 运行测试验证**

Run: `cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && ctest --test-dir build/test`
Expected: 全部通过（test_encoder.cpp 中的测试应全部通过）

- [ ] **Step 3: Commit**

```bash
git add App/Src/encoder.cpp
git commit -m "refactor: replace encoder magic numbers with hw::TIMER_PERIOD"
```

---

### Task 5: `power_monitor.cpp` — 电压换算常量

**Files:**
- Modify: `App/Src/power_monitor.cpp:17` — 替换 24363 和 4095

- [ ] **Step 1: 添加 include 并替换常量**

在 `power_monitor.cpp` 顶部添加 `#include "hw_constants.hpp"`。

将 line 17：
```cpp
    return static_cast<uint32_t>(adc_value) * 24363 / 4095;
```
替换为：
```cpp
    return static_cast<uint32_t>(adc_value) * hw::VOLTAGE_SCALE_NUM / hw::ADC_RESOLUTION;
```

- [ ] **Step 2: 运行测试验证**

Run: `cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && ctest --test-dir build/test`
Expected: test_power_monitor 全部通过

- [ ] **Step 3: Commit**

```bash
git add App/Src/power_monitor.cpp
git commit -m "refactor: replace power_monitor magic numbers with hw:: constants"
```

---

### Task 6: `fram_storage.cpp` — SPI 指令和 GPIO 常量

**Files:**
- Modify: `App/Src/fram_storage.cpp:15,19,23,27,32,41,50,92-95,106` — 替换 GPIO 和 SPI 常量

- [ ] **Step 1: 添加 include 并替换 GPIO 常量**

在 `fram_storage.cpp` 顶部添加 `#include "hw_constants.hpp"`。

替换 GPIO 引脚（4 处）：
- Line 15: `HAL_GPIO_WritePin(GPIOB, hw::FRAM_CS_PIN, GPIO_PIN_RESET);`
- Line 19: `HAL_GPIO_WritePin(GPIOB, hw::FRAM_CS_PIN, GPIO_PIN_SET);`
- Line 23: `HAL_GPIO_WritePin(GPIOB, hw::FRAM_WP_PIN, GPIO_PIN_SET);`
- Line 27: `HAL_GPIO_WritePin(GPIOB, hw::FRAM_WP_PIN, GPIO_PIN_RESET);`

- [ ] **Step 2: 替换 SPI 指令常量**

- Line 32: `uint8_t cmd = hw::FRAM_CMD_WREN;`
- Line 41: `uint8_t hdr[3] = {hw::FRAM_CMD_WRITE, ...};`
- Line 50: `uint8_t hdr[3] = {hw::FRAM_CMD_READ, ...};`
- Line 95: `uint8_t hdr[3] = {hw::FRAM_CMD_WRITE, ...};` (emergency_save)
- Line 106: `uint8_t hdr2[3] = {hw::FRAM_CMD_WRITE, ...};` (emergency_save)

- [ ] **Step 3: 运行测试验证**

Run: `cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && ctest --test-dir build/test`
Expected: test_fram_storage 全部通过

- [ ] **Step 4: Commit**

```bash
git add App/Src/fram_storage.cpp
git commit -m "refactor: replace fram_storage magic numbers with hw:: constants"
```

---

### Task 7: `zoom_table.hpp/cpp` — Flash 偏移和角度常量

**Files:**
- Modify: `App/Inc/zoom_table.hpp:17` — 新增 `FULL_ROTATION_X100` 常量
- Modify: `App/Src/zoom_table.cpp:38,128,131,133,155,157-160,169,171-174` — 替换裸数字

- [ ] **Step 1: 在 zoom_table.hpp 中新增常量**

在 `zoom_table.hpp` 的 `HOME_OFFSET` 后面（line 17 后）添加：
```cpp
    static constexpr int32_t FULL_ROTATION_X100 = 36000; // 360° × 100
```

- [ ] **Step 2: 在 zoom_table.cpp 中添加 include 并替换常量**

在 `zoom_table.cpp` 顶部添加 `#include "hw_constants.hpp"`。

替换 `36000`：
- Line 38: `static_cast<int64_t>(angle_x100) * TOTAL_RANGE / FULL_ROTATION_X100`

替换 Flash 偏移量裸数字 `2`：
- `save_to_flash()` 中：
  - Line 128: `uint32_t addr = FLASH_ZOOM_ADDR + hw::FLASH_HALFWORD_SIZE;`
  - Line 131: `addr += hw::FLASH_HALFWORD_SIZE;`
  - Line 133: `addr += hw::FLASH_HALFWORD_SIZE;`

- `load_from_flash()` BUILD_TESTING 分支中：
  - Line 155: `uint32_t offset = hw::FLASH_HALFWORD_SIZE;`
  - Line 158: `offset += hw::FLASH_HALFWORD_SIZE;`
  - Line 160: `offset += hw::FLASH_HALFWORD_SIZE;`

- `load_from_flash()` else 分支中：
  - Line 169: `uint32_t offset = hw::FLASH_HALFWORD_SIZE;`
  - Line 172: `offset += hw::FLASH_HALFWORD_SIZE;`
  - Line 174: `offset += hw::FLASH_HALFWORD_SIZE;`

- [ ] **Step 3: 运行测试验证**

Run: `cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && ctest --test-dir build/test`
Expected: test_zoom_table 全部通过

- [ ] **Step 4: Commit**

```bash
git add App/Inc/zoom_table.hpp App/Src/zoom_table.cpp
git commit -m "refactor: replace zoom_table magic numbers with named constants"
```

---

### Task 8: `motor_task.hpp/cpp` — 业务常量提取

**Files:**
- Modify: `Tasks/Inc/motor_task.hpp` — 新增 3 个常量
- Modify: `Tasks/Src/motor_task.cpp:651,723,752,831` — 替换裸数字

- [ ] **Step 1: 在 motor_task.hpp 中新增常量**

在 `STALL_TEST_SPEEDS` 定义后面（line 63 后）添加：
```cpp
    static constexpr uint16_t STALL_TEST_ZOOM_X10  = 20;    // 2.0x 测试位置
    static constexpr uint16_t STALL_PRINT_INTERVAL = 500;   // ADC 打印间隔 (ms)
    static constexpr uint16_t HEARTBEAT_INTERVAL   = 5000;  // 心跳间隔 (ms)
```

- [ ] **Step 2: 替换 motor_task.cpp 中的裸数字**

- Line 651: `int32_t iTarget = m_pZoom->get_position(STALL_TEST_ZOOM_X10);`
- Line 723: `if (m_iStallDwellCount % STALL_PRINT_INTERVAL == 0) {`
- Line 752: `int32_t iTarget = m_pZoom->get_position(STALL_TEST_ZOOM_X10);`
- Line 831: `if (++iHeartbeat >= HEARTBEAT_INTERVAL) {`

- [ ] **Step 3: 运行测试验证**

Run: `cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && ctest --test-dir build/test`
Expected: 全部通过

- [ ] **Step 4: Commit**

```bash
git add Tasks/Inc/motor_task.hpp Tasks/Src/motor_task.cpp
git commit -m "refactor: extract motor_task magic numbers to named constants"
```

---

### Task 9: `self_test.cpp` — 删除错误代码 + 修正重复定义

**Files:**
- Modify: `App/Src/self_test.cpp:159-166,172,177-178,201-206,220,252-253,275,279-280`

- [ ] **Step 1: 添加 include**

在 `self_test.cpp` 顶部添加：
```cpp
#include "task_config.hpp"
```

- [ ] **Step 2: 删除 adc_to_current_ma lambda**

删除 `self_test.cpp:159-166`（注释 + lambda 定义）：
```cpp
    // --- Helper lambdas for ADC conversions ---
    // ADC->mA: R_sense=0.2 Ohm, Vref=3.3V, 12-bit ADC
    // V_sense = ADC * 3300mV / 4095
    // I = V_sense / R_sense = V_sense / 0.2 = V_sense * 5  (mV->mA when R in Ohm)
    // I_mA = ADC * 3300 * 5 / 4095 = ADC * 16500 / 4095
    auto adc_to_current_ma = [](uint16_t adc) -> uint32_t {
        return static_cast<uint32_t>(adc) * 16500 / 4095;
    };
```

- [ ] **Step 3: 删除电流 mA 变量，保留 ADC 原始值**

保留 `iCRaw`（line 170），仅删除 `iCma`（line 172）：
```cpp
    // 删除这行：
    uint32_t iCma = adc_to_current_ma(iCRaw);
```

修改 BASELINE 打印部分（line 200-206），删除 mA 显示：
```cpp
    // BASELINE
    {
        swo_printf(" BASELINE ..... %s  ADC=%u (max %u)\n",
                   m_stResult.aPass[1] ? "PASS" : "FAIL",
                   m_stResult.iMeasuredBaseline, BASELINE_MAX);
    }
```

修改 Protection Thresholds 打印（line 252-253），删除 mA 显示：
```cpp
        swo_printf(" Stall           ADC>=%u  confirm=%u ticks\n",
                   StallDetect::STALL_THRESHOLD,
                   StallDetect::STALL_CONFIRM_COUNT);
        swo_printf(" Overcurrent     ADC>=%u  confirm=%u ticks\n",
                   StallDetect::OVERCURRENT_THRESHOLD,
                   StallDetect::OVERCURRENT_CONFIRM);
```

修改 Live Readings 打印（line 279-280），改为原始 ADC 值：
```cpp
        swo_printf(" Current ADC     %u\n", iCRaw);
```

- [ ] **Step 4: 替换重复定义**

将 line 177-178：
```cpp
    constexpr uint16_t FW_VERSION = 0x1000;  // rsp::FW_VERSION
    constexpr uint16_t LENS_TYPE  = 0x0004;  // rsp::LENS_TYPE
```
替换为：
```cpp
    constexpr uint16_t FW_VERSION = rsp::FW_VERSION;
    constexpr uint16_t LENS_TYPE  = rsp::LENS_TYPE;
```

将 line 220：
```cpp
        constexpr int32_t HOMING_RETRACT = 8000;     // from MotorTask
```
替换为：
```cpp
        constexpr int32_t HOMING_RETRACT = homing::RETRACT_DISTANCE;
```

- [ ] **Step 5: 修正 STATE_NAMES 数组越界检查**

将 line 275：
```cpp
        const char* pState = (iStateIdx < 6) ? STATE_NAMES[iStateIdx] : "?";
```
替换为：
```cpp
        const char* pState = (iStateIdx < sizeof(STATE_NAMES) / sizeof(STATE_NAMES[0]))
                             ? STATE_NAMES[iStateIdx] : "?";
```

- [ ] **Step 6: 运行测试验证**

Run: `cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && ctest --test-dir build/test`
Expected: test_self_test 全部通过

- [ ] **Step 7: Commit**

```bash
git add App/Src/self_test.cpp
git commit -m "refactor: remove incorrect current conversion, fix duplicate constants in self_test"
```

---

### Task 10: `current_profile_diag.cpp` — 删除错误电流函数

**Files:**
- Modify: `Test/diag/current_profile_diag.cpp:80-83` — 删除 adc_to_ma 函数及调用

- [ ] **Step 1: 删除 adc_to_ma 函数并更新 10 处调用**

删除 line 80-83 的函数定义：
```cpp
// Convert ADC to mA (R_sense=0.2Ω)
uint32_t adc_to_ma(uint16_t adc) {
    return static_cast<uint32_t>(adc) * 16500 / 4095;
}
```

将以下 10 处 `adc_to_ma(xxx)` 调用替换为 `xxx`（直接使用 ADC 值），同时将对应的 printf 格式说明从 `mA` 改为 `ADC`：
- Line 137: `adc_to_ma(stBaseline.avg())` → `stBaseline.avg()`
- Line 265: `adc_to_ma(aConstFwd[s].avg())` → `aConstFwd[s].avg()`
- Line 289: `adc_to_ma(aConstRev[s].avg())` → `aConstRev[s].avg()`
- Line 343: `adc_to_ma(stStallRev.avg())` → `stStallRev.avg()`
- Line 388: `adc_to_ma(stStallFwd.avg())` → `stStallFwd.avg()`
- Line 396: `adc_to_ma(stBaseline.avg())` → `stBaseline.avg()`
- Line 402: `adc_to_ma(aConstFwd[s].avg())` → `aConstFwd[s].avg()`
- Line 406: `adc_to_ma(aConstRev[s].avg())` → `aConstRev[s].avg()`
- Line 411: `adc_to_ma(stStallRev.avg())` → `stStallRev.avg()`
- Line 414: `adc_to_ma(stStallFwd.avg())` → `stStallFwd.avg()`

- [ ] **Step 2: 编译验证**

Run: `cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test`
Expected: 编译通过（diag 文件可能不在测试构建中，确认无编译错误即可）

- [ ] **Step 3: Commit**

```bash
git add Test/diag/current_profile_diag.cpp
git commit -m "refactor: remove incorrect adc_to_ma from diagnostic tool"
```

---

### Task 11: 固件编译验证

- [ ] **Step 1: 固件编译**

Run: `cmake -B build/fw -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake && cmake --build build/fw`
Expected: 编译通过，无 warning

- [ ] **Step 2: 运行全部单元测试确认无回归**

Run: `cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && ctest --test-dir build/test --output-on-failure`
Expected: 全部通过

- [ ] **Step 3: 更新 verify.md**

在 `docs/phases/phase7-backlash-accuracy/verify.md` 中追加硬编码清理的测试结果。

- [ ] **Step 4: 验证无残留旧名称引用**

Run: `grep -rn "set_current_limit\|adc_to_current_ma\|adc_to_ma" App/ Tasks/ Test/ --include="*.cpp" --include="*.hpp"`
Expected: 无匹配结果

- [ ] **Step 5: Commit**

```bash
git add docs/phases/phase7-backlash-accuracy/verify.md
git commit -m "docs: add hardcode cleanup verification results"
```
