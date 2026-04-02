# 硬编码清理设计规格

**日期：** 2026-03-21
**范围：** 产品代码（App/、Tasks/），测试代码后续单独处理

## 目标

消除产品代码中的 magic number，提取为命名常量，提高可维护性。

## 方案

分层常量管理（方案 A + C 结合）：

| 层级 | 文件 | 职责 |
|------|------|------|
| 硬件层 | `App/Inc/hw_constants.hpp`（新建） | MCU 硬件参数：ADC/DAC 分辨率、参考电压、编码器位宽、GPIO 引脚、SPI 指令、Flash 编程粒度 |
| 协议/任务层 | `Tasks/Inc/task_config.hpp`（已有） | 通信命令、响应码、任务优先级、归零参数 |
| 业务层 | 各模块 `.hpp`（已有） | 各模块业务常量（速度、阈值、距离等） |

## 新建文件：`App/Inc/hw_constants.hpp`

```cpp
#pragma once
#include <cstdint>

namespace zlens {
namespace hw {

// --- ADC / DAC ---
static constexpr uint16_t ADC_RESOLUTION    = 4095;   // 12-bit max
static constexpr uint16_t DAC_RESOLUTION    = 4095;   // 12-bit max
static constexpr uint16_t VREF_MV           = 3300;   // 参考电压 3.3V

// --- Encoder (TIM8, 16-bit) ---
static constexpr uint32_t TIMER_PERIOD      = 65536;  // 2^16

// --- FRAM SPI Commands ---
static constexpr uint8_t  FRAM_CMD_WREN     = 0x06;
static constexpr uint8_t  FRAM_CMD_WRITE    = 0x02;
static constexpr uint8_t  FRAM_CMD_READ     = 0x03;

// --- FRAM GPIO ---
static constexpr uint16_t FRAM_CS_PIN       = 1 << 12;  // PB12
static constexpr uint16_t FRAM_WP_PIN       = 1 << 11;  // PB11

// --- Flash Programming ---
static constexpr uint8_t  FLASH_HALFWORD_SIZE = 2;    // STM32 Flash 半字编程粒度 (bytes)

// --- Voltage Divider ---
// 推导：V_in = V_adc × (R_top + R_bot) / R_bot
//       V_mV = ADC × VREF_MV / ADC_RESOLUTION × (30000 + 4700) / 4700
//            = ADC × 24363 / ADC_RESOLUTION
static constexpr uint16_t VDIV_R_TOP        = 30000;  // 上分压电阻 30kΩ
static constexpr uint16_t VDIV_R_BOTTOM     = 4700;   // 下分压电阻 4.7kΩ
static constexpr uint16_t VOLTAGE_SCALE_NUM = VREF_MV * (VDIV_R_TOP + VDIV_R_BOTTOM) / VDIV_R_BOTTOM;

} // namespace hw
} // namespace zlens
```

## 已有文件修改：`Tasks/Inc/task_config.hpp`

新增 `homing` 命名空间（无 FreeRTOS 依赖），将归零相关机械参数从 `motor_task.hpp` 搬入：

```cpp
namespace zlens {
namespace homing {

static constexpr int32_t RETRACT_DISTANCE   = 4096;   // 粗定位后回退距离
static constexpr int32_t SETTLE_DISTANCE    = 2048;   // 归零稳定偏移 (= ZoomTable::HOME_OFFSET)
static constexpr int32_t FAR_DISTANCE       = 1000000; // 归零远距离移动

} // namespace homing
} // namespace zlens
```

`motor_task.hpp` 中原有的 `HOMING_RETRACT_DISTANCE`、`HOMING_SETTLE_DISTANCE`、`HOMING_FAR_DISTANCE` 删除，改为引用 `homing::` 命名空间。

## 各模块改动

### 1. `motor_ctrl.hpp/cpp` — 函数重命名

- `set_current_limit(uint16_t milliamps)` → `set_vref_mv(uint16_t mv)`
- 删除 `* 2` 换算，直接用 `hw::DAC_RESOLUTION / hw::VREF_MV` 转 DAC 值
- 调用处 `app_instances.cpp`：`set_current_limit(1000)` → `set_vref_mv(2000)`

### 2. `encoder.cpp` — 编码器溢出

- `65536` → `hw::TIMER_PERIOD`
- `65535` → `hw::TIMER_PERIOD - 1`

### 3. `power_monitor.cpp` — 电压换算

- `24363` → `hw::VOLTAGE_SCALE_NUM`
- `4095` → `hw::ADC_RESOLUTION`

### 4. `fram_storage.cpp` — SPI + GPIO

- `0x06`/`0x02`/`0x03` → `hw::FRAM_CMD_WREN`/`WRITE`/`READ`
- `1 << 12`/`1 << 11` → `hw::FRAM_CS_PIN`/`FRAM_WP_PIN`

### 5. `zoom_table.hpp/cpp` — Flash 偏移 + 角度常量

- 在 `ZoomTable` 类中新增（与 `TOTAL_RANGE` 并列）：
  - `static constexpr int32_t FULL_ROTATION_X100 = 36000;`（360° × 100）
- `zoom_table.cpp` 中 Flash 读写偏移的裸数字 `2` → `hw::FLASH_HALFWORD_SIZE`
- `36000` → `FULL_ROTATION_X100`

### 6. `motor_task.hpp/cpp` — 业务常量

- 在 `motor_task.hpp` 新增：
  - `STALL_TEST_ZOOM_X10 = 20`（2.0x 变倍位置）
  - `STALL_PRINT_INTERVAL = 500`（打印间隔 ms）
  - `HEARTBEAT_INTERVAL = 5000`（心跳间隔 ms）
- 删除归零距离常量，改为引用 `homing::RETRACT_DISTANCE` 等
- 替换 `motor_task.cpp` 中对应裸数字

### 7. `self_test.cpp` — 删除和修正

- **删除** `adc_to_current_ma` lambda 及 `print_report()` 中所有 mA 相关打印（ADC 只能监控电压，电流值不准确）
- **删除重复定义**：
  - `FW_VERSION` → 引用 `rsp::FW_VERSION`
  - `LENS_TYPE` → 引用 `rsp::LENS_TYPE`
  - `HOMING_RETRACT = 8000`（错误值）→ 引用 `homing::RETRACT_DISTANCE`
- `6`（STATE_NAMES 越界检查）→ 用数组长度推导

### 8. `Test/diag/current_profile_diag.cpp` — 诊断工具

- **删除** `adc_to_ma` 函数及其调用

## 不改动的部分

- `stall_detect.hpp` 中的阈值（`STALL_THRESHOLD=600` 等）：已是命名常量
- `motor_ctrl.hpp` 中的业务常量（`MAX_SPEED` 等）：已是命名常量
- `task_config.hpp` 中的协议常量：已是命名常量
- CRC16 算法中的多项式常量（`0xA001`、`0xFFFF`）：标准算法值，就地注释即可
- 测试代码中的硬编码值：后续单独清理
