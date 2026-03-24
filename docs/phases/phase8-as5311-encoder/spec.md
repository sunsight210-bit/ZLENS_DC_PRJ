# Phase 8: AS5311 外置线性编码器替换方案

## 背景

当前系统使用电机内置编码器（半闭环），编码器在电机轴端，齿轮间隙、传动误差无法被检测，精度受限。更换为 AS5311 线性磁编码器，直接测量输出端线性位移，实现全闭环控制，从根本上提升定位精度。

## AS5311 关键规格

| 参数 | 值 |
|------|-----|
| 类型 | 线性磁编码器 |
| 磁极对间距 | 2.0mm |
| 增量输出 | A/B/Index 正交，每极对 256 脉冲 |
| x4 解码后分辨率 | 1024 步/极对 = 512 步/mm ≈ 1.95µm/步 |
| 输出类型 | CMOS 推挽 |
| 供电 | 3.3V 或 5V |
| CSn | 已硬件拉低（启用增量输出） |
| INDEX | 每极对 1 个脉冲（每 2mm） |
| 最高速度 | 650mm/s（增量模式） |

## 机械安装

- AS5311 芯片固定在机身上
- 磁条安装在大齿轮（直径 41.5mm，周长 ≈ 130.38mm）
- 芯片与磁体间隙：0.3mm - 0.6mm

---

## 设计第 1 节：硬件层 — TIM8 → TIM4 编码器接口

### 引脚重映射

| 信号 | 旧引脚 | 新引脚 | 外设 |
|------|--------|--------|------|
| ENC_A | PC6 (TIM8_CH1) | PB6 (TIM4_CH1) | TIM4 编码器模式 |
| ENC_B | PC7 (TIM8_CH2) | PB7 (TIM4_CH2) | TIM4 编码器模式 |
| ENC_Z | PA9 (EXTI9) | PB8 (EXTI8) | GPIO 上升沿中断 |

### CubeMX 改动

- PB6/PB7: GPIO_Input (LENS_ID) → TIM4_CH1/CH2 编码器模式
- PB8: GPIO_Input (LENS_ID) → GPIO_EXTI8 上升沿中断
- PC6/PC7: 释放，TIM8 编码器功能关闭
- PA9: 释放 EXTI9
- **LENS_ID 功能废弃**: PB6/PB7/PB8 原为 LENS_ID0/1/2 预留引脚，代码中从未使用，直接替换
- **TIM4 无需 AFIO remap**: PB6/PB7 是 TIM4 的默认引脚映射，CubeMX 中确保 `TIM4_REMAP` 未设置

### TIM4 配置

- 模式: `TIM_ENCODERMODE_TI12` (AB 双沿 x4 倍频)
- ARR = 65535 (16 位满量程)
- IC 极性: 两通道均 `TIM_ICPOLARITY_RISING` (AS5311 标准正交输出，无需反相)
- 滤波器: `0x0F` (最大滤波，抗噪声)
- PB6/PB7 GPIO: 浮空输入 (AS5311 CMOS 推挽输出，无需上下拉)

### Encoder 类改动

当前 `encoder.cpp` 中所有寄存器访问直接使用 `TIM8->CNT`、`TIM8->DIER`、`TIM8->SR` 硬编码。改动方式：

- **全局替换**: `TIM8` → `TIM4`（简单直接，无需重构类接口）
- 溢出中断: `TIM8_UP_IRQHandler` → `TIM4_IRQHandler`
- Z 脉冲中断: PA9 (EXTI line 9) → PB8 (EXTI line 8)，两者都在 `EXTI9_5_IRQHandler` 中处理，中断回调中检查的引脚从 `GPIO_PIN_9` 改为 `GPIO_PIN_8`
- C-linkage wrapper `encoder_overflow_handler()`: 保持不变，只是 ISR 入口改变
- Mock 测试: `mock_TIM8_instance` → `mock_TIM4_instance`
- 类公共接口 (`get_position()`, `set_position()`, `handle_overflow()`, `handle_z_pulse()`) **不变**

---

## 设计第 2 节：位置参数重新标定

### AS5311 编码器计数特征

- 每极对 2.0mm = 1024 步 (x4 解码)
- 每 mm = 512 步
- 大齿轮周长 ≈ 130.38mm
- 预估总行程 ≈ 66,755 counts (实际以 homing 实测为准)

### TOTAL_RANGE 运行时化

当前 `ZoomTable::TOTAL_RANGE` 是 `static constexpr`，需改为运行时变量：

- `ZoomTable` 新增 `set_total_range(int32_t iRange)` 方法
- **初始化流程**: boot → homing 实测总行程 → `ZoomTable::set_total_range()` → zoom 命令生效
- **homing 前 zoom 命令**: 拒绝执行，返回 NOT_HOMED 错误（与现有逻辑一致，MonitorTask 在 homing 完成前不允许 zoom 命令）
- **默认估计值**: `constexpr int32_t ESTIMATED_TOTAL_RANGE = 66755`，用于非关键场景的 fallback
- `angle_to_position()` 改为使用 `m_iTotalRange` 成员变量

### 常量重新标定

| 常量 | 旧值 (电机编码器) | 新值 (AS5311) | 物理含义 |
|------|-------------------|---------------|---------|
| `TOTAL_RANGE` | 858,592 (constexpr) | homing 实测 (runtime) | 大齿轮满圈计数 |
| `HOME_OFFSET` | 2,048 | 512 | ~1mm 安全距离 |
| `DEADZONE` | 1,000 | 50 | ~100µm 定位容差 (初始保守值) |
| `FINE_DEADZONE` | 500 | 25 | ~50µm 精细制动 (初始保守值) |
| `POSITION_TOLERANCE` | 500 | 25 | ~50µm 定位验收 (初始保守值) |
| `DECEL_DISTANCE` | 15,000 | 2,560 | ~5mm 减速起始 (保守值，防过冲) |
| `CRAWL_DISTANCE` | 5,000 | 512 | ~1mm 爬行距离 |
| `SAFE_LIMIT_MIN` | 1,024 | 256 | ~0.5mm 硬限位安全制动 (< HOME_OFFSET，设计如此) |
| `BACKLASH_MARGIN` | 200 | 删除 | 全闭环不需要 |

> **调优策略**: DEADZONE/POSITION_TOLERANCE 初始值设为保守的 50/25（~100µm/50µm），板上验证稳定后逐步收紧至 10/5（~20µm/10µm）。若出现振荡/反复修正则回退。

### Backlash 补偿策略变更

**删除两阶段补偿逻辑** (overshoot → forward crawl)，改为全闭环直接逼近:

- `move_to()` 直接移向目标，AS5311 实时反馈实际位置
- 到达 DEADZONE 内即成功
- 超调时直接反向修正 (AS5311 能看到真实位置，双向修正均可)
- `MAX_CORRECTIONS` 保留 (改为 3 次)，作为异常检测手段

**MotorCtrl 清理以下 backlash 相关接口:**
- 删除: `set_backlash()`, `set_backlash_enabled()`, `get_backlash()`, `is_backlash_enabled()`
- 删除成员变量: `m_iBacklash`, `m_bBacklashEnabled`, `m_bPhase2Active`
- 删除常量: `BACKLASH_MARGIN`, `PHASE2_SPEED`
- `m_iFinalTarget` 不再需要（target 就是 final target）

### FRAM 存储影响

- `backlash_counts` / `backlash_valid` 字段废弃 (保留字段避免改 FRAM 版本号)
- `current_position`, `min_position`, `max_position` 数值范围缩至 ~67K，`int32_t` 足够

---

## 设计第 3 节：INDEX 脉冲周期校准

### 原理

AS5311 每经过一个磁极对 (2.0mm = 1024 步) 输出一个 INDEX 脉冲，可用于运行时检测累积误差。

### 实现方式

- PB8 EXTI 上升沿中断触发 `handle_z_pulse()`
- 每次 INDEX 中断时记录当前位置 `iZPos`
- 校验: `iZPos % 1024` 应为固定偏移值 (首次 INDEX 时记录为基准 `m_iZeroOffset`)
- 偏差超过阈值 (±4 步，~8µm) 时，设置 `m_bDriftDetected` 标志

> **阈值说明**: AS5311 增量输出有 ±1 LSB 的内插误差，加上运动中的机械振动，±2 步可能产生误报。初始阈值设为 ±4 步 (~8µm)，验证稳定后可收紧。

### Encoder 类新增接口

```cpp
// 已有
void handle_z_pulse();
int32_t get_z_position() const;

// 新增
bool is_drift_detected() const;   // 是否检测到漂移
void clear_drift_flag();           // 清除漂移标志
int16_t get_drift_error() const;   // 最近一次漂移偏差量 (steps)
```

新增成员变量:
```cpp
int16_t m_iZeroOffset = 0;        // 首次 INDEX 基准偏移
bool m_bZeroOffsetSet = false;    // 基准是否已记录
bool m_bDriftDetected = false;    // 漂移检测标志
int16_t m_iDriftError = 0;        // 最近漂移偏差
```

### MotorCtrl 集成

- `update()` 每次循环检查 `is_drift_detected()`
- 检测到漂移: 不中断运动，SWO 打印警告 + 设置状态位
- CommTask 通过现有状态查询命令上报漂移信息

### 不做的事

- 不自动修正位置 (漂移可能是磁条安装误差)
- 不阻断运动 (仅告警)
- 不增加通信协议命令 (利用现有状态位)

---

## 改动范围总结

| 层级 | 文件 | 改动内容 |
|------|------|---------|
| CubeMX | `ZLENS_DC.ioc` | PB6/PB7→TIM4, PB8→EXTI8, 释放 TIM8/PC6/PC7/PA9, 确保无 TIM4 remap |
| HAL 生成 | `main.c`, `stm32f1xx_it.c`, `main.h` | TIM4 init, IRQ handler, 引脚定义, 删除 LENS_ID 定义 |
| Encoder | `encoder.hpp`, `encoder.cpp` | TIM8→TIM4 全局替换, INDEX 校准逻辑, drift 检测接口 |
| MotorCtrl | `motor_ctrl.hpp`, `motor_ctrl.cpp` | 删除 backlash API/成员/常量, 删除两阶段逻辑, 更新运动常量, drift 检查 |
| MotorTask | `motor_task.hpp`, `motor_task.cpp` | 删除 backlash 测量流程, 更新 homing 常量 |
| ZoomTable | `zoom_table.hpp`, `zoom_table.cpp` | `TOTAL_RANGE` 从 constexpr 改为 runtime, 新增 `set_total_range()` |
| MonitorTask | `monitor_task.cpp` | 删除 backlash 校准启动逻辑, homing 完成后调用 `set_total_range()` |
| FRAM | `fram_storage.hpp` | backlash 字段标记废弃 |
| App Init | `app_instances.cpp` | TIM4 替换 TIM8 启动 |
| Mock | `mock_hal.hpp`, `mock_hal.cpp` | `mock_TIM8_instance` → `mock_TIM4_instance` |
| 测试 | `test_*.cpp` | 适配新常量、新接口、删除 backlash 相关测试 |
