# ZLENS_DC 齿轮间隙补偿与精度优化设计

> 日期：2026-03-20
> 状态：已确认

## 背景

ZLENS_DC 电动变倍镜头控制系统在实际使用中存在两个定位精度问题：

1. **齿轮间隙（Backlash）** — 换向时电机轴已转但输出轴未动，导致到位后实际位置偏差
2. **累计误差** — 多次换向后间隙误差累积，位置逐渐偏离目标

同时，回零和自检流程存在设计冗余，需要一并重构。

## 设计目标

- 消除换向时的齿轮间隙导致的定位偏差
- 消除多次往返后的累计误差
- 简化回零流程（去掉不必要的正极限测量）
- 解耦自检与回零（职责分离）
- 新增间隙自动测量和精度诊断能力

## 验收标准

- 定位精度：补偿后残余误差 < DEADZONE（50 counts），即 < 0.085° 输出轴角度
- Zoom 档位准确：每次定位到 zoom 值无可感知偏差
- 长期稳定：3-5 次全程往返后位置不漂移

---

## 模块一：回零流程简化

### 现有流程（5 阶段）

HOMING_REVERSE → HOMING_RETRACT → HOMING_FORWARD → HOMING_TO_SOFT_MIN → IDLE

问题：
- 正向找正极限（HOMING_FORWARD）不必要 — 行程由机械结构决定（0°~347°），无需运行时测量 total_range
- 软限位计算（HOMING_TO_SOFT_MIN）不必要 — 机械限位本身就是保护

### 新流程（4 阶段）

| 阶段 | 速度 | 动作 | 结束条件 |
|------|------|------|----------|
| HOMING_FAST | 中速（~50% duty） | 反向找负极限 | 堵转检测 |
| HOMING_RETRACT | 低速 | 正向脱离 4096 counts | 到位 |
| HOMING_SLOW | 低速 | 反向精确找负极限 | 堵转检测 |
| HOMING_SETTLE | 低速 | 正向固定 800 counts | 到位，设编码器零点 |

### 速度控制机制

当前 `MotorCtrl::move_to()` 始终使用梯形加减速（MIN_SPEED → MAX_SPEED → MIN_SPEED）。
回零阶段需要不同速度，采用以下方案：

- 新增 `MotorCtrl::set_speed_limit(uint16_t iMaxSpeed)` 方法，临时限制最大速度
- HOMING_FAST 阶段：`set_speed_limit(MAX_SPEED / 2)` 约 50% duty
- HOMING_RETRACT/SLOW/SETTLE 阶段：`set_speed_limit(MIN_SPEED)` 低速运行
- 回零完成后：`set_speed_limit(MAX_SPEED)` 恢复默认

梯形加减速逻辑不变，只是加速上限被限制。

### 速度策略设计依据

- **中速粗定位 + 低速精定位**：避免全速撞击机械限位可能导致的反弹
- **4096 counts 脱离距离**：2 圈电机轴（现有代码 `HOMING_RETRACT_DISTANCE = 8000`，减为 4096 是因为不再需要跨越 Z 脉冲区域，仅需脱离反弹范围），慢爬仅需约 0.4 秒
- **800 counts 偏移**：固定距离，近似 0.6X 位置，避免每次回零撞击极限影响使用寿命。注意：这不是从 zoom table 查表得出，而是固定的机械保护偏移

### 回零期间间隙补偿

**回零期间必须禁用间隙补偿。** 回零的 HOMING_SLOW 阶段需要真正的反向运动去接触机械限位，如果间隙补偿介入会将其转为"反向→正向趋近"，永远无法触达极限。

实现方式：`MotorCtrl` 提供 `set_backlash_enabled(bool)` 开关，MotorTask 在回零前禁用、回零后启用。

### 涉及文件

- `App/Inc/motor_ctrl.hpp` — 新增 `set_speed_limit()`、`set_backlash_enabled()`
- `App/Src/motor_ctrl.cpp`
- `Tasks/Inc/motor_task.hpp`
- `Tasks/Src/motor_task.cpp`

---

## 模块二：自检与回零解耦

### 设计原则

- **自检**只回答"硬件是否正常"（电气/存储检测）
- **回零**只回答"零点在哪"（机械定位）
- 两者独立，可单独触发

### 自检精简为 4 项

| 项目 | 检测内容 | 失败含义 |
|------|----------|----------|
| VOLTAGE | 供电电压正常 | 电源异常 |
| BASELINE | 空载电流 < 50 ADC | 电机/驱动接线异常 |
| FRAM_RW | FRAM 读写验证 | 存储芯片故障 |
| ENCODER_DIR | 编码器方向检测（含极性补偿） | 编码器接线反 |

去掉的项目：
- ~~HOMING（回零不再属于自检）~~
- ~~RANGE_CHECK（不再测量 total_range）~~
- ~~LIMITS_CHECK（不再计算软限位）~~
- ~~FRAM_SAVE（回零完成后由 MotorTask 保存）~~

### 上电流程

| 条件 | 行为 |
|------|------|
| FRAM 无效（首次上电） | 自检 → 回零 → 保存 FRAM |
| FRAM 有效，`position_valid = 0`（异常断电） | 回零 → 保存 FRAM |
| FRAM 有效，`position_valid = 0xFF` | 恢复位置，直接 READY |
| 收到 0x60 自检命令 | 自检 → 回零 |

### 0x60 命令编排

收到 0x60 命令时，由 MonitorTask 编排两步流程：
1. MonitorTask 启动 SelfTest（4 项电气检测）
2. SelfTest 完成且全通过 → MonitorTask 通知 MotorTask 启动回零
3. 回零完成 → MotorTask 执行间隙测量和精度诊断（模块四）
4. 全部完成 → MotorTask 保存 FRAM 并通知 MonitorTask 发送结果

### 失败处理

- 自检失败 → 不启动回零，系统报错
- 自检通过但回零失败 → 系统报机械故障

### 涉及文件

- `App/Inc/self_test.hpp`
- `App/Src/self_test.cpp`
- `Tasks/Inc/monitor_task.hpp`
- `Tasks/Src/monitor_task.cpp`
- `Tasks/Src/motor_task.cpp`

---

## 模块三：齿轮间隙补偿 — 单向趋近法

### 核心规则

所有定位最终从正方向（position 增大方向）趋近目标，消除换向导致的齿轮间隙。

### 实现位置

在 `MotorCtrl` 层面内部处理，外部调用者（MotorTask、SelfTest）无感知。
可通过 `set_backlash_enabled(false)` 禁用（回零和间隙测量时需要禁用）。

### 运动逻辑

```
move_to(target):
  if !backlash_enabled:
      直接走到 target（原始行为）
  if target > current_pos:
      直接正向走到 target（无需补偿）
  if target == current_pos:
      不动
  if target < current_pos:
      第一段：反向走到 (target - backlash - backlash / 2)
      第一段到位后自动启动第二段：正向走到 target
```

### 参数

- `backlash`：间隙值（counts），从 FRAM 读取，自检时自动测量并存入
- `margin`：安全余量 = backlash / 2（整数除法，截断）
- 未标定前默认 backlash = 200 counts

### MotorCtrl 内部状态变化

新增内部状态 `APPROACHING`（`MOTOR_STATE_E` 枚举新增值）：
- 第一段反向运动到位 → 进入 APPROACHING
- APPROACHING 状态自动启动第二段正向运动
- 第二段到位 → 回到 IDLE

### APPROACHING 与堵转/过流的交互

- **过流**：APPROACHING 期间发生过流 → `emergency_stop()`，状态直接到 IDLE，取消第二段运动。MotorTask 按正常过流处理。
- **堵转**：APPROACHING 期间发生堵转 → `emergency_stop()`，状态直接到 IDLE。说明运动路径上有障碍，取消补偿运动。
- **MotorTask 视角**：MotorTask 通过 `get_state()` 可以看到 `APPROACHING` 状态。当 `APPROACHING` 期间收到堵转/过流通知时，MotorTask 按 MOVING 状态的堵转/过流逻辑处理（报告错误并停止）。

### 涉及文件

- `App/Inc/motor_ctrl.hpp`
- `App/Src/motor_ctrl.cpp`
- `App/Inc/fram_storage.hpp`
- `App/Src/fram_storage.cpp`

---

## 模块四：自检新增间隙测量 + 精度诊断

在回零完成后，由 MotorTask 执行。间隙测量和精度诊断都属于 0x60 自检命令的扩展流程。

### BACKLASH_MEASURE（间隙测量）

**重要：间隙测量必须禁用间隙补偿**（`set_backlash_enabled(false)`），使用原始 `move_to()` 避免循环依赖。

| 步骤 | 动作 |
|------|------|
| 1 | 禁用间隙补偿 |
| 2 | 低速正向运动到行程中间位置（约 107,000 counts），确保从正方向到达 |
| 3 | 记录停止位置 P1 |
| 4 | 低速反向运动 4096 counts |
| 5 | 低速正向运动，目标 P1 |
| 6 | 记录实际到位位置 P2 |
| 7 | 本次间隙 = P2 - P1（含 DEADZONE 误差，后续减去 DEADZONE 得到纯间隙值） |
| 8 | 重复步骤 4-7 共 3 次，取平均 |
| 9 | backlash = max(average - DEADZONE, 0)，存入 FRAM |
| 10 | 启用间隙补偿，使用新测量的 backlash 值 |

注：步骤 7 中 P2 - P1 包含两部分：实际齿轮间隙 + DEADZONE 到位误差。步骤 9 中减去 DEADZONE 分离出纯间隙值。

### ACCURACY_TEST（精度诊断）

**使用间隙补偿后的 `move_to()`**，验证补偿效果。

| 步骤 | 动作 |
|------|------|
| 1 | 启用间隙补偿（使用刚测量的 backlash 值） |
| 2 | 正向运动到 0.6X 位置（800 counts），记录参考位置 R |
| 3 | 正向运动到 7.0X 位置，记录到位误差 |
| 4 | 反向运动回 0.6X 位置，记录到位误差 |
| 5 | 重复步骤 3-4 共 3-5 次 |
| 6 | 最后一次到位后，与参考位置 R 比较，计算累计漂移 |
| 7 | SWO 输出精度报告 |

### 精度诊断通过/失败标准

- **通过**：补偿后每次到位残余误差 < DEADZONE（50 counts），且 3-5 次往返后累计漂移 < DEADZONE
- **失败**：残余误差或累计漂移超过 DEADZONE。失败不阻断系统运行，仅 SWO 报告警告

### SWO 报告内容

- 测量的间隙值（counts 和输出轴角度）
- 每次到位误差（counts）
- 漂移方向（单向/随机）
- 最大残余误差（counts）
- 通过/失败判定

### 涉及文件

- `Tasks/Inc/motor_task.hpp`
- `Tasks/Src/motor_task.cpp`

---

## FRAM 参数变更

### 新增字段

- `backlash_counts`（int16）— 测量的齿轮间隙值
- `backlash_valid`（uint8）— 0x00 = 未标定（使用默认值 200），0xFF = 已标定

### 删除字段

- ~~`total_range`~~（不再需要运行时测量）
- ~~`soft_limit_offset`~~（不再使用软限位）

### FRAM 版本迁移

- `version` 字段从 1 递增为 2
- 旧版 FRAM 数据（version = 1）视为无效，触发首次上电流程（自检 → 回零）
- 不做新旧格式兼容迁移

### 涉及文件

- `App/Inc/fram_storage.hpp`
- `App/Src/fram_storage.cpp`

---

## Z 脉冲运行时校准

**暂不实现。**

理由：
- 编码器在电机轴上，Z 脉冲也在电机轴上，只能校准编码器计数本身的误差
- 累计误差的实际来源是齿轮间隙在多次换向中的累积，Z 脉冲无法检测
- 当前系统的编码器计数可靠性高（TIM8 硬件解码 + 优先级 0 溢出中断），编码器计数误差发生概率极低
- 作为未来可选的安全网保留，当前优先级不高

---

## 不涉及的内容

- 上位机协议变更（0x60 命令复用，无需新增命令）
- Zoom table 映射逻辑（保持不变）
- 掉电保护流程（保持不变，FRAM 存储位置字段不变）
- 电机 PWM/梯形加减速参数（保持不变）
