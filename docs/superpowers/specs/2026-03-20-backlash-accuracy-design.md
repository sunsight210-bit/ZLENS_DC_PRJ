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

- 定位精度：反复运动后误差可控（具体阈值待间隙测量后确定）
- Zoom 档位准确：每次定位到 zoom 值无可感知偏差
- 长期稳定：多次循环后位置不漂移

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
| HOMING_SETTLE | 低速 | 正向前进 800 counts | 到达 0.6X 位置，设编码器零点 |

### 速度策略设计依据

- **中速粗定位 + 低速精定位**：避免全速撞击机械限位可能导致的反弹
- **4096 counts 脱离距离**：2 圈电机轴，远大于可能的反弹距离（100-300 counts），慢爬仅需约 0.4 秒
- **800 counts 偏移**：0.6X 位置距离机械负极限 800 counts，避免每次回零撞击极限影响使用寿命

### 涉及文件

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

### 运动逻辑

```
move_to(target):
  if target > current_pos:
      直接正向走到 target（无需补偿）
  if target == current_pos:
      不动
  if target < current_pos:
      第一段：反向走到 (target - backlash - margin)
      第一段到位后自动启动第二段：正向走到 target
```

### 参数

- `backlash`：间隙值（counts），从 FRAM 读取，自检时自动测量并存入
- `margin`：安全余量 = backlash × 0.5，确保完全消除间隙
- 未标定前默认 backlash = 200 counts

### MotorCtrl 内部状态变化

新增内部状态 `APPROACHING`：
- 第一段反向运动到位 → 进入 APPROACHING
- APPROACHING 状态自动启动第二段正向运动
- 第二段到位 → 回到 IDLE

### 涉及文件

- `App/Inc/motor_ctrl.hpp`
- `App/Src/motor_ctrl.cpp`
- `App/Inc/fram_storage.hpp`
- `App/Src/fram_storage.cpp`

---

## 模块四：自检新增间隙测量 + 精度诊断

在自检流程的回零完成后，新增两个测试项。

### BACKLASH_MEASURE（间隙测量）

| 步骤 | 动作 |
|------|------|
| 1 | 低速正向运动到行程中间位置（约 107,000 counts） |
| 2 | 记录停止位置 P1 |
| 3 | 低速反向运动一小段（如 4096 counts） |
| 4 | 停稳后，低速正向运动回到 P1 附近 |
| 5 | 记录实际到位位置 P2 |
| 6 | 间隙 = \|P2 - P1\|，重复 3 次取平均 |
| 7 | 存入 FRAM |

### ACCURACY_TEST（精度诊断）

| 步骤 | 动作 |
|------|------|
| 1 | 从 0.6X 正向运动到 7.0X，记录到位误差 |
| 2 | 从 7.0X 反向运动回 0.6X，记录到位误差 |
| 3 | 重复 3-5 次往返 |
| 4 | SWO 输出精度报告 |

ACCURACY_TEST 使用间隙补偿后的 `move_to()`，测试的是补偿后的残余误差，直接验证补偿效果。

### SWO 报告内容

- 测量的间隙值（counts）
- 每次到位误差（counts）
- 漂移方向（单向/随机）
- 最大残余误差（counts）

### 涉及文件

- `App/Inc/self_test.hpp`
- `App/Src/self_test.cpp`

---

## FRAM 参数变更

### 新增字段

- `backlash_counts`（int16）— 测量的齿轮间隙值

### 删除字段

- ~~`total_range`~~（不再需要运行时测量）
- ~~`soft_limit_offset`~~（不再使用软限位）

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
