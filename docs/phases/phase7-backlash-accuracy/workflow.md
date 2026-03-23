# 自检诊断全流程工作描述

## 全量诊断链路总览

```
上电 / 0x60 命令
    │
    ▼
┌─────────────┐
│  1. 自检     │  电压→基线电流→FRAM→编码器方向
│  (SelfTest)  │  无电机运动 / 短距编码器验证
└──────┬──────┘
       │ PASS
       ▼
┌─────────────┐
│  2. 归零     │  快退→回弹→慢退→定位
│  (Homing)   │  建立编码器零点
└──────┬──────┘
       │ DONE
       ▼
┌─────────────┐
│  3. 间隙测量  │  8 轮正反运动
│  (Backlash)  │  测量齿轮箱回差
└──────┬──────┘
       │ DONE
       ▼
┌─────────────┐
│  4. 精度测试  │  8 趟全行程往返
│  (Accuracy)  │  验证定位精度
└──────┬──────┘
       │ DONE
       ▼
    IDLE (就绪)
```

### 触发条件

| 路径 | 条件 | 动作 |
|------|------|------|
| 全量诊断 | FRAM 无效 / version≠2 / 收到 0x60 命令 | 自检→归零→间隙→精度 |
| 仅归零 | FRAM 有效但 position_invalid | 归零（不带诊断） |
| 正常启动 | FRAM 有效 + position_valid + homing_done | 恢复位置，跳过全部 |

关键代码：`Tasks/Src/monitor_task.cpp` 行 47（FRAM version 检查）、行 111-123（0x60 触发）

---

## 阶段 1：自检 (SelfTest)

**源文件**：`App/Src/self_test.cpp`, `App/Inc/self_test.hpp`

5 个顺序检测项，任一失败则中止：

| 序号 | 阶段 | 检测内容 | 判定条件 | 电机运动 |
|------|------|---------|---------|---------|
| 1 | VOLTAGE | 电源电压 | ≥ POWER_DOWN_THRESHOLD | 无 |
| 2 | BASELINE | 空载电流 | < 50 ADC counts (~2.4mA) | 无 |
| 3 | FRAM_RW | FRAM 读写 | 地址 0x00FE 写 0xA5 后读回一致 | 无 |
| 4 | ENCODER_DIR | 编码器正转检测 | 正向移动 2000 counts，3s 内位移 > 500 | 正向短距 |
| 5 | ENCODER_DIR_REVERSE | 编码器反转检测（仅正转失败时） | 反向移动成功→翻转 TIM8 IC1 极性 | 反向短距 |

编码器方向检测使用标准 `move_to()` 加速曲线，速度范围 480~3600 (11%~84%)。

SWO 输出：自检报告包含电压/电流测量值、编码器状态、机械参数。

---

## 阶段 2：归零 (Homing)

**源文件**：`Tasks/Src/motor_task.cpp` `start_homing()` + `run_homing()`

4 个子阶段，建立编码器绝对零点：

### 2.1 HOMING_FAST — 快速反向找粗限位

```
方向：REVERSE（减小 encoder count）
目标：-1,000,000（不可达，靠堵转停止）
速度：1800 (42%)  [HOMING_FAST_SPEED]
限速：set_speed_limit(1800)
终止：堵转检测触发（电流 > STALL_THRESHOLD 持续 1s）
动作：到达机械负极限
```

### 2.2 HOMING_RETRACT — 正向回弹

```
触发：HOMING_FAST 堵转
动作：set_position(0)  ← 粗零点
方向：FORWARD
目标：+4096 counts  [HOMING_RETRACT_DISTANCE]
速度：1500 (35%)  [HOMING_SLOW_SPEED]
终止：到达目标位置 → SETTLING → IDLE
目的：离开机械限位，为精确二次接近留空间
```

### 2.3 HOMING_SLOW — 慢速反向找精限位

```
方向：REVERSE
目标：-1,000,000（不可达）
速度：1500 (35%)  [HOMING_SLOW_SPEED]
终止：堵转检测触发
目的：低速接近，获得精确限位位置
```

### 2.4 HOMING_SETTLE — 正向定位到安全距离

```
触发：HOMING_SLOW 堵转
动作：set_position(0)  ← 精零点（最终零点）
方向：FORWARD
目标：+800 counts  [HOMING_SETTLE_DISTANCE]
速度：1500 (35%)  [HOMING_SLOW_SPEED]
终止：到达目标 → SETTLING → IDLE
```

### 归零完成后

- 编码器位置 = 800（安全偏移量）
- 速度限制恢复：`set_speed_limit(MAX_SPEED=3600)`
- 间隙补偿开启：`set_backlash_enabled(true)`
- 发送 UART 响应：`A5 01 00 0F`（HOMING_DONE）
- 若 `bFullDiagnostics=true`：链式进入间隙测量

---

## 阶段 3：间隙测量 (Backlash Measure)

**源文件**：`Tasks/Src/motor_task.cpp` `start_backlash_measure()` + `run_backlash_measure()`

**测量原理**：在固定位置做 8 轮正反运动，测量齿轮箱回差。

**关键参数**：
- 测量位置：BL_MEASURE_MID = 107,000 counts（~0.5x 行程）
- 反向距离：BL_REVERSE_DIST = 4,096 counts
- 速度限制：1500 (35%) [HOMING_SLOW_SPEED]
- 间隙补偿：测量期间禁用

### 单轮流程（×8 轮）

```
1. MOVE_TO_MID     正向移动到 107,000 → 速度 1500 (35%)
2. SETTLE_MID      等待稳定 100ms → 记录参考位置
3. REVERSE         反向移动到 102,904 (107000-4096) → 速度 1500 (35%)
4. SETTLE_REV      等待稳定 100ms
5. FORWARD         正向返回 107,000 → 速度 1500 (35%)
6. SETTLE_FWD      等待稳定 100ms → 误差 = |实际位置 - 107000|
                   → 存入 m_aBLSamples[k]
```

### 间隙计算

```
iAvg = Σ(8个样本) / 8
iBacklash = (iAvg > DEADZONE) ? iAvg - DEADZONE : 0
    其中 DEADZONE = 50 counts
```

完成后：
- `set_backlash(iBacklash)` 设置到电机控制器
- 保存到 FRAM（通过 save queue）
- 若 `bFullDiagnostics`：链式进入精度测试

---

## 阶段 4：精度测试 (Accuracy Test)

**源文件**：`Tasks/Src/motor_task.cpp` `start_accuracy_test()` + `run_accuracy_test()`

**测试目的**：验证间隙补偿后的全行程定位精度。

**关键参数**：
- 起点：ACC_START_POS = 800 counts（0.6X）
- 终点：ACC_END_POS = 206,821 counts（~7.0X）
- 往返次数：8 趟（16 次定位）
- 速度：MAX_SPEED = 3600 (84%) 正常变倍加减速曲线
- 间隙补偿：**启用**

### 单趟流程（×8 趟）

```
1. MOVE_TO_END     正向移动到 206,821 → 速度 3600 (84%)
                   加速 480→3600 → 恒速 → 减速 → 制动 → 修正
2. SETTLE_END      等待稳定 100ms
                   → 误差 = 实际位置 - 206821 → 存入 m_aAccErrors[]
3. MOVE_TO_START   反向移动到 800 → 速度 3600 (84%)
                   间隙补偿：两阶段接近
                   Phase 1: 过冲到 (800 - backlash - 100) → REVERSE
                   Phase 2: 正向修正到 800 → 速度 1500 (35%)
4. SETTLE_RETURN   等待稳定 100ms
                   → 误差 = 实际位置 - 800 → 存入 m_aAccErrors[]
```

### 精度报告 (SWO)

```
============ ACCURACY TEST REPORT ============
Backlash: <测量值> counts
Trip 1 FWD: error=<counts>
Trip 1 REV: error=<counts>
... (16 行，8 趟 × 2 方向)
Cumulative drift: <counts>
Max error: <counts>
RESULT: PASS / FAIL
```

**判定标准**：所有误差 ≤ ±DEADZONE (50 counts) 为 PASS。

---

## 速度常量速查表

| 常量名 | CCR | 占空比 | 用途 |
|--------|-----|--------|------|
| MIN_SPEED | 480 | 11% | 加减速最低值（实际驱动不了齿轮箱） |
| MIN_CORRECTION_SPEED | 1500 | 35% | 修正运动 / Phase 2 接近的最低有效速度 |
| HOMING_SLOW_SPEED | 1500 | 35% | 归零慢速 / 间隙测量 |
| HOMING_FAST_SPEED | 1800 | 42% | 归零快速 |
| MAX_SPEED | 3600 | 84% | 正常变倍 / 精度测试 |
| PWM_ARR (制动) | 4266 | 100% | CH1=CH2=ARR → H 桥制动 |

**PWM 硬件**：TIM3, PSC=0, ARR=4266, 72MHz → 固定 16.87 KHz
**驱动芯片**：A4950 全桥 DMOS，直通型，MCU PWM 直接控制电机

---

## 保护机制

| 保护项 | 阈值 | 动作 |
|--------|------|------|
| 堵转检测 | 电流 > STALL_THRESHOLD 持续 1s | 停止 + 报告 0xE1 |
| 过流保护 | 电流 > OVERCURRENT_THRESHOLD 持续 100ms | 紧急停止 + 报告 0xE2 |
| 编码器堵转 | 无位移 500ms | 停止 |
| 消隐时间 | 启动后 200ms | 忽略堵转（避免启动电流误触发） |

---

## 关键文件

| 文件 | 职责 |
|------|------|
| `Tasks/Src/monitor_task.cpp` | 启动流程控制、FRAM 检查、触发诊断链 |
| `App/Src/self_test.cpp` | 自检 5 项（电压/电流/FRAM/编码器） |
| `Tasks/Src/motor_task.cpp` | 归零 + 间隙测量 + 精度测试 |
| `App/Src/motor_ctrl.cpp` | 电机控制（加减速、间隙补偿、修正循环） |
| `Tasks/Inc/task_config.hpp` | 命令码/响应码/队列消息定义 |











0x60	0xXXXX		设置镜头运行速度
0x61	保留字节		镜头运行速度+50RPM
0x62	保留字节		镜头运行速度-50RPM
0x63	0xXXXX		设置镜头运行最小速度
0x64	0xXXXX		设置镜头运行最大速度
0x65	0xXXXX		设置镜头自检






