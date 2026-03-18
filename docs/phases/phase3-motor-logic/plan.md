# 阶段 3：MotorTask 核心逻辑

## 目标
实现 MotorTask 的完整运动控制逻辑，包括归零校准、变倍定位、循环运动、堵转/过流保护。

## 涉及文件

| 文件 | 操作 | 说明 |
|------|------|------|
| Tasks/Src/motor_task.cpp | 修改 | 核心状态机实现 |
| Tasks/Inc/motor_task.hpp | 修改 | 公开接口与状态定义 |
| Test/test_motor_task.cpp | 修改 | 15 个测试用例 |

## 设计要点

### 状态机（6 个状态）

```
IDLE → HOMING_REVERSE → HOMING_RETRACT → HOMING_FORWARD → HOMING_TO_SOFT_MIN → IDLE
IDLE → MOVING → IDLE
IDLE → CYCLING (MOVING ↔ DWELLING) → IDLE
```

### 归零校准流程（5 步）

1. **HOMING_REVERSE** — 反向全速运动，直到堵转检测触发（找到负限位）
2. 堵转触发 → 编码器清零，转入 HOMING_RETRACT
3. **HOMING_RETRACT** — 正向移动 4096 counts（脱离限位）
4. 到达后转入 **HOMING_FORWARD** — 正向全速运动，直到堵转（找到正限位）
5. 堵转触发 → 记录 total_range，移动到 soft_min，归零完成

### 软限位保护
- SOFT_LIMIT_OFFSET = 200 counts（约 50 脉冲）
- 有效运动范围：[200, total_range - 200]
- 未归零时不执行软限位裁剪

### 循环运动
- 步进变倍：按 step 方向遍历 ZoomTable
- 驻留计时：每个位置停留 dwell_x100ms × 100ms
- 边界反弹：到达变倍表边界时自动反转方向

### 堵转/过流处理
- 归零中堵转 = 预期行为，驱动状态机前进
- 非归零中堵转 = 异常，紧急停车 + STALL_ALARM
- 过流 = 10ms 内触发紧急停车 + OVERCURRENT 报警

## 进度清单

- [x] 状态机实现（6 状态）
- [x] 归零校准 5 步流程
- [x] 变倍定位（SET_ZOOM）
- [x] 循环运动（CYCLE_START/STOP）
- [x] 堵转/过流保护
- [x] 软限位裁剪
- [x] Mock 测试 15 用例
- [x] 固件编译
- [x] 板上测试
