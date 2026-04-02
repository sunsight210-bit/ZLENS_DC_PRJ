# 回零编码器堵转检测设计规格

**日期：** 2026-03-21
**范围：** `Tasks/Src/motor_task.cpp`

## 问题

回零流程依赖 ADC 电流堵转检测（`STALL_THRESHOLD=600`，确认 1000ms）。
`HOMING_FAST_SPEED=1200`（~28% 占空比）时，PWM 与 ADC 采样不同步，
EMA 滤波后的电流值无法稳定超过阈值 1000ms，导致回零卡在 `HOMING_FAST` 阶段。

SWO 日志证据：电机到达机械限位后 position 不再变化，但始终未触发 `handle_stall()`。

## 方案

在 `run_once()` 中增加编码器堵转检测，仅限 `HOMING_FAST` 和 `HOMING_SLOW` 阶段。

### 改动

`Tasks/Src/motor_task.cpp` `run_once()` 方法，在现有 ADC 堵转检测之后加入：

```cpp
// 编码器堵转（仅 homing 阶段 — 撞限位时编码器必定停转）
if (m_pStall->encoder_stalled() &&
    (m_eTaskState == TASK_STATE_E::HOMING_FAST ||
     m_eTaskState == TASK_STATE_E::HOMING_SLOW)) {
    handle_stall();
    return;
}
```

### 设计要点

- 仅 `HOMING_FAST` 和 `HOMING_SLOW` 启用：这两个阶段需要撞限位检测
- `HOMING_RETRACT` 和 `HOMING_SETTLE` 不启用：定距移动，不需要限位检测
- 复用现有 `handle_stall()`：已正确处理各 homing 子状态转换
- 不改 `StallDetect` 类：`encoder_stalled()` 已实现，只是未被调用
- 不影响正常运行：`MOVING` 状态不检查编码器堵转

### 不改动的部分

- `stall_detect.hpp/cpp`：阈值和逻辑不变
- 正常运行堵转检测：保持纯电流检测
- `ENCODER_STALL_TICKS=500`（500ms 无编码器变化即判定堵转）

## 测试

- 单元测试：模拟 homing 阶段编码器不变 500+ ticks，验证触发 `handle_stall()`
- 板上测试：SWO 确认回零完整走完 4 阶段（fast → retract → slow → settle）
