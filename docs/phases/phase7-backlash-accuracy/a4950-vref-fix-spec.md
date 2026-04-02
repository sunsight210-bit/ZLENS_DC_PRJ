# 修复 A4950 VREF 未设置导致低占空比无法驱动电机

## 问题分析

PWM 占空比低于 30% 时电机完全不转。

**根因**: `app_init()` 中完成 DAC 硬件初始化后，从未调用 `set_current_limit()` 设置 VREF 电压。
DAC 输出默认 0V → A4950 VREF=0V → I_trip = 0V / (10 × 0.2Ω) = **0A** → 电机电流被立即截断。

高占空比（>30%）之所以能转，是因为高电压脉冲能在 A4950 内部比较器响应前产生足够力矩。

## 修改方案

### 唯一改动: `App/Src/app_instances.cpp`

在 `app_init()` 函数中，`g_Motor.init()` 之后（第 54 行后）添加一行：

```cpp
g_Motor.set_current_limit(500);  // VREF = 1.0V → I_trip = 500mA
```

### 不需要改动

- `motor_ctrl.hpp/cpp` — `set_current_limit()` 已实现且正确
- DAC 初始化 — CubeMX 配置已正确
- 自检流程 — 自检中的 baseline 电流采样会在 VREF 设置后执行，不受影响

## 验证计划

1. **单元测试**: 现有 `test_motor_ctrl.cpp` 已有 `set_current_limit(500)` 测试，无需新增
2. **固件编译**: `cmake --build build/fw`
3. **板上验证**:
   - 烧录后用万用表测量 PA5 电压应为 ~1.0V（500mA × 2 = 1000mV）
   - 低占空比（10%-20%）时电机应能转动
   - 自检 SWO 报告中 baseline 电流应正常（~50mA）
