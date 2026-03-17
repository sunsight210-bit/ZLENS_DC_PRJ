# ZLENS_DC 项目规则

## 命名规范（必须遵循）

完整版见 `docs/coding-style.md`。

### 文件
- C++: `snake_case.hpp` / `snake_case.cpp`；C: `snake_case.h` / `snake_case.c`
- 命名空间: `snake_case`（如 `namespace zlens`）

### 类型
- 类（有方法）: `PascalCase` → `MotorCtrl`
- 结构体（纯数据）: `UPPER_SNAKE_CASE_S` → `ZOOM_ENTRY_S`
- 枚举: `UPPER_SNAKE_CASE_E` → `MOTOR_STATE_E`；枚举值: `UPPER_SNAKE_CASE`
- 联合体: `UPPER_SNAKE_CASE_U` → `PARAM_VALUE_U`
- 类型别名/模板参数: `PascalCase`

### 函数
- 方法/函数: `snake_case()` → `get_position()`
- bool 方法: `is_`/`has_`/`can_` 前缀 → `is_ready()`

### 变量：作用域 + 类型前缀 + PascalCase
- 类型前缀: `b`bool `c`char `i`int/uint `f`float `d`double `p`指针 `a`数组 `st`结构体 `e`枚举 (类实例无前缀)
- 作用域: (无)局部 `m_`成员 `s_`静态 `g_`全局
- 示例: `iPosition` `m_bReady` `m_eState` `s_iCount` `g_bRunning`
- 数组前缀 `a` 不与元素类型叠加: `m_aTable[16]`

### 常量与宏
- `constexpr` / `#define`: `UPPER_SNAKE_CASE` → `MAX_SPEED`

### 第三方库
- HAL / FreeRTOS / CMSIS: 保持原样，不重命名

### 补充
- 类成员顺序: `public` → `protected` → `private`
- 定点数单位后缀: `iZoom_x10`, `iTemp_x100`
- 头文件保护: `#pragma once`
