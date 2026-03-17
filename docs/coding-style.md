# ZLENS_DC 嵌入式 C++ 命名规范

> 本文档定义项目的完整命名与编码规范，适用于所有新增代码。
> 第三方库（HAL / FreeRTOS / CMSIS）保持其原有风格不变。

---

## 1 文件与命名空间

| 对象 | 规则 | 示例 |
|------|------|------|
| C++ 头文件 | snake_case + `.hpp` | `motor_ctrl.hpp` |
| C++ 源文件 | snake_case + `.cpp` | `motor_ctrl.cpp` |
| C 头文件 | snake_case + `.h` | `stm32f1xx_hal.h` |
| C 源文件 | snake_case + `.c` | `system_stm32f1xx.c` |
| 命名空间 | snake_case | `namespace zlens` |

**设计理由：** snake_case 文件名在 Linux/Windows/macOS 间无大小写歧义，与 STM32 HAL 库文件风格一致。C++ 专用扩展名 `.hpp`/`.cpp` 便于 CMake `file(GLOB)` 和编辑器语法区分。

---

## 2 类型命名

| 对象 | 规则 | 示例 |
|------|------|------|
| 类（有方法） | PascalCase | `MotorCtrl`, `CommProtocol` |
| 结构体（纯数据） | UPPER_SNAKE_CASE + `_S` 后缀 | `ZOOM_ENTRY_S`, `FRAM_PARAMS_S` |
| 枚举 | UPPER_SNAKE_CASE + `_E` 后缀 | `MOTOR_STATE_E`, `DIRECTION_E` |
| 枚举值 | UPPER_SNAKE_CASE | `IDLE`, `ACCELERATING` |
| 联合体 | UPPER_SNAKE_CASE + `_U` 后缀 | `PARAM_VALUE_U` |
| 类型别名 (using) | PascalCase | `using Callback = std::function<void()>` |
| 模板参数 | PascalCase | `typename T`, `typename Derived` |

**设计理由：**
- 类 vs 结构体通过命名风格即可一眼区分行为对象（PascalCase）与纯数据容器（UPPER_SNAKE）。
- 后缀 `_S` / `_E` / `_U` 来自嵌入式工业惯例（MISRA 友好），在代码中看到变量类型名时立即知道它是结构体、枚举还是联合体。

### 2.1 结构体 vs 类的判定

- **结构体 `_S`**：所有成员 public、无虚函数、无非平凡构造/析构。可以有简单的初始化构造函数。
- **类 PascalCase**：有 private/protected 成员、有方法逻辑、管理资源。

```cpp
// 纯数据 → 结构体
struct ZOOM_ENTRY_S {
    int32_t iPosition;
    int32_t iSpeed;
};

// 有行为 → 类
class MotorCtrl {
public:
    void move_to(int32_t iTarget);
private:
    int32_t m_iPosition;
};
```

---

## 3 函数命名

| 对象 | 规则 | 示例 |
|------|------|------|
| 方法 / 自由函数 | snake_case | `get_position()`, `move_to()` |
| bool 返回的方法 | `is_` / `has_` / `can_` 前缀 | `is_valid()`, `has_data()`, `can_move()` |

**设计理由：** snake_case 函数与 PascalCase 类形成视觉层次区分。bool 前缀使条件语句自然可读：`if (motor.is_ready())`。

---

## 4 变量命名：类型前缀 + PascalCase

本项目采用**改良匈牙利命名法**，组合顺序为：

```
作用域前缀 + 类型前缀 + PascalCase名称
```

### 4.1 类型前缀

| 前缀 | 类型 | 示例 |
|------|------|------|
| `b` | bool | `bReady` |
| `c` | char / unsigned char | `cData` |
| `i` | int / uint（所有位宽：8/16/32） | `iPosition`, `iCount` |
| `f` | float | `fVoltage` |
| `d` | double | `dPrecision` |
| `p` | 指针 | `pBuffer`, `pEntry` |
| `a` | 数组 | `aTable`, `aEntries` |
| `st` | 结构体实例 | `stEntry`, `stParams` |
| `e` | 枚举实例 | `eState`, `eDirection` |
| (无) | 类实例 | `Motor`, `Protocol` |

**注意事项：**
- 数组前缀 `a` 不与元素类型前缀叠加：`int32_t aTable[16]`（不是 ~~`aiTable`~~）。
- 指针前缀 `p` 叠加元素类型：`uint8_t* pBuffer`（p 表示指针）。但如果指针指向类实例，则只用 `p`：`MotorCtrl* pMotor`。
- 类实例无类型前缀，首字母大写即可区分。

### 4.2 作用域前缀

| 前缀 | 作用域 | 示例 |
|------|------|------|
| (无) | 局部变量 / 函数参数 | `iPosition`, `pBuffer` |
| `m_` | 私有/保护成员变量 | `m_iPosition`, `m_bReady` |
| `s_` | 静态变量 | `s_iCount`, `s_pInstance` |
| `g_` | 全局变量 | `g_bRunning`, `g_iSystemState` |

### 4.3 综合示例

```cpp
class MotorCtrl {
public:
    void move_to(int32_t iTarget);
    bool is_ready() const;
    int32_t get_position() const { return m_iPosition; }

private:
    int32_t         m_iPosition;        // 成员 int
    bool            m_bReady;           // 成员 bool
    uint8_t*        m_pBuffer;          // 成员指针
    MOTOR_STATE_E   m_eState;           // 成员枚举实例
    ZOOM_ENTRY_S    m_stEntry;          // 成员结构体实例
    CommProtocol    m_Protocol;         // 成员类实例（无类型前缀）
    float           m_fBaseline;        // 成员 float
    int32_t         m_aTable[16];       // 成员数组

    static int32_t  s_iInstanceCount;   // 静态 int
    static MotorCtrl* s_pInstance;      // 静态指针
};

// 全局变量
bool    g_bRunning = false;
int32_t g_iSystemState = 0;

// 函数参数和局部变量
void process_data(uint8_t* pData, int32_t iLength) {
    int32_t iIndex = 0;
    bool    bFound = false;
    ZOOM_ENTRY_S stResult{};
    // ...
}
```

---

## 5 常量与宏

| 对象 | 规则 | 示例 |
|------|------|------|
| `constexpr` 常量 | UPPER_SNAKE_CASE，放命名空间内 | `MAX_SPEED`, `FRAME_HEADER` |
| 宏 `#define` | UPPER_SNAKE_CASE | `USE_SWO`, `STM32F103xE` |

```cpp
namespace zlens {
    constexpr int32_t MAX_SPEED      = 5000;
    constexpr uint8_t FRAME_HEADER   = 0xFF;
    constexpr float   GEAR_RATIO     = 3.5f;
}

#define USE_SWO          1
#define STM32F103xE
```

**设计理由：** `constexpr` 优先于 `#define`（类型安全、有作用域）。两者都用 UPPER_SNAKE_CASE，在代码中「常量」一目了然。

---

## 6 第三方库

| 库 | 规则 | 示例 |
|----|------|------|
| HAL | 保持原样 | `HAL_GPIO_WritePin()`, `GPIO_TypeDef` |
| FreeRTOS | 保持原样 | `xQueueSend()`, `vTaskDelay()` |
| CMSIS | 保持原样 | `NVIC_EnableIRQ()`, `SysTick_Handler()` |

不对第三方库做包装重命名。在自己的代码中调用时直接使用其原生 API 名称。

---

## 7 补充规则

### 7.1 类成员顺序

```cpp
class Example {
public:      // 1. 公有接口
protected:   // 2. 保护成员
private:     // 3. 私有实现
};
```

### 7.2 数值单位后缀

当整数表示定点数或有特殊缩放时，用 `_x10`、`_x100` 等后缀标注：

```cpp
int32_t iZoom_x10 = 145;    // 实际值 14.5，放大 10 倍存储
int32_t iTemp_x100 = 2537;  // 实际值 25.37°C
```

### 7.3 头文件保护

统一使用 `#pragma once`，不使用传统 include guard。

```cpp
#pragma once

#include <cstdint>
// ...
```

### 7.4 枚举推荐用法

优先使用 `enum class`（强类型），需要与 C 兼容或位运算时可用普通 `enum`：

```cpp
// 推荐：强类型枚举
enum class MOTOR_STATE_E : uint8_t {
    IDLE,
    ACCELERATING,
    CONSTANT_SPEED,
    DECELERATING,
    ERROR
};

// 使用
MOTOR_STATE_E eState = MOTOR_STATE_E::IDLE;
```

---

## 8 边界情况 FAQ

**Q: 回调函数指针变量用什么前缀？**
A: 用 `p` 前缀，因为函数指针本质是指针：`void (*pOnComplete)(int32_t)`。

**Q: `std::array` 或 `std::vector` 用什么前缀？**
A: 用 `a` 前缀（语义上是数组/容器）：`std::array<int32_t, 8> aBuffer`。

**Q: `volatile` 变量需要特殊前缀吗？**
A: 不需要额外前缀，但建议加注释说明原因：
```cpp
volatile int32_t m_iEncoderCount;  // ISR 中更新
```

**Q: ISR 回调函数命名？**
A: 遵循 CMSIS/HAL 命名（`HAL_UART_RxCpltCallback`），不做修改。自定义的处理函数用 snake_case：`handle_uart_rx()`。

**Q: 单例模式的静态实例？**
A: `s_pInstance`（静态 + 指针前缀）。

**Q: const 引用参数？**
A: 与值参数相同规则，按照实际含义选前缀：
```cpp
void set_params(const ZOOM_ENTRY_S& stEntry);  // st 前缀
void set_name(const char* pName);               // p 前缀
```
