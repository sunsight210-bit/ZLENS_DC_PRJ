# ZLENS_DC 电动变倍镜头控制系统 - 设计文档

## Context

为 STM32F103RCT6 电动变倍镜头控制系统创建完整的嵌入式 C++ 工程。项目当前仅有需求文档和参考资料，无任何源代码。需要从零搭建 CMake 构建系统、FreeRTOS 多任务架构、HAL 驱动集成，以及全部应用层代码。

---

## 1. 技术选型

| 项目 | 选型 |
|------|------|
| 构建系统 | CMake + Ninja |
| HAL 库 | STM32CubeMX 生成 |
| 架构 | FreeRTOS 多任务 (4任务) |
| 运动控制 | 梯形加减速 |
| 堵转检测 | 固定阈值 ADC>200（首次标定时采集 baseline_current 存入 FRAM，预留动态阈值扩展） |
| 掉电恢复 | FRAM + Z脉冲校验 |
| 编程语言 | C++ (App/Tasks层), C (HAL/驱动层) |
| 编译器 | arm-none-eabi-gcc 14.3 (STM32CubeIDE 2.1.0 自带) |
| 开发方法 | TDD（测试驱动开发），每阶段测试覆盖率 >80% |
| 测试框架 | Google Test (gtest) — 主机端单元测试，HAL Mock 层隔离硬件 |

**编译器路径：**
`/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin/`

**CMake 路径：**
`/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cmake.linux64_1.1.100.202601091506/tools/bin/cmake`

**Ninja 路径：**
`/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.ninja.linux64_1.1.100.202601091506/tools/bin/ninja`

---

## 2. 项目目录结构

```
ZLENS_DC/
├── CMakeLists.txt                    # 顶层 CMake
├── cmake/
│   ├── stm32f103rc.cmake             # 交叉编译工具链文件
│   └── stm32f1xx.ld                  # 链接脚本
├── Core/                             # CubeMX 生成
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f1xx_hal_conf.h
│   │   └── FreeRTOSConfig.h
│   ├── Src/
│   │   ├── main.cpp
│   │   ├── stm32f1xx_it.cpp
│   │   └── system_stm32f1xx.c
│   └── Startup/
│       └── startup_stm32f103rctx.s
├── Drivers/                          # CubeMX 生成
│   ├── STM32F1xx_HAL_Driver/
│   ├── CMSIS/
│   └── Third_Party/FreeRTOS/
├── App/                              # 应用层（核心业务代码）
│   ├── Inc/
│   │   ├── motor_ctrl.hpp
│   │   ├── encoder.hpp
│   │   ├── stall_detect.hpp
│   │   ├── zoom_table.hpp
│   │   ├── comm_protocol.hpp
│   │   ├── fram_storage.hpp
│   │   ├── power_monitor.hpp
│   │   └── system_manager.hpp
│   └── Src/
│       ├── motor_ctrl.cpp
│       ├── encoder.cpp
│       ├── stall_detect.cpp
│       ├── zoom_table.cpp
│       ├── comm_protocol.cpp
│       ├── fram_storage.cpp
│       ├── power_monitor.cpp
│       └── system_manager.cpp
├── Tasks/                            # FreeRTOS 任务入口
│   ├── Inc/
│   │   ├── motor_task.hpp
│   │   ├── comm_task.hpp
│   │   ├── storage_task.hpp
│   │   └── monitor_task.hpp
│   └── Src/
│       ├── motor_task.cpp
│       ├── comm_task.cpp
│       ├── storage_task.cpp
│       └── monitor_task.cpp
├── Test/                             # 单元测试（主机端 gtest）
│   ├── CMakeLists.txt                # 测试构建配置
│   ├── mocks/                        # HAL Mock 层
│   │   ├── mock_hal.hpp              # STM32 HAL 函数 mock
│   │   ├── mock_hal.cpp
│   │   ├── mock_freertos.hpp         # FreeRTOS API mock
│   │   └── mock_freertos.cpp
│   ├── test_encoder.cpp
│   ├── test_motor_ctrl.cpp
│   ├── test_stall_detect.cpp
│   ├── test_zoom_table.cpp
│   ├── test_comm_protocol.cpp
│   ├── test_fram_storage.cpp
│   ├── test_power_monitor.cpp
│   └── test_system_manager.cpp
└── docs/
```

---

## 3. FreeRTOS 任务架构

### 3.1 任务列表

| 任务 | 优先级 | 栈大小 | 周期 | 职责 |
|------|--------|--------|------|------|
| MotorTask | 4 (最高) | 512 words | 1ms | 电机控制、编码器读取、堵转检测、软限位保护、归零标定 |
| CommTask | 3 | 256 words | 事件触发 | UART DMA 接收、帧解析、CRC校验、应答反馈 |
| StorageTask | 2 | 256 words | 事件触发 | FRAM 定期保存(500ms)、到位保存、堵转保存、上电恢复 |
| MonitorTask | 1 | 256 words | 1ms | 系统状态机、自检、看门狗 |

**掉电检测方案**：不依赖 MonitorTask 任务调度，改用 **ADC 模拟看门狗中断（ADC_AWD）** 硬件级触发。配置 ADC1 看门狗监视 PB0(IN8) 通道，低阈值设为 3103（对应 10V），电压低于阈值触发 ADC_IRQHandler，在中断中直接执行紧急保存（刹车 + 读位置 + FRAM写入），不经过任何任务队列。此方案不受 FreeRTOS 任务优先级影响，响应延迟 < 100μs。

### 3.2 任务间通信

| 来源 | 目标 | 机制 | 数据 |
|------|------|------|------|
| CommTask → MotorTask | xQueue (cmdQueue, 深度8) | 命令字+参数 |
| MotorTask → CommTask | xQueue (rspQueue, 深度8) | 执行结果/反馈帧 |
| MotorTask → StorageTask | xQueue (saveQueue, 深度4) | 位置/参数快照 |
| MonitorTask → MotorTask | xTaskNotify | 紧急停止信号 |
| MonitorTask → StorageTask | xTaskNotify | 紧急保存信号 |
| ISR → MotorTask | xTaskNotifyFromISR | 编码器/ADC事件 |

### 3.3 掉电紧急路径（ADC 看门狗中断级）

ADC_AWD 中断触发（PB0 < 3103）→ 刹车(IN1=IN2=1) → 读 TIM8 位置 → FRAM 紧急写入(position + valid) → ~50μs 完成。此路径在硬件中断中执行，不依赖 FreeRTOS 任务调度。

### 3.4 中断优先级分配

| 中断 | NVIC 优先级 | 说明 |
|------|------------|------|
| TIM8_UP | 0 (最高，高于 configMAX_SYSCALL_INTERRUPT_PRIORITY) | 编码器溢出计数，不可被 RTOS 临界区屏蔽 |
| EXTI9 | 1 | Z 脉冲捕获 |
| ADC1 (AWD) | 2 | 掉电检测紧急保存 |
| DMA1_CH6 (UART2_RX) | 5 | 串口 DMA 接收 |
| TIM3 | 6 | PWM 更新（通常不需要中断） |
| SysTick | 15 | FreeRTOS 时基 |

configMAX_SYSCALL_INTERRUPT_PRIORITY = 4，即优先级 0-3 的中断不受 RTOS 临界区影响。

---

## 4. 核心模块设计

### 4.1 encoder (编码器)

- **TIM8 编码器模式**：TI1+TI2，4倍频
- **32位扩展**：`volatile int32_t overflow`，TIM8 更新中断维护（NVIC 优先级 0，不受 RTOS 临界区屏蔽）
- **接口**：
  - `get_position() → int32_t`
  - `set_position(int32_t pos)` — 上电恢复用
  - `get_z_position() → int32_t` — 最近一次 Z 脉冲位置
  - `reset()` — 清零

### 4.2 motor_ctrl (电机控制)

- **状态机**：IDLE → ACCELERATING → CONSTANT → DECELERATING → BRAKING → IDLE / STALLED
- **PWM**：TIM3_CH1(PA6) + TIM3_CH2(PA7)，15kHz，ARR=4799
- **控制接口**：
  - `move_to(int32_t target)` — 启动梯形运动
  - `stop()` — 正常停止(减速→刹车)
  - `emergency_stop()` — 紧急刹车(IN1=IN2=1)
  - `update()` — 1ms周期调用，更新加减速
  - `set_current_limit(uint16_t milliamps)` — 设置 DAC 输出 VREF 控制 A4950 最大电流 (I_max = VREF / 2)
  - `get_state() → MotorState` — 获取当前电机状态
  - `get_direction() → Direction` — 获取当前运动方向（堵转限位判定用）
- **梯形参数**：加速度、最大速度(PWM占空比)、减速距离阈值
- **DAC**：PA5(DAC_CH2) 输出 VREF，I_TripMAX = VREF / (10 × R_S) = VREF / 2

### 4.3 stall_detect (堵转检测)

- **ADC DMA**：PC0(电流) + PB0(电压) 连续采样
- **堵转判定**：
  - 启动屏蔽窗口 200ms
  - ADC>200 连续 1000 次(1秒) → 堵转确认
  - 编码器辅助：500ms 无脉冲变化
- **过流保护**：ADC>350 连续 10ms → 立即刹车 + DAC=0V + 0xE2 报警
- **方向判定**：正转堵转=正限位，反转堵转=负限位
- **与 motor_ctrl 交互**：stall_detect 在 MotorTask 的 `update()` 循环中被调用，堵转确认后直接调用 `motor_ctrl.emergency_stop()`，同一任务上下文，无跨任务通信
- **baseline_current 采集**：首次归零标定时，电机低速运行中采集正常电流 ADC 均值，存入 FRAM 的 baseline_current 字段，供未来动态阈值扩展使用

### 4.4 zoom_table (倍率映射表) — 可配置，存储在 STM32 内部 Flash

不同型号镜头的倍率映射表不同，因此映射表通过**工厂模式 0xF1 命令**写入，存储在 STM32 内部 Flash 的最后一个扇区（STM32F103RCT6 Flash 256KB，最后2KB页 @ 0x0803F800）。

- **角度驱动方式**：0xF1 参数格式为 [倍率×10(2B)][角度×100(2B)]。例如 3.0x 对应 207.5° → 倍率=0x001E, 角度=0x510E
- **角度→编码器计数换算**：`encoder_count = angle_x100 * total_range / 36000`（total_range 来自首次标定）
- **默认映射表**（出厂预置，可被工厂模式覆盖）：14 级 0.6x~7.0x
- **存储格式**：[条目数(1B)][{zoom_x10(2B), angle_x100(2B)} × N][CRC16(2B)]，最多支持 32 条倍率条目
- **查表接口**：
  - `get_position(uint16_t zoom_x10) → int32_t` — 倍率→编码器计数
  - `get_nearest_zoom(int32_t position) → uint16_t` — 位置→最近倍率
  - `is_valid_zoom(uint16_t zoom_x10) → bool` — 校验合法倍率
  - `get_next_zoom(uint16_t current, int8_t step) → uint16_t` — 循环变倍用
  - `set_entry(uint16_t zoom_x10, uint16_t angle_x100)` — 工厂模式写入单条
  - `erase_all()` — 工厂模式擦除所有条目（0xF0 命令）
  - `save_to_flash()` — 批量写入 Flash
  - `load_from_flash() → bool` — 上电从 Flash 加载

### 4.5 comm_protocol (通信协议) — 基于 电动镜头通信协议v2.5.docx

- **UART2 DMA**：115200, 8N1, 空闲中断触发帧解析
- **CRC16/MODBUS**：初始 0xFFFF，多项式 0xA001

**工作模式帧格式（6字节）**：[0xA5][CMD][PARAM_H][PARAM_L][CRC_L][CRC_H]

| 命令字 | 名称 | 参数含义 |
|--------|------|---------|
| 0x01 | 归零 | 保留 |
| 0x02 | 强制停止 | 保留 |
| 0x03 | 查询堵转计数 | 保留 |
| 0x10 | 设置倍率 | 倍率×10，仅限映射表中已标定的值 |
| 0x11 | 增加倍率 | 增量×10 |
| 0x12 | 减少倍率 | 减量×10 |
| 0x20 | 查询当前倍率 | 保留 |
| 0x21 | 查询运行状态 | 保留 |
| 0x22 | 查询速度 | 保留 |
| 0x23 | 查询镜头类型 | 保留，返回 0x0004（自带编码器直流电机镜头） |
| 0x24 | 查询倍率范围 | 保留，返回 min_zoom + max_zoom 两帧 |
| 0x25 | 查询固件版本 | 保留 |
| 0x30 | 循环变倍 | 高字节=步距(倍率×10)，低字节=停留时间(×0.1秒) |
| 0x31 | 停止循环 | 保留 |
| 0x40 | 设置运行速度 | PWM 占空比值 |
| 0x41 | 速度+50RPM | 保留 |
| 0x42 | 速度-50RPM | 保留 |
| 0x43 | 设置最小速度 | 最小 PWM |
| 0x44 | 设置最大速度 | 最大 PWM |
| 0x47 | 串口调试开关 | 非零=开启，零=关闭 |
| 0xFA | 切换工厂模式 | 0xFAE5 |

**工厂模式帧格式（8字节）**：[0xA5][CMD][PARAM(4B)][CRC_L][CRC_H]

| 命令字 | 名称 | 参数含义 |
|--------|------|---------|
| 0xF0 | 擦除所有倍率数据 | 0xFAE500FA |
| 0xF1 | 设置倍率-角度 | [倍率×10(2B)][角度×100(2B)]，角度驱动方式 |
| 0xFA | 切换工作模式 | 0xFAE500FA |

**下位机应答帧**：

| 命令字 | 参数 | 含义 |
|--------|------|------|
| 0x01 | 0x000F | 找到原点 |
| 0x02 | 0x000A | 移动到位 |
| 0x03 | 0x000E | 镜头变倍中，请求无效 |
| 0x10 | 倍率×10 | 当前倍率 |
| 0x11 | 状态 | 0=停止,1=运行,2=堵转,3=过流 |
| 0x12 | 速度 | 当前速度 |
| 0x13 | 0x0004 | 镜头类型（自带编码器直流电机） |
| 0x14 | min_zoom×10 | 最小倍率 |
| 0x15 | max_zoom×10 | 最大倍率 |
| 0x16 | 版本号 | 固件版本 |
| 0xE0 | 保留 | 命令参数错误 |
| 0xE1 | 保留 | 多次堵转，停止变倍 |
| 0xE2 | 保留 | 过流 |
| 0xE3 | 计数 | 堵转超过1s的计数 |

- **握手应答**：收到合法帧后原样回传
- **两种模式**：工作模式(6字节帧) 和 工厂模式(8字节帧)，通过 0xFA 命令切换

### 4.6 fram_storage (FRAM 存储)

- **SPI2**：PB12(CS), PB13(SCK), PB14(MISO), PB15(MOSI), PB11(WP)
- **存储结构**：60 字节参数（见需求文档 7.1 节）
- **地址映射**：主区 0x0000, 备份区 0x0040, 标定表 0x0100
- **双备份**：写入时同步更新两区，读取主区优先
- **紧急保存**：解除 WP → 仅写 position + valid → 恢复 WP (~50μs)
- **接口**：
  - `save_params(const FramParams&)` — 完整保存
  - `load_params(FramParams&) → bool` — 加载+CRC校验
  - `emergency_save(int32_t position)` — 紧急保存
  - `is_valid() → bool` — 检查 magic + CRC

### 4.7 power_monitor (电源监测)

- **PB0 ADC**：DMA 连续采样，12V 正常=ADC~3723
- **掉电检测**：ADC 模拟看门狗(AWD)硬件监视 IN8 通道，低阈值 3103（对应 10V）
- **紧急保存**：在 ADC_IRQHandler(AWD 中断)中直接执行：
  1. 刹车 (IN1=IN2=1)
  2. 读 TIM8 当前位置
  3. 设置全局 `spi_emergency` 标志，阻止 StorageTask 发起新的 SPI 事务
  4. 等待 SPI2 BUSY 标志清除（如当前有事务进行中，最多等待 ~10μs）
  5. FRAM: 解除 WP → 写 position + position_valid=0xFF + last_save_reason=2 → 恢复 WP
- **SPI 总线仲裁**：StorageTask 每次 SPI 操作前检查 `spi_emergency` 标志，已设置则放弃操作。紧急保存在 ISR 中通过轮询 SPI_SR.BSY 确保总线空闲后再操作，避免 DMA/中断嵌套冲突
- **MonitorTask 补充**：任务中定期(100ms)读 ADC 值用于状态上报，非紧急路径
- **与需求文档差异说明**：需求文档 6.3.3 节描述的是软件轮询方案（ADC每1ms采样+连续5ms确认），本设计优化为 ADC AWD 硬件中断方案，响应更快、不依赖任务调度，是对原始需求的增强实现

### 4.8 system_manager (系统状态管理)

- **状态机**：INIT → SELF_TEST → HOMING → READY → BUSY → ERROR
- **自检项**：编码器方向、编码器连接、Z通道连通性、ADC电流基线、电源电压
- **协议错误码**：0xE0(参数错误)、0xE1(多次堵转停止)、0xE2(过流)、0xE3(堵转计数)
- **内部错误码**：0xE4(编码器故障)、0xE5(Z通道故障)、0xE6(FRAM故障)
- **状态查询**：提供 `get_state()` 接口，CommTask 通过调用判断系统状态（READY/BUSY/ERROR），用于忙碌拒绝和状态上报(0x21→0x11应答：0=停止,1=运行,2=堵转,3=过流)
- **诊断信息输出**：在自检、归零、标定等关键流程中，STM32 通过串口发送可读状态字符串（如 "SELF_TEST: encoder OK\r\n"），供上位机实时显示当前诊断状态。通过 0x47 命令开启/关闭串口调试输出

---

## 5. 关键流程

### 5.1 首次上电全行程标定

1. INIT → 外设初始化
2. SELF_TEST → ADC/编码器/FRAM 自检
3. HOMING:
   - 反转低速(30%PWM) → 堵转 → 刹车 → position=0
   - 延时 200ms → 正转退回至少 4096 计数(2个电机轴转) → 确保远离物理限位
   - 继续正转低速 → 找 Z 脉冲(超时 214569 计数=一圈输出轴未找到→0xE5 Z通道故障) → 记录 z_offset
   - 标定 baseline_current：采集当前 ADC 电流均值（100次采样取平均），存入 FRAM
   - 继续正转 → 堵转 → 记录 range
   - soft_min=200, soft_max=range-200
   - 运动到 soft_min
4. FRAM 全参数保存 → READY

### 5.2 掉电恢复上电

1. INIT → 外设初始化
2. FRAM 读取 → CRC 通过 + homing_done + position_valid
3. 恢复编码器：TIM8->CNT = pos%65536, overflow = pos/65536
4. Z 脉冲校验：低速找最近 Z → |期望-实际| < 100 → READY
5. 偏差 > 100 → 完整归零

### 5.3 正常变倍

1. CommTask 收帧 → CRC 校验 → 握手应答
2. 系统 READY? 否 → 忙碌拒绝
3. 查映射表 → 参数有效? 否 → 参数无效
4. cmdQueue → MotorTask → 梯形运动
5. 到位(±50计数) → 刹车 → FRAM 保存 → 反馈帧

### 5.4 掉电恢复 Z 校验条件

- `last_save_reason == 掉电(2)` → **必须**执行 Z 脉冲校验
- `last_save_reason == 到位(0)` 或 `定期(1)` → 跳过 Z 校验，直接 READY
- 校验方式：低速运动找最近 Z 脉冲（最多 2048 计数范围），比较 |期望Z位置 - 实际Z位置|

### 5.5 循环变倍

1. 收到 0x30 → 解析参数：高字节 = 步距（在14级中跳跃的级数，1~13），低字节 = 停留时间（单位 100ms，1~255，即 0.1~25.5 秒）
2. 从当前倍率开始，按步距在 14 级倍率表中逐级跳跃移动
3. 每到达一个倍率 → 停留指定时间 → 继续移动到下一级
4. 到达最大倍率(7.0x)或最小倍率(0.6x)后，反转方向继续循环
5. 循环过程中：堵转保护和过流保护正常工作，掉电保护正常工作
6. 收到 0x31 → 当前运动完成到位后停止（不中途急停）
7. 循环中收到其他变倍命令(0x10/0x11/0x12) → 忙碌拒绝(CMD=0x03, PARAM=0x000E)

---

## 6. RAM 使用预算（STM32F103RCT6: 48KB SRAM）

| 项目 | 大小 | 说明 |
|------|------|------|
| MotorTask 栈 | 2048 B | 512 words |
| CommTask 栈 | 1024 B | 256 words |
| StorageTask 栈 | 1024 B | 256 words |
| MonitorTask 栈 | 1024 B | 256 words |
| FreeRTOS 内核 | ~2000 B | TCB + 队列 + 内核对象 |
| cmdQueue (8×8B) | 64 B | 命令队列 |
| rspQueue (8×8B) | 64 B | 响应队列 |
| saveQueue (4×64B) | 256 B | 保存队列 |
| ADC DMA buffer | 64 B | 双缓冲 2ch × 16 samples |
| UART DMA buffer | 64 B | 双缓冲 2 × 32B |
| 应用全局变量 | ~1000 B | 状态、参数、映射表等 |
| **合计** | **~8.6 KB** | 占 48KB 的 ~18% |
| 可用堆/余量 | ~39 KB | 充裕 |

---

## 7. 风险与约束

| 风险 | 影响 | 应对 |
|------|------|------|
| LM321A 增益=1，ADC 有效量程仅 9%（~300mV/3300mV） | 堵转检测信噪比低，噪声可能导致误判 | 软件滤波(滑动均值)；堵转阈值 ADC>200 远大于正常值 ~50，有足够裕量；连续1秒确认消除毛刺 |
| 50 脉冲软限位偏移是否合适 | 偏移过大浪费行程，过小可能碰到物理限位 | 设为可配置参数(FRAM存储)，默认200计数，上板验证后调整 |
| 掉电后电容放电曲线未测量 | 紧急保存时间可能不足 | FRAM 写入仅 ~50μs，极端情况也够；ADC 看门狗在硬件中断级触发，延迟极小 |
| AS5311 磁编码器预留引脚 | PB6/PB7/PB8 未连接可能浮空 | CubeMX 中配置为 GPIO Input Pull-Down |

---

## 8. CubeMX 外设配置清单

| 外设 | 配置 | 用途 |
|------|------|------|
| TIM3 | PWM Mode 1, 15kHz, CH1(PA6)+CH2(PA7) | 电机驱动 |
| TIM8 | Encoder Mode TI1+TI2, CH1(PC6)+CH2(PC7) | 编码器 |
| DAC | Channel 2, PA5 | A4950 VREF |
| ADC1 | DMA Continuous, IN10(PC0)+IN8(PB0) | 电流+电压 |
| USART2 | 115200/8N1, DMA RX, IDLE中断 | RS232 通信 |
| SPI2 | Master, 10MHz, PB12-15 | FRAM |
| ADC1 AWD | 看门狗监视 IN8, 低阈值 3103 | 掉电检测 |
| Flash | 最后2KB页 @ 0x0803F800 | 倍率映射表存储 |
| GPIO | PB11(WP输出), PA9(Z脉冲EXTI), PB6/7/8(下拉输入) | 控制+中断+预留 |
| FreeRTOS | 4 任务, 3 队列, CMSIS V2 | 操作系统 |

---

## 9. 测试策略（TDD，覆盖率 >80%）

### 9.1 测试架构

采用 **TDD（测试驱动开发）**：每个模块先编写测试用例，再编写实现代码。

- **主机端单元测试**：使用 Google Test (gtest)，在 x86 主机上运行，通过 HAL Mock 层隔离硬件依赖
- **HAL Mock 层**：mock STM32 HAL 函数（GPIO、TIM、ADC、SPI、UART、DAC）和 FreeRTOS API（xQueue、xTask、xSemaphore）
- **构建方式**：CMake 双目标 — `ninja`(ARM交叉编译) + `ninja test`(主机gtest)

```bash
# 主机端测试构建与运行
cd ZLENS_DC && mkdir build-test && cd build-test
cmake -G Ninja -DBUILD_TESTING=ON ..
ninja && ctest --output-on-failure
```

### 9.2 各模块测试重点

| 模块 | 测试内容 | 覆盖目标 |
|------|---------|---------|
| encoder | 位置计算、溢出处理、Z脉冲记录、set/get一致性 | >90% |
| motor_ctrl | 梯形加减速状态机转换、PWM输出值、到位判断、紧急停止 | >85% |
| stall_detect | 阈值判定、屏蔽窗口、连续计数、过流保护触发、方向判定 | >85% |
| zoom_table | 倍率查找、有效性校验、最近倍率计算、角度-计数换算、Flash读写 | >90% |
| comm_protocol | CRC16计算、帧解析、所有命令字处理、工厂模式切换、错误帧拒绝 | >90% |
| fram_storage | 参数序列化/反序列化、CRC校验、双备份恢复、紧急保存 | >85% |
| power_monitor | 掉电阈值判定、ADC值换算 | >80% |
| system_manager | 状态机转换、自检流程、错误状态进入/退出 | >85% |

### 9.3 编译验证（ARM交叉编译）

```bash
cd ZLENS_DC && mkdir build && cd build
cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=../cmake/stm32f103rc.cmake ..
ninja
```
确认生成 .elf/.bin/.hex 文件，无编译错误。

### 9.4 硬件验证（上板后）

- 编码器：手动转电机，观察计数变化和方向
- PWM：示波器检查 PA6/PA7 波形
- UART：PC 发送 6 字节帧，验证握手应答
- FRAM：写入/读取参数，掉电后重读验证
- ADC：静止时电流基线、手动堵转时电流值

### 9.5 系统级验证

- 完整归零流程
- 所有倍率逐一到达验证
- 掉电恢复 + Z 脉冲校验
- 循环变倍 + 停止
- 堵转保护触发
- 工厂模式倍率表写入/擦除
