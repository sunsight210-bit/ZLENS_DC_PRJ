# ZLENS_DC 开发交接文档

## 1. 项目简介

电动变倍镜头控制系统。STM32F103RC + FreeRTOS + PID 闭环 + AS5311 磁编码器。
通过 UART 接收上位机命令，驱动 DC 有刷电机实现 0.6×~7.0× 变倍切换。

## 2. 开发环境搭建

### 工具链

| 用途 | 工具 | 路径 |
|------|------|------|
| 交叉编译 | arm-none-eabi-gcc 10.3.1 | STM32CubeIDE 内置 |
| 单元测试 | g++ (host) + GoogleTest | `/usr/bin/g++` |
| 构建系统 | CMake 3.22 | `/usr/bin/cmake` |
| 烧录 | STM32_Programmer_CLI | CubeIDE 插件目录 |
| 调试 | OpenOCD | CubeIDE 插件目录 |

完整路径见 `docs/dev/build-flash-swo-workflow.md`。

### 编译命令

```bash
# 固件
cmake -B build/fw -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake
cmake --build build/fw

# 单元测试
cmake -B build/test -DBUILD_TESTING=ON
cmake --build build/test
ctest --test-dir build/test --output-on-failure
```

### 烧录 + 复位

详见 `docs/delivery/flash-guide.md`。

## 3. 代码结构导航

```
App/Inc/          App 模块头文件（motor_ctrl, encoder, pid_ctrl, zoom_table 等）
App/Src/          App 模块实现
Tasks/Inc/        FreeRTOS 任务头文件 + task_config.hpp（命令/响应常量）
Tasks/Src/        任务实现（motor_task, comm_task, storage_task, monitor_task）
Core/Inc/         CubeMX 生成的 HAL 配置
Core/Src/         main.c + HAL 初始化
Test/             单元测试
Test/mocks/       HAL/FreeRTOS mock
Test/diag/        诊断固件（独立构建）
cmake/            工具链文件
scripts/          验证脚本 + SWO 解析器
docs/delivery/    交付文档（架构、引脚、存储、协议等）
docs/dev/         开发流程文档（编译烧录、编码规范）
docs/phases/      各阶段开发记录
docs/ref/         数据手册、协议原文
docs/req/         需求文档
```

## 4. 倍率切换工作流

```
上位机发送 SET_ZOOM(zoom_x10)
  → CommTask 解析 + CRC 校验
  → g_cmdQueue 转发 MotorTask
  → ZoomTable::get_position(zoom_x10) 查表得到编码器目标位置
  → MotorCtrl::move_to(target) 启动 PID 闭环
  → 每 1ms: PidCtrl 计算误差 → PWM 输出 → Encoder 读位置
  → |position - target| <= DEADZONE(2) 持续 100ms → 到位
  → 发送 ARRIVED 响应 + FRAM 保存位置
```

## 5. 可调参数表

### PID 参数

| 参数 | 位置 | 当前值 | 调整方法 |
|------|------|--------|----------|
| Kp | `App/Inc/pid_ctrl.hpp` | 0.50 | 改常量重编译 |
| Ki | 同上 | 0.003 | 同上 |
| Kd | 同上 | 0.00 | 同上 |
| INTEGRAL_MAX | 同上 | 192.0 | 积分饱和限幅 |
| INTEGRAL_SEPARATION | 同上 | 32 | 误差小于此值才积分 |

### 速度参数

| 参数 | 存储位置 | 当前默认值 | 调整方法 |
|------|----------|------------|----------|
| speed_duty | Flash 配置区 | 300 (30%) | SET_SPEED 命令(0x60) |
| min_speed_duty | Flash 配置区 | 40 (4%) | SET_MIN_SPEED 命令(0x63) |
| max_speed_duty | Flash 配置区 | 300 (30%) | SET_MAX_SPEED 命令(0x64) |

### 电机保护参数

| 参数 | 位置 | 当前值 | 说明 |
|------|------|--------|------|
| STALL_THRESHOLD | `App/Inc/stall_detect.hpp` | 1200 ADC | 堵转电流阈值，1000ms 确认 |
| OVERCURRENT_THRESHOLD | 同上 | 3000 ADC | 过流阈值，50ms 快速保护 |
| ENCODER_STALL_TICKS | 同上 | 500 | 编码器无变化判定堵转(ms) |
| BLANKING_TICKS | 同上 | 200 | 启动消隐期(ms) |

### 回零参数

| 参数 | 位置 | 当前值 | 说明 |
|------|------|--------|------|
| HOME_OFFSET | `App/Inc/zoom_table.hpp` | 128 counts | 归零后逻辑原点偏移 |
| RETRACT_DISTANCE | `Tasks/Inc/task_config.hpp` | 256 counts | 粗定位后回退距离 |
| REHOME_MOVE_COUNT | `Tasks/Inc/monitor_task.hpp` | 500 | 累计移动次数超限强制回零 |

### DAC 电流限值

| 参数 | 位置 | 当前值 | 说明 |
|------|------|--------|------|
| VREF | `App/Src/app_instances.cpp` | 2500 mV | A4950 电流限值基准 |

## 6. 已知陷阱

### 硬件

| 陷阱 | 症状 | 解决 |
|------|------|------|
| ST-Link TVCC 接线 | 电路板发烫、MCU 无法 POR | 断开 TVCC 引脚，只保留 SWDIO/SWCLK/GND/SWO |
| 编码器 A/B 接反 | 电机运动方向与编码器反馈相反 | 软件补偿：TIM4 IC1 极性设为 FALLING |
| SWD 寄生电流 | 烧录后 UART 无响应 | 烧录后必须 OpenOCD reset run |
| Voltage 显示 0.00V | 误以为供电异常 | 正常现象（TVCC 已断开） |

### 软件

| 问题 | 说明 | 位置 |
|------|------|------|
| ZoomTable Flash 未挂钩 | `save_to_flash()`/`load_from_flash()` 写了但未接入产品代码，每次上电用硬编码默认表 | `App/Src/zoom_table.cpp` |
| Phase 7 未完成 | 齿轮间隙补偿阶段标记"进行中"，Phase 8 切换 AS5311 后间隙由 PID 闭环自动消除 | `docs/dev/progress.md` |

## 7. 技术债与 Pre-existing 测试失败

| 测试 | 失败数 | 原因 | 影响 |
|------|--------|------|------|
| test_pid_ctrl | 7/8 | PID 参数调优后测试期望值未更新 | 不影响固件功能 |
| test_stall_detect | 2/9 | 堵转检测参数调整后测试未同步 | 不影响固件功能 |
| test_motor_ctrl | 4/26 | PID 模式下行为变化，测试基于旧梯形加减速 | 不影响固件功能 |
| test_motor_task | 3/N | speed 保存改为 Flash 后部分测试未完全适配 | 不影响固件功能 |

## 8. 后续优化方向

- **S 曲线加减速**：替代当前的线性 PID，减少到位振荡（方案讨论见 `docs/req/motor_ctrl_strategy_discussion.md`）
- **摩擦前馈补偿**：低速段 PID 输出不足以克服静摩擦，可加前馈项
- **ZoomTable Flash 挂钩**：接入 `load_from_flash()` + 串口标定写入
- **Flash 配置区命令接口**：当前只有 speed 通过串口可调，其他配置需要编码写入
- **测试修复**：同步 PID/StallDetect 相关测试的期望值

## 9. 文档索引

### 交付文档 (docs/delivery/)

| 文档 | 内容 |
|------|------|
| `architecture.md` | 系统架构、任务划分、模块职责、数据流 |
| `pin-peripheral-map.md` | 引脚分配、外设配置、ADC/PWM/SPI 参数 |
| `storage-map.md` | FRAM/Flash 存储映射、参数表、地址分配 |
| `protocol-spec.md` | 通信协议 v2.7 规格书 |
| `user-guide.md` | 产品使用指南 |
| `flash-guide.md` | 生产烧录指南 |
| `handover.md` | 本文档 |

### 开发文档 (docs/dev/)

| 文档 | 内容 |
|------|------|
| `build-flash-swo-workflow.md` | 编译、烧录、SWO 调试操作手册 |
| `coding-style.md` | 命名规范（匈牙利前缀、类型后缀） |
| `progress.md` | 开发进度看板 |

### 参考资料 (docs/ref/)

| 文档 | 内容 |
|------|------|
| `stm32f103rc.pdf` | MCU 数据手册 |
| `AS5311_DS000200_2-00.pdf` | 磁编码器数据手册 |
| `A4950.pdf` | 电机驱动芯片 |
| `电动镜头通信协议v2.7.docx` | 协议原文 |
