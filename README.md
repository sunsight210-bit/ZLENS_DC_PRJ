# ZLENS_DC

电动变倍镜头直流电机控制器。

## 项目简介

ZLENS_DC 是基于 STM32F103RCT6 的电动变倍镜头控制系统，通过 UART 接收命令驱动直流减速电机，配合 AS5311 磁编码器实现精确变倍定位。

- **MCU**: STM32F103RCT6, 64MHz (HSI/2 x PLL16)
- **RTOS**: FreeRTOS
- **编码器**: AS5311 线性磁编码器 (TIM4 正交解码, PB6/PB7)
- **电机驱动**: A4950 H-bridge (TIM3 PWM 15kHz, DAC Vref 限流)
- **通信**: UART 115200 (USART2), CRC16-Modbus 校验
- **存储**: SPI FRAM (双备份)

## 目录结构

```
ZLENS_DC/
├── App/Inc/, App/Src/        # 应用模块 (11个)
├── Tasks/Inc/, Tasks/Src/    # FreeRTOS 任务 (4个)
├── Core/                     # CubeMX 生成: main.c, 中断, HAL 配置
├── Drivers/                  # STM32 HAL + CMSIS
├── Middlewares/              # FreeRTOS
├── Test/                     # Google Test 单元测试 + Mock
├── docs/                     # 项目文档
│   ├── delivery/             # 交付包 (架构/协议/指南/交接)
│   ├── dev/                  # 开发规范 (编码/构建/进度/模板)
│   ├── phases/               # 阶段文档 (plan/verify/review)
│   ├── ref/                  # 硬件参考资料
│   └── req/                  # 需求报告
├── scripts/                  # 验证/调试脚本
├── cmake/                    # CMake 工具链
└── build/                    # 构建产物 (fw/test/swo)
```

## 快速上手

### 环境要求

- ARM GCC toolchain (`arm-none-eabi-gcc`)
- CMake >= 3.16
- ST-Link V3 (or compatible)
- Python 3 (验证脚本)

### 编译固件

```bash
cmake -B build/fw -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake
cmake --build build/fw
```

### 运行单元测试

```bash
cmake -B build/test -DBUILD_TESTING=ON
cmake --build build/test
ctest --test-dir build/test
```

### 烧录

参见 `docs/delivery/flash-guide.md`。

## 系统架构

4 个 FreeRTOS 任务:

| 任务 | 优先级 | 周期 | 职责 |
|------|--------|------|------|
| MotorTask | 4 (最高) | 1ms | PID 闭环控制、归零、堵转检测 |
| CommTask | 3 | 事件驱动 | UART 帧解析、命令分发、响应 |
| StorageTask | 2 | 500ms/事件 | FRAM 参数持久化 |
| MonitorTask | 1 (最低) | 20ms | 电压监控、看门狗、系统状态 |

## 文档索引

| 文档 | 说明 |
|------|------|
| `docs/delivery/architecture.md` | 系统架构设计 |
| `docs/delivery/pin-peripheral-map.md` | 引脚与外设映射 |
| `docs/delivery/handover.md` | 交接文档（工作流/参数/陷阱/优化） |
| `docs/delivery/protocol-spec.md` | 通信协议规格书 |
| `docs/delivery/storage-map.md` | 参数存储映射表 |
| `docs/delivery/user-guide.md` | 产品使用指南 |
| `docs/delivery/flash-guide.md` | 生产烧录指南 |
| `docs/dev/coding-style.md` | 编码规范 |
| `docs/dev/build-flash-swo-workflow.md` | 编译烧录调试流程 |
| `docs/dev/progress.md` | 开发进度 |
| `docs/dev/TEMPLATE.md` | 阶段文档模板 |

## 许可证

Internal use only.
