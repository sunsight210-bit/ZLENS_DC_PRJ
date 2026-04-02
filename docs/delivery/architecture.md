# ZLENS_DC 系统架构

ZLENS_DC 是基于 STM32F103RCT6 的电动变倍镜头控制系统，运行 FreeRTOS，采用 4 任务架构，PID 闭环定位。

已完成 Phase 1-9，所有功能模块就绪。

## 系统总览

```
┌─────────────────────────────────────────────────────┐
│                   外部接口                           │
│  镜头主机 ←UART2→  MCU  ←SWD/SWO→ ST-Link V3       │
└─────────────────────────────────────────────────────┘
                        │
┌─────────────────────────────────────────────────────┐
│                FreeRTOS Tasks                        │
│                                                      │
│  MotorTask (P4, 1ms)    CommTask (P3, 事件驱动)      │
│  PID闭环+回零+堵转      UART DMA收发+协议v2.7       │
│         │                      │                     │
│         ├── g_cmdQueue ←───────┘                     │
│         ├── g_rspQueue ────────→                     │
│         └── g_saveQueue ──→ StorageTask (P2, 10ms)   │
│                              FRAM 状态持久化          │
│                                                      │
│  MonitorTask (P1, 100ms)                             │
│  启动判断+看门狗+电压监控                             │
└─────────────────────────────────────────────────────┘
                        │
┌─────────────────────────────────────────────────────┐
│                  App 核心模块                         │
│  MotorCtrl   PidCtrl    Encoder      StallDetect     │
│  ZoomTable   FramStorage FlashConfig CommProtocol    │
│  SystemManager PowerMonitor AdcFilter CRC16 SwoDebug │
└─────────────────────────────────────────────────────┘
                        │
┌─────────────────────────────────────────────────────┐
│                  硬件外设                             │
│  TIM3(PWM) TIM4(编码器) ADC1+DMA DAC(VREF)          │
│  SPI2(FRAM) USART2(DMA RX) IWDG(看门狗)             │
└─────────────────────────────────────────────────────┘
```

## 任务间通信

| 队列 | 方向 | 容量 | 消息体 | 用途 |
|------|------|------|--------|------|
| `g_cmdQueue` | CommTask → MotorTask | 8 | `CMD_MESSAGE_S{cmd, param}` | 运动/回零/速度命令 |
| `g_rspQueue` | MotorTask → CommTask | 8 | `RSP_MESSAGE_S{cmd, param}` | 到达/堵转/过流响应 |
| `g_saveQueue` | MotorTask → StorageTask | 4 | `SAVE_MESSAGE_S{pos, zoom, reason, ...}` | 位置保存 |
| TaskNotify | MonitorTask → MotorTask | — | bit 0x02 | 掉电信号 |

## MotorTask — 电机控制 (Priority 4, 1ms)

职责：PID 位置闭环、回零序列、堵转/过流保护、循环运动、速度管理

### 状态机

```
IDLE ──SET_ZOOM/INC/DEC──→ MOVING ──到位──→ IDLE
  │                            └──堵转/过流──→ IDLE
  ├──HOMING──→ HOMING_FAST ──堵转──→ HOMING_RETRACT ──到位──→ HOMING_SLOW
  │                                                              └──堵转──→ HOMING_SETTLE ──到位──→ IDLE
  └──CYCLE_START──→ CYCLING ──CYCLE_STOP──→ IDLE
```

### 每周期处理 (run_once, 1ms)

1. 检查掉电通知（最高优先级）
2. 读编码器位置 + ADC 电流
3. StallDetect::update() 堵转/过流判断
4. MotorCtrl::update() PID 运算 + PWM 输出
5. 过流检测 → emergency_stop
6. 堵转检测 → handle_stall
7. 接收 g_cmdQueue 命令 → dispatch_command
8. 状态处理：process_moving / process_homing / process_cycling

### PID 闭环参数

| 参数 | 值 | 说明 |
|------|-----|------|
| Kp | 0.50 | 比例增益 |
| Ki | 0.003 | 积分增益 |
| Kd | 0.00 | 微分增益 |
| INTEGRAL_MAX | 192.0 | 积分饱和限幅 |
| INTEGRAL_SEPARATION | 32 | 积分分离阈值 |
| DEADZONE | 2 counts | 到位死区 |

### 回零四阶段

| 阶段 | 动作 | 结束条件 |
|------|------|----------|
| HOMING_FAST | 反向半速直驱（非 PID） | 编码器堵转或电流堵转 |
| HOMING_RETRACT | PID 正向回退 256 counts | 到位 |
| HOMING_SLOW | 反向慢速直驱 | 编码器堵转 |
| HOMING_SETTLE | PID 正向移动 128 counts (HOME_OFFSET) | 到位，位置归零 |

## CommTask — 通信任务 (Priority 3, 事件驱动)

职责：UART DMA 接收、协议帧解析（工作/工厂模式）、CRC 校验、命令分发、响应发送

### 命令路由

| 命令类型 | 处理方式 |
|----------|----------|
| 查询命令 (0x20-0x25) | CommTask 直接响应 |
| GET_STALL_CNT (0x03) | CommTask 直接响应 |
| 速度命令 (0x60-0x65) | 转发 MotorTask（不检查忙状态） |
| 运动命令 (0x01/0x10-0x12/0x30) | 系统忙时拒绝(REQ_INVALID)，否则转发 MotorTask |
| SWITCH_FACTORY (0xFA) | 切换工作/工厂模式 |

### UART 接收链路

```
UART2 RX DMA → 空闲线中断 → s_bRxReady 标志 → CommTask 轮询 → parse + dispatch
```

UART 看门狗：检测 DMA 卡死，2 级恢复（abort+retry → DeInit+Init）。

## StorageTask — 存储任务 (Priority 2, 10ms)

职责：FRAM 运行时状态持久化、启动恢复

### 触发方式

- **事件驱动**：收到 g_saveQueue 消息 → 立即写入
- **周期保存**：每 500ms 检查位置变化 → 有变化则写入

### 存储内容 (FRAM_STATE_S, 32B)

详见 `storage-map.md`。核心字段：current_position, current_zoom_x10, homing_done, position_valid, move_count。

双副本写入：主区 0x0000 + 备份区 0x0020。

## MonitorTask — 监控任务 (Priority 1, 100ms)

职责：启动路径判断、看门狗喂狗、电压监控

### 上电启动判断

```
读取 FRAM → magic/CRC 校验
  ├── 有效 + homing_done + position_valid + move_count<500 → Normal Boot（跳过回零）
  ├── 有效 + homing_done → Homing-Only Boot
  └── 无效 → First Boot（强制回零）
```

Normal Boot 时恢复编码器位置，检查是否需要移动到最近的倍率位置。

### 掉电保护

电压低于 8.3V (ADC<1394) → xTaskNotify 通知 MotorTask → emergency_stop + FRAM 紧急保存位置。

## 模块职责一览

| 模块 | 文件 | 职责 |
|------|------|------|
| MotorCtrl | `App/*/motor_ctrl.*` | PID 位置闭环、PWM 输出、方向切换、DAC 电流限值、软限位 |
| PidCtrl | `App/Inc/pid_ctrl.hpp` | PID 控制器（header-only），积分分离、输出限幅 |
| Encoder | `App/*/encoder.*` | TIM4 正交解码(PB6/PB7)、32-bit 位置扩展、AS5311 INDEX 漂移检测 |
| StallDetect | `App/*/stall_detect.*` | 电流堵转(1000ms)、过流保护(50ms)、编码器堵转(500ms) |
| ZoomTable | `App/*/zoom_table.*` | 倍率↔编码器位置映射、默认 14 点表、Flash 持久化(未挂钩) |
| FramStorage | `App/*/fram_storage.*` | SPI FRAM 读写、双副本、CRC 校验、紧急保存 |
| FlashConfig | `App/*/flash_config.*` | 内部 Flash 配置区读写（速度/行程/镜头参数） |
| CommProtocol | `App/*/comm_protocol.*` | 帧解析 v2.7、工作/工厂模式 |
| SystemManager | `App/*/system_manager.*` | 状态机：INIT→SELF_TEST→HOMING→READY→BUSY→ERROR_STATE |
| PowerMonitor | `App/*/power_monitor.*` | ADC 电压监控、欠压检测 |
| AdcFilter | `App/*/adc_filter.*` | IIR 低通滤波 (alpha=1/16) |
| CRC16 | `App/*/crc16.*` | CRC16-Modbus 校验 |
| SwoDebug | `App/*/swo_debug.*` | SWO ITM 调试打印 (PB3) |

## 数据流 — SET_ZOOM 命令示例

```
主机 → [A5 10 00 14 CRC_L CRC_H] → UART2 DMA
  → CommTask: CRC校验 + 握手回显 + xQueueSend(g_cmdQueue, {0x10, 20})
    → MotorTask: ZoomTable::get_position(20) → MotorCtrl::move_to(target)
      → 每1ms: PidCtrl计算 → PWM输出 → Encoder读位置
      → 到位: stop() + xQueueSend(g_rspQueue, {0x02, 20})
              + xQueueSend(g_saveQueue, {pos, 20, ARRIVED})
    → CommTask: [A5 02 00 14 CRC_L CRC_H] → 主机
    → StorageTask: FRAM 写入位置
```
