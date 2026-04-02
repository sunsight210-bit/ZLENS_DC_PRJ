# 引脚与外设配置

MCU: STM32F103RCT6 | 系统时钟: 64MHz (HSI/2 x PLL_MUL16) | APB1: 32MHz | APB2: 64MHz

## 引脚分配总表

| 引脚 | 功能 | 外设 | 方向 | 备注 |
|------|------|------|------|------|
| PA2 | UART_TX | USART2 | 输出 | 115200 8N1 |
| PA3 | UART_RX | USART2 | 输入 | DMA1_CH6 接收 |
| PA4 | DAC_OUT2 | DAC_CH2 | 输出 | A4950 VREF 限流, 默认 2.5V |
| PA6 | MOTOR_PWM_FWD | TIM3_CH1 | 输出 | 正转 PWM |
| PA7 | MOTOR_PWM_REV | TIM3_CH2 | 输出 | 反转 PWM |
| PA9 | ENCODER_Z | EXTI9 | 输入 | AS5311 INDEX 脉冲, 上升沿触发 |
| PB0 | ADC_VOLTAGE | ADC1_CH8 | 输入 | 供电电压采样 (30k/4.7k 分压) |
| PB3 | SWO | SWO | 输出 | ITM 调试输出 |
| PB6 | ENCODER_A | TIM4_CH1 | 输入 | AS5311 正交 A 信号 (CubeMX 标签: LENS_ID0) |
| PB7 | ENCODER_B | TIM4_CH2 | 输入 | AS5311 正交 B 信号 (CubeMX 标签: LENS_ID1) |
| PB8 | LENS_ID2 | GPIO | 输入 | 镜头 ID bit2 (CubeMX 标签, 预留) |
| PB11 | FRAM_WP | GPIO_PP | 输出 | FRAM 写保护, 低电平保护 |
| PB12 | FRAM_CS | GPIO_PP | 输出 | FRAM 片选, 低电平有效 |
| PB13 | SPI2_SCK | SPI2 | 输出 | FRAM 时钟 |
| PB14 | SPI2_MISO | SPI2 | 输入 | FRAM 数据输入 |
| PB15 | SPI2_MOSI | SPI2 | 输出 | FRAM 数据输出 |
| PC0 | ADC_CURRENT | ADC1_CH10 | 输入 | 电机电流采样 |

> **注意**: PB6/PB7 在 CubeMX (.ioc) 中标注为 LENS_ID0/LENS_ID1，这是历史遗留标签。Phase 8 将编码器从 TIM8(PC6/PC7) 迁移到 TIM4(PB6/PB7) 后，这两个引脚的实际功能是 AS5311 编码器 A/B 信号。

## 外设配置详情

### TIM3 — 电机 PWM

| 参数 | 值 | 说明 |
|------|-----|------|
| 时钟源 | APB1 x2 = 64MHz | APB1 分频后定时器自动倍频 |
| Prescaler | 0 | 不分频 |
| Period (ARR) | 4266 | PWM 频率 = 64MHz / 4267 ≈ 15kHz |
| 模式 | PWM1 (向上计数) | CCR < CNT 时输出高 |
| CH1 (PA6) | 正转 | CCR1 = duty, CCR2 = 0 |
| CH2 (PA7) | 反转 | CCR1 = 0, CCR2 = duty |
| 制动 | CCR1 = CCR2 = ARR | 两路同时拉高, 动态制动 |

PWM 占空比转换: `duty_pwm = duty_x10 * 4266 / 1000`

速度范围:

| 名称 | PWM 值 | 占空比 |
|------|--------|--------|
| MIN_SPEED | 128 | 3.0% |
| MIN_SPEED_TIER1 | 192 | 4.5% (卡住 500ms 后) |
| MIN_SPEED_TIER2 | 280 | 6.6% (卡住 1000ms 后) |
| MAX_SPEED | 1280 | 30.0% |

### TIM4 — AS5311 编码器

| 参数 | 值 | 说明 |
|------|-----|------|
| 模式 | TIM_ENCODERMODE_TI12 | 4 倍频正交解码 |
| Period | 65535 | 16-bit 满量程 |
| IC1 (PB6) | FALLING 极性 | A 信号, 反相补偿电机方向 |
| IC2 (PB7) | RISING 极性 | B 信号, 正常极性 |
| 滤波 | 0x0F | 15 个时钟周期滤波 |
| 溢出中断 | 开启 | TIM4_IRQn, 用于 32-bit 位置扩展 |

位置计算: `position = overflow * 65536 + TIM4->CNT`

AS5311 规格: 2.0mm 磁极对间距, 1024 steps/pole, 分辨率 ~1.95µm/step

### ADC1 — 电流/电压采样

| 参数 | 值 |
|------|-----|
| 模式 | 连续扫描, 2 通道 |
| 触发 | 软件触发 |
| 对齐 | 右对齐 |
| 分辨率 | 12-bit (0-4095) |
| 采样时间 | 239.5 cycles (两通道相同) |
| DMA | DMA1_CH1, 连续传输 → `g_aAdcDmaBuf[2]` |

通道配置:

| 通道 | 引脚 | Rank | 用途 | 换算 |
|------|------|------|------|------|
| CH10 | PC0 | 1 | 电机电流 | 经 AdcFilter IIR 滤波 (alpha=1/16) |
| CH8 | PB0 | 2 | 供电电压 | `voltage_mV = adc * 24363 / 4095` |

电压分压器: R_top = 30kOhm, R_bottom = 4.7kOhm, 满量程 ~24.4V

关键阈值:

| 名称 | ADC 值 | 物理值 | 用途 |
|------|--------|--------|------|
| OVERCURRENT_THRESHOLD | 3000 | ~882mA | 过流保护, 50ms 确认 |
| STALL_THRESHOLD | 1200 | ~967mV | 堵转检测, 1000ms 确认 |
| POWER_DOWN_THRESHOLD | 1394 | ~8.3V | 欠压保护 |

### DAC — 电机限流 (VREF)

| 参数 | 值 |
|------|-----|
| 通道 | DAC_CHANNEL_2 (PA4) |
| 触发 | 无触发 (直接输出) |
| 输出缓冲 | 开启 |
| 默认值 | 2.5V (对应 A4950 限流点) |

换算: `dac_value = target_mV * 4095 / 3300`

### SPI2 — FRAM 存储

| 参数 | 值 |
|------|-----|
| 模式 | Master, 全双工 |
| 数据宽度 | 8-bit |
| 时钟极性 | CPOL=0, CPHA=0 (Mode 0) |
| 波特率 | APB1/4 = 8MHz |
| 位序 | MSB First |
| NSS | 软件控制 (PB12) |

FRAM 命令:

| 命令 | 字节 | 说明 |
|------|------|------|
| WREN | 0x06 | 写使能 |
| WRITE | 0x02 | 写数据 |
| READ | 0x03 | 读数据 |

存储地址: 主区 0x0000, 备份区 0x0020, 每区 32 字节（详见 storage-map.md）

### USART2 — 上位机通信

| 参数 | 值 |
|------|-----|
| 波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 |
| 流控 | 无 |
| RX DMA | DMA1_CH6, IDLE 线检测 |

### IWDG — 看门狗

| 参数 | 值 |
|------|-----|
| 时钟 | LSI ~32kHz |
| Prescaler | 64 |
| Reload | 2499 |
| 超时 | (2500 x 64) / 32000 ≈ 5s |
| 喂狗点 | MonitorTask, 每 100ms |

## DMA 通道分配

| 通道 | 外设 | 方向 | 优先级 | 说明 |
|------|------|------|--------|------|
| DMA1_CH1 | ADC1 | 外设→内存 | 5 | 连续采样 2 通道 → g_aAdcDmaBuf[2] |
| DMA1_CH6 | USART2_RX | 外设→内存 | 5 | UART 接收, IDLE 线中断触发 |

## 中断优先级

| 中断源 | 优先级 | 用途 |
|--------|--------|------|
| DMA1_CH1 (ADC) | 5 | ADC DMA 传输完成 |
| DMA1_CH6 (UART RX) | 5 | UART 帧接收 |
| TIM4 (编码器溢出) | 5 | 16-bit 计数器溢出, 扩展为 32-bit |
| EXTI9_5 (INDEX) | 5 | AS5311 Z 脉冲, 漂移检测 |

> FreeRTOS 下所有外设中断优先级 ≥ 5 (`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`)，确保可安全调用 FreeRTOS API。
