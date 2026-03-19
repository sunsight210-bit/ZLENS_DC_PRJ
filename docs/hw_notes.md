# 硬件调试注意事项

## ST-Link 接线

### 必须断开 TVCC 引脚

ST-Link V3 的 TVCC 引脚会与板载 3.3V LDO 形成反向供电，导致：

1. **电路板发烫** — TVCC 与板载电源冲突，产生异常功耗
2. **MCU 无法正常 POR** — 镜头掉电后，SWD 引脚（SWDIO/SWCLK）通过 ESD 保护二极管提供寄生电流，MCU 不会真正断电，上电后外设状态残留，UART 不工作
3. **Voltage 显示 0.00V 误导判断** — 断开 TVCC 后 ST-Link 检测不到目标电压，但 MCU 实际由板载 LDO 正常供电

**正确接线**：只保留 4 根线

| 引脚 | 功能 | 必须 |
|------|------|------|
| SWDIO | 调试数据 | Yes |
| SWCLK | 调试时钟 | Yes |
| GND | 地线 | Yes |
| SWO (PB3) | 调试输出 | Yes |
| TVCC | **断开** | No |

断开 TVCC 后 SWD 调试、烧录、SWO 输出均正常工作。

### 烧录后必须强制复位

由于 SWD 寄生电流问题，`STM32_Programmer_CLI -rst` 的软复位可能不够彻底。烧录后应使用 OpenOCD 强制复位：

```bash
openocd -f interface/stlink-dap.cfg -f target/stm32f1x.cfg \
  -c "init; reset run; sleep 1000; shutdown"
```

### UART 无响应排查流程

1. OpenOCD `reset run` 强制复位
2. 抓 SWO 确认 `[BOOT]` / `[COMM] Task started` 输出
3. SWO 正常 → UART 能工作，直接测试
4. 不要因为 Voltage=0.00V 就怀疑电源或改代码

## 编码器接线

### A/B 通道接反

当前硬件编码器 A/B 通道接反，软件通过反转 TIM8 IC1 极性补偿：

```c
sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;  // 正常应为 RISING
```

参见 `Core/Src/main.c` TIM8 初始化。硬件修改后需改回 `TIM_ICPOLARITY_RISING`。
