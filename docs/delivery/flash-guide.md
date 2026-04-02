# ZLENS_DC 生产烧录指南

## 硬件要求

| 项目 | 要求 |
|------|------|
| 调试器 | ST-Link V3 (stlink-dap 模式) |
| 接线 | SWDIO + SWCLK + GND + SWO(PB3)，**不接 TVCC** |
| 供电 | 控制板自身供电 8.3V~24V DC |

**警告：不要连接 ST-Link 的 TVCC 引脚。** 会导致电路板发烫、MCU 上电异常。

## 软件要求

| 工具 | 用途 |
|------|------|
| STM32_Programmer_CLI | 烧录固件 |
| OpenOCD | 复位 + SWO 验证 |
| Python 3 + parse_swo.py | SWO 日志解析 |

工具路径在 STM32CubeIDE 安装目录下，完整路径见 `docs/dev/build-flash-swo-workflow.md`。

## 烧录步骤

### 第 1 步：烧录

```bash
STM32_Programmer_CLI -c port=SWD -w ZLENS_DC.bin 0x08000000
```

成功标志：`File download complete`

### 第 2 步：复位

烧录后 MCU 因 SWD 寄生电流可能无法正常上电，**必须执行复位**：

```bash
openocd -s <OCD_SCRIPTS> \
  -f interface/stlink-dap.cfg -f target/stm32f1x.cfg \
  -c "init; reset run; sleep 1000; shutdown"
```

### 第 3 步：验证（SWO 抓取）

```bash
openocd -s <OCD_SCRIPTS> \
  -f interface/stlink-dap.cfg -f target/stm32f1x.cfg \
  -c "init; reset halt; \
      tpiu config internal swo.bin uart off 64000000 2000000; \
      itm port 0 on; \
      resume; sleep 15000; shutdown"

python3 scripts/parse_swo.py swo.bin
```

验证标准：

| 日志 | 含义 | 预期 |
|------|------|------|
| `[BOOT] ZLENS_DC v1.0 starting...` | 系统启动 | 必须出现 |
| `[MOTOR] Task started` | 电机任务启动 | 必须出现 |
| `[COMM] Task started` | 通信任务启动 | 必须出现 |
| `[PASS] Homing complete` 或 `[PASS] Normal boot` | 启动成功 | 必须出现其一 |

## 常见失败及解决

| 症状 | 原因 | 解决 |
|------|------|------|
| `No STM32 target found` | ST-Link 未连接或接线错误 | 检查 SWDIO/SWCLK/GND 三根线 |
| `Error: init mode failed` | MCU 供电不足 | 确认控制板已通电 (>8.3V) |
| 烧录成功但 SWO 日志为空 | 未执行 reset halt | 必须先 `reset halt` 再配 TPIU |
| `Voltage: 0.00V` | TVCC 未接 | 正常现象，不影响烧录和运行 |
| 烧录后 UART 无响应 | SWD 寄生电流致 MCU 未 POR | 执行 OpenOCD `reset run` |
| SWO 日志只有乱码 | traceclk 参数错误 | 必须用 64000000（64MHz），不是 72MHz |
