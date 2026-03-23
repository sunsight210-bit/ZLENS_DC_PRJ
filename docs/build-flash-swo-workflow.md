# 编译-烧录-SWO 调试工作流

本文档是硬件调试的**唯一参考**。AI 执行硬件操作前必须先读取本文档。

## 硬件安全规则

- 执行烧录、电机运动、串口测试前，**必须先告知用户当前步骤和预期行为**
- 烧录后 UART 无响应，**第一步 OpenOCD `reset run` + SWO 确认**，不改代码

## 环境信息

| 项目 | 值 |
|------|-----|
| MCU | STM32F103RC |
| 系统时钟 | **64MHz**（HSI/2=4MHz × PLL_MUL16），参见 `Core/Src/main.c` `SystemClock_Config()` |
| SWO 引脚 | PB3 |
| 调试器 | ST-Link V3（stlink-dap 模式） |
| 镜头串口 | `/dev/ttyUSB0`（USB-TTL 适配器） |
| ST-Link 虚拟串口 | `/dev/ttyACM0`（不用于通信） |

## 硬件陷阱

### ST-Link 必须断开 TVCC 引脚

ST-Link V3 的 TVCC 引脚会与板载 3.3V LDO 形成反向供电，导致：

1. **电路板发烫** — TVCC 与板载电源冲突，产生异常功耗
2. **MCU 无法正常 POR** — 镜头掉电后，SWD 引脚通过 ESD 保护二极管提供寄生电流，MCU 不会真正断电，上电后外设状态残留，UART 不工作
3. **Voltage 显示 0.00V 误导判断** — 断开 TVCC 后 ST-Link 检测不到目标电压，但 MCU 实际由板载 LDO 正常供电

**正确接线**：只保留 4 根线

| 引脚 | 功能 | 必须 |
|------|------|------|
| SWDIO | 调试数据 | Yes |
| SWCLK | 调试时钟 | Yes |
| GND | 地线 | Yes |
| SWO (PB3) | 调试输出 | Yes |
| TVCC | **断开** | No |

### 编码器 A/B 通道接反

当前硬件编码器 A/B 通道接反，软件通过反转 TIM8 IC1 极性补偿：

```c
sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;  // 正常应为 RISING
```

参见 `Core/Src/main.c` TIM8 初始化。硬件修改后需改回 `TIM_ICPOLARITY_RISING`。

### UART 无响应排查流程

1. OpenOCD `reset run` 强制复位
2. 抓 SWO 确认 `[BOOT]` / `[COMM] Task started` 输出
3. SWO 正常 → UART 能工作，直接测试
4. 不要因为 Voltage=0.00V 就怀疑电源或改代码

## 工具链

### 编译工具

| 用途 | 工具 | 路径/版本 |
|------|------|-----------|
| 固件交叉编译 | arm-none-eabi-gcc 10.3.1 | STM32CubeIDE 内置：`/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin/` |
| 单元测试编译 | g++ (host) | `/usr/bin/g++` |
| 构建系统 | CMake 3.22 | `/usr/bin/cmake` |
| 覆盖率 | gcov | `/usr/bin/gcov` |

固件编译工具链由 `cmake/stm32f103rc.cmake` 自动配置，无需手动指定。

### 烧录与调试工具

| 用途 | 变量名 | 完整路径 |
|------|--------|----------|
| OpenOCD | `OPENOCD` | `/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_2.4.400.202601091506/tools/bin/openocd` |
| OpenOCD 脚本 | `OCD_SCRIPTS` | `/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.debug.openocd_2.3.300.202602021527/resources/openocd/st_scripts/` |
| 烧录器 | `PROGRAMMER` | `/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.400.202601091506/tools/bin/STM32_Programmer_CLI` |

### 脚本工具

| 用途 | 工具 | 路径/版本 |
|------|------|-----------|
| 验证脚本 / SWO 解析 | Python 3.10 | `/usr/bin/python3` |
| 串口通信 | pyserial 3.5 | `pip3 install pyserial` |

### AI 使用方式

在同一条 Bash 命令中用 `&&` 连接变量定义和实际命令：
```bash
OPENOCD="..." && OCD_SCRIPTS="..." && $OPENOCD -s "$OCD_SCRIPTS" ...
```

## 操作流程

按编号顺序执行。根据场景选择路径 A 或路径 B。

### 1. 单元测试

```bash
cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && ctest --test-dir build/test --output-on-failure
```

**通过标准**：全部 PASS，不允许跳过失败继续。

### 2. 固件编译

```bash
cmake -B build/fw -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake && cmake --build build/fw
```

**输出确认**：末尾有 `text data bss` 行即编译成功。记录 text 段大小用于 verify.md。

### 3. 烧录

```bash
PROGRAMMER="..." && $PROGRAMMER -c port=SWD -w build/fw/ZLENS_DC.bin 0x08000000
```

**成功标志**：`File download complete`。

烧录后 MCU 因 SWD 寄生电流可能无法正常 POR，**必须接步骤 4**。

### 4. 复位 + 运行（二选一）

烧录后必须复位。根据是否需要 SWO 日志，选择路径 A 或路径 B：

#### 路径 A：仅复位运行（不需要 SWO）

```bash
OPENOCD="..." && OCD_SCRIPTS="..." && $OPENOCD -s "$OCD_SCRIPTS" \
  -f interface/stlink-dap.cfg -f target/stm32f1x.cfg \
  -c "init; reset run; sleep 1000; shutdown"
```

板子复位后正常运行，可以通过串口操作。`Target voltage: 0.00xV` 是正常的（TVCC 已断开）。

#### 路径 B：复位 + SWO 抓取（需要调试日志）

```bash
OPENOCD="..." && OCD_SCRIPTS="..." && $OPENOCD -s "$OCD_SCRIPTS" \
  -f interface/stlink-dap.cfg -f target/stm32f1x.cfg \
  -c "init; reset halt; \
      tpiu config internal build/swo/swo.bin uart off 64000000 2000000; \
      itm port 0 on; \
      resume; \
      sleep <毫秒数>; \
      shutdown"
```

`sleep` 时间根据场景选择：

| 场景 | sleep 值 | 说明 |
|------|----------|------|
| 仅看启动日志 | 5000 | 抓 BOOT + SELF-TEST |
| 回零测试 | 15000 | 等回零完成 |
| 全量诊断 | 60000 | 自检+回零+间隙+精度 |
| 等待用户手动操作 | 30000 | 用户通过上位机发命令 |

**SWO 踩坑记录（必须遵守）：**

1. **必须 `reset halt` → 配置 TPIU → `resume`**，不能 `reset run` 后再配 TPIU（日志为空）
2. **用 `tpiu config internal ...` 旧语法**，不要用 `tpiu create` 新语法（不可靠）
3. **traceclk 必须是 64000000**（64MHz），不是 72MHz

### 5. SWO 日志解析

```bash
python3 scripts/parse_swo.py build/swo/swo.bin
```

脚本自动将解析后的文本保存到 `build/swo/swo.txt`，同时输出到终端。

**验证抓取成功**：输出中应包含 `[BOOT] ZLENS_DC v1.0 starting...`，如果为空说明 SWO 配置有问题。

### 6. 串口命令发送

镜头通信协议 v2.5，帧格式：`[0xA5] [CMD] [PARAM_H] [PARAM_L] [CRC16_L] [CRC16_H]`

```python
import serial, struct

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF

def send_cmd(port, cmd, param=0):
    ser = serial.Serial(port, 115200, timeout=2)
    hdr = struct.pack('>BBH', 0xA5, cmd, param)
    frame = hdr + struct.pack('<H', crc16(hdr))
    ser.write(frame)
    return ser

# 常用命令
# send_cmd('/dev/ttyUSB0', 0x01, 0)     # HOMING (param=0 普通, param=1 全诊断)
# send_cmd('/dev/ttyUSB0', 0x10, 20)    # SET_ZOOM 2.0x
# send_cmd('/dev/ttyUSB0', 0x21, 0)     # QUERY_STATUS
# send_cmd('/dev/ttyUSB0', 0x60, 0)     # SELF_TEST (0x60 触发全诊断链)
```

如需 SWO 抓取同时发串口命令，将步骤 4B 以后台模式（`run_in_background`）运行，等 OpenOCD 就绪后（约 3 秒）再发串口命令。

## SWO 日志关键标记

| 标记 | 含义 | 预期 |
|------|------|------|
| `[BOOT] ZLENS_DC v1.0 starting...` | 系统启动 | 总是出现 |
| `[MOTOR] Task started` | 电机任务启动 | 总是出现 |
| `[COMM] Task started` | 通信任务启动 | 总是出现 |
| `SELF-TEST REPORT ... ALL PASSED` | 自检通过 | 正常启动 |
| `[MOTOR] CMD 0xNN param=0xNNNN` | 收到命令 | 发命令后 |
| `[INFO] Homing started` | 回零开始 | HOMING 命令后 |
| `[INFO] Homing: coarse limit found` | 粗定位完成 | 编码器/电流堵转 |
| `[INFO] Homing: retract done` | 回退完成 | 粗定位后 |
| `[INFO] Homing: precise limit found` | 精定位完成 | 慢速堵转 |
| `[PASS] Homing complete` | 回零成功 | 最终状态 |
| `[FAIL] Unexpected stall` | 异常堵转 | 需排查 |
| `[MOTOR] HB state=N motor=N pos=N` | 心跳（每 5s） | 持续运行时 |
