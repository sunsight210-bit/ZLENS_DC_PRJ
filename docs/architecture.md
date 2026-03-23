# ZLENS_DC 系统架构分析

## 1. 项目概况

ZLENS_DC 是基于 STM32F103RCT6 的**电动变倍镜头控制系统**，运行 FreeRTOS，采用 4 任务架构。
当前已完成 Phase 1-6，所有功能模块就绪。

---

## 2. 系统总架构

```mermaid
graph TB
    subgraph "外部接口"
        HOST["镜头主机<br/>UART2 协议通信"]
        STLINK["ST-Link V3<br/>SWD + SWO 调试"]
    end

    subgraph "FreeRTOS Tasks"
        MT["<b>MotorTask</b><br/>Priority 4 · 1ms<br/>电机控制 + 编码器<br/>堵转检测 + 归零"]
        CT["<b>CommTask</b><br/>Priority 3 · 事件驱动<br/>UART DMA 收发<br/>协议解析 + 分发"]
        ST["<b>StorageTask</b><br/>Priority 2 · 事件+500ms<br/>FRAM 参数持久化"]
        MoT["<b>MonitorTask</b><br/>Priority 1 · 20ms<br/>系统状态机 + 自检<br/>看门狗 + 电压监控"]
    end

    subgraph "App 核心模块"
        MC["MotorCtrl<br/>梯形加减速 PWM"]
        ENC["Encoder<br/>32位编码器位置"]
        SD["StallDetect<br/>堵转/过流检测"]
        ZT["ZoomTable<br/>倍率-位置映射"]
        CP["CommProtocol<br/>帧解析 v2.5"]
        FS["FramStorage<br/>SPI FRAM 存储"]
        SM["SystemManager<br/>系统状态机"]
        PM["PowerMonitor<br/>供电电压监控"]
        SELF["SelfTest<br/>多步自检"]
        AF["AdcFilter<br/>IIR 滤波"]
        CRC["CRC16<br/>Modbus 校验"]
        SWO["SwoDebug<br/>SWO 调试输出"]
    end

    subgraph "硬件外设"
        TIM3["TIM3<br/>PWM 15kHz"]
        TIM8["TIM8<br/>编码器 4x"]
        ADC1["ADC1 + DMA<br/>电流/电压"]
        DAC["DAC CH2<br/>电流限值"]
        SPI2["SPI2<br/>FRAM"]
        UART2["USART2<br/>DMA RX"]
        IWDG["IWDG<br/>看门狗"]
    end

    HOST -->|"协议帧"| UART2
    UART2 -->|"协议帧"| HOST
    STLINK -.->|"SWD + SWO PB3"| SWO

    CT --> CP
    CT -->|"g_cmdQueue"| MT
    MT -->|"g_rspQueue"| CT
    MT -->|"g_saveQueue"| ST
    MoT -->|"TaskNotify"| MT

    MT --> MC
    MT --> ENC
    MT --> SD
    MT --> ZT
    CT --> CRC
    ST --> FS
    ST --> CRC
    MoT --> SM
    MoT --> PM
    MoT --> SELF

    MC --> TIM3
    MC --> DAC
    ENC --> TIM8
    SD --> AF
    AF --> ADC1
    FS --> SPI2
    CP --> UART2
    MoT --> IWDG
```

---

## 3. 任务间通信

```mermaid
flowchart LR
    subgraph Queues["消息队列"]
        Q1["g_cmdQueue<br/>8 x CMD_MESSAGE_S<br/>cmd + param"]
        Q2["g_rspQueue<br/>8 x RSP_MESSAGE_S<br/>cmd + param"]
        Q3["g_saveQueue<br/>4 x SAVE_MESSAGE_S<br/>position + zoom + reason"]
    end

    CT["CommTask"] -->|"运动/归零命令"| Q1 -->|"非阻塞轮询"| MT["MotorTask"]
    MT -->|"到达/堵转/过流响应"| Q2 -->|"非阻塞轮询"| CT
    MT -->|"位置快照"| Q3 -->|"非阻塞轮询"| ST["StorageTask"]

    MoT["MonitorTask"] -->|"xTaskNotify<br/>掉电信号"| MT
```

---

## 4. 各任务架构与数据流

### 4.1 MotorTask — 电机控制任务 (Priority 4, 1ms)

**职责**: 电机状态机、梯形加减速、编码器读取、堵转/过流保护、软限位、归零序列、周期运动

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> MOVING: SET_ZOOM / ZOOM_INC / ZOOM_DEC
    IDLE --> HOMING_REVERSE: HOMING 命令
    IDLE --> CYCLING: CYCLE_START

    MOVING --> IDLE: 到达目标位置
    MOVING --> IDLE: FORCE_STOP / 堵转 / 过流

    HOMING_REVERSE --> HOMING_RETRACT: 堵转检测反向极限
    HOMING_RETRACT --> HOMING_FIND_Z: 回退完成
    HOMING_FIND_Z --> HOMING_FORWARD: 检测到Z信号
    HOMING_FORWARD --> IDLE: 堵转检测正向极限, 归零完成

    CYCLING --> IDLE: CYCLE_STOP / 循环完成
```

```mermaid
flowchart TD
    START["run_once() — 每 1ms"]
    START --> CHK_PD["检查 TaskNotify<br/>掉电信号?"]
    CHK_PD -->|"是"| EMSTOP["紧急停止 + 保存位置"]
    CHK_PD -->|"否"| READ["读取编码器位置<br/>读取 ADC 电流值"]
    READ --> STALL["StallDetect::update()<br/>堵转/过流判断"]
    STALL --> MOTOR["MotorCtrl::update()<br/>梯形加减速"]
    MOTOR --> OC{"过流?<br/>ADC超过1200"}
    OC -->|"是"| EMSTOP2["emergency_stop()<br/>发送 OVERCURRENT 响应"]
    OC -->|"否"| STCHK{"堵转?<br/>ADC超200持续1000周期"}
    STCHK -->|"是"| HSTALL["handle_stall()<br/>发送STALL_STOP响应"]
    STCHK -->|"否"| QRCV["xQueueReceive g_cmdQueue<br/>非阻塞"]
    QRCV --> DISPATCH["dispatch_command()"]
    DISPATCH --> PROCESS["process_moving()<br/>process_homing()<br/>process_cycling()"]
    PROCESS --> DONE["发送响应 → g_rspQueue<br/>保存位置 → g_saveQueue"]
```

**依赖模块**: MotorCtrl, Encoder, StallDetect, ZoomTable, AdcFilter

---

### 4.2 CommTask — 通信任务 (Priority 3, 事件驱动)

**职责**: UART DMA 接收、协议帧解析(工作/工厂模式)、CRC 校验、命令分发、响应发送、UART 看门狗

```mermaid
flowchart TD
    START["run_once() — 事件驱动 ~10ms"]

    START --> RSP["xQueueReceive(g_rspQueue)<br/>非阻塞轮询"]
    RSP -->|"有响应"| SEND_RSP["构建响应帧 + CRC<br/>HAL_UART_Transmit()"]
    RSP -->|"无"| RXCHK{"s_bRxReady?<br/>DMA 空闲中断"}

    RXCHK -->|"否"| WDG["UART 看门狗检测<br/>gState/RxState/DMA 卡死"]
    WDG -->|"异常"| RECOVERY["2级恢复<br/>abort+retry<br/>DeInit+Init"]

    RXCHK -->|"是"| PARSE["on_frame_received()"]
    PARSE --> MODE{"工厂模式?"}
    MODE -->|"是"| FACTORY["parse_factory_frame()<br/>8 字节帧"]
    MODE -->|"否"| WORK["parse_work_frame()<br/>6 字节帧"]

    FACTORY --> ECHO1["握手回显"]
    WORK --> ECHO2["握手回显"]

    ECHO1 --> FDISPATCH["dispatch_factory_command()"]
    ECHO2 --> WDISPATCH["dispatch_work_command()"]

    WDISPATCH --> CMDTYPE{"命令类型"}
    CMDTYPE -->|"查询命令<br/>0x20-0x25"| DIRECT["直接回复<br/>zoom/status/speed/type"]
    CMDTYPE -->|"运动命令<br/>0x01/0x10-0x12"| QSEND["xQueueSend(g_cmdQueue)<br/>转发 MotorTask"]
    CMDTYPE -->|"自检/循环"| QSEND
```

**依赖模块**: CommProtocol, CRC16

**UART 接收链路**:
```
UART2 RX DMA → 空闲线检测 ISR → s_bRxReady 标志 → CommTask 轮询处理
```

---

### 4.3 StorageTask — 存储任务 (Priority 2, 事件+周期)

**职责**: FRAM 参数持久化(位置、倍率、总行程等)、启动恢复、事件驱动保存 + 周期保存

```mermaid
flowchart TD
    START["run_once()"]
    START --> QRCV["xQueueReceive(g_saveQueue)<br/>非阻塞"]
    QRCV -->|"有消息"| UPDATE["更新内存参数<br/>position, zoom_x10, reason"]
    QRCV -->|"无"| TIMER{"距上次保存<br/>> 500ms?"}

    UPDATE --> SAVE["FramStorage::save_params()<br/>SPI2 写入"]
    TIMER -->|"是 + 位置变化"| SAVE
    TIMER -->|"否"| SLEEP["vTaskDelay(10ms)"]

    SAVE --> CRC_CALC["CRC16 计算"]
    CRC_CALC --> PRIMARY["写入主地址"]
    PRIMARY --> BACKUP["写入备份地址"]
    BACKUP --> SLEEP
```

**存储内容**:
| 参数 | 说明 |
|------|------|
| position (int32) | 编码器绝对位置 |
| zoom_x10 (uint16) | 当前倍率 ×10 |
| total_range (int32) | 总行程(归零后测得) |
| bHomingDone (bool) | 是否已完成归零 |
| overflow_cnt (int32) | 编码器溢出计数 |
| baseline_current (uint16) | 基准电流 |

**依赖模块**: FramStorage, CRC16

---

### 4.4 MonitorTask — 监控任务 (Priority 1, 20ms)

**职责**: 系统状态机管理、自检编排、看门狗喂狗、掉电检测、首次/正常启动判断

```mermaid
stateDiagram-v2
    [*] --> INIT: 上电
    INIT --> SELF_TEST: 首次启动,未归零
    INIT --> READY: 正常启动,FRAM恢复
    SELF_TEST --> HOMING: 自检通过
    SELF_TEST --> FAULT: 自检失败
    HOMING --> READY: 归零完成
    READY --> BUSY: 收到运动命令
    BUSY --> READY: 运动完成/停止
    READY --> FAULT: 严重错误
    BUSY --> FAULT: 严重错误
```

```mermaid
flowchart TD
    START["run_once() — 每 20ms"]
    START --> WDG["HAL_IWDG_Refresh()<br/>喂狗"]
    WDG --> VOLT["PowerMonitor::update()<br/>电压检测"]
    VOLT --> PD{"电压过低?"}
    PD -->|"是"| NOTIFY["xTaskNotify(MotorTask)<br/>掉电信号"]
    PD -->|"否"| STATE{"当前状态?"}

    STATE -->|"INIT"| BOOT{"FRAM 已归零?"}
    BOOT -->|"是"| RESTORE["恢复参数 → READY"]
    BOOT -->|"否"| SELFTEST["启动自检序列"]

    STATE -->|"SELF_TEST"| STEPS["执行自检步骤:<br/>1. 电压检查<br/>2. 基准电流采样<br/>3. 编码器方向验证<br/>4. 归零序列<br/>5. 行程验证<br/>6. FRAM 读写校验"]

    STATE -->|"READY/BUSY"| CYCLE["周期运动管理<br/>(如有)"]
```

**自检项目 (SelfTest)**:
1. **电压检查** — ADC 读取供电电压是否在范围内
2. **基准电流** — 空载时采样 ADC 电流基准值
3. **编码器方向** — 短暂驱动电机，验证编码器增减方向正确
4. **归零序列** — 完整归零流程
5. **行程验证** — 测量总行程是否在合理范围
6. **FRAM 校验** — 写入/读回验证 SPI FRAM 正常

**依赖模块**: SystemManager, PowerMonitor, SelfTest, SwoDebug

---

## 5. 完整数据流 — Zoom 命令示例

```mermaid
sequenceDiagram
    participant Host as 镜头主机
    participant UART as UART2 DMA
    participant Comm as CommTask
    participant CmdQ as g_cmdQueue
    participant Motor as MotorTask
    participant RspQ as g_rspQueue
    participant SaveQ as g_saveQueue
    participant Storage as StorageTask
    participant FRAM as SPI FRAM

    Host->>UART: [A5 10 xx xx CRC_H CRC_L]
    UART-->>Comm: DMA 空闲中断 → s_bRxReady

    Comm->>Comm: parse_work_frame() + CRC 校验
    Comm->>Host: 握手回显(原帧)
    Comm->>CmdQ: CMD_MESSAGE_S{0x10, zoom_x10}

    CmdQ-->>Motor: xQueueReceive (非阻塞)
    Motor->>Motor: ZoomTable::get_position(zoom_x10)
    Motor->>Motor: clamp_to_soft_limits(target)
    Motor->>Motor: MotorCtrl::move_to(target)
    Note over Motor: 状态=MOVING, 系统=BUSY

    loop 每 1ms
        Motor->>Motor: Encoder::get_position()
        Motor->>Motor: MotorCtrl::update() 梯形加减速
        Motor->>Motor: StallDetect::update()
    end

    Motor->>Motor: 到达目标, stop()
    Motor->>RspQ: RSP_MESSAGE_S{0x02, zoom_x10}
    Motor->>SaveQ: SAVE_MESSAGE_S{pos, zoom, ARRIVED}

    RspQ-->>Comm: xQueueReceive
    Comm->>Host: [A5 02 xx xx CRC_H CRC_L]

    SaveQ-->>Storage: xQueueReceive
    Storage->>FRAM: save_params(主+备份)
```

---

## 6. 硬件外设映射

```mermaid
graph LR
    subgraph MCU["STM32F103RCT6"]
        TIM3["TIM3 PWM<br/>PA6(CH1) PA7(CH2)"]
        TIM8["TIM8 Encoder<br/>PA0(CH1) PA1(CH2)"]
        ADC1["ADC1 DMA<br/>PC0(电流) PB0(电压)"]
        DAC["DAC CH2<br/>PA5(电流限值)"]
        SPI2["SPI2<br/>PB12-15"]
        UART2["USART2<br/>PA2(TX) PA3(RX)"]
        SWO_PIN["SWO PB3"]
        IWDG_P["IWDG"]
    end

    TIM3 -->|"PWM 15kHz"| MOTOR["DC 电机<br/>H 桥驱动"]
    TIM8 -->|"4x 正交解码"| ENC_HW["增量编码器"]
    ADC1 -->|"连续采样"| SENSE["电流/电压传感"]
    DAC -->|"VREF"| ILIMIT["电流限值基准"]
    SPI2 -->|"读写"| FRAM_HW["FRAM 芯片"]
    UART2 -->|"115200 baud"| HOST_HW["镜头主机"]
    SWO_PIN -->|"ITM trace"| STLINK_HW["ST-Link"]
```

---

## 7. 模块职责一览

| 模块 | 文件 | 职责 |
|------|------|------|
| **MotorCtrl** | `App/*/motor_ctrl.*` | PWM 控制、梯形加减速、方向切换、DAC 电流限值 |
| **Encoder** | `App/*/encoder.*` | TIM8 正交解码、32 位扩展(溢出中断)、无竞争读取 |
| **StallDetect** | `App/*/stall_detect.*` | 电流阈值堵转检测、过流保护、消隐期管理 |
| **ZoomTable** | `App/*/zoom_table.*` | 倍率-编码器位置映射表、线性插值 |
| **FramStorage** | `App/*/fram_storage.*` | SPI FRAM 读写、主/备份双地址、CRC 校验 |
| **CommProtocol** | `App/*/comm_protocol.*` | 工作/工厂模式帧解析、CRC 校验、协议 v2.5 |
| **SystemManager** | `App/*/system_manager.*` | 系统状态机(INIT→SELF_TEST→HOMING→READY→BUSY→FAULT) |
| **PowerMonitor** | `App/*/power_monitor.*` | ADC 供电电压监控、掉电预警 |
| **SelfTest** | `App/*/self_test.*` | 多步自检编排(电压→基准→编码器→归零→行程→FRAM) |
| **AdcFilter** | `App/*/adc_filter.*` | IIR 低通滤波(alpha=1/16) |
| **CRC16** | `App/*/crc16.*` | CRC16-Modbus 校验算法 |
| **SwoDebug** | `App/*/swo_debug.*` | SWO ITM 调试打印(PB3) |
| **AppInstances** | `App/*/app_instances.*` | 全局单例 + 外设初始化 + 任务创建 |
