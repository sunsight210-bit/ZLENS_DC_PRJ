# 参数存储映射表

MCU: STM32F103RC (256KB Flash, 2KB/页)  
外部存储: FRAM (SPI, FM25 系列)

## 存储区域总览

| 区域 | 存储介质 | 地址 | 大小 | 副本 | 用途 |
|------|----------|------|------|------|------|
| FRAM 主区 | FRAM | `0x0000` | 32B | 双副本 | 运行时状态 |
| FRAM 备份区 | FRAM | `0x0020` | 32B | ↑ | 主区镜像 |
| Flash 配置区 | 内部 Flash | `0x0803F000` | 48B (1页 2KB) | 单副本+CRC | 产品配置 |
| Flash ZoomTable | 内部 Flash | `0x0803F800` | ≤260B (1页 2KB) | 单副本+CRC | 不规则倍率映射表 |

## Flash 空间分布

```
0x08000000 ┌──────────────────────┐
           │ 固件代码 (~43.5KB)    │
0x0800AE00 ├──────────────────────┤
           │ 可用空间 (~209KB)     │
0x0803F000 ├──────────────────────┤
           │ Flash 配置区 (1页)    │
0x0803F800 ├──────────────────────┤
           │ ZoomTable (1页)      │
0x08040000 └──────────────────────┘
```

## FRAM 运行时状态 — FRAM_STATE_S (32B)

高频写入，掉电必须保留。双副本保护。

| 偏移 | 参数 | 类型 | 字节 | 写入频率 | 用途 |
|------|------|------|------|----------|------|
| 0 | `magic` | uint16 | 2 | 每次保存 | 有效性标识 |
| 2 | `current_position` | int32 | 4 | 每次到位/掉电 | 编码器当前位置 |
| 6 | `current_zoom_x10` | uint16 | 2 | 每次到位 | 当前倍率 ×10 |
| 8 | `homing_done` | uint8 | 1 | 回零完成 | 上电是否需要回零 |
| 9 | `position_valid` | uint8 | 1 | 每次到位 | 上电位置是否可信 |
| 10 | `move_count` | uint16 | 2 | 每次移动 | 超限强制回零 (阈值 500) |
| 12 | `last_save_reason` | uint8 | 1 | 每次保存 | 诊断：上次保存原因 |
| 13 | `reserved[17]` | — | 17 | — | 预留扩展 |
| 30 | `crc16` | uint16 | 2 | 每次保存 | CRC16-Modbus 校验 |

### FRAM 地址分配

- 主区: `0x0000` ~ `0x001F` (32B)
- 备份区: `0x0020` ~ `0x003F` (32B)

### 写入规则

- `save_state()`: 同时写主区 + 备份区
- `load_state()`: 先读主区，magic/CRC 失败则读备份区
- `emergency_save()`: 掉电中断中只写主区 `current_position` + `position_valid`

### 上电启动判断

```
magic 有效 + homing_done==1 + position_valid==0xFF + move_count<500
  → 正常启动（跳过回零）

magic 有效 + homing_done==1
  → 仅回零

其他（magic 无效 / CRC 失败）
  → 首次启动，强制回零
```

## Flash 配置区 — FLASH_CONFIG_S (48B)

低频写入（出厂标定或现场调参），运行中永远不写 Flash。  
CRC 校验失败时回退硬编码默认值。

| 偏移 | 参数 | 类型 | 字节 | 写入频率 | 用途 |
|------|------|------|------|----------|------|
| 0 | `magic` | uint16 | 2 | 标定/调参 | 有效性标识 |
| 2 | `speed_duty` | uint16 | 2 | 调速 | 默认速度 duty ×10 (0~1000) |
| 4 | `min_speed_duty` | uint16 | 2 | 调速 | 最小速度 duty ×10 |
| 6 | `max_speed_duty` | uint16 | 2 | 调速 | 最大速度 duty ×10 |
| 8 | `min_position` | int32 | 4 | 出厂标定 | 编码器行程下限 |
| 12 | `max_position` | int32 | 4 | 出厂标定 | 编码器行程上限 |
| 16 | `min_zoom_x10` | uint16 | 2 | 出厂标定 | 最小倍率 ×10 |
| 18 | `max_zoom_x10` | uint16 | 2 | 出厂标定 | 最大倍率 ×10 |
| 20 | `zoom_step_x10` | uint16 | 2 | 出厂标定 | 规则表步进 (0=不规则表) |
| 22 | `lens_type` | uint8 | 1 | 出厂标定 | 镜头型号编号 |
| 23 | `stall_threshold` | uint16 | 2 | 现场调参 | 堵转电流阈值 |
| 25 | `reserved[21]` | — | 21 | — | 预留扩展 |
| 46 | `crc16` | uint16 | 2 | 写配置时 | CRC16-Modbus 校验 |

### 倍率表类型

- `zoom_step_x10 > 0`: 规则表，运行时由 `min_zoom_x10`/`max_zoom_x10`/`zoom_step_x10` 动态生成，不使用 ZoomTable Flash 页
- `zoom_step_x10 == 0`: 不规则表，从 ZoomTable Flash 页 (`0x0803F800`) 加载

### 写入规则

- 仅在用户主动操作时写入（调速命令、出厂标定、现场调参）
- 运行中永远不写 Flash
- 擦页 + 半字编程，写入前读出校验

## Flash ZoomTable 区 (0x0803F800)

不规则倍率映射表。仅当 `FLASH_CONFIG_S.zoom_step_x10 == 0` 时使用。

### 存储格式

| 偏移 | 内容 | 类型 | 字节 |
|------|------|------|------|
| 0 | `count` | uint8 (半字对齐) | 2 |
| 2 | `entries[0].zoom_x10` | uint16 | 2 |
| 4 | `entries[0].angle_x100` | uint16 | 2 |
| ... | ... | ... | ... |
| 2+N×4 | `crc16` | uint16 | 2 |

- 每条 `ZOOM_ENTRY_S`: `zoom_x10` (倍率×10) + `angle_x100` (角度×100)
- 最多 64 条 = 2 + 64×4 + 2 = 260B
- 角度→编码器位置: `position = HOME_OFFSET + angle_x100 × 65536 / 36000`

## 仅 RAM（不持久化）

| 参数 | 类型 | 用途 |
|------|------|------|
| `stall_count` | uint16 | 堵转次数统计，上电归零 |
| `baseline_current` | uint16 | 空载电流基线，每次上电重新采样 |

## 代码对应关系

| 存储区 | 类 | 文件 |
|--------|-----|------|
| FRAM 运行时状态 | `FramStorage` | `App/Inc/fram_storage.hpp`, `App/Src/fram_storage.cpp` |
| Flash 配置区 | `FlashConfig` | `App/Inc/flash_config.hpp`, `App/Src/flash_config.cpp` |
| Flash ZoomTable | `ZoomTable` | `App/Inc/zoom_table.hpp`, `App/Src/zoom_table.cpp` |
