# Phase 6: 全面对齐通信协议 v2.5

## 目标
修正所有命令 ID、响应格式、握手回显机制，使固件完全符合协议 v2.5 规范。

## 改动范围

### 固件
- `Tasks/Inc/task_config.hpp`: 重写命令常量 (cmd/rsp_cmd/rsp namespace)
- `Tasks/Inc/comm_task.hpp`: 添加 m_iSpeedKhz、m_iStallCount 成员
- `Tasks/Src/comm_task.cpp`: 重写命令分发、握手回显、查询响应 (0x1X cmd)
- `Tasks/Src/motor_task.cpp`: 更新命令 ID + 响应格式 (rsp_cmd)
- `App/Inc/fram_storage.hpp`: 添加 motor_speed_khz 字段
- `Tasks/Src/storage_task.cpp`: 启动恢复速度

### 测试
- `Test/test_comm_task.cpp`: 握手回显、查询响应 0x1X、BUSY 格式、新命令
- `Test/test_motor_task.cpp`: 命令 ID 更新、响应格式更新、ZOOM_INC/DEC

## 关键变更

### A. 命令 ID 映射
| 协议 Cmd | 功能 | 旧常量 | 新常量 |
|----------|------|--------|--------|
| 0x01 | 回原点 | HANDSHAKE | HOMING |
| 0x02 | 强制停止 | HOMING | FORCE_STOP |
| 0x03 | 获取堵转计数 | FORCE_STOP | GET_STALL_CNT |
| 0x11 | 倍率增加 | CYCLE_START | ZOOM_INC |
| 0x12 | 倍率减少 | CYCLE_STOP | ZOOM_DEC |
| 0x22 | 查询速度 | QUERY_RANGE | QUERY_SPEED |
| 0x24 | 查询范围 | — | QUERY_RANGE |
| 0x25 | 查询版本 | — | QUERY_VERSION |
| 0x30 | 循环变倍 | SWITCH_FACTORY | CYCLE_START |
| 0x31 | 停止循环 | — | CYCLE_STOP |
| 0xFA | 工厂模式 | — | SWITCH_FACTORY |

### B. 响应命令字 (rsp_cmd namespace)
- 查询响应使用 0x1X (非 0x2X)
- 到位帧: 0x02 + 0x000A
- 回原点完成: 0x01 + 0x000F
- 忙/无效: 0x03 + 0x000E
- 堵转: 0xE1, 过流: 0xE2

### C. 握手机制
所有命令收到后先原样回显原始帧字节，再处理命令。

## 质量门控
Mock 测试通过 (覆盖率 >80%) → 固件编译 → 板上测试通过
