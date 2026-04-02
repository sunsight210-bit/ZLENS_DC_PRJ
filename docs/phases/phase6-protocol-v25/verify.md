# Phase 6 验证记录

## Mock 单元测试 (15/15 通过)
- [x] test_crc16
- [x] test_comm_protocol
- [x] test_encoder
- [x] test_zoom_table
- [x] test_fram_storage
- [x] test_adc_filter
- [x] test_stall_detect
- [x] test_motor_ctrl
- [x] test_power_monitor
- [x] test_system_manager
- [x] test_motor_task (24 cases)
- [x] test_comm_task (26 cases)
- [x] test_storage_task
- [x] test_self_test
- [x] test_monitor_task

## 测试覆盖率
- [x] comm_task.cpp: **93.62%** (>80% 门控)
- [x] motor_task.cpp: **89.89%** (>80% 门控)

## 固件编译
- [x] `ninja -C build/fw` 无错误 (55456 text, 232 data, 17592 bss)

## 板上测试 (46/46 通过, 2026-03-19)

脚本: `scripts/phase6_verify.py --port /dev/ttyUSB0`

- [x] Query Zoom (0x20) → echo + rsp cmd=0x10, zoom=6.0x
- [x] Query Status (0x21) → echo + rsp cmd=0x11, status=READY
- [x] Query Speed (0x22) → echo + rsp cmd=0x12, speed=15kHz
- [x] Query Type (0x23) → echo + rsp cmd=0x13, type=0x0004
- [x] Query Range (0x24) → echo + 两帧 cmd=0x14(0.6x) + 0x15(7.0x)
- [x] Query Version (0x25) → echo + rsp cmd=0x16, version=0x1000
- [x] Get Stall Count (0x03) → echo + rsp cmd=0xE3, count=0
- [x] Self-Test (0x60) → echo + ACK cmd=0x60, param=OK
- [x] 握手回显: 全部 8 条命令均正确原样回显

## 新增/修改测试用例 (Phase 6 特有)

### test_comm_task 新增
- Handshake_QueryEchoesRawFrame: 查询命令先回显原始帧
- Handshake_MotionCommandEchoes: 运动命令先回显
- Handshake_FactoryModeEchoes: 工厂模式命令回显
- QueryZoom_Response0x10: 查询倍率响应 cmd=0x10
- QueryStatus_Response0x11: 查询状态响应 cmd=0x11
- QuerySpeed_Response0x12: 查询速度响应 cmd=0x12
- QueryType_Response0x13: 查询类型响应 cmd=0x13
- QueryRange_TwoFrames0x14_0x15: 查询范围两帧 0x14+0x15
- QueryVersion_Response0x16: 查询版本响应 cmd=0x16
- GetStallCount_Response0xE3: 堵转计数响应
- BusyReject_Response0x03_0x000E: 忙拒绝 {0x03, 0x000E}
- ZoomInc_ForwardsToMotor / ZoomDec_ForwardsToMotor: 相对变倍转发
- ZoomInc_BusyReject: 相对变倍忙拒绝
- CycleStart_ForwardsToMotor / CycleStop_ForwardsToMotor
- SpeedAccessor_DefaultAndSet / StallCountAccessor_DefaultAndSet
- RunOnce_ZoomResponseUpdatesCurrentZoom / RunOnce_ArrivedResponseNoZoomUpdate

### test_motor_task 新增/修改
- SetZoom_Arrived_TwoFrames: 到位发两帧 (ZOOM + ARRIVED)
- Homing_CmdIs0x01: 回原点使用 0x01
- HomingComplete_TwoFrames: 回原点完成发两帧 (ZOOM + HOMING_DONE)
- ForceStop_CmdIs0x02: 强制停止使用 0x02
- StallDetected_Response0xE1: 堵转告警 cmd=0xE1
- OvercurrentDetected_Response0xE2: 过流告警 cmd=0xE2
- ZoomInc_RelativeIncrease / ZoomInc_ClampToMax
- ZoomDec_RelativeDecrease / ZoomDec_ClampToMin
- CycleStart_CmdIs0x30 / CycleStop_CmdIs0x31
