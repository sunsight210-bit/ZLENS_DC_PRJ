# Phase 5 验证记录

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
- [x] test_motor_task
- [x] test_comm_task
- [x] test_storage_task
- [x] test_self_test
- [x] test_monitor_task

## 固件编译
- [x] `ninja -C build/fw` 无错误 (54848 text, 228 data, 17588 bss)

## 板上集成测试 (26/26 通过)
- [x] handshake 回显 (param=0x1234)
- [x] query_status = READY (0x0000)
- [x] query_type = 0x0004
- [x] homing 完成 (result=OK)
- [x] query_range 两帧 (0.6x ~ 7.0x)
- [x] query_zoom (1.8x)
- [x] set_zoom 6.0x + query 确认
- [x] busy_rejection (param=0xFFFF)
- [x] force_stop + status=READY
- [x] self_test ACK (param=OK)

## Power-Down Recovery (4/4 通过)
- [x] 复位后设备响应 status=READY
- [x] Handshake echo 正确
- [x] 变倍位置从 FRAM 恢复 (2.0x)
- [x] 镜头类型 = 0x0004

## 修复记录
- 产品代码缺陷：motor_task 未更新 SystemManager 状态，导致 BUSY 拒绝失效
  - 修复：motor_task 运动开始/结束时调用 transition_to(BUSY/HOMING/READY)
  - system_manager.hpp: is_busy() 覆盖 HOMING 状态
- 测试脚本修正：set_zoom/busy_rejection 目标值超出实际变倍范围 (0.6x~7.0x)
