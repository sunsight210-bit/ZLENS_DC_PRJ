# Phase 5: 端到端集成验证

## 目标
验证上位机命令→电机动作→响应回传完整链路，补全 0x60 SELF_TEST 命令。

## 改动范围

### 固件
- `task_config.hpp`: 添加 `g_bUartSelfTestReq` 全局标志
- `comm_task.cpp`: 0x60 dispatch → 置标志 + ACK
- `monitor_task.cpp`: READY 状态检测标志 → 触发自检
- `system_manager.cpp`: READY→SELF_TEST 合法转换

### 测试
- `test_comm_task.cpp`: 0x60 ACK 测试
- `test_system_manager.cpp`: READY→SELF_TEST 转换测试
- `test_monitor_task.cpp`: UART 自检触发测试

### 集成测试
- `scripts/integration_test.py`: Python 串口自动化测试
- `scripts/integration_full.sh`: 编译→烧录→测试全流程

## 质量门控
Mock 测试通过 → 固件编译 → 板上集成测试通过
