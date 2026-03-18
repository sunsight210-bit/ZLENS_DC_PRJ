# Phase 5 Review

## 代码审查
- [x] 命名规范符合 coding-style.md
  - SystemManager 指针: m_pSm (成员+指针前缀)
  - transition_to 调用参数使用 SYSTEM_STATE_E 枚举
- [x] 无产品代码为迎合测试而修改
  - motor_task BUSY 状态管理是产品缺陷修复，非测试驱动
  - 测试脚本目标值修正是测试代码自身 bug
- [x] 全局标志模式与 g_bSpiEmergency 一致

## 集成质量
- [x] Python CRC16 与固件精确匹配 (crc16_modbus)
- [x] 所有自动化测试可重复执行 (integration_test.py --port /dev/ttyUSB0)
- [x] CLAUDE.md 添加硬件操作告知规则

## 改动文件清单
- `CLAUDE.md` — 添加硬件操作告知规则
- `App/Inc/system_manager.hpp` — is_busy() 覆盖 HOMING 状态
- `Tasks/Inc/motor_task.hpp` — 添加 SystemManager* 成员和 init 参数
- `Tasks/Src/motor_task.cpp` — 运动开始/结束时更新 SM 状态
- `App/Src/app_instances.cpp` — motor_task_entry 传入 g_SystemManager
- `scripts/integration_test.py` — 修正变倍目标值到实际范围
- `Test/CMakeLists.txt` — test_motor_task 添加 system_manager.cpp
- `Test/test_motor_task.cpp` — SetUp 传入 SystemManager
