# Phase 6 Review

## 代码审查
- [x] 命名规范符合 coding-style.md
  - 新成员: m_iSpeedKhz, m_iStallCount (成员+类型前缀)
  - 新 namespace: rsp_cmd (响应命令字)
  - 新方法: get_speed_khz(), set_stall_count() (snake_case)
- [x] 无产品代码为迎合测试而修改
  - 所有变更均为协议对齐需求，非测试驱动
- [x] 握手机制使用 const_cast 传递原始帧 (HAL API 要求 non-const)
- [x] FRAM 结构体添加 motor_speed_khz 字段，reserved 从 13→11，static_assert 仍 60 字节

## 协议对齐完整性
- [x] 命令 ID: 全部 11 个命令对齐 v2.5
  - 0x01=HOMING, 0x02=FORCE_STOP, 0x03=GET_STALL_CNT
  - 0x11=ZOOM_INC, 0x12=ZOOM_DEC
  - 0x22=QUERY_SPEED, 0x24=QUERY_RANGE, 0x25=QUERY_VERSION
  - 0x30=CYCLE_START, 0x31=CYCLE_STOP, 0xFA=SWITCH_FACTORY
- [x] 响应命令字: 查询用 0x1X (ZOOM=0x10, STATUS=0x11, SPEED=0x12, TYPE=0x13)
- [x] 到位帧: {0x02, 0x000A}
- [x] 回原点完成帧: {0x01, 0x000F}
- [x] 忙/无效拒绝: {0x03, 0x000E}
- [x] 堵转: 0xE1, 过流: 0xE2, 堵转计数: 0xE3
- [x] 握手回显: 所有有效帧收到后原样回显

## 改动文件清单
- `Tasks/Inc/task_config.hpp` — 重写 cmd/rsp_cmd/rsp namespace
- `Tasks/Inc/comm_task.hpp` — 添加 m_iSpeedKhz, m_iStallCount + accessor
- `Tasks/Src/comm_task.cpp` — 握手回显、查询 0x1X、BUSY {0x03,0x000E}、GET_STALL_CNT
- `Tasks/Src/motor_task.cpp` — 命令 ID 对齐、ZOOM/ARRIVED 双帧、ZOOM_INC/DEC、0xE1/0xE2
- `App/Inc/fram_storage.hpp` — 添加 motor_speed_khz 字段
- `Test/test_comm_task.cpp` — 26 个测试用例全面覆盖
- `Test/test_motor_task.cpp` — 24 个测试用例全面覆盖
