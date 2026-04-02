# 阶段 1：App 核心模块

## 目标
实现 ZLENS_DC 电动变倍镜头控制系统的所有底层功能模块，每个模块独立可测，为上层 FreeRTOS 任务提供完整的硬件抽象和业务逻辑接口。

## 涉及文件

| 文件 | 操作 | 说明 |
|------|------|------|
| `App/Inc/crc16.hpp` / `App/Src/crc16.cpp` | 新增 | CRC16/MODBUS 校验算法 |
| `App/Inc/comm_protocol.hpp` / `App/Src/comm_protocol.cpp` | 新增 | 通信协议帧解析，工作/工厂模式 |
| `App/Inc/encoder.hpp` / `App/Src/encoder.cpp` | 新增 | 编码器模块，32位溢出扩展，Z脉冲 |
| `App/Inc/zoom_table.hpp` / `App/Src/zoom_table.cpp` | 新增 | 可配置变倍表，角度驱动，Flash存储 |
| `App/Inc/fram_storage.hpp` / `App/Src/fram_storage.cpp` | 新增 | FRAM 双备份存储，CRC 校验，紧急保存 |
| `App/Inc/stall_detect.hpp` / `App/Src/stall_detect.cpp` | 新增 | 堵转检测，消隐窗口，过流，编码器辅助 |
| `App/Inc/motor_ctrl.hpp` / `App/Src/motor_ctrl.cpp` | 新增 | 电机控制，梯形加减速，DAC 电流限制 |
| `App/Inc/power_monitor.hpp` / `App/Src/power_monitor.cpp` | 新增 | 电源监控，ADC 看门狗阈值检测 |
| `App/Inc/system_manager.hpp` / `App/Src/system_manager.cpp` | 新增 | 系统状态管理器，状态机转换验证 |
| `Test/test_crc16.cpp` | 新增 | CRC16 单元测试 |
| `Test/test_comm_protocol.cpp` | 新增 | 通信协议单元测试 |
| `Test/test_encoder.cpp` | 新增 | 编码器单元测试 |
| `Test/test_zoom_table.cpp` | 新增 | 变倍表单元测试 |
| `Test/test_fram_storage.cpp` | 新增 | FRAM 存储单元测试 |
| `Test/test_stall_detect.cpp` | 新增 | 堵转检测单元测试 |
| `Test/test_motor_ctrl.cpp` | 新增 | 电机控制单元测试 |
| `Test/test_power_monitor.cpp` | 新增 | 电源监控单元测试 |
| `Test/test_system_manager.cpp` | 新增 | 系统状态管理器单元测试 |
| `cmake/stm32f103rc.cmake` | 新增 | ARM 交叉编译工具链 |
| `Test/CMakeLists.txt` | 新增 | GTest 测试构建配置 |
| `Test/mocks/` | 新增 | HAL/FreeRTOS Mock 层 |

## 设计要点
- 每个模块通过构造函数注入 HAL 依赖，便于 Mock 测试
- 所有模块遵循匈牙利命名规范（见 CLAUDE.md）
- 编码器使用 32 位扩展避免 16 位 TIM 溢出
- FRAM 双备份 + CRC 保障存储可靠性
- 梯形加减速通过定时器中断驱动

## 进度清单

- [x] 编写 CRC16 模块 + 测试
- [x] 编写 CommProtocol 模块 + 测试
- [x] 编写 Encoder 模块 + 测试
- [x] 编写 ZoomTable 模块 + 测试
- [x] 编写 FramStorage 模块 + 测试
- [x] 编写 StallDetect 模块 + 测试
- [x] 编写 MotorCtrl 模块 + 测试
- [x] 编写 PowerMonitor 模块 + 测试
- [x] 编写 SystemManager 模块 + 测试
- [x] Mock 测试全部通过
- [x] 固件编译通过
- [x] 板上测试通过
- [x] git commit & push
