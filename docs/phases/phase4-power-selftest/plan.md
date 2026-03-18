# 阶段 4：掉电保护 & 自检

## 目标
1. **自检模块** — 独立 SelfTest 类，首次上电完整自检（电压/基线/FRAM/编码器方向/归零/量程/限位/FRAM保存），后续可通过 0x60 命令调用
2. **掉电保护** — 20ms 轮询检测低压 → 紧急停车 + FRAM 保存位置
3. **正常上电恢复** — FRAM 有效时跳过自检，恢复位置，移动到最近倍率

## 涉及文件

| 文件 | 操作 | 说明 |
|------|------|------|
| App/Inc/self_test.hpp | 新建 | SelfTest 状态机、结果结构体 |
| App/Src/self_test.cpp | 新建 | 自检流程实现（14 个状态） |
| Test/test_self_test.cpp | 新建 | 18 个测试用例 |
| App/Inc/fram_storage.hpp | 修改 | 添加 encoder_compensated 字段、test_rw() |
| App/Src/fram_storage.cpp | 修改 | 实现 test_rw() |
| Tasks/Inc/task_config.hpp | 修改 | 添加 POWER_DOWN/SELF_TEST 代码，栈增至 384 |
| Tasks/Inc/motor_task.hpp | 修改 | 添加 FramStorage*、handle_power_down() |
| Tasks/Src/motor_task.cpp | 修改 | 掉电通知处理、紧急保存 |
| Tasks/Inc/monitor_task.hpp | 修改 | SelfTest 集成、正常上电路径 |
| Tasks/Src/monitor_task.cpp | 修改 | 首次/正常上电分支、20ms 轮询 |
| App/Src/system_manager.cpp | 修改 | 允许 SELF_TEST → READY 转换 |
| Test/mocks/stm32f1xx_hal.h | 修改 | 添加 CCER 字段、TIM_CCER_CC1P |
| Test/test_motor_task.cpp | 修改 | 更新 init + 3 个掉电测试 |
| Test/test_monitor_task.cpp | 修改 | 10 个测试覆盖正常/首次上电 |
| Test/CMakeLists.txt | 修改 | 添加 test_self_test 目标 |

## 设计要点
- SelfTest 纯 App 层，无 FreeRTOS 依赖，非阻塞状态机
- 编码器方向自动补偿：翻转 TIM8 CC1P 位，FRAM 记录 compensated
- 掉电处理通过 xTaskNotify 位 0x02 通知 MotorTask
- 正常上电路径：FRAM valid → 跳过自检 → 恢复位置 → READY

## 进度清单

- [x] FRAM 扩展 + test_rw()
- [x] SelfTest 类（18 测试通过）
- [x] MotorTask 掉电处理（3 测试通过）
- [x] MonitorTask 集成（10 测试通过）
- [x] 固件编译通过
- [ ] 板上测试
