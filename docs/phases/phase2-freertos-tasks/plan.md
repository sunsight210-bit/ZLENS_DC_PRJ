# 阶段 2：FreeRTOS Tasks 层

## 目标
基于阶段 1 的 App 核心模块，构建 FreeRTOS 任务框架层。4 个任务通过队列通信，实现串口命令接收→电机执行→存储保存→系统监控的完整数据流。

## 涉及文件

| 文件 | 操作 | 说明 |
|------|------|------|
| `Tasks/Inc/task_config.hpp` | 新增 | 任务配置：命令码、消息结构体、优先级、队列声明 |
| `Tasks/Inc/comm_task.hpp` / `Tasks/Src/comm_task.cpp` | 新增 | 通信任务：UART 收帧→协议解析→命令分发→响应回传 |
| `Tasks/Inc/motor_task.hpp` / `Tasks/Src/motor_task.cpp` | 新增 | 电机任务：命令执行、归零状态机、循环模式、堵转处理 |
| `Tasks/Inc/storage_task.hpp` / `Tasks/Src/storage_task.cpp` | 新增 | 存储任务：接收保存消息、周期保存、FRAM 写入 |
| `Tasks/Inc/monitor_task.hpp` / `Tasks/Src/monitor_task.cpp` | 新增 | 监控任务：自检、电压监控、看门狗喂狗 |
| `App/Inc/app_instances.hpp` / `App/Src/app_instances.cpp` | 新增 | 全局模块实例管理 |
| `App/Inc/swo_debug.hpp` / `App/Src/swo_debug.cpp` | 新增 | SWO 调试输出工具 |
| `Test/test_comm_task.cpp` | 新增 | CommTask 单元测试 |
| `Test/test_motor_task.cpp` | 新增 | MotorTask 单元测试 |
| `Test/test_storage_task.cpp` | 新增 | StorageTask 单元测试 |
| `Test/test_monitor_task.cpp` | 新增 | MonitorTask 单元测试 |
| `Test/mocks/mock_freertos.hpp/.cpp` | 修改 | 补充队列/任务 Mock 实现 |
| `Test/mocks/mock_hal.hpp/.cpp` | 修改 | 补充 IWDG/ADC Mock |

## 设计要点
- 4 个任务通过 3 个 FreeRTOS Queue 通信：`g_cmdQueue`（Comm→Motor）、`g_rspQueue`（Motor→Comm）、`g_saveQueue`（Motor→Storage）
- 任务优先级：Motor(4) > Comm(3) > Storage(2) > Monitor(1)
- MotorTask 含完整归零状态机（5 步：反向找限位→回退→找Z脉冲→正向找限位→回到软限位）
- MonitorTask 负责上电自检和运行时电压监控
- `g_bSpiEmergency` 全局标志用于掉电紧急保存触发

## 进度清单

- [x] 编写 task_config.hpp（命令码、消息结构体、队列声明）
- [x] 编写 CommTask
- [x] 编写 MotorTask（含归零状态机、循环模式）
- [x] 编写 StorageTask（含周期保存）
- [x] 编写 MonitorTask（含自检）
- [x] 编写 app_instances 和 swo_debug
- [x] 编写 4 个 Task 的单元测试
- [x] Mock 测试全部通过（115 用例全通过，其中 Task 层 43 用例）
- [x] 固件编译通过（text=49172, data=212, bss=17500）
- [x] 板上测试通过（SWO: BOOT + FreeRTOS scheduler running）
- [ ] git commit & push
