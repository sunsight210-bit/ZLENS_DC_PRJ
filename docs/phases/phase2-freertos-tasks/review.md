# 阶段 2 回顾

## 完成情况
全部 4 个 FreeRTOS Task 按计划完成：CommTask、MotorTask、StorageTask、MonitorTask。含 task_config.hpp 消息定义、app_instances 全局实例管理、swo_debug SWO 输出工具。

- Mock 测试：43 用例全通过（含阶段 1 共 115 用例）
- 固件编译：49172 bytes text（含完整 App + Tasks + FreeRTOS）
- 板上验证：SWO 输出 `[BOOT]` + `[PASS] FreeRTOS scheduler running`，5 秒内无 HardFault

## 经验教训
- main.c 中 `app_init()` 必须在 `osKernelInitialize()` 之后、`osKernelStart()` 之前调用（队列创建需要 FreeRTOS 内核已初始化）
- SWO ITM 初始化必须放在 main.c 的 USER CODE 2 区域（HAL 初始化之后），不能依赖 FreeRTOS 任务来初始化
- OpenOCD 的 tpiu 配置中 `reset halt` 后再 `resume` 确保 SWO 捕获从启动开始
- ITM 4 字节帧格式（header=0x03 + char + 0x00x3）需要专门的解码脚本

## 遗留问题
- flash_and_test.sh 中的 SWO 解析尚未处理 ITM 帧解码（grep 直接匹配因 NUL 字节失败）
- MonitorTask 的自检目前只检查 ADC 电压基线，未检查 FRAM 连通性（首次启动无有效数据）
- 看门狗 IWDG 已初始化但 MonitorTask 的喂狗周期（100ms）需在实际运行中确认是否足够

## 对后续阶段的影响
- 阶段 3 的 MotorTask 核心逻辑（归零、循环模式）框架已就绪，需要实际电机硬件验证
- 阶段 4 的掉电保护依赖 `g_bSpiEmergency` 标志和 StorageTask 的紧急保存流程
- SWO 调试输出基础设施已就绪，后续阶段可直接使用 `swo_printf` 输出测试结果
