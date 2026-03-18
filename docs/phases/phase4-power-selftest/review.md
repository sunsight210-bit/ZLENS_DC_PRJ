# 阶段 4 回顾

## 完成情况
- SelfTest 非阻塞状态机（14 阶段，9 项自检），18 个 Mock 测试
- MotorTask 掉电处理：emergency_stop + FRAM emergency_save，3 个 Mock 测试
- MonitorTask 集成：首次/正常上电分支、20ms 轮询、10 个 Mock 测试
- 板上验证：首次上电（归零 + 自检 PASS）、正常上电（跳过自检 READY）

## 经验教训
- SELF_TEST 状态下必须喂看门狗，否则长时间自检（含归零）触发 IWDG 复位
- 跨任务回调需要显式连接：SelfTest 等待 homing_done 需 MonitorTask 从 g_rspQueue 转发
- SystemManager 状态转换表需提前考虑正常上电路径（SELF_TEST→READY）

## 遗留问题
- 掉电保护板上验证（需物理拔电）— 归入后续集成测试
- 编码器 A/B 反接补偿验证 — 归入后续集成测试
- 0x60 自检命令接口 — 预留，归入 CommTask 命令扩展

## 对后续阶段的影响
- MotorTask::init() 签名变更（新增 FramStorage*），后续阶段注意
- MONITOR_TASK_STACK 增至 384 words
- g_rspQueue 在自检期间被 MonitorTask 消费，CommTask 在此期间不应处理响应
