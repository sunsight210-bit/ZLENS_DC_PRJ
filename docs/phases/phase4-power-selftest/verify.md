# 阶段 4 验证记录

## Mock 测试

| 日期 | 测试数 | 通过 | 失败 | 备注 |
|------|--------|------|------|------|
| 2026-03-18 | 15 (全部) | 15 | 0 | 新增 test_self_test(18)、更新 test_motor_task(18+3)、test_monitor_task(10) |

## 固件编译

| 日期 | text | data | bss | 结果 |
|------|------|------|-----|------|
| 2026-03-18 | 54360 | 224 | 17592 | PASS |

## 板上测试

| 日期 | 测试项 | 结果 | SWO 输出 |
|------|--------|------|---------|
| 2026-03-18 | 首次上电（FRAM 无效） | PASS | Homing range=371681, Self-test passed |
| 2026-03-18 | 正常上电（FRAM 有效） | PASS | Normal boot: skipped self-test, READY, moved to zoom=0.6 |

## 发现的问题

| # | 问题 | 严重度 | 状态 | 解决方案 |
|---|------|--------|------|---------|
| 1 | SystemManager SELF_TEST→READY 转换不合法 | 中 | 已修复 | 添加 READY 到 SELF_TEST 合法转换列表 |
| 2 | SELF_TEST 期间未喂看门狗导致连续重启 | 高 | 已修复 | SELF_TEST case 中添加 feed_watchdog() |
| 3 | SelfTest homing_done 回调未连接 | 高 | 已修复 | MonitorTask 在 HOMING_WAIT 期间读 g_rspQueue |
