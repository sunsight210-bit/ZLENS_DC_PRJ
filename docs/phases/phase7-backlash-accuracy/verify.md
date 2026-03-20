# Phase 7 验证记录

## Mock 测试

| Task | 测试数量 | 通过 | 覆盖率 | 日期 |
|------|----------|------|--------|------|
| 1 FRAM v2 | 15/15 | PASS | — | 2026-03-20 |
| 2 ZoomTable | | | | |
| 3 MotorCtrl | | | | |
| 4 Homing | | | | |
| 5 SelfTest | | | | |
| 6 间隙测量 | | | | |
| 7 精度诊断 | | | | |
| 8 0x60 编排 | | | | |
| 9 FRAM 持久化 | | | | |
| 10 全量测试 | | | | |

## 固件编译

- [ ] `cmake -B build/fw -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake && cmake --build build/fw`
- 编译结果：
- Warning 数量：

## 板上测试

- [ ] 烧录成功
- [ ] SWO 自检报告通过
- [ ] 回零流程正常（4 阶段）
- [ ] 间隙测量完成
- [ ] 精度诊断通过
- [ ] 正常定位（各 zoom 档位）
