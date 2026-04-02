# Phase 7: 齿轮间隙补偿与精度优化

## 目标

消除齿轮间隙导致的定位偏差和累计误差，简化回零流程，解耦自检与回零。

## 设计文档

- 设计规格：`docs/superpowers/specs/2026-03-20-backlash-accuracy-design.md`
- 实现计划：`docs/superpowers/plans/2026-03-20-backlash-accuracy.md`

## 任务概览

| Task | 内容 | 涉及文件 |
|------|------|----------|
| 1 | FRAM v2 schema | fram_storage.hpp, storage_task.cpp |
| 2 | ZoomTable 固定常量 | zoom_table.hpp/cpp |
| 3 | MotorCtrl 间隙补偿 | motor_ctrl.hpp/cpp |
| 4 | 回零流程简化 | motor_task.hpp/cpp |
| 5 | 自检与回零解耦 | self_test.hpp/cpp, monitor_task.hpp/cpp |
| 6 | 间隙测量流程 | motor_task.hpp/cpp |
| 7 | 精度诊断测试 | motor_task.hpp/cpp |
| 8 | 0x60 命令编排 | monitor_task.cpp, motor_task.cpp |
| 9 | FRAM backlash 持久化 | storage_task.cpp, task_config.hpp |
| 10 | 编译固件 + 全量测试 | 全部 |

## 依赖关系

```
Task 1 + Task 2 (可并行) → Task 3 → Task 4 → Task 5 → Task 8 → Task 9 → Task 10
                                        ├── Task 6 ──┘
                                        └── Task 7 ──┘
```
