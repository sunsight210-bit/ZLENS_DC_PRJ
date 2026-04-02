# Phase 9: 参数存储重构 — 实施计划

**目标：** FRAM_PARAMS_S(60B) 精简为 FRAM_STATE_S(32B)，新建 FlashConfig 管理 FLASH_CONFIG_S(48B)，所有引用代码适配。

**设计规格：** `docs/delivery/storage-map.md`

---

## Task 1: FRAM 结构体精简

**文件：**
- 改: `App/Inc/fram_storage.hpp` — `FRAM_PARAMS_S` → `FRAM_STATE_S`(32B)
- 改: `App/Src/fram_storage.cpp` — BACKUP_ADDR 改为 `0x0020`，`emergency_save` 适配新偏移

**要点：**
- 删除: version, min/max_position, backlash_counts/valid, zero_reference, z_offset, lens_type, min/max_zoom_x10, stall_count, baseline_current, stall_threshold, encoder_overflow, encoder_compensated, speed_duty×3
- 保留: magic, current_position, current_zoom_x10, homing_done, position_valid, move_count, last_save_reason, reserved[17], crc16
- MAGIC bump 为新值（使旧 FRAM 数据失效）
- `static_assert(sizeof(FRAM_STATE_S) == 32)`

## Task 2: StorageTask 适配

**文件：**
- 改: `Tasks/Inc/storage_task.hpp` — 类型 `FRAM_PARAMS_S` → `FRAM_STATE_S`
- 改: `Tasks/Src/storage_task.cpp` — 去掉 backlash/speed 写入逻辑，去掉 version 赋值
- 改: `Tasks/Inc/task_config.hpp` — `SAVE_MESSAGE_S` 删除 backlash_counts/valid, speed_duty×3, speed_valid

## Task 3: MonitorTask 适配

**文件：**
- 改: `Tasks/Src/monitor_task.cpp` — 启动判断去掉 version 检查，类型改为 `FRAM_STATE_S`
- 改: `Tasks/Inc/monitor_task.hpp` — 如有类型引用则改

## Task 4: MotorTask 适配

**文件：**
- 改: `Tasks/Src/motor_task.cpp`:
  - `send_save()` 去掉 speed 字段
  - `motor_task_entry()` 中 speed 恢复逻辑改为从 FlashConfig 读取（Task 5 完成后）
  - speed 保存（SET_SPEED/SET_MIN_SPEED/SET_MAX_SPEED）改为直接调 FlashConfig::save()
- 改: `Tasks/Inc/motor_task.hpp` — 去掉 FramStorage 依赖（如不再需要）

## Task 5: 新建 FlashConfig 类

**文件：**
- 新建: `App/Inc/flash_config.hpp` — FLASH_CONFIG_S(48B) + FlashConfig 类
- 新建: `App/Src/flash_config.cpp` — Flash 擦页/写入/读取/CRC/load_defaults

**接口：**
```cpp
class FlashConfig {
public:
    static constexpr uint16_t MAGIC = 0x5A50;
    static constexpr uint32_t FLASH_ADDR = 0x0803F000;

    bool load(FLASH_CONFIG_S& stConfig);
    bool save(const FLASH_CONFIG_S& stConfig);
    static void load_defaults(FLASH_CONFIG_S& stConfig);
};
```

## Task 6: 系统集成

**文件：**
- 改: `App/Inc/app_instances.hpp` — 添加 `extern FlashConfig g_FlashConfig`
- 改: `App/Src/app_instances.cpp` — 初始化 FlashConfig，启动时 load→fallback defaults
- 改: `Tasks/Src/motor_task.cpp` — `motor_task_entry()` 从 g_FlashConfig 恢复 speed

## Task 7: 测试适配

**文件：**
- 改: `Test/test_fram_storage.cpp` — 适配 FRAM_STATE_S，更新 sizeof 断言为 32
- 改: `Test/test_storage_task.cpp` — 去掉 backlash/speed 测试用例
- 改: `Test/test_monitor_task.cpp` — 去掉 version 字段，适配 FRAM_STATE_S
- 改: `Test/test_motor_task.cpp` — 适配 SAVE_MESSAGE_S 精简
- 新建: `Test/test_flash_config.cpp` — FlashConfig 读写/CRC/defaults 测试
- 改: `Test/CMakeLists.txt` — 添加 test_flash_config
- 改: `Test/diag/*.cpp` — 全局 FramStorage 引用适配

## Task 8: 构建验证 & 提交

- 单元测试全部通过
- 固件编译通过
- 记录到 `docs/phases/phase9-storage-refactor/verify.md`
