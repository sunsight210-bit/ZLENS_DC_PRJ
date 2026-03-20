# 齿轮间隙补偿与精度优化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 消除齿轮间隙导致的定位偏差和累计误差，简化回零流程，解耦自检与回零。

**Architecture:** MotorCtrl 层新增单向趋近补偿（APPROACHING 状态），MotorTask 重写回零状态机（4 阶段快慢堵转），SelfTest 精简为 4 项纯电气检测并与回零解耦，新增间隙测量和精度诊断流程。

**Tech Stack:** STM32F103RC, C++, FreeRTOS, GTest (host mock), CMake, SWO debug

**Spec:** `docs/superpowers/specs/2026-03-20-backlash-accuracy-design.md`

---

## 关键注意事项

### ZoomTable 的 total_range 依赖

当前 `ZoomTable::angle_to_position()` 依赖 `m_iTotalRange` 将角度转换为编码器位置。删除 FRAM 中的 `total_range` 后，需要改为固定常量。

计算：512 PPR × 4（四倍频）× 104.77（减速比）= **214,648 counts**（对应完整 360°）

行程 347° 对应 214,648 × 347/360 = 206,821 counts。

**关键：** `angle_to_position()` 的除数必须与 TOTAL_RANGE 的定义匹配：
- TOTAL_RANGE = 214,648（代表 360°）→ 除以 36000
- TOTAL_RANGE = 206,821（代表 347°）→ 除以 34700

本计划采用 **TOTAL_RANGE = 214,648 + 除以 36000**，与现有公式一致，无需改动 `angle_to_position()` 的除数。

### 测试文件对应关系

| 产品文件 | 测试文件 |
|----------|----------|
| `App/Inc/motor_ctrl.hpp` / `App/Src/motor_ctrl.cpp` | `Test/test_motor_ctrl.cpp` |
| `App/Inc/fram_storage.hpp` / `App/Src/fram_storage.cpp` | `Test/test_fram_storage.cpp` |
| `App/Inc/zoom_table.hpp` / `App/Src/zoom_table.cpp` | `Test/test_zoom_table.cpp` |
| `App/Inc/self_test.hpp` / `App/Src/self_test.cpp` | `Test/test_self_test.cpp` |
| `Tasks/Inc/motor_task.hpp` / `Tasks/Src/motor_task.cpp` | `Test/test_motor_task.cpp` |
| `Tasks/Inc/monitor_task.hpp` / `Tasks/Src/monitor_task.cpp` | `Test/test_monitor_task.cpp` |
| `Tasks/Src/storage_task.cpp` | `Test/test_storage_task.cpp` |

### 构建命令

```bash
# Host 测试
cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && cd build/test && ctest --output-on-failure

# ARM 固件
cmake -B build/fw -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake && cmake --build build/fw
```

---

## Task 1: FRAM v2 schema — 新增 backlash 字段，删除废弃字段

**Files:**
- Modify: `App/Inc/fram_storage.hpp:14-39` — 更新 `FRAM_PARAMS_S` 结构体
- Modify: `App/Src/fram_storage.cpp` — 更新 `emergency_save()` 如有 total_range 引用
- Modify: `Tasks/Src/storage_task.cpp:69-73` — `write_params()` version 改为 2
- Modify: `Tasks/Src/storage_task.cpp:87-90` — `storage_task_entry` 删除 `set_total_range` 调用
- Test: `Test/test_fram_storage.cpp`

- [ ] **Step 1: 写失败测试 — FRAM v2 结构体大小和字段**

在 `Test/test_fram_storage.cpp` 中新增测试：

```cpp
TEST_F(FramStorageTest, V2_StructSize_Is60Bytes) {
    EXPECT_EQ(sizeof(FRAM_PARAMS_S), 60u);
}

TEST_F(FramStorageTest, V2_BacklashFields_DefaultZero) {
    FRAM_PARAMS_S stParams{};
    EXPECT_EQ(stParams.backlash_counts, 0);
    EXPECT_EQ(stParams.backlash_valid, 0);
}
```

- [ ] **Step 2: 运行测试确认失败**

```bash
cmake --build build/test && cd build/test && ctest -R test_fram_storage --output-on-failure
```

预期：编译失败，`backlash_counts` 和 `backlash_valid` 未定义。

- [ ] **Step 3: 修改 FRAM_PARAMS_S**

在 `App/Inc/fram_storage.hpp` 中：

```cpp
#pragma pack(push, 1)
struct FRAM_PARAMS_S {
    uint16_t magic_number;       // 0x5A3C
    uint8_t  version;            // 2 (was 1)
    int32_t  current_position;
    uint16_t current_zoom_x10;
    int32_t  min_position;
    int32_t  max_position;
    int16_t  backlash_counts;    // was: total_range (int32→int16, saves 2 bytes)
    uint8_t  backlash_valid;     // was: soft_limit_offset (uint16→uint8, saves 1 byte)
    uint8_t  zero_reference;
    int32_t  z_offset;
    uint8_t  lens_type;
    uint16_t min_zoom_x10;
    uint16_t max_zoom_x10;
    uint16_t stall_count;
    uint16_t baseline_current;
    uint16_t stall_threshold;
    uint8_t  homing_done;
    uint8_t  position_valid;
    int16_t  encoder_overflow;
    uint8_t  last_save_reason;
    uint8_t  encoder_compensated;
    uint16_t motor_speed_khz;
    uint8_t  reserved[14];       // adjusted to maintain 60 bytes
    uint16_t crc16;
};
#pragma pack(pop)

static_assert(sizeof(FRAM_PARAMS_S) == 60, "FRAM_PARAMS_S must be 60 bytes");
```

注意：`total_range`(int32=4B) + `soft_limit_offset`(uint16=2B) = 6B 被替换为 `backlash_counts`(int16=2B) + `backlash_valid`(uint8=1B) = 3B，多出 3B 加到 `reserved`（11→14）。

- [ ] **Step 4: 修改 storage_task.cpp**

在 `Tasks/Src/storage_task.cpp` 的 `write_params()` 中：

```cpp
void StorageTask::write_params() {
    m_stParams.magic_number = FramStorage::MAGIC;
    m_stParams.version = 2;  // was 1
    m_stParams.crc16 = FramStorage::calc_crc(m_stParams);
    m_pFram->save_params(m_stParams);
}
```

在 `storage_task_entry()` 中删除 `g_ZoomTable.set_total_range(stParams.total_range);` 这一行。

- [ ] **Step 5: 修复所有引用 total_range / soft_limit_offset 的编译错误**

搜索并修复所有 `.total_range` 和 `.soft_limit_offset` 的引用：
- `App/Src/self_test.cpp` — FRAM_SAVE 阶段的 `stParams.total_range` 赋值（后续 Task 5 会删除此阶段，此步先注释或设 0）
- `Tasks/Src/storage_task.cpp` — 已在 Step 4 处理

- [ ] **Step 6: 运行测试确认通过**

```bash
cmake --build build/test && cd build/test && ctest --output-on-failure
```

预期：所有 FRAM 测试通过。注意：引用了 `total_range` / `soft_limit_offset` 的其他测试可能也会编译失败，需要同步修复。

- [ ] **Step 7: Commit**

```bash
git add App/Inc/fram_storage.hpp App/Src/fram_storage.cpp Tasks/Src/storage_task.cpp Test/test_fram_storage.cpp
git commit -m "refactor: FRAM v2 schema — add backlash fields, remove total_range/soft_limit"
```

---

## Task 2: ZoomTable — total_range 改为固定常量

**Files:**
- Modify: `App/Inc/zoom_table.hpp:33-34` — 删除 `set_total_range()`/`get_total_range()`，加 `constexpr`
- Modify: `App/Src/zoom_table.cpp:26-29` — `init()` 不再清零 `m_iTotalRange`
- Modify: `Tasks/Src/motor_task.cpp:440-451` — 删除 `clamp_to_soft_limits()`
- Test: `Test/test_zoom_table.cpp`

- [ ] **Step 1: 写失败测试 — 固定 total_range 常量**

在 `Test/test_zoom_table.cpp` 中新增测试：

```cpp
TEST_F(ZoomTableTest, TotalRange_IsFixedConstant) {
    // 不需要 set_total_range()，ZoomTable 应始终使用固定值
    ZoomTable zt;
    zt.init();
    zt.load_defaults();
    // 0.6x at 0° should map to position 0
    EXPECT_EQ(zt.get_position(6), 0);
    // 7.0x at 347° should map to ~206821 (34700 * 214648 / 36000)
    int32_t iPos70 = zt.get_position(70);
    EXPECT_GT(iPos70, 205000);
    EXPECT_LT(iPos70, 210000);
}
```

- [ ] **Step 2: 运行测试确认失败**

```bash
cmake --build build/test && cd build/test && ctest -R test_zoom_table --output-on-failure
```

预期：测试失败，因为当前 `init()` 后 `m_iTotalRange = 0`，所有位置返回 0。

- [ ] **Step 3: 修改 ZoomTable**

在 `App/Inc/zoom_table.hpp` 中：

```cpp
class ZoomTable {
public:
    // 512 PPR × 4 × 104.77 = 214648 (full 360°)
    static constexpr int32_t TOTAL_RANGE = 214648;
    // ...
    // 删除 set_total_range() 和 get_total_range()
```

在 `App/Src/zoom_table.cpp` 中：

```cpp
void ZoomTable::init() {
    m_iCount = 0;
    std::memset(m_aEntries, 0, sizeof(m_aEntries));
}

int ZoomTable::angle_to_position(uint16_t angle_x100) const {
    return static_cast<int>(
        static_cast<int64_t>(angle_x100) * TOTAL_RANGE / 36000
    );
}
```

删除 `m_iTotalRange` 成员变量。

- [ ] **Step 4: 修复所有 set_total_range / get_total_range 的编译错误**

搜索并修复：
- `Tasks/Src/motor_task.cpp` — `process_homing()` 中 `m_pZoom->set_total_range(m_iTotalRange)` 删除
- `Tasks/Src/motor_task.cpp` — `clamp_to_soft_limits()` 中 `m_pZoom->get_total_range()` — 整个函数将在 Task 4 删除，此步先注释
- `Tasks/Src/monitor_task.cpp` — `m_pZoom->set_total_range(0)` 和 `m_pZoom->get_total_range()` 删除
- `Test/test_motor_task.cpp` — 所有 `zoom.set_total_range(100000)` 调用删除
- `Test/test_zoom_table.cpp` — 所有 `zoom.set_total_range()` 调用删除
- `Test/test_monitor_task.cpp` — 相关调用删除

- [ ] **Step 5: 运行测试确认通过**

```bash
cmake --build build/test && cd build/test && ctest --output-on-failure
```

预期：所有 ZoomTable 测试通过。MotorTask 测试中涉及 soft limit 的测试可能需要调整期望值（不再有 soft limit clamping）。

- [ ] **Step 6: Commit**

```bash
git add App/Inc/zoom_table.hpp App/Src/zoom_table.cpp Tasks/Src/motor_task.cpp Tasks/Src/monitor_task.cpp Test/
git commit -m "refactor: ZoomTable uses fixed TOTAL_RANGE constant, remove set_total_range"
```

---

## Task 3: MotorCtrl — 单向趋近补偿 + speed limit API

**Files:**
- Modify: `App/Inc/motor_ctrl.hpp` — 新增 `APPROACHING` 状态、`set_backlash()`、`set_backlash_enabled()`、`set_speed_limit()`
- Modify: `App/Src/motor_ctrl.cpp` — 实现单向趋近逻辑
- Test: `Test/test_motor_ctrl.cpp`

- [ ] **Step 1: 写失败测试 — speed limit API**

在 `Test/test_motor_ctrl.cpp` 中新增：

```cpp
TEST_F(MotorCtrlTest, SetSpeedLimit_LimitsMaxSpeed) {
    motor.set_speed_limit(1000);
    motor.move_to(100000);
    // Run many updates to reach max speed
    for (int i = 0; i < 500; ++i) motor.update();
    EXPECT_LE(motor.get_current_speed(), 1000u);
}

TEST_F(MotorCtrlTest, SetSpeedLimit_RestoreDefault) {
    motor.set_speed_limit(1000);
    motor.set_speed_limit(MotorCtrl::MAX_SPEED);
    motor.move_to(100000);
    for (int i = 0; i < 500; ++i) motor.update();
    EXPECT_GT(motor.get_current_speed(), 1000u);
}
```

- [ ] **Step 2: 运行测试确认失败**

```bash
cmake --build build/test && cd build/test && ctest -R test_motor_ctrl --output-on-failure
```

预期：编译失败，`set_speed_limit` 未定义。

- [ ] **Step 3: 实现 set_speed_limit**

在 `App/Inc/motor_ctrl.hpp` 中，`set_max_speed()` 已存在，`set_speed_limit()` 只是语义更清晰的别名：

```cpp
void set_speed_limit(uint16_t iLimit) { m_iMaxSpeed = iLimit; }
```

- [ ] **Step 4: 运行测试确认通过**

```bash
cmake --build build/test && cd build/test && ctest -R test_motor_ctrl --output-on-failure
```

- [ ] **Step 5: 写失败测试 — backlash 补偿（正向运动不补偿）**

```cpp
TEST_F(MotorCtrlTest, Backlash_ForwardMove_NoCompensation) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(1000);
    motor.move_to(5000);
    // Forward: no compensation, target should be 5000
    EXPECT_EQ(motor.get_target(), 5000);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::FORWARD);
}
```

- [ ] **Step 6: 运行测试确认失败**

预期：编译失败，`set_backlash` 和 `set_backlash_enabled` 未定义。

- [ ] **Step 7: 实现 backlash API（声明 + 空实现）**

在 `App/Inc/motor_ctrl.hpp` 中新增：

```cpp
enum class MOTOR_STATE_E { IDLE, ACCELERATING, CONSTANT, DECELERATING, BRAKING, STALLED, APPROACHING };

// ...public:
    void set_backlash(int16_t iCounts) { m_iBacklash = iCounts; }
    void set_backlash_enabled(bool bEnabled) { m_bBacklashEnabled = bEnabled; }
    int16_t get_backlash() const { return m_iBacklash; }
    bool is_backlash_enabled() const { return m_bBacklashEnabled; }

// ...private:
    int16_t m_iBacklash = 200;       // default 200 counts
    bool m_bBacklashEnabled = false;  // disabled by default
    int32_t m_iFinalTarget = 0;       // real target for APPROACHING
```

- [ ] **Step 8: 运行测试确认通过**

- [ ] **Step 9: 写失败测试 — backlash 补偿（反向运动触发两段）**

```cpp
TEST_F(MotorCtrlTest, Backlash_ReverseMove_TwoPhase) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);
    // Phase 1: should overshoot past target
    // overshoot = target - backlash - backlash/2 = 5000 - 200 - 100 = 4700
    EXPECT_EQ(motor.get_target(), 4700);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::REVERSE);
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::ACCELERATING);
}

TEST_F(MotorCtrlTest, Backlash_ReverseMove_ApproachPhase) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);

    // Simulate arrival at overshoot position
    encoder.set_position(4700);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::APPROACHING) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::APPROACHING);

    // After one more update, should start forward to final target
    motor.update();
    EXPECT_EQ(motor.get_target(), 5000);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::FORWARD);
}

TEST_F(MotorCtrlTest, Backlash_ReverseMove_CompletesAtFinalTarget) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);

    // Phase 1: arrive at overshoot
    encoder.set_position(4700);
    for (int i = 0; i < 500; ++i) motor.update();

    // Phase 2: arrive at final target
    encoder.set_position(5000);
    for (int i = 0; i < 500; ++i) {
        motor.update();
        if (motor.get_state() == MOTOR_STATE_E::IDLE) break;
    }
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}

TEST_F(MotorCtrlTest, Backlash_Disabled_NoCompensation) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(false);
    encoder.set_position(10000);
    motor.move_to(5000);
    // No compensation: target is 5000 directly
    EXPECT_EQ(motor.get_target(), 5000);
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::REVERSE);
}
```

- [ ] **Step 10: 运行测试确认失败**

- [ ] **Step 11: 实现 move_to() 的补偿逻辑**

在 `App/Src/motor_ctrl.cpp` 中修改 `move_to()`：

```cpp
void MotorCtrl::move_to(int32_t target) {
    int32_t pos = m_pEncoder->get_position();
    int32_t diff = target - pos;

    if (std::abs(diff) <= DEADZONE) {
        m_eState = MOTOR_STATE_E::IDLE;
        return;
    }

    if (m_bBacklashEnabled && diff < 0) {
        // Reverse move: two-phase approach
        m_iFinalTarget = target;
        int32_t iOvershoot = target - m_iBacklash - m_iBacklash / 2;
        m_iTarget = iOvershoot;
    } else {
        m_iTarget = target;
        m_iFinalTarget = target;
    }

    m_eDirection = (m_iTarget - pos > 0) ? DIRECTION_E::FORWARD : DIRECTION_E::REVERSE;
    m_iCurrentSpeed = m_iMinSpeed;
    m_eState = MOTOR_STATE_E::ACCELERATING;
    set_pwm(m_eDirection, m_iCurrentSpeed);
}
```

在 `update()` 中，到达判定后检查是否需要启动第二段：

```cpp
void MotorCtrl::update() {
    if (m_eState == MOTOR_STATE_E::IDLE || m_eState == MOTOR_STATE_E::STALLED) return;

    // Handle APPROACHING: start phase 2
    if (m_eState == MOTOR_STATE_E::APPROACHING) {
        m_iTarget = m_iFinalTarget;
        m_eDirection = DIRECTION_E::FORWARD;
        m_iCurrentSpeed = m_iMinSpeed;
        m_eState = MOTOR_STATE_E::ACCELERATING;
        set_pwm(m_eDirection, m_iCurrentSpeed);
        return;
    }

    int32_t pos = m_pEncoder->get_position();
    int32_t diff = m_iTarget - pos;
    int32_t remaining = std::abs(diff);

    bool bOvershot = (m_eDirection == DIRECTION_E::FORWARD && diff < -DEADZONE) ||
                     (m_eDirection == DIRECTION_E::REVERSE && diff > DEADZONE);
    if (remaining <= DEADZONE || bOvershot) {
        brake();
        m_iCurrentSpeed = 0;
        // Check if this was phase 1 of backlash compensation
        if (m_bBacklashEnabled && m_iTarget != m_iFinalTarget) {
            m_eState = MOTOR_STATE_E::APPROACHING;
            return;
        }
        m_eState = MOTOR_STATE_E::IDLE;
        return;
    }

    // ... rest of switch unchanged ...
}
```

- [ ] **Step 12: 运行测试确认通过**

```bash
cmake --build build/test && cd build/test && ctest -R test_motor_ctrl --output-on-failure
```

- [ ] **Step 13: 写测试 — APPROACHING 状态下 emergency_stop**

```cpp
TEST_F(MotorCtrlTest, Backlash_EmergencyStop_DuringApproach_CancelsPhase2) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    encoder.set_position(10000);
    motor.move_to(5000);

    // Arrive at overshoot → APPROACHING
    encoder.set_position(4700);
    for (int i = 0; i < 500; ++i) motor.update();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::APPROACHING);

    motor.emergency_stop();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
    // Should NOT start phase 2
    motor.update();
    EXPECT_EQ(motor.get_state(), MOTOR_STATE_E::IDLE);
}
```

- [ ] **Step 14: 运行测试确认通过**

`emergency_stop()` 已经设置状态为 IDLE 并清零速度，APPROACHING 后的 update 不会再启动 phase 2 因为状态已是 IDLE。应直接通过。

- [ ] **Step 15: Commit**

```bash
git add App/Inc/motor_ctrl.hpp App/Src/motor_ctrl.cpp Test/test_motor_ctrl.cpp
git commit -m "feat: MotorCtrl backlash compensation with unidirectional approach"
```

---

## Task 4: 回零流程简化 — 4 阶段快慢堵转

**Files:**
- Modify: `Tasks/Inc/motor_task.hpp:16-21` — 替换 TASK_STATE_E 枚举
- Modify: `Tasks/Src/motor_task.cpp` — 重写 `start_homing()`、`process_homing()`、`handle_stall()`
- Test: `Test/test_motor_task.cpp`

- [ ] **Step 1: 写失败测试 — 新的回零状态机**

在 `Test/test_motor_task.cpp` 中，替换所有回零相关测试：

```cpp
// ============================================================
// New Homing: 4 stages (FAST → RETRACT → SLOW → SETTLE)
// ============================================================

TEST_F(MotorTaskTest, HomingFast_ReverseAtMediumSpeed) {
    task.start_homing();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_FAST);
    // Speed limit should be ~50% of MAX_SPEED
    EXPECT_LE(motor.get_current_speed(), MotorCtrl::MAX_SPEED / 2 + MotorCtrl::ACCEL_STEP);
}

TEST_F(MotorTaskTest, HomingFast_StallTriggersRetract) {
    task.start_homing();
    trigger_stall();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_RETRACT);
    // Retract target should be +4096 from position 0
    EXPECT_EQ(motor.get_target(), MotorTask::HOMING_RETRACT_DISTANCE);
}

TEST_F(MotorTaskTest, HomingRetract_DoneTriggersSlow) {
    task.start_homing();
    trigger_stall(); // FAST → RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_SLOW);
    // Should be moving in reverse
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::REVERSE);
}

TEST_F(MotorTaskTest, HomingSlow_StallTriggersSettle) {
    task.start_homing();
    trigger_stall(); // FAST → RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();
    // Now in SLOW
    trigger_stall(); // SLOW → SETTLE
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::HOMING_SETTLE);
    // Position should be reset to 0 at negative limit
    // Settle target should be HOMING_SETTLE_DISTANCE (800)
    EXPECT_EQ(motor.get_target(), MotorTask::HOMING_SETTLE_DISTANCE);
}

TEST_F(MotorTaskTest, HomingSettle_Done_HomingComplete) {
    task.start_homing();
    trigger_stall(); // FAST → RETRACT
    encoder.set_position(MotorTask::HOMING_RETRACT_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();
    trigger_stall(); // SLOW → SETTLE
    encoder.set_position(MotorTask::HOMING_SETTLE_DISTANCE);
    iAdcCurrent = 0;
    for (int i = 0; i < 10; ++i) task.run_once();

    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
    EXPECT_TRUE(task.is_homing_done());

    auto rsps = drain_rsp();
    ASSERT_GE(rsps.size(), 2u);
    EXPECT_EQ(rsps[0].cmd, rsp_cmd::ZOOM);
    EXPECT_EQ(rsps[1].cmd, rsp_cmd::HOMING_DONE);
}

TEST_F(MotorTaskTest, Homing_BacklashDisabledDuringHoming) {
    motor.set_backlash(200);
    motor.set_backlash_enabled(true);
    task.start_homing();
    // Backlash should be disabled during homing
    EXPECT_FALSE(motor.is_backlash_enabled());
}
```

- [ ] **Step 2: 运行测试确认失败**

预期：编译失败，新的状态枚举和常量未定义。

- [ ] **Step 3: 更新 motor_task.hpp**

```cpp
class MotorTask {
public:
    enum class TASK_STATE_E {
        IDLE, MOVING,
        HOMING_FAST, HOMING_RETRACT, HOMING_SLOW, HOMING_SETTLE,
        CYCLING,
        BACKLASH_MEASURE, ACCURACY_TEST  // for Task 6/7
    };

    // ...
    static constexpr int32_t HOMING_RETRACT_DISTANCE = 4096;  // was 8000
    static constexpr int32_t HOMING_SETTLE_DISTANCE = 800;
    static constexpr int32_t HOMING_FAR_DISTANCE = 1000000;
    static constexpr uint16_t HOMING_FAST_SPEED = MotorCtrl::MAX_SPEED / 2;
    static constexpr uint16_t HOMING_SLOW_SPEED = MotorCtrl::MIN_SPEED;
```

删除：`SOFT_LIMIT_OFFSET`、`get_total_range()`、`get_z_offset()`、`m_iTotalRange`、`m_iZOffset`。
删除：`clamp_to_soft_limits()` 方法声明和 `HOMING_FIND_Z`、`HOMING_FORWARD`、`HOMING_TO_SOFT_MIN` 枚举值。

- [ ] **Step 4: 实现新的回零逻辑**

在 `Tasks/Src/motor_task.cpp` 中：

```cpp
void MotorTask::start_homing(bool bFullDiag) {
    m_bHomingDone = false;
    m_bFullDiagnostics = bFullDiag;
    // Disable backlash during homing
    m_pMotor->set_backlash_enabled(false);
    m_pMotor->set_speed_limit(HOMING_FAST_SPEED);
    m_eTaskState = TASK_STATE_E::HOMING_FAST;
    m_pStall->set_direction(StallDetect::Direction::REVERSE);
    m_pStall->start_motor();
    m_pMotor->move_to(-HOMING_FAR_DISTANCE);
}

void MotorTask::process_homing() {
    switch (m_eTaskState) {
    case TASK_STATE_E::HOMING_RETRACT:
        if (m_pMotor->get_state() == MOTOR_STATE_E::IDLE) {
            // Retract done → start slow reverse
            m_pMotor->set_speed_limit(HOMING_SLOW_SPEED);
            m_eTaskState = TASK_STATE_E::HOMING_SLOW;
            m_pStall->set_direction(StallDetect::Direction::REVERSE);
            m_pStall->start_motor();
            m_pMotor->move_to(-HOMING_FAR_DISTANCE);
        }
        break;
    case TASK_STATE_E::HOMING_SETTLE:
        if (m_pMotor->get_state() == MOTOR_STATE_E::IDLE) {
            // Settle done → homing complete
            m_pMotor->set_speed_limit(MotorCtrl::MAX_SPEED); // restore
            m_pMotor->set_backlash_enabled(true); // re-enable
            m_bHomingDone = true;
            m_pStall->reset();
            int32_t iPos = m_pEncoder->get_position();
            uint16_t iZoom = m_pZoom->get_nearest_zoom(iPos);
            send_response(rsp_cmd::ZOOM, iZoom);
            send_response(rsp_cmd::HOMING_DONE, rsp::HOMING_DONE_PARAM);
            send_save(save_reason::ARRIVED);
            m_eTaskState = TASK_STATE_E::IDLE;
            m_pSm->transition_to(SYSTEM_STATE_E::READY);
        }
        break;
    default:
        break;
    }
}

void MotorTask::handle_stall() {
    switch (m_eTaskState) {
    case TASK_STATE_E::HOMING_FAST:
        // Coarse negative limit found
        m_pMotor->emergency_stop();
        m_pEncoder->set_position(0);
        m_pStall->reset();
        m_pMotor->set_speed_limit(HOMING_SLOW_SPEED);
        m_eTaskState = TASK_STATE_E::HOMING_RETRACT;
        m_pStall->set_direction(StallDetect::Direction::FORWARD);
        m_pStall->start_motor();
        m_pMotor->move_to(HOMING_RETRACT_DISTANCE);
        break;
    case TASK_STATE_E::HOMING_SLOW:
        // Precise negative limit found
        m_pMotor->emergency_stop();
        m_pEncoder->set_position(0);
        m_pStall->reset();
        m_eTaskState = TASK_STATE_E::HOMING_SETTLE;
        m_pStall->set_direction(StallDetect::Direction::FORWARD);
        m_pStall->start_motor();
        m_pMotor->move_to(HOMING_SETTLE_DISTANCE);
        break;
    case TASK_STATE_E::HOMING_RETRACT:
        // Stall during retract — unusual, skip to slow
        m_pMotor->emergency_stop();
        m_pStall->reset();
        m_eTaskState = TASK_STATE_E::HOMING_SLOW;
        m_pStall->set_direction(StallDetect::Direction::REVERSE);
        m_pStall->start_motor();
        m_pMotor->move_to(-HOMING_FAR_DISTANCE);
        break;
    default:
        // Unexpected stall during normal operation
        m_pMotor->emergency_stop();
        m_pStall->reset();
        m_eTaskState = TASK_STATE_E::IDLE;
        m_pSm->transition_to(SYSTEM_STATE_E::READY);
        send_response(rsp_cmd::STALL_STOP, 0);
        send_save(save_reason::STALL);
        break;
    }
}
```

同时删除 `clamp_to_soft_limits()` 函数，并从 `dispatch_command()` 中所有 `iTarget = clamp_to_soft_limits(iTarget)` 调用中移除该调用。

更新 `run_once()` 中的 homing 状态分支：

```cpp
case TASK_STATE_E::HOMING_FAST:
case TASK_STATE_E::HOMING_RETRACT:
case TASK_STATE_E::HOMING_SLOW:
case TASK_STATE_E::HOMING_SETTLE:
    process_homing();
    break;
```

更新 `handle_overcurrent()` 中的 homing 状态列表：

```cpp
if (m_eTaskState == TASK_STATE_E::HOMING_FAST ||
    m_eTaskState == TASK_STATE_E::HOMING_SLOW ||
    m_eTaskState == TASK_STATE_E::HOMING_RETRACT ||
    m_eTaskState == TASK_STATE_E::HOMING_SETTLE) {
    handle_stall();
    return;
}
```

- [ ] **Step 5: 删除旧回零测试，运行新测试**

删除 `Test/test_motor_task.cpp` 中旧的回零测试（`HomingComplete_TwoFrames`、`HomingReverse_UntilStall`、`HomingRetract_4096Counts`、`HomingFindZ_RecordsOffset`、`HomingForward_RecordsRange`、`SoftLimit_ClampsTarget`），替换为 Step 1 中的新测试。

```bash
cmake --build build/test && cd build/test && ctest --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add Tasks/Inc/motor_task.hpp Tasks/Src/motor_task.cpp Test/test_motor_task.cpp
git commit -m "feat: simplify homing to 4-stage fast/slow with backlash disable"
```

---

## Task 5: 自检与回零解耦 — SelfTest 精简 + MonitorTask 编排

**Files:**
- Modify: `App/Inc/self_test.hpp` — 精简枚举和结构体
- Modify: `App/Src/self_test.cpp` — 删除 HOMING/RANGE/LIMITS/FRAM_SAVE 阶段
- Modify: `Tasks/Inc/monitor_task.hpp` — 更新 boot 流程
- Modify: `Tasks/Src/monitor_task.cpp` — 重写 run_once()
- Test: `Test/test_self_test.cpp`
- Test: `Test/test_monitor_task.cpp`

- [ ] **Step 1: 写失败测试 — SelfTest 只有 4 项**

在 `Test/test_self_test.cpp` 中新增/替换测试：

```cpp
TEST_F(SelfTestTest, ItemCount_Is4) {
    EXPECT_EQ(static_cast<uint8_t>(SELF_TEST_ITEM_E::COUNT), 4u);
}

TEST_F(SelfTestTest, AllPass_When4ItemsPass) {
    // Setup: valid voltage, low baseline, good FRAM, correct encoder
    // Run through all phases
    st.start();
    // ... drive through VOLTAGE, BASELINE, FRAM_RW, ENCODER_DIR
    // After ENCODER_DIR completes, SelfTest should be DONE
    // (no HOMING, no RANGE_CHECK, no LIMITS_CHECK, no FRAM_SAVE)
}
```

- [ ] **Step 2: 运行测试确认失败**

- [ ] **Step 3: 精简 self_test.hpp**

```cpp
enum class SELF_TEST_ITEM_E : uint8_t {
    VOLTAGE = 0,
    BASELINE,
    FRAM_RW,
    ENCODER_DIR,
    COUNT  // must be last — now 4
};

struct SELF_TEST_RESULT_S {
    bool aPass[static_cast<uint8_t>(SELF_TEST_ITEM_E::COUNT)];
    uint16_t iMeasuredVoltage;
    uint16_t iMeasuredBaseline;
    bool bEncoderCompensated;
    bool bAllPassed;
};

enum class SELF_TEST_PHASE_E : uint8_t {
    IDLE,
    VOLTAGE,
    BASELINE,
    FRAM_RW,
    ENCODER_DIR_START,
    ENCODER_DIR_WAIT,
    ENCODER_DIR_REVERSE,
    ENCODER_DIR_REVERSE_WAIT,
    DONE
};
```

删除：`HOMING_START`、`HOMING_WAIT`、`RANGE_CHECK`、`LIMITS_CHECK`、`FRAM_SAVE` 阶段。
删除：`iTotalRange` 字段、`notify_homing_done()` 方法、`m_bHomingNotified`/`m_bHomingSuccess`/`m_iHomingTotalRange`/`m_iHomingStartTick` 成员。

- [ ] **Step 4: 精简 self_test.cpp**

`step()` 函数中：
- `ENCODER_DIR_WAIT` 成功后直接 `m_ePhase = SELF_TEST_PHASE_E::DONE; finalize(); return true;`
- `ENCODER_DIR_REVERSE_WAIT` 成功后同上
- 删除 `HOMING_START`、`HOMING_WAIT`、`RANGE_CHECK`、`LIMITS_CHECK`、`FRAM_SAVE` 的 case 分支

`print_report()` 中：
- 删除 HOMING、RANGE_CHECK、LIMITS_CHECK、FRAM_SAVE 的打印
- 删除 Mechanical 段中依赖 `m_stResult.iTotalRange` 的内容（使用 `ZoomTable::TOTAL_RANGE` 常量替代）

- [ ] **Step 5: 更新 MonitorTask — 上电流程和 0x60 编排**

在 `Tasks/Src/monitor_task.cpp` 中：

上电流程（`run_once()` 的 `SELF_TEST` 状态）：

```cpp
case SYSTEM_STATE_E::SELF_TEST:
    feed_watchdog();
    if (!m_bBootDecided) {
        FRAM_PARAMS_S stParams{};
        bool bValid = m_pFram->load_params(stParams);
        if (bValid && FramStorage::check_magic(stParams) &&
            FramStorage::verify_crc(stParams) && stParams.version == 2) {
            if (stParams.position_valid == 0xFF && stParams.homing_done == 1) {
                m_bNormalBoot = true;  // FRAM valid, position valid → skip all
            } else if (stParams.homing_done == 1) {
                m_bNeedHomingOnly = true;  // FRAM valid but position lost → homing only
            }
            // else: FRAM invalid → full self-test + homing
        }
        m_bBootDecided = true;
    }

    if (m_bNormalBoot) {
        if (!m_bNormalBootStarted) start_normal_boot();
        else run_normal_boot();
    } else if (m_bNeedHomingOnly) {
        // Skip self-test, trigger homing directly
        if (!m_bHomingTriggered) {
            m_bHomingTriggered = true;
            CMD_MESSAGE_S stCmd = {cmd::HOMING, 0};
            xQueueSend(g_cmdQueue, &stCmd, 0);
            m_pSm->transition_to(SYSTEM_STATE_E::HOMING);
        }
    } else {
        // Full self-test then homing
        // ... run 4-item SelfTest ...
        // On pass: trigger homing
        // On fail: ERROR_STATE
    }
    break;
```

新增成员变量：`bool m_bNeedHomingOnly = false;`、`bool m_bHomingTriggered = false;`

0x60 命令处理：当收到 0x60 → 运行 4 项自检 → 自检通过后触发回零（通过 cmd queue 发 HOMING 命令）

- [ ] **Step 6: 修复编译错误并更新测试**

更新 `Test/test_self_test.cpp` — 删除依赖 HOMING/RANGE/LIMITS 的测试。
更新 `Test/test_monitor_task.cpp` — 适配新的上电流程。
删除 `MonitorTask::notify_homing_done()` 方法。

- [ ] **Step 7: 运行全部测试**

```bash
cmake --build build/test && cd build/test && ctest --output-on-failure
```

- [ ] **Step 8: Commit**

```bash
git add App/Inc/self_test.hpp App/Src/self_test.cpp Tasks/Inc/monitor_task.hpp Tasks/Src/monitor_task.cpp Test/test_self_test.cpp Test/test_monitor_task.cpp
git commit -m "refactor: decouple self-test from homing, simplify to 4 electrical checks"
```

---

## Task 6: 间隙测量流程

**Files:**
- Modify: `Tasks/Inc/motor_task.hpp` — 新增 BACKLASH_MEASURE 子状态
- Modify: `Tasks/Src/motor_task.cpp` — 实现测量逻辑
- Test: `Test/test_motor_task.cpp`

- [ ] **Step 1: 写失败测试 — 间隙测量**

```cpp
TEST_F(MotorTaskTest, BacklashMeasure_MovesToMiddle) {
    // After homing, start backlash measurement
    complete_homing(); // helper that drives through all 4 homing stages
    task.start_backlash_measure();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::BACKLASH_MEASURE);
    // Should be moving forward to mid-range
    EXPECT_EQ(motor.get_direction(), DIRECTION_E::FORWARD);
}

TEST_F(MotorTaskTest, BacklashMeasure_StoresResult) {
    complete_homing();
    task.start_backlash_measure();
    // Simulate: drive to middle, do 3 forward-reverse cycles
    // ... simulate measurement sequence ...
    // After completion, backlash should be set on motor
    EXPECT_GT(motor.get_backlash(), 0);
}
```

- [ ] **Step 2: 运行测试确认失败**

- [ ] **Step 3: 实现间隙测量**

在 `Tasks/Inc/motor_task.hpp` 中新增成员：

```cpp
// Backlash measurement
enum class BL_MEASURE_PHASE_E : uint8_t {
    MOVE_TO_MID, REVERSE, FORWARD
};
BL_MEASURE_PHASE_E m_eBLPhase = BL_MEASURE_PHASE_E::MOVE_TO_MID;
int32_t m_iBLRefPos = 0;
int32_t m_aBLSamples[3] = {};
uint8_t m_iBLSampleIdx = 0;
static constexpr int32_t BL_MEASURE_MID = 107000;
static constexpr int32_t BL_REVERSE_DIST = 4096;
bool m_bFullDiagnostics = false;  // true when triggered by 0x60

void start_homing(bool bFullDiag = false);  // overload, sets m_bFullDiagnostics
void start_backlash_measure();
void process_backlash_measure();
```

在 `Tasks/Src/motor_task.cpp` 中实现：

```cpp
void MotorTask::start_backlash_measure() {
    m_pMotor->set_backlash_enabled(false);  // raw moves
    m_pMotor->set_speed_limit(HOMING_SLOW_SPEED);
    m_eBLPhase = BL_MEASURE_PHASE_E::MOVE_TO_MID;
    m_iBLSampleIdx = 0;
    m_eTaskState = TASK_STATE_E::BACKLASH_MEASURE;
    m_pStall->set_direction(StallDetect::Direction::FORWARD);
    m_pStall->start_motor();
    m_pMotor->move_to(BL_MEASURE_MID);
}

void MotorTask::process_backlash_measure() {
    if (m_pMotor->get_state() != MOTOR_STATE_E::IDLE) return;

    switch (m_eBLPhase) {
    case BL_MEASURE_PHASE_E::MOVE_TO_MID:
        m_iBLRefPos = m_pEncoder->get_position();
        m_eBLPhase = BL_MEASURE_PHASE_E::REVERSE;
        m_pStall->set_direction(StallDetect::Direction::REVERSE);
        m_pStall->start_motor();
        m_pMotor->move_to(m_iBLRefPos - BL_REVERSE_DIST);
        break;
    case BL_MEASURE_PHASE_E::REVERSE:
        m_eBLPhase = BL_MEASURE_PHASE_E::FORWARD;
        m_pStall->set_direction(StallDetect::Direction::FORWARD);
        m_pStall->start_motor();
        m_pMotor->move_to(m_iBLRefPos);
        break;
    case BL_MEASURE_PHASE_E::FORWARD: {
        int32_t iActual = m_pEncoder->get_position();
        int32_t iError = iActual - m_iBLRefPos;
        if (iError < 0) iError = -iError;
        m_aBLSamples[m_iBLSampleIdx++] = iError;
        if (m_iBLSampleIdx < 3) {
            // Next cycle
            m_eBLPhase = BL_MEASURE_PHASE_E::REVERSE;
            m_pStall->set_direction(StallDetect::Direction::REVERSE);
            m_pStall->start_motor();
            m_pMotor->move_to(m_iBLRefPos - BL_REVERSE_DIST);
        } else {
            // Calculate average, subtract DEADZONE
            int32_t iSum = m_aBLSamples[0] + m_aBLSamples[1] + m_aBLSamples[2];
            int32_t iAvg = iSum / 3;
            int16_t iBacklash = static_cast<int16_t>(
                iAvg > MotorCtrl::DEADZONE ? iAvg - MotorCtrl::DEADZONE : 0);
            m_pMotor->set_backlash(iBacklash);
            m_pMotor->set_speed_limit(MotorCtrl::MAX_SPEED);
            m_pStall->reset();
            // Save to FRAM via save queue with special reason
            send_save(save_reason::ARRIVED);
            m_eTaskState = TASK_STATE_E::IDLE;
            m_pSm->transition_to(SYSTEM_STATE_E::READY);
        }
        break;
    }
    default:
        break;
    }
}
```

在 `run_once()` 的 switch 中添加：

```cpp
case TASK_STATE_E::BACKLASH_MEASURE:
    process_backlash_measure();
    break;
```

- [ ] **Step 4: 运行测试**

```bash
cmake --build build/test && cd build/test && ctest --output-on-failure
```

- [ ] **Step 5: Commit**

```bash
git add Tasks/Inc/motor_task.hpp Tasks/Src/motor_task.cpp Test/test_motor_task.cpp
git commit -m "feat: add backlash measurement procedure in motor task"
```

---

## Task 7: 精度诊断测试

**Files:**
- Modify: `Tasks/Inc/motor_task.hpp` — 新增 ACCURACY_TEST 子状态
- Modify: `Tasks/Src/motor_task.cpp` — 实现往返测试和 SWO 报告
- Test: `Test/test_motor_task.cpp`

- [ ] **Step 1: 写失败测试 — 精度诊断**

```cpp
TEST_F(MotorTaskTest, AccuracyTest_RoundTrips) {
    complete_homing();
    task.start_accuracy_test();
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::ACCURACY_TEST);
}

TEST_F(MotorTaskTest, AccuracyTest_CompletesAfterRoundTrips) {
    complete_homing();
    task.start_accuracy_test();
    // Simulate 3 round trips
    // ... drive to max, then back to min, repeat ...
    // Should return to IDLE when done
    EXPECT_EQ(task.get_state(), MotorTask::TASK_STATE_E::IDLE);
}
```

- [ ] **Step 2: 运行测试确认失败**

- [ ] **Step 3: 实现精度诊断**

在 `Tasks/Inc/motor_task.hpp` 中新增成员：

```cpp
// Accuracy test
enum class ACC_TEST_PHASE_E : uint8_t {
    MOVE_TO_START, MOVE_TO_END, MOVE_TO_START_RETURN, DONE
};
ACC_TEST_PHASE_E m_eAccPhase = ACC_TEST_PHASE_E::MOVE_TO_START;
uint8_t m_iAccTripCount = 0;
int32_t m_iAccRefPos = 0;
int32_t m_aAccErrors[10] = {};  // max 5 trips × 2 directions
uint8_t m_iAccErrorIdx = 0;
static constexpr uint8_t ACC_NUM_TRIPS = 3;
static constexpr int32_t ACC_START_POS = 800;   // 0.6X
static constexpr int32_t ACC_END_POS = 206821;  // ~7.0X position from zoom table

void start_accuracy_test();
void process_accuracy_test();
void print_accuracy_report();
```

在 `Tasks/Src/motor_task.cpp` 中实现：

```cpp
void MotorTask::start_accuracy_test() {
    m_pMotor->set_backlash_enabled(true);  // test WITH compensation
    m_eAccPhase = ACC_TEST_PHASE_E::MOVE_TO_START;
    m_iAccTripCount = 0;
    m_iAccErrorIdx = 0;
    m_eTaskState = TASK_STATE_E::ACCURACY_TEST;
    m_pStall->set_direction(StallDetect::Direction::FORWARD);
    m_pStall->start_motor();
    m_pMotor->move_to(ACC_START_POS);
}

void MotorTask::process_accuracy_test() {
    // Wait for full completion (including APPROACHING phase 2)
    if (m_pMotor->get_state() != MOTOR_STATE_E::IDLE) return;

    switch (m_eAccPhase) {
    case ACC_TEST_PHASE_E::MOVE_TO_START:
        m_iAccRefPos = m_pEncoder->get_position();
        m_eAccPhase = ACC_TEST_PHASE_E::MOVE_TO_END;
        m_pStall->set_direction(StallDetect::Direction::FORWARD);
        m_pStall->start_motor();
        m_pMotor->move_to(ACC_END_POS);
        break;
    case ACC_TEST_PHASE_E::MOVE_TO_END: {
        int32_t iError = m_pEncoder->get_position() - ACC_END_POS;
        m_aAccErrors[m_iAccErrorIdx++] = iError;
        m_eAccPhase = ACC_TEST_PHASE_E::MOVE_TO_START_RETURN;
        m_pStall->set_direction(StallDetect::Direction::REVERSE);
        m_pStall->start_motor();
        m_pMotor->move_to(ACC_START_POS);
        break;
    }
    case ACC_TEST_PHASE_E::MOVE_TO_START_RETURN: {
        int32_t iError = m_pEncoder->get_position() - ACC_START_POS;
        m_aAccErrors[m_iAccErrorIdx++] = iError;
        m_iAccTripCount++;
        if (m_iAccTripCount < ACC_NUM_TRIPS) {
            m_eAccPhase = ACC_TEST_PHASE_E::MOVE_TO_END;
            m_pStall->set_direction(StallDetect::Direction::FORWARD);
            m_pStall->start_motor();
            m_pMotor->move_to(ACC_END_POS);
        } else {
            // Calculate cumulative drift
            int32_t iDrift = m_pEncoder->get_position() - m_iAccRefPos;
            print_accuracy_report();
            m_pStall->reset();
            m_eTaskState = TASK_STATE_E::IDLE;
            m_pSm->transition_to(SYSTEM_STATE_E::READY);
        }
        break;
    }
    default:
        break;
    }
}

void MotorTask::print_accuracy_report() {
#ifndef BUILD_TESTING
    int32_t iDrift = m_pEncoder->get_position() - m_iAccRefPos;
    int32_t iMaxErr = 0;
    bool bPass = true;
    swo_printf("\n============ ACCURACY TEST REPORT ============\n");
    swo_printf(" Backlash: %d counts\n", m_pMotor->get_backlash());
    for (uint8_t i = 0; i < m_iAccErrorIdx; ++i) {
        int32_t e = m_aAccErrors[i];
        if (std::abs(e) > std::abs(iMaxErr)) iMaxErr = e;
        if (std::abs(e) > MotorCtrl::DEADZONE) bPass = false;
        swo_printf(" Trip %u %s: error=%ld counts\n",
                   i / 2 + 1, (i % 2 == 0) ? "FWD" : "REV",
                   static_cast<long>(e));
    }
    if (std::abs(iDrift) > MotorCtrl::DEADZONE) bPass = false;
    swo_printf(" Cumulative drift: %ld counts\n", static_cast<long>(iDrift));
    swo_printf(" Max error: %ld counts\n", static_cast<long>(iMaxErr));
    swo_printf(" RESULT: %s\n", bPass ? "PASS" : "FAIL");
    swo_printf("===============================================\n");
#endif
}
```

- [ ] **Step 4: 运行测试**

```bash
cmake --build build/test && cd build/test && ctest --output-on-failure
```

- [ ] **Step 5: Commit**

```bash
git add Tasks/Inc/motor_task.hpp Tasks/Src/motor_task.cpp Test/test_motor_task.cpp
git commit -m "feat: add accuracy test with round-trip diagnostics and SWO report"
```

---

## Task 8: 0x60 命令编排 — 串联自检→回零→间隙测量→精度诊断

**Files:**
- Modify: `Tasks/Src/monitor_task.cpp` — 0x60 触发完整流程
- Modify: `Tasks/Inc/monitor_task.hpp` — 新增流程状态
- Modify: `Tasks/Src/motor_task.cpp` — 回零完成后自动触发间隙测量→精度诊断
- Test: `Test/test_monitor_task.cpp`

- [ ] **Step 1: 写测试 — 0x60 触发完整链路**

```cpp
TEST_F(MonitorTaskTest, SelfTestCmd_TriggersFullSequence) {
    // Simulate 0x60 request
    g_bUartSelfTestReq = true;
    task.run_once();
    // Should start 4-item self-test
    EXPECT_EQ(sm.get_state(), SYSTEM_STATE_E::SELF_TEST);
}
```

- [ ] **Step 2: 实现 0x60 编排**

在 `Tasks/Src/monitor_task.cpp` 中，0x60 处理保持现有结构但适配新的 SelfTest（4 项），自检通过后发 HOMING 命令。

在 `Tasks/Src/motor_task.cpp` 中，回零完成后检查是否需要运行间隙测量和精度诊断（通过 flag 或命令参数区分普通回零和自检触发的回零）：

```cpp
// In process_homing() SETTLE done:
if (m_bFullDiagnostics) {
    // Self-test triggered: run backlash measurement next
    start_backlash_measure();
} else {
    // Normal homing: done
    m_eTaskState = TASK_STATE_E::IDLE;
    m_pSm->transition_to(SYSTEM_STATE_E::READY);
}
```

间隙测量完成后自动触发精度诊断：

```cpp
// In process_backlash_measure() completion:
if (m_bFullDiagnostics) {
    start_accuracy_test();
} else {
    m_eTaskState = TASK_STATE_E::IDLE;
}
```

- [ ] **Step 3: 运行全部测试**

```bash
cmake --build build/test && cd build/test && ctest --output-on-failure
```

- [ ] **Step 4: Commit**

```bash
git add Tasks/ Test/
git commit -m "feat: wire 0x60 self-test → homing → backlash measure → accuracy test"
```

---

## Task 9: FRAM backlash 持久化 — StorageTask 集成

**Files:**
- Modify: `Tasks/Src/storage_task.cpp` — 保存/恢复 backlash 值
- Modify: `Tasks/Src/motor_task.cpp` — 启动时从 FRAM 加载 backlash
- Test: `Test/test_storage_task.cpp`

- [ ] **Step 1: 写测试 — backlash 保存和恢复**

```cpp
TEST_F(StorageTaskTest, SaveParams_IncludesBacklash) {
    FRAM_PARAMS_S stParams{};
    stParams.backlash_counts = 150;
    stParams.backlash_valid = 0xFF;
    // ... save and load ...
    EXPECT_EQ(loaded.backlash_counts, 150);
    EXPECT_EQ(loaded.backlash_valid, 0xFF);
}
```

- [ ] **Step 2: 实现 backlash 持久化**

在 `storage_task_entry()` 的恢复逻辑中：

```cpp
if (task.restore_params(stParams)) {
    g_Encoder.set_position(stParams.current_position);
    if (stParams.backlash_valid == 0xFF) {
        g_Motor.set_backlash(stParams.backlash_counts);
    }
    g_Motor.set_backlash_enabled(true);
}
```

在 `motor_task.cpp` 间隙测量完成后，需要将 backlash 值保存到 FRAM。

**方案：扩展 `SAVE_MESSAGE_S`**

在 `Tasks/Inc/task_config.hpp` 中：

```cpp
struct SAVE_MESSAGE_S {
    int32_t position;
    uint16_t zoom_x10;
    uint8_t reason;
    int16_t backlash_counts;    // 新增：0 表示不更新
    uint8_t backlash_valid;     // 新增：0 表示不更新，0xFF 表示已标定
};
```

在 `Tasks/Src/storage_task.cpp` 的 `handle_save()` 中：

```cpp
void StorageTask::handle_save(const SAVE_MESSAGE_S& stMsg) {
    if (g_bSpiEmergency) return;
    m_stParams.current_position = stMsg.position;
    m_stParams.current_zoom_x10 = stMsg.zoom_x10;
    m_stParams.last_save_reason = stMsg.reason;
    // Update backlash if provided
    if (stMsg.backlash_valid == 0xFF) {
        m_stParams.backlash_counts = stMsg.backlash_counts;
        m_stParams.backlash_valid = 0xFF;
    }
    write_params();
    m_iLastSavedPosition = stMsg.position;
    m_iLastSaveTick = HAL_GetTick();
}
```

在 `motor_task.cpp` 间隙测量完成后发送带 backlash 的保存：

```cpp
SAVE_MESSAGE_S stSave = {
    m_pEncoder->get_position(),
    m_pZoom->get_nearest_zoom(m_pEncoder->get_position()),
    save_reason::ARRIVED,
    iBacklash,  // backlash_counts
    0xFF        // backlash_valid = calibrated
};
xQueueSend(m_saveQueue, &stSave, 0);
```

- [ ] **Step 3: 运行测试**

```bash
cmake --build build/test && cd build/test && ctest --output-on-failure
```

- [ ] **Step 4: Commit**

```bash
git add Tasks/ Test/
git commit -m "feat: persist backlash value in FRAM, restore on boot"
```

---

## Task 10: 编译固件 + 全量测试

**Files:** 无新增，确认全部编译通过

- [ ] **Step 1: Host 测试全量运行**

```bash
cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test && cd build/test && ctest --output-on-failure
```

预期：所有测试通过，覆盖率 >80%。

- [ ] **Step 2: ARM 固件编译**

```bash
cmake -B build/fw -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake && cmake --build build/fw
```

预期：编译成功，无 warning。

- [ ] **Step 3: 检查覆盖率**

```bash
cd build/test && gcov -r CMakeFiles/ | grep -A 1 "motor_ctrl\|motor_task\|self_test\|fram_storage"
```

预期：核心模块覆盖率 >80%。

- [ ] **Step 4: Commit（如有修复）**

```bash
git add App/ Tasks/ Test/ && git commit -m "fix: resolve build issues for firmware compilation"
```

---

## Task 依赖关系

```
Task 1 (FRAM v2) ──────┐
                        ├── Task 3 (MotorCtrl backlash) ── Task 4 (Homing) ── Task 5 (SelfTest) ── Task 8 (0x60 编排) ── Task 9 (FRAM 持久化) ── Task 10 (Build)
Task 2 (ZoomTable) ─────┘                                       │
                                                                 ├── Task 6 (间隙测量) ──┤
                                                                 └── Task 7 (精度诊断) ──┘
```

Task 1 和 Task 2 可并行。Task 3 依赖 1+2。Task 4 依赖 3。**Task 5 依赖 Task 4**（因为 MonitorTask 引用了 MotorTask 的新状态枚举）。Task 6/7 依赖 4。Task 8 依赖 5+6+7。Task 9 依赖 8。Task 10 最后。
