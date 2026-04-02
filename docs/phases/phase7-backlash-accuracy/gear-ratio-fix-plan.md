# 齿轮比修正与倍率表更新 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修正外部 4:1 齿轮比缺失导致的位置计算 4 倍偏差，更新倍率表为厂家实测数据。

**Architecture:** 修改 `ZoomTable` 中的 `TOTAL_RANGE` 常量（214648→858592），新增 `HOME_OFFSET`(2048) 和 `EXTERNAL_GEAR_RATIO`(4)，`angle_to_position()` 加统一偏移。下游常量（`HOMING_SETTLE_DISTANCE`、`ACC_START_POS`、`ACC_END_POS`、`BL_MEASURE_MID`）和 `self_test.cpp` 诊断输出同步更新。

**Tech Stack:** C++, GTest, CMake

**Spec:** `docs/phases/phase7-backlash-accuracy/gear-ratio-fix-spec.md`

---

### Task 1: 更新 ZoomTable 常量和 angle_to_position

**Files:**
- Modify: `App/Inc/zoom_table.hpp:15` — TOTAL_RANGE, 新增 HOME_OFFSET, EXTERNAL_GEAR_RATIO
- Modify: `App/Src/zoom_table.cpp:36-40` — angle_to_position 加 HOME_OFFSET

- [ ] **Step 1: 修改 zoom_table.hpp 常量**

在 `App/Inc/zoom_table.hpp` 中，将：
```cpp
static constexpr int32_t TOTAL_RANGE = 214648;
```
替换为：
```cpp
static constexpr int32_t EXTERNAL_GEAR_RATIO = 4;
static constexpr int32_t TOTAL_RANGE = 858592;   // 214648 × 4, 大齿轮满圈
static constexpr int32_t HOME_OFFSET = 2048;      // 归零后逻辑原点偏移
```

- [ ] **Step 2: 修改 angle_to_position 加统一偏移**

在 `App/Src/zoom_table.cpp:36-40` 中，将：
```cpp
int ZoomTable::angle_to_position(uint16_t angle_x100) const {
    return static_cast<int>(
        static_cast<int64_t>(angle_x100) * TOTAL_RANGE / 36000
    );
}
```
替换为：
```cpp
int ZoomTable::angle_to_position(uint16_t angle_x100) const {
    return HOME_OFFSET + static_cast<int>(
        static_cast<int64_t>(angle_x100) * TOTAL_RANGE / 36000
    );
}
```

---

### Task 2: 更新默认倍率表

**Files:**
- Modify: `App/Src/zoom_table.cpp:19-24` — kDefaultTable 完整替换

- [ ] **Step 1: 替换 kDefaultTable**

在 `App/Src/zoom_table.cpp` 中，将：
```cpp
static const ZOOM_ENTRY_S kDefaultTable[] = {
    { 6,     0}, { 8,  1800}, {10,  3600}, {12,  6000},
    {15,  9000}, {18, 12000}, {20, 15000}, {25, 18000},
    {30, 21000}, {35, 24000}, {40, 27000}, {50, 30000},
    {60, 33000}, {70, 34700}
};
```
替换为：
```cpp
static const ZOOM_ENTRY_S kDefaultTable[] = {
    { 6,     0}, {10,  7200}, {15, 11450}, {20, 14800},
    {25, 18150}, {30, 21100}, {35, 23550}, {40, 25750},
    {45, 27700}, {50, 29450}, {55, 31150}, {60, 32500},
    {65, 33800}, {70, 34700}
};
```

---

### Task 3: 更新 MotorTask 下游常量

**Files:**
- Modify: `Tasks/Inc/motor_task.hpp:37,47,52-53` — HOMING_SETTLE_DISTANCE, BL_MEASURE_MID, ACC_START_POS, ACC_END_POS

- [ ] **Step 1: 更新 HOMING_SETTLE_DISTANCE**

在 `Tasks/Inc/motor_task.hpp:37` 中，将：
```cpp
static constexpr int32_t HOMING_SETTLE_DISTANCE = 3000;
```
替换为：
```cpp
static constexpr int32_t HOMING_SETTLE_DISTANCE = 2048;  // = ZoomTable::HOME_OFFSET
```

- [ ] **Step 2: 更新 BL_MEASURE_MID**

在 `Tasks/Inc/motor_task.hpp:47` 中，将：
```cpp
static constexpr int32_t BL_MEASURE_MID = 107000;
```
替换为：
```cpp
static constexpr int32_t BL_MEASURE_MID = 434781;  // ≈2.5x 中点
```

- [ ] **Step 3: 更新 ACC_START_POS 和 ACC_END_POS**

在 `Tasks/Inc/motor_task.hpp:52-53` 中，将：
```cpp
static constexpr int32_t ACC_START_POS = 800;      // 0.6X position
static constexpr int32_t ACC_END_POS = 206821;     // ~7.0X position
```
替换为：
```cpp
static constexpr int32_t ACC_START_POS = 2048;      // 0.6x position = HOME_OFFSET
static constexpr int32_t ACC_END_POS = 829635;      // 7.0x position
```

---

### Task 4: 更新 self_test.cpp 诊断输出

**Files:**
- Modify: `App/Src/self_test.cpp:219-230`

- [ ] **Step 1: 更新 SOFT_LIMIT_OFFSET 和诊断字符串**

在 `App/Src/self_test.cpp:219` 中，将：
```cpp
constexpr int32_t SOFT_LIMIT_OFFSET = 800;  // from MotorTask
```
替换为：
```cpp
constexpr int32_t SOFT_LIMIT_OFFSET = ZoomTable::HOME_OFFSET;
```

在 `App/Src/self_test.cpp:230` 中，将：
```cpp
swo_printf(" Encoder         512PPR x4 = 2048/rev  ratio=104.77:1\n");
```
替换为：
```cpp
swo_printf(" Encoder         512PPR x4 = 2048/rev  ratio=104.77:1 x gear 4:1 = 419.08:1\n");
```

---

### Task 5: 更新 test_zoom_table.cpp

**Files:**
- Modify: `Test/test_zoom_table.cpp`

- [ ] **Step 1: 更新注释**

在 `Test/test_zoom_table.cpp:15` 中，将：
```cpp
// TOTAL_RANGE is now a fixed constant (214648)
```
替换为：
```cpp
// TOTAL_RANGE=858592 (214648×4), HOME_OFFSET=2048
```

- [ ] **Step 2: 更新 GetPosition_FirstEntry 测试**

在 `Test/test_zoom_table.cpp:35-37` 中，将：
```cpp
TEST_F(ZoomTableTest, GetPosition_FirstEntry) {
    EXPECT_EQ(table.get_position(6), 0);
}
```
替换为：
```cpp
TEST_F(ZoomTableTest, GetPosition_FirstEntry) {
    // 0.6x @ 0°: HOME_OFFSET + 0 = 2048
    EXPECT_EQ(table.get_position(6), ZoomTable::HOME_OFFSET);
}
```

- [ ] **Step 3: 更新 GetPosition_LastEntry 测试**

在 `Test/test_zoom_table.cpp:39-43` 中，将：
```cpp
TEST_F(ZoomTableTest, GetPosition_LastEntry) {
    int32_t pos = table.get_position(70);
    EXPECT_GT(pos, 190000);
    EXPECT_LT(pos, 210000);
}
```
替换为：
```cpp
TEST_F(ZoomTableTest, GetPosition_LastEntry) {
    // 7.0x @ 347°: HOME_OFFSET + 34700*858592/36000 = 2048+827587 = 829635
    int32_t pos = table.get_position(70);
    EXPECT_GT(pos, 828000);
    EXPECT_LT(pos, 831000);
}
```

- [ ] **Step 4: 更新 GetNearestZoom 测试**

在 `Test/test_zoom_table.cpp:49-53` 中，将：
```cpp
TEST_F(ZoomTableTest, GetNearestZoom) {
    EXPECT_EQ(table.get_nearest_zoom(0), 6);
    EXPECT_EQ(table.get_nearest_zoom(5000), 6);
    EXPECT_EQ(table.get_nearest_zoom(15000), 8);
}
```
替换为：
```cpp
TEST_F(ZoomTableTest, GetNearestZoom) {
    EXPECT_EQ(table.get_nearest_zoom(0), 6);
    EXPECT_EQ(table.get_nearest_zoom(2048), 6);
    // Midpoint between 0.6x(2048) and 1.0x(173766) ≈ 87907
    EXPECT_EQ(table.get_nearest_zoom(87907), 6);
    EXPECT_EQ(table.get_nearest_zoom(173766), 10);
}
```

- [ ] **Step 5: 更新 GetNextZoom 测试（倍率条目已变）**

倍率表现在条目为 6,10,15,20,25,30,35,40,45,50,55,60,65,70（无 8/12/18）。

在 `Test/test_zoom_table.cpp:55-58` 中，将：
```cpp
TEST_F(ZoomTableTest, GetNextZoom_Forward) {
    EXPECT_EQ(table.get_next_zoom(6, 1), 8);
    EXPECT_EQ(table.get_next_zoom(60, 1), 70);
}
```
替换为：
```cpp
TEST_F(ZoomTableTest, GetNextZoom_Forward) {
    EXPECT_EQ(table.get_next_zoom(6, 1), 10);
    EXPECT_EQ(table.get_next_zoom(60, 1), 65);
}
```

在 `Test/test_zoom_table.cpp:64-67` 中，将：
```cpp
TEST_F(ZoomTableTest, GetNextZoom_Backward) {
    EXPECT_EQ(table.get_next_zoom(10, -1), 8);
    EXPECT_EQ(table.get_next_zoom(6, -1), 6);
}
```
替换为：
```cpp
TEST_F(ZoomTableTest, GetNextZoom_Backward) {
    EXPECT_EQ(table.get_next_zoom(10, -1), 6);
    EXPECT_EQ(table.get_next_zoom(6, -1), 6);
}
```

在 `Test/test_zoom_table.cpp:69-71` 中，将：
```cpp
TEST_F(ZoomTableTest, GetNextZoom_MultiStep) {
    EXPECT_EQ(table.get_next_zoom(6, 3), 12);
}
```
替换为：
```cpp
TEST_F(ZoomTableTest, GetNextZoom_MultiStep) {
    // 6 → +3 → index 0+3 = index 3 → zoom_x10=20
    EXPECT_EQ(table.get_next_zoom(6, 3), 20);
}
```

- [ ] **Step 6: 更新 AngleToPositionConversion 测试**

在 `Test/test_zoom_table.cpp:81-87` 中，将：
```cpp
TEST_F(ZoomTableTest, AngleToPositionConversion) {
    table.erase_all();
    table.set_entry(25, 18000);
    int32_t pos = table.get_position(25);
    // 18000 * 214648 / 36000 = 107324
    EXPECT_NEAR(pos, 107324, 2);
}
```
替换为：
```cpp
TEST_F(ZoomTableTest, AngleToPositionConversion) {
    table.erase_all();
    table.set_entry(25, 18000);
    int32_t pos = table.get_position(25);
    // HOME_OFFSET + 18000 * 858592 / 36000 = 2048 + 429296 = 431344
    EXPECT_NEAR(pos, 431344, 2);
}
```

- [ ] **Step 7: 更新 TotalRange_IsFixedConstant 测试**

在 `Test/test_zoom_table.cpp:94-104` 中，将：
```cpp
TEST_F(ZoomTableTest, TotalRange_IsFixedConstant) {
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
替换为：
```cpp
TEST_F(ZoomTableTest, TotalRange_IsFixedConstant) {
    ZoomTable zt;
    zt.init();
    zt.load_defaults();
    // 0.6x at 0°: HOME_OFFSET + 0 = 2048
    EXPECT_EQ(zt.get_position(6), ZoomTable::HOME_OFFSET);
    // 7.0x at 347°: HOME_OFFSET + 34700*858592/36000 = 829635
    int32_t iPos70 = zt.get_position(70);
    EXPECT_GT(iPos70, 828000);
    EXPECT_LT(iPos70, 831000);
}
```

---

### Task 6: 更新 test_motor_ctrl.cpp

**Files:**
- Modify: `Test/test_motor_ctrl.cpp:246-254`

- [ ] **Step 1: 更新 Phase2_NearZero 测试**

在 `Test/test_motor_ctrl.cpp:246-255` 中，将：
```cpp
TEST_F(MotorCtrlTest, Phase2_NearZero_BacklashNotSkipped) {
    // 0.6x scenario: target=800, backlash=257
    // overshoot = 800 - 257 - 200 = 343 >= FINE_DEADZONE(100), compensation NOT skipped
    motor.set_backlash(257);
    motor.set_backlash_enabled(true);
    encoder.set_position(206821);  // 7.0x position
    motor.move_to(800);
    EXPECT_EQ(motor.get_target(), 343);  // overshoot target
    EXPECT_NE(motor.get_target(), 800);  // compensation was applied
}
```
替换为：
```cpp
TEST_F(MotorCtrlTest, Phase2_NearZero_BacklashNotSkipped) {
    // 0.6x scenario: target=2048, backlash=257
    // overshoot = 2048 - 257 - 200 = 1591 >= FINE_DEADZONE(100), compensation NOT skipped
    motor.set_backlash(257);
    motor.set_backlash_enabled(true);
    encoder.set_position(829635);  // 7.0x position
    motor.move_to(2048);
    EXPECT_EQ(motor.get_target(), 1591);  // overshoot target
    EXPECT_NE(motor.get_target(), 2048);  // compensation was applied
}
```

---

### Task 7: 更新 test_motor_task.cpp 中的注释

**Files:**
- Modify: `Test/test_motor_task.cpp:543-545`

- [ ] **Step 1: 更新 AccuracyTest 注释**

在 `Test/test_motor_task.cpp:543-545` 中，将：
```cpp
    // After homing, position=HOMING_SETTLE_DISTANCE(1500).
    // ACC_START_POS(800) is within DEADZONE(1000) of 1500,
    // so move_to returns IDLE immediately — motor already "at" start.
```
替换为：
```cpp
    // After homing, position=HOMING_SETTLE_DISTANCE(2048).
    // ACC_START_POS(2048) == HOMING_SETTLE_DISTANCE,
    // so move_to returns IDLE immediately — motor already at start.
```

注意：`test_motor_task.cpp` 中的其他位置均使用符号常量（`MotorTask::HOMING_SETTLE_DISTANCE`、`MotorTask::ACC_START_POS` 等），会自动跟随头文件更新，无需手动改值。

---

### Task 8: 运行测试并验证

- [ ] **Step 1: 编译测试**

```bash
cmake -B build/test -DBUILD_TESTING=ON && cmake --build build/test
```

Expected: 编译成功，无错误。

- [ ] **Step 2: 运行测试**

```bash
ctest --test-dir build/test --output-on-failure
```

Expected: 所有测试 PASS。

- [ ] **Step 3: 编译固件**

```bash
cmake -B build/fw -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake && cmake --build build/fw
```

Expected: 编译成功。

- [ ] **Step 4: 记录到 verify.md**

将测试结果和编译结果追加到 `docs/phases/phase7-backlash-accuracy/verify.md`。

- [ ] **Step 5: Commit**

```bash
git add App/Inc/zoom_table.hpp App/Src/zoom_table.cpp \
        Tasks/Inc/motor_task.hpp App/Src/self_test.cpp \
        Test/test_zoom_table.cpp Test/test_motor_ctrl.cpp Test/test_motor_task.cpp \
        docs/phases/phase7-backlash-accuracy/
git commit -m "fix: correct 4:1 external gear ratio and update zoom table to factory data"
```
