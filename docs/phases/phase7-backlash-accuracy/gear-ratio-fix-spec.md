# 齿轮比修正与倍率表更新 — 设计规格

## 问题描述

当前 `TOTAL_RANGE = 214,648` 仅覆盖电机输出轴（小齿轮）一圈 360°，
未计入外部 4:1 齿轮比（小齿轮带动大齿轮）。倍率表中的角度是大齿轮角度，
导致 `angle_to_position()` 计算结果偏小 4 倍。

## 物理参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 编码器 PPR | 512 | 电机轴 |
| 四倍频 | ×4 | 2048 counts/转（电机轴） |
| 电机减速箱 | 104.77:1 | 行星齿轮箱 |
| 外部齿轮比 | 4:1 | 小齿轮:大齿轮 |
| 总减速比 | 419.08:1 | 104.77 × 4 |
| 大齿轮 360° | 858,592 counts | 214,648 × 4 |

注：`TOTAL_RANGE = 214648 × 4 = 858592`，基于原始校准值 214,648（电机输出轴一圈）乘以整数齿轮比 4。

## 设计方案（方案 B：显式常量 + 统一偏移）

### 核心常量

```cpp
static constexpr int32_t EXTERNAL_GEAR_RATIO = 4;
static constexpr int32_t TOTAL_RANGE = 858592;   // 214648 × 4，大齿轮满圈
static constexpr int32_t HOME_OFFSET = 2048;      // 归零后逻辑原点偏移（暂定，后期可改 1024）
```

### 位置公式

```
position = HOME_OFFSET + angle_x100 × TOTAL_RANGE / 36000
```

统一偏移，所有倍率条目（含 0.6x @ 0°）使用同一公式，无特殊分支。

### 倍率表（14 条目，完整替换 kDefaultTable）

角度数据来源：镜头厂家实测大齿轮角度。此表**完整替换**现有 `kDefaultTable`，
条目数不变（14），但 zoom_x10 值和 angle_x100 值均有变化（新增 4.5x/5.5x/6.5x，
移除 0.8x/1.2x/1.8x）。

| 倍率 | zoom_x10 | 角度(°) | angle_x100 | position |
|------|----------|---------|------------|----------|
| 0.6x | 6 | 0 | 0 | 2,048 |
| 1.0x | 10 | 72 | 7200 | 173,766 |
| 1.5x | 15 | 114.5 | 11450 | 275,056 |
| 2.0x | 20 | 148 | 14800 | 354,918 |
| 2.5x | 25 | 181.5 | 18150 | 434,781 |
| 3.0x | 30 | 211 | 21100 | 505,167 |
| 3.5x | 35 | 235.5 | 23550 | 563,576 |
| 4.0x | 40 | 257.5 | 25750 | 615,996 |
| 4.5x | 45 | 277 | 27700 | 662,511 |
| 5.0x | 50 | 294.5 | 29450 | 704,264 |
| 5.5x | 55 | 311.5 | 31150 | 744,779 |
| 6.0x | 60 | 325 | 32500 | 776,961 |
| 6.5x | 65 | 338 | 33800 | 808,048 |
| 7.0x | 70 | 347 | 34700 | 829,635 |

### angle_to_position 修改

```cpp
int ZoomTable::angle_to_position(uint16_t angle_x100) const {
    return HOME_OFFSET + static_cast<int>(
        static_cast<int64_t>(angle_x100) * TOTAL_RANGE / 36000
    );
}
```

## 下游常量更新

| 常量 | 所在文件 | 旧值 | 新值 | 来源 |
|------|---------|------|------|------|
| `TOTAL_RANGE` | zoom_table.hpp | 214,648 | 858,592 | ×4 齿轮比 |
| `HOME_OFFSET` | zoom_table.hpp | (新增) | 2,048 | 负极限安全偏移 |
| `EXTERNAL_GEAR_RATIO` | zoom_table.hpp | (新增) | 4 | 外部齿轮比 |
| `HOMING_SETTLE_DISTANCE` | motor_task.hpp | 3,000 | 2,048 | 与 HOME_OFFSET 对齐，归零后落在 0.6x |
| `ACC_START_POS` | motor_task.hpp | 800 | 2,048 | 0.6x position |
| `ACC_END_POS` | motor_task.hpp | 206,821 | 829,635 | 7.0x position |
| `BL_MEASURE_MID` | motor_task.hpp | 107,000 | 434,781 | ≈2.5x 中点 |
| `SOFT_LIMIT_OFFSET` | self_test.cpp | 800 | 改为引用 `HOME_OFFSET` | 消除硬编码重复 |
| 默认倍率表 | zoom_table.cpp | 14 条旧角度 | 14 条新角度/倍率 | 厂家实测数据，完整替换 |

### HOMING_SETTLE_DISTANCE 与 HOME_OFFSET 的关系

归零完成后电机正向移动 `HOMING_SETTLE_DISTANCE` 远离负极限。
将 `HOMING_SETTLE_DISTANCE` 设为 `HOME_OFFSET`（2048），使归零后电机恰好停在 0.6x 位置。
这样归零后无需额外移动即可开始工作。

### self_test.cpp 更新

- `SOFT_LIMIT_OFFSET = 800` → 改为引用 `ZoomTable::HOME_OFFSET`
- SWO 诊断字符串更新：`"ratio=104.77:1"` → `"ratio=104.77:1 × gear 4:1 = 419.08:1"`

## 测试文件更新

以下测试文件包含硬编码旧值，需同步更新：

| 文件 | 内容 | 处理方式 |
|------|------|---------|
| test_zoom_table.cpp:15 | 注释 `TOTAL_RANGE is now a fixed constant (214648)` | 更新注释 |
| test_zoom_table.cpp:85 | `EXPECT_NEAR(pos, 107324, 2)` | 重新计算期望值 |
| test_zoom_table.cpp:99-100 | `get_position(6)` 期望 0，`get_position(70)` 期望 ~206821 | 更新为 2048 和 ~829635 |
| test_motor_ctrl.cpp:251 | `encoder.set_position(206821)` 注释 "7.0x" | 更新为 829635 |
| test_motor_ctrl.cpp:252 | `motor.move_to(800)` | 更新为 2048 |

## 不受影响的部分

- `DEADZONE`(1000)、`FINE_DEADZONE`(100)、`m_iBacklash`(200) — 编码器 counts 级运动控制参数，不随齿轮比变化
- Flash 存储格式 — `[zoom_x10, angle_x100]` 结构不变
- 通信协议 — zoom_x10 单位不变
- 归零流程 — 堵转检测后 encoder 清零，逻辑不变

## 风险与注意事项

1. **HOME_OFFSET 可调性**：暂定 2048，后期可改小至 1024，作为命名常量方便调整
2. **Flash 中旧倍率表兼容**：如果板子 Flash 中存有旧表（旧角度值），加载后角度不变但 position 会因 TOTAL_RANGE 变化而不同 — 需要确认是否需要擦除旧表或加版本号
3. **DECEL_DISTANCE 比例变化**：行程从 ~207K 增大到 ~830K counts，`DECEL_DISTANCE`(15000) 占比从 7% 降至 1.8%，减速区间变窄。首次板上测试应重点验证短行程（相邻倍率）和长行程（0.6x↔7.0x）的减速表现，必要时按比例增大 DECEL_DISTANCE
4. **反向间隙默认值**：`m_iBacklash` 默认 200 counts，在新的齿轮比下每个 count 对应大齿轮角度更小（精度更高），实际测量的间隙绝对值可能变大。首次板上间隙测量后确认默认值是否合理
