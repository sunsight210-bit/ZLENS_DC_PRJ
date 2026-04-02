# DC有刷电机控制策略讨论

日期：2026-04-01

---

## 一、1.0X → 2.0X 倍率切换完整流程

### 目标位置计算

- 1.0X → `angle_x100=7200` → `pos = 128 + 7200×65536/36000 = 13235`
- 2.0X → `angle_x100=14800` → `pos = 128 + 14800×65536/36000 = 27075`
- 移动距离 ≈ **13840 counts**

### 粗调阶段（阶梯速度限幅主导，纯P控制）

| error 范围 | speedCap | 行为 |
|-----------|---------|------|
| > 4000 | 1280（全速）| P输出远超上限，被截断为MAX_SPEED |
| 1000~4000 | 480 | P输出被截断为480 |
| ≤ 1000 | 300 | 进入精调区 |

- `error > INTEGRAL_SEPARATION(32)` 时积分清零，纯P控制
- `MIN_SPEED` 保底：`128 + error×172/200`，防止P输出过小电机不转

### 精调阶段（error ≤ 1000）

- speedCap=300，P项逐渐减小
- `error ≤ 32`：积分开始累积（KI=0.003），MIN_SPEED保底关闭

### 精调-积分段（error ≤ 32）详细过程

P项极小（0.5×32=16），不足以克服静摩擦，进入"攒力-跳动"模式：

```
积分每tick累积 +KI×error
需200~500ms蓄力到 I≈10~15
→ 电机移动1 count
→ error减1，重复
```

脱困爬坡（stuck ramp）：位置不变时每50ms对iMinSpeed ×110%，上限300，强制克服静摩擦。

### 制动稳定

- `|error| ≤ 1` → `brake()`（CCR1=CCR2=ARR=4266）
- 可能因惯性回弹，重新PID修正
- 连续100ms位置不变 → IDLE

### 完成

- `get_nearest_zoom()` → ZOOM=20，发ARRIVED，存FRAM

---

## 二、当前方案问题分析

### 为何不选双环PID（方案A）

- AS5311通过TIM4每1ms采样，精调阶段速度≈0~1 count/ms，速度信号退化为噪声
- 最后几格是静力学问题（克服静摩擦），速度环无帮助
- FreeRTOS 1ms调度周期已是极限，内环无法做到0.1~0.2ms

### 当前方案本质：单环位置PI + 四个补丁

| 补丁 | 替代了什么 |
|------|---------|
| 阶梯限速 | 减速规划 |
| 误差比例最低速 | 速度内环保持力矩 |
| stuck ramp | 积分克服静摩擦 |
| 振荡检测减速 | 速度环阻尼 |

### 不同倍率死区/静摩擦不同

镜头螺旋副摩擦力随位置变化，一组固定参数无法覆盖全程：
- 摩擦小区域：fixed V_END过大 → 过冲
- 摩擦大区域：fixed V_END不够 → 卡住

---

## 三、推荐方案：方案D + 方案B

### 方案B：位置相关摩擦前馈表

**标定原理**：在各倍率停止状态下从0逐步增大PWM，记录电机刚启动时的CCR值。

**ZoomTable扩展**：

```cpp
struct ZOOM_ENTRY_S {
    uint16_t zoom_x10;
    uint16_t angle_x100;
    uint16_t friction_ff;   // 新增：该位置静摩擦补偿PWM值
};
```

**叠加时机**：error进入精调区（≤50）且规划完成后叠加：

```cpp
if (m_Planner.is_done() && std::abs(m_iTarget - pos) <= FRICTION_FF_ZONE) {
    iSpeed += m_iFrictionFf;
}
```

**效果**：
- 当前方案：P=2.5，积分需累积~500ms才够
- 方案B：P=2.5 + ff=110 → 立即克服静摩擦，收敛 < 10ms

---

### 方案D：S曲线轨迹规划

**规划器参数（1.0X→2.0X，dist=13840）**：

```
V_MAX  = 1280 counts/ms
ACCEL  = 20   counts/ms²
DECEL  = 15   counts/ms²
V_END  = 80   counts/ms  （略高于friction_ff对应的最低速）
```

由于 dist=13840 < 减速距离，实际为三角形速度曲线：

```
V_peak = sqrt(2×20×15×13840/35) ≈ 344 counts/ms
```

**速度曲线**：

```
速度
344  │     ▲
     │    / \
     │   /   \
  80 │__/     \___
     └──────────→ 时间
      加速  减速  末端
```

**规划器核心逻辑**：

```cpp
int32_t TrajPlanner::update() {
    int32_t iRemain = std::abs(m_iTo - m_iPosRef);
    int32_t iDecelDist = (m_iVelCur² - V_END²) / (2 × DECEL);

    if (iRemain <= iDecelDist + V_END)
        m_iVelCur = max(m_iVelCur - DECEL, V_END);  // 减速
    else if (m_iVelCur < V_MAX)
        m_iVelCur = min(m_iVelCur + ACCEL, V_MAX);  // 加速

    m_iPosRef += m_iDir × m_iVelCur;
    return m_iPosRef;
}
```

**PID跟踪规划位置（不再跟踪最终目标）**：

```cpp
int32_t posRef = m_Planner.update();
int32_t iError = posRef - pos;   // 跟踪规划曲线
```

---

### 方案D+B 下 1.0X→2.0X 新时间线

```
[SET_ZOOM(20)]  pos=13235 → target=27075

加速段（0~17ms）
  vel: 0→344，posRef: 13235→15200

减速段（17~57ms）
  vel: 344→80，posRef: 15200→26800

末端匀速（57~59ms）
  vel=80，posRef→27075，规划完成

方案B接管（<50ms收敛）
  iSpeed = P + friction_ff(110) → 直接克服静摩擦
  积分微调最后±1

制动稳定（100ms）

ARRIVED
总耗时：~200ms（原来500~1500ms）
```

---

### 可删除的现有补丁代码

| 现有代码 | 处理 | 原因 |
|---------|------|------|
| SPEED_CAP_TIER1/2 阶梯限速 | **删除** | 规划器控制速度 |
| MIN_SPEED_RANGE/ERR_SCALE 误差比例最低速 | **删除** | 前馈替代 |
| STUCK_RAMP_PERIOD/PCT 卡死爬坡 | **删除** | 前馈保证启动力 |
| OSCILLATION_WINDOW/THRESHOLD 振荡检测 | **删除** | 规划曲线不过冲 |
| INTEGRAL_SEPARATION 积分分离 | **保留** | 防大误差积分饱和 |
| DEADZONE + SETTLE_COUNT | **保留** | 到位判定仍需要 |

净结果：motor_ctrl.cpp 删除约60行补丁，新增规划器约50行，复杂度下降。

---

### 实施顺序建议

1. **先做方案B**（摩擦前馈表）
   - 扩展 `ZOOM_ENTRY_S` 加 `friction_ff` 字段
   - 实测各倍率位置最小启动PWM并填表
   - 验证末端收敛时间 < 50ms
   - 这一步就能消除stuck ramp和积分蓄力等待

2. **再做方案D**（S曲线规划）
   - 新增 `TrajPlanner` 类
   - motor_ctrl.cpp 的 `update()` 改为跟踪规划参考位置
   - 删除阶梯限速、误差比例最低速、振荡检测补丁
   - 验证全行程加减速平滑无突变
