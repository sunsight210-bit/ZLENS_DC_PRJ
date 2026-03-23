# ZLENS_DC 项目规则

## 命名规范（必须遵循）

完整版见 `docs/coding-style.md`。核心要点：

- 变量: `作用域前缀` + `类型前缀` + `PascalCase` → `m_iPosition`, `m_eState`
- 类型前缀: `b`bool `c`char `i`int/uint `f`float `d`double `p`指针 `a`数组 `st`结构体 `e`枚举（类实例无前缀）
- 作用域: (无)局部 `m_`成员 `s_`静态 `g_`全局
- 数组前缀 `a` 不与元素类型叠加: `m_aTable[16]`
- 类 `PascalCase` / 结构体 `_S` / 枚举 `_E` / 联合体 `_U`
- 函数 `snake_case()` / bool 方法 `is_`/`has_`/`can_` 前缀
- 常量/宏 `UPPER_SNAKE_CASE` / 文件 `snake_case.hpp` / 命名空间 `snake_case`
- 类成员顺序 `public→protected→private` / 定点后缀 `iZoom_x10` / 头文件 `#pragma once`
- HAL / FreeRTOS / CMSIS 保持原样

## 阶段管理

- 进度看板：`docs/progress.md`（**每次会话开始先读取**）
- 每阶段目录：`docs/phases/phaseN-xxx/`（格式见 `docs/TEMPLATE.md`）
- Superpowers skill 产出保存到阶段目录，不使用 `docs/superpowers/`

每阶段必须创建以下文件：

| 文件 | 职责 |
|------|------|
| `plan.md` | 目标、改动范围、设计要点（阶段开始前） |
| `verify.md` | Mock / 编译 / 板上测试结果（逐步填写） |
| `review.md` | 代码审查、待改进（阶段完成时） |
| `spec.md` | 设计规格（可选，复杂阶段） |

每个新阶段必须有 `scripts/phaseN_verify.py`（项目根目录），接口 `--port`（默认 `/dev/ttyUSB0`）、`--baud`（默认 115200）。脚本只负责验证（发命令 → 校验 → PASS/FAIL），编译烧录由 shell 脚本完成。`scripts/` 禁止放一次性调试脚本。

## 开发流程（必须遵循）

### 质量门控（每步未通过不进入下一步）

1. TDD 编写测试（Red）
2. 实现代码（Green），可重构（Refactor）
3. Mock 测试通过（`gcov` 覆盖率 >80%）→ 记录到阶段 `verify.md`
4. 固件编译通过 → 记录到阶段 `verify.md`
5. 板上测试通过 → 记录到阶段 `verify.md`
6. 填写阶段 `review.md` → git commit → git push

### 构建与调试

**执行编译、烧录、SWO 抓取等硬件操作前，必须先读取 `docs/build-flash-swo-workflow.md`**，按其中步骤执行。

构建目录仅 `build/fw/`、`build/test/`、`build/swo/`，禁止散落 `build_*`。

### 代码边界

- **禁止为迎合测试/诊断而改产品架构代码** — 先区分根因：产品 bug 则修产品代码，测试 bug 则只改测试代码
- 诊断程序的作用是**暴露问题**，不是驱动变更的理由
