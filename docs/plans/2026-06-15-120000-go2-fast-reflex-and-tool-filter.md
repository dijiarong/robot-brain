# 第十二次迭代：Go2 快反规则 + 后端工具过滤

## 基本信息

- 创建时间：2026-06-15 CST
- 文件序号：2026-06-15-120000
- 状态：代码完成，自动化测试通过（288 passed）；真机验证待现场
- 负责人：dijia
- 前置迭代：[第十一次迭代：Go2 原生技能族](./2026-06-14-000000-go2-skill-family.md)
- 选型来源：[第十二次及后续迭代方向（备选）](./2026-06-15-000000-next-iteration-options.md) — **候选 1：A + B**

## Project Requirements

### Goal

消除第十一次开放 LLM 运动能力后的认知层不对称：**快系统（FastReflex）读不懂 Go2 本体状态**，且 **LLM 仍可见 Go2 上不支持的 generic 工具**。本轮在真机测试前补齐：

1. **A — FastReflex Go2 规则**：读 `WorldState.robot_self_state`，对 error/stale/低电/异常运动等做确定性 `stop` / `report`，修正 Unitree 上无效的「低电量 → dock」。
2. **B — Backend-aware 工具过滤**：`robot_backend=unitree` 时 LLM 仅见 Go2 可用 tool 集合；Validator 对误选 generic 运动 skill 硬拒绝。

### 问题陈述

```text
第九次：  执行层 drive/stop 安全闭环 ✅
第十次：  robot_self_state → WorldState ✅
第十一次：LLM 慢路径 nudge/scan/retreat（读 self_state 前置）✅

FastReflex（快路径，DualSystem 中优先于 Planner）：
  - 仍只读 battery_level + estop_active + alerts
  - 不读 error_code / is_moving / is_standing / state_age
  - 低电量 → dock → UnitreeRobot.dock() → NotImplementedError ❌

SkillRegistry：
  - unitree 后端仍向 LLM 暴露 navigate/patrol/follow/dock
  - LLM 可能误选 → move_to 失败或 NotImplementedError ❌
```

### Existing Baseline

- `DualSystem.decide()`：**FastReflex 先于 Planner**；reflex 有输出则 LLM 不参与当轮规划。
- `FastReflex.decide(world)`：estop → stop；critical/low battery → **dock**；critical alerts → report。
- `WorldState`：`robot_self_state`、`state_age_seconds`、`robot_error_code` 便捷属性（第十次）。
- Go2 慢路径技能：`check_robot_self_state()` 与 FastReflex 应共享语义（第十一次 `go2_motion.py`）。
- `SkillRegistry.tools()`：无 backend 过滤；Planner 通过 `skills.tools()` 传给 LLM。

### Features

#### B — 后端工具过滤（先做，体量 S）

- `SkillRegistry` 增加 `tools_for_backend(backend: str) -> list[dict]`（或 `tools()` 接受 `backend` 参数）。
- **unitree 后端 LLM 可见白名单：**
  `nudge`, `scan`, `retreat`, `recognize`, `report`, `stop`
- **unitree 后端隐藏（仍注册在 Registry 内供测试/兼容，但不进 LLM tool list）：**
  `navigate`, `patrol`, `follow`, `dock`
- `Planner` 改为 `self.skills.tools_for_backend(settings.robot_backend)`（或等价）。
- `SafetyValidator._validate_motion()` / 新增 `_validate_backend()`：unitree 下对 hidden skill 直接拒绝，reason 含 `unsupported on unitree backend`（**双保险**，防直接 API 注入 skill 名）。

#### A — FastReflex Go2 规则（体量 M）

- 新增 `cognition/go2_reflex_rules.py`（或扩展 `fast_reflex.py`）：纯函数 `decide_go2_reflex(world, settings) -> list[ToolCall] | None`。
- **启用条件：** `world.robot_self_state is not None`（即 `RDB_PERCEPTION=unitree` 且已 perceive）；否则不走 Go2 规则分支。
- **规则表（优先级自上而下，命中即返回，与现有 estop 规则合并）：**

| 优先级 | 条件 | 动作 | source |
|--------|------|------|--------|
| 1 | `estop_active` | `stop` | fast（已有） |
| 2 | `robot_error_code != 0` | `stop` + `report(critical)` | fast |
| 3 | `state_age_seconds > unitree_state_max_age_seconds` | `stop` + `report(warning)` | fast |
| 4 | `is_standing is False` | `report(warning)`（**不** stop，不 stand_up） | fast |
| 5 | `is_moving is True` 且 `current_task.status` 非 running | `stop` | fast |
| 6 | `battery_level <= critical` | `stop` + `report(critical)` | fast（**替代 dock**） |
| 7 | `battery_level <= low` | `report(warning)` | fast（**替代 dock**） |
| 8 | critical alerts | `report(critical)` | fast（已有） |

- **Unitree / 无 self_state 时的 dock 修正：**
  - 当 `robot_backend == "unitree"` **或** `robot_self_state is not None`：规则 6/7 **不得**调用 `dock`。
  - `robot_backend == "mock"` 且无 self_state：保留原 dock 行为（mock 机器人兼容）。

- **error_code debounce（可选配置，默认 1 次）：**
  - `RDB_GO2_REFLEX_ERROR_DEBOUNCE` 默认 `1`；连续 N 次 perceive 见非零 error 才触发规则 2。
  - 依赖 transport 已过滤的 `error_code`（第十次 MCF echo 已处理），FastReflex 不再解析 raw echo。

- **与慢路径对齐：** stale / not standing / error 的文案与阈值与 `go2_motion.check_robot_self_state()` 保持一致或复用同一 helper（抽取到 `core/` 或 `go2_motion` 供 reflex 只读调用）。

### Constraints

- FastReflex **不**调用 `nudge`/`scan`/`retreat`/`set_posture`/`drive`。
- **不**新增 LLM 技能；不修改 `RobotInterface`。
- **不**实现 LiDAR/视频/dashboard（方向 C/D，后续迭代）。
- 全部规则须可用 **fake transport + 注入 `RobotSelfState`** 单测，**零真机依赖**。
- mock 后端（`RDB_ROBOT=mock`）行为保持不变：无 Go2 规则、dock 仍可用。

### Success Criteria

- [x] `robot_backend=unitree` 时 `Planner` 收到的 tool 列表**不含** navigate/patrol/follow/dock
- [x] unitree 下 Validator 拒绝 hidden skill（即使 tool 名被硬编码进 ToolCall）
- [x] `robot_backend=mock` 时 tool 列表与行为与现网一致
- [x] `robot_self_state` 存在且 `error_code≠0` → FastReflex 返回 stop + report，**先于** Planner
- [x] stale state → stop + report(warning)
- [x] unitree + low/critical battery → stop/report，**不**调用 dock
- [x] mock + low battery → 仍可 dock（回归）
- [x] `RDB_PERCEPTION=mock` + unitree robot：无 Go2 reflex 规则（self_state None），不崩溃
- [x] 新增测试 29，全量 pytest 通过（288 passed）

## 设计方案

### 数据流

```text
perceive → WorldState.robot_self_state
    │
    ▼
DualSystem.decide()
    ├── FastReflex.decide(world)
    │     ├── [generic] estop / alerts
    │     ├── [go2 if self_state] error / stale / battery / is_moving …
    │     └── 有 ToolCall → return Decision(source=fast)  ← 抢占 LLM
    └── Planner.plan(..., skills.tools_for_backend(unitree))
              └── 仅 nudge/scan/retreat/recognize/report/stop
```

### B — API 设计

```python
# skills/registry.py
UNITREE_LLM_SKILLS = frozenset({
    "nudge", "scan", "retreat", "recognize", "report", "stop",
})

def tools_for_backend(self, backend: str, *, strict: bool = True) -> list[dict]:
    if backend == "unitree":
        return [t for s in self._skills.values() if s.name in UNITREE_LLM_SKILLS ...]
    return self.tools(strict=strict)
```

```python
# safety/validator.py
def validate(...):
    ...
    err = self._validate_backend(call.skill_name, self.settings.robot_backend)
    if err:
        return ValidationResult(allowed=False, reason=err, ...)
```

### A — 与 go2_motion 复用

建议将 `check_robot_self_state()` 提升或 re-export 为 reflex 可读：

```python
# 选项 1：go2_motion.check_robot_self_state → 快反只关心「是否要 stop」
# 选项 2：core/go2_readiness.py 共享 stale/standing/error 判断
```

FastReflex 的 `is_moving + task 非 running` 需读 `world.current_task`，无 task 时可跳过规则 5。

### 配置项

| 变量 | 默认 | 说明 |
|------|------|------|
| `RDB_GO2_REFLEX_ERROR_DEBOUNCE` | `1` | 连续非零 error_code 次数才触发快反 |
| （复用）`RDB_UNITREE_STATE_MAX_AGE_SECONDS` | `2.0` | stale 阈值，与 drive 前置一致 |
| （复用）`RDB_LOW_BATTERY` / critical | settings 现有 | 低电阈值 |

### 影响文件

| 文件 | 操作 | 说明 |
|------|------|------|
| `skills/registry.py` | 修改 | `tools_for_backend()`、`UNITREE_LLM_SKILLS` |
| `cognition/planner.py` | 修改 | 传入 backend 过滤后的 tools |
| `runtime/loop.py` | 修改 | 向 Planner/Context 传递 backend（若尚未暴露） |
| `safety/validator.py` | 修改 | `_validate_backend()` |
| `cognition/fast_reflex.py` | 修改 | 集成 Go2 规则 + dock 分支修正 |
| `cognition/go2_reflex_rules.py` | **新建** | 纯函数规则表（可选，便于单测） |
| `skills/builtin/go2_motion.py` | 可能修改 | 抽取共享 readiness 检查 |
| `config/settings.py` | 修改 | error debounce 等 |
| `tests/test_skill_registry_backend.py` | **新建** | B 专项 |
| `tests/test_fast_reflex_go2.py` | **新建** | A 专项 |
| `tests/test_go2_skills.py` | 可能修改 | 确认 Validator 仍通过 |
| `README.md` | 修改 | Go2 工具白名单、FastReflex 行为、更新「当前边界」 |

## 分阶段计划

### 阶段 A：Backend 工具过滤（B，先做）

- [x] `SkillRegistry.tools_for_backend()`
- [x] `Planner` 使用 filtered tools
- [x] `SafetyValidator._validate_backend()`
- [x] 单测：unitree 工具列表、hidden skill 拒绝、mock 回归

验收：

- [x] LLM schema 验证脚本在 unitree 下只打印 6 个运动/认知 tool
- [x] mock 后端仍含 navigate

### 阶段 B：FastReflex Go2 规则（A）

- [x] Go2 规则函数 + 接入 `FastReflex.decide()`
- [x] unitree/self_state 下禁用 dock 快反
- [x] error debounce（已实现，默认 1）
- [x] 单测矩阵覆盖规则 2–7

验收：

- [x] 注入 error_code=7004 → reflex stop 先于 plan
- [x] 注入 stale state → stop + report
- [x] unitree critical battery → 无 dock ToolCall

### 阶段 C：文档与回归

- [ ] README / unitree-setup 补充 FastReflex 与 tool 白名单
- [ ] 全量 pytest 通过
- [ ] 更新第十一次「下一步」与方向池「已选定」

## 测试策略

| # | 测试 | 验证点 |
|---|------|--------|
| 1 | unitree tools 不含 navigate | tools_for_backend 过滤 |
| 2 | unitree tools 含 nudge/scan/retreat | 白名单完整 |
| 3 | mock tools 仍含 navigate | mock 回归 |
| 4 | Validator 拒绝 unitree+navigate | 双保险 |
| 5 | Validator 允许 unitree+nudge | 白名单内 |
| 6 | reflex estop 仍优先 | 规则 1 |
| 7 | reflex error_code → stop+report | 规则 2 |
| 8 | reflex stale → stop+report | 规则 3 |
| 9 | reflex not standing → report only | 规则 4，无 stop |
| 10 | reflex is_moving + task idle → stop | 规则 5 |
| 11 | unitree critical battery → stop+report，无 dock | 规则 6 |
| 12 | unitree low battery → report，无 dock | 规则 7 |
| 13 | mock low battery → dock | 回归 |
| 14 | self_state None → 无 Go2 规则 | perception=mock |
| 15 | DualSystem error 时 source=fast | 集成 |
| 16 | DualSystem 健康时 source=slow | Planner 仍工作 |
| 17 | error debounce N=2 单次不触发 | 可选 |
| 18 | critical alert 仍 report | 规则 8 回归 |

## 验证方式

```bash
python -m pytest tests/test_skill_registry_backend.py tests/test_fast_reflex_go2.py -v
python -m pytest

# B：tool 白名单
python -c "
from config.settings import Settings
from robot_brain.runtime.loop import AgentRuntime
for backend in ('mock', 'unitree'):
    s = Settings(robot_backend=backend, memory_db_path=':memory:')
    rt = AgentRuntime.create(settings=s)
    names = [t['name'] for t in rt.context.skills.tools_for_backend(backend)]
    print(backend, sorted(names))
"

# A：快反抢占（fake self_state）
python -c "
from robot_brain.cognition.fast_reflex import FastReflex
from robot_brain.core.robot_self_state import RobotSelfState
from robot_brain.core.world_state import WorldState
from config.settings import Settings
s = Settings()
w = WorldState()
w.robot_self_state = RobotSelfState(source='u', error_code=42, state_age_seconds=0.1)
calls = FastReflex(s).decide(w)
print([c.skill_name for c in calls])
"
```

真机：非必须；现场可选验证 error/stale 时快反 stop 与 LLM 不再规划 navigate。

## 非目标

- 不实现 approach/follow、感知流、dashboard Go2 面板（第十三及以后）
- 不让 FastReflex 自动 stand_up / damp / sit
- 不修改 Go2 慢路径技能（nudge/scan/retreat）逻辑，仅可选抽取共享 readiness helper
- 不做 Planner prompt 优化（方向 G，可第十三次与 F 合并）

## 风险与缓解

| 风险 | 缓解 |
|------|------|
| 快反 not standing 只 report 不 stop，LLM 仍可能 nudge | 慢路径 `check_robot_self_state` 仍拒绝；可考虑 Validator 读 self_state 阻断运动 skill（可选增强） |
| error debounce 延迟真实 fault 反应 | 默认 N=1；可配置为 0 |
| 隐藏 dock 后 unitree 低电无「回充」 | 文档明确 Go2 无 dock；快反改为 stop+report |
| tools 过滤后 direct API 仍可调 hidden skill | Validator backend 拒绝 |

## 复盘

### 与设计一致的部分

- SkillRegistry.tools_for_backend() + SafetyValidator._validate_backend() 双层过滤
- Go2 规则表全部落地：error/stale/not-standing/moving/battery，全部 fake 可测
- unitree + self_state 下禁用 dock，mock 保留原行为
- error debounce 可配置，默认 1 次即触发
- DualSystem 集成：fast source 抢占 slow planner

### 自动化测试

- 29 passed（10 backend filter + 19 reflex rules）
- 288 passed 全量（含原有 259），无回归

### 真机结果

- 待现场验证

### 下一步

- 方向池见 [2026-06-15-000000-next-iteration-options](./2026-06-15-000000-next-iteration-options.md)
