# 迭代计划：能力底座 — Tool / Skill / Policy / Catalog 契约

## 基本信息

- 创建时间：2026-07-10 17:00:02 CST
- 文件序号：2026-07-10-170002
- 状态：已完成（代码/测试完成，fake/dry-run 可验收；真机待现场）
- 负责人：dijia

## 背景

当前项目已经具备完整认知闭环：感知、规划、安全校验、技能执行、反思、记忆和服务化都已落地。现有 `Skill` 抽象支撑了 mock 行为、Go2 原生动作、探索模式和 LLM tool 暴露，但它同时承担了多个职责：

- 给 LLM 导出的 function/tool schema
- 可执行的机器人动作或行为
- 参数解析和部分前置条件
- 安全校验的命名锚点
- backend 能力过滤的单位

这让系统在小规模技能集下可以工作，但继续扩展自主行为、底层工具、传感器能力和不同机器人 backend 时，复杂度会快速集中到 `SkillRegistry`、`SafetyValidator` 和具体 skill 实现中。典型表现是安全策略按 skill 名称写死、backend 过滤靠静态白名单、低层动作和高层行为没有清晰边界。

本次迭代目标不是推倒重构，而是建立一个最小可运行的能力底座，让后续能力按统一契约生长。

## 核心判断

项目下一阶段应将能力拆成四层：

| 层 | 职责 | 是否直接给 LLM |
|----|------|----------------|
| `Tool` | 原子能力，面向 runtime 执行，例如停止、短时 drive、读取状态、报告消息 | 默认否 |
| `Skill` | 行为编排，组合一个或多个 tool，表达用户/规划器可理解的动作意图 | 可选 |
| `Policy` / `SafetyRule` | 独立安全约束，根据能力 metadata、world state、settings 判定是否允许 | 否 |
| `PlannerCatalog` | 给 LLM / planner 的可见能力集合，负责暴露、隐藏、降级和描述 | 是 |

原则：**tool 是机器能力，skill 是行为语义，policy 是安全边界，catalog 是规划器视图。**

## 目标

- [x] 新增基础 `Tool` 契约，支持参数 schema、执行结果、能力 metadata 和 backend 声明
- [x] 新增 `ToolRegistry`，作为 runtime 内部原子能力注册表
- [x] 定义 `CapabilityMetadata`，覆盖风险等级、运动类型、是否需要确认、backend allowlist、是否暴露给 planner
- [x] 新增最小 `Policy` / `SafetyRule` 结构，把至少一部分硬编码安全规则从 skill 名称判断迁出
- [x] 新增 `PlannerCatalog` 或等价适配层，明确 LLM 可见能力不是 registry 全量能力
- [x] 迁移一个最小纵切：优先选择 `stop`，可选再迁移 `nudge`
- [x] 保持现有 skill API 兼容，避免一次性重写全部技能
- [x] 增加测试覆盖，证明旧链路不回归，新契约可用

## 非目标

- 不重写整个 orchestration graph
- 不一次性迁移所有 Go2 skills
- 不改变现有 HTTP API 行为
- 不开放新的真实动作能力
- 不把所有底层 tool 暴露给 LLM
- 不引入复杂工作流引擎或插件系统

## 设计方案

### 1. Tool 契约

新增模块建议：

```text
robot_brain/tools/
  __init__.py
  base.py
  registry.py
  builtin/
    __init__.py
    control.py
```

`Tool` 是 runtime 内部原子能力，不等同于 OpenAI function tool。建议字段：

| 字段 | 说明 |
|------|------|
| `name` | 稳定机器名，例如 `stop_motion` |
| `description` | 面向开发者和 planner catalog 的说明 |
| `params_model` | Pydantic 参数模型 |
| `metadata` | 能力风险和 backend 声明 |
| `execute(params, context)` | 执行原子动作 |

建议新增 `ToolContext`，避免每个 tool 的执行签名不断扩张：

```text
ToolContext(settings, world, robot, perception, short_term, long_term)
```

初期可只放 `settings`、`world`、`robot`，后续再扩展。

### 2. CapabilityMetadata

建议字段：

| 字段 | 示例 | 用途 |
|------|------|------|
| `risk_level` | `read_only` / `low` / `medium` / `high` / `critical` | 安全分级 |
| `motion_kind` | `none` / `stop` / `linear` / `yaw` / `posture` | 运动约束 |
| `requires_confirmation` | `true` / `false` | 人工确认 |
| `backend_allowlist` | `["mock", "unitree"]` | backend 过滤 |
| `planner_visible` | `true` / `false` | 是否可进入 planner catalog |
| `tags` | `{"go2", "motion"}` | 查询、审计和 UI 展示 |

这部分是本次迭代的关键。它要替代一部分散落在 `SafetyValidator` 和 `SkillRegistry` 中的硬编码知识。

### 3. Policy / SafetyRule

先做轻量结构，不要过度抽象。建议新增：

```text
robot_brain/safety/policy.py
```

第一批规则：

- backend 不允许则拒绝
- 急停状态下仅允许 `motion_kind=stop` 或 `risk_level=read_only`
- 低电量仅允许停止、报告、回充类能力
- 需要确认但未确认则返回 `requires_confirmation`
- 参数超出 metadata 或 rule 限制则拒绝

短期内 `SafetyValidator` 可以继续存在，但把部分判断委托给 policy。目标是渐进迁移，而不是制造双套安全系统。

### 4. PlannerCatalog

新增 planner 可见层，建议模块：

```text
robot_brain/planning/catalog.py
```

职责：

- 从 skill registry 和未来 tool registry 中筛选 planner 可见能力
- 根据 backend、settings、world state 生成 LLM tool schema
- 对外保持当前 `tools_for_backend()` 兼容语义
- 明确低层 tool 默认不可见

第一阶段可以先服务现有 skill，后续再允许少量安全 tool 直接暴露。

### 5. 最小纵切

优先迁移 `stop`：

1. 新增 `StopMotionTool`
2. `StopSkill.execute()` 内部调用 `StopMotionTool`
3. `StopMotionTool.metadata.motion_kind = "stop"`
4. policy 允许急停状态下执行 stop
5. 保持现有 LLM tool 名称 `stop` 不变

如果时间允许，再迁移 `nudge` 的一部分：

1. 新增 `Go2DriveSegmentTool`
2. `NudgeSkill` 继续负责距离、方向和分段语义
3. 低层 tool 只负责单段限时 drive
4. policy 对 `motion_kind=linear` 应用速度、距离、确认和 backend 规则

## 影响模块

| 模块 | 预计变化 |
|------|----------|
| `robot_brain/tools/*` | 新增 tool 契约、registry 和首批内置 tool |
| `robot_brain/skills/base.py` | 可选增加 metadata 或 tool 调用辅助，不破坏现有接口 |
| `robot_brain/skills/builtin/catalog.py` | `StopSkill` 迁移到底层 tool |
| `robot_brain/skills/builtin/go2_catalog.py` / `go2_motion.py` | 可选迁移 `nudge` 纵切 |
| `robot_brain/safety/validator.py` | 增加 policy 委托，减少按 skill 名称判断 |
| `robot_brain/safety/policy.py` | 新增统一能力校验规则 |
| `robot_brain/cognition/planner.py` | 可选改为使用 PlannerCatalog 生成 tools |
| `robot_brain/runtime/loop.py` | 注入 ToolRegistry / ToolContext |
| `tests/*` | 新增契约和纵切测试，保持旧测试通过 |

## 实施步骤

### 阶段 A：契约落地

- [x] 新增 `CapabilityMetadata`
- [x] 新增 `ToolResult`、`ToolContext`、`Tool`
- [x] 新增 `ToolRegistry`
- [x] 增加基础单元测试：注册、重复注册、schema 导出、backend metadata

### 阶段 B：Policy 最小化

- [x] 新增 `SafetyPolicy`
- [x] 支持 backend allowlist 校验
- [x] 支持 estop + motion_kind 校验
- [x] 支持 requires_confirmation 校验
- [x] 在 `SafetyValidator` 中接入 policy，但保留旧逻辑兜底

### 阶段 C：Stop 纵切

- [x] 新增 `StopMotionTool`
- [x] 修改 `StopSkill` 通过 tool 执行
- [x] 确认急停、低电量、确认逻辑不回归
- [x] 增加 stop skill/tool 端到端测试

### 阶段 D：PlannerCatalog 适配

- [x] 新增 `PlannerCatalog`
- [x] 当前先从 `SkillRegistry` 生成 planner tools
- [x] 将 backend 过滤逻辑从 `SkillRegistry` 迁移或包装到 catalog
- [x] 保持现有 LLM client 输入格式不变

### 阶段 E：可选 Nudge 纵切

- [x] 新增 `Go2DriveSegmentTool`
- [x] `NudgeSkill` 使用底层 tool 执行分段 drive
- [x] metadata 表达 Go2 backend、低速线性运动、确认需求
- [x] 增加 fake transport 测试

## 验证方式

- [x] `pytest tests/test_skill_registry_backend.py`
- [x] `pytest tests/test_go2_skills.py`
- [x] `pytest tests/test_fast_reflex_go2.py`
- [x] 新增 `tests/test_tool_registry.py`
- [x] 新增 `tests/test_safety_policy.py`
- [x] 新增 `tests/test_capability_stop_vertical.py`（含 stop + nudge 纵切）
- [x] 全量测试通过（429 passed, 4 skipped）
- [x] 手动运行 `python examples/run_demo.py`
- [x] 可选：`RDB_ROBOT=unitree RDB_PERCEPTION=unitree` 使用 fake/dry-run 验证服务启动（已用 fake transport 冒烟验证 runtime 创建、tool 注入、stop 端到端）

## 验收标准

- ✅ 现有 LLM tool 名称和用户 API 不破坏
- ✅ 至少一个现有 skill 通过底层 tool 执行（stop + nudge 两个）
- ✅ planner 可见能力与 runtime 内部能力有明确分层（`PlannerCatalog` 仅暴露 skill；`stop_motion`/`go2_drive_segment` 默认 `planner_visible=False`）
- ✅ `SafetyValidator` 不再是唯一承载所有能力知识的地方（已迁移 skill 的 estop/battery/backend/confirmation 由 `SafetyPolicy` 按 metadata 判定）
- ✅ 新增契约有测试覆盖，不依赖真机
- ✅ 文档能指导后续迁移 `scan`、`retreat`、`explore` 和感知类工具

## 风险与取舍

| 风险 | 处理方式 |
|------|----------|
| 抽象过度，拖慢实际能力开发 | 本轮只做 stop 纵切，nudge 作为可选 |
| 新旧 validator 并存导致重复判断 | 明确 policy 先覆盖通用规则，skill 名称规则逐步缩小 |
| LLM tool 与内部 tool 命名混淆 | 代码和文档统一使用 `PlannerCatalog` 区分 planner-visible tools |
| Go2 真机验证不可用 | 所有验收默认 fake/dry-run，可现场再补 live 记录 |

## 后续方向

- 将 `scan`、`retreat` 迁移为 skill 组合底层 motion tool
- 将 `recognize`、`observe` 抽成 read-only perception tools
- 将 `explore` 改为显式 skill composition，便于调试和审计
- 为 service/dashboard 展示 capability catalog、风险等级和最近 tool 执行记录
- 支持按场景生成不同 planner catalog，例如巡逻、遥操、只读观察、现场验收

## 复盘

迭代完成后补充：

### 实际迁移了哪些 skill/tool

- 新增四层契约：`robot_brain/tools/`（`Tool` / `ToolResult` / `ToolContext` / `CapabilityMetadata` / `ToolRegistry`）、`robot_brain/safety/policy.py`（`SafetyPolicy`）、`robot_brain/planning/catalog.py`（`PlannerCatalog`）。
- 迁移 `stop`：新增 `StopMotionTool`（`motion_kind=stop`），`StopSkill.execute()` 委托底层 tool，`capability_metadata` 转发 tool metadata。
- 迁移 `nudge`：新增 `Go2DriveSegmentTool`（`motion_kind=linear`、`backend_allowlist=("unitree",)`、`requires_confirmation=True`），`NudgeSkill` 保留距离/方向/分段语义，逐段调底层 tool，`data` 形状与 `run_go2_drive_segments` 时代完全一致。
- `scan` / `retreat` / `explore` 本轮未迁移，继续走 `go2_motion.run_go2_drive_segments` 与 legacy 校验路径。

### `SafetyValidator` 中减少了哪些硬编码

- 已迁移 skill（stop / nudge）的 backend allowlist、estop、低电量、confirmation 判断从 `ALWAYS_ALLOWED` / `_validate_backend` / `require_confirmation_for` 迁到 `SafetyPolicy`，按 `CapabilityMetadata` 判定。
- 非迁移 skill 仍走 legacy 名称路径（`ALWAYS_ALLOWED`、`_validate_backend`、`require_confirmation_for`），二者覆盖不相交的能力集，避免双套判断。
- `_validate_motion`（按 skill 名的参数范围校验）本轮保留，待后续随 skill 迁移逐步下沉到 tool/rule。

### 是否影响现有 Go2 fake/dry-run 链路

- 未影响。`run_go2_drive_segments` 函数保留（scan/retreat/explore 仍用，且 `test_go2_skills.py` 直接导入测试）；`NudgeSkill` 输出 `data` 形状不变。
- `SkillRegistry.tools_for_backend()` 行为不变（懒导入委托 `PlannerCatalog`），`tools()` 输出格式不变，LLM client / output validator 无感知。
- 全量 429 passed / 4 skipped；mock 与 unitree(fake/dry-run) 两种 backend 的 runtime 创建、tool 注入、`stop` 端到端均冒烟通过。

### 下一轮应优先迁移 motion skill 还是 perception tool

- 建议先迁移 `scan` / `retreat`：二者与 `nudge` 同构（分段 drive），可直接复用 `Go2DriveSegmentTool`，把 `go2_motion.run_go2_drive_segments` 的剩余调用方收敛到统一底层 tool，并补 `motion_kind=yaw`（scan）的 policy 路径。
- 随后抽 `recognize` / `observe` 为 `risk_level=read_only` 的 perception tool，验证 estop 下只读能力放行的 policy 分支。
- `explore` 改为显式 skill composition（组合 scan/nudge/retreat 而非直调 motion），便于审计与单步调试。

### 取舍记录

- `PlannerCatalog` 与 `SkillRegistry` 解耦靠懒导入避免循环依赖；backend 过滤常量 `UNITREE_LLM_SKILLS` / `UNITREE_HIDDEN_SKILLS` 仍留在 `skills/registry.py` 作为单一事实源，catalog 与 validator 共同引用。
- `ToolContext` 初期只放 `settings` / `world` / `robot`，`perception` / `short_term` / `long_term` 预留为 `None`，待 perception tool / 审计需求落地再启用。
- 阶段 E 中 nudge 的参数范围校验（10–50 cm）暂留在 `_validate_motion`，未迁入 policy，避免与 motion 校验顺序产生行为变更；待 motion 校验整体下沉时一并处理。

### Review 修复（P1–P3）

- **P1（已修）确认顺序**：原实现把 `SafetyPolicy.evaluate`（含 confirmation）放在参数解析与 `_validate_motion` 之前，导致已迁移技能（如 nudge）非法参数会先返回 `SAFETY_CONFIRMATION_REQUIRED`（例如 `distance_cm=999`），等于让操作者确认一个本来就非法的动作。修复：把 policy 拆成 `check_backend` / `check_state` / `check_confirmation` 三段，validator 按 `backend → 解析 → state → preconditions → motion → confirmation` 顺序调用，confirmation 放到最后。新增回归测试 `test_nudge_out_of_range_is_motion_violation_not_confirmation`。
- **P2（已修）.gitignore**：原 `.gitignore` 第 10 行误用全角句号 `。claude`，导致 `.claude/` 未被忽略（含 `settings.local.json` 的 `bypassPermissions`）。改为 `.claude/`，`git check-ignore` 已确认生效。
- **P3（确认为有意设计）确认配置兼容**：nudge 的确认需求从 `settings.require_confirmation_for` 迁到 `Go2DriveSegmentTool.metadata.requires_confirmation=True`，这是文档「metadata 作为安全事实源」方向的预期结果，非遗漏。已迁移技能的 `requires_confirmation` 以 metadata 为准，`settings.require_confirmation_for` 仅对未迁移技能生效。代价：无法再通过修改 settings 取消已迁移运动技能的确认（默认行为未变）。按场景调整确认策略属于文档「后续方向」中的 scenario catalog 机制，不走 settings。
