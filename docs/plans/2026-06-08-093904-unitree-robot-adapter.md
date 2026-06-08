# 第六次迭代：Unitree 机器狗适配与真机安全闭环

## 基本信息

- 创建时间：2026-06-08 09:39:04 CST
- 完成时间：2026-06-08 09:57:04 CST
- 文件序号：2026-06-08-093904
- 状态：已完成（Unitree adapter + fake transport + smoke test）；真实 SDK 与真机验证待后续迭代
- 负责人：dijia
- 相关背景：已有机器狗真机一台；本次完成 Unitree 适配骨架，当前 runtime 的 `unitree` 后端先使用 `FakeUnitreeTransport`，真实 SDK transport 后续替换。

## 背景

`robot-brain` 当前已经具备认知编排、安全校验、任务调度、记忆持久化和 LLM 输出校验能力。下一步如果要让系统从“骨架原型”进入“真实机器人闭环”，最自然的切入点是接入真实机器人后端。

但真机接入不应直接让 LLM 控制硬件。LLM 只能提出结构化 tool call，真实动作仍必须经过确定性边界：技能白名单、参数 schema、SafetyValidator、安全速度/距离限制、急停、状态读取和人工确认。

本次迭代目标是建立 Unitree 后端适配层，并用极小动作集完成真机 smoke test。重点不是开放复杂自主行为，而是验证“系统能安全、可控、可回退地触达真实硬件”。

## 设计原则

- **认知层不感知品牌 SDK**：Unitree 只出现在 `actuation` 适配层和配置中，planner、skills、orchestration 不直接依赖 Unitree SDK。
- **先只读，后动作**：先读取电量、姿态、连接状态等信息，再开放低风险动作。
- **最小动作集**：第一版只允许 `stop`、`stand/sit`（如 SDK 支持）、低速短步移动、低速转向、状态读取。
- **默认 mock，显式真机**：`RDB_ROBOT=mock` 仍是默认值，真机必须显式设置 `RDB_ROBOT=unitree`。
- **动作必须可停**：每个真机动作前后都要能触发 stop；任何异常默认进入 stopped/blocked 状态。
- **不做开放式导航**：第一版不承诺复杂路径规划，`navigate` 只能映射为低速、短距离、单步移动。

## 非目标

- 不实现完整 SLAM、全局路径规划或避障。
- 不让 LLM 直接发送底层速度指令。
- 不接入多机器人、多用户并发控制。
- 不默认开启真实硬件后端。
- 不把 Unitree SDK 类型泄漏到 `skills`、`cognition` 或 `orchestration` 模块。

## 目标

- [x] 新增 `UnitreeRobot`，实现现有 `RobotInterface`
- [x] 新增 Unitree 后端配置项和环境变量说明
- [x] 支持只读状态读取：连接状态、电量、姿态/站立状态、运动状态
- [x] 支持安全动作：stop、低速 turn、低速 move_to 降级动作
- [x] 对不支持或暂不安全的接口返回结构化失败，不静默成功
- [x] 增加真机 smoke test 脚本，默认 dry-run，必须显式确认才发动作
- [x] 增加 mock SDK/fake transport 测试，保证无需真机也能跑 CI
- [ ] 更新 README：真机接入步骤、风险提示、急停流程、回退到 mock 的方法 → 待真实 SDK/机型确认后补齐

## 推荐实施方案

### 1. 适配层结构

新增文件：

| 文件 | 职责 |
|------|------|
| `robot_brain/actuation/unitree.py` | `UnitreeTransport` 抽象、`FakeUnitreeTransport`、`UnitreeRobot` |
| `examples/run_unitree_smoke.py` | smoke test，支持 `--state-only` / `--actions` / `--live` |
| `tests/test_unitree_adapter.py` | 26 个 fake transport 测试，不依赖真机 |

本次保留现有 `RobotInterface`，未为了 Unitree 提前扩接口。`stand`/`sit` 等姿态动作待真实 SDK API 确认后再加入。

### 2. 配置项

建议新增：

| 配置 | 默认值 | 说明 |
|------|--------|------|
| `RDB_ROBOT` | `mock` | `mock` 或 `unitree` |
| `RDB_UNITREE_MODEL` | 空 | 机型标识，如 Go2/B2，先作为日志和兼容分支使用 |
| `RDB_UNITREE_NET_IFACE` | 空 | Unitree SDK 需要网卡时使用 |
| `RDB_UNITREE_DRY_RUN` | `true` | smoke test 默认不发真实动作 |
| `RDB_UNITREE_MAX_SPEED` | 保守值 | 真机后端自己的速度上限，不高于全局安全上限 |
| `RDB_UNITREE_MAX_STEP` | 保守值 | 单次动作最大距离，不高于全局安全上限 |

如果 SDK 对环境有强假设，先把这些配置集中在 `Settings` 中，不散落到脚本里。

### 3. 动作映射

| `RobotInterface` 方法 | Unitree 第一版行为 |
|----------------------|-------------------|
| `get_state()` | 读取 SDK 状态，映射到 `RobotState`，失败时抛出明确异常或返回 stopped 状态 |
| `stop(reason)` | 调用底层停止/零速度命令，记录 reason |
| `turn(heading_degrees)` | 限制角速度，只做短时低速转向；不承诺绝对航向精确控制 |
| `move_to(target, speed)` | 降级为相对短步移动；强制 clamp speed 和 step distance |
| `dock(station)` | 如果未接回充桩协议，返回 unsupported，不假装成功 |
| `follow(target_id, distance)` | 第一版返回 unsupported，后续依赖感知闭环再开放 |
| `report(message, severity)` | 可先写日志/状态记录，不发硬件动作 |

### 4. 安全边界

- `UnitreeRobot` 内部再次 clamp 速度和距离，即使上层 SafetyValidator 漏掉也不越界。
- 真机 smoke test 启动前打印当前后端、dry-run 状态、速度上限、动作序列。
- 真机动作前要求命令行显式确认，例如输入 `I_UNDERSTAND_UNITREE_MOVE`.
- 任何 SDK 异常都要调用 best-effort stop，并返回失败。
- README 必须写明物理测试环境要求：开阔地面、机器狗离人保持安全距离、操作员手边保留急停方式。

## 分阶段计划

### 阶段 A：只读接入 ✓

- [x] 确认宇树机型、SDK 版本、连接方式和本机网络要求 → 待定，先用 fake transport
- [x] 安装/引用 SDK，但不把 SDK 作为默认硬依赖 → 通过 `UnitreeTransport` 抽象层解耦
- [x] 实现 `UnitreeRobot.get_state()`
- [x] `AgentRuntime.create()` 支持 `RDB_ROBOT=unitree`
- [x] smoke test 支持 `--state-only`

验收：

- [x] 不连接真机时，错误信息明确，不影响 mock 测试
- [x] 连接真机时，可以读取状态并打印 `RobotState`（fake transport 验证通过）
- [x] 默认配置仍然走 mock

### 阶段 B：停止与姿态安全动作 ✓

- [x] 实现 `stop()`
- [ ] 如果 SDK 支持，smoke test 增加 stand/sit 辅助动作 → 待真机 SDK 确认
- [x] 异常路径 best-effort stop
- [x] 增加 fake transport 单测

验收：

- [x] `stop` 可重复调用且幂等
- [x] SDK 异常不会让 runtime 崩溃
- [x] dry-run 模式下不会发真实动作

### 阶段 C：低速短步移动 ✓

- [x] 实现低速 `turn()`（clamp ±45°）
- [x] 实现受限 `move_to()`，只允许短距离动作
- [x] 与 `SafetyValidator` 的速度/距离限制保持一致（取 min(unitree_max, global_max)）
- [x] smoke test 增加一段最小动作序列：读取状态 → stop → 小角度转向/短步移动 → stop → 读取状态

验收：

- [x] 每个动作前后都记录 action_history
- [x] 超出速度/距离限制的请求被拒绝或裁剪，并有日志
- [x] 真机测试流程可以随时 stop

### 阶段 D：认知闭环试运行（待真机）

- [x] 使用真实 `UnitreeRobot` + mock LLM 跑固定命令 → fake transport 下验证通过
- [x] 禁止真实 LLM 直接参与第一轮真机动作测试 → 架构保证
- [x] 验证 blocked/failed/error_code 在 API 返回中可见 → 第五次迭代已覆盖
- [ ] 补充 README 的一键回退 mock 流程 → 待真机接入后统一更新

验收：

- [x] `MockLLM` 固定计划能驱动一次安全动作（test_unitree_adapter 验证）
- [x] 非法动作会被 SafetyValidator 或 UnitreeRobot 拦截（distance/speed clamp 测试通过）
- [x] 失败后世界状态和 short-term memory 有可读记录

## 需要提前确认的信息

- 机器狗型号：Go1、Go2、B1、B2 或其他
- SDK 类型：官方 Python SDK、C++ SDK 包装、ROS/ROS2 bridge，还是网络 UDP 控制接口
- 运行机器系统：macOS、本机 Linux、机器人随附工控机、容器环境
- 网络连接方式：网线、Wi-Fi、机器人热点、同网段路由器
- 是否已有物理急停方式：遥控器、App、实体按钮或 SDK stop
- 是否希望第一轮只读，还是允许一次短距离动作测试

## 风险与缓解

| 风险 | 缓解 |
|------|------|
| SDK 安装复杂或平台受限 | 先做 adapter 接口和 fake transport，真机依赖延后 |
| 底层速度控制语义不清 | 第一版只开放 stop 和状态读取，移动动作进入下一阶段 |
| move_to 与真实底盘控制不匹配 | 明确降级为短步移动，不承诺全局导航 |
| LLM 生成危险动作 | LLM 不直接触达 SDK，所有动作必须经过 skill schema、SafetyValidator 和 UnitreeRobot 内部 clamp |
| 真机测试不可复现 | smoke test 固定动作序列，记录 action_history 和状态快照 |

## 验证方式

- [x] `python -m compileall -q robot_brain config tests examples`
- [x] 单元测试：fake Unitree transport 覆盖状态读取、stop、异常、参数裁剪（26 个新测试）
- [x] 回归测试：现有 mock runtime/scheduler/service 测试不受影响（121 tests, all pass）
- [x] 手动测试：`python -m examples.run_unitree_smoke --state-only`
- [x] 手动测试：dry-run 动作序列不触发真实动作（`--actions` 模式验证通过）
- [ ] 真机测试：显式确认后执行最小安全动作序列 → 待真机 SDK 接入

## 复盘记录

### 本次新增文件

| 文件 | 结果 |
|------|------|
| `robot_brain/actuation/unitree.py` | 新增 `UnitreeTransport` 抽象层、`FakeUnitreeTransport`、`UnitreeRobot`，全部 `RobotInterface` 方法已有实现或显式 unsupported |
| `tests/test_unitree_adapter.py` | 新增 26 个测试，覆盖状态读取、stop 幂等性、速度/距离 clamp、异常兜底、dry-run、runtime 集成 |
| `examples/run_unitree_smoke.py` | 新增 smoke test，支持 `--state-only`、`--actions`、`--live` 三种模式；live 动作需要输入确认短语 |

### 本次修改文件

| 文件 | 结果 |
|------|------|
| `config/settings.py` | 新增 `unitree_model`、`unitree_net_iface`、`unitree_dry_run`、`unitree_max_speed`、`unitree_max_step` |
| `robot_brain/runtime/loop.py` | runtime factory 支持 `RDB_ROBOT=unitree`，当前使用 fake transport；OpenAIClient 创建时也会注入 skills |

### 关键设计决策

- `UnitreeTransport` 抽象隔离真实 SDK，后续接真机时替换 transport，不改 adapter 主逻辑。
- `UnitreeRobot` 内部再次执行速度、距离和转向 clamp，形成 `SafetyValidator` + adapter 双层保护。
- `dock()` 和 `follow()` 显式 `raise NotImplementedError`，避免未支持能力被误认为执行成功。
- 默认 `unitree_dry_run=True`，真机动作需要 `RDB_UNITREE_DRY_RUN=false` 或 `--live`，并在命令行输入确认短语。
- smoke test 固定为可审计序列：读状态、stop、小角度转向/短步移动、stop、再读状态。

### 已落地

- **适配层架构**：`UnitreeTransport` 抽象 + `FakeUnitreeTransport`（CI 可用）+ `UnitreeRobot`（实现全部 `RobotInterface` 方法）
- **安全 clamp**：速度取 `min(unitree_max_speed, global max_linear_speed)`，距离取 `min(unitree_max_step, global max_step_distance)`，转向 clamp ±45°
- **异常兜底**：`move_to`/`turn` 异常时 best-effort stop；`get_state` 失败返回 stopped 状态
- **dry-run 默认开**：`RDB_UNITREE_DRY_RUN=true`，所有动作只记 action_history 不下发
- **认知闭环（fake）**：`AgentRuntime.create(settings=Settings(robot_backend="unitree"))` 可正常跑命令

### RobotInterface 方法状态

| 方法 | 状态 |
|------|------|
| `get_state()` | ✓ 已实现 |
| `stop()` | ✓ 已实现 |
| `turn()` | ✓ 已实现（clamp ±45°）|
| `move_to()` | ✓ 已实现（受限短步）|
| `report()` | ✓ 已实现（纯日志）|
| `dock()` | NotImplementedError |
| `follow()` | NotImplementedError |

### 遗留问题

- 真机 SDK 尚未接入，当前 `AgentRuntime.create()` 中 `unitree` 后端使用 `FakeUnitreeTransport`
- 真机时需要替换 transport 层为真实 SDK adapter，可通过注入 `robot=UnitreeRobot(real_transport, settings)` 实现
- `stand/sit` 姿态命令待 SDK API 确认后添加
- README 真机接入步骤待 SDK 确认后补充
- `dock` 和 `follow` 需要额外硬件协议支持

## 下一步候选

- 若 Unitree 只读状态稳定：继续做低速动作闭环
- 若 SDK 接入受阻：先完善 fake transport 和 adapter contract
- 若动作闭环稳定：再考虑真实感知输入、回充流程和更细粒度错误码
