# 第十二次及后续迭代方向（备选）

## 基本信息

- 创建时间：2026-06-15 CST
- 文件序号：2026-06-15-000000
- 状态：方向池 — **第十二次已选定候选 1（A + B）**，见 [正式计划](./2026-06-15-120000-go2-fast-reflex-and-tool-filter.md)
- 负责人：dijia
- 前置完成：[第九次 实机操控闭环](./2026-06-11-172047-unitree-live-control-loop.md) · [第十次 Perception Bridge](./2026-06-13-000000-unitree-perception-bridge.md) · [第十一次 Go2 技能族](./2026-06-14-000000-go2-skill-family.md)

## 当前基线（2026-06-15）

| 层 | 已有能力 | 主要缺口 |
|----|----------|----------|
| 执行 | WebRTC drive / stop / 姿态、motion lease、watchdog、teleop 示例 | 真机 Level 0–5 分级验收未正式归档 |
| 感知 | `robot_self_state` 注入 WorldState（sport/IMU/速度/新鲜度） | 无 LiDAR / 视频 / 里程计流 |
| 认知 | LLM + mock 规划；FastReflex 仅电量/急停/alerts | FastReflex **未读** `robot_self_state` |
| 技能 | generic（navigate 等）+ Go2（nudge/scan/retreat） | generic 在 Go2 下仍暴露；无 approach |
| 服务 | FastAPI + 轻量 dashboard + WS | dashboard 不显示 Go2 本体状态；teleop 未并入主服务 |

**真机约束：** 若近期仍无法上狗，优先选「fake/mock 可完整验收」的方向；需真机的方向可只做设计与 stub，验收标为待现场。

---

## 如何选型

每条方向标注：

- **体量：** S（约 1–3 天）/ M（约 1 周）/ L（2 周+）
- **真机：** 必须 / 可选 / 不需要
- **依赖：** 必须先具备的前置项
- **价值：** 对整体架构的主贡献

建议组合（按常见优先级）：

1. **稳妥推进认知闭环：** 方向 A → 方向 B → 方向 G  
2. **强化操作与可观测：** 方向 C → 方向 H  
3. **感知扩展（工作量大）：** 方向 D → 方向 E  
4. **等真机后再做：** 方向 H 中的分级归档、方向 D 的 LiDAR 实机对齐  

---

## 方向 A：FastReflex 真传感器规则（推荐优先）

**代号：** `fast-reflex-go2-rules`  
**体量：** M · **真机：** 可选（fake `robot_self_state` 可测大部分规则）

### 目标

让快系统在读 `WorldState.robot_self_state` 后，对 Go2 本体状态做**确定性**反应，不经过 LLM。与第十一次「LLM 慢路径技能」互补。

### 典型规则（可分期实现）

| 条件 | 动作 | 备注 |
|------|------|------|
| `estop_active` | `stop` | 已有 |
| `battery_level ≤ critical` | `stop` + `report` | Unitree **无 dock**，不宜再调 `dock`（当前会 NotImplemented） |
| `robot_error_code ≠ 0` 连续 N 次 | `stop` + `report(critical)` | 需 debounce，过滤 MCF echo |
| `state_age_seconds > 阈值` | `stop` + `report(warning)` | 与第九次 stale 语义一致 |
| `is_moving` 且任务已结束 / 异常 | `stop` | 防「停不下来」 |
| `not is_standing` 且队列含运动 skill | 阻塞或 `report` | 不自动 stand_up（操作者权限） |

### 主要改动

- `cognition/fast_reflex.py`：读 `world.robot_self_state`、`world.state_age_seconds`
- `config/settings.py`：阈值、debounce、Go2 专用开关
- `tests/test_fast_reflex_go2.py`

### 非目标

- 不新增 LLM 技能；不让 FastReflex 调用 `nudge`/`set_posture`

### 验收

- fake transport + 注入 `RobotSelfState` 覆盖各规则
- Go2 后端下 FastReflex 在 perceive 后能抢先 `stop`/`report`
- 修正「低电量 → dock」在 Unitree 上的无效路径

---

## 方向 B：Backend-aware 工具暴露（Go2 LLM 安全）

**代号：** `skill-registry-backend-filter`  
**体量：** S · **真机：** 不需要

### 目标

解决第十一次已知限制：`robot_backend=unitree` 时 LLM 仍能看到 `navigate`/`patrol`/`follow`，易误调用。

### 方案（二选一或组合）

1. **过滤：** `SkillRegistry.tools(backend=...)` 仅暴露 Go2 + 通用只读（recognize/report/stop）  
2. **标注：** 保留全量 tools，但在 description 标 `[unsupported on Go2]`，Validator 直接拒绝  

### 主要改动

- `skills/registry.py`、`runtime/loop.py`、`safety/validator.py`
- Planner / LLM prompt 使用的 tool 列表
- 测试：unitree 后端 tools 不含 navigate，或 navigate 校验失败

### 验收

- mock LLM 在 Go2 后端只能选到合理 tool 集合
- mock 后端行为不变

---

## 方向 C：服务/dashboard 只读 Go2 监控

**代号：** `service-go2-monitor`  
**体量：** M · **真机：** 可选（fake 可演示 UI）

### 目标

把第十次 `robot_self_state`、第十一次技能审计**只读**接入主服务：状态页 / WS / `/api/status` 可见 sport_mode、error_code、速度、IMU、最近 skill 结果。

### 明确不做

- **不**把 teleop `drive` 或 Web 按住操控挂到 HTTP API  
- **不**让 LLM 经新 API 绕过 confirmation  

### 可选子集

- C1：仅 API + WS 字段扩展（S）  
- C2：dashboard 增加 Go2 状态卡片 + 急停（M）  
- C3：内嵌只读链接到现有 `run_unitree_teleop_web`（S，运维向）  

### 主要改动

- `service/runner.py`、`service/dashboard.py`
- `WorldState.snapshot()` 已在 LLM 路径可用，需确认 WS payload

### 验收

- `RDB_ROBOT=unitree RDB_PERCEPTION=unitree` 启动服务后，dashboard/WS 显示 `robot_self_state`
- 默认 mock 路径无回归

---

## 方向 D：WebRTC 感知流（LiDAR / 里程计 / 视频）

**代号：** `unitree-webrtc-perception-streams`  
**体量：** L · **真机：** 必须（至少一种流）

### 目标

对齐 [DimOS 连接文档](../dimos-go2-connection.md)：在 transport 或独立 perception 模块订阅 LiDAR、里程计、视频，转为 `Observation` 扩展字段或独立 topic。

### 分期建议

| 子阶段 | 内容 | 真机 |
|--------|------|------|
| D1 | 里程计 / sport 位姿增强（仍用现有 state） | 可选 |
| D2 | LiDAR 点云 → `detected_objects` 或 payload | 必须 |
| D3 | 视频帧 → 缓存/抽样，不接 VLM | 必须 |
| D4 | 多模态 LLM（非本轮） | — |

### 风险

- WebRTC 视频 track 与现有连接稳定性（代码里已有 video recv 相关注释）
- 带宽与 CPU；需明确「认知层默认不消费全帧率视频」

### 验收

- 至少一条流在 fake/injected 下可测；真机一种流连续 60s

---

## 方向 E：Go2 技能族 II（approach / 条件 retreat）

**代号:** `go2-skill-family-v2`  
**体量：** M · **真机：** 可选（approach 需感知目标）

### 目标

在 nudge/scan/retreat 之上增加**有前提**的运动技能。

| 技能 | 行为 | 前提 |
|------|------|------|
| **approach** | 朝 `known_objects` 中某 id 短距 nudge（1–2 段） | 对象 fresh（TTL）、距离可估 |
| **retreat_from** | 背离某对象后退 | 同上 |
| **hold** | 无运动，仅 report 当前 self_state | 调试/确认用 |

### 约束

- 仍不实现 follow / 路径规划 / 避障  
- 距离仍受分段 drive 与 max_drive_duration 限制  

### 依赖

- 方向 B（避免 LLM 仍选 navigate）  
- 若有真实 LiDAR/检测（方向 D）价值更大；否则仅用 mock `known_objects` 也可测  

---

## 方向 F：主服务 + Go2 任务闭环（fake 端到端）

**代号:** `service-go2-task-e2e`  
**体量：** S–M · **真机：** 不需要（dry-run/fake）

### 目标

证明「提交任务 → perceive → LLM/mock 规划 → confirm → nudge → 审计落库/WS」全链路在 **unitree + fake** 下可重复跑通。

### 范围

- 示例脚本或 integration test：POST `/api/tasks`，objective 触发 nudge  
- 确认 `/api/tasks/{id}/confirm` 与 `require_confirmation_for` 联动  
- 任务摘要 / execution summary 含 Go2 skill `data.segments`  

### 价值

- 集成测试文档化，真机来时只换 transport  
- 不新加业务能力，偏工程质量  

---

## 方向 G：认知层消费 self_state（Planner / Prompt）

**代号:** `planner-self-state-context`  
**体量：** S · **真机：** 不需要

### 目标

LLM 规划时**结构化**看到 Go2 状态，减少「未站立仍 nudge」类决策。

### 范围

- Planner system prompt / world snapshot 增加 `robot_self_state` 摘要字段说明  
- mock LLM 测试：standing=false 时不输出 nudge  
- 与方向 A 分工：A=硬规则拦截，G=软提示  

### 非目标

- 不替换 Validator / FastReflex  

---

## 方向 H：真机验收与运维包（现场就绪）

**代号:** `go2-field-acceptance`  
**体量：** S（文档+脚本）· **真机：** 必须

### 目标

在能接触 Go2 时，**一次性**跑完积压验收并归档。

### 清单

| 项 | 来源 | 产出 |
|----|------|------|
| 分级 Level 0–5 | 第九次 | `--graded --output-json acceptance.json` |
| Perception smoke | 第十次 | 服务启动后 `robot_self_state` 非空 |
| LLM nudge live | 第十一次 | confirm 后短 nudge + 审计 |
| Web teleop 回归 | 第九次 E | 已有记录，可选复测 |

### 无真机时可做

- dry-run 脚本生成「预验收报告」模板  
- CI 只跑 fake 部分  

---

## 方向 I：姿态恢复与安全姿态（操作者 / FastReflex 专用）

**代号:** `go2-posture-recovery`  
**体量：** M · **真机：** 必须  
**优先级：** 低（除非现场频繁 lie down）

### 目标

`recovery_stand` / `damp` / `sit` 仅 **FastReflex 或 CLI**，不注册 LLM skill；用于 error 或人工恢复流程。

### 注意

- 与第九次「LLM 不控制姿态」一致  
- 需与 motion gate、prep 序列文档对齐  

---

## 方向 J：SQLite / 记忆与 Go2 审计持久化

**代号:** `audit-persistence-go2`  
**体量：** S · **真机：** 不需要

### 目标

将 `SkillResult.data`（segments、end_reason）写入 execution summary 或 experiences，便于事后追溯。

### 范围

- `memory/execution_summary.py` 扩展字段  
- 查询 API 或 dashboard 展示最近 Go2 动作  

---

## 对比矩阵

| 方向 | 体量 | 真机 | 直接用户价值 | 与现有缺口匹配 |
|------|------|------|--------------|----------------|
| **A** FastReflex Go2 | M | 可选 | 高（安全） | ★★★★★ |
| **B** Tool 过滤 | S | 否 | 高（防误用） | ★★★★☆ |
| **C** 服务监控 | M | 可选 | 中（可观测） | ★★★★☆ |
| **D** 感知流 | L | 必须 | 高（长期） | ★★★☆☆ |
| **E** 技能族 II | M | 可选 | 中 | ★★★☆☆ |
| **F** 服务 E2E | S–M | 否 | 中（集成） | ★★★☆☆ |
| **G** Planner 上下文 | S | 否 | 中 | ★★★☆☆ |
| **H** 真机验收包 | S | 必须 | 高（现场） | ★★★★★ |
| **I** 姿态恢复 | M | 必须 | 低–中 | ★★☆☆☆ |
| **J** 审计持久化 | S | 否 | 低–中 | ★★☆☆☆ |

---

## 建议的第十二次迭代候选（三选一）

### 候选 1：A + B（推荐，无真机可闭环）— **已选定 → [第十二次迭代](./2026-06-15-120000-go2-fast-reflex-and-tool-filter.md)**

**主题：** Go2 快反 + LLM 工具安全  
**理由：** 第十一次刚开放 LLM 运动，先补 FastReflex 与 tool 过滤，风险收益比最高。  
**交付：** 新规则测试 + unitree 后端 tool 列表收敛。

### 候选 2：F + G + J（工程整合）

**主题：** 服务链路可演示、可审计  
**理由：** 适合对外 demo 或写进 README 的「一条命令跑通 Go2 Agent」。  
**交付：** integration test、prompt 更新、summary 存 segments。

### 候选 3：C1 + B（可观测 + 安全）

**主题：** 主服务能看见 Go2，且 LLM 不会误选 navigate  
**理由：** 运维/调试友好，不改执行层。  
**交付：** `/api/status` + WS 含 self_state；tool filter。

---

## 选定后如何开新计划

1. 从本文复制对应章节，新建 `docs/plans/YYYY-MM-DD-HHMMSS-<topic>.md`（用 [迭代模板](./iteration-template.md)）  
2. 在 [plans/README](./README.md) 登记  
3. 更新上一迭代文档「下一步」链接  

---

## 复盘

（选定方向并实施后再填）
