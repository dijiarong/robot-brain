# 第十八次迭代：Explore 现场可验收闭环

## 基本信息

- 创建时间：2026-07-11 20:45:45 CST
- 文件序号：2026-07-11-204545
- 状态：阶段 A 代码/测试完成（mock/fake 可验收）；阶段 B 真机联调待现场
- 负责人：dijia
- 真机依赖：主迭代不依赖真机；现场验收项需要 Go2
- 前置完成：[第十六次 Bounded Explore](./2026-06-24-160000-bounded-explore-mode.md) · [第十七次 本地 VLM 可通行性 Hint](./2026-06-24-170000-vlm-passability-hint.md) · [能力底座契约](./2026-07-10-170002-capability-foundation-tool-skill-contract.md)

## 背景

第十七次迭代已经让 `explore` 可以接入本地 VLM，基于前视图像输出 `PassabilityHint`，并保持“超声波硬安全、VLM 软建议”的边界。当前系统已经从“规则探索”升级到“规则 + 视觉 Hint 探索”，但还缺少现场可验收能力：

- `explore` 的 `actions` 仍是字符串列表，现场复盘时很难知道每一步为什么这么做
- VLM client / frame tap 的生命周期还没有统一关闭，常驻服务存在资源清理缺口
- 真机视频 tap 需要手动接线，`RDB_VLM_ENABLED=true` 还不等于主服务自动可用
- 还没有 `no_progress` / ping-pong 保护，连续 scan / retreat / hold 时缺少明确停止理由
- 服务状态里缺少 VLM / explore 的最近诊断信息，现场调参容易只能翻日志

本轮目标不是做 SLAM，也不是追求更聪明的视觉推理，而是把 `explore` 做成**可解释、可停止、可复盘、可现场验收**的闭环。

## 目标

### 阶段 A：不依赖真机，必须完成

- [x] VLM 生命周期收口：`PassabilityAnalyzer.aclose()` 关闭 client，停止 frame source 后台任务
- [x] `AgentRuntime.close()` 统一释放 VLM client / Go2 frame tap / 数据库资源
- [x] `explore` 输出结构化 step trace，替代或补充粗粒度字符串 `actions`
- [x] 增加 `no_progress` / ping-pong 保护，连续无有效前进时安全停止
- [x] 服务状态 API 暴露 VLM / passability / explore 最近诊断信息
- [x] 新增 mock / fake acceptance 脚本，输出可归档 JSON 报告
- [x] 保持 `RDB_VLM_ENABLED=false` 时行为与第十六次 / 第十七次默认路径兼容
- [x] 自动化测试覆盖，无需真机即可验收

### 阶段 B：需要真机，作为现场验收项

- [ ] Go2 WebRTC 连接后自动注册 VLM frame tap
- [ ] 验证真机前视帧能进入 `PassabilityAnalyzer`
- [x] 记录 VLM 延迟、frame age、hint 命中率和 fallback 原因
- [x] dry-run 跑通 `explore`，生成现场验收 JSON
- [ ] live gated 小步验证：超声波硬门、VLM stop、no_progress 均能生效

## 非目标

- 不做 SLAM / 栅格地图 / frontier exploration
- 不让 VLM 直接输出速度或绕过 `SafetyValidator`
- 不新增自由导航或长距离路径规划
- 不把 VLM 图像直接塞进 Planner prompt
- 不解决 video relay 与 VLM track tee 的最终形态，只做可配置优先级和清晰告警

## 核心设计

### 1. 生命周期收口

新增或扩展：

```text
PassabilityAnalyzer.aclose()
FrameSource.stop() / aclose()
AgentRuntime.close()
```

要求：

- `VLMClient.aclose()` 被调用，关闭内部 `httpx.AsyncClient`
- `Go2VideoFrameSource.stop()` 被调用，取消后台 drain task
- 多次 close 幂等
- VLM 关闭或未启用时 close 不报错

建议测试：

- fake client 记录 `aclose_called`
- fake frame source 记录 `stop_called`
- `AgentRuntime.create(vlm_enabled=True)` 后 `close()` 能释放 passability 资源

### 2. 自动接入 Go2 frame tap

当前已有 `AgentRuntime.attach_passability_tap(conn)`，但需要手动调用。本轮建议把它接入 Unitree WebRTC transport 创建 / connect 后的主路径。

可选策略：

| 策略 | 说明 |
|------|------|
| `vlm_only` | VLM 消费 video track，优先保证 passability hint |
| `relay_only` | 保持现有 RTP relay，不启用 VLM tap |
| `manual` | 只暴露 helper，不自动接入 |

配置建议：

| 变量 | 默认 | 说明 |
|------|------|------|
| `RDB_VLM_FRAME_SOURCE` | `auto` | `auto` / `file` / `go2_tap` / `none` |
| `RDB_VLM_VIDEO_PRIORITY` | `vlm` | `vlm` / `relay` / `manual` |

若无法安全 tee，同一条 video track 不应被两个消费者默默抢读。系统应在状态中明确显示当前模式。

### 3. Explore Step Trace

新增结构化 trace 模型，建议放在 `robot_brain/skills/builtin/explore.py` 或独立 `robot_brain/core/explore_trace.py`：

```python
class ExploreStepTrace(BaseModel):
    step_index: int
    heading_before: float | None = None
    heading_after: float | None = None
    ultrasonic: dict[str, float | None] = {}
    passability_hint: dict | None = None
    chosen_action: str
    fallback_reason: str = ""
    stop_reason: str | None = None
    segments: list[dict] = []
    duration_ms: float | None = None
```

`SkillResult.data` 建议保留兼容字段：

```text
actions: list[str]          # 兼容旧测试 / UI
trace: list[ExploreStepTrace]
stop_reason: str
steps_completed: int
```

trace 需要回答三个问题：

1. 当时传感器看到了什么？
2. VLM 给了什么建议，是否被采纳？
3. 最终为什么 scan / nudge / retreat / hold / stop？

### 4. no_progress / ping-pong 保护

本轮不做里程计建图，但要避免探索循环无意义振荡。

建议规则：

- 连续 `N` 步没有 `nudge` 成功，停止：`stop_reason=no_progress`
- 连续 `N` 次 `vlm_hold`，停止：`stop_reason=semantic_hold`
- 连续左右交替 `scan_alt_left` / `scan_alt_right` 超阈值，停止：`stop_reason=ping_pong`
- Go2 下若可读速度 / 位姿变化，后续再升级为真实位移判断；本轮先用行为 trace 判定

配置建议：

| 变量 | 默认 | 说明 |
|------|------|------|
| `RDB_EXPLORE_NO_PROGRESS_STEPS` | `3` | 连续无前进动作后停止 |
| `RDB_EXPLORE_PING_PONG_STEPS` | `4` | 左右交替扫描阈值 |
| `RDB_EXPLORE_MAX_HOLDS` | `2` | VLM 连续 stop / hold 阈值 |

### 5. 服务可观测

扩展 `/api/status` 或 service snapshot，至少包含：

```json
{
  "vlm": {
    "enabled": true,
    "frame_source": "go2_tap",
    "last_hint": {"recommended_direction": "left", "confidence": 0.82},
    "last_error": "",
    "last_latency_ms": 812,
    "last_frame_age_ms": 430
  },
  "explore": {
    "last_stop_reason": "no_progress",
    "last_steps_completed": 3,
    "last_trace_count": 3
  }
}
```

Dashboard 可选，本轮 API 优先。

### 6. Acceptance 脚本

新增：

```text
examples/run_explore_acceptance.py
```

模式：

| 模式 | 是否需要真机 | 说明 |
|------|--------------|------|
| `mock` | 否 | MockRobot + scripted world / hint |
| `unitree-fake` | 否 | Unitree fake transport + dry-run |
| `unitree-live` | 是 | 需要 motion gate + 人工确认 |

输出：

```json
{
  "mode": "unitree-fake",
  "vlm_enabled": true,
  "result": "completed",
  "stop_reason": "max_steps",
  "trace": [],
  "checks": {
    "ultrasonic_hard_gate": "passed",
    "vlm_fallback": "passed",
    "no_progress_stop": "passed"
  }
}
```

## 影响模块

| 模块 | 预计变化 |
|------|----------|
| `robot_brain/vlm/passability.py` | + lifecycle / diagnostics |
| `robot_brain/vlm/frame_source.py` | frame age、stop/aclose、tap 状态 |
| `robot_brain/runtime/loop.py` | close 资源释放、自动 tap 接入、status 字段 |
| `robot_brain/skills/builtin/explore.py` | step trace、no_progress、ping-pong stop |
| `robot_brain/service/runner.py` / `service/app.py` | status 输出扩展 |
| `examples/run_explore_acceptance.py` | 新增验收脚本 |
| `tests/test_explore_trace.py` | 新增 trace 测试 |
| `tests/test_explore_no_progress.py` | 新增停止保护测试 |
| `tests/test_vlm_lifecycle.py` | 新增资源释放测试 |

## 验证方式

### 自动化

- [x] `pytest tests/test_explore_vlm.py`
- [x] `pytest tests/test_passability_analyzer.py`
- [x] `pytest tests/test_explore_trace.py`
- [x] `pytest tests/test_explore_no_progress.py`
- [x] `pytest tests/test_vlm_lifecycle.py`
- [x] `pytest tests/test_service.py`
- [x] 全量测试通过
- [x] `python -m ruff check .`

### 手动 / fake

```bash
python -m examples.run_explore_acceptance --mode mock --output-json acceptance-mock.json
python -m examples.run_explore_acceptance --mode unitree-fake --output-json acceptance-fake.json
```

### 现场 / 真机

```bash
RDB_ROBOT=unitree \
RDB_PERCEPTION=unitree \
RDB_UNITREE_TRANSPORT=webrtc \
RDB_VLM_ENABLED=true \
RDB_UNITREE_DRY_RUN=true \
python -m examples.run_explore_acceptance --mode unitree-fake --output-json acceptance-go2-dry.json
```

live gated 仅在空旷、安全、人工急停可用时执行。

## 验收标准

- 不接真机时，mock / fake 可以完整生成 acceptance JSON
- `explore` 每一步都有结构化 trace，可解释决策原因
- VLM 失败、无帧、限流、低置信度均可安全 fallback，并体现在 trace 中
- 连续无进展会停止，stop reason 可区分 `no_progress` / `semantic_hold` / `ping_pong`
- runtime close 不泄露 VLM client 或 frame tap task
- `RDB_VLM_ENABLED=false` 默认路径无回归
- 真机不可用不阻塞代码完成；真机结果后续归档为 verification record

## 风险与取舍

| 风险 | 处理方式 |
|------|----------|
| trace 模型过重 | 先保留核心字段，复杂传感器原始数据只放摘要 |
| video relay 与 VLM 抢 track | 本轮明确优先级和状态告警，不做完整 tee |
| no_progress 误停 | 默认阈值保守，可配置；先宁愿停，不做无意义动作 |
| 现场 VLM 延迟高 | trace 记录 latency；超时 fallback；不阻塞硬安全 |

## 复盘

### 阶段 A 落地（代码/测试完成，不依赖真机）

- **生命周期收口**：`PassabilityAnalyzer.aclose()`（关 VLM client + stop frame source，幂等）；`FrameSource` 增 `kind`/`stop()`/`frame_age_ms`；`AgentRuntime.aclose()`（async，全量释放）+ `close()`（sync，无运行 loop 时 asyncio.run 兜底，有 loop 则告警并做同步部分）；`AgentService.stop()` 改 `await aclose()`。
- **Step Trace**：`ExploreStepTrace` 模型，mock/go2 两 loop 每步产 trace（heading before/after、ultrasonic、passability_hint、chosen_action、fallback_reason、stop_reason、segments、duration_ms）；`SkillResult.data` 同时保留 `actions`（兼容）+ `trace`。
- **停止保护**：`_ExploreGuard` 行为 trace 判定（无里程计）-- `no_progress`（连续 N 步无 nudge）、`semantic_hold`（连续 vlm_hold）、`ping_pong`（左右交替 scan_alt）。阈值可配（`RDB_EXPLORE_NO_PROGRESS_STEPS=3` / `PING_PONG_STEPS=4` / `MAX_HOLDS=2`）。
- **服务可观测**：`AgentRuntime.diagnostics()` 汇总 vlm+explore；`/api/status` 增 `vlm`（enabled/frame_source/last_hint/last_error/last_latency_ms/last_frame_age_ms/video_priority/video_warning）与 `explore`（last_stop_reason/last_steps_completed/last_trace_count）。
- **Acceptance 脚本**：`examples/run_explore_acceptance.py`，mock / unitree-fake / unitree-live 三模式，输出可归档 JSON（mode/vlm_enabled/result/stop_reason/trace/checks）。mock + unitree-fake 无真机可跑。
- **frame source 配置**：`RDB_VLM_FRAME_SOURCE`（auto/file/go2_tap/none）+ `RDB_VLM_VIDEO_PRIORITY`（vlm/relay/manual）；relay 优先级抑制 Go2 tap；status 在 tap 与 relay 抢读时显式告警（本轮不做 tee）。
- **测试**：新增 test_vlm_lifecycle(7) / test_explore_trace(5) / test_explore_no_progress(4)；全量 480 passed / 4 skipped；ruff 全清。`RDB_VLM_ENABLED=false` 默认路径无回归（no_progress/ping_pong 仅在命中时改 stop_reason，现有 max_steps/nudge 场景不触发）。

### 阶段 B（真机联调待现场）

- 代码可写部分已完成：frame source 配置、`attach_passability_tap(conn)` helper、status 模式与抢读告警、acceptance 脚本。
- **未完成（待现场）**：Go2 WebRTC connect 主路径自动注册 tap（需 transport connect hook，本轮无干净注入点，保留 helper + 文档）、真机前视帧进入 analyzer 验证、真机 dry-run 验收 JSON、live gated 小步验证。
- 现场验收后归档：VLM 平均延迟/失败率/fallback 分布、no_progress/ping_pong 是否过保守。

### 取舍

- `close()` 同步 + `aclose()` 异步双轨：service 走 aclose（不泄露 VLM client），tests/scripts 走 close（无 loop 时 asyncio.run 兜底）。
- no_progress 用行为 trace 判定（无里程计），阈值保守可配；先宁愿停。
- video relay vs VLM track：本轮只做优先级配置 + 状态告警，不做 tee（非目标）。
- 真机不可用不阻塞代码完成；真机结果后续归档为 verification record。

### 下一阶段候选

- track tee（relay + VLM 并存）、里程计进展判断（替代行为 trace 的 no_progress）、轻量地图记忆、`scan`/`retreat` 迁移到底层 motion tool（能力底座后续）。

### Review 修复（P1–P2）

- **P1（已修）unitree-live 误导**：原 `--mode unitree-live` 实际走 `_build_unitree_fake()`（硬编码 fake/dry-run/无 motion），会误导现场验收。已移除 `unitree-live` 选项，仅保留 `mock` / `unitree-fake`（均无真机）。真机 live 验证在现场用 service/runtime + live env（`RDB_UNITREE_ENABLE_MOTION=true`）直接跑，不进本脚本。
- **P2a（已修）默认验收命令退出码非 0**：原默认 `--front-m 0.15 --max-steps 4` 触发 no_progress -> `aborted` -> 退出 1。改为 `--scenario {clear,blocked}`（默认 clear，front 1.0，happy path -> `completed` -> 退出 0）；`--scenario blocked` 显式测停止保护。README 同步标注 blocked 为「保护触发示例，退出码 1」。
- **P2b（已修）Go2 trace segments 不完整**：原 scan segments 只入 `segments_total` 不入 trace；alt scan segments 既不入 trace 也不入 `segments_total`（丢失）。`ExploreStepTrace` 拆为 `scan_segments` / `alt_scan_segments` / `move_segments` 三段；`_decide_and_move_go2` 返回 alt+move segments；`segments_total = scan + alt + move`。新增 `test_go2_trace_records_all_segment_phases` 覆盖。
