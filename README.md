# Robot Brain

一个面向机器人的 Python L3 认知层骨架。项目将感知、规划、安全校验、技能执行、记忆和任务调度拆成可替换模块，并提供本地 FastAPI 服务与轻量状态控制台。

当前默认使用 mock 机器人、mock 感知和离线规则规划器，因此不连接真机或外部模型也可以完整运行。

## 当前能力

- 快慢双系统：低电量、急停和关键告警优先走快速规则，复杂目标交给规划器。
- 技能注册表：内置导航、巡逻、识别、跟随、回充、上报和停止技能。
- 安全校验：技能白名单、参数校验、速度和距离限制、目标新鲜度检查、人工确认。
- SQLite 持久化：会话消息、长期经验、checkpoint、世界状态快照和调度任务。
- 任务调度：优先级队列、取消、有限重试、重启恢复、告警优先、自动回充。
- 对象生命周期：感知对象记录最后出现时间，并按 TTL 清理陈旧对象。
- 服务化：后台调度循环、HTTP API、WebSocket 状态流和内嵌状态页面。

## 快速开始

项目需要 Python 3.10 或更高版本。

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e .
python examples\run_service.py
```

浏览器打开：

```text
http://127.0.0.1:8000
```

也可以在安装后直接运行：

```powershell
robot-brain-service
```

## 示例

运行单次认知循环示例：

```powershell
python examples\run_demo.py
```

运行任务调度示例：

```powershell
python examples\run_scheduler_demo.py
```

运行本地服务：

```powershell
python examples\run_service.py
```

### Unitree Go2（WebRTC 实机操控）

项目已支持通过 WebRTC（LocalSTA / 同 LAN）连接宇树 Go2，完成姿态、急停和低速限时速度操控。**默认仍为 mock/fake，不会连接真机或下发动作。**

依赖（可选）：

```bash
pip install -e ".[unitree]"   # unitree-webrtc-connect 等
```

常用环境变量见 [`docs/unitree-setup.md`](./docs/unitree-setup.md)。最小实机配置：

```bash
export RDB_UNITREE_ROBOT_IP=10.10.x.x    # 或 DIMOS_ROBOT_IP / ROBOT_IP
export RDB_UNITREE_ENABLE_MOTION=true    # 硬安全门，默认 false
# 新固件可能还需要：export UNITREE_AES_128_KEY=<32-hex>
```

操作者入口（**不接入 LLM / 服务 API**）：

| 脚本 | 用途 |
| --- | --- |
| `examples/run_unitree_smoke.py` | 只读 / 姿态 / 短时 drive / 分级验收（Level 0–5） |
| `examples/run_unitree_teleop.py` | 终端离散 nudge + 急停 |
| `examples/run_unitree_teleop_web.py` | 浏览器按住操控（车式键位：W/S 前后，A/D 转弯，Q/E 平移） |

Web 面板示例（真机会移动，需确认短语）：

```bash
RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_unitree_teleop_web \
    --transport webrtc --live --strong
```

分级真机验收（Level 0–5 逐级：只读 → 急停 → 姿态 → 旋转 → 直线 → 运动中急停）。每级记录前后状态与动作审计，可用 `--output-json` 导出完整摘要：

```bash
RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_unitree_smoke \
    --transport webrtc --graded --live --level 5 \
    --output-json acceptance.json
```

调试时可缩短 Level 0 只读观察时长，例如 `--level0-seconds 10`（正式验收默认 60 秒）。任一级失败会立即停止，不会进入下一级。

安全要点：`RDB_UNITREE_DRY_RUN=true`（默认）、`RDB_UNITREE_ENABLE_MOTION=false`（默认）；真实平移需同时 `--live` 与 motion gate。MCF 固件上前进/侧移走 Move(1008)，含转向（含 W+D 弧线）走虚拟摇杆通道，详见第九次迭代文档。

**感知桥接（第十次迭代）：** 设置 `RDB_PERCEPTION=unitree` 后，Go2 的真实 sport mode、error_code、速度、IMU 姿态、站立/运动状态会通过 `UnitreePerceptionAdapter` 注入 `WorldState.robot_self_state`，进入认知链路（FastReflex / LLM / 技能均可读取）。不影响默认 mock 路径。真机验证方式：

```bash
RDB_ROBOT=unitree RDB_UNITREE_TRANSPORT=webrtc RDB_PERCEPTION=unitree \
RDB_UNITREE_DRY_RUN=true RDB_UNITREE_ROBOT_IP=<ip> \
python -m examples.run_service
```

**LLM 可调用 Go2 技能（第十一次迭代）：** 设置 `RDB_ROBOT=unitree` 后，以下运动技能自动注册，LLM 可通过 tool call 调用。每个技能走完整安全链路（前置检查 → 分段 drive → post-verify），速度和距离受限，live 默认需操作者确认。

| 技能 | 说明 | 范围 |
|------|------|------|
| `nudge` | 短距平移 (forward/back/left/right) | 10–50 cm |
| `scan` | 原地旋转观察 (±yaw_degrees) | ±90° |
| `retreat` | 后退安全距离 | 10–100 cm |

**依赖**：技能前置检查读取 `WorldState.robot_self_state`，因此须同时设置 `RDB_PERCEPTION=unitree`，否则技能会返回 `self-state not available`。

```bash
# mock 下体验 (dry-run, 不动作)
RDB_ROBOT=unitree RDB_PERCEPTION=unitree python -m examples.run_service
```

## API

| Method | Path | Description |
| --- | --- | --- |
| `GET` | `/health` | 健康检查 |
| `GET` | `/api/status` | 查询服务、世界状态和任务 |
| `GET` | `/api/tasks` | 查询任务列表 |
| `GET` | `/api/tasks/{task_id}` | 查询单个任务 |
| `POST` | `/api/tasks` | 提交任务 |
| `DELETE` | `/api/tasks/{task_id}` | 取消任务 |
| `POST` | `/api/tasks/{task_id}/confirm` | 确认或拒绝待确认任务 |
| `POST` | `/api/events` | 发送命令、告警或急停事件 |
| `POST` | `/api/estop/reset` | 重置急停状态 |
| `WS` | `/ws` | 接收状态快照和调度事件 |

提交任务示例：

```powershell
Invoke-RestMethod `
  -Uri "http://127.0.0.1:8000/api/tasks" `
  -Method Post `
  -ContentType "application/json" `
  -Body '{"objective":"patrol the lobby","priority":0}'
```

发送急停：

```powershell
Invoke-RestMethod `
  -Uri "http://127.0.0.1:8000/api/events" `
  -Method Post `
  -ContentType "application/json" `
  -Body '{"type":"interrupt","message":"operator stop"}'
```

## SQLite 存储

默认数据库会在首次创建 runtime 时自动生成：

```text
data/robot_brain.sqlite3
```

可以通过环境变量覆盖路径：

```powershell
$env:RDB_MEMORY_DB = "D:\robot-data\robot_brain.sqlite3"
```

数据库包含：

- `threads`
- `messages`
- `experiences`
- `checkpoints`
- `world_state_snapshots`
- `scheduled_tasks`

## 配置

主要配置位于 `config/settings.py`。

| Environment Variable | Default | Description |
| --- | --- | --- |
| `RDB_LLM` | `mock` | LLM 后端：`mock` / `openai`（Responses API）/ `compatible`（Chat Completions） |
| `RDB_ROBOT` | `mock` | 机器人后端 |
| `RDB_PERCEPTION` | `mock` | 感知后端，支持 `mock` / `unitree`（Go2 本体状态桥接） |
| `RDB_MEMORY_DB` | `data/robot_brain.sqlite3` | SQLite 路径 |
| `RDB_VERBOSE` | `true` | 是否启用详细日志 |
| `RDB_OPENAI_MODEL` | `gpt-4o-mini` | 可选 OpenAI 适配器模型 |
| `RDB_UNITREE_TRANSPORT` | `fake` | Unitree 传输层：`fake` / `sdk` / `webrtc` |
| `RDB_UNITREE_ROBOT_IP` | 空 | Go2 WebRTC LAN IP（亦读 `DIMOS_ROBOT_IP` / `ROBOT_IP`） |
| `RDB_UNITREE_DRY_RUN` | `true` | `false` 才允许向真机下发（仍受 motion gate 约束） |
| `RDB_UNITREE_ENABLE_MOTION` | `false` | 真实姿态/平移硬安全门；`stop` 在已连接时始终允许 |
| `RDB_VLM_ENABLED` | `false` | 本地 VLM 可通行性 Hint 总开关（默认关，不影响 mock/CI） |
| `RDB_VLM_BASE_URL` | `http://10.10.197.175:8080` | 局域网 Qwen3-VL 服务地址 |
| `RDB_VLM_MODEL` | `/Users/dijia/models/Qwen3-VL-8B-4bit` | 服务端 model 字段 |
| `RDB_VLM_CONFIDENCE_MIN` | `0.5` | 低于此置信度忽略 Hint，走规则 fallback |
| `RDB_VLM_MIN_INTERVAL` | `2.0` | explore 内两次 VLM 调用最小间隔（秒） |
| `RDB_VLM_TIMEOUT` | `30` | VLM 请求超时（秒）；超时回退规则 |
| `RDB_VLM_FRAME_PATH` | 空 | mock/CI 静态图片路径（无真机相机时用） |

更多 Unitree 变量（限速、watchdog、Move/摇杆策略等）见 [`docs/unitree-setup.md`](./docs/unitree-setup.md) 与 `config/settings.py`。

### LLM 后端详细配置

| 后端 | API 类型 | 适用场景 |
|------|----------|----------|
| `mock` | 无（规则匹配） | 开发/测试，无需 API Key |
| `openai` | OpenAI Responses API | OpenAI 官方模型（gpt-4o 等） |
| `compatible` | Chat Completions + tools | DeepSeek / Ollama / vLLM / LM Studio 等 OpenAI 兼容服务 |

**DeepSeek 配置：**

```bash
export RDB_LLM=compatible
export OPENAI_BASE_URL=https://api.deepseek.com
export OPENAI_API_KEY=sk-...
export RDB_OPENAI_MODEL=deepseek-chat
```

**Ollama 本地配置：**

```bash
export RDB_LLM=compatible
export OPENAI_BASE_URL=http://127.0.0.1:11434/v1
export OPENAI_API_KEY=ollama
export RDB_OPENAI_MODEL=qwen2.5:7b
```

**推荐模型：**

| 模型 | Tool Calling | 备注 |
|------|:---:|------|
| deepseek-chat (V3) | ✅ | 国内部署、成本低 |
| gpt-4o / gpt-4o-mini | ✅ | 需 `openai` 后端（Responses API） |
| qwen2.5:7b+ | ✅ | Ollama 本地，支持 tool calling |
| llama3.1:8b+ | ⚠️ | tool calling 不稳定，建议 ≥70B |
| mistral-nemo | ⚠️ | 需实测 |

### VLM 可通行性 Hint（第十七次迭代）

`explore` 技能可接入本地 Qwen3-VL（OpenAI 兼容 multimodal）作为**只读语义传感器**，在换向时给出软方向建议。**超声波仍是硬安全门**，VLM 不能 override；失败/超时/低置信度自动回退第十六次规则（固定 +90° 或超声波 left/right）。默认关闭，不影响 mock/CI。

> 安装依赖：`pip install -e ".[vlm]"`（`httpx` + `Pillow`）。Go2 真机抽帧还需 `unitree-webrtc` extra（带 `av`）；开发装 `.[dev]` 已含全部测试依赖。

```bash
export RDB_VLM_ENABLED=true
export RDB_VLM_BASE_URL=http://10.10.197.175:8080
export RDB_VLM_MODEL=/Users/dijia/models/Qwen3-VL-8B-4bit
# 可选：mock/CI 用静态图片代替真机视频
export RDB_VLM_FRAME_PATH=/tmp/front.jpg
```

- 连通性自测：`python -m examples.vlm_passability_smoke --image path/to/front.jpg`
- 真机抽帧（unitree+webrtc）：`AgentRuntime.attach_passability_tap(conn)` 在 `await conn.connect()` 后注册 Go2 视频抽帧。注意 aiortc 单消费者：与 RTP relay 同 track 并存需 tee（后续工作）。
- 设计原则：VLM 与文本 LLM 分客户端；Hint 不入 LLM tool list、不直接 drive；JSON 输出 + 本地校验 + 三层兜底（限频/超时/置信度）。详见 [迭代计划](./docs/plans/2026-06-24-170000-vlm-passability-hint.md)。

**第十八次迭代（explore 可验收闭环）：** explore 每步输出结构化 `trace`（传感器/VLM 建议/决策原因），新增 `no_progress`/`semantic_hold`/`ping_pong` 停止保护（`RDB_EXPLORE_NO_PROGRESS_STEPS=3` / `RDB_EXPLORE_PING_PONG_STEPS=4` / `RDB_EXPLORE_MAX_HOLDS=2`）；VLM 生命周期收口（`AgentRuntime.aclose()`）；`/api/status` 暴露 `vlm`/`explore` 诊断；frame source 可配（`RDB_VLM_FRAME_SOURCE=auto` / `RDB_VLM_VIDEO_PRIORITY=vlm`）。现场验收：

```bash
python -m examples.run_explore_acceptance --mode mock --output-json acceptance-mock.json
python -m examples.run_explore_acceptance --mode unitree-fake --output-json acceptance-fake.json
# 测停止保护（no_progress）：--scenario blocked 触发，result=aborted，退出码 1
python -m examples.run_explore_acceptance --scenario blocked --output-json acceptance-protection.json
```

项目测试基于 `pytest`（部分用例使用 `pytest-asyncio`）：

```bash
python -m pytest tests/ -q
python -m compileall -q robot_brain config tests examples
```

## 目录结构

```text
robot_brain/
  core/             世界状态、事件和调度任务模型
  perception/       感知接口与 mock
  cognition/        快慢双系统与规划器
  skills/           技能定义、注册表和内置技能
  safety/           安全校验与独立急停
  memory/           SQLite、短期记忆、经验、checkpoint 和任务队列
  orchestration/    LangGraph 决策循环
  runtime/          单次 runtime 与持久化 scheduler
  service/          后台服务、HTTP API、WebSocket 与状态页面
  actuation/        机器人适配层（含 Unitree Go2 WebRTC / SDK）
examples/
  run_unitree_*.py  Go2 操作者 smoke / teleop（不接入 Agent）
docs/
  unitree-setup.md  Go2 连接、环境变量与实机验收
  plans/            架构与迭代记录
```

## 技能列表

| 技能 | 说明 | 后端 |
|------|------|------|
| `navigate` | 导航到坐标 | mock |
| `patrol` | 巡逻多个路点 | mock |
| `nudge` | 短距移动 (10–50cm) | mock / unitree |
| `scan` | 原地旋转 (±90°) | mock / unitree |
| `retreat` | 后退 (10–100cm) | mock / unitree |
| `explore` | **有限步探索** — 循环 scan/nudge/retreat，带硬停止条件 | mock / unitree |
| `recognize` | 查询已知物体 | all |
| `report` | 发送状态报告 | all |
| `stop` | 立即停止 | all |

### explore 技能

组合技能，规则驱动循环：scan → 读障碍 → 换向/nudge/retreat。

**参数：**
- `max_steps`: 最大循环次数 (1–20, Validator 限 `RDB_EXPLORE_MAX_STEPS`, 默认 5)
- `step_distance_cm`: 单步距离 (10–50cm, 默认 20)
- `scan_degrees`: 每步扫描角度 (10–90°, 默认 45)
- `report_every`: 每 N 步记录一次 (默认 2)

**停止条件（硬规则）：** max_steps / max_duration / 低电量 / 急停 / 机器人错误 / 数据 stale / 四面堵

**环境变量：**

| 变量 | 默认 | 说明 |
|------|------|------|
| `RDB_EXPLORE_MAX_STEPS` | `5` | Validator 硬顶 |
| `RDB_EXPLORE_MAX_DURATION` | `120` | 单次最长秒数 |
| `RDB_EXPLORE_STEP_CM` | `20` | 默认单步距离 |
| `RDB_EXPLORE_SCAN_DEG` | `45` | 默认扫描角度 |

## 当前边界

- 默认服务仅监听 `127.0.0.1:8000`，适用于本机可信环境。
- 尚未加入身份认证、HTTPS、schema migration 和数据清理策略。
- Unitree Go2 已支持 WebRTC 低速操控与操作者 teleop，但**未**接入主服务 API、LLM 技能或自主导航；SDK transport 仍为只读。
- 当前急停入口可以立即触发，但无法强制中断阻塞中的厂商 SDK 调用。

更详细的架构与迭代记录见 [`docs/plans/`](./docs/plans/README.md)。
