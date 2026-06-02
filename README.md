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
| `RDB_LLM` | `mock` | LLM 后端，当前支持 `mock` 和可选 `openai` |
| `RDB_ROBOT` | `mock` | 机器人后端 |
| `RDB_PERCEPTION` | `mock` | 感知后端 |
| `RDB_MEMORY_DB` | `data/robot_brain.sqlite3` | SQLite 路径 |
| `RDB_VERBOSE` | `true` | 是否启用详细日志 |
| `RDB_OPENAI_MODEL` | `gpt-4o-mini` | 可选 OpenAI 适配器模型 |

## 测试

项目测试基于标准库 `unittest`，无需额外测试框架即可运行：

```powershell
python -m unittest discover -s tests -v
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
```

## 当前边界

- 默认服务仅监听 `127.0.0.1:8000`，适用于本机可信环境。
- 尚未加入身份认证、HTTPS、schema migration 和数据清理策略。
- 真机 SDK、ROS2 和真实传感器适配器仍需根据硬件型号实现。
- 当前急停入口可以立即触发，但无法强制中断阻塞中的厂商 SDK 调用。

更详细的架构与迭代记录见 [`docs/plans/`](./docs/plans/README.md)。
