# 第三次迭代：常驻任务调度与对象生命周期

## 基本信息

- 日期：2026-06-02
- 状态：已完成
- 负责人：待定

## 背景

前两次迭代已经完成会话、长期经验、checkpoint 和世界状态快照的 SQLite 持久化。但 runtime 仍然以单次 `run_command()` 为主要入口，缺少常驻 Agent 所需的任务队列；同时，感知对象只会持续累积，没有过期机制。

如果继续扩展技能，会出现两个基础问题：

- 普通任务、告警任务和低电量回充无法统一调度。
- 机器人可能基于很久以前见过的对象继续执行跟随等动作。

因此，本次迭代优先建立可恢复的任务调度层，并补齐感知对象生命周期。

## 子迭代 A：对象生命周期

### 目标

- [x] 为感知对象增加 `last_seen_at`。
- [x] 为感知结果增加 `observed_at`。
- [x] 增加可配置 TTL，并在每次感知后清理陈旧对象。
- [x] 跟随安全校验只允许最近仍可确认存在的目标。
- [x] 世界状态快照继续兼容对象时间信息。

### 取舍

- 第一版使用固定 TTL，不做复杂轨迹预测。
- 新鲜感知覆盖历史状态；未再次出现的对象在 TTL 到期后删除。
- 恢复旧快照时，缺少 `last_seen_at` 的对象视为不可信。

## 子迭代 B：持久化任务队列

### 目标

- [x] 定义任务模型：目标、优先级、状态、重试次数、来源和 `thread_id`。
- [x] 增加 SQLite `scheduled_tasks` 表和内存版任务存储。
- [x] 支持提交、查询、取消和重启后恢复未完成任务。
- [x] 高优先级任务先于普通任务执行。
- [x] 失败任务在配置范围内有限重试。

### 任务状态

- `queued`：等待调度。
- `running`：正在执行。
- `awaiting_confirmation`：等待人工确认。
- `paused`：被急停打断，等待恢复。
- `completed`：执行完成。
- `failed`：达到重试上限或不可继续。
- `cancelled`：由调用方取消。

## 子迭代 C：常驻调度入口

### 目标

- [x] 新增 scheduler，保留现有 `run_command()` 作为底层单次执行接口。
- [x] 调度前刷新感知，低电量时优先自动回充，不消费普通任务。
- [x] `COMMAND` 事件进入普通任务队列。
- [x] `WARNING` 事件进入高优先级队列，优先于普通任务。
- [x] `INTERRUPT` 事件立即触发急停，并暂停正在运行的任务。
- [x] 支持调度任务的人工确认恢复。

## 验证方式

- [x] 自动化测试：陈旧对象会过期，跟随校验拒绝陈旧目标。
- [x] 自动化测试：任务队列按优先级执行并跨 runtime 恢复。
- [x] 自动化测试：取消任务后不会执行。
- [x] 自动化测试：失败任务有限重试，达到上限后标记失败。
- [x] 自动化测试：低电量先自动回充，普通任务保留到下一轮。
- [x] 自动化测试：告警任务优先于普通任务。
- [x] 自动化测试：急停事件立即执行并阻止后续调度。
- [x] 自动化测试：完整回归测试通过。
- [x] 手动验证：运行一个组合场景，确认 SQLite 中产生任务记录。

## 复盘

已完成对象生命周期和持久化调度闭环：

- `DetectedObject` 增加 `last_seen_at`，`Observation` 增加 `observed_at`。
- `WorldState` 会在感知后按 `object_ttl_seconds` 清理陈旧对象；跟随校验也会独立拒绝陈旧目标。
- 增加 `ScheduledTask`、`TaskStatus`、`TaskQueue` 和内存版任务存储。
- SQLite 增加 `scheduled_tasks` 表，任务可跨 runtime 恢复；异常退出时遗留的 `running` 任务会重新排队。
- 增加 `AgentScheduler`，支持提交、查询、取消、优先级、有限重试、人工确认恢复和急停重置。
- 调度前会刷新感知。低电量时优先自动回充，不会消费普通任务。
- `WARNING` 事件会规范化为高优先级告警上报任务，`COMMAND` 事件进入普通任务队列，`INTERRUPT` 事件立即急停。
- 增加 `examples/run_scheduler_demo.py`，可直接观察调度顺序和动作历史。

验证结果：

- `python -m unittest discover -s tests -v`：25 个测试全部通过。
- `python -m compileall -q robot_brain config tests examples`：通过。
- `git diff --check`：通过。
- 手动组合场景：调度顺序为 `auto_recharge -> completed warning -> completed patrol`，动作顺序为 `dock -> report -> move_to -> move_to`。
- 手动组合场景：SQLite 中两个任务最终均为 `completed`，并写入 22 个世界状态快照。

遗留问题：

- scheduler 当前提供轮询式 `run_next()` / `run_until_idle()`，还没有独立后台进程和优雅停机机制。
- 急停可以立即通过事件入口执行，但单进程同步技能执行期间还不能真正抢占正在等待中的 SDK 调用。
- 对象生命周期目前只有固定 TTL，没有置信度衰减、目标轨迹和区域级规则。
- SQLite 仍需要 schema migration、快照清理和任务归档策略。
