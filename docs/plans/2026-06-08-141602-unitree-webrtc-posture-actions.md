# 第八次迭代：WebRTC 真实姿态/急停动作下发与安全门

## 基本信息

- 创建时间：2026-06-08 14:16:02 CST
- 文件序号：2026-06-08-141602
- 状态：已完成（transport send_command + adapter set_posture + smoke 姿态序列 + 单测）；真机姿态验证待手动执行
- 负责人：dijia
- 前置迭代：[第七次迭代：真实 Unitree SDK Transport 接入与只读实机验证](./2026-06-08-101349-unitree-sdk-transport-readonly.md)

## Project Requirements

### Goal

在第七次迭代完成只读实机验证（WebRTC STA-L 模式成功读取 Go2 状态）的基础上，开放**第一类真实动作下发**：仅限**不产生平移的姿态/急停命令**（StandUp、StandDown、Sit、BalanceStand、RecoveryStand、Damp、StopMove）。

本轮目标是验证"软件能安全地让真机改变姿态并急停"，平移（Move/Turn）仍然拒绝，留到下一轮专门处理速度语义和短步限制。每轮只引入一类可控的物理风险。

### Features

- WebRTC transport 的 `send_command` 把 `UnitreeCommand.action` 映射到 Go2 sport API（`rt/api/sport/request` + `SPORT_CMD` api_id）。
- 新增硬安全门 `unitree_enable_motion`（`RDB_UNITREE_ENABLE_MOTION`，默认 false）：即使 `dry_run=False`，没开此门也拒绝下发；`stop`（StopMove）作为安全命令永远放行。
- 平移类命令（move/turn）在 WebRTC transport 显式拒绝（`NotImplementedError`）。
- `UnitreeRobot` 新增 `set_posture(posture)`：dry-run 门 + 历史记录 + 失败兜底 stop + 重抛。
- smoke test 新增 webrtc 姿态序列（BalanceStand → StandDown → RecoveryStand → BalanceStand），需要 `--live` + `enable_motion` + 终端确认短语。
- 注入 fake conn 的单测覆盖每个姿态映射、enable_motion 门、平移拒绝、未连接、下发异常、adapter 集成。

### Constraints

- 本轮不开放真实平移、转向、dock、follow。
- 本轮不让 LLM 参与真机动作。
- `RDB_ROBOT=mock` 仍为默认；`unitree` 后端仍需显式开启；`RDB_UNITREE_TRANSPORT` 默认 `fake`。
- `RDB_UNITREE_DRY_RUN` 默认 true；`RDB_UNITREE_ENABLE_MOTION` 默认 false。两道门同时打开才会真实下发非急停命令。
- 命令路径不硬依赖真实 WebRTC 库：sport api_id 与 topic 在本模块内硬编码，注入 conn 即可测试。
- 不在 `skills`、`cognition`、`orchestration` 中引用 transport 细节。

### Success Criteria

- WebRTC transport 能把姿态/急停命令正确映射并通过 data channel 下发。
- enable_motion 关闭时，除 stop 外的命令被拒绝且不下发。
- 平移命令始终被拒绝。
- 默认 fake/mock 行为与既有测试不受影响。
- 单测覆盖映射、门控、拒绝、异常、adapter 集成。

## 推荐实施方案

### 命令映射

| action | SPORT_CMD | api_id | 平移 |
|--------|-----------|--------|------|
| stop | StopMove | 1003 | 否（急停，永远放行） |
| balance_stand | BalanceStand | 1002 | 否 |
| stand_up | StandUp | 1004 | 否 |
| stand_down | StandDown | 1005 | 否 |
| recovery_stand | RecoveryStand | 1006 | 否 |
| damp | Damp | 1001 | 否 |
| sit | Sit | 1009 | 否 |
| move / turn | —— | —— | 是（拒绝） |

下发方式：`conn.datachannel.pub_sub.publish_request_new("rt/api/sport/request", {"api_id": ...})`。
后台事件循环存在时通过 `run_coroutine_threadsafe` 调度并 `asyncio.to_thread` 等待，避免阻塞调用方循环；注入 conn 时直接 await。

### 两道安全门

1. adapter 层 `dry_run`：true 时 `set_posture` / `stop` 只记录不下发。
2. transport 层 `enable_motion`：false 时拒绝除 stop 外的所有命令。

### Smoke Test

```text
# 只读（无需任何门）
python -m examples.run_unitree_smoke --state-only --transport webrtc

# 真实姿态序列（两道门 + 确认短语）
RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_unitree_smoke \
    --transport webrtc --actions --live
```

## 分阶段计划

### 阶段 A：配置与命令映射 ✓

- [x] 新增 `RDB_UNITREE_ENABLE_MOTION` 配置
- [x] transport 内硬编码 sport topic + api_id 映射

### 阶段 B：transport send_command ✓

- [x] 姿态/急停映射并下发
- [x] enable_motion 门（stop 例外）
- [x] 平移类拒绝
- [x] 后台循环调度 + 注入 conn 直接 await

### 阶段 C：adapter set_posture ✓

- [x] dry-run 门 + 历史记录
- [x] 失败兜底 stop + 重抛
- [x] 非法姿态 ValueError

### 阶段 D：smoke test + 单测 ✓

- [x] webrtc 姿态序列（确认短语 + live + enable_motion）
- [x] 15 个 webrtc transport 单测

### 阶段 E：真机姿态验证（待手动执行）

- [ ] 平地、空旷、可断电环境下执行姿态序列
- [ ] 记录输出与实际动作是否一致，补进复盘

## 风险与缓解

| 风险 | 缓解 |
|------|------|
| 误触发真机动作 | 两道门（dry_run + enable_motion）默认全关 + 确认短语 |
| stop 被门挡住导致无法急停 | stop 绕过 enable_motion 门，永远放行 |
| 平移语义与路点不一致 | 本轮直接拒绝 move/turn，留到下一轮 |
| 下发阻塞调用方循环 | run_coroutine_threadsafe + asyncio.to_thread |
| 测试依赖真实库 | api_id/topic 硬编码 + 注入 fake conn |

## 验证结果

- 155 tests, all pass（原 140 + 新 15）
- `python -m compileall` 通过
- unitree 相关 60 测试全过
- 默认 fake/mock 行为不变

## 遗留

- 真机姿态序列需在安全环境手动执行验证
- 下一步（第九次迭代）：低速短步平移 —— 把 Move(1008) 的速度语义接入，配合时长/限速/限步与急停闭环
