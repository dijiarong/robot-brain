# 第九次迭代：Unitree Go2 WebRTC 实机操控安全闭环

## 基本信息

- 创建时间：2026-06-11 17:20:47 CST
- 文件序号：2026-06-11-172047
- 状态：阶段 A–E 代码完成，自动化测试通过；Level 0–5 分级验收 **待现场**（Web teleop 已单独实机验证通过）
- 负责人：dijia
- 前置迭代：[第八次迭代：WebRTC 真实姿态/急停动作下发与安全门](./2026-06-08-141602-unitree-webrtc-posture-actions.md)
- 技术参考：[DimOS 连接宇树 Go2 技术文档](../dimos-go2-connection.md)

## Project Requirements

### Goal

在现有 WebRTC 连接、状态读取、姿态命令和虚拟摇杆 `drive` 原语之上，完成一套可以在真实 Go2 上分级验收的**低速、限时、可抢占、可审计**操控闭环。

本轮不是开放自主导航，也不是让 LLM 直接控制机器狗。目标是先证明：操作者可以安全地下发一次短时速度指令，系统能持续发送控制帧、在超时或急停时可靠归零、验证动作前后状态，并在异常时停止后退出。

### Existing Baseline

当前仓库已经具备：

- `UnitreeWebRTCTransport`：通过 `LocalSTA` WebRTC 连接 Go2，订阅 sport/low state。
- 姿态与急停命令：`stand_up`、`balance_stand`、`stand_down`、`recovery_stand`、`damp`、`sit`、`stop`。
- 虚拟摇杆速度原语：`drive(vx, vy, vyaw, duration)`；MCF 下混合 Move(1008) 与 `rt/wirelesscontroller`。
- `stream_hold()`：Web 按住操控的连续 50Hz 流，避免分片归零卡顿。
- `release_drive()`：松手只零帧，不 StopMove（DimOS 语义）。
- 操作者入口：`run_unitree_teleop.py`（终端 nudge）、`run_unitree_teleop_web.py`（浏览器车式键位）。
- 基础安全门：`unitree_dry_run`、`unitree_enable_motion`、速度/角速度/时长 clamp、终端确认短语。
- smoke test：只读、姿态序列和短时前进/后退/旋转 demo。

当前实现仍缺少：

- 单一控制所有权；多个 `drive` 并发时可能互相覆盖。
- 急停对正在运行的摇杆发送任务的抢占和等待确认。
- `drive` 被取消或发送异常时的强制归零保证。
- 控制链路 watchdog、状态新鲜度判断和断连后的停止策略。
- 真机动作前置条件、动作后验和分级验收记录。
- 面向操作者的安全遥控入口；上层 Agent 仍不应直接获得速度控制能力。

### Features

- 引入单一运动控制租约，同一时间最多允许一个活动运动命令。
- 新指令默认替换旧指令；替换前必须取消旧发送任务并发送零摇杆帧。
- `stop` 成为最高优先级抢占操作：停止活动运动流、发送零摇杆帧和 `StopMove`，并等待本地停止完成。
- 使用 `try/finally` 保证 `drive` 正常结束、异常、超时、取消时都发送多次零摇杆帧。
- 增加命令 watchdog：控制帧超过设定间隔未续发时自动归零并记录原因。
- 增加状态新鲜度和动作前置检查：已连接、状态未过期、机器人处于可运动姿态、无已知错误。
- 增加动作后验：动作结束后读取状态，确认机器人已停止；失败时升级执行 `stop`。
- 面向操作者的安全遥控入口，只暴露离散低速 nudge、Web 按住 teleop 和急停，不接入 LLM/skills。
- 更新 Go2 接入文档，明确 WebRTC 配置、AES 密钥、实机验收顺序、急停与回退流程。
- 使用 fake connection 覆盖并发、抢占、取消、超时、断连和异常路径。

### Constraints

- `RDB_ROBOT=mock`、`RDB_UNITREE_TRANSPORT=fake`、`RDB_UNITREE_DRY_RUN=true` 和 `RDB_UNITREE_ENABLE_MOTION=false` 继续作为默认安全配置。
- 本轮真实平移只允许 WebRTC 虚拟摇杆路径；DDS transport 保持只读。
- 本轮仅开放短时速度原语，不实现 `move_to`、绝对航向 `turn`、路径规划、避障、跟随或回充。
- 不把 `drive` 注册为 LLM 可调用 skill，不允许自然语言直接生成底层速度。
- 首轮真机验收速度不超过 `0.2 m/s`，角速度不超过 `0.3 rad/s`，单次时长不超过 `0.5 s`。
- 无法从无线摇杆 topic 获得 ACK，因此必须依靠本地发送完成、零帧、`StopMove` 和状态后验构成闭环。
- 任意异常均按“先停止，再上报”处理；停止失败必须清晰暴露，不能静默成功。

### Success Criteria

- 同一时间最多存在一个活动 `drive`；并发请求不会产生交错的非零摇杆流。
- `stop` 可以在活动 `drive` 中途抢占，且抢占后不再发送非零摇杆帧。
- `drive` 正常结束、被取消、发送异常和断连时均尝试发送零摇杆帧。
- watchdog 能在控制续发中断后自动停止，并记录结构化原因。
- 状态过期、未站立、存在错误码或 motion gate 未开启时，真实运动被拒绝。
- 每次真实动作均记录请求值、裁剪值、开始/结束时间、结束原因、前后状态和停止结果。
- 自动化测试覆盖关键竞态和失败路径，现有测试保持通过。
- 真机分级验收从“只读 → 急停 → 姿态 → 原地旋转 → 直线短步”逐级通过；任一级失败立即停止，不进入下一级。

## 设计方案

### 1. 控制链路

```text
Operator CLI / Web Teleop
    │
    ▼
UnitreeRobot.drive() | stream_hold()
    ├── dry-run / 参数 clamp / 动作审计
    ├── 状态前置检查（stream_hold 会话开始时一次）
    └── UnitreeCommand(action="drive"|"release"|"stop")
            │
            ▼
UnitreeWebRTCTransport
    ├── enable_motion 硬安全门
    ├── 单一 motion lease + generation
    ├── 50 Hz 控制帧（Move 或摇杆，按 vy/vyaw 自动选择）
    ├── watchdog
    └── stop/disconnect: 零帧 + StopMove；release: 仅零帧
            │
            ▼
Go2 rt/wirelesscontroller + rt/api/sport/request (Move 1008)
```

### 2. 单一运动租约与抢占

`UnitreeWebRTCTransport` 维护一个活动运动任务和递增 generation：

- 每个新 `drive` 获得新 generation。
- 新 `drive` 开始前先取消并等待旧任务完成归零。
- 发送循环每次发送前检查 generation；失去所有权后不得继续发送非零帧。
- `stop` 使 generation 失效，取消活动任务，发送零帧，再发送 `StopMove`。
- `disconnect()` 先执行同样的停止流程，再断开 WebRTC。

该机制用于解决当前最危险的竞态：旧 `drive` 在急停或新命令之后继续发送非零摇杆值。

### 3. Watchdog 与归零策略

- 新增 `RDB_UNITREE_CONTROL_WATCHDOG_SECONDS`，建议默认 `0.25` 秒。
- 新增 `RDB_UNITREE_ZERO_FRAME_COUNT`，建议默认 `5` 帧。
- 每次成功发送非零控制帧更新本地心跳时间。
- 活动命令超时、任务取消、发布异常或连接关闭时，在 `finally` 中发送零帧。
- `stop` 额外发送 sport `StopMove`，作为与无线摇杆归零互补的停止手段。

### 4. 前置检查与后验

真实 `drive` 开始前检查：

- transport 已连接。
- 最近 sport state 时间不超过 `RDB_UNITREE_STATE_MAX_AGE_SECONDS`。
- `error_code == 0`。
- 机器人处于站立/平衡状态；若状态无法可靠判断则拒绝，而不是自动站立。
- `dry_run=false` 且 `enable_motion=true`。

动作结束后：

- 读取最新状态并等待 `is_moving=false`，设置短超时。
- 若超时仍在运动，执行升级停止并返回失败。
- 记录停止原因：`completed`、`preempted`、`operator_stop`、`watchdog`、`cancelled`、`transport_error` 或 `disconnect`。

### 5. 操作者入口

| 入口 | 文件 | 行为 |
|------|------|------|
| 终端离散 nudge | `examples/run_unitree_teleop.py` | 单次固定时长 nudge；W/S/A/D/Q/E |
| Web 按住 teleop | `examples/run_unitree_teleop_web.py` | 连续 `stream_hold`；默认车式键位 |
| 分级验收 | `examples/run_unitree_smoke.py --graded` | Level 0–5 逐级真机验收 |

Web teleop 键位（默认 car）：

- W/S：前进 / 后退
- A/D：左转 / 右转（vyaw）
- Q/E：左 / 右平移（vy）
- W+D：前进 + 右转弧线（走虚拟摇杆，非斜移）

MCF 混合通道：`vy != 0` 且无 vyaw 时走 Move(1008)；**含 vyaw 时走 wirelesscontroller**（DimOS 同路径），解决 Move 无法组合弧线的问题。

Prep 序列（实机 teleop 自动执行）：`stand_up → balance_stand → free_walk → SwitchJoystick + SpeedLevel=1`。

### 6. 预计影响文件

| 文件 | 计划变更 |
|------|----------|
| `robot_brain/actuation/unitree_webrtc.py` | 单一运动租约、stop 抢占、watchdog、归零保证、状态时间戳 |
| `robot_brain/actuation/unitree.py` | 动作前置检查、动作后验和更完整的审计记录 |
| `config/settings.py` | watchdog、状态新鲜度、零帧数量和首轮实机限值配置 |
| `examples/run_unitree_smoke.py` | 分级真机验收流程和结果摘要 |
| `examples/run_unitree_teleop.py` | 面向操作者的离散低速遥控入口 |
| `examples/run_unitree_teleop_web.py` | Web 按住 teleop、车式键位、drive_ack 调试 |
| `tests/test_teleop_web_combine.py` | 多键向量合成与通道标签 |
| `docs/unitree-setup.md` | WebRTC 实机连接、环境变量、teleop 与故障排查 |
| `README.md` | Go2 快速入口与环境变量摘要 |

## 分阶段计划

### 阶段 A：控制任务生命周期

- [x] 为 WebRTC transport 增加活动运动任务、generation 和互斥保护
- [x] `drive` 使用 `try/finally` 保证归零
- [x] `stop`、新 `drive` 和 `disconnect` 能抢占旧运动任务
- [x] 为结束原因建立稳定枚举或结构化字符串

验收：

- [x] 并发 `drive` 测试中不存在旧任务晚于新任务发送非零帧
- [x] stop 返回后不再出现非零摇杆帧
- [x] 取消和异常路径均产生零帧

### 阶段 B：Watchdog、前置检查与后验

- [x] 增加状态更新时间和 stale-state 判断
- [x] 增加控制 watchdog
- [x] 增加连接、站立状态和错误码前置检查
- [x] 动作完成后确认停止，失败时升级 stop
- [x] 审计记录补充请求值、裁剪值、时长、结束原因和前后状态

验收：

- [x] stale state、未站立、错误码非零时拒绝真实动作
- [x] watchdog 到期自动归零（专用单测已补）
- [x] 动作后仍移动时执行升级停止并返回失败

### 阶段 C：操作者遥控与自动化测试

- [x] 新增离散低速 teleop 示例
- [x] smoke test 改造成分级验收，并输出可保存的验收摘要
- [x] fake connection 覆盖抢占、取消、watchdog、异常和断连
- [x] 保持现有 mock/fake/sdk 行为不变

验收：

- [x] 无真机环境可完整运行 dry-run 与 fake 测试
- [x] 默认配置不会触发真实动作
- [x] 完整测试与编译检查通过

### 阶段 D：真机分级验收

- [ ] Level 0：只读连接，连续观察状态至少 60 秒
- [ ] Level 1：只执行 stop，确认可重复且不改变姿态
- [ ] Level 2：执行站立/平衡/FreeWalk + SwitchJoystick 序列
- [ ] Level 3：原地左右旋转 nudge，各不超过 `0.3 rad/s × 0.5 s`
- [ ] Level 4：前进与后退 nudge，各不超过 `0.2 m/s × 0.5 s`
- [ ] Level 5：运动中人工急停，确认立即停止且无后续非零帧
- [x] Web teleop 实机：前进、弧线（W+D）、连续按住流畅度（STA-L / MCF）

验收：

- [x] 每一级均有前后状态和动作审计记录（分级脚本支持 JSON 摘要 + `--output-json`）
- [ ] 任一级失败后系统停止，且不会自动进入下一级
- [x] 操作者能够使用 Web 面板 + STOP 完成低速 teleop（2026-06-11 实机确认）

### 阶段 E：Web teleop 与 MCF 控制通道

- [x] `stream_hold()` 连续 50Hz，替代 0.15s 分片 drive，消除按住卡顿
- [x] 车式默认键位（A/D 转弯）；`--omni` 保留全向映射
- [x] MCF 混合通道：Move 负责 vy/纯 vx；vyaw 与 W+D 弧线走 virtual joystick
- [x] `release` 与 `stop` 分离；WebSocket 异步调度避免 UI 死锁
- [x] Web 面板 `drive_ack` 显示 vx/vy/vyaw 与通道标签

验收：

- [x] W+D 状态栏显示 `joystick (arc)` 且实机走弧线（非斜移）
- [x] Q/E 侧移仍可用（Move 通道）
- [x] 183+ 自动化测试通过

## 真机验收前检查清单

- [ ] Go2 位于平坦、防滑、开阔地面，四周至少留出 2 米空间
- [ ] 现场只有一套软件控制源，Unitree App/遥控器不同时发送运动指令
- [ ] 操作者与旁观者保持安全距离
- [ ] 操作者手边有 App、遥控器或断电急停方式
- [ ] 已确认机器人 IP、AES 密钥、motion mode 和状态读取正常
- [ ] `RDB_UNITREE_MAX_SPEED=0.2`
- [ ] `RDB_UNITREE_MAX_YAW_SPEED=0.3`
- [ ] `RDB_UNITREE_MAX_DRIVE_DURATION=0.5`
- [ ] 仅在开始对应等级时显式关闭 dry-run、打开 motion gate

## 风险与缓解

| 风险 | 缓解 |
|------|------|
| 急停后旧发送任务继续发非零帧 | generation + 任务取消等待 + stop 返回后断言无非零帧 |
| 发送循环异常导致未归零 | `try/finally` 多次发送零帧，随后发送 `StopMove` |
| WebRTC 无线摇杆 topic 无 ACK | 本地发送审计 + 状态后验 + 超时升级停止 |
| 状态陈旧导致错误判断 | 记录 topic 更新时间，超过阈值拒绝真实运动 |
| 多个控制源争夺机器人 | 本轮只允许单进程操作者入口，并在文档中要求关闭其他控制源 |
| 上层 Agent 生成危险速度 | 本轮不注册速度 skill，不开放服务 API 或自然语言控制 |
| 真机姿态不适合运动 | 运动前要求状态检查，无法确认则拒绝 |
| 断连时无法保证远端已停止 | 断连前尽力归零和 StopMove；现场保留独立物理急停 |

## 验证方式

- [x] `python -m compileall -q robot_brain config tests examples`
- [x] `python -m pytest`（188 passed，含 watchdog / 取消 / 异常 / 断连单测）
- [x] fake connection：正常结束后归零
- [x] fake connection：新 `drive` 抢占旧 `drive`
- [x] fake connection：stop 抢占活动 `drive`
- [x] fake connection：watchdog 到期自动归零
- [x] fake connection：任务取消、发布异常、disconnect 后归零
- [x] fake connection：stale state、未站立阻止真实动作
- [x] hybrid channel：vx+vyaw 走 joystick，vy 走 Move
- [ ] 真机：按 Level 0–5 顺序完整跑通 `--graded`（Web teleop 已单独验收）

## 非目标

- 不实现自主导航、SLAM、路径规划或机载避障集成。
- 不把 DimOS 整套 Blueprint/stream 架构直接移植进 robot-brain。
- 不开放无限时长或持续按键式高速运动（Web hold 有 5s 会话上限，可循环续按）。
- 不实现 WebRTC 视频、LiDAR 或里程计感知流；这些能力留给后续感知迭代。
- 不实现多机编队或多操作者并发控制。
- 不将 `move_to` 和绝对航向 `turn` 映射成开环速度动作。

## 复盘

### 与设计一致的部分

- 单一 motion lease + generation 抢占；stop / release / disconnect 语义清晰。
- 默认安全配置不变；操作者入口与 Agent 隔离。
- 前置检查、后验、审计字段、`MotionEndReason` 枚举均落地。

### 实机偏差与修正

| 现象 | 原因 | 修正 |
|------|------|------|
| 侧移无效 | MCF 上纯摇杆 lx 不驱动 vy | 引入 Move(1008) fire-and-forget |
| 按住 teleop 卡顿 | 0.15s 分片 drive + 每片归零 | `stream_hold()` 连续 50Hz |
| W+D 像斜走 | 全向键位 A/D=vy | 默认车式键位 A/D=vyaw |
| W+D 仍像直走 | Move 不支持 vx+vyaw 弧线 | 含 vyaw 时改走 wirelesscontroller |

### 自动化测试

- 183 passed（含 `test_unitree_webrtc_transport`、`test_teleop_web_combine` 等）。
- 188 passed（新增 watchdog、取消、发布异常、断连、后验升级 stop 单测）。
- 混合通道、motion lease、Move payload、release/stop 均有单测覆盖。

### 真机结果（2026-06-11，Go2 STA-L / MCF / IP 10.10.196.239）

- WebRTC 连接、prep、前进、侧移、Web 按住 teleop、W+D 弧线：**通过**。
- `--graded` Level 0–5 正式跑通：**待补**（脚本已就绪）。

### 下一步

- 真机跑完 `--graded --level 5 --output-json acceptance.json` 并归档 JSON 摘要。
- 感知流（LiDAR / 里程计 / 视频）与闭环局部运动。
- 评估是否将 teleop 能力以只读监控形式接入主服务（仍不开放 LLM drive）。
