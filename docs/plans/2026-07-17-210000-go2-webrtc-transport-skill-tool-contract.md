# 第二十次迭代：Go2 WebRTC 传输与 Skill / Tool 能力契约收口

## 基本信息

- 创建时间：2026-07-17 21:00 CST
- 文件序号：2026-07-17-210000
- 状态：计划中
- 负责人：dijia
- 真机依赖：阶段 A 不依赖真机；阶段 B 需要 Go2 WebRTC / SDK 现场验证
- 前置完成：
  - [第十八次 Explore 现场可验收闭环](./2026-07-11-204545-explore-field-verification-loop.md)
  - [第十九次借鉴 DimOS 的 Go2 局部导航底座](./2026-07-16-003000-dimos-inspired-local-navigation.md)
  - [能力底座契约](./2026-07-10-170002-capability-foundation-tool-skill-contract.md)

## 核心判断

下一阶段不是在 **WebRTC 控狗** 和 **Skill / Tool 控狗** 之间二选一，而是把两者的边界固定下来：

```text
Planner / LLM / Service API
  -> Skill: explore / nudge / scan / retreat / go2_local_nav
  -> Tool: stop_motion / go2_drive_segment / read_robot_state
  -> Robot adapter: UnitreeRobot
  -> Transport: WebRTC / SDK / fake
  -> Go2
```

**WebRTC 是必要的底层传输路线，但不应该成为上层业务接口。**

WebRTC 负责连接 Go2、下发低延迟运动命令、订阅 sport state / odom / video / ultrasonic 等现场数据；Skill / Tool 负责能力语义、安全边界、确认机制、审计、回放和测试。

## 背景

项目已经具备：

- WebRTC Go2 低速限时操控和急停安全门
- `UnitreeRobot` / `UnitreeTransport` 抽象
- `nudge` / `scan` / `retreat` / `explore` / `go2_local_nav` 等 Go2 技能
- `Tool` / `Skill` / `Policy` / `PlannerCatalog` 能力底座
- explore step trace、VLM hint、odom 优先 no_progress 判定
- fake odom 自动化测试和 acceptance JSON 脚本

但当前仍存在几个路线层面的风险：

- WebRTC transport 中的固件差异、topic 订阅、连接重试、抢读、运动通道选择等细节很复杂，不能泄漏到 planner 或业务 API。
- Skill 层已经能表达行为意图，但部分真机状态和诊断仍依赖 transport 内部字段，现场排障不够直观。
- 第十八、十九次迭代的阶段 B 都指向同一个现实问题：真机数据是否稳定进入 trace，以及机器人没动时系统是否能解释并停止。
- 如果过早开放全局 `navigate` / `patrol`，会绕过当前短距、可验收、可停止的安全边界。

本轮目标是把 **WebRTC 作为 transport，Skill / Tool 作为能力契约** 这条路线落成工程事实。

## 目标

### 阶段 A：离线 / fake 可完成

- [ ] 新增一份 Go2 能力分层说明，明确 transport、robot adapter、tool、skill、planner catalog 的职责边界。
- [ ] 抽出 odom / pose 映射 helper，支持从 WebRTC dict、SDK sport state、fake transport 数据映射到内部 `RobotPose` / `OdometryData`。
- [ ] 为 `go2_drive_segment` / `stop_motion` / `read_robot_state` 梳理 metadata，确保 risk、motion_kind、backend_allowlist、requires_confirmation 表达完整。
- [ ] 确认 Go2 下 planner 可见能力仍只来自安全 skill / catalog，不暴露底层 WebRTC 或任意速度流。
- [ ] `examples/run_explore_acceptance.py` 输出 transport / source / odom freshness / motion delta 的诊断字段。
- [ ] 增加 fake 测试覆盖：
  - WebRTC-like dict odom 映射
  - SDK-like sport state odom 映射
  - planner catalog 不暴露底层 transport 能力
  - `go2_local_nav` 和 `explore` 在 odom stale / no delta 时安全停止或降级

### 阶段 B：真机现场验证

- [ ] WebRTC 连接后确认 sport state / odom / ultrasonic / video 的可用字段。
- [ ] dry-run 运行 explore acceptance，生成真机 trace，不下发运动。
- [ ] live gated 小步验证：
  - 原地旋转 15-30 度，trace 看到 `delta_yaw_deg`
  - 前进 10-20 cm，trace 看到 `delta_m`
  - 手动阻挡 / 打滑 / 抱起时触发 `no_progress`
- [ ] 验证 VLM frame tap 与 video relay 的优先级和抢读提示。
- [ ] 输出现场 acceptance JSON，归档 `transport/source/pose_before/pose_after/delta/stop_reason`。

## 非目标

- 不让 LLM 或服务 API 直接调用 WebRTC topic / api_id / 速度流。
- 不开放全局 `navigate` / `patrol` 给 Go2 planner。
- 不移植完整 DimOS Blueprint / SLAM / VoxelGrid / CostMapper / A* / frontier exploration。
- 不把 VLM 输出直接变成运动命令。
- 不绕过 motion gate、人工确认、急停和低电量策略。

## 设计方案

### 1. 分层边界

| 层 | 职责 | 允许知道什么 | 不应知道什么 |
| --- | --- | --- | --- |
| Transport | WebRTC / SDK / fake 通信，topic 订阅，下发原始命令 | Go2 协议、api_id、连接状态、固件差异 | 用户目标、LLM tool schema |
| Robot adapter | 统一机器人动作接口 | `drive` / `stop` / `get_state` | planner prompt、业务任务 |
| Tool | 原子能力 | 风险等级、确认要求、backend、运动类型 | WebRTC topic 细节 |
| Skill | 行为语义和编排 | nudge/scan/explore/local_nav 的参数、步骤、trace | 任意速度流、全局地图导航 |
| PlannerCatalog | 对 LLM 暴露安全能力集合 | 当前 backend 可用 skill schema | transport 全量能力 |

### 2. WebRTC 继续作为 Go2 主传输

保留 WebRTC 路线，因为它对 Go2 现场能力最关键：

- 低延迟运动控制
- sport state / ultrasonic / odom 订阅
- 前视视频进入 VLM
- 与现有 teleop / acceptance 脚本兼容

但 WebRTC 只作为 transport 实现。上层不直接依赖 `_SPORT_API_ID`、topic 名称或连接库对象。

### 3. Skill / Tool 作为长期能力接口

Go2 的真实行为继续通过 skill / tool 暴露：

- `stop_motion`：底层停止能力，急停态仍允许
- `go2_drive_segment`：短时、低速、受限的原子运动段
- `nudge`：短距离线性动作
- `scan`：有限角度原地观察
- `retreat`：安全后退
- `explore`：有限步探索
- `go2_local_nav`：短距相对局部目标

所有运动能力都必须经过 metadata + policy + confirmation + trace。

### 4. Odom 映射 helper

新增或整理 helper，输入允许是：

- WebRTC sport state dict
- WebRTC odom topic dict
- SDK sport state object
- fake `UnitreeState`

输出统一为：

```text
OdometryData(
  pose=RobotPose(x_m, y_m, z_m, yaw_deg, frame_id, timestamp),
  vx_mps=...,
  vy_mps=...,
  yaw_rate_dps=...,
  source="webrtc" | "sdk" | "fake" | "sport_state"
)
```

原则：

- 字段缺失时部分填充，不抛出无意义异常。
- 坐标系不明时只承诺相邻帧 delta，不承诺全局地图语义。
- timestamp / age 不可信时标记为 stale，让 skill 层降级或停止。

### 5. Acceptance 诊断字段

现场 JSON 至少包含：

```json
{
  "mode": "unitree-live",
  "transport": "webrtc",
  "odom_source": "webrtc",
  "trace": [
    {
      "chosen_action": "nudge_forward",
      "progress_source": "odom",
      "pose_before": {},
      "pose_after": {},
      "motion_delta": {
        "delta_m": 0.12,
        "delta_yaw_deg": 1.8,
        "valid": true
      },
      "stop_reason": null
    }
  ]
}
```

## 影响模块

| 模块 | 变化 |
| --- | --- |
| `robot_brain/core/robot_self_state.py` | 可能补充 odom source / timestamp / freshness 辅助字段 |
| `robot_brain/perception/unitree.py` | 使用统一 odom mapping helper |
| `robot_brain/actuation/unitree_webrtc.py` | 暴露 WebRTC odom / sport state 映射所需字段，不泄漏到 skill |
| `robot_brain/actuation/unitree_sdk.py` | SDK sport state 进入同一 mapping helper |
| `robot_brain/tools/*` | 梳理 Go2 原子 tool metadata |
| `robot_brain/skills/builtin/go2_motion.py` | 确认技能只依赖 robot/tool 契约 |
| `robot_brain/skills/builtin/explore.py` | trace 增强 source/freshness 诊断 |
| `robot_brain/planning/catalog.py` | 确认 planner 不暴露 transport 能力 |
| `examples/run_explore_acceptance.py` | 输出真机诊断 JSON |
| `docs/` | 增加分层说明和现场验收记录 |

## 验证方式

### 自动化

```bash
pytest tests/test_perception_unitree.py
pytest tests/test_unitree_webrtc_transport.py
pytest tests/test_go2_skills.py
pytest tests/test_explore_trace.py
pytest tests/test_explore_no_progress.py
pytest tests/test_tool_registry.py
pytest tests/test_skill_registry_backend.py
python -m ruff check .
```

### fake 验收

```bash
python -m examples.run_explore_acceptance \
  --mode unitree-fake \
  --max-steps 2 \
  --output-json acceptance-go2-contract-fake.json
```

期望：

- trace 中有 `transport/source/pose_before/pose_after/motion_delta`
- 有 odom 时 `progress_source=odom`
- no-motion 场景触发 `no_progress`
- planner catalog 不出现 WebRTC 原始能力

### 真机 dry-run

```bash
RDB_ROBOT=unitree \
RDB_PERCEPTION=unitree \
RDB_UNITREE_TRANSPORT=webrtc \
RDB_UNITREE_DRY_RUN=true \
RDB_UNITREE_ROBOT_IP=<ip> \
python -m examples.run_explore_acceptance \
  --mode unitree-live \
  --max-steps 2 \
  --output-json acceptance-go2-contract-dry.json
```

### 真机 live gated

仅在空旷、安全、人工急停可用时执行：

```bash
RDB_ROBOT=unitree \
RDB_PERCEPTION=unitree \
RDB_UNITREE_TRANSPORT=webrtc \
RDB_UNITREE_DRY_RUN=false \
RDB_UNITREE_ENABLE_MOTION=true \
RDB_UNITREE_ROBOT_IP=<ip> \
python -m examples.run_explore_acceptance \
  --mode unitree-live \
  --max-steps 2 \
  --output-json acceptance-go2-contract-live.json
```

## 交付标准

- 上层 planner / service / skill 不直接依赖 WebRTC 协议细节。
- Go2 真机状态通过统一 odom / self-state 模型进入 WorldState。
- `explore` 和 `go2_local_nav` trace 能回答“命令发了，狗到底动没动”。
- WebRTC 仍是可替换 transport，fake / SDK 不破坏上层能力契约。
- 运动能力继续受 motion gate、确认、急停、低电量、超声波/VLM/odom 停止保护约束。

## 复盘

迭代完成后补充：

- 真机 WebRTC 可用字段列表
- odom / sport state / ultrasonic / video 的现场稳定性
- dry-run 与 live acceptance JSON 路径
- 仍需人工处理的固件差异或连接问题
- 是否可以进入轻量局部地图或外部导航服务 adapter 阶段
