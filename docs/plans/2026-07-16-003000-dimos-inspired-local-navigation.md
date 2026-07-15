# 第十九次迭代：借鉴 DimOS 的 Go2 局部导航底座

## 基本信息

- 创建时间：2026-07-16 00:30 CST
- 文件序号：2026-07-16-003000
- 状态：阶段 A 初版完成（pose/odom + go2_local_nav fake 可验收）
- 负责人：dijia
- 真机依赖：阶段 A 不依赖真机；阶段 B 需要 Go2 WebRTC / SDK 现场验证
- 前置完成：[第十八次 Explore 现场可验收闭环](./2026-07-11-204545-explore-field-verification-loop.md)
- 参考源码：`/Users/dijia/project/topsun_dimos`

## 背景

第十八次迭代已经把 `explore` 做成可解释、可停止、可复盘的闭环：每步有 trace，VLM 只是软建议，超声波仍是硬门，连续无进展会停。

下一步如果想从“探索”走向“导航”，不能直接把 `navigate` 暴露给 Go2。当前 robot-brain 仍缺三件事：

- 没有可靠的真机位姿反馈进入 `WorldState`
- `no_progress` 仍主要基于行为 trace，而不是真实位移
- Go2 的移动仍是短段速度/摇杆命令，缺少“命令发出后是否真的到达”的闭环

DimOS 在 Go2 上已经有完整路线：WebRTC 订阅 `ROBOTODOM` / `ULIDAR_ARRAY` / video，转换成 `PoseStamped` / `PointCloud2` / `Image`，再由导航模块输出 `cmd_vel` / `Twist`。但 DimOS 的完整栈包含 Blueprint、stream、TF、LCM/pSHM、VoxelGrid、CostMapper、A*、frontier exploration，复杂度明显高于 robot-brain 当前阶段。

本轮目标是**借鉴 DimOS 的导航感知链路和局部闭环思想，而不是移植整套导航栈**。

## DimOS 可借鉴点

### 1. Go2 连接模块的传感器流

DimOS `GO2Connection` 暴露：

```text
lidar_stream()  -> PointCloud2
odom_stream()   -> PoseStamped
video_stream()  -> Image
cmd_vel         -> Twist
```

关键源码：

- `/Users/dijia/project/topsun_dimos/dimos/robot/unitree/go2/connection.py`
- `/Users/dijia/project/topsun_dimos/dimos/robot/unitree/connection.py`
- `/Users/dijia/project/topsun_dimos/dimos/robot/unitree/type/odometry.py`
- `/Users/dijia/project/topsun_dimos/dimos/robot/unitree/type/lidar.py`

对 robot-brain 的启发：

- WebRTC / SDK transport 不应只给 `RobotSelfState` 填电量、站立、错误码，也要能填 `pose` / `odom`
- 视频 tap 已经用于 VLM，下一步要把 odom/LiDAR 也变成可诊断输入
- 需要统一“传感器新鲜度”，避免用旧 pose 判断进展

### 2. Twist / cmd_vel 作为局部运动语义

DimOS 把标准 `Twist` 映射到 Go2 WebRTC 控制：

```text
linear.x  -> forward/back
linear.y  -> lateral
angular.z -> yaw
```

robot-brain 已经有 `go2_drive_segment` / `UnitreeRobot.drive()`，不需要引入 ROS `Twist` 类型，但可以引入等价的轻量内部对象：

```python
LocalVelocity(vx_mps: float, vy_mps: float, yaw_rps: float, duration_s: float)
```

### 3. 相对位姿目标

DimOS `relative_move(forward, left, degrees)` 的核心思想是：

```text
当前 pose + 本体系 offset -> 世界系 goal pose -> navigation set_goal
```

robot-brain 本轮不接完整 `set_goal` planner，但可以先实现轻量 `LocalGoal`：

```text
forward_m / left_m / yaw_deg
```

然后由局部闭环执行器在短距离内完成：

1. 读取当前 pose
2. 下发小段速度
3. 重新读取 pose
4. 判断 delta 是否达到目标或触发停止条件

### 4. 闭环旋转

DimOS `rotate_in_place_degrees()` 用累计 yaw 而不是绝对角差，避免 360 度 wrap bug。

robot-brain 的 `scan` / `explore` Go2 路径可以借这个思想：如果 odom 可用，`scan_segments` 不只记录发了多少段，还记录实际 yaw delta。

## 本轮目标

### 阶段 A：不依赖真机，必须完成

- [x] 定义轻量导航感知模型：
  - `RobotPose`
  - `OdometryData`
  - `LocalMotionDelta`
  - `LocalGoal`
- [x] 扩展 `RobotSelfState` 或 `WorldState`，可记录：
  - 当前 pose
  - 线速度 / 角速度
  - pose timestamp / age
  - 最近一次运动 delta
- [ ] 增加 odom 映射 helper，支持从 dict / fake transport 数据映射到内部 pose
- [x] `UnitreePerceptionAdapter` 将 odom 写入 world
- [x] `explore` trace 增加：
  - `pose_before`
  - `pose_after`
  - `delta_m`
  - `delta_yaw_deg`
  - `progress_source`: `odom` / `behavior`
- [x] `no_progress` 优先用 odom delta 判定；无 odom 时回退第十八次行为判定
- [x] 新增 fake odom 测试，不接真机即可验证：
  - nudge 后 delta_m 增加
  - scan 后 delta_yaw_deg 增加
  - 发了 move 但 odom 不动时触发 `no_progress`
- [x] 保持 Go2 下 `navigate` / `patrol` 仍不暴露给 LLM

### 阶段 B：需要真机，现场验证

- [ ] WebRTC 路径读取 Go2 `ROBOTODOM` 或等价 sport/utlidar odom
- [ ] SDK 路径若只有 sport state，先填 `pose/velocity` 的可用子集
- [ ] 现场 dry-run 记录 odom trace，不下发运动
- [ ] live gated 小步验证：
  - 原地旋转 15-30 度，trace 能看到 yaw delta
  - 前进 10-20 cm，trace 能看到 delta_m
  - 手动阻挡/抱起/打滑时，`no_progress` 能停
- [ ] 输出 acceptance JSON，归档 `pose_before/after/delta/stop_reason`

## 非目标

- 不移植 DimOS Blueprint / Module / stream 架构
- 不引入 LCM / pSHM / reactivex 作为 robot-brain 主架构依赖
- 不做完整 SLAM / VoxelGrid / CostMapper / ReplanningAStar / frontier exploration
- 不恢复 Go2 的全局 `navigate` / `patrol` tool 暴露
- 不让 LLM 直接输出速度流
- 不做无边界自主漫游

## 设计建议

### 1. 数据模型

建议新增或扩展在 `robot_brain/core/robot_self_state.py`：

```python
class RobotPose(BaseModel):
    x_m: float
    y_m: float
    z_m: float = 0.0
    yaw_deg: float
    frame_id: str = "world"
    timestamp: float | None = None

class OdometryData(BaseModel):
    pose: RobotPose | None = None
    vx_mps: float | None = None
    vy_mps: float | None = None
    yaw_rate_dps: float | None = None
    source: str = "unknown"
```

不要把 DimOS 的 `PoseStamped` / `Quaternion` 直接搬进来。robot-brain 只需要二维局部导航所需字段：`x/y/yaw/timestamp`。

### 2. 进展判定

新增 helper：

```text
compute_motion_delta(before_pose, after_pose)
```

输出：

```text
delta_m
delta_yaw_deg
age_ms
valid
```

`explore` 的 guard 改为：

```text
如果 odom 有效：
  nudge/retreat 后 delta_m < 阈值 -> no_progress_count += 1
  scan 后 delta_yaw_deg < 阈值 -> scan_no_progress_count += 1
否则：
  回退行为 trace 判定
```

默认阈值建议：

| 配置 | 默认 | 说明 |
|------|------|------|
| `RDB_ODOM_PROGRESS_MIN_M` | `0.03` | 认为发生有效平移的最小位移 |
| `RDB_ODOM_PROGRESS_MIN_YAW_DEG` | `3.0` | 认为发生有效旋转的最小角度 |
| `RDB_ODOM_MAX_AGE_SECONDS` | `1.0` | odom 新鲜度 |

### 3. 局部导航技能

本轮已新增轻量相对局部导航技能：

```text
go2_local_nav
```

参数：

```text
forward_m: -1.0..1.0
left_m: -0.5..0.5
yaw_deg: -90..90
max_duration_s
```

边界：

- 只在 `unitree` 后端注册
- 默认需要确认
- 不接受地图坐标，只接受相对短距目标
- 每次只执行一个短目标
- 执行过程中如果 odom stale、超声波近障、VLM stop、急停、低电量，立即停止

当前实现范围：

- 已支持相对 `forward_m` / `left_m` / `yaw_degrees`，输出 move/yaw segments 和 odom delta
- 已在 Unitree tool 白名单中注册，但保持全局 `navigate` / `patrol` 隐藏
- 已加入确认要求和参数范围校验
- 已覆盖 fake odom 测试；真机 odom/stale/VLM stop 联动仍放阶段 B

## 影响模块

| 模块 | 变化 |
|------|------|
| `robot_brain/core/robot_self_state.py` | 增加 pose / odom 模型 |
| `robot_brain/core/world_state.py` | snapshot/cognitive_snapshot 包含 odom 摘要 |
| `robot_brain/perception/unitree.py` | 将 UnitreeState odom 映射到 world |
| `robot_brain/actuation/unitree.py` | fake transport 支持模拟 pose delta |
| `robot_brain/actuation/unitree_webrtc.py` | 读取并映射 WebRTC odom 数据 |
| `robot_brain/skills/builtin/explore.py` | trace 和 no_progress 升级为 odom 优先 |
| `config/settings.py` | 增加 odom 进展阈值 |
| `examples/run_explore_acceptance.py` | JSON 增加 pose/delta 字段 |
| `tests/` | fake odom、trace delta、no_progress 回归 |

## 验证方式

### 自动化

```bash
pytest tests/test_perception_unitree.py
pytest tests/test_explore_trace.py
pytest tests/test_explore_no_progress.py
pytest tests/test_unitree_webrtc_transport.py
pytest tests/test_go2_skills.py
python -m ruff check .
```

### fake 验收

```bash
python -m examples.run_explore_acceptance --mode unitree-fake --max-steps 2 --output-json acceptance-odom-fake.json
```

期望：

- trace 中每步包含 pose/delta 字段
- 有 odom 时 `progress_source=odom`
- fake nudge/scan 能产生 delta
- fake no-motion 场景能触发 `no_progress`

### 现场验收

```bash
RDB_ROBOT=unitree \
RDB_PERCEPTION=unitree \
RDB_UNITREE_TRANSPORT=webrtc \
RDB_UNITREE_DRY_RUN=true \
python -m examples.run_explore_acceptance --mode unitree-fake --output-json acceptance-odom-dry.json
```

live gated 仅在空旷、安全、人工急停可用时执行。

## 风险与缓解

| 风险 | 缓解 |
|------|------|
| odom 坐标系和当前 heading 方向不一致 | 先只用 delta 范数和 yaw delta，不承诺全局坐标语义 |
| odom timestamp 不稳定 | 加 `RDB_ODOM_MAX_AGE_SECONDS`，stale 时回退行为判定或停止 |
| WebRTC / SDK 可用字段不同 | 内部模型允许字段为空，按可用性降级 |
| fake odom 与真实 odom 差异大 | acceptance JSON 明确 `source=fake/webrtc/sdk` |
| 过早暴露 navigate | 保持 tool 过滤，新增能力只能从 bounded/local 技能进入 |

## 交付标准

- 不接真机时，fake odom 测试和 acceptance 可跑通
- `explore` trace 能回答“命令发了，机器人实际动了吗”
- `no_progress` 在 odom 可用时基于真实位移/旋转
- odom 不可用时不破坏第十八次行为
- Go2 仍不会收到 LLM 的全局 `navigate` / `patrol`

## 后续方向

完成本轮后，再考虑三个方向：

1. **局部导航技能**：`go2_local_nav` 从 trace 打底升级为可调用技能
2. **轻量局部地图**：只记录近场障碍扇区，不做完整 SLAM
3. **DimOS planner adapter**：如果未来真需要 A* / frontier，再把 DimOS 作为外部导航服务，而不是把整套栈揉进 robot-brain
