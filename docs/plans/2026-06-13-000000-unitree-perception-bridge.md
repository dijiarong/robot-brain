# 第十次迭代：Go2 Perception Bridge

## 基本信息

- 创建时间：2026-06-13 CST
- 状态：代码完成，自动化测试通过（213 passed）；WebRTC 真机感知验证 **待现场**
- 负责人：dijia
- 前置迭代：[第九次迭代：Unitree Go2 WebRTC 实机操控安全闭环](./2026-06-11-172047-unitree-live-control-loop.md)

## Project Requirements

### Goal

打通 Go2 sport state → Observation → WorldState 的数据链路，让认知层（FastReflex、SafetyValidator、Skills、LLM）能读到真实的机器人本体状态（sport mode、error_code、速度、IMU、站立/运动、电量新鲜度）。

本轮不做复杂反射规则和 Go2 技能族。

### Existing Baseline

第九次迭代完成后：
- 执行层扎实：`UnitreeRobot.drive()` / `stop()` / 姿态命令、motion lease、watchdog、pre/post-check、审计
- 感知断层：`WorldState` 只有 mock 数据，Go2 真实状态从未进入认知链路
- `UnitreeState` 携带 `is_standing`、`error_code`、`sport_mode`、速度、IMU，但 `RobotState` 映射时丢弃
- `MockPerception` 是唯一的 `PerceptionAdapter` 实现

### Features

- 新增 `UnitreePerceptionAdapter`：调用 `robot.get_state()` 取 generic 字段，`robot.transport.read_state()` 取 Go2 本体状态
- 新增 `RobotSelfState` Pydantic 模型，含 `Velocity`、`ImuRPY` 子模型，分离环境状态与本体状态
- `Observation.self_state: RobotSelfState | None` — 可选字段，非 Unitree 后端为 None
- `WorldState.robot_self_state: RobotSelfState | None` + `apply_observation()` null-safe merge
- `WorldState` 新增 `state_age_seconds`、`robot_error_code` 便捷查询属性
- `AgentRuntime.create()` 支持 `RDB_PERCEPTION=unitree` 接线
- `FakeUnitreeTransport` 完整支持 `state_age_seconds()`、velocity/imu_rpy 状态操作
- WebRTC 和 SDK transport 的 `_map_state` 填充 `velocity` 和 `imu_rpy`

### Constraints

- `RobotSelfState`、`Velocity`、`ImuRPY` 定义在 `core/robot_self_state.py`（独立模块，避免循环导入）
- IMU rpy：`UnitreeState` 存弧度（原始 Go2 格式），`RobotSelfState` 存度（人类可读）
- transport read 失败时降级为只有 source 的 RobotSelfState，不抛异常
- `MockPerception` 行为不变：`obs.self_state is None`

### Success Criteria

- [x] `UnitreePerceptionAdapter.observe()` 产生带完整 `self_state` 的 `Observation`
- [x] `WorldState.apply_observation()` 正确处理 `self_state`（null-safe merge）
- [x] mock observation (`self_state=None`) 不覆盖已有的 `robot_self_state`
- [x] `AgentRuntime.create(perception_backend="unitree")` 正确创建 adapter
- [x] `AgentRuntime.create(perception_backend="mock")` 行为不变
- [x] transport 断连时 adapter 降级不崩溃
- [x] 25 新增测试 + 188 已有测试全部通过
- [ ] 真机：`RDB_ROBOT=unitree RDB_UNITREE_TRANSPORT=webrtc RDB_PERCEPTION=unitree` 启动服务，确认 `world.robot_self_state` 有真实 Go2 数据

## 设计方案

### 核心模型

```python
class Velocity(BaseModel):
    vx: float = 0.0  # forward (m/s)
    vy: float = 0.0  # lateral (m/s)
    vz: float = 0.0  # vertical (m/s)

class ImuRPY(BaseModel):
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

class RobotSelfState(BaseModel):
    source: str                        # "unitree_go2"
    is_standing: bool | None = None
    is_moving: bool | None = None
    sport_mode: int | None = None
    error_code: int | None = None
    velocity: Velocity | None = None
    imu_rpy: ImuRPY | None = None
    state_age_seconds: float | None = None
```

### 数据流

```
Go2 sport state → UnitreeState (velocity, imu_rpy)
  → UnitreePerceptionAdapter.observe()
    → Observation(generic fields + self_state)
    → WorldState.apply_observation()
    → WorldState.robot_self_state
    → snapshot() → LLM prompt
```

### 影响文件

| 文件 | 操作 | 说明 |
|------|------|------|
| `core/robot_self_state.py` | **新建** | Velocity/ImuRPY/RobotSelfState，独立模块避免循环导入 |
| `perception/base.py` | 修改 | Observation 加 self_state 字段；导入从 core.robot_self_state |
| `perception/unitree.py` | **新建** | UnitreePerceptionAdapter + _build_self_state |
| `perception/__init__.py` | 修改 | 导出新模型和 adapter |
| `perception/mock.py` | 不变 | MockPerception 不设 self_state（向后兼容） |
| `actuation/unitree.py` | 修改 | UnitreeState 加 velocity/imu_rpy；transport 抽象加 state_age_seconds；Fake 覆写；UnitreeRobot 加 transport property |
| `actuation/unitree_webrtc.py` | 修改 | _map_state 填充 velocity/imu_rpy |
| `actuation/unitree_sdk.py` | 修改 | _map_sport_state/_map_state 填充新字段；加时间戳追踪 |
| `core/world_state.py` | 修改 | 加 robot_self_state 字段、apply_observation merge、查询属性 |
| `runtime/loop.py` | 修改 | AgentRuntime.create() perception_backend="unitree" 分支 |
| `tests/test_perception_unitree.py` | **新建** | 25 个测试覆盖所有路径 |

## 真机验证方式

```bash
RDB_ROBOT=unitree \
RDB_UNITREE_TRANSPORT=webrtc \
RDB_PERCEPTION=unitree \
RDB_UNITREE_DRY_RUN=true \
RDB_UNITREE_ROBOT_IP=10.10.196.239 \
python -c "
import asyncio
from config.settings import Settings
from robot_brain.runtime.loop import AgentRuntime

async def main():
    s = Settings(robot_backend='unitree', perception_backend='unitree',
                 unitree_transport='webrtc', unitree_dry_run=True,
                 memory_db_path=':memory:')
    rt = AgentRuntime.create(settings=s)
    await rt.context.robot.transport.connect()
    await rt.refresh_world(reason='smoke')
    ws = rt.context.world
    assert ws.robot_self_state is not None, 'robot_self_state missing'
    assert ws.robot_self_state.source == 'unitree_go2'
    print('PASS:', ws.robot_self_state.model_dump(mode='json'))

asyncio.run(main())
"
```

## 下一步

- 第十一次迭代：Go2 原生技能族（nudge/scan/approach/retreat，映射到 drive，LLM 可安全调用）
- 第十二次迭代：FastReflex 真传感器规则（低电量自动 dock、连续 error_code 自动 sit、运动中异常自动 stop）
- 之后：Teleop 仪表盘、LiDAR/Video 感知流、多模态视觉 LLM
