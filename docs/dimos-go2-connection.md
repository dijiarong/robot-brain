# DimOS 连接宇树 Go2 技术文档

> 来源项目：[topsun_dimos](https://github.com/dimensionalinc/dimos)（DimOS）  
> 本文档总结 DimOS 如何连接宇树 Go2，重点说明连接层实现细节。

---

## 1. 概述

DimOS 通过 **Module + Blueprint** 架构连接 Go2。核心连接模块是 `GO2Connection`（`dimos/robot/unitree/go2/connection.py`），它根据运行模式自动选择连接后端，把机器人传感器数据发布到 DimOS 内部 stream，并接收 `cmd_vel` 等控制指令。

**两条真机连接路径：**

| 路径 | 协议 | 适用 Blueprint | 典型用途 |
|------|------|----------------|----------|
| **主路径（默认）** | WebRTC（`:9991`） | `unitree-go2`、`unitree-go2-agentic` 等 | 导航、建图、Agent |
| **备选路径** | DDS / SDK2 | `unitree-go2-keyboard-teleop` 等 | 键盘遥操、纯速度控制 |

无需越狱、无需 ROS，官方固件 1.1.7+ 即可。

---

## 2. 快速上手

### 2.1 首次配网与发现 IP

```bash
# 扫描 BLE + 局域网，打印 IP
dimos go2tool discover

# 首次配 WiFi（通过 BLE）
dimos go2tool connect-wifi --ssid <wifi> --password <password>

export ROBOT_IP=<发现的 IP>
```

### 2.2 运行（真机）

```bash
# 基础导航栈
dimos run unitree-go2 --robot-ip "$ROBOT_IP"

# 带 LLM Agent
dimos run unitree-go2-agentic --robot-ip "$ROBOT_IP"
```

### 2.3 新固件 AES 密钥（可选）

部分新固件 LAN 连接需要 per-device AES-128 密钥：

```bash
uv run scripts/fetch_unitree_aes_key.py <账号> <密码> <序列号> --region cn

export UNITREE_AES_KEY=<32位十六进制>
dimos run unitree-go2 \
  --robot-ip "$ROBOT_IP" \
  --unitree-webrtc-aes-key "$UNITREE_AES_KEY"
```

### 2.4 无硬件开发

```bash
dimos --replay run unitree-go2          # 回放录制数据
dimos --simulation run unitree-go2      # MuJoCo 仿真
```

---

## 3. 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│  CLI: dimos run unitree-go2 --robot-ip 192.168.x.x          │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  Blueprint: unitree_go2                                     │
│    └── unitree_go2_basic                                    │
│          ├── vis_module (Rerun / Foxglove 可视化)            │
│          └── GO2Connection  ← 核心连接模块                   │
│    └── VoxelGridMapper / CostMapper / Planner / ...         │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  make_connection(ip, GlobalConfig)                          │
│    ├── replay  → ReplayConnection                           │
│    ├── mujoco  → MujocoConnection                           │
│    ├── dimsim  → DimSimConnection                           │
│    └── webrtc  → UnitreeWebRTCConnection  (真机默认)         │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
                    宇树 Go2 机器人
                    (WebRTC :9991 或 DDS)
```

**关键源文件：**

| 文件 | 职责 |
|------|------|
| `dimos/robot/unitree/go2/connection.py` | `GO2Connection` 模块、`make_connection()` 工厂 |
| `dimos/robot/unitree/connection.py` | `UnitreeWebRTCConnection` WebRTC 实现 |
| `dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py` | 挂载 `GO2Connection` 的基础 Blueprint |
| `dimos/core/global_config.py` | `robot_ip`、AES 密钥、连接类型等全局配置 |
| `dimos/hardware/drive_trains/unitree_go2/adapter.py` | DDS 路径的 `UnitreeGo2TwistAdapter` |
| `dimos/robot/unitree/go2/cli/go2tool.py` | BLE 配网、LAN 发现工具 |

---

## 4. 配置系统

### 4.1 连接类型判定

`GlobalConfig.unitree_connection_type` 属性决定 `make_connection()` 走哪条路径：

```python
@property
def unitree_connection_type(self) -> str:
    if self.replay:
        return "replay"
    if self.simulation:
        return self.simulation   # "mujoco" 或 "dimsim"
    return "webrtc"              # 真机默认
```

### 4.2 主要配置项

| 配置项 | CLI 参数 | 环境变量 | 说明 |
|--------|----------|----------|------|
| `robot_ip` | `--robot-ip` | `DIMOS_ROBOT_IP` | Go2 局域网 IP |
| `unitree_webrtc_aes_key` | `--unitree-webrtc-aes-key` | `UNITREE_AES_KEY` / `UNITREE_AES_128_KEY` | WebRTC AES 密钥 |
| `unitree_cloud_region` | — | — | `"cn"` 或 `"global"`，默认 `global` |
| `unitree_webrtc_connect_timeout_sec` | — | — | 连接超时，默认 30s |
| `replay` | `--replay` | `DIMOS_REPLAY` | 启用回放模式 |
| `simulation` | `--simulation` | — | `mujoco` / `dimsim` |
| `obstacle_avoidance` | — | — | 是否开启机载避障，默认 `True` |

配置优先级：**CLI 参数 > 环境变量 > `.env` > 默认值**。

### 4.3 模块级配置 `ConnectionConfig`

```python
class ConnectionConfig(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)
    mode: Go2Mode = Go2Mode.DEFAULT          # DEFAULT 或 RAGE
    aes_128_key: str | None = Field(default_factory=lambda m: m["g"].unitree_webrtc_aes_key)
```

---

## 5. 连接实现详解（WebRTC 主路径）

### 5.1 工厂函数 `make_connection()`

位于 `dimos/robot/unitree/go2/connection.py`：

```python
def make_connection(
    ip: str | None,
    cfg: GlobalConfig,
    aes_128_key: str | None = None,
) -> Go2ConnectionProtocol:
    connection_type = cfg.unitree_connection_type

    if ip in ("fake", "mock", "replay") or connection_type == "replay":
        dataset = cfg.replay_db
        return ReplayConnection(dataset=dataset)
    elif ip == "mujoco" or connection_type == "mujoco":
        from dimos.robot.unitree.mujoco_connection import MujocoConnection
        return MujocoConnection(cfg)
    elif connection_type == "dimsim":
        from dimos.robot.unitree.dimsim_connection import DimSimConnection
        return DimSimConnection(cfg)
    elif connection_type == "webrtc":
        assert ip is not None, "IP address must be provided"
        return UnitreeWebRTCConnection(
            ip,
            aes_128_key=aes_128_key,
            region=cfg.unitree_cloud_region,
            device_type="Go2",
            connect_timeout_sec=cfg.unitree_webrtc_connect_timeout_sec,
        )
    else:
        raise ValueError(f"Unknown simulator {cfg.simulation!r}. Choose from: mujoco, dimsim")
```

`GO2Connection.__init__()` 中调用此工厂，结果赋给 `self.connection`。

### 5.2 `UnitreeWebRTCConnection` 初始化

底层依赖第三方库 `unitree-webrtc-connect`（代码中别名为 `LegionConnection`），封装在 `dimos/robot/unitree/connection.py`。

**构造参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ip` | — | 机器人局域网 IP |
| `mode` | `"ai"` | Motion Switcher 模式名 |
| `aes_128_key` | 从配置/环境变量读取 | 32 位十六进制，新固件必需 |
| `region` | `"global"` | 云端区域 |
| `device_type` | `"Go2"` | 设备类型 |
| `connect_timeout_sec` | `30.0` | 连接超时 |

**AES 密钥解析逻辑**（`_legion_connection_kwargs`）：

1. 优先使用传入的 `aes_128_key`
2. 否则依次读 `UNITREE_AES_128_KEY`、`UNITREE_AES_KEY`、`DIMOS_UNITREE_WEBRTC_AES_KEY`
3. 校验格式：必须匹配 `[0-9a-fA-F]{32}`
4. 若 SDK 版本不支持 `aes_128_key` 参数，抛出 RuntimeError 提示升级

**连接方式：** `WebRTCConnectionMethod.LocalSTA`（局域网直连，非云端中继）。

### 5.3 WebRTC 连接建立流程

```
UnitreeWebRTCConnection.__init__()
    │
    ├── 创建 LegionConnection(LocalSTA, ip=..., aes_128_key=...)
    │
    └── connect()
          │
          ├── 新建 asyncio 事件循环 + 后台守护线程
          │
          └── async_connect() [在后台线程中运行]
                │
                ├── await conn.connect()          → 连到 {ip}:9991
                ├── disableTrafficSaving(True)    → 关闭流量节省
                ├── set_decoder("native")         → 视频解码器
                ├── publish_request(MOTION_SWITCHER, mode="ai")
                │     → 切换到 AI 运动模式
                └── 设置 connected_event，保持心跳
```

**超时与错误处理：**

- 等待 `connection_ready` 事件，默认 30 秒
- 超时 → `TimeoutError`，提示检查 IP/网络/AES 密钥
- 连接异常 → `RuntimeError`，附带原始异常链

核心连接代码：

```python
self.conn = LegionConnection(
    WebRTCConnectionMethod.LocalSTA,
    **_legion_connection_kwargs(self.ip, aes_128_key, region, device_type),
)

async def async_connect() -> None:
    await self.conn.connect()
    await self.conn.datachannel.disableTrafficSaving(True)
    self.conn.datachannel.set_decoder(decoder_type="native")
    await self.conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["MOTION_SWITCHER"],
        {"api_id": 1002, "parameter": {"name": self.mode}},
    )
```

### 5.4 数据通道（RTC Topic）

连接建立后，通过 WebRTC DataChannel 的 pub/sub 机制收发数据：

| 方向 | RTC Topic | 数据类型 | 用途 |
|------|-----------|----------|------|
| 订阅 | `ULIDAR_ARRAY` | 原始 LiDAR | → `PointCloud2` 点云 |
| 订阅 | `ROBOTODOM` | 里程计 | → `PoseStamped` 位姿 |
| 订阅 | `LOW_STATE` | 低层状态 | 关节/IMU 等（可选） |
| 订阅 | WebRTC Video Track | H.264 视频帧 | → RGB `Image` |
| 发布 | `WIRELESS_CONTROLLER` | `{lx, ly, rx, ry}` | 速度控制（虚拟摇杆） |
| 发布 | `SPORT_MOD` | `{api_id, parameter}` | 站立/趴下/模式切换 |
| 发布 | `OBSTACLES_AVOID` | `{enable}` | 机载避障开关 |
| 发布 | `VUI` | 颜色/灯光 | LED 控制 |

**传感器流转换链：**

```
ULIDAR_ARRAY  → raw_lidar_stream() → pointcloud2_from_webrtc_lidar → lidar_stream()
ROBOTODOM     → raw_odom_stream()  → Odometry.from_msg               → odom_stream()
Video Track   → raw_video_stream()  → Image.from_numpy(RGB)           → video_stream()
```

所有 stream 均通过 `reactivex` Observable 暴露，并施加 `backpressure` 背压控制。

**订阅机制：** `unitree_sub_stream()` 将 Unitree DataChannel 回调包装为 Observable，通过 `call_soon_threadsafe` 在 WebRTC 事件循环线程中注册/注销订阅。

### 5.5 运动控制 `move()`

接收 DimOS 标准 `Twist` 消息，映射到 WebRTC 虚拟摇杆坐标：

```python
# DimOS Twist → WebRTC WIRELESS_CONTROLLER
# WebRTC 坐标系：
#   lx = -linear.y  (左右)
#   ly =  linear.x  (前后)
#   rx = -angular.z (旋转)
data = {"lx": -y, "ly": x, "rx": -yaw, "ry": 0}
```

**控制特性：**

- `duration == 0`：单次发送，连续运动；0.2 秒无新指令自动停止
- `duration > 0`：以 10ms 间隔持续发送，到期后停止
- 通过 `asyncio.run_coroutine_threadsafe()` 跨线程调度到 WebRTC 事件循环

### 5.6 运动模式 API

| 方法 | SPORT_MOD api_id | 说明 |
|------|------------------|------|
| `standup()` | `StandUp` | 站立 |
| `liedown()` | `StandDown` | 趴下 |
| `balance_stand()` | `BalanceStand` | 平衡站立（启用摇杆控制的前置状态） |
| `free_walk()` | `FreeWalk` | 自由行走模式 |
| `enable_rage_mode()` | `2059` + `SwitchJoystick` | Rage Mode（~2.5 m/s 速度上限） |
| `set_obstacle_avoidance()` | OBSTACLES_AVOID `1001` | 机载避障开关 |

### 5.7 `GO2Connection` 模块生命周期

**`start()` 完整时序：**

```
1. connection.start()                    # WebRTC 路径为空操作（已在 __init__ 连接）
2. 订阅 connection.lidar_stream()  → self.lidar.publish
3. 订阅 connection.odom_stream()   → self._publish_tf (发布 TF + odom)
4. 订阅 connection.video_stream()  → self.color_image.publish
5. 订阅 self.cmd_vel               → self.move
6. 启动 camera_info 发布线程（1Hz 静态内参）
7. self.standup()                        # 站立
8. time.sleep(3)                         # 等待站立完成
9. self.connection.balance_stand()      # 进入平衡站立
10. [若 RAGE 模式] enable_rage_mode()
11. set_obstacle_avoidance(config.g.obstacle_avoidance)
```

**`stop()` 时序：**

```
1. self.liedown()                        # 趴下
2. connection.stop()                     # 发送零速 + disconnect WebRTC
3. 等待 camera_info 线程退出
4. super().stop()
```

**对外暴露的 Stream：**

| Stream | 类型 | Transport |
|--------|------|-----------|
| `lidar` | `Out[PointCloud2]` | pSHMTransport |
| `color_image` | `Out[Image]` | pSHMTransport |
| `odom` | `Out[PoseStamped]` | LCM |
| `camera_info` | `Out[CameraInfo]` | LCM |
| `cmd_vel` | `In[Twist]` | LCM |

**TF 发布：** 里程计消息经 `_odom_to_tf()` 转换为 `base_link`、`camera_link`、`camera_optical` 三级变换链后发布。

---

## 6. 连接实现详解（DDS 备选路径）

用于 `unitree-go2-keyboard-teleop` 等 Blueprint，不走 WebRTC，直接通过 **Unitree SDK2 + CycloneDDS** 控制。

### 6.1 核心类 `UnitreeGo2TwistAdapter`

文件：`dimos/hardware/drive_trains/unitree_go2/adapter.py`

实现 `TwistBaseAdapter` 接口，3 DOF 速度控制 `(vx, vy, wz)`。

### 6.2 连接流程

```
connect()
  │
  ├── 1. ChannelFactoryInitialize(0)     # 初始化 DDS 域
  │
  ├── 2. MotionSwitcherClient.Init()
  │      └── 轮询 CheckMode() 最多 5s，等待 Sport 模式就绪
  │
  ├── 3. 订阅 rt/sportmodestate         # 遥测
  │
  ├── 4. SportClient.Init()
  │
  ├── 5. _initialize_locomotion()
  │      ├── StandUp()
  │      ├── FreeWalk()
  │      └── SpeedLevel()
  │
  └── 6. [可选] set_rage_mode(True)
        └── 发布 WirelessController_ 到 rt/wirelesscontroller_unprocessed
```

### 6.3 与 WebRTC 路径对比

| 维度 | WebRTC | DDS/SDK2 |
|------|--------|-----------|
| 依赖 | `unitree-webrtc-connect` | `unitree-sdk2py` + CycloneDDS |
| 安装 | `uv sync --extra unitree` | `uv pip install -e ".[unitree-dds]"` + Nix CycloneDDS |
| 传感器 | LiDAR + 相机 + 里程计 | 仅 `SportModeState` 遥测 |
| 控制 | DataChannel 虚拟摇杆 | SportClient API + DDS 发布 |
| 适用 | 完整导航/感知栈 | 纯速度遥操 |
| 网络 | 需 IP + 可选 AES 密钥 | 需 IP（DDS 组播发现） |

---

## 7. 其他连接模式

### 7.1 回放模式 `ReplayConnection`

- 触发：`--replay` 或 `ip="replay"`
- 数据源：`TimedSensorReplay`，默认数据集 `go2_short`
- 提供与 WebRTC 相同的 stream 接口（lidar / odom / video），但 `move()` 为空操作
- 不建立任何网络连接

### 7.2 MuJoCo 仿真 `MujocoConnection`

- 触发：`--simulation` 或 `ip="mujoco"`
- 通过 MuJoCo 物理引擎模拟 Go2
- 提供相同的传感器 stream 和控制接口

### 7.3 DimSim 仿真 `DimSimConnection`

- 触发：`--simulation dimsim`
- 通过 DimSim 远程仿真服务连接

### 7.4 多机编队 `Go2FleetConnection`

- 文件：`dimos/robot/unitree/go2/fleet_connection.py`
- 配置：`robot_ips="ip1,ip2,ip3"`（逗号分隔）
- 第一台为主机器人（完整传感器订阅），其余仅接收广播控制指令
- 每台机器人各自调用 `make_connection()` 建立独立 WebRTC 连接

---

## 8. Blueprint 与模块映射

以 `unitree-go2` 为例，运行时各模块职责：

| 模块 | 职责 |
|------|------|
| **GO2Connection** | WebRTC 连接，LiDAR/视频/里程计流，速度控制 |
| **VoxelGridMapper** | 3D 体素建图（CUDA 加速） |
| **CostMapper** | 3D 地图 → 2D 代价地图 |
| **ReplanningAStarPlanner** | 动态 A* 路径规划 |
| **WavefrontFrontierExplorer** | 自主 frontier 探索 |
| **MovementManager** | 路径跟踪与速度下发 |
| **vis_module** | Rerun / Foxglove 3D 可视化 |
| **ClockSyncConfigurator** | 时钟同步 |

Agent 版（`unitree-go2-agentic`）在此基础上额外挂载 `McpServer` + `McpClient` + Skill 容器。

**Blueprint 层级：**

```
unitree_go2
├── unitree_go2_basic
│   ├── vis_module
│   └── GO2Connection
├── VoxelGridMapper
├── CostMapper
├── ReplanningAStarPlanner
├── WavefrontFrontierExplorer
├── PatrollingModule
└── MovementManager
```

---

## 9. 故障排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `Timed out connecting to WebRTC at {ip}:9991` | IP 不可达 / 机器人未开机 | `ping $ROBOT_IP`，确认同一局域网 |
| 同上 + 新固件 | 缺少 AES 密钥 | 运行 `fetch_unitree_aes_key.py`，传入 `--unitree-webrtc-aes-key` |
| 连接成功但无法移动 | 未进入 BalanceStand | 检查日志中 `balance_stand()` 是否成功 |
| DDS 路径 `No sport mode active` | DDS 发现失败 | 确认 CycloneDDS 安装、`ROBOT_IP` 配置、防火墙 |
| 传感器无数据 | WebRTC 订阅失败 | 检查 DataChannel 是否建立，查看 `dimos log -f` |

**常用诊断命令：**

```bash
dimos status                    # 查看运行状态
dimos log -f                    # 实时日志
dimos show-config               # 查看解析后的 GlobalConfig
ping $ROBOT_IP                  # 网络连通性
dimos go2tool discover          # 重新发现机器人
```

---

## 10. 关键代码索引

```
dimos/
├── core/
│   └── global_config.py              # robot_ip, AES 密钥, 连接类型
├── robot/unitree/
│   ├── connection.py                 # UnitreeWebRTCConnection（WebRTC 实现）
│   ├── go2/
│   │   ├── connection.py             # GO2Connection 模块 + make_connection()
│   │   ├── fleet_connection.py       # 多机编队
│   │   ├── connection_spec.py        # GO2ConnectionSpec Protocol
│   │   ├── cli/
│   │   │   ├── go2tool.py            # BLE 配网 + LAN 发现 CLI
│   │   │   ├── ble.py                # BLE 扫描实现
│   │   │   └── landiscovery.py       # 局域网 mDNS/UDP 发现
│   │   └── blueprints/
│   │       ├── basic/unitree_go2_basic.py   # 挂载 GO2Connection
│   │       └── smart/unitree_go2.py         # 完整导航栈
│   └── type/
│       ├── lidar.py                  # WebRTC LiDAR → PointCloud2
│       └── odometry.py               # WebRTC Odom → Pose
├── hardware/drive_trains/unitree_go2/
│   └── adapter.py                    # UnitreeGo2TwistAdapter（DDS 路径）
└── scripts/
    └── fetch_unitree_aes_key.py      # 从 Unitree 云端获取 AES 密钥
```

---

## 11. 与 robot-brain 的关系

robot-brain 项目自身的 Go2 接入见 [`unitree-setup.md`](./unitree-setup.md)，采用 **DDS / unitree_sdk2_python** 路径。

DimOS 的 Go2 连接方案与之互补：

| 项目 | 主连接方式 | 特点 |
|------|-----------|------|
| **DimOS** | WebRTC | 无需编译 CycloneDDS，自带 LiDAR/相机/里程计流，完整导航栈 |
| **robot-brain** | DDS / SDK2 | 轻量，适合自定义控制逻辑，默认 `RDB_UNITREE_TRANSPORT=fake` 可离线开发 |

若 robot-brain 需要参考 DimOS 的 WebRTC 传感器接入或 Blueprint 架构，可对照本文档第 5 节实现细节。
