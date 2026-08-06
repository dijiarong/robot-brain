# Unitree Go2 接入指南

本文说明如何在 robot-brain 中连接宇树 Go2、读取状态，以及通过**操作者专用入口**进行低速限时操控。这些脚本**不接入**主服务 API、LLM 或技能层。

## 连接方式概览

| 方式 | Transport | 能力 | 依赖 |
|------|-----------|------|------|
| 内存模拟 | `fake` | 全功能 dry-run | 无 |
| WebRTC（推荐） | `webrtc` | 状态 + 姿态 + 低速 drive | `unitree-webrtc-connect` |
| CycloneDDS SDK | `sdk` | **只读**状态 | `unitree_sdk2_python` + CycloneDDS |

开发与实机验证优先使用 **WebRTC + 同 LAN（STA）**；机器狗与开发机在同一 Wi-Fi/路由器下即可，不必连接 Go2 热点。

技术细节可参考 [DimOS 连接宇树 Go2 技术文档](./dimos-go2-connection.md)。

## 安装

```bash
# 基础项目
pip install -e .

# WebRTC 实机（云端 4G 或局域网；不安装 DDS SDK）
pip install -e ".[unitree-webrtc]"

# 只有同时需要 WebRTC 和 CycloneDDS SDK 时才使用：
# pip install -e ".[unitree]"
```

未安装 WebRTC 依赖时，使用 `RDB_UNITREE_TRANSPORT=fake` 仍可跑完全部测试与 dry-run 示例。

### SDK 路径（只读，可选）

```bash
brew install cmake
pip install cyclonedds
pip install git+https://github.com/unitreerobotics/unitree_sdk2_python.git
```

SDK transport 在本仓库中**仅开放只读**，不能下发动作。

## 网络与 IP

1. Go2 上电，确认与开发机在同一 LAN（或连接 Go2 热点 AP 模式）。
2. 获取机器人 IP：Unitree App 设备信息、`dimos go2tool discover`（若已装 DimOS），或路由器 DHCP 列表。
3. AP 模式默认 IP 常为 `192.168.123.161`；STA 模式为路由器分配的地址（如 `10.10.x.x`）。
4. WebRTC 信令端口：`9991`（连接前会做可达性探测）。

```bash
export RDB_UNITREE_ROBOT_IP=10.10.196.239
# 兼容 DimOS 命名：
# export DIMOS_ROBOT_IP=...
# export ROBOT_IP=...
```

固件 ≥ 1.1.15 可能还需要 AES 密钥：

```bash
export UNITREE_AES_128_KEY=<32-hex>
```

### 通过 4G / 宇树云端连接（不需要机器人 IP）

机器人不在同一局域网、但已通过 4G 在线时，可以用宇树账号和设备序列号走
WebRTC Remote：

```bash
export RDB_UNITREE_TRANSPORT=webrtc
export RDB_UNITREE_WEBRTC_CONNECTION_MODE=remote
export RDB_UNITREE_SERIAL=<设备序列号>
export RDB_UNITREE_CLOUD_USERNAME=<宇树账号>
export RDB_UNITREE_CLOUD_PASSWORD=<宇树密码>
export RDB_UNITREE_CLOUD_REGION=cn
```

密码只从环境变量读取，不要写入仓库、`.env` 示例或命令输出。中国区注册账号使用
`cn`；海外账号使用 `global`。`auto` 模式下，有显式机器人 IP 时优先局域网；没有
IP 且序列号、账号、密码齐全时自动切换云端。

## 环境变量

### 基础

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `RDB_ROBOT` | `mock` | 设为 `unitree` 启用 Unitree 适配器 |
| `RDB_UNITREE_TRANSPORT` | `fake` | `fake` / `sdk` / `webrtc` |
| `RDB_UNITREE_WEBRTC_CONNECTION_MODE` | `auto` | `auto` / `local` / `remote`；远程 4G 使用 `remote` |
| `RDB_UNITREE_MODEL` | `go2` | 机型标识 |
| `RDB_UNITREE_ROBOT_IP` | 空 | WebRTC LAN IP；亦读 `UNITREE_ROBOT_IP`、`DIMOS_ROBOT_IP`、`ROBOT_IP` |
| `RDB_UNITREE_SERIAL` | 空 | 局域网发现可选；云端 Remote 必填 |
| `RDB_UNITREE_CLOUD_USERNAME` | 空 | 宇树云账号；Remote 必填 |
| `RDB_UNITREE_CLOUD_PASSWORD` | 空 | 宇树云密码；Remote 必填，禁止提交仓库 |
| `RDB_UNITREE_CLOUD_REGION` | `global` | 中国区账号设 `cn`，海外账号用 `global` |
| `RDB_UNITREE_CLOUD_DEVICE_TYPE` | `Go2` | 云端 AppName/设备类型 |
| `RDB_UNITREE_LIDAR_STREAM` | `false` | 显式请求自带 LiDAR 点云；`direct_go2` 导航会自动开启 |
| `RDB_UNITREE_LIDAR_ALLOW_UNCOMPRESSED` | `false` | 诊断未压缩点云回退；4G 流量较大，不建议常开 |
| `RDB_UNITREE_VIDEO_RELAY` | `true` | Go2 相机 → 本地 ffmpeg RTP（topsun）；与 Mid-360 同机时务必 `false` |
| `RDB_UNITREE_AUDIO_RELAY` | `true` | 双向音频 ffmpeg RTP；同上 |
| `RDB_UNITREE_MEDIA_ON_DEMAND` | `false` | `true` 时连接不启 ffmpeg，需 `POST /api/media/relays/start` 或 `ensure_media_relays()` |
| `RDB_UNITREE_NET_IFACE` | 空 | SDK 用 CycloneDDS 网卡名（如 `en0`），**不是**机器人 IP |
| `RDB_UNITREE_MOTION_MODE` | `mcf` | WebRTC 连接后 motion switcher 模式 |

### 安全门与限速（第九次迭代默认值）

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `RDB_UNITREE_DRY_RUN` | `true` | `false` 才向真机下发命令 |
| `RDB_UNITREE_ENABLE_MOTION` | `false` | **硬安全门**；未开启时拒绝姿态/平移（`stop` 在已连接时仍允许） |
| `RDB_UNITREE_MAX_SPEED` | `0.2` | 线速度上限 (m/s) |
| `RDB_UNITREE_MAX_YAW_SPEED` | `0.3` | 角速度上限 (rad/s) |
| `RDB_UNITREE_MAX_DRIVE_DURATION` | `0.5` | 单次 drive 最长时长 (s) |
| `RDB_UNITREE_MAX_STEP` | `2.0` | 适配器层步长上限 (m) |

### 控制闭环

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `RDB_UNITREE_CONTROL_WATCHDOG_SECONDS` | `0.25` | 控制帧续发 watchdog |
| `RDB_UNITREE_ZERO_FRAME_COUNT` | `5` | 停止时连续零速帧数 |
| `RDB_UNITREE_STATE_MAX_AGE_SECONDS` | `2.0` | 运动前状态最大陈旧时间 |
| `RDB_UNITREE_POST_DRIVE_STOP_TIMEOUT` | `3.0` | drive 后等待停稳超时 |
| `RDB_UNITREE_WEBRTC_CONNECT_TIMEOUT` | `30.0` | WebRTC 连接超时 |
| `RDB_UNITREE_WEBRTC_DRIVE_VIA_MOVE` | `true` | MCF 下纯平移/纯旋转使用 Move(1008)；平移与 yaw 组合弧线自动改走虚拟摇杆 |

## 验证流程

### 0. 空间记忆离线闭环

无需 ROS2 或真机，验证所有房间/物品移动均经过可替换 Navigation Provider：

```bash
python scripts/verify_spatial_memory_navigation.py
```

报告需满足 `ok=true` 且 `forbidden_direct_motion=[]`，导航记录应包含
`set_absolute_goal` 和目标途中命中后的 `cancel`。

### 1. Fake / dry-run（无需真机）

```bash
python -m examples.run_unitree_smoke --transport fake
python -m examples.run_unitree_teleop --transport fake
python -m examples.run_unitree_teleop_web --transport fake
```

### 2. WebRTC 只读

```bash
export RDB_UNITREE_ROBOT_IP=<ip>
python -m examples.run_unitree_smoke --transport webrtc --state-only
```

自带 LiDAR + odom 局部导航只读验收（不会发送运动）：

```bash
export RDB_UNITREE_ROBOT_IP=<ip>
python scripts/verify_direct_go2_navigation.py
```

成功报告必须同时满足：`sensor_snapshot.ready=true`、`pose_source` 为
`unitree_robotodom`、点数大于零、`obstacle_frame=base_link`。导航位姿订阅
Go2 `rt/utlidar/robot_pose`，SportModeState.position 只作为诊断回退，默认不允许
驱动局部导航。若原始点云为 `world`，只有点云携带的原点与
Go2 odom 一致时才会安全转换到 `base_link`，否则报告
`untrusted_obstacle_frame`。

### 3. WebRTC 姿态（需 motion gate）

```bash
export RDB_UNITREE_ROBOT_IP=<ip>
RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_unitree_smoke \
    --transport webrtc --actions --live
```

启动时会要求输入确认短语。

局部导航小步 live 验收应在姿态和只读 LiDAR 验收通过之后执行：

```bash
export RDB_UNITREE_ROBOT_IP=<ip>
python scripts/verify_direct_go2_navigation.py \
  --live --confirm I_UNDERSTAND_DIRECT_GO2_NAV --forward-m 0.1
```

脚本以 0.25 秒小段执行，每段前重新检查点云。点云/odom 过期、路径中出现
障碍、连续无位移或超时都会停止，输出结构化 `final_state.error_code`。

### 4. 分级真机验收（Level 0–5）

```bash
RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_unitree_smoke \
    --transport webrtc --graded --live --level 0
```

逐级通过后再提高 `--level`；任一级失败即停止。

### 5. 终端 teleop（离散 nudge）

```bash
RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_unitree_teleop \
    --transport webrtc --live --robot-ip <ip>
```

键位：W/S 前后，A/D 侧移，Q/E 原地转；空格急停。`--strong` 提高 nudge 幅度（仍受 settings clamp）。

### 6. Web 面板 teleop（按住连续运动）

```bash
RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_unitree_teleop_web \
    --transport webrtc --live --strong
```

- 默认 **车式键位**：W/S 前后，**A/D 转弯**，Q/E 平移；W+D 为前进+右转弧线。
- `--omni` 恢复全向映射（A/D 侧移，Q/E 转向）。
- 浏览器打开 `http://127.0.0.1:8765/`；松开键归零，STOP/空格急停。
- 状态栏会显示合成速度与通道（如 `joystick (arc)`）。

连接后会自动执行 prep：`stand_up → balance_stand → free_walk → SwitchJoystick + SpeedLevel`。

## 控制通道说明（MCF 固件）

Go2 MCF 模式下速度下发采用**混合策略**（见 `UnitreeWebRTCTransport`）：

| 运动类型 | 通道 | 说明 |
|----------|------|------|
| 纯前进/后退 | Move(1008) | `rt/api/sport/request` fire-and-forget |
| 侧移 (vy) | Move(1008) | 虚拟摇杆 lx 在 MCF 上常无效 |
| 含转向 (vyaw)，含 W+D 弧线 | 虚拟摇杆 | `rt/wirelesscontroller`，与 DimOS 一致 |

停止时：`release` 只发零帧；`stop` 零帧 + `StopMove`。Web 按住操控使用 `stream_hold` 连续 50Hz 流，避免分片归零造成卡顿。

## Mid-360 Navigation 联调（Orin + 边上 brain）

默认全开的 brain（WebRTC + ffmpeg 音视频中继 + 可选 native/VLM）不宜与狗上
Livox + Super-LIO + Nav2 同机硬叠。推荐：

1. **Orin**：只跑 Navigation（`config/profiles/orin-nav-only.env` 明示拒绝 brain 媒体栈）。
2. **边上机 / 笔记本**：lean brain + `RDB_NAVIGATION_BACKEND=nav2`，同 Wi‑Fi、同 `ROS_DOMAIN_ID`。

```bash
# Orin：启动导航栈后采资源基线（确认无 ffmpeg / robot-brain）
./scripts/collect_orin_nav_baseline.sh

# 边上机：加载 lean 配置
./scripts/run_with_profile.sh edge-brain-lean python -m examples.run_service

# 只读：action / odom / localization
./scripts/run_with_profile.sh edge-brain-lean python scripts/verify_nav2_provider.py

# 控制面：状态、随时 cancel、cancel 后再发 goal（默认可恢复）
./scripts/run_with_profile.sh edge-brain-lean \
  python scripts/verify_nav2_control_surface.py --read-only
```

关键开关：

| 变量 | lean 建议 | 说明 |
|------|-----------|------|
| `RDB_NAVIGATION_BACKEND` | `nav2` | 经 `/navigate_to_pose` 间接动腿 |
| `RDB_UNITREE_VIDEO_RELAY` / `AUDIO_RELAY` | `false` | 不启 ffmpeg |
| `RDB_UNITREE_MEDIA_ON_DEMAND` | `true` | 需要媒体时再 `POST /api/media/relays/start` |
| `RDB_UNITREE_LIDAR_STREAM` / `RDB_VLM_ENABLED` | `false` | 避免与 Mid-360 抢 CPU |

控制面保留：`GET/WS` 导航状态、`POST /api/navigation/cancel`、map-goal / Nav2 goal 可重复下发。
进程内遥操作走共享 `MotionAuthority`（dashboard / gRPC / gateway 同一 `TeleopSession`），避免多租约抢电机。

## 回退到 Mock

```bash
unset RDB_ROBOT RDB_UNITREE_TRANSPORT RDB_UNITREE_ENABLE_MOTION
# 或
export RDB_ROBOT=mock
export RDB_UNITREE_TRANSPORT=fake
```

## 故障排查

| 症状 | 排查 |
|------|------|
| `Cannot reach Go2 WebRTC at …:9991` | IP 错误、不在同网段、狗未上电；检查 App IP |
| WebRTC 连接超时 | AES 密钥、防火墙；增大 `RDB_UNITREE_WEBRTC_CONNECT_TIMEOUT` |
| `Motion disabled` / motion gate | 设置 `RDB_UNITREE_ENABLE_MOTION=true` |
| `robot not ready for drive` | 先 prep 站立/FreeWalk；Web teleop 会自动 prep |
| `stale robot state` | 状态 topic 未订阅成功；断连重连 |
| 只能前进、不能侧移/转 | 确认已 `free_walk` + `SwitchJoystick`；侧移需 Move，转向需摇杆 |
| W+D 像直走 | 确认面板显示 `joystick (arc)` 且 `vyaw≠0`；需本次迭代混合通道 |
| SDK `State read failed` | 检查 `RDB_UNITREE_NET_IFACE`、CycloneDDS、同网段 |

## 安全注意事项

- **默认配置不会动真机**：`DRY_RUN=true`、`ENABLE_MOTION=false`。
- 真实运动前：平坦开阔地面、2m+ 安全距离、关闭 App/遥控器并发控制、手边物理急停。
- 确认短语：`I_UNDERSTAND_UNITREE_MOVE`（smoke）、`I_UNDERSTAND_UNITREE_TELEOP`（teleop/web）、`I_UNDERSTAND_UNITREE_GRADED_ACCEPTANCE`（分级验收）。
- `drive` / teleop **未**注册为 LLM skill；主服务 `robot-brain-service` 不会直接驱动 Go2。
- 异常时系统 best-effort `stop`；断连前会尝试归零 + StopMove。

## 相关文档

- [第九次迭代：实机操控安全闭环](./plans/2026-06-11-172047-unitree-live-control-loop.md)
- [第八次迭代：WebRTC 姿态/急停](./plans/2026-06-08-141602-unitree-webrtc-posture-actions.md)
- [DimOS Go2 连接参考](./dimos-go2-connection.md)
