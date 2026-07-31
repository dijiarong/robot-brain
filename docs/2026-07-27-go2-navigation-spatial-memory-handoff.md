# Go2 导航与空间物品记忆：开发交接和实机测试

更新时间：2026-07-30

## 1. 目标

在 `robot-brain` 中实现接近 `topsun_dimos` 的能力：

- 统一 Navigation Provider，隔离业务技能与具体导航实现；
- 使用 Go2 自带里程计和 LiDAR 做安全受限的局部导航；
- 可选接入外部 Nav2，提供全局导航和持久地图身份；
- 让空间物品记忆通过 Navigation Provider 返回已记录位置；
- 支持局域网 WebRTC 和宇树账号/密码/序列号的 4G Remote WebRTC。

### 1.1 原始 `/goal`

Goal ID：`019fa248-8281-7171-b5aa-64edd04a001f`

原始目标全文：

> 在 robot-brain 中实现基于可替换 Navigation Provider 的 Go2 空间物品记忆完整闭环：第一阶段利用 Go2 自带 LiDAR、里程计和现有 Navigation/Nav2 能力完成安全的局部导航、动态避障与真机验收；第二阶段增加 map 坐标绝对目标、定位/重定位状态和地图身份契约；第三阶段将房间、物品观察点和跨房间查找重构为地图/会话绑定的持久记忆，所有真机移动经导航 Provider 执行，并保留后续接入 Mid-360/Super-LIO 或其他 SLAM 后端的能力。

**2026-07-30 结论：** 第一阶段真机验收已收口（见 §1.4）。第二阶段默认路径（L1 建图 + AMCL + map 绝对目标 + `/cmd_vel`→Move）已真机收口：同图 `--live-absolute` 出现 `ok=true` / `succeeded`。**Mid-360 / Super-LIO 仅为可选增强后端**。当前主缺口在第三阶段真机「记点→离开→返回」与重定位触发 API。

### 1.2 Goal 分阶段状态

| 阶段 | 原计划 | 当前状态 | 还缺什么 |
|---|---|---|---|
| 第一阶段 | Go2 自带 LiDAR + odom，局部导航、动态避障、真机验收 | **真机验收通过（2026-07-29）** | 无阻塞；精度/速度不作为本阶段硬指标 |
| 第二阶段 | map 绝对目标、定位/重定位、地图身份 | **L1 建图 + AMCL + live-absolute 真机通过（2026-07-30）** | 重定位触发 API 仍缺；长距/绕障可另补 |
| 第三阶段 | 房间/物品观察点持久化、跨房间查找 | 离线闭环已完成；**`--live-return` 脚本已就绪** | 真机跑通记点返回；完整 VLM 扫房间仍可选 |
| 后续扩展（可选） | Mid-360 / Super-LIO / 其他 SLAM | 接口预留；**非必选** | 有外接雷达时再换更强定位后端，RB 契约不变 |

### 1.4 第一阶段真机验收结论（2026-07-29）

连接方式：宇树 **4G Remote WebRTC**（账号 / 序列号 / 密码环境变量）。

| 验收项 | 结果 | 备注 |
|---|---|---|
| Remote 连接 / SportState / 电量 | 通过 | Data Channel Verification OK |
| 权威 odom（`rt/utlidar/robot_pose`） | 通过 | `pose_source=unitree_robotodom` |
| 压缩点云（`voxel_map_compressed`） | 通过 | 曾被坐标系门禁拦住，修复后 `ready=true` |
| 传感器只读 `ok` / `obstacle_frame=base_link` | 通过 | |
| ~10 cm 前进（真迈步） | 通过 | 有过超调；不继续抠精度 |
| 原地旋转 ~10–15° | 通过 | |
| 遇障停止（前方人/障碍） | 通过 | `error_code=obstacle`，停在人前约十几 cm，未撞 |
| 取消 / Ctrl+C 急停 | 通过 | 须在**连续长推**中途打断才好观察；短段导航难看出 |

本阶段**不宣称**厘米级到位精度或匀速美观步态；宣称的是：能连、能感知、能短距动、能避障停、能急停。

### 1.3 本轮实际改动范围

主要新增或修改：

- `robot_brain/navigation/base.py`：Provider 契约、绝对/相对目标、定位状态、地图身份；
- `robot_brain/navigation/direct_go2.py`：Go2 短分段局部导航（累计进度、odom settle、段时长）；
- `robot_brain/navigation/sensors.py`：odom/点云新鲜度；**odom 帧点云转 base_link**（忽略体素 AABB origin）；
- `robot_brain/navigation/nav2.py`：Nav2 绝对目标、定位与地图身份；
- `robot_brain/perception/pointcloud.py`：Unitree WebRTC 点云标准化；
- `robot_brain/actuation/unitree_webrtc.py`：Remote、ROBOTODOM、LiDAR；**Ctrl+C 时取消后台 drive future**；
- `robot_brain/memory/spatial.py`：地图/会话绑定的空间记忆数据；
- `robot_brain/skills/builtin/spatial_memory.py`：房间/物品导航全部经 Provider；
- `robot_brain/skills/builtin/navigation.py` 和 `robot_brain/tools/builtin/navigation.py`：导航技能/工具；
- `robot_brain/runtime/loop.py`：按配置组装 Fake、Nav2 或 direct_go2；
- `scripts/verify_direct_go2_navigation.py`：只读 / live / `--gait-probe` / `--cancel-probe`；
- `scripts/verify_spatial_memory_navigation.py`：空间记忆离线闭环验收；
- 对应导航、传感器、空间记忆和 WebRTC 测试。

### 1.5 第一阶段踩坑总结

1. **安装 extra 选错**  
   必须 `.[unitree-webrtc]`，不要用 `.[unitree]`（会拉 DDS/GitHub，网络差易整装失败）。国内可用 pip 镜像。

2. **点云「收不到」→ 后来能收到**  
   交接初判 Remote 点云计数为 0；2026-07 真机复测时压缩点云已通。若再现 0，再比 DimOS Remote 订阅时序；不要先改 decoder。

3. **`untrusted_obstacle_frame`（frame=`odom`）**  
   归一化原先只处理 `world`。真机 voxel 是 `odom` 帧 → 需转到 `base_link`。

4. **体素 `origin` 不是机器人位姿**  
   真机常见 `origin≈(-3.225,-3.225,…)` 为体素格角点。若拿它和 `robot_pose` 比会误拦。**`odom` 帧一律用当前位姿做转换，忽略 AABB origin。**

5. **设备时间戳不可靠**  
   `sensor_timestamp_valid=false`、大量 `lidar_timestamp_repair_count` 属常见现象；靠本地修补，不单独阻塞。

6. **站桩前倾不迈步 / `no_progress`**  
   短段 + 零速打断步态；`sport_mode` 遥测常为 0 不可靠。有效做法：备步 `stand_up → balance_stand → free_walk → enable_omni_teleop`，`Move(1008)`，`vx≈0.35`，段时长拉长，累计进度 + odom settle。

7. **`SpeedLevel=2` 被拒**  
   本机路径上 `SpeedLevel=1` 可用；`=2` 曾返回 `-1`。

8. **Ctrl+C 看不出急停**  
   短段导航缝隙里打断观感差。根因之一：取消主协程时 **WebRTC 后台 `run_coroutine_threadsafe` 的 velocity stream 仍在推**。已改为 `wrap_future` + cancel 时 `fut.cancel()` 并 bump `motion_gen`。验收用 `--cancel-probe` 连续长推再中途 stop。

9. **避障是机载 LiDAR 走廊，不是视觉/全局规划**  
   前方约停障距离 / 半宽 / 高度带在 `direct_go2` 配置；人挡在前方可触发 `obstacle`。

### 1.6 第一阶段有效运行命令（备忘）

工作目录与 venv：

```bash
cd /home/dijia/project/Robot-Brain
source .venv/bin/activate
# 若尚未安装：
# python -m pip install -e ".[unitree-webrtc]"
```

Remote 凭据（密码勿入库）：

```bash
export RDB_UNITREE_TRANSPORT=webrtc
export RDB_UNITREE_WEBRTC_CONNECTION_MODE=remote
export RDB_UNITREE_SERIAL="<设备序列号>"
export RDB_UNITREE_CLOUD_USERNAME="<宇树账号>"
export RDB_UNITREE_CLOUD_REGION="cn"
export RDB_UNITREE_CLOUD_DEVICE_TYPE="Go2"
export RDB_UNITREE_WEBRTC_CONNECT_TIMEOUT="60"
export RDB_UNITREE_CLOUD_PASSWORD  # 交互或自行 export，勿提交

export RDB_NAVIGATION_BACKEND=direct_go2
export RDB_UNITREE_LIDAR_STREAM=true
```

只读传感器：

```bash
export RDB_UNITREE_DRY_RUN=true
export RDB_UNITREE_ENABLE_MOTION=false
python scripts/verify_direct_go2_navigation.py --sensor-timeout-s 30
```

期望：`ok=true`，`ready=true`，`obstacle_frame=base_link`，点云 `*_to_base`。

Live 运动公共开关（测完立刻改回 dry-run）：

```bash
export RDB_UNITREE_DRY_RUN=false
export RDB_UNITREE_ENABLE_MOTION=true
export RDB_UNITREE_WEBRTC_DRIVE_VIA_MOVE=true
export RDB_UNITREE_MAX_SPEED=0.35
export RDB_UNITREE_MAX_DRIVE_DURATION=3.0
export RDB_DIRECT_NAV_SEGMENT_DURATION_S=2.0
```

10 cm / 小目标前进：

```bash
python scripts/verify_direct_go2_navigation.py \
  --live --confirm I_UNDERSTAND_DIRECT_GO2_NAV \
  --forward-m 0.10 --timeout-s 15 --sensor-timeout-s 30
```

原地旋转：

```bash
python scripts/verify_direct_go2_navigation.py \
  --live --confirm I_UNDERSTAND_DIRECT_GO2_NAV \
  --forward-m 0 --yaw-degrees 15 --timeout-s 15 --sensor-timeout-s 30
```

遇障（前方站人，再发前进）：

```bash
python scripts/verify_direct_go2_navigation.py \
  --live --confirm I_UNDERSTAND_DIRECT_GO2_NAV \
  --forward-m 1.0 --timeout-s 20 --sensor-timeout-s 30
```

步态探测（连续前进，人眼看是否迈步）：

```bash
python scripts/verify_direct_go2_navigation.py \
  --live --confirm I_UNDERSTAND_DIRECT_GO2_NAV \
  --gait-probe --gait-probe-vx 0.35 --gait-probe-duration-s 3.0
```

急停探测（连续长推约 2.5 s 后自动 STOP；也可中途 Ctrl+C）：

```bash
export RDB_UNITREE_MAX_DRIVE_DURATION=8.0
python scripts/verify_direct_go2_navigation.py \
  --live --confirm I_UNDERSTAND_DIRECT_GO2_NAV \
  --cancel-probe
```

测完恢复：

```bash
export RDB_UNITREE_DRY_RUN=true
export RDB_UNITREE_ENABLE_MOTION=false
unset RDB_UNITREE_CLOUD_PASSWORD
```

前置提醒：平整空旷；人可急停；退出 Unitree App 及其他 WebRTC 客户端。

## 2. 已完成

### 2.1 导航抽象

- `NavigationClient` 支持相对目标、绝对目标、取消、状态和定位状态。
- `FakeNavigationClient` 用于离线测试。
- `Nav2NavigationClient` 支持 ROS2 `NavigateToPose`、里程计、地图身份和取消。
- `DirectGo2NavigationClient` 将相对目标拆成短速度段，每段重新检查传感器。
- 导航过程包含超时、取消、障碍、里程计无进展和传感器过期保护。

### 2.2 Go2 自带传感器

- WebRTC 订阅 `rt/utlidar/robot_pose` 作为权威 session-local 里程计。
- 订阅 `rt/utlidar/voxel_map_compressed`，可选诊断未压缩点云。
- 订阅 `rt/utlidar/lidar_state` 并输出独立健康计数。
- 导航默认拒绝使用 `SportModeState.position` 替代权威里程计。
- 点云必须新鲜且处于可信的机器人相对坐标系；`odom`/`world` 点云安全转换到 `base_link`（`odom` 忽略体素 AABB origin，改用 `robot_pose`）。

### 2.3 空间物品记忆

- 房间、物品和观察位置可以保存地图身份和位姿信息。
- `go_to_room`、`go_to_object` 等返回动作经过 Navigation Provider，不再直接拼接机器人运动。
- 导航途中识别到目标后会取消旧目标，避免继续驶过物品。
- session-local Go2 odom 和持久 Nav2 地图区分处理，避免跨重启误用坐标。

### 2.4 WebRTC Remote

支持三种连接选择：

- `local`：局域网 `LocalSTA`；
- `remote`：宇树云 `Remote`，使用账号、密码、序列号和区域；
- `auto`：有显式 IP 时优先 local；否则凭据完整时使用 remote。

密码只从环境变量读取，`Settings` 的密码字段不参与 `repr`，连接日志不打印账号、密码或 token。

### 2.5 已验证结果

自动测试最后一次完整结果：

```text
549 passed, 4 skipped, 11 subtests passed
```

随后新增 LiDAR 诊断的相关测试结果：

```text
68 passed, 11 subtests passed
```

实机 4G Remote（2026-07-29 更新）：

- 云端登录和 TURN/ICE 成功；
- Data Channel Verification 成功；
- SportState / LowState / 电量可读；
- `robot_pose` 与压缩点云均可收到；导航传感器 `ready=true`、`obstacle_frame=base_link`；
- 第一阶段运动/避障/急停验收见 §1.4；有效命令见 §1.6。

## 3. 尚未完成

### 3.1 4G Remote 点云（历史卡点，已解除）

原文档记录 Remote 点云计数为 0、需对齐 DimOS。**2026-07 真机复测点云已通**；剩余问题是坐标系归一化（已修，见 §1.5）。若日后再次出现 `lidar_*_message_count=0`，再按下列对比 DimOS 订阅时序；否则不要回到改 decoder。

### 3.2 第一阶段真机运动验收（已完成）

§1.4 所列运动项已在真机通过。空间记忆「返回房间/物品」属第三阶段真机项，仍未做。

### 3.3 全局导航和跨重启重定位（第二阶段主线）

`DirectGo2NavigationClient` 只提供 session-local 短距离导航，不是全局规划器。跨重启空间物品记忆需要 **真实 `map→odom`**，默认用狗自带雷达，不强制外接激光：

1. **默认（推荐当前推进）**：Go2 L1 → `/scan` + odom → 2D 建图（如 `slam_toolbox`）→ 保存地图 → **AMCL** 重定位 → Nav2；
2. **会话冒烟（已做）**：`map≡odom` 单位阵，只验动狗与 Provider 契约，**不能**当持久定位；
3. **可选增强**：Mid-360 + Super-LIO / 其他 3D SLAM——输出同一套 TF/`navigate_to_pose` 即可接入，**不是框架特殊要求**。

Robot-Brain 只消费 Provider（位姿、绝对目标、`MapIdentity`）；定位后端可替换。

第二阶段代码侧已有：`AbsoluteNavigationGoal`、`LocalizationState`、`MapIdentity`、`Nav2NavigationClient`。  
真机进度（2026-07-30）：L1→Nav2→Move 已前进；仍缺稳定 `succeeded`、以及默认路径上的 L1 建图/AMCL；仓库尚无独立「重定位触发」API（仅能读状态）。

### 3.4 第二阶段第一步（只读，2026-07-29 起）

前置：本机或容器已 source Navigation（ROS2 Humble）工作区，Nav2 Action、`/odom`、`map→base_link` TF 可用。

```bash
export RDB_NAVIGATION_BACKEND=nav2
export RDB_NAV2_ACTION_NAME=/navigate_to_pose
export RDB_NAV2_ODOM_TOPIC=/odom
export RDB_NAV2_GOAL_FRAME=odom
export RDB_NAV2_MAP_FRAME=map
export RDB_NAV2_MAP_ID=<stable-map-id>    # 绝对目标必需；不设则 supports_absolute_goals=false
export RDB_NAV2_MAP_VERSION=v1            # 可选

# 默认只读：action + odom + localization + map identity
python scripts/verify_nav2_provider.py
```

期望：`ok=true`，有 odom pose；localization `status=localized`（若 map TF 可用）；配置了 `MAP_ID` 时 identity 可用。  
**先不要** `--live`；只读通过后再做短距相对/绝对目标。

### 3.5 Go2 自带雷达 → ROS2 / Nav2（默认路径，2026-07-30）

**默认用狗 L1，不需要 Mid-360。** WebRTC 桥发 `/odom` `/scan` `/points` + TF；可选 `--enable-cmd-vel` 把 Nav2 速度回灌 Go2 Move。

```bash
# 终端 A：WebRTC 桥 + Nav2（运动关闭）
source /opt/ros/jazzy/setup.bash
source /home/dijia/project/Robot-Brain/.venv-jazzy/bin/activate
# 先 export Remote 凭据 + RDB_UNITREE_LIDAR_STREAM=true ...
bash /home/dijia/project/Navigation/scripts/phase2_go2_lidar_nav2.sh
```

```bash
# 终端 B
source /opt/ros/jazzy/setup.bash
source /home/dijia/project/Robot-Brain/.venv-jazzy/bin/activate
cd /home/dijia/project/Robot-Brain
ros2 topic hz /scan
export RDB_NAVIGATION_BACKEND=nav2
export RDB_NAV2_MAP_ID=lab-floor1
export RDB_NAV2_MAP_VERSION=v1
python scripts/verify_nav2_provider.py
```

桥接脚本：`scripts/go2_webrtc_ros_bridge.py`（`/odom`、`/scan`、`/points`、TF）。默认不动狗。

要把 Nav2 `/cmd_vel` 发回狗（空旷场地）：

```bash
# 终端 A（会动狗）
export RDB_UNITREE_ENABLE_MOTION=true
export RDB_UNITREE_WEBRTC_DRIVE_VIA_MOVE=true
export RDB_UNITREE_MAX_SPEED=0.35
ENABLE_CMD_VEL=1 bash /home/dijia/project/Navigation/scripts/phase2_go2_lidar_nav2.sh
```

```bash
# 终端 B（机体前向偏移；禁倒车已在桥内默认开启）
python scripts/verify_nav2_provider.py --live-absolute --absolute-dx-m 0.3 --timeout-s 60
```

期望：狗迈步前进；理想 `succeeded`。若只前进中途 abort，先查代价地图 / `out of map bounds`，再调大 local costmap。

**2026-07-30 真机收口：** `--live-absolute --absolute-dx-m 0.3` 已出现 `ok=true` / `status=succeeded`（机体前向目标 + 禁倒车 + L1→Nav2→Move）。位移相对目标仍偏短（progress≈0.37），但 Provider 链路与动狗验收通过。下一步：L1 2D 建图 + AMCL，换掉会话内 `map≡odom`。

**定位下一步（仍用 L1，无 Mid-360）：** 用同一 `/scan`+`/odom` 跑 2D 建图并保存，再 AMCL 加载该图，使 `map→odom` 由定位维护；RB 侧 `map_id` 绑这张图。Mid-360 仅作日后可选换后端。

### 3.6 L1 建图 + AMCL（默认定位路径，2026-07-30）

先 **Ctrl+C 停掉** 旧的 `phase2_go2_lidar_nav2.sh`（它会发假 `map≡odom`，和 SLAM/AMCL 冲突）。

**A. 建图（慢速绕房间一圈，尽量闭环）**

```bash
# 终端 A
export RDB_UNITREE_ENABLE_MOTION=true
# …其余 Remote 凭据…
bash /home/dijia/project/Navigation/scripts/phase2_go2_lidar_mapping.sh
```

```bash
# 终端 B：键盘遥控（桥会抬升到可迈步速度）
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

绕完后存图：

```bash
source /opt/ros/jazzy/setup.bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/dijia/project/Navigation/navigation/src/bringup/maps/lab_floor1
```

**B. 定位 + Nav2（用刚存的图）**

```bash
# 终端 A（可先只读；动狗再加 ENABLE_CMD_VEL=1）
MAP_YAML=/home/dijia/project/Navigation/navigation/src/bringup/maps/lab_floor1.yaml \
  bash /home/dijia/project/Navigation/scripts/phase2_go2_lidar_localize.sh
```

在建图起点附近发一次 `/initialpose`（脚本会打印示例），然后：

```bash
# 终端 B
export RDB_NAVIGATION_BACKEND=nav2 RDB_NAV2_MAP_ID=lab-floor1 RDB_NAV2_MAP_VERSION=v1
python scripts/verify_nav2_provider.py
# 确认 tf map→odom 由 AMCL 维护后再 --live-absolute
```

脚本：`phase2_go2_lidar_mapping.sh`、`phase2_go2_lidar_localize.sh`；桥用 `--no-publish-map-tf`。旧的 `phase2_go2_lidar_nav2.sh` 仍可用于会话内假定位冒烟。

**2026-07-30 定位只读收口：** 已存 `lab_floor1` 图；`phase2_go2_lidar_localize.sh` + `/initialpose` 后 `verify_nav2_provider.py` 出现 `ok=true`、`localization.status=localized`、`usable_for_persistent_memory=true`（map 位姿与 odom 位姿可不同，说明 AMCL 在维护 `map→odom`）。

**2026-07-30 同图 live-absolute 收口：** `ENABLE_CMD_VEL=1` 下 `--live-absolute --absolute-dx-m 0.3` 出现 `ok=true` / `succeeded`。`phase2_go2_lidar_localize.sh` 现为 **先等 `/amcl_pose`（须设 `/initialpose`）再起 Nav2**，避免 costmap 超时 Aborting。

### 3.7 第三阶段真机：记点 → 离开 → 返回（Nav2）

前置：终端 A 已跑 localize（运动开）且 Nav2 已起来；前方约 1m 净空。另开 RViz：

```bash
bash /home/dijia/project/Navigation/scripts/phase2_go2_rviz.sh
```

```bash
# 终端 B（先 venv 再 ROS）
source /home/dijia/project/Robot-Brain/.venv-jazzy/bin/activate
source /opt/ros/jazzy/setup.bash
cd /home/dijia/project/Robot-Brain
export RDB_NAVIGATION_BACKEND=nav2 RDB_NAV2_MAP_ID=lab-floor1 RDB_NAV2_MAP_VERSION=v1
# 硬验收：默认 away=1m；away 后 map 位移须 ≥ away*0.5，否则判假成功失败
python scripts/verify_spatial_memory_navigation.py --live-return --away-m 1.0 --timeout-s 120
```

成功条件：`ok=true`；`away_travel_m` ≥ `min_away_m`；away/return 均 `succeeded`；`distance_to_anchor_m` ≤ `--reach-tolerance-m`（默认 0.25）。RViz 应见青色锚点与橙色 `/goal_pose`。记忆写入 `data/spatial_phase3.sqlite`（gitignore）。离线仍跑：`python scripts/verify_spatial_memory_navigation.py`。

## 4. 安装

```bash
cd /path/to/robot-brain
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[unitree-webrtc]"
```

不要为了 WebRTC 使用 `.[unitree]`；该 extra 还会从 GitHub 安装 DDS SDK。在 GitHub 网络不可用时会导致整个安装失败。

## 5. 自动测试

```bash
cd /path/to/robot-brain
source .venv/bin/activate

python -m pytest -q
python -m ruff check config robot_brain tests \
  scripts/verify_direct_go2_navigation.py \
  scripts/verify_spatial_memory_navigation.py
git diff --check
```

空间记忆离线闭环：

```bash
python scripts/verify_spatial_memory_navigation.py
```

成功条件：

```text
ok=true
forbidden_direct_motion=[]
```

## 6. 4G Remote 只读测试

### 6.1 配置

```bash
export RDB_UNITREE_TRANSPORT=webrtc
export RDB_UNITREE_WEBRTC_CONNECTION_MODE=remote
export RDB_UNITREE_SERIAL="<设备序列号>"
export RDB_UNITREE_CLOUD_USERNAME="<宇树账号>"
export RDB_UNITREE_CLOUD_REGION="cn"
export RDB_UNITREE_CLOUD_DEVICE_TYPE="Go2"
export RDB_UNITREE_WEBRTC_CONNECT_TIMEOUT="60"

read -s "RDB_UNITREE_CLOUD_PASSWORD?请输入宇树账号密码: "
echo
export RDB_UNITREE_CLOUD_PASSWORD

export RDB_UNITREE_DRY_RUN=true
export RDB_UNITREE_ENABLE_MOTION=false
export RDB_UNITREE_VIDEO_RELAY=false
export RDB_UNITREE_AUDIO_RELAY=false
```

上面配置加启动命令的完整单块版本如下，可直接复制后替换占位符：

```bash
cd /path/to/robot-brain
source .venv/bin/activate

export RDB_UNITREE_TRANSPORT=webrtc
export RDB_UNITREE_WEBRTC_CONNECTION_MODE=remote
export RDB_UNITREE_SERIAL="<设备序列号>"
export RDB_UNITREE_CLOUD_USERNAME="<宇树账号>"
export RDB_UNITREE_CLOUD_REGION="cn"
export RDB_UNITREE_CLOUD_DEVICE_TYPE="Go2"
export RDB_UNITREE_WEBRTC_CONNECT_TIMEOUT="60"
export RDB_UNITREE_DRY_RUN=true
export RDB_UNITREE_ENABLE_MOTION=false
export RDB_UNITREE_VIDEO_RELAY=false
export RDB_UNITREE_AUDIO_RELAY=false
export RDB_UNITREE_LIDAR_STREAM=true
export RDB_UNITREE_LIDAR_ALLOW_UNCOMPRESSED=false
export RDB_NAVIGATION_BACKEND=direct_go2

unset RDB_UNITREE_ROBOT_IP UNITREE_ROBOT_IP DIMOS_ROBOT_IP ROBOT_IP

read -s "RDB_UNITREE_CLOUD_PASSWORD?请输入宇树账号密码: "
echo
export RDB_UNITREE_CLOUD_PASSWORD

python scripts/verify_direct_go2_navigation.py --sensor-timeout-s 30
```

### 6.2 状态连接

```bash
python -m examples.run_unitree_smoke \
  --transport webrtc \
  --state-only
```

只读状态命令不要添加 `--live`。`--live` 会把 dry-run 改为 false。

### 6.3 点云和里程计

```bash
export RDB_NAVIGATION_BACKEND=direct_go2
export RDB_UNITREE_LIDAR_STREAM=true
export RDB_UNITREE_LIDAR_ALLOW_UNCOMPRESSED=false

python scripts/verify_direct_go2_navigation.py \
  --sensor-timeout-s 30
```

需要短时诊断未压缩点云时：

```bash
export RDB_UNITREE_LIDAR_ALLOW_UNCOMPRESSED=true
python scripts/verify_direct_go2_navigation.py --sensor-timeout-s 30
export RDB_UNITREE_LIDAR_ALLOW_UNCOMPRESSED=false
```

未压缩点云可能消耗较多 4G 流量，不应常开。

成功条件：

```text
sensor_snapshot.ready=true
pose_source=unitree_robotodom
point_count>0
obstacle_frame=base_link
lidar_frame_count>0
```

诊断解释：

- `lidar_state_count>0` 且两个 message count 为 0：雷达正常，但点云没有进入当前数据通道；
- message count 大于 0、`lidar_frame_count=0`：消息到达但解析失败；
- `odom_frame_count=0`：权威里程计不可用；
- `untrusted_obstacle_frame`：点云坐标系无法安全转换，禁止运动。

## 7. 局域网只读测试

Mac/Ubuntu 必须与 Go2 位于同一广播域。先发现：

```bash
python - <<'PY'
from unitree_webrtc_connect.multicast_scanner import discover_ip_sn
print(discover_ip_sn(timeout=8))
PY
```

然后配置：

```bash
export RDB_UNITREE_WEBRTC_CONNECTION_MODE=local
export RDB_UNITREE_SERIAL="<设备序列号>"
export RDB_UNITREE_ROBOT_IP="<发现到的局域网IP>"
export RDB_UNITREE_DRY_RUN=true
export RDB_UNITREE_ENABLE_MOTION=false
export RDB_NAVIGATION_BACKEND=direct_go2
export RDB_UNITREE_LIDAR_STREAM=true
```

连接和点云测试与第 6 节相同。

若固件要求 AES key，使用环境变量：

```bash
read -s "UNITREE_AES_128_KEY?请输入设备 AES Key: "
echo
export UNITREE_AES_128_KEY
```

## 8. 真机局部导航测试

**有效真机参数与完整命令以 §1.6 为准**（含 Move、速度、gait/cancel probe）。下列为文档早期模板，首次小步仍可用，但若站桩不迈步请改用 §1.6。

前置条件：

- 第 6 或第 7 节点云只读报告全部成功；
- 狗处于平整空旷地面；
- 前方至少两米无障碍；
- 操作者在旁边，可以急停或断电；
- Unitree App 和其他 WebRTC 客户端已退出；
- 首次只允许 0.10 m，禁止直接测试大目标。

10 cm 前进：

```bash
export RDB_UNITREE_DRY_RUN=false
export RDB_UNITREE_ENABLE_MOTION=true
export RDB_UNITREE_MAX_SPEED=0.10
export RDB_UNITREE_MAX_YAW_SPEED=0.20
export RDB_UNITREE_MAX_DRIVE_DURATION=0.25

python scripts/verify_direct_go2_navigation.py \
  --live \
  --confirm I_UNDERSTAND_DIRECT_GO2_NAV \
  --forward-m 0.10 \
  --timeout-s 8 \
  --sensor-timeout-s 30
```

原地旋转 10 度：

```bash
python scripts/verify_direct_go2_navigation.py \
  --live \
  --confirm I_UNDERSTAND_DIRECT_GO2_NAV \
  --forward-m 0 \
  --yaw-degrees 10 \
  --timeout-s 8 \
  --sensor-timeout-s 30
```

测试结束立即恢复安全门：

```bash
export RDB_UNITREE_DRY_RUN=true
export RDB_UNITREE_ENABLE_MOTION=false
unset RDB_UNITREE_CLOUD_PASSWORD
```

## 9. 完整服务测试

先以只读模式启动：

```bash
export RDB_ROBOT=unitree
export RDB_PERCEPTION=unitree
export RDB_NAVIGATION_BACKEND=direct_go2
export RDB_UNITREE_TRANSPORT=webrtc
export RDB_UNITREE_DRY_RUN=true
export RDB_UNITREE_ENABLE_MOTION=false

python -m examples.run_service
```

只有局部导航分级验收全部通过后，才允许：

```bash
export RDB_UNITREE_DRY_RUN=false
export RDB_UNITREE_ENABLE_MOTION=true
python -m examples.run_service
```

## 10. 安全和清理

- 不提交 `.env`、账号、密码、序列号对应的密钥、token 或 AES key。
- 一次只能有一个主要 WebRTC 客户端；退出 Unitree App 实时控制。
- dry-run 断开不会发送运动控制帧；live 断开仍会归零并发送 StopMove。
- 点云、odom 或坐标系不可信时必须 fail closed。
- `.tmp/` 是本地未跟踪目录，不属于本次提交。
