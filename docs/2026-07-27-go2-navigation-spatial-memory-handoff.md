# Go2 导航与空间物品记忆：开发交接和实机测试

更新时间：2026-07-27

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

Goal 记录目前仍为 `blocked`。最初的阻塞原因是缺少真机 IP，后来已经通过宇树 4G Remote 成功连上真机；当前真正的技术阻塞更新为：**本项目的 Remote 连接能收到 odom 和 lidar_state，但收不到点云，而 Ubuntu 上的 DimOS Remote 实测能显示点云。两边实现版本尚未完成逐行对齐。**

### 1.2 Goal 分阶段状态

| 阶段 | 原计划 | 当前状态 | 还缺什么 |
|---|---|---|---|
| 第一阶段 | Go2 自带 LiDAR + odom，局部导航、动态避障、真机验收 | 代码和自动测试完成；Remote 状态/odom 真机通过；点云和真实运动未通过 | 对齐 Ubuntu DimOS Remote 点云实现；随后做 10 cm 和障碍停止验收 |
| 第二阶段 | map 绝对目标、定位/重定位状态、地图身份契约 | 接口、Fake、Nav2 和 session-local Go2 身份已实现 | 用真实 Nav2/SLAM 验证 map goal、重定位和持久 map identity |
| 第三阶段 | 房间/物品观察点持久化、跨房间查找、所有移动走 Provider | 重构和离线闭环测试已完成 | 真机地图上录入房间/物品并完成跨 session 返回测试 |
| 后续扩展 | Mid360/Super-LIO/其他 SLAM | 接口预留完成 | 尚未接入真实 Mid360/Super-LIO 数据和 launch 系统 |

### 1.3 本轮实际改动范围

主要新增或修改：

- `robot_brain/navigation/base.py`：Provider 契约、绝对/相对目标、定位状态、地图身份；
- `robot_brain/navigation/direct_go2.py`：Go2 短分段局部导航；
- `robot_brain/navigation/sensors.py`：odom/点云新鲜度和坐标系安全边界；
- `robot_brain/navigation/nav2.py`：Nav2 绝对目标、定位与地图身份；
- `robot_brain/perception/pointcloud.py`：Unitree WebRTC 点云标准化；
- `robot_brain/actuation/unitree_webrtc.py`：Remote 连接、ROBOTODOM、LiDAR、健康诊断、只读断开；
- `robot_brain/memory/spatial.py`：地图/会话绑定的空间记忆数据；
- `robot_brain/skills/builtin/spatial_memory.py`：房间/物品导航全部经 Provider；
- `robot_brain/skills/builtin/navigation.py` 和 `robot_brain/tools/builtin/navigation.py`：导航技能/工具；
- `robot_brain/runtime/loop.py`：按配置组装 Fake、Nav2 或 direct_go2；
- `scripts/verify_direct_go2_navigation.py`：真机只读与小步运动验收；
- `scripts/verify_spatial_memory_navigation.py`：空间记忆离线闭环验收；
- 对应导航、传感器、空间记忆和 WebRTC 测试。

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
- 点云必须新鲜且处于可信的机器人相对坐标系；`world` 点云只有原点与同时刻 odom 匹配时才转换到 `base_link`。

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

实机 4G Remote 已验证：

- 云端登录和 TURN/ICE 成功；
- Data Channel Verification 成功；
- 能读取 SportState、LowState、电量和 `mcf` 模式；
- `robot_pose` 约 18 Hz；
- `lidar_state` 约 5 Hz；
- 雷达自身报告 `error_state=0`、点云频率约 15 Hz、丢包率 0；
- 当前 `robot-brain` Remote 测试中压缩和未压缩点云消息计数仍为 0。

## 3. 尚未完成

### 3.1 4G Remote 点云差异尚未定位

Ubuntu 上的 DimOS 使用以下形式启动，并据用户实测能在 Runner 中显示点云：

```bash
dimos \
  --unitree-webrtc-method remote \
  --unitree-username "$UNITREE_USERNAME" \
  --unitree-password "$UNITREE_PASSWORD" \
  --unitree-serial "$UNITREE_SERIAL" \
  --unitree-region "$UNITREE_REGION" \
  run unitree-go2-agentic-deepseek \
  --disable security-module
```

对应的完整环境变量配置示例：

```bash
cd /path/to/topsun_dimos
source .venv/bin/activate

export UNITREE_USERNAME="<宇树账号>"
export UNITREE_SERIAL="<设备序列号>"
export UNITREE_REGION="cn"

read -s "UNITREE_PASSWORD?请输入宇树账号密码: "
echo
export UNITREE_PASSWORD

dimos \
  --unitree-webrtc-method remote \
  --unitree-username "$UNITREE_USERNAME" \
  --unitree-password "$UNITREE_PASSWORD" \
  --unitree-serial "$UNITREE_SERIAL" \
  --unitree-region "$UNITREE_REGION" \
  run unitree-go2-agentic-deepseek \
  --disable security-module
```

测试完成后：

```bash
unset UNITREE_PASSWORD
```

但是当前 Mac 仓库的 `origin/jtlinux` 不包含上述 CLI 参数，仍显示 `LocalSTA + ip` 实现。这说明 Ubuntu 工作区可能存在未推送提交、不同 commit 或安装的 `dimos` 指向另一份源码。

接手者需要先在 Ubuntu 采集：

```bash
cd /path/to/topsun_dimos
source .venv/bin/activate

git branch --show-current
git rev-parse HEAD
git status --short
which dimos

python - <<'PY'
import dimos
print(dimos.__file__)
PY

rg -n \
  'unitree-webrtc-method|unitree_webrtc_method|unitree_username|unitree_serial|WebRTCConnectionMethod.Remote' \
  dimos

python -m pip show unitree-webrtc-connect \
  | grep -E 'Name|Version|Location'
```

禁止把真实密码、AES key 或 token 写入输出、issue、提交和聊天记录。

拿到 Ubuntu 的实际连接源码后，重点比较：

1. `UnitreeWebRTCConnection` 构造参数；
2. `disableTrafficSaving(True)` 的调用时机和返回值；
3. decoder 类型；
4. `rt/utlidar/switch` 的发布时机；
5. LiDAR 订阅是在连接前登记还是连接后登记；
6. Remote 是否使用不同点云话题或数据通道；
7. `unitree-webrtc-connect` 的精确版本和本地补丁。

### 3.1.1 当前准确卡点

本项目 Remote 实机数据：

```text
WebRTC connection: connected
Robot Connection Mode: 4G
sport_state_count: 599
low_state_count: 599
odom_frame_count: 561
lidar_state_count: 149
lidar_compressed_message_count: 0
lidar_uncompressed_message_count: 0
lidar_frame_count: 0
```

雷达状态本身正常：

```text
software_version: 1.0.0.38
error_state: 0
cloud_frequency: 15.235347
cloud_packet_loss_rate: 0
cloud_size: 6878
cloud_scan_num: 210
```

因此卡点不是雷达硬件、云连接、odom 或 decoder 抛错，而是点云消息没有进入本项目注册的两个回调。下一步不能继续修改点云解析器；必须先取得 Ubuntu 上实际工作的 DimOS Remote 源码和版本，比较连接及订阅时序。

### 3.1.2 下一位开发者的第一组动作

1. 在 Ubuntu 上保存 `git rev-parse HEAD`、`dimos.__file__` 和依赖版本。
2. 找到实现 `--unitree-webrtc-method remote` 的文件。
3. 复制该文件的 Remote 构造和 post-connect/subscribe 部分到安全的临时文本，删除凭据。
4. 与 `robot_brain/actuation/unitree_webrtc.py` 的 `_do_connect()` 和 `_connect_once()` 对比。
5. 优先复现实测相同的订阅顺序，不要先改 decoder。
6. 重新运行第 6.3 节，直到 `lidar_compressed_message_count>0`。
7. 点云只读成功后，才进入第 8 节真机运动验收。

### 3.2 尚未完成真机运动验收

以下功能只有自动测试，尚未在真机完成：

- 10 cm 前进局部目标；
- 原地小角度旋转；
- 运动中出现障碍后的停止；
- 里程计无进展后的停止；
- 断连、取消和进程退出时的停止；
- 通过空间记忆返回房间或物品位置。

点云未通过只读验收前，不得执行这些真机运动测试。

### 3.3 全局导航和跨重启重定位

`DirectGo2NavigationClient` 只提供 session-local 短距离导航，不是全局规划器。实现跨重启空间物品记忆仍需要以下之一：

- `Navigation` 项目的 Nav2/SLAM；
- Mid360 + Fast-LIO/SLAM；
- Go2/拓展坞端运行建图、重定位和导航服务，Mac/云端只发送目标；
- 移植 DimOS 的预地图、ICP 重定位、VoxelGrid/Costmap 链路。

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
