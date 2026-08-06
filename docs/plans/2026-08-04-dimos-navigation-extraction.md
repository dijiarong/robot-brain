# DIMOS 导航能力完整抽离计划与验收矩阵

## 最终目标

`robot-brain` 在未安装、未导入、未启动 DIMOS 或 Nav2 时，独立提供 Go2 导航、
建图、重定位、探索、巡逻、控制权协调和诊断能力。迁移算法与行为语义，不迁移
DIMOS Module、Blueprint、LCM/pSHM、reactivex 和消息类型。

明日的 0.3 米直行、1–3 米绕障等测试只是阶段 1 现场门禁，不代表完整抽离完成。

机器可读完成矩阵位于
`docs/evidence/native-navigation-completion-matrix.json`。执行
`python scripts/audit_native_navigation_completion.py --run-verifiers` 会同时运行全量测试、
无 DIMOS/Nav2 离线端到端和 221 文件源审计。审计结果严格区分
`offline_artifacts_ok` 与 `complete`：任何现场/实录 gate 未提供绑定 gate ID、原始产物和
可复算 SHA-256 的通过报告时，`complete` 必须保持 false。当前固定审计证据为
`docs/evidence/2026-08-04-native-navigation-completion-audit.json`。

外部证据必须先经过领域语义注册器，例如：

```bash
python scripts/register_native_navigation_evidence.py \
  go2_relocalization_replay evidence/relocalization-report.json \
  evidence/relocalization-session.jsonl.gz \
  --output evidence/go2_relocalization_replay-registered.json
```

注册器不是通用的 `ok` 包装器：它按 gate 检查传感器 ready、六场景齐全、建图帧/voxel、
重定位质量、闭环边、探索增量、四种巡逻策略、视觉收敛、MLS 安全或服务/仲裁字段；领域
字段不匹配时拒绝注册。完成审计再逐个复算注册清单中的原始产物 hash。

## 能力矩阵

| 能力 | DIMOS 权威实现 | robot-brain 目标 | 当前状态 | 完成证据 |
| --- | --- | --- | --- | --- |
| Go2 odom/LiDAR/cmd_vel | `robot/unitree/go2/connection.py` | 原生 WebRTC transport + `UnitreeNavigationSensorProvider` | 已有，待真机复验 | 传感器新鲜度/坐标系测试与现场 JSON |
| 点云转 costmap | `mapping/voxels.py`, `mapping/costmapper.py` | 依赖无关二维 costmap、过滤、膨胀和障碍距离梯度 | 局部与持久地图均已实现；距离场使用有界多源传播，避免随障碍数乘法退化 | `test_native_navigation.py` |
| 全局地图累积 | `mapping/voxels.py` | odom/map 帧增量 voxel map | 稀疏 voxel、ray free-space、carving、容量限制已实现 | `test_native_map_store.py`, replay artifact |
| A* 与目标安全化 | `navigation/replanning_a_star/*` | A*、安全目标搜索、路径净空 | 已实现障碍软代价、连续线段栅格化净空与机器人宽度走廊检查；仅与 A* 一致地豁免被膨胀覆盖的当前起始格 | `test_native_goal_validator.py`, `test_native_navigation.py` |
| 路径跟随 | `replanning_a_star/local_planner.py`, `basic_path_follower` | Go2 omni 分段跟随与到达判定 | 阶段 1 已实现 | Fake odom 闭环与现场轨迹 |
| 动态重规划 | `global_planner.py`, `replan_limiter.py` | 新点云触发、有频率上限的有限重规划 | 每段新观测重规划、无路径次数与最小间隔已实现 | 动态阻挡、replan count、限频配置测试 |
| 停止/人工接管 | `movement_manager/movement_manager.py` | 导航、teleop、急停所有权仲裁 | teleop 抢占、失败关门及急停同步已实现 | `test_navigation_control_arbitration.py` |
| 地图保存/加载 | `memory2`, `mapping/utils/cli/map.py` | 版本化地图文件、hash、原子写入 | schema v2 已实现：稳定坐标 map version 与可变内容 revision 分离；v1 验证迁移读取 | 原子保存、重开 identity、revision、v1 migration、篡改拒绝测试 |
| 旧地图重定位 | `mapping/relocalization/*` | 固定初值 + ICP、全局 fallback、质量门 | 二维粗到细匹配、初值与全局 fallback 已实现；原生 gzip 传感器实录可直接复跑并输出 map identity/fitness/RMSE/内点/预算；实录待采集 | `test_native_relocalization.py`, `test_navigation_replay.py`, `verify_native_mapping_replay.py` |
| 在线 loop closure / PGO | `mapping/loop_closure/*`, `cmu_nav/modules/pgo/*` | scan-verified loop edge、pose graph、corrected odometry/map→odom | 依赖无关平面 pose graph、硬锚点、迭代预算、残差/修正幅度门、时间插值、有预算点云 loop 验证及在线 keyframe tracker已接 native_go2；默认关闭，校正仅进入全局建图/探索/三维坐标，局部运动保持 raw odom；同一生产 tracker 可从 gzip 实录自动判定 loop edge 与优化前后残差，闭环实录待采集 | `test_native_pose_graph.py`, `test_navigation_replay.py`, native runtime frame-isolation test, `verify_native_mapping_replay.py` |
| 地图合并 | `RelocalizationModule.merged_map` | 旧地图与新观测合并、可选 carving | 定位后合并与多次 miss carving 已实现 | frame gate、移除障碍、容量测试 |
| Frontier 探索 | `frontier_exploration/*` | frontier 提取、评分、黑名单和终止条件 | 提取、黑名单、信息增益和可取消控制器已实现；统一 trace 自动核对 frontier 选择→导航终态→known-cell 增量及允许的终止原因；现场覆盖实录待采集 | frontier/exploration/diagnostics tests, `analyze_native_navigation.py` |
| 巡逻/覆盖 | `patrolling/*` | random/frontier/coverage/固定路线 | coverage/frontier/random/least-visited 路由、多周期执行、取消与 `nav_patrol` 技能已接；生成时对照 costmap 自动验证 known-free/唯一性/蛇形/frontier 邻接未知/访问计数顺序，trace 核对每个 waypoint 终态；四策略现场实录待采集 | patrol route/controller/runtime skill/diagnostics tests |
| 相对/绝对目标 | `NavigationInterface`, Unitree skills | `NavigationClient` + map identity | 相对位置/yaw、稳定 map version、known-free 腐蚀后的全局 A*、跨局部窗口分段执行与局部无路时有限全局重规划已实现；未知空间拒绝 | native provider 4 m route/unknown/global-replan tests |
| 拓扑导航 | `navigation/topology.py` | 地点/门/房间地标图、同地图连边与 A* 粗到细路由 | 自有 `TopologyGraph` 已实现，严格按 frame/map identity 隔离 | `test_native_topology.py` |
| 导航状态与失败码 | `navigation/base.py` | 统一 status/error/stop_reason | 已扩展 | provider/skill/API 测试 |
| trace 与诊断 | `navigation/diagnostics/*` | 会话关联、manifest、JSONL、传感器回放、离线指标/报告 | goal 关联、正常/进程中断 session 判别、脱敏 immutable manifest、有界非阻塞后台 trace（含 dropped/writer error）、path/command/odom/yaw 事件、gzip replay、连续投影横向误差、最终切线超调、角速度翻转、蛇形候选、command→odom yaw-rate 响应延迟和多次 replan 非累加路径报告均已实现；证据等级区分 observed/correlated | diagnostics/replay tests、`analyze_native_navigation.py` |
| DIMOS Rerun 导出 | `diagnostics/rerun_export.py` | 不引入 DIMOS/Rerun 运行依赖；保留可移植证据 | 明确不适用：JSONL + gzip replay + JSON report 是等价可审计产物，可由外部工具可视化；运行时依赖隔离优先 | dependency isolation test；report schema |
| 视觉/检测导航 | `bbox_navigation.py`, `visual/query.py`, `visual_servoing/*` | 作为上层目标生成器接 NavigationClient | bbox、2D/3D 目标、单次 `nav_go_to_bbox` 与连续 `nav_visual_servo` 已接；兼容 fraction/Qwen-1000/大图像素/推断尺度；英文 token 与中英文子串目标匹配，中文模糊歧义 fail-closed；连续技能仅在真实帧源/识别器存在时注册，经短程安全导航逐段重捕获，丢失/陈旧/低置信/超时停止；统一 trace 已记录无图像的观测→命令→导航终态→稳定收敛链并可自动判定，现场待验 | visual navigation/controller/skill/diagnostics tests, `analyze_native_navigation.py` |
| CMU 3D 导航 | `navigation/cmu_nav/*` | 地形图、可达性与三维路径；不与二维偷换等价 | 自有滚动地形图具备距离/高度裁剪、体素降采样、衰减、近场合并、容量限制和手工清理；可取消三维路径执行已接 `NavigationClient`；PLY/PCD/原生 gzip 实录验证器输出输入 hash、台阶/坡度/净空/路径/耗时，统一 trace 自动核对每个 MLS waypoint 的命令与终态；Go2 地形实录待采集 | terrain/replay/controller/diagnostics tests, `verify_native_terrain3d.py`, `analyze_native_navigation.py` |
| CMU boundary/外加障碍 | `local_planner` 的 `navigation_boundary`, `added_obstacles` | 全局坐标多边形硬边界、原子替换临时障碍覆盖层 | 已实现并同时接入只读 `native_terrain_plan` 与确认制三维运动技能；顶点/障碍数量有界，非法/自交边界拒绝，覆盖变化会重验旧路径安全前缀 | terrain overlay/tool tests |
| CMU slow/safety/acceleration | `local_planner.slow_down`, `path_follower.safety_stop/maxAccel` | 新鲜外部 safety signal、立即停车、速度缩放、平面矢量加速度门 | 已实现；信号接入后陈旧即停，stop 不发运动，slowdown 与加速度记录进 command trace；未接信号源不改变默认行为 | `test_native_motion_safety.py`, native provider tests |
| CMU TARE 三维探索 | `cmu_nav/modules/tare_planner` | MLS frontier、未知覆盖增益、距离/净空评分、visited 黑名单、有界多目标执行 | 已实现 `frontier_goals`、native fresh-scan 单步规划和可取消多目标控制器；`nav_terrain_explore` 仅在 `mcf` 注册且需高风险确认 | terrain frontier/controller/native-client/skill tests；真实覆盖待验 |
| MLS 3D planner | `navigation/nav_3d/*` | 多层 surface nodes/edges/path | 自有 MLS 具备机器人净空、多层表面、墙软代价、台阶/坡度门禁、全局重建、局部圆柱替换、路径代价和旧路径安全前缀；验证器已在 DIMOS Unity traversable map 上以生产安全参数复验并输出输入 SHA-256/106 节点路径/台阶/坡度/0.2 m 最小净空；真实传感器边缘净空仍需现场几何证明 | synthetic/controller/replay acceptance tests；Unity 固定输入离线通过；Go2/Mid360 原生实录仍是开放门禁 |

## 当前运行时入口

- Provider：`RDB_NAVIGATION_BACKEND=native_go2`。
- 持久地图：`RDB_NATIVE_NAV_MAP_PATH`；存在时加载，重定位质量门通过后开放绝对目标。
- Trace：`RDB_NATIVE_NAV_TRACE_PATH`；回放：`RDB_NATIVE_NAV_REPLAY_PATH`。
- Planner skills：`nav_go_relative`、`nav_go_to_pose`（持久地图）、`nav_cancel`、
  `nav_explore`、`nav_patrol`、`nav_go_to_bbox`；VLM 相机帧源和识别器均可用时
  额外注册逐段重捕获的 `nav_visual_servo`；当 Unitree motion mode 为 `mcf`
  时额外注册高风险、需确认的 `nav_go_terrain_relative` 与 `nav_terrain_explore`。
- Operator/internal：`nav_relocalize`、`native_map_get_state`、`native_map_save`。
- 三维只读入口：`native_terrain_plan` 始终可用；非 `mcf` 模式只规划、不注册
  三维运动技能。
- 三维库：`MultiLevelTerrainPlanner` + `RollingTerrainMap`；执行层
  `TerrainPathController` 只接收已通过三维可达性门禁的路径，并在每个水平段前重取定位。
- 三维 boundary/added obstacles 由只读工具与运动技能使用同一规划参数；外部
  slow/safety 信号通过 `set_motion_safety_signal` 注入，信号一旦接入必须持续刷新。
- 隔离验收：`python scripts/verify_native_navigation_offline.py` 会主动屏蔽
  DIMOS、rclpy、Open3D、reactivex 后跑端到端绕障。
- 三维隔离回放：`python scripts/verify_native_terrain3d.py POINTCLOUD` 原生读取
  ASCII/binary PLY 与 PCD，主动屏蔽同一组外部栈后输出 surface/component/path/timing JSON。

“不适用”必须记录机器人场景、替代能力和验证依据，不能仅因迁移困难而排除。

机器可执行的源覆盖审计：
`python scripts/audit_dimos_navigation_extraction.py`。当前审计扫描 221 个 DIMOS
导航相关 Python 文件，按 13 个能力组核对实现和测试；未分类、缺失证据或运行时
导入 DIMOS/reactivex/rclpy/Open3D 均返回非零。固定证据见
`docs/evidence/2026-08-04-dimos-navigation-source-audit.json`，其中包含源摘要哈希及每个
排除项的逐文件理由。

## 阶段 1：自主局部导航主链路

数据流：

```text
Go2 ROBOTODOM + ULIDAR
        -> freshness/frame gate
        -> body-frame costmap + inflation
        -> A*
        -> body-frame waypoint follower
        -> UnitreeRobot safety clamps
        -> odom progress + replan
```

安全不变量：

- 缺失、陈旧或非可信坐标系的传感器一律停止。
- 真机运动仍必须同时关闭 dry-run 并打开 motion gate。
- 每段运动前使用最新点云重新规划，并保留独立紧急走廊停车门。
- 取消、超时、无路径和无进展都有结构化 `stop_reason`。
- 局部规划只承诺当前感知窗口；远距离绝对目标必须通过已重定位、同一稳定
  `map_version` 且经 known-free 腐蚀的持久地图分段路由，未知空间一律拒绝。

## 阶段 1 现场门禁

逐项命令、人员/场地安全门与机器判定见
[`docs/native-navigation-live-acceptance.md`](../native-navigation-live-acceptance.md)。

1. 默认只读启动，确认权威 `unitree_robotodom`、LiDAR 帧、点数和新鲜度。
2. 空旷环境 0.3 米直行，到达误差、重规划次数和停止原因写入 JSON。
3. 1–3 米静态绕障，不得擦碰膨胀区。
4. 运动中取消，记录取消到零速度的延迟与停止后滑移。
5. 突然加入障碍，先停车再产生新路径。
6. 抱起或阻挡机器人，有限段数后以 `no_progress` 安全退出。

每次报告必须包含传感器摘要、起终位姿、costmap 参数、path、replan_count、
stop_reason、命令审计以及完整 trace。

## 后续顺序

1. 全局 voxel map 与版本化持久化。
2. 安全目标搜索、重规划限频和控制权仲裁。
3. 固定初值 ICP、全局重定位 fallback 与地图合并。
4. Frontier 探索和覆盖/巡逻路由。
5. 持久 trace、回放捕获和诊断报告。
6. 逐项关闭能力矩阵，最终执行无 DIMOS/Nav2 环境验收。
