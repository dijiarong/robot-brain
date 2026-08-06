# 导航完整抽离完成度审计

审计原则：自动化测试只证明其覆盖的行为；合成或离线回放不替代 Go2 现场运动证据。
因此本文件中的“离线完成”不等于 Goal 完成。

源码范围由 `scripts/audit_dimos_navigation_extraction.py` 机器复核；当前固定 JSON 证据
覆盖 221 个 topsun-dimos 文件、13 个能力组，unclassified/missing/forbidden 均为 0。
该结果证明当前源码没有静默漏列，不替代各能力的行为及现场证据。

| 原始要求 | 当前证据 | 审计结论 |
| --- | --- | --- |
| 1. Go2 odom、LiDAR、frame、timestamp、cmd_vel 安全驱动 | `UnitreeNavigationSensorProvider`；有限位姿/点云过滤；负/NaN age 拒绝；WebRTC motion gate；sensor/direct/native tests | 离线完成；真机只读与运动待验 |
| 2. 点云、voxel、2D grid、膨胀、carving、容量、保存加载 | `grid.py`, `map_store.py`；schema v2；稳定 map version + content revision；v1 migration；hash/atomic tests | 离线完成；真实长时地图容量待现场 |
| 3. 安全目标、A*、净空、跟随、动态停车/重规划、yaw | `planner.py`, `native_go2.py`；局部有限网格；known-free 全局 A*；全局/局部重规划；final yaw tests | 离线完成；直行/绕障/突障现场待验 |
| 4. cancel、estop、timeout、no path、lost/stale/no progress、teleop | provider guards、TeleopSession 抢占、失败拒绝接管、场景判定 tests | 离线完成；cancel/突障/stuck 现场待验 |
| 5. 初值/全局重定位、预算、质量门、map→odom、合并、绝对目标 | `relocalization.py`；candidate budget；fitness/RMSE；map merge；4 m known-free route；unknown reject | 合成完成；旧地图真机/同期实录待验 |
| 5a. CMU PGO loop closure corrected odometry | `pose_graph.py`：scan-verified loop constraint、硬锚点 pose graph、残差改善/最大修正门、时间插值、关键帧/时间/空间候选与容量预算；native_go2 可选接入且严格隔离 global corrected frame 与 local raw odom；synthetic/runtime tests | 离线实现与运行时接入完成；默认关闭，闭环实录/真机标定未验，不能视为现场完成 |
| 6. frontier、黑名单、信息增益、巡逻四策略、多周期取消 | `frontier.py`, `exploration.py`, `patrol.py`, `patrol_controller.py` 与 runtime skills | 离线完成；现场探索/巡逻待验 |
| 7. bbox、2D/3D servo、多层地形 | `visual_navigation.py` + `visual_controller.py`；fraction/Qwen-1000/pixel/inferred bbox 归一化；保守标签匹配；条件注册 `nav_visual_servo`；短程 NavigationClient 段、逐段重捕获、稳定帧到达、取消传播、检测丢失/陈旧/低置信/超时/异常停止测试；MLS/rolling terrain/controller；PLY/PCD 隔离回放；mcf-only motion skill | 连续控制离线完成，真实检测流待验；实录地形规划完成，物理地形待验 |
| 7a. CMU boundary/added obstacle/slow/safety/acceleration | terrain polygon + obstacle overlays；同参只读/运动入口；timestamped safety signal；vector acceleration limiter；稳定 UNAVAILABLE 终态 | 离线实现完成；真实 boundary、外部安全源刷新和减速停车距离待现场 |
| 7b. CMU TARE 三维 frontier 探索 | MLS unknown-neighbor gain、距离/净空/代价评分、visited blacklist、fresh native scan 单步规划、可取消有界多目标 controller、mcf-only high-risk skill | 离线与 native read-only 接入完成；真实多层覆盖效率和物理运动待现场 |
| 8. 状态、失败码、trace、replay、分析报告 | goal/path/replan/stop reason；正常关闭/进程中断 session；有界非阻塞 trace 与 dropped/writer error；脱敏 manifest；严格 JSON；gzip replay；连续 CTE、切线 overshoot、angular flips、snake candidate、command→odom yaw response lag；observed/correlated 证据等级 | 离线完成；现场报告待生成 |
| 9. task/地点记忆/安全/技能/teleop/service | NavigationClient；spatial skills；TopologyGraph；native skills/tools；service diagnostics；teleop arbitration | 离线完成；多房间真机任务待验 |
| 10. Fake/dry-run/unit/integration/replay/live gates | 全量 pytest、两项无依赖 verifier、五场景 live script 与现场手册 | 除真机阶段外完成 |

## 不适用与替代证据

- DIMOS Module/Blueprint、LCM/pSHM、reactivex 消息体系：按目标明确不迁移；由直接 Python
  contracts、async controllers 与 Unitree transport 替代。
- Nav2/ROS2：native_go2 隔离 verifier 主动阻断 `rclpy`，仍完成端到端绕障。
- DIMOS Rerun exporter：不作为运行依赖；严格 JSONL、gzip replay、manifest 与 JSON report
  保留可移植原始证据，可由外部可视化工具读取。
- CMU `free_paths` 点云仅用于候选轨迹可视化，不参与运动决策；由 trace 中实际 path、
  rejected overlay 和命令审计替代。`two_way_drive` 是非全向车的前后朝向选择；Go2 使用
  原生全向 `vx/vy`，不复制该开关，验证依据是所有 body-frame 路径段均可含横向速度。

## 尚未满足、禁止关闭 Goal 的证据

1. Go2 只读传感器现场报告。
2. 0.3 米直行、1–3 米几何绕障、运动中取消、突然阻挡停车、卡住 no_progress。
3. 旧持久地图与现场当前扫描的重定位/合并证据。
4. 显式开启在线 PGO 后的闭环实录、corrected odometry 与地图一致性证据。
5. Frontier 探索、巡逻和连续视觉检测流的真实运行证据。
6. `mcf` 下真实斜坡/台阶/墙边/断崖以及 TARE 多层覆盖的分阶段三维门禁。
7. 上述现场产物的完整严格 JSON 与命令审计。

现场步骤见 `docs/native-navigation-live-acceptance.md`。所有项目完成并再次全量回归、依赖
隔离审计后，才可执行最终矩阵复核并关闭 Goal。
