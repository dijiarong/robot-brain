# 原生三维地形与 MLS 验收

## 点云/实录离线规划

验证器接受 PLY、PCD 或原生 `NavigationReplayWriter` 生成的 `.jsonl.gz`。回放中的
body-frame 点云按每帧权威 odom 转到世界坐标，再进入与运行时相同的 MLS 表面图和 A*：

```bash
python scripts/verify_native_terrain3d.py evidence/terrain-session.jsonl.gz \
  --resolution 0.20 --robot-height 0.30 --max-step 0.16 \
  --max-slope 25 --wall-clearance 0.10 --wall-buffer 0.75 \
  --output evidence/terrain-planning-report.json
```

通过要求：存在至少两个节点的连通表面路径；每条边的最大台阶不超过 0.16 米、坡度不
超过 25 度；所有节点 traversal cost 有限；路径与墙的净空严格大于配置硬门。报告固定
输入 SHA-256、输入类型、点数、表面节点数、最大连通域、路径长度/爬升/最小净空、各
阶段耗时和禁止依赖加载结果。无路径、预算超限、端点离表面过远或任一安全门失败均非零
退出，输出文件拒绝覆盖。

PLY/PCD 只能证明给定点云上的算法行为；只有 Go2/Mid360 原生回放才能关闭真实传感器
几何门禁。odom 是机器人报告值，不是外部地面真值。

## 三维路径实际执行

为真机场次设置唯一 `RDB_NATIVE_NAV_TRACE_PATH`，通过 `nav_terrain_go_to` 或
`nav_terrain_explore` 执行已经安全验证的三维路径，再运行：

```bash
python scripts/analyze_native_navigation.py evidence/terrain-motion.jsonl \
  --output evidence/terrain-motion-report.json
```

通过要求：`terrain_execution.ok=true`，每个 MLS waypoint 都有一条不超过 3 米的短程
命令及精确 `succeeded` 终态，`attempted == reached`，最终 `goal_reached`。缺定位、取消、
目标拒绝、任何短程失败或命令与终态数量不一致都不通过。执行证据还必须与同一场次的
规划报告、原始回放和现场场地记录一起保存；仅离线有路径不代表机器人实际可通过。

边界或临时障碍覆盖层变化后必须重新规划；旧路径只能保留当前仍安全且在障碍前至少
回退 1 米的前缀，不允许沿缓存路径穿过新障碍。
