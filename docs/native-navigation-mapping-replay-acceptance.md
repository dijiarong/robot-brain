# 原生重定位与闭环实录验收

该流程复用 `robot-brain` 运行时写出的 gzip JSONL 传感器回放，不启动或导入 DIMOS、
ROS2、Nav2、Open3D。采集时设置唯一的新文件：

```bash
export RDB_NATIVE_NAV_REPLAY_PATH=evidence/relocalization-session.jsonl.gz
```

回放逐帧保存权威 odom、body-frame 点云、传感器 age 和 frame；不得用规划路径或手工
编辑后的点替代原始传感器证据。

## 建图回放

```bash
python scripts/verify_native_mapping_replay.py mapping \
  evidence/mapping-session.jsonl.gz --output evidence/mapping-report.json
```

通过要求：至少一帧有效 body-frame 点云进入生产 `SparseVoxelMap`，输出帧数、输入点数、
最终 voxel 数、容量上限及 map id/version/content revision；空回放、不可信 frame、空地图
或容量不变量不成立均失败。该报告证明可重复建图，不替代现场尺度/漂移测量。

## 旧地图重定位

在已知大致初值时：

```bash
python scripts/verify_native_mapping_replay.py relocalization \
  evidence/relocalization-session.jsonl.gz --map evidence/reference-map.json \
  --initial-x 1.0 --initial-y 2.0 --initial-yaw 15 \
  --output evidence/relocalization-report.json
```

不知道初值时显式打开有预算的全局 fallback：

```bash
python scripts/verify_native_mapping_replay.py relocalization \
  evidence/relocalization-session.jsonl.gz --map evidence/reference-map.json \
  --global-fallback --output evidence/relocalization-global-report.json
```

通过要求：`ok=true`、`reason=accepted`，并记录 map id、稳定 version、内容 revision、
fitness、RMSE、内点数、源点数、候选数和 map-frame 位姿。候选预算超限、低 fitness、
高 RMSE、无初值且未显式允许全局搜索均必须失败关闭。

## 在线闭环与 PGO

采集一条至少 8 个有效 keyframe、总时长超过 20 秒、最终实际回到旧位置的闭合路线，
然后执行：

```bash
python scripts/verify_native_mapping_replay.py loop_closure \
  evidence/closed-loop-session.jsonl.gz \
  --output evidence/closed-loop-report.json
```

通过要求：至少一个旧 keyframe 候选经点云匹配验证并形成 loop edge，优化后图残差低于
优化前，平移和 yaw 修正未越质量门；仅 odom 位置接近、没有点云验证，或只有候选但
优化被拒绝都不通过。报告逐 keyframe 记录 loop fitness/RMSE、图优化前后 RMSE 与最大
修正量，原始回放和报告均应保留 SHA-256 后再进入最终验收包。
