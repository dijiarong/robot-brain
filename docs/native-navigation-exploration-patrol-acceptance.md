# 原生探索与巡逻验收

为每个独立场次设置唯一 `RDB_NATIVE_NAV_TRACE_PATH`，通过 `nav_explore` 或
`nav_patrol` 执行。所有运动仍需 live 与 motion gate 双门；离线只允许 Fake/回放。

## Frontier 探索

执行后生成报告：

```bash
python scripts/analyze_native_navigation.py evidence/exploration.jsonl \
  --output evidence/exploration-report.json
```

通过要求：`exploration.ok=true`；至少到达一个 frontier；每次选择均记录 frontier cell
数、物理长度、评分、visited 黑名单规模和选择时 known cells；每个目标都有精确成功终态
及运动后地图已知单元增量样本。只允许以 `complete`、`max_goals` 或有连续证据支持的
`no_information_gain` 正常结束。无 frontier 起步、frame/定位不一致、导航失败、缺少增量
样本或命令—终态不完整均不通过。

地图增量来自机器人自身传感器，不是外部覆盖真值；现场报告还应记录起止地图 revision、
场地区域和人工确认的已访问房间/走廊。

## 巡逻与覆盖

coverage、frontier、random、least-visited 四种策略分别执行并生成独立报告。路线生成时
会立即对照同一 costmap 复验：所有 waypoint 必须在 known-free、无重复；coverage 必须
保持往复蛇形；frontier 点必须紧邻未知单元；least-visited 的访问计数必须非递减。随机
策略使用固定 seed 保证复现，但随机性本身不替代安全性。

报告通过要求：`patrol.ok=true`、`route_evaluation.ok=true`，每个 waypoint 都有成功终态，
`attempted == reached`、`failed == 0`，完成配置周期数。至少另做一次运动中取消，要求取消
传播至活动目标；取消场次是安全门证据，不计为完整巡逻通过。

多策略验收不能用同一份成功报告互相顶替；最终证据包应分别保留四份原始 trace、分析
报告、配置、地图 identity/revision 与 SHA-256。
