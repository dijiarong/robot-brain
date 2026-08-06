# Native Go2 导航现场验收

本手册验收 robot-brain 的 `native_go2`，不启动 DIMOS、ROS2 或 Nav2。所有真实运动
命令都必须同时包含 `--live` 和精确确认串；缺少任一项时脚本只读或直接拒绝。

## 场地与人员门禁

- 机器人周围保留至少 1 米人员安全区，指定一人负责实体急停。
- 电量、站立状态、地面摩擦、悬崖/楼梯边缘和网络质量确认正常。
- 先完成只读传感器检查；任何 odom/LiDAR 陈旧、frame 不明或点数异常都停止验收。
- `sudden_block` 使用可移走的软障碍物，从机器人侧面放入，人员不得进入前进方向。
- `stuck` 只允许安全牵制或在保护人员配合下轻微抱起，禁止拉扯关节。

## 0. 只读传感器

```bash
python scripts/verify_native_go2_navigation.py
```

通过条件：`mode=read_only`、`sensors.ready=true`、`pose_source` 为权威 robot odom、
`obstacle_frame` 为受支持的 body frame、点云和位姿 age 均在配置上限内。

## 1. 空旷 0.3 米直行

```bash
python scripts/verify_native_go2_navigation.py --live \
  --scenario straight --forward-m 0.3 --timeout-s 20 \
  --confirm I_UNDERSTAND_NATIVE_GO2_NAV
```

通过条件：终态 `succeeded`、`stop_reason=goal_reached`，且 `motion_sample` 中由真实
odom 投影得到的前向进度不低于“请求距离减 0.15 米”（最少 0.05 米）；报告包含起终
位姿、path、replan count、trace 和 command audit。只有成功状态而没有观测位移不通过。

## 2. 1–3 米静态绕障

先在路线中放置静态软障碍，再执行（距离按场地选择 1–3 米）：

```bash
python scripts/verify_native_go2_navigation.py --live \
  --scenario obstacle --forward-m 1.5 --timeout-s 45 \
  --confirm I_UNDERSTAND_NATIVE_GO2_NAV
```

通过条件：终态 `succeeded`、`stop_reason=goal_reached`，规划路径偏离和真实 odom 轨迹
横向偏离均大于 0.05 米，并满足前向进度门；仅有多次重规划或只有规划线弯曲而机器人
没有实际绕行都不算通过。

## 3. 运动中取消

```bash
python scripts/verify_native_go2_navigation.py --live \
  --scenario cancel --forward-m 1.0 --cancel-after-s 0.5 --timeout-s 20 \
  --confirm I_UNDERSTAND_NATIVE_GO2_NAV
```

通过条件：精确终态 `canceled`、`stop_reason=canceled`，报告包含取消原因的 stop 审计，
且 `cancel_latency_s <= 1.25` 秒。

## 4. 突然阻挡停车

运动开始后从侧面放入软障碍：

```bash
python scripts/verify_native_go2_navigation.py --live \
  --scenario sudden_block --forward-m 1.0 --timeout-s 25 \
  --confirm I_UNDERSTAND_NATIVE_GO2_NAV
```

通过条件：trace 出现 `emergency_stop`，command audit 同时出现原因包含 obstacle 的
`stop`；两份证据缺一不可。障碍持续存在时最终失败或超时不影响“先停车”门禁，
但必须另记为未完成路线。

## 5. 卡住退出

```bash
python scripts/verify_native_go2_navigation.py --live \
  --scenario stuck --forward-m 0.6 --timeout-s 25 \
  --confirm I_UNDERSTAND_NATIVE_GO2_NAV
```

通过条件：精确终态 `no_progress`、`stop_reason=no_progress`，command audit 中包含
`native navigation made no progress` 停止；`timed_out` 不算通过。

## 报告保存

每次标准输出都是独立严格 JSON；建议给每次命令追加唯一的
`--report-path evidence/<场景>-<时间>.json`。脚本以 exclusive-create 写入，目标已存在会
拒绝覆盖，非有限传感器值会写为 `null` 而不是非标准 `Infinity/NaN`。验收汇总
必须记录脚本版本、地图 identity/version、传感器摘要、场地布置、五项结果和所有失败
原因。现场阶段通过不代表完整导航抽离 Goal 通过；重定位、探索、巡逻、视觉与三维能力
仍按能力矩阵分别验收。

每个运动报告的 `acceptance.failures` 必须为空；该字段列出状态、实测里程、实际绕行、
停止审计或取消延迟中未满足的联合门禁，禁止人工忽略后把 `ok=false` 改写为通过。

六份报告完成后运行总验收（文件名按现场实际替换）：

```bash
python scripts/summarize_native_go2_acceptance.py evidence/read-only.json \
  evidence/straight.json evidence/obstacle.json evidence/cancel.json \
  evidence/sudden-block.json evidence/stuck.json \
  --output evidence/native-go2-acceptance-summary.json
```

汇总器要求六种场景各一份、运动报告确为 `live`、只读传感器确实 ready、所有单项均
通过且 `acceptance.failures` 为空，并记录每份原始报告的 SHA-256；缺失、重复或失败
任一项均以非零状态退出，汇总文件也使用 exclusive-create，拒绝覆盖旧证据。

## 6. Navigation → teleop → 急停仲裁

该场次只给 teleop 发送零速度：导航开始后，租约获取必须先取消活动导航，随后立即急停
并验证旧租约不能再发运动。真实执行仍可能在抢占前产生极短导航动作，场地按运动测试
标准清空：

```bash
python scripts/verify_native_go2_arbitration.py --live \
  --confirm I_UNDERSTAND_GO2_CONTROL_ARBITRATION \
  --output evidence/go2-live-teleop-estop-arbitration.json
```

通过要求：权威传感器 ready；活动导航精确进入 canceled 且出现 PREEMPTED 事件；teleop
租约成功；零速度 setpoint 被接受；急停产生物理 stop 审计；旧租约随后提交非零速度被
拒绝。dry-run 可验证状态机，但证据注册器只接受 `mode=live` 的报告。
