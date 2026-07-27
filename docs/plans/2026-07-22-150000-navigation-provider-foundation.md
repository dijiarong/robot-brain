# 第二十一次迭代：可替换导航 Provider 与 Fake 验证底座

## 目标

把导航作为可替换的外部能力接入 robot-brain，使 Agent、安全策略和测试不依赖
DimOS、Nav2、ROS2 或 Go2 WebRTC 的具体实现。

本阶段只支持有界相对目标，不做 SLAM、地图管理、全局路径规划和地点语义导航。

## 能力边界

- `NavigationClient`：统一 `get_state`、`set_relative_goal`、`cancel` 契约。
- `FakeNavigationClient`：可模拟成功、失败、超时、无进展、取消和 provider 不可用。
- `nav_get_state` Tool：内部只读状态查询，不直接进入当前 Skill-only LLM Catalog。
- `nav_go_relative` Skill：提交有界相对目标并等待后端终态。
- `nav_cancel` Skill：高优先级取消，急停状态下仍允许执行。

## 装配规则

- mock backend 默认装配 `FakeNavigationClient`，用于 Agent 场景测试。
- unitree backend 不默认装配 Fake；只有显式注入真实 Provider 后才向规划器暴露新能力。
- 现有 `go2_local_nav` 继续作为 Go2 直接驱动能力，不在本阶段迁移或删除。

## 验收

- [x] Fake 相对目标成功并按机器人朝向更新 odom pose。
- [x] 失败、超时、无进展均返回结构化 `stop_reason`。
- [x] 活动目标可以取消，空闲取消保持幂等。
- [x] mock runtime 自动注册导航 Tool/Skill。
- [x] unitree runtime 未注入 Provider 时不暴露新导航能力。
- [x] 注入 Provider 后 Unitree Planner Catalog 可见 `nav_go_relative` 和 `nav_cancel`。
- [ ] 实现 `DirectGo2NavigationClient`，复用现有 `go2_local_nav` 运动闭环。
- [x] 实现首个外部导航适配器：复用 `topsun-bot/Navigation` 的 Nav2
  `/navigate_to_pose` Action、`/odom` 和取消接口。
- [ ] 真机完成短距离 dry-run/live 验收。

## 下一阶段

优先接入一个外部项目作为独立 Navigation Service。robot-brain 只新增 Adapter，保持
ROS2/SLAM 等依赖不进入核心运行时；验收继续复用本阶段的状态、取消和失败语义。

## Navigation 项目接入

已验证的本地源码为 `/Users/dijia/project/Navigation`，主干接口如下：

- Nav2 Action：`/navigate_to_pose`
- 连续位姿：`/odom`（`odom -> base_link`）
- 全局规划坐标：`map`，Navigation 发布静态 `map -> odom` TF
- 底盘输出：`/cmd_vel -> go2_bridge -> /tmp/go2_sport.sock -> go2_sport_proxy`

robot-brain 默认在 `odom` 坐标系生成目标。Nav2 通过 Navigation 已有 TF 转换到 `map`
执行全局规划；相对目标的 forward/left 会根据当前 odom yaw 转成绝对目标。

在 Navigation 的 ROS2 Humble 容器中 source 工作空间后启用：

```bash
export RDB_NAVIGATION_BACKEND=nav2
export RDB_NAV2_ACTION_NAME=/navigate_to_pose
export RDB_NAV2_ODOM_TOPIC=/odom
export RDB_NAV2_GOAL_FRAME=odom
```

默认只读验收，不产生运动：

```bash
python scripts/verify_nav2_provider.py
```

确认机器人周边安全且底盘桥接正常后，显式发送 0.2 米目标：

```bash
python scripts/verify_nav2_provider.py --live --forward-m 0.2 --timeout-s 12
```

脚本输出结构化 JSON，包含初始位姿、goal id、过程中 progress、最终状态与错误码；超时会
主动取消目标。未安装 `rclpy`、Action server 未启动或 `/odom` 无数据时只报告 unavailable，
不会回退到 Fake，也不会发送运动。
