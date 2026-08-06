# 原生视觉伺服验收

视觉验收使用 `nav_visual_servo` 的真实帧源和识别器，并通过 `NavigationClient` 的短程
安全目标运动；不允许绕过导航层直接下发速度。先为本次场次设置唯一 trace：

```bash
export RDB_NATIVE_NAV_TRACE_PATH=evidence/visual-servo-session.jsonl
```

在确认目标名称、相机内参和人员安全区后执行技能。trace 只保存 bbox 中心误差、估计
距离、置信度、观测 age、短程命令和导航终态，不保存相机图像。

完成后生成自动报告：

```bash
python scripts/analyze_native_navigation.py \
  evidence/visual-servo-session.jsonl --output evidence/visual-servo-report.json
```

通过要求：`visual_servo.ok=true`、最终 `stop_reason=target_reached`，每条短程命令都有
精确的 `succeeded` 终态，发生运动后必须重新识别目标，最终连续稳定帧满足居中与距离
容差。只有首次 bbox、只有命令、只有规划成功、运动后没有重捕获，或以丢失/陈旧/
低置信/超时结束均不通过。

报告中的距离来自已声明物体宽度与相机模型，不是外部测距真值；现场应另记录目标实际
尺寸、内参来源及终点人工尺量。需要 3D 目标时，必须记录 body-frame 点云来源，并以
`source=points_3d` 区分，未知坐标系不进入控制。
