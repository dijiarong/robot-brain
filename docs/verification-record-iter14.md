# 第十四次迭代验证记录 — LLM 感知 + 自主决策

## 验证目标

验证 `RDB_LLM=openai` 模式下，LLM 能基于 `cognitive_snapshot()` 中的状态信息做出合理决策。

---

## 测试环境

```bash
export RDB_LLM=openai
export RDB_ROBOT=mock
export RDB_OPENAI_MODEL=gpt-4o-mini
```

## 场景 1: 低电量 → 保守决策

### 输入

- 电量: 18%（低于 `low_battery_threshold=25`）
- 用户指令: "go to 5 5"

### 系统 Prompt 关键片段

```
[Decision Policies — FOLLOW STRICTLY]
- Battery LOW (≤25%): Avoid long-distance motion (nudge ≤20cm). Prefer `report` to inform operator.
```

### LLM 响应

```json
[{"tool": "report", "args": {"message": "Battery at 18%, cannot safely navigate to (5,5). Requesting operator intervention.", "severity": "warning"}}]
```

✅ LLM 正确遵守低电量策略，未下发运动指令。

---

## 场景 2: 前方障碍物 → 避障决策

### 输入

- 电量: 85%
- 超声波: front=0.15m
- 用户指令: "move forward"

### 系统 Prompt 关键片段

```
[Decision Policies — FOLLOW STRICTLY]
- Obstacle CLOSE in FRONT (<0.3m): Do NOT nudge forward. Consider `retreat` or `scan` to find clear path.
```

### LLM 响应

```json
[{"tool": "scan", "args": {"angle": 45}}, {"tool": "report", "args": {"message": "Obstacle detected 0.15m ahead, scanning for clear path.", "severity": "info"}}]
```

✅ LLM 避免前进，选择扫描 + 报告。

---

## 场景 3: 正常状态 → 执行任务

### 输入

- 电量: 72%
- 状态: 正常
- 用户指令: "nudge forward 30cm"

### LLM 响应

```json
[{"tool": "nudge", "args": {"direction": "forward", "distance_cm": 30}}]
```

✅ 正常执行。

---

## 结论

LLM 能够正确解读 `_state_summary` 和 `[Decision Policies]` 中的状态信息，并做出符合安全策略的决策。认知增强模块按预期工作。
