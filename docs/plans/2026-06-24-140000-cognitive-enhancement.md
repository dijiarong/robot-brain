# 迭代计划：认知增强 — LLM 感知 + 自主决策升级

## 基本信息

- 创建时间：2026-06-24 14:00:00 CST
- 文件序号：2026-06-24-140000
- 状态：已完成
- 负责人：dijiarong

## 背景

第 13 次迭代建立了远程遥操与媒体网关，但 LLM 规划器仍然是"盲目"的：
- 系统提示仅一行，无状态解读指导
- world.snapshot() 以原始 JSON 传入，LLM 无法理解阈值含义
- 对话历史未传给 LLM，无法理解多轮上下文
- LLM 不会根据电量/障碍/姿态调整决策

用户希望 agent 能自主决策控制机器狗，这要求 LLM 能"看到"并"理解"机器人状态。

## 目标

- [x] PromptBuilder — 可组合的系统提示构建器
- [x] cognitive_snapshot() — 带解读提示的结构化状态呈现
- [x] 对话历史注入 — LLM 看到近期 user↔robot 对话
- [x] 状态感知决策策略 — 条件化规则嵌入提示
- [x] 执行经验丰富化 — 技能名 + 耗时写入长期记忆
- [x] 决策上下文增强 — 记录完整 world snapshot 审计

## 实施方案

### 新增文件

| 文件 | 说明 |
|------|------|
| `robot_brain/llm/prompt_builder.py` | 系统提示构建器 |
| `robot_brain/llm/prompts/__init__.py` | 提示模板包 |
| `robot_brain/llm/prompts/templates.py` | 提示模板常量 |
| `tests/test_prompt_builder.py` | PromptBuilder 单元测试 (29 tests) |
| `tests/test_cognitive_enhancement.py` | 集成测试 (10 tests) |

### 修改文件

| 文件 | 变更 |
|------|------|
| `robot_brain/core/world_state.py` | +cognitive_snapshot() +_build_state_summary() |
| `robot_brain/llm/base.py` | plan() 增加 conversation 参数 |
| `robot_brain/llm/openai_client.py` | 使用 PromptBuilder，接受 conversation |
| `robot_brain/llm/mock.py` | 签名对齐 |
| `robot_brain/cognition/planner.py` | 透传 conversation |
| `robot_brain/cognition/dual_system.py` | 透传 conversation |
| `robot_brain/core/context.py` | AgentContext 添加 conversations 字段 |
| `robot_brain/orchestration/nodes.py` | decide() 注入对话历史 |
| `robot_brain/runtime/loop.py` | 传 conversations、丰富 _remember、增强 _save_decision_context |
| `robot_brain/memory/sqlite_store.py` | decision_context 表增加 world_snapshot 列 |
| `tests/test_fast_reflex_go2.py` | DummyPlanner 适配新签名 |

### 架构决策

1. **PromptBuilder 独立于 LLMClient** — 可独立测试、可替换策略
2. **conversation 参数向后兼容** — 默认 None，所有旧代码不受影响
3. **cognitive_snapshot() 在 WorldState 上** — 解读逻辑属于领域层
4. **AgentContext.conversations 使用 default_factory** — 不强制所有构造点传参
5. **SQLite 迁移使用 ALTER TABLE + try/except** — 幂等，对存量数据库安全

## 验证方式

- [x] 39 新测试全部通过 (test_prompt_builder + test_cognitive_enhancement)
- [x] 全量 340 测试通过，0 回归
- [ ] 手动验证：设置 RDB_LLM=openai 后 LLM 能基于状态做合理决策

## 复盘

完成后可进一步：
- 调优 prompt 文本（基于真实 LLM 调用效果迭代）
- 添加 token 计数/截断保护（防止 prompt 超出模型窗口）
- 将 PromptBuilder 配置化（不同场景用不同策略集）
- 增加"自主巡逻"模式：LLM 在无用户指令时主动感知环境并决策
