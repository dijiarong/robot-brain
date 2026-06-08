# 第五次迭代：LLM 输出校验、Tool Call Schema 与结构化错误码

## 基本信息

- 日期：2026-06-08
- 状态：已完成
- 负责人：dijia

## 背景

项目接入真实 LLM（OpenAI）后，模型返回的 function_call 可能包含不存在的 skill 名称、非法参数 JSON 或不符合 schema 的参数结构。此前这些情况要么直接 crash（json.loads 失败），要么流入后续编排图被 SafetyValidator 拦截——但错误信息是散落的裸字符串，无法被程序化消费。

本次迭代在 LLM 适配器层加入结构化输出校验，并引入统一错误码枚举。Safety → Orchestration → Runtime → API 链路通过 `error_code` 字段传播结构化错误；LLM 层的校验错误则记入 `OpenAIClient.validation_errors` 和 Planner 的 short-term memory，供诊断和 replan 使用。当 LLM 输出全部被过滤时，主流程视为空计划正常完成，不会主动带出 LLM 错误码到 RunResult。

## 目标

- [x] 引入 `ErrorCode` 枚举和 `BrainError` 结构化错误模型
- [x] 新建 `LLMOutputValidator`：在 LLM 返回后立即校验 skill 白名单和参数 schema
- [x] OpenAI 适配器增强：JSON 解析容错、validator 集成、`last_error` 结构化降级原因
- [x] SkillRegistry 输出 `strict: True` schema，启用 OpenAI 服务端校验
- [x] SafetyValidator 所有拒绝路径附带 `error_code`
- [x] `GraphState` 和 `RunResult` 传播 `error_code` 到 API 响应
- [x] Planner 集成 validator，将被拦截的 LLM 错误记入 short-term memory 供 replan 参考

## 实施方案

### 新增模块

| 文件 | 职责 |
|------|------|
| `robot_brain/core/errors.py` | `ErrorCode` StrEnum + `BrainError` BaseModel |
| `robot_brain/llm/output_validator.py` | `LLMOutputValidator`：skill 白名单 + 参数 schema 校验 |
| `tests/test_output_validator.py` | 覆盖 validator、OpenAI client、SafetyValidator、RunResult 的错误码测试 |

### 修改模块

| 文件 | 变更 |
|------|------|
| `robot_brain/llm/openai_client.py` | JSON 解析 try/except、可选 validator 集成（需传入 skills）、`last_error`/`validation_errors` |
| `robot_brain/skills/registry.py` | `has()` 方法、`tools(strict=True)` |
| `robot_brain/safety/validator.py` | `ValidationResult.error_code` 字段，所有 reject 路径附带错误码 |
| `robot_brain/orchestration/state.py` | `GraphState` 增加 `error_code` 字段 |
| `robot_brain/orchestration/nodes.py` | `select_action`/`validate`/`execute`/`reflect` 传播 `error_code` |
| `robot_brain/runtime/loop.py` | `RunResult.error_code`、`_to_result` 传播 |
| `robot_brain/cognition/planner.py` | 集成 `LLMOutputValidator`，错误记入 short-term memory |

### 错误码设计

三层错误码覆盖主要失败场景：

- **LLM 层**：`llm_timeout`、`llm_api_error`、`llm_invalid_output`、`llm_unknown_skill`、`llm_param_validation`、`llm_degraded`
- **安全层**：`safety_not_whitelisted`、`safety_invalid_params`、`safety_estop_active`、`safety_battery_critical`、`safety_precondition_failed`、`safety_motion_violation`、`safety_confirmation_required`
- **运行时**：`runtime_max_iterations`、`runtime_missing_checkpoint`、`runtime_skill_not_found`、`runtime_no_result`

## 验证方式

- [x] 自动化测试：LLMOutputValidator 正确过滤非法 skill 和参数
- [x] 自动化测试：OpenAI client JSON 解析失败时不 crash，记录结构化错误
- [x] 自动化测试：SafetyValidator 各拒绝路径携带正确 error_code
- [x] 自动化测试：RunResult.error_code 正确传播到 API 响应
- [x] 自动化测试：Planner 过滤 LLM 非法输出并记入 memory
- [x] 回归：95 个测试全部通过（原 73 + 新 22）
- [x] 编译检查：`python -m compileall -q robot_brain config tests examples` 通过

## 复盘

本次迭代建立了 LLM 输出校验和错误码体系：

- `LLMOutputValidator` 作为 LLM 和编排图之间的防护层，确保只有合法的 tool call 进入决策流程
- 结构化 `ErrorCode` 使上层（API、告警、日志系统）可以程序化判断错误类型，不再依赖字符串匹配
- Safety / Orchestration / Runtime / resume 各失败路径均通过 `RunResult.error_code` 传播到 API；LLM 层错误则通过 `validation_errors` 和 short-term memory 可观测
- OpenAI 适配器的 `last_error` 和 `validation_errors` 提供细粒度的降级诊断信息
- `AgentRuntime.create()` 在创建 OpenAIClient 时注入 skills，使适配器层也可做前置过滤；外部传入的 LLM 通过 `set_skills()` 注入
- `strict: True` schema 在 OpenAI 服务端就进行参数校验，减少无效请求到达本地

遗留问题：

- 当 LLM 所有 tool call 都被 validator 过滤时，当前行为是返回空列表（等同于"无计划"），主流程正常完成且 RunResult.error_code 为 None；后续可考虑自动重试或将此情况提升为显式错误码
- 错误码尚未接入监控/告警系统，需要配合日后的可观测性迭代
- `strict: True` 对 OpenAI Responses API 生效，其他 LLM 后端需要各自处理

### 下一步计划

- 第二优先：真实机器人 SDK 接入后的安全边界和恢复逻辑
- 第三优先：schema migration、认证、OpenAPI 文档
