# 第十五次迭代验证记录 — OpenAI 兼容 Chat Completions 适配器

## 验证目标

验证 `RDB_LLM=compatible` 后端能正确：
1. 将工具 schema 转换为 Chat Completions 格式（去掉 `strict`）
2. 解析 `tool_calls` 响应为 `ToolCall` 列表
3. 失败时降级到 MockLLM
4. 通过 AgentRuntime factory 正确创建

---

## 单元测试验证

```
$ python -m pytest tests/test_compatible_client.py -v
14 passed in 0.33s
```

覆盖项：
- ✅ 工具 schema 转换（strict 字段被剥离）
- ✅ tool_calls 解析（多 tool、malformed JSON 跳过）
- ✅ content fallback 解析（JSON array / single dict / plain text）
- ✅ 超时/API 错误降级 MockLLM
- ✅ 重试机制（initial + max_retries）
- ✅ 恢复后清除 degraded 状态
- ✅ AgentRuntime factory 正确创建 CompatibleLLMClient

---

## 端到端模拟验证

### 场景 1: DeepSeek Chat — 正常 tool calling

```bash
RDB_LLM=compatible
OPENAI_BASE_URL=https://api.deepseek.com
RDB_OPENAI_MODEL=deepseek-chat
```

**输入**: `report status`, battery=85%, position=(0,0)

**发送的 tools schema**:
```json
[{
  "type": "function",
  "function": {
    "name": "report",
    "description": "Send report",
    "parameters": {}
  }
}]
```

✅ `strict` 字段已正确剥离（第三方服务不支持该字段）

**响应解析结果**:
```
report({'message': 'Battery at 85%, position (0,0). All systems nominal.', 'severity': 'info'})
```

✅ tool_calls 正确解析为 ToolCall

### 场景 2: Ollama 本地 — content fallback

部分本地模型不使用 `tool_calls` 字段，而是在 `content` 中返回 JSON。

**模拟 content 响应**:
```json
[{"name": "stop", "arguments": {}}]
```

**解析结果**: 1 个 ToolCall(skill_name="stop")

✅ Content fallback 解析正常工作

### 场景 3: API 不可用 — 降级

**模拟**: `asyncio.TimeoutError` / `RuntimeError("connection refused")`

**结果**: 
- `is_degraded = True`
- 自动调用 `MockLLM.plan()` 返回合理结果
- 下次成功调用自动恢复 `is_degraded = False`

✅ 降级和恢复机制按预期工作

---

## 真实 API 验证（待补充）

> ⚠️ 当前环境未配置 `OPENAI_API_KEY`，以下验证待有 API access 时补充：

- [ ] DeepSeek API 实际调用（deepseek-chat）
- [ ] Ollama 本地调用（qwen2.5:7b，需先 `ollama pull qwen2.5:7b`）

验证命令：
```bash
# DeepSeek
export RDB_LLM=compatible OPENAI_BASE_URL=https://api.deepseek.com OPENAI_API_KEY=sk-... RDB_OPENAI_MODEL=deepseek-chat
python -c "
from robot_brain.runtime.loop import AgentRuntime
import asyncio
rt = AgentRuntime.create()
print(asyncio.run(rt.run_command('report status')))
"

# Ollama
export RDB_LLM=compatible OPENAI_BASE_URL=http://127.0.0.1:11434/v1 OPENAI_API_KEY=ollama RDB_OPENAI_MODEL=qwen2.5:7b
python -c "
from robot_brain.runtime.loop import AgentRuntime
import asyncio
rt = AgentRuntime.create()
print(asyncio.run(rt.run_command('nudge forward 20cm')))
"
```

---

## 结论

CompatibleLLMClient 代码路径完整验证通过：
- 工具 schema 转换正确（strict 剥离）
- 响应解析覆盖标准 tool_calls + content fallback
- 降级/恢复机制健壮
- AgentRuntime factory 正确集成

待真实 API 补充后即可标记为完全验证。
