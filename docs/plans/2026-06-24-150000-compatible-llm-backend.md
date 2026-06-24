# 第十五次迭代：OpenAI 兼容 Chat Completions 适配器

## 动机

当前 LLM 层只支持 `mock` 和 `openai`（Responses API）。DeepSeek、Ollama、vLLM、LM Studio
等主流方案均走 Chat Completions + tools 接口。仅改 `OPENAI_BASE_URL` 会触发 Responses API
格式不兼容而静默降级到 MockLLM。需要一个通用适配器覆盖所有 OpenAI 兼容服务。

## 方案

新增 `RDB_LLM=compatible` 后端，走标准 Chat Completions API：

- `robot_brain/llm/compatible_client.py` — `CompatibleLLMClient(LLMClient)`
- 工具 schema 自动转换：Responses API 平铺格式 → Chat Completions 嵌套格式
- 响应解析：`response.choices[0].message.tool_calls` → `list[ToolCall]`
- 降级：超时/API 错误后自动 fallback 到 MockLLM
- 兼容性：保留现有 `openai` 后端（Responses API）不动

## 关键设计决策

1. **不为每个厂商单独写适配器** — 一个 Chat Completions 适配器覆盖全部
2. **保留旧 openai 后端** — 已有依赖不破坏
3. **Content fallback 解析** — 部分小模型不走 tool_calls 字段而是把 JSON 写在 content 里，做兼容解析
4. **不做 streaming** — 第一版只需 tool calling 结果

## 文件变更

| 文件 | 说明 |
|------|------|
| `robot_brain/llm/compatible_client.py` | 新建 — Chat Completions 适配器 |
| `robot_brain/runtime/loop.py` | 注册 `compatible` 分支 |
| `tests/test_compatible_client.py` | 新建 — 13 个单测 |
| `README.md` | LLM 后端配置说明 + 推荐模型表 |

## 验证方式

- [x] 13 新测试全部通过 (test_compatible_client)
- [x] 全量测试通过，0 回归
- [ ] 手动验证：DeepSeek / Ollama 实际调用

## 环境变量

```bash
# DeepSeek
RDB_LLM=compatible
OPENAI_BASE_URL=https://api.deepseek.com
OPENAI_API_KEY=sk-...
RDB_OPENAI_MODEL=deepseek-chat

# Ollama
RDB_LLM=compatible
OPENAI_BASE_URL=http://127.0.0.1:11434/v1
OPENAI_API_KEY=ollama
RDB_OPENAI_MODEL=qwen2.5:7b
```
