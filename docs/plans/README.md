# 迭代计划

这里用于沉淀项目规划、阶段目标和迭代复盘，避免计划只保存在编辑器的隐藏目录中。

## 已归档计划

- [初始架构计划](./initial-architecture.md)：从 `.cursor/plans` 提取的机器狗大脑 Agent 骨架设计。
- [第一次迭代：记忆持久化](./2026-06-02-memory-persistence.md)：使用 SQLite 补齐会话记录、长期经验和待确认任务的本地持久化。
- [第二次迭代：运行状态持久化与重启恢复](./2026-06-02-world-state-recovery.md)：保存世界状态快照，并在重启后恢复最近上下文。
- [第三次迭代：常驻任务调度与对象生命周期](./2026-06-02-task-scheduler-and-object-lifecycle.md)：增加可恢复任务队列、优先级调度、自动回充和对象 TTL。
- [第四次迭代：后台服务、控制 API 与状态页面](./2026-06-02-service-api-dashboard.md)：将 scheduler 包装为常驻服务，并提供 HTTP、WebSocket 与轻量控制台。
- [第五次迭代：LLM 输出校验与错误码](./2026-06-08-llm-validation-error-codes.md)：在 LLM 层加入结构化输出校验，引入统一 ErrorCode 枚举覆盖全链路错误传播。
- [第六次迭代：Unitree 机器狗适配与真机安全闭环](./2026-06-08-093904-unitree-robot-adapter.md)：接入宇树机器狗适配层，先只读再低速短步动作，保持 mock 默认与安全边界。
- [第七次迭代：真实 Unitree SDK Transport 接入与只读实机验证](./2026-06-08-101349-unitree-sdk-transport-readonly.md)：把 Unitree transport 抽象接到真实 SDK 或通信接口，先完成只读状态验证，不开放真实动作。
- [第八次迭代：WebRTC 真实姿态/急停动作下发与安全门](./2026-06-08-141602-unitree-webrtc-posture-actions.md)：在 WebRTC transport 开放不产生平移的姿态/急停命令，引入 enable_motion 硬安全门，平移仍拒绝。
- [第九次迭代：Unitree Go2 WebRTC 实机操控安全闭环](./2026-06-11-172047-unitree-live-control-loop.md)：低速限时操控、motion lease、Web teleop、MCF 混合通道；阶段 A–E 代码/测试完成，分级验收待现场。
- [第十次迭代：Go2 Perception Bridge](./2026-06-13-000000-unitree-perception-bridge.md)：打通 Go2 sport state → Observation → WorldState，新增 RobotSelfState 模型与 UnitreePerceptionAdapter；代码/测试完成，真机验证待现场。
- [第十一次迭代：Go2 原生技能族](./2026-06-14-000000-go2-skill-family.md)：nudge/scan/retreat 分段 drive 映射 LLM tool；代码/测试完成，真机待现场。
- [第十二次迭代：Go2 快反规则 + 后端工具过滤](./2026-06-15-120000-go2-fast-reflex-and-tool-filter.md)：FastReflex 读 robot_self_state；unitree 后端 LLM tool 白名单；代码/测试完成，真机待现场。
- [第十四次迭代：认知增强 — LLM 感知 + 自主决策升级](./2026-06-24-140000-cognitive-enhancement.md)：PromptBuilder 可组合提示、cognitive_snapshot 状态解读、对话历史注入、状态感知决策策略；代码/测试完成。
- [第十五次迭代：OpenAI 兼容 Chat Completions 适配器](./2026-06-24-150000-compatible-llm-backend.md)：新增 `RDB_LLM=compatible` 后端，走标准 Chat Completions + tools API，覆盖 DeepSeek / Ollama / vLLM / LM Studio；代码/测试完成。

## 后续方向池

- [第十二次及后续迭代方向（备选）](./2026-06-15-000000-next-iteration-options.md)：FastReflex、tool 过滤、服务监控、感知流等多方向对比与选型建议。

## 新增计划

复制 [迭代计划模板](./iteration-template.md)。从 2026-06-08 开始，计划文件按 `YYYY-MM-DD-HHMMSS-topic.md` 命名，使用本地时间戳，方便同一天多次迭代时按先后排序，例如：

```text
2026-06-08-093904-unitree-robot-adapter.md
```

每次迭代结束后，请及时更新任务状态和复盘记录。
