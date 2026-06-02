# 迭代计划

这里用于沉淀项目规划、阶段目标和迭代复盘，避免计划只保存在编辑器的隐藏目录中。

## 已归档计划

- [初始架构计划](./initial-architecture.md)：从 `.cursor/plans` 提取的机器狗大脑 Agent 骨架设计。
- [第一次迭代：记忆持久化](./2026-06-02-memory-persistence.md)：使用 SQLite 补齐会话记录、长期经验和待确认任务的本地持久化。
- [第二次迭代：运行状态持久化与重启恢复](./2026-06-02-world-state-recovery.md)：保存世界状态快照，并在重启后恢复最近上下文。
- [第三次迭代：常驻任务调度与对象生命周期](./2026-06-02-task-scheduler-and-object-lifecycle.md)：增加可恢复任务队列、优先级调度、自动回充和对象 TTL。
- [第四次迭代：后台服务、控制 API 与状态页面](./2026-06-02-service-api-dashboard.md)：将 scheduler 包装为常驻服务，并提供 HTTP、WebSocket 与轻量控制台。

## 新增计划

复制 [迭代计划模板](./iteration-template.md)，按 `YYYY-MM-DD-topic.md` 命名，例如：

```text
2026-06-02-robot-adapter.md
```

每次迭代结束后，请及时更新任务状态和复盘记录。
