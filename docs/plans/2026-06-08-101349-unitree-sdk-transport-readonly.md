# 第七次迭代：真实 Unitree SDK Transport 接入与只读实机验证

## 基本信息

- 创建时间：2026-06-08 10:13:49 CST
- 文件序号：2026-06-08-101349
- 状态：已完成（SDK transport 骨架 + fake client 测试 + smoke test）；真机只读验证待 SDK 安装后执行
- 负责人：dijia
- 前置迭代：[第六次迭代：Unitree 机器狗适配与真机安全闭环](./2026-06-08-093904-unitree-robot-adapter.md)

## Project Requirements

### Goal

把第六次迭代完成的 `UnitreeTransport` 抽象层接到真实宇树 SDK 或真实通信接口上，先完成只读实机验证：能稳定连接机器狗、读取状态、映射为 `UnitreeState` / `RobotState`，并保留 fake transport、dry-run 和 mock 后端作为默认安全路径。

本轮目标不是让机器狗开始真实移动，而是确认“软件能看见真机，并且看见的状态可信”。只有只读链路稳定后，后续才进入真实 stop、姿态命令和低速动作验证。

### Features

- 新增真实 transport 实现，例如 `UnitreeSDKTransport` 或按 SDK 类型命名的 transport。
- 支持显式配置真实 transport，不改变默认 fake/mock 行为。
- 支持连接、断开、读取状态，并把 SDK 原始状态转换成 `UnitreeState`。
- smoke test 增加真实只读模式，能打印连接状态、电量、姿态/站立状态、运动状态、错误码等信息。
- 当 SDK 未安装、网络不通、真机未连接或读取失败时，错误信息必须明确，不影响 mock/fake 测试。
- 为真实 transport 增加边界测试，使用 fake SDK/client 模拟连接成功、连接失败、状态缺字段、SDK 异常。
- README 或专门文档记录当前机型、SDK 安装方式、网络配置、只读验证命令和常见故障。
- 保持 `UnitreeRobot` adapter 逻辑不变，真实 SDK 细节只存在于 transport 层。

### Constraints

- 本轮不开放真实移动、真实转向、真实 dock、真实 follow。
- 本轮不让 LLM 参与真机动作测试。
- `RDB_ROBOT=mock` 仍为默认，`unitree` 后端仍必须显式开启。
- 如果真实 SDK 只能在 Linux 或特定网络环境运行，需要把平台限制写清楚，不能让 macOS 开发体验变差。
- 不把真实 SDK 设为基础依赖；如有必要放入 optional dependency 或运行时动态导入。
- 不在 `skills`、`cognition`、`orchestration` 中引用 Unitree SDK 类型。
- 真实 transport 的异常不能拖垮常驻服务；读取失败应被转换为明确错误或 stopped 状态。

### Success Criteria

- 可以通过配置选择 fake transport 或真实 Unitree transport。
- 未安装 SDK 时，项目默认测试和 mock runtime 不受影响。
- 真实只读 smoke test 能连接机器狗并打印至少一次状态快照。
- `UnitreeState` 至少包含：`connected`、`battery_level`、`heading_degrees`、`is_standing`、`is_moving`、`error_code`。
- 读取失败、网络断开、SDK 抛错都有可读错误信息和测试覆盖。
- `UnitreeRobot.get_state()` 可以基于真实 transport 返回标准 `RobotState`。
- 自动化回归测试通过，并覆盖 fake SDK/client 下的真实 transport 行为。
- 文档说明如何回退到 fake/mock，以及只读验证前需要确认的物理和网络条件。

## 背景

第六次迭代已经完成 `UnitreeTransport` 抽象、`FakeUnitreeTransport`、`UnitreeRobot`、dry-run smoke test 和 26 个 adapter 测试。当前 `AgentRuntime.create(settings=Settings(robot_backend="unitree"))` 可以跑通，但底层仍是 fake transport。

第七次迭代应该把注意力放在真实 SDK transport，而不是扩大动作能力。只有真实状态读取稳定，后续 stop、stand/sit、低速短步动作才有可靠基线。

## 推荐实施方案

### 1. 先确认真实接入路径

需要先确认：

- 机器狗型号：Go1、Go2、B1、B2 或其他
- SDK 类型：官方 Python SDK、C++ SDK Python binding、ROS/ROS2 bridge、UDP/LCM 网络接口
- 运行平台：macOS、本机 Linux、机器人随附工控机、Docker/虚拟机
- 网络方式：网线、Wi-Fi、机器人热点、同网段路由器
- 是否能在当前开发机直接读状态，还是必须在机器人配套主机上运行

### 2. Transport 文件组织

建议新增：

| 文件 | 职责 |
|------|------|
| `robot_brain/actuation/unitree_sdk.py` | 真实 SDK transport，实现 `UnitreeTransport` |
| `tests/test_unitree_sdk_transport.py` | 使用 fake SDK/client 测试真实 transport 的映射和异常路径 |
| `docs/unitree-setup.md` | SDK 安装、网络配置、只读验证流程 |

如果真实 SDK 名称或接入方式明确，也可以把文件名改成更具体的名字，例如 `unitree_go2_sdk.py` 或 `unitree_udp.py`。

### 3. 配置建议

沿用第六次迭代已有配置，并补充一个 transport selector：

| 配置 | 默认值 | 说明 |
|------|--------|------|
| `RDB_ROBOT` | `mock` | `mock` 或 `unitree` |
| `RDB_UNITREE_TRANSPORT` | `fake` | `fake` 或 `sdk`，避免 `unitree` 后端一开启就碰真机 |
| `RDB_UNITREE_MODEL` | 空 | 机型标识 |
| `RDB_UNITREE_NET_IFACE` | 空 | SDK 需要指定网卡时使用 |
| `RDB_UNITREE_DRY_RUN` | `true` | 本轮仍保持 true |

### 4. 状态映射

真实 SDK 的原始字段不要直接传到上层。先统一映射成：

| `UnitreeState` 字段 | 说明 |
|--------------------|------|
| `connected` | SDK/client 连接是否可用 |
| `battery_level` | 百分比，范围 0-100 |
| `position` | 如果 SDK 暂无可靠全局位置，保持默认 `Position()` |
| `heading_degrees` | 机体航向；若 SDK 只有 yaw 弧度，需要转换 |
| `is_standing` | 站立/趴下/姿态模式的保守布尔映射 |
| `is_moving` | 是否正在运动；无法判断时保守为 false |
| `error_code` | SDK 原始错误码；没有则 0 |

### 5. Smoke Test 调整

`examples/run_unitree_smoke.py` 建议支持：

```text
python -m examples.run_unitree_smoke --state-only --transport fake
python -m examples.run_unitree_smoke --state-only --transport sdk
```

本轮 `--transport sdk` 只允许 state-only。即使用户传 `--actions --live`，也应明确拒绝，直到下一次迭代专门开放真实 stop/姿态动作。

## 分阶段计划

### 阶段 A：环境确认 ✓

- [x] 确认机型和 SDK 类型 → Go2 + unitree_sdk2_python
- [x] 确认可运行平台和网络连接方式 → macOS + Wi-Fi 直连
- [x] 记录 SDK 安装步骤和版本 → docs/unitree-setup.md
- [x] 确认只读状态 API 的字段来源 → SportModeState via DDS

验收：

- [x] `docs/unitree-setup.md` 中写清机型、SDK、平台和网络要求
- [x] 不满足环境时有明确阻塞说明

### 阶段 B：真实 Transport 骨架 ✓

- [x] 新增真实 transport 类，实现 `connect()`、`disconnect()`、`read_state()`、`send_command()`
- [x] `send_command()` 第一版显式拒绝（raise NotImplementedError）
- [x] SDK 动态导入，缺依赖时报可读错误
- [x] 增加 `RDB_UNITREE_TRANSPORT` 配置

验收：

- [x] 未安装 SDK 时，mock/fake 测试仍能通过（140 tests pass）
- [x] fake SDK/client 单测覆盖 connect/read/disconnect（19 新测试）

### 阶段 C：状态映射与错误处理 ✓

- [x] 把真实 SDK 状态映射成 `UnitreeState`（`_map_state` + `_map_sport_state`）
- [x] 处理缺字段、单位转换、异常、连接断开（None 容错 + ConnectionError 包装）
- [x] 读取失败时保留错误上下文，避免服务崩溃

验收：

- [x] battery、heading、standing、moving、error_code 映射有测试
- [x] SDK 异常路径有测试

### 阶段 D：只读实机验证（待 SDK 安装）

- [x] smoke test 支持 `--transport sdk --state-only`
- [ ] 在真实机器狗连接环境中读取一次状态 → 待 SDK 安装完成
- [ ] 保存或记录输出摘要，补进复盘

验收：

- [ ] 真机状态可读 → 待验证
- [x] 断开连接后的错误可读（测试覆盖 + smoke test 错误输出）
- [x] 默认 fake/mock 行为不变

## 风险与缓解

| 风险 | 缓解 |
|------|------|
| SDK 只能在 Linux/特定网卡运行 | transport 动态导入，macOS 保持 fake/mock 可用 |
| SDK 文档字段和真实返回不一致 | 先写字段探测日志，只读验证后再固化映射 |
| 没有可靠位置字段 | `position` 保持默认，不伪造导航能力 |
| 状态读取阻塞 | 给 SDK 调用加超时或在线程中隔离 |
| 用户误触发真实动作 | 本轮 sdk transport 只读，动作入口拒绝 |

## 验证方式

- [ ] `python -m compileall -q robot_brain config tests examples`
- [ ] fake SDK/client 单元测试
- [ ] 现有 Unitree adapter 测试继续通过
- [ ] mock runtime/service 回归测试继续通过
- [ ] `python -m examples.run_unitree_smoke --state-only --transport fake`
- [ ] `python -m examples.run_unitree_smoke --state-only --transport sdk`

## 复盘记录

### 实际环境

- 机型：Unitree Go2
- SDK：`unitree_sdk2_python`（GitHub，依赖 CycloneDDS）
- 开发机：macOS，Wi-Fi 直连机器狗热点
- SDK 安装状态：CycloneDDS 编译耗时较长，macOS 可用但需要 CMake；当前开发环境未完成安装

### 本次新增

| 文件 | 职责 |
|------|------|
| `robot_brain/actuation/unitree_sdk.py` | 真实 SDK transport，动态导入 SDK，read-only，send_command 显式拒绝 |
| `tests/test_unitree_sdk_transport.py` | 19 个测试（fake SDK client 注入），覆盖 connect/disconnect/read_state/null 字段/异常/read-only command |
| `docs/unitree-setup.md` | Go2 接入指南：SDK 安装、网络配置、只读验证、回退方法、故障排查 |

### 本次修改

| 文件 | 变更 |
|------|------|
| `config/settings.py` | 新增 `unitree_transport` 配置（`fake` 或 `sdk`） |
| `robot_brain/runtime/loop.py` | 工厂基于 `unitree_transport` 选择 fake 或 SDK transport |
| `examples/run_unitree_smoke.py` | 新增 `--transport fake\|sdk` 参数，sdk 模式限制只读，错误输出友好 |

### 验证结果

- 140 tests, all pass（原 121 + 新 19）
- `python -m compileall` 通过
- `--transport fake --state-only` 和 `--transport fake --actions` 正常
- `--transport sdk --state-only` 未安装 SDK 时输出清晰错误提示
- `--transport sdk --actions` 被显式拒绝
- mock runtime / service 回归测试不受影响

### 设计要点

- `UnitreeSDKTransport` 使用 injected client 模式支持测试，不依赖真实 SDK
- `_import_sdk()` 动态导入，缺依赖时提供安装指引
- `send_command()` 本轮 raise `NotImplementedError`，确保只读
- `_map_state()` 对 None 字段容错，不会因缺字段崩溃
- 真实 SDK 调用 via `asyncio.to_thread()` 隔离阻塞

### 遗留

- 真机只读验证需要完成 SDK 安装后手动执行 `--transport sdk --state-only`
- Go2 SportModeState 的字段映射基于文档推测，连接真机后可能需要微调
- 电量字段（battery_level）在 sport state 中可能不直接可用，需要接 low-level state 或 BMS topic
- 下一步：SDK 安装完成 → 真机只读验证 → 真实 stop/stand → 低速动作
