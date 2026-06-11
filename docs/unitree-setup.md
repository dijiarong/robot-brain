# Unitree Go2 接入指南

## 硬件与环境

| 项目 | 当前配置 |
|------|---------|
| 机型 | Unitree Go2 |
| SDK | [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) |
| 通信协议 | CycloneDDS (DDS) |
| 开发机 | macOS (Apple Silicon / Intel) |
| 连接方式 | Wi-Fi 直连机器狗热点 |
| 机器狗 IP | `192.168.123.161`（默认） |
| 开发机 IP | `192.168.123.x`（自动分配） |

## SDK 安装

> ⚠️ `unitree_sdk2_python` 依赖 CycloneDDS，编译需要 CMake 和 C++ 编译器。macOS 上安装可能较慢。

```bash
# 1. 确保已安装 CMake
brew install cmake

# 2. 从 GitHub 安装 SDK
pip install git+https://github.com/unitreerobotics/unitree_sdk2_python.git

# 如果 CycloneDDS 编译失败，尝试先单独安装：
pip install cyclonedds
pip install git+https://github.com/unitreerobotics/unitree_sdk2_python.git
```

SDK 不是项目的基础依赖。未安装时，`RDB_UNITREE_TRANSPORT=fake`（默认）仍可正常运行项目。

## 网络配置

1. 打开 Go2 电源，等待机器狗站立
2. 在 Mac 上连接 Go2 的 Wi-Fi 热点（通常名为 `Unitree_GoXXXX`）
3. 确认获得 `192.168.123.x` 段 IP
4. 验证网络连通：`ping 192.168.123.161`

如果需要指定网卡：

```bash
export RDB_UNITREE_NET_IFACE="en0"  # 或你的 Wi-Fi 接口名
```

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `RDB_ROBOT` | `mock` | 设为 `unitree` 启用 Unitree 后端 |
| `RDB_UNITREE_TRANSPORT` | `fake` | `fake`=内存模拟, `sdk`=真实 SDK |
| `RDB_UNITREE_MODEL` | `go2` | 机型标识 |
| `RDB_UNITREE_NET_IFACE` | 空 | CycloneDDS 网卡名 |
| `RDB_UNITREE_DRY_RUN` | `true` | `true`=动作不下发, `false`=真实动作 |
| `RDB_UNITREE_MAX_SPEED` | `0.5` | 适配器层速度上限 (m/s) |
| `RDB_UNITREE_MAX_STEP` | `2.0` | 单次动作最大距离 (m) |

## 只读验证

**第一步：确认 fake transport 工作正常**

```bash
python -m examples.run_unitree_smoke --state-only --transport fake
```

**第二步：连接真机，读取状态**

```bash
export RDB_ROBOT=unitree
export RDB_UNITREE_TRANSPORT=sdk
python -m examples.run_unitree_smoke --state-only --transport sdk
```

预期输出包含电量、姿态、站立状态等信息。

## 回退到 Mock

随时可以回退到纯模拟模式：

```bash
# 方法 1：删除环境变量
unset RDB_ROBOT
unset RDB_UNITREE_TRANSPORT

# 方法 2：显式设置
export RDB_ROBOT=mock
```

回退后所有测试和服务正常运行，不需要真机连接。

## 当前限制

- **本迭代 SDK transport 为只读**：只能读取状态，不能发送动作命令
- 动作命令（stop、move、turn）目前只在 fake transport 上可用
- 真实动作能力将在后续迭代中逐步开放
- `dock()` 和 `follow()` 尚未支持（显式 NotImplementedError）

## 故障排查

| 症状 | 排查 |
|------|------|
| `unitree_sdk2_python is not installed` | 确认 SDK 已安装，或改用 `RDB_UNITREE_TRANSPORT=fake` |
| `Failed to connect to Unitree Go2` | 检查 Wi-Fi 连接、ping 机器人 IP |
| `CycloneDDS` 编译失败 | 安装 CMake (`brew install cmake`)，确认 Xcode CLI tools |
| `State read failed` | 机器人可能未启动完成，等待 30s 后重试 |
| 测试失败但不涉及真机 | 确认 `RDB_ROBOT=mock`（默认），不受 SDK 安装影响 |

## 安全注意事项

- **只读验证不需要物理准备**——机器狗不会移动
- 后续动作验证前务必确保：开阔平坦地面、人与机器狗保持 2m 以上距离、手边有物理急停方式（遥控器或 App）
- 真实动作需要输入确认短语 `I_UNDERSTAND_UNITREE_MOVE`
- 任何异常情况，系统会尝试 best-effort stop
