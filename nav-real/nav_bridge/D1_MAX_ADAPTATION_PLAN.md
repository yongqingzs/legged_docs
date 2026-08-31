# nav_bridge 适配智元 D1 Max 方案

> 当前实现采用低风险双后端方案：X30 原有协议和状态机保留，仅移动到 `include/nav_bridge/x30/`、`src/x30/`；D1 Max 独有实现位于 `include/nav_bridge/d1_max/`、`src/d1_max/`。对外只保留 `launch/nav_bridge.launch.py`，由 `config/nav_bridge.yaml` 的 `robot_type: x30|d1_max` 选择节点。X30 暂不强制重写为 `RobotBackend`，避免破坏既有实机逻辑。

本文档基于以下材料分析：

- 现有桥接实现：`/home/jazzy/drive_ws/src/nav_bridge`
- D1 Max SDK 当前来源：`/home/jazzy/drive_ws/src/RobotSDK-0.2.1`；头文件和 x86_64/aarch64 库均复制到仓库内 `nav_bridge/third_party/robot_sdk`，不链接工作区外路径
- SDK 中文文档：`RobotSDK-0.2.1/docs/zh`
- 原始协议：`RobotSDK-0.2.1/docs/protocol/Protocol-1.3.0.pdf`

> 实现状态（2026-08）：已落地 `RobotBackend` 基础接口、`D1MaxBackend` SDK 后端、ARM64/x86_64 架构选择和 `d1_max_nav_bridge_node` 独立节点；X30 已提供 `RobotBackend` 兼容面。D1 节点实际接入 IMU、运动里程计、关节状态、速度、机器人状态、电池和故障回调，并已使用 RobotSDK-0.2.1 编译通过。X30 的原始 UDP 业务仍保留在 `X30NavBridge`，尚未完全拆成独立 `X30Backend`。

## 1. 结论

D1 Max 不适合直接复用 X30 的 UDP 指令码、心跳和步态状态机。D1 Max SDK 通过 `robot_sdk::SDKClient` 暴露高层控制 API，默认使用 UDP 连接机器人 `IP:8082`（SDK 也支持 WebSocket），并通过 `IDataCallback`/`IControlCallback` 异步上报状态和命令应答。

建议把 `nav_bridge` 从“X30 节点中包含大量 X30 业务逻辑”逐步调整为：

```text
ROS 2 公共接口
        |
        v
RobotBackend / RobotStateStore / ActionExecutor
        |                         |
        +-- X30Backend             +-- D1MaxBackend
            UDP 原始协议               RobotSDK SDKClient
```

当前采用第一阶段方案：保持现有 X30 行为不变，在同一 ROS 2 包中增加 D1 Max 后端和独立启动配置；D1 不复用 X30 的 RL 步态状态机。后续才考虑把 `ActionExecutor` 和状态等待进一步抽到后端接口。

## 2. SDK 能力与现有接口的对应关系

### 2.1 连接和控制权

SDK 提供：

- `Connect(ip, port, block, handler)`、`Disconnect()`；
- `IsConnected()`、`GetConnectionState()`；
- `TakeControl()`、`ReleaseControl()`；
- `OnControlLost`、`OnControlAvailable` 回调；
- `TakeControlAck`、`ReleaseControlAck` 命令应答。

SDK 控制权有一个重要约束：APP 可以抢 SDK 控制权，但 SDK 不能抢回 APP 已持有的控制权。因此 D1 后端不能照搬 X30 的“持续发送 heartbeat 即保持控制权”策略，应以 SDK 的控制源状态和控制权回调为准。建议在控制权丢失时停止速度发送、更新连接/控制状态，并在 `OnControlAvailable` 后按策略重新调用 `TakeControl(5000)`。

### 2.2 运动控制

SDK 提供以下与导航最相关的接口：

| SDK API | 适配用途 |
|---|---|
| `Move(left_right, forward_back, yaw)` | `/cmd_vel` 速度转发 |
| `StandUp()` | `~/stand` |
| `LieDown()` | `~/lie` |
| `Crawl()` | 可作为 D1 专用动作或高度语义 |
| `BalanceStandUp()` / `CrawlWalk()` / `Stair()` | 平衡站立、匍匐行走、登阶动作；0.2.1 已移除 `SetMode` |
| `SetSpeed(1/2/3)` | 低/中/高速档位 |
| `SoftEmergencyStop(bool)` | `~/soft_estop` |
| `TakeControl()` | 获取 SDK 控制权 |
| `ReleaseControl()` | 释放 SDK 控制权 |
| `Gait()`、`Climb()`、`Slim()`、`DSB()` | 可选的 D1 专用动作接口 |

`Move` 参数是归一化百分比 `[-1, 1]`，顺序为左右、前后、偏航；文档说明最新 Move 指令维持 1 秒。桥接层应在收到 `/cmd_vel` 后按固定频率重复调用 `Move`，超时后发送 `Move(0, 0, 0)`，而不是只发送一次。

### 2.3 状态和传感器

SDK 的 `IDataCallback` 包含：

- `OnImuData(ImuData)`：加速度、角速度、四元数；
- `OnMcData(MotionData)`：世界/机体速度、位置、姿态和时间戳；
- `OnSpeedData(SpeedData)`：当前速度；
- `OnJointStateData(JointStateData)`：16 个轮足关节的名称、位置、速度、力矩；
- `OnRobotStateData(RobotState)`：运动状态、模式、速度档位、急停、控制源、电池、里程和关节温度；
- `OnFaultData(FaultDatas)`：故障码、等级和文本；
- `OnControlLost/OnControlAvailable`：控制权变化。

这比 X30 当前只解析 `RcsData`、`MotionStateData`、IMU 和电池的接口更丰富。建议第一版至少接入 IMU、MotionData、RobotState、SpeedData、FaultData，关节状态作为可选扩展。

## 3. 建议的 ROS 2 兼容语义

保持上层导航系统不变，优先复用现有公共接口：

| ROS 2 接口 | D1 Max 实现 |
|---|---|
| `/cmd_vel` | 将 `Twist.linear.x/y`、`angular.z` 限幅后转换为 `Move(left_right, forward_back, yaw)` |
| `/imu/data` | `ImuData` 直接填充标准 IMU；四元数分量按 SDK 定义核对顺序 |
| `/leg_odom` | `MotionData.position`、`v_body` 和姿态转换为 `nav_msgs/Odometry` |
| `/robot_basic_state` | `MotionStatus` 映射为桥接内部的统一基本状态 |
| `/robot_gait_state` | D1 0.2.1 没有 X30 的 11 种步态，且已移除 `SportMode`；当前不伪造 X30 gait 值 |
| `/robot_body_height_state` | 第一版发布 `CRAWL`/未知的 D1 映射；若业务不需要，可标记为不支持 |
| `/battery/level` | 用 `(power1 + power2) / 2`，并在电池在位信息有效时计算 |
| `/battery_text` | 复用 X30 的 RViz 叠加显示 |
| `/charge_manager_state` | 只有确认 D1 SDK/固件提供回充 API 后实现，否则明确返回“不支持” |

推荐保留统一导航语义接口；D1 与 X30 一样使用 `~/set_gait`，当前仅将 `MOUNTAIN(33)` 映射为 RobotSDK `Gait()` 通用模式，其他步态返回不支持，不新增 D1 专用上层接口。

## 4. D1 状态映射方案

D1 SDK 0.2.1 状态主要由 `MotionStatus`、`MachineStatus` 和 `CtrlSource` 组成；0.2.1 已移除旧版 `SportMode`。

| D1 状态 | 统一基本状态建议 | 说明 |
|---|---|---|
| `MOTION_STATUS_LIE_DOWN` | `LYING_DOWN` | 已卧倒 |
| `MOTION_STATUS_STAND_UP` | `STANDING_UP` | 正在站立 |
| `MOTION_STATUS_WALK` / `BALANCE_STAND` | `MOVING` | 行走/平衡站立 |
| `MOTION_STATUS_STAIR` | `STEPPING` | 登阶运动 |
| `MOTION_STATUS_CRAWL` | `STEPPING` | 匍匐运动 |
| `MOTION_STATUS_CRAWL_WALK` | `MOVING` | 匍匐行走 |
| `MOTION_STATUS_LOCKED` | `SOFT_ESTOP` | 锁定/保护状态，需实机确认是否等价急停 |
| `MOTION_STATUS_CLIMB`/`SLIM`/`GAIT` | `STEPPING` | 动作期间统一视为不可接收导航速度，直到状态恢复 |
| `MOTION_STATUS_DSB`/`POS_CONTROL`/`SK_WALK`/`SAND` | `MOVING` | 0.2.1 新增专用动作状态 |

不要仅根据 `MotionStatus` 推断控制权。`RobotState.control_source` 才是判断 SDK、APP 或其他来源的依据；同时要监听 `OnFaultData` 和软件/硬件急停字段。

## 5. 线程和回调设计

SDK 文档明确要求数据回调轻量化，不能在回调中执行耗时计算、文件 I/O 或网络发送。因此 D1 后端应采用以下结构：

1. SDK 回调只做数据复制、时间戳标记和条件变量通知；
2. `RobotStateStore` 保存最近一份 `ImuData`、`MotionData`、`RobotState`、`SpeedData` 和故障快照；
3. ROS 定时器或单独发布线程读取快照并发布 ROS 消息；
4. 动作服务通过条件变量等待目标 `MotionStatus`，不能在 SDK 回调里阻塞；
5. 所有 SDK 命令串行化，避免 `/cmd_vel` 定时器、动作服务和控制权恢复线程同时调用 `SDKClient`。

当前已实现 `D1MaxBackend::DataCallback`/`ControlCallback`，回调只复制/转换数据后调用已注册的轻量回调；后续仍需进一步引入状态仓库和统一命令队列。

## 6. 控制时序

### 6.1 初始化

1. 创建 `SDKClient`，启用 `auto_reconnect=true`，设置连接超时 3~5 s、重连间隔不小于 1 s；
2. 注册数据和控制回调；
3. 调用 `Connect(d1_host_ip, d1_host_port, true)`；
4. 当前实现连接成功后配置 `SetImuConfig(100)`、`SetMcConfig(true)`、`SetSpeedReportConfig(true, 50)`、`SetJointStateConfig(true)`；不会自动 `TakeControl`，首次 `/cmd_vel` 时才尝试获取控制权；
5. 进入 ROS 发布和控制定时器。

### 6.2 `/cmd_vel`

- ROS `linear.x` -> D1 `forward_back`；
- ROS `linear.y` -> D1 `left_right`；
- ROS `angular.z` -> D1 `yaw`；
- 按参数配置的最大速度归一化并限幅到 `[-1,1]`；
- 以 20~50 Hz 重复调用 `Move`，满足 SDK “指令维持 1 秒”的约束；
- 输入超过超时时间发送零速度；
- 当前动作服务与速度定时器共享 SDK 客户端，尚未实现统一命令队列；实机使用时应避免动作服务与高频 `/cmd_vel` 同时调用；
- `control_source != CTRL_SOURCE_SDK`、连接未建立、故障或急停时禁止发送非零速度。

坐标正负号必须用实机验证：SDK 文档定义正 `left_right` 为左移、正 `forward_back` 为前进、正 `yaw` 为左转，和现有 X30 的摇杆符号约定不同。

### 6.3 `~/stand` 和 `~/lie`

D1 SDK 的动作 API 已经封装了底层命令确认，第一版不应照搬 X30 的“toggle 命令重发”状态机：

- `stand`：调用 `StandUp(timeout_ms)`；当前 ROS 服务返回 SDK 命令发送结果，不额外等待最终 `MotionStatus`；
- `lie`：调用 `LieDown(timeout_ms)`，等待 `MOTION_STATUS_LIE_DOWN`；
- 失败时返回 `std::error_code.message()`，同时结合最近的故障和运动状态生成 ROS 服务原因；
- 若 SDK 同步 API 的成功仅代表“命令发送完成”，必须再用 `OnRobotStateData` 等待最终状态，不能把发送成功当作动作完成。

### 6.4 控制权释放

`~/release_control` 调用 `Move(0,0,0)` 后再调用 `ReleaseControl(5000)`。节点关闭时先停止控制定时器、发送零速度，再调用 `Disconnect(true)`。若收到 `OnControlLost`，立即清零速度并把内部控制状态标为不可用。

## 7. 工程改造步骤

### 阶段 A：SDK 可链接性验证

- 在 Ubuntu 22.04 x86_64 主机上确认 GCC 11、CMake 3.8+、Boost 1.74；
- `find_path` 指向 `RobotSDK-0.2.1/include` 的仓库内副本，按架构选择 `librobot_sdk.so.0.2.1`；
- 在 CMake 中增加导入目标 `robot_sdk::robot_sdk`，设置 `BUILD_RPATH` 或安装后的运行时库路径；
- 编写最小连接/回调 smoke test，验证 SDK 版本、连接、TakeControl、Move(0,0,0)、Disconnect。

### 阶段 B：公共后端抽象（已完成的部分）

已落地/保留的组件：

- `include/nav_bridge/robot_backend.hpp`：连接、控制权、速度、stand/lie/estop、状态快照接口；
- `include/nav_bridge/d1_max_backend.hpp` 与 `src/d1_max_backend.cpp`：已封装 `SDKClient` 和 SDK 回调；
- `include/nav_bridge/d1_max_protocol.hpp`：暂未新增，状态映射目前位于 `src/d1_max_backend.cpp`；
- `src/d1_max_nav_bridge_node.cpp`：已实现 ROS 装配层，复用现有话题/服务名称；
- `config/d1_max_params.yaml`、`launch/d1_max_nav_bridge.launch.py`：D1 独立参数和启动入口。

当前可执行入口为 `d1_max_nav_bridge_node`，构建时显式开启 `-DNAV_BRIDGE_BUILD_D1_MAX=ON`。该节点已提供 `/cmd_vel`、`~/stand`、`~/lie`、`~/soft_estop`、`~/release_control`、`~/set_gait`、`~/set_speed`、`/robot_basic_state`、`/battery/level`、`/imu/data`、`/leg_odom`、`/joint_states` 和 `/robot_fault`。

已使用以下命令完成构建验证（Ubuntu 22.04/ROS 2 Jazzy）：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav_bridge \
  --cmake-args -DNAV_BRIDGE_BUILD_D1_MAX=ON -DBUILD_TESTING=OFF
```

安装产物中的 `d1_max_nav_bridge_node` 已由 `ldd` 验证可加载随包安装的 `librobot_sdk.so.0.2.1`。当前 `~/soft_estop` 服务只执行 SDK 的启用急停操作；解除急停应通过后续专用服务或直接扩展接口，不能把重复调用该服务当作安全解除。

现有 `UdpTransport`、`x30_protocol.hpp`、`ActionExecutor` 保留给 X30 后端；将 `ActionExecutor` 改为依赖抽象后端后，才考虑复用其“等待目标状态”框架。

### 阶段 C：ROS 接口兼容和 D1 专用扩展

- 先保证 `/cmd_vel`、`/imu/data`、`/leg_odom`、基本状态、电池和四个基础服务可用；
- `~/set_speed` 已映射 SDK `SetSpeed(1/2/3)`；D1 不暴露 `~/set_mode`，`~/set_gait` 的 `MOUNTAIN(33)` 映射 SDK `Gait()`；
- 对 `~/set_gait`、`~/set_body_height`、`~/charge_command` 做能力声明，不支持的操作返回结构化错误，不伪造成功；
- 故障可增加 `/robot_fault`，类型可先采用 `diagnostic_msgs` 或项目统一消息。

## 8. CMake 和运行时注意事项

- SDK 是预编译 `.so`，已复制到 `third_party/robot_sdk`，构建不依赖工作区外的 `Agibot_D1_Max` 路径；必须按目标架构选择，不能在 x86_64 和 aarch64 间混用；
- SDK 头文件使用 `std::error_code`、`std::function`、`std::shared_ptr`，应确保 C++ 标准和 ABI 与当前工程一致；
- SDK 依赖 Boost 1.74，构建和部署环境需安装对应开发包；
- 建议通过包参数配置 `d1_host_ip`（默认 `192.168.234.1`）和 `d1_host_port`（默认 `8082`），不要硬编码；
- 默认开启 SDK 自动重连，但重连后必须重新确认连接状态、数据上报配置和控制权，不能直接恢复发送速度；
- SDK 文档示例显示 APP 连接后 SDK 可能无法接管，部署说明应明确关闭 APP 控制或先释放 APP 控制权。

仓库内第三方目录结构：

```text
third_party/robot_sdk/
├── include/robot_sdk/       # SDK 头文件
└── lib/
    ├── x86_64/librobot_sdk.so.0.2.1
    └── aarch64/librobot_sdk.so.0.2.1
```

## 9. 自主回充的处理意见

D1 Max 产品文档列出“自主回充 API（选配）”；RobotSDK-0.2.1 已提供 `StartRechargeTask`/`StopRechargeTask` 和离桩任务接口，但仍需在目标固件上验证前置条件与状态回调。现有 X30 的 `~/charge_command` 不能直接移植。

因此分两步：

1. 第一版 D1 后端保留 `~/charge_command` 名称但明确返回“不支持”，或仅发布电池充电状态；
2. 只有拿到 D1 对应固件/SDK 的回充 API、命令参数和状态回调后，才实现 START/STOP/RESET/QUERY，并补充版本和充电桩前置条件验证。

## 10. 验证计划

## 10.1 已实现内容与 X30 对比

| 对比项 | D1 Max（当前实现） | X30（当前实现） |
| --- | --- | --- |
| 后端入口 | `D1MaxBackend`，实现 `RobotBackend` | `X30NavBridge` 同时实现 `NavBridgeBase` 与兼容性的 `RobotBackend` |
| 底层通信 | RobotSDK-0.2.1 `SDKClient`，默认 UDP `IP:8082`；SDK 负责协议编解码和重连 | `UdpTransport` 直接收发 X30 原始 UDP 报文，节点管理心跳/查询/接管 |
| 部署架构 | `third_party/robot_sdk` 内置 x86_64、aarch64 头文件和 `.so`；CMake 按 `CMAKE_SYSTEM_PROCESSOR` 选择 | 不依赖 D1 SDK，使用 X30 自有 UDP 协议 |
| 速度 | ROS `linear.x/y`、`angular.z` 限幅到 `[-1,1]`，转换为 D1 `Move(left_right, forward_back, yaw)`，50 Hz 重发，500 ms 超时清零 | 按 X30 步态/速度上限映射并发送轴指令，控制权期间持续 heartbeat |
| 控制权 | 使用 SDK `TakeControl`/`ReleaseControl` 与 `CtrlSource`、控制权回调；首次 `/cmd_vel` 尝试接管 | 自己维护控制会话、heartbeat、query 和释放流程 |
| 状态来源 | SDK 回调：`RobotState`、`MotionData`、`SpeedData`、`ImuData`、`JointStateData`、`FaultDatas` | 解析 X30 `RcsData`、运动状态、IMU、电池和充电相关 UDP 数据 |
| ROS 已接入 | `/cmd_vel`、`/imu/data`、`/leg_odom`、`/joint_states`、`/robot_basic_state`、`/battery/level`、`/robot_fault`；`~/stand`、`~/lie`、`~/soft_estop`、`~/release_control`、`~/set_speed` | 保留既有 X30 话题、动作服务、步态/体高/充电业务和 RViz 状态显示 |
| 步态/模式 | 0.2.1 移除 `SportMode` 和 `SetMode`，使用 `MotionStatus` 及专用动作（`BalanceStandUp`、`CrawlWalk`、`Stair` 等）；当前 `~/set_mode` 明确返回不支持 | 有 X30 专用 RL 步态、匍匐/力控站立、体高和充电状态机 |
| 回充 | SDK 已有回充/离桩 API，但 nav_bridge 当前尚未接入 ROS 服务，需实机验证 | 已有 X30 专用充电 UDP 流程 |
| 当前风险 | 动作服务尚未等待最终状态；动作和 `/cmd_vel` 尚未统一串行；D1 固件与 SDK 版本仍需实机确认 | X30 逻辑成熟但型号专用，进一步抽象仍在进行 |

D1 的适配重点是“把 SDK 高层 API 转成公共 ROS 接口”，而不是复制 X30 的报文和状态机。两者共享 `/cmd_vel`、IMU、里程计、电池和基础动作语义，但控制权、速度单位、步态集合和回充协议均不可互换。

### 单元和接口测试

- SDK API 错误码到 ROS 服务响应的映射：`not_connected`、`timed_out`、`ControlledDenial`、`operation_canceled`；
- D1 `MotionStatus`、`MachineStatus`、`CtrlSource` 到统一状态的映射；
- `/cmd_vel` 归一化、限幅、超时清零和控制权丢失保护；
- IMU 四元数顺序、坐标系和时间戳；
- 双电池电量聚合、缺电池和未知状态。

### 实机回归顺序

1. 确认机器人网络、SDK 连接和 `CONNECTED` 状态；
2. 确认 `control_source=SDK`，测试 `TakeControl`/`ReleaseControl`；
3. 开启 IMU、运动、速度和关节数据回调；
4. 测试 `~/stand`，确认最终运动状态；
5. 在通用模式低速发送 `/cmd_vel`，确认 20~50 Hz 连续控制和方向；
6. 停止 `/cmd_vel`，确认 1 s 内收到零速度；
7. 测试 `~/soft_estop`、故障回调和控制权丢失；
8. 测试 `~/lie` 和节点关闭流程；
9. 测试速度服务；`~/set_mode` 应确认返回“不支持”，专用姿态命令待后续增加；
10. 最后再验证可选回充能力。

## 11. 后续实机验证与改进项

- D1 Max 实际固件版本，以及 SDK `0.2.1` 是否与该固件完全匹配；
- SDK `Move` 的调用频率、速度档位和各专用姿态的现场行为；
- `LOCKED` 是否等价软急停，以及 `MachineStatus::SAFETY` 的安全语义；
- `MotionData.quat` 的实际排列和坐标系方向；
- `SetMcConfig`、`SetImuConfig` 等配置是否在自动重连后保持；
- 将 RobotSDK-0.2.1 的回充/离桩任务 API 接入 ROS 服务，并验证充电桩前置条件；
- 为动作服务增加最终 `MotionStatus` 等待和超时；
- 为 SDK 命令增加串行执行队列，避免动作服务与 `/cmd_vel` 并发访问；
- 是否需要保留 X30 专用话题，还是允许 D1 节点对不适用接口返回明确错误。

## 12. 导航主机部署与真机验证记录（2026-08-31）

### 部署环境

- 导航主机：`robot@192.168.168.100`（`aarch64`，ROS 2 Humble）。
- 运动主机：`robot@192.168.168.168`，同时具有 `192.168.234.1` 无线/p2p 地址。
- 工作空间：`~/Workspace/driver_ws/src/nav_bridge`。
- 构建选项：`NAV_BRIDGE_BUILD_D1_MAX=ON`、`NAV_BRIDGE_BUILD_X30=OFF`、`BUILD_TESTING=OFF`。
- SDK 库来自仓库内 `third_party/robot_sdk/lib/aarch64`，未链接开发机或 SDK 源目录的绝对路径。

### 网络结论

导航主机访问 `192.168.234.1` 时，系统默认路由会错误地经 `192.168.144.144` 发送，导致 RobotSDK UDP 握手失败。运动主机已开启 IPv4 forwarding；在导航主机添加临时路由后，SDK 仍不稳定地使用错误源路径。因此本部署将 D1 SDK 目标配置为运动主机在共享网段的地址：

```bash
ip route replace 192.168.234.0/24 via 192.168.168.168 dev enP8p1s0
```

`config/d1_max_params.yaml` 的 `d1_host_ip` 已设为 `192.168.168.168`。如果现场网络拓扑不同，应以 `ip route get <运动主机地址>` 验证后再改参数。

### 验证结果

通过唯一入口启动：

```bash
source /opt/ros/humble/setup.bash
source ~/Workspace/driver_ws/install/setup.bash
ros2 launch nav_bridge nav_bridge.launch.py
```

实机日志确认：`D1 Max backend connected to 192.168.168.168:8082`。以下公共 ROS 接口已在真机收到数据：

- `/imu/data`：约 12 Hz；
- `/leg_odom`：约 15 Hz；
- `/joint_states`：约 15 Hz，包含 16 个关节；
- `/battery/level`：读取到 46%；
- `/robot_basic_state`、`/robot_fault`：已建立发布器。

本次未发送非零 `/cmd_vel`，也未调用站立、趴下等会改变机器狗姿态的动作服务；因此控制运动安全性、方向符号和控制权抢占仍需在具备安全看护条件时单独回归。独立 RobotSDK 例程使用 `192.168.234.1:8082` 的失败与 nav_bridge 一致，而使用 `192.168.168.168:8082` 成功，进一步证明此前故障是导航主机 UDP 目标/路由选择问题，不是 ROS 消息转换问题。
