# ROSClaw 项目详细分析报告

## 1. 项目概述

ROSClaw 是一个 Python 包，提供生产级中间件，通过 ROS 2 和模型上下文协议（MCP）将 LLM 连接到物理机器人（UR5e 机械臂）。它实现了四层安全架构，带有基于 MuJoCo 的数字孪生防火墙。

**核心定位：** 连接 LLM 与物理机器人的桥梁——让 AI 代理安全地控制真实机器人。

## 2. 整体架构

### 2.1 四层架构总览

```mermaid
graph TB
    subgraph OpenClaw["OpenClaw Agent Layer<br/>TypeScript/Node.js"]
        AEP["Agent Event Loop<br/>src/agents/acp-spawn.ts"]
        MCB["mcporter Bridge<br/>MCP 集成桥"]
    end

    subgraph ROSClaw["ROSClaw MCP Server<br/>Python - stdio JSON-RPC 2.0"]
        direction TB
        L4["Layer 4: Embodiment MCP<br/>UR5MCPServer<br/>工具: ur5_move_joints, ur5_get_joint_states..."]
        L3["Layer 3: Digital Twin<br/>DigitalTwinFirewall<br/>MuJoCo 物理仿真<br/>碰撞/限位/力矩检测"]
        L2["Layer 2: Semantic-HAL<br/>DataFlywheel + RingBuffer<br/>高频数据捕获 1kHz"]
        L1["Layer 1: ROS 2 Runtime<br/>UR5ROSNode<br/>rclpy 节点<br/>发布者/订阅者/ActionClient"]
    end

    subgraph Physical["Physical Robot - UR5e"]
        RC["ROS 2 Controller<br/>RTDE/URScript"]
    end

    AEP -->|ACP Protocol| MCB
    MCB -->|stdio MCP 协议| L4
    L4 --> L3
    L3 --> L2
    L2 --> L1
    L1 -->|ROS 2 DDS| RC
    RC -->|反馈| L1

    style OpenClaw fill:#e1f5fe
    style ROSClaw fill:#fff3e0
    style Physical fill:#e8f5e9
    style L4 fill:#ce93d8,color:#000
    style L3 fill:#ef5350,color:#fff
    style L2 fill:#42a5f5,color:#fff
    style L1 fill:#66bb6a,color:#fff
```

### 2.2 四层架构详解

| 层级 | 模块 | 职责 | 延迟目标 | 关键技术 |
|------|------|------|----------|----------|
| Layer 4 | MCP Server | 对外暴露 MCP 工具接口 | - | MCP/JSON-RPC 2.0 |
| Layer 3 | Digital Twin | 轨迹安全验证 | < 10ms | MuJoCo 物理仿真 |
| Layer 2 | Semantic-HAL | 高频数据捕获/缓存 | < 1ms | NumPy RingBuffer |
| Layer 1 | ROS 2 Runtime | 实际硬件通信 | - | rclpy/DDS |

## 3. 模块详细分析

### 3.1 项目结构

```
rosclaw/
├── src/rosclaw/
│   ├── __init__.py                    # 包入口 - 导出防火墙相关类
│   ├── data/                          # Layer 2: 数据层
│   │   ├── __init__.py
│   │   ├── ring_buffer.py             # 高性能环形缓冲区
│   │   └── flywheel.py               # 事件驱动数据捕获系统
│   ├── firewall/                      # Layer 3: 安全防火墙
│   │   ├── __init__.py
│   │   └── decorator.py               # DigitalTwinFirewall + 装饰器
│   ├── mcp/                          # Layer 4: MCP 服务
│   │   ├── __init__.py
│   │   └── ur5_server.py             # UR5ROSNode + UR5MCPServer
│   └── specs/
│       └── ur5e.xml                   # UR5e MuJoCo MJCF 模型
├── tests/
│   ├── test_data_layer.py            # RingBuffer + DataFlywheel 测试
│   ├── test_firewall.py              # DigitalTwinFirewall 测试
│   └── test_mcp_server.py            # MCP 结构测试（mock rclpy）
├── scripts/
│   └── integration_test.py           # 端到端集成测试
├── docs/
│   ├── OPENCLAW_INTEGRATION.md       # OpenClaw 集成文档
│   └── design/
│       ├── mcp_hub_design.md
│       ├── safety_firewall_design.md # 四层安全防火墙设计
│       └── skill_market_design.md
├── pyproject.toml                    # 项目配置
└── .github/workflows/ci.yml          # CI/CD 流程
```

### 3.2 Layer 1: ROS 2 Runtime - `UR5ROSNode`

**文件：** `src/rosclaw/mcp/ur5_server.py`

**职责：** 实际的 ROS 2 通信层，负责与 UR5e 机械臂的物理交互。

```mermaid
classDiagram
    class UR5ROSNode {
        +Node base
        +rclpy.Node ros_node
        +ActionClient trajectory_client
        +Subscriber joint_state_sub
        +Publisher script_pub
        +RobotState last_state
        +DigitalTwinFirewall firewall
        +get_joint_states() RobotState
        +move_joints(positions, duration) bool
        +execute_trajectory(waypoints) bool
        +emergency_stop() bool
        +get_limits() dict
        +validate_trajectory(trajectory) ValidationResult
        +run() void
    }
```

**关键特性：**
- 继承自 `rclpy.node.Node`
- 使用 `FollowJointTrajectory` Action 进行运动控制
- 订阅 `/joint_states` 获取实时反馈
- 内部集成 `DigitalTwinFirewall` 进行实时安全校验
- ROS 消息导入使用 try/except 兼容非 ROS 环境

### 3.3 Layer 2: Semantic-HAL - `RingBuffer` & `DataFlywheel`

**文件：** `src/rosclaw/data/ring_buffer.py` + `src/rosclaw/data/flywheel.py`

**职责：** 高频数据采集与事件驱动的数据持久化。

#### RingBuffer 类图

```mermaid
classDiagram
    class BufferFullStrategy {
        <<enumeration>>
        OVERWRITE
        EXPAND
        DROP
    }

    class RingBufferConfig {
        +int capacity
        +tuple shape
        +np.dtype dtype
        +BufferFullStrategy strategy
    }

    class RingBuffer {
        +int capacity
        +np.ndarray _buffer
        +np.ndarray _timestamps
        +int _head
        +int _size
        +bool _is_full
        +append(data, timestamp) void
        +get_last_n(n) tuple
        +get_range(start, end) tuple
        +get_all() tuple
        +latest() tuple
        +clear() void
    }

    class MultiChannelRingBuffer {
        +dict~str, RingBuffer~ channels
        +append(data, timestamp) void
        +get_last_n(n) dict
        +get_range(start, end) dict
        +clear() void
    }

    RingBufferConfig --> RingBuffer : configures
    MultiChannelRingBuffer o-- RingBuffer : manages
```

**设计原则：**
- 预分配内存，运行时无 GC 压力
- NumPy 向量化，O(1) 追加
- 环形缓冲区自动覆盖最旧数据
- 支持多通道同步捕获

#### DataFlywheel 类图

```mermaid
classDiagram
    class EventType {
        <<enumeration>>
        SUCCESS
        FAILURE
        EMERGENCY
        USER_MARK
        MILESTONE
    }

    class RobotState {
        +float timestamp
        +ndarray joint_positions
        +ndarray joint_velocities
        +ndarray joint_torques
        +ndarray end_effector_pose
        +float gripper_state
        +validate(expected_dof) bool
    }

    class DataEvent {
        +str event_id
        +EventType event_type
        +float timestamp
        +str robot_id
        +float pre_event_duration
        +float post_event_duration
        +dict metadata
        +dict data_paths
        +to_dict() dict
        +from_dict(data) DataEvent
    }

    class DataFlywheel {
        +str robot_id
        +int joint_dof
        +int sampling_rate_hz
        +MultiChannelRingBuffer _buffers
        +list DataEvent _events
        +threading.Lock _event_lock
        +Path _storage_path
        +on_control_cycle(state) void
        +trigger_event(type, metadata) str
        +export_to_lerobot(output_path) Path
        +get_stats() dict
        +clear() void
    }

    DataFlywheel o-- RingBuffer : manages
    DataFlywheel o-- DataEvent : tracks
    DataEvent --> EventType : has
    RobotState --> DataFlywheel : feeds
```

**核心创新：** 事件驱动持久化策略
- 持续在内存中循环缓存 60 秒数据（1kHz，无存储开销）
- 仅在有趣事件时提取数据并持久化
- 背景线程异步写入，不阻塞控制循环
- 100 倍存储优化（10GB/天 vs 1TB/天）

```mermaid
sequenceDiagram
    participant CL as Control Loop
    participant DB as DataFlywheel
    participant RB as RingBuffer
    participant ST as Storage Thread
    participant FS as File System

    loop 1kHz
        CL->>DB: on_control_cycle(state)
        DB->>RB: append O(1)
    end

    Note over DB: 事件触发 (e.g., 成功抓取)
    CL->>DB: trigger_event(SUCCESS)
    DB->>RB: get_last_n(pre_samples)
    DB->>ST: start_thread(save_event)
    ST->>FS: save *.npy + metadata.json
    DB-->>CL: return event_id
```

### 3.4 Layer 3: Digital Twin - `DigitalTwinFirewall`

**文件：** `src/rosclaw/firewall/decorator.py`

**职责：** 在执行前通过 MuJoCo 物理仿真验证机器人轨迹。

```mermaid
classDiagram
    class SafetyLevel {
        <<enumeration>>
        STRICT - 拒绝任何违规
        MODERATE - 允许轻微碰撞
        LENIENT - 仅警告
    }

    class ValidationResult {
        +bool is_safe
        +bool collision_detected
        +bool joint_limit_violated
        +bool torque_limit_exceeded
        +float max_predicted_torque
        +float min_distance_to_collision
        +int simulation_steps
        +list violation_details
        +to_dict() dict
    }

    class DigitalTwinFirewall {
        +str model_path
        +mujoco.MjModel model
        +mujoco.MjData data
        +list joint_names
        +dict joint_limits
        +dict torque_limits
        +float safety_margin
        +int sim_steps_per_check
        +reset() void
        +set_joint_positions(pos) void
        +set_joint_velocities(vel) void
        +apply_control(ctrl) void
        +step(n_steps) void
        +check_collision() tuple
        +check_joint_limits() tuple
        +check_torque_limits() tuple
        +validate_trajectory(traj) ValidationResult
        +decorator(safety_level) Callable
    }

    class SafetyViolationError {
        +str message
        +ValidationResult result
    }

    SafetyLevel --> DigitalTwinFirewall : configures
    DigitalTwinFirewall --> ValidationResult : returns
    SafetyViolationError --> ValidationResult : contains

    note for DigitalTwinFirewall "四层递进验证\n1. 硬限位检查\n2. MuJoCo 仿真\n3. 碰撞检测\n4. 力矩限制"
```

**验证流程：**

```mermaid
flowchart TD
    A["输入: trajectory<br/>joint positions 序列"] --> B["Pre-validate<br/>检查目标是否超限位"]
    B -->|超限| Z["返回 ValidationResult<br/>is_safe=False"]
    B -->|通过| C["reset 仿真"]
    C --> D["for each step:"]
    D --> E["apply_control<br/>设置控制信号"]
    E --> F["step 物理仿真<br/>sim_steps_per_check 步"]
    F --> G["check_collision"]
    G -->|碰撞| H{safety_level?}
    H -->|STRICT| Z
    H -->|MODERATE| I["check_joint_limits"]
    H -->|LENIENT| J["check_torque_limits"]
    I -->|超限| Z
    I -->|通过| J
    J -->|超限| Z
    J -->|通过| D
    D -->|所有步骤完成| K["分析结果"]
    K --> L["MODERATE:<br/>允许轻微碰撞"]
    K --> M["LENIENT:<br/>仅拒绝严重违规"]
    L --> N["返回 ValidationResult<br/>is_safe = 评估结果"]
    M --> N
```

**装饰器用法：**

```python
@DigitalTwinFirewall(
    model_path="src/rosclaw/specs/ur5e.xml",
    joint_limits=JOINT_LIMITS,
    torque_limits=TORQUE_LIMITS
).decorator(safety_level=SafetyLevel.STRICT)
def execute_motion(trajectory):
    return actual_robot_move(trajectory)  # 仅通过验证后执行
```

### 3.5 Layer 4: MCP Server - `UR5MCPServer`

**文件：** `src/rosclaw/mcp/ur5_server.py`

**职责：** 通过 MCP 协议暴露机器人控制工具，供 LLM 代理调用。

```mermaid
classDiagram
    class UR5MCPServer {
        +UR5ROSNode ros_node
        +DigitalTwinFirewall firewall
        +Server mcp
        +register_tools() void
        +move_robot(params) str
        +execute_trajectory(params) str
        +get_robot_state(params) str
        +emergency_stop(params) str
        +get_limits(params) str
        +validate_trajectory(params) str
        +run() async
    }

    class UR5ROSNode {
        <<rclpy.Node>>
        +RobotState last_state
        +DigitalTwinFirewall firewall
        +get_joint_states() RobotState
        +move_joints(pos, dur) bool
        +execute_trajectory(wps) bool
        +emergency_stop() bool
        +get_limits() dict
    }

    class RobotState {
        +list joint_positions
        +list joint_velocities
        +list joint_efforts
        +list joint_names
        +Pose end_effector_pose
        +bool is_connected
    }

    UR5MCPServer --> UR5ROSNode : uses
    UR5MCPServer --> RobotState : returns
    UR5ROSNode --> RobotState : provides
```

**MCP 工具列表（JSON-RPC 2.0）：**

| 工具名 | 参数 | 说明 |
|--------|------|------|
| `ur5_move_joints` | `joint_positions` (6x), `duration` | 移动到指定关节位置 |
| `ur5_execute_trajectory` | `waypoints` (Nx6), `times` | 执行多点位轨迹 |
| `ur5_get_joint_states` | 无 | 获取当前关节状态 |
| `ur5_emergency_stop` | 无 | 紧急停止 |
| `ur5_get_limits` | 无 | 获取机器人限制参数 |
| `ur5_validate_trajectory` | `trajectory` (Nx6) | 安全验证（不执行） |

### 3.6 MuJoCo 机器人模型

**文件：** `src/rosclaw/specs/ur5e.xml`

```mermaid
graph TB
    subgraph UR5e["UR5e Robot Structure"]
        Base["base<br/>cylinder Ø0.15 × 0.076"]
        SP["shoulder_pan_joint<br/>hinge ±2π"]
        SL["shoulder_lift_joint<br/>hinge ±2π"]
        EJ["elbow_joint<br/>hinge ±π"]
        W1["wrist_1_joint<br/>hinge ±2π"]
        W2["wrist_2_joint<br/>hinge ±2π"]
        W3["wrist_3_joint<br/>hinge ±2π"]
        TCP["tool0 / tcp"]
        
        Base --> SP --> SL --> EJ --> W1 --> W2 --> W3 --> TCP
    end

    subgraph Actuators["Actuators"]
        M1["shoulder_pan_motor<br/>±150 Nm"]
        M2["shoulder_lift_motor<br/>±150 Nm"]
        M3["elbow_motor<br/>±100 Nm"]
        M4["wrist_1_motor<br/>±28 Nm"]
        M5["wrist_2_motor<br/>±28 Nm"]
        M6["wrist_3_motor<br/>±28 Nm"]
    end

    subgraph Sensors["Sensors"]
        S1["jointpos × 6"]
        S2["tcp_force"]
        S3["tcp_torque"]
    end
```

## 4. 关键数据流

### 4.1 工具调用流程（从 LLM 到机器人）

```mermaid
sequenceDiagram
    participant LLM as LLM Agent
    participant MC as mcporter Bridge
    participant MCP as UR5MCPServer
    participant FW as DigitalTwinFirewall
    participant ROS as UR5ROSNode
    participant ROBOT as UR5e Hardware

    LLM->>MC: LLM 决策调用 move_robot
    MC->>MCP: JSON-RPC {"method":"tools/call","name":"ur5_move_joints"}
    MCP->>MCP: 参数验证
    MCP->>FW: validate_trajectory(trajectory)
    FW->>FW: MuJoCo 仿真模拟
    FW-->>MCP: ValidationResult(is_safe=true/false)
    
    alt is_safe == false
        MCP-->>MC: SafetyViolationError(违规详情)
        MC-->>LLM: 拒绝执行 + 违规原因
    else is_safe == true
        MCP->>ROS: move_joints(positions, duration)
        ROS->>ROBOT: FollowJointTrajectory Action
        ROBOT->>ROS: 执行反馈
        ROS-->>MCP: 执行结果
        MCP-->>MC: {"success": true, "actual_positions": [...]}
        MC-->>LLM: 执行结果
    end
```

### 4.2 数据捕获流程

```mermaid
sequenceDiagram
    participant CL as Control Loop 1kHz
    participant DF as DataFlywheel
    participant RB as RingBuffer (60s)
    participant E as Event Trigger
    participant ST as Storage Thread
    participant FS as Local SSD

    loop 1kHz 持续捕获
        CL->>DF: on_control_cycle(state)
        DF->>RB: append O(1) 无存储
    end

    Note over E: 事件触发<br/>(SUCCESS/FAILURE/EMERGENCY)
    E->>DF: trigger_event(type, metadata)
    DF->>RB: get_last_n(pre_samples)
    DF->>ST: 后台线程保存
    ST->>FS: *.npy + metadata.json
    RB->>RB: 自动覆盖最旧数据
    DF-->>E: event_id
```

### 4.3 MCP Server 启动流程

```mermaid
sequenceDiagram
    participant CLI as 命令行入口
    participant MAIN as main()
    participant ROS as rclpy.init()
    participant NODE as UR5ROSNode
    participant MCPS as UR5MCPServer
    participant STDIO as MCP stdio transport

    CLI->>MAIN: rosclaw-ur5-mcp
    MAIN->>ROS: rclpy.init()
    MAIN->>NODE: UR5ROSNode(robot_ip, DT_enabled)
    activate NODE
    NODE->>ROS: 创建 Publisher/Subscriber/ActionClient
    MAIN->>MCPS: UR5MCPServer(node)
    MCPS->>MCPS: register_tools() 注册 6 个 MCP 工具
    MCPS->>STDIO: stdio_server()
    STDIO->>STDIO: await run() 阻塞等待 MCP 请求
    Note over STDIO: 通过 stdio 接收 JSON-RPC 2.0 消息
```

### 4.4 装饰器安全验证流程

```mermaid
flowchart TD
    A["调用被 @mujoco_firewall 装饰的函数"] --> B["装饰器包装器拦截调用"]
    B --> C["提取 trajectory<br/>从 args/kwargs"]
    C --> D["创建 DigitalTwinFirewall"]
    D --> E["firewall.validate_trajectory()"]
    E --> F["MuJoCo 仿真"]
    F --> G["检查结果"]
    G -->|is_safe=true| H["执行原函数"]
    G -->|is_safe=false| I["抛出 SafetyViolationError"]
    H --> J["返回原函数结果"]
    I --> K["携带 ValidationResult"]

    style A fill:#e3f2fd
    style F fill:#fff3e0
    style H fill:#e8f5e9
    style I fill:#ffebee
```

## 5. CI/CD 流水线

```mermaid
flowchart LR
    PR["Pull Request<br/>→ main/develop"] --> LINT["Lint & Format<br/>ruff check + format"]
    PR --> TYPE["Type Check<br/>mypy"]
    LINT --> TEST["Test<br/>pytest + coverage<br/>Python 3.10/3.11/3.12"]
    TYPE --> TEST
    TEST --> INTTEST["Integration Test<br/>scripts/integration_test.py"]
    INTTEST --> BUILD["Build<br/>hatch build"]
    BUILD -->|成功| PUBLISH{"Tag v*?"}
    PUBLISH -->|是| PYPI["Publish to PyPI"]
    PUBLISH -->|否| END["完成"]
    PYPI --> END
```

**CI 流水线步骤：**
1. **Lint & Format**: ruff check + format check
2. **Type Check**: mypy strict mode
3. **Test**: pytest + coverage (多 Python 版本矩阵)
4. **Integration Test**: 端到端集成测试
5. **Build**: hatch 打包
6. **Release**: 匹配 `refs/tags/v*` 时发布到 PyPI

## 6. 配置与依赖

### 6.1 依赖关系

```mermaid
graph LR
    CORE["核心依赖<br/>numpy ≥1.24<br/>mujoco ≥3.0<br/>mcp ≥1.0"]
    ROS["ROS 2 可选依赖<br/>rclpy ≥3.0<br/>geometry-msgs<br/>sensor-msgs<br/>std-msgs<br/>trajectory-msgs<br/>control-msgs"]
    DEV["开发依赖<br/>pytest ≥7.0<br/>pytest-asyncio ≥0.21<br/>ruff ≥0.1<br/>mypy ≥1.5"]
    
    CORE --> PYPI["发布到 PyPI"]
    ROS --> PYPI
    DEV -.-> LOCAL["本地开发"]
```

### 6.2 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROBOT_IP` | `192.168.1.100` | UR5 机械臂 IP |
| `ROBOT_PORT` | `50002` | RTDE 端口 |
| `DIGITAL_TWIN_ENABLED` | `true` | 是否启用数字孪生 |
| `MUJOCO_MODEL_PATH` | `src/rosclaw/specs/ur5e.xml` | MuJoCo 模型路径 |
| `SAFETY_LEVEL` | `strict` | 安全验证级别 |

### 6.3 关键常量 - UR5e 规格

| 关节 | 位置限制 (rad) | 速度限制 (rad/s) | 力矩限制 (Nm) |
|------|----------------|-------------------|----------------|
| shoulder_pan | ±2π (±6.28) | 3.15 | 150 |
| shoulder_lift | ±2π (±6.28) | 3.15 | 150 |
| elbow | ±π (±3.14) | 3.15 | 100 |
| wrist_1 | ±2π (±6.28) | 6.28 | 28 |
| wrist_2 | ±2π (±6.28) | 6.28 | 28 |
| wrist_3 | ±2π (±6.28) | 6.28 | 28 |

## 7. 安全机制详解

### 7.1 DigitalTwinFirewall 验证层次

```mermaid
graph TB
    subgraph L1["Layer 1: 硬限制<br/>~0.1ms"]
        JPL["关节位置限位"]
        JVL["关节速度限位"]
        TQL["力矩限位"]
    end

    subgraph L2["Layer 2: 解析验证<br/>~1ms (设计文档中)"]
        KIN["运动学验证"]
        DYN["动力学验证"]
    end

    subgraph L3["Layer 3: MuJoCo 仿真<br/>~10ms"]
        COLL["碰撞检测"]
        LIMIT["限位检查"]
        TORQUE["力矩超限检测"]
    end

    subgraph L4["Layer 4: 预测性安全<br/>设计文档中 (未实现)"]
        NN["神经网络世界模型"]
    end

    cmd["输入: 机器人命令"] --> L1
    L1 -->|"通过"| L2
    L2 -->|"通过"| L3
    L3 -->|"通过"| SAFE["✅ 批准执行"]
    L1 -->|"违规"| REJECT["❌ 拒绝"]
    L2 -->|"违规"| REJECT
    L3 -->|"违规"| REJECT

    style L1 fill:#e8f5e9
    style L2 fill:#fff3e0
    style L3 fill:#ffebee
    style L4 fill:#fce4ec
    style SAFE fill:#c8e6c9
    style REJECT fill:#ffcdd2
```

### 7.2 SafetyLevel 策略对比

| 级别 | 碰撞 | 限位违规 | 力矩超限 | 适用场景 |
|------|------|----------|----------|----------|
| `STRICT` | 拒绝 | 拒绝 | 拒绝 | 生产环境/人类共存 |
| `MODERATE` | 允许轻微 | 拒绝 | 拒绝 | 仿真/调试 |
| `LENIENT` | 允许 | 允许 | 仅严重拒绝 | 离线仿真测试 |

## 8. 测试策略

### 8.1 测试分层

```mermaid
flowchart LR
    subgraph Unit["单元测试 (pytest)"]
        T1["test_data_layer.py<br/>RingBuffer + DataFlywheel"]
        T2["test_firewall.py<br/>DigitalTwinFirewall + 装饰器"]
        T3["test_mcp_server.py<br/>MCP 结构 (mock rclpy)"]
    end

    subgraph Integration["集成测试"]
        T4["integration_test.py<br/>端到端验证"]
    end

    T1 --> CI["CI/CD 流水线"]
    T2 --> CI
    T3 --> CI
    T4 --> CI

    style T1 fill:#e3f2fd
    style T2 fill:#e8f5e9
    style T3 fill:#fff3e0
    style T4 fill:#fce4ec
```

### 8.2 测试关键模式

```python
# 测试防火墙 - 直接使用真实 MuJoCo 模型
MODEL_PATH = Path(__file__).parent.parent / "src" / "rosclaw" / "specs" / "ur5e.xml"

@pytest.fixture
def firewall():
    return DigitalTwinFirewall(
        model_path=str(MODEL_PATH),
        joint_limits=JOINT_LIMITS,
        sim_steps_per_check=10,  # 加速测试
    )

# 测试 MCP - 在导入前 mock 所有 ROS 模块
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
# ... 更多 mock
from rosclaw.mcp.ur5_server import UR5ROSNode
```

## 9. 关键技术决策

### 9.1 为什么选择装饰器模式进行安全验证？

- **透明集成**：无需修改业务逻辑，一行装饰器即可
- **灵活级别**：不同函数可使用不同 `SafetyLevel`
- **明确异常**：`SafetyViolationError` 携带完整验证结果
- **可组合**：可与 `@functools.wraps` 等装饰器链式使用

### 9.2 为什么使用环形缓冲区？

- **确定性延迟**：O(1) 操作，无动态内存分配
- **零 GC 压力**：预分配 NumPy 数组
- **自动覆写**：满时自动覆盖最旧数据
- **高效提取**：`get_last_n()` 支持跨环边界的合并复制

### 9.3 为什么用 MuJoCo 而非 Gazebo/PyBullet？

- **性能**：MuJoCo 的隐式积分器更高效
- **集成**：`mujoco` Python 包 API 简洁
- **仿真精度**：更适合精细机械臂仿真

## 10. 当前实现 vs 设计文档规划

| 特性 | 当前实现 | 设计文档 (规划) |
|------|----------|-----------------|
| 硬限制检查 | ✅ `DigitalTwinFirewall` | Layer 1: `HardLimitChecker` |
| MuJoCo 仿真 | ✅ `DigitalTwinFirewall` | Layer 3: `MJXFirewall` (GPU 并行) |
| 解析验证 | ❌ 未实现 | Layer 2: `Pinocchio + Ruckig` |
| 预测性安全 | ❌ 未实现 | Layer 4: `Neural Twin` |
| 多通道缓冲区 | ✅ `MultiChannelRingBuffer` | ✅ 已实现 |
| LeRobot 导出 | ✅ `export_to_lerobot()` | ✅ 已实现 (简化版) |
| MCP 工具 | ✅ 6 个 `ur5_*` 工具 | ✅ 已实现 |
| 安全编排器 | ❌ 未实现 | `SafetyFirewallOrchestrator` |

## 11. 文件速查表

| 文件 | 核心类/函数 | 行数 |
|------|------------|------|
| `src/rosclaw/mcp/ur5_server.py` | `UR5ROSNode`, `UR5MCPServer`, `RobotState` | ~500 |
| `src/rosclaw/firewall/decorator.py` | `DigitalTwinFirewall`, `mujoco_firewall` | ~430 |
| `src/rosclaw/data/flywheel.py` | `DataFlywheel`, `RobotState`, `DataEvent` | ~410 |
| `src/rosclaw/data/ring_buffer.py` | `RingBuffer`, `MultiChannelRingBuffer` | ~300 |
| `tests/test_firewall.py` | 防火墙测试类 | ~360 |
| `tests/test_data_layer.py` | 数据层测试类 | ~320 |
| `tests/test_mcp_server.py` | MCP 结构测试 (mock) | ~370 |
| `scripts/integration_test.py` | 端到端集成测试 | ~270 |
| `specs/ur5e.xml` | MuJoCo UR5e 模型 | ~128 |
