# ROSClaw 项目详细分析报告

## 1. 项目概述

ROSClaw 是一个 Python 包，提供生产级中间件，通过 ROS 2 和模型上下文协议（MCP）将 LLM 连接到物理机器人（UR5e 机械臂）。它实现了四层递进式安全架构，带有基于 MuJoCo 的数字孪生防火墙。

**项目定位：** 连接 LLM 与物理机器人的桥梁——让 AI 代理安全地控制真实机器人。

**项目状态：** Pre-Alpha (V0.1) — 结构性概念验证。

---

## 2. 系统总体架构

### 2.1 系统总体架构（从上到下分层）

下图展示整个系统的分层架构和核心职责。AI 代理发出自然语言指令，经过四层处理，最终安全驱动物理机器人执行：

```mermaid
graph TB
    subgraph L6["第六层：AI 代理层"]
        direction LR
        AG["Claude Code / OpenClaw<br/>AI 代理（自然语言交互）"]
        AGEN["Agent Event Loop<br/>调度工具调用、管理上下文"]
        AG --> AGEN
    end

    subgraph L5["第五层：MCP 协议网关"]
        direction LR
        MCP_GW["MCP 传输层<br/>stdio 进程间通信<br/>JSON-RPC 2.0 协议"]
        TOOL_REG["工具注册中心<br/>6 个标准机器人工具"]
        AGEN -->|"调用工具<br/>如 '移动到 home 位置'"| MCP_GW
        MCP_GW --> TOOL_REG
    end

    subgraph L4["第四层：安全防火墙<br/><i>核心创新</i>"]
        direction LR
        DTW["Digital Twin<br/>数字孪生引擎"]
        MUJOCO["MuJoCo 物理仿真<br/>碰撞检测 · 关节限位 · 力矩限制"]
        TOOL_REG -->|"每条指令先验证<br/>再执行"| DTW
        DTW --> MUJOCO
        MUJOCO -->|"通过 → 可执行"| L3_IN
        MUJOCO -->|"违规 → 拒绝"| REJECT["返回错误<br/>告知代理原因"]
    end

    subgraph L3_IN[" "]
    end

    subgraph L2["第二层：数据飞轮<br/><i>核心创新</i>"]
        direction LR
        RB["环形缓冲区<br/>60秒滚动缓存<br/>1kHz 高频采集"]
        DF["DataFlywheel<br/>事件驱动<br/>自动归档"]
        RB --> DF
    end

    subgraph L1["第一层：ROS 2 运行时"]
        direction LR
        ROS_NODE["UR5ROSNode<br/>rclpy 节点"]
        ACTION_CLIENT["FollowJointTrajectory Action<br/>发送给 UR 控制器"]
        ROS_NODE --> ACTION_CLIENT
    end

    subgraph PH["物理层：UR5e 机械臂"]
        DIR["机械臂运动执行"]
    end

    DF --> ROS_NODE
    ACTION_CLIENT --> DIR

    style L6 fill:#e3f2fd,stroke:#1565c0
    style L5 fill:#f3e5f5,stroke:#7b1fa2
    style L4 fill:#ffebee,stroke:#c62828
    style L2 fill:#e8f5e9,stroke:#2e7d32
    style L1 fill:#fff3e0,stroke:#e65100
    style PH fill:#eceff1,stroke:#37474f
    style DTW fill:#ffcdd2,stroke:#b71c1c
    style MUJOCO fill:#ffcdd2,stroke:#b71c1c
    style RB fill:#c8e6c9,stroke:#2e7d32
    style DF fill:#a5d6a7,stroke:#1b5e20
    style ACTION_CLIENT fill:#ffe0b2,stroke:#e65100
    style REJECT fill:#ffcdd2,color:#b71c1c
```

**分层说明：**

| 层级 | 核心职责 | 解决的问题 |
|------|------|------|
| **L6: AI 代理** | 用自然语言理解用户意图 | 人类/代理无需懂机器人编程 |
| **L5: MCP 网关** | 标准化接口，对接任意 AI 框架 | 不绑定单一 Agent，plug-and-play |
| **L4: 安全防火墙** | 每条指令先在数字孪生中仿真验证 | 防止 LLM 幻觉导致物理事故 |
| **L2: 数据飞轮** | 持续采集机器人数据，事件触发自动归档 | 为后续 VLA 模型训练积累数据 |
| **L1: ROS 2 运行时** | 与真实机器人硬件通信 | 抽象底层 DDS 协议，统一控制接口 |
| **物理层** | 实际执行运动 | UR5e 机械臂 |

### 2.2 分层详解

| 层级 | 模块 | 职责 | 延迟目标 | 关键技术 |
|------|------|------|----------|----------|
| L6 | OpenClaw / Claude Code | AI 代理，自然语言交互 | - | Agent Event Loop |
| L5 | MCP 协议网关 | 标准化接口，对接任意 Agent | - | stdio + JSON-RPC 2.0 |
| L4 | `DigitalTwinFirewall` | 每条指令先在数字孪生中仿真验证 | < 10ms | MuJoCo 物理仿真 |
| L2 | `DataFlywheel` + `RingBuffer` | 持续采集 + 事件驱动归档 | < 1ms | NumPy 预分配 RingBuffer |
| L1 | `UR5ROSNode` | 与真实机器人硬件通信 | - | rclpy/DDS |

### 2.3 模块关系图

```mermaid
graph TB
    subgraph "入口层"
        ENTRY["rosclaw-ur5-mcp<br/>pyproject.toml entry point"]
    end

    subgraph "MCP 服务层"
        MCPSRV["UR5MCPServer<br/>_register_tools()"]
        TOOLS["6 MCP Tools:<br/>ur5_move_joints<br/>ur5_execute_trajectory<br/>ur5_get_joint_states<br/>ur5_emergency_stop<br/>ur5_get_limits<br/>ur5_validate_trajectory"]
    end

    subgraph "安全验证层"
        FW["DigitalTwinFirewall<br/>validate_trajectory()"]
        MJCOCORE["mujoco.MjModel + MjData"]
        UR5XML["specs/ur5e.xml<br/>MuJoCo MJCF 模型"]
        FW --> MJCOCORE
        MJCOCORE --> UR5XML
    end

    subgraph "ROS 2 层"
        ROSNODE["UR5ROSNode<br/>rclpy.Node"]
        JOINTSUB["Subscriber: /joint_states"]
        TRAJPUB["Publisher: /joint_trajectory_controller/command"]
        ACTION["ActionClient: FollowJointTrajectory"]
        ROSNODE --> JOINTSUB
        ROSNODE --> TRAJPUB
        ROSNODE --> ACTION
    end

    subgraph "数据捕获层"
        RING["RingBuffer<br/>O(1) 预分配"]
        MC["MultiChannelRingBuffer<br/>多通道同步"]
        FLY["DataFlywheel<br/>事件触发持久化"]
        FLY --> RING
        RING --> MC
    end

    ENTRY --> MCPSRV
    MCPSRV --> TOOLS
    TOOLS --> FW
    TOOLS --> ROSNODE
    FW --> ROSNODE
    ROSNODE --> RING
    ROSNODE --> FLY
    FLY -->|"export_to_lerobot()"| OUTPUT["LeRobot Dataset<br/>.npy + metadata.json"]

    style ENTRY fill:#fff9c4
    style MCPSRV fill:#ce93d8,color:#000
    style FW fill:#ef5350,color:#fff
    style ROSNODE fill:#66bb6a,color:#fff
    style FLY fill:#42a5f5,color:#fff
    style OUTPUT fill:#e0e0e0
```

---

## 3. 核心模块详细分析

### 3.1 Layer 4: Embodiment MCP (`src/rosclaw/mcp/ur5_server.py`)

**职责：** 通过 MCP 协议暴露机器人控制工具，供 LLM 代理调用。

**核心类：**

| 类名 | 行数 | 职责 |
|------|------|------|
| `UR5MCPServer` | ~350 | MCP 服务器，6 个 `ur5_*` 工具注册和路由 |
| `UR5ROSNode` | ~260 | rclpy 节点，关节状态订阅/发布，轨迹 Action Client |
| `RobotState` | ~10 | 数据类，封装关节位姿/速度/力矩状态 |

**MCP 工具列表：**

```mermaid
graph LR
    subgraph "控制类工具"
        T1["ur5_move_joints<br/>单点运动"]
        T2["ur5_execute_trajectory<br/>多点轨迹"]
        T6["ur5_validate_trajectory<br/>纯验证不执行"]
    end

    subgraph "状态类工具"
        T3["ur5_get_joint_states<br/>当前关节状态"]
        T5["ur5_get_limits<br/>限制参数"]
    end

    subgraph "安全类工具"
        T4["ur5_emergency_stop<br/>紧急停止"]
    end

    style T1 fill:#e3f2fd
    style T2 fill:#e3f2fd
    style T6 fill:#fff3e0
    style T3 fill:#e8f5e9
    style T5 fill:#e8f5e9
    style T4 fill:#ffebee,color:#000
```

**关键实现要点：**
- MCP Server 使用 `stdio_server()` 传输（Claude Code 默认）
- 每个工具处理函数通过 `_handle_<tool_name>()` 方法路由
- 运动工具默认启用 `validate=True`，走 Digital Twin 验证
- `rclpy` 导入使用 try/except 兼容非 ROS 环境

### 3.2 Layer 3: Digital Twin Firewall (`src/rosclaw/firewall/decorator.py`)

**职责：** 执行前通过 MuJoCo 物理仿真验证机器人轨迹。

**核心类：**

| 类/枚举 | 职责 |
|---------|------|
| `DigitalTwinFirewall` | MuJoCo 仿真引擎，加载 UR5e 模型，执行碰撞/限位/力矩检测 |
| `SafetyLevel` | 枚举：`STRICT` / `MODERATE` / `LENIENT` |
| `ValidationResult` | frozen dataclass，封装验证结果 |
| `SafetyViolationError` | 携带 ValidationResult 的异常 |
| `mujoco_firewall()` | 快捷装饰器工厂 |

**验证流程图：**

```mermaid
flowchart TD
    A["输入: 关节位置序列 trajectory"] --> B["Pre-validate<br/>检查目标是否超限位"]

    B -->|"超限"| Z["返回 ValidationResult<br/>is_safe=False"]

    B -->|"通过"| C["reset 仿真状态"]

    C --> D["for each 轨迹点:"]
    D --> E["apply_control<br/>设置控制信号"]
    E --> F["step 物理仿真<br/>sim_steps_per_check 步"]
    F --> G{"碰撞检测"}

    G -->|"有碰撞"| H{"SafetyLevel?"}
    H -->|"STRICT"| Z
    H -->|"MODERATE"| I["继续 - 允许轻微碰撞"]
    H -->|"LENIENT"| I

    I --> J{"关节限位检查"}
    J -->|"超限"| Z
    J -->|"通过"| K{"力矩超限检查"}
    K -->|"超限"| Z
    K -->|"通过"| D

    D -->|"所有点完成"| L["分析最终结果"]

    L --> M{"SafetyLevel?"}
    M -->|"STRICT"| N["is_safe = 全部通过"]
    M -->|"MODERATE"| O["is_safe = 排除轻微碰撞"]
    M -->|"LENIENT"| P["is_safe = 仅拒绝严重力矩违规"]

    N --> R["返回 ValidationResult"]
    O --> R
    P --> R
    Z --> R

    style A fill:#e3f2fd
    style Z fill:#ffebee,color:#000
    style R fill:#e8f5e9
    style D fill:#fff3e0
```

**装饰器用法：**

```python
# 方式 1: 装饰器
@mujoco_firewall(model_path="specs/ur5e.xml", safety_level=SafetyLevel.STRICT)
def execute_motion(trajectory):
    return robot.move(trajectory)  # 仅验证通过后执行

# 方式 2: 直接调用
firewall = DigitalTwinFirewall("specs/ur5e.xml")
result = firewall.validate_trajectory(trajectory)
if result.is_safe:
    robot.move(trajectory)
```

### 3.3 Layer 2: Data Layer (`src/rosclaw/data/`)

#### RingBuffer - 高性能环形缓冲区

**职责：** 1kHz 高频数据采集，预分配内存，O(1) 追加，零 GC 压力。

```mermaid
classDiagram
    class BufferFullStrategy {
        <<enumeration>>
        OVERWRITE 覆盖最旧
        EXPAND 扩展缓冲区
        DROP 丢弃新数据
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
        +append data, timestamp void
        +get_last_n n tuple
        +get_range start, end tuple
        +get_all tuple
        +latest tuple
        +clear void
    }

    class MultiChannelRingBuffer {
        +dict channels
        +append data dict, timestamp void
        +get_last_n n dict
        +get_range start, end dict
        +clear void
        +size int
    }

    RingBufferConfig --> RingBuffer : configures
    MultiChannelRingBuffer o-- RingBuffer : manages
```

**设计原则：**
- 预分配 NumPy 数组，运行时无 GC 压力
- O(1) 追加操作（环形指针轮换）
- `get_last_n()` 自动处理跨环边界的合并复制
- `MultiChannelRingBuffer` 支持多通道同步捕获（joint/velocity/torque/camera）

#### DataFlywheel - 事件驱动数据捕获

**职责：** 事件触发的高频数据持久化，实现 100 倍存储优化。

```mermaid
sequenceDiagram
    participant CL as 控制循环 1kHz
    participant DF as DataFlywheel
    participant RB as RingBuffer (60s)
    participant EVT as 事件触发
    participant ST as 后台存储线程
    participant FS as 本地磁盘

    loop 持续捕获
        CL->>DF: on_control_cycle(state)
        DF->>RB: append O(1) 无存储
    end

    Note over EVT: 事件触发 (成功/失败/紧急)
    EVT->>DF: trigger_event(type, metadata)
    DF->>RB: get_last_n(pre_samples)
    DF->>ST: start_thread(save_event)
    ST->>FS: save *.npy + metadata.json
    RB->>RB: 自动覆盖最旧数据
    DF-->>EVT: return event_id
```

**数据流架构：**

```mermaid
graph TB
    subgraph "数据采集层"
        CTRL["控制循环 1kHz"]
        STATE["RobotState<br/>position/velocity/torque"]
        CTRL -->|每周期| STATE
    end

    subgraph "内存缓存层"
        RING["MultiChannelRingBuffer<br/>60s 滚动窗口 @ 1kHz"]
        STATE --> RING
        RING -.->|"自动覆盖最旧数据"| RING2[RING (满)]
    end

    subgraph "事件触发层"
        EVT["DataFlywheel.trigger_event()"]
        EVT_TYPES["SUCCESS / FAILURE / EMERGENCY / USER_MARK / MILESTONE"]
        EVT --> EVT_TYPES
    end

    subgraph "持久化层"
        DIR["{robot_id}_{event_id}/<br/>joint_positions.npy<br/>joint_velocities.npy<br/>joint_torques.npy<br/>metadata.json"]
        LE["export_to_lerobot()<br/>LeRobot Dataset 格式"]
        RING2 -->|"get_last_n"| DIR
        DIR --> LE
    end

    CTRL -.->|1kHz 驱动| EVT
    style CTRL fill:#e3f2fd
    style RING fill:#42a5f5,color:#000
    style EVT fill:#fff3e0
    style DIR fill:#e8f5e9
    style LE fill:#fce4ec
```

### 3.4 Layer 1: ROS 2 Runtime (`src/rosclaw/mcp/ur5_server.py`)

**职责：** 实际的 ROS 2 通信层，负责与 UR5e 机械臂的物理交互。

```mermaid
classDiagram
    class UR5ROSNode {
        +str robot_ip
        +str namespace
        +RobotState state
        +ReentrantCallbackGroup callback_group
        +rclpy.Subscriber joint_state_sub
        +rclpy.Publisher joint_trajectory_pub
        +rclpy.Publisher velocity_pub
        +ActionClient trajectory_client
        +get_current_joint_positions list
        +validate_joint_limits tuple
        +execute_joint_trajectory async
        +emergency_stop void
    }

    class UR5MCPServer {
        +str robot_ip
        +DigitalTwinFirewall firewall
        +Server mcp
        +_register_tools
        +_handle_move_joints async
        +_handle_execute_trajectory async
        +run async
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
    UR5ROSNode --> RobotState : provides
```

**ROS 2 通信拓扑：**

```mermaid
graph LR
    subgraph "话题订阅"
        SUB["/joint_states<br/>JointState 消息"]
        SUB -->|"每~33ms"| NODE["UR5ROSNode<br/>_joint_state_callback"]
        NODE --> STATE["RobotState<br/>last_state 缓存"]
    end

    subgraph "话题发布"
        NODE --> TRAJ["/joint_trajectory_controller/command<br/>JointTrajectory 消息"]
        NODE --> VEL["/cmd_vel<br/>Twist 消息 (紧急停止)"]
    end

    subgraph "ActionClient"
        NODE --> ACTION["/follow_joint_trajectory<br/>FollowJointTrajectory"]
        ACTION -->|"Goal → Result"| RESULT["(success/error_code)"]
    end

    style SUB fill:#e3f2fd
    style NODE fill:#66bb6a,color:#fff
    style STATE fill:#fff3e0
    style TRAJ fill:#ffebee
    style VEL fill:#fce4ec
    style ACTION fill:#f3e5f5
```

### 3.5 MuJoCo 机器人模型 (`src/rosclaw/specs/ur5e.xml`)

```mermaid
graph TB
    subgraph UR5e["UR5e 机械臂结构"]
        BASE["base<br/>cylinder Ø0.15 x 0.076"]
        J1["shoulder_pan_joint<br/>hinge ±2π<br/>150 Nm"]
        J2["shoulder_lift_joint<br/>hinge ±2π<br/>150 Nm"]
        J3["elbow_joint<br/>hinge ±π<br/>100 Nm"]
        J4["wrist_1_joint<br/>hinge ±2π<br/>28 Nm"]
        J5["wrist_2_joint<br/>hinge ±2π<br/>28 Nm"]
        J6["wrist_3_joint<br/>hinge ±2π<br/>28 Nm"]
        TCP["tool0 / tcp<br/>末端执行器"]

        BASE --> J1 --> J2 --> J3 --> J4 --> J5 --> J6 --> TCP
    end

    subgraph "传感器"
        S1["jointpos x 6"]
        S2["tcp_force"]
        S3["tcp_torque"]
    end

    subgraph "UR5ROSNode 常量"
        JL["JOINT_LIMITS<br/>6 DOF 限位字典"]
        TL["TORQUE_LIMITS<br/>80% 安全余量"]
    end

    JL -.-> J1
    JL -.-> J3
    TL -.-> J3
    S1 --> NODE["ROS joint_states 话题"]
    S2 --> NODE
    S3 --> NODE

    style UR5e fill:#e8f5e9
    style sensor fill:#e3f2fd
    style JL fill:#fff3e0
```

---

## 4. 数据流与交互流程

### 4.1 工具调用全流程（LLM → 机器人）

```mermaid
sequenceDiagram
    participant LLM as LLM Agent
    participant MCP as UR5MCPServer
    participant FW as DigitalTwinFirewall
    participant ROS as UR5ROSNode
    participant ROBOT as UR5e Hardware

    LLM->>MCP: MCP Tool Call<br/>ur5_move_joints(poses, dur=2.0)
    MCP->>MCP: 参数验证 (6 个位置)
    MCP->>ROS: validate_joint_limits(poses)
    ROS-->>MCP: (True, "")

    MCP->>FW: validate_trajectory(interpolated)
    FW->>FW: MuJoCo reset
    FW->>FW: 逐点仿真 sim_steps_per_check 步
    FW->>FW: 碰撞/限位/力矩检测
    FW-->>MCP: ValidationResult(is_safe=True/False)

    alt is_safe == False
        MCP-->>LLM: "Digital Twin validation FAILED<br/>collision/joint/torque details"
    else is_safe == True
        MCP->>ROS: execute_joint_trajectory([poses], [dur])
        ROS->>ROBOT: FollowJointTrajectory Action Goal
        ROBOT->>ROS: 执行反馈
        ROS-->>MCP: (True, "Motion executed")
        MCP-->>LLM: {"success": true, "message": "Motion executed"}
    end
```

### 4.2 MCP Server 启动流程

```mermaid
sequenceDiagram
    participant CLI as CLI: rosclaw-ur5-mcp
    participant MAIN as main()
    participant RCL as rclpy.init()
    participant NODE as UR5ROSNode
    participant MCPS as UR5MCPServer
    participant STDIO as MCP stdio transport

    CLI->>MAIN: 启动入口
    MAIN->>RCL: rclpy.init(args=None)
    MAIN->>NODE: UR5ROSNode(robot_ip)
    activate NODE
    NODE->>RCL: create_subscription(joint_states)
    NODE->>RCL: create_publisher(joint_trajectory)
    NODE->>RCL: ActionClient(FollowJointTrajectory)
    NODE-->>MAIN: Node ready
    deactivate NODE

    MAIN->>MCPS: UR5MCPServer(robot_ip, model_path)
    MCPS->>MCPS: DigitalTwinFirewall(model_path)
    MCPS->>MCPS: _register_tools() (6 tools)
    MAIN->>STDIO: stdio_server(mcp)
    STDIO->>STDIO: await run() (阻塞)
```

### 4.3 装饰器安全验证流程

```mermaid
flowchart TD
    A["调用 @mujoco_firewall 装饰函数"] --> B["装饰器 wrapper 拦截参数"]
    B --> C["提取 trajectory<br/>从 args/kwargs"]
    C --> D["创建 DigitalTwinFirewall"]
    D --> E["firewall.validate_trajectory()"]
    E --> F["MuJoCo 仿真<br/>逐点模拟 + 碰撞/限位/力矩检测"]
    F --> G{is_safe?}
    G -->|True| H["执行原函数"]
    G -->|False| I["抛出 SafetyViolationError<br/>携带 ValidationResult"]
    H --> J["返回原函数结果"]
    I --> K["异常处理: 返回错误信息"]

    style A fill:#e3f2fd
    style F fill:#fff3e0
    style H fill:#e8f5e9
    style I fill:#ffebee,color:#000
```

### 4.4 数据捕获流程

```mermaid
flowchart TB
    subgraph "持续捕获 (1kHz)"
        CTRL["控制循环"] -->|"每周期"| STATE["RobotState"]
        STATE -->|"append"| RB["RingBuffer<br/>60s 滚动窗口"]
        RB -->|"满时覆盖最旧"| RB
    end

    subgraph "事件触发 (异步)"
        EVENT["trigger_event()"] -->|"提取"| DATA["get_last_n(pre_samples)"]
        DATA -->|"写入"| FILE["{robot_id}_{event_id}/<br/>*.npy + metadata.json"]
    end

    subgraph "数据导出"
        EVENT -->|"收集"| EVENTS["self._events list"]
        EVENTS -->|"export_to_lerobot()"| LEROBOT["LeRobot Dataset<br/>dataset.json + episodes"]
    end

    CTRL -.->|触发条件| EVENT
    style RB fill:#42a5f5,color:#000
    style EVENT fill:#fff3e0
    style LEROBOT fill:#e8f5e9
```

---

## 5. 模块依赖关系

```mermaid
graph LR
    subgraph "外部依赖"
        NP["numpy"]
        MU["mujoco"]
        MCP["mcp (MCP SDK)"]
        RCL["rclpy (可选)"]
    end

    subgraph "内部模块"
        RING["ring_buffer.py"]
        FLY["flywheel.py"]
        FW["decorator.py"]
        MCP_SRV["ur5_server.py"]
    end

    subgraph "模型/配置"
        UR5XML["specs/ur5e.xml"]
    end

    NP --> RING
    NP --> FLY
    NP --> FW
    MU --> FW
    MCP --> MCP_SRV
    RCL --> MCP_SRV
    FW --> NP
    FLY --> RING
    MCP_SRV --> FW
    FW --> UR5XML

    style NP fill:#e0e0e0
    style MU fill:#e0e0e0
    style MCP fill:#e0e0e0
    style RCL fill:#e0e0e0
    style RING fill:#42a5f5,color:#000
    style FLY fill:#1e88e5,color:#fff
    style FW fill:#ef5350,color:#fff
    style MCP_SRV fill:#ce93d8,color:#000
    style UR5XML fill:#e8f5e9
```

---

## 6. 测试架构

### 6.1 测试分层

```mermaid
flowchart LR
    subgraph "单元测试 (pytest)"
        T1["test_data_layer.py<br/>RingBuffer + DataFlywheel"]
        T2["test_firewall.py<br/>DigitalTwinFirewall + 装饰器"]
        T3["test_mcp_server.py<br/>MCP 结构 (mock rclpy)"]
    end

    subgraph "集成测试"
        T4["integration_test.py<br/>端到端验证"]
    end

    T1 --> CI["CI/CD Pipeline"]
    T2 --> CI
    T3 --> CI
    T4 --> CI
    CI --> BUILD["hatch build"]

    style T1 fill:#42a5f5,color:#fff
    style T2 fill:#ef5350,color:#fff
    style T3 fill:#fff3e0
    style T4 fill:#fce4ec
    style CI fill:#e0e0e0
```

### 6.2 关键测试模式

**防火墙测试 — 使用真实 MuJoCo 模型：**
```python
@pytest.fixture
def firewall():
    return DigitalTwinFirewall(
        model_path=MODEL_PATH,
        joint_limits=JOINT_LIMITS,
        sim_steps_per_check=10,  # 加速测试
    )
```

**MCP 测试 — mock rclpy：**
```python
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
from rosclaw.mcp.ur5_server import UR5ROSNode
```

### 6.3 CI/CD Pipeline

```mermaid
flowchart LR
    PR["Pull Request<br/>→ main/develop"] --> LINT["Lint & Format<br/>ruff check + ruff format"]
    PR --> TYPE["Type Check<br/>mypy strict"]
    LINT --> TEST["Test<br/>pytest + coverage<br/>3.10/3.11/3.12 矩阵"]
    TYPE --> TEST
    TEST --> INTTEST["Integration Test"]
    INTTEST --> BUILD["Build<br/>hatch build"]
    BUILD -->|"v* tag?"| PYPI["Publish to PyPI"]

    style PR fill:#e3f2fd
    style LINT fill:#e8f5e9
    style TYPE fill:#e8f5e9
    style TEST fill:#fff3e0
    style INTTEST fill:#fce4ec
    style BUILD fill:#f3e5f5
    style PYPI fill:#ffebee
```

---

## 7. 配置与常量

### 7.1 pyproject.toml 关键配置

```toml
[project]
requires-python = ">=3.10"
dependencies = ["numpy>=1.24", "mujoco>=3.0", "mcp>=1.0"]

[project.optional-dependencies]
ros2 = ["rclpy>=3.0", "geometry-msgs", "sensor-msgs", ...]
dev = ["pytest>=7", "ruff>=0.1", "mypy>=1.5"]

[tool.ruff]
line-length = 100
ignore = ["E501"]

[tool.mypy]
strict = true
ignore_missing_imports = true

[tool.hatch.build.targets.wheel]
packages = ["src/rosclaw"]
```

### 7.2 UR5e 规格常量

| 关节 | 位置限制 (rad) | 速度限制 (rad/s) | 力矩限制 (Nm) |
|------|------|------|------|
| shoulder_pan | ±2π | 3.15 | 150 × 0.8 |
| shoulder_lift | ±2π | 3.15 | 150 × 0.8 |
| elbow | ±π | 3.15 | 100 × 0.8 |
| wrist_1 | ±2π | 6.28 | 28 × 0.8 |
| wrist_2 | ±2π | 6.28 | 28 × 0.8 |
| wrist_3 | ±2π | 6.28 | 28 × 0.8 |

### 7.3 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROBOT_IP` | `192.168.1.100` | UR5 机器人 IP |
| `ROBOT_PORT` | `50002` | RTDE 端口 |
| `DIGITAL_TWIN_ENABLED` | `true` | 是否启用数字孪生 |
| `MUJOCO_MODEL_PATH` | `src/rosclaw/specs/ur5e.xml` | MuJoCo 模型路径 |
| `SAFETY_LEVEL` | `strict` | 安全验证级别 |

---

## 8. 设计文档中规划但尚未实现的功能

| 特性 | 当前实现 | 设计文档 (规划) |
|------|----------|--------|
| 硬限制检查 | `DigitalTwinFirewall` | Layer 1: `HardLimitChecker` |
| MuJoCo 仿真 | `DigitalTwinFirewall` | Layer 3: `MJXFirewall` (GPU 并行) |
| 解析验证 | ❌ 未实现 | Layer 2: `Pinocchio + Ruckig` |
| 预测性安全 | ❌ 未实现 | Layer 4: `Neural Twin` |
| 多通道缓冲区 | `MultiChannelRingBuffer` | ✅ 已实现 |
| LeRobot 导出 | `export_to_lerobot()` | ✅ 已实现 (简化版) |
| MCP 工具 | 6 个 `ur5_*` 工具 | ✅ 已实现 |
| 安全编排器 | ❌ 未实现 | `SafetyFirewallOrchestrator` |
| Skill 市场 | ❌ 未实现 | `ClawHub` + `rosclaw skills` CLI |
| 跨硬件迁移 | ❌ 未实现 | `CrossEmbodimentAdapter` |

---

## 9. 关键文件速查

| 文件 | 核心类/函数 | 行数 |
|------|------|------|
| `src/rosclaw/mcp/ur5_server.py` | `UR5ROSNode`, `UR5MCPServer`, `RobotState` | ~650 |
| `src/rosclaw/firewall/decorator.py` | `DigitalTwinFirewall`, `mujoco_firewall` | ~440 |
| `src/rosclaw/data/flywheel.py` | `DataFlywheel`, `RobotState`, `DataEvent` | ~410 |
| `src/rosclaw/data/ring_buffer.py` | `RingBuffer`, `MultiChannelRingBuffer` | ~300 |
| `src/rosclaw/__init__.py` | Package entry, exports | ~20 |
| `tests/test_firewall.py` | 防火墙测试类 | ~360 |
| `tests/test_data_layer.py` | 数据层测试类 | ~320 |
| `tests/test_mcp_server.py` | MCP 结构测试 (mock) | ~370 |
| `scripts/integration_test.py` | 端到端集成测试 | ~270 |
| `src/rosclaw/specs/ur5e.xml` | MuJoCo UR5e 模型 | ~128 |
