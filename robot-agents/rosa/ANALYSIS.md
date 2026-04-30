# ROSA 项目分析报告

## 1. 项目概述

**ROSA** (Robot Operating System Agent) 是由 NASA JPL 开发的 AI 驱动机器人操作系统助手。它基于 LangChain 的 `create_tool_calling_agent` 框架构建，允许用户通过自然语言查询和控制 ROS1/ROS2 机器人系统。

**核心定位**：用自然语言与 ROS 系统交互的 AI Agent，降低机器人开发与运维门槛。

**技术栈**：Python 3.9+、LangChain、OpenAI/Azure/Anthropic/Ollama LLM、ROS1 (Noetic) / ROS2 (Humble/Iron/Jazzy)

**版本**：1.0.10 | **许可**：Apache 2.0 | **发布**：PyPI (`jpl-rosa`)

---

## 2. 系统功能总体描述图


```mermaid
block-beta
    columns 2
    block:用户层
        columns 1
        User["\U0001F464 用户\n自然语言输入\n(查询/控制/诊断)"]
    end
    block:Agent层
        columns 1
        ROSA["\U0001F916 ROSA Agent 核心\n- LLM 决策引擎\n- 对话历史管理\n- 最多100次迭代循环"]
    end
    block:工具层
        columns 4
        block:通用工具
            columns 1
            Calc["\U0001F522 数学计算\n17个工具\n加减乘除/三角函数/几何"]
            Log["\U0001F4CB 日志工具\n1个工具\n日志读取/过滤"]
            Sys["\U0001F9E1 系统工具\n3个工具\n噪音/调试/等待"]
        end
        block:ROS1工具
            columns 1
            ROS1["\U0001F4E6 ROS1 (Noetic)\n20个工具\nrostopic/rosnode/\nrosservice/rosparam/\nrospkg/roslaunch等"]
        end
        block:ROS2工具
            columns 1
            ROS2["\U0001F4E6 ROS2 (Humble)\n13个工具\nros2 node/topic/\nservice/param/doctor等"]
        end
        block:黑名单机制
            columns 1
            Black["\U0001F6D1 自动黑名单注入\n每个工具执行前\n强制注入\nLLM无法绕过"]
        end
    end
    block:执行层
        columns 3
        ROSMaster["\U0001F517 ROS Master\n(rospy API)\n节点注册/话题发现\n服务注册/参数存储"]
        ROS2CLI["\U0001F6E1\ufe0f ros2 CLI\n(subprocess)\nnode/topic/service\nparam 命令执行"]
        FileSystem["\U0001F4C1 文件系统\n日志文件/launch文件\nROS参数存储"]
    end
    block:机器人层
        columns 1
        Robot["\U0001F916 机器人/仿真环境\nTurtleSim / 真实机器人\nNvidia IsaacSim / 其他"]
    end
    User --> ROSA
    ROSA --> Calc
    ROSA --> Log
    ROSA --> Sys
    ROSA --> ROS1
    ROSA --> ROS2
    Calc -.-> Black
    ROS1 --> ROSMaster
    ROS1 --> FileSystem
    ROS2 --> ROS2CLI
    ROS2 --> FileSystem
    ROSMaster --> Robot
    ROS2CLI --> Robot
    FileSystem --> Robot

    style User fill:#e1f5fe,stroke:#01579b
    style ROSA fill:#fff9c4,stroke:#f57f17
    style Calc fill:#e8f5e9,stroke:#2e7d32
    style Log fill:#e8f5e9,stroke:#2e7d32
    style Sys fill:#e8f5e9,stroke:#2e7d32
    style ROS1 fill:#fce4ec,stroke:#c62828
    style ROS2 fill:#fce4ec,stroke:#c62828
    style Black fill:#fff3e0,stroke:#e65100
    style ROSMaster fill:#f3e5f5,stroke:#6a1b9a
    style ROS2CLI fill:#f3e5f5,stroke:#6a1b9a
    style FileSystem fill:#f3e5f5,stroke:#6a1b9a
    style Robot fill:#e0f2f1,stroke:#00695c
```

---

## 3. 核心架构图

```mermaid
graph TB
    User["\U0001F4AC 用户自然语言查询"] --> ROSA["ROSA 类\nsrc/rosa/rosa.py"]

    subgraph InitInit ["ROSA.__init__ 初始化"]
        LLM["初始化 LLM\nChatOpenAI/Azure/Anthropic/Ollama"]
        Tools["组装工具集\n根据 ros_version 条件加载"]
        Prompt["构建 PromptTemplate\n8条系统指令 + 自定义 + 历史"]
        Exec["创建 AgentExecutor"]
    end

    ROSA --> InitInit
    InitInit --> LLM
    InitInit --> Tools
    InitInit --> Prompt
    InitInit --> Exec

    subgraph AgentLoop ["LangChain AgentExecutor 决策循环"]
        Direction["LLM 循环: 理解 --> 决策 --> 调用工具 --> 观察\n最多100次迭代"]
    end

    ROSA --> Invoke["invoke() 同步阻塞\n返回最终响应"]
    ROSA --> Stream["astream() 异步流式\nyield token/tool_start/tool_end/final/error"]

    Invoke --> AgentLoop
    Stream --> AgentLoop
    AgentLoop --> Direction

    subgraph ToolsDetail ["工具执行"]
        ROSATools["ROSATools.get_tools()\n[calculation, log, system, ros1/ros2]"]
        ToolDecor["@tool 装饰函数\n自动发现 + 黑名单注入"]
        PromptSystem["Prompt 系统\n- system_prompts (8条硬编码)\n- RobotSystemPrompts (自定义)\n- 对话历史 + 用户输入"]
        ROSATools --> ToolDecor
    end

    AgentLoop --> ToolsDetail

    style User fill:#e1f5fe,stroke:#01579b
    style ROSA fill:#fff9c4,stroke:#f57f17
    style Invoke fill:#c8e6c9,stroke:#2e7d32
    style Stream fill:#c8e6c9,stroke:#2e7d32
    style AgentLoop fill:#fff9c4,stroke:#f57f17
    style ToolsDetail fill:#e8f5e9,stroke:#2e7d32
    style PromptSystem fill:#fce4ec,stroke:#c62828
```

---

## 4. 数据流与处理流程

### 4.1 单次请求处理流程

```mermaid
sequenceDiagram
    participant U as 用户
    participant R as ROSA Agent
    participant L as LLM
    participant T as 工具
    participant ROS as ROS系统

    U->>R: invoke("画个圆")
    R->>L: 发送上下文+用户输入
    L-->>R: "需要移动工具"
    R->>T: rosnode_list()
    T->>ROS: rosnode list (subprocess)
    ROS-->>T: [node列表]
    T-->>R: {"nodes": [...]}

    R->>T: rostopic_list()
    T->>ROS: rostopic list
    ROS-->>T: [topic列表]
    T-->>R: {"topics": [...]}

    R->>T: ros2_param_set(...)
    T->>ROS: ros2 param set
    ROS-->>T: {success: true}
    T-->>R: {"success": true}

    L-->>R: 最终响应
    R-->>U: "已设置参数..."
    R->>R: chat_history: HumanMessage -> AIMessage
```

### 4.2 Agent 迭代循环流程

```mermaid
flowchart TD
    A["用户查询"] --> B["LLM 理解意图"]
    B --> C{需要工具?}
    C -- 是 --> D["工具执行引擎"]
    D --> E["1. 查找工具\n2. 注入黑名单\n3. 执行调用\n4. 返回结果"]
    E --> F{"任务完成?"}
    F -- 否 --> G{>=100次迭代?}
    G -- 否 --> B
    G -- 是 --> H["停止,返回错误"]
    F -- 是 --> I["LLM 生成最终响应"]
    I --> J["返回给用户"]
    C -- 否 --> I

    style A fill:#e3f2fd,stroke:#1565c0
    style B fill:#fff9c4,stroke:#f57f17
    style D fill:#e8f5e9,stroke:#2e7d32
    style I fill:#e3f2fd,stroke:#1565c0
    style J fill:#c8e6c9,stroke:#2e7d32
    style H fill:#ffebee,stroke:#c62828
```

---

## 5. 模块详细分析

### 5.1 核心模块

#### 5.1.1 ROSA 类 (`src/rosa/rosa.py`)

**职责**：整个系统的入口和协调器。

| 属性/方法 | 说明 |
|-----------|------|
| `ros_version` | 1=ROS1, 2=ROS2，决定加载哪组ROS工具 |
| `llm` | 支持的模型：ChatOpenAI / AzureChatOpenAI / ChatAnthropic / ChatOllama |
| `invoke(query)` | 同步阻塞调用，返回最终响应 |
| `astream(query)` | 异步流式调用，yield token/tool_start/tool_end/final/error 事件 |
| `clear_chat()` | 清空对话历史 |
| `_get_tools()` | 根据 ros_version 组装工具集 |
| `_get_prompts()` | 构建 ChatPromptTemplate (系统提示 + 对话历史 + 用户输入) |
| `_token_callback()` | OpenAI token 用量追踪 (仅 ChatOpenAI/AzureChatOpenAI 支持) |

**关键设计**：
- 工具自动发现：`dir(package)` 遍历所有 `@tool` 装饰的函数
- `blacklist` 自动注入：`inject_blacklist` 装饰器在每个工具执行前自动注入黑名单
- token 追踪：仅 OpenAI 系列模型支持，非 OpenAI 模型自动禁用
- streaming 默认开启，但 streaming 模式下不显示 token 用量

#### 5.1.2 Prompts 系统 (`src/rosa/prompts.py`)

**system_prompts** (8条硬编码指令)，按顺序排列：

| # | 主题 | 关键内容 |
|---|---|---|
| 1 | 身份定义 | ROSA 身份，实时信息优先 |
| 2 | 工具调用强制 | 声称"不存在"前必须先调工具验证 |
| 3 | 顺序执行 | **禁止并行工具调用**，必须一个个执行 |
| 4 | 行动工作流 | 先 rosnode_list + rostopic_list，再行动 |
| 5 | 名称确认 | 使用前先获取列表，不用假设的名字 |
| 6 | /rosa 命名空间 | agent 自用的 rosparam 必须用 `/rosa/` 前缀 |
| 7 | 文件读取 | 超过 32KB 需分段读取 |
| 8 | 数学计算强制 | 角度/距离/坐标计算必须用数学工具 |

**RobotSystemPrompts**：用户可自定义的系统提示扩展，支持 9 个字段（身份、操作员信息、关键指令、约束、环境、能力、假设、任务目标、环境变量）。

#### 5.1.3 ROSATools 类 (`src/rosa/tools/__init__.py`)

**职责**：工具注册与管理的核心类。

```mermaid
graph LR
    subgraph Init ["ROSATools.__init__"]
        Auto["自动加载\ncalculation/log/system"]
        Cond["条件加载\nros_version=1 → ros1\nros_version=2 → ros2"]
        Discover["__iterative_add()\n遍历 dir(package)\n发现 @tool 函数"]
    end
    subgraph API ["公开接口"]
        AddTools["add_tools(tools)\n手动添加工具列表"]
        AddPkgs["add_packages(pkg)\n手动添加工具包"]
        GetTools["get_tools()\n返回 List[Tool]"]
    end

    Init --> Auto
    Init --> Cond
    Auto --> Discover
    Cond --> Discover
    Init --> API
```

**inject_blacklist 机制**：当工具函数签名包含 `blacklist` 参数时，自动将默认黑名单 + 工具自带黑名单合并注入。LLM 无法绕过此机制。

### 5.2 工具分类

#### 通用工具 (所有版本共用)

**calculation.py (17个工具)**：
- 聚合运算：`add_all`, `multiply_all`, `mean`, `median`, `mode`, `variance`
- 二元运算：`add`, `subtract`, `multiply`, `divide`, `exponentiate`, `modulo`
- 三角函数：`sine`, `cosine`, `tangent`, `asin`, `acos`, `atan`, `sinh`, `cosh`, `tanh`
- 计数：`count_list`, `count_words`, `count_lines`
- 单位转换：`degrees_to_radians`, `radians_to_degrees`
- 几何核心：`sqrt`, `atan2`, `distance_between_points`, `calculate_line_angle_and_distance`

**log.py (1个工具)**：
- `read_log`：读取日志文件，支持级别过滤 (ERROR/INFO/DEBUG 等) 和行数限制，最大 200 行返回

**system.py (3个工具)**：
- `set_verbosity`：控制 agent 输出详细程度
- `set_debugging`：控制 agent 调试输出 (API调用/工具执行详情)
- `wait`：等待指定秒数

#### ROS1 专属工具 (ros1.py, 14个工具)

| 工具 | 功能 |
|------|------|
| `rosgraph_get` | 获取ROS图 (publisher/topic/subscriber) |
| `rostopic_list` | 列出话题 (支持 pattern/namespace/blacklist 过滤) |
| `rosnode_list` | 列出节点 (支持 pattern/namespace/blacklist 过滤) |
| `rostopic_info` | 话题详细信息 (type/publishers/subscribers) |
| `rostopic_echo` | 实时回声话题消息 (1-100条) |
| `rosnode_info` | 节点详细信息 |
| `rosservice_list` | 列出服务 (支持多种过滤) |
| `rosservice_info` | 服务详细信息 |
| `rosservice_call` | 调用服务 |
| `rosmsg_info` | ROS消息类型定义 |
| `rossrv_info` | ROS服务类型定义 |
| `rosparam_list` | 列出参数 |
| `rosparam_get` | 获取参数值 |
| `rosparam_set` | 设置参数值 (支持 /rosa 命名空间) |
| `rospkg_list` | 列出ROS包 |
| `rospkg_info` | ROS包详情 (路径/依赖/manifest) |
| `rospkg_roots` | ROS包根路径 |
| `roslog_list` | 列出ROS日志文件 |
| `roslaunch` | 启动launch文件 |
| `roslaunch_list` | 列出launch文件 |
| `rosnode_kill` | 杀死节点 |

#### ROS2 专属工具 (ros2.py, 11个工具)

| 工具 | 功能 |
|------|------|
| `ros2_node_list` | 列出节点 (pattern/blacklist) |
| `ros2_topic_list` | 列出话题 (pattern/blacklist) |
| `ros2_topic_echo` | 回声话题 (1-10条) |
| `ros2_service_list` | 列出服务 (pattern/blacklist) |
| `ros2_node_info` | 节点详细信息 |
| `ros2_topic_info` | 话题详细信息 |
| `ros2_param_list` | 列出参数 (按node/pattern过滤) |
| `ros2_param_get` | 获取参数值 |
| `ros2_param_set` | 设置参数值 |
| `ros2_service_info` | 服务类型信息 |
| `ros2_service_call` | 调用服务 |
| `ros2_doctor` | 检查ROS环境 |
| `roslog_list` | 列出ROS2日志文件 |

### 5.3 示例 Agent：TurtleSim

**`src/turtle_agent/`** 是一个自定义 Agent 示例，展示如何扩展 ROSA。

```mermaid
graph TB
    subgraph Example["TurtleAgent (继承 ROSA)"]
        direction TB
        CustomLLM["自定义 LLM (get_llm)"]
        CustomPrompt["自定义 Prompts (get_prompts)"]
        Blacklist["自定义黑名单 ['master', 'docker']"]
        subgraph CustomTools["自定义工具"]
            dir1["@tool: cool_turtle_tool"]
            dir2["Tool(blast_off)\n手动创建Tool对象"]
            dir3["tool_packages=[turtle_tools]\n从包自动加载"]
        end
        ROS1Tools["ROS1 专属工具 (全部)"]
        subgraph UI["交互界面"]
            UIRun["run(): 主循环\nrich UI / help/examples/clear/exit"]
            UISubmit["submit(): 提交查询"]
            UIPrint["print_response(): 同步显示"]
            UIStream["stream_response(): 流式显示\ntoken实时渲染"]
            UIEvent["show_event_details(): 事件详情"]
        end
    end

    CustomLLM --> Example
    CustomPrompt --> Example
    Blacklist --> Example
    CustomTools --> Example
    ROS1Tools --> Example
    UI --> Example

    style Example fill:#e3f2fd,stroke:#1565c0
    style CustomTools fill:#fff9c4,stroke:#f57f17
    style UI fill:#e8f5e9,stroke:#2e7d32
```

---

## 6. 目录结构

```
rosa/
├── src/
│   ├── rosa/                     # 核心包
│   │   ├── __init__.py           # 导出: ROSA, RobotSystemPrompts, ChatModel
│   │   ├── rosa.py               # ROSA 类 (341行) - 入口与协调器
│   │   ├── prompts.py            # system_prompts (8条) + RobotSystemPrompts 类
│   │   └── tools/
│   │       ├── __init__.py       # ROSATools 类 + inject_blacklist
│   │       ├── calculation.py    # 17个数学/几何工具
│   │       ├── log.py            # 1个日志读取工具
│   │       ├── system.py         # 3个系统管理工具
│   │       ├── ros1.py           # 20个ROS1专属工具
│   │       └── ros2.py           # 13个ROS2专属工具
│   └── turtle_agent/             # TurtleSim 示例 Agent
│       ├── launch/agent.launch   # ROS launch 文件
│       ├── package.xml           # ROS package 元数据
│       ├── CMakeLists.txt        # ROS build 配置
│       └── scripts/
│           ├── turtle_agent.py   # TurtleAgent 主程序 (366行)
│           ├── llm.py            # LLM 配置
│           ├── prompts.py        # 示例自定义提示
│           ├── help.py           # 帮助信息
│           └── tools/
│               └── turtle.py     # 海龟自定义工具
├── tests/
│   └── test_rosa/
│       ├── test_rosa.py          # ROSA 核心测试
│       ├── test_prompts.py       # Prompt 测试
│       ├── test_signal_handling.py # 信号处理测试
│       └── tools/
│           ├── test_calculation.py
│           ├── test_log.py
│           ├── test_system.py
│           ├── test_rosa_tools.py
│           ├── test_ros1.py      # ROS_VERSION=1 过滤
│           ├── test_ros2.py      # ROS_VERSION=2 过滤
│           └── test_rosa_tools.py
├── setup.py                      # 安装配置
├── pyproject.toml                # 依赖配置 (版本 1.0.10)
├── Dockerfile                    # ROS1 Noetic 容器
├── .github/
│   ├── workflows/
│   │   ├── ci.yml                # CI: Noetic + Humble 双容器测试
│   │   └── publish.yml           # PyPI 发布
│   └── copilot-instructions.md   # Copilot 指令 (与CLAUDE.md内容一致)
├── README.md
├── TESTING.md
├── CONTRIBUTING.md
├── CHANGELOG.md
├── LICENSE (Apache 2.0)
└── demo.sh
```

---

## 7. 依赖分析

### 7.1 核心依赖 (`pyproject.toml`)

| 依赖 | 版本 | 用途 |
|------|------|------|
| langchain | ~=0.3.23 | Agent 框架核心 |
| langchain-community | ~=0.3.21 | 社区工具集成 |
| langchain-core | ~=0.3.52 | Agent 核心抽象 |
| langchain-openai | ~=0.3.14 | OpenAI LLM 适配 |
| pydantic | - | 数据验证 |
| pyinputplus | - | 用户输入验证 |
| azure-identity | - | Azure 认证 |
| cffi | - | C 扩展接口 |
| rich | - | 终端渲染 |
| pillow | >=10.4.0 | 图像处理 |
| numpy | >=1.26.4 | 数值计算 |
| PyYAML | ==6.0.1 | YAML 解析 |
| python-dotenv | >=1.0.1 | 环境变量加载 |

### 7.2 可选依赖

| 额外 | 依赖 | 用途 |
|------|------|------|
| anthropic | langchain-anthropic ~=0.3.12 | Anthropic Claude LLM |
| ollama | langchain-ollama ~=0.3.2 | Ollama 本地 LLM |
| all | 以上两者 | 全部可选依赖 |

### 7.3 ROS 运行时依赖 (条件加载)

```mermaid
graph LR
    subgraph ROSONE ["ros_version=1"]
        R1a["rospy"]
        R1b["rosgraph"]
        R1c["rosmsg"]
        R1d["rosnode"]
        R1e["rosparam"]
        R1f["rospkg"]
        R1g["rosservice"]
        R1h["rostopic"]
    end
    subgraph ROSOVTWO ["ros_version=2"]
        R2a["rclpy\n(subprocess调用ros2 cli)"]
    end
```

---

## 8. 测试体系

### 8.1 测试框架

- **框架**：Python unittest (pytest 也兼容)
- **过滤机制**：`ROS_VERSION` 环境变量

### 8.2 测试命令

```bash
# 运行全部测试
python -m unittest discover -s tests --verbose

# 运行单个测试文件
python -m unittest tests/test_rosa/tools/test_calculation.py -v

# 运行指定测试类
python -m unittest tests/test_rosa/tools/test_calculation.py::TestCalculationTools -v

# ROS1 环境测试
ROS_VERSION=1 python -m unittest discover -s tests --verbose

# ROS2 环境测试
ROS_VERSION=2 python -m unittest discover -s tests --verbose
```

### 8.3 CI/CD 流程

```mermaid
flowchart TB
    Push["Push/PR to main/dev"] --> Check{分支?}
    Check --> |main| Noetic["test-noetic\nROS1 Noetic / Python 3.9 / Docker"]
    Check --> |dev| Humble["test-humble\nROS2 Humble / Python 3.10 / Docker"]
    Release["Release 创建"] --> Publish["publish\nPyPI 发布"]

    Noetic --> RunTests["python -m unittest discover -s tests --verbose"]
    Humble --> RunTests
    RunTests --> Results["测试结果"]

    style Push fill:#e3f2fd,stroke:#1565c0
    style Noetic fill:#fff9c4,stroke:#f57f17
    style Humble fill:#fff9c4,stroke:#f57f17
    style Publish fill:#e8f5e9,stroke:#2e7d32
```

---

## 9. 关键技术设计

### 9.1 工具自动发现机制

```python
# ROSATools.__iterative_add() 实现
for tool_name in dir(package):
    if not tool_name.startswith("_"):
        t = getattr(package, tool_name)
        if hasattr(t, "name") and hasattr(t, "func"):
            self.__tools.append(t)
```

**说明**：所有 `@tool` 装饰的函数无需手动注册即可自动生效。

```mermaid
graph TB
    Pkg["工具包 module"] --> Dir["dir(package)"]
    Dir --> Iterate["遍历每个属性"]
    Iterate --> Check{"有 name+func?"}
    Check -- 是 --> Add["self.__tools.append"]
    Check -- 否 --> Skip["跳过"]

    style Pkg fill:#e3f2fd,stroke:#1565c0
    style Add fill:#c8e6c9,stroke:#2e7d32
    style Skip fill:#ffebee,stroke:#c62828
```

### 9.2 黑名单注入机制

```
工具定义: def my_tool(blacklist: List[str] = None)

执行时:
  args[0]["blacklist"] = default_blacklist + user_blacklist
  # LLM 无法绕过，因为每次调用都会被强制注入
```

### 9.3 Prompt 组装顺序

```
1. system_prompts[0]  "你是ROSA..."
2. system_prompts[1]  "CRITICAL - 工具使用要求..."
3. system_prompts[2]  "CRITICAL - 顺序执行..."
4. system_prompts[3]  "行动工作流..."
5. system_prompts[4]  "名称确认..."
6. system_prompts[5]  "/rosa 命名空间..."
7. system_prompts[6]  "文件读取..."
8. system_prompts[7]  "数学计算强制..."
9. RobotSystemPrompts (用户自定义，追加到 system_prompts 之后)
10. MessagesPlaceholder("chat_history")  (对话历史)
11. ("user", "{input}")  (用户输入)
12. MessagesPlaceholder("agent_scratchpad")  (Agent 中间结果)
```

```mermaid
graph LR
    subgraph SP["system_prompts (8条硬编码)"]
        S1["身份"]
        S2["工具强制"]
        S3["顺序执行"]
        S4["行动工作流"]
        S5["名称确认"]
        S6["rosa命名空间"]
        S7["文件读取"]
        S8["数学强制"]
    end
    subgraph RP["RobotSystemPrompts (用户自定义)"]
        U1["用户设定"]
    end
    subgraph CH["对话上下文"]
        H1["chat_history"]
        I1["{input}"]
        S11["agent_scratchpad"]
    end

    SP --> RP
    RP --> CH
    CH --> Template["ChatPromptTemplate"]

    style SP fill:#fce4ec,stroke:#c62828
    style RP fill:#e3f2fd,stroke:#1565c0
    style CH fill:#fff9c4,stroke:#f57f17
```

### 9.4 流式响应事件类型

| 事件类型 | 内容 | 触发时机 |
|----------|--|------|
| `token` | `{"content": ".."}` | LLM 生成每个 token |
| `tool_start` | `{"name": "..", "input": {...}}` | 工具开始执行 |
| `tool_end` | `{"name": "..", "output": ".."}` | 工具执行完成 |
| `final` | `{"content": ".."}` | Agent 返回最终响应 |
| `error` | `{"content": ".."}` | 发生错误 |

```mermaid
flowchart LR
    A["astream() 调用"] --> T{"事件类型?"}
    T --> |token| Token["content: 实时文本片段"]
    T --> |tool_start| TS["name + input"]
    T --> |tool_end| TE["name + output"]
    T --> |final| Final["content: 最终响应"]
    T --> |error| Err["content: 错误信息"]

    style A fill:#e3f2fd,stroke:#1565c0
    style Token fill:#c8e6c9,stroke:#2e7d32
    style TS fill:#fff9c4,stroke:#f57f17
    style TE fill:#fff9c4,stroke:#f57f17
    style Final fill:#c8e6c9,stroke:#2e7d32
    style Err fill:#ffebee,stroke:#c62828
```

---

## 10. 扩展指南

### 10.1 添加新工具

```python
# src/rosa/tools/my_tools.py
from langchain.agents import tool

@tool
def my_new_tool(description: str) -> str:
    """描述: 新工具的功能说明。"""
    return f"处理结果: {description}"
```

**规则**：
- 使用 `@tool` 装饰器
- 自动被 `dir(package)` 发现
- 如需黑名单支持，添加 `blacklist: Optional[List[str]] = None` 参数

### 10.2 创建自定义 Agent

```python
from rosa import ROSA, RobotSystemPrompts

class MyAgent(ROSA):
    def __init__(self):
        super().__init__(
            ros_version=1,
            llm=my_llm,
            tools=[my_new_tool],           # 或 tool_packages=[my_tools_pkg]
            prompts=RobotSystemPrompts(
                embodiment_and_persona="你的机器人设定",
                critical_instructions="关键指令",
            ),
            blacklist=["node_to_exclude"],
        )
```

### 10.3 扩展新 ROS 版本支持

在 `src/rosa/tools/` 下新建 `ros3.py`，在 `ROSATools.__init__` 中添加 `elif self.__ros_version == 3:` 分支。

---

## 11. 安全考虑

1. **黑名单注入**：`inject_blacklist` 强制注入，LLM 无法绕过
2. **参数验证**：`execute_ros_command` 在 ros2.py 中校验命令白名单 (`node/topic/service/param/doctor`)
3. **文件读取限制**：日志读取超过 200 行需分页，超过 32KB 需分段
4. **消息数量限制**：`rostopic_echo` count 限制 1-100，`ros2_topic_echo` 限制 1-10
5. **子进程执行**：ROS2 工具通过 `subprocess` 执行，ROS1 通过 `rospy` API

---

## 12. 版本信息

| 项目 | 值 |
|------|---|
| 当前版本 | 1.0.10 |
| Python 要求 | 3.9 - 3.x |
| ROS1 支持 | Noetic |
| ROS2 支持 | Humble / Iron / Jazzy |
| 主要维护者 | Rob Royce (Jet Propulsion Laboratory) |
| 论文 | arXiv: 2410.06472 |
