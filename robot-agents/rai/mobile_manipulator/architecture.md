# Mobile Manipulator — 智能仓库机器人架构说明

## 1. 项目概述

这是一个基于 **Robotnik Kairos** 移动操作机器人的仓库自动化演示系统。
系统在 O3DE 模拟环境中运行，通过多智能体系统（LangGraph + LangChain）调度 LLM/VLM 进行感知、规划和任务执行，所有推理均在本地 AMD Ryzen AI 硬件上完成。

**核心技术栈：** ROS 2 (Jazzy) + O3DE + RAI Framework + Python 3.12 + uv + Docker Compose

---

## 2. 系统分层架构

```mermaid
graph TB
    subgraph LLM-VLM["LLM/VLM 推理层"]
        LLM["LLM<br/>gpt-oss-20b<br/>port 8080"]
        VLM["VLM<br/>lfm2-vl<br/>port 8081"]
        EMB["Embedding<br/>Qwen3-0.6b<br/>port 8082"]
        RR["Reranker<br/>Qwen3-Reranker<br/>port 8083"]
    end

    subgraph Agent["多智能体层 (rai_app/)"]
        subgraph MegaMind["MegaMind 编排器"]
            ORCH["Agent Orchestrator<br/>任务调度循环"]
            EXEC_HK["housekeep 执行器"]
            EXEC_PM["package_movement 执行器"]
            EXEC_IA["image_analysis 执行器"]
        end
        INSPECT["Inspection Agent<br/>VLM 异常检测"]
        SAFETY["Safety Agent<br/>RAG 安全合规"]
    end

    subgraph Control["控制层 (rai_app/control/)"]
        KC["KairosController"]
        NC["NavigationController<br/>Nav2 导航"]
        MC["ManipulatorController<br/>MoveIt2 机械臂"]
        SM["SceneManager<br/>仓库场景管理"]
    end

    subgraph ROS2["ROS 2 中间件层"]
        NAV2["Nav2 导航栈"]
        MOVEIT["MoveIt2 运动规划"]
        HMI["HMI 人机界面"]
    end

    subgraph Sim["O3DE 仿真层"]
        O3DE["O3DE 引擎<br/>仓库场景 + 机器人模型"]
    end

    LLM --> ORCH
    VLM --> ORCH
    VLM --> INSPECT
    VLM --> SAFETY
    EMB --> SAFETY
    RR --> SAFETY

    ORCH --> EXEC_HK
    ORCH --> EXEC_PM
    ORCH --> EXEC_IA
    EXEC_HK --> KC
    EXEC_PM --> KC
    EXEC_IA --> KC

    KC --> NC
    KC --> MC
    KC --> SM

    NC --> NAV2
    MC --> MOVEIT

    NAV2 <--> ROS2
    MOVEIT <--> ROS2
    HMI <--> ROS2

    ROS2 <--> Sim
```

---

## 3. MegaMind 多智能体编排

```mermaid
graph LR
    subgraph TaskInput["任务输入"]
        UT["/user_tasks<br/>低优先级任务"]
        IR["/inspection_result<br/>高优先级检测异常"]
        ES["/emergency_stop<br/>紧急停止"]
    end

    subgraph Scheduler["Orchestrator 调度循环"]
        HPQ["高优先级队列<br/>high_prio_task_queue"]
        LPQ["低优先级队列<br/>low_prio_task_queue"]
        LOOP["orchestrator_loop()<br/>任务选择 -> astream()"]
    end

    subgraph Executors["Executor 执行器"]
        HK["housekeep<br/>仓库整理/巡检/丢垃圾"]
        PM["package_movement<br/>包件搬运/投递"]
        IA["image_analysis<br/>损坏检测/图像描述"]
    end

    subgraph Tools["工具层"]
        T1["MoveFromCollectionToCollectionTool"]
        T2["HouseKeepTool"]
        T3["SortReturnedPackageTool"]
        T4["InspectionWarehouseRouteTool"]
        T5["ThrowTrashOutTool"]
        T6["MoveFromPoseToInspectionAreaTool"]
        T7["IsPackageDamagedTool"]
        T8["DescribeImageTool"]
    end

    UT --> LPQ
    IR --> HPQ
    ES -.-> ORCH

    HPQ --> LOOP
    LPQ --> LOOP
    LOOP --> HK
    LOOP --> PM
    LOOP --> IA

    HK --> T2
    HK --> T3
    HK --> T4
    HK --> T5
    PM --> T1
    PM --> T6
    PM --> T5
    IA --> T7
    IA --> T8
```

### 3.1 任务执行流程

```mermaid
sequenceDiagram
    participant HMI as HMI 人机界面
    participant ROS as ROS 2 话题
    participant ORCH as Orchestrator
    participant GG as LangGraph MegaMind
    participant EXEC as Executor Sub-agent
    participant TOOL as ROS 2 Tool
    participant KC as KairosController
    participant NAV as Nav2/MoveIt2

    HMI->>ROS: 发布任务到 /user_tasks
    ROS->>ORCH: add_task(msg) → low_prio_task_queue
    Note over ORCH: orchestrator_loop() 轮询任务队列
    ORCH->>ORCH: 从队列取任务 (高优先级优先)
    ORCH->>GG: astream(initial_state, config)
    GG->>GG: LLM 分析任务，选择执行器
    GG->>EXEC: 委派任务给 Executor
    EXEC->>EXEC: ReAct 循环，决定调用工具
    EXEC->>TOOL: 调用 Tool._run()
    TOOL->>KC: 导航/操作指令
    KC->>NAV: ROS 2 action/service call
    NAV-->>KC: 执行结果
    KC-->>TOOL: 操作完成/失败
    TOOL-->>EXEC: 返回结果字符串
    EXEC-->>GG: 工具执行结果
    GG-->>ORCH: 流式输出 (astream chunks)
    Note over ORCH: AgentActionsCallback 发布到 /agent/current_action
    ORCH->>ROS: 发布进度到 ROS 2 话题
    loop 任务通知 (1Hz)
        ORCH->>ROS: 发布当前任务/队列到 /orchestrator/*
    end
```

---

## 4. 控制层架构

```mermaid
graph TB
    subgraph KairosController["KairosController<br/>高层操作编排"]
        direction TB
        KC["统一接口"]
    end

    subgraph Navigation["NavigationController<br/>移动底盘导航"]
        NAV["Navigator<br/>ROS 2 Action 客户端"]
        N1["navigate_to_pose"]
        N2["drive_on_heading"]
        N3["follow_waypoints"]
        N4["spin"]
        N5["approach_target"]
        N6["warehouse_route"]
        CM["Costmap 检查<br/>is_position_available"]
    end

    subgraph Manipulator["ManipulatorController<br/>机械臂控制"]
        M1["move_arm_to_target_pose"]
        M2["move_arm_to_staging_pose"]
        M3["open/close_gripper"]
        M4["set_arm_joints"]
        M5["move_arm_to_base_pose"]
    end

    subgraph Strategies["ManipulationStrategy<br/>基于高度选择策略"]
        S_LOW["LowStrategy<br/>z < 0.6m"]
        S_MID_LOW["MidLowStrategy<br/>0.6m ≤ z < 0.9m"]
        S_MID_HIGH["MidHighStrategy<br/>0.9m ≤ z < 1.4m"]
        S_HIGH["HighStrategy<br/>z ≥ 1.4m"]
    end

    subgraph SceneManager["SceneManager<br/>仓库场景管理"]
        SLT["Slot 槽位管理"]
        COL["SlotsCollection<br/>货架/桌子集合"]
        ENT["Entity 实体管理<br/>spawn/get_pose/find_slot"]
    end

    KC --> Navigation
    KC --> Manipulator
    KC --> SceneManager
    KC --> Strategies

    NAV --> N1
    NAV --> N2
    NAV --> N3
    NAV --> N4
    N5 --> N1
    N6 --> N3
    CM --> N1
```

### 4.1 物品搬运核心流程

```mermaid
flowchart TD
    A["任务: 移动物品 A 到槽位 B"] --> B["确定抓取策略<br/>determine_strategy(pose.z)"]
    B --> C{"高度判断"}
    C -->|"z < 0.6"| D["Low 策略<br/>staging=1.8m, gripping=1.2m"]
    C -->|"0.6 ≤ z < 0.9"| E["MidLow 策略<br/>staging=1.2m, gripping=0.9m"]
    C -->|"0.9 ≤ z < 1.4"| F["MidHigh 策略<br/>staging=1.2m, gripping=0.9m"]
    C -->|"z ≥ 1.4"| G["High 策略<br/>staging=1.4m, gripping=0.8m"]

    D --> H["navigate_to_and_pick"]
    E --> H
    F --> H
    G --> H

    H --> I["1. approach_target_along_orientation"]
    I --> J["2. move_back 到 gripping 距离"]
    J --> K["3. lift_object:<br/>staging → target → close_gripper → above_target"]
    K --> L["4. move_back 到 staging 距离"]
    L --> M["5. move_arm_to_base_pose"]

    M --> N["navigate_to_and_place"]
    N --> O["1. approach_target_along_orientation"]
    O --> P["2. strategy.execute_placement"]
    P --> Q["3. place_object:<br/>above_target → target → open_gripper → staging"]
    Q --> R["4. move_back + arm_to_base"]
    R --> S["完成 ✓"]
```

---

## 5. 巡检智能体 (Inspection Agent)

```mermaid
flowchart LR
    subgraph Camera["摄像头"]
        CAM["/rgbd_camera/camera_image_color"]
    end

    subgraph VLMInspector["VlmWarehouseInspector"]
        Q["process_queue<br/>任务队列"]
        VLM_W["vlm_worker<br/>VLM 工作线程"]
        DETECT["detect_obstacle()<br/>VLM 结构化输出"]
        CHECK["check_if_anomaly_is_reported()<br/>去重 (60s TTL)"]
    end

    subgraph SceneGT["场景真值检测"]
        GT["get_anomaly_box_pose()<br/>从模拟获取物体位姿"]
    end

    subgraph Output["输出"]
        ANOM["/inspection_result<br/>Anomaly 消息"]
        MARKER["/marker<br/>Rviz2 可视化"]
        VLM_T["/vlm_topic<br/>VLM 描述"]
    end

    CAM --> Q
    Q --> VLM_W
    GT -.-> VLM_W
    VLM_W --> DETECT
    DETECT --> CHECK
    CHECK -->|"新异常"| ANOM
    CHECK -->|"已报告"| STOP[("跳过")]
    ANOM --> MARKER
    ANOM --> VLM_T
```

### 5.1 VLM 检测流程

```mermaid
flowchart TD
    A["收到图像帧"] --> B["VLM 描述性分析<br/>INSPECTION_TEXT_SYSTEM_PROMPT"]
    B --> C["VLM 结构化检测<br/>INSPECTION_JSON_SYSTEM_PROMPT"]
    C --> D{inspection_results<br/>是否为空?}
    D -->|是| E["AnomalyDescription<br/>obstacle_type=nothing"]
    D -->|否| F["最终分类<br/>INSPECTION_TEXT_FINAL"]
    F --> G{obstacle_type}
    G -->|"box"| H["发布 Anomaly(box)<br/>含 gripping_point 位姿"]
    G -->|"trash"| I["发布 Anomaly(trash)<br/>含 gripping_point 位姿"]
    G -->|"other"| J["发布 Anomaly(other)<br/>使用 robot 位姿"]
    G -->|"nothing"| K["不发布"]
```

---

## 6. 安全合规智能体 (Safety Agent)

```mermaid
flowchart TB
    subgraph Input["输入"]
        CAM["/rgbd_camera/camera_image_color"]
    end

    subgraph SafetyAgent["SafetyAgent"]
        DEDup["图像去重<br/>SSIM 相似度检查"]
        AGENT["Image Regulation Agent<br/>LangGraph StateGraph"]
    end

    subgraph AgentGraph["三阶段评估图"]
        V["vision 节点<br/>VLM 图像描述<br/>→ VisionObservation"]
        R["retrieve 节点<br/>FAISS 向量检索 +<br/>Reranker 重排序"]
        F["final 节点<br/>VLM 合规判定<br/>→ FinalImageSafetyOutput"]
    end

    subgraph RAG["RAG 法规知识库"]
        FAISS["FAISS 向量数据库"]
        REG["OSHA 法规文本<br/>chunk + embedding"]
        RERANK["Reranker 模型<br/>Qwen3-Reranker-0.6B"]
    end

    subgraph Output["输出"]
        VIOL["/safety 话题<br/>JSON 违规信息"]
        STORE["ViolationStorage<br/>safety_violations.json"]
    end

    CAM --> DEDup
    DEDup -->|"不同图像"| AGENT
    AGENT --> V
    V --> R
    R --> F
    R --> FAISS
    FAISS --> REG
    R --> RERANK
    F --> VIOL
    F --> STORE
```

---

## 7. 仓库场景管理

```mermaid
graph TB
    subgraph DataManager["SceneManager"]
        CSV["slots.csv 槽位定义"]
        CSV2["spawnables.csv 可生成实体"]
        CSV3["rack_assignment.csv 货架-物品映射"]
    end

    subgraph Collections["集合类型"]
        subgraph Racks["货架 (rack)"]
            R1["A01, A02... G01... K01..."]
            R_ITEMS["按物品类型存储<br/>cpu, gpu, pipes, cpu_trash..."]
        end
        subgraph Tables["桌子 (table)"]
            T1["t1 — returned_packages<br/>退货包件"]
            T2["t2 — outbound_shipment<br/>出货区"]
            T3["t3 — free<br/>空闲区"]
            T4["t4 — inspection<br/>检验区"]
        end
        subgraph Garbage["垃圾箱"]
            GC["GarbageContainer01"]
        end
    end

    subgraph SlotMgmt["槽位管理"]
        SL["Slot<br/>tag, origin_pose, entity_name"]
        OPS["find_empty_slots<br/>find_used_slots<br/>assign_entity_to_slot<br/>is_entity_within_slot"]
    end

    subgraph EntityOps["实体操作"]
        SPAWN["spawn_on_slot()<br/>带高斯噪声"]
        GET_POSE["get_pose()<br/>tf2 坐标变换"]
        FIND_SLOT["find_entity_slot()<br/>空间包含检测"]
        GRIP["get_top_gripping_point()<br/>find_closest_side_gripping_point()"]
    end

    CSV --> Collections
    CSV2 --> EntityOps
    CSV3 --> R_ITEMS
    Collections --> SlotMgmt
    SlotMgmt --> OPS
    EntityOps --> GET_POSE
    EntityOps --> FIND_SLOT
    EntityOps --> GRIP
```

---

## 8. Docker Compose 服务编排

```mermaid
graph TB
    subgraph DockerServices["Docker Compose 服务"]
        SIM["sim<br/>O3DE 仿真"]
        KAIROS["kairos<br/>ROS 2 机器人启动"]
        HMI["hmi<br/>人机界面"]
        AGENTS["agents<br/>MegaMind 编排器"]
        NAV_A["nav2_agent"]
        MOVE_A["moveit2_agent"]
        SCENE_A["scene_agent"]
        NAV_LM["nav_lifecycle_node"]
        INSPECT_A["inspection_agent"]
        SAFETY_A["safety_agent"]
        EMB_S["safety-embeddings<br/>llama.cpp:8082"]
        RR_S["safety-reranker<br/>llama.cpp:8083"]
    end

    SIM -->|ROS 2 通信| KAIROS
    AGENTS -->|ROS 2 通信| KAIROS
    AGENTS -->|LangChain| LLM_S["LLM/VLM 服务"]
    INSPECT_A -->|VLM 调用| LLM_S
    SAFETY_A -->|VLM 调用| LLM_S
    SAFETY_A -->|embedding| EMB_S
    SAFETY_A -->|reranking| RR_S
    HMI -->|ROS 2 通信| KAIROS
```

---

## 9. 关键配置文件

| 文件 | 用途 |
|------|------|
| `config.toml` | 本地模型配置 (llama.cpp 服务端口 8080-8083) |
| `cloud_config.toml` | 云端模型配置 (Docker Compose 默认使用) |
| `scripts/resources/slots.csv` | 仓库槽位布局 (坐标 + 四元数) |
| `scripts/resources/spawnables.csv` | 可生成实体及其 URI 映射 |
| `scripts/resources/rack_assignment.csv` | 货架与物品类型的对应关系 |
| `scripts/resources/warehouse_route.csv` | 巡检路径 waypoints |
| `docker/compose.yaml` | Docker Compose 多服务编排 |

---

## 10. ROS 2 关键话题与服务

### 10.1 话题

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/user_tasks` | `std_msgs/msg/String` | 用户任务输入 |
| `/inspection_result` | `robotec_kairos_ur10/msg/Anomaly` | 巡检异常输出 |
| `/emergency_stop` | `std_msgs/msg/String` | 紧急停止信号 |
| `/agent/current_action` | `rai_interfaces/msg/HRIMessage` | 智能体当前动作 |
| `/orchestrator/current_task` | `std_msgs/msg/String` | 当前执行任务 |
| `/orchestrator/tasks_queue` | `std_msgs/msg/String` | 任务队列状态 |
| `/orchestrator/heartbeat` | `std_msgs/msg/Header` | 心跳信号 |
| `/safety` | `std_msgs/msg/String` | 安全违规 JSON |
| `/vlm_topic` | `demo_msgs/msg/VlmDescription` | VLM 描述输出 |

### 10.2 Action

| Action | 类型 | 说明 |
|--------|------|------|
| `/rai/nav2/navigate_to_pose` | `NavigateToPose` | 导航到目标位姿 |
| `/rai/nav2/drive_on_heading` | `DriveOnHeading` | 沿当前航向行驶 |
| `/rai/nav2/follow_waypoints` | `FollowWaypoints` | 跟随路点序列 |
| `/rai/nav2/spin` | `Spin` | 原地旋转 |

### 10.3 Service

| Service | 类型 | 说明 |
|---------|------|------|
| `/rai/moveit2/move_arm` | `MoveArm` | 机械臂位姿控制 |
| `/rai/moveit2/set_arm_joints` | `SetArmJoints` | 设置关节角度 |
| `/spawn_entity` | `SpawnEntity` | 生成仿真实体 |
| `/get_entity_state` | `GetEntityState` | 获取实体状态 |
| `/global_costmap/get_costmap` | `GetCostmap` | 获取全局代价地图 |
