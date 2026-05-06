# sem_nav_ctr —— 语义导航控制模块解析

> 对应启动脚本：`nav_agent/scripts/run_sem_nav.sh`  
> 源码目录：`nav_agent/sem_nav_ctr/src/`

---

## 1. 整体架构

`sem_nav_ctr` 是 HoloAgent 面向宇树 G1 人形机器人的**语义导航控制层**，以 ROS 2 为通信骨架，将"语音理解 → 场景图检索 → 目标位姿发布 → Nav2 规划 → 机器人运动控制"串联成完整流水线。模块由 4 个 ROS 2 节点（进程）组成，通过 tmux 多窗格并发启动。

```
┌──────────────────────────────────────────────────────────────┐
│                       sem_nav_ctr                            │
│                                                              │
│  ┌────────────────┐   chat_loc_pub   ┌─────────────────────┐│
│  │ DRobotCNode    │ ───────────────► │ GoalPosePublisher   ││
│  │ (chat_loc_     │                  │ (goal_publisher)    ││
│  │  python)       │ ◄─────────────── │                     ││
│  │                │  waypoint_reached│  HOV-SG 场景图查询   ││
│  │  语音采集      │                  │  → /object_pose     ││
│  │  WebSocket ASR │                  │  → /follow_waypoints││
│  │  LLM 对话      │                  └─────────────────────┘│
│  └────────────────┘                           │              │
│                                               │ Nav2         │
│                                       /cmd_vel│              │
│                                               ▼              │
│                        ┌─────────────────────────────────┐   │
│                        │ g1_getvel_node (getvel.cpp)      │   │
│                        │   订阅 /cmd_vel → 写 /tmp/vel_fifo│  │
│                        └───────────────┬─────────────────┘   │
│                                        │ FIFO (二进制)        │
│                                        ▼                      │
│                        ┌─────────────────────────────────┐   │
│                        │ g1_pubvel_node (pubvel.cpp)      │   │
│                        │   读 /tmp/vel_fifo → Unitree SDK  │   │
│                        │   → G1 LocoClient.Move()         │   │
│                        └─────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────┘
```

---

## 2. 启动流程（run_sem_nav.sh）

脚本在宿主机上创建名为 `robot_nav` 的 tmux 会话，在 4 个窗格中依次启动以下节点：

| 窗格 | 节点命令 | 功能 |
|------|---------|------|
| Pane 0 | `ros2 run chat_loc_python topic_chat_loc_pub` | 语音交互与意图解析 |
| Pane 1 | `ros2 run goal_publisher goal_pose_publisher` | 语义定位与目标发布 |
| Pane 2 | `ros2 run g1_move g1_getvel_node` | 读取 Nav2 速度指令写入管道 |
| Pane 3 | `ros2 run g1_move g1_pubvel_node` | 读取管道速度并驱动真实机器人 |

所有节点均需 source ROS 2 工作空间 `install/setup.bash` 并 `unset ASAN_OPTIONS`（避免内存检测工具干扰运行时）。

---

## 3. 各节点详解

### 3.1 `chat_loc_python` — 语音交互节点（`DRobotCNode`）

**源文件：** `chat_loc_python/node_chat_loc_class.py`、`drobotc_g1.py`

#### 功能
- 通过 **PyAudio** 从 ReSpeaker 麦克风阵列实时采集音频（16 kHz 单声道）。
- 经由 **WebSocket** 将音频流上传到远端语音服务（`ws://180.76.187.170:10071`），该服务完成以下工作：
  - 自动语音识别（ASR）：语音 → 文本
  - 大模型对话（LLM）：文本意图解析，返回结构化指令（目标地点/对象）
- 将服务端返回的结果按类型分发：
  - `loc` 类型 → 发布到 `/chat_loc_pub`（传给语义定位节点）
  - `signal` 类型 → 发布到 `/chat_signal_pub`（控制信号）
  - `qa` 类型 → 发布到 `/chat_qa_pub`（问答播报）
- 订阅 `/waypoint_reached`，将导航到达信号回传给远端服务，实现多轮对话与导航状态联动。

#### 关键类
- `DRobotC`：封装 WebSocket 双向通信、音频收发、线程管理。
- `DRobotCNode`：ROS 2 节点包装，定时（25 Hz）从消息队列取结果并发布。

---

### 3.2 `goal_publisher` — 语义定位节点（`GoalPosePublisher`）

**源文件：** `goal_publisher/goal_pose_publisher.py`  
**配置文件：** `config/visualize_query_graph_demo.yaml`

#### 功能
- 加载预构建的 **HOV-SG 层级场景图**（Hierarchical Open Vocabulary Scene Graph）。
- 订阅 `/chat_loc_pub` 接收自然语言目标（如"茶水间" / "沙发"）。
- 在场景图中执行多层级（楼层 → 房间 → 物体）语义检索，定位目标物体的三维坐标。
- 将检索到的目标位置转换到 LiDAR 地图坐标系，发布到 `/object_pose`。
- 通过 Nav2 的 `FollowWaypoints` Action 接口驱动机器人前往目标。
- 导航完成后发布 `/waypoint_reached` 信号。

#### 场景图初始化
```python
# 自动根据视觉 embedding 判断房间类型
hmsg.generate_room_names(
    generate_method="view_embedding",
    default_room_types=["Hallway", "Reception area", "Exhibition Hall", ...]
)
# 也支持人工指定
hmsg.set_room_names(room_names=designated_room_names_1127demo)
```

---

### 3.3 `g1_getvel_node` — 速度桥接节点（`getvel.cpp`）

**源文件：** `g1_move/src/getvel.cpp`

- 订阅 Nav2 规划输出的 `/cmd_vel`（`geometry_msgs/Twist`）。
- 提取线速度 (x, y) 和角速度 (z)，封装为二进制 `Vel` 结构体。
- 写入命名管道 `/tmp/vel_fifo`（由启动脚本用 `mkfifo` 预先创建）。

**设计意图：** 将 ROS 2 进程与宇树 SDK 进程解耦。ROS 域中的 Nav2 和 Unitree SDK 的运行时环境可能存在冲突，管道实现了跨进程的安全隔离。

---

### 3.4 `g1_pubvel_node` — 机器人驱动节点（`pubvel.cpp`）

**源文件：** `g1_move/src/pubvel.cpp`

- 读取 `/tmp/vel_fifo` 管道中的 `Vel` 结构体。
- 对速度值做**安全限幅**：
  - 纯旋转时：角速度绝对值不低于 0.3 rad/s（避免指令太小无效果）
  - 直线+旋转时：角速度绝对值死区为 0.1 rad/s，当角速度较大（> ±0.3）时线速度上限压至 0.22 m/s
- 调用 Unitree G1 SDK `LocoClient::Move(vx, vy, vr)` 实时控制机器人底盘。

---

## 4. 语义地图——有吗？如何体现？

**有语义地图。** 该模块依托 **HOV-SG（Hierarchical Open-Vocabulary Scene Graph）** 框架，以"场景图"形式组织三维语义信息，而非传统的占据栅格或点云。

### 4.1 语义表示的三层层级结构

```
Floor（楼层）
  └── Room（房间）  ← 视觉 embedding 自动分类或人工标注
        └── Object（物体）  ← 开放词汇 CLIP 特征 + 3D 点云
```

每一层节点均携带：
- **几何信息**：`open3d` 点云（`.pcd`），表示空间范围
- **语义信息**：CLIP 视觉-语言嵌入向量，支持任意自然语言描述的开放词汇检索

### 4.2 语义的具体承载形式

| 层级 | 语义信息来源 | 表示方式 |
|------|------------|---------|
| 物体（Object） | CLIP 图像特征（ViT-L/14） | 高维向量，与文本 query 计算余弦相似度 |
| 房间（Room） | 视图 embedding（`view_embedding`） | 自动对应预定义房间类型（"茶水间"、"会议室"等） |
| 楼层（Floor） | 拓扑关系 | 节点索引 |

场景图通过 FAST-LIVO2（LiDAR-惯性-视觉 SLAM）实时建图，再由 HOVSG 离线构建，最终以序列化文件存储（`graph_path` 指向的目录）。

### 4.3 语义查询流程

```
用户说出目标（自然语言）
        │
        ▼ LLM 解析
"去茶水间的咖啡机旁边"
        │
        ▼ hmsg.query_hierarchy_protected(query, ans, top_k=1)
  ① 楼层匹配（Floor）
  ② 房间检索（Room）—— query 与房间 CLIP embedding 计算相似度
  ③ 物体检索（Object）—— query 与物体 CLIP embedding 计算相似度
        │
        ▼ 得到 obj.pcd.get_center()
  物体三维坐标（场景图坐标系）
        │
        ▼ 坐标变换 T_tomap
  LiDAR 地图坐标 (x, y, z)
        │
        ▼ 发布 /object_pose → Nav2 → /cmd_vel
  机器人导航前往
```

分数过滤逻辑（余弦距离）：
- `object_query != 'unknown'` 且 `object_scores[0] < 0.15` → 判定为"找不到目标"
- `room_query == 'unknown'` 且 `object_query == 'unknown'` 且 `object_scores[0] < 0.18` → 忽略

---

## 5. 语义导航实现原理（完整链路）

```
┌─────────────────────────────────────────────────────────────────────┐
│ 离线建图阶段（部署前）                                                │
│                                                                     │
│  机器人巡游场景                                                       │
│    → FAST-LIVO2：LiDAR + IMU + Camera → 3D 点云地图 + 位姿            │
│    → HOVSG Pipeline：                                               │
│        ① 点云分割（Open3D / OVSeg）→ 提取物体实例点云                  │
│        ② CLIP 编码每个实例的多视图图像 → 语义向量                       │
│        ③ 按空间关系聚类成房间，生成场景图                               │
│        ④ 序列化保存（graph_path）                                     │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ 在线导航阶段（运行时）                                                │
│                                                                     │
│  用户语音                                                            │
│    → ASR（WebSocket 远端服务）→ 文本                                  │
│    → LLM（远端服务）→ 结构化目标描述                                   │
│    → /chat_loc_pub                                                  │
│                                                                     │
│  GoalPosePublisher                                                  │
│    → 加载场景图（HOV-SG Graph）                                       │
│    → CLIP 文本编码目标描述                                            │
│    → 层级检索（Floor → Room → Object）                               │
│    → 取 Object 点云中心坐标                                           │
│    → 坐标变换至 Nav2 地图坐标系                                        │
│    → 发布 /object_pose → Nav2 FollowWaypoints Action               │
│                                                                     │
│  Nav2 导航栈                                                         │
│    → GlobalPlanner：全局路径规划（基于 LiDAR 占据地图）                 │
│    → LocalPlanner：局部避障                                           │
│    → 输出 /cmd_vel（线速度 + 角速度）                                  │
│                                                                     │
│  g1_getvel_node                                                     │
│    → 拦截 /cmd_vel → 写入 /tmp/vel_fifo                              │
│                                                                     │
│  g1_pubvel_node                                                     │
│    → 读取 /tmp/vel_fifo                                              │
│    → 安全限幅处理                                                     │
│    → 调用 Unitree G1 SDK LocoClient::Move()                         │
│    → 机器人实际运动                                                   │
└─────────────────────────────────────────────────────────────────────┘
```

### 核心技术要点

1. **开放词汇检索（Open-Vocabulary）**：场景图中的物体和房间均以 CLIP 嵌入表示，可直接与任意自然语言描述进行零样本匹配，无需预定义物体类别枚举。

2. **层级过滤提升精度**：先缩小到楼层→房间，再在小范围内匹配物体，避免全局搜索带来的歧义和噪声。

3. **跨进程隔离（命名管道）**：ROS 2 节点（Nav2 输出）与宇树 Unitree SDK（运动控制）运行在不同进程，通过 `/tmp/vel_fifo` 进行二进制通信，规避动态库冲突。

4. **双坐标系管理**：场景图以 FAST-LIVO2 SLAM 的传感器坐标系存储，导航时通过固定的轴变换矩阵 `T_tomap`（绕轴置换）转换至 Nav2 地图坐标系。

5. **导航状态回路**：`waypoint_reached` 话题使语音对话服务感知导航完成状态，支持"到达后继续指令"的多轮交互。

---

## 6. 依赖关系

| 依赖 | 用途 |
|------|------|
| **FAST-LIVO2** | LiDAR-惯性-视觉 SLAM，离线建立点云地图 |
| **HOVSG (HOV-SG)** | 层级开放词汇场景图构建与查询（`hmsg` 包） |
| **CLIP (ViT-L/14)** | 视觉-语言嵌入，开放词汇语义匹配 |
| **Nav2** | ROS 2 导航栈，路径规划与局部避障 |
| **Unitree G1 SDK** | 宇树 G1 机器人运动控制底层接口 |
| **DRobotC 后端** | 远端 ASR + LLM 对话服务（WebSocket） |
| **Open3D** | 点云处理与可视化 |
| **PyAudio** | 音频采集与播放 |
