# 智能体与工具参考

## 1. MegaMind 编排器 (Agent Orchestrator)

**入口:** `rai_app/agents/agent_orchestrator.py`

核心类 `AgentOrchestrator` 负责：
- 通过 ROS 2 话题接收任务 (`/user_tasks`, `/inspection_result`)
- 维护双优先级任务队列（低优先级 / 高优先级）
- 通过 `orchestrator_loop()` 轮询选择任务并提交给 LangGraph MegaMind
- 背景线程定期发布任务状态 (`task_notifier`, `heartbeat_notifier`)
- 响应 `/emergency_stop` 紧急停止信号

### 任务优先级

| 来源 | 优先级 | 说明 |
|------|--------|------|
| `/user_tasks` | 低 | HMI 或外部系统发起的常规任务 |
| `/inspection_result` | 高 | 巡检智能体发现的异常（box/trash） |
| housekeep 任务 | 高 | 仓库整理任务 |

---

## 2. Executor 执行器

通过 LangGraph `create_megamind()` 构建，MegaMind 作为"超级大脑"将任务分派给专用执行器。

### 2.1 housekeep 执行器

**系统提示:** `HOUSEKEEP_EXECUTOR_SYSTEM_PROMPT`

负责仓库整理、退货分拣、巡检路线、垃圾清理。

| 工具 | 功能 |
|------|------|
| `do_housekeeping` | 对指定货架做整理，校正放歪的盒子 |
| `sort_returned_package` | 自动分拣退货区包件（VLM 检测损坏 → 决定目标位置） |
| `route_around_warehouse` | 沿预设巡检路线巡逻 |
| `throw_out_trash` | 将指定位置的垃圾丢入垃圾桶 |

### 2.2 package_movement 执行器

**系统提示:** `MOVEMENT_EXECUTOR_SYSTEM_PROMPT`（注入仓库上下文）

负责包件在货架/桌子间的搬运。

| 工具 | 功能 |
|------|------|
| `move_object_between_collections` | 从源集合移动物品到目标集合，可按物品类型筛选 |
| `move_object_from_pose_to_inspection_area` | 从自由位姿拾取物品放入检验区 |
| `throw_out_trash` | 丢弃指定位置的垃圾 |

### 2.3 image_analysis 执行器

**系统提示:** `IMAGE_ANALYSIS_EXECUTOR_SYSTEM_PROMPT`

负责图像分析和包件损坏检测。

| 工具 | 功能 |
|------|------|
| `is_package_damaged_tool` | 通过 VLM 判断当前视角下的包件是否损坏 |
| `describe_image` | 用 VLM 详细描述手腕摄像头拍摄的图像 |

---

## 3. 工具详细接口

### WarehouseTool (基类)

所有仓库工具继承自 `WarehouseTool`，持有：
- `kairos_controller: KairosController` — 机器人控制接口
- `scene_manager: SceneManager` — 场景管理接口

核心方法：
- `check_the_origin_collection()` — 检查源集合，找到可用物品槽位
- `check_the_target_collection()` — 检查目标集合，找到空闲槽位
- `approach_and_filter_collection()` — 机器人接近集合，过滤机械臂可达的槽位
- `refresh_data()` — 刷新实体-槽位映射

### 工具列表

```
MoveFromCollectionToCollectionTool  --  集合间物品搬运
    输入: origin_collection_name, target_collection_name, item_type(可选)
    流程:
    1. 检查目标集合有空位
    2. 检查源集合有物品(可按item_type过滤)
    3. KairosController.move_object_to_slot()

HouseKeepTool  --  货架整理
    输入: rack (如 "A01")
    流程:
    1. 从两面接近货架
    2. 对每个可达槽位中的物品检查 yaw 偏差
    3. 偏差 > 0.35 rad 则调用 align_object_with_slot()

SortReturnedPackageTool  --  退货自动分拣
    输入: 无(自动解析)
    流程:
    1. 从退货桌(t1)取一个包件
    2. 调用 IsPackageDamagedTool 检查损坏
    3. 损坏 → 移到检验桌(t4)
    4. 完好 → 从实体名提取item_type → 移到对应货架

InspectionWarehouseRouteTool  --  仓库巡检路线
    输入: 无
    流程:
    1. 读取 warehouse_route.csv 中的 waypoints
    2. 调用 follow_waypoints action

ThrowTrashOutTool  --  丢垃圾
    输入: x, y, z (垃圾位置)
    流程:
    1. 接近垃圾
    2. 抓取 → 导航到垃圾箱 → 投放

MoveFromPoseToInspectionAreaTool  --  从自由位姿到检验区
    输入: x, y, z (物体位置)
    流程:
    1. 在检验区集合(t4)找空位
    2. 接近并拾取 → 放置到检验区槽位

IsPackageDamagedTool  --  包件损坏检测
    输入: 无(使用当前手腕摄像头)
    流程:
    1. 获取手腕摄像头图像
    2. VLM 描述性分析 (BOX_CONDITION_SYSTEM_PROMPT)
    3. VLM 结构化输出 (BoxConditionOutput)
    4. 输出: True(损坏) / False(完好)

DescribeImageTool  --  图像描述
    输入: prompt (描述提示词)
    流程:
    1. 获取手腕摄像头图像
    2. VLM.invoke([system, multimodal_human])
    3. 返回 VLM 描述文本
```

---

## 4. 独立智能体

### Inspection Agent (巡检智能体)

**入口:** `rai_app/agents/inspection_agent.py`
**类:** `VlmWarehouseInspector`

独立运行的 VLM 智能体，不经过 MegaMind 编排器。

工作流程：
1. 订阅摄像头话题 `/rgbd_camera/camera_image_color`
2. 通过 `get_anomaly_box_pose()` 从模拟环境获取异常物体位姿
3. 将任务放入 `process_queue`
4. 后台 `vlm_worker` 线程按间隔(n_seconds)处理
5. VLM 三阶段分析：描述 → 结构化检测 → 分类
6. 去重（60s TTL，基于位姿距离和角度）
7. 发布 `Anomaly` 消息到 `/inspection_result`

异常分类输出 (`AnomalyDescription`):
- `box` → 盒子，需移至检验区
- `trash` → 垃圾，需丢入垃圾箱
- `other` → 其他异常，无法自动处理
- `nothing` → 无异常

### Safety Agent (安全合规智能体)

**入口:** `rai_app/agents/safety_agent.py`
**类:** `SafetyAgent`

基于 RAG 的仓库安全合规检测。

工作流程：
1. 订阅摄像头话题
2. SSIM 图像去重（跳过相似帧）
3. 调用 `ImageRegAgent`（三阶段 LangGraph StateGraph）
4. 发布违规 JSON 到 `/safety`
5. 持久化到 `ViolationStorage`

`ImageRegAgent` 三阶段图(`rai_app/warehouse_regulations_agent/warehouse_safety_agent.py`):

```
START → vision → retrieve → final
```

- **vision 节点**: VLM 图像描述 + 异常提取 → `VisionObervation`
- **retrieve 节点**: 对每个异常做 FAISS 相似度检索 → Reranker 重排序 → 取 top-3
- **final 节点**: VLM 合规判定 → `FinalImageSafetyOutput`(hazard, applicable_regulations, severity, rationale)

---

## 5. 回调系统

### AgentProgressCallback
- 监听 LLM 调用开始/结束，计算耗时
- 监听 chain 结束，提取 `steps_done` 发布到 `/agent/past_steps`

### AgentActionsCallback
- 处理 `agent.astream()` 流式输出的 chunk
- 提取 reasoning 内容、文本内容、tool_calls
- 发布到 `/agent/current_action` (ROS 2 HRIMessage)

### OrchestratorTasksNotifier
- 发布当前任务到 `/orchestrator/current_task`
- 发布队列状态到 `/orchestrator/tasks_queue`
- 发布心跳到 `/orchestrator/heartbeat`

---

## 6. 上下文提供者

`WarehouseContext(ContextProvider)` 为 executor 提供仓库上下文信息：
- 各桌子的用途（退货桌 t1、出货桌 t2、检验桌 t4）
- 各货架存储的物品类型
- 通过 `scene_manager.get_warehouse_collections_description()` 动态生成
