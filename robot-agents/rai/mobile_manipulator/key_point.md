## 关键点

### agent_orchestrator.py — MegaMind 中央编排器

`agent_orchestrator.py` 是整个系统的**调度大脑**，使用 `create_megamind()` 构建 LangGraph 状态图，核心包含三大部分：

#### 1. AgentOrchestrator 类 — 双队列任务调度

- 维护两个异步任务队列：`low_prio_task_queue`（容量 50）和 `high_prio_task_queue`（容量 50）
- `orchestrator_loop()` 持续轮询，**优先执行高优先级任务**
- 任务来源：
  - **低优先级** — 订阅 `/user_tasks`，用户通过 HMI 下达的自然语言指令
  - **高优先级** — 订阅 `/inspection_result`，仿真环境检测到的异常（箱子/垃圾），自动生成移动或丢弃任务
- 通过 `agent.astream()` 异步执行任务，支持 checkpoint 断点恢复（`InMemorySaver`）

#### 2. MegaMind 图结构

MegaMind 编排器（`megamind_agent` LLM）负责**任务分解 → 选择 Executor → 分配工具 → 检查结果 → 判定成功/失败**，下辖三个 Executor 子智能体：

| Executor | 工具集 | 职责 |
|----------|--------|------|
| `housekeep` | `HouseKeepTool`, `SortReturnedPackageTool`, `InspectionWarehouseRouteTool`, `ThrowTrashOutTool` | 清扫、分拣、巡检路线、扔垃圾 |
| `package_movement` | `MoveFromCollectionToCollectionTool`, `MoveFromPoseToInspectionAreaTool`, `ThrowTrashOutTool` | 包裹搬运、移动至检测区 |
| `image_analysis` | `IsPackageDamagedTool`, `DescribeImageTool` | 包裹损伤判定、图像描述 |

#### 3. 辅助机制

- **TaskSubscriber** — ROS 2 消息回调 → 入队
- **emergency_stop_callback** — 监听 `/emergency_stop`，收到后立即取消任务、关闭线程、退出进程
- **AgentActionsCallback / OrchestratorTasksNotifier** — 将当前任务、队列状态、心跳发布到 ROS 2 topic，供 HMI 展示

#### 数据流

```
/user_tasks (ROS2)  ─┐
/inspection_result ──┤→ TaskSubscriber → [双队列] → orchestrator_loop()
                                                                    ↓
                                                          MegaMind 图 (megamind_agent LLM)
                                                                    ↓
                                                          选择 Executor → 调用 Tool
                                                                    ↓
                                        /rai/nav2/navigate_to_pose、/rai/moveit2/* 等 ROS2 Action
                                                                    ↓
                                                          Nav2Agent / MoveIt2Agent 执行
```

#### 启动方式

**注意：`run_ros2_stack.sh` 中未包含编排器启动命令**，需手动执行：
```bash
uv run python rai_app/agents/agent_orchestrator.py
```