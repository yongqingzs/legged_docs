# D1 Max RobotSDK-0.2.1 自主充电分析

## 1. 结论

`RobotSDK-0.2.1/docs/zh/sdk_recharge_task_zh.md` 是当前 SDK 仓库中最完整的充电与离桩任务说明，内容与 0.2.1 的头文件和回调接口一致。但文档中仍有旧版本路径引用，因此应将其视为当前仓库版本的参考文档，不应据此断言为厂商永久意义上的最新文档。

RobotSDK-0.2.1 提供了充电任务控制接口，但 SDK 本身不是完整的自主回充导航系统。视觉识别、二维码定位、导航到充电桩和最终对接仍依赖机器人固件、导航主机和充电桩系统。

## 2. SDK 充电接口

```cpp
StartRechargeTask()
StopRechargeTask()
StartUnDockTask()
StopUnDockTask()
```

相关控制确认回调：

```cpp
OnStartRechargeTask()
OnStopRechargeTask()
OnStartUnDockTask()
OnStopUnDockTask()
```

任务状态通过以下回调提供：

```cpp
OnTaskStateData(const TaskStateInfo& info)
```

关键枚举：

```text
TaskType::RECHARGING  = 充电任务
TaskType::UNDOCK      = 离桩任务

TaskStatus::STARTING
TaskStatus::RUNNING
TaskStatus::SUCCESS
TaskStatus::FAILURE
TaskStatus::STOPPED

MachineStatus::RECHARGE = 充电机器状态
MachineStatus::UNDOCK   = 离桩机器状态
```

接口返回成功只表示命令发送或协议层确认成功，不代表完整任务已经完成。最终判定应结合控制 ACK、`TaskStateInfo` 和 `RobotState.machine_status`。

## 3. `recharge.cpp` 示例能力

`RobotSDK-0.2.1/example/recharge.cpp` 是充电与离桩接口的交互式 SDK 示例，覆盖：

```text
1 -> StartRechargeTask()
2 -> StopRechargeTask()
3 -> StartUnDockTask()
4 -> StopUnDockTask()
o -> 打印 RobotState
q -> 退出并断开连接
```

示例注册并打印：

- `OnStartRechargeTask()` 等四个控制确认回调；
- `OnTaskStateData()` 中的任务类型、状态、阶段和错误码；
- `RobotState.machine_status`；
- 充电桩连接状态和电池状态。

因此它适合做 SDK 接口、命令确认、任务状态和失败错误码的基础真机测试。

示例不负责：

- 视觉识别充电桩或二维码；
- 计算导航路径；
- 将机器人导航到充电桩前方；
- 视觉对接和充电触点控制；
- 任务超时、重试和导航级恢复；
- 连接导航主机 `192.168.168.100:10010`。

## 4. 示例使用方法

当前源码的命令行格式只有两个参数：

```bash
./recharge <ip> <port>
```

例如：

```bash
cd /home/jazzy/drive_ws/src/RobotSDK-0.2.1/example/build
./recharge 192.168.168.168 8081
```

但当前 `recharge.cpp` 只解析前两个参数。下面的命令中，后两个参数会被忽略：

```bash
./recharge 192.168.168.168 8081 192.168.168.100 10010
```

老版本文档中的四参数命令可能对应一个额外集成了导航主机服务的 Demo，不能直接推断当前 0.2.1 `recharge.cpp` 也实现了同样的功能。

当前 nav_bridge 使用过的 SDK 目标是 `192.168.168.168:8082`，而旧自主回充说明使用 `8081`。测试前必须确认实际固件端口，必要时分别尝试 `8081` 和 `8082`，不能仅根据旧文档决定端口。

## 5. 建议的真机测试项目

### 5.1 连接和状态

1. 确认没有其他 SDK 客户端或 nav_bridge 占用控制连接。
2. 启动 `recharge` 并确认连接成功。
3. 按 `o` 检查 `MotionStatus`、`MachineStatus`、充电桩连接状态和电池状态。

### 5.2 启动充电任务

在机器人满足充电桩前置条件后按 `1`，检查：

```text
StartRechargeTask() 返回/发送成功
收到 OnStartRechargeTask()
TaskType == RECHARGING
TaskStatus == STARTING 或 RUNNING
MachineStatus == RECHARGE
```

文档说明充电任务通常不会自然进入 `SUCCESS`，成功运行时可能长期保持 `RUNNING`；失败时重点记录 `error_code` 和 `phase`。

### 5.3 停止充电任务

按 `2`，检查：

```text
收到 OnStopRechargeTask()
TaskType == RECHARGING
TaskStatus == STOPPED
```

### 5.4 启动和停止离桩任务

机器人处于充电桩相关状态时按 `3`，检查：

```text
收到 OnStartUnDockTask()
TaskType == UNDOCK
TaskStatus == STARTING 或 RUNNING
MachineStatus == UNDOCK
```

正常离桩可能进入 `SUCCESS`，失败进入 `FAILURE` 并查看 `error_code`。中途按 `4` 停止时，检查 `UNDOCK + STOPPED` 和 `OnStopUnDockTask()`。

### 5.5 退出

按 `q`，确认示例执行 `Disconnect()` 并退出。测试异常中断后应检查并清理残留进程，避免下次连接出现控制拒绝。

## 6. 与 X30 充电流程的比较

X30 当前使用感知主机充电管理协议：

```text
检查 X30 状态
-> 退出 RL_MODE
-> 切换速度源到导航
-> 向感知主机发送 START/STOP/RESET/QUERY
-> 周期查询 charge_manager_state
```

D1 应使用 SDK 原生任务模型：

```text
检查连接和当前状态
-> 停止 /cmd_vel
-> 必要时退出特殊运动模式
-> StartRechargeTask()
-> 等待控制确认和 TaskStateInfo
-> 辅助检查 MachineStatus::RECHARGE
```

停止充电使用 `StopRechargeTask()`，离桩使用 `StartUnDockTask()`。不能直接复制 X30 的充电状态码，因为 X30 的 `charge_manager_state` 和 D1 的 `TaskStatus` 不是同一套枚举。

## 7. nav_bridge 适配建议

建议保留现有兼容接口：

```text
/nav_bridge_node/charge_command
/charge_manager_state
```

内部映射可以设计为：

```text
START -> StartRechargeTask()
STOP  -> StopRechargeTask()
QUERY -> 返回缓存的 TaskState/MachineStatus
```

`/charge_manager_state` 可发布 X30 兼容的导航语义，但必须在文档中说明它是 D1 SDK 任务状态的映射，而不是原生 X30 充电管理状态。充电动作期间应暂停 `/cmd_vel`，发送零速度，并在停止或离桩完成后恢复导航控制。

正式实现前还需要在真机确认：

- 导航主机充电视觉/二维码服务是否已运行；
- `StartRechargeTask()` 是否能触发完整无图回充；
- 是否确实需要额外的 `192.168.168.100:10010` 服务；
- `TaskStateInfo.phase` 和 `error_code` 是否能区分对接失败；
- 充电完成后任务是否长期 `RUNNING`；
- `StopRechargeTask()` 与 `StartUnDockTask()` 的实际动作边界。

