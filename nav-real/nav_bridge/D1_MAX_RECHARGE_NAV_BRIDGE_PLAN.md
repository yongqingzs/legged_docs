# D1 Max 自主充电 nav_bridge 适配方案

## 前提

当前假设 RobotSDK-0.2.1 的充电接口和 D1 机器人端服务正常工作，充电桩正在维修，暂不做完整对接实机验证。

启动充电前，由 nav_bridge 上层导航负责将机器人移动到充电桩前方，满足：

- D1 Max 正对充电桩；
- 头部距离充电桩约 1.5 m；
- 相机完整看到充电桩二维码。

nav_bridge 不负责重新规划到充电桩的路径，只负责在前置位置满足后启动和管理 D1 SDK 充电任务。

## SDK 依据

RobotSDK-0.2.1 提供：

```cpp
StartRechargeTask()
StopRechargeTask()
StartUnDockTask()
StopUnDockTask()
```

相关反馈：

```cpp
OnStartRechargeTask()
OnStopRechargeTask()
OnStartUnDockTask()
OnStopUnDockTask()
OnTaskStateData(const TaskStateInfo&)
```

辅助状态：

```cpp
MachineStatus::RECHARGE
MachineStatus::UNDOCK
```

`StartRechargeTask()` 返回成功只表示命令发送或协议层确认成功，不能表示已经完成回充。任务执行结果必须使用 `TaskStateInfo` 和 `MachineStatus` 判断。

## nav_bridge 对外接口

继续复用与 X30 一致的接口：

```text
/nav_bridge_node/charge_command
/charge_manager_state
```

建议命令映射：

```text
0 或 START         -> StartRechargeTask()
1 或 STOP          -> StopRechargeTask()
3 或 QUERY         -> 返回当前缓存充电状态
4 或 UNDOCK_START  -> StartUnDockTask()
5 或 UNDOCK_STOP   -> StopUnDockTask()
2 或 RESET         -> 暂不支持
```

不新增 D1 专用 ROS 服务，避免上层导航同时适配多套接口。

## 启动充电流程

```text
上层导航到达充电桩前置位
-> nav_bridge 检查 SDK 连接
-> 检查当前是否已有充电/离桩任务
-> 设置充电动作互斥标志
-> 停止 /cmd_vel 转发并发送零速度
-> 检查当前运动状态是否适合启动任务
-> TakeControl（必要时）
-> StartRechargeTask()
-> 等待 OnStartRechargeTask()
-> 等待 RECHARGING + STARTING/RUNNING
-> 辅助确认 MachineStatus::RECHARGE
```

充电任务执行期间：

- 禁止普通 `/cmd_vel` 转发；
- 继续发送零速度，防止上层残留速度造成运动；
- 禁止并发执行 `stand`、`lie`、`set_gait` 等动作；
- 持续发布 `/charge_manager_state`。

## 停止充电流程

```text
StopRechargeTask()
-> OnStopRechargeTask()
-> RECHARGING + STOPPED
-> MachineStatus 不再是 RECHARGE
-> 清除充电动作互斥标志
-> 恢复导航控制
```

不能仅依据 `StopRechargeTask()` 的返回值判断充电任务已停止。

## 离桩流程

```text
StartUnDockTask()
-> OnStartUnDockTask()
-> UNDOCK + STARTING/RUNNING
-> MachineStatus::UNDOCK
-> UNDOCK + SUCCESS 或 FAILURE
```

离桩期间同样禁止 `/cmd_vel`。成功后才恢复普通导航控制；失败时保持速度控制禁用并记录 `phase` 和 `error_code`。

## 状态缓存与兼容发布

D1 backend 应缓存：

```text
TaskType task_type
TaskStatus task_status
TaskStateInfo.phase
TaskStateInfo.error_code
MachineStatus machine_status
```

`/charge_manager_state` 采用 X30 兼容语义，但必须注明它是映射值：

```text
无任务/停止          -> IDLE
RECHARGING STARTING  -> 充电任务启动中
RECHARGING RUNNING   -> 充电任务执行中
MachineStatus RECHARGE -> CHARGING
RECHARGING FAILURE   -> 充电失败
RECHARGING STOPPED   -> IDLE
UNDOCK STARTING/RUNNING -> 离桩执行中
UNDOCK SUCCESS       -> IDLE
UNDOCK FAILURE       -> 离桩失败
```

D1 的 `TaskStatus` 与 X30 的充电状态码不是同一套枚举，不能直接声称数值完全等价。错误详情应通过日志或 `/robot_fault` 保留。

## 当前代码实现

当前 `D1MaxBackend` 已接入：

- `StartRechargeTask()`；
- `StopRechargeTask()`；
- `StartUnDockTask()`；
- `StopUnDockTask()`；
- `OnTaskStateData()` 缓存；
- `/charge_manager_state` 的兼容状态发布；
- 充电动作期间的 `/cmd_vel` 抑制。

当前 `RESET(2)` 暂不支持。完整的任务终态等待、ACK 独立记录和错误状态细分仍应在充电桩修复后结合真机反馈继续校准。

## 真机验证顺序

充电桩可用后，按以下顺序验证：

1. 确认没有其他 SDK 客户端占用运动主机连接；
2. 启动 nav_bridge 并确认 SDK 连接；
3. 由上层导航把机器人停到充电桩前置位；
4. 调用 `charge_command=START`；
5. 检查 ACK、`RECHARGING` 任务状态、`phase`、`error_code`；
6. 检查 `MachineStatus::RECHARGE` 和电池充电状态；
7. 调用 `charge_command=STOP`，确认 `STOPPED`；
8. 调用 `UNDOCK_START`，确认离桩任务 `SUCCESS` 或记录 `FAILURE`；
9. 确认任务结束后 `/cmd_vel` 恢复；
10. 退出并清理所有测试进程。

## 未确认事项

- 当前 D1 运动主机端口是 `8081` 还是 `8082`；
- 老版本命令中的 `192.168.168.100:10010` 是否仍是必需的导航服务；
- `StartRechargeTask()` 是否由机器人端自动协调二维码视觉和对接；
- 充电任务是否长期保持 `RUNNING`；
- `StopRechargeTask()` 与 `StartUnDockTask()` 的动作边界；
- `TaskStateInfo.phase` 和 `error_code` 的实际取值。

