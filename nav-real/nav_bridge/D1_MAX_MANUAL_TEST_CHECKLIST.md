# D1 Max nav_bridge 手动测试清单

本文用于在 D1 Max 导航主机上启动 `nav_bridge` 后，手动验证 SDK 连接、ROS 话题和控制接口。姿态动作及非零速度测试必须在机器狗周围无障碍、急停可用并有现场人员看护时执行。

## 1. 启动节点

在导航主机终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/Workspace/driver_ws/install/setup.bash
ros2 launch nav_bridge nav_bridge.launch.py
```

启动日志应包含：

```text
D1 Max backend connected to 192.168.168.168:8082
```

另开终端并加载相同环境：

```bash
source /opt/ros/humble/setup.bash
source ~/Workspace/driver_ws/install/setup.bash
```

## 2. 检查节点、话题和服务

```bash
ros2 node list
ros2 node info /nav_bridge_node
ros2 topic list
ros2 service list | grep nav_bridge
```

预期主要话题：

```text
/cmd_vel
/imu/data
/leg_odom
/joint_states
/battery/level
/robot_basic_state
/robot_gait_state
/robot_body_height_state
/charge_manager_state
/robot_fault
```

预期服务：

```text
/nav_bridge_node/stand
/nav_bridge_node/lie
/nav_bridge_node/soft_estop
/nav_bridge_node/release_control
/nav_bridge_node/set_gait
/nav_bridge_node/set_speed
/nav_bridge_node/set_body_height
/nav_bridge_node/charge_command
```

## 3. 检查传感器数据

分别确认各话题能收到一条真实数据：

```bash
ros2 topic echo /imu/data --once
ros2 topic echo /leg_odom --once
ros2 topic echo /joint_states --once
ros2 topic echo /battery/level --once
ros2 topic echo /robot_basic_state --once
ros2 topic echo /robot_gait_state --once
ros2 topic echo /robot_body_height_state --once
ros2 topic echo /charge_manager_state --once
ros2 topic echo /robot_fault --once
```

检查主要数据频率，按 `Ctrl-C` 停止：

```bash
ros2 topic hz /imu/data
ros2 topic hz /leg_odom
ros2 topic hz /joint_states
```

默认 `imu_source` 为 `imu_driver`，因此 `/imu/data` 应转发导航主机的
`/imu_driver/imu_central`，预期约为 200 Hz。检查转发链路和 QoS：

```bash
ros2 topic info /imu_driver/imu_central -v
ros2 topic info /imu/data -v
ros2 topic hz /imu_driver/imu_central
ros2 topic hz /imu/data
```

需要验证 SDK 备用源时，将参数改为 `imu_source:=sdk`；此时 `/imu/data`
来自 RobotSDK `OnImuData` 回调，实际频率通常低于本机 IMU 驱动。

`joint_states` 应包含 D1 Max 的 16 个关节；电池话题应为 0 到 100 的整数百分比。
`charge_manager_state` 固定为 `-1`，表示 D1 暂不支持充电管理。故障话题没有故障时可能没有消息。

状态话题含义：

```text
/robot_basic_state        D1 MotionStatus 的统一状态映射
/robot_gait_state         当前兼容 gait 值（WALK=0、MOUNTAIN=33 等）
/robot_body_height_state  -1=Crawl/CrawlWalk，0=Normal/未知
/charge_manager_state     -1=D1 不支持充电
```

## 4. 检查网络和 SDK 目标

导航主机到运动主机应使用共享网段地址，而不是无线/p2p 地址：

```bash
ip route get 192.168.168.168
ping -c 3 192.168.168.168
```

当前目标参数为：

```text
d1_host_ip: 192.168.168.168
d1_host_port: 8082
```

如果现场必须通过 `192.168.234.1` 访问，应先确认 UDP 回程路由，而不能只根据 SSH 跳转成功判断 SDK 网络正常。

## 5. 测试速度档位

该服务只设置速度档位，不会直接使机器狗移动：

```bash
ros2 service call /nav_bridge_node/set_speed \
  rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: speed, value: {type: 2, integer_value: 1}}]}"
```

档位含义：

```text
1 = SLOW
2 = MEDIUM
3 = HIGH
```

建议首次测试使用 `1`。

## 6. 测试导航步态接口

```bash
ros2 service call /nav_bridge_node/set_gait \
  rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: gait, value: {type: 2, integer_value: 33}}]}"
```

当前 D1 适配使用与 X30 一致的 `set_gait` 导航语义：

```text
WALK(0)、L_WALK(32)       -> Gait() + SLOW
MOUNTAIN(33)、SILENT(34)  -> Gait() + MEDIUM
RUN(3)                    -> Gait() + HIGH
STAIR_SOLID(6)、STAIR_ACC(7)、STAIR45_ACC(8)、L_STAIR(36)
                            -> Stair()
```

也可以使用字符串参数，例如 `"MOUNTAIN"`、`"RUN"` 或 `"STAIR"`。D1 没有独立的 Crawl gait 码；`Crawl()` 是后端姿态动作，不作为 `set_gait` 参数暴露。

## 7. 零速度控制链路

此命令会触发 SDK 控制权申请，但速度为零，不应产生运动：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

观察状态和故障：

```bash
ros2 topic echo /robot_basic_state
ros2 topic echo /robot_fault
```

在确认控制权前，不要发送非零速度。

## 8. 通用模式键盘控制测试

切换到 D1 通用模式后，可以使用 ROS2 `teleop_twist_keyboard` 验证
`/cmd_vel` 是否被 nav_bridge 正确接收并转发。该测试会使机器狗实际运动，
必须确认周围无障碍、有人看护并随时准备释放键盘或触发急停。

### 8.1 准备环境

在导航主机的终端 A 启动 nav_bridge，并在终端 B 加载相同环境：

```bash
source /opt/ros/humble/setup.bash
source ~/Workspace/driver_ws/install/setup.bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

确认当前模式和状态：

```bash
ros2 topic echo /robot_basic_state --once
ros2 topic echo /robot_gait_state --once
```

### 8.2 切换通用模式并站立

建议先使用中速通用模式 `MOUNTAIN(33)`：

```bash
ros2 service call /nav_bridge_node/set_gait \
  rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: gait, value: {type: 4, string_value: 'MOUNTAIN'}}]}"
```

如果机器狗尚未站立，在安全看护下执行：

```bash
ros2 service call /nav_bridge_node/stand std_srvs/srv/Trigger "{}"
```

确认 `/robot_gait_state` 为 `33`，并且 `/robot_basic_state` 为可运动状态后再继续。

### 8.3 启动键盘节点

如果系统尚未安装：

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

启动键盘控制：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

常用按键（以键盘节点启动后显示的帮助为准）：

```text
i / , / o / u / . / m    前进、后退及斜向移动
j / l                    左转、右转
k                        发送零速度
q / z                    提高/降低速度档位
CTRL-C                   退出键盘节点
```

只按住一个方向键，观察机器狗是否按预期运动；松开按键后键盘节点会继续发布
零速度或停止指令。也可以在终端 C 观察转发链路：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /robot_basic_state
ros2 topic echo /robot_gait_state
```

验证要点：

- `MOUNTAIN` 模式下，`/cmd_vel` 的 `linear.x` 控制前后，`linear.y` 控制左右，`angular.z` 控制转向；
- nav_bridge 以 `cmd_vel_rate_hz`（默认 50 Hz）定频调用 SDK `Move()`；
- 未持有控制权时，收到 `/cmd_vel` 会先申请 `TakeControl()`；
- 当前状态不允许运动、动作服务执行中或指令超过 `cmd_vel_timeout_ms`（默认 500 ms）时，nav_bridge 会发送零速度；
- 仅看到 `/cmd_vel` 消息而机器狗不动，需检查 `/robot_basic_state`、控制权和 SDK 连接日志，不要连续提高速度。

### 8.4 结束测试

先按 `k` 发送零速度，再按 `CTRL-C` 退出键盘节点；确认机器狗停止后释放控制权：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 service call /nav_bridge_node/release_control std_srvs/srv/Trigger "{}"
```

如果运动方向与预期相反，立即发送零速度并触发软急停，然后记录
`/cmd_vel`、D1 模式和实际运动方向；不要在现场直接修改坐标变换。

## 9. 姿态动作测试（高风险）

以下命令会实际改变机器狗姿态，只能在安全看护条件下执行：

站立：

```bash
ros2 service call /nav_bridge_node/stand std_srvs/srv/Trigger "{}"
```

趴下：

```bash
ros2 service call /nav_bridge_node/lie std_srvs/srv/Trigger "{}"
```

软急停：

```bash
ros2 service call /nav_bridge_node/soft_estop std_srvs/srv/Trigger "{}"
```

释放控制权：

```bash
ros2 service call /nav_bridge_node/release_control std_srvs/srv/Trigger "{}"
```

以下接口目前保留用于兼容 X30，但 D1 会明确返回不支持，不会执行动作：

```bash
ros2 service call /nav_bridge_node/set_body_height \
  rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: body_height, value: {type: 4, string_value: 'NORMAL'}}]}"

ros2 service call /nav_bridge_node/charge_command \
  rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: charge_command, value: {type: 2, integer_value: 0}}]}"
```

## 10. 推荐现场顺序

1. 检查节点、话题和服务是否存在。
2. 检查 IMU、里程计、关节和电池数据。
3. 检查网络路由及 SDK 连接日志。
4. 设置低速档 `speed=1`。
5. 发送一次零速度并确认控制链路。
6. 在安全看护下测试 `stand`。
7. 确认机器狗状态后，再低速发送非零 `/cmd_vel`。
8. 测试结束发送零速度并调用 `release_control`。

## 11. 当前已知限制

- 未经安全看护不得执行 `stand`、`lie` 或非零 `/cmd_vel`。
- D1 的 `set_body_height` 和 `charge_command` 当前仅返回不支持。
- `/charge_manager_state` 固定发布 `-1`，不能用于判断真实充电状态。
- `body_height_state` 只能根据 `Crawl/CrawlWalk` 推断，RobotSDK 的 `HighLowStance` 反馈尚未接入。
- `/robot_basic_state`、`/robot_gait_state`、`/robot_body_height_state`、`/charge_manager_state` 即使未持有控制权也会持续发布。
- `/cmd_vel` 只有在持有控制权、动作未被抑制且 MotionStatus 允许时才转发；超时后持续发送零速度。
- `imu_source` 默认是 `imu_driver`，SDK 源仅作为备用选项。
- 现场测试应记录 SDK 连接日志、各话题频率、速度方向和控制权释放结果。

## 12. IMU 频率排查说明

默认配置下 `/imu_driver/imu_central` 是输入，`/imu/data` 是 nav_bridge 转发输出；只有设置 `imu_source=sdk` 时才使用 RobotSDK 链路。

| 话题 | 来源 | 数据链路 | 配置频率 |
| --- | --- | --- | --- |
| `/imu_driver/imu_central` | 系统 `imu_driver` 节点 | 导航主机本地 SPI（`/dev/spidev0.0`、`/dev/spidev0.1`） | `imu_params.yaml` 中为 200 Hz |
| `/imu/data`（默认） | `nav_bridge_node` | 转发 `/imu_driver/imu_central` | 导航主机 IMU 配置约 200 Hz |
| `/imu/data`（`imu_source=sdk`） | `nav_bridge_node` | D1 RobotSDK-0.2.1 UDP `OnImuData` 回调 | `D1MaxBackend` 调用 `SetImuConfig(100)` |

确认 `/imu_driver/imu_central` 来源：

```bash
ps -ef | grep '[i]mu_driver'
cat /opt/robot/robot-sensors/install/imu_driver/share/imu_driver/config/imu_params.yaml
```

导航系统由运行时以 `ROS_DOMAIN_ID=24`、`RMW_IMPLEMENTATION=rmw_zenoh_cpp` 启动。使用其他 Domain 或 RMW 检查时，可能看不到该话题：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic info /imu_driver/imu_central -v
ros2 topic hz /imu_driver/imu_central
ros2 topic info /imu/data -v
ros2 topic hz /imu/data
```

RobotSDK-0.2.1 文档规定 `SetImuConfig` 的频率范围为 `[0, 100]`，因此系统 SPI IMU 的 200 Hz 不是 D1 SDK 可直接复用的配置值。当前实测 SDK `/imu/data` 约 10--18 Hz，说明 D1 机器人经 SDK 上报的实际频率低于请求值，需进一步确认机器人固件对 IMU 配置的实现或 UDP 数据发送策略；不能通过 ROS 话题重新发布来恢复不存在的 SDK 样本。后续应在 SDK 连接成功且无重复客户端时，记录 `OnImuConfig` 配置回调和 SDK 原始回调计数，再判断是机器人端限频还是 SDK 接收端问题。

如果启动日志出现 `Robot Controlled denial of service`，先检查是否有多个 nav_bridge/SDK 客户端连接运动主机；RobotSDK 文档明确多客户端会触发控制拒绝。清理重复进程后再测频率，避免把连接冲突误判为 IMU 丢帧。

### 12.1 SDK 示例对照结果

RobotSDK-0.2.1 官方 `example/data.cpp` 在导航主机使用同一 arm64 库、同一目标 `192.168.168.168:8082` 测试时，`SetImuConfig(200)` 虽然会被 SDK/机器人端限制在合法范围内，但在开启 IMU 的约 3 秒阶段实际收到约 246 次 `OnImuData` 回调，约 80 Hz。该结果说明机器人端和 SDK 接收线程能够提供远高于 13 Hz 的数据，`nav_bridge` 的数据转换不是天然只能达到 13 Hz。

`ros2 topic hz` 的输出是“当前 CLI 订阅端收到的消息频率”，不是发布者回调频率；ROS 2 命令本身也提示该数值会受订阅端资源和 QoS 影响。使用 `imu_source=sdk` 时，D1 节点将每个 SDK 回调发布到 `/imu/data`；在导航主机的 `rmw_zenoh_cpp` 环境中，可靠传输、Zenoh 调度或 Python `ros2 topic hz` 订阅端均可能影响观测值。默认 `imu_source=imu_driver` 时，应重点比较本机 IMU 输入和转发输出：

```bash
ros2 topic info /imu/data -v
ros2 topic hz /imu/data
```

测试时还要保证只有一个 `d1_max_nav_bridge_node` 进程，否则多个 SDK 客户端会触发 `Controlled denial`，造成连接或数据状态异常。当前结论是：13 Hz 首先应视为 ROS/Zenoh 订阅观测值或发布端可靠 QoS 背压，不能据此断定 RobotSDK 仅上报 13 Hz；官方示例回调计数证明 SDK 链路实际可达到约 80 Hz。SDK 文档规定 IMU 请求频率上限为 100 Hz，若要确认是否能稳定达到 100 Hz，还需用轻量回调计数程序连续测量，而不是依赖打印型示例或 `ros2 topic hz` 单一结果。

### 12.2 最新复测：IMU 配置未生效

本次在导航主机清理重复 `nav_bridge` 后重新测试，节点日志显示 SDK 连接成功，但 `/imu/data` 在测量窗口内没有消息。随后停止 `nav_bridge`，单独运行同一份 RobotSDK-0.2.1 arm64 官方 `data` 示例，示例同样显示连接成功但没有 `OnImuData` 回调。因此本轮故障不是 ROS 消息转换或 `ros2 topic hz` 单独造成的，更像是机器人端 IMU 上报配置未生效或服务端传感器订阅状态异常。

代码侧需重点核查：`D1MaxBackend::connect()` 在 `Connect(..., true)` 返回后立即异步调用 `SetImuConfig(100)`，没有等待发送结果，也没有实现 `IControlCallback::OnImuConfig` 确认机器人是否接受配置；官方示例则是在连接完成回调成功后才开始传感器配置。若配置命令在握手完成后的短窗口内被丢弃，节点仍会打印 `connected`，但不会有 IMU 数据。后续修复应采用同步发送或回调确认、记录错误码，并在配置失败时重试。

### 12.3 控制锁恢复后的最终实测

控制锁释放后，使用修复版导航主机二进制单客户端运行，SDK 输出：

```text
D1 Max backend connected to 192.168.168.168:8082
[D1 SDK] IMU callback count=100 elapsed=4.95 s rate=20.19 Hz
[D1 SDK] IMU callback count=200 elapsed=11.75 s rate=17.02 Hz
[D1 SDK] IMU callback count=300 elapsed=19.10 s rate=15.71 Hz
```

同时收到：

```text
[D1 SDK] IMU configuration acknowledged: 1 Hz
```

这里的 `1` 是 RobotSDK 的配置确认状态值，表示 IMU 上报已启用，并非实际数据频率；请求值 100 Hz 也不会在该回调中原样返回。该次测量直接统计 `OnImuData` 回调，因此可确认低频已经发生在运动主机/RobotSDK 数据链路，非 ROS `publish()` 或 `ros2 topic hz` 订阅端造成。导航系统的 `/imu/data` 通常会观测到约 15--20 Hz，与 SDK 原始回调一致；本地 SPI 驱动 `/imu_driver/imu_central` 的 200 Hz 是另一颗/另一条 IMU 数据链路，不能作为 D1 SDK 的目标频率。

RobotSDK-0.2.1 文档仅规定 `SetImuConfig` 请求参数范围 `[0, 100]`，没有承诺机器人一定按请求值发送 100 Hz。当前实机表现说明 D1 运动主机固件或协议服务对 SDK IMU 上报存在约 15--20 Hz 的实际限频（或按 UDP 报文周期发送）。若导航算法必须使用 200 Hz，应直接使用导航主机的 `/imu_driver/imu_central`，并另行完成坐标系/时间戳校准，而不是复制 SDK 消息来“补频率”。
