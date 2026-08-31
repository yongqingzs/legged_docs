# nav_bridge 已完成功能

本文档依据 `/home/jazzy/drive_ws/src/nav_bridge` 当前源码整理，描述 `nav_bridge_node` 已实现的功能、ROS 2 接口、控制流程和运行边界。

## 1. 功能定位

`nav_bridge` 是绝影 X30 机器狗的 ROS 2 与 UDP 协议桥接节点，使用 C++17 实现。节点负责：

- 将 ROS 2 标准速度指令转换成 X30 模拟手柄轴指令；
- 将 X30 运动主机反馈解析为 IMU、足式里程计、状态、电池等 ROS 2 数据；
- 提供起立、趴下、急停、步态、高度和控制权管理服务；
- 通过感知主机 UDP 接口管理自主充电，并发布充电状态。

## 2. 已实现的核心能力

### 2.1 UDP 通信与协议封装

- `UdpTransport` 封装 Linux UDP socket，支持默认目标发送、指定目标发送、本地端口绑定和带超时接收。
- 运动主机默认地址：`192.168.1.103:43893`，本地接收端口：`43897`。
- 感知主机默认地址：`192.168.1.105`，充电请求端口：`3333`，配置端口：`43899`，本地端口：`49004`。
- `x30_protocol.hpp` 定义指令头、状态报文、IMU/关节/电池结构体、状态枚举、步态速度上限和速度到轴值的映射函数。

### 2.2 控制权、心跳与安全超时

- 节点启动时可由 `startup_acquire_control` 自动接管控制权，默认开启。
- 控制权持有期间按 `heartbeat_interval_ms` 周期发送 `CMD_HEARTBEAT`，首次会话发送 `CMD_QUERY_103` 确认连接。
- `/cmd_vel` 回调只缓存最新速度并刷新输入时间，实际 UDP 下发由统一定时器执行，默认 50 Hz。
- `/cmd_vel` 超过 `cmd_vel_timeout_ms`（默认 5 s）后持续发送零速度，但不会自动释放控制权。
- 只有 `~/release_control` 会停止心跳并显式交还控制权。
- 动作服务执行期间临时屏蔽 `/cmd_vel`，避免动作命令和轴速度同时控制。
- 节点关闭时尝试发送三轴零速度、停止接收线程并关闭 socket。

### 2.3 速度控制

订阅 `/cmd_vel`（`geometry_msgs/msg/Twist`），支持 `vx`、`vy`、`vyaw` 三个自由度。

- 仅在 `STEPPING + 受支持步态` 或 `RL_MODE + RL 类步态` 状态下转发速度；
- 根据当前步态速度上限分别限幅前进、后退、侧移和偏航速度；
- 映射到 X30 轴值 `[-32767, 32767]`；
- 小于底层死区 `655` 的轴值归零；
- 处理 ROS 坐标系与 X30 摇杆正负方向差异。

### 2.4 起立与趴下动作

`ActionExecutor` 集中实现动作状态机，采用状态反馈等待、控制预热和 toggle 指令重发机制。

- `~/stand`：将机器人收敛到 `RL_MODE + stand_target_gait`，默认目标步态为 `MOUNTAIN(33)`。典型路径为：软急停/趴下 -> 起立 -> 力控站立 -> 踏步 -> 目标 RL 步态。
- `~/lie`：根据当前状态安全回到 `LYING_DOWN`。支持从 RL、踏步、力控站立、初始站立、进行中趴下和软急停等状态恢复；必要时停止运动、等待站立稳定并重发趴下指令。
- 对 `CMD_STAND_UP_DOWN`、`CMD_MOTION` 等 toggle 指令提供持续接管窗口和固定间隔重试。

### 2.5 步态切换

`~/set_gait` 使用 `rcl_interfaces/srv/SetParameters`，请求必须恰好包含一个名为 `gait` 的参数，支持整数或大小写不敏感的字符串：

| 值 | 名称 | 最终基本状态 |
|---:|---|---|
| 0 | `WALK` | `STEPPING` |
| 1 | `OBSTACLE` | `STEPPING` |
| 2 | `SLOPE` | `STEPPING` |
| 3 | `RUN` | `STEPPING` |
| 6 | `STAIR_SOLID` | `STEPPING` |
| 7 | `STAIR_ACC` | `STEPPING` |
| 8 | `STAIR45_ACC` | `STEPPING` |
| 32 | `L_WALK` | `RL_MODE` |
| 33 | `MOUNTAIN` | `RL_MODE` |
| 34 | `SILENT` | `RL_MODE` |
| 36 | `L_STAIR` | `RL_MODE` |

服务允许从 `STEPPING`、`RL_MODE` 或 `FORCE_STAND` 发起；从力控站立开始时会先重试 `CMD_MOTION` 进入踏步，再发送目标步态，并等待目标基本状态和步态状态同时成立。匍匐高度下只允许 `WALK` 或 `SLOPE`。

### 2.6 机体高度切换

`~/set_body_height` 使用 `rcl_interfaces/srv/SetParameters`，参数名为 `body_height`：

- `0` 或 `CRAWL`：匍匐；
- `2` 或 `NORMAL`：正常。

服务会检查连接状态、当前步态和目标状态；RL 类步态（`L_WALK`、`MOUNTAIN`、`SILENT`、`L_STAIR`）不允许切换机体高度，目标为匍匐时当前步态必须是 `WALK` 或 `SLOPE`。指令发送后等待 `/robot_body_height_state` 反馈确认。

### 2.7 软急停与控制权释放

- `~/soft_estop`：发送 `CMD_SOFT_ESTOP (0x21010C0E)`，请求机器人进入软急停/关节保护状态。
- `~/release_control`：发送零速度、停止 heartbeat，并清除内部控制权锁存状态。

### 2.8 自主充电

`~/charge_command` 使用 `rcl_interfaces/srv/SetParameters`，参数名为 `charge_command`，支持 `0..3` 或 `start/stop/reset/query`：

| 值 | 命令 | 期望状态 |
|---:|---|---|
| 0 | `START` | `do_charge_task` 或 `charging` |
| 1 | `STOP` | `idle`，随后切回手动模式 |
| 2 | `RESET` | `idle` |
| 3 | `QUERY` | 返回当前响应状态 |

START 前会将感知主机速度源切换为导航模式；若机器人处于 RL 模式，会先退出到可充电状态。服务通过独立接收线程和条件变量等待充电反馈，识别充电桩错误、安全告警、标签超时、位姿跳变、无充电插头等失败状态。响应 `reason` 为 JSON 文本，包含 `charge_state`、`state_name` 和 `message`。

节点还可按 `enable_charge_state_query` 周期查询充电状态，并发布 `/charge_manager_state`。

## 3. ROS 2 接口

### 3.1 订阅

| 接口 | 类型 | 作用 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 导航速度输入 |

### 3.2 发布

| 接口 | 类型 | 来源/作用 |
|---|---|---|
| `/imu/data` | `sensor_msgs/msg/Imu` | 欧拉角转四元数、角速度、线加速度 |
| `/leg_odom` | `nav_msgs/msg/Odometry` | 足式二维位姿和机体速度 |
| `/robot_basic_state` | `std_msgs/msg/Int32` | 基本状态枚举 |
| `/robot_gait_state` | `std_msgs/msg/Int32` | 步态枚举 |
| `/robot_body_height_state` | `std_msgs/msg/Int32` | 机体高度状态 |
| `/charge_manager_state` | `std_msgs/msg/Int32` | 充电管理状态 |
| `/battery/level` | `std_msgs/msg/UInt8` | 电池剩余百分比 |
| `/battery_text` | `rviz_2d_overlay_msgs/msg/OverlayText` | RViz 电量叠加显示 |

可通过 `publish_tf` 发布 `odom -> base_link` TF；坐标帧参数为 `imu_frame_id`、`odom_frame_id`、`base_frame_id`。

### 3.3 服务

| 服务 | 类型 | 作用 |
|---|---|---|
| `~/stand` | `std_srvs/srv/Trigger` | 起立并进入目标 RL 步态 |
| `~/lie` | `std_srvs/srv/Trigger` | 安全趴下 |
| `~/soft_estop` | `std_srvs/srv/Trigger` | 软急停 |
| `~/release_control` | `std_srvs/srv/Trigger` | 释放控制权 |
| `~/set_gait` | `rcl_interfaces/srv/SetParameters` | 切换 11 种支持步态 |
| `~/set_body_height` | `rcl_interfaces/srv/SetParameters` | 切换匍匐/正常高度 |
| `~/charge_command` | `rcl_interfaces/srv/SetParameters` | 自主充电控制 |

## 4. 状态接收与数据转换

后台接收线程校验报文头、类型和载荷长度后，处理以下 X30 报文：

- `0x1008 RcsData`：连接状态、控制模式、里程和运行时间诊断；
- `0x1009 MotionStateData`：更新状态仓库，发布基本状态、步态和足式里程计；
- `0x100A ControllerSensorData`：发布 IMU；
- `0x21050F0A BatterySensorData`：发布电量和 RViz 电量文本；
- `0x11050F08 BodyHeightState`：更新并发布机体高度。

`RobotStateStore` 使用互斥锁、原子状态和条件变量保存连接、基本状态、步态、高度及最近接收时间，为动作服务提供事件驱动的状态等待。

## 5. 启动与配置

启动文件 `launch/nav_bridge.launch.py` 会加载 `config/x30_params.yaml`：

```bash
colcon build --packages-select nav_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash
ros2 launch nav_bridge nav_bridge.launch.py
```

主要可配置项：运动/感知主机地址和端口、心跳周期、速度下发频率、速度超时、是否自动接管控制权、`stand_target_gait`、充电状态查询周期、各 ROS frame ID 以及 `publish_tf`。

## 6. 当前实现边界

- 当前实现面向绝影 X30 UDP 协议，未抽象出其他型号的具体协议实现。
- `percept_udp_` 目前主要用于自主充电链路；其他感知主机配置指令未形成完整业务接口。
- 控制接管是基于 heartbeat/query 的工程策略，不依赖单独的底层控制权 ACK。
- 动作服务依赖 X30 实机状态反馈和固件时序，异常状态会通过超时和失败原因返回。
- 代码仓库未包含针对 UDP 报文和动作状态机的自动化单元测试；现场验证应按“连接 -> stand -> cmd_vel -> lie -> set_gait/set_body_height -> charge -> release_control”顺序执行。

