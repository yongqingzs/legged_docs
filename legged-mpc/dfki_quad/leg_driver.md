### LegDriver 节点功能详细说明

`LegDriver` 是四足机器人控制系统中的腿部驱动节点（ROS2 节点），它提供了一个抽象层，将高层控制器（如 MPC、WBC）的力/位置指令转换为低层电机驱动的关节命令。该节点支持多种控制模式和状态机，确保安全、灵活和实时控制。以下是其核心功能的详细说明。

#### 1. 概述
- **目的**：抽象腿部控制，允许高层控制器专注于力/位置规划，而无需直接处理关节状态。节点在腿基坐标系中工作，支持笛卡尔和关节级控制。
- **架构**：继承自 `rclcpp::Node`，使用定时器（`update_timer_`）以固定频率（`update_freq_`，默认 400 Hz）运行状态机。
- **关键组件**：
  - **发布者**：`joint_cmd_goal_pub_` 发布 `JointCmd` 到电机驱动。
  - **订阅者**：`quad_state_sub_`、`leg_cmd_sub_`、`leg_joint_cmd_sub_` 接收状态和指令。
  - **服务**：`set_keep_joint_pos_srv_`、`set_daming_mode_srv_`、`set_energ_daming_mode_srv_`、`switch_op_mode_srv_` 处理模式切换和安全操作。
  - **模型**：使用 `QuadModelPino` 进行运动学计算（如逆运动学、雅可比矩阵）。

#### 2. 操作模式（OperationMode）
节点支持四种操作模式，通过参数 `control_mode` 设置（0-3）。每种模式处理不同的指令类型：

- **`kJointControl` (0)**：关节级 PD 控制。
  - 输入：`JointCmd`（位置、速度、力矩、Kp、Kd）。
  - 处理：直接转发指令到电机驱动，无额外计算。
  - 适用：简单关节跟踪。

- **`kJointTorqueControl` (1)**：关节扭矩控制。
  - 输入：`JointCmd`（位置、速度、力矩、Kp、Kd）。
  - 处理：计算 PD 力矩（`tau = effort + kp*(des_pos - current_pos) + kd*(des_vel - current_vel)`），限制扭矩（`torque_limit_`）。
  - 适用：需要扭矩反馈的控制。

- **`kCartesianStiffnessControl` (2)**：笛卡尔刚度控制。
  - 输入：`LegCmd`（末端位置、速度、力、Kp、Kd）。
  - 处理：
    - 计算雅可比矩阵（`J`）和当前末端位置/速度（`x_e`、`v_e`）。
    - 计算力矩：`tau = J^T * (kp*(x_d - x_e) + kd*(v_d - v_e) + f_d)`。
    - 限制扭矩。
  - 适用：末端力控制，如支撑相。

- **`kCartesianJointControl` (3)**：笛卡尔关节控制。
  - 输入：`LegCmd`（末端位置、速度、力、Kp、Kd）。
  - 处理：
    - 逆运动学计算关节位置（`q_d`）。
    - 微分运动学计算关节速度和力矩（`qd_d`、`tau`）。
    - 设置 PD 增益。
  - 适用：末端位置跟踪，如摆动相。

#### 3. 状态机（State）
节点使用状态机管理操作，确保安全和自动切换。状态通过 `switchState` 方法转换：

- **`INIT_WAIT`**：初始等待状态。等待第一个指令消息和四足状态，收到后切换到 `OPERATE`。
- **`OPERATE`**：操作状态。执行控制模式，发布关节命令。如果消息重复计数（`msg_repeat_`）超过 `max_msg_repeat_`，切换到 `DAMPING`。
- **`KEEP_POSE`**：保持姿势状态。保持指定关节位置（默认或服务设置），使用 PD 控制。
- **`DAMPING`**：阻尼状态。发送零位置/速度和高 Kd（`damping_kd_`）阻尼命令，收到新消息后切换回 `OPERATE`。
- **`EMERGENCY_DAMPING`**：紧急阻尼状态。类似 `DAMPING`，但只能通过服务切换，只能切换到 `DAMPING`。

状态转换规则：防止无效切换（如从 `EMERGENCY_DAMPING` 直接到 `OPERATE`），并记录日志。

#### 4. 主要方法功能
- **`do_control()`**：核心控制函数。根据操作模式计算 `joint_cmd_goal_` 并发布。包括看门狗检查（`msg_repeat_`）。
- **`do_damping()` / `do_emergency_damp()`**：发送阻尼命令，Kp=0，Kd=高值。
- **`do_keep_pose()`**：发送保持姿势命令，使用 `keep_joint_pos_`。
- **`do_init_wait()`**：等待消息，记录警告。
- **`handle_*` 回调**：
  - `handle_quad_state_update`：更新 `quad_state_`。
  - `handle_leg_cmd_update` / `handle_leg_joint_cmd_update`：更新指令，重置 `msg_repeat_`。
  - `handle_keep_pose`：设置保持姿势，切换状态。
  - `handle_change_mode`：切换操作模式，重新注册订阅者。
  - `handle_damping` / `handle_emerg_damping`：切换到阻尼状态。
- **`registerSubscribersAccordingToOperatioMode()`**：根据模式注册订阅者（`leg_cmd` 或 `leg_joint_cmd`）。

#### 5. 数据流
- **输入**：
  - `QuadState`：关节位置/速度，用于运动学计算。
  - `LegCmd` 或 `JointCmd`：高层指令（位置、力等）。
- **处理**：
  - 根据模式使用 `QuadModelPino` 计算逆运动学、雅可比、力矩。
  - 应用扭矩限制（`torque_limit_`）。
  - 状态机管理安全切换。
- **输出**：
  - `JointCmd`：关节位置、速度、力矩、Kp、Kd 到电机驱动。

#### 6. 安全机制
- **看门狗**：`msg_repeat_` 计数器，防止消息丢失导致失控。
- **扭矩限制**：所有力矩限制在 `torque_limit_` 内。
- **阻尼模式**：自动或手动切换到阻尼，防止机器人坠落。
- **参数验证**：启动时检查参数大小（如 `default_keep_joint_positions`）。

#### 7. 其他功能
- **参数**：从配置文件加载 `update_freq`、`torque_limit`、`damping_kd` 等。
- **服务**：允许动态切换模式和姿势。
- **日志**：使用 `RCLCPP_INFO`、`WARN`、`ERROR` 记录状态变化和错误。
- **实时性**：高频率更新，确保控制响应。

该节点是控制系统的执行层，桥接高层规划和低层硬件，确保四足机器人的稳定运动。有关更多细节，请参考代码注释或 ROS2 文档。


### MjbotsMotorDriver 功能详细说明

`MjbotsMotorDriver` 是四足机器人控制系统中的电机驱动节点（ROS2 节点），继承自 `AbstractMotorDriver` 抽象基类。它专门用于与 mjbots 硬件（如 pi3hat 和 moteus 电机）交互，提供低层电机控制接口。该节点使用 mjbots API 处理 CAN 总线通信、电机配置和实时控制，确保硬件层的安全和高效操作。

#### 1. 概述
- **目的**：作为电机硬件的驱动程序，接收高层关节命令，转换为电机指令发送到硬件，并反馈电机状态和 IMU 数据。它是控制系统的执行层，桥接 ROS2 消息与物理电机。
- **架构**：单线程执行器（`SingleThreadedExecutor`），使用定时器回调处理命令发送和状态发布。支持信号处理（如 SIGINT）以安全关闭。
- **关键组件**：
  - **硬件接口**：`MjbotsHardwareInterface` 用于与 pi3hat 和 moteus 电机通信。
  - **订阅者**：`joint_cmd` 话题（`JointCmd` 消息）。
  - **发布者**：`joint_states`（`JointState` 消息）、`imu_measurement`（`Imu` 消息）。
  - **服务**：无直接服务，但通过抽象基类支持启用/禁用电机等操作。

#### 2. 主要功能
- **电机配置**（`configure_motors()`）：
  - 从参数加载电机属性（如 bus_id、motor_id、Kp、Kd、max_torque、direction、zero_offset）。
  - 创建 `JointMoteus` 对象向量，配置 CAN 总线和实时参数（`RealtimeParams`）。
  - 初始化 `MjbotsHardwareInterface`，设置 IMU 安装角度和采样率。
  - 目的：确保电机正确映射和初始化。

- **电机启用**（`enable_motors()`）：
  - 发送启用命令到电机，检查模式（目标模式 10：位置控制）。
  - 读取初始电机状态（位置、速度、扭矩）。
  - 返回电机状态映射（`std::map<std::tuple<int, int>, MotorState>`）。
  - 目的：激活电机并获取初始反馈。

- **电机禁用**（`disable_motors()`）：
  - 发送停止命令，检查模式（目标模式 0：停止）。
  - 目的：安全关闭电机，防止意外运动。

- **设置零点**（`set_zero()`）：
  - 将所有电机位置目标设置为 0。
  - 发送命令。
  - 目的：校准电机位置。

- **发送命令**（`send_command()`）：
  - 接收命令映射（`std::map<std::tuple<int, int>, MotorCommand>`），设置电机目标（位置、速度、Kp、Kd、扭矩）。
  - 调用 `SendCommand()` 发送到硬件。
  - 读取电机状态和 IMU 数据（加速度、角速度、四元数）。
  - 返回状态和 IMU 数据元组。
  - 目的：实时控制电机并反馈状态。

- **数据流**：
  - **输入**：`JointCmd` 消息（位置、速度、Kp、Kd、扭矩）。
  - **处理**：使用 mjbots API 转换为 CAN 指令，限制扭矩（`max_torque`）。
  - **输出**：`JointState`（位置、速度、扭矩）、`Imu`（姿态、加速度、角速度）。

- **安全机制**：
  - **看门狗**：继承自 `AbstractMotorDriver`，监控消息丢失，超时禁用电机。
  - **扭矩限制**：防止过载。
  - **信号处理**：SIGINT 触发禁用电机和关闭。

#### 3. 与 LegDriver 的关系
- **角色分工**：
  - `LegDriver` 是高层抽象节点，专注于腿部控制逻辑（如逆运动学、力控制），接收 `LegCmd` 或 `JointCmd`，计算关节指令，并发布到 `joint_cmd` 话题。它处理力/位置规划，不直接与硬件交互。
  - `MjbotsMotorDriver` 是低层执行节点，接收 `joint_cmd`，直接控制电机硬件，并发布状态反馈。它是硬件接口，无高层逻辑。

- **通信方式**：
  - **发布-订阅**：`LegDriver` 发布 `JointCmd` 到 `joint_cmd` 话题；`MjbotsMotorDriver` 订阅该话题。
  - **反馈循环**：`MjbotsMotorDriver` 发布 `joint_states` 和 `imu_measurement`；`LegDriver` 订阅这些话题获取反馈，用于状态更新和控制调整。
  - **数据转换**：`LegDriver` 将笛卡尔指令转换为关节命令；`MjbotsMotorDriver` 将关节命令转换为电机指令。

- **集成关系**：
  - 形成控制链：高层规划（MPC/WBC）→ `LegDriver`（抽象控制）→ `MjbotsMotorDriver`（硬件执行）→ 电机硬件。
  - `LegDriver` 依赖 `MjbotsMotorDriver` 的状态反馈来闭环控制；`MjbotsMotorDriver` 依赖 `LegDriver` 的指令来执行动作。
  - 两者通过 ROS2 话题解耦，提高模块化：`LegDriver` 可与不同电机驱动兼容，`MjbotsMotorDriver` 可独立测试。

- **配置与参数**：
  - 两者共享电机参数（如 Kp、Kd），从 YAML 文件加载。
  - `AbstractMotorDriver` 提供通用接口，确保 `MjbotsMotorDriver` 与其他驱动（如 T-Motor）兼容。

该节点确保实时、低延迟的电机控制，支持四足机器人的动态运动。有关更多细节，请参考代码注释或 ROS2 文档。