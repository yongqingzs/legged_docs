# Go1 MuJoCo 模型文件 (go1.xml) 分析

本文档分析了 `go1.xml` MuJoCo 模型文件，旨在说明机器狗模型的关键参数定义，包括电机、IMU 以及仿真与现实的映射关系。

## 1. 各电机的初始位置和位置上下界

在 MuJoCo 的 XML 模型中，电机的属性与关节（joint）紧密相关。

### 1.1 位置上下界 (Joint Range)

电机的**位置上下界**由 `<joint>` 标签的 `range` 属性定义，单位为弧度。这些定义通常被归类在 `<default>` 中以便复用。

- **示例 (外展/内收关节 - Abduction):**
  ```xml
  <default class="abduction">
    <joint axis="1 0 0" range="-0.863 0.863"/>
  </default>
  ...
  <!-- 应用 default class -->
  <joint name="FR_hip_joint" class="abduction"/>
  ```
  `FR_hip_joint` 继承了 `abduction` 类的属性，其活动范围被限制在 **-0.863 到 0.863** 弧度之间。

- **示例 (膝关节 - Knee):**
  ```xml
  <default class="knee">
    <joint range="-2.818 -0.888"/>
    <motor ctrlrange="-35.55 35.55"/>
  </default>
  ...
  <joint name="FR_calf_joint" class="knee"/>
  ```
  `FR_calf_joint` 的活动范围被限制在 **-2.818 到 -0.888** 弧度之间。

### 1.2 初始位置 (Initial Position)

所有关节的**初始位置**由 `<keyframe>` 部分的 `<key name="home" ...>` 统一定义。

- **定义:**
  ```xml
  <keyframe>
    <key name="home" qpos="0 0 0.1 1 0 0 0 0.5 1.2 -2.7 -0.5 1.2 -2.7 0.5 1.2 -2.7 -0.5 1.2 -2.7"
         ctrl="0 0 0 0 0 0 0 0 0 0 0 0" />
  </keyframe>
  ```
  - `qpos` 属性定义了所有广义坐标的初始值。
  - **前 7 个值**: `0 0 0.1 1 0 0 0`，代表浮动基座的初始位姿，格式为 `(x, y, z, qw, qx, qy, qz)`。即位置 `(0, 0, 0.1)`，姿态为单位四元数。
  - **后 12 个值**: `0.5 1.2 -2.7 ...`，依次对应 12 个关节的初始角度（单位：弧度），顺序通常为 FR, FL, RR, RL 四条腿的 hip, thigh, calf 关节。
    - `0.5`: FR_hip_joint
    - `1.2`: FR_thigh_joint
    - `-2.7`: FR_calf_joint
    - `-0.5`: FL_hip_joint
    - ... 以此类推

### 1.3 坐标系定义

关节的坐标系是相对于其父 `<body>` 的局部坐标系定义的。

- **`pos`**: 关节的旋转中心相对于父 `<body>` 原点的位置。
- **`axis`**: 关节的旋转轴，定义在关节自身的局部坐标系中。

- **示例 (FL_hip 关节):**
  ```xml
  <body name="base_link" pos="0 0 0.445" ...>
    ...
    <body name="FL_hip" pos="0.1881 0.04675 0">
      ...
      <joint name="FL_hip_joint" class="abduction"/>
      ...
    </body>
    ...
  </body>
  ```
  - `FL_hip` body 的原点位于 `base_link` 坐标系下的 `(0.1881, 0.04675, 0)`。
  - `FL_hip_joint` 的旋转中心与 `FL_hip` body 的原点重合。
  - `FL_hip_joint` 继承自 `abduction` 类，其旋转轴 `axis` 为 `(1 0 0)`，即绕着 `FL_hip` body 局部坐标系的 X 轴旋转。

## 2. IMU 的初始位置和坐标系

### 2.1 初始位置和坐标系

IMU 在 MuJoCo 中不是一个实体，而是通过一个虚拟的**传感器站点（site）**来定义其位置和姿态。

- **定义:**
  ```xml
  <body name="base_link" ...>
    ...
    <site name="imu" pos="-0.02557 0 0.04232"/>
    ...
  </body>
  ```
  - **位置**: `imu` 站点的位置被定义在 `base_link` 坐标系下，其局部坐标为 `(-0.02557, 0, 0.04232)`。
  - **坐标系**: `imu` 站点的坐标系与 `base_link` 的坐标系**姿态相同**，仅原点发生了平移。

### 2.2 传感器定义

`go1.xml` 文件在 `<sensor>` 部分定义了三个与 IMU 相关的传感器，它们都附着在 `imu` 站点上，从而获取该点的运动信息。

```xml
<sensor>
  ...
  <framequat name="imu_quat" objtype="site" objname="imu"/>
  <gyro name="imu_gyro" site="imu"/>
  <accelerometer name="imu_acc" site="imu"/>
  ...
</sensor>
```
- `framequat`: 测量 `imu` 站点的姿态四元数。
- `gyro`: 测量 `imu` 站点的角速度。
- `accelerometer`: 测量 `imu` 站点的线加速度。

## 3. 仿真与实际传感器映射关系

仿真与实际传感器之间的映射是通过**命名约定**和**软件桥接**实现的。`unitree_mujoco` 包中的 `unitree_sdk2_bridge` 扮演了关键的桥梁角色，它模拟了真实机器狗的 `unitree_sdk2` 行为。

### 3.1 仿真中的传感器定义

`go1.xml` 的 `<sensor>` 部分为所有需要的数据都定义了虚拟传感器，并赋予了唯一的 `name`，这是映射的基础。

- **关节数据**:
  ```xml
  <jointpos name="FR_hip_pos" joint="FR_hip_joint"/>
  <jointvel name="FR_hip_vel" joint="FR_hip_joint"/>
  <jointactuatorfrc name="FR_hip_torque" joint="FR_hip_joint"/>
  ```
- **IMU 数据**:
  ```xml
  <framequat name="imu_quat" objtype="site" objname="imu"/>
  <gyro name="imu_gyro" site="imu"/>
  <accelerometer name="imu_acc" site="imu"/>
  ```
- **足端接触力**:
  ```xml
  <touch name="FR_touch" site="FR_touch"/>
  ```

### 3.2 软件桥接 (`unitree_sdk2_bridge`)

`unitree_mujoco/simulate/src/unitree_sdk2_bridge/unitree_sdk2_bridge.cc` 文件中的 `UnitreeSdk2Bridge` 类负责从 MuJoCo 仿真环境中读取这些虚拟传感器的值，并将其填充到 `unitree_go::msg::dds_::LowState_` 消息中。这个消息结构与真实机器狗 `unitree_sdk2` 使用的 DDS 消息完全一致。

#### 映射流程

1.  **获取传感器 ID**: 在初始化阶段，桥接代码通过 MuJoCo 的 `mj_name2id` 函数，将 XML 中定义的传感器名称（如 `"imu_gyro"`）转换为 MuJoCo 内部的整数 ID。
2.  **读取传感器数据**: 在每个仿真步中，代码从 `mjData->sensordata` 数组中，根据上一步获取的 ID 索引，读取对应的传感器浮点数值。
3.  **填充 DDS 消息**: 将读取到的仿真数据填充到 `LowState_` 消息的相应字段中。例如，将名为 `imu_gyro` 的传感器的三个值，填充到 `low_state.imu_state().gyroscope()` 数组中。
4.  **发布 DDS 消息**: 通过 `ChannelPublisher` 将填充好的 `LowState_` 消息发布出去。

#### 简化代码逻辑

```cpp
// 在 UnitreeSdk2Bridge::Init() 中:
// 将传感器名称 "imu_gyro" 转换为 ID
imu_gyro_sensor_id_ = mj_name2id(model_, mjOBJ_SENSOR, "imu_gyro");

// 在 UnitreeSdk2Bridge::Update() 中:
// 1. 从 MuJoCo 读取数据
double* imu_gyro_data = mjData_->sensordata + imu_gyro_sensor_id_;

// 2. 填充 LowState 消息
low_state.imu_state().gyroscope({imu_gyro_data[0], imu_gyro_data[1], imu_gyro_data[2]});

// 3. 发布消息
low_state_publisher_->Write(low_state);
```

## 电机初始值
### mujoco初始值

| 数组索引 | 关节名称 | 物理意义 |
|---------|----------|----------|
| 0 | FR_hip_joint | 前右髋关节 |
| 1 | FR_thigh_joint | 前右大腿关节 |
| 2 | FR_calf_joint | 前右小腿关节 |
| 3 | FL_hip_joint | 前左髋关节 |
| 4 | FL_thigh_joint | 前左大腿关节 |
| 5 | FL_calf_joint | 前左小腿关节 |
| 6 | RR_hip_joint | 后右髋关节 |
| 7 | RR_thigh_joint | 后右大腿关节 |
| 8 | RR_calf_joint | 后右小腿关节 |
| 9 | RL_hip_joint | 后左髋关节 |
| 10 | RL_thigh_joint | 后左大腿关节 |
| 11 | RL_calf_joint | 后左小腿关节 |

| Motor ID | 模式 (`mode`) | 位置 (`q`) | 速度 (`dq`) | 加速度 (`ddq`) | 力矩 (`tau`) | 温度 (`temperature`) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **0** | `` | -0.100945 | -0.004309 | 0.000000 | 0.004309 | `` |
| **1** | `` | 1.618299 | 0.000172 | 0.000000 | -0.000172 | `` |
| **2** | `` | -2.828159 | -0.000016 | 0.000000 | 0.000016 | `` |
| **3** | `` | 0.100619 | 0.004253 | 0.000000 | -0.004253 | `` |
| **4** | `` | 1.617846 | 0.000171 | 0.000000 | -0.000171 | `` |
| **5** | `` | -2.828163 | -0.000016 | 0.000000 | 0.000016 | `` |
| **6** | `` | -0.053601 | -0.002813 | 0.000000 | 0.002813 | `` |
| **7** | `` | 1.233548 | -0.000213 | 0.000000 | 0.000213 | `` |
| **8** | `` | -2.827233 | -0.000015 | 0.000000 | 0.000015 | `` |
| **9** | `` | 0.053196 | 0.002757 | 0.000000 | -0.002757 | `` |
| **10** | `` | 1.233554 | -0.000214 | 0.000000 | 0.000214 | `` |
| **11** | `` | -2.827238 | -0.000015 | 0.000000 | 0.000015 | `` |

## 校准

### 1. 校准电机零点（initial position offset）

* **实物机器人**通常需要给每个关节电机一个“零点”校准，也就是定义 **电机编码器角度 = 0** 时，对应的实际关节姿态。
* 在 mujoco 里，初始姿态是你建模时定义的 `qpos0`，而在实物里，电机安装和连杆装配的初始位置不一定一致。
* 解决方法：

  1. 让机器人 **趴下（或其他已知静态姿态）**，记录此时所有电机的原始编码器读数。
  2. 将这个编码器读数与仿真模型对应姿态的关节角度做差，得到 **offset**。
  3. 控制器使用的关节角度要始终经过 `θ_corrected = θ_measured - offset`。

---

### 2. 处理传动比（gear ratio）

* 有些电机通过齿轮箱或连杆传动带动关节，导致编码器角度和关节角度不一致。

* 在 mujoco 模型里，通常 `joint angle` 就是你定义的实际关节角度，而实物编码器读数需要乘上/除以传动比才能对应：

  ```
  θ_joint = θ_encoder / gear_ratio
  ```

* **gear\_ratio 校准方法**：

  1. 手动转动电机或关节，量测电机转一圈，关节转多少度。
  2. 或者查电机/减速器规格表。
  3. 在驱动层做转换，让控制器接收到的始终是 **关节空间角度**。

---

### 3. 确认运动学一致性

* 在实物的零点对齐 & gear ratio 校准之后，把机器人各关节摆到几种典型姿态，检查和 mujoco 的末端位置（足端位置）是否一致。
* 如果还是有差异，说明连杆尺寸建模或 DH 参数有问题，需要修正 mujoco 模型，保证仿真和实物运动学结构严格一致。

---

### 4. 限位 & 初始约束

* 真实电机可能有物理限位，mujoco 模型必须加上相同的 `joint range`。
* 机器人趴下时，如果仿真中没有和实物一样的角度范围，就会出现不一致。

---

### 5. 控制器与实物对接

* 在 ocs2 中，控制量往往是 **关节角度/速度/力矩**。确保你传进去的是 **关节角度**（不是原始电机编码器值）。
* 观测值同样要经过 `offset + gear_ratio` 处理，保持和仿真定义完全一致。
