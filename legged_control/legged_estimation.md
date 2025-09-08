# `legged_estimation` 包代码分析

## 1. 功能与节点关系

### 1.1. 核心功能

`legged_estimation` 包的核心功能是为腿式机器人提供状态估计。它本身不包含任何直接运行的ROS节点，而是以一个C++库的形式存在。这个库提供了多个状态估计算法类，这些类可以被其他ROS节点（例如主控制器节点）实例化和调用，以获取机器人实时的状态信息，包括位姿（位置和姿态）、线速度和角速度等。

该库的输出是标准的ROS消息，主要发布到以下两个话题：
- `/odom` (`nav_msgs::Odometry`): 包含机器人的位姿和速度信息。
- `/pose` (`geometry_msgs::PoseWithCovarianceStamped`): 只包含机器人的位姿信息。

### 1.2. "节点"关系 (类之间的关系)

这个包中的“节点”实际上是三个核心的C++类，它们之间的关系如下：

- **`StateEstimateBase` (抽象基类)**
  - 这是所有状态估计器的基类，定义了统一的接口。它负责处理来自上层（如硬件接口）的数据输入，例如IMU数据和关节状态数据，并提供了发布最终估计结果的功能。具体的估计算法由其派生类实现。

- **`FromTopicStateEstimate` (派生类)**
  - 这是一个简单的状态估计器，它继承自 `StateEstimateBase`。
  - **功能**: 它直接订阅一个给定的ROS话题（默认为 `/ground_truth/state`），并将接收到的里程计消息作为“真实”的机器人状态。
  - **用途**: 主要用于仿真环境，可以直接从仿真器获取精确的机器人状态，方便调试上层算法。

- **`KalmanFilterEstimate` (派生类)**
  - 这是一个复杂且实用的状态估计器，同样继承自 `StateEstimateBase`。
  - **功能**: 它实现了一个线性卡尔曼滤波器，融合多种传感器信息来估计机器人的状态。
    - **输入**:
      1.  **IMU数据**: 提供机器人的角速度和线加速度。
      2.  **关节状态**: 提供各关节的位置和速度，用于运动学计算。
      3.  **足端接触状态**: 判断哪些脚与地面接触。
      4.  **外部里程计 (可选)**: 订阅一个外部里程计话题（如 `/tracking_camera/odom/sample`），用于修正长期漂移。
  - **核心思想**:
    - **预测**: 利用IMU的加速度数据来预测机器人的运动。
    - **修正**: 利用运动学模型和足端接触信息进行修正。基本假设是：当一只脚与地面接触时，它在世界坐标系中应保持静止。通过比较运动学计算出的足端速度与“静止”这个假设，来反向修正机器人的主体状态。

---

## 2. 各个类中主要方法的功能

### 2.1. `StateEstimateBase` (基类)

- `StateEstimateBase(...)`
  - **功能**: 构造函数。初始化Pinocchio（用于运动学计算）、质心模型信息以及ROS话题的发布器（`/odom` 和 `/pose`）。

- `virtual void updateJointStates(const vector_t& jointPos, const vector_t& jointVel)`
  - **功能**: 从外部（通常是硬件接口）更新机器人的关节位置和速度。

- `virtual void updateContact(contact_flag_t contactFlag)`
  - **功能**: 更新足端的接触状态标志。

- `virtual void updateImu(...)`
  - **功能**: 更新IMU数据。它接收原始的四元数、角速度和线加速度，并进行初步处理，计算出全局角速度，然后调用 `updateAngular` 更新到状态向量中。

- `virtual vector_t update(const ros::Time& time, const ros::Duration& period) = 0`
  - **功能**: 纯虚函数，是状态估计的核心。每个派生类必须实现这个函数，执行具体的估计算法，并返回更新后的状态向量。

- `void updateAngular(const vector_t& zyx, const vector_t& angularVel)`
  - **功能**: 将计算出的角度（欧拉角）和角速度更新到内部状态向量 `rbdState_` 中。

- `void updateLinear(const vector_t& pos, const vector_t& linearVel)`
  - **功能**: 将计算出的位置和线速度更新到内部状态向量 `rbdState_` 中。

- `void publishMsgs(const nav_msgs::Odometry& odom)`
  - **功能**: 以一定的频率发布里程计和位姿消息。

### 2.2. `FromTopicStateEstimate` (派生类)

- `FromTopicStateEstimate(...)`
  - **功能**: 构造函数。在基类的基础上，额外初始化一个ROS订阅器，用于订阅 `/ground_truth/state` 话题。

- `void callback(const nav_msgs::Odometry::ConstPtr& msg)`
  - **功能**: 订阅器回调函数。当接收到消息时，将其存入一个线程安全的缓冲区 `buffer_`。

- `vector_t update(const ros::Time& time, const ros::Duration& period)`
  - **功能**: 实现基类的 `update` 方法。它从缓冲区读取最新的里程计消息，然后直接调用基类的 `updateAngular` 和 `updateLinear` 方法更新状态，并发布消息。

### 2.3. `KalmanFilterEstimate` (派生类)

- `KalmanFilterEstimate(...)`
  - **功能**: 构造函数。初始化卡尔曼滤波器的各个矩阵（如状态转移矩阵A、协方差矩阵P、噪声矩阵Q和R等），并订阅外部里程计话题。

- `vector_t update(const ros::Time& time, const ros::Duration& period)`
  - **功能**: 卡尔曼滤波的核心。
    1.  **状态预测 (Prediction)**: 基于上一时刻的状态和IMU测得的加速度，预测当前时刻的状态。公式类似于 `x_hat = A * x + B * u`。
    2.  **协方差预测 (Covariance Prediction)**: 更新预测模型的不确定性。公式为 `P = A * P * A^T + Q`。
    3.  **测量更新 (Measurement Update)**:
        - 通过正向运动学计算出每个脚的位置和速度。
        - 构建测量向量 `y`，其核心思想是接触地面的脚速度应为零。
        - 计算卡尔曼增益 `K`。
        - 使用测量值 `y` 和卡尔曼增益 `K` 来修正预测的状态 `x_hat` 和协方差 `P`。
    4.  调用 `updateLinear` 更新最终估计出的线速度和位置。
    5.  调用 `getOdomMsg` 生成里程计消息并发布。

- `void updateFromTopic()`
  - **功能**: 当接收到外部里程计（如视觉SLAM）的数据时被调用。它计算当前估计位姿与外部里程计位姿之间的变换关系，并以此来修正状态估计中的位置，有效抑制了仅靠IMU和运动学积分带来的长期漂移。

- `void loadSettings(const std::string& taskFile, bool verbose)`
  - **功能**: 从配置文件（`.info` 文件）中加载卡尔曼滤波器的各项噪声参数，方便调节滤波器性能。

- `nav_msgs::Odometry getOdomMsg()`
  - **功能**: 将内部估计出的状态（位置、姿态、速度等）打包成一个 `nav_msgs::Odometry` 消息，以供发布。

---

## 3. 其他需要说明的内容

- **状态向量**:
  - 基类中的 `rbdState_` 是一个通用的机器人状态向量，包含了三维欧拉角、三维位置、三维角速度、三维线速度以及所有关节的位置和速度。
  - 卡尔曼滤波器内部的状态向量 `xHat_` 则更加专注，它主要包含：机器人的位置、线速度以及所有脚在世界坐标系下的估计位置。

- **坐标系管理**:
  - `KalmanFilterEstimate` 能够处理多个坐标系之间的转换（例如 `world`, `odom`, `base` 以及外部传感器坐标系），这是正确融合多源信息的关键。

- **可调参数**:
  - `KalmanFilterEstimate` 的性能高度依赖于其噪声参数的设置（例如IMU的噪声、足端传感器的噪声等）。这些参数都在配置文件中提供，需要针对不同的机器人和场景进行细致的调节。
