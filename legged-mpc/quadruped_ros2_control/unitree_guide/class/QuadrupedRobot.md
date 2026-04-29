# QuadrupedRobot 类分析

## 1. 功能

QuadrupedRobot 类是一个封装四足机器人运动学和动力学计算的核心类，主要用于四足机器人的控制系统中。它基于 KDL (Kinematics and Dynamics Library) 库，提供以下主要功能：

- **运动学计算**：计算关节位置、脚端位置、雅可比矩阵等正逆运动学问题
- **动力学计算**：计算关节扭矩、脚端速度等
- **状态更新**：从 ROS2 控制接口获取当前关节状态
- **机器人模型管理**：管理四条腿的运动学链和相关参数

该类设计用于与 ROS2 控制框架集成，通过 CtrlInterfaces 接口获取关节状态数据，并为控制器提供必要的运动学计算服务。

## 2. 各方法实现的功能

### 构造函数
```cpp
QuadrupedRobot(CtrlInterfaces &ctrl_interfaces, const std::string &robot_description,
               const std::vector<std::string> &feet_names, const std::string &base_name)
```
- 从 URDF 字符串解析机器人树结构
- 为四条腿创建运动学链 (fr_chain_, fl_chain_, rr_chain_, rl_chain_)
- 初始化四条腿的 RobotLeg 对象
- 计算机器人总质量
- 设置默认站立姿态下的脚端位置

### getQ 方法 (关节位置计算)
```cpp
std::vector<KDL::JntArray> getQ(const std::vector<KDL::Frame> &pEe_list) const
Vec12 getQ(const Vec34 &vecP) const
```
- **功能**：根据给定的脚端位置计算对应的关节角度
- **实现**：使用逆运动学求解器计算每条腿的关节角度
- **输入**：脚端位置 (Frame 或 Vec34 格式)
- **输出**：关节角度向量 (12维，3关节×4腿)

### getQd 方法 (关节速度计算)
```cpp
Vec12 getQd(const std::vector<KDL::Frame> &pos, const Vec34 &vel)
```
- **功能**：根据脚端位置和速度计算关节速度
- **实现**：使用雅可比矩阵的逆计算关节速度
- **公式**：$\dot{q} = J^{-1} \dot{p}_{ee}$

### getFeet2BPositions 方法 (脚端位置计算)
```cpp
std::vector<KDL::Frame> getFeet2BPositions() const
KDL::Frame getFeet2BPositions(const int index) const
```
- **功能**：根据当前关节角度计算脚端在基座坐标系中的位置
- **实现**：使用正运动学计算每条腿的脚端位置
- **输出**：脚端位置的 Frame 对象 (包含位置和姿态)

### getJacobian 方法 (雅可比矩阵计算)
```cpp
KDL::Jacobian getJacobian(const int index) const
```
- **功能**：计算指定腿部的雅可比矩阵
- **实现**：基于当前关节角度计算雅可比矩阵
- **用途**：用于速度映射和力矩计算

### getTorque 方法 (扭矩计算)
```cpp
KDL::JntArray getTorque(const Vec3 &force, const int index) const
KDL::JntArray getTorque(const KDL::Vector &force, int index) const
```
- **功能**：根据外部力和关节状态计算关节扭矩
- **实现**：使用雅可比矩阵的转置计算扭矩
- **公式**：$\tau = J^T F$

### getFeet2BVelocities 方法 (脚端速度计算)
```cpp
KDL::Vector getFeet2BVelocities(const int index) const
std::vector<KDL::Vector> getFeet2BVelocities() const
```
- **功能**：计算脚端在基座坐标系中的速度
- **实现**：使用雅可比矩阵将关节速度映射到笛卡尔空间
- **公式**：$\dot{p}_{ee} = J \dot{q}$

### update 方法 (状态更新)
```cpp
void update()
```
- **功能**：从 ROS2 控制接口更新当前关节位置和速度
- **实现**：读取 joint_position_state_interface 和 joint_velocity_state_interface 的值
- **调用时机**：每个控制周期开始时调用

## 3. 需要说明的内容

### 依赖项和库
- **KDL (Kinematics and Dynamics Library)**：用于运动学计算
- **kdl_parser**：用于从 URDF 解析机器人树
- **Eigen**：用于向量和矩阵运算 (通过 mathTypes.h)
- **ROS2 控制接口**：通过 CtrlInterfaces 获取关节状态

### 数据结构
- **RobotLeg 类**：封装单腿的运动学计算
- **KDL::Chain**：表示运动学链
- **Vec12/Vec34/Vec3**：自定义向量类型 (基于 Eigen)
- **CtrlInterfaces**：ROS2 控制接口结构体

### 坐标系约定
- **基座坐标系 (Base Frame)**：机器人躯干坐标系
- **世界坐标系 (World Frame)**：全局参考坐标系
- **脚端坐标系 (End-effector Frame)**：每条腿的末端坐标系

### 假设和限制
- **关节配置**：假设每条腿有 3 个关节 (hip, thigh, calf)
- **运动学模型**：使用串联刚体模型，忽略关节摩擦和柔性
- **计算精度**：逆运动学可能有多个解，默认选择第一个解
- **实时性**：所有计算都是确定性的，无阻塞操作

### 性能考虑
- **计算复杂度**：运动学计算相对轻量，适合实时控制
- **内存使用**：存储四条腿的运动学链和当前状态
- **线程安全**：update() 方法需要与控制循环同步调用

### 扩展性
- **腿部数量**：目前固定为 4 条腿，可扩展为其他配置
- **关节数量**：每腿 3 关节，可适应不同关节配置
- **传感器集成**：可扩展支持力传感器、IMU 等

## 4. 总结

QuadrupedRobot 类是四足机器人控制系统的运动学核心，提供完整的正逆运动学计算功能。其设计简洁高效，易于集成到 ROS2 控制框架中。通过封装复杂的 KDL 计算，该类为高层控制器提供了清晰的接口，支持实时控制需求。

在实际应用中，该类通常与控制器 (如 WBC 或 MPC) 配合使用，负责将笛卡尔空间的控制命令转换为关节空间的执行命令。