# WBC (Whole Body Control) 模块分析

## 1. 功能概述

WBC (Whole Body Control) 模块是四足机器人控制系统中的核心组件，负责将来自上层控制器（如 MPC）的期望力和运动指令转换为具体的关节力矩命令。该模块通过求解约束优化问题，确保机器人在满足物理约束的前提下尽可能接近期望的运动状态。

## 2. 数据处理和更新流程

### 2.1 输入数据形式
```txt
估计的 base 姿态(四元数): 4   rbdStateMeasured[0:3] -> q_measured_[3:6]
估计的 base 位置: 3          rbdStateMeasured[3:6] -> q_measured_[0:3]  
关节角度: 12                rbdStateMeasured[6:18] -> q_measured_[6:18]
估计的 base 全局角速度: 3    rbdStateMeasured[18:21] -> v_measured_[3:6] (转换后)
估计的 base 线速度: 3        rbdStateMeasured[21:24] -> v_measured_[0:3]
关节角速度: 12              rbdStateMeasured[24:36] -> v_measured_[6:18]
期望状态向量: stateDesired
期望输入向量: inputDesired
接触模式: mode (步态信息)
```

### 2.2 输出数据形式
```txt
关节加速度: 18               决策变量 x[0:18]
接触力: 12                  决策变量 x[18:30] (4个足端×3维力)
关节力矩: 12                决策变量 x[30:42]
```

### 2.3 数据处理流程

#### 2.3.1 主要参与类和方法
- **WbcBase::update()**: 主要更新入口，协调整个处理流程
- **WbcBase::updateMeasured()**: 处理测量数据，进行坐标转换和动力学计算
- **WbcBase::updateDesired()**: 处理期望状态数据
- **WeightedWbc::update()**: 求解加权优化问题
- **HierarchicalWbc::update()**: 求解分层优化问题

#### 2.3.2 具体处理步骤

1. **数据预处理** (`WbcBase::updateMeasured()`)
   - 坐标转换：base 位置和姿态重新排列
   - 角速度转换：全局角速度 -> 欧拉角导数
   - 前向运动学计算：计算末端执行器位置
   - 雅可比矩阵计算：计算 J 和 dJ 矩阵
   - 动力学量计算：质量矩阵 M 和非线性项 nle

2. **任务公式化**
   - 浮动基座动力学约束：`formulateFloatingBaseEomTask()`
   - 力矩限制约束：`formulateTorqueLimitsTask()`
   - 摩擦锥约束：`formulateFrictionConeTask()`
   - 非接触运动约束：`formulateNoContactMotionTask()`
   - 基座加速度任务：`formulateBaseAccelTask()`
   - 摆动腿任务：`formulateSwingLegTask()`
   - 接触力任务：`formulateContactForceTask()`

3. **优化求解**
   - WeightedWbc: 使用 qpOASES 求解加权二次规划问题
   - HierarchicalWbc: 使用 HoQp 求解分层优化问题

### 2.4 足端力传感器作用

足端力传感器在当前 WBC 实现中的作用：

- **不直接参与 WBC 计算**: 足端力传感器数据主要用于上层控制器的状态估计和接触检测
- **间接影响**: 通过接触模式 `mode` 参数影响约束条件的设置
- **非必需**: WBC 模块本身不依赖足端力传感器，主要依赖期望的接触力 `inputDesired`
- **作用范围**: 足端力数据主要在状态估计器和接触检测模块中使用

## 3. 各类方法功能说明

### 3.1 WbcBase 类

#### 3.1.1 核心方法
- **update()**: 主要更新函数，协调整个 WBC 流程
- **updateMeasured()**: 处理测量状态数据，进行动力学计算
- **updateDesired()**: 处理期望状态数据，计算期望运动学

#### 3.1.2 任务公式化方法
- **formulateFloatingBaseEomTask()**: 浮动基座运动方程约束
  - 确保机器人满足牛顿-欧拉方程
  - 形式：M*qdd - J^T*F - S^T*τ = -nle
  
- **formulateTorqueLimitsTask()**: 关节力矩限制约束
  - 防止关节力矩超出安全范围
  - 形式：-τ_max ≤ τ ≤ τ_max

- **formulateNoContactMotionTask()**: 非接触运动约束
  - 确保接触足端速度为零
  - 形式：J*qdd + dJ*qd = 0

- **formulateFrictionConeTask()**: 摩擦锥约束
  - 确保接触力在摩擦锥内
  - 防止足端滑动

- **formulateBaseAccelTask()**: 基座加速度任务
  - 跟踪期望的基座运动
  - 基于质心动量方程

- **formulateSwingLegTask()**: 摆动腿跟踪任务
  - 使摆动腿跟踪期望轨迹
  - PD 控制：kp*(pos_des - pos_meas) + kd*(vel_des - vel_meas)

- **formulateContactForceTask()**: 接触力任务
  - 跟踪期望的接触力

### 3.2 WeightedWbc 类 (重点)

WeightedWbc 是 WbcBase 的派生类，采用加权二次规划方法求解优化问题。

#### 3.2.1 核心特点
- **优化目标**: 最小化加权任务误差
- **求解器**: 使用 qpOASES 库
- **权重配置**: 通过配置文件设置不同任务的权重

#### 3.2.2 关键方法

**update() 方法**:
```cpp
vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired,
                             const vector_t& rbdStateMeasured, size_t mode, scalar_t period)
```
- **功能**: 求解加权优化问题，返回最优解
- **流程**:
  1. 调用基类 update() 进行数据预处理
  2. 公式化约束条件 `formulateConstraints()`
  3. 公式化加权目标函数 `formulateWeightedTasks()`
  4. 构建 QP 问题：min 0.5*x^T*H*x + g^T*x s.t. A*x = b, D*x ≤ f
  5. 使用 qpOASES 求解
  6. 返回决策变量向量

**formulateConstraints() 方法**:
- **功能**: 合并所有硬约束条件
- **包含**: 动力学约束 + 力矩限制 + 摩擦锥 + 接触约束

**formulateWeightedTasks() 方法**:
- **功能**: 构建加权目标函数
- **权重项**:
  - `weightSwingLeg_`: 摆动腿跟踪权重
  - `weightBaseAccel_`: 基座加速度权重  
  - `weightContactForce_`: 接触力跟踪权重

**loadTasksSetting() 方法**:
- **功能**: 从配置文件加载权重参数
- **参数**: swingLeg、baseAccel、contactForce 权重

#### 3.2.3 优化问题数学形式
```
min  w1*||A1*x - b1||² + w2*||A2*x - b2||² + w3*||A3*x - b3||²
s.t. Aeq*x = beq     (等式约束：动力学、接触)
     Aineq*x ≤ bineq  (不等式约束：力矩限制、摩擦锥)
```

### 3.3 HierarchicalWbc 类

#### 3.3.1 特点
- **分层优化**: 按优先级顺序求解任务
- **严格优先级**: 高优先级任务严格满足，低优先级任务在可行空间内优化

#### 3.3.2 优先级层次
1. **最高优先级**: 硬约束（动力学、力矩限制、摩擦锥、接触约束）
2. **中优先级**: 基座加速度和摆动腿跟踪
3. **最低优先级**: 接触力跟踪

### 3.4 HoQp 类

#### 3.4.1 功能
- **分层二次规划**: 实现严格的任务优先级
- **零空间投影**: 将低优先级任务投影到高优先级任务的零空间

#### 3.4.2 关键方法
- **构造函数**: 递归构建分层优化问题
- **getSolutions()**: 获取最优解
- **buildZMatrix()**: 构建零空间投影矩阵

## 4. 其他重要说明

### 4.1 决策变量结构
WBC 优化的决策变量 x 包含三部分：
- **关节加速度** (18维): 包含浮动基座6自由度 + 关节12自由度
- **接触力** (12维): 4个足端各3个方向的力
- **关节力矩** (12维): 12个关节的驱动力矩

### 4.2 坐标系转换
- **输入状态**: [姿态(4), 位置(3), 关节角(12), 角速度(3), 线速度(3), 关节速度(12)]
- **内部表示**: [位置(3), 姿态(3), 关节角(12), 线速度(3), 角速度(3), 关节速度(12)]
- **转换原因**: 符合 Pinocchio 库的标准格式

### 4.3 物理约束
- **动力学约束**: 确保运动符合牛顿力学定律
- **摩擦约束**: 防止足端滑动，维持稳定接触
- **力矩限制**: 保护关节电机，确保安全运行
- **运动学约束**: 确保接触足端不发生位移

### 4.4 性能考虑
- **实时性**: 优化问题需在控制周期内求解（通常1-10ms）
- **数值稳定性**: 通过正则化项和适当的权重调节确保数值稳定
- **可行性**: 合理设置约束边界，确保问题始终有解

### 4.5 调参指导
- **权重调节**: 根据任务优先级合理设置权重比例
- **约束松紧**: 适当的力矩限制和摩擦系数设置
- **收敛性**: 监控 QP 求解器的收敛状态和迭代次数