# GaitGenerator 类分析

## 1. 功能

GaitGenerator 类是一个四足机器人步态轨迹生成器，负责根据期望的运动命令（速度、转角、步态高度）和当前步态状态生成每条腿的脚端位置和速度轨迹。该类结合 WaveGenerator 的相位信息和 FeetEndCalc 的目标计算，使用摆线（cycloid）轨迹规划实现平滑的腿部运动。

主要功能包括：
- **轨迹规划**：为摆动相腿生成平滑的3D轨迹
- **状态管理**：区分支撑相和摆动相的控制逻辑
- **实时更新**：根据相位实时计算位置和速度
- **参数配置**：支持动态调整步态参数

该类是四足机器人控制系统的核心组件之一，连接高层运动规划和底层关节控制。

## 2. 各方法实现的功能

### 构造函数
```cpp
GaitGenerator(CtrlComponent &ctrl_component)
```
- **功能**：初始化步态生成器
- **实现**：获取 WaveGenerator、Estimator 和 FeetEndCalc 的引用，设置首次运行标志

### setGait 方法 (步态参数设置)
```cpp
void setGait(Vec2 vxy_goal_global, double d_yaw_goal, double gait_height)
```
- **功能**：设置期望的步态参数
- **参数**：
  - vxy_goal_global: 全局坐标系下的期望XY速度
  - d_yaw_goal: 期望的偏航角速度
  - gait_height: 步态高度（摆动腿抬高距离）

### generate 方法 (轨迹生成主函数)
```cpp
void generate(Vec34 &feet_pos, Vec34 &feet_vel)
```
- **功能**：生成四条腿的脚端位置和速度
- **实现流程**：
  1. 初始化首次运行时的起始位置
  2. 遍历四条腿，根据接触状态处理：
     - **支撑相**：保持当前位置，速度为0
     - **摆动相**：计算目标位置，使用摆线轨迹规划
- **关键逻辑**：相位 < 0.5 时更新支撑起始位置，确保轨迹连续性

### restart 方法 (重置状态)
```cpp
void restart()
```
- **功能**：重置生成器状态，用于步态切换或重新开始
- **实现**：重置首次运行标志，清空速度命令，初始化 FeetEndCalc

### getFootPos 方法 (单脚位置计算)
```cpp
Vec3 getFootPos(int i)
```
- **功能**：计算指定腿的当前脚端位置
- **实现**：使用摆线函数分别计算XY平面和Z方向的位置

### getFootVel 方法 (单脚速度计算)
```cpp
Vec3 getFootVel(int i)
```
- **功能**：计算指定腿的当前脚端速度
- **实现**：使用摆线函数的导数计算XY平面和Z方向的速度

### 摆线轨迹计算方法
```cpp
static double cycloidXYPosition(double startXY, double endXY, double phase)
static double cycloidZPosition(double startZ, double height, double phase)
double cycloidXYVelocity(double startXY, double endXY, double phase) const
double cycloidZVelocity(double height, double phase) const
```
- **功能**：实现摆线轨迹的数学计算
- **XY平面位置**：
  \[
  pos = (end - start) \cdot \frac{\theta - \sin\theta}{2\pi} + start, \quad \theta = 2\pi \cdot phase
  \]
- **Z方向位置**：
  \[
  pos = height \cdot \frac{1 - \cos\theta}{2} + start, \quad \theta = 2\pi \cdot phase
  \]
- **XY平面速度**：
  \[
  vel = (end - start) \cdot \frac{1 - \cos\theta}{T_{swing}}
  \]
- **Z方向速度**：
  \[
  vel = height \cdot \pi \cdot \frac{\sin\theta}{T_{swing}}
  \]

## 3. 需要说明的内容

### 摆线轨迹的优势
- **平滑性**：加速度连续，避免冲击
- **可控性**：参数化控制轨迹形状
- **计算效率**：解析表达式，适合实时计算

### 相位使用规则
- **支撑相**：相位 0-0.5，腿着地，位置固定
- **摆动相**：相位 0.5-1.0，腿抬起，执行摆线轨迹
- **起始位置更新**：在支撑相早期（相位 < 0.5）更新起始位置，确保轨迹连续

### 坐标系约定
- **全局坐标系**：vxy_goal_global 在世界坐标系中
- **身体坐标系**：脚端位置在身体坐标系中计算
- **轨迹坐标系**：摆线轨迹在腿部局部坐标系中定义

### 关键参数
- **gait_height**：摆动腿的最大抬高高度，影响步态稳定性和通过性
- **T_swing**：摆动相时间，从 WaveGenerator 获取
- **起始/结束位置**：start_p_ 从估计器获取，end_p_ 从 FeetEndCalc 计算

### 状态管理
- **first_run_**：首次运行标志，用于初始化起始位置
- **contact_**：从 WaveGenerator 获取的接触状态
- **phase_**：从 WaveGenerator 获取的相位值

### 依赖组件
- **WaveGenerator**：提供相位和接触状态
- **Estimator**：提供当前脚端位置估计
- **FeetEndCalc**：计算摆动相的目标位置

### 性能考虑
- **实时性**：所有计算都是解析的，无需数值优化
- **内存效率**：存储少量历史状态
- **数值稳定性**：使用标准数学函数，避免奇异点

### 扩展性
- **轨迹类型**：可扩展其他轨迹规划算法（如多项式、贝塞尔曲线）
- **多相位步态**：可适应更复杂的步态模式
- **地形适应**：可集成地形信息调整轨迹

## 4. 总结

GaitGenerator 类通过摆线轨迹规划实现了四足机器人的平滑步态轨迹生成。该类巧妙地结合了时序控制（WaveGenerator）、状态估计（Estimator）和目标计算（FeetEndCalc），为每条腿生成适合的运动轨迹。在实际应用中，该类确保了步态的连续性和平滑性，是实现动态四足运动的关键组件。