# BalanceCtrl 类分析

## 1. 功能

BalanceCtrl 类是一个基于二次规划 (Quadratic Programming, QP) 的四足机器人平衡控制器，主要用于计算四条腿的期望脚端力，以实现机器人的平衡控制。该类通过求解约束优化问题，确保机器人能够跟踪期望的身体加速度和角加速度，同时满足摩擦约束和接触状态限制。

该控制器采用模型预测控制 (MPC) 的思想，将平衡问题转化为 QP 优化问题，通过最小化力矩变化和跟踪误差来计算最优的脚端力分布。适用于四足机器人的站立、行走和跳跃等运动模式下的平衡控制。

## 2. 各方法实现的功能

### 构造函数
```cpp
BalanceCtrl(const std::shared_ptr<QuadrupedRobot>& robot)
```
- **功能**：初始化平衡控制器参数
- **实现**：
  - 从 QuadrupedRobot 获取机器人质量
  - 设置 QP 权重参数 (alpha_, beta_)
  - 初始化重力向量、重心位置、惯性矩阵
  - 设置摩擦系数和摩擦约束矩阵
  - 初始化权重矩阵 (S_, W_, U_) 用于 QP 目标函数

### calF 方法 (主计算函数)
```cpp
Vec34 calF(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rot_matrix,
           const Vec34 &feet_pos_2_body, const VecInt4 &contact)
```
- **功能**：计算期望的脚端力
- **实现流程**：
  1. 计算动力学矩阵 A
  2. 计算期望力向量 bd
  3. 计算约束矩阵 (摩擦约束和接触约束)
  4. 构建 QP 问题 (目标函数 G_, g0T_)
  5. 求解 QP 得到最优力 F_
  6. 更新上一时刻力用于平滑性约束
- **输入**：
  - ddPcd: 期望身体加速度
  - dWbd: 期望身体角加速度
  - rot_matrix: 当前身体旋转矩阵
  - feet_pos_2_body: 脚端相对于身体的位置
  - contact: 接触状态向量 (0/1)
- **输出**：期望脚端力向量 (12维 → 4×3)

### calMatrixA 方法 (动力学矩阵计算)
```cpp
void calMatrixA(const Vec34 &feet_pos_2_body, const RotMat &rotM)
```
- **功能**：构建牛顿-欧拉动力学方程的矩阵 A
- **实现**：
  - 前3行：质量矩阵 (单位矩阵 × 质量)
  - 后3行：转动惯量矩阵 (叉积矩阵)
- **公式**：
  \[
  A = \begin{bmatrix}
  I_3 & I_3 & I_3 & I_3 \\
  \hat{r}_1 & \hat{r}_2 & \hat{r}_3 & \hat{r}_4
  \end{bmatrix}
  \]
  其中 \(\hat{r}_i\) 是脚端位置的叉积矩阵

### calVectorBd 方法 (期望力向量计算)
```cpp
void calVectorBd(const Vec3 &ddPcd, const Vec3 &dWbd, const RotMat &rotM)
```
- **功能**：计算期望的广义力向量
- **实现**：
  - 前3维：质量 × (期望加速度 - 重力)
  - 后3维：旋转惯量 × 期望角加速度
- **公式**：
  \[
  b_d = \begin{bmatrix}
  m(\ddot{p}_{cd} - g) \\
  I_b \dot{\omega}_{bd}
  \end{bmatrix}
  \]

### calConstraints 方法 (约束矩阵计算)
```cpp
void calConstraints(const VecInt4 &contact)
```
- **功能**：根据接触状态构建约束矩阵
- **实现**：
  - **等式约束 (CE_, ce0_)**：非接触腿的力为0
  - **不等式约束 (CI_, ci0_)**：接触腿的摩擦锥约束
- **摩擦约束**：使用线性化的摩擦锥近似
  \[
  \begin{bmatrix}
  1 & 0 & \mu \\
  -1 & 0 & \mu \\
  0 & 1 & \mu \\
  0 & -1 & \mu \\
  0 & 0 & 1
  \end{bmatrix}
  \begin{bmatrix}
  F_x \\ F_y \\ F_z
  \end{bmatrix} \leq 0
  \]

### solveQP 方法 (QP 求解)
```cpp
void solveQP()
```
- **功能**：使用 QuadProg++ 库求解 QP 问题
- **QP 形式**：
  \[
  \begin{aligned}
  \min_x & \frac{1}{2}x^T G x + g_0^T x \\
  \text{s.t.} & CE^T x = ce_0 \\
  & CI^T x \geq ci_0
  \end{aligned}
  \]
- **实现**：将 Eigen 矩阵转换为 QuadProg++ 格式，调用 solve_quadprog 函数

## 3. 需要说明的内容

### QP 优化问题详解
- **目标函数**：
  \[
  J = \frac{1}{2}F^T (A^T S A + \alpha W + \beta U) F + (-b_d^T S A - \beta F_{prev}^T U)^T F
  \]
  - \(A^T S A\): 跟踪误差惩罚
  - \(\alpha W\): 力变化惩罚 (平滑性)
  - \(\beta U\): 力大小惩罚 (能量效率)

- **约束条件**：
  - **动力学约束**：\(A F = b_d\)
  - **摩擦约束**：力在摩擦锥内
  - **接触约束**：非接触腿力为0

### 关键参数说明
- **质量 (mass_)**：从 QuadrupedRobot 获取
- **惯性矩阵 (Ib_)**：身体转动惯量，对角矩阵
- **摩擦系数 (friction_ratio_)**：默认 0.4
- **权重矩阵**：
  - S_: 跟踪权重 [20,20,50,450,450,450] (平移/旋转)
  - W_: 力变化权重 [10,10,4,...] (x,y,z 方向)
  - U_: 力大小权重 [3,3,3,...]
- **平滑参数**：alpha_=0.001, beta_=0.1

### 坐标系和假设
- **身体坐标系**：控制计算在身体坐标系中进行
- **世界坐标系**：重力在世界坐标系中定义
- **假设**：
  - 点质量模型 (忽略身体几何)
  - 刚体动力学
  - 准静态平衡 (忽略动量变化)

### 依赖项
- **QuadProg++**：QP 求解器
- **Eigen**：矩阵运算库
- **QuadrupedRobot**：提供机器人参数
- **mathTypes.h**：自定义数学类型定义

### 性能和实时性
- **计算复杂度**：QP 求解 O(n^3)，n=12，适合实时控制
- **求解器**：使用活跃集方法，数值稳定
- **收敛性**：在合理参数下保证收敛

### 扩展性
- **多接触点**：可扩展到任意接触配置
- **非线性摩擦**：可使用更精确的摩擦模型
- **动态效果**：可添加动量和角动量约束

## 4. 总结

BalanceCtrl 类实现了四足机器人的平衡控制核心，通过 QP 优化计算最优脚端力分布。该方法结合了动力学建模、约束优化和实时控制，能够有效处理四足机器人的平衡问题。在实际应用中，该控制器通常作为高层控制器的基础组件，与步态规划器和轨迹跟踪器配合使用，实现复杂的运动控制任务。