# `mpc.cpp` 文件分析文档

本文档旨在详细分析 `mpc.cpp` 文件的功能、方法、数据流及其在四足机器人控制系统中的作用。该文件实现了模型预测控制（MPC）的核心逻辑，使用 Acados 库解决二次规划（QP）问题。

## 1. 文件各方法的功能

`mpc.cpp` 实现了 `MPC` 类，该类继承自 `MPCInterface`，负责基于线性化模型的预测控制计算。主要方法包括静态辅助方法、构造函数、更新方法、设置方法和核心求解方法。

### 静态辅助方法
这些方法用于计算 MPC 所需的线性化矩阵和约束：

- **`GetA(double psi, double dt, bool update_linearization, Eigen::Ref<AMatrixT> A)`**
  - **功能**: 计算状态转移矩阵 A，用于描述系统状态的线性化动力学。
  - **关键说明**: A 矩阵描述了状态（如位置、速度、姿态）如何随时间演变。`psi` 是偏航角，`dt` 是时间步长。`update_linearization` 控制是否更新线性化项。

- **`GetB(double psi, double m, const Eigen::Ref<const Eigen::Matrix3d> &inertia, const std::array<const Eigen::Ref<const Eigen::Vector3d>, NUM_FEET> &r, const std::array<bool, NUM_FEET> &contacts, double dt, bool update_linearization, Eigen::Ref<BMatrixT> B)`**
  - **功能**: 计算输入矩阵 B，用于描述输入（地面反作用力）对状态的影响。
  - **关键说明**: B 矩阵考虑了机器人的质量、惯量、足端位置和接触状态。只有在接触状态下，力才会影响状态。`r` 是足端相对于质心的位置向量。

- **`GetR(double psi, double multiply_with, bool update_linearization, Eigen::Ref<Eigen::Matrix3d> R)`**
  - **功能**: 计算旋转矩阵 R，用于将惯性坐标系转换为世界坐标系。
  - **关键说明**: 基于偏航角 `psi` 计算旋转矩阵，确保动力学方程在正确坐标系中。

- **`GetIWorld(double psi, const Eigen::Ref<const Eigen::Matrix3d> &inertia, Eigen::Ref<Eigen::Matrix3d> IWorld)`**
  - **功能**: 计算世界坐标系下的惯量矩阵。
  - **关键说明**: 将身体坐标系的惯量矩阵旋转到世界坐标系。

- **`GetC(double mu, Eigen::Ref<Eigen::Matrix<double, NUM_POLY_CONST_PER_FOOT * NUM_FEET, INPUT_SIZE>> C)`**
  - **功能**: 计算摩擦锥约束矩阵 C，用于确保足端力在摩擦锥内。
  - **关键说明**: `mu` 是摩擦系数，C 矩阵定义了力约束的多面体。

- **`GetCbounds(double fmin, double fmax, Eigen::Ref<Eigen::Matrix<double, NUM_POLY_CONST_PER_FOOT * NUM_FEET, 1>> c_lb, Eigen::Ref<Eigen::Matrix<double, NUM_POLY_CONST_PER_FOOT * NUM_FEET, 1>> c_ub)`**
  - **功能**: 设置摩擦锥约束的上下界。
  - **关键说明**: `fmin` 和 `fmax` 是最小和最大力，定义了力的范围。

- **`GetUBounds(const std::array<bool, NUM_FEET> &contact, double fmin, double fmax, Eigen::Ref<inputVecT> lb, Eigen::Ref<inputVecT> ub)`**
  - **功能**: 设置输入（力）的上下界。
  - **关键说明**: 根据接触状态设置力的约束，非接触足端力设为零。

### 构造函数和析构函数
- **`MPC::MPC(...)`**
  - **功能**: 初始化 MPC 求解器，包括创建 QP 问题维度、设置求解器选项、分配内存。
  - **关键说明**: 配置 Acados 求解器（如 HPIPM 或 OSQP），设置预测时域、权重矩阵、约束。初始化所有矩阵为默认值。

- **`MPC::~MPC()`**
  - **功能**: 释放 Acados 相关资源。
  - **关键说明**: 清理 QP 维度、配置、选项、输入/输出和求解器内存。

### 更新方法
- **`UpdateState(const StateInterface &quad_state)`**
  - **功能**: 更新当前机器人状态。
  - **关键说明**: 将传入的状态复制到内部状态接口。

- **`UpdateModel(const ModelInterface &quad_model)`**
  - **功能**: 更新机器人模型参数。
  - **关键说明**: 更新质量、惯量等，并标记模型已更新。

- **`UpdateGaitSequence(const GaitSequence &gait_sequence)`**
  - **功能**: 更新步态序列。
  - **关键说明**: 设置支撑/摆动相的计划。

### 设置方法
- **`SetStateWeights(const Eigen::Matrix<double, STATE_SIZE - 1, 1> &weights)`**
  - **功能**: 设置状态权重，用于调整 MPC 目标函数中状态误差的惩罚。
  - **关键说明**: 更新 Q 矩阵的对角线。

- **`SetInputWeights(double alpha)`**
  - **功能**: 设置输入权重，用于调整控制输入的惩罚。
  - **关键说明**: 更新 R 矩阵。

- **`SetFmax(double fmax)`**
  - **功能**: 设置最大力约束。
  - **关键说明**: 更新力上下界。

- **`SetMu(double mu)`**
  - **功能**: 设置摩擦系数。
  - **关键说明**: 更新摩擦锥矩阵。

### 核心求解方法
- **`GetWrenchSequence(WrenchSequence &wrench_sequence, MPCPrediction &state_prediction, SolverInformation &solver_information)`**
  - **功能**: 构建并求解 QP 问题，输出地面反作用力序列和状态预测。
  - **关键说明**: 
    - **构建 QP**: 根据当前状态、模型、步态序列和目标轨迹，计算 A、B 矩阵、约束和目标函数。
    - **求解**: 使用 Acados 调用 QP 求解器（如 HPIPM 或 OSQP）。
    - **输出**: 如果求解成功，提取力序列和状态预测；否则，报告失败。
    - **关键步骤**:
      1. 计算预测时域内的线性化矩阵。
      2. 设置约束（摩擦锥、力界限）。
      3. 定义目标函数（最小化状态误差和控制输入）。
      4. 求解 QP。
      5. 检查解的有效性（NaN 检查）。
      6. 提取结果并计算目标函数值。

## 2. 数据流变化和传输过程

MPC 的数据流是一个典型的预测控制循环：

1. **输入数据**:
   - **状态**: 通过 `UpdateState` 传入当前机器人状态（位置、速度、姿态）。
   - **模型**: 通过 `UpdateModel` 传入机器人模型参数（质量、惯量）。
   - **步态序列**: 通过 `UpdateGaitSequence` 传入支撑/摆动计划。
   - **目标轨迹**: 在 `GetWrenchSequence` 中使用，来自步态序列的目标位置、速度、姿态。

2. **内部处理**:
   - **线性化**: 使用静态方法计算 A、B 矩阵，描述系统动力学。
   - **QP 构建**: 设置权重矩阵 Q、R，约束矩阵 C、上下界。
   - **求解**: Acados 求解器计算最优控制序列。
   - **验证**: 检查解的数值稳定性。

3. **输出数据**:
   - **力序列**: `wrench_sequence` 包含预测时域内的地面反作用力。
   - **状态预测**: `state_prediction` 包含预测的状态轨迹。
   - **求解信息**: `solver_information` 包含求解时间、迭代次数、成功标志等。

数据传输通过方法调用和引用传递，确保高效的内存使用。

## 3. 其他有价值的信息

- **MPC 在四足机器人中的作用**: MPC 提供了一个全局优化框架，能够同时考虑多个时间步的控制输入，确保平稳的运动和力分布。它特别适合处理多足机器人的复杂动力学和约束。
- **Acados 库的使用**: 该文件依赖 Acados 进行高效的 QP 求解，支持多种求解器（如 HPIPM 用于速度，OSQP 用于鲁棒性）。配置选项允许调整求解精度和速度。
- **实时性考虑**: MPC 计算在预测时域内进行，时间复杂度取决于时域长度和求解器。代码中使用了部分凝聚（partial condensing）来加速求解。
- **数值稳定性**: 代码包含 NaN 检查和残差计算，确保解的可靠性。
- **扩展性**: 通过接口设计，MPC 可以轻松集成不同的模型和求解器。
- **调试支持**: 代码中包含调试打印（在 `DEBUG_PRINTS` 定义时启用），有助于分析求解过程。

此分析基于代码结构和注释，MPC 是控制系统中的关键组件，确保机器人能够高效、安全地执行复杂任务。