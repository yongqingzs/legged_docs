# `LeggedInterface` 类分析

`LeggedInterface` 类是 OCS2 (Optimal Control for Switched Systems) 框架与特定机器人（在这里是四足机器人）之间的核心桥梁。它继承自通用的 `RobotInterface`，其主要职责是**解析配置文件**（URDF、任务描述文件等），并基于这些配置**构建一个完整的最优控制问题 (`OptimalControlProblem`)**。

可以把这个类理解为一个**工厂**，它负责生产和组装 MPC (模型预测控制) 求解器所需的所有部件，如动力学模型、代价函数、约束条件和参考管理器等。

## 1. 类方法功能

-   **`LeggedInterface(...)` (构造函数)**
    -   **功能**: 类的入口，负责加载所有顶层配置文件和设置。
    -   **执行细节**:
        1.  检查 `task_file`、`urdf_file` 和 `reference_file` 是否存在。
        2.  从 `task_file` 中加载 `model_settings` (模型设置)、`mpc_settings` (MPC 设置)、`sqp_settings` (SQP 求解器设置) 和 `rollout_settings` (前向推演设置)。这些设置定义了控制器的核心参数，如 MPC 频率、优化迭代次数、积分器类型等。

-   **`setupJointNames(...)`**
    -   **功能**: 设置模型的关节名和足端名。
    -   **执行细节**: 这是一个必须在 `setupOptimalControlProblem` 之前调用的配置函数，它将从外部（`CtrlComponent`）传入的关节和足端名称列表存储到 `model_settings_` 中，供后续的模型构建使用。

-   **`setupOptimalControlProblem(...)`**
    -   **功能**: 这是该类最核心的方法，它按照特定顺序调用其他辅助方法来构建和组装整个最优控制问题。
    -   **执行细节**:
        1.  **`setupModel(...)`**: 创建机器人模型，包括完整的刚体动力学模型 (`PinocchioInterface`) 和 MPC 使用的简化质心动力学模型 (`CentroidalModelInfo`)。
        2.  加载初始状态 `initial_state_`。
        3.  **`setupReferenceManager(...)`**: 创建 `SwitchedModelReferenceManager`，用于管理目标轨迹和步态计划。
        4.  **创建 `OptimalControlProblem` 实例**: `problem_ptr_` 被创建，后续的所有组件都将添加到这个实例中。
        5.  **设置动力学**: 创建并设置 `LeggedRobotDynamicsAD`，它使用自动微分 (AD) 来计算动力学方程的导数。
        6.  **设置代价函数**: 调用 `getBaseTrackingCost` 创建一个二次代价函数，用于惩罚状态和输入与参考轨迹的偏差，并将其添加到 `problem_ptr_->costPtr`。
        7.  **设置约束**: 循环遍历每个足端，添加多种约束：
            -   **摩擦锥约束**: 确保支撑腿的接触力在摩擦锥内（可以是硬约束或软约束）。
            -   **零力约束**: 确保摆动腿的接触力为零。
            -   **零速约束**: 确保支撑腿的足端速度为零。
            -   **法向速度约束**: 确保摆动腿在接触地面时的法向速度不为正（即不能穿透地面）。
        8.  **设置自碰撞约束**: 添加一个状态软约束，以避免机器人腿部之间的碰撞。
        9.  **设置预计算模块**: 创建 `LeggedRobotPreComputation`，用于在每次 MPC 迭代前预计算一些常用量（如雅可比矩阵），以提高效率。
        10. **设置 Rollout**: 创建 `TimeTriggeredRollout`，用于在 MPC 中前向积分动力学模型。
        11. **设置 Initializer**: 创建 `LeggedRobotInitializer`，用于为 MPC 的每次求解提供一个良好的初始猜测值。

-   **`setupModel(...)`**
    -   **功能**: 负责从 URDF 文件中加载机器人模型，并创建两种不同的模型表示。
    -   **执行细节**:
        1.  调用 `centroidal_model::createPinocchioInterface`，解析 URDF 文件，创建一个 `PinocchioInterface` 实例。这是一个包含完整机器人运动学和动力学信息的对象。
        2.  调用 `centroidal_model::createCentroidalModelInfo`，基于 `PinocchioInterface` 创建一个 `CentroidalModelInfo` 实例。这是一个简化的质心动力学模型，它描述了机器人的状态维度、输入维度、接触点等信息，专门供 MPC 使用。

-   **`setupReferenceManager(...)`**
    -   **功能**: 创建并配置参考管理器。
    -   **执行细节**:
        1.  创建一个 `SwingTrajectoryPlanner`，用于生成摆动腿的轨迹。
        2.  创建一个 `SwitchedModelReferenceManager`，并将 `SwingTrajectoryPlanner` 和一个从 `reference_file` 加载的 `GaitSchedule` (步态计划) 传递给它。

-   **各种 `get...` 和 `load...` 辅助方法**
    -   **功能**: 这些方法是工厂函数，负责创建或加载最优控制问题的特定组件（如某个约束、代价项或配置文件中的参数）。这种设计使得 `setupOptimalControlProblem` 的逻辑更清晰，并且易于扩展和修改。

## 2. 关键数据结构与转换

`LeggedInterface` 在初始化过程中执行了多次重要的数据转换，将人类可读的配置文件转换为求解器使用的高效数据结构。

1.  **URDF -> `PinocchioInterface`**
    -   **输入**: URDF (Unified Robot Description Format) 文件，这是一个描述机器人连杆、关节、惯量、碰撞体等物理属性的 XML 文件。
    -   **处理**: Pinocchio 库解析这个文件，在内存中构建一个高效的、基于 C++ 对象的刚体动力学模型。
    -   **输出 (`pinocchio_interface_ptr_`)**: 一个 `PinocchioInterface` 对象，它提供了用于计算正运动学、雅可比矩阵、动力学等各种机器人学算法的接口。

2.  **`PinocchioInterface` -> `CentroidalModelInfo`**
    -   **输入**: `PinocchioInterface` (完整模型) 和一些配置参数（如质心模型类型）。
    -   **处理**: 从完整模型中提取或计算出简化模型所需的关键信息。这是一个**降维**过程，将机器人的状态从完整的关节空间和浮动基座（通常 > 30 维）简化为质心动力学模型（24 维）。
    -   **输出 (`centroidal_model_info_`)**: 一个 `CentroidalModelInfo` 对象，它定义了 MPC 优化的状态和输入向量的维度和物理意义。
        -   **状态向量 (24维)**: `[质心动量(6), 基座姿态(6), 关节角度(12)]`
        -   **输入向量 (24维)**: `[足端接触力(12), 关节速度(12)]`

3.  **`.info` 配置文件 -> C++ 对象**
    -   **输入**: `.info` 格式的文本文件，其中以键值对的形式存储了大量的参数，包括代价矩阵 `Q` 和 `R`、MPC 频率、步态序列等。
    -   **处理**: `loadData` 系列函数解析这些文件。
    -   **输出**:
        -   **`Eigen::Matrix`**: 代价矩阵 `Q` 和 `R` 被加载为 Eigen 库中的矩阵对象。
        -   **`struct`**: 像 `ModelSettings`, `mpc::Settings` 这样的结构体被文件中的相应参数填充。
        -   **`ModeSchedule`**: 步态定义被解析成 OCS2 中的模式计划对象。

## 3. 其他说明

-   **配置驱动的设计**: `LeggedInterface` 的核心设计思想是“配置驱动”。几乎所有的行为，从动力学模型的选择到代价函数的权重，再到约束的参数，都是通过外部的 `.info` 文件来定义的。这使得算法的调试、调整和对不同机器人的适配变得非常高效，无需重新编译代码。
-   **一次性构建**: `LeggedInterface` 的主要作用在**初始化阶段**。它构建出 `OptimalControlProblem` 对象后，这个对象就会被传递给 MPC 求解器。在运行过程中，控制器主要与 `ReferenceManager` 和求解器本身交互，而很少直接与 `LeggedInterface` 交互。
-   **自动微分 (AD) 的应用**: 该接口通过 `LeggedRobotDynamicsAD` 和 `PinocchioEndEffectorKinematicsCppAd` 等类利用了 CppAD 库。这意味着开发者只需要定义动力学和运动学的正向计算，其导数（雅可比矩阵等）将由库自动计算，极大地简化了复杂动力学模型的实现。
