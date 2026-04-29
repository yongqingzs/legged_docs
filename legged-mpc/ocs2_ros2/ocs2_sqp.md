# ocs2_sqp 与 ocs2_mpc 分析

## 1. 核心类功能与数据流分析 (以 ocs2_sqp 为视角)

`ocs2_sqp` 是一个基于序列二次规划（Sequential Quadratic Programming, SQP）的非线性模型预测控制（MPC）求解器。它被 `ocs2_mpc` 框架调用，以实现对复杂系统（如四足机器人）的优化控制。

整个数据流和调用链可以概括为：外部系统 -> `MPC_BASE` -> `SqpMpc` -> `SqpSolver`。

### 1.1. `MPC_BASE` (位于 `ocs2_mpc`)

这是一个 MPC 的抽象基类，定义了 MPC 控制器的通用接口。

-   **功能**:
    -   `run(currentTime, currentState)`: 这是 MPC 的主入口函数。它接收外部系统（例如，机器人状态估计模块）提供的**当前时间** (`scalar_t`) 和**当前状态** (`vector_t`)。它的核心逻辑是设置好时间范围（当前时间 + 预测时域），然后调用 `calculateController` 方法。
    -   `calculateController(...)`: 这是一个纯虚函数，由派生类（如 `SqpMpc`）实现。它的职责是调用具体的优化求解器来计算最优控制策略。
    -   `getSolverPtr()`: 获取底层求解器的指针。

-   **数据交互**:
    -   **接收外部数据**: `run` 方法是主要的数据输入接口，接收最关键的外部信息：`currentTime` 和 `currentState`。`currentState` 是一个向量，包含了机器人当前的所有状态量，例如机身的位姿、线速度、角速度、关节角度、关节速度等。
    -   **向下传递数据**: `run` 方法将 `currentTime` 和 `currentState` 传递给 `calculateController`，从而启动一次优化计算。

### 1.2. `SqpMpc` (位于 `ocs2_sqp`)

这个类继承自 `MPC_BASE`，是 `ocs2_mpc` 框架和 `SqpSolver` 求解器之间的桥梁。

-   **功能**:
    -   **构造函数**: 创建一个 `SqpSolver` 的实例。
    -   `calculateController(initTime, initState, finalTime)`: 实现了基类的纯虚函数。它的功能非常直接：调用其内部持有的 `SqpSolver` 实例的 `run` 方法，将初始时间、初始状态和终止时间传递给求解器。

-   **数据交互**:
    -   **接收数据**: 从 `MPC_BASE::run` 接收 `initTime`, `initState`, `finalTime`。
    -   **数据传递**: 不对数据进行处理，直接将其“原封不动”地传递给 `SqpSolver`。

### 1.3. `SqpSolver` (位于 `ocs2_sqp`)

这是 SQP 算法的核心实现，负责求解最优控制问题。

-   **主要方法功能**:
    -   `runImpl(initTime, initState, finalTime)`: 求解器的主执行函数。这是整个优化计算的核心。它接收初始状态和时间，然后执行一个 SQP 迭代循环，直到找到最优解或达到收敛条件。
    -   `setupQuadraticSubproblem(...)`: 在每个 SQP 迭代中，此函数将**非线性**的最优控制问题（包含动力学、成本函数、约束）在当前的轨迹点（状态x, 输入u）附近进行**线性化**，从而构建一个**二次规划（QP）子问题**。这是最核心的数据转换步骤。
    -   `getOCPSolution(...)`: 调用底层的 QP 求解器（如 HPIPM）来求解 `setupQuadraticSubproblem` 构建的 QP 问题。QP 问题的解是状态和输入的**增量** (`deltaXSol`, `deltaUSol`)。
    -   `takeStep(...)`: 根据 QP 求解器给出的增量，使用**线性搜索（Linesearch）**算法来决定一个合适的步长 `alpha`，然后更新当前的状态和输入轨迹 (`x = x + alpha * deltaX`, `u = u + alpha * deltaU`)。
    -   `toPrimalSolution(...)`: 当 SQP 迭代收敛后，此函数将最终优化好的状态和输入轨迹（`x`, `u`）以及计算出的反馈增益（如果启用）打包成一个 `PrimalSolution` 对象。
    -   `getPrimalSolution(...)`: 这是一个对外提供最终结果的接口。外部模块（如 `MPC_BASE` 的调用者）通过这个函数获取 `PrimalSolution`，其中包含了未来一段时间内的最优状态、输入和反馈控制策略。

-   **数据处理与转换流程**:
    1.  **输入**: 接收来自 `SqpMpc` 的初始状态 `initState` (`vector_t`)。
    2.  **初始化**:
        -   **形式**: `initState` 是一个向量。
        -   **转换**: 求解器首先需要为整个预测时域生成一个初始的猜测轨迹（状态序列 `x` 和输入序列 `u`）。如果这是第一次运行或冷启动，它会使用 `Initializer` 来生成一个初始轨迹；否则，它会利用上一次求解的结果 (`primalSolution_`) 作为热启动的初始猜测。
    3.  **核心转换 (非线性 -> QP)**:
        -   **输入**: 当前的状态和输入轨迹 (`vector_array_t x`, `vector_array_t u`)。
        -   **过程**: `setupQuadraticSubproblem` 函数遍历轨迹上的每个时间点，对系统的非线性动力学、成本和约束进行泰勒展开，忽略高阶项，从而得到一个线性的动力学模型和二次的成本/约束模型。
        -   **输出**: 一系列描述 QP 问题的矩阵，如成本函数的二次项（Hessian矩阵）、一次项（梯度向量），以及线性约束的雅可比矩阵等。这些矩阵被存储在 `cost_`, `dynamics_`, `stateInputEqConstraints_` 等成员变量中。
    4.  **QP 求解与结果转换**:
        -   **输入**: 上一步生成的 QP 问题矩阵。
        -   **过程**: `getOCPSolution` 调用 HPIPM 求解器。
        -   **输出**: 求解结果是状态和输入的**增量**序列 (`vector_array_t deltaXSol`, `vector_array_t deltaUSol`)，它指明了当前轨迹应该如何调整才能变得更优。
    5.  **轨迹更新**:
        -   **输入**: 增量序列 `deltaXSol`, `deltaUSol`。
        -   **过程**: `takeStep` 函数通过线性搜索确定最佳步长 `alpha`，并更新轨迹 `x` 和 `u`。
        -   **输出**: 更新后的状态和输入轨迹。
    6.  **最终输出**:
        -   **形式**: 当迭代收敛后，最终的轨迹 `x` 和 `u` 被打包成 `PrimalSolution` 结构体。
        -   **内容**: `PrimalSolution` 包含了 `timeTrajectory_` (时间序列), `stateTrajectory_` (状态序列), `inputTrajectory_` (输入序列), 以及 `controller_` (反馈控制器，包含了前馈输入和反馈增益矩阵 `K`)。这个结构体就是 MPC 对外提供的最终计算结果。

## 2. 是否需要足端传感器数据？

**结论：需要，足端传感器数据对于 MPC 的高性能运行至关重要，但它不是直接输入给 `SqpSolver` 的。**

-   **间接参与**: `SqpSolver` 本身是一个通用的优化求解器，它的 `run` 函数只接收状态和时间。它不直接处理任何传感器的原始数据。
-   **核心作用**: 足端传感器（如力传感器或接触开关）的核心作用是**准确判断足底是否与地面接触**。这个接触状态信息是 MPC 问题建模的**最关键输入之一**。
-   **数据流**:
    1.  足端传感器数据通常被送入**状态估计模块**。
    2.  状态估计模块结合传感器数据、运动学和动力学模型，确定当前的**接触状态**（哪个脚在支撑，哪个脚在摆动）。
    3.  这个接触状态序列（通常称为 `ModeSchedule` 或步态时序）被传递给 `ReferenceManager`。
    4.  当 `SqpSolver` 运行时，它从 `ReferenceManager` 获取 `ModeSchedule`。
    5.  `SqpSolver` 根据 `ModeSchedule` 来构建优化问题。例如：
        -   对于**支撑腿**，它会施加**零速度约束**（脚不能在地面上滑动）。
        -   对于**摆动腿**，它会规划一个无碰撞的轨迹，并可能施加**摆动高度**等约束。
        -   动力学模型也会根据接触状态发生改变。

因此，足端传感器的数据决定了 MPC 优化问题的**模型和约束**。没有准确的接触状态信息，MPC 会基于错误的模型进行优化，导致机器人行走不稳定甚至摔倒。所以，足端传感器数据通过影响 `OptimalControlProblem` 的定义来**必须参与** MPC 的计算。

## 3. 其他应当说明的内容

-   **多线程并行计算**: `SqpSolver` 利用一个线程池 (`ThreadPool`) 在 `setupQuadraticSubproblem` 等计算密集型函数中并行处理不同时间节点的计算任务，这大大加快了求解速度，对于满足 MPC 的实时性要求至关重要。
-   **冷启动与热启动**: `SqpMpc` 的设置中有一个 `coldStart_` 标志。如果为 `false`（即热启动），`SqpSolver` 会利用上一个控制周期的解作为当前周期优化的初始猜测。这可以显著减少收敛所需的迭代次数，是 MPC 在实际应用中的常用技巧。
-   **日志与调试**: `SqpSolver` 提供了丰富的日志功能（`SqpLogging`），可以记录每次迭代的详细信息，如计算时间、步长、成本变化、收敛状态等。这对于算法的调试和性能分析非常有帮助。
-   **反馈策略**: `SqpSolver` 不仅可以计算前馈的输入轨迹 `u(t)`，还可以通过 `useFeedbackPolicy` 选项计算线性反馈增益 `K(t)`。最终的控制器 `u = u_ff - K * (x - x_nominal)` 能够对扰动和模型误差进行补偿，增强鲁棒性。

