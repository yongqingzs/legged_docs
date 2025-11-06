### FSM（有限状态机）的运行机制说明

基于提供的代码（fsm.hpp 和 fsm_go1.hpp），FSM（Finite State Machine）用于管理机器人（如 Go1）的不同行为状态（如被动、起立、行走）。以下是 FSM 的详细运行机制：

#### 1. **FSM 的创建和初始化**
   - **工厂模式注册**：FSM 通过 `FSMFactory` 和 `FSMManager` 管理。`Go1FSMFactory` 继承 `FSMFactory`，定义了 Go1 支持的状态（`RLFSMStatePassive`、`RLFSMStateGetUp`、`RLFSMStateGetDown`、`RLFSMStateRLLocomotion`）。
     - 宏 `REGISTER_FSM_FACTORY(Go1FSMFactory, "RLFSMStatePassive")` 在程序启动时自动注册工厂到 `FSMManager` 单例。
   - **实例创建**：在 `RL_Real` 构造函数中，调用 `FSMManager::GetInstance().CreateFSM("go1", this)` 创建 FSM：
     - 工厂创建所有支持的状态实例，并添加到 FSM 的 `states_` 映射中。
     - 设置初始状态为 `"RLFSMStatePassive"`（通过 `fsm->SetInitialState()`），并调用该状态的 `Enter()` 方法。
   - **上下文传递**：`this`（RL_Real 实例）作为上下文传递给状态类，用于访问 RL 参数、状态和命令。

#### 2. **FSM 的运行循环**
   - **触发点**：FSM 不是独立线程，而是嵌入在主控制循环中。在 `RL::StateController()` 中调用 `fsm.Run()`（每次机器人控制周期执行一次）。
     - `StateController()` 先更新所有状态的 `fsm_state` 和 `fsm_command` 指针（指向当前机器人状态和命令），然后执行 `fsm.Run()`。
   - **运行逻辑**：
     - **NORMAL 模式**：
       - 调用当前状态的 `Run()` 方法（例如，`RLFSMStateRLLocomotion` 的 `Run()` 执行 RL 控制）。
       - 调用 `CheckChange()` 检查是否需要切换状态（基于输入，如键盘 '0' 或手柄 A 键）。
       - 如果需要切换，设置 `mode_` 为 `CHANGE`，并记录下一个状态。
     - **CHANGE 模式**：
       - 调用当前状态的 `Exit()` 方法。
       - 切换到新状态，调用新状态的 `Enter()` 方法。
       - 如果 `Enter()` 中又请求状态切换，则延迟处理（避免递归）。
       - 重置为 `NORMAL` 模式，并执行新状态的 `Run()`。
   - **状态类实现**：
     - 每个状态（如 `RLFSMStatePassive`）实现：
       - `Enter()`：进入状态时的初始化（例如，打印提示信息）。
       - `Run()`：状态运行逻辑（例如，设置电机命令为零阻尼）。
       - `Exit()`：退出状态时的清理。
       - `CheckChange()`：返回下一个状态名称（如果无变化，返回当前状态名）。

#### 3. **状态切换机制**
   - **触发条件**：基于 RL 的输入（如 `rl.control.current_keyboard` 或 `rl.control.current_gamepad`）。
     - 示例：在 `RLFSMStatePassive` 中，按 '0' 或 A 键切换到 `"RLFSMStateGetUp"`。
   - **安全检查**：`RequestStateChange()` 检查状态是否存在，避免无效切换。
   - **插值过渡**：在状态如 `RLFSMStateGetUp` 中，使用 `Interpolate()` 函数平滑过渡关节位置，避免突变。

#### 4. **与 RL 系统的集成**
   - **数据交互**：FSM 通过 `fsm_state` 读取传感器数据，通过 `fsm_command` 设置电机命令。
   - **RL 控制**：在 `RLFSMStateRLLocomotion` 中，`Run()` 调用 `RLControl()`，处理 RL 模型输出（动作 → 关节命令）。
   - **错误处理**：如果 RL 初始化失败，FSM 请求切换到被动状态，确保安全。

#### 5. **关键优势**
   - **模块化**：状态独立，便于扩展（如添加新状态）。
   - **实时性**：运行在控制循环中，确保低延迟。
   - **安全性**：状态切换受输入控制，防止意外行为。

如果需要状态图、代码示例或特定状态的详细说明，请提供更多细节。