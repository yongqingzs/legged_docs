# task.info 参数分析文档

本文档详细分析 `task.info` 配置文件中的各项参数，说明其作用、物理意义、被哪些代码模块使用，并提供相应的调参建议。

## 1. 全局与接口参数

### `centroidalModelType`
```
centroidalModelType             0 
```
-   **使用者**: `LeggedInterface`
-   **作用**: 定义了机器人动力学模型的类型。
-   **物理意义**:
    -   `0`: `FullCentroidalDynamics` - 使用完整的质心动力学模型，该模型考虑了动量变化率，更精确，但计算量稍大。
    -   `1`: `SingleRigidBodyDynamics` - 将机器人简化为单个刚体，忽略了腿部运动对整体动量的影响，计算速度更快，但在高动态运动中精度较低。
-   **调参建议**: 通常保持为 `0` 以获得最佳性能。

### `legged_robot_interface`
```
legged_robot_interface
{
  verbose                               false  // show the loaded parameters
}
```
-   **使用者**: `LeggedInterface`
-   **作用**: 控制接口模块的配置。
-   **参数**:
    -   `verbose`: `true` 时，会在加载参数时打印详细信息，用于调试。

## 2. 模型与代码生成

### `model_settings`
```
model_settings
{
  positionErrorGain             0.0
  phaseTransitionStanceTime     0.1

  verboseCppAd                  true
  recompileLibrariesCppAd       false
  modelFolderCppAd              ocs2_cpp_ad/go2
}
```
-   **使用者**: `LeggedInterface` (加载后传递给 `PinocchioInterface`)
-   **作用**: 配置动力学模型和 CppAD 代码生成。
-   **参数**:
    -   `positionErrorGain`: 用于模型中位置误差的增益，通常在动力学约束中使用。
    -   `phaseTransitionStanceTime`: 步态切换时，定义一个短暂的过渡时间，在此期间腿被认为是处于站立相，以确保平滑过渡。
    -   `verboseCppAd`: `true` 时，打印 CppAD 代码生成的详细日志。
    -   `recompileLibrariesCppAd`: `true` 时，强制重新生成和编译动力学模型的 CppAD 库。
    -   `modelFolderCppAd`: 指定生成的 CppAD 库的存放路径。
-   **调参建议**:
    -   `recompileLibrariesCppAd` 在修改了机器人模型（URDF）或动力学实现后应设为 `true`，平时保持 `false` 以加快启动速度。
    -   `phaseTransitionStanceTime` 可根据实际步态切换的平滑度微调，通常不需要更改。

## 3. MPC 与 SQP 求解器参数

这部分参数是 MPC 性能的核心。

### `sqp` (Multiple-Shooting SQP settings)
```
; Multiple_Shooting SQP settings
sqp
{
  nThreads                              3
  dt                                    0.015
  sqpIteration                          1
  deltaTol                              1e-4
  g_max                                 1e-2
  g_min                                 1e-6
  inequalityConstraintMu                0.1
  inequalityConstraintDelta             5.0
  projectStateInputEqualityConstraints  true
  printSolverStatistics                 true
  printSolverStatus                     false
  printLinesearch                       false
  useFeedbackPolicy                     false
  integratorType                        RK2
  threadPriority                        60
}
```
-   **使用者**: `SqpSolver` (通过 `sqp::loadSettings` 加载)
-   **作用**: 配置序列二次规划（SQP）求解器的行为。
-   **参数分析**:
    -   `nThreads`: **物理意义**: 求解器用于并行计算的线程数。**调参建议**: 通常设置为 CPU 核心数减一，以在获得并行加速的同时为其他系统进程留出资源。
    -   `dt`: **物理意义**: MPC 预测时域内的离散化时间步长。**调参建议**: `dt` 越小，模型精度越高，但计算量越大。它需要与 `timeHorizon` 配合。通常 `0.01` 到 `0.02` 是一个比较合适的值。
    -   `sqpIteration`: **物理意义**: 每个 MPC 周期内，SQP 求解器执行的最大迭代次数。**调参建议**: 对于实时应用，通常设置为 `1`（即实时迭代方案 Real-Time Iteration, RTI），以保证计算时间的可预测性。如果计算性能足够，可以增加迭代次数以获得更优的解。
    -   `deltaTol`: **物理意义**: 收敛判据之一。当状态和输入的更新量（范数）小于此阈值时，认为求解收敛。
    -   `g_max`, `g_min`: **物理意义**: 线性搜索（Linesearch）中用于约束违反的阈值。`g_max` 是可接受的最大约束违反值，`g_min` 是认为约束已被满足的阈值。
    -   `inequalityConstraintMu`, `inequalityConstraintDelta`: **物理意义**: 不等式约束（如摩擦锥）的松弛对数障碍（relaxed log barrier）参数，用于将不等式约束平滑地整合到成本函数中。
    -   `projectStateInputEqualityConstraints`: **物理意义**: 是否使用投影法处理状态-输入等式约束。可以提高求解效率。
    -   `useFeedbackPolicy`: **物理意义**: `true` 时，求解器会计算并使用反馈增益矩阵，使控制器具有反馈鲁棒性。`false` 时，只使用前馈控制。**调参建议**: 在仿真中可以设为 `false` 以简化问题，但在实际硬件上，`true` 通常能带来更好的鲁棒性。
    -   `integratorType`: **物理意义**: 离散化动力学时使用的积分器类型，如 `RK2` (二阶龙格-库塔)。

### `mpc`
```
mpc
{
  timeHorizon                     1.0  ; [s]
  solutionTimeWindow              -1   ; maximum [s]
  coldStart                       false

  debugPrint                      false

  mpcDesiredFrequency             100  ; [Hz]
  mrtDesiredFrequency             1000 ; [Hz] Useless
}
```
-   **使用者**: `MPC_BASE`, `SqpMpc` (通过 `mpc::loadSettings` 加载)
-   **作用**: 配置 MPC 循环的宏观行为。
-   **参数分析**:
    -   `timeHorizon`: **物理意义**: MPC 的预测时域长度（单位：秒）。**调参建议**: 更长的时域可以看到更远的未来，有助于规划更复杂的动作（如大步态），但会显著增加计算负担。通常在 `0.5` 到 `1.5` 秒之间权衡。
    -   `coldStart`: **物理意义**: `true` 时，每次 MPC 迭代都从一个固定的初始猜测开始。`false` 时（热启动），使用上一次迭代的解作为本次迭代的初始猜测。**调参建议**: 强烈建议设为 `false`（热启动），这可以极大减少求解所需的迭代次数，是保证实时性的关键。
    -   `mpcDesiredFrequency`: **物理意义**: MPC 控制循环的期望运行频率（单位：Hz）。这决定了机器人重新规划其动作的频率。**调参建议**: 频率越高，机器人对变化的响应越快，但对计算性能的要求也越高。`100Hz` 是一个常见且有效的值。

## 4. 成本函数权重

成本函数定义了“好”的运动是什么样的。其形式通常为 `J = x'Qx + u'Ru`。

```
; standard state weight matrix
Q
{
  scaling 1e+0

  ;; Normalized Centroidal Momentum: [linear, angular] ;;
  (0,0)   15.0     ; vcom_x
  (1,1)   15.0     ; vcom_y
  (2,2)   100.0    ; vcom_z
  (3,3)   10.0     ; L_x / robotMass
  (4,4)   30.0     ; L_y / robotMass
  (5,5)   30.0     ; L_z / robotMass

  ;; Base Pose: [position, orientation] ;;
  (6,6)   1000.0   ; p_base_x
  (7,7)   1000.0   ; p_base_y
  (8,8)   1500.0   ; p_base_z
  (9,9)   100.0    ; theta_base_z
  (10,10) 300.0    ; theta_base_y
  (11,11) 300.0    ; theta_base_x

  ;; Leg Joint Positions: [FL, RL, FR, RR] ;;
  (12,12) 5.0     ; FL_hip_joint
  (13,13) 5.0     ; FL_thigh_joint
  (14,14) 2.5     ; FL_calf_joint
  (15,15) 5.0     ; RL_hip_joint
  (16,16) 5.0     ; RL_thigh_joint
  (17,17) 2.5     ; RL_calf_joint
  (18,18) 5.0     ; FR_hip_joint
  (19,19) 5.0     ; FR_thigh_joint
  (20,20) 2.5     ; FR_calf_joint
  (21,21) 5.0     ; RR_hip_joint
  (22,22) 5.0     ; RR_thigh_joint
  (23,23) 2.5     ; RR_calf_joint
}
```
### `Q` (state weight matrix)

-   **使用者**: `LeggedInterface`
-   **作用**: 定义了状态误差的二次惩罚权重。
-   **物理意义**: `Q` 矩阵对角线上的每个值，代表了相应状态变量偏离期望值的“代价”。值越大，MPC 就越会努力使该状态变量保持在期望值附近。
    -   `vcom_x, vcom_y, vcom_z`: 质心线速度的权重。
    -   `L_x, L_y, L_z`: 质心角动量的权重（间接控制机身角速度）。
    -   `p_base_x, p_base_y, p_base_z`: 机身位置的权重。
    -   `theta_base_z, theta_base_y, theta_base_x`: 机身姿态（欧拉角）的权重。
    -   `*_joint`: 各个关节角度的权重。
-   **调参建议**:
    -   这是调参的核心部分。首先通过 `scaling` 设置一个基础缩放因子。
    -   提高机身姿态（特别是 `theta_base_y`, `theta_base_x`，即俯仰和滚转角）的权重，可以使机身更稳定。
    -   提高 `p_base_z` 的权重，可以使机身高度更稳定。
    -   关节权重的设置可以鼓励机器人保持在一个较为“自然”的姿态。

### `R` (control weight matrix)

```
; control weight matrix
R
{
  scaling 1e-3

  ;; Feet Contact Forces: [FL, FR, RL, RR] ;;
  (0,0)   1.0       ; front_left_force
  (1,1)   1.0       ; front_left_force
  (2,2)   1.0       ; front_left_force
  (3,3)   1.0       ; front_right_force
  (4,4)   1.0       ; front_right_force
  (5,5)   1.0       ; front_right_force
  (6,6)   1.0       ; rear_left_force
  (7,7)   1.0       ; rear_left_force
  (8,8)   1.0       ; rear_left_force
  (9,9)   1.0       ; rear_right_force
  (10,10) 1.0       ; rear_right_force
  (11,11) 1.0       ; rear_right_force

  ;; foot velocity relative to base: [FL, RL, FR, RR] (uses the Jacobian at nominal configuration) ;;
  (12,12) 5000.0    ; x
  (13,13) 5000.0    ; y
  (14,14) 5000.0    ; z
  (15,15) 5000.0    ; x
  (16,16) 5000.0    ; y
  (17,17) 5000.0    ; z
  (18,18) 5000.0    ; x
  (19,19) 5000.0    ; y
  (20,20) 5000.0    ; z
  (21,21) 5000.0    ; x
  (22,22) 5000.0    ; y
  (23,23) 5000.0    ; z
}
```

-   **使用者**: `LeggedInterface`
-   **作用**: 定义了控制输入的二次惩罚权重。
-   **物理意义**: `R` 矩阵对角线上的值代表了使用相应控制输入的“代价”。值越大，MPC 越倾向于使用更小的控制输入。
    -   `*_force`: 足底接触力的权重。
    -   `foot velocity relative to base`: 摆动腿足端相对于机身的速度权重。这是一种正则化项，用于平滑摆动腿的运动。
-   **调参建议**:
    -   `R` 的 `scaling` 因子是 `Q` 和 `R` 之间的重要平衡。如果 `R` 太小，可能会导致控制输入（特别是接触力）剧烈变化，不平滑。如果 `R` 太大，机器人可能变得“懒惰”，不愿意出力来精确跟踪期望状态。
    -   通常从一个较小的值（如 `1e-3` 到 `1e-4`）开始尝试。

## 5. 任务与约束

### `swing_trajectory_config`
```
swing_trajectory_config
{
  liftOffVelocity               0.05
  touchDownVelocity            -0.1
  swingHeight                   0.08
  swingTimeScale                0.15
}
```
-   **使用者**: `SwingTrajectoryPlanner` (通过 `loadSwingTrajectorySettings` 加载)
-   **作用**: 配置摆动腿轨迹生成器的参数。
-   **参数分析**:
    -   `liftOffVelocity`: **物理意义**: 摆动相开始时，足端的初始垂直速度。
    -   `touchDownVelocity`: **物理意义**: 摆动相结束时，足端的期望垂直速度（通常为负值，表示向下运动）。
    -   `swingHeight`: **物理意义**: 摆动过程中，足端相对于起始点和终点连线的最大抬升高度。
    -   `swingTimeScale`: **物理意义**: 用于调整摆动轨迹形状的时间缩放因子。
-   **调参建议**:
    -   `swingHeight` 是最直观的参数。增加它可以让机器人跨越更高的障碍物，但也会增加能耗和运动的夸张程度。
    -   `liftOffVelocity` 和 `touchDownVelocity` 影响轨迹的平滑性，通常保持默认值即可。

### `frictionConeSoftConstraint`
```
frictionConeSoftConstraint
{
  frictionCoefficient    0.3

  ; relaxed log barrier parameters
  mu                     0.1
  delta                  5.0
}
```
-   **使用者**: `LeggedInterface`
-   **作用**: 配置足底接触力的摩擦锥约束。
-   **物理意义**: 确保足底接触力在摩擦锥内，以防止打滑。这是一个软约束，通过对数障碍函数加入到成本中。
    -   `frictionCoefficient`: 地面的摩擦系数。
    -   `mu`, `delta`: 对数障碍函数的参数。

### `selfCollision`
```
selfCollision
{
  ; Self Collision raw object pairs
  collisionObjectPairs
  {
  }

  ; Self Collision pairs
  collisionLinkPairs
  {
    [0] "FL_calf, FR_calf"
    [1] "RL_calf, RR_calf"
    [2] "FL_calf, RL_calf"
    [3] "FR_calf, RR_calf"
    [4] "FL_foot, FR_foot"
    [5] "RL_foot, RR_foot"
    [6] "FL_foot, RL_foot"
    [7] "FR_foot, RR_foot"
  }

  minimumDistance  0.05

  ; relaxed log barrier parameters
  mu      1e-2
  delta   1e-3
}
```
-   **使用者**: `LeggedInterface`
-   **作用**: 配置机器人自身的碰撞避免约束。
-   **物理意义**: 定义了哪些连杆对之间需要保持最小距离，以防止自碰撞。
    -   `collisionLinkPairs`: 需要检查碰撞的连杆对。
    -   `minimumDistance`: 需要保持的最小安全距离。

### `wbc` (Whole Body Control) 相关参数
```
frictionConeTask
{
  frictionCoefficient    0.3
}

swingLegTask
{
    kp                   350
    kd                   37
}

weight
{
    swingLeg        100
    baseAccel       1
    contactForce    0.05
}
```
-   **使用者**: `WeightedWbc`
-   **作用**: 配置全身控制器（WBC）中各个子任务的参数和权重。WBC 负责将 MPC 计算出的期望接触力和摆动腿加速度，转化为最终的关节力矩。
-   **参数分析**:
    -   `torqueLimitsTask`: 各关节的力矩限制。
    -   `frictionConeTask`: WBC层面再次施加的摩擦锥约束。
    -   `swingLegTask`: 摆动腿跟踪任务的 PD 控制增益 (`kp`, `kd`)。
    -   `weight`: 不同任务之间的权重。这是 WBC 调参的核心，用于平衡不同任务（如：保持机身稳定 vs 精确跟踪摆动腿轨迹 vs 满足接触力）的优先级。

## 6. 状态估计

### `kalmanFilter`
```
; State Estimation
kalmanFilter
{
    footRadius                  0.02
    imuProcessNoisePosition     0.02
    imuProcessNoiseVelocity     0.02
    footProcessNoisePosition    0.002
    footSensorNoisePosition     0.005
    footSensorNoiseVelocity     0.1
    footHeightSensorNoise       0.01
}
```
-   **使用者**: `KalmanFilterEstimate`
-   **作用**: 配置卡尔曼滤波器状态估计器的噪声参数。
-   **物理意义**:
    -   `imuProcessNoisePosition`, `imuProcessNoiseVelocity`: IMU 过程噪声。代表了对 IMU 积分得到的位置和速度的信任程度。值越大，表示越不信任积分结果，而更依赖于其他测量（如运动学）。
    -   `footProcessNoisePosition`: 接触脚的过程噪声。代表了认为接触地面的脚会有多大的滑动。
    -   `footSensorNoisePosition`, `footSensorNoiseVelocity`: 足端位置/速度测量的噪声。代表了通过正向运动学计算出的足端位置/速度的信任程度。
-   **调参建议**:
    -   这些参数的调整需要对卡尔曼滤波有深入理解。
    -   如果估计出的机身漂移严重，可以适当减小 `imuProcessNoise`。
    -   如果在崎岖地面上行走时机身晃动，可能需要调整 `footProcessNoise` 和 `footSensorNoise`。

## 7. 初始状态

### `initialState`
```
initialState
{
   ;; Normalized Centroidal Momentum: [linear, angular] ;;
   (0,0)  0.0     ; vcom_x
   (1,0)  0.0     ; vcom_y
   (2,0)  0.0     ; vcom_z
   (3,0)  0.0     ; L_x / robotMass
   (4,0)  0.0     ; L_y / robotMass
   (5,0)  0.0     ; L_z / robotMass

   ;; Base Pose: [position, orientation] ;;
   (6,0)  0.0     ; p_base_x
   (7,0)  0.0     ; p_base_y
   (8,0)  0.35     ; p_base_z
   (9,0)  0.0     ; theta_base_z
   (10,0) 0.0     ; theta_base_y
   (11,0) 0.0     ; theta_base_x

   ;; Leg Joint Positions: [FL, RL, FR, RR] ;;
   (12,0) -0.0   ; FL_hip_joint
   (13,0)  0.72   ; FL_thigh_joint
   (14,0) -1.44   ; FL_calf_joint
   (15,0) -0.0   ; RL_hip_joint
   (16,0)  0.72   ; RL_thigh_joint
   (17,0) -1.44   ; RL_calf_joint
   (18,0)  0.0   ; FR_hip_joint
   (19,0)  0.72   ; FR_thigh_joint
   (20,0) -1.44   ; FR_calf_joint
   (21,0)  0.0   ; RR_hip_joint
   (22,0)  0.72   ; RR_thigh_joint
   (23,0) -1.44   ; RR_calf_joint
}
```
-   **使用者**: `LeggedInterface`
-   **作用**: 定义了系统启动时的初始状态。
-   **物理意义**: 这是一个 24x1 的向量，详细定义了机器人的初始姿态。包括质心动量、机身位姿和所有关节的角度。
-   **调参建议**: 这个值应该与机器人实际的站立姿态匹配，否则机器人启动时会因状态不匹配而摔倒。
