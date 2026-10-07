# 转向优先跟踪模式与 Nav2 局部控制器对比

本文说明本地 `local_planner` 中 `tracking_mode: "turn_preferred"` 的工作方式，并对比 ROS 2 Jazzy 对应的 Nav2 常见控制器。目标是满足轮足机器狗的运动偏好：普通纠偏更多依靠航向调整，同时保留横移能力，支持死胡同原地掉头和必要的避障平移。

## 1. turn_preferred 改变了哪个环节

当前导航链路为：

```text
全局路径 /plan
  → localPlanner：结合地形选择可行局部路径 /local_path
  → pathFollower：跟踪局部路径，输出 vx、vy、角速度 /cmd_vel
```

本次转向优先模式主要改变 `pathFollower` 的跟踪控制。`localPlanner` 仍然负责选择可行局部路径；新模式决定机器人如何执行这条路径。

模式在 `pathFollower.ros__parameters` 下配置：

```yaml
pathFollower:
  ros__parameters:
    tracking_mode: "turn_preferred"
```

`omni` 为原有全向跟踪模式，也是代码未提供该参数时的默认值；`turn_preferred` 为新增模式。参数在启动时读取，修改后需要重启节点，并确保加载的是修改后的配置文件和新版程序。遥控模式 `NavType=2` 仍使用原有控制分支。

## 2. 如何让机器人更倾向调整航向

### 2.1 横向位置误差也会产生转向动作

控制器先找到机器人在局部路径上的最近投影，再沿路径向前选择一个前视点。

```text
航向误差 = 机器人指向前视点的方向 − 当前车头方向
```

例如，机器人位于直线路径右侧，即使车头已经与路径平行，前视点仍在左前方。控制器会向左转，依靠前进逐渐回到路径，然后重新摆正车头。

这是减少横移的关键：横向位置偏移也会通过航向调整纠正。正常跟踪根据前视点的位置计算方向；接近需要最终朝向对齐的目标时，才改用目标朝向。

前视距离按速度变化：

```text
前视距离 = clamp(lookaheadTime × 当前控制速度大小,
                 turn_min_lookahead,
                 turn_max_lookahead)
```

实现中使用上一周期的内部平移控制速度计算前视距离。

### 2.2 降低普通跟踪中的横向速度

普通跟踪中的期望速度大致按以下关系生成，随后再经过速度、加减速度限制和输出死区处理：

```text
角速度 = yawRateGain × 航向误差

航向减速系数 = max(0, cos(航向误差))

前向速度 = 基础速度 × 航向减速系数 × cos(航向误差)

横向速度 = 基础速度 × 航向减速系数 × sin(航向误差)
           × turn_lateral_weight
```

基础速度还受当前模型限速、距局部路径末点的距离和减速参数影响。航向误差越大，普通行进速度越低。

本次说明所核对的本地 D1M 配置中：

```yaml
turn_lateral_weight: 0.25
yawRateGain: 1.5
```

`0.25` 表示将新控制器在普通跟踪中计算出的横移分量乘以四分之一。它不表示最终横移速度一定是旧 `omni` 模式的四分之一，因为两个模式的控制计算不同，还受到限速、加速度和输出死区影响。

### 2.3 航向差较大时，先减速再原地转身

```yaml
turn_rotate_enter_deg: 60.0
turn_rotate_exit_deg: 20.0
```

普通跟踪时，航向误差达到 60°，进入转身状态，先降低平移速度，再原地转向；误差减小到 20°以内，退出转身状态，恢复行进跟踪。

进入和退出使用不同阈值，避免在一个角度附近反复切换。死胡同需要调头时，可以通过这套机制先转身再前进。

### 2.4 真正需要横移时，恢复横移能力

| 情况 | 处理方式 |
| --- | --- |
| 转向方案预测会碰撞 | 评估向两侧转向、完整横移、减速、等待和后退，先检查整段轨迹安全，再评分。 |
| 外部发布 `/path_follower/lateral_override=true` | 暂时放开横移权重，仍受限速、加速度和碰撞检查约束。 |
| 接近需要最终朝向对齐的目标 | 恢复完整平移分量，方便同时调整位置和朝向。 |

接近最终目标的处理由 `turn_final_align_distance` 控制，当前值为 `0.3 m`。终点位置和朝向判据仍受 `stopDisThre`、`approveYawDeg` 及上层任务语义影响。

静态场景使用 `turn_collision_horizon: 0.6`，运动障碍使用 `turn_prediction_horizon: 2.5`。检查矩形车体沿整段预测轨迹是否接触地形点，包含实际限速、加减速度和死区。普通轨迹安全时直接使用；否则最多评估 36 条候选。横移量和动作变化都是软代价，碰撞是硬约束。

全部候选动作均不可行，或者地形输入缺失、超时、格式或坐标系不满足要求，会输出零速度并标记 blocked；这不能保证移动障碍不会继续撞向机器人。

外部横移放行示例：

```bash
ros2 topic pub -r 10 /path_follower/lateral_override \
  std_msgs/msg/Bool '{data: true}'
```

请求有效期由 `turn_lateral_override_timeout` 控制，当前为 `0.5 s`，需要持续发送；发送 `false` 可取消。该接口尚未连接自动紧急避障检测器。当前自动横移回退来自碰撞预测，不能等同于完整的紧急事件识别。

### 2.5 主要调节参数

| 参数 | 本次核对值 | 调节含义 |
| --- | ---: | --- |
| `turn_lateral_weight` | `0.25` | 降低后普通跟踪更少横移；恢复横移的条件仍然有效。 |
| `yawRateGain` | `1.5` | 增大后转向纠偏更积极，过大可能振荡。 |
| `turn_min_lookahead` | `0.1 m` | 前视距离下限，影响低速和起步时的纠偏。 |
| `turn_max_lookahead` | `2.0 m` | 前视距离上限；前视较远通常更平缓，但急弯容易切角。 |
| `lookaheadTime` | `2.5 s` | 随速度增加前视距离的系数，结果仍受前视上下限约束。 |
| `turn_rotate_enter_deg` | `60°` | 达到该航向误差时进入转身状态。 |
| `turn_rotate_exit_deg` | `20°` | 转身后误差降低到该范围时恢复行进。 |
| `turn_final_align_distance` | `0.3 m` | 接近最终末点时，开始考虑目标朝向并恢复完整平移分量。 |
| `turn_collision_horizon` | `0.6 s` | 候选指令的短时碰撞预测范围。 |
| `turn_collision_margin` | `0.05 m` | 矩形车体额外碰撞余量。 |

最终运动仍受当前模型的 `maxSpeed`、`maxSpeedY`、`maxYawRate` 和各方向加减速度约束。碰撞检查的 `vehicleLength`、`vehicleWidth` 应与实机和局部规划器保持一致。

该模式保留前视反馈控制，在预测风险时使用类似 DWB 的有限轨迹搜索。`turn_lateral_cost: 0.6` 表示横移软代价；不会禁止必要的横移。旧的固定侧移与单点释放规则已经删除。

## 3. Nav2 自带的常见局部控制器

在 Nav2 中，这类插件通常称为 Controller：接收路径，结合机器人状态和局部障碍信息，输出速度。以下按 ROS 2 Jazzy 对应的官方实现说明。

| 控制器 | 设计思路，直白地说 | 运动特点 | 适用机器人 |
| --- | --- | --- | --- |
| **DWB** | 尝试一批 `vx、vy、角速度`，预测每组速度产生的轨迹，根据障碍、路径偏差、目标距离等打分。 | 行为取决于评分权重；支持前进、转向和横移，调参不当可能摆动。 | 原生支持差速、全向；现成轨迹生成器不直接面向阿克曼。 |
| **MPPI** | 预测未来一段时间的速度序列，采样大量候选轨迹，按代价更新整段控制方案，每周期执行其中第一步。 | 能综合权衡绕障、跟踪、转向和横移；参数和算力要求较高。 | 原生支持差速、全向、阿克曼。 |
| **RPP：Regulated Pure Pursuit** | 追踪路径前方的一个点，根据几何关系转向；急弯、靠障碍、接近目标时减速。 | 跟踪直接、计算轻；主要输出前向和角速度，不利用横移；不会像 MPPI 那样主动搜索绕障轨迹。 | 差速、腿式、阿克曼；全向机器人也能用，但横移能力发挥不出来。 |
| **Graceful Motion Controller** | 根据前方目标的位置和朝向，使用平滑反馈控制生成弧线运动。 | 强调平滑接近目标，车头自然随运动调整；主要用于跟踪，不是全面的绕障搜索器。 | 主要面向差速机器人；腿式机器人可按这种运动方式执行。 |
| **Rotation Shim** | 先原地转到新路径的大致方向，再交给 DWB、MPPI 等主控制器跟踪。 | 改善起步朝向差较大时的动作；后续横移倾向仍由主控制器决定。 | 能原地旋转的差速、全向、腿式机器人；普通阿克曼不适合。 |

腿式机器人能使用这些控制器，前提是底层能够执行对应速度命令。这些 Nav2 控制器主要处理平面导航，不直接规划落足点、步态和身体平衡。阿克曼机器人通常不能原地转向，使用支持该机型的控制器时，应配置匹配的运动模型或关闭原地转向行为，并使用可执行的路径。

### 3.1 DWB：速度采样和轨迹评分

DWB 延续并扩展 ROS 1 DWA 和轨迹展开的思路。轨迹生成器负责产生候选运动，critic 负责评价障碍、路径距离、目标距离、姿态和振荡等因素。

可以把它理解为：每个周期尝试多种短时运动，选择综合评分最好的一种。它能支持全向机器人，但“优先转向、少横移”是否表现稳定，需要评分项、采样范围和权重共同配合。

插件名称：`dwb_core::DWBLocalPlanner`。

### 3.2 MPPI：预测未来一段运动并优化

MPPI 是模型预测控制的一种方法。它在上一周期的控制序列附近加入随机扰动，通过运动模型预测大量未来轨迹，再根据障碍、路径和目标等代价更新控制序列。

它比较的是未来一段时间内的运动结果，因此能更充分地权衡“先转向再前进”与“直接横移绕开障碍”。支持的模型包括 `DiffDrive`、`Omni`、`Ackermann`。

插件名称：`nav2_mppi_controller::MPPIController`。

### 3.3 RPP：沿路径追踪前视点

RPP 基于 Pure Pursuit，通过前视点计算所需曲率，再生成前向和角速度，并根据曲率、障碍接近程度及目标距离调节速度，还会检查预测运动是否碰撞。

它的重点是可靠地跟踪可执行路径。障碍阻挡路径时，通常依赖上层重新规划或恢复行为，不会搜索大量绕障候选轨迹。它适合作为主要依靠转向跟踪的对照算法，但无法充分保留全向横移能力。

插件名称：`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`。

### 3.4 Graceful：平滑地接近目标位姿

Graceful 使用基于稳定性分析的反馈控制律，把前方运动目标的位置和朝向一起考虑，生成平滑的弧线。其设计面向差速轮式机器人，关注运动连贯性和目标位姿收敛。

插件名称：`nav2_graceful_controller::GracefulController`。

### 3.5 Rotation Shim：原地转向的包装控制器

Rotation Shim 在主控制器前增加原地转向阶段。收到新路径时，如果机器人与路径大致方向相差较大，先转到适合起步的方向，再交给主控制器执行。还可配置在终点接管最终朝向调整。

它本身不决定整段路径中如何分配横移与转向。主控制器仍需配置为符合机器人的运动偏好。

插件名称：`nav2_rotation_shim_controller::RotationShimController`。

### 3.6 常见的 TEB 是否属于 Nav2 自带插件

TEB 的思路是优化带时间信息的轨迹，兼顾障碍、运动约束和耗时。它常出现在局部规划算法对比中，但不属于 Nav2 Jazzy 官方仓库内置的上述控制器，不能直接假设当前系统已安装。

## 4. 针对轮足机器狗的选择建议

### 4.1 当前链路继续验证 turn_preferred

目前可以保留已有的地形点云和运动原语规划链路，验证 `turn_preferred` 在实机上的效果。它的行为容易解释，也方便定位问题：前视点是否合适、转向是否够快、何时触发横移回退，都可以分别观察。

本地地图测试采用真实 ROS 节点配合理想平面运动学仿真，结果不能代替对实机惯性、打滑和通信延迟的验证。本地代码或参数变化也不代表 D1M-B 已经同步并生效。

### 4.2 比较 Nav2 通用控制器时，优先 MPPI 的 Omni 模型

对于“常态更喜欢转向，必要时仍可横移”的需求，MPPI 的 `Omni` 模型保留了所需运动自由度。

希望稳定表达这项偏好，可以在代价函数中对普通横移增加惩罚，同时让碰撞约束和避障需求仍能影响最终选择。可考虑添加横移惩罚 critic，而不是把横移最大速度设为零。

`PreferForwardCritic` 不能等同于专门惩罚横移。减小横移采样幅度也不等于建立了明确的横移代价。若使用 `DiffDrive` 模型，则会去掉横移自由度，无法完整满足该需求。

### 4.3 RPP 作为转向跟踪的对照组

RPP 计算轻、行为直观，适合比较主要通过前进和转向跟踪路径时的表现。但它不能充分利用横移，因此不能完整替代希望保留横移的跟踪方案。

### 4.4 接入 Nav2 Controller 需要调整运行链路

当前 `localPlanner → pathFollower` 是独立节点链路。Nav2 控制器运行在 `controller_server` 中，使用其路径任务、TF、局部代价地图和速度输出接口。

因此，接入 MPPI、DWB 等插件需要调整路径输入、局部代价地图和速度输出接管关系，不能只在现有配置中改一个插件名称。接入后还需明确哪个节点负责最终输出机器人速度，以及如何保留当前任务、终点、地形和模型切换语义。

## 5. 源码与参考资料

本地实现与参数：

- [转向优先控制实现](/home/jazzy/nav_t_ws/src/local_planner/include/turn_preferred.hpp)
- [pathFollower 接入实现](/home/jazzy/nav_t_ws/src/local_planner/src/pathFollower.cpp)
- [D1M 参数文件](/home/jazzy/nav_t_ws/src/local_planner/config/d1m.yaml)
- [local_planner 使用与测试说明](/home/jazzy/nav_t_ws/src/local_planner/README.md)

Nav2 Jazzy 官方源码文档：

- [DWB](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_dwb_controller/README.md)
- [MPPI](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/README.md)
- [Regulated Pure Pursuit](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_regulated_pure_pursuit_controller/README.md)
- [Graceful Motion Controller](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_graceful_controller/README.md)
- [Rotation Shim](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_rotation_shim_controller/README.md)

说明核对日期：2026-10-02。参数值对应核对时的本地配置，后续调整以实际运行节点加载的配置为准。
