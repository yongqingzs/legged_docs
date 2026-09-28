# 全局规划器路径对比

`compare_global_planners.py` 会在独立、仅本机可发现的 ROS Domain 中启动临时 Nav2 `map_server`、`planner_server` 和生命周期管理器，逐个调用真实的 `/compute_path_to_pose` action。脚本不会向机器狗下发导航目标，也不会修改本包的启动参数或 D1M-B 上正在运行的导航服务。

## 环境准备

运行机器需要安装 ROS 2 Jazzy、待测试的 Nav2 规划器插件，以及 Python 包 `rclpy`、`PyYAML`、`numpy`、`Pillow`。地图 YAML 和其中引用的图片必须位于运行脚本的机器上。

如果在当前工作站测试 D1M-B 的 `VVY4GdI` 地图，可先把地图文件复制到仓库之外的临时目录：

```bash
mkdir -p /tmp/VVY4GdI
scp -P 20004 cat@47.99.202.196:/home/cat/Workspace/Maps/VVY4GdI/map_000.yaml /tmp/VVY4GdI/
scp -P 20004 cat@47.99.202.196:/home/cat/Workspace/Maps/VVY4GdI/map_000.png /tmp/VVY4GdI/
```

## 运行示例

在本仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/compare_global_planners.py \
  --map /tmp/VVY4GdI/map_000.yaml \
  --start 90,81 --goal 210,123 --pixels \
  --output /tmp/planner-comparison \
  --allow-unknown false

# 例子1
python3 scripts/compare_global_planners.py \
  --map /home/jazzy/nav_t_ws/src/multi_map_nav_ros2/tmp/VVY4GdI/map_000.yaml \
  --start 95,85 --goal 215,125 --pixels \
  --planners smac2d,theta \
  --output /home/jazzy/nav_t_ws/src/multi_map_nav_ros2/tmp/VVY4GdI/planner-comparison \
  --allow-unknown false
```

示例中的 `(90,81)` 和 `(210,123)` 是图片左上角为原点的像素坐标，仅用于演示脚本运行；实际比较时请换成目标场景的起终点。省略 `--pixels` 时，坐标使用 ROS `map` 坐标系下的米，格式为 `x,y` 或 `x,y,yaw`，其中 yaw 单位为弧度。负坐标建议写成 `--start=-9.95,5.05`。

`--map` 也可以指定地图目录；此时用 `--map-name` 选择 YAML 文件名（不含 `.yaml`，默认 `map_000`）。脚本会检查起终点是否落在图片范围内；点被占据或路径不可达时，Nav2 的错误信息会写入 `results.json`。

## 规划器与参数

默认测试五个规划器：`navfn`、`smac2d`、`hybrid`、`lattice`、`theta`。用 `--planners smac2d,theta` 可只测试其中两个。默认 footprint、膨胀参数和 `allow_unknown: true` 与当前 `new_local.yaml` 的全局代价地图配置一致。比较盖板路线时，建议先使用 `--allow-unknown false`，再观察路径与盖板边界的距离。

### 五种规划器直观区别

| 名称 | 直白地说 | 路径通常有什么特点 |
| --- | --- | --- |
| `navfn` | 在栅格地图上找一条代价较低的路。本脚本启用 A* 搜索；它不考虑机器人能否按某个转弯半径转过去。 | 往往比较直接、较短；遇到转角可能贴近障碍或边界，路线形状也会受栅格影响。 |
| `smac2d` | 也是二维栅格 A*，但更重视代价地图中的通行代价，并对结果做平滑。本脚本设置 `cost_travel_multiplier: 10.0`。 | 如果边缘有膨胀代价，通常愿意绕一点路来离边缘远些；如果边缘没有代价，它也可能贴边。 |
| `hybrid` | 搜索时同时考虑位置和朝向，让路径符合设定的转弯模型。本脚本默认 Reeds-Shepp、最小转弯半径 `0.5 m`。 | 通常由更连续的弧线和转弯组成，可能比二维路径长；狭窄处可能因转弯约束而绕路或无解。其倒车能力只是模型假设，不代表机器狗会按这条路径倒走。 |
| `lattice` | 从预先生成的一组运动片段中拼出路线。本脚本使用 Nav2 的 5 cm 全向运动原语。 | 路线会受运动片段形状影响，可能出现平移和弧线组合；与 `hybrid` 的区别取决于所选原语文件，换一套原语可能得到明显不同的路径。 |
| `theta` | 在搜索过程中尝试让相互可见的点直接连线，减少沿栅格逐格转弯。 | 常出现较长的直线段和较少的拐点；直线“抄近路”也可能靠近边缘，不能仅凭线条笔直判断更安全。 |

这些是**常见倾向，不是固定结果**：同一插件在不同地图、起终点、footprint、未知区域设置和膨胀代价下可能画出完全不同的路线。五种规划器都不会自动把“盖板中线”当成目标。比较机器狗在 1 m 盖板上的安全性时，除了看路径长度和耗时，还应看路径到盖板边缘的最小距离，以及实际局部路径和落脚位置；本脚本目前只输出全局路径。

Lattice 默认使用 Nav2 提供的 **5 cm 全向运动原语**。除非通过 `--lattice-file` 指定与新分辨率匹配的原语文件，否则 `--costmap-resolution` 应保持默认的 `0.05`。Hybrid 默认使用 Reeds-Shepp 模型、最小转弯半径 `0.5 m`；可通过 `--hybrid-model` 和 `--turning-radius` 调整。

当前 footprint 下，Nav2 会提示默认 `0.25 m` 膨胀半径小于约 `0.28 m` 的外接半径，可能降低足迹碰撞检测的效率。比较另一组代价地图参数时，可以另跑一次 `--inflation-radius 0.35`。更换此参数后，比较结果属于**不同代价地图策略**，应与单纯更换规划算法的结果分开看。

## `new_local.yaml` 参数说明

以下数值对应本仓库的 `params/new_local.yaml`。它同时配置 Nav2 的全局代价地图、局部代价地图、全局规划器、控制器、行为树导航器，以及本包的多地图调度节点。**改 YAML 不会让已启动的节点自动重新读取文件**；在真机上切换配置应先停止当前导航任务，再按部署流程重启相关节点。本地文件与 D1M-B 上的同名文件也不会自动同步。

### 全局代价地图：`global_costmap.global_costmap.ros__parameters`

| 参数及当前值 | 含义 | 如何调整 |
| --- | --- | --- |
| `update_frequency: 2.0` | 每秒尝试更新代价地图 2 次。 | 地图变化快、更新跟不上时再提高；提高会增加 CPU 负载。它不是规划器搜索频率。 |
| `publish_frequency: 1.0` | 每秒最多发布 1 次代价地图供查看或订阅。 | 需要更及时地看图时提高；不等于提高实际规划频率。 |
| `global_frame: map`、`robot_base_frame: base_link` | 地图和机器人基座使用的 TF 坐标系。 | 必须与实际 TF 一致；不能用改名字的方式修正定位误差。 |
| `footprint: [[0.045, ±0.115], [-0.225, ±0.115]]` | 规划碰撞检查用的机器人平面外廓：前后约 `0.27 m`、左右约 `0.23 m`。 | 核对实机机身、摆腿及可能落脚的最大范围。当前配置的宽度不能直接当成足端安全宽度；改大可能使窄道无解，改小会漏掉碰边风险。 |
| `footprint_padding: 0.02` | 在 footprint 外再加 `0.02 m` 缓冲。 | 按定位误差、地图误差和跟踪误差逐步增加；要用实测误差决定，不能只靠这 2 cm 保证不脱足。 |
| `resolution: 0.05` | 请求的代价地图栅格大小，单位米。 | 更小的栅格能描述窄边界，但增加计算量；静态层加载地图后可能按地图元数据调整尺寸和分辨率，最终以实际 `/global_costmap/costmap` 为准。 |
| `track_unknown_space: true` | 保留未知区，而不是直接当作空地。 | 建议保留；是否允许规划穿越未知区还由下文 `allow_unknown` 控制。 |
| `always_send_full_costmap: true` | 发布完整代价地图，而非主要发送局部更新。 | 通常只影响通信/可视化负载，贴边问题先不调它。 |
| `plugins: [static_layer, inflation_layer]` | 全局图只叠加静态地图和膨胀层，**没有启用动态障碍层**。 | 需要全局路径主动绕开实时障碍时，要设计并验证传感器层；当前配置不能靠调此表中其他参数获得动态避障。 |
| `static_layer.plugin: nav2_costmap_2d::StaticLayer`、`map_subscribe_transient_local: true` | 从 map server 接收静态栅格图，并允许新订阅者收到最近一张地图。 | 一般保持；若地图与代价地图不一致，先查 map server 和地图切换结果。 |
| `inflation_layer.plugin: nav2_costmap_2d::InflationLayer` | 在障碍物附近生成高代价区域。 | 保持启用，结合下面两个参数调整。 |
| `inflation_layer.inflation_radius: 0.25` | 从障碍边界向外影响的半径，单位米。 | 增大可让障碍影响更远的路径，但过大可能堵死 1 m 盖板通道。当前半径小于本 footprint 约 `0.28 m` 的外接半径，可离线比较 `0.30`、`0.35` 等设置。 |
| `inflation_layer.cost_scaling_factor: 5.0` | 膨胀代价随距离降低的速度。**越大，代价下降越快**。 | 想让离障碍较远处仍有代价，可适当调小；过小可能让整条窄通道都处于高代价。每次只改一项再对比路径。 |

盖板边缘若在地图中是未知值 `-1`，当前 `allow_unknown: true` 仍可能让规划器穿过它；而膨胀层默认不把未知区当作膨胀源。先核实边缘在 `/map` 和 `/global_costmap/costmap` 中的实际值，再测试 `allow_unknown: false`。如需在未知区边缘形成代价带，可另外评估膨胀层的 `inflate_around_unknown: true`；此项**不在当前 YAML 中**，必须确认插件版本支持并离线验证效果。软代价不能替代足端不得越界的硬约束。

### 局部代价地图：`local_costmap.local_costmap.ros__parameters`

| 参数及当前值 | 含义 | 如何调整 |
| --- | --- | --- |
| `update_frequency: 1.0`、`publish_frequency: 1.0` | 局部代价地图更新和发布频率，单位 Hz。 | 当前局部地图几乎不处理障碍；调高频率不能让它自动感知盖板边缘。 |
| `global_frame: odom`、`robot_base_frame: base_link` | 局部地图使用的 TF 坐标系。 | 与实际 TF 保持一致。 |
| `rolling_window: false`、`width: 1`、`height: 1` | 固定的 `1 m × 1 m` 局部地图，不随机器人滚动。 | 如果将来启用局部障碍层，需要重新设计窗口范围、滚动模式和更新频率；不要只改宽高。 |
| `resolution: 0.05`、`footprint`、`footprint_padding: 0.02` | 局部地图栅格大小及机器人外廓；footprint 与全局配置相同。 | 真正启用局部碰撞检查时，需与实机和全局安全边界一并核对。 |
| `min_obstacle_height: -0.4`、`max_obstacle_height: 0.5` | 预留的障碍点高度范围，单位米。 | 当前没有启用障碍层，调这两个值不能改变路径；启用传感器障碍层后再按传感器高度调整。 |
| `always_send_full_costmap: false` | 局部代价地图可发布增量更新。 | 主要影响通信量，非贴边调参项。 |
| `plugins: [static_layer]`、`static_layer.enabled: false` | 唯一列出的局部图层被禁用。 | 当前 Nav2 局部代价地图不承担盖板边缘避障；实际局部路线要查独立的 `localPlanner`、`pathFollower` 配置与输出。 |

### 全局规划器：`planner_server.ros__parameters`

| 参数及当前值 | 含义 | 如何调整 |
| --- | --- | --- |
| `expected_planner_frequency: 2.0` | 用于监测规划耗时的期望频率，`2 Hz` 对应约 `0.5 s` 的周期。 | 主要是性能告警阈值；调大它不会让 A* 自动更快。 |
| `planner_plugins: [GridBased]`、`GridBased.plugin: nav2_smac_planner::SmacPlanner2D` | 注册名为 `GridBased` 的全局规划器，并指定当前插件。 | 换算法时改插件类型及该插件适用的参数；上方带 `#` 的 Navfn 示例是注释，不会生效。 |
| `GridBased.tolerance: 0.5` | 精确终点不可达时，允许在目标附近寻找可接受终点的距离，单位米。 | 1 m 宽盖板上 `0.5 m` 可能允许规划结果停在偏离目标的位置；宜先以更小值离线验证，避免用它掩盖目标点被占据的问题。 |
| `downsample_costmap: false`、`downsampling_factor: 1` | 是否降低代价地图分辨率进行搜索；当前没有降采样。 | 大图搜索过慢时才尝试；在窄盖板上降采样可能抹掉边界细节。 |
| `allow_unknown: true` | 允许搜索经过未知栅格。 | 对已标好通行区的盖板，优先比较 `false`；先看未知区是否正是盖板外侧。 |
| `max_iterations: 1000000` | 搜索展开次数上限。 | 频繁报迭代超限时再评估地图规模与此值；调高不会改善贴边。 |
| `max_on_approach_iterations: 1000` | 接近终点时，为寻找容差内可达位置允许的额外搜索次数。 | 与 `tolerance` 一起看；目标附近经常失败时先检查目标是否确实在安全可通行区。 |
| `max_planning_time: 5.0` | 单次搜索及平滑可用的时间预算，单位秒。 | 有超时才考虑增大，同时查代价地图和 CPU；增加预算不改变安全偏好。 |
| `cost_travel_multiplier: 10.0` | Smac2D 对代价地图高代价区域的惩罚强度。 | 边缘已有代价时，提高它通常更愿意绕开边缘；若边缘没有代价，调高也无效。建议保持地图不变，单独对比不同取值。 |
| `minimum_turning_radius: 0.0` | 写在配置中，但**当前 SmacPlanner2D 不读取此参数**。 | 调它不会改变当前二维路径；Hybrid-A* 才使用转弯半径参数。 |
| `use_final_approach_orientation: false` | 路径终点沿用请求的目标朝向；设为 `true` 时，改用路径最后一段的接近方向。 | 需要到点后保持指定朝向时保留 `false`；不要把它当作防贴边参数。 |
| `smoother.max_iterations: 1000`、`smoother.tolerance: 1.0e-10` | 平滑器最大迭代次数及收敛阈值。 | 规划耗时明显花在平滑上时再调整；阈值越小通常越难提前收敛。 |
| `smoother.w_smooth: 0.3`、`smoother.w_data: 0.2` | 平滑程度与保持原始搜索路径的权重。 | 增大 `w_smooth` 倾向更圆滑，增大 `w_data` 倾向少偏离原路径；改后必须检查路径是否接近盖板边缘。 |

### Nav2 控制器：`controller_server.ros__parameters`

| 参数及当前值 | 含义 | 如何调整 |
| --- | --- | --- |
| `controller_frequency: 10.0` | Nav2 控制器每秒运行约 10 次。 | 当前 `FollowPath` 是 `DummyController`，它输出零速度；机器狗实际跟踪频率不由此值决定。 |
| `costmap_update_timeout: 0.30` | 等待控制器代价地图更新的最长时间，单位秒。 | 只有出现等待代价地图超时日志时再结合更新频率排查。 |
| `min_x_velocity_threshold`、`min_y_velocity_threshold`、`min_theta_velocity_threshold: 0.001` | 将里程计中低于阈值的微小速度视为零；前两项单位 m/s，角速度单位 rad/s。 | 按里程计噪声调整；不是运动速度上下限。 |
| `failure_tolerance: 0.3` | 控制器短暂计算失败的容忍时间，单位秒。 | 出现控制器计算异常时再排查；不能用它解决机器人贴边。 |
| `progress_checker_plugin: [progress_checker]`、`progress_checker.plugin: nav2_controller::SimpleProgressChecker` | 启用进展检测器。 | 一般保留；若误报卡住，结合下面两个阈值和实际速度排查。 |
| `required_movement_radius: 0.25`、`time_allowance: 20.0` | 在 20 秒内至少移动约 0.25 m，否则可能判定没有进展。 | 窄道低速行走时若误报进展失败，应核对里程计和实际位移，再调整。 |
| `goal_checker_plugin: [goal_checker]`、`goal_checker.plugin: nav2_controller::SimpleGoalChecker` | 启用位置与朝向到达判定。 | 保持与多地图节点的目标容差策略一致。 |
| `xy_goal_tolerance: 0.2`、`yaw_goal_tolerance: 0.2` | 控制器初始到点容差，分别为米和弧度。 | 本包在**每个导航子段发送前**会用下文 `normal_*` 或 `transition_*` 参数动态覆盖；调整日常到点标准应看下文，而不只改这两项。 |
| `goal_checker.stateful: true` | 到达位置容差后，继续进行朝向判定时记住已到达位置的状态。 | 通常保持；若到点行为异常，结合实际机器人轨迹分析。 |
| `controller_plugins: [FollowPath]`、`FollowPath.plugin: multi_map_nav/DummyController` | Nav2 使用占位控制器验证进展/到达，但它始终输出零速度。 | 真正控制机器狗的是外部 `localPlanner` 和 `pathFollower`；贴边发生在 `/local_path` 或实走轨迹时，应调整那条控制链。 |

### 行为树导航器：`bt_navigator.ros__parameters`

| 参数及当前值 | 含义 | 如何调整 |
| --- | --- | --- |
| `global_frame: map`、`robot_base_frame: base_link`、`odom_topic: /odom` | 导航器使用的全局坐标、机器人基座及里程计话题。 | 必须与 TF 和实际话题对应；仅改名字不会修正坐标误差。 |
| `bt_loop_duration: 10` | 行为树每轮执行间隔，单位**毫秒**。 | 一般保持；过小会增加调度开销。 |
| `default_server_timeout: 20`、`wait_for_service_timeout: 1000` | 行为树节点等待下游响应及等待服务出现的时间，单位**毫秒**。 | 出现对应超时再按日志调整；不是全局路径搜索的 5 秒预算。 |
| `action_server_result_timeout: 900.0` | action 完成后保留结果的时间，单位秒。 | 通常保持；它不是导航任务允许运行的最长时间。 |
| `navigators`、`navigate_to_pose.plugin`、`navigate_through_poses.plugin` | 启用单目标和多目标导航 action 及其插件。 | 当前场景一般不用调；只有更换行为树导航器实现时才改。 |
| `error_code_names: [compute_path_error_code, follow_path_error_code]` | 从行为树收集规划/跟踪失败码，以便反馈错误原因。 | 保持；排错时优先看 action 的 `error_msg` 和节点日志。 |

### 多地图调度：`multi_map_nav_node.ros__parameters`

| 参数及当前值 | 含义 | 如何调整 |
| --- | --- | --- |
| `default_align_final_yaw: true` | 普通最终目标默认要求朝向对齐。 | 仅在业务允许不对齐时关闭；传送点有单独的忽略最终朝向策略。 |
| `default_obstacle_policy: avoid` | 传给外部局部规划器的默认策略；也可选 `stop`。 | 依据任务是否允许绕动态障碍选择；它不改变 Smac2D 静态全局路径的代价。 |
| `manual_switch_odometry_timeout_sec: 1.0` | 人工切图前要求最近一次根坐标系里程计在 1 秒内。 | 只有人工切图被误判里程计过期时，结合实际发布频率排查；不影响单图路径形状。 |
| `normal_xy_tolerance: 0.2`、`normal_yaw_tolerance: 0.2` | 普通终点的到达容差，单位分别为米和弧度。 | 盖板上应按允许停车范围和实际跟踪误差设置；过大可能提前判定到达，过小可能反复调整或任务超时。 |
| `transition_xy_tolerance: 0.5`、`transition_yaw_tolerance: 6.283185307179586` | 跨地图传送点的位置容差 `0.5 m`，朝向容差约 `2π rad`，即基本不要求朝向。 | 只影响跨图切换点；传送点靠近盖板边缘时尤其要核对其安全位置及位置容差。 |

### 盖板场景的调参顺序

1. 固定地图、起终点和规划器，确认盖板边缘在 `/map` 与实际全局代价地图中是障碍、未知还是自由栅格。先用 `--allow-unknown false` 对比，确认路径没有穿过未知区。
2. 根据机身和足端运动范围、定位误差、跟踪误差确定安全外廓与禁行边界，再核对 `footprint`、`footprint_padding`。若收缩后的通道不足以安全通过，应让规划失败，而不是强迫规划器贴边找路。
3. 保持起终点不变，分别改变 `inflation_radius`、`cost_scaling_factor` 和 `cost_travel_multiplier`，观察路径到盖板边缘的最小距离、是否无解、长度和耗时。每次只改一类参数；不能仅凭路线更短判断更安全。
4. 若全局 `/plan` 居中而局部 `/local_path` 或实走轨迹贴边，应排查外部 `localPlanner`、`pathFollower` 与定位误差；继续调 Smac 参数不能保证落脚安全。

## 输出结果

输出目录包含：

- `overlay.png`：所有成功规划的路径叠加图，绿色圆点为起点，红色圆点为终点。
- `<规划器名称>.png`：每个规划器的单独路径图。
- `results.json`：路径坐标、路径长度、Nav2 规划耗时、调用耗时与错误信息。
- `map_server.log`、`planner_server.log`、`lifecycle_manager_navigation.log`：临时 Nav2 进程日志。

`planning_time_s` 是 Nav2 返回的规划耗时，不包含节点启动时间。脚本只比较全局路径；局部规划、实际跟踪误差和机器狗落脚位置不在测试范围内。

## 添加其他插件

无需修改脚本即可加入新插件。例如，将以下内容保存为一个 YAML 文件：

```yaml
voronoi:
  plugin: nav2_voronoi_planner/VoronoiPlanner
  allow_unknown: false
  publish_voronoi_grid: false
  recompute_on_costmap_update: true
  precompute_voronoi: true
  debug: false
  costmap_topic: /global_costmap/costmap_raw
```

然后在原命令后添加 `--plugin-config /path/to/plugins.yaml --planners smac2d,voronoi`。自定义插件必须已安装在运行脚本的 ROS 环境中；若初始化失败，查看输出目录中的 `planner_server.log`。
