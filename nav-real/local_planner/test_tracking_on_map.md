### GIF 回放与动态障碍

`test_tracking_on_map.py` 现在默认输出每个场景的 GIF，两种模式并排显示。
蓝色为 `omni`，橙色为 `turn_preferred`，实心矩形为机器人，白线指出车头方向；
黑色虚线为输入的全局/参考路径，绿色实线为实际收到的局部路径，红色矩形为移动障碍，
红色箭头表示障碍运动方向。左上角显示时间、车体速度和控制状态，车体边框变红表示接触。
GIF 默认 8 帧/秒、2 倍速，最后一帧停留 1.5 秒再循环；某模式先结束时，其面板保持最后状态。
每张图有 1 m 比例尺，画面按场景范围裁剪。显示文字使用英文，便于在无中文字体的环境运行。

新增场景在 VW7En2y 的空闲区域（像素中心 `[1174,671]`）使用一条 7 m 的直线全局路径。
机器人从该中心的世界坐标 `[-2,0] m` 偏移处向 `[5,0] m` 偏移处行驶，初始/终点航向为 0。
这条路线是仿真输入的直线，不是 Smac2D 规划结果；避障路线由真实 `localPlanner` 生成。
所有偏移和速度均在世界坐标系中定义，和机器人的转向无关。

## 用 YAML 修改机器人和障碍

场景输入已经从 Python 提取到
[`scripts/tracking_scenarios.yaml`](/home/jazzy/nav_t_ws/src/local_planner/scripts/tracking_scenarios.yaml)。
脚本默认读取它，也可以用 `--scenario-config` 指定另一份 YAML。修改 YAML 后不需要重新编译。
每次运行还会把实际使用的文件复制到输出目录的 `scenario-config.yaml`。

机器人速度限制在 YAML 顶层 `robot` 中设置，数值会覆盖 `--config` 对 `flat/offroad` 的同名限制：

```yaml
robot:
  max_speed: 0.8       # 前向速度上限，m/s
  max_speed_y: null    # null 表示沿用 --config；也可填 0.4
  max_yaw_rate: null   # rad/s
  max_accel_x: null    # m/s²；其余 max_accel_*、max_decel_* 同理
```

每个场景的 `robot` 设置初始位姿：

```yaml
robot:
  reference: route_start  # route_start、centre 或 absolute
  offset: [0.0, 0.3]      # 相对 reference 的世界坐标偏移，m
  yaw: 0.0                # 初始朝向，rad
```

路线支持 `pixel_line`（端点是地图像素）和 `centre_line`（端点相对 `map.centre_pixel`，单位 m）。
静态障碍写在 `static_obstacles`，每一项是线段采样：

```yaml
static_obstacles:
  - {reference: centre, start: [-0.8, -0.37], end: [0.8, -0.37], samples: 33}
```

也可以用 `shape: rectangle` 定义填充矩形障碍，适合模拟门板、行人或箱体：

```yaml
static_obstacles:
  - {shape: rectangle, reference: centre, centre: [0.0, -3.31], size: [0.8, 6.0], spacing: 0.05}
```

其中 `size` 是矩形长宽，`spacing` 是点云采样间距；矩形的 `centre` 按 `reference` 解释，
`yaw`（可选）只旋转矩形外形。

动态障碍写在 `dynamic_obstacles`：

```yaml
dynamic_obstacles:
  - name: oncoming
    origin_reference: centre
    origin: [5.0, 0.3]
    velocity: [-0.45, 0.0]
    size: [0.8, 1.0]
    duration: 16.0
    delay: 0.0
    yaw: 3.141592653589793
```

`origin_reference` 可用 `centre`、`route_start` 或 `absolute`；`velocity` 是世界坐标速度，
`size` 是长宽（m），`yaw` 只旋转障碍外形，不会自动改变速度方向。脚本仍支持命令行的
`--obstacle-speed-scale`、`--obstacle-size-scale` 和 `--dynamic-start-delay`，它们作用于 YAML
中的动态障碍。

例如复制一份配置并只调整迎面障碍与机器人初始位置：

```bash
cp scripts/tracking_scenarios.yaml tmp/VW7En2y/tracking-test/my_tracking.yaml
# 编辑 tmp/VW7En2y/tracking-test/my_tracking.yaml 后运行：
/usr/bin/python3 scripts/test_tracking_on_map.py \
  --scenario-config tmp/VW7En2y/tracking-test/my_tracking.yaml \
  --scenarios dynamic_head_on --modes omni,turn_preferred \
  --output tmp/VW7En2y/tracking-test/head-on-custom ...
```

`...` 代表原命令中的地图、安装包、配置和二进制参数。脚本仍兼容原有
`--scenario-file` JSON：它适合批量替换动态障碍；一般单场景调试优先使用 YAML。

矩形的 `length` 沿自身 x 轴，`width` 沿自身 y 轴。例如：

```python
# 1.0 m 长、0.5 m 宽，中心在 centre 左侧 3 m，静止不动
"static_cart": ("cart", [-3.0, 0.0], [0.0, 0.0], [1.0, 0.5], 1.0, 0.0),

# 从路线下方 2 m 处向上穿行，障碍长边朝世界 y 正方向
"crossing_custom": (
    "crossing_custom", [1.0, -2.0], [0.0, 0.4], [0.7, 0.5], 10.0, 1.57079632679
),

# 迎面运动：障碍朝向机器人（朝向只影响矩形外形，不会自动改变速度方向）
"head_on_custom": (
    "head_on_custom", [4.0, 0.0], [-0.35, 0.0], [0.8, 0.6], 12.0, 3.14159265359
),
```

这里有几个容易混淆的规则：

- `yaw=0` 时，矩形长边沿世界 x 轴；`yaw=pi/2` 时，长边沿世界 y 轴。
- `yaw` 只旋转障碍几何形状，不会根据朝向自动生成速度。运动方向完全由 `[vx, vy]`
  决定；需要障碍朝向和运动方向一致时，要同时设置这两个字段。
- `duration` 决定运动多久。障碍最终中心为
  `origin + velocity * duration`；开始前停在 `origin`，结束后停在终点，不会消失。
- `delay` 不是元组字段，而是创建 `MovingObstacle` 时的启动等待时间。当前所有内置障碍
  由命令行 `--dynamic-start-delay` 统一设置；等待期间障碍保持在初始位置。
- 障碍是填充矩形，采样点间距不超过 5 cm，既用于 `/terrain_map`，也用于 GIF 和精确矩形
  接触检测。因此仅改变 GIF 外观不会改变仿真的障碍位置。

### 用像素位置设置初始点

如果更习惯用地图像素，先将像素转换为世界坐标，再减去 `centre` 得到 YAML 中的偏移：

```python
pixel_world = grid.pixel_world(np.array([[1200, 671]]))[0]
offset = pixel_world - centre
```

然后把 `offset.tolist()` 填入定义的第二项。地图 YAML 的 `origin`、分辨率和原点旋转
都会由 `pixel_world()` 处理，不要直接用 `pixel * resolution` 代替。也可以直接写绝对世界
坐标，此时将对应的 `reference` 设为 `absolute`，并把 `offset` 或 `origin` 写成绝对坐标。

### 不改源码时的统一调整

以下参数会作用于所有内置动态障碍：

```bash
--obstacle-speed-scale 1.5   # 速度乘 1.5，运动终点保持不变
--obstacle-size-scale 1.2    # 长宽都乘 1.2
--dynamic-start-delay 2      # 先静止 2 s，再开始运动
```

速度缩放同时把 `duration` 除以相同倍数，所以障碍的空间终点不变。例如原速度
`0.5 m/s`、持续 `10 s`，使用 `--obstacle-speed-scale 2` 后变为 `1.0 m/s`、
持续 `5 s`，仍然移动相同的 5 m。若要改变终点，必须直接修改定义中的初始偏移、速度或
`duration`，不能只调整速度缩放。

修改 YAML 后重新运行仿真即可；不需要重新编译 C++。每个场景目录的 `scene.json` 会保存
实际使用的 `origin`、速度、尺寸、朝向和持续时间，`obstacle-*.csv` 保存每个时刻的位置。
可以用 `--render-only` 重画 GIF，但它只会重放已经保存的运动参数，不会读取后来改动的
YAML。

| `--scenarios` 名称 | 场景 | 障碍长度 × 宽度 | 初始中心偏移（m） | 速度（m/s） | 运动持续时间 |
| --- | --- | --- | --- | --- | --- |
| `dynamic_head_on` | 迎面接近 | 0.8 × 1.0 m | `[5,0.3]` | `[-0.45,0]` | 16 s |
| `dynamic_crossing` | 横向穿行 | 0.6 × 0.6 m | `[0.5,-2.4]` | `[0,0.5]` | 9.6 s |
| `dynamic_overtaking` | 追越同向慢速障碍 | 0.8 × 0.6 m | `[0,0]` | `[0.25,0]` | 16 s |

新增的 `static_narrow_gap` 场景模拟门洞：两个矩形静态障碍物位于同一堵横向障碍墙的
左右两侧，中间只留 `0.62 m` 门缝，机器人宽度为 `0.47 m`；远处侧墙封住主障碍外侧。
机器人从门外沿 X 方向驶入门内。该场景直接给 `pathFollower` 发布参考路径，同时发布静态 `/terrain_map`，
用于验证两种跟踪模式在窄门中的车体碰撞检查；它不会让 `localPlanner` 重新选择绕门路线。
运行命令：

```bash
/usr/bin/python3 scripts/test_tracking_on_map.py \
  --map tmp/VW7En2y/map_000.yaml \
  --package /home/jazzy/nav_t_ws/install/local_planner/share/local_planner \
  --config config/d1m.yaml \
  --binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/pathFollower \
  --planner-binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/localPlanner \
  --scenario-config scripts/tracking_scenarios.yaml \
  --scenarios static_narrow_gap --modes omni,turn_preferred \
  --output tmp/VW7En2y/tracking-test/static-doorway-final --timeout 20
```

在该地图和当前安装二进制上，两种模式均无碰撞到达：`omni` 约 `4.67 s`，
`turn_preferred` 约 `4.64 s`；两者最大路线偏差约 `0.025 m`，累计横移均为 `0`。

障碍为填充的矩形，点间距不超过 5 cm，体积相当于行人占用区或小推车，
不会因为只有一两个点而被体素过滤掉。运动开始前、结束后均停留在端点，不会突然消失。
同一时刻的障碍位置用于点云、车体接触检测和回放，动态障碍采用矩形精确相交检测，
并记录机器人轮廓到障碍轮廓的最小距离。点云约 10 Hz、里程计约 50 Hz。

运行动态障碍仿真并生成 GIF：

```bash
cd /home/jazzy/nav_t_ws/src/local_planner

source /opt/ros/jazzy/setup.bash
source /home/jazzy/task_ws/install/local_setup.bash
source /home/jazzy/nav_t_ws/install/local_setup.bash

/usr/bin/python3 scripts/test_tracking_on_map.py \
  --map tmp/VW7En2y/map_000.yaml \
  --package /home/jazzy/nav_t_ws/install/local_planner/share/local_planner \
  --config /home/jazzy/nav_t_ws/install/local_planner/share/local_planner/config/d1m.yaml \
  --binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/pathFollower \
  --planner-binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/localPlanner \
  --scenarios dynamic_head_on,dynamic_crossing,dynamic_overtaking \
  --modes omni,turn_preferred \
  --output tmp/VW7En2y/tracking-test/workspace-dynamic \
  --timeout 45

# head_on
/usr/bin/python3 scripts/test_tracking_on_map.py \
  --map tmp/VW7En2y/map_000.yaml \
  --package /home/jazzy/nav_t_ws/install/local_planner/share/local_planner \
  --config /home/jazzy/nav_t_ws/install/local_planner/share/local_planner/config/d1m.yaml \
  --binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/pathFollower \
  --planner-binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/localPlanner \
  --scenarios dynamic_head_on \
  --modes omni,turn_preferred \
  --output tmp/VW7En2y/tracking-test/head_on1 \
  --timeout 45

# crossing
/usr/bin/python3 scripts/test_tracking_on_map.py \
  --map tmp/VW7En2y/map_000.yaml \
  --package /home/jazzy/nav_t_ws/install/local_planner/share/local_planner \
  --config /home/jazzy/nav_t_ws/install/local_planner/share/local_planner/config/d1m.yaml \
  --binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/pathFollower \
  --planner-binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/localPlanner \
  --scenarios dynamic_crossing \
  --modes omni,turn_preferred \
  --output tmp/VW7En2y/tracking-test/crossing1 \
  --timeout 45

# overtaking
/usr/bin/python3 scripts/test_tracking_on_map.py \
  --map tmp/VW7En2y/map_000.yaml \
  --package /home/jazzy/nav_t_ws/install/local_planner/share/local_planner \
  --config /home/jazzy/nav_t_ws/install/local_planner/share/local_planner/config/d1m.yaml \
  --binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/pathFollower \
  --planner-binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/localPlanner \
  --scenarios dynamic_overtaking \
  --modes omni,turn_preferred \
  --output tmp/VW7En2y/tracking-test/overtaking1 \
  --timeout 45

# overtaking
/usr/bin/python3 scripts/test_tracking_on_map.py \
  --map tmp/VW7En2y/map_000.yaml \
  --package /home/jazzy/nav_t_ws/install/local_planner/share/local_planner \
  --config /home/jazzy/nav_t_ws/install/local_planner/share/local_planner/config/d1m.yaml \
  --binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/pathFollower \
  --planner-binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/localPlanner \
  --scenarios static_narrow_gap \
  --modes omni,turn_preferred \
  --output tmp/VW7En2y/tracking-test/narrow1 \
  --timeout 45
```

这里：

- `--binary`：使用已编译的路径跟踪器。
- `--planner-binary`：使用已编译的局部规划器。
- `--package`：使用安装目录中的候选路径库 `paths/`。
- `--config`：使用安装目录中的 D1M 参数。
- 脚本自动启动并关闭节点，无需另外运行 launch。

目前安装目录的程序链接到 `build/local_planner/`，配置链接到源码的 `config/d1m.yaml`。\*\*修改 C++ 后需要重新编译；修改 `tracking_scenarios.yaml` 后重新运行仿真即可。\*\*机器人速度由该 YAML 的 `robot` 段控制，默认前向上限为 `0.8 m/s`，并按 `--modes` 覆盖跟踪模式。

如果只测试 `turn_preferred`，改为：

```
--modes turn_preferred
```

如果要运行包含 Smac2D 转弯路线的全部场景，删除 `--scenarios ...`，并增加：

```
--corner-path tmp/VW7En2y/tracking-test/global-corner/results.json
```

输出目录包含 GIF、PNG、轨迹 CSV、节点日志和 `report.json`。出现碰撞时即使到达终点，整组测试也会返回 `FAIL`；这是仿真检测结果

| 参数 | 用途 |
| --- | --- |
| `--gif` / `--no-gif` | 开启/关闭 GIF；默认开启，PNG 和原始记录始终保留。 |
| `--gif-fps 10` | 动画帧率；不改变 ROS 控制频率。 |
| `--gif-speed 1` | 1 倍速回放；默认 2 倍速。 |
| `--gif-width 1400` | 整张对比图宽度；默认 1100 像素。 |
| `--gif-height 600` | 指定高度；默认 0，按场景自动适配。 |
| `--obstacle-speed-scale 1.5` | 所有动态障碍速度乘 1.5；运动时间相应缩短，空间端点保持一致。 |
| `--obstacle-size-scale 1.2` | 所有动态障碍长宽乘 1.2。 |
| `--dynamic-start-delay 2` | 障碍先在初始位置等待 2 秒，再开始运动。 |
| `--render-only` | 从已有输出重新生成 GIF/PNG，无需启动 ROS，也不重新仿真。 |

例如只调整回放速度和清晰度：

```bash
/usr/bin/python3 scripts/test_tracking_on_map.py \
  --output tmp/VW7En2y/tracking-test/dynamic-gif \
  --render-only --gif-speed 1 --gif-fps 10 --gif-width 1400
```

回放依据该输出目录的 `report.json`，恢复实际运行过的场景、模式及地图。
各子目录新增 `scene.json`（路径、车体尺寸和障碍运动参数）、`local_paths.json`
（收到的局部路径及时间）、`obstacle-*.csv`（障碍位置和速度）；`trajectory.csv`
新增 `blocked` 和 `collision` 两列。`--render-only` 仅支持包含这些记录的新版本输出。
车体尺寸读取 `--config` 中 `localPlanner.vehicleLength/vehicleWidth`，默认 D1M 为 1.0 × 0.47 m。
绘图在所有 ROS 运行完成后进行，避免渲染占用控制循环时间；运行时保存实际使用的参数快照。

#### 通用预测轨迹评估（2026-10-03）

原来的固定侧移、3 m 触发距离和单点释放阈值已删除。改进借鉴 Nav2 DWB 的有限候选
轨迹评分思路，仍在 `pathFollower` 内执行，`localPlanner` 的路径库、路径评分和频率没有改动。
正常前视跟踪轨迹如果安全，直接使用；否则最多检查 36 条轨迹。候选包括向两侧转向、
保留航向横移、减速、等待和后退，所有候选都遵守模型限速、加减速度和输出死区。

安全是硬条件：预测矩形车体沿整段轨迹与运动障碍是否重叠；连停车也必须验证，不能将
“速度为零”视为安全。通过安全检查后，综合前进进度、路径距离、航向、横移量和前后周期
的选择变化评分。`turn_lateral_cost` 只增加横移代价，不会把危险的转向动作改为可执行。
当前点云通过空间分组和矩形包围盒预筛选加速，精细碰撞检查保留全部原始障碍点。

动态预测仍由连续 `/terrain_map` 估计障碍速度，不读取仿真对象的速度设定。
`omni` 不使用这套逻辑；仿真不修改原始地图。匀速预测对定位/点云误差、遮挡和突然加速
仍有误差；无安全候选时只能报告 blocked，不能保证任意动态障碍都不会撞向停住的机器人。

```yaml
pathFollower:
  ros__parameters:
    turn_predictive_steering: true
    turn_prediction_horizon: 2.5  # 动态障碍预测长度（s）
    turn_collision_horizon: 0.6  # 静态场景预测长度（s）
    turn_lateral_weight: 0.25    # 普通跟踪横移分量权重
    turn_lateral_cost: 0.6       # 安全候选轨迹的横移软代价
```

旧的 `turn_predictive_lateral_weight` 和 `turn_predictive_clear_distance` 已删除，不再调节。
终点位置误差较小时，仅对仍在位置容差外的轴补偿速度死区，避免末端卡住。

仿真新增 `--scenario-file` 和 `--quality-checks`。自定义场景文件见
`test/scenarios/dynamic_tracking_sweep.json`：`base` 选择已有路线；`origin_offset` 相对路线
起点，其他障碍参数采用世界坐标。例如 `[7,0.1]` 相对起点，等价于当前迎面场景相对中心
的 `[5,0.1]`。`--quality-checks` 额外要求动态间距至少 5 cm、轨迹不超过参考路线的 1.3 倍、
累计航向不形成一整圈、沿路线的后退量不超过 0.5 m、控制计算 P99 小于 20 ms。
这些阈值用于当前直线动态回归用例，不代表任何导航任务的统一验收阈值。

```bash
source /opt/ros/jazzy/setup.bash
source /home/jazzy/task_ws/install/local_setup.bash
source /home/jazzy/nav_t_ws/install/local_setup.bash
/usr/bin/python3 scripts/test_tracking_on_map.py \
  --map tmp/VW7En2y/map_000.yaml \
  --package /home/jazzy/nav_t_ws/install/local_planner/share/local_planner \
  --config /home/jazzy/nav_t_ws/install/local_planner/share/local_planner/config/d1m.yaml \
  --binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/pathFollower \
  --planner-binary /home/jazzy/nav_t_ws/install/local_planner/lib/local_planner/localPlanner \
  --scenario-file test/scenarios/dynamic_tracking_sweep.json \
  --scenarios head_0p0,head_0p1,head_neg0p1,head_0p3,head_neg0p3,head_0p5,head_neg0p5,head_slow,head_fast,head_large,cross_reverse,two_oncoming,dynamic_crossing,dynamic_overtaking \
  --modes turn_preferred --timeout 35 --quality-checks \
  --output tmp/VW7En2y/tracking-test/rollout-regression --domain-id 195 --no-gif
```

`--no-gif` 只关闭 GIF 输出，PNG、场景配置、速度与轨迹、局部路径、节点日志仍会保留。
`result.json` 新增计算耗时 P50/P99/最大值、最大候选数量、最大航向变化、路径横向偏差、
沿参考路线的后退量，以及各项质量检查。确认报告后可 `--render-only` 生成 GIF。
仿真用理想平面速度积分，包含完整 ROS 规划与跟踪节点；不模拟实机惯性、打滑和碰撞物理。


本轮在 `/home/jazzy/nav_t_ws/build` 和 `install` 完成编译验证，未部署到 D1M-B。
14 个动态实例（7 个迎面左右偏移、不同速度/尺寸、双向横穿、追越和两个连续迎面障碍）
均通过上述质量检查。最小动态轮廓间距约 0.082 m；控制计算 P99 最大约 1.53 ms，
单次最大约 3.34 ms，低于 50 Hz 控制周期的 20 ms。约 4,941 点的加密障碍压力测试
平均约 0.68 ms、最大约 0.85 ms。耗时是当前本地机器、0.8 m/s 测试模型的测量，
包含索引和候选控制计算，不包含点云回调、通信和系统调度；D1M-B 应另行测量整条链路。

`[5.0, 0.1]` 的同场景对比：

| 指标 | omni | turn_preferred |
| --- | ---: | ---: |
| 接触采样次数 | 0 | 0 |
| 到达时间 | 18.09 s | 17.75 s |
| 轨迹长度 | 7.53 m | 7.23 m |
| 累计车体横移 | 0.89 m | 0.23 m |
| 最小动态间距 | 0.225 m | 0.083 m |
| 控制计算 P99 | 未统计 | 1.13 ms |

结果目录：`tmp/VW7En2y/tracking-test/rollout-regression/`（动态扫描报告、各场景 PNG）、
`rollout-head-on-01/`（两种模式对比 GIF/PNG）。`rollout-regression/validation-summary.json`
汇总报告、程序 SHA256、压力测试和静态回归。静态检查涵盖平行偏移、死胡同调头、横移
放行、贴墙转身和地形超时；原先保存的 11 m Smac2D 拐角路线在 `rollout-corner/` 通过。
首次拐角检查误选了约 113 m 路线，45 s 不足以走完，原始超时报告保留在 `rollout-static/`。
另有 42 个独立运动学组合（7 种偏移 × 3 种速度 × 2 种路线朝向）通过，使用矩形 SAT
作为独立碰撞判据，同时要求短路线和无航向回环；300 个位姿检查确认空间预筛选没有
改变精细点碰撞结果。

几何、时间线、地图坐标转换和 GIF 文件回归检查：

```bash
/usr/bin/python3 test/test_map_simulation.py
```
