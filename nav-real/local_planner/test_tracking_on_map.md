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

## 动态障碍参数怎么改

动态障碍当前集中定义在源码
[`test_tracking_on_map.py`](/home/jazzy/nav_t_ws/src/local_planner/scripts/test_tracking_on_map.py:149)
的 `definitions` 字典中。每一项的格式是：

```python
"场景名称": (
    "障碍名称",          # 用于图例和输出文件名
    [x0, y0],             # 初始中心相对 centre 的世界坐标偏移，单位 m
    [vx, vy],             # 世界坐标速度，单位 m/s
    [length, width],      # 障碍尺寸，单位 m；length 沿障碍自身 x 轴
    duration,             # 移动持续时间，单位 s
    yaw,                  # 障碍朝向，单位 rad，逆时针为正
)
```

当前脚本先取地图像素 `[1174, 671]` 的世界坐标作为 `centre`，再计算
`origin = centre + [x0, y0]`。因此 `[5, 0]` 表示从路线中心向世界 x 正方向
5 m 处开始，而不是图片像素坐标，也不是相对机器人坐标。障碍速度同样是世界坐标
速度；机器人转向后，脚本会在发布点云时把它转换到机器人 `horizon` 坐标系。

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

如果更习惯用地图像素，先将像素转换为世界坐标，再减去 `centre` 得到偏移：

```python
pixel_world = grid.pixel_world(np.array([[1200, 671]]))[0]
offset = pixel_world - centre
```

然后把 `offset.tolist()` 填入定义的第二项。地图 YAML 的 `origin`、分辨率和原点旋转
都会由 `pixel_world()` 处理，不要直接用 `pixel * resolution` 代替。也可以直接写绝对世界
坐标，但需要把脚本中的
`(centre + offset).tolist()` 改为目标坐标列表；保留当前写法更容易在不同地图中心附近移动场景。

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

修改定义后重新运行仿真即可；不需要重新编译 C++。每个场景目录的 `scene.json` 会保存
实际使用的 `origin`、速度、尺寸、朝向和持续时间，`obstacle-*.csv` 保存每个时刻的位置。
可以用 `--render-only` 重画 GIF，但它只会重放已经保存的运动参数，不会读取后来改动的
源码定义。

| `--scenarios` 名称 | 场景 | 障碍长度 × 宽度 | 初始中心偏移（m） | 速度（m/s） | 运动持续时间 |
| --- | --- | --- | --- | --- | --- |
| `dynamic_head_on` | 迎面接近 | 0.8 × 0.6 m | `[5,0]` | `[-0.45,0]` | 16 s |
| `dynamic_crossing` | 横向穿行 | 0.6 × 0.6 m | `[1,-2.4]` | `[0,0.5]` | 9.6 s |
| `dynamic_overtaking` | 追越同向慢速障碍 | 0.8 × 0.6 m | `[0,0]` | `[0.25,0]` | 16 s |

障碍为填充的矩形，点间距不超过 5 cm，体积相当于行人占用区或小推车，
不会因为只有一两个点而被体素过滤掉。运动开始前、结束后均停留在端点，不会突然消失。
同一时刻的障碍位置用于点云、车体接触检测和回放，动态障碍采用矩形精确相交检测，
并记录机器人轮廓到障碍轮廓的最小距离。点云约 10 Hz、里程计约 50 Hz。

运行动态障碍仿真并生成 GIF：

```
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
```

这里：

- `--binary`：使用已编译的路径跟踪器。
- `--planner-binary`：使用已编译的局部规划器。
- `--package`：使用安装目录中的候选路径库 `paths/`。
- `--config`：使用安装目录中的 D1M 参数。
- 脚本自动启动并关闭节点，无需另外运行 launch。

目前安装目录的程序链接到 `build/local_planner/`，配置链接到源码的 `config/d1m.yaml`。\*\*修改 C++ 后需要重新编译；修改该 YAML 后重新运行仿真即可。\*\*仿真会将前向最大速度覆盖为 `0.8 m/s`，并按 `--modes` 覆盖跟踪模式。

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

**2026-10-02 的动态测试结果：**三个场景 × 两种模式均到达目标，但只有追越场景
两种模式均无接触。迎面和横穿场景均发生接触，所以整组报告为 `FAIL`、进程返回码为 1。
本轮 `turn_preferred` 追越的最小轮廓间距约 0.164 m，`omni` 仅约 0.004 m；
无接触不等于安全裕量足够。迎面场景的 `turn_preferred` 曾停车，障碍继续移动后撞向机器人。
这些是当前参数和一次运行的结果，受实时调度影响，不代表实机的稳定安全能力。

当前控制器检查的是当前障碍快照，没有预测其运动速度；动画里的箭头来自仿真设定，
并未作为速度信息提供给控制器。脚本不修改控制算法、D1M 配置或原始栅格地图。
接触后仍按指令积分，以暴露后续行为，因此接触之后的轨迹不模拟真实碰撞物理。
`success` 表示场景检查通过（普通场景检查到达，地形中断场景检查停车）；
`arrived` 表示到达判据满足，`safe_arrival` 还要求无接触；总报告也要求所有场景无接触。
`collision_steps` 是检测到接触的采样次数，不是独立事故次数；`collision_seconds` 是累计接触时长。

几何、时间线、地图坐标转换和 GIF 文件回归检查：

```bash
/usr/bin/python3 test/test_map_simulation.py
```
