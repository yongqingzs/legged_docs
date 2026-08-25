# rmf 是如何处理多楼层地图问题的，坐标、地图切换、跨楼层等
• RMF 对多楼层的处理核心是：

> 每个楼层是一个独立的二维 map；跨楼层不是坐标系里直接沿 Z 轴移动，而是在导航图中通过“电梯/楼梯等连接事件”从一个 map 切换到另一个 map。

因此，RMF Traffic Schedule 不是一个完整的三维碰撞系统。
```
L1: (x, y, yaw)
L2: (x, y, yaw)
L3: (x, y, yaw)
```
即使 L1 与 L2 都有坐标 (3.0, 5.0)，它们也不是同一个物理位置，因为 map 名不同：
```
("L1", 3.0, 5.0) != ("L2", 3.0, 5.0)
```
这也解释了你之前看到的 Node.cpp 判断：
```
if (route->map() != vc->route->map())
continue;
```
不同楼层的普通路线不会被 DetectConflict::between() 直接拿来做几何冲突检测。

## 1. 地图与坐标

RMF Building Map 中定义多个 level，例如 hotel demo：
```
levels:
L1:
L2:
L3:

lifts:
Lift1:
    reference_floor_name: L1
    level_doors:
    L1: [lift1_door]
    L2: [lift1_door]
    L3: [lift1_door]
```
可参见 rmf_demos_maps/maps/hotel/hotel.building.yaml:17 和 rmf_demos_maps/maps/hotel/hotel.building.yaml:1027。

RMF 的导航图 Graph 中，每个 waypoint 本身都有：
```
map_name
Eigen::Vector2d location
```
见 /home/jazzy/rmf_ws/src/rmf/rmf_traffic/rmf_traffic/include/rmf_traffic/agv/Graph.hpp:114。

所以内部语义是：
```
Waypoint 17:
map = "L1"
position = (12.1, 8.3)

Waypoint 42:
map = "L2"
position = (12.1, 8.3)
```
它们可以坐标相同，但代表不同楼层。

对于真实机器人，Fleet Adapter 还必须解决“机器人自身定位坐标系”到 RMF level 坐标系的转换：
```
robot/map 或 SLAM 坐标
    -- 标定的平移、旋转、比例变换 -->
RMF building map 的 L1 / L2 坐标
```
机器人报告位置时，不能只报 (x, y, yaw)；adapter 还必须知道它当前属于哪个 RMF map，例如 L1。跨层完成后，它将位置状态切换为 L2。

## 2. 规划时如何跨楼层

rmf_traffic::agv::Planner 使用的是一张逻辑上连通的导航图，而不是为每层单独运行一次规划器。

例如：
```
L1 办公室
-> L1 走廊
-> L1 电梯入口
-> Lift1 移动到 L2
-> L2 电梯出口
-> L2 走廊
-> L2 目标房间
```
导航图中的节点带 map 名；lane 可以连接不同 map 的 waypoint。普通跨 map lane 在模型上可存在，但工程上应通过电梯、楼梯或坡道等真实设施事件来表达，不应把两个楼层“直接连一条普通线”。

电梯 lane 会带事件，而不只是“从 L1 坐标插值飞到 L2 坐标”。RMF 定义了：
```
LiftSessionBegin  获取并锁定电梯会话
LiftMove          电梯移动到目标楼层
LiftDoorOpen      开门
LiftSessionEnd    结束/释放电梯会话
```
定义在 /home/jazzy/rmf_ws/src/rmf/rmf_traffic/rmf_traffic/include/rmf_traffic/agv/Graph.hpp:312。

规划器会把这些事件和预计耗时纳入路径成本。例如典型跨层 plan 是：
```
t=00  L1：驶向电梯
t=20  L1：申请 Lift1，进入轿厢
t=28  Lift1：从 L1 移动至 L2
t=38  L2：开门、驶出轿厢
t=45  L2：继续驶向目标
```
RMF 的 Planner 单元测试也明确构造了：
```
LiftSessionBegin(lift_name, L1, ...)
LiftMove(lift_name, L2, ...)
LiftSessionEnd(lift_name, L2, ...)
```
见 /home/jazzy/rmf_ws/src/rmf/rmf_traffic/rmf_traffic/test/unit/agv/test_Planner.cpp:3085。

## 3. itinerary 会被拆成多个 Route

一项跨层计划不会是一条标记为 L1 的长 trajectory，而是按地图拆开：
```
Itinerary
├─ Route(map="L1")
│   └─ 从起点到 Lift1 门口/轿厢的 2D trajectory
├─ 电梯相关事件
│   └─ 请求、占用、移动、开门、释放
└─ Route(map="L2")
    └─ 从 Lift1 门口到目标点的 2D trajectory
```
每个 Route 都有一个 map 字符串和一条 trajectory，见 /home/jazzy/rmf_ws/src/rmf/rmf_traffic/rmf_traffic/src/rmf_traffic/Route.cpp:114。

因此：
- L1 走廊中的冲突，在 map="L1" 的 Route 间检测；
- L2 走廊中的冲突，在 map="L2" 的 Route 间检测；
- 电梯资源竞争，不应只依赖 L1/L2 的几何冲突检测。

## 4. 电梯如何防止两台机器人同时使用

电梯是一个共享、容量有限、会改变楼层位置的资源，不能仅靠 DetectConflict::between() 处理。

Fleet Adapter 执行到 LiftSessionBegin 时，会创建 RequestLift phase，请求电梯并保存当前计划状态，见 /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_fleet_adapter/src/rmf_fleet_adapter/events/ExecutePlan.cpp:243。

rmf_lift_supervisor 按 lift 名维护 active session；一个电梯已有不同 session 时，不会直接接管该会话。相关实现见 /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_fleet_adapter/src/lift_supervisor/Node.cpp:45。

所以要分开理解：

Traffic Schedule：
协调同一楼层 map 上机器人预计的时空占用。

Lift Supervisor / Lift Manager：
协调 Lift1 这个设备的会话、目标楼层、门状态与独占控制。

Fleet Adapter：
把计划中的电梯事件变成对真实电梯或仿真电梯的请求，并在获得正确的电梯状态后命令机器人进出轿厢。

实际集成时，Lift Manager 必须可靠实现 session ID、门状态、当前楼层和权限控制。默认 rmf_lift_supervisor 是会话协调组件，源码中也明确留有“更智能地调度电梯”的 TODO；复杂电梯群的调度策略通常应在真实 Lift Manager 或上层系统实现。

## 5. 机器人如何“切换地图”

机器人不应在轿厢移动期间假装在 L1 或 L2 走廊中移动。通常状态机是：
```
当前 map = L1
-> 到达 L1 电梯门前
-> 获得 Lift1 session
-> 进入轿厢
-> 电梯到达 L2 且门已打开
-> 更新当前 map = L2
-> 从 L2 电梯门口继续导航
```
切换时机应基于电梯反馈，而不是只依据计划中的预计时间。因为电梯可能延迟、被人工占用、门未打开或实际停在非预期楼层。

若实际进度偏离原 itinerary，adapter 应更新 itinerary、延迟或触发重新规划/协商。否则 Schedule 仍会以为机器人已按时抵达 L2，而真实机器人可能仍困在 L1 电梯门前。

一句话总结：
```
RMF 用 map 名隔离楼层二维坐标；
用一个跨 map 的导航图寻找可达路径；
用电梯/门等 lane event 表达跨层动作；
用设备会话控制电梯资源；
用 itinerary 的分段 Route 在各楼层进行 Traffic Schedule 协调。
```