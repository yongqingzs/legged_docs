## task1
运行
```
ros2 launch rmf_demos_gz office.launch.xml
```
出现错误:
```
[gazebo-18] [Err] [Server.cc:86] Error Code 14: [/sdf/world[@name="sim_world"]/include[53]/uri:/home/jazzy/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/office.world:L644]: Msg: Unable to find uri[model://TeleportDispenser]
[gazebo-18] [Err] [Server.cc:86] Error Code 14: [/sdf/world[@name="sim_world"]/include[54]/uri:/home/jazzy/rmf_ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/office.world:L650]: Msg: Unable to find uri[model://TeleportDispenser]
```
请排查解决

解决方案:
```
PATH=/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin MAKEFLAGS="-j4" colcon build --mixin release lld   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

### 1. 这个 demo 实现了什么功能，目的是什么:
```
ros2 launch rmf_demos_gz office.launch.xml
```

它模拟一个办公室，包含两台 tinyRobot、可控门、充电点、导航车道，以及饮料出货机 coke_dispenser 和收货点 coke_ingestor。RMF 负责把任务分配给机器人、规划无冲突路径、协调门的开关、管理充电与停车，并在 RViz/Gazebo 中显示运行状态。

启动后包含的主要模块：
- Gazebo：加载 office 物理场景和两台机器人。
- RMF Traffic Schedule / Blockade：维护交通时刻表并协调潜在路径冲突。
- Fleet Manager 与 Fleet Adapter：把 RMF 的任务和路径计划转换为对 tinyRobot 车队的控制。
- Task Dispatcher：接收配送、巡逻等任务并向车队询价、分配任务。
- Door Supervisor、Queue Manager、Mutex Supervisor：协调门、共享区域和队列。
- RViz 可视化：展示地图、机器人、路径、交通状态和设施状态。

其目的在于验证或演示真实设施接入 RMF 时的完整协同链路：多个机器人在共享空间内执行任务，同时遵守交通规则、使用基础设施并处理电量约束。

该启动命令本身主要是把系统运行起来；可以另开终端提交任务。例如配送一罐可乐：
```
source /home/jazzy/rmf_ws/install/setup.bash

ros2 run rmf_demos_tasks dispatch_delivery \
-p pantry -ph coke_dispenser \
-d hardware_2 -dh coke_ingestor \
--use_sim_time
```
预期流程是：RMF 分配机器人到 pantry，请求出货机提供 Coke，机器人取货后经规划路径穿过办公室，必要时请求开门，最终到 hardware_2 的收货机完成交付。

也可提交巡逻任务：
```
ros2 run rmf_demos_tasks dispatch_patrol \
-p coe lounge -n 3 --use_sim_time
```
相关配置入口是 rmf_demos/launch/office.launch.xml:7、rmf_demos/config/office/tinyRobot_config.yaml:1 和 README.md:119。


### 2. "RMF Traffic Schedule / Blockade：维护交通时刻表并协调潜在路径冲突"具体是如何做的？这个功能在哪里实现？

• 这里需要把两个组件分开看：Schedule 是主交通协调机制；Blockade 是另一套基于路径检查点的“分段通行许可”机制。它们都被 office 的通用启动文件启动，但默认的 office.launch.xml 主要依赖前者。

Traffic Schedule 如何协调冲突

每台由 RMF 控制的机器人都是一个 schedule participant。它提交的不是“我要走 A 到 B”，而是带时间的 itinerary：

- 每段路线属于哪个地图；
- 机器人的预计位置轨迹 trajectory；
- 预计通过每个位置的时间；
- 机器人的几何外形/安全半径 profile；
- 当前计划 ID、已到达的路线索引、累计延迟等状态。

整体流程：

任务 / 目标点
-> Fleet Adapter 根据导航图 + 当前 Schedule 快照规划带时间路径
-> 提交 itinerary 到 Traffic Schedule
-> Schedule 保存所有 participant 的 itinerary
-> 按“空间 + 时间 + 机器人外形”检测轨迹冲突
-> 若冲突，发起 negotiation
-> 涉及冲突的 Fleet Adapter 提出等待或绕行等替代 itinerary
-> 达成无冲突方案后，各 participant 更新自己的 itinerary
-> Adapter 将最终路线下发给机器人

实际在路径规划阶段，fleet adapter 已使用 schedule 的快照避开现有计划，而不只是“碰撞后才处理”。例如 /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_fleet_adapter/src/rmf_fleet_adapter/events/
GoToPlace.cpp:607 将 schedule()->snapshot()、自身 participant ID 和 robot profile 交给 FindPath，得到考虑其他机器人预期轨迹的路径；随后将完整 itinerary 写回 schedule。

机器人因为堵塞、门未及时打开等原因偏离预测时间时，adapter 会发布 itinerary_delay 或更新 itinerary。Schedule 因此得知“该机器人晚了”，能重新判断对其他机器人的影响。

冲突检测在哪里实现

启动入口在 rmf_demos/launch/common.launch.xml:17，启动的可执行程序是：
```
<node pkg="rmf_traffic_ros2" exec="rmf_traffic_schedule" .../>
```
其核心节点源码是 /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/rmf_traffic_ros2/schedule/Node.cpp:78。

其中：

- /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/rmf_traffic_ros2/schedule/Node.cpp:352 订阅 itinerary_set、itinerary_delay、itinerary_reached 等 ROS 2 topic，更新全局交通数据库。
- /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/rmf_traffic_ros2/schedule/Node.cpp:145 调用 rmf_traffic::DetectConflict::between(...)，用两条时空轨迹及两个 profile 检测是否会在同一
时段占据相交/过近的空间。

- /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/rmf_traffic_ros2/schedule/Node.cpp:558 收集冲突 participant；/home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/
rmf_traffic_ros2/schedule/Node.cpp:619 发布 negotiation_notice，通知相关 fleet adapter 参与协商。

协商通信由 /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/rmf_traffic_ros2/schedule/Negotiation.cpp:322 实现，使用 negotiation_notice、proposal、rejection、conclusion 等 topic。算法
层的协商器位于 /home/jazzy/rmf_ws/src/rmf/rmf_traffic/rmf_traffic/src/rmf/rmf_traffic/agv/SimpleNegotiator.cpp:1。结果通常表现为一方等待、调整出发时刻，或采用另一条可行路线，而不是 Schedule 节
点直接操纵机器人。

重要边界是：RMF 的 schedule 协调的是“已接入 RMF 且会汇报计划/状态”的交通参与者。对人、临时障碍物等，仍由机器人本体的传感器与局部避障负责；局部避障造成的延迟需要 adapter 回报，RMF 才能将影响传
播到全局计划。

Blockade 实际做什么

rmf_traffic_blockade 的启动位置是 rmf_demos/launch/common.launch.xml:22。它不是上面那种通用 itinerary 冲突检测器，而是让机器人为一串空间检查点申请“当前允许走到哪一段”的通行范围：

- participant 提交带半径的 checkpoint 路径；
- moderator 根据彼此重叠的路径分配每台机器人当前可通行的 checkpoint 区间；
- 机器人报告 ready、reached、release；
- moderator 经 blockade_heartbeat 下发新的许可区间，并能报告 gridlock。

核心 API 在 /home/jazzy/rmf_ws/src/rmf/rmf_traffic/rmf_traffic/include/rmf_traffic/blockade/Moderator.hpp:109，ROS 2 中转节点在 /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/
rmf_traffic_ros2/blockade/Node.cpp:107。

在这个默认 office demo 中，两台 tinyRobot 使用 full-control fleet adapter，主要走 Schedule + negotiation。Blockade 节点虽然随通用启动文件出现，但典型使用者是 traffic-light 类型 adapter，例如 /
home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_fleet_adapter/src/rmf_fleet_adapter/agv/EasyTrafficLight.cpp:344。对应的 office 变体是 office_mock_traffic_light.launch.xml，而不是当前的
office.launch.xml。


### 3. 能用具体的例子讲解一下"Traffic Schedule 如何协调冲突"吗

• 可以。假设办公室里两台机器人正好要通过同一段狭窄走廊：
```
tinyRobot1: pantry  -> hardware_2
tinyRobot2: lounge  -> coe

                    狭窄走廊
pantry ---- A ================= B ---- hardware_2
                ^
                两台车都会经过
```
两台机器人半径约 0.3 m，因此 RMF 不允许它们在走廊中同一时刻相向通过。

一开始，每台机器人会把自己的“预计时空占用”提交给 Traffic Schedule，而不只是提交几何路线：
```
tinyRobot1 itinerary
09:00:00  到 A
09:00:05  进入走廊
09:00:15  到 B
09:00:25  到 hardware_2

tinyRobot2 itinerary
09:00:02  到 B
09:00:07  进入走廊
09:00:17  到 A
09:00:27  到 coe
```
Schedule 看见两条轨迹：
```
时间        Robot1              Robot2
09:00:05    进入走廊             在 B 等待
09:00:07    走廊中               进入走廊  <- 冲突
09:00:15    到 B                 走廊中
```
它并不简单按“谁先报到谁优先”直接下命令，而是发出协商通知给两个参与者：
```
RMF Traffic Schedule
"Robot1 和 Robot2 的当前计划在走廊发生时空冲突，请协商。"
```
两个 robot fleet adapter 根据各自的任务、当前位置、电量、预计完成时间等，给出替代方案。例如：
```
Robot1 提案：按原计划通过走廊
Robot2 提案：在 B 前的 holding point 等待 10 秒，之后再进入
```
最终获批的 schedule 可能是：
```
tinyRobot1
09:00:05 - 09:00:15  使用走廊

tinyRobot2
09:00:02 - 09:00:15  在 B 前等待
09:00:15 - 09:00:25  使用走廊
```
随后两个 adapter 各自更新自己的 itinerary，并把路线下发给机器人。结果是机器人 2 在一个安全等待点停住，等机器人 1 通过后再出发。
```
原始冲突：
R1 ---> <--- R2

协商后：
R1 -------->
            R2 -------->
```
这就是“时间表”而非普通避障的关键：RMF 不要求两台车到狭窄走廊入口后才临时互让，而是提前规划谁在什么时间段占用哪个空间。

再看一个运行中的动态情况。假设原定 Robot1 在 09:00:15 离开走廊，但它因门打开慢或被人挡住，实际会晚 8 秒：
```
原计划：Robot1 09:00:15 离开走廊
实际预测：Robot1 09:00:23 才能离开
```
Robot1 的 adapter 会向 Schedule 报告累计延迟。此时 Robot2 原本 09:00:15 的通行计划又会冲突，RMF 会重新协商，更新为例如：
```
Robot2 改为 09:00:23 后再进入走廊
```
如果 Robot2 有可用绕路，adapter 也可能提交“改走另一条车道”的提案。选择等待还是绕行由各 adapter 的规划结果和协商过程决定。

在 office demo 中，发送两个配送或巡逻任务后，tinyRobot1、tinyRobot2 会提交这样的 itinerary。你可以在 RViz 的 schedule 可视化中看到它们的预计轨迹和时间演化。实际物理安全仍依赖机器人局部避障；Traffic Schedule 解决的是已接入 RMF 的机器人之间的全局时空协调。


### 4. 能用具体例子说明"Blockade 实际做什么"吗，它和"Traffic Schedule"的异同？让我有一个直观的认识

• 可以把 Blockade 想成“窄路分段放行员”，而 Traffic Schedule 是“全园区的预约式交通规划系统”。

假设有一条只能容纳一台车的单行狭窄通道，设有 4 个检查点：
```
停车点 P0 ---- C1 ---- C2 ---- C3 ---- P4
                狭窄通道
```
两台车计划相向通过：
```
R1: P0 -> C1 -> C2 -> C3 -> P4
R2: P4 -> C3 -> C2 -> C1 -> P0
```
Blockade 的工作方式是，每台机器人先提交自己的 checkpoint 路径和安全半径，然后不断汇报：

- ready(C1)：我已经在 C1，准备往前走。
- reached(C2)：我已到 C2。
- release(C1)：我已离开 C1，后方资源可释放。

Blockade Moderator 计算后不会给机器人一个完整时间表，而是下发“你当前获准走到哪里”的连续区间。例如：
```
R1 获准：C1 -> C3
R2 获准：P4 -> C3 前的等待点
```
R1 可以推进到 C3；R2 必须停在自己被许可区间的末端。R1 到达并释放 C3 后，Moderator 再扩大 R2 的许可范围：
```
R1 已离开狭窄通道
R2 新获准：C3 -> C1
```
直观上像这样：
```
时刻 1:
R1 [允许 C1~C3]  ---->
R2 [仅允许到等待点]  停

时刻 2:
R1 已通过并释放通道
R2 [允许 C3~C1]  <----
```
它特别适合“只能前进/暂停，难以精确执行复杂路径或重新规划”的车辆，例如 traffic-light fleet。RMF 可以对这类车说“现在可以继续到第 N 个检查点”，但未必能让其执
行一个复杂的全局重规划。
```
维度                Traffic Schedule                          Blockade
━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
协调对象            带时间的完整轨迹                          带空间检查点的路径
──────────────────  ────────────────────────────────────────  ───────────────────────────────────────
核心决策            谁在何时占用何处                          谁现在获准推进到哪个检查点
──────────────────  ────────────────────────────────────────  ───────────────────────────────────────
冲突处理            规划避让、等待、绕行、协商新 itinerary    限制可通行区间，分段放行
──────────────────  ────────────────────────────────────────  ───────────────────────────────────────
对机器人能力要求    能接受带时间路径、上报进度并可重规划      至少能停止、继续并报告 checkpoint
──────────────────  ────────────────────────────────────────  ───────────────────────────────────────
适用场景            full-control AMR、多机器人全局调度        窄通道、受限车辆、traffic-light 控制
──────────────────  ────────────────────────────────────────  ───────────────────────────────────────
空间模型            连续轨迹 + 时间 + 机器人外形              checkpoint 路径 + 半径 + 当前许可范围
```
两者并不冲突，可以一起使用：
```
Traffic Schedule：先决定总体上 R1 先过、R2 后过。
Blockade：运行时确保 R2 即使出现延迟或控制误差，也不会越过许可检查点进入通道。
```
对于当前的默认命令：
```
ros2 launch rmf_demos_gz office.launch.xml
```
两台 tinyRobot 属于 full-control fleet，主机制是 Traffic Schedule。rmf_traffic_blockade 节点会被通用启动文件启动，但默认场景通常没有 Blockade participant
实际向它申请路径许可。

要直观看到 Blockade 的作用，应启动该仓库的 traffic-light 变体：
```
ros2 launch rmf_demos_gz office_mock_traffic_light.launch.xml
```
其中 tinyRobot2 使用 traffic-light 控制模式，更符合“只允许推进到某个检查点”的 Blockade 使用场景。


### 5. 我运行这个demo，发现演示中只有一台机器人在规划并动作，这个符合预期吗？这不应该是多机协同吗？:
```
ros2 launch rmf_demos_gz office.launch.xml
```
这个任务:
```
ros2 run rmf_demos_tasks dispatch_patrol -p coe lounge -n 3 --use_sim_time
```


符合预期。你提交的是一个 patrol 任务，不是“让两台机器人各巡逻一部分”的并行任务：
```
ros2 run rmf_demos_tasks dispatch_patrol -p coe lounge -n 3 --use_sim_time
```
其中 -n 3 的意思是同一任务在 coe 与 lounge 间往返 3 轮，不是分给 3 台机器人，也不会把一轮拆给两台机器人。一个 RMF task 最终只会分配给一台机器人执行，所以另
一台保持空闲是正常行为。

office 配置中确实注册了两台机器人：

- tinyRobot1
- tinyRobot2

它们属于同一个 tinyRobot fleet。Dispatcher 向 fleet 发出任务询价，fleet adapter 根据位置、预计完成时间、电量、忙碌状态等，选择一个机器人生成 bid；中标后整
个 patrol task 由该机器人完成。

当前配置的分配策略是 DefaultFastest，并且 busy_mode: Binary，可见于 rmf_demos/config/office/tinyRobot_config.yaml:37。因此第一条任务通常分给距离更近或预计
完成更快的机器人，另一台没有额外任务就不会移动。

“多机协同”不等于每个任务一定让多台车同时参与。它体现为：

同时出现多个独立任务
-> RMF 将不同任务分给合适的机器人
-> 两车计划在共享道路相遇
-> Traffic Schedule 协调等待、时序或绕行

要观察它，保持当前 patrol 仍在运行时，再开一个终端提交第二个任务：
```
source /home/jazzy/rmf_ws/install/setup.bash

ros2 run rmf_demos_tasks dispatch_delivery \
-p pantry -ph coke_dispenser \
-d hardware_2 -dh coke_ingestor \
--use_sim_time
```
通常第一台已忙的情况下，第二个任务会由空闲的 tinyRobot2 执行。此时才会出现两车同时运动，RMF 的交通调度才有实际对象可协调。

若希望强制指定第二台车，便于验证两车同时运行，可以使用：
```
ros2 run rmf_demos_tasks dispatch_patrol \
-F tinyRobot -R tinyRobot2 \
-p coe lounge -n 3 --use_sim_time
```
但这会指定执行机器人，绕开“由 fleet 自动选择哪台车最合适”的部分；它适合测试 Traffic Schedule，不适合演示自动任务分配。


### 6. rmf fleet adapter 具体实现了什么功能，能否举例说明？

• rmf_fleet_adapter 是 RMF 与“机器人车队系统”之间的适配层。

RMF 不直接控制底盘、导航栈或厂商机器人 API。它负责全局调度、任务分配和交通协调；fleet adapter 则负责把这些抽象决策转换成某个车队能执行的具体动作，并把机器人状态反馈给 RMF。
```
RMF Dispatcher / Traffic Schedule
        |
        | 任务、路径、交通协商
        v
Fleet Adapter
        |
        | 厂商 API / ROS 2 / REST / MQTT 等
        v
Fleet Manager / Robot
```
在当前 office demo 中，rmf_demos_fleet_adapter 是一个示例 adapter。它通过模拟的 fleet manager 与 tinyRobot1、tinyRobot2 通信。

它主要实现以下功能。

1. 注册车队与机器人能力

adapter 启动时告诉 RMF：

- 车队名称是 tinyRobot
- 有 tinyRobot1、tinyRobot2
- 机器人尺寸、安全范围、速度、加速度
- 电池容量、充电速度和低电量阈值
- 支持配送和巡逻任务
- 每台机器人的充电点

例如 rmf_demos/config/office/tinyRobot_config.yaml:5 中定义了这些参数。

这使 RMF 能计算“哪台车能做任务、预计多久完成、是否需要先充电”。

2. 为任务报价并选择机器人

例如提交：
```
ros2 run rmf_demos_tasks dispatch_delivery \
-p pantry -ph coke_dispenser \
-d hardware_2 -dh coke_ingestor \
--use_sim_time
```
流程是：

Dispatcher 广播：谁能完成这单配送？
-> tinyRobot fleet adapter 评估 tinyRobot1 与 tinyRobot2
-> 选择更合适的一台，计算完成时间与成本
-> 提交 BidProposal
-> Dispatcher 将任务授予 tinyRobot fleet
-> adapter 将该任务落实给具体机器人

在同一个 fleet 内，adapter 才知道每台机器人的实时位置、电量、是否忙碌，因此“具体由哪台车执行”是 adapter 的职责。

3. 规划路径并提交 Traffic Schedule

假设 tinyRobot1 获得配送任务，adapter 会：
```
当前位置
-> pantry
-> coke_dispenser 出货
-> hardware_2
-> coke_ingestor 收货
```
它根据 office 导航图生成路径，并把机器人预计在什么时间经过什么位置的 itinerary 提交给 Traffic Schedule。

Traffic Schedule 提供其他机器人的预期轨迹；adapter 据此选择可行路径。若有冲突，adapter 参与协商，可能让机器人等待、延迟进入通道或绕路。

核心实现位于 RMF 的 /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_fleet_adapter/src/rmf_fleet_adapter/events/GoToPlace.cpp:607，其中规划器接收 schedule snapshot 以进行交通感知规划。

4. 将 RMF 计划转换成机器人命令

RMF 给出的目标是逻辑路径和任务步骤，真实机器人控制方式则取决于车队。

例如，厂商可能接受：
```
{
"robot": "tinyRobot1",
"waypoints": [
    [8.1, -3.2, 0.0],
    [9.4, -1.0, 1.57]
]
}
```
或者只接受“去 waypoint pantry”的 REST 请求。

adapter 负责转换。当前 demo 使用 rmf_demos_fleet_adapter/launch/fleet_adapter.launch.xml:13 启动模拟 fleet manager 和 adapter；真实部署时，只需替换这一层的
厂商接口实现，而无需修改 RMF 的调度逻辑。

5. 反馈状态与处理偏差

机器人不一定完全按计划运动。比如：
```
计划：10:00:15 到达门口
实际：门打开慢，10:00:22 才到达
```
fleet adapter 收到真实位置、剩余路径、电池状态后，会：

- 更新 RMF 中机器人的位置和任务状态；
- 向 Traffic Schedule 报告进度或累计延迟；
- 必要时触发重新规划；
- 让受影响的其他机器人调整计划。

这一步非常关键。没有 adapter 的状态反馈，RMF 只能按旧的预测运行，无法正确协调真实世界变化。

6. 调用设施接口完成任务事件

配送任务不只是导航。到达取货点后，adapter 还要通过 RMF 的 dispenser/ingestor 协议执行：
```
到 pantry
-> 请求 coke_dispenser 出货
-> 确认物品已出货
-> 导航到 hardware_2
-> 请求 coke_ingestor 接收
-> 确认完成任务
```
在该 demo 中，对应的是 Gazebo 里的 TeleportDispenser 与 TeleportIngestor 插件。它们用“瞬移物品”的方式模拟现实中的出货机和收货机。

简而言之，fleet adapter 是让 RMF 能接入不同机器人品牌和不同车队控制系统的关键边界层：
```
RMF：决定“谁做什么、何时走哪里”
Adapter：翻译并监督“这台具体机器人如何做到”
Robot/Fleet Manager：执行运动、导航和设备控制
```

### 7. 对话中，提到了 RMF、Adapter、Fleet Manager、Traffic Schedule、Blockade 等概念，问题:
- 这些概念对应的具体模块是哪些？
- 这些模块的组织和层次结构如何？
- 并且列出具体的例子说明


• RMF 不是一个单独节点，而是一组 ROS 2 模块组成的设施级机器人协调系统。当前 office demo 把这些模块组合起来，形成“任务请求 -> 车队执行 -> 交通协调 -> 设施交互 -> 仿真展示”的闭环。

| 概念 | 在当前工作区对应的模块 | office demo 中的实例 |
|------|-------------------------|---------------------|
| RMF | rmf_task_ros2、rmf_fleet_adapter、rmf_traffic、rmf_traffic_ros2、地图与设施模块等的总称 | 整个 ros2 launch rmf_demos_gz office.launch.xml 启动的协调系统 |
| Task Dispatcher | rmf_task_ros2 的 rmf_task_dispatcher | 接收 patrol / delivery 请求，发起 bidding，授予一个 fleet |
| Fleet Adapter | 核心库 rmf_fleet_adapter；demo 实现 rmf_demos_fleet_adapter | tinyRobot_fleet_adapter，将 RMF 计划转为 TinyRobot 车队命令 |
| Fleet Manager | 通常是厂商车队管理系统；本 demo 是模拟实现 | rmf_demos_fleet_adapter/fleet_manager.py，监听/执行 demo 机器人的控制请求 |
| Traffic Schedule | 核心库 rmf_traffic；ROS 2 服务层 rmf_traffic_ros2 | rmf_traffic_schedule，保存所有机器人带时间的 itineraries，检测并协调冲突 |
| Blockade | rmf_traffic 的 blockade 库；ROS 2 层 rmf_traffic_ros2 | rmf_traffic_blockade，按 checkpoint 给 traffic-light 类机器人分段通行许可 |
| 设施协调 | rmf_fleet_adapter 的 door/lift supervisor；各设施插件 | door_supervisor、Gazebo 的 TeleportDispenser / TeleportIngestor |
| 地图与导航图 | rmf_demos_maps、rmf_building_map_tools | office 建筑地图、车道图、门、充电点、pantry 等地点 |
| 仿真与可视化 | rmf_demos_gz、ros_gz_sim、rmf_visualization | Gazebo office 场景与 RViz 状态、轨迹可视化 |

整体层次可以这样看：
```
用户 / UI / CLI
dispatch_patrol、dispatch_delivery
            |
            v
任务层：rmf_task_ros2
rmf_task_dispatcher
任务竞价、授予、生命周期管理
            |
            v
车队协调层：rmf_fleet_adapter
每个 Fleet Adapter 为本车队选机器人、规划并执行任务
            |
            +-------------------------------+
            |                               |
            v                               v
交通层：rmf_traffic / rmf_traffic_ros2      设施层
Traffic Schedule                           door_supervisor
itinerary、冲突检测、协商                  lift_supervisor
Blockade（可选的分段通行许可）             dispenser / ingestor
            |                               |
            +---------------+---------------+
                            |
                            v
设备接口层：Fleet Manager / 厂商 API
REST、ROS 2、MQTT、WebSocket 等
                            |
                            v
机器人、门、电梯、充电桩、出货机
本 demo 中由 Gazebo 插件模拟
```
以 office 的配送任务为例：
```
ros2 run rmf_demos_tasks dispatch_delivery \
-p pantry -ph coke_dispenser \
-d hardware_2 -dh coke_ingestor \
--use_sim_time
```
对应的模块协作是：

1. dispatch_delivery
    向 rmf_task_dispatcher 提交“从 pantry 取 Coke，送到 hardware_2”。

2. rmf_task_dispatcher
    发布 BidNotice，询问哪个 fleet 能完成。

3. tinyRobot Fleet Adapter
    评估 tinyRobot1 / tinyRobot2 的位置、正在执行的工作、电量和预计完成时间，
    选一台并提交 BidProposal。

4. Dispatcher
    将任务授予 tinyRobot fleet。

5. Fleet Adapter
    为具体机器人规划：
    当前位置 -> pantry -> hardware_2；
    生成带预计时间的 itinerary。

6. Traffic Schedule
    将新 itinerary 与其他 RMF participant 的轨迹比较。
    若走廊内会在同一时间相遇，触发 negotiation，让某台车等待或重规划。

7. Fleet Manager
    收到 adapter 的移动命令，驱动 Gazebo 中的 TinyRobot。

8. 设施模块
    机器人到 pantry 后请求 coke_dispenser 出货；
    到 hardware_2 后请求 coke_ingestor 收货；
    完成状态回传给 adapter 和 Dispatcher。

几个需要区分的边界：

- RMF 是系统总称，不等于 rmf_fleet_adapter。
- Fleet Adapter 是 RMF 侧的车队翻译与协调组件；Fleet Manager 是机器人厂商或站点侧真正管理/控制车队的系统。
- Traffic Schedule 负责全局、带时间的路径协调，是 default office demo 中实际主要使用的交通机制。
- Blockade 是可选的 checkpoint 通行许可协议，更适合只能“暂停/继续”的 traffic-light 车队；默认 office 的 full-control tinyRobot 通常不实际使用它。
- 局部避障不是 Traffic Schedule 的职责。机器人靠自身传感器绕开人或障碍；adapter 再将延迟和状态反馈给 RMF，使全局交通计划更新。

启动组织的直接入口是 rmf_demos_gz/launch/office.launch.xml:8。其中加载 rmf_demos/launch/office.launch.xml:7，后者再加载 rmf_demos/launch/common.launch.xml:17 和 rmf_demos_fleet_adapter/launch/fleet_adapter.launch.xml:13。


## 8. 启动 "ros2 launch rmf_demos_gz office.launch.xml" 后，为什么有一个机器人就已经开始运动？
• 这是预期行为，不是启动时自动接收了 patrol 或 delivery 任务，而是 RMF 为机器人配置了“空闲/收尾行为”。

在 office 配置中：
```
rmf_fleet:
finishing_request: "charge"
```
表示车队任务完成后默认去充电。

但 tinyRobot1 有自己的覆盖配置：
```
robots:
tinyRobot1:
    charger: "tinyRobot1_charger"
    finishing_request:
    type: "park"
    waypoint_name: "supplies"
```
也就是说：

- tinyRobot1：空闲时停到 supplies
- tinyRobot2：默认去自己的充电点 tinyRobot2_charger

Gazebo 初始生成两台车时，它们位于各自的 charger waypoint。由于 tinyRobot1 的 charger 和 supplies 不是同一个位置，启动后它会自动规划：
```
tinyRobot1_charger -> supplies
```
因此你会看到它先移动一段距离，然后停在 supplies。tinyRobot2 初始就在自己的充电位置附近，所以通常不会明显移动。

代码流程如下：
```
读取 finishing_request
-> 为 tinyRobot1 设置 park supplies 空闲任务
-> TaskManager 启动 idle task
-> 规划到 supplies
-> Fleet Adapter 调用 fleet manager 的 navigate()
-> Gazebo 中的机器人执行路径
```
对应实现：

- 机器人专属 finishing_request 解析：/home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_fleet_adapter/src/rmf_fleet_adapter/agv/EasyFullControl.cpp:2845
- 设置机器人收尾任务：/home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_fleet_adapter/src/rmf_fleet_adapter/agv/EasyFullControl.cpp:3762
- 空闲时启动该任务：/home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_fleet_adapter/src/rmf_fleet_adapter/TaskManager.cpp:1885
- adapter 最终发送导航命令：rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/fleet_adapter.py:234

所以这段运动可以理解为：

> 机器人启动后执行 RMF 配置好的默认停车行为。

它不是多机协同任务，也不是异常。

如果希望启动后机器人保持原地，可以将车队的 finishing request 改为：
```
rmf_fleet:
finishing_request: "nothing"
```
并删除或修改 tinyRobot1 下的专属 finishing_request。修改配置后需要重新构建并重新 source 工作区。


问题:
1. 在 ~/rmf_ws 下 "MAKEFLAGS="-j4" colcon build" 出现错误:
```
--- stderr: rmf_site_map_msgs                                            
failed to create symbolic link '/home/jazzy/rmf_ws/build/rmf_site_map_msgs/ament_cmake_python/rmf_site_map_msgs/rmf_site_map_msgs' because existing path cannot be removed: Is a directory
```
这是之前没有的

2. demo: "ros2 launch rmf_demos_gz office.launch.xml"中，一个机器人接收某个任务后，如何人为使其提前停止

• 推荐做法是取消该机器人的当前 RMF 任务：
```
source /opt/ros/jazzy/setup.bash
source ~/rmf_ws/install/setup.bash
```
ros2 run rmf_demos_tasks cancel_robot_task \
-F tinyRobot \
-R tinyRobot1

将 tinyRobot1 换成实际在运动的机器人名称。这个脚本会读取 /fleet_states，找到该机器人的当前 task_id，再向 RMF 发送取消请求。

取消过程不是只在 Gazebo 中“刹车”，而是完整地：

取消任务
-> Fleet Adapter 中止当前导航执行
-> 调用 demo Fleet Manager 的 stop API
-> 发布当前位置的空路径，让 Gazebo 机器人停车
-> 移除/更新该机器人的 Traffic Schedule itinerary

如果已知具体任务 ID，也可以取消指定任务：

ros2 run rmf_demos_tasks cancel_task \
-id '<task_id>'

例如 README 中的旧格式示例：

ros2 run rmf_demos_tasks cancel_task -id patrol.dispatch-0

需要注意：取消后，tinyRobot1 可能会先停下、随后又开往 supplies。这是当前配置的空闲行为，不是取消失败：

tinyRobot1:
finishing_request:
    type: park
    waypoint_name: supplies

它的任务被取消后会进入 idle behavior，并自动停车到 supplies。如果希望“取消后永久停在当前位置”，需将该配置改为：

rmf_fleet:
finishing_request: "nothing"

并移除 tinyRobot1 的专属 finishing_request，然后重建 rmf_demos 并重启 launch。

不建议直接调用 demo Fleet Manager 的 /stop_robot REST 接口来做正常停止，因为 RMF 仍认为任务在执行，adapter 可能重新下发导航命令，导致状态与 Traffic Schedule 不一致。任务取消才是与 RMF 协调一致的停止方式。
