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

#### Traffic Schedule 如何协调冲突

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

实际在路径规划阶段，fleet adapter 已使用 schedule 的快照避开现有计划，而不只是“碰撞后才处理”。例如 /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_fleet_adapter/src/rmf_fleet_adapter/events/GoToPlace.cpp:607 将 schedule()->snapshot()、自身 participant ID 和 robot profile 交给 FindPath，得到考虑其他机器人预期轨迹的路径；随后将完整 itinerary 写回 schedule。

机器人因为堵塞、门未及时打开等原因偏离预测时间时，adapter 会发布 itinerary_delay 或更新 itinerary。Schedule 因此得知“该机器人晚了”，能重新判断对其他机器人的影响。

冲突检测在哪里实现

启动入口在 rmf_demos/launch/common.launch.xml:17，启动的可执行程序是：
```
<node pkg="rmf_traffic_ros2" exec="rmf_traffic_schedule" .../>
```
其核心节点源码是 /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/rmf_traffic_ros2/schedule/Node.cpp:78。

其中：

- /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/rmf_traffic_ros2/schedule/Node.cpp:352 订阅 itinerary_set、itinerary_delay、itinerary_reached 等 ROS 2 topic，更新全局交通数据库。
- /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/rmf_traffic_ros2/schedule/Node.cpp:145 调用 rmf_traffic::DetectConflict::between(...)，用两条时空轨迹及两个 profile 检测是否会在同一时段占据相交/过近的空间。

- /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/rmf_traffic_ros2/schedule/Node.cpp:558 收集冲突 participant；/home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/rmf_traffic_ros2/schedule/Node.cpp:619 发布 negotiation_notice，通知相关 fleet adapter 参与协商。

协商通信由 /home/jazzy/rmf_ws/src/rmf/rmf_ros2/rmf_traffic_ros2/src/rmf_traffic_ros2/schedule/Negotiation.cpp:322 实现，使用 negotiation_notice、proposal、rejection、conclusion 等 topic。算法层的协商器位于 /home/jazzy/rmf_ws/src/rmf/rmf_traffic/rmf_traffic/src/rmf/rmf_traffic/agv/SimpleNegotiator.cpp:1。结果通常表现为一方等待、调整出发时刻，或采用另一条可行路线，而不是 Schedule 节点直接操纵机器人。

重要边界是：RMF 的 schedule 协调的是“已接入 RMF 且会汇报计划/状态”的交通参与者。对人、临时障碍物等，仍由机器人本体的传感器与局部避障负责；局部避障造成的延迟需要 adapter 回报，RMF 才能将影响传播到全局计划。

### Blockade 实际做什么

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


### 8. 启动 "ros2 launch rmf_demos_gz office.launch.xml" 后，为什么有一个机器人就已经开始运动？
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

ros2 run rmf_demos_tasks cancel_robot_task \
-F tinyRobot \
-R tinyRobot1
```
将 tinyRobot1 换成实际在运动的机器人名称。这个脚本会读取 /fleet_states，找到该机器人的当前 task_id，再向 RMF 发送取消请求。

取消过程不是只在 Gazebo 中“刹车”，而是完整地：
```
取消任务
-> Fleet Adapter 中止当前导航执行
-> 调用 demo Fleet Manager 的 stop API
-> 发布当前位置的空路径，让 Gazebo 机器人停车
-> 移除/更新该机器人的 Traffic Schedule itinerary
```
如果已知具体任务 ID，也可以取消指定任务：
```
ros2 run rmf_demos_tasks cancel_task \
-id '<task_id>'
```
例如 README 中的旧格式示例：
```
ros2 run rmf_demos_tasks cancel_task -id patrol.dispatch-0
```
需要注意：取消后，tinyRobot1 可能会先停下、随后又开往 supplies。这是当前配置的空闲行为，不是取消失败：
```
tinyRobot1:
finishing_request:
    type: park
    waypoint_name: supplies
```
它的任务被取消后会进入 idle behavior，并自动停车到 supplies。如果希望“取消后永久停在当前位置”，需将该配置改为：
```
rmf_fleet:
finishing_request: "nothing"
```
并移除 tinyRobot1 的专属 finishing_request，然后重建 rmf_demos 并重启 launch。

不建议直接调用 demo Fleet Manager 的 /stop_robot REST 接口来做正常停止，因为 RMF 仍认为任务在执行，adapter 可能重新下发导航命令，导致状态与 Traffic Schedule 不一致。任务取消才是与 RMF 协调一致的停止方式。

## DetectConflict::between()
• Node.cpp:145 调用的这个函数，本质上只回答一个问题：
```
> 两个带有几何外形的机器人，沿各自的时间轨迹运动时，是否有某个时刻进入了对方的安全范围？如果有，最早是什么时候？
```
它不负责重新规划、停车或决定优先级。

调用代码是：
```
rmf_traffic::DetectConflict::between(
vc->description.profile(), vc->route->trajectory(), nullptr,
description->profile(), route->trajectory(), nullptr);
```
这里两个 nullptr 表示本次没有额外传入 itinerary 依赖约束。

### 1. 先做快速返回检查

源码中的 Implementation::between() 首先检查：
1. 两条轨迹至少包含两个 waypoint，能够组成一个 segment。
2. 两个 profile 至少有可用的 footprint/vicinity。
3. 两条轨迹的时间范围是否重叠。

例如：
```
机器人 A：10:00:00 - 10:00:20
机器人 B：10:00:25 - 10:00:40
```
即使两者经过同一个位置，也不会冲突，因为时间完全错开。

源码位置：
```
/home/jazzy/rmf_ws/src/rmf/rmf_traffic/rmf_traffic/src/rmf_traffic/DetectConflict.cpp:891
```
### 2. 将 waypoint 之间的运动转换成连续样条

轨迹中的 waypoint 通常包含：
```
时间
x, y, yaw
速度
```
例如：
```
t=0   (-5, 0)
t=10  ( 5, 0)
```
函数不会只检查 t=0 和 t=10 两个端点，而是把相邻 waypoint 之间构造成 Spline，连续计算 segment 中任意时间的位置和姿态。

因此，即使：
```
A 的 waypoint：(-5, 0) -> (5, 0)
B 的 waypoint：(0, -5) -> (0, 5)
```
两组 waypoint 本身没有相同坐标，函数仍能发现它们在中间位置相交。

### 3. 按时间重叠区间逐段比较

两条轨迹可能分别有很多段：
```
A: A0 -> A1 -> A2 -> A3
B: B0 -> B1 -> B2
```
函数维护两个 segment 游标，每次取当前两个 segment 的时间交集：
```
start_time  = max(segment_a.start, segment_b.start)
finish_time = min(segment_a.finish, segment_b.finish)
```
只在这个时间区间内比较两个运动物体。某一条 segment 结束后，推进对应游标，继续比较下一组 segment。

如果存在 dependencies，Crawler::ignore() 会跳过明确允许忽略的 segment。当前 Node.cpp 调用传入的是 nullptr，所以普通 office demo 中没有这类额外忽略条件。

### 4. Broad phase：先用包围盒排除不可能冲突的情况

对每个样条 segment，函数计算其整个运动范围的二维包围盒，而不是只使用起点和终点。

然后根据 profile 扩张包围盒：
```
轨迹包围盒 + footprint 尺寸
轨迹包围盒 + vicinity 尺寸
```
如果两个包围盒在 x 或 y 方向完全分离，就直接认为这两个 segment 不可能发生冲突，避免进行更昂贵的几何计算。

例如：
```
A segment 范围：x=0..1, y=0..1
B segment 范围：x=10..11, y=10..11
```
显然无需运行精确碰撞检测。

源码位置：
```
/home/jazzy/rmf_ws/src/rmf/rmf_traffic/rmf_traffic/src/rmf_traffic/DetectConflict.cpp:668
```
### 5. Narrow phase：用 FCL 做连续碰撞检测

包围盒有重叠后，函数使用 FCL 的 Continuous Collision Detection：
```
fcl::collide(&obj_a, &obj_b, request, result);
```
配置为：
```
request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;
request.gjk_solver_type = fcl::GST_LIBCCD;
request.num_max_iterations = 15;
```
也就是说，它检测的是两个几何物体在一段时间内的连续运动，而不是：

每隔 0.1 秒采样一次，然后检查采样点

FCL 返回首次接触的归一化时间 time_of_contact，代码再把它转换成真实时间：
```
real_time =
start_time + scaled_time * (finish_time - start_time)
```
由于迭代次数有限，冲突时刻是近似值；但“是否冲突”的判断是主要目标。

### 6. 实际检查的不是单纯 footprint 对 footprint

函数检查两种组合：
```
A 的 footprint  vs  B 的 vicinity
A 的 vicinity  vs  B 的 footprint
```
源码位置：
```
/home/jazzy/rmf_ws/src/rmf/rmf_traffic/rmf_traffic/src/rmf_traffic/DetectConflict.cpp:671
```
含义是：

- footprint：机器人实际占据的空间；
- vicinity：机器人希望周围保持的安全空间。

因此，只要 A 的实体进入 B 的安全范围，或者 B 的实体进入 A 的安全范围，就可以判定冲突。

如果 vicinity == footprint，两者就退化为普通的实际几何碰撞检查。

如果 profile 没有设置 vicinity，代码会自动把 footprint 作为 vicinity：
```
if (!output.vicinity)
output.vicinity = output.footprint;
```
### 7. 起点已经很近时的特殊处理

between() 先检查两个 segment 的起始状态是否已经处于彼此 vicinity 内：
```
if (close_start(...))
return detect_approach(...);
```
如果机器人一开始就很近，函数不会简单地把“已经在安全范围内”立刻报告为冲突，而是判断它们是否继续靠近。

这样可以允许：
```
两个机器人初始停在一起
其中一个随后驶离
```
否则机器人刚启动时就会因为初始距离较近而永远产生冲突。

如果初始距离足够远，则使用 detect_invasion()，检测后续是否有机器人侵入另一台机器人的 vicinity。

源码位置：
```
/home/jazzy/rmf_ws/src/rmf/rmf_traffic/rmf_traffic/src/rmf_traffic/DetectConflict.cpp:934
```
### 8. 一个直观例子

假设两个圆形机器人半径均为 0.3 m，vicinity 半径为 0.8 m。

#### 轨迹

机器人 A：
t=0   (-5, 0)
t=10  ( 5, 0)

机器人 B：
t=0   (0, -5)
t=10  (0, 5)

两台机器人都在 10 秒内经过中心点 (0, 0)。

函数大致执行：

1. 发现两条轨迹的时间区间都是 0..10，存在时间重叠。
2. 为两条直线段分别生成样条运动。
3. 计算运动范围包围盒，发现它们可能相交。
4. 使用 FCL 连续检查：
    - A 的半径 0.3 是否进入 B 的半径 0.8 安全区；
    - B 的半径 0.3 是否进入 A 的半径 0.8 安全区。

5. 当中心距离小于：
```
0.3 + 0.8 = 1.1 m
```
时，报告冲突。
6. 返回：
```
Conflict {
a_it,       // A 所在的 segment
b_it,       // B 所在的 segment
time        // 首次进入冲突状态的时间
}
```
如果只把实际碰撞作为标准，则阈值约为：
```
0.3 + 0.3 = 0.6 m
```
但 RMF 通常还会通过 vicinity 提前发现安全距离不足。

### 9. 在 Node.cpp 中如何使用结果

Node.cpp 只关心返回值是否为空：
```
if (found_conflict.has_value())
{
conflicts.push_back({participant, vc->participant});
}
```
因此整体逻辑是：
```
同一地图
+ 没有被依赖关系忽略
+ 时间区间重叠
+ footprint/vicinity 连续几何运动发生接近
= schedule conflict
```
检测结果随后用于 Traffic Schedule 的冲突集合和 negotiation 流程。DetectConflict::between() 本身不会向机器人发送停止命令，也不会自动生成替代路径。


## rmf_traffic::DetectConflict::between(...) 是检测轨迹是否产生重叠，但初始提交的轨迹是如何产生的
• 初始提交的轨迹不是由 DetectConflict::between() 生成的，而是由 Fleet Adapter 内的路径规划器生成的。DetectConflict 只负责验证或发现已提交轨迹之间的时空冲突。

整体链路是：
```
任务目标
-> Fleet Adapter 取得机器人当前位置
-> rmf_traffic::agv::Planner 生成带时间的路径
-> 转换为 itinerary（一个或多个 Route）
-> Schedule Participant 提交到 Traffic Schedule
-> DetectConflict 检查与其他 itinerary 的冲突
-> 有冲突则 negotiation 后重新生成/替换 itinerary
```
以你的 patrol 任务为例：
```
ros2 run rmf_demos_tasks dispatch_patrol -p coe lounge -n 3 --use_sim_time
```
某台机器人被分配到 coe -> lounge 这一段时，adapter 会得到：
```
当前状态：
时间：t0
位置：coe 对应的导航图 waypoint
朝向：当前机器人 yaw

目标：
lounge 对应的导航图 waypoint
```
然后调用 rmf_traffic::agv::Planner。它的输入主要是：

- 导航图：building map 导出的 waypoint、lane、地图名、单向限制、门、电梯、等待点等；
- 机器人运动参数：最大线速度、角速度、加速度、减速度；
- 机器人 profile：footprint 和 vicinity；
- 当前时刻、当前位置、朝向；
- 目标 waypoint；
- 当前 Traffic Schedule 快照或 negotiation 的约束。

规划器在导航图中搜索可走路线，并为每一段 lane 计算实际通过时间。它生成的不是简单几何折线，而是带时间戳、位姿、速度的 Trajectory：
```
t0       coe，静止
t0 + 1s  原地转向
t0 + 5s  经过走廊中的 waypoint
t0 + 9s  到达 lounge 前
t0 + 10s lounge，静止
```
每条轨迹元素大致是：
```
(time, [x, y, yaw], [vx, vy, vyaw])
```
同一张地图上的连续轨迹会组成一个 Route：
```
rmf_traffic::Route {
map = "L1",
trajectory = ...
}
```
整段任务可能形成多个 Route，例如跨楼层时会按不同地图拆分；这些 route 的集合就是 itinerary。随后 adapter 通过其 ScheduleParticipant 调用类似：
```
participant.set_itinerary(plan_id, itinerary);
```
提交给 Traffic Schedule。

关键区别是：
```
Planner：
“从当前位置到目标，按车辆运动能力，预计怎样走、何时到？”

DetectConflict：
“这条预计轨迹和其他已提交的预计轨迹会不会在同一时刻太接近？”
```
初始规划时通常已经会参考当前 schedule，尽量避开已有 itinerary。例如走廊被另一台机器人在 t0+5s ~ t0+15s 占用，规划器可能选择：
```
方案 A：改走另一条 lane
方案 B：在 coe 等待 10 秒后再进入走廊
```
但多机器人可能同时产生初始计划，或运行中出现延迟、任务变更。因此初始 itinerary 提交后，Schedule 仍会做冲突检测；发现冲突则触发 negotiation，adapter 以新的时间约束或禁行约束再次调用 planner，生成替代 itinerary 并替换旧计划。

对应源码职责可从这些模块理解：

- rmf_traffic/agv/Planner：导航图搜索、运动时间计算、生成 Route/Trajectory。
- rmf_fleet_adapter：把任务、机器人状态和 planner 串起来，并执行/提交 itinerary。
- rmf_traffic_ros2/schedule/Node.cpp：维护 schedule、找出已提交 itinerary 的冲突。
- DetectConflict::between()：两个具体 trajectory 的连续时空几何检测。

真实机器人中“当前位置”来自定位系统或 Fleet Manager；office demo 中则来自 Gazebo 仿真状态经 demo fleet adapter 上报。启动时若看到静止或保留轨迹，通常是 adapter 为机器人当前位置提交的 hold/空闲状态；真正让机器人移动的轨迹，通常在任务被分配后才由 planner 产生。


## RMF Traffic Schedule 进行协商 "Robot1 和 Robot2 的当前计划在走廊发生时空冲突，请协商。"
两个 robot fleet adapter 根据各自的任务、当前位置、电量、预计完成时间等，给出替代方案。例如：
```
Robot1 提案：按原计划通过走廊
Robot2 提案：在 B 前的 holding point 等待 10 秒，之后再进入
```
但考虑实际情况，如果出现 1:
```
Robot1 提案：按原计划通过走廊
Robot2 提案：按原计划通过走廊
```
2:
```
Robot1 提案：在 A 前的 holding point 等待 10 秒，之后再进入
Robot2 提案：在 B 前的 holding point 等待 10 秒，之后再进入
```
这类情况，如何处理？
因为各个 robot fleet adapter 只会考虑自己这个 robot，其他 robot 的提案只有 fleet adapter 知道。
另外为什么不直接由 RMF Traffic Schedule 进行总体协商？


• 核心点是：RMF 的协商不是“两个 adapter 各自提交一次方案，然后 Traffic Schedule 从两份方案中挑一个”。它是一个由 Traffic Schedule 组织的、按候选方案逐步展开的分布式协商协议。

每个 adapter 虽然只负责自己的机器人，但在协商过程中会看到协商表中已经提交的其他参与者方案。它提交的也不是一句“我等待 10 秒”，而是自己的完整候选 itinerary，也就是带时间戳的完整轨迹。

### 协商中谁做什么

可以把角色分成三层：
```
Traffic Schedule / Negotiation Manager
    负责发现冲突、建立协商、传递候选方案、判断方案是否仍冲突

Robot1 Fleet Adapter / Negotiator
    负责生成 Robot1 的候选 itinerary

Robot2 Fleet Adapter / Negotiator
    负责生成 Robot2 的候选 itinerary
```
Traffic Schedule 通常不会替机器人决定“Robot1 应该等，还是 Robot2 应该等”，因为它并不知道：

- Robot1 的任务优先级；
- Robot2 是否电量不足；
- 某个 fleet 是否允许倒车；
- 某个 holding point 是否适合该车型；
- 任务的截止时间和业务代价；
- fleet adapter 内部的控制能力。

它只负责协议和安全判定：候选轨迹放在一起后是否仍有冲突。

### 情况 1：两个机器人都坚持原计划

例如：
```
Robot1：原计划通过走廊
Robot2：原计划通过走廊
```
Traffic Schedule 会把两个 itinerary 放在同一个协商表中，并再次执行冲突检测：
```
DetectConflict(Robot1 原计划, Robot2 原计划)
    => 仍然冲突
```
因此这个方案不能被接受。

之后协议会要求参与者继续提交替代方案。例如：
```
Robot1：原计划
Robot2：等待 10 秒后再走
```
或者：
```
Robot1：等待 10 秒后再走
Robot2：原计划
```
如果某个 adapter 反复提交原计划，协商分支就无法解决冲突，最终会出现：

- 该协商分支被判定为失败；
- 协商超时或参与者放弃该分支；
- fleet adapter 重新规划；
- 任务进入失败、等待或需要人工处理的状态。

重要的是：协商失败时，系统不会把两个仍然冲突的方案当成成功方案下发给机器人。

### 情况 2：两个机器人都等待 10 秒

例如：
```
Robot1：在 A 前等待 10 秒
Robot2：在 B 前等待 10 秒
```
这不一定能解决冲突。

假设原计划如下：
```
Robot1：10:00:00 进入走廊
Robot2：10:00:00 进入走廊
```
两者都等待 10 秒后：
```
Robot1：10:00:10 进入走廊
Robot2：10:00:10 进入走廊
```
它们仍然在同一时间占用走廊，因此：
```
DetectConflict(Robot1 等待后的轨迹,
                Robot2 等待后的轨迹)
    => 仍然冲突
```
这个候选组合会被拒绝。

但如果等待策略不同，例如：
```
Robot1：原计划，10:00:00 通过
Robot2：等待 10 秒，10:00:10 通过
```
并且 Robot1 已经完全离开走廊，那么两条轨迹的空间占用时间不重叠：
```
DetectConflict(...)
    => 无冲突
```
这个候选组合才可能被接受。

所以 Traffic Schedule 判断的不是“两个方案看起来是否合理”，而是把完整轨迹组合起来重新进行时空检测。

### Robot adapter 怎么知道其他机器人的方案

不是只有 Robot1 知道 Robot1 的提案、Robot2 知道 Robot2 的提案。

协商过程中的某个参与者会看到类似这样的表：

当前协商表：
```
Robot1:
itinerary_0 = 原计划

Robot2:
itinerary_0 = 等待 10 秒后的计划
```
当轮到 Robot1 响应时，它可以根据表中 Robot2 的 itinerary 生成自己的替代方案：
```
Robot1:
candidate_0 = 原计划
candidate_1 = 等待 5 秒
candidate_2 = 改走另一条 lane
```
然后 Traffic Schedule 会将这些候选组合继续检查。

因此 adapter 不需要预先知道对方“打算怎么做”，但在协商协议中会逐步看到已经提交的对方候选轨迹，并针对它进行响应。

### 这实际上是一个搜索过程

可以把协商抽象成：
```
冲突：
R1 原计划 + R2 原计划

尝试分支 1：
R1 原计划 + R2 等待 10 秒
-> 无冲突，成功

尝试分支 2：
R1 等待 10 秒 + R2 原计划
-> 无冲突，成功

尝试分支 3：
R1 等待 10 秒 + R2 等待 10 秒
-> 仍冲突，失败

尝试分支 4：
R1 改道 + R2 原计划
-> 无冲突，成功
```
Traffic Schedule/Negotiation Manager 负责管理这些分支和参与者顺序；每个 adapter 负责为自己的机器人产生候选 itinerary。

最终接受的是一个满足以下条件的完整组合：
```
每个参与者都有一个有效 itinerary
所有 itinerary 两两之间无未解决冲突
满足依赖关系和协商约束
```
### 为什么不让 Traffic Schedule 统一规划所有机器人

主要是职责和信息边界不同。

Traffic Schedule 擅长的是：
```
维护全局时间空间占用记录
检测冲突
组织协商
管理依赖和参与者状态
```
但它不应该负责：
```
理解每种机器人底盘的运动学
知道每个 fleet 的控制接口
决定电量不足的机器人是否必须充电
判断某个 holding point 是否可停
处理机器人无法倒车、无法原地旋转等限制
优化任务优先级和业务成本
```
这些信息通常只存在于 fleet adapter 或 fleet manager 内部。

例如同一个冲突：
```
Robot1：可以原地旋转，等待成本低
Robot2：不能倒车，且电量低，必须优先完成任务
```
对 Traffic Schedule 来说，两条轨迹只是几何和时间对象；它无法可靠判断哪台机器人更适合等待。由各自 adapter 生成候选方案，才能使用本 fleet 的真实能力和业务规则。

因此 RMF 采用的是：
```
Schedule 负责全局一致性和安全验证
Adapter 负责本机器人候选方案和执行能力
```
Traffic Schedule 不是完全被动的。它实际上掌握全局冲突信息，并负责协调协商流程；只是它不替所有 fleet 做具体路径优化。

最后要注意：DetectConflict::between() 只检测给定的两条轨迹，不负责协商本身。协商流程发现某个候选仍冲突时，会退回给相应的 negotiator 生成另一个候选 itinerary。只有在整个候选组合无冲突后，新的 itinerary 才会被正式提交并用于机器人执行。


## 刚才说的这种冲突协商方式，意味着机器人 A 要获取到机器人 D 提交的 itinerary (D 必须在时间上先于 A 提交 itinerary)。如果 A、D 是一起提交 itinerary，那么很可能产生新冲突。我这样理解是否正确？

• 你的理解对了一半，需要区分“初始 itinerary 提交”和“进入 negotiation 后的提案提交”。

初始提交阶段确实可能出现你说的竞态：
```
t0：A 和 D 都读取 Schedule 版本 100
t1：A 根据版本 100 规划 itinerary_A
t1：D 根据版本 100 规划 itinerary_D
t2：A、D 几乎同时提交
t3：Schedule 发现 itinerary_A 与 itinerary_D 冲突
```
因为 A 规划时看不到 D 尚未提交的轨迹，D 同样看不到 A。两份单独看都合法的 itinerary，组合起来可能冲突。这是 RMF 预期要由后续 negotiation 处理的情况。

不过在 negotiation 阶段，不要求现实时间上“D 必须先发布正式 itinerary，A 才能规划”。Traffic Schedule 会建立一张有顺序、有版本的协商表。

例如某个协商分支中：
```
A 提交候选 A1
    ↓
D 收到包含 A1 的协商表
    ↓
Schedule 检查 A1 + D1
```
如果 A1 + D1 冲突，D 可以拒绝这个分支，或者协议换一个分支：
```
D 提交候选 D2
    ↓
A 针对 D2 提交候选 A2
    ↓
Schedule 棏查 A2 + D2
```
这里的“先后”是某个 negotiation 分支中的逻辑顺序，不是要求机器人初始计划必须永久按固定顺序提交。

可以理解为：
```
初始 itinerary：
    允许并发提交
    允许因为旧 Schedule 快照而产生冲突

协商候选 itinerary：
    按协商表逐层提交
    后响应者能够看到该分支中前面参与者的候选
    整个候选组合无冲突才算协商成功
```
即使协商成功后，又有第三台机器人 E 并发提交了新 itinerary，导致：
```
A2 + D2 无冲突
但 A2 + E1 冲突
```
Schedule 仍会检测到这个新冲突，并启动新的协商。RMF Traffic Schedule 更接近“带版本和冲突恢复机制的乐观并发协调”，而不是把整个场地加锁后一次性规划所有机器人。

因此不能保证“任何时刻都绝不会有人提交冲突 itinerary”，它保证的是：
1. 所有 itinerary 最终进入同一个 Schedule。
2. 冲突能够被全局检测。
3. 协商候选必须针对已知提案进行验证。
4. 冲突候选组合不能被当作成功协商结果。
5. 因并发更新产生的新冲突会再次触发协商。

还要区分“提交计划”和“实际进入冲突区域”。机器人可以先发布预计轨迹，但 adapter 会根据协商结果、依赖关系和时间进度控制实际执行。RMF 的目标不是禁止冲突计划短暂出现在 Schedule 中，而是阻止机器人按照尚未解决的冲突计划同时占用危险区域。
