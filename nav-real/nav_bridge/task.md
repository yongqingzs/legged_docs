# @/home/jazzy/drive_ws/src/nav_bridge 完成了哪些功能，输出 markdown 到 /home/jazzy/agent_ws/src/legged_docs/nav-real/nav_bridge 下。

问题:
1. 我现在想新增 nav_bridge 适配智元 d1 max，这是 d1 max 的 sdk: @/home/jazzy/drive_ws/src/Agibot_D1_Max(ubuntu22.04 的 sdk，sdk 文档 @/home/jazzy/drive_ws/src/Agibot_D1_Max/ubuntu22.04(x86_64_arm64)/RobotSDK-0.1.1/docs)，请分析并说明你的适配方案，写入 /home/jazzy/agent_ws/src/legged_docs/nav-real/nav_bridge 下

2. 不要继续扩展当前 NavBridgeBase，应先引入 RobotBackend(进行真正的后端抽象)，再迁移 X30；完成 x30 迁移后根据 @/home/jazzy/drive_ws/src/Agibot_D1_Max 迁移 d1 max，但注意 nav_bridge 最终跑在 arm64 上

3. 你不要链接 @/home/jazzy/drive_ws/src/Agibot_D1_Max 上的 lib，你如果需要使用 @@/home/jazzy/drive_ws/src/Agibot_D1_Max 中的文件，直接复制进 nav_bridge 下某目录(看你觉得哪里合适，或者 third_party/)

4. 我已经编译了 @/home/jazzy/drive_ws/src/Agibot_D1_Max/ubuntu22.04(x86_64_arm64)/RobotSDK-0.1.1/example/build，这些例子分别代表什么？我已经通过无线网连接 192.168.234.1(ssh robot@192.168.234.1 密码: bot)，但发现运行"./data 192.168.234.1 8081"出现"[ERROR] Connect failed: Robot ShakeHand failed [ERROR] Not connected. State: 3"，请查找原因。

5. 刚才适配的 @/home/jazzy/drive_ws/src/Agibot_D1_Max 是老版本的，我拿到实际可用的新版本 @/home/jazzy/drive_ws/src/RobotSDK-0.2.1，里面的 example/build/control 例程该如何使用以控制机器狗(按键分别表示什么)，并更新 @/home/jazzy/agent_ws/src/legged_docs/nav-real/nav_bridge/D1_MAX_SDK_EXAMPLES_AND_HANDSHAKE.md

6. nav_bridge 刚才新增 d1 max 支持依据 @/home/jazzy/drive_ws/src/Agibot_D1_Max(版本 0.1.1)，而 @/home/jazzy/drive_ws/src/RobotSDK-0.2.1 才是最新的，请根据最新 sdk 更新代码和链接库

7. 请根据你最新的适配方案，更新 @/home/jazzy/agent_ws/src/legged_docs/nav-real/nav_bridge/D1_MAX_ADAPTATION_PLAN.md，说明你对 d1 max 的实际适配，并和 x30 比较

8. 我看 nav_bridge 中还是有 nav_bridge_base.hpp、x30_nav_bridge.hpp 等文件，说明 x30 还是用的以前的适配方案，没有接入 robot_backend? 这符合预期吗，先讨论，不要修改源码。

9. nav_bridge_base.hpp 和 robot_backend.hpp 分别作为基类有什么区别？为什么新方案要改为 robot_backend，nav_bridge_base 作为基类有什么问题？

10. d1_max_backend 只是从 d1_max 的 sdk 里取出数据吗? 它没有像@/home/jazzy/drive_ws/src/nav_bridge/launch/nav_bridge.launch.py 一样转换为 ros2 消息发出吗?

# 将 X30 也拆成 X30Backend，然后让 X30 和 D1 共用一个公共 ROS 装配层，而不是让 X30 继续把后端和 ROS 逻辑都放在 X30NavBridge 里，删除原先的 x30_nav_bridge.hpp、nav_bridge_base.hpp 等。另外共有的类可以放在 @/home/jazzy/drive_ws/src/nav_bridge/include/nav_bridge、@/home/jazzy/drive_ws/src/nav_bridge/src 下，d1_max / x30 独有的需要放在分别的文件夹(如 @/home/jazzy/drive_ws/src/nav_bridge/src/x30)下

问题:
1. 考虑 x30 迁移成本过大，我觉得一开始的方案就有问题，没有考虑 x30 这巨量的迁移代价。因此，我进行了版本回退。请重新审视新增 d1_max 支持的方案(sdk: @/home/jazzy/drive_ws/src/RobotSDK-0.2.1)，需要满足以下要求:
- x30 原先的代码可以保留，但不是共有的代码需要放入 x30 独立文件夹下
- d1_max 独有代码也要放入独立文件夹
- 对外只暴露一个 launch，launch 通过 yaml 选择机器狗型号
- d1_max 对外暴露的 ros2 接口需要和 x30 尽量一致，从而更好地适配导航
- 不要链接 RobotSDK-0.2.1 的本地路径，可以复制进 nav_bridge 的 third_party 下
重新审视方案

2. @/home/jazzy/drive_ws/src/nav_bridge/src/nav_bridge_node.cpp 和 @/home/jazzy/drive_ws/src/nav_bridge/src/d1_max/d1_max_nav_bridge_node.cpp 的关系很奇怪阿，你是怎么看待和处理的？原先一些不适合作为通用类的可以改名或者增加适配选项，现在的处理有些别扭

# 在 d1_max 上测试
说明:
1. 当前已经连接 d1_max
2. d1_max 有两个主机
- 运动主机: ssh robot@192.168.234.1(密码: bot)
- 导航主机(需先连接运动主机再 ssh): ssh robot@192.168.168.100(密码: 1)
3. 请在 nav_bridge 部署在 d1_max 导航主机的 ~/Workspace/driver_ws/src 下
4. 真机测试 nav_bridge 对 d1_max 的适配是否正确

问题:
1. 先跳过刚才的问题。我在导航主机上启动 d1_max 的 launch 后，我该如何手动用哪些命令进行检验
2. 我在导航主机上测试"ros2 topic hz /imu/data"，发现只有 13 hz 左右，但是"ros2 topic hz /imu_driver/imu_central"有 200 hz，200 hz 是合理的，请排查问题(/imu_driver/imu_central 来自哪里)，sdk: @/home/jazzy/drive_ws/src/RobotSDK-0.2.1

3. 咱们通过 sdk 得到的 imu/data 频率为什么这么低，连 100 也没有达到，在 @/home/jazzy/drive_ws/src/RobotSDK-0.2.1/example/build 测试 "./data 192.168.234.1 8082" 时似乎也没有这么低

## 4. x30 的运动模式切换如何，d1_max_nav_bridge 有做运动模式切换吗

例如：
```
ros2 service call /nav_bridge_node/set_gait \
rcl_interfaces/srv/SetParameters \
"{parameters: [{name: gait, value: {type: 4, string_value: 'WALK'}}]}"
```
X30 切换时会：

- 检查当前基本状态和步态；
- 判断当前是否允许切换；
- 必要时先从 FORCE_STAND 进入踏步状态；
- 发送具体 X30 UDP 步态命令；
- 等待反馈状态确认；
- 对 RL 步态和普通步态采用不同的等待逻辑；
- 限制 /cmd_vel 只能在兼容的模式下发送。

| 能力 | X30 | D1 Max 当前实现 |
|---|---|---|
| 通用 `set_mode` | 没有单独依赖，使用 `set_gait` | 有接口，但明确返回不支持 |
| 步态切换 | 已实现，包含状态机和反馈等待 | 未实现 |
| 站立 | 包含复杂步态切换流程 | 直接调用 `StandUp()` |
| 登阶 / 匍匐 / 爬高台等 | X30 有专用协议支持 | SDK 有 API，但 `nav_bridge` 尚未暴露 |
| 模式状态反馈 | 详细发布基本状态和 gait 状态 | 仅映射为统一 `BackendMotionState`，`mode=0` |
| `/cmd_vel` 前置检查 | 严格检查当前步态 / 模式 | 主要依赖控制权，未做等价步态兼容检查 |


X30 当前对外提供的 ROS 服务包括：
```
~/stand
~/lie
~/soft_estop
~/release_control
~/set_gait
~/set_body_height
~/charge_command
```
在统一节点名 nav_bridge_node 下，实际服务名通常是：
```
/nav_bridge_node/stand
/nav_bridge_node/lie
/nav_bridge_node/soft_estop
/nav_bridge_node/release_control
/nav_bridge_node/set_gait
/nav_bridge_node/set_body_height
/nav_bridge_node/charge_command
```
各接口含义如下：

- stand：执行完整站立状态机。不是简单发一个站立报文，会根据当前状态处理力控站立、踏步、RL 步态和目标步态切换。
- lie：执行趴下/卧倒流程。
- soft_estop：发送软急停。
- release_control：发送零速度、停止心跳并释放控制权。
- set_gait：切换导航相关步态，例如 WALK、SLOPE、MOUNTAIN、L_WALK、L_STAIR 等。
- set_body_height：切换身体高度，包括普通高度和匍匐高度，并检查当前步态是否允许。
- charge_command：处理充电相关状态机，包括启动、停止、复位、查询等命令。

此外，X30 还发布与导航状态机相关的话题：
```
/robot_basic_state
/robot_gait_state
/robot_body_height_state
/charge_manager_state
/battery/level
```

/cmd_vel 也不是无条件转发。X30 会检查：
- 是否已经取得控制权；
- 当前是否处于允许运动的基本状态；
- 当前步态是否兼容导航速度控制；
- 速度是否超出该步态的限制；
- 控制命令是否超时。

所以 X30 现有实现本质上是一个面向导航的状态机，而不是一组原生接口的简单转发。

需要特别区分：

- X30 的 set_gait、set_body_height 是 nav_bridge 设计的导航接口；
- X30 底层 UDP 的各种命令只是实现手段；
- 上层导航不应该感知底层具体命令；
- D1 适配应实现同样的导航语义，而不是把 SDK 的 Stair()、Crawl()、Gait() 等 API 全部新增暴露出来。

当前 D1 只实现了其中一部分公共语义：
```
stand
lie
soft_estop
release_control
set_speed
```
还没有实现与 X30 等价的 set_gait、set_body_height 和充电状态机。set_mode 目前只是兼容性占位接口，并不是真正可用的导航模式切换接口。

## D1 stand 修改
把 D1 的 stand() 改为内部流程，例如：
1. 检查 SDK 是否连接
2. 获取当前 MotionStatus
3. 如果处于软急停，先解除或恢复到安全状态
4. TakeControl
5. 如果当前正在行走，发送零速度并等待运动停止
6. 根据当前状态选择 StandUp 或 BalanceStandUp(如果已经是通用模式，则跳过 6、7，如果是别的运动模式，则切回通用模式)
7. 设置当前运行模式(SetMode 为通用模式，同时删除 D1 暴露的 set_mode，和 x30 一致使用 set_gait，对应 MOUNTAIN)
8. 将速度设置成中等
9. 将 D1 状态映射成统一导航状态
10. 返回 stand 成功

问题:
1. X30 的 lie 完成了哪些工作？ D1 MAX 该如何实现类似的逻辑？

## D1 lie 修改
D1 内部可以按下面流程实现：
1. 检查 SDK 是否连接
2. 读取当前 MotionStatus
3. 如果已经是 LIE_DOWN，直接成功
4. TakeControl
5. 如果当前是 WALK、CRAWL_WALK、STAIR 等运动状态：
    - 发送 Move(0, 0, 0)
    - 等待运动状态停止或进入可执行姿态
6. 如果当前是特殊动作状态：
    - 根据 MotionStatus 选择退出动作或等待动作完成
7. 如果是站立或其他运动模式，先切为匍匐模式，再趴下，这样会更自然

## D1 set_gait 适配
D1 对于 /nav_bridge_node/set_gait 该如何适配，我觉得:
- 通用模式 低速 对应 WALK
- 通用模式 中速 对应 MOUNTAIN
- 通用模式 高速 对应 RUN
- 登阶模式 对应 台阶
你觉得呢

## D1 cmd_vel 适配
D1 /cmd_vel 如何实现类似 x30 的逻辑

/cmd_vel 也不是无条件转发。X30 会检查：
- 是否已经取得控制权；
- 当前是否处于允许运动的基本状态；
- 当前步态是否兼容导航速度控制；
- 速度是否超出该步态的限制；
- 控制命令是否超时。

实现 D1 的逻辑：
```
ROS /cmd_vel
-> D1 节点缓存速度
-> RobotBackend 负责控制权和 Move
-> MotionStatus 负责状态门控
-> 定时器以固定频率发送
-> 动作流程期间暂停发送
-> 超时发送零速度
```
这样上层导航看到的 /cmd_vel 行为就会与 X30 基本一致，同时 D1 的 SDK 差异仍被隐藏在 backend 内部。

## D1 适配话题
以下命令 D1 如何适配，以类似 X30:
- set_body_height：切换身体高度，包括普通高度和匍匐高度，并检查当前步态是否允许。(直接切换为 匍匐模式？set_gait 里有对应匍匐模式的码吗)
- charge_command：可以先返回不支持

此外，X30 还发布与导航状态机相关的话题：
```
/robot_basic_state
/robot_gait_state
/robot_body_height_state
/charge_manager_state
/battery/level
```
D1 如何适配？实现类似的效果，充电先显示不支持

## D1 imu/data 这个话题要不先直接转发 /imu_driver/imu_central？你觉得如何，合理吗？

## 当前 x30_nav_bridge 是选择性编译的吗(因为你之前对 cmake 的修改)？
```
colcon build --packages-select nav_bridge \
--cmake-args -DNAV_BRIDGE_BUILD_X30=OFF \
-DNAV_BRIDGE_BUILD_D1_MAX=ON
```

问题:
1. 照当前的修改，@/home/jazzy/drive_ws/src/nav_bridge/include/nav_bridge/x30/nav_bridge_base.hpp 适合成为 d1_max 和 x30 的共同基类吗？也就是舍弃 d1_max_backend、robot_backend，以重新建立子类。客观分析。

说明:
1. 当前已经连接 d1_max
2. d1_max 有两个主机
- 运动主机: ssh robot@192.168.234.1(密码: bot)
- 导航主机(需先连接运动主机再 ssh): ssh robot@192.168.168.100(密码: 1)
3. 请将 nav_bridge 更新在 d1_max 导航主机的 ~/Workspace/driver_ws/src 下
4. 真机测试 nav_bridge 对 d1_max 的适配是否正确，你先测试 imu 消息(ros2 转发)是否正常达到 200hz
5. 测试完成后请清理进程

问题:
1. 我发现我在导航主机上启动，/imu/data 只有 170 hz
2. 一直打印"IMU callback count=800 elapsed=64.5646 s rate=12.3907 Hz"，我是不需要这个打印的，另外为什么显示 12.3907 hz，难道转发 ros2 消息时也在读取回调里的 imu 数据吗？打印需要删除，并且需要分析当前 imu 数据是否并行读取。
3. 测试完成后请清理进程
先分析，出解决方案

解决方案:
第一步，删除 SDK IMU 频率打印。

第二步，默认 imu_driver 模式下不启用 SDK IMU 配置和接收；imu_source=sdk 时才启用。imu_driver 同理

第三步，将 /imu/data 发布器改为传感器数据 QoS，降低可靠传输背压：
```
imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS());
```
第四步，在导航主机重新测量：
```
ros2 topic hz /imu_driver/imu_central
ros2 topic hz /imu/data
ros2 topic info /imu/data -v
```

问题:
1. 当前 d1 切换 lie 是什么流程，我实机测试，怎么会有撞击地面的情况？不是先匍匐模式再卧倒吗？

2. 当前 d1 切换为通用模式后，应该能接收 /cmd_vel 信息，我该如何用键盘测试其是否正常，说明并写入 @/home/jazzy/agent_ws/src/legged_docs/nav-real/nav_bridge/D1_MAX_MANUAL_TEST_CHECKLIST.md

3. 这是对应 d1 通用模式的切换:
```
ros2 service call /nav_bridge_node/set_gait \
  rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: gait, value: {type: 2, integer_value: 33}}]}"
```
其他模式应该发什么: "type: 2, integer_value: 33"

## 最新的 D1 连接方式
说明:
1. 当前已经连接 d1_max
2. d1_max 有两个主机
- 运动主机: ssh robot@10.0.40.216(密码: bot)
- 导航主机(需先连接运动主机再 ssh): ssh robot@192.168.168.100(密码: 1)
3. 请将 nav_bridge 更新在 d1_max 导航主机的 ~/Workspace/driver_ws/src 下
4. 真机测试 nav_bridge 对 d1_max 的适配是否正确，你先测试 imu 消息(ros2 转发)是否正常达到 200hz
5. 测试完成后请清理进程

## D1 自主充电
问题:
1. @/home/jazzy/drive_ws/src/RobotSDK-0.2.1/docs/zh 中有提供关于 D1 MAX 自主充电的说明吗？@/home/jazzy/drive_ws/src/RobotSDK-0.2.1/example/recharge.cpp 里是关于 D1 MAX 自主充电的实现示例吗？参考 X30 评估一下 D1 MAX 自主充电的实现方案。

2. @/home/jazzy/drive_ws/src/Agibot_D1_Max/docs/source/5.2自主回充说明.md 这是老版本 sdk 对 d1 自主充电的说明，@/home/jazzy/drive_ws/src/RobotSDK-0.2.1 的 example 中 "./control 192.168.168.168 8081 192.168.168.100 10010" 是否实现一样的功能

```bash
./control 10.0.40.216 8081 192.168.168.100 10010
```

3. @/home/jazzy/drive_ws/src/RobotSDK-0.2.1/docs/zh/sdk_recharge_task_zh.md 是否就是 0.2.1 sdk关于自主充电的最新说明，请分析，我们需要测试哪些接口？现有 recharge 示例是否能满足测试接口的要求，其如何用？

假设 0.2.1 的接口正常，操作要求: "将四足机器人D1 Max设备正对充电桩，设备头部距离充电桩1.5m左右，确保在相机视野中可完整看到充电桩的二维码"，该如何在 nav_bridge 新增对 d1 自主充电的支持，类似 x30 评估方案。

4. 当前先认为假设成立，因为充电桩还在修。另外，nav_bridge 启用充电前会先由 nav_bridge 的上层导航移动至"将四足机器人D1 Max设备正对充电桩，设备头部距离充电桩1.5m左右，确保在相机视野中可完整看到充电桩的二维码"。所以先实现你的方案，并且将方案写入 @/home/jazzy/agent_ws/src/legged_docs/nav-real/nav_bridge。

这是:
```
## 最新的 D1 连接方式
说明:
1. 当前已经连接 d1_max
2. d1_max 有两个主机
- 运动主机: ssh robot@10.0.40.216(密码: bot)
- 导航主机(需先连接运动主机再 ssh): ssh robot@192.168.168.100(密码: 1)
3. 测试完成后请清理进程
```
我在导航主机(Jetson NX 16g)上通过 apt 安装了 opencl，但显示:
```
robot@orin-nx:~$ clinfo
Number of platforms  0
```
请查看，并设计实施方案使得 OpenCL 真正能调用 nx 上的 gpu。


## 修改项
问题:
1. nav_bridge d1_max 启动后，我发送充电任务:
```bash
ros2 service call /nav_bridge_node/charge_command \
  rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: charge_command, value: {type: 2, integer_value: 0}}]}"
```
但实际可能由于充电桩故障没有完成充电任务，但这个 service 立马返回 success，这是不对的。应该实际唤起 sdk 的充电任务后，过一段时间确认其在充电状态才可以认为其在充电。退出充电也类似。并且我发现没有在充电中，可以趴下/起立，但可能 cmd_vel 无法接收的情况，我怀疑是充电任务屏蔽的问题。先分析。参考 @/home/jazzy/drive_ws/src/RobotSDK-0.2.1/docs/zh/sdk_recharge_task_zh.md、@/home/jazzy/drive_ws/src/RobotSDK-0.2.1/docs/zh/sdk_state_zh.md 等

2. 当前 d1 max 实现的进入/退出充电的语义化和 x30 接近吗？请比较 

3. @/home/jazzy/drive_ws/src/RobotSDK-0.2.1/docs/zh 这里面是否有一个 gait 步态，能否将其映射为 d1 max 的 l-walk 步态(x30的定义)，请评估

4. d1 nav_bridge 有使用过这个函数吗
"
std::error_code Gait(int timeout_ms = 0,
WriteHandler handler = [](const std::error_code&, std::size_t) {})
"

5. 当前启动充电任务,我故意让视野里没有充电桩(使得无法完成充电)，但还是返回:
```
response:
rcl_interfaces.srv.SetParameters_Response(results=[rcl_interfaces.msg.SetParametersResult(successful=True, reason='D1 task confirmed running.')])
```
评估该问题

6. 你这个实现方式是错误的，不应该 sdk 返回什么你就直接返回什么。对于充电任务，至少调用 sdk 开始充电任务后，通过"确认充电状态处于充电中"，你才能返回结果(如果 sdk 第一次调用就失败，则直接返回失败)。否则你这个 service 返回将没有意义。进行离开充电桩任务也类似，现在无论是否在充电桩上你都直接调用"离开充电桩"，这不是瞎搞吗？和充电相关的都需要确认状态。你不能瞎搞。请评估。


7. 我运行在 cat 板卡上运行 nav_bridge，出现:
```
(D1M-A) ~ ros2 service call /nav_bridge_node/charge_command \
  rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: charge_command, value: {type: 2, integer_value: 0}}]}"

[ERROR] [1788400573.739893010] [rmw_zenoh_cpp]: z_reply_is_ok returned False Reason: Timeout for service '/nav_bridge_node/charge_command'
```
但其实已经进入充电。请评估。


## cat 板卡调试 nav_bridge
有三个板卡:
cat板卡: ssh cat@10.0.40.226，密码: cat
运动主机(需先连接cat板卡再 ssh): ssh robot@192.168.168.168(密码: bot)
导航主机(需先连接运动主机再 ssh): ssh robot@192.168.168.100(密码: 1)
几个板卡上的模块通过 ros2 连通

cat 板卡上调用 service，超过 10s 会报错
```
[ERROR] [1788400573.739893010] [rmw_zenoh_cpp]: z_reply_is_ok returned False Reason: Timeout for service '/nav_bridge_node/charge_command
```
cat 需要先运行
如何修改配置

修改文件：
```
/home/cat/Workspace/zenoh_ws/router_config.json5
```
新增：
```
queries_default_timeout: 120000,
```
含义为 Zenoh service/query 最长等待 120 秒，覆盖 D1 充电和离桩任务的 30-60 秒执行时间。

已完成：

- 原配置备份：
```
/home/cat/Workspace/zenoh_ws/router_config.json5.bak.20260903-104602
```
- 停止旧 rmw_zenohd
- 使用正确 ROS 环境重新启动：
```
PID 11989
/home/cat/Workspace/zenoh_ws/install/rmw_zenoh_cpp/lib/rmw_zenoh_cpp/rmw_zenohd
```
- 确认进程使用：
```
ZENOH_ROUTER_CONFIG_URI=/home/cat/Workspace/zenoh_ws/router_config.json5
```
现在 Cat 上调用充电 service 不会再因 Zenoh 默认 10 秒超时而提前失败，最长可等待 120 秒。

日志中的 timestamp warning 与 service 超时无关，是其他板卡时钟/时间戳偏差问题。


### 问题
cat 主机(ssh cat@10.0.40.226，密码: cat)上运行 zenoh
```
ros2 run rmw_zenoh_cpp rmw_zenohd
```
显示
```
2026-09-03T03:24:13.835206Z ERROR rx-1 ThreadId(07) zenoh::net::routing::dispatcher::pubsub: Error treating timestamp for received Data (incoming timestamp from cdadde95c7cda559e960a02cf5ad29fc exceeding delta 500ms is rejected: 2026-09-03T03:27:02.029631477Z vs. now: 2026-09-03T03:24:13.835202452Z). Replace timestamp: Some(7681144654197164496/132458b6777f9f9976080f2567264e7a)
```
什么原因，请评估

问题:
```
只要 SDK 最近一次任务状态仍是：
TaskType == RECHARGING 或 UNDOCK
TaskStatus == STARTING 或 RUNNING
就会屏蔽非零速度。
```
1. 有这么一种情况会导致这种方式出现问题，比如使用 sdk 进入充电，但是使用遥控器退出充电，sdk 状态没有切换，导致 /cmd_vel 不可控，请评估。

因此最简洁且完整的规则应是：
```
自主回充执行阶段：
    屏蔽 /cmd_vel
实际充电阶段：
    屏蔽 /cmd_vel
回充失败或遥控器退出后：
    根据实时 RobotState 自动解除屏蔽
```

2. 请将 nav_bridge D1 MAX 的 L_WALK 步态映射替换为 @/home/jazzy/drive_ws/src/RobotSDK-0.2.1/docs/zh/sdk_client_api_zh.md 中 Gait 步态(这是一种步态类型，和 gait 的通用指向不同)。并且 D1 MAX 切换状态机(nav_bridge 自己的状态机)时需要打印输出。请评估。

```cpp
std::error_code Gait(int timeout_ms = 0,
                     WriteHandler handler = [](const std::error_code&, std::size_t) {})
```

3. 现在 nav_bridge 是否调用
```cpp
std::error_code Gait(int timeout_ms = 0,
                     WriteHandler handler = [](const std::error_code&, std::size_t) {})
``` 
后，再设置 slow 作为 L_WALK 步态？这样可能有些问题，因为 Gait 和通用模式的低、中、高其实是四种不同的模式，L_WALK 调用 GAIT 后不应该调用 SLOW 了。请评估。


4. d1 max 接收 /cmd_vel 和 x30 逻辑一致吗？ d1 max 接收 /cmd_vel 是否会产生延迟。


问题:
@/home/jazzy/drive_ws/src/RobotSDK-0.2.1/docs/zh/sdk_client_api_zh.md 中对于 
```cpp
std::error_code Move(float left_right, float forward_back, float yaw, 
                     int timeout_ms = 0,
                     WriteHandler handler = [](const std::error_code&, std::size_t) {})
```
使用百分比表示，那么 D1 MAX 中几个状态机 MOUNTAIN、L_WALK 是怎么进行缩放的或如何处理这个百分比问题的？

1. /cmd_vel 发出的应该是实际速度 m/s，所以应该根据其对应的速度等级进行相应的缩放，而不是使用 MOVE 发出百分比。比如其速度等级是 2，接收 /cmd_vel = 2 m/s 时，应当除 2 再发出。请客观评估。

2. 我刚才说错了， L_WALK→Gait() 的速度上限和 "通用模式+中速" 应当一致。现在在该步态下给予 1 m/s，可能会到 2 m/s。请修复。 

3. D1 MAX 的 MOUNTAIN 模式改为映射到 "通用模式+高速"。


问题:
1. D1 MAX 通过 nav_bridge 退出充电会出现 "实际已经退出充电，但是显示退出充电失败"，当前判断进入/退出充电成功是依赖什么，是单纯依靠固定时间判定吗？
2. D1 MAX 通过 nav_bridge 进行状态机切换，会出现状态机信息一直打印的情况，我只希望状态机在实际切换时打印
请先评估。
