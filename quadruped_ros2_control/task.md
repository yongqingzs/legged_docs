## tips
1. 使用中文完成文档。
2. 使用中文和我对话。
3. 不要进行编译(colcon build)。

## task1
详细阅读 quadruped_ros2_control 项目并进行分析，完成一个总体的分析文档，包含以下内容:
1. gazebo 模拟下数据如何在各个节点(需指出接口)传递。
2. 实际硬件下数据如何在各个节点(需指出接口)传递。
3. 说明每个（mpc/rl/pd）控制器数据交互的流程（例如：指出指令/期望轨迹->MPC->WBC是如何交互和数据传递的）。给出具体公式和流程示例。
4. 将整理好的内容输出到 docs 下的一个新创建的文档中。

## task2
分析并梳理 ocs2_quadruped_controller 包中的代码，完成以下任务:
1. 说明其实现的功能和各个节点间的关系(关系可以用 Mermaid 表示，但也需用文字表述节点的关系)。
2. 说明各个节点中各个方法的功能。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs 下的 ocs2_quadruped_controller.md 中。

## task3
分析并梳理 rl_quadruped_controller 包中的代码，完成以下任务:
1. 说明其实现的功能和各个节点间的关系(关系可以用 Mermaid 表示，但也需用文字表述节点的关系)。
2. 说明各个节点中各个方法的功能。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs 下的 rl_quadruped_controller.md 中。

## task4
分析并梳理 ocs2_quadruped_controller 包中的代码，完成以下任务:
1. 说明"步态(gait)"是如何在代码里体现，gait 切换又会如何体现。
2. 将整理好的内容输出到 docs 下的 ocs2_quadruped_controller.md 中（新增相关内容即可）。

## task5
分析并梳理 estimator 中的代码，完成以下任务:
1. 说明其实现的功能和各个节点间的关系(关系可以用 Mermaid 表示，但也需用文字表述节点的关系)。
2. 说明各个节点中各个方法的功能。
3. 着重分析 KalmanFilterEstimate 类的内容，并说明其输入输出具体是什么，输入从哪里获取。
4. 说明你觉得应当说明的内容。
5. 将整理好的内容输出到 docs 下的 estimator.md 中。

## task6
分析并梳理 Ocs2QuadrupedController 类中，完成以下任务:
1. 说明该类各个方法的功能(需要说明各个方法什么时候被调用，调用顺序如何。调用顺序和时间点可以用 Mermaid 表示，但也需用文字表述)。
2. 说明如 command_interfaces_、state_interfaces_ 等接口项从哪里读入，其如何更新。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs 下的 Ocs2QuadrupedController.md 中。

## task7
有两个启动命令:
```bash
# gazebo
ros2 launch ocs2_quadruped_controller gazebo.launch.py pkg_description:=go2_description

# mujoco
ros2 launch ocs2_quadruped_controller mujoco.launch.py pkg_description:=go2_description
```
分析这两个启动命令涉及到的配置文件，完成以下任务:
1. 忽略 .jpg、.dae 等非文本类型文件。分析 launch.py 如何加载各项配置文件(包括各配置的嵌套关系)。
2. 分析其如何配置 ros2_control。
3. mujoco.launch.py 是否可用于真实的硬件，用于真实的硬件需要注意什么。
4. 说明你觉得应当说明的内容。
5. 将整理好的内容输出到 docs 下的 launch_py.md 中。

## task8
分析并梳理 StateOCS2 类中的代码(会涉及其他类)，完成以下任务:
1. 说明该类各个方法的功能(尤其是 run 方法)。
2. 需要分析 evaluatePolicy、 updatePolicy 中 Policy 具体是什么(一个函数、一个变量还是类似神经网络，其接收什么输入，输出什么)。
3. 需要分析"传感器->mpc->wbc->机器狗"每一次数据传输、处理过程中的输入输出具体是哪些。
4. 说明你觉得应当说明的内容。
5. 将整理好的内容输出到 docs 下的 StateOCS2.md 中。

## task9
分析并梳理 CtrlComponent 类中的代码(会涉及其他类)，完成以下任务:
1. 说明该类各个方法的功能(尤其是 updateState 方法)。
2. 需要分析 updateState 过程中发生的数据转换，如: measured_rbd_state_ 如何转换为 observation_.state ， observation_.state 具体包括哪些量（物理意义以及所在数组位数）。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs 下的 CtrlComponent.md 中。

## task10
分析并梳理 TargetManager 类中的代码，完成以下任务:
1. 说明该类各个方法的功能(尤其是 update 方法)。
2. 需要分析 update 过程中发生的几次数据转换（物理意义以及所在数组位数）。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs 下的 TargetManager.md 中。

## task11
分析并梳理 SwitchedModelReferenceManager 类及其父类 ReferenceManager 中的代码，完成以下任务：
1. 说明类的各个方法的功能(尤其是 "涉及接受外部数据和更新对外数据" 方法)。
2. 需要分析 "涉及接受外部数据和更新对外数据" 方法执行过程中发生的数据转换（物理意义以及所在数组位数）。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs/class 下的 SwitchedModelReferenceManager.md 中。

## task12
分析并梳理 LeggedInterface 类中的代码，完成以下任务：
1. 说明类的各个方法的功能(重点关注其数据处理和更新的流程)。
2. 需要总结 LeggedInterface 控制的组件名单。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs/class 下的 LeggedInterface.md 中。

## task13
分析并梳理 GaitManager 类中的代码，完成以下任务：
1. 说明类的各个方法的功能(重点关注"涉及接受外部数据和更新对外数据"的流程)。
2. 说明你觉得应当说明的内容。
3. 将整理好的内容输出到 docs/class 下的 GaitManager.md 中。

## task14
分析并梳理 unitree_sdk2 包中的代码，完成以下任务：
1. 可以基于 go2 分析以下内容。
2. 说明该包实现的功能，其数据处理和更新的流程。
3. 说明该包如何与 硬件/mujoco 交互，如何调用该包获取数据。
4. 说明该包是否进行了代码封装(即隐藏具体实现)。如果有的话，如何进行替代。
5. 说明你觉得应当说明的内容。
6. 将整理好的内容输出到 docs/external 下的 unitree_sdk2.md 中。

## task15
分析并梳理 unitree_mujoco 包中的代码，完成以下任务：
1. 可以基于 go2 分析以下内容。
2. 说明该包实现的功能，其数据处理和更新的主要流程。
3. 说明该包如何与 unitree_sdk2 交互。
4. 说明你觉得应当说明的内容。
5. 将整理好的内容输出到 docs/external 下的 unitree_mujoco.md 中。

## task16
分析并梳理 hardware_unitree_sdk2 包中的代码，完成以下任务：
1. 说明该包实现的功能，其数据处理和更新的主要流程。
2. 着重并详细说明该包如何与 ros2_control、unitree_sdk2 、unitree_mujoco 进行数据交互(需要说明数据的具体类型，数组位数含义)。它们是什么关系(关系可以用 Mermaid 表示，但也需用文字表述节点的关系)。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 docs 下的 hardware_unitree_sdk2.md 中。

## task17
对比 hardware_unitree_sdk2 包和 unitree_ros2 包。不要摘录过多源码，使用叙述性文字和 Mermaid 图。完成以下任务：
1. 说明这两个包之间的关系。hardware_unitree_sdk2 是基于 unitree_ros2 完成的吗？如果是的话，它添加了什么工作？
2. 说明你觉得应当说明的内容。
3. 将整理好的内容添加到 hardware_unitree_sdk2.md 中(不要修改现有 md 内容，只需要新增)。

## task18
分析并梳理 UnitreeSdk2Bridge 类中的代码。不要摘录过多源码，完成以下任务：
1. 说明该类各个方法的功能。
2. 着重说明其数据处理和更新的主要流程(其如何与外部交互，比如mujoco或其他)。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容添加到 unitree_mujoco.md 中(不要修改现有 md 内容，只需要新增)。

## task19
分析并梳理 perceptive 目录中的代码。不要摘录过多源码，完成以下任务：
1. 说明该目录下代码实现的功能，其数据处理和更新的主要流程。
2. 说明各类中各个方法的功能。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 perceptive.md 中。

## task20
分析并梳理 unitree_sdk2 包中的代码。不要摘录过多源码，完成以下任务：
1. 分析 unitree_sdk2/include/unitree 实现的功能，并着重说明其关键类和方法
2. 分析以下例程，包括其实现的功能和使用说明
- unitree_sdk2/example/go2、 
- unitree_sdk2/example/jsonize
- unitree_sdk2/example/state_machine
- unitree_sdk2/example/wireless_controller
3. 依据你的分析，回答以下问题
- unitree_sdk2 只是对 dds 进行了封装，提供类似发布者订阅者的接口？
- unitree_sdk2 为什么要对 dds 进行了封装？
- unitree_sdk2 是否支持不同的 dds(如: fast dds、cyclone dds)？
4. 说明你觉得应当说明的内容。
5. 将整理好的内容添加到 unitree_sdk2.md 中(不要修改现有 md 内容，只需要新增)。

## task21
分析并梳理 wbc 目录中的代码。不要摘录过多源码，完成以下任务：
1. 说明该目录下代码实现的功能，其数据处理和更新的主要流程。
- 说明涉及每次处理前和处理后数据的具体形式，类似:
```txt
估计的 base 姿态(欧拉角): 3  转换测量值zyx
估计的 base 位置:  3  xHat_
关节角度:  12  测量值joint_pos
估计的 base 全局角速度:  3  转换测量值angularVel
估计的 base 线速度:  3  xHat_
关节角速度:  12  测量值joint_vel
```
- 说明哪个类的哪个方法参与了数据处理
- 说明足端力传感器起到什么作用，是否参与计算，是否必需
2. 说明各类中各个方法的功能。需要着重说明 WeightedWbc 类。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 wbc.md 中(不要新创建 wbc.md，输出到现有的 wbc.md )。

## task22
分析并梳理 SwingTrajectoryPlanner 类中的代码。不要摘录过多源码，完成以下任务：
1. 说明其数据处理和更新的主要流程。并绘制流程图(可以用 Mermaid 表示，但也需用文字表述流程)。
2. 说明数据的输入输出分别和什么模块交互。
3. 说明类中各个方法的功能。
4. 说明该模块的作用，和代表的机理。
5. 说明你觉得应当说明的内容。
6. 将整理好的内容输出到 SwingTrajectoryPlanner.md 中(不要新创建 SwingTrajectoryPlanner.md，输出到现有的 SwingTrajectoryPlanner.md )。

## task23
分析 task.info。完成以下任务：
1. 说明各参数被哪个类最终使用了(如果有多个类使用，也请说明)
- 提示: task.info 在代码里会被类似"taskfile"等加载
- 指出各参数作用
2. 说明各参数的物理意义(针对算法相关参数，逐一说明)。
3. 说明调参建议(针对算法相关参数，逐一说明)。
4. 说明你觉得应当说明的内容。
5. 将整理好的内容输出到 docs/task_info.md 中。

## task24
分析 unitree_guide_controller/Estimator。完成以下任务：
1. 说明该类各个方法的功能(着重说明其数据处理和更新的主要流程)。
2. 说明其与 ocs2_quadruped_controller/LinearKalmanFilter 的区别。
- ocs2_quadruped_controller/LinearKalmanFilter 是否可替换为 unitree_guide_controller/Estimator。如果可以，给出对接流程。
- ocs2_quadruped_controller/LinearKalmanFilter 中需要用到足端接触力的数据，是否有方式替代足端接触力的使用。
3. 说明你觉得应当说明的内容。
4. 将整理好的内容输出到 unitree_guide/class/Estimator.md 中。

## task25
hardware_unitree_sdk2 适配 unitree_sdk2 (即底层经过 dds 消息)。现在想实现 hardware_unitree_ros2 适配 unitree_ros2( 硬件消息也通过 ros2 消息发出 )。请参照hardware_unitree_sdk2、unitree_ros2 实现 hardware_unitree_ros2。
- ros2 消息类型参照 unitree_ros2/unitree_go
- 实现和 hardware_unitree_sdk2 类似的功能

## task26
unitree_mujoco/unitree_sdk2_bridge 实现了 mujoco 和 unitree_sdk2 的桥接。现在需要实现 unitree_mujoco/unitree_ros2_bridge，将 mujoco 和 hardware_unitree_ros2 进行桥接。
1. ros2 消息类型参照 hardware_unitree_ros2 的实现
2. 同时补充 hardware_unitree_ros2 的实现，添加:
- ROS2 publishers/subscribers的实际实现
- 消息桥接组件
- 安全和错误处理机制
3. 去除 no_foot.cc，因为其只实现了去除足端力传感器的功能，在 unitree_sdk2_bridge 添加相应的 if-else 处理即可(可从 config.yaml 中读取 no_foot 选项)

## task27
在 src 下完成一个简单例程。实现 dds 发布者和 ros2 订阅者消息交互的演示。
1. 消息类型由你定义。dds idl 消息或可由 ros2 msg 生成，具体看你决定。
2. 实现 dds 发布者，定期发布消息。
3. 实现 ros2 订阅者，接收 dds 发布的原生消息并打印。

## task28
hardware_unitree_ros2 目前无法直接接收 unitree_mujoco 发出的 dds 消息(到时候硬件会直接发出 ros2 消息)。需要在 hardware_unitree_ros2 实现一个单独的节点，使用类似 hardware_unitree_sdk2 的接收 dds 消息的方式，并将其转变为 hardware_unitree_ros2 需要的 ros2 消息发出。
- 该节点能类似 hardware_unitree_sdk2，接收 dds 消息，并转发 ros2 消息
- 该节点能接收 hardware_unitree_ros2 发出的 ros2 消息，并转发为 dds 消息
- 需要在 hardware_unitree_ros2 添加 crc 等类似处理

## task29
在 dds_ros2_bridge_node 节点中添加新功能：
1. log_flag = 1 时，接收 "control_input_msgs/msg/inputs.hpp" 消息 
2. 当 inputs_.command 发生变化后，记录 10s 的 low_state 和 low_cmd 到 csv 文件中
- csv 文件需要说明 command 切换
3. 提供绘制 low_state 和 low_cmd 的 python 脚本

## task30
修改调用 python3 脚本绘图的逻辑:
1. 先索引包位置( 该脚本在 hardware_unitree_ros2/scripts 下)，索引到位置进行调用
2. 检测 ～/venvs 下是否存在 motor 环境，如果存在则 activate motor 环境后再调用脚本
如果不存在则先在 ～/venvs 下创建 motor 环境，并安装 numpy、matplotlib、pandas 包，然后 activate motor 环境后再调用脚本
3. 以上逻辑写成单独的函数进行调用

## task31
根据 HardwareRos2Bridge 节点，实现一个数据记录和可视化的节点。要求如下:
1. 功能逻辑按照 HardwareRos2Bridge，但不处理 dds 消息，只接收 low_cmd、low_state 消息，并进行相应的数据处理;

## task32
分析 go1.xml 与读取文件相关代码，说明以下内容，并将说明内容写进 go1_xml.md:
1. (关键)各电机的初始位置和位置上下界如何定义，坐标系如何定义
2. imu的初始位置如何定义，坐标系如何定义
3. 如何实现仿真和实际机器狗传感器关系的映射
@unitree_mujoco/unitree_robots/go1/go1.xml @unitree_mujoco/simulate/ @legged_docs/quadruped_ros2_control/external/go1_xml.md
