## task1
分别梳理 rl_real_a1，rl_sdk，observation_buffer 各模块的功能，说明:
1. 各模块相关数据如何传输和交互(可以用 Mermaid 表示，但也需用文字表述)
2. 各模块的各函数功能
3. 你觉得应当说明的内容
将各模块的说明分别输出到 docs/ 下

## task2
请参照 a1 相关模块，完成 go1 相关模块的编写，注意:
1. go1 和 a1 的硬件接口一致

## task3
rl_real_go1 原先和硬件耦合的接口使用 unitree_legged_sdk，现改为使用 free_dog_sdk_cpp，请完成相关接口的修改，注意:
1. 请参照 unitree_legged_sdk 和 free_dog_sdk_cpp 完成修改
2. 修改完成后输出相关说明文档至 docs/ 下

## task4
梳理 actuator_net 模块的功能，说明:
1. 该模块相关数据如何传输和交互(可以用 Mermaid 表示，但也需用文字表述)
2. 该模块的各函数功能
3. 你觉得应当说明的内容
将模块的说明输出到 docs/ 下

## task5
gazebo.launch.py 是针对 humble 版本编写的，请创建一个针对 jazzy 版本的 launch 文件，注意:
1. jazzy 版本和 humble 版本的差异

## task6
rl_real_go1 的对外消息交互接口参照 hardware_free_dog_sdk 设计，现在参考 hardware_unitree_ros2 需要增加对 unitree_ros2 消息的支持，请完成该任务。注意:
1. 在原 rl_real_go1 上增加支持或创建新的 rl_real_go1_unitree_ros2 模块均可，请选定合适方案
2. 在一次启动中只允许一种消息交互，该 flag 可通过 yaml 配置文件指定
3. 需要继续支持 control_input 消息类型
4. 不能破坏原有的消息交互方式

## task7
你刚才进行了硬件接口抽象，但源码中存在大量 fdsc 残余，请完成修复，并且将 hardware_interface 相关转移至 core/hardware_interface/ 下，说明:
1. 注意 unitree_ros2 的位置
2. 编译指令 "cd ~/rl_sar && ./build.sh" 请通过编译