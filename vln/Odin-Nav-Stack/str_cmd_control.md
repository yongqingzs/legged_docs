# STR Command Controller — `str_cmd_control.py`

## 概述
`str_cmd_control.py` 实现了一个轻量级的字符串命令控制节点，用于通过简单的文本指令控制机器人发送局部导航目标或短时速度命令。该节点监听 `/cmd_str`（`std_msgs/String`），并把解析后的动作转换为：

- 发布到 `/move_base_simple/goal`（`geometry_msgs/PoseStamped`）作为局部目标；
- 或短时间发布 `/cmd_vel`（`geometry_msgs/Twist`）用于旋转控制；

节点名：`str_cmd_controller`。

## 功能要点
- 订阅 `/cmd_str`（`std_msgs/String`），接收单字串命令。
- 支持命令：
  - `stop`：停止（发布零速度）并发送零位移目标；
  - `forward`：发布局部前进目标（x 偏移 0.5 m，frame 为 `base_link`）；
  - `left`：向左原地旋转 30°；
  - `right`：向右原地旋转 30°；
- 旋转通过发布 `/cmd_vel` 的 `angular.z` 实现，旋转速率由常量 `ANGULAR_SPEED_DEG`（默认 90°/s）控制。
- 命令由后台线程 `command_worker()` 异步处理，使用锁保护命令变量。

## ROS Topic（订阅 / 发布）
- 订阅：
  - `/cmd_str` (`std_msgs/String`) — 字符串命令输入。
- 发布：
  - `/move_base_simple/goal` (`geometry_msgs/PoseStamped`) — 局部目标（默认 `frame_id` 为 `base_link`）。
  - `/cmd_vel` (`geometry_msgs/Twist`) — 旋转速度控制（短时脉冲）。

## 配置常量
- `TOPIC_CMD_STR = "/cmd_str"`
- `TOPIC_CMD_VEL = "/cmd_vel"`
- `TOPIC_GOAL = "/move_base_simple/goal"`
- `GOAL_FRAME_ID = "base_link"` — PoseStamped 的 `header.frame_id`（可根据导航栈期望修改）。
- `ANGULAR_SPEED_DEG = 90.0` — 角速度（度/秒），用于计算旋转所需时间与发布角速度（转换为弧度/秒）。

## 使用示例
- 启动节点（在已 source ROS 环境下）：

```bash
# 方式 1: 使用 rosrun（若已打包为 ROS 包）
rosrun <your_package> str_cmd_control.py

# 方式 2: 直接运行脚本（需在 ROS 环境中）
python3 /path/to/ros_ws/src/Odin-Nav-Stack/scripts/str_cmd_control.py
```

- 发送命令示例：

```bash
# 发送一次性命令
rostopic pub -1 /cmd_str std_msgs/String "data: 'forward'"
rostopic pub -1 /cmd_str std_msgs/String "data: 'left'"
rostopic pub -1 /cmd_str std_msgs/String "data: 'right'"
rostopic pub -1 /cmd_str std_msgs/String "data: 'stop'"
```

## 实现细节与注意事项
- `publish_goal(x_offset)` 会发布一个 `PoseStamped`，`header.frame_id` 使用 `GOAL_FRAME_ID`。默认 `base_link` 意味着目标是相对于机器人基座的局部目标；如果你的导航栈期望接收 `map` 或 `odom` 下的目标，请调整 `GOAL_FRAME_ID` 或在外层将目标转换到正确坐标系。
- `publish_rotation(angle_degrees)`：先发布一个零位移目标（`publish_goal(0.0)`），短暂停顿后计算需要旋转的时间 `t_need = abs(angle)/ANGULAR_SPEED_DEG`，然后以常量角速度循环发布 `Twist`，直至时间到，最后发布零速度并短暂停顿。
- 该节点只实现了非常基础的命令集合，适合作为远程控制或上层脚本触发器；在部署到真实机器人/导航栈时，请评估对现有控制器（例如 move_base）指令的冲突风险。

## 文件位置
- 源文件：`ros_ws/src/Odin-Nav-Stack/scripts/str_cmd_control.py`
- 文档：`ros_ws/doc/str_cmd_control.md`

---
文档已生成并保存到 `ros_ws/doc/str_cmd_control.md`。