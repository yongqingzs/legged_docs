# TF Relay Node — `tf_relay_node.py`

## 概述
`tf_relay_node.py` 实现了一个将静态外参（extrinsics）以“带传感器时间戳”的形式动态广播的 ROS 节点，主要目的是解决传感器数据与 TF 时间戳不一致导致的同步问题。节点通过读取已有 TF（例如 `odom -> odin1_base_link` 或 `map -> odom`）的最新时间戳，将该时间戳赋给要广播的静态变换，从而保证外参在时间轴上与传感器数据对齐。

## 主要功能要点
- 使用 `tf2_ros.Buffer` + `tf2_ros.TransformListener` 获取传感器相关 frame 的最新 transform（优先查询 `odom -> odin1_base_link`，失败则尝试 `map -> odom`）。
- 以被查询到的 transform 的 `header.stamp` 作为 `sensor_timestamp`，将该时间戳用于广播以下两个静态变换（带时间戳）：
  - `odin1_base_link` -> `lidar_link`
  - `lidar_link` -> `camera_link`
- 仅在 `sensor_timestamp` 比上一次广播的时间更大时才进行广播，避免重复时间戳导致的 `TF_REPEATED_DATA` 警告。
- 广播使用 `tf2_ros.TransformBroadcaster.sendTransform()`，消息类型为 `geometry_msgs/TransformStamped`。
- 运行频率：10 Hz（由 `rospy.Rate(10)` 控制）。

## 硬编码的外参与常量（代码中）
- `odin_to_lidar_trans` = [0.00347, 0.03447, 0.02174]
- `odin_to_lidar_rot` = [0, 0, 0, 1]  （四元数，identity）
- `lidar_to_camera_trans` = [0.04354, -0.01748, -0.02099]
- `lidar_to_camera_rot` = [-0.5063, 0.4978, -0.4978, 0.5063]  （qx, qy, qz, qw）
- `rate` = 10 Hz

注：`lidar_to_camera_*` 的注释来源于文件内的 `Tcl_0` 标注（O1-N090100030）。

## 关键函数说明
- `__init__()`：初始化 ROS 节点、TF 广播/监听器、默认外参、发布频率以及日志提示。
- `publish_tf_with_sensor_timestamp()`：主循环函数，负责查找参考 transform、取时间戳、判断时间是否推进、调用广播函数并按频率休眠。
- `publish_static_transform(parent_frame, child_frame, translation, rotation, timestamp)`：构造 `TransformStamped` 并通过 `sendTransform()` 广播。
- `main()`：创建 `TFRelayNode` 实例并开始广播循环。

## 依赖
- ROS（`rospy`）和 TF2：`tf2_ros`
- 消息类型：`geometry_msgs/TransformStamped`
- Python 库：`numpy`, `math`

## 使用方法
1. 将节点作为 ROS 节点运行：

```bash
rosrun yolo_ros tf_relay_node.py
```

或在 launch 文件中启动该节点。该节点应与包含 `odom`、`odin1_base_link`、`map` 等 frame 的 TF tree 一起运行，以便获取参考时间戳。

## 注意事项与建议改进
- 当前外参写在代码中（硬编码），建议改为从 YAML/参数服务器读取，以便在不同平台上复用而无需修改源码。
- 时间戳来源依赖 `odom -> odin1_base_link` 或 `map -> odom` 的 TF；确保这些 transform 存在且带有合理时间戳，若不存在节点会退回到 `rospy.Time.now()`。
- 若需要广播更多静态外参或不同 frames，可在 `publish_tf_with_sensor_timestamp()` 中加入相应调用或把外参配置化。
- 节点通过比较时间戳避免重复广播，如果需要更严格的去重或恢复策略，可扩展逻辑（例如允许一定的回退窗口）。

## 源代码位置
- 源文件：[ros_ws/src/Odin-Nav-Stack/ros_ws/src/yolo_ros/scripts/tf_relay_node.py](ros_ws/src/Odin-Nav-Stack/ros_ws/src/yolo_ros/scripts/tf_relay_node.py)

---
文档路径：`ros_ws/doc/tf_relay_node.md`。