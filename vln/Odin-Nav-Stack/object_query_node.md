# Object Query Node — `object_query_node.py`

## 概述
`object_query_node.py` 实现了一个面向交互的对象查询与基于检测结果的导航控制节点。它订阅来自 YOLO 节点的 2D/3D 检测结果与类别映射，提供：

- 面向终端的交互式查询（文本/可选语音）；
- 将查询结果以 RViz Marker 可视化；
- 将查询/导航命令解析为目标点并发布到 `/move_base_simple/goal`；
- 支持中/英文自然语言的简单解析（方向词、序号选择）；
- 可选的语音识别（Vosk + sounddevice），将语音命令转为文本并执行查询/导航。

源码位置： [ros_ws/src/Odin-Nav-Stack/ros_ws/src/yolo_ros/scripts/object_query_node.py](ros_ws/src/Odin-Nav-Stack/ros_ws/src/yolo_ros/scripts/object_query_node.py)

---

## 主要功能
- 订阅检测：`/yolo_detections_3d`（`vision_msgs/Detection3DArray`）与 `/yolo_detections`（`vision_msgs/Detection2DArray`，作为 3D 不可用时的回退）；
- 订阅类别映射：`/yolo_class_names`（`std_msgs/String`，JSON 映射）；
- 发布：
  - `/object_markers`（`visualization_msgs/MarkerArray`）——在 RViz 中标注检测物体位置与标签；
  - `/object_query_result`（`std_msgs/String`，JSON）——查询结果，方便其他节点订阅；
  - `/navigation_goal_marker`（`visualization_msgs/Marker`）——导航目标的可视化；
  - `/move_base_simple/goal`（`geometry_msgs/PoseStamped`）——向导航堆栈发送目标位姿；
- 在终端提供交互式界面：查询、导航命令、列出检测物体与类别、启/关语音模式；
- 命令解析：支持中英文方向（left/right/front/behind 与 中文左/右/前/后）、支持 #N、Nth、中文“第 N 个”序号选择（最多 1-5）；
- 目标点计算：支持基于相机坐标或机器人基座方向计算目标点（`calculate_target_position*`），并可将相机坐标通过 TF 转换到地图（map）坐标系。

---

## ROS 参数
- `~direction_distance` (float, default `1.0`)：偏移距离（m），用于在物体相对方向上计算目标点。
- `~camera_frame` (string, default `camera_link`)：相机 frame id。
- `~target_frame` (string, default `map`)：目标坐标系（用于发布目标与 TF 查询）。
- `~base_frame` (string, default `odin1_base_link`)：机器人 base frame。
- `~home_frame` (string, default `odom`)：home 目标的坐标系。
- `~home_x`, `~home_y`, `~home_yaw`：home 位置与朝向（默认坐标）。

语音相关（可选）：
- `~enable_voice` (bool, default `False`)：是否启用语音识别（需安装 Vosk + sounddevice 并提供模型）。
- `~voice_model_path`：Vosk 模型目录。
- `~voice_sample_rate`, `~voice_block_size`, `~voice_device`：音频采样配置。
- `~voice_interval_sec`, `~voice_end_silence`, `~voice_min_duration`, `~voice_max_duration`, `~voice_debounce`, `~voice_partial`：语音端点与节流参数。

---

## 订阅 / 发布的 topic
- 订阅：
  - `/yolo_detections_3d` (`vision_msgs/Detection3DArray`)
  - `/yolo_detections` (`vision_msgs/Detection2DArray`)
  - `/yolo_class_names` (`std_msgs/String`) — 类别映射 JSON
- 发布：
  - `/object_markers` (`visualization_msgs/MarkerArray`)
  - `/object_query_result` (`std_msgs/String`) — JSON 输出
  - `/navigation_goal_marker` (`visualization_msgs/Marker`)
  - `/move_base_simple/goal` (`geometry_msgs/PoseStamped`)

---

## 关键函数说明
- `class_names_callback(msg)`：加载并缓存类别映射（JSON 字符串）。
- `detections_callback(msg)` / `detections2d_callback(msg)`：缓存最新 3D/2D 检测结果，供查询/可视化使用。
- `find_object(object_name)`：按名称（支持部分匹配）查找当前缓存的 3D 检测，返回 `(class_id, class_name, x_map, y_map, z_map, score)` 列表。
- `visualize_objects(objects, object_name)`：对找到的对象在 RViz 中用线框立方体与文本标注绘制 Marker。
- `parse_navigation_command(command)`：解析自然语言导航命令（方向 + 可选序号），支持中文语句解析（如“走到第三个人的左边”）。
- `calculate_target_position*`：根据检测到的目标位置与方向计算偏移目标点（支持 camera-frame、map-frame 与基于 robot base 的偏移）。
- `transform_camera_to_map(camera_pos)`：使用 TF 将相机坐标点转换为地图坐标。
- `send_navigation_goal(target_pos_map)`：发布 `/move_base_simple/goal` 并在 RViz 显示箭头标记。
- `process_simple_command(text)`：处理“不依赖检测”的简单命令（例如回到原点）并发布 home 目标。
- `process_navigation_command(command)`：完整处理导航命令的流程：解析、查找目标、选择目标（最近或按序号）、计算偏移目标、发送导航目标。
- `query_object(object_name)`：针对某一类别查询并打印/发布结果（含中/英文映射、可信度、位置），并调用 `visualize_objects()`。
- `interactive_mode()`：命令行交互模式，支持文本指令及可选语音控制指令。
- 语音接口：`_start_voice_thread()`、`_stop_voice_thread()`、`_voice_loop()`、`_handle_voice_text()` — 当启用语音时捕获并识别语音，转换为文本后复用已有解析逻辑。

---

## 使用说明
1. 确保正在运行的 YOLO 节点会发布 `/yolo_detections_3d` 与 `/yolo_class_names`；没有 3D 时会回退到 `/yolo_detections`（仅供列表展示）。
2. 启动节点：

```bash
rosrun yolo_ros object_query_node.py
```

3. 交互式示例：
- 查询示例（文本）：
  - `person` 或 `人` → 列出检测到的“人”并在 RViz 中标注；
- 导航命令（英文）：
  - `Move to the right of the person` 或 `right person`；
  - `behind car 2nd` → 移动到第 2 个车的后面；
- 导航命令（中文）：
  - `走到第3个人的左边`、`运动到沙发的前边`；
- 简单命令：`home` / `回来` → 发布回家目标（`~home_frame`、`~home_x`, `~home_y`）。

4. 语音模式（可选）：如果启用 `~enable_voice=true` 并正确配置 Vosk 模型路径与音频设备，可在交互中使用 `voice on` 开启语音监听，节点会周期性识别并处理语音命令。

---

## 依赖与注意事项
- 依赖 ROS（`rospy`, `tf2_ros` 等）、`vision_msgs`, `visualization_msgs`；Python 库包括 `numpy`、`vosk`（可选）、`sounddevice`（可选）。
- 导航功能依赖有效的 TF（camera → map 与 base → map），请确保 TF tree 完整。
- 查询返回的位置信息依赖 `/yolo_detections_3d` 的质量；仅有 2D 检测时无法用于导航。
- 语音识别需安装 Vosk 模型并正确配置 `voice_device`，在 headless/无麦克风环境下该功能不可用。
- 类别映射以 `/yolo_class_names` 发布的 JSON 为准，脚本内包含丰富的中文/英文映射词典以提高匹配率。

---

文档文件： [ros_ws/doc/object_query_node.md](ros_ws/doc/object_query_node.md)
