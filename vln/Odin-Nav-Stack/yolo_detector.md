# YOLO Detector — `yolo_detector.py`

## 概述
该节点实现了基于 YOLOv5 的目标检测与简单的 3D 定位与避障控制：
- 订阅相机图像与深度图，使用 YOLOv5（模型文件 `models/yolov5s.pt`）进行目标检测；
- 将 2D 检测结果封装为 `vision_msgs/Detection2DArray` 并发布；
- 若可用深度图，使用 FishPoly 鱼眼相机模型将检测框中心点反投影为 3D（相机坐标），并发布 `vision_msgs/Detection3DArray`；
- 提供基于最近目标的简单避障控制（发布 `/cmd_vel`）；
- 发布带标注的调试图像用于可视化。

此脚本路径：[ros_ws/src/Odin-Nav-Stack/ros_ws/src/yolo_ros/scripts/yolo_detector.py](ros_ws/src/Odin-Nav-Stack/ros_ws/src/yolo_ros/scripts/yolo_detector.py)

## 依赖
- ROS（`rospy`）、`tf2_ros`、`cv_bridge`
- Python 包：`numpy`, `opencv-python`, `torch`（与 yolov5 依赖项），`message_filters`
- vision message types：`vision_msgs`（Detection2D/3D）
- yolov5 源码（项目内通过相对路径加载）：
  - 在脚本中将 `../../../../yolov5` 加入 `sys.path`，因此需要将 yolov5 放在相对位置或调整 `YOLOV5_PATH`。
  - 模型文件默认加载路径：`models/yolov5s.pt`（相对于脚本目录）。

## 节点功能要点
- 模型加载：使用 `attempt_load()` 加载 TorchScript/权重并自动选择设备（`select_device()`）。
- 输入预处理：`letterbox()` 做 resize + pad（保证 stride 对齐），并将图像归一化到 [0,1]。 
- 推理与 NMS：网络输出经 `non_max_suppression()` 去重；检测框按原始图像尺度缩放（`scale_boxes`）。
- 2D→3D：若收到深度图，取检测框中心像素的深度，通过 `unproject_fisheye()`（FishPoly 模型）计算相机坐标系下 (X,Y,Z)，并填充 `Detection3D`。
- TF 变换：`transform_to_target_frame()` 将相机坐标点变换到目标坐标系（默认 `map`），可选使用 `rospy.Time(0)` 获取最新 TF。
- 避障控制：`simple_avoidance_control()` 根据最近检测目标的 (x,z) 发布线速度/角速度到 `/cmd_vel`。

## ROS 参数（节点私有参数，默认值）
- `~rgb_topic` : `/odin1/image/compressed` — RGB 输入（支持 `sensor_msgs/Image` 或 `sensor_msgs/CompressedImage`）
- `~depth_topic` : `/odin1/depth_img_competetion` — 深度图（`sensor_msgs/Image`）
- `~auto_nav_mode` : `False` — 是否自动启用避障控制
- `~camera_frame` : `camera_link` — 相机 frame id
- `~target_frame` : `map` — 目标输出坐标系（用于 TF 变换）
- `~use_latest_tf` : `True` — 是否使用最新 TF（`rospy.Time(0)`）以避免时间戳不匹配
- `~depth_mode` : `z` — 深度模式，`z`（深度为光轴方向的 z）或 `range`（深度为射线距离）

## 订阅 / 发布的 topic
- 订阅：
  - RGB 图像：`~rgb_topic`（`sensor_msgs/Image` 或 `sensor_msgs/CompressedImage`）
  - 深度图：`~depth_topic`（`sensor_msgs/Image`，支持 `32FC1`、`16UC1` 等）
- 发布：
  - `/yolo_detections`（`vision_msgs/Detection2DArray`）
  - `/yolo_detections_3d`（`vision_msgs/Detection3DArray`）
  - `/yolo_debug_image`（标注后的 `sensor_msgs/Image`）
  - `/cmd_vel`（`geometry_msgs/Twist`） — 简单避障命令
  - `/yolo_class_names`（`std_msgs/String`，latched） — 类名映射（JSON 字符串）

## 关键函数说明
- `__init__()`：初始化模型、参数、TF 监听、订阅/发布器、相机内参与外参（`Tcl`）。
- `letterbox(img, new_shape, stride)`：按 yolov5 要求 resize + pad。
- `image_callback(msg)`：主回调，完成预处理、推理、NMS、2D/3D 构建、发布调试图像与检测消息，并在 `auto_nav_mode` 打开时调用避障。
- `depth_callback(msg)`：接收并缓存深度图，统一转换为 `float32`（单位 m）。
- `unproject_fisheye(u, v, depth)`：FishPoly 模型将像素坐标与深度反投影为相机坐标系 3D 点。
- `transform_to_target_frame(x,y,z,timestamp)`：使用 TF2 将相机坐标点转换到 `target_frame`。
- `simple_avoidance_control(det3d_msg)`：基于最近目标的 3D 位姿生成差动式速度指令并发布到 `/cmd_vel`。

## 使用与部署建议
1. 放置 yolov5 代码与权重：确保 `YOLOV5_PATH` 指向有效的 yolov5 源码目录，并将 `models/yolov5s.pt` 放到脚本相对的 `models/` 中，或修改 `attempt_load()` 路径。 
2. 安装依赖：

```bash
pip install torch torchvision opencv-python numpy pillow
# ROS 侧确保安装 cv_bridge、vision_msgs 等
```

3. 启动节点（ROS 1）：

```bash
rosrun yolo_ros yolo_detector.py
# 或将节点加入 launch 文件中启动
```

4. 调试：订阅 `/yolo_debug_image` 查看标注图，订阅 `/yolo_class_names` 获取类别映射。

## 注意事项
- 相机为鱼眼（FishPoly）模型，反投影在大入射角下可能失败（函数会 return None）；请注意 `maxIncidentAngle` 参数。 
- 深度质量直接影响 3D 结果；若深度图与 RGB 同步存在问题，请调整 TF 与时间戳或使用 `use_latest_tf=True`。
- 本脚本使用 `rospy`（ROS 1）接口，确认运行环境为 ROS 1（或在 ROS2 中使用 ros1_bridge）。

---
文档由脚本自动生成，文件位置：`ros_ws/doc/yolo_detector.md`。