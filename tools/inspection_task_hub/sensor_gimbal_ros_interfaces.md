# sensor_gimbal ROS 2 控制接口统计

统计依据：`/home/jazzy/task_ws/src/sensor_gimbal` 当前 `main` 源码，2026-09-28。下文的名称是默认名称，可被参数或 ROS remap 修改。`sensor_gimbal_node` 在 `enable_gimbal=true` 时创建云台节点 `gimbal_stub_node`；`sensor_gimbal.launch.py` 默认启用云台和热成像、关闭烟雾检测。`inspection_with_hardware.launch.py` 则启用云台、关闭热成像和烟雾检测。

## 云台本体：可调用入口

共 **3 个 action、5 个命令 topic、2 个查询 service**。Action 适合需要执行结果或取消的任务；手动 topic 是异步命令。

| 类别 | 默认名称 | 类型 | 可以控制的动作 / 关键输入 |
| --- | --- | --- | --- |
| Action | `follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 转到指定水平/俯仰角；也支持目标坐标解算、变倍、按元数据启用 YOLO 居中闭环。`post_task_home` 分支当前有实现缺口，见下文。 |
| Action | `capture_media` | `inspection_interfaces/action/CaptureMedia` | `mode=photo` 拍照/连拍；`mode=video` 限时录像。可选可见光、红外、双通道或 ROS 深度图像；可指定照片数量/间隔、录像秒数和 `zoom_level`。 |
| Action | `center_gimbal_and_capture` | `inspection_interfaces/action/CenterGimbalAndCapture` | 先归位云台，再设置变倍并拍照/连拍；可指定归位/抓拍超时。这里的 `center` 指归位，**此 action 不执行 YOLO 目标居中**。 |
| Topic 订阅 | `/gimbal/manual_move` | `std_msgs/msg/String` | `left`、`right`、`up`、`down`，每次相对转动；步长由 `gimbal_manual_move_step_deg` 控制，默认 3 度。 |
| Topic 订阅 | `/gimbal/home` | `std_msgs/msg/Bool` | `data: true` 触发归位；`false` 被忽略。 |
| Topic 订阅 | `/gimbal/zoom` | `std_msgs/msg/String` | `in`/`up` 放大，`out`/`down` 缩小；步长由 `gimbal_zoom_step` 控制，默认 1，范围默认 1～32。 |
| Topic 订阅 | `/gimbal/focus` | `std_msgs/msg/String` | `near`/`far` 手动对焦脉冲；`semi_auto` 配合 `distance_m` 设置半自动对焦距离。也接受源码中定义的方向别名。 |
| Topic 订阅 | `/gimbal/reboot` | `std_msgs/msg/Bool` | `data: true` 重启海康设备并等待重新登录；`false` 被忽略。设备重启会短暂中断其他操作。 |
| Service | `/gimbal/query_pose` | `std_srvs/srv/Trigger` | 查询设备当前 `pan_deg`、`tilt_deg` 和 `zoom`；结果在响应 `message` 字符串中。 |
| Service | `/gimbal/query_zoom` | `std_srvs/srv/Trigger` | 查询当前 `zoom`，响应也附带水平/俯仰角；结果在 `message` 字符串中。 |

手动移动、变倍、对焦 topic 支持纯命令字符串，也支持放在字符串里的 JSON：`{"command":"left","tid":"...","bid":"..."}`。`tid`、`bid` 会回传至 `/gimbal/manual_event`；半自动对焦需要 `{"command":"semi_auto","distance_m":1.5}`。五个命令 topic 均依赖海康后端及设备就绪，查询 service 同样只支持 `camera_backend=gimbal_hk`。

### 三个 action 的实际语义

- `follow_joint_trajectory` 的常规角度目标要求 `joint_names: [pan_joint, tilt_joint]`，第一轨迹点有两个 `positions`，顺序为水平、俯仰。`joint_positions_use_radians=true` 时输入是弧度；`sensor_gimbal.launch.py` 所用 `stub_params.yaml` 将其设为 `false`，即输入按度解释；`inspection_with_hardware.launch.py` 显式覆盖为 `true`。`time_from_start` 被用作本次请求超时，非轨迹插值时长。`trajectory.header.frame_id` 可包含分号分隔的 `command_type=angle`、`calibration_type=instrument`、`zoom_level=...` 等元数据；`instrument` 才触发 YOLO/RKNN 目标居中。源码还接受 6 维旧式目标坐标和 13 维任务中枢坐标格式，不能把它们当成普通双关节轨迹。
- `calibration_hint=post_task_home` 会进入 `gimbal_stub_hk::request_home()`；该函数当前只是占位实现，固定返回 `Skipped`，因此对应 `follow_joint_trajectory` goal 会失败，**不能作为可用的归位接口**。需要归位时可用 `/gimbal/home`，它调用云台节点的实际归位逻辑；但 topic 没有 action 式完成结果。
- `capture_media` 的照片请求至少设置 `mode: photo`、`photo_count >= 1`；录像至少设置 `mode: video`、`recording_seconds >= 1`。`camera_type` 可用 `visible`、`thermal`/`ir`、`both`、`depth`/`realsense`/`depth_rgb`。红外照片通过同进程热成像节点的快照 service 完成，要求热成像节点已启动监控；`both` 只在海康后端拍照可用。海康录像默认保存原始 `.hikstream.dat`；配置可选择转 MP4。成功后可能按 `gimbal_home_after_capture_success` 自动归位，当前 `stub_params.yaml` 为 `true`。`CaptureMedia.action` 虽声明了 `start_recording`、`focal_length`，但当前处理代码未读取这两个字段，也不支持文中注释所称的 0 秒持续录像/另发停止命令。
- `center_gimbal_and_capture` 至少设置 `photo_count >= 1`；`home_timeout_sec`、`capture_timeout_sec`、`photo_interval_seconds` 不可为负。它先执行 home，再按 `camera_type` 抓拍，结果返回最后一张图片 URI 和实际数量。与 `capture_media` 不同，它不调用热成像快照 service，也没有 `both` 双通道组合逻辑；`camera_type=thermal`/`ir` 走海康红外通道 JPEG 抓拍。

### 反馈与输入

| 方向 | 默认名称 | 类型 | 含义 |
| --- | --- | --- | --- |
| 发布 | `/gimbal/manual_event` | `std_msgs/msg/String` | 手动移动、变倍、对焦的 JSON 结果；归位 topic 当前不发此事件。 |
| 发布 | `/gimbal/reboot_event` | `std_msgs/msg/String` | 重启和重连状态 JSON，`state` 为 `complete` 或 `error`。 |
| 发布 | `/centering_feedback` | `std_msgs/msg/Float64MultiArray` | YOLO 居中过程：`[pitch_delta_deg, yaw_delta_deg, within_tol, detection_count, consecutive_miss, frame_index]`。 |
| 订阅 | `/odometry_multi_maps` | `nav_msgs/msg/Odometry` | 姿态补偿的输入；`gimbal_pose_compensation_enabled` 默认关闭，不是控制命令。 |

三个 action 自带 ROS action 的 goal/result/cancel 通道；`CaptureMedia` 和 `CenterGimbalAndCapture` 还提供 `state`、`progress` feedback。云台本体**没有**公开 `SetGimbalAngles` ROS service，源码中的同名结构只供进程内部使用；也没有单独的拍照、录像命令 topic。

## 同一进程内的可选节点

这些是 `sensor_gimbal_node` 可创建的其他 ROS 节点，**不计入上面的云台本体 3/5/2**。

| 节点 / 启用开关 | 可调用接口 | 输出 |
| --- | --- | --- |
| `hik_alarm_node` / `enable_thermal` | `/monitor/thermal_camera/start`、`stop`、`test_alarm`：`std_srvs/srv/Trigger`；`/monitor/thermal_camera/set_parameters`：`rcl_interfaces/srv/SetParameters`，仅支持 `alarm_threshold_c`；`/monitor/thermal_camera/capture_snapshot`：`inspection_interfaces/srv/CaptureThermalSnapshot`，按需保存带温度标注快照。 | `/monitor/thermal_camera/status` (`diagnostic_msgs/msg/DiagnosticStatus`)、`/monitor/thermal_camera/heatmap` (`sensor_msgs/msg/Image`)。创建节点并不自动开始测温；需调用 `start`。 |
| `hik_smoke_alarm_node` / `enable_smoke` | `/monitor/smoke/start`、`stop`、`reset`、`test_alarm`：均为 `std_srvs/srv/Trigger`。 | `/monitor/smoke/status` (`diagnostic_msgs/msg/DiagnosticStatus`)。 |
| 独立进程 `post_waypoint_home_bridge` / `launch_post_waypoint_home_bridge` | 订阅 `/event_log` (`std_msgs/msg/String`)，在一航点全部任务完成后向 `follow_joint_trajectory` 发送 `post_task_home` goal。 | 本身没有云台命令 topic；它是 action 客户端。`inspection_with_hardware.launch.py` 默认启动此桥接，`sensor_gimbal.launch.py` 默认不启动；当前 goal 会因上述占位实现而失败。 |

## 直接调用示例

以下命令只展示接口格式，运行前应检查 `ros2 node list`、`ros2 action list`、`ros2 topic list`、`ros2 service list`，确认实际 remap、相机后端及设备状态。执行后会真的移动、拍摄或重启设备。

```bash
# 相对左转一步；反馈见 /gimbal/manual_event
ros2 topic pub --once /gimbal/manual_move std_msgs/msg/String '{data: "left"}'

# 放大一步、半自动对焦、查询姿态
ros2 topic pub --once /gimbal/zoom std_msgs/msg/String '{data: "in"}'
ros2 topic pub --once /gimbal/focus std_msgs/msg/String '{data: "{\"command\":\"semi_auto\",\"distance_m\":1.5}"}'
ros2 service call /gimbal/query_pose std_srvs/srv/Trigger '{}'

# 可见光单张拍照；录像 10 秒
ros2 action send_goal /capture_media inspection_interfaces/action/CaptureMedia '{task_id: 1, mode: photo, camera_type: visible, photo_count: 1, photo_interval_seconds: 0.0, zoom_level: 1.0}'
ros2 action send_goal /capture_media inspection_interfaces/action/CaptureMedia '{task_id: 2, mode: video, camera_type: visible, recording_seconds: 10}'
```

### 转到指定角度

以下两条命令的目标相同：水平 `pan_joint=30°`、俯仰 `tilt_joint=10°`。只运行与当前 `joint_positions_use_radians` 参数匹配的一条；该参数决定 `positions` 的单位。`time_from_start.sec: 30` 在此节点中是请求超时秒数，不是匀速转动 30 秒。

`sensor_gimbal.launch.py` 默认加载 `stub_params.yaml`，其中 `joint_positions_use_radians=false`，输入单位为**度**：

```bash
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {header: {frame_id: "command_type=angle;calibration_type=none"}, joint_names: [pan_joint, tilt_joint], points: [{positions: [30.0, 10.0], time_from_start: {sec: 30}}]}}'
```

`inspection_with_hardware.launch.py` 将 `joint_positions_use_radians` 覆盖为 `true`，输入单位为**弧度**：

```bash
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {header: {frame_id: "command_type=angle;calibration_type=none"}, joint_names: [pan_joint, tilt_joint], points: [{positions: [0.5235987756, 0.1745329252], time_from_start: {sec: 30}}]}}'
```

### 拍照文件存储

默认配置中，云台节点的 `save_image_dir: captured_images` 是相对于 `workspace_root` 的路径，即 `<workspace_root>/captured_images/`；在本机 `/home/jazzy/task_ws` 工作区部署时通常为 `/home/jazzy/task_ws/captured_images/`。热成像节点单独使用 `save_pic_dir: captured_images`，在标准安装布局下也解析到工作区的 `captured_images/`。板端部署、参数覆盖或绝对路径配置会改变实际位置，应以运行参数和 action 返回的路径为准。

| 拍照方式 | 默认落盘文件 | 返回路径 |
| --- | --- | --- |
| `capture_media` 可见光，海康后端 | `<save_image_dir>/req_<时间戳>_task<task_id>/capture_raw_iter<N>.jpg` | `result.media_uris`，每张一项 |
| `capture_media` 双通道 `camera_type=both` | 可见光为同一请求目录下的 `capture_visible_raw_iter<N>.jpg`；红外文件见下一行 | `result.media_uris`，每组按可见光、红外顺序返回 |
| `capture_media` 红外 `camera_type=thermal`/`ir` | `<save_pic_dir>/thermal_temperature_task<task_id>_iter<N>_<毫秒时间戳>.jpg` | `result.media_uris`；需热成像监控已启动 |
| `capture_media` ROS 图像 topic 后端 | `<save_image_dir>/req_<时间戳>_task<task_id>/capture_raw_iter<N>.png` | `result.media_uris`，每张一项 |
| `center_gimbal_and_capture` | `<save_image_dir>/req_<时间戳>_task<task_id>/capture_raw_iter<N>.jpg`（海康）或 `.png`（ROS topic） | `result.image_uri`，仅最后一张 |

`<时间戳>` 为请求创建时的本地时间；`task_id=0` 时目录名不带 `_task0`。默认 `capture_media_photo_save_raw=true`；若设为 `false`，单通道可见光或 ROS topic 拍照只校验抓拍而不保留照片，`media_uris` 为空。双通道仍保存可见光和红外；红外快照仍由热成像节点保存。以上路径只指照片，录像另有保存配置。

源码入口：`src/action/actions.cpp`、`src/action/follow.cpp`、`src/action/capture.cpp`、`src/action/capture_execute.cpp`、`src/action/center_capture.cpp`、`src/camera/hk_manual_topics.cpp`、`src/camera/hk_reboot.cpp`、`src/camera/hk_state_services.cpp`、`src/thermal/hik_alarm_node.cpp`、`src/smoke/hik_smoke_node.cpp`、`src/post_waypoint_home_bridge.cpp`。
