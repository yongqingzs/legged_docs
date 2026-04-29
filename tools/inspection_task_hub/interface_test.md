## base
```bash
# StartRoute default
ros2 service call /start_route inspection_task_hub/srv/StartRoute "{route_config_path: ''}"

# CancelRoute
ros2 service call /cancel_route inspection_task_hub/srv/CancelRoute "{}"

# GetStatus
ros2 service call /get_status inspection_task_hub/srv/GetStatus

# PauseRoute
ros2 service call /pause_route inspection_task_hub/srv/PauseRoute

# ResumeRoute
ros2 service call /resume_route inspection_task_hub/srv/ResumeRoute
```

## 功能测试
- start_route:
1. route 地址可以通过 /start_route 发送，并存在默认地址
```bash
jazzy@juice-Legion-Y9000P-IAH7H:~/nav_ws$ ros2 service call /start_route inspection_task_hub/srv/StartRoute "{}"
requester: making request: inspection_task_hub.srv.StartRoute_Request(route_config_path='')

response:
inspection_task_hub.srv.StartRoute_Response(success=True, message='Route started successfully')
```
2. 请求时无论是否执行成功均返回相关信息
3. route 执行过程中，必须通过 cancel_route 停止当前 route 后，才能执行新 route
4. 反复请求时 service 执行正确

- cancel_route:
1. 停止并消除当前 route
```bash
jazzy@juice-Legion-Y9000P-IAH7H:~/nav_ws$ ros2 service call /cancel_route inspection_task_hub/srv/CancelRoute "{}"
requester: making request: inspection_task_hub.srv.CancelRoute_Request()

response:
inspection_task_hub.srv.CancelRoute_Response(success=True, message='Route cancelled successfully')
```

- get_status:
1. 获取当前 route 的执行状态(TODO: 精简 status 的输出)
```bash
jazzy@juice-Legion-Y9000P-IAH7H:~/nav_ws$ ros2 service call /get_status inspection_task_hub/srv/GetStatus 
requester: making request: inspection_task_hub.srv.GetStatus_Request()

response:
inspection_task_hub.srv.GetStatus_Response(route_status=inspection_task_hub.msg.RouteStatus(route_name='', state=3, current_waypoint_index=3, total_waypoints=3, progress_percentage=100.0, error_message='', start_time=builtin_interfaces.msg.Time(sec=0, nanosec=0), last_update_time=builtin_interfaces.msg.Time(sec=0, nanosec=0)), current_waypoint=inspection_task_hub.msg.WaypointStatus(waypoint_id=0, waypoint_name='', position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), state=0, current_task_index=0, total_tasks=0, error_message='', arrival_time=builtin_interfaces.msg.Time(sec=0, nanosec=0)), pending_tasks=[])
```
2. 可在 route 运行时获得

- pause_route/resume_route
1. 和 resume 结合使用，可实现 pause + resume
```bash
[INFO] [1774419043.603302498] [task_hub_node]: Received pause_route request
[INFO] [1774419043.603346999] [task_hub_node]: [INFO] Route paused
[INFO] [1774419044.260232516] [task_hub_node]: [INFO] Photo result task_id=101, success=true
[INFO] [1774419044.260295827] [task_hub_node]: Task completed: Task 101 completed: Photo task completed successfully
[INFO] [1774419050.308747469] [task_hub_node]: Received resume_route request
[INFO] [1774419050.308856081] [task_hub_node]: [INFO] Route resumed
[INFO] [1774419050.473116202] [task_hub_node]: [INFO] Starting navigation to waypoint 2 (attempt 1/1)
[INFO] [1774419051.474324520] [task_hub_node]: [INFO] Waypoint 2 reached - starting tasks
[INFO] [1774419051.474747014] [task_hub_node]: Task started: Task 201 (photo) started, retry=0/0
[INFO] [1774419051.474786793] [task_hub_node]: Published gimbal control for task 201
[INFO] [1774419054.475723898] [task_hub_node]: [INFO] Photo result task_id=201, success=true
[INFO] [1774419054.475895069] [task_hub_node]: Task completed: Task 201 completed: Photo task completed successfully
[INFO] [1774419054.580832797] [task_hub_node]: [INFO] Starting navigation to waypoint 3 (attempt 1/1)
[INFO] [1774419055.582583299] [task_hub_node]: [INFO] Waypoint 3 reached - starting tasks
[INFO] [1774419055.582845045] [task_hub_node]: Task started: Task 301 (photo) started, retry=0/0
[INFO] [1774419055.582928228] [task_hub_node]: Published gimbal control for task 301
[INFO] [1774419058.583748316] [task_hub_node]: [INFO] Photo result task_id=301, success=true
[INFO] [1774419058.583914498] [task_hub_node]: Task completed: Task 301 completed: Photo task completed successfully
[INFO] [1774419058.689994140] [task_hub_node]: [INFO] All waypoints completed successfully
```