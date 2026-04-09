## base
启动前
```bash
pkill -f "ros2 launch bme_gazebo_basics spawn_robot.launch.py" || true
pkill -f mogi_trajectory_server || true
pkill -f "ros_gz_bridge/parameter_bridge" || true
pkill -f rviz2 || true
pkill -f "gz sim" || true
```

gz-sim livox 插件
```bash
https://github.com/RobotecAI/RGLGazeboPlugin
```