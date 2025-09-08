## leggedcontrol_go2 different code
- legged_controllers  
```txt
LeggedController.cpp
```

- legged_estimation  
感觉没有本质差别
```txt
LinearKalmanFilter.h
LinearKalmanFilter.cpp
```

- legged_gazebo
```txt
default.yaml
leggedHWSim.cpp
```

- legged_unitree_description mini
```txt
# add
meshes/go2

# change
# urdf
robot.xacro
common/imu.xacro
common/leg.xacro
go2/
```

未迁移
```txt
common/imu.xacro
common/leg.xacro
```
