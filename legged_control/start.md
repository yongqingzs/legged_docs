# Legged Control 构建和启动文档
## build
clone
```bash
# ubuntu20.04
# ros: noetic
mkdir -p ~/noetic_ws/src 
cd ~/noetic_ws/src
git clone https://github.com/qiayuanl/legged_control.git

# Clone OCS2
git clone https://github.com/leggedrobotics/ocs2.git
# Clone pinocchio
git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
# Clone hpp-fcl
git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
# Clone ocs2_robotic_assets
git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
# Install dependencies
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
```

build ocs2
```bash
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization -j4

# simu
catkin build legged_controllers legged_unitree_description -j4
catkin build legged_gazebo -j4

# keyboard
sudo apt-get install ros-noetic-teleop-twist-keyboard
```

## start
初始化
```bash
# 这里我删去了环境变量的设置，以避免繁琐的设置过程
# terminal1
roslaunch legged_unitree_description empty_world.launch

# terminal2
roslaunch legged_controllers load_controller.launch cheater:=false

# terminal3
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']                   
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0" 
```

启动
```bash
# cmd_vel 先站起来
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.1
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10

# gait
# 切换为1
```

控制方式1
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

控制方式2
```bash
# 暂停
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10

# 前进
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10

# 转弯
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1" -r 10
```

demo:
![](resources/demo.gif)


迁移至go2：
- fork
- urdf、dae 来自 https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2_description
- 动力学模型通过 pinocchio 构建
- 未修改 mpc: Q/R ; wbc: kp/kd .. 等可调参数，控制律参数来自 go1
- demo:
![](resources/demo1.gif)