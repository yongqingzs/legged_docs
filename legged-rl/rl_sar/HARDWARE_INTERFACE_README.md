# Hardware Interface Support for rl_real_go1

## 概述

`rl_real_go1` 现已支持两种硬件通信协议:
- **free_dog_sdk** (默认) - 直接UDP通信
- **unitree_ros2** - 通过ROS2消息通信 (需要DDS桥接)

## 快速开始

### 1. 使用 free_dog_sdk (默认)

编辑 `policy/go1/base.yaml`:
```yaml
go1:
  hardware_protocol: "free_dog_sdk"
```

编译和运行:
```bash
cd /home/jazzy/rl_sar
./build.sh
./build/rl_real_go1
```

### 2. 使用 unitree_ros2

#### 前置条件
- 已安装 `unitree_go` ROS2 消息包
- 运行 DDS-ROS2 桥接节点

#### 配置
编辑 `policy/go1/base.yaml`:
```yaml
go1:
  hardware_protocol: "unitree_ros2"
```

#### 编译
```bash
cd /home/jazzy/rl_sar
source /opt/ros/jazzy/setup.bash
source /home/jazzy/jazzy_ws/install/setup.bash  # 包含 unitree_go 消息
./build.sh
```

#### 运行

终端1 - 启动DDS桥接:
```bash
source /home/jazzy/jazzy_ws/install/setup.bash
ros2 launch hardware_unitree_ros2 dds_ros2_bridge.launch.py
```

终端2 - 运行RL控制器:
```bash
cd /home/jazzy/rl_sar
source /opt/ros/jazzy/setup.bash
source /home/jazzy/jazzy_ws/install/setup.bash
./build/rl_real_go1
```

## 架构说明

### 文件结构
```
src/rl_sar/
├── include/
│   ├── hardware_interface_base.hpp          # 抽象基类
│   ├── hardware_interface_free_dog_sdk.hpp  # free_dog_sdk适配器
│   ├── hardware_interface_unitree_ros2.hpp  # unitree_ros2适配器
│   └── rl_real_go1.hpp                      # 主程序头文件
├── src/
│   ├── hardware_interface_free_dog_sdk.cpp
│   ├── hardware_interface_unitree_ros2.cpp
│   └── rl_real_go1.cpp
└── CMakeLists.txt                           # 编译配置
```

### 通信流程

#### free_dog_sdk 模式:
```
rl_real_go1 <--UDP--> Robot (Go1)
```

#### unitree_ros2 模式:
```
rl_real_go1 <--ROS2 Topics--> DDS Bridge <--DDS--> Robot (Go1/Go2/B2)
```

## ROS2 话题

使用 `unitree_ros2` 时的话题:
- `/low_cmd` (unitree_go/msg/LowCmd) - 发布关节命令
- `/low_state` (unitree_go/msg/LowState) - 订阅机器人状态

## 兼容性

- ✅ 保持 `control_input_msgs` 支持
- ✅ 保持 `cmd_vel` (geometry_msgs/Twist) 支持
- ✅ 向后兼容原有 free_dog_sdk 代码
- ⚠️ 手柄按键解析仅在 `free_dog_sdk` 模式下可用

## 故障排除

### 编译错误: "unitree_go not found"
解决方案: 编译时会自动降级到 `free_dog_sdk` 模式，在运行时配置为 `"free_dog_sdk"` 即可

### 运行时错误: "Hardware interface failed to become ready"
检查:
1. free_dog_sdk 模式: 确认机器人网络连接 (默认 UDP)
2. unitree_ros2 模式: 确认 DDS bridge 节点正在运行

### 切换协议后无法连接
解决方案: 修改 `base.yaml` 后需要重新运行程序，无需重新编译

## 性能对比

| 特性 | free_dog_sdk | unitree_ros2 |
|-----|-------------|--------------|
| 延迟 | 低 (~0.5ms) | 中等 (~2-5ms) |
| 配置复杂度 | 低 | 中等 |
| 支持机器人 | Go1, A1 | Go1, Go2, B2, H1 |
| 需要额外节点 | 否 | 是 (DDS bridge) |
| 数据记录 | 需自行实现 | 支持 (可选) |

## 推荐使用场景

- **free_dog_sdk**: 低延迟控制、简单部署、Go1实机测试
- **unitree_ros2**: 统一接口、多机器人支持、需要ROS2生态工具
