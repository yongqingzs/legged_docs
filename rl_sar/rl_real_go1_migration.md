# `rl_real_go1` 从 `unitree_legged_sdk` 到 `free_dog_sdk_cpp` 的迁移说明

## 概述

本文档说明了如何将 `rl_real_go1` 模块从 `unitree_legged_sdk` 迁移到 `free_dog_sdk_cpp`。由于 Go1 和 A1 使用相同的硬件接口，但 `free_dog_sdk_cpp` 提供了更现代化和模块化的通信架构，此次迁移主要涉及通信层面的接口替换。

## 主要变更

### 1. SDK 库替换

#### 原接口 (`unitree_legged_sdk`)
```cpp
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"

UNITREE_LEGGED_SDK::Safety unitree_safe;
UNITREE_LEGGED_SDK::UDP unitree_udp;
UNITREE_LEGGED_SDK::LowCmd unitree_low_command = {0};
UNITREE_LEGGED_SDK::LowState unitree_low_state = {0};
xRockerBtnDataStruct unitree_joy;
```

#### 新接口 (`free_dog_sdk_cpp`)
```cpp
#include "fdsc_utils/free_dog_sdk_h.hpp"

std::shared_ptr<FDSC::UnitreeConnection> fdsc_conn;
FDSC::lowCmd fdsc_low_command;
FDSC::lowState fdsc_low_state;
std::vector<std::vector<uint8_t>> fdsc_data_buffer;
```

### 2. 初始化流程变更

#### 原始初始化
```cpp
RL_Real::RL_Real(int argc, char **argv) : unitree_udp(UNITREE_LEGGED_SDK::LOWLEVEL)
{
    // ...
    this->unitree_udp.InitCmdData(this->unitree_low_command);
    // ...
}
```

#### 新初始化流程
```cpp
RL_Real::RL_Real(int argc, char **argv)
{
    // ...
    // 初始化 free_dog_sdk 连接
    std::string connection_settings = "LOW_WIRED_DEFAULTS";  // 有线连接
    this->fdsc_conn = std::make_shared<FDSC::UnitreeConnection>(connection_settings);
    this->fdsc_conn->startRecv();
    
    // 发送初始命令建立连接
    std::vector<uint8_t> init_cmd = this->fdsc_low_command.buildCmd(false);
    this->fdsc_conn->send(init_cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // ...
}
```

**连接设置选项**:
- `"LOW_WIRED_DEFAULTS"`: 有线连接（推荐用于Go1）
- `"LOW_WIFI_DEFAULTS"`: WiFi连接
- `"HIGH_WIFI_DEFAULTS"`: 高层WiFi连接
- `"SIM_DEFAULTS"`: 仿真模式

### 3. UDP 通信接口变更

#### 原 UDP 发送/接收
```cpp
void UDPSend() { unitree_udp.Send(); }
void UDPRecv() { unitree_udp.Recv(); }
```

#### 新 UDP 发送/接收
```cpp
void RL_Real::UDPSend()
{
    std::vector<uint8_t> cmd_bytes = this->fdsc_low_command.buildCmd(false);
    this->fdsc_conn->send(cmd_bytes);
}

void RL_Real::UDPRecv()
{
    this->fdsc_conn->getData(this->fdsc_data_buffer);
}
```

### 4. 状态获取（GetState）变更

#### IMU 数据读取

**原方式**:
```cpp
state->imu.quaternion[0] = this->unitree_low_state.imu.quaternion[0]; // w
state->imu.quaternion[1] = this->unitree_low_state.imu.quaternion[1]; // x
state->imu.quaternion[2] = this->unitree_low_state.imu.quaternion[2]; // y
state->imu.quaternion[3] = this->unitree_low_state.imu.quaternion[3]; // z
state->imu.gyroscope[i] = this->unitree_low_state.imu.gyroscope[i];
```

**新方式**:
```cpp
// 从缓冲区获取最新数据并解析
if (!this->fdsc_data_buffer.empty())
{
    std::vector<uint8_t> latest_data = this->fdsc_data_buffer.back();
    this->fdsc_low_state.parseData(latest_data);
    this->fdsc_data_buffer.clear();
}

// IMU数据格式相同
state->imu.quaternion[0] = this->fdsc_low_state.imu_quaternion[0]; // w
state->imu.quaternion[1] = this->fdsc_low_state.imu_quaternion[1]; // x
state->imu.quaternion[2] = this->fdsc_low_state.imu_quaternion[2]; // y
state->imu.quaternion[3] = this->fdsc_low_state.imu_quaternion[3]; // z
state->imu.gyroscope[i] = this->fdsc_low_state.imu_gyroscope[i];
```

#### 电机状态读取

**原方式**:
```cpp
state->motor_state.q[i] = this->unitree_low_state.motorState[motor_idx].q;
state->motor_state.dq[i] = this->unitree_low_state.motorState[motor_idx].dq;
state->motor_state.tau_est[i] = this->unitree_low_state.motorState[motor_idx].tauEst;
```

**新方式** (相同):
```cpp
int motor_idx = this->params.Get<std::vector<int>>("joint_mapping")[i];
state->motor_state.q[i] = this->fdsc_low_state.motorState[motor_idx].q;
state->motor_state.dq[i] = this->fdsc_low_state.motorState[motor_idx].dq;
state->motor_state.tau_est[i] = this->fdsc_low_state.motorState[motor_idx].tauEst;
```

#### 手柄输入读取

**原方式**:
```cpp
memcpy(&this->unitree_joy, this->unitree_low_state.wirelessRemote, 40);
if (this->unitree_joy.btn.components.A) this->control.SetGamepad(Input::Gamepad::A);
this->control.x = this->unitree_joy.ly;
```

**新方式**:
```cpp
// free_dog_sdk 直接提供解析好的结构体
if (this->fdsc_low_state.wirelessdata.btn.components.A) 
    this->control.SetGamepad(Input::Gamepad::A);
this->control.x = this->fdsc_low_state.wirelessdata.ly;
```

### 5. 命令设置（SetCommand）变更

#### 原方式
```cpp
void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->unitree_low_command.motorCmd[motor_idx].mode = 0x0A;
        this->unitree_low_command.motorCmd[motor_idx].q = command->motor_command.q[i];
        this->unitree_low_command.motorCmd[motor_idx].dq = command->motor_command.dq[i];
        this->unitree_low_command.motorCmd[motor_idx].Kp = command->motor_command.kp[i];
        this->unitree_low_command.motorCmd[motor_idx].Kd = command->motor_command.kd[i];
        this->unitree_low_command.motorCmd[motor_idx].tau = command->motor_command.tau[i];
    }
    this->unitree_udp.SetSend(this->unitree_low_command);
}
```

#### 新方式
```cpp
void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        int motor_idx = this->params.Get<std::vector<int>>("joint_mapping")[i];
        std::string motor_name = std::to_string(motor_idx);
        
        // 准备电机命令数据: [q, dq, tau, Kp, Kd]
        std::vector<float> motor_data = {
            command->motor_command.q[i],
            command->motor_command.dq[i],
            command->motor_command.tau[i],
            command->motor_command.kp[i],
            command->motor_command.kd[i]
        };
        
        // 使用 free_dog_sdk 接口设置电机命令
        this->fdsc_low_command.motorCmd.setMotorCmd(
            motor_name, 
            FDSC::MotorModeLow::Servo, 
            motor_data
        );
    }
    // 命令发送在 UDPSend() 中完成
}
```

**电机模式枚举**:
- `FDSC::MotorModeLow::Servo`: 伺服模式（对应原 0x0A）
- `FDSC::MotorModeLow::Damping`: 阻尼模式
- 其他模式请参考 `fdsc_utils/common.hpp`

### 6. 析构函数变更

#### 新增连接关闭
```cpp
RL_Real::~RL_Real()
{
    // ... 关闭其他线程 ...
    
    // 停止 free_dog_sdk 连接
    if (this->fdsc_conn)
    {
        this->fdsc_conn->stopRecv();
    }
    
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}
```

### 7. 绘图功能变更（如果启用 PLOT）

#### 原方式
```cpp
this->plot_real_joint_pos[i].push_back(this->unitree_low_state.motorState[i].q);
this->plot_target_joint_pos[i].push_back(this->unitree_low_command.motorCmd[i].q);
```

#### 新方式
```cpp
int motor_idx = this->params.Get<std::vector<int>>("joint_mapping")[i];
this->plot_real_joint_pos[i].push_back(this->fdsc_low_state.motorState[motor_idx].q);
this->plot_target_joint_pos[i].push_back(this->fdsc_low_command.motorCmd.motors[motor_idx].q);
```

## 接口对照表

| 功能 | unitree_legged_sdk | free_dog_sdk_cpp |
|------|-------------------|------------------|
| 命令结构体 | `LowCmd` | `lowCmd` |
| 状态结构体 | `LowState` | `lowState` |
| 通信类 | `UDP` | `UnitreeConnection` |
| IMU四元数 | `imu.quaternion` | `imu_quaternion` |
| IMU陀螺仪 | `imu.gyroscope` | `imu_gyroscope` |
| 电机状态 | `motorState[i]` | `motorState[i]` (相同) |
| 手柄数据 | `wirelessRemote` (需手动解析) | `wirelessdata` (已解析) |
| 电机命令 | `motorCmd[i].q = ...` | `motorCmd.setMotorCmd(...)` |
| 发送命令 | `udp.SetSend()` | `conn->send(buildCmd())` |
| 接收数据 | `udp.GetRecv()` | `conn->getData()` |

## 优势与改进

### free_dog_sdk_cpp 的优势

1. **更清晰的数据流**:
   - 使用显式的 `buildCmd()` 和 `parseData()` 方法
   - 数据以字节流形式传输，便于调试和监控

2. **更灵活的连接配置**:
   - 支持多种连接模式（有线/WiFi/仿真）
   - 通过字符串参数轻松切换

3. **更好的手柄数据处理**:
   - 自动解析手柄数据到结构体
   - 无需手动 `memcpy`

4. **线程安全**:
   - 内置互斥锁保护数据
   - 使用 Boost.Asio 异步IO

5. **更模块化**:
   - 命令构建与发送分离
   - 便于添加中间处理步骤

### 需要注意的事项

1. **数据缓冲**:
   - `free_dog_sdk_cpp` 使用缓冲区存储接收的数据
   - 需要定期清空缓冲区以获取最新数据

2. **初始化延迟**:
   - 建立连接后需要等待一小段时间（100ms）
   - 确保初始命令正确发送

3. **电机索引**:
   - `setMotorCmd` 接受字符串或数字索引
   - 推荐使用数字字符串（如 `"0"`, `"1"`）

4. **错误处理**:
   - `parseData()` 会进行数据校验
   - 建议检查缓冲区是否为空

## 编译配置

确保在 `CMakeLists.txt` 中正确链接 `free_dog_sdk_cpp`:

```cmake
# 添加 free_dog_sdk_cpp 包含目录
include_directories(
    ${PROJECT_ROOT_DIR}/free_dog_sdk_cpp/fdsc_utils/include
)

# 链接 fdsc_utils 库
target_link_libraries(rl_real_go1
    fdsc_utils
    # ... 其他库 ...
)
```

## 测试建议

1. **连接测试**: 先测试基本的连接和数据接收
2. **手柄测试**: 验证手柄输入是否正确解析
3. **电机测试**: 从被动模式开始，逐步测试电机控制
4. **RL控制测试**: 最后测试完整的RL控制流程

## 总结

从 `unitree_legged_sdk` 迁移到 `free_dog_sdk_cpp` 主要是接口层面的改动，核心的控制逻辑和状态机保持不变。新SDK提供了更现代化的接口设计和更好的可维护性，推荐在新项目中使用。
