# HardwareBridge 类分析

本文档旨在深入分析 `Cheetah-Software/robot/include/HardwareBridge.h` 和 `HardwareBridge.cpp` 文件中的各类，阐述其工作原理、功能及各成员函数的实现。

## 1. 整体功能概述

`HardwareBridge` 系列类是 Cheetah-Software 框架中机器人代码与硬件之间的接口层。它负责初始化硬件、处理通信、管理任务调度，并为控制器提供访问硬件的统一接口。框架支持两种机器人：Mini Cheetah（使用 SPI 通信）和 Cheetah 3（使用 EtherCAT 通信）。类的核心功能包括：

- **硬件初始化**：设置栈、调度器、LCM 通信和传感器（如 IMU、遥控器）。
- **任务管理**：使用周期性任务管理器运行控制循环、SPI/EtherCAT 通信、可视化和日志记录。
- **通信处理**：通过 LCM 处理游戏手柄命令和控制参数请求，支持动态参数调整。
- **安全机制**：提供紧急停止和参数验证，确保机器人安全运行。
- **多线程支持**：使用线程处理 LCM 消息，避免阻塞主控制循环。

该系列类是硬件抽象层，确保控制器代码与具体硬件解耦，支持仿真和真实部署。

## 2. HardwareBridge 基类

### 2.1. 功能概述
`HardwareBridge` 是所有硬件桥的基类，提供通用功能，如 LCM 处理、任务调度和参数管理。

### 2.2. 成员函数详细功能

- **`HardwareBridge(RobotController* robot_ctrl)`**: 构造函数。初始化 LCM 实例、任务管理器和控制器指针，获取用户控制参数。
- **`~HardwareBridge()`**: 析构函数。删除 `RobotRunner` 实例。
- **`prefaultStack()`**: 预分配栈空间。写入 16KB 缓冲区，锁定内存页，防止页面错误和内存交换，提高实时性。
- **`setupScheduler()`**: 设置实时调度器。将进程优先级设置为 FIFO 调度策略，确保低延迟。
- **`initError(const char* reason, bool printErrno)`**: 初始化错误处理。打印错误消息并退出程序。
- **`initCommon()`**: 通用初始化。预分配栈、设置调度器、订阅 LCM 主题（游戏手柄和参数请求），启动 LCM 处理线程。
- **`handleGamepadLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const gamepad_lcmt* msg)`**: LCM 游戏手柄消息处理器。解析消息并设置游戏手柄命令。
- **`handleInterfaceLCM()`**: LCM 接口处理循环。持续处理传入的 LCM 消息。
- **`handleControlParameter(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const control_parameter_request_lcmt* msg)`**: 控制参数处理器。处理参数设置请求，支持用户和机器人参数的动态调整，包括类型检查和响应发布。
- **`publishVisualizationLCM()`**: 发布可视化数据。通过 LCM 发送机器人状态可视化信息。
- **`run_sbus()`**: 运行 SBUS 遥控器。接收 SBUS 数据包并处理完成。

## 3. MiniCheetahHardwareBridge 类

### 3.1. 功能概述
`MiniCheetahHardwareBridge` 继承自 `HardwareBridge`，专门为 Mini Cheetah 机器人定制。使用 SPI 通信与电机板交互，支持 Microstrain IMU 和 VectorNav IMU。

### 3.2. 成员函数详细功能

- **`MiniCheetahHardwareBridge(RobotController* robot_ctrl, bool load_parameters_from_file)`**: 构造函数。初始化 SPI 和 Microstrain LCM 实例，设置参数加载标志。
- **`run()`**: 主运行函数。初始化通用硬件、加载参数、创建 `RobotRunner`、启动任务（SPI、Microstrain、控制器、可视化、SBUS、日志）。
- **`initHardware()`**: 初始化 Mini Cheetah 特定硬件。初始化 VectorNav 或 Microstrain IMU 和 SPI。
- **`runSpi()`**: 运行 SPI 通信。发送命令、运行驱动、接收数据，并通过 LCM 发布 SPI 数据。
- **`runMicrostrain()`**: 运行 Microstrain IMU。持续读取 IMU 数据并更新 VectorNav 数据结构。
- **`logMicrostrain()`**: 日志 Microstrain 数据。通过 LCM 发布 IMU 数据。
- **`abort(const std::string& reason)` 和 `abort(const char* reason)`**: 中止函数。打印原因并退出（未实现）。

## 4. Cheetah3HardwareBridge 类

### 4.1. 功能概述
`Cheetah3HardwareBridge` 继承自 `HardwareBridge`，专门为 Cheetah 3 机器人定制。使用 EtherCAT 通信与电机板交互。

### 4.2. 成员函数详细功能

- **`Cheetah3HardwareBridge(RobotController* rc)`**: 构造函数。初始化 EtherCAT LCM 实例。
- **`run()`**: 主运行函数。初始化通用硬件、加载参数、创建 `RobotRunner`、启动任务（EtherCAT、控制器、可视化）。
- **`initHardware()`**: 初始化 Cheetah 3 特定硬件。初始化 VectorNav IMU。
- **`runEcat()`**: 运行 EtherCAT 通信。设置命令、运行 EtherCAT、获取数据，并发布 LCM 数据。
- **`publishEcatLCM()`**: 发布 EtherCAT 数据。通过 LCM 发送命令和数据消息。

## 5. 补充说明

- **通信协议**：Mini Cheetah 使用 SPI 和 LCM，Cheetah 3 使用 EtherCAT 和 LCM，支持高带宽、低延迟通信。
- **IMU 支持**：Mini Cheetah 支持 VectorNav 和 Microstrain IMU，Cheetah 3 仅 VectorNav。
- **任务调度**：使用 `PeriodicTaskManager` 管理周期性任务，确保实时性能。
- **参数管理**：支持从文件或 LCM 加载参数，便于调试和部署。
- **安全与鲁棒性**：包括栈锁定、实时调度和参数验证，防止系统崩溃。
- **扩展性**：基类设计允许轻松添加新机器人类型。

这份文档全面地分析了 `HardwareBridge` 系列类的架构和技术细节。
