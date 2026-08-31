# D1 Max SDK 示例与连接失败诊断

本文记录 RobotSDK-0.2.1 已编译示例的用途和 `control` 例程的实际操作方式；文末保留 RobotSDK-0.1.1 连接失败诊断作为历史背景。

## 0. RobotSDK-0.2.1 `control` 快速使用

新 SDK 目录：

```text
/home/jazzy/drive_ws/src/RobotSDK-0.2.1/example/build
```

在 x86_64 开发机上运行已编译例程：

```bash
cd /home/jazzy/drive_ws/src/RobotSDK-0.2.1/example/build
./control 192.168.234.1 8082
```

程序会异步连接机器人，等待连接状态变为 `CONNECTED`，打印系统版本和帮助，然后读取终端输入。它不会自动替你获取 SDK 控制权；如果当前控制源是 APP，应先关闭 APP 或在程序中按 `t` 获取控制权。确认机器狗处于安全、可运动区域后再发送运动键。

退出按 `q` 或 `Q`，也可按 `Ctrl+C`。退出前程序会断开 SDK 连接。键盘输入通常需要按回车才会被终端交给程序；若希望单键即时响应，需要修改终端为 raw mode，当前示例源码本身使用 `std::getchar()`，没有实现 raw mode。按空格停止时也需要再按 Enter，因此实机操作建议用 `p` + Enter 停止。

建议首次控制按以下顺序操作：

```text
o + Enter     查看状态和 Control Source
t + Enter     获取 SDK 控制权，等待 Take Control Success
z + Enter     站立
4 + Enter     设置低速档
w + Enter     短暂前进
p + Enter     明确发送停止
c + Enter     测试结束后卧倒（确认环境安全后执行）
y + Enter     释放控制权
q + Enter     退出
```

### `control` 按键映射

大小写有区别：动作表中的小写键才会执行命令，`Q/H/O/T/Y` 等帮助文字中的大写写法实际也对应源码注册的小写键（程序对退出 `Q/q` 特殊处理）。建议按下表的小写键；`N` 是特意保留的大写键。

| 按键 | 作用 | SDK 调用/说明 |
| --- | --- | --- |
| `w` / `s` | 前进 / 后退 | `Move(0, +0.11, 0)` / `Move(0, -0.11, 0)` |
| `a` / `d` | 左移 / 右移 | `Move(+0.1, 0, 0)` / `Move(-0.1, 0, 0)` |
| `l` / `r` | 左转 / 右转 | `Move(0, 0, +0.1)` / `Move(0, 0, -0.1)` |
| ` `（空格）或 `p` | 停止运动、转动和头部控制 | 发送零 `Move`、`Turn(0)`、`ControlHead(0,0)` |
| `4` / `5` / `6` | 低 / 中 / 高速度档 | `SetSpeed(1/2/3)` |
| `1` | 平衡站立 | `BalanceStandUp()` |
| `2` | 匍匐行走 | `CrawlWalk()` |
| `3` | 登阶模式 | `Stair()` |
| `z` / `x` / `c` | 站立 / 匍匐 / 卧倒 | `StandUp()` / `Crawl()` / `LieDown()` |
| `g` / `j` / `k` | Gait / Climb / Slim | 对应姿态或运动模式命令 |
| `u` / `/` / `;` | PosControl / SkWalk / Sand | 对应专用运动模式 |
| `+` / `-` | PosControl 高度上升 / 下降 | `PosMove(z=+0.2/-0.2)` |
| `7` / `8` | 左滚 / 右滚 | `Turn(1/2)`，用于平衡站立操作 |
| `9` / `0` | 头部向左看 / 抬头 | `ControlHead(0.5,0)` / `ControlHead(0,0.5)` |
| `t` / `y` | 获取 / 释放控制权 | `TakeControl()` / `ReleaseControl()` |
| `f` / `b` / `n` | 前灯 / 后灯 / 自动模式灯开关 | 每次按键切换状态 |
| `N` | LED 自动/手动模式切换 | `SetLedAutoMode()` |
| `=` | 查询 LED 自动/手动模式 | `GetLedAutoMode()` |
| `{` / `}` | 全部 LED 橙色闪烁 / 熄灭 | `SetLedCommand(BLINK/OFF)` |
| `i` | 避障开关 | `ObstacleAvoidance()`，每次切换 |
| `[` / `]` | 前 / 后相机拍照 | `TakePhoto(FRONT/BACK)` |
| `e` | 软件急停开关 | `SoftEmergencyStop()`，每次切换；解除急停前确认安全 |
| `m` | 锁定 | `Locked()` |
| `v` | 头尾方向反转 | `ReverseHeadTail()` |
| `,` / `.` | 设置 / 查询 M1 12V 外设电源 | `SetPeriphPower()` / `GetPeriphPower()` |
| `o` / `h` | 打印状态 / 打印帮助 | 输出当前缓存的 `RobotState` / 帮助 |
| `q` 或 `Q` | 退出 | 停止主循环并断开连接 |

移动键每次只发送一个归一化速度命令（平移约 `0.1`、前后约 `0.11`、偏航约 `0.1`）。SDK 文档说明最新 `Move` 指令维持 1 秒；示例没有松键检测，也不会在松键时发送停止命令。连续运动需要在 1 秒内重复输入，停止请明确按 `p` + Enter。方向正负和具体动作应先在低速、空旷区域实机确认。

0.2.1 还提供 `led`、`recharge` 等新增示例；它们分别用于 LED 控制和充电/回桩任务，不应与 `control` 同时运行争抢控制权。

## 1. RobotSDK-0.2.1 已编译示例

目录：

```text
/home/jazzy/drive_ws/src/RobotSDK-0.2.1/example/build
```

| 可执行文件 | 功能 | 是否会控制机器人 |
| --- | --- | --- |
| `data` | 传感器/状态订阅演示：IMU、光照、运动里程、速度、关节状态、机器人状态和故障；同时演示打开/关闭各类数据上报配置。 | 主要是数据订阅，会发送配置命令 |
| `control` | 交互式键盘控制：站立、卧倒、爬行、登阶、步态/模式、速度档、灯光、急停以及获取/释放控制权。 | 是 |
| `sync` | 同步（阻塞）连接及动作确认演示，依次执行站立、卧倒、再次站立和前后灯控制。 | 是 |
| `async` | 异步连接、回调和异步命令发送演示；示例中周期切换前后灯。 | 是 |
| `auto_reconnect` | 开启 SDK 内置自动重连，连接后周期发送零速度 `Move(0,0,0)`，打印连接状态和故障。 | 会发送零速度 |
| `manual_reconnect` | 应用层重连管理器：断线后由独立线程按间隔重试连接，并处理连接状态变化。 | 通常只发送零速度/重连 |
| `take_control` | 控制权演示：监视 APP/SDK/其他控制源，收到可用通知后重新调用 `TakeControl`，打印控制权应答。 | 会获取控制权 |
| `camera_bitrate` | 交互配置前/后摄像头码率（`f,<bps>`、`b,<bps>`），范围为 50000--100000000 bps。 | 会修改相机配置 |
| `led` | 独立 LED 自动模式、颜色/效果和持续时间控制示例。 | 会修改 LED 配置 |
| `recharge` | 充电、回桩/离桩任务控制和任务状态回调示例。 | 是，可能触发回桩动作 |

这些二进制由当前 x86_64 构建生成（`file` 显示为 `ELF 64-bit x86-64`），不能直接复制到 D1 Max 的 arm64 机器人上运行。应在 arm64 目标机重新编译，或使用 SDK 提供的 arm64 库和交叉编译工具链。

## 2. 端口和传输协议

SDK `SDKClient` 构造函数的默认传输协议是 `TransportProtocol::Udp`。官方文档规定：

| 传输协议 | 机器人端口 |
| --- | ---: |
| WebSocket | TCP `8081` |
| UDP（默认） | UDP `8082` |

因此未修改的 `data` 示例应运行：

```bash
./data 192.168.234.1 8082
```

命令 `./data 192.168.234.1 8081` 把默认 UDP 客户端指向了 WebSocket 端口，端口/协议组合错误。机器人端实测监听如下：

```text
tcp LISTEN 0 0 0.0.0.0:8081
udp UNCONN 0 0 0.0.0.0:8082
udp UNCONN 0 0 0.0.0.0:8083
```

## 3. RobotSDK-0.1.1 `ShakeHand failed` 的历史原因

仅改成 `8082` 后仍然失败，实测输出为：

```text
[INFO] SDK Version: 0.1.1
[INFO] Protocol Version: 1.2.0
[ERROR] ... json.exception.out_of_range.403 key 'charging_pile' not found
[ERROR] Connect failed: Robot ShakeHand failed
[ERROR] Not connected. State: 3
```

这说明网络路径是通的，机器人也返回了握手数据；失败发生在 SDK 对握手 JSON 的解码阶段。RobotSDK-0.1.1 的 `DecodeHandShake` 无条件读取 `charging_pile` 字段，而当前机器人返回的握手响应没有该字段，JSON 库因此抛出 `out_of_range.403`，SDK 将其统一报告为 `ShakeHandFailed`。

为排除端口因素，使用显式 `TransportProtocol::WebSocket` 的测试客户端连接 TCP `8081`，同样得到 `charging_pile not found`。所以根因不是 Wi-Fi、SSH 或机器人未启动，而是 SDK 与机器人软件版本/握手 schema 不匹配。

`State: 3` 对应 SDK 的 `ConnectionState::HANDSHAKING`：底层连接已建立，但握手解析未完成，状态没有机会转为 `CONNECTED`。

## 4. 当时的机器人版本证据

通过 SSH 查询 `/opt/release/version.yaml` 得到：

```yaml
version: 0.3.2
packages:
  robot-ws: v0.4.5
  motion-control: v0.6.3
  Robots Dog Msgs: v0.8.2
```

SDK 文档同时列出 `ProtocolMismatch`（常见于操作旧版本机器），并要求 SDK 与机器人软件版本匹配。当前现象属于更具体的握手字段不兼容，SDK 可能尚未把该解析异常映射成 `ProtocolMismatch`。

## 5. 新旧 SDK 兼容性结论

1. RobotSDK-0.1.1/协议 1.2.0 对该机器人握手字段不兼容，不再用于当前 D1 Max 适配。
2. 当前改用 RobotSDK-0.2.1/协议 1.3.0，并继续使用默认 UDP `8082`。
3. 0.2.1 自带版本表只明确记录软件平台 `0.3.1` 与 SDK `0.2.1` 完全兼容；机器人当前报告 `0.3.2`，未出现在这份表中。已知实际可用可以继续联调，但新增的避障、LED、Sand、回充等能力仍应逐项验证，不能仅凭基础连接成功认定全部接口兼容。
4. 运行控制示例前关闭 APP 或释放 APP 控制权；连接成功不等于已经取得运动控制权。
5. 建议按 `data -> control 中 o/t/低速移动/p -> 其他专用能力` 的顺序验证，始终先验证停止和急停路径。

## 6. 可复现检查命令

```bash
ssh robot@192.168.234.1
cat /opt/release/version.yaml
ss -lntu | grep -E ':808[123]\\b'

# 在 RobotSDK-0.2.1/example/build 目录
./data 192.168.234.1 8082
./control 192.168.234.1 8082
```
